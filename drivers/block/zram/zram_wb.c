/*
 * Write back zram to backing device
 *
 * Copyright (C) 2020 vivo Co., Ltd. All rights reserved.
 * Author: Jiewen Wang
 */

#define KMSG_COMPONENT "zram_wb"
#define pr_fmt(fmt) KMSG_COMPONENT ":<%s>: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/kobject.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/genhd.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/atomic.h>
#include <linux/freezer.h>

#include "zram_drv.h"
#include <linux/f2fs_fs.h>
#include "../../../fs/f2fs/f2fs.h"

// used of ram_max
#define DEFAULT_WMARK_HIGH 86
#define DEFAULT_WMARK_A 88
#define DEFAULT_WMARK_B 96

#define DEFAULT_ENABLED true

#define DEFAULT_SLEEP_MS 100
#define DEFAULT_PAGES_PER_TIME 2560 // 10MB

#define DEFAULT_HUGE_INTERVAL_SEC 300

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
#define DEFAULT_DSWAPPINESS_LOW 80
#define DEFAULT_DSWAPPINESS_HIGH 120
#define DEFAULT_DSWAPPINESS_MULTIPLE 2

#define HIGH_SWAPPINESS 1
#define LOW_SWAPPINESS 2
#endif

// reclaim level, corresponds to reclaim_level_str
#define RECLAIM_HIGH 0x0u
#define RECLAIM_READY 0x1u
#define RECLAIM_A 0x02u
#define RECLAIM_B 0x04u

#define RECLAIMABLE (RECLAIM_A|RECLAIM_B)
// reclaim huge first, if not enough then idle
#define RECLAIMABLE_A (RECLAIM_READY|RECLAIM_A)
// reclaim idle
#define RECLAIMABLE_B (RECLAIM_B)

// copy from zram_drv.c
#define HUGE_WRITEBACK 1
#define IDLE_WRITEBACK 2

// set zram_wb kthread to little core?
#if defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY) && defined(CONFIG_RSC_BINDER_OPTIMIZATION)
#define ZRAM_WBD_LITTLE_CORE
#endif

#ifdef ZRAM_WBD_LITTLE_CORE
extern struct cpumask rsc_cpu_littlecore; // depend on
#endif

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
#ifdef CONFIG_RSC_ZRAM_SWAPPINESS
extern int rsc_zram_swappiness;
#endif
extern int vm_swappiness;
#endif

// stats of system
atomic64_t zram_wb_kswapd_wakeup = ATOMIC64_INIT(0);
#ifdef CONFIG_RSC_MEM_MON
atomic64_t zram_wb_slowpath = ATOMIC64_INIT(0); // how many enter slowpath that cost over 10ms
#endif

static const char *reclaim_level_str[] = {
	"reclaim_high",
	"reclaim_ready",
	"reclaim_a",
	"reclaim_b",
};

static inline bool zram_wb_ready(struct zram *zram)
{
	return zram->disksize && zram->backing_dev;
}

static inline bool zram_wb_init_done(struct zram *zram)
{
	if (!zram->wb)
		return false;
	return true;
}

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
void zram_wb_update_swappiness(struct zram *zram)
{
	u64 used_size, base_size;
	unsigned int high_level;
	struct zram_wb *wb;

	if (!zram_wb_init_done(zram) || !zram->wb->dswappiness_enable)
		return;

	wb = zram->wb;
	base_size = zram->wb->ram_max;
	used_size = (u64)atomic64_read(&zram->stats.pages_stored) -
		(u64)atomic64_read(&zram->stats.bd_count);
	high_level = wb->wmark[WMARK_HIGH] -
		wb->dswappiness_multiple * (wb->wmark[WMARK_A] - wb->wmark[WMARK_HIGH]);

	if (high_level >= 100 || high_level <= 0) {
		pr_err("invalid high level %u", high_level);
		return;
	}

	if (used_size < base_size * high_level / 100 &&
			wb->dswappiness_location != HIGH_SWAPPINESS) {
		pr_info("up change rsc_swappiness from %d to %d",
				rsc_zram_swappiness, wb->dswappiness_high);
		wb->dswappiness_location = HIGH_SWAPPINESS;
#ifdef CONFIG_RSC_ZRAM_SWAPPINESS
		rsc_zram_swappiness = wb->dswappiness_high;
#else
		vm_swappiness = wb->dswappiness_high;
#endif
	} else if (used_size > base_size * wb->wmark[WMARK_A]/ 100 &&
			wb->dswappiness_location != LOW_SWAPPINESS) {
		pr_info("down change rsc_swappiness from %d to %d",
				rsc_zram_swappiness, wb->dswappiness_low);
		wb->dswappiness_location = LOW_SWAPPINESS;
#ifdef CONFIG_RSC_ZRAM_SWAPPINESS
		rsc_zram_swappiness = wb->dswappiness_low;
#else
		vm_swappiness = wb->dswappiness_low;
#endif
	}
}
#endif

static bool zram_wb_writebackable(struct zram *zram)
{
	u64 ram_stored, target, lru_pages, bd_free;

	spin_lock(&zram->wb_limit_lock);
	if (zram->wb_limit_enable && !zram->bd_wb_limit) {
		spin_unlock(&zram->wb_limit_lock);
		return false;
	}
	spin_unlock(&zram->wb_limit_lock);

	ram_stored = (u64)atomic64_read(&zram->stats.pages_stored) -
		(u64)atomic64_read(&zram->stats.bd_count);
	target = zram->wb->ram_max * zram->wb->wmark[WMARK_HIGH] / 100;
	lru_pages = (u64)atomic64_read(&zram->stats.lru_pages);
	if (ram_stored < target || lru_pages < ram_stored - target) {
#if 0
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
		pr_info("return false with ram_stored=%llu, target=%llu, lru_pages=%llu",
				ram_stored, target, lru_pages);
#endif
#endif
		return false;
	}

	bd_free = zram->nr_pages - (u64)atomic64_read(&zram->stats.bd_count);
	if (bd_free < zram->wb->bd_reclaim_min) {
#if 0
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
		pr_info("return false with bd_free=%llu, bd_reclaim_min=%llu",
				bd_free, zram->wb->bd_reclaim_min);
#endif
#endif
		return false;
	}

	return true;
}

static unsigned int zram_wb_get_reclaim_level(struct zram *zram)
{
	u64 used_size, base_size;
	int reclaim_level;

	if (!zram_wb_ready(zram) || !zram_wb_init_done(zram))
		return RECLAIM_HIGH;

	base_size = zram->wb->ram_max;
	used_size = (u64)atomic64_read(&zram->stats.pages_stored) -
		(u64)atomic64_read(&zram->stats.bd_count);

	if (used_size < base_size * zram->wb->wmark[WMARK_HIGH] / 100)
		reclaim_level = RECLAIM_HIGH;
	else if (used_size < base_size * zram->wb->wmark[WMARK_A] / 100)
		reclaim_level = RECLAIM_READY;
	else if (used_size < base_size * zram->wb->wmark[WMARK_B] / 100)
		reclaim_level = RECLAIM_A;
	else
		reclaim_level = RECLAIM_B;

	return reclaim_level;
}

static void zram_wb_bd_reclaim_min_refresh(struct zram *zram)
{
	struct zram_wb *wb = zram->wb;
	wb->bd_reclaim_min = wb->ram_max * (wb->wmark[WMARK_A] - wb->wmark[WMARK_HIGH]) / 100;
}

static void zram_wb_ram_max_refresh(struct zram *zram)
{
	zram->wb->ram_max = (zram->disksize >> PAGE_SHIFT) - zram->nr_pages;
	zram_wb_bd_reclaim_min_refresh(zram);
}

// debugfs is not available in ard11(cts limit). change to sysfs 
/**** sysfs begin ****/
static inline struct zram *subkobj_to_zram(struct kobject *kobj)
{
	return dev_to_zram(kobj_to_dev(kobj->parent));
}

static ssize_t wmark_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct zram *zram = subkobj_to_zram(kobj);
	struct zram_wb *wb = zram->wb;
	unsigned int wmark_high, wmark_a, wmark_b;

	if (sscanf(buf, "%u %u %u", &wmark_high, &wmark_a, &wmark_b) != WMARK_COUNT) {
		pr_err("set wmark err, not %d elements. buf %s", WMARK_COUNT, buf);
		return -EINVAL;
	}

	// when new wmark is 0, keep it as it is
	if (!wmark_high)
		wmark_high = wb->wmark[WMARK_HIGH];
	if (!wmark_a)
		wmark_a = wb->wmark[WMARK_A];
	if (!wmark_b)
		wmark_b = wb->wmark[WMARK_B];

	if (wmark_high >= 100 || wmark_a >= 100 || wmark_b >= 100 ||
			wmark_a <= wmark_high || wmark_b <= wmark_a) {
		pr_err("set wmark err, illegal value. buf %s", buf);
		return -EINVAL;
	}

	pr_info("set new wmark: high=%u->%u a=%u->%u b=%u->%u",
			wb->wmark[WMARK_HIGH], wmark_high,
			wb->wmark[WMARK_A], wmark_a,
			wb->wmark[WMARK_B], wmark_b);
	wb->wmark[WMARK_HIGH] = wmark_high;
	wb->wmark[WMARK_A] = wmark_a;
	wb->wmark[WMARK_B] = wmark_b;
	zram_wb_bd_reclaim_min_refresh(zram);

	return len;
}

static ssize_t wmark_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct zram *zram = subkobj_to_zram(kobj);
	struct zram_wb *wb = zram->wb;

	return scnprintf(buf, PAGE_SIZE, "%u %u %u\n",
			wb->wmark[WMARK_HIGH], wb->wmark[WMARK_A], wb->wmark[WMARK_B]);
}

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
static ssize_t dswappiness_high_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct zram *zram = subkobj_to_zram(kobj);
	unsigned long val;
	int ret = -EINVAL;

	if (kstrtoul(buf, 0, &val))
		return ret;

	if (val < 0 || val > 200 || val < zram->wb->dswappiness_low)
		return ret;

	zram->wb->dswappiness_high = val;
#ifdef CONFIG_RSC_ZRAM_SWAPPINESS
	rsc_zram_swappiness = val;
#endif
	ret = len;

	return ret;
}

static ssize_t dswappiness_low_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct zram *zram = subkobj_to_zram(kobj);
	unsigned long val;
	int ret = -EINVAL;

	if (kstrtoul(buf, 0, &val))
		return ret;

	if (val < 0 || val > 200 || val > zram->wb->dswappiness_high)
		return ret;

	zram->wb->dswappiness_low = val;
#ifdef CONFIG_RSC_ZRAM_SWAPPINESS
	vm_swappiness = val;
#endif
	ret = len;

	return ret;
}
#endif

#define ZRAM_WB_STAT_VERSION 2
#define ZRAM_WB_STAT_SIZE 20
#ifndef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
static ssize_t stat_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct zram *zram = subkobj_to_zram(kobj);
	struct zram_wb *wb = zram->wb;
	bool set_to_zero = false;
	int ret;

	ret = kstrtobool(buf, &set_to_zero);
	if (ret) {
		pr_err("invalid reset stat. buf %s, set_to_zero %d\n", buf, set_to_zero);
		return -EINVAL;
	}

	down_write(&zram->init_lock);
	atomic64_set(&wb->do_work_times, 0);
	atomic64_set(&zram_wb_kswapd_wakeup, 0);
	atomic64_set(&wb->stay_free[STAY1], 0);
	atomic64_set(&wb->stay_free[STAY2], 0);
	atomic64_set(&wb->stay_free[STAY3], 0);
	atomic64_set(&wb->stay_free[STAY_DEFAULT], 0);
	atomic64_set(&wb->stay_read[STAY1], 0);
	atomic64_set(&wb->stay_read[STAY2], 0);
	atomic64_set(&wb->stay_read[STAY3], 0);
	atomic64_set(&wb->stay_read[STAY_DEFAULT], 0);
	atomic64_set(&wb->bd_reads, 0);
	atomic64_set(&wb->bd_writes, 0);
	atomic64_set(&wb->bd_max_used, (u64)atomic64_read(&zram->stats.bd_count));
#ifdef CONFIG_RSC_MEM_MON
	atomic64_set(&zram_wb_slowpath, 0);
#endif
	up_write(&zram->init_lock);

	return len;
}
#else
static ssize_t stat_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	return len;
}
#endif

static void stat_show_val(char *buf, ssize_t *len, u64 num)
{
	*len += scnprintf(buf + *len, PAGE_SIZE - *len, "%llu ", num);
}

static ssize_t stat_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct zram *zram = subkobj_to_zram(kobj);
	struct zram_wb *wb = zram->wb;
	ssize_t len = 0;

	down_read(&zram->init_lock);
	stat_show_val(buf, &len, ZRAM_WB_STAT_VERSION);
	stat_show_val(buf, &len, ZRAM_WB_STAT_SIZE);
	// set to zero while echo 1 > /d/zram0_wb/stat begin
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->do_work_times));
	stat_show_val(buf, &len, (u64)atomic64_read(&zram_wb_kswapd_wakeup));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_free[STAY1]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_free[STAY2]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_free[STAY3]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_free[STAY_DEFAULT]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_read[STAY1]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_read[STAY2]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_read[STAY3]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->stay_read[STAY_DEFAULT]));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->bd_reads));
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->bd_writes));
	// set to zero while echo 1 > /d/zram0_wb/stat end
	stat_show_val(buf, &len, (u64)atomic64_read(&wb->bd_max_used));
	stat_show_val(buf, &len, zram->nr_pages);
	stat_show_val(buf, &len, (u64)atomic64_read(&zram->stats.pages_stored));
	stat_show_val(buf, &len, zs_get_total_pages(zram->mem_pool));
	stat_show_val(buf, &len, (u64)atomic64_read(&zram->stats.huge_pages));
	stat_show_val(buf, &len,
#ifdef CONFIG_RSC_MEM_MON
			(u64)atomic64_read(&zram_wb_slowpath)  // can reset to zero
#else
			0
#endif
			);
	stat_show_val(buf, &len, (u64)atomic64_read(&zram->stats.bd_count));
	stat_show_val(buf, &len, zram->disksize >> PAGE_SHIFT);
	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	up_read(&zram->init_lock);

	return len;
}

static void stat_readable_show_val(char *buf, ssize_t *len, const char *s, u64 size_of_pages)
{
	*len += scnprintf(buf + *len, PAGE_SIZE - *len, s);
	*len += scnprintf(buf + *len, PAGE_SIZE - *len, "%16llu B %8llu page\n",
			size_of_pages << PAGE_SHIFT, size_of_pages);
}

static ssize_t stat_readable_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct zram *zram = subkobj_to_zram(kobj);
	struct zram_wb *wb = zram->wb;
	ssize_t len = 0;
	u64 orig_data_size, orig_in_bd, orig_in_ram;

	down_read(&zram->init_lock);

	orig_data_size = (u64)atomic64_read(&zram->stats.pages_stored);
	orig_in_bd = (u64)atomic64_read(&zram->stats.bd_count);
	orig_in_ram = orig_data_size - orig_in_bd;

	len = scnprintf(buf + len, PAGE_SIZE - len,
#ifndef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
			"clean\n"
#else
			"not clean\n"
#endif
			"version:                  %16d\n"
			"wakeup_times(clean):      %16llu\n",
			ZRAM_WB_STAT_VERSION, (u64)atomic64_read(&wb->do_work_times));
	stat_readable_show_val(buf, &len, "disksize:             ",
			zram->disksize >> PAGE_SHIFT);
	stat_readable_show_val(buf, &len, "backing_dev_size:     ", zram->nr_pages);
	stat_readable_show_val(buf, &len, "ram_max_size:         ", wb->ram_max);
	stat_readable_show_val(buf, &len, "orig_stored:          ", orig_data_size);
	stat_readable_show_val(buf, &len, "orig_in_backing_dev:  ", orig_in_bd);
	stat_readable_show_val(buf, &len, "orig_in_ram:          ", orig_in_ram);
	stat_readable_show_val(buf, &len, "zspool_size:          ",
			zs_get_total_pages(zram->mem_pool));
	stat_readable_show_val(buf, &len, "mem_used_max:         ",
			(long)atomic_long_read(&zram->stats.max_used_pages));
	stat_readable_show_val(buf, &len, "bd_max_used(clean):   ",
			(u64)atomic64_read(&wb->bd_max_used));
#ifdef CONFIG_APP_LINK
	stat_readable_show_val(buf, &len, "zram_only_pages:      ",
			(u64)atomic64_read(&zram->stats.zram_only_pages));
	stat_readable_show_val(buf, &len, "zram_only_huge_pages: ",
			(u64)atomic64_read(&zram->stats.zram_only_huge_pages));
	stat_readable_show_val(buf, &len, "zram_only_same_pages: ",
			(u64)atomic64_read(&zram->stats.zram_only_same_pages));
#endif
	stat_readable_show_val(buf, &len, "same_pages:           ",
			(u64)atomic64_read(&zram->stats.same_pages));
	stat_readable_show_val(buf, &len, "huge_pages:           ",
			(u64)atomic64_read(&zram->stats.huge_pages));
	stat_readable_show_val(buf, &len, "bd_huge_pages:        ",
			(u64)atomic64_read(&zram->stats.bd_huge_pages));
	stat_readable_show_val(buf, &len, "zram_lru_pages:       ",
			(u64)atomic64_read(&zram->stats.lru_pages));
	stat_readable_show_val(buf, &len, "zram_lru_huge_pages:  ",
			(u64)atomic64_read(&zram->stats.lru_huge_pages));
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"stay_free(clean):     %16llu %8llu %8llu %8llu page\n",
			(u64)atomic64_read(&wb->stay_free[STAY1]),
			(u64)atomic64_read(&wb->stay_free[STAY2]),
			(u64)atomic64_read(&wb->stay_free[STAY3]),
			(u64)atomic64_read(&wb->stay_free[STAY_DEFAULT]));
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"stay_read(clean):     %16llu %8llu %8llu %8llu page\n",
			(u64)atomic64_read(&wb->stay_read[STAY1]),
			(u64)atomic64_read(&wb->stay_read[STAY2]),
			(u64)atomic64_read(&wb->stay_read[STAY3]),
			(u64)atomic64_read(&wb->stay_read[STAY_DEFAULT]));
	len += scnprintf(buf + len, PAGE_SIZE - len, "sched_exetime:        %16llu ns\n",
			wb->thread->se.sum_exec_runtime);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	len += scnprintf(buf + len, PAGE_SIZE - len, "reclaim_time:         %16llu ns\n",
			(u64)atomic64_read(&zram->wb->reclaim_time));
	len += scnprintf(buf + len, PAGE_SIZE - len, "reclaim_huge_time:    %16llu ns\n",
			(u64)atomic64_read(&zram->wb->reclaim_huge_time));
	len += scnprintf(buf + len, PAGE_SIZE - len, "reclaim_idle_time:    %16llu ns\n",
			(u64)atomic64_read(&zram->wb->reclaim_idle_time));
#endif
	spin_lock(&zram->wb_limit_lock);
	len += scnprintf(buf + len, PAGE_SIZE - len, "wb_limit_enable:      %16llu\n",
			zram->wb_limit_enable);
	len += scnprintf(buf + len, PAGE_SIZE - len, "wb_limit:             %16llu page\n",
			zram->bd_wb_limit);
	spin_unlock(&zram->wb_limit_lock);
	up_read(&zram->init_lock);

	return len;
}

static ssize_t bd_size_limit_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct zram *zram = subkobj_to_zram(kobj);

	return scnprintf(buf, PAGE_SIZE, "%d %d\n",
			zram->nr_pages, zram->wb->max_bd_nr_pages);
}

static ssize_t bd_size_limit_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	struct zram *zram = subkobj_to_zram(kobj);
	unsigned long bd_size;
	int ret;

	ret = kstrtoul(buf, 0, &bd_size);
	if (ret || bd_size > zram->wb->max_bd_nr_pages) {
		pr_err("invalid bd_size_limit. buf %s, bd_size %d, max %d\n",
				buf, bd_size, zram->wb->max_bd_nr_pages);
		return -EINVAL;
	}
	zram->nr_pages = bd_size;
	zram_wb_ram_max_refresh(zram);

	return len;
}

/* vivo linzhichi add for recording pids protect begin */
#ifdef CONFIG_APP_LINK
#define MAX_PID_NUM 10
#define PIDS_PROTECT_HASHSIZE 7
#define HASH_FUNC(pid) (pid % PIDS_PROTECT_HASHSIZE)

struct pids_protect_node {
	struct hlist_node link;
	pid_t pid;
};

static struct pids_protect_head {
	spinlock_t lock;
	struct hlist_head list;
} pids_protect_htable[PIDS_PROTECT_HASHSIZE];

static atomic_t pid_num = ATOMIC_INIT(0);
static bool __read_mostly init_complete = false;

bool is_protected_pid(pid_t pid)
{
	int idx = HASH_FUNC(pid);
	bool is_protected = false;
	struct pids_protect_node *pos = NULL;

	if (!init_complete) {
		// pr_err("pids protect haven't init complete\n");
		return false;
	}
	spin_lock(&pids_protect_htable[idx].lock);
	hlist_for_each_entry(pos, &pids_protect_htable[idx].list, link) {
		if (pid == pos->pid) {
			is_protected = true;
			break;
		}
	}
	spin_unlock(&pids_protect_htable[idx].lock);

	return is_protected;
}
EXPORT_SYMBOL(is_protected_pid);

static void init_pids_protect()
{
	int idx;

	for (idx = 0; idx < ARRAY_SIZE(pids_protect_htable); idx++) {
		INIT_HLIST_HEAD(&pids_protect_htable[idx].list);
		spin_lock_init(&pids_protect_htable[idx].lock);
	}

	init_complete = true;
	pr_info("pids_protect init complete\n");
}

static ssize_t pids_protect_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	pid_t pid;
	int err, idx;
	struct pids_protect_node *node = NULL;

	if (!atomic_add_unless(&pid_num, 1, MAX_PID_NUM)) {
		pr_err("pid_num is %d\n", atomic_read(&pid_num));
		return -ENOMEM;
	}
	err = kstrtoint(buf, 10, (int *)&pid);
	if (err < 0) {
		atomic_dec(&pid_num);
		pr_err("store failed, buf is %s\n", buf);
		return err;
	}

	if(is_protected_pid(pid)) {
		atomic_dec(&pid_num);
		pr_info("%d has been protected\n", pid);
		return len;
	}

	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		atomic_dec(&pid_num);
		pr_err("fail to allocte pids node\n");
		return -ENOMEM;
	}
	node->pid = pid;
	idx = HASH_FUNC(pid);

	spin_lock(&pids_protect_htable[idx].lock);
	hlist_add_head(&node->link, &pids_protect_htable[idx].list);
	spin_unlock(&pids_protect_htable[idx].lock);

	pr_info("store pid is %d, idx is %d\n", pid, idx);

	return len;
}

static ssize_t pids_protect_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int len = 0, idx;
	struct pids_protect_node *pos = NULL;

	for (idx = 0; idx < ARRAY_SIZE(pids_protect_htable); idx++) {
		spin_lock(&pids_protect_htable[idx].lock);
		hlist_for_each_entry(pos, &pids_protect_htable[idx].list, link) {
			len += sprintf(buf + len, "%d\n", pos->pid);
		}
		spin_unlock(&pids_protect_htable[idx].lock);
	}
	if (len == 0) {
		len = sprintf(buf, "don't have pid\n");
	}

	return len;
}

void exit_pid_protected(struct task_struct *task)
{
	pid_t pid = task->pid;
	int idx = HASH_FUNC(pid);
	struct pids_protect_node *pos = NULL;

	if (!init_complete) {
		// pr_err("pids protect exit haven't init complete\n");
		return;
	}
	spin_lock(&pids_protect_htable[idx].lock);
	hlist_for_each_entry(pos, &pids_protect_htable[idx].list, link) {
		if (pid == pos->pid) {
			break;
		}
	}
	if (pos) {
		atomic_dec(&pid_num);
		pr_info("delete %s, %d successfully\n", task->comm, pid);
		hlist_del_init(&pos->link);
	}
	spin_unlock(&pids_protect_htable[idx].lock);

	if (pos)
		kfree(pos);
	pos = NULL;
}
EXPORT_SYMBOL(exit_pid_protected);
#endif
/* vivo linzhichi add for recording pids protect end */

// please add _show or _store above here
#define zram_wb_ul_store(_variable)	\
static ssize_t _variable##_store(struct kobject *kobj,	\
	struct kobj_attribute *attr, const char *buf, size_t len)	\
{	\
	struct zram *zram = subkobj_to_zram(kobj);	\
	unsigned long val;	\
	int ret = -EINVAL;	\
	ret = kstrtoul(buf, 0, &val);	\
	if (!ret) {	\
		zram->wb->_variable = val;	\
		ret = len;	\
	}	\
	return ret;	\
}

#define zram_wb_ul_show(_variable)	\
static ssize_t _variable##_show(struct kobject *kobj,	\
		struct kobj_attribute *attr, char *buf)	\
{	\
	return scnprintf(buf, PAGE_SIZE, "%lu\n",	\
			subkobj_to_zram(kobj)->wb->_variable);	\
}

#define ZRAM_WB_UL_RW_FUNC(_variable)	\
zram_wb_ul_store(_variable);	\
zram_wb_ul_show(_variable);

ZRAM_WB_UL_RW_FUNC(huge_interval);
ZRAM_WB_UL_RW_FUNC(sleep_millisecs);
ZRAM_WB_UL_RW_FUNC(pages_per_time);
zram_wb_ul_show(bd_reclaim_min);
zram_wb_ul_show(ram_max);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
ZRAM_WB_UL_RW_FUNC(dswappiness_multiple);
zram_wb_ul_show(dswappiness_high);
zram_wb_ul_show(dswappiness_low);
#endif

#define zram_wb_bool_store(_variable)	\
static ssize_t _variable##_store(struct kobject *kobj,	\
	struct kobj_attribute *attr, const char *buf, size_t len)	\
{	\
	struct zram *zram = subkobj_to_zram(kobj);	\
	bool bv; \
	int ret = -EINVAL;	\
	ret = kstrtobool(buf, &bv); \
	if (!ret) {	\
		zram->wb->_variable = bv;	\
		ret = len;	\
	}	\
	return ret;	\
}

#define zram_wb_bool_show(_variable)	\
static ssize_t _variable##_show(struct kobject *kobj,	\
		struct kobj_attribute *attr, char *buf)	\
{	\
	struct zram *zram = subkobj_to_zram(kobj); \
	if (zram->wb->_variable) \
		return scnprintf(buf, 10, "%c\n", 'Y'); \
	else \
		return scnprintf(buf, 10, "%c\n", 'N'); \
}

#define ZRAM_WB_BOOL_RW_FUNC(_variable)	\
zram_wb_bool_store(_variable);	\
zram_wb_bool_show(_variable);

ZRAM_WB_BOOL_RW_FUNC(enable);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
ZRAM_WB_BOOL_RW_FUNC(dswappiness_enable);
#endif

#define ZRAM_WB_ATTR_RW(_name) \
	static struct kobj_attribute _name##_attr = \
	    __ATTR(_name, 0660, _name##_show, _name##_store)
#define ZRAM_WB_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = \
	    __ATTR(_name, 0440, _name##_show, NULL)

ZRAM_WB_ATTR_RW(enable);
ZRAM_WB_ATTR_RW(wmark);
ZRAM_WB_ATTR_RW(stat);
ZRAM_WB_ATTR_RO(stat_readable);
ZRAM_WB_ATTR_RW(huge_interval);
ZRAM_WB_ATTR_RW(sleep_millisecs);
ZRAM_WB_ATTR_RW(pages_per_time);
ZRAM_WB_ATTR_RW(bd_size_limit);
ZRAM_WB_ATTR_RO(bd_reclaim_min);
ZRAM_WB_ATTR_RO(ram_max);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
ZRAM_WB_ATTR_RW(dswappiness_enable);
ZRAM_WB_ATTR_RW(dswappiness_high);
ZRAM_WB_ATTR_RW(dswappiness_low);
ZRAM_WB_ATTR_RW(dswappiness_multiple);
#endif
/* vivo linzhichi add for recording pids protect begin */
#ifdef CONFIG_APP_LINK
ZRAM_WB_ATTR_RW(pids_protect);
#endif
/* vivo linzhichi add for recording pids protect end */

static const struct attribute *zram_wb_attrs[] = {
	&enable_attr.attr,
	&wmark_attr.attr,
	&stat_attr.attr,
	&stat_readable_attr.attr,
	&huge_interval_attr.attr,
	&sleep_millisecs_attr.attr,
	&pages_per_time_attr.attr,
	&bd_size_limit_attr.attr,
	&bd_reclaim_min_attr.attr,
	&ram_max_attr.attr,
#ifdef CONFIG_APP_LINK
	&pids_protect_attr.attr, // add by linzhichi for pids protect
#endif
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
	&dswappiness_enable_attr.attr,
	&dswappiness_high_attr.attr,
	&dswappiness_low_attr.attr,
	&dswappiness_multiple_attr.attr,
#endif
	NULL,
};

static int zram_wb_kobj_create(struct zram *zram)
{
	struct kobject *zram_wb_kobj = NULL;

	zram_wb_kobj = kobject_create_and_add("zram_wb", &disk_to_dev(zram->disk)->kobj);
	if (!zram_wb_kobj) {
		pr_err("create sub kobject zram_wb failed.\n");
		return -EBUSY;
	}

	sysfs_create_files(zram_wb_kobj, zram_wb_attrs);

	/* vivo linzhichi add for recording pids protect begin */
#ifdef CONFIG_APP_LINK
	init_pids_protect();
#endif
	/* vivo linzhichi add for recording pids protect end */
	zram->wb->kobj = zram_wb_kobj;
	return 0;
}

static void zram_wb_kobj_destroy(struct zram *zram)
{
	sysfs_remove_files(zram->wb->kobj, zram_wb_attrs);
	kobject_put(zram->wb->kobj);
}
/**** sysfs end ****/

static bool zram_wbd_should_run(struct zram *zram, unsigned int reclaimable_level)
{
	unsigned int reclaim_level = zram_wb_get_reclaim_level(zram);

	return zram->wb->enable &&
		(reclaim_level & reclaimable_level) &&
		zram_wb_writebackable(zram);
}

void wake_up_zram_wbd(struct zram *zram)
{
	if (zram_wb_init_done(zram) && zram_wbd_should_run(zram, RECLAIMABLE))
		wake_up_interruptible(&zram->wb->wait_queue);
}

// since no slot lock, slot maybe free after get index, please check the slot state
static int zram_last_lru_index(struct zram *zram)
{
	int index = -ENOSPC;
	spin_lock(&zram->lru_lock);
	if (!list_empty(&zram->lru))
		index = list_last_entry(&zram->lru, struct zram_table_entry, lru_node) - zram->table;
	spin_unlock(&zram->lru_lock);
	return index;
}

#define SEC_TO_NS(t) ((t) * 1000000000)
static int zram_last_huge_index(struct zram *zram)
{
	struct zram_table_entry *entry;
	int index = -ENOSPC;
	spin_lock(&zram->huge_lock);
	if (!list_empty(&zram->huge)) {
		entry = list_last_entry(&zram->huge, struct zram_table_entry, huge_node);
		if (entry->ac_time + SEC_TO_NS(zram->wb->huge_interval) < ktime_to_ns(ktime_get()))
			index = entry - zram->table;
	}
	spin_unlock(&zram->huge_lock);
	return index;
}

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
static void reclaim_time_update(struct zram *zram, u64 time, bool is_reclaim_huge)
{
	atomic64_add(time, &zram->wb->reclaim_time);
	if (is_reclaim_huge) {
		atomic64_add(time, &zram->wb->reclaim_huge_time);
	} else {
		atomic64_add(time, &zram->wb->reclaim_idle_time);
	}
}
#endif

// return means can loop this again
static bool zram_wb_reclaim(struct zram *zram, int wb_mode)
{
	bool ret = true;
	int _ret;
	int index;
	unsigned long nr_pages = zram->disksize >> PAGE_SHIFT;
	unsigned long pages_per_time = zram->wb->pages_per_time;
	unsigned long blk_idx = 0;
	unsigned long once_write = 0;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	u64 t0, t1;
#endif

	if (wb_mode != HUGE_WRITEBACK && wb_mode != IDLE_WRITEBACK) {
		pr_err("err wb_mode %d\n", wb_mode);
		return false;
	}

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	//t0 = local_clock();
	t0 = ktime_get_ns();
#endif
	down_read(&zram->init_lock);
	while (once_write < pages_per_time) {
		spin_lock(&zram->wb_limit_lock);
		if (zram->wb_limit_enable && !zram->bd_wb_limit) {
			spin_unlock(&zram->wb_limit_lock);
			ret = false;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
			pr_info("stop by wb limit, enable=%d, limit=%d, mode=%d, ret=%d",
					zram->wb_limit_enable, zram->bd_wb_limit, wb_mode, ret);
#endif
			break;
		}
		spin_unlock(&zram->wb_limit_lock);

		if (!blk_idx) {
			blk_idx = alloc_block_bdev(zram);
			if (!blk_idx) {
				ret = false;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
				pr_info("stop cannot alloc blkidx, blk_idx=%d, mode=%d, ret=%d",
						blk_idx, wb_mode, ret);
#endif
				break;
			}
		}

		index = (wb_mode == HUGE_WRITEBACK) ?
			zram_last_huge_index(zram) : zram_last_lru_index(zram);
		if (index < 0 || index >= nr_pages) {
			ret = false;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
			pr_info("error index=%d(0-%d), mode=%d, ret=%d",
					index, nr_pages, wb_mode, ret);
#endif
			break;
		}

		_ret = zram_wb_write_slot(zram, index, wb_mode, zram->wb->t_page, blk_idx); 
		if (!_ret) {
			blk_idx = 0;
			once_write += 1;
		} else {
			ret = false;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
			pr_info("write slot failed, index=%d, blkidx=%d, mode=%d, ret=%d, _ret=%d",
					index, blk_idx, wb_mode, ret, _ret);
#endif
			break;
		}
	}
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	//t1 = local_clock();
	t1 = ktime_get_ns();
	if (t1 > t0)
		reclaim_time_update(zram, t1 - t0, wb_mode == HUGE_WRITEBACK);
	else
		pr_err("error time t1=%llu <= t0=%llu", t1, t0);
	pr_info("reclaim mode=%d ret=%d write=%d(page) time=%lluns\n",
			wb_mode, ret, once_write, t1 - t0);
#endif

	if (blk_idx)
		free_block_bdev(zram, blk_idx);
	up_read(&zram->init_lock);

	return ret;
}

static void zram_wb_reclaim_a(struct zram *zram)
{
	int wb_mode = HUGE_WRITEBACK;
	while (zram_wbd_should_run(zram, RECLAIMABLE_A)) {
		msleep_interruptible(zram->wb->sleep_millisecs);
		if (!zram_wb_reclaim(zram, wb_mode)) {
			if (wb_mode == HUGE_WRITEBACK)
				wb_mode = IDLE_WRITEBACK;
			else
				break;
		}
	}
}

static void zram_wb_reclaim_b(struct zram *zram)
{
	while (zram_wbd_should_run(zram, RECLAIMABLE_B)) {
		msleep_interruptible(zram->wb->sleep_millisecs);
		if (!zram_wb_reclaim(zram, IDLE_WRITEBACK))
			break;
	}
}

static void zram_wb_pr_reclaim_stat(struct zram *zram)
{
	struct zram_wb *wb = zram->wb;
	u64 total_stored, bd_stored, ram_stored;
	int lv = zram_wb_get_reclaim_level(zram);
	if (lv == RECLAIM_B)
		lv = 3;

	total_stored = (u64)atomic64_read(&zram->stats.pages_stored);
	bd_stored = (u64)atomic64_read(&zram->stats.bd_count);
	ram_stored = total_stored - bd_stored;

	down_read(&zram->init_lock);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	pr_info("not clean(page)");
#else
	pr_info("clean(page)");
#endif
	pr_cont(",lv=%s,wakeup_c=%llu",
			reclaim_level_str[lv], (u64)atomic64_read(&wb->do_work_times));
	pr_cont(",orig=%llu/%llu/%llu(h)/%llu(s)",
			zram->disksize >> PAGE_SHIFT, total_stored,
			(u64)atomic64_read(&zram->stats.huge_pages),
			(u64)atomic64_read(&zram->stats.same_pages));
	pr_cont(",orig_ram=%llu/%llu", wb->ram_max, ram_stored);
	pr_cont(",orig_bd=%llu/%llu/%llu(h_c)", zram->nr_pages, bd_stored,
			(u64)atomic64_read(&zram->stats.bd_huge_pages));
	pr_cont(",compr=%llu/%llu/%lu(max)",
			zs_get_total_pages(zram->mem_pool) << PAGE_SHIFT,
			(u64)atomic64_read(&zram->stats.compr_data_size),
			(long)atomic_long_read(&zram->stats.max_used_pages) << PAGE_SHIFT);
	pr_cont(",lru=%llu/%llu(h)",
			(u64)atomic64_read(&zram->stats.lru_pages),
			(u64)atomic64_read(&zram->stats.lru_huge_pages));
	pr_cont(",stay_free_c=%llu/%llu/%llu/%llu,stay_read_c=%llu/%llu/%llu/%llu",
			(u64)atomic64_read(&wb->stay_free[STAY1]),
			(u64)atomic64_read(&wb->stay_free[STAY2]),
			(u64)atomic64_read(&wb->stay_free[STAY3]),
			(u64)atomic64_read(&wb->stay_free[STAY_DEFAULT]),
			(u64)atomic64_read(&wb->stay_read[STAY1]),
			(u64)atomic64_read(&wb->stay_read[STAY2]),
			(u64)atomic64_read(&wb->stay_read[STAY3]),
			(u64)atomic64_read(&wb->stay_read[STAY_DEFAULT]));
#ifdef CONFIG_APP_LINK
	pr_cont(",zram_only=%llu/%llu(s)/%llu(h)",
			(u64)atomic64_read(&zram->stats.zram_only_pages),
			(u64)atomic64_read(&zram->stats.zram_only_huge_pages),
			(u64)atomic64_read(&zram->stats.zram_only_same_pages));
#endif
	pr_cont(",exe_time=%lluns", current->se.sum_exec_runtime);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	pr_cont("/%llu/%llu(h)/%llu(i)",
			(u64)atomic64_read(&zram->wb->reclaim_time),
			(u64)atomic64_read(&zram->wb->reclaim_huge_time),
			(u64)atomic64_read(&zram->wb->reclaim_idle_time));
#endif
	spin_lock(&zram->wb_limit_lock);
	pr_cont(",wb_limit_%s=%d", zram->wb_limit_enable ? "enable" : "close",
			zram->bd_wb_limit);
	spin_unlock(&zram->wb_limit_lock);
	pr_cont("\n");
	up_read(&zram->init_lock);
}

static void zram_wb_pr_pinfile_stat(struct zram *zram)
{
	struct inode *inode = zram->backing_dev->f_mapping->host;

	pr_info("inode=%lu, is_pinfile=%d, f2fs_flags=%lx",
			inode->i_ino, f2fs_is_pinned_file(inode), F2FS_I(inode)->flags);
}

static void zram_wb_do_work(struct zram *zram)
{
	pr_info("%s_wbd do work begin\n", zram->disk->disk_name);
	zram_wb_pr_pinfile_stat(zram);
	zram_wb_pr_reclaim_stat(zram);
	while (zram_wbd_should_run(zram, RECLAIMABLE)) {
		if (zram_wbd_should_run(zram, RECLAIMABLE_A))
			zram_wb_reclaim_a(zram);
		else if (zram_wbd_should_run(zram, RECLAIMABLE_B))
			zram_wb_reclaim_b(zram);
		else
			pr_info("no need to reclaim\n");
		zram_wb_pr_reclaim_stat(zram);
	}
	atomic64_inc(&zram->wb->do_work_times);
	pr_info("%s_wbd do work end\n", zram->disk->disk_name);
}

static int zram_wb_work_thread(void *z)
{
	struct zram *zram = (struct zram *)z;
#ifdef ZRAM_WBD_LITTLE_CORE
	const struct cpumask *cpumask = &rsc_cpu_littlecore;
	struct task_struct *tsk = current;

	if (!cpumask_empty(cpumask)) {
		set_cpus_allowed_ptr(tsk, cpumask);
		pr_info("set cpu affinity to littlecore: %*pbl",
				cpumask_pr_args(&rsc_cpu_littlecore));
	}
#endif

	set_freezable();
	set_user_nice(current, 10);

	while (!kthread_should_stop()) {
		try_to_freeze();

		if (zram_wbd_should_run(zram, RECLAIMABLE))
			zram_wb_do_work(zram);

		wait_event_freezable(zram->wb->wait_queue,
				zram_wbd_should_run(zram, RECLAIMABLE) || kthread_should_stop());
	}

	return 0;
}

static int zram_wb_run(struct zram *zram)
{
	struct task_struct *wbd;

	wbd = kthread_run(zram_wb_work_thread, zram, "%s_wbd", zram->disk->disk_name);
	if (IS_ERR(wbd)) {
		zram->wb->thread = NULL;
		return PTR_ERR(wbd);
	}
	zram->wb->thread = wbd;

	return 0;
}

static void zram_wb_stop(struct zram *zram)
{
	if (zram->wb->thread) {
		kthread_stop(zram->wb->thread);
		zram->wb->thread = NULL;
	}
}

static int zram_wb_init(struct zram *zram)
{
	struct zram_wb *wb;

	wb = kzalloc(sizeof(struct zram_wb), GFP_KERNEL);
	if (!wb)
		return -ENOMEM;

	wb->t_page = alloc_page(GFP_KERNEL);
	if (!wb->t_page) {
		pr_err("alloc_page failed\n");
		kfree(wb);
		return -ENOMEM;
	}

	zram->wb = wb;

	init_waitqueue_head(&wb->wait_queue);

	wb->enable = DEFAULT_ENABLED;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
	wb->dswappiness_enable = DEFAULT_ENABLED;
	wb->dswappiness_high = DEFAULT_DSWAPPINESS_HIGH;
	wb->dswappiness_low = DEFAULT_DSWAPPINESS_LOW;
	wb->dswappiness_multiple = DEFAULT_DSWAPPINESS_MULTIPLE;
	wb->dswappiness_location = HIGH_SWAPPINESS;
#endif

	wb->wmark[WMARK_HIGH] = DEFAULT_WMARK_HIGH;
	wb->wmark[WMARK_A] = DEFAULT_WMARK_A;
	wb->wmark[WMARK_B] = DEFAULT_WMARK_B;

	wb->huge_interval = DEFAULT_HUGE_INTERVAL_SEC;

	wb->sleep_millisecs = DEFAULT_SLEEP_MS;
	wb->pages_per_time = DEFAULT_PAGES_PER_TIME;

	zram_wb_ram_max_refresh(zram);

	wb->max_bd_nr_pages = zram->nr_pages;

	return 0;
}

static void zram_wb_destroy(struct zram *zram)
{
	if (zram->wb) {
		__free_page(zram->wb->t_page);
		kfree(zram->wb);
		zram->wb = NULL;
	}
}

// need down write zram->init_lock
void zram_wb_register(struct zram *zram)
{
	int ret;
	if (!zram_wb_ready(zram) || zram->wb)
		return;

	ret = zram_wb_init(zram);
	if (ret) {
		pr_err("Failed to init zram_wb %d.\n", ret);
		return;
	}

	ret = zram_wb_run(zram);
	if (ret) {
		pr_err("Failed to start zram_wbd %d.\n", ret);
		zram_wb_destroy(zram);
		return;
	}

	ret = zram_wb_kobj_create(zram);
	if (ret) {
		pr_err("Failed to create zram_wb controler %d.\n", ret);
		zram_wb_stop(zram);
		zram_wb_destroy(zram);
	}

	pr_info("%s writeback thread created!\n", zram->disk->disk_name);
}

// need down write zram->init_lock
void zram_wb_unregister(struct zram *zram)
{
	if (!zram_wb_init_done(zram))
		return;

	zram_wb_kobj_destroy(zram);
	zram_wb_stop(zram);
	zram_wb_destroy(zram);

	pr_info("%s writeback thread destroied!\n", zram->disk->disk_name);
}

