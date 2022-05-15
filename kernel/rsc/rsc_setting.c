/*
 * kernel/rsc/rsc_cpu_top.c
 *
 * VIVO Resource Control.
 *
 * fast feed back task cpu usage, no need to scan /proc dir
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */
#include <linux/atomic.h>
#include <linux/err.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/profile.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_cpu_internal.h>
#if defined(CONFIG_RSC_VAUDIT) && defined(CONFIG_RSC_APP_LAUNCH_BOOST)
#include <linux/vivo_rsc/rsc_vaudit.h>
#endif
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG)
#include <linux/cpuset.h>
#endif
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
#include <linux/cpuset.h>
#include "../../kernel/sched/sched.h"
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
#include <linux/vivo_rsc/rsc_test_svp.h>
#endif

static DEFINE_MUTEX(rsc_svp_lock);
#define SVP_PID_HASH_BITS	11

static DECLARE_HASHTABLE(hash_table, SVP_PID_HASH_BITS);
static LIST_HEAD(tpid_entry_head);
static LIST_HEAD(tpid_free_head);

struct rsc_svp_entry {
	struct hlist_node hash;
	struct list_head link;
	u32		tv_sec;/* seconds */
	pid_t pid;
};

#define RSC_MAX_SVP_PID 64
static struct rsc_svp_entry svp_tentry[RSC_MAX_SVP_PID];
#if defined(CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK) || defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
atomic_t rsc_total_svp_pid;
int __read_mostly rsc_svp_set_count;
#else
static atomic_t rsc_total_svp_pid;
#endif

static struct rsc_svp_entry *rsc_find_svp_pid_entry(pid_t pid)
{
	struct rsc_svp_entry *tpid_entry;
	hash_for_each_possible(hash_table, tpid_entry, hash, pid) {
		if (tpid_entry->pid == pid)
			return tpid_entry;
	}
	return NULL;
}

static struct rsc_svp_entry *rsc_find_or_register_svp_pid(pid_t pid,
	struct task_struct *task)
{
	struct rsc_svp_entry *tpid_entry;
	int count;

	tpid_entry = rsc_find_svp_pid_entry(pid);
	if (tpid_entry)
		return tpid_entry;

	count = atomic_inc_return(&rsc_total_svp_pid);
	if (count <= RSC_MAX_SVP_PID) {
		tpid_entry = list_first_entry_or_null(&tpid_free_head,
			struct rsc_svp_entry, link);
		if (!tpid_entry) {
			rsc_err("%s: no free buf. pid: %d rsc_total_svp_pid: %d\n", __func__, pid,  atomic_read(&rsc_total_svp_pid));
			return NULL;
		}
		list_move_tail(&tpid_entry->link, &tpid_entry_head);
	} else {
		atomic_dec(&rsc_total_svp_pid);
		rsc_info("%s: no free buf. pid: %d. setting pid more than rsc_total_svp_pid: %d\n",
			__func__, pid, atomic_read(&rsc_total_svp_pid));
		return NULL;
	}

#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
	rsc_svp_task_uid = task_uid(task).val;
#endif

	tpid_entry->tv_sec = (u32)RSC_JIFF_TO_S(get_jiffies_64());
	tpid_entry->pid = pid;
	hash_add(hash_table, &tpid_entry->hash, pid);

	return tpid_entry;
}


static struct rsc_svp_entry *rsc_del_svp_pid(pid_t pid,
	struct task_struct *task)
{
	struct rsc_svp_entry *tpid_entry;

	tpid_entry = rsc_find_svp_pid_entry(pid);
	if (!tpid_entry)
		return NULL;

	hash_del(&tpid_entry->hash);
	list_move_tail(&tpid_entry->link, &tpid_free_head);
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
	if (atomic_dec_and_test(&rsc_total_svp_pid))
		rsc_svp_task_uid = -1;

	if ((atomic_read(&rsc_total_svp_pid) < HEAVYTASK_LIMIT_SVP_MIN_CNT) || !(atomic_read(&rsc_total_svp_pid)))
		rsc_svp_set_count = atomic_read(&rsc_total_svp_pid);
#else
	atomic_dec(&rsc_total_svp_pid);
#endif

	if (atomic_read(&rsc_total_svp_pid) < 0) {
		rsc_err("%s: error! pid: %d rsc_total_svp_pid: %d\n",
			__func__, pid, atomic_read(&rsc_total_svp_pid));
		return NULL;
	}

	return tpid_entry;
}

static ssize_t show_rsc_svp(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	struct rsc_svp_entry *tpid_entry;
	int cnt = 0;
	u32 cursec;
	u32 elaps;
	struct task_struct *p, *gl;
	BUILD_BUG_ON(NR_CPUS > 16);

	cursec = (u32)RSC_JIFF_TO_S(get_jiffies_64());
	mutex_lock(&rsc_svp_lock);
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "total: %d"
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
			"\t%d"
#endif
			"\n", atomic_read(&rsc_total_svp_pid)
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
			, rsc_svp_set_count
#endif
		);
	ret += cur;

	list_for_each_entry(tpid_entry, &tpid_entry_head, link) {

		if (cursec >= tpid_entry->tv_sec)
			elaps = cursec - tpid_entry->tv_sec;
		else
			elaps = (U32_MAX - tpid_entry->tv_sec) + cursec;

		rcu_read_lock();
		p = find_task_by_vpid(tpid_entry->pid);
		if (p)
			gl = p->group_leader;
		else
			gl = NULL;
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%2d\t%16s\t%5d\t%16s\t%5d\t%x\telap\t%u\n",
			cnt,
			p?p->comm:"NULL", tpid_entry->pid,
			gl?gl->comm:"NULL", gl?gl->pid:99999,
			p?p->rsc_svp:0xE,
			elaps
		);
		ret += cur;
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		cnt++;
	}

	if (atomic_read(&rsc_total_svp_pid) != cnt) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "list error! %d != %d\n",
		atomic_read(&rsc_total_svp_pid), cnt);
		ret += cur;
	}
	mutex_unlock(&rsc_svp_lock);

	return ret;
}

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
extern struct task_struct *rsc_get_hok_render_thread(struct task_struct *p);
struct rsc_svp_workaround_map {
	int orgpid;
	int tarpid;
} rsc_svp_map;
int rsc_svp_map_cnt;
#define RSC_SVP_BINDER_OPT 1
#define RSC_SVP_RUNNABLE_OPT 2

#define RSC_SVP_ENQUEUE_ENABLE 0x8000

int rsc_svp_enable = RSC_SVP_BINDER_OPT|RSC_SVP_RUNNABLE_OPT;
int rsc_svp_workaround;

int do_svp_limit_process_num = 2;
int do_svp_limit_process_num_backup = 2;
char do_svp_limit_process_list_str[RSC_HEAVYTASK_LIMIT_MAX_PROCESS][RSC_MAX_PACKAGE_NAME] = {
	"com.tencent.tmgp.sgame",
	"com.tencent.tmgp.sgamece"
};

char *do_svp_limit_process_list[RSC_HEAVYTASK_LIMIT_MAX_PROCESS] = {
	do_svp_limit_process_list_str[0],
	do_svp_limit_process_list_str[1],
	do_svp_limit_process_list_str[2],
	do_svp_limit_process_list_str[3],
	do_svp_limit_process_list_str[4],
	do_svp_limit_process_list_str[5],
	do_svp_limit_process_list_str[6],
	do_svp_limit_process_list_str[7]
};

int do_svp_limit_sched_setaffinity_num = 2;
int do_svp_limit_sched_setaffinity_num_backup = 2;
char do_svp_limit_sched_setaffinity_list_str[RSC_SET_AFFINITY_LIMIT_MAX_PROCESS][RSC_MAX_PACKAGE_NAME] = {
	"com.tencent.tmgp.sgame",
	"com.tencent.tmgp.sgamece"
};
char *do_svp_limit_sched_setaffinity_list[RSC_SET_AFFINITY_LIMIT_MAX_PROCESS] = {
	do_svp_limit_sched_setaffinity_list_str[0],
	do_svp_limit_sched_setaffinity_list_str[1],
	do_svp_limit_sched_setaffinity_list_str[2],
	do_svp_limit_sched_setaffinity_list_str[3],
	do_svp_limit_sched_setaffinity_list_str[4],
	do_svp_limit_sched_setaffinity_list_str[5],
	do_svp_limit_sched_setaffinity_list_str[6],
	do_svp_limit_sched_setaffinity_list_str[7]
};

#define SVP_ENABLE_MAINTHREAD_FAST_SCHED 1
#define SVP_ENABLE_MAINTHREAD_BIND_BIGCORE 2
u32 svp_enable_mainthread = SVP_ENABLE_MAINTHREAD_FAST_SCHED/*|SVP_ENABLE_MAINTHREAD_BIND_BIGCORE*/;
#endif

static ssize_t store_rsc_svp(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct task_struct *p;
	struct rsc_svp_entry *tpid_entry;
	int pid, val;
	int ret;
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
	int do_conver_orgpid = 0;
#endif

	BUILD_BUG_ON(RSC_FEAT_NR > RSC_SWITCH_CLEAR);

	ret = sscanf(buf, "%d %d", &pid, &val);
	if ((ret != 2) || val < 0)
		return -EINVAL;

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
	if (val && !rsc_svp_enable) {
		rsc_info("rsc_svp disable val %d\n", val);
		return -EINVAL;
	}
#endif

	mutex_lock(&rsc_svp_lock);
	rcu_read_lock();

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
	if (!val && rsc_svp_map_cnt) {
		if (pid == rsc_svp_map.orgpid) {
			do_conver_orgpid = pid;
			pid = rsc_svp_map.tarpid;
			rsc_svp_map_cnt--;
			rsc_svp_map.orgpid = 0;
			rsc_svp_map.tarpid = 0;
		}
	}
#endif

	p = find_task_by_vpid(pid);
	if (!p) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		rsc_info("%s could not find pid %d!\n",
			__func__, pid);
		mutex_unlock(&rsc_svp_lock);
		return -EINVAL;
	}

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
	/* force to set the render thread, because there are bugs in game watch!!! */
	if (val && !rsc_svp_map_cnt && rsc_svp_workaround) {
		if (strncmp("UnityMain", p->comm, TASK_COMM_LEN) && strncmp("Thread-", p->comm, 7)) {
			p = rsc_get_hok_render_thread(p);
			if (!p) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
				rcu_read_unlock_inline();
#else
				rcu_read_unlock();
#endif
				rsc_info("%s could not find render pid %d!\n",
					__func__, pid);
				mutex_unlock(&rsc_svp_lock);
				return -EINVAL;
			}
			rsc_svp_map.orgpid = pid;
			rsc_svp_map.tarpid = p->pid;
			pid = p->pid;
			rsc_svp_map_cnt++;
		}
	} else {
		if (do_conver_orgpid) {
			rsc_info("get hok_render uid %5d %16s %5d! orgpid %d val %d mapcnt: %d\n",
				task_uid(p).val, p->comm, p->pid, do_conver_orgpid, val, rsc_svp_map_cnt);
		}
	}
#endif

	get_task_struct(p);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif

	if (val) {
		tpid_entry = rsc_find_or_register_svp_pid(pid, p);
		if (tpid_entry) {
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
			int val = 0;

			/*p->rsc_svp |= RSC_SVP_MANUAL;*/
			/*
			if (rsc_svp_enable & RSC_SVP_BINDER_OPT)
				val |= RSC_SVP_BINDER_PROMOTE_ONLY;
			*/
			if (rsc_svp_enable & RSC_SVP_RUNNABLE_OPT)
				val |= RSC_SVP_MANUAL;

			set_rsc_svp(p, val);
			/*clear_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK);*/
			{
				char app_name[RSC_MAX_PACKAGE_NAME];
				int res;

				res = get_cmdline(p, app_name, sizeof(app_name));
				if (res > 0) {
					if (res < sizeof(app_name))
						app_name[res] = 0;
					else
						app_name[sizeof(app_name)-1] = 0;
					if (rsc_should_do_action(app_name, do_svp_limit_process_list, do_svp_limit_process_num)/*!strncmp(app_name, "com.tencent.tmgp.sgame", strlen("com.tencent.tmgp.sgame"))*/) {
						set_rsc_svp(p->group_leader, RSC_SVP_MANUAL_GROUP_LEADER);
						rsc_svp_set_count = atomic_read(&rsc_total_svp_pid);
					}
				}
			}
#else
			p->rsc_svp = val;
#endif
		} else
			count = -ENOSPC;
	} else {
		rsc_del_svp_pid(pid, p);
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
		/*p->rsc_svp &= ~RSC_SVP_MANUAL;*/
		clear_rsc_svp(p, RSC_SVP_MANUAL/*|RSC_SVP_BINDER_PROMOTE_ONLY*/);
		if ((atomic_read(&rsc_total_svp_pid) < HEAVYTASK_LIMIT_SVP_MIN_CNT) || !(atomic_read(&rsc_total_svp_pid)))
			rsc_svp_set_count = atomic_read(&rsc_total_svp_pid);
#else
		p->rsc_svp = val;
#endif
	}

#if defined(CONFIG_RSC_VAUDIT) || defined(CONFIG_RSC_APP_LAUNCH_BOOST)
	if (val) {
		if (rsc_vaudit_svp_thread == RSC_VAUDIT_SVP_THREAD_ALL)
			p->mainthread |= RSC_VAUDIT_SVP_MAINTHREAD;
		else if ((val == 60) && (rsc_vaudit_svp_thread & RSC_VAUDIT_SVP_THREAD_60FPS_ONLY))
			p->mainthread |= RSC_VAUDIT_SVP_MAINTHREAD;
		else if ((val == 40) && (rsc_vaudit_svp_thread & RSC_VAUDIT_SVP_THREAD_40FPS_ONLY))
			p->mainthread |= RSC_VAUDIT_SVP_MAINTHREAD;
		else if ((val == 30) && (rsc_vaudit_svp_thread & RSC_VAUDIT_SVP_THREAD_30FPS_ONLY))
			p->mainthread |= RSC_VAUDIT_SVP_MAINTHREAD;
		else if ((val == 25) && (rsc_vaudit_svp_thread & RSC_VAUDIT_SVP_THREAD_25FPS_ONLY))
			p->mainthread |= RSC_VAUDIT_SVP_MAINTHREAD;
	} else
		p->mainthread &= ~RSC_VAUDIT_SVP_MAINTHREAD;

	/*reset svp task schedwait time*/
	task_thread_info(p)->rsc_schedwait_start = 0;
	task_thread_info(p)->rsc_schedwait = 0;
#endif

	put_task_struct(p);
	mutex_unlock(&rsc_svp_lock);
	return count;
}

static int rsc_svp_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
{
	struct task_struct *task = v;

	if (!task || (atomic_read(&rsc_total_svp_pid) <= 0))
		return NOTIFY_OK;

	mutex_lock(&rsc_svp_lock);
	rsc_del_svp_pid(task->pid, task);
	mutex_unlock(&rsc_svp_lock);
	return NOTIFY_OK;
}

#define RSC_SVP rsc_svp

static struct kobj_attribute rsc_svp_attr =
__ATTR(RSC_SVP, 0660, show_rsc_svp, store_rsc_svp);

static struct notifier_block rsc_svp_notifier_block = {
	.notifier_call	= rsc_svp_notifier,
};

static DEFINE_MUTEX(rsc_switch_lock);

#undef RSC_FEAT
#define RSC_FEAT(name, enabled)														\
/*	(																				\
	do {	(*/ 																	\
	(1UL << RSC_FEAT_##name) * enabled |											\
/*		) } while (0)																\
	) */

unsigned int __read_mostly sysctl_rsc_switch = 
#include <linux/vivo_rsc/rsc_features.h>
	0;

#undef RSC_FEAT
#define RSC_FEAT(name, enabled)	\
/*	(																				\
		do {	(*/ 																\
	#name,																			\
/*		) } while (0)																\
	) */

static const char * const rsc_feat_names[] = {
#include <linux/vivo_rsc/rsc_features.h>
};
#undef RSC_FEAT

static ssize_t show_rsc_switch(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	int i;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "0x%x\n", sysctl_rsc_switch);
	ret += cur;

	for (i = 0; i < RSC_FEAT_NR; i++) {
		if (!(sysctl_rsc_switch & (1UL << i))) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "NO_");
			ret += cur;
		}
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%s %lx\t", rsc_feat_names[i], (1UL << i));
		ret += cur;
	}

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\nexample, set or clear bit1:\n"
								"setval: echo        0x1 > /sys/rsc/rsc_switch\n"
								" clear: echo 0x80000001 > /sys/rsc/rsc_switch\n");
	ret += cur;

	return ret;
}

static ssize_t store_rsc_switch(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int value;
	unsigned int oldval;
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
	bool set_surfaceflinger = false;
#endif

#if 1
	/* use kstrtouint instead sscanf to prevent buffer overfollow. */
	ret = kstrtouint(buf, 16, &value);
	if (ret != 0) {
			rsc_err("set rsc_switch %x val: %x error! ret: %d buf: %s\n",
				sysctl_rsc_switch, value, ret, buf);
		return ret;
	}
#else
	ret = sscanf(buf, "%x", &value);
	if (ret != 1)
		return -EINVAL;
#endif

	mutex_lock(&rsc_switch_lock);
	oldval = sysctl_rsc_switch;
	if (value & (1 << RSC_SWITCH_CLEAR))
		sysctl_rsc_switch &= ~value;
	else {
		if ((rsc_val_feats(value, BINDER_LIMIT_NORMAL, BINDER_LIMIT_AGGRESS) |
			rsc_feats(BINDER_LIMIT_NORMAL, BINDER_LIMIT_AGGRESS)) ==
			((1UL << RSC_FEAT_BINDER_LIMIT_NORMAL) | (1UL << RSC_FEAT_BINDER_LIMIT_AGGRESS))) {
			rsc_err("set rsc_switch %x val: %x BINDER_LIMIT_NORMAL %lx and BINDER_LIMIT_AGGRESS %lx should be mutual exclusion."
				"echo 0x80000005 > /sys/rsc/rsc_switch then echo 0x4 > /sys/rsc/rsc_switch\n",
				sysctl_rsc_switch, value, (1UL << RSC_FEAT_BINDER_LIMIT_NORMAL), (1UL << RSC_FEAT_BINDER_LIMIT_AGGRESS));
			mutex_unlock(&rsc_switch_lock);
			return -EINVAL;
		}
		sysctl_rsc_switch |= value;
	}

#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
	if (rsc_val_feat(value, RSC_LOCK_BOOST_FUTEX_ENABLE)) {
		if (!(value & (1 << RSC_SWITCH_CLEAR)) && rsc_surfaceflinger_pid) {
			struct task_struct *m;
			u16 pid = rsc_surfaceflinger_pid;

			rcu_read_lock();
			m = find_task_by_vpid(pid);
			if (m && !test_dynamic_svp_nocheck(m, DYNAMIC_SVP_FORCE)) {
				dynamic_svp_inc_extern(m, DYNAMIC_SVP_FORCE);
				set_surfaceflinger = true;
			}
			rcu_read_unlock();
		}
	}
#endif

	rsc_info("%s rsc_switch %x -> %x val: %x surface%d\n", (value & (1 << RSC_SWITCH_CLEAR))?"clear":"set",
		oldval, sysctl_rsc_switch, value
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
		, set_surfaceflinger
#endif
	);

	mutex_unlock(&rsc_switch_lock);

	return count;	
}

static struct kobj_attribute rsc_switch_attr =
__ATTR(rsc_switch, 0660, show_rsc_switch, store_rsc_switch);

#ifdef CONFIG_RSC_NEW_FIX_SPF_DEADLOCK
atomic_long_t rsc_cow_fast_cnt;
atomic_long_t rsc_cow_slow_cnt;
atomic_long_t rsc_cow_over_cnt;
#endif

#ifdef CONFIG_RSC_APP_LAUNCH_BOOST
static unsigned long rsc_appboost_late/* = 3*HZ*/;
unsigned long rsc_appboost_late_no_framework = 3*HZ;
static DEFINE_MUTEX(rsc_launch_lock);
unsigned long __read_mostly rsc_app_boost_stime;
u32 __read_mostly rsc_app_boost_uid;
u16 __read_mostly rsc_app_boost_mainpid;

static ssize_t show_appboost_late(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur, i;
	struct task_struct *g, *p;

	/*when rsc_appboost_late is zero, use system_server setting, echo 3000 > /sys/rsc/launch*/
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "%u ms %s"
#ifdef CONFIG_RSC_NEW_FIX_SPF_DEADLOCK
		" fastcow %6ld slowcow %6ld slowover: %6ld"
#endif
		"\n",
		jiffies_to_msecs(rsc_appboost_late),
		rsc_appboost_late?"use fixed setting":"use system_server setting"
#ifdef CONFIG_RSC_NEW_FIX_SPF_DEADLOCK
		, atomic_long_read(&rsc_cow_fast_cnt), atomic_long_read(&rsc_cow_slow_cnt), atomic_long_read(&rsc_cow_over_cnt)
#endif
		);
	ret += cur;

	i = 0;
	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (p->mainthread
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
			|| p->rsc_svp
#endif
#ifdef CONFIG_RSC_LOCK_BOOST
			|| atomic64_read(&p->dynamic_svp)
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
		|| p->dynamic_svp_ulock_pid[DYNAMIC_SVP_BINDER]
#endif
		) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%02d m%02x "
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
				"v%02x g%d "
#endif
#ifdef CONFIG_RSC_LOCK_BOOST
				"d%d %llx "
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
				"B %5u "
#endif
				"uid %5d %16s %5d(%16s %5d)\n",
				i, p->mainthread,
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
				p->rsc_svp,
				rsc_get_task_group_nolock(p),
#endif
#ifdef CONFIG_RSC_LOCK_BOOST
				(int)!!atomic64_read(&p->dynamic_svp), atomic64_read(&p->dynamic_svp),
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
				(u32)p->dynamic_svp_ulock_pid[DYNAMIC_SVP_BINDER],
#endif
				task_uid(p).val, p->comm, p->pid,
				p->group_leader?p->group_leader->comm:"NULL",
				p->group_leader?p->group_leader->pid:99999);
			ret += cur;
			i++;
		}
	}
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
{
	int cpu, cnt, empty;

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"total heavysvpcnt: %d\n", atomic_read(&rsc_svp_task_cnt));
	ret += cur;

	for_each_online_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);
		cnt = rq->svp_manual_cnt;
		empty = list_empty(&rq->svp_task_list);

		if (cnt || !empty) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"cpu%d heavysvpcnt: %d svptask%d\n", cpu, cnt, !empty);
			ret += cur;
		}
	}
}
#endif

	return ret;
}

static ssize_t store_appboost_late(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long oldv;
	unsigned long value;

	/* use kstrtouint instead sscanf to prevent buffer overfollow. */
	ret = _kstrtoul(buf, 10, &value);
	if (ret != 0) {
		rsc_err("set appboost_lateerror %lu ! ret: %d buf: %s\n",
			value, ret, buf);
		return ret;
	}

	oldv = rsc_appboost_late;
	rsc_appboost_late = msecs_to_jiffies(value);
	rsc_appboost_late_no_framework = rsc_appboost_late;
	rsc_info("set appboost_late %lu -> %lu val: %lu\n",
			oldv, rsc_appboost_late, value);

	return count;
}

static struct kobj_attribute rsc_appboost_late_attr =
__ATTR(appboost_late, 0660, show_appboost_late, store_appboost_late);

static ssize_t show_launch(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "%u\t%u\tms\n",
		time_before(jiffies, rsc_app_boost_stime)?jiffies_to_msecs(rsc_app_boost_stime - jiffies):0,
		jiffies_to_msecs(rsc_appboost_late));
	ret += cur;

	return ret;
}

#if 0//def CONFIG_RSC_SVP_TASK_SCHEDULE
static atomic_t rsc_launch_cpuset_cnt = ATOMIC_INIT(0);
static struct task_struct *rsc_launch_task;
#ifndef tsk_cpus_allowed
#define tsk_cpus_allowed(tsk) (&(tsk)->cpus_allowed)
#endif

static void rsc_restore_cpuset_func(struct work_struct *work)
{
	struct task_struct *p = rsc_launch_task;
	cpumask_t cpumask;

	if (p) {
		cpumask_copy(&cpumask, tsk_cpus_allowed(p));
		rsc_set_cpus_allowed_ptr(p, NULL, 0);
		if (rsc_debug & RSC_APP_LAUNCH_BOOST)
			rsc_info("appboost %16s %5d cnt: %d restore cpuset g%d  %*pbl -> %*pbl all %*pbl lit %*pbl big %*pbl\n",
				p->comm, p->pid, atomic_read(&rsc_launch_cpuset_cnt),
				rsc_get_task_group(p),
				cpumask_pr_args(&cpumask),
				cpumask_pr_args(tsk_cpus_allowed(p)),
				cpumask_pr_args(&rsc_cpu_all),
				cpumask_pr_args(&rsc_cpu_littlecore),
				cpumask_pr_args(&rsc_cpu_bigcore));
		/*clear also in __set_cpus_allowed_ptr*/
		clear_rsc_svp(p, RSC_SVP_FORBIT_DO_CPUSET);
		put_task_struct(p);
	}
	atomic_dec(&rsc_launch_cpuset_cnt);
}
DECLARE_DELAYED_WORK(rsc_restore_cpuset, rsc_restore_cpuset_func);
#endif

#if defined(CONFIG_RSC_VAUDIT) && defined(CONFIG_RSC_APP_LAUNCH_BOOST)
extern unsigned long __read_mostly rsc_vaudit_app_launch_stime;
#endif
static ssize_t store_launch(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long value;
	unsigned long setvalue;
	u32 uid, pid;
	bool app;
	bool dualapp;
	struct task_struct *p;
	static unsigned long launchtime;

	/* use kstrtouint instead sscanf to prevent buffer overfollow. */
	/*ret = _kstrtoul(buf, 10, &value);
		if (ret != 0) {
		rsc_err("set launch error %lu ! ret: %d buf: %s\n",
			value, ret, buf);
		return ret;
	}
	*/
	ret = sscanf(buf, "%lu %u %u", &value, &uid, &pid);
	if (ret != 3) {
		rsc_err("set launch error! ret: %d buf: %s\n",
			ret, buf);
		return count;
	}

	app = (uid > RSC_APP_START_ && uid < (RSC_APP_START_+RSC_APP_MAX_));
	dualapp = (uid > RSC_DUAL_APP_START_ && uid < (RSC_DUAL_APP_START_+RSC_APP_MAX_));

	if (!app && !dualapp) {
		if (uid != SYSTEM_ID.val)
			rsc_err("set launch value error! time: %lu ms uid: %u pid: %u buf: %s\n",
				value, uid, pid, buf);
		else
			rsc_dbg(RSC_APP_LAUNCH_BOOST, "set launch value error! time: %lu ms uid: %u pid: %u buf: %s\n",
				value, uid, pid, buf);
		return -EINVAL;
	}

	launchtime++;
#ifndef CONFIG_RSC_VAUDIT
	if (rsc_debug & RSC_APP_LAUNCH_BOOST) {
#endif
		rcu_read_lock();
		/*cost 400ns - 900ns*/
		p = find_task_by_vpid(pid);
		if (!p || !thread_group_leader(p)) {
			rsc_err("set launch cound not find pid or not group leader%d ! time: %lu ms uid: %u pid: %u buf: %s\n",
				p?thread_group_leader(p):1, value, uid, pid, buf);
			/*rcu_read_unlock should after rsc_err because rsc_err use thread_group_leader(p)*/
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
			rcu_read_unlock_inline();
#else
			rcu_read_unlock();
#endif
			return -EINVAL;
		}

		/*task_uid(p), p->comm maybe been not set at this time!*/
#ifdef CONFIG_RSC_VAUDIT
		if (rsc_debug & (RSC_APP_LAUNCH_BOOST|RSC_VAUDIT))
#endif
			rsc_info("appboost %s %sAPP_LAUNCH_BOOST %sAPP_LAUNCH_BY_FRAMEWORK launchtime: %lu value %lu uid: %u "
				"task_uid: %u pid: %u %16s %5d mainthread%x appboost_late %lu  ms stime: %lu buf: %s\n",
				value?"enable":"disable", (rsc_feat(APP_LAUNCH_BOOST))?"":"NO_", rsc_feat(APP_LAUNCH_BY_FRAMEWORK)?"":"NO_",
				launchtime, value, uid, task_uid(p), pid, p->comm, p->pid, p->mainthread,
				msecs_to_jiffies(rsc_appboost_late), rsc_app_boost_stime, buf);
		/*p->mainthread |= RSC_UI_MAINTHREAD;*/
#ifdef CONFIG_RSC_VAUDIT
		p->mainthread |= RSC_VAUDIT_MAINTHREAD;
#endif
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
		/*p->rsc_svp |= RSC_SVP_MAINTHREAD;*/
		if (svp_enable_mainthread & SVP_ENABLE_MAINTHREAD_FAST_SCHED)
			set_rsc_svp(p, RSC_SVP_MAINTHREAD);

#if 0
		if (svp_enable_mainthread & SVP_ENABLE_MAINTHREAD_BIND_BIGCORE) {
			if (unlikely(atomic_inc_return(&rsc_launch_cpuset_cnt) > 1)) {
				rsc_warn("appboost %16s %5d cnt: %d launch too fast ignore! cpuset %*pbl\n",
					p->comm, p->pid, atomic_read(&rsc_launch_cpuset_cnt), cpumask_pr_args(tsk_cpus_allowed(p)));
				atomic_dec(&rsc_launch_cpuset_cnt);
			} else {
				rsc_set_cpus_allowed_ptr(p, &rsc_cpu_bigcore, 2);
				if (unlikely(rsc_appboost_late))
					setvalue = rsc_appboost_late;
				else
					setvalue = msecs_to_jiffies(value);
				get_task_struct(p);
				rsc_launch_task = p;
				smp_wmb();
				if (rsc_debug & RSC_APP_LAUNCH_BOOST)
					rsc_info("appboost %16s %5d cnt: %d launch start! cpuset %*pbl\n",
						p->comm, p->pid, atomic_read(&rsc_launch_cpuset_cnt), cpumask_pr_args(tsk_cpus_allowed(p)));
				schedule_delayed_work(&rsc_restore_cpuset, setvalue);
			}
		}
#endif
		/*clear_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK);*/
#endif
		rcu_read_unlock();
#ifndef CONFIG_RSC_VAUDIT
	}
#endif

#if 0
	//for CONFIG_RSC_LOCK_BOOST
	/*skill bbk launcher, m.bbk.launcher or m.bbk.launcher2*/
	/*^RSC appboost enable APP_LAUNCH_BOOST APP_LAUNCH_BY_FRAMEWORK launch value 3000 uid: 10110 pid: 2393			   main  2393 mainthread0 appboost_late 0  ms stime: 0 buf: 3000 10110 2393*/
	if (unlikely(launchtime < 1)/* || unlikely(!strncmp("m.bbk.launcher", p->comm, 14))*/) {
		return count;
	}
#endif

	mutex_lock(&rsc_launch_lock);
	rsc_app_boost_uid = uid;
	rsc_app_boost_mainpid = pid;
	smp_wmb();
	/*when rsc_appboost_late is zero, use system_server setting, echo 3000 > /sys/rsc/launch*/
	if (unlikely(rsc_appboost_late))
		setvalue = rsc_appboost_late;
	else
		setvalue = msecs_to_jiffies(value);

	if (value && (rsc_feat(APP_LAUNCH_BOOST)))
		rsc_app_boost_stime = jiffies + setvalue;
	else
		rsc_app_boost_stime = jiffies;

#if defined(CONFIG_RSC_VAUDIT) && defined(CONFIG_RSC_APP_LAUNCH_BOOST)
	rsc_vaudit_app_launch_stime = jiffies + 5*HZ;
#endif

	mutex_unlock(&rsc_launch_lock);

	return count;
}

static struct kobj_attribute rsc_launch_attr =
__ATTR(launch, 0660, show_launch, store_launch);
#endif

static DEFINE_MUTEX(rsc_svp_limit_lock);

#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
#define RSC_SVP_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute kobj_attr_##_name \
		= __ATTR(_name, _mode, _show, _store)

#define RSC_SVP_MODE_RW 0660
#define RSC_SVP_MODE_RO 0440

static const int min_sched_delay_granularity;
static const int max_sched_delay_granularity = 16;
static const int min_migration_delay_granularity;
static const int max_migration_delay_granulartiy = 16;

static ssize_t svp_min_sched_delay_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", svp_min_sched_delay_granularity);
}

static ssize_t svp_min_sched_delay_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int delay;

	ret = kstrtoint(buf, 10, &delay);
	if (ret)
		return ret;

	if (delay < min_sched_delay_granularity || delay > max_sched_delay_granularity)
		return -EINVAL;

	svp_min_sched_delay_granularity = delay;

	return len;
}

static ssize_t svp_min_migration_delay_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", svp_min_migration_delay);
}

static ssize_t svp_min_migration_delay_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int delay;
	ret = kstrtoint(buf, 10, &delay);
	if (ret)
		return ret;

	if (delay < min_migration_delay_granularity || delay > max_migration_delay_granulartiy)
		return -EINVAL;

	svp_min_migration_delay = delay;

	return len;
}

static ssize_t svp_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", rsc_svp_enable);
}

/*
* echo 0x8003 > /sys/rsc/svp/svp_enable to enable svp realtime task schedule
* now, default val:
* echo    0x3 > /sys/rsc/svp/svp_enable
*/
static ssize_t svp_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int enable;

	ret = sscanf(buf, "%x", &enable);
	if (ret != 1)
		return -EINVAL;

	rsc_svp_enable = enable;
	if (enable & RSC_SVP_ENQUEUE_ENABLE)
		RSC_SVP_MANUAL_VAL = RSC_SVP_MANUAL;
	else
		RSC_SVP_MANUAL_VAL = 0;
	return len;
}

static ssize_t svp_workaround_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", rsc_svp_workaround);
}

static ssize_t svp_workaround_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int enable;
	ret = kstrtoint(buf, 10, &enable);
	if (ret)
		return ret;

	rsc_svp_workaround = !!enable;

	return len;
}

u32 rsc_debug_enqueue_overcnt;
u32 rsc_debug_enqueue_thread_overcnt;
u32 rsc_debug_enqueue_bind_bigcore_cnt;
u32 rsc_debug_enqueue_otherthread_totalcnt;
u32 rsc_debug_enqueue_otherthread_actcnt;
u32 rsc_debug_enqueue_otherthread_cnt[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
char rsc_debug_enqueue_otherthread[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX][TASK_COMM_LEN];
char rsc_debug_enqueue_otherthread_gl[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX][TASK_COMM_LEN];
u32 rsc_debug_enqueue_heavytask_uid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
u16 rsc_debug_enqueue_heavytask_pid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
u16 rsc_debug_enqueue_heavytask_glpid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];

/*(477<<20)/1000000 == 500ms, unit is ms*/
u64 svp_enqueue_overtime = 477UL;
/*6s, unit is ms*/
u64 heavytask_limit_maxtime = 8000UL;
u64 heavytask_limit_intervaltime = 64UL;

/* force to set heavy task limit:
* 1.set heavytask_limit_to_litcore to HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_ALLTASK(2)
* 	echo 2 > /sys/rsc/svp/heavytask_limit_to_litcore
* 2. set HEAVYTASK_LIMIT_SVP_MIN_CNT to 0
*	echo 0 > /sys/rsc/svp/heavytask_svp_min_cnt
*/
#if 1
u32 heavytask_limit_to_litcore = HEAVYTASK_LIMIT_TO_LITCORE_SLIM;
/* enable heavytask limit to litcore, when svn cnt is more than HEAVYTASK_LIMIT_SVP_MIN_CNT */
int HEAVYTASK_LIMIT_SVP_MIN_CNT = 2;
#else
u32 heavytask_limit_to_litcore = HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_ALLTASK;
/* enable heavytask limit to litcore, when svn cnt is more than HEAVYTASK_LIMIT_SVP_MIN_CNT */
int HEAVYTASK_LIMIT_SVP_MIN_CNT;
#endif

/* 98.5% */
int svp_max_cpu_usage = 985;
u32 __read_mostly svp_max_cpu_usage_diff_cnt = 66;
u32  svp_max_cpu_usage_overload_cnt;
u32 svp_max_cpu_usage_overload_jiffies_ignore_cnt;
u32 svp_max_cpu_usage_overload_delay_ms = 10000;
u32 svp_max_cpu_usage_overload_preempt_ignore_cnt;

int svp_limit_sched_setaffinity = 1;
atomic_t svp_limit_sched_setaffinity_cnt;
atomic_t svp_notlimit_sched_setaffinity_cnt;

static ssize_t svp_enqueue_overtime_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0, cur;
	int i;
	int cnt = min(rsc_debug_enqueue_otherthread_actcnt, (u32)RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX);

	cur = scnprintf(buf, PAGE_SIZE, "%llu\t%u\t%u\t%u\t%u\t%u\n",
		svp_enqueue_overtime, rsc_debug_enqueue_overcnt,
		rsc_debug_enqueue_thread_overcnt, rsc_debug_enqueue_otherthread_totalcnt,
		cnt, rsc_debug_enqueue_bind_bigcore_cnt);
	ret += cur;
	if (cnt) {
		for (i = 0; i < cnt; i++) {
			cur = scnprintf(buf+ret, PAGE_SIZE-ret, "%16s\t%5d\t%16s\t%5d\t%5d\t%u\n",
				rsc_debug_enqueue_otherthread[i], rsc_debug_enqueue_heavytask_pid[i],
				rsc_debug_enqueue_otherthread_gl[i], rsc_debug_enqueue_heavytask_glpid[i],
				rsc_debug_enqueue_heavytask_uid[i], rsc_debug_enqueue_otherthread_cnt[i]);
			ret += cur;
		}
	}
	return ret;
}

static ssize_t svp_enqueue_overtime_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	u64 time;

	ret = kstrtoull(buf, 10, &time);
	if (ret)
		return ret;

	svp_enqueue_overtime = time;

	return len;
}

static ssize_t svp_limit_maxtime_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%llu\n",
		heavytask_limit_maxtime);
}

static ssize_t svp_limit_maxtime_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	u64 time;

	ret = kstrtoull(buf, 10, &time);
	if (ret)
		return ret;

	heavytask_limit_maxtime = time;

	return len;
}

static ssize_t svp_heavytask_limit_to_litcore_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0x%x\n",
		heavytask_limit_to_litcore);
}

static ssize_t svp_heavytask_limit_to_litcore_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int mode;

	ret = kstrtoint(buf, 10, &mode);
	if (ret)
		return ret;

	heavytask_limit_to_litcore = mode;

	return len;
}


static ssize_t svp_limit_sched_setaffinity_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %d %d\n",
		svp_limit_sched_setaffinity,
		atomic_read(&svp_limit_sched_setaffinity_cnt),
		atomic_read(&svp_notlimit_sched_setaffinity_cnt));
}

static ssize_t svp_limit_sched_setaffinity_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int enable;

	ret = kstrtoint(buf, 10, &enable);
	if (ret)
		return ret;

	svp_limit_sched_setaffinity = !!enable;

	return len;
}

static ssize_t  svp_limit_process_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0, cur;
	int i;
	int heavytask_limit_num = READ_ONCE(do_svp_limit_process_num);
	int affinity_num = READ_ONCE(do_svp_limit_sched_setaffinity_num);

	cur = scnprintf(buf, PAGE_SIZE, "%d\t%d\t%d\t%d\n",
			heavytask_limit_num, affinity_num, do_svp_limit_process_num_backup, do_svp_limit_sched_setaffinity_num_backup);
	ret += cur;

	if (heavytask_limit_num) {
		cur = scnprintf(buf+ret, PAGE_SIZE-ret, "heavyproc:\n");
		ret += cur;
		for (i = 0; i < heavytask_limit_num; i++) {
			cur = scnprintf(buf+ret, PAGE_SIZE-ret, "\t%d %s\n",
				i, do_svp_limit_process_list_str[i]);
			ret += cur;
		}
	}

	if (affinity_num) {
		cur = scnprintf(buf+ret, PAGE_SIZE-ret, "affinityproc:\n");
		ret += cur;
		for (i = 0; i < affinity_num; i++) {
			cur = scnprintf(buf+ret, PAGE_SIZE-ret, "\t%d %s\n",
				i, do_svp_limit_sched_setaffinity_list_str[i]);
			ret += cur;
		}
	}

	return ret;
}

/*
* limit heavytask:					echo 1 com.tencent.tmgp.sgame xxx.sgame > /sys/rsc/svp/svp_limit_process
* limit sched_setaffinity: 			echo 2 com.tencent.tmgp.sgame xxx.sgame > /sys/rsc/svp/svp_limit_process
* disable limit heavytask:			echo 3 0 > /sys/rsc/svp/svp_limit_process
* enable limit heavytask:			echo 3 1 > /sys/rsc/svp/svp_limit_process
* disable limit sched_setaffinity:	echo 4 0 > /sys/rsc/svp/svp_limit_process
* enable limit sched_setaffinity:	echo 4 1 > /sys/rsc/svp/svp_limit_process
*/
static ssize_t svp_limit_process_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int mode = -1;
	int ret;
	BUILD_BUG_ON(RSC_HEAVYTASK_LIMIT_MAX_PROCESS < 8);
	BUILD_BUG_ON(RSC_SET_AFFINITY_LIMIT_MAX_PROCESS < 8);
	BUILD_BUG_ON(RSC_MAX_PACKAGE_NAME < 48);

	mutex_lock(&rsc_svp_limit_lock);

	ret = sscanf(buf, "%d", &mode);
	if (ret == 1) {
		if (mode == RSC_LIMIT_PROCESS_MODE_TO_LITCORE) {
			ret = sscanf(buf, "%d %47s %47s %47s %47s %15s %47s %47s %47s", &mode
							, do_svp_limit_process_list_str[0]
							, do_svp_limit_process_list_str[1]
							, do_svp_limit_process_list_str[2]
							, do_svp_limit_process_list_str[3]
							, do_svp_limit_process_list_str[4]
							, do_svp_limit_process_list_str[5]
							, do_svp_limit_process_list_str[6]
							, do_svp_limit_process_list_str[7]
				);
			smp_mb();
			if (ret > 1 && ret <= (RSC_HEAVYTASK_LIMIT_MAX_PROCESS+1)) {
				do_svp_limit_process_num = ret - 1;
				do_svp_limit_process_num_backup = do_svp_limit_process_num;
			}
		} else if (mode == RSC_LIMIT_PROCESS_MODE_SCHED_AFFINITY) {
			ret = sscanf(buf, "%d %47s %47s %47s %47s %15s %47s %47s %47s", &mode
							, do_svp_limit_sched_setaffinity_list_str[0]
							, do_svp_limit_sched_setaffinity_list_str[1]
							, do_svp_limit_sched_setaffinity_list_str[2]
							, do_svp_limit_sched_setaffinity_list_str[3]
							, do_svp_limit_sched_setaffinity_list_str[4]
							, do_svp_limit_sched_setaffinity_list_str[5]
							, do_svp_limit_sched_setaffinity_list_str[6]
							, do_svp_limit_sched_setaffinity_list_str[7]
			);
			smp_mb();
			if (ret > 1 && ret <= (RSC_SET_AFFINITY_LIMIT_MAX_PROCESS+1)) {
					do_svp_limit_sched_setaffinity_num = ret - 1;
					do_svp_limit_sched_setaffinity_num_backup = do_svp_limit_sched_setaffinity_num;
				}
		} else if (mode == RSC_LIMIT_PROCESS_MODE_TO_LITCORE_SWITCH) {
				int enable;

				ret = sscanf(buf, "%d %d", &mode, &enable);
				if (ret == 2) {
					if (enable) {
						do_svp_limit_process_num = do_svp_limit_process_num_backup;
					} else {
						do_svp_limit_process_num_backup = do_svp_limit_process_num;
						do_svp_limit_process_num = 0;
					}
				} else
					goto error_exit;
			} else if (mode == RSC_LIMIT_PROCESS_MODE_SCHED_AFFINITY_SWITCH) {
				int enable;

				ret = sscanf(buf, "%d %d", &mode, &enable);
				if (ret == 2) {
					if (enable) {
						do_svp_limit_sched_setaffinity_num = do_svp_limit_sched_setaffinity_num_backup;
					} else {
						do_svp_limit_sched_setaffinity_num_backup = do_svp_limit_sched_setaffinity_num;
						do_svp_limit_sched_setaffinity_num = 0;
					}
				} else
					goto error_exit;
			} else
				goto error_exit;
	} else
		goto error_exit;

	mutex_unlock(&rsc_svp_limit_lock);
	return len;

error_exit:
	mutex_unlock(&rsc_svp_limit_lock);

	return -EINVAL;
}

static ssize_t svp_heavytask_svp_min_cnt_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
		HEAVYTASK_LIMIT_SVP_MIN_CNT);
}

static ssize_t svp_heavytask_svp_min_cnt_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int cnt;

	ret = kstrtoint(buf, 10, &cnt);
	if (ret)
		return ret;

	HEAVYTASK_LIMIT_SVP_MIN_CNT = cnt;

	return len;
}

static ssize_t svp_max_cpu_usage_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d %u overload: %u %u %u delay %u ms\n",
		svp_max_cpu_usage, svp_max_cpu_usage_diff_cnt, svp_max_cpu_usage_overload_cnt,
		svp_max_cpu_usage_overload_jiffies_ignore_cnt, svp_max_cpu_usage_overload_preempt_ignore_cnt,
		svp_max_cpu_usage_overload_delay_ms);
}

static ssize_t  svp_max_cpu_usage_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int per;
	u32 delay;

	ret = sscanf(buf, "%u %d", &delay, &per);

	if (ret < 1 && ret > 2)
		return -EINVAL;

	svp_max_cpu_usage_overload_delay_ms = delay;
	if (ret > 1) {
		if (per < 0 || per > 1000)
			return -EINVAL;

		svp_max_cpu_usage = per;
		if (1000 != per)
			svp_max_cpu_usage_diff_cnt = 1000/(1000 - per);
		else
			svp_max_cpu_usage_diff_cnt = UINT_MAX;
	}
	return len;
}

static ssize_t svp_enable_mainthread_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
		svp_enable_mainthread);
}

static ssize_t  svp_enable_mainthread_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	int enable;

	ret = kstrtoint(buf, 10, &enable);
	if (ret)
		return ret;

	svp_enable_mainthread = enable;

	return len;
}

static RSC_SVP_ATTR(svp_min_sched_delay_granularity, RSC_SVP_MODE_RW, svp_min_sched_delay_show, svp_min_sched_delay_store);
static RSC_SVP_ATTR(svp_min_migration_delay, RSC_SVP_MODE_RW, svp_min_migration_delay_show, svp_min_migration_delay_store);
static RSC_SVP_ATTR(svp_enable, RSC_SVP_MODE_RW, svp_enable_show, svp_enable_store);
static RSC_SVP_ATTR(svp_workaround, RSC_SVP_MODE_RW, svp_workaround_show, svp_workaround_store);
static RSC_SVP_ATTR(svp_enqueue_overtime, RSC_SVP_MODE_RW, svp_enqueue_overtime_show, svp_enqueue_overtime_store);
static RSC_SVP_ATTR(limit_maxtime, RSC_SVP_MODE_RW, svp_limit_maxtime_show, svp_limit_maxtime_store);
static RSC_SVP_ATTR(heavytask_limit_to_litcore, RSC_SVP_MODE_RW, svp_heavytask_limit_to_litcore_show, svp_heavytask_limit_to_litcore_store);
static RSC_SVP_ATTR(svp_limit_sched_setaffinity, RSC_SVP_MODE_RW, svp_limit_sched_setaffinity_show, svp_limit_sched_setaffinity_store);
static RSC_SVP_ATTR(svp_limit_process, RSC_SVP_MODE_RW, svp_limit_process_show, svp_limit_process_store);
static RSC_SVP_ATTR(heavytask_svp_min_cnt, RSC_SVP_MODE_RW, svp_heavytask_svp_min_cnt_show, svp_heavytask_svp_min_cnt_store);
static RSC_SVP_ATTR(svp_max_cpu_usage, RSC_SVP_MODE_RW, svp_max_cpu_usage_show, svp_max_cpu_usage_store);
static RSC_SVP_ATTR(svp_enable_mainthread, RSC_SVP_MODE_RW, svp_enable_mainthread_show, svp_enable_mainthread_store);


#ifdef CONFIG_RSC_LOCK_BOOST
static ssize_t svp_stat_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	int type;
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
	int i, j, depth;
#endif
	unsigned long depthcnt = 0;
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
	unsigned long totalartboost = 0;
	unsigned long enqueue_count = 0;
	unsigned long deq_write_count = 0;
	int cpu;
#endif

	for (type = 0; type < SVP_DEPTH_MAX; type++)
		depthcnt += dynamic_svp_depth_count[type];

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"testenq enable: %u max_depth %2d count %8lu granularity %d ms tickhit %u to %u "
		"tick_preempt %u preempt_pick %u select_cpu_hit %lu "
		"svp_run_cpus %*pbl\n",
		dynamic_svp_enqueue_enable, dynamic_svp_max_depth, depthcnt,
		svp_max_dynamic_granularity, dynamic_svp_tick_hit,
		dynamic_svp_timeout_total_count, dynamic_svp_tick_preempt_tick_count,
		rsc_svp_preempt_pickup_count,
		rsc_svp_select_cpu_hit_count,
		cpumask_pr_args(&rsc_svp_run_cpus)
		);
	ret += cur;

#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX

	for_each_possible_cpu(cpu) {
		enqueue_count += per_cpu(rsc_rwsem_enq_boost_count, cpu);
		deq_write_count += per_cpu(rsc_rwsem_deq_boost_write_count, cpu);
	}

	for (i = 0; i < RSC_MAX_BOOST_PID_ARRAY_SIZE; i++)
		for (j = 0; j < 2; j++)
			totalartboost += rsc_futex_boost_count[i][j];

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "futexboost count %7lu %7lu valwr %2u pidcg %5u pidinv %2u "
		"zero32 %6u %6u "
#ifdef RSC_FUTEX_DEBUG
		"stsk %6u "
#endif
		"nb32 %7lu %7lu "
#ifdef RSC_FUTEX_DEBUG
		"stsk %7lu "
#endif
		"timeinv32: %3u %3u "
		"mutex wake %8u boost %5u fail %5u over %u "
		"rwsem enq %6lu deq %6lu "
		"DYNAMIC_SVP_PAGE_ALLOC: %7u UIorMAN %6u dynamic %6u "
		"\n"
		, rsc_futex_lock_boost_hit
		, totalartboost
		, rsc_futex_uaddr2_value_wrong_count
		, rsc_futex_pid_change_count
		, rsc_futex_pid_change_pid_invalid_count
		, rsc_fuetx_boost_pid_zero[0], rsc_fuetx_boost_pid_zero[1]
#ifdef RSC_FUTEX_DEBUG
		, rsc_fuetx_boost_pid_zero[2]
#endif
		, rsc_futex_not_boost_count[0], rsc_futex_not_boost_count[1]
#ifdef RSC_FUTEX_DEBUG
		, rsc_futex_not_boost_count[2]
#endif
		, rsc_futex_timespec_invalid_count[0], rsc_futex_timespec_invalid_count[1]

		, rsc_mutex_wakeup_count
		, rsc_mutex_seq_boost_count
		, rsc_mutex_seq_boost_fail_count
		, rsc_mutex_seq_boost_loop_over_count
		, enqueue_count, deq_write_count
		, dynamic_svp_count[DYNAMIC_SVP_PAGE_ALLOC]
		, dynamic_svp_count[DYNAMIC_SVP_PAGE_ALLOC] - dynamic_svp_page_alloc_count
		, dynamic_svp_page_alloc_count
		);
	ret += cur;

	for (i = 0; i < RSC_MAX_BOOST_PID_ARRAY_SIZE; i++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "NAME %7s COUNT32 %10u %10u "
#ifdef RSC_FUTEX_DEBUG
		"stsk %8u "
#endif
		"ZERO32  %10u(%3llu%%) %10u(%3llu%%) "
#ifdef RSC_FUTEX_DEBUG
		"stsk %8u(%3llu%%) "
#endif
		"\n"
		, rsc_futex_boost_pid_name[i]
		, rsc_futex_boost_count[i][0], rsc_futex_boost_count[i][1]
#ifdef RSC_FUTEX_DEBUG
		, rsc_futex_boost_count[i][2]
#endif
		, rsc_futex_boost_zero_count[i][0], (u64)((u64)rsc_futex_boost_zero_count[i][0]*100/(u64)rsc_futex_boost_count[i][0])
		, rsc_futex_boost_zero_count[i][1], (u64)((u64)rsc_futex_boost_zero_count[i][1]*100/(u64)rsc_futex_boost_count[i][1])
#ifdef RSC_FUTEX_DEBUG
		, rsc_futex_boost_zero_count[i][2], (u64)((u64)rsc_futex_boost_zero_count[i][2]*100/(u64)rsc_futex_boost_count[i][2])
#endif
		);
		ret += cur;
	}

	for (type = 0; type < DYNAMIC_SVP_MAX; type++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"%16s deadloop %4u %4u trace:", rsc_dynamic_svp_name[type],
				rsc_futex_block_chain_dead_loop_count[type][0], rsc_futex_block_chain_dead_loop_count[type][1]);
		ret += cur;
		for (depth = 0; depth < SVP_DEPTH_MAX; depth++) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				" %d: %6u", depth, rsc_lock_boost_trace[type][depth]);
			ret += cur;
		}
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"\n");
		ret += cur;
	}
#endif

	for (type = 0; type < DYNAMIC_SVP_MAX; type++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"NAME %16s COUNT %8lu "
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
			"chain %5u "
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_STAT
			"actt %10llu %5llu us/pertime "
#endif
			"time %10llu %5llu us/pertime "
			"TO %5u deq %5u\n",
			rsc_dynamic_svp_name[type], dynamic_svp_count[type],
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
			rsc_block_chain_boost_count[type],
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_STAT
			dynamic_svp_cost_time[type]/1000,
			dynamic_svp_cost_time[type]/(dynamic_svp_count[type]*1000 ? dynamic_svp_count[type]*1000 : 1),
#endif
			dynamic_svp_slim_cost_time[type]/1000,
			dynamic_svp_slim_cost_time[type]/(dynamic_svp_count[type]*1000 ? dynamic_svp_count[type]*1000 : 1),
			dynamic_svp_timeout_count[type], dynamic_svp_timeout_deq_count[type]
			);
		ret += cur;
	}

	for (type = 0; type <= SVP_DEPTH_MAX; type++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"depth%d %8u\n",
			type, dynamic_svp_depth_count[type]
			);
		ret += cur;
	}

	return ret;
}

static ssize_t svp_stat_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	u32 value1, value2;

	ret = sscanf(buf, "%u %u", &value1, &value2);
	if (ret < 1 || ret > 2)
		return -EINVAL;

	if (ret > 0)
		dynamic_svp_enqueue_enable = value1;
	if (ret > 1)
		svp_max_dynamic_granularity = value2;

	return len;
}

static RSC_SVP_ATTR(svp_stat, RSC_SVP_MODE_RW, svp_stat_show, svp_stat_store);
#endif

#endif

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
#define RSC_SIG_MODE_UID_AND_SIGNUM 0
#define RSC_SIG_MODE_PROCESS_NAME 1
#define RSC_SIG_MODE_ENABLE_DISABLE 2
#define RSC_SIG_MODE_THRESHOLD_TIME_US 3
#define RSC_SIG_MODE_FORCE_IGNORE_SIG24 4
#define RSC_SIG_MODE_FORCE_DEALOCK_TIME_US 5
#define RSC_SIG_MODE_SET_RESEND_SIGNUM 6
#define RSC_SIG_MODE_SET_GC_THREAD 7

static ssize_t show_rsc_sig(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	int i;
	struct task_struct *g, *p;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "%u\t%d\t%d\t%d\t%d\t%lu\t%d\t%d\t%u\t%d\t%d\t%d\n",
		rsc_hok_changename_cnt,/*change name cnt*/
		atomic_read(&rsc_fix_suspend_deadlock_time),/*fix time*/
		atomic_read(&rsc_forcefix_suspend_deadlock_time),/*force fix time*/
		atomic_read(&rsc_fix_suspend_deadlock_fail_time),/*fail time*/
		atomic_read(&rsc_fix_suspend_try_time),/*try time*/
		rsc_sigsuspend_cnt,/*suspend count*/
		rsc_sigsuspend_toolong_cnt,/*suspend too long count*/
		rsc_sig24_ignore,/*ignore count*/
		rsc_max_sigsuspend_costtime_ms,/*max suspend time ms*/
		atomic_read(&rsc_hok_not_in_suspend_cnt[1]),/*mainthread not in suspend*/
		atomic_read(&rsc_hok_not_in_suspend_cnt[2]),/*gc thread not in suspend*/
		atomic_read(&rsc_hok_not_in_suspend_cnt[3]));/*no gc thread not in suspend*/
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "UID: %d sigstart: %d sigend: %d "
		"sig1: %d sig2: %d sig3: %d sig4: %d\n"
		"checknum: %d ",
		rsc_check_suspend_uid, rsc_suspend_check_start_signum, rsc_suspend_check_end_signum,
		rsc_suspend_check_sig[0], rsc_suspend_check_sig[1], rsc_suspend_check_sig[2], rsc_suspend_check_sig[3],
		rsc_suspend_check_task_num);
	ret += cur;

	for (i = 0; i < rsc_suspend_check_task_num; i++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%16s "
									, rsc_suspend_check_task[i]);
		ret += cur;
	}
#if 0
/*debug*/
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "xx1 %16s xx2 %16s xx3 %s ooo %s AAA %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c"
								, rsc_suspend_check_task[0], rsc_suspend_check_task[1], &rsc_suspend_check_task[1][0], (char *)((unsigned long)&rsc_suspend_check_task[0][0]+17)
								, rsc_suspend_check_task[1][0], rsc_suspend_check_task[1][1], rsc_suspend_check_task[1][2], rsc_suspend_check_task[1][3]
								, rsc_suspend_check_task[1][4], rsc_suspend_check_task[1][5], rsc_suspend_check_task[1][6], rsc_suspend_check_task[1][7]
								, rsc_suspend_check_task[1][8], rsc_suspend_check_task[1][9], rsc_suspend_check_task[1][10], rsc_suspend_check_task[1][11]
								, rsc_suspend_check_task[1][12], rsc_suspend_check_task[1][13], rsc_suspend_check_task[1][14], rsc_suspend_check_task[1][15]
								);
	ret += cur;
#endif
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\nexample:\n"
								"disable fix: echo 2 0 > /sys/rsc/rsc_sig\n"
								"force delay: echo 5 10000 > /sys/rsc/rsc_sig	for easily reproducing\n"
								"set     sig: echo 0 uid sigstart sigend sig1 sig2 sig3 sig4 > /sys/rsc/rsc_sig\n"
								"set process: echo 1 process1 ... process8 > /sys/rsc/rsc_sig\n");
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "enable%d fignore: %d rsendsig: %d fixtime: %d ffixtime: %d failtime: %d "
											"trytime: %d thrtime: %lu forcedelay %lu us supendcnt: %lu sigtoolongcnt: %d "
											"fatalpend : %d otherpend %d rsc_sig24_ignore: %d maxstime: %u ms notsuspend %d %d %d gcthread: %s exitcnt %u gcmd %u cname %u\n",
		rsc_check_signal_enable, rsc_force_ignore_sig24, rsc_resend_signum, atomic_read(&rsc_fix_suspend_deadlock_time),
		atomic_read(&rsc_forcefix_suspend_deadlock_time),
		atomic_read(&rsc_fix_suspend_deadlock_fail_time),
		atomic_read(&rsc_fix_suspend_try_time),
		rsc_sig_cost_time_thr/1000,
		rsc_force_delay_for_suspend_deadlock_time_ns/1000,
		rsc_sigsuspend_cnt, rsc_sigsuspend_toolong_cnt,
		rsc_sig_fatal_pending_cnt, rsc_sig_other_pending_cnt, rsc_sig24_ignore,
		rsc_max_sigsuspend_costtime_ms,
		atomic_read(&rsc_hok_not_in_suspend_cnt[1]),
		atomic_read(&rsc_hok_not_in_suspend_cnt[2]),
		atomic_read(&rsc_hok_not_in_suspend_cnt[3]),
		rsc_hok_gc_thread, rsc_sig_has_val_cnt, rsc_get_cmdline_cnt, rsc_hok_changename_cnt);
	ret += cur;

	/*
	* only root can read it
	* I am not sure, rsc_need_check_suspend -> p->group_leader->comm is or not safe?
	*/
	if ((current_uid().val == 0) && (unlikely(rsc_debug & RSC_FIX_DEADLOCK))) {
		rcu_read_lock();
		i = 0;
		for_each_process_thread(g, p) {
			u32 cnt;
			if (try_get_task_stack(p)) {
				cnt = atomic_read(&task_thread_info(p)->rsc_sig);
				put_task_stack(p);
				if (cnt) {
					cur = snprintf(buf + ret, PAGE_SIZE - ret, "%2d rsc_sig: high %d low %d uid %5d %16s %5d(%16s %5d) needcheck%d\n",
						i, cnt>>16, cnt&0xffff, task_uid(p).val, p->comm, p->pid,
						p->group_leader?p->group_leader->comm:"NULL",
						p->group_leader?p->group_leader->pid:99999,
						rsc_need_check_suspend(p));
					ret += cur;
					i++;
				}
			}
		}
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
	}

	return ret;
}

static ssize_t store_rsc_sig(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int uid = -1, sigstart = -1, sigend = -1, mode = -1;
	struct task_struct *real_parent;
	BUILD_BUG_ON(RSC_SUSPEND_CHECK_MAX_PROCESS < 8);
	BUILD_BUG_ON(TASK_COMM_LEN < 16);
	BUILD_BUG_ON(RSC_SUSPEND_CHECK_SPEC_SIGNUM < 4);

	ret = sscanf(buf, "%d", &mode);
	if (ret < 1)
		return -EINVAL;

	if (RSC_SIG_MODE_UID_AND_SIGNUM == mode) {
		ret = sscanf(buf, "%d %d %d %d %d %d %d %d",
			&mode, &uid, &sigstart, &sigend,
			&rsc_suspend_check_sig[0],
			&rsc_suspend_check_sig[1],
			&rsc_suspend_check_sig[2],
			&rsc_suspend_check_sig[3]
			);
		if (ret >= 2)
			rsc_check_suspend_uid = uid;
		if (ret >= 3)
			rsc_suspend_check_start_signum = sigstart;
		if (ret >= 4)
			rsc_suspend_check_end_signum = sigend;
	} else if (RSC_SIG_MODE_PROCESS_NAME == mode) {
		if (strlen(buf) > (1+1+(TASK_COMM_LEN-1+1)*RSC_SUSPEND_CHECK_MAX_PROCESS-1))
			return -EINVAL;
		ret = sscanf(buf, "%d %15s %15s %15s %15s %15s %15s %15s %15s", &mode
									, rsc_suspend_check_task[0]
									, rsc_suspend_check_task[1]
									, rsc_suspend_check_task[2]
									, rsc_suspend_check_task[3]
									, rsc_suspend_check_task[4]
									, rsc_suspend_check_task[5]
									, rsc_suspend_check_task[6]
									, rsc_suspend_check_task[7]
				);
			smp_mb();
			rsc_suspend_check_task_num = min(ret - 1, RSC_SUSPEND_CHECK_MAX_PROCESS);
	} else if (RSC_SIG_MODE_ENABLE_DISABLE == mode) {
		ret = sscanf(buf, "%d %d",
			&mode, &rsc_check_signal_enable);
	} else if (RSC_SIG_MODE_THRESHOLD_TIME_US == mode) {
		unsigned long time_us;
		ret = sscanf(buf, "%d %lu",
			&mode, &time_us);
		if (ret >= 2)
			rsc_sig_cost_time_thr = time_us*1000;
	} else if (RSC_SIG_MODE_FORCE_IGNORE_SIG24 == mode) {

		rcu_read_lock();
		real_parent = rcu_dereference(current->real_parent);
		/*only root can modify it*/
		if ((current_uid().val == 0) && real_parent && !strncmp("adbd", real_parent->comm, TASK_COMM_LEN))
			ret = sscanf(buf, "%d %d",
				&mode, &rsc_force_ignore_sig24);
		rcu_read_unlock();
	} else if (RSC_SIG_MODE_FORCE_DEALOCK_TIME_US == mode) {
		unsigned long time_us;

		rcu_read_lock();
		real_parent = rcu_dereference(current->real_parent);
		/*only root can modify it*/
		if ((current_uid().val == 0) && real_parent && !strncmp("adbd", real_parent->comm, TASK_COMM_LEN)) {
			ret = sscanf(buf, "%d %lu",
				&mode, &time_us);
			if (ret >= 2)
				rsc_force_delay_for_suspend_deadlock_time_ns = time_us*1000;
		}
		rcu_read_unlock();
	} else if (RSC_SIG_MODE_SET_RESEND_SIGNUM == mode) {
		/*only root can modify it*/
		if (current_uid().val == 0) {
			ret = sscanf(buf, "%d %d",
					&mode, &rsc_resend_signum);
		}
	} else if (RSC_SIG_MODE_SET_GC_THREAD == mode) {
		ret = sscanf(buf, "%d %15s", &mode, rsc_hok_gc_thread);
	} else
		return -EINVAL;

	rcu_read_lock();
	real_parent = rcu_dereference(current->real_parent);
	rsc_info("sigsuspend set rsc_sig uid %5d %16s %5d(%16s %5d) <- (%16s %5d) ret: %d mode: %d buf: %s\n"
				, current_uid().val
				, current->comm, current->pid
				, current->group_leader?current->group_leader->comm:"NULL"
				, current->group_leader?current->group_leader->pid:99999
				, real_parent?real_parent->comm:"NULL"
				, real_parent?real_parent->pid:99999
				, ret, mode, buf);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif

	return count;
}

static struct kobj_attribute rsc_sig_attr =
__ATTR(rsc_sig, 0660, show_rsc_sig, store_rsc_sig);
#endif

#define RSC_SVP_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute kobj_attr_##_name \
		= __ATTR(_name, _mode, _show, _store)

#define RSC_SVP_MODE_RW 0660

#ifdef CONFIG_RSC_KSWAPD_FORCE_SLEEP
extern u32 kswapd_force_sleep_free_pages;
extern u32 kswapd_force_sleep_filecache_pages;
extern u32 kswapd_force_sleep_count[5];
extern u32 kswapd_force_sleep_count_in_balance[5];
extern u32 kswapd_wakeup_order_count[5];
extern u32 direct_reclaim_over_thr_count[5];
static ssize_t kswapd_force_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0, cur;

	cur = scnprintf(buf, PAGE_SIZE, "%u %u MB (%u %u)\n"
				"%u %u %u %u %u\n"
				"%u %u %u %u %u\n"
				"%u %u %u %u %u\n"
				"%u %u %u %u %u\n",
				kswapd_force_sleep_free_pages >> (20 - PAGE_SHIFT),
				kswapd_force_sleep_filecache_pages >> (20 - PAGE_SHIFT),
				kswapd_force_sleep_free_pages,
				kswapd_force_sleep_filecache_pages,
				kswapd_force_sleep_count[0],
				kswapd_force_sleep_count[1],
				kswapd_force_sleep_count[2],
				kswapd_force_sleep_count[3],
				kswapd_force_sleep_count[4],
				kswapd_force_sleep_count_in_balance[0],
				kswapd_force_sleep_count_in_balance[1],
				kswapd_force_sleep_count_in_balance[2],
				kswapd_force_sleep_count_in_balance[3],
				kswapd_force_sleep_count_in_balance[4],
				direct_reclaim_over_thr_count[0],
				direct_reclaim_over_thr_count[1],
				direct_reclaim_over_thr_count[2],
				direct_reclaim_over_thr_count[3],
				direct_reclaim_over_thr_count[4],
				kswapd_wakeup_order_count[0],
				kswapd_wakeup_order_count[1],
				kswapd_wakeup_order_count[2],
				kswapd_wakeup_order_count[3],
				kswapd_wakeup_order_count[4]
			);
	ret += cur;

	return ret;
}

static ssize_t kswapd_force_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int ret;
	u32 free, filecache;
	u32 freeback = kswapd_force_sleep_free_pages;
	u32 filecacheback =kswapd_force_sleep_filecache_pages;

	ret = sscanf(buf, "%u %u"
					, &free
					, &filecache
				);
	if (ret >= 1)
		kswapd_force_sleep_free_pages = free << (20 - PAGE_SHIFT);
	if (ret >= 2)
		kswapd_force_sleep_filecache_pages = filecache << (20 - PAGE_SHIFT);

	rsc_info("set kswapd_force_sleep "
			"free %u MB %u -> %u pages "
			"filecache %u MB %u -> %u pages "
			"ret: %d buf: %s\n",
			kswapd_force_sleep_free_pages >> (20 - PAGE_SHIFT), freeback, kswapd_force_sleep_free_pages,
			kswapd_force_sleep_filecache_pages >> (20 - PAGE_SHIFT), filecacheback, kswapd_force_sleep_filecache_pages,
			ret, buf);

	return len;
}
static RSC_SVP_ATTR(kswapd_force_sleep, RSC_SVP_MODE_RW, kswapd_force_sleep_show, kswapd_force_sleep_store);
#endif

#ifdef CONFIG_RSC_KSWAPD_FORCE_SLEEP
static struct attribute *rsc_svp_attrs[] = {
#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
	&kobj_attr_svp_min_sched_delay_granularity.attr,
	&kobj_attr_svp_min_migration_delay.attr,
	&kobj_attr_svp_enable.attr,
	&kobj_attr_svp_workaround.attr,
	&kobj_attr_svp_enqueue_overtime.attr,
	&kobj_attr_limit_maxtime.attr,
	&kobj_attr_heavytask_limit_to_litcore.attr,
	&kobj_attr_svp_limit_sched_setaffinity.attr,
	&kobj_attr_svp_limit_process.attr,
	&kobj_attr_heavytask_svp_min_cnt.attr,
	&kobj_attr_svp_max_cpu_usage.attr,
	&kobj_attr_svp_enable_mainthread.attr,
#endif
#ifdef CONFIG_RSC_LOCK_BOOST
	&kobj_attr_svp_stat.attr,
#endif
#ifdef CONFIG_RSC_KSWAPD_FORCE_SLEEP
	&kobj_attr_kswapd_force_sleep.attr,
#endif
	NULL,
};

#if (defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)) || defined(CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE) || defined(CONFIG_RSC_DROP_MMAP_SEM_IN_MADVISE)
//core.c
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0) && defined(CONFIG_RSC_CHECK_MMAP_SEM)
extern u32 rsc_io_sched_to_count;
extern u32 rsc_io_sched_count;
extern u32 rsc_io_sched_to_write_count;
extern u32 rsc_io_sched_write_count;

extern u32 rsc_io_sched_waitlock_count;
extern u32 rsc_io_sched_waitread_count;
extern u32 rsc_io_sched_waitwrite_count;
extern u32 rsc_io_sched_otherwait_count;
extern u32 rsc_io_sched_nonpagewait_count;
extern u32 rsc_io_sched_waitvdsoc_count;
extern u32 rsc_io_sched_waitvdsod_count;
extern u32 rsc_io_sched_waituptodatenocg_count;

extern u32 rsc_io_sched_waitlock_inslowpath_count;
extern u32 rsc_io_sched_waitread_inslowpath_count;
extern u32 rsc_io_sched_waitwrite_inslowpath_count;
extern u32 rsc_io_sched_otherwait_inslowpath_count;
extern u32 rsc_io_sched_nonpagewait_inslowpath_count;

extern u32 rsc_io_sched_bread_count;
extern u32 rsc_io_sched_bwrite_count;
extern u32 rsc_io_sched_bother_count;
extern u32 rsc_io_sched_bread_inslowpath_count;
extern u32 rsc_io_sched_bwrite_inslowpath_count;
extern u32 rsc_io_sched_bother_inslowpath_count;


#define RSC_MAX_MMAP_IO_SCHED_CNT 200
extern const u16 rsc_max_mmap_io_sched_cnt;
/*unit is 2ms*/
#define RSC_MAX_MMAP_IO_SCHED_UNIT_MS 2
extern u32 rsc_mmap_io_sched_time[RSC_MAX_MMAP_IO_SCHED_CNT];
extern u32 rsc_mmap_max_io_sched_time_ms;
extern u32 rsc_mmap_allmax_io_sched_time_ms;
extern u32 rsc_mmap_io_sched_cnt;
extern u32 rsc_io_sched_count_for_lock;
extern u32 rsc_mmap_io_sched_time_for_lock[RSC_MAX_MMAP_IO_SCHED_CNT];
extern u32 rsc_mmap_max_io_sched_time_ms_for_lock;
extern u32 rsc_io_sched_count_for_other;
extern u32 rsc_mmap_max_io_sched_time_ms_for_other;
#endif

//f2fs/file.c
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)
extern u32 rsc_unlock_mmap_for_mkwrite_cached_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_filemap_fault_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_cnt_svp;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_miss_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_filemap_fault_ptechange_cnt;
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 1)
extern u32 rsc_unlock_mmap_for_mkwrite_cached_miss_ptechange_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_miss_sizechange_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_miss_nondirty_cnt;
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
extern u32 rsc_unlock_mmap_for_mkwrite;
extern u32 rsc_unlock_mmap_for_mkwrite_svp;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_isize_err_cnt;
extern u32 rsc_unlock_mmap_for_mkwrite_cached_act_filemap_fault_ptechange_cnt;
#endif

//filemap.c
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)
extern u32 rsc_unlock_mmap_for_fault_cached_cnt;
extern u32 rsc_unlock_mmap_for_fault_cached_cnt_svp;
extern u32 rsc_unlock_mmap_for_fault_cached_uptodate_miss_cnt;
extern u32 rsc_unlock_mmap_for_fault_cached_filechange_miss_cnt;
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
extern u32 rsc_unlock_mmap_for_io_count;
extern u32 rsc_unlock_mmap_for_io_count_for_svp;
extern u32 rsc_unlock_mmap_for_io_act_count;

#define RSC_MAX_MMAP_IO_WAIT_CNT 200
extern const u16 rsc_max_mmap_io_cnt;
/*unit is 2ms*/
#define RSC_MAX_MMAP_IO_WAIT_UNIT_MS 2
extern u32 rsc_mmap_io_wait_time[RSC_MAX_MMAP_IO_WAIT_CNT];
extern u32 rsc_mmap_max_io_wait_time_ms;

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
extern atomic_t rsc_total_svp_pid;
extern u32 rsc_unlock_mmap_for_io_act_count_for_svp;
extern u32 rsc_mmap_io_wait_time_for_svp[RSC_MAX_MMAP_IO_WAIT_CNT];
extern u32 rsc_mmap_max_io_wait_time_ms_for_svp;
#endif

extern u32 rsc_retry_fault_pte_change_cnt;
extern u32 rsc_retry_fault_from_mkwrite_cnt;
extern unsigned long rsc_fault_filemap_cnt;
#endif

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE
DECLARE_PER_CPU(unsigned long, rsc_fault_cached);
DECLARE_PER_CPU(unsigned long, rsc_fault_cached_hit);
DECLARE_PER_CPU(unsigned long, rsc_mkwrite_cached);
DECLARE_PER_CPU(unsigned long, rsc_mkwrite_cached_hit);
DECLARE_PER_CPU(unsigned long, rsc_cached_release);
#endif

//memory.c
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
extern u32 rsc_page_fault_err_cnt;
extern u32 rsc_second_fault_ptechange_normal_exit_cnt;
extern u32 rsc_second_fault_install_pte_exit_cnt;
#endif

/*1 ~ 3*/
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG < 4)
u32 rsc_mmap_sem_debug = MMAP_SEM_DBG_OFF;
/*>3*/
#else
u32 rsc_mmap_sem_debug = MMAP_SEM_DBG_BRIEF|MMAP_SEM_DBG_DETAIL;
#endif
u32 rsc_io_schedule_print_ms = 10;
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG)
#if (CONFIG_RSC_DROP_MMAP_SEM_DEBUG < 2)
int rsc_io_schedule_print_group = RSC_GROUP_FG;
#elif (CONFIG_RSC_DROP_MMAP_SEM_DEBUG < 3)
int rsc_io_schedule_print_group = RSC_GROUP_DEFAULT;
#elif (CONFIG_RSC_DROP_MMAP_SEM_DEBUG < 4)
int rsc_io_schedule_print_group = RSC_GROUP_BG;
#elif (CONFIG_RSC_DROP_MMAP_SEM_DEBUG < 5)
int rsc_io_schedule_print_group = RSC_GROUP_BG;
#endif
#endif

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MADVISE
extern u32 rsc_readahead_total_cnt;
extern u32 rsc_readahead_max_vma_cnt;
extern u32 rsc_readahead_fail_cnt;
#endif

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_SHOWMAP
extern unsigned long rsc_not_drop_mmap_sem_in_showmap_cnt;
DECLARE_PER_CPU(unsigned long, rsc_drop_mmap_sem_in_showmap_get_vma_cnt);
DECLARE_PER_CPU(unsigned long, rsc_drop_mmap_sem_in_showmap_put_vma_cnt);

extern atomic_t rsc_drop_mmap_sem_in_showmap_page_cnt;

extern atomic_t  rsc_drop_mmap_sem_in_showmap_cnt;
extern atomic_t rsc_drop_mmap_sem_in_showmap_release_next_cnt;
extern atomic_t rsc_drop_mmap_sem_in_showmap_release_stop_cnt;

extern atomic_t rsc_drop_mmap_sem_in_showmap_reentry_cnt;

extern u32 rsc_drop_mmap_sem_in_showmap_vma_addr_change;
extern u32 rsc_drop_mmap_sem_in_showmap_find_last_addr_fail;

extern atomic_t rsc_drop_mmap_sem_in_showmap_next_vma_cache;

extern u32 rsc_drop_mmap_sem_in_showmap_checkvma_error;
extern u32 rsc_drop_mmap_sem_in_showmap_checkvma_destory;
#endif

enum {
	RSC_DIS_DROP_SEM_IN_FILEMAP				= 1 << 0,//1
	RSC_DIS_DROP_SEM_IN_MKWRITE				= 1 << 1,//2
	RSC_DIS_DROP_SEM_IN_MADVISE				= 1 << 2,//4
	RSC_DIS_DROP_SEM_IN_SHOWMAP				= 1 << 3,//8
	RSC_DIS_DROP_SEM_IN_GPU					= 1 << 4,//16
 };
bool rsc_disable_drop_sem_in_filemap_fault;
bool rsc_disable_drop_sem_in_mkwrite;
bool rsc_disable_drop_sem_in_madvise;
bool rsc_disable_drop_sem_in_showmap;
bool rsc_disable_drop_sem_in_gpu_driver;

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_GPU_DRIVER
extern unsigned long rsc_drop_mmap_sem_in_gpu_cnt;
extern unsigned long rsc_drop_mmap_sem_fail_con_in_gpu_cnt;
extern u32 rsc_drop_mmap_sem_fail_no_vma_in_gpu_cnt;
extern u32 rsc_drop_mmap_sem_fail_addr_err_in_gpu_cnt;

extern unsigned long rsc_drop_mmap_sem_in_syncset_gpu_cnt;
extern unsigned long rsc_drop_mmap_sem_fail_con_in_syncset_gpu_cnt;
extern u32 rsc_drop_mmap_sem_fail_no_vma_in_syncset_gpu_cnt;
extern u32 rsc_drop_mmap_sem_fail_addr_err_in_syncset_gpu_cnt;
extern u32 rsc_drop_mmap_sem_trylock_fail_in_syncset_gpu_cnt;

extern u32 rsc_drop_mmap_sem_in_gpu_checkvma_error;
extern u32 rsc_drop_mmap_sem_in_gpu_checkvma_destory;

extern u32 rsc_drop_mmap_sem_in_gpu_commit_nolock_cnt;
extern u32 rsc_drop_mmap_sem_in_gpu_commit_writelock_cnt;
#endif

#define PER_LINE_COUNT 20
static ssize_t show_mmap_sem_info(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0) && defined(CONFIG_RSC_CHECK_MMAP_SEM)
	int i;
	struct task_struct *g, *p;
#endif
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE
	unsigned long fault_cached = 0;
	unsigned long fault_cached_hit = 0;
	unsigned long mkwrite_cached = 0;
	unsigned long mkwrite_cached_hit = 0;
	unsigned long cached_release = 0;
	unsigned long fault_map_cached_hit = 0;
	int cpu;
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_SHOWMAP
	unsigned long getvma = 0;
	unsigned long putvma = 0;
#endif
#endif

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"disable fmap%d mkw%d madv%d smap%d gpu%d\n",
		rsc_disable_drop_sem_in_filemap_fault,
		rsc_disable_drop_sem_in_mkwrite,
		rsc_disable_drop_sem_in_madvise,
		rsc_disable_drop_sem_in_showmap,
		rsc_disable_drop_sem_in_gpu_driver);
	ret += cur;

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE
	for_each_possible_cpu(cpu) {
		fault_cached += per_cpu(rsc_fault_cached, cpu);
		fault_cached_hit += per_cpu(rsc_fault_cached_hit, cpu);
		mkwrite_cached += per_cpu(rsc_mkwrite_cached, cpu);
		mkwrite_cached_hit += per_cpu(rsc_mkwrite_cached_hit, cpu);
		cached_release += per_cpu(rsc_cached_release, cpu);
		fault_map_cached_hit = 0;
		//fault_map_cached_hit += per_cpu(rsc_fault_map_cached_hit, cpu);
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_SHOWMAP
		getvma += per_cpu(rsc_drop_mmap_sem_in_showmap_get_vma_cnt, cpu);
		putvma += per_cpu(rsc_drop_mmap_sem_in_showmap_put_vma_cnt, cpu);
#endif
	}

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"cached filemap: %lu mkw: %lu fmaph: %lu fh: %lu mkh: %lu rl: %lu toc: %lu toh: %lu toc-toh: %ld "
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)
		"mkwptecg: %u "
#endif
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
		"%u "
#endif
		"DEBUG%d: %u ioprintms: %u"
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MADVISE
		" madvise: %u fail: %u maxvma: %u"
#endif
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_SHOWMAP
		" showmap: %lu vma g: %lu p: %lu get-put: %ld"
		" drop: page %u %u (rel: %u + %u ck %d) reentry: %u"
		" addrcg: %u faf: %u"
		" cache: %u vma des: %u err: %u"
#endif
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_GPU_DRIVER
		" gpu drop: %lu fail: %lu %u %u"
		" syncset drop: %lu fail: %lu %u %u"
		" des: %u err: %u"
		" commit lock: %u nl: %u"
		" tlf: %u"
#endif
		"\n",
		fault_cached,
		mkwrite_cached,
		fault_map_cached_hit,
		fault_cached_hit,
		mkwrite_cached_hit,
		cached_release,
		fault_cached + mkwrite_cached,
		fault_map_cached_hit + fault_cached_hit + cached_release,
		(fault_cached + mkwrite_cached) -
		(fault_map_cached_hit + fault_cached_hit + cached_release),
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)
		rsc_unlock_mmap_for_mkwrite_cached_filemap_fault_ptechange_cnt,
#endif
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
		rsc_unlock_mmap_for_mkwrite_cached_act_filemap_fault_ptechange_cnt,
#endif
		CONFIG_RSC_DROP_MMAP_SEM_DEBUG,
		rsc_mmap_sem_debug, rsc_io_schedule_print_ms
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_MADVISE
		, rsc_readahead_total_cnt, rsc_readahead_fail_cnt, rsc_readahead_max_vma_cnt
#endif
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_SHOWMAP
		, rsc_not_drop_mmap_sem_in_showmap_cnt
		, getvma, putvma, getvma - putvma

		, atomic_read(&rsc_drop_mmap_sem_in_showmap_page_cnt)
		, atomic_read(&rsc_drop_mmap_sem_in_showmap_cnt)
		, atomic_read(&rsc_drop_mmap_sem_in_showmap_release_next_cnt)
		, atomic_read(&rsc_drop_mmap_sem_in_showmap_release_stop_cnt)
		, atomic_read(&rsc_drop_mmap_sem_in_showmap_cnt) -
			(
			atomic_read(&rsc_drop_mmap_sem_in_showmap_release_next_cnt) +
			atomic_read(&rsc_drop_mmap_sem_in_showmap_release_stop_cnt)
			)
		, atomic_read(&rsc_drop_mmap_sem_in_showmap_reentry_cnt)

		, rsc_drop_mmap_sem_in_showmap_vma_addr_change
		, rsc_drop_mmap_sem_in_showmap_find_last_addr_fail

		, atomic_read(&rsc_drop_mmap_sem_in_showmap_next_vma_cache)
		, rsc_drop_mmap_sem_in_showmap_checkvma_destory
		, rsc_drop_mmap_sem_in_showmap_checkvma_error
#endif
#ifdef CONFIG_RSC_DROP_MMAP_SEM_IN_GPU_DRIVER
		, rsc_drop_mmap_sem_in_gpu_cnt
		, rsc_drop_mmap_sem_fail_con_in_gpu_cnt
		, rsc_drop_mmap_sem_fail_no_vma_in_gpu_cnt
		, rsc_drop_mmap_sem_fail_addr_err_in_gpu_cnt

		, rsc_drop_mmap_sem_in_syncset_gpu_cnt
		, rsc_drop_mmap_sem_fail_con_in_syncset_gpu_cnt
		, rsc_drop_mmap_sem_fail_no_vma_in_syncset_gpu_cnt
		, rsc_drop_mmap_sem_fail_addr_err_in_syncset_gpu_cnt

		, rsc_drop_mmap_sem_in_gpu_checkvma_destory
		, rsc_drop_mmap_sem_in_gpu_checkvma_error

		, rsc_drop_mmap_sem_in_gpu_commit_writelock_cnt
		, rsc_drop_mmap_sem_in_gpu_commit_nolock_cnt

		, rsc_drop_mmap_sem_trylock_fail_in_syncset_gpu_cnt
#endif
		);
	ret += cur;
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0) && defined(CONFIG_RSC_CHECK_MMAP_SEM)
	if (rsc_io_sched_count + rsc_io_sched_write_count) {
		int j;

		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"mmap gbsched %u %u tm %u %u maxtime %u io %u lk %u ot %u ms wait read(slp): %u (%u) "
			"ww: %u (%u) lk: %u (%u) ow: %u (%u) npw: %u (%u) ot: %u vdso c: %u d: %u ocg: %u "
			"dmbuf read(slp): %u (%u) ww: %u (%u) ot %u (%u)\n",
			rsc_io_sched_count, rsc_io_sched_write_count,
			rsc_io_sched_to_count, rsc_io_sched_to_write_count,
			rsc_mmap_allmax_io_sched_time_ms,
			rsc_mmap_max_io_sched_time_ms,
			rsc_mmap_max_io_sched_time_ms_for_lock,
			rsc_mmap_max_io_sched_time_ms_for_other,
			rsc_io_sched_waitread_count, rsc_io_sched_waitread_inslowpath_count,
			rsc_io_sched_waitwrite_count, rsc_io_sched_waitwrite_inslowpath_count,
			rsc_io_sched_waitlock_count, rsc_io_sched_waitlock_inslowpath_count,
			rsc_io_sched_otherwait_count, rsc_io_sched_otherwait_inslowpath_count,
			rsc_io_sched_nonpagewait_count, rsc_io_sched_nonpagewait_inslowpath_count,
			rsc_io_sched_count_for_other,
			rsc_io_sched_waitvdsoc_count, rsc_io_sched_waitvdsod_count,
			rsc_io_sched_waituptodatenocg_count,

			rsc_io_sched_bread_count, rsc_io_sched_bread_inslowpath_count,
			rsc_io_sched_bwrite_count, rsc_io_sched_bwrite_inslowpath_count,
			rsc_io_sched_bother_count, rsc_io_sched_bother_inslowpath_count
		);
		ret += cur;

		if (rsc_mmap_io_sched_cnt) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"mmap iosched %u (r %u + w %u) maxtime %u ms\n",
				rsc_mmap_io_sched_cnt, rsc_io_sched_waitread_count,
				rsc_io_sched_waitwrite_count, rsc_mmap_max_io_sched_time_ms);
			ret += cur;
			for (i = 0; i < rsc_max_mmap_io_sched_cnt/PER_LINE_COUNT; i++) {
				cur = snprintf(buf + ret, PAGE_SIZE - ret,
						"%03u:\t", i*PER_LINE_COUNT*RSC_MAX_MMAP_IO_SCHED_UNIT_MS);
				ret += cur;
				for (j = 0; j < PER_LINE_COUNT; j++) {
					if (rsc_mmap_io_sched_time[i*PER_LINE_COUNT+j])
						break;
				}
				if (j >= PER_LINE_COUNT) {
					cur = snprintf(buf + ret, PAGE_SIZE - ret,
					"ZERO\n");
					ret += cur;
				} else {
					for (j = 0; j < PER_LINE_COUNT; j++) {
						cur = snprintf(buf + ret, PAGE_SIZE - ret,
							"%3u ", rsc_mmap_io_sched_time[i*PER_LINE_COUNT+j]);
						ret += cur;
					}
					cur = snprintf(buf + ret, PAGE_SIZE - ret,
						"\n");
					ret += cur;
				}
			}
		}

		if (rsc_io_sched_count_for_lock) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"mmap lksched %u maxtime %u ms\n",
				rsc_io_sched_count_for_lock, rsc_mmap_max_io_sched_time_ms_for_lock);
			ret += cur;
			for (i = 0; i < rsc_max_mmap_io_sched_cnt/PER_LINE_COUNT; i++) {
				cur = snprintf(buf + ret, PAGE_SIZE - ret,
						"%03u:\t", i*PER_LINE_COUNT*RSC_MAX_MMAP_IO_SCHED_UNIT_MS);
				ret += cur;
				for (j = 0; j < PER_LINE_COUNT; j++) {
					if (rsc_mmap_io_sched_time_for_lock[i*PER_LINE_COUNT+j])
						break;
				}
				if (j >= PER_LINE_COUNT) {
					cur = snprintf(buf + ret, PAGE_SIZE - ret,
					"ZERO\n");
					ret += cur;
				} else {
					for (j = 0; j < PER_LINE_COUNT; j++) {
						cur = snprintf(buf + ret, PAGE_SIZE - ret,
							"%3u ", rsc_mmap_io_sched_time_for_lock[i*PER_LINE_COUNT+j]);
						ret += cur;
					}
					cur = snprintf(buf + ret, PAGE_SIZE - ret,
						"\n");
					ret += cur;
				}
			}
		}
	}

	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (p->mmap_sem_read_cnt) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%03d ms%d "
				"uid %5d %16s %5d(%16s %5d)\n",
				i, p->mmap_sem_read_cnt,
				task_uid(p).val, p->comm, p->pid,
				p->group_leader?p->group_leader->comm:"NULL",
				p->group_leader?p->group_leader->pid:99999);
			ret += cur;
			i++;
		}
	}
	rcu_read_unlock();
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"filecache: %u hit fh: %u %u miss utd: %u fcg: %u "
			"mkwcache: %u hit mw: %u %u fmh: %u miss: %u"
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 1)
			" ndirty: %u ptecg: %u sc: %u"
#endif
			"\n",
			fault_cached,
			rsc_unlock_mmap_for_fault_cached_cnt, rsc_unlock_mmap_for_fault_cached_cnt_svp,
			rsc_unlock_mmap_for_fault_cached_uptodate_miss_cnt, rsc_unlock_mmap_for_fault_cached_filechange_miss_cnt,
			mkwrite_cached,
			rsc_unlock_mmap_for_mkwrite_cached_cnt, rsc_unlock_mmap_for_mkwrite_cached_cnt_svp,
			rsc_unlock_mmap_for_mkwrite_cached_filemap_fault_cnt,
			rsc_unlock_mmap_for_mkwrite_cached_miss_cnt
#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 1)
			, rsc_unlock_mmap_for_mkwrite_cached_miss_nondirty_cnt
			, rsc_unlock_mmap_for_mkwrite_cached_miss_ptechange_cnt
			, rsc_unlock_mmap_for_mkwrite_cached_miss_sizechange_cnt
#endif
			);
		ret += cur;
#endif

#if defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 2)
	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"unlock_mmap mw: %u %u\n",
		rsc_unlock_mmap_for_mkwrite, rsc_unlock_mmap_for_mkwrite_svp
		);
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"mkw sizeerr: %u rtptecg: %u fmw: %u faultaroundmaps: %lu "
		"pferr: %u secpfptecg: %u secpginspte: %u\n",
		rsc_unlock_mmap_for_mkwrite_cached_isize_err_cnt,
		rsc_retry_fault_pte_change_cnt,
		rsc_retry_fault_from_mkwrite_cnt,
		rsc_fault_filemap_cnt,
		rsc_page_fault_err_cnt,
		rsc_second_fault_ptechange_normal_exit_cnt,
		rsc_second_fault_install_pte_exit_cnt
		);
	ret += cur;
#endif

	return ret;
}

/*
* example:
* disable drop mmap_sem
* echo 0x1f > /sys/rsc/mmap_sem
* only allow io or io_schedule time >= 20 ms to printk output.
* echo 0 1 3 20 > /sys/rsc/mmap_sem
*/
static ssize_t store_mmap_sem_info(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, gp;
	unsigned int debug, printms;
	u32 disable;

	ret = sscanf(buf, "%x %u %d %u", &disable, &debug, &gp, &printms);

	if (ret != 1 && ret != 4)
		goto fail;

	rsc_disable_drop_sem_in_filemap_fault = !!(disable & RSC_DIS_DROP_SEM_IN_FILEMAP);
	rsc_disable_drop_sem_in_mkwrite = !!(disable & RSC_DIS_DROP_SEM_IN_MKWRITE);
	rsc_disable_drop_sem_in_madvise = !!(disable & RSC_DIS_DROP_SEM_IN_MADVISE);
	rsc_disable_drop_sem_in_showmap = !!(disable & RSC_DIS_DROP_SEM_IN_SHOWMAP);
	rsc_disable_drop_sem_in_gpu_driver = !!(disable & RSC_DIS_DROP_SEM_IN_GPU);
	if (ret == 4) {
		rsc_mmap_sem_debug = debug;
		rsc_io_schedule_print_group = gp;
		rsc_io_schedule_print_ms = printms;
	}
	rsc_info("%s: disable: 0x%x debug = %u gp = %d printms = %u\n", __func__,
		disable, rsc_mmap_sem_debug, rsc_io_schedule_print_group, rsc_io_schedule_print_ms);

	return count;

fail:
	rsc_err("usage: echo xxx > /sys/rsc/mmap_sem\n");

	return -EINVAL;
}

static struct kobj_attribute rsc_mmap_sem_info_attr =
__ATTR(mmap_sem, 0660, show_mmap_sem_info, store_mmap_sem_info);

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IO_DECAY
/*
* 0 is not decay.
* percentage of zoom add.
*/
u32 __read_mostly rsc_drop_mmap_sem_io_decay_zoom;

/*
* 0 only for svp process, 1 for full process
*/
u32 rsc_drop_mmap_sem_io_decay_range;

/*
* delay time(ms) when do mkwrite.
*/
u32 rsc_drop_mmap_sem_io_decay_fix_plus_ms;

u64 rsc_drop_mmap_sem_io_decay_mkwrite_svp_cnt;
u64 rsc_drop_mmap_sem_io_decay_mkwrite_svp_total_us;
u64 rsc_drop_mmap_sem_io_decay_mkwrite_total_cnt;
u64 rsc_drop_mmap_sem_io_decay_mkwrite_total_us;

u64 rsc_drop_mmap_sem_io_decay_madvise_svp_cnt;
u64 rsc_drop_mmap_sem_io_decay_madvise_svp_total_us;
u64 rsc_drop_mmap_sem_io_decay_madvise_total_cnt;
u64 rsc_drop_mmap_sem_io_decay_madvise_total_us;

/*
* io_decay in mkwrite or madvise
* for example, io_decay 50ms in mkwrite
* echo 0xDEADBEEF 50 > /sys/rsc/io_decay
*/
static ssize_t store_io_decay(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	u32 fix_plus, zoom, range;
	u32 magic;
	int ret;

	ret = sscanf(buf, "%x %u %u %u", &magic, &fix_plus, &zoom, &range);

	if (ret < 2 && ret > 4)
		goto fail;
	if (magic != 0xDEADBEEF)
		goto fail;
	rsc_drop_mmap_sem_io_decay_fix_plus_ms = fix_plus;
	if (ret >= 3)
		rsc_drop_mmap_sem_io_decay_zoom = zoom;
	if (ret >= 4)
		rsc_drop_mmap_sem_io_decay_range = range;

	if (fix_plus) {
		if (!rsc_drop_mmap_sem_io_decay_zoom)
			rsc_drop_mmap_sem_io_decay_zoom = 1;
	}

	return count;

fail:
	rsc_err("usage: echo 0xDEADBEEF 50 > /sys/rsc/io_decay\n");

	return -EINVAL;
}

static ssize_t show_io_decay(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	u32 zoom = rsc_drop_mmap_sem_io_decay_zoom;
	u32 range = rsc_drop_mmap_sem_io_decay_range;

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"fixplus %u ms zoom %7s +%2u%% range for %4s %u\n",
		rsc_drop_mmap_sem_io_decay_fix_plus_ms,
		zoom ? "enable" : "disable",
		zoom,
		range ? "full" : "svp",
		range
		);
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"mkwrite svpcnt %llu elapse %llu us totalcnt %llu elapse %llu us\n"
		"madvise svpcnt %llu elapse %llu us totalcnt %llu elapse %llu us\n",
		rsc_drop_mmap_sem_io_decay_mkwrite_svp_cnt,
		rsc_drop_mmap_sem_io_decay_mkwrite_svp_total_us,
		rsc_drop_mmap_sem_io_decay_mkwrite_total_cnt,
		rsc_drop_mmap_sem_io_decay_mkwrite_total_us,
		rsc_drop_mmap_sem_io_decay_madvise_svp_cnt,
		rsc_drop_mmap_sem_io_decay_madvise_svp_total_us,
		rsc_drop_mmap_sem_io_decay_madvise_total_cnt,
		rsc_drop_mmap_sem_io_decay_madvise_total_us
		);
	ret += cur;

	return ret;
}
static struct kobj_attribute rsc_io_decay_attr =
__ATTR(io_decay, 0660, show_io_decay, store_io_decay);
#endif
#endif

/**
 * purpose: create sysfs nodes for module
 * arguments:
 *    none
 * return:
 *    kobject : for future destory.
 */
static struct kobject *rsc_sysfs_create(void)
{
	int err, i;
	struct kobject *kobj = NULL;

	kobj = kobject_create_and_add("svp", rsc_root_dir);
	if (!kobj) {
		rsc_err("svp failed to create sysfs node.\n");
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(rsc_svp_attrs) - 1; i++) {
		/* create sysfs */
		err = sysfs_create_file(kobj, rsc_svp_attrs[i]);
		rsc_chown_to_system(kobj, rsc_svp_attrs[i]);
	}

	if (err) {
		rsc_err("svp failed to create sysfs attrs.\n");
		kobject_put(kobj);
		return NULL;
	}
	return kobj;
}
#endif
static int __init rsc_settting_init(void)
{
	long ret;
	int i;
	BUILD_BUG_ON(ENABLE_BINDER_LIMIT_NORMAL && ENABLE_BINDER_LIMIT_AGGRESS);

	ret = sysfs_create_file(rsc_root_dir, &rsc_switch_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_switch_attr.attr);

	ret = sysfs_create_file(rsc_root_dir, &rsc_svp_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_svp_attr.attr);
	for (i = 0; i < RSC_MAX_SVP_PID; i++) {
		struct rsc_svp_entry *tpid_entry;

		tpid_entry = &svp_tentry[i];
		list_add_tail(&tpid_entry->link, &tpid_free_head);
	}
	profile_event_register(PROFILE_TASK_EXIT, &rsc_svp_notifier_block);

#ifdef CONFIG_RSC_APP_LAUNCH_BOOST
	ret = sysfs_create_file(rsc_root_dir, &rsc_appboost_late_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_appboost_late_attr.attr);

	ret = sysfs_create_file(rsc_root_dir, &rsc_launch_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_launch_attr.attr);

#endif

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	ret = sysfs_create_file(rsc_root_dir, &rsc_sig_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_sig_attr.attr);
#endif

#ifdef CONFIG_RSC_KSWAPD_FORCE_SLEEP
	rsc_sysfs_create();
#endif

#if (defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)) || defined(CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE)
	ret = sysfs_create_file(rsc_root_dir, &rsc_mmap_sem_info_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_mmap_sem_info_attr.attr);
#endif

#ifdef CONFIG_RSC_DROP_MMAP_SEM_IO_DECAY
	ret = sysfs_create_file(rsc_root_dir, &rsc_io_decay_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_io_decay_attr.attr);
#endif

fail:

	return 0;
}

late_initcall(rsc_settting_init);
