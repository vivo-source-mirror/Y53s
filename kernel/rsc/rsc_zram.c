#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/hugetlb.h>
#include <linux/huge_mm.h>
#include <asm/tlbflush.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>

#include "rsc_zram.h"

/* Globals */

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
/*this is need to adjust by platform, for sdm660 it is 10%*/
#ifdef CONFIG_ARCH_SDM845
#define ZRAM_COMPRESS_CPU_USAGE_PERCENT	5
#else
#define ZRAM_COMPRESS_CPU_USAGE_PERCENT	10
#endif
atomic_t rsc_zram_load = ATOMIC_INIT(0);
atomic_t rsc_zram_load_val[RSC_ZRAM_LOAD_NUM];
int rsc_zram_idle_threshold = RSC_ZRAM_IDLE_THRESHOLD;
#if defined(CONFIG_MTK_PLATFORM)
u32 RSC_ZRAM_TICK_CNT[RSC_ZRAM_LOAD_NUM] = {RSC_ZRAM_TICK_CNT_CONST, RSC_ZRAM_TICK_CNT_CONST};
int  rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT_CONST * NR_CPUS *
							RSC_ZRAM_IDLE_THRESHOLD) / 100;
seqcount_t rsc_zram_load_seq_lock;
#else
int __read_mostly rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT * NR_CPUS *
							RSC_ZRAM_IDLE_THRESHOLD) / 100;
#endif
atomic_t rsc_zram_wait_condition = ATOMIC_INIT(RSC_ZRAM_WAIT_CON_ACTION);
atomic_t rsc_zram_period_cnt = ATOMIC_INIT(0);
int __read_mostly rsc_zram_enable;

DECLARE_WAIT_QUEUE_HEAD(rsc_zram_wait);
#endif

static struct rsc_zram_module rsc_zram_module;
unsigned long cur_nr_free_pages;
long rsc_zram_swap_min_free;
int rsc_zram_anon_pages_max = RSC_ZRAM_ANON_PAGE_MAX;
atomic_long_t rsc_zram_decompress_time;
atomic_long_t rsc_zram_decompress_pages;
int __read_mostly rsc_zram_speed_enable;
static DEFINE_MUTEX(rsc_zram_lock);

/*low ram project need adjust RSC_ZRAM_FREE_PAGE_MIN*/
#if 1
/*2GB ram is 130MB, 4GB ram is 320MB*/
unsigned long RSC_ZRAM_FREE_PAGE_MIN = ((160*1024*1024)>>(PAGE_SHIFT));
void rsc_set_zram_free_min(void)
{
	rsc_zram_module.free_pages_min = (int)RSC_ZRAM_FREE_PAGE_MIN;
}
#endif
static inline unsigned long elapsed_jiffies(unsigned long start)
{
	unsigned long end = jiffies;

	if (end >= start)
		return (unsigned long)(end - start);

	return (unsigned long)(end + (MAX_JIFFY_OFFSET - start) + 1);
}

static bool is_swap_full(int max_percent)
{
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
/*for good performance*/
	long total = total_swap_pages;
	long nrfree = atomic_long_read(&nr_swap_pages);
	long used;

	used = total - nrfree;
	if (used < 0)
		return true;

	if (used * 100 >= total * max_percent)
		return true;

	return false;
#else
	struct sysinfo si;

	si_swapinfo(&si);
	if (si.totalswap == 0)
		return true;
	/* free/total > max_percent% */
	if ((si.totalswap - si.freeswap) * 100 >= si.totalswap * max_percent)
		return true;

	return false;
#endif
}

static bool is_memory_free_enough(int free_pages_min)
{
	unsigned long nr_free_pages;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
	nr_free_pages = global_zone_page_state(NR_FREE_PAGES);
#else
	nr_free_pages = global_page_state(NR_FREE_PAGES);
#endif
	cur_nr_free_pages = nr_free_pages;
	if (nr_free_pages > free_pages_min)
		return true;
	return false;
}

static bool is_anon_page_enough(int anon_pages_min)
{
	unsigned long nr_pages;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	nr_pages = global_node_page_state(NR_INACTIVE_ANON);
	nr_pages += global_node_page_state(NR_ACTIVE_ANON);
#else
	nr_pages = global_page_state(NR_INACTIVE_ANON);
	nr_pages += global_page_state(NR_ACTIVE_ANON);
#endif
	if (nr_pages > anon_pages_min)
		return true;

	return false;
}

static inline void rsc_zram_set_full_clean(struct rsc_zram_module *rcc, int set)
{
	atomic_set(&rcc->full_clean_flag, !!set);
	/*rsc_info("rzram full_clean_flag = %d\n", atomic_read(&rcc->full_clean_flag));*/
}

static inline bool is_display_off(struct rsc_zram_module *rcc)
{
	return rcc->display_off;
}

static void rsc_zram_update_display_stat(struct rsc_zram_module *rcc, bool display_on)
{
	rcc->display_off = !display_on;
	rsc_info("rzram display_off = %d\n", rcc->display_off);
}

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
/**
 * purpose: check cpu is really in idle status
 * arguments:
 *    rcc: struct rsc_zram_module. we use rsc_zram_idle_* values here.
 * return:
 *    none.
 */
static inline bool is_cpu_idle(struct rsc_zram_module *rcc, bool addcompress)
{
	int threshold;	
	int cur_load_val[RSC_ZRAM_LOAD_NUM];	
#if defined(CONFIG_MTK_PLATFORM)
	int thrload[RSC_ZRAM_LOAD_NUM];
	unsigned int seq;
#else
	int thrload;
#endif
	static unsigned int glcnt, overloadcnt;

	threshold = rsc_zram_idle_threshold;
	/* when display off, we try do more work. */
	if (rcc->display_off)
		threshold += 5;

	glcnt++;
	/*compress the anon memory need cpu!*/
	if (addcompress)
		threshold += ZRAM_COMPRESS_CPU_USAGE_PERCENT;
#if defined(CONFIG_MTK_PLATFORM)
	do {
		seq = read_seqcount_begin(&rsc_zram_load_seq_lock);
		thrload[0] = (RSC_ZRAM_TICK_CNT[0] * NR_CPUS *
						threshold) / 100;
		thrload[1] = (RSC_ZRAM_TICK_CNT[1] * NR_CPUS *
						threshold) / 100;
		cur_load_val[0] = atomic_read(&rsc_zram_load_val[0]) ;
		cur_load_val[1] = atomic_read(&rsc_zram_load_val[1]) ;
	} while (read_seqcount_retry(&rsc_zram_load_seq_lock, seq));
	if ((cur_load_val[0] <= thrload[0]) &&
		(cur_load_val[1] <= thrload[1]))
		return true;
#else
	thrload = (RSC_ZRAM_TICK_CNT * NR_CPUS *
					threshold) / 100;
	if ((atomic_read(&rsc_zram_load_val[0]) <= thrload) &&
		(atomic_read(&rsc_zram_load_val[1]) <= thrload))
		return true;
	cur_load_val[0] = atomic_read(&rsc_zram_load_val[0]) ;
	cur_load_val[1] = atomic_read(&rsc_zram_load_val[1]) ;
#endif

	if ((rsc_debug & RSC_ZRAM) && !(overloadcnt % 1)) {
		int percent[RSC_ZRAM_LOAD_NUM];

#if defined(CONFIG_MTK_PLATFORM)
		percent[0] =
			cur_load_val[0] * 100 / (RSC_ZRAM_TICK_CNT[0] * NR_CPUS);
		percent[1] =
			cur_load_val[1] * 100 / (RSC_ZRAM_TICK_CNT[1] * NR_CPUS);
#else
		percent[0] =
			cur_load_val[0] * 100 / (RSC_ZRAM_TICK_CNT * NR_CPUS);
		percent[1] =
			cur_load_val[1] * 100 / (RSC_ZRAM_TICK_CNT * NR_CPUS);
#endif
		rsc_info("rzram %5u load s %2d %2d/%3d per %2d %2d >= thr %2d overcnt %3u\n",
			glcnt, atomic_read(&rsc_zram_load_val[0]),
			atomic_read(&rsc_zram_load_val[1]),
#if defined(CONFIG_MTK_PLATFORM)
			((int)RSC_ZRAM_TICK_CNT[0] * NR_CPUS),
#else
			(RSC_ZRAM_TICK_CNT * NR_CPUS),
#endif
			percent[0], percent[1],
			threshold, overloadcnt);
	}
	overloadcnt++;

	return false;
}
#if 0
/**
 *  purpose: wrapper of _get_cpu_load()
 * return:
 *    cpu load in percent.
 */
static int get_cpu_load(struct rsc_zram_module *rcc, bool checkstat)
{
	int percent;
	int loadval;
	static unsigned int glcnt, overloadcnt;

	loadval = rsc_zram_load_val;

	percent = loadval * 100 / (RSC_ZRAM_TICK_CNT * NR_CPUS);
	glcnt++;
	if (percent >= rcc->idle_threshold) {
		if (!(overloadcnt % 1))
			rsc_info("rzram %5u load s %2d/%3d per %3d >= thr %2d overcnt %3u\n",
				glcnt, rsc_zram_load_val, (RSC_ZRAM_TICK_CNT * NR_CPUS),
				percent, rcc->idle_threshold, overloadcnt);
		overloadcnt++;
	}

	rcc->cpu_load[0] = percent;

	return percent;
}
#endif
#else
/**
 * purpose: check cpu is really in idle status
 * arguments:
 *    rcc: struct rsc_zram_module. we use rsc_zram_idle_* values here.
 * return:
 *    none.
 */
static inline bool is_cpu_idle(struct rsc_zram_module *rcc)
{
	int threshold = rcc->idle_threshold;

	/* when display off, we try do more work. */
	if (rcc->display_off)
		threshold += 5;

	if (rcc->cpu_load[0] <= threshold
	    && rcc->cpu_load[1] <= threshold
	    && rcc->cpu_load[2] <= threshold) {
		return true;
	}
	return false;
}

#ifdef arch_idle_time

static cputime64_t get_idle_time(int cpu)
{
	cputime64_t idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

#else

static u64 get_idle_time(int cpu)
{
	u64 idle, idle_time = -1ULL;

	if (cpu_online(cpu))
		idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		/* !NO_HZ or cpu offline so we can rely on cpustat.idle */
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = usecs_to_cputime64(idle_time);

	return idle;
}

#endif

#define RSC_NEG_VAL -1
/**
 *  purpose: get average cpuload from last stat
 * return:
 *    cpu load in percent.
 */
static int _get_cpu_load(struct rsc_zram_module *rcc, clock_t *last_cpu_stat)
{
	int i;
	clock_t stat[IDX_CPU_MAX], tmp, sum = 0;
	u64 cpustat[IDX_CPU_MAX] = { 0, 0, 0 };
	int percent;
	static unsigned int glcnt, overloadcnt;

	for_each_possible_cpu(i) {
		cpustat[IDX_CPU_USER] += kcpustat_cpu(i).cpustat[CPUTIME_USER];
		cpustat[IDX_CPU_SYSTEM] +=
		    kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		cpustat[IDX_CPU_IDLE] += get_idle_time(i);
	}

	for (i = 0; i < IDX_CPU_MAX; i++) {
		tmp = cputime64_to_clock_t(cpustat[i]);
		stat[i] = tmp - last_cpu_stat[i];
		last_cpu_stat[i] = tmp;
	}
	sum += stat[IDX_CPU_USER] + stat[IDX_CPU_SYSTEM] + stat[IDX_CPU_IDLE];

	if (sum == 0)
		return RSC_NEG_VAL;

	percent = 100 * (stat[IDX_CPU_USER] + stat[IDX_CPU_SYSTEM]) / sum;
	glcnt++;
	if (percent >= rcc->idle_threshold) {
		if (!(overloadcnt % 100))
			rsc_info("rzram %5u load usr %2ld sys %2ld idle %2ld s %3ld per %3d >= thr %2d overcnt %3u\n",
				glcnt, stat[IDX_CPU_USER], stat[IDX_CPU_SYSTEM], stat[IDX_CPU_IDLE],
				sum, percent, rcc->idle_threshold, overloadcnt);
		overloadcnt++;
	}

	return percent;
}

/**
 *  purpose: wrapper of _get_cpu_load()
 * return:
 *    cpu load in percent.
 */
static int get_cpu_load(struct rsc_zram_module *rcc, bool need_delay)
{
	int cpu_load;
	/* delay some time for calc recent cpu load. */
	if (need_delay) {
		_get_cpu_load(rcc, rcc->last_cpu_stat);
		msleep(25);
	}

	cpu_load = _get_cpu_load(rcc, rcc->last_cpu_stat);
	if (cpu_load >= 0) {
		rcc->cpu_load[2] = rcc->cpu_load[1];
		rcc->cpu_load[1] = rcc->cpu_load[0];
		rcc->cpu_load[0] = cpu_load;
	}
	return cpu_load;
}
#endif
/**
 * purpose: swap out memory.
 * return count of reclaimed pages.
 */
static int rsc_zram_swap_out(int nr_pages, int scan_mode)
{
	int unit_pages, total, real = 0;
	static unsigned int swap_cnt;
#if defined(CONFIG_MACH_MT6765)
	struct cpumask new_cpu_mask;
#endif

	for (total = 0; total < nr_pages; total += 32) {
		unit_pages =
		    ((total + 32) > nr_pages) ? (nr_pages - total) : 32;
		real += rsc_try_to_free_pages(unit_pages, scan_mode);
		cond_resched();
	}
	if (!(swap_cnt % 40)) {
		rsc_info("rzram %5u swap mode %d %3d/%3d pages tl %4ld\n", swap_cnt, scan_mode, real,
			nr_pages, M(get_nr_swap_pages()));
#if defined(CONFIG_MACH_MT6765)
		/*
		* 6765 cpu4-7 is little core, when enter suspend, system will shut down cpu1-7, then
		* rsc_zram cpus_allowed will fallback to 0 - 7 and it will occupy the bigcore
		*/
		if (!cpumask_empty(&rsc_cpu_littlecore)) {
			long sched_ret;

			if (cpumask_and(&new_cpu_mask, cpu_active_mask, &rsc_cpu_littlecore)
					&& (!cpumask_equal(&current->cpus_allowed, &new_cpu_mask))) {
				sched_ret = sched_setaffinity(current->pid, &new_cpu_mask);
				rsc_info("rzram bind cpu: %*pbl ret: %ld\n", cpumask_pr_args(&new_cpu_mask), sched_ret);
			}
		} else {
			rsc_err("rzram rsc_cpu_littlecore is empty!\n");
		}
#endif
	}
	swap_cnt++;
	/* return real reclaimed page count */
	return real;
}

/* purpose: check is background thread running */
static inline bool rsc_zram_is_enabled(struct rsc_zram_module *rcc)
{
	return !!rcc->task;
}

/**
 * purpose: check system is ready to wakeup compress thread.
 *    now we check: enable & idle & memory_full & swap_full.
 * arguments:
 *    rcc: struct rsc_zram_module.
 *    end: start condition checker or end condition checker.
 * return:
 *    thread exit code.
 */
static inline int get_system_stat(struct rsc_zram_module *rcc, bool end)
{
	int ret = 0, value;

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
	if (!is_cpu_idle(rcc, end))
		ret |= WF_CPU_BUSY;
#else
	if (!is_cpu_idle(rcc))
		ret |= WF_CPU_BUSY;
#endif
	value = rcc->free_pages_min + (end ? RSC_ZRAM_FREE_PAGE_MIN_EX : 0);
	if (is_memory_free_enough(value))
		ret |= WF_MEM_FREE_ENOUGH;

	value = end ? rcc->anon_pages_min : rcc->anon_pages_max;
	if (!is_anon_page_enough(value))
		ret |= WF_NO_ANON_PAGE;

	value = rcc->swap_percent_low + (end ? RSC_ZRAM_SWAP_PERCENT_LOW_EX : 0);
	if (is_swap_full(value))
		ret |= WF_SWAP_FULL;

	return ret ? ret : WS_NEED_WAKEUP;
}

/* purpose: wakeup background thread */
static int rsc_zram_thread_wakeup(struct rsc_zram_module *rcc, int period_mode)
{
	/* up semaphore of background thread. */
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
	if (!period_mode)
		up(&rcc->wait);
	else {
		if (waitqueue_active(&rsc_zram_wait)) {
			atomic_set(&rsc_zram_wait_condition, 0);
			wake_up_all(&rsc_zram_wait);
		}
	}
#else
	up(&rcc->wait);
#endif
	return 0;
}

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
static void rsc_add_wait(int action)
{
	static u32 cnt;
	char *cause;

	atomic_set(&rsc_zram_wait_condition, action);
	cnt++;
	cause = (action == RSC_ZRAM_WAIT_CON_PERIOD)?"period":"action";
	rsc_info("rzram %3d wait %s!\n", cnt, cause);
	wait_event(rsc_zram_wait, !atomic_read(&rsc_zram_wait_condition));
	rsc_info("rzram %3d exit %s!\n", cnt, cause);
}

/**
 * purpose: waiting wakeup event in background thread.
 * arguments:
 *    rcc: struct rsc_zram_module.
 *    timeout: timeout to wait. unit is ms. <0 for infinite wait.
 * return:
 *    -ETIME: timeout to wait.
 *         0: received events
 */
static int rsc_zram_thread_wait(struct rsc_zram_module *rcc, long timeout)
{
	int tout;

	if (!rcc->period_mode) {
		/* down semaphore of background thread. */
		if (timeout > 0)
			tout = down_timeout(&rcc->wait, msecs_to_jiffies(timeout));
		else {
			down(&rcc->wait);
			tout = 0;
		}
		/*recount swap_min_free*/
		rsc_zram_swap_min_free = total_swap_pages * (100 - rcc->swap_percent_low)/100;
		return tout;
	} else {
		/*recount swap_min_free*/
		rsc_zram_swap_min_free = total_swap_pages * (100 - rcc->swap_percent_low)/100;
		/*full clean no need to wait*/
		if (!atomic_read(&rcc->full_clean_flag)) {
			atomic_set(&rsc_zram_period_cnt, 0);
			rsc_add_wait(RSC_ZRAM_WAIT_CON_PERIOD);
		}
	}
	return 0;
}

#define DO_SWAP_OUT(mode) do { 													\
		cur_jiffies = jiffies; 													\
		if (ret & WF_CPU_BUSY) { 												\
			nr_pages = 0; 														\
			rsc_add_wait(RSC_ZRAM_WAIT_CON_ACTION); 							\
			busy_count++; 														\
		} else { 																\
			nr_pages = rsc_zram_swap_out(RSC_ZRAM_NR_SWAP_UNIT_SIZE, mode); 	\
			time_jiffies += elapsed_jiffies(cur_jiffies); 						\
			swap_cnt++;															\
		} 																		\
	} while (0)

#define _UPDATE_STATE() do { 													\
		nr_total_pages += nr_pages; 											\
		ret = get_system_stat(rcc, !(ret & WF_CPU_BUSY)); 						\
	} while (0)
#else

/**
 * purpose: waiting wakeup event in background thread.
 * arguments:
 *    rcc: struct rsc_zram_module.
 *    timeout: timeout to wait. unit is ms. <0 for infinite wait.
 * return:
 *    -ETIME: timeout to wait.
 *         0: received events
 */
static int rsc_zram_thread_wait(struct rsc_zram_module *rcc, long timeout)
{
	/*recount swap_min_free*/
	rsc_zram_swap_min_free = total_swap_pages * (100 - rcc->swap_percent_low)/100;

	/* down semaphore of background thread. */
	if (timeout > 0)
		return down_timeout(&rcc->wait, msecs_to_jiffies(timeout));
	down(&rcc->wait);
	return 0;
}

#define DO_SWAP_OUT(mode) do { \
		cur_jiffies = jiffies; \
		if (ret & WF_CPU_BUSY) { \
			nr_pages = 0; \
			busy_count++; \
		} else { \
			nr_pages = rsc_zram_swap_out(RSC_ZRAM_NR_SWAP_UNIT_SIZE, mode); \
			time_jiffies += elapsed_jiffies(cur_jiffies); \
		} \
	} while (0)

#define _UPDATE_STATE() do { \
		nr_total_pages += nr_pages; \
		msleep(RSC_ZRAM_IDLE_FAST); \
		get_cpu_load(rcc, false); \
		if (kthread_should_stop()) \
			goto out;  \
		ret = get_system_stat(rcc, true); \
	} while (0)
#endif

static const char *wf_flags[] = {
	"DISABLE",
	"BUSY",
	"FREE_ENOUGH",
	"SWAP_FULL",
	"NO_ANON"
};

#define WS_NEED_WAKEUP			0
#define WF_NOT_ENABLED			0x01
#define WF_CPU_BUSY				0x02
#define WF_MEM_FREE_ENOUGH		0x04
#define WF_SWAP_FULL			0x08
#define WF_NO_ANON_PAGE			0x10
#define RSX_MAX_ZONE 2

/**
 * purpose: main working thread for compress RAM
 * arguments:
 *    unused: unused.
 * return:
 *    thread exit code.
 */
static int rsc_zram_thread(void *unused)
{
	int ret, wait_ret, nr_pages, nr_total_pages, busy_count, filepages;
	unsigned long time_jiffies, cur_jiffies;
	struct task_struct *tsk = current;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	long timeout;
	static bool ignore_first_event_fullclean;
	int notfirst_run;
	int i, num;
	char print_buf[128];
	int swap_cnt;
	unsigned long pages[RSX_MAX_ZONE][NR_LRU_LISTS];
	unsigned long free[RSX_MAX_ZONE];
	int lru;
	struct zone *zone;
	int zoneidx[RSX_MAX_ZONE];

	/* need swap out, PF_FREEZER_SKIP is protection from hung_task. */
	tsk->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD | PF_FREEZER_SKIP;

#ifndef CONFIG_MTK_PLATFORM
		if (!cpumask_empty(&rsc_cpu_littlecore)) {
			long sched_ret;

			sched_ret = sched_setaffinity(current->pid, &rsc_cpu_littlecore);
			rsc_info("rzram bind cpu: %*pbl ret: %ld\n", cpumask_pr_args(&rsc_cpu_littlecore), sched_ret);
		} else {
			rsc_err("rzram rsc_cpu_littlecore is empty!\n");
		}
#endif

	while (true) {

restart:
		notfirst_run = rcc->notfirst_run;
		if (unlikely(!notfirst_run)) {
			rcc->notfirst_run = 1;
			timeout = rcc->complete_time;;
		} else if (!rcc->period_mode)
			timeout = RSC_ZRAM_WAIT_INFINITE;
		else
			timeout = RSC_ZRAM_IDLE_SLOW;

		if (unlikely(rcc->task_exit || kthread_should_stop()))
			goto out;

		/* wait wakeup_event or timeout. */
		wait_ret = rsc_zram_thread_wait(rcc, timeout);

#ifdef CONFIG_MTK_PLATFORM
		/*bind cpu to little core here, because cpufreq policy init very late!*/
		if (unlikely(!notfirst_run)) {
				if (!cpumask_empty(&rsc_cpu_littlecore)) {
					long sched_ret;

					sched_ret = sched_setaffinity(current->pid, &rsc_cpu_littlecore);
					rsc_info("rzram bind cpu: %*pbl ret: %ld\n", cpumask_pr_args(&rsc_cpu_littlecore), sched_ret);
				} else {
					rsc_err("rzram rsc_cpu_littlecore is empty!\n");
				}
		}
#endif

		if (unlikely(rcc->task_exit || kthread_should_stop()))
			goto out;
#ifndef RSC_ZRAM_FAST_LOAD_CHECK
		/* force update cpu load stat. */
		get_cpu_load(rcc, true);
#endif
		nr_total_pages = 0;
		busy_count = 0;
		swap_cnt = 0;
		time_jiffies = 0;
		ret = get_system_stat(rcc, false);

		num = 0;
		for (i = 1; i < ARRAY_SIZE(wf_flags); i++) {
			if (ret & (1 << i))
				num += snprintf(print_buf+num, sizeof(print_buf)-num, "|%s", wf_flags[i]);
		}
		if (!num)
			snprintf(print_buf, sizeof(print_buf), "|NEED_WAKEUP|");
		else
			snprintf(print_buf+num, sizeof(print_buf), "|");

		i = 0;
		for_each_zone(zone) {
			if (i < RSX_MAX_ZONE) {
				for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
					pages[i][lru] = zone_page_state(zone, NR_ZONE_LRU_BASE + lru);
#else
					pages[i][lru] = zone_page_state(zone, NR_LRU_BASE + lru);
#endif
				zoneidx[i] = zone_idx(zone);
				free[i] = zone_page_state(zone, NR_FREE_PAGES);
			}
			i++;
		}
		rsc_info("rzram %s free min %4u %4lu ,swap min %ld %ld ,anon min %u %lu MB ,stat %x %s "
				"FREE INA_ANON ACT_ANON INA_FILE ACT_FILE UNEVICTABLE z %d %4lu %4lu %4lu %4lu %4lu %4lu z %d %4lu %4lu %4lu %4lu %4lu %4lu MB\n",
				atomic_read(&rcc->full_clean_flag)?"full":"normal",
				M(rcc->free_pages_min), M(cur_nr_free_pages), M(rsc_zram_swap_min_free),
				M(atomic_long_read(&nr_swap_pages)), M(rsc_zram_anon_pages_max),
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
				M(global_node_page_state(NR_INACTIVE_ANON) + global_node_page_state(NR_ACTIVE_ANON)),
#else
				M(global_page_state(NR_INACTIVE_ANON) + global_page_state(NR_ACTIVE_ANON)),
#endif
				ret, print_buf,
				zoneidx[0], M(free[0]), M(pages[0][LRU_INACTIVE_ANON]), M(pages[0][LRU_ACTIVE_ANON]), 
				M(pages[0][LRU_INACTIVE_FILE]), M(pages[0][LRU_ACTIVE_FILE]), M(pages[0][LRU_UNEVICTABLE]),
				zoneidx[1], M(free[1]), M(pages[1][LRU_INACTIVE_ANON]), M(pages[1][LRU_ACTIVE_ANON]),
				M(pages[1][LRU_INACTIVE_FILE]), M(pages[1][LRU_ACTIVE_FILE]), M(pages[1][LRU_UNEVICTABLE]));

		if (atomic_xchg(&rcc->full_clean_flag, 0)) {
			/*
			ignore one full clean! maybe it comes from RSC_ZRAM_WAIT_COMPLETE
			or echo fullclean > event
			*/
			if (unlikely(!notfirst_run)) {
				/*timeout*/
				if (wait_ret == -ETIME)
					ignore_first_event_fullclean = 1;
			}
			if (unlikely(ignore_first_event_fullclean &&
				(1 == rcc->fullclean_count))) {
				ignore_first_event_fullclean = 0;
				rsc_info("rzram wakeup ignore first full cnt %u - %u ret %d\n",
					rcc->wakeup_count, rcc->fullclean_count, wait_ret);
				goto restart;
			}

			rcc->fullclean_count++;
			/*after first run set to period mode*/
			if (unlikely((1 == rcc->fullclean_count) &&
					!rcc->force_period))
				rcc->period_mode = 1;

			rcc->wakeup_count++;
			rsc_info("rzram wakeup full cnt %u - %u ret %d\n", rcc->wakeup_count,
				rcc->fullclean_count, wait_ret);

			/* clean some file cache first */
			while (nr_total_pages < rcc->full_clean_file_pages) {
				DO_SWAP_OUT(RSC_ZRAM_MODE_FILE);
				_UPDATE_STATE();
				if (swap_cnt > (5*RSC_ZRAM_FULL_CLEAN_FILE_PAGE/RSC_ZRAM_NR_SWAP_UNIT_SIZE))
					break;
			};
			swap_cnt = 0;
			filepages = nr_total_pages;
			nr_total_pages = 0;
			/* full fill swap area. */
			do {
				DO_SWAP_OUT(RSC_ZRAM_MODE_ANON);
				_UPDATE_STATE();
				if (swap_cnt > (5*RSC_ZRAM_MAX_RECLAIM_ON_BOOT/RSC_ZRAM_NR_SWAP_UNIT_SIZE))
					break;
			} while (!rcc->task_exit && !(ret & WF_SWAP_FULL)
				 && !(ret & WF_NO_ANON_PAGE)
				 && nr_total_pages < rcc->full_clean_anon_pages);

			rcc->nr_full_clean_pages += (filepages + nr_total_pages);
			rcc->total_spent_times += time_jiffies;
			/*
			[  168.770935]@5[03-08 11:47:20]
			^RSC rzram full 345388 ( 17128 328260 ) KB cost 8180 ms busy 157 stat  4 cnt 1 - 1
			the zram compress speed of lz4 for sdm660 is about 328260/8180ms = 40MB/s
			*/
			rsc_info("rzram full free %4lu MB %6u ( %5u %6u ) KB cost %4u ms busy %3d stat %2d cnt %u - %u\n",
				M(cur_nr_free_pages), K(filepages + nr_total_pages),
				K(filepages), K(nr_total_pages),
				jiffies_to_msecs(time_jiffies),
				busy_count, ret,  rcc->wakeup_count,
				rcc->fullclean_count);
		} else if (ret == WS_NEED_WAKEUP) {
			rcc->wakeup_count++;
			rsc_dbg(RSC_ZRAM, "rzram wakeup normal %u\n", rcc->wakeup_count);

			/* swap out pages. */
			do {
				DO_SWAP_OUT(RSC_ZRAM_MODE_ANON);
				_UPDATE_STATE();
				if (atomic_read(&rcc->full_clean_flag) || (busy_count > 1000) ||
					(swap_cnt > (RSC_ZRAM_MAX_SWAP_SIZE/RSC_ZRAM_NR_SWAP_UNIT_SIZE)))
					break;
			} while (!rcc->task_exit && !(ret & WF_MEM_FREE_ENOUGH)
				 && !(ret & WF_SWAP_FULL)
				 && !(ret & WF_NO_ANON_PAGE));
			rcc->nr_normal_clean_pages += nr_total_pages;
			rcc->total_spent_times += time_jiffies;
			rsc_info("rzram normal free %4lu MB %6u KB cost %4u ms busy %3d swapcnt %3d stat %2d  cnt %u - %u\n",
				M(cur_nr_free_pages), K(nr_total_pages), jiffies_to_msecs(time_jiffies),
				busy_count, swap_cnt, ret, rcc->wakeup_count,
				rcc->wakeup_count - rcc->fullclean_count);
		}
	}
out:
	rcc->task_out = 1;
	tsk->flags &=
	    ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD | PF_FREEZER_SKIP);
	rsc_info("rzram exiting!\n");
	do {
		msleep_interruptible(100);
	} while (!kthread_should_stop());
	rsc_info("rzram exit done!\n");

	return 0;
}

static ssize_t decompress_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	return 0;
}

/*#define RSC_DECOM_DEBUG*/
extern long rsc_get_swap_pages(struct vm_area_struct *vma, unsigned long start, unsigned long nr_pages,
		unsigned int gup_flags);
static ssize_t decompress_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t len)
{
#if 0
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	int map_count;
	int i;
	int ret;
	unsigned long nr_pages;
#ifdef RSC_DECOM_DEBUG
	u64 t1, t2;
#endif
	u64 t3, t4;
	struct task_struct *p;
	pid_t pid;
	char comm[TASK_COMM_LEN];
	vm_flags_t flags;
	int decom_pages = 0;
#ifdef RSC_DECOM_DEBUG
	struct seq_file m;
	char kbuf[128];

	memset(&m, 0, sizeof(m));
	m.buf = kbuf;
	m.size = sizeof(kbuf);
#endif

	ret = kstrtoint(buf, 10, &pid);
	if (ret || pid < 0) {
		rsc_err("rzram decompress pid %d ret %d error!\n",
			pid, ret);
		return ret;
	}

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		rsc_err("rzram decompress could not find pid %d!\n",
			pid);
		return -EINVAL;
	}
	strlcpy(comm, p->comm, TASK_COMM_LEN);
	get_task_struct(p);
	rcu_read_unlock();

	mm = get_task_mm(p);
	put_task_struct(p);
	if (!mm)
		return -EINVAL;

	if (!atomic_inc_not_zero(&mm->mm_users))
		return -EINVAL;

	down_read(&mm->mmap_sem);
	map_count = mm->map_count;
	vma = mm->mmap;
	t3 = local_clock();
	for (i = 0; i < map_count && vma; i++) {
		/*if (vma->vm_file)
			continue;*/
		if (!is_vm_hugetlb_page(vma)) {
			flags = vma->vm_flags;
			if (flags & (VM_READ | VM_WRITE | VM_EXEC) && vma->anon_vma/* && (vma_is_anonymous(vma))*/) {
				nr_pages = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
#ifdef RSC_DECOM_DEBUG
				t1 = local_clock();
#endif
				/*ret = get_user_pages(p, mm, vma->vm_start, nr_pages,
					0, 1, NULL, NULL);*/
				ret = rsc_get_swap_pages(vma, (vma->vm_start + (PAGE_SIZE-1)) & PAGE_MASK, nr_pages,
						FOLL_TOUCH);
				decom_pages += ret;
#ifdef RSC_DECOM_DEBUG
				t2 = local_clock();
				if (vma->vm_file)
					seq_file_path(&m, vma->vm_file, "\n");
				rsc_info("rzram i %3d start: 0x%lx nr_pages: %4ld ret: %4d cost %4llu anon_vma: %p file %p vma->vm_ops %ps path: %s\n",
					i, vma->vm_start, nr_pages, ret, (t2 - t1)/1000, vma->anon_vma, vma->vm_file,
					 vma->vm_ops, vma->vm_file?kbuf:NULL);
#endif
			}
		}
		vma = vma->vm_next;
	}

	t4 = local_clock();
	rsc_info("rzram decompress %16s (%5d) map_count: %d cost %llu us decom size: %d KB\n",
		comm, pid, map_count, (t4 - t3)/1000, decom_pages << (PAGE_SHIFT - 10));
	flush_tlb_mm(mm);
	up_read(&mm->mmap_sem);
	mmput(mm);
#endif
	return len;
}

/**
 * purpose: initialize this module
 * arguments:
 *    rcc: module handler need to be initialized.
 * return:
 *    none.
 */
static void rsc_zram_setup(struct rsc_zram_module *rcc)
{
	/* memset(rcc,0,sizeof(struct rsc_zram_module)); */
	init_rwsem(&rcc->lock);
	sema_init(&rcc->wait, 0);

#if defined(CONFIG_MTK_PLATFORM)
	rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT[0] * NR_CPUS *
								RSC_ZRAM_IDLE_THRESHOLD) / 100;
#else
	rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT * NR_CPUS *
								RSC_ZRAM_IDLE_THRESHOLD) / 100;
#endif
	rcc->period_mode = 0;

	/*at this time, total_swap_pages is 0?*/
	rsc_zram_swap_min_free = total_swap_pages * (100 - RSC_ZRAM_SWAP_PERCENT_LOW)/100;
#ifndef RSC_ZRAM_FAST_LOAD_CHECK
	rcc->idle_threshold = RSC_ZRAM_IDLE_THRESHOLD;
	rcc->cpu_load[0] = 100; /* init cpu load as 100% */
#endif
	rcc->swap_percent_low = RSC_ZRAM_SWAP_PERCENT_LOW;
	rcc->free_pages_min = RSC_ZRAM_FREE_PAGE_MIN;
	rcc->full_clean_file_pages = RSC_ZRAM_FULL_CLEAN_FILE_PAGE;
	rcc->anon_pages_min = RSC_ZRAM_ANON_PAGE_MIN;
	rsc_zram_anon_pages_max = RSC_ZRAM_ANON_PAGE_MAX;
	rcc->anon_pages_max = RSC_ZRAM_ANON_PAGE_MAX;
	rcc->full_clean_anon_pages = RSC_ZRAM_MAX_RECLAIM_ON_BOOT;
	rcc->complete_time = RSC_ZRAM_WAIT_COMPLETE;
}

/* purpose: start backgroud thread */
static int rsc_zram_thread_start(struct rsc_zram_module *rcc, int fullclean)
{
	if (rcc->task)
		return -EFAIL;

	rsc_zram_set_full_clean(rcc, fullclean);
	rcc->task_exit = 0;

	rcc->task = kthread_run(rsc_zram_thread, NULL, "rsc_zram");
	if (IS_ERR(rcc->task)) {
		rsc_err("rzram failed to start thread\n");
		return -EFAIL;
	}
	rcc->task_out = 0;

	rsc_info("rzram thread started.\n");
	return 0;
}

/* purpose: stop backgroud thread */
static void rsc_zram_thread_stop(struct rsc_zram_module *rcc)
{
	if (!rcc->task)
		return;

	rcc->task_exit = 1;
	do {
		rsc_zram_thread_wakeup(rcc, rcc->period_mode);/* need wakeup thread first. */
		msleep_interruptible(20);
		rsc_info("rzram thread stopped loop %d.\n", rcc->task_out);
	} while (!rcc->task_out);
	kthread_stop(rcc->task);
	rcc->task = NULL;
	rsc_info("rzram thread stopped.\n");
	rcc->task_out = 0;
}

#define RSC_ZRAM_MODE_RO 0440
#define RSC_ZRAM_MODE_RW 0660

#define RSC_ZRAM_ATTR(_name, _mode, _show, _store) \
	struct kobj_attribute kobj_attr_##_name \
		= __ATTR(_name, _mode, _show, _store)

#ifdef RSC_ZRAM_THREAD_SWITCH
/*
 * echo 1 > /proc/sys/vm/rsc_zram_enable
 * for selinux permission.
 */
int rsc_zram_enable_handler(struct ctl_table *table, int write,
	void __user *buffer, size_t *length, loff_t *ppos)
{
	int ret;
	struct rsc_zram_module *rcc = &rsc_zram_module;

	mutex_lock(&rsc_zram_lock);
	ret = proc_dointvec(table, write, buffer, length, ppos);
	if (ret)
		goto out;
	if (write) {
		if (rsc_zram_enable)
			rsc_zram_thread_start(rcc, !rcc->notfirst_run);
		else
			rsc_zram_thread_stop(rcc);
	}

out:
	mutex_unlock(&rsc_zram_lock);

	return 0;
}

/* purpose: attr status: is backgroud thread running */
static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	return scnprintf(buf, PAGE_SIZE, "%u\n", rsc_zram_is_enabled(rcc));
}

/* purpose: attr set: backgroud thread start/stop */
static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t len)
{
	int ret;
	u16 enable;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	ret = kstrtou16(buf, 10, &enable);
	if (ret)
		return ret;

	mutex_lock(&rsc_zram_lock);
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
	rsc_zram_enable = enable;
#endif
	if (enable)
		rsc_zram_thread_start(rcc, !rcc->notfirst_run);
	else
		rsc_zram_thread_stop(rcc);
	mutex_unlock(&rsc_zram_lock);

	return len;
}
#endif

static ssize_t event_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n");
}

static ssize_t event_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t len)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	if (!strncmp(buf, "displayoff", strlen("displayoff"))) {
		rsc_zram_update_display_stat(rcc, 0);
		rsc_zram_thread_wakeup(rcc, rcc->period_mode);
	} else if (!strncmp(buf, "displayon", strlen("displayon"))) {
		rsc_zram_update_display_stat(rcc, 1);
	} else if (!strncmp(buf, "fullclean", strlen("fullclean"))) {
		rsc_zram_set_full_clean(rcc, 1);
		rsc_zram_thread_wakeup(rcc, rcc->period_mode);
		rsc_info("rzram do fullclean cnt %d\n", rcc->fullclean_count);
	} else if (!strncmp(buf, "periodoff", strlen("periodoff"))) {
		rcc->force_period = 1;
		barrier();
		smp_wmb();
		if (!rcc->period_mode)
			return len;
		rcc->period_mode = 0;
		rsc_zram_thread_wakeup(rcc, 1);
	} else if (!strncmp(buf, "periodon", strlen("periodon"))) {
		if (rcc->period_mode)
			return len;
		rcc->period_mode = 1;
		rsc_zram_thread_wakeup(rcc, 0);
	} else {
		rsc_err("rzram unknown event: [%s] size=%zu\n",
			   buf, strlen(buf));
	}
	return len;
}

/* purpose: attr status:  */
static ssize_t idle_threshold_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d%%\n",
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
		rsc_zram_idle_threshold
#else
		rsc_zram_module.idle_threshold
#endif
	);
}

/* purpose: attr set:  */
static ssize_t idle_threshold_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t len)
{
	int ret;
	u16 value;
#ifndef RSC_ZRAM_FAST_LOAD_CHECK
	struct rsc_zram_module *rcc = &rsc_zram_module;
#endif
	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;
	if (value > 100 || value < 0)
		return -EINVAL;

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
	rsc_zram_idle_threshold = value;
#if defined(CONFIG_MTK_PLATFORM)
	rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT[0] * NR_CPUS *
							value) / 100;
#else
	rsc_zram_idle_load = (RSC_ZRAM_TICK_CNT * NR_CPUS *
							value) / 100;
#endif
#else
	rcc->idle_threshold = value;
#endif
	return len;
}

/* purpose: attr status:  */
static ssize_t swap_percent_low_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	return scnprintf(buf, PAGE_SIZE, "%d, extra: +%d\n",
		       rcc->swap_percent_low, RSC_ZRAM_SWAP_PERCENT_LOW_EX);
}

/* purpose: attr set:  */
static ssize_t swap_percent_low_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t len)
{
	int ret;
	u16 value;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	ret = kstrtou16(buf, 10, &value);
	if (ret)
		return ret;
	if (value > 100)
		return -EINVAL;
	if (value > 90)
		return value;

	rsc_zram_swap_min_free = total_swap_pages * (100 - value)/100;
	rcc->swap_percent_low = value;
	return len;
}

/* purpose: attr status:  */
static ssize_t free_size_min_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	return scnprintf(buf, PAGE_SIZE, "%d MB(%lu) , extra: +%d MB\n",
		       (rcc->free_pages_min >> (20 - PAGE_SHIFT)), (unsigned long)M(RSC_ZRAM_FREE_PAGE_MIN),
		       (RSC_ZRAM_FREE_PAGE_MIN_EX >> (20 - PAGE_SHIFT)));
}

/* purpose: attr set:  */
static ssize_t free_size_min_store(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf,
			size_t len)
{
	u64 size;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	size = memparse(buf, NULL);
	if (!size)
		return -EINVAL;
	rcc->free_pages_min = (size >> PAGE_SHIFT);

	/*low ram project need adjust RSC_ZRAM_FREE_PAGE_MIN*/
#if 1
	RSC_ZRAM_FREE_PAGE_MIN = (size >> PAGE_SHIFT);
#endif
	rsc_info("rzram set free_size_min to %lu(%lu MB)\n", (unsigned long) size, (unsigned long)M((size >> PAGE_SHIFT)));

	return len;
}

/* purpose: attr status:  */
static ssize_t full_clean_size_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	int size = (rcc->full_clean_file_pages >> (20 - PAGE_SHIFT));
	return scnprintf(buf, PAGE_SIZE, "%d MB\n", size);
}

/* purpose: attr set:  */
static ssize_t full_clean_size_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t len)
{
	u64 size;
	unsigned long nr_file_pages;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	size = memparse(buf, NULL);
	if (!size)
		return -EINVAL;

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	nr_file_pages = global_node_page_state(NR_INACTIVE_FILE);
	nr_file_pages += global_node_page_state(NR_ACTIVE_FILE);
#else
	nr_file_pages = global_page_state(NR_INACTIVE_FILE);
	nr_file_pages += global_page_state(NR_ACTIVE_FILE);
#endif
	size = size >> PAGE_SHIFT;

	/* size should not larger than file cache pages. */
	if (size > nr_file_pages)
		size = nr_file_pages;

	rcc->full_clean_file_pages = size;
	return len;
}

/* purpose: attr status:  */
static ssize_t rsc_zram_stat_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	int ret, num, i;
	char print_buf[128];
	unsigned long nr_pages;

	ret = get_system_stat(rcc, false);
	num = 0;
	for (i = 0; i < ARRAY_SIZE(wf_flags); i++) {
		if (ret & (1 << i))
			num += snprintf(print_buf+num, sizeof(print_buf)-num, "|%s", wf_flags[i]);
	}
	if (!num)
		snprintf(print_buf, sizeof(print_buf), "|NEED_WAKEUP|");
	else
		snprintf(print_buf+num, sizeof(print_buf), "|");
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	nr_pages = global_node_page_state(NR_INACTIVE_ANON) +
		global_node_page_state(NR_ACTIVE_ANON);
#else
	nr_pages = global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_ACTIVE_ANON);
#endif
	return scnprintf(buf, PAGE_SIZE,
		"fullclean %8u normal %8u KB "
		"anon pages min %u max %8u KB "
		"wake count %3u %2u cost %6u ms "
		"period %d ,free min %4u %4lu ,swap min %ld %ld ,anon min %u %lu MB ,stat %x %s\n",
		K(rcc->nr_full_clean_pages), K(rcc->nr_normal_clean_pages),
		K(rcc->anon_pages_min), K(rcc->anon_pages_max),
		rcc->wakeup_count, rcc->fullclean_count,
		jiffies_to_msecs(rcc->total_spent_times),
		rcc->period_mode, M(rcc->free_pages_min),
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
		M(global_zone_page_state(NR_FREE_PAGES)),
#else
		M(global_page_state(NR_FREE_PAGES)),
#endif
		M(rsc_zram_swap_min_free), M(atomic_long_read(&nr_swap_pages)),
		M(rsc_zram_anon_pages_max), M(nr_pages), ret, print_buf);
}

/* purpose: show the size of max anon page to reclaim  */
static ssize_t max_anon_clean_size_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	int size = (rcc->full_clean_anon_pages >> (20 - PAGE_SHIFT));
	return scnprintf(buf, PAGE_SIZE, "%d MB\n", size);
}

/* purpose: set max anon page size to reclaim */
static ssize_t max_anon_clean_size_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t len)
{
	u64 size;
	struct rsc_zram_module *rcc = &rsc_zram_module;
	size = memparse(buf, NULL);
	if (!size)
		return -EINVAL;

	size = size >> PAGE_SHIFT;
	rcc->full_clean_anon_pages = size;
	return len;
}

/*purpose:
 lz0 speed: compress is about 188MB/s,  decompress is about 210MB/s
 lz4 speed: compress is about 229MB/s,  decompress is about 270MB/s
 */
static ssize_t zram_speed_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	unsigned long pswpin, pswpout;
	u64 time = atomic_long_read(&rsc_zram_decompress_time);
	u64 pages = atomic_long_read(&rsc_zram_decompress_pages);
	u32 comp_time = jiffies_to_msecs(rcc->total_spent_times);
	u64 comp_pages;

	comp_pages = rcc->nr_full_clean_pages + rcc->nr_normal_clean_pages;
	time = time/1000;
	pswpin = 0;
	pswpout = 0;
#ifdef CONFIG_VM_EVENT_COUNTERS
	{
		int cpu;

		get_online_cpus();
		for_each_online_cpu(cpu) {
		struct vm_event_state *this = &per_cpu(vm_event_states, cpu);

			pswpin += this->event[PSWPIN];
			pswpout += this->event[PSWPOUT];
		}
		put_online_cpus();
	}
#endif

	return scnprintf(buf, PAGE_SIZE, "decompress speed stat is %s. pages %llu %lu, cost %llu ms, decompspeed %llu MB/s, "
									"comp pages %llu %lu, cost %u ms, compspeed %llu MB/s\n",
									rsc_zram_speed_enable?"Enalbe":"Disable", pages, pswpin, time/1000, time?M(pages*1000000UL/time):0,
									comp_pages, pswpout, comp_time, comp_time?M(comp_pages*1000UL/comp_time):0);
}

/* purpose: enable or disable zram decompress speed!*/
static ssize_t zram_speed_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t len)
{
	int ret;
	u16 enable;

	ret = kstrtou16(buf, 10, &enable);
	if (ret)
		return ret;
	if (enable)
		rsc_zram_speed_enable = 1;
	else
		rsc_zram_speed_enable = 0;
	return len;
}

static ssize_t complete_time_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;
	return scnprintf(buf, PAGE_SIZE, "%d\n", rcc->complete_time);
}

/* purpose: attr set: backgroud thread start/stop */
static ssize_t complete_time_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t len)
{
	int ret;
	int time = 0;
	struct rsc_zram_module *rcc = &rsc_zram_module;

	ret = kstrtoint(buf, 10, &time);

	/*only set before task create!*/
	if (rcc->task) {
		rsc_err("rzram only set before rsc_zram create! ret %d time: %d\n", ret, time);
		return -EFAIL;
	}

	if (ret)
		return ret;

	rcc->complete_time = time;
	return len;
}

#if defined(CONFIG_RSC_KSWAPD_IMPROVE)
static ssize_t vspwad_enable_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	struct sysinfo i;
	unsigned long exetime = vswapd_total_exe_time_us/(1000*1000);

	if (!exetime)
		exetime = 1;
	si_swapinfo(&i);
	return scnprintf(buf, PAGE_SIZE, "start %u MB %u pages stop %u MB %u pages cstop %u MB %u pages "
								"maxruntime %u ms reclaim %7lu MB "
								"time %8lu exe %8llu ms wakeupcnt %5u ssw %4u "
								"timeout %2u kswapdrun %3u mincache %3u emstm %2u miss %2u zerocnt %2u "
								"free %3lu fc %4lu fs %4lu MB speed %lu MB/s "
#if 0 //disable fo MTK
								"swap_fn %u nw %u exe %llu ms"
#endif
								"\n",
								vswapd_start_pages >> (20 - PAGE_SHIFT), vswapd_start_pages,
								vswapd_stop_pages >> (20 - PAGE_SHIFT), vswapd_stop_pages,
								vswapd_min_filecache_stop_pages >> (20 - PAGE_SHIFT), vswapd_min_filecache_stop_pages,
								RSC_JIFF_TO_MS(vswapd_max_running_time_jiffies),
								vswapd_total_pages >> (20 - PAGE_SHIFT),
								RSC_JIFF_TO_MS(vswapd_total_time_jiffies),
								vswapd_total_exe_time_us/1000,
								vswapd_total_wakeup_cnt,
								vswapd_short_sleep_wakeup_cnt,
								vswapd_total_timeout_cnt,
								vswapd_total_kswap_running_cnt,
								vswapd_total_min_filecache_cnt,
								vswapd_emergency_stop_timeout_cnt,
								vswapd_emergency_stop_wakeup_miss_cnt,
								vswapd_reclaim_zero_cnt,
								global_zone_page_state(NR_FREE_PAGES) >> (20 - PAGE_SHIFT),
								global_node_page_state(NR_FILE_PAGES) >> (20 - PAGE_SHIFT),
								i.freeswap >> (20 - PAGE_SHIFT),
								(vswapd_total_pages >> (20 - PAGE_SHIFT))/exetime
#if 0 //disable fo MTK
								,vswap_fn_wakeup_cnt,
								vswap_fn_wakeup_no_swap_cnt,
								vswap_fn_task ? vswap_fn_task->se.sum_exec_runtime/(1000*1000) : 0UL
#endif
						);
}

/*
* Disable vswapd:
* adb shell "echo 0 0 > /sys/rsc/zram/vspwad_enable"
* Enable vswapd:
* adb shell "echo 200 300 > /sys/rsc/zram/vspwad_enable"
*/
static ssize_t vspwad_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t len)
{
	int ret;
	u32 start0, stop0, mincache0, maxruntime0;
	u32 start, stop, mincache, maxruntime;

	ret = sscanf(buf, "%u %u %u %u", &start, &stop, &mincache, &maxruntime);
	if ((ret >= 2) && (ret <= 4) && (start <= stop)) {
		start0 = vswapd_start_pages;
		stop0 = vswapd_stop_pages;
		mincache0 = vswapd_min_filecache_stop_pages;
		maxruntime0 = vswapd_max_running_time_jiffies;
		vswapd_start_pages = start << (20 - PAGE_SHIFT);
		vswapd_stop_pages = stop << (20 - PAGE_SHIFT);
		if (ret >= 3)
			vswapd_min_filecache_stop_pages = mincache << (20 - PAGE_SHIFT);
		if (ret >= 4)
			vswapd_max_running_time_jiffies = RSC_MS_TO_JIFF(maxruntime);
		smp_wmb();

		/*recalculate watermark*/
		setup_per_zone_wmarks();

		/*enable vswapd*/
		if (!start0 && start && vswapd_task) {
			vswapd_emergency_stop = 1;
			smp_wmb();
			wake_up_process(vswapd_task);
		}
		/*
		* if disalbe vswapd,
		* park vswapd in the right position,
		* then waitqueue_active(&vswapd_wait) will be return false.
		* and only do one judgment in __alloc_pages_nodemask
		* for good performance.
		* see rsc_vswapd.
		*/
		if (!start && waitqueue_active(&vswapd_wait))
			wake_up_interruptible(&vswapd_wait);
	} else {
		rsc_err("set vspwad_enable error! ret: %d buf: %s\n",
			ret, buf);
		return -EINVAL;
	}
	rsc_info("set vspwad_enable okay! "
			"start %u MB %u -> %u pages "
			"stop %u MB %u -> %u pages "
			"mincache %u MB %u -> %u pages "
			"%u -> %u ms"
			"ret: %d buf: %s\n",
			vswapd_start_pages >> (20 - PAGE_SHIFT), start0, vswapd_start_pages,
			vswapd_stop_pages >> (20 - PAGE_SHIFT), stop0, vswapd_stop_pages,
			vswapd_min_filecache_stop_pages >> (20 - PAGE_SHIFT), mincache0, vswapd_min_filecache_stop_pages,
			RSC_JIFF_TO_MS(maxruntime0), RSC_JIFF_TO_MS(vswapd_max_running_time_jiffies),
			ret, buf);

	return len;
}
static RSC_ZRAM_ATTR(vspwad_enable, RSC_ZRAM_MODE_RW, vspwad_enable_show, vspwad_enable_store);
#endif

#ifdef RSC_ZRAM_THREAD_SWITCH
static RSC_ZRAM_ATTR(enable, RSC_ZRAM_MODE_RW, enable_show, enable_store);
#endif
static RSC_ZRAM_ATTR(event, RSC_ZRAM_MODE_RW, event_show, event_store);
static RSC_ZRAM_ATTR(idle_threshold, RSC_ZRAM_MODE_RW, idle_threshold_show,
		idle_threshold_store);
static RSC_ZRAM_ATTR(swap_percent_low, RSC_ZRAM_MODE_RW, swap_percent_low_show,
		swap_percent_low_store);
static RSC_ZRAM_ATTR(free_size_min, RSC_ZRAM_MODE_RW, free_size_min_show,
		free_size_min_store);
static RSC_ZRAM_ATTR(full_clean_size, RSC_ZRAM_MODE_RW, full_clean_size_show,
		full_clean_size_store);
static RSC_ZRAM_ATTR(stat, RSC_ZRAM_MODE_RO, rsc_zram_stat_show, NULL);
static RSC_ZRAM_ATTR(max_anon_clean_size, RSC_ZRAM_MODE_RW,
			max_anon_clean_size_show, max_anon_clean_size_store);
static RSC_ZRAM_ATTR(speed, RSC_ZRAM_MODE_RW,
			zram_speed_show, zram_speed_store);
static RSC_ZRAM_ATTR(complete_time, RSC_ZRAM_MODE_RW,
			complete_time_show, complete_time_store);
static RSC_ZRAM_ATTR(decomp, RSC_ZRAM_MODE_RW, decompress_show, decompress_store);

static struct attribute *rsc_zram_attrs[] = {
#ifdef RSC_ZRAM_THREAD_SWITCH
	&kobj_attr_enable.attr,
#endif
	&kobj_attr_event.attr,
	&kobj_attr_idle_threshold.attr,
	&kobj_attr_swap_percent_low.attr,
	&kobj_attr_free_size_min.attr,
	&kobj_attr_full_clean_size.attr,
	&kobj_attr_stat.attr,
	&kobj_attr_max_anon_clean_size.attr,
	&kobj_attr_speed.attr,
	&kobj_attr_complete_time.attr,
	&kobj_attr_decomp.attr,
#if defined(CONFIG_RSC_KSWAPD_IMPROVE)
	&kobj_attr_vspwad_enable.attr,
#endif
	NULL,
};

#if 0
static struct attribute_group rsc_zram_module_attr_group = {
	.attrs = rsc_zram_attrs,
};
#endif
/**
 * purpose: create sysfs nodes for module
 * arguments:
 *    none
 * return:
 *    kobject : for future destory.
 */
static struct kobject *sysfs_create(void)
{
	int err, i;
	struct kobject *kobj = NULL;

	kobj = kobject_create_and_add("zram", rsc_root_dir);
	if (!kobj) {
		rsc_err("rzram failed to create sysfs node.\n");
		return NULL;
	}
#if 1
	for (i = 0; i < ARRAY_SIZE(rsc_zram_attrs) - 1; i++) {
		/* create sysfs */
		err = sysfs_create_file(kobj, rsc_zram_attrs[i]);
		rsc_chown_to_system(kobj, rsc_zram_attrs[i]);
	}
#else
	err = sysfs_create_group(kobj, &rsc_zram_module_attr_group);
#endif
	if (err) {
		rsc_err("rzram failed to create sysfs attrs.\n");
		kobject_put(kobj);
		return NULL;
	}
	return kobj;
}

/**
 * purpose: destory sysfs nodes
 * arguments:
 *    kobj : kobject for release.
 * return:
 *    none
 */
static void sysfs_destory(struct kobject *kobj)
{
	if (!kobj)
		return;
	kobject_put(kobj);
}

#ifdef CONFIG_MTK_PLATFORM
#include <mt-plat/mtk_boot_common.h>
#else
extern unsigned int power_off_charging_mode;
#endif
extern unsigned int is_atboot;
/*
power_off_charging_mode = 0; normal mode
power_off_charging_mode = 1; charge mode ,system not bootup
unsigned int is_atboot = 0; normal mode
unsigned int is_atboot = 1; AT mode
*/
/* purpose: this module init */
static int __init rsc_zram_init(void)
{
/*msm8953, 8937 donot enable in the kernel start, kernel version is 3.18.71*/
/*#if LINUX_VERSION_CODE != KERNEL_VERSION(3, 18, 71)*/
	 int ret;
/*#endif*/
	struct rsc_zram_module *rcc = &rsc_zram_module;
#ifdef CONFIG_MTK_PLATFORM
	enum boot_mode_t bmode = get_boot_mode();
#endif

	/*rsc_info("rzram init...\n");*/

	rsc_zram_setup(rcc);
/*#if LINUX_VERSION_CODE != KERNEL_VERSION(3, 18, 71)*/
	/*first time alway do full clean*/
	if (
#ifdef CONFIG_MTK_PLATFORM
		(bmode == NORMAL_BOOT)
#else
		!power_off_charging_mode
#endif
		&& !is_atboot) {
		ret = rsc_zram_thread_start(rcc, 1);
		if (ret) {
			goto failed_to_create_thread;
		}
		rsc_zram_enable = 1;
	}
/*#endif*/
	rcc->kobj = sysfs_create();
	if (!rcc->kobj)
		goto failed_to_create_sysfs;

	rsc_info("rzram init successfully power_off_charging_mode %u is_atboot %u\n",
#ifdef CONFIG_MTK_PLATFORM
		bmode,
#else
		power_off_charging_mode,
#endif
		is_atboot);
	return 0;

failed_to_create_sysfs:
	rsc_zram_thread_stop(rcc);
/*#if LINUX_VERSION_CODE != KERNEL_VERSION(3, 18, 71)*/
failed_to_create_thread:
	rsc_info("rzram init failed!\n");
/*#endif*/
	return -EFAIL;
}

/* purpose: this module de-init */
static void __exit rsc_zram_exit(void)
{
	struct rsc_zram_module *rcc = &rsc_zram_module;

	sysfs_destory(rcc->kobj);
	rcc->kobj = NULL;

	rsc_zram_thread_stop(rcc);
}

module_init(rsc_zram_init);
module_exit(rsc_zram_exit);
