/*
 * kernel/rsc/rsc_cpu_capability.c
 *
 * VIVO Resource Control.
 *
 * CPU Capability change notify.
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 * Jinling.ke<kjl@vivo.com>
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/cpu.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/atomic.h>

#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_cpu_internal.h>
#ifdef CONFIG_RSC_TASK_CPU_USAGE
#include <linux/vivo_rsc/rsc_cpu_usage.h>
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
#include <linux/arch_topology.h>
#endif

/*
   if #define RSC_CPUCAP_NOLOCK,
   it will donot use spinlock_irqsave,
   instead, use no lock operation.
   A producer consumer system:
	ring bufer + disable irq + cmpxchg cmd +
	atomic operation: write_read_bit,
	see function rsc_cpucap_enqueue
 */
#define RSC_CPUCAP_NOLOCK

#define CPUCAP_NOTIFY_NAME cpucap_notif
#define CPUCAP_NOTIFY_DETAIL_NAME cpucap_notif_detail
#define CPUCAP_NOTIFY_DEBUG_NAME cpucap_notif_debug

#define CPUCAP_ADJUST_QUERY_BIT 5
#define CPUCAP_ADJUST_QUERY_SIZE (1 << CPUCAP_ADJUST_QUERY_BIT)
#define CPUCAP_ADJUST_QUERY_SIZE_MASK (CPUCAP_ADJUST_QUERY_SIZE - 1)

#define  PERMILLAGE_MODE 1000
#define  PERCENTAGE_MODE 100
/*RSC_DEFAULT_MAX_PERCENT should be 1000 or 100*/
#define RSC_DEFAULT_MAX_PERCENT PERMILLAGE_MODE
unsigned int MAX_CPUCAP_PERCENT = RSC_DEFAULT_MAX_PERCENT;

/*see cpucap_mode_str */
typedef enum {
	CPUCAP_NORMAL_MODE = 0,
	CPUCAP_DETAIL_MODE,
	CPUCAP_DEBUG_MODE,
	CPUCAP_END_MODE
} cpucap_mode_t;

static const char *cpucap_mode_str[CPUCAP_END_MODE] = {
	[CPUCAP_NORMAL_MODE] = __stringify(CPUCAP_NOTIFY_NAME),
	[CPUCAP_DETAIL_MODE] = __stringify(CPUCAP_NOTIFY_DETAIL_NAME),
	[CPUCAP_DEBUG_MODE] = __stringify(CPUCAP_NOTIFY_DEBUG_NAME),
};

#define RSC_S_TO_US(s)	(s * 1000 * 1000)
#define RSC_NS_TO_US(s)	(s / 1000)

#define rsc_abs_diff(a, b) ({			\
	typeof(a) __a = (a);				\
	typeof(b) __b = (b);				\
	(void) (&__a == &__b);				\
	__a > __b ? (__a - __b) : (__b - __a); })

#if 0
typedef enum {
	CL_ZERO = 0,
	CL_ONE,
	CL_END,
} cluster_type;

struct rsc_cpucap_adj_t {
	rsc_cpucap_adj_type mode;
	struct cpumask cpumask;
	unsigned int cur_freq, min_freq, max_freq, max_mitigated_freq;
	int cpu;
	/*cpu offline*/
	struct cpumask offline;
};
#endif

/*
  for atomic_t write_read_bit[CPUCAP_ADJUST_QUERY_SIZE];
  low 16bit for write check, high 16bit for read check
*/
#define WRITE_BIT_OFFSET 0
#define READ_BIT_OFFSET 16
#define READ_BIT_VALUE ((unsigned int)(1 << READ_BIT_OFFSET))
#define WRITE_BIT_MASK ((unsigned int)(READ_BIT_VALUE - 1))
#define READ_BIT_MASK ((unsigned int)(~WRITE_BIT_MASK))

struct cpucap_adj_info_st {
	struct cpufreq_adj_cell data[CPUCAP_ADJUST_QUERY_SIZE];
#ifndef RSC_CPUCAP_NOLOCK
	/*struct mutex lock;*/
	spinlock_t spinlock;
#endif
	unsigned int write_pos;
	atomic_t read_pos;
	atomic_t last_pos;
	unsigned int ever_read;
#ifdef RSC_CPUCAP_NOLOCK
	/*low 16bit for write check, high 16bit for read check*/
	atomic_t write_read_bit[CPUCAP_ADJUST_QUERY_SIZE];
#endif
	int is_full;
	int ever_adj;
	unsigned int que_size;
	unsigned int que_size_mask;
	atomic_t totalcnt;
	atomic_t capcnt;
	atomic_t all_notify_cnt;
	atomic_t cap_notify_cnt;
};

extern unsigned long arch_get_cpu_efficiency(int cpu);

/*
	the definition is so complex for fixing the check patch error :
	Macros with complex values should be	enclosed in parentheses
	scripts/checkpatch.pl xxx.patch
*/
#ifndef cpumask_pr_args
#define cpumask_pr_args(maskp)													\
/*	(																			\
		do {	(*/																	\
			nr_cpu_ids, cpumask_bits(maskp) 									\
/*		) } while (0)																\
	) */
#endif

unsigned long rsc_total_cpu_efficiency;
unsigned long rsc_min_max_freq = UINT_MAX;
unsigned long rsc_max_max_freq;
unsigned long rsc_min_freq_cpu;
unsigned long rsc_max_freq_cpu;

unsigned long rsc_min_possible_efficiency = UINT_MAX;
#ifdef RSC_RECORD_BIGCORE_USAGE
unsigned long rsc_max_possible_efficiency;
unsigned long rsc_top_possible_efficiency;
struct cpumask rsc_cpu_bigcore;
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
struct cpumask rsc_cpu_littlecore;
struct cpumask rsc_cpu_all = CPU_MASK_ALL;
#endif
#endif

static int rsc_maxcpu;
int rsc_cluster_num;
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
int rsc_bigcore_num;
#endif

/*
 * Increase resolution of cpu_capacity calculations
 */
#ifndef SCHED_CAPACITY_SHIFT
#define SCHED_CAPACITY_SHIFT	10
#endif

#ifndef SCHED_CAPACITY_SCALE
#define SCHED_CAPACITY_SCALE	(1L << SCHED_CAPACITY_SHIFT)
#endif

#ifndef reinit_completion
#define INIT_COMPLETION(x)	((x).done = 0)
#define reinit_completion(x) INIT_COMPLETION(*(x))
#endif

DEFINE_PER_CPU(unsigned long, rsc_scale_cpu_efficiency) = SCHED_CAPACITY_SCALE;
DEFINE_PER_CPU(unsigned long, rsc_scale_cpu_freq) = 1;
DEFINE_PER_CPU(unsigned long, rsc_cpu_maxfreq) = 1;

DEFINE_PER_CPU(int, cpu_to_cluster_map);
int cluster_pol_cpu[CL_END] = {
	[0 ... CL_END - 1] = -1,
};


static struct cpucap_adj_info_st rsc_maxcap = {
#ifndef RSC_CPUCAP_NOLOCK
		/*.lock = __MUTEX_INITIALIZER(rsc_maxcap.lock),*/
		.spinlock = __SPIN_LOCK_UNLOCKED(rsc_maxcap.spinlock),
#endif
		.last_pos = ATOMIC_INIT(-1),
		.que_size = CPUCAP_ADJUST_QUERY_SIZE,
		.que_size_mask = CPUCAP_ADJUST_QUERY_SIZE_MASK,
};

struct cpufreq_adj_cell rsc_cpu_cluster[CL_END] = {
	[0 ... CL_END - 1] = {
		.max_freq = UINT_MAX,
		.max_mitigated_freq = UINT_MAX,
	}
};

struct cpufreq_freqs rsc_cpu_freqs[CL_END];
u32 rsc_max_freq[CL_END];
u32 rsc_max_mitigated_freq[CL_END];

int rsc_cpucap_initialized;

static char *cpucap_adjust_reason[LAST_RSC_ADJUST] = {
	"policy adjust",
	"lmh_dcvs adjust",
	"lmh_setlim adjust",
	"cpu_down adjust",
	"cpu_up adjust",
	"thermal_isolate",
	"thermal_unisolate"
};

static int RSC_STACK_PRINT_ENTRYS = 20;
#define CPUCAP_DETAIL_SHOW_ALL	0
#define CPUCAP_DETAIL_SHOW_LAST	1
static int rsc_cpucap_detail_mode = CPUCAP_DETAIL_SHOW_ALL;

static int rsc_cpucap_debug_go;
static int rsc_cpucap_debug_start;
static int rsc_cpucap_debug_count;
static int rsc_cpucap_debug_current;
static int rsc_cpucap_debug_showcount;

#if (RSC_DEFAULT_MAX_PERCENT == PERMILLAGE_MODE)
static int notify_change_percent = 100/*25*/;
#elif (RSC_DEFAULT_MAX_PERCENT == PERCENTAGE_MODE)
static int notify_change_percent = 10/*3*/;
#endif

enum {
	/*
	   when touch the display panel, app will set the minfreq value to
	   upper freq(maybe maxfreq), at the same time, it will let maxfreq == minfreq.
	   see cpufreq_verify_within_limits and  set_cpu_min_freq.
	   we should ingnore these notify!
	 */
	INGNORE_PERFD_SET_MINFREQ = 1 << 0,
	/*
		when turn off display, system_server process will offline cpu 1 ~ 7 and online cpu 1 ~ 7
		about every 600 seconds, it will notify much message to user space and print too much log!
		we donot want to print it.
		call backtrace:
		state_store ->	pm_suspend -> suspend_devices_and_enter -> suspend_enter ->
		  disable_nonboot_cpus -> _cpu_down
		or
		  enable_nonboot_cpus -> _cpu_up
	*/
	INGNORE_SYSTEM_SERVER_OFFLINE_CPU = 1 << 1,
	/* add iterm here! */
	INGNORE_LAST_ITEM,
};
static unsigned int rsc_notify_mask = INGNORE_PERFD_SET_MINFREQ | INGNORE_SYSTEM_SERVER_OFFLINE_CPU;

#define RSC_SHOW_WAIT_TIME_MS	30

DEFINE_SPINLOCK(last_percent_spinlock);
static int last_update_percent = RSC_DEFAULT_MAX_PERCENT;

int rsc_cpucap_percent = RSC_DEFAULT_MAX_PERCENT;

struct completion cpucap_notify_complete;
static struct task_struct *cpucap_task;

static atomic_t cpucap_cnt = ATOMIC_INIT(0);
static atomic_t cpucap_debug_cnt = ATOMIC_INIT(0);
static atomic_t lastone_notify =  ATOMIC_INIT(0);

/*
   for update the last capability percent
*/
static struct delayed_work rsc_notify_delayed_work;
static unsigned long rsc_notify_delayed_work_timeout = RSC_S_TO_US(30);/*us, 30s*/

#if (RSC_DEFAULT_MAX_PERCENT == PERMILLAGE_MODE)
static int last_show_percent = PERMILLAGE_MODE;
#elif (RSC_DEFAULT_MAX_PERCENT == PERCENTAGE_MODE)
static int last_show_percent = PERCENTAGE_MODE;
#endif

static DEFINE_MUTEX(cpucap_notif_debug_mutex);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
extern struct cpumask cpus_isolated_by_thermal;
#endif

static void rsc_notify_delayed_work_fn(struct work_struct *work)
{

	struct cpucap_adj_info_st *rsc_cap;
	int pos, new_percent;

	rsc_cap = &rsc_maxcap;
	/*pos = atomic_read(&rsc_cap->last_pos) % rsc_cap->que_size;*/
	pos = atomic_read(&rsc_cap->last_pos) & rsc_cap->que_size_mask;
	new_percent = rsc_cap->data[pos].percent;
	/*
	  Could not use  last_show_percent to compare.
	  Consider read cpu capability task is blocked long time.
	  1000(last_show_percent) -> 900(block) -> 950(block) -> 982(block)
	  -> 1000(new_percent, donot notify),
	  at this case: last_show_percent == new_percent,
	  but we also need to do notify_delayed_work.

	  last_update_percent != new_percent fix net scenario:
		902 -> 1000(last_show_percent) -> 982(donot notify) ->
		1000(donot notify)	... -> 982(donot notify) -> 1000(should not notify?)
	*/
	if (/*last_show_percent != new_percent*/last_update_percent != new_percent)/*notify only percent change!*/
		sysfs_notify(rsc_root_dir, NULL, __stringify(CPUCAP_NOTIFY_NAME));
}

static inline unsigned long get_scale_cpu_efficiency(int cpu)
{
	return ((arch_get_cpu_efficiency(cpu) << 10) / rsc_min_possible_efficiency);
}

static inline unsigned long get_scale_cpu_freq(unsigned long freq)
{
	return ((freq << 10) / rsc_min_max_freq);
}

static __ref int do_cpucap_notify(void *data)
{
	int ret = 0;
#ifdef CONFIG_RSC_TASK_CPU_USAGE
	long comret;
	long left = (CPU_USAGE_INIT_PERIOD > 0)?:MAX_SCHEDULE_TIMEOUT;
#endif

/*
	struct sched_param param = {.sched_priority = MAX_RT_PRIO-2};

	sched_setscheduler(current, SCHED_FIFO, &param);
*/
	while (!kthread_should_stop()) {
#ifdef CONFIG_RSC_TASK_CPU_USAGE
		if (left) {
			do {
				rsc_dbg(CPU_TASK_USAGE, "completion: left: %ld\n", left);
				comret = wait_for_completion_interruptible_timeout(&cpucap_notify_complete, left);
				rsc_dbg(CPU_TASK_USAGE, "completion: comret: %ld\n", comret);
			} while (comret < 0);
		} else
			comret = 0;

		left = rsc_cpu_usage_notify(comret);

		if (!comret)
			continue;
#else
	while (wait_for_completion_interruptible(&cpucap_notify_complete) != 0)
		;

	reinit_completion(&cpucap_notify_complete);
#endif

		/*sysfs_notify and cancel_delayed_work_sync could not call in atomic context*/
		cancel_delayed_work_sync(&rsc_notify_delayed_work);

		if (atomic_read(&lastone_notify))
			schedule_delayed_work(&rsc_notify_delayed_work, usecs_to_jiffies(rsc_notify_delayed_work_timeout));

		rsc_dbg(CPU_CAP,
		/*rsc_info(*/
				"%s: last_percent: %4d debug_cnt: %d cnt: %d lastone_notify: %d t: %12llu\n",
				__func__, last_update_percent, atomic_read(&cpucap_debug_cnt),
				atomic_read(&cpucap_cnt), atomic_read(&lastone_notify), RSC_NS_TO_US(local_clock()));
		if (atomic_read(&cpucap_debug_cnt) > 0) {
			do {
				atomic_inc(&rsc_maxcap.all_notify_cnt);
				sysfs_notify(rsc_root_dir, NULL, __stringify(CPUCAP_NOTIFY_DETAIL_NAME));
			} while (atomic_dec_return(&cpucap_debug_cnt));
		}

		if (atomic_read(&cpucap_cnt) > 0) {
			do {
				atomic_inc(&rsc_maxcap.cap_notify_cnt);
				sysfs_notify(rsc_root_dir, NULL, __stringify(CPUCAP_NOTIFY_NAME));
			} while (atomic_dec_return(&cpucap_cnt));
		}
	}

	return ret;
}

#ifdef RSC_CPUCAP_NOLOCK
/*for producer consumer system, no lock operation*/
static unsigned int rsc_cpucap_enqueue(struct cpucap_adj_info_st *p)
{
	unsigned int cur_in, new_in, cnt;
	unsigned int old_in;
	unsigned long flags;
	unsigned int loop = 0, mloop = 0;

	do {
next:
		/*disable irq to prevent deadlock*/
		local_irq_save(flags);
		cur_in  = p->write_pos;
		/*new_in = (cur_in + 1) % p->que_size;*/
		new_in = (cur_in + 1) & p->que_size_mask;

		cnt = (unsigned int)atomic_inc_return(&p->write_read_bit[cur_in]);
		if ((cnt & READ_BIT_MASK) > 0) {
			atomic_dec(&p->write_read_bit[cur_in]);
			/*somebody(task must be in another cpu, because disable irq) must be  reading the data. try again! */
			loop++;
			local_irq_restore(flags);
			goto next;
		}

		if (cur_in == atomic_read(&p->last_pos)) {
			rsc_err("cur_in == p->last_pos(%2d) Ring buffer has no enough! "
				"Should be more than %d threads to be enqueueing now! total: %5u write_read_bit: %6x\n",
				cur_in, p->que_size, atomic_read(&p->totalcnt), cnt);
		}
		old_in = cmpxchg(&p->write_pos, cur_in, new_in);
		if (old_in != cur_in)
			atomic_dec(&p->write_read_bit[cur_in]);
		local_irq_restore(flags);
		mloop++;
	/*if old_in == cur_in,
	there is no other thread to modify p->write_pos;*/
	} while (old_in != cur_in);

	if (loop)
		rsc_info("pos(%2d) is being read, try again! cpu: %d qsize: %2d total: %5u write_read_bit: %6x mloop:%u loop: %u\n",
			cur_in, raw_smp_processor_id(), p->que_size, atomic_read(&p->totalcnt), cnt, mloop, loop);
	return cur_in;
}
#if 0
static unsigned int rsc_cpucap_dequeue(struct cpucap_adj_info_st *p)
{
	unsigned int cur_in;
	unsigned int cur_out;
	unsigned int old_out;

	do {
		cur_out = p->read_pos;
		cur_in = p->write_pos;

		old_out = cmpxchg(&p->read_pos, cur_out, cur_out + 1);
	} while (old_out != cur_out)

	return cur_out;
}
#endif
#endif

static inline int show_one_notify(int i, int begin, char *buf, int bufsize, struct cpucap_adj_info_st *inf, cpucap_mode_t debug_mode)
{
	__kernel_suseconds_t us;
	unsigned int fromfreq, tofreq;
	unsigned int reqfreq;
	int cpu;
	rsc_cpucap_adj_type mode;
	struct stack_trace *s;
	int j, diff, clust;
	char *reason;
	unsigned long *entries;
	char notify;
	int ret = 0, cur = 0;
	const char *str;
	int error = 0;
#ifdef RSC_CPUCAP_NOLOCK
	int renotify, cnt;
	unsigned long  flags;
	unsigned int loop = 0;
#endif

	str = cpucap_mode_str[debug_mode];

	if (!inf->data[begin].is_valid) {
		rsc_info("%s: i: %d entry: %2d is not valid. mode: %s(%d)\n",
				__func__, i, begin, str, debug_mode);
		return 0;
	}

#ifdef RSC_CPUCAP_NOLOCK
renotify = 0;
again:
	/*priority of writer is higher that read, prevent writer  starvation!*/
	cnt = (unsigned int)atomic_read(&inf->write_read_bit[begin]);
	if (cnt & WRITE_BIT_MASK) {
		if (!loop)
			rsc_info("%s: cpu: %d somebody is writing pos: %2d , renotify it. "
				"inf->write_read_bit: %6x debug_mode: %d loop: %u\n",
				__func__, raw_smp_processor_id(), begin, cnt, debug_mode, loop);
		loop++;
		/*wait 30ms (RSC_SHOW_WAIT_TIME_MS)*/
		schedule_timeout(msecs_to_jiffies(RSC_SHOW_WAIT_TIME_MS));
		goto again;
	}

	local_irq_save(flags);
	cnt = (unsigned int)atomic_add_return(READ_BIT_VALUE, &inf->write_read_bit[begin]);
	if (!(cnt & WRITE_BIT_MASK)) {
#endif
		if (debug_mode == CPUCAP_NORMAL_MODE) {
			last_show_percent = inf->data[begin].percent;
			cur = snprintf(buf + ret, bufsize - ret, "%d\n", inf->data[begin].percent);
			ret += cur;
		} else {/*CPUCAP_DETAIL_MODE or CPUCAP_DEBUG_MODE*/
			cpu = cpumask_first(&inf->data[begin].cpumask);
			if (cpu >= nr_cpumask_bits)	{
				rsc_info("%s: cpu: %d(max: %d) cpumask empty or error: %*pbl.\n",
				__func__, cpu, nr_cpumask_bits,  cpumask_pr_args(&inf->data[begin].cpumask));
				cpu = 0;
				error = 1;
			}
			clust = per_cpu(cpu_to_cluster_map, cpu);
			us = inf->data[begin].tv.tv_usec;
			mode = inf->data[begin].mode;
			reason = cpucap_adjust_reason[mode];
			/*In platform sdm660, sdm835 ,for freq adjust type "thermal max"(THERMAL_ADJUST_MAX) and
			"cooling max"(COOLING_ADJUST_MAX) ,  freq is directly limit by hardware.
			you can see update_cpu_freq->msm_lmh_dcvs_update->msm_lmh_dcvs_write and
			cpufreq_set_cur_state->lmh_set_max_limit->msm_lmh_dcvs_write.
			Another question is how freq adjust work in low battery: lmh_activate_trip->msm_lmh_dcvs_write*/
			if (mode == LMH_DCVS_ADJUST_FREQ) {
				fromfreq = inf->data[begin].last_max_mitifreq;
				tofreq = inf->data[begin].max_mitigated_freq;
				reqfreq = rsc_cpu_cluster[clust].max_freq;
			} else if (mode == POLICY_ADJUST_FREQ) {
				fromfreq = inf->data[begin].last_max_freq;
				tofreq = inf->data[begin].max_freq;
				reqfreq = rsc_cpu_cluster[clust].max_mitigated_freq;
			} else if (mode >= RSC_CPU_DOWN && mode <= RSC_CPU_UNISO)/*(mode == RSC_CPU_UP || mode == RSC_CPU_DOWN)*/ {
				fromfreq = rsc_cpu_cluster[clust].last_max_mitifreq;
				tofreq = rsc_cpu_cluster[clust].max_mitigated_freq;
				reqfreq = rsc_cpu_cluster[clust].max_freq;
			} else {
				rsc_err("%s: mode:%d error\n", __func__, inf->data[begin].mode);
				reason = "unknown mode";
				fromfreq = reqfreq = tofreq = 0;
			}
			diff = inf->data[begin].percent - inf->data[begin].last_percent;
			notify = (abs(diff) >=  notify_change_percent)?'N':'D';
			#define RSC_PRINT_DETAIL "%4d %4d(%c %4d | %3d) seq: %5u q: %2d clu: %d %s, freq(kHz) from %8u to %10u (rf: %10u),"\
				" reason: %18s(%d), task: %16s pid: %5d, tl: %5u %5u %5u %5u f: %d, time: %6llds %6lldus cpum: %*pbl "\
				"offline: %*pbl cpu: %d ent: %d (%d - %d, %d - %d) pos: %2d\n",\
				inf->data[begin].percent, inf->data[begin].last_percent, notify, diff,\
				notify_change_percent, inf->data[begin].seq, i, clust, str, fromfreq, tofreq, reqfreq,\
				reason, mode, inf->data[begin].comm, inf->data[begin].pid, atomic_read(&inf->totalcnt), atomic_read(&inf->all_notify_cnt),\
				atomic_read(&inf->capcnt), atomic_read(&inf->cap_notify_cnt), inf->is_full,\
				(u64)inf->data[begin].tv.tv_sec, (u64)us, cpumask_pr_args(&inf->data[begin].cpumask),\
				cpumask_pr_args(&inf->data[begin].offline), inf->data[begin].cpu, RSC_STACK_PRINT_ENTRYS,\
				rsc_cpu_cluster[0].max_freq, rsc_cpu_cluster[0].max_mitigated_freq,\
				rsc_cpu_cluster[1].max_freq, rsc_cpu_cluster[1].max_mitigated_freq, begin

			cur = snprintf(buf + ret, bufsize - ret, RSC_PRINT_DETAIL);
			if (error)
				rsc_info(RSC_PRINT_DETAIL);
			else
				rsc_dbg(CPU_CAP, RSC_PRINT_DETAIL);
			ret += cur;

			s = &inf->data[begin].callstack;
			entries = inf->data[begin].stack_entries;

			j = 0;/*CPUCAP_DEBUG_MODE always ,show all call backtrace*/
			if (CPUCAP_DETAIL_MODE == debug_mode) {
				/*could not print too more callstack, because buffer size is PAGE_SIZE*/
				if (s->nr_entries >= RSC_STACK_PRINT_ENTRYS) {
					j = s->nr_entries - RSC_STACK_PRINT_ENTRYS;
					/*could not use s->entries[s->nr_entries - 1],
					  because s->entries maybe set to NULL, because no lock!*/
					if (entries[s->nr_entries - 1] == ULONG_MAX)
						if (j > 1)
							j--;
				} else
					j = 0;
			}

			for (; j < s->nr_entries; j++) {
				/*could not use s->entries[j],
				  because s->entries maybe set to NULL, because no lock!*/
				if (entries[j] == ULONG_MAX)
					break;

				#define RSC_PRINT_BACKTRACE 	"%2d)	"/*"[<%p>] "*/"%ps\n",\
						j, /*(void *)entries[j], */(void *)entries[j]

				if (error)
					rsc_info(RSC_PRINT_BACKTRACE);
				else
					rsc_dbg(CPU_CAP, RSC_PRINT_BACKTRACE);
				cur = snprintf(buf + ret, bufsize - ret, RSC_PRINT_BACKTRACE);
				if (cur <= 0)
					break;
				ret += cur;
			}
		}
#ifdef RSC_CPUCAP_NOLOCK
	} else
		renotify = 1;

	atomic_sub(READ_BIT_VALUE, &inf->write_read_bit[begin]);
	local_irq_restore(flags);

	if (renotify) {
		renotify = 0;
		goto again;
	}
	if (loop)
		rsc_info("%s: cpu: %d return somebody is writing pos: %2d , renotify it. "
				"inf->write_read_bit: %6x debug_mode: %d loop: %u\n",
				__func__, raw_smp_processor_id(), begin, cnt, debug_mode, loop);
#endif

return ret;
}

static int reset_rsc_cpucap_debug_notif(struct cpucap_adj_info_st *inf)
{
	int ret = 0, last_pos, start = -1, cnt = 0;

	last_pos = atomic_read(&inf->last_pos);

	if (inf->ever_adj) {
		if (atomic_read(&inf->totalcnt) >= inf->que_size) {
			/*start = (last_pos + 1) % inf->que_size;*/
			start = (last_pos + 1) & inf->que_size_mask;
			cnt = inf->que_size;
		} else {
			start = 0;
			cnt = last_pos + 1;
		}
		rsc_cpucap_debug_start = start;
		rsc_cpucap_debug_current = start;
		rsc_cpucap_debug_showcount	= 0;
		/*In nolock mode(#defined RSC_CPUCAP_NOLOCK),
		 maybe the array data between start ~ start + last be crashed,
		 but we have inf->write_read_bit to check, so it is also saft. */
		rsc_cpucap_debug_count = cnt;
		rsc_cpucap_debug_go = 1;
	} else
		rsc_cpucap_debug_go = 0;

	rsc_dbg(CPU_CAP,
	/*rsc_info(*/
		"%s: inf->ever_adj = %u start: %d cnt: %d\n", __func__, inf->ever_adj, start, cnt);

	return ret;
}

static int do_rsc_cpucap_notif_show(char *buf, struct cpucap_adj_info_st *inf, cpucap_mode_t debug_mode)
{
	int ret = 0, cur = 0;
	int i;
	int start = 0, last = 0;
	int maxadj = 0;
	int read_pos, last_pos;
	const char *str;

#ifndef RSC_CPUCAP_NOLOCK
	unsigned long  flags;

	/*we could not use mutex, because it maybe in atomic context.*/
	/*mutex_lock(&inf->lock);*/
	spin_lock_irqsave(&inf->spinlock, flags);
#endif

	str = cpucap_mode_str[debug_mode];

	if (inf == &rsc_maxcap)
		maxadj = 1;

	last_pos = atomic_read(&inf->last_pos);

	if (unlikely(last_pos < 0)) {
		if (inf->ever_adj)
			rsc_err("%s: atomic_read(&inf->last_pos): %d < 0 debug_mode:%d\n",
				__func__, last_pos, debug_mode);
			goto out;
	}

	if (CPUCAP_NORMAL_MODE == debug_mode) {
		/*only show the last one in normal or detail mode*/
		if (inf->ever_adj) {
			start = last_pos;
			last = 1;
		}
	} else if (CPUCAP_DETAIL_MODE == debug_mode) {
		if (CPUCAP_DETAIL_SHOW_ALL == rsc_cpucap_detail_mode) {
			read_pos = atomic_read(&inf->read_pos);
			atomic_set(&inf->read_pos, last_pos);
			if (inf->ever_adj) {
				/*first time read*/
				if (!inf->ever_read) {
					inf->ever_read = 1;
					/*full*/
					if (atomic_read(&inf->totalcnt) >= inf->que_size) {
						/*start = (last_pos + 1) % inf->que_size;*/
						start = (last_pos + 1) & inf->que_size_mask;
					} else
						start = 0;
				} else {
					/*start = (read_pos + 1) % inf->que_size;*/
					start = (read_pos + 1) & inf->que_size_mask;
				}

				/*In nolock mode(#defined RSC_CPUCAP_NOLOCK),
				   maybe the array data between start ~ start + last be crashed,
				   but we have inf->write_read_bit to check, so it is also saft.*/
				if (inf->is_full) {
					inf->is_full = 0;
					/*next full!*/
					/*last = ((last_pos + inf->que_size - read_pos) % inf->que_size) + 1;*/
					last = ((last_pos + inf->que_size - read_pos) & inf->que_size_mask) + 1;
				} else {
					/*last = (last_pos + inf->que_size - read_pos) % inf->que_size;*/
					last = (last_pos + inf->que_size - read_pos) & inf->que_size_mask;
				}
			}

		} else
			last = 0;

		if (!last && inf->ever_adj) { /*always show the last one*/
			start = last_pos;
			last = 1;
		}
	/*CPUCAP_DEBUG_MODE*/
	} else {
		mutex_lock(&cpucap_notif_debug_mutex);
		if (!rsc_cpucap_debug_go)
			reset_rsc_cpucap_debug_notif(inf);

		if (rsc_cpucap_debug_go) {
			rsc_cpucap_debug_showcount++;
			if (rsc_cpucap_debug_showcount >= rsc_cpucap_debug_count) {
				rsc_cpucap_debug_go = 0;
			}
			start = rsc_cpucap_debug_current;
			last = 1;
			/*rsc_cpucap_debug_current = (rsc_cpucap_debug_current + 1) % inf->que_size;*/
			rsc_cpucap_debug_current = (rsc_cpucap_debug_current + 1) & inf->que_size_mask;
			rsc_dbg(CPU_CAP,
			/*rsc_info(*/
				"%s: %s %2d start: %2d current: %2d cnt: %2d\n", __func__, str, rsc_cpucap_debug_showcount - 1,
				rsc_cpucap_debug_start, start, rsc_cpucap_debug_count);
		}
		mutex_unlock(&cpucap_notif_debug_mutex);
	}

	for (i = 0; i < last; i++) {
		int begin;

		begin = start + i;
		/*begin %= inf->que_size;*/
		begin &= inf->que_size_mask;
		cur = show_one_notify(i, begin, buf + ret, PAGE_SIZE -  ret, inf, debug_mode);
		ret += cur;
	}

out:
	if (inf == &rsc_maxcap)
		rsc_dbg(CPU_CAP, "%s: str:%s totalcnt:%10u pos:%d is_full:%d ret:%d\n", __func__, str,  atomic_read(&inf->totalcnt), atomic_read(&inf->last_pos), inf->is_full, ret);

	/*show function for sysfs_notify could not return zero!
		or  could not block in poll fucntion, and cause the dead loop: poll -> read ->lseek -> poll -> read ->lseek -> poll ...
		Because,  if return zero, then in next read, read fucntion will alway return 0 in function seq_read(No 9).
		Auctually, do not call fucntion do_cpufreq_adjust_show.  see the next explain 1 - 10
	kernel space code:
		ssize_t seq_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
		{
			struct seq_file *m = file->private_data;
		...
														//4. lseek(fd, 0, SEEK_SET),call poll func and task is poll in poll_schedule_timeout.
														//   SyS_ppoll ->  kernfs_fop_poll (return DEFAULT_POLLMASK, of->event == atomic_read(&on->event))
														//   than call poll_schedule_timeout.

														//5. kernel excute sysfs_notify: add event count(sysfs_notify->kernfs_notify_workfn ->atomic_inc(&on->event))
														//   and wakeup poll task.
// Don't assume *ppos is where we left it
			if (unlikely(*ppos != m->read_pos)) {			//6. do read, in function seq_read, m->read_pos is 0(last call do_cpufreq_adjust_show return 0),
														//   and *ppos is 0, m->index is 1 , so do not entry the if contion and do not call function traverse
														//   if *ppos != m->read_pos,such as  *ppos is 0 (need user space to set *ppos to 0,lseek(fd, 0, SEEK_SET);)
														//   and  m->read_pos is Non-zero(167) that is the last return value of function do_cpufreq_adjust_show.
														//   then m->index will be reset to *ppos that is 0
				while ((err = traverse(m, *ppos)) == -EAGAIN)
					;
				if (err) {
					// With prejudice...
					m->read_pos = 0;
					m->version = 0;
					m->index = 0;
					m->count = 0;
					goto Done;
				} else {
					m->read_pos = *ppos;
				}
			}

			// we need at least one record in buffer
			pos = m->index;								//7. m->index is 1, pos is 1
			p = m->op->start(m, &pos);					//8. kernfs_seq_start return null, so p value is null

			while (1) {
				err = PTR_ERR(p);
				if (!p || IS_ERR(p))						//9. p is null,then break,do not call  m->op->show(m, p) that is function kernfs_seq_show,
					break;								//   donot set of->event = atomic_read(&of->kn->attr.open->event) that is set in kernfs_seq_show
														//10.lseek(fd, 0, SEEK_SET),call poll func in user space.
														//   SyS_ppoll -> kernfs_fop_poll (return DEFAULT_POLLMASK|POLLERR|POLLPRI;of->event != atomic_read(&on->event))
														//   not call poll_schedule_timeout and then call read. That is the dead loop!!
				err = m->op->show(m, p);
				if (err < 0)
					break;
				if (unlikely(err))
					m->count = 0;
				if (unlikely(!m->count)) {					//1.after call m->op->show (kernfs_seq_show) ,
														//  m->index is 0,do_cpufreq_adjust_show return 0, then m->count is 0,
					p = m->op->next(m, p, &pos);			//2.pos val form 0 to 1
					m->index = pos;						//3.m->index val form 0 to 1, p is null, m->read_pos is 0 (It is the key. read_pos is 0 and cause the dead loop),
														//  *ppos is 0, then return user space and call poll func
					continue;
				}
				if (m->count < m->size)
					goto Fill;
				m->op->stop(m, p);
				kvfree(m->buf);
				m->count = 0;
				m->buf = seq_buf_alloc(m->size <<= 1);
				if (!m->buf)
					goto Enomem;
				m->version = 0;
				pos = m->index;
				p = m->op->start(m, &pos);
			}
			m->op->stop(m, p);
			m->count = 0;
			goto Done;

	user space code:
		while (1) {
			int polret;
			fds.fd = fd;
			fds.events = POLLERR|POLLPRI;
			fds.revents = 0;
			polret = poll(&fds, 1, -1);
			if (polret == -1) {
				printf("Error in uevent poll %s.\n", uevent);
				break;
			}
			memset(buf, 0 , sizeof(buf));
			ret = read(fd, buf, sizeof(buf));
			printf("poll file:%s read %d bytes polret:%d\n", uevent, ret, polret);
			if (ret < 0)
				printf("sysfs_notify sysfs[%s] read error %d", uevent, errno);

			lseek(fd, 0, SEEK_SET);
			printf("%s:buf is :\n%s\n",__func__, buf);
		}

	call stack:
		do_cpufreq_adjust_show
		show_rsc_maxfreq_notif+0x3c/0x74
		kobj_attr_show+0x18/0x28
		sysfs_kf_seq_show+0xa4/0x110
		kernfs_seq_show+0x44/0x50
		seq_read+0x190/0x3b4
		kernfs_fop_read+0x58/0x17c
		__vfs_read+0x48/0xe8
		vfs_read+0x90/0x108
		SyS_read+0xbc/0x164
		cpu_switch_to+0x210/0x3e0
	*/
	if (!ret) {
		if (debug_mode == CPUCAP_NORMAL_MODE) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%3d\n", last_show_percent);
			ret += cur;
		} else {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%3d Empty! There is not any %s setting.\n", last_show_percent, str);
			ret += cur;
		}
	}

#ifndef RSC_CPUCAP_NOLOCK
	spin_unlock_irqrestore(&inf->spinlock, flags);
	/*mutex_unlock(&inf->lock);*/
#endif
	return ret;
}

static ssize_t show_rsc_cpucap_notif(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	rsc_dbg(CPU_CAP, "%s: %s rsc_cpu_initialized:%d\n", __func__, __stringify(CPUCAP_NOTIFY_NAME), rsc_cpucap_initialized);

	if (unlikely(!rsc_cpucap_initialized))
		return 0;

	ret = do_rsc_cpucap_notif_show(buf, &rsc_maxcap, CPUCAP_NORMAL_MODE);

	return ret;
}

static ssize_t store_rsc_cpucap_notif(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int level, maxp, enale;
	unsigned int prev;

	ret = sscanf(buf, "%u %u %u", &maxp, &level, &enale);
	if (ret < 2 && ret > 3)
		goto fail;

	if (maxp == PERMILLAGE_MODE) {
		if (level > PERMILLAGE_MODE)
			goto fail;
	} else if (maxp == PERCENTAGE_MODE) {
		if (level > PERCENTAGE_MODE)
			goto fail;
	} else {
		goto fail;
	}

	prev = rsc_notify_mask;
	if (3 == ret) {
		if (enale & (~(((INGNORE_LAST_ITEM - 1) << 1) - 1)))
			goto fail;
		rsc_notify_mask = enale;
	} else
		enale = rsc_notify_mask;

	MAX_CPUCAP_PERCENT = maxp;
	notify_change_percent = level;

	rsc_info("%s: ret: %d maxpercent: %u notifpercent: %u rsc_notify_mask: %x -> %x buf: %s\n",
		__func__, ret, maxp, level, prev, enale, buf);

	return count;

fail:
	rsc_err("usage: echo MAXPERCENT(1000) notifypercent(26) > /sys/rsc/cpucap_notif , maxpercent: %u notifpercent: %u buf: %s\n",
		MAX_CPUCAP_PERCENT, notify_change_percent, buf);

	return -EINVAL;
}

static ssize_t show_rsc_cpucap_detail_notif(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	rsc_dbg(CPU_CAP, "%s: %s rsc_cpu_initialized:%d\n", __func__, __stringify(CPUCAP_NOTIFY_DETAIL_NAME), rsc_cpucap_initialized);

	if (unlikely(!rsc_cpucap_initialized))
		return 0;

	ret = do_rsc_cpucap_notif_show(buf, &rsc_maxcap, CPUCAP_DETAIL_MODE);

	return ret;
}

static ssize_t store_rsc_cpucap_detail_notif(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int mode = CPUCAP_DETAIL_SHOW_ALL;
	unsigned int level;

	ret = sscanf(buf, "%u %u", &level, &mode);
	if (ret != 1 && ret != 2)
		goto fail;

	if (level <= RSC_MAX_STACK_ENRTYS)
		RSC_STACK_PRINT_ENTRYS = level;
	else
		goto fail;

	if (ret == 2) {
		if (mode == CPUCAP_DETAIL_SHOW_ALL || mode == CPUCAP_DETAIL_SHOW_LAST)
			rsc_cpucap_detail_mode = mode;
		else
			goto fail;
	}
	rsc_info("%s: RSC_STACK_PRINT_ENTRYS = %u rsc_cpucap_detail_mode: %u\n", __func__, level, mode);

	return count;

fail:
	rsc_err("usage: echo maxentry(0 - %u) mode(0 or 1) > /sys/rsc/cpucap_notif_detail\n", RSC_MAX_STACK_ENRTYS);

	return -EINVAL;
}

static ssize_t show_rsc_cpucap_debug_notif(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	rsc_dbg(CPU_CAP, "%s: %s rsc_cpu_initialized:%d\n", __func__, __stringify(CPUCAP_NOTIFY_DEBUG_NAME), rsc_cpucap_initialized);

	if (unlikely(!rsc_cpucap_initialized))
		return 0;

	ret = do_rsc_cpucap_notif_show(buf, &rsc_maxcap, CPUCAP_DEBUG_MODE);

	return ret;
}

static ssize_t store_rsc_cpucap_debug_notif(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int level;
	struct cpucap_adj_info_st *inf = &rsc_maxcap;

	ret = sscanf(buf, "%u", &level);
	if (ret != 1)
		goto fail;

	if (level != 1)
		goto fail;

	mutex_lock(&cpucap_notif_debug_mutex);
	reset_rsc_cpucap_debug_notif(inf);
	mutex_unlock(&cpucap_notif_debug_mutex);

	return count;

fail:
	rsc_err("usage: echo 1  > /sys/rsc/cpucap_notif_debug\n");

	return -EINVAL;
}

static int rsc_cal_cpucap_percent(struct cpufreq_adj_cell *pd)
{
	unsigned long eff, newcap;
	unsigned long freq_cap[CL_END], freq;
	int percent;
	unsigned int lastfreq, cpu, clust;
	int i, j, k;

	struct cpufreq_adj_cell *c_data = rsc_cpu_cluster;

	for (i = 0; i < rsc_cluster_num; i++) {
		if ((c_data[i].max_freq != UINT_MAX) || (c_data[i].max_mitigated_freq != UINT_MAX)) {
			lastfreq = min(c_data[i].max_freq, c_data[i].max_mitigated_freq);
#if 1
			/*
			* fix wrong thermal freq and percent more than 100%.
			*/
			cpu = cluster_pol_cpu[i];
			freq_cap[i] = min(get_scale_cpu_freq(lastfreq), per_cpu(rsc_scale_cpu_freq, cpu));
			rsc_dbg(CPU_CAP,
			/*rsc_info(*/
				"clust: %d cpu: %d freqcap: %ld(limit: %ld - scal: %ld ) softlimitfreq:%d thermallimitfreq: %d\n",
				i, cpu, freq_cap[i], get_scale_cpu_freq(lastfreq), per_cpu(rsc_scale_cpu_freq, cpu),
				c_data[i].max_freq, c_data[i].max_mitigated_freq);
#else
			freq_cap[i] = get_scale_cpu_freq(lastfreq);
#endif
		} else {
			cpu = cluster_pol_cpu[i];
			if (cpu >= nr_cpumask_bits)	{
				rsc_err("%s: i: %d cpu: %d could not bigger then %d\n",
					__func__, i, cpu, nr_cpumask_bits);
				dump_stack();
				cpu = 0;
			}
			freq_cap[i] = per_cpu(rsc_scale_cpu_freq, cpu);
			rsc_dbg(CPU_CAP,
			/*rsc_info(*/
				"clust: %d cpu: %d freqcap: %ld nolimit\n",
				i, cpu, freq_cap[i]);
		}
	}

/*
		Thermal adjust the maxfreq and at the same time Governor also adjust maxfreq,
		then it will select the min freq to compute the schedule capability.
			Governor adjust logic:
				cpufreq_update_policy -> cpufreq_set_policy ->
				blocking_notifier_call_chain(&cpufreq_policy_notifier_list, CPUFREQ_NOTIFY, new_policy); ->
				cpufreq_notifier_policy -> update_cpu_cluster_capacity -> compute_capacity ->
				capacity_scale_cpu_freq -> cluster_max_freq
				static inline unsigned int cluster_max_freq(struct sched_cluster *cluster)
				{
					 //Governor and thermal driver don't know the other party's mitigation
					 //voting. So struct cluster saves both and return min() for current
					 // cluster fmax.
					return min(cluster->max_mitigated_freq, cluster->max_freq);
				}

			Thermal adjust logic:(common function is sched_update_cpu_freq_min_max)
				1. first time setting, it will disable irq and user 10ms poll timer, see lmh_dcvs_notify
				2. when current maxfreq is euqal or bigger than
				   hw->max_freq, it will enable irq and disable poll timer, see msm_lmh_dcvs_poll
				3. when current maxfreq is lesser than  hw->max_freq, it will use 10ms poll timer to monitor the freq change.
				   note: set hw maxfreq directly(change the register: hw->osm_hw_reg), it will trigger the irq when irq is enable.
				   msm_lmh_dcvs_write(hw->affinity, MSM_LIMITS_SUB_FN_GENERAL, MSM_LIMITS_DOMAIN_MAX, freq);

				Thermal engine:
					update_cpu_freq->msm_lmh_dcvs_update ->
					msm_lmh_dcvsh_sw_notify(cpu) -> lmh_dcvs_notify ->
					msm_lmh_mitigation_notify -> sched_update_cpu_freq_min_max
					   cluster->max_mitigated_freq = fmax;
					   update_cpu_cluster_capacity(cpus);

					  A. Firstly, set maxfreq call backtrace, use do_freq_mitigation thread
							454 454(25) q: 0 clu: 0 cpucap_notif_detail, freq(kHz) from  1843200 to    1747200 (rf:    1843200), reason:	lmh_dcvs adjust(1),
							task:  msm_thermal:fre, tl:   211 f: 0, time: 1505952173s  57745us cpum: 0-3
							offline: 5-7 cpu: 0 ent: 12 (1843200 - 1747200, 2208000 - 1747200)
							 0) 	rsc_cpucap_adjust_handler
							 1) 	rsc_cpucap_adjust_internal
							 2) 	rsc_cpucap_adjust
							 3) 	sched_update_cpu_freq_min_max
							 4) 	msm_lmh_mitigation_notify
							 5) 	msm_lmh_dcvsh_sw_notify
							 6) 	msm_lmh_dcvs_update
							 7) 	update_cluster_freq
							 8) 	do_freq_mitigation
							 9) 	kthread
							10) 	cpu_switch_to
					   B. Secondarily, set maxfreq call backtrace, use poll timer, set in softirq atomic context.
							472 454(25) q: 0 clu: 0 cpucap_notif_detail, freq(kHz) from  1747200 to    1843200 (rf:    1843200), reason:	lmh_dcvs adjust(1),
							task:		  Thread-3, tl:   212 f: 0, time: 1505952175s  62707us cpum: 0-3
							offline: 5-7 cpu: 0 ent: 12 (1843200 - 1843200, 2208000 - 1747200)
							 1) 	rsc_cpucap_adjust_internal
							 2) 	rsc_cpucap_adjust
							 3) 	sched_update_cpu_freq_min_max
							 4) 	msm_lmh_mitigation_notify
							 5) 	msm_lmh_dcvs_poll
							 6) 	call_timer_fn
							 7) 	run_timer_softirq
							 8) 	__do_softirq
							 9) 	irq_exit
							10) 	__handle_domain_irq
							11) 	gic_handle_irq
							12) 	el0_irq_naked
				irq thread: (command:
						//make the highest freq 1843200
						echo 0 > /sys/class/thermal/cooling_device0/cur_state
						//set freq to 1843200
						echo 3 > /sys/class/thermal/cooling_device0/cur_state)
							note: first time call lmh_dcvs_notify will disable irq( disable_irq_nosync(hw->irq_num))
							and then use softirq(poll timer)   or when current maxfreq is euqal or bigger than
							hw->max_freq, it will enable irq, see msm_lmh_dcvs_poll
							lmh_dcvs_handle_isr -> lmh_dcvs_notify ->
							msm_lmh_mitigation_notify(hw) -> sched_update_cpu_freq_min_max
				softirq(poll timer): (command: echo 5 > /sys/class/thermal/cooling_device0/cur_state)
						msm_lmh_dcvs_poll -> msm_lmh_mitigation_notify(hw) -> sched_update_cpu_freq_min_max
*/
	newcap = 0;
	for_each_possible_cpu(j) {
		if (cpumask_test_cpu(j, &pd->offline)) {
			eff = 0;
			freq = 0;
		} else {
			eff = per_cpu(rsc_scale_cpu_efficiency, j);
			clust = per_cpu(cpu_to_cluster_map, j);
			if (clust >= CL_END) {
				rsc_err("%s: clust %d bigger than %d\n", __func__, clust, CL_END);
				clust = CL_ZERO;
			}
			freq = freq_cap[clust];
		}
		newcap += (((eff /* * 1024/1024 */) * freq) >> 10);
		k = j;
		rsc_dbg(CPU_CAP,
		/*rsc_info(*/
			"j: %d freqcap: %ld(%ld) eff:% ld minf: %ld mineff: %ld newcap: %5ld total:%ld percent: %ld offline: %*pbl cpu: %d\n",
			j, freq, per_cpu(rsc_scale_cpu_freq, j), eff, rsc_min_max_freq,
			rsc_min_possible_efficiency, newcap, rsc_total_cpu_efficiency,
			newcap*100/rsc_total_cpu_efficiency, cpumask_pr_args(&pd->offline), pd->cpu);
	}
#if 0
	/*always show last percent*/
	if (!(rsc_debug & ALL)) {
		j = k;
		rsc_info("j: %d freqcap: %ld(%ld) eff:% ld minf: %ld mineff: %ld newcap: %5ld total:%ld percent: %ld offline: %*pbl cpu : %d\n",
			j, freq, per_cpu(rsc_scale_cpu_freq, j), eff, rsc_min_max_freq,
			rsc_min_possible_efficiency, newcap, rsc_total_cpu_efficiency,
			newcap*100/rsc_total_cpu_efficiency, cpumask_pr_args(&pd->offline), pd->cpu);
	}
#endif
	percent = 0;
	if (rsc_total_cpu_efficiency) {
		/*For sdm660, maxfreq of all 4 little	cpus set form 1843200 to	633600(min) ,percent is 77%, decrease 23%
					 maxfreq of all 4 big	cpus set form 2208000 to   1113600(min) ,percent is 67%, decrease 33%
					 The little cpu's efficiency is 1024, The big cpu's efficiency is 1638.
					 all cpu adjust to lowest freq, cap is 44%,
					 level: 44%, 67%, 77% , 100%
					 current thermal setting lowest capability:
					 offline 3 big cpu 4,5,7 , big cpu (cpu4) set maxfreq to 1747200 and maxfreq of little cpu set form
					 1843200 to 1536000, the percent is 41.5%
					 big 	  cpu capability is 16.0%,
					 little cpu capability is 9% .
					 4 big cpu  freq(kHz) from  2150400 to    2208000, capability increase 2%,such as 98% increase to 100%,
					 that is 	1 big core increase 0.5% freq(kHz) from  2150400 to    2208000
							1 big core increase 1.4% freq(kHz) from  1958400 to    2150400
							1 big core increase 1.8% freq(kHz) from  1843200 to    1958400
							1 big core increase 1.6% freq(kHz) from  1747200 to    1843200
							1 big core increase 3.4% freq(kHz) from  1747200 to    1958400
		 */
		percent = (newcap * MAX_CPUCAP_PERCENT)/rsc_total_cpu_efficiency;
	}

	/*rsc_dbg(CPU_CAP, "minf: %ld mineff: %ld clust: %d newcap: %ld total:%ld percent: %ld cpus_offlined: %x\n",
		rsc_min_max_freq, rsc_min_possible_efficiency, clust, newcap, rsc_total_cpu_efficiency, percent, cpus_offlined);
	*/
	if (percent > MAX_CPUCAP_PERCENT || !percent) {
		rsc_err("%s: percent(%d) error!. minf: %ld mineff: %ld newcap: %ld total:%ld\n",
			 __func__, percent, rsc_min_max_freq, rsc_min_possible_efficiency, newcap, rsc_total_cpu_efficiency);
		percent = MAX_CPUCAP_PERCENT;
	}

	return percent;
}

static int rsc_cpucap_adjust_handler(struct notifier_block *nb,
		unsigned long val, void *v)
{
	rsc_cpucap_adj_type mode = (rsc_cpucap_adj_type)val;
	struct rsc_cpucap_adj_t *policy = (struct rsc_cpucap_adj_t *)v;
	unsigned int orig_max_freq = 0;
	int update_capacity = 0;
	struct cpufreq_adj_cell *cluster;
	int clust, cpu, pos, percent = MAX_CPUCAP_PERCENT;
	unsigned int last_max_freq = UINT_MAX, last_max_mitifreq = UINT_MAX;
	int ret = 0;
	struct cpufreq_adj_cell *pd;
	struct cpucap_adj_info_st *rsc_cap = &rsc_maxcap;
#ifndef RSC_CPUCAP_NOLOCK
	unsigned long flags;
#endif

	if (unlikely(!rsc_cpucap_initialized))
			return 0;

#ifdef RSC_CPUCAP_NOLOCK
	/*do nothing.*/
#else
	/*we could not use mutex, because it maybe in atomic env.*/
	/*mutex_lock(&rsc_cap->lock);*/
	spin_lock_irqsave(&rsc_cap->spinlock, flags);
	pos = rsc_cap->write_pos;
	pd = &rsc_cap->data[pos];
#endif

	cpu = cpumask_first(&policy->cpumask);
	if (cpu >= nr_cpumask_bits)	{
		rsc_err("%s: cpu: %d(max: %d) policy->cpumask: %*pbl. error\n",
			__func__, cpu, nr_cpumask_bits,  cpumask_pr_args(&policy->cpumask));
			dump_stack();
			cpu = 0;
	}
	clust = per_cpu(cpu_to_cluster_map, cpu);

	if (clust >= CL_END) {
		rsc_err("%s: clust %d bigger than %d\n", __func__, clust, CL_END);
		ret = -1;
		goto out;
	}

	cluster = &rsc_cpu_cluster[clust];

	if (POLICY_ADJUST_FREQ == mode) {
		last_max_freq = cluster->last_max_freq;
		orig_max_freq = cluster->last_max_freq = cluster->max_freq;
		cluster->min_freq = policy->min_freq;
		cluster->max_freq = policy->max_freq;
		rsc_max_freq[clust] = policy->max_freq;
		cluster->cur_freq = policy->cur_freq;
		update_capacity +=  (orig_max_freq != cluster->max_freq);
		if (update_capacity) {
#ifdef RSC_CPUCAP_NOLOCK
			pos = rsc_cpucap_enqueue(rsc_cap);
			pd = &rsc_cap->data[pos];
#endif
			/*donot clear pd buffer*/
			/*memset(pd, 0 ,sizeof(struct cpufreq_adj_cell));*/
			pd->mode = POLICY_ADJUST_FREQ;
			pd->last_max_freq = orig_max_freq;
			pd->min_freq = cluster->min_freq;
			pd->max_freq = cluster->max_freq ;
			pd->cur_freq = cluster->cur_freq ;
			cpumask_andnot(&pd->offline, cpu_possible_mask, cpu_online_mask);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
			cpumask_or(&pd->offline, &pd->offline, &cpus_isolated_by_thermal);
#endif
		}
	} else if (LMH_DCVS_ADJUST_FREQ == mode || LMH_SETLIM_ADJUST_FREQ == mode) {
		last_max_mitifreq = cluster->last_max_mitifreq;
		orig_max_freq = cluster->last_max_mitifreq =  cluster->max_mitigated_freq;
		cluster->max_mitigated_freq = policy->max_mitigated_freq;
		rsc_max_mitigated_freq[clust] = policy->max_mitigated_freq;
		update_capacity +=	(orig_max_freq != cluster->max_mitigated_freq);
		if (update_capacity) {
#ifdef RSC_CPUCAP_NOLOCK
			pos = rsc_cpucap_enqueue(rsc_cap);
			pd = &rsc_cap->data[pos];
#endif
			/*memset(pd, 0 ,sizeof(struct cpufreq_adj_cell));*/
			pd->mode = mode;
			pd->last_max_mitifreq = cluster->last_max_mitifreq ;
			pd->max_mitigated_freq = cluster->max_mitigated_freq;
			cpumask_andnot(&pd->offline, cpu_possible_mask, cpu_online_mask);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
			cpumask_or(&pd->offline, &pd->offline, &cpus_isolated_by_thermal);
#endif
		}
	} else if (mode >= RSC_CPU_DOWN && mode <= RSC_CPU_UNISO)/*(RSC_CPU_DOWN == mode || RSC_CPU_UP == mode)*/ {
#ifdef RSC_CPUCAP_NOLOCK
		pos = rsc_cpucap_enqueue(rsc_cap);
		pd = &rsc_cap->data[pos];
#endif
		/*memset(pd, 0 ,sizeof(struct cpufreq_adj_cell));*/
		pd->mode = mode;
		last_max_freq = 0;
		update_capacity  = 1;
		cpumask_copy(&pd->offline, &policy->offline);
	} else {
		rsc_err("%s: mod: %d not support now!\n", __func__, mode);
		ret = -2;
		goto out;
	}

	if (update_capacity) {
		int diff;
		int last_percent;
#ifdef RSC_CPUCAP_NOLOCK
		unsigned long s_flags;
#endif

		pd->is_valid = 0;
		cpumask_copy(&pd->cpumask, &policy->cpumask);
		pd->cpu = policy->cpu;

		/*save backtrace*/
		pd->callstack.entries = pd->stack_entries;
		pd->callstack.nr_entries = 0;
		pd->callstack.max_entries = ARRAY_SIZE(pd->stack_entries);
		pd->callstack.skip = 2;
		save_stack_trace(&pd->callstack);
		pd->pid = current->pid;
		strlcpy(pd->comm, current->comm, TASK_COMM_LEN);
		do_gettimeofday(&pd->tv);

		percent =  rsc_cal_cpucap_percent(pd);
		pd->percent = percent;
		last_percent = last_update_percent;
		pd->last_percent = last_percent;
		pd->seq = (unsigned int)atomic_inc_return(&rsc_cap->totalcnt);
		pd->is_valid = 1;
		atomic_set(&rsc_cap->last_pos, pos);
		rsc_cap->ever_adj = 1;
#ifdef RSC_CPUCAP_NOLOCK
		if (pos >= (rsc_cap->que_size - 1))
			rsc_cap->is_full = 1;
		atomic_dec(&rsc_cap->write_read_bit[pos]);
#else
		rsc_cap->write_pos++;
		if (rsc_cap->write_pos >= rsc_cap->que_size) {
			rsc_cap->is_full = 1;
			rsc_cap->write_pos = 0;
		}
#endif
		/*notify only when capability  change equal and more than notify_change_percent.
		    At the same time, ignore the first time call,such as select governor in /system/etc/init.qcom.post_boot.sh :
		    echo "interactive" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
		    call backtrace: store_scaling_governor -> cpufreq_set_policy */
		atomic_set(&lastone_notify, 1);
		atomic_inc(&cpucap_debug_cnt);

		#define RSC_NOTIFY_INFO "%s: percent: %4d(%4d) mode: %d clust%d  tl:%4u (%7d -> %7d) %7d %7d - (%7d -> %7d) "\
				"cpu: %d t: %12llu comm: %16s pid: %5d offline: %*pbl\n", __func__, percent, last_percent, mode, clust, pd->seq,\
				pd->last_max_freq, pd->max_freq, pd->min_freq, pd->cur_freq, pd->last_max_mitifreq,\
				pd->max_mitigated_freq, pd->cpu, RSC_NS_TO_US(local_clock()), pd->comm, pd->pid, cpumask_pr_args(&pd->offline)
#ifdef RSC_CPUCAP_NOLOCK
		spin_lock_irqsave(&last_percent_spinlock, s_flags);
#endif
		diff = rsc_abs_diff(last_update_percent, percent);
		rsc_cpucap_percent = percent;
		if (diff >= notify_change_percent) {
			last_update_percent = percent;
#ifdef RSC_CPUCAP_NOLOCK
			spin_unlock_irqrestore(&last_percent_spinlock, s_flags);
#endif
			atomic_set(&lastone_notify, 0);
			atomic_inc(&cpucap_cnt);
			atomic_inc(&rsc_cap->capcnt);
			/*could not call cancel_delayed_work_sync here, cancel_delayed_work_sync can block!*/
			/*cancel_delayed_work_sync(&rsc_notify_delayed_work);*/
			rsc_cpucap(RSC_NOTIFY_INFO);
		} else {
#ifdef RSC_CPUCAP_NOLOCK
			spin_unlock_irqrestore(&last_percent_spinlock, s_flags);
#endif
			/*cancel_delayed_work_sync(&rsc_notify_delayed_work);*/
			/*schedule_delayed_work(&rsc_notify_delayed_work,
				usecs_to_jiffies(rsc_notify_delayed_work_timeout));*/
			rsc_dbg(CPU_CAP, RSC_NOTIFY_INFO);
		}
	}

out:

#ifndef RSC_CPUCAP_NOLOCK
	spin_unlock_irqrestore(&rsc_cap->spinlock, flags);
	/*mutex_unlock(&rsc_cap->lock);*/
#endif

	if (update_capacity) {
		/*cound not call sysfs_notify directly. Because we will  in atomic context*/
		/*sysfs_notify(rsc_root_dir, NULL, __stringify(CPUCAP_NOTIFY_NAME));*/
		/*sysfs_notify(rsc_root_dir, NULL, __stringify(CPUCAP_NOTIFY_DETAIL_NAME));*/
		complete(&cpucap_notify_complete);
	}

	return ret;
}

static BLOCKING_NOTIFIER_HEAD(rsc_cpucap_notifier_list);

static int rsc_cpucap_notifier_call(rsc_cpucap_adj_type mode, struct rsc_cpucap_adj_t *data)
{
    return blocking_notifier_call_chain(&rsc_cpucap_notifier_list, (unsigned long)mode, (void *)data);
}
EXPORT_SYMBOL(rsc_cpucap_notifier_call);

static int rsc_cpucap_adjust_internal(struct rsc_cpucap_adj_t *adj)
{
	return rsc_cpucap_adjust_handler(NULL,	(unsigned long)adj->mode, (void *)adj);
}

static struct notifier_block rsc_cpucap_adjust_notifier = {
	.notifier_call = rsc_cpucap_adjust_handler,
	.priority = INT_MAX,
};

int rsc_cpucap_adjust_register_notifier(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&rsc_cpucap_notifier_list, nb);
}
EXPORT_SYMBOL(rsc_cpucap_adjust_register_notifier);

int rsc_cpucap_adjust_unregister_notifier(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&rsc_cpucap_notifier_list, nb);
}
EXPORT_SYMBOL(rsc_cpucap_adjust_unregister_notifier);

static int policy_freq_adjust_notifier(struct notifier_block *nb,
		unsigned long val, void *data)
{
		struct cpufreq_policy *policy = (struct cpufreq_policy *)data;
		struct rsc_cpucap_adj_t adj;
		int ret = 0;
		int cpu, clust;
		struct cpufreq_adj_cell *cluster;

		if (val != CPUFREQ_NOTIFY || !policy)
			return 0;

		cpu = cpumask_first(policy->related_cpus);
		if (cpu >= nr_cpumask_bits) {
			rsc_err("%s: cpu: %d(max: %d) policy->related_cpus: %*pbl. error\n",
				__func__, cpu, nr_cpumask_bits,  cpumask_pr_args(policy->related_cpus));
				dump_stack();
				cpu = 0;
		}

		clust = per_cpu(cpu_to_cluster_map, cpu);
		if (clust >= CL_END) {
			rsc_err("%s: clust %d bigger than %d\n", __func__, clust, CL_END);
			return -EINVAL;
		}
		rsc_dbg(CPU_CAP, "%s: clust: %8d min: %8d max: %8d cur: %8d cpu: %d\n", __func__, clust,
			policy->min, policy->max, policy->cur, policy->cpu);

		cluster = &rsc_cpu_cluster[clust];

		/*
		perfd name:
		845 platform call:
			cat /proc/693/comm
				perf@1.0-servic
			ps -A name:
				vendor.qti.hardware.perf@1.0-service
		660 platform name: perfd
		*/
		if (!strcmp(current->comm, "perfd") || strnstr(current->comm,
			"vendor.qti.hard", TASK_COMM_LEN)) {
				rsc_dbg(CPU_CAP, "%s: mod: %d perfd adjust freq"
					" minfreq(%d)	maxfreq(%d) last_max_freq(%d)!\n",
						__func__, POLICY_ADJUST_FREQ, policy->min,
						policy->max, cluster->last_max_freq);

			/*when touch the display panel, app will set the minfreq value to
			   upper freq(maybe maxfreq), at the same time, it will let maxfreq == minfreq.
			   see cpufreq_verify_within_limits and  set_cpu_min_freq.
			   we should ingnore these notify!*/
			if ((rsc_notify_mask & INGNORE_PERFD_SET_MINFREQ) && (policy->min == policy->max)) {
				rsc_dbg(CPU_CAP, "%s: mode: %d perfd adjust minfreq(%d) > last_max_freq(%d) !\n",
					__func__, POLICY_ADJUST_FREQ, policy->min, cluster->last_max_freq);
				goto out;
			}
		}
		memset(&adj, 0, sizeof(adj));
		adj.mode = POLICY_ADJUST_FREQ;
		adj.min_freq = policy->min;
		adj.max_freq = policy->max;
		adj.cur_freq = policy->cur;
		adj.cpu = policy->cpu;
		/*adj.offline = 0;*/
		/*
		if (!rsc_cpu_cluster_init[clust]) {
			rsc_cpu_cluster[clust].min_freq = policy->cpuinfo.min_freq;
			rsc_cpu_cluster[clust].max_freq = policy->cpuinfo.max_freq;
			rsc_cpu_cluster[clust].max_mitigated_freq = policy->cpuinfo.max_freq;
			rsc_cpu_cluster_init[clust] = 1;
		}
		*/
		cpumask_copy(&adj.cpumask, policy->related_cpus);

		ret = rsc_cpucap_adjust_internal(&adj);

out:
		return ret;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
int rsc_cpu_up_ext(unsigned int cpu, int mode)
{
	struct rsc_cpucap_adj_t *adj;
	struct rsc_cpucap_adj_t realadj;

	adj = &realadj;
	memset(adj, 0, sizeof(*adj));
	adj->min_freq = 0;
	adj->max_freq = 0;
	adj->cur_freq = 0;
	adj->cpu = cpu;
	cpumask_copy(&adj->cpumask, cpu_online_mask);
	cpumask_andnot(&adj->offline, cpu_possible_mask, cpu_online_mask);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
	cpumask_or(&adj->offline, &adj->offline, &cpus_isolated_by_thermal);
#endif
	adj->mode =	mode;
	rsc_dbg(CPU_CAP,
		"%s cpu%d online! offline: %*pbl poss: %*pbl online: %*pbl\n",
		__func__, cpu, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask), cpumask_pr_args(cpu_online_mask));
	if (cpumask_test_cpu(cpu, &adj->offline))
		rsc_err("%s cpu%d online error? offline: %*pbl poss: %*pbl online: %*pbl\n",
		__func__, cpu, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask), cpumask_pr_args(cpu_online_mask));

	if (!((rsc_notify_mask & INGNORE_SYSTEM_SERVER_OFFLINE_CPU) &&
		!strcmp(current->comm, "system_server")))
			rsc_cpucap_adjust_internal(adj);
	else
		rsc_dbg(CPU_CAP, "%s: mode: %d ignore system_server offline and online cpu%d!\n",
			__func__, adj->mode, cpu);

	return 0;
}

int rsc_cpu_down_ext(unsigned int cpu, int mode)
{
	struct rsc_cpucap_adj_t *adj;
	struct rsc_cpucap_adj_t realadj;

	adj = &realadj;
	memset(adj, 0, sizeof(*adj));
	adj->min_freq = 0;
	adj->max_freq = 0;
	adj->cur_freq = 0;
	adj->cpu = cpu;
	cpumask_copy(&adj->cpumask, cpu_online_mask);
	cpumask_andnot(&adj->offline, cpu_possible_mask, cpu_online_mask);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
	cpumask_or(&adj->offline, &adj->offline, &cpus_isolated_by_thermal);
#endif
	adj->mode = mode;
	rsc_dbg(CPU_CAP,
		"%s cpu%d offline! offline: %*pbl poss: %*pbl online: %*pbl\n",
		__func__, cpu, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask),
		cpumask_pr_args(cpu_online_mask));

	if (!((rsc_notify_mask & INGNORE_SYSTEM_SERVER_OFFLINE_CPU) &&
		!strcmp(current->comm, "system_server")))
			rsc_cpucap_adjust_internal(adj);
	else
		rsc_dbg(CPU_CAP, "%s: mode: %d ignore system_server offline and online cpu%d!\n",
			__func__, adj->mode, cpu);

		return 0;
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
static int rsc_cpu_up(unsigned int cpu)
{
	return rsc_cpu_up_ext(cpu, RSC_CPU_UP);
}

static int rsc_cpu_down(unsigned int cpu)
{
	return rsc_cpu_down_ext(cpu, RSC_CPU_DOWN);
}
#else
static int rsc_cpu_updown(struct notifier_block *nfb,
				      unsigned long action, void *hcpu)
{
	int cpu = (long)hcpu;
	struct rsc_cpucap_adj_t *adj;
	struct rsc_cpucap_adj_t realadj;
	unsigned long mode = (action & ~CPU_TASKS_FROZEN);

	switch (mode) {
	case CPU_ONLINE:
	case CPU_DEAD:
		adj = &realadj;
		memset(adj, 0, sizeof(*adj));
		adj->min_freq = 0;
		adj->max_freq = 0;
		adj->cur_freq = 0;
		adj->cpu = cpu;
		cpumask_copy(&adj->cpumask, cpu_online_mask);
		cpumask_andnot(&adj->offline, cpu_possible_mask, cpu_online_mask);

		if (mode == CPU_ONLINE) {
			adj->mode =	RSC_CPU_UP;
			rsc_dbg(CPU_CAP, "%s cpu%d online! action: %lx offline: %*pbl poss: %*pbl online: %*pbl\n",
				__func__, cpu, action, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask), cpumask_pr_args(cpu_online_mask));
			if (cpumask_test_cpu(cpu, &adj->offline))
				rsc_err("%s cpu%d online error? action: %lx offline: %*pbl poss: %*pbl online: %*pbl\n",
				__func__, cpu, action, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask), cpumask_pr_args(cpu_online_mask));
		} else if (mode == CPU_DEAD) {
			adj->mode = RSC_CPU_DOWN;
			rsc_dbg(CPU_CAP, "%s cpu%d offline! action: %lx offline: %*pbl poss: %*pbl online: %*pbl\n",
				__func__, cpu, action, cpumask_pr_args(&adj->offline), cpumask_pr_args(cpu_possible_mask),
				cpumask_pr_args(cpu_online_mask));
		}

		if (!((rsc_notify_mask & INGNORE_SYSTEM_SERVER_OFFLINE_CPU) &&
			!strcmp(current->comm, "system_server")))
				rsc_cpucap_adjust_internal(adj);
		else
			rsc_dbg(CPU_CAP, "%s: mode: %d ignore system_server offline and online cpu%d!\n",
				__func__, adj->mode, cpu);

		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block rsc_cpu_updown_notifier = {
	.notifier_call = rsc_cpu_updown,
};
#endif

static struct notifier_block policy_freq_adjust = {
	.notifier_call = policy_freq_adjust_notifier
};

static struct kobj_attribute rsc_cpucap_notif_attr =
__ATTR(CPUCAP_NOTIFY_NAME, 0660, show_rsc_cpucap_notif, store_rsc_cpucap_notif);

static struct kobj_attribute rsc_cpucap_notif_detail_attr =
__ATTR(CPUCAP_NOTIFY_DETAIL_NAME, 0660, show_rsc_cpucap_detail_notif, store_rsc_cpucap_detail_notif);

static struct kobj_attribute rsc_cpucap_notif_debug_attr =
__ATTR(CPUCAP_NOTIFY_DEBUG_NAME, 0660, show_rsc_cpucap_debug_notif, store_rsc_cpucap_debug_notif);

static int init_cpu_cluster(void)
{
	int i, ret = 0;

	unsigned int cpu, first_cpu;
	int cluster = 0;
	int actcpu, cpubak = -1;
#ifdef RSC_RECORD_BIGCORE_USAGE
	unsigned long cpueff;
	int reccnt = 0;
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
	int setlittle = 0;
#endif
#endif

	rsc_maxcpu = 0;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy;

		rsc_maxcpu++;
		if (cpu  > CONFIG_NR_CPUS) {
			rsc_err("%s cpu(%d) is bigger than %d\n", __func__, cpu, CONFIG_NR_CPUS);
			ret = -EINVAL;
			goto out;
		}
#ifdef RSC_RECORD_BIGCORE_USAGE
		cpueff = arch_get_cpu_efficiency(cpu);
		/*msm8150 is tripple clusters, we would find the secondary big core*/
		if ((reccnt < 2) && (cpueff > rsc_max_possible_efficiency)) {
			reccnt++;
			rsc_max_possible_efficiency = cpueff;
		}
		rsc_top_possible_efficiency = max(rsc_top_possible_efficiency, cpueff);
		rsc_min_possible_efficiency = min(rsc_min_possible_efficiency, cpueff);
		rsc_dbg(CPU_CAP,
			"cpu%d reccnt%d cpueff %lu rsc_max_possible_efficiency %lu "
			"rsc_min_possible_efficiency %lu\n",
			cpu, reccnt, cpueff, rsc_max_possible_efficiency,
			rsc_min_possible_efficiency);
#else
		rsc_min_possible_efficiency = min(rsc_min_possible_efficiency, arch_get_cpu_efficiency(cpu));
#endif
		actcpu = cpu;
retry:
		policy = cpufreq_cpu_get(actcpu);
		if (policy) {
			first_cpu = cpumask_first(policy->related_cpus);
			if (first_cpu >= nr_cpumask_bits) {
				rsc_err("%s: cpu: %d(max: %d) policy->related_cpus: %*pbl. error\n",
				__func__, cpu, nr_cpumask_bits, cpumask_pr_args(policy->related_cpus));
				dump_stack();
				ret = -EINVAL;
				goto out;
			}
			if (rsc_max_max_freq < (unsigned long)policy->cpuinfo.max_freq) {
				rsc_max_max_freq = (unsigned long)policy->cpuinfo.max_freq;
				rsc_max_freq_cpu = actcpu;
			}
			if (rsc_min_max_freq > (unsigned long)policy->cpuinfo.max_freq) {
				rsc_min_max_freq = (unsigned long)policy->cpuinfo.max_freq;
				rsc_min_freq_cpu = actcpu;
			}
			/*
			rsc_max_max_freq = max(rsc_max_max_freq, (unsigned long)policy->cpuinfo.max_freq);
			rsc_min_max_freq = min(rsc_min_max_freq, (unsigned long)policy->cpuinfo.max_freq);
			*/
			per_cpu(rsc_cpu_maxfreq, cpu) = policy->cpuinfo.max_freq;
			for (i = 0; i < cluster; i++) {
				if (cluster_pol_cpu[i] == first_cpu)
				break;
			}
			per_cpu(cpu_to_cluster_map, cpu) = i;

			if (i >= cluster) {
				cluster_pol_cpu[cluster] = first_cpu;
				cluster++;
				if (cluster > CL_END) {
					rsc_err("%s cluster_num(%d) is bigger than %d\n", __func__, cluster, CL_END);
					ret = -EINVAL;
					goto out;
				}
			}
			cpufreq_cpu_put(policy);
		} else {
			/*
				 workround for msm8976, cpufreq_cpu_get(5) (cpu5-7) return NULL.
			*/
			if (cpubak != cpu) {
				actcpu = (cpu / 4) * 4;
				cpubak = cpu;
				rsc_err("%s cpu: %d policy is NULL!! use cpu: %d to try!\n", __func__, cpu, actcpu);
				goto retry;
			}
			rsc_err("%s cpu: %d %d %d policy is NULL!!\n", __func__, actcpu, cpu, cpubak);
			ret = -EINVAL;
			goto out;
		}
	}
	rsc_cluster_num = cluster;

	rsc_total_cpu_efficiency = 0;
	for_each_possible_cpu(cpu) {
		unsigned long freq;
		unsigned long eff;

#ifdef RSC_RECORD_BIGCORE_USAGE
		if (arch_get_cpu_efficiency(cpu) >= rsc_max_possible_efficiency) {
			cpumask_set_cpu(cpu, &rsc_cpu_bigcore);
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
			rsc_bigcore_num++;
#endif
		}
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
		else {
			setlittle = 1;
			cpumask_set_cpu(cpu, &rsc_cpu_littlecore);
		}
#endif
#endif

		/*see static int compute_capacity(struct sched_cluster *cluster) ,
		   capacity_scale_cpu_efficiency , capacity_scale_cpu_freq in hmp.c*/
		eff = get_scale_cpu_efficiency(cpu);
		per_cpu(rsc_scale_cpu_efficiency, cpu) = eff;

		freq = get_scale_cpu_freq(per_cpu(rsc_cpu_maxfreq, cpu));
		per_cpu(rsc_scale_cpu_freq, cpu) = freq;

		rsc_total_cpu_efficiency += (((eff /* * 1024/1024 */) * freq) >> 10);
	}

	if (rsc_min_possible_efficiency == rsc_max_possible_efficiency) {
		if (rsc_max_freq_cpu != rsc_min_freq_cpu) {
			if ((rsc_max_freq_cpu < NR_CPUS) && (rsc_min_freq_cpu < NR_CPUS)) {
				if (rsc_max_freq_cpu > rsc_min_freq_cpu) {
					setlittle = 1;
					cpumask_clear(&rsc_cpu_littlecore);
					cpumask_clear(&rsc_cpu_bigcore);
					for (i = 0; i < rsc_max_freq_cpu; i++)
						cpumask_set_cpu(i, &rsc_cpu_littlecore);
					for (i = rsc_max_freq_cpu; i < NR_CPUS; i++)
						cpumask_set_cpu(i, &rsc_cpu_bigcore);
				} else {
					setlittle = 1;
					cpumask_clear(&rsc_cpu_littlecore);
					cpumask_clear(&rsc_cpu_bigcore);
					for (i = 0; i < rsc_min_freq_cpu; i++)
						cpumask_set_cpu(i, &rsc_cpu_bigcore);
					for (i = rsc_min_freq_cpu; i < NR_CPUS; i++)
						cpumask_set_cpu(i, &rsc_cpu_littlecore);
				}
			}
		}

	}

out:
#ifdef RSC_RECORD_BIGCORE_USAGE
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
	if (!setlittle)
		cpumask_copy(&rsc_cpu_littlecore, &rsc_cpu_bigcore);
#endif

	rsc_info("%s cluster num: %d topeff: %lu maxeff: %lu mineff: %lu rsc_cpu_bigcore: %*pbl"
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
			" littlecore: %*pbl"
#endif
			" mincpu%ld %ld maxcpu%ld %ld"
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
			" bigcorenum: %d"
#endif
			"\n",
		__func__, rsc_cluster_num, rsc_top_possible_efficiency,
		rsc_max_possible_efficiency, rsc_min_possible_efficiency,
		cpumask_pr_args(&rsc_cpu_bigcore)
#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
		, cpumask_pr_args(&rsc_cpu_littlecore)
#endif
		, rsc_min_freq_cpu, rsc_min_max_freq
		, rsc_max_freq_cpu, rsc_max_max_freq
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
		, rsc_bigcore_num
#endif
		);
#else
	rsc_info("%s cluster num: %d\n", __func__, rsc_cluster_num);
#endif

	for (i = 0; i < rsc_cluster_num; i++) {
		rsc_info("%s cluster%d firstcpu: %d \n", __func__, i, cluster_pol_cpu[i]);
	}

	return ret;
}

#ifdef CONFIG_MTK_PLATFORM
int rsc_cpucap_notif_init(void)
#else
static int __init rsc_cpucap_notif_init(void)
#endif
{
	long ret = 0;

	init_completion(&cpucap_notify_complete);
	cpucap_task = kthread_run(do_cpucap_notify, NULL, "rsc_noti");
	if (IS_ERR(cpucap_task)) {
		rsc_err("Failed to create do_hotplug thread. err:%ld\n",
				PTR_ERR(cpucap_task));
		ret = PTR_ERR(cpucap_task);
		return ret;
		/*goto fail_kthread_run;*/
	}

	ret = init_cpu_cluster();
	if (ret) {
		rsc_err("%s init_cpu_cluster error!\n", __func__);
		return ret;
	}

#ifdef CONFIG_RSC_KSWAPD_IMPROVE
	kthread_run(rsc_vswapd, NULL, "vswapd0");
#if 0 //disable fo MTK
	kthread_run(rsc_vswap_fn, NULL, "vswap_fn");
#endif
#endif

	ret = rsc_cpucap_adjust_register_notifier(&rsc_cpucap_adjust_notifier);
	if (ret)
		return ret;

	ret = cpufreq_register_notifier(&policy_freq_adjust,
						CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		goto fail_cpufreq_register;

	/* Register cpu up down notifiers */
	/*cpu_notifier(rsc_cpu_updown, INT_MIN);*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN, "rsc_cpu:offline",
				NULL, rsc_cpu_down);
	/*rsc_info("%s cpuhp_setup_state CPUHP_BP_PREPARE_DYN %d ret: %d\n",
		__func__, CPUHP_BP_PREPARE_DYN, ret);*/
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "rsc_cpu:online",
				rsc_cpu_up, NULL);
	/*rsc_info("%s cpuhp_setup_state CPUHP_AP_ONLINE_DYN %d ret: %d\n",
		__func__, CPUHP_AP_ONLINE_DYN, ret);*/
	/*
	* Returns:
	*	On success:
	*	   Positive state number if @state is CPUHP_AP_ONLINE_DYN
	*	   0 for all other states
	*	On failure: proper (negative) error code
	*/
	if (ret < 0) {
		rsc_err("%s cpuhp_setup_state error! ret: %ld\n", __func__, ret);
		goto fail_register_cpu_noti;
	}
#else
	ret = register_cpu_notifier(&rsc_cpu_updown_notifier);
	if (ret) {
		rsc_err("%s cpuhp_setup_state error! ret: %ld\n", __func__, ret);
		goto fail_register_cpu_noti;
	}
#endif

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpucap_notif_attr.attr);
	if (ret)
		goto fail_sysfs_create_file1;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpucap_notif_attr.attr);

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpucap_notif_detail_attr.attr);
	if (ret)
		goto fail_sysfs_create_file2;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpucap_notif_detail_attr.attr);

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpucap_notif_debug_attr.attr);
	if (ret)
		goto fail_sysfs_create_file3;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpucap_notif_debug_attr.attr);

	INIT_DELAYED_WORK(&rsc_notify_delayed_work, rsc_notify_delayed_work_fn);

	rsc_register_cpucap_adjust(rsc_cpucap_adjust_internal);

	rsc_info("%s finish!\n", __func__);
	rsc_cpucap_initialized = 1;
	return (int)ret;

/*
fail_kthread_run:
	sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_debug_attr.attr);
*/
fail_sysfs_create_file3:
	sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_detail_attr.attr);
fail_sysfs_create_file2:
	sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_attr.attr);
fail_sysfs_create_file1:
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	unregister_cpu_notifier(&rsc_cpu_updown_notifier);
#endif
fail_register_cpu_noti:
	cpufreq_unregister_notifier(&policy_freq_adjust, CPUFREQ_POLICY_NOTIFIER);
fail_cpufreq_register:
	rsc_cpucap_adjust_unregister_notifier(&rsc_cpucap_adjust_notifier);

	return (int)ret;
}

static void __exit rsc_cpufreq_adjust_exit(void)
{
	if (rsc_cpucap_initialized) {
		rsc_cpucap_initialized = 0;
		cancel_delayed_work_sync(&rsc_notify_delayed_work);
		rsc_register_cpucap_adjust(NULL);
		kthread_stop(cpucap_task);
		cpucap_task = NULL;
		sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_attr.attr);
		sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_detail_attr.attr);
		sysfs_remove_file(rsc_root_dir, &rsc_cpucap_notif_debug_attr.attr);
		rsc_cpucap_adjust_unregister_notifier(&rsc_cpucap_adjust_notifier);
		cpufreq_unregister_notifier(&policy_freq_adjust, CPUFREQ_POLICY_NOTIFIER);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
		unregister_cpu_notifier(&rsc_cpu_updown_notifier);
#endif
	}
}

/*for MTK platform 6763, rsc_cpucap_notif_init called in cpufreq_register_driver*/
#ifndef CONFIG_MTK_PLATFORM
module_init(rsc_cpucap_notif_init);
module_exit(rsc_cpufreq_adjust_exit);
#endif
