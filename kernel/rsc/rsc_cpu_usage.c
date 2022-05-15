/*
 * kernel/rsc/rsc_cpu_usage.c
 *
 * VIVO Resource Control.
 *
 * stat task cpu usage.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/sort.h>
#include <linux/vivo_rsc_ioctl.h>
#include <linux/vivo_rsc/rsc_cpu_internal.h>
#include <linux/vivo_rsc/rsc_cpu_usage.h>
#include <linux/rtc.h>
#define RSC_I_AM_IN_CPU_USAGE
#define CREATE_TRACE_POINTS
#include <trace/events/vivo_rsc.h>

DEFINE_PER_CPU(struct rsc_task_cpu_usage_t,
	task_cpu_usage[RSC_STAT_MAX_TASK]);
DEFINE_PER_CPU(u8, task_cpu_usage_tbl[RSC_STAT_MAX_TASK]);
DEFINE_PER_CPU(u32, task_cpu_usage_cnt);
DEFINE_PER_CPU(u32, task_cpu_usage_idx);
#ifdef RECORD_DEBUG_LOG
DEFINE_PER_CPU(u32, task_call_cnt);
#endif
#ifdef RECORD_IDLE_TICK_CNT
DEFINE_PER_CPU(u32, task_cpu_usage_idle);
#endif
DEFINE_PER_CPU(u32, task_cpu_usage_notidle);
DEFINE_PER_CPU(u32, task_cpu_usage_total);
DEFINE_PER_CPU(u32, task_cpu_usage_user);
/*hardware irq*/
DEFINE_PER_CPU(u32, task_cpu_usage_hirq);
/*soft irq*/
DEFINE_PER_CPU(u32, task_cpu_usage_sirq);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) || defined(RSC_USE_SYS_IRQ_TIME)
/*hardware irq*/
DEFINE_PER_CPU(u64, task_cpu_usage_hirq_time);
/*soft irq*/
DEFINE_PER_CPU(u64, task_cpu_usage_sirq_time);
#endif
DEFINE_PER_CPU(u8,
	task_cpu_usage_map_cnt[RSC_MAX_HASH_TASK]);

DEFINE_PER_CPU(u8,
	task_cpu_usage_map_idx[RSC_MAX_HASH_TASK]);

#ifdef RECORD_GET_LOCK_FAIL
atomic_t task_get_lock_fail_cnt = ATOMIC_INIT(-1);
struct rsc_task_cpu_usage_t task_cpu_usage_fail[RSC_GET_LOCK_FAIL_MAX_TASK];
#endif

#ifndef PM_QOS_CPU_USAGE_DEFAULT_VALUE
#define PM_QOS_CPU_USAGE_DEFAULT_VALUE 10
#endif

#define CPU_USAGE_MIN_PERIOD 100 /*mini value is 100ms*/
atomic_t __read_mostly task_cpu_usage_enable = ATOMIC_INIT(CPU_USAGE_DEFAULT_ENABLE);
/*static atomic64_t rsc_task_cpu_usage_period = ATOMIC_INIT(CPU_USAGE_DEF_PERIOD);*/ /*mini second*/
static atomic64_t rsc_task_cpu_usage_period = ATOMIC_INIT(CPU_USAGE_INIT_PERIOD); /*mini second*/
u16 __read_mostly task_cpu_usage_seq = 1;
static atomic_t task_cpu_usage_notify_seq = ATOMIC_INIT(0);
/*max_stat_task could not bigger than RSC_STAT_MAX_TASK*/
const u32 max_stat_task = RSC_STAT_MAX_TASK;

#ifdef SHOW_TASK_INFO_IN_PRINTK
DEFINE_PER_CPU(u32, task_cpu_usage_max_tick_const);
DEFINE_PER_CPU(u32, task_cpu_usage_min_tick_const);
#endif

/*size: 72 x 8 x 16 = 9KB*/
static struct rsc_task_cpu_usage_t glb_task_cpu_usage[NR_CPUS * RSC_STAT_MAX_TASK];
/*size:   1 x 8 x 16 = 128bytes*/
#if (NR_CPUS * RSC_STAT_MAX_TASK <= 256)
static u8	glb_task_cpu_usage_tbl[NR_CPUS * RSC_STAT_MAX_TASK];
#else
static u16	glb_task_cpu_usage_tbl[NR_CPUS * RSC_STAT_MAX_TASK];
#endif

#ifdef SORT_IN_USER_PROCESS
static DEFINE_PER_CPU(u32, task_cpu_usage_last_cnt);
static  int data_is_sorted;
#endif

#ifdef RSC_HASH_FASTHIT_DEBUG
DEFINE_PER_CPU(u32, task_fasthit_cnt);
DEFINE_PER_CPU(u32, task_fasthit_new_cnt);
DEFINE_PER_CPU(u32, task_notfasthit_cnt);
#endif

static u32 glb_task_cpu_usage_cnt;
static long rsc_task_cpu_usage_last_period;
static u32 glb_task_cpu_usage_total;
static u32 glb_task_cpu_usage_user;
static u32 glb_task_cpu_hirq_total;
static u32 glb_task_cpu_sirq_total;

static u8 glb_task_cpu_num;
static u64 rsc_consume_jiffes;
static struct timeval cpu_usage_stime, cpu_usage_last_stime;
static struct timeval cpu_usage_etime, cpu_usage_last_etime;
static int notify_percent = PM_QOS_CPU_USAGE_DEFAULT_VALUE;
#define RSC_CPU_USAGE_MAX_FPID	16
static int cpu_usage_fpid[RSC_CPU_USAGE_MAX_FPID] = {
	[0 ... RSC_CPU_USAGE_MAX_FPID - 1] = -1,
};
static int cpu_usage_fpid_cnt;

/*
could not user percpu var,
percpu var is not array,
address such as:
percpu 0: ffffffc0fa154e8c
percpu 1: ffffffc0fa16ae8c
percpu 2: ffffffc0fa180e8c
percpu 3: ffffffc0fa196e8c
percpu 4: ffffffc0fa1ace8c
percpu 5: ffffffc0fa1c2e8c
percpu 6: ffffffc0fa1d8e8c
percpu 7: ffffffc0fa1eee8c
*/
/*
static DEFINE_PER_CPU(u32, task_cpu_usage_last_notidle);
static DEFINE_PER_CPU(u32, task_cpu_usage_last_hirq);
static DEFINE_PER_CPU(u32, task_cpu_usage_last_sirq);
*/
static u32 task_cpu_usage_last_notidle[NR_CPUS];
static u32 task_cpu_usage_last_user[NR_CPUS];
static u32 task_cpu_usage_last_hirq[NR_CPUS];
static u32 task_cpu_usage_last_sirq[NR_CPUS];

static u64 rsc_jiffes_start, rsc_jiffes_end;

static  int data_is_new;

static DEFINE_MUTEX(cpu_usage_lock);

#define KTOP_PERIOD_SET_STR "period:"
#define KTOP_FORCESTOP_STR "forcestop"
/*Note: donot disable it automatically*/
#define KTOP_FORCESTART_STR "forcestart"
static  int forcestart_work;

/*
struct rc_id_map_t {
	u32 id;
	const char *name;
};

static struct rc_id_map_t rsc_ids[] = {
	{RSC_ID_PERFD, RSC_ID(RSC_ID_PERFD)},
	{RSC_ID_BBKLOG, RSC_ID(RSC_ID_BBKLOG)},
	{RSC_ID_POWER, RSC_ID(RSC_ID_POWER)},
	{RSC_ID_VIVOD, RSC_ID(RSC_ID_VIVOD)},
	{RSC_ID_BIGDATA, RSC_ID(RSC_ID_BIGDATA)},
	{RSC_ID_SHELL, RSC_ID(RSC_ID_SHELL)},
	{RSC_ID_OTHER, RSC_ID(RSC_ID_OTHER)}
};
*/

static unsigned long rsc_ktop_map;
/*us*/
static u64 cpu_usage_disable_time;

#ifdef RSC_RECORD_BIGCORE_USAGE
static u32 glb_task_bigcore_tick[NR_CPUS * RSC_STAT_MAX_TASK];
#endif

#ifdef RSC_RECORD_LATENCY
bool rsc_ktop_latency_enable;
u64 rsc_ktop_start_time[NR_CPUS];
u32 rsc_ktop_cur_latency[NR_CPUS];
u32 rsc_ktop_max_latency[NR_CPUS];
u32 rsc_ktop_min_latency[NR_CPUS] = {
	[0 ... NR_CPUS - 1] = U32_MAX,
};
u32 rsc_ktop_avg_latency[NR_CPUS] = {
	[0 ... NR_CPUS - 1] = U32_MAX,
};
u64 rsc_ktop_avg_latency_total[NR_CPUS];
u64 rsc_ktop_avg_latency_cnt[NR_CPUS];
#endif

#if 0
static int disable_task_cpu_usage(void *arg)
{
	atomic_set(&task_cpu_usage_enable, 0);
}
#endif

#define USE_SCHEDTIMEOUT_FOR_GOOD_PERFORMANCE
#define SCHEDTIMEOUT_TICkS 1

#ifndef USE_SCHEDTIMEOUT_FOR_GOOD_PERFORMANCE
static  void wait_cpu_tick(void *data)
{
/*
	rsc_info("%s: cpu: %d wait here! disable cpu: %d\n",
		__func__, raw_smp_processor_id(), *(int*)(data));
*/
}
#endif

static int rsc_cpu_usage_disable(void)
{
	int cpu;
	u64 t0, t1, diff;
#ifdef USE_SCHEDTIMEOUT_FOR_GOOD_PERFORMANCE
	long ret;
#endif
#if 0
	stop_machine(disable_task_cpu_usage, NULL, NULL);
#else
	atomic_set(&task_cpu_usage_enable, 0);
	smp_wmb();
#endif
	rsc_jiffes_end = get_jiffies_64();
	rsc_dbg(CPU_TASK_USAGE, "period: %lld rsc_jiffes_end: %lld rsc_jiffes_start: %lld  diff: %lld\n",
		(u64)atomic64_read(&rsc_task_cpu_usage_period), rsc_jiffes_end, rsc_jiffes_start,
		rsc_jiffes_end - rsc_jiffes_start);

	do_gettimeofday(&cpu_usage_etime);
	cpu = raw_smp_processor_id();
	t0 = local_clock();

	/*wait all cpu finish cpu usage stat*/
#ifdef USE_SCHEDTIMEOUT_FOR_GOOD_PERFORMANCE
retry:
	set_current_state(TASK_UNINTERRUPTIBLE);
	ret = schedule_timeout(SCHEDTIMEOUT_TICkS);
	t1 = local_clock();
	diff = t1 - t0;
	/*maybe wake up by cpu capability change!*/
	if (ret && (diff < jiffies_to_nsecs(SCHEDTIMEOUT_TICkS)))
		goto retry;
	__set_current_state(TASK_RUNNING);
#else
	smp_call_function(wait_cpu_tick, (void *)&cpu, 1);
	t1 = local_clock();
	diff = t1 - t0;
#endif

	cpu_usage_disable_time = diff/1000;
#if 0
	/*wait all cpu finish cpu usage stat*/
	for_each_possible_cpu(cpu)
		while (atomic_read(&per_cpu(task_cpu_in_stat, cpu)))
			;
#endif
	return 0;
}

static int rsc_cpu_usage_enable(void)
{
	int cpu;
#if 0
	struct task_struct *g, *p;

	/*read_lock(&tasklist_lock);*/
	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (p) {
			memset(p->rsc_utime, 0, sizeof(p->rsc_utime));
			memset(p->rsc_stime, 0, sizeof(p->rsc_stime));
		}
	}

#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	/*read_unlock(&tasklist_lock);*/
#endif
/*
	if (atomic_read(&task_cpu_usage_enable)) {
		rsc_err("%s: task_cpu_usage_enable : %d should disable first!\n",
			__func__, atomic_read(&task_cpu_usage_enable));
		return -EINVAL;
	}
*/
	/*need to backup the data when do rsc_cpu_usage_disable!*/
	for_each_possible_cpu(cpu) {
		per_cpu(task_cpu_usage_cnt, cpu) = 0;
		per_cpu(task_cpu_usage_idx, cpu) = 0;
#ifdef RECORD_DEBUG_LOG
		per_cpu(task_call_cnt, cpu) = 0;
#endif
		/*atomic_set(&per_cpu(task_cpu_in_stat, cpu), 0);*/
		per_cpu(task_cpu_usage_total, cpu) = 0;
#ifdef RECORD_IDLE_TICK_CNT
		per_cpu(task_cpu_usage_idle, cpu) = 0;
#endif
		per_cpu(task_cpu_usage_notidle, cpu) = 0;
		per_cpu(task_cpu_usage_user, cpu) = 0;
		per_cpu(task_cpu_usage_hirq, cpu) = 0;
		per_cpu(task_cpu_usage_sirq, cpu) = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) || defined(RSC_USE_SYS_IRQ_TIME)
		per_cpu(task_cpu_usage_hirq_time, cpu) =
			kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		per_cpu(task_cpu_usage_sirq_time, cpu) =
			kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
#endif
		memset(&per_cpu(task_cpu_usage_map_cnt, cpu), 0,
			sizeof(u8)  * RSC_MAX_HASH_TASK);
		memset(&per_cpu(task_cpu_usage_map_idx, cpu), 0,
			sizeof(u8)  * RSC_MAX_HASH_TASK);
	}

#if defined(RSC_RECORD_LATENCY)
	memset(rsc_ktop_cur_latency, 0, sizeof(rsc_ktop_cur_latency));
#endif

	task_cpu_usage_seq++;

	if (!task_cpu_usage_seq)
		task_cpu_usage_seq = 1;

	/*test only*/
	/*
	if (task_cpu_usage_seq % 2)
		max_stat_task = 8;
	else
		max_stat_task = min(16, RSC_STAT_MAX_TASK);
	*/
	rsc_jiffes_start = get_jiffies_64();
	rsc_dbg(CPU_TASK_USAGE, "period: %lld rsc_jiffes_start: %lld\n",
		(u64)atomic64_read(&rsc_task_cpu_usage_period), rsc_jiffes_start);
	do_gettimeofday(&cpu_usage_stime);
	atomic_inc(&task_cpu_usage_notify_seq);
	smp_wmb();

	atomic_set(&task_cpu_usage_enable, 1);

	return 0;
}

#define NEG_VAL -1
#define POS_VAL  1
static int cmp_task(const void *a, const void *b)
{
	const struct rsc_task_cpu_usage_t *l = a, *r = b;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
	int lpos, rpos;
	u32 l_time, r_time;

	lpos = l - glb_task_cpu_usage;
	rpos = r - glb_task_cpu_usage;

	tbl = glb_task_cpu_usage_tbl;
	l = &glb_task_cpu_usage[tbl[lpos]];
	r = &glb_task_cpu_usage[tbl[rpos]];

	l_time = l->time;
	r_time = r->time;

	if (l_time > r_time)
		return NEG_VAL;
	if (l_time < r_time)
		return POS_VAL;
	return 0;
}

static void swap_task(void *a, void *b, int size)
{
	struct rsc_task_cpu_usage_t *l = a, *r = b;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
	int lpos, rpos, tmp;

	lpos = l - glb_task_cpu_usage;
	rpos = r - glb_task_cpu_usage;

	tbl = glb_task_cpu_usage_tbl;
	tmp = tbl[lpos];
	tbl[lpos] = tbl[rpos];
	tbl[rpos] = tmp;
}

#ifndef SORT_IN_USER_PROCESS
/*sort in kthread*/
static void rsc_cpu_usage_sort(void)
{
	int i, j, cpu, len;
	int len0;
	struct rsc_task_cpu_usage_t *data, *base;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
#ifdef SHOW_TASK_INFO_IN_PRINTK
	u64 t0, t1, t2;

	t0 = local_clock();
#endif
	len0 = per_cpu(task_cpu_usage_cnt, 0);
	base = glb_task_cpu_usage;
	memcpy(base, &per_cpu(task_cpu_usage, 0),
		sizeof(struct rsc_task_cpu_usage_t) * len0);

	glb_task_cpu_usage_total = 0;
	glb_task_cpu_usage_user = 0;
	glb_task_cpu_hirq_total = 0;
	glb_task_cpu_sirq_total = 0;
	glb_task_cpu_num = 0;
	for_each_possible_cpu(cpu) {
		glb_task_cpu_num++;
		task_cpu_usage_last_notidle[cpu] = per_cpu(task_cpu_usage_notidle, cpu);
		glb_task_cpu_usage_total +=  per_cpu(task_cpu_usage_notidle, cpu);
		task_cpu_usage_last_user[cpu] = per_cpu(task_cpu_usage_user, cpu);
		glb_task_cpu_usage_user +=  per_cpu(task_cpu_usage_user, cpu);
#if defined(RSC_USE_SYS_IRQ_TIME)
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
		task_cpu_usage_last_hirq[cpu] = (kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] -
			per_cpu(task_cpu_usage_hirq_time, cpu))/TICK_NSEC;
		task_cpu_usage_last_sirq[cpu] = (kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ] -
			per_cpu(task_cpu_usage_sirq_time, cpu))/TICK_NSEC;
#else
		task_cpu_usage_last_hirq[cpu] = cputime_to_jiffies(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] -
			per_cpu(task_cpu_usage_hirq_time, cpu));
		task_cpu_usage_last_sirq[cpu] = cputime_to_jiffies(kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ] -
			per_cpu(task_cpu_usage_sirq_time, cpu));
#endif
#else
		task_cpu_usage_last_hirq[cpu] = per_cpu(task_cpu_usage_hirq, cpu);
		task_cpu_usage_last_sirq[cpu] = per_cpu(task_cpu_usage_sirq, cpu);
#endif
		glb_task_cpu_hirq_total += task_cpu_usage_last_hirq[cpu];
		glb_task_cpu_sirq_total += task_cpu_usage_last_sirq[cpu];

		if (cpu) {
			data = per_cpu(task_cpu_usage, cpu);
			len = per_cpu(task_cpu_usage_cnt, cpu);
			for (i = 0; i < len; i++) {
				for (j = 0; j < len0; j++) {
					if (base[j].task == data[i].task) {
						base[j].time += data[i].time;
						base[j].stime += data[i].stime;
						base[j].utime += data[i].utime;
						break;
					}
				}
				if (j >= len0) {
					memcpy(&base[len0], &data[i],
						sizeof(struct rsc_task_cpu_usage_t));
					if (len0 < (NR_CPUS * RSC_STAT_MAX_TASK))
						len0++;
					else
						rsc_err("%s: task num %d too more!\n", __func__, len0);
				}
			}
		}
	}
#ifdef SHOW_TASK_INFO_IN_PRINTK
	t1 = local_clock();
#endif
	tbl = glb_task_cpu_usage_tbl;
	for (j = 0; j < len0; j++)
		tbl[j] = j;

	sort(base, len0, sizeof(struct rsc_task_cpu_usage_t),
	     cmp_task, swap_task);
#ifdef SHOW_TASK_INFO_IN_PRINTK
	t2 = local_clock();
	rsc_info("sort usage, size: %3d merge time: %4u sort time: %4u\n",
		len0, (u32)(t1 - t0), (u32)(t2 - t1));
	data = base;
	for (i = 0; i < len0; i++) {
		j = tbl[i];
		rsc_info("sort usage: %%%2ld i: %2d j: %2d comm: %16s "
			"time: %4u utime: %4u stime: %4u pid: %5d tgid: %5d  "
			"tgcomm: %16s ppid: %5d\n",
			data[j].time*100/RSC_MS_TO_JIFF(atomic64_read(&rsc_task_cpu_usage_period)),
			i, j, data[j].comm, data[j].time,
			data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
			data[j].tgcomm, data[j].ppid);
	}
#endif
	glb_task_cpu_usage_cnt = len0;
	rsc_task_cpu_usage_last_period = atomic64_read(&rsc_task_cpu_usage_period);
	if (rsc_jiffes_end >= rsc_jiffes_start)
		rsc_consume_jiffes = rsc_jiffes_end - rsc_jiffes_start;
	else
		rsc_consume_jiffes = (U64_MAX - rsc_jiffes_end) + rsc_jiffes_start;
	cpu_usage_last_stime = cpu_usage_stime;
	cpu_usage_last_etime = cpu_usage_etime;
	data_is_new = 1;
}
#endif

#ifdef SORT_IN_USER_PROCESS
/*backup data into the glb buffer*/
static void rsc_cpu_usage_backup(void)
{
	int cpu, len;
	struct rsc_task_cpu_usage_t *base;

	base = glb_task_cpu_usage;
	glb_task_cpu_usage_total = 0;
	glb_task_cpu_usage_user = 0;
	glb_task_cpu_hirq_total = 0;
	glb_task_cpu_sirq_total = 0;
	glb_task_cpu_num = 0;
	for_each_possible_cpu(cpu) {
		glb_task_cpu_num++;
		task_cpu_usage_last_notidle[cpu] = per_cpu(task_cpu_usage_notidle, cpu);
		glb_task_cpu_usage_total +=  per_cpu(task_cpu_usage_notidle, cpu);
		task_cpu_usage_last_user[cpu] = per_cpu(task_cpu_usage_user, cpu);
		glb_task_cpu_usage_user +=  per_cpu(task_cpu_usage_user, cpu);
#if defined(RSC_USE_SYS_IRQ_TIME)
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 14, 0)
		task_cpu_usage_last_hirq[cpu] = (kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] -
			per_cpu(task_cpu_usage_hirq_time, cpu))/TICK_NSEC;
		task_cpu_usage_last_sirq[cpu] = (kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ] -
			per_cpu(task_cpu_usage_sirq_time, cpu))/TICK_NSEC;
#else
		task_cpu_usage_last_hirq[cpu] = cputime_to_jiffies(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] -
			per_cpu(task_cpu_usage_hirq_time, cpu));
		task_cpu_usage_last_sirq[cpu] = cputime_to_jiffies(kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ] -
			per_cpu(task_cpu_usage_sirq_time, cpu));
#endif
#else
		task_cpu_usage_last_hirq[cpu] = per_cpu(task_cpu_usage_hirq, cpu);
		task_cpu_usage_last_sirq[cpu] = per_cpu(task_cpu_usage_sirq, cpu);
#endif
		glb_task_cpu_hirq_total += task_cpu_usage_last_hirq[cpu];
		glb_task_cpu_sirq_total += task_cpu_usage_last_sirq[cpu];

		len = per_cpu(task_cpu_usage_cnt, cpu);
		per_cpu(task_cpu_usage_last_cnt, cpu) = len;
		memcpy(base, &per_cpu(task_cpu_usage, cpu),
			sizeof(struct rsc_task_cpu_usage_t) * len);
		base += len;

	}

	rsc_task_cpu_usage_last_period = atomic64_read(&rsc_task_cpu_usage_period);
	if (rsc_jiffes_end >= rsc_jiffes_start)
		rsc_consume_jiffes = rsc_jiffes_end - rsc_jiffes_start;
	else
		rsc_consume_jiffes = (U64_MAX - rsc_jiffes_end) + rsc_jiffes_start;
	cpu_usage_last_stime = cpu_usage_stime;
	cpu_usage_last_etime = cpu_usage_etime;
	data_is_new = 1;
	data_is_sorted = 0;
}

/*
#undef SHOW_TASK_INFO_IN_PRINTK
#define SHOW_TASK_INFO_IN_PRINTK
*/
static void do_cpu_usage_sort(void)
{
	int i, j, cpu, len;
	int len0;
	int prelen, cmplen;
	struct rsc_task_cpu_usage_t *data, *base;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
#ifdef SHOW_TASK_INFO_IN_PRINTK
	u64 t0, t1, t2;

	t0 = local_clock();
#endif

	if (data_is_sorted)
		return;

	base = glb_task_cpu_usage;
	data = glb_task_cpu_usage;
	len0 = per_cpu(task_cpu_usage_last_cnt, 0);
	prelen = 0;

#ifdef RSC_RECORD_BIGCORE_USAGE
	if (likely(!cpumask_test_cpu(0, &rsc_cpu_bigcore)))
		memset(glb_task_bigcore_tick, 0, sizeof(u32) * len0);
	else {
		for (i = 0; i < len0; i++)
			glb_task_bigcore_tick[i] = base[i].time;
	}
#endif

	for_each_possible_cpu(cpu) {
		if (cpu) {
			data = base + prelen;
			len = per_cpu(task_cpu_usage_last_cnt, cpu);
			cmplen = len0;
			for (i = 0; i < len; i++) {
#ifdef SHOW_TASK_INFO_IN_PRINTK
			rsc_info("sort merge usage: cpu: %d i: %2d j: %2d comm: %16s "
				"time: %3u utime: %3u stime: %3u pid: %5d tgid: %5d  "
				"tgcomm: %16s ppid: %5d "
	#ifdef RECORD_CPUBIT
				"bit: %2x "
	#endif
				"seq: %4u"
	#ifdef RECORD_STAT_EXPTIME
				" exptime: %5u %5u"
	#endif
				"\n",
				cpu, i, i, data[i].comm, data[i].time,
				data[i].utime, data[i].stime, data[i].pid, data[i].tgid,
				data[i].tgcomm, data[i].ppid,
	#ifdef RECORD_CPUBIT
				data[i].cpubit,
	#endif
				data[i].seq
	#ifdef RECORD_STAT_EXPTIME
				, data[i].exptime, data[i].minexptime
	#endif
				);
#endif
				for (j = 0; j < cmplen; j++) {
					if ((base[j].task == data[i].task) && (base[j].pid == data[i].pid)) {
						base[j].time += data[i].time;
						base[j].stime += data[i].stime;
						base[j].utime += data[i].utime;
#ifdef RSC_USE_TASKLOCK_LIST
						if (base[j].ppid == RSC_END_PID) {
							base[j].ppid = data[i].ppid;
							strlcpy(base[j].tgcomm, data[i].tgcomm, TASK_COMM_LEN);
						}
#endif
#ifdef RSC_RECORD_BIGCORE_USAGE
						if (cpumask_test_cpu(cpu, &rsc_cpu_bigcore))
							glb_task_bigcore_tick[j] += data[i].time;
#endif
						break;
					}
				}
				if (j >= cmplen) {
					memcpy(&base[len0], &data[i],
						sizeof(struct rsc_task_cpu_usage_t));
#ifdef RSC_RECORD_BIGCORE_USAGE
					if (cpumask_test_cpu(cpu, &rsc_cpu_bigcore))
						glb_task_bigcore_tick[len0] = data[i].time;
					else
						glb_task_bigcore_tick[len0] = 0;
#endif
					if (len0 < (NR_CPUS * RSC_STAT_MAX_TASK))
						len0++;
					else
						rsc_err("%s: task num %d too more!\n", __func__, len0);
				}
			}
		}
#ifdef SHOW_TASK_INFO_IN_PRINTK
		if (!cpu) {
			len = per_cpu(task_cpu_usage_last_cnt, cpu);
			for (i = 0; i < len; i++)
				rsc_info("sort merge usage: cpu: %d i: %2d j: %2d comm: %16s "
					"time: %3u utime: %3u stime: %3u pid: %5d tgid: %5d  "
					"tgcomm: %16s ppid: %5d "
	#ifdef RECORD_CPUBIT
					"bit: %2x "
	#endif
					"seq: %4u"
	#ifdef RECORD_STAT_EXPTIME
					" exptime: %5u %5u"
	#endif
					"\n",
					cpu, i, i, data[i].comm, data[i].time,
					data[i].utime, data[i].stime, data[i].pid, data[i].tgid,
					data[i].tgcomm, data[i].ppid,
	#ifdef RECORD_CPUBIT
					data[i].cpubit,
	#endif
					data[i].seq
	#ifdef RECORD_STAT_EXPTIME
					, data[i].exptime, data[i].minexptime
	#endif
					);
		}
#endif
		prelen += per_cpu(task_cpu_usage_last_cnt, cpu);
	}
#ifdef SHOW_TASK_INFO_IN_PRINTK
	t1 = local_clock();
#endif
	tbl = glb_task_cpu_usage_tbl;
	for (j = 0; j < len0; j++)
		tbl[j] = j;

	sort(base, len0, sizeof(struct rsc_task_cpu_usage_t),
	     cmp_task, swap_task);
#ifdef SHOW_TASK_INFO_IN_PRINTK
	t2 = local_clock();
	rsc_info("sort usage, size: %3d merge time: %4u sort time: %4u\n",
		len0, (u32)(t1 - t0), (u32)(t2 - t1));
	data = base;
	for (i = 0; i < len0; i++) {
		j = tbl[i];
		rsc_info("sort usage: %%%2ld i: %2d j: %2d comm: %16s "
			"time: %4u utime: %4u stime: %4u pid: %5d tgid: %5d  "
			"tgcomm: %16s ppid: %5d\n",
			data[j].time*100/RSC_MS_TO_JIFF(atomic64_read(&rsc_task_cpu_usage_period)),
			i, j, data[j].comm, data[j].time,
			data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
			data[j].tgcomm, data[j].ppid);
	}
#endif
	glb_task_cpu_usage_cnt = len0;
	data_is_sorted = 1;
}
/*
#undef SHOW_TASK_INFO_IN_PRINTK
*/
#endif

/*static struct rsc_cpuusg_info rsc_cpu_inf[8];*/
static int cpu_usage_notify(u32 seq)
{
	u32 period;
#ifdef RSC_NOTIFY_BY_LOADING
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
	struct rsc_task_cpu_usage_t *dt, *pt;
	int k, l, len;
#else
	int cpu;
#endif

#ifdef SHOW_TASK_INFO_IN_PRINTK
	int i, j;
	struct rsc_task_cpu_usage_t *data;
	u32 total = 0;
	u8 *map_cnt;
	#if !defined(RSC_NOTIFY_BY_LOADING)
	int len;
	#endif
	#if defined(RSC_NOTIFY_BY_LOADING)
	int cpu;
	#endif

	for_each_possible_cpu(cpu) {
		len = per_cpu(task_cpu_usage_cnt, cpu);
		data = per_cpu(task_cpu_usage, cpu);
		tbl = per_cpu(task_cpu_usage_tbl, cpu);

		for (i = 0; i < len; i++) {
			j = tbl[i];
			rsc_info("usage: cpu: %d i: %2d j: %2d comm: %16s "
				"time: %3u utime: %3u stime: %3u pid: %5d tpid: %5d  "
				"tgcomm: %16s ppid: %5d "
	#ifdef RECORD_CPUBIT
				"bit: %2x "
	#endif
				"seq: %4u"
	#ifdef RECORD_STAT_EXPTIME
				" exptime: %5u %5u"
	#endif
				"\n",
				cpu, i, j, data[j].comm, data[j].time,
				data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
				data[j].tgcomm, data[j].ppid,
	#ifdef RECORD_CPUBIT
				data[j].cpubit,
	#endif
				data[j].seq
	#ifdef RECORD_STAT_EXPTIME
				, data[j].exptime, data[j].minexptime
	#endif
				);
		}
		rsc_info("usage: sort cpu: %d %%%2ld notidle: %4u "
	#ifdef RECORD_IDLE_TICK_CNT
			"idle: %4u "
	#endif
			"total: %4u\n",
			 cpu, per_cpu(task_cpu_usage_notidle, cpu)*100/RSC_MS_TO_JIFF(atomic64_read(&rsc_task_cpu_usage_period)),
			per_cpu(task_cpu_usage_notidle, cpu),
	#ifdef RECORD_IDLE_TICK_CNT
			per_cpu(task_cpu_usage_idle, cpu),
	#endif
			per_cpu(task_cpu_usage_total, cpu));
		rsc_info("usage: cpu: %d jiffes: %llu\n", cpu,
			rsc_jiffes_end - rsc_jiffes_start);

		rsc_info("usage: cpu: %d maxusage: %4u minuage: %4u\n", cpu,
			per_cpu(task_cpu_usage_max_tick_const, cpu),
			per_cpu(task_cpu_usage_min_tick_const, cpu));

		total +=  per_cpu(task_cpu_usage_notidle, cpu);

		map_cnt = per_cpu(task_cpu_usage_map_cnt, cpu);
		for (j = 0; j < RSC_MAX_HASH_TASK; j++)
			if (map_cnt[j])
				rsc_info("usage: cpu: %d map_cnt[%3d]: %2d\n", cpu, j, (int)map_cnt[j]);
	}

	rsc_info("usage: sort total cpuusage: %%%2ld total: %4d\n",
		total*100/(cpu * RSC_MS_TO_JIFF(atomic64_read(&rsc_task_cpu_usage_period))), total);

	#ifdef RECORD_GET_LOCK_FAIL
	rsc_info("usage:  task_get_lock_fail_cnt: %d\n", atomic_read(&task_get_lock_fail_cnt));

	len = atomic_read(&task_get_lock_fail_cnt) + 1;
	data = &task_cpu_usage_fail[0];
	for (j = 0; j < len; j++) {
		rsc_info("getlockfail!usage: cpu: %d j: %2d tbl: %2d comm: %16s "
		"time: %5u utime: %5u stime: %5u pid: %5d tpid: %5d  "
		"tgcomm: %16s ppid: %5d "
		#ifdef RECORD_CPUBIT
		"cpubit: 0x%2x "
		#endif
		"seq: %4u"
		#ifdef RECORD_STAT_EXPTIME
		" exptime: %5u %5u"
		#endif
		"\n",
		cpu, j, tbl[j], data[j].comm, data[j].time,
		data[j].utime, data[j].stime, data[j].pid, data[j].tgid,
		data[j].tgcomm, data[j].ppid,
		#ifdef RECORD_CPUBIT
		data[j].cpubit,
		#endif
		data[j].seq
		#ifdef RECORD_STAT_EXPTIME
		, data[j].exptime, data[j].minexptime
		#endif
		);
	}
	#endif
#endif

	mutex_lock(&cpu_usage_lock);

	rsc_cpu_usage_disable();
	if (seq == (u32)atomic_read(&task_cpu_usage_notify_seq)) {
#ifdef SORT_IN_USER_PROCESS
		rsc_cpu_usage_backup();
		period = (u32)rsc_consume_jiffes;

		trace_rsc_cpu_usage(task_cpu_usage_last_notidle,
			glb_task_cpu_num, period);
		/*detail information*/
		trace_rsc_cpu_usaged(task_cpu_usage_last_notidle,
			task_cpu_usage_last_user,
			task_cpu_usage_last_hirq,
			task_cpu_usage_last_sirq,
			glb_task_cpu_num, period);
	#ifdef RSC_NOTIFY_BY_LOADING
		/*if total usage if less than notify_percent, quickly return!*/
		if ((glb_task_cpu_usage_total * 100) >= (period * notify_percent))
			do_cpu_usage_sort();
		else
			goto out;
	#endif
#else
		rsc_cpu_usage_sort();
		period = (u32)rsc_consume_jiffes;

		trace_rsc_cpu_usage(task_cpu_usage_last_notidle,
			glb_task_cpu_num, period);
		trace_rsc_cpu_usaged(task_cpu_usage_last_notidle,
			task_cpu_usage_last_user,
			task_cpu_usage_last_hirq,
			task_cpu_usage_last_sirq,
			glb_task_cpu_num, period);
#endif

#ifdef RSC_NOTIFY_BY_LOADING
		dt = glb_task_cpu_usage;
		tbl = glb_task_cpu_usage_tbl;
		len = glb_task_cpu_usage_cnt;

		for (k = 0; k < len; k++) {
			pt = &dt[tbl[k]];
			if ((pt->time * 100) >= (period * notify_percent)) {
				for (l = 0; l < cpu_usage_fpid_cnt; l++) {
					if (cpu_usage_fpid[l] == pt->tgid) {
						break;
					}
				}
				/*not in the fpid list! notify it!*/
				if (l >= cpu_usage_fpid_cnt) {
					sysfs_notify(rsc_root_dir, NULL, __stringify(CPU_USAGE_NAME));
					break;
				}
			} else
				break;
		}
#else
		for_each_possible_cpu(cpu) {
			/*notify when any one cpu usage bigger than notify_percent!*/
			if ((per_cpu(task_cpu_usage_notidle, cpu) * 100)
				>= (period * notify_percent)) {
				sysfs_notify(rsc_root_dir, NULL, __stringify(CPU_USAGE_NAME));
				break;
			}
		}
#endif
	} else
		rsc_info("%s: ignore seq: %u != task_cpu_usage_notify_seq: %u\n",
			__func__, seq, atomic_read(&task_cpu_usage_notify_seq));
out:
	rsc_cpu_usage_enable();
	mutex_unlock(&cpu_usage_lock);

	return 0;
}

long rsc_cpu_usage_notify(long comret)
{
	static int init_once;
	static long last_period;
	static u64 sjiffs;
	static u32 noti_seq;
	u64 period;
	u64 ejiffs;
	long left;

	if (!init_once) {
		init_once = 1;
		last_period = CPU_USAGE_INIT_PERIOD;/*atomic64_read(&rsc_task_cpu_usage_period);*/
		noti_seq = (u32)atomic_read(&task_cpu_usage_notify_seq);
		sjiffs = get_jiffies_64();
	}

	period = atomic64_read(&rsc_task_cpu_usage_period);
	/*rsc_info("completion: comret: %ld period: %ld last_period: %ld\n",
		comret, period, last_period);*/
	/*
	   from disable to enable or enable to disable,
	   it should not notify!
	*/
	if (!comret && (period != -1) && (last_period == period))
			cpu_usage_notify(noti_seq);

	if (last_period != period || !comret) {
		if (period <= 0)
			left = MAX_SCHEDULE_TIMEOUT;
		else
			left = msecs_to_jiffies(period);
		sjiffs = get_jiffies_64();
		rsc_dbg(CPU_TASK_USAGE, "completion: reset comret: %ld period: %lld last_period: %ld left: %ld\n",
			comret, period, last_period, left);
		last_period = period;
		noti_seq = (u32)atomic_read(&task_cpu_usage_notify_seq);
	} else {
		if (period > 0) {
			ejiffs = get_jiffies_64();
			if (time_after_eq64(ejiffs,
				sjiffs + msecs_to_jiffies(period))) {
				left = 0;
				sjiffs = ejiffs;
			} else {
				if (ejiffs > sjiffs)
					left = msecs_to_jiffies(period) - (ejiffs - sjiffs);
				else
					left = msecs_to_jiffies(period) - (sjiffs + (U64_MAX - sjiffs));
			}
		} else
			left = MAX_SCHEDULE_TIMEOUT;

		rsc_dbg(CPU_TASK_USAGE, "completion: normalreset comret: %ld period: %lld last_period: %ld left: %ld\n",
			comret, period, last_period, left);

		/*last_period = period;*/
	}
	return left;
}

#if defined(RSC_RECORD_UID_DEBUG) || !defined(RSC_RECORD_UID)
#define UID_ERR 99999
static inline void rsc_get_uid(pid_t pid, pid_t tgid,
	u32 *ac_uid, u32 *ac_gid)
{
	struct task_struct *tsk;
	struct user_namespace *user_ns = current_user_ns();
	const struct cred *tcred;

	rcu_read_lock();
	tsk = find_task_by_vpid(pid);
	if (!tsk && (pid != tgid))
		tsk = find_task_by_vpid(tgid);
	if (tsk)
		get_task_struct(tsk);

#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	();

	if (tsk) {
		tcred = __task_cred(tsk);
		*ac_uid	 = from_kuid_munged(user_ns, tcred->uid);
		*ac_gid	 = from_kgid_munged(user_ns, tcred->gid);
		put_task_struct(tsk);
	} else {
		*ac_uid = UID_ERR;
		*ac_gid = UID_ERR;
	}
}
#endif

#if defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
extern int rsc_cpucap_percent;
#endif

#if !defined(CONFIG_MTK_PLATFORM)
extern unsigned int sysctl_sched_boost;
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
extern struct cpumask cpus_isolated_by_thermal;
#endif

#if defined(CONFIG_RSC_MEM_MON) && defined(CONFIG_RSC_MON_CPU_HOTPLUG)
extern int rsc_cpu_up_fail_suspend;
extern int rsc_cpu_up_fail_other;
extern int rsc_cpu_up_fail_policy;
#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
extern int rsc_cpu_up_act_fail_suspend;
extern struct timeval rsc_cpu_up_fail_stime;
extern int rsc_cpu_up_fail_shell_cnt;
extern bool rsc_fix_cpu_hotplug_disable;
extern u32 rsc_cpu_up_fail_elaps;
extern u32 rsc_cpu_up_fail_max_elaps;
extern u32 rsc_cpu_down_elaps[NR_CPUS];
extern u32 rsc_cpu_down_max_elaps[NR_CPUS];
extern u32 rsc_cpu_down_whole_elaps;
extern u32 rsc_cpu_down_whole_max_elaps;
extern struct rsc_cpu_hotplug_info rsc_cpu_hp_info[NR_CPUS];
extern struct rsc_cpu_hotplug_info rsc_cpu_cpuup_fail_info;
extern struct cpumask rsc_cpu_up_fail_cpus_app_nofix;
extern int rsc_cpu_up_app_nofix_cnt;
extern int rsc_cpu_up_state_notmatch_cnt;
extern int rsc_cpu_up_state_shell_notmatch_cnt;
#endif
#endif
static ssize_t show_rsc_cpu_usage(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct rsc_task_cpu_usage_t *data;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
	int i, j;
	int cur = 0, ret = 0;
	u32 tick;
	/*u32 period;*/
	int len, usg, cpu;
	int systick;
	struct rtc_time tmstart, tmend;
	struct cpumask offline;

	mutex_lock(&cpu_usage_lock);
	if (!data_is_new) {
		#define RSC_PRINT_CPU_USAGE_NOTUD									\
			"usage notupdate\t%u\t%u\t%u\t%u\t%d\t%d\t%d\n", 				\
			rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,						\
			rsc_max_freq[0], rsc_max_freq[1],								\
			rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1],			\
			atomic_read(&task_cpu_usage_enable)
		cur = snprintf(buf + ret, PAGE_SIZE - ret, RSC_PRINT_CPU_USAGE_NOTUD);
		ret += cur;
		rsc_dbg(CPU_TASK_USAGE, RSC_PRINT_CPU_USAGE_NOTUD);
		mutex_unlock(&cpu_usage_lock);

		return ret;
	}

#ifdef SORT_IN_USER_PROCESS
	do_cpu_usage_sort();
#endif

	data = glb_task_cpu_usage;
	tbl = glb_task_cpu_usage_tbl;
	len = min(glb_task_cpu_usage_cnt, (u32)RSC_SHOW_MAX_TASK);

	/*period = rsc_task_cpu_usage_last_period;
	tick = RSC_MS_TO_JIFF(period);*/
	tick = rsc_consume_jiffes;
	if (!tick)
		tick = 1;
	if (!glb_task_cpu_num)
		glb_task_cpu_num = 1;
	/*usg = min((u32)100, glb_task_cpu_usage_total*100/(tick * glb_task_cpu_num));*/

	rtc_time_to_tm((unsigned long)cpu_usage_last_stime.tv_sec-sys_tz.tz_minuteswest * 60, &tmstart);
	rtc_time_to_tm((unsigned long)cpu_usage_last_etime.tv_sec-sys_tz.tz_minuteswest * 60, &tmend);
	usg = min((u32)100, DIV_ROUND_CLOSEST(glb_task_cpu_usage_total*100, tick * glb_task_cpu_num));
#if defined(RSC_RECORD_BIGCORE_USAGE)
	#define RSC_PRINT_CPU_USAGE_HEAD														\
		"cpu_usage:\t%3d\tnum:\t%2d\tenable:\t%d\ttotal:\t%5u\t"							\
		"user:\t%4u\tsys:\t%4u\thirq:\t%4u\t"												\
		"sirq:\t%4u\tperiod:\t%5u\tfrom:\t%04d-%02d-%02d "									\
		"%02d:%02d:%02d ~ %02d:%02d:%02d\tstop\t%llu\tHZ\t%u"								\
		"\tclust\t%d"																		\
		"\n",																				\
		usg, len, atomic_read(&task_cpu_usage_enable),										\
		glb_task_cpu_usage_total, glb_task_cpu_usage_user,									\
		glb_task_cpu_usage_total - glb_task_cpu_usage_user -								\
		glb_task_cpu_hirq_total - glb_task_cpu_sirq_total,									\
		glb_task_cpu_hirq_total, glb_task_cpu_sirq_total, tick,								\
		tmstart.tm_year+1900, tmstart.tm_mon+1, tmstart.tm_mday,							\
		tmstart.tm_hour, tmstart.tm_min, tmstart.tm_sec,									\
		tmend.tm_hour, tmend.tm_min, tmend.tm_sec,											\
		cpu_usage_disable_time, CONFIG_HZ, rsc_cluster_num
#else
	#define RSC_PRINT_CPU_USAGE_HEAD														\
		"cpu_usage:\t%3d\tnum:\t%2d\tenable:\t%d\ttotal:\t%5u\t"							\
		"user:\t%4u\tsys:\t%4u\thirq:\t%4u\t"												\
		"sirq:\t%4u\tperiod:\t%5u\tfrom:\t%04d-%02d-%02d "									\
		"%02d:%02d:%02d ~ %02d:%02d:%02d\tstop\t%llu\tHZ\t%u"								\
		"\n",																				\
		usg, len, atomic_read(&task_cpu_usage_enable),										\
		glb_task_cpu_usage_total, glb_task_cpu_usage_user,									\
		glb_task_cpu_usage_total - glb_task_cpu_usage_user -								\
		glb_task_cpu_hirq_total - glb_task_cpu_sirq_total,									\
		glb_task_cpu_hirq_total, glb_task_cpu_sirq_total, tick,								\
		tmstart.tm_year+1900, tmstart.tm_mon+1, tmstart.tm_mday,							\
		tmstart.tm_hour, tmstart.tm_min, tmstart.tm_sec,									\
		tmend.tm_hour, tmend.tm_min, tmend.tm_sec,											\
		cpu_usage_disable_time, CONFIG_HZ
#endif

	cur = snprintf(buf + ret, PAGE_SIZE - ret, RSC_PRINT_CPU_USAGE_HEAD);
	ret += cur;
	/*rsc_info(CPU_TASK_USAGE, RSC_PRINT_CPU_USAGE_HEAD);*/
	rsc_dbg(CPU_TASK_USAGE, RSC_PRINT_CPU_USAGE_HEAD);
	/*
	* percpu usage    1708800 2016000 1708800 2016000 -1      -1
	* 0:100:1023:40:973:7:3   1:100:1023:92:973:6:1   2:100:1023:47:973:4:0
	* 3:100:1022:81:973:4:1   4:100:1023:16:973:3:0   5:100:1023:22:973:3:0
	* 6:100:1023:14:973:2:2   7:100:1023:29:973:2:0
	*/
	/*
	* example: percpu usage    1708800 2016000 1708800 2016000 -1      -1 means
	* 	cluster0 freq 1708800, cluster1 freq 2016000
	* 	cluster0 soft maxfreq 1708800, cluster0 soft maxfreq 2016000
	* 	cluster0 hard maxfreq -1 ,cluste1 hard maxfreq -1(-1 mean no limit)
	*/
#if defined(RSC_RECORD_BIGCORE_USAGE)
	if (rsc_cluster_num > 2) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "percpu usage\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t",
			rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new, rsc_cpu_freqs[2].new,
			rsc_max_freq[0], rsc_max_freq[1], rsc_max_freq[2],
			rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1], rsc_max_mitigated_freq[2]);
		ret += cur;
		rsc_dbg(CPU_TASK_USAGE, "percpu usage\t%u\t%u\t%u\t%u\t%d\t%d\t",
			rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,
			rsc_max_freq[0], rsc_max_freq[1],
			rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1]);
	} else {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "percpu usage\t%d\t%d\t%d\t%d\t%d\t%d\t",
			rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,
			rsc_max_freq[0], rsc_max_freq[1],
			rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1]);
		ret += cur;
		rsc_dbg(CPU_TASK_USAGE, "percpu usage\t%u\t%u\t%u\t%u\t%d\t%d\t",
			rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,
			rsc_max_freq[0], rsc_max_freq[1],
			rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1]);
	}
#else
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "percpu usage\t%d\t%d\t%d\t%d\t%d\t%d\t",
		rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,
		rsc_max_freq[0], rsc_max_freq[1],
		rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1]);
	ret += cur;
	rsc_dbg(CPU_TASK_USAGE, "percpu usage\t%u\t%u\t%u\t%u\t%d\t%d\t",
		rsc_cpu_freqs[0].new, rsc_cpu_freqs[1].new,
		rsc_max_freq[0], rsc_max_freq[1],
		rsc_max_mitigated_freq[0], rsc_max_mitigated_freq[1]);
#endif

	/*
	* example: 	6:100:1023:14:973:2:2 means
	*			cpu6:pencent usage 100%:total tick 1023:usr tick 14:sys tick 973:hard irq tick 2: soft irq tick 2
	*/
	#define RSC_PRINT_PERCPU_USAGE	"%u:%u:%u:%u:%d:%u:%u\t"					\
			, cpu 																\
			, DIV_ROUND_CLOSEST(task_cpu_usage_last_notidle[cpu]*100, tick)		\
			, task_cpu_usage_last_notidle[cpu]									\
			, task_cpu_usage_last_user[cpu] 									\
			, systick															\
			, task_cpu_usage_last_hirq[cpu]										\
			, task_cpu_usage_last_sirq[cpu]

	for_each_possible_cpu(cpu) {
		if (task_cpu_usage_last_notidle[cpu] > (task_cpu_usage_last_user[cpu] +	\
			task_cpu_usage_last_hirq[cpu] + task_cpu_usage_last_sirq[cpu]))
			systick = task_cpu_usage_last_notidle[cpu] - (task_cpu_usage_last_user[cpu] +	\
				task_cpu_usage_last_hirq[cpu] + task_cpu_usage_last_sirq[cpu]);
		else
			systick = 0;
		cur = snprintf(buf + ret, PAGE_SIZE - ret, RSC_PRINT_PERCPU_USAGE);
		ret += cur;
		rsc_dbg(CPU_TASK_USAGE, RSC_PRINT_PERCPU_USAGE);
	}

#if 1
	{
		cpumask_andnot(&offline, cpu_possible_mask, cpu_online_mask);

		cur = snprintf(buf + ret, PAGE_SIZE - ret,
#if defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
			"\tcpucap: %d"
#endif
			"\toffline: %*pbl"
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
			"\tthermal: %*pbl"
#endif
#if !defined(CONFIG_MTK_PLATFORM)
			"\tiso: %*pbl"
#endif
#if !defined(CONFIG_MTK_PLATFORM)
			"\tsched_boost: %d"
#endif
#if defined(CONFIG_RSC_VAUDIT)
			"\tiover: %lu heavyio: %lu"
#endif
#if defined(CONFIG_RSC_MEM_MON) && defined(CONFIG_RSC_MON_CPU_HOTPLUG)
			"\tcpuupfail: %d %d %d"
#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
			" %d%s"
#else
			"\n"
#endif
#else
			"\n"
#endif

#if defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
			, rsc_cpucap_percent
#endif
			, cpumask_pr_args(&offline)
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0) && !defined(CONFIG_MTK_PLATFORM)
			, cpumask_pr_args(&cpus_isolated_by_thermal)
#endif
#if !defined(CONFIG_MTK_PLATFORM)
			, cpumask_pr_args(cpu_isolated_mask)
#endif
#if !defined(CONFIG_MTK_PLATFORM)
			, sysctl_sched_boost
#endif
#if defined(CONFIG_RSC_VAUDIT)
			, rsc_vaudit_iowait_over_count, rsc_vaudit_backtrace_show_count
#endif
#if defined(CONFIG_RSC_MEM_MON) && defined(CONFIG_RSC_MON_CPU_HOTPLUG)
			, rsc_cpu_up_fail_suspend, rsc_cpu_up_fail_other, rsc_cpu_up_fail_policy
#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
			, rsc_fix_cpu_hotplug_disable
			, (rsc_cpu_up_fail_suspend || (!cpumask_empty(&offline)))?"":"\n"
#endif
#endif
			);
		ret += cur;

#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
		if (rsc_cpu_up_fail_suspend) {
			struct rtc_time failtime;

			rtc_time_to_tm((unsigned long)rsc_cpu_up_fail_stime.tv_sec-sys_tz.tz_minuteswest * 60, &failtime);
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"\tappfix: %d %*pbl"
				"\tnotmat: %d %d"
				"\tfailtime: %u %u"
				"\tcpuupfix: %d %d"
				"\tkiller: %s %d(%s %d)"
				"\telaps: %u %u"
				"\t%04d-%02d-%02d %02d:%02d:%02d"
				, (!cpumask_empty(&rsc_cpu_up_fail_cpus_app_nofix))?rsc_cpu_up_app_nofix_cnt+1:rsc_cpu_up_app_nofix_cnt
				, cpumask_pr_args(&rsc_cpu_up_fail_cpus_app_nofix)
				, rsc_cpu_up_state_notmatch_cnt, rsc_cpu_up_state_shell_notmatch_cnt
				, rsc_cpu_down_whole_elaps
				, rsc_cpu_down_whole_max_elaps
				, rsc_cpu_up_act_fail_suspend
				, rsc_cpu_up_fail_shell_cnt
				, rsc_cpu_cpuup_fail_info.down_comm
				, rsc_cpu_cpuup_fail_info.down_pid
				, rsc_cpu_cpuup_fail_info.down_pcomm
				, rsc_cpu_cpuup_fail_info.down_ppid
				, rsc_cpu_up_fail_elaps
				, rsc_cpu_up_fail_max_elaps
				, failtime.tm_year+1900, failtime.tm_mon+1, failtime.tm_mday
				, failtime.tm_hour, failtime.tm_min, failtime.tm_sec
			);
			ret += cur;
		}
		if (rsc_cpu_up_fail_suspend || !cpumask_empty(&offline)) {
			struct rtc_time failtime;
			int scpu = 0;
			u32 dwelaps;

#ifdef CONFIG_ARCH_SDM439
			for (i = 0; i < NR_CPUS; i++)
				if (rsc_cpu_hp_info[i].down_time) {
					scpu = i;
					break;
				}
#else
			for (i = NR_CPUS-1; i >= 0; i--)
				if (rsc_cpu_hp_info[i].down_time) {
					scpu = i;
					break;
				}
#endif
			if (rsc_cpu_hp_info[scpu].up_time >= rsc_cpu_hp_info[scpu].down_time)
				dwelaps = rsc_cpu_hp_info[scpu].up_time - rsc_cpu_hp_info[scpu].down_time;
			else {
				if (!cpumask_empty(&offline))
					dwelaps = 93949394;
				else
					dwelaps = 0;
			}

			rtc_time_to_tm((unsigned long)rsc_cpu_hp_info[scpu].down_time-sys_tz.tz_minuteswest * 60, &failtime);
			cur = snprintf(buf + ret, PAGE_SIZE - ret,
				"\tcpuhp: %d %s %d(%s %d)"
				"\tdwelaps: %u"
				"\t%04d-%02d-%02d %02d:%02d:%02d\n"
				, scpu
				, rsc_cpu_hp_info[scpu].down_comm
				, rsc_cpu_hp_info[scpu].down_pid
				, rsc_cpu_hp_info[scpu].down_pcomm
				, rsc_cpu_hp_info[scpu].down_ppid
				, dwelaps
				, failtime.tm_year+1900, failtime.tm_mon+1, failtime.tm_mday
				, failtime.tm_hour, failtime.tm_min, failtime.tm_sec
			);
			ret += cur;
		}
#endif
	}
#else
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	ret += cur;
#endif
	rsc_dbg(CPU_TASK_USAGE, "\n");

#if defined(RSC_RECORD_UID_DEBUG)
	#define RSC_PRINT_CPU_USAGE_BEG 	"ID\tusg\t%5sThread%5s\t"			\
		" TIME\tUTIME\tSTIME\t  PID\t TGID\t"								\
		"%5sTGTASK%5s\t PPID\t  UID\t RUID\t  GID\n", " ", " ", " ", " "
#elif defined(RSC_RECORD_UID)
	#if defined(RSC_RECORD_BIGCORE_USAGE)
	/*
	* busg means bigcore usage percent,
	* example: 1000 256 means 100% run in big core and cpu usage is 25.6%
	*/
	#define RSC_PRINT_CPU_USAGE_BEG 	"ID\tusg\t%5sThread%5s\t"			\
	" TIME\tUTIME\tSTIME\t  PID\t TGID\t"									\
	"%5sTGTASK%5s\t PPID\t  UID\t bigcoreusg\n", " ", " ", " ", " "
	#else
	#define RSC_PRINT_CPU_USAGE_BEG 	"ID\tusg\t%5sThread%5s\t"			\
		" TIME\tUTIME\tSTIME\t  PID\t TGID\t"								\
		"%5sTGTASK%5s\t PPID\t  UID\t  GID\n", " ", " ", " ", " "
	#endif
#else
	#define RSC_PRINT_CPU_USAGE_BEG 	"ID\tusg\t%5sThread%5s\t"			\
		" TIME\tUTIME\tSTIME\t  PID\t TGID\t"								\
		"%5sTGTASK%5s\t PPID\t  UID\t  GID\n", " ", " ", " ", " "
#endif

	cur = snprintf(buf + ret, PAGE_SIZE - ret, RSC_PRINT_CPU_USAGE_BEG);
	/*rsc_info(RSC_PRINT_CPU_USAGE_BEG);*/
	rsc_dbg(CPU_TASK_USAGE, RSC_PRINT_CPU_USAGE_BEG);
	ret += cur;

	for (i = 0; i < len; i++) {
#if defined(RSC_RECORD_UID_DEBUG) || !defined(RSC_RECORD_UID)
		u32 ac_uid, ac_gid;
#endif

		j = tbl[i];
#if defined(RSC_RECORD_UID_DEBUG) || !defined(RSC_RECORD_UID)
		rsc_get_uid(data[j].pid, data[j].tgid, &ac_uid, &ac_gid);
#endif

#if defined(RSC_RECORD_UID_DEBUG)
	#define RSC_PRINT_CPU_USAGE 	"%2d\t%3d\t%16s\t"						\
			"%5u\t%5u\t%5u\t%5d\t%5d\t" 									\
			"%16s\t%5d\t%5d\t%5d\t%5d\n",									\
			i, min((u32)100, DIV_ROUND_CLOSEST(data[j].time*100, tick)),	\
			data[j].comm, data[j].time, data[j].utime, data[j].stime,		\
			data[j].pid, data[j].tgid, data[j].tgcomm, data[j].ppid,		\
			ac_uid, data[j].uid, ac_gid
#elif defined(RSC_RECORD_UID)
	#if defined(RSC_RECORD_BIGCORE_USAGE)
	#define RSC_PRINT_CPU_USAGE 	"%2d\t%3d\t%16s\t"						\
			"%5u\t%5u\t%5u\t%5d\t%5d\t"										\
			"%16s\t%5d\t%5d\t%4d %4d %d\n",									\
			i, min((u32)100, DIV_ROUND_CLOSEST(data[j].time*100, tick)),	\
			data[j].comm, data[j].time, data[j].utime, data[j].stime,		\
			data[j].pid, data[j].tgid, data[j].tgcomm, data[j].ppid,		\
			data[j].uid,													\
			data[j].time?DIV_ROUND_CLOSEST(glb_task_bigcore_tick[j]*1000, data[j].time):0,	\
			DIV_ROUND_CLOSEST(glb_task_bigcore_tick[j]*1000, tick), data[j].group
	#else
	/*
	* assume uid==gid for performance and save memory,
	* show gid just for compatibility that the app needs the same length.
	*/
	#define RSC_PRINT_CPU_USAGE 	"%2d\t%3d\t%16s\t"						\
			"%5u\t%5u\t%5u\t%5d\t%5d\t"										\
			"%16s\t%5d\t%5d\t%5d\n",										\
			i, min((u32)100, DIV_ROUND_CLOSEST(data[j].time*100, tick)),	\
			data[j].comm, data[j].time, data[j].utime, data[j].stime,		\
			data[j].pid, data[j].tgid, data[j].tgcomm, data[j].ppid,		\
			data[j].uid, data[j].uid
	#endif
#else
	#define RSC_PRINT_CPU_USAGE 	"%2d\t%3d\t%16s\t"						\
			"%5u\t%5u\t%5u\t%5d\t%5d\t"										\
			"%16s\t%5d\t%5d\t%5d\t%5d\n",									\
			i, min((u32)100, DIV_ROUND_CLOSEST(data[j].time*100, tick)),	\
			data[j].comm, data[j].time, data[j].utime, data[j].stime,		\
			data[j].pid, data[j].tgid, data[j].tgcomm, data[j].ppid,		\
			ac_uid, data[j].uid, ac_gid
#endif

		cur = snprintf(buf + ret, PAGE_SIZE - ret, RSC_PRINT_CPU_USAGE);
		/*rsc_info(RSC_PRINT_CPU_USAGE);*/
		rsc_dbg(CPU_TASK_USAGE, RSC_PRINT_CPU_USAGE);
		ret += cur;
		if (cur <= 0)
			break;
	}

#if defined(RSC_RECORD_BIGCORE_USAGE)
	/*debug*/
	if (unlikely(rsc_debug & (CPU_TASK_LATENCY | CPU_TASK_USAGE))) {
		u32 bigticks = 0;
		u32 bigirqticks = 0, cpubigticks = 0;
		u32 appticks = 0, irqticks = 0;

		cur = snprintf(buf + ret, PAGE_SIZE - ret, "cputskcnt\t");
		ret += cur;
		for_each_possible_cpu(cpu) {
			if (cpumask_test_cpu(cpu, &rsc_cpu_bigcore)) {
				cpubigticks += task_cpu_usage_last_notidle[cpu];
				bigirqticks += task_cpu_usage_last_hirq[cpu] + task_cpu_usage_last_sirq[cpu];
			}
			irqticks += task_cpu_usage_last_hirq[cpu] + task_cpu_usage_last_sirq[cpu];
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%d:%d\t"
#ifdef RSC_RECORD_LATENCY
				"lat\t%u:%u:%u:%u\t"
#endif
				, cpu, per_cpu(task_cpu_usage_last_cnt, cpu)
#ifdef RSC_RECORD_LATENCY
				, rsc_ktop_min_latency[cpu]
				, rsc_ktop_cur_latency[cpu]
				, rsc_ktop_max_latency[cpu]
				, rsc_ktop_avg_latency[cpu]
#endif
			);
			ret += cur;
		}
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "\n");
		ret += cur;

		for (i = 0; i < glb_task_cpu_usage_cnt; i++) {
			bigticks += glb_task_bigcore_tick[i];
			appticks += data[i].time;
		}
		cur = snprintf(buf + ret, PAGE_SIZE - ret,
			"cluster_num\t%d\ttopeff\t%lu\tmaxeff\t%lu\tmineff\t%lu\tbigcore\t%*pbl\tbigtick\t%u(%u)\tpercpubigtick\t"
			"%u\tbigirqtick\t%u\tapptick\t%u\tirqtick\t%u\ttotal\t%u\ttaskcnt\t%u\n\n",
			rsc_cluster_num, rsc_top_possible_efficiency, rsc_max_possible_efficiency,
			rsc_min_possible_efficiency, cpumask_pr_args(&rsc_cpu_bigcore), bigticks,
			cpubigticks-bigirqticks, cpubigticks, bigirqticks,
			appticks, irqticks, glb_task_cpu_usage_total,
			glb_task_cpu_usage_cnt);
		ret += cur;
		rsc_dbg(CPU_TASK_USAGE, "cluster_num\t%d\ttopeff\t%lu\tmaxeff\t%lu\tmineff\t%lu\tbigcore\t%*pbl\tbigtick\t%u(%u)\tpercpubigtick\t"
			"%u\tbigirqtick\t%u\tapptick\t%u\tirqtick\t%u\ttotal\t%u\ttaskcnt\t%u\n\n",
			rsc_cluster_num, rsc_top_possible_efficiency, rsc_max_possible_efficiency,
			rsc_min_possible_efficiency, cpumask_pr_args(&rsc_cpu_bigcore), bigticks,
			cpubigticks-bigirqticks, cpubigticks, bigirqticks,
			appticks, irqticks, glb_task_cpu_usage_total,
			glb_task_cpu_usage_cnt);
	}
#endif

	mutex_unlock(&cpu_usage_lock);

	return ret;
}
#if 0
static ssize_t store_rsc_cpu_usage(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}
#endif
struct cpu_usage_dwork_t {
	u32 id;
	struct delayed_work	work;   /* Work handler */
};

static struct cpu_usage_dwork_t cpu_usage_delay_work[RSC_ID_END - RSC_ID_START];
/*20 minutes == 1200s, after 1200s, disable it automatically*/
#define DEFAULT_DELAYED_WORK_TM 1200 /*second*/
static const unsigned long cpu_usage_delayed_work_timeout = RSC_S_TO_JIFF(DEFAULT_DELAYED_WORK_TM);/*jiffes, 600s*/
static  u64  cpu_usage_set_time[RSC_ID_END - RSC_ID_START];

static ssize_t show_rsc_cpu_usage_set(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur = 0;
	int i, asize = ARRAY_SIZE(rsc_ids);
	u64 jiff;

	mutex_lock(&cpu_usage_lock);
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "rsc_ktop_map:\t%lx\tktop_enable:\t%d period:\t%ldms\t"
		"notify_percent:\t%d\t%d\tusage:\n",
		rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
		(long)atomic64_read(&rsc_task_cpu_usage_period), notify_percent,
		forcestart_work);
	ret += cur;

	jiff = get_jiffies_64();
	for (i = 0; i < asize; i++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "echo %28s 1 > /sys/rsc/ktop_set", rsc_ids[i].name);
		ret += cur;
		if (rsc_ktop_map & (1 << i)) {
			u64 left;

			if (jiff >= cpu_usage_set_time[i])
				left = jiff - cpu_usage_set_time[i];
			else
				left = (U64_MAX - cpu_usage_set_time[i]) + jiff;

			if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id) {
				if (left < cpu_usage_delayed_work_timeout)
					left = cpu_usage_delayed_work_timeout - left;
				else
					left = 0;
				cur = snprintf(buf + ret, PAGE_SIZE - ret, "\tleft time: %8ums\n",
					jiffies_to_msecs((unsigned long)left));
				ret += cur;
			} else {
				cur = snprintf(buf + ret, PAGE_SIZE - ret, "\telap time: %8llums\n",
					RSC_JIFF_TO_MS(left));
				ret += cur;
			}
		} else {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "\n");
			ret += cur;
		}
	}
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "echo %30s > /sys/rsc/ktop_set\n", "forcestart 1 period: 10000");
	ret += cur;
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "echo %30s > /sys/rsc/ktop_set\n", "forcestop 0");
	ret += cur;

	mutex_unlock(&cpu_usage_lock);

	return ret;
}

static void cpu_usage_work_handler(struct work_struct *work)
{
	struct cpu_usage_dwork_t *delay_work =
		container_of(work, struct cpu_usage_dwork_t, work.work);
	int idx = delay_work->id - RSC_ID_START;

	if (idx >= RSC_ID_END - RSC_ID_START)
		return;

	mutex_lock(&cpu_usage_lock);
	clear_bit(delay_work->id - RSC_ID_START, &rsc_ktop_map);
	cpu_usage_set_time[idx] = 0;
	if (!rsc_ktop_map) {
		if (forcestart_work) {
			rsc_info("delaywork forcestart_work ktop not disable! id: %s rsc_ktop_map: 0x%lx period: %ldms\n",
				rsc_ids[idx].name, rsc_ktop_map, (long)atomic64_read(&rsc_task_cpu_usage_period));
				goto out;
		}
		if (atomic_read(&task_cpu_usage_enable)) {
			/*-1 mean disable*/
			atomic64_set(&rsc_task_cpu_usage_period, -1);
			rsc_cpu_usage_disable();
			complete(&cpucap_notify_complete);
			data_is_new = 0;
			rsc_info("delaywork ktop disable! id: %s rsc_ktop_map: 0x%lx period: %ldms\n",
				rsc_ids[idx].name, rsc_ktop_map, (long)atomic64_read(&rsc_task_cpu_usage_period));
		} else
			rsc_info("delaywork ktop has been disable! id: %s\n", rsc_ids[idx].name);
	} else
		rsc_info("delaywork ktop not disable!"
				"id: %s rsc_ktop_map: 0x%lx enable: %d period: %ldms\n",
				rsc_ids[idx].name, rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
				(long)atomic64_read(&rsc_task_cpu_usage_period));
out:
	mutex_unlock(&cpu_usage_lock);
}

/*
usage:
	enable:
		echo RSC_ID_BIGDATA 1 > /sys/rsc/ktop_set
		or
		echo RSC_ID_POWER 1 > /sys/rsc/ktop_set
	disable:
		echo RSC_ID_BIGDATA 0 > /sys/rsc/ktop_set
		or
		echo RSC_ID_POWER 0 > /sys/rsc/ktop_set

	forcestop:
		echo forcestop 0 > /sys/rsc/ktop_set
	forcestart:
		echo forcestart 1 period: 10000 > /sys/rsc/ktop_set
	see rsc_ids
note:
when  processed(bbklog and rms)  that enable RSC_ID_BBKLOG and RSC_ID_BIGDATA
was killed, it will restart always.
1. when rms restart, it will always disable RSC_ID_BIGDATA.
2. when bbklog exit, it will call echo RSC_ID_BBKLOG 0 > /sys/rsc/ktop_set to disable.
*/
static ssize_t store_rsc_cpu_usage_set(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int enable;
	long period;
	char id[RSC_MAX_ID_SIZE];
	char str[32];
	int i = 0;
	int len =  strlen(RSC_ID_PREFIX);
	int asize = ARRAY_SIZE(rsc_ids);

	mutex_lock(&cpu_usage_lock);

	ret = sscanf(buf, "%31s %u %15s %ld", id, &enable, str, &period);

	if (ret != 2 && ret != 4)
		goto fail;

	/*
	this is use for press test only.
	stop all!
	echo forcestop 1 > /sys/rsc/ktop_set

	stop forcestart only!
	echo forcestop 0 > /sys/rsc/ktop_set
	*/
	if ((ret == 2) && !strncmp(id, KTOP_FORCESTOP_STR, strlen(KTOP_FORCESTOP_STR)) && (enable == 0)) {
		rsc_info("warning! forcestop ktop! rsc_ktop_map: 0x%lx-> 0 enable: %d -> 0 period: %ldms\n",
			rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
			(long)atomic64_read(&rsc_task_cpu_usage_period));
		forcestart_work = 0;
		if (1 == enable) {
			rsc_ktop_map = 0;
			goto forcestop;
		} else if (!rsc_ktop_map)
			goto forcestop;
		else
			goto success;
	}

	/*this is use for press test only.
	   echo forcestart 1 period: 10000 > /sys/rsc/ktop_set
	*/
	if ((ret == 4) && !strncmp(id, KTOP_FORCESTART_STR, strlen(KTOP_FORCESTART_STR))) {
		if (strncmp(str, KTOP_PERIOD_SET_STR, strlen(KTOP_PERIOD_SET_STR)))
			goto fail;
		if  (period <= 0/* || (period < CPU_USAGE_MIN_PERIOD)*/) {
			rsc_err("forcestart ktop set error! period(%ld) shoule be "
				"positive num. id: %s\n", period, id);
			goto fail;
		}
		if (enable == 1) {
			rsc_info("forcestart ktop enable! id: %s rsc_ktop_map: 0x%lx enable: %d -> %d period: %ldms -> %ldms!\n",
				id, rsc_ktop_map, atomic_read(&task_cpu_usage_enable), enable,
				(long)atomic64_read(&rsc_task_cpu_usage_period), period);
			if (!atomic_read(&task_cpu_usage_enable) ||
				atomic64_read(&rsc_task_cpu_usage_period) != period) {
				atomic64_set(&rsc_task_cpu_usage_period, period);
				rsc_cpu_usage_enable();
				complete(&cpucap_notify_complete);
				/*mark data in buffer is old*/
				data_is_new = 0;
			}
			forcestart_work = 1;
			goto success;
		} else if (enable == 0) {
			forcestart_work = 0;
			if (!rsc_ktop_map)
				goto forcestop;
			else
				goto success;
		}
		goto fail;
	}

	if (strncmp(id, RSC_ID_PREFIX, len))
		goto fail;

	for (i = 0; i < asize; i++) {
		if (!strncmp(id + len, (char *)(rsc_ids[i].name) + len, RSC_MAX_ID_SIZE - len))
			break;
	}

	if (i >= asize)
		goto fail;

	if (ret == 2) {
		if (enable == 0) {

			if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id) {
				/*unlock here to prevent dead lock!*/
				mutex_unlock(&cpu_usage_lock);
				cancel_delayed_work_sync(&cpu_usage_delay_work[i].work);
				mutex_lock(&cpu_usage_lock);
			}

			clear_bit(rsc_ids[i].id - RSC_ID_START, &rsc_ktop_map);
			cpu_usage_set_time[i] = 0;
			if (!rsc_ktop_map) {
				if (forcestart_work) {
					rsc_info("forcestart_work ktop not disable! id: %s rsc_ktop_map: 0x%lx period: %ldms\n",
						id, rsc_ktop_map, (long)atomic64_read(&rsc_task_cpu_usage_period));
						goto success;
				}
forcestop:
				if (atomic_read(&task_cpu_usage_enable)) {
					/*-1 mean disable*/
					atomic64_set(&rsc_task_cpu_usage_period, -1);
					rsc_cpu_usage_disable();
					complete(&cpucap_notify_complete);
					data_is_new = 0;
					rsc_info("ktop disable! id: %s rsc_ktop_map: 0x%lx period: %ldms\n",
						id, rsc_ktop_map, (long)atomic64_read(&rsc_task_cpu_usage_period));
				} else
					rsc_info("ktop has been disable! id: %s\n", id);
			} else
				rsc_info("ktop not disable!"
					"id: %s rsc_ktop_map: 0x%lx enable: %d period: %ldms\n",
					id, rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
					(long)atomic64_read(&rsc_task_cpu_usage_period));
			goto success;
		} else if (enable == 1) {

			if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id) {
				/*unlock here to prevent dead lock!*/
				mutex_unlock(&cpu_usage_lock);
				cancel_delayed_work_sync(&cpu_usage_delay_work[i].work);
				mutex_lock(&cpu_usage_lock);
			}

			cpu_usage_set_time[i] = get_jiffies_64();
			set_bit(rsc_ids[i].id - RSC_ID_START, &rsc_ktop_map);

			if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id)
				schedule_delayed_work(&cpu_usage_delay_work[i].work, cpu_usage_delayed_work_timeout);

			if (!atomic_read(&task_cpu_usage_enable)) {
				atomic64_set(&rsc_task_cpu_usage_period, CPU_USAGE_DEF_PERIOD);
				rsc_cpu_usage_enable();
				complete(&cpucap_notify_complete);
				/*glb_task_cpu_usage_cnt = 0;*/
				/*mark data in buffer is old*/
				data_is_new = 0;
				rsc_info("ktop enable! id: %s "
					"rsc_ktop_map: 0x%lx enable: %d period: %ldms\n",
					id, rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
					(long)atomic64_read(&rsc_task_cpu_usage_period));
			} else
				rsc_info("ktop has been enable! id: %s "
					"rsc_ktop_map: 0x%lx enable: %d period: %ldms\n",
					id, rsc_ktop_map, atomic_read(&task_cpu_usage_enable),
					(long)atomic64_read(&rsc_task_cpu_usage_period));
			goto success;
		}
		goto fail;
	}

	if ((ret == 4) && (enable == 1)) {
		if (strcmp(str, KTOP_PERIOD_SET_STR))
			goto fail;

		if  (period <= 0/* || (period < CPU_USAGE_MIN_PERIOD)*/) {
			rsc_err("ktop set error! period(%ld) shoule be "
				"positive num. id: %s\n", period, id);
			goto fail;
		}

		if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id) {
			/*unlock here to prevent dead lock!*/
			mutex_unlock(&cpu_usage_lock);
			cancel_delayed_work_sync(&cpu_usage_delay_work[i].work);
			mutex_lock(&cpu_usage_lock);
		}

		cpu_usage_set_time[i] = get_jiffies_64();
		set_bit(rsc_ids[i].id - RSC_ID_START, &rsc_ktop_map);

		if (RSC_ID_BIGDATA != rsc_ids[i].id && RSC_ID_BBKLOG != rsc_ids[i].id)
			schedule_delayed_work(&cpu_usage_delay_work[i].work, cpu_usage_delayed_work_timeout);

		rsc_info("ktop enable! id: %s rsc_ktop_map: 0x%lx enable: %d -> %d period: %ldms -> %ldms!\n",
			id, rsc_ktop_map, atomic_read(&task_cpu_usage_enable), enable,
			(long)atomic64_read(&rsc_task_cpu_usage_period), period);
		atomic64_set(&rsc_task_cpu_usage_period, period);
		rsc_cpu_usage_enable();
		complete(&cpucap_notify_complete);
		/*mark data in buffer is old*/
		data_is_new = 0;
		goto success;
	}

fail:
	mutex_unlock(&cpu_usage_lock);
	rsc_err("usage: echo RSC_ID_BIGDATA(or RSC_ID_SHELL ...) 1 > /sys/rsc/ktop_set buf: %s\n", buf);
	return -EINVAL;

success:
	rsc_info("ktop %16s(%5d) uid: %5d\n", current->comm, current->pid, current_uid().val);
	mutex_unlock(&cpu_usage_lock);
	return count;
}

static ssize_t store_rsc_cpu_usage_percent(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, percent;

	mutex_lock(&cpu_usage_lock);
	ret = sscanf(buf, "%d", &percent);
	if (ret == 1) {
		if (percent >= 0 && percent <= 100) {
			rsc_info("ktop notify percent change: enable: %d period: %ldms percent: %d -> %d\n",
				atomic_read(&task_cpu_usage_enable),
				(long)atomic64_read(&rsc_task_cpu_usage_period), notify_percent, percent);
			notify_percent = percent;
			mutex_unlock(&cpu_usage_lock);
			return count;
		}
	}
	mutex_unlock(&cpu_usage_lock);
	rsc_err("usage: echo 15 > /sys/rsc/ktop_percent buf: %s\n", buf);

	return -EINVAL;
}

static ssize_t show_rsc_cpu_usage_percent(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&cpu_usage_lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", notify_percent);
	mutex_unlock(&cpu_usage_lock);

	return ret;
}

static ssize_t store_rsc_cpu_usage_foreground(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, fpid[RSC_CPU_USAGE_MAX_FPID];
	int i;

	BUILD_BUG_ON(RSC_CPU_USAGE_MAX_FPID < 16);
	mutex_lock(&cpu_usage_lock);
	ret = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
		&fpid[0], &fpid[1], &fpid[2], &fpid[3], &fpid[4], &fpid[5], &fpid[6], &fpid[7],
		&fpid[8], &fpid[9], &fpid[10], &fpid[11], &fpid[12], &fpid[13], &fpid[14], &fpid[15]
		);
	if (ret >= 1 && ret <= RSC_CPU_USAGE_MAX_FPID) {
		for (i = 0; i < ret; i++)
			if (!(fpid[i] >= -1 && fpid[i] <= PID_MAX_DEFAULT))
				goto fail;
		for (i = 0; i < ret; i++) {
			rsc_dbg(CPU_TASK_USAGE, "ktop set foreground pid cnt: %d [%d]: %d -> %d\n",
				ret, i, cpu_usage_fpid[i], fpid[i]);
			cpu_usage_fpid[i] = fpid[i];
		}
		for (i = ret; i < RSC_CPU_USAGE_MAX_FPID; i++)
			cpu_usage_fpid[i] = -1;
		cpu_usage_fpid_cnt = ret;
		mutex_unlock(&cpu_usage_lock);
		return count;
	}

fail:
	mutex_unlock(&cpu_usage_lock);
	rsc_err("usage: echo pid0 > /sys/rsc/ktop_fpid buf: %s\n", buf);

	return -EINVAL;
}

static ssize_t show_rsc_cpu_usage_foreground(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;
	int cur;

	mutex_lock(&cpu_usage_lock);
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "setcnt: %d\n", cpu_usage_fpid_cnt);
	ret += cur;
	for (i = 0; i < RSC_CPU_USAGE_MAX_FPID; i++) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%d ", cpu_usage_fpid[i]);
		ret += cur;
	}
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	ret += cur;
	mutex_unlock(&cpu_usage_lock);

	return ret;
}

#if defined(RSC_RECORD_LATENCY)
static ssize_t show_rsc_cpu_usage_latency(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur, cpu;

	cur = snprintf(buf, PAGE_SIZE, "%s\t", rsc_ktop_latency_enable?" enable":"disable");
	ret += cur;
	for_each_possible_cpu(cpu) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%d:%d\tlat\t%u:%u:%u:%u\t",
			cpu, per_cpu(task_cpu_usage_last_cnt, cpu),
			rsc_ktop_min_latency[cpu],
			rsc_ktop_cur_latency[cpu],
			rsc_ktop_max_latency[cpu],
			rsc_ktop_avg_latency[cpu]
		);
		ret += cur;
	}
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	ret += cur;

	return ret;
}

static ssize_t store_rsc_cpu_usage_latency(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, enable;

	ret = sscanf(buf, "%d", &enable);
	if (ret == 1) {
		mutex_lock(&cpu_usage_lock);
		rsc_enable_record_latency(enable);
		mutex_unlock(&cpu_usage_lock);
		rsc_info("echo %d > /sys/rsc/ktop_latency buf: %s %16s(%5d) uid: %5d\n",
			enable, buf, current->comm, current->pid, current_uid().val);
	}
	return count;
}
#endif

static struct kobj_attribute rsc_cpu_usage_attr =
__ATTR(CPU_USAGE_NAME, 0440, show_rsc_cpu_usage, NULL/*store_rsc_cpu_usage*/);

static struct kobj_attribute rsc_cpu_usage_set_attr =
__ATTR(CPU_USAGE_SET, 0660, show_rsc_cpu_usage_set, store_rsc_cpu_usage_set);

static struct kobj_attribute rsc_cpu_usage_percent_attr =
__ATTR(CPU_USAGE_PERCENT, 0660, show_rsc_cpu_usage_percent, store_rsc_cpu_usage_percent);

static struct kobj_attribute rsc_cpu_usage_foreground_attr =
__ATTR(CPU_USAGE_FPID, 0660, show_rsc_cpu_usage_foreground, store_rsc_cpu_usage_foreground);

#if defined(RSC_RECORD_LATENCY)
static struct kobj_attribute rsc_cpu_usage_latency_attr =
__ATTR(CPU_USAGE_LATENCY, 0660, show_rsc_cpu_usage_latency, store_rsc_cpu_usage_latency);
#endif

#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
#ifdef CONFIG_RSC_MEM_DEFRAG
/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
extern int rsc_highatomic_reserve_pages;
#endif
static ssize_t show_rsc_cpu_up_fail(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur, i;
	struct rtc_time failtime;
	int scpu = 0;
	u32 dwelaps;
	struct cpumask offline;

	if (rsc_cpu_up_fail_suspend)
		rtc_time_to_tm((unsigned long)rsc_cpu_up_fail_stime.tv_sec-sys_tz.tz_minuteswest * 60, &failtime);
	else
		memset(&failtime, 0, sizeof(failtime));

	cur = snprintf(buf, PAGE_SIZE,
		"cpuupfail: %d %d %d %d"
		"\tappfix: %d %*pbl"
		"\tnotmat: %d %d"
		"\tfailtime: %u %u"
		"\tcpuupfix: %d %d"
		"\tkiller: %s %d(%s %d)"
		"\telaps: %u %u"
		"\t%04d-%02d-%02d %02d:%02d:%02d\npercpufailtime"
		, rsc_cpu_up_fail_suspend, rsc_cpu_up_fail_other, rsc_cpu_up_fail_policy, rsc_fix_cpu_hotplug_disable
		, (!cpumask_empty(&rsc_cpu_up_fail_cpus_app_nofix))?rsc_cpu_up_app_nofix_cnt+1:rsc_cpu_up_app_nofix_cnt
		, cpumask_pr_args(&rsc_cpu_up_fail_cpus_app_nofix)
		, rsc_cpu_up_state_notmatch_cnt, rsc_cpu_up_state_shell_notmatch_cnt
		, rsc_cpu_down_whole_elaps, rsc_cpu_down_whole_max_elaps
		, rsc_cpu_up_act_fail_suspend
		, rsc_cpu_up_fail_shell_cnt
		, rsc_cpu_cpuup_fail_info.down_comm
		, rsc_cpu_cpuup_fail_info.down_pid
		, rsc_cpu_cpuup_fail_info.down_pcomm
		, rsc_cpu_cpuup_fail_info.down_ppid
		, rsc_cpu_up_fail_elaps
		, rsc_cpu_up_fail_max_elaps
		, rsc_cpu_up_fail_suspend?failtime.tm_year+1900:0, rsc_cpu_up_fail_suspend?failtime.tm_mon+1:0, failtime.tm_mday
		, failtime.tm_hour, failtime.tm_min, failtime.tm_sec
	);
	ret += cur;

	for (i = 0; i < NR_CPUS; i++) {
		cur = snprintf(buf+ret, PAGE_SIZE, " cpu%d: %u %u\t",
			i, rsc_cpu_down_elaps[i], rsc_cpu_down_max_elaps[i]
		);
		ret += cur;
	}

#ifdef CONFIG_ARCH_SDM439
	for (i = 0; i < NR_CPUS; i++)
		if (rsc_cpu_hp_info[i].down_time) {
			scpu = i;
			break;
		}
#else
	for (i = NR_CPUS-1; i >= 0; i--)
		if (rsc_cpu_hp_info[i].down_time) {
			scpu = i;
			break;
		}
#endif

	cpumask_andnot(&offline, cpu_possible_mask, cpu_online_mask);
	if (rsc_cpu_hp_info[scpu].up_time >= rsc_cpu_hp_info[scpu].down_time)
		dwelaps = rsc_cpu_hp_info[scpu].up_time - rsc_cpu_hp_info[scpu].down_time;
	else {
		if (!cpumask_empty(&offline))
			dwelaps = 93949394;
		else
			dwelaps = 0;
	}

	rtc_time_to_tm((unsigned long)rsc_cpu_hp_info[scpu].down_time-sys_tz.tz_minuteswest * 60, &failtime);
	cur = snprintf(buf + ret, PAGE_SIZE - ret,
		"\ncpuhp: %d %s %d(%s %d)"
		"\tdwelaps: %u"
		"\t%04d-%02d-%02d %02d:%02d:%02d offline: %*pbl"
#ifdef CONFIG_RSC_MEM_DEFRAG
/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
		"\tatomicpages: %d"
#endif
		"\n"
		, scpu
		, rsc_cpu_hp_info[scpu].down_comm
		, rsc_cpu_hp_info[scpu].down_pid
		, rsc_cpu_hp_info[scpu].down_pcomm
		, rsc_cpu_hp_info[scpu].down_ppid
		, dwelaps
		, failtime.tm_year+1900, failtime.tm_mon+1, failtime.tm_mday
		, failtime.tm_hour, failtime.tm_min, failtime.tm_sec
		, cpumask_pr_args(&offline)
#ifdef CONFIG_RSC_MEM_DEFRAG
/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
		, rsc_highatomic_reserve_pages
#endif
	);
	ret += cur;
/*
	cur = snprintf(buf+ret, PAGE_SIZE, "\n");
	ret += cur;
*/
	return ret;
}

static ssize_t store_rsc_cpu_up_fail(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, val;

	ret = sscanf(buf, "%u", &val);
	if (ret == 1) {
		rsc_info("rsc_fix_cpu_hotplug_disable from %d -> %d\n",
			rsc_fix_cpu_hotplug_disable, val
		);
		rsc_fix_cpu_hotplug_disable = val;
		return count;
	}

	return -EINVAL;
}

static struct kobj_attribute rsc_cpu_up_fail_node =
__ATTR(cpuupfail, 0660, show_rsc_cpu_up_fail, store_rsc_cpu_up_fail);
#endif

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
struct task_struct *rsc_get_hok_render_thread(struct task_struct *p)
{
	struct rsc_task_cpu_usage_t *data;
	typeof(glb_task_cpu_usage_tbl[0]) *tbl;
	int i, j, len;
	int pid = -1;

	mutex_lock(&cpu_usage_lock);
	data = glb_task_cpu_usage;
	tbl = glb_task_cpu_usage_tbl;
	len = min(glb_task_cpu_usage_cnt, (u32)5);

	for (i = 0; i < len; i++) {
		j = tbl[i];
		if (task_uid(p).val == data[j].uid &&
			strncmp("UnityMain", data[j].comm, TASK_COMM_LEN)) {
			if (!strncmp(data[j].comm, "Thread-", 7)) {
				pid = data[j].pid;
				break;
			}
		}
	}
	mutex_unlock(&cpu_usage_lock);
	if (pid > 0) {
		p = find_task_by_vpid(pid);
		rsc_info("set hok_render uid %5d %16s %5d!\n",
			task_uid(p).val, p->comm, p->pid);
	} else
		p = NULL;

	return p;
}
#endif

static int __init rsc_cpu_usage_init(void)
{
	long ret = 0;
	int i;

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_usage_attr.attr);
	if (ret)
		return ret;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpu_usage_attr.attr);

	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_usage_set_attr.attr);
	if (ret)
		goto fail_create_cpu_usage_set;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpu_usage_set_attr.attr);

	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_usage_percent_attr.attr);
	if (ret)
		goto fail_create_cpu_usage_percent;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpu_usage_percent_attr.attr);

	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_usage_foreground_attr.attr);
	if (ret)
		goto fail_create_cpu_usage_foreground;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpu_usage_foreground_attr.attr);

#if defined(RSC_RECORD_LATENCY)
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_usage_latency_attr.attr);
	if (ret)
		goto fail_create_cpu_usage_latency;
	rsc_chown_to_system(rsc_root_dir, &rsc_cpu_usage_latency_attr.attr);
#endif

#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
	ret = sysfs_create_file(rsc_root_dir, &rsc_cpu_up_fail_node.attr);
	if (!ret)
		rsc_chown_to_system(rsc_root_dir, &rsc_cpu_up_fail_node.attr);
#endif

	for (i = 0; i < (RSC_ID_END - RSC_ID_START); i++) {
		cpu_usage_delay_work[i].id = RSC_ID_START + i;
		INIT_DELAYED_WORK(&cpu_usage_delay_work[i].work, cpu_usage_work_handler);
	}

	return ret;

#if defined(RSC_RECORD_LATENCY)
fail_create_cpu_usage_latency:
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_foreground_attr.attr);
#endif

fail_create_cpu_usage_foreground:
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_percent_attr.attr);

fail_create_cpu_usage_percent:
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_set_attr.attr);

fail_create_cpu_usage_set:
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_attr.attr);

	return ret;
}

static void __exit rsc_cpu_usage_exit(void)
{
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_attr.attr);
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_set_attr.attr);
	sysfs_remove_file(rsc_root_dir, &rsc_cpu_usage_percent_attr.attr);
}

module_init(rsc_cpu_usage_init);
module_exit(rsc_cpu_usage_exit);
