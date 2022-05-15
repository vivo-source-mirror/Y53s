#include <linux/mm.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/vmstat.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/math64.h>
#include <linux/writeback.h>
#include <linux/compaction.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/vivo_rsc/rsc_mem_mon.h>
#include <linux/cred.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <linux/oom.h>

#ifdef CONFIG_RSC_EVENT_MMONITOR
#include <xxx/mmonitor.h>

DEFINE_PER_CPU(struct mmonitor_event_state, mmonitor_event_states) = { {0} };
EXPORT_PER_CPU_SYMBOL(mmonitor_event_states);
#endif

/*
/sys/module/rsc_mem_mon/parameters/enable
disable:
echo 0 > /sys/module/rsc_mem_mon/parameters/enable
*/
int __read_mostly rsc_mem_mon_enable = 1;
module_param_named(enable, rsc_mem_mon_enable, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

/*
/sys/module/rsc_mem_mon/parameters/slowpath_trace
disable:
echo 0 > /sys/module/rsc_mem_mon/parameters/slowpath_trace
printk outout when buffer full:
echo 2 > /sys/module/rsc_mem_mon/parameters/slowpath_trace
see #define rsc_mem_printf(timeout, fmt, args...)
*/
int __read_mostly rsc_mem_slowpath_trace = 1;
module_param_named(slowpath_trace, rsc_mem_slowpath_trace, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

#ifdef CONFIG_RSC_MEM_PRIORITY
u32 __read_mostly rsc_mem_slowpath_wait = 1;
u32 __read_mostly rsc_mem_pri_enable = 1;

#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
/*
TD1904_ERD:/sys/devices/system/cpu/cpu1 # cat  /sys/devices/system/cpu/cpu1/cpufreq/cpuinfo_max_freq
1638000
TD1904_ERD:/sys/devices/system/cpu/cpu1 # cat /sys/devices/system/cpu/cpu1/cpu_capacity
363
TD1904_ERD:/sys/devices/system/cpu/cpu1 # cat /sys/devices/system/cpu/cpu7/cpufreq/cpuinfo_max_freq
2210000
TD1904_ERD:/sys/devices/system/cpu/cpu1 # cat /sys/devices/system/cpu/cpu7/cpu_capacity
1024
4+4 samsung 9609, bigcore cpucap: 19.8%, littlecore cpucap: 5.2%
bigcore cap is 4 times as littlecore.
so, we could not let too many backgroup thread into slowpath or
system crash in getting spin_lock too long time or hung task occurs.
*/
/*qcom and mtk is 32, samsung is 8*/
u32 __read_mostly rsc_mem_pri_slowpath_max_cnt = 32;

/*background task warter mark ALLOC_WMARK_LOW level try count*/
u16 __read_mostly rsc_mem_pri_slowpath_bg_try_cnt = 8;

/*default task warter mark ALLOC_WMARK_RBACK level try count*/
u16 __read_mostly rsc_mem_pri_slowpath_def_try_cnt = 4;
#endif
#endif

/*
disable:
echo 0 > /sys/module/rsc_mem_mon/parameters/mem_speed_interval
enable and set interval to 1000ms:
echo 1000 > /sys/module/rsc_mem_mon/parameters/mem_speed_interval
*/
int __read_mostly rsc_mem_speed_interval/*= 1000*/;
module_param_named(mem_speed_interval, rsc_mem_speed_interval,
	int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
#if 0
int rsc_mem_logmask = MEM_REC_SLOWPATH|MEM_REC_WAIT;
#else
int __read_mostly rsc_mem_logmask = MEM_REC_SLOWPATH;
#endif
/*
echo 7 > /sys/module/rsc_mem_mon/parameters/logmask
*/
module_param_named(logmask, rsc_mem_logmask, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

/*printk slowpath time 16ms*/
#define RSC_MEM_PRINT_TIME_US (16*1000)
int rsc_slowpath_print = RSC_MEM_PRINT_TIME_US;
/*
echo 50000 > /sys/module/rsc_mem_mon/parameters/slowpath_print
*/
module_param_named(slowpath_print, rsc_slowpath_print, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

/*printk slowpath detail time 200ms*/
#define RSC_MEM_DETAIL_TIME_US (200*1000)
int rsc_slowpath_detail = RSC_MEM_DETAIL_TIME_US;
/*
echo 50000 > /sys/module/rsc_mem_mon/parameters/slowpath_detail
*/
module_param_named(slowpath_detail, rsc_slowpath_detail, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

int rsc_slowpath_group = RSC_CGROUP_END;
/*
echo 4 > /sys/module/rsc_mem_mon/parameters/slowpath_group
*/
module_param_named(slowpath_group, rsc_slowpath_group, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

static long memlog_print;/*us*/
/*
echo 10000 > /sys/module/rsc_mem_mon/parameters/memlog_print
*/
module_param_named(memlog_print, memlog_print, long, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

/* RSC_MEM_PRI_SLOWPATH_STRATEGY, change from 1ms to 2ms*/
int __read_mostly mem_record_ns = 2*1000*1000;/*ms*/

/*
echo 0 > /sys/module/rsc_mem_mon/parameters/mem_record_ms
*/
module_param_named(mem_record_ns, mem_record_ns, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

#if defined(CONFIG_RSC_MEM_MON) && defined(CONFIG_RSC_MON_CPU_HOTPLUG)
int rsc_cpu_up_fail_suspend;
module_param_named(cpu_up_fail_suspend, rsc_cpu_up_fail_suspend, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
int rsc_cpu_up_fail_other;
module_param_named(cpu_up_fail_other, rsc_cpu_up_fail_other, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
int rsc_cpu_up_fail_policy;
module_param_named(cpu_up_fail_policy, rsc_cpu_up_fail_policy, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);

#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
int rsc_cpu_up_act_fail_suspend;
module_param_named(cpu_up_act_fail_suspend, rsc_cpu_up_act_fail_suspend, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
#endif
#endif

#if 0
/*
/sys/module/rsc_mem_mon/parameters/cgroup_debug
add log:
echo 1 > /sys/module/rsc_mem_mon/parameters/rsc_cgroup_debug
or
echo 3 > /sys/module/rsc_mem_mon/parameters/rsc_cgroup_debug

*/
/*
see
#define RSC_CGROUP_DEBUG		1
#define RSC_CGROUP_PERMIT_PEM_RA_WRITE_SUBTHREAD		2

0: forbit pem_ra write sub thread cgroup of theadgroup,
if xxxpid is a thread pid, not thread group pid
pem_ra do echo xxxpid > /dev/vivo_rsc/bg/cgroup.procs will return error -40(ELOOP)
07-05 07:36:39.145  2577  2834 V PRA     : setPolicy com.google.android.webview:sandboxed_process0, pid = 32470, pri = 0
07-05 07:36:39.145  2577  2834 E PRA     : setPolicy error:
07-05 07:36:39.145  2577  2834 E PRA     : java.io.IOException: write failed: ELOOP (Too many symbolic links encountered)
07-05 07:36:39.145  2577  2834 E PRA     : 	at libcore.io.IoBridge.write(IoBridge.java:558)
1: add debug log
2: permit pem_ra write sub thread cgroup of theadgroup,
*/
int __read_mostly rsc_cgroup_debug;
module_param_named(rsc_cgroup_debug, rsc_cgroup_debug, int, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
#endif

#define FIRST_APP_UID KUIDT_INIT(10000)
#define LAST_APP_UID  KUIDT_INIT(19999)

#define VISIBLE_APP_ADJ    (100)
#define VISIBLE_APP_ADJ1    (101)
#define FOREGROUND_APP_ADJ (0)
#define RSC_HIGHPRIO_ADJ    (300)
#define RSC_PAGEALLOC_STACK_ENRTYS 20

atomic_long_t rsc_slowpath_count[RSC_ORDER_LIMIT] = {ATOMIC_LONG_INIT(0)};
u64 rsc_slowpath_time[RSC_ORDER_LIMIT];

atomic_long_t rsc_pgalloc_count[MAX_ORDER+1] = {ATOMIC_LONG_INIT(0)};

atomic_long_t rsc_dual_app_cnt;
struct rsc_alloc_stat_t rsc_dual_app_time[RSC_APP_MAX] = {
	[0 ... RSC_APP_MAX-1] = {
		.isapp = 1,
		.tcnt = &rsc_dual_app_cnt,
	}
};

atomic_long_t rsc_app_cnt;
struct rsc_alloc_stat_t rsc_app_time[RSC_APP_MAX] = {
	[0 ... RSC_APP_MAX-1] = {
		.isapp = 1,
		.tcnt = &rsc_app_cnt,
	}
};

atomic_long_t rsc_9990_system_cnt;
struct rsc_alloc_stat_t rsc_9990_system_time[RSC_9990_SYSTEM_MAX] = {
	[0 ... RSC_9990_SYSTEM_MAX-1] = {
		.tcnt = &rsc_9990_system_cnt,
	}
};

atomic_long_t rsc_3000_system_cnt;
struct rsc_alloc_stat_t rsc_3000_system_time[RSC_3000_SYSTEM_MAX] = {
	[0 ... RSC_3000_SYSTEM_MAX-1] = {
		.tcnt = &rsc_3000_system_cnt,
	}
};

atomic_long_t rsc_2000_system_cnt;
struct rsc_alloc_stat_t rsc_2000_system_time[RSC_2000_SYSTEM_MAX] = {
	[0 ... RSC_2000_SYSTEM_MAX-1] = {
		.tcnt = &rsc_2000_system_cnt,
	}
};

atomic_long_t rsc_1000_system_cnt;
struct rsc_alloc_stat_t rsc_1000_system_time[RSC_1000_SYSTEM_MAX] = {
	[0 ... RSC_1000_SYSTEM_MAX-1] = {
		.tcnt = &rsc_1000_system_cnt,
	},
	[RSC_AID_CAMERASERVER-RSC_1000_SYSTEM_START] = {
		.tcnt = &rsc_1000_system_cnt,
		.isapp = 1,
	}
};

atomic_long_t rsc_native_cnt;
struct rsc_alloc_stat_t rsc_native_time = {
	.tcnt = &rsc_native_cnt,
};

atomic_long_t rsc_other_app_cnt;
struct rsc_alloc_stat_t rsc_other_app_time = {
	.tcnt = &rsc_other_app_cnt,
};

atomic_long_t rsc_system_server_cnt;
struct rsc_alloc_stat_t rsc_system_server_time = {
	.tcnt = &rsc_system_server_cnt,
};

struct rsc_alloc_print_t {
	struct rsc_alloc_stat_t *alloc;
	atomic_long_t *pcnt;
	int size;
};

struct rsc_alloc_print_t rsc_alloc_array[] = {
	{rsc_dual_app_time, &rsc_dual_app_cnt, ARRAY_SIZE(rsc_dual_app_time)},
	{rsc_app_time, &rsc_app_cnt, ARRAY_SIZE(rsc_app_time)},
	{&rsc_system_server_time, &rsc_system_server_cnt, 1},
	{rsc_9990_system_time, &rsc_9990_system_cnt, ARRAY_SIZE(rsc_9990_system_time)},
	{rsc_3000_system_time, &rsc_3000_system_cnt, ARRAY_SIZE(rsc_3000_system_time)},
	{rsc_2000_system_time, &rsc_2000_system_cnt, ARRAY_SIZE(rsc_2000_system_time)},
	{rsc_1000_system_time, &rsc_1000_system_cnt, ARRAY_SIZE(rsc_1000_system_time)},
	{&rsc_native_time, &rsc_native_cnt, 1},
	{&rsc_other_app_time, &rsc_other_app_cnt, 1},
};

DEFINE_SPINLOCK(rsc_slowpath_spinlock);
DEFINE_SPINLOCK(rsc_seq_spinlock);

/*
save as PREFIX_MAX and  LOG_LINE_MAX
in pintk.c
dmesg cannot get the full buffer, but cat /dev/kmsg full buffer,
for compatibility, we set RSC_PREFIX_MAX from 64 to  33*7
log example as follow:
[   42.511100]@3[02-24 14:24:22]|RSC  xxx
*/
#define RSC_PREFIX_MAX		(32*10)
#define RSC_LOG_LINE_MAX		(1024 - RSC_PREFIX_MAX)
struct rsc_seq_file_t rsc_seq_file[RSC_MAX_THREADS_IN_SLOWPATH];
/*struct rsc_seq_file_t rsc_kswapd_seq_file;*/
#if 0
static int rsc_seq_file_idx;
#endif
int rsc_seq_alloc_num;

int rsc_max_seq_alloc;
int rsc_buf_alloc_num;

/*unsigned long rsc_seq_alloc_bit;*/

#define RSC_MEMBUF_NONE	-1
#define RSC_MEMBUF_FULL	-2

enum {
	MEM_MON,
	MEM_FG,
	MEM_LASTFG,
};

/*seconds, after (current time - mem_lastfg_backtime) will print*/
u32 mem_lastfg_backtime = 10;

LIST_HEAD(rsc_mem_head);

static inline void mem_seq_set_overflow(struct rsc_seq_file_t *m)
{
	m->count = RSC_MAX_LOG_BUF_SIZE;
#ifdef RSC_MEM_RING_BUFF_ENABLE
	memset(m->buf+m->point, 0, RSC_MAX_LOG_BUF_SIZE - m->point);
#endif
	/*printk("@RSC warning %s %d buf full!\n", current->comm, current->pid);*/
	/*dump_stack();*/
}

#ifdef RSC_MEM_RING_BUFF_ENABLE
static inline int mem_vprintf(struct rsc_seq_file_t *m, const char *f, va_list args)
{
	int len;

	len = vsnprintf(m->buf + m->point, RSC_MAX_LOG_BUF_SIZE - m->point, f, args);
	if (m->point + len < RSC_MAX_LOG_BUF_SIZE) {
		m->point += len;
		if (m->point > m->count)
			m->count = m->point;
		else if (m->point < (RSC_MAX_LOG_BUF_SIZE - 1))
			m->buf[m->point] = '@';
		return 0;
	}
	mem_seq_set_overflow(m);
	m->point = 0;
	len = vsnprintf(m->buf + m->point, RSC_MAX_LOG_BUF_SIZE - m->point, f, args);
	if (m->point + len < RSC_MAX_LOG_BUF_SIZE) {
		m->point += len;
		if (m->point > m->count)
			m->count = m->point;
		return 0;
	}

	return RSC_MEMBUF_FULL;
}
#else
static inline int mem_vprintf(struct rsc_seq_file_t *m, const char *f, va_list args)
{
	int len;

	if (m->count < RSC_MAX_LOG_BUF_SIZE) {
		len = vsnprintf(m->buf + m->count, RSC_MAX_LOG_BUF_SIZE - m->count, f, args);
		if (m->count + len < RSC_MAX_LOG_BUF_SIZE) {
			m->count += len;
			return 0;
		}
	}
	mem_seq_set_overflow(m);
	return RSC_MEMBUF_FULL;
}
#endif

int rsc_mem_printf_inter(const char *f, ...)
{
	struct rsc_seq_file_t *m;
	va_list args;
	int ret;

	m = (struct rsc_seq_file_t *)current_thread_info()->rsc_seq;
	va_start(args, f);
	ret = mem_vprintf(m, f, args);
	va_end(args);

	return ret;
}

#if 0
static inline int rsc_is_background(void)
{
	int adj;
/*
	fix me! need! get lock? I think it is noneed!
	unsigned long flags;
	if (lock_task_sighand(task, &flags)) {
		oom_score_adj = task->signal->oom_score_adj;
		unlock_task_sighand(task, &flags);
	}
*/
	adj = current->signal->oom_score_adj;
#if 1
	return (adj < RSC_HIGHPRIO_ADJ) ? 0 : 1;
#else
	kuid_t uid;
	int adj;
	struct mm_struct *mm;

	mm = current->mm;
	if (!mm)
		return 0;
	uid = current_uid();
	if (!mm || uid_lt(uid, FIRST_APP_UID) || uid_gt(uid, LAST_APP_UID))
		return 1;
	if (adj != VISIBLE_APP_ADJ && adj != FOREGROUND_APP_ADJ && adj != VISIBLE_APP_ADJ1)
		return 1;
#endif
	return 0;
}
#endif

#define show_slowpath(atime, i, count, idx, tm, mode)																		\
			do {																											\
				if (atime[i].time) {																						\
					t_cnt += atomic_read(&atime[i].cnt);																	\
					f_cnt += atime[i].slowcnt[RSC_CGROUP_TOPAPP]; 															\
					r_cnt += atime[i].slowcnt[RSC_CGROUP_RBG]; 																\
					r_cnt += atime[i].slowcnt[RSC_CGROUP_DEFAULT]; 															\
					b_cnt += atime[i].slowcnt[RSC_CGROUP_SYS_BACKGROUND]; 													\
					total_time += atime[i].time;																			\
					fg_total_time += atime[i].fg_time;																		\
					if ((MEM_MON == mode) ||																				\
						(atime[i].slowcnt[RSC_CGROUP_TOPAPP] && ((MEM_FG == mode) ||										\
						((MEM_LASTFG == mode) && (atime[i].last_tv_sec +  mem_lastfg_backtime) >= dtime.tv_sec)))) {		\
						rtc_time_to_tm((unsigned long)atime[i].tv_sec-sys_tz.tz_minuteswest * 60, &tm);						\
						rtc_time_to_tm((unsigned long)atime[i].max_tv_sec-sys_tz.tz_minuteswest * 60, &m_tm);				\
						rtc_time_to_tm((unsigned long)atime[i].last_tv_sec-sys_tz.tz_minuteswest * 60, &l_tm);				\
						seq_printf(s, "%3d\t%5d\t%16s\t%5d\t%16s\t%5d\t%5d\t%5d\t%4u\t%4u\t%4u\t%4u\t%4u\t%6u\t%10u\t"		\
										"%8u\t%8u\t%8u\t%u\t%3d\t%04d-%02d-%02d %02d:%02d:%02d\t"							\
										"%04d-%02d-%02d %02d:%02d:%02d\t%02d:%02d:%02d\t%5u\t%8u\t%5u\t%5u\n",				\
							idx, atime[i].inf.uid,																			\
							(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.comm:atime[i].inf.tgcomm,					\
							(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.pid:atime[i].inf.tgid,						\
							(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.tgcomm:atime[i].inf.comm,					\
							(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.tgid:atime[i].inf.pid,						\
							atime[i].inf.ppid, (s16)atime[i].adj, (u32)atomic_read(&atime[i].sorder), 						\
							(u32)atomic_read(&atime[i].cnt), atime[i].slowcnt[RSC_CGROUP_TOPAPP],							\
							atime[i].slowcnt[RSC_CGROUP_RBG]+atime[i].slowcnt[RSC_CGROUP_DEFAULT],							\
							atime[i].slowcnt[RSC_CGROUP_SYS_BACKGROUND],													\
							atime[i].time/max((u32)1, (u32)atomic_read(&atime[i].cnt)),										\
							atime[i].time, atime[i].fg_time, atime[i].fg_maxtime, atime[i].maxtime, (int)atime[i].morder, 	\
							atime[i].group, tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, 	\
							m_tm.tm_year+1900, m_tm.tm_mon+1, m_tm.tm_mday, m_tm.tm_hour, m_tm.tm_min, m_tm.tm_sec,			\
							l_tm.tm_hour, l_tm.tm_min, l_tm.tm_sec, (u32)dtime.tv_sec - atime[i].record_tv_sec,				\
							atime[i].record_fg_time, atime[i].record_cnt, atime[i].record_page								\
						 ); 																								\
						idx++;																								\
					}																										\
				}																											\
			} while (0)

/*20 spaces are for reserved, 1 for \n*/
#define SLOWPATH_HEAD_RESERVE	(20 + 1)
#define SLOWPATH_HEAD_FORMAT 	"slowpath overview:\ttotal\t%8ld\tfcnt\t%6ld\t%3ld\t"			\
		"rcnt\t%6ld\t%3ld\tbcnt\t%8ld\t%3ld\tfgttime\t%9lu\tttime\t%9lu ms\tlastfg\t%u"			\
		"\tLastTime\t%04d-%02d-%02d %02d:%02d:%02d                    \n"
#define SLOWPATH_ACTHEAD_FORMAT "slowpath overview:\ttotal\t%8ld\tfcnt\t%6ld\t%3ld\t"			\
		"rcnt\t%6ld\t%3ld\tbcnt\t%8ld\t%3ld\tfgttime\t%9lu\tttime\t%9lu ms\tlastfg\t%u"			\
		"\tLastTime\t%04d-%02d-%02d %02d:%02d:%02d"
static void print_slowpath(struct seq_file *s, int mode)
{
	int j, i, idx = 0;
	struct rsc_alloc_stat_t *atime;
	long count;
	unsigned long t_cnt = 0, f_cnt = 0, r_cnt = 0, b_cnt = 0;
	struct rtc_time tm;
	struct rtc_time m_tm;
	struct rtc_time l_tm;
	int hstart, hend;
	unsigned long total_time = 0;
	unsigned long fg_total_time = 0;
	int ret;
	struct timespec64 dtime;

	BUILD_BUG_ON(sizeof(SLOWPATH_HEAD_FORMAT) != (sizeof(SLOWPATH_ACTHEAD_FORMAT) + SLOWPATH_HEAD_RESERVE));
	getnstimeofday64_nolock(&dtime);
	hstart = s->count;
	seq_printf(s, SLOWPATH_HEAD_FORMAT,
		0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0,
		0, 0, 0, 0, 0, 0);
	hend = s->count;
	seq_printf(s, "IDX\t  UID\t          TGTASK\t TGID\t   ThreadorPTASK\t  PID\t PPID\t"
		"  ADJ\tSORD\tCOUNT\tFCNT\tRBCT\tBCNT\t ATIME\t     TTIME\t  FG_TTIME\tFG_MTIME\t   MTIME\tMO\tGrp\t"
		" Time of Occurrence\tMax Time Occurrence\tLastTime\tNsecs\tLastCost\tCout\tPages\n");

	for (j = 0; j < ARRAY_SIZE(rsc_alloc_array); j++) {
		if (atomic_long_read(rsc_alloc_array[j].pcnt)) {
			atime = rsc_alloc_array[j].alloc;
			/*idx = 0;*/
			for (i = 0; i < rsc_alloc_array[j].size; i++)
				show_slowpath(atime, i, count, idx, tm, mode);
		}
	}

	count = atomic_long_read(&rsc_other_app_cnt);
	if ((mode == MEM_MON) && count) {
		atime = &rsc_other_app_time;
		seq_printf(s, "Warning!!!The Last one app not record!\n");
	}

	rtc_time_to_tm((unsigned long)dtime.tv_sec-sys_tz.tz_minuteswest * 60, &l_tm);
	ret = snprintf(s->buf + hstart, s->size - hstart, SLOWPATH_ACTHEAD_FORMAT,
		t_cnt, f_cnt, f_cnt*100/t_cnt, r_cnt, r_cnt*100/t_cnt,
		b_cnt, b_cnt*100/t_cnt, fg_total_time/1000, total_time/1000, mem_lastfg_backtime,
		l_tm.tm_year+1900, l_tm.tm_mon+1, l_tm.tm_mday, l_tm.tm_hour, l_tm.tm_min, l_tm.tm_sec);
	if (ret && (ret <= (hend - hstart)))
		if ((hstart+ret) < s->size)
			s->buf[hstart+ret] = ' ';

#if 0
	count = atomic_long_read(&rsc_app_cnt);
	if (count) {
		atime = rsc_app_time;
		/*idx = 0;*/
		for (i = 0; i < RSC_APP_MAX; i++)
			show_slowpath(atime, i, count, idx);
	}
#endif
}

void rsc_print_mem_buf(struct rsc_seq_file_t *seq, int group)
{
	int i, j, s, c;
	char tmp;

	if (seq && seq->count > 0) {
		if (seq->count < RSC_LOG_LINE_MAX) {
			seq->buf[min(RSC_MAX_LOG_BUF_SIZE-1, (int)seq->count)] = '\0';
			printk("%s", seq->buf);
		} else {
			if (seq->count >= RSC_MAX_LOG_BUF_SIZE)
				seq->count = RSC_MAX_LOG_BUF_SIZE - 1;
			i = 0;
			s = 0;
			for (;;) {
				c = 0;
				j = -1;
				while (c < RSC_LOG_LINE_MAX && i < seq->count) {
					if (seq->buf[i] == '\n')
						j = i;
					i++;
					c++;
				}

				if (i >= seq->count)
					j = seq->count-1;
				else if (j == -1)
					j = i;
				tmp = seq->buf[j + 1];
				seq->buf[j + 1] = '\0';
				if (group != RSC_GROUP_FG)
					printk("%s", &seq->buf[s]);
				else
					printk_deferred("%s", &seq->buf[s]);
				/*printk_emit(1, LOGLEVEL_DEFAULT, "RSC BBBB", strlen("RSC BBBB"), "TTTT %s", &seq->buf[s]);*/
#if 0
				printk("RSC DBG %d->%d seq->buf %s RSC eeee\n", s, j, &seq->buf[((j-s) > 100)?(j-100):s]);
#endif
				seq->buf[j + 1] = tmp;
				if (i >= seq->count)
					break;
				i = j+1;
				s = j+1;
			}
		}

	}
}

#define show_mem_speed(atime, i, idx, tm)																	\
do {																										\
		rtc_time_to_tm((unsigned long)atime[i].speed_tv_sec-sys_tz.tz_minuteswest * 60, &tm);				\
		seq_printf(s, "%3d\t%5d\t%16s\t%5d\t%16s\t%5d\t%5d\t%7u\t"											\
						"%3d\t%04d-%02d-%02d %02d:%02d:%02d\n",												\
			idx, atime[i].inf.uid,																				\
			(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.comm:atime[i].inf.tgcomm,					\
			(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.pid:atime[i].inf.tgid,						\
			(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.tgcomm:atime[i].inf.comm,					\
			(atime[i].inf.tgid == atime[i].inf.pid)?atime[i].inf.tgid:atime[i].inf.pid,						\
			atime[i].inf.ppid, atime[i].speed_max, atime[i].speed_group,									\
			tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec 						\
		 ); 																								\
		idx++;																								\
} while (0)

static void print_mem_speed(struct seq_file *s)
{
	int j, i, idx = 0;
	struct rsc_alloc_stat_t *atime;
	struct rtc_time tm;

	seq_printf(s, "\nIDX\t  UID\t          TGTASK\t TGID\t   ThreadorPTASK\t  PID\t PPID\t"
		"   KB/s\tGrp\t Time of Occurrence\n");

	for (j = 0; j < ARRAY_SIZE(rsc_alloc_array); j++) {
		/*idx = 0;*/
		for (i = 0; i < rsc_alloc_array[j].size; i++) {
			atime = rsc_alloc_array[j].alloc;
			if (!atime[i].tcnt)
				seq_printf(s, "RSC ERR ![%d][%d] tcnt is NULL\n", j, i);
			if (atime[i].speed_enable)
				show_mem_speed(atime, i, idx, tm);
		}
	}
}

#ifdef CONFIG_RSC_MEM_PRIORITY
atomic_t rsc_cgroup_cnt[RSC_CGROUP_END+1];

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
wait_queue_head_t rsc_group_wait[RSC_CGROUP_END] = {
	[RSC_CGROUP_TOPAPP] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_TOPAPP].spinlock),
		.head = {&rsc_group_wait[RSC_CGROUP_TOPAPP].head, &rsc_group_wait[RSC_CGROUP_TOPAPP].head},
	},
	[RSC_CGROUP_RBG] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_RBG].spinlock),
		.head = {&rsc_group_wait[RSC_CGROUP_RBG].head, &rsc_group_wait[RSC_CGROUP_RBG].head},
	},
	[RSC_CGROUP_DEFAULT] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_DEFAULT].spinlock),
		.head = {&rsc_group_wait[RSC_CGROUP_DEFAULT].head, &rsc_group_wait[RSC_CGROUP_DEFAULT].head},
	},
	[RSC_CGROUP_SYS_BACKGROUND] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].spinlock),
		.head = {&rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].head, &rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].head},
	},
	[RSC_CGROUP_BACKGROUND] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_BACKGROUND].spinlock),
		.head = {&rsc_group_wait[RSC_CGROUP_BACKGROUND].head, &rsc_group_wait[RSC_CGROUP_BACKGROUND].head},
	},
};
#else
wait_queue_head_t rsc_group_wait[RSC_CGROUP_END] = {
	[RSC_CGROUP_TOPAPP] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_TOPAPP].spinlock),
		.task_list = {&rsc_group_wait[RSC_CGROUP_TOPAPP].task_list, &rsc_group_wait[RSC_CGROUP_TOPAPP].task_list},
	},
	[RSC_CGROUP_RBG] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_TOPAPP].spinlock),
		.task_list = {&rsc_group_wait[RSC_CGROUP_RBG].task_list, &rsc_group_wait[RSC_CGROUP_RBG].task_list},
	},
	[RSC_CGROUP_DEFAULT] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_DEFAULT].spinlock),
		.task_list = {&rsc_group_wait[RSC_CGROUP_DEFAULT].task_list, &rsc_group_wait[RSC_CGROUP_DEFAULT].task_list},
	},
	[RSC_CGROUP_SYS_BACKGROUND] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].spinlock),
		.task_list = {&rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].task_list, &rsc_group_wait[RSC_CGROUP_SYS_BACKGROUND].task_list},
	},
	[RSC_CGROUP_BACKGROUND] = {
		.lock = __SPIN_LOCK_UNLOCKED(rsc_group_wait[RSC_CGROUP_BACKGROUND].spinlock),
		.task_list = {&rsc_group_wait[RSC_CGROUP_BACKGROUND].task_list, &rsc_group_wait[RSC_CGROUP_BACKGROUND].task_list},
	},
};
#endif
#endif

#ifdef CONFIG_RSC_EVENT_MMONITOR
/*
 * sum up all events from per cpu.
*/
static void mmonitor_calcu_events(unsigned long *ret)
{
	int i, cpu;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct mmonitor_event_state *s =
				&per_cpu(mmonitor_event_states, cpu);

		for (i = 0; i < NR_MMONITOR_EVENT_ITEMS; i++)
			ret[i] += s->event[i];
	}
	put_online_cpus();
}

static void mmonitor_clear_events(void)
{
	unsigned int i, cpu;

	for (i = 0; i < ARRAY_SIZE(rsc_pgalloc_count); i++)
		atomic_long_set(&rsc_pgalloc_count[i], 0);
	atomic_long_set(&rsc_slowpath_count[0], 0);
	atomic_long_set(&rsc_slowpath_count[1], 0);
	atomic_long_set(&rsc_slowpath_count[2], 0);
	atomic_long_set(&rsc_slowpath_count[3], 0);
	atomic_long_set(&rsc_slowpath_count[4], 0);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct mmonitor_event_state *s =
				&per_cpu(mmonitor_event_states, cpu);

		for (i = 0; i < NR_MMONITOR_EVENT_ITEMS; i++)
			s->event[i] = 0;

#ifdef CONFIG_VM_EVENT_COUNTERS
		struct vm_event_state *this =
				&per_cpu(vm_event_states, cpu);

		this->event[COMPACTSTALL] = 0;
		this->event[COMPACTSUCCESS] = 0;
		this->event[PGMAJFAULT] = 0;
#endif
	}
	put_online_cpus();
}
#endif

static int mmonitor_show(struct seq_file *s, void *data)
{
	int i;
#ifdef CONFIG_RSC_MEM_PRIORITY
	struct zone *zone;
#endif
#ifdef CONFIG_RSC_EVENT_MMONITOR
	char str[MMONITOR_MAX_STR_LEN] = {0};
	unsigned long *mmonitor_buf;
	unsigned long *vm_buf;

	if ((!enable) || (BETA_USER != get_logusertype_flag())) {
		seq_printf(s, "MMONITOR NOT SUPPORT\n");
		return 0;
	}

	mmonitor_buf = kzalloc(sizeof(struct mmonitor_event_state), GFP_KERNEL);
	if (mmonitor_buf == NULL) {
		pr_err("mmonitor_buf is invalid\n");
		return -EINVAL;
	}
	mmonitor_calcu_events(mmonitor_buf);

#ifdef CONFIG_VM_EVENT_COUNTERS
	vm_buf = kzalloc(sizeof(struct vm_event_state), GFP_KERNEL);
	if (vm_buf == NULL) {
		pr_err("vm_buf is invalid\n");
		kfree(mmonitor_buf);
		return -EINVAL;
	}
	all_vm_events(vm_buf);
#endif
#endif

	seq_printf(s,
		"slowpath0:\t%ld\t%llu us\n"
		"slowpath1:\t%ld\t%llu us\n"
		"slowpath2:\t%ld\t%llu us\n"
		"slowpath3:\t%ld\t%llu us\n"
		"slowpath4:\t%ld\t%llu us\n"

#ifdef CONFIG_RSC_EVENT_MMONITOR
#ifdef CONFIG_VM_EVENT_COUNTERS
		"compact_stall: %ld\n"
		"compact_suc: %ld\n"
#endif
		"warn_alloc_failed: %ld\n"
		"fcache : %ld\n"
		"fcache miss: %ld\n"
#endif
		,
		atomic_long_read(&rsc_slowpath_count[0]),
		rsc_slowpath_time[0],
		atomic_long_read(&rsc_slowpath_count[1]),
		rsc_slowpath_time[1],
		atomic_long_read(&rsc_slowpath_count[2]),
		rsc_slowpath_time[2],
		atomic_long_read(&rsc_slowpath_count[3]),
		rsc_slowpath_time[3],
		atomic_long_read(&rsc_slowpath_count[4]),
		rsc_slowpath_time[4]
#ifdef CONFIG_RSC_EVENT_MMONITOR
		,
#ifdef CONFIG_VM_EVENT_COUNTERS
		vm_buf[COMPACTSTALL],
		vm_buf[COMPACTSUCCESS],
#endif
		mmonitor_buf[ALLOC_FAILED_COUNT],
		mmonitor_buf[FILE_CACHE_READ_COUNT] + mmonitor_buf[FILE_CACHE_MAP_COUNT],
		mmonitor_buf[FILE_CACHE_MISS_COUNT] + vm_buf[PGMAJFAULT]
#endif
	);

	for (i = 0; i < ARRAY_SIZE(rsc_pgalloc_count); i++)
		seq_printf(s, "pg_alloc%d:\t%ld\n",
			i, atomic_long_read(&rsc_pgalloc_count[i]));

	seq_printf(s, "rsc_seq_alloc\t%d\tmax\t%d\tball\t%d\n",
		rsc_seq_alloc_num, rsc_max_seq_alloc,
			rsc_buf_alloc_num);

#ifdef CONFIG_RSC_MEM_PRIORITY
#define K(x) ((x) << (PAGE_SHIFT - 10))
	i = 0;
	for_each_zone(zone) {
		seq_printf(s, "zone[%d] watermark\t%lu\t%lu\t%lu\t%lu\n",
			i, K(zone->watermark[WMARK_MIN]), K(zone->watermark[WMARK_RBACK]),
			K(zone->watermark[WMARK_LOW]), K(zone->watermark[WMARK_HIGH]));
		i++;
	}
	for (i = 0; i < ARRAY_SIZE(rsc_cgroup_cnt); i++)
		seq_printf(s, "gcnt%d:\t%d\n",
			i, atomic_read(&rsc_cgroup_cnt[i]));

	seq_printf(s, "\n");
#endif
	print_slowpath(s, MEM_MON);

	print_mem_speed(s);

#ifdef CONFIG_RSC_EVENT_MMONITOR
	kfree(mmonitor_buf);
#ifdef CONFIG_VM_EVENT_COUNTERS
	kfree(vm_buf);
#endif
#endif
	return 0;
}

static ssize_t mmonitor_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
#ifdef CONFIG_RSC_EVENT_MMONITOR
	char ctl;

	if ((!enable) || (BETA_USER != get_logusertype_flag())) {
		printk("write to mmonitor not support\n");
		return -EFAULT;
	}

	if (copy_from_user((&ctl), buffer, sizeof(char)))
		return -EFAULT;

	if (ctl - 'C' == 0)
		mmonitor_clear_events();
#endif
	return 1;
}

static int mmonitor_open(struct inode *inode, struct file *file)
{
/*
	return single_open(file, mmonitor_show, NULL);
*/
	/*force to allocate 16KB buffer, it can speed up!*/
	size_t size = 16 * 1024;
	int ret;

	ret = single_open_size(file, mmonitor_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations mmonitor_file_operations = {
	.owner = THIS_MODULE,
	.open = mmonitor_open,
	.write = mmonitor_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mem_fg_show(struct seq_file *s, void *data)
{
	print_slowpath(s, MEM_FG);
	return 0;
}

static int mem_fg_open(struct inode *inode, struct file *file)
{
	size_t size = 2*PAGE_SIZE;
	int ret;

	ret = single_open_size(file, mem_fg_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations mem_fg_file_operations = {
	.owner = THIS_MODULE,
	.open = mem_fg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mem_lastfg_show(struct seq_file *s, void *data)
{
	print_slowpath(s, MEM_LASTFG);
	return 0;
}

static ssize_t mem_lastfg_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	char buf[16];
	int ret;
	u32 time;

	memset(buf, 0, sizeof(buf));
	if (count > sizeof(buf) - 1)
		count = sizeof(buf) - 1;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	ret = sscanf(buf, "%u", &time);
	if (ret != 1)
		return -EFAULT;

	mem_lastfg_backtime = time;
	rsc_info("mem slowpath  set mem_lastfg_backtime : %us\n", mem_lastfg_backtime);

	return count;
}

static int mem_lastfg_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, mem_lastfg_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations mem_lastfg_file_operations = {
	.owner = THIS_MODULE,
	.open = mem_lastfg_open,
	.read = seq_read,
	.write = mem_lastfg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int memlog_show(struct seq_file *m, void *v)
{
	unsigned long s_flags;
	struct rsc_seq_file_t *pseq;
	struct rtc_time tm;
	int i = 0;

	spin_lock_irqsave(&rsc_seq_spinlock, s_flags);

	list_for_each_entry(pseq, &rsc_mem_head, link) {
		if (!pseq->inuse) {
			if (pseq->cost_us >= memlog_print) {
				rtc_time_to_tm((unsigned long)pseq->tv_sec-sys_tz.tz_minuteswest * 60, &tm);
				seq_printf(m, "%2d\t%16s\t%5d\t%16s\t%5d\t%5d\tcost %u\tg %d\tord %d\t"
							"size %5u\t%04d-%02d-%02d %02d:%02d:%02d\n", i,
							(pseq->tgid == pseq->pid)?pseq->comm:pseq->tgcomm,
							(pseq->tgid == pseq->pid)?pseq->pid:pseq->tgid,
							(pseq->tgid == pseq->pid)?pseq->tgcomm:pseq->comm,
							(pseq->tgid == pseq->pid)?pseq->tgid:pseq->pid, pseq->ppid,
							pseq->cost_us, (int)pseq->group, (int)pseq->order, pseq->count,
							tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec
				);/*time 19 char*/
				seq_printf(m, "%s", pseq->buf);
				seq_printf(m, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n\n");
				i++;
			}
		}
	}

	spin_unlock_irqrestore(&rsc_seq_spinlock, s_flags);
	seq_printf(m, "==========size %lu============\n", m->count);

	return 0;
}

static int memlog_release(struct inode *inode, struct file *file)
{
	single_release(inode, file);

	return 0;
}

static int memlog_open(struct inode *inode, struct file *file)
{
	/*force to allocate big buffer, it can speed up!*/
	size_t size = (RSC_MAX_LOG_BUF_SIZE * RSC_MAX_THREADS_IN_SLOWPATH);
	int ret;

	ret = single_open_size(file, memlog_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations memlog_file_operations = {
	.owner = THIS_MODULE,
	.open = memlog_open,
	.write = NULL,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = memlog_release,
};

#ifdef CONFIG_RSC_MEM_PRIORITY
static int swait_show(struct seq_file *m, void *v)
{
	seq_printf(m, "mem slowpath wait %s, adjust mem pri %s."
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
			" slowpath_max_cnt: %u bgtrycnt: %u deftrycnt: %u"
#endif
			"\n",
			rsc_mem_slowpath_wait?"enable":"disable",
			rsc_mem_pri_enable?"enable":"disable"
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
			, rsc_mem_pri_slowpath_max_cnt, rsc_mem_pri_slowpath_bg_try_cnt, rsc_mem_pri_slowpath_def_try_cnt
#endif
	);
	return 0;
}

/*
disable slowpath wait and backgroup watermark:
echo 0 0 > /proc/vivo_rsc/mem_pri
enable slowpath wait and backgroup watermark:
echo 0 0 > /proc/vivo_rsc/mem_pri
*/
static ssize_t swait_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	u32 enable, backgrond;
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
	u32 maxcnt, bgcnt, defcnt;
#endif
	int i, ret;
	char kbuf[64];

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%u %u"
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
		" %u %u %u"
#endif
		, &enable, &backgrond
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
		, &maxcnt, &bgcnt, &defcnt
#endif
	);
	if (
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
		(ret < 2)
#else
		(ret != 2)
#endif
		|| (enable > 1) || (backgrond > 1))
		return -EINVAL;

	if (!enable && (rsc_mem_slowpath_wait != enable)) {
		for (i = 0; i < RSC_CGROUP_END; i++)
			if (waitqueue_active(&rsc_group_wait[i]))
				wake_up_all(&rsc_group_wait[i]);
	}
	rsc_mem_slowpath_wait = enable;
	rsc_mem_pri_enable = backgrond;
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
	if (ret >= 3)
		rsc_mem_pri_slowpath_max_cnt = maxcnt;
	if (ret >= 4)
		rsc_mem_pri_slowpath_bg_try_cnt = bgcnt;
	if (ret >= 5)
		rsc_mem_pri_slowpath_def_try_cnt = defcnt;
#endif
	smp_wmb();

	rsc_info("mem slowpath wait %s, backgrond watermark %s."
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
		" slowpath_max_cnt: %u bgtrycnt: %u deftrycnt: %u"
#endif
		"\n",
		enable?"enable":"disable",
		backgrond?"enable":"disable"
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
		, maxcnt, bgcnt, defcnt
#endif
		);

	return count;
}

static int swait_release(struct inode *inode, struct file *file)
{
	single_release(inode, file);

	return 0;
}

static int swait_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, swait_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations swait_file_operations = {
	.owner = THIS_MODULE,
	.open = swait_open,
	.write = swait_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = swait_release,
};

static int extfreekb_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", extra_free_kbytes);
	return 0;
}

static int extfreekb_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, extfreekb_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

/*enhance for /proc/sys/vm/extra_free_kbytes*/
static ssize_t extfreekb_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	int extfrekb;
	int ret;
	char kbuf[64];
	int free_kb;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%d", &extfrekb);
	/*should not bigger than 2GB*/
	if ((extfrekb < 0) || (extfrekb > 2*1024*1024))
		return -EINVAL;

	free_kb = extra_free_kbytes;
	if (free_kb != extfrekb) {
		user_min_free_kbytes = min_free_kbytes;
		extra_free_kbytes = extfrekb;
		setup_per_zone_wmarks();
		if (extfrekb > free_kb)
			rsc_wakeup_kswapd(GFP_HIGHUSER | __GFP_MOVABLE | __GFP_CMA,
				2, free_kb, extfrekb);
	}

	return count;
}

static int extfreekb_release(struct inode *inode, struct file *file)
{
	single_release(inode, file);

	return 0;
}

static const struct file_operations extfreekb_file_operations = {
	.owner = THIS_MODULE,
	.open = extfreekb_open,
	.write = extfreekb_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = extfreekb_release,
};
#endif

static DEFINE_MUTEX(rsc_procmem_lock);

#define task_rsc_alloc(task)		(task_cred_xxx((task), rsc_alloc))
#define RSC_MAX_UID_MEM (2*RSC_APP_MAX + RSC_9990_SYSTEM_MAX +			\
	RSC_3000_SYSTEM_MAX + RSC_2000_SYSTEM_MAX + RSC_1000_SYSTEM_MAX + 5)
/*root process 25, system task 80*/
#define RSC_MAX_SYSTEM_TASK	(80+25)

struct rsc_uid_mem_show_t {
	struct rsc_alloc_appinfo_t inf;
	struct rsc_uid_mem_t mem;
};
int *rsc_start_tbl;
struct rsc_uid_mem_show_t *rsc_start_mem;

#define CMP_NEG -1
#define CMP_POS 1
static int cmp_mem(const void *a, const void *b)
{
	const struct rsc_uid_mem_show_t *l = a, *r = b;
	int *tbl;
	int lpos, rpos;
	int l_pages, r_pages;

	lpos = l - rsc_start_mem;
	rpos = r - rsc_start_mem;

	tbl = rsc_start_tbl;
	l = &rsc_start_mem[tbl[lpos]];
	r = &rsc_start_mem[tbl[rpos]];

	l_pages = l->mem.rss_pages;
	r_pages = r->mem.rss_pages;

	if (l_pages > r_pages)
		return CMP_NEG;
	if (l_pages < r_pages)
		return CMP_POS;
	return 0;
}

static void swap_mem(void *a, void *b, int size)
{
	struct rsc_uid_mem_show_t *l = a, *r = b;
	int lpos, rpos, tmp;
	int *tbl;

	lpos = l - rsc_start_mem;
	rpos = r - rsc_start_mem;

	tbl = rsc_start_tbl;
	tmp = tbl[lpos];
	tbl[lpos] = tbl[rpos];
	tbl[rpos] = tmp;
}

#define show_appmem_detail(atime, idx)																		\
do {																										\
		seq_printf(s, "%3d\t%5d\t%16s\t%5d\t%16s\t%5d\t%5d\t%8ld\t"											\
						"%7ld\t%7ld\t%7ld\n",																\
			idx, atime->inf.uid,																			\
			(atime->inf.tgid == atime->inf.pid)?atime->inf.comm:atime->inf.tgcomm,							\
			(atime->inf.tgid == atime->inf.pid)?atime->inf.pid:atime->inf.tgid,								\
			(atime->inf.tgid == atime->inf.pid)?atime->inf.tgcomm:atime->inf.comm,							\
			(atime->inf.tgid == atime->inf.pid)?atime->inf.tgid:atime->inf.pid,								\
			atime->inf.ppid, (long)(atime->mem.rss_pages << (PAGE_SHIFT - 10)),								\
			(long)(atime->mem.file_pages << (PAGE_SHIFT - 10)),												\
			(long)(atime->mem.anon_pages << (PAGE_SHIFT - 10)),												\
			(long)(atime->mem.swap_pages << (PAGE_SHIFT - 10))												\
		 ); 																								\
} while (0)

#define show_appmem(atime, idx)																				\
do {																										\
		seq_printf(s, "%3d\t%5d\t%5d\t%8ld\t"																\
						"%7ld\t%7ld\t%7ld\n",																\
			idx, atime->inf.uid, atime->inf.pid,															\
			(long)(atime->mem.rss_pages << (PAGE_SHIFT - 10)),												\
			(long)(atime->mem.file_pages << (PAGE_SHIFT - 10)),												\
			(long)(atime->mem.anon_pages << (PAGE_SHIFT - 10)),												\
			(long)(atime->mem.swap_pages << (PAGE_SHIFT - 10))												\
		 ); 																								\
} while (0)

static void print_appmem(struct seq_file *s, struct rsc_uid_mem_show_t *mem,
			int cnt, int *tbl, int mode)
{
	int j;
	struct rsc_uid_mem_show_t *atime;

	if (unlikely(mode)) {
		seq_printf(s, "IDX\t  UID\t          TGTASK\t TGID\t   ThreadorPTASK\t  PID\t PPID\t"
			"Total KB\t FCache\t   Anon\t  Swaps\t\n");

		for (j = 0; j < cnt; j++) {
			atime = &mem[tbl[j]];
			show_appmem_detail(atime, j);
		}
	} else {
		seq_printf(s, "IDX\t  UID\t  PID\t"
			"Total KB\t FCache\t   Anon\t  Swaps\t\n");

		for (j = 0; j < cnt; j++) {
			atime = &mem[tbl[j]];
			show_appmem(atime, j);
		}
	}
}

#if defined(SPLIT_RSS_COUNTING)
static void sync_task_mm_rss(struct task_struct *task, struct mm_struct *mm)
{
	int i;

	for (i = 0; i < NR_MM_COUNTERS; i++) {
		if (task->rss_stat.count[i]) {
			add_mm_counter(mm, i, task->rss_stat.count[i]);
			task->rss_stat.count[i] = 0;
		}
	}
}
#endif

#undef CONFIG_RSC_FAST_PAGERECLAIM
static int appmem_show_internal(struct seq_file *m, void *v, int mode)
{
	struct task_struct *tsk;
	kuid_t uid;
	struct rsc_alloc_stat_t *atime;
	struct rsc_uid_mem_show_t *mem;
	char comm[TASK_COMM_LEN];
	pid_t ppid;
	pid_t tgid;
	int j, i, cnt;
	int *tbl;
	int uidnum = 0, systemtask = 0;
	u64 t1, t2, t3;
	long t_all = 0, t_file = 0, t_anon = 0, t_swaps = 0;
	long s_all = 0, s_file = 0, s_anon = 0, s_swaps = 0;
	int c_file, c_anon, c_swaps;
	int c_file0, c_anon0, c_swaps0;
	struct task_struct *t;

	mem = (struct rsc_uid_mem_show_t *)vmalloc((RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK) *
		(sizeof(struct rsc_uid_mem_show_t) + sizeof(int)));

	if (!mem)
		return -ENOMEM;

	mutex_lock(&rsc_procmem_lock);
	tbl = (int *)((char *)mem + (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK) *
		sizeof(struct rsc_uid_mem_show_t));

	rsc_start_mem = mem;
	rsc_start_tbl = tbl;

	t1 = local_clock();

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;

		if (tsk->flags & PF_KTHREAD)
			continue;

#ifdef CONFIG_RSC_FAST_PAGERECLAIM
		p = rsc_find_lock_task_mm(tsk);
#else
		p = find_lock_task_mm(tsk);
#endif
		if (!p)
			continue;
		uid = task_uid(p);

		c_file0 = get_mm_counter(p->mm, MM_FILEPAGES);
		c_anon0= get_mm_counter(p->mm, MM_ANONPAGES);
		c_swaps0 = get_mm_counter(p->mm, MM_SWAPENTS);

		for_each_thread(p, t) {
			sync_task_mm_rss(t, p->mm);
		}

		if ((uid_eq(uid, KUIDT_INIT(1000)) || uid_eq(uid, KUIDT_INIT(0)) || uid_eq(uid, KUIDT_INIT(1001)))
			&& (systemtask < (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK))) {

			c_file = get_mm_counter(p->mm, MM_FILEPAGES);
			c_anon = get_mm_counter(p->mm, MM_ANONPAGES);
			c_swaps = get_mm_counter(p->mm, MM_SWAPENTS);
			mem[systemtask].mem.file_pages = c_file;
			mem[systemtask].mem.anon_pages = c_anon;
			mem[systemtask].mem.swap_pages = c_swaps;
			mem[systemtask].mem.rss_pages = c_file + c_anon + c_swaps;
			s_file += c_file;
			s_anon += c_anon;
			s_swaps += c_swaps;
			s_all += c_file + c_anon + c_swaps;
			t_file += c_file;
			t_anon += c_anon;
			t_swaps += c_swaps;
			t_all += c_file + c_anon + c_swaps;
			rsc_get_pcomm(p, comm, &ppid, &tgid, NULL);
			strlcpy(mem[systemtask].inf.comm, p->comm, TASK_COMM_LEN);
			mem[systemtask].inf.pid = p->pid;
			mem[systemtask].inf.tgid = tgid;
			mem[systemtask].inf.ppid = ppid;
			mem[systemtask].inf.uid = (u32)uid.val;
			strlcpy(mem[systemtask].inf.tgcomm, comm, TASK_COMM_LEN);
			systemtask++;
			uidnum++;
#ifdef CONFIG_RSC_FAST_PAGERECLAIM
			read_unlock(&p->rsc_alloc_lock);
#else
			task_unlock(p);
#endif
			rsc_dbg(PROFILE, "appmemd UID %5d pid %5d %16s gl %5d %16s tt %d (%d) file %d (%d) anon %d (%d) swap %d (%d) KB\n",
				(u32)uid.val,
				p->pid, p->comm,
				tgid, comm,
				(c_file+c_anon+c_swaps) << (PAGE_SHIFT-10),
				(c_file0+c_anon0+c_swaps0) << (PAGE_SHIFT-10),
				c_file << (PAGE_SHIFT-10),
				c_file0 << (PAGE_SHIFT-10),
				c_anon << (PAGE_SHIFT-10),
				c_anon0 << (PAGE_SHIFT-10),
				c_swaps << (PAGE_SHIFT-10),
				c_swaps0 << (PAGE_SHIFT-10)
				);

			continue;
		}

		atime = task_rsc_alloc(p);

		if (unlikely(!atime)) {
			rsc_err("rsc_alloc NULL  appmem uid: %d %16s %5d\n",
				uid.val, p->comm, p->pid);
			atime = find_alloc_slot((u32)uid.val);
		}

		if (!atime->uid_mem.first_set) {
			memset(&atime->uid_mem, 0, sizeof(struct rsc_uid_mem_t));
			atime->uid_mem.first_set = 1;
			atime->uid_mem.enable = 1;
			/*if (!atime->inf.comm[0]) {*/
				rsc_get_pcomm(p, comm, &ppid, &tgid, NULL);
				strlcpy(atime->inf.comm, p->comm, TASK_COMM_LEN);
				atime->inf.pid = p->pid;
				atime->inf.tgid = tgid;
				atime->inf.ppid = ppid;
				atime->inf.uid = (u32)uid.val;
				strlcpy(atime->inf.tgcomm, comm, TASK_COMM_LEN);
				/*rsc_info("uidnum %3d appmem %16s %5d <- %16s %5d uid %6u\n",
					uidnum, p->comm, p->pid, comm, (p->pid == tgid) ? ppid : tgid, (u32)uid.val);*/
			/*}*/
			uidnum++;
		}
		c_file = get_mm_counter(p->mm, MM_FILEPAGES);
		c_anon = get_mm_counter(p->mm, MM_ANONPAGES);
		c_swaps = get_mm_counter(p->mm, MM_SWAPENTS);
		atime->uid_mem.file_pages += c_file;
		atime->uid_mem.anon_pages += c_anon;
		atime->uid_mem.swap_pages += c_swaps;
		atime->uid_mem.rss_pages += c_file + c_anon + c_swaps;
		t_file += c_file;
		t_anon += c_anon;
		t_swaps += c_swaps;
		t_all += c_file + c_anon + c_swaps;
		rsc_dbg(PROFILE, "appmemd UID %5d pid %5d %16s gl %5d %16s tt %d (%d) file %d (%d) anon %d (%d) swap %d (%d) KB\n",
			(u32)uid.val,
			p->pid, p->comm,
			tgid, comm,
			(c_file+c_anon+c_swaps) << (PAGE_SHIFT-10),
			(c_file0+c_anon0+c_swaps0) << (PAGE_SHIFT-10),
			c_file << (PAGE_SHIFT-10),
			c_file0 << (PAGE_SHIFT-10),
			c_anon << (PAGE_SHIFT-10),
			c_anon0 << (PAGE_SHIFT-10),
			c_swaps << (PAGE_SHIFT-10),
			c_swaps0 << (PAGE_SHIFT-10)
			);
#ifdef CONFIG_RSC_FAST_PAGERECLAIM
		read_unlock(&p->rsc_alloc_lock);
#else
		task_unlock(p);
#endif
	}
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	t2 = local_clock();

	cnt = systemtask;
	for (j = 0; j < ARRAY_SIZE(rsc_alloc_array); j++) {
		atime = rsc_alloc_array[j].alloc;
		for (i = 0; i < rsc_alloc_array[j].size; i++) {
			if (atime[i].uid_mem.enable) {
				if (cnt >= (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK))
					rsc_err("appmem cnt %d >= %d\n", cnt, (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK));
				else {
					memcpy(&mem[cnt].mem, &atime[i].uid_mem, sizeof(struct rsc_uid_mem_t));
					memcpy(&mem[cnt].inf, &atime[i].inf, sizeof(struct rsc_alloc_appinfo_t));
					/*rsc_info("tidy %3d appmem %16s %5d <- %16s %5d uid %6u\n",
						cnt, mem[cnt].inf.comm, mem[cnt].inf.pid, mem[cnt].inf.tgcomm, mem[cnt].inf.tgid, (u32)mem[cnt].inf.uid);*/
				}
				cnt++;
			}
			atime[i].uid_mem.first_set = 0;
			atime[i].uid_mem.enable = 0;
		}
	}

	for (j = 0; j < min(cnt, (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK)); j++)
		tbl[j] = j;

	sort(mem, min(cnt, (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK)), sizeof(struct rsc_uid_mem_show_t),
	     cmp_mem, swap_mem);
	t3 = local_clock();
	seq_printf(m, "appmem RSS info: cost traverse proc %4llu sort %4llu us "
		"uidnum %3d systasknum %3d all %4ld file %4ld anon %4ld swaps %4ld MB "
		"sysall %4ld sysfile %4ld sysanon %4ld sysswaps %4ld MB\n",
		(t2 - t1)/1000, (t3 - t2)/1000, cnt - systemtask, systemtask,
		t_all >> (20 - PAGE_SHIFT),	t_file >> (20 - PAGE_SHIFT),
		t_anon >> (20 - PAGE_SHIFT), t_swaps >> (20 - PAGE_SHIFT),
		s_all >> (20 - PAGE_SHIFT),	s_file >> (20 - PAGE_SHIFT),
		s_anon >> (20 - PAGE_SHIFT), s_swaps >> (20 - PAGE_SHIFT)
		);
	print_appmem(m, mem, min(cnt, (RSC_MAX_UID_MEM + RSC_MAX_SYSTEM_TASK)), tbl, mode);
	mutex_unlock(&rsc_procmem_lock);

	vfree(mem);

	return 0;
}

static int appmem_show(struct seq_file *m, void *v)
{
	return appmem_show_internal(m, v, 0);

}

static int appmem_show_detail(struct seq_file *m, void *v)
{
	return appmem_show_internal(m, v, 1);
}

static int appmem_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, appmem_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static int appmem_detail_open(struct inode *inode, struct file *file)
{
	size_t size = 2*PAGE_SIZE;
	int ret;

	ret = single_open_size(file, appmem_show_detail, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static int appmem_release(struct inode *inode, struct file *file)
{
	single_release(inode, file);

	return 0;
}

static const struct file_operations appmem_detail_file_operations = {
	.owner = THIS_MODULE,
	.open = appmem_detail_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = appmem_release,
};

static const struct file_operations appmem_file_operations = {
	.owner = THIS_MODULE,
	.open = appmem_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = appmem_release,
};

#ifdef CONFIG_RSC_MEM_DEFRAG
DEFINE_SPINLOCK(rsc_antifrag_spinlock);
u32 mem_smallord_MB, mem_bigord_MB;
static ssize_t mem_antifrag_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	u32 enable;
	int ret;
	char kbuf[64];
	struct zone *zone;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%u", &enable);
	if (ret != 1)
		return -EINVAL;

	if ((enable != 0) || (enable == !rsc_mem_antifrag_disabled)) {
		rsc_err("mem_antifrag can only set to zero ontime! enable %u cur %u\n", enable, !rsc_mem_antifrag_disabled);
		return -EINVAL;
	}

	spin_lock(&rsc_antifrag_spinlock);
	rsc_mem_antifrag_disabled = 1;
	for_each_zone(zone) {
		if (is_dma_zone(zone)) {
			rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_SORD,
				rsc_mem_antifrag_disabled);
			rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_BORD,
				rsc_mem_antifrag_disabled);
		}
	}
	spin_unlock(&rsc_antifrag_spinlock);
	return count;
}

static int mem_antifrag_show(struct seq_file *m, void *v)
{
	seq_printf(m, "enable:%d smallord:%d bigorder:%d MB\n", !rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB);
	return 0;
}

static int mem_antifrag_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, mem_antifrag_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations mem_antifrag_file_operations = {
	.owner = THIS_MODULE,
	.open = mem_antifrag_open,
	.write = mem_antifrag_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t mem_blocks_mb_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	u32 sord, bord;
	int ret;
	char kbuf[64];
	struct zone *zone;
	bool dosord = false, dobord = false;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%u %u", &sord, &bord);
	if (ret != 2) {
		rsc_err("mem_blocks_mb cmd error! example: echo smallordMB bigordMB > /proc/vivo_rsc/mem_blocks_mb\n"
				"enable:%d smallord:%d bigorder:%d MB\n",
				!rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB
		);
		return -EINVAL;
	}

	spin_lock(&rsc_antifrag_spinlock);
	if (mem_smallord_MB != sord) {
		mem_smallord_MB = sord;
		dosord = true;
	}

	if (mem_bigord_MB != bord) {
		mem_bigord_MB = bord;
		dobord = true;
	}
	ret = 0;
	for_each_zone(zone) {
		if (is_dma_zone(zone)) {
			if (dosord || rsc_mem_antifrag_disabled) {
				ret = rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_SORD,
					1);
				ret = rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_SORD,
					0);
				if (ret) {
					rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_SORD,
						1);
					rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_BORD,
						1);
					rsc_mem_antifrag_disabled = 1;
					rsc_info("set mem_blocks_mb smallorder error! set %s ret: %d enable:%d smallord:%d bigorder:%d MB\n",
						ret?"error!":" okay!", ret, !rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB
					);
					goto out;
				}
			}

			if (dobord || rsc_mem_antifrag_disabled) {
				ret = rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_BORD,
					1);
				ret = rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_BORD,
					0);
				if (ret) {
					rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_SORD,
						1);
					rsc_setup_zone_isolate_page(zone, MIGRATE_UNMOVABLE_ISOLATE_BORD,
						1);
					rsc_mem_antifrag_disabled = 1;
					rsc_info("set mem_blocks_mb bigorder error! set %s ret: %d enable:%d smallord:%d bigorder:%d MB\n",
						ret?"error!":" okay!", ret, !rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB
					);
					goto out;
				}
			}
		}
	}
	rsc_mem_antifrag_disabled = 0;
out:
	rsc_info("set mem_blocks_mb set %s ret: %d enable:%d smallord:%d bigorder:%d MB\n",
		ret?"error!":" okay!", ret, !rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB
		);
	spin_unlock(&rsc_antifrag_spinlock);

	return count;
}

static int mem_blocks_mb_show(struct seq_file *m, void *v)
{
	seq_printf(m, "enable:%d smallord:%d bigorder:%d MB\n", !rsc_mem_antifrag_disabled, mem_smallord_MB, mem_bigord_MB);
	return 0;
}

static int mem_blocks_mb_open(struct inode *inode, struct file *file)
{
	size_t size = PAGE_SIZE;
	int ret;

	ret = single_open_size(file, mem_blocks_mb_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations mem_blocks_mb_file_operations = {
	.owner = THIS_MODULE,
	.open = mem_blocks_mb_open,
	.write = mem_blocks_mb_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

static int __init mmonitor_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *dir;

#ifdef CONFIG_RSC_CGROUP
	BUILD_BUG_ON((int)RSC_CGROUP_TOPAPP != (int)RSC_GROUP_FG);
	BUILD_BUG_ON((int)RSC_CGROUP_RBG != (int)RSC_GROUP_RBG);
	BUILD_BUG_ON((int)RSC_CGROUP_SYS_BACKGROUND != (int)RSC_GROUP_BG);
	BUILD_BUG_ON((int)RSC_CGROUP_END != (int)RSC_GROUP_END);
#endif

	/*show all processes*/
	dir = proc_create_data("mem_mon", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &mmonitor_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_mon error!\n");

	dir = proc_create_data("mem_fg", S_IRUSR|S_IRGRP, vivo_rsc, &mem_fg_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_fg error!\n");

	dir = proc_create_data("mem_lastfg", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &mem_lastfg_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_lastfg error!\n");

	dir = proc_create_data("mem_log", S_IRUSR|S_IRGRP, vivo_rsc, &memlog_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_log error!\n");

#ifdef CONFIG_RSC_MEM_PRIORITY
	dir = proc_create_data("mem_extfreekb", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &extfreekb_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_extfreekb error!\n");

	dir = proc_create_data("mem_pri", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &swait_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_pri error!\n");
#endif

	dir = proc_create_data("appmemd", S_IRUSR|S_IRGRP, vivo_rsc, &appmem_detail_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create appmem error!\n");

	dir = proc_create_data("appmem", S_IRUSR|S_IRGRP, vivo_rsc, &appmem_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create appmem error!\n");

#ifdef CONFIG_RSC_MEM_DEFRAG
	/*
	 * The switch of disable anti memory fragmemtation function, expect 0 or 1.
	 * /proc/vivo_rsc/mem_antifrag
	 * Once set as 0, the value will not be changed.
	 * echo 0 > /proc/vivo_rsc/mem_antifrag
	 */
	dir = proc_create_data("mem_antifrag", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &mem_antifrag_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_antifrag error!\n");

	/*
	 * echo smallordMB bigordMB > /proc/vivo_rsc/mem_blocks_mb
	 * echo 0 88 >  /proc/vivo_rsc/mem_blocks_mb
	 * If set failed, we will disable mem antifragmentation.
	 * the interface only be used to debug!!!
	 */
	dir = proc_create_data("mem_blocks_mb", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &mem_blocks_mb_file_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create mem_blocks_mb error!\n");
#endif

#endif
	return 0;
}

late_initcall(mmonitor_init);
