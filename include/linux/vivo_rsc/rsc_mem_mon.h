/*
 * kernel/rsc/rsc_mem_mon.h
 *
 * VIVO Resource Control Memory.
 *
 */

#ifndef __RSC_MEM_MON_H__
#define __RSC_MEM_MON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/cpuset.h>
#include <linux/kallsyms.h>
#include <linux/delayacct.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#endif

#define RSC_CGROUP_SET_BIT (1<<8)

enum rsc_slowpath_t {
	RSC_SLOWPATH_MARK,
	RSC_SLOWPATH_WAIT,
	RSC_SLOWPATH_EXIT,
};

#if LINUX_VERSION_CODE == KERNEL_VERSION(3, 18, 71)
#define RSC_MAX_LOG_BUF_SIZE		(6*1024)
#else
#define RSC_MAX_LOG_BUF_SIZE		(10*1024)
#endif
#define RSC_RESERVE_LOG_BUF_SIZE	(400)

#define RSC_MAX_ZONE 2

#define RSC_MEM_RING_BUFF_ENABLE

struct rsc_time_t {
	/*us*/
	u32 shrink_lru[RSC_MAX_ZONE];
	u32 shrink_slab[RSC_MAX_ZONE];
	u32 shrink_lmk;
	/*unit is 10ns*/
	u32 shrink_zram[RSC_MAX_ZONE];
	/*unit is 10ns*/
	u32 shrink_io[RSC_MAX_ZONE];
	/*unit is 10ns*/
	u32 shrink_zcache[RSC_MAX_ZONE];
	/*super_cache_scan time , unit is 10ns*/
	u32 shrink_super_slab;
	/*zcache_scan time , unit is 10ns*/
	u32 shrink_zcache_slab;
	u16 shrink_zram_cnt[RSC_MAX_ZONE];
	u16 shrink_io_cnt[RSC_MAX_ZONE];
	u16 shrink_zcache_cnt[RSC_MAX_ZONE];
	/*ticks*/
	u8 rsc_wait;
	u8 iso_wait;
	u8 io_wait;
};

#ifdef CONFIG_TASK_DELAY_ACCT
struct rsc_delay_t {
	u32 blkio_delay;	/*us wait for sync block io completion */
	u32 swapin_delay;	/*us wait for swapin block io completion */
	u32 blkio_count;	/* total count of the number of sync block */
	u32 swapin_count;	/* total count of the number of swapin block */
	u32 freepages_delay;/*us wait for memory reclaim */
	u32 freepages_count;/* total count of memory reclaim */
};
#endif

struct rsc_seq_file_t {
	struct list_head link;
	u32 count;
#ifdef RSC_MEM_RING_BUFF_ENABLE
	u32 point;
#endif
	pid_t pid;
	pid_t tgid;
	pid_t ppid;
	/*4byte is enough*/
	u32 tv_sec;
	u32 cost_us;
	struct {
		unsigned idx:16;
		unsigned inuse:8;
		unsigned order:4;
		unsigned group:4;
	};
	char comm[TASK_COMM_LEN];
	char tgcomm[TASK_COMM_LEN];
	char buf[RSC_MAX_LOG_BUF_SIZE];
	struct rsc_time_t rt;
#ifdef CONFIG_TASK_DELAY_ACCT
	struct rsc_delay_t dl;
#endif
};

#if 0
struct rsc_alloc_stat_t {
	atomic_long_t speed_pages;
	u64 speed_jiffs;
	u64 speed_time;
	atomic_t speed_start;
	/*KB/s*/
	u32 speed_max;
	u32 speed_tv_sec;
#if 0
	u8 speed_enable;
	char speed_group;

	char group;
	char isapp;
	short adj;
	u8 morder;
#else
	struct {
		unsigned adj:16;
		unsigned morder:4;
		unsigned speed_group:4;
		unsigned group:4;
		unsigned speed_enable:1;
		unsigned isapp:1;
	};
#endif
	struct rsc_uid_mem_t uid_mem;

	/*total cnt*/
	atomic_long_t *tcnt;
	u32 time;
	u32 fg_time;
	/*max time*/
	u32 maxtime;
	u32 fg_maxtime;
	atomic_t  sorder;
	atomic_t cnt;
	/*group slowpath count*/
	u32 slowcnt[RSC_CGROUP_END];
	/*4byte is enough*/
	/*Time of Occurrence, slowpath time cost bigger than rsc_slowpath_detail*/
	u32 tv_sec;
	/*
	the time of max cost slowpath , when fg cnt is zero
	the time of max cost fg slowpath , when fg cnt is non-zero
	*/
	u32 max_tv_sec;
	/*the last slowpath time of fg task*/
	u32 last_tv_sec;
	/*the time of start record cost*/
	u32 record_tv_sec;
	/*form record_tv_sec up to now, the total slowpath time cost*/
	u32 record_fg_time;
	u32 record_page;
	u32 record_cnt;
	struct rsc_alloc_appinfo_t inf;
}

#endif
extern int rsc_mem_printf_inter(const char *f, ...);
extern void rsc_print_mem_buf(struct rsc_seq_file_t *seq, int group);
extern int getnstimeofday64_nolock(struct timespec64 *ts);

#ifdef CONFIG_RSC_FAST_PAGERECLAIM
#define rsc_fast_info(fmt, ...) \
	printk_deferred(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#endif

#define RSC_MAX_THREADS_IN_SLOWPATH	64
#define RSC_MAX_THREADS_IN_SLOWPATH_MASK	(RSC_MAX_THREADS_IN_SLOWPATH-1)

#define RSC_MEM_KSWAPD_PRINT_TIME_US (1500UL*1000UL)
#define RSC_MEM_KSWAPD_DETAIL_TIME_US (5UL*1000UL*1000UL)

/*
sync with:
/system/core/include/cutils/android_filesystem_config.h
size is about 40KB
such as:
cameraserver process
#define AID_CAMERASERVER 1047
*/
#define RSC_AID_CAMERASERVER 1047

#define RSC_APP_MAX	600
#define RSC_DUAL_APP_START	99910000
#define RSC_DUAL_APP_END	(RSC_DUAL_APP_START + RSC_APP_MAX)

#define RSC_APP_START	10000
#define RSC_APP_END	(RSC_APP_START + RSC_APP_MAX)

#define RSC_9990_SYSTEM_MAX	10
#define RSC_9990_SYSTEM_START	9990
#define RSC_9990_SYSTEM_END	(RSC_9990_SYSTEM_START + RSC_9990_SYSTEM_MAX)

#define RSC_3000_SYSTEM_MAX	20
#define RSC_3000_SYSTEM_START	3000
#define RSC_3000_SYSTEM_END	(RSC_3000_SYSTEM_START + RSC_3000_SYSTEM_MAX)

#define RSC_2000_SYSTEM_MAX	10
#define RSC_2000_SYSTEM_START	2000
#define RSC_2000_SYSTEM_END	(RSC_2000_SYSTEM_START + RSC_2000_SYSTEM_MAX)

#define RSC_1000_SYSTEM_MAX	100
#define RSC_1000_SYSTEM_START	1000
#define RSC_1000_SYSTEM_END	(RSC_1000_SYSTEM_START + RSC_1000_SYSTEM_MAX)

#define DUAL_APP_PREFIX 999
#define APP_BASE 100000

#ifndef RSC_MS_TO_NS
#define RSC_MS_TO_NS(msec)		((msec) * 1000 * 1000)
#endif

#ifndef RSC_SEC_TO_NS
#define RSC_SEC_TO_NS(sec)		((sec) * 1000 * 1000 * 1000)
#endif

#ifndef RSC_NS_TO_MS
#define RSC_NS_TO_MS(nsec)		((nsec) / (1000 * 1000))
#endif

#ifndef RSC_NS_TO_US
#define RSC_NS_TO_US(nsec)		((nsec) / 1000)
#endif

#ifndef RSC_US_TO_NS
#define RSC_US_TO_NS(usec)		((usec) * 1000)
#endif

#define current_rsc_alloc()		(current_cred_xxx(rsc_alloc))

extern struct rsc_alloc_stat_t rsc_dual_app_time[RSC_APP_MAX];
extern atomic_long_t rsc_dual_app_cnt;

extern struct rsc_alloc_stat_t rsc_app_time[RSC_APP_MAX];
extern atomic_long_t rsc_app_cnt;

extern struct rsc_alloc_stat_t rsc_9990_system_time[RSC_9990_SYSTEM_MAX];
extern atomic_long_t rsc_9990_system_cnt;

extern struct rsc_alloc_stat_t rsc_3000_system_time[RSC_3000_SYSTEM_MAX];
extern atomic_long_t rsc_3000_system_cnt;

extern struct rsc_alloc_stat_t rsc_2000_system_time[RSC_2000_SYSTEM_MAX];
extern atomic_long_t rsc_2000_system_cnt;

extern struct rsc_alloc_stat_t rsc_1000_system_time[RSC_1000_SYSTEM_MAX];
extern atomic_long_t rsc_1000_system_cnt;

extern struct rsc_alloc_stat_t rsc_native_time;
extern atomic_long_t rsc_native_cnt;

extern struct rsc_alloc_stat_t rsc_other_app_time;
extern atomic_long_t rsc_other_app_cnt;

extern struct rsc_alloc_stat_t rsc_system_server_time;
extern atomic_long_t rsc_system_server_cnt;

extern int rsc_seq_alloc_num;
extern int rsc_max_seq_alloc;
extern unsigned long rsc_seq_alloc_bit;
extern struct rsc_seq_file_t rsc_seq_file[RSC_MAX_THREADS_IN_SLOWPATH];
extern struct rsc_seq_file_t rsc_kswapd_seq_file;
extern spinlock_t rsc_slowpath_spinlock;
extern spinlock_t rsc_seq_spinlock;
extern int __read_mostly rsc_mem_mon_enable;
extern int __read_mostly rsc_mem_slowpath_trace;

extern struct list_head rsc_mem_head;
extern int rsc_buf_alloc_num;
extern int __read_mostly rsc_mem_speed_interval;
extern u32 __read_mostly rsc_mem_pri_enable;
extern int __read_mostly rsc_mem_logmask;
extern atomic_long_t rsc_pgalloc_count[MAX_ORDER+1];

extern int __read_mostly mem_record_ns;
extern u32 mem_lastfg_backtime;

/*(!(current->flags & PF_KSWAPD)) && */
#define rsc_mem_printf(timeout, m, fmt, args...)												\
	do {																						\
		if ((m) && (timeout) && (current->flags & PF_MEMALLOC)) {								\
			if ((rsc_mem_printf_inter("|RSC %2d "fmt, m->idx++, ##args) < 0) &&					\
				(rsc_mem_slowpath_trace > 1))													\
					printk("#RSC %2d "fmt,  m->idx++, ##args);/*direct output*/					\
		}																						\
	} while (0)

#define rsc_mem_force_printf(fmt, args...)														\
	do {																						\
		struct rsc_seq_file_t *m = (struct rsc_seq_file_t *)current_thread_info()->rsc_seq;	\
		/*may be  the buffer m is NULL*/														\
		if (m && (rsc_mem_printf_inter("|RSC %2d "fmt, m->idx++, ##args) < 0) &&			\
			(rsc_mem_slowpath_trace > 1))													\
			printk("#RSC "fmt, ##args);/*direct output*/										\
	} while (0)

#define rsc_kswapd_printf(timeout, fmt, args...)												\
	do {																						\
		if ((timeout) && (current->flags & PF_MEMALLOC)) {										\
			struct rsc_seq_file_t *m = (struct rsc_seq_file_t *)current_thread_info()->rsc_seq;	\
			if (m && (rsc_mem_printf_inter("|RSC %2d "fmt, m->idx++, ##args) < 0) &&			\
				(rsc_mem_slowpath_trace > 1))													\
				printk("#RSC %2d "fmt, m->idx++, ##args);/*direct output*/							\
		}																						\
	} while (0)


#define rsc_local_clock()													\
({																			\
		u64 __ret;															\
		if ((!(current->flags & PF_KSWAPD)) 								\
			&& (current->flags & PF_MEMALLOC)) 								\
			__ret = local_clock();/*ktime_get_ns()*/							\
		else																\
			__ret = 0;														\
		__ret;																\
})

#define rsc_kswapd_clock()													\
({																			\
		u64 __ret;															\
		if ((current->flags & PF_MEMALLOC))									\
			__ret = local_clock();											\
		else																\
			__ret = 0;														\
		__ret;																\
})

#define is_background(g)	(g >= RSC_CGROUP_SYS_BACKGROUND?1:0)
#define is_rbackground(g)	((g == RSC_CGROUP_RBG) || (g == RSC_CGROUP_DEFAULT)?1:0)
#define is_foreground(g)	(g <= RSC_CGROUP_TOPAPP?1:0)


enum logmask_t {
	MEM_REC_SLOWPATH = 1 << 0,
	/*has flag __GFP_DIRECT_RECLAIM*/
	MEM_REC_WAIT = 1 << 1,
	/*has flag __GFP_KSWAPD_RECLAIM*/
	MEM_REC_WKSWAD = 1 << 2,
};

/*
use list  to replace find_first_zero_bit/set_bit/clear_bit
bit = find_first_zero_bit(&rsc_seq_alloc_bit, RSC_MAX_THREADS_IN_SLOWPATH);
set_bit(bit, &rsc_seq_alloc_bit);
clear_bit(bit, &rsc_seq_alloc_bit);
*/
static inline void rsc_alloc_seq_buf(void)
{
	unsigned long s_flags;
	struct thread_info *thread = current_thread_info();
	int seq;
	struct rsc_seq_file_t *pseq;
	struct timespec64 dtime;

	spin_lock_irqsave(&rsc_seq_spinlock, s_flags);
	seq = rsc_seq_alloc_num;
	rsc_seq_alloc_num++;
	rsc_max_seq_alloc = max(rsc_seq_alloc_num,
			rsc_max_seq_alloc);

	if (unlikely(!rsc_mem_slowpath_trace))
		goto exit;

	if (unlikely(rsc_buf_alloc_num < RSC_MAX_THREADS_IN_SLOWPATH)) {
		pseq = &rsc_seq_file[rsc_buf_alloc_num];
		rsc_buf_alloc_num++;
		pseq->inuse = 1;
		list_add_tail(&pseq->link, &rsc_mem_head);
		spin_unlock_irqrestore(&rsc_seq_spinlock, s_flags);
		memset(&pseq->rt, 0, sizeof(struct rsc_time_t));
		pseq->count = 0;
#ifdef RSC_MEM_RING_BUFF_ENABLE
		pseq->point = 0;
#endif
		pseq->idx = 0;
		pseq->buf[RSC_MAX_LOG_BUF_SIZE-1] = '\0';
		getnstimeofday64_nolock(&dtime);
		pseq->tv_sec = (u32)dtime.tv_sec;
#ifdef CONFIG_TASK_DELAY_ACCT
		if (likely(current->delays)) {
			pseq->dl.blkio_delay = (u32)current->delays->blkio_delay/1000;
			pseq->dl.swapin_delay = (u32)current->delays->swapin_delay/1000;
			pseq->dl.blkio_count = current->delays->blkio_count;
			pseq->dl.swapin_count = current->delays->swapin_count;
			pseq->dl.freepages_delay = (u32)current->delays->freepages_delay/1000;
			pseq->dl.freepages_count = current->delays->freepages_count;
		} else
			memset(&pseq->dl, 0, sizeof(struct rsc_delay_t));
#endif
		thread->rsc_seq = (void *)pseq;
		return;
	} else {
		if (likely(seq < RSC_MAX_THREADS_IN_SLOWPATH)) {
			/*int i = 0;*/
			list_for_each_entry(pseq, &rsc_mem_head, link) {
				if (!pseq->inuse) {
					/*rsc_info("allocdbg %d\n", i);*/
					pseq->inuse = 1;
					list_move_tail(&pseq->link, &rsc_mem_head);
					spin_unlock_irqrestore(&rsc_seq_spinlock, s_flags);
					memset(&pseq->rt, 0, sizeof(struct rsc_time_t));
					pseq->count = 0;
#ifdef RSC_MEM_RING_BUFF_ENABLE
					pseq->point = 0;
#endif
					pseq->idx = 0;
					pseq->buf[RSC_MAX_LOG_BUF_SIZE-1] = '\0';
					getnstimeofday64_nolock(&dtime);
					pseq->tv_sec = (u32)dtime.tv_sec;
#ifdef CONFIG_TASK_DELAY_ACCT
					if (likely(current->delays)) {
						pseq->dl.blkio_delay = current->delays->blkio_delay/1000;
						pseq->dl.swapin_delay = current->delays->swapin_delay/1000;
						pseq->dl.blkio_count = current->delays->blkio_count;
						pseq->dl.swapin_count = current->delays->swapin_count;
						pseq->dl.freepages_delay = current->delays->freepages_delay/1000;
						pseq->dl.freepages_count = current->delays->freepages_count;
					} else
						memset(&pseq->dl, 0, sizeof(struct rsc_delay_t));
#endif
					thread->rsc_seq = (void *)pseq;
					return;
				}
				/*i++;*/
			}
			rsc_err("slowpath cound not find seq_buf! used %d max %d buff size %d\n",
				rsc_seq_alloc_num, rsc_max_seq_alloc,
				RSC_MAX_THREADS_IN_SLOWPATH);
		} else {
			printk_ratelimited("RSC slowpath seq_buf full! used %d max %d buff size %d\n",
				rsc_seq_alloc_num, rsc_max_seq_alloc,
				RSC_MAX_THREADS_IN_SLOWPATH);
		}
	}
exit:
	thread->rsc_seq = (void *)NULL;
	spin_unlock_irqrestore(&rsc_seq_spinlock, s_flags);
}

static inline void rsc_free_seq_buf(void)
{
	struct thread_info *thread = current_thread_info();
	unsigned long s_flags;
	struct rsc_seq_file_t *pseq = (struct rsc_seq_file_t *)thread->rsc_seq;

	spin_lock_irqsave(&rsc_seq_spinlock, s_flags);
	rsc_seq_alloc_num--;
	if (pseq) {
		if (likely(pseq->count))
			list_move_tail(&pseq->link, &rsc_mem_head);
		else/*move the head!!*/
			list_move(&pseq->link, &rsc_mem_head);
		pseq->inuse = 0;
		thread->rsc_seq = NULL;
	}
	spin_unlock_irqrestore(&rsc_seq_spinlock, s_flags);
}

static inline void rsc_get_pcomm(struct task_struct *p, char *comm,
			pid_t *ppid, pid_t *tgid, char *ppcomm)
{
	struct task_struct *pt;
	struct task_struct *ppt;

	rcu_read_lock();
	ppt = rcu_dereference(p->real_parent);
#if 0
	{/*test!!*/
		struct task_struct *sig_ppt;
		sig_ppt = rcu_dereference(p->parent);

		if (sig_ppt != ppt)
			if (sig_ppt && ppt)
				rsc_info("note parent, rppt %16s %5d sppt %16s %5d\n",
				ppt->comm, ppt->pid, sig_ppt->comm, sig_ppt->pid);
	}
#endif
	pt = rcu_dereference(p->group_leader);
	if (ppt) {
		*ppid = ppt->pid;
		if (ppcomm)
			strlcpy(ppcomm, ppt->comm, TASK_COMM_LEN);
	} else {
		*ppid = RSC_END_PID;
		if (ppcomm)
			ppcomm[0] = '\0';
	}

	if (pt) {
		*tgid = pt->pid;
		if (thread_group_leader(p)) {
			if (ppt)
				strlcpy(comm, ppt->comm, TASK_COMM_LEN);
			else
				strlcpy(comm, "group_leader", TASK_COMM_LEN);
		} else {
			if (pt)
				strlcpy(comm, pt->comm, TASK_COMM_LEN);
			else
				strlcpy(comm, "group_leader", TASK_COMM_LEN);
		}
	} else {
		strlcpy(comm, "group_leader", TASK_COMM_LEN);
		*tgid = RSC_END_PID;
	}
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
}

static inline struct rsc_alloc_stat_t *find_alloc_slot(u32 uid)
{
	/* 99901000         .server.telecom*/
	u32 auid;

	if (uid >= (DUAL_APP_PREFIX * APP_BASE))
		auid = uid - DUAL_APP_PREFIX * APP_BASE;
	else
		auid = uid;

	if (likely(uid >= RSC_APP_START && uid < RSC_APP_END))
		return &rsc_app_time[uid - RSC_APP_START];
	else if (auid >= RSC_1000_SYSTEM_START && auid < RSC_1000_SYSTEM_END) {
/*at this time, the task name of system_server has not been set.
	we change the cred->rsc_alloc in prctl PR_SET_NAME
		if ((auid == RSC_1000_SYSTEM_START) &&
			!strncmp(current->comm, "system_server", TASK_COMM_LEN))
			return &rsc_system_server_time;
*/
		return &rsc_1000_system_time[auid - RSC_1000_SYSTEM_START];
	} else if (uid == 0)
		return &rsc_native_time;
	else if (uid >= RSC_DUAL_APP_START && uid < RSC_DUAL_APP_END)
		return &rsc_dual_app_time[uid - RSC_DUAL_APP_START];
	else if (auid >= RSC_3000_SYSTEM_START && auid < RSC_3000_SYSTEM_END)
		return &rsc_3000_system_time[auid - RSC_3000_SYSTEM_START];
	else if (auid >= RSC_2000_SYSTEM_START && auid < RSC_2000_SYSTEM_END)
		return &rsc_2000_system_time[auid - RSC_2000_SYSTEM_START];
	else if (auid >= RSC_9990_SYSTEM_START && auid < RSC_9990_SYSTEM_END)
		return &rsc_9990_system_time[auid - RSC_9990_SYSTEM_START];
	else
		return &rsc_other_app_time;
}

static inline void rsc_mem_speed(int ord)
{
	kuid_t uid;
	struct rsc_alloc_stat_t *atime;
	u64 stime, tdif;
	int interval;

	interval = rsc_mem_speed_interval;
	if (likely((interval <= 0) || in_interrupt()))
		return;

#if 0
	/*test only!!*/
	{
		char comm[TASK_COMM_LEN];
		pid_t ppid;
		pid_t tgid;

		rsc_get_pcomm(current, comm, &ppid, &tgid, NULL);
	}
#endif
	uid = current_uid();
	atime = current_rsc_alloc();
	if (unlikely(!atime)) {
		rsc_err("rsc_alloc NULL  memspeed uid: %d %16s %5d\n",
			uid.val, current->comm, current->pid);
		atime = find_alloc_slot((u32)uid.val);
	}

	/*
	There are some race conditions here, but it is a monitor only, no need very accuracy.
	so we donot use spinlock to protect it.
	*/
	if (unlikely(!atomic_xchg(&atime->speed_start, 1))) {
		atomic_long_set(&atime->speed_pages, 1<<ord);
		atime->speed_jiffs = get_jiffies_64();
		atime->speed_time = local_clock();
	} else {
		u64 jif;

		atomic_long_add(1<<ord, &atime->speed_pages);
		jif = get_jiffies_64();
		if (unlikely(time_after_eq64(jif,
			atime->speed_jiffs + msecs_to_jiffies(interval)))) {
			struct timespec64 dtime;
			u32 max;

			stime = local_clock();
			tdif = stime - atime->speed_time;

			/*deal with race condition, fix tdiff is very little and maxval is very big.*/
			if (likely(tdif >= RSC_MS_TO_NS(interval/2))) {
				/*let another thread not get in for performance!*/
				atime->speed_jiffs = jif;

				/*donot use smp_wmb(); for performance*/
				barrier();

				max = ((u64)atomic_long_read(&atime->speed_pages) * RSC_SEC_TO_NS(1))/tdif;
				/*convert to KB/s*/
				max <<= (PAGE_SHIFT-10);
				if (unlikely(max > atime->speed_max)) {
					char comm[TASK_COMM_LEN];
					pid_t ppid;
					pid_t tgid;
					int group;

					atime->speed_max = max;
					getnstimeofday64_nolock(&dtime);
					atime->speed_tv_sec = (u32)dtime.tv_sec;

					group = current_thread_info()->rsc_group;
					if (group & RSC_CGROUP_SET_BIT) {
						group &= ~RSC_CGROUP_SET_BIT;
						atime->speed_group = group;
						current_thread_info()->rsc_group = group;
					} else
						atime->speed_group = rsc_task_group(current);
					if (!atime->inf.comm[0] || !atime->isapp) {
						rsc_get_pcomm(current, comm, &ppid, &tgid, NULL);
						strlcpy(atime->inf.comm, current->comm, TASK_COMM_LEN);
						atime->inf.pid = current->pid;
						atime->inf.tgid = tgid;
						atime->inf.ppid = ppid;
						atime->inf.uid = (u32)uid.val;
						strlcpy(atime->inf.tgcomm, comm, TASK_COMM_LEN);
					}
					atime->speed_enable = 1;
				}
				atomic_long_set(&atime->speed_pages, 0);
				/*donot use smp_wmb(); for performance*/
				barrier();
				atomic_set(&atime->speed_start, 0);
			}
		}
	}
}

/*ns, 80000ms, 80s*/
#define RSC_SYSTEM_BOOTTIME		(80*1000)
 static const char *rsc_skip_func[] = {
	"xt_alloc_table_info",
	"mtp_request_new",
	"diagfwd_buffers_init",
	"allocate_rx_intent",
	"check_bufsize_for_encoding"
};
static char rsc_skip_func_print[ARRAY_SIZE(rsc_skip_func)];

/*
 * rsc_bigorder_monitor - monitor bigorder(>=3) allocate
 * @order: page_order
 *
 */
static inline void rsc_bigorder_monitor(unsigned int order, gfp_t gfp_mask,
	enum zone_type classzone_idx, struct page *page, void *ret_ip)
{
	char comm[TASK_COMM_LEN];
	pid_t ppid;
	pid_t tgid;
	int iswait;
	static int checktime;

	if (likely(!(rsc_mem_logmask & (MEM_REC_WAIT | MEM_REC_WKSWAD))))
		return;

	if (order >= PAGE_ALLOC_COSTLY_ORDER) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 4, 0)
		if ((gfp_mask & __GFP_DIRECT_RECLAIM) || (gfp_mask & __GFP_KSWAPD_RECLAIM)) {
#else
		if ((gfp_mask & __GFP_WAIT) || !(gfp_mask & __GFP_NO_KSWAPD)) {
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 4, 0)
			if (gfp_mask & __GFP_DIRECT_RECLAIM)
#else
			if (gfp_mask & __GFP_WAIT)
#endif
				iswait = 1;
			else
				iswait = 0;

			/*skip boottime alloc big order memory*/
			if (unlikely(!checktime)) {
				if (time_before(jiffies, INITIAL_JIFFIES + msecs_to_jiffies(RSC_SYSTEM_BOOTTIME)))
					return;
				checktime = 1;
			}

			if ((iswait && (rsc_mem_logmask & MEM_REC_WAIT)) ||
				(!iswait && (rsc_mem_logmask & MEM_REC_WKSWAD))) {
				unsigned long stack_entries[4];
				struct stack_trace callstack;
				int j, i, print;
				char buffer[KSYM_SYMBOL_LEN];
				char pcomm[TASK_COMM_LEN];
				#define PRINT_FUNC_BASE 2

				if (order > MAX_ORDER)
					order = MAX_ORDER;

				atomic_long_inc(&rsc_pgalloc_count[order]);
/*
1. FIX skip mtp alloc  mtp_request_new, order is 8, call stack:
				^RSC enter bigorder wait  adj -1000 idx 1 order 8 gfp_mask	24040c0 			init	 1 <-		 swapper/0	   0	 0 page ffffffbdc3974000 ret_ip alloc_kmem_pages
				CPU: 6 PID: 1 Comm: init Tainted: G 	   W  O    4.4.21-perf+ #286
				Hardware name: vivo, Inc. SDM 660 PM660 + PM660A PD1709 (DT)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x5e4/0x620
				__alloc_pages_nodemask+0x19c/0xb0c
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264
				mtp_request_new+0x44/0x78
				mtp_function_bind+0x12c/0x310
				usb_add_function+0x84/0x10c
				configfs_composite_bind+0x274/0x300
				udc_bind_to_driver+0x38/0xd0
				usb_udc_attach_driver+0x80/0xa8
				gadget_dev_desc_UDC_store+0xac/0xf0
				configfs_write_file+0xe8/0x128
				__vfs_write+0x28/0xd0
				vfs_write+0xac/0x144
				SyS_write+0x48/0x84
				cpu_switch_to+0x250/0x420

2. FIX skip net allocate xt_alloc_table_info, order is 3, call stack:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c					skip == 0
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x890/0x920
				__alloc_pages_nodemask+0x1a8/0xbe8
				alloc_kmem_pages+0x18/0x20			skip == 4
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264					skip == 7
				xt_alloc_table_info+0x64/0x88
				do_ip6t_set_ctl+0x10c/0x214
				nf_setsockopt+0x5c/0x7c
				ipv6_setsockopt+0x9c/0xbc
				rawv6_setsockopt+0x5c/0x78
				sock_common_setsockopt+0x18/0x20
				SyS_setsockopt+0x8c/0xb8
				cpu_switch_to+0x250/0x420

3.mmc_blk_ioctl_copy_from_user, order is 3 :
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x908/0x944
				__alloc_pages_nodemask+0x1a8/0xbe8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264
				mmc_blk_ioctl_copy_from_user+0xa8/0x134
				mmc_blk_ioctl_rpmb_cmd.isra.28+0xf4/0x570
				mmc_blk_ioctl+0x1c4/0x3b8
				__blkdev_driver_ioctl+0x20/0x30
				blkdev_ioctl+0x7bc/0x834
				block_ioctl+0x40/0x4c
				do_vfs_ioctl+0x48c/0x564
				SyS_ioctl+0x60/0x88
				cpu_switch_to+0x250/0x420

4.前面500秒左右有调用
^RSC enter bigorder wait  adj -1000 idx 1 ord 5 gfp  24002c2         adsprpcd   995 <-         adsprpcd   888     1 p ffffffbdc1b3d000 cal arm_iommu_alloc_attrs ()
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x94c/0x988
				__alloc_pages_nodemask+0x1a8/0xbe8
				arm_iommu_alloc_attrs+0x2dc/0x440
				get_args+0x8d0/0xa24
				fastrpc_internal_invoke+0x43c/0xbe8
				fastrpc_device_ioctl+0x194/0x544
				do_vfs_ioctl+0x48c/0x564
				SyS_ioctl+0x60/0x88
				cpu_switch_to+0x250/0x420

5. FIX check_bufsize_for_encoding 频度不高，monkey测试15小时，出现5次
^RSC enter bigorder wait  adj     0 idx 1 ord 4 gfp  24040c0   kworker/u16:17 10705 <-         kthreadd     2     2 p           (null) cal alloc_kmem_pages ()
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x94c/0x988
				__alloc_pages_nodemask+0x1a8/0xbe8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc_track_caller+0x4c/0x264
				krealloc+0x64/0xa8
				check_bufsize_for_encoding+0x78/0xa4
				diagfwd_data_process_done+0x130/0x234
				diagfwd_data_read_untag_done+0x46c/0x730
				diagfwd_channel_read_done+0xb4/0xd0
				diag_socket_read+0x384/0x450
				diagfwd_channel_read+0x118/0x12c

6. FIX allocate_rx_intent
^RSC enter bigorder wait  adj     0 idx 1 ord 3 gfp  24040c0    kworker/u16:8   645 <-         kthreadd     2     2 p ffffffbdc3919200 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xa3c/0xa78
				__alloc_pages_nodemask+0x1a8/0xbec
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264
				allocate_rx_intent+0x20/0x54
				glink_queue_rx_intent+0x324/0x7f8
				glink_xprt_qrx_intent_worker+0x68/0x90
				process_one_work+0x228/0x3ec
				worker_thread+0x300/0x420
				kthread+0xdc/0xe4
				cpu_switch_to+0x210/0x420

7. FIX function alloc_one_pg_vec_page
^RSC enter bigorder wait  adj -1000 idx 1 ord 10 gfp  240d2c0            clatd 25556 <-     Binder:915_2  1934  1934 p           (null) cal __get_free_pages ()
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0x94c/0x988
				__alloc_pages_nodemask+0x1a8/0xbe8
				__get_free_pages+0x18/0x5c
				packet_set_ring+0x1e4/0x580
				packet_setsockopt+0x328/0xb28
				SyS_setsockopt+0x8c/0xb8
				cpu_switch_to+0x250/0x420

8. FIX it , diagfwd_buffers_init , only allocate in system bootup?
^RSC enter bigorder wait  adj     0 idx 1 ord 3 gfp  240c0c0    kworker/u16:8   644 <-         kthreadd     2     2 p ffffffbdc36a0800 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xbb4/0xbf0
				__alloc_pages_nodemask+0x1a8/0xd1c
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				diagfwd_buffers_init+0x654/0x710
				socket_read_work_fn+0x2c/0x40
				process_one_work+0x228/0x3ec
				worker_thread+0x300/0x420
				kthread+0xdc/0xe4
				cpu_switch_to+0x210/0x420

^RSC enter bigorder wait  adj -1000 idx 1 ord 3 gfp  240c0c0            ipacm  1157 <-            ipacm  1123     1 p ffffffbdc0154800 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xbb0/0xbec
				__alloc_pages_nodemask+0x1c0/0x10b8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				wan_ioctl+0x180/0xac4
				do_vfs_ioctl+0x48c/0x564
				SyS_ioctl+0x60/0x88
				cpu_switch_to+0x250/0x420

^RSC enter bigorder wait  adj -1000 idx 1 ord 3 gfp  24040c0            ipacm  1157 <-            ipacm  1123     1 p ffffffbdc348ac00 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xbb0/0xbec
				__alloc_pages_nodemask+0x1c0/0x10b8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264
				qmi_encode_and_send_req+0x158/0x36c
				qmi_send_req_wait+0x64/0x2a4
				qmi_filter_request_send+0x19c/0x1c8
				wan_ioctl+0x1e0/0xac4
				do_vfs_ioctl+0x48c/0x564
				SyS_ioctl+0x60/0x88
				cpu_switch_to+0x250/0x420

^RSC enter bigorder wait  adj -1000 idx 1 ord 3 gfp  24040c0            ipacm  1157 <-            ipacm  1123     1 p ffffffbdc390fc00 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xbb0/0xbec
				__alloc_pages_nodemask+0x1c0/0x10b8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc+0x4c/0x264
				qmi_encode_and_send_req+0x158/0x36c
				qmi_send_req_wait+0x64/0x2a4
				qmi_filter_request_send+0x19c/0x1c8
				wan_ioctl+0x1e0/0xac4
				do_vfs_ioctl+0x48c/0x564
				SyS_ioctl+0x60/0x88
				cpu_switch_to+0x250/0x420

^RSC enter bigorder wait  adj -1000 idx 0 ord 3 gfp  24152c1               ip 14272 <-          netmgrd  1195  1195 p ffffffbdc0154800 cal alloc_kmem_pages (-)
				Call trace:
				dump_backtrace+0x0/0x194
				show_stack+0x14/0x1c
				dump_stack+0x8c/0xac
				rsc_bigorder_monitor+0xbb0/0xbec
				__alloc_pages_nodemask+0x1c0/0x10b8
				alloc_kmem_pages+0x18/0x20
				kmalloc_order+0x28/0x8c
				kmalloc_order_trace+0x28/0xec
				__kmalloc_track_caller+0x4c/0x264
				__alloc_skb+0x98/0x198
				netlink_dump+0x9c/0x24c
				netlink_recvmsg+0x1b4/0x324
				sock_recvmsg+0x44/0x54
				___sys_recvmsg+0xe4/0x1b4
				__sys_recvmsg+0x40/0x64
				SyS_recvmsg+0x1c/0x24
				cpu_switch_to+0x250/0x420
*/

				rsc_get_pcomm(current, comm, &ppid, &tgid, pcomm);
				/*when order is 3, ingore native process and kernel thread, parent is not zygote,*/
				if (order == PAGE_ALLOC_COSTLY_ORDER &&
					((ppid == 1) || (ppid == 2) ||
					(strncmp(pcomm, "main", TASK_COMM_LEN))
					))
					return;

#if 0
				buffer[0] = '\0';
				sprint_symbol_no_offset(buffer, (unsigned long)ret_ip);
#endif
				/*save backtrace*/
				callstack.entries = stack_entries;
				callstack.nr_entries = 0;
				callstack.max_entries = ARRAY_SIZE(stack_entries);
				callstack.skip = 7;
				save_stack_trace(&callstack);
#if 0
				for (j = 0; j < callstack.nr_entries; j++) {
					if (stack_entries[j] == ULONG_MAX)
						break;
					rsc_info("%2d)	%ps\n",	j, (void *)stack_entries[j]);
				}
#endif
				for (j = 0; j < callstack.nr_entries; j++) {
					if (stack_entries[j] == ULONG_MAX)
						break;
					/*rsc_info("%2d)	%ps\n",	j, (void *)stack_entries[j]);*/
					buffer[0] = '\0';
					sprint_symbol_no_offset(buffer, (unsigned long)stack_entries[j]);
					for (i = 0; i < ARRAY_SIZE(rsc_skip_func); i++)
						if (!strcmp(buffer, rsc_skip_func[i])) {
							if (!rsc_skip_func_print[i]) {
								rsc_skip_func_print[i] = 1;
								print = PRINT_FUNC_BASE+i;
							} else
								print = 0;
							goto find;
						}
				}
				print = 1;
find:
				if (print) {
						rsc_info("enter bigorder %s adj %5d idx %d ord %u "
							"gfp %8x %16s %5d <- %16s %5d %5d p %p "
#if 0
						"caller %s "
#endif
						"cal %ps (%s)\n",
						iswait?"wait ":"kswap", current->signal->oom_score_adj, classzone_idx, order, gfp_mask,
						current->comm, current->pid, comm, (current->pid == tgid)?ppid:tgid, ppid, page,
#if 0
						buffer,
#endif
						ret_ip, print >= PRINT_FUNC_BASE?rsc_skip_func[print-PRINT_FUNC_BASE]:"-");
					if (print == 1)
						dump_stack();
				}
			}
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 4, 0)
		}
#else
		}
#endif
	}
}

extern atomic_long_t rsc_slowpath_count[];
extern u64 rsc_slowpath_time[];
extern int rsc_slowpath_print;
extern int rsc_slowpath_detail;
extern int rsc_slowpath_group;
#ifdef CONFIG_ZRAM
extern atomic_t rsc_zram_write_cnt;
#endif

#define RSC_ORDER_LIMIT 5

static inline void rsc_save_task_info(struct rsc_alloc_stat_t *atime,
	char *comm_p, pid_t *ppid_p, pid_t *tgid_p, kuid_t uid, unsigned int ord, u32 group)
{
	rsc_get_pcomm(current, comm_p, ppid_p, tgid_p, NULL);
	atime->inf.uid = (u32)uid.val;
	atime->adj = current->signal->oom_score_adj;
	atime->morder = ord;
	atime->inf.pid = current->pid;
	strlcpy(atime->inf.comm, current->comm, TASK_COMM_LEN);
	atime->inf.tgid = *tgid_p;
	atime->inf.ppid = *ppid_p;
	atime->group = group;
	strlcpy(atime->inf.tgcomm, comm_p, TASK_COMM_LEN);
}

/*FIX ME: surface flinger  belong default group and system back group?*/
/*
 * rsc_slowpath_monitor -record slowpath allocate.
 * @order: page_order
 *
 */
static inline void rsc_slowpath_monitor(unsigned int order, gfp_t gfp_mask, u64 ct, u64 rt,
	enum zone_type classzone_idx, struct page *page, unsigned long retrycnt, u64 exetime)
{
	char comm[TASK_COMM_LEN];
	pid_t ppid;
	pid_t tgid;
	char *comm_p;
	pid_t *ppid_p;
	pid_t *tgid_p;
	char *str;
	unsigned int ord;
	u64 ttime;
	kuid_t uid;
	struct rsc_alloc_stat_t *atime;
	/*struct timeval dtime;*/
	struct timespec64 dtime;
	u32 group;
	struct rsc_seq_file_t *seq;
	unsigned long s_flags;

/*
	if (unlikely(!rsc_mem_mon_enable))
		return;
*/

	ttime = ct + rt;
#if 1
	/*skip < 1ms*/
	if (
#ifdef CONFIG_RSC_MEM_MON_DEBUG
		page &&
#endif
		likely(ttime < mem_record_ns)) {
		/*if (ttime)
			rsc_info("%16s %5d slowpathdebug ttime %llu < mem_record_ns: %d ct %llu rt %llu\n",
				current->comm, current->pid, ttime, mem_record_ns, ct, rt);*/
		return;
	}
#else
	/*donot enter reclaim, but enter compact, skill compact time < 2ms*/
	if (!rt && (ct < RSC_MS_TO_NS(2)))
		return;
#endif
	ttime = RSC_NS_TO_US(ttime);
#ifdef CONFIG_RSC_MEM_PRIORITY
	group = current_thread_info()->rsc_group & ~RSC_CGROUP_SET_BIT;
#else
	group = rsc_task_group(current);
	if (unlikely(group >= RSC_CGROUP_END))
		group = RSC_CGROUP_TOPAPP;
	current_thread_info()->rsc_group = group | RSC_CGROUP_SET_BIT;
#endif
	seq = (struct rsc_seq_file_t *)current_thread_info()->rsc_seq;

	if (likely(seq)) {
		comm_p = seq->tgcomm;
		ppid_p = &seq->ppid;
		tgid_p = &seq->tgid;
	} else {
		comm_p = comm;
		ppid_p = &ppid;
		tgid_p = &tgid;
	}
	*ppid_p = -1;

	ord = order;
	/*sum up all the order larger than 4-order as one*/
	if (order > RSC_ORDER_LIMIT - 1)
		order = RSC_ORDER_LIMIT - 1;

	getnstimeofday64_nolock(&dtime);
	spin_lock_irqsave(&rsc_slowpath_spinlock, s_flags);
	atomic_long_inc(&rsc_slowpath_count[order]);
	if (ttime > rsc_slowpath_time[order]) {
		str = "almax";
		rsc_slowpath_time[order] = ttime;
	} else
		str = "alloc";

	uid = current_uid();
	atime = current_rsc_alloc();
	if (unlikely(!atime)) {
		rsc_err("rsc_alloc NULL  slowpath uid: %d %16s %5d\n",
			uid.val, current->comm, current->pid);
		atime = find_alloc_slot((u32)uid.val);
	}
/*
	rsc_info("RSCSET tcnt: %p  rsc_app_cnt: %p %p slow %d %16s %5d\n",
		atime->tcnt, &rsc_app_cnt, &rsc_1000_system_cnt,
		uid.val, current->comm, current->pid);
*/
	atomic_long_inc(atime->tcnt);

	atime->time += (u32)ttime;
	atomic_inc(&atime->cnt);
	atime->slowcnt[group]++;
	atomic_add((1 << ord), &atime->sorder);
	if (group == RSC_CGROUP_TOPAPP) {
		atime->last_tv_sec = (u32)dtime.tv_sec;
		/*reset the tv_sec of backgroup task*/
		if (unlikely(!atime->fg_time)) {
			atime->tv_sec = 0;
			atime->record_tv_sec = 0;
		}
		atime->fg_time += (u32)ttime;
		if ((atime->record_tv_sec + mem_lastfg_backtime) >= dtime.tv_sec) {
			atime->record_fg_time += (u32)ttime;
			atime->record_page += (1 << ord);
			atime->record_cnt++;
		} else {
			atime->record_tv_sec = (u32)dtime.tv_sec;
			atime->record_fg_time = (u32)ttime;
			atime->record_page = (1 << ord);
			atime->record_cnt = 1;
		}
		atime->maxtime = max((u32)ttime, atime->maxtime);
		if (unlikely(ttime >= rsc_slowpath_detail)) {
			rsc_save_task_info(atime, comm_p, ppid_p, tgid_p, uid, ord, group);
			atime->tv_sec = (u32)dtime.tv_sec;
		}

		if (unlikely(ttime > atime->fg_maxtime)) {
			atime->fg_maxtime = (u32)ttime;
			spin_unlock_irqrestore(&rsc_slowpath_spinlock, s_flags);
			/*maybe there is race condition, but for performance it is worth*/
			rsc_save_task_info(atime, comm_p, ppid_p, tgid_p, uid, ord, group);
			atime->max_tv_sec = (u32)dtime.tv_sec;
		} else
			spin_unlock_irqrestore(&rsc_slowpath_spinlock, s_flags);
	} else {
		if (!atime->slowcnt[RSC_CGROUP_TOPAPP]) {
			atime->last_tv_sec = (u32)dtime.tv_sec;
			if (unlikely(ttime >= rsc_slowpath_detail)) {
				rsc_save_task_info(atime, comm_p, ppid_p, tgid_p, uid, ord, group);
				atime->tv_sec = (u32)dtime.tv_sec;
			}
			if ((atime->record_tv_sec + mem_lastfg_backtime) >= dtime.tv_sec) {
				atime->record_fg_time += (u32)ttime;
				atime->record_page += (1 << ord);
				atime->record_cnt++;
			} else {
				atime->record_tv_sec = (u32)dtime.tv_sec;
				atime->record_fg_time = (u32)ttime;
				atime->record_page = (1 << ord);
				atime->record_cnt = 1;
			}
		}
		if (unlikely(ttime > atime->maxtime)) {
			atime->maxtime = (u32)ttime;
			spin_unlock_irqrestore(&rsc_slowpath_spinlock, s_flags);
			if (!atime->slowcnt[RSC_CGROUP_TOPAPP]) {
				/*maybe there is race condition, but for performance it is worth*/
				rsc_save_task_info(atime, comm_p, ppid_p, tgid_p, uid, ord, group);
				atime->max_tv_sec = (u32)dtime.tv_sec;
			}
		} else
			spin_unlock_irqrestore(&rsc_slowpath_spinlock, s_flags);
	}

	if (rsc_mem_logmask & MEM_REC_SLOWPATH) {
		u32 wait_time;
		u32 iso_time;
		u32 io_time;

		if (seq) {
			wait_time = jiffies_to_usecs(seq->rt.rsc_wait);
			if (wait_time)/*sub the error range*/
				wait_time -= jiffies_to_usecs(1)/2;
			iso_time = jiffies_to_usecs(seq->rt.iso_wait);
			io_time = jiffies_to_usecs(seq->rt.io_wait);
			seq->pid = current->pid;
			strlcpy(seq->comm, current->comm, TASK_COMM_LEN);
			seq->cost_us = (u32)ttime;
			seq->order = ord;
			seq->group = group;
#ifdef CONFIG_TASK_DELAY_ACCT
			if (current->delays) {
				seq->dl.blkio_delay = current->delays->blkio_delay/1000 - seq->dl.blkio_delay;
				seq->dl.swapin_delay = current->delays->swapin_delay/1000 - seq->dl.swapin_delay;
				seq->dl.blkio_count = current->delays->blkio_count - seq->dl.blkio_count;
				seq->dl.swapin_count = current->delays->swapin_count - seq->dl.swapin_count;
				seq->dl.freepages_delay = current->delays->freepages_delay/1000 - seq->dl.freepages_delay;
				seq->dl.freepages_count = current->delays->freepages_count - seq->dl.freepages_count;
			}
#endif
		} else {
			wait_time = 0;
			iso_time = 0;
			io_time = 0;
		}

		if (*ppid_p == -1)
			rsc_get_pcomm(current, comm_p, ppid_p, tgid_p, NULL);

		if (
#ifdef CONFIG_RSC_MEM_MON_DEBUG
		/*
		* RSC_MEM_PRI_SLOWPATH_STRATEGY
		* skip to print log for noretry alloc
		*/
		(!page && !(gfp_mask & __GFP_NORETRY)) ||
#endif
		(group <= RSC_CGROUP_TOPAPP && ttime >= rsc_slowpath_print) || (ttime >= 5*rsc_slowpath_print)) {
#ifdef CONFIG_RSC_FAST_PAGERECLAIM
			rsc_fast_info(
#else
			rsc_info(
#endif
				"%16s %5d <- %16s %5d %5d uid %6u enter slowpath %s, cost %7llu exe %6llu "
					"( %5llu + %6llu %5u %6u %5u %6u (s %5u z %5u) %5u rw %6d iw %d %d "
					"zram %5u %2u %5u %2u i %5u %2u %5u %2u z %5u %2u %5u %2u) us. "
					"adj %5d ord %u gfp %7x idx %d g %d p%d log %4u %4u cnt %lu\n"
#ifdef CONFIG_TASK_DELAY_ACCT
					"|RSC io (%5u %2u) swap (%5u %2u) free (%6u %2u)"
#endif
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
					" load %2d %2d"
#endif
#if defined(CONFIG_TASK_DELAY_ACCT) || defined(RSC_ZRAM_FAST_LOAD_CHECK)
					"\n"
#endif
					,
					current->comm, current->pid, comm_p, (current->pid == *tgid_p) ? *ppid_p : *tgid_p,
					*ppid_p, (u32)uid.val, str, ttime, (current->se.sum_exec_runtime - exetime)/1000,
					ct/1000, rt/1000, seq?seq->rt.shrink_lru[0] : 0,
					seq ? seq->rt.shrink_lru[1] : 0, seq ? seq->rt.shrink_slab[0] : 0, seq ? seq->rt.shrink_slab[1] : 0,
					seq ? seq->rt.shrink_super_slab/100 : 0, seq ? seq->rt.shrink_zcache_slab/100 : 0,
					seq ? seq->rt.shrink_lmk : 0, wait_time, iso_time, io_time,
					seq ? seq->rt.shrink_zram[0]/100 : 0, seq ? seq->rt.shrink_zram_cnt[0] : 0,
					seq ? seq->rt.shrink_zram[1]/100 : 0, seq ? seq->rt.shrink_zram_cnt[1] : 0,
					seq ? seq->rt.shrink_io[0]/100 : 0, seq ? seq->rt.shrink_io_cnt[0] : 0,
					seq ? seq->rt.shrink_io[1]/100 : 0, seq ? seq->rt.shrink_io_cnt[1] : 0,
					seq ? seq->rt.shrink_zcache[0]/100 : 0, seq ? seq->rt.shrink_zcache_cnt[0] : 0,
					seq ? seq->rt.shrink_zcache[1]/100 : 0, seq ? seq->rt.shrink_zcache_cnt[1] : 0,
					current->signal->oom_score_adj,
					ord, gfp_mask, classzone_idx, group, page?1:0, seq ? seq->count : -1, seq ? seq->point : -1, retrycnt
#ifdef CONFIG_TASK_DELAY_ACCT
					, seq ? seq->dl.blkio_delay : 0, seq ? seq->dl.blkio_count : 0
					, seq ? seq->dl.swapin_delay : 0, seq ? seq->dl.swapin_count : 0
					, seq ? seq->dl.freepages_delay : 0, seq ? seq->dl.freepages_count : 0
#endif
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
#if defined(CONFIG_MTK_PLATFORM)
					, min(100, atomic_read(&rsc_zram_load_val[0])*100/((int)RSC_ZRAM_TICK_CNT[0] * NR_CPUS))
					, min(100, atomic_read(&rsc_zram_load_val[1])*100/((int)RSC_ZRAM_TICK_CNT[1] * NR_CPUS))

#else
					, min(100, atomic_read(&rsc_zram_load_val[0])*100/((int)RSC_ZRAM_TICK_CNT * NR_CPUS))
					, min(100, atomic_read(&rsc_zram_load_val[1])*100/((int)RSC_ZRAM_TICK_CNT * NR_CPUS))
#endif
#endif
					);
			if (
#ifdef CONFIG_RSC_MEM_MON_DEBUG
				/*
				* RSC_MEM_PRI_SLOWPATH_STRATEGY
				* skip to print log for noretry alloc
				*/
				(!page && !(gfp_mask & __GFP_NORETRY)) ||
#endif
				(ttime >= (rsc_slowpath_detail+wait_time) && group <= rsc_slowpath_group) ||
				(ttime >= rsc_slowpath_detail && group <= RSC_CGROUP_TOPAPP) ||
				rsc_slowpath_detail <= 0) {
				rsc_print_mem_buf(seq, group);
#ifdef CONFIG_RSC_FAST_PAGERECLAIM
				rsc_fast_info(
#else
				rsc_info(
#endif
					"%s %d ======\n", current->comm, current->pid);
			}
		}

		if (likely(seq)) {
#ifndef RSC_MEM_RING_BUFF_ENABLE
			if (seq->count >= (RSC_MAX_LOG_BUF_SIZE - RSC_RESERVE_LOG_BUF_SIZE))
				seq->count = (RSC_MAX_LOG_BUF_SIZE - RSC_RESERVE_LOG_BUF_SIZE);
#endif
			/*size is about 340*/
			rsc_mem_force_printf("%16s %5d <- %16s %5d %5d uid %6u enter slowpath %s, cost %7llu exe %6llu "
					"( %5llu + %6llu %5u %6u %5u %6u (s %5u z %5u) %5u rw %6d iw %d %d "
					"zram %5u %2u %5u %2u i %5u %2u %5u %2u z %5u %2u %5u %2u) us. "
					"adj %5d ord %u gfp %7x idx %d g %d p%d log %4u %4u cnt %lu\n"
#ifdef CONFIG_TASK_DELAY_ACCT
					"|RSC io (%5u %2u) swap (%5u %2u) free (%6u %2u)"
#endif
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
					" load %2d %2d"
#endif
#if defined(CONFIG_TASK_DELAY_ACCT) || defined(RSC_ZRAM_FAST_LOAD_CHECK)
					"\n"
#endif
					,
					current->comm, current->pid, comm_p, (current->pid == *tgid_p) ? *ppid_p : *tgid_p,
					*ppid_p, (u32)uid.val, str, ttime, (current->se.sum_exec_runtime - exetime)/1000,
					ct/1000, rt/1000, seq?seq->rt.shrink_lru[0] : 0,
					seq ? seq->rt.shrink_lru[1] : 0, seq ? seq->rt.shrink_slab[0] : 0, seq ? seq->rt.shrink_slab[1] : 0,
					seq ? seq->rt.shrink_super_slab/100 : 0, seq ? seq->rt.shrink_zcache_slab/100 : 0,
					seq ? seq->rt.shrink_lmk : 0, wait_time, iso_time, io_time,
					seq ? seq->rt.shrink_zram[0]/100 : 0, seq ? seq->rt.shrink_zram_cnt[0] : 0,
					seq ? seq->rt.shrink_zram[1]/100 : 0, seq ? seq->rt.shrink_zram_cnt[1] : 0,
					seq ? seq->rt.shrink_io[0]/100 : 0, seq ? seq->rt.shrink_io_cnt[0] : 0,
					seq ? seq->rt.shrink_io[1]/100 : 0, seq ? seq->rt.shrink_io_cnt[1] : 0,
					seq ? seq->rt.shrink_zcache[0]/100 : 0, seq ? seq->rt.shrink_zcache_cnt[0] : 0,
					seq ? seq->rt.shrink_zcache[1]/100 : 0, seq ? seq->rt.shrink_zcache_cnt[1] : 0,
					current->signal->oom_score_adj,
					ord, gfp_mask, classzone_idx, group, page?1:0, seq ? seq->count : -1, seq ? seq->point : -1, retrycnt
#ifdef CONFIG_TASK_DELAY_ACCT
					, seq ? seq->dl.blkio_delay : 0, seq ? seq->dl.blkio_count : 0
					, seq ? seq->dl.swapin_delay : 0, seq ? seq->dl.swapin_count : 0
					, seq ? seq->dl.freepages_delay : 0, seq ? seq->dl.freepages_count : 0
#endif
#ifdef RSC_ZRAM_FAST_LOAD_CHECK
#if defined(CONFIG_MTK_PLATFORM)
			, min(100, atomic_read(&rsc_zram_load_val[0])*100/((int)RSC_ZRAM_TICK_CNT[0] * NR_CPUS))
			, min(100, atomic_read(&rsc_zram_load_val[1])*100/((int)RSC_ZRAM_TICK_CNT[1] * NR_CPUS))
					
#else
			, min(100, atomic_read(&rsc_zram_load_val[0])*100/((int)RSC_ZRAM_TICK_CNT * NR_CPUS))
			, min(100, atomic_read(&rsc_zram_load_val[1])*100/((int)RSC_ZRAM_TICK_CNT * NR_CPUS))
#endif

#endif
					);
		}
	}
}

#ifdef CONFIG_RSC_MEM_PRIORITY
#include <linux/swap.h>

extern void rsc_wakeup_kswapd(gfp_t gfp_mask,
	unsigned int order, int orgfree, int curfree);
extern int min_free_kbytes;
extern int user_min_free_kbytes;
extern int extra_free_kbytes;

extern atomic_t rsc_cgroup_cnt[RSC_CGROUP_END+1];
extern wait_queue_head_t rsc_group_wait[RSC_CGROUP_END];
extern u32 __read_mostly rsc_mem_slowpath_wait;
extern u32 __read_mostly rsc_mem_pri_enable;

/*#define RSC_DEBUG_SLOWPATH_CNT*/
/*fix me , need adjust*/
#define RSC_MEM_WAIT_MS 100
static inline int rsc_need_wait(u32 group, int *cnt)
{
#if 1
	*cnt = atomic_read(&rsc_cgroup_cnt[group]);
#else
	int i;

	*cnt = 0;
	for (i = 0; i <= group; i++)
		*cnt += atomic_read(&rsc_cgroup_cnt[i]);
#endif
	if (*cnt)
		return 1;
	else
		return 0;
}

/*maybe the reason is that could not use define in define!!*/
#define FIX_PRINT_OOPS
#define RSC_WAITUP_FORMAT 																\
	"%s %d %s wall g %d cnt %d adj %5d\n", current->comm, current->pid, step,			\
		group, atomic_read(&rsc_cgroup_cnt[group+1]), current->signal->oom_score_adj

#define RSC_WAITEVENT_FORMAT 															\
	"%s %d slowpath wevt g %d cnt %d adj %5d\n", current->comm, current->pid,			\
		group, cnt, current->signal->oom_score_adj

#define RSC_EXITEVENT_FORMAT 															\
	"%s %d slowpath eevt g %d cnt %d adj %5d %ld\n", current->comm, current->pid,		\
		group, cnt, current->signal->oom_score_adj, ret

static inline void rsc_slowwait_enter(int group)
{
	if (current_is_kswapd())
		return;

	atomic_inc(&rsc_cgroup_cnt[group]);
	/*
	if (!rsc_mem_slowpath_wait)
		return;
	*/
}

static inline void rsc_slowwait_action(int group, int curg)
{
	long ret;
	int i;
	int cnt, c;
	int wgrp;
	/*u32 curg;*/

	if (current_is_kswapd())
		return;

	/*curg = rsc_task_group(current);*/
	if (unlikely(curg >= RSC_CGROUP_END))
		curg = RSC_CGROUP_TOPAPP;
	/*group change , wake up thread*/
	if (curg != group) {
		if (!atomic_dec_return(&rsc_cgroup_cnt[group])) {
			if (waitqueue_active(&rsc_group_wait[group])) {
				wake_up_all(&rsc_group_wait[group]);
				rsc_mem_force_printf("%s %d slowpath wall gchange %d -> %d cnt %d adj %5d\n",
					current->comm, current->pid, group, curg,
					atomic_read(&rsc_cgroup_cnt[group+1]),
					current->signal->oom_score_adj);
				rsc_info("warning %s %d slowpath wall gchange %d -> %d cnt %d adj %5d\n",
					current->comm, current->pid, group, curg,
					atomic_read(&rsc_cgroup_cnt[group+1]),
					current->signal->oom_score_adj);
			}
		}
		rsc_info("warning %s %d gchange %d -> %d cnt %d!\n", current->comm, current->pid,
			group, curg, atomic_read(&rsc_cgroup_cnt[group]));
#ifdef RSC_DEBUG_SLOWPATH_CNT
		if (atomic_read(&rsc_cgroup_cnt[group]) < 0)
			rsc_err("%s %d gchange %d -> %d cnt wrong %d!\n", current->comm, current->pid,
			group, curg, atomic_read(&rsc_cgroup_cnt[group]));
#endif
		current_thread_info()->rsc_group = curg | RSC_CGROUP_SET_BIT;
		group = curg;
		atomic_inc(&rsc_cgroup_cnt[group]);
	}

	if (!rsc_mem_slowpath_wait)
		return;

	/*only limit backgroup task!!!*/
	if (!is_background(group))
		return;
	cnt = 0;
	wgrp = 0;
	for (i = 0; i < group; i++) {
		c = atomic_read(&rsc_cgroup_cnt[i]);
		if (c) {
			wgrp = i;
			cnt += c;
		}
#ifdef RSC_DEBUG_SLOWPATH_CNT
		if (c < 0)
			rsc_err("%s %d prewait g %d cnt wrong %d!\n", current->comm, current->pid,
				i, c);
#endif
	}
/*
	printk("RSC TST group %d cnt %d wgrp %d %d %d %d %d %d\n", group, cnt, wgrp,
		atomic_read(&rsc_cgroup_cnt[0]),
		atomic_read(&rsc_cgroup_cnt[1]),
		atomic_read(&rsc_cgroup_cnt[2]),
		atomic_read(&rsc_cgroup_cnt[3]),
		atomic_read(&rsc_cgroup_cnt[4])
		);
*/
	if (cnt) {
		/*
		 * If a fatal signal is pending, this process should not throttle.
		 * It should return quickly so it can exit and free its memory
		 */
		if (fatal_signal_pending(current))
			return;
#ifdef FIX_PRINT_OOPS
		rsc_mem_force_printf("%s %d slowpath wevt g %d wg %d cnt %d adj %5d\n", current->comm, current->pid,
			group, wgrp, cnt, current->signal->oom_score_adj);
#else
		rsc_mem_force_printf(RSC_WAITEVENT_FORMAT);
#endif
		/*printk("RSC TST "RSC_WAITEVENT_FORMAT);*/

		ret = wait_event_interruptible_timeout(rsc_group_wait[wgrp],
			!rsc_need_wait(wgrp, &cnt), msecs_to_jiffies(RSC_MEM_WAIT_MS));
		if (ret >= 0 && ret < msecs_to_jiffies(RSC_MEM_WAIT_MS)) {
			struct rsc_seq_file_t *m = (struct rsc_seq_file_t *)current_thread_info()->rsc_seq;
			if (m)
				m->rt.rsc_wait += msecs_to_jiffies(RSC_MEM_WAIT_MS) - ret;
		}
#ifdef FIX_PRINT_OOPS
		rsc_mem_force_printf("%s %d slowpath eevt g %d cnt %d adj %5d %ld\n", current->comm, current->pid,
			group, cnt, current->signal->oom_score_adj, ret);
#else
		rsc_mem_force_printf(RSC_EXITEVENT_FORMAT);
#endif
		/*printk("RSC TST "RSC_EXITEVENT_FORMAT);*/
	}
#if 0
	else
		printk("RSC TST %s %d slowpath cnt zero! g %d cnt %d adj %5d\n", current->comm, current->pid,
			group, cnt, current->signal->oom_score_adj);
#endif

}

static inline void rsc_slowwait_exit(int group)
{
	if (current_is_kswapd())
		return;

	if (!atomic_dec_return(&rsc_cgroup_cnt[group])) {

		if (!rsc_mem_slowpath_wait)
			return;

		if (waitqueue_active(&rsc_group_wait[group])) {
			wake_up_all(&rsc_group_wait[group]);

#ifdef FIX_PRINT_OOPS
			rsc_mem_force_printf("%s %d slowpath wall g %d cnt %d adj %5d\n",
				current->comm, current->pid, group,
				atomic_read(&rsc_cgroup_cnt[group+1]),
				current->signal->oom_score_adj);
#else
			rsc_mem_force_printf(RSC_WAITUP_FORMAT);
#endif
			/*printk("RSC TST "RSC_WAITUP_FORMAT);*/
		}
	}
#ifdef RSC_DEBUG_SLOWPATH_CNT
	if (atomic_read(&rsc_cgroup_cnt[group]) < 0)
		rsc_err("%s %d exit g %d cnt wrong %d!\n", current->comm, current->pid,
			group, atomic_read(&rsc_cgroup_cnt[group]));
#endif
}

#define RSC_MEM_PRI_SLOWPATH_STRATEGY
#ifdef RSC_MEM_PRI_SLOWPATH_STRATEGY
extern u32 __read_mostly rsc_mem_pri_slowpath_max_cnt;
/*background task warter mark ALLOC_WMARK_LOW level try count*/
extern u16 __read_mostly rsc_mem_pri_slowpath_bg_try_cnt;
/*default task warter mark ALLOC_WMARK_RBACK level try count*/
extern u16 __read_mostly rsc_mem_pri_slowpath_def_try_cnt;
#endif

#endif
#ifdef __cplusplus
}
#endif

#endif /*__RSC_MEM_MON_H__*/
