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

#define CPU_TOP_GET_CMDLINE
#define CPU_TOP_ONLY_RECORD_USAGE_EXIT_TASK

#define PID_HASH_BITS	11

#ifdef CPU_TOP_GET_CMDLINE
#define PROCESS_NAME_LEN	47
#else
#define PROCESS_NAME_LEN	TASK_COMM_LEN
#endif

#define RSC_TASK_EXIT_BIT 7

#ifndef nsecs_to_cputime
#define nsecs_to_cputime(axu) axu
#endif

static DECLARE_HASHTABLE(hash_table, PID_HASH_BITS);

static DEFINE_MUTEX(pid_lock);

struct tpid_entry {
	u64 utime;
	u64 stime;
	struct hlist_node hash;
	struct list_head link;
	u64 jiffs;
	u32 sum_exe;
	pid_t tid;
	pid_t pid;
	pid_t ppid;
	uid_t uid;
	char comm[PROCESS_NAME_LEN];
	char tname[TASK_COMM_LEN];
	u8 task_exit;
};
#if 0
struct rsc_cpu_top {
	u64 utime;
	u64 stime;
	pid_t pid;/*pid or prcess count*/
	char comm[TASK_COMM_LEN];
} __packed __aligned(4);
#endif
#define RSC_MAX_RECORD_PID 64
static int rsc_total_pid;
static LIST_HEAD(tpid_entry_head);

static struct tpid_entry top_tentry[RSC_MAX_RECORD_PID];

static struct tpid_entry *rsc_find_pid_entry(pid_t pid)
{
	struct tpid_entry *tpid_entry;
	hash_for_each_possible(hash_table, tpid_entry, hash, pid) {
		if (tpid_entry->pid == pid)
			return tpid_entry;
	}
	return NULL;
}

static struct tpid_entry *rsc_find_or_register_pid(pid_t pid,
	struct task_struct *pt, struct task_struct *task)
{
	struct tpid_entry *tpid_entry;
	char *comm;
    char app_name[PROCESS_NAME_LEN];
	int res;

	tpid_entry = rsc_find_pid_entry(pid);
	if (tpid_entry) {
		list_move_tail(&tpid_entry->link, &tpid_entry_head);
#if 1
		rsc_dbg(CPU_TOP,
#else
		rsc_info(
#endif
			"%s: found move to tail! comm: %16s(%d) uid: %d ppid: %d add pid: %d\n",
				__func__, tpid_entry->comm, tpid_entry->pid,
				tpid_entry->uid, tpid_entry->ppid, pid);
		return tpid_entry;
	}

	if (rsc_total_pid < RSC_MAX_RECORD_PID) {
		tpid_entry = &top_tentry[rsc_total_pid];
		list_add_tail(&tpid_entry->link, &tpid_entry_head);
		rsc_total_pid++;
	} else {
		tpid_entry = list_first_entry_or_null(&tpid_entry_head,
			struct tpid_entry, link);
		if (tpid_entry) {
#if 0
			struct tpid_entry *tp, *tmp1;
			int j = 0;
			list_for_each_entry(tp, &tpid_entry_head, link) {
				rsc_info("%s: %d pid: %d xx1tpid_entry_head.next:%p "
					"prev: %p\n",
				__func__, j, tp->pid, tp->link.next, tp->link.prev);
				j++;
			}
#endif
#if 1
			rsc_dbg(CPU_TOP,
#else
			rsc_info(
#endif
				"%s: notfound del comm: %16s(%d) uid: %d ppid: %d add pid: %d\n",
				__func__, tpid_entry->comm, tpid_entry->pid,
				tpid_entry->uid, tpid_entry->ppid, pid);
			hash_del(&tpid_entry->hash);
			list_move_tail(&tpid_entry->link, &tpid_entry_head);
		} else
			return NULL;
	}
	tpid_entry->utime = 0;
	tpid_entry->stime = 0;
	tpid_entry->sum_exe = 0;

	comm = pt->comm;
#ifdef CPU_TOP_GET_CMDLINE
	if (!(pt->flags & PF_KTHREAD)) {
		res = get_cmdline(pt, app_name, sizeof(app_name));
		if (res > 0) {
			if (res < sizeof(app_name))
				app_name[res] = 0;
			else
				app_name[sizeof(app_name)-1] = 0;
			comm = app_name;
		}
	}
#endif
	strlcpy(tpid_entry->comm, comm, sizeof(tpid_entry->comm));
	if (pt->real_parent)
		tpid_entry->ppid = pt->real_parent->pid;
	else
		tpid_entry->ppid = RSC_END_PID;
	tpid_entry->uid = from_kuid_munged(
				current_user_ns(), task_uid(pt));
	tpid_entry->pid = pid;
	tpid_entry->task_exit = 0;
/*
	tpid_entry->tid = task->pid;
	strlcpy(tpid_entry->tname, task->comm, sizeof(tpid_entry->tname));
*/
	hash_add(hash_table, &tpid_entry->hash, pid);

	return tpid_entry;
}

#ifdef CONFIG_64BIT
static inline u64 read_sum_exec_runtime(struct task_struct *t)
{
	return t->se.sum_exec_runtime;
}
#else
extern u64 read_sum_exec_runtime(struct task_struct *t);
/*
define in kernel/sched/core.c
u64 read_sum_exec_runtime(struct task_struct *t)
{
	u64 ns;
	unsigned long rf;
	struct rq *rq;

	rq = task_rq_lock(t, &rf);
	ns = t->se.sum_exec_runtime;
	task_rq_unlock(rq, t, &rf);

	return ns;
}
*/
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static void rsc_thread_group_cputime(struct task_struct *tsk,
		u64 *ut, u64 *st, unsigned long long *sum_exec_runtime)
#else
static void rsc_thread_group_cputime(struct task_struct *tsk,
	cputime_t *ut, cputime_t *st, unsigned long long *sum_exec_runtime)
#endif
{
	struct signal_struct *sig = tsk->signal;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	u64 utime, stime;
#else
	cputime_t utime, stime;
#endif
	struct task_struct *t;
	unsigned int seq, nextseq;
	unsigned long flags;
#if 0
	rcu_read_lock();
#endif
	/* Attempt a lockless read on the first round. */
	nextseq = 0;
	do {
		seq = nextseq;
		flags = read_seqbegin_or_lock_irqsave(&sig->stats_lock, &seq);
		*ut = sig->utime;
		*st = sig->stime;
		*sum_exec_runtime = sig->sum_sched_runtime;

		for_each_thread(tsk, t) {
			if (!(t->rsc_exit & RSC_CPU_TOP_EXIT)) {
				task_cputime(t, &utime, &stime);
				*ut += utime;
				*st += stime;
				*sum_exec_runtime += read_sum_exec_runtime(t);
			}
		}
		/* If lockless access failed, take the lock. */
		nextseq = 1;
	} while (need_seqretry(&sig->stats_lock, seq));
	done_seqretry_irqrestore(&sig->stats_lock, seq, flags);
#if 0
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
#endif
}

/*
 * Perform (stime * rtime) / total, but avoid multiplication overflow by
 * loosing precision when the numbers are big.
 */
static u64 scale_stime(u64 stime, u64 rtime, u64 total)
{
	u64 scaled;

	for (;;) {
		/* Make sure "rtime" is the bigger of stime/rtime */
		if (stime > rtime)
			swap(rtime, stime);

		/* Make sure 'total' fits in 32 bits */
		if (total >> 32)
			goto drop_precision;

		/* Does rtime (and thus stime) fit in 32 bits? */
		if (!(rtime >> 32))
			break;

		/* Can we just balance rtime/stime rather than dropping bits? */
		if (stime >> 31)
			goto drop_precision;

		/* We can grow stime and shrink rtime and try to make them both fit */
		stime <<= 1;
		rtime >>= 1;
		continue;

drop_precision:
		/* We drop from rtime, it has more bits than stime */
		rtime >>= 1;
		total >>= 1;
	}

	/*
	 * Make sure gcc understands that this is a 32x32->64 multiply,
	 * followed by a 64/32->64 divide.
	 */
	scaled = div_u64((u64) (u32) stime * (u64) (u32) rtime, (u32)total);
	return (__force u64) scaled;
}

static void rsc_cputime_adjust(struct task_cputime *curr,
/*workaround for msm8953 android 8.1*/
#if LINUX_VERSION_CODE == KERNEL_VERSION(3, 18, 71)
				struct cputime *prev,
#else
				struct prev_cputime *prev,
#endif
			   u64 *ut, u64 *st)
{
	u64 rtime, stime, utime;

#if LINUX_VERSION_CODE != KERNEL_VERSION(3, 18, 71)
	unsigned long flags;

	/* Serialize concurrent callers such that we can honour our guarantees */
	raw_spin_lock_irqsave(&prev->lock, flags);
#endif
#if 1
	rtime = curr->sum_exec_runtime;
#else
	rtime = nsecs_to_cputime(curr->sum_exec_runtime);
#endif
	/*
	 * This is possible under two circumstances:
	 *  - rtime isn't monotonic after all (a bug);
	 *  - we got reordered by the lock.
	 *
	 * In both cases this acts as a filter such that the rest of the code
	 * can assume it is monotonic regardless of anything else.
	 */
	if (prev->stime + prev->utime >= rtime)
		goto out;

	stime = curr->stime;
	utime = curr->utime;

	/*
	 * If either stime or both stime and utime are 0, assume all runtime is
	 * userspace. Once a task gets some ticks, the monotonicy code at
	 * 'update' will ensure things converge to the observed ratio.
	 */
	if (stime == 0) {
		utime = rtime;
		goto update;
	}

	if (utime == 0) {
		stime = rtime;
		goto update;
	}

	stime = scale_stime((__force u64)stime, (__force u64)rtime,
			    (__force u64)(stime + utime));

update:
	/*
	 * Make sure stime doesn't go backwards; this preserves monotonicity
	 * for utime because rtime is monotonic.
	 *
	 *  utime_i+1 = rtime_i+1 - stime_i
	 *            = rtime_i+1 - (rtime_i - utime_i)
	 *            = (rtime_i+1 - rtime_i) + utime_i
	 *            >= utime_i
	 */
	if (stime < prev->stime)
		stime = prev->stime;
	utime = rtime - stime;

	/*
	 * Make sure utime doesn't go backwards; this still preserves
	 * monotonicity for stime, analogous argument to above.
	 */
	if (utime < prev->utime) {
		utime = prev->utime;
		stime = rtime - utime;
	}

	prev->stime = stime;
	prev->utime = utime;
out:
	*ut = prev->utime;
	*st = prev->stime;
#if LINUX_VERSION_CODE != KERNEL_VERSION(3, 18, 71)
	raw_spin_unlock_irqrestore(&prev->lock, flags);
#endif
}

#define CPU_TOP_TASK_BATCHING 400
/*
 * To avoid extending the RCU grace period for an unbounded amount of time,
 * periodically exit the critical section and enter a new one.
 *
 * For preemptible RCU it is sufficient to call rcu_read_unlock in order
 * to exit the grace period. For classic RCU, a reschedule is required.
 * see rcu_lock_break in hung_task.c
 */
static bool rcu_mutex_break(struct task_struct *g)
{
	bool can_cont;

	get_task_struct(g);

#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	mutex_unlock(&pid_lock);
	cond_resched();
	mutex_lock(&pid_lock);
	rcu_read_lock();
	/*
	* p->pids[PIDTYPE_PID].pid is free in
	* do_exit -> exit_notify -> release_task -> __exit_signal ->
	* __unhash_process -> detach_pid(p, PIDTYPE_PID)
	* If pid_alive(g), we can make sure task g is not do_exit!!!
	*/
	can_cont = pid_alive(g);
	put_task_struct(g);

	return can_cont;
}

/*2^64 max length is 20*/
#define TOP_HEAD_FORMAT "%16s\t%5u\t%4u\t%5u\t%4u\t%20llu\t"	\
				"%20llu\t%8lu\ttim\t%20llu\t%8llu\t%8llu\n"
static int stat_top_show(struct seq_file *m, void *v, int type)
{
	struct task_struct *p, *relp;
	struct task_struct *free_task = NULL;
	struct tpid_entry *tpid_entry;
	char *comm;
	unsigned long flags;
	struct task_cputime inf;
	u64 utime, stime, ut, st;
	u64 time0, time1, time2;
	pid_t ppid;
	int pcount, acount;
	/*unsigned long bkt;*/
	int reccount, ecount;
	u8 tmpc = 0;
	int headcnt;
	u64 curjif, dif;
#ifdef CPU_TOP_GET_CMDLINE
    /*const struct cred *cred;*/
    char app_name[PROCESS_NAME_LEN];
	int res;
#endif
	int batch_count;

again:
	batch_count = CPU_TOP_TASK_BATCHING;
	pcount = 0;
	acount = 0;
	utime = 0;
	stime = 0;

	seq_printf(m, TOP_HEAD_FORMAT,
			 "STAT_TOP HEAD", 0, 0, 0, 0, (u64)0,
			 (u64)0, 0UL, (u64)0, (u64)0, (u64)0);
	headcnt = m->count;

	mutex_lock(&pid_lock);
	time0 = local_clock();
#ifdef CPU_TOP_GET_CMDLINE
	/*read_lock(&tasklist_lock);*/
	rcu_read_lock();
#else
	rcu_read_lock();
#endif
	for_each_process(p) {

#ifdef CPU_TOP_GET_CMDLINE
		if (free_task) {
			put_task_struct(free_task);
			free_task = NULL;
		}
#endif
		if (!--batch_count) {
			batch_count = CPU_TOP_TASK_BATCHING;
			if (!rcu_mutex_break(p)) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
				rcu_read_unlock_inline();
#else
				rcu_read_unlock();
#endif
				mutex_unlock(&pid_lock);
				/*reset count, fix KE*/
				m->count = 0;
				rsc_err("%s: rcu_mutex_break fail!\n",
					__func__);
				goto again;
			}
		}

		tpid_entry = rsc_find_pid_entry(p->pid);
		if (lock_task_sighand(p, &flags)) {
			#if 1
			rsc_thread_group_cputime(p, &inf.utime, &inf.stime,
				&inf.sum_exec_runtime);
			inf.sum_exec_runtime = nsecs_to_cputime(inf.sum_exec_runtime);
			if (tpid_entry)
				inf.sum_exec_runtime += tpid_entry->sum_exe;
			rsc_cputime_adjust(&inf, &p->signal->prev_cputime, &ut, &st);
			if (tpid_entry) {
				tpid_entry->task_exit |= (1 << RSC_TASK_EXIT_BIT);
				ut += tpid_entry->utime;
				st += tpid_entry->stime;
			}
			#else
			thread_group_cputime_adjusted(p, &inf.utime, &inf.stime);
			#endif
			utime += ut;
			stime += st;
			unlock_task_sighand(p, &flags);
		} else {
			rsc_err("%s: get lock_task_sighand error!\n", __func__);
			ut = 0;
			st = 0;
		}

		relp = p->real_parent;
		if (relp)
			ppid = relp->pid;
		else
			ppid = RSC_END_PID;
		acount++;

		if (ut || st)
			pcount++;
		else
			if (type != 0)
				continue;

#ifdef CPU_TOP_GET_CMDLINE
		get_task_struct(p);
		/*
		* when call get_cmdline, we could not get rcu_read_lock.
		* Because when we get rcu_read_lock, we could not sleep.
		* see get_cmdline -> down_read -> might_sleep -> ___might_sleep -> preempt_count_equals
		current->rcu_read_lock_nesting is not zero, when get rcu_read_lock
		static inline int preempt_count_equals(int preempt_offset)
		{
			int nested = preempt_count() + current->rcu_read_lock_nesting;

			return (nested == preempt_offset);
		}
		*/
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		comm = p->comm;
		if (!(p->flags & PF_KTHREAD)) {
			res = get_cmdline(p, app_name, sizeof(app_name));
			if (res > 0) {
				if (res < sizeof(app_name))
					app_name[res] = 0;
				else
					app_name[sizeof(app_name)-1] = 0;
				comm = app_name;
			}
		}
		rcu_read_lock();
		free_task = p;
		if (atomic_read(&p->usage) == 1)
			rsc_warn("%s: taskexit %16s %5d %16s %5d ready to exit\n", __func__, p->comm, p->pid,
				p->group_leader?p->group_leader->comm:"NULL",
				p->group_leader?p->group_leader->pid:99999);
		/*
		* we could not put_task_struct here or task_struct p will be free.
		*/
		/*put_task_struct(p);*/
#else
		comm = p->comm;
#endif
		seq_printf(m, "%u\t%llu\t%llu\t%s\t%d\t%d\n",
			p->pid, (u64)ut, (u64)st, comm,
			from_kuid_munged(
			current_user_ns(), task_uid(p)), ppid);
	}

#ifdef CPU_TOP_GET_CMDLINE
	if (free_task) {
		put_task_struct(free_task);
		free_task = NULL;
	}
	/*read_unlock(&tasklist_lock);*/
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
#else
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
#endif
	time1 = local_clock();
	reccount = 0;
	ecount = 0;
	curjif = get_jiffies_64();

	list_for_each_entry(tpid_entry, &tpid_entry_head, link) {
		reccount++;
	/*hash_for_each(hash_table, bkt, tpid_entry, hash) {*/
		if (!(tpid_entry->task_exit & (1 << RSC_TASK_EXIT_BIT))) {
			utime += tpid_entry->utime;
			stime += tpid_entry->stime;
			ecount++;
		}
		if ((!(tpid_entry->task_exit & (1 << RSC_TASK_EXIT_BIT))) ||
			(type == 0)) {
			/*(type == 0), show all for debug*/
			if (curjif >= tpid_entry->jiffs)
				dif = jiffies_to_msecs(curjif - tpid_entry->jiffs);
			else
				dif = jiffies_to_msecs((U64_MAX - tpid_entry->jiffs) + curjif);
			if (!(tpid_entry->task_exit & (1 << RSC_TASK_EXIT_BIT)))
				seq_printf(m, "%u\t%llu\t%llu\t%s\t%d\t%d\ttim\t%llu\t"
					"%u\t%s\t%d\n",
					tpid_entry->pid, (u64)tpid_entry->utime,
					(u64)tpid_entry->stime,	tpid_entry->comm,
					tpid_entry->uid, tpid_entry->ppid, dif,
					tpid_entry->tid, tpid_entry->tname,
					tpid_entry->task_exit&(~(1 << RSC_TASK_EXIT_BIT)));
			else
				seq_printf(m, "%u\t%llu\t%llu\t%s\t%d\t%d\ttim\t%llu\t"
					"%u\t%s\t%d\tMerge\n",
					tpid_entry->pid, (u64)tpid_entry->utime,
					(u64)tpid_entry->stime, tpid_entry->comm,
					tpid_entry->uid, tpid_entry->ppid, dif,
					tpid_entry->tid, tpid_entry->tname,
					tpid_entry->task_exit&(~(1 << RSC_TASK_EXIT_BIT)));
		}
		tpid_entry->task_exit &= ~(1 << RSC_TASK_EXIT_BIT);
	}
	time2 = local_clock();

	if (headcnt < m->size)
		tmpc = m->buf[headcnt];
	snprintf(m->buf, m->size, TOP_HEAD_FORMAT,
		 "STAT_TOP HEAD", pcount, ecount, acount, reccount,
		(u64)utime, (u64)stime, (unsigned long)m->count, curjif,
		(time1 - time0)/1000, (time2 - time1)/1000);
	if (headcnt < m->size)
		m->buf[headcnt] = tmpc;
	mutex_unlock(&pid_lock);
	rsc_dbg(CPU_TOP, "%s: T: %d m->size: %lu count: %lu sizeof(rsc_cpu_top): %u pcount: %d jiffs: %llu time:%llu %llu\n",
		__func__, type, (unsigned long)m->size, (unsigned long)m->count,  (unsigned int)sizeof(inf), pcount, get_jiffies_64(),
		(time1 - time0)/1000, (time2 - time1)/1000);

	return 0;
}

static int stat_cpu_show_all(struct seq_file *m, void *v)
{
	return stat_top_show(m, v, 0);
}

static int stat_top_open_all(struct inode *inode, struct file *file)
{
	/*force to allocate 64KB buffer, it can speed up!
	max 1000 process:  sizeof(struct rsc_cpu_top) x 1000 = 64 * 1000 = 64KB*/
	size_t size = 64 * 1024;
	int ret;

	ret = single_open_size(file, stat_cpu_show_all, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static int stat_cpu_show(struct seq_file *m, void *v)
{
	return stat_top_show(m, v, 1);
}

static int stat_top_open(struct inode *inode, struct file *file)
{
	/*force to allocate 32KB buffer, it can speed up!
	max 1000 process:  32 x 1000 = 32 * 1000 = 32KB*/
	size_t size = 32 * 1024;
	int ret;

	ret = single_open_size(file, stat_cpu_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
	((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations stat_top_all_fops = {
	.open		= stat_top_open_all,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static const struct file_operations stat_top_fops = {
	.open		= stat_top_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int rsc_top_exit;
static int top_exit_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", rsc_top_exit);
	return 0;
}

static int top_exit_open(struct inode *inode, struct file *file)
{
	size_t size = 4096;
	int ret;

	ret = single_open_size(file, top_exit_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
	((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static ssize_t top_exit_write(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret;
	int val;
	char kbuf[128];

	if (!count)
		return -EFAULT;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	kbuf[count] = '\0';
	ret = sscanf(kbuf, "%u", &val);
	if ((ret != 1) || val < 0 || val > 1)
		return -EINVAL;
	mutex_lock(&pid_lock);
	if (val) {
#ifndef CONFIG_RSC_UID_IO
		if (!rsc_top_exit)
			profile_event_register(PROFILE_TASK_EXIT, &top_process_notifier_block);
#endif
	} else {
#ifndef CONFIG_RSC_UID_IO
		if (rsc_top_exit)
			profile_event_unregister(PROFILE_TASK_EXIT, &top_process_notifier_block);
#endif
	}

	rsc_top_exit = val;
	mutex_unlock(&pid_lock);

	return count;
}

static const struct file_operations top_exit_fops = {
	.open		= top_exit_open,
	.read		= seq_read,
	.write 		= top_exit_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifndef CONFIG_RSC_UID_IO
static int rsc_top_process_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
#else
int rsc_top_process_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
#endif
{
	struct task_struct *task = v, *pt;
	struct tpid_entry *tpid_entry;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	u64 utime, stime;
#else
	cputime_t utime, stime;
#endif
	pid_t tpid;

	if (!rsc_top_exit)
		return NOTIFY_OK;

#ifndef CONFIG_RSC_UID_IO
	if (!task)
		return NOTIFY_OK;
#endif
	task_cputime_adjusted(task, &utime, &stime);

#ifdef CPU_TOP_ONLY_RECORD_USAGE_EXIT_TASK
	if (!utime && !stime)
		return NOTIFY_OK;
#endif

	mutex_lock(&pid_lock);

	rcu_read_lock();
	if (thread_group_leader(task))
		pt = task;
	else
		pt = rcu_dereference(task->group_leader);
	if (!pt) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		goto exit;
	}
	tpid = pt->pid;

	get_task_struct(pt);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	tpid_entry = rsc_find_or_register_pid(tpid, pt, task);
	put_task_struct(pt);
	if (!tpid_entry) {
		rsc_err("%s: failed to find pid %d\n", __func__, tpid);
		goto exit;
	}
	tpid_entry->utime += utime;
	tpid_entry->stime += stime;
	tpid_entry->sum_exe += nsecs_to_cputime(read_sum_exec_runtime(task));
	tpid_entry->jiffs = get_jiffies_64();
	tpid_entry->tid = task->pid;
	strlcpy(tpid_entry->tname, task->comm, sizeof(tpid_entry->tname));
	if (tpid_entry->task_exit < ((1 << RSC_TASK_EXIT_BIT) - 1))
		tpid_entry->task_exit++;
#if 1
	rsc_dbg(CPU_TOP,
#else
	rsc_info(
#endif
	"%s: rsc_total_pid: %d Comm: %16s(%d)"
		" tpcomm: %16s(%d) utime: %lu(%lu) stime: %lu(%lu)\n",
		__func__, rsc_total_pid, task->comm, task->pid,
		tpid_entry->comm, tpid, (unsigned long)utime, (unsigned long)tpid_entry->utime,
		(unsigned long)stime, (unsigned long)tpid_entry->stime);
	task->rsc_exit |= RSC_CPU_TOP_EXIT;

exit:
	mutex_unlock(&pid_lock);
	return NOTIFY_OK;
}

#ifndef CONFIG_RSC_UID_IO
static struct notifier_block top_process_notifier_block = {
	.notifier_call	= rsc_top_process_notifier,
};
#endif

static int __init proc_cpu_top_init(void)
{
	struct proc_dir_entry *dir;

	/*show all processes*/
	dir = proc_create_data("topa", S_IRUSR|S_IRGRP, vivo_rsc, &stat_top_all_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	/*show process that utime or stime is non-zero*/
	dir = proc_create_data("top", S_IRUSR|S_IRGRP, vivo_rsc, &stat_top_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	dir = proc_create_data("top_exit", S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP, vivo_rsc, &top_exit_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
#ifndef CONFIG_RSC_UID_IO
	if (rsc_top_exit)
		profile_event_register(PROFILE_TASK_EXIT, &top_process_notifier_block);
#endif
	/*rsc_info("cputime_one_jiffy: %ld\n", cputime_one_jiffy);*/
	return 0;
}

late_initcall(proc_cpu_top_init);
