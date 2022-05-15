/*
 * kernel/rsc/rsc_uid_io.c
 *
 * VIVO Resource Control.
 *
 * stat uid io.
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
#include <linux/vivo_rsc/rsc_internal.h>

#define UID_HASH_BITS	10
static DECLARE_HASHTABLE(hash_table, UID_HASH_BITS);

static DEFINE_MUTEX(uid_lock);

struct sort_entry {
	u64 read_bytes;
	u64 write_bytes;
	u64 cancelled_write_bytes;
	uid_t uid;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	u32 rsc_sig;
#endif
};

struct uid_io_entry {
	struct sort_entry ent;
	u64 active_read_bytes;
	u64 active_write_bytes;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	u32 active_cancelled_write_bytes;
	u32 rsc_sig;
#else
	u64 active_cancelled_write_bytes;
#endif
	struct hlist_node hash;
};

static u32 total_uid;
#define MAX_UID_IO 4096

static struct uid_io_entry *find_uid_entry(uid_t uid)
{
	struct uid_io_entry *uid_entry;
	hash_for_each_possible(hash_table, uid_entry, hash, uid) {
		if (uid_entry->ent.uid == uid)
			return uid_entry;
	}
	return NULL;
}

static struct uid_io_entry *find_or_register_uid(uid_t uid)
{
	struct uid_io_entry *uid_entry;

	uid_entry = find_uid_entry(uid);
	if (uid_entry)
		return uid_entry;

	uid_entry = kzalloc(sizeof(struct uid_io_entry), GFP_ATOMIC);
	if (!uid_entry)
		return NULL;
	total_uid++;
	uid_entry->ent.uid = uid;

	hash_add(hash_table, &uid_entry->hash, uid);

	return uid_entry;
}

#define CMP_NEG -1
#define CMP_POS 1
static int cmp_io(const void *a, const void *b)
{
	const struct sort_entry *l = a, *r = b;

	if (l->write_bytes > r->write_bytes)
		return CMP_NEG;
	if (l->write_bytes < r->write_bytes)
		return CMP_POS;
	return 0;
}

static void swap_io(void *a, void *b, int size)
{
	struct sort_entry *l = a, *r = b;
	struct sort_entry tmp;

	tmp = *l;
	*l = *r;
	*r = tmp;
}

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
u32 rsc_sig_has_val_cnt;
#endif

#define REMOVE_RESET_IN_BEGIN
/*#define RSC_SHOW_CANCELLED_WRITE_BYTES*/
static int uid_stat_show(struct seq_file *m, void *v)
{
	struct uid_io_entry *uid_entry;
	struct task_struct *task, *temp;
	unsigned long bkt;
	int i, num, actnum;
	int size;
	struct sort_entry *entry;
	void (*free_fun)(const void *addr);
	struct timeval stime;

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	if (rsc_check_signal_enable) {
		struct task_struct *g, *p;

		seq_printf(m, "enable%d fignore: %d rsendsig: %d fixtime: %d ffixtime: %d failtime: %d "
						"trytime: %d thrtime: %lu forcedelay %lu us supendcnt: %lu sigtoolongcnt: %d "
						"fatalpend : %d otherpend %d rsc_sig24_ignore: %d maxstime: %u ms notsuspend %d %d %d gcthread: %s exitcnt %u cname %u gcmd %u\n",
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
		/*
		* I am not sure, rsc_need_check_suspend -> p->group_leader->comm is or not safe?
		*/
		if (unlikely(rsc_debug & RSC_FIX_DEADLOCK)) {
			i = 0;
			rcu_read_lock();
			for_each_process_thread(g, p) {
				u32 cnt;
				if (try_get_task_stack(p)) {
					cnt = atomic_read(&task_thread_info(p)->rsc_sig);
					put_task_stack(p);
					if (cnt) {
						seq_printf(m, "%2d rsc_sig: high %d low %d uid %5d %16s %5d(%16s %5d) needcheck%d\n",
							i, cnt>>16, cnt&0xffff, task_uid(p).val, p->comm, p->pid,
							p->group_leader?p->group_leader->comm:"NULL",
							p->group_leader?p->group_leader->pid:99999,
							rsc_need_check_suspend(p));
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
	}
#endif

	mutex_lock(&uid_lock);
#ifndef REMOVE_RESET_IN_BEGIN
	hash_for_each(hash_table, bkt, uid_entry, hash) {
		uid_entry->active_read_bytes = 0;
		uid_entry->active_write_bytes = 0;
		uid_entry->active_cancelled_write_bytes = 0;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		uid_entry->rsc_sig = 0;
#endif
	}
#endif
	read_lock(&tasklist_lock);
	do_each_thread(temp, task) {
		uid_entry = find_or_register_uid(from_kuid_munged(
			current_user_ns(), task_uid(task)));
		if (!uid_entry) {
			read_unlock(&tasklist_lock);
			mutex_unlock(&uid_lock);
			rsc_err("%s: failed to find the uid_entry for uid %d\n",
				__func__, from_kuid_munged(current_user_ns(),
				task_uid(task)));
			return -ENOMEM;
		}
		/* if this task is exiting, we have already accounted for the
		 * data.
		 */
		if (!(task->rsc_exit & RSC_UID_IO_EXIT)) {
			uid_entry->active_read_bytes += task->ioac.read_bytes;
			uid_entry->active_write_bytes += task->ioac.write_bytes;
			uid_entry->active_cancelled_write_bytes += task->ioac.cancelled_write_bytes;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
			if (try_get_task_stack(task)) {
				uid_entry->rsc_sig += atomic_read(&task_thread_info(task)->rsc_sig);
				put_task_stack(task);
			}
#endif
		}
#if 0
	/*
		   uid 9999 is the next process!
		   rmt_storage
		msm_irqbalance
	*/
		if (uid_entry->ent.uid == 9999)
			rsc_info("comm(%5d) : %16s uid is 999, parent(%5d): %16s\n", task->pid, task->comm,
			task->real_parent->pid, task->real_parent->comm);
#endif
	} while_each_thread(temp, task);
	read_unlock(&tasklist_lock);

	if (total_uid > MAX_UID_IO)
		size = MAX_UID_IO * sizeof(struct sort_entry);
	else
		size = total_uid * sizeof(struct sort_entry);

	entry = NULL;
	free_fun = kfree;
	if (size <= (16 * 1024))
		entry = kmalloc(size, GFP_KERNEL);
	if (!entry) {
		entry = vmalloc(size);
		free_fun = vfree;
	}
	if (!entry) {
		mutex_unlock(&uid_lock);
		rsc_err("%s: Not enough memory!\n", __func__);
		return -ENOMEM;
	}

	num = 0;
	actnum = 0;
	hash_for_each(hash_table, bkt, uid_entry, hash) {
		actnum++;
		if (num < MAX_UID_IO) {
			entry[num].read_bytes = uid_entry->ent.read_bytes +
				uid_entry->active_read_bytes;
			entry[num].write_bytes = uid_entry->ent.write_bytes +
				uid_entry->active_write_bytes;
			entry[num].cancelled_write_bytes = uid_entry->ent.cancelled_write_bytes +
				uid_entry->active_cancelled_write_bytes;
			entry[num].uid = uid_entry->ent.uid;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
			entry[num].rsc_sig = uid_entry->ent.rsc_sig + uid_entry->rsc_sig;
#endif
			num++;
		} else
			rsc_err("%s: num %d bigger than max: %d!\n", __func__, actnum, MAX_UID_IO);
#ifdef REMOVE_RESET_IN_BEGIN
		uid_entry->active_read_bytes = 0;
		uid_entry->active_write_bytes = 0;
		uid_entry->active_cancelled_write_bytes = 0;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		uid_entry->rsc_sig = 0;
#endif
#endif
	}
	sort(entry, num, sizeof(struct sort_entry),
	     cmp_io, swap_io);

	do_gettimeofday(&stime);

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	if (rsc_check_signal_enable) {
	seq_printf(m, "%5s\t%11s\t%11s\t"
#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
		"%11s\t"
#endif
		"total_uid\t%d\t%llu %llu\tsig high\tlow\n",
		"UID", "write_bytes", "read_bytes",
#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
		"cancelwrite",
#endif
		total_uid,
		(u64)stime.tv_sec, (u64)stime.tv_usec);

	for (i = 0; i < num; i++)
		seq_printf(m, "%d\t%llu\t%llu"
	#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
		"\t%llu"
	#endif
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		"\t%u\t%u"
#endif
		"\n", entry[i].uid,
			entry[i].write_bytes, entry[i].read_bytes
	#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
			, entry[i].cancelled_write_bytes
	#endif
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
			, entry[i].rsc_sig>>16, entry[i].rsc_sig&0xffff
#endif
			);
	} else
#endif
	{
		seq_printf(m, "%5s\t%11s\t%11s\t"
#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
			"%11s\t"
#endif
			"total_uid\t%d\t%llu %llu\n",
			"UID", "write_bytes", "read_bytes",
#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
			"cancelwrite",
#endif
			total_uid,
			(u64)stime.tv_sec, (u64)stime.tv_usec);

		for (i = 0; i < num; i++)
			seq_printf(m, "%d\t%llu\t%llu"
	#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
			"\t%llu"
	#endif
			"\n", entry[i].uid,
				entry[i].write_bytes, entry[i].read_bytes
	#ifdef RSC_SHOW_CANCELLED_WRITE_BYTES
				, entry[i].cancelled_write_bytes
	#endif
				);
	}
#if 0
	hash_for_each(hash_table, bkt, uid_entry, hash) {
		u64 read_bytes;
		u64 write_bytes;
		u64 cancelled_write_bytes;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		u32 rsc_sig;
#endif

		read_bytes = uid_entry->ent.read_bytes +
				uid_entry->active_read_bytes;
		write_bytes = uid_entry->ent.write_bytes +
				uid_entry->active_write_bytes;
		cancelled_write_bytes = uid_entry->ent.cancelled_write_bytes +
				uid_entry->active_cancelled_write_bytes;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		rsc_sig = uid_entry->ent.rsc_sig + uid_entry->rsc_sig;
#endif
		seq_printf(m, "%5d: %11llu %11llu %10llu\n", uid_entry->ent.uid,
			write_bytes, read_bytes, cancelled_write_bytes);

#ifdef REMOVE_RESET_IN_BEGIN
		uid_entry->active_read_bytes = 0;
		uid_entry->active_write_bytes = 0;
		uid_entry->active_cancelled_write_bytes = 0;
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
		uid_entry->rsc_sig = 0;
#endif
#endif
	}
#endif
	free_fun(entry);
	mutex_unlock(&uid_lock);

	rsc_dbg(UID_IO, "%s: m->size: %lu count: %lu\n",
		__func__, (unsigned long)m->size, (unsigned long)m->count);

	return 0;
}

static int uid_stat_open(struct inode *inode, struct file *file)
{
	/*force to allocate 8KB buffer, it can speed up!*/
	size_t size = 2 * PAGE_SIZE;
	int ret;

	ret = single_open_size(file, uid_stat_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations uid_io_stat_fops = {
	.open		= uid_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int uid_remove_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, NULL);
}

static ssize_t uid_remove_write(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	struct uid_io_entry *uid_entry;
	struct hlist_node *tmp;
	char uids[128];
	char *start_uid, *end_uid = NULL;
	long int uid_start = 0, uid_end = 0;

	if (count >= sizeof(uids))
		count = sizeof(uids) - 1;

	if (copy_from_user(uids, buffer, count))
		return -EFAULT;

	uids[count] = '\0';
	end_uid = uids;
	start_uid = strsep(&end_uid, "-");

	if (!start_uid || !end_uid)
		return -EINVAL;

	if (kstrtol(start_uid, 10, &uid_start) != 0 ||
		kstrtol(end_uid, 10, &uid_end) != 0) {
		return -EINVAL;
	}
	mutex_lock(&uid_lock);

	for (; uid_start <= uid_end; uid_start++) {
		hash_for_each_possible_safe(hash_table, uid_entry, tmp,
							hash, (uid_t)uid_start) {
			if (uid_start == uid_entry->ent.uid) {
				total_uid--;
				hash_del(&uid_entry->hash);
				kfree(uid_entry);
			}
		}
	}

	mutex_unlock(&uid_lock);
	return count;
}

static const struct file_operations uid_io_remove_fops = {
	.open		= uid_remove_open,
	.release	= single_release,
	.write		= uid_remove_write,
};

static int io_process_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
{
	struct task_struct *task = v;
	struct uid_io_entry *uid_entry;
	uid_t uid;

	if (!task)
		return NOTIFY_OK;

	mutex_lock(&uid_lock);
	uid = from_kuid_munged(current_user_ns(), task_uid(task));
	uid_entry = find_or_register_uid(uid);
	if (!uid_entry) {
		rsc_err("%s: failed to find uid %d\n", __func__, uid);
		goto exit;
	}
	uid_entry->ent.read_bytes += task->ioac.read_bytes;
	uid_entry->ent.write_bytes += task->ioac.write_bytes;
	uid_entry->ent.cancelled_write_bytes += task->ioac.cancelled_write_bytes;
	task->rsc_exit |= RSC_UID_IO_EXIT;

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
{
	u32 rsc_sig = (u32)atomic_read(&task_thread_info(task)->rsc_sig);
	if (rsc_sig) {
		uid_entry->ent.rsc_sig += rsc_sig;
		rsc_sig_has_val_cnt++;
	}
}
#endif

exit:
	mutex_unlock(&uid_lock);
#ifdef CONFIG_RSC_CPU_TOP
	rsc_top_process_notifier(self, cmd, v);
#endif
	return NOTIFY_OK;
}

static struct notifier_block io_process_notifier_block = {
	.notifier_call	= io_process_notifier,
};

static int __init proc_uid_io_init(void)
{
	struct proc_dir_entry *dir;

	hash_init(hash_table);

	dir = proc_create_data("remove_uid_io", S_IWUSR|S_IWGRP, vivo_rsc, &uid_io_remove_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	dir = proc_create_data("show_uid_io", S_IRUSR|S_IRGRP, vivo_rsc, &uid_io_stat_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	profile_event_register(PROFILE_TASK_EXIT, &io_process_notifier_block);

	return 0;
}

late_initcall(proc_uid_io_init);
