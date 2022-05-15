/*
 * kernel/rsc/rsc_lmk_stat.c
 *
 * VIVO Resource Control.
 *
 * stat low memory kill process.
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

#define UID_HASH_BITS	8
static DECLARE_HASHTABLE(hash_table, UID_HASH_BITS);

static DEFINE_MUTEX(uid_lock);
/* (128 - 10 )/2 = 59*/
#define MAX_PROCSS_PER_UID 59
#define MAX_LMK_UID	256
#define MAX_LMK_PROCESS	1024
#define LMK_FREE_UID_PID 	0x80000000
#define LMK_REMOVE_UID_PID 	0x40000000

/*#define RECORD_PARENT_COMM*/
#define RECORD_PROCESS_KILL_BY_PID
/*#define RECORD_LMK_TYPE*/
#define LMK_RECORD_TIME
#define LMK_RECORD_ALWAYS

struct lmk_process_t {
#ifdef LMK_RECORD_TIME
	u64 time;
#endif
	pid_t tpid;
	pid_t ppid;
#ifndef RECORD_PROCESS_KILL_BY_PID
	u16 killed_time;
#endif
	char tcomm[TASK_COMM_LEN];
#ifdef RECORD_PARENT_COMM
	char pcomm[TASK_COMM_LEN];
#endif
#ifdef RECORD_LMK_TYPE
	/*0: lowmemorykiller 1: out of memory*/
	u8 type;
#endif
};

struct entry_t {
	uid_t uid;
	u16 killed_time;
	u16 killed_num;
	u16 pos;
	u16 proc_group[MAX_PROCSS_PER_UID];
};

struct uid_lmk_entry {
	struct entry_t ent;
	struct hlist_node hash;
};

static struct uid_lmk_entry lmk_entry[MAX_LMK_UID];
static struct lmk_process_t lmk_process[MAX_LMK_PROCESS];
static u32 total_uid, act_total_uid;
static u32 lmk_kill_time, act_lmk_kill_time;
static u16 proc_allocate_num, act_proc_allocate_num, proc_allocate_pos;
static int lmk_stat_init;
static int lmk_prev_index;

static struct uid_lmk_entry *find_uid_entry(uid_t uid)
{
	struct uid_lmk_entry *uid_entry;
	hash_for_each_possible(hash_table, uid_entry, hash, uid) {
		if (uid_entry->ent.uid == uid)
			return uid_entry;
	}
	return NULL;
}

static struct uid_lmk_entry *find_or_register_uid(uid_t uid)
{
	struct uid_lmk_entry *uid_entry;
	int i, j;

	uid_entry = find_uid_entry(uid);
	if (uid_entry)
		return uid_entry;

	if (total_uid >= MAX_LMK_UID)
		return NULL;
	for (i = 0; i < MAX_LMK_UID; i++) {
		j = (i + act_total_uid) % MAX_LMK_UID;
		if (lmk_entry[j].ent.uid & LMK_FREE_UID_PID) {
			uid_entry = &lmk_entry[j];
			break;
		}
	}

	if (i >= MAX_LMK_UID)
		return NULL;
	total_uid++;
	act_total_uid++;
	uid_entry->ent.uid = uid;

	hash_add(hash_table, &uid_entry->hash, uid);

	return uid_entry;
}

static __always_inline void rsc_lmk_stat_internal(u8 type, struct task_struct *task)
{
	struct uid_lmk_entry *uid_entry;
	struct task_struct *pt;
	struct task_struct *rt;
#ifdef LMK_RECORD_TIME
	u64 time;
#endif
	int i, j, k;

	if (!lmk_stat_init)
		return;

	mutex_lock(&uid_lock);

	uid_entry = find_or_register_uid(from_kuid_munged(
		current_user_ns(), task_uid(task)));

	if (!uid_entry) {
		mutex_unlock(&uid_lock);
		rsc_err("%s: failed to find the uid_entry for uid %d\n",
			__func__, from_kuid_munged(current_user_ns(),
			task_uid(task)));
		return;
	}

	rcu_read_lock();

	i = lmk_prev_index;
#ifdef LMK_RECORD_TIME
	time = local_clock();
#endif
	pt = rcu_dereference(task->group_leader);
	if (!pt)
		pt = task;

	if (!thread_group_leader(task)) {
#if 0
		rsc_info(
#else
		rsc_dbg(UID_LMK,
#endif
			"%s: type: %d - %d process %s(%d) belong group %s(%d) "
#ifdef LMK_RECORD_TIME
			"timedif %llu (%llu - %llu) ms, ingore it."
#endif
			"\n",
			__func__,
#ifdef RECORD_LMK_TYPE
			lmk_process[i].type,
#else
			0,
#endif
			type, task->comm, task->pid, pt->comm, pt->pid
#ifdef LMK_RECORD_TIME
			, (time - lmk_process[i].time)/(1000*1000),
			lmk_process[i].time/(1000*1000), time/(1000*1000)
#endif
			);
		goto out;
	}

#ifndef LMK_RECORD_ALWAYS
	/*if kill 2 time in one second, it will be ignored.*/
	if ((lmk_process[i].tpid == pt->pid) &&
		!strncmp(lmk_process[i].tcomm, pt->comm, TASK_COMM_LEN)
#ifdef LMK_RECORD_TIME
		&& time_before64(time, lmk_process[i].time + 1 * 1000 * 1000 * 1000)
#endif
		) {
		rt = task->real_parent;
		if ((rt && (rt->pid == lmk_process[i].ppid)) || !rt) {
			rsc_dbg(UID_LMK, "%s: type: %d - %d process %s(%d) killed 2times"
#ifdef LMK_RECORD_TIME
				" in one second, timedif %llu (%llu - %llu) ms, ingore it."
#endif
				"\n",
				__func__,
#ifdef RECORD_LMK_TYPE
				lmk_process[i].type,
#else
				0,
#endif
				 type, pt->comm, pt->pid
#ifdef LMK_RECORD_TIME
				, (time - lmk_process[i].time)/(1000*1000),
				lmk_process[i].time/(1000*1000), time/(1000*1000)
#endif
			);
			if (rsc_debug & UID_LMK)
				dump_stack();
			goto out;
		}
	}
#endif

	lmk_kill_time++;
	act_lmk_kill_time++;
	if (uid_entry->ent.killed_time < USHRT_MAX)
		uid_entry->ent.killed_time++;
#ifdef RECORD_PROCESS_KILL_BY_PID
	i = uid_entry->ent.killed_num;
#else
	for (i = 0; i < uid_entry->ent.killed_num; i++) {
		j = uid_entry->ent.proc_group[i];
		if (!strncmp(lmk_process[j].tcomm, pt->comm, TASK_COMM_LEN)) {
			lmk_process[j].killed_time++;
			goto out;
		}
	}
#endif
	if (i >= uid_entry->ent.killed_num) {
		if (i >= MAX_PROCSS_PER_UID) {
			rsc_err("%s: per uid proc more than max %d  proc_allocate_num %d! comm: %s pid: %d\n",
				__func__, MAX_PROCSS_PER_UID, proc_allocate_num, pt->comm, pt->pid);
			/*goto out;*/
		};
		i = uid_entry->ent.pos;
		uid_entry->ent.pos = (uid_entry->ent.pos + 1) % MAX_PROCSS_PER_UID;

		if (proc_allocate_num >= MAX_LMK_PROCESS) {
			rsc_err("%s: kill process more than proc_allocate_num %d! comm: %s pid: %d\n",
				__func__, proc_allocate_num, pt->comm, pt->pid);
			j = proc_allocate_pos;
			/*goto out;*/
		} else {
			for (k = 0; k < MAX_LMK_PROCESS; k++) {
				j = (act_proc_allocate_num + k) % MAX_LMK_PROCESS;
				if (lmk_process[j].tpid & LMK_FREE_UID_PID)
					break;
			}
			if (j >= MAX_LMK_PROCESS) {
				rsc_err("%s: kill process more than j %d! comm: %s pid: %d\n",
					__func__, j, pt->comm, pt->pid);
				goto out;
			}
		}

		proc_allocate_pos = (proc_allocate_pos + 1) % MAX_LMK_PROCESS;
		uid_entry->ent.proc_group[i] = j;
#ifdef LMK_RECORD_TIME
		lmk_process[j].time = time;
#endif
		lmk_prev_index = j;
		strlcpy(lmk_process[j].tcomm, pt->comm, TASK_COMM_LEN);
		lmk_process[j].tpid = pt->pid;
#ifdef RECORD_LMK_TYPE
		lmk_process[j].type = type;
#endif
		rt = task->real_parent;
		if (rt) {
			lmk_process[j].ppid = rt->pid;
#ifdef RECORD_PARENT_COMM
			strlcpy(lmk_process[j].pcomm, rt->comm, TASK_COMM_LEN);
#endif
		}

#ifndef RECORD_PROCESS_KILL_BY_PID
		if (lmk_process[j].killed_time < USHRT_MAX)
			lmk_process[j].killed_time++;
#endif
		if (proc_allocate_num < MAX_LMK_PROCESS)
			proc_allocate_num++;

		act_proc_allocate_num++;
		if (uid_entry->ent.killed_num < MAX_PROCSS_PER_UID)
			uid_entry->ent.killed_num++;
	}

out:
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	mutex_unlock(&uid_lock);

}

static int uid_stat_show(struct seq_file *m, void *v)
{
	struct uid_lmk_entry *uid_entry;
	unsigned long bkt;
	int i, j, num;

	if (!lmk_stat_init)
		return 0;

	mutex_lock(&uid_lock);

	seq_printf(m, "total_uid:\t%4u\t%4u\tlmk_kill_time:\t%6u\t%6u\tkilled_proc_num:\t%4u\t%4u\n",
					total_uid, act_total_uid, lmk_kill_time, act_lmk_kill_time,
					proc_allocate_num, act_proc_allocate_num);

	hash_for_each(hash_table, bkt, uid_entry, hash) {
		num = uid_entry->ent.killed_num;
		seq_printf(m, "UID:\t%5u\tkilled_time:\t%5u\tnum:\t%5u\n",
					uid_entry->ent.uid, uid_entry->ent.killed_time,
					uid_entry->ent.killed_num);

		seq_printf(m, "    \t%5s\t%16s\t%5s\t%s\t%5s"
#ifdef RECORD_PARENT_COMM
			"\t%16s"
#endif
#ifdef LMK_RECORD_TIME
			"\t%10s"
#endif
			"\n",
				"PID", "TASK", "KTIME", "T", "PPID"
#ifdef RECORD_PARENT_COMM
				, "PTASK"
#endif
#ifdef LMK_RECORD_TIME
				, "TIME"
#endif
				);
		for (i = 0; i < num; i++) {
			j = uid_entry->ent.proc_group[i];
			seq_printf(m, "    \t%5u\t%16s\t%5u\t%d\t%5u"
#ifdef RECORD_PARENT_COMM
				"\t%16s"
#endif
#ifdef LMK_RECORD_TIME
				"\t%llu"
#endif
				"\n",
					lmk_process[j].tpid, lmk_process[j].tcomm,
#ifndef RECORD_PROCESS_KILL_BY_PID
					lmk_process[j].killed_time,
#else
					1,
#endif
#ifdef RECORD_LMK_TYPE
					lmk_process[j].type,
#else
					0,
#endif
					lmk_process[j].ppid
#ifdef RECORD_PARENT_COMM
					, lmk_process[j].pcomm
#endif
#ifdef LMK_RECORD_TIME
					, lmk_process[j].time/1024
#endif
					);
		}
		seq_printf(m, "\n");
	}

	seq_printf(m, "show all remove UID:\n");
	for (j = 0; j < MAX_LMK_UID; j++)
		if (lmk_entry[j].ent.uid & LMK_REMOVE_UID_PID)
			seq_printf(m, "    \tREMOVE UID:\t%5u\n",
			lmk_entry[j].ent.uid & ~(LMK_FREE_UID_PID | LMK_REMOVE_UID_PID));

	seq_printf(m, "\nshow all remove proc:\n");
	seq_printf(m, "    \t%5s\t%16s\t%5s\t%s\t%5s"
#ifdef RECORD_PARENT_COMM
		"\t%16s"
#endif
#ifdef LMK_RECORD_TIME
		"\t%10s"
#endif
		"\n",
				"REPID", "TASK", "KTIME", "T", "PPID"
#ifdef RECORD_PARENT_COMM
				, "PTASK"
#endif
#ifdef LMK_RECORD_TIME
				, "TIME"
#endif
				);
	for (j = 0; j < MAX_LMK_PROCESS; j++)
		if (lmk_process[j].tpid & LMK_REMOVE_UID_PID) {
			seq_printf(m, "    \t%5u\t%16s\t%5u\t%d\t%5u\t"
#ifdef RECORD_PARENT_COMM
				"%16s"
#endif
#ifdef LMK_RECORD_TIME
				"\t%llu"
#endif
				"\n",
					lmk_process[j].tpid & ~(LMK_FREE_UID_PID | LMK_REMOVE_UID_PID),
					lmk_process[j].tcomm,
#ifndef RECORD_PROCESS_KILL_BY_PID
					lmk_process[j].killed_time,
#else
					1,
#endif
#ifdef RECORD_LMK_TYPE
					lmk_process[j].type,
#else
					0,
#endif
					lmk_process[j].ppid
#ifdef RECORD_PARENT_COMM
					, lmk_process[j].pcomm
#endif
#ifdef LMK_RECORD_TIME
					, lmk_process[j].time/1024
#endif
					);
			}
	mutex_unlock(&uid_lock);

	rsc_dbg(UID_LMK, "%s: m->size: %lu count: %lu\n",
		__func__, (unsigned long)m->size, (unsigned long)m->count);

	return 0;
}

/*
caller has got rcu lock! we must release it first, because we will get
mutex_lock(&uid_lock); in rsc_lmk_stat_internal
*/
void rsc_lmk_stat(u8 type, struct task_struct *selected)
{
	get_task_struct(selected);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	rsc_lmk_stat_internal(type, selected);
	rcu_read_lock();
	put_task_struct(selected);
}

static int uid_stat_open(struct inode *inode, struct file *file)
{
	/*force to allocate 16KB buffer, it can speed up!*/
	size_t size = 4 * PAGE_SIZE;
	int ret;

	ret = single_open_size(file, uid_stat_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations uid_lmk_stat_fops = {
	.open		= uid_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int uid_check_open(struct inode *inode, struct file *file)
{
	return single_open(file, NULL, NULL);
}

/*
Note: uid_check_write could not return zero,
or echo pid 1 > /proc/vivo_rsc/check_lmk  will go into deadloop!!!
usage:
check and remove:		echo pid 1 > /proc/vivo_rsc/check_lmk
check donot remove:	echo pid 0 > /proc/vivo_rsc/check_lmk
				or	echo pid    > /proc/vivo_rsc/check_lmk
return value	< 0, not find;
			> 0, find
call stack:
uid_check_write+0x2c/0x1dc
proc_reg_write+0x74/0x90
__vfs_write+0x28/0xd0
vfs_write+0xac/0x144
SyS_write+0x48/0x84
cpu_switch_to+0x250/0x420
*/
static ssize_t uid_check_write(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret;
	pid_t pid;
	int i, j, k, num;
	struct uid_lmk_entry *uid_entry;
	unsigned long bkt;
	int remove = 0;
	ssize_t rcnt = -ENOENT;
	char kbuf[128];

	if (!count)
		return -EFAULT;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	kbuf[count] = '\0';
	ret = sscanf(kbuf, "%u %u", &pid, &remove);
	if (ret == 2 && (remove != 1))
			remove = 0;
	if (ret == 1 || ret == 2) {
		mutex_lock(&uid_lock);
		hash_for_each(hash_table, bkt, uid_entry, hash) {
			num = uid_entry->ent.killed_num;
			for (i = 0; i < num; i++) {
				j = uid_entry->ent.proc_group[i];
				if (pid == lmk_process[j].tpid) {
					if (remove == 1) {
						for (k = i; k < (num - 1); k++)
							uid_entry->ent.proc_group[k] =
								uid_entry->ent.proc_group[k+1];
#ifndef RECORD_PROCESS_KILL_BY_PID
						uid_entry->ent.killed_time -= lmk_process[j].killed_time;
						lmk_kill_time -= lmk_process[j].killed_time;
#else
						uid_entry->ent.killed_time -= 1;
						lmk_kill_time -= 1;
#endif
						uid_entry->ent.killed_num--;
						uid_entry->ent.pos = uid_entry->ent.killed_num;
						lmk_process[j].tpid |= LMK_FREE_UID_PID | LMK_REMOVE_UID_PID;
						proc_allocate_num--;
						if (!uid_entry->ent.killed_num) {
							total_uid--;
							hash_del(&uid_entry->hash);
							uid_entry->ent.uid |= LMK_FREE_UID_PID | LMK_REMOVE_UID_PID;
							/*kfree(uid_entry);*/
						}
					}
					rcnt = count;
					goto find;
				}
			}
		}
find:
		mutex_unlock(&uid_lock);
	}

	return rcnt;
}


static const struct file_operations uid_lmk_check_fops = {
	.open		= uid_check_open,
	.release	= single_release,
	.write		= uid_check_write,
};

static int __init proc_uid_lmk_init(void)
{
	int i;
	struct proc_dir_entry *dir;

	hash_init(hash_table);

	dir = proc_create_data("show_lmk_stat", S_IRUSR|S_IRGRP, vivo_rsc, &uid_lmk_stat_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	dir = proc_create_data("check_lmk", S_IWUSR|S_IWGRP, vivo_rsc, &uid_lmk_check_fops,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);

	for (i = 0; i < MAX_LMK_UID; i++)
		lmk_entry[i].ent.uid = LMK_FREE_UID_PID;

	for (i = 0; i < MAX_LMK_PROCESS; i++)
		lmk_process[i].tpid = LMK_FREE_UID_PID;

	lmk_stat_init = 1;

	return 0;
}

late_initcall(proc_uid_lmk_init);
