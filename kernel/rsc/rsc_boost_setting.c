/*
 * kernel/rsc/rsc_boost_setting.c
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

static DEFINE_MUTEX(rsc_boost_lock);
#define BOOST_PID_HASH_BITS	11

static DECLARE_HASHTABLE(hash_table, BOOST_PID_HASH_BITS);
static LIST_HEAD(tpid_entry_head);
static LIST_HEAD(tpid_free_head);

struct rsc_boost_entry {
	struct hlist_node hash;
	struct list_head link;
	u32		tv_sec;/* seconds */
	pid_t pid;
};

#define RSC_MAX_BOOST_PID 64
static struct rsc_boost_entry boost_tentry[RSC_MAX_BOOST_PID];
static atomic_t rsc_total_boost_pid;

static struct rsc_boost_entry *rsc_find_boost_pid_entry(pid_t pid)
{
	struct rsc_boost_entry *tpid_entry;
	hash_for_each_possible(hash_table, tpid_entry, hash, pid) {
		if (tpid_entry->pid == pid)
			return tpid_entry;
	}
	return NULL;
}

static struct rsc_boost_entry *rsc_find_or_register_boost_pid(pid_t pid,
	struct task_struct *task)
{
	struct rsc_boost_entry *tpid_entry;
	int count;

	tpid_entry = rsc_find_boost_pid_entry(pid);
	if (tpid_entry)
		return tpid_entry;

	count = atomic_inc_return(&rsc_total_boost_pid);
	if (count <= RSC_MAX_BOOST_PID) {
		tpid_entry = list_first_entry_or_null(&tpid_free_head,
			struct rsc_boost_entry, link);
		if (!tpid_entry) {
			rsc_err("%s: no free buf. pid: %d rsc_total_boost_pid: %d\n",
				__func__, pid,  atomic_read(&rsc_total_boost_pid));
			return NULL;
		}
		list_move_tail(&tpid_entry->link, &tpid_entry_head);
	} else {
		atomic_dec(&rsc_total_boost_pid);
		rsc_info("%s: no free buf. pid: %d. setting pid more than rsc_total_boost_pid: %d\n",
			__func__, pid, atomic_read(&rsc_total_boost_pid));
		return NULL;
	}

	tpid_entry->tv_sec = (u32)(get_jiffies_64()/HZ);
	tpid_entry->pid = pid;
	hash_add(hash_table, &tpid_entry->hash, pid);

	return tpid_entry;
}


static struct rsc_boost_entry *rsc_del_boost_pid(pid_t pid,
	struct task_struct *task)
{
	struct rsc_boost_entry *tpid_entry;

	tpid_entry = rsc_find_boost_pid_entry(pid);
	if (!tpid_entry)
		return NULL;

	hash_del(&tpid_entry->hash);
	list_move_tail(&tpid_entry->link, &tpid_free_head);
	atomic_dec(&rsc_total_boost_pid);

	if (atomic_read(&rsc_total_boost_pid) < 0) {
		rsc_err("%s: error! pid: %d rsc_total_boost_pid: %d\n",
			__func__, pid, atomic_read(&rsc_total_boost_pid));
		return NULL;
	}

	return tpid_entry;
}

static ssize_t show_rsc_boost(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	struct rsc_boost_entry *tpid_entry;
	int cnt = 0;
	u32 cursec;
	u32 elaps;
	struct task_struct *p, *gl;

	cursec = (u32)(get_jiffies_64()/HZ);
	mutex_lock(&rsc_boost_lock);
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "total: %d\n", atomic_read(&rsc_total_boost_pid));
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
			p?p->rsc_boost:0xE,
			elaps
		);
		ret += cur;
		rcu_read_unlock();
		cnt++;
	}

	if (atomic_read(&rsc_total_boost_pid) != cnt) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "list error! %d != %d\n",
		atomic_read(&rsc_total_boost_pid), cnt);
		ret += cur;
	}
	mutex_unlock(&rsc_boost_lock);

	return ret;
}

static ssize_t store_rsc_boost(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct task_struct *p, *gl;
	struct rsc_boost_entry *tpid_entry;
	int pid, val;
	int ret;

	ret = sscanf(buf, "%d %d", &pid, &val);
	if ((ret != 2))
		return -EINVAL;
	if (val && (val != 1))
		return -EINVAL;

	mutex_lock(&rsc_boost_lock);
	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		rsc_info("%s could not find pid %d!\n",
			__func__, pid);
		mutex_unlock(&rsc_boost_lock);
		return -EINVAL;
	}
	get_task_struct(p);

	gl = rcu_dereference(p->group_leader);
	if (val) {
		tpid_entry = rsc_find_or_register_boost_pid(pid, p);
		if (tpid_entry) {
			p->rsc_boost = val;
			gl->rsc_boost = val*2;
		} else
			count = -ENOSPC;
	} else {
		rsc_del_boost_pid(pid, p);
		p->rsc_boost = val;
		gl->rsc_boost = val;
	}

	rcu_read_unlock();
	put_task_struct(p);
	mutex_unlock(&rsc_boost_lock);
	return count;
}

static int rsc_boost_notifier(struct notifier_block *self,
			unsigned long cmd, void *v)
{
	struct task_struct *task = v;

	if (!task || (atomic_read(&rsc_total_boost_pid) <= 0))
		return NOTIFY_OK;

	mutex_lock(&rsc_boost_lock);
	rsc_del_boost_pid(task->pid, task);
	mutex_unlock(&rsc_boost_lock);
	return NOTIFY_OK;
}

#define RSC_BOOST rsc_boost

static struct kobj_attribute rsc_boost_attr =
__ATTR(RSC_BOOST, 0660, show_rsc_boost, store_rsc_boost);

static struct notifier_block rsc_boost_notifier_block = {
	.notifier_call	= rsc_boost_notifier,
};

static int __init rsc_boost_settting_init(void)
{
	long ret;
	int i;

	ret = sysfs_create_file(rsc_root_dir, &rsc_boost_attr.attr);
	if (ret)
		goto fail;
	rsc_chown_to_system(rsc_root_dir, &rsc_boost_attr.attr);
	for (i = 0; i < RSC_MAX_BOOST_PID; i++) {
		struct rsc_boost_entry *tpid_entry;

		tpid_entry = &boost_tentry[i];
		list_add_tail(&tpid_entry->link, &tpid_free_head);
	}
	profile_event_register(PROFILE_TASK_EXIT, &rsc_boost_notifier_block);
fail:

	return 0;
}

late_initcall(rsc_boost_settting_init);
