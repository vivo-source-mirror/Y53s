#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/cpu.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/atomic.h>

#include <linux/vivo_rsc/rsc_internal.h>

#include <linux/proc_fs.h>
#include <linux/poll.h>

#define BINDER_EVENT_NAME binder_event
#define BINDER_EVENT_CNT (128)

#define BINDER_CMD_NAME binder_cmd

#define BINDER_EVENT_USE_SYSFS
//#define BINDER_EVENT_USE_PROCFS


extern bool cgroup_freezing_or_frozen(struct task_struct* task);

enum event_type_t {
	BBL,
	BFL,
	ERR
};

static const char *event_type_str[] = {
	"BBL",
	"BFL",
	"ERR"
};

struct binder_event {
	int ver;
	int type;
	int idx;
	int src_uid;
	int src_pid;
	int dst_uid;
	int dst_pid;
};

struct binder_event_data {
	spinlock_t bf_lock;
	atomic_t cnt;									/* binder blocked events count in queue */
	atomic_t idx;
	size_t event_pos;
	int initialized;
	int bbl_switch;									/* binder blocked event notify control switch */
	struct binder_event events[BINDER_EVENT_CNT];	/* binder blocked events queue */
	unsigned long long r_idx, w_idx;
};

static struct binder_event_data bf_mdata;

#ifdef BINDER_EVENT_USE_SYSFS
/* app read event */
static ssize_t show_rsc_binder_event(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	struct binder_event event;

	if (unlikely(!bf_mdata.initialized))
		return 0;

	/* no data */
	if (atomic_read(&bf_mdata.cnt) == 0) {
		return 0;
	}

	/* get event from events queue */
	spin_lock_bh(&bf_mdata.bf_lock);
	event = bf_mdata.events[(bf_mdata.r_idx++)%BINDER_EVENT_CNT];
	atomic_dec(&bf_mdata.cnt);
	spin_unlock_bh(&bf_mdata.bf_lock);

	/* format: "event_type,ver_No,idx,src_uid,src_pid,dst_uid,dst_pid" */
	ret = snprintf(buf, PAGE_SIZE, "%3s,%d,%d,%d,%d,%d,%d",
			event_type_str[event.type], event.ver, event.idx,
			event.src_uid, event.src_pid, event.dst_uid, event.dst_pid);
	return ret;
}

/* mannually write this file for debug */
static ssize_t store_rsc_binder_event(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
#if 1/* code for debug binder event */
	int ret;
	struct task_struct *tsk;
	struct binder_event event;
	int type, src_uid, src_pid, dst_uid, dst_pid;

	ret = sscanf(buf, "%d,%d,%d,%d,%d", &type, &src_uid, &src_pid, &dst_uid, &dst_pid);

	if (ret < 5) {
		pr_debug("binder_event %s invalid arg %s\n", __func__, buf);
		return -EINVAL;
	}

	if (BBL != type && BFL != type) {
		pr_debug("binder_event %s invalid arg %s\n", __func__, buf);
		return -EINVAL;
	}

	if (unlikely(!bf_mdata.initialized)) {
		pr_debug("binder_event not initialized!\n");
		return -EINVAL;
	}

	/* increase the idx nomatter we can save this event or not */
	event.idx = atomic_inc_return(&bf_mdata.idx);

	if (!bf_mdata.bbl_switch) {
		pr_debug("binder_event disabled!\n");
		return -EINVAL;
	}

#if 0
	tsk = find_get_task_by_vpid(dst_pid);
	if (tsk) {
		if (cgroup_freezing_or_frozen(tsk)) {
			pr_err("proc %d is freezing or frozen\n", dst_pid);
		}
	}
#endif

	if (unlikely(atomic_read(&bf_mdata.cnt) >= BINDER_EVENT_CNT)) {
		/* log buffer full error */
		pr_err("binder_event %s event queue full\n", __func__);
		return -EINVAL;
	}

	event.ver = 1;
	event.type = type;
	event.src_uid = src_uid;
	event.src_pid = src_pid;
	event.dst_uid = dst_uid;
	event.dst_pid = dst_pid;

	/* write pid to buffer */
	spin_lock_bh(&bf_mdata.bf_lock);
	bf_mdata.events[(bf_mdata.w_idx++)%BINDER_EVENT_CNT] = event;
	atomic_inc(&bf_mdata.cnt);
	spin_unlock_bh(&bf_mdata.bf_lock);

	sysfs_notify(rsc_root_dir, NULL, __stringify(BINDER_EVENT_NAME));

	pr_debug("notify app proc %d:%d -> %d:%d, by write sysfs\n",
		src_uid, src_pid, dst_uid, dst_pid);

#endif/* code for debug binder event */
	return count;
}

static ssize_t show_rsc_binder_cmd(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret, cur;

	ret = 0;
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "Binder Blocked Notify: %s\n", bf_mdata.bbl_switch? "STARTED": "STOPED");
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "Usage:\n    echo \"BBL#START\" > binder_cmd\n");
	ret += cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "    echo \"BBL#STOP\" > binder_cmd\n");
	ret += cur;

	return ret;
}

/*
Usage:
	enable binder blocked notify:
		echo "BBL#START" > /sys/rsc/binder_cmd
	disable binder blocked notify:
		echo "BBL#STOP" > /sys/rsc/binder_cmd
*/
static ssize_t store_rsc_binder_cmd(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (!strncmp(buf, "BBL#START", strlen("BBL#START"))) {
		bf_mdata.bbl_switch = 1;
	} else if (!strncmp(buf, "BBL#STOP", strlen("BBL#STOP"))) {
		bf_mdata.bbl_switch = 0;
	} else if (!strncmp(buf, "BBL#SET", strlen("BBL#SET"))) {
		/* TODO */
	} else {
		/* unknow cmd */
		pr_err("bind_cmd unknow cmd %s\n", buf);
		return -EINVAL;
	}

	return count;
}

static struct kobj_attribute rsc_binder_event_attr =
__ATTR(BINDER_EVENT_NAME, 0660, show_rsc_binder_event, store_rsc_binder_event);

static struct kobj_attribute rsc_binder_cmd_attr =
__ATTR(BINDER_CMD_NAME, 0660, show_rsc_binder_cmd, store_rsc_binder_cmd);

#elif defined(BINDER_EVENT_USE_PROCFS)/* ifdef BINDER_EVENT_USE_SYSFS */

static DECLARE_WAIT_QUEUE_HEAD(event_poll_wait);

static int binder_event_open(struct inode *inode, struct file *file)
{
	file->private_data = current;
	pr_info(" binder_event: binder_proc_open event pid(%d)\n", ((struct task_struct *)file->private_data)->pid);
	return 0;
}

static int binder_event_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	pr_info(" binder_event: binder_event_release\n");
	return 0;
}

static unsigned int binder_event_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &event_poll_wait, wait);
	if (atomic_read(&bf_mdata.cnt) > 0)
		return POLLIN | POLLRDNORM;
	return 0;
}

ssize_t binder_event_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int err;
	char buffer[64] = {0};
	int ret = 0;
	struct binder_event event;

	if (unlikely(!bf_mdata.initialized))
		return 0;

	/* no data */
	if (atomic_read(&bf_mdata.cnt) == 0) {
		return 0;
	}

	/* get event from events queue */
	spin_lock_bh(&bf_mdata.bf_lock);
	event = bf_mdata.events[(bf_mdata.r_idx++)%BINDER_EVENT_CNT];
	atomic_dec(&bf_mdata.cnt);
	spin_unlock_bh(&bf_mdata.bf_lock);

	/* format: "event_type,ver_No,idx,src_uid,src_pid,dst_uid,dst_pid" */
	ret = snprintf(buffer, 64, "%3s,%d,%d,%d,%d,%d,%d",
			event_type_str[event.type], event.ver, event.idx,
			event.src_uid, event.src_pid, event.dst_uid, event.dst_pid);

	if (ret > size)
		return -ENOMEM;

	err = copy_to_user(buf, buffer, ret);
	if (!err) {
		*ppos += ret;
		return ret;
	}
	return -EFAULT;
}

static ssize_t binder_event_write(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	int ret;
	struct task_struct *tsk;
	struct binder_event event;
	int src_uid, src_pid, dst_uid, dst_pid;
	char buf[64] = {0};

	if (count > 64) {
		pr_debug("binder_event input too long!\n");
		return -EINVAL;
	}

	if (copy_from_user(buf, user_buff, count)) {
			pr_warn(" binder_event: event copy_from_user fail\n");
			return -EINVAL;
	}

	ret = sscanf(buf, "%d,%d,%d,%d", &src_uid, &src_pid, &dst_uid, &dst_pid);

	if (ret < 4) {
		pr_debug("binder_event %s invalid arg %s\n", __func__, buf);
		return -EINVAL;
	}

	if (unlikely(!bf_mdata.initialized)) {
		pr_debug("binder_event not initialized!\n");
		return -EINVAL;
	}

	/* increase the idx nomatter we can save this event or not */
	event.idx = atomic_inc_return(&bf_mdata.idx);

	if (!bf_mdata.bbl_switch) {
		pr_debug("binder_event disabled!\n");
		return -EINVAL;
	}

#if 0
	tsk = find_get_task_by_vpid(dst_pid);
	if (tsk) {
		if (cgroup_freezing_or_frozen(tsk)) {
			pr_err("proc %d is freezing or frozen\n", dst_pid);
		}
	}
#endif

	if (unlikely(atomic_read(&bf_mdata.cnt) >= BINDER_EVENT_CNT)) {
		/* log buffer full error */
		pr_err("binder_event %s event queue full\n", __func__);
		return -EINVAL;
	}

	event.ver = 1;
	event.type = BBL;
	event.src_uid = src_uid;
	event.src_pid = src_pid;
	event.dst_uid = dst_uid;
	event.dst_pid = dst_pid;

	/* write pid to buffer */
	spin_lock_bh(&bf_mdata.bf_lock);
	bf_mdata.events[(bf_mdata.w_idx++)%BINDER_EVENT_CNT] = event;
	atomic_inc(&bf_mdata.cnt);
	spin_unlock_bh(&bf_mdata.bf_lock);

	pr_debug("notify app proc %d:%d -> %d:%d, by write sysfs\n",
		src_uid, src_pid, dst_uid, dst_pid);

	wake_up(&event_poll_wait);

	return count;
}

static int binder_cmd_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Binder Blocked Notify: %s\n", bf_mdata.bbl_switch? "STARTED": "STOPED");

	seq_printf(m, "Usage:\n    echo \"BBL#START\" > binder_cmd\n");

	seq_printf(m, "    echo \"BBL#STOP\" > binder_cmd\n");

	return 0;
}

static int binder_cmd_open(struct inode *inode, struct file *file)
{
	pr_info("binder_event: @%s, uid:%d, pid:%d\n", __func__, task_uid(current).val, current->pid);
	return single_open(file, binder_cmd_show, (void *)&bf_mdata);
}

static ssize_t binder_cmd_store(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	char buf[64] = {0};
	int len = 0;

	len = (count < 64)? count: 64;
	pr_info("binder_event: @%s, uid:%d, pid:%d\n", __func__, task_uid(current).val, current->pid);

	if (copy_from_user(buf, user_buff, len)) {
		pr_warn(" binder_event: cmd copy_from_user fail\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "BBL#START", strlen("BBL#START"))) {
		bf_mdata.bbl_switch = 1;
	} else if (!strncmp(buf, "BBL#STOP", strlen("BBL#STOP"))) {
		bf_mdata.bbl_switch = 0;
	} else if (!strncmp(buf, "BBL#SET", strlen("BBL#SET"))) {
		/* TODO */
	} else {
		/* unknow cmd */
		pr_err("bind_cmd unknow cmd %s\n", buf);
		return -EINVAL;
	}

	return count;
}


static const struct file_operations binder_event_fops = {
	.owner = THIS_MODULE,
	.open = binder_event_open,
	.read = binder_event_read,
	.write = binder_event_write,
	.release = binder_event_release,
	.poll = binder_event_poll,
};

static const struct file_operations binder_cmd_fops = {
	.owner = THIS_MODULE,
	.open = binder_cmd_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = binder_cmd_store,
	.release = single_release,
};


#endif/* elif defined(BINDER_EVENT_USE_PROCFS) */


/*
API for binder driver to notify userspace app that the binder receiver is freezing or frozen.

Note:
1. userspace process must call poll with EPOLLERR|EPOLLPRI.
2. Once poll/select indicates that the value has changed, you need to close and re-open the file, or seek to 0 and read again.

see comment of the kernel function:
__poll_t kernfs_generic_poll(struct kernfs_open_file *of, poll_table *wait);
*/
void notify_app_binder_freez(int type, int src_uid, int src_pid, int dst_uid, int dst_pid)
{
	struct binder_event event;

	if (unlikely(!bf_mdata.initialized)) {
		pr_err("binder_event %s not initialized\n", __func__);
		return;
	}

	/* increase the idx nomatter we can save this event or not */
	event.idx = atomic_inc_return(&bf_mdata.idx);

	if (!bf_mdata.bbl_switch) {
		pr_debug("binder_event %s bbl_switch disabled\n", __func__);
		return;
	}

	/* event queue full */
	if (unlikely(atomic_read(&bf_mdata.cnt) >= BINDER_EVENT_CNT)) {
		pr_debug("binder_event queue full!\n");
		pr_err("binder_event drop: %s uid:%d(pid:%d) -> uid:%d(pid:%d)!\n",
			event_type_str[type], src_uid, src_pid, dst_uid, dst_pid);
		goto NOITFY;
	}

	event.ver = 1;
	event.type = type;
	event.src_uid = src_uid;
	event.src_pid = src_pid;
	event.dst_uid = dst_uid;
	event.dst_pid = dst_pid;

	/* write event to events queue */
	spin_lock_bh(&bf_mdata.bf_lock);
	bf_mdata.events[(bf_mdata.w_idx++)%BINDER_EVENT_CNT] = event;
	atomic_inc(&bf_mdata.cnt);
	spin_unlock_bh(&bf_mdata.bf_lock);

NOITFY:
#ifdef BINDER_EVENT_USE_SYSFS
	sysfs_notify(rsc_root_dir, NULL, __stringify(BINDER_EVENT_NAME));
#elif defined(BINDER_EVENT_USE_PROCFS)
	/* procfs wakeup poll waitqueue */
	wake_up(&event_poll_wait);
#endif

	pr_debug("notify app binder dst proc %d:%d is freezing or frozen\n", dst_uid, dst_pid);
}


static int __init rsc_binder_event_notif_init(void)
{
	long ret = 0;

#ifdef BINDER_EVENT_USE_SYSFS
	/* create sysfs binder_event */
	ret = sysfs_create_file(rsc_root_dir, &rsc_binder_event_attr.attr);
	if (ret) {
		pr_err("creat sysfs file binder_event failed\n");
		return ret;
	}
	rsc_chown_to_system(rsc_root_dir, &rsc_binder_event_attr.attr);

	/* create sysfs binder_cmd */
	ret = sysfs_create_file(rsc_root_dir, &rsc_binder_cmd_attr.attr);
	if (ret) {
		pr_err("creat sysfs file binder_cmd failed\n");
		sysfs_remove_file(rsc_root_dir, &rsc_binder_event_attr.attr);
		return ret;
	}
	rsc_chown_to_system(rsc_root_dir, &rsc_binder_cmd_attr.attr);

#elif defined(BINDER_EVENT_USE_PROCFS)

	umode_t cMode = 0660;
	struct proc_dir_entry *_entry;
	struct proc_dir_entry *root = NULL;

	if (vivo_rsc)
		root = vivo_rsc;

	_entry = proc_create_data(__stringify(BINDER_EVENT_NAME),
			cMode, root, &binder_event_fops, (void *)&bf_mdata);

	if (_entry == NULL) {
		pr_warn(" binder_event: Create file /proc/%s error!\n", __stringify(BINDER_EVENT_NAME));
		return -ENOMEM;
	}
	proc_set_user(_entry, SYSTEM_ID, SYSTEM_GROUP_ID);
	pr_info(" binder_event: Create file /proc/%s!\n", __stringify(BINDER_EVENT_NAME));

	_entry = proc_create_data(__stringify(BINDER_CMD_NAME),
			cMode, root, &binder_cmd_fops, (void *)&bf_mdata);
	if (_entry == NULL) {
		pr_warn(" binder_event: Create file /proc/%s error!\n", __stringify(BINDER_CMD_NAME));
		remove_proc_entry(__stringify(BINDER_EVENT_NAME), root);
		return -ENOMEM;
	}
	proc_set_user(_entry, SYSTEM_ID, SYSTEM_GROUP_ID);
	pr_info(" binder_event: Create dir /proc/%s!\n", __stringify(BINDER_CMD_NAME));

#endif

	bf_mdata.bbl_switch = 0;
	bf_mdata.r_idx = 0;
	bf_mdata.w_idx = 0;
	atomic_set(&bf_mdata.cnt, 0);
	atomic_set(&bf_mdata.idx, -1); /* so that atomic_inc_return will return 0 at first time */
	spin_lock_init(&bf_mdata.bf_lock);

	pr_info("%s finish!\n", __func__);
	bf_mdata.initialized = 1;
	return (int)ret;
}

static void __exit rsc_binder_event_notif_exit(void)
{
	if (bf_mdata.initialized) {
		bf_mdata.initialized = 0;
#ifdef BINDER_EVENT_USE_SYSFS
		sysfs_remove_file(rsc_root_dir, &rsc_binder_event_attr.attr);

		sysfs_remove_file(rsc_root_dir, &rsc_binder_cmd_attr.attr);
#elif defined(BINDER_EVENT_USE_PROCFS)
		struct proc_dir_entry *root;
		if (vivo_rsc)
			root = vivo_rsc;
		remove_proc_entry(__stringify(BINDER_EVENT_NAME), root);
		remove_proc_entry(__stringify(BINDER_CMD_NAME), root);
#endif
	}
}

module_init(rsc_binder_event_notif_init);
module_exit(rsc_binder_event_notif_exit);
