/*
 *  linux/drivers/pem/cpulimit/cpulimit.c
 *
 *  Copyright 2017-2017 vivo pem daizhihui
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <pem.h>
#include <linux/profile.h>
static const char *cpulimit_name = "cpulimit";

struct pid_data {
	struct list_head list;
	struct task_struct *task;
	int pid;
	unsigned long expires;
	unsigned long sleep_time;
	unsigned long run_time;
	bool stop;
	volatile bool isVaild;
};

struct pem_cpulimit_data {
	struct pid_data head;
	struct mutex lock;
	spinlock_t spinlock;
	struct timer_list timer;
	unsigned long expires;
	char *store_buf;
	size_t buf_size;
	bool stop;
	struct notifier_block cpulimit_notifier_block;
};
static struct pem_cpulimit_data cpulimit_data = {
	.buf_size = 0,
};

static void pem_timer_do(unsigned long data)
{
	struct pid_data *tmp;
	struct list_head *n, *next_list;
	unsigned long curJiffies = jiffies;

	cpulimit_data.expires = curJiffies + 10000000;
	list_for_each_safe(next_list, n, &cpulimit_data.head.list) {
		tmp = list_entry(next_list, struct pid_data, list);
		if (tmp->task == NULL) {
			tmp->isVaild = false;
			list_del(&tmp->list);
			pem_info(" pem_driver: pem_timer_do del because died, pid(%d)\n", tmp->pid);
			kfree(tmp);
			continue;
		}
		if (tmp->expires <= curJiffies) {
			if (tmp->stop) {
				tmp->stop = false;
				force_sig(SIGCONT, tmp->task);
				tmp->expires = curJiffies + tmp->run_time;
			} else {
				tmp->stop = true;
				if (tmp->isVaild) {
					if (tmp->task->state == TASK_RUNNING || tmp->task->state == TASK_INTERRUPTIBLE || tmp->task->state == TASK_UNINTERRUPTIBLE) {
						force_sig(SIGSTOP, tmp->task);
					}
				}
				tmp->expires = curJiffies + tmp->sleep_time;
			}
		}
		if (cpulimit_data.expires > tmp->expires) {
			cpulimit_data.expires = tmp->expires;
			cpulimit_data.stop = tmp->stop;
		}
	}
	mod_timer(&cpulimit_data.timer, cpulimit_data.expires);
}

static int pem_proc_show(struct seq_file *m, void *v)
{
	struct pid_data *tmp;
	struct list_head *n, *next_list;
	unsigned long curJiffies = jiffies;

	seq_puts(m, "pids   sleep  run   stop   expires\n");
	list_for_each_safe(next_list, n, &cpulimit_data.head.list) {
		tmp = list_entry(next_list, struct pid_data, list);
		seq_printf(m, "%d	%ld	%ld	%d	%ld\n", tmp->pid, tmp->sleep_time,
				tmp->run_time, tmp->stop, (tmp->expires-curJiffies));
	}
	return 0;
}

static int pem_proc_open(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver: pem_proc_open\n");
	return single_open(file, pem_proc_show, (void *)&cpulimit_data);
}

static inline char *findNext(char *s, char *e)
{
	while (s < e) {
		if (*s == ',')
			return s + 1;
		if (*s == '\0')
			return NULL;
		s++;
	}
	return NULL;
}

static ssize_t pem_write_proc(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	int pid, result;
	char *off_tt;
	char *off_end;
	unsigned long curjiffies;
	struct task_struct *new_task;
	unsigned int sleep_base, run_base;
	struct pid_data *tmp;
	struct list_head *n, *next_list;

	if (count < 1)
		return -EINVAL;
	if (cpulimit_data.buf_size < count) {
		if (cpulimit_data.store_buf != NULL)
			kfree(cpulimit_data.store_buf);
		cpulimit_data.store_buf = kmalloc(count+1, GFP_KERNEL);
		if (cpulimit_data.store_buf == NULL)
			return -EINVAL;
		cpulimit_data.buf_size = count+1;
	}

	mutex_lock(&cpulimit_data.lock);
	if (cpulimit_data.store_buf != NULL) {
		memset(cpulimit_data.store_buf, 0, cpulimit_data.buf_size);
	}
	if (copy_from_user(cpulimit_data.store_buf, user_buff, count)) {
		pem_warn(" pem_driver: copy_from_user fail\n");
		mutex_unlock(&cpulimit_data.lock);
		return -EINVAL;
	}

	curjiffies = jiffies;
	off_tt = cpulimit_data.store_buf;
	off_end = cpulimit_data.store_buf + count;
	if (cpulimit_data.expires < curjiffies || cpulimit_data.expires - curjiffies > 1000)
		cpulimit_data.expires = curjiffies + msecs_to_jiffies(HZ);
	while (true) {
		tmp = NULL;
		result = sscanf(off_tt, "%d %u %u", &pid, &sleep_base, &run_base);
		if (result < 3)
			break;
		list_for_each_safe(next_list, n, &cpulimit_data.head.list) {
			tmp = list_entry(next_list, struct pid_data, list);
			if (tmp->pid == pid)
				break;
			tmp = NULL;
		}
		if (sleep_base == 0 || run_base == 0) {
			if (tmp != NULL) {
				tmp->isVaild = false;
				list_del(&tmp->list);
				if (tmp->stop) {
					tmp->task = pid_task(find_vpid(tmp->pid), PIDTYPE_PID);
					if (tmp->task != NULL)
						force_sig(SIGCONT, tmp->task);
				}
				kfree(tmp);
				pem_info(" pem_driver: pem_write_proc del pid(%d)\n", pid);
			}
		} else {
			if (tmp == NULL) {
				new_task = pid_task(find_vpid(pid), PIDTYPE_PID);
				if (new_task != NULL) {
					tmp = kmalloc(sizeof(struct pid_data), GFP_KERNEL);
					if (tmp != NULL) {
						tmp->pid = pid;
						tmp->task = new_task;
						tmp->stop = false;
						tmp->isVaild = true;
						list_add(&tmp->list, &cpulimit_data.head.list);
					}
				}
			}
			if (tmp != NULL) {
				tmp->run_time = msecs_to_jiffies(HZ*run_base);
				tmp->sleep_time = msecs_to_jiffies(HZ*sleep_base);
				tmp->expires = cpulimit_data.expires;
				pem_info(" pem_driver: pem_write_proc add pid(%d), sleep(%ld ms), run(%ld ms), expires(%ld)\n",
						pid, tmp->sleep_time, tmp->run_time, tmp->expires);
			}
		}
		off_tt = findNext(off_tt, off_end);
		if (off_tt == NULL)
			break;
	}
	if (cpulimit_data.timer.expires != cpulimit_data.expires)
		mod_timer(&cpulimit_data.timer, cpulimit_data.expires);
	mutex_unlock(&cpulimit_data.lock);

	return count;
}

static const struct file_operations pem_proc_fops = {
	.owner = THIS_MODULE,
	.open = pem_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = pem_write_proc,
	.release = single_release,
};

int cpulimit_process_notifier(struct notifier_block *self, unsigned long cmd, void *v)
{
	struct pid_data *tmp;
	struct list_head *n, *next_list;
	struct task_struct *task = v;

	if (task) {
		list_for_each_safe(next_list, n, &cpulimit_data.head.list) {
			tmp = list_entry(next_list, struct pid_data, list);
			if (task->pid == tmp->pid) {
				tmp->task = NULL;
				return NOTIFY_OK;
			}
		}
	}

	return NOTIFY_OK;
}


int cpulimit_init(struct proc_dir_entry *root)
{
	umode_t cMode = 0666;
	struct proc_dir_entry *pem_cpulimit = proc_create_data(cpulimit_name,
			cMode, root, &pem_proc_fops, (void *)&cpulimit_data);

	if (pem_cpulimit == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", cpulimit_name);
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", cpulimit_name);

	INIT_LIST_HEAD(&(cpulimit_data.head.list));
	mutex_init(&cpulimit_data.lock);
	spin_lock_init(&cpulimit_data.spinlock);
	init_timer(&cpulimit_data.timer);
	cpulimit_data.expires = 0;
	cpulimit_data.stop = false;
	cpulimit_data.timer.function = &pem_timer_do;
	cpulimit_data.timer.data = (unsigned long)(&cpulimit_data);

	cpulimit_data.cpulimit_notifier_block.notifier_call = cpulimit_process_notifier;
	profile_event_register(PROFILE_TASK_EXIT, &cpulimit_data.cpulimit_notifier_block);

	return 0;
}

void cpulimit_remove(struct proc_dir_entry *root)
{
	struct pid_data *tmp;
	struct list_head *n, *next_list;

	profile_event_unregister(PROFILE_TASK_EXIT, &cpulimit_data.cpulimit_notifier_block);

	remove_proc_entry(cpulimit_name, root);
	if (cpulimit_data.store_buf != NULL)
		kfree(cpulimit_data.store_buf);
	del_timer(&cpulimit_data.timer);
	list_for_each_safe(next_list, n, &cpulimit_data.head.list) {
		tmp = list_entry(next_list, struct pid_data, list);
		list_del(&tmp->list);
		if (tmp->stop) {
			tmp->task = pid_task(find_vpid(tmp->pid), PIDTYPE_PID);
			if (tmp->task != NULL)
				force_sig(SIGCONT, tmp->task);
		}
		pem_info(" pem_driver: pem_remove del pid(%d)\n", tmp->pid);
		kfree(tmp);
	}
}

void execCpulimitCmd(const struct pem_cmd *cmd)
{
	pem_info(" pem_driver: cpulimit can not suppot cmd(%c)\n", cmd->what);
}

struct pem_module *cpulimit_attach(struct proc_dir_entry *root)
{
	struct pem_module *mdl = kmalloc(sizeof(struct pem_module), GFP_KERNEL);

	if (mdl == NULL)
		return NULL;
	if (cpulimit_init(root) < 0) {
		kfree(mdl);
		return NULL;
	}
	mdl->fun = '1';
	mdl->next = NULL;
	mdl->execCmd = execCpulimitCmd;
	mdl->remove = cpulimit_remove;
	return mdl;
}
