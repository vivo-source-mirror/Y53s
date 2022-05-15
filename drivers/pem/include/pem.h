/*
 *  driver/pem/include/pem.h
 *
 *  Copyright 2017-2017 vivo pem daizhihui
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_PEM_H
#define __LINUX_PEM_H

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>

struct pem_cmd {
	char fun;
	char what;
	char *args[20];
	int argCnt;
};
#define MAX_ARGS 20

struct pem_module {
	char fun;
	struct pem_module *next;
	void (*remove)(struct proc_dir_entry *root);
	void (*execCmd)(const struct pem_cmd *cmd);
};

#define pem_info(fmt, args...) pr_info(fmt, ##args)
#define pem_warn(fmt, args...) pr_warn(fmt, ##args)
#define PEM_ATTACH(module_name, root) module_name##_attach(root)
#define DECLARATION_ATTACH(module_name) extern struct pem_module *module_name##_attach(struct proc_dir_entry *root)


extern void addPemEvent(char which, const char *fmt, ...);
extern void addPemEventNoArgs(char which);

#if IS_ENABLED(CONFIG_CGROUP_NET_PRIO)
extern void skb_prioidx_spin_lock_init(void);
extern void set_skb_fg_uid(u32 *data, u32 len);
#endif

#ifdef CONFIG_NET_SPEED_LIMIT
extern void set_net_speedlimit_uid(u32 *data, u32 len);
extern void set_net_speedlimit_windows(__be16 data);
#endif

#endif /* __LINUX_PEM_H */
