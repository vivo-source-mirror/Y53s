// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2019, The Linux Foundation. All rights reserved.
 *
 * author:wangweiguo, reason: add bsptest record info.
 * date:2021-02-20
 */

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#define BSPTEST_INFO_MAX	(1024)

static char bsptest_info[BSPTEST_INFO_MAX] = {0};

static int bsptest_info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", bsptest_info);
	return 0;
}

static int bsptest_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, bsptest_info_show, NULL);
}

static ssize_t bsptest_info_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
    if (count >= BSPTEST_INFO_MAX) {
		printk(KERN_ERR "BSPTest: write string too long(%d)\n", count);
		return -ENOSPC;
	}
	memset(bsptest_info, 0, count);
	if(copy_from_user(bsptest_info, buf, count - 1)) {
		printk(KERN_ERR "BSPTest: write(%d) failed!!!\n", count);
		return -EFAULT;
	}
	printk(KERN_INFO "BSPTest:%s\n", bsptest_info);
	return count;
}

static const struct file_operations proc_bsptest_info_operations = {
	.open 		= bsptest_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= bsptest_info_write,
};

static int __init proc_bsptest_info_init(void)
{
	if(!proc_create("bsptest_info", S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, NULL,
			&proc_bsptest_info_operations)) {
		printk(KERN_ERR "BSPTest: Failed to register proc interface\n");
		return -EFAULT;
	}
	return 0;
}

fs_initcall(proc_bsptest_info_init);

