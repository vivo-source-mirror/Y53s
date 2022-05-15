/*
 * mm/vivo_mem_monitor/mem_monitor.c
 *
 * VIVO Kernel Monitor Engine(Memory)
 *
 * Copyright (C) 2019 VIVO Technology Co., Ltd
 * <rongqianfeng@vivo.com>
*/
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/mmzone.h>
#include <linux/rbtree.h>
#include <linux/seq_file.h>
#include <linux/sort.h>
#include <linux/pid.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#include <linux/mem_monitor.h>

/*KB*/
#define ONE_GB (1*1024*1024UL)
#define TWO_GB (2*1024*1024UL)
#define THREE_GB (3*1024*1024UL)
#define FOUR_GB (4*1024*1024UL)
#define FIVE_GB (5*1024*1024UL)
#define SEVEN_GB (7*1024*1024UL)

#define ONE_GB_PAGES (ONE_GB  >> (PAGE_SHIFT - 10))
#define TWO_GB_PAGES (TWO_GB >> (PAGE_SHIFT - 10))
#define THREE_GB_PAGES (THREE_GB  >> (PAGE_SHIFT - 10))
#define FOUR_GB_PAGES (FOUR_GB  >> (PAGE_SHIFT - 10))
#define FIVE_GB_PAGES (FIVE_GB  >> (PAGE_SHIFT - 10))
#define SEVEN_GB_PAGES (SEVEN_GB  >> (PAGE_SHIFT - 10))

struct kobject *kmm_kobj;

//static int slabtrace_enable;
static int slab_monitor_enable = 1;
static int page_table_moitor_enable = 1;
static int kernel_stack_monitor_enable = 1;
static unsigned int expires_sec = 60;
static unsigned long slab_unreclaim_limit;
static unsigned long page_table_limit;
static unsigned long kernel_stack_limit;
static struct timer_list monitor_timer;

static bool is_slab_leak(unsigned long *slab_unreclaim)
{
	if (!slab_monitor_enable || !slab_unreclaim_limit)
		return false;

	*slab_unreclaim = global_zone_page_state(NR_PAGETABLE);
	/*pages to KB*/
	*slab_unreclaim <<= (PAGE_SHIFT - 10);

	if (*slab_unreclaim > slab_unreclaim_limit)
		return true;

	return false;
}

static bool is_page_table_leak(unsigned long *page_table)
{
	if (!page_table_moitor_enable || !page_table_limit)
		return false;

	*page_table = global_zone_page_state(NR_PAGETABLE);
	/*pages to KB*/
	*page_table <<= (PAGE_SHIFT - 10);

	if (*page_table > page_table_limit)
		return true;

	return false;
}

static bool is_kernel_stack_leak(unsigned long *kernel_stack)
{
	if (!kernel_stack_monitor_enable || !kernel_stack_limit)
		return false;

	*kernel_stack = global_zone_page_state(NR_KERNEL_STACK_KB);

	if (*kernel_stack > kernel_stack_limit)
		return true;

	return false;
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
static void monitor_handler(struct timer_list *timer)
#else
static void monitor_handler(unsigned long timer)
#endif
{
	unsigned long slab_unreclaim = 0;
	unsigned long page_table = 0;
	unsigned long kernel_stack = 0;

	pr_info ("[mm momitor] into slab_monitor_handler");
	if (is_slab_leak(&slab_unreclaim)) {
		pr_err ("[mm momitor] SUnreclaim %luKB is over the limit %luKB \n",
			slab_unreclaim, slab_unreclaim_limit);
		show_mem(0, NULL);
		panic("slab memory leak");
	}

	if (is_page_table_leak(&page_table)) {
		pr_err ("[mm momitor] page_table %luKB is over the limit %luKB \n",
			page_table, page_table_limit);
		show_mem(0, NULL);
		panic("page table memory leak");
	}

	if (is_kernel_stack_leak(&kernel_stack)) {
		pr_err ("[mm momitor] kernel_stack %luKB is over the limit %luKB \n",
			kernel_stack, kernel_stack_limit);
		show_mem(0, NULL);
		panic("kernel stack memory leak");
	}
	pr_info ("[mm momitor] out slab_monitor_handler");

	if (!timer_pending(&monitor_timer))
		mod_timer(&monitor_timer, jiffies + msecs_to_jiffies(expires_sec * 1000));
}

static void irq_timer_setup(unsigned char nsec)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	timer_setup(&monitor_timer, monitor_handler, 0);
#else
	init_timer(&monitor_timer);
	monitor_timer.function = monitor_handler;
#endif
	monitor_timer.expires = jiffies + HZ * nsec;
	add_timer(&monitor_timer);
}

/*slab monitor*/
static ssize_t SUnreclaim_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", slab_unreclaim_limit);
}

static ssize_t SUnreclaim_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	unsigned long temp_size;

	ret = kstrtoul(buf, 10, &temp_size);
	if (temp_size <= 0) {
		pr_err("[mm momitor] change slab_unreclaim_limit error, size %ld < 0 ", temp_size);
		return ret;
	}
	slab_unreclaim_limit = temp_size;
	pr_info("[mm momitor] change slab_unreclaim_limit to %ld", slab_unreclaim_limit);
	return count;
}

static ssize_t slab_enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", slab_monitor_enable);
}

static ssize_t slab_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int temp_enable;

	ret = kstrtoint(buf, 10, &temp_enable);

	if (temp_enable < 0)
		return ret;
	if (!temp_enable)
		slab_monitor_enable = 0;
	else
		slab_monitor_enable = 1;

	return count;
}

static ssize_t expires_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", expires_sec);
}

static ssize_t expires_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	unsigned int temp_expires;

	ret = kstrtoint(buf, 10, &temp_expires);

	if (temp_expires <= 0)
		return ret;
	else
		expires_sec = temp_expires;

	return count;
}

static struct kobj_attribute slab_unreclaim_limit_attr =
	__ATTR(slab_unreclaim_limit, 0640, SUnreclaim_show, SUnreclaim_store);
static struct kobj_attribute slab_enable_attr =
	__ATTR(enable, 0640, slab_enable_show, slab_enable_store);

/*for slab page_table kerenl_stack*/
static struct kobj_attribute expires_attr =
	__ATTR(expires_sec, 0640, expires_show, expires_store);

static struct attribute *slab_monitor_attrs[] = {
	&slab_unreclaim_limit_attr.attr,
	&slab_enable_attr.attr,
	&expires_attr.attr,
	NULL,
};

static struct attribute_group attr_slab_group = {
	.name = "slab",
	.attrs = slab_monitor_attrs,
};

/*page table monitor*/
static ssize_t page_table_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", page_table_limit);
}

static ssize_t page_table_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	unsigned long temp_size;

	ret = kstrtoul(buf, 10, &temp_size);
	if (temp_size <= 0) {
		pr_err("[mm momitor] change page_table_limit error, size %ld < 0 ", temp_size);
		return ret;
	}
	page_table_limit = temp_size;
	pr_info("[mm momitor] change page_table_limit to %ld", page_table_limit);
	return count;
}

static ssize_t page_table_enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", page_table_moitor_enable);
}

static ssize_t page_table_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int temp_enable;

	ret = kstrtoint(buf, 10, &temp_enable);

	if (temp_enable < 0)
		return ret;
	if (!temp_enable)
		page_table_moitor_enable = 0;
	else
		page_table_moitor_enable = 1;

	return count;
}

static struct kobj_attribute page_table_limit_attr =
	__ATTR(page_table_limit, 0640, page_table_show, page_table_store);
static struct kobj_attribute page_table_enable_attr =
	__ATTR(enable, 0640, page_table_enable_show, page_table_enable_store);

static struct attribute *page_table_monitor_attrs[] = {
	&page_table_limit_attr.attr,
	&page_table_enable_attr.attr,
	&expires_attr.attr,
	NULL,
};

static struct attribute_group attr_page_table_group = {
	.name = "page_table",
	.attrs = page_table_monitor_attrs,
};

/*kernel stack monitor*/
static ssize_t kernel_stack_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%lu\n", page_table_limit);
}

static ssize_t kernel_stack_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	unsigned long temp_size;

	ret = kstrtoul(buf, 10, &temp_size);
	if (temp_size <= 0) {
		pr_err("[mm momitor] change page_table_limit error, size %ld < 0 ", temp_size);
		return ret;
	}
	kernel_stack_limit = temp_size;
	pr_info("[mm momitor] change page_table_limit to %ld", kernel_stack_limit);
	return count;
}

static ssize_t kernel_stack_enable_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", kernel_stack_monitor_enable);
}

static ssize_t kernel_stack_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	int temp_enable;

	ret = kstrtoint(buf, 10, &temp_enable);

	if (temp_enable < 0)
		return ret;
	if (!temp_enable)
		kernel_stack_monitor_enable = 0;
	else
		kernel_stack_monitor_enable = 1;

	return count;
}

static struct kobj_attribute kernel_stack_limit_attr =
	__ATTR(kernel_stack_limit, 0640, kernel_stack_show, kernel_stack_store);
static struct kobj_attribute kernel_stack_enable_attr =
	__ATTR(enable, 0640, kernel_stack_enable_show, kernel_stack_enable_store);

static struct attribute *kernel_stack_monitor_attrs[] = {
	&kernel_stack_limit_attr.attr,
	&kernel_stack_enable_attr.attr,
	&expires_attr.attr,
	NULL,
};

static struct attribute_group attr_kernel_stack_group = {
	.name = "kernel_stack",
	.attrs = kernel_stack_monitor_attrs,
};

static void memory_leak_water_mark(void)
{
	/*because we open the slab debug so need expand the leak water mark*/
	if (totalram_pages > FIVE_GB_PAGES)
		slab_unreclaim_limit = 1536*1024; //1.5G
	else if (totalram_pages > THREE_GB_PAGES)
		slab_unreclaim_limit = 1024*1024; //1G
	else if (totalram_pages > TWO_GB_PAGES)
		slab_unreclaim_limit = 820*1024; //820M
	else if (totalram_pages > ONE_GB_PAGES)
		slab_unreclaim_limit = 700*1024; //700M  Ben200320-026 leak 766616 kB

#ifdef CONFIG_DEBUG_KMEMLEAK
	if (kmemleak_skip_disable) {
		pr_info("kmemleak is enabled");
		slab_unreclaim_limit *= 2;
	}
#endif

/*#ifdef CONFIG_MTK_MEMCFG
	if (slabtrace_enable)
		slab_unreclaim_limit += ONE_GB/2;
#endif*/

	/* Ben200228-083 12452 thread, kernel_stack 198320KB. limit 10000 threads, use KB */
	kernel_stack_limit = THREAD_SIZE*10000/1024;

	/* Ben200228-083 12452 thread, PageTables 964068KB. limit 700MB, use KB */
	page_table_limit = 700*1024;
}

static int __init mem_mnt_init(void)
{
	int ret;

	if (!bsp_test_mode)
		return 0;

	kmm_kobj = kobject_create_and_add("mem_monitor", kernel_kobj);
	if (!kmm_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(kmm_kobj, &attr_slab_group);
	if (ret)
		goto slab_failed;

	ret = sysfs_create_group(kmm_kobj, &attr_page_table_group);
	if (ret)
		goto page_table_failed;

	ret = sysfs_create_group(kmm_kobj, &attr_kernel_stack_group);
	if (ret)
		goto kernel_stack_failed;

	memory_leak_water_mark();

	irq_timer_setup(expires_sec);

	return ret;

kernel_stack_failed:
	sysfs_remove_group(kmm_kobj, &attr_page_table_group);
page_table_failed:
	sysfs_remove_group(kmm_kobj, &attr_slab_group);
slab_failed:
	kobject_put(kmm_kobj);

	return ret;
}

static void __exit mem_mnt_exit(void)
{
	if (bsp_test_mode) {
		sysfs_remove_group(kmm_kobj, &attr_kernel_stack_group);
		sysfs_remove_group(kmm_kobj, &attr_page_table_group);
		sysfs_remove_group(kmm_kobj, &attr_slab_group);
		kobject_put(kmm_kobj);
		del_timer_sync(&monitor_timer);
	}
}

module_init(mem_mnt_init);
module_exit(mem_mnt_exit);
MODULE_LICENSE("GPL v2");
