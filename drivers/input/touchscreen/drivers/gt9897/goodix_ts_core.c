 /*
  * Goodix Touchscreen Driver
  * Copyright (C) 2020 - 2021 Goodix, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be a reference
  * to you, when you are integrating the GOODiX's CTP IC into your system,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * General Public License for more details.
  *
  */
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif

#include "goodix_ts_core.h"

#define GOOIDX_INPUT_PHYS	"goodix_ts/input0"

struct goodix_module goodix_modules_gt9897;

#define CORE_MODULE_UNPROBED     0
#define CORE_MODULE_PROB_SUCCESS 1
#define CORE_MODULE_PROB_FAILED -1
#define CORE_MODULE_REMOVED     -2
int core_module_prob_sate = CORE_MODULE_UNPROBED;

#define to_ts_core(vts_dev)	container_of(vts_dev,struct goodix_ts_core, vtsdev)

/**
 * __do_register_ext_module - register external module
 * to register into touch core modules structure
 * return 0 on success, otherwise return < 0
 */
static int __do_register_ext_module(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module, *next;
	struct list_head *insert_point = &goodix_modules_gt9897.head;

	ts_info("__do_register_ext_module IN");
	/* prority level *must* be set */
	if (module->priority == EXTMOD_PRIO_RESERVED) {
		ts_err("Priority of module [%s] needs to be set",
		       module->name);
		return -EINVAL;
	}
	mutex_lock(&goodix_modules_gt9897.mutex);
	/* find insert point for the specified priority */
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (ext_module == module) {
				ts_info("Module [%s] already exists",
					module->name);
				mutex_unlock(&goodix_modules_gt9897.mutex);
				return 0;
			}
		}

		/* smaller priority value with higher priority level */
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (ext_module->priority >= module->priority) {
				insert_point = &ext_module->list;
				break;
			}
		}
	}

	if (module->funcs && module->funcs->init) {
		if (module->funcs->init(goodix_modules_gt9897.core_data,
					module) < 0) {
			ts_err("Module [%s] init error",
			       module->name ? module->name : " ");
			mutex_unlock(&goodix_modules_gt9897.mutex);
			return -EFAULT;
		}
	}

	list_add(&module->list, insert_point->prev);
	mutex_unlock(&goodix_modules_gt9897.mutex);

	ts_info("Module [%s] registered,priority:%u", module->name,
		module->priority);
	return 0;
}

static void goodix_register_ext_module_work(struct work_struct *work) {
	struct goodix_ext_module *module =
			container_of(work, struct goodix_ext_module, work);

	ts_info("module register work IN");
	if (core_module_prob_sate == CORE_MODULE_PROB_FAILED
		|| core_module_prob_sate == CORE_MODULE_REMOVED) {
		ts_err("core layer state error %d", core_module_prob_sate);
		return;
	}

	if (core_module_prob_sate == CORE_MODULE_UNPROBED) {
		/* waitting for core layer */
		if (!wait_for_completion_timeout(&goodix_modules_gt9897.core_comp,
						 25 * HZ)) {
			ts_err("Module [%s] timeout", module->name);
			return;
		}
	}

	/* driver probe failed */
	if (core_module_prob_sate != CORE_MODULE_PROB_SUCCESS) {
		ts_err("Can't register ext_module core error");
		return;
	}

	if (__do_register_ext_module(module))
		ts_err("failed register module: %s", module->name);
	else
		ts_info("success register module: %s", module->name);
}

static void goodix_core_module_init(void)
{
	if (goodix_modules_gt9897.initilized)
		return;
	goodix_modules_gt9897.initilized = true;
	INIT_LIST_HEAD(&goodix_modules_gt9897.head);
	mutex_init(&goodix_modules_gt9897.mutex);
	init_completion(&goodix_modules_gt9897.core_comp);
}

/**
 * goodix_register_ext_module - interface for register external module
 * to the core. This will create a workqueue to finish the real register
 * work and return immediately. The user need to check the final result
 * to make sure registe is success or fail.
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int gt9897_register_ext_module(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;

	ts_info("goodix_register_ext_module IN");

	goodix_core_module_init();
	INIT_WORK(&module->work, goodix_register_ext_module_work);
	schedule_work(&module->work);

	ts_info("goodix_register_ext_module OUT");
	return 0;
}
EXPORT_SYMBOL_GPL(gt9897_register_ext_module);

/**
 * goodix_register_ext_module_no_wait
 * return: 0 ok, <0 failed
 */
int gt9897_register_ext_module_no_wait(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;
	ts_info("goodix_register_ext_module_no_wait IN");
	goodix_core_module_init();
	/* driver probe failed */
	if (core_module_prob_sate != CORE_MODULE_PROB_SUCCESS) {
		ts_err("Can't register ext_module core error");
		return -EINVAL;
	}
	return __do_register_ext_module(module);
}
EXPORT_SYMBOL_GPL(gt9897_register_ext_module_no_wait);

/* remove all registered ext module
 * return 0 on success, otherwise return < 0
 */
int goodix_gt9897_unregister_all_module(void)
{
	int ret = 0;
	struct goodix_ext_module *ext_module, *next;

	if (!goodix_modules_gt9897.initilized)
		return 0;

	if (!goodix_modules_gt9897.core_data)
		return 0;

	mutex_lock(&goodix_modules_gt9897.mutex);
	if (list_empty(&goodix_modules_gt9897.head)) {
		mutex_unlock(&goodix_modules_gt9897.mutex);
		return 0;
	}

	list_for_each_entry_safe(ext_module, next,
				 &goodix_modules_gt9897.head, list) {
		if (ext_module->funcs && ext_module->funcs->exit) {
			ret = ext_module->funcs->exit(goodix_modules_gt9897.core_data,
					ext_module);
			if (ret) {
				ts_err("failed register ext module, %d:%s",
					ret, ext_module->name);
				break;
			}
		}
		ts_info("remove module: %s", ext_module->name);
		list_del(&ext_module->list);
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);
	return ret;
}

/**
 * goodix_unregister_ext_module - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int gt9897_unregister_ext_module(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module, *next;
	bool found = false;

	if (!module)
		return -EINVAL;

	if (!goodix_modules_gt9897.initilized)
		return -EINVAL;

	if (!goodix_modules_gt9897.core_data)
		return -ENODEV;

	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (ext_module == module) {
				found = true;
				break;
			}
		}
	} else {
		mutex_unlock(&goodix_modules_gt9897.mutex);
		return 0;
	}

	if (!found) {
		ts_debug("Module [%s] never registed",
				module->name);
		mutex_unlock(&goodix_modules_gt9897.mutex);
		return 0;
	}

	list_del(&module->list);
	mutex_unlock(&goodix_modules_gt9897.mutex);

	if (module->funcs && module->funcs->exit)
		module->funcs->exit(goodix_modules_gt9897.core_data, module);

	ts_info("Moudle [%s] unregistered",
		module->name ? module->name : " ");
	return 0;
}
EXPORT_SYMBOL_GPL(gt9897_unregister_ext_module);

static void goodix_ext_sysfs_release(struct kobject *kobj)
{
	ts_info("Kobject released!");
}

#define to_ext_module(kobj)	container_of(kobj,\
				struct goodix_ext_module, kobj)
#define to_ext_attr(attr)	container_of(attr,\
				struct goodix_ext_attribute, attr)

static ssize_t goodix_ext_sysfs_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->show)
		return ext_attr->show(module, buf);

	return -EIO;
}

static ssize_t goodix_ext_sysfs_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->store)
		return ext_attr->store(module, buf, count);

	return -EIO;
}

static const struct sysfs_ops goodix_ext_ops = {
	.show = goodix_ext_sysfs_show,
	.store = goodix_ext_sysfs_store
};

static struct kobj_type goodix_ext_ktype = {
	.release = goodix_ext_sysfs_release,
	.sysfs_ops = &goodix_ext_ops,
};

struct kobj_type *gt9897_get_default_ktype(void)
{
	return &goodix_ext_ktype;
}
EXPORT_SYMBOL_GPL(gt9897_get_default_ktype);

struct kobject *gt9897_get_default_kobj(void)
{
	struct kobject *kobj = NULL;

	if (goodix_modules_gt9897.core_data &&
			goodix_modules_gt9897.core_data->pdev)
		kobj = &goodix_modules_gt9897.core_data->pdev->dev.kobj;
	return kobj;
}
EXPORT_SYMBOL_GPL(gt9897_get_default_kobj);


/* show external module infomation */
static ssize_t goodix_ts_extmod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ext_module *module, *next;
	size_t offset = 0;
	int r;

	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(module, next,
					 &goodix_modules_gt9897.head, list) {
			r = snprintf(&buf[offset], PAGE_SIZE,
				     "priority:%u module:%s\n",
				     module->priority, module->name);
			if (r < 0) {
				mutex_unlock(&goodix_modules_gt9897.mutex);
				return -EINVAL;
			}
			offset += r;
		}
	}

	mutex_unlock(&goodix_modules_gt9897.mutex);
	return offset;
}

/* show driver infomation */
static ssize_t goodix_ts_driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t goodix_ts_chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_fw_version chip_ver;
	int ret, cnt = -EINVAL;

	if (hw_ops->read_version) {
		ret = hw_ops->read_version(core_data, &chip_ver);
		if (!ret) {
			cnt = snprintf(&buf[cnt], PAGE_SIZE,
				"rom_pid:%s\nrom_vid:%02x%02x%02x\n",
				chip_ver.rom_pid, chip_ver.rom_vid[0],
				chip_ver.rom_vid[1], chip_ver.rom_vid[2]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"patch_pid:%s\npatch_vid:%02x%02x%02x%02x\n",
				chip_ver.patch_pid, chip_ver.patch_vid[0],
				chip_ver.patch_vid[1], chip_ver.patch_vid[2],
				chip_ver.patch_vid[3]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"sensorid:%d\n", chip_ver.sensor_id);			
		}
	}

	return cnt;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	if (!buf || count <= 0)
		return -EINVAL;
	if (buf[0] != '0')
		hw_ops->reset(core_data, GOODIX_NORMAL_RESET_DELAY_MS);
	return count;
}

/* show irq infomation */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		     atomic_read(&core_data->irq_enabled) ?
		     "enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		     desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
		     "echo 0/1 > irq_info to disable/enable irq");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		hw_ops->irq_enable(core_data, true);
	else
		hw_ops->irq_enable(core_data, false);
	return count;
}

static DEVICE_ATTR(extmod_info, S_IRUGO, goodix_ts_extmod_show, NULL);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_ts_driver_info_show, NULL);
static DEVICE_ATTR(chip_info, S_IRUGO, goodix_ts_chip_info_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, goodix_ts_reset_store);
static DEVICE_ATTR(irq_info, S_IRUGO | S_IWUSR | S_IWGRP,
		   goodix_ts_irq_info_show, goodix_ts_irq_info_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_extmod_info.attr,
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_reset.attr,
	&dev_attr_irq_info.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	int ret;

	ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (ret) {
		ts_err("failed create core sysfs group");
		return ret;
	}

	return ret;
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

#ifdef CONFIG_DEBUG_FS
#define MAX_DEBUGFS_BUF_SIZE 4096
static struct goodix_debug_fs {
	wait_queue_head_t log_wait;

	struct dentry *rootdir;
	char buf[MAX_DEBUGFS_BUF_SIZE];
	char *r_pos;
	char *w_pos;
	char *start_pos;
	char *end_pos;
	struct mutex mutex;
} goodix_dbgfs;

#define MAX_LINE_SIZE 120
void goodix_debugfs_printf(const char *fmt, ...)
{
	char tmp_buf[MAX_LINE_SIZE] = {0};
	char *buf_ptr;
	va_list args;
	int need_copy, copy_size;

	if (!goodix_dbgfs.rootdir)
		return;

	va_start(args, fmt);
	need_copy = vscnprintf(tmp_buf,
			MAX_LINE_SIZE, fmt, args);
	va_end(args);

	mutex_lock(&goodix_dbgfs.mutex);
	buf_ptr = &tmp_buf[0];
	if (goodix_dbgfs.r_pos <= goodix_dbgfs.w_pos) {
		if (need_copy <= goodix_dbgfs.end_pos - goodix_dbgfs.w_pos) {
			memcpy(goodix_dbgfs.w_pos, buf_ptr, need_copy);
			goodix_dbgfs.w_pos += need_copy;
			if (goodix_dbgfs.w_pos >= goodix_dbgfs.end_pos) {
				goodix_dbgfs.w_pos = goodix_dbgfs.start_pos;
				/* modify r_pos */
				if (goodix_dbgfs.w_pos >= goodix_dbgfs.r_pos)
					goodix_dbgfs.r_pos =
						goodix_dbgfs.w_pos + 1;
			}
		} else {
			/* roll back */
			copy_size = goodix_dbgfs.end_pos - goodix_dbgfs.w_pos;
			memcpy(goodix_dbgfs.w_pos, buf_ptr, copy_size);
			goodix_dbgfs.w_pos = goodix_dbgfs.start_pos;
			buf_ptr += copy_size;
			need_copy -= copy_size;
			memcpy(goodix_dbgfs.w_pos, buf_ptr, need_copy);
			goodix_dbgfs.w_pos += need_copy;

			/* modify r_pos */
			if (goodix_dbgfs.w_pos >= goodix_dbgfs.r_pos)
				goodix_dbgfs.r_pos = goodix_dbgfs.w_pos + 1;
		}
	} else {
		if (need_copy < goodix_dbgfs.end_pos - goodix_dbgfs.w_pos) {
			memcpy(goodix_dbgfs.w_pos, buf_ptr, need_copy);
			goodix_dbgfs.w_pos += need_copy;
			if (goodix_dbgfs.w_pos >= goodix_dbgfs.r_pos)
				goodix_dbgfs.r_pos = goodix_dbgfs.w_pos + 1;
		} else {
			/* roll back */
			copy_size = goodix_dbgfs.end_pos - goodix_dbgfs.w_pos;
			memcpy(goodix_dbgfs.w_pos, buf_ptr, copy_size);
			goodix_dbgfs.w_pos = goodix_dbgfs.start_pos;
			goodix_dbgfs.r_pos = goodix_dbgfs.w_pos + 1;
			buf_ptr += copy_size;
			need_copy -= copy_size;
			memcpy(goodix_dbgfs.w_pos, buf_ptr, need_copy);
			goodix_dbgfs.w_pos += need_copy;
			/* modify r_pos */
			if (goodix_dbgfs.w_pos >= goodix_dbgfs.r_pos)
				goodix_dbgfs.r_pos = goodix_dbgfs.w_pos + 1;
		}
	}

	/* r_pos overflow */
	if (goodix_dbgfs.r_pos >= goodix_dbgfs.end_pos)
		goodix_dbgfs.r_pos = goodix_dbgfs.start_pos;
	mutex_unlock(&goodix_dbgfs.mutex);
	wake_up_interruptible(&goodix_dbgfs.log_wait);
}
EXPORT_SYMBOL(goodix_debugfs_printf);

static ssize_t goodix_debugfs_read(struct file *file,
			char __user *buf, size_t size, loff_t *ppos)
{
	int ret;
	ssize_t valied_data = 0;
	ssize_t copy_data = 0;

	ret = wait_event_interruptible(goodix_dbgfs.log_wait,
			      goodix_dbgfs.r_pos != goodix_dbgfs.w_pos);
	if (ret)
		return -EAGAIN;

	mutex_lock(&goodix_dbgfs.mutex);

	if (goodix_dbgfs.r_pos < goodix_dbgfs.w_pos) {
		valied_data = goodix_dbgfs.w_pos - goodix_dbgfs.r_pos;
		copy_data = valied_data < size ? valied_data : size;
		ret = copy_to_user(buf, goodix_dbgfs.r_pos, copy_data);
		copy_data -= ret;
		goodix_dbgfs.r_pos += copy_data;
		goto out;
	}

	/* to simplify code design, when divie read operation into two syscall */
	valied_data = goodix_dbgfs.end_pos - goodix_dbgfs.r_pos;
	copy_data = valied_data < size ? valied_data : size;
	ret = copy_to_user(buf, goodix_dbgfs.r_pos, copy_data);
	copy_data -= ret;
	goodix_dbgfs.r_pos += copy_data;
	if (goodix_dbgfs.r_pos >= goodix_dbgfs.end_pos)
		goodix_dbgfs.r_pos = goodix_dbgfs.start_pos;

out:
	mutex_unlock(&goodix_dbgfs.mutex);
	return copy_data;
}

static int goodix_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int goodix_debugfs_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations goodix_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = goodix_debugfs_open,
	.read = goodix_debugfs_read,
	.release = goodix_debugfs_release,
};

static int goodix_debugfs_init(struct goodix_ts_core *cd)
{
	struct dentry *rootdir;
	struct dentry *retval;

	rootdir = debugfs_create_dir("goodix_ts", NULL);
	if (!rootdir) {
		ts_err("failed create goodix debugfs dir");
		return -EINVAL;
	}
	retval = debugfs_create_file("event", S_IRUGO | S_IWUSR, rootdir,
		cd, &goodix_debugfs_ops);
	if (IS_ERR_OR_NULL(retval)) {
		debugfs_remove_recursive(rootdir);
		return -EINVAL;
	}
	init_waitqueue_head(&goodix_dbgfs.log_wait);
	goodix_dbgfs.r_pos = goodix_dbgfs.buf;
	goodix_dbgfs.w_pos = goodix_dbgfs.buf;
	goodix_dbgfs.start_pos = goodix_dbgfs.buf;
	goodix_dbgfs.end_pos = goodix_dbgfs.buf + MAX_DEBUGFS_BUF_SIZE;
	goodix_dbgfs.rootdir = rootdir;
	mutex_init(&goodix_dbgfs.mutex);

	return 0;
}

static void goodix_debugfs_remove(struct goodix_ts_core *cd)
{
	if (!goodix_dbgfs.rootdir)
		return;
	debugfs_remove_recursive(goodix_dbgfs.rootdir);
}
#else
static int goodix_debugfs_init(struct goodix_ts_core *cd)
{
	return -EINVAL;
}
static void goodix_debugfs_remove(struct goodix_ts_core *cd)
{
}
void goodix_debugfs_printf(const char *fmt, ...)
{
	return;
}
EXPORT_SYMBOL(goodix_debugfs_printf);
#endif
/* event notifier */
static BLOCKING_NOTIFIER_HEAD(ts_notifier_list);
/**
 * goodix_ts_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *  see enum ts_notify_event in goodix_ts_core.h
 */
int gt9897_ts_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(gt9897_ts_register_notifier);

/**
 * goodix_ts_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int gt9897_ts_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(gt9897_ts_unregister_notifier);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int gt9897_ts_blocking_notify(enum ts_notify_event evt, void *v)
{
	int ret;

	ret = blocking_notifier_call_chain(&ts_notifier_list,
			(unsigned long)evt, v);
	return ret;
}
EXPORT_SYMBOL_GPL(gt9897_ts_blocking_notify);

#ifdef CONFIG_OF
/**
 * goodix_parse_dt_resolution - parse resolution from dt
 * @node: devicetree node
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt_resolution(struct device_node *node,
		struct goodix_ts_board_data *board_data)
{
	int ret;

	ret = of_property_read_u32(node, "goodix,panel-max-x",
				 &board_data->panel_max_x);
	if (ret) {
		ts_err("failed get panel-max-x");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-y",
				 &board_data->panel_max_y);
	if (ret) {
		ts_err("failed get panel-max-y");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-w",
				 &board_data->panel_max_w);
	if (ret) {
		ts_err("failed get panel-max-w");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-p",
				 &board_data->panel_max_p);
	if (ret) {
		ts_err("failed get panel-max-p, use default");
		board_data->panel_max_p = GOODIX_PEN_MAX_PRESSURE;
	}

	return 0;
}

/**
 * goodix_parse_dt - parse board data from dt
 * @dev: pointer to device
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt(struct device_node *node,
	struct goodix_ts_board_data *board_data)
{
	const char *name_tmp;
	int r;

	if (!board_data) {
		ts_err("invalid board data");
		return -EINVAL;
	}

	r = of_get_named_gpio(node, "goodix,reset-gpio", 0);
	if (r < 0) {
		ts_err("invalid reset-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get reset-gpio[%d] from dt", r);
	board_data->reset_gpio = r;

	r = of_get_named_gpio(node, "goodix,irq-gpio", 0);
	if (r < 0) {
		ts_err("invalid irq-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get irq-gpio[%d] from dt", r);
	board_data->irq_gpio = r;

	r = of_property_read_u32(node, "goodix,irq-flags",
			&board_data->irq_flags);
	if (r) {
		ts_err("invalid irq-flags");
		return -EINVAL;
	}

	memset(board_data->avdd_name, 0, sizeof(board_data->avdd_name));
	r = of_property_read_string(node, "goodix,avdd-name", &name_tmp);
	if (!r) {
		ts_info("avdd name form dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->avdd_name))
			strncpy(board_data->avdd_name,
				name_tmp, sizeof(board_data->avdd_name));
		else
			ts_info("invalied avdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->avdd_name));
	}

	memset(board_data->iovdd_name, 0, sizeof(board_data->iovdd_name));
	r = of_property_read_string(node, "goodix,iovdd-name", &name_tmp);
	if (!r) {
		ts_info("iovdd name form dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->iovdd_name))
			strncpy(board_data->iovdd_name,
				name_tmp, sizeof(board_data->iovdd_name));
		else
			ts_info("invalied iovdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->iovdd_name));
	}
	/* get xyz resolutions */
	r = goodix_parse_dt_resolution(node, board_data);
	if (r) {
		ts_err("Failed to parse resolutions:%d", r);
		return r;
	}


	/*get pen-enable switch and pen keys, must after "key map"*/
	board_data->pen_enable = of_property_read_bool(node,
					"goodix,pen-enable");
	if (board_data->pen_enable)
		ts_info("goodix pen enabled");

	ts_debug("[DT]x:%d, y:%d, w:%d, p:%d", board_data->panel_max_x,
		 board_data->panel_max_y, board_data->panel_max_w,
		 board_data->panel_max_p);
	return 0;
}
#endif

static void goodix_ts_report_pen(struct input_dev *dev,
		struct goodix_pen_data *pen_data)
{
	int i;

	if (pen_data->coords.status == TS_TOUCH) {
		input_report_key(dev, BTN_TOUCH, 1);
		input_report_key(dev, pen_data->coords.tool_type, 1);
	} else if (pen_data->coords.status == TS_RELEASE) {
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, pen_data->coords.tool_type, 0);
	}
	if (pen_data->coords.status) {
		input_report_abs(dev, ABS_X, pen_data->coords.x);
		input_report_abs(dev, ABS_Y, pen_data->coords.y);
		input_report_abs(dev, ABS_PRESSURE, pen_data->coords.p);
	}
	/* report pen button */
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		if (!pen_data->keys[i].status)
			continue;
		if (pen_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, pen_data->keys[i].code, 1);
		else if (pen_data->keys[i].status == TS_RELEASE)
			input_report_key(dev, pen_data->keys[i].code, 0);
	}
	input_sync(dev);
}

static void goodix_ts_report_finger(struct goodix_ts_core *cd,
		struct goodix_touch_data *touch_data, ktime_t kt)
{
	unsigned int touch_num = touch_data->touch_num;
	struct vts_device *vtsdev = cd->vtsdev;
	struct input_dev *dev = cd->input_dev;
	static u32 pre_fin;
	int i;

	/*first touch down and last touch up condition*/
	#if 0
	if (touch_num && !pre_fin)
		input_report_key(dev, BTN_TOUCH, 1);
	else if (!touch_num && pre_fin)
		input_report_key(dev, BTN_TOUCH, 0);
	#endif
	pre_fin = touch_num;
	//up first  
	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (!touch_data->coords[i].status)
			continue;
		if (touch_data->coords[i].status == TS_RELEASE) {
			//input_mt_slot(dev, i);
			//input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
			vts_report_point_up(vtsdev, i, touch_num, cd->fingerPressRecord[i][0],  cd->fingerPressRecord[i][1],
				cd->fingerPressRecord[i][2], cd->fingerPressRecord[i][3], false, kt);
			
		}

	}
	//down 
	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (!touch_data->coords[i].status || touch_data->coords[i].status == TS_RELEASE)
			continue;
		
		vts_report_point_down(vtsdev, i, touch_num, touch_data->coords[i].x,
		touch_data->coords[i].y, touch_data->coords[i].w, touch_data->coords[i].p, false, touch_data->custom_data_buf, sizeof(touch_data->custom_data_buf), kt);
		
		cd->fingerPressRecord[i][0] = touch_data->coords[i].x;
		cd->fingerPressRecord[i][1] = touch_data->coords[i].y;
		cd->fingerPressRecord[i][2] = touch_data->coords[i].w;
		cd->fingerPressRecord[i][3] = touch_data->coords[i].p;

	}
	vts_report_point_sync(vtsdev);

	input_sync(dev);
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data, ktime_t kt)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	int ret;

	core_data->irq_trig_cnt++;
	/* inform external module */
	mutex_lock(&goodix_modules_gt9897.mutex);
	list_for_each_entry_safe(ext_module, next,
				 &goodix_modules_gt9897.head, list) {
		if (!ext_module->funcs->irq_event)
			continue;
		ret = ext_module->funcs->irq_event(core_data, ext_module);
		if (ret == EVT_CANCEL_IRQEVT) {
			mutex_unlock(&goodix_modules_gt9897.mutex);
			return IRQ_HANDLED;
		}
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);

	/* read touch data from touch device */
	ret = hw_ops->event_handler(core_data, ts_event);
	if (likely(!ret)) {
		if (ts_event->event_type == EVENT_TOUCH) {
			/* report touch */
			goodix_ts_report_finger(core_data,
					&ts_event->touch_data, kt);
		}
		if (0) {//ts_event->event_type == EVENT_PEN   disable report pen
			goodix_ts_report_pen(core_data->pen_dev,
					&ts_event->pen_data);
		}
	}

	hw_ops->after_event_handler(core_data);
	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int ret;
	struct vts_device *vtsdev = core_data->vtsdev;

	/* if ts_bdata-> irq is invalid */
	core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	if (core_data->irq < 0) {
		ts_err("failed get irq num %d", core_data->irq);
		return -EINVAL;
	}

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);

	ret = vts_interrupt_register(vtsdev, core_data->irq, goodix_ts_threadirq_func, ts_bdata->irq_flags, core_data);

	/*
	ret = devm_request_threaded_irq(&core_data->pdev->dev,
				      core_data->irq, NULL,
				      goodix_ts_threadirq_func,
				      ts_bdata->irq_flags | IRQF_ONESHOT,
				      GOODIX_CORE_DRIVER_NAME,
				      core_data);
				      */
	if (ret < 0)
		ts_err("Failed to requeset threaded irq:%d", ret);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return ret;
}

/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct device *dev = core_data->bus->dev;//&core_data->pdev->dev;
	int ret = 0;

	ts_info("Power init");
	if (strlen(ts_bdata->avdd_name)) {
		core_data->avdd = devm_regulator_get(dev,
				 ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			ret = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", ret);
			core_data->avdd = NULL;
			return ret;
		} else {
			ret = regulator_set_load(core_data->avdd, 30000);
			if (ret < 0) {
				VTE("fail to set load for avdd");
			}
		}
	} else {
		ts_info("Avdd name is NULL");
	}

	if (strlen(ts_bdata->iovdd_name)) {
		core_data->iovdd = devm_regulator_get(dev,
				 ts_bdata->iovdd_name);
		if (IS_ERR_OR_NULL(core_data->iovdd)) {
			ret = PTR_ERR(core_data->iovdd);
			ts_err("Failed to get regulator iovdd:%d", ret);
			core_data->iovdd = NULL;
		} else {
			ret = regulator_set_load(core_data->iovdd, 30000);
			if (ret < 0) {
				VTE("fail to set load for iovdd");
			}
		}
	} else {
		ts_info("iovdd name is NULL");
	}


	return ret;
}

/**
 * goodix_ts_power_on - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int gt9897_ts_power_on(struct goodix_ts_core *cd)
{
	int ret = 0;

	ts_info("power on");
	if (cd->power_on)
		return 0;

	ret = cd->hw_ops->power_on(cd, true);
	if (!ret)
		cd->power_on = 1;
	else
		ts_err("failed power on, %d", ret);
	if (gpio_is_valid(cd->board_data.reset_gpio)) {
		gpio_set_value(cd->board_data.reset_gpio, 1);
	}
	return ret;
}

/**
 * goodix_ts_power_off - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int gt9897_ts_power_off(struct goodix_ts_core *cd)
{
	int ret;

	ts_info("Device power off");
	if (!cd->power_on)
		return 0;

	ret = cd->hw_ops->power_on(cd, false);
	if (!ret)
		cd->power_on = 0;
	else
		ts_err("failed power off, %d", ret);
	if (gpio_is_valid(cd->board_data.reset_gpio)) {
		gpio_set_value(cd->board_data.reset_gpio, 0);
	}
	return ret;
}

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
				  GPIOF_OUT_INIT_LOW, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
				  GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}

	return 0;
}

/**
 * goodix_ts_input_dev_config - Requset and config a input device
 *  then register it to input sybsystem.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
 #if 0
static int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int r;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	input_dev->name = GOODIX_CORE_DRIVER_NAME;
	input_dev->phys = GOOIDX_INPUT_PHYS;
	input_dev->id.product = 0xDEAD;
	input_dev->id.vendor = 0xBEEF;
	input_dev->id.version = 10427;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* set input parameters */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, ts_bdata->panel_max_w, 0, 0);
#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH,
			    INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		input_free_device(input_dev);
		return r;
	}

	return 0;
}
#endif
static int goodix_ts_pen_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *pen_dev = NULL;
	int r;

	pen_dev = input_allocate_device();
	if (!pen_dev) {
		ts_err("Failed to allocated pen device");
		return -ENOMEM;
	}
	core_data->pen_dev = pen_dev;
	input_set_drvdata(pen_dev, core_data);

	pen_dev->name = GOODIX_PEN_DRIVER_NAME;
	pen_dev->id.product = 0xDEAD;
	pen_dev->id.vendor = 0xBEEF;
	pen_dev->id.version = 10427;

	pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(ABS_X, pen_dev->absbit);
	__set_bit(ABS_Y, pen_dev->absbit);
	__set_bit(BTN_STYLUS, pen_dev->keybit);
	__set_bit(BTN_STYLUS2, pen_dev->keybit);
	__set_bit(BTN_TOUCH, pen_dev->keybit);
	__set_bit(BTN_TOOL_PEN, pen_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, 0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(pen_dev, ABS_PRESSURE, 0,
			     ts_bdata->panel_max_w, 0, 0);

	r = input_register_device(pen_dev);
	if (r < 0) {
		ts_err("Unable to register pen device");
		input_free_device(pen_dev);
		return r;
	}

	return 0;
}

static void goodix_ts_input_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->input_dev)
		return;
	input_unregister_device(core_data->input_dev);
	input_free_device(core_data->input_dev);
	core_data->input_dev = NULL;
}

static void goodix_ts_pen_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->pen_dev)
		return;
	input_unregister_device(core_data->pen_dev);
	input_free_device(core_data->pen_dev);
	core_data->pen_dev = NULL;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd = container_of(dwork,
			struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *cd = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = 0;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	if (!hw_ops->esd_check)
		return;

	ret = hw_ops->esd_check(cd);
	if (ret) {
		ts_err("esd check failed");
		gt9897_ts_power_off(cd);
		gt9897_ts_power_on(cd);
		if (hw_ops->reset)
			hw_ops->reset(cd, GOODIX_NORMAL_RESET_DELAY_MS);
		vts_abnormal_reset_collect(TOUCH_VCODE_UNEXPECTED_RESET_EVENT);
	}
	if (atomic_read(&ts_esd->esd_on))
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
static void goodix_ts_esd_on(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!misc->esd_addr)
		return;

	atomic_set(&ts_esd->esd_on, 1);
	if (!schedule_delayed_work(&ts_esd->esd_work, 2 * HZ)) {
		ts_info("esd work already in workqueue");
	}
	ts_info("esd on");
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
static void goodix_ts_esd_off(struct goodix_ts_core *cd)
{
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;
	int ret;

	atomic_set(&ts_esd->esd_on, 0);
	ret = cancel_delayed_work_sync(&ts_esd->esd_work);
	ts_info("Esd off, esd work state %d", ret);
}

/**
 * goodix_esd_notifier_callback - notification callback
 *  under certain condition, we need to turn off/on the esd
 *  protector, we use kernel notify call chain to achieve this.
 *
 *  for example: before firmware update we need to turn off the
 *  esd protector and after firmware update finished, we should
 *  turn on the esd protector.
 */
static int goodix_esd_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_esd *ts_esd = container_of(nb,
			struct goodix_ts_esd, esd_notifier);

	switch (action) {
	case NOTIFY_FWUPDATE_START:
	case NOTIFY_SUSPEND:
	case NOTIFY_ESD_OFF:
		goodix_ts_esd_off(ts_esd->ts_core);
		break;
	case NOTIFY_FWUPDATE_FAILED:
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_RESUME:
	case NOTIFY_ESD_ON:
		goodix_ts_esd_on(ts_esd->ts_core);
		break;
	default:
		break;
	}

	return 0;
}

/**
 * goodix_ts_esd_init - initialize esd protection
 */
static int goodix_ts_esd_init(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!cd->hw_ops->esd_check || !misc->esd_addr) {
		ts_info("missing key info for esd check");
		return 0;
	}

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	ts_esd->ts_core = cd;
	atomic_set(&ts_esd->esd_on, 0);
	ts_esd->esd_notifier.notifier_call = goodix_esd_notifier_callback;
	gt9897_ts_register_notifier(&ts_esd->esd_notifier);
	goodix_ts_esd_on(cd);

	return 0;
}

static void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
	struct input_dev *input_dev = core_data->input_dev;
	struct input_mt *mt = input_dev->mt;
	int i;
	ktime_t kt = ktime_get();
	memset(&core_data->ts_event, 0, sizeof(core_data->ts_event));
	goodix_ts_report_finger(core_data,
			&core_data->ts_event.touch_data, kt);

	if (mt) {
		for (i = 0; i < mt->num_slots; i++) {
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER,
					false);
		}
		input_report_key(input_dev, BTN_TOUCH, 0);
		//input_mt_sync_frame(input_dev);
		input_sync(input_dev);
	}
}

/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to  sleep
 */
static int goodix_ts_suspend(struct goodix_ts_core *core_data, int sleep_mode)
{
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;

	ts_info("Suspend start");

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	gt9897_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;

			if (sleep_mode) {
				if (strcmp(ext_module->name, "Goodix_gsx_gesture") == 0)
					continue;
			}

			ret = ext_module->funcs->before_suspend(core_data,
							      ext_module);
			if (ret == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_gt9897.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);

	/* disable irq */
	hw_ops->irq_enable(core_data, false);

	/* let touch ic work in sleep mode */
	if (hw_ops->suspend)
		hw_ops->suspend(core_data);
	atomic_set(&core_data->suspended, 1);
	atomic_set(&core_data->gestured, 0);

	/* inform exteranl modules */
	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (!ext_module->funcs->after_suspend)
				continue;

			ret = ext_module->funcs->after_suspend(core_data,
							     ext_module);
			if (ret == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_gt9897.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);

out:
	ts_info("Suspend end");
	return 0;
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
static int goodix_ts_resume(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;

	ts_info("Resume start");
	goodix_ts_release_connects(core_data);

	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (!ext_module->funcs->before_resume)
				continue;

			ret = ext_module->funcs->before_resume(core_data,
							     ext_module);
			if (ret == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_gt9897.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);

	atomic_set(&core_data->suspended, 0);
	atomic_set(&core_data->gestured, 0);
	/* resume device */
	if (hw_ops->resume)
		hw_ops->resume(core_data);

	mutex_lock(&goodix_modules_gt9897.mutex);
	if (!list_empty(&goodix_modules_gt9897.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules_gt9897.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			ret = ext_module->funcs->after_resume(core_data,
							    ext_module);
			if (ret == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_gt9897.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_gt9897.mutex);

	hw_ops->irq_enable(core_data, true);

	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	ts_info("try notify resume");
	gt9897_ts_blocking_notify(NOTIFY_RESUME, NULL);
out:
	ts_debug("Resume end");
	return 0;
}

#if 0//def CONFIG_PM
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_suspend(core_data,1);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_resume(core_data);
}
#endif
#endif

/**
 * goodix_generic_noti_callback - generic notifier callback
 *  for goodix touch notification event.
 */
static int goodix_generic_noti_callback(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct goodix_ts_core *cd = container_of(self,
			struct goodix_ts_core, ts_notifier);
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	ts_info("notify event type 0x%x", (unsigned int)action);
	switch (action) {
	case NOTIFY_FWUPDATE_START:
		hw_ops->irq_enable(cd, 0);
		break;
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_FWUPDATE_FAILED:
		if (hw_ops->read_version(cd, &cd->fw_version))
			ts_info("failed read fw version info[ignore]");
		hw_ops->irq_enable(cd, 1);
		break;
	default:
		break;
	}
	return 0;
}

static int goodix_ts_stage2_init(struct goodix_ts_core *cd)
{
	int ret;

	/* alloc/config/register input device */
	/*
	ret = goodix_ts_input_dev_config(cd);
	if (ret < 0) {
		ts_err("failed set input device");
		return ret;
	}
	*/
	if (cd->board_data.pen_enable) {
		ret = goodix_ts_pen_dev_config(cd);
		if (ret < 0) {
			ts_err("failed set pen device");
			goto exit;
		}
	}
	if(0){//after fw update 
		/* request irq line */
		ret = goodix_ts_irq_setup(cd);
		if (ret < 0) {
			ts_info("failed set irq");
			goto exit;
		}
		ts_info("success register irq");
	}
#ifdef CONFIG_FB
	if (fb_register_client(&cd->fb_notifier))
		ts_err("Failed to register fb notifier client:%d", ret);
#endif
	/*create sysfs files*/
	goodix_ts_sysfs_init(cd);

	/* esd protector */
	goodix_ts_esd_init(cd);
	goodix_debugfs_init(cd);
	return 0;
exit:
	goodix_ts_pen_dev_remove(cd);
	return ret;
}

/* try send the config specified with type */
static int goodix_send_ic_config(struct goodix_ts_core *cd, int type)
{
	u32 config_id;
	struct goodix_ic_config *cfg;

	if (type >= GOODIX_MAX_CONFIG_GROUP) {
		ts_err("unsupproted config type %d", type);
		return -EINVAL;
	}

	cfg = cd->ic_configs[type];
	if (!cfg || cfg->len <= 0) {
		ts_info("no valid normal config found");
		return -EINVAL;
	}

	config_id = goodix_get_file_config_id(cfg->data);
	if (cd->ic_info.version.config_id == config_id) {
		ts_info("config id is equal 0x%x, skiped", config_id);
		return 0;
	}

	ts_info("try send config, id=0x%x", config_id);
	return cd->hw_ops->send_config(cd, cfg->data, cfg->len);
}
#if 1
static int goodix_gt9897s_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)
{	
	int ret;
	u8 buf[2];
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	buf[0] = 0x01;
	VTI("enter goodix_later_init_thread");
	if(0){
		VTI("write 0xc900 0x01");
		cd->hw_ops->write(cd,0xc900,buf,1);
		mdelay(5);
		cd->hw_ops->write(cd,0xc900,buf,1);
	}

	/* setp 1: get config data from config bin */
	if (goodix_get_config_proc(cd))
		ts_info("no valid ic config found");
	else
		ts_info("success read ic config");
	
	cd->cfg_has_parsed = 1;
	goodix_cpy_fw(firmware);
	
	ret = gt9897_do_fw_update(cd->ic_configs[CONFIG_TYPE_NORMAL],
			UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST);
	if (ret)
		ts_err("failed do fw update");
	/* setp3: get fw version and ic_info
	 * at this step we believe that the ic is in normal mode,
	 * if the version info is invalid there must have some
	 * problem we cann't cover so exit init directly.
	 */
	ret = hw_ops->read_version(cd, &cd->fw_version);
	if (ret) {
		ts_err("invalid fw version, abort");
		goto uninit_fw;
	}
	ret = hw_ops->get_ic_info(cd, &cd->ic_info);
	if (ret) {
		ts_err("invalid ic info, abort");
		goto uninit_fw;
	}	


	/* the recomend way to update ic config is throuth ISP,
	 * if not we will send config with interactive mode
	 */
	goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);

	/* request irq line */
	ret = goodix_ts_irq_setup(cd);
	if (ret < 0) {
		ts_info("failed set irq");
		goto exit;
	}
	ts_info("success register irq");
  return ret;
exit:
	
uninit_fw:
	ts_err("stage2 init failed");
	cd->init_stage = CORE_INIT_FAIL;
	/*
	for (i = 0; i < GOODIX_MAX_CONFIG_GROUP; i++) {
		if (cd->ic_configs[i])
			kfree(cd->ic_configs[i]);
		cd->ic_configs[i] = NULL;
	}
	*/
	return ret;


}
#endif
#if 1
static int goodix_later_init_thread(void *data)
{
	int ret;
	struct goodix_ts_core *cd = data;


	/* setp 2: init fw struct add try do fw upgrade */
	ret = goodix_gt9897_fw_update_init(cd);
	if (ret) {
		ts_err("failed init fw update module");
		goto err_out;
	}

	/* init other resources */
	ret = goodix_ts_stage2_init(cd);
	if (ret) {
		ts_err("stage2 init failed");
		goto uninit_fw;
	}
	cd->init_stage = CORE_INIT_STAGE2;
	return 0;

uninit_fw:
	goodix_gt9897_fw_update_uninit();
err_out:
	ts_err("stage2 init failed");
	cd->init_stage = CORE_INIT_FAIL;
	/*
	for (i = 0; i < GOODIX_MAX_CONFIG_GROUP; i++) {
		if (cd->ic_configs[i])
			kfree(cd->ic_configs[i]);
		cd->ic_configs[i] = NULL;
	}
	*/
	return ret;
}
#endif

#if 0
static int goodix_later_init_thread(void *data)
{
	int ret, i;
	u8 buf[2];
	struct goodix_ts_core *cd = data;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	buf[0] = 0x01;
	VTI("enter goodix_later_init_thread");
	if(1){
	VTI("write 0xc900 0x01");
	cd->hw_ops->write(cd,0xc900,buf,1);
	mdelay(5);
	cd->hw_ops->write(cd,0xc900,buf,1);
	}
	/* setp 1: get config data from config bin */
	if (goodix_get_config_proc(cd))
		ts_info("no valid ic config found");
	else
		ts_info("success get valid ic config");

	/* setp 2: init fw struct add try do fw upgrade */
	ret = goodix_gt9897_fw_update_init(cd);
	if (ret) {
		ts_err("failed init fw update module");
		goto err_out;
	}

	ret = gt9897_do_fw_update(cd->ic_configs[CONFIG_TYPE_NORMAL],
			UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST);
	if (ret)
		ts_err("failed do fw update");
	/* setp3: get fw version and ic_info
	 * at this step we believe that the ic is in normal mode,
	 * if the version info is invalid there must have some
	 * problem we cann't cover so exit init directly.
	 */
	ret = hw_ops->read_version(cd, &cd->fw_version);
	if (ret) {
		ts_err("invalid fw version, abort");
		goto uninit_fw;
	}
	ret = hw_ops->get_ic_info(cd, &cd->ic_info);
	if (ret) {
		ts_err("invalid ic info, abort");
		goto uninit_fw;
	}

	/* the recomend way to update ic config is throuth ISP,
	 * if not we will send config with interactive mode
	 */
	goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);

	/* init other resources */
	ret = goodix_ts_stage2_init(cd);
	if (ret) {
		ts_err("stage2 init failed");
		goto uninit_fw;
	}
	cd->init_stage = CORE_INIT_STAGE2;
	return 0;

uninit_fw:
	goodix_gt9897_fw_update_uninit();
err_out:
	ts_err("stage2 init failed");
	cd->init_stage = CORE_INIT_FAIL;
	for (i = 0; i < GOODIX_MAX_CONFIG_GROUP; i++) {
		if (cd->ic_configs[i])
			kfree(cd->ic_configs[i]);
		cd->ic_configs[i] = NULL;
	}
	
	return ret;
}
#endif
#if 1
static int goodix_start_later_init(struct vts_device *vtsdev)
{
	struct goodix_ts_core *ts_core = vts_get_drvdata(vtsdev);
	if(ts_core)
		return goodix_later_init_thread(ts_core);	
	else
		return 0;

}
#endif
static int gt9897s_set_charging(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	if(NULL != cd && NULL !=hw_ops){
		ret = cd->hw_ops->set_charger(cd, state);
	}
	return ret;

}
static void bbk_gdix_brl_enter_gesture(struct goodix_ts_core *core_data)
{
	if (atomic_read(&core_data->suspended) == 1) {
		if (core_data->hw_ops->resume)
			core_data->hw_ops->resume(core_data);

		core_data->hw_ops->irq_enable(core_data, true);
	}

	goodix_ts_suspend(core_data, 0);
}

static int bbk_gdix_brl_mode_change(struct vts_device *vtsdev, int which)
{	int ret = 0;
	struct goodix_ts_core *core_data = vts_get_drvdata(vtsdev);
	static int last_mode = 0;
	VTI("mode change swtich = %d",which);
	switch (which) {
	case VTS_ST_NORMAL:
		gt9897_ts_power_on(core_data);
		ret = goodix_ts_resume(core_data);
		if (ret) {
			ts_err("Change normal mode fail, ret %d", ret);
			ret = -1;
		}
		break;
	case VTS_ST_SLEEP:
		ret = goodix_ts_suspend(core_data, 1);
		if (ret) {
			ts_err("Change sleep mode fail, ret %d", ret);
			ret = -1;
		}
		gt9897_ts_power_off(core_data);
		break;
	case VTS_ST_GESTURE:
		if (last_mode == VTS_ST_SLEEP) {
			gt9897_ts_power_on(core_data);
		}
		bbk_gdix_brl_enter_gesture(core_data);
		break;
	default:
		ts_err("Invalid mode change params %d", which);
		ret = -1;
		break;
	}
	last_mode = which;
	return ret;
}

static int bbk_goodix_long_press_enable(struct vts_device *vtsdev, int enable) 
{

	return setFingerGesture(enable);
}
static int bbk_goodix_set_finger_mode(struct vts_device *vtsdev, int mode) 
{
	int state = 3;
	if(1 == mode)
		state = 2;

	return setFingerGesture(state);
}
static int gt9897_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 15;
	return 0;
}

/*
 *data->type : 0-absolute inhibition
 *             1-long press inhibition
 *data->mode : 0-portrait
 *             1-landscape
 *data->block: 0-side
 *             1-Left & right corners under the portrait
 *             2-upper_corner of landscape
 *             3-bottom_corner of landscape
 */
static int gt9897_set_rejection_zone(struct vts_device *vtsdev, int scene)
{
	int ret = 0;
	struct goodix_ts_core *core_data = vts_get_drvdata(vtsdev);
	struct goodix_ts_cmd edge_abs_cmd;
	struct goodix_ts_cmd edge_longpress_cmd;
	struct goodix_ts_cmd edge_changeable_cmd;
	struct goodix_ts_cmd edge_landscape_side_cmd;
	struct vts_rejection_config *config;
	struct vts_rejection_data *data;
	int i;
	int mode =0;

	if (core_data == NULL) {
		VTE("core_data is null");
		return -EINVAL;
	}

	if (vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) {
		VTE("Touch is not VTS_ST_NORMAL");
		return -EIO;
	}

	config = vts_rejection_zone_config_get(vtsdev, scene);
	if (config == NULL)
		return -EINVAL;

	VTI("scene = %d, config_num = %d", config->scene, config->config_num);
	for (i = 0; i < config->config_num; i++) {
		data = config->data + i;
		if (data == NULL) {
			VTI("no config data to set rejection zone");
			continue;
		}

		if (data->type == 0xff) {
			VTI("no need to set rejection zone");
			continue;
		}

		mode = data->mode == 0 ? 0x01 : 0x02;
		if (data->type == 0x100) {//type 0x07
			memset(edge_changeable_cmd.data, 0, MAX_CMD_DATA_LEN);
			edge_changeable_cmd.cmd = 0x1F;
			edge_changeable_cmd.len = 7;	//2 + length(cmddata) + 2
			edge_changeable_cmd.data[0] = 0x07;
			edge_changeable_cmd.data[1] = mode;
			edge_changeable_cmd.data[2] = data->x1/2;
			if (core_data->hw_ops->send_cmd(core_data, &edge_changeable_cmd)) {
				VTE("failed send enter edge_changeable_cmd");
				ret = -1;
			}
		} else if (data->type == 0 && data->mode == 0) {//type 0x01
			edge_abs_cmd.cmd = 0x1F;
			edge_abs_cmd.len = 12;	//2 + length(cmddata) + 2
			edge_abs_cmd.data[0] = 0x01;
			edge_abs_cmd.data[1] = mode;
			if (data->block == 0) {
				edge_abs_cmd.data[2] = data->x1/2;
				edge_abs_cmd.data[3] = data->x2/2;
			} else if (data->block == 1) {
				edge_abs_cmd.data[4] = data->x1/2;
				edge_abs_cmd.data[5] = data->y1/2;
				edge_abs_cmd.data[6] = data->x2/2;
				edge_abs_cmd.data[7] = data->y2/2;
			} else {
				VTE("type:0x%x config data error!", mode);
				return -EINVAL;
			}
		} else if (data->type == 1 && data->mode == 0) {//type 0x02
			edge_longpress_cmd.cmd = 0x1F;
			edge_longpress_cmd.len = 12;	//2 + length(cmddata) + 2
			edge_longpress_cmd.data[0] = 0x02;
			edge_longpress_cmd.data[1] = mode;
			if (data->block == 0) {
				edge_longpress_cmd.data[2] = data->x1/2;
				edge_longpress_cmd.data[3] = data->x2/2;
			} else if (data->block == 1) {
				edge_longpress_cmd.data[4] = data->x1/2;
				edge_longpress_cmd.data[5] = data->y1/2;
				edge_longpress_cmd.data[6] = data->x2/2;
				edge_longpress_cmd.data[7] = data->y2/2;
			} else {
				VTE("type:0x%x config data error!", mode);
				return -EINVAL;
			}
		} else if (data->type == 0 && data->mode == 1) {//type 0x04
			edge_abs_cmd.cmd = 0x1F;
			edge_abs_cmd.len = 14;	//2 + length(cmddata) + 2
			edge_abs_cmd.data[0] = 0x04;
			edge_abs_cmd.data[1] = mode;
			if (data->block == 2) {
				edge_abs_cmd.data[2] = data->x1/2;
				edge_abs_cmd.data[3] = data->y1/2;
				edge_abs_cmd.data[4] = data->x2/2;
				edge_abs_cmd.data[5] = data->y2/2;
			} else if (data->block == 3) {
				edge_abs_cmd.data[6] = data->x1/2;
				edge_abs_cmd.data[7] = data->y1/2;
				edge_abs_cmd.data[8] = data->x2/2;
				edge_abs_cmd.data[9] = data->y2/2;
			}
		} else if (data->type == 1 && data->mode == 1) {//type 0x06
			edge_longpress_cmd.cmd = 0x1F;
			edge_longpress_cmd.len = 14;	//2 + length(cmddata) + 2
			edge_longpress_cmd.data[0] = 0x06;
			edge_longpress_cmd.data[1] = mode;
			if (data->block == 2) {
				edge_longpress_cmd.data[2] = data->x1/2;
				edge_longpress_cmd.data[3] = data->y1/2;
				edge_longpress_cmd.data[4] = data->x2/2;
				edge_longpress_cmd.data[5] = data->y2/2;
			} else if (data->block == 3) {
				edge_longpress_cmd.data[6] = data->x1/2;
				edge_longpress_cmd.data[7] = data->y1/2;
				edge_longpress_cmd.data[8] = data->x2/2;
				edge_longpress_cmd.data[9] = data->y2/2;
			}
		} else {
			VTE("unknow config!");
			return -EINVAL;
		}

		if (data->mode == 1 && data->block == 0) {//type 0x03 & type 0x05
			edge_landscape_side_cmd.cmd = 0x1F;
			edge_landscape_side_cmd.len = 10;
			memset(edge_landscape_side_cmd.data, 0, MAX_CMD_DATA_LEN);
			edge_landscape_side_cmd.data[0] = data->type == 0 ? 0x03 : 0x05;
			edge_landscape_side_cmd.data[1] = mode;
			edge_landscape_side_cmd.data[2] = data->x1/2;
			edge_landscape_side_cmd.data[3] = data->y1/2;
			edge_landscape_side_cmd.data[4] = data->x2/2;
			edge_landscape_side_cmd.data[5] = data->y2/2;
			if (core_data->hw_ops->send_cmd(core_data, &edge_landscape_side_cmd)) {
				VTE("failed send enter edge_landscape_side_cmd");
				ret = -1;
			}
		}
	}

	if (scene == 6 || scene == 7)
		return ret;

	if (core_data->hw_ops->send_cmd(core_data, &edge_abs_cmd)) {
		VTE("failed send enter edge_abs_cmd");
		ret = -1;
	}

	if (core_data->hw_ops->send_cmd(core_data, &edge_longpress_cmd)) {
		VTE("failed send enter edge_longpress_cmd");
		ret = -1;
	}

	return ret;
}

#define GOODIX_SET_CARD_REGION 0x20
static int goodix_set_card_region (struct vts_device *vtsdev, int enable)
{
	unsigned int y0 = vtsdev->y0;
	unsigned int y1 = vtsdev->y1;
	unsigned int width = vtsdev->width;
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
	int ret = 0;

	if (enable == 0) {
		return 0;
	}

	if (!y0 || !y1) {
		VTE("card region cfg invalid, y0 = %d y1 = %d width = %d", y0, y1, width);
		return -1;
	}

	cmd.cmd = GOODIX_SET_CARD_REGION;
	cmd.len = 0x0B;
	cmd.data[0] = 0xFF;
	cmd.data[1] = width & 0xff;
	cmd.data[2] = width >> 8;
	cmd.data[3] = y0 & 0xff;
	cmd.data[4] = y0 >> 8;
	cmd.data[5] = y1 & 0xff;
	cmd.data[6] = y1 >> 8;
	ret = core_data->hw_ops->send_cmd(core_data, &cmd);
	if (ret)
		ts_err("failed send keep active cmd");

	return ret;
}

static const int gesture_bit[] = {
	VTS_GESTURE_C,
	VTS_GESTURE_E,
	VTS_GESTURE_F,
	VTS_GESTURE_M,
	VTS_GESTURE_O,
	VTS_GESTURE_W,
	VTS_GESTURE_A,
	VTS_GESTURE_DCLICK,
	VTS_GESTURE_UP,
	VTS_GESTURE_DOWN,
	VTS_GESTURE_LR,
	VTS_GESTURE_LR,
	0,
	0,
	0,
	0
};
#define GOODIX_SET_GESTURE_BIT 0x12
static int goodix_ts_set_gesture(struct vts_device *vtsdev, int enable)
{
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
	int ret = 0;
	int data = 0xffff;   //0-ON   1-OFF
	int i = 0;

	VTI("%s: set state = %d", __func__, enable);

	for (i = 0; i < ARRAY_SIZE(gesture_bit); i++) {
		if (enable & gesture_bit[i])
			data &= (~(1 << i));
	}

	cmd.cmd = GOODIX_SET_GESTURE_BIT;
	cmd.len = 6;
	cmd.data[0] = data & 0xff;
	cmd.data[1] = data >> 8;
	ret = core_data->hw_ops->send_cmd(core_data, &cmd);
	if (ret)
		ts_err("failed send gesture bit cmd");

	return ret;
	
}

static int gt9897_get_ic_mode(struct vts_device *vtsdev)
{
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	unsigned char mode_reg_9897;
	hw_ops->read(cd, 0x10168, &mode_reg_9897, 1);
	if(mode_reg_9897 & 0x08) {
		ts_info("now is Gesture mode");
		return 0;
	} else {
		ts_info("now is normal mode");
		return 1;
	}
}

#define GOODIX_SET_CLOCK_AREA_CMD 0x16
static int gt9897_set_screen_clock_area(struct vts_device *vtsdev,int report_enable)
{
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct vts_screen_clock_cmd sclock_cmd;
	struct goodix_ts_cmd set_clock_area;
	int x_start=0;
	int y_start=0;
	int width=0;
	int height=0;
	u32 display_x = 1;
	u32 display_y = 1;
	u32 dimention_x = 1;
	u32 dimention_y = 1;
	u32 resolution = 0;

	if(vts_get_run_mode(vtsdev) == VTS_ST_SLEEP) {
		VTE("get run mode is sleep");
		return -EINVAL;
	}

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);
	}

	vts_get_screen_clock_zone(&sclock_cmd, &vtsdev->screen_clock_zone);
	x_start = sclock_cmd.x;
	y_start = sclock_cmd.y;
	width= sclock_cmd.width;
	height= sclock_cmd.height;

	set_clock_area.cmd = GOODIX_SET_CLOCK_AREA_CMD;
	set_clock_area.len = 13;
	set_clock_area.data[0] = 1;
	set_clock_area.data[1] = x_start & 0xFF;
	set_clock_area.data[2] = x_start >> 8;
	set_clock_area.data[3] = y_start & 0xFF;
	set_clock_area.data[4] = y_start >> 8;
	set_clock_area.data[5] = width & 0xFF;
	set_clock_area.data[6] = width >> 8;
	set_clock_area.data[7] = height & 0xFF;
	set_clock_area.data[8] = height >> 8;
	VTI("write :buffer[] = 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",
		set_clock_area.data[0],set_clock_area.data[1],set_clock_area.data[2],
		set_clock_area.data[3],set_clock_area.data[4],set_clock_area.data[5],
		set_clock_area.data[6],set_clock_area.data[7],set_clock_area.data[8]);
	if (cd->hw_ops->send_cmd(cd, &set_clock_area))
		VTE("failed send set_clock_area cmd");

	return 0;
}

/****       add for new game_mode begin       ****/
static int bbk_gt9897_set_game_mode(struct vts_device *vtsdev, int state)
{
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_cmd game_cmd;
	u8 game_in = 0;
	
	if (state == PROC_IN_GAME) {
		game_in = 0x1A;
		cd->game_mode = 1;
	}
	if (state == PROC_OUT_GAME) {
		game_in = 0x1B;
		cd->game_mode = 0;
	}

	game_cmd.cmd = game_in;
	game_cmd.len = 4;
	if (cd->hw_ops->send_cmd(cd, &game_cmd))
		VTE("failed send game in cmd");
	return 0;
}
#define GOODIX_SET_REPORT_RATE_CMD 0x22
static int bbk_gt9897_set_high_report_rate(struct vts_device *vtsdev, int rate)
{
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_cmd rate_cmd;
	u8 report_rate = 0;

	if (rate == RATE_NORMAL) {
		report_rate = 0;
		cd->high_report_rate = 0;
	}
	if (rate == RATE_HIGH) {
		report_rate = 1;
		cd->high_report_rate = 1;
	}

	rate_cmd.cmd = GOODIX_SET_REPORT_RATE_CMD;
	rate_cmd.len = 5;
	rate_cmd.data[0] = report_rate;
	if (cd->hw_ops->send_cmd(cd, &rate_cmd))
		VTE("failed send report rate cmd");
	return 0;
}
#define GOODIX_SET_IDLE_TIME_CMD 0x21
static int bbk_gt9897_set_idle_time(struct vts_device *vtsdev, int times)
{
	struct goodix_ts_core *cd = vts_get_drvdata(vtsdev);
	struct goodix_ts_cmd rate_cmd;
	u8 idle_time = 0;

	if (times == IDLE_TIME_1S) {
		idle_time = 1;
	}
	if (times == IDLE_TIME_2S) {
		idle_time = 2;
	}
	if (times == IDLE_TIME_10S) {
		idle_time = 10;
	}

	rate_cmd.cmd = GOODIX_SET_IDLE_TIME_CMD;
	rate_cmd.len = 6;
	rate_cmd.data[0] = idle_time;
	if (cd->game_mode && cd->high_report_rate && idle_time > 8)
		rate_cmd.data[1] = idle_time - 8;     //8s--1KHz idle  data[1]--active
	else
		rate_cmd.data[1] = 0;
	if (cd->hw_ops->send_cmd(cd, &rate_cmd))
		VTE("failed send idle time cmd");
	return 0;
}
/****       add for new game_mode end       ****/


static const struct vts_operations gt9897s_ops = {
	.init = goodix_start_later_init,
	.update_firmware = goodix_gt9897s_fw_update,
	.set_charging = gt9897s_set_charging,
	.change_mode = bbk_gdix_brl_mode_change,
	.get_fw_version = bbk_gdix_brl_get_fw_version,
	.get_fw_resolution = bbk_gdix_brl_get_fw_resolution,
	.rom_read = bbk_gdix_brl_readUdd,
	.rom_write = bbk_gdix_brl_writeUdd,
	.set_auto_idle = bbk_gdix_brl_idleEnableOrDisable,
	.get_frame = bbk_gdix_get_rawordiff_data,
	.set_rotation = bbk_gdix_setEdgeRestainSwitch,
	.set_long_press = bbk_goodix_long_press_enable,
	.set_finger_mode = bbk_goodix_set_finger_mode,
	.rom_size = gt9897_get_flash_size,
	.set_rejection_zone = gt9897_set_rejection_zone,
	.set_card_region = goodix_set_card_region,
	.set_gesture = goodix_ts_set_gesture,
	.get_ic_mode = gt9897_get_ic_mode,
	.set_screen_clock_area = gt9897_set_screen_clock_area,
	.set_game_mode = bbk_gt9897_set_game_mode,
	.set_high_report_rate = bbk_gt9897_set_high_report_rate,
	.set_idle_time = bbk_gt9897_set_idle_time,
	/*
	.set_charging = bbk_goodix_set_charger_bit_V2,
	.set_rotation = bbk_goodix_set_Edge_Switch_V2,
	.update_firmware = goodix_cfg_bin_proc_V2,
	.change_mode = bbk_goodix_mode_change_V2,
	.get_fw_version = bbk_goodix_get_fw_version_V2,
	.rom_size = bbk_goodix_get_flash_size,

	.set_auto_idle = bbk_goodix_set_auto_idle_V2,
	.get_frame = bbk_goodix_get_rawordiff_data_V2,
	.set_virtual_prox = goodix_faceDetect_startEnd,
	.set_long_press = bbk_goodix_long_press_enable,
	.set_input_method = bbk_goodix_set_input_method,
	.get_tp_channel_comp_data = bbk_goodix_get_channel_comp,
	.set_bus_state = bbk_goodix_set_doze_state,
	.set_screen_clock_area = bbk_set_goodix_clock_area,
	*/
};

/**
 * goodix_ts_probe - called by kernel when Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = NULL;
	struct goodix_bus_interface *bus_interface;
	struct vts_device *vtsdev = NULL;
	int ret;

	ts_info("goodix_ts_probe IN");

	bus_interface = pdev->dev.platform_data;
	if (!bus_interface) {
		ts_err("Invalid touch device");
		core_module_prob_sate = CORE_MODULE_PROB_FAILED;
		return -ENODEV;
	}
	
	vtsdev = vts_device_alloc();
	if (!vtsdev) {
		VTE("Alloc failed");
		return -ENOMEM;
	}

	core_data = devm_kzalloc(&pdev->dev,
			sizeof(struct goodix_ts_core), GFP_KERNEL);
	if (!core_data) {
		ts_err("Failed to allocate memory for core data");
		core_module_prob_sate = CORE_MODULE_PROB_FAILED;
		return -ENOMEM;
	}
	core_data->vtsdev = vtsdev;
	vtsdev->ops = &gt9897s_ops;
	bus_interface->vtsdev = vtsdev;
	if (IS_ENABLED(CONFIG_OF) && bus_interface->dev->of_node) {
		/* parse devicetree property */
		ret = goodix_parse_dt(bus_interface->dev->of_node,
					&core_data->board_data);
		if (ret) {
			ts_err("failed parse device info form dts, %d", ret);
			return -EINVAL;
		}
	} else {
		ts_err("no valid device tree node found");
		return -ENODEV;
	}

	core_data->hw_ops = goodix_get_hw_ops();
	if (!core_data->hw_ops) {
		ts_err("hw ops is NULL");
		core_module_prob_sate = CORE_MODULE_PROB_FAILED;
		return -EINVAL;
	}
	vts_parse_dt_property(vtsdev, bus_interface->dev->of_node);
	vts_set_drvdata(vtsdev, core_data);
	
	goodix_core_module_init();
	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->bus = bus_interface;
	core_data->cfg_has_parsed = 0;
	platform_set_drvdata(pdev, core_data);
	/* get GPIO resource */
	ret = goodix_ts_gpio_setup(core_data);
	if (ret) {
		ts_err("failed init gpio");
		goto err_out;
	}

	ret = goodix_ts_power_init(core_data);
	if (ret) {
		ts_err("failed init power");
		goto err_out;
	}

	ret = gt9897_ts_power_on(core_data);
	if (ret) {
		ts_err("failed power on");
		goto err_out;
	}
	gpio_direction_output(core_data->board_data.reset_gpio, 1);
	mdelay(10);

	core_data->hw_ops->reset(core_data, GOODIX_NORMAL_RESET_DELAY_MS);
	/* confirm it's goodix touch dev or not */
	ret = core_data->hw_ops->dev_confirm(core_data);
	if (ret) {
		ts_err("goodix device confirm failed");
		goto err_out;
	}

	/* generic notifier callback */
	core_data->ts_notifier.notifier_call = goodix_generic_noti_callback;
	gt9897_ts_register_notifier(&core_data->ts_notifier);
	
	ret = vts_register_driver(vtsdev);
	if (ret) {
		VTE("register vts driver failed!, ret = %d\n", ret);
		}
	core_data->input_dev = vtsdev->idev;

	core_data->init_stage = CORE_INIT_STAGE1;
	goodix_modules_gt9897.core_data = core_data;
	core_module_prob_sate = CORE_MODULE_PROB_SUCCESS;

	complete_all(&goodix_modules_gt9897.core_comp);
	ts_info("goodix_ts_core probe success");
	return 0;

err_out:
	core_data->init_stage = CORE_INIT_FAIL;
	core_module_prob_sate = CORE_MODULE_PROB_FAILED;
	ts_err("goodix_ts_core failed, ret:%d", ret);
	/* wakeup ext module register work */
	complete_all(&goodix_modules_gt9897.core_comp);
	return ret;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = platform_get_drvdata(pdev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;

	if (goodix_gt9897_unregister_all_module())
		return -EBUSY;

	gt9897_ts_unregister_notifier(&core_data->ts_notifier);

	if (core_data->init_stage >= CORE_INIT_STAGE2) {
		hw_ops->irq_enable(core_data, false);
	#ifdef CONFIG_FB
		fb_unregister_client(&core_data->fb_notifier);
	#endif
		core_module_prob_sate = CORE_MODULE_REMOVED;
		if (atomic_read(&core_data->ts_esd.esd_on))
			goodix_ts_esd_off(core_data);
		gt9897_ts_unregister_notifier(&ts_esd->esd_notifier);

		goodix_debugfs_remove(core_data);
		goodix_gt9897_fw_update_uninit();
		goodix_ts_input_dev_remove(core_data);
		goodix_ts_pen_dev_remove(core_data);
		goodix_ts_sysfs_exit(core_data);
		gt9897_ts_power_off(core_data);
	}

	return 0;
}

#if 0//def CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
#endif
};
#endif

static const struct platform_device_id gt9897_ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, gt9897_ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#if 0//def CONFIG_PM
		.pm = &dev_pm_ops,
#endif
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = gt9897_ts_core_ids,
};

int gt9897_ts_core_init(void)
{
	ts_info("Core layer init:%s", GOODIX_DRIVER_VERSION);
	//ret = goodix_bus_init_gt9897();
	//if (ret) {
	//	ts_err("failed add bus driver");
		//return ret;
	//}
	return platform_driver_register(&goodix_ts_driver);
}
void gt9897_ts_core_exit(void)
{
	VTI("Core layer exit");
	platform_driver_unregister(&goodix_ts_driver);
	return;
}


