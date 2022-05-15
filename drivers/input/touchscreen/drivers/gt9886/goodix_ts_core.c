 /*
  * Goodix Touchscreen Driver
  * Core layer of touchdriver architecture.
  *
  * Copyright (C) 2015 - 2016 Goodix, Inc.
  * Authors:  Yulong Cai <caiyulong@goodix.com>
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/completion.h>
#include <linux/of_irq.h>
#include <linux/version.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include "../vts_core.h"
#include "goodix_ts_core.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif


#define GOOIDX_INPUT_PHYS		"goodix_ts/input0"
#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"

extern int goodix_cfg_bin_proc_V2(struct vts_device *vtsdev, const struct firmware *firmware);
//extern int goodix_cfg_bin_proc_V2(void *data);

extern int goodix_start_cfg_bin(struct goodix_ts_core *ts_core);
extern struct goodix_ext_module goodix_fwu_module_V2;

struct goodix_module goodix_modules_V2;

/**
 * __do_register_ext_module - register external module
 * to register into touch core modules structure
 */
static void  __do_register_ext_module(struct work_struct *work)
{
	struct goodix_ext_module *module =
			container_of(work, struct goodix_ext_module, work);
	struct goodix_ext_module *ext_module;
	struct list_head *insert_point = &goodix_modules_V2.head;
	int ret = -1;

	VTI("__do_register_ext_module IN, goodix_modules_V2.core_exit:%d", goodix_modules_V2.core_exit);

	/* waitting for core layer 
	if (!wait_for_completion_timeout(&goodix_modules_V2.core_comp, 100 * HZ)) {
		VTE("Module [%s] timeout", module->name);
		return;
	}
	*/

	do {
		
		ret = wait_for_completion_interruptible_timeout(&goodix_modules_V2.core_comp,  200 * HZ);
		VTI("the value of wait_for_completion_interruptible_timeout is %d", ret);
		if (ret == 0){
			VTE("Module [%s] timeout", module->name);
			return;
		}
	}while (ret == -ERESTARTSYS);


	/* driver probe failed */
	if (goodix_modules_V2.core_exit) {
		VTE("Can't register ext_module, core exit");
		return;
	}

	VTI("start register ext_module");

	/* prority level *must* be set */
	if (module->priority == EXTMOD_PRIO_RESERVED) {
		VTE("Priority of module [%s] needs to be set",
				module->name);
		return;
	}

	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (ext_module == module) {
				VTI("Module [%s] already exists",
						module->name);
				mutex_unlock(&goodix_modules_V2.mutex);
				return;
			}
		}

		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			/* small value of priority have
			 * higher priority level*/
			if (ext_module->priority >= module->priority) {
				insert_point = &ext_module->list;
				break;
			}
		} /* else module will be inserted
		 to goodix_modules_V2->head */
	}

	if (module->funcs && module->funcs->init) {
		if (module->funcs->init(goodix_modules_V2.core_data,
					module) < 0) {
			VTE("Module [%s] init error",
					module->name ? module->name : " ");
			mutex_unlock(&goodix_modules_V2.mutex);
			return;
		}
	}

	list_add(&module->list, insert_point->prev);
	goodix_modules_V2.count++;
	mutex_unlock(&goodix_modules_V2.mutex);

	VTI("Module [%s] registered,priority:%u",
			module->name,
			module->priority);
	return;
}

/**
 * goodix_register_ext_module_V2 - interface for external module
 * to register into touch core modules structure
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int goodix_register_ext_module_V2(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;

	if (!goodix_modules_V2.initilized) {
		goodix_modules_V2.initilized = true;
		goodix_modules_V2.core_exit = true;
		INIT_LIST_HEAD(&goodix_modules_V2.head);
		mutex_init(&goodix_modules_V2.mutex);
		init_completion(&goodix_modules_V2.core_comp);
	}

/*	if (goodix_modules_V2.core_exit) {
		VTE("Can't register ext_module, core exit");
		return -EFAULT;
	}
*/
	//msleep(500);
	VTI("goodix_register_ext_module_V2 IN");

	INIT_WORK(&module->work, __do_register_ext_module);
	schedule_work(&module->work);

	VTI("goodix_register_ext_module_V2 OUT");


	return 0;
}
EXPORT_SYMBOL_GPL(goodix_register_ext_module_V2);

/**
 * goodix_unregister_ext_module_V2 - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int goodix_unregister_ext_module_V2(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module;
	bool found = false;

	if (!module)
		return -EINVAL;

	if (!goodix_modules_V2.initilized)
		return -EINVAL;

	if (!goodix_modules_V2.core_data)
		return -ENODEV;

	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (ext_module == module) {
				found = true;
				break;
			}
		}
	} else {
		mutex_unlock(&goodix_modules_V2.mutex);
		return -EFAULT;
	}

	if (!found) {
		VTE("Module [%s] never registed",
				module->name);
		mutex_unlock(&goodix_modules_V2.mutex);
		return -EFAULT;
	}

	list_del(&module->list);
	mutex_unlock(&goodix_modules_V2.mutex);

	if (module->funcs && module->funcs->exit)
		module->funcs->exit(goodix_modules_V2.core_data, module);
	goodix_modules_V2.count--;

	VTI("Moudle [%s] unregistered",
			module->name ? module->name : " ");
	return 0;
}
EXPORT_SYMBOL_GPL(goodix_unregister_ext_module_V2);

static void goodix_ext_sysfs_release(struct kobject *kobj)
{
	VTI("Kobject released!");
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

struct kobj_type *goodix_get_default_ktype_V2(void)
{
	return &goodix_ext_ktype;
}
EXPORT_SYMBOL_GPL(goodix_get_default_ktype_V2);

struct kobject *goodix_get_default_kobj_V2(void)
{
	struct kobject *kobj = NULL;

	if (goodix_modules_V2.core_data &&
			goodix_modules_V2.core_data->pdev)
		kobj = &goodix_modules_V2.core_data->pdev->dev.kobj;
	return kobj;
}
EXPORT_SYMBOL_GPL(goodix_get_default_kobj_V2);

/* show external module infomation */
static ssize_t goodix_ts_extmod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ext_module *module;
	size_t offset = 0;
	int r;

	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(module, &goodix_modules_V2.head, list) {
			r = snprintf(&buf[offset], PAGE_SIZE,
					"priority:%u module:%s\n",
					module->priority, module->name);
			if (r < 0) {
				mutex_unlock(&goodix_modules_V2.mutex);
				return -EINVAL;
			}
			offset += r;
		}
	}

	mutex_unlock(&goodix_modules_V2.mutex);
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
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_version chip_ver;
	int r, cnt = 0;

	cnt += snprintf(buf, PAGE_SIZE,
			"TouchDeviceName:%s\n", ts_dev->name);
	if (ts_dev->hw_ops->read_version) {
		r = ts_dev->hw_ops->read_version(ts_dev, &chip_ver);
		if (!r && chip_ver.valid) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
					"PID:%s\nVID:%02x %02x %02x %02x\nSensorID:%02x\n",
					chip_ver.pid, chip_ver.vid[0],
					chip_ver.vid[1], chip_ver.vid[2],
					chip_ver.vid[3], chip_ver.sensor_id);
		}
	}

	return cnt;
}

/* show chip configuration data */
static ssize_t goodix_ts_config_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_config *ncfg = ts_dev->normal_cfg;
	u8 *data;
	int i, r, offset = 0;

	if (ncfg && ncfg->initialized && ncfg->length < PAGE_SIZE) {
		data = kmalloc(ncfg->length, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		r = ts_dev->hw_ops->read(ts_dev, ncfg->reg_base,
				&data[0], ncfg->length);
		if (r < 0) {
			kfree(data);
			return -EINVAL;
		}

		for (i = 0; i < ncfg->length; i++) {
			if (i != 0 && i % 20 == 0)
				buf[offset++] = '\n';
			offset += snprintf(&buf[offset], PAGE_SIZE - offset,
					"%02x ", data[i]);
		}
		buf[offset++] = '\n';
		buf[offset++] = '\0';
		kfree(data);
		return offset;
	}

	return -EINVAL;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	if (ts_dev->hw_ops->reset)
		ts_dev->hw_ops->reset(ts_dev);
	return count;

}

static ssize_t goodix_ts_read_cfg_show(struct device *dev,
				struct device_attribute *attr,
						char *buf)
{
	struct goodix_ts_core *core_data =
				dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int ret, i, offset;
	char *cfg_buf;

	cfg_buf = kzalloc(4096, GFP_KERNEL);
	disable_irq(core_data->irq);
	if (ts_dev->hw_ops->read_config)
		ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf, 0);
	else
		ret = -EINVAL;
	enable_irq(core_data->irq);

	offset = 0;
	if (ret > 0) {
		for (i = 0; i < ret; i++) {
			if (i != 0 && i % 20 == 0)
				buf[offset++] = '\n';
			offset += snprintf(&buf[offset], 4096 - offset,
					"%02x ", cfg_buf[i]);
		}

	}
	kfree(cfg_buf);
	return ret;
}

static int goodix_ts_convert_0x_data(const u8 *buf,
				int buf_size,
				unsigned char *out_buf,
				int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}
	VTI("***m_size:%d", m_size);

	if (m_size <= 1) {
		VTE("cfg file ERROR, valid data count:%d\n", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X') {
			if (temp_index >= m_size) {
				VTE("exchange cfg data error, overflow, temp_index:%d,m_size:%d\n",
						temp_index, m_size);
				return -EINVAL;
			}
			if (buf[i + 1] >= '0' && buf[i + 1] <= '9')
				out_buf[temp_index] = (buf[i + 1] - '0') << 4;
			else if (buf[i + 1] >= 'a' && buf[i + 1] <= 'f')
				out_buf[temp_index] = (buf[i + 1] - 'a' + 10) << 4;
			else if (buf[i + 1] >= 'A' && buf[i + 1] <= 'F')
				out_buf[temp_index] = (buf[i + 1] - 'A' + 10) << 4;

			if (buf[i + 2] >= '0' && buf[i + 2] <= '9')
				out_buf[temp_index] += (buf[i + 2] - '0');
			else if (buf[i + 2] >= 'a' && buf[i + 2] <= 'f')
				out_buf[temp_index] += (buf[i + 2] - 'a' + 10);
			else if (buf[i + 2] >= 'A' && buf[i + 2] <= 'F')
				out_buf[temp_index] += (buf[i + 2] - 'A' + 10);

			temp_index++;
		}
	}
	return 0;
}



static ssize_t goodix_ts_send_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct goodix_ts_core *core_data =
				dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en, r;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;

	VTI("******IN");

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	VTI("en:%d", en);

	disable_irq(core_data->irq);

	/*request configuration*/
	r = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	if (r < 0) {
		VTE("cfg file [%s] not available,errno:%d", GOODIX_DEFAULT_CFG_NAME, r);
		goto exit;
	} else
		VTI("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);

	config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (config == NULL) {
		VTE("Memory allco err");
		goto exit;
	}
	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		VTE("convert config data FAILED");
		goto exit;
	}

	config->reg_base = ts_dev->reg.cfg_addr;
	config->initialized = true;

	if (ts_dev->hw_ops->send_config)
		ts_dev->hw_ops->send_config(ts_dev, config);

exit:
	enable_irq(core_data->irq);
	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}
	
	VTI("******OUT");
	return count;
}

/* show irq infomation */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n",
			core_data->irq);
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
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable_V2(core_data, en);
	return count;
}

static ssize_t goodix_ts_get_rawdiff_data_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int opt, i;
	int sen_num, drv_num;
	int data_size;
	int *temp_data = NULL;

	if (sscanf(buf, "%d", &opt) != 1)
		return -EINVAL;


	bbk_goodix_get_channel_num_V2(&sen_num, &drv_num);
	data_size = sen_num * drv_num;
	temp_data = kzalloc(data_size * 4, GFP_KERNEL);
	if (bbk_goodix_get_rawordiff_data_V2_v1(opt, temp_data)) {
		kfree(temp_data);
		temp_data = NULL;
		return -EINVAL;
	}

	for (i = 0; i < data_size; i++)
		VTI("%d", temp_data[i]);

	kfree(temp_data);
	temp_data = NULL;
	return count;
}

static ssize_t goodix_ts_fw_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int opt, ret = count;
	const struct firmware *firmware = NULL;

	if (sscanf(buf, "%d", &opt) != 1)
		return -EINVAL;

	if (1 == opt) {
		ret = request_firmware(&firmware, "goodix_firmware.bin", dev);
		if(ret < 0){
			VTE("request_firmware fail");
			ret = -EINVAL;
			goto exit;
		}
		if (bbk_goodix_fw_update_V2(NULL, firmware) < 0) {
			ret = -EINVAL;
			goto exit;
		}
	}
exit:
	if(firmware){
		release_firmware(firmware);
		firmware = NULL;
	}
	return ret;
}
static ssize_t goodix_ts_enable_idle_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int state;

	if (sscanf(buf, "%d", &state) != 1)
		return -EINVAL;

	if (bbk_goodix_set_auto_idle_V2(NULL, state) == -1)
		return -EINVAL;

	return count;
}

static ssize_t goodix_ts_flash_udd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, offset = 0;

	u8 temp_data[15] = {0};

	if (bbk_goodix_readUdd_V2_v1(temp_data))
		return -EINVAL;

	for (i = 0; i < 15; i++) {
		offset += snprintf(&buf[offset], PAGE_SIZE - offset,
				 "%02x ", temp_data[i]);
	}

	buf[offset++] = '\n';
	buf[offset++] = '\0';

	return offset;
}

static ssize_t goodix_ts_flash_udd_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int opt = 0, ret = 0;
	unsigned char temp_buf[15] = {8, 6, 6, 3, 6, 1,
					0, 3, 2, 6, 0, 6, 3, 2, 5};

	if (sscanf(buf, "%d", &opt) != 1)
		return -EINVAL;

	if (1 == opt) {
		ret = bbk_goodix_writeUdd_V2_v1(temp_buf);
		if (ret)
			return -EINVAL;
	}

	return count;
}

static ssize_t goodix_ts_switch_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int mode;

	if (sscanf(buf, "%d", &mode) != 1)
		return -EINVAL;

	if (bbk_goodix_mode_change_V2(NULL, mode) == -1)
		return -EINVAL;

	return count;
}

static ssize_t goodix_ts_version_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int fw_ver = 0, cfg_ver = 0;

	fw_ver = bbk_goodix_get_fw_version_V2_v1(0);
	if (fw_ver == -1)
		return -EINVAL;

	cfg_ver = bbk_goodix_get_fw_version_V2_v1(1);
	if (cfg_ver == -1)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "fw version:%08x, cfg version:%02x\n",
						fw_ver, cfg_ver);
}

static ssize_t goodix_ts_gesture_point_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int i, ret = 0;
	int opt = 0;
	u16 temp_data[12] = {0};

	if (sscanf(buf, "%d", &opt) != 1)
		return -EINVAL;

	if (1 == opt) {
		ret = bbk_goodix_gesture_point_get_V2_v1(temp_data);
		if (-1 == ret)
			return -EINVAL;
	}

	for (i = 0; i < 6; i++) {
		VTI("[%d](%d,%d)", i, temp_data[i * 2],
				temp_data[i * 2 + 1]);
	}

	return count;
}

static ssize_t goodix_ts_set_charger_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int state;

	if (sscanf(buf, "%d", &state) != 1)
		return -EINVAL;

	if (bbk_goodix_set_charger_bit_V2(NULL, state))
		return -EINVAL;

	return count;
}

static ssize_t goodix_ts_edge_restain_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int state;

	if (sscanf(buf, "%d", &state) != 1)
		return -EINVAL;

	if (bbk_goodix_set_Edge_Switch_V2(NULL, state))
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(extmod_info, S_IRUGO, goodix_ts_extmod_show, NULL);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_ts_driver_info_show, NULL);
static DEVICE_ATTR(chip_info, S_IRUGO, goodix_ts_chip_info_show, NULL);
static DEVICE_ATTR(config_data, S_IRUGO, goodix_ts_config_data_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, goodix_ts_reset_store);
static DEVICE_ATTR(send_cfg, S_IWUSR | S_IWGRP, NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, S_IRUGO, goodix_ts_read_cfg_show, NULL);
static DEVICE_ATTR(irq_info, S_IRUGO | S_IWUSR | S_IWGRP,
		goodix_ts_irq_info_show, goodix_ts_irq_info_store);

static DEVICE_ATTR(bbk_rawdiff_data, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_get_rawdiff_data_store);
static DEVICE_ATTR(bbk_fw_update, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_fw_update_store);
static DEVICE_ATTR(bbk_enable_idle, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_enable_idle_store);
static DEVICE_ATTR(bbk_flash_udd, S_IRUGO | S_IWUSR | S_IWGRP,
		goodix_ts_flash_udd_show, goodix_ts_flash_udd_store);
static DEVICE_ATTR(bbk_switch_mode, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_switch_mode_store);
static DEVICE_ATTR(bbk_version_info, S_IRUGO, goodix_ts_version_info_show, NULL);
static DEVICE_ATTR(bbk_gesture_point, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_gesture_point_store);
static DEVICE_ATTR(bbk_set_charger, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_set_charger_store);
static DEVICE_ATTR(bbk_edge_restain_switch, S_IWUSR | S_IWGRP,
			NULL, goodix_ts_edge_restain_switch_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_extmod_info.attr,
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_config_data.attr,
	&dev_attr_reset.attr,
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_bbk_rawdiff_data.attr,
	&dev_attr_bbk_fw_update.attr,
	&dev_attr_bbk_enable_idle.attr,
	&dev_attr_bbk_flash_udd.attr,
	&dev_attr_bbk_switch_mode.attr,
	&dev_attr_bbk_version_info.attr,
	&dev_attr_bbk_gesture_point.attr,
	&dev_attr_bbk_set_charger.attr,
	&dev_attr_bbk_edge_restain_switch.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

int goodix_ts_sysfs_init_V2(struct goodix_ts_core *core_data)
{
	return sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* event notifier */
static BLOCKING_NOTIFIER_HEAD(ts_notifier_list);
/**
 * goodix_ts_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *  see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_register_notifier_V2(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_register_notifier_V2);

/**
 * goodix_ts_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_unregister_notifier_V2(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_unregister_notifier_V2);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_blocking_notify_V2(enum ts_notify_event evt, void *v)
{
	return blocking_notifier_call_chain(&ts_notifier_list,
			(unsigned long)evt, v);
}
EXPORT_SYMBOL_GPL(goodix_ts_blocking_notify_V2);



/**
 * goodix_ts_input_report - report touch event to input subsystem
 *
 * @dev: input device pointer
 * @touch_data: touch data pointer
 * return: 0 ok, <0 failed
 */
static int goodix_ts_input_report(struct goodix_ts_core *core_data,
		struct goodix_touch_data *touch_data, ktime_t kt)
{
	struct goodix_ts_coords *coords = &touch_data->coords[0];
	struct input_dev *dev = core_data->input_dev;
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct vts_device *vtsdev = core_data->vtsdev;
	unsigned int touch_num = touch_data->touch_num;
	static u32 pre_fin;
	static u8 pre_key;
	int i, id;

	/*first touch down and last touch up condition*/
	if (touch_num != 0 && pre_fin == 0x0000) {
		/*first touch down event*/
		//input_report_key(dev, BTN_TOUCH, 1);
		core_data->ts_dev->int_state = true;
	} else if (touch_num == 0 && pre_fin != 0x0000) {
		/*no finger exist*/
		//input_report_key(dev, BTN_TOUCH, 0);
		core_data->ts_dev->int_state = false;
	}

	/*report key, include tp's key and pen's key */
	if (unlikely(touch_data->have_key)) {
		for (i = 0; i < ts_bdata->panel_max_key; i++) {
			input_report_key(dev, ts_bdata->panel_key_map[i],
							touch_data->key_value & (1 << i));
		}
		pre_key = touch_data->key_value;
		/*VTI("$$$$$$pre_key:0x%02x",pre_key);*/
	} else if (pre_key != 0x00) {
		/*VTI("******no key, by pre_key is not ZERO! pre_key:0x%02x", pre_key);*/
		for (i = 0; i < ts_bdata->panel_max_key; i++) {
			if (pre_key & (1 << i)) {
				input_report_key(dev, ts_bdata->panel_key_map[i], 0);
				pre_key &= ~(1 << i);
				/*VTI("******report i:%d, key:%d leave", i, ts_bdata->panel_key_map[i]);*/
			}
		}
		/*VTI("******after, pre_key:0x%02x", pre_key);*/
	}

#if 1
	/*protocol B*/

	/*report pen*/
	if (touch_num >= 1 && touch_data->pen_down) {
		touch_num -= 1;

		input_mt_slot(dev, ts_bdata->panel_max_id * 2);
		input_report_abs(dev, ABS_MT_TRACKING_ID, touch_data->pen_coords[0].id);
		input_report_abs(dev, ABS_MT_TOOL_TYPE, MT_TOOL_PEN);


		input_report_abs(dev, ABS_MT_POSITION_X, touch_data->pen_coords[0].x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch_data->pen_coords[0].y);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, touch_data->pen_coords[0].w);
		input_report_abs(dev, ABS_MT_PRESSURE, touch_data->pen_coords[0].p);

		pre_fin |= 1 << 20;
		VTD("!!!!!!report pen  DOWN,%d,%d,%d",
				touch_data->pen_coords[0].x,
				touch_data->pen_coords[0].y,
				touch_data->pen_coords[0].p);
	} else {
		if (pre_fin & (1 << 20)) {
			input_mt_slot(dev, ts_bdata->panel_max_id * 2);
			input_report_abs(dev, ABS_MT_TRACKING_ID, -1);
			pre_fin &= ~(1 << 20);
			VTI("!!!!!!report pen LEAVE");
		}
	}

	/*report finger*/
	/*report up*/
	id = coords->id;
	for (i = 0; i < ts_bdata->panel_max_id * 2; i++){
		if(touch_num && i == id){
			 id = (++coords)->id;
			}else if (pre_fin & (1 << i)) {/* release touch */
				/*input_mt_slot(dev, i);
				input_report_abs(dev, ABS_MT_TRACKING_ID, -1);*/
				//vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
				//vts_report_point_up(vtsdev, i, touch_num, coords->x,  coords->y, coords->w,coords->p, 0);
				vts_report_point_up(vtsdev, i, touch_num, core_data->fingerPressRecord[i][0],  core_data->fingerPressRecord[i][1],
						core_data->fingerPressRecord[i][2], core_data->fingerPressRecord[i][3], 0, kt);
				pre_fin &= ~(1 << i);
				VTI("report leave:%d", i);
			}
		}
	/*report down*/
	coords = &touch_data->coords[0];
	id = coords->id;
	for (i = 0; i < ts_bdata->panel_max_id * 2; i++) {
		if (touch_num && i == id) { /* this is a valid touch down event */
			/*
			input_mt_slot(dev, id);
			input_report_abs(dev, ABS_MT_TRACKING_ID, i);
			input_report_abs(dev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER);

			input_report_abs(dev, ABS_MT_POSITION_X, coords->x);
			input_report_abs(dev, ABS_MT_POSITION_Y, coords->y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, coords->w);
			input_report_abs(dev, ABS_MT_PRESSURE, coords->p);
			*/

			//vivoTsInputReport(VTS_TOUCH_DOWN, id, coords->x, coords->y, coords->w);
			vts_report_point_down(vtsdev, id, touch_num, coords->x,  coords->y, coords->w,coords->p, 0, coords->custom_data, 2, kt);
			core_data->fingerPressRecord[id][0] = coords->x;
			core_data->fingerPressRecord[id][1] = coords->y;
			core_data->fingerPressRecord[id][2] = coords->w;
			core_data->fingerPressRecord[id][3] = coords->p;
			pre_fin |= 1 << i;
			id = (++coords)->id;
		}
	}
#endif

#if 0
	/*report pen use protocl A*/
	if (touch_data->pen_down) {
		/*input_report_key(dev, BTN_TOOL_PEN, 1);*/
		/*input_report_key(dev, BTN_TOUCH, 1);*/

		input_report_abs(dev, ABS_MT_POSITION_X, touch_data->pen_coords[0].x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch_data->pen_coords[0].y);

		input_report_abs(dev, ABS_MT_PRESSURE, touch_data->pen_coords[0].p);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, touch_data->pen_coords[0].w);
		/*input_report_abs(dev, ABS_MT_TRACKING_ID, touch_data->pen_coords[0].id);*/
		input_report_abs(dev, ABS_MT_TOOL_TYPE, 1);

		input_mt_sync(dev);
	} else {
		if (pre_fin & (1 << 10) && touch_num == 0) {
			/*input_report_key(dev, BTN_TOOL_PEN, 0);*/
			/*input_report_key(dev, BTN_TOUCH, 0);*/

			pre_fin &= ~(1 << 10);
		}
	}

	/* report abs */
	id = coords->id;
	for (i = 0; i < ts_bdata->panel_max_id; i++) {
		if (touch_num && i == id) { /* this is a valid touch down event */

			/*input_report_key(dev, BTN_TOUCH, 1);*/
			/*input_report_abs(dev, ABS_MT_TRACKING_ID, id);*/

			input_report_abs(dev, ABS_MT_POSITION_X, coords->x);
			input_report_abs(dev, ABS_MT_POSITION_Y, coords->y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, coords->w);
			input_report_abs(dev, ABS_MT_PRESSURE, coords->p);
			input_mt_sync(dev);

			pre_fin |= 1 << i;
			id = (++coords)->id;
		} else {
			if (pre_fin & (1 << i)) {
				/*input_mt_sync(dev);*/

				pre_fin &= ~(1 << i);
			}
		}
	}
#endif
	vts_report_point_sync(vtsdev);
	input_sync(dev);
	return 0;
}

struct goodix_ts_core *irq_core_data_V2;

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static void goodix_ts_threadirq_func(struct goodix_ts_core *data, ktime_t kt)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_device *ts_dev =  core_data->ts_dev;
	struct goodix_ext_module *ext_module;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	int r;

	core_data->irq_trig_cnt++;
	/* inform external module */
	list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
		if (!ext_module->funcs || !ext_module->funcs->irq_event)
			continue;
		r = ext_module->funcs->irq_event(core_data, ext_module);
		if (r == EVT_CANCEL_IRQEVT)
			//return IRQ_HANDLED;
			return;
	}

	/* read touch data from touch device */
	r = ts_dev->hw_ops->event_handler(core_data, ts_dev, ts_event, kt);
	if (likely(r >= 0)) {
		if (ts_event->event_type == EVENT_TOUCH) {
			/* report touch */
			goodix_ts_input_report(core_data,
					&ts_event->event_data.touch_data, kt);
		}
	}

	//return IRQ_HANDLED;
	return;
	
}

static irqreturn_t goodix_enter_irq(int irq, void *data, ktime_t kt)
{
	irq_core_data_V2 = (struct goodix_ts_core *)data;

	goodix_ts_threadirq_func(irq_core_data_V2, kt);

	return IRQ_HANDLED;
}


/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_setup_V2(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;
	struct vts_device *vtsdev = core_data->vtsdev;
	
	/* if ts_bdata-> irq is invalid */
	if (ts_bdata->irq <= 0) 
		core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	else 
		core_data->irq = ts_bdata->irq;

	VTI("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	r = vts_interrupt_register(vtsdev, core_data->irq, goodix_enter_irq, ts_bdata->irq_flags, core_data);
	/*
	r = devm_request_threaded_irq(&core_data->pdev->dev,
			core_data->irq, NULL,
			goodix_ts_threadirq_func,
			ts_bdata->irq_flags | IRQF_ONESHOT,
			GOODIX_CORE_DRIVER_NAME,
			core_data);
	*/
	if (r < 0)
		VTE("Failed to requeset threaded irq:%d", r);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return r;
}

/**
 * goodix_ts_irq_enable_V2 - Enable/Disable a irq
 * @core_data: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_enable_V2(struct goodix_ts_core *core_data,
			bool enable)
{
	if (enable) {
		if (!atomic_cmpxchg(&core_data->irq_enabled, 0, 1)) {
			enable_irq(core_data->irq);
			VTD("Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&core_data->irq_enabled, 1, 0)) {
			disable_irq(core_data->irq);
			VTD("Irq disabled");
		}
	}

	return 0;
}
EXPORT_SYMBOL(goodix_ts_irq_enable_V2);
/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct device *dev = NULL;
	struct goodix_ts_board_data *ts_bdata;
	int r = 0;

	VTI("Power init");

	/* dev:i2c client device or spi slave device*/
	dev =  core_data->ts_dev->dev;
	ts_bdata = board_data(core_data);

	if (gpio_is_valid(ts_bdata->power_gpio)) {
		r = gpio_request(ts_bdata->power_gpio, GOODIX_CORE_DRIVER_NAME);
		if (r) {
			VTE("gpio request fail!");
			return r;
		}
	} else {
		if (ts_bdata->avdd_name) {
			core_data->avdd = devm_regulator_get(dev,
				 	ts_bdata->avdd_name);
			if (IS_ERR_OR_NULL(core_data->avdd)) {
				r = PTR_ERR(core_data->avdd);
				VTE("Failed to get regulator avdd:%d", r);
				core_data->avdd = NULL;
				return r;
			}
		}
	}
	
	if (ts_bdata->dvdd_name) {
		core_data->dvdd = devm_regulator_get(dev,
				 ts_bdata->dvdd_name);
		if (IS_ERR_OR_NULL(core_data->dvdd)) {
			r = PTR_ERR(core_data->dvdd);
			VTE("Failed to get regulator dvdd:%d", r);
			core_data->dvdd = NULL;
			return r;
		}
	} else {
		if (gpio_is_valid(ts_bdata->dvdd_gpio)) {
			r = gpio_request(ts_bdata->dvdd_gpio, GOODIX_CORE_DRIVER_NAME);
			if (r) {
				VTE("dvdd_gpio request failed!");
				return r;
			}
		}
	}

	return r;
}

/**
 * goodix_ts_power_on_V2 - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_on_V2(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;

	VTI("Device power on");
	if (core_data->power_on)
		return 0;

	if (gpio_is_valid(ts_bdata->power_gpio)) {		
		r = gpio_direction_output(ts_bdata->power_gpio, 1); /* 3V */
		if (r) {
			VTE("unable to set direction out to 1 for gpio [%d]\n", ts_bdata->power_gpio);
		}

	} else {
		if (regulator_count_voltages(core_data->avdd) > 0) {
			r = regulator_set_voltage(core_data->avdd, 3000000,
								3000000);
			if (r) {
				VTI(" avdd regulator set_vtg:bus_reg vcc_ana failed r=%d", r);
				regulator_put(core_data->avdd);
				return r;
			}
		}
		regulator_set_load(core_data->avdd, 100000);
		r = regulator_enable(core_data->avdd);
		if (r) {
			VTI("Regulator avdd enable failed r=%d", r);
			return r;
		}

	}

	if (core_data->dvdd) {
		r = regulator_enable(core_data->dvdd);
		if (!r) {
			VTI("regulator enable SUCCESS");
			if (ts_bdata->power_on_delay_us)
				usleep_range(ts_bdata->power_on_delay_us,
						ts_bdata->power_on_delay_us);
		} 
	}else {
			if (gpio_is_valid(ts_bdata->dvdd_gpio)) {
				r = gpio_direction_output(ts_bdata->dvdd_gpio, 1);
				if (r) {
					VTE("unable to set direction out to 1 for gpio [%d]\n", ts_bdata->dvdd_gpio);
				}else {
					VTI("regulator dvdd_gpio enable SUCCESS");
					if (ts_bdata->power_on_delay_us)
						usleep_range(ts_bdata->power_on_delay_us,
							ts_bdata->power_on_delay_us);
				}
			}
		}
	
	/* set reset gpio high */
	if(gpio_is_valid(ts_bdata->reset_gpio)){
		VTI("reset high++++++++++");
		usleep_range(100, 110);
		gpio_direction_output(ts_bdata->reset_gpio, 1);
	}

	core_data->power_on = 1;
	return 0;
}

/**
 * goodix_ts_power_off_V2 - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_off_V2(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r;

	VTI("Device power off");
	if (!core_data->power_on)
		return 0;
	
	if(gpio_is_valid(ts_bdata->reset_gpio)){
		/* set reset gpio low */
		VTI("reset low---------------------");
		gpio_direction_output(ts_bdata->reset_gpio, 0);
		usleep_range(100, 110);
	}
	
	if (gpio_is_valid(ts_bdata->power_gpio)) {		
		r = gpio_direction_output(ts_bdata->power_gpio, 0); /*3V */
		if (r) {
			VTE("unable to set direction out to 1 for gpio [%d]\n", ts_bdata->power_gpio);
		}

	} else {
		regulator_set_load(core_data->avdd, 0);
		r = regulator_disable(core_data->avdd);
		if (!r) {
			VTI(" avdd regulator disable SUCCESS");

		}
		else {
			VTI(" avdd regulator disable failed");
			return r;
		}
	}

	if (core_data->dvdd) {
		r = regulator_disable(core_data->dvdd);
		if (!r) {
			VTI("regulator disable SUCCESS");
			if (ts_bdata->power_off_delay_us)
				usleep_range(ts_bdata->power_off_delay_us,
						ts_bdata->power_off_delay_us);
		} 
	}else {
			if (gpio_is_valid(ts_bdata->dvdd_gpio)) {
				r = gpio_direction_output(ts_bdata->dvdd_gpio, 0); /*1.8V */
				if (r) {
					VTE("unable to set direction out to 0 for dvdd_gpio [%d]\n", ts_bdata->dvdd_gpio);
				}
		  }
	 }


	core_data->power_on = 0;
	mdelay(50);
	return 0;
}

#ifdef CONFIG_PINCTRL
/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_ts_core *core_data)
{
	bool active = false;
	/* get pinctrl handler from of node */
	core_data->pinctrl = devm_pinctrl_get(core_data->ts_dev->dev);
    //VTI("-----pinctrl device----%s",core_data->ts_dev->dev);
	if (IS_ERR_OR_NULL(core_data->pinctrl)) {
		VTE("Failed to get pinctrl handler");
		return -1;
	}

	/* active state */
	core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		VTE("Failed to get pinctrl state:%s", PINCTRL_STATE_ACTIVE);
	} else {
		active = true;
	}

	/* suspend state */
	core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		VTE("Failed to get pinctrl state:%s", PINCTRL_STATE_SUSPEND);
	} else {
		active = true;
	}

	//set spi clk/mosi driver strength
	core_data->spi_mosi_active = pinctrl_lookup_state(core_data->pinctrl, "spi_mosi_active");
	if (IS_ERR_OR_NULL(core_data->spi_mosi_active)) {
		VTE("Failed to get spi mosi gpio pinctrl");
	} else {
		active = true;
	}

	core_data->spi_clk_active = pinctrl_lookup_state(core_data->pinctrl, "spi_clk_active");
	if (IS_ERR_OR_NULL(core_data->spi_clk_active)) {
		VTE("Failed to get spi clk gpio pinctrl");
	} else {
		active = true;
	}

	if (!active) {
		devm_pinctrl_put(core_data->pinctrl);
		core_data->pinctrl = NULL;
		return -1;
	}

	return 0;
}
#endif

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 *	reset_gpio and irq_gpio number are obtained from goodix_ts_device
 *  which created in hardware layer driver. e.g.goodix_xx_i2c.c
 *	A goodix_ts_device should set those two fileds to right value
 *	before registed to touch core driver.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata =
			board_data(core_data);
	int r = 0;

	VTI("GPIO setup,reset-gpio:%d, irq-gpio:%d",
	ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->reset_gpio,
			GPIOF_OUT_INIT_LOW,
			"ts_reset_gpio");
	if (r < 0) {
		VTE("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->irq_gpio,
			GPIOF_IN,
			"ts_irq_gpio");
	if (r < 0) {
		VTE("Failed to request irq gpio, r:%d", r);
		return r;
	}

	return 0;
}


/**
 * goodix_input_set_params - set input parameters
 */
static void goodix_ts_set_input_params(struct input_dev *input_dev,
		struct goodix_ts_board_data *ts_bdata)
{
	int i;

	if (ts_bdata->swap_axis)
		swap(ts_bdata->panel_max_x, ts_bdata->panel_max_y);

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			0, ts_bdata->panel_max_id, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			0, ts_bdata->panel_max_w, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			0, ts_bdata->panel_max_p, 0, 0);

	if (ts_bdata->panel_max_key) {
		for (i = 0; i < ts_bdata->panel_max_key; i++)
			input_set_capability(input_dev, EV_KEY,
					ts_bdata->panel_key_map[i]);
	}
}

/**
 * goodix_ts_input_dev_config_V2 - Requset and config a input device
 *  then register it to input sybsystem.
 *  NOTE that some hardware layer may provide a input device
 *  (ts_dev->input_dev not NULL).
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_input_dev_config_V2(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int r;

	input_dev = devm_input_allocate_device(&core_data->pdev->dev);
	if (!input_dev) {
		VTE("Failed to allocated input device");
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
	__set_bit(BTN_TOOL_PEN, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* set input parameters */
	goodix_ts_set_input_params(input_dev, ts_bdata);

	/*set ABS_MT_TOOL_TYPE*/
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
						0, 1, 0, 0);

#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	/*input_mt_init_slots(input_dev, ts_bdata->panel_max_id,
			INPUT_MT_DIRECT);*/
	input_mt_init_slots(input_dev,
			ts_bdata->panel_max_id * 2 + 1,
			INPUT_MT_DIRECT);
#else
	/*input_mt_init_slots(input_dev, ts_bdata->panel_max_id);*/
	input_mt_init_slots(input_dev,
			ts_bdata->panel_max_id * 2 + 1);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	r = input_register_device(input_dev);
	if (r < 0) {
		VTE("Unable to register input device");
		return r;
	}

	return 0;
}

/**
 * goodix_ts_hw_init_V2 - Hardware initilize
 *  poweron - hardware reset - sendconfig
 * @core_data: pointer to touch core data
 * return: 0 intilize ok, <0 failed
 */
int goodix_ts_hw_init_V2(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_hw_ops *hw_ops =
		ts_hw_ops(core_data);
	int r;

	/* reset touch device */
	if (hw_ops->reset) {
		r = hw_ops->reset(core_data->ts_dev);
		if (r < 0)
			goto exit;
	}

	/* init */
	if (hw_ops->init) {
		r = hw_ops->init(core_data->ts_dev);
		if (r < 0) {
			core_data->hw_err = true;
			goto exit;
		}
	}

exit:
	/* if bus communication error occured then
	 * exit driver binding, other errors will
	 * be ignored */
	if (r != -EBUS)
		r = 0;
	return r;
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
	struct goodix_ts_core *core = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(core);
	int r = 0;
	u8 data = 0xaa;

	if (ts_esd->esd_on == false)
		return;

	VTD("goodix_ts_esd_work function is running");

	if (hw_ops->check_hw)
		r = hw_ops->check_hw(core->ts_dev);
	if (r < 0) {
		goodix_ts_power_off_V2(core);
		msleep(100);
		goodix_ts_power_on_V2(core);
		if (hw_ops->reset)
			hw_ops->reset(core->ts_dev);

		/*init static esd*/
		if (core->ts_dev->ic_type == IC_TYPE_NANJING) {
			r = hw_ops->write(core->ts_dev,
					0x8043, &data, 1);
			if (r < 0)
				VTE("nanjing esd reset, init static esd FAILED, i2c wirte ERROR");
		}

		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			VTE("esd reset, init dynamic esd FAILED, i2c write ERROR");

		vts_abnormal_reset_collect(TOUCH_VCODE_UNEXPECTED_RESET_EVENT);
	} else {
		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			VTE("esd init watch dog FAILED, i2c write ERROR");
	}

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on)
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
	mutex_unlock(&ts_esd->esd_mutex);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
static void goodix_ts_esd_on(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	if (core->ts_dev->reg.esd == 0)
		return;

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on == false) {
		ts_esd->esd_on = true;
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
		mutex_unlock(&ts_esd->esd_mutex);
		VTI("Esd on");
		return;
	}
	mutex_unlock(&ts_esd->esd_mutex);
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
static void goodix_ts_esd_off(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	mutex_lock(&ts_esd->esd_mutex);
	if (ts_esd->esd_on == true) {
		ts_esd->esd_on = false;
		cancel_delayed_work(&ts_esd->esd_work);
		mutex_unlock(&ts_esd->esd_mutex);
		VTI("Esd off");
		return;
	}
	mutex_unlock(&ts_esd->esd_mutex);
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
	case NOTIFY_FWUPDATE_END:
	case NOTIFY_RESUME:
	case NOTIFY_ESD_ON:
		goodix_ts_esd_on(ts_esd->ts_core);
		break;
	}

	return 0;
}

/**
 * goodix_ts_esd_init_V2 - initialize esd protection
 */
int goodix_ts_esd_init_V2(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	struct goodix_ts_device *dev = core->ts_dev;
	u8 data = 0xaa;
	int r;

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	mutex_init(&ts_esd->esd_mutex);
	ts_esd->ts_core = core;
	ts_esd->esd_on = false;
	ts_esd->esd_notifier.notifier_call = goodix_esd_notifier_callback;
	goodix_ts_register_notifier_V2(&ts_esd->esd_notifier);

	if (core->ts_dev->board_data->esd_default_on &&
			dev->hw_ops->check_hw &&
			dev->reg.esd != 0) {
		/*init static esd*/
		if (dev->ic_type == IC_TYPE_NANJING) {
			r = dev->hw_ops->write_trans(core->ts_dev,
				0x8043, &data, 1);
			if (r < 0)
				VTE("static ESD init ERROR, i2c write failed");
		}

		/*init dynamic esd*/
		r = dev->hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			VTE("dynamic ESD init ERROR, i2c write failed");

		goodix_ts_esd_on(core);
	}
	return 0;
}

/**
 * goodix_ts_suspend_V2 - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to  sleep
 */
int goodix_ts_suspend_V2(struct goodix_ts_core *core_data, int is_sleep)
{
	struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int r;

	VTI("Suspend start");

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify_V2(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (!ext_module->funcs || !ext_module->funcs->before_suspend)
				continue;

			if (is_sleep) {
				if (strcmp(ext_module->name, "Goodix_gsx_gesture_V2") == 0)
					continue;
			}

			r = ext_module->funcs->before_suspend(core_data, ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_V2.mutex);
				VTI("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_V2.mutex);

	/* disable irq */
	goodix_ts_irq_enable_V2(core_data, false);

	/* let touch ic work in sleep mode */
	if (ts_dev && ts_dev->hw_ops->suspend)
		ts_dev->hw_ops->suspend(ts_dev);
	atomic_set(&core_data->suspended, 1);
	atomic_set(&core_data->gestured, 0);

#ifdef CONFIG_PINCTRL
if(core_data->power_off_sleep){//only when power off in sleep ,should select suspend pinctrl state  
	if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		VTI("select suspend pinstate");
		r = pinctrl_select_state(core_data->pinctrl,
				core_data->pin_sta_suspend);
		if (r < 0)
			VTE("Failed to select active pinstate, r:%d", r);
	}
}
#endif

	/* inform exteranl modules */
	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (!ext_module->funcs || !ext_module->funcs->after_suspend)
				continue;

			r = ext_module->funcs->after_suspend(core_data, ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules_V2.mutex);
				VTI("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_V2.mutex);

out:
	/* release all the touch IDs */
	core_data->ts_event.event_data.touch_data.touch_num = 0;
	goodix_ts_input_report(core_data,
			&core_data->ts_event.event_data.touch_data, ktime_get());
	VTI("Suspend end");
	return 0;
}

/**
 * goodix_ts_resume_V2 - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
int goodix_ts_resume_V2(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module;
	struct goodix_ts_device *ts_dev =
				core_data->ts_dev;
	int r;

	VTI("Resume start");
	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (!ext_module->funcs || !ext_module->funcs->before_resume)
				continue;

			r = ext_module->funcs->before_resume(core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_V2.mutex);
				VTI("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_V2.mutex);

#ifdef CONFIG_PINCTRL
	if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		VTI("select active pinstate");
		r = pinctrl_select_state(core_data->pinctrl,
					core_data->pin_sta_active);
		if (r < 0)
			VTE("Failed to select active pinstate, r:%d", r);
	}
#endif

	atomic_set(&core_data->suspended, 0);
	atomic_set(&core_data->gestured, 0);

	/* resume device */
	if (ts_dev && ts_dev->hw_ops->resume)
		ts_dev->hw_ops->resume(ts_dev);

	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (!ext_module->funcs || !ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data, ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules_V2.mutex);
				VTI("Canceled by module:%s", ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules_V2.mutex);
	if(ts_dev)
    ts_dev->goodix_sensor_test = 0;
	goodix_ts_irq_enable_V2(core_data, true);
out:
	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	goodix_ts_blocking_notify_V2(NOTIFY_RESUME, NULL);
	VTD("Resume end");
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * goodix_ts_earlysuspend - Early suspend function
 * Called by kernel during system suspend phrase
 */
static void goodix_ts_earlysuspend(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_suspend_V2(core_data, 1);
}
/**
 * goodix_ts_lateresume - Late resume function
 * Called by kernel during system wakeup
 */
static void goodix_ts_lateresume(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_resume_V2(core_data);
}
#endif

#ifdef CONFIG_PM
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_suspend_V2(core_data, 1);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_resume_V2(core_data);
}
#endif
#endif

/**
 * goodix_generic_noti_callback_V2 - generic notifier callback
 *  for goodix touch notification event.
 */
int goodix_generic_noti_callback_V2(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct goodix_ts_core *ts_core = container_of(self,
			struct goodix_ts_core, ts_notifier);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(ts_core);
	int r;

	switch (action) {
	case NOTIFY_FWUPDATE_END:
		if (ts_core->hw_err && hw_ops->init) {
			/* Firmware has been updated, we need to reinit
			 * the chip, read the sensor ID and send the
			 * correct config data based on sensor ID.
			 * The input parameters also needs to be updated.*/
			r = hw_ops->init(ts_core->ts_dev);
			if (r < 0)
				goto exit;

			goodix_ts_set_input_params(ts_core->input_dev,
					ts_core->ts_dev->board_data);
			ts_core->hw_err = false;
		}
		break;
	}

exit:
	return 0;

}


extern int touch_state;
extern void bbk_goodix_cmds_init_V2(struct goodix_ts_cmd *ts_cmd,
					     u8 cmds, u8 cmd_data, u32 reg_addr);
/*
static int goodix_report_finger_icon(int flag)
{
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd finger_icon_cmd;
	int ret = 0;

	if (atomic_read(&(vivoTsGetVtsData()->run_mode)) != TOUCHSCREEN_GESTURE) {
		VTI("run_mode is not gesture mode, no need to change scan rate");
		return -EPERM;
	}

	if (touch_state != VTS_ST_GESTURE) {
		VTI("run_mode is not gesture mode, no need to change scan rate");
		return -EPERM;
	}
	
	if (flag == 1) {  // The Fingerprint icon is light 
		bbk_goodix_cmds_init_V2(&finger_icon_cmd, 0x1a, 0x01, ts_dev->reg.command);
	} else if (flag == 3) {  // The Fingerprint icon is extinguish 
		bbk_goodix_cmds_init_V2(&finger_icon_cmd, 0x1a, 0x02, ts_dev->reg.command);
	} else {
		VTE("set finger icon flag is not 1 or 3 !");
		return -EPERM;
	}
	VTD("%02X, %02X, %02X", finger_icon_cmd.cmds[0], finger_icon_cmd.cmds[1], finger_icon_cmd.cmds[2]);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_icon_cmd);
	if (ret) {
		VTE("Send finger icon command error");
		ret = -EPERM;
	}

	return ret;
}
*/

int get_faceDect_state(void) {
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	VTI("22 enter in get_faceDect_state");
	return core_data->face_detect_cmd;
}

#define BBK_CMD_ADDR_SPI    		0x30F1
static int goodix_faceDetect_startEnd(struct vts_device *vtsdev, int cmd)
{
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd face_detect_cmd;
	int ret = 0;
	int i = 0;
	u8 read_val = 0;
	u16 feed_reg;

	VTI("---------------------cmd is %d", cmd);
	if (cmd == 1) {
		bbk_goodix_cmds_init_V2(&face_detect_cmd, 0x12, 0x00, ts_dev->reg.command);
		VTD("%02X, %02X, %02X", face_detect_cmd.cmds[0], face_detect_cmd.cmds[1], face_detect_cmd.cmds[2]);
		core_data->face_detect_cmd = cmd;
	} else if (cmd == 0) {
		bbk_goodix_cmds_init_V2(&face_detect_cmd, 0x13, 0x00, ts_dev->reg.command);
		VTD("%02X, %02X, %02X", face_detect_cmd.cmds[0], face_detect_cmd.cmds[1], face_detect_cmd.cmds[2]);
		core_data->face_detect_cmd = cmd;
	} else {
		ret = -1;
		VTI("The cmd is invalid!");
		goto END;
	}

	if (ts_dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		feed_reg = BBK_CMD_ADDR_SPI;
		for (i = 0; i < 5; i++) {
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &face_detect_cmd);
			if (ret) {
				VTE("Send finger icon command error");
				ret = -1;
			}
			mdelay(10);
			if (ts_dev->hw_ops->read(ts_dev, feed_reg, &read_val, 1))
				VTE("Read cmd reg fail.");
			else
				VTI("Read cmd reg value:%02x.", read_val);

			VTI("read_val:%d", read_val);
			if (cmd) {
				if (read_val & 0x10) {
					VTI("------enable faceDetect completely!");
					ret = 0;
					break;
				}
			} else {
				if (!(read_val & 0x10)) {
					VTI("------disable faceDetect completely!");
					ret = 0;
					break;
				}
			}
		}
	} else {
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &face_detect_cmd);
		if (ret) {
			VTE("Send finger icon command error");
			ret = -1;
		}
		mdelay(10);
		if (cmd)
			VTI("------enable faceDetect completely!");
		else
			VTI("------disable faceDetect completely!");
	}

END:
	return ret;
}


#define GOODIX_FW_STATE_ADDR          0x3101
#define GOODIX_FW_STATE_ADDR_SPI      0x3F01

#define GOODIX_FW_STATE_BIT           (0x01 << 3)

static int bbk_goodix_get_ic_mode(struct vts_device *vtsdev)
{
u8 val;
int ret;
struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
struct goodix_ts_device *ts_dev = core_data->ts_dev;

if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0){
	    
		VTE("disable doze mode FAILED before set gesture area");
		
		goto exit;
}
if(ts_dev->ic_type == IC_TYPE_NORMANDY_SPI)
	ret = ts_dev->hw_ops->read_trans(ts_dev, GOODIX_FW_STATE_ADDR_SPI, &val, 1);
else
	ret = ts_dev->hw_ops->read_trans(ts_dev, GOODIX_FW_STATE_ADDR, &val, 1);
if(ret < 0){
 	VTE("Failed to get IC mode from GTxxx");
	val = -1;
	goto exit;
}
VTI("read value:%02x",val);

val &= GOODIX_FW_STATE_BIT;

VTI("0:gesture-----1:normal----ic read mode is %d",!val);

exit:
if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0){
	    
		VTE("enable doze mode FAILED after set ");
		return -1;
}

return !val;

}
void covert_point_pixel(struct vts_device *vts_dev, u16 src_x, u16 src_y, u16 *des_x, u16 *des_y, int default_zero)
{
	u32 display_max_x = 1;
	u32 display_max_y = 1;
	u32 touch_max_x = 1;
	u32 touch_max_y = 1;
	u32 resolution = 0;
 	vts_property_get(vts_dev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		VTI("get resolution adjust parm");
		vts_property_get(vts_dev, VTS_PROPERTY_DISPLAY_X, &display_max_x);
		vts_property_get(vts_dev, VTS_PROPERTY_DISPLAY_Y, &display_max_y);
		vts_property_get(vts_dev, VTS_PROPERTY_DIMENTION_X, &touch_max_x);
		vts_property_get(vts_dev, VTS_PROPERTY_DIMENTION_Y, &touch_max_y);
		//VTI("display_max_x:%d--display_max_y:%d--touch_max_x:%d--touch_max_y:%d--",display_max_x,display_max_y,touch_max_x,touch_max_y);
	}else{
		*des_x = src_x;
		*des_y = src_y;
		 return;
	}
	if(default_zero){//screen clock 
	*des_x = (u32)(src_x * touch_max_x)/display_max_x;
	*des_y = (u32)(src_y * touch_max_y)/display_max_y;	
	}
	else {//normal 
	*des_x = (u32)(src_x * display_max_x)/touch_max_x;
	*des_y = (u32)(src_y * display_max_y)/touch_max_y;
	}
	
}

static int bbk_set_goodix_clock_area(struct vts_device *vtsdev, int state)
{   
    u16 start_x_coor,start_y_coor,width,height;
	u16 start_x_coor_temp,start_y_coor_temp,width_temp,height_temp;
  	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd gesture_area_cmd;
	struct vts_screen_clock_cmd  sclock_cmd;
    u8 buffer[10] = {0};
	char feed_back = 0x00;
	int ret;
	vts_get_screen_clock_zone(&sclock_cmd, &vtsdev->screen_clock_zone);
	start_x_coor_temp = sclock_cmd.x;
	start_y_coor_temp = sclock_cmd.y;
	width_temp= sclock_cmd.width;
	height_temp= sclock_cmd.height;
	
	covert_point_pixel(vtsdev, start_x_coor_temp, start_y_coor_temp, &start_x_coor, &start_y_coor, 1);
	covert_point_pixel(vtsdev, width_temp, height_temp, &width, &height, 1);


	if(0 == width && 0 == height){//disable 
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0)
		VTE("disable doze mode FAILED before set gesture area");
	
		 bbk_goodix_cmds_init_V2(&gesture_area_cmd, 0x1C, 0X00, ts_dev->reg.command);
		 ret =  ts_dev->hw_ops->send_cmd(ts_dev, &gesture_area_cmd);
		 if (ret < 0) {
			VTE("Failed to send gesture area cmd");
			goto exit;
		 }
		 VTI("success to disable gesture report clock dclik ");
		 if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		 VTE("disable doze mode FAILED after set gesture area");
		 return ret;
	}

	
	buffer[0] = start_x_coor & 0xFF;
	buffer[1] = start_x_coor >> 8;
	buffer[2] = start_y_coor & 0xFF;
	buffer[3] = start_y_coor >> 8;
	buffer[4] = width & 0xFF;
	buffer[5] = width >> 8;
	buffer[6] = height & 0xFF;
	buffer[7] = height >> 8;


	buffer[9] = checksum_u8(buffer,8);
    
	buffer[8] = 0 -buffer[9];
	
    VTI("write :buffer[] = 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",
	buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);

	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0)
		VTE("disable doze mode FAILED before set gesture area");


	ret = ts_dev->hw_ops->write_trans(ts_dev, BBK_GES_AREA_ADDR, buffer, 9);
	if (ret < 0) {
		VTE("Failed to write area to GT9886");
		goto exit;
	}
   
	bbk_goodix_cmds_init_V2(&gesture_area_cmd, 0x1C, 0X01, ts_dev->reg.command);
	ret =  ts_dev->hw_ops->send_cmd(ts_dev, &gesture_area_cmd);
	if (ret < 0) {
		VTE("Failed to send gesture area cmd");
		goto exit;
	}
	msleep(30);
	// the feedback operating can add for debug first , if there is no more abnormal ,it can be remove.
	ret = ts_dev->hw_ops->read_trans(ts_dev, BBK_GES_AREA_FEED_BACK_ADDR, &feed_back, 1);
	if (ret < 0) {
		VTE("Failed to write area to GT IC");
		goto exit;
	} else if (0xDD == feed_back){
		VTI("Gesture area set successed!");
	} else {
		VTI("Gesture area set FAILED! ,feed_back : 0x%02x",feed_back);
	}

exit:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		VTE("disable doze mode FAILED after set ");

	return ret;

	
}


static int bbk_goodix_set_screen_clock_region(struct vts_device *vtsdev,int report_enable)
{   
    u16 start_x_coor,start_y_coor,width,height;
	u16 start_x_coor_temp,start_y_coor_temp,width_temp,height_temp;
  	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd gesture_area_cmd;
	struct vts_screen_clock_cmd  sclock_cmd;
    u8 buffer[10] = {0};
	char feed_back = 0x00;
	int ret;
	if(0 == report_enable){//disable 
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0)
		VTE("disable doze mode FAILED before set gesture area");
	
		 bbk_goodix_cmds_init_V2(&gesture_area_cmd, 0x1B, 0X00, ts_dev->reg.command);
		 ret =  ts_dev->hw_ops->send_cmd(ts_dev, &gesture_area_cmd);
		 if (ret < 0) {
			VTE("Failed to send gesture area cmd");
			goto exit;
		 }
		 VTI("success to disable gesture report abs ");
		 if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		 VTE("disable doze mode FAILED after set gesture area");
		 return ret;
	}
	vts_get_screen_clock_zone(&sclock_cmd, &vtsdev->screen_clock_zone);
	start_x_coor_temp = sclock_cmd.x;
	start_y_coor_temp = sclock_cmd.y;
	width_temp= sclock_cmd.width;
	height_temp= sclock_cmd.height;
	
	covert_point_pixel(vtsdev, start_x_coor_temp, start_y_coor_temp, &start_x_coor, &start_y_coor, 1);
	covert_point_pixel(vtsdev, width_temp, height_temp, &width, &height, 1);
	/*
	if(width >= vtsdev->module->properties[VTS_PROPERTY_DIMENTION_X])
	width = vtsdev->module->properties[VTS_PROPERTY_DIMENTION_X] - 1;
	if(height >= vtsdev->module->properties[VTS_PROPERTY_DIMENTION_Y])
	height = vtsdev->module->properties[VTS_PROPERTY_DIMENTION_Y] - 1;
	*/
	
	buffer[0] = start_x_coor & 0xFF;
	buffer[1] = start_x_coor >> 8;
	buffer[2] = start_y_coor & 0xFF;
	buffer[3] = start_y_coor >> 8;
	buffer[4] = width & 0xFF;
	buffer[5] = width >> 8;
	buffer[6] = height & 0xFF;
	buffer[7] = height >> 8;


	buffer[9] = checksum_u8(buffer,8);
    
	buffer[8] = 0 -buffer[9];
	
    VTI("write :buffer[] = 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",
	buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);

	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0)
		VTE("disable doze mode FAILED before set gesture area");


	ret = ts_dev->hw_ops->write_trans(ts_dev, BBK_GES_AREA_ADDR, buffer, 9);
	if (ret < 0) {
		VTE("Failed to write area to GT9886");
		goto exit;
	}
   
	bbk_goodix_cmds_init_V2(&gesture_area_cmd, 0x1B, 0X01, ts_dev->reg.command);
	ret =  ts_dev->hw_ops->send_cmd(ts_dev, &gesture_area_cmd);
	if (ret < 0) {
		VTE("Failed to send gesture area cmd");
		goto exit;
	}
	msleep(30);
	// the feedback operating can add for debug first , if there is no more abnormal ,it can be remove.
	ret = ts_dev->hw_ops->read_trans(ts_dev, BBK_GES_AREA_FEED_BACK_ADDR, &feed_back, 1);
	if (ret < 0) {
		VTE("Failed to write area to GT9886");
		goto exit;
	} else if (0xDD == feed_back){
		VTI("Gesture area set successed!");
	} else {
		VTI("Gesture area set FAILED! ,feed_back : 0x%02x",feed_back);
	}

exit:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		VTE("disable doze mode FAILED after set ");

	return ret;

	
}
#define FP_STATE_ADDR      0x3102
#define FP_STATE_ADDR_SPI  0x30f2

static int bbk_goodix_finger_cmd_check(struct goodix_ts_device *ts_dev, enum goodix_cmd_type cmd_type ,int enable)
{
	int ret, r, i;
	u8 buf = 0xff;
	ret = 0;

	VTD("enter :%s",__FUNCTION__);
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0)
		VTE("disable doze mode FAILED before read finger state");

	r = ts_dev->hw_ops->read_trans(ts_dev, 0x3101, &buf, 1);
	if (r < 0)
		VTD("Read from 0x3101 faild, ret=%d", r);
	else
		VTD("read from 0x3101 value: %02x", buf);

	buf = 0xff;
	for (i = 0; i < 3; i++){
		mdelay(10);
		if(ts_dev->ic_type == IC_TYPE_NORMANDY_SPI)
			r = ts_dev->hw_ops->read_trans(ts_dev, FP_STATE_ADDR_SPI, &buf, 1);
		else
			r = ts_dev->hw_ops->read_trans(ts_dev, FP_STATE_ADDR, &buf, 1);

		VTI("enable = %d,read value = %02x",enable, buf);
		if(cmd_type == FINGER_HILIGHT_CMD){// finger hilight  check bit 1
			if((buf&0x02) == 0x02 && 1 == enable)
				break;
			if((buf&0x02) == 0x00 && 0 == enable)
				break;
		}
		else if(cmd_type == FINGER_MODE_CMD){//finger mode cmd check bit 0
			if((buf&0x01) == 0x01 && 1 == enable)
				break;
			if((buf&0x01) == 0x00 && 0 == enable)
				break;
		}
	}
	if(i == 3){
		VTE("read goodix finger state fail!");
		ret = -1;
	}
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		VTE("enable doze mode FAILED after read finger state");

	return ret;

}
static int bbk_goodix_long_press_enable(struct vts_device *vtsdev, int enable) {
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd finger_dclick_cmd;
	int ret = 0;
	int retry_time = 0;
	u32 fp_feedback;
	VTI("enale = %d", enable);
	do{
		if (enable) {
			bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x00, ts_dev->reg.command);
			VTD("%02X, %02X, %02X", finger_dclick_cmd.cmds[0], finger_dclick_cmd.cmds[1], finger_dclick_cmd.cmds[2]);
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
			if (ret) {
				VTE("Send finger dclick command error");
			}
		}
		else {
			bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x01, ts_dev->reg.command);
			VTD("%02X, %02X, %02X", finger_dclick_cmd.cmds[0], finger_dclick_cmd.cmds[1], finger_dclick_cmd.cmds[2]);
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
			if (ret) {
				VTE("Send finger dclick command error");
			}
		}
		vts_property_get(vtsdev,VTS_PROPERTY_FD_FEED_BACK,&fp_feedback);
		if(fp_feedback) {
			ret = bbk_goodix_finger_cmd_check(ts_dev, FINGER_HILIGHT_CMD, enable);
			if(ret >= 0)
				break;
			retry_time++;
			VTI("retry times: %d", retry_time);
		} else {
			break;
		}
	} while (retry_time < 3);

	return ret;

}
#define  GT9886_TP_TX_BIT   0x6EA0
#define  GT9886_TP_RX_BIT   0x6EA2
#define  GT9886_TX_BYTE_NUM 2    
#define  GT9886_RX_BYTE_NUM 5 


u8 get_bit_value(u8 *buf, u8 bit_num)
{
	 u8 ret = 0;
	 ret = (*buf & (0x01<<bit_num));
	 return ret;
}

int bbk_goodix_get_channel_comp(struct vts_device *vtsdev ,bool true_false)
{
	u8 i,j,k,flag = 0;
	u8 Tx_num = 0;
	u8 Rx_num = 0;
	static u8 tx_buf[GT9886_TX_BYTE_NUM] = {0};
	static u8 rx_buf[GT9886_RX_BYTE_NUM] = {0};
	u8 tx_buf_temp[GT9886_TX_BYTE_NUM];
	u8 rx_buf_temp[GT9886_RX_BYTE_NUM];
	u16 Tx_bit = 0;
	int r = 0;
	long long Rx_bit = 0;
	u32 drv_num,sen_num;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	r =  ts_dev->hw_ops->read(ts_dev, GT9886_TP_TX_BIT, &tx_buf_temp[0], GT9886_TX_BYTE_NUM);
	if(r < 0){
		VTE("get TX bit error ");
		goto exit;
	}
	r = ts_dev->hw_ops->read(ts_dev, GT9886_TP_RX_BIT, &rx_buf_temp[0], GT9886_RX_BYTE_NUM);
	if(r < 0){
		VTE("get RX bit error ");
		goto exit;
	}
	
	if(0 == strcmp(tx_buf_temp,tx_buf) && 0 == strcmp(rx_buf_temp,rx_buf)){
		VTI("panel channel state is ok");
		goto exit;
	}
	
	VTI("TX BIT IN IC :%02x,%02x",tx_buf_temp[0],tx_buf_temp[1]);
	//dtsi 
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, &drv_num);
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, &sen_num);
	VTI("drv num:%d",drv_num);
	if(0 != strcmp(tx_buf_temp,tx_buf)){
		for(i =0; i < drv_num; i++){
			j = i % 8;
			k = i / 8;
			if(get_bit_value(&tx_buf_temp[k],j))
			{
				Tx_num++;
			}
			Tx_bit |= (tx_buf_temp[k]<<(k*8));
			
		}
		flag++;
	}
	VTI("RX BIT IN IC :%02x,%02x,%02x,%02x,%02x",rx_buf_temp[0],rx_buf_temp[1],rx_buf_temp[2],rx_buf_temp[3],rx_buf_temp[4]);
	
	VTI("sen num:%d",sen_num);
	if(0 != strcmp(rx_buf_temp,rx_buf)){
		for(i =0; i < sen_num; i++){
			j = i % 8;
			k = i / 8;
			if(get_bit_value(&rx_buf_temp[k],j))
			{
				Rx_num++;
			}
			Rx_bit |= (rx_buf_temp[k]<<(k*8));
		}
		flag++;
	}

	VTI("TXnum:%d,RXnum:%d,TXbit:%04x,Rxbit:%06llx",Tx_num,Rx_num,Tx_bit,Rx_bit);
	if(flag){
		vts_channel_broken_collect(TOUCH_VCODE_CHN_EVENT, Tx_num, Rx_num, Tx_bit, Rx_bit);
	}

exit:
	return r;

}


static int goodix_send_gesture_bit_cmd(struct vts_device *vtsdev, u16 bit_map)
{
	u8 buf[2];
	int ret;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	buf[0] = bit_map & 0xff;
	buf[1] = bit_map >> 8;
	VTI("goodix bit map:%02X, %02X",buf[0],buf[1]);
	//set bit map 
	ret = ts_dev->hw_ops->write_trans(ts_dev,GESTURE_BIT_CMD_ADDR, &buf[0],2);
	if (ret) {
		VTE("Send bit map error");
		return 0;
	}
	mdelay(8);
	buf[0] = 0;
	buf[1] = 0;
	ret = ts_dev->hw_ops->read_trans(ts_dev,GESTURE_BIT_CMD_ADDR, &buf[0],2);
	if (ret) {
		VTE("read bit set error");
		return 0;
	}
	VTI("bit read in IC:buf:%02x--%02x",buf[0],buf[1]);

	return 0;
}


u32 goodix_gest_map[] = {
	[0] 		= 	VTS_GESTURE_C,
	[1]  		=	VTS_GESTURE_E,
	[2] 		=	VTS_GESTURE_F,
	[3] 		= 	VTS_GESTURE_M,
	[4]  		= 	VTS_GESTURE_O,
	[5] 		= 	VTS_GESTURE_W,
	[6] 		= 	VTS_GESTURE_A,
	[7] 	    = 	VTS_GESTURE_DCLICK,
	[8] 		= 	VTS_GESTURE_UP,
	[9] 		= 	VTS_GESTURE_DOWN,
	[10] 		= 	VTS_GESTURE_LR,
	[11] 		= 	VTS_GESTURE_LR,

};
static int set_bit_one(u16 *psrc ,u8 bit_num)
{
	 *psrc |= ((0x01 << bit_num));
	  return 0;
}

static int bbk_goodix_set_gesture(struct vts_device *vtsdev, int enable)
{
	u8 i = 0;
	u8 buf[2];
	int ret = 0;
	u16 gesture_bit_map = 0;
	int gesture_type = enable;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0){
		VTE("disable doze mode FAILED before set gesture ");
		goto exit;
	}

 	ret = ts_dev->hw_ops->read_trans(ts_dev,GESTURE_BIT_CMD_ADDR, &buf[0],2);
	//set default value 
	gesture_bit_map &= 0xf000;
	for(i=0;i<12;i++){
		if(0 == (gesture_type & goodix_gest_map[i])){
		  set_bit_one(&gesture_bit_map, i);
		}
	}
	VTI("i: gesture_bit_map:%d--%04X",i,gesture_bit_map);
	if(gesture_bit_map & 0x0fff){
		//send  cmd  
		goodix_send_gesture_bit_cmd(vtsdev,gesture_bit_map);
	}
	
	exit:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		VTE("enable doze mode FAILED after set gesture ");

	return ret;
}
#define INPUT_METHOD_ADDR   0x6EAB
static int bbk_goodix_set_input_method(struct vts_device *vtsdev, int state)
{
	u8 buf[2];
	int ret = 0;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0){
		VTE("disable doze mode FAILED before send input method ");
		goto exit;
	}
    if(state == 1)
		buf[0] = 0xAA;
	else
		buf[0] = 0x00;
 	ret = ts_dev->hw_ops->write_trans(ts_dev,INPUT_METHOD_ADDR, &buf[0],1);
	VTI("send input method data :%02x",buf[0]);
	exit:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0)
		VTE("enable doze mode FAILED after  send input method ");

	return ret;

}
static int bbk_goodix_set_doze_state(struct vts_device *vtsdev, int state)
{

 	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	if(ts_dev->hw_ops->set_doze_mode(ts_dev,state) != 0){
		VTE("set doze state FAILED,state = %d",state);
		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
		return -1;
	}else{
		VTD("set doze mode SUCCESS,state = %d ",state);	 	
	}
	return 0;
		
}
//set_finger_mode)(struct vts_device *vtsdev, int mode);
static int bbk_goodix_set_finger_mode(struct vts_device *vtsdev, int mode) {
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_cmd finger_dclick_cmd;
	int ret = 0;
	int retry_time = 0;
	u32 fp_feedback = 0;
	VTI("mode = %d", mode);
	do{
		if (mode) {
			bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x1A, 0xFF, ts_dev->reg.command);
			VTI("%02X, %02X, %02X", finger_dclick_cmd.cmds[0], finger_dclick_cmd.cmds[1], finger_dclick_cmd.cmds[2]);
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
			if (ret) {
				VTE("Send keep active command error");
			}
		}
		else {
			bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x1A, 0x00, ts_dev->reg.command);
			VTD("%02X, %02X, %02X", finger_dclick_cmd.cmds[0], finger_dclick_cmd.cmds[1], finger_dclick_cmd.cmds[2]);
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
			if (ret) {
				VTE("Send enable idle command error");
			}
		}
		vts_property_get(vtsdev,VTS_PROPERTY_FD_FEED_BACK,&fp_feedback);
		if(fp_feedback) {
			ret = bbk_goodix_finger_cmd_check(ts_dev, FINGER_MODE_CMD ,mode);
			if(ret >= 0)
				break;
			retry_time++;
			VTI("retry times: %d", retry_time);
		} else {
			break;
		}
	} while(retry_time < 3);

	return ret;

}

int FPPointCnt_V2;
/**
 * goodix_ts_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */

static const struct vts_operations gt9886_ops = {
	.set_charging = bbk_goodix_set_charger_bit_V2,
	.set_rotation = bbk_goodix_set_Edge_Switch_V2,
	.update_firmware = goodix_cfg_bin_proc_V2,
	.change_mode = bbk_goodix_mode_change_V2,
	.get_fw_version = bbk_goodix_get_fw_version_V2,
	.rom_size = bbk_goodix_get_flash_size,
	.rom_read = bbk_goodix_readUdd_V2,
	.rom_write = bbk_goodix_writeUdd_V2,
	.set_auto_idle = bbk_goodix_set_auto_idle_V2,
	.get_frame = bbk_goodix_get_rawordiff_data_V2,
	.set_virtual_prox = goodix_faceDetect_startEnd,
	.set_long_press = bbk_goodix_long_press_enable,
	.get_ic_mode = bbk_goodix_get_ic_mode,
	.set_screen_clock_report_abs = bbk_goodix_set_screen_clock_region,
	.set_gesture = bbk_goodix_set_gesture,
	.set_input_method = bbk_goodix_set_input_method,
	.get_tp_channel_comp_data = bbk_goodix_get_channel_comp,
	.set_bus_state = bbk_goodix_set_doze_state,
	.set_screen_clock_area = bbk_set_goodix_clock_area,
	.set_finger_mode = bbk_goodix_set_finger_mode,
	.dump = bbk_goodix_dump_fw_data
};

static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = NULL;
	struct goodix_ts_device *ts_device;
	struct vts_device *vtsdev = NULL;
	int r;
	int i2c_read_max_size =0;
	u8 read_val = 0;
	VTI("goodix_ts_probe IN======1");

	ts_device = pdev->dev.platform_data;
	if (!ts_device || !ts_device->hw_ops ||
			!ts_device->board_data) {
		VTE("Invalid touch device");
		return -ENODEV;
	}

	vtsdev = vts_device_alloc();
	if (!vtsdev) {
		VTE("Alloc failed");
		return -ENOMEM;
	}
	VTI("goodix_ts_probe IN======2");
	
	FPPointCnt_V2 = 0;
	core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_ts_core),
						GFP_KERNEL);
	if (!core_data) {
		VTE("Failed to allocate memory for core data");
		return -ENOMEM;
	}
	VTI("goodix_ts_probe IN======3");
	core_data->vtsdev = vtsdev;
	vtsdev->ops = &gt9886_ops;
	r = vts_parse_dt_property(vtsdev, ts_device->dev->of_node);
	if (r == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto err_free_vts;
	}
	vts_set_drvdata(vtsdev, core_data);
	r = vts_register_driver(vtsdev);
	if (r) {
		VTE("register vts driver failed!, ret = %d\n", r);
	}
	VTI("goodix_ts_probe IN======3");
	/*init i2c_set_doze_mode para*/
	ts_device->doze_mode_set_count = 0;
	mutex_init(&ts_device->doze_mode_lock);
	/* i2c reset mutex */
	mutex_init(&ts_device->i2c_reset_mutex);
	/*i2c access mutex */
	mutex_init(&ts_device->i2c_access_mutex);
	
	/* touch core layer is a platform driver */
	ts_device->goodix_sensor_test = 0;
	ts_device->vtsdev = vtsdev;
	core_data->pdev = pdev;
	core_data->ts_dev = ts_device;
	platform_set_drvdata(pdev, core_data);
	core_data->input_dev = vtsdev->idev;
	//input_set_drvdata(core_data->input_dev, core_data);
	core_data->cfg_group_parsed = false;
	core_data->charge_sta = false;
	core_data->power_off_sleep = false;
	goodix_modules_V2.core_data = core_data;
	
	VTI("goodix_ts_probe IN======4");
	r = goodix_ts_power_init(core_data);
	if (r < 0)
		goto err_free_vts;//debug 
	VTI("goodix_ts_probe IN======5");
	/* get GPIO resource */
	r = goodix_ts_gpio_setup(core_data);
	if (r < 0)
		goto err_free_vts;

	r = goodix_ts_power_on_V2(core_data);
	if (r < 0)
		goto err_free_vts;//debug

#ifdef CONFIG_PINCTRL
	// Pinctrl handle is optional.
	r = goodix_ts_pinctrl_init(core_data);
	if (!r) {
		if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->pin_sta_active)) {
			VTE("select pin_sta_active pinctrl success");
			pinctrl_select_state(core_data->pinctrl, core_data->pin_sta_active);
		}

		if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->spi_clk_active)) {
			VTI("select spi_clk_active pinctrl success");
			pinctrl_select_state(core_data->pinctrl, core_data->spi_clk_active);
		}

		if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->spi_mosi_active)) {
			VTI("select spi_mosi_active pinctrl success");
			pinctrl_select_state(core_data->pinctrl, core_data->spi_mosi_active);
		}
	}
#endif

	/*create sysfs files*/
	goodix_ts_sysfs_init_V2(core_data);
	VTI("goodix_ts_probe IN======6");

	goodix_modules_V2.core_data->i2c_addr_buf = kzalloc(GOODIX_I2C_ADDR_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
	if (!goodix_modules_V2.core_data->i2c_addr_buf) {
		VTE("Failed to allocate memory for core data1 !");
		r = -ENOMEM;
		goto err_free_vts;
	}
	goodix_modules_V2.core_data->i2c_get_buf = kzalloc(GOODIX_I2C_GET_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
	if (!goodix_modules_V2.core_data->i2c_get_buf) {
		VTE("Failed to allocate memory for core data2 !");
		r = -ENOMEM;
        goto err_free_addr_buf;
	}
	goodix_modules_V2.core_data->i2c_put_buf = kzalloc(GOODIX_I2C_PUT_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
	if (!goodix_modules_V2.core_data->i2c_put_buf) {
		VTE("Failed to allocate memory for core data3 !");
		r = -ENOMEM;
		goto err_free_get_buf;
	}
	
	VTI("goodix_ts_probe IN======7");

	i2c_read_max_size = ts_device->board_data->i2c_read_max_size;
   	goodix_modules_V2.core_data->i2c_max_buf = kzalloc(i2c_read_max_size, GFP_DMA|GFP_KERNEL);
  	if (!goodix_modules_V2.core_data->i2c_max_buf) {
		VTE("Failed to allocate memory for core data4 !");
		r = -ENOMEM;
		goto err_free_i2c_read;
	}

	msleep(100);
    if(core_data->ts_dev->hw_ops && core_data->ts_dev->hw_ops->read_trans)
		r = core_data->ts_dev->hw_ops->read_trans(core_data->ts_dev, 0x3100, &read_val, 1);
	else 
		r = -1;
	//453c  452c
	if (0 == r) {
		VTI("***read firmware_version SUCCESS");
		VTI("30f0:%02X",read_val);
	} else {
		VTE("***read firmware_version FAILED");
		goto err_i2c;
	}

	
	goodix_modules_V2.core_exit = false;
	complete_all(&goodix_modules_V2.core_comp);

	VTI("goodix_ts_probe OUT");	
	return r;

err_i2c:
	kfree(goodix_modules_V2.core_data->i2c_put_buf);//free 
	goodix_modules_V2.core_data->i2c_put_buf = NULL;

err_free_i2c_read:
	kfree(goodix_modules_V2.core_data->i2c_max_buf);//free 
	goodix_modules_V2.core_data->i2c_max_buf = NULL;
err_free_get_buf:
   kfree(goodix_modules_V2.core_data->i2c_get_buf);//free 
   goodix_modules_V2.core_data->i2c_get_buf = NULL;
  

err_free_addr_buf:
	kfree(goodix_modules_V2.core_data->i2c_addr_buf);//free 
	goodix_modules_V2.core_data->i2c_addr_buf = NULL;	

err_free_vts:
	vts_unregister_driver(vtsdev);
	devm_kfree(&pdev->dev,core_data);
	vts_device_free(vtsdev);
	

	return r;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data =
		platform_get_drvdata(pdev);

	if (goodix_fwu_module_V2.priv_data) {
		kfree(goodix_fwu_module_V2.priv_data);
		goodix_fwu_module_V2.priv_data = NULL;
	}
	if (!goodix_modules_V2.core_data->i2c_put_buf) {
	  kfree(goodix_modules_V2.core_data->i2c_put_buf);
	  goodix_modules_V2.core_data->i2c_put_buf = NULL;
	}
	if (!goodix_modules_V2.core_data->i2c_get_buf) {
	  kfree(goodix_modules_V2.core_data->i2c_get_buf);
	  goodix_modules_V2.core_data->i2c_get_buf = NULL;
	}
	if (!goodix_modules_V2.core_data->i2c_addr_buf) {
	  kfree(goodix_modules_V2.core_data->i2c_addr_buf);
	  goodix_modules_V2.core_data->i2c_addr_buf = NULL;
	}
   if(goodix_modules_V2.core_data->i2c_max_buf){
   	  kfree(goodix_modules_V2.core_data->i2c_max_buf);
	  goodix_modules_V2.core_data->i2c_max_buf = NULL;
   	}
	goodix_ts_power_off_V2(core_data);
	goodix_ts_sysfs_exit(core_data);
    //free vts & core_data
    vts_device_free(core_data->vtsdev);
	devm_kfree(&pdev->dev,core_data);
	
	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
#endif
};
#endif

static const struct platform_device_id ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &dev_pm_ops,
#endif
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};


int goodix_ts_core_V2_init(void)
{
	if (!goodix_modules_V2.initilized) {
		goodix_modules_V2.initilized = true;
		goodix_modules_V2.core_exit = true;
		INIT_LIST_HEAD(&goodix_modules_V2.head);
		mutex_init(&goodix_modules_V2.mutex);
		init_completion(&goodix_modules_V2.core_comp);
	}

	return platform_driver_register(&goodix_ts_driver);
}

void goodix_ts_core_V2_exit(void)
{
	VTI("Core layer exit");
	platform_driver_unregister(&goodix_ts_driver);
	return;
}
