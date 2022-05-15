/* drivers/input/touchscreen/sec_ts_fw.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/vmalloc.h>

#include <linux/uaccess.h>
/*#include <asm/gpio.h>*/

#include "../vts_core.h"


#include "sec_ts.h"

u8 lv1cmd;
u8 *read_lv1_buff;
static int lv1_readsize;
static int lv1_readremain;
static int lv1_readoffset;

static ssize_t sec_ts_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regreadsize_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sec_ts_enter_recovery_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_regread_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_gesture_status_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_cali_reg_show(struct device *dev, 
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_vgk_deadzone_show(struct device *dev, 
		struct device_attribute *attr, char *buf);
static ssize_t sec_ts_vgk_deadzone_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t sec_ts_release_points_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(sec_ts_reg, (S_IWUSR | S_IWGRP), NULL, sec_ts_reg_store);
static DEVICE_ATTR(sec_ts_regreadsize, (S_IWUSR | S_IWGRP), NULL, sec_ts_regreadsize_store);
static DEVICE_ATTR(sec_ts_enter_recovery, (S_IWUSR | S_IWGRP), NULL, sec_ts_enter_recovery_store);
static DEVICE_ATTR(sec_ts_regread, S_IRUGO, sec_ts_regread_show, NULL);
static DEVICE_ATTR(sec_ts_gesture_status, S_IRUGO, sec_ts_gesture_status_show, NULL);
static DEVICE_ATTR(sec_ts_read_cali_reg, S_IRUGO, sec_ts_cali_reg_show, NULL);
static DEVICE_ATTR(sec_ts_vgk_deadzone, (S_IRUGO | S_IWUSR), sec_ts_vgk_deadzone_show, sec_ts_vgk_deadzone_store);
static DEVICE_ATTR(sec_ts_release_points, (S_IWUSR | S_IWGRP), NULL, sec_ts_release_points_store);

static struct attribute *cmd_attributes[] = {
	&dev_attr_sec_ts_reg.attr,
	&dev_attr_sec_ts_regreadsize.attr,
	&dev_attr_sec_ts_enter_recovery.attr,
	&dev_attr_sec_ts_regread.attr,
	&dev_attr_sec_ts_gesture_status.attr,
	&dev_attr_sec_ts_read_cali_reg.attr,
	&dev_attr_sec_ts_vgk_deadzone.attr,
	&dev_attr_sec_ts_release_points.attr,
	NULL,
};

static struct attribute_group cmd_attr_group = {
	.attrs = cmd_attributes,
};

/* for debugging--------------------------------------------------------------------------------------*/
static ssize_t sec_ts_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: Power off state\n", __func__);
		return -EIO;
	}

	mutex_lock(&ts->device_mutex);
	ts->sec_test_flag = 1;
	if (size > 0)
		ts->sec_ts_i2c_write_burst(ts, (u8 *)buf, size);

	VTI("%s: 0x%x, 0x%x, size %d\n", __func__, buf[0], buf[1], (int)size);
	mutex_unlock(&ts->device_mutex);
	return size;
}

static ssize_t sec_ts_regread_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	int length;
	int remain;
	int offset;
	
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: Power off state\n", __func__);
		return -EIO;
	}
	
	mutex_lock(&ts->device_mutex);
	disable_irq(ts->client->irq);
	ts->sec_test_flag = 1;

	read_lv1_buff = kzalloc(lv1_readsize, GFP_KERNEL);
	if (!read_lv1_buff) {
		VTI("%s kzalloc failed\n", __func__);
		goto malloc_err;
	}

	remain = lv1_readsize;
	offset = 0;
	do {
		if (remain >= ts->i2c_burstmax)
			length = ts->i2c_burstmax;
		else
			length = remain;

		if (offset == 0)
			ret = ts->sec_ts_i2c_read(ts, lv1cmd, &read_lv1_buff[offset], length);
		else
			ret = ts->sec_ts_i2c_read_bulk (ts, &read_lv1_buff[offset], length);

		if (ret < 0) {
			VTE("%s: i2c read %x command, remain =%d\n", __func__, lv1cmd, remain);
			goto i2c_err;
		}

		remain -= length;
		offset += length;
	} while (remain > 0);

	VTI("%s: lv1_readsize = %d\n", __func__, lv1_readsize);
	memcpy(buf, read_lv1_buff + lv1_readoffset, lv1_readsize);

i2c_err:
	if (read_lv1_buff) {
		kfree(read_lv1_buff);
		read_lv1_buff = NULL;
	}
malloc_err:
	enable_irq(ts->client->irq);
	mutex_unlock(&ts->device_mutex);
	lv1_readremain = 0;

	return lv1_readsize;
}

static ssize_t sec_ts_gesture_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);
	memcpy(buf, ts->gesture_status, sizeof(ts->gesture_status));
	VTI("%s: GESTURE STATUS %x %x %x %x %x %x\n", __func__,
				ts->gesture_status[0], ts->gesture_status[1], ts->gesture_status[2],
				ts->gesture_status[3], ts->gesture_status[4], ts->gesture_status[5]);
	mutex_unlock(&ts->device_mutex);

	return sizeof(ts->gesture_status);
}

static ssize_t sec_ts_cali_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	u8 data;
	int ret = 0;

	mutex_lock(&ts->device_mutex);
	ts->sec_ts_i2c_read(ts, SEC_TS_VIVO_STATUS_COMMAND, &data, 1);
	VTI("read 1F is %d", data);
	ret = sprintf(buf, "%x ", data);
	mutex_unlock(&ts->device_mutex);

	return ret;
}

static ssize_t sec_ts_regreadsize_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->device_mutex);
	ts->sec_test_flag = 1;
	lv1cmd = buf[0];
	lv1_readsize = ((unsigned int)buf[4] << 24) |
			((unsigned int)buf[3] << 16) | ((unsigned int) buf[2] << 8) | ((unsigned int)buf[1] << 0);
	lv1_readoffset = 0;
	lv1_readremain = 0;
	mutex_unlock(&ts->device_mutex);
	return size;
}

static ssize_t sec_ts_enter_recovery_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int ret;
	unsigned long on;

	ret = kstrtoul(buf, 10, &on);
	if (ret != 0) {
		VTE("%s: failed to read:%d\n",
					__func__, ret);
		return -EINVAL;
	}

	if (on == 1) {
		disable_irq(ts->client->irq);
		gpio_free(pdata->irq_gpio);

		VTI("%s: gpio free\n", __func__);
		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio, GPIOF_OUT_INIT_LOW, "sec,tsp_int");
			VTI("%s: gpio request one\n", __func__);
			if (ret < 0)
				VTE("%s: Unable to request tsp_int [%d]: %d\n", __func__, pdata->irq_gpio, ret);
		} else {
			VTE("%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(100);
		pdata->power(ts, true);
	} else {
		gpio_free(pdata->irq_gpio);

		if (gpio_is_valid(pdata->irq_gpio)) {
			ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN, "sec,tsp_int");
			if (ret) {
				VTE("%s: Unable to request tsp_int [%d]\n", __func__, pdata->irq_gpio);
				return -EINVAL;
			}
		} else {
			VTE("%s: Failed to get irq gpio\n", __func__);
			return -EINVAL;
		}

		pdata->power(ts, false);
		sec_ts_delay(500);
		pdata->power(ts, true);
		sec_ts_delay(500);

		/* AFE Calibration */
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CALIBRATION_AMBIENT, NULL, 0);
		if (ret < 0)
			VTE("%s: fail to write AFE_CAL\n", __func__);

		sec_ts_delay(1000);
		enable_irq(ts->client->irq);
	}

	sec_ts_read_information(ts);

	return size;
}

static inline ssize_t sec_ts_show_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct sec_ts_data *ts = dev_get_drvdata(dev);

	VTE("%s: read only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

static inline ssize_t sec_ts_store_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	//struct sec_ts_data *ts = dev_get_drvdata(dev);

	VTE("%s: write only function, %s\n", __func__, attr->attr.name);
	return -EPERM;
}

static ssize_t sec_ts_vgk_deadzone_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	u8 data[2];
	int ret = 0;

	mutex_lock(&ts->device_mutex);
	ts->sec_ts_i2c_read(ts, SEC_TS_CMD_VGK_DEADZONE, data, 2);
	VTI("read virtual gamekey deadzone is %d, %d", data[0], data[1]);
	ret = sprintf(buf, "%x %x", data[0], data[1]);
	mutex_unlock(&ts->device_mutex);

	return ret;
}
static ssize_t sec_ts_vgk_deadzone_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: Power off state\n", __func__);
		return -EIO;
	}

	if (kstrtou8(buf, 10, &ts->vgk_deadzone[0])) {
		VTI("string to int error! buf: %s\n", buf);
		return -EIO;
	}

	ts->vgk_deadzone[1] = 0xff;

	mutex_lock(&ts->device_mutex);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_VGK_DEADZONE, ts->vgk_deadzone, 2);
	if (ret < 0)
		VTE("send clear gamekey state error");

	mutex_unlock(&ts->device_mutex);

	VTI("%s: 0x%x, 0x%x", __func__, ts->vgk_deadzone[0], ts->vgk_deadzone[1]);
	
	return size;
}

int sec_ts_raw_device_init(struct sec_ts_data *ts)
{
	int ret;

	ts->sec_class = class_create(THIS_MODULE, "sec_vivo");
	ret = IS_ERR_OR_NULL(ts->sec_class);
	if (ret) {
		VTE("%s: fail - class_create\n", __func__);
		return ret;
	}

	ts->dev = device_create(ts->sec_class, NULL, 0, ts, "sec_ts_vivo");

	ret = IS_ERR(ts->dev);
	if (ret) {
		VTE("%s: fail - device_create\n", __func__);
		return ret;
	}

	ret = sysfs_create_group(&ts->dev->kobj, &cmd_attr_group);
	if (ret < 0) {
		VTE("%s: fail - sysfs_create_group\n", __func__);
		goto err_sysfs;
	}

	return ret;
err_sysfs:
	VTE("%s: fail\n", __func__);
	return ret;
}

static ssize_t sec_ts_release_points_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);
	struct vts_device *vtsdev = ts->vtsdev;
	VTI("sec_ts_release_points");
	vts_report_release(vtsdev);
	return size;
}