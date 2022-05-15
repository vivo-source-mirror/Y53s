/*
 *	drivers/sensor_hub_monitor/sensor_hub_monitor.c
 *
 * Copyright (C) 2019 Vivo smartphone, Inc.
 * Author: xiaohua tian <tianxiaohua@vivo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
*/

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
//#include <linux/platform_data/nanohub.h>


#define TAG "SENSOR_HUB_MONITOR"


#define MAX_NAME_SIZE 20
static unsigned int baseminor;
static unsigned int devicecount = 1;
static dev_t devno;
static struct class *sensorhub_class;
static struct device *sensor_device;


#define DEVICE_NAME            "monitor"
#define CLASS_NAME             "sensorhub"

extern void nanohub_register_notifier(struct notifier_block *nb);

static int sensor_hub_callback(struct notifier_block *nb, unsigned long value, void *priv)
{
	char *envp[2];

	envp[1] = NULL;

	if (value == 0) {
		pr_info("[%s]:[%s] value is %lu.\n", TAG, __func__, value);
		envp[0] = "SENSOR_HUB_STATE=on";
		kobject_uevent_env(&sensor_device->kobj, KOBJ_ADD, envp);
	}

	return NOTIFY_OK;
}

static struct notifier_block sensor_hub_notifier_block = {
	.notifier_call = sensor_hub_callback,
	.priority = -INT_MAX,
};

static int __init sensor_hub_monitor_init(void)
{
	int ret = -1;

	pr_info("[%s]:[%s] Enter\n", TAG, __func__);

	ret = alloc_chrdev_region(&devno, baseminor, devicecount, DEVICE_NAME);
	if (ret < 0) {
		pr_err("[%s]:[%s] Failed to alloc char device. ret=%d\n", TAG, __func__, ret);
		goto fail_alloc_chrdev;
	}

	sensorhub_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(sensorhub_class)) {
		ret = PTR_ERR(sensorhub_class);
		pr_err("[%s]:[%s] Failed to create class. ret=%d\n", TAG, __func__, ret);
		goto fail_create_class;
	}

	sensor_device = device_create(sensorhub_class, NULL, devno, NULL, DEVICE_NAME);
	if (IS_ERR(sensor_device)) {
		ret = PTR_ERR(sensor_device);
		pr_err("[%s]:[%s] Failed to create device. ret=%d\n", TAG, __func__, ret);
		goto fail_create_device;
	}

	nanohub_register_notifier(&sensor_hub_notifier_block);

	pr_info("[%s]:[%s] End\n", TAG, __func__);
	return 0;

fail_create_device:
	class_destroy(sensorhub_class);

fail_create_class:
	unregister_chrdev_region(devno, devicecount);

fail_alloc_chrdev:
	return ret;
}

static void __exit sensor_hub_monitor_exit(void)
{
	device_destroy(sensorhub_class, devno);
	class_destroy(sensorhub_class);
	unregister_chrdev_region(devno, devicecount);
}

module_init(sensor_hub_monitor_init);
module_exit(sensor_hub_monitor_exit);

MODULE_AUTHOR("tianxiaohua@vivo.com");
MODULE_DESCRIPTION("sensorhub driver");
MODULE_LICENSE("GPL v2");

