/* amd sensor driver
 * Copyright (C) 2020 vivo Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[amd] " fmt

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>

#include <hwmsensor.h>
#include <sensors_io.h>
#include "situation.h"
#include "amd.h"
#include <hwmsen_helper.h>

#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"

static struct situation_init_info amd_init_info;
static int amd_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_AMD, &data);
	if (err < 0) {
		pr_err("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp		= data.time_stamp;
	*probability	= data.gesture_data_t.probability;
	return 0;
}
static int amd_open_report_data(int open)
{
	int ret = 0;

#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_AMD, 120);
#elif defined CONFIG_NANOHUB

#else

#endif
	pr_debug("%s : type=%d, open=%d\n",
		__func__, ID_AMD, open);
	ret = sensor_enable_to_hub(ID_AMD, open);
	return ret;
}
static int amd_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sensor_batch_to_hub(ID_AMD,
		flag, samplingPeriodNs, maxBatchReportLatencyNs);
}
static int amd_flush(void)
{
	return sensor_flush_to_hub(ID_AMD);
}
static int amd_recv_data(struct data_unit_t *event,
	void *reserved)
{
	if (event->flush_action == FLUSH_ACTION)
		situation_flush_report(ID_AMD);
	else if (event->flush_action == DATA_ACTION) {
		situation_data_report(ID_AMD,
			event->tilt_event.state);
		pr_err("[AMD]:state=%d\n", event->tilt_event.state);
	}
	return 0;
}

static int amd_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	ctl.open_report_data = amd_open_report_data;
	ctl.batch = amd_batch;
	ctl.flush = amd_flush,
	ctl.is_support_wake_lock = true;
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_AMD);
	if (err) {
		pr_err("register amd control path err\n");
		goto exit;
	}

	data.get_data = amd_get_data;
	err = situation_register_data_path(&data, ID_AMD);
	if (err) {
		pr_err("register amd data path err\n");
		goto exit;
	}
	err = scp_sensorHub_data_registration(ID_AMD,
		amd_recv_data);
	if (err) {
		pr_err("SCP_sensorHub_data_registration fail!!\n");
		goto exit_create_attr_failed;
	}
	return 0;
exit:
exit_create_attr_failed:
	return -1;
}
static int amd_local_uninit(void)
{
	return 0;
}

static struct situation_init_info amd_init_info = {
	.name = "amd_hub",
	.init = amd_local_init,
	.uninit = amd_local_uninit,
};

static int __init amd_init(void)
{
	situation_driver_add(&amd_init_info, ID_AMD);
	return 0;
}

static void __exit amd_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(amd_init);
module_exit(amd_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMD driver");
MODULE_AUTHOR("yusheng.zhu@vivo.com");

