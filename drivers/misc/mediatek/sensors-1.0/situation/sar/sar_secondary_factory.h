/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __SAR_FACTORY_H__
#define __SAR_FACTORY_H__

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/vsen_common.h>

#include <hwmsen_helper.h>
#include <hwmsensor.h>
#include <sensors_io.h>


struct sar_secondary_factory_fops {
	int (*enable_sensor)(bool enabledisable,
			int64_t sample_periods_ms);
	int (*get_data)(int32_t sensor_data[3]);
	int (*enable_calibration)(void);
	int (*get_cali)(int32_t data[3]);
	int (*do_vsen_commands)(uint8_t sensorType, int32_t *args, int args_len);
	int (*do_self_test)(void);
};

struct sar_secondary_factory_public {
	uint32_t gain;
	uint32_t sensitivity;
	struct sar_secondary_factory_fops *fops;
};
int sar_secondary_factory_device_register(struct sar_secondary_factory_public *dev);
int sar_secondary_factory_device_deregister(struct sar_secondary_factory_public *dev);
#endif
