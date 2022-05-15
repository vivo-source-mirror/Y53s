/* sarhub_secondary motion sensor driver
 *
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

#define pr_fmt(fmt) "[sarhub_secondary] " fmt

#include <hwmsensor.h>
#include "sarhub.h"
#include <situation.h>
#include <SCP_sensorHub.h>
#include <linux/notifier.h>
#include "scp_helper.h"
#include "scp_helper.h"
#include "sar_secondary_factory.h"

static struct situation_init_info sarhub_secondary_init_info;
static DEFINE_SPINLOCK(calibration_lock);
struct sarhub_secondary_ipi_data {
	bool factory_enable;
	bool android_enable;
	atomic_t selftest_status;

	int32_t cali_data[3];
	int8_t cali_status;
	struct completion calibration_done;
	struct completion selftest_done;
};
static struct sarhub_secondary_ipi_data *obj_ipi_data;


static int sar_secondary_factory_enable_sensor(bool enabledisable,
					 int64_t sample_periods_ms)
{
	int err = 0;
	struct sarhub_secondary_ipi_data *obj = obj_ipi_data;
	if (obj->android_enable) {
		pr_err("sar_factory_enable_sensor android enabled, don't enable or disable!\n");
		return 0;
	}

	if (enabledisable == true)
		WRITE_ONCE(obj->factory_enable, true);
	else
		WRITE_ONCE(obj->factory_enable, false);
	if (enabledisable == true) {
		err = sensor_set_delay_to_hub(ID_SAR_SECONDARY,
					      sample_periods_ms);
		if (err) {
			pr_err("sensor_set_delay_to_hub failed!\n");
			return -1;
		}
	}
	err = sensor_enable_to_hub(ID_SAR_SECONDARY, enabledisable);
	if (err) {
		pr_err("sensor_enable_to_hub failed!\n");
		return -1;
	}
	return 0;
}

static int sar_secondary_factory_get_data(int32_t sensor_data[3])
{
	int err = 0;
	struct data_unit_t data;

	err = sensor_get_data_from_hub(ID_SAR_SECONDARY, &data);
	if (err < 0) {
		pr_err_ratelimited("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	sensor_data[0] = data.sar_event.data[0];
	sensor_data[1] = data.sar_event.data[1];
	sensor_data[2] = data.sar_event.data[2];

	return err;
}

static int sar_secondary_factory_enable_calibration(void)
{
	return sensor_calibration_to_hub(ID_SAR_SECONDARY);
}

static int sar_secondary_factory_get_cali(int32_t data[3])
{
	int err = 0;
	struct sarhub_secondary_ipi_data *obj = obj_ipi_data;
	int8_t status = 0;

	err = wait_for_completion_timeout(&obj->calibration_done,
					  msecs_to_jiffies(3000));
	if (!err) {
		pr_err("sar factory get cali fail!\n");
		return -1;
	}
	spin_lock(&calibration_lock);
	data[0] = obj->cali_data[0];
	data[1] = obj->cali_data[1];
	data[2] = obj->cali_data[2];
	status = obj->cali_status;
	spin_unlock(&calibration_lock);
	if (status != 0) {
		pr_debug("sar cali fail!\n");
		return -2;
	}
	return 0;
}
static int sar_secondary_factory_do_vsen_command(uint8_t sensorType, int32_t *args, int args_len)
{
	int err = 0;
	vsen_eng_cmd cmd = args[0];
	err = sensor_set_vsen_cmd_to_hub(sensorType, args, args_len);
	pr_err("%s sensorType=%d,(cmd)0x%x (val1)%d (val2)%d err(%d)\n", __func__, sensorType, cmd, *(args + 1), *(args + 2), err);
	if (err < 0)
		return -1;
	return 0;
}
static int sar_secondary_factory_do_self_test(void)
{
	int ret = 0;
	struct sarhub_secondary_ipi_data *obj = obj_ipi_data;

	atomic_set(&obj->selftest_status, 1);

	ret = sensor_selftest_to_hub(ID_SAR_SECONDARY);
	if (ret < 0)
		return -1;

	ret = wait_for_completion_timeout(&obj->selftest_done,
					  msecs_to_jiffies(3000));
	//pr_err_ratelimited("sar_factory_do_self_test %d ,ret=%d\n",obj->selftest_status, ret);
	if (!ret)
		return -1;
	return atomic_read(&obj->selftest_status);
}
static struct sar_secondary_factory_fops sarhub_secondary_factory_fops = {
	.enable_sensor = sar_secondary_factory_enable_sensor,
	.get_data = sar_secondary_factory_get_data,
	.enable_calibration = sar_secondary_factory_enable_calibration,
	.get_cali = sar_secondary_factory_get_cali,
	.do_vsen_commands = sar_secondary_factory_do_vsen_command,
	.do_self_test = sar_secondary_factory_do_self_test,
};

static struct sar_secondary_factory_public sarhub_secondary_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &sarhub_secondary_factory_fops,
};


static int sar_secondary_get_data(int *probability, int *status)
{
	int err = 0;
	struct data_unit_t data;
	uint64_t time_stamp = 0;

	err = sensor_get_data_from_hub(ID_SAR_SECONDARY, &data);
	if (err < 0) {
		pr_err_ratelimited("sensor_get_data_from_hub fail!!\n");
		return -1;
	}
	time_stamp		= data.time_stamp;
	*probability	= data.sar_event.data[0];
	return 0;
}
static int sar_secondary_open_report_data(int open)
{
	int ret = 0;
	struct sarhub_secondary_ipi_data *obj = obj_ipi_data;
#if defined CONFIG_MTK_SCP_SENSORHUB_V1
	if (open == 1)
		ret = sensor_set_delay_to_hub(ID_SAR_SECONDARY, 120);
#elif defined CONFIG_NANOHUB

#else

#endif
	ret = sensor_enable_to_hub(ID_SAR_SECONDARY, open);
	if (open)
		obj->android_enable = true;
	else
		obj->android_enable = false;
	return ret;
}
static int sar_secondary_batch(int flag,
	int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return sensor_batch_to_hub(ID_SAR_SECONDARY,
		flag, samplingPeriodNs, maxBatchReportLatencyNs);
}

static int sar_secondary_flush(void)
{
	return sensor_flush_to_hub(ID_SAR_SECONDARY);
}

static int sar_secondary_recv_data(struct data_unit_t *event, void *reserved)
{
	struct sarhub_secondary_ipi_data *obj = obj_ipi_data;
	int32_t value[6] = {0};
	int err = 0;

	if (event->flush_action == FLUSH_ACTION)
		err = situation_flush_report(ID_SAR_SECONDARY);
	else if (event->flush_action == DATA_ACTION) {
		value[0] = event->sar_event.data[0];
		value[1] = event->sar_event.data[1];
		value[2] = event->sar_event.data[2];
		err = sar_secondary_data_report_t(value, (int64_t)event->time_stamp);
	} else if (event->flush_action == CALI_ACTION) {
		spin_lock(&calibration_lock);
		obj->cali_data[0] =
			event->sar_event.x_bias;
		obj->cali_data[1] =
			event->sar_event.y_bias;
		obj->cali_data[2] =
			event->sar_event.z_bias;
		obj->cali_status =
			(int8_t)event->sar_event.status;
		spin_unlock(&calibration_lock);
		complete(&obj->calibration_done);
	}  else if (event->flush_action == TEST_ACTION) {
		atomic_set(&obj->selftest_status, event->sar_event.status);
		//pr_err_ratelimited("sar_recv_data:event->sar_event.status=%d\n",event->sar_event.status);
		complete(&obj->selftest_done);
	}
	return err;
}


static int sarhub_secondary_local_init(void)
{
	struct situation_control_path ctl = {0};
	struct situation_data_path data = {0};
	int err = 0;

	struct sarhub_secondary_ipi_data *obj;

	pr_debug("%s\n", __func__);
	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(*obj));
	obj_ipi_data = obj;
	WRITE_ONCE(obj->factory_enable, false);
	WRITE_ONCE(obj->android_enable, false);
	init_completion(&obj->calibration_done);
	init_completion(&obj->selftest_done);

	ctl.open_report_data = sar_secondary_open_report_data;
	ctl.batch = sar_secondary_batch;
	ctl.flush = sar_secondary_flush;
	ctl.is_support_wake_lock = true;
	ctl.is_support_batch = false;
	err = situation_register_control_path(&ctl, ID_SAR_SECONDARY);
	if (err) {
		pr_err("register sar control path err\n");
		goto exit;
	}

	data.get_data = sar_secondary_get_data;
	err = situation_register_data_path(&data, ID_SAR_SECONDARY);
	if (err) {
		pr_err("register sar data path err\n");
		goto exit;
	}


	err = sar_secondary_factory_device_register(&sarhub_secondary_factory_device);
	if (err) {
		pr_err("sar_factory_device register failed\n");
		goto exit;
	}


	err = scp_sensorHub_data_registration(ID_SAR_SECONDARY,
		sar_secondary_recv_data);
	if (err) {
		pr_err("SCP_sensorHub_data_registration fail!!\n");
		goto exit;
	}
	return 0;
exit:
	return -1;
}
static int sarhub_secondary_local_uninit(void)
{
	return 0;
}

static struct situation_init_info sarhub_secondary_init_info = {
	.name = "sarhub_secondary",
	.init = sarhub_secondary_local_init,
	.uninit = sarhub_secondary_local_uninit,
};

static int __init sarhub_secondary_init(void)
{
	situation_driver_add(&sarhub_secondary_init_info, ID_SAR_SECONDARY);
	return 0;
}

static void __exit sarhub_secondary_exit(void)
{
	pr_debug("%s\n", __func__);
}

module_init(sarhub_secondary_init);
module_exit(sarhub_secondary_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SAR_HUB driver");
MODULE_AUTHOR("Jashon.zhang@mediatek.com");