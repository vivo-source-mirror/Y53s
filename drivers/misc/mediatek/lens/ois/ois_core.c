/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ois_core.h"

/*ois log define*/
int log_ois_level;
//module_param(log_ois_level, int, 0644);
EXPORT_SYMBOL_GPL(log_ois_level);

static struct ois *ois_instance[2] = {NULL, NULL};

int ois_interface_create(struct i2c_client *client, struct device *dev, char *ois_name)
{
	int ret = 0;
	struct ois *t_ois_instance = NULL;
	log_ois_level = OIS_LOG_INFO;

	OIS_BUG(!client);
	OIS_BUG(!ois_name);

	t_ois_instance = (struct ois *)(kzalloc(sizeof(struct ois), GFP_KERNEL));
	if (!t_ois_instance) {
		LOG_OIS_ERR("ois create fail");
		goto p_err;
	}
	t_ois_instance->ic_name = ois_name;
	t_ois_instance->client = (struct i2c_client *)(kzalloc(sizeof(struct i2c_client), GFP_KERNEL));
	if (!t_ois_instance->client) {
		LOG_OIS_ERR("ois client buffer alloc fail");
		goto p_err_client;
	}
	memcpy((void *)t_ois_instance->client, (void *)client, sizeof(struct i2c_client));

	t_ois_instance->dev = dev;

	t_ois_instance->flash_info = (struct ois_flash_info *)(kzalloc(sizeof(struct ois_flash_info), GFP_KERNEL));
	if (!t_ois_instance->flash_info) {
		LOG_OIS_ERR("ois_status_check_buf kzalloc failed(%d)\n", ret);
		t_ois_instance->flash_info = NULL;
		goto p_err_instance;
	}

	t_ois_instance->ois_otp = (struct ois_otp_info *)(kzalloc(sizeof(struct ois_otp_info), GFP_KERNEL));
	if (!t_ois_instance->ois_otp) {
		LOG_OIS_ERR("ois otp data kzalloc failed(%d)\n", ret);
		t_ois_instance->ois_otp = NULL;
		goto p_err_flash;
	}
	t_ois_instance->ready_check = 0;
	if ((!strncmp(ois_name, "LC89129", 7)) && (ois_instance[0] == NULL)) {
		ois_instance[0] = t_ois_instance;
		LC89129_get_ops(t_ois_instance);
	} else if ((!strncmp(ois_name, "DW9781C", 7)) && (ois_instance[1] == NULL)) {
		ois_instance[1] = t_ois_instance;
		dw9781c_get_ops(t_ois_instance);
	} else {
		LOG_OIS_ERR("unsupport ic type(%s)", ois_name);
		goto p_err_instance;
	}

	//alloc o+e info buffer
	mutex_init(&t_ois_instance->op_lock);
	t_ois_instance->lens_info_buf = kzalloc(sizeof(struct ois_lens_info_buf), GFP_KERNEL);
	if (!t_ois_instance->lens_info_buf) {
		LOG_OIS_ERR("alloc o2e buffer fail");
		goto p_err;
	};

	LOG_OIS_INF("ois(%s instance=%p client=%p lens_info=%p) create success",
		ois_name, t_ois_instance, t_ois_instance->client, t_ois_instance->lens_info_buf);

	t_ois_instance = NULL;

	return ret;
p_err_flash:
	if (t_ois_instance->flash_info) {
		LOG_OIS_INF("release status buffer %p", t_ois_instance->flash_info);
		kfree(t_ois_instance->flash_info);
		t_ois_instance->flash_info = NULL;
	}
p_err_instance:
	if (t_ois_instance->client) {
		LOG_OIS_INF("release ois client buffer(%p)", t_ois_instance->client);
		kfree(t_ois_instance->client);
		t_ois_instance->client = NULL;
	}

p_err_client:
	if (t_ois_instance) {
		LOG_OIS_INF("client fail, release ois buffer(%p)", t_ois_instance);
		kfree(t_ois_instance);
		t_ois_instance = NULL;
	}
p_err:
	return ret;
}

int ois_interface_destroy(char *ois_name)
{
	int ret = 0;
	struct ois *t_ois_instance = NULL;
	if ((!strncmp(ois_name, "LC89129", 7)) && (ois_instance[0] != NULL)) {
		t_ois_instance = ois_instance[0];
		ois_instance[0] = NULL;
	} else if ((!strncmp(ois_name, "DW9781C", 7)) && (ois_instance[1] != NULL)) {
		t_ois_instance = ois_instance[1];
		ois_instance[1] = NULL;
	} else {
		return ret;
	}

	if (t_ois_instance->lens_info_buf) {
		LOG_OIS_INF("release oe buffer(%p)", t_ois_instance->lens_info_buf);
		kfree(t_ois_instance->lens_info_buf);
		t_ois_instance->lens_info_buf = NULL;
	}

	if (t_ois_instance->client) {
		LOG_OIS_INF("release ois client buffer(%p)", t_ois_instance->client);
		kfree(t_ois_instance->client);
		t_ois_instance->client = NULL;
	}

	if (t_ois_instance->flash_info) {
		LOG_OIS_INF("release status buffer %p", t_ois_instance->flash_info);
		kfree(t_ois_instance->flash_info);
		t_ois_instance->flash_info = NULL;
	}

	if (t_ois_instance->ois_otp) {
		LOG_OIS_INF("release status buffer %p", t_ois_instance->ois_otp);
		kfree(t_ois_instance->ois_otp);
		t_ois_instance->ois_otp = NULL;
	}

	if (t_ois_instance) {
		LOG_OIS_INF("release ois buffer(%p)", t_ois_instance);
		kfree(t_ois_instance);
		t_ois_instance = NULL;
	}
	LOG_OIS_INF("ois destroy success");
	return ret;
}

static long ois_dispatcher(struct ois *ois, unsigned int ioc_command, __user void *buf)
{
	long ret = 0;
	int mode = 0x0000;
	int log_level = OIS_LOG_INFO;

	OIS_BUG(!ois);
	OIS_BUG(!(ois->ops));

	switch (ioc_command) {
	case AFIOC_X_OIS_SETMODE: {
		OIS_BUG(!buf);
		ret = copy_from_user(&mode, buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy mode fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_set_mode, mode);
		break;

	}
	case AFIOC_X_OIS_SETACC: {
		ret = CALL_OISOPS(ois, ois_set_acc, buf);
		break;
	}
	case AFIOC_X_OIS_STATUSCHECK: {
		ret = CALL_OISOPS(ois, ois_status_check, buf);
		break;
	}
	case AFIOC_X_OIS_SETGYROGAIN: {
		ret = CALL_OISOPS(ois, ois_set_gyro_gain, buf);
		break;
	}
	case AFIOC_X_OIS_SETFIXMODE: {
		ret = CALL_OISOPS(ois, ois_set_target, buf);
		break;
	}
	case AFIOC_X_OIS_SETSINEMODE:
	case AFIOC_X_OIS_SETCIRCLEMODE: {
		ret = CALL_OISOPS(ois, ois_set_sinewave, buf);
		break;
	}
	case AFIOC_X_OIS_SETSTROKELIMIT: {
		ret = CALL_OISOPS(ois, ois_set_stroke_limit, buf);
		break;
	}
	case AFIOC_X_OIS_SETPANTILT: {
		ret = CALL_OISOPS(ois, ois_set_pantilt, buf);
		break;
	}
	case AFIOC_X_OIS_GETMODE: {
		ret = CALL_OISOPS(ois, ois_get_mode, buf);
		break;
	}
	case AFIOC_X_OIS_GETINITINFO: {
		ret = CALL_OISOPS(ois, ois_get_init_info, buf);
		break;
	}
	case AFIOC_X_OIS_GETFWVERSION: {
		ret = CALL_OISOPS(ois, ois_get_fw_version, buf);
		break;
	}
	case AFIOC_X_OIS_GETGYROOFFSET: {
		ret = CALL_OISOPS(ois, ois_get_gyro_offset, buf);
		break;
	}
	case AFIOC_X_OIS_GETGYROGAIN: {
		ret = CALL_OISOPS(ois, ois_get_gyro_gain, buf);
		break;
	}
	case AFIOC_X_OIS_GETLENSINFO: {
		ret = CALL_OISOPS(ois, ois_get_lens_info, buf);
		break;
	}
	case AFIOC_X_OIS_GETOTPINFO: {
		ret = CALL_OISOPS(ois, ois_format_otp_data, buf);
		break;
	}
	case AFIOC_X_OIS_OFFSETCAL: {
		ret = CALL_OISOPS(ois, ois_set_offset_calibration);
		break;
	}
	case AFIOC_X_OIS_FWUPDATE: {
		ret = CALL_OISOPS(ois, ois_fw_update, buf);
		break;
	}
	case AFIOC_X_OIS_FLASHSAVE: {
		ret = CALL_OISOPS(ois, ois_flash_save);
		break;
	}
	case AFIOC_X_OIS_INIT: {
		ret = CALL_OISOPS(ois, ois_init);
		ret = CALL_OISOPS(ois, ois_init_vsync_thread);
		break;
	}
	case AFIOC_X_OIS_INIT_SLAVE: {
		ret = CALL_OISOPS(ois, ois_init_slave);
		break;
	}
	case AFIOC_X_OIS_DEINIT: {
		ret = CALL_OISOPS(ois, ois_deinit_vsync_thread);
		ret = CALL_OISOPS(ois, ois_deinit);
		break;
	}
	case AFIOC_X_OIS_STREAMON: {
		ret = CALL_OISOPS(ois, ois_stream_on);
		break;
	}
	case AFIOC_X_OIS_STREAMOFF: {
		ret = CALL_OISOPS(ois, ois_stream_off);
		break;
	}
	case AFIOC_X_OIS_SETTRIPOD: {
		ret = CALL_OISOPS(ois, ois_set_tripod, buf);
		break;
	}
	case AFIOC_X_OIS_SETSMOOTH: {
		ret = CALL_OISOPS(ois, ois_set_smooth, buf);
		break;
	}
	case AFICO_X_OIS_VSYNC: {
		ret = CALL_OISOPS(ois, ois_vsync_signal, buf);
		break;
	}
	case AFIOC_X_OIS_READY_CHECK: {
		ret = CALL_OISOPS(ois, ois_ready_check);
		break;
	}
	case AFICO_X_OIS_LOG_LEVEL: {
		OIS_BUG(!buf);
		ret = copy_from_user(&log_level, buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy log fail(%d)", ret);
			goto p_err;
		}
		ret = CALL_OISOPS(ois, ois_log_control, log_level);
		break;
	}
	case AFICO_X_OIS_GETLOOPGAIN: {
		ret = CALL_OISOPS(ois, ois_get_loopgain, buf);
		break;
	}
	default:
		LOG_OIS_VERB("unsupport command(%08x)", ioc_command);
		ret = -EPERM;
		goto p_err;
	}
	LOG_OIS_VERB("execute ois command(%08x) result(%d)", ioc_command, ret);
p_err:
	return ret;
}

long ois_interface_dispatcher(unsigned int ioc_command, void *buf, char *ois_name)
{
	long ret = 0;
	struct ois *t_ois_instance = NULL;
	if ((!strncmp(ois_name, "LC89129", 7)) && (ois_instance[0] != NULL)) {
		if ((ois_instance[1] != NULL) && (ioc_command == AFIOC_X_OIS_INIT)) {
			LOG_OIS_INF("gyro share LC89129 OIS AFIOC_X_OIS_INIT_SLAVE(DW9781C)");
			//ret = ois_dispatcher(ois_instance[1], AFIOC_X_OIS_DEINIT, buf);
			ret = ois_dispatcher(ois_instance[1], AFIOC_X_OIS_INIT_SLAVE, buf);
		}
		t_ois_instance = ois_instance[0];
	} else if ((!strncmp(ois_name, "DW9781C", 7)) && (ois_instance[1] != NULL)) {
		if ((ois_instance[0] != NULL) && (ioc_command == AFIOC_X_OIS_INIT)) {
			LOG_OIS_INF("gyro share DW9781C OIS AFIOC_X_OIS_INIT_SLAVE(DW9781C)");
			ioc_command = AFIOC_X_OIS_INIT_SLAVE;
		} else if ((ois_instance[0] != NULL) && (ioc_command != AFIOC_X_OIS_DEINIT)) {
			LOG_OIS_INF("gyro share skip DW9781C OIS");
			return ret;
		}
		t_ois_instance = ois_instance[1];
	} else {
		//LOG_OIS_INF("unsupport OIS");
		return ret;
	}
	ret = ois_dispatcher(t_ois_instance, ioc_command, buf);
	return ret;
}

