/* drivers/input/touchscreen/sec_ts_fn.c
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
#include <asm/unaligned.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include "sec_ts.h"
#include "../vts_core.h"

/*
void bbk_slsi_init_callback(struct sec_ts_data *ts)
{
	g_ts_data = ts;
	
	ts->get_rawordiff_data = bbk_slsi_get_rawordiff_data;
	ts->fw_update	= bbk_slsi_fw_update;
	ts->set_auto_idle = set_auto_idle;
	ts->readUdd	= bbk_slsi_readUdd;
	ts->writeUdd	= bbk_slsi_writeUdd;
	ts->mode_change	= bbk_slsi_mode_change; 
	ts->get_fw_version = bbk_slsi_get_fw_version;
	ts->gesture_point_get	= bbk_slsi_gesture_point_get;
	ts->set_charger_bit	= bbk_slsi_set_charger_bit;
	ts->read_charger_bit	= bbk_slsi_read_charger_bit;
	ts->get_module_id	= bbk_slsi_get_module_id;
	ts->sensor_test	= bbk_slsi_sensor_test;
	ts->setEdgeRestainSwitch	= setEdgeRestainSwitch;	
}*/

/*
int bbk_slsi_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	struct sec_ts_data *ts = g_ts_data;
	unsigned int readbytes = 0xFF;
	int *pRead = NULL;
	int ret = 0;
	int i = 0;
	//int j = 0;

	VTI("%s: enter, which = %d", __func__, which);

	VTI("ts->rx_count=%d, ts->tx_count=%d", ts->rx_count, ts->tx_count);
	//set data length, allocation buffer memory 
	readbytes = ts->rx_count * ts->tx_count;

	pRead = data;

	switch (type) {
	case TYPE_RAW_DATA:
		//read rawdata
		ret = run_rawdata_read_all(ts);
		if (ret < 0)
			VTE("run_rawdata_read_all fail");
		break;
	case TYPE_SIGNAL_DATA:
		//read delta/diff data
		ret = run_delta_read_all(ts);
		if (ret < 0)
			VTE("run_signaldata_read_all fail");
		break;
	case TYPE_AMBIENT_BASELINE:
		//read ambient data
		ret = run_ambient_read_all(ts);
		if (ret < 0)
			VTE("run_ambient_data_read_all fail");
		break;
	default:
		VTI("%s: Invalid data type", __func__);
		ret = -EINVAL;
		goto out;
	}

	if( ret == 0){
		for (i = 0; i<ts->rx_count*ts->tx_count;  i++) {
			//for (j = 0; j < ts->rx_count; j++)
				//pRead[(i * ts->tx_count) + j] = (int)ts->pFrame[(i* ts->tx_count) +j];
				pRead[i] = (int)ts->pFrame[i];

		}
	}

	//data = pRead;
out:
	//kfree(pRead);
	return ret;
}
*/

/*
int bbk_slsi_fw_update(struct vts_device *vtsdev,const struct firmware *firmware)
{
	struct sec_ts_data *ts = g_ts_data;
	int error = 0;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("[ERROR] Touch is stopped");
		return -ERROR;
	}

	if (ts->client->irq)
		disable_irq(ts->client->irq);

	//firmware_update(driver data, fw data, fw size, bootup, calib, retry)
	if (sec_ts_firmware_update(ts, firmware->data, firmware->size, 0, 0, 3) < 0) {
		VTI("update fw fail");
		error = -1;
	} else {
		VTI("update fw success");
		error = 0;
	}

	sec_ts_save_version_of_ic(ts);

	if (ts->client->irq)
		enable_irq(ts->client->irq);

	return error;
}
*/
int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len);
extern int bbk_xxsw_reset(struct sec_ts_data *ts);
int bbk_xxsw_reset(struct sec_ts_data *ts)
{
		int ret;

	VTI("sw reset");
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		VTE("%s: write fail, sw_reset\n", __func__);
	}

	sec_ts_delay(300);

	return ret;
}
extern int sec_ts_set_callmode(struct vts_device *vtsdev, int state);
/*
int bbk_slsi_mode_change(struct vts_device *vtsdev, int which)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	unsigned char read_result[4] = {0};

	if (which == last_state) {
		VTE("%s: No mode change (last_state:%d, which:%d)",
				__func__, last_state, which);
		return -1;
	}

	switch(which) {
		case TOUCHSCREEN_NORMAL:
			VTI("change to normal mode");
			
			if (last_state == TOUCHSCREEN_GESTURE) {
				ret = sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
				if (ret < 0)
					VTE("Failed to start device");
			} else { //if (last_state == TOUCHSCREEN_SLEEP) 
				ret = sec_ts_start_device(ts);
				if (ret < 0)
					VTE("Failed to start device");
			}
			if (vivo_ts_prox_ear_detect_get()) {
				sec_ts_set_callmode(1);
			}
			VTI("change to normal mode end");
			ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, read_result, 4);
			if (ret < 0) {
				VTI("read data err");
			}
			VTI("read status:0x%x, 0x%x, 0x%x, 0x%x", read_result[0], read_result[1], read_result[2], read_result[3]);

			break;
		case TOUCHSCREEN_GESTURE:
			VTI("change to gesture mode");

			if(last_state == TOUCHSCREEN_SLEEP) {
				ret = sec_ts_start_device(ts);
					if (ret < 0)
						VTE("Failed to start device");
			}
			if (vivo_ts_prox_ear_detect_get()) {
				sec_ts_set_callmode(0);
			}
			//ts->lowpower_mode = SEC_TS_MODE_LSI_AOD;
			ts->lowpower_mode = SEC_TS_MODE_LOWPOWER_FLAG;     //To turn on all the wakeup gesture
			ret = sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
			VTI("change to gesture mode end");
			break;
		case TOUCHSCREEN_SLEEP:
			VTI("change to sleep mode");
			if (vivo_ts_prox_ear_detect_get()) {
				sec_ts_set_callmode(0);
			}
			ret = sec_ts_stop_device(ts);

			VTI("change to sleep mode end");
			break;

		default : break;
	}

	last_state = which;
	return ret;
}
*/

/*
int bbk_slsi_get_fw_version(struct vts_device *vtsdev, int which)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;

	//read from chip
	ret = sec_ts_save_version_of_ic(ts);
	if (ret < 0) {
		VTE("fail to save ic version");
		return -EIO;
	}

	switch (which) {
	case FW_VERSION:
		ret = (ts->plat_data->img_version_of_ic[2]<<8)|ts->plat_data->img_version_of_ic[3];
		break;
	case CONFIG_VERSION:
		ret = (ts->plat_data->config_version_of_ic[2]<<8)|ts->plat_data->config_version_of_ic[3];
		break;
	}
	return ret;
}
*/

int bbk_slsi_read_charger_bit(void)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	char data = 0;

	ret = sec_ts_i2c_read(ts, SET_TS_CMD_SET_NOISE_MODE, &data, 1);
	if (ret < 0) {
		VTE("%s: failed to read charger stauts(%d)\n",
					__func__, ret);
		return ret;
	}

	return (int)data;
}

/*
int bbk_slsi_readUdd(unsigned char *udd)
{
	struct sec_ts_data *ts = g_ts_data;
	unsigned char buf[32] = {0};
	int ret = 0;
	ret = get_user_nvm_data(ts, buf, 15);
	if (ret < 0)
		VTE("%s: read user data failed\n", __func__);
	else
		memcpy(udd, buf, 15);

	return ret;
}
int bbk_slsi_writeUdd(unsigned char *udd)
{
	struct sec_ts_data *ts = g_ts_data;

	unsigned char buf[32] = {0};

	memcpy(buf, udd, 15);

	return set_user_nvm_data(ts, buf, 15);
}
*/

/*
int bbk_slsi_set_auto_idle(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = -1;

	if (state) {
		ret = sec_ts_release_tmode(ts);
		if (ret < 0)
			VTE("%s: failed to enable idle\n", __func__);
	} else {
		ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
		if (ret < 0)
			VTE("%s: failed to disable idle\n", __func__);
	}
	return ret;
}
*/

int bbk_slsi_get_module_id(void)
{
	int ret = 0;
	struct sec_ts_data *ts = g_ts_data;
	/*
	add get modul id
	*/
	u8 tBuff[8];
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, tBuff, 8);
	VTI("%s: Read IMG version: %x %x %x %x\n", __func__,
			tBuff[0], tBuff[1], tBuff[2], tBuff[3]);

	return ret;
}

/*
int bbk_slsi_sensor_test(struct vts_device *vtsdev, char *buf,  int at_sensor_test_cmd, void *pdata, int tmp)
{
	struct sec_ts_data *ts = g_ts_data;
	u8 rBuff[SEC_TS_EVENT_BUFF_SIZE];
	int len = 0;
	int i = 0;
	char selftest_str[15][7] =
	{
		"MIN","MAX","SLPRX","SLPTX","OPNRX",
		"OPNTX","SRTRG","SRTTG","SRTRR","SRTTT",
		"SRTTR", "pass","fail","Pass","Failed"
	};

	VTI("enter");

	
	//add self test
	
	run_trx_short_test(ts, rBuff);
	VTI("rBuff:%d %d %d %d %d %d %d %d", rBuff[0], rBuff[1], rBuff[2], rBuff[3], rBuff[4], rBuff[5], rBuff[6], rBuff[7]);

	//All test result
	if( (rBuff[2] == 0) && ((rBuff[3]&0x7) == 0))
		len = sprintf(buf,"%s\n",selftest_str[13]);	//all test success
	else
		len = sprintf(buf,"%s\n",selftest_str[14]);	//one or all test failed

	for( i=0;i<8; i++){
		len += sprintf(&buf[len],"%s test:",selftest_str[i]);
		if( (rBuff[2] >> i) & 0x1)
			len += sprintf(&buf[len],"%s\n",selftest_str[12]);
		else
			len += sprintf(&buf[len],"%s\n",selftest_str[11]);
	}

	for( i=0;i<3; i++){
		len += sprintf(&buf[len],"%s test:",selftest_str[i+8]);
		if( (rBuff[3] >> i) & 0x1)
			len += sprintf(&buf[len],"%s\n",selftest_str[12]);
		else
			len += sprintf(&buf[len],"%s\n",selftest_str[11]);
	}

	return len;
}
*/

/*
int setEdgeRestainSwitch(struct vts_device *vtsdev, int on)
{
	int ret = 0;
	
	//add edge restian command
	
	return 0;
	ret = dead_zone_enable(on);
	return ret;
}
*/

int bbk_slsi_gesture_point_get(u16 *data)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	int i = 0;
	u8 buff[20] = { 0 };

	VTI("gesture point num is %d", ts->read_gesture_point_num);
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, buff, 20);
	if (ret < 0) {
		VTE("i2c read gesture coord failed\n");
		return ret;
	}

	switch(buff[0]) {
		case GESTURE_DOUBLE_CLICK:
		case GESTURE_DOWN_SLIDE:
		case GESTURE_LEFT_SLIDE:
		case GESTURE_RIGHT_SLIDE:
		case GESTURE_M:
			return 0;
		case GESTURE_UP_SLIDE:
			ret = 2;
			break;
		case GESTURE_W:
			ret = 5;
			break;
		default:
			ret = 6;
			break;
	}

	for (i = 0; i < 6; i++) {
		if (i >= ret) {
			data[i * 2] = 0;
			data[i * 2 + 1] = 0;
			continue;
		}
		data[i * 2] = (buff[3 * i + 1] << 4) | (buff[3 * i + 3] >> 4);
		data[i * 2 + 1] = (buff[3 * i + 2] << 4) | (buff[3 * i + 3] & 0x0f);
	}
	if (buff[0] == GESTURE_O)  // orientation info for 'O'
		data[12] = buff[19];

	return ret;
}