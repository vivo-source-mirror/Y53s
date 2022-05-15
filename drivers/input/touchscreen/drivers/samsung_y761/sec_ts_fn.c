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


static int execute_selftest(struct sec_ts_data *ts, bool save_result);

#if 0
static int fw_update(void)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[64] = { 0 };
	int retval = 0;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
				__func__);		
		return -ERROR;
	}
	retval = sec_ts_firmware_update_on_hidden_menu(ts, sec->cmd_param[0]);
	if (retval < 0) {
		snprintf(buff, sizeof(buff), "%s", "NA");
		VTI("%s: failed [%d]\n", __func__, retval);
	} else {
		snprintf(buff, sizeof(buff), "%s", "OK");
		VTI("%s: success [%d]\n", __func__, retval);
	}
	return retval;
}
#endif

int sec_ts_execute_force_calibration(struct sec_ts_data *ts, int cal_mode)
{
	int rc = -1;
	u8 cmd;

	VTI("%s: cal_mode = %d", __func__, cal_mode);

	cmd = SEC_TS_CMD_CALIBRATION_OFFSET_SDC;

	if (ts->sec_ts_i2c_write(ts, cmd, NULL, 0) < 0) {
		VTE("%s: send cal cmd error", __func__);
		return rc;
	}

	sec_ts_delay(1500);

	rc = sec_ts_wait_for_ready(ts, SEC_TS_VENDOR_ACK_OFFSET_CAL_DONE);

	return rc;
}


int sec_ts_fix_tmode(struct sec_ts_data *ts, u8 mode, u8 state)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_OFF};
	u8 onoff_4a[1] = {1};
	u8 tBuff[2] = { mode, state };
	u32 idle_time;
	u8 idle_time_buf[1];
	int gamemode_freq = 0;

	VTI("%s\n", __func__);
	vts_property_get(ts->vtsdev, VTS_PROPERTY_GAMEMODE_FREQ, &gamemode_freq);
	vts_property_get(ts->vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);

	if (gamemode_freq) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_HIGH_FREQ, onoff_4a, 1);
		sec_ts_delay(20);
	}

	if (idle_time == 0) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
		sec_ts_delay(20);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CHG_SYSMODE , tBuff, sizeof(tBuff));
		sec_ts_delay(20);
	} else if (idle_time <= 0xff) {
		idle_time_buf[0] = (u8)idle_time & 0xff;
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_ENTER_GAMEMODE, onoff_4a, 1);
		sec_ts_delay(20);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_IDLE_TIME, idle_time_buf, 1);
		sec_ts_delay(20);
	} else {
		VTE("no idle time, no cmd to send.");
		ret = -1;
	}

	return ret;
}

int sec_ts_release_tmode(struct sec_ts_data *ts)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_ON};
	u8 onoff_4a[1] = {0};
	u32 idle_time;
	u8 idle_time_buf[1] = {1};
	int gamemode_freq = 0;
	
	VTI("%s\n", __func__);
	
	vts_property_get(ts->vtsdev, VTS_PROPERTY_GAMEMODE_FREQ, &gamemode_freq);
	vts_property_get(ts->vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);

	if (gamemode_freq) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_HIGH_FREQ, onoff_4a, 1);
		sec_ts_delay(20);
	}

	if (idle_time == 0) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
		sec_ts_delay(20);
	} else if (idle_time <= 0xff) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_ENTER_GAMEMODE, onoff_4a, 1);
		sec_ts_delay(20);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_IDLE_TIME, idle_time_buf, 1);
		sec_ts_delay(20);
	} else {
		VTE("idle time is error.");
		ret = -1;
	}

	return ret;
}
#if 1
void sec_ts_print_frame(struct sec_ts_data *ts, short *min, short *max)
{
	int i = 0;
	int j = 0;
	unsigned char *pStr = NULL;
	unsigned char pTmp[16] = { 0 };

	VTI("%s\n", __func__);

	pStr = kzalloc(6 * (ts->tx_count + 1), GFP_KERNEL);
	if (pStr == NULL) {
		VTI("SEC_TS pStr kzalloc failed\n");
		return;
	}

	memset(pStr, 0x0, 6 * (ts->tx_count + 1));
	snprintf(pTmp, sizeof(pTmp), "      TX");
	strncat(pStr, pTmp, 6 * ts->tx_count);

	for (i = 0; i < ts->tx_count; i++) {
		snprintf(pTmp, sizeof(pTmp), " %02d ", i);
		strncat(pStr, pTmp, 6 * ts->tx_count);
	}

	VTI("%s\n", pStr);
	memset(pStr, 0x0, 6 * (ts->tx_count + 1));
	snprintf(pTmp, sizeof(pTmp), "     +");
	strncat(pStr, pTmp, 6 * ts->tx_count);

	for (i = 0; i < ts->tx_count; i++) {
		snprintf(pTmp, sizeof(pTmp), "----");
		strncat(pStr, pTmp, 6 * ts->rx_count);
	}

	VTI("%s\n", pStr);

	for (i = 0; i < ts->rx_count; i++) {
		memset(pStr, 0x0, 6 * (ts->tx_count + 1));
		snprintf(pTmp, sizeof(pTmp), "Rx%02d | ", i);
		strncat(pStr, pTmp, 6 * ts->tx_count);

		for (j = 0; j < ts->tx_count; j++) {
			snprintf(pTmp, sizeof(pTmp), " %3d", ts->pFrame[(j * ts->rx_count) + i]);

			if (i > 0) {
				if (ts->pFrame[(j * ts->rx_count) + i] < *min)
					*min = ts->pFrame[(j * ts->rx_count) + i];

				if (ts->pFrame[(j * ts->rx_count) + i] > *max)
					*max = ts->pFrame[(j * ts->rx_count) + i];
			}
			strncat(pStr, pTmp, 6 * ts->rx_count);
		}
		VTI("%s\n", pStr);
	}
	kfree(pStr);
}
#endif
static int sec_ts_read_frame(struct sec_ts_data *ts, u8 type, short *min,
		short *max)
{
	unsigned int readbytes = 0xFF;
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	int j = 0;
	short *temp = NULL;
	//u8 * selfdata;
	
	VTI("%s\n", __func__);

	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count * 2;

	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead) {
		VTE("%s: Read frame kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	/* set OPCODE and data type */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &type, 1);
	if (ret < 0) {
		VTI("Set rawdata type failed\n");
		goto ErrorExit;
	}

	sec_ts_delay(50);

	if (type == TYPE_OFFSET_DATA_SDC) {
		/* excute selftest for real cap offset data, because real cap data is not memory data in normal touch. */
		char para = TO_TOUCH_MODE;

		disable_irq(ts->client->irq);

		execute_selftest(ts, false);

		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			VTI("%s: Set rawdata type failed\n", __func__);
			enable_irq(ts->client->irq);
			goto ErrorRelease;
		}

		enable_irq(ts->client->irq);
	}

	/* read data */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_TOUCH_RAWDATA, pRead, readbytes);
	if (ret < 0) {
		VTE("%s: read rawdata failed!\n", __func__);
		goto ErrorRelease;
	}

	memset(ts->pFrame, 0x00, readbytes);

	for (i = 0; i < readbytes; i += 2)
		ts->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);

	*min = *max = ts->pFrame[0];

#ifdef DEBUG_MSG
	VTI("02X%02X%02X readbytes=%d\n",
			pRead[0], pRead[1], pRead[2], readbytes);
#endif
	sec_ts_print_frame(ts, min, max);
if (0) {
		temp = kzalloc(readbytes, GFP_KERNEL);
		if (!temp) {
			VTE("%s: failed to alloc mem!\n", __func__);
			goto ErrorRelease;

		}

		memcpy(temp, ts->pFrame, ts->tx_count * ts->rx_count * 2);
		memset(ts->pFrame, 0x00, ts->tx_count * ts->rx_count * 2);

		for (i = 0; i < ts->tx_count; i++) {
			for (j = 0; j < ts->rx_count; j++)
				ts->pFrame[(j * ts->tx_count) + i] = temp[(i * ts->rx_count) + j];
		}

		kfree(temp);
	}
	
ErrorRelease:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &mode, 1);
	if (ret < 0)
		VTI("Set rawdata type failed\n");

ErrorExit:
	kfree(pRead);
	//kfree(selfdata);
	
	return ret;
}

static int sec_ts_read_channel(struct sec_ts_data *ts, u8 type, short *min, short *max)
{
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int ii = 0;
	int jj = 0;
	unsigned int data_length = (ts->tx_count + ts->rx_count) * 2;
	u8 w_data;

	VTI("%s\n", __func__);

	pRead = kzalloc(data_length, GFP_KERNEL);
	if (!pRead) {
		VTE("%s: Fail to alloc memory\n", __func__);
		return -ENOMEM;
	}

	/* set OPCODE and data type */
	w_data = type;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &w_data, 1);
	if (ret < 0) {
		VTE("%s: Set rawdata type failed\n", __func__);
		goto out_read_channel;
	}

	sec_ts_delay(50);

	if (type == TYPE_OFFSET_DATA_SDC) {
		char para = TO_TOUCH_MODE;
		disable_irq(ts->client->irq);
		execute_selftest(ts, false);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			VTE("%s: set rawdata type failed!\n", __func__);
			enable_irq(ts->client->irq);
			goto err_read_data;
		}
		enable_irq(ts->client->irq);
	}

	/* read data */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA, pRead, data_length);
	if (ret < 0) {
		VTE("%s: read rawdata failed!\n", __func__);
		goto err_read_data;
	}

	/* clear all pFrame data */
	memset(ts->pFrame, 0x00, data_length);

	for (ii = 0; ii < data_length; ii += 2) {
		ts->pFrame[jj] = ((pRead[ii] << 8) | pRead[ii + 1]);

		if (ii == 0)
			*min = *max = ts->pFrame[jj];

		*min = min(*min, ts->pFrame[jj]);
		*max = max(*max, ts->pFrame[jj]);

		VTI("%s: [%s][%d] %d\n", __func__,
				(jj < ts->tx_count) ? "TX" : "RX", jj, ts->pFrame[jj]);
		jj++;
	}

err_read_data:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &mode, 1);
	if (ret < 0)
		VTE("%s: Set rawdata type failed\n", __func__);

out_read_channel:
	kfree(pRead);

	return ret;
}

static int sec_ts_read_raw_data(struct sec_ts_data *ts,
						struct sec_ts_test_mode *mode)
{
	int ret = 0;
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
				__func__);
		goto error_power_state;
	}

	VTI("%s: %d, %s\n",
			__func__, mode->type, mode->allnode ? "ALL" : "");


	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		VTE("%s: failed to fix tmode\n",
				__func__);
		goto error_test_fail;
	}

	if (mode->frame_channel)
		ret = sec_ts_read_channel(ts, mode->type, &mode->min, &mode->max);
	else
		ret = sec_ts_read_frame(ts, mode->type, &mode->min, &mode->max);
	if (ret < 0) {
		VTE("%s: failed to read frame\n",
				__func__);
		goto error_test_fail;
	}

	ret = sec_ts_release_tmode(ts);
	if (ret < 0) {
		VTE("%s: failed to release tmode\n",
				__func__);
		goto error_test_fail;
	}

//	kfree(buff);

	sec_ts_locked_release_all_finger(ts);

	return ret;

error_test_fail:
error_power_state:
//	kfree(buff);
//error_alloc_mem:
	ret = -EINVAL;
	sec_ts_locked_release_all_finger(ts);

	return ret;
}

#if 0 //not used
static int get_fw_ver_bin(void)
{
	struct sec_ts_data *ts = g_ts_data;

	char buff[16] = { 0 };

	sprintf(buff, "SE%02X%02X%02X",
		ts->plat_data->panel_revision, ts->plat_data->img_version_of_bin[2],
		ts->plat_data->img_version_of_bin[3]);


	VTI("%s: %s\n", __func__, buff);
	return 0;
}
#endif //not used

int get_fw_ver_ic(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[16] = { 0 };
	int ret;
	u8 fw_ver[4];

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
				__func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		return -EIO;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, fw_ver, 4);
	if (ret < 0) {
		VTI("%s: firmware version read error\n ", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		return -EIO;
	}

	snprintf(buff, sizeof(buff), "SE%02X%02X%02X",
			ts->plat_data->panel_revision, fw_ver[2], fw_ver[3]);

	VTI("%s: %s\n", __func__, buff);

	ret = (fw_ver[3]<<8)|fw_ver[2];
	return ret;
}

int get_config_ver(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[20] = { 0 };
	int ret;

	sprintf(buff, "para:%02X%02X",
		ts->plat_data->config_version_of_ic[2], ts->plat_data->config_version_of_ic[3]);

	VTI("%s: %s\n", __func__, buff);
	
	ret = (ts->plat_data->config_version_of_ic[3]<<8)|ts->plat_data->config_version_of_ic[2];
	return ret;
}

int run_reference_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SET;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_rawcap_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_rawdata_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_RAW_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_delta_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;
	
	VTI("enter in run_delta_read_all!!!!");
	
	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_ambient_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_AMBIENT_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

/* self reference : send TX power in TX channel, receive in TX channel */
int run_self_reference_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SET;
	mode.frame_channel= TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_self_rawcap_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel= TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_self_rawdata_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_RAW_DATA;
	mode.frame_channel= TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_self_delta_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.frame_channel= TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}

int run_self_ambient_read_all(void *device_data)
{
	struct sec_ts_data *ts = device_data;
	struct sec_ts_test_mode mode;

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_AMBIENT_DATA;
	mode.frame_channel= TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	return sec_ts_read_raw_data(ts, &mode);
}
#if 1
//#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
int sec_ts_run_rawdata_all(struct sec_ts_data *ts)
{
	short min, max;
	int ret;

	sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
	ret = sec_ts_read_frame(ts, TYPE_OFFSET_DATA_SET, &min, &max);
	if (ret < 0){
		VTE("%s, 19,Offset error ## ret:%d\n", __func__, ret);
		goto err_exit;
	}else{
		VTE("%s, 19,Offset Max/Min %d,%d ##\n", __func__, max, min);
		sec_ts_release_tmode(ts);
	}

	sec_ts_delay(20);
	
	sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
	ret = sec_ts_read_frame(ts, TYPE_RAW_DATA, &min, &max);
	if (ret < 0){
		VTE("%s, 0,Ambient error ## ret:%d\n", __func__, ret);
		goto err_exit;
	}else{
		VTE("%s, 0,Ambient Max/Min %d,%d ##\n", __func__, max, min);
		sec_ts_release_tmode(ts);
	}
err_exit:
	return ret;
}
#endif
int set_user_nvm_data(struct sec_ts_data *ts, u8 * data)
{
	/* data length is fixed as 15bytes */
	char buff[15 + 2] = { 0 };
	unsigned int length = 15;
	int ret = 0;

	VTI("%s\n", __func__);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("%s: [ERROR] Touch is stopped\n",
		__func__);
		snprintf(buff, sizeof(buff), "%s", "NVMPowerOff");		
		return -EIO;
	}


	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte, value is  1 - 1 = 0)
	 * buff[2] : write data
	 */
	buff[0] = 0;
	buff[1] = length - 1;
	memcpy(&buff[2], data, sizeof(u8) * length);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_NVM, buff, 2 + length);
	if (ret < 0)
		VTE("%s nvm write failed. ret: %d\n", __func__, ret);	

	sec_ts_delay(20);

	return ret;
}

int get_user_nvm_data(struct sec_ts_data *ts, u8* data)
{
	int ret = 0;
	const unsigned int length = 15;
	struct vts_device *vtsdev = ts->vtsdev;

	VTI("%s, offset:%u\n", __func__, 0);
	vts_report_release(vtsdev);
	/* SENSE OFF -> CELAR EVENT STACK -> READ NV -> SENSE ON */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		VTE("%s: fail to write Sense_off\n", __func__);
		goto out_nvm;
	}

	VTI("%s: SENSE OFF\n", __func__);

	sec_ts_delay(100);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		VTE("%s: i2c write clear event failed\n", __func__);
		goto out_nvm;
	}

	VTI("%s: CLEAR EVENT STACK\n", __func__);

	sec_ts_delay(100);
	
	/* send NV data using command
	 * Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte, value is  1 - 1 = 0)
	 */
	memset(data, 0x00, length);
	data[0] = 0;
	data[1] = length - 1;
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_NVM, data, 2);
	if (ret < 0) {
		VTE("%s nvm send command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

	sec_ts_delay(20);

	/* read NV data
	 * Use TSP NV area : in this model, use only one byte
	 */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_NVM, data, length);
	if (ret < 0) {
		VTE("%s nvm send command failed. ret: %d\n", __func__, ret);
		goto out_nvm;
	}

	//VTI("%s: data:%X\n", __func__, buff[0]);
	
out_nvm:
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		VTE("%s: fail to write Sense_on\n", __func__);
	else
		VTI("%s: SENSE ON\n", __func__);
	
	return ret;
}

int dead_zone_enable(struct vts_device *vtsdev, u8 orient)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	//char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret = -1;
	char data = 0;

	if (orient < 0 || orient > 2) {
		VTE("%s NG: wrong orientation\n", __func__);
		return ret;
	}

	data = orient;
	VTI("%s: orient = %d (fw value)", __func__, orient);
		
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_GRIP_ORIENTATION, &data, 1);
	if (ret < 0)
		VTE("%s failed to set deadzone, ret: %d\n", __func__, ret);

	return ret;
};

#if 0 //not used
u8 *set_sft_result(u8 result,u8* pSrc, u8* pDes)
{
	char str[2][5] = { "PASS ", "FAIL " };

	strlcpy(pSrc, pDes, strlen(pDes));
	pSrc += strlen(pDes);
	if (result) {
		strlcpy(pSrc, str[0], 6);
		pSrc += strlen(str[0]);
	} else {
		strlcpy(pSrc, str[1], 6);
		pSrc += strlen(str[1]);
	}
	
	return pSrc;
}
#endif

static void sec_ts_swap(u8 *a, u8 *b)
{
	u8 temp = *a;
	*a = *b;
	*b = temp;
}

static void rearrange_sft_result(u8 *data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		sec_ts_swap(&data[i], &data[i+3]);
		sec_ts_swap(&data[i+1], &data[i+2]);
	}
}

static int print_selftest_result(struct sec_ts_data *ts)
{
	int rc;
	//u8 tpara = 0x23;
	u8 *rBuff;
	u16 *rBuff16;
	int i = 0;
	int j = 0;
	int result_size = SEC_TS_SELFTEST_REPORT_SIZE + ts->tx_count * ts->rx_count * 2;

	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff) {
		VTI("allocation failed!");
		return -ENOMEM;
	}

	rc = ts->sec_ts_i2c_read(ts, SEC_TS_READ_SELFTEST_RESULT, rBuff, result_size);
	if (rc < 0) {
		VTI("Selftest execution time out!");
		goto err_exit;
	}
	rearrange_sft_result(rBuff, result_size);

	for (i = 0; i < 80; i += 4) {
		if (i % 8 == 0)
			VTI("\n");
		if (i % 4 == 0)
			VTI("VIVO_TS sec_ts : ");

		if (i / 4 == 0)
			VTI("SIG");
		else if (i / 4 == 1)
			VTI("VER");
		else if (i / 4 == 2)
			VTI("SIZ");
		else if (i / 4 == 3)
			VTI("CRC");
		else if (i / 4 == 4)
			VTI("RES");
		else if (i / 4 == 5)
			VTI("COU");
		else if (i / 4 == 6)
			VTI("PAS");
		else if (i / 4 == 7)
			VTI("FAI");
		else if (i / 4 == 8)
			VTI("CHA");
		else if (i / 4 == 9)
			VTI("AMB");
		else if (i / 4 == 10)
			VTI("RXS");
		else if (i / 4 == 11)
			VTI("TXS");
		else if (i / 4 == 12)
			VTI("RXO");
		else if (i / 4 == 13)
			VTI("TXO");
		else if (i / 4 == 14)
			VTI("RXG");
		else if (i / 4 == 15)
			VTI("TXG");
		else if (i / 4 == 16)
			VTI("RXR");
		else if (i / 4 == 17)
			VTI("TXT");
		else if (i / 4 == 18)
			VTI("RXT");
		else if (i / 4 == 19)
			VTI("TXR");

		//this 4byte's each bit represents each channel
		VTI(" %2X, %2X, %2X, %2X  ", rBuff[i], rBuff[i + 1], rBuff[i + 2], rBuff[i + 3]);

		if (i / 4 == 4) {
			if ((rBuff[i + 3] & 0x30) != 0)// RX, RX open check.
				rc = 0;
			else
				rc = 1;
		}
	}

	rBuff16 = (u16 *)&rBuff[80];
	VTI("VIVO_TS Ambient Data\n");
	for (i = 0; i < ts->tx_count; i++) {
		VTI("TX%2x :", i);
		for (j = 0; j < ts->rx_count; j++)
			VTI("%6d ", rBuff16[i*ts->rx_count + j]);
		VTI("\n");
	}

err_exit:
	kfree(rBuff);
	return rc;
}

static int execute_selftest(struct sec_ts_data *ts, bool save_result)
{
	int rc;
	u8 tpara;

	if (save_result)
		tpara = 0x23;
	else
		tpara = 0xA3;
	//u8 *rBuff;
	//u8 *rBuffPtr;
	//int i;
	//int result_size;
	//u8 tBuff[SEC_TS_EVENT_BUFF_SIZE];

	/*char selftest_str[15][5] =
	{
		"MIN","MAX","SLPRX","SLPTX","OPNRX",
		"OPNTX","SRTRG","SRTTG","SRTRR","SRTTT",
		"SRTTR", "pass","fail","Pass","Failed"
	};*/

	/*result_size = sizeof(char)*11*10+6;

	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff) {
		VTE("%s: allocation failed!\n", __func__);
		return -ENOMEM;
	}*/

	VTI("Self test start!");
	rc = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SELFTEST, &tpara, 1);
	if (rc < 0) {
		VTI("Send selftest cmd failed!");
		goto err_exit;
	}

	sec_ts_delay(1000);	//350	

	rc = sec_ts_wait_for_ready(ts, SEC_TS_VENDOR_ACK_SELF_TEST_DONE);
	if (rc < 0) {
		VTE("%s: self test execution timeout!\n", __func__);
		goto err_exit;
	}

	VTI("Self test done!");

	print_selftest_result(ts);
/*	rBuffPtr = rBuff ;	

	if( (tBuff[2] != 0) || (tBuff[3] != 0)){  
		strcpy(rBuffPtr,selftest_str[14]);
		rBuffPtr += strlen(selftest_str[14]);
	}
	else{
		strcpy(rBuffPtr,selftest_str[13]);
		rBuffPtr += strlen(selftest_str[13]);
	}

	for( i=0;i<8; i++){		
		rBuffPtr = set_sft_result((tBuff[2]>>i)&0x1, rBuffPtr,selftest_str[i]);
	}

	for( i=0;i<3; i++){
		rBuffPtr = set_sft_result((tBuff[3]>>i)&0x1, rBuffPtr,selftest_str[i+8]);
	}*/

/*	rc = ts->sec_ts_i2c_read(ts, SEC_TS_READ_SELFTEST_RESULT, rBuff, result_size);
	if (rc < 0) {
		VTE("%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}*/
	return rc;
err_exit:
	//kfree(rBuff);
	return rc;
}

void run_trx_short_test(void *device_data, u8* data)
{
	struct sec_ts_data *ts = device_data;
	char buff[SEC_CMD_STR_LEN] = {0};
	int rc;
	char para = TO_TOUCH_MODE;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("Touch is stopped!");
		return;
	}

	disable_irq(ts->client->irq);

	rc = execute_selftest(ts, true);
	if (rc) {

		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		enable_irq(ts->client->irq);
		VTI("%s: execute selftest fail\n", __func__);
		return;
	}

	ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	enable_irq(ts->client->irq);

	snprintf(buff, sizeof(buff), "%s", "NG");
	VTI("%s: %s", __func__, buff);
	return;

}

#if 0
void set_grip_data_to_ic(struct sec_ts_data *ts, u8 flag)
{
	u8 data[8] = { 0 };

	VTI("%s: flag: %02X (clr,lan,nor,edg,han)\n", __func__, flag);

	if (flag & G_SET_EDGE_HANDLER) {
		if (ts->grip_edgehandler_direction == 0) {
			data[0] = 0x0;
			data[1] = 0x0;
			data[2] = 0x0;
			data[3] = 0x0;
		} else {
			data[0] = (ts->grip_edgehandler_start_y >> 4) & 0xFF;
			data[1] = (ts->grip_edgehandler_start_y << 4 & 0xF0) | ((ts->grip_edgehandler_end_y >> 8) & 0xF);
			data[2] = ts->grip_edgehandler_end_y & 0xFF;
			data[3] = ts->grip_edgehandler_direction & 0x3;
		}
		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_EDGE_HANDLER, data, 4);
		VTI("%s: 0x%02X %02X,%02X,%02X,%02X\n",
				__func__, SEC_TS_CMD_EDGE_HANDLER, data[0], data[1], data[2], data[3]);
	}

	if (flag & G_SET_EDGE_ZONE) {
		data[0] = (ts->grip_edge_range >> 8) & 0xFF;
		data[1] = ts->grip_edge_range  & 0xFF;
		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_EDGE_AREA, data, 2);
		VTI("%s: 0x%02X %02X,%02X\n",
				__func__, SEC_TS_CMD_EDGE_AREA, data[0], data[1]);
	}

	if (flag & G_SET_NORMAL_MODE) {
		data[0] = ts->grip_deadzone_up_x & 0xFF;
		data[1] = ts->grip_deadzone_dn_x & 0xFF;
		data[2] = (ts->grip_deadzone_y >> 8) & 0xFF;
		data[3] = ts->grip_deadzone_y & 0xFF;
		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_DEAD_ZONE, data, 4);
		VTI("%s: 0x%02X %02X,%02X,%02X,%02X\n",
				__func__, SEC_TS_CMD_DEAD_ZONE, data[0], data[1], data[2], data[3]);
	}

	if (flag & G_SET_LANDSCAPE_MODE) {
		data[0] = ts->grip_landscape_mode & 0x1;
		data[1] = (ts->grip_landscape_edge >> 4) & 0xFF;
		data[2] = (ts->grip_landscape_edge << 4 & 0xF0) | ((ts->grip_landscape_deadzone >> 8) & 0xF);
		data[3] = ts->grip_landscape_deadzone & 0xFF;
		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_LANDSCAPE_MODE, data, 4);
		VTI("%s: 0x%02X %02X,%02X,%02X,%02X\n",
				__func__, SEC_TS_CMD_LANDSCAPE_MODE, data[0], data[1], data[2], data[3]);
	}

	if (flag & G_CLR_LANDSCAPE_MODE) {
		data[0] = ts->grip_landscape_mode;
		ts->sec_ts_i2c_write(ts, SEC_TS_CMD_LANDSCAPE_MODE, data, 1);
		VTI("%s: 0x%02X %02X\n",
				__func__, SEC_TS_CMD_LANDSCAPE_MODE, data[0]);
	}
}
#endif

#if 0
static void second_screen_enable(void *device_data)
{
	struct sec_ts_data *ts = g_ts_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };


/* set lowpower mode by spay, edge_swipe function.
	ts->lowpower_mode = sec->cmd_param[0];
*/
	if (sec->cmd_param[0])
		ts->lowpower_mode |= SEC_TS_MODE_LSI_SIDE_GESTURE;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_LSI_SIDE_GESTURE;

	return;

}

static void set_lowpower_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };


	return;

}
#endif
