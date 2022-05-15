/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sec_ts.h"

#include "../vts_core.h"
//#include <linux/bbk_drivers_info.h>

#ifdef USE_POWER_RESET_WORK
struct sec_ts_data *tsp_info;
struct sec_ts_data *g_ts_data;

static void sec_ts_reset_work(struct work_struct *work);
#endif
#if 0
static void sec_ts_read_info_work(struct work_struct *work);
#endif
#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev);
static void sec_ts_input_close(struct input_dev *dev);
#endif

#define parse_property(np, prop_name, data_type, val, err_return, err_default) do { \
		if(of_property_read_##data_type(np, prop_name, val)) { \
			if (err_return) {\
				VTE("get property "prop_name" failed!!\n"); \
				return -EINVAL; \
			} \
			\
			*val = err_default; \
			VTI("property "prop_name" not configed, set to default value:"#err_default"\n"); \
		} \
	} while (0)

#define parse_property_u32_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, u32, val, false, err_default)
#define parse_property_u8_with_default(np, prop_name, val, err_default) parse_property(np, prop_name, u8, val, false, err_default)

int sec_ts_read_information(struct sec_ts_data *ts);

static int bbk_slsi_setEdgeRejectArea(struct vts_device *vtsdev, struct vts_edge_cmd *cmd);

int sec_ts_i2c_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[I2C_WRITE_BUFFER_SIZE + 1];
	int ret;
	unsigned char retry;
	struct vts_device *vtsdev = ts->vtsdev;
	struct i2c_msg msg;
	int i;

	if (len > I2C_WRITE_BUFFER_SIZE) {
		VTE("%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1)
			break;

		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
			VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
			mutex_unlock(&ts->i2c_mutex);
			goto err;
		}

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			VTE("%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			ts->comm_err_count++;
			vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
		}
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C write over retry limit\n", __func__);
		ret = -EIO;
		vts_report_ic_exception(vtsdev, VTS_EXCEPTION_I2C_ERR, ret);
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going) {
			if (ts->reset_count > RESET_MAX) {
				VTE("tp reset over retry limit %d", ts->reset_count -1);
				goto err;
			}
			ts->reset_count++;
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
		}
#endif
	} else {
		ts->reset_count = 0;
	}

	vts_debug_code() {
		pr_info("VIVO_TS sec_input:i2c_cmd: W: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	if (ret == 1)
		return 0;
err:
	return -EIO;
}


int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 buf[4];
	int ret;
	unsigned char retry;	
	struct vts_device *vtsdev = ts->vtsdev;
	struct i2c_msg msg[2];
	int remain = len;
	int i;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	mutex_lock(&ts->i2c_mutex);

	if (len <= ts->i2c_burstmax) {

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			if (ret == 2)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
				vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
			}
		}

	} else {
		/*
		 * I2C read buffer is 256 byte. do not support long buffer over than 256.
		 * So, try to seperate reading data about 256 bytes.
		 */

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
				vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
			}
		}

		do {
			if (remain > ts->i2c_burstmax)
				msg[1].len = ts->i2c_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->i2c_burstmax;

			for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
				ret = i2c_transfer(ts->client->adapter, &msg[1], 1);
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
					VTE("%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
					mutex_unlock(&ts->i2c_mutex);
					goto err;
				}

				if (retry > 1) {
					VTE("%s: I2C retry %d, ret:%d\n",
						__func__, retry + 1, ret);
					ts->comm_err_count++;
					vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
				}
			}

			msg[1].buf += msg[1].len;

		} while (remain > 0);

	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C read over retry limit\n", __func__);
		ret = -EIO;
		vts_report_ic_exception(vtsdev, VTS_EXCEPTION_I2C_ERR, ret);
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going) {
			if (ts->reset_count > RESET_MAX) {
				VTE("tp reset over retry limit %d", ts->reset_count -1);
				goto err;
			}
			ts->reset_count++;
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));	
		}							
#endif

	} else {
		ts->reset_count = 0;
	}

	vts_debug_code() {
		pr_info("VIVO_TS sec_input:i2c_cmd: R: %02X | ", reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	return ret;

err:
	return -EIO;
}

static int sec_ts_i2c_write_burst(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	int retry;
	struct vts_device *vtsdev = ts->vtsdev;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_master_send(ts->client, data, len);
		if (ret == len)
			break;

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			VTE("%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->i2c_mutex);
	if (retry == SEC_TS_I2C_RETRY_CNT) {
		VTE("%s: I2C write over retry limit\n", __func__);
		ret = -EIO;	
		vts_report_ic_exception(vtsdev, VTS_EXCEPTION_I2C_ERR, ret);
	}

	return ret;
}

static int sec_ts_i2c_read_bulk(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;
	unsigned char retry;
	int remain = len;
	struct i2c_msg msg;
	struct vts_device *vtsdev = ts->vtsdev;

	msg.addr = ts->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	mutex_lock(&ts->i2c_mutex);

	do {
		if (remain > ts->i2c_burstmax)
			msg.len = ts->i2c_burstmax;
		else
			msg.len = remain;

		remain -= ts->i2c_burstmax;

		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, &msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);

			if (retry > 1) {
				VTE("%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->comm_err_count++;
			}
		}

		if (retry == SEC_TS_I2C_RETRY_CNT) {
			VTE("%s: I2C read over retry limit\n", __func__);
			ret = -EIO;			
			vts_report_ic_exception(vtsdev, VTS_EXCEPTION_I2C_ERR, ret);
			break;
		}
		msg.buf += msg.len;

	} while (remain > 0);

	mutex_unlock(&ts->i2c_mutex);

	if (ret == 1)
		return 0;

	return -EIO;
}

/*
#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
#include <linux/sec_debug.h>
extern struct tsp_dump_callbacks dump_callbacks;
static struct delayed_work *p_ghost_check;
*/
static void sec_ts_check_rawdata(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data, ghost_check.work);

	if (ts->tsp_dump_lock == 1) {
		VTE("%s: ignored ## already checking..\n", __func__);
		return;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: ignored ## IC is power off\n", __func__);
		return;
	}

	VTI("%s: start ##\n", __func__);
	sec_ts_run_rawdata_all(ts);
	msleep(100);

	VTI("%s: done ##\n", __func__);
}
/*
static void dump_tsp_log(void)
{
	pr_info("%s: %s %s: start\n", SEC_TS_I2C_NAME, SECLOG, __func__);

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		VTE(("%s: %s %s: ignored ## lpm charging Mode!!\n", SEC_TS_I2C_NAME, SECLOG, __func__);
		return;
	}
#endif

	if (p_ghost_check == NULL) {
		VTE(("%s: %s %s: ignored ## tsp probe fail!!\n", SEC_TS_I2C_NAME, SECLOG, __func__);
		return;
	}
	schedule_delayed_work(p_ghost_check, msecs_to_jiffies(100));
}
#endif
*/

void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack)
{
	int rc = -1;
	int retry = 0;
	u8 tBuff[SEC_TS_EVENT_BUFF_SIZE] = {0,};

	memset(tBuff, 0, SEC_TS_EVENT_BUFF_SIZE * sizeof(u8));
	
	while (sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, tBuff, SEC_TS_EVENT_BUFF_SIZE) > 0) {
		if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		} else if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_VENDOR_INFO) {
			if (tBuff[1] == ack) {
				rc = 0;
				break;
			}
		}

		if (++retry > (ts->mode_change_retry_flag ? SEC_TS_WAIT_RESET_RETRY_CNT : SEC_TS_WAIT_RETRY_CNT)) {
			VTE("%s: Time Over\n", __func__);
			break;
		}
		sec_ts_delay(20);
	}

	VTI("%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X [%d]\n",
			__func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3],
			tBuff[4], tBuff[5], tBuff[6], tBuff[7], retry);

	return rc;
}

int bbk_slsi_erase_vivo_cal(struct sec_ts_data *info ,u8 val)
{
	int ret = 0;
	struct sec_ts_data *ts = info;
	u8 tBuff[1] = { 0x00 };
	VTI("enter");
	tBuff[0] = val;
	ret = sec_ts_i2c_write(ts, SEC_TS_VIVO_STATUS_COMMAND, tBuff, sizeof(tBuff));// Set to Doesn't have vivo offset calibration
	if (ret < 0) {
		VTI("fail to Erase Vivo Calibration flag!");
	}

	return ret;
}

int bbk_slsi_erase_vivo_specail_cal(struct sec_ts_data *info, u8 val)
{
	int ret = 0;
	struct sec_ts_data *ts = info;
	u8 tBuff[1] = { 0x00 };
	VTI("enter");
	tBuff[0] = val;
	ret = sec_ts_i2c_write(ts, SEC_TS_VIVO_SPECIAL_CALI_STATUS_COMMAND, tBuff, sizeof(tBuff));// Set to Doesn't have vivo offset calibration
	if (ret < 0) {
		VTI("fail to Erase Vivo Calibration flag!");
	}

	return ret;
}

int bbk_slsi_get_vsync_freq(struct sec_ts_data *info, int *vsync_freq)
{
	int ret = 0;
	struct sec_ts_data *ts = info;
	u8 vivo_vsync_freq[8] = {0};
	VTI("enter");
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_VSYNC_FREQ, vivo_vsync_freq, 8);

	if (ret < 0) {
		VTI("failed to read Vivo Calibration status");
		return ret;
	}

	VTI("vsync frequency is %d", vivo_vsync_freq[7]);
	*vsync_freq = vivo_vsync_freq[7];
	
	return ret;
}


int bbk_slsi_get_vivo_calibration_status(struct sec_ts_data *info, int *cali_satus)
{
	int ret = 0;
	struct sec_ts_data *ts = info;
	u8 vivo_cal_status[1] = {0};
	VTI("enter");
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_STATUS_COMMAND, vivo_cal_status, 1);

	if (ret < 0) {
		VTI("failed to read Vivo Calibration status");
		return ret;
	}

	VTI("cal status is 0x%x", vivo_cal_status[0]);
	*cali_satus = vivo_cal_status[0];
	return  ret;
}

int bbk_slsi_get_vivo_special_calibration_status(struct sec_ts_data *info, int *cali_satus)
{
	int ret = 0;
	struct sec_ts_data *ts = info;
	u8 vivo_cal_status[1] = {0};
	VTI("enter");
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_SPECIAL_CALI_STATUS_COMMAND, vivo_cal_status, 1);

	if (ret < 0) {
		VTI("failed to read Vivo Calibration status");
		return ret;
	}

	VTI("special cal status from 0x1E is 0x%x", vivo_cal_status[0]);
	*cali_satus = vivo_cal_status[0];
	return  ret;
}


int bbk_slsi_start_force_calibration(struct sec_ts_data *info)
{
	int ret = 0;
	struct sec_ts_data *ts = info;

	VTI("enter");
	ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SET);
	if (ret < 0) {
		VTI("fail to write OFFSET CAL SEC!");
	} else {
		VTI("success to write OFFSET CAL SEC!");
	}

	return ret;
}

void sec_ts_reinit(struct sec_ts_data *ts)
{
	u8 w_data[2] = {0x00, 0x00};
	int ret = 0;
	int i;
	struct vts_device *vtsdev = ts->vtsdev;

	VTI("%s : charger=0x%x, touch_functions=0x%x, Power mode=0x%x, noise_mode=%d\n",
			__func__, ts->charger_mode, ts->touch_functions,
			ts->power_status, ts->touch_noise_status);

	ts->touch_noise_status = 0;

	/* charger mode */
	if (ts->charger_mode != SEC_TS_BIT_CHARGER_MODE_NO) {
		w_data[0] = ts->charger_mode;
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SET_TS_CMD_SET_CHARGER_MODE);
	}

	/* Cover mode */
	if (ts->touch_functions & SEC_TS_BIT_SETFUNC_COVER) {
		w_data[0] = ts->cover_cmd;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SEC_TS_CMD_SET_COVERTYPE);
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
	if (ret < 0)
		VTE("%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_SET_TOUCHFUNCTION);

	/* Power mode */
	if (ts->power_status == SEC_TS_STATE_LPM) {
		/*
		w_data[0] = (ts->lowpower_mode & SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SEC_TS_CMD_WAKEUP_GESTURE_MODE);
		*/

		w_data[0] = TO_LOWPOWER_MODE;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, (u8 *)&w_data[0], 1);
		if (ret < 0)
			VTE("%s: Failed to send command(0x%x)",
					__func__, SEC_TS_CMD_SET_POWER_MODE);

		sec_ts_delay(50);

	} else {
		sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);
	}
	vts_reset(vtsdev);

	ts->edge_app_flag = 0;
	for(i = 1; i < EDGE_SAVE_NUM; i++) {
		bbk_slsi_setEdgeRejectArea(vtsdev, &ts->cmd_store[i]);
	}
	
	return;
}

static int sec_get_touch_ic_mode(struct vts_device *vtsdev)
{
	int ret = 0;
	unsigned char read_result[4] = {0};

	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, read_result, 4);
	if (ret < 0) {
		VTI("read data err");
	}
	VTI("read status:0x%x, 0x%x, 0x%x, 0x%x", read_result[0], read_result[1], read_result[2], read_result[3]);

	if (read_result[1] == SEC_TS_READ_IS_NORMAL_MODE)
		ret = 1;
	return ret;
}

/* 90HZ calibration_status
 * 0-susscess, 1-fail
 */
static int sec_get_vivo_special_calibration_status(struct vts_device *vtsdev)
{
	int ret = 0;
	int cali_status = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	ret = bbk_slsi_get_vivo_special_calibration_status(ts, &cali_status);
	if (ret >= 0 && (cali_status == 0xa1))
		return 0;
	else
		return 1;
}

/* When enter call mode : state = 1
 * When exit call mode : state = 0
 */

static int sec_ts_set_callmode(struct vts_device *vtsdev, int state)
{
       struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
       int ret = 0;
       u8 para = 0;

       VTI("%s: state = %d", __func__, state);

       para = state;

       ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_CALLMODE, &para, 1);
       if (ret < 0)
               VTE("%s: send callmode cmd error", __func__);

       return ret;
}

/* When enable FOD detection : state = 1
 * When disable FOD detection : state = 0
 */
static int sec_ts_set_fod_detection(struct vts_device *vtsdev, int enable)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 para = 0;
	u8 get_para = 0;

	VTI("%s: set_state = %d", __func__, enable);

	para = enable;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_FOD_DETECTION, &para, 1);
	if (ret < 0)
		VTE("%s: send detection set cmd error", __func__);
	
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_SET_FOD_DETECTION, &get_para, 1);
	if (ret < 0)
		VTE("%s: send detection set cmd error", __func__);
	VTI("%s:fod detection enable is %d", __func__, get_para);
	if (ret >= 0)
		ret = 0;
	return ret;
}

static const int gesture_bit[] = {
	0,
	VTS_GESTURE_C,
	VTS_GESTURE_E,
	VTS_GESTURE_F,
	VTS_GESTURE_M,
	VTS_GESTURE_O,
	VTS_GESTURE_DCLICK,
	VTS_GESTURE_W,
	VTS_GESTURE_A,
	VTS_GESTURE_UP,
	VTS_GESTURE_DOWN,
	VTS_GESTURE_LR,
	VTS_GESTURE_LR,
	0,
	0,
	0
};


static int sec_ts_set_gesture(struct vts_device *vtsdev, int enable)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 data[2];
	u16 data_temp = 0;
	int i;

	VTI("%s: set state = %d", __func__, enable);

	if (enable == 0) {
		data_temp = 0;
	} else {
		for (i = 0; i < ARRAY_SIZE(gesture_bit); i++) {
			if (enable & gesture_bit[i])
				data_temp |= (1 << i);
			else
				data_temp &= ~(1 << i);
		}
	}

	data[1] = (u8)data_temp & 0xff;
	data[0] = (u8)(data_temp >> 8) & 0xff;

	if(vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN)) {
		data[0] |= 0x20;
	}

	VTD("set_data = %d, %d", data[0], data[1]);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE, data, 2);
	if (ret < 0)
		VTE("%s: send gesture state cmd error", __func__);

	if (ret >= 0)
		ret = 0;

	return ret;
}

static int sec_ts_get_fw_resolution(struct vts_device *vtsdev, int enable)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	u8 fw_resolution[4] = {0};
	int ret = 0;
	u8 data[1] = {0x11};
	u8 flash_data[5];
	u8 get_data[4];
	u32 reg;
	int display_x = 0;
	int display_y = 0;
	int resolution_adjust = 0;

	if (!enable) {
		VTI("log switch not open !");
		goto exit;
	}
	
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_FW_RESOLUTION, data, 1);
	if (ret < 0) {
		VTE("%s: send 11 to 0xD4 error", __func__);
		goto exit;
	}
	mdelay(1);
	
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_FW_RESOLUTION, get_data, 4);
	if (ret < 0) {
		VTE("%s: fail to get resolution reg addr", __func__);
		goto exit;
	}
	
	reg = get_data[3] | (get_data[2] << 8) | (get_data[1] << 16) | (get_data[0] << 24);
	reg += 0x24;
	flash_data[0] = SEC_TS_CMD_FLASH_READ_ADDR;
	flash_data[1] = reg >> 24;
	flash_data[2] = (reg >> 16) & 0xff;
	flash_data[3] = (reg >> 8) & 0xff;
	flash_data[4] = reg & 0xff;
	ts->sec_ts_i2c_write_burst(ts, (u8 *)flash_data, 5);
	
	flash_data[0] = SEC_TS_CMD_FLASH_READ_SIZE;
	flash_data[1] = 0;
	flash_data[2] = 0x04;
	flash_data[3] = 0;
	flash_data[4] = 0;
	ts->sec_ts_i2c_write_burst(ts, (u8 *)flash_data, 5);
	mdelay(1);
	
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_FLASH_READ_DATA, fw_resolution, 4);
	if (ret < 0) {
		VTE("%s: fail to get resolution from reg %d", __func__, reg);
		goto exit;
	}
	vtsdev->fw_y = fw_resolution[0] | fw_resolution[1] << 8;
	vtsdev->fw_x = fw_resolution[2] | fw_resolution[3] << 8;
	VTI("fw_x  fw_y is: %d * %d ", vtsdev->fw_x, vtsdev->fw_y);

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution_adjust);
	if (resolution_adjust) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
		if (display_x == vtsdev->fw_x && display_y == vtsdev->fw_y)
			ts->report_byte_flag = SEC_TS_REPORT_8_BYTE;
	}

	if (ret >= 0)
		ret = 0;

exit:
	return ret;
}


#include <uapi/linux/input.h>
int sec_ts_set_active_mode(struct sec_ts_data *ts, bool enable)
{
	int ret;
	u8 active_mode_on[] = {0x01};
	u8 active_mode_off[] = {0x00};

	if (enable) {
		VTI("sec_ts_set_active_mode : active mode on!!==\n");
		ret = ts->sec_ts_i2c_write(ts, 0x48, active_mode_on, sizeof(active_mode_on));
		if (ret < 0)
			VTE("sec_ts_set_active_mode: fail to write ACTIVE_MODE_ON\n");
	} else {
		VTI("sec_ts_set_active_mode : active mode off!!\n");
		ret = ts->sec_ts_i2c_write(ts, 0x48, active_mode_off, sizeof(active_mode_off));
		if (ret < 0)
			VTE("sec_ts_set_active_mode: fail to write ACTIVE_MODE_OFF\n");
	}

	return ret;
}

static int sec_ts_get_channel_num(int channel_bit)
{	
	int num;
	int channel_num = 0;

	for(num = 0; num < 32; num ++) {
		if ((channel_bit >> num) & 0x01)
			channel_num ++;
	}
	VTD("channel_num %d", channel_num);

	return channel_num;
}

static void sec_ts_read_event(struct sec_ts_data *ts, ktime_t kt)
{
	int ret;
	u8 t_id;
	u8 event_id;
	u8 left_event_count;
	u8 *event_buff;
	struct sec_ts_event_coordinate *p_event_coord;
	struct sec_ts_gesture_status *p_gesture_status;
	struct sec_ts_event_status *p_event_status;
	struct vts_device *vtsdev = ts->vtsdev;
	int curr_pos;
	int remain_event_count = 0;
	int pre_ttype = 0;
	u16 double_click_x = 0;
	u16 double_click_y = 0;
#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
	int pick[3] = {0};
#endif
	u32 display_max_x = 1;
	u32 display_max_y = 1;
	u32 touch_max_x = 1;
	u32 touch_max_y = 1;
	u32 resolution = 0;
	u32 channel_comp;
	int Tx_bit = 0;
	int Rx_bit = 0;
	int Tx_num = 0;
	int Rx_num = 0;
	u8 need_release = 0;
	u32 virtual_key = 0;
	u32 hsync_freq, hsync_collect = 0;
	u32 hsync_lower, hsync_upper = 0;
	static u8 *read_event_buffer = NULL;
	u8 custom_data[2] = {0};

	if (read_event_buffer == NULL) {
		read_event_buffer = kzalloc(MAX_EVENT_COUNT * SEC_TS_EVENT_DOUBLE_BUFF_SIZE * sizeof(u8), GFP_DMA);
		if (read_event_buffer == NULL) {
			VTE("all_event_buffer fai to allocate memory!");
			return;
		}
	}

	memset(read_event_buffer, 0, MAX_EVENT_COUNT * SEC_TS_EVENT_DOUBLE_BUFF_SIZE * sizeof(u8));

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_max_y);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &touch_max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &touch_max_y);
	}

	mutex_lock(&ts->device_mutex);
	if (ts->power_status == SEC_TS_STATE_LPM) {
		//wake_lock_timeout(&ts->wakelock, msecs_to_jiffies(500));

		/* waiting for blsp block resuming, if not occurs i2c error */
		ret = wait_for_completion_interruptible_timeout(&ts->resume_done, msecs_to_jiffies(500));
		if (ret == 0) {
			VTE("%s: LPM: pm resume is not handled\n", __func__);
			mutex_unlock(&ts->device_mutex);
			return;
		}

		if (ret < 0) {
			VTE("%s: LPM: -ERESTARTSYS if interrupted, %d\n", __func__, ret);
			mutex_unlock(&ts->device_mutex);
			return;
		}

		VTI("%s: run LPM interrupt handler, %d\n", __func__, ret);
		/* run lpm interrupt handler */
	}

	ret = t_id = event_id = curr_pos = remain_event_count = 0;
	/* repeat READ_ONE_EVENT until buffer is empty(No event) */
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, (u8 *)read_event_buffer, ts->event_buf_size);
	if (ret < 0) {
		VTE("%s: i2c read one event failed\n", __func__);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	VTD("ONE: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			read_event_buffer[0], read_event_buffer[1],
			read_event_buffer[2], read_event_buffer[3],
			read_event_buffer[4], read_event_buffer[5],
			read_event_buffer[6], read_event_buffer[7]);

	if (read_event_buffer[0] == 0) {
		VTI("%s: event buffer is empty\n", __func__);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	left_event_count = read_event_buffer[7] & 0x3F;
	remain_event_count = left_event_count;

	if (left_event_count > MAX_EVENT_COUNT) {
		VTE("%s: event buffer overflow\n", __func__);

		/* write clear event stack command when read_event_count > MAX_EVENT_COUNT */
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0)
			VTE("%s: i2c write clear event failed\n", __func__);

		sec_ts_unlocked_release_all_finger(ts);
		vts_report_release(vtsdev);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	if (left_event_count > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ALL_EVENT, read_event_buffer + ts->event_buf_size,
				sizeof(u8) * (ts->event_buf_size) * (left_event_count));
		if (ret < 0) {
			VTE("%s: i2c read one event failed\n", __func__);
			mutex_unlock(&ts->device_mutex);
			return;
		}
	}

	do {
		event_buff = read_event_buffer + (curr_pos * ts->event_buf_size);
		event_id = event_buff[0] & 0x3;

		VTD("ALL: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				event_buff[0], event_buff[1], event_buff[2], event_buff[3],
				event_buff[4], event_buff[5], event_buff[6], event_buff[7]);

		switch (event_id) {
		case SEC_TS_STATUS_EVENT:
			p_event_status = (struct sec_ts_event_status *)event_buff;

			/* tchsta == 0 && ttype == 0 && eid == 0 : buffer empty */
			if (p_event_status->stype > 0)
				VTI("%s: STATUS %x %x %x %x %x %x %x %x\n", __func__,
						event_buff[0], event_buff[1], event_buff[2],
						event_buff[3], event_buff[4], event_buff[5],
						event_buff[6], event_buff[7]);

		
			vts_property_get(vtsdev, VTS_PROPERTY_TP_CHANNEL_COMP, &channel_comp);
			if (channel_comp) {
				if (event_buff[1] == 0xf0) {
					VTI("TX channel error");
					Tx_bit = (((((event_buff[5] << 8) | event_buff[4]) << 8) | event_buff[3]) << 8)| event_buff[2];
					Tx_num = sec_ts_get_channel_num(Tx_bit);
				} else if (event_buff[1] == 0xf1) {
					VTI("RX channel error");
					Rx_bit = (((((event_buff[5] << 8) | event_buff[4]) << 8) | event_buff[3]) << 8)| event_buff[2];
					Rx_num = sec_ts_get_channel_num(Rx_bit);
				} else if (event_buff[1] == 0xf2) {
					VTI("MULTI channel error");
					++Rx_num;
					++Tx_num;
				}

				if(Tx_num || Rx_num) {
					vts_channel_broken_collect(TOUCH_VCODE_CHN_EVENT, Tx_num, Rx_num, Tx_bit, Rx_bit);
				}
			}

			/* watchdog reset -> send SENSEON command */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_BOOT_COMPLETE) &&
				(p_event_status->status_data_1 == 0x20 || event_buff[2] == 0x10)) {
				if (p_event_status->status_data_1 == 0x20)
					vts_abnormal_reset_collect(TOUCH_VCODE_UNEXPECTED_RESET_EVENT);
				sec_ts_unlocked_release_all_finger(ts);
				vts_report_release(vtsdev);

				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
				if (ret < 0)
					VTE("%s: fail to write Sense_on\n", __func__);

				sec_ts_reinit(ts);
			}

			/* event queue full-> all finger release */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_QUEUE_FULL)) {
				VTE("%s: IC Event Queue is full\n", __func__);
				sec_ts_unlocked_release_all_finger(ts);
				vts_report_release(vtsdev);
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id == SEC_TS_ERR_EVENT_ESD)) {
				VTE("%s: ESD detected. run reset\n", __func__);
#ifdef USE_RESET_DURING_POWER_ON
				schedule_work(&ts->reset_work.work);
#endif
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id == SEC_TS_ACK_WET_MODE)) {
				ts->wet_mode = p_event_status->status_data_1;
				VTI("%s: water wet mode %d\n",
						__func__, ts->wet_mode);
				if (ts->wet_mode)
					ts->wet_count++;
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
				(p_event_status->status_id == SEC_TS_VENDOR_ACK_PALM_EVENT)) {
				if (p_event_status->status_data_1 == SEC_TS_ACK_PALM_DETECT) {
                	VTI("%s: Palm Detect", __func__);
                	/* Detect */
					ts->large_press = true;
					vts_report_point_down(vtsdev, 0, 0, 0, 0, 0, 0, ts->large_press, NULL, 0, kt);  // simulate a point to report large press event
       			}
        		else if (p_event_status->status_data_1 == SEC_TS_ACK_PALM_RELEASE) {
                	VTI("%s: Palm Release", __func__);
                	/* Release */
					ts->large_press = false;
					vts_report_point_up(vtsdev, 0, 0, 0, 0, 0, 0, ts->large_press, kt); // simulate a point to report large press event
        		}
				if (ts->large_press)
					ts->large_press_count++;
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
					(p_event_status->status_id == SEC_TS_VENDOR_ACK_NOISE_STATUS_NOTI)) {

				ts->touch_noise_status = !!p_event_status->status_data_1;
				VTI("%s: TSP NOISE MODE %s[%d]\n",
						__func__, ts->touch_noise_status == 0 ? "OFF" : "ON",
						p_event_status->status_data_1);

				if (ts->touch_noise_status)
					ts->noise_count++;
			}
			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
                (p_event_status->status_id == SEC_TS_VENDOR_ACK_CALLMODE_EVENT)) {
        		if (p_event_status->status_data_1 == SEC_TS_ACK_CALL_DETECT) {
                	VTI("%s: Proxi Detect", __func__);
                	/* Detect */
					pick[0] = 0;
					#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
						VTI("algo-prox report near event");
						vts_proxminity_report(vtsdev, pick[0], pick[1], pick[2]);
					#endif
       			}
        		else if (p_event_status->status_data_1 == SEC_TS_ACK_CALL_RELEASE) {
                	VTI("%s: Proxi Release", __func__);
                	/* Release */
					pick[0] = 1;
					#ifdef CONFIG_VIRTUAL_PROX_ON_TOUCH
						VTI("algo-prox report far event");
						vts_proxminity_report(vtsdev, pick[0], pick[1], pick[2]);
					#endif
        		}
			}

			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
			                (p_event_status->status_id == SEC_TS_VENDOR_ACK_FOD_EVENT)) {
			        if (p_event_status->status_data_1 == SEC_TS_ACK_FOD_DETECT) {
			                VTI("%s: FOD Detect", __func__);
						ts->tx_freq_index = event_buff[15];
						vtsdev->aoi_real_point[0] = (int)p_event_status->status_data_2 + ((int)(p_event_status->status_data_3 & 0xF0) << 4);
						vtsdev->aoi_real_point[1] = (int)p_event_status->status_data_4 + ((int)(p_event_status->status_data_3 & 0x0F) << 8);
						VTI("real coordinate x = %d, y = %d", vtsdev->aoi_real_point[0], vtsdev->aoi_real_point[1]);

							vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
			        }
			        else if (p_event_status->status_data_1 == SEC_TS_ACK_FOD_RELEASE) {
			                VTI("%s: FOD Release", __func__);
			                /* FOD Release */
							vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
						if (ts->fod_finger_count >= 2)
							vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT);
						vts_report_release_aoi_point(vtsdev);
						ts->fod_finger_count = 0;
			        }
			}

			vts_property_get(vtsdev, VTS_PROPERTY_HSYNC_COLLECT, &hsync_collect);
			if (hsync_collect && (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL)) {
				vts_property_get(vtsdev, VTS_PROPERTY_HSYNC_LOWER, &hsync_lower);
				vts_property_get(vtsdev, VTS_PROPERTY_HSYNC_UPPER, &hsync_upper);
				if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
					(p_event_status->status_id == SEC_TS_VENDOR_ACK_HSYNC_IMMIGRATE)) {
						hsync_freq = p_event_status->status_data_1 | (p_event_status->status_data_2<< 8);
						VTI("hsync=%dkhz", hsync_freq);
#if 0
						if (hsync_freq < hsync_lower || (hsync_freq > hsync_upper && hsync_freq < 30000))
							vts_hsync_state_collect(TOUCH_VCODE_HSYNC_EVENT);
#endif
				}
			}

			vts_property_get(vtsdev, VTS_PROPERTY_VIRTUAL_KEY, &virtual_key);
			if (virtual_key) {
				if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
				                (p_event_status->status_id == SEC_TS_VENDOR_ACK_LONGPRESS_EVENT)) {
				        if (p_event_status->status_data_1 == SEC_TS_ACK_INSIDE_SLIDE_DOWN) {
							if (p_event_status->status_data_2 == SEC_TS_ACK_INSIDE_SLIDE_LEFT) {
				                VTI("%s: Inside Slide Left Down", __func__);
								vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT);
								ts->inside_slide = 1;
							} else if (p_event_status->status_data_2 == SEC_TS_ACK_INSIDE_SLIDE_RIGHT) {
				                VTI("%s: Inside Slide Right Down", __func__);
								vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT);
								ts->inside_slide = 2;
							}
				        }
				        else if (p_event_status->status_data_1 == SEC_TS_ACK_INSIDE_SLIDE_RELEASE) {
							if (ts->inside_slide == 1) {
				                VTI("%s: Inside Slide Left Up", __func__);
								vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT);
							} else if (ts->inside_slide == 2) {
				                VTI("%s: Inside Slide Right Up", __func__);
								vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT);
							}
							ts->inside_slide = 0;
				        }
				}

				if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
								(p_event_status->status_id == SEC_TS_VENDOR_ACK_QUIT_ACTIVE_EVENT)) {
						VTI("%s: QUIT active mode", __func__);
						sec_ts_set_active_mode(ts, 0);
						vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE);
						vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE);

				}
		/*******************************add to get fingerprint info begin*************************************/
			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
							(p_event_status->status_id == SEC_TS_VENDOR_ACK_FOD_AREA_INFO)) {
				ts->fod_index = p_event_status->status_data_1 & 0x01;
				ts->fod_touch_id = (p_event_status->status_data_1 >> 1 ) & 0x7F;
				if (ts->fod_index == SEC_TS_ACK_FOD_AREA_INFO_X) {
					ts->fod_area_info[0] = (int)p_event_status->status_data_2 + ((int)(p_event_status->status_data_3 & 0xF0) << 4);
					ts->fod_area_info[1] = (int)p_event_status->status_data_4 + ((int)(p_event_status->status_data_3 & 0x0F) << 8);
					ts->fod_count++;
				}
				else if (ts->fod_index == SEC_TS_ACK_FOD_AREA_INFO_Y) {
					ts->fod_area_info[2] = (int)p_event_status->status_data_2 + ((int)(p_event_status->status_data_3 & 0xF0) << 4);
					ts->fod_area_info[3] = (int)p_event_status->status_data_4 + ((int)(p_event_status->status_data_3 & 0x0F) << 8);
					ts->fod_count++;
				}
				if (ts->fod_count >= 2) {
					ts->fod_count = 0;
					vtsdev->aoi_area_point[0] = (ts->fod_area_info[1] + ts->fod_area_info[0]) / 2;
					vtsdev->aoi_area_point[1] = (ts->fod_area_info[3] + ts->fod_area_info[2]) / 2;
					VTI("area info coordinate is x = %d, y = %d, finger_id = %d", vtsdev->aoi_area_point[0], vtsdev->aoi_area_point[1], ts->fod_touch_id);
					if ((ts->fod_area_info[1] >= ts->fod_area_info[0]) && (ts->fod_area_info[3] >= ts->fod_area_info[2])) {
						ts->fod_area_size[0] = (ts->fod_area_info[1] - ts->fod_area_info[0]) * (ts->fod_area_info[3] - ts->fod_area_info[2]);
					}
				}
			}
			
			if ((p_event_status->stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
							(p_event_status->status_id == SEC_TS_VENDOR_ACK_FOD_DETECT_INFO)) {
				ts->fod_index = p_event_status->status_data_1 & 0x01;
				ts->fod_touch_id = (p_event_status->status_data_1 >> 1 ) & 0x7F;
				ts->fod_area_percent = p_event_status->status_data_2;
				ts->fod_avg_signal = p_event_status->status_data_3;
				if ((ts->fod_index == 1) && (ts->fod_area_percent <= 100)) {
					ts->fod_area_size[1] = (100 - ts->fod_area_percent) * ts->fod_area_size[0] / ts->fod_area_percent;
					VTI("%s: FOD Detect Info: index = %d, touch id = %d, active percentage = %d, aoi_in_size = %d, aoi_out_size = %d, avg signal = %d", 
						__func__, ts->fod_index, ts->fod_touch_id, ts->fod_area_percent,
						ts->fod_area_size[0], ts->fod_area_size[1], ts->fod_avg_signal);
					if (ts->fod_touch_id >= 1) {
						vtsdev->aoi_info[ts->fod_touch_id - 1].aoi_area_in = ts->fod_area_size[0];
						vtsdev->aoi_info[ts->fod_touch_id - 1].aoi_area_out = ts->fod_area_size[1];
						vtsdev->aoi_info[ts->fod_touch_id - 1].aoi_area_width = ts->fod_area_info[1] - ts->fod_area_info[0];
						vtsdev->aoi_info[ts->fod_touch_id - 1].aoi_area_height = ts->fod_area_info[3] - ts->fod_area_info[2];
						vtsdev->aoi_info[ts->fod_touch_id - 1].aoi_avg_signal = ts->fod_avg_signal;
						ts->fod_finger_count++;
						if (ts->fod_finger_count >= 2) {
							vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT);
						}	
					}
				}
					
			}
		/*******************************add to get fingerprint info end*************************************/
	}
							
			break;

		case SEC_TS_COORDINATE_EVENT:
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				VTE("%s: device is closed\n", __func__);
				break;
			}
			p_event_coord = (struct sec_ts_event_coordinate *)event_buff;

			t_id = (p_event_coord->tid - 1);
			custom_data[0] = p_event_coord->tx_freq_8_15;
			custom_data[1] = p_event_coord->tx_freq_0_7;

			if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
				pre_ttype = ts->coord[t_id].ttype;
				ts->coord[t_id].id = t_id;
				ts->coord[t_id].action = p_event_coord->tchsta;
				if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE) {
					ts->coord[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
					ts->coord[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
				} else if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
					ts->coord[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0) | (p_event_coord->x_15_12 << 12);
					ts->coord[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0) | (p_event_coord->y_15_12 << 12);
				}
				ts->coord[t_id].z = p_event_coord->z & 0x3F;
				ts->coord[t_id].ttype = p_event_coord->ttype_3_2 << 2 | p_event_coord->ttype_1_0 << 0;
				ts->coord[t_id].major = p_event_coord->major;
				ts->coord[t_id].minor = p_event_coord->minor;

				if (!ts->coord[t_id].palm && (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM))
					ts->coord[t_id].palm_count++;

				ts->coord[t_id].palm = (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM);
				ts->coord[t_id].left_event = p_event_coord->left_event;

				if (ts->coord[t_id].z <= 0)
					ts->coord[t_id].z = 1;

				if ((ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_NORMAL)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_WET)
						|| (ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GLOVE)) {

					if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
						if (ts->touch_count > 0)
							ts->touch_count--;
						vts_report_point_up(vtsdev, ts->coord[t_id].id, ts->touch_count,  ts->coord[t_id].x, ts->coord[t_id].y,
							ts->coord[t_id].major,ts->coord[t_id].minor, 0, kt);
						if (ts->touch_count == 0) {
							need_release = 1;
							ts->check_multi = 0;
						}
						
						ts->coord[t_id].action = SEC_TS_COORDINATE_ACTION_NONE;
						ts->coord[t_id].mcount = 0;
						ts->coord[t_id].palm_count = 0;


					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS) {

					ktime_get_ts64(&ts->time_pressed[t_id]);
					ts->touch_count++;
					ts->all_finger_count++;

						ts->max_z_value = max((unsigned int)ts->coord[t_id].z, ts->max_z_value);
						ts->min_z_value = min((unsigned int)ts->coord[t_id].z, ts->min_z_value);
						ts->sum_z_value += (unsigned int)ts->coord[t_id].z;
						if ((ts->touch_count > 4) && (ts->check_multi == 0)) {
							ts->check_multi = 1;
							ts->multi_count++;
						}
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
						//vivoTsInputReport(VTS_TOUCH_DOWN, ts->coord[t_id].id, ts->coord[t_id].x, ts->coord[t_id].y,  ts->coord[t_id].major);
						vts_report_point_down(vtsdev, ts->coord[t_id].id, ts->touch_count,  ts->coord[t_id].x, ts->coord[t_id].y,
							ts->coord[t_id].major,ts->coord[t_id].minor, 0, custom_data, sizeof(custom_data), kt);

						//VTI("%s[P] tID:%d x:%d y:%d z:%d major:%d minor:%d tc:%d type:%X noise:%x\n",
						//		ts->dex_name, t_id, ts->coord[t_id].x,
						//		ts->coord[t_id].y, ts->coord[t_id].z,
						//		ts->coord[t_id].major, ts->coord[t_id].minor,
						//		ts->touch_count,
						//		ts->coord[t_id].ttype, ts->touch_noise_status);
#else
						//vivoTsInputReport(VTS_TOUCH_DOWN, ts->coord[t_id].id, ts->coord[t_id].x, ts->coord[t_id].y,  ts->coord[t_id].major);
                        vts_report_point_down(vtsdev, ts->coord[t_id].id, ts->touch_count,  ts->coord[t_id].x, ts->coord[t_id].y,
							ts->coord[t_id].major,ts->coord[t_id].minor, 0, custom_data, sizeof(custom_data), kt);
						//VTI("%s[P] tID:%d z:%d major:%d minor:%d tc:%d type:%X noise:%x\n",
						//		ts->dex_name,
						//		t_id, ts->coord[t_id].z, ts->coord[t_id].major,
						//		ts->coord[t_id].minor, ts->touch_count,
						//		ts->coord[t_id].ttype, ts->touch_noise_status);
#endif
					} else if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_MOVE) {
						//vivoTsInputReport(VTS_TOUCH_DOWN, ts->coord[t_id].id, ts->coord[t_id].x, ts->coord[t_id].y,  ts->coord[t_id].major);
						vts_report_point_down(vtsdev, ts->coord[t_id].id, ts->touch_count,  ts->coord[t_id].x, ts->coord[t_id].y,
							ts->coord[t_id].major,ts->coord[t_id].minor, 0, custom_data, sizeof(custom_data), kt);
						if ((ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GLOVE) && !ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = true;
							//input_report_switch(ts->input_dev, SW_GLOVE, 1);
						} else if ((ts->coord[t_id].ttype != SEC_TS_TOUCHTYPE_GLOVE) && ts->touchkey_glove_mode_status) {
							ts->touchkey_glove_mode_status = false;
							//input_report_switch(ts->input_dev, SW_GLOVE, 0);
						}
						ts->coord[t_id].mcount++;
					} else {
						VTD("%s: do not support coordinate action(%d)\n", __func__, ts->coord[t_id].action);
					}

					if ((ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS)
							|| (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

						if (ts->coord[t_id].ttype != pre_ttype) {
							VTI("%s : tID:%d ttype(%x->%x)\n",
									__func__, ts->coord[t_id].id,
									pre_ttype, ts->coord[t_id].ttype);
						}
					}

				} else {
					VTD("%s: do not support coordinate type(%d)\n", __func__, ts->coord[t_id].ttype);
				}
			} else {
				VTE("%s: tid(%d) is out of range\n", __func__, t_id);
			}
			break;

		case SEC_TS_GESTURE_EVENT:
			p_gesture_status = (struct sec_ts_gesture_status *)event_buff;
			ts->read_gesture_point_num = p_gesture_status->gesture_point_num;
			VTI("%s: Gesture event. Type : %x, read point num : %d",
					__func__, p_gesture_status->gesture_id, ts->read_gesture_point_num);
			bbk_slsi_gesture_point_handle(vtsdev, p_gesture_status->gesture_id);

			ts->tx_freq_index = event_buff[15];
		
			switch (p_gesture_status->gesture_id) {
			case TOUCH_GESTURE_CLOCK:
			case TOUCH_GESTURE_DOUBLTAP:
				if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE) {
					double_click_x = p_gesture_status->gesture_data_2 | ((u16)(p_gesture_status->gesture_data_4 & 0xf0) << 4);
					double_click_y = p_gesture_status->gesture_data_3 | ((u16)(p_gesture_status->gesture_data_4 & 0x0f) << 8);
				}
				if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
					double_click_x = ts->dbclick_x;
					double_click_y = ts->dbclick_y;
				}
				if (resolution) {
					double_click_x = double_click_x * display_max_x / touch_max_x;
					double_click_y = double_click_y * display_max_y / touch_max_y;
				}
				VTI("double_click_x is %d, double_click_y is %d", double_click_x, double_click_y);
				vts_update_dclick_point(vtsdev, double_click_x, double_click_y);
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
				break;
			case TOUCH_GESTURE_W:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
				break;
			case TOUCH_GESTURE_O:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
				break;
			case TOUCH_GESTURE_M:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
				break;
			case TOUCH_GESTURE_E:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
				break;
			case TOUCH_GESTURE_F:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
				break;
			case TOUCH_GESTURE_AT:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
				break;
			case TOUCH_GESTURE_C:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
				break;
			case TOUCH_GESTURE_UP:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
				break;
			case TOUCH_GESTURE_DOWN:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
				break;
			case TOUCH_GESTURE_LEFT:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
				break;
			case TOUCH_GESTURE_RIGHT:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
				break;
			case TOUCH_GESTURE_DOUBLTAP_EDGE:
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_VK_DC);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_VK_DC);
				break;
			}

			vts_report_point_sync(vtsdev);
			break;
		default:
			VTE("%s: unknown event %x %x %x %x %x %x\n", __func__,
					event_buff[0], event_buff[1], event_buff[2],
					event_buff[3], event_buff[4], event_buff[5]);
			break;
		}

		curr_pos++;
		remain_event_count--;
	} while (remain_event_count >= 0);

	vts_report_point_sync(vtsdev);
	if (need_release == 1) {
		need_release = 0;
		vts_report_release(vtsdev);
	}
	vts_boost_enable(vtsdev, ts->touch_count);
	mutex_unlock(&ts->device_mutex);
}

static irqreturn_t sec_ts_irq_thread(int irq, void *ptr, ktime_t kt)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)ptr;

	mutex_lock(&ts->eventlock);
	sec_ts_read_event(ts, kt);
	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

int get_tsp_status(void)
{
	return 0;
}
EXPORT_SYMBOL(get_tsp_status);

int sec_ts_set_charger(struct sec_ts_data *ts, int charge_bit)
{
	int ret = 0;
	u8 noise_mode_wirless[] = {0x04};
	u8 noise_mode_on[] = {0x02};
	u8 noise_mode_off[] = {0x01};

	if (charge_bit == 0) {
		VTI("sec_ts_set_charger : charger DISCONNECTED!!\n");
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, noise_mode_off, sizeof(noise_mode_off));
		if (ret < 0)
			VTE("sec_ts_set_charger: fail to write CHARGER_MODE_OFF\n");
	}
	if (charge_bit == 1) {
		VTI("sec_ts_set_charger : ==charger CONNECTED!!==\n");
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, noise_mode_on, sizeof(noise_mode_on));
		if (ret < 0)
			VTE("sec_ts_set_charger: fail to write CHARGER_MODE_ON\n");
	}
	if (charge_bit == 4) {
		VTI("sec_ts_set_charger : ==wirlessd charger CONNECTED!!==\n");
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SET_CHARGER_MODE, noise_mode_wirless, sizeof(noise_mode_wirless));
		if (ret < 0)
			VTE("sec_ts_set_charger: fail to write wirless CHARGER_MODE_ON\n");
	}

	return ret;
}
EXPORT_SYMBOL(sec_ts_set_charger);

int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode)
{
	int ret;

	if (mode)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_GLOVE | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_GLOVE)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: pwr off, glove:%d, status:%x\n", __func__,
				mode, ts->touch_functions);
		goto glove_enable_err;
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0) {
		VTE("%s: Failed to send command", __func__);
		goto glove_enable_err;
	}

	VTI("%s: glove:%d, status:%x\n", __func__,
			mode, ts->touch_functions);

	return 0;

glove_enable_err:
	return -EIO;
}
EXPORT_SYMBOL(sec_ts_glove_mode_enables);

int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable)
{
	int ret;

	VTI("%s: %d\n", __func__, ts->cover_type);


	switch (ts->cover_type) {
	case SEC_TS_VIEW_WIRELESS:
	case SEC_TS_VIEW_COVER:
	case SEC_TS_VIEW_WALLET:
	case SEC_TS_FLIP_WALLET:
	case SEC_TS_LED_COVER:
	case SEC_TS_MONTBLANC_COVER:
	case SEC_TS_CLEAR_FLIP_COVER:
	case SEC_TS_QWERTY_KEYBOARD_EUR:
	case SEC_TS_QWERTY_KEYBOARD_KOR:
		ts->cover_cmd = (u8)ts->cover_type;
		break;
	case SEC_TS_CHARGER_COVER:
	case SEC_TS_COVER_NOTHING1:
	case SEC_TS_COVER_NOTHING2:
	default:
		ts->cover_cmd = 0;
		VTE("%s: not chage touch state, %d\n",
				__func__, ts->cover_type);
		break;
	}

	if (enable)
		ts->touch_functions = (ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER)) | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: pwr off, close:%d, status:%x\n", __func__,
				enable, ts->touch_functions);
		goto cover_enable_err;
	}

	if (enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
		if (ret < 0) {
			VTE("%s: Failed to send covertype command: %d", __func__, ts->cover_cmd);
			goto cover_enable_err;
		}
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		VTE("%s: Failed to send command", __func__);
		goto cover_enable_err;
	}

	VTI("%s: close:%d, status:%x\n", __func__,
			enable, ts->touch_functions);

	return 0;

cover_enable_err:
	return -EIO;


}
EXPORT_SYMBOL(sec_ts_set_cover_type);

void sec_ts_set_grip_type(struct sec_ts_data *ts, u8 set_type)
{
	return;
# if 0
	u8 mode = G_NONE;

	VTI("%s: re-init grip(%d), edh:%d, edg:%d, lan:%d\n", __func__,
			set_type, ts->grip_edgehandler_direction, ts->grip_edge_range, ts->grip_landscape_mode);

	/* edge handler */
	if (ts->grip_edgehandler_direction != 0)
		mode |= G_SET_EDGE_HANDLER;

	if (set_type == GRIP_ALL_DATA) {
		/* edge */
		if (ts->grip_edge_range != 60)
			mode |= G_SET_EDGE_ZONE;

		/* dead zone */
		if (ts->grip_landscape_mode == 1)	/* default 0 mode, 32 */
			mode |= G_SET_LANDSCAPE_MODE;
		else
			mode |= G_SET_NORMAL_MODE;
	}

	if (mode)
		set_grip_data_to_ic(ts, mode);
#endif
}

static int sec_ts_pinctrl_lookup_state(struct sec_ts_data *ts)
{
	if(IS_ERR_OR_NULL(ts->plat_data->pinctrl)) {
		VTE("%s: sec ts pinctrl is null\n", __func__);
		return 0;
	}
	
	ts->pin_state[VTS_INT_ACTIVE] = pinctrl_lookup_state(ts->plat_data->pinctrl, "ts_active");
	if (IS_ERR(ts->pin_state[VTS_INT_ACTIVE]))
		VTE("%s: could not get active pinstate\n", __func__);
	ts->pin_state[VTS_INT_SUSPEND] = pinctrl_lookup_state(ts->plat_data->pinctrl, "ts_suspend");
	if (IS_ERR(ts->pin_state[VTS_INT_SUSPEND]))
		VTE("%s: could not get suspend pinstate\n", __func__);
	ts->pin_state[VTS_RESET_ACTIVE] = pinctrl_lookup_state(ts->plat_data->pinctrl, "ts_reset_active");
	if (IS_ERR(ts->pin_state[VTS_RESET_ACTIVE]))
		VTE("%s: could not get reset active pinstate\n", __func__);
	ts->pin_state[VTS_RESET_SUSPEND] = pinctrl_lookup_state(ts->plat_data->pinctrl, "ts_reset_suspend");
	if (IS_ERR(ts->pin_state[VTS_RESET_SUSPEND]))
		VTE("%s: could not get lowpower pinstate\n", __func__);

	return 0;

}

static int sec_ts_pinctrl_configure(struct sec_ts_data *ts, bool enable)
{
	
	VTI("%s: %s\n", __func__, enable ? "ACTIVE" : "SUSPEND");
	if(enable) {
		if (!IS_ERR_OR_NULL(ts->pin_state[VTS_INT_ACTIVE])) 
			return pinctrl_select_state(ts->plat_data->pinctrl, ts->pin_state[VTS_INT_ACTIVE]);
		
	} else {
		if (!IS_ERR_OR_NULL(ts->pin_state[VTS_INT_SUSPEND]))
			return pinctrl_select_state(ts->plat_data->pinctrl, ts->pin_state[VTS_INT_SUSPEND]);
	}
	return 0;

}

static int sec_ts_reset_pinctrl_configure(struct sec_ts_data *ts, bool enable)
{	
	VTI("%s: %s\n", __func__, enable ? "RESET ACTIVE" : "RESET SUSPEND");

	if(enable) {
		if (!IS_ERR_OR_NULL(ts->pin_state[VTS_RESET_ACTIVE])) 
			return pinctrl_select_state(ts->plat_data->pinctrl, ts->pin_state[VTS_RESET_ACTIVE]);
		
	} else {
		if (!IS_ERR_OR_NULL(ts->pin_state[VTS_RESET_SUSPEND]))
			return pinctrl_select_state(ts->plat_data->pinctrl, ts->pin_state[VTS_RESET_SUSPEND]);
	}

	return 0;
}


/*================BBK*===========================*/
int bbk_slsi_sec_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	unsigned int readbytes = 0xFF;
	short *pRead = NULL;
	int ret = 0;
	int i = 0;
	//int j = 0;

	VTI("%s: enter, type = %d", __func__, type);

	VTI("ts->rx_count=%d, ts->tx_count=%d", ts->rx_count, ts->tx_count);
	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count;

	pRead = data;

	switch (type) {
	VTI("enter in switch");
	case VTS_FRAME_MUTUAL_RAW:
		//read rawdata
		VTI("enter in VTS_FRAME_MUTUAL_RAW");
		ret = run_rawdata_read_all(ts);
		if (ret < 0)
			VTE("run_rawdata_read_all fail");
		break;
	case VTS_FRAME_MUTUAL_DELTA:
		//read delta/diff data
		VTI("enter in VTS_FRAME_MUTUAL_DELTA");
		ret = run_delta_read_all(ts);
		if (ret < 0)
			VTE("run_signaldata_read_all fail");
		break;
	case VTS_FRAME_AMBIENT_BASELINE:
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
	VTI("out rawordiff");
	return ret;
}

/* ret >= 0 clear ok   ret < 0  fail */
static int sec_ts_clear_status(struct sec_ts_data *ts)
{
	int ret = 0;

	if (ts == NULL) {
		VTE("ts is null");
		ret = -EPERM;
	}

	ts->large_press = false;
	ts->edge_app_flag = 0;

	return ret;
}

int bbk_slsi_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int error = 0;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTI("[ERROR] Touch is stopped");
		return -EIO;
	}

	if (ts->client->irq)
		disable_irq(ts->client->irq);

	//firmware_update(driver data, fw data, fw size, bootup, calib, retry)
	if (sec_ts_firmware_update(ts, firmware->data, firmware->size, 0, 0, 0) < 0) {
		VTI("update fw fail");
		error = -1;
	} else {
		VTI("update fw success");
		sec_ts_i2c_read(ts, SEC_TS_READ_POINT_BYTE_FLAG, &ts->report_byte_flag, 1);
		VTI("report point byte flag is %d", ts->report_byte_flag);
		if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE) {
			ts->event_buf_size = SEC_TS_EVENT_BUFF_SIZE;
		}
		if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
			ts->event_buf_size = SEC_TS_EVENT_DOUBLE_BUFF_SIZE;
		}
		error = 0;
	}

	sec_ts_save_version_of_ic(ts);
	sec_ts_clear_status(ts);

	if (ts->client->irq)
		enable_irq(ts->client->irq);

	return error;
}

static int vivo_collcet_ng_panel(struct sec_ts_data *ts)
{
	int ret = 0;
	u8 is_ng_panel[1] = {0};

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_NG_PANEL, is_ng_panel, 1);
	if (ret < 0) {
		VTE("%s: failed to read ng panel(%d)\n", __func__, ret);
		return ret;
	}
	if (0 != is_ng_panel[0]) {
		VTI("is_ng_panel: %X \n", is_ng_panel[0]);
		vts_ng_panel_collect(TOUCH_VCODE_NG_PANEL_DETECT);
	}

	return 0;
}

int sec_ts_enable_irq_wake(struct sec_ts_data *ts, u8 enable)
{
	VTI("irq wake is %s", enable? "enable" : "disable");
	if (enable) {
		if(ts->irq_wake_flag == 1)
			return 0;
		else {
			if (device_may_wakeup(&ts->client->dev)) {
				enable_irq_wake(ts->client->irq);
				ts->irq_wake_flag = 1;
			}
		}
	} else {
		if (ts->irq_wake_flag == 0) {
			return 0;
		} else {
			ts->irq_wake_flag = 0;
			if (device_may_wakeup(&ts->client->dev))
				disable_irq_wake(ts->client->irq);
		}
	}

	return 0;
}

int bbk_slsi_dump_reg_status (struct sec_ts_data *ts)
{
	u8 hsync_reg[8] = {0};
	u8 touch_status[4] = {0};
	u8 interrupt_contrl[1] = {0};
	u8 enable_flag[4] = {0};
	int ret = 0;

	memset(hsync_reg, 0, 8);
	ret = sec_ts_i2c_read(ts, SEC_TS_VIVO_VSYNC_FREQ, hsync_reg, 8);
	if (ret < 0)
		VTI("read hsync_reg err");
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, touch_status, 4);
	if (ret < 0)
		VTI("read touch_status err");
	ret = sec_ts_i2c_read(ts, 0x89, interrupt_contrl, 1);
	if (ret < 0)
		VTI("read interrupt_contrl err");
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_STATEMANAGE_ON, enable_flag, 1);
	if (ret < 0)
		VTI("read enable_flag err");
	VTI("REG STATUS: %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x",
		hsync_reg[0], hsync_reg[1], hsync_reg[2], hsync_reg[3], hsync_reg[4], hsync_reg[5],
		hsync_reg[6], hsync_reg[7], touch_status[0], touch_status[1], touch_status[2], touch_status[3],
		interrupt_contrl[0], enable_flag[0]);

	return ret;
}


//static int last_state = VTS_ST_NORMAL;
extern int tsp_sync_enable(bool enable);
int bbk_slsi_mode_change(struct vts_device *vtsdev, int which)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	unsigned char read_result[4] = {0};
	u32 ng_panel = 0;

	if (which == ts->last_state) {
		VTE("%s: No mode change (last_state:%d, which:%d)",
				__func__, ts->last_state, which);
		return -1;
	}

	mutex_lock(&ts->modechange);

	switch(which) {
		case VTS_ST_NORMAL:
			VTI("change to normal mode");
			sec_ts_enable_irq_wake(ts, false);
			if (ts->last_state == VTS_ST_GESTURE) {
				tsp_sync_enable(1);
				gpio_set_value(ts->plat_data->reset_gpio, 0);
				sec_ts_delay(5);
				gpio_set_value(ts->plat_data->reset_gpio, 1);
				sec_ts_delay(70);
				ts->mode_change_retry_flag = 1;
				
				ret = sec_ts_start_device(ts, false);
				if (ret < 0) {
					VTE("Failed to start device");
				}
			} else { //if (last_state == VTS_ST_SLEEP) 
				ret = sec_ts_start_device(ts, true);
				if (ret < 0) {
					VTE("Failed to start device");
				}
				tsp_sync_enable(1);
			}

			VTI("change to normal mode end");
			ts->mode_change_retry_flag = 0;
			ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, read_result, 4);
			if (ret < 0) {
				VTI("read data err");
			}
			VTI("read status:0x%x, 0x%x, 0x%x, 0x%x", read_result[0], read_result[1], read_result[2], read_result[3]);
			break;
		case VTS_ST_GESTURE:
			VTI("change to gesture mode");

			vts_property_get(vtsdev, VTS_PROPERTY_NG_PANEL, &ng_panel);
			if (ng_panel && ts->last_state == VTS_ST_NORMAL)
				vivo_collcet_ng_panel(ts);

			sec_ts_enable_irq_wake(ts, true);
			tsp_sync_enable(1);
			sec_ts_clear_status(ts);
			if(ts->last_state == VTS_ST_SLEEP) {
				ret = sec_ts_start_device(ts, true);
					if (ret < 0)
						VTE("Failed to start device");
			}

			//ts->lowpower_mode = SEC_TS_MODE_LSI_AOD;
			ts->lowpower_mode = SEC_TS_MODE_LOWPOWER_FLAG;     //To turn on all the wakeup gesture
			ret = sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
			ts->sec_test_flag = 0;
			VTI("change to gesture mode end");
			break;
		case VTS_ST_SLEEP:
			VTI("change to sleep mode");

			vts_property_get(vtsdev, VTS_PROPERTY_NG_PANEL, &ng_panel);
			if (ng_panel && ts->last_state == VTS_ST_NORMAL)
				vivo_collcet_ng_panel(ts);

			tsp_sync_enable(0);
			sec_ts_enable_irq_wake(ts, false);
			if (ts->last_state == VTS_ST_GESTURE) {
				if (ts->longpress_gesture) {
					VTI("longpress down and go to sleep, release longpress gesture");
					vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE);
					vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE);
					ts->longpress_gesture = false;
				}
			}

			sec_ts_clear_status(ts);
			if (vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN) && !vts_state_get(vtsdev, VTS_STA_CALLING))
				mdelay(100);
			ret = sec_ts_stop_device(ts, true);
			sec_ts_reset_pinctrl_configure(ts, false);
			ts->sec_test_flag = 0;
			VTI("change to sleep mode end");
			break;

		default : break;
	}

	if (which == VTS_ST_NORMAL || which == VTS_ST_GESTURE || which == VTS_ST_SLEEP)
		ts->last_state = which;

	mutex_unlock(&ts->modechange);
	if (ret >= 0)
		return 0;
	else
		return ret;
}

int bbk_slsi_expect_mode (void) {
	if (g_ts_data) {
		if (g_ts_data->vtsdev && g_ts_data->vtsdev->policy) {
			return g_ts_data->vtsdev->policy->expected_mode(g_ts_data->vtsdev);
		} else {
			VTE("vtsdev is NULL!");
			return -ENOMEM;
		}
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(bbk_slsi_expect_mode);


int bbk_slsi_get_fw_version(struct vts_device *vtsdev, u64 *version)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;

	//read from chip
	ret = sec_ts_save_version_of_ic(ts);
	if (ret < 0) {
		VTE("fail to save ic version");
		return -EIO;
	}

	*version = ((u64)(ts->plat_data->img_version_of_ic[2]<<8)|ts->plat_data->img_version_of_ic[3])<<16 
											| ((u64)(ts->plat_data->config_version_of_ic[2]<<8)|ts->plat_data->config_version_of_ic[3]);
/*
	switch (which) {
	case FW_VERSION:
		ret = (ts->plat_data->img_version_of_ic[2]<<8)|ts->plat_data->img_version_of_ic[3];
		break;
	case CONFIG_VERSION:
		ret = (ts->plat_data->config_version_of_ic[2]<<8)|ts->plat_data->config_version_of_ic[3];
		break;
	}
*/
	//*version = ((unsigned int)(ts->plat_data->img_version_of_ic[2]<<8)|ts->plat_data->img_version_of_ic[3]);

	if (ret >= 0)
		ret = 0;
	return ret;
}


int bbk_slsi_set_charger_bit(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret;

	mutex_lock(&ts->device_mutex);
	ret = sec_ts_set_charger(ts, state);
	mutex_unlock(&ts->device_mutex);
	return ret;
}


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


int bbk_slsi_readUdd(unsigned char *udd)
{
	struct sec_ts_data *ts = g_ts_data;
	unsigned char buf[32] = {0};
	int ret = 0;
	ret = get_user_nvm_data(ts, buf);
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

	return set_user_nvm_data(ts, buf);
}


int bbk_slsi_set_auto_idle(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
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

/****       add for new game_mode begin       ****/
static int sec_slsi_set_game_mode(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 onoff_4a[1] = {0};

	if (state == PROC_IN_GAME) {
		state = 1;
	}
	if (state == PROC_OUT_GAME) {
		state = 0;
	}

	onoff_4a[0] = state;
	ts->game_mode_in = state;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_ENTER_GAMEMODE, onoff_4a, 1);    //0x4A   set edge reject
	if (ret < 0) {
		VTE("fail to set game mode of SEC_TS_CMD_ENTER_GAMEMODE 0x%X", SEC_TS_CMD_ENTER_GAMEMODE);
	} else {
		VTI("success to set game mode of SEC_TS_CMD_ENTER_GAMEMODE 0x%X", SEC_TS_CMD_ENTER_GAMEMODE);
	}
	sec_ts_delay(20);     
	
	if (ret >= 0)
		ret = 0;

	return ret;
}
static int sec_slsi_set_report_rate(struct vts_device *vtsdev, int rate)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 onoff_4f[1] = {0};

	if (rate == RATE_NORMAL) {
		rate = 0;
	}
	if (rate == RATE_HIGH) {
		rate = 1;
	}
	
	onoff_4f[0] = rate;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_HIGH_FREQ, onoff_4f, 1);     //0x4F   set high rate
	if (ret < 0) {
		VTE("fail to set game mode of SEC_TS_CMD_HIGH_FREQ 0x%X", SEC_TS_CMD_HIGH_FREQ);
	} else {
		VTI("success to set game mode of SEC_TS_CMD_HIGH_FREQ 0x%X", SEC_TS_CMD_HIGH_FREQ);
	}
	sec_ts_delay(20);

	if (ret >= 0)
		ret = 0;

	return ret;
}
static int sec_slsi_set_idle_time(struct vts_device *vtsdev, int idle_time)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 onoff[1] = {0};
	u8 tBuff[2] = { TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH};
	u8 idle_time_buf[1] = {0};
	u8 idle_time_cmd = SEC_TS_CMD_IDLE_TIME; 

	if (idle_time == IDLE_TIME_1S) {
		idle_time = 1;
	}
	if (idle_time == IDLE_TIME_2S) {
		idle_time = 2;
	}
	if (idle_time == IDLE_TIME_10S) {
		idle_time = 10;
	}

	if (ts->game_mode_in) {          //game mode idletime cmd 0x4C    else 0x4E
		idle_time_cmd = SEC_TS_CMD_IDLE_TIME;
	} else {
		idle_time_cmd = SEC_TS_CMD_GAME_OUT_IDLE_TIME;
	}
	if (idle_time == 0) {
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
		sec_ts_delay(20);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_CHG_SYSMODE , tBuff, sizeof(tBuff));
		sec_ts_delay(20);
	} else if (idle_time <= 0xff) {
		idle_time_buf[0] = (u8)idle_time & 0xff;
		ret = ts->sec_ts_i2c_write(ts, idle_time_cmd, idle_time_buf, 1);
		if (ret < 0) {
			VTE("fail to set game mode of idle_time_cmd 0x%X", idle_time_cmd);
		} else {
			VTI("success to set game mode of idle_time_cmd 0x%X", idle_time_cmd);
		}
		sec_ts_delay(20);
	}

	if (ret >= 0)
		ret = 0;

	return ret;
}
/****       add for new game_mode end       ****/


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


int bbk_slsi_sensor_test(struct vts_device *vtsdev, char *buf,  int at_sensor_test_cmd, void *pdata, int tmp)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
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

	/*
	add self test
	*/
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

int bbk_slsi_setEdgeRestainSwitch(struct vts_device *vtsdev, int on)
{
	int ret = 0;
	/*
	add edge restian command
	*/
	int orient = 0;

	VTI("%s: Enter, on = %d\n", __func__, on);
	switch(on) {
		case 1: 
			orient = 0;
			break;
		case 2: 
			orient = 2;
			break;
		case 0:
		default: orient = 1;
	}
	ret = dead_zone_enable(vtsdev, orient);
	return ret;
}

static int bbk_slsi_setBandState(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);

	if (!state && ts->self_sensing_mode == 1) {
		VTI("self sensing is selete AvddSelf(3.0V)");
		ts->self_sensing_mode = 0;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SELF_SENSING_MODE, &ts->self_sensing_mode, 1);
	} else if (state && ts->self_sensing_mode == 0){
		VTI("self sensing is selete RefSelf(1.6V)");
		ts->self_sensing_mode = 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SELF_SENSING_MODE, &ts->self_sensing_mode, 1);
	}

	return ret;
}

static int bbk_slsi_edge_reject_limit(struct vts_device *vtsdev, struct vts_edge_cmd *cmd)
{
	u32 max_y;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	struct sec_ts_edge_reject_para para;
	int para_index = 0;
	u32 resolution = 0;

	if (cmd->index >= EDGE_PARA_NUM) {
		VTD("function num is large than para num");
		return 0;
	}
	para_index = cmd->index;
	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution)
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &max_y);
	else
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &max_y);
	para.reject_height_max = ts->edge_para[para_index].reject_height_max;
	para.reject_top = ts->edge_para[para_index].reject_top;
	para.reject_buttom = ts->edge_para[para_index].reject_buttom;

	if (cmd->index == EDGE_FUNC_NAIL || cmd->index == EDGE_FUNC_VIRTUAL_KEY || 
		cmd->index == EDGE_FUNC_VIRTUAL_KEY_SINGLE || cmd->index == EDGE_FUNC_VIRTUAL_KEY_DOUBLE) {
		if (cmd->area_type != (VTS_EDGE_AREA_ABS_TYPE | VTS_EDGE_AREA_LONG_PRESS_TYPE | VTS_EDGE_AREA_CHARACTER_TYPE)) {
			VTE("app calling cannot to set other type");
			return -EINVAL;
		}

		if (cmd->height > para.reject_height_max) {
			VTE("app calling cannot set area size more than 500 px");
			return -EINVAL;
		}

		if (cmd->y < para.reject_top) {
			if (cmd->y + cmd->height < para.reject_top) {
				VTE("app set reject area in forbid zone");
				return -EINVAL;
			} else {
				cmd->height = cmd->y + cmd->height - para.reject_top;
				cmd->y = para.reject_top;
				
			}
		} else if (cmd->y + cmd->height > max_y - para.reject_buttom) {
			if (cmd->y > max_y - para.reject_buttom) {
				VTE("app set reject area in forbid zone");
				return -EINVAL;
			} else {
				cmd->height = max_y - para.reject_buttom - cmd->y;
			}
		}
	}

	return 0;
}

static int bbk_slsi_setEdgeRejectArea(struct vts_device *vtsdev, struct vts_edge_cmd *cmd)
{
	u8 set_data[8];
	int ret = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	u32 max_x;
	u32 max_y;
	static u8 grip_data[2] = {0, 0};
	u32 resolution = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &max_y);
	} else {
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &max_y);
	}
	VTI("max_x = %d, max_y = %d", max_x, max_y);

	if (ts == NULL) {
		VTE("ts is null");
		return -EINVAL;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("Touch is stopped");
		return -EIO;
	}

	if (cmd->index >= EDGE_FUNC_NUM || cmd->index < 0) {
		VTE("forbid any other calling to set the edge for function");
		return -EINVAL;
	}

	if (bbk_slsi_edge_reject_limit(vtsdev, cmd) < 0) {
		VTE("edge reject is out of limit, forbid to set");
		return -EINVAL;
	}

	if (cmd->index < EDGE_FUNC_NUM) {
		if (cmd->index == EDGE_FUNC_NAIL)
			memcpy(&ts->cmd_store[EDGE_SAVE_NAIL], cmd, sizeof(struct vts_edge_cmd));
		else if (cmd->index == EDGE_FUNC_VIRTUAL_KEY)
			memcpy(&ts->cmd_store[EDGE_SAVE_VIRTUAL_KEY], cmd, sizeof(struct vts_edge_cmd));
		else if (cmd->index == EDGE_FUNC_VIRTUAL_KEY_SINGLE || cmd->index == EDGE_FUNC_VIRTUAL_KEY_DOUBLE)
			memcpy(&ts->cmd_store[EDGE_SAVE_VIRTUAL_KEY_GESTURE], cmd, sizeof(struct vts_edge_cmd));
		else if (cmd->index == EDGE_FUNC_REPORT_POINT_LEFT || cmd->index == EDGE_FUNC_REPORT_POINT_RIGHT || 
			cmd->index == EDGE_FUNC_REPORT_POINT_DOUBLE)
			memcpy(&ts->cmd_store[EDGE_SAVE_REPORT_POINT], cmd, sizeof(struct vts_edge_cmd));
	}

	switch (cmd->area_type) {
		case (VTS_EDGE_AREA_ABS_TYPE | VTS_EDGE_AREA_LONG_PRESS_TYPE | VTS_EDGE_AREA_CHARACTER_TYPE):
			grip_data[0] = cmd->index;
			if (cmd->enable) {
				memset(set_data, 0, sizeof(set_data));
				ts->edge_app_flag &= ~cmd->index;
				grip_data[1] = 15;
			} else {
				if (cmd->x < max_x / 2) {
					set_data[0] = 1;  // left
					grip_data[1] = cmd->x;
				} else if ((cmd->x > max_x / 2) && ((cmd->width + cmd->x) <= max_x)) {
					set_data[0] = 2;  // right
					grip_data[1] = max_x - cmd->x - cmd->width;
				} else {
					VTE("invalid input argument");
					return -EPERM;
				}
				if (grip_data[1] > 15) grip_data[1] = 15;
				if (cmd->index == EDGE_FUNC_VIRTUAL_KEY_SINGLE)
					set_data[0] |= 0x20;
				else if (cmd->index == EDGE_FUNC_VIRTUAL_KEY_DOUBLE)
					set_data[0] = 0x23;
				else
					set_data[0] |= ((u8)cmd->index << 4);
				set_data[1] = (u8)((cmd->y + cmd->height / 2) >> 8) & 0xff;
				set_data[2] = (u8)(cmd->y + cmd->height / 2) & 0xff;
				set_data[3] = (u8)(cmd->height >> 8) & 0xff;
				set_data[4] = (u8)cmd->height & 0xff;
				set_data[5] = (u8)(cmd->width >> 8) & 0xff;
				set_data[6] = (u8)cmd->width & 0xff;
				if (cmd->index == EDGE_FUNC_VIRTUAL_KEY_SINGLE || cmd->index == EDGE_FUNC_VIRTUAL_KEY_DOUBLE)
					ts->edge_app_flag = EDGE_FUNC_VIRTUAL_KEY;
				else
					ts->edge_app_flag |= cmd->index;
			}
			/* select channel */
			VTD("select edge screen is %d", ts->edge_app_flag);
			ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_SELETE, &ts->edge_app_flag, 1);

			sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_SELETE, &ts->edge_app_flag, 1);
			VTD("read after select edge screen is %d", ts->edge_app_flag);

			/* set edge zone */
			VTD("edge zone set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6]);
			ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_SET, set_data, sizeof(set_data));

			sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_SET, set_data, sizeof(set_data));
			VTD("read after set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6]);

			/* set dead zone */
			VTD("dead zone grip_data = 0x%2x, 0x%2x", grip_data[0], grip_data[1]);
			ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_DEAD, grip_data, 2);

			sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_DEAD, set_data, 4);
			VTD("read after grip_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3]);

			/* set longpress zone */
			if (cmd->index == EDGE_FUNC_NAIL || cmd->index == EDGE_FUNC_VIRTUAL_KEY) {
				set_data[0] = cmd->index;
				set_data[1] = 60;
				VTD("longpress set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);
				ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				VTD("read after longpress set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);
			} else if (cmd->index == EDGE_FUNC_VIRTUAL_KEY_SINGLE || cmd->index == EDGE_FUNC_VIRTUAL_KEY_DOUBLE) {
				set_data[0] = EDGE_FUNC_VIRTUAL_KEY;
				set_data[1] = 12;
				VTD("longpress set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);
				ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				VTD("read after longpress set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);
			}
			break;

		case VTS_ACTIVE_AREA_GESTURE_TYPE:
			if (cmd ->enable) {
				VTD("x=%d, y = %d, width = %d, height = %d", cmd->x, cmd->y, cmd->height, cmd->width);
				set_data[0] = (u8)((cmd->x) >> 8) & 0xff;
				set_data[1] = (u8)cmd->x & 0xff;            //[0][1] top_left_x
				set_data[2] = (u8)((cmd->y) >> 8) & 0xff;
				set_data[3] = (u8)(cmd->y) & 0xff;          //[2][3] top_left_y
				set_data[4] = (u8)(cmd->width >> 8) & 0xff;
				set_data[5] = (u8)cmd->width & 0xff;       //[4][5] width
				set_data[6] = (u8)(cmd->height >> 8) & 0xff;
				set_data[7] = (u8)cmd->height & 0xff;        //[6][7] height
			} else {
				memset(set_data, 0, sizeof(set_data));
			}
			
			VTD("write active mode area = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
			ret = sec_ts_i2c_write(ts, SET_TS_CMD_ACTIVE_ZONE_AREA, set_data, sizeof(set_data));
			
			sec_ts_i2c_read(ts, SET_TS_CMD_ACTIVE_ZONE_AREA, set_data, sizeof(set_data));
			VTD("read after active mode area set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
			break;

		case VTS_EDGE_AREA_LONG_PRESS_TYPE:
			if (cmd->enable) {
				if (cmd->y != 0 && cmd->height != max_y) {
					grip_data[0] = 1;
					if ((cmd->x + cmd->width) < max_x / 2) {
						set_data[0] = (u8)((cmd->x + cmd->width) & 0xff);
					} else if ((cmd->x > max_x / 2) && (cmd->x <= max_x)) {
						set_data[0] = (u8)((max_x - cmd->x) & 0xff);
					} else {
						VTE("invalid input argument");
						return -EPERM;
					}
				} else {
					grip_data[0] = 2;
					if ((cmd->x + cmd->width) < max_x / 2) {
						set_data[0] = (u8)((cmd->x + cmd->width) >> 8) & 0xff;
						set_data[1] = (u8)((cmd->x + cmd->width) & 0xff);
					} else if ((cmd->x > max_x / 2) && (cmd->x <= max_x)) {
						set_data[0] = (u8)((max_x - cmd->x) >> 8) & 0xff;
						set_data[1] = (u8)((max_x - cmd->x) & 0xff);
					} else {
						VTE("invalid input argument");
						return -EPERM;
					}
				}
			} else {
				set_data[0] = 0;
				set_data[1] = 0;
			}
			if (grip_data[0] == 1) {
				set_data[1] = set_data[0];
				VTD("longpress zone with short edge, set_data = 0x%2x", set_data[1]);
				//set_data[0] = 1;
				//ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				set_data[0] = 2;
				ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 2);
				sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_LP, set_data, 4);
				VTD("read after grip_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3]);
			} else {
				VTD("set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);
				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_LONGPRESS_RANGE, set_data, 2);
			}
			break;

		case VTS_EDGE_AREA_ABS_TYPE:
			if (cmd->enable) {
				if (cmd->y != 0 && cmd->height != max_y) {
					grip_data[0] = 1;
				} else {
					grip_data[0] = 2;
				}
				if ((cmd->x + cmd->width) < max_x / 2) {
					set_data[0] = (u8)((cmd->x + cmd->width) & 0xff);
				} else if ((cmd->x > max_x / 2) && (cmd->x <= max_x)) {
					set_data[0] = (u8)((max_x - cmd->x) & 0xff);
				} else {
					VTE("invalid input argument");
					return -EPERM;
				}
			} else {
				set_data[0] = 0;
			}
			if (grip_data[0] == 1) {
				set_data[1] = set_data[0];
				VTD("dead zone with short edge, set_data = 0x%2x", set_data[1]);
				set_data[0] = 1;
				ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_DEAD, set_data, 2);
				set_data[0] = 2;
				ret = sec_ts_i2c_write(ts, SET_TS_CMD_EDGE_ZONE_DEAD, set_data, 2);
				sec_ts_i2c_read(ts, SET_TS_CMD_EDGE_ZONE_DEAD, set_data, 4);
				VTD("read after grip_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3]);
			} else {
				VTD("dead zone with long edge, set_data = 0x%2x", set_data[0]);
				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_EDGE_REJECT_RANGE, set_data, 1);
			}
			break;

		case VTS_EDGE_AREA_CHARACTER_TYPE:
			if (cmd->enable) {
				if ((cmd->y + cmd->height) < max_y / 2) {
					set_data[0] = 1;
					set_data[1] = grip_data[0] = (u8)((cmd->y + cmd->height) & 0xff);
					set_data[2] = grip_data[1];
				} else if (cmd->y > max_y / 2) {
					set_data[0] = 1;
					set_data[1] = grip_data[0];
					set_data[2] = grip_data[1] = (u8)((max_y - cmd->y) & 0xff);
					
				} else {
					VTE("invalid input argument");
					return -EPERM;
				}
				VTD("set_data = 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2]);
				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_GRIP_MODE_RANGE, set_data, 3);
			} else {
				set_data[0] = 0;
				VTD("set_data = 0x%2x", set_data[0]);
				ret = sec_ts_i2c_write(ts, SEC_TS_CMD_GRIP_MODE_RANGE, set_data, 1);
			}
			break;

		default:
			VTE("unknown grip command");
			break;
	}
	
	return ret;
}

static int bbk_slsi_setVirtualKey(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	
	if (state == 1) {
		ret = bbk_slsi_setEdgeRejectArea(vtsdev, &ts->cmd_store[EDGE_SAVE_VIRTUAL_KEY_GESTURE]);
		if (ret < 0)
			VTE("%s: send virtual key error", __func__);
		ret = bbk_slsi_setEdgeRejectArea(vtsdev, &ts->cmd_store[EDGE_SAVE_REPORT_POINT]);
		if (ret < 0)
			VTE("%s: send report point error", __func__);
	} else {
		VTI("set nothing");
	}

	return ret;
}

static int bbk_slsi_setVkLongpress(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 para = 0;

	VTI("%s: set_state = %d", __func__, state);

	para = state;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_VK_LONGPRESS, &para, 1);
	if (ret < 0)
		VTE("%s: send virtual key longpress switch error", __func__);

	return ret;
}

static int bbk_slsi_setVkActivemode(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;

	VTI("%s: set_state = %d", __func__, state);

	if (state == 1) {
		ret = sec_ts_set_active_mode(ts, 1);
		if (ret < 0) {
			VTE("Set VIRTUAL_KEY_ON fail");
		}
	} else {
		VTI("set nothing");
	}

	return ret;
}

static int bbk_slsi_set_finger_center(struct vts_device *vtsdev, int state)
 {
 	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
 	int ret = 0;	
	u32 resolution = 0;
	u8 reg_value[4];
	u32 finger_center_x;
	u32 finger_center_y;
	u8 finger_reg_value[4];

	if (ts == NULL) {
		VTE("ts is null");
		ret = -EINVAL;
		goto exit;
	}

	vts_property_get(vtsdev, VTS_PROPERTY_FINGER_CENTER, &resolution);
	VTD("resolution is %d", resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_FINGER_CENTER_X, &finger_center_x);
		vts_property_get(vtsdev, VTS_PROPERTY_FINGER_CENTER_Y, &finger_center_y);
		VTD("finger_center_x is %x", finger_center_x);
		VTD("finger_center_y is %x", finger_center_y);
		finger_reg_value[1] = finger_center_x & 0xff;
		finger_reg_value[0] = (finger_center_x >> 8) & 0xff;
		finger_reg_value[3] = finger_center_y & 0xff;
		finger_reg_value[2] = (finger_center_y >> 8) & 0xff;
	} else 
		goto exit;
	
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_FINGER_CENTER, finger_reg_value, 4);
 	if (ret < 0) {
 		VTE("Failed to write 0xBF");
		goto exit;
 	}
 
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_FINGER_CENTER, reg_value, 4);
	if (ret < 0)
		VTE("Failed to read 0xBF");
	VTD("reg_value is %x, %x, %x, %x", reg_value[0], reg_value[1], reg_value[2], reg_value[3]);

	if (ret >= 0)
		ret = 0;

exit:
	return ret;

}

static int bbk_slsi_landscape_gamemode(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret;
	u8 para = 0;

	VTI("set_state = %d", state);

	para = state;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_LANDSCAPE_GAMEMODE, &para, 1);
	if (ret < 0)
		VTE("send landscape gamemode error");

	return ret;
}

static int bbk_slsi_virtual_gamekey(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret;
	u8 set_data[5];
	int data_x = ts->gamekey_size[0];
	int data_y = ts->gamekey_size[1];

	VTI("set_state = %d", state);

	sec_ts_i2c_read(ts, SEC_TS_CMD_VIRTUAL_GAMEKEY, set_data, 5);
	VTI("read set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4]);

	if (state == 1) {
		set_data[0] = 0x11;
	} else if (state == 2) {
		set_data[0] = 0x21;
	} else if (state == 3) {
		set_data[0] = 0x31;
	} else {
		set_data[0] &= ~0x01;
	}
	set_data[1] = (u8)(data_x >> 8);
	set_data[2] = (u8)(data_x);
	set_data[3] = (u8)(data_y >> 8);
	set_data[4] = (u8)(data_y);
	VTI("need to set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4]);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_VIRTUAL_GAMEKEY, set_data, 5);
	if (ret < 0)
		VTE("send clear gamekey state error");
	
	sec_ts_i2c_read(ts, SEC_TS_CMD_VIRTUAL_GAMEKEY, set_data, 5);
	VTI("read set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4]);

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_VGK_DEADZONE, ts->vgk_deadzone, 2);
	if (ret < 0)
		VTE("send clear gamekey state error");
	sec_ts_i2c_read(ts, SEC_TS_CMD_VGK_DEADZONE, set_data, 2);
	VTI("read set_data = 0x%2x, 0x%2x", set_data[0], set_data[1]);

	return ret;
}

static int bbk_slsi_set_screen_clock_region(struct vts_device *vtsdev, int report_enable)
{	
	u8 screen_clock_report_enable[] = {0x01};
	u8 screen_clock_report_disable[] = {0x00};
	u16 start_x_coor, start_y_coor, width, height;
	u8 set_data[8];
	int ret = 0;
	struct vts_screen_clock_cmd sclock_cmd;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	if (report_enable == 0) {
		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SCREEN_CLOCK_ENABLE, screen_clock_report_disable, sizeof(screen_clock_report_disable));
		if(ret < 0) {
			VTE("failed to disable gesture report abs");
			goto exit;
		}	
		VTI("success to disable gesture report abs ");
	}
	else if (report_enable == 1) {
		vts_get_screen_clock_zone(&sclock_cmd, &vtsdev->screen_clock_zone);
		start_x_coor = sclock_cmd.x;
		start_y_coor = sclock_cmd.y;
		width = sclock_cmd.width;
		height = sclock_cmd.height;
		set_data[0] = (u8)(start_x_coor >> 8) & 0xff;
		set_data[1] = (u8)start_x_coor & 0xff;			//[0][1] top_left_x
		set_data[2] = (u8)(start_y_coor >> 8) & 0xff;
		set_data[3] = (u8)start_y_coor & 0xff;			//[2][3] top_left_y
		set_data[4] = (u8)(width >> 8) & 0xff;
		set_data[5] = (u8)width & 0xff;    //[4][5] width
		set_data[6] = (u8)(height >> 8) & 0xff;
		set_data[7] = (u8)height & 0xff;		 //[6][7] height
	
		VTD("write active mode area = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
		ret = sec_ts_i2c_write(ts, SET_TS_CMD_SCREEN_CLOCK_AREA, set_data, sizeof(set_data));
						
		sec_ts_i2c_read(ts, SET_TS_CMD_SCREEN_CLOCK_AREA, set_data, sizeof(set_data));
		VTD("read after active mode area set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);

		ret = ts->sec_ts_i2c_write(ts, SET_TS_CMD_SCREEN_CLOCK_ENABLE, screen_clock_report_enable, sizeof(screen_clock_report_enable));
		if(ret < 0) {
			VTE("failed to enable gesture report abs");
			goto exit;
		}	
		VTI("success to enable gesture report abs ");
	}
		
	if (ret >= 0)
		ret = 0;

exit:
	return ret;
}

static int sec_ts_set_screen_clock_area(struct vts_device *vtsdev, int state)
{
	u16 start_x_coor, start_y_coor, width, height;
	u8 set_data[8];
	int ret = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	struct vts_screen_clock_cmd sclock_cmd;
	vts_get_screen_clock_zone(&sclock_cmd, &vtsdev->screen_clock_zone);
	start_x_coor = sclock_cmd.x;
	start_y_coor = sclock_cmd.y;
	width = sclock_cmd.width;
	height = sclock_cmd.height;
	set_data[0] = (u8)(start_x_coor >> 8) & 0xff;
	set_data[1] = (u8)start_x_coor & 0xff;			//[0][1] top_left_x
	set_data[2] = (u8)(start_y_coor >> 8) & 0xff;
	set_data[3] = (u8)start_y_coor & 0xff;			//[2][3] top_left_y
	set_data[4] = (u8)(width >> 8) & 0xff;
	set_data[5] = (u8)width & 0xff;    //[4][5] width
	set_data[6] = (u8)(height >> 8) & 0xff;
	set_data[7] = (u8)height & 0xff;		 //[6][7] height
		
	VTD("write active mode area = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
	ret = sec_ts_i2c_write(ts, SET_TS_CMD_SCREEN_CLOCK_AREA, set_data, sizeof(set_data));
							
	sec_ts_i2c_read(ts, SET_TS_CMD_SCREEN_CLOCK_AREA, set_data, sizeof(set_data));
	VTD("read after active mode area set_data = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
	
	if (ret >= 0)
		ret = 0;
	
	return ret;
}


static int bbk_slsi_setFingerMode(struct vts_device *vtsdev, int mode)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret;
	u8 para = 0;

	VTI("set_state = %d", mode);

	para = mode;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_FOD_MODE, &para, 1);
	if (ret < 0)
		VTE("send FOD mode error");

	if (ret >= 0)
		ret = 0;

	return ret;
}

static int bbk_slsi_set_input_method(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 para = 0;
	int tx_write_flag = 0;

	vts_property_get(ts->vtsdev, VTS_PROPERTY_VIRTUAL_KEY, &tx_write_flag);
	if (tx_write_flag)
		return 0;
	para = state ? 1 : 0;
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_INPUT_METHOD, &para, 1);	//reuse 0x49 with vklongpress
	if (ret < 0)
		VTE("set input mode error");
	
	return ret;
}

int bbk_slsi_gesture_point_handle(struct vts_device *vtsdev, u8 id)
{
	struct sec_ts_data *ts = g_ts_data;
	int ret = 0;
	int i = 0;
	int report_num;
	u8 buff[40] = {0, };
	u16 gesture_point_x[10] = {0, };
	u16 gesture_point_y[10] = {0, };

	VTI("gesture point num is %d", ts->read_gesture_point_num);
	if (ts->read_gesture_point_num == 0)
		return 0;

	if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE) {
		ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, buff, 20);
	} else if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
		ret = sec_ts_i2c_read(ts, SEC_TS_CMD_GESTURECOORD, buff, 40);
	}
	if (ret < 0) {
		VTE("i2c read gesture coord failed\n");
		return ret;
	}

	VTI("buff = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
		buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7], 
		buff[8], buff[9], buff[10], buff[11], buff[12], buff[13], buff[14], buff[15], 
		buff[16], buff[17], buff[18], buff[19]);

	if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
		VTI("buff = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			buff[20], buff[21], buff[22], buff[23], buff[24], buff[25], buff[26], buff[27], 
			buff[28], buff[29], buff[30], buff[31], buff[32], buff[33], buff[34], buff[35], 
			buff[36], buff[37], buff[38], buff[39]);
	}
	
	switch(id) {
		case TOUCH_GESTURE_DOUBLTAP:
			report_num = 1;
			break;
		case TOUCH_GESTURE_DOWN:
		case TOUCH_GESTURE_LEFT:
		case TOUCH_GESTURE_RIGHT:
		case TOUCH_GESTURE_M:
			VTI("Therer is no need to report gesture point");
			return 0;
		case TOUCH_GESTURE_UP:
			report_num = 2;
			break;
		case TOUCH_GESTURE_C:
			report_num = 3;
			break;
		case TOUCH_GESTURE_W:
			report_num = 5;
			break;
		default:
			report_num = 6;
			break;
	}

	if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE) {
		for (i = 0; i < report_num; i++) {
			gesture_point_x[i] = ((buff[3 * i] << 4) | (buff[3 * i + 2] >> 4));
			gesture_point_y[i] = ((buff[3 * i + 1] << 4) | (buff[3 * i + 2] & 0x0f));
		}
	} else if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE) {
		for (i = 0; i < report_num; i++) {
			gesture_point_x[i] = ((buff[4 * i] << 8) | (buff[4 * i + 1] & 0x0f));
			gesture_point_y[i] = ((buff[4 * i + 2] << 8) | (buff[4 * i + 3] & 0x0f));
		}
	}

	ts->dbclick_x = gesture_point_x[0];
	ts->dbclick_y = gesture_point_y[0];

	if (buff[0] == GESTURE_O)  // orientation info for 'O'
		gesture_point_y[9] = buff[19];
	VTD("point_num = %d, point = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", report_num,
		gesture_point_x[0], gesture_point_y[0], gesture_point_x[1], gesture_point_y[1], 
		gesture_point_x[2], gesture_point_y[2], gesture_point_x[3], gesture_point_y[3], 
		gesture_point_x[4], gesture_point_y[4], gesture_point_x[5], gesture_point_y[5], 
		gesture_point_x[6], gesture_point_y[6], gesture_point_x[7], gesture_point_y[7], 
		gesture_point_x[8], gesture_point_y[8], gesture_point_x[9], gesture_point_y[9]);

	ret = vts_report_coordinates_set(vtsdev, gesture_point_x, gesture_point_y, report_num);
	
	return ret;
}

extern void sec_ts_print_frame(struct sec_ts_data *ts, short *min, short *max);
int sec_ts_dump_fw_data(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	unsigned int readbytes = 0xFF;
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	short min;
	short max;
	u8 type = 0x5;
	int retry = 0;
	int data_valid = 0;
	int left_event_count = 0;
	static u8 *read_event_buff = NULL;

	if (read_event_buff == NULL) {
		read_event_buff = kzalloc(sizeof(u8) * ts->event_buf_size * MAX_EVENT_COUNT, GFP_DMA);
		if (read_event_buff == NULL) {
			ret = -ENOMEM;
			goto exit;
		}
	}

	if (gpio_is_valid(ts->plat_data->irq_gpio)) {
		ret = gpio_get_value(ts->plat_data->irq_gpio);
		VTI("irq status is %s", ret ? "high" : "low");
	}
	memset(read_event_buff, 0, MAX_EVENT_COUNT * ts->event_buf_size);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ONE_EVENT, (u8 *)read_event_buff, ts->event_buf_size);
	if (ret < 0) {
		VTE("i2c read one event failed");
	}

	left_event_count = read_event_buff[7] & 0x3F;
	if (left_event_count > 0) {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_ALL_EVENT, (u8 *)(read_event_buff + ts->event_buf_size),
				sizeof(u8) * (ts->event_buf_size) * (left_event_count));
		if (ret < 0) {
			VTE("%s: i2c read one event failed\n", __func__);
		}
	}
	for (i = 0; i < (left_event_count + 1); i++) {
		VTI("%02x %02x %02x %02x %02x %02x %02x %02x", read_event_buff[i * ts->event_buf_size], read_event_buff[i * ts->event_buf_size + 1], 
			read_event_buff[i * ts->event_buf_size + 2], read_event_buff[i * ts->event_buf_size + 3], read_event_buff[i * ts->event_buf_size + 4], 
			read_event_buff[i * ts->event_buf_size + 5], read_event_buff[i * ts->event_buf_size + 6], read_event_buff[i * ts->event_buf_size + 7]);
	}

	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count * 2;

	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead) {
		VTE("%s: Read frame kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	/* set OPCODE and data type */
	ret = ts->sec_ts_i2c_write(ts, 0x70, &type, 1);
	if (ret < 0) {
		VTE("Set #5 data type failed\n");
		goto ErrorExit;
	}
	if (state)
		sec_ts_delay(150);
	else
		sec_ts_delay(50);
	do {
		/* read data */
		ret = ts->sec_ts_i2c_read(ts, 0x72, pRead, readbytes);
		if (ret < 0) {
			VTE("%s: read #5 data failed!\n", __func__);
			goto ErrorRelease;
		}
		for (i = 0; i < readbytes; i++) {
			if (pRead[i] != 0) {
				data_valid = 1;
				break;
			}
		}
	} while((++retry <= 5) && (data_valid == 0));

	memset(ts->pFrame, 0x00, readbytes);

	for (i = 0; i < readbytes; i += 2)
		ts->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);

	min = max = ts->pFrame[0];
	sec_ts_print_frame(ts, &min, &max);
	
ErrorRelease:
	ret = ts->sec_ts_i2c_write(ts, 0x70, &mode, 1);
	if (ret < 0)
		VTE("Set #5 data type failed\n");

ErrorExit:
	kfree(pRead);

exit:
	return ret;
}

int bbk_slsi_dump_fw_value(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	
	ret = bbk_slsi_dump_reg_status(ts);
	if (ret < 0)
		VTE("%s: dump fw reg failed!\n", __func__);
	
	ret = sec_ts_dump_fw_data(vtsdev, state);
	if (ret)
		VTE("%s: dump fw date failed!\n", __func__);

	return ret;
}

static int bbk_slsi_check_status_regular(struct vts_device *vtsdev)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;

	mutex_lock(&ts->device_mutex);
	if (!ts->sec_test_flag) {
		ret = bbk_slsi_dump_reg_status(ts);
		if (ret < 0)
			VTE("%s: dump fw reg failed!\n", __func__);
	}
	mutex_unlock(&ts->device_mutex);
	return ret;
}

static int bbk_slsi_set_rejection_zone(struct vts_device *vtsdev, int scene)
{
	int ret = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	u8 set_data[10];
	u8 set_data_temp[10];
	struct vts_rejection_config *config;
	struct vts_rejection_data *data;
	int i;
	int j = 0;
	if (ts == NULL) {
		VTE("ts is null");
		return -EINVAL;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("Touch is stopped");
		return -EIO;
	}

	config = vts_rejection_zone_config_get(vtsdev, scene);
	if (config == NULL)
		return -EINVAL;

	VTI("scene = %d, config_num = %d", config->scene, config->config_num);
	mutex_lock(&ts->device_mutex);
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

		if (data->type == 0x100) {
			set_data[0] = (u8)(data->x1 >> 8) & 0xff;
			set_data[1] = (u8)data->x1 & 0xff;
			set_data[2] = 0xff;
			set_data[3] = 0xff;
			set_data[4] = 0xff;
			VTI("set_data_temp =%d, %d ",set_data[0], set_data[1]);
			j = 0;
		retry:
			j++;
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_LONGPRESSZONE_RANGE, set_data, 5);
			if (ret)
				VTE("write rejection zone failed");
			sec_ts_i2c_read(ts, SEC_TS_CMD_LONGPRESSZONE_RANGE, set_data_temp, 5);
			VTD("set_data_temp = %d, %d, %d, %d, %d", set_data_temp[0], set_data_temp[1], set_data_temp[2], set_data_temp[3], set_data_temp[4]);
			if (set_data_temp[1] != set_data[1] && j < 3)
				goto retry;
			continue;
		}

		if (data->type == 0) {
			if (data->block == 0)
				set_data[0] = 0;
			else if (data->block == 1)
				set_data[0] = 1;
		} else if (data->type == 1) {
			if (data->block == 0)
				set_data[0] = 2;
			else if (data->block == 1)
				set_data[0] = 4;
		}
		if (data->mode == 1 && data->type == 0) {
			if (data->block == 2)
				set_data[0] = 6;
			else if (data->block == 3)
				set_data[0] = 1;
		} else if (data->mode == 1 && data->type == 1) {
			if (data->block == 2)
				set_data[0] = 7;
			else if (data->block == 3)
				set_data[0] = 4;
		}
		if (data->mode == 1)
			set_data[1] = 1;
		else
			set_data[1] = 0;
		set_data[2] = (u8)(data->x1 >> 8) & 0xff;
		set_data[3] = (u8)data->x1 & 0xff;
		set_data[4] = (u8)(data->y1 >> 8) & 0xff;
		set_data[5] = (u8)data->y1 & 0xff;
		set_data[6] = (u8)(data->x2 >> 8) & 0xff;
		set_data[7] = (u8)data->x2 & 0xff;
		set_data[8] = (u8)(data->y2 >> 8) & 0xff;
		set_data[9] = (u8)data->y2 & 0xff;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_EDGE_REJECT_RANGE, set_data, 10);
		if (ret)
			VTE("write rejection zone failed");
		
		if (data->type == 0 && data->block == 1 && data->mode == 1) {
			set_data[0] = 6;
			ret = sec_ts_i2c_write(ts, SEC_TS_CMD_EDGE_REJECT_RANGE, set_data, 10);
			if (ret)
				VTE("write rejection zone failed");
		}
	}
	mutex_unlock(&ts->device_mutex);

	return ret;
}

/*================BBK*===========================*/


int sec_ts_init_regulator(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int retval;
	if (gpio_is_valid(pdata->iovcc_gpio)) {    //1.8v
		retval = devm_gpio_request(&client->dev ,pdata->iovcc_gpio, "sec, tsp_iovcc");
		if (retval){
			VTE("fail to request iovcc gpio !!!");
		}
	} else {
		pdata->regulator_dvdd = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
			VTE("%s: Failed to get dvdd ONE regulator.\n", __func__);
			pdata->regulator_dvdd = regulator_get(&client->dev, "vcc_i2c_s");
			if (IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
				VTE("%s: Failed to get dvdd TWO regulator.\n", __func__);
				retval = PTR_ERR(pdata->regulator_dvdd);
				goto err_dvdd;
			}
		}

		retval = regulator_set_voltage(pdata->regulator_dvdd, 1800000,
								1800000);
		if (retval) {
			VTI("regulator set_vtg:vcc_i2c failed retval = %d", retval);
		}
	}
	
	if (gpio_is_valid(pdata->power_gpio)) {    //3v
		retval = devm_gpio_request(&client->dev, pdata->power_gpio, "sec, tsp_power");
		if (retval){
			VTE("fail to request power gpio !!!");
		}
	} else {
		//regulator_avdd = regulator_get(NULL, pdata->regulator_avdd);
		pdata->regulator_avdd = regulator_get(&client->dev, "avdd_pwr");
		if (IS_ERR_OR_NULL(pdata->regulator_avdd)) {
			VTE("%s: Failed to get avdd regulator.\n", __func__);
			retval = PTR_ERR(pdata->regulator_avdd);
			goto err_avdd;
		}
		retval = regulator_set_voltage(pdata->regulator_avdd, 3000000,
								3000000);
		if (retval) {
			VTI("regulator set_vtg:avdd_pwr failed retval = %d", retval);
		}
	}

	VTI("INIT REGULATOR");

	return retval;

err_avdd:
	if (pdata->regulator_avdd) {
		regulator_put(pdata->regulator_avdd);
		pdata->regulator_avdd = NULL;
	}
err_dvdd:
	if (pdata->regulator_dvdd) {
		regulator_put(pdata->regulator_dvdd);
		pdata->regulator_dvdd = NULL;
	}
	return retval;
}

int sec_ts_power(void *data, bool on)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)data;
	struct sec_ts_plat_data *pdata = ts->plat_data;
	
	int ret = 0;

	if (pdata->enabled == on) {
		VTI("enabled == on");
		return ret;
	}

	if (on) {
		VTI("regulator_enable");
		if (gpio_is_valid(pdata->iovcc_gpio)) {
			ret = gpio_direction_output(pdata->iovcc_gpio, 1);
			if(ret) {
				VTE("fail to set iovcc 1.8v !!!");
			}
		} else {
			if (!IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
				ret = regulator_set_voltage(pdata->regulator_dvdd, 1800000, 1800000);
				if (ret) {
					VTI("regulator set_vtg:vcc_i2c failed retval = %d", ret);
				}
				regulator_set_load(pdata->regulator_dvdd, 30000);
				ret = regulator_enable(pdata->regulator_dvdd);
				if (ret) {
					VTE("%s: Failed to enable dvdd: %d\n", __func__, ret);
					//goto out;
				}
			}
		}

		sec_ts_delay(1);

		if(gpio_is_valid(pdata->power_gpio)) {
			ret = gpio_direction_output(pdata->power_gpio, 1);
			if(ret) {
				VTE("fail to set avdd 3.0v !!!");
			}
		} else {
			if (!IS_ERR_OR_NULL(pdata->regulator_avdd)) {
				ret = regulator_set_voltage(pdata->regulator_avdd, 3000000, 3000000);
				if (ret) {
					VTI("regulator set_vtg:avdd failed retval = %d", ret);
				}
				regulator_set_load(pdata->regulator_avdd, 30000);
				ret = regulator_enable(pdata->regulator_avdd);
				if (ret) {
					VTE("%s: Failed to enable avdd: %d\n", __func__, ret);
					//goto out;
				}
			}
		}
		sec_ts_delay(1);
		gpio_set_value(pdata->reset_gpio, 1);
		if (gpio_is_valid(pdata->sda_gpio))
			gpio_set_value(pdata->sda_gpio, 1);
		if (gpio_is_valid(pdata->scl_gpio))
			gpio_set_value(pdata->scl_gpio, 1);
		msleep(5);
	} else {
		gpio_set_value(pdata->reset_gpio, 0);
		if (gpio_is_valid(pdata->sda_gpio))
			gpio_set_value(pdata->sda_gpio, 0);
		if (gpio_is_valid(pdata->scl_gpio))
			gpio_set_value(pdata->scl_gpio, 0);
		msleep(5);
		if (gpio_is_valid(pdata->power_gpio)) {
			ret = gpio_direction_output(pdata->power_gpio, 0);
			if(ret) {
				VTE("fail to shutdown power 3.0v !!!");
			}
		} else {
			if (!IS_ERR_OR_NULL(pdata->regulator_avdd)) {
				ret = regulator_disable(pdata->regulator_avdd);
				if (ret) {
					VTE("fail to disable avdd !!!");
				}
				if (regulator_count_voltages(pdata->regulator_avdd) > 0) {
					regulator_set_voltage(pdata->regulator_avdd, 0, 3000000);
					regulator_set_load(pdata->regulator_avdd, 0);
				}
			}
		}
		sec_ts_delay(4);
		if (gpio_is_valid(pdata->iovcc_gpio)) {
			ret = gpio_direction_output(pdata->iovcc_gpio, 0);
			if(ret) {
				VTE("fail to shutdown iovcc 1.8v !!!");
			}
		} else {
			if (!IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
				ret = regulator_disable(pdata->regulator_dvdd);
				if (ret) {
					VTE("fail to disable dvdd !!!");
				}
				if (regulator_count_voltages(pdata->regulator_dvdd) > 0) {
					 regulator_set_load(pdata->regulator_dvdd, 0);
					 regulator_set_voltage(pdata->regulator_dvdd, 0, 1800000);
				}
			}
		}
		sec_ts_delay(10);
	}

	pdata->enabled = on;

//out:
	if (!gpio_is_valid(pdata->power_gpio) && !IS_ERR_OR_NULL(pdata->regulator_avdd)) {
		VTI("%s: avdd:%s\n", on ? "on" : "off",
			regulator_is_enabled(pdata->regulator_avdd) ? "on" : "off");
	}
	if (!gpio_is_valid(pdata->iovcc_gpio) && !IS_ERR_OR_NULL(pdata->regulator_dvdd)) {
		VTI("%s: dvdd:%s\n", on ? "on" : "off",
			regulator_is_enabled(pdata->regulator_dvdd) ? "on" : "off");
	}
	return ret;

}

static int sec_ts_parse_dt(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_ts_plat_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	int ret = 0;	
	struct property *prop;

	prop = of_find_property(np, "samsung,power-gpio", NULL);
	if (prop && prop->length) {
		pdata->power_gpio = of_get_named_gpio_flags(np,
			"samsung,power-gpio", 0, NULL);
		VTI("dt find:samsung,power-gpio = %d", pdata->power_gpio);
	} else {
		pdata->power_gpio = -1;
	}

	prop = of_find_property(np, "samsung,iovcc-gpio", NULL);
	if (prop && prop->length) {
		pdata->iovcc_gpio = of_get_named_gpio_flags(np,
			"samsung,iovcc-gpio", 0, NULL);
		VTI("dt find:samsung,iovcc-gpio = %d", pdata->iovcc_gpio);
	} else {
		pdata->iovcc_gpio = -1;
	}

	pdata->irq_gpio = of_get_named_gpio_flags(np,
			"samsung,irq-gpio", 0,
			(enum of_gpio_flags *)&pdata->irq_type);

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
	
	VTI("%s: irq_type property:%X, %d\n", __func__,
					pdata->irq_type, pdata->irq_type);

	client->irq = gpio_to_irq(pdata->irq_gpio);
	VTI("%s: gpio : %d IRQ : %d", __func__, pdata->irq_gpio, client->irq);

	prop = of_find_property(np, "samsung,reset-gpio", NULL);
	if (prop && prop->length) {
		pdata->reset_gpio = of_get_named_gpio_flags(np,
				"samsung,reset-gpio", 0, NULL);
		ret = gpio_request_one(pdata->reset_gpio, GPIOF_DIR_OUT, "sec,tsp_reset");
		VTI("%s: reset pin property %X, %d\n",
			__func__, pdata->reset_gpio, pdata->reset_gpio);
	} else {
		pdata->reset_gpio = -1;
		VTE("%s: parsing reset pin failed\n", __func__);
	}

	prop = of_find_property(np, "samsung,sda-gpio", NULL);
	if (prop && prop->length) {
		pdata->sda_gpio = of_get_named_gpio_flags(np,
				"samsung,sda-gpio", 0, NULL);
		ret = gpio_request_one(pdata->sda_gpio, GPIOF_DIR_OUT, "sec,tsp_sda");
		VTI("%s: sda pin property %X, %d\n",
			__func__, pdata->sda_gpio, pdata->sda_gpio);
	} else {
		pdata->sda_gpio = -1;
		VTE("%s: parsing sda pin failed\n", __func__);
	}

	prop = of_find_property(np, "samsung,scl-gpio", NULL);
	if (prop && prop->length) {
		pdata->scl_gpio = of_get_named_gpio_flags(np,
				"samsung,scl-gpio", 0, NULL);
		ret = gpio_request_one(pdata->scl_gpio, GPIOF_DIR_OUT, "sec,tsp_scl");
		VTI("%s: scl pin property %X, %d\n",
			__func__, pdata->scl_gpio, pdata->scl_gpio);
	} else {
		pdata->scl_gpio = -1;
		VTE("%s: parsing scl pin failed\n", __func__);
	}

	pdata->i2c_burstmax = 32;

	pdata->power = sec_ts_power;

	pdata->always_lpmode = 0;
	pdata->bringup = 0;
	pdata->mis_cal_check = 0;
    pdata->firmware_name = NULL;
	pdata->project_name = NULL;
	pdata->model_name = NULL;

	pdata->tsp_icid = 0;
	pdata->tsp_id = 0;
	pdata->tsp_vsync = 0;

	pdata->regulator_boot_on = 0;
	pdata->support_sidegesture = 0;
	pdata->support_dex = 0;

	VTI("i2c buffer limit: %d, bringup:%d, FW:%s(%d), id:%d, mis_cal:%d dex:%d, gesture:%d\n",
		pdata->i2c_burstmax, pdata->bringup, pdata->firmware_name,
			pdata->tsp_id, pdata->tsp_icid, pdata->mis_cal_check, pdata->support_dex, pdata->support_sidegesture);

	return ret;
}
static int sec_ts_parse_dt_property(struct vts_device *vtsdev)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	struct device *dev = &ts->client->dev;
	struct device_node *np = dev->of_node;
	int i;

	parse_property_u32_with_default(np, "nail,edge_reject_height_max", (u32 *)&ts->edge_para[EDGE_NAIL].reject_height_max, 500);
	parse_property_u32_with_default(np, "nail,edge_reject_top", (u32 *)&ts->edge_para[EDGE_NAIL].reject_top, 300);
	parse_property_u32_with_default(np, "nail,edge_reject_buttom", (u32 *)&ts->edge_para[EDGE_NAIL].reject_buttom, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_height_max", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_height_max, 500);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_top", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_top, 300);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY].reject_buttom, 750);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_SINGLE].reject_buttom, 750);
	parse_property_u32_with_default(np, "vk,edge_reject_buttom", (u32 *)&ts->edge_para[EDGE_VIRTUAL_KEY_DOUBLE].reject_buttom, 750);

	for (i = 1; i < EDGE_PARA_NUM; i++) {
		VTI("edge_para_%d, height = %d, top = %d, buttom = %d", i, ts->edge_para[i].reject_height_max, ts->edge_para[i].reject_top, ts->edge_para[i].reject_buttom);
	}

	parse_property_u32_with_default(np, "virtual_gamekey_width", (u32 *)&ts->gamekey_size[0], 300);
	parse_property_u32_with_default(np, "virtual_gamekey_height", (u32 *)&ts->gamekey_size[1], 36);
	parse_property_u8_with_default(np, "virtual_gamekey_up_deadzone", (u8 *)&ts->vgk_deadzone[0], 0);
	parse_property_u8_with_default(np, "virtual_gamekey_down_deadzone", (u8 *)&ts->vgk_deadzone[1], 0);
	
	return 0;
}

int sec_ts_read_information(struct sec_ts_data *ts)
{
	unsigned char data[13] = { 0 };
	int ret;

	memset(data, 0x0, 3);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_ID, data, 3);
	if (ret < 0) {
		VTE("%s: failed to read device id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: %X, %X, %X\n",
			__func__, data[0], data[1], data[2]);
	memset(data, 0x0, 11);
	ret = sec_ts_i2c_read(ts,  SEC_TS_READ_PANEL_INFO, data, 11);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: nTX:%X, nRX:%X, rY:%d, rX:%d\n",
			__func__, data[8], data[9],
			(data[2] << 8) | data[3], (data[0] << 8) | data[1]);

	/* Set X,Y Resolution from IC information. */
	if (((data[0] << 8) | data[1]) > 0)
		ts->plat_data->max_x = ((data[0] << 8) | data[1]) - 1;

	if (((data[2] << 8) | data[3]) > 0)
		ts->plat_data->max_y = ((data[2] << 8) | data[3]) - 1;

	ts->tx_count = data[8];
	ts->rx_count = data[9];

	data[0] = 0;
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: STATUS : %X\n",
			__func__, data[0]);

	memset(data, 0x0, 4);
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, data, 4);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: TOUCH STATUS : %02X, %02X, %02X, %02X\n",
			__func__, data[0], data[1], data[2], data[3]);
	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,  (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		VTE("%s: failed to read touch functions(%d)\n",
				__func__, ret);
		return ret;
	}

	VTI("%s: Functions : %02X\n",
			__func__, ts->touch_functions);

	return ret;
}

static void sec_ts_set_input_prop(struct sec_ts_data *ts, struct input_dev *dev, u8 propbit)
{
	static char sec_ts_phys[64] = { 0 };

	snprintf(sec_ts_phys, sizeof(sec_ts_phys), "%s/input1",
			dev->name);
	dev->phys = sec_ts_phys;
	dev->id.bustype = BUS_I2C;
	dev->dev.parent = &ts->client->dev;

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SW, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_TOOL_FINGER, dev->keybit);
	//set_bit(KEY_BLACK_UI_GESTURE, dev->keybit);
#ifdef SEC_TS_SUPPORT_TOUCH_KEY
	if (ts->plat_data->support_mskey) {
		int i;

		for (i = 0 ; i < ts->plat_data->num_touchkey ; i++)
			set_bit(ts->plat_data->touchkey[i].keycode, dev->keybit);

		set_bit(EV_LED, dev->evbit);
		set_bit(LED_MISC, dev->ledbit);
	}
#endif
	if (ts->plat_data->support_sidegesture) {
		//set_bit(KEY_SIDE_GESTURE, dev->keybit);
		//set_bit(KEY_SIDE_GESTURE_RIGHT, dev->keybit);
		//set_bit(KEY_SIDE_GESTURE_LEFT, dev->keybit);
	}
	set_bit(propbit, dev->propbit);
	set_bit(KEY_HOMEPAGE, dev->keybit);

	//input_set_capability(dev, EV_SW, SW_GLOVE);

	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->plat_data->max_x, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->plat_data->max_y, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	//input_set_abs_params(dev, ABS_MT_CUSTOM, 0, 0xFFFFFFFF, 0, 0);
	if (ts->plat_data->support_mt_pressure)
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (propbit == INPUT_PROP_POINTER)
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_POINTER);
	else
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT, INPUT_MT_DIRECT);

	input_set_drvdata(dev, ts);
}


/*
static unsigned int report_lcm_id(void)
{
	int lcm_id = 0x31;
	return lcm_id;
}

static int bbk_sec_get_lcm_id(void) {
	int lcm_id;
	lcm_id = report_lcm_id();
	return lcm_id;
}
*/
static int sec_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 32;
	return 0;
}

static ssize_t sec_flash_read(struct vts_device *vtsdev, u8*udd, size_t nbytes)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	unsigned char buf[32] = {0};
	int ret = 0;
	ret = get_user_nvm_data(ts, buf);
	if (ret < 0) {
		VTE("%s: read user data failed\n", __func__);
		return ret;
	}	
	else
		memcpy(udd, buf, 15);

	return nbytes;
}

static ssize_t sec_flash_write(struct vts_device *vtsdev, u8 *udd, size_t nbytes)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	unsigned char buf[32] = {0};

	memcpy(buf, udd, 15);
	ret = set_user_nvm_data(ts, buf);
	if (ret < 0) {
		VTE("%s: wtire user data failed\n", __func__);
		return ret;
	}	
	else
		memcpy(udd, buf, 15);
	return nbytes;
}

static int bbk_slsi_set_aoi_zone(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	u8 set_data[8];
	int ret = 0;
	int x, y, width, height;

	VTI("%s: set_state = %d", __func__, state);

	x = vtsdev->aoi_cmd.aoi_left;
	y = vtsdev->aoi_cmd.aoi_top;
	width = vtsdev->aoi_cmd.aoi_right - vtsdev->aoi_cmd.aoi_left;
	height = vtsdev->aoi_cmd.aoi_bottom - vtsdev->aoi_cmd.aoi_top;

	if (state) {
		set_data[0] = (u8)(x >> 8) & 0xff;
		set_data[1] = (u8)x & 0xff;
		set_data[2] = (u8)(y >> 8) & 0xff;
		set_data[3] = (u8)y & 0xff;
		set_data[4] = (u8)(width >> 8) & 0xff;
		set_data[5] = (u8)width & 0xff;
		set_data[6] = (u8)(height >> 8) & 0xff;
		set_data[7] = (u8)height & 0xff;

		VTD("write fod active area = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_FOD_RANGE, set_data, sizeof(set_data));
	}

	VTI("set SEC_TS_CMD_FOD_ENABLE 0x%x with value %d", SEC_TS_CMD_FOD_ENABLE, state);
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_FOD_ENABLE, (u8 *)&state, 1);

	if (ret >= 0)
		ret = 0;
	
	return ret;
}

static int bbk_slsi_set_aoi_int_zone(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	u8 set_data[8];
	int ret = 0;
	int x, y, width, height;

	VTI("%s: set_state = %d", __func__, state);

	x = vtsdev->aoi_cmd.aoi_left;
	y = vtsdev->aoi_cmd.aoi_top;
	width = vtsdev->aoi_cmd.aoi_right - vtsdev->aoi_cmd.aoi_left;
	height = vtsdev->aoi_cmd.aoi_bottom - vtsdev->aoi_cmd.aoi_top;

	if (state) {
		set_data[0] = (u8)(x >> 8) & 0xff;
		set_data[1] = (u8)x & 0xff;
		set_data[2] = (u8)(y >> 8) & 0xff;
		set_data[3] = (u8)y & 0xff;
		set_data[4] = (u8)(width >> 8) & 0xff;
		set_data[5] = (u8)width & 0xff;
		set_data[6] = (u8)(height >> 8) & 0xff;
		set_data[7] = (u8)height & 0xff;

		VTD("write fod active area = 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x", set_data[0], set_data[1], set_data[2], set_data[3], set_data[4], set_data[5], set_data[6], set_data[7]);
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_FOD_RANGE, set_data, sizeof(set_data));
	}
	if (ret >= 0)
		ret = 0;
	
	return ret;
}

static int bbk_slsi_set_aoi_intpin(struct vts_device *vtsdev, int state)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 para = 0;

	VTI("%s: set_state = %d", __func__, state);

	para = state;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_AOI_INTPIN_TEST, &para, 1);
	if (ret < 0)
		VTE("%s: send virtual key longpress switch error", __func__);

	if (ret >= 0)
		ret = 0;

	return ret;
}

#define REGION_SIZE_REG 0xEB
static int bbk_slsi_set_card_region(struct vts_device *vtsdev, int enable)
{
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	unsigned char ptmp[6] = {0};
	int ret = 0;
	/*temporarily set to a fixed value*/
	unsigned int y0 = vtsdev->y0;
	unsigned int y1 = vtsdev->y1;
	unsigned int width = vtsdev->width;
	unsigned int y0_read, y1_read, width_read;

	VTI("enter");

	if (enable == 0) {
		return 0;
	}

	if (!y0 || !y1) {
		VTE("card region cfg invalid, y0 = %d y1 = %d width = %d", y0, y1, width);
		return -1;
	}

	VTI("set reg y0 = %d y1 = %d width = %d", y0, y1, width);

	ptmp[0] = 00;
	ptmp[1] = (unsigned char)((y0>>8) & 0xff);
	ptmp[2] = (unsigned char)(y0 & 0xff);
	y0 = (int)((ptmp[1]<<8) + ptmp[2]);
	VTI("before from reg y0 = %d 0x%x  0x%x 0x%x", y0, ptmp[0], ptmp[1], ptmp[2]);
	ret = sec_ts_i2c_write(ts, REGION_SIZE_REG, ptmp, 3);
	if (ret < 0)
		VTE("write region failed");

	ptmp[0] = 01;
	ptmp[1] = (unsigned char)((y1>>8) & 0xff);
    ptmp[2] = (unsigned char)(y1 & 0xff);
	VTI("before from reg y1 = %d 0x%x  0x%x 0x%x", y1, ptmp[0], ptmp[1], ptmp[2]);
	ret = sec_ts_i2c_write(ts, REGION_SIZE_REG, ptmp, 3);
	if (ret < 0)
		VTE("write region failed");

	ptmp[0] = 02;
	ptmp[1] = (unsigned char)((width>>8) & 0xff);
	ptmp[2] = (unsigned char)(width & 0xff);
	VTI("before from reg width = %d 0x%x  0x%x 0x%x", width, ptmp[0], ptmp[1], ptmp[2]);
	ret = sec_ts_i2c_write(ts, REGION_SIZE_REG, ptmp, 3);
	if (ret < 0)
		VTE("write region failed");

	ret = sec_ts_i2c_read(ts, REGION_SIZE_REG, ptmp, sizeof(ptmp));
	if (ret < 0)
		VTE("read data failed");

	y0_read = (unsigned int)((ptmp[0]<<8) + ptmp[1]);
	y1_read = (unsigned int)((ptmp[2]<<8) + ptmp[3]);
	width_read = (unsigned int)((ptmp[4]<<8) + ptmp[5]);

	if (y0_read != y0 || y1_read != y1 || width_read != width) {
		VTE("IC reg data error, after from reg y0_read = %d  y1_read = %d width_read %d 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
			y0_read, y1_read, width_read, ptmp[0], ptmp[1], ptmp[2], ptmp[3], ptmp[4], ptmp[5]);
	}

	VTI("out");
	return 0;
}

extern int samsung_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result);
extern int samsung_sensor_caliberate(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result);
extern unsigned int tp_recovery_mode;
extern unsigned int tp_survival_mode;

static const struct vts_operations sec_vts_ops = {
	//.init = ftm4_hw_init,
	.update_firmware = bbk_slsi_fw_update,
	.change_mode = bbk_slsi_mode_change,
	.get_frame = bbk_slsi_sec_get_rawordiff_data,
	.get_fw_version = bbk_slsi_get_fw_version,
	.set_charging = bbk_slsi_set_charger_bit,
	.set_rotation = bbk_slsi_setEdgeRestainSwitch,
	.set_edge_reject_area = bbk_slsi_setEdgeRejectArea,
	.set_bandstate = bbk_slsi_setBandState,
	.set_virtual_key = bbk_slsi_setVirtualKey,
	.set_vk_longpress = bbk_slsi_setVkLongpress,
	.set_vk_activemode = bbk_slsi_setVkActivemode,
	// .atSensorTest = samsung_at_sensor_test,
	//.sensor_test = samsung_sensor_test,
	.sensor_caliberate = samsung_sensor_caliberate,
	.set_auto_idle = bbk_slsi_set_auto_idle,
	.set_landscape_gamemode = bbk_slsi_landscape_gamemode,
	.set_virtual_gamekey = bbk_slsi_virtual_gamekey,
	//.process_package = bbk_slsi_process_by_package,
	//.otherInfo = ftm4_other_info,
	//.get_grip_status = get_grip_area,
	.get_ic_mode = sec_get_touch_ic_mode,
	.set_virtual_prox = sec_ts_set_callmode,
	.set_long_press = sec_ts_set_fod_detection,
	.set_finger_mode = bbk_slsi_setFingerMode,
	.set_gesture = sec_ts_set_gesture,
	.set_screen_clock_report_abs = bbk_slsi_set_screen_clock_region,
	.rom_size = sec_get_flash_size,
	.rom_read = sec_flash_read,
	.rom_write = sec_flash_write,
	.set_screen_clock_area = sec_ts_set_screen_clock_area,
	.set_finger_center = bbk_slsi_set_finger_center,
	.get_calibration_status = sec_get_vivo_special_calibration_status,
	.get_fw_resolution = sec_ts_get_fw_resolution,
	.set_rejection_zone = bbk_slsi_set_rejection_zone,
	.set_input_method = bbk_slsi_set_input_method,
	.dump = bbk_slsi_dump_fw_value,
	.check_state_sliently = bbk_slsi_check_status_regular,
	.set_aoi_zone = bbk_slsi_set_aoi_zone,
	.set_aoi_int_zone = bbk_slsi_set_aoi_int_zone,
	.set_aoi_intpin = bbk_slsi_set_aoi_intpin,
	.set_card_region = bbk_slsi_set_card_region,
	.set_game_mode = sec_slsi_set_game_mode,
	.set_high_report_rate = sec_slsi_set_report_rate,
	.set_idle_time = sec_slsi_set_idle_time,
};

static int sec_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sec_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	struct vts_device *vtsdev = NULL;
	int ret = 0;
	bool force_update = false;
	bool valid_firmware_integrity = false;
	unsigned char data[5] = { 0 };
	unsigned char deviceID[5] = { 0 };
	unsigned char result = 0;
	int i;

	VTI("%s\n", __func__);

	vtsdev = vts_device_alloc();
	if(vtsdev == NULL) {
		return -1;
	}
	
	tsp_sync_enable(1);
	
	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			VTE("%s: Failed to allocate platform data\n", __func__);
			goto error_allocate_pdata;
		}
		client->dev.platform_data = pdata;
		ret = sec_ts_parse_dt(client);
		if (ret) {
			VTE("%s: Failed to parse dt\n", __func__);
			goto error_allocate_mem;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			VTE("%s: No platform data found\n", __func__);
			goto error_allocate_pdata;
		}
	}

	if (!pdata->power) {
		VTE("%s: No power contorl found\n", __func__);
		goto error_allocate_mem;
	}

	ts = kzalloc(sizeof(struct sec_ts_data), GFP_KERNEL);
	if (!ts)
		goto error_allocate_mem;

	ts->client = client;
	ts->plat_data = pdata;
	ts->flash_page_size = SEC_TS_FW_BLK_SIZE_DEFAULT;
	ts->sec_ts_i2c_read = sec_ts_i2c_read;
	ts->sec_ts_i2c_write = sec_ts_i2c_write;
	ts->sec_ts_i2c_write_burst = sec_ts_i2c_write_burst;
	ts->sec_ts_i2c_read_bulk = sec_ts_i2c_read_bulk;
	ts->i2c_burstmax = pdata->i2c_burstmax;
#ifdef USE_POWER_RESET_WORK
	INIT_DELAYED_WORK(&ts->reset_work, sec_ts_reset_work);
	ts->reset_wakelock = vts_wakelock_register(vtsdev, "tp_reset_work");
	if (!ts->reset_wakelock) {
		vts_dev_err(vtsdev, "reset_wakelock init fail!\n");
		return -ENOMEM;
	}
#endif
	//INIT_DELAYED_WORK(&ts->work_read_info, sec_ts_read_info_work);
	g_ts_data = ts;

	i2c_set_clientdata(client, ts);

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad = input_allocate_device();
		if (!ts->input_dev_pad) {
			VTE("%s: allocate device err!\n", __func__);
			ret = -ENOMEM;
			goto err_allocate_input_dev_pad;
		}
	}

	ts->touch_count = 0;
	ts->max_z_value = 0;
	ts->min_z_value = 0xFFFFFFFF;
	ts->sum_z_value = 0;
	ts->event_buf_size = SEC_TS_EVENT_BUFF_SIZE;

	mutex_init(&ts->lock);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->eventlock);
	mutex_init(&ts->modechange);

	init_completion(&ts->resume_done);
	complete_all(&ts->resume_done);

	if (pdata->always_lpmode)
		ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

	VTI("%s: init resource\n", __func__);
	
	pdata->pinctrl = devm_pinctrl_get(&client->dev);
    if (IS_ERR(pdata->pinctrl)) {
         dev_err(&client->dev, "%s: could not get pinctrl\n", __func__);
    } else {
		sec_ts_pinctrl_lookup_state(ts);
	}

	//gpio_set_value(ts->plat_data->reset_gpio, 1);
	//sec_ts_delay(5);

	ret = sec_ts_init_regulator(client);
	if (!ret) {
		sec_ts_power(ts, true);
		sec_ts_delay(70);
	} else {
		goto error_regulator_init;
	}
	/* power enable */
	
	VTI("%s: init power\n", __func__);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	ts->external_factory = false;

	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	
	ret = sec_ts_i2c_read(ts, SEC_TS_READ_DEVICE_ID, deviceID, 5);
	if (ret < 0)
		VTE("%s: failed to read device ID(%d)\n", __func__, ret);
	else
		VTI("%s: TOUCH DEVICE ID : %02X, %02X, %02X, %02X, %02X\n", __func__,
				deviceID[0], deviceID[1], deviceID[2], deviceID[3], deviceID[4]);

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_FIRMWARE_INTEGRITY, &result, 1);
	if (ret < 0) {
		VTE("%s: failed to integrity check (%d)\n", __func__, ret);
	} else {
		if (result & 0x80) {
			valid_firmware_integrity = true;
		} else if (result & 0x40) {
			valid_firmware_integrity = false;
			VTE("%s: invalid firmware (0x%x)\n", __func__, result);
		} else {
			valid_firmware_integrity = false;
			VTE("%s: invalid integrity result (0x%x)\n", __func__, result);
		}
	}

	ret = sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &data[0], 1);
	if (ret < 0) {
		VTE("%s: failed to read sub id(%d)\n",
				__func__, ret);
	} else {
		ret = sec_ts_i2c_read(ts, SEC_TS_READ_TS_STATUS, &data[1], 4);
		if (ret < 0) {
			VTE("%s: failed to touch status(%d)\n",
					__func__, ret);
		}
	}
	VTI("%s: TOUCH STATUS : %02X || %02X, %02X, %02X, %02X\n",
			__func__, data[0], data[1], data[2], data[3], data[4]);

	if (data[0] == SEC_TS_STATUS_BOOT_MODE)
		force_update = true;
	else if ((data[0] == SEC_TS_STATUS_APP_MODE && data[2] == TOUCH_SYSTEM_MODE_FLASH) ||
			(valid_firmware_integrity == false))
		force_update = true;
	else
		force_update = false;
	
#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	ret = sec_ts_firmware_update_on_probe(ts, force_update);
	if (ret < 0)
		VTE("calibrate on probe is failed, ret = %d", ret);
#else
	VTI("%s: fw update on probe disabled!\n", __func__);
#endif

	ret = sec_ts_read_information(ts);
	if (ret < 0) {
		VTE("%s: fail to read information 0x%x\n", __func__, ret);
		goto err_init;
	}

	ts->touch_functions |= SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		VTE("%s: Failed to send touch func_mode command", __func__);

	/* Sense_on */
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		VTE("%s: fail to write Sense_on\n", __func__);
		goto err_init;
	}

	ts->pFrame = kzalloc(ts->tx_count * ts->rx_count * 2, GFP_KERNEL);
	if (!ts->pFrame) {
		ret = -ENOMEM;
		goto err_allocate_frame;
	}

	sec_ts_i2c_read(ts, SEC_TS_READ_POINT_BYTE_FLAG, &ts->report_byte_flag, 1);
	VTI("report point byte flag is %d", ts->report_byte_flag);
	if (ts->report_byte_flag == SEC_TS_REPORT_8_BYTE)
		ts->event_buf_size = SEC_TS_EVENT_BUFF_SIZE;
	if (ts->report_byte_flag == SEC_TS_REPORT_16_BYTE)
		ts->event_buf_size = SEC_TS_EVENT_DOUBLE_BUFF_SIZE;

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad->name = "sec_touchpad";
		sec_ts_set_input_prop(ts, ts->input_dev_pad, INPUT_PROP_POINTER);
	}
	ts->dex_name = "";

	if (ts->plat_data->support_dex) {
		ret = input_register_device(ts->input_dev_pad);
		if (ret) {
			VTE("%s: Unable to register %s input device\n", __func__, ts->input_dev_pad->name);
			goto err_input_pad_register_device;
		}
	}

	/* need remove below resource @ remove driver */
	sec_ts_raw_device_init(ts);

	device_init_wakeup(&client->dev, true);

	if (0)
		INIT_DELAYED_WORK(&ts->ghost_check, sec_ts_check_rawdata);
#if 0
	schedule_delayed_work(&ts->work_read_info, msecs_to_jiffies(50));

	schedule_delayed_work(&ts->ghost_check, msecs_to_jiffies(100));
#endif

	for (i = 0; i < EDGE_SAVE_NUM; i++) {
		ts->cmd_store[i].enable = 1;
	}
	
	g_ts_data->vtsdev = vtsdev;
	vtsdev->ops = &sec_vts_ops;
	vts_parse_dt_property(vtsdev, client->dev.of_node);
	vts_set_drvdata(vtsdev, g_ts_data);
	ret = vts_register_driver(vtsdev);
	if (ret) {
		VTE("register vts driver failed!, ret = %d\n", ret);
	}

	sec_ts_parse_dt_property(vtsdev);

	ret = vts_interrupt_register(vtsdev, client->irq, sec_ts_irq_thread, IRQ_TYPE_LEVEL_LOW, ts);
	if (ret < 0) {
		VTE("%s: Unable to request threaded irq %d\n", __func__, ret);
		goto err_irq;
	}
	
	ret = bbk_slsi_set_finger_center(vtsdev, 0);
	if (ret < 0) {
		VTE("%s: Unable to set finger center %d\n", __func__, ret);
		goto err_irq;
	}

	ret = sec_ts_get_special_cal_status_on_probe(vtsdev);
	if (ret < 0) {
		VTE("%s: Unable to special force calibration %d\n", __func__, ret);
		goto err_irq;
	}
	
	sec_ts_i2c_read(ts, SEC_TS_CMD_SELF_SENSING_MODE, &ts->self_sensing_mode, 1);
	VTI("read self sensing mode is %d", ts->self_sensing_mode);
	sec_ts_i2c_read(ts, SEC_TS_READ_CALIBRATION_REPORT, ts->ic_info, 8);
	VTI("read F1 is %d, %d, %d, %d, %d, %d, %d, %d", ts->ic_info[0], ts->ic_info[1], ts->ic_info[2], ts->ic_info[3],
		ts->ic_info[4], ts->ic_info[5], ts->ic_info[6], ts->ic_info[7]);

	VTI("%s: request_irq done = %d\n", __func__, client->irq);
	
	ts->probe_done = true;
	VTI("%s: done\n", __func__);

	return 0;

	/* need to be enabled when new goto statement is added */
#if 0
	sec_ts_fn_remove(ts);
	free_irq(client->irq, ts);
#endif

err_irq:
	vts_unregister_driver(vtsdev);
	if (ts->plat_data->support_dex) {
		input_unregister_device(ts->input_dev_pad);
		ts->input_dev_pad = NULL;
	}
err_input_pad_register_device:
	kfree(ts->pFrame);
err_allocate_frame:
err_init:
	sec_ts_power(ts, false);
error_regulator_init:
	if ((!gpio_is_valid(pdata->power_gpio)) && pdata->regulator_avdd) {
		regulator_put(pdata->regulator_avdd);
		pdata->regulator_avdd = NULL;
	}
	if ((!gpio_is_valid(pdata->iovcc_gpio)) && pdata->regulator_dvdd) {
		regulator_put(pdata->regulator_dvdd);
		pdata->regulator_dvdd = NULL;
	}
	if (ts->plat_data->support_dex) {
		if (ts->input_dev_pad)
			input_free_device(ts->input_dev_pad);
	}
err_allocate_input_dev_pad:
#ifdef USE_POWER_RESET_WORK
	vts_wakelock_unregister(ts->reset_wakelock);
	ts->reset_wakelock = NULL;
#endif
	kfree(ts);
error_allocate_mem:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->tsp_id))
		gpio_free(pdata->tsp_id);
	if (gpio_is_valid(pdata->tsp_icid))
		gpio_free(pdata->tsp_icid);

error_allocate_pdata:
	if (vtsdev != NULL) {
		vts_device_free(vtsdev);
		vtsdev = NULL;
	}
	if (ret == -ECONNREFUSED)
		sec_ts_delay(100);
	ret = -ENODEV;
	VTE("%s: failed(%d)\n", __func__, ret);

	return ret;
}

void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc:%d tc:%d v:%02X%02X cal:%02X id(%d,%d) p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->tspid_val,
					ts->tspicid_val, ts->coord[i].palm_count);
			
			ktime_get_ts64(&ts->time_released[i]);
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;
}

void sec_ts_locked_release_all_finger(struct sec_ts_data *ts)
{
	int i;

	mutex_lock(&ts->eventlock);

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action == SEC_TS_COORDINATE_ACTION_MOVE)) {

			ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
			VTI("%s: [RA] tID:%d mc: %d tc:%d, v:%02X%02X, cal:%X id(%d,%d), p:%d\n",
					__func__, i, ts->coord[i].mcount, ts->touch_count,
					ts->plat_data->img_version_of_ic[2],
					ts->plat_data->img_version_of_ic[3],
					ts->cal_status, ts->tspid_val, ts->tspicid_val, ts->coord[i].palm_count);

			ktime_get_ts64(&ts->time_released[i]);
			if (ts->time_longest < (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec))
				ts->time_longest = (ts->time_released[i].tv_sec - ts->time_pressed[i].tv_sec);
		}

		ts->coord[i].mcount = 0;
		ts->coord[i].palm_count = 0;

	}

	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;

	mutex_unlock(&ts->eventlock);
}

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
			reset_work.work);
	int ret;
	struct vts_device *vtsdev = ts->vtsdev;

	__pm_stay_awake(ts->reset_wakelock);

	if (vtsdev == NULL) {
		VTE("vtsdev is null");
		goto END_RELEAX;
	}

	if (ts->reset_is_on_going) {
		VTE("%s: reset is ongoing\n", __func__);
		goto END_RELEAX;
	}

	disable_irq(ts->client->irq);
	mutex_lock(&ts->modechange);
	//wake_lock(&ts->wakelock);

	vts_abnormal_reset_collect(TOUCH_VCODE_UNEXPECTED_RESET_EVENT);

	ts->reset_is_on_going = true;
	VTI("%s\n", __func__);

	sec_ts_stop_device(ts, true);

	sec_ts_delay(30);

	ret = sec_ts_start_device(ts, true);
	if (ret < 0) {
		VTE("%s: failed to reset, ret:%d\n", __func__, ret);
		ts->reset_is_on_going = false;
		cancel_delayed_work(&ts->reset_work);
		if (ts->reset_wakelock->active)
			__pm_relax(ts->reset_wakelock);
		__pm_stay_awake(ts->reset_wakelock);
		if(ts->reset_count < RESET_MAX) {
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
			ts->reset_count++;
		} else {
			VTE("tp reset over retry limit %d", ts->reset_count -1);
		}

		//wake_unlock(&ts->wakelock);

		goto END_UNLOCK;
	}

	if (ts->last_state== VTS_ST_NORMAL) {
		VTI("stay in normal mode, do nothing");
	} else if (ts->last_state == VTS_ST_GESTURE) {
		ret = sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
		if (ret < 0) {
			VTE("%s: failed to reset, ret:%d\n", __func__, ret);
			ts->reset_is_on_going = false;
			cancel_delayed_work(&ts->reset_work);
			if (ts->reset_wakelock->active)
				__pm_relax(ts->reset_wakelock);
			__pm_stay_awake(ts->reset_wakelock);
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
			//wake_unlock(&ts->wakelock);
			goto END_UNLOCK;
		}
		sec_ts_clear_status(ts);
		bbk_slsi_setEdgeRejectArea(vtsdev, &ts->cmd_store[EDGE_SAVE_VIRTUAL_KEY_GESTURE]);
		bbk_slsi_setEdgeRejectArea(vtsdev, &ts->cmd_store[EDGE_SAVE_REPORT_POINT]);
	} else {
		sec_ts_stop_device(ts, true);
		sec_ts_clear_status(ts);
	}
	
	vts_reset(vtsdev);
	
	ts->reset_is_on_going = false;

	//wake_unlock(&ts->wakelock);

END_UNLOCK:
	mutex_unlock(&ts->modechange);
	enable_irq(ts->client->irq);
END_RELEAX:
	if (ts->reset_wakelock->active)
		__pm_relax(ts->reset_wakelock);
	return;
}
#endif
#if 0
static void sec_ts_read_info_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
			work_read_info.work);
	char para = TO_TOUCH_MODE;
	int ret;

	VTI("%s\n", __func__);
#if 0
	/* run self-test */
	disable_irq(ts->client->irq);
	execute_selftest(ts, false);
	enable_irq(ts->client->irq);

	VTI("%s: %02X %02X %02X %02X\n",
		__func__, ts->ito_test[0], ts->ito_test[1]
		, ts->ito_test[2], ts->ito_test[3]);
#endif
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		 VTE("%s: Failed to set\n", __func__);

	sec_ts_delay(350);

	//input_log_fix();

	sec_ts_run_rawdata_all(ts);
	ts->info_work_done = true;
}
#endif

int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode)
{
	int ret;
	int retrycnt = 0;
	//u8 data;
	char para = 0;

	VTI("%s: %s(%X)\n", __func__,
			mode == TO_LOWPOWER_MODE ? "ENTER" : "EXIT", ts->lowpower_mode);
/*
	if (mode) {

		data = (ts->lowpower_mode & SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE, &data, 1);
		if (ret < 0) {
			VTE("%s: Failed to set\n", __func__);
			goto i2c_error;
		}
	}
*/
retry_pmode:
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_POWER_MODE, &mode, 1);
	if (ret < 0) {
		VTE("%s: failed\n", __func__);
		goto i2c_error;
	}

	sec_ts_delay(50);

	ret = sec_ts_i2c_read(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0) {
		VTE("%s: read power mode failed!\n", __func__);
		goto i2c_error;
	} else {
		VTI("%s: power mode - write(%d) read(%d)\n", __func__, mode, para);
	}

	if (mode != para) {
		retrycnt++;
		if (retrycnt < 5) {
			goto retry_pmode;
		} else {
			VTE("set mode failed and retry %d times, reset IC to recovery work", retrycnt);
			if (ts->probe_done && !ts->reset_is_on_going)
				schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
		}
	}

	if (mode) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0) {
			VTE("%s: i2c write clear event failed\n", __func__);
			goto i2c_error;
		}
	}

	sec_ts_locked_release_all_finger(ts);

	sec_ts_enable_irq_wake(ts, mode);
	
	if (mode == TO_LOWPOWER_MODE)
		ts->power_status = SEC_TS_STATE_LPM;
	else
		ts->power_status = SEC_TS_STATE_POWER_ON;

i2c_error:
	VTI("%s: end %d\n", __func__, ret);

	return ret;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);
	int ret;

	if (!ts->info_work_done) {
		VTE("%s not finished info work\n", __func__);
		return 0;
	}

	mutex_lock(&ts->modechange);

	ts->input_closed = false;

	VTI("%s\n", __func__);

	if (ts->power_status == SEC_TS_STATE_LPM) {
#ifdef USE_RESET_EXIT_LPM
		schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#else
		sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
#endif
	} else {
		ret = sec_ts_start_device(ts, true);
		if (ret < 0)
			VTE("%s: Failed to start device\n", __func__);
	}

	/* because edge and dead zone will recover soon */
	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	mutex_unlock(&ts->modechange);
	
	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);

	if (!ts->info_work_done) {
		VTE("%s not finished info work\n", __func__);
		return;
	}

	mutex_lock(&ts->modechange);

	ts->input_closed = true;

	VTI("%s\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work(&ts->reset_work);
	if (ts->reset_wakelock->active)
		__pm_relax(ts->reset_wakelock);
		
#endif

	if (ts->lowpower_mode) {
		sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
	} else {
		sec_ts_stop_device(ts, true);
	}

	mutex_unlock(&ts->modechange);
}
#endif

static int sec_ts_remove(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	VTI("%s\n", __func__);

	//cancel_delayed_work_sync(&ts->work_read_info);
	//flush_delayed_work(&ts->work_read_info);

	//cancel_delayed_work_sync(&ts->ghost_check);
	//flush_delayed_work(&ts->ghost_check);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts->vtsdev);
	VTI("%s: irq disabled\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);
	vts_wakelock_unregister(ts->reset_wakelock);
	ts->reset_wakelock = NULL;

	VTI("%s: flush queue\n", __func__);

#endif
	vts_unregister_driver(ts->vtsdev);
	vts_device_free(ts->vtsdev);

	device_init_wakeup(&client->dev, false);
	//wake_lock_destroy(&ts->wakelock);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	device_destroy(ts->sec_class, 0);
	class_destroy(ts->sec_class);

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev_pad = NULL;
	ts->plat_data->power(ts, false);

	if (gpio_is_valid(ts->plat_data->irq_gpio))
		gpio_free(ts->plat_data->irq_gpio);
	if (gpio_is_valid(ts->plat_data->reset_gpio))
		gpio_free(ts->plat_data->reset_gpio);
	if (gpio_is_valid(ts->plat_data->tsp_id))
		gpio_free(ts->plat_data->tsp_id);
	if (gpio_is_valid(ts->plat_data->tsp_icid))
		gpio_free(ts->plat_data->tsp_icid);

	if ((!gpio_is_valid(ts->plat_data->power_gpio)) && ts->plat_data->regulator_avdd) {
		regulator_put(ts->plat_data->regulator_avdd);
		ts->plat_data->regulator_avdd = NULL;
	}
	if ((!gpio_is_valid(ts->plat_data->iovcc_gpio)) && ts->plat_data->regulator_dvdd) {
		regulator_put(ts->plat_data->regulator_dvdd);
		ts->plat_data->regulator_dvdd = NULL;
	}

	mutex_destroy(&ts->lock);
	mutex_destroy(&ts->device_mutex);
	mutex_destroy(&ts->i2c_mutex);
	mutex_destroy(&ts->eventlock);
	mutex_destroy(&ts->modechange);

	kfree(ts->pFrame);
	kfree(ts);
	return 0;
}

static void sec_ts_shutdown(struct i2c_client *client)
{
	struct sec_ts_data *ts = i2c_get_clientdata(client);

	VTI("%s\n", __func__);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts->vtsdev);
	VTI("%s: irq disabled\n", __func__);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);

	VTI("%s: flush queue\n", __func__);

#endif

	device_init_wakeup(&client->dev, false);
	//wake_lock_destroy(&ts->wakelock);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev_pad = NULL;
	ts->plat_data->power(ts, false);
	if ((!gpio_is_valid(ts->plat_data->power_gpio)) && ts->plat_data->regulator_avdd) {
		regulator_put(ts->plat_data->regulator_avdd);
		ts->plat_data->regulator_avdd = NULL;
	}
	if ((!gpio_is_valid(ts->plat_data->iovcc_gpio)) && ts->plat_data->regulator_dvdd) {
		regulator_put(ts->plat_data->regulator_dvdd);
		ts->plat_data->regulator_dvdd = NULL;
	}
}

int sec_ts_stop_device(struct sec_ts_data *ts, bool hardware)
{
	u8 status;
	int ret = 0;
	
	VTI("stop by %s\n", hardware == true ? "hardware" : "software");

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		VTE("%s: already power off\n", __func__);
		goto out;
	}
	disable_irq(ts->client->irq);

	sec_ts_unlocked_release_all_finger(ts);

	if (hardware) {
		sec_ts_pinctrl_configure(ts, false);
		ts->plat_data->power(ts, false);
	} else {
		status = 1;
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_OFF, &status, 1);
		if (ret < 0) {
			VTE("%s: failed\n", __func__);
			goto out;
		}
		sec_ts_pinctrl_configure(ts, false);
	}

	ts->power_status = SEC_TS_STATE_POWER_OFF;

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int sec_ts_start_device(struct sec_ts_data *ts, bool hardware)
{
	int ret = -1;
	u8 wait_for_ready_flag = 0;
	u8 freq_index = 0;
	int tx_write_flag = 0;

	VTI("start by %s\n",hardware == true ? "hardware" : "software");

	sec_ts_pinctrl_configure(ts, true);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_ON && hardware) {
		VTE("%s: already power on\n", __func__);
		goto out;
	}
	
	sec_ts_unlocked_release_all_finger(ts);

	ts->power_status = SEC_TS_STATE_POWER_ON;

	if (hardware) {
		ts->plat_data->power(ts, true);
		sec_ts_delay(70);
	}

	ts->touch_noise_status = 0;

	ret = sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	if (ret < 0) {
		VTE("%s: Failed to wait_for_ready\n", __func__);
		wait_for_ready_flag = 1;
		vts_hsync_state_collect(TOUCH_VCODE_HSYNC_EVENT);
		goto err;
	}

	vts_property_get(ts->vtsdev, VTS_PROPERTY_TX_FREQ_WRITE, &tx_write_flag);
	if (tx_write_flag) {
		freq_index = 0x01 | (ts->tx_freq_index << 4);
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_FREQ_SEND, &freq_index, 1);
		if (ret < 0) {
			VTE("Failed to write SEC_TS_CMD_FREQ_SEND 0x48");
		}
	}

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

	if (ts->flip_enable) {
		ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_COVERTYPE, &ts->cover_cmd, 1);
		if (ret < 0)
			goto err;

		ts->touch_functions = ts->touch_functions | SEC_TS_BIT_SETFUNC_COVER;
		VTI("%s: cover cmd write type:%d, mode:%x, ret:%d\n",
				__func__, ts->touch_functions, ts->cover_cmd, ret);
	} else {
		ts->touch_functions = (ts->touch_functions & (~SEC_TS_BIT_SETFUNC_COVER));
		VTI("%s: cover open, not send cmd\n", __func__);
	}

	ts->touch_functions = ts->touch_functions | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, (u8 *)&ts->touch_functions, 2);
	if (ret < 0) {
		VTE("%s: Failed to send touch function command\n", __func__);
		goto err;
	}

	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	if (ts->dex_mode) {
		VTI("%s: set dex mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_DEX_MODE, &ts->dex_mode, 1);
		if (ret < 0) {
			VTE("%s: failed to set dex mode %x\n", __func__, ts->dex_mode);
			goto err;
		}
	}

	if (ts->brush_mode) {
		VTI("%s: set brush mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_BRUSH_MODE, &ts->brush_mode, 1);
		if (ret < 0) {
			VTE("%s: failed to set brush mode\n", __func__);
			goto err;
		}
	}

	if (ts->touchable_area) {
		VTI("%s: set 16:9 mode\n", __func__);
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA, &ts->touchable_area, 1);
		if (ret < 0) {
			VTE("%s: failed to set 16:9 mode\n", __func__);
			goto err;
		}
	}

err:
	/* Sense_on */
	if (wait_for_ready_flag == 1) {
		ret = sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
		if (ret < 0)
			VTE("Failed to change normal mode");
	}

	ret = sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		VTE("%s: fail to write Sense_on\n", __func__);

	enable_irq(ts->client->irq);
	
	

out:
	mutex_unlock(&ts->device_mutex);
	return ret;
}

#ifdef CONFIG_PM
static int sec_ts_pm_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_ts_pm_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	return 0;
}
#endif

#ifdef CONFIG_PM
static struct dev_pm_ops sec_ts_dev_pm_ops = {
	.suspend = sec_ts_pm_suspend,
	.resume = sec_ts_pm_resume,
};
#endif

static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_I2C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sec_ts_id);

#ifdef CONFIG_OF
static struct of_device_id sec_ts_match_table[] = {
	{ .compatible = "samsung,y761",},
	{ },
};
MODULE_DEVICE_TABLE(of, sec_ts_match_table);
#else
#define sec_ts_match_table NULL
#endif

static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SEC_TS_I2C_NAME,
		.of_match_table = sec_ts_match_table,
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};

static const int ic_number[1] = {VTS_IC_SEC_Y761};
module_vts_driver(sec_y761, ic_number, i2c_add_driver(&sec_ts_driver), i2c_del_driver(&sec_ts_driver));

