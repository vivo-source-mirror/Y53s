/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include "synaptics_tcm_core.h"
//#include <linux/vts_core.h>

#define SYSFS_DIR_NAME "vivo_interface"

#define INTF_REPORT_TIMEOUT_100MS 30

#define UDD_DATA_SIZE 16

struct vivo_intf_hcd {
	/* handler for synaptiucs tcm device */
	struct syna_tcm_hcd *tcm_hcd;
	/* flag to indicate the error out during the process */
	bool err_flag;

	/* for report image using */
	int *report_data;
	int report_size;
	unsigned char report_rows;
	unsigned char report_cols;
	unsigned char report_type;
	bool report_is_ready;

	/* flag for lpwg mode */
	bool is_lpwg;
};

DECLARE_COMPLETION(vivo_intf_remove_complete_v2);

static struct vivo_intf_hcd *intf_hcd;

/*
 * requested APIs defined in VIVO's specification
 * history:
 *    2017.Dec  -  reference from vivo spec v3.0
 *                 support: API. 1, 2, 3, 4, 5, 6, 8, 11
 *                 not support: API. 7, 10, 12
 *
 */

/* 1. function to get a raw/delta data image */
static int bbk_xxx_get_rawordiff_data(struct vts_device *vts_data, enum vts_frame_type type, short *data, int size);
/* 2. function to do firmware update */
static int bbk_xxx_fw_update(struct vts_device *vts_data, const struct firmware *firmware);
/* 3. function to enable or disable the idle mode */
static int bbk_xxx_idleEnableOrDisable(struct vts_device *vts_data, int state);
/* 4. function to read/write the udd configration */
//static ssize_t bbk_xxx_readUdd(struct vts_device *vts_data, u8 *imei, size_t nbytes);
//static ssize_t bbk_xxx_writeUdd(struct vts_device *vts_data, u8 *imei, size_t nbytes);
/* 5. function to switch the normal/sleppmode */
static int bbk_xxx_mode_change(struct vts_device *vts_data, int which);
/* 6. function to get the firmware packrat */
static int bbk_xxx_get_fw_version(struct vts_device *vts_data, u64 *version);
/* 7. function to get the gesture point, not support */
/* 8. function to control the charger bit */
static int bbk_xxx_set_charger_bit(struct vts_device *vts_data, int state);
/* 9. function to perform production test, APK */
/*10. function to setup the panel rotation, not support */
/*11. function to get the debug message from fw, not support */
static int bbk_xxx_set_edge_bit(struct vts_device *vts_data, int on);

/*
 * helper function to set the report type
 * input:
 *    0 = raw image
 *    1 = delta image
 *
 * return:
 *    true = valid report ; false = invalid report
 */
static bool intf_helper_set_report_type(int input)
{
	bool is_valid = false;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	switch (input) {
	case VTS_FRAME_MUTUAL_RAW: /* raw image */
	case VTS_FRAME_MUTUAL_SELF_RAW:
		intf_hcd->report_type = REPORT_RAW;
		is_valid = true;
		break;
	case VTS_FRAME_MUTUAL_DELTA: /* delta image */
	case VTS_FRAME_MUTUAL_SELF_DELTA:
		intf_hcd->report_type = REPORT_DELTA;
		is_valid = true;
		break;
	case 2: /* baseline */
		LOGE(tcm_hcd->pdev->dev.parent, "not support baseline image\n");
		is_valid = false;
		break;
	default:
		LOGE(tcm_hcd->pdev->dev.parent,
			"unknown input [0:raw/1:delta] (input = %d)\n", input);
		is_valid = false;
		break;
	}

	return is_valid;
}
/*
 * helper function to enable/disable the report stream
 * input:
 *    true = enable / false = disable
 * return:
 *    0 = succeed ; otherwise, failure
 */
static int intf_helper_enable_report(bool enable)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char command;

	if (enable)
		command = CMD_ENABLE_REPORT;
	else
		command = CMD_DISABLE_REPORT;

	retval = tcm_hcd->write_message(tcm_hcd,
					command,
					&intf_hcd->report_type,
					1,
					&tcm_hcd->resp.buf,
					&tcm_hcd->resp.buf_size,
					&tcm_hcd->resp.data_length,
					0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to write command %s\n", STR(command));
		return -EINVAL;
	}

	return 0;
}
/*
 * helper function to enable/disable the touch report stream
 * input:
 *    true = enable / false = disable
 * return:
 *    0 = succeed ; otherwise, failure
 */
static int intf_helper_enable_touch_report(bool enable)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char command;
	unsigned char report_type = 0x11;

	if (enable)
		command = CMD_ENABLE_REPORT;
	else
		command = CMD_DISABLE_REPORT;

	retval = tcm_hcd->write_message(tcm_hcd,
					command,
					&report_type,
					1,
					&tcm_hcd->resp.buf,
					&tcm_hcd->resp.buf_size,
					&tcm_hcd->resp.data_length,
					0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to write command %s\n", STR(command));
		return -EINVAL;
	}

	return 0;
}
#if 0
/*
 * helper function to get the dynamic config
 * input:
 *      id = field id of dynamic config
 *   value = current setting
 * return:
 *    0 = succeed ; otherwise, failure
 */
static int intf_helper_get_dynamic_config(unsigned char id,
		unsigned short *value)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf = (unsigned char)id;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_DYNAMIC_CONFIG,
			&out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to write command %s\n",
				STR(CMD_GET_DYNAMIC_CONFIG));
		goto exit;
	}

	if (resp_length < 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid data length\n");
		retval = -EINVAL;
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;

}
#endif
/*
 * helper function to set the dynamic config
 * input:
 *      id = field id of dynamic config
 *   value = value for writing
 * return:
 *    0 = succeed ; otherwise, failure
 */
static int intf_helper_set_dynamic_config(unsigned char id,
		unsigned short value)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char out_buf[3];
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	LOGI(tcm_hcd->pdev->dev.parent,
				"%s: id=%d, value=%d", __func__, id, value);
	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf[0] = (unsigned char)id;
	out_buf[1] = (unsigned char)value;
	out_buf[2] = (unsigned char)(value >> 8);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_DYNAMIC_CONFIG,
			out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to write command %s\n",
				STR(CMD_SET_DYNAMIC_CONFIG));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}




/*
 * vivo api 1 - function to get a raw/delta data image
 *
 * input:
 *    which  - 0 = raw image
 *             1 = delta image
 *             2 = baseline
 *    *data  - an integer array containing (Tx rows * Rx columns)
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_get_rawordiff_data(struct vts_device *vts_data, enum vts_frame_type type, short *data, int size)
{
	int retval = -EINVAL;
	struct syna_tcm_hcd *tcm_hcd = vts_get_drvdata(vts_data);
	unsigned char timeout_count = 0;
	short *p_data_16;
	struct syna_tcm_buffer buf;
	int i, j;
	int size_mutual = 0;
	int size_mutual_self = 0; 

	if (!data) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid parameter, data buffer\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {

		intf_hcd->err_flag = true;

		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid app status (id_info.mode = 0x%x) (app_status = 0x%x)\n",
				tcm_hcd->id_info.mode, tcm_hcd->app_status);
		retval =  -EINVAL;
		goto exit;
	}
	if (intf_helper_enable_touch_report(false) < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to disable the requested touch report\n");
	}

	if (!intf_helper_set_report_type(type)) {
		retval = -EINVAL;
		LOGE(tcm_hcd->pdev->dev.parent, "failed to set report type\n");
		goto exit;
	}

	intf_hcd->report_is_ready = false;

	intf_hcd->report_rows =
		le2_to_uint(tcm_hcd->app_info.num_of_image_rows);
	intf_hcd->report_cols =
		le2_to_uint(tcm_hcd->app_info.num_of_image_cols);
	LOGI(tcm_hcd->pdev->dev.parent,
				"Rows=%d, Cols=%d\n", intf_hcd->report_rows, intf_hcd->report_cols);
	size_mutual = intf_hcd->report_rows * intf_hcd->report_cols;
	size_mutual_self = size_mutual + intf_hcd->report_rows + intf_hcd->report_cols;

	INIT_BUFFER(buf, false);

	/* send tcm command - to enable the report */
	if (intf_helper_enable_report(true) < 0) {
		retval = -EINVAL;
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to enable the requested report\n");
		goto exit;
	}

	/* waiting for the completion of requested report */
	do {
		if (timeout_count == INTF_REPORT_TIMEOUT_100MS)
			break;

		msleep(100);
		timeout_count++;
	} while (!intf_hcd->report_is_ready);

	if (timeout_count == INTF_REPORT_TIMEOUT_100MS) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"timeout waiting for a report image\n");

		intf_helper_enable_report(false); /* close the report stream */

		retval = -EINVAL;
		goto exit;
	}

	/* clone the request report data */
	LOCK_BUFFER(buf);

	buf.buf = kzalloc(tcm_hcd->report.buffer.data_length, GFP_KERNEL);
	if (!buf.buf) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to allocate temporary buffer\n");
		UNLOCK_BUFFER(buf);

		intf_helper_enable_report(false); /* close the report stream */

		RELEASE_BUFFER(buf);
		retval = -EINVAL;
		goto exit;
	}
	buf.buf_size = tcm_hcd->report.buffer.data_length;
	buf.data_length = 0;

	memcpy((void *)buf.buf,
			(const void *)tcm_hcd->report.buffer.buf,
			buf.buf_size);
	buf.data_length = tcm_hcd->report.buffer.data_length;

	UNLOCK_BUFFER(buf);
	if (intf_helper_enable_touch_report(true) < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to enable the requested touch report\n");
	}

	/* to disable the report stream */
	if (intf_helper_enable_report(false) < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to disable the report\n");
		RELEASE_BUFFER(buf);
		return -EINVAL;
	}

	/* move data to the output buffer */
	p_data_16 = (short *)buf.buf;
	for (i = 0; i < intf_hcd->report_rows; i++) {
		for (j = 0; j < intf_hcd->report_cols; j++) {
			data[i * intf_hcd->report_cols + j] = (int)*p_data_16;

			p_data_16++;
		}
	}

	if (type == VTS_FRAME_MUTUAL_SELF_RAW || type == VTS_FRAME_MUTUAL_SELF_DELTA) {
		for (i = size_mutual; i < size_mutual_self; i++) {
			data[i] = (unsigned int)*p_data_16 & 0xffff;
			p_data_16++;
		}
	}

	retval = 0;
	RELEASE_BUFFER(buf);

exit:
	return retval;
}


/*
 * vivo api 2 - function to perform firmware update
 *
 * input:
 *    struct firmware - firmware binary
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_fw_update(struct vts_device *vts_data, const struct firmware *firmware)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	if (!firmware) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid parameter, firmware binary\n");
		return -EINVAL;
	}

	LOGN(tcm_hcd->pdev->dev.parent,
				"firmware image size = %d\n",
				(unsigned int)firmware->size);

	retval = reflash_wrapper_run_fw_update(firmware->data,
					(unsigned int)firmware->size);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to do firmware update\n");
		return -EINVAL;
	}

	return retval;
}


/*
 * vivo api 3 - function to enable or disable the doze(idle) mode
 *
 * input:
 *    state - 1: enable
 *            0: disable
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_idleEnableOrDisable(struct vts_device *vts_data, int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (0 == state)?0:1;

	VTI("set high report rate %d", !value);
	retval = intf_helper_set_dynamic_config(DC_HIGH_REPORT_RATE, !value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_HIGH_REPORT_RATE\n");
		return -EINVAL;
	}

	return retval;
}

#if 1
static int bbk_xxx_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = UDD_DATA_SIZE;
	return 0;
}

/*
 * vivo api 4.1 - function to read the udd data
 *
 * input:
 *    char * - buffer to keep the udd data
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static ssize_t bbk_xxx_readUdd(struct vts_device *vts_data, u8 *imei, size_t nbytes)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	//VTI("there something wrong of this interface, just return temp");
	//return 0;

	retval = reflash_wrapper_read_oem_data(imei, UDD_DATA_SIZE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to read oem data\n");
		return -EINVAL;
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"udd data: %s\n", imei);

	return nbytes;
}


/*
 * vivo api 4.2 - function to write the udd data
 *
 * input:
 *    char * - data for writing
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static ssize_t bbk_xxx_writeUdd(struct vts_device *vts_data, u8 *imei, size_t nbytes)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	//VTI("there something wrong of this interface, just return temp");
	//return 0;

	retval = reflash_wrapper_write_oem_data(imei, UDD_DATA_SIZE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to write oem data\n");
		return -EINVAL;
	}

	return nbytes;
}

#endif

static void syna_tcm_hw_reset(struct syna_tcm_hcd *tcm_hcd)
{
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	mutex_lock(&tcm_hcd->io_ctrl_mutex);
	if (bdata->reset_gpio >= 0) {
		VTI("hw reset");
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);
}

//extern int mdss_dsi_panel_reset_and_powerctl(int enable);
static int resume_exception_recovery(struct syna_tcm_hcd *tcm_hcd);
/*
 * vivo api 5 - function to switch the normal/sleep mode
 *
 * input:
 *    which - 0: normal mode
 *            1: sleep mode
 *            2: lpwg mode
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_mode_change(struct vts_device *vts_data, int which)
{
	static int last_state = VTS_ST_NORMAL;
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	VTI("====start====, which:%d (0:Normal, 1:Sleep, 2:Gesture)", which);

	tcm_hcd->switch_to_mode = which;
	switch (which) {
	case VTS_ST_NORMAL:
		// power on
		vts_report_release(vts_data);
		tcm_hcd->enable_irq(tcm_hcd, false);
		if (last_state == VTS_ST_SLEEP)
			tcm_hcd->power_on(tcm_hcd);
		if (last_state == VTS_ST_GESTURE) {
			syna_tcm_hw_reset(tcm_hcd);
		}
		tcm_hcd->enable_irq(tcm_hcd, true);
		msleep(30);
		tcm_hcd->resume(&tcm_hcd->pdev->dev);
		if (0)
			resume_exception_recovery(tcm_hcd);
		break;
	case VTS_ST_SLEEP:
		vts_report_release(vts_data);
		if (last_state == VTS_ST_GESTURE) {
			tcm_hcd->resume(&tcm_hcd->pdev->dev);
			VTD("SLEEP  :Resume end, Suspend start");
		}
		tcm_hcd->enable_irq(tcm_hcd, false);
		tcm_hcd->suspend(&tcm_hcd->pdev->dev);	
		// power off
		tcm_hcd->power_off(tcm_hcd);
		break;

	case VTS_ST_GESTURE:
		if (last_state == VTS_ST_SLEEP) {
			// power on
			tcm_hcd->power_on(tcm_hcd);
			tcm_hcd->enable_irq(tcm_hcd, true);
			msleep(30);
		}
		tcm_hcd->resume(&tcm_hcd->pdev->dev);
		VTD("GESTURE:Resume end, Suspend start");
		tcm_hcd->suspend(&tcm_hcd->pdev->dev);
	
		break;

	default:
		LOGE(tcm_hcd->pdev->dev.parent,
			"unknown selection (%d)\n", which);
		return -EINVAL;
	}
	last_state = which;
	return retval;
}

/*
 * vivo api 6 - function to get the firmware packrat
 *
 * input:
 *    which - 0: fw version
 *            1: config version
 * return:
 *    packrat number.
 *    if not support, return 0;
 *    return (-1) when error
 */
static int bbk_xxx_get_fw_version(struct vts_device *vts_data, u64 *version)
{
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	*version = (int)(tcm_hcd->app_info.customer_config_id[3]);

	return 0;
}


/*
 * vivo api 8 - function to control the charger bit
 *
 * input:
 *    state - 1: connected
 *            0: dis-connected
 * return:
 *    succeed- 0
 *    fail   - (-1)
 */
extern int bbk_gl_reset_tag_v2;
static int bbk_xxx_set_charger_bit(struct vts_device *vts_data, int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (0 == state)?0:1;

	retval = intf_helper_set_dynamic_config(DC_CHARGER_CONNECTED, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_CHARGER_CONNECTED\n");
		retval = (-1);
		return retval;
	}
	VTI("====sucess====, value:%d (0:OFF, 1:ON)", value);

	return retval;
}

static int bbk_xxx_set_edge_bit(struct vts_device *vts_data, int on)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (1 == on) ? 0 : 1;

	retval = intf_helper_set_dynamic_config(DC_ROTATE_OR_HORIZONTAL, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_ROTATE_OR_HORIZONTAL\n");
		retval = (-1);
		return retval;
	}
	VTI("====sucess====, state:%d (0:Left Horizontal, 1:Vertical, 2:Right Horizontal)", on);
	if (on == 0) {
		retval = intf_helper_set_dynamic_config(DC_DAKZONE_ENABLE_BIT, 0x03);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to config DC_DAKZONE_ENABLE_BIT\n");
			retval = (-1);
			return retval;
		}
	} else if (on == 1) {
		retval = intf_helper_set_dynamic_config(DC_DAKZONE_ENABLE_BIT, 0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to config DC_DAKZONE_ENABLE_BIT\n");
			retval = (-1);
			return retval;
		}
	} else if (on == 2) {
		retval = intf_helper_set_dynamic_config(DC_DAKZONE_ENABLE_BIT, 0x0C);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to config DC_DAKZONE_ENABLE_BIT\n");
			retval = (-1);
			return retval;
		}
	}
	return 0;
}

static int syna_set_virtual_prox(struct vts_device *vtsdev, int enable)
{
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (1 == enable) ? 0 : 1;

	tcm_hcd->virtual_prox_enable = enable;
	
	retval = intf_helper_set_dynamic_config(DC_NO_DOZE, !value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_NO_DOZE\n");
		goto exit;
	}

	retval = intf_helper_set_dynamic_config(DC_FD_DISABLE, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_FD_ENABLE\n");
		goto exit;
	}
	VTI("====sucess==== set FD state %d", value);

exit:
	return retval;
}

static int syna_tcm_set_gpio(struct syna_tcm_hcd *tcm_hcd, int gpio,
		bool config, int dir, int state)
{
	int retval;
	char label[16];

	if (config) {
		snprintf(label, 16, "tcm_gpio_%d\n", gpio);

		retval = gpio_request(gpio, label);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to request GPIO %d\n",
					gpio);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set GPIO %d direction\n",
					gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return 0;
}

static int syna_tcm_config_gpio(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (bdata->irq_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio,
				true, 0, 0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure interrupt GPIO\n");
			goto err_set_gpio_irq;
		}
	}

	if (bdata->power_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure power GPIO\n");
			goto err_set_gpio_power;
		}
	}

	if (bdata->vddi_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->vddi_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure vddi GPIO\n");
			goto err_set_gpio_vddi;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio,
				true, 1, bdata->reset_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure reset GPIO\n");
			goto err_set_gpio_reset;
		}
	}
	
	return 0;

err_set_gpio_reset:
	if (bdata->vddi_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->vddi_gpio, false, 0, 0);
err_set_gpio_vddi:
	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);
err_set_gpio_power:
	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

err_set_gpio_irq:
	return retval;
}

extern irqreturn_t syna_tcm_isr(int irq, void *data);

static int syna_tcm_irq_register(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	//struct vts_device *vtsdev = platform_get_drvdata(tcm_hcd->pdev);
//#if 0
	if (tcm_hcd->irq < 0) {
		VTE("irq flags is invalid !");
		goto interrupt_register_err;
	}
	retval = request_irq(tcm_hcd->irq, syna_tcm_isr, bdata->irq_flags, PLATFORM_DRIVER_NAME, tcm_hcd);
	if (retval < 0) {
		VTE("Failed to create interrupt thread\n");
		goto interrupt_register_err;
	}
//#endif
#if 0
	retval = vts_interrupt_register(vtsdev, tcm_hcd->irq, 
				syna_tcm_isr, bdata->irq_flags, tcm_hcd);
	if (retval < 0) {
		VTE("Failed to create interrupt thread\n");
		goto interrupt_register_err;
	}
#endif
	tcm_hcd->irq_enabled = true;
	return retval;

interrupt_register_err:
	if (retval < 0) {
#ifdef FALL_BACK_ON_POLLING
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(POLLING_DELAY_MS));
		tcm_hcd->do_polling = true;
		retval = 0;
#endif
	}

	return retval;
}

extern int syna_tcm_regulator_init_v2(struct syna_tcm_hcd *tcm_hcd);
static int bbk_xxx_hw_init(struct vts_device *vts_data)
{
	struct syna_tcm_hcd *tcm_hcd = vts_get_drvdata(vts_data);
	int retval;
	int retry = 0;
	struct syna_tcm_module_pool *module_pool = &tcm_hcd->mod_pool;

	retval = syna_tcm_regulator_init_v2(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to init regulator\n");
		goto exit;
	}

	retval = syna_tcm_config_gpio(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to configure GPIO's\n");
		goto err_config_gpio;
	}

	tcm_hcd->set_pinctrl(tcm_hcd, false);

	retval = tcm_hcd->power_on(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to power on\n");
		goto err_power_on;
	}

	
	syna_tcm_hw_reset(tcm_hcd);

	syna_tcm_irq_register(tcm_hcd);
	retval = tcm_hcd->enable_irq(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enable interrupt\n");
		goto err_enable_irq;
	}
if (0) {
	do {
		retval = tcm_hcd->reset(tcm_hcd, false);
		retry++;
	} while (retval < 0 && retry < 3);
}

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		tcm_hcd->init_okay = false;
		tcm_hcd->watchdog.run = false;
		tcm_hcd->update_watchdog(tcm_hcd, false);
		tcm_hcd->enable_irq(tcm_hcd, false);
#ifndef KEEP_DRIVER_ON_ERROR
		goto err_reset;
#endif
	} else {
		tcm_hcd->init_okay = true;
		tcm_hcd->update_watchdog(tcm_hcd, true);
	}

#ifdef REPORT_NOTIFIER
	tcm_hcd->notifier_thread = kthread_run(syna_tcm_report_notifier,
			tcm_hcd, "syna_tcm_report_notifier");
	if (IS_ERR(tcm_hcd->notifier_thread)) {
		retval = PTR_ERR(tcm_hcd->notifier_thread);
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create and run tcm_hcd->notifier_thread\n");
		goto err_reset;
	}
#endif

	retval = queue_work(module_pool->workqueue, &module_pool->work);
	if (retval < 0) {
		goto err_queue_module;
	}

	return 0;

err_queue_module:
#ifdef REPORT_NOTIFIER
	kthread_stop(tcm_hcd->notifier_thread);
#endif

#ifndef KEEP_DRIVER_ON_ERROR
err_reset:
#endif

err_enable_irq:
	if (tcm_hcd->bus_reg)
		regulator_disable(tcm_hcd->bus_reg);
	if (tcm_hcd->pwr_reg)
		regulator_disable(tcm_hcd->pwr_reg);
err_power_on:
	if (tcm_hcd->hw_if->bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, tcm_hcd->hw_if->bdata->power_gpio, false, 0, 0);

	if (tcm_hcd->hw_if->bdata->vddi_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, tcm_hcd->hw_if->bdata->vddi_gpio, false, 0, 0);

	if (tcm_hcd->hw_if->bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, tcm_hcd->hw_if->bdata->irq_gpio, false, 0, 0);

	if (tcm_hcd->hw_if->bdata->reset_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, tcm_hcd->hw_if->bdata->reset_gpio, false, 0, 0);
err_config_gpio:
	if (tcm_hcd->bus_reg)
		regulator_put(tcm_hcd->bus_reg);
	if (tcm_hcd->pwr_reg)
		regulator_put(tcm_hcd->pwr_reg);

exit:
	return retval;
}


/*
 *
 *
 *
 */
static int resume_exception_recovery(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	if (bbk_gl_reset_tag_v2) {
		VTI("******************xxx do bbk sw_reset**************");
		retval = tcm_hcd->reset(tcm_hcd, false);
		if (retval < 0) {
			VTI("bbk sw_reset fail");
		}
		bbk_gl_reset_tag_v2 = 0;
	}
	return retval;
}

static int bbk_get_fw_resolution(struct vts_device *vtsdev, int enable)
{
	struct syna_tcm_hcd *tcm_hcd = vts_get_drvdata(vtsdev);
	if (!enable) {
		VTI("log switch not open !");
		return 0;
	}
	vtsdev->fw_x = le2_to_uint(tcm_hcd->app_info.max_x);
	vtsdev->fw_y = le2_to_uint(tcm_hcd->app_info.max_y);
	VTI("fw resolution is %d * %d", vtsdev->fw_x, vtsdev->fw_y);
	return 0;
}


static int syna_set_fingerprint_enable(struct vts_device *vtsdev, int enable)
{
	u8 retval = 0;

	retval = intf_helper_set_dynamic_config(DC_FINGERPRINT_ENABLE, enable);
	if (retval < 0) {
		VTE("fail to set finger unlock !");
	} else {
		VTI("====sucess====, value:%d (0:OFF, 1:ON)", enable);
	}

	return retval;
}

static const int gesture_bit[] = {
	VTS_GESTURE_DCLICK,
	VTS_GESTURE_UP,
	VTS_GESTURE_O,
	0,			 //screen clock
	VTS_GESTURE_V,
	1,
	0,
	0,
	VTS_GESTURE_C,
	VTS_GESTURE_E,
	VTS_GESTURE_W,
	VTS_GESTURE_M,
	0,
	0,
	VTS_GESTURE_F,
	VTS_GESTURE_A		
};

static int syna_set_gesture(struct vts_device *vtsdev, int gesture_state)
{
	int retval = 0;
	int i;
	u32 value = 0;
	for (i = 0; i < ARRAY_SIZE(gesture_bit); i++) {
		if (i == LETTER_GESTURE_BIT) {
			value |= (1 << 5);
			continue;
		}
		if (gesture_state & gesture_bit[i])
			value |= (1 << i);
		else
			value &= ~(1 << i);
	}
	if (gesture_state & (VTS_GESTURE_M | VTS_GESTURE_DOWN | VTS_GESTURE_LR)) {
		value |= (1 << 1);
	}
	if (vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN)) {
		value |= (1 << 3);
	}
	retval = intf_helper_set_dynamic_config(DC_GESTURE_MASK, value);
	if (retval < 0) {
		VTE("fail to set gesture bit 0x%x !", value);
	} else {
		VTI("====sucess====, value:0x%x", value);
	}
	return retval;
}

static int syna_set_screen_clock_area(struct vts_device *vtsdev ,int state)
{
	int retval = 0;
	struct vts_screen_clock_cmd clock_area;
	vts_get_screen_clock_zone(&clock_area, &vtsdev->screen_clock_zone);
	retval = intf_helper_set_dynamic_config(DC_AOD_DOUBLE_TAP_X, clock_area.x);
	retval = intf_helper_set_dynamic_config(DC_AOD_DOUBLE_TAP_Y, clock_area.y);
	retval = intf_helper_set_dynamic_config(DC_AOD_DOUBLE_TAP_W, clock_area.width);
	retval = intf_helper_set_dynamic_config(DC_AOD_DOUBLE_TAP_H, clock_area.height);
	if (retval >= 0) {
		VTI("success to set AOD area: x:%d y:%d w:%d h:%d", 
			clock_area.x, clock_area.y, clock_area.width, clock_area.height);
	} else {
		VTE("fail to set AOD area !");
	}
	return retval;
}

static int syna_set_finger_mode(struct vts_device *vtsdev, int mode)
{
	int retval = 0;
	retval = intf_helper_set_dynamic_config(DC_FG_ENTRY_SPECIAL_MODE_CONTROL, mode);
	if (retval < 0) {
		VTE("fail to set finger mode with %d !", mode);
	} else {
		VTI("====sucess====, value:%d (0:OFF, 1:ON)", mode);
	}
	return retval;
}

const struct vts_operations syna_vts_ops = {
	.init = bbk_xxx_hw_init,
	.get_frame = bbk_xxx_get_rawordiff_data,
	.get_fw_version = bbk_xxx_get_fw_version,
	.set_rotation = bbk_xxx_set_edge_bit,
	.set_charging = bbk_xxx_set_charger_bit,
	.set_auto_idle = bbk_xxx_idleEnableOrDisable,
	.rom_size = bbk_xxx_get_flash_size,
	.rom_read = bbk_xxx_readUdd,
	.rom_write = bbk_xxx_writeUdd,
	.change_mode = bbk_xxx_mode_change,
	.update_firmware = bbk_xxx_fw_update,
	.set_virtual_prox = syna_set_virtual_prox,
	.get_fw_resolution = bbk_get_fw_resolution,
	.set_long_press = syna_set_fingerprint_enable,
	.set_gesture = syna_set_gesture,
	.set_screen_clock_area = syna_set_screen_clock_area,
	.set_finger_mode = syna_set_finger_mode,
};

#if 0
int vivo_operations_get(struct vts_device *vtsdev)
{
	if (vtsdev == NULL) {
		VTE("failed to get vivo descriple struct");
		return -EIO;
	}

	vtsdev->ops = &bbk_vts_ops;

	return 0;
}
#endif

/*
 * module initialization
 * to allocate the struct vivo_intf_hcd, then create the sysfs files
 */
static int vivo_intf_init(struct syna_tcm_hcd *tcm_hcd)
{
	VTI("vivo_intf_init start !");
	LOGN(tcm_hcd->pdev->dev.parent, "+\n");

	intf_hcd = kzalloc(sizeof(struct vivo_intf_hcd), GFP_KERNEL);
	if (!intf_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to allocate memory for vivo_intf_hcd\n");
		return -ENOMEM;
	}

	intf_hcd->tcm_hcd = tcm_hcd;
	intf_hcd->err_flag = false;
	intf_hcd->report_data = NULL;
	intf_hcd->is_lpwg = false;
	intf_hcd->report_cols = 0;
	intf_hcd->report_rows = 0;

	return 0;
}

/*
 * to remove the module and sysfs files as well
 */
static int vivo_intf_remove(struct syna_tcm_hcd *tcm_hcd)
{
	LOGN(tcm_hcd->pdev->dev.parent, "+\n");

	if (!intf_hcd)
		goto exit;

	kfree(intf_hcd);
	intf_hcd = NULL;

exit:
	complete(&vivo_intf_remove_complete_v2);

	return 0;
}

/*
 * call by synaptics_tcm_core.c when a tcm message is dispatched
 */
static int vivo_intf_syncbox(struct syna_tcm_hcd *tcm_hcd)
{
	if (!intf_hcd)
		return 0;
	/* once to receive a requested report */
	if (tcm_hcd->report.id == intf_hcd->report_type)
		intf_hcd->report_is_ready = true;

	return 0;
}


/*
 * module definition, vivo_intf_module
 */

static struct syna_tcm_module_cb vivo_intf_module = {
	.type = TCM_VIVO_INTERFACE,
	.init = vivo_intf_init,
	.remove = vivo_intf_remove,
	.syncbox = vivo_intf_syncbox,
	.asyncbox = NULL,
	.reset = NULL,
	.suspend = NULL,
	.resume = NULL,
};

int vivo_intf_module_init(void)
{
	return syna_tcm_add_module_v2(&vivo_intf_module, true);
}

void vivo_intf_module_exit(void)
{
	syna_tcm_add_module_v2(&vivo_intf_module, false);

	wait_for_completion(&vivo_intf_remove_complete_v2);

	return;
}


/*static int __init vivo_intf_module_init(void)
{
	return syna_tcm_add_module_v2(&vivo_intf_module, true);
}

static void __exit vivo_intf_module_exit(void)
{
	syna_tcm_add_module_v2(&vivo_intf_module, false);

	wait_for_completion(&vivo_intf_remove_complete_v2);

	return;
}

late_initcall(vivo_intf_module_init);
module_exit(vivo_intf_module_exit);*/

