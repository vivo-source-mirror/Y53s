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
#include "synaptics_tcm_core.h"
#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_ts_function.h>

#define SYSFS_DIR_NAME "vivo_interface"

#define FIRMWARE_IMG_NAME "syna_fw.img"

#define INTF_REPORT_TIMEOUT_100MS 30

#define UDD_DATA_SIZE 15

struct vivo_intf_hcd {
	/* handler for synaptiucs tcm device */
	struct syna_tcm_hcd *tcm_hcd;
	/* to create the sysfs */
	struct kobject *sysfs_dir;
	/* flag to indicate the error out during the process */
	bool err_flag;

	/* for report image using */
	int *report_data;
	unsigned char report_rows;
	unsigned char report_cols;
	unsigned char report_type;
	bool report_is_ready;

	/* flag for lpwg mode */
	bool is_lpwg;
};

DECLARE_COMPLETION(vivo_intf_remove_complete);

static struct vivo_intf_hcd *intf_hcd;

SHOW_STORE_PROTOTYPE(intf, get_rawordiff_data)
//STORE_PROTOTYPE(intf, fw_update)
SHOW_STORE_PROTOTYPE(intf, idle_enable_disable)
//SHOW_STORE_PROTOTYPE(intf, udd_data)
STORE_PROTOTYPE(intf, mode_change)
SHOW_PROTOTYPE(intf, fw_version)
SHOW_STORE_PROTOTYPE(intf, charger_bit)
//SHOW_PROTOTYPE(intf, fw_version_in_file)
SHOW_STORE_PROTOTYPE(intf, edge_bit)

static struct device_attribute *attrs[] = {
	ATTRIFY(get_rawordiff_data),
	//ATTRIFY(fw_update),
	ATTRIFY(idle_enable_disable),
	//ATTRIFY(udd_data),
	ATTRIFY(mode_change),
	ATTRIFY(fw_version),
	ATTRIFY(charger_bit),
	//ATTRIFY(fw_version_in_file),
	ATTRIFY(edge_bit),
};

/*
 * requested APIs defined in VIVO's specification
 * history:
 *    2017.Dec  -  reference from vivo spec v3.0
 *                 support: API. 1, 2, 3, 4, 5, 6, 8, 11
 *                 not support: API. 7, 10, 12
 *
 */

/* 1. function to get a raw/delta data image */
static int bbk_xxx_get_rawordiff_data(int which, int *data);
/* 2. function to do firmware update */
//static int bbk_xxx_fw_update(const struct firmware *fw);
/* 3. function to enable or disable the idle mode */
static int idleEnableOrDisable(int state);
/* 4. function to read/write the udd configration */
//static int bbk_xxx_readUdd(unsigned char *udd);
//static int bbk_xxx_writeUdd(unsigned char *udd);
/* 5. function to switch the normal/sleppmode */
static int bbk_xxx_mode_change(int which);
/* 6. function to get the firmware packrat */
static int bbk_xxx_get_fw_version(int which);
/* 7. function to get the gesture point, not support */
/* 8. function to control the charger bit */
static int bbk_xxx_set_charger_bit(int state);
/* 9. function to perform production test, APK */
/*10. function to setup the panel rotation, not support */
/*11. function to get the fw version from the image file */
//static int bbk_xxx_get_header_file_version(int which, const unsigned char *fw);
/*12. function to get the debug message from fw, not support */

static int vivo_td4330_set_edge_bit (int state);

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
	case 0: /* raw image */
		intf_hcd->report_type = REPORT_RAW;
		is_valid = true;
		break;
	case 1: /* delta image */
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
					NULL,
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
					NULL,
					0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to write command %s\n", STR(command));
		return -EINVAL;
	}

	return 0;
}

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
			NULL,
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
			NULL,
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
#if 0
/*
 * helper function to enable/disable the sleep mode
 * input:
 *   enable = true / false
 * return:
 *    0 = succeed ; otherwise, failure
 */
static int intf_helper_set_sleep(bool enable)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char command;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	command = (enable) ?
		CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			command,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				(enable)  ?
				STR(CMD_ENTER_DEEP_SLEEP) :
				STR(CMD_EXIT_DEEP_SLEEP));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}
#endif

/*
 * sysfs name  - get_rawordiff_data
 * description - funciton will call bbk_xxx_get_rawordiff_data to get
 *               a report image
 * usage       -
 *               $ echo [0/1/2] > get_rawordiff_data
 */
static ssize_t intf_sysfs_get_rawordiff_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	int report_size;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	intf_hcd->err_flag = false;

	intf_hcd->report_rows =
		le2_to_uint(tcm_hcd->app_info.num_of_image_rows);
	intf_hcd->report_cols =
		le2_to_uint(tcm_hcd->app_info.num_of_image_cols);
	report_size = intf_hcd->report_rows * intf_hcd->report_cols;

	/* allocate the buffer for one report data */
	intf_hcd->report_data = kzalloc(report_size*sizeof(int), GFP_KERNEL);
	if (!intf_hcd->report_data) {

		intf_hcd->err_flag = true;

		retval =  -ENOMEM;
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to allocate mem for intf_hcd->report_data\n");
		goto exit;
	}

	/* call VIVO's API to get the requested report */
	retval = bbk_xxx_get_rawordiff_data((int)input, intf_hcd->report_data);
	if (retval < 0) {
		intf_hcd->err_flag = true;

		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to get the requested image (type: %x)\n",
				input);
		goto exit;
	}

	retval = count;

exit:
	return retval;
}
/*
 * sysfs name  - get_rawordiff_data
 * description - funciton will show the image data onto the stdout
 * usage       -
 *               $ cat get_rawordiff_data
 */
static ssize_t intf_sysfs_get_rawordiff_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int row, col;
	int i, j;
	int cnt;
	int count = 0;
	int *p_data;

	if (!intf_hcd->report_data)
		return snprintf(buf, PAGE_SIZE,
				"\nerror: report data is not allocated\n\n");

	if (intf_hcd->err_flag) {
		kfree(intf_hcd->report_data);
		intf_hcd->report_data = NULL;

		return snprintf(buf, PAGE_SIZE,
				"\nerror: unable to get the requested image\n\n");
	}

	/* print out the report data */
	row = intf_hcd->report_rows;
	col = intf_hcd->report_cols;
	cnt = snprintf(buf, PAGE_SIZE - count,
					"\nreport type = 0x%x (row = %d, column = %d)\n\n",
					intf_hcd->report_type, row, col);
	buf += cnt;
	count += cnt;

	p_data = &intf_hcd->report_data[0];
	for (i = 0; i < row; i++) {
		for (j = 0; j < col; j++) {
			cnt = snprintf(buf, PAGE_SIZE - count,
						"%-4d ", *p_data);
			buf += cnt;
			count += cnt;

			p_data++;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	/* release the allocated buffer */
	kfree(intf_hcd->report_data);
	intf_hcd->report_data = NULL;

	return count;
}
#if 0
/*
 * sysfs name  - fw_update
 * description - funciton will call bbk_xxx_fw_update to perform
 *               fw update
 * usage       - users have to put the syna_fw.img into the filesystem
 *               then, run following command to start the fw updating
 *               $ echo 1 > fw_update
 */
static ssize_t intf_sysfs_fw_update_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	const struct firmware *fw_entry;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	/* get the firmware binary */
	retval = request_firmware(&fw_entry, FIRMWARE_IMG_NAME,
				tcm_hcd->pdev->dev.parent);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to request %s\n",
				FIRMWARE_IMG_NAME);
		return -EINVAL;
	}

	LOGD(tcm_hcd->pdev->dev.parent,
			"firmware image size = %d\n",
			(unsigned int)fw_entry->size);

	/* call VIVO's API to perform fw update */
	retval = bbk_xxx_fw_update(fw_entry);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to do fw update\n");
		goto exit;
	}

	retval = count;

exit:
	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}

	return retval;
}
#endif
/*
 * sysfs name  - idle_enable_disable
 * description - funciton will call idleEnableOrDisable to
 *               enable/disable the doze mode
 * usage       - to enable
 *               $ echo 1 > idle_enable_disable
 *               to disable
 *               $ echo 0 > idle_enable_disable
 */
static ssize_t intf_sysfs_idle_enable_disable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	retval = idleEnableOrDisable(input);
	if (retval < 0)
		return -EINVAL;

	retval = count;

	return retval;
}
/*
 * sysfs name  - idle_enable_disable
 * description - funciton will show the current setting of doze mde
 * usage       -
 *               $ cat idle_enable_disable
 */
static ssize_t intf_sysfs_idle_enable_disable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned short value;

	retval = intf_helper_get_dynamic_config(DC_NO_DOZE, &value);
	if (retval < 0) {
		return snprintf(buf, PAGE_SIZE,
				"\nerror: unable to get the setting of idle mode\n\n");
	}

	return snprintf(buf, PAGE_SIZE, "\nidle mode: %d\n\n", value);
}
#if 0
/*
 * sysfs name  - udd_data
 * description - funciton will call bbk_xxx_writeUdd to
 *               write the udd data
 * usage       - to write the appointed testing string
 *               echo 1 > udd_data
 */
static ssize_t intf_sysfs_udd_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char buffer[UDD_DATA_SIZE] = {"synaptics.test"};
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	retval =  bbk_xxx_writeUdd(buffer);
	if (retval < 0)
		return -EINVAL;

	retval = count;

	return retval;
}
/*
 * sysfs name  - udd_data
 * description - funciton will call bbk_xxx_readUdd to
 *               read the udd data
 * usage       -
 *               $ cat udd_data
 */
static ssize_t intf_sysfs_udd_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned char buffer[UDD_DATA_SIZE] = {0};

	retval =  bbk_xxx_readUdd(buffer);
	if (retval < 0) {
		return snprintf(buf, PAGE_SIZE,
				"\nerror: unable to get the udd data\n\n");
	}

	return snprintf(buf, PAGE_SIZE,
				"\n%s\n\n", buffer);
}
#endif
/*
 * sysfs name  - mode_change
 * description - funciton will call ibbk_xxx_mode_change to
 *               do the touch mode switching
 * usage       - echo [0/1/2] > udd_data
 */
static ssize_t intf_sysfs_mode_change_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	retval = bbk_xxx_mode_change(input);
	if (retval < 0)
		return -EINVAL;

	retval = count;

	return retval;
}
/*
 * sysfs name  - fw_version
 * description - funciton will call bbk_xxx_get_fw_version to
 *               show the fw version
 * usage       -
 *               $ cat fw_version
 */
static ssize_t intf_sysfs_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
				"\n%d\n\n", bbk_xxx_get_fw_version(0));
}

/*
 * sysfs name  - charger_bit
 * description - funciton will call bbk_xxx_set_charger_bit to
 *               control the charger bit
 * usage       - to enable
 *               $ echo 1 > charger_bit
 *               to disable
 *               $ echo 0 > charger_bit
 */
static ssize_t intf_sysfs_charger_bit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	retval = bbk_xxx_set_charger_bit(input);
	if (retval < 0)
		return -EINVAL;

	retval = count;

	return retval;
}
/*
 * sysfs name  - charger_bit
 * description - funciton will show the current setting of charger bit
 * usage       -
 *               $ cat charger_bit
 */
static ssize_t intf_sysfs_charger_bit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned short value;

	retval = intf_helper_get_dynamic_config(DC_CHARGER_CONNECTED, &value);
	if (retval < 0) {
		return snprintf(buf, PAGE_SIZE,
				"\nerror: unable to get the setting of charger bit\n\n");
	}

	return snprintf(buf, PAGE_SIZE, "\ncharger bit: %d\n\n", value);
}
/*
 * sysfs name  - edge_bit
 * description - funciton will call bbk_xxx_set_edge_bit to
 *               control the edge bit
 * usage       - horizontal
 *               $ echo 1 > edge_bit
 *               vertical
 *               $ echo 0 > edge_bit
 */
static ssize_t intf_sysfs_edge_bit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	retval = vivo_td4330_set_edge_bit(input);
	if (retval < 0)
		return -EINVAL;

	retval = count;

	return retval;
}
/*
 * sysfs name  - edge_bit
 * description - funciton will show the current setting of edge bit
 * usage       -
 *               $ cat edge_bit
 */
static ssize_t intf_sysfs_edge_bit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned short value;

	retval = intf_helper_get_dynamic_config(DC_DAKZONE_ENABLE_BIT, &value);
	if (retval < 0) {
		return snprintf(buf, PAGE_SIZE,
				"\nerror: unable to get the setting of edge bit\n\n");
	}

	return snprintf(buf, PAGE_SIZE, "\nedge bit: %d\n\n", value);
}
#if 0
/*
 * sysfs name  - fw_version_in_file
 * description - funciton will call bbk_xxx_get_header_file_version to
 *               show the fw version defined in image file
 * usage       -
 *               $ cat fw_version_in_file
 */
static ssize_t intf_sysfs_fw_version_in_file_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	const struct firmware *fw_entry;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	/* get the firmware binary */
	retval = request_firmware(&fw_entry, FIRMWARE_IMG_NAME,
				tcm_hcd->pdev->dev.parent);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to request %s\n",
				FIRMWARE_IMG_NAME);
		return -EINVAL;
	}

	retval = bbk_xxx_get_header_file_version(0, fw_entry->data);

	if (fw_entry) {
		release_firmware(fw_entry);
		fw_entry = NULL;
	}

	return snprintf(buf, PAGE_SIZE, "\n%d\n\n", retval);
}
#endif



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
static int bbk_xxx_get_rawordiff_data(int which, int *data)
{
	int retval = -EINVAL;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned char timeout_count = 0;
	short *p_data_16;
	struct syna_tcm_buffer buf;
	int i, j;

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

	if (!intf_helper_set_report_type(which)) {
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

	retval = 0;
	RELEASE_BUFFER(buf);

exit:
	return retval;
}

#if 0
/*
 * vivo api 2 - function to perform firmware update
 *
 * input:
 *    struct firmware - firmware binary
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_fw_update(const struct firmware *fw)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	if (!fw) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"invalid parameter, firmware binary\n");
		return -EINVAL;
	}

	retval = reflash_wrapper_run_fw_update(fw->data, (unsigned int)fw->size);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to do fw update\n");
		return -EINVAL;
	}

	return 0;
}
#endif

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
static int idleEnableOrDisable(int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (0 == state)?0:1;

	if ((atomic_read(&vivoTsGetVtsData()->tsState) != TOUCHSCREEN_NORMAL) || 
		(vivoTsGetVtsData()->hasLcdShutoff != 0) ||
		!atomic_read(&tcm_hcd->host_fw_valid)) {
		VTI("touch firmware invalid or not in normal state");
		return 0;
	}

	retval = intf_helper_set_dynamic_config(DC_NO_DOZE, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_NO_DOZE\n");
		return -EINVAL;
	}

	return 0;
}

#if 0
/*
 * vivo api 4.1 - function to read the udd data
 *
 * input:
 *    char * - buffer to keep the udd data
 * return:
 *    succeed- 0
 *    fail   - (-EINVAL)
 */
static int bbk_xxx_readUdd(unsigned char *udd)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	retval = reflash_wrapper_read_oem_data(udd, UDD_DATA_SIZE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to read oem data\n");
		return -EINVAL;
	}

	LOGN(tcm_hcd->pdev->dev.parent,
			"udd data: %s\n", udd);

	return 0;
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
static int bbk_xxx_writeUdd(unsigned char *udd)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;

	retval = reflash_wrapper_write_oem_data(udd, UDD_DATA_SIZE);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to write oem data\n");
		return -EINVAL;
	}

	return 0;
}
#endif

#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
#else
static int mdss_dsi_panel_reset_and_powerctl(int enable)
{
	return 0;
}
#endif
static int resume_exception_recovery(void);

/*
 * Judge gesture open or not
 * false - close gesture  ture - open
 */
static bool bbk_gesture_open_judge(void)
{
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();

	if (vtsData != NULL) {
		if (vtsData->dClickSwitch == 0 && vtsData->gestureSwitch == 0 &&
			vtsData->swipeSwitch == 0 && vtsData->udfGestureSwitch == 0 && vtsData->exportSwitch == 0) {
			return false;
		} else {
			return true;
		}
	}

	return false;
}

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
static int bbk_xxx_mode_change(int which)
{
	static int last_state = TOUCHSCREEN_NORMAL;
	int retval = 0;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	#ifdef CONFIG_OF
	/* const struct syna_tcm_pinctrl *pinctrl = &tcm_hcd->hw_if->pinctrl; */ /* if VDDI can be shutdown in sleep mode, it should be open */
	#endif

	VTI("====start====, which:%d (0:Normal, 1:Sleep, 2:Gesture)", which);
	atomic_set(&tcm_hcd->current_mode, which);

	#ifdef CONFIG_OF
	/* pinctrl_select_state(pinctrl->pinctrl_gpios, pinctrl->spi_cs_set); */ /* if VDDI can be shutdown in sleep mode, it should be open */
	#endif
	switch (which) {
	case TOUCHSCREEN_NORMAL:
		//if (last_state != TOUCHSCREEN_GESTURE) {
		//tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		//}
		tcm_hcd->er_disable_irq_flag = false;
		tcm_hcd->resume(&tcm_hcd->pdev->dev);
		if (0) {
			resume_exception_recovery();
		}
		break;
	case TOUCHSCREEN_SLEEP:
		if (last_state == TOUCHSCREEN_GESTURE) {
			tcm_hcd->resume(&tcm_hcd->pdev->dev);
			VTD("SLEEP  :Resume end, Suspend start");
		}
		tcm_hcd->suspend(&tcm_hcd->pdev->dev);
		tcm_hcd->enable_irq(tcm_hcd, false, NULL);
		if (!bbk_gesture_open_judge()) {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			#ifdef CONFIG_OF
			/* pinctrl_select_state(pinctrl->pinctrl_gpios, pinctrl->pin_cs_pulllow); */ /* if VDDI can be shutdown in sleep mode, it should be open */ 
			#endif
			mdss_dsi_panel_reset_and_powerctl(0);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		} else {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(2);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		}
		atomic_set(&tcm_hcd->host_fw_valid, 0);
		break;

	case TOUCHSCREEN_GESTURE:
		if (last_state == TOUCHSCREEN_SLEEP) {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(1);
			msleep(2);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
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
static int bbk_xxx_get_fw_version(int which)
{
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	int retval;

	if (0 == which) {
		retval = tcm_hcd->app_info.customer_config_id[15];
		/*retval = tcm_hcd->packrat_number;*/
	} else if (1 == which) {
		/*retval = tcm_hcd->app_info.customer_config_id[15];*/
		retval = -FUN_NOT_REG;
	} else {
		retval = (-1);
	}

	return retval;
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
extern int bbk_sw_reset(void);
extern int bbk_gl_reset_tag;
static int bbk_xxx_set_charger_bit(int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (0 == state)?0:1;

	if (!atomic_read(&tcm_hcd->host_fw_valid)) {
		VTI("touch firmware invalid");
		return 0;
	}

	retval = intf_helper_set_dynamic_config(DC_CHARGER_CONNECTED, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_NO_DOZE\n");
		retval = (-1);
		return retval;
	}
	VTI("====sucess====, value:%d (0:OFF, 1:ON)", value);

	return 0;
}

static int vivo_td4330_set_edge_bit (int state)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = intf_hcd->tcm_hcd;
	unsigned short value = (1 == state)?0:1;

	if (!atomic_read(&tcm_hcd->host_fw_valid)) {
		VTI("touch firmware invalid");
		return 0;
	}

	retval = intf_helper_set_dynamic_config(DC_ROTATE_OR_HORIZONTAL, value);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
			"failed to config DC_ROTATE_OR_HORIZONTAL\n");
		retval = (-1);
		return retval;
	}
	VTI("====sucess====, state:%d (0:Left Horizontal, 1:Vertical, 2:Right Horizontal)", state);
	if (state == 0) {
		retval = intf_helper_set_dynamic_config(DC_DAKZONE_ENABLE_BIT, 0x03);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to config DC_DAKZONE_ENABLE_BIT\n");
			retval = (-1);
			return retval;
		}
	} else if (state == 1) {
		retval = intf_helper_set_dynamic_config(DC_DAKZONE_ENABLE_BIT, 0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
				"failed to config DC_DAKZONE_ENABLE_BIT\n");
			retval = (-1);
			return retval;
		}
	} else if (state == 2) {
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
#if 0
/*
 * vivo api 11 - function to get the fw id defined in the image file
 *
 * input:
 *    which - 0: fw version
 *            1: config version
 * return:
 *    packrat number in image file.
 *    if not support, return 0;
 *    return (-1) when error
 */
static int bbk_xxx_get_header_file_version(int which, const unsigned char *fw)
{
	int retval;

	if (0 == which)
		retval = reflash_wrapper_get_image_fw_id(fw);
	else if (1 == which)
		retval = 0;
	else
		retval = (-1);

	return retval;
}
#endif
/*
 *
 *
 *
 */
static int resume_exception_recovery(void)
{
	int retval;
	if (bbk_gl_reset_tag) {
		VTI("******************xxx do bbk sw_reset**************");
		retval = bbk_sw_reset();
		if (retval < 0) {
			VTI("bbk sw_reset fail");
		}
		bbk_gl_reset_tag = 0;
	}
	return retval;
}

/*
 * module initialization
 * to allocate the struct vivo_intf_hcd, then create the sysfs files
 */
static int vivo_intf_init(struct syna_tcm_hcd *tcm_hcd)
{
	int retval = 0;
	int idx;
	struct vivo_ts_struct *vivo_ts_data = vivoTsGetVtsData();

	LOGN(tcm_hcd->pdev->dev.parent, "+\n");

	if (vivo_ts_data == NULL) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to get vivo descriple struct\n");
		return -EIO;
	}
	vivo_ts_data->getRawOrDiffData = bbk_xxx_get_rawordiff_data;
	vivo_ts_data->getIcFirmwareOrConfigVersion = bbk_xxx_get_fw_version;
	vivo_ts_data->setChargerFlagSwitch = bbk_xxx_set_charger_bit;
	vivo_ts_data->setEdgeRestainSwitch = vivo_td4330_set_edge_bit;
	#if 0
	vivo_ts_data->readImei = bbk_xxx_readUdd;
	vivo_ts_data->writeImei = bbk_xxx_writeUdd;
	vivo_ts_data->updateFirmware = bbk_xxx_fw_update;	
	#endif
	vivo_ts_data->idleEnableOrDisable = idleEnableOrDisable;
	vivo_ts_data->icModeChange = bbk_xxx_mode_change;
	vivo_ts_data->sensorTestKey = "com.vivotouchscreen.sensortestsyna4330:ActivityVIVOProduction:com.vivotouchscreen.sensortestsyna4330:0:com.vivotouchscreen.sensortestsyna4330";
	vivo_ts_data->lcmNoiseTestKey = "com.vivotouchscreen.synadeltadiff:MainActivity:null:null:null";

	intf_hcd = kzalloc(sizeof(struct vivo_intf_hcd), GFP_KERNEL);
	if (!intf_hcd) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to allocate memory for vivo_intf_hcd\n");
		return -ENOMEM;
	}

	intf_hcd->tcm_hcd = tcm_hcd;

	intf_hcd->sysfs_dir = kobject_create_and_add(SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!intf_hcd->sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"failed to create sysfs directory\n");
		goto err_sysfs_create_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		retval = sysfs_create_file(intf_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"failed to create sysfs file\n");
			goto err_sysfs_create_file;
		}
	}

	intf_hcd->err_flag = false;
	intf_hcd->report_data = NULL;
	intf_hcd->is_lpwg = false;
	intf_hcd->report_cols = 0;
	intf_hcd->report_rows = 0;

	vivoTsAfterProbeCompleteCall(vivo_ts_data->client, NULL, -1);
	return 0;

err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(intf_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(intf_hcd->sysfs_dir);

err_sysfs_create_dir:
	kfree(intf_hcd);
	intf_hcd = NULL;

	return retval;
}

/*
 * to remove the module and sysfs files as well
 */
static int vivo_intf_remove(struct syna_tcm_hcd *tcm_hcd)
{
	int idx;

	LOGN(tcm_hcd->pdev->dev.parent, "+\n");

	if (!intf_hcd)
		goto exit;

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++)
		sysfs_remove_file(intf_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(intf_hcd->sysfs_dir);

	kfree(intf_hcd);
	intf_hcd = NULL;

exit:
	complete(&vivo_intf_remove_complete);

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

static int __init vivo_intf_module_init(void)
{
	return syna_tcm_add_module(&vivo_intf_module, true);
}

static void __exit vivo_intf_module_exit(void)
{
	syna_tcm_add_module(&vivo_intf_module, false);

	wait_for_completion(&vivo_intf_remove_complete);

	return;
}

module_init(vivo_intf_module_init);
module_exit(vivo_intf_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM VIVO Interface Module");
MODULE_LICENSE("GPL v2");
