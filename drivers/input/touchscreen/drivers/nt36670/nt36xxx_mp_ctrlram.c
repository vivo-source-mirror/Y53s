/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 21288 $
 * $Date: 2018 -01 -05 11:38:47 + 0800 $
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include "nt36xxx.h"
#include "nt36xxx_mp_ctrlram.h"
#include "../vts_core.h"

#if NVT_TOUCH_MP

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define MP_MODE_CC 0x41
#define FREQ_HOP_DISABLE 0x66
#define FREQ_HOP_ENABLE 0x65

/*
#define SHORT_TEST_CSV_FILE "/data/local/tmp/ShortTest.csv"
#define OPEN_TEST_CSV_FILE "/data/local/tmp/OpenTest.csv"
#define FW_RAWDATA_CSV_FILE "/data/local/tmp/FWMutualTest.csv"
#define FW_CC_CSV_FILE "/data/local/tmp/FWCCTest.csv"
#define NOISE_TEST_CSV_FILE "/data/local/tmp/NoiseTest.csv"
 */
#define SHORT_TEST_CSV_FILE "/sdcard/ShortTest.csv"
#define OPEN_TEST_CSV_FILE "/sdcard/OpenTest.csv"
#define FW_RAWDATA_CSV_FILE "/sdcard/FWMutualTest.csv"
#define FW_CC_CSV_FILE "/sdcard/FWCCTest.csv"
#define NOISE_TEST_CSV_FILE "/sdcard/NoiseTest.csv"
/*
#define FW_CC1D_CSV_FILE "/data/local/tmp/FWCCTest1D.csv"
#define DIGITAL_TEST_CSV_FILE "/data/local/tmp/DigitalModeTest.csv"
 */
#define FW_CC1D_CSV_FILE "/sdcard/FWCCTest1D.csv"
#define DIGITAL_TEST_CSV_FILE "/sdcard/DigitalModeTest.csv"

#define nvt_mp_seq_printf(m, fmt, args...) do {	\
	seq_printf(m, fmt, ##args);	\
	if (!nvt_mp_test_result_printed)	\
		printk(fmt, ##args);	\
} while (0)

static uint8_t *RecordResult_Short;
static uint8_t *RecordResult_Short_Diff;
static uint8_t *RecordResult_Short_Base;
static uint8_t *RecordResult_Open;
static uint8_t *RecordResult_FWMutual;
static uint8_t *RecordResult_FW_CC;
static uint8_t *RecordResult_FW_CC_I;
static uint8_t *RecordResult_FW_CC_Q;
static uint8_t *RecordResult_FW_DiffMax;
static uint8_t *RecordResult_FW_DiffMin;

static int32_t TestResult_Short;
static int32_t TestResult_Short_Diff;
static int32_t TestResult_Short_Base;
static int32_t TestResult_Open;
static int32_t TestResult_FW_Rawdata;
static int32_t TestResult_FWMutual;
static int32_t TestResult_FW_CC;
static int32_t TestResult_FW_CC_I;
static int32_t TestResult_FW_CC_Q;
static int32_t TestResult_Noise;
static int32_t TestResult_FW_DiffMax;
static int32_t TestResult_FW_DiffMin;

static int32_t TestResult_FW_DozeCC;/*[20180426] */
static int32_t TestResult_FW_DIGITAL;/*[20180426] */
static int32_t *RawData_Short;
static int32_t *RawData_Short_Diff;
static int32_t *RawData_Short_Base;
static int32_t *RawData_Open;
static int32_t *RawData_Diff;
static int32_t *RawData_Diff_Min;
static int32_t *RawData_Diff_Max;
static int32_t *RawData_FWMutual;
static int32_t *RawData_FW_CC;
static int32_t *RawData_FW_CC_I;
static int32_t *RawData_FW_CC_Q;

static int32_t *RawData_FW_CC_DOZE;/*[20180426] */
static int32_t *RawData_FW_DIGITAL;/*[20180426] */


static struct proc_dir_entry *NVT_proc_selftest_entry;
static int8_t nvt_mp_test_result_printed;

extern void nt36670_change_mode(uint8_t mode);
extern uint8_t nt36670_get_fw_pipe(void);
extern void nt36670_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr);
extern void nt36670_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num);
void nt36670_mp_parse_dt(struct device_node *root, const char *node_compatible);

/*******************************************************
Description:
	Novatek touchscreen allocate buffer for mp selftest.

return:
	Executive outcomes. 0-- - succeed. -12-- - Out of memory
 *******************************************************/
static int nvt_mp_buffer_init(void)
{
	size_t RecordResult_BufSize = IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE;
	size_t RawData_BufSize = (IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE) * sizeof(int32_t);

	RecordResult_Short = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_Short) {
		NVT_ERR("kzalloc for RecordResult_Short failed!\n");
		return -ENOMEM;
	}

	RecordResult_Short_Diff = RecordResult_Short;

	RecordResult_Short_Base = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_Short_Base) {
		NVT_ERR("kzalloc for RecordResult_Short_Base failed!\n");
		return -ENOMEM;
	}

	RecordResult_Open = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_Open) {
		NVT_ERR("kzalloc for RecordResult_Open failed!\n");
		return -ENOMEM;
	}

	RecordResult_FWMutual = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FWMutual) {
		NVT_ERR("kzalloc for RecordResult_FWMutual failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_CC = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_CC) {
		NVT_ERR("kzalloc for RecordResult_FW_CC failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_CC_I = RecordResult_FW_CC;

	RecordResult_FW_CC_Q = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_CC_Q) {
		NVT_ERR("kzalloc for RecordResult_FW_CC_Q failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_DiffMax = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_DiffMax) {
		NVT_ERR("kzalloc for RecordResult_FW_DiffMax failed!\n");
		return -ENOMEM;
	}

	RecordResult_FW_DiffMin = (uint8_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RecordResult_FW_DiffMin) {
		NVT_ERR("kzalloc for RecordResult_FW_DiffMin failed!\n");
		return -ENOMEM;
	}

	RawData_Short = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Short) {
		NVT_ERR("kzalloc for RawData_Short failed!\n");
		return -ENOMEM;
	}

	RawData_Short_Diff = RawData_Short;

	RawData_Short_Base = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Short_Base) {
		NVT_ERR("kzalloc for RawData_Short_Base failed!\n");
		return -ENOMEM;
	}

	RawData_Open = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Open) {
		NVT_ERR("kzalloc for RawData_Open failed!\n");
		return -ENOMEM;
	}

	RawData_Diff = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff) {
		NVT_ERR("kzalloc for RawData_Diff failed!\n");
		return -ENOMEM;
	}

	RawData_Diff_Min = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff_Min) {
		NVT_ERR("kzalloc for RawData_Diff_Min failed!\n");
		return -ENOMEM;
	}

	RawData_Diff_Max = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_Diff_Max) {
		NVT_ERR("kzalloc for RawData_Diff_Max failed!\n");
		return -ENOMEM;
	}

	RawData_FWMutual = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FWMutual) {
		NVT_ERR("kzalloc for RawData_FWMutual failed!\n");
		return -ENOMEM;
	}

	RawData_FW_CC = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FW_CC) {
		NVT_ERR("kzalloc for RawData_FW_CC failed!\n");
		return -ENOMEM;
	}

	RawData_FW_CC_I = RawData_FW_CC;

	RawData_FW_CC_Q = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FW_CC_Q) {
		NVT_ERR("kzalloc for RawData_FW_CC_Q failed!\n");
		return -ENOMEM;
	}

	RawData_FW_DIGITAL = (int32_t *)kzalloc(RawData_BufSize, GFP_KERNEL);
	if (!RawData_FW_DIGITAL) {
		NVT_ERR("kzalloc for RawData_FW_DIGITAL failed!\n");
		return -ENOMEM;
	}

	RawData_FW_CC_DOZE = (int32_t *)kzalloc(RecordResult_BufSize, GFP_KERNEL);
	if (!RawData_FW_CC_DOZE) {
		NVT_ERR("kzalloc for RawData_FW_CC_DOZE failed!\n");
		return -ENOMEM;
	}

	return 0;
}

#if 0
static struct file *nvt_file_open(const char *filename, int flags, umode_t mode)
{
	return filp_open(filename, flags, mode);
}
#endif

static int32_t nvt_save_rawdata_to_csv(int32_t *rawdata, uint8_t x_ch, uint8_t y_ch, const char *file_path, uint32_t offset)
{
#if 0
	int32_t x = 0;
	int32_t y = 0;
	int32_t iArrayIndex = 0;
	struct file *fp = NULL;
	char *fbufp = NULL;
	mm_segment_t org_fs;
	int32_t write_ret = 0;
	uint32_t output_len = 0;
	loff_t pos = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
	int32_t keydata_output_offset = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	printk("%s:++\n", __func__);
	fbufp = (char *)kzalloc(8192, GFP_KERNEL);
	if (!fbufp) {
		NVT_ERR("kzalloc for fbufp failed!\n");
		return -ENOMEM;
	}

	for (y = 0; y < y_ch; y++) {
		for (x = 0; x < x_ch; x++) {
			iArrayIndex = y * x_ch + x;
			printk("%5d, ", rawdata[iArrayIndex]);
			snprintf(fbufp + iArrayIndex * 7 + y * 2, 1023, "%5d, ", rawdata[iArrayIndex]);
		}
		printk("\n");
		snprintf(fbufp + (iArrayIndex + 1) * 7 + y * 2, 1023, "\r\n");
	}
#if TOUCH_KEY_NUM > 0
	keydata_output_offset = y_ch * x_ch * 7 + y_ch * 2;
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = y_ch * x_ch + k;
		printk("%5d, ", rawdata[iArrayIndex]);
		snprintf(fbufp + keydata_output_offset + k * 7, 1023, "%5d, ", rawdata[iArrayIndex]);
	}
	printk("\n");
	snprintf(fbufp + y_ch * x_ch * 7 + y_ch * 2 + Key_Channel * 7, 1023, "\r\n");
#endif /* #if TOUCH_KEY_NUM > 0 */

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = nvt_file_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp) {
			kfree(fbufp);
			fbufp = NULL;
		}
		x = -1;
		return x;
	}

#if TOUCH_KEY_NUM > 0
	output_len = y_ch * x_ch * 7 + y_ch * 2 + Key_Channel * 7 + 2;
#else
	output_len = y_ch * x_ch * 7 + y_ch * 2;
#endif /* #if TOUCH_KEY_NUM > 0 */
	pos = offset;
	write_ret = vfs_write(fp, (char __user *)fbufp, output_len, &pos);
	if (write_ret <= 0) {
		NVT_ERR("write %s failed\n", file_path);
		set_fs(org_fs);
		if (fp) {
			filp_close(fp, NULL);
			fp = NULL;
		}
		if (fbufp) {
			kfree(fbufp);
			fbufp = NULL;
		}
		x = -1;
		return x;
	}

	set_fs(org_fs);
	if (fp) {
		filp_close(fp, NULL);
		fp = NULL;
	}
	if (fbufp) {
		kfree(fbufp);
		fbufp = NULL;
	}

	printk("%s:--\n", __func__);
#endif
	return 0;
}

static int32_t nvt_polling_hand_shake_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

		/*-- - read fw status-- - */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] == 0xA0) || (buf[1] == 0xA1))
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("polling hand shake status failed, buf[1]=0x%02X\n", buf[1]);
		buf[0] = 0xFF;
		buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 6);
		NVT_ERR("Read back 5 bytes from offset EVENT_MAP_HOST_CMD: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
		i = -1;
		return i;
	} else {
		return 0;
	}
}

static int8_t nvt_switch_FreqHopEnDis(uint8_t FreqHopEnDis)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	NVT_LOG("++\n");

	for (retry = 0; retry < 20; retry++) {
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

		/*-- - switch FreqHopEnDis-- - */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = FreqHopEnDis;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		NVT_ERR("switch FreqHopEnDis 0x%02X failed, buf[1]=0x%02X\n", FreqHopEnDis, buf[1]);
		ret = -1;
	}

	NVT_LOG("--\n");

	return ret;
}

static int32_t nvt_read_baseline(int32_t *xdata)
{
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	NVT_LOG("++\n");

	nt36670_read_mdata(nt36670_ts->mmap->BASELINE_ADDR, nt36670_ts->mmap->BASELINE_BTN_ADDR);

	nt36670_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (nt36670_ts->carrier_system) {
				xdata[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
			} else {
				xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		if (nt36670_ts->carrier_system) {
			xdata[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
		} else {
			xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
		}
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	printk("%s:\n", __func__);
	/* Save Rawdata to CSV file */
	if (nvt_save_rawdata_to_csv(xdata, X_Channel, Y_Channel, FW_RAWDATA_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}

	NVT_LOG("--\n");

	return 0;
}

static int32_t nvt_read_CC(int32_t *xdata)
{
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
	int32_t xdata_tmp = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */
	uint32_t rawdata_cc_q_offset = 0;

	NVT_LOG("++\n");

	if (nt36670_get_fw_pipe() == 0)
		nt36670_read_mdata(nt36670_ts->mmap->DIFF_PIPE1_ADDR, nt36670_ts->mmap->DIFF_BTN_PIPE1_ADDR);
	else
		nt36670_read_mdata(nt36670_ts->mmap->DIFF_PIPE0_ADDR, nt36670_ts->mmap->DIFF_BTN_PIPE0_ADDR);

	nt36670_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (nt36670_ts->carrier_system) {
				xdata_tmp = xdata[iArrayIndex];
				RawData_FW_CC_I[iArrayIndex] = (uint8_t)(xdata_tmp & 0xFF);
				RawData_FW_CC_Q[iArrayIndex] = (uint8_t)((xdata_tmp >> 8) & 0xFF);
			} else {
				xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		if (nt36670_ts->carrier_system) {
			xdata_tmp = xdata[iArrayIndex];
			RawData_FW_CC_I[iArrayIndex] = (uint8_t)(xdata_tmp & 0xFF);
			RawData_FW_CC_Q[iArrayIndex] = (uint8_t)((xdata_tmp >> 8) & 0xFF);
		} else {
			xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
		}
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	printk("%s:\n", __func__);
	if (nt36670_ts->carrier_system) {
		printk("%s:RawData_CC_I:\n", __func__);
		/* Save Rawdata to CSV file */
		if (nvt_save_rawdata_to_csv(RawData_FW_CC_I, X_Channel, Y_Channel, FW_CC_CSV_FILE, 0) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
#if TOUCH_KEY_NUM > 0
		rawdata_cc_q_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2 + Key_Channel * 7 + 2;
#else
		rawdata_cc_q_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
#endif /* #if TOUCH_KEY_NUM > 0 */
		printk("%s:RawData_CC_Q:\n", __func__);
		/* Save Rawdata to CSV file */
		if (nvt_save_rawdata_to_csv(RawData_FW_CC_Q, X_Channel, Y_Channel, FW_CC_CSV_FILE, rawdata_cc_q_offset) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
	} else {
		/* Save Rawdata to CSV file */
		if (nvt_save_rawdata_to_csv(xdata, X_Channel, Y_Channel, FW_CC_CSV_FILE, 0) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
	}

	NVT_LOG("--\n");

	return 0;
}

static void nvt_enable_noise_collect(int32_t frame_num)
{
	uint8_t buf[8] = {0};

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - enable noise collect-- - */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x47;
	buf[2] = 0xAA;
	buf[3] = frame_num;
	buf[4] = 0x00;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 5);
}

static int32_t nvt_read_fw_noise(int32_t *xdata)
{
	uint8_t x_num = 0;
	uint8_t y_num = 0;
	uint32_t x = 0;
	uint32_t y = 0;
	int32_t iArrayIndex = 0;
	int32_t frame_num = 0;
	uint32_t rawdata_diff_min_offset = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	NVT_LOG("++\n");

	/*-- -Enter Test Mode-- - */
	if (nt36670_clear_fw_status()) {
		return -EAGAIN;
	}

	frame_num = PS_Config_Diff_Test_Frame / 10;
	if (frame_num <= 0)
		frame_num = 1;
	printk("%s: frame_num=%d\n", __func__, frame_num);
	nvt_enable_noise_collect(frame_num);
	/* need wait PS_Config_Diff_Test_Frame * 8.3ms */
	msleep(frame_num * 83);

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

	if (nt36670_get_fw_info()) {
		return -EAGAIN;
	}

	if (nt36670_get_fw_pipe() == 0)
		nt36670_read_mdata(nt36670_ts->mmap->DIFF_PIPE0_ADDR, nt36670_ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nt36670_read_mdata(nt36670_ts->mmap->DIFF_PIPE1_ADDR, nt36670_ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nt36670_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (nt36670_ts->carrier_system) {
				RawData_Diff_Max[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
				RawData_Diff_Min[iArrayIndex] = 0;
			} else {
				RawData_Diff_Max[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
				RawData_Diff_Min[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		if (nt36670_ts->carrier_system) {
			RawData_Diff_Max[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
			RawData_Diff_Min[iArrayIndex] = 0;
		} else {
			RawData_Diff_Max[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
			RawData_Diff_Min[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
		}
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	/*-- - Leave Test Mode-- - */
	nt36670_change_mode(NORMAL_MODE);

	printk("%s:RawData_Diff_Max:\n", __func__);
	/* Save Rawdata to CSV file */
	if (nvt_save_rawdata_to_csv(RawData_Diff_Max, X_Channel, Y_Channel, NOISE_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}

	if (!nt36670_ts->carrier_system) {
#if TOUCH_KEY_NUM > 0
		rawdata_diff_min_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2 + Key_Channel * 7 + 2;
#else
		rawdata_diff_min_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
#endif /* #if TOUCH_KEY_NUM > 0 */
		printk("%s:RawData_Diff_Min:\n", __func__);
		/* Save Rawdata to CSV file */
		if (nvt_save_rawdata_to_csv(RawData_Diff_Min, X_Channel, Y_Channel, NOISE_TEST_CSV_FILE, rawdata_diff_min_offset) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
	}

	NVT_LOG("--\n");

	return 0;
}

static void nvt_enable_open_test(void)
{
	uint8_t buf[8] = {0};

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - enable open test-- - */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x45;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 5);
}

static void nvt_enable_short_test(void)
{
	uint8_t buf[8] = {0};

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - enable short test-- - */
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x43;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 5);
}

static int32_t nvt_read_fw_open(int32_t *xdata)
{
	uint32_t raw_pipe_addr = 0;
	uint8_t *rawdata_buf = NULL;
	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t buf[128] = {0};
#if TOUCH_KEY_NUM > 0
	uint32_t raw_btn_pipe_addr = 0;
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	NVT_LOG("++\n");

	/*-- -Enter Test Mode-- - */
	if (nt36670_clear_fw_status()) {
		return -EAGAIN;
	}

	nvt_enable_open_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

#if TOUCH_KEY_NUM > 0
	rawdata_buf = (uint8_t *)kzalloc((IC_X_CFG_SIZE * IC_Y_CFG_SIZE + IC_KEY_CFG_SIZE) * 2, GFP_KERNEL);
#else
	rawdata_buf = (uint8_t *)kzalloc(IC_X_CFG_SIZE * IC_Y_CFG_SIZE * 2, GFP_KERNEL);
#endif /* #if TOUCH_KEY_NUM > 0 */
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (nt36670_get_fw_pipe() == 0)
		raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE0_ADDR;
	else
		raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE1_ADDR;

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		/*-- - change xdata index-- - */
		buf[0] = 0xFF;
		buf[1] = (uint8_t)(((raw_pipe_addr + y * IC_X_CFG_SIZE * 2) >> 16) & 0xFF);
		buf[2] = (uint8_t)(((raw_pipe_addr + y * IC_X_CFG_SIZE * 2) >> 8) & 0xFF);
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
		buf[0] = (uint8_t)((raw_pipe_addr + y * IC_X_CFG_SIZE * 2) & 0xFF);
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, IC_X_CFG_SIZE * 2 + 1);
		memcpy(rawdata_buf + y * IC_X_CFG_SIZE * 2, buf + 1, IC_X_CFG_SIZE * 2);
	}
#if TOUCH_KEY_NUM > 0
	if (nt36670_get_fw_pipe() == 0)
		raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE0_ADDR;
	else
		raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE1_ADDR;

	/*-- - change xdata index-- - */
	buf[0] = 0xFF;
	buf[1] = (uint8_t)((raw_btn_pipe_addr >> 16) & 0xFF);
	buf[2] = (uint8_t)((raw_btn_pipe_addr >> 8) & 0xFF);
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
	buf[0] = (uint8_t)(raw_btn_pipe_addr & 0xFF);
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, IC_KEY_CFG_SIZE * 2 + 1);
	memcpy(rawdata_buf + IC_Y_CFG_SIZE * IC_X_CFG_SIZE * 2, buf + 1, IC_KEY_CFG_SIZE * 2);
#endif /* #if TOUCH_KEY_NUM > 0 */

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		for (x = 0; x < IC_X_CFG_SIZE; x++) {
			if ((AIN_Y[y] != 0xFF) && (AIN_X[x] != 0xFF)) {
				xdata[AIN_Y[y] * X_Channel + AIN_X[x]] = (int16_t)((rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2] + 256 * rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2 + 1]));
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < IC_KEY_CFG_SIZE; k++) {
		if (AIN_KEY[k] != 0xFF)
			xdata[Y_Channel * X_Channel + AIN_KEY[k]] = (int16_t)(rawdata_buf[(IC_Y_CFG_SIZE * IC_X_CFG_SIZE + k) * 2] + 256 * rawdata_buf[(IC_Y_CFG_SIZE * IC_X_CFG_SIZE + k) * 2 + 1]);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	/*-- - Leave Test Mode-- - */
	nt36670_change_mode(NORMAL_MODE);


	printk("%s:RawData_Open\n", __func__);
	/* Save RawData to CSV file */
	if (nvt_save_rawdata_to_csv(xdata, X_Channel, Y_Channel, OPEN_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}

	NVT_LOG("--\n");

	return 0;
}

static int32_t nvt_read_fw_short(int32_t *xdata)
{
	uint32_t raw_pipe_addr = 0;
	uint8_t *rawdata_buf = NULL;
	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t buf[128] = {0};
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	uint32_t raw_btn_pipe_addr = 0;
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */
	uint32_t rawdata_short_base_offset = 0;

	NVT_LOG("++\n");

	/*-- -Enter Test Mode-- - */
	if (nt36670_clear_fw_status()) {
		return -EAGAIN;
	}

	nvt_enable_short_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

#if TOUCH_KEY_NUM > 0
    rawdata_buf = (uint8_t *)kzalloc((X_Channel * Y_Channel + Key_Channel) * 2, GFP_KERNEL);
#else
    rawdata_buf = (uint8_t *)kzalloc(X_Channel * Y_Channel * 2, GFP_KERNEL);
#endif /* #if TOUCH_KEY_NUM > 0 */
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (nt36670_ts->carrier_system) {
		/* to get short diff rawdata at pipe0 */
		raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE0_ADDR;
	} else {
		if (nt36670_get_fw_pipe() == 0)
			raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE0_ADDR;
		else
			raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE1_ADDR;
	}

	for (y = 0; y < Y_Channel; y++) {
		/*-- - change xdata index-- - */
		buf[0] = 0xFF;
		buf[1] = (uint8_t)(((raw_pipe_addr + y * X_Channel * 2) >> 16) & 0xFF);
		buf[2] = (uint8_t)(((raw_pipe_addr + y * X_Channel * 2) >> 8) & 0xFF);
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
		buf[0] = (uint8_t)((raw_pipe_addr + y * X_Channel * 2) & 0xFF);
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, X_Channel * 2 + 1);
		memcpy(rawdata_buf + y * X_Channel * 2, buf + 1, X_Channel * 2);
	}
#if TOUCH_KEY_NUM > 0
	if (nt36670_ts->carrier_system) {
		/* to get button short diff rawdata at pipe0 */
		raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE0_ADDR;
	} else {
		if (nt36670_get_fw_pipe() == 0)
			raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE0_ADDR;
		else
			raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE1_ADDR;
	}

    /*-- - change xdata index-- - */
	buf[0] = 0xFF;
	buf[1] = (uint8_t)((raw_btn_pipe_addr >> 16) & 0xFF);
	buf[2] = (uint8_t)((raw_btn_pipe_addr >> 8) & 0xFF);
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
	buf[0] = (uint8_t)(raw_btn_pipe_addr & 0xFF);
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, Key_Channel * 2 + 1);
	memcpy(rawdata_buf + Y_Channel * X_Channel * 2, buf + 1, Key_Channel * 2);
#endif /* #if TOUCH_KEY_NUM > 0 */

	for (y = 0; y < Y_Channel; y++) {
		for (x = 0; x < X_Channel; x++) {
			iArrayIndex = y * X_Channel + x;
			xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = Y_Channel * X_Channel + k;
		xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	/* for carrier sensing system to get short baseline rawdata */
	if (nt36670_ts->carrier_system) {
		/* to get short baseline rawdata at pipe1 */
		raw_pipe_addr = nt36670_ts->mmap->RAW_PIPE1_ADDR;

		for (y = 0; y < Y_Channel; y++) {
			/*-- - change xdata index-- - */
			buf[0] = 0xFF;
			buf[1] = (uint8_t)(((raw_pipe_addr + y * X_Channel * 2) >> 16) & 0xFF);
			buf[2] = (uint8_t)(((raw_pipe_addr + y * X_Channel * 2) >> 8) & 0xFF);
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
			buf[0] = (uint8_t)((raw_pipe_addr + y * X_Channel * 2) & 0xFF);
			NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, X_Channel * 2 + 1);
			memcpy(rawdata_buf + y * X_Channel * 2, buf + 1, X_Channel * 2);
		}
#if TOUCH_KEY_NUM > 0
		/* to get button short baseline rawdata at pipe1 */
		raw_btn_pipe_addr = nt36670_ts->mmap->RAW_BTN_PIPE1_ADDR;

	    /*-- - change xdata index-- - */
		buf[0] = 0xFF;
		buf[1] = (uint8_t)((raw_btn_pipe_addr >> 16) & 0xFF);
		buf[2] = (uint8_t)((raw_btn_pipe_addr >> 8) & 0xFF);
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
		buf[0] = (uint8_t)(raw_btn_pipe_addr & 0xFF);
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, Key_Channel * 2 + 1);
		memcpy(rawdata_buf + Y_Channel * X_Channel * 2, buf + 1, Key_Channel * 2);
#endif /* #if TOUCH_KEY_NUM > 0 */

		for (y = 0; y < Y_Channel; y++) {
			for (x = 0; x < X_Channel; x++) {
				iArrayIndex = y * X_Channel + x;
				RawData_Short_Base[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
			}
		}
#if TOUCH_KEY_NUM > 0
		for (k = 0; k < Key_Channel; k++) {
			iArrayIndex = Y_Channel * X_Channel + k;
			RawData_Short_Base[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
		}
#endif /* #if TOUCH_KEY_NUM > 0 */
	}

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	/*-- - Leave Test Mode-- - */
	nt36670_change_mode(NORMAL_MODE);

	if (nt36670_ts->carrier_system)
		printk("%s:RawData_Short_Diff:\n", __func__);
	else
		printk("%s:RawData_Short\n", __func__);
	/* Save Rawdata to CSV file */
	if (nvt_save_rawdata_to_csv(xdata, X_Channel, Y_Channel, SHORT_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}
	if (nt36670_ts->carrier_system) {
#if TOUCH_KEY_NUM > 0
		rawdata_short_base_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2 + Key_Channel * 7 + 2;
#else
		rawdata_short_base_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
#endif /* #if TOUCH_KEY_NUM > 0 */
		printk("%s:RawData_Short_Base:\n", __func__);
		/* Save Rawdata to CSV file */
		if (nvt_save_rawdata_to_csv(RawData_Short_Base, X_Channel, Y_Channel, SHORT_TEST_CSV_FILE, rawdata_short_base_offset) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
	}

	NVT_LOG("--\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen raw data test for each single point function.

return:
	Executive outcomes. 0-- - passed. negative-- - failed.
 *******************************************************/
static int32_t RawDataTest_SinglePoint_Sub(int32_t rawdata[], uint8_t RecordResult[], uint8_t x_ch, uint8_t y_ch, int32_t Rawdata_Limit_Postive[], int32_t Rawdata_Limit_Negative[])
{
	int32_t i = 0;
	int32_t j = 0;
#if TOUCH_KEY_NUM > 0
    int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */
	int32_t iArrayIndex = 0;
	bool isPass = true;
	unsigned char line_buf[300];
	int offset = 0;
	unsigned char positive_line_buf[300];
	int positive_offset = 0;
	unsigned char negative_line_buf[300];
	int negative_offset = 0;

	for (j = 0; j < y_ch; j++) {
		memset(line_buf, 0, sizeof(line_buf));
		offset = 0;
		offset += snprintf(&line_buf[offset], 20, "Rawdata: ");
		memset(positive_line_buf, 0, sizeof(positive_line_buf));
		positive_offset = 0;
		positive_offset += snprintf(&positive_line_buf[positive_offset], 20, "Pos Lim: ");
		memset(negative_line_buf, 0, sizeof(negative_line_buf));
		negative_offset = 0;
		negative_offset += snprintf(&negative_line_buf[negative_offset], 20, "Neg Lim: ");
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;

			RecordResult[iArrayIndex] = 0x00; /* default value for PASS */

			if (rawdata[iArrayIndex] > Rawdata_Limit_Postive[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x01;

			if (rawdata[iArrayIndex] < Rawdata_Limit_Negative[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x02;
			offset += snprintf(&line_buf[offset], 10, "%5d ", rawdata[iArrayIndex]);
			positive_offset += snprintf(&positive_line_buf[positive_offset], 10, "%5d ", Rawdata_Limit_Postive[iArrayIndex]);
			negative_offset += snprintf(&negative_line_buf[negative_offset], 10, "%5d ", Rawdata_Limit_Negative[iArrayIndex]);
		}
		vts_dev_dbg(nt36670_ts->vtsdev, "%s", positive_line_buf);
		vts_dev_info(nt36670_ts->vtsdev, "%s", line_buf);
		vts_dev_dbg(nt36670_ts->vtsdev, "%s", negative_line_buf);
	}
#if TOUCH_KEY_NUM > 0
	memset(line_buf, 0, sizeof(line_buf));
	offset = 0;
	offset += snprintf(&line_buf[offset], 18, "Key Rawdata: ");
	memset(positive_line_buf, 0, sizeof(positive_line_buf));
	positive_offset = 0;
	positive_offset += snprintf(&positive_line_buf[positive_offset], 18, "Key Pos Lim: ");
	memset(negative_line_buf, 0, sizeof(negative_line_buf));
	negative_offset = 0;
	negative_offset += snprintf(&negative_line_buf[negative_offset], 18, "Key Neg Lim: ");
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = y_ch * x_ch + k;

		RecordResult[iArrayIndex] = 0x00; /* default value for PASS */

		if (rawdata[iArrayIndex] > Rawdata_Limit_Postive[iArrayIndex])
			RecordResult[iArrayIndex] |= 0x01;

		if (rawdata[iArrayIndex] < Rawdata_Limit_Negative[iArrayIndex])
			RecordResult[iArrayIndex] |= 0x02;

		offset += snprintf(&line_buf[offset], 10, "%5d ", rawdata[iArrayIndex]);
		positive_offset += snprintf(&positive_line_buf[positive_offset], 10, "%5d ", Rawdata_Limit_Postive[iArrayIndex]);
		negative_offset += snprintf(&negative_line_buf[negative_offset], 10, "%5d ", Rawdata_Limit_Negative[iArrayIndex]);
	}
	vts_dev_dbg(nt36670_ts->vtsdev, "%s", positive_line_buf);
	vts_dev_info(nt36670_ts->vtsdev, "%s", line_buf);
	vts_dev_dbg(nt36670_ts->vtsdev, "%s", negative_line_buf);
#endif /* #if TOUCH_KEY_NUM > 0 */

	/*-- - Check RecordResult-- - */
	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			if (RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}
#if TOUCH_KEY_NUM > 0
	for (k = 0; k < Key_Channel; k++) {
		iArrayIndex = y_ch * x_ch + k;
		if (RecordResult[iArrayIndex] != 0) {
			isPass = false;
			break;
		}
	}
#endif /* #if TOUCH_KEY_NUM > 0 */

	if (isPass == false) {
		i = -1;
		return i; /* FAIL */
	} else {
		return 0; /* PASS */
	}
}

/*******************************************************
Description:
	Novatek touchscreen print self - test result function.

return:
	n.a.
 *******************************************************/
void nt36670_print_selftest_result(struct seq_file *m, int32_t TestResult, uint8_t RecordResult[], int32_t rawdata[], uint8_t x_len, uint8_t y_len)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
#if TOUCH_KEY_NUM > 0
	int32_t k = 0;
#endif /* #if TOUCH_KEY_NUM > 0 */

	switch (TestResult) {
	case 0:
		nvt_mp_seq_printf(m, " PASS!\n");
		break;

	case 1:
		nvt_mp_seq_printf(m, " ERROR! Read Data FAIL!\n");
		break;

	case -1:
		nvt_mp_seq_printf(m, " FAIL!\n");
		nvt_mp_seq_printf(m, "RecordResult:\n");
		for (i = 0; i < y_len; i++) {
			for (j = 0; j < x_len; j++) {
				iArrayIndex = i * x_len + j;
				nvt_mp_seq_printf(m, "0x%02X, ", RecordResult[iArrayIndex]);
			}
			nvt_mp_seq_printf(m, "\n");
		}
#if TOUCH_KEY_NUM > 0
		for (k = 0; k < Key_Channel; k++) {
			iArrayIndex = y_len * x_len + k;
			nvt_mp_seq_printf(m, "0x%02X, ", RecordResult[iArrayIndex]);
		}
		nvt_mp_seq_printf(m, "\n");
#endif /* #if TOUCH_KEY_NUM > 0 */
		nvt_mp_seq_printf(m, "ReadData:\n");
		for (i = 0; i < y_len; i++) {
			for (j = 0; j < x_len; j++) {
				iArrayIndex = i * x_len + j;
				nvt_mp_seq_printf(m, "%5d, ", rawdata[iArrayIndex]);
			}
			nvt_mp_seq_printf(m, "\n");
		}
#if TOUCH_KEY_NUM > 0
		for (k = 0; k < Key_Channel; k++) {
			iArrayIndex = y_len * x_len + k;
			nvt_mp_seq_printf(m, "%5d, ", rawdata[iArrayIndex]);
		}
		nvt_mp_seq_printf(m, "\n");
#endif /* #if TOUCH_KEY_NUM > 0 */
		break;
	}
	nvt_mp_seq_printf(m, "\n");
}

/*******************************************************
Description:
	Novatek touchscreen self - test sequence print show
	function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t c_show_selftest(struct seq_file *m, void *v)
{
	NVT_LOG("++\n");

	nvt_mp_seq_printf(m, "FW Version: %d\n\n", nt36670_ts->fw_ver);

	nvt_mp_seq_printf(m, "Short Test");
	if ((TestResult_Short == 0) || (TestResult_Short == 1)) {
		nt36670_print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
	} else { /* TestResult_Short is -1 */
		if (nt36670_ts->carrier_system) {
			nvt_mp_seq_printf(m, " FAIL!\n");
			if (TestResult_Short_Diff == -1) {
				nvt_mp_seq_printf(m, "Short Diff");
				nt36670_print_selftest_result(m, TestResult_Short_Diff, RecordResult_Short_Diff, RawData_Short_Diff, X_Channel, Y_Channel);
			}
			if (TestResult_Short_Base == -1) {
				nvt_mp_seq_printf(m, "Short Base");
				nt36670_print_selftest_result(m, TestResult_Short_Base, RecordResult_Short_Base, RawData_Short_Base, X_Channel, Y_Channel);
			}
		} else {
			nt36670_print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
		}
	}

	nvt_mp_seq_printf(m, "Open Test");
	nt36670_print_selftest_result(m, TestResult_Open, RecordResult_Open, RawData_Open, X_Channel, Y_Channel);

	nvt_mp_seq_printf(m, "FW Rawdata Test");
	if ((TestResult_FW_Rawdata == 0) || (TestResult_FW_Rawdata == 1)) {
		 nt36670_print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
	} else { /* TestResult_FW_Rawdata is -1 */
		nvt_mp_seq_printf(m, " FAIL!\n");
		if (TestResult_FWMutual == -1) {
			nvt_mp_seq_printf(m, "FW Mutual");
			nt36670_print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
		}
		if (TestResult_FW_CC == -1) {
			if (nt36670_ts->carrier_system) {
				if (TestResult_FW_CC_I == -1) {
					nvt_mp_seq_printf(m, "FW CC_I");
					nt36670_print_selftest_result(m, TestResult_FW_CC_I, RecordResult_FW_CC_I, RawData_FW_CC_I, X_Channel, Y_Channel);
				}
				if (TestResult_FW_CC_Q == -1) {
					nvt_mp_seq_printf(m, "FW CC_Q");
					nt36670_print_selftest_result(m, TestResult_FW_CC_Q, RecordResult_FW_CC_Q, RawData_FW_CC_Q, X_Channel, Y_Channel);
				}
			} else {
				nvt_mp_seq_printf(m, "FW CC");
				nt36670_print_selftest_result(m, TestResult_FW_CC, RecordResult_FW_CC, RawData_FW_CC, X_Channel, Y_Channel);
			}
		}
	}

	nvt_mp_seq_printf(m, "Noise Test");
	if ((TestResult_Noise == 0) || (TestResult_Noise == 1)) {
		nt36670_print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
	} else { /* TestResult_Noise is -1 */
		nvt_mp_seq_printf(m, " FAIL!\n");

		if (TestResult_FW_DiffMax == -1) {
			nvt_mp_seq_printf(m, "FW Diff Max");
			nt36670_print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
		}
		if (TestResult_FW_DiffMin == -1) {
			nvt_mp_seq_printf(m, "FW Diff Min");
			nt36670_print_selftest_result(m, TestResult_FW_DiffMin, RecordResult_FW_DiffMin, RawData_Diff_Min, X_Channel, Y_Channel);
		}
	}

	nvt_mp_test_result_printed = 1;

	NVT_LOG("--\n");

    return 0;
}

/*******************************************************
Description:
	Novatek touchscreen self - test sequence print start
	function.

return:
	Executive outcomes. 1-- - call next function.
	NULL-- - not call next function and sequence loop
	stop.
 *******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self - test sequence print next
	function.

return:
	Executive outcomes. NULL-- - no next and call sequence
	stop function.
 *******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self - test sequence print stop
	function.

return:
	n.a.
 *******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nt36670_nvt_selftest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show_selftest
};


/*-- -20180425-- - Add test Item */
#define SYSTEM_POWER_STATUS_SPECIAL 0x54


static void DigitalModeSet(uint32_t u32Addr, uint8_t u8D0, uint8_t u8D1)
{
	uint8_t buf[8] = {0};
	/*-- - set xdata index to Setting-- - */
	buf[0] = 0xFF;
	buf[1] = (u32Addr >> 16) & 0xFF;
	buf[2] = (u32Addr >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
	/*-- - set -- - */
	buf[0] = (u32Addr) & 0xFF;;
	buf[1] = u8D0;
	buf[2] = u8D1;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
}


static int32_t nvt_selftest_addTest_DigitalModeRawdata(int32_t *xdata)
{
	int32_t criteriaMAX = 400;
	int32_t criteriaMIN = -400;
	int32_t i32tmp = 0;
	int32_t i32Max = 0;
	int32_t i32Min = 0;
	int32_t ret = 0;

	uint8_t x_num = 0;
	uint8_t y_num = 0;

	/*0x54 */
	if (nvt_switch_FreqHopEnDis(SYSTEM_POWER_STATUS_SPECIAL)) {
		/*mutex_unlock(&nt36670_ts->lock); */
		NVT_ERR("switch SYSTEM_POWER_STATUS_SPECIAL failed!\n");
		return -EAGAIN;
	}

	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		/*mutex_unlock(&nt36670_ts->lock); */
		NVT_ERR("switch frequency hopping disable failed!\n");
		return -EAGAIN;
	}

	if (nt36670_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		/*mutex_unlock(&nt36670_ts->lock); */
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}


	/* < Chagnge to DigitalMode > */
	nt36670_change_mode(TEST_MODE_2);

	DigitalModeSet(0x24E20, 0xE4, 0x4E);

	DigitalModeSet(0x25908, 0xCE, 0x0F);
	DigitalModeSet(0x250D4, 0xCE, 0x0F);
	DigitalModeSet(0x25110, 0xCE, 0x0F);
	DigitalModeSet(0x2514C, 0xCE, 0x0F);
	DigitalModeSet(0x25188, 0xCE, 0x0F);

	DigitalModeSet(0x25200, 0xCE, 0x0F);
	DigitalModeSet(0x251C4, 0xCE, 0x0F);
	DigitalModeSet(0x2523C, 0xCE, 0x0F);
	DigitalModeSet(0x25278, 0xCE, 0x0F);
	DigitalModeSet(0x252B4, 0xCE, 0x0F);

	/*nt36670_change_mode(NORMAL_MODE); */
	{
		uint8_t buf[8] = {0};
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);
		/*-- - set 0xBB 1st-- - */
			buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
			buf[1] = 0xBB;/*HANDSHAKING_HOST_READY */
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);
		msleep(50);

		/*-- - set 0xBB 2nd-- - */
			buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
			buf[1] = 0xBB;/*HANDSHAKING_HOST_READY */
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);
		msleep(10);
	}




	if (nt36670_get_fw_info()) {
		/*mutex_unlock(&nt36670_ts->lock); */
		return -EAGAIN;
	}

	if (nt36670_get_fw_pipe() == 0)
		nt36670_read_mdata(nt36670_ts->mmap->RAW_PIPE0_ADDR, nt36670_ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nt36670_read_mdata(nt36670_ts->mmap->RAW_PIPE1_ADDR, nt36670_ts->mmap->RAW_BTN_PIPE1_ADDR);


	nt36670_get_mdata(xdata, &x_num, &y_num);

		NVT_ERR("_____show\n");
		{


			int32_t i = 0;
			int32_t j = 0;

			printk("-------------------------------\n\n");
			i32Max = 0;
			i32Min = 0;
			for (i = 0; i < (nt36670_ts->y_num); i++) {
				for (j = 0; j < ((nt36670_ts->x_num) - 1); j++) {
					i32tmp = xdata[i * nt36670_ts->x_num + j] - xdata[i * nt36670_ts->x_num + (j + 1)];
					if (i32tmp > i32Max)
						i32Max = i32tmp;
					if (i32tmp < i32Min)
						i32Min = i32tmp;
					printk("%5d, ", i32tmp);
				}
				printk("\n");
			}
			printk("tmp(Max,min)=[%d,%d]\n\n", i32Max, i32Min);

			for (i = 0; i < ((nt36670_ts->y_num) - 1); i++) {
				for (j = 0; j < (nt36670_ts->x_num); j++) {
					i32tmp = xdata[i * nt36670_ts->x_num + j] - xdata[(i + 1) * nt36670_ts->x_num + j];
					if (i32tmp > i32Max)
						i32Max = i32tmp;
					if (i32tmp < i32Min)
						i32Min = i32tmp;
					printk("%5d, ", i32tmp);
				}
				printk("\n");
			}
			printk("Criteria(Max,min)=[%d,%d]\n", criteriaMAX, criteriaMIN);
			printk("Result  (Max,min)=[%d,%d]\n", i32Max, i32Min);
			printk("-------------------------------\n");
		}


	/* Save Rawdata to CSV file */
	if (nvt_save_rawdata_to_csv(xdata, x_num, y_num, DIGITAL_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}


	if (criteriaMAX < i32Max)
		ret = -1;/* 0:PASS, -1:FAIL */
	else if (criteriaMIN > i32Min)
		ret = -1;/* 0:PASS, -1:FAIL */
	else
		ret = 0;/* 0:PASS, -1:FAIL */

	NVT_ERR("Result = [%d]\n", ret);

	return ret;
}

int32_t nt36670_CC_Doze[36 * 2]	 = {0};
static int32_t nvt_selftest_addTest_GetDozeCC(int32_t *nt36670_CC_Doze)
{
	int32_t criteriaMAX = 300;
	int32_t criteriaMIN = 70;
	int32_t i32tmp = 0;
	int32_t i32Max = criteriaMIN;
	int32_t i32Min = criteriaMAX;
	int32_t ret = 0;


	int32_t i = 0;
	int32_t j = 0;
	uint8_t buf[64] = {0};


	const int32_t  offsetAddrLen = 36;
	const int32_t  offsetTableLen = 96;
	const int32_t  ccTableLen = 88;
	const int32_t  ccSelReg_R_Len = 37;

	const int32_t  CC_Len = 36 * 2;			/*2 *Y_Num */

	int32_t idx;
	uint32_t tmpWord;
	uint8_t offsetTable[96] = {0};
	int32_t offsetAddr_L[36] = {0};
	int32_t offsetAddr_R[36] = {0};

	uint8_t ccTable[88] = {0};
	int32_t ccSelReg_L[37] = {0};/*[0]Key, [1~36]AA_Zone */
	int32_t ccSelReg_R[37] = {0};/*[0]Key, [1~36]AA_Zone */

	int32_t CC_Final[36 * 2] = {0};/*2 *Y_Num */


	int32_t i32_Addr_CCOffsetTable = 0x25E40;
	int32_t i32_Addr_CCTable = 0x25830;

	/*Get_Doze_Offset: */
	/*-- - set xdata index to CC Offset Table-- - */
	buf[0] = 0xFF;
	buf[1] = (i32_Addr_CCOffsetTable >> 16) & 0xFF;
	buf[2] = (i32_Addr_CCOffsetTable >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - read CC Offset Table-- - */
	buf[0] = (i32_Addr_CCOffsetTable) & 0xFF;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 1 + offsetTableLen / 2);
	for (i = 0; i < (offsetTableLen / 2); i++) {
		offsetTable[i] = buf[1 + i];
	}

	buf[0] = (i32_Addr_CCOffsetTable + (offsetTableLen / 2)) & 0xFF;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 1 + offsetTableLen / 2);
	for (i = 0; i < (offsetTableLen / 2); i++) {
		offsetTable[(offsetTableLen / 2) + i] = buf[1 + i];
	}

	/* Decode OFFSET_ADDR_L */
	idx = 0;
	for (i = 0; i < offsetAddrLen; i += 3) {
		tmpWord = (uint32_t)(offsetTable[idx] + (offsetTable[idx + 1] << 8) + (offsetTable[idx + 2] << 16) + (offsetTable[idx + 3] << 24));
		offsetAddr_L[i] = (int)(tmpWord & 0x3FF);
		offsetAddr_L[i + 1] = (int)((tmpWord >> 10) & 0x3FF);
		offsetAddr_L[i + 2] = (int)((tmpWord >> 20) & 0x3FF);
		idx += 4;
	}
	/* Decode OFFSET_ADDR_R */
	idx = offsetTableLen / 2;
	for (i = 0; i < offsetAddrLen; i += 3) {
		tmpWord = (uint32_t)(offsetTable[idx] + (offsetTable[idx + 1] << 8) + (offsetTable[idx + 2] << 16) + (offsetTable[idx + 3] << 24));
		offsetAddr_R[i] = (int)(tmpWord & 0x3FF);
		offsetAddr_R[i + 1] = (int)((tmpWord >> 10) & 0x3FF);
		offsetAddr_R[i + 2] = (int)((tmpWord >> 20) & 0x3FF);
		idx += 4;
	}




	/*------------ */
	/* Get 1D offset table */
	/*-- - set xdata index to  CC Table-- - */
	buf[0] = 0xFF;
	buf[1] = (i32_Addr_CCTable >> 16) & 0xFF;
	buf[2] = (i32_Addr_CCTable >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - read CC Offset Table-- - */
	buf[0] = (i32_Addr_CCTable) & 0xFF;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 1 + ccTableLen / 2);
	for (i = 0; i < (ccTableLen / 2); i++) {
		ccTable[i] = buf[1 + i];
	}
	buf[0] = (i32_Addr_CCTable + (ccTableLen / 2)) & 0xFF;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 1 + ccTableLen / 2);
	for (i = 0; i < (ccTableLen / 2); i++) {
		ccTable[(ccTableLen / 2) + i] = buf[1 + i];
	}
	/* Decode low byte of CC */
	idx = 0;
	for (i = 0; i < ccSelReg_R_Len; i++) {
		ccSelReg_R[i] = ccTable[idx];
		ccSelReg_L[i] = ccTable[idx + 2];
		idx = (i % 2 == 0) ? idx + 1 : idx + 3;
	}

	/* Decode high byte of CC & merge with low byte data */
	idx = 76; /* Depend on ctrlram structure */
	for (i = 0; i < ccSelReg_R_Len; i++) {
		ccSelReg_R[i] = (int)((ccSelReg_R[i] & 0xFF) + (((ccTable[idx] >> (i % 8)) & 0x01) << 8));
		ccSelReg_L[i] = (int)((ccSelReg_L[i] & 0xFF) + (((ccTable[idx + 2] >> (i % 8)) & 0x01) << 8));
		if ((i + 1) % 16 == 0) {
			idx += 3;
		} else if ((i + 1) % 8 == 0) {
			idx++;
		}
	}
	/* CC arrangement	 */
	for (i = 0; i < offsetAddrLen; i++) {
		if (offsetAddr_R[i] != 0x3FF)
			CC_Final[offsetAddr_R[i]] = ccSelReg_R[i + 1]; /* Offset[0] for AFE1, CC[1] for AFE1, and so on */
	}
	for (i = 0; i < offsetAddrLen; i++) {
		if (offsetAddr_L[i] != 0x3FF)
		CC_Final[offsetAddr_L[i]] = ccSelReg_L[i + 1]; /* Offset[0] for AFE1, CC[1] for AFE1, and so on */
	}

	/* Remap CC to FactoryTest UI structure */
	idx = 0;
	for (i = 0; i < CC_Len; i += 2) {
		nt36670_CC_Doze[i] = CC_Final[idx];
		nt36670_CC_Doze[i + 1] = CC_Final[CC_Len / 2 + idx];
		idx++;
	}


			/*NVT_ERR("nt36670_CC_Doze\n");
			for (i = 0; i < CC_Len; i++) {
				if (i%(CC_Len / 2) == 0)
					printk("\n");
				printk("%3d,", nt36670_CC_Doze[i]);
			}
			printk("\n");
			 */
			printk("-------------------------------\n\n");
			i32Max = criteriaMIN;/*Make update work! */
			i32Min = criteriaMAX;
			for (i = 0; i < 2; i++) {/*Y */
				for (j = 0; j < 18; j++) {/*X */
					i32tmp = nt36670_CC_Doze[i * 18 + j];
					if (i32tmp > i32Max)
						i32Max = i32tmp;
					if (i32tmp < i32Min)
						i32Min = i32tmp;
					/*printk("%5d, ", i32tmp); */
				}
				/*printk("\n"); */
			}
			printk("Criteria(Max,min)=[%d,%d]\n", criteriaMAX, criteriaMIN);
			printk("Result  (Max,min)=[%d,%d]\n", i32Max, i32Min);
			printk("-------------------------------\n\n");

	/* Save Rawdata to CSV file */
	if (nvt_save_rawdata_to_csv(nt36670_CC_Doze, 18, 2, FW_CC1D_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}

	if (criteriaMAX < i32Max)
		ret = -1;/* 0:PASS, -1:FAIL */
	else if (criteriaMIN > i32Min)
		ret = -1;/* 0:PASS, -1:FAIL */
	else
		ret = 0;/* 0:PASS, -1:FAIL			 */

	NVT_ERR("Result = [%d]\n", ret);


	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen / proc / nvt_selftest open function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
static int32_t nvt_selftest_open(struct inode *inode, struct file *file)
{
	struct device_node *np = nt36670_ts->client->dev.of_node;
	unsigned char mpcriteria[32] = {0};	/*novatek - mp - criteria - default */

	TestResult_Short = 0;
	TestResult_Short_Diff = 0;
	TestResult_Short_Base = 0;
	TestResult_Open = 0;
	TestResult_FW_Rawdata = 0;
	TestResult_FWMutual = 0;
	TestResult_FW_CC = 0;
	TestResult_FW_CC_I = 0;
	TestResult_FW_CC_Q = 0;
	TestResult_Noise = 0;
	TestResult_FW_DiffMax = 0;
	TestResult_FW_DiffMin = 0;

	NVT_LOG("++\n");

	if (mutex_lock_interruptible(&nt36670_ts->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif

	if (nt36670_get_fw_info()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("get fw info failed!\n");
		return -EAGAIN;
	}

	/* Parsing criteria from dts */
	if (of_property_read_bool(np, "novatek,mp-support-dt")) {
		u32 lcmid;

		vts_get_lcmid(nt36670_ts->vtsdev, &lcmid);
		snprintf(mpcriteria, 32, "novatek-mp-criteria-%d", lcmid);
		nt36670_mp_parse_dt(np, mpcriteria);
	} else {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("Not found novatek,mp-support-dt!\n");
		return -EAGAIN;
	}

	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("switch frequency hopping disable failed!\n");
		return -EAGAIN;
	}

	if (nt36670_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	msleep(100);

	/*-- -Enter Test Mode-- - */
	if (nt36670_clear_fw_status()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nt36670_change_mode(MP_MODE_CC);

	if (nt36670_check_fw_status()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	/*-- -FW Rawdata Test-- - */
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}
	if (nvt_read_CC(RawData_FW_CC) != 0) {
		TestResult_FW_CC = 1;
		if (nt36670_ts->carrier_system) {
			TestResult_FW_CC_I = 1;
			TestResult_FW_CC_Q = 1;
		}
	} else {
		if (nt36670_ts->carrier_system) {
			TestResult_FW_CC_I = RawDataTest_SinglePoint_Sub(RawData_FW_CC_I, RecordResult_FW_CC_I, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_I_P, PS_Config_Lmt_FW_CC_I_N);
			TestResult_FW_CC_Q = RawDataTest_SinglePoint_Sub(RawData_FW_CC_Q, RecordResult_FW_CC_Q, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_Q_P, PS_Config_Lmt_FW_CC_Q_N);
			if ((TestResult_FW_CC_I == -1) || (TestResult_FW_CC_Q == -1))
				TestResult_FW_CC = -1;
			else
				TestResult_FW_CC = 0;
		} else {
			TestResult_FW_CC = RawDataTest_SinglePoint_Sub(RawData_FW_CC, RecordResult_FW_CC, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_P, PS_Config_Lmt_FW_CC_N);
		}
	}

	if ((TestResult_FWMutual == 1) || (TestResult_FW_CC == 1)) {
		TestResult_FW_Rawdata = 1;
	} else {
		if ((TestResult_FWMutual == -1) || (TestResult_FW_CC == -1))
			TestResult_FW_Rawdata = -1;
		else
			TestResult_FW_Rawdata = 0;
	}

	/*-- - Leave Test Mode-- - */
	nt36670_change_mode(NORMAL_MODE);

	/*-- - Noise Test-- - */
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	/* 1: ERROR */
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff_Max, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);

		/* for carrier sensing system, only positive noise data */
		if (nt36670_ts->carrier_system) {
			TestResult_FW_DiffMin = 0;
		} else {
			TestResult_FW_DiffMin = RawDataTest_SinglePoint_Sub(RawData_Diff_Min, RecordResult_FW_DiffMin, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);
		}

		if ((TestResult_FW_DiffMax == -1) || (TestResult_FW_DiffMin == -1))
			TestResult_Noise = -1;
		else
			TestResult_Noise = 0;
	}

	/*--Short Test-- - */
	if (nvt_read_fw_short(RawData_Short) != 0) {
		TestResult_Short = 1; /* 1:ERROR */
		if (nt36670_ts->carrier_system) {
			TestResult_Short_Diff = 1;
			TestResult_Short_Base = 1;
		}
	} else {
		/*-- - Self Test Check -- - // 0:PASS, -1:FAIL */
		if (nt36670_ts->carrier_system) {
			TestResult_Short_Diff = RawDataTest_SinglePoint_Sub(RawData_Short_Diff, RecordResult_Short_Diff, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Diff_P, PS_Config_Lmt_Short_Diff_N);
			TestResult_Short_Base = RawDataTest_SinglePoint_Sub(RawData_Short_Base, RecordResult_Short_Base, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Base_P, PS_Config_Lmt_Short_Base_N);

			if ((TestResult_Short_Diff == -1) || (TestResult_Short_Base == -1))
				TestResult_Short = -1;
			else
				TestResult_Short = 0;
		} else {
			TestResult_Short = RawDataTest_SinglePoint_Sub(RawData_Short, RecordResult_Short, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Rawdata_P, PS_Config_Lmt_Short_Rawdata_N);
		}
	}

	/*-- - Open Test-- - */
	if (nvt_read_fw_open(RawData_Open) != 0) {
		TestResult_Open = 1;    /* 1:ERROR */
	} else {
		/*-- - Self Test Check -- - // 0:PASS, -1:FAIL */
		TestResult_Open = RawDataTest_SinglePoint_Sub(RawData_Open, RecordResult_Open, X_Channel, Y_Channel,
											PS_Config_Lmt_Open_Rawdata_P, PS_Config_Lmt_Open_Rawdata_N);
	}

	/*-- - Reset IC-- - */
	nt36670_bootloader_reset();


/*-- -20180425 */
	if ((nt36670_ts->nvt_pid == 0x5B06) | (nt36670_ts->nvt_pid == 0x5B07)) {
		NVT_LOG("----PID=0x%4X-----\n", nt36670_ts->nvt_pid);
		if (nt36670_check_fw_reset_state(RESET_STATE_REK)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("check fw reset state failed!\n");
			return -EAGAIN;
		}

		if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("switch frequency hopping disable failed!\n");
			return -EAGAIN;
		}

		if (nt36670_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("check fw reset state failed!\n");
			return -EAGAIN;
		}

		msleep(100);

		/*-- -Enter Test Mode-- - */
		if (nt36670_clear_fw_status()) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("clear fw status failed!\n");
			return -EAGAIN;
		}

		/*nt36670_change_mode(MP_MODE_CC); */
		TestResult_FW_DozeCC = nvt_selftest_addTest_GetDozeCC(RawData_FW_CC_DOZE);

		/*-- - Leave Test Mode-- - */
		nt36670_change_mode(NORMAL_MODE);

		TestResult_FW_DIGITAL = nvt_selftest_addTest_DigitalModeRawdata(RawData_FW_DIGITAL);

		nt36670_bootloader_reset();
	} /*if ((nt36670_ts->nvt_pid == 0x5B06) | (nt36670_ts->nvt_pid == 0x5B07)) */


	mutex_unlock(&nt36670_ts->lock);

	NVT_LOG("--\n");

	nvt_mp_test_result_printed = 0;

	return seq_open(file, &nt36670_nvt_selftest_seq_ops);
}
#if (_CustomerFunction_)
int nt36670_at_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result)
{
	struct device_node *np = nt36670_ts->client->dev.of_node;
	unsigned char mpcriteria[32] = {0};	/*novatek - mp - criteria - default */
	TestResult_Short = 0;
	TestResult_Short_Diff = 0;
	TestResult_Short_Base = 0;
	TestResult_Open = 0;
	TestResult_FW_Rawdata = 0;
	TestResult_FWMutual = 0;
	TestResult_FW_CC = 0;
	TestResult_FW_CC_I = 0;
	TestResult_FW_CC_Q = 0;
	TestResult_Noise = 0;
	TestResult_FW_DiffMax = 0;
	TestResult_FW_DiffMin = 0;
	/*-- */
	NVT_LOG("++\n");

    if (mutex_lock_interruptible(&nt36670_ts->lock)) {
		return -ERESTARTSYS;
	}
#if NVT_TOUCH_ESD_PROTECT
	    nvt_esd_check_enable(false);
#endif

	if (nt36670_get_fw_info()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("get fw info failed!\n");
		return -EAGAIN;
	}

	/* Parsing criteria from dts */
	if (of_property_read_bool(np, "novatek,mp-support-dt")) {
		u32 lcmid;

		vts_get_lcmid(nt36670_ts->vtsdev, &lcmid);
		snprintf(mpcriteria, 32, "novatek-mp-criteria-%d", lcmid);
		nt36670_mp_parse_dt(np, mpcriteria);
	} else {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("Not found novatek,mp-support-dt!\n");
		return -EAGAIN;
	}

	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("switch frequency hopping disable failed!\n");
		return -EAGAIN;
	}

	if (nt36670_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	msleep(100);

	/*-- -Enter Test Mode-- - */
	if (nt36670_clear_fw_status()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	nt36670_change_mode(MP_MODE_CC);

	if (nt36670_check_fw_status()) {
		mutex_unlock(&nt36670_ts->lock);
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	/*-- -FW Rawdata Test-- - */
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FWMutual:");
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}
	if (nvt_read_CC(RawData_FW_CC) != 0) {
		TestResult_FW_CC = 1;
		if (nt36670_ts->carrier_system) {
			TestResult_FW_CC_I = 1;
			TestResult_FW_CC_Q = 1;
		}
	} else {
		if (nt36670_ts->carrier_system) {
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FW_CC_I:");
			TestResult_FW_CC_I = RawDataTest_SinglePoint_Sub(RawData_FW_CC_I, RecordResult_FW_CC_I, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_I_P, PS_Config_Lmt_FW_CC_I_N);
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FW_CC_Q:");
			TestResult_FW_CC_Q = RawDataTest_SinglePoint_Sub(RawData_FW_CC_Q, RecordResult_FW_CC_Q, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_Q_P, PS_Config_Lmt_FW_CC_Q_N);
			if ((TestResult_FW_CC_I == -1) || (TestResult_FW_CC_Q == -1))
				TestResult_FW_CC = -1;
			else
				TestResult_FW_CC = 0;
		} else {
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FW_CC:");
			TestResult_FW_CC = RawDataTest_SinglePoint_Sub(RawData_FW_CC, RecordResult_FW_CC, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_CC_P, PS_Config_Lmt_FW_CC_N);
		}
	}

	if ((TestResult_FWMutual == 1) || (TestResult_FW_CC == 1)) {
		TestResult_FW_Rawdata = 1;
	} else {
		if ((TestResult_FWMutual == -1) || (TestResult_FW_CC == -1))
			TestResult_FW_Rawdata = -1;
		else
			TestResult_FW_Rawdata = 0;
	}
	/*-- - Leave Test Mode-- - */
	nt36670_change_mode(NORMAL_MODE);
	/*-- - Noise Test-- - */
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	/* 1: ERROR */
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FW DiffMax:");
		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff_Max, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);

		/* for carrier sensing system, only positive noise data */
		if (nt36670_ts->carrier_system) {
			TestResult_FW_DiffMin = 0;
		} else {
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata FW DiffMin:");
			TestResult_FW_DiffMin = RawDataTest_SinglePoint_Sub(RawData_Diff_Min, RecordResult_FW_DiffMin, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);
		}

		if ((TestResult_FW_DiffMax == -1) || (TestResult_FW_DiffMin == -1))
			TestResult_Noise = -1;
		else
			TestResult_Noise = 0;
	}

	/*--Short Test-- - */
	if (nvt_read_fw_short(RawData_Short) != 0) {
		vts_dev_info(nt36670_ts->vtsdev, "chenpeng 1");
		TestResult_Short = 1; /* 1:ERROR */
		if (nt36670_ts->carrier_system) {
			vts_dev_info(nt36670_ts->vtsdev, "chenpeng 2");
			TestResult_Short_Diff = 1;
			TestResult_Short_Base = 1;
		}
	} else {
		/*-- - Self Test Check -- - // 0:PASS, -1:FAIL */
		if (nt36670_ts->carrier_system) {
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata Short Diff:");
			TestResult_Short_Diff = RawDataTest_SinglePoint_Sub(RawData_Short_Diff, RecordResult_Short_Diff, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Diff_P, PS_Config_Lmt_Short_Diff_N);
			TestResult_Short_Base = RawDataTest_SinglePoint_Sub(RawData_Short_Base, RecordResult_Short_Base, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Base_P, PS_Config_Lmt_Short_Base_N);

			if ((TestResult_Short_Diff == -1) || (TestResult_Short_Base == -1)) {
				vts_dev_info(nt36670_ts->vtsdev, "chenpeng 5");
				TestResult_Short = -1;
			} else {
				vts_dev_info(nt36670_ts->vtsdev, "chenpeng 6");
				TestResult_Short = 0;
			}
		} else {
			vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata Short Test:");
			TestResult_Short = RawDataTest_SinglePoint_Sub(RawData_Short, RecordResult_Short, X_Channel, Y_Channel,
											PS_Config_Lmt_Short_Rawdata_P, PS_Config_Lmt_Short_Rawdata_N);
		}
	}

	/*-- - Open Test-- - */
	if (nvt_read_fw_open(RawData_Open) != 0) {
		TestResult_Open = 1;    /* 1:ERROR */
	} else {
		/*-- - Self Test Check -- - // 0:PASS, -1:FAIL */
		vts_dev_info(nt36670_ts->vtsdev, "FW Rawdata Open Test:");
		TestResult_Open = RawDataTest_SinglePoint_Sub(RawData_Open, RecordResult_Open, X_Channel, Y_Channel,
											PS_Config_Lmt_Open_Rawdata_P, PS_Config_Lmt_Open_Rawdata_N);
	}

	/*-- - Reset IC-- - */
	nt36670_bootloader_reset();

/*-- -20180425 */
	if ((nt36670_ts->nvt_pid == 0x5B06) | (nt36670_ts->nvt_pid == 0x5B07)) {
		NVT_LOG("----PID=0x%4X-----\n", nt36670_ts->nvt_pid);
		if (nt36670_check_fw_reset_state(RESET_STATE_REK)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("check fw reset state failed!\n");
			return -EAGAIN;
		}

		if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("switch frequency hopping disable failed!\n");
			return -EAGAIN;
		}

		if (nt36670_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("check fw reset state failed!\n");
			return -EAGAIN;
		}

		msleep(100);

		/*-- -Enter Test Mode-- - */
		if (nt36670_clear_fw_status()) {
			mutex_unlock(&nt36670_ts->lock);
			NVT_ERR("clear fw status failed!\n");
			return -EAGAIN;
		}

		/*nt36670_change_mode(MP_MODE_CC); */
		TestResult_FW_DozeCC = nvt_selftest_addTest_GetDozeCC(RawData_FW_CC_DOZE);

		/*-- - Leave Test Mode-- - */
		nt36670_change_mode(NORMAL_MODE);

		TestResult_FW_DIGITAL = nvt_selftest_addTest_DigitalModeRawdata(RawData_FW_DIGITAL);

		nt36670_bootloader_reset();
	} /*if ((nt36670_ts->nvt_pid == 0x5B06) | (nt36670_ts->nvt_pid == 0x5B07)) */

	/*-- */
	{
		*result = VTS_SENSOR_TEST_SUCCESS;
		if (TestResult_Short) {
			*result |= VTS_SENSOR_TEST_SHORT_FAILED;
			vts_dev_info(nt36670_ts->vtsdev, "Short test fail");
		}
		if (TestResult_Open) {
			*result |= VTS_SENSOR_TEST_OPEN_FAILED;
			vts_dev_info(nt36670_ts->vtsdev, "Open test fail");
		}
		if (TestResult_FW_Rawdata) {
			*result |= VTS_SENSOR_TEST_DATA_FAILED;
			vts_dev_info(nt36670_ts->vtsdev, "Rawdata test fail");
		}
		if (TestResult_Noise) {
			*result |= VTS_SENSOR_TEST_NOISE_FAILED;
			vts_dev_info(nt36670_ts->vtsdev, "Noise test fail");
		}

		if ((nt36670_ts->nvt_pid == 0x5B06) | (nt36670_ts->nvt_pid == 0x5B07)) {
			if (TestResult_FW_DozeCC) {
				*result |= VTS_SENSOR_TEST_DOZECC_FAILED;
				vts_dev_info(nt36670_ts->vtsdev, " DozeCC test fail");
			}
			if (TestResult_FW_DIGITAL) {
				*result |= VTS_SENSOR_TEST_DIGITAL_FAILED;
				vts_dev_info(nt36670_ts->vtsdev, " DIGITAL test fail");
			}
		}
	}
	mutex_unlock(&nt36670_ts->lock);
	NVT_LOG("--\n");

	nvt_mp_test_result_printed = 0;

	return 0;

}
#endif /*#if (_CustomerFunction_) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_selftest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_selftest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_selftest_fops = {
	.proc_open = nvt_selftest_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

#ifdef CONFIG_OF
/*******************************************************
Description:
	Novatek touchscreen parse AIN setting for array type.

return:
	n.a.
 *******************************************************/
void nt36670_mp_parse_ain(struct device_node *np, const char *name, uint8_t *array, int32_t size)
{
	struct property *data;
	int32_t len, ret;
	int32_t tmp[40];
	int32_t i;

	data = of_find_property(np, name, &len);
	len /= sizeof(u32);
	if ((!data) || (!len) || (len != size)) {
		NVT_ERR("error find %s. len=%d\n", name, len);
	} else {
		NVT_LOG("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, tmp, len);
		if (ret) {
			NVT_ERR("error reading %s. ret=%d\n", name, ret);
			return;
		}

		for (i = 0; i < len; i++)
			array[i] = tmp[i];

#if NVT_DEBUG
		printk("[NVT-ts] %s = ", name);
		for (i = 0; i < len; i++) {
			printk("%02d ", array[i]);
		}
		printk("\n");
#endif
	}
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for u32 type.

return:
	n.a.
 *******************************************************/
void nt36670_nvt_mp_parse_u32(struct device_node *np, const char *name, int32_t *para)
{
	int32_t ret;

	ret = of_property_read_u32(np, name, para);
	if (ret)
		NVT_ERR("error reading %s. ret=%d\n", name, ret);
	else {
#if NVT_DEBUG
		NVT_LOG("%s=%d\n", name, *para);
#endif
	}
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for array type.

return:
	n.a.
 *******************************************************/
void nt36670_mp_parse_array(struct device_node *np, const char *name, int32_t *array,
		int32_t size)
{
	struct property *data;
	int32_t len, ret;
#if NVT_DEBUG
	int32_t i, j, iArrayIndex = 0;
#endif

	data = of_find_property(np, name, &len);
	len /= sizeof(u32);
	if ((!data) || (!len) || (len < size)) {
		NVT_ERR("error find %s. len=%d\n", name, len);
	} else {
		NVT_LOG("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, array, len);
		if (ret) {
			NVT_ERR("error reading %s. ret=%d\n", name, ret);
			return;
		}

#if NVT_DEBUG
		NVT_LOG("%s =\n", name);
		for (j = 0; j < Y_Channel; j++) {
			printk("[NVT-ts] ");
			for (i = 0; i < X_Channel; i++) {
				iArrayIndex = j * X_Channel + i;
				printk("%d ", array[iArrayIndex]);
			}
			printk("\n");
		}
#if TOUCH_KEY_NUM > 0
		printk("[NVT-ts] ");
		for (i = 0; i < Key_Channel; i++) {
			iArrayIndex++;
			printk("%d ", array[iArrayIndex]);
		}

		printk("\n");
#endif
#endif
	}
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree mp function.

return:
	n.a.
 *******************************************************/
void nt36670_mp_parse_dt(struct device_node *root, const char *node_compatible)
{
	struct device_node *np = root;
	struct device_node *child = NULL;

	NVT_LOG("Parse mp criteria for node %s\n", node_compatible);

	/* find each MP sub - nodes */
	for_each_child_of_node(root, child) {
		/* find the specified node */
		if (of_device_is_compatible(child, node_compatible)) {
			NVT_LOG("found child node %s\n", node_compatible);
			np = child;
			break;
		}
	}
	if (child == NULL) {
		NVT_ERR("Not found compatible node %s, use default setting!\n", node_compatible);
		return;
	}

	/* MP Config */
	nt36670_nvt_mp_parse_u32(np, "IC_X_CFG_SIZE", &IC_X_CFG_SIZE);

	nt36670_nvt_mp_parse_u32(np, "IC_Y_CFG_SIZE", &IC_Y_CFG_SIZE);

#if TOUCH_KEY_NUM > 0
	nt36670_nvt_mp_parse_u32(np, "IC_KEY_CFG_SIZE", &IC_KEY_CFG_SIZE);
#endif

	nt36670_nvt_mp_parse_u32(np, "X_Channel", &X_Channel);

	nt36670_nvt_mp_parse_u32(np, "Y_Channel", &Y_Channel);

	nt36670_mp_parse_ain(np, "AIN_X", AIN_X, IC_X_CFG_SIZE);

	nt36670_mp_parse_ain(np, "AIN_Y", AIN_Y, IC_Y_CFG_SIZE);

#if TOUCH_KEY_NUM > 0
	nt36670_mp_parse_ain(np, "AIN_KEY", AIN_KEY, IC_KEY_CFG_SIZE);
#endif

	/* MP Criteria */
	if (nt36670_ts->carrier_system) {
		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Diff_P", PS_Config_Lmt_Short_Diff_P,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Diff_N", PS_Config_Lmt_Short_Diff_N,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Base_P", PS_Config_Lmt_Short_Base_P,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Base_N", PS_Config_Lmt_Short_Base_N,
				X_Channel * Y_Channel + Key_Channel);
	} else {
		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_P", PS_Config_Lmt_Short_Rawdata_P,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_N", PS_Config_Lmt_Short_Rawdata_N,
				X_Channel * Y_Channel + Key_Channel);
	}

	nt36670_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_P", PS_Config_Lmt_Open_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_N", PS_Config_Lmt_Open_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_P", PS_Config_Lmt_FW_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_N", PS_Config_Lmt_FW_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_P", PS_Config_Lmt_FW_CC_P,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_N", PS_Config_Lmt_FW_CC_N,
			X_Channel * Y_Channel + Key_Channel);

	if (nt36670_ts->carrier_system) {
		nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_I_P", PS_Config_Lmt_FW_CC_I_P,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_I_N", PS_Config_Lmt_FW_CC_I_N,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_Q_P", PS_Config_Lmt_FW_CC_Q_P,
				X_Channel * Y_Channel + Key_Channel);

		nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_CC_Q_N", PS_Config_Lmt_FW_CC_Q_N,
				X_Channel * Y_Channel + Key_Channel);
	}

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_P", PS_Config_Lmt_FW_Diff_P,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_N", PS_Config_Lmt_FW_Diff_N,
			X_Channel * Y_Channel + Key_Channel);

	nt36670_nvt_mp_parse_u32(np, "PS_Config_Diff_Test_Frame", &PS_Config_Diff_Test_Frame);

	NVT_LOG("Parse mp criteria done!\n");
}
#endif /* #ifdef CONFIG_OF */

/*******************************************************
Description:
	Novatek touchscreen MP function proc. file node
	initial function.

return:
	Executive outcomes. 0-- - succeed. -1-- - failed.
 *******************************************************/
int32_t nt36670_mp_proc_init(void)
{
	int ret = 0;
	NVT_proc_selftest_entry = proc_create("nvt_selftest", 0444, NULL, &nvt_selftest_fops);
	if (NVT_proc_selftest_entry == NULL) {
		NVT_ERR("create /proc/nvt_selftest Failed!\n");
		ret = -1;
		return ret;
	} else {
		if (nvt_mp_buffer_init()) {
			NVT_ERR("Allocate mp memory failed\n");
			ret = -1;
			return ret;
		} else {
			NVT_LOG("create /proc/nvt_selftest Succeeded!\n");
		}
		return 0;
	}
}

#endif /* #if NVT_TOUCH_MP */
