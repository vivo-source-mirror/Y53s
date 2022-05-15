/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
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

#if NVT_TOUCH_MP

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define MP_MODE_CC 0x41
#define FREQ_HOP_DISABLE 0x66
#define FREQ_HOP_ENABLE 0x65

#define SHORT_TEST_CSV_FILE "/data/local/tmp/ShortTest.csv"
#define OPEN_TEST_CSV_FILE "/data/local/tmp/OpenTest.csv"
#define FW_RAWDATA_CSV_FILE "/data/local/tmp/FWMutualTest.csv"
#define FW_CC_CSV_FILE "/data/local/tmp/FWCCTest.csv"
#define NOISE_TEST_CSV_FILE "/data/local/tmp/NoiseTest.csv"

#define nvt_mp_seq_printf(m, fmt, args...) do {	\
	seq_printf(m, fmt, ##args);	\
	if (!nvt_mp_test_result_printed)	\
		printk(fmt, ##args);	\
} while (0)

static uint8_t *RecordResult_Short = NULL;
static uint8_t *RecordResult_Short_Diff = NULL;
static uint8_t *RecordResult_Short_Base = NULL;
static uint8_t *RecordResult_Open = NULL;
static uint8_t *RecordResult_FWMutual = NULL;
static uint8_t *RecordResult_FW_CC = NULL;
static uint8_t *RecordResult_FW_CC_I = NULL;
static uint8_t *RecordResult_FW_CC_Q = NULL;
static uint8_t *RecordResult_FW_DiffMax = NULL;
static uint8_t *RecordResult_FW_DiffMin = NULL;

static int32_t TestResult_Short = 0;
static int32_t TestResult_Short_Diff = 0;
static int32_t TestResult_Short_Base = 0;
static int32_t TestResult_Open = 0;
static int32_t TestResult_FW_Rawdata = 0;
static int32_t TestResult_FWMutual = 0;
static int32_t TestResult_FW_CC = 0;
static int32_t TestResult_FW_CC_I = 0;
static int32_t TestResult_FW_CC_Q = 0;
static int32_t TestResult_Noise = 0;
static int32_t TestResult_FW_DiffMax = 0;
static int32_t TestResult_FW_DiffMin = 0;

static int32_t *RawData_Short = NULL;
static int32_t *RawData_Short_Diff = NULL;
static int32_t *RawData_Short_Base = NULL;
static int32_t *RawData_Open = NULL;
static int32_t *RawData_Diff = NULL;
static int32_t *RawData_Diff_Min = NULL;
static int32_t *RawData_Diff_Max = NULL;
static int32_t *RawData_FWMutual = NULL;
static int32_t *RawData_FW_CC = NULL;
static int32_t *RawData_FW_CC_I = NULL;
static int32_t *RawData_FW_CC_Q = NULL;

static struct proc_dir_entry *NVT_proc_selftest_entry = NULL;
static int8_t nvt_mp_test_result_printed = 0;

extern void ntnf_change_mode(uint8_t mode);
extern uint8_t ntnf_get_fw_pipe(void);
extern void ntnf_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr);
extern void ntnf_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num);
static int32_t nvt_mp_parse_dt(struct device_node *root, const char *node_compatible);

/*******************************************************
Description:
	Novatek touchscreen allocate buffer for mp selftest.

return:
	Executive outcomes. 0---succeed. -12---Out of memory
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
  
	return 0;
}

static void nvt_mp_buffer_deinit(void)
{
	kfree(RawData_FW_CC_Q);
	kfree(RawData_FW_CC);
	kfree(RawData_FWMutual);
	kfree(RawData_Diff_Max);
	kfree(RawData_Diff_Min);
	kfree(RawData_Diff);
	kfree(RawData_Open);
	kfree(RawData_Short_Base);
	kfree(RawData_Short);
	kfree(RecordResult_FW_DiffMin);
	kfree(RecordResult_FW_DiffMax);
	kfree(RecordResult_FW_CC_Q);
	kfree(RecordResult_FW_CC);
	kfree(RecordResult_FWMutual);
	kfree(RecordResult_Open);
	kfree(RecordResult_Short_Base);
	kfree(RecordResult_Short);
	RawData_FW_CC_I = NULL;
	RawData_Short_Diff = NULL;
	RecordResult_FW_CC_I = NULL;
	RecordResult_Short_Diff = NULL;
}
/*******************************************************
Description:
	Novatek touchscreen self-test criteria print function.

return:
	n.a.
*******************************************************/
static void nvt_print_lmt_array(int32_t *array, int32_t x_ch, int32_t y_ch)
{
	int32_t i = 0;
	int32_t j = 0;

	for (j = 0; j < y_ch; j++) {
		for(i = 0; i < x_ch; i++) {
			printk("%5d, ", array[j * x_ch + i]);
		}
		printk("\n");
	}
}

static void nvt_print_criteria(void)
{
	NVT_LOG("++\n");

	if (ntnf_spi->carrier_system) {
		//---PS_Config_Lmt_Short_Diff---
		printk("PS_Config_Lmt_Short_Diff_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Diff_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_Short_Diff_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Diff_N, X_Channel, Y_Channel);
		//---PS_Config_Lmt_Short_Base---
		printk("PS_Config_Lmt_Short_Base_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Base_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_Short_Base_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Base_N, X_Channel, Y_Channel);
	} else {
		//---PS_Config_Lmt_Short_Rawdata---
		printk("PS_Config_Lmt_Short_Rawdata_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Rawdata_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_Short_Rawdata_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_Short_Rawdata_N, X_Channel, Y_Channel);
	}

	//---PS_Config_Lmt_Open_Rawdata---
	printk("PS_Config_Lmt_Open_Rawdata_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Open_Rawdata_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_Open_Rawdata_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_Open_Rawdata_N, X_Channel, Y_Channel);

	//---PS_Config_Lmt_FW_Rawdata---
	printk("PS_Config_Lmt_FW_Rawdata_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Rawdata_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_FW_Rawdata_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Rawdata_N, X_Channel, Y_Channel);

	if (ntnf_spi->carrier_system) {
		//---PS_Config_Lmt_FW_CC_I---
		printk("PS_Config_Lmt_FW_CC_I_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_I_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_FW_CC_I_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_I_N, X_Channel, Y_Channel);
		//---PS_Config_Lmt_FW_CC_Q---
		printk("PS_Config_Lmt_FW_CC_Q_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_Q_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_FW_CC_Q_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_Q_N, X_Channel, Y_Channel);
	} else {
		//---PS_Config_Lmt_FW_CC---
		printk("PS_Config_Lmt_FW_CC_P:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_P, X_Channel, Y_Channel);
		printk("PS_Config_Lmt_FW_CC_N:\n");
		nvt_print_lmt_array(PS_Config_Lmt_FW_CC_N, X_Channel, Y_Channel);
	}

	//---PS_Config_Lmt_FW_Diff---
	printk("PS_Config_Lmt_FW_Diff_P:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Diff_P, X_Channel, Y_Channel);
	printk("PS_Config_Lmt_FW_Diff_N:\n");
	nvt_print_lmt_array(PS_Config_Lmt_FW_Diff_N, X_Channel, Y_Channel);

	NVT_LOG("--\n");
}

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
			sprintf(fbufp + iArrayIndex * 7 + y * 2, "%5d, ", rawdata[iArrayIndex]);
		}
		printk("\n");
		sprintf(fbufp + (iArrayIndex + 1) * 7 + y * 2,"\r\n");
	}

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file_path, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open %s failed\n", file_path);
		set_fs(org_fs);
		if (fbufp) {
			kfree(fbufp);
			fbufp = NULL;
		}
		return -1;
	}

	output_len = y_ch * x_ch * 7 + y_ch * 2;
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
		return -1;
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
		//---set xdata index to EVENT BUF ADDR---
		ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

		if ((buf[1] == 0xA0) || (buf[1] == 0xA1))
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("polling hand shake status failed, buf[1]=0x%02X\n", buf[1]);

		// Read back 5 bytes from offset EVENT_MAP_HOST_CMD for debug check
		ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 6);
		NVT_ERR("Read back 5 bytes from offset EVENT_MAP_HOST_CMD: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);

		return -1;
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
		//---set xdata index to EVENT BUF ADDR---
		ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch FreqHopEnDis---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = FreqHopEnDis;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

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

	NVT_LOG("++\n");

	ntnf_read_mdata(ntnf_spi->mmap->BASELINE_ADDR, ntnf_spi->mmap->BASELINE_BTN_ADDR);

	ntnf_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (ntnf_spi->carrier_system) {
				xdata[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
			} else {
				xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
			}
		}
	}

	printk("%s:\n", __func__);
	// Save Rawdata to CSV file
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
	uint32_t rawdata_cc_q_offset = 0;

	NVT_LOG("++\n");

	if (ntnf_get_fw_pipe() == 0)
		ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE1_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE1_ADDR);
	else
		ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE0_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE0_ADDR);

	ntnf_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (ntnf_spi->carrier_system) {
				xdata_tmp = xdata[iArrayIndex];
				RawData_FW_CC_I[iArrayIndex] = (uint8_t)(xdata_tmp & 0xFF);
				RawData_FW_CC_Q[iArrayIndex] = (uint8_t)((xdata_tmp >> 8) & 0xFF);
			} else {
				xdata[iArrayIndex] = (int16_t)xdata[iArrayIndex];
			}
		}
	}

	printk("%s:\n", __func__);
	if (ntnf_spi->carrier_system) {
		printk("%s:RawData_CC_I:\n", __func__);
		// Save Rawdata to CSV file
		if (nvt_save_rawdata_to_csv(RawData_FW_CC_I, X_Channel, Y_Channel, FW_CC_CSV_FILE, 0) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}

		rawdata_cc_q_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
		printk("%s:RawData_CC_Q:\n", __func__);
		// Save Rawdata to CSV file
		if (nvt_save_rawdata_to_csv(RawData_FW_CC_Q, X_Channel, Y_Channel, FW_CC_CSV_FILE, rawdata_cc_q_offset) < 0) {
			NVT_ERR("save rawdata to CSV file failed\n");
			return -EAGAIN;
		}
	} else {
		// Save Rawdata to CSV file
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

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable noise collect---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x47;
	buf[2] = 0xAA;
	buf[3] = frame_num;
	buf[4] = 0x00;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 5);
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

	NVT_LOG("++\n");

	//---Enter Test Mode---
	if (ntnf_clear_fw_status()) {
		return -EAGAIN;
	}

	frame_num = PS_Config_Diff_Test_Frame / 10;
	if (frame_num <= 0)
		frame_num = 1;
	printk("%s: frame_num=%d\n", __func__, frame_num);
	nvt_enable_noise_collect(frame_num);
	// need wait PS_Config_Diff_Test_Frame * 8.3ms
	msleep(frame_num * 83);

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

	if (ntnf_get_fw_info()) {
		return -EAGAIN;
	}

	if (ntnf_get_fw_pipe() == 0)
		ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE0_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE1_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE1_ADDR);

	ntnf_get_mdata(xdata, &x_num, &y_num);

	for (y = 0; y < y_num; y++) {
		for (x = 0; x < x_num; x++) {
			iArrayIndex = y * x_num + x;
			if (ntnf_spi->carrier_system) {
				RawData_Diff_Max[iArrayIndex] = (uint16_t)xdata[iArrayIndex];
				RawData_Diff_Min[iArrayIndex] = 0;
			} else {
				RawData_Diff_Max[iArrayIndex] = (int8_t)((xdata[iArrayIndex] >> 8) & 0xFF);
				RawData_Diff_Min[iArrayIndex] = (int8_t)(xdata[iArrayIndex] & 0xFF);
			}
		}
	}
	//---Leave Test Mode---
	ntnf_change_mode(NORMAL_MODE);

	printk("%s:RawData_Diff_Max:\n", __func__);
	// Save Rawdata to CSV file
	if (nvt_save_rawdata_to_csv(RawData_Diff_Max, X_Channel, Y_Channel, NOISE_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}

	if (!ntnf_spi->carrier_system) {
		rawdata_diff_min_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
		printk("%s:RawData_Diff_Min:\n", __func__);
		// Save Rawdata to CSV file
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

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable open test---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x45;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 5);
}

static void nvt_enable_short_test(void)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---enable short test---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x43;
	buf[2] = 0xAA;
	buf[3] = 0x02;
	buf[4] = 0x00;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 5);
}

static int32_t nvt_read_fw_open(int32_t *xdata)
{
	uint32_t raw_pipe_addr = 0;
	uint8_t *rawdata_buf = NULL;
	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t buf[128] = {0};
	NVT_LOG("++\n");

	//---Enter Test Mode---
	if (ntnf_clear_fw_status()) {
		return -EAGAIN;
	}

	nvt_enable_open_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

	rawdata_buf = (uint8_t *)kzalloc(IC_X_CFG_SIZE * IC_Y_CFG_SIZE * 2, GFP_KERNEL);
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (ntnf_get_fw_pipe() == 0)
		raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE0_ADDR;
	else
		raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE1_ADDR;

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		//---change xdata index---
		ntnf_set_page(raw_pipe_addr + y * IC_X_CFG_SIZE * 2);

		buf[0] = (uint8_t)((raw_pipe_addr + y * IC_X_CFG_SIZE * 2) & 0xFF);
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, IC_X_CFG_SIZE * 2 + 1);
		memcpy(rawdata_buf + y * IC_X_CFG_SIZE * 2, buf + 1, IC_X_CFG_SIZE * 2);
	}

	for (y = 0; y < IC_Y_CFG_SIZE; y++) {
		for (x = 0; x < IC_X_CFG_SIZE; x++) {
			if ((AIN_Y[y] != 0xFF) && (AIN_X[x] != 0xFF)) {
				xdata[AIN_Y[y] * X_Channel + AIN_X[x]] = (int16_t)((rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2] + 256 * rawdata_buf[(y * IC_X_CFG_SIZE + x) * 2 + 1]));
			}
		}
	}

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	//---Leave Test Mode---
	ntnf_change_mode(NORMAL_MODE);


	printk("%s:RawData_Open\n", __func__);
	// Save RawData to CSV file
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
	uint32_t rawdata_short_base_offset = 0;

	NVT_LOG("++\n");

	//---Enter Test Mode---
	if (ntnf_clear_fw_status()) {
		return -EAGAIN;
	}

	nvt_enable_short_test();

	if (nvt_polling_hand_shake_status()) {
		return -EAGAIN;
	}

    rawdata_buf = (uint8_t *)kzalloc(X_Channel * Y_Channel * 2, GFP_KERNEL);
	if (!rawdata_buf) {
		NVT_ERR("kzalloc for rawdata_buf failed!\n");
		return -ENOMEM;
	}

	if (ntnf_spi->carrier_system) {
		// to get short diff rawdata at pipe0
		raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE0_ADDR;
	} else {
		if (ntnf_get_fw_pipe() == 0)
			raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE0_ADDR;
		else
			raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE1_ADDR;
	}

	for (y = 0; y < Y_Channel; y++) {
		//---change xdata index---
		ntnf_set_page(raw_pipe_addr + y * X_Channel * 2);

		buf[0] = (uint8_t)((raw_pipe_addr + y * X_Channel * 2) & 0xFF);
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, X_Channel * 2 + 1);
		memcpy(rawdata_buf + y * X_Channel * 2, buf + 1, X_Channel * 2);
	}

	for (y = 0; y < Y_Channel; y++) {
		for (x = 0; x < X_Channel; x++) {
			iArrayIndex = y * X_Channel + x;
			xdata[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
		}
	}

	// for carrier sensing system to get short baseline rawdata
	if (ntnf_spi->carrier_system) {
		// to get short baseline rawdata at pipe1
		raw_pipe_addr = ntnf_spi->mmap->RAW_PIPE1_ADDR;

		for (y = 0; y < Y_Channel; y++) {
			//---change xdata index---
			ntnf_set_page(raw_pipe_addr + y * X_Channel * 2);

			buf[0] = (uint8_t)((raw_pipe_addr + y * X_Channel * 2) & 0xFF);
			NTNF_CTP_SPI_READ(ntnf_spi->client, buf, X_Channel * 2 + 1);
			memcpy(rawdata_buf + y * X_Channel * 2, buf + 1, X_Channel * 2);
		}

		for (y = 0; y < Y_Channel; y++) {
			for (x = 0; x < X_Channel; x++) {
				iArrayIndex = y * X_Channel + x;
				RawData_Short_Base[iArrayIndex] = (int16_t)(rawdata_buf[iArrayIndex * 2] + 256 * rawdata_buf[iArrayIndex * 2 + 1]);
			}
		}
	}

	if (rawdata_buf) {
		kfree(rawdata_buf);
		rawdata_buf = NULL;
	}

	//---Leave Test Mode---
	ntnf_change_mode(NORMAL_MODE);

	if (ntnf_spi->carrier_system)
		printk("%s:RawData_Short_Diff:\n", __func__);
	else
		printk("%s:RawData_Short\n", __func__);
	// Save Rawdata to CSV file
	if (nvt_save_rawdata_to_csv(xdata, X_Channel, Y_Channel, SHORT_TEST_CSV_FILE, 0) < 0) {
		NVT_ERR("save rawdata to CSV file failed\n");
		return -EAGAIN;
	}
	if (ntnf_spi->carrier_system) {
		rawdata_short_base_offset = Y_Channel * X_Channel * 7 + Y_Channel * 2;
		printk("%s:RawData_Short_Base:\n", __func__);
		// Save Rawdata to CSV file
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
	Executive outcomes. 0---passed. negative---failed.
*******************************************************/
static int32_t RawDataTest_SinglePoint_Sub(int32_t rawdata[], uint8_t RecordResult[], uint8_t x_ch, uint8_t y_ch, int32_t Rawdata_Limit_Postive[], int32_t Rawdata_Limit_Negative[])
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;
	bool isPass = true;

	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			iArrayIndex = j * x_ch + i;

			RecordResult[iArrayIndex] = 0x00; // default value for PASS

			if(rawdata[iArrayIndex] > Rawdata_Limit_Postive[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x01;

			if(rawdata[iArrayIndex] < Rawdata_Limit_Negative[iArrayIndex])
				RecordResult[iArrayIndex] |= 0x02;
		}
	}

	//---Check RecordResult---
	for (j = 0; j < y_ch; j++) {
		for (i = 0; i < x_ch; i++) {
			if (RecordResult[j * x_ch + i] != 0) {
				isPass = false;
				break;
			}
		}
	}

	if (isPass == false) {
		return -1; // FAIL
	} else {
		return 0; // PASS
	}
}

/*******************************************************
Description:
	Novatek touchscreen print self-test result function.

return:
	n.a.
*******************************************************/
static void print_selftest_result(struct seq_file *m, int32_t TestResult, uint8_t RecordResult[], int32_t rawdata[], uint8_t x_len, uint8_t y_len)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t iArrayIndex = 0;

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
			nvt_mp_seq_printf(m, "ReadData:\n");
			for (i = 0; i < y_len; i++) {
				for (j = 0; j < x_len; j++) {
					iArrayIndex = i * x_len + j;
					nvt_mp_seq_printf(m, "%5d, ", rawdata[iArrayIndex]);
				}
				nvt_mp_seq_printf(m, "\n");
			}
			break;
	}
	nvt_mp_seq_printf(m, "\n");
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show_selftest(struct seq_file *m, void *v)
{
	NVT_LOG("++\n");

	nvt_mp_seq_printf(m, "FW Version: %d\n\n", ntnf_spi->fw_ver);

	nvt_mp_seq_printf(m, "Short Test");
	if ((TestResult_Short == 0) || (TestResult_Short == 1)) {
		print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
	} else { // TestResult_Short is -1
		if (ntnf_spi->carrier_system) {
			nvt_mp_seq_printf(m, " FAIL!\n");
			if (TestResult_Short_Diff == -1) {
				nvt_mp_seq_printf(m, "Short Diff");
				print_selftest_result(m, TestResult_Short_Diff, RecordResult_Short_Diff, RawData_Short_Diff, X_Channel, Y_Channel);
			}
			if (TestResult_Short_Base == -1) {
				nvt_mp_seq_printf(m, "Short Base");
				print_selftest_result(m, TestResult_Short_Base, RecordResult_Short_Base, RawData_Short_Base, X_Channel, Y_Channel);
			}
		} else {
			print_selftest_result(m, TestResult_Short, RecordResult_Short, RawData_Short, X_Channel, Y_Channel);
		}
	}

	nvt_mp_seq_printf(m, "Open Test");
	print_selftest_result(m, TestResult_Open, RecordResult_Open, RawData_Open, X_Channel, Y_Channel);

	nvt_mp_seq_printf(m, "FW Rawdata Test");
	if ((TestResult_FW_Rawdata == 0) || (TestResult_FW_Rawdata == 1)) {
		 print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
	} else { // TestResult_FW_Rawdata is -1
		nvt_mp_seq_printf(m, " FAIL!\n");
		if (TestResult_FWMutual == -1) {
			nvt_mp_seq_printf(m, "FW Mutual");
			print_selftest_result(m, TestResult_FWMutual, RecordResult_FWMutual, RawData_FWMutual, X_Channel, Y_Channel);
		}
		if (TestResult_FW_CC == -1) {
			if (ntnf_spi->carrier_system) {
				if (TestResult_FW_CC_I == -1) {
					nvt_mp_seq_printf(m, "FW CC_I");
					print_selftest_result(m, TestResult_FW_CC_I, RecordResult_FW_CC_I, RawData_FW_CC_I, X_Channel, Y_Channel);
				}
				if (TestResult_FW_CC_Q == -1) {
					nvt_mp_seq_printf(m, "FW CC_Q");
					print_selftest_result(m, TestResult_FW_CC_Q, RecordResult_FW_CC_Q, RawData_FW_CC_Q, X_Channel, Y_Channel);
				}
			} else {
				nvt_mp_seq_printf(m, "FW CC");
				print_selftest_result(m, TestResult_FW_CC, RecordResult_FW_CC, RawData_FW_CC, X_Channel, Y_Channel);
			}
		}
	}

	nvt_mp_seq_printf(m, "Noise Test");
	if ((TestResult_Noise == 0) || (TestResult_Noise == 1)) {
		print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
	} else { // TestResult_Noise is -1
		nvt_mp_seq_printf(m, " FAIL!\n");

		if (TestResult_FW_DiffMax == -1) {
			nvt_mp_seq_printf(m, "FW Diff Max");
			print_selftest_result(m, TestResult_FW_DiffMax, RecordResult_FW_DiffMax, RawData_Diff_Max, X_Channel, Y_Channel);
		}
		if (TestResult_FW_DiffMin == -1) {
			nvt_mp_seq_printf(m, "FW Diff Min");
			print_selftest_result(m, TestResult_FW_DiffMin, RecordResult_FW_DiffMin, RawData_Diff_Min, X_Channel, Y_Channel);
		}
	}

	nvt_mp_test_result_printed = 1;

	NVT_LOG("--\n");

    return 0;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen self-test sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations ntnf_selftest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show_selftest
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_selftest open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_selftest_open(struct inode *inode, struct file *file)
{
	struct device_node *np = ntnf_spi->node;
	unsigned char mpcriteria[32] = {0};	//novatek-mp-criteria-default

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

	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---Download MP FW---
	ntnf_update_firmware(MP_UPDATE_FIRMWARE_NAME, FWTYPE_MP, FWTYPE_REQUEST_YES);

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		NVT_ERR("get fw info failed!\n");
		return -EAGAIN;
	}

	/* Parsing criteria from dts */
	if(of_property_read_bool(np, "novatek,mp-support-dt")) {
		/*
		 * Parsing Criteria by Novatek PID
		 * The string rule is "novatek-mp-criteria-<nvt_pid>"
		 * nvt_pid is 2 bytes (show hex).
		 *
		 * Ex. nvt_pid = 500A
		 *     mpcriteria = "novatek-mp-criteria-500A"
		 */
		snprintf(mpcriteria, 32, "novatek-mp-criteria-%04X", ntnf_spi->nvt_pid);

		if (nvt_mp_parse_dt(np, mpcriteria)) {
			//---Download Normal FW---
			ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
			mutex_unlock(&ntnf_spi->lock);
			NVT_ERR("mp parse device tree failed!\n");
			return -EINVAL;
		}
	} else {
		NVT_LOG("Not found novatek,mp-support-dt, use default setting\n");
		//---Print Test Criteria---
		nvt_print_criteria();
	}

	if (nvt_switch_FreqHopEnDis(FREQ_HOP_DISABLE)) {
		mutex_unlock(&ntnf_spi->lock);
		NVT_ERR("switch frequency hopping disable failed!\n");
		return -EAGAIN;
	}

	if (ntnf_check_fw_reset_state(RESET_STATE_NORMAL_RUN)) {
		mutex_unlock(&ntnf_spi->lock);
		NVT_ERR("check fw reset state failed!\n");
		return -EAGAIN;
	}

	msleep(100);

	//---Enter Test Mode---
	if (ntnf_clear_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		NVT_ERR("clear fw status failed!\n");
		return -EAGAIN;
	}

	ntnf_change_mode(MP_MODE_CC);

	if (ntnf_check_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		NVT_ERR("check fw status failed!\n");
		return -EAGAIN;
	}

	//---FW Rawdata Test---
	if (nvt_read_baseline(RawData_FWMutual) != 0) {
		TestResult_FWMutual = 1;
	} else {
		TestResult_FWMutual = RawDataTest_SinglePoint_Sub(RawData_FWMutual, RecordResult_FWMutual, X_Channel, Y_Channel,
												PS_Config_Lmt_FW_Rawdata_P, PS_Config_Lmt_FW_Rawdata_N);
	}
	if (nvt_read_CC(RawData_FW_CC) != 0) {
		TestResult_FW_CC = 1;
		if (ntnf_spi->carrier_system) {
			TestResult_FW_CC_I = 1;
			TestResult_FW_CC_Q = 1;
		}
	} else {
		if (ntnf_spi->carrier_system) {
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

	//---Leave Test Mode---
	ntnf_change_mode(NORMAL_MODE);

	//---Noise Test---
	if (nvt_read_fw_noise(RawData_Diff) != 0) {
		TestResult_Noise = 1;	// 1: ERROR
		TestResult_FW_DiffMax = 1;
		TestResult_FW_DiffMin = 1;
	} else {
		TestResult_FW_DiffMax = RawDataTest_SinglePoint_Sub(RawData_Diff_Max, RecordResult_FW_DiffMax, X_Channel, Y_Channel,
											PS_Config_Lmt_FW_Diff_P, PS_Config_Lmt_FW_Diff_N);

		// for carrier sensing system, only positive noise data
		if (ntnf_spi->carrier_system) {
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

	//--Short Test---
	if (nvt_read_fw_short(RawData_Short) != 0) {
		TestResult_Short = 1; // 1:ERROR
		if (ntnf_spi->carrier_system) {
			TestResult_Short_Diff = 1;
			TestResult_Short_Base = 1;
		}
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		if (ntnf_spi->carrier_system) {
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

	//---Open Test---
	if (nvt_read_fw_open(RawData_Open) != 0) {
		TestResult_Open = 1;    // 1:ERROR
	} else {
		//---Self Test Check --- // 0:PASS, -1:FAIL
		TestResult_Open = RawDataTest_SinglePoint_Sub(RawData_Open, RecordResult_Open, X_Channel, Y_Channel,
											PS_Config_Lmt_Open_Rawdata_P, PS_Config_Lmt_Open_Rawdata_N);
	}

	//---Download Normal FW---
	ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("--\n");

	nvt_mp_test_result_printed = 0;

	return seq_open(file, &ntnf_selftest_seq_ops);
}

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
int32_t ntnf_mp_parse_ain(struct device_node *np, const char *name, uint8_t *array, int32_t size)
{
	struct property *data;
	int32_t len, ret;
	int32_t tmp[40];
	int32_t i;

	data = of_find_property(np, name, &len);
	len /= sizeof(u32);
	if ((!data) || (!len) || (len != size)) {
		NVT_ERR("error find %s. len=%d\n", name, len);
		return -1;
	} else {
		NVT_LOG("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, tmp, len);
		if (ret) {
			NVT_ERR("error reading %s. ret=%d\n", name, ret);
			return -1;
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

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for u32 type.

return:
	n.a.
*******************************************************/
int32_t ntnf_mp_parse_u32(struct device_node *np, const char *name, int32_t *para)
{
	int32_t ret;

	ret = of_property_read_u32(np, name, para);
	if (ret) {
		NVT_ERR("error reading %s. ret=%d\n", name, ret);
		return -1;
	} else {
#if NVT_DEBUG
		NVT_LOG("%s=%d\n", name, *para);
#endif
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse criterion for array type.

return:
	n.a.
*******************************************************/
int32_t ntnf_mp_parse_array(struct device_node *np, const char *name, int32_t *array,
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
		return -1;
	} else {
		NVT_LOG("%s. len=%d\n", name, len);
		ret = of_property_read_u32_array(np, name, array, len);
		if (ret) {
			NVT_ERR("error reading %s. ret=%d\n", name, ret);
			return -1;
		}

#if NVT_DEBUG
		NVT_LOG("%s =\n", name);
		for (j = 0; j < Y_Channel; j++) {
			printk("[NVT-ts] ");
			for (i = 0; i < X_Channel; i++) {
				iArrayIndex = j * X_Channel + i;
				printk("%5d, ", array[iArrayIndex]);
			}
			printk("\n");
		}
#endif
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree mp function.

return:
	n.a.
*******************************************************/
static int32_t nvt_mp_parse_dt(struct device_node *root, const char *node_compatible)
{
	struct device_node *np = root;
	struct device_node *child = NULL;

	NVT_LOG("Parse mp criteria for node %s\n", node_compatible);

	/* find each MP sub-nodes */
	for_each_child_of_node(root, child) {
		/* find the specified node */
		if (of_device_is_compatible(child, node_compatible)) {
			NVT_LOG("found child node %s\n", node_compatible);
			np = child;
			break;
		}
	}
	if (child == NULL) {
		NVT_ERR("Not found compatible node %s!\n", node_compatible);
		return -1;
	}

	/* MP Config*/
	if (ntnf_mp_parse_u32(np, "IC_X_CFG_SIZE", &IC_X_CFG_SIZE))
		return -1;

	if (ntnf_mp_parse_u32(np, "IC_Y_CFG_SIZE", &IC_Y_CFG_SIZE))
		return -1;

	if (ntnf_mp_parse_u32(np, "X_Channel", &X_Channel))
		return -1;

	if (ntnf_mp_parse_u32(np, "Y_Channel", &Y_Channel))
		return -1;

	if (ntnf_mp_parse_ain(np, "AIN_X", AIN_X, IC_X_CFG_SIZE))
		return -1;

	if (ntnf_mp_parse_ain(np, "AIN_Y", AIN_Y, IC_Y_CFG_SIZE))
		return -1;

	/* MP Criteria */
	if (ntnf_spi->carrier_system) {
		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Diff_P", PS_Config_Lmt_Short_Diff_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Diff_N", PS_Config_Lmt_Short_Diff_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Base_P", PS_Config_Lmt_Short_Base_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Base_N", PS_Config_Lmt_Short_Base_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;
	} else {
		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_P", PS_Config_Lmt_Short_Rawdata_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Short_Rawdata_N", PS_Config_Lmt_Short_Rawdata_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;
	}

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_P", PS_Config_Lmt_Open_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_Open_Rawdata_N", PS_Config_Lmt_Open_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_P", PS_Config_Lmt_FW_Rawdata_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_Rawdata_N", PS_Config_Lmt_FW_Rawdata_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_spi->carrier_system) {
		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_I_P", PS_Config_Lmt_FW_CC_I_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_I_N", PS_Config_Lmt_FW_CC_I_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_Q_P", PS_Config_Lmt_FW_CC_Q_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_Q_N", PS_Config_Lmt_FW_CC_Q_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;
	} else {
		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_P", PS_Config_Lmt_FW_CC_P,
				X_Channel * Y_Channel + Key_Channel))
			return -1;

		if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_CC_N", PS_Config_Lmt_FW_CC_N,
				X_Channel * Y_Channel + Key_Channel))
			return -1;
	}

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_P", PS_Config_Lmt_FW_Diff_P,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_mp_parse_array(np, "PS_Config_Lmt_FW_Diff_N", PS_Config_Lmt_FW_Diff_N,
			X_Channel * Y_Channel + Key_Channel))
		return -1;

	if (ntnf_mp_parse_u32(np, "PS_Config_Diff_Test_Frame", &PS_Config_Diff_Test_Frame))
		return -1;

	NVT_LOG("Parse mp criteria done!\n");

	return 0;
}
#endif /* #ifdef CONFIG_OF */

/*******************************************************
Description:
	Novatek touchscreen MP function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t ntnf_mp_proc_init(void)
{	
	NVT_proc_selftest_entry = proc_create("nvt_selftest", 0666, NULL, &nvt_selftest_fops);
	if (NVT_proc_selftest_entry == NULL) {
		NVT_ERR("create /proc/nvt_selftest Failed!\n");
		return -ENOMEM;
	}

	if(nvt_mp_buffer_init()) {
		proc_remove(NVT_proc_selftest_entry);
		NVT_ERR("Allocate mp memory failed\n");
		return -ENOMEM;
	}

	return 0;
}

void nvt_mp_proc_deinit_Spi(void)
{
	nvt_mp_buffer_deinit();
	proc_remove(NVT_proc_selftest_entry);
}
#else
int32_t ntnf_mp_proc_init(void)
{	
	return 0;
}

void nvt_mp_proc_deinit_Spi(void)
{
}
#endif
