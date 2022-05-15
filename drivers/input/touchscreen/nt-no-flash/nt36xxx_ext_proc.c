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
#include <asm/uaccess.h>
#include <linux/vivo_ts_function.h>
#include <linux/uaccess.h>

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#define NVT_UPDATE "nvt_update"
#define NVT_TRIMID "nvt_trimid"
#define NVT_CMDTEST "nvt_CmdTest"	//To Trigger Test Cmd


#define SPI_TANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
static int32_t xdata_i[2048] = {0};
static int32_t xdata_q[2048] = {0};

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_fwupdate_entry;
static struct proc_dir_entry *NVT_proc_trimid_entry;

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode_Spi(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts_spi->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts_spi->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe_Spi(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts_spi->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_Spi(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[SPI_TANSFER_LENGTH + 2] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts_spi->x_num * ts_spi->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / SPI_TANSFER_LENGTH); j++) {
			//---change xdata index---
			nvt_set_page(head_addr + (XDATA_SECTOR_SIZE * i) + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts_spi->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (residual_len / SPI_TANSFER_LENGTH + 1); j++) {
			//---change xdata index---
			nvt_set_page(xdata_addr + data_len - residual_len + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			CTP_SPI_READ(ts_spi->client, buf, SPI_TANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < SPI_TANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + SPI_TANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + SPI_TANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);
	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts_spi->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts_spi->x_num * ts_spi->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata_Spi_rss_Spi(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	nvt_read_mdata_Spi(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ts_spi->x_num * ts_spi->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	nvt_read_mdata_Spi(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ts_spi->x_num * ts_spi->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));

	for (i = 0; i < (ts_spi->x_num * ts_spi->y_num + TOUCH_KEY_NUM); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata_Spi(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts_spi->x_num;
    *m_y_num = ts_spi->y_num;
    memcpy(buf, xdata, ((ts_spi->x_num * ts_spi->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts_spi->fw_ver, ts_spi->x_num, ts_spi->y_num, ts_spi->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts_spi->y_num; i++) {
		for (j = 0; j < ts_spi->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts_spi->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts_spi->x_num * ts_spi->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
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
	Novatek touchscreen xdata sequence print next
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
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

static const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops_Spi = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	nvt_change_mode_Spi(TEST_MODE_2);

	if (nvt_check_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (ts_spi->carrier_system) {
		nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->BASELINE_ADDR, ts_spi->mmap->BASELINE_Q_ADDR,
				ts_spi->mmap->BASELINE_BTN_ADDR, ts_spi->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		nvt_read_mdata_Spi(ts_spi->mmap->BASELINE_ADDR, ts_spi->mmap->BASELINE_BTN_ADDR);
	}

	nvt_change_mode_Spi(NORMAL_MODE);

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops_Spi);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	nvt_change_mode_Spi(TEST_MODE_2);

	if (nvt_check_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (ts_spi->carrier_system) {
		if (nvt_get_fw_pipe_Spi() == 0)
			nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->RAW_PIPE0_ADDR, ts_spi->mmap->RAW_PIPE0_Q_ADDR,
				ts_spi->mmap->RAW_BTN_PIPE0_ADDR, ts_spi->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->RAW_PIPE1_ADDR, ts_spi->mmap->RAW_PIPE1_Q_ADDR,
				ts_spi->mmap->RAW_BTN_PIPE1_ADDR, ts_spi->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe_Spi() == 0)
			nvt_read_mdata_Spi(ts_spi->mmap->RAW_PIPE0_ADDR, ts_spi->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_Spi(ts_spi->mmap->RAW_PIPE1_ADDR, ts_spi->mmap->RAW_BTN_PIPE1_ADDR);
	}

	nvt_change_mode_Spi(NORMAL_MODE);

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops_Spi);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	nvt_change_mode_Spi(TEST_MODE_2);

	if (nvt_check_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (ts_spi->carrier_system) {
		if (nvt_get_fw_pipe_Spi() == 0)
			nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->DIFF_PIPE0_ADDR, ts_spi->mmap->DIFF_PIPE0_Q_ADDR,
				ts_spi->mmap->DIFF_BTN_PIPE0_ADDR, ts_spi->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->DIFF_PIPE1_ADDR, ts_spi->mmap->DIFF_PIPE1_Q_ADDR,
				ts_spi->mmap->DIFF_BTN_PIPE1_ADDR, ts_spi->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (nvt_get_fw_pipe_Spi() == 0)
			nvt_read_mdata_Spi(ts_spi->mmap->DIFF_PIPE0_ADDR, ts_spi->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata_Spi(ts_spi->mmap->DIFF_PIPE1_ADDR, ts_spi->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	nvt_change_mode_Spi(NORMAL_MODE);

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops_Spi);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

int bbk_slsi_get_rawordiff_data(int which, int *data)
{
	int32_t i = 0;
	int32_t j = 0;	

	VTI("ts_spi->x_num:%d ts_spi->y_num:%d", ts_spi->x_num, ts_spi->y_num);

	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif

	if (nvt_clear_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	nvt_change_mode_Spi(TEST_MODE_2);

	if (nvt_check_fw_status_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info_Spi()) {
		mutex_unlock(&ts_spi->lock);
		return -EAGAIN;
	}

	if (which == 0) {	//rawdata
		if (ts_spi->carrier_system) {
			if (nvt_get_fw_pipe_Spi() == 0)
				nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->RAW_PIPE0_ADDR, ts_spi->mmap->RAW_PIPE0_Q_ADDR,
					ts_spi->mmap->RAW_BTN_PIPE0_ADDR, ts_spi->mmap->RAW_BTN_PIPE0_Q_ADDR);
			else
				nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->RAW_PIPE1_ADDR, ts_spi->mmap->RAW_PIPE1_Q_ADDR,
					ts_spi->mmap->RAW_BTN_PIPE1_ADDR, ts_spi->mmap->RAW_BTN_PIPE1_Q_ADDR);
		} else {
			if (nvt_get_fw_pipe_Spi() == 0)
				nvt_read_mdata_Spi(ts_spi->mmap->RAW_PIPE0_ADDR, ts_spi->mmap->RAW_BTN_PIPE0_ADDR);
			else
				nvt_read_mdata_Spi(ts_spi->mmap->RAW_PIPE1_ADDR, ts_spi->mmap->RAW_BTN_PIPE1_ADDR);
		}
	} else if (which == 1) {	//diff
		if (ts_spi->carrier_system) {
			if (nvt_get_fw_pipe_Spi() == 0)
				nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->DIFF_PIPE0_ADDR, ts_spi->mmap->DIFF_PIPE0_Q_ADDR,
					ts_spi->mmap->DIFF_BTN_PIPE0_ADDR, ts_spi->mmap->DIFF_BTN_PIPE0_Q_ADDR);
			else
				nvt_read_mdata_Spi_rss_Spi(ts_spi->mmap->DIFF_PIPE1_ADDR, ts_spi->mmap->DIFF_PIPE1_Q_ADDR,
					ts_spi->mmap->DIFF_BTN_PIPE1_ADDR, ts_spi->mmap->DIFF_BTN_PIPE1_Q_ADDR);
		} else {
			if (nvt_get_fw_pipe_Spi() == 0)
				nvt_read_mdata_Spi(ts_spi->mmap->DIFF_PIPE0_ADDR, ts_spi->mmap->DIFF_BTN_PIPE0_ADDR);
			else
				nvt_read_mdata_Spi(ts_spi->mmap->DIFF_PIPE1_ADDR, ts_spi->mmap->DIFF_BTN_PIPE1_ADDR);
		}
	}		

	/*for (i = 0; i < ts_spi->y_num; i++) {
		for (j = 0; j < ts_spi->x_num; j++) {
			//seq_printf(m, "%5d, ", xdata[i * ts_spi->x_num + j]);
			data[i * ts_spi->x_num + j] = xdata[i * ts_spi->x_num + j];
		}
		//seq_puts(m, "\n");
	}*/
	for (i = 0; i < ts_spi->y_num; i++) {
		for (j = 0; j < ts_spi->x_num; j++) {
			//seq_pryntf(m, "%5d, ", xdata[y * ts_spi->x_num + j]);
			data[j * ts_spi->y_num + i] = xdata[i * ts_spi->x_num + j];
		}
	}
	
	nvt_change_mode_Spi(NORMAL_MODE);
	mutex_unlock(&ts_spi->lock);

	return 0;
}


#if(_CustomerFunction_)//[20180525,jx]	


#define	EVENTBUFFER_STATUS_OFF 		(0x51)
#define	EVENTBUFFER_STATUS_DC		(0x52)
#define	EVENTBUFFER_STATUS_AC		(0x53)

#define FM_mode_Enter				(0x75)//[20180308,jx]Add FM Nodify
#define FM_mode_Leave				(0x76)

//#define	INT_Enable				(0x77) 
//#define	INT_Disable 			(0x78) 
#define	IDLE_Enable 				(0x79)
#define	IDLE_Disable 				(0x7A)

#define	CMD_EDGE_REJECT_VERTICAL	(0x7C)
#define	CMD_EDGE_REJECT_LEFT_Up  	(0x7D)
#define	CMD_EDGE_REJECT_RIGHT_Up 	(0x7E) 


#define	USB_PLUGOUT				(0)
#define	USB_PLUGIN				(1)

#define	IDLE_SET_Disable 		(0)
#define	IDLE_SET_Enable 		(1) 

#define	EDGE_REJECT_VERTICAL 	(1) 
#define	EDGE_REJECT_LEFT_Up 	(2) 
#define	EDGE_REJECT_RIGHT_Up	(0) 

int8_t nvt_customizeCmd(uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	NVT_LOG("++ Cmd=0x%02X\n",u8Cmd);
	for (retry = 0; retry < 20; retry++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_SPI_WRITE(ts_spi->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts_spi->client, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	NVT_LOG("--\n");
	return ret;
}


int8_t nvt_customizeCmd_WaitSet_Spi(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;
	VTI("enter");
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---read fw status---
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		CTP_SPI_READ(ts_spi->client, buf, 2);

	  	if (buf[1] >= u8WaitStatus){	//[20170815,jx]set  >= u8WaitStatus , is for A->B->C mode	  
			break;
	  	}
		msleep(10);
	}

	if (i >= retry) {
		VTI("HANDSHAKING failed, i=%d, buf[1]=0x%02X,0x%02X", i, buf[1],buf[2]);
		return -1;
	} 
	else {
		//return 0;
		VTI("HANDSHAKING OK    , i=%d, buf[1]=0x%02X,0x%02X", i, buf[1],buf[2]);
	}

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_SPI_WRITE(ts_spi->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts_spi->client, buf, 2);
		

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i >= retry)) {
		VTI("customizeCmd 0x%02X failed, buf[1]=0x%02X", u8Cmd, buf[1]);
		return -1;
	}
	
	VTI("end");
	return 0;
}

int bbk_xxx_get_chip_fw_version(int which)
{
	unsigned int ver = 0;

	if(FW_VERSION == which) {
#if NVT_TOUCH_ESD_PROTECT
		nvt_esd_check_enable(false);
#endif
		if (nvt_get_fw_info_Spi()) {
		
			return 0;
		}
		ver = ts_spi->fw_ver;
	}
	if(CONFIG_VERSION == which) {
		ver = 0;
	}
	return ver;
}


int tpd_usb_plugin_Spi(int plugin)
{
	int ret = -1;
	mutex_lock(&ts_spi->lock);
	
	switch(plugin) {
        case USB_PLUGOUT:
            NVT_LOG("usb plug [out] .\n");
            ret = nvt_customizeCmd(EVENTBUFFER_STATUS_OFF);
            if (ret < 0)
            {
                NVT_LOG("tpd_usb_plugin_Spi 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_OFF);
            }
            break;
			
        case USB_PLUGIN:
		default:			//default is AC		
            NVT_LOG("usb plug [in ] .\n");
            ret = nvt_customizeCmd(EVENTBUFFER_STATUS_AC);
            if (ret < 0)
            {
                NVT_LOG("tpd_usb_plugin_Spi 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_AC);
            }			
            break;        
    }
	mutex_unlock(&ts_spi->lock);
	return ret;
}

int bbk_xxx_set_charger_bit(int state)
{
	int32_t i32ret = 0;
	i32ret = tpd_usb_plugin_Spi(state);
	NVT_LOG("i32ret=[%d]\n",i32ret);///jx
	return	i32ret;
}

int bbk_xxx_read_charger_bit(void)
{	
	uint8_t buf[8] = {0};
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	buf[0] = 0x5C;//[20180525]5C_bit2 is new; 62 is Old!
	buf[1] = 0xFF;
	CTP_SPI_READ(ts_spi->client, buf, 2);

	VTI("charger_bit=[%d],bit=[%X]", buf[1], (buf[1] & 0x04));

	if (buf[1] & 0x04)	//[20180525]5C_bit2
		return (1);	
	else
		return (-1);		
}

int idleEnableOrDisable(int state)
{
	int32_t i32ret = 0;
	if(state == IDLE_SET_Disable)
		i32ret = nvt_customizeCmd(IDLE_Disable);
	else if(state == IDLE_SET_Enable)		
		i32ret = nvt_customizeCmd(IDLE_Enable);
	else
		i32ret = (-1);	

	VTI("para=[%d],i32ret=[%d]", state, i32ret);
	return	i32ret;		
}

int setEdgeRestainSwitch(int i32Switch)
{
	int32_t i32ret = 0;
	if(i32Switch == EDGE_REJECT_VERTICAL)		
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_VERTICAL);
	else if(i32Switch == EDGE_REJECT_LEFT_Up)		
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_LEFT_Up);
	else if(i32Switch == EDGE_REJECT_RIGHT_Up)		
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_RIGHT_Up);	
	else{
		i32ret = (-1);	
	}

	VTI("para=[%d],i32ret=[%d]", i32Switch, i32ret);
	return	i32ret;		
}

//[20180525]
//[20180110]Creat
uint8_t getFWcmdStatus(void)
{
	uint8_t buf[8] = {0};
	uint8_t u8ret = 0;
	
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	buf[0] = 0x5C;
	buf[1] = 0xFF;//[5C]
	buf[2] = 0xFF;//[5D]
	buf[3] = 0xFF;//[5E]
	buf[4] = 0xFF;//[5F]	
	CTP_SPI_READ(ts_spi->client, buf, 5);	

	NVT_LOG("buf[1,2,3,4]=[0x%02X,0x%02X,0x%02X,0x%02X]\n" ,buf[1] ,buf[2] ,buf[3] ,buf[4]);

	u8ret=buf[1];	
	return u8ret;
}



int FM_notify(int OnOff) 
{
	int32_t i32ret = 0;
	NVT_LOG("++\n");
		
	if(OnOff == 1) {
		nvt_customizeCmd(FM_mode_Enter);
		NVT_LOG("On\n");
	}else if(OnOff == 0) {
		nvt_customizeCmd(FM_mode_Leave);
		NVT_LOG("Off\n");		
	}else{	
		NVT_LOG("Parameter Error:[%d]\n",OnOff);			
		i32ret = (-1);	
	}	
	NVT_LOG("--\n");
	return i32ret;
}

#endif//(_CustomerFunction_)


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update
	function.

return:
	n.a.
*******************************************************/
//#define FWTYPE_MP     (0)
//#define FWTYPE_Normal (1)
/*static ssize_t nvt_fwupdate_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
	//uint8_t tmp = 0;
	uint8_t fwtype = FWTYPE_Normal;
	uint8_t *str = NULL;
	
	NVT_LOG("++\n");//[20180902,jx]//[20180902,jx]move &debug
	if (mutex_lock_interruptible(&ts_spi->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif // #if NVT_TOUCH_ESD_PROTECT 

	//sprintf(&tmp, "%c", buf[0]);//[20180902,jx] Disable

  // allocate buffer for spi transfer 
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		return -ENOMEM;
	}

	if (copy_from_user(str, buf, 1)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}
	
	//fwtype = simple_strtol(&tmp, NULL, 10);
	//kstrtoint (str,10,&tmp);
	//[20180902,jx] parsing, avoid call simple_strtol()	
	if (str[0]=='1')
		fwtype = FWTYPE_Normal;
	else if (str[0]=='0')
		fwtype = FWTYPE_MP;
	else
		fwtype = 0xFF;//Error parameter

	NVT_LOG("fwtype is %d\n", fwtype);
	switch (fwtype) {
		case FWTYPE_Normal:
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal);
			break;
		case FWTYPE_MP:
			nvt_update_firmware(MP_UPDATE_FIRMWARE_NAME, FWTYPE_MP);			
			break;
		default:
			NVT_ERR("fwtype error\n");
	}

	NVT_LOG("--\n");
	mutex_unlock(&ts_spi->lock);

	return count;
}*/

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update open function.
return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_fwupdate_open(struct inode *inode, struct file *file)
{
	struct nvt_fwupdate_data *dev;

	NVT_LOG("++\n");
	dev = kmalloc(sizeof(struct nvt_fwupdate_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	NVT_LOG("--\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update close function.
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_fwupdate_close(struct inode *inode, struct file *file)
{
	struct nvt_fwupdate_data *dev = file->private_data;
	
	NVT_LOG("++\n");
	if (dev)
		kfree(dev);

	NVT_LOG("--\n");
	return 0;
}
/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update read function.
return:
	Executive outcomes. 0---succeed.
History:
	[20180903,jx]Correct to use copy_from_user() 
	[20180904,jx]Parsing to use kstrtouint(),string to uint.
	[20180904,jx]Change to read node (from write node) for SELinux
*******************************************************/
//#define FWTYPE_MP     (0)
//#define FWTYPE_Normal (1)
static ssize_t nvt_fwupdate_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	uint8_t fwtype = FWTYPE_Normal;		
	int32_t ret = 0;

	NVT_LOG("++\n");

	if (count > NVT_TANSFER_LEN) {
		NVT_ERR("invalid transfer len!\n");
		return -EFAULT;
	}

	/* allocate buffer for spi transfer */
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		goto kzalloc_failed;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		ret = -EFAULT;
		goto out;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	fwtype=str[0];

	NVT_LOG("fwtype is %d\n", fwtype);

	
	switch (fwtype) {
		case FWTYPE_Normal:
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
			break;
		case FWTYPE_MP:
			nvt_update_firmware(MP_UPDATE_FIRMWARE_NAME, FWTYPE_MP, FWTYPE_REQUEST_NO);
			break;
		default:
			NVT_ERR("fwtype error\n");
	}

	
	NVT_LOG("--\n");

out:
	kfree(str);
kzalloc_failed:
	return ret;
}

static const struct file_operations nvt_fwupdateV2_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fwupdate_open,
	.read = nvt_fwupdate_read,		
	.release = nvt_fwupdate_close,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_trimid
	function.

return:
	n.a.
*******************************************************/
static int32_t c_trimid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%02X%02X%02X%02X%02X%02X\n",
			ts_spi->trimid[0], ts_spi->trimid[1], ts_spi->trimid[2],
			ts_spi->trimid[3], ts_spi->trimid[4], ts_spi->trimid[5]);

	return 0;
}

const struct seq_operations nvt_trimid_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_trimid_show
};

static int32_t nvt_trimid_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &nvt_trimid_seq_ops);
}

static const struct file_operations nvt_trimid_fops = {
	.owner = THIS_MODULE,
	.open = nvt_trimid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_CmdTest
	function.

return:
	n.a.
*******************************************************/
static int32_t c_cmdtest_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s,jxTest\n",__func__);

	return 0;
}

const struct seq_operations nvt_cmdtest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_cmdtest_show
};

static int32_t nvt_CmdTest_open(struct inode *inode, struct file *file)
{
//TestAction

	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);

	nvt_customizeCmd_WaitSet_Spi(EVENT_MAP_RESET_COMPLETE,RESET_STATE_INIT,0x13);		

/*
	bbk_xxx_set_charger_bit(USB_PLUGOUT);
		msleep(200);			
		bbk_xxx_read_charger_bit(); 		
		getFWcmdStatus();
			
	bbk_xxx_set_charger_bit(USB_PLUGIN);
		msleep(200);			
		bbk_xxx_read_charger_bit();
		getFWcmdStatus();

	idleEnableOrDisable(2);//Err Parameter
		msleep(200);
		getFWcmdStatus();		
	idleEnableOrDisable(IDLE_SET_Enable);
		msleep(200);
		getFWcmdStatus();		
	idleEnableOrDisable(IDLE_SET_Disable);
		msleep(200);
		getFWcmdStatus();

	setEdgeRestainSwitch(3);//Err Parameter
		msleep(1000);
		getFWcmdStatus();				
	setEdgeRestainSwitch(EDGE_REJECT_VERTICAL);
		msleep(1000);
		getFWcmdStatus();							
	setEdgeRestainSwitch(EDGE_REJECT_LEFT_Up);
		msleep(1000);
		getFWcmdStatus();					
	setEdgeRestainSwitch(EDGE_REJECT_RIGHT_Up); 		
		msleep(1000);
		getFWcmdStatus();			

	FM_notify(3);//Err Parameter
		msleep(1000);	
		getFWcmdStatus();					
	FM_notify(1);
		msleep(1000);	
		getFWcmdStatus();					
	FM_notify(0);
		msleep(1000);	
		getFWcmdStatus();			
	*/	
		
	//TestLog
	/*
	[ 1290.842977] [NVT-ts] tpd_usb_plugin_Spi 709: usb plug [out] .
	[ 1290.847356] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x51
	[ 1290.902305] [NVT-ts] nvt_customizeCmd 635: --
	[ 1290.905660] [NVT-ts] bbk_xxx_set_charger_bit 735: i32ret=[0]
	[ 1291.125669] [NVT-ts] bbk_xxx_read_charger_bit 749: charger_bit=[49]
	[ 1291.134719] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x31,0x00,0x01,0x00]
	[ 1291.140763] [NVT-ts] tpd_usb_plugin_Spi 719: usb plug [in ] .
	[ 1291.146044] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x53
	[ 1291.203428] [NVT-ts] nvt_customizeCmd 635: --
	[ 1291.206789] [NVT-ts] bbk_xxx_set_charger_bit 735: i32ret=[0]
	[ 1291.425899] [NVT-ts] bbk_xxx_read_charger_bit 749: charger_bit=[53]
	[ 1291.434781] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	
	[ 1291.442899] [NVT-ts] idleEnableOrDisable 767: para=[2],i32ret=[-1]
	[ 1291.655676] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.661756] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x79
	[ 1291.723288] [NVT-ts] nvt_customizeCmd 635: --
	[ 1291.726650] [NVT-ts] idleEnableOrDisable 767: para=[1],i32ret=[0]
	[ 1291.944774] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.950836] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7A
	[ 1292.003344] [NVT-ts] nvt_customizeCmd 635: --
	[ 1292.006706] [NVT-ts] idleEnableOrDisable 767: para=[0],i32ret=[0]
	[ 1292.226022] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1292.232083] [NVT-ts] setEdgeRestainSwitch 785: para=[3],i32ret=[-1]
	[ 1293.242293] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	
	[ 1293.248255] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7C
	[ 1293.301057] [NVT-ts] nvt_customizeCmd 635: --
	[ 1293.304394] [NVT-ts] setEdgeRestainSwitch 785: para=[1],i32ret=[0]
	[ 1294.322212] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1294.328170] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7D
	[ 1294.381043] [NVT-ts] nvt_customizeCmd 635: --
	[ 1294.384381] [NVT-ts] setEdgeRestainSwitch 785: para=[2],i32ret=[0]
	[ 1295.402157] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x45,0x00,0x01,0x00]
	[ 1295.408113] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7E
	[ 1295.461033] [NVT-ts] nvt_customizeCmd 635: --
	[ 1295.464374] [NVT-ts] setEdgeRestainSwitch 785: para=[0],i32ret=[0]
	[ 1296.482340] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]
	
	[ 1296.488298] [NVT-ts] FM_notify 817: ++
	[ 1296.492118] [NVT-ts] FM_notify 826: Parameter Error:[3]
	[ 1296.497238] [NVT-ts] FM_notify 829: --
	[ 1297.512574] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]
	[ 1297.518535] [NVT-ts] FM_notify 817: ++
	[ 1297.522939] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x75
	[ 1297.571110] [NVT-ts] nvt_customizeCmd 635: --
	[ 1297.574447] [NVT-ts] FM_notify 821: On
	[ 1297.578177] [NVT-ts] FM_notify 829: --
	[ 1298.594103] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x03,0x00]
	[ 1298.600198] [NVT-ts] FM_notify 817: ++
	[ 1298.603804] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x76
	[ 1298.662285] [NVT-ts] nvt_customizeCmd 635: --
	[ 1298.665628] [NVT-ts] FM_notify 824: Off
	[ 1298.669451] [NVT-ts] FM_notify 829: --
	[ 1299.684187] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]

	*/
	

	return seq_open(file, &nvt_cmdtest_seq_ops);
}

static const struct file_operations nvt_CmdTest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_CmdTest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init_Spi_Spi(void)
{
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_fw_version Succeeded!\n");
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_baseline Succeeded!\n");
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_raw Succeeded!\n");
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_diff Succeeded!\n");
	}
	
	
	
	//[20180904,jx]Change to read node (from write node) for SELinux
	//NVT_proc_fwupdate_entry = proc_create(NVT_UPDATE, 0222, NULL,&nvt_fwupdate_fops);//no use!
	NVT_proc_fwupdate_entry = proc_create(NVT_UPDATE, 0444, NULL,&nvt_fwupdateV2_fops);	
	if (NVT_proc_fwupdate_entry == NULL) {
		NVT_ERR("create proc/nvt_update Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_update Succeeded!\n");
	}

	NVT_proc_trimid_entry = proc_create(NVT_TRIMID, 0444, NULL,&nvt_trimid_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/nvt_trimid Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_trimid Succeeded!\n");
	}
	
	NVT_proc_trimid_entry = proc_create(NVT_CMDTEST, 0444, NULL,&nvt_CmdTest_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/nvt_trimid Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/nvt_trimid Succeeded!\n");
	}

	return 0;
}

#define FM_Enable                       0x75
#define FM_Disable                      0x76
static int nvt_fm_notify(int OnOff)
{
    int32_t ret = 0;

	return 0;
	
	if (ts_spi->fm_switch == 0 && 0 != OnOff) {
        ts_spi->fm_switch = 1;
        ret = nvt_customizeCmd(FM_Enable);
    } else if (ts_spi->fm_switch != 0 && 0 == OnOff) {
        ts_spi->fm_switch = 0;
        ret = nvt_customizeCmd(FM_Disable);
    }
    return ret;
}

int FM_state_rewrite(void)
{
    int32_t ret = 0;
    if (ts_spi->fm_switch) {
        ret = nvt_customizeCmd(FM_Enable);
    } else {
        ret = nvt_customizeCmd(FM_Disable);
    }
    return ret;
}

static int gama_mode_state;
static int bbk_xxx_game_mode_ctl(int state)
{
	if (state == 1) {
		VTI("enter game mode");
		gama_mode_state = 1;
	} else if (state == 0) {
		VTI("exit game mode");
		gama_mode_state = 0;
	}
	return 0;
}

int bbk_xxx_process_by_package(unsigned char *package_name)
{
    if (!strcmp(package_name, "FM_ON")) {
		VTI("FM_ON");
        nvt_fm_notify(1);
    }
	if (!strcmp(package_name, "FM_OFF")) {
		VTI("FM_OFF");
        nvt_fm_notify(0);
    }

	if (strcmp(package_name, "VivoGameMode:0") == 0) {
		VTI("VivoGameMode:0");
		bbk_xxx_game_mode_ctl(0);
	}
	if (strcmp(package_name, "VivoGameMode:1") == 0) {
		VTI("VivoGameMode:1");
		bbk_xxx_game_mode_ctl(1);
	}

	if (!strcmp(package_name, "VivoPhoneState:1")) {
		VTI("VivoPhoneState:1");
        //rf on
    }
	if (!strcmp(package_name, "VivoPhoneState:0")) {
		VTI("VivoPhoneState:0");
        //rf off
    }
	
    return 0;
}

#endif
