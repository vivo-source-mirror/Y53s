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

//static uint8_t xdata_tmp[2048] = {0};
//static int32_t xdata[2048] = {0};
//static int32_t xdata_i[2048] = {0};
//static int32_t xdata_q[2048] = {0};

#define XDATE_BUF_SIZE 2048
static uint8_t *xdata_tmp = NULL;
static int32_t *xdata = NULL;
static int32_t *xdata_i = NULL;
static int32_t *xdata_q = NULL;

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_fwupdate_entry;
static struct proc_dir_entry *NVT_proc_trimid_entry;
static struct proc_dir_entry *NVT_proc_cmdtest_entry;

#if NVT_TOUCH_ESD_PROTECT
extern struct delayed_work nvt_esd_check_work_v2;
extern u32 esd_check_support;
#endif

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void ntnf_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t ntnf_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void ntnf_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
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
	data_len = ntnf_spi->x_num * ntnf_spi->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---read xdata by SPI_TANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / SPI_TANSFER_LENGTH); j++) {
			//---change xdata index---
			ntnf_set_page(head_addr + (XDATA_SECTOR_SIZE * i) + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			NTNF_CTP_SPI_READ(ntnf_spi->client, buf, SPI_TANSFER_LENGTH + 1);

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
			ntnf_set_page(xdata_addr + data_len - residual_len + (SPI_TANSFER_LENGTH * j));

			//---read data---
			buf[0] = SPI_TANSFER_LENGTH * j;
			NTNF_CTP_SPI_READ(ntnf_spi->client, buf, SPI_TANSFER_LENGTH + 1);

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

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data from IQ to rss function.

return:
	n.a.
*******************************************************/
void ntnf_read_mdata_rss_Spi(uint32_t xdata_i_addr, uint32_t xdata_q_addr, uint32_t xdata_btn_i_addr, uint32_t xdata_btn_q_addr)
{
	int i = 0;

	ntnf_read_mdata(xdata_i_addr, xdata_btn_i_addr);
	memcpy(xdata_i, xdata, ((ntnf_spi->x_num * ntnf_spi->y_num) * sizeof(int32_t)));

	ntnf_read_mdata(xdata_q_addr, xdata_btn_q_addr);
	memcpy(xdata_q, xdata, ((ntnf_spi->x_num * ntnf_spi->y_num) * sizeof(int32_t)));

	for (i = 0; i < (ntnf_spi->x_num * ntnf_spi->y_num); i++) {
		xdata[i] = (int32_t)int_sqrt((unsigned long)(xdata_i[i] * xdata_i[i]) + (unsigned long)(xdata_q[i] * xdata_q[i]));
	}
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void ntnf_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ntnf_spi->x_num;
    *m_y_num = ntnf_spi->y_num;
    memcpy(buf, xdata, ((ntnf_spi->x_num * ntnf_spi->y_num) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ntnf_spi->fw_ver, ntnf_spi->x_num, ntnf_spi->y_num, ntnf_spi->max_button_num);
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

	for (i = 0; i < ntnf_spi->y_num; i++) {
		for (j = 0; j < ntnf_spi->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ntnf_spi->x_num + j]);
		}
		seq_puts(m, "\n");
	}

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

const struct seq_operations ntnf_seq_ops = {
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
	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_fw_version_fops = {
	.proc_open = nvt_fw_version_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (ntnf_clear_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	ntnf_change_mode(TEST_MODE_2);

	if (ntnf_check_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_spi->carrier_system) {
		ntnf_read_mdata_rss_Spi(ntnf_spi->mmap->BASELINE_ADDR, ntnf_spi->mmap->BASELINE_Q_ADDR,
				ntnf_spi->mmap->BASELINE_BTN_ADDR, ntnf_spi->mmap->BASELINE_BTN_Q_ADDR);
	} else {
		ntnf_read_mdata(ntnf_spi->mmap->BASELINE_ADDR, ntnf_spi->mmap->BASELINE_BTN_ADDR);
	}

	ntnf_change_mode(NORMAL_MODE);

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &ntnf_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_baseline_fops = {
	.proc_open = nvt_baseline_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (ntnf_clear_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	ntnf_change_mode(TEST_MODE_2);

	if (ntnf_check_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_spi->carrier_system) {
		if (ntnf_get_fw_pipe() == 0)
			ntnf_read_mdata_rss_Spi(ntnf_spi->mmap->RAW_PIPE0_ADDR, ntnf_spi->mmap->RAW_PIPE0_Q_ADDR,
				ntnf_spi->mmap->RAW_BTN_PIPE0_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE0_Q_ADDR);
		else
			ntnf_read_mdata_rss_Spi(ntnf_spi->mmap->RAW_PIPE1_ADDR, ntnf_spi->mmap->RAW_PIPE1_Q_ADDR,
				ntnf_spi->mmap->RAW_BTN_PIPE1_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE1_Q_ADDR);
	} else {
		if (ntnf_get_fw_pipe() == 0)
			ntnf_read_mdata(ntnf_spi->mmap->RAW_PIPE0_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE0_ADDR);
		else
			ntnf_read_mdata(ntnf_spi->mmap->RAW_PIPE1_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE1_ADDR);
	}

	ntnf_change_mode(NORMAL_MODE);

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &ntnf_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_raw_fops = {
	.proc_open = nvt_raw_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif	

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (ntnf_clear_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	ntnf_change_mode(TEST_MODE_2);

	if (ntnf_check_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_spi->carrier_system) {
		if (ntnf_get_fw_pipe() == 0)
			ntnf_read_mdata_rss_Spi(ntnf_spi->mmap->DIFF_PIPE0_ADDR, ntnf_spi->mmap->DIFF_PIPE0_Q_ADDR,
				ntnf_spi->mmap->DIFF_BTN_PIPE0_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE0_Q_ADDR);
		else
			ntnf_read_mdata_rss_Spi(ntnf_spi->mmap->DIFF_PIPE1_ADDR, ntnf_spi->mmap->DIFF_PIPE1_Q_ADDR,
				ntnf_spi->mmap->DIFF_BTN_PIPE1_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE1_Q_ADDR);
	} else {
		if (ntnf_get_fw_pipe() == 0)
			ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE0_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE1_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE1_ADDR);
	}

	ntnf_change_mode(NORMAL_MODE);

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("--\n");

	return seq_open(file, &ntnf_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_diff_fops = {
	.proc_open = nvt_diff_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

int ntnf_get_frame(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size)
{
	int32_t i = 0;
	int32_t j = 0;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}

	VTI("ntnf_spi->x_num:%d ntnf_spi->y_num:%d vts_frame_type:%d", ntnf_spi->x_num, ntnf_spi->y_num, which);

	if (which != VTS_FRAME_MUTUAL_RAW && which != VTS_FRAME_MUTUAL_DELTA) {
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif

	if (ntnf_clear_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	ntnf_change_mode(TEST_MODE_2);

	if (ntnf_check_fw_status()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (ntnf_get_fw_info()) {
		mutex_unlock(&ntnf_spi->lock);
		return -EAGAIN;
	}

	if (which == VTS_FRAME_MUTUAL_RAW) {	//rawdata
			if (ntnf_get_fw_pipe() == 0)
				ntnf_read_mdata(ntnf_spi->mmap->RAW_PIPE0_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE0_ADDR);
			else
				ntnf_read_mdata(ntnf_spi->mmap->RAW_PIPE1_ADDR, ntnf_spi->mmap->RAW_BTN_PIPE1_ADDR);
		
	} else if (which == VTS_FRAME_MUTUAL_DELTA) {	//diff
			if (ntnf_get_fw_pipe() == 0)
				ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE0_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE0_ADDR);
			else
				ntnf_read_mdata(ntnf_spi->mmap->DIFF_PIPE1_ADDR, ntnf_spi->mmap->DIFF_BTN_PIPE1_ADDR);
	}		

	/*for (i = 0; i < ntnf_spi->y_num; i++) {
		for (j = 0; j < ntnf_spi->x_num; j++) {
			//seq_printf(m, "%5d, ", xdata[i * ntnf_spi->x_num + j]);
			data[i * ntnf_spi->x_num + j] = xdata[i * ntnf_spi->x_num + j];
		}
		//seq_puts(m, "\n");
	}*/
	for (i = 0; i < ntnf_spi->y_num; i++) {
		for (j = 0; j < ntnf_spi->x_num; j++) {
			//seq_pryntf(m, "%5d, ", xdata[y * ntnf_spi->x_num + j]);
			data[j * ntnf_spi->y_num + i] = xdata[i * ntnf_spi->x_num + j];
		}
	}
	
	ntnf_change_mode(NORMAL_MODE);
	mutex_unlock(&ntnf_spi->lock);

	return 0;
}


#if(_CustomerFunction_)//[20180525,jx]	

#define 	FW_STATUS_4_VIVO 			(0x41)
#define	EVENTBUFFER_STATUS_OFF 		(0x51)
#define	EVENTBUFFER_STATUS_DC		(0x52)
#define	EVENTBUFFER_STATUS_AC		(0x53)

#define FM_mode_Enter				(0x75)//[20180308,jx]Add FM Nodify
#define FM_mode_Leave				(0x76)

#define Typing_mode_Enter				(0x73)//
#define Typing_mode_Leave				(0x74)

//#define	INT_Enable				(0x77) 
//#define	INT_Disable 			(0x78) 
#define	IDLE_Enable 				(0x79)
#define	IDLE_Disable 				(0x7A)

#define	CMD_EDGE_REJECT_VERTICAL	(0x7C)
#define	CMD_EDGE_REJECT_LEFT_Up  	(0x7D)
#define	CMD_EDGE_REJECT_RIGHT_Up 	(0x7E)

#define CMD_CUSTOM					(0x7F)
#define CMD_CUSTOM_GAME_MODE		(0x03)

#define	USB_PLUGOUT				(0)
#define	USB_PLUGIN				(1)

#define	IDLE_SET_Disable 		(0)
#define	IDLE_SET_Enable 		(1) 

#define	EDGE_REJECT_VERTICAL 	(1) 
#define	EDGE_REJECT_LEFT_Up 	(2) 
#define	EDGE_REJECT_RIGHT_Up	(0) 

int8_t ntnf_customizeCmd(uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	#if (NVTFLASH_WORK_PROTECT)
		if(atomic_read(&u8_NT36xxx_flashWorking)){
			NVT_ERR("EXECUTE FAIL!!! u8_NT36xxx_flashWorking=[%d]\n", atomic_read(&u8_NT36xxx_flashWorking));
			NVT_LOG("--\n");
			return -1;
		}
	#endif

	NVT_LOG("++ Cmd=0x%02X\n",u8Cmd);
	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	for (retry = 0; retry < 20; retry++) {


		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

		if (buf[1] == 0x00 || (retry == 1 && buf[1] == 0xfe))
			break;
	}

	if (unlikely(retry == 20) || (retry == 1 && buf[1] == 0xfe)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	NVT_LOG("--\n");
	return ret;
}

int8_t nvt_customizeExtCmd(uint8_t u8Cmd, uint8_t u8subCmd, uint8_t u8Param)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	NVT_LOG("++ Cmd=0x%02X,0x%02X,0x%02X\n",u8Cmd, u8subCmd, u8Param);

	#if (NVTFLASH_WORK_PROTECT)
		if(atomic_read(&u8_NT36xxx_flashWorking)){
			NVT_ERR("EXECUTE FAIL!!! u8_NT36xxx_flashWorking=[%d]\n", atomic_read(&u8_NT36xxx_flashWorking));
			NVT_LOG("--\n");
			return -1;
		}
	#endif

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	for (retry = 0; retry < 20; retry++) {


		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8subCmd;
		buf[3] = u8Param;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 4);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

		if (buf[1] == 0x00 || (retry == 1 && buf[1] == 0xfe))
			break;
	}

	if (unlikely(retry == 20) || (retry == 1 && buf[1] == 0xfe)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	NVT_LOG("--\n");
	return ret;
}

int8_t nvt_customizeExtCmd_WaitSet(uint8_t u8WaitAddr, RST_COMPLETE_STATE check_reset_state, uint8_t *u8Cmd, uint8_t len)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	VTI("++ Addr:0x%02X,WaitStatus:0x%02X,len: %d\n",(unsigned int)u8WaitAddr, (unsigned int)check_reset_state, len);

	if(len > 7)
		len = 7;
	
	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	for (i = 0; i < retry_max; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---read fw status---
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			break;
		}


		if(unlikely(i > retry_max)) {
			NVT_ERR("HANDSHAKING failed, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				i, buf[1], buf[2], buf[3], buf[4], buf[5]);
			break;
		}

		usleep_range(10000, 10000);

	}

	for (i = 0; i < retry_max; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		memcpy(&buf[1], u8Cmd, len);
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, len+1);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);//[20180621]Correct

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i >= retry_max)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd[0], buf[1]);
		return -1;
	}
	

	NVT_LOG("--\n");
	return 0;
}

int8_t ntnf_customizeCmd_WaitSet_Spi(uint8_t u8WaitAddr, RST_COMPLETE_STATE check_reset_state, uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;
	VTI("enter");
	//---set xdata index to EVENT BUF ADDR---

	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);
	for (i = 0; i < retry_max; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---read fw status---
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 6);

		if (buf[1] >= check_reset_state  && buf[1] <= RESET_STATE_MAX) {	//[20170815,jx]set  >= u8WaitStatus , is for A->B->C mode
			break;
		}
		if (i >= retry_max) {
			VTI("HANDSHAKING failed, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
					i, buf[1], buf[2], buf[3], buf[4], buf[5]);
			return -1;
		} 
		usleep_range(10000, 10000);

	}

	for (i = 0; i < retry_max; i++) {
		//---set xdata index to EVENT BUF ADDR---
		//ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);
		

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(i >= retry_max)) {
		VTI("customizeCmd 0x%02X failed, buf[1]=0x%02X", u8Cmd, buf[1]);
		return -1;
	}
	
	VTI("end");
	return 0;
}

int ntnf_get_chip_fw_version(struct vts_device *vtsdev, u64 *version)
{
	unsigned int fwver = 0;
	unsigned int cfgver = 0;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}
	

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable_v2(false);
#endif
	if (ntnf_get_fw_info()) {
	
		return 0;
	}
	fwver = ntnf_spi->fw_ver;
	*version = (fwver << 16 | cfgver);
	return 0;
}
int ntnf_get_touch_ic_mode(struct vts_device *vtsdev)
{
	uint8_t buf[8]= {0};
	int ret;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return 0;
		}
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | FW_STATUS_4_VIVO);

	//---read fw status---
	buf[0] = FW_STATUS_4_VIVO;
	buf[1] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

	ret=(buf[1] & 0x01)|(buf[1]&0x02);
	return ret;
}

int ntnf_tpd_usb_plugin(int plugin)
{
	int ret = -1;
	mutex_lock(&ntnf_spi->lock);
	
	switch(plugin) {
        case USB_PLUGOUT:
            NVT_LOG("usb plug [out] .\n");
            ret = ntnf_customizeCmd(EVENTBUFFER_STATUS_OFF);
            if (ret < 0)
            {
                NVT_LOG("ntnf_tpd_usb_plugin 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_OFF);
            }
            break;
			
        case USB_PLUGIN:
		default:			//default is AC		
            NVT_LOG("usb plug [in ] .\n");
            ret = ntnf_customizeCmd(EVENTBUFFER_STATUS_AC);
            if (ret < 0)
            {
                NVT_LOG("ntnf_tpd_usb_plugin 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_AC);
            }			
            break;        
    }
	mutex_unlock(&ntnf_spi->lock);
	return ret;
}

int ntnf_set_charger_bit(struct vts_device *vtsdev, int state)
{
	int32_t i32ret = 0;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return 0;
	}
	ntnf_spi->charging = state;
	i32ret = ntnf_tpd_usb_plugin(state);
	NVT_LOG("i32ret=[%d]\n",i32ret);///jx
	return	i32ret;
}

int ntnf_read_charger_bit(void)
{	
	uint8_t buf[8] = {0};
	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	buf[0] = 0x5C;//[20180525]5C_bit2 is new; 62 is Old!
	buf[1] = 0xFF;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

	VTI("charger_bit=[%d],bit=[%X]", buf[1], (buf[1] & 0x04));

	if (buf[1] & 0x04)	//[20180525]5C_bit2
		return (1);	
	else
		return (-1);		
}

int ntnf_set_idle(struct vts_device *vtsdev, int state)
{
	int32_t i32ret = 0;
	u32 idle_time;
	
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}

	vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);

	if (idle_time == 0) {
		if (state == IDLE_SET_Disable)
			i32ret = ntnf_customizeCmd(IDLE_Disable);
		else if (state == IDLE_SET_Enable)		
			i32ret = ntnf_customizeCmd(IDLE_Enable);
		else
			i32ret = (-1);	
	} else if (idle_time <= 0xff) {
		if (state == IDLE_SET_Disable)
			i32ret = nvt_customizeExtCmd(CMD_CUSTOM, CMD_CUSTOM_GAME_MODE, idle_time);
		else if (state == IDLE_SET_Enable)	
			i32ret = nvt_customizeExtCmd(CMD_CUSTOM, CMD_CUSTOM_GAME_MODE, 1);
		else
			i32ret = (-1);	
	} else {
		i32ret = (-1);
		VTE("invalid idle time");
	}

	VTI("para=[%d],i32ret=[%d]", state, i32ret);
	return 0;
}

int ntnf_set_rotation(struct vts_device *vtsdev, int i32Switch)
{
	int32_t i32ret = 0;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return 0;
	}
	if(i32Switch == EDGE_REJECT_VERTICAL)		
		i32ret = ntnf_customizeCmd(CMD_EDGE_REJECT_VERTICAL);
	else if(i32Switch == EDGE_REJECT_LEFT_Up)		
		i32ret = ntnf_customizeCmd(CMD_EDGE_REJECT_LEFT_Up);
	else if(i32Switch == EDGE_REJECT_RIGHT_Up)		
		i32ret = ntnf_customizeCmd(CMD_EDGE_REJECT_RIGHT_Up);	
	else{
		i32ret = (-1);	
	}

	ntnf_spi->rotation = i32Switch;
	VTI("para=[%d],i32ret=[%d]", i32Switch, i32ret);
	return	i32ret;		
}

//[20180525]
//[20180110]Creat
uint8_t ntnf_getFWcmdStatus(void)
{
	uint8_t buf[8] = {0};
	uint8_t u8ret = 0;
	
	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	buf[0] = 0x5C;
	buf[1] = 0xFF;//[5C]
	buf[2] = 0xFF;//[5D]
	buf[3] = 0xFF;//[5E]
	buf[4] = 0xFF;//[5F]	
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);	

	NVT_LOG("buf[1,2,3,4]=[0x%02X,0x%02X,0x%02X,0x%02X]\n" ,buf[1] ,buf[2] ,buf[3] ,buf[4]);

	u8ret=buf[1];	
	return u8ret;
}



int ntnf_fm_notify(int OnOff) 
{
	int32_t i32ret = 0;
	NVT_LOG("++\n");
		
	if(OnOff == 1) {
		ntnf_customizeCmd(FM_mode_Enter);
		NVT_LOG("On\n");
	}else if(OnOff == 0) {
		ntnf_customizeCmd(FM_mode_Leave);
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
	if (mutex_lock_interruptible(&ntnf_spi->lock)) {
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
			ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal);
			break;
		case FWTYPE_MP:
			ntnf_update_firmware(MP_UPDATE_FIRMWARE_NAME, FWTYPE_MP);			
			break;
		default:
			NVT_ERR("fwtype error\n");
	}

	NVT_LOG("--\n");
	mutex_unlock(&ntnf_spi->lock);

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

	if (ntnf_spi->mmap == NULL) {
		VTE("check NT ic failed");
		return -EFAULT;;
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
	if (esd_check_support) {
		cancel_delayed_work_sync(&nvt_esd_check_work_v2);
		nvt_esd_check_enable_v2(false);
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	fwtype=str[0];

	NVT_LOG("fwtype is %d\n", fwtype);

	mutex_lock(&ntnf_spi->lock);
	switch (fwtype) {
		case FWTYPE_Normal:
			ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
			break;
		case FWTYPE_MP:
			ntnf_update_firmware(MP_UPDATE_FIRMWARE_NAME, FWTYPE_MP, FWTYPE_REQUEST_YES);
			break;
		default:
			NVT_ERR("fwtype error\n");
	}
	mutex_unlock(&ntnf_spi->lock);
	
	NVT_LOG("--\n");

out:
	kfree(str);
kzalloc_failed:
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_fwupdateV2_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fwupdate_open,
	.read = nvt_fwupdate_read,		
	.release = nvt_fwupdate_close,
};
#else
static const struct proc_ops nvt_fwupdateV2_fops = {
	.proc_open = nvt_fwupdate_open,
	.proc_read = nvt_fwupdate_read,		
	.proc_release = nvt_fwupdate_close,
};
#endif	

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_trimid
	function.

return:
	n.a.
*******************************************************/
static int32_t c_trimid_show(struct seq_file *m, void *v)
{
	if (ntnf_spi->trimid == NULL) {
		VTE("check NT ic failed");
		return -EFAULT;;
	}
	
	seq_printf(m, "%02X%02X%02X%02X%02X%02X\n",
			ntnf_spi->trimid[0], ntnf_spi->trimid[1], ntnf_spi->trimid[2],
			ntnf_spi->trimid[3], ntnf_spi->trimid[4], ntnf_spi->trimid[5]);

	return 0;
}

const struct seq_operations ntnf_trimid_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_trimid_show
};

static int32_t nvt_trimid_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &ntnf_trimid_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_trimid_fops = {
	.owner = THIS_MODULE,
	.open = nvt_trimid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_trimid_fops = {
	.proc_open = nvt_trimid_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

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

const struct seq_operations ntnf_cmdtest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_cmdtest_show
};

static int32_t nvt_CmdTest_open(struct inode *inode, struct file *file)
{
//TestAction

	ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);

	ntnf_customizeCmd_WaitSet_Spi(EVENT_MAP_RESET_COMPLETE,RESET_STATE_INIT,0x13);		

/*
	ntnf_set_charger_bit(USB_PLUGOUT);
		msleep(200);			
		ntnf_read_charger_bit(); 		
		ntnf_getFWcmdStatus();
			
	ntnf_set_charger_bit(USB_PLUGIN);
		msleep(200);			
		ntnf_read_charger_bit();
		ntnf_getFWcmdStatus();

	ntnf_set_idle(2);//Err Parameter
		msleep(200);
		ntnf_getFWcmdStatus();		
	ntnf_set_idle(IDLE_SET_Enable);
		msleep(200);
		ntnf_getFWcmdStatus();		
	ntnf_set_idle(IDLE_SET_Disable);
		msleep(200);
		ntnf_getFWcmdStatus();

	ntnf_set_rotation(3);//Err Parameter
		msleep(1000);
		ntnf_getFWcmdStatus();				
	ntnf_set_rotation(EDGE_REJECT_VERTICAL);
		msleep(1000);
		ntnf_getFWcmdStatus();							
	ntnf_set_rotation(EDGE_REJECT_LEFT_Up);
		msleep(1000);
		ntnf_getFWcmdStatus();					
	ntnf_set_rotation(EDGE_REJECT_RIGHT_Up); 		
		msleep(1000);
		ntnf_getFWcmdStatus();			

	ntnf_fm_notify(3);//Err Parameter
		msleep(1000);	
		ntnf_getFWcmdStatus();					
	ntnf_fm_notify(1);
		msleep(1000);	
		ntnf_getFWcmdStatus();					
	ntnf_fm_notify(0);
		msleep(1000);	
		ntnf_getFWcmdStatus();			
	*/	
		
	//TestLog
	/*
	[ 1290.842977] [NVT-ts] ntnf_tpd_usb_plugin 709: usb plug [out] .
	[ 1290.847356] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x51
	[ 1290.902305] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1290.905660] [NVT-ts] ntnf_set_charger_bit 735: i32ret=[0]
	[ 1291.125669] [NVT-ts] ntnf_read_charger_bit 749: charger_bit=[49]
	[ 1291.134719] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x31,0x00,0x01,0x00]
	[ 1291.140763] [NVT-ts] ntnf_tpd_usb_plugin 719: usb plug [in ] .
	[ 1291.146044] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x53
	[ 1291.203428] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1291.206789] [NVT-ts] ntnf_set_charger_bit 735: i32ret=[0]
	[ 1291.425899] [NVT-ts] ntnf_read_charger_bit 749: charger_bit=[53]
	[ 1291.434781] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	
	[ 1291.442899] [NVT-ts] ntnf_set_idle 767: para=[2],i32ret=[-1]
	[ 1291.655676] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.661756] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x79
	[ 1291.723288] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1291.726650] [NVT-ts] ntnf_set_idle 767: para=[1],i32ret=[0]
	[ 1291.944774] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.950836] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x7A
	[ 1292.003344] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1292.006706] [NVT-ts] ntnf_set_idle 767: para=[0],i32ret=[0]
	[ 1292.226022] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1292.232083] [NVT-ts] ntnf_set_rotation 785: para=[3],i32ret=[-1]
	[ 1293.242293] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	
	[ 1293.248255] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x7C
	[ 1293.301057] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1293.304394] [NVT-ts] ntnf_set_rotation 785: para=[1],i32ret=[0]
	[ 1294.322212] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1294.328170] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x7D
	[ 1294.381043] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1294.384381] [NVT-ts] ntnf_set_rotation 785: para=[2],i32ret=[0]
	[ 1295.402157] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x45,0x00,0x01,0x00]
	[ 1295.408113] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x7E
	[ 1295.461033] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1295.464374] [NVT-ts] ntnf_set_rotation 785: para=[0],i32ret=[0]
	[ 1296.482340] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]
	
	[ 1296.488298] [NVT-ts] ntnf_fm_notify 817: ++
	[ 1296.492118] [NVT-ts] ntnf_fm_notify 826: Parameter Error:[3]
	[ 1296.497238] [NVT-ts] ntnf_fm_notify 829: --
	[ 1297.512574] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]
	[ 1297.518535] [NVT-ts] ntnf_fm_notify 817: ++
	[ 1297.522939] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x75
	[ 1297.571110] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1297.574447] [NVT-ts] ntnf_fm_notify 821: On
	[ 1297.578177] [NVT-ts] ntnf_fm_notify 829: --
	[ 1298.594103] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x03,0x00]
	[ 1298.600198] [NVT-ts] ntnf_fm_notify 817: ++
	[ 1298.603804] [NVT-ts] ntnf_customizeCmd 611: ++ Cmd=0x76
	[ 1298.662285] [NVT-ts] ntnf_customizeCmd 635: --
	[ 1298.665628] [NVT-ts] ntnf_fm_notify 824: Off
	[ 1298.669451] [NVT-ts] ntnf_fm_notify 829: --
	[ 1299.684187] [NVT-ts] ntnf_getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]

	*/
	

	return seq_open(file, &ntnf_cmdtest_seq_ops);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_CmdTest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_CmdTest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#else
static const struct proc_ops nvt_CmdTest_fops = {
	.proc_open = nvt_CmdTest_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static void free_xdata_buf(void)
{
	if (xdata_tmp != NULL) {
		kfree(xdata_tmp);
		xdata_tmp = NULL;
	}
	if (xdata != NULL) {
		kfree(xdata);
		xdata = NULL;
	}
	if (xdata_i != NULL) {
		kfree(xdata_i);
		xdata_i = NULL;
	}
	if (xdata_q != NULL) {
		kfree(xdata_q);
		xdata_q = NULL;
	}
}

static int malloc_xdata_buf(void)
{
	xdata_tmp = kzalloc(sizeof(uint8_t) * XDATE_BUF_SIZE, GFP_KERNEL);
	if (xdata_tmp == NULL) {
		NVT_ERR("Failed to allocate memory for xdata_tmp\n");
		goto error_enomem;
	}

	xdata = kzalloc(sizeof(int32_t) * XDATE_BUF_SIZE, GFP_KERNEL);
	if (xdata == NULL) {
		NVT_ERR("Failed to allocate memory for xdata\n");
		goto error_enomem;
	}

	xdata_i = kzalloc(sizeof(int32_t) * XDATE_BUF_SIZE, GFP_KERNEL);
	if (xdata_i == NULL) {
		NVT_ERR("Failed to allocate memory for xdata_i\n");
		goto error_enomem;
	}

	xdata_q = kzalloc(sizeof(int32_t) * XDATE_BUF_SIZE, GFP_KERNEL);
	if (xdata_q == NULL) {
		NVT_ERR("Failed to allocate memory for xdata_i\n");
		goto error_enomem;
	}

	return 0;

error_enomem:
	free_xdata_buf();
	return -ENOMEM;
}

int32_t ntnf_extra_proc_init(void)
{
	if (malloc_xdata_buf())
		goto errorcode1;

	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/nvt_fw_version Failed!\n");
		goto errorcode1;
	}
	NVT_LOG("create proc/nvt_fw_version Succeeded!\n");

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/nvt_baseline Failed!\n");
		goto errorcode2;
	}
	NVT_LOG("create proc/nvt_baseline Succeeded!\n");

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/nvt_raw Failed!\n");
		goto errorcode3;
	}
	NVT_LOG("create proc/nvt_raw Succeeded!\n");

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/nvt_diff Failed!\n");
		goto errorcode4;
	}
	NVT_LOG("create proc/nvt_diff Succeeded!\n");

	NVT_proc_fwupdate_entry = proc_create(NVT_UPDATE, 0444, NULL,&nvt_fwupdateV2_fops);	
	if (NVT_proc_fwupdate_entry == NULL) {
		NVT_ERR("create proc/nvt_update Failed!\n");
		goto errorcode5;
	}
	NVT_LOG("create proc/nvt_update Succeeded!\n");

	NVT_proc_trimid_entry = proc_create(NVT_TRIMID, 0444, NULL,&nvt_trimid_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/nvt_trimid Failed!\n");
		goto errorcode6;
	}
	NVT_LOG("create proc/nvt_trimid Succeeded!\n");
	
	NVT_proc_cmdtest_entry = proc_create(NVT_CMDTEST, 0444, NULL,&nvt_CmdTest_fops);
	if (NVT_proc_cmdtest_entry == NULL) {
		NVT_ERR("create proc/nvt_trimid Failed!\n");
		goto errorcode7;
	}
	NVT_LOG("create proc/nvt_trimid Succeeded!\n");
	return 0;

errorcode7:
	proc_remove(NVT_proc_trimid_entry);
errorcode6:
	proc_remove(NVT_proc_fwupdate_entry);
errorcode5:
	proc_remove(NVT_proc_diff_entry);
errorcode4:
	proc_remove(NVT_proc_raw_entry);
errorcode3:
	proc_remove(NVT_proc_baseline_entry);
errorcode2:
	proc_remove(NVT_proc_fw_version_entry);
errorcode1:
	return -ENOMEM;
}

void nvt_extra_proc_deinit_Spi_Spi(void)
{
	proc_remove(NVT_proc_cmdtest_entry);
	proc_remove(NVT_proc_trimid_entry);
	proc_remove(NVT_proc_fwupdate_entry);
	proc_remove(NVT_proc_diff_entry);
	proc_remove(NVT_proc_raw_entry);
	proc_remove(NVT_proc_baseline_entry);
	proc_remove(NVT_proc_fw_version_entry);
	free_xdata_buf();
}

#define FM_Enable                       0x75
#define FM_Disable                      0x76
static int nvt_fm_notify(int OnOff)
{
    int32_t ret = 0;

	return 0;
	
	if (ntnf_spi->fm_switch == 0 && 0 != OnOff) {
        ntnf_spi->fm_switch = 1;
        ret = ntnf_customizeCmd(FM_Enable);
    } else if (ntnf_spi->fm_switch != 0 && 0 == OnOff) {
        ntnf_spi->fm_switch = 0;
        ret = ntnf_customizeCmd(FM_Disable);
    }
    return ret;
}

int ntnf_FM_state_rewrite(void)
{
    int32_t ret = 0;
    if (ntnf_spi->fm_switch) {
        ret = ntnf_customizeCmd(FM_Enable);
    } else {
        ret = ntnf_customizeCmd(FM_Disable);
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

int ntnf_process_by_package(struct vts_device *vtsdev, unsigned char *package_name)
{
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return 0;
	}
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
int nvt_set_input_method(struct vts_device *vtsdev, int state)
{
	int32_t i32ret = 0;
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return 0;
	}
	if(state == 1)		
		i32ret = ntnf_customizeCmd(Typing_mode_Enter);
	else 	
		i32ret = ntnf_customizeCmd(Typing_mode_Leave);
	ntnf_getFWcmdStatus();
	VTI("input para=[%d],i32ret=[%d]", state, i32ret);
	return	i32ret;		
}
#else
int32_t ntnf_extra_proc_init(void)
{
	return 0;
}
void nvt_extra_proc_deinit_Spi_Spi(void)
{
	return ;
}
#endif
