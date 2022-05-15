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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#include <linux/vivo_ts_function.h>
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"


#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init_Spi_Spi(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init_Spi(void);
#endif

struct nvt_ts_data *ts_spi;

//static struct workqueue_struct *nvt_wq;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware_Spi(struct work_struct *work);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

uint32_t SWRST_N8_ADDR = 0; //read from dtsi

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array_Spi[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_POWER,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT

	KEY_POWER,  //GESTURE_WORD_AT	
	KEY_POWER,  //GESTURE_WORD_F		
};
#endif

#ifdef CONFIG_MTK_SPI
const struct mt_chip_conf spi_ctrdata = {
	.setuptime = 25,
	.holdtime = 25,
	.high_time = 5,	/* 10MHz (SPI_SPEED=100M / (high_time+low_time(10ns)))*/
	.low_time = 5,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif

static int32_t nvt_ts_suspend(struct device *dev);
static int32_t nvt_ts_resume(struct device *dev);


static uint8_t bTouchIsAwake = 0;

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};
	int retry = 0;

	VTD("wait spi bus wakeup");
	while(ts_spi->suspended) {
		msleep(10);
		if (++retry > 50) {
			VTI("after 500ms delay, device is stil in suspend mode!\n");
			return 0;
		}
	}

	switch (rw) {
		case NVTREAD:
			t.tx_buf = &buf[0];
			t.rx_buf = ts_spi->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = buf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	buf[0] = SPI_READ_MASK(buf[0]);	

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
		msleep(5);
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts_spi->rbuf+2), (len-1));
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts_spi->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts_spi->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts_spi->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen enable hw bld crc function.

return:
	N/A.
*******************************************************/
void nvt_bld_crc_enable(void)
{
	uint8_t buf[3] = {0};

	//---set xdata index to BLD_CRC_EN_ADDR---
	nvt_set_page(ts_spi->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts_spi->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts_spi->client, buf, 2);

	//---write data to index---
	buf[0] = ts_spi->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts_spi->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[2] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts_spi->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts_spi->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts_spi->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle_Spi(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
#define DISABLE_NT36XXX_SPI_FASTREAD 1 //[20180820,jx]To cover Qcom Platform latch-delay-1bit in mode-0!
#if (DISABLE_NT36XXX_SPI_FASTREAD)
	void nvt_disable_spiFastRead(void)
	{
		const uint32_t FAST_MODE_ADDR = 0x3F310;
		uint8_t setValue = 0x00;
		uint8_t buf[8] = {0};

		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(FAST_MODE_ADDR);

		//---setting---
		buf[0] = (FAST_MODE_ADDR & 0xFF);
		buf[1] = setValue;
		CTP_SPI_WRITE(ts_spi->client, buf, 2);
	}
#endif//(DISABLE_FASTMODE)

void nvt_bootloader_reset_Spi(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST
#if (DISABLE_NT36XXX_SPI_FASTREAD)
	nvt_disable_spiFastRead();
#endif//(DISABLE_FASTMODE)
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status_Spi(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts_spi->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts_spi->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status_Spi(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts_spi->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state_Spi(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts_spi->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 50)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		msleep(10);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid_Spi(void)
{
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts_spi->client, buf, 3);

	ts_spi->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts_spi->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info_Spi(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts_spi->client, buf, 17);
	ts_spi->fw_ver = buf[1];
	ts_spi->x_num = buf[3];
	ts_spi->y_num = buf[4];
	ts_spi->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts_spi->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts_spi->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts_spi->fw_ver = 0;
		ts_spi->x_num = 18;
		ts_spi->y_num = 32;
		ts_spi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts_spi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ts_spi->max_button_num = TOUCH_KEY_NUM;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts_spi->fw_ver, ts_spi->x_num, ts_spi->y_num,
					ts_spi->abs_x_max, ts_spi->abs_y_max, ts_spi->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	//---Get Novatek PID---
	nvt_read_pid_Spi();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTSPI"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	int32_t ret = 0;
	int32_t retries = 0;
	int8_t spi_wr = 0;
    uint8_t *buf = NULL;
	
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

	buf = (uint8_t *)kzalloc((count), GFP_KERNEL | GFP_DMA);
    if(buf == NULL) {
        NVT_ERR("kzalloc for buf failed!\n");
        ret = -ENOMEM;
        kfree(str);
        str = NULL;
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

	spi_wr = str[0] >> 7;
    memcpy(buf, str+2, ((str[0] & 0x7F) << 8) | str[1]);
	
	if (spi_wr == NVTWRITE) {	//SPI write
		while (retries < 20) {
			ret = CTP_SPI_WRITE(ts_spi->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else if (spi_wr == NVTREAD) {	//SPI read
		while (retries < 20) {
			ret = CTP_SPI_READ(ts_spi->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
			if (!ret)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

        // copy buff to user if spi transfer
        memcpy(str+2, buf, ((str[0] & 0x7F) << 8) | str[1]);
		if (retries < 20) {
			if (copy_to_user(buff, str, count)) {
				ret = -EFAULT;
				goto out;
			}
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			ret = -EIO;
			goto out;
		}
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		ret = -EFAULT;
		goto out;
	}

out:
	kfree(str);
	kfree(buf);	
kzalloc_failed:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close_Spi(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close_Spi,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTSPI initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL,&nvt_flash_fops);
	if (NVT_proc_entry == NULL) {
		NVT_ERR("Failed!\n");
		return -ENOMEM;
	} else {
		NVT_LOG("Succeeded!\n");
	}

	NVT_LOG("============================================================\n");
	NVT_LOG("Create /proc/NVTSPI\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

static struct wake_lock gestrue_wakelock;

//[20171025,jx]Correct for VIVO
#define GESTURE_EXT_WORD_AT		(30)	//'@'	forVIVO prj,F0-01-1E
#define GESTURE_EXT_WORD_F		(31)	//'F'	forVIVO prj,F0-01-1F
struct WakeUpTrace{
	uint8_t id;
	uint8_t OClockwise;
	uint16_t u16aX[9];
	uint16_t u16aY[9];		
}gsWakeUpTrace_Spi;
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report_Spi(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];
	uint8_t i;

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = KEY_C;
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = KEY_W;
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = KEY_WAKEUP;
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = KEY_M;
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = KEY_O;
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = KEY_E;
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = KEY_UP;
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = KEY_WAKEUP_SWIPE;
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = KEY_LEFT;
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = KEY_RIGHT;
			break;

#if(_CustomerFunction_)					
		case GESTURE_EXT_WORD_AT:
			NVT_LOG("Gesture : Custom_Word-AT.\n");
			keycode = KEY_A;					
			break;					

		case GESTURE_EXT_WORD_F:
			NVT_LOG("Gesture : Custom_Word-F.\n");
			keycode = KEY_F;								
			break;					
#endif

			
		default:
			VTI("not used gesture event");
			keycode = 0;
			break;
	}

	VTI("keycode= %d", keycode);
	if ((keycode > 0) && (keycode==KEY_F || keycode==KEY_A || keycode==KEY_UP || keycode==KEY_E || 
		keycode==KEY_O || keycode==KEY_C || keycode==KEY_W)) {
		vtsGesturePointsClean();
		gsWakeUpTrace_Spi.id=data[3];
		gsWakeUpTrace_Spi.OClockwise=data[43];
		for(i = 0; i < 9; i++) {
			gsWakeUpTrace_Spi.u16aX[i]=(data[4*i+5]<<8)+data[4*i+4];
			gsWakeUpTrace_Spi.u16aY[i]=(data[4*i+7]<<8)+data[4*i+6];
		}

		VTI("input_id=[%02d],OClockwise[%02X]"
				,gsWakeUpTrace_Spi.id, gsWakeUpTrace_Spi.OClockwise);
		for(i = 0; i < 9; i++) {
			VTI("%2d(%4d,%4d)",i ,gsWakeUpTrace_Spi.u16aX[i] ,gsWakeUpTrace_Spi.u16aY[i]);
			vtsGesturePointsReport(VTS_GESTURE_POINT, i, gsWakeUpTrace_Spi.u16aX[i], gsWakeUpTrace_Spi.u16aY[i]);
		}
		if (keycode == GESTURE_WORD_O) {
			vtsGesturePointsReport(VTS_GESTURE_O_DIR, 1, -1, -1);
		}
	}

	if (keycode > 0 ) {
		vivoTsInputReport(VTS_GESTURE_EVENT, keycode, -1, -1, -1);
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;	
	int32_t ret = 0;
	
	ts_spi->pinctrl_gpios = devm_pinctrl_get(dev);
	if (IS_ERR(ts_spi->pinctrl_gpios)) {
		ret = PTR_ERR(ts_spi->pinctrl_gpios);
		printk("can not find touch pintrl1");
		return ret;
	}
	
#if NVT_TOUCH_SUPPORT_HW_RST
	ts_spi->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts_spi->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts_spi->reset_gpio);
#endif	
	
	ts_spi->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts_spi->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts_spi->irq_gpio);

	ts_spi->spi_cs_set = pinctrl_lookup_state(ts_spi->pinctrl_gpios, "spi_cs_set");
	if (IS_ERR(ts_spi->spi_cs_set))	{
		ret = PTR_ERR(ts_spi->spi_cs_set);
		NVT_ERR("%s spi_cs_set pinctrl\n", __func__);		
	}
	pinctrl_select_state(ts_spi->pinctrl_gpios, ts_spi->spi_cs_set);

	ts_spi->pin_cs_pulllow = pinctrl_lookup_state(ts_spi->pinctrl_gpios,  "spi_cs_pulllow");
	if (IS_ERR(ts_spi->pin_cs_pulllow))	{
		ret = PTR_ERR(ts_spi->pin_cs_pulllow);
		NVT_ERR("%s pin_cs_pulllow pinctrl\n", __func__);		
	}
	
	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts_spi->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts_spi->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts_spi)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts_spi->reset_gpio)) {
		ret = gpio_request_one(ts_spi->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts_spi->irq_gpio)) {
		ret = gpio_request_one(ts_spi->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;


err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts_spi->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* enable/disable esd check flag */
	esd_check = enable;
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		mutex_lock(&ts_spi->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
		nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
		mutex_unlock(&ts_spi->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 10;
   uint8_t recovery_enable = false;
   uint8_t i = 0;

   recovery_cnt++;

   /* check pattern */
   for (i=1 ; i<7 ; i++) {
       if ((point_data[i] != 0xFD) && point_data[i] != 0xFE) {
           recovery_cnt = 0;
           break;
       }
   }

   if (recovery_cnt > recovery_cnt_max){
       recovery_enable = true;
       recovery_cnt = 0;
   }

   return recovery_enable;
}
#endif	/* #if NVT_TOUCH_WDT_RECOVERY */

#define POINT_DATA_LEN 65
#define FW_EVENT_LEN 5

#if POINT_DATA_CHECKSUM
/*******************************************************
Description:
	Novatek touchscreen check i2c packet checksum function.

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int32_t nvt_ts_point_data_checksum(uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;

	// Generate checksum
	for (i = 0; i < length-1; i++) {
		checksum += buf[i+1];
	}
	checksum = (~checksum + 1);

	// Compare ckecksum and dump fail data
	if (checksum != buf[length]) {	
		NVT_ERR("spi packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n", (length), buf[length], checksum);

		for (i = 0; i < 10; i++) {
			NVT_ERR("%02X %02X %02X %02X %02X %02X\n", buf[1+i*6], buf[2+i*6], buf[3+i*6], buf[4+i*6], buf[5+i*6], buf[6+i*6]);
		}

		for (i = 0; i < (length - 60); i++) {
			NVT_ERR("%02X ", buf[1+60+i]);
		}
		return -1;
	}

	return 0;
}
#endif /* POINT_DATA_CHECKSUM */

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static void nvt_ts_work_func(void *work)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + FW_EVENT_LEN + 2] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_z = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;

	if (atomic_read(&vivoTsGetVtsData()->tsState) == TOUCHSCREEN_GESTURE) {	//avoid spi slee cause gesture no function
		int retry = 0;
		VTD("wait spi bus wakeup");
		while(ts_spi->suspended) {
			msleep(10);
			if (++retry > 50) {
				VTI("after 500ms delay, device is stil in suspend mode!\n");
				enable_irq(ts_spi->client->irq);
				return ;
			}
		}
	}

	mutex_lock(&ts_spi->lock);

	ret = CTP_SPI_READ(ts_spi->client, point_data, POINT_DATA_LEN + FW_EVENT_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//--- dump SPI buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
*/

#if NVT_TOUCH_WDT_RECOVERY
   /* ESD protect by WDT */
   if (nvt_wdt_fw_recovery(point_data)) {
       NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
       nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
       goto XFER_ERROR;
   }
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

#if NVT_TOUCH_ESD_PROTECT
	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {

	#if POINT_DATA_CHECKSUM
		ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_LEN);
		if (ret < 0) {
			goto XFER_ERROR;
		}
	#endif /* POINT_DATA_CHECKSUM */
		
		input_id = (uint8_t)(point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report_Spi(input_id, point_data);
		enable_irq(ts_spi->client->irq);
		mutex_unlock(&ts_spi->lock);
		return;
	}
#endif

	//[20171023,jx]Catch Palm Event
	if ((point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x01)){
		VTI("got PalmOn");	
		//<VIVO can do action for Palm On>
		//large press chenpeng add
		vivoTsCoverMute(0, 1);
		enable_irq(ts_spi->client->irq);
		mutex_unlock(&ts_spi->lock);
		return; 	
	} else if ( (point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x02)){
		VTI("got PalmOFF");	
		enable_irq(ts_spi->client->irq);
		mutex_unlock(&ts_spi->lock);
		return; 	
	}
	/* get finger count */
	finger_cnt = 0;
	for (i = 0; i < ts_spi->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id > ts_spi->max_touch_num) || (input_id <= 0)) {
			continue;
		}
		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
			    continue;
			if ((input_x > ts_spi->abs_x_max) || (input_y > ts_spi->abs_y_max))
			    continue;
			finger_cnt++;
		}
	}
	ret = vivoTsCoverMute(finger_cnt, 0);
	if (ret < 0) {
	    if (-2 == ret) {
	        VTI("enter larger or 3 finger mode");
	        enable_irq(ts_spi->client->irq);
	        mutex_unlock(&ts_spi->lock);
	        return;
	    }
	    if (-3 == ret) {
	        VTI("lcd shutoff,not report points");
	        enable_irq(ts_spi->client->irq);
	        mutex_unlock(&ts_spi->lock);
	        return;
	    }
	}
	

	finger_cnt = 0;
	for (i = 0; i < ts_spi->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts_spi->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts_spi->abs_x_max) || (input_y > ts_spi->abs_y_max))
				continue;
			input_w = (uint32_t)(point_data[position + 4]);
			if (input_w == 0)
				input_w = 1;
			if (i < 2) {
				input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			} else {
				input_p = (uint32_t)(point_data[position + 5]);
			}
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;

			input_z = (input_x % 10 + input_y % 10 + 4) >> 1;
			input_z |= (point_data[66] << 8) | (point_data[67] << 16);
			
			vivoTsInputReport(VTS_TOUCH_DOWN, input_id - 1, input_x, input_y, input_z);
			//input_mt_slot(ts_spi->input_dev, input_id - 1);
			//input_mt_report_slot_state(ts_spi->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			//input_report_abs(ts_spi->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			//input_report_key(ts_spi->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			//input_report_abs(ts_spi->input_dev, ABS_MT_POSITION_X, input_x);
			//input_report_abs(ts_spi->input_dev, ABS_MT_POSITION_Y, input_y);
			//input_report_abs(ts_spi->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			//input_report_abs(ts_spi->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			//input_mt_sync(ts_spi->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts_spi->max_touch_num; i++) {
		if (press_id[i] != 1) {
			//input_mt_slot(ts_spi->input_dev, i);
			//input_report_abs(ts_spi->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			//input_report_abs(ts_spi->input_dev, ABS_MT_PRESSURE, 0);
			//input_mt_report_slot_state(ts_spi->input_dev, MT_TOOL_FINGER, false);
			vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
		}
	}

	if (finger_cnt == 0) {
		vivoTsReleasePoints();
	}

	//input_report_key(ts_spi->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		//input_report_key(ts_spi->input_dev, BTN_TOUCH, 0);
		//input_mt_sync(ts_spi->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts_spi->max_button_num; i++) {
			input_report_key(ts_spi->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts_spi->max_button_num; i++) {
			input_report_key(ts_spi->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	//input_sync(ts_spi->input_dev);

XFER_ERROR:
	enable_irq(ts_spi->client->irq);

	mutex_unlock(&ts_spi->lock);
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	disable_irq_nosync(ts_spi->client->irq);
	VTD("disable_irq_nosync");
	
#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {
		wake_lock_timeout(&gestrue_wakelock, msecs_to_jiffies(5000));
	}
#endif

	//queue_work(nvt_wq, &ts_spi->nvt_work);
	vtsIrqBtmThreadWake();
	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset_Spi();

		//---set xdata index to 0x1F600---
		nvt_set_page(0x1F600);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts_spi->client, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts_spi->trimid = trim_id_table[list].id;
				ts_spi->mmap = trim_id_table[list].mmap;
				ts_spi->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts_spi->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

static int last_touch_state = TOUCHSCREEN_NORMAL;

#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
#else
static int mdss_dsi_panel_reset_and_powerctl(int enable)
{
	return 0;
}
#endif

extern int8_t nvt_customizeCmd_WaitSet_Spi(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd);

/*0 is disable and 1 is enable*/
static void enable_or_disable_irq_wake(int enable)
{
	static int state_of_irq;
	if (state_of_irq == enable) {
		return;
	}
	if (enable) {
		enable_irq_wake(ts_spi->client->irq);
		VTD("enable_irq_wake");
	} else {
		disable_irq_wake(ts_spi->client->irq);
		VTD("disable_irq_wake");
	}
	state_of_irq = enable;
	return;
}

/**
 * zhj add for unbalanced irq when mode change  
 * bbk_slsi_irq_enable - Enable/Disable a irq
 * @ts: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
static int bbk_nt_irq_enable(struct nvt_ts_data *ts_spi, bool enable)
{
	VTI("****bbk_nt_irq_enable**** %d  atomic: %d", enable, atomic_read(&ts_spi->irq_enabled));
	if (enable) {
		if (!atomic_cmpxchg(&ts_spi->irq_enabled, 0, 1)) {
			enable_irq(ts_spi->client->irq);
			VTI("===mode_change=== Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&ts_spi->irq_enabled, 1, 0)) {
			disable_irq(ts_spi->client->irq);
			VTI("+++mode_change+++ Irq disabled");
		}
	}

	return 0;
}

/*
 * For 0-flash incell NT3667x, if display abnormal reset ic, can call it to restore the firmware to ic and set usb and other state
 * Input:  None
 * Output: 0 - success  other - fail
 */
int nvt_ts_abnormal_reset_download_fw(void)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();

	if (vtsData == NULL) {
		VTE("vtsData is not initialize, not to download firmware");
		ret = -1;
		return ret;
	}

	if (atomic_read(&vtsData->tsState) != TOUCHSCREEN_NORMAL) {
		VTE("Touch is not in normal state, not to download firmware, now the state is %d", atomic_read(&vtsData->tsState));
		ret = -1;
		return ret;
	}

	VTI("enter nvt_ts_abnormal_reset_download_fw");
	
	mutex_lock(&ts_spi->lock);
	
	bbk_nt_irq_enable(ts_spi, false);
	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
	ret = nvt_check_fw_reset_state_Spi(RESET_STATE_REK);
	if (ret) {
		VTE("nvt check fw reset fail");
	}
	bbk_nt_irq_enable(ts_spi, true);

	mutex_unlock(&ts_spi->lock);

	/* usb state write */
	ret = vivoTsSetChargerFlagToChip(vtsData->usbChargerFlag);
	if (-FUN_NOT_REG == ret) {
		VTI("vivoTsSetChargerFlagToChip no define.");
	} else if (ret < 0) {
		VTI("vivoTsSetChargerFlagToChip fail.");
	}
	/*while resume,is sure in edgeRestainSwitch on */
	vtsData->edgeRestainSwitch = 1;
	ret = vivoTsSetEdgeRestainToChip(vtsData->edgeRestainSwitch);
	if (-FUN_NOT_REG == ret) {
		VTI("vivoTsSetEdgeRestainToChip no define.");
	} else if (ret < 0) {
		VTI("vivoTsSetEdgeRestainToChip fail.");
	}

	vivoTsReleasePointsAndKeys();
	vtsData->isLargePressMode = 0;
	vtsData->is3FingerMode = 0;

	VTI("exit nvt_ts_abnormal_reset_download_fw");

	return ret;
}


static int32_t nvt_ts_suspend_to_lowpower(struct device *dev)
{
	uint8_t buf[2] = {0};

	mutex_lock(&ts_spi->lock);

	NVT_LOG("start\n");

	//---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_SPI_WRITE(ts_spi->client, buf, 2);

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("end\n");

	return 0;
}

static int bbk_xxx_mode_change(int which) {
	if (ts_spi == NULL) {
		VTI("ts is NULL");
		return 0;
	}

	#ifdef CONFIG_OF
	/* pinctrl_select_state(ts_spi->pinctrl_gpios, ts_spi->spi_cs_set); */ /* if VDDI can be shutdown in sleep mode, it should be open */
	#endif
	if(which == TOUCHSCREEN_NORMAL) {
		VTI("change to normal mode");
		nvt_ts_resume(&ts_spi->client->dev);
		enable_or_disable_irq_wake(0);
		bbk_nt_irq_enable(ts_spi, true);
	}
	if(which == TOUCHSCREEN_GESTURE) {
		VTI("change to gesture mode");
		if (last_touch_state == TOUCHSCREEN_SLEEP) {
			//power on first
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));			
			mdss_dsi_panel_reset_and_powerctl(1);
			//mdelay(50);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			
			//nvt_ts_resume(&ts_spi->client->dev);
			mutex_lock(&ts_spi->lock);
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
			nvt_check_fw_reset_state_Spi(RESET_STATE_INIT);
			mutex_unlock(&ts_spi->lock);

			//mdelay(20);
			nvt_customizeCmd_WaitSet_Spi(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT, 0x13);
			
		}
		enable_or_disable_irq_wake(1);
		bbk_nt_irq_enable(ts_spi, true);
	}
	if(which == TOUCHSCREEN_SLEEP) {
		VTI("change to sleep mode");

		if (last_touch_state != TOUCHSCREEN_SLEEP) {
			nvt_ts_suspend_to_lowpower(&ts_spi->client->dev);
		}
		
		//power off	
		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		#ifdef CONFIG_OF
		/* pinctrl_select_state(ts_spi->pinctrl_gpios, ts_spi->pin_cs_pulllow); */ /* if VDDI can be shutdown in sleep mode, it should be open */
		#endif
		mdss_dsi_panel_reset_and_powerctl(0);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		VTI("touchscreen is shutdown!!!");

		enable_or_disable_irq_wake(0);
		bbk_nt_irq_enable(ts_spi, false);
	}

	last_touch_state = which;

	return 0;
}
//extern int qup_i2c_suspended;
static int bbk_xxx_get_iic_bus_state(void) {
	return 0;
}
extern unsigned int mdss_report_lcm_id(void);
static unsigned int report_lcm_id(void)
{
	int lcm_id = 0;
	lcm_id = mdss_report_lcm_id();
	VTI("lcm_id get from mdss is %d", lcm_id);
	
	return lcm_id;
}
static int bbk_xxx_get_lcm_id(void)
{
	return report_lcm_id();
}

static int bbk_xxx_module_id(void)
{
	/*just for display in *#225# view*/
	return VTS_MVC_TMA; /* default boe*/
}

static int nt_early_suspend_run(void)
{
	VTI("early suspend run,nt set 13 cmd");
	nvt_ts_suspend(&ts_spi->client->dev);
	return 0;
}

extern int bbk_xxx_process_by_package(unsigned char *package_name);
extern int bbk_xxx_set_charger_bit(int state);
extern int bbk_xxx_read_charger_bit(void);
extern int idleEnableOrDisable(int state);
extern int setEdgeRestainSwitch(int i32Switch);
extern int bbk_xxx_get_chip_fw_version(int which);
extern int bbk_slsi_get_rawordiff_data(int which, int *data);
#include "firmware/PD1831F_no_flash_fw.h"
#include "firmware/PD1831F_no_flash_fw_mp.h"
#include "firmware/PD1831F_no_flash_36670_fw.h"
#include "firmware/PD1831F_no_flash_36670_fw_mp.h"
#include "firmware/PD1813B_no_flash_fw.h"
#include "firmware/PD1813B_no_flash_fw_mp.h"


/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
#if ((TOUCH_KEY_NUM > 0) || WAKEUP_GESTURE)
	int32_t retry = 0;
#endif
	struct vivo_ts_struct *vts_data = NULL;

	VTI("start");
	vts_data = vivoTsAlloc();
	if(vts_data == NULL) {
		VTI("vivoTsAlloc fail");
		return 0;
	}
	vts_data->busType = VTS_BUS_SPI;
	vts_data->spiClient = client;
#if defined(PD1813F_EX)
		vts_data->tsDimensionX = 1080;
		vts_data->tsDimensionY = 2280;
		vivoTsSetTxRxNum(18, 36);
#else
	vts_data->tsDimensionX = 1080;
	vts_data->tsDimensionY = 2340;
	vivoTsSetTxRxNum(16, 36);
#endif	
	vts_data->keyType = VTS_KEY_NOKEY;
	vts_data->isIndependentKeyIc = 1;
	vts_data->hasFingerPrint = 1;
	vts_data->hasHomeKey = 0;
	vts_data->hasMenuKey = 0;
	vts_data->hasBackKey = 0;
	vts_data->imeiWriteSupport = UNSUPPORT;

	vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
	vts_data->irqBtmThreadHandler = nvt_ts_work_func;

	vts_data->icNum = VTS_IC_NT36525;
	vts_data->mtkOrQcom = VTS_PLATFORM_MTK_NEW;

	vts_data->sensorTestKey = "com.nttouchscreen.mptest:MainActivity:android.intent.action.novatek:0:testResult";
	vts_data->lcmNoiseTestKey = "com.nt36xxxtouchscreen.deltadiff:MainActivity:null:null:null";
	/*vts_data->bspLcmNoiseTestKey = "com.nt36xxxtouchscreen.deltadiff:BspTest:android.intent.action.novatek:0:testResult";*/
	vts_data->rawdataTestKey = "com.nttouchscreen.getdata:MainActivity:null:null:null";

	vts_data->proFcFilterSwitch = 1;
	vts_data->notifyMutexSwitch = 1;
	
	vts_data->getTsPinValue = NULL;
	vts_data->sensorTest = NULL;
	vts_data->icModeChange = bbk_xxx_mode_change;
	vts_data->setChargerFlagSwitch = bbk_xxx_set_charger_bit;
	vts_data->idleEnableOrDisable = idleEnableOrDisable;
	vts_data->setEdgeRestainSwitch = setEdgeRestainSwitch;
	vts_data->getIcFirmwareOrConfigVersion = bbk_xxx_get_chip_fw_version;
	vts_data->getLcmId = bbk_xxx_get_lcm_id;
	vts_data->getI2cBusState = bbk_xxx_get_iic_bus_state;
	vts_data->readImei = NULL;	//0-flash
	vts_data->writeImei = NULL;	//0-flash
	vts_data->getModuleId = bbk_xxx_module_id;
	vts_data->earlySuspendRun = nt_early_suspend_run;
	vts_data->processByPackage = bbk_xxx_process_by_package;
	vts_data->getRawOrDiffData = bbk_slsi_get_rawordiff_data;
	//vts_data->callbackType = 1;


	//vts_data->lcmNoiseTestKey = "com.st.frameLoggerNew:ProductionTest:null:null:null";
	//vts_data->sensorTestKey = "";
/*	
 	vts_data->updateFirmware = ;
	
	vts_data->getRawOrDiffData = bbk_slsi_get_rawordiff_data;
	vts_data->getModuleId = bbk_sec_module_id;
	vts_data->getGesturePointData = bbk_slsi_gesture_point_get;
		
	vts_data->atSensorTest = samsung_at_sensor_test;
	
	vts_data->otherInfo = bbk_slsi_get_fw_debug_info;
	vts_data->earlySuspendRun = bbk_slsi_early_suspend_run;
*/
	ret = vivoTsInit((void *)client, NULL, -1);
	if(ret < 0) {
		vivoTsFree();
		return -1;
	}
	
	vts_data->resumeEventBlank = FB_EVENT_BLANK;	//must set after vivoTsInit
	vts_data->suspendEventBlank = FB_EVENT_BLANK;
	
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1831F_no_flash_fw, sizeof(PD1831F_no_flash_fw), "PD1831F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TMA, 0x10);
	vivoTsFwAdd(VTS_FW_TYPE_MP, PD1831F_no_flash_fw_mp, sizeof(PD1831F_no_flash_fw_mp), "PD1831F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TMA, 0x10);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1831F_no_flash_36670_fw, sizeof(PD1831F_no_flash_36670_fw), "PD1831F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TMA, 0x11);
	vivoTsFwAdd(VTS_FW_TYPE_MP, PD1831F_no_flash_36670_fw_mp, sizeof(PD1831F_no_flash_36670_fw_mp), "PD1831F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TMA, 0x11);
	vivoTsFwAdd(VTS_FW_TYPE_FW, PD1813B_no_flash_fw, sizeof(PD1813B_no_flash_fw), "PD1813F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_BOE, 0x12);
	vivoTsFwAdd(VTS_FW_TYPE_MP, PD1813B_no_flash_fw_mp, sizeof(PD1813B_no_flash_fw_mp), "PD1813F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_BOE, 0x12);
	
	ts_spi = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts_spi == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts_spi->client = client;
	ts_spi->suspended = false;
	spi_set_drvdata(client, ts_spi);

	//---prepare for spi parameter---
	if (ts_spi->client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts_spi->client->bits_per_word = 8;
	ts_spi->client->mode = SPI_MODE_0;

	ret = spi_setup(ts_spi->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

#ifdef CONFIG_MTK_SPI
	memcpy(&ts_spi->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
	ts_spi->client->controller_data = (void *)&ts_spi->spi_ctrl;
#endif

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts_spi->client->mode, ts_spi->client->max_speed_hz);

	//---parse dts_spi---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts_spi);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}    

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts_spi->lock);

	ts_spi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts_spi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

	//---create workqueue---
	/*nvt_wq = create_workqueue("nvt_wq");
	if (!nvt_wq) {
		NVT_ERR("nvt_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_wq_failed;
	}
	INIT_WORK(&ts_spi->nvt_work, nvt_ts_work_func);*/


	//---allocate input device---
	ts_spi->input_dev = input_allocate_device();
	if (ts_spi->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts_spi->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts_spi->max_button_num = TOUCH_KEY_NUM;
#endif

	ts_spi->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts_spi->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts_spi->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts_spi->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
	input_mt_init_slots(ts_spi->input_dev, ts_spi->max_touch_num, 0);
#endif

	input_set_abs_params(ts_spi->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts_spi->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts_spi->input_dev, ABS_MT_POSITION_X, 0, ts_spi->abs_x_max, 0, 0);
	input_set_abs_params(ts_spi->input_dev, ABS_MT_POSITION_Y, 0, ts_spi->abs_y_max, 0, 0);
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts_spi->input_dev, ABS_MT_TRACKING_ID, 0, ts_spi->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts_spi->max_button_num; retry++) {
		input_set_capability(ts_spi->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

#if WAKEUP_GESTURE
	for (retry = 0; retry < (sizeof(gesture_key_array_Spi) / sizeof(gesture_key_array_Spi[0])); retry++) {
		input_set_capability(ts_spi->input_dev, EV_KEY, gesture_key_array_Spi[retry]);
	}
	wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	sprintf(ts_spi->phys, "input/ts");
	ts_spi->input_dev->name = NVT_TS_NAME;
	ts_spi->input_dev->phys = ts_spi->phys;
	ts_spi->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts_spi->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts_spi->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts_spi->irq_gpio);
	if (client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts_spi->int_trigger_type);
#if (0)
#if WAKEUP_GESTURE
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				ts_spi->int_trigger_type | IRQF_NO_SUSPEND, NVT_SPI_NAME, ts_spi);
#else
		ret = request_irq(client->irq, nvt_ts_irq_handler,
				ts_spi->int_trigger_type, NVT_SPI_NAME, ts_spi);
#endif
#endif
		ret = vivoTsInterruptRegister(client->irq, NULL, nvt_ts_irq_handler, ts_spi->int_trigger_type/* | IRQF_NO_SUSPEND |IRQF_ONESHOT*/, ts_spi);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			disable_irq(client->irq);
			atomic_set(&ts_spi->irq_enabled, 1);
			NVT_LOG("request irq %d succeed\n", client->irq);
		}
	}

#if BOOT_UPDATE_FIRMWARE
if (0) {
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}

	INIT_DELAYED_WORK(&ts_spi->nvt_fwu_work, Boot_Update_Firmware_Spi);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts_spi->nvt_fwu_work, msecs_to_jiffies(14000));
} else {
	Boot_Update_Firmware_Spi(NULL);
}
#endif

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init_Spi_Spi();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init_Spi();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if defined(CONFIG_FB)
if (0) {
	ts_spi->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts_spi->fb_notif);
	if(ret) {
		NVT_ERR("register fb_notifier failed. ret=%d\n", ret);
		goto err_register_fb_notif_failed;
	}
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts_spi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts_spi->early_suspend.suspend = nvt_ts_early_suspend;
	ts_spi->early_suspend.resume = nvt_ts_late_resume;
	ret = register_early_suspend(&ts_spi->early_suspend);
	if(ret) {
		NVT_ERR("register early suspend failed. ret=%d\n", ret);
		goto err_register_early_suspend_failed;
	}
#endif

	bTouchIsAwake = 1;
	NVT_LOG("end\n");

	enable_irq(client->irq);

	vivoTsAfterProbeCompleteCall(NULL, NULL, -1);

	return 0;

#if defined(CONFIG_FB)
err_register_fb_notif_failed:
#elif defined(CONFIG_HAS_EARLYSUSPEND)
err_register_early_suspend_failed:
#endif
#if (NVT_TOUCH_PROC || NVT_TOUCH_EXT_PROC || NVT_TOUCH_MP)
err_init_NVT_ts:
#endif
	//free_irq(client->irq, ts_spi);
	vivoTsInterruptUnregister();
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
err_input_register_device_failed:
	input_free_device(ts_spi->input_dev);
err_input_dev_alloc_failed:
//err_create_nvt_wq_failed:
	mutex_destroy(&ts_spi->lock);
err_chipvertrim_failed:
	gpio_free(ts_spi->irq_gpio);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	kfree(ts_spi);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct spi_device *client)
{
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts_spi->fb_notif))
		NVT_ERR("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts_spi->early_suspend);
#endif

	mutex_destroy(&ts_spi->lock);

	NVT_LOG("Removing driver...\n");

	//free_irq(client->irq, ts_spi);
	vivoTsInterruptUnregister();
	input_unregister_device(ts_spi->input_dev);
	spi_set_drvdata(client, NULL);
	kfree(ts_spi);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
#if MT_PROTOCOL_B
	//uint32_t i = 0;
#endif
	
	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	mutex_lock(&ts_spi->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE
	//---write spi command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	CTP_SPI_WRITE(ts_spi->client, buf, 2);

	/* enable_irq_wake(ts_spi->client->irq); */

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	disable_irq(ts_spi->client->irq);

	//---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_SPI_WRITE(ts_spi->client, buf, 2);
#endif // WAKEUP_GESTURE

	/* release all touches */
#if MT_PROTOCOL_B
	//for (i = 0; i < ts_spi->max_touch_num; i++) {
		//input_mt_slot(ts_spi->input_dev, i);
		//input_report_abs(ts_spi->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(ts_spi->input_dev, ABS_MT_PRESSURE, 0);
		//input_mt_report_slot_state(ts_spi->input_dev, MT_TOOL_FINGER, 0);
	//}
#endif
	//input_report_key(ts_spi->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	//input_mt_sync(ts_spi->input_dev);
#endif
	//input_sync(ts_spi->input_dev);

	msleep(50);

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}

	mutex_lock(&ts_spi->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts_spi->reset_gpio, 1);
#endif
	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
	nvt_check_fw_reset_state_Spi(RESET_STATE_REK);

#if !WAKEUP_GESTURE
	enable_irq(ts_spi->client->irq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	mutex_unlock(&ts_spi->lock);

	NVT_LOG("end\n");

	return 0;
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nvt_ts_data *ts_spi =
		container_of(self, struct nvt_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			nvt_ts_suspend(&ts_spi->client->dev);
		}
	} else if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			nvt_ts_resume(&ts_spi->client->dev);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Description:
	Novatek touchscreen driver early suspend function.

return:
	n.a.
*******************************************************/
static void nvt_ts_early_suspend(struct early_suspend *h)
{
	nvt_ts_suspend(ts_spi->client, PMSG_SUSPEND);
}

/*******************************************************
Description:
	Novatek touchscreen driver late resume function.

return:
	n.a.
*******************************************************/
static void nvt_ts_late_resume(struct early_suspend *h)
{
	nvt_ts_resume(ts_spi->client);
}
#endif

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "vivo,ts-spi_v1",},
	{ },
};
#endif

static int nvt_ts_dev_suspend(struct device *dev)
{
	struct nvt_ts_data *ts_spi = dev_get_drvdata(dev);
	ts_spi->suspended = true;
	return 0;
}

static int nvt_ts_dev_resume(struct device *dev)
{
	struct nvt_ts_data *ts_spi = dev_get_drvdata(dev);
	ts_spi->suspended = false;
	return 0;
}

static const struct dev_pm_ops nvt_ts_pm = {
	.suspend = nvt_ts_dev_suspend,
	.resume = nvt_ts_dev_resume,
};

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
		.pm = &nvt_ts_pm,
	},
};

static void vivo_ts_nt_spi_driver_init(void)
{
	int32_t ret = 0;
	ret = spi_register_driver(&nvt_spi_driver);
	if (ret) {
		VTI("failed to add spi driver");
		return ;
	}
	VTI("spi register success");
	return ;
}

//extern unsigned int power_off_charging_mode;
#if 0
extern unsigned int is_atboot;
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);

extern unsigned int report_lcm_id(void);
#endif
/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;
	//enum boot_mode_t boot_mode;
	const char *productName = NULL;

	VTI("start\n");

	/*if (is_atboot == 1 || power_off_charging_mode == 1) {
		VTI("TS is in at mood of power off charging mode!");
		return 0;
	}*/
	
	ret = report_lcm_id();
	VTI("0-flash module lcm id is 0x%x", ret);
	if (ret != 0x10 && ret != 0x11 && ret != 0x12) {
		VTI("not NT spi module");
		return 0;	
	}

	vivo_touchscreen_get_product_name(&productName);	

	if (productName == NULL) {
		VTE("The product name is null");
	} else {
		VTI("The productName is:%s", productName);
		if (strcmp(productName, "PD1831F_EX")) {
			VTI("The product name is not PD1831F_EX");
			return 0;
		}
	}

	#if 0
	boot_mode = get_boot_mode();

	VTI("mode=%d", boot_mode);
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT  ||
		boot_mode == LOW_POWER_OFF_CHARGING_BOOT ||
		boot_mode == META_BOOT ||
		boot_mode == FACTORY_BOOT ||
		boot_mode == ADVMETA_BOOT) {
		VTI("in (%d) mode,we do not load driver", boot_mode);
		boot_mode_normal = false;
	} else {
		boot_mode_normal = true;
	}


	if (is_atboot == 1 || !boot_mode_normal) {
		VTI("TS is in at mode or power off charging mode.");
		return 0;
	}	
	#endif
	ret = vivo_touchscreen_new_module_init(vivo_ts_nt_spi_driver_init, "vivo_ts_nt_spi_driver_init");

	VTI("finished...");

	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	spi_unregister_driver(&nvt_spi_driver);

	/*if (nvt_wq)
		destroy_workqueue(nvt_wq);*/

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
