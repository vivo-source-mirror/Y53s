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
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "nt36xxx.h"
#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_ESD_PROTECT
struct delayed_work nvt_esd_check_work_v2;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check_v2 = false;
u32 esd_check_support = 0;
uint8_t esd_retry_v2 = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

extern int32_t ntnf_extra_proc_init(void);
extern void nvt_extra_proc_deinit_Spi_Spi(void);
extern int32_t ntnf_mp_proc_init(void);
extern int32_t nvt_mp_proc_deinit_Spi(void);
extern int ntnf_process_by_package(struct vts_device *vtsdev, unsigned char *package_name);
extern int ntnf_set_charger_bit(struct vts_device *vtsdev, int state);
extern int ntnf_set_idle(struct vts_device *vtsdev, int state);
extern int ntnf_set_rotation(struct vts_device *vtsdev, int i32Switch);
extern int ntnf_get_chip_fw_version(struct vts_device *vtsdev, u64 *version);
extern int ntnf_get_touch_ic_mode(struct vts_device *vtsdev);
extern int ntnf_get_frame(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size);
extern void nvt_update_gesture_firmware_V2(char * firmware_name,int fwtype,int fwRequest);
extern int nvt_set_input_method(struct vts_device *vtsdev,int state);
struct nvt_ts_data *ntnf_spi;

//static struct workqueue_struct *nvt_wq;

#if BOOT_UPDATE_FIRMWARE
extern void Ntnf_Boot_Update_Firmware(struct work_struct *work);
#endif

uint32_t NFNF_SWRST_N8_ADDR = 0; //read from dtsi
uint32_t ENG_RST_ADDR_V2  = 0x7FFF80;

#ifdef CONFIG_SPI_MT65XX
const struct mtk_chip_config spi_ctrdata = {
	.rx_mlsb = 1,
    .tx_mlsb = 1,
    .cs_pol = 0,
    .sample_sel = 0,
	.cs_setuptime = 25,
	.cs_holdtime = 12,
	.cs_idletime = 12,
};
#elif defined CONFIG_MTK_SPI
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

#ifdef CONFIG_ARCH_BENGAL

const struct spi_geni_qcom_ctrl_data qcom_4125_ctrl_data = {
  	.spi_cs_clk_delay = 3,   
  	//.spi_inter_words_delay = 0,
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

	memcpy(ntnf_spi->xbuf, buf, len);
	switch (rw) {
		case NVTREAD:
			//t.tx_buf = &buf[0];
			t.tx_buf = &ntnf_spi->xbuf[0];
			t.rx_buf = ntnf_spi->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ntnf_spi->xbuf;
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
int32_t NTNF_CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;
	
	mutex_lock(&ntnf_spi->xbuf_lock);

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
		memcpy((buf+1), (ntnf_spi->rbuf+2), (len-1));
	}
	mutex_unlock(&ntnf_spi->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t NTNF_CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ntnf_spi->xbuf_lock);

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
	mutex_unlock(&ntnf_spi->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t ntnf_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t ntnf_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
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
void ntnf_bld_crc_enable(void)
{
	uint8_t buf[3] = {0};

	//---set xdata index to BLD_CRC_EN_ADDR---
	ntnf_set_page(ntnf_spi->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ntnf_spi->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

	//---write data to index---
	buf[0] = ntnf_spi->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void ntnf_fw_crc_enable(void)
{
	uint8_t buf[2] = {0};

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void ntnf_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	ntnf_write_addr(ntnf_spi->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void ntnf_sw_reset(void)
{
	//---software reset cmds to NFNF_SWRST_N8_ADDR---
	ntnf_write_addr(NFNF_SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void ntnf_sw_reset_idle(void)
{
	//---MCU idle cmds to NFNF_SWRST_N8_ADDR---
	ntnf_write_addr(NFNF_SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset_v2(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	ntnf_write_addr(ENG_RST_ADDR_V2, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}


/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
#define DISABLE_NT36XXX_SPI_FASTREAD 1 //[20180820,jx]To cover Qcom Platform latch-delay-1bit in mode-0!
#if (DISABLE_NT36XXX_SPI_FASTREAD)
	void ntnf_disable_spiFastRead(void)
	{
		const uint32_t FAST_MODE_ADDR = 0x3F310;
		uint8_t setValue = 0x00;
		uint8_t buf[8] = {0};

		//---set xdata index to EVENT BUF ADDR---
		ntnf_set_page(FAST_MODE_ADDR);

		//---setting---
		buf[0] = (FAST_MODE_ADDR & 0xFF);
		buf[1] = setValue;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
	}
#endif//(DISABLE_FASTMODE)

void ntnf_bootloader_reset(void)
{
	//---reset cmds to NFNF_SWRST_N8_ADDR---
	ntnf_write_addr(NFNF_SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST
#if (DISABLE_NT36XXX_SPI_FASTREAD)
	ntnf_disable_spiFastRead();
#endif//(DISABLE_FASTMODE)
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t ntnf_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

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
int32_t ntnf_check_fw_status(void)
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
int32_t ntnf_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
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
int32_t ntnf_read_pid(void)
{
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	ret = ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 3);

	ntnf_spi->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	ret = ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ntnf_spi->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t ntnf_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}

info_retry:	
	//---set xdata index to EVENT BUF ADDR---
	ntnf_set_page(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 17);
	ntnf_spi->fw_ver = buf[1];
	ntnf_spi->x_num = buf[3];
	ntnf_spi->y_num = buf[4];
	ntnf_spi->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ntnf_spi->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ntnf_spi->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ntnf_spi->fw_ver = 0;
		ntnf_spi->x_num = 18;
		ntnf_spi->y_num = 32;
		ntnf_spi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ntnf_spi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
		ntnf_spi->max_button_num = 0;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ntnf_spi->fw_ver, ntnf_spi->x_num, ntnf_spi->y_num,
					ntnf_spi->abs_x_max, ntnf_spi->abs_y_max, ntnf_spi->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}
	NVT_LOG("fw_ver = 0x%02X, fw_type = 0x%02X\n", ntnf_spi->fw_ver, buf[14]); 

	//---Get Novatek PID---
	ntnf_read_pid();

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
	if (esd_check_support) {
		cancel_delayed_work_sync(&nvt_esd_check_work_v2);
		nvt_esd_check_enable_v2(false);
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	spi_wr = str[0] >> 7;
    memcpy(buf, str+2, ((str[0] & 0x7F) << 8) | str[1]);
	
	if (spi_wr == NVTWRITE) {	//SPI write
		while (retries < 20) {
			ret = NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
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
			ret = NTNF_CTP_SPI_READ(ntnf_spi->client, buf, ((str[0] & 0x7F) << 8) | str[1]);
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

#if (NVTFLASH_WORK_PROTECT)
	atomic_t u8_NT36xxx_flashWorking = ATOMIC_INIT(0);
#endif
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

	#if (NVTFLASH_WORK_PROTECT)
		atomic_set(&u8_NT36xxx_flashWorking, 1);
		NVT_ERR("u8_NT36xxx_flashWorking=[%d]\n", atomic_read(&u8_NT36xxx_flashWorking));
	#endif

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

	#if (NVTFLASH_WORK_PROTECT)
		atomic_set(&u8_NT36xxx_flashWorking, 0);
		NVT_ERR("u8_NT36xxx_flashWorking=[%d]\n", atomic_read(&u8_NT36xxx_flashWorking));
	#endif

	if (dev)
		kfree(dev);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close_Spi,
	.read = nvt_flash_read,
};
#else
static const struct proc_ops nvt_flash_fops = {
	.proc_open = nvt_flash_open,
	.proc_release = nvt_flash_close_Spi,
	.proc_read = nvt_flash_read,
};
#endif	

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

static void nvt_flash_proc_deinit(void)
{
	proc_remove(NVT_proc_entry);
}
#else
static int32_t nvt_flash_proc_init(void)
{
	return 0;
}

static void nvt_flash_proc_deinit(void)
{
	return ;
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

//[20171025,jx]Correct for VIVO
#define GESTURE_EXT_WORD_AT		(30)	//'@'	forVIVO prj,F0-01-1E
#define GESTURE_EXT_WORD_F		(31)	//'F'	forVIVO prj,F0-01-1F
struct WakeUpTrace{
	uint8_t id;
	uint8_t OClockwise;
	uint16_t u16aX[9];
	uint16_t u16aY[9];		
};
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void ntnf_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];
	uint8_t i;
	struct WakeUpTrace gsWakeUpTrace_Spi;

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
			keycode = VTS_EVENT_GESTURE_PATTERN_C;
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_W;
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = VTS_EVENT_GESTURE_DOUBLE_CLICK;
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_M;
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_O;
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_E;
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_UP;
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_DOWN;
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_LEFT;
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_RIGHT;
			break;

#if(_CustomerFunction_)					
		case GESTURE_EXT_WORD_AT:
			NVT_LOG("Gesture : Custom_Word-AT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_A;					
			break;					

		case GESTURE_EXT_WORD_F:
			NVT_LOG("Gesture : Custom_Word-F.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_F;								
			break;					
#endif

			
		default:
			VTI("not used gesture event");
			keycode = 0;
			break;
	}

	VTI("keycode= %d", keycode);
	if ((keycode > 0) && (keycode==VTS_EVENT_GESTURE_PATTERN_F || keycode==VTS_EVENT_GESTURE_PATTERN_A || keycode==VTS_EVENT_GESTURE_PATTERN_UP || keycode==VTS_EVENT_GESTURE_PATTERN_E || 
		keycode==VTS_EVENT_GESTURE_PATTERN_O || keycode==VTS_EVENT_GESTURE_PATTERN_C || keycode==VTS_EVENT_GESTURE_PATTERN_W)) {
		gsWakeUpTrace_Spi.id=data[3];
		gsWakeUpTrace_Spi.OClockwise=data[43];
		for(i = 0; i < 9; i++) {
			gsWakeUpTrace_Spi.u16aX[i]=(data[4*i+5]<<8)+data[4*i+4];
			gsWakeUpTrace_Spi.u16aY[i]=(data[4*i+7]<<8)+data[4*i+6];
		}
                          
		VTI("input_id=[%02d],OClockwise[%02X]"
				,gsWakeUpTrace_Spi.id, gsWakeUpTrace_Spi.OClockwise);
		vts_report_coordinates_set(ntnf_spi->vtsdev, gsWakeUpTrace_Spi.u16aX, gsWakeUpTrace_Spi.u16aY, 9);
	}

	if (keycode > 0 ) {
		vts_report_event_down(ntnf_spi->vtsdev, keycode);
		vts_report_event_up(ntnf_spi->vtsdev, keycode);
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
static int32_t nvt_parse_dt(struct device_node *np)
{	
	int32_t ret = 0;

	ntnf_spi->pinctrl = devm_pinctrl_get(ntnf_spi->dev);
	if (IS_ERR_OR_NULL(ntnf_spi->pinctrl)) {
		NVT_LOG("no defined pinctrl");
	} else {
		VTI("find pinctrl");
		ntnf_spi->pinctrl_default = pinctrl_lookup_state(ntnf_spi->pinctrl, "device_default");
		if (IS_ERR_OR_NULL(ntnf_spi->pinctrl_default)) {
			NVT_LOG("default pinctrl IS_ERR_OR_NULL");
		}

		ntnf_spi->spi_cs_active = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_cs_set");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_cs_active)) {
			NVT_LOG("select spi cs pinctrl failed");
		}
		
		ntnf_spi->spi_cs_sleep_pulllow = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_cs_pulllow");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_cs_sleep_pulllow)) {
			NVT_LOG("select spi cs gpio pinctrl failed");
		}

		ntnf_spi->spi_miso_active = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_miso_set");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_miso_active)) {
			NVT_LOG("select spi miso pinctrl failed");
		}
		
		ntnf_spi->spi_miso_sleep_pulllow = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_miso_pulllow");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_miso_sleep_pulllow)) {
			NVT_LOG("select spi miso gpio pinctrl failed");
		}
		ntnf_spi->spi_mosi_active = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_mosi_active");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_mosi_active)) {
			NVT_LOG("select spi mosi gpio pinctrl failed");
		}
		ntnf_spi->spi_clk_active = pinctrl_lookup_state(ntnf_spi->pinctrl, "spi_clk_active");
		if (IS_ERR_OR_NULL(ntnf_spi->spi_clk_active)) {
			NVT_LOG("select spi clk gpio pinctrl failed");
		}
	}
	
#if NVT_TOUCH_SUPPORT_HW_RST
	ntnf_spi->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ntnf_spi->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ntnf_spi->reset_gpio);
#endif	
	
	ntnf_spi->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ntnf_spi->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ntnf_spi->irq_gpio);

	ntnf_spi->cs_gpio = of_get_named_gpio_flags(np, "novatek,cs-gpio", 0, &ntnf_spi->cs_flags);
	NVT_LOG("novatek,cs-gpio=%d\n", ntnf_spi->cs_gpio);
	
	ntnf_spi->miso_gpio = of_get_named_gpio_flags(np, "novatek,miso-gpio", 0, &ntnf_spi->miso_flags);

	NVT_LOG("novatek,miso-gpio=%d\n", ntnf_spi->miso_gpio);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &NFNF_SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("NFNF_SWRST_N8_ADDR=0x%06X\n", NFNF_SWRST_N8_ADDR);
	}

	ret = of_property_read_u32(np, "spi-max-frequency", &ntnf_spi->spi_frequency);
	if (ret || ntnf_spi->spi_frequency <= 0 || ntnf_spi->spi_frequency > 9600000) {
		ntnf_spi->spi_frequency = 6000000;
		NVT_ERR("error reading spi frequency, use default frequency");
		ret = 0;
	} 
	NVT_LOG("spi frequency:%d\n", ntnf_spi->spi_frequency);

#if defined(CONFIG_MEDIATEK_SOLUTION)
	ntnf_spi->vddi_poweroff = of_property_read_bool(np, "mtk,vts-vddi-poweroff");
	if (ntnf_spi->vddi_poweroff) {
		NVT_LOG("vddi is poweroff in sleep mode");
	} else {
		NVT_LOG("vddi no poweroff in sleep mode");
	}

	ntnf_spi->cs_bootup = of_property_read_bool(np, "mtk,vts-cs-bootup");
	if (ntnf_spi->cs_bootup) {
		NVT_LOG("cs is pull-down in boot loader");
	} else {
		NVT_LOG("cs no pull-down in boot loader");
	}
#endif
	ntnf_spi->nt_4power = of_property_read_bool(np, "novatek,4power");
	if (ntnf_spi->nt_4power) {
		NVT_LOG("4 power");
	} else {
		NVT_LOG("no 4 power");
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ntnf_spi)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ntnf_spi->reset_gpio)) {
		ret = gpio_request_one(ntnf_spi->reset_gpio, GPIOF_OUT_INIT_HIGH, "NVT-tp-rst");
		if (ret) {
			ntnf_spi->reset_gpio = -1;
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ntnf_spi->irq_gpio)) {
		ret = gpio_request_one(ntnf_spi->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}
#if defined(CONFIG_ARCH_QCOM)
	if (gpio_is_valid(ntnf_spi->cs_gpio)) {
		ret = gpio_request_one(ntnf_spi->cs_gpio, GPIOF_OUT_INIT_HIGH, "NVT-spi-cs");
		if (ret) {
			NVT_ERR("Failed to request NVT-spi-cs GPIO\n");
			goto err_request_cs_gpio;
		}
	}
	if (gpio_is_valid(ntnf_spi->miso_gpio)) {
		ret = gpio_request_one(ntnf_spi->miso_gpio, GPIOF_INIT_HIGH, "NVT-spi-miso");
		if (ret) {
			NVT_ERR("Failed to request NVT-spi-miso GPIO\n");
			goto err_request_miso_gpio;
		}
	}
#endif

#if defined(CONFIG_MEDIATEK_SOLUTION)
	if (!IS_ERR_OR_NULL(ntnf_spi->spi_clk_active)) {
		VTI("select clk default state");
		pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_clk_active);
	}

	if (!IS_ERR_OR_NULL(ntnf_spi->spi_mosi_active)) {
		VTI("select mosi default state");
		pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_mosi_active);
	}
#endif
	if (!IS_ERR_OR_NULL(ntnf_spi->pinctrl_default)) {
		VTI("select pinctrl default state");
		pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->pinctrl_default);
	}

	return ret;
#if defined(CONFIG_ARCH_QCOM)
err_request_miso_gpio:
	gpio_free(ntnf_spi->miso_gpio);
err_request_cs_gpio:
	gpio_free(ntnf_spi->irq_gpio);
#endif
err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	if(gpio_is_valid(ntnf_spi->reset_gpio))
	   gpio_free(ntnf_spi->reset_gpio);
#endif
	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable_v2(uint8_t enable)
{
	/* enable/disable esd check flag */
	esd_check_v2 = enable;
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry_v2 = enable ? 0 : esd_retry_v2;
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
	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check_v2 && (esd_retry_v2 < 3)) {
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry_v2);
		/* do esd recovery, reload fw */
		vts_reset(ntnf_spi->vtsdev);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry_v2++;
	}
	if (esd_check_support) {
		queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work_v2,
				msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
	}
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_WDT_RECOVERY
static uint8_t recovery_cnt = 0;
static uint8_t nvt_wdt_fw_recovery(uint8_t *point_data)
{
   uint32_t recovery_cnt_max = 3;
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
static irqreturn_t nvt_ts_work_func(int irq, void *handle, ktime_t kt)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + FW_EVENT_LEN + 2] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
	uint32_t input_z = 1;
	int32_t i = 0;
	bool large_press = false;

	mutex_lock(&ntnf_spi->lock);

	ret = NTNF_CTP_SPI_READ(ntnf_spi->client, point_data, POINT_DATA_LEN + FW_EVENT_LEN + 1);
	if (ret < 0) {
		NVT_ERR("NTNF_CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}

#if NVT_TOUCH_WDT_RECOVERY
	/* ESD protect by WDT */
	if (nvt_wdt_fw_recovery(point_data)) {
		NVT_ERR("Recover for fw reset, %02X\n", point_data[1]);
		vts_abnormal_reset_collect(TOUCH_VCODE_UNEXPECTED_RESET_EVENT);
		ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_WDT_RECOVERY */

#if NVT_TOUCH_ESD_PROTECT
	/* ESD protect by FW handshake */
	if ((esd_check_support == 1) && nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable_v2(true);
		goto XFER_ERROR;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if POINT_DATA_CHECKSUM
	ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_LEN);
	if (ret < 0) {
		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
		goto XFER_ERROR;
	}
#endif /* POINT_DATA_CHECKSUM */

#if WAKEUP_GESTURE
	if (bTouchIsAwake == 0) {

	#if 0//POINT_DATA_CHECKSUM
		ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_LEN);
		if (ret < 0) {
			goto XFER_ERROR;
		}
	#endif /* POINT_DATA_CHECKSUM */
		
		input_id = (uint8_t)(point_data[1] >> 3);
		ntnf_wakeup_gesture_report(input_id, point_data);
		mutex_unlock(&ntnf_spi->lock);
		return IRQ_HANDLED;
	}
#endif

	//[20171023,jx]Catch Palm Event
	if ((point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x01)){
		VTI("got PalmOn");
		large_press = true;
		vts_report_event_down(ntnf_spi->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
	} else if ( (point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x02)){
		VTI("got PalmOFF");
		large_press = false;
		vts_report_release(ntnf_spi->vtsdev);
		vts_report_event_up(ntnf_spi->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
	}

	memset(ntnf_spi->press_id, 0 ,sizeof(ntnf_spi->press_id));
	for (i = 0; i < ntnf_spi->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ntnf_spi->max_touch_num))
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
			if ((input_x > ntnf_spi->abs_x_max) || (input_y > ntnf_spi->abs_y_max))
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

			ntnf_spi->press_id[input_id - 1] = 1;
			if (ntnf_spi->press_id_pre[input_id - 1] == 0)
				ntnf_spi->finger_cnt++;

			input_z = (input_x % 10 + input_y % 10 + 4) >> 1;
			ntnf_spi->custom_data[0] = point_data[66];
			ntnf_spi->custom_data[1] = point_data[67];
			ntnf_spi->down_x[input_id - 1] = input_x;
			ntnf_spi->down_y[input_id - 1] = input_y;
			ntnf_spi->down_z[input_id - 1] = input_z;
			vts_report_point_down(ntnf_spi->vtsdev, input_id - 1, ntnf_spi->finger_cnt, input_x, input_y, input_z, input_z, large_press, ntnf_spi->custom_data, 2, kt);
			;
		}
	}

	for (i = 0; i < ntnf_spi->max_touch_num; i++) {
		if (ntnf_spi->press_id_pre[i] == 1 && ntnf_spi->press_id[i] != 1) {
			ntnf_spi->finger_cnt--;
			vts_report_point_up(ntnf_spi->vtsdev, i, ntnf_spi->finger_cnt, ntnf_spi->down_x[i], ntnf_spi->down_y[i], ntnf_spi->down_z[i], ntnf_spi->down_z[i], false, kt);
		}
		ntnf_spi->press_id_pre[i] = ntnf_spi->press_id[i];
	}

XFER_ERROR:
	mutex_unlock(&ntnf_spi->lock);
	vts_report_point_sync(ntnf_spi->vtsdev);
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

		ntnf_bootloader_reset();

		//---set xdata index to 0x1F600---
		ntnf_set_page(0x1F600);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 7);
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
				ntnf_spi->trimid = trim_id_table[list].id;
				ntnf_spi->mmap = trim_id_table[list].mmap;
				ntnf_spi->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ntnf_spi->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				ret = 0;
				goto out;
			} else {
				ntnf_spi->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

static int last_touch_state = VTS_ST_NORMAL;

extern int8_t ntnf_customizeCmd_WaitSet_Spi(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd);
extern int8_t nvt_customizeExtCmd_WaitSet(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t *u8Cmd, uint8_t len);

/*0 is disable and 1 is enable*/
static void enable_or_disable_irq_wake(int enable)
{
	static int state_of_irq = 2;
	if (state_of_irq == enable) {
		return;
	}
	if (enable) {
		enable_irq_wake(ntnf_spi->client->irq);
		VTD("enable_irq_wake");
	} else {
		disable_irq_wake(ntnf_spi->client->irq);
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
static int bbk_nt_irq_enable(struct nvt_ts_data *ntnf_spi, bool enable)
{
	VTI("****bbk_nt_irq_enable**** %d  atomic: %d", enable, atomic_read(&ntnf_spi->irq_enabled));
	if (enable) {
		if (!atomic_cmpxchg(&ntnf_spi->irq_enabled, 0, 1)) {
			enable_irq(ntnf_spi->client->irq);
			VTI("===mode_change=== Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&ntnf_spi->irq_enabled, 1, 0)) {
			disable_irq(ntnf_spi->client->irq);
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
int ntnf_abnormal_reset_download_fw(void)
{
	int ret = 0;

	if (ntnf_spi == NULL || ntnf_spi->vtsdev == NULL) {
		VTE("vtsData is not initialize, not to download firmware");
		ret = -1;
		return ret;
	}

	if (last_touch_state != VTS_ST_NORMAL) {
		VTE("Touch is not in normal state, not to download firmware, now the state is %d", last_touch_state);
		ret = -1;
		return ret;
	}

	VTI("enter ntnf_abnormal_reset_download_fw");
	mutex_lock(&ntnf_spi->lock);
	bbk_nt_irq_enable(ntnf_spi, false);
	ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
	ret = ntnf_check_fw_reset_state(RESET_STATE_REK);
	if (ret) {
		VTE("nvt check fw reset fail");
	}
	bbk_nt_irq_enable(ntnf_spi, true);

	mutex_unlock(&ntnf_spi->lock);

	ntnf_set_charger_bit(ntnf_spi->vtsdev, ntnf_spi->charging);

	ntnf_set_rotation(ntnf_spi->vtsdev, 1);

	vts_report_release(ntnf_spi->vtsdev);

	VTI("exit ntnf_abnormal_reset_download_fw");

	return ret;
}


static int32_t nvt_ts_suspend_to_lowpower(struct device *dev)
{
	uint8_t buf[2] = {0};

	mutex_lock(&ntnf_spi->lock);

	NVT_LOG("start\n");

	//---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("end\n");

	return 0;
}

static int bbk_xxx_mode_change(struct vts_device *vtsdev, int which) {
	u8 buf[4];
	u32 sleep_time;
	u32 avee_time;
#ifndef CONFIG_MEDIATEK_SOLUTION
	int cs_retry = 0;
#endif
	
	#if (NVTFLASH_WORK_PROTECT)
		atomic_set(&u8_NT36xxx_flashWorking, 0);
		NVT_ERR("u8_NT36xxx_flashWorking=[%d]\n", atomic_read(&u8_NT36xxx_flashWorking));
	#endif
	
	if (ntnf_spi == NULL) {
		VTI("ts is NULL");
		return -EINVAL;
	}
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -EINVAL;
	}

	if(which == VTS_ST_NORMAL) {
		VTI("change to normal mode");
#ifndef CONFIG_MEDIATEK_SOLUTION
		if (gpio_is_valid(ntnf_spi->cs_gpio))
			gpio_set_value(ntnf_spi->cs_gpio, 1);
	#if defined(CONFIG_ARCH_QCOM)
		if (gpio_is_valid(ntnf_spi->miso_gpio))
			gpio_set_value(ntnf_spi->miso_gpio, 1);
	#endif
	
#endif
#if defined(CONFIG_MEDIATEK_SOLUTION)
		if (ntnf_spi->vddi_poweroff) {
			if (!IS_ERR_OR_NULL(ntnf_spi->spi_cs_active)) {
				pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_cs_active);
			}
			if (!IS_ERR_OR_NULL(ntnf_spi->spi_miso_active)) {
				pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_miso_active);
			}
		}
#endif
	
		
		vts_property_get(vtsdev, VTS_PROPERTY_SLEEP_TIME, &sleep_time);
		if (sleep_time) {
			VTI("force run normal resume");
			bTouchIsAwake = 0;
		}
		nvt_ts_resume(&ntnf_spi->client->dev);
		enable_or_disable_irq_wake(0);
		bbk_nt_irq_enable(ntnf_spi, true);
	}
	if(which == VTS_ST_GESTURE) {
		VTI("change to gesture mode");
		if (last_touch_state == VTS_ST_SLEEP) {		
			vts_dsi_panel_reset_power_ctrl(1);
#ifndef CONFIG_MEDIATEK_SOLUTION
			if (gpio_is_valid(ntnf_spi->cs_gpio))
			gpio_set_value(ntnf_spi->cs_gpio, 1);
	#if defined(CONFIG_ARCH_QCOM)
			if (gpio_is_valid(ntnf_spi->miso_gpio))
			gpio_set_value(ntnf_spi->miso_gpio, 1);
	#endif

#endif	
#if defined(CONFIG_MEDIATEK_SOLUTION)
			if (ntnf_spi->vddi_poweroff) {
				if (!IS_ERR_OR_NULL(ntnf_spi->spi_cs_active)) {
					pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_cs_active);
				}
				if (!IS_ERR_OR_NULL(ntnf_spi->spi_miso_active)) {
					pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_miso_active);
				}
			}
#endif
			//nvt_ts_resume(&ntnf_spi->client->dev);
			mutex_lock(&ntnf_spi->lock);
			ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
			ntnf_check_fw_reset_state(RESET_STATE_INIT);
			mutex_unlock(&ntnf_spi->lock);
			
		if(ntnf_spi->gesture_set.need_set){	
			buf[0] = 0x7f;
			buf[1] = 0x01;
			buf[2] = ntnf_spi->gesture_set.ges_bit[0];
			buf[3] = ntnf_spi->gesture_set.ges_bit[1];
			nvt_customizeExtCmd_WaitSet(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT,buf,4);
		}
			//mdelay(20);
			if(ntnf_spi->wkg_info){
				nvt_update_gesture_firmware_V2(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
			}
			else{	
				ntnf_customizeCmd_WaitSet_Spi(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT, 0x13);
			}
			
		}
		if (ntnf_spi->nt_4power)
			vts_dsi_panel_reset_power_ctrl(5);
		enable_or_disable_irq_wake(1);
		bbk_nt_irq_enable(ntnf_spi, true);
	}
	if(which == VTS_ST_SLEEP) {
		VTI("change to sleep mode");
		vts_property_get(vtsdev, VTS_PROPERTY_SLEEP_TIME, &sleep_time);
		if (sleep_time) {
			VTI("run nvt_ts_suspend in mode change!!!");
			nvt_ts_suspend(&ntnf_spi->client->dev);
		}
		
		if (last_touch_state != VTS_ST_SLEEP) {
			nvt_ts_suspend_to_lowpower(&ntnf_spi->client->dev);
			vts_property_get(vtsdev, VTS_PROPERTY_DELAY_AVEE, &avee_time);
			if (avee_time) {
				mdelay(avee_time);
				VTI("delay avee %dms after 0x11", avee_time);
			}
		}
#ifndef CONFIG_MEDIATEK_SOLUTION
		msleep(240);
		if (gpio_is_valid(ntnf_spi->cs_gpio)) {
			do {
				msleep(10);
				VTI("cs still high!!!");
				gpio_set_value(ntnf_spi->cs_gpio, 0);
				cs_retry++;
			} while (gpio_get_value(ntnf_spi->cs_gpio) && (cs_retry < 5));
		}
	#if defined(CONFIG_ARCH_QCOM)
		if (gpio_is_valid(ntnf_spi->miso_gpio))
			gpio_set_value(ntnf_spi->miso_gpio, 0);
	#endif

#endif
#if defined(CONFIG_MEDIATEK_SOLUTION)
		if (ntnf_spi->vddi_poweroff) {
			if (!IS_ERR_OR_NULL(ntnf_spi->spi_cs_sleep_pulllow)) {
				pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_cs_sleep_pulllow);
			}
			if (!IS_ERR_OR_NULL(ntnf_spi->spi_miso_sleep_pulllow)) {
				pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_miso_sleep_pulllow);
			}
		}
#endif
		vts_dsi_panel_reset_power_ctrl(0);
		VTI("touchscreen is shutdown!!!");

		/*NT datasheet requre more than 10ms between power_off and power_on*/
		msleep(30);

		enable_or_disable_irq_wake(0);
		bbk_nt_irq_enable(ntnf_spi, false);
	}

	last_touch_state = which;

	return 0;
}

static int nt_early_suspend_run(struct vts_device *vtsdev)
{
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}
	VTI("early suspend run,nt set 13 cmd");
	nvt_ts_suspend(&ntnf_spi->client->dev);
	return 0;
}

static int bbk_xxx_hw_init(struct vts_device *vtsdev)
{
	int ret;
	
	ret = nvt_gpio_config(ntnf_spi);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		return ret;
	}

#if defined(CONFIG_MEDIATEK_SOLUTION)
	if (ntnf_spi->cs_bootup) {
		if (!IS_ERR_OR_NULL(ntnf_spi->spi_cs_active)) {
			pinctrl_select_state(ntnf_spi->pinctrl, ntnf_spi->spi_cs_active);
		}
	}
#endif
	
#if NVT_TOUCH_SUPPORT_HW_RST
	if(gpio_is_valid(ntnf_spi->reset_gpio)){
		//---eng reset before TP_RESX high
		nvt_eng_reset_v2();
		gpio_set_value(ntnf_spi->reset_gpio, 1);
	}
#endif

	msleep(10);

	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		return -ENODEV;
	}

	NVT_LOG("int_trigger_type=%d\n", ntnf_spi->int_trigger_type);
	ret = vts_interrupt_register(vtsdev, ntnf_spi->client->irq, nvt_ts_work_func, ntnf_spi->int_trigger_type, ntnf_spi);
	if (ret != 0) {
		NVT_ERR("request irq failed. ret=%d\n", ret);
		return ret;
	}

	disable_irq(ntnf_spi->client->irq);
	atomic_set(&ntnf_spi->irq_enabled, 1);
	NVT_LOG("request irq %d succeed\n", ntnf_spi->client->irq);
/*
#if BOOT_UPDATE_FIRMWARE
	Ntnf_Boot_Update_Firmware(NULL);
#endif
*/
	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	if (esd_check_support) {
		INIT_DELAYED_WORK(&nvt_esd_check_work_v2, nvt_esd_check_func);

		nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
		queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work_v2,
				msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;
	NVT_LOG("end\n");

	enable_irq(ntnf_spi->client->irq);
	return 0;
}

static int bbk_xxx_update_firmware(struct vts_device *vtsdev, const struct firmware *firmware)
{
	if (ntnf_spi->mmap == NULL) {
		VTI("check NT ic failed");
		return -1;
	}
	Ntnf_Boot_Update_Firmware(NULL);
	return 0;
}

static int bbk_xxx_reset(struct vts_device *vtsdev)
{
	return bbk_xxx_update_firmware(vtsdev, NULL);
}

static int set_bit_zero_nt(u8 *psrc ,u8 bit_num)
{
 *psrc &= (~(0x01 << bit_num));
 return 0;
}
static int ntnf_get_gesture_bit(struct vts_device *vtsdev ,int gesture_type, u8 *buf)
{
	
	u8 i = 0;
	buf[0] = 0xff;
	buf[1] = 0x7f;
    set_bit_zero_nt(&buf[0],2);//V
	set_bit_zero_nt(&buf[0],4);//Z

	set_bit_zero_nt(&buf[1],0);//S
	//set_bit_zero_nt(&buf[1],5);//AT
	if(0 == (gesture_type & VTS_GESTURE_C)){
		VTI("C");
		set_bit_zero_nt(&buf[0],0);
		i++;
	}
	if(0 == (gesture_type & VTS_GESTURE_E)){
	set_bit_zero_nt(&buf[0],7);
	i++;
	VTI("E");
	}
	if(0 == (gesture_type & VTS_GESTURE_F)){
	set_bit_zero_nt(&buf[1],6);
	i++;
	VTI("F");
	}
	if(0 == (gesture_type & VTS_GESTURE_M)){
	set_bit_zero_nt(&buf[0],5);
	VTI("M");
	i++;
	}
	if(0 == (gesture_type & VTS_GESTURE_O)){
	set_bit_zero_nt(&buf[0],6);
	VTI("O");
	i++;
	}
	if(0 == (gesture_type & VTS_GESTURE_W)){
	set_bit_zero_nt(&buf[0],1);
	VTI("W");
	i++;
	}
	
	if(0 == (gesture_type & VTS_GESTURE_A)){
	set_bit_zero_nt(&buf[1],5);
	VTI("AT");
	i++;
	}
	
	//
	if(0 == (gesture_type & VTS_GESTURE_DCLICK)){
	set_bit_zero_nt(&buf[0],3);
	i++;
	VTI("DCLICK");
	}
	if(0 == (gesture_type & VTS_GESTURE_UP)){
	set_bit_zero_nt(&buf[1],1);
	i++;
	VTI("UP");
	}
	if(0 == (gesture_type & VTS_GESTURE_DOWN)){
	set_bit_zero_nt(&buf[1],2);
	i++;
	VTI("DOWN");
	}
	if(0 == (gesture_type & VTS_GESTURE_LR)){
	set_bit_zero_nt(&buf[1],3);
	set_bit_zero_nt(&buf[1],4);
	VTI("LR");
	i++;
	}
	VTI("01: 0x%x",buf[0]);
	VTI("02: 0x%x",buf[1]);
	
   return 0;
}

static int ntnf_update_gesture_bit(struct vts_device *vtsdev, int state)
{
	u8 buf[2];
	int gesture_state  = state;
	
	VTI("enter in NT gesture bit set, state = %d ", state);
	
	mutex_lock(&ntnf_spi->lock);
	ntnf_get_gesture_bit(vtsdev, gesture_state, &buf[0]);
	memcpy(ntnf_spi->gesture_set.ges_bit, buf, 2);
	ntnf_spi->gesture_set.need_set = 1;
	mutex_unlock(&ntnf_spi->lock);
	
	return 0;
}

static int card_regin_set(struct vts_device *vtsdev, int enable)
{
	//uint8_t u8A_customizedSlideLeft [6] = {0x03, 0xE8, 0x05, 0xDC, 0x01, 0x2C};
	uint8_t u8A_customizedSlideLeft [6] = 
	{
		(vtsdev->y0>>8) & 0xff, 
		vtsdev->y0 & 0xff, 
		(vtsdev->y1>>8) & 0xff, 
		vtsdev->y1 & 0xff, 
		(vtsdev->width>>8) & 0xff, 
		vtsdev->width & 0xff
	};
	uint8_t retry = 0;
	uint8_t buf[10] = {0};
	int ret = 0;

	if (enable == 0) {
		return 0;
	}

	for (retry = 0; retry < 3; retry++) {
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x7F;
		buf[2] = 0x04;
		buf[3] = u8A_customizedSlideLeft[0];//y0_H
		buf[4] = u8A_customizedSlideLeft[1];//y0_L
		buf[5] = u8A_customizedSlideLeft[2];//y1_H
		buf[6] = u8A_customizedSlideLeft[3];//y1_L
		buf[7] = u8A_customizedSlideLeft[4];//width_H
		buf[8] = u8A_customizedSlideLeft[5];//width_L
		ret = NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 9);
		if (ret) {
			NVT_ERR("write region cmd failed");
			continue;
		}

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		ret = NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);
		if (ret)
			NVT_ERR("read region cmd failed");

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 3)) {
		NVT_ERR("set customizedSlideLeft failed, buf[1]=0x%02X\n", buf[1]);
	} else {
		NVT_LOG("set customizedSlideLeft[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X], retry = %d\n"
			, buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], retry);
	}

	return ret;
}


static const struct vts_operations nvt_vts_ops = {
	.init = bbk_xxx_hw_init,
	.change_mode = bbk_xxx_mode_change,
	.get_frame = ntnf_get_frame,
	.get_fw_version = ntnf_get_chip_fw_version,
	.get_ic_mode = ntnf_get_touch_ic_mode,
	.set_charging = ntnf_set_charger_bit,
	.set_rotation = ntnf_set_rotation,
	.set_auto_idle = ntnf_set_idle,
	.process_package = ntnf_process_by_package,
	.early_suspend = nt_early_suspend_run,
	.update_firmware = bbk_xxx_update_firmware,
	.reset = bbk_xxx_reset,
	.update_gesture_bit = ntnf_update_gesture_bit,
	.set_input_method = nvt_set_input_method,
	.set_card_region = card_regin_set,
};

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client, struct device_node *np)
{
	int32_t ret = 0;
	struct vts_device *vtsdev = NULL;

	if (client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		return -EIO;
	}

	ntnf_spi = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (!ntnf_spi) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ntnf_spi->xbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1), GFP_KERNEL | GFP_DMA);
	if (ntnf_spi->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		if (ntnf_spi) {
			kfree(ntnf_spi);
			ntnf_spi = NULL;
		}
		return -ENOMEM;
	}
	ntnf_spi->g_data= (uint8_t *)kzalloc((NTV_GESTURE_FW_CHECK_LEN+2), GFP_KERNEL | GFP_DMA);
	if (ntnf_spi->g_data == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		if(ntnf_spi->xbuf){
			kfree(ntnf_spi->xbuf);
			ntnf_spi->xbuf = NULL;
		}
		if (ntnf_spi) {
			kfree(ntnf_spi);
			ntnf_spi = NULL;
		}
		return -ENOMEM;
	}

	ntnf_spi->node = np;
	ntnf_spi->client = client;
	ntnf_spi->dev = &client->dev;
	ntnf_spi->dev->of_node = np;	
	spi_set_drvdata(client, ntnf_spi);
	ntnf_spi->client->bits_per_word = 8;
	ntnf_spi->client->mode = SPI_MODE_0;
	ntnf_spi->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ntnf_spi->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	ntnf_spi->max_touch_num = TOUCH_MAX_FINGER_NUM;
	ntnf_spi->int_trigger_type = INT_TRIGGER_TYPE;
	mutex_init(&ntnf_spi->lock);
	mutex_init(&ntnf_spi->xbuf_lock);

	ret = spi_setup(ntnf_spi->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto errcode1;
	}
#ifdef CONFIG_SPI_MT65XX
	memcpy(&ntnf_spi->spi_ctrl, &spi_ctrdata, sizeof(struct mtk_chip_config));
	ntnf_spi->client->controller_data = (void *)&ntnf_spi->spi_ctrl;
#elif defined CONFIG_MTK_SPI
	memcpy(&ntnf_spi->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
	ntnf_spi->client->controller_data = (void *)&ntnf_spi->spi_ctrl;
#endif
#ifdef CONFIG_ARCH_BENGAL
	NVT_LOG("qcom 4250 set spi clk_cs delay ");
	memcpy(&ntnf_spi->qcom_4250_spi_ctrl, &qcom_4125_ctrl_data, sizeof(struct spi_geni_qcom_ctrl_data));
	ntnf_spi->client->controller_data = (void *)&ntnf_spi->qcom_4250_spi_ctrl;
#endif
	
	//---parse dntnf_spi---
	ret = nvt_parse_dt(np);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto errcode2;
	}

	client->irq = gpio_to_irq(ntnf_spi->irq_gpio);
	ntnf_spi->client->max_speed_hz = ntnf_spi->spi_frequency;
	
	NVT_LOG("mode=%d, max_speed_hz=%d\n", ntnf_spi->client->mode, ntnf_spi->client->max_speed_hz);

	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto errcode3;
	}

	ret = ntnf_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto errcode4;
	}

	ret = ntnf_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto errcode5;
	}

	vtsdev = vts_device_alloc();
	if(!vtsdev) {
		NVT_ERR("vivoTsAlloc fail");
		ret = -ENOMEM;
		goto errcode6;
	}

	vtsdev->ops = &nvt_vts_ops;
	vtsdev->busType = BUS_SPI;
	ntnf_spi->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, np);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto errcode7;
	}
	vts_set_drvdata(vtsdev, ntnf_spi);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {
		NVT_ERR("vts_register_driver failed");
		goto errcode7;
	}
	vts_property_get(vtsdev, VTS_PROPERTY_ESD_CHECK, &esd_check_support);
	VTI("esd check support value %d", esd_check_support);
	return 0;

errcode7:
	vts_device_free(vtsdev);
errcode6:
	nvt_mp_proc_deinit_Spi();
errcode5:
	nvt_extra_proc_deinit_Spi_Spi();
errcode4:
errcode3:
	nvt_flash_proc_deinit();
errcode2:
errcode1:
	mutex_destroy(&ntnf_spi->lock);
	mutex_destroy(&ntnf_spi->xbuf_lock);
	spi_set_drvdata(client, NULL);
	kfree(ntnf_spi->g_data);
	kfree(ntnf_spi->xbuf);
	kfree(ntnf_spi);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct spi_device *client, struct device_node *np)
{
	vts_unregister_driver(ntnf_spi->vtsdev);
	vts_device_free(ntnf_spi->vtsdev);
	nvt_mp_proc_deinit_Spi();
	nvt_extra_proc_deinit_Spi_Spi();
	nvt_flash_proc_deinit();
	mutex_destroy(&ntnf_spi->lock);
	mutex_destroy(&ntnf_spi->xbuf_lock);
	spi_set_drvdata(client, NULL);
	kfree(ntnf_spi);
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq && esd_check_support)
		destroy_workqueue(nvt_esd_check_wq);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
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
	
	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	mutex_lock(&ntnf_spi->lock);

	NVT_LOG("start\n");

	bTouchIsAwake = 0;

#if NVT_TOUCH_ESD_PROTECT
	if (esd_check_support) {
		cancel_delayed_work_sync(&nvt_esd_check_work_v2);
		nvt_esd_check_enable_v2(false);
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if WAKEUP_GESTURE	
	if(ntnf_spi->gesture_set.need_set){
		buf[0] = 0x7f;
		buf[1] = 0x01;
		buf[2] = ntnf_spi->gesture_set.ges_bit[0];
		buf[3] = ntnf_spi->gesture_set.ges_bit[1];
		nvt_customizeExtCmd_WaitSet(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT,buf,4);
	}
	if (ntnf_spi->wkg_info) {
		nvt_update_gesture_firmware_V2(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
	}else{
		//---write spi command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;

		NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf,2);
	}
	enable_irq_wake(ntnf_spi->client->irq);

	NVT_LOG("Enabled touch wakeup gesture\n");

#else // WAKEUP_GESTURE
	disable_irq(ntnf_spi->client->irq);

	//---write spi command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 2);
#endif // WAKEUP_GESTURE
	msleep(50);

	mutex_unlock(&ntnf_spi->lock);

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

	mutex_lock(&ntnf_spi->lock);

	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	if(gpio_is_valid(ntnf_spi->reset_gpio))
		gpio_set_value(ntnf_spi->reset_gpio, 1);
#endif
	ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
	ntnf_check_fw_reset_state(RESET_STATE_REK);

#if !WAKEUP_GESTURE
	enable_irq(ntnf_spi->client->irq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (esd_check_support) {
		queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work_v2,
				msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	bTouchIsAwake = 1;

	mutex_unlock(&ntnf_spi->lock);

	NVT_LOG("end\n");

	return 0;
}
void nvts_shut_down(struct spi_device *spi)
{
	VTI("nvts shut down !!!");

	bbk_nt_irq_enable(ntnf_spi, false);
	if (gpio_is_valid(ntnf_spi->cs_gpio))
		gpio_set_value(ntnf_spi->cs_gpio, 0);

	vts_dsi_panel_reset_power_ctrl(8);
}

static struct vts_spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.compatible = "novatek,NVT-ts-spi-v2",
	.shutdown = nvts_shut_down,
};

static const int ic_numbers[] = {VTS_IC_NT36672, VTS_IC_NT36670, VTS_IC_NT36675,VTS_IC_NT36525};
module_vts_driver(nt_no_flash, ic_numbers, vts_spi_drv_reigster(&nvt_spi_driver), vts_spi_drv_unreigster(&nvt_spi_driver));

