/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 14651 $
 * $Date: 2017-07-21 14:58:50 +0800 (Fri, 21 Jul 2017) $
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
/*#include <linux/wakelock.h>*/
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/vivo_ts_function.h>

#include "nt36xxx.h"
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"


#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
uint8_t esd_retry_max = 5;
#endif

static void enable_or_disable_irq_wake(int enable);
static void enable_or_disable_irq(int enable);

#if NVT_TOUCH_EXT_PROC
extern int32_t nvt_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nvt_mp_proc_init(void);
#endif

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

#if 0
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif
#endif
static const struct nvt_ts_mem_map NT36672A_memory_map = {
	.EVENT_BUF_ADDR           = 0x21C00,
	.RAW_PIPE0_ADDR           = 0x20000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x23000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x20BFC,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x23BFC,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x206DC,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x236DC,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x20510,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x23510,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x20BF0,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x23BF0,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x24000,
	.RW_FLASH_DATA_ADDR       = 0x24002,
};

static const struct nvt_ts_mem_map NT36772_memory_map = {
	.EVENT_BUF_ADDR           = 0x11E00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10E70,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12E70,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x10830,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x12830,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10E60,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12E60,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10E68,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12E68,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

static const struct nvt_ts_mem_map NT36525_memory_map = {
	.EVENT_BUF_ADDR           = 0x11A00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10B08,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12B08,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x1064C,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x1264C,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10634,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12634,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

static const struct nvt_ts_mem_map NT36870_memory_map = {
	.EVENT_BUF_ADDR           = 0x25000,
	.RAW_PIPE0_ADDR           = 0x20000,
	.RAW_PIPE0_Q_ADDR         = 0x204C8,
	.RAW_PIPE1_ADDR           = 0x23000,
	.RAW_PIPE1_Q_ADDR         = 0x234C8,
	.BASELINE_ADDR            = 0x21350,
	.BASELINE_Q_ADDR          = 0x21818,
	.BASELINE_BTN_ADDR        = 0x24350,
	.BASELINE_BTN_Q_ADDR      = 0x24358,
	.DIFF_PIPE0_ADDR          = 0x209B0,
	.DIFF_PIPE0_Q_ADDR        = 0x20E78,
	.DIFF_PIPE1_ADDR          = 0x239B0,
	.DIFF_PIPE1_Q_ADDR        = 0x23E78,
	.RAW_BTN_PIPE0_ADDR       = 0x20990,
	.RAW_BTN_PIPE0_Q_ADDR     = 0x20998,
	.RAW_BTN_PIPE1_ADDR       = 0x23990,
	.RAW_BTN_PIPE1_Q_ADDR     = 0x23998,
	.DIFF_BTN_PIPE0_ADDR      = 0x21340,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0x21348,
	.DIFF_BTN_PIPE1_ADDR      = 0x24340,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0x24348,
	.READ_FLASH_CHECKSUM_ADDR = 0x24000,
	.RW_FLASH_DATA_ADDR       = 0x24002,
};

static const struct nvt_ts_mem_map NT36676F_memory_map = {
	.EVENT_BUF_ADDR           = 0x11A00,
	.RAW_PIPE0_ADDR           = 0x10000,
	.RAW_PIPE0_Q_ADDR         = 0,
	.RAW_PIPE1_ADDR           = 0x12000,
	.RAW_PIPE1_Q_ADDR         = 0,
	.BASELINE_ADDR            = 0x10B08,
	.BASELINE_Q_ADDR          = 0,
	.BASELINE_BTN_ADDR        = 0x12B08,
	.BASELINE_BTN_Q_ADDR      = 0,
	.DIFF_PIPE0_ADDR          = 0x1064C,
	.DIFF_PIPE0_Q_ADDR        = 0,
	.DIFF_PIPE1_ADDR          = 0x1264C,
	.DIFF_PIPE1_Q_ADDR        = 0,
	.RAW_BTN_PIPE0_ADDR       = 0x10634,
	.RAW_BTN_PIPE0_Q_ADDR     = 0,
	.RAW_BTN_PIPE1_ADDR       = 0x12634,
	.RAW_BTN_PIPE1_Q_ADDR     = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0x10AFC,
	.DIFF_BTN_PIPE0_Q_ADDR    = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0x12AFC,
	.DIFF_BTN_PIPE1_Q_ADDR    = 0,
	.READ_FLASH_CHECKSUM_ADDR = 0x14000,
	.RW_FLASH_DATA_ADDR       = 0x14002,
};

#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
  {.id = {0x0A, 0xFF, 0xFF, 0x72, 0x65, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x0A, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x0A, 0xFF, 0xFF, 0x82, 0x66, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x0A, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x0B, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x0A, 0xFF, 0xFF, 0x72, 0x67, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
  {.id = {0x55, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0x55, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xAA, 0x00, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xAA, 0x72, 0xFF, 0x00, 0x00, 0x00}, .mask = {1, 1, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x72, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x67, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x72, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36772_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x25, 0x65, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36525_memory_map, .carrier_system = 0},
  {.id = {0xFF, 0xFF, 0xFF, 0x70, 0x68, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36870_memory_map, .carrier_system = 1},
  {.id = {0xFF, 0xFF, 0xFF, 0x76, 0x66, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
          .mmap = &NT36676F_memory_map, .carrier_system = 0},
  {.id = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, .mask = {1, 1, 1, 1, 1, 1},
          .mmap = &NT36672A_memory_map, .carrier_system = 0},
};


#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
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
};
#endif

uint8_t bTouchIsAwake_672;

#if 1
static int qup_i2c_suspended;
#else
extern int qup_i2c_suspended;
#endif

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;
	int retry_count = 0;  
	
	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while(qup_i2c_suspended > 0) { 
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) { 
			VTI("after 60ms delay , fts iic is still  on suspend.");
			return -EIO;
		}
	}

	while (retries < 5) {
		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		ret = i2c_transfer(client->adapter, msgs, 2);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		if (ret == 2)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;

	ret = CTP_I2C_READ(client, address, buf, 2);
	if (ret < 0)
		NVT_ERR("CTP_I2C_READ_DUMMY failed.(%d)\n", ret);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;
	int retry_count = 0;  

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while(qup_i2c_suspended > 0) { 
		VTI("iic is on suspend.delay %dms.", (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) { 
			VTI("after 60ms delay , fts iic is still  on suspend.");
			return -EIO;
		}
	}

	while (retries < 5) {
		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		ret = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		if (ret == 1)
			break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		vivoTsGetVtsData()->iicErrorNum++;
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
int nvt_sw_reset_idle(void)
{
	int ret = 0;
	uint8_t buf[4]={0};

	//---write i2c cmds to reset idle---
	buf[0]=0x00;
	buf[1]=0xA5;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	//---write i2c cmds to reset---
	buf[0] = 0x00;
	buf[1] = 0x69;
	CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

	// need 35ms delay after bootloader reset
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if (buf[1] == 0x00)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		ret = -1;
		return ret;
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
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		buf[0] = 0xFF;
		buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		msleep(10);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		ret = -1;
		return ret;
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
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			NVT_LOG("check_reset_state=%04X\n", buf[1]);  //[20180128]
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > 100)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
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
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];
	//snprintf(ts->nvt_pid, sizeof(ts->nvt_pid), "%02X%02X", buf[2], buf[1]);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 3);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_I2C_READ(ts->client, I2C_FW_Address, buf, 17);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	ts->max_button_num = buf[11];

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 18;
		ts->y_num = 36;
		ts->abs_x_max = 1080;
		ts->abs_y_max = 2280;
		ts->max_button_num = 0;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, \
					abs_x_max=%d, abs_y_max=%d, max_button_num=%d!\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max, ts->max_button_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_PROC
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash read function.

return:
	Executive outcomes. 2---succeed. -5,-14---failed.
*******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
/*	int32_t retries = 0;*/
	int8_t i2c_wr = 0;

	if (vivoTsGetState() == TOUCHSCREEN_SLEEP) {
		VTI("sleep mode, not to do nvt_flash_read");
		return -EIO;
	}

	if (count > sizeof(str)) {
		NVT_ERR("error count=%zu\n", count);
		return -EFAULT;
	}

	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif	
	
	i2c_wr = str[0] >> 7;

	if (i2c_wr == 0) {	//I2C write
		ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
		if (ret < 0) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}
		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
		if (ret < 0) {
			NVT_ERR("error, ret=%d\n", ret);
			return -EIO;
		}
		// copy buff to user if i2c transfer
		if (copy_to_user(buff, str, count))
			return -EFAULT;
		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}

/*	if (i2c_wr == 0) {	//I2C write
		while (retries < 20) {
			ret = CTP_I2C_WRITE(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 1)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else if (i2c_wr == 1) {	//I2C read
		while (retries < 20) {
			ret = CTP_I2C_READ(ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		// copy buff to user if i2c transfer
		if (retries < 20) {
			if (copy_to_user(buff, str, count))
				return -EFAULT;
		}

		if (unlikely(retries == 20)) {
			NVT_ERR("error, ret = %d\n", ret);
			return -EIO;
		}

		return ret;
	} else {
		NVT_ERR("Call error, str[0]=%d\n", str[0]);
		return -EFAULT;
	}
}*/

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash open function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;
	VTI("apk open NVT flash");
	ts->is_app_test = 1;
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
	Novatek touchscreen /proc/NVTflash close function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;
	VTI("apk close NVT flash");
	ts->is_app_test = 0;
	if (dev)
		kfree(dev);

	return 0;
}

static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/NVTflash initial function.

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
	NVT_LOG("Create /proc/NVTflash\n");
	NVT_LOG("============================================================\n");

	return 0;
}
#endif

#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24
#define GESTURE_EXT_WORD_AT		(30)	/*'@'	forVIVO prj */
#define GESTURE_EXT_WORD_F		(31)	/*'F'	forVIVO prj */
//static struct wake_lock gestrue_wakelock;

struct WakeUpTrace{
	uint8_t id;
	uint8_t OClockwise;
	uint16_t u16aX[9];
	uint16_t u16aY[9];
}gsWakeUpTrace;
/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t point_data[64 + 2])
{
	int i = 0;

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			VTI("Gesture : Word-C.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_C, -1, -1, -1);
			break;
		case GESTURE_WORD_W:
			VTI("Gesture : Word-W.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_W, -1, -1, -1);
			break;
		case GESTURE_WORD_V:
			VTI("Gesture : Word-V.");
			break;
		case GESTURE_DOUBLE_CLICK:
			VTI("Gesture : Double Click.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_WAKEUP, -1, -1, -1);
			break;
		case GESTURE_WORD_Z:
			VTI("Gesture : Word-Z.");
			break;
		case GESTURE_WORD_M:
			VTI("Gesture : Word-M.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_M, -1, -1, -1);
			break;
		case GESTURE_WORD_O:
			VTI("Gesture : Word-O.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_O, -1, -1, -1);
			break;
		case GESTURE_WORD_e:
			VTI("Gesture : Word-e.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_E, -1, -1, -1);
			break;
		case GESTURE_WORD_S:
			VTI("Gesture : Word-S.");
			break;
		case GESTURE_SLIDE_UP:
			VTI("Gesture : Slide UP.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_UP, -1, -1, -1);
			break;
		case GESTURE_SLIDE_DOWN:
			VTI("Gesture : Slide DOWN.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_WAKEUP_SWIPE, -1, -1, -1);
			break;
		case GESTURE_SLIDE_LEFT:
			VTI("Gesture : Slide LEFT.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_LEFT, -1, -1, -1);
			break;
		case GESTURE_SLIDE_RIGHT:
			VTI("Gesture : Slide RIGHT.");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_RIGHT, -1, -1, -1);
			break;
	case GESTURE_EXT_WORD_AT:
			VTI("Gesture : A");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_A, -1, -1, -1);
			break;
	case GESTURE_EXT_WORD_F:
			VTI("Gesture : F");
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_F, -1, -1, -1);
			break;
		default:
			break;
	}
	
	if ((gesture_id != GESTURE_DOUBLE_CLICK && gesture_id >= 12 && gesture_id <= 26 && gesture_id != 22 && gesture_id != 23 && gesture_id != 24) || (gesture_id == 30) || (gesture_id == 31)) {
		gsWakeUpTrace.id = point_data[3];
		gsWakeUpTrace.OClockwise = point_data[43];
		for(i = 0; i<9; i++) {
			gsWakeUpTrace.u16aX[i] = (point_data[4*i+5]<<8)+point_data[4*i+4];
			gsWakeUpTrace.u16aY[i] = (point_data[4*i+7]<<8)+point_data[4*i+6];
		}

		VTI("input_id=[%02d],OClockwise[%02X]", gsWakeUpTrace.id, gsWakeUpTrace.OClockwise);
		for(i = 0; i < 9; i++) {
			VTI("%d(%02d,%02d)", i, gsWakeUpTrace.u16aX[i], gsWakeUpTrace.u16aY[i]);					
		}				
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
static void nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;

	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

}
#else
static void nvt_parse_dt(struct device *dev)
{
	ts->irq_gpio = NVTTOUCH_INT_PIN;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}

	return ret;

err_request_irq_gpio:
	return ret;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	if(enable) {
		/* update interrupt timer */
		irq_timer = jiffies;
	}
	/* enable/disable esd check flag */
	esd_check = enable;
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

	NVT_ERR("esd_check = %d (retry %d/%d)\n", esd_check, esd_retry, esd_retry_max);	//DEBUG

	if (esd_retry >= esd_retry_max)
		nvt_esd_check_enable(false);

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, bootloader reset */
		nvt_bootloader_reset();
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif

#define POINT_DATA_LEN 65
#define FW_EVENT_LEN 5
static int last_press_id[TOUCH_MAX_FINGER_NUM] = {0};

/*******************************************************
Description:
	Novatek touchscreen check i2c packet checksum function.
return:
	Executive outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t nvt_ts_point_data_checksum(uint8_t *buf, uint8_t length)
{
	uint8_t checksum = 0;
	int32_t i = 0;
	int ret = 0;

	/*Generate checksum*/
	for (i = 0; i < length - 1; i++) {
		checksum += buf[i + 1];
	}
	checksum = (~checksum + 1);

	/*Compare checksum and dump fail data*/
	if (checksum != buf[length]) {
		VTI("i2c packet checksum not match. (point_data[%d]=0x%02x, checksum=0x%02x)", length, buf[length], checksum);
		for (i = 0; i < 10; i++) {
			VTI("%02x %02x %02x %02x %02x %02x", buf[1 + i * 6], buf[2 + i * 6], buf[3 + i * 6], buf[4 + i * 6], buf[5 + i * 6], buf[6 + i * 6]);
		}
		for (i = 0; i < (length - 60); i++) {
			VTI("%02x", buf[1 + 60 + i]);
		}
		ret = -1;
		return ret;
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static void nvt_ts_work_func(void* work)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + FW_EVENT_LEN + 1] = {0};
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

	mutex_lock(&ts->lock);

	ret = CTP_I2C_READ(ts->client, I2C_FW_Address, point_data, POINT_DATA_LEN + FW_EVENT_LEN + 1);
	if (ret < 0) {
		NVT_ERR("CTP_I2C_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//--- dump I2C buf ---
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1+i*6], point_data[2+i*6], point_data[3+i*6], point_data[4+i*6], point_data[5+i*6], point_data[6+i*6]);
	}
	printk("\n");
*/

#if NVT_TOUCH_ESD_PROTECT
	//NVT_ERR("%02X %02X %02X\n", point_data[1], point_data[2], point_data[3]);	//DEBUG
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif

	ts->fw_debug_info[0] = point_data[66];
	ts->fw_debug_info[1] = point_data[67];

	if ((point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x01)) {
		VTI("got PalmOn");
		vivoTsGetVtsData()->largePressNum++;
		/* <VIVO can do action for Palm On> */
		vivoTsCoverMute(0, 1);
		/*enable_irq(ts->client->irq);*/
		enable_or_disable_irq(1);
		mutex_unlock(&ts->lock);
		return;
	} else if ((point_data[1]==0xF0) && (point_data[2]==0x04) && (point_data[3]==0x02)) {
		printk("[NVT-ts] got PalmOFF\n");
		/*enable_irq(ts->client->irq);*/
		enable_or_disable_irq(1);
		mutex_unlock(&ts->lock);
		return;
	}

#if WAKEUP_GESTURE
	if (bTouchIsAwake_672 == 0) {
		/*to avoid mistaken trigger of gesture during power on by caculate the checksum*/
		ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_LEN);
		if (ret < 0) {
			goto XFER_ERROR;
		}

		input_id = (uint8_t)(point_data[1] >> 3);
		NVT_LOG("[NVT-ts] %02X %02X %02X\n", point_data[1], point_data[2], point_data[3]);	//question
		if ((point_data[1] == 0xF0) && (point_data[2] == 0x01)) {	/* judge f and @ gesture */
			input_id = point_data[3];	
		} else if (point_data[1] > 0xF0) {
			VTE("input_id %d is invalid, func_type=%d, func_id=%d", input_id, point_data[2], point_data[3]);
			/*enable_irq(ts->client->irq);*/
			enable_or_disable_irq(1);
			mutex_unlock(&ts->lock);
			return;
		}
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		/*enable_irq(ts->client->irq);*/
		enable_or_disable_irq(1);
		mutex_unlock(&ts->lock);
		return;
	}
#endif

	/* get the point num */
	finger_cnt = 0;
	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id>ts->max_touch_num) || (input_id<=0)) {
			continue;
		}

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
				continue;
			finger_cnt++;
		}
	}

	ret = vivoTsCoverMute(finger_cnt, 0);
	if (ret < 0) {
		if (-2 == ret) {
			VTI("enter larger or 3 finger mode");
			/*enable_irq(ts->client->irq);*/
			enable_or_disable_irq(1);
			mutex_unlock(&ts->lock);
			return;
		}
		if (-3 == ret) {
			VTI("lcd shutoff,not report points");
			/*enable_irq(ts->client->irq);*/
			enable_or_disable_irq(1);
			mutex_unlock(&ts->lock);
			return;
		}
	}
	
	finger_cnt = 0;
	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id>ts->max_touch_num) || (input_id<=0)) {
			continue;
		}

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			if(esd_check) {
				irq_timer = jiffies;
			}
#endif		
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
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

			press_id[input_id - 1] = 1;
			input_z = (input_x%10+input_y%10+4)>>1;
			input_z |= (point_data[66] << 8) | (point_data[67] << 16);
			vivoTsInputReport(VTS_TOUCH_DOWN, input_id - 1, input_x, input_y, input_z);
			finger_cnt++;
		}
	}
	
	for (i = 0; i < ts->max_touch_num; i++) {
		if (last_press_id[i] == 1 && press_id[i] == 0) {
			/*up */
			vivoTsInputReport(VTS_TOUCH_UP, i, input_x, input_y, input_p);
		}
	}
	if(finger_cnt == 0) {
		vivoTsReleasePoints();
	}


	for(i=0; i<ts->max_touch_num; i++) {
		last_press_id[i] = press_id[i];
	}
	
XFER_ERROR:
	/*enable_irq(ts->client->irq);*/
	enable_or_disable_irq(1);
	mutex_unlock(&ts->lock);
}

/*******************************************************
Description:
	External interrupt service routine.

return:
	irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int32_t irq, void *dev_id)
{
	spin_lock(&ts->irq_spinlock);
	if (ts->irq_enable) {
	disable_irq_nosync(ts->client->irq);
		VTD("disable_irq_nosync");
		ts->irq_enable = 0;
	}
	spin_unlock(&ts->irq_spinlock);

	//nvt_ts_work_func(NULL);
	vtsIrqBtmThreadWake();
	return IRQ_HANDLED;
}
/*******************************************************
Description:
	Novatek touchscreen check and stop crc reboot loop.

return:
	n.a.
 *******************************************************/
void nvt_stop_crc_reboot(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	VTI("enter nvt_stop_crc_reboot");
	/*read dummy buffer to check CRC fail reboot is happening or not*/

	/*---change I2C index to prevent geting 0xFF, but not 0xFC---*/
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF6;
	CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

	/*---read to check if buf is 0xFC which means IC is in CRC reboot ---*/
	buf[0] = 0x4E;
	CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 4);

	if ((buf[1] == 0xFC) ||
		((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

		/*IC is in CRC fail reboot loop, needs to be stopped!*/
		for (retry = 5; retry > 0; retry--) {

			/*---write i2c cmds to reset idle : 1st---*/
			buf[0] = 0x00;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);

			/*---write i2c cmds to reset idle : 2rd---*/
			buf[0] = 0x00;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			msleep(1);

			/*---clear CRC_ERR_FLAG---*/
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 2);

			/*---check CRC_ERR_FLAG---*/
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0x00;
			CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 2);

			if (buf[1] == 0xA5)
				break;
		}
		if (retry == 0)
			NVT_ERR("CRC auto reboot is not able to be stopped! buf[1]=0x%02X\n", buf[1]);
	}
	return;
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

	nvt_bootloader_reset(); /*NOT in retry loop */
	/*---Check for 5 times---*/
	for (retry = 3; retry > 0; retry--) {
		/*nvt_bootloader_reset(); */
		ret = nvt_sw_reset_idle();
		if (ret < 0) {
			break;
		}

		buf[0] = 0x00;
		buf[1] = 0x35;
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			break;
		}
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
		if (ret < 0) {
			break;
		}

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 7);
		if (ret < 0) {
			break;
		}
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		/*---Stop CRC check to prevent IC auto reboot---*/
		if ((buf[1] == 0xFC) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
			continue;
		}

		/* compare read chip id on supported list*/
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			/* compare each byte */
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
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}
		/*
		if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nvt_stop_crc_reboot();
		}*/

		msleep(10);
	}

out:
	return ret;
}

static int nt_get_gesture_point(u16 *data)
{
	uint8_t i;

	VTI("enter");
	for(i = 0; i < 9; i++) {
		data[i*2]  =gsWakeUpTrace.u16aX[i];
		data[i*2+1]=gsWakeUpTrace.u16aY[i];					   
	}		

	VTI("end");
	
	return 10;//10means Points
}

static int nt_get_module_id(void)
{
	return VTS_MVC_TRY; // default TRY
}

extern int nt_get_ic_fw_version(void);
static int nt_get_fw_version(int which)
{
	int ver = 0;

	if(CONFIG_VERSION == which) {
		ver = -1;
	}
	if(FW_VERSION == which) {
		ver = nt_get_ic_fw_version();
	}

	return ver;
}

extern int hand_update_firmware(const struct firmware *firmware);
static int nt_fw_update(const struct firmware *firmware)
{
	//fts_ctpm_fw_upgrade_bin_vivo(fts_i2c_client, (unsigned char *)firmware->data, firmware->size);
	int ret = 0;
	ret = hand_update_firmware(firmware);
	return ret;
}

static int last_ts_state;
static int32_t nvt_ts_resume(struct device *dev);
static int32_t nvt_ts_suspend(int status);
extern int8_t nvt_customizeCmd_WaitSet(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd);

#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
#else
static int mdss_dsi_panel_reset_and_powerctl(int enable)
{
	return 0;
}
#endif

/*0 is disable and 1 is enable*/
static void enable_or_disable_irq_wake(int enable)
{
	static int state_of_irq = 2;
	if (state_of_irq == enable) {
		return;
	}
	if (enable) {
		enable_irq_wake(ts->client->irq);
		VTD("enable_irq_wake");
	} else {
		disable_irq_wake(ts->client->irq);
		VTD("disable_irq_wake");
	}
	state_of_irq = enable;
	return;
}

/*0 is disable and 1 is enable*/
static void enable_or_disable_irq(int enable)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts->irq_spinlock, irqflags);

	if (ts->irq_enable == enable) {
		spin_unlock_irqrestore(&ts->irq_spinlock, irqflags);
		return;
	}
	if (enable) {
		enable_irq(ts->client->irq);
		VTD("enable_irq");
	} else {
		disable_irq_nosync(ts->client->irq);
		VTD("disable_irq_nosync");
	}
	ts->irq_enable = enable;

	spin_unlock_irqrestore(&ts->irq_spinlock, irqflags);

	return;
}


static int nt_mode_change(int which)
{

	if(which == TOUCHSCREEN_NORMAL) {
		VTI("change to normal mode");
		//normal power on set by lcm,tp not contronl
		nvt_ts_resume(&ts->client->dev);
		enable_or_disable_irq_wake(0);
		enable_or_disable_irq(1);
	} else {
		ts->is_app_test = 0;
	}

	if(which == TOUCHSCREEN_GESTURE) {
		VTI("change to gesture mode");
	
		if (last_ts_state == TOUCHSCREEN_SLEEP) {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(1);
			mdelay(50);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		}

		if (TOUCHSCREEN_SLEEP == last_ts_state) {
			/*nvt_ts_suspend(TOUCHSCREEN_GESTURE);*/
			mdelay(40);
			VTI("JUST FOR DEBUG");
			nvt_customizeCmd_WaitSet(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT, 0x13);
		}
		enable_or_disable_irq_wake(1);
		enable_or_disable_irq(1);
	}

	if (which == TOUCHSCREEN_SLEEP) {
		VTI("change to sleep mode");

		enable_or_disable_irq_wake(0);
		enable_or_disable_irq(0);

		if (last_ts_state != TOUCHSCREEN_SLEEP) {
			nvt_ts_suspend(TOUCHSCREEN_SLEEP);
		}

		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		mdss_dsi_panel_reset_and_powerctl(0);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
	}

	last_ts_state = which;
	
	return 0;
}

extern unsigned int mdss_report_lcm_id(void);
static int nt_get_lcm_id(void)
{
	int lcm_id;
	lcm_id = mdss_report_lcm_id();
	VTI("lcm id: 0x%x", lcm_id);
	return lcm_id;
}

static int get_iic_bus_state(void)
{
	return qup_i2c_suspended;
}

static int nt_early_suspend_run(void)
{
	VTI("early suspend run,nt set 13 cmd");
	nvt_ts_suspend(TOUCHSCREEN_GESTURE);
	return 0;
}

static int nvt_get_header_fw_version(int fwOrConfig, unsigned char* fw)
{
	int version = 0;

	if(fwOrConfig == CONFIG_VERSION) {
		version = -1;
	}

	if(fwOrConfig == FW_VERSION) {	
		//fw = vivoTsGetFw(VTS_FW_TYPE_FW, &size);
		version = fw[0x1A000];		//#define FW_BIN_VER_OFFSET 0x1A000
	}
	
	return version;
}

static int bbk_nt_get_fw_debug_info(unsigned char *buf)
{
	int ret = 0, i = 0;
	unsigned char result[8] = {0};
	unsigned char workFreq = ts->fw_debug_info[1] >> 4;

	for (i = 0; i < 8; i++) {
		result[i] = (ts->fw_debug_info[0] >> i) & 1;
	}

	VTI("get fw_debug_info:0x%x, 0x%x", ts->fw_debug_info[0], ts->fw_debug_info[1]);
	ret = snprintf(buf, 255, "Water Reject Mode: %d\nActive Mode: %d\nCharge Mode: %d\nPalm Mode: %d\nGesture Mode: %d\nWorking Frequency: %d\n", result[6], result[0], result[7], result[5], result[2], workFreq);

	ts->fw_debug_info[0] = 0;
	ts->fw_debug_info[1] = 0;
	return ret;
}


extern int vivo_nt_set_charger_bit(int state);
extern int vivo_nt_idle_ctl(int state);
extern int vivo_nt_set_edge_restain_switch(int on);
extern int vivo_nt_get_rawordiff_data(int which, int *data);
extern int vivo_nt_readUdd(unsigned char *udd);
extern int vivo_nt_writeUdd(unsigned char *udd);
extern int bbk_nt_sensor_test(char *buf, void *pdata, int tmp);

#include "fw_tm.h"
#include "fw_xl.h"
#include "fw_jdi.h"
#include "fw_boe.h"
/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
	int err = 0;
	struct vivo_ts_struct *vts_data = NULL;

	VTI("start");

	vts_data = vivoTsAlloc();	
	if(vts_data == NULL) {	
		VTE("vts_data alloc fail");
		ret = -1;
		return ret;
	}

	vts_data->client = client;
	vts_data->tsDimensionX = 1080;		//pdata->x_max;	
	vts_data->tsDimensionY = 2280;		//pdata->y_max;	
	vivoTsSetTxRxNum(18, 36);
	//must set for key logic	
	//vts_data->vkeyString = NULL;	
	vts_data->keyType = VTS_KEY_NOKEY;	
	vts_data->isIndependentKeyIc = 0;
	vts_data->hasFingerPrint = 1;
	vts_data->hasHomeKey = 0;
	vts_data->hasMenuKey = 0;
	vts_data->hasBackKey = 0;

	vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
	vts_data->irqBtmThreadHandler = nvt_ts_work_func;
	
	vts_data->icNum = VTS_IC_NT36525;
	vts_data->mtkOrQcom = VTS_PLATFORM_MTK_NEW;

	vts_data->sensorTestKey = "com.nttouchscreen.mptest:MainActivity:android.intent.action.novatek:0:testResult";
	vts_data->lcmNoiseTestKey = "com.nt36xxxtouchscreen.deltadiff:MainActivity:android.intent.action.nvtdeltadiff:0:testResult";
	//vts_data->bspLcmNoiseTestKey = "com.nt36xxxtouchscreen.deltadiff:BspTest:android.intent.action.nvtdeltadiff:0:testResult";
	vts_data->rawdataTestKey = "com.nttouchscreen.getdata:MainActivity:null:null:null";
		
	vts_data->writeImei= vivo_nt_writeUdd;
	vts_data->readImei= vivo_nt_readUdd;
	vts_data->icModeChange = nt_mode_change;
	vts_data->updateFirmware = nt_fw_update;
	vts_data->setGestureSwitch = NULL;		//if all gusture switch default on,this function no use	
	vts_data->getRawOrDiffData = vivo_nt_get_rawordiff_data;
	vts_data->getIcFirmwareOrConfigVersion = nt_get_fw_version;
	vts_data->getModuleId = nt_get_module_id;
	vts_data->getGesturePointData = nt_get_gesture_point;	
	vts_data->setChargerFlagSwitch = vivo_nt_set_charger_bit;	
	vts_data->getTsPinValue = NULL;	
	vts_data->sensorTest = NULL;
	vts_data->setEdgeRestainSwitch = vivo_nt_set_edge_restain_switch;	
	vts_data->getLcmId = nt_get_lcm_id;	
	vts_data->getI2cBusState = get_iic_bus_state;
	vts_data->idleEnableOrDisable = vivo_nt_idle_ctl;
	vts_data->earlySuspendRun = nt_early_suspend_run;
	vts_data->getHeaderFileFWVersionOrConfig = nvt_get_header_fw_version;
	vts_data->sensorTest = bbk_nt_sensor_test;
	vts_data->proFcFilterSwitch = 1;
	vts_data->notifyMutexSwitch = 1;
	vts_data->otherInfo = bbk_nt_get_fw_debug_info;

	ret = vivoTsInit(client, NULL, -1);
	if(ret < 0) {
		VTE("vivoTsInit fail");
		err = -ENOMEM;
		goto free_vivo_ts;
		
	}
	vts_data->resumeEventBlank = FB_EVENT_BLANK;
	vts_data->suspendEventBlank = FB_EVENT_BLANK; 


	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_tm,     sizeof(vts_fw_tm),     "PD1813",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x10);
	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_tm,     sizeof(vts_fw_tm),     "PD1813F_EX",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x10);

	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_boe,     sizeof(vts_fw_boe),     "PD1813",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x30);
	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_boe,     sizeof(vts_fw_boe),     "PD1813F_EX",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x30);

	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_jdi,     sizeof(vts_fw_jdi),     "PD1813",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x40);
	vivoTsFwAdd(VTS_FW_TYPE_FW,     vts_fw_jdi,     sizeof(vts_fw_jdi),     "PD1813F_EX",  VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x40);


	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		VTE("failed to allocated memory for nvt ts data");
		ret = -ENOMEM;
		goto deinit_vivo_ts;
	}
	
	ts->client = client;
	i2c_set_clientdata(client, ts);

	//---parse dts---
	nvt_parse_dt(&client->dev);

	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		VTE("gpio config error!");
		goto err_gpio_config_failed;
	}

	//---check i2c func.---
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VTE("i2c_check_functionality failed. (no I2C_FUNC_I2C)");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		VTE("chip is not identified");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	mutex_init(&ts->lock);
	spin_lock_init(&ts->irq_spinlock);
	ts->irq_enable = 2;

	mutex_lock(&ts->lock);
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);
	nvt_get_fw_info();
	mutex_unlock(&ts->lock);

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	ts->int_trigger_type = INT_TRIGGER_TYPE;

#if WAKEUP_GESTURE
	//wake_lock_init(&gestrue_wakelock, WAKE_LOCK_SUSPEND, "poll-wake-lock");
#endif

	snprintf(ts->phys, 1023, "input/ts");

	//---set int-pin & request irq---
	client->irq = gpio_to_irq(ts->irq_gpio);
	if (client->irq) {
		VTI("int_trigger_type=%d", ts->int_trigger_type);
#if 0
#if WAKEUP_GESTURE
		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type | IRQF_NO_SUSPEND, client->name, ts);
#else
		ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
#endif
#endif
	ret = vivoTsInterruptRegister(client->irq, NULL, nvt_ts_irq_handler, ts->int_trigger_type/* | IRQF_NO_SUSPEND |IRQF_ONESHOT*/, ts);

		if (ret != 0) {
			VTE("request irq failed. ret=%d", ret);
			goto err_int_request_failed;
		} else {
			/*disable_irq(client->irq);*/
			enable_or_disable_irq(0);
			VTI("request irq %d succeed", client->irq);
		}
	}

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	//queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
	Boot_Update_Firmware(NULL);
#endif

#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

	//---set device node---
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nvt_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nvt_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

	bTouchIsAwake_672 = 1;
	VTI("end");

	/*enable_irq(client->irq);*/
	enable_or_disable_irq(1);
	
	vivoTsAfterProbeCompleteCall(client, NULL, -1);

	return 0;

#if (NVT_TOUCH_PROC || NVT_TOUCH_EXT_PROC || NVT_TOUCH_MP)
err_init_NVT_ts:
#endif
	free_irq(client->irq, ts);
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
err_int_request_failed:
	mutex_destroy(&ts->lock);
err_chipvertrim_failed:
err_check_functionality_failed:
	gpio_free(ts->irq_gpio);
err_gpio_config_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ts);
deinit_vivo_ts:
	vivoTsDeInit();
free_vivo_ts:
	vivoTsFree();
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	mutex_destroy(&ts->lock);

	VTI("Removing driver...");

	free_irq(client->irq, ts);

	i2c_set_clientdata(client, NULL);
	kfree(ts);

	vivoTsDeInit();
	vivoTsFree();
		
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(int status)
{
#if 1
	uint8_t buf[4] = {0};

	mutex_lock(&ts->lock);

	VTI("start");

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif
	if(status==TOUCHSCREEN_GESTURE){
		VTI("Write cmd 0x13, wakeup gesture mode");
		bTouchIsAwake_672 = 0;

		//---write i2c command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;

		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);

		/*enable_irq_wake(ts->client->irq);// to do*/

		VTI("Enabled touch wakeup gesture");

	}else if(status==TOUCHSCREEN_SLEEP){
		/*disable_irq_wake(ts->client->irq);//to do*/
		VTI("Write cmd 0x11, deep sleep mode");
		bTouchIsAwake_672 = 0;
		//---write i2c command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
	}

	mdelay(50);
	mutex_unlock(&ts->lock);

	VTI("end");
	return 0;
	#endif
#if 0
	uint8_t buf[4] = {0};

	if (!bTouchIsAwake_672) {
		VTI("Touch is already suspend");
		return 0;
	}

	mutex_lock(&ts->lock);

	VTI("start");

	bTouchIsAwake_672 = 0;

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif
	
#if WAKEUP_GESTURE
	//---write i2c command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
#if 0 // Do not set 0xFF first, ToDo
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 4);
#else
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif

	enable_irq_wake(ts->client->irq);

	VTI("Enabled touch wakeup gesture");

#else // WAKEUP_GESTURE
	disable_irq(ts->client->irq);

	//---write i2c command to enter "deep sleep mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x11;
	CTP_I2C_WRITE(ts->client, I2C_FW_Address, buf, 2);
#endif // WAKEUP_GESTURE

	/* release all touches */
	msleep(50);
	mutex_unlock(&ts->lock);

	VTI("end");

	return 0;
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	VTI("start");
	mutex_lock(&ts->lock);
	if (bTouchIsAwake_672) {
		VTI("Touch is already resume");
		mutex_unlock(&ts->lock);
		return 0;
	}


	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_REK);

#if !WAKEUP_GESTURE
	/*enable_irq(ts->client->irq);*/
	enable_or_disable_irq(1);
#endif

#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

	bTouchIsAwake_672 = 1;
	mutex_unlock(&ts->lock);

	VTI("end");

	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "vivo,ts-spi",},
	{ },
};
#endif
/*
static struct i2c_board_info __initdata nvt_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(NVT_I2C_NAME, I2C_FW_Address),
	},
};
*/

static struct i2c_driver nvt_i2c_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
//	.suspend	= nvt_ts_suspend,
//	.resume		= nvt_ts_resume,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_I2C_NAME,
		.owner	= THIS_MODULE,
#if 0
#ifdef CONFIG_PM
		.pm = &nvt_ts_dev_pm_ops,
#endif
#endif
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
	},
};

static void nvt_add_i2c_driver(void)
{
	int32_t ret = 0;
	ret = i2c_add_driver(&nvt_i2c_driver);
	if (ret) {
		VTE("failed to add i2c driver");
	}
}
/*extern unsigned int is_atboot;
extern unsigned int power_off_charging_mode;*/

extern unsigned int is_atboot;
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	enum boot_mode_t boot_mode;

	if ((0x10 != nt_get_lcm_id()) && (0x30 != nt_get_lcm_id()) && (0x40 != nt_get_lcm_id())) {
		VTI("not Novatek 36672A_1813");
		return 0;
	} else {
		VTI("start Novatek 36672A_1813");
	}
/*
	if(is_atboot==1 || power_off_charging_mode==1) {		
		VTI("TS is in at mood of power off charging mode");		
		return 0;		 	
	}
*/	

	boot_mode = get_boot_mode();

	VIVO_TS_LOG_INF("mode=%d", boot_mode);
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT  ||
		boot_mode == LOW_POWER_OFF_CHARGING_BOOT ||
		boot_mode == META_BOOT ||
		boot_mode == FACTORY_BOOT ||
		boot_mode == ADVMETA_BOOT) {
		VIVO_TS_LOG_INF("in (%d) mode,we do not load driver", boot_mode);
		boot_mode_normal = false;
	} else {
		boot_mode_normal = true;
	}


	if (is_atboot == 1 || !boot_mode_normal) {
		VIVO_TS_LOG_INF("TS is in at mode or power off charging mode.");
		return 0;
	}

	vivo_touchscreen_new_module_init(nvt_add_i2c_driver, "nvt_add_i2c_driver");
	VTI("finished.");

	return 0;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	i2c_del_driver(&nvt_i2c_driver);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq)
		destroy_workqueue(nvt_fwu_wq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif

}

//late_initcall(nvt_driver_init);
module_init(nvt_driver_init);
module_exit(nvt_driver_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
