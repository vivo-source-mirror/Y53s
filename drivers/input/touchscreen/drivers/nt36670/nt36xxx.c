/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 14651 $
 * $Date: 2017 -07 -21 14:58:50 + 0800 (Fri, 21 Jul 2017) $
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "../vts_core.h"

#include "nt36xxx.h"

#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer;
uint8_t esd_check;
uint8_t esd_retry;
uint8_t esd_retry_max = 5;
#endif

#if NVT_TOUCH_EXT_PROC
extern int32_t nt36670_extra_proc_init(void);
#endif

#if NVT_TOUCH_MP
extern int32_t nt36670_mp_proc_init(void);
#endif

struct nvt_ts_data *nt36670_ts;

#if BOOT_UPDATE_FIRMWARE
extern void nt36670_Boot_Update_Firmware(struct work_struct *work);
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
		.mmap = &NT36676F_memory_map, .carrier_system = 0}
};

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

#if WAKEUP_GESTURE
const uint16_t nt36670_gesture_key_array[] = {
	KEY_POWER,  /*GESTURE_WORD_C */
	KEY_POWER,  /*GESTURE_WORD_W */
	KEY_POWER,  /*GESTURE_WORD_V */
	KEY_POWER,  /*GESTURE_DOUBLE_CLICK */
	KEY_POWER,  /*GESTURE_WORD_Z */
	KEY_POWER,  /*GESTURE_WORD_M */
	KEY_POWER,  /*GESTURE_WORD_O */
	KEY_POWER,  /*GESTURE_WORD_e */
	KEY_POWER,  /*GESTURE_WORD_S */
	KEY_POWER,  /*GESTURE_SLIDE_UP */
	KEY_POWER,  /*GESTURE_SLIDE_DOWN */
	KEY_POWER,  /*GESTURE_SLIDE_LEFT */
	KEY_POWER,  /*GESTURE_SLIDE_RIGHT */
};
#endif

uint8_t nt36670_bTouchIsAwake_672;

/*******************************************************
Description:
	Novatek touchscreen i2c read function.

return:
	Executive outcomes. 2-- - succeed. -5-- - I / O error
 *******************************************************/
int32_t NT36670_CTP_I2C_READ(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msgs[2];
	int32_t ret = -1;
	int32_t retries = 0;
	struct nvt_ts_data *nt36670 = i2c_get_clientdata(client);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len - 1;
	msgs[1].buf   = &buf[1];

	while (retries < 5) {
		mutex_lock(&nt36670->i2c_access_lock);
		ret = i2c_transfer(client->adapter, msgs, 2);
		mutex_unlock(&nt36670->i2c_access_lock);
		if (ret == 2)
			break;
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
	Novatek touchscreen i2c dummy read function.

return:
	Executive outcomes. 1-- - succeed. -5-- - I / O error
 *******************************************************/
int32_t NT36670_CTP_I2C_READ_DUMMY(struct i2c_client *client, uint16_t address)
{
	uint8_t buf[8] = {0};
	int32_t ret = -1;

	ret = NT36670_CTP_I2C_READ(client, address, buf, 2);
	if (ret < 0)
		NVT_ERR("NT36670_CTP_I2C_READ_DUMMY failed.(%d)\n", ret);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen i2c write function.

return:
	Executive outcomes. 1-- - succeed. -5-- - I / O error
 *******************************************************/
int32_t NT36670_CTP_I2C_WRITE(struct i2c_client *client, uint16_t address, uint8_t *buf, uint16_t len)
{
	struct i2c_msg msg;
	int32_t ret = -1;
	int32_t retries = 0;
	struct nvt_ts_data *nt36670 = i2c_get_clientdata(client);

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 5) {
		mutex_lock(&nt36670->i2c_access_lock);
		ret = i2c_transfer(client->adapter, &msg, 1);
		mutex_unlock(&nt36670->i2c_access_lock);
		if (ret == 1)
			break;
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
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
 *******************************************************/
void nt36670_sw_reset_idle(void)
{
	uint8_t buf[4] = {0};

	/*-- - write i2c cmds to reset idle-- - */
	buf[0] = 0x00;
	buf[1] = 0xA5;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
 *******************************************************/
void nt36670_bootloader_reset(void)
{
	uint8_t buf[8] = {0};

	/*-- - write i2c cmds to reset-- - */
	buf[0] = 0x00;
	buf[1] = 0x69;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);

	/* need 35ms delay after bootloader reset */
	msleep(35);
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0-- - succeed. -1-- - fail.
 *******************************************************/
int32_t nt36670_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;
	int ret = 0;

	for (i = 0; i < retry; i++) {
		/*-- - set xdata index to EVENT BUF ADDR-- - */
		buf[0] = 0xFF;
		buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
		buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

		/*-- - clear fw status-- - */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);

		/*-- - read fw status-- - */
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 2);

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
	Executive outcomes. 0-- - succeed. -1-- - failed.
 *******************************************************/
int32_t nt36670_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;
	int ret = 0;

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
	Executive outcomes. 0-- - succeed. -1-- - failed.
 *******************************************************/
int32_t nt36670_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	while (1) {
		msleep(10);

		/*-- - read reset state-- - */
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			NVT_LOG("check_reset_state=%04X\n", buf[1]);  /*[20180128]*/
			ret = 0;
			break;
		}

		retry++;
		if (unlikely(retry > 100)) {
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
	Executive outcomes. 0-- - success. -1-- - fail.
 *******************************************************/
int32_t nt36670_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - read project id-- - */
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 3);

	nt36670_ts->nvt_pid = (buf[2] << 8) + buf[1];
	/*snprintf(nt36670_ts->nvt_pid, sizeof(nt36670_ts->nvt_pid), "%02X%02X", buf[2], buf[1]); */

	NVT_LOG("PID=%04X\n", nt36670_ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0-- - success. -1-- - fail.
 *******************************************************/
int32_t nt36670_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	/*-- - set xdata index to EVENT BUF ADDR-- - */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 3);

	/*-- - read fw info-- - */
	buf[0] = EVENT_MAP_FWINFO;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_FW_Address, buf, 17);
	nt36670_ts->fw_ver = buf[1];
	nt36670_ts->x_num = buf[3];
	nt36670_ts->y_num = buf[4];
	nt36670_ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	nt36670_ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);
	nt36670_ts->max_button_num = buf[11];

	/*-- - clear x_num, y_num if fw info is broken-- - */
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		nt36670_ts->fw_ver = 0;
		nt36670_ts->x_num = 18;
		nt36670_ts->y_num = 32;
		nt36670_ts->abs_x_max = 1080;
		nt36670_ts->abs_y_max = 2280;
		nt36670_ts->max_button_num = 0;

		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=0, x_num=18, y_num=32, abs_x_max=1080, abs_y_max=1920, max_button_num=0!\n");
			ret = -1;
		}
	} else {
		ret = 0;
	}

	/*-- - Get Novatek PID-- - */
	nt36670_read_pid();

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
	Novatek touchscreen / proc / NVTflash read function.

return:
	Executive outcomes. 2-- - succeed. -5, -14-- - failed.
 *******************************************************/
static ssize_t nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t str[68] = {0};
	int32_t ret = -1;
	int32_t retries = 0;
	int8_t i2c_wr = 0;

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

	if (i2c_wr == 0) {	/*I2C write */
		while (retries < 20) {
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, (str[0] & 0x7F), &str[2], str[1]);
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
	} else if (i2c_wr == 1) {	/*I2C read */
		while (retries < 20) {
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, (str[0] & 0x7F), &str[2], str[1]);
			if (ret == 2)
				break;
			else
				NVT_ERR("error, retries=%d, ret=%d\n", retries, ret);

			retries++;
		}

		/* copy buff to user if i2c transfer */
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
}

#if (NVTFLASH_WORK_PROTECT)
	atomic_t u8_NT36670_flashWorking = ATOMIC_INIT(0);
#endif
/*******************************************************
Description:
	Novatek touchscreen / proc / NVTflash open function.

return:
	Executive outcomes. 0-- - succeed. -12-- - failed.
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
		atomic_set(&u8_NT36670_flashWorking, 1);
		NVT_ERR("u8_NT36670_flashWorking=[%d]\n", atomic_read(&u8_NT36670_flashWorking));
	#endif

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen / proc / NVTflash close function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	#if (NVTFLASH_WORK_PROTECT)
		atomic_set(&u8_NT36670_flashWorking, 0);
		NVT_ERR("u8_NT36670_flashWorking=[%d]\n", atomic_read(&u8_NT36670_flashWorking));
	#endif

	if (dev)
		kfree(dev);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.read = nvt_flash_read,
};
#else
static const struct proc_ops nvt_flash_fops = {
	.proc_open = nvt_flash_open,
	.proc_release = nvt_flash_close,
	.proc_read = nvt_flash_read,
};
#endif

/*******************************************************
Description:
	Novatek touchscreen / proc / NVTflash initial function.

return:
	Executive outcomes. 0-- - succeed. -12-- - failed.
 *******************************************************/
static int32_t nvt_flash_proc_init(void)
{
	NVT_proc_entry = proc_create(DEVICE_NAME, 0444, NULL, &nvt_flash_fops);
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
/*static struct wake_lock gestrue_wakelock; */

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
 *******************************************************/
static void nvt_ts_wakeup_gesture_report_36670(struct nvt_ts_data *nt36670, uint8_t gesture_id, uint8_t point_data[64 + 2])
{
	int i = 0;
	uint16_t u16aX[9];
	uint16_t u16aY[9];
	struct vts_device *vtsdev = nt36670->vtsdev;

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
	case GESTURE_WORD_C:
		vts_dev_info(vtsdev, "Gesture : Word-C.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
		break;
	case GESTURE_WORD_W:
		vts_dev_info(vtsdev, "Gesture : Word-W.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
		break;
	case GESTURE_WORD_V:
		vts_dev_info(vtsdev, "Gesture : Word-V.");
		break;
	case GESTURE_DOUBLE_CLICK:
		vts_dev_info(vtsdev, "Gesture : Double Click.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
		break;
	case GESTURE_WORD_Z:
		vts_dev_info(vtsdev, "Gesture : Word-Z.");
		break;
	case GESTURE_WORD_M:
		vts_dev_info(vtsdev, "Gesture : Word-M.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
		break;
	case GESTURE_WORD_O:
		vts_dev_info(vtsdev, "Gesture : Word-O.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
		break;
	case GESTURE_WORD_e:
		vts_dev_info(vtsdev, "Gesture : Word-e.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
		break;
	case GESTURE_WORD_S:
		vts_dev_info(vtsdev, "Gesture : Word-S.");
		break;
	case GESTURE_SLIDE_UP:
		vts_dev_info(vtsdev, "Gesture : Slide UP.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
		break;
	case GESTURE_SLIDE_DOWN:
		vts_dev_info(vtsdev, "Gesture : Slide DOWN.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
		break;
	case GESTURE_SLIDE_LEFT:
		vts_dev_info(vtsdev, "Gesture : Slide LEFT.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
		break;
	case GESTURE_SLIDE_RIGHT:
		vts_dev_info(vtsdev, "Gesture : Slide RIGHT.");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
		break;
	case GESTURE_EXT_WORD_AT:
		vts_dev_info(vtsdev, "Gesture : A");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
		break;
	case GESTURE_EXT_WORD_F:
		vts_dev_info(vtsdev, "Gesture : F");
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
		break;
	default:
		break;
	}

	if ((gesture_id != GESTURE_DOUBLE_CLICK && gesture_id >= 12 && gesture_id <= 26 && gesture_id != 22 && gesture_id != 23 && gesture_id != 24) || (gesture_id == 30) || (gesture_id == 31)) {
		for (i = 0; i < 9; i++) {
			u16aX[i] = (point_data[4 * i + 5] << 8) + point_data[4 * i + 4];
			u16aY[i] = (point_data[4 * i + 7] << 8) + point_data[4 * i + 6];
		}
		vts_report_coordinates_set(vtsdev, u16aX, u16aX, 9);
		vts_dev_info(vtsdev, "input_id=[%02d],OClockwise[%02X]", point_data[3], point_data[43]);
		for (i = 0; i < 9; i++) {
			vts_dev_info(vtsdev, "%d(%02d,%02d)", i, u16aX[i], u16aY[i]);
		}
	}
}
#endif

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	if (enable) {
		/* update interrupt timer */
		irq_timer = jiffies;
	}
	/* enable / disable esd check flag */
	esd_check = enable;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
}

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i = 1; i < 7; i++) {
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

	NVT_ERR("esd_check = %d (retry %d/%d)\n", esd_check, esd_retry, esd_retry_max);	/*DEBUG */

	if (esd_retry >= esd_retry_max)
		nvt_esd_check_enable(false);

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, bootloader reset */
		nt36670_bootloader_reset();
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
static int last_press_id[TOUCH_MAX_FINGER_NUM] = {0};
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
 *******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *handle, ktime_t kt)
{
	int32_t ret = -1;
	uint8_t point_data[POINT_DATA_LEN + 1] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	bool large_press = false;
	struct nvt_ts_data *nt36670 = (struct nvt_ts_data *)handle;
	struct vts_device *vtsdev = nt36670->vtsdev;

	mutex_lock(&nt36670->lock);

	ret = NT36670_CTP_I2C_READ(nt36670->client, I2C_FW_Address, point_data, POINT_DATA_LEN + 1);
	if (ret < 0) {
		NVT_ERR("NT36670_CTP_I2C_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}
/*
	//-- - dump I2C buf -- -
	for (i = 0; i < 10; i++) {
		printk("%02X %02X %02X %02X %02X %02X  ", point_data[1 + i * 6], point_data[2 + i * 6], point_data[3 + i * 6], point_data[4 + i * 6], point_data[5 + i * 6], point_data[6 + i * 6]);
	}
	printk("\n");
 */

#if NVT_TOUCH_ESD_PROTECT
	/*NVT_ERR("%02X %02X %02X\n", point_data[1], point_data[2], point_data[3]);	//DEBUG */
	if (nvt_fw_recovery(point_data)) {
		nvt_esd_check_enable(true);
		goto XFER_ERROR;
	}
#endif

	if ((point_data[1] == 0xF0) && (point_data[2] == 0x04) && (point_data[3] == 0x01)) {
		vts_dev_info(vtsdev, "got PalmOn");
		large_press = true;
	} else if ((point_data[1] == 0xF0) && (point_data[2] == 0x04) && (point_data[3] == 0x02)) {
		printk("[NVT-ts] got PalmOFF\n");
		large_press = false;
	}

#if WAKEUP_GESTURE
	if (nt36670_bTouchIsAwake_672 == 0) {
		input_id = (uint8_t)(point_data[1] >> 3);
		NVT_LOG("[NVT-ts] %02X %02X %02X\n", point_data[1], point_data[2], point_data[3]);	/*question */
		if ((point_data[1] == 0xF0) && (point_data[2] == 0x01)) {	/* judge f and @ gesture */
			input_id = point_data[3];	
		} else if (input_id > 30) {
			vts_dev_err(nt36670_ts->vtsdev, "[NVT-ts] %02X %02X %02X\n", point_data[1], point_data[2], point_data[3]);	/*question */
			mutex_unlock(&nt36670->lock);
			return IRQ_HANDLED;
		}
		nvt_ts_wakeup_gesture_report_36670(nt36670, input_id, point_data);
		mutex_unlock(&nt36670->lock);
		return IRQ_HANDLED;
	}
#endif

	for (i = 0; i < nt36670->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id > nt36670->max_touch_num) || (input_id <= 0)) {
			continue;
		}

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	/*finger down (enter & moving) */
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			if (esd_check) {
				irq_timer = jiffies;
			}
#endif
			input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);
			if ((input_x < 0) || (input_y < 0))
				continue;
			if ((input_x > nt36670->abs_x_max) || (input_y > nt36670->abs_y_max))
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
			vts_report_point_down(vtsdev, input_id - 1, ++finger_cnt, input_x, input_y,
				(input_x % 10 + input_y % 10 + 4),(input_x % 10 + input_y % 10 + 4), large_press, NULL, 0, kt);
		}
	}

	if (finger_cnt == 0) {
		vts_report_release(vtsdev);
	} else {
		for (i = 0; i < nt36670->max_touch_num; i++) {
			if (last_press_id[i] == 1 && press_id[i] == 0) {
				/*up */
				vts_report_point_up(vtsdev, i, 0, input_x, input_y, input_p, input_p, large_press, kt);
			}
		}
	}

XFER_ERROR:
	mutex_unlock(&nt36670->lock);

	for (i = 0; i < nt36670->max_touch_num; i++) {
		last_press_id[i] = press_id[i];
	}

	vts_report_point_sync(vtsdev);
	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Novatek touchscreen check and stop crc reboot loop.

return:
	n.a.
 *******************************************************/
void nt36670_stop_crc_reboot(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;

	/*read dummy buffer to check CRC fail reboot is happening or not */

	/*-- - change I2C index to prevent geting 0xFF, but not 0xFC-- - */
	buf[0] = 0xFF;
	buf[1] = 0x01;
	buf[2] = 0xF6;
	NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);

	/*-- - read to check if buf is 0xFC which means IC is in CRC reboot -- - */
	buf[0] = 0x4E;
	NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 4);

	if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
		((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {

		/*IC is in CRC fail reboot loop, needs to be stopped! */
		for (retry = 5; retry > 0; retry--) {

			/*-- - write i2c cmds to reset idle : 1st-- - */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);

			/*-- - write i2c cmds to reset idle : 2rd-- - */
			buf[0] = 0x00;
			buf[1] = 0xA5;
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
			msleep(1);

			/*-- - clear CRC_ERR_FLAG-- - */
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0xA5;
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 2);

			/*-- - check CRC_ERR_FLAG-- - */
			buf[0] = 0xFF;
			buf[1] = 0x03;
			buf[2] = 0xF1;
			NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);

			buf[0] = 0x35;
			buf[1] = 0x00;
			NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 2);

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
	Executive outcomes. 0-- - NVT IC. -1-- - not NVT IC.
 *******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	nt36670_bootloader_reset(); /*NOT in retry loop */
	/*-- - Check for 5 times-- - */
	for (retry = 5; retry > 0; retry--) {
		/*nt36670_bootloader_reset(); */
		nt36670_sw_reset_idle();

		buf[0] = 0x00;
		buf[1] = 0x35;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
		msleep(10);

		buf[0] = 0xFF;
		buf[1] = 0x01;
		buf[2] = 0xF6;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		/* compare read chip id on supported list */
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
				nt36670_ts->mmap = trim_id_table[list].mmap;
				nt36670_ts->carrier_system = trim_id_table[list].carrier_system;
				ret = 0;
				goto out;
			} else {
				nt36670_ts->mmap = NULL;
				ret = -1;
			}
		}
		/*-- - Stop CRC check to prevent IC auto reboot-- - */
		if (((buf[1] == 0xFC) && (buf[2] == 0xFC) && (buf[3] == 0xFC)) ||
			((buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF))) {
			nt36670_stop_crc_reboot();
		}

		msleep(10);
	}

out:
	return ret;
}

extern int nt36670_get_ic_fw_version(void);

static int nt36670_fw_version(struct vts_device *vtsdev, u64 *version)
{
	*version = (u64)nt36670_get_ic_fw_version();
	return 0;
}

static int last_ts_state;
static int32_t nvt_ts_resume(struct device *dev);
static int32_t nvt_ts_suspend(int status);
extern int8_t nt36670_nvt_customizeCmd_WaitSet(uint8_t u8WaitAddr, uint8_t u8WaitStatus, uint8_t u8Cmd);
#ifdef	FM_NOTIFY
extern int nt36670_FM_state_rewrite(void);
#endif
/**
 * zhj add for unbalanced irq when mode change  
 * bbk_slsi_irq_enable - Enable/Disable a irq
 * @ts: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
static int bbk_nt_irq_enable(struct nvt_ts_data *nt36670_ts, bool enable)
{
	vts_dev_info(nt36670_ts->vtsdev, "****bbk_nt_irq_enable**** %d  atomic: %d", enable, atomic_read(&nt36670_ts->irq_enabled));
	if (enable) {
		if (!atomic_cmpxchg(&nt36670_ts->irq_enabled, 0, 1)) {
			enable_irq(nt36670_ts->client->irq);
			vts_dev_info(nt36670_ts->vtsdev, "===mode_change=== Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&nt36670_ts->irq_enabled, 1, 0)) {
			disable_irq(nt36670_ts->client->irq);
			vts_dev_info(nt36670_ts->vtsdev, "+++mode_change+++ Irq disabled");
		}
	}

	return 0;
}

static int nt36670_mode_change(struct vts_device *vtsdev, int which)
{
	#if (NVTFLASH_WORK_PROTECT)
		atomic_set(&u8_NT36670_flashWorking, 0);
		NVT_ERR("u8_NT36670_flashWorking=[%d]\n", atomic_read(&u8_NT36670_flashWorking));
	#endif
	if (which == VTS_ST_NORMAL) {
		vts_dev_info(vtsdev, "change to normal mode");
		/*normal power on set by lcm, tp not contronl */
		nvt_ts_resume(&nt36670_ts->client->dev);
#ifdef	FM_NOTIFY
		if (nt36670_ts->fm_switch) {
			nt36670_FM_state_rewrite();
		}
#endif
		bbk_nt_irq_enable(nt36670_ts, true);
	}

	if (which == VTS_ST_GESTURE) {
		vts_dev_info(vtsdev, "change to gesture mode");

		if (last_ts_state == VTS_ST_SLEEP) {
			mutex_lock(&nt36670_ts->i2c_access_lock);
			vts_dsi_panel_reset_power_ctrl(1);
			mdelay(50);
			mutex_unlock(&nt36670_ts->i2c_access_lock);
		}

		if (VTS_ST_SLEEP == last_ts_state) {
			/*nvt_ts_suspend(VTS_ST_GESTURE);*/
			mdelay(40);
			vts_dev_info(vtsdev, "JUST FOR DEBUG");
			nt36670_nvt_customizeCmd_WaitSet(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT, 0x13);
		}
		bbk_nt_irq_enable(nt36670_ts, true);
	}
	if (which == VTS_ST_SLEEP) {
		vts_dev_info(vtsdev, "change to sleep mode");
		/*nvt_ts_suspend(VTS_ST_SLEEP); */

		/*power down */
		vts_dsi_panel_reset_power_ctrl(0);
		bbk_nt_irq_enable(nt36670_ts, false);
	}

	last_ts_state = which;

	return 0;
}

static int nt36670_early_suspend_run(struct vts_device *vtsdev)
{
	vts_dev_info(vtsdev, "early suspend run,nt set 13 cmd");
	nvt_ts_suspend(VTS_ST_GESTURE);
	return 0;
}

extern int nt36670_tpd_usb_plugin(int plugin);

extern int nt36670_set_charging(struct vts_device *vtsdev, int state);
extern int nt36670_hand_update_firmware(const struct firmware *firmware);
static int nt36670_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)
{
	return nt36670_hand_update_firmware(firmware);
}

static int nt36670_rom_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 19;
	return 0;
}

static int nt36670_hw_init(struct vts_device *vtsdev)
{
	int ret = 0;

	/* need 10ms delay after POR(power on reset) */
	msleep(10);

	/*-- - check chip version trim-- - */
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		vts_dev_err(nt36670_ts->vtsdev, "chip is not identified");
		return -EINVAL;
	}

	mutex_lock(&nt36670_ts->lock);
	nt36670_bootloader_reset();
	nt36670_check_fw_reset_state(RESET_STATE_INIT);
	nt36670_get_fw_info();
	mutex_unlock(&nt36670_ts->lock);

	ret = vts_interrupt_register(vtsdev, nt36670_ts->client->irq, nvt_ts_work_func, IRQ_TYPE_EDGE_RISING, nt36670_ts);
	if (ret) {
		vts_dev_err(nt36670_ts->vtsdev, "request irq failed. ret=%d", ret);
		return ret;
	}

	disable_irq(nt36670_ts->client->irq);
	atomic_set(&nt36670_ts->irq_enabled, 1);
	vts_dev_info(vtsdev, "request irq %d succeed", nt36670_ts->client->irq);

#if BOOT_UPDATE_FIRMWARE
	/* please make sure boot update start after display reset(RESX) sequence */
	nt36670_Boot_Update_Firmware(NULL);
#endif
	nt36670_bTouchIsAwake_672 = 1;
	enable_irq(nt36670_ts->client->irq);
	return 0;
}

extern int32_t nt36670_read_projectid(uint8_t *u8Am_IDread);
extern ssize_t nt36670_rom_read(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
extern int32_t nt36670_write_projectid(uint8_t *u8Am_IDdata);
extern ssize_t nt36670_rom_write(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
#ifdef FM_NOTIFY
extern int nt36670_process_package(struct vts_device *vtsdev, unsigned char *package_name);
#endif
extern int nt36670_get_frame(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size);
extern int nt36670_at_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result);
extern int nt36670_set_rotation(struct vts_device *vtsdev, int on);
extern int nt36670_set_idle(struct vts_device *vtsdev, int state);
static const struct vts_operations nt36670_vts_ops = {
	.init = nt36670_hw_init,
	.set_charging = nt36670_set_charging,
	.set_rotation = nt36670_set_rotation,
	.update_firmware = nt36670_fw_update,
	.change_mode = nt36670_mode_change,
	.get_fw_version = nt36670_fw_version,
	.set_auto_idle = nt36670_set_idle,
	.sensor_test = nt36670_at_sensor_test,
	.early_suspend = nt36670_early_suspend_run,
	.get_frame = nt36670_get_frame,
	.rom_size = nt36670_rom_size,
	.rom_read = nt36670_rom_read,
	.rom_write = nt36670_rom_write,
#ifdef FM_NOTIFY
	.process_package = nt36670_process_package,
#endif
};

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed
 *******************************************************/
static int32_t nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
	struct vts_device *vtsdev;
	client->addr = 0x62;

	VTI("start");
	nt36670_ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (nt36670_ts == NULL) {
		vts_dev_err(nt36670_ts->vtsdev, "failed to allocated memory for nvt ts data");
		return -ENOMEM;
	}

	nt36670_ts->irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "novatek,irq-gpio", 0, &nt36670_ts->irq_flags);
	if (nt36670_ts->irq_gpio < 0) {
		vts_dev_err(nt36670_ts->vtsdev, "parse irq gpio failed! ret = %d\n", nt36670_ts->irq_gpio);
		goto parse_irq_gpio_err;
	}
	NVT_LOG("novatek,irq-gpio=%d\n", nt36670_ts->irq_gpio);

	ret = gpio_request_one(nt36670_ts->irq_gpio, GPIOF_IN, "NVT-int");
	if (ret) {
		NVT_ERR("Failed to request NVT-int GPIO\n");
		goto err_request_irq_gpio;
	}

	client->irq = gpio_to_irq(nt36670_ts->irq_gpio);
	snprintf(nt36670_ts->phys, 1023, "input/ts");
	nt36670_ts->client = client;
	i2c_set_clientdata(client, nt36670_ts);
	mutex_init(&nt36670_ts->lock);
	mutex_init(&nt36670_ts->i2c_access_lock);
	nt36670_ts->max_touch_num = TOUCH_MAX_FINGER_NUM;
#if TOUCH_KEY_NUM > 0
	nt36670_ts->max_button_num = TOUCH_KEY_NUM;
#endif
#if WAKEUP_GESTURE
#endif

#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = create_workqueue("nvt_esd_check_wq");
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

	/*-- - set device node-- - */
#if NVT_TOUCH_PROC
	ret = nvt_flash_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt flash proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_EXT_PROC
	ret = nt36670_extra_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt extra proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

#if NVT_TOUCH_MP
	ret = nt36670_mp_proc_init();
	if (ret != 0) {
		NVT_ERR("nvt mp proc init failed. ret=%d\n", ret);
		goto err_init_NVT_ts;
	}
#endif

	vtsdev = vts_device_alloc();
	if (vtsdev == NULL) {
		vts_dev_err(nt36670_ts->vtsdev, "vtsdev alloc fail");
		ret = -ENOMEM;
		goto err_init_NVT_ts;
	}

	nt36670_ts->vtsdev = vtsdev;
	vtsdev->ops = &nt36670_vts_ops;
	vtsdev->busType = BUS_I2C;
	ret = vts_parse_dt_property(vtsdev, client->dev.of_node);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto register_vts_err;
	}
	vts_set_drvdata(vtsdev, nt36670_ts);
	ret = vts_register_driver(vtsdev);
	if (ret) {
		NVT_ERR("register vts driver failed!, ret = %d\n", ret);
		goto register_vts_err;
	}

	vts_dev_info(vtsdev, "end");
	return 0;

register_vts_err:
	vts_device_free(vtsdev);
err_init_NVT_ts:
	mutex_destroy(&nt36670_ts->i2c_access_lock);
	mutex_destroy(&nt36670_ts->lock);
	i2c_set_clientdata(client, NULL);
	gpio_free(nt36670_ts->irq_gpio);
err_request_irq_gpio:
parse_irq_gpio_err:
	kfree(nt36670_ts);
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_ts_remove(struct i2c_client *client)
{
	vts_dev_info(nt36670_ts->vtsdev, "Removing driver...");
	vts_unregister_driver(nt36670_ts->vtsdev);
	vts_device_free(nt36670_ts->vtsdev);
	mutex_destroy(&nt36670_ts->i2c_access_lock);
	mutex_destroy(&nt36670_ts->lock);
	i2c_set_clientdata(client, NULL);
	gpio_free(nt36670_ts->irq_gpio);
	kfree(nt36670_ts);
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq)
		destroy_workqueue(nvt_esd_check_wq);
#endif
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_ts_suspend(int status)
{
#if 1
	uint8_t buf[4] = {0};

	mutex_lock(&nt36670_ts->lock);

	vts_dev_info(nt36670_ts->vtsdev, "start");

#if NVT_TOUCH_ESD_PROTECT
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif
	if (status == VTS_ST_GESTURE) {

		nt36670_bTouchIsAwake_672 = 0;

		/*-- - write i2c command to enter "wakeup gesture mode"-- - */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;

		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);

		enable_irq_wake(nt36670_ts->client->irq);/* to do */

		vts_dev_info(nt36670_ts->vtsdev, "Enabled touch wakeup gesture");

	} else if (status == VTS_ST_SLEEP) {
		disable_irq_wake(nt36670_ts->client->irq);/*to do */

		/*-- - write i2c command to enter "deep sleep mode"-- - */
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_FW_Address, buf, 2);
	}

	mdelay(50);
	mutex_unlock(&nt36670_ts->lock);

	vts_dev_info(nt36670_ts->vtsdev, "end");
	return 0;
	#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0-- - succeed.
 *******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{

	vts_dev_info(nt36670_ts->vtsdev,"start");
	mutex_lock(&nt36670_ts->lock);
	if (nt36670_bTouchIsAwake_672) {
		vts_dev_info(nt36670_ts->vtsdev, "Touch is already resume");
		mutex_unlock(&nt36670_ts->lock);
		return 0;
	}

	/* please make sure display reset(RESX) sequence and mipi dsi cmds sent before this */
	nt36670_bootloader_reset();
	nt36670_check_fw_reset_state(RESET_STATE_REK);

#if !WAKEUP_GESTURE
	enable_irq(nt36670_ts->client->irq);
#endif

#if NVT_TOUCH_ESD_PROTECT
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

	nt36670_bTouchIsAwake_672 = 1;
	mutex_unlock(&nt36670_ts->lock);

	vts_dev_info(nt36670_ts->vtsdev,"end");

	return 0;
}

static const struct i2c_device_id nvt_ts_id[] = {
	{ NVT_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-36670", },
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
	.probe		 = nvt_ts_probe,
	.remove		 = nvt_ts_remove,
/*	.suspend	 = nvt_ts_suspend, */
/*	.resume		 = nvt_ts_resume, */
	.id_table	 = nvt_ts_id,
	.driver = {
		.name	 = NVT_I2C_NAME,
		.owner	 = THIS_MODULE,
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

static const int ic_numbers[] = {VTS_IC_NT36672, VTS_IC_NT36670};
module_vts_driver(nt_i2c, ic_numbers, i2c_add_driver(&nvt_i2c_driver), i2c_del_driver(&nvt_i2c_driver));

