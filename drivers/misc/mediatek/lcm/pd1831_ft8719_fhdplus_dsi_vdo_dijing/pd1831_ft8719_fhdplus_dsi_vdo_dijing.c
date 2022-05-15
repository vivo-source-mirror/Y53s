/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define tp_reset_setting(cmd) \
		lcm_util.tp_reset_setting(cmd)
#define lcm_reset_setting(cmd) \
			lcm_util.lcm_reset_setting(cmd)
#define lcm_enp_setting(cmd) \
			lcm_util.lcm_enp_setting(cmd)
#define lcm_enn_setting(cmd) \
			lcm_util.lcm_enn_setting(cmd)
#define lcm_bkg_setting(cmd) \
			lcm_util.lcm_bkg_setting(cmd)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2340)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(69500)
#define LCM_PHYSICAL_HEIGHT									(150580)
#define LCM_DENSITY											(480)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static struct LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;

extern unsigned int lcm_software_id;

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0x55, 1, {0x01} },
};
static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0x55, 1, {0x03} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0x55, 1, {0x00} },
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 50, {} },
	{0x00, 1, {0x00} },
	{0xF7, 4, {0x5A, 0xA5, 0x95, 0x27} },
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0x00, 1, {0x00}},
	{0xFF, 3, {0x87,0x19,0x01}},
	{0x00, 1, {0x80}},
	{0xFF, 2, {0x87,0x19}},
	{0x00, 1, {0x82}},
	{0xA4, 2, {0x29,0x23}},
	{0x00, 1, {0x80}},
	{0xA7, 1, {0x03}},
	{0x00, 1, {0x82}},
	{0xA7, 2, {0x11,0x01}},
	{0x00, 1, {0x90}},
	{0xC3, 4, {0x0C,0x00,0x00,0x01}},
	{0x00, 1, {0xA0}},
	{0xC3, 7, {0x31,0x21,0x02,0x10,0x01,0x20,0x12}},
	{0x00, 1, {0xB3}},
	{0xC5, 1, {0x08}},
	{0x00, 1, {0x80}},
	{0xC2, 4, {0x82,0x01,0x20,0x56}},
	{0x00, 1, {0xA0}},
	{0xC2, 15, {0x00,0x00,0x00,0x24,0x98,0x01,0x00,0x00,0x24,0x98,0x02,0x00,0x00,0x24,0x98}},
	{0x00, 1, {0xB0}},
	{0xC2, 10, {0x03,0x00,0x00,0x24,0x98,0x00,0x02,0x03,0x00,0x80}},
	{0x00, 1, {0xE0}},
	{0xC2, 8, {0x33,0x33,0x73,0x33,0x33,0x33,0x00,0x00}},
	{0x00, 1, {0xFA}},
	{0xC2, 3, {0x23,0xFF,0x23}},
	{0x00, 1, {0x80}},
	{0xCB, 16 ,{0x00,0x01,0x00,0x00,0xFD,0x01,0x00,0x00,0x00,0x00,0xFD,0x01,0x00,0x01,0x00,0x03}},
	{0x00, 1, {0x90}},
	{0xCB, 16, {0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}},
	{0x00, 1, {0xA0}},
	{0xCB, 4, {0x00,0x00,0x00,0x00}},
	{0x00, 1, {0xB0}},
	{0xCB, 4, {0x55,0x55,0x55,0x57}},
	{0x00, 1, {0x80}},
	{0xCC, 16, {0x00,0x29,0x00,0x23,0x00,0x0A,0x00,0x00,0x09,0x08,0x07,0x06,0x00,0x00,0x00,0x00}},
	{0x00, 1, {0x90}},
	{0xCC, 8, {0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00, 1, {0x80}},
	{0xCD, 16, {0x00,0x00,0x00,0x02,0x00,0x0A,0x00,0x00,0x09,0x08,0x07,0x06,0x00,0x00,0x00,0x00}},
	{0x00, 1, {0x90}},
	{0xCD, 8, {0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00, 1, {0xA0}},
	{0xCC, 16, {0x00,0x29,0x00,0x23,0x00,0x0A,0x00,0x00,0x06,0x07,0x08,0x09,0x00,0x00,0x00,0x00}},
	{0x00, 1, {0xB0}},
	{0xCC, 8, {0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00, 1, {0xA0}},
	{0xCD, 16, {0x00,0x00,0x00,0x02,0x00,0x0A,0x00,0x00,0x06,0x07,0x08,0x09,0x00,0x00,0x00,0x00}},
	{0x00, 1, {0xB0}},
	{0xCD, 8, {0x00,0x18,0x16,0x17,0x00,0x1C,0x1D,0x1E}},
	{0x00, 1, {0x80}},
	{0xC0, 6, {0x00,0x7A,0x00,0x6C,0x00,0x10}},
	{0x00, 1, {0x89}},
	{0xC0, 3, {0x01,0x1D,0x04}},
	{0x00, 1, {0xA0}},
	{0xC0, 6, {0x01,0x09,0x00,0x3A,0x00,0x10}},
	{0x00, 1, {0xB0}},
	{0xC0, 5, {0x00,0x7A,0x02,0x10,0x10}},
	{0x00, 1, {0xC1}},
	{0xC0, 8, {0x00,0xB1,0x00,0x8B,0x00,0x76,0x00,0xD0}},
	{0x00, 1, {0xCA}},
	{0xC0, 1, {0x80}},
	{0x00, 1, {0xD7}},
	{0xC0, 6, {0x00,0x76,0x00,0x6F,0x00,0x10}},
	{0x00, 1, {0xA5}},
	{0xC1, 4, {0x00,0x36,0x00,0x02}},
	{0x00, 1, {0x82}},
	{0xCE, 13, {0x01,0x09,0x00,0xD8,0x00,0xD8,0x00,0x90,0x00,0x90,0x0D,0x0E,0x09}},
	{0x00, 1, {0x90}},
	{0xCE, 8, {0x00,0x82,0x0D,0x5C,0x00,0x82,0x80,0x09}},
	{0x00, 1, {0xA0}},
	{0xCE, 3, {0x00,0x00,0x00}},
	{0x00, 1, {0xB0}},
	{0xCE, 3, {0x11,0x00,0x00}},
	{0x00, 1, {0xD1}},
	{0xCE, 7, {0x00,0x0A,0x01,0x01,0x00,0x5D,0x01}},
	{0x00, 1, {0xE1}},
	{0xCE, 11, {0x08,0x02,0x15,0x02,0x15,0x02,0x15,0x00,0x2B,0x00,0x60}},
	{0x00, 1, {0xF1}},
	{0xCE, 9, {0x16,0x0B,0x0F,0x01,0x12,0x01,0x11,0x01,0x23}},
	{0x00, 1, {0xB0}},
	{0xCF, 4, {0x00,0x00,0x6C,0x70}},
	{0x00, 1, {0xB5}},
	{0xCF, 4, {0x04,0x04,0xA4,0xA8}},
	{0x00, 1, {0xC0}},
	{0xCF, 4, {0x08,0x08,0xCA,0xCE}},
	{0x00, 1, {0xC5}},
	{0xCF, 4, {0x00,0x00,0x08,0x0C}},
	{0x00, 1, {0x90}},
	{0xC0, 6, {0x00,0x7A,0x00,0x6C,0x00,0x10}},
	{0x00, 1, {0xA1}},
	{0xB3, 6, {0x04,0x38,0x09,0x24,0xC0,0xF8}},
	{0x00, 1, {0x82}},
	{0xC5, 7, {0x4B,0x4B,0x3C,0x3C,0x00,0x60,0x0C}},
	{0x00, 1, {0x00}},
	{0xD8, 2, {0x2B,0x2B}},
	{0x00, 1, {0xA3}},
	{0xC5, 1, {0x1B}},
	{0x00, 1, {0xA9}},
	{0xC5, 1, {0x21}},
	{0x00, 1, {0x86}},
	{0xC3, 3, {0x00,0x00,0x00}},
	{0x00, 1, {0x89}},
 	{0xF5, 1, {0x0D}},
	{0x00, 1, {0x96}},
	{0xF5, 1, {0x0D}},
	{0x00, 1, {0xA6}},
	{0xF5, 1, {0x5F}},
	{0x00, 1, {0xB1}},
	{0xF5, 1, {0x1E}},
	{0x00, 1, {0x81}},
	{0xF5, 2, {0x5F,0x5F}},
	{0x00, 1, {0x86}},
	{0xF5, 2, {0x5F,0x5F}},
	{0x00, 1, {0xAA}},
	{0xF5, 1, {0x8E}},
	{0x00, 1, {0x85}},
	{0xC4, 1, {0x1E}},
	{0x00, 1, {0xB7}},
	{0xCE, 2, {0x2B,0x05}},
	{0x00, 1, {0x90}},
	{0xC5, 1, {0x83}},
	{0x00, 1, {0x92}},
	{0xC5, 1, {0x63}},
	{0x00, 1, {0xE8}},
	{0xC0, 1, {0x40}},
	{0x00, 1, {0x87}},
	{0xC4, 1, {0x40}},
	{0x00, 1, {0x9B}},
	{0xF5, 4, {0x8D,0x8C,0x8D,0x8A}},
	{0x00, 1, {0x91}},
	{0xF5, 2, {0xED,0x8C}},
	{0x00, 1, {0x95}},
	{0xF5, 1, {0x8A}},
	{0x00, 1, {0x98}},
	{0xF5, 1, {0xEB}},
	{0x00, 1, {0x85}},
	{0xA7, 1, {0x0F}},
	{0x00, 1, {0xA0}},
	{0xD7, 1, {0x04}},
	{0x00, 1, {0x00}},
	{0xE1, 40, {0x00,0x04,0x0A,0x12,0x20,0x1D,0x26,0x2C,0x37,0x8C,0x3F,0x46,0x4C,0x52,0xE8,0x57,0x5F,0x67,0x6D,0x23,0x74,0x7C,0x83,0x8C,0xED,0x95,0x9B,0xA1,0xA8,0x81,0xB0,0xBA,0xC7,0xCE,0x57,0xD8,0xE4,0xEB,0xF0,0xB7}},
	{0x00, 1, {0x00}},
	{0xE2, 40, {0x00,0x04,0x0A,0x12,0x20,0x1D,0x26,0x2D,0x38,0xC0,0x41,0x48,0x4E,0x54,0x78,0x58,0x61,0x69,0x6F,0xE3,0x76,0x7C,0x83,0x8C,0xED,0x95,0x9B,0xA1,0xA8,0x81,0xB0,0xBA,0xC6,0xCE,0x50,0xD8,0xE4,0xEB,0xF0,0x77}},
	{0x00, 1, {0x00}},
	{0xE3, 40 ,{0x00,0x04,0x09,0x11,0x19,0x1C,0x24,0x2B,0x36,0xB8,0x3E,0x45,0x4A,0x50,0x8D,0x55,0x5D,0x64,0x6B,0x2D,0x72,0x79,0x80,0x88,0x19,0x90,0x96,0x9C,0xA2,0xD1,0xA9,0xB1,0xBB,0xC2,0x1E,0xC9,0xD1,0xD7,0xD9,0xB7}},
	{0x00, 1, {0x00}},
	{0xE4, 40 ,{0x00,0x04,0x09,0x11,0x19,0x1C,0x24,0x2B,0x36,0xF9,0x3F,0x47,0x4C,0x52,0x8D,0x57,0x5F,0x66,0x6D,0x1E,0x74,0x79,0x80,0x88,0x29,0x90,0x96,0x9B,0xA1,0xC1,0xA9,0xB1,0xBB,0xC1,0x1B,0xC8,0xD1,0xD7,0xD9,0x12}},
	{0x00, 1, {0x00}},
	{0xE5, 40, {0x00,0x04,0x09,0x11,0x2A,0x1C,0x25,0x2C,0x37,0xD0,0x3F,0x46,0x4C,0x51,0x02,0x56,0x5E,0x66,0x6D,0x74,0x73,0x7B,0x83,0x8B,0xE0,0x94,0x9A,0xA0,0xA7,0x41,0xAF,0xB9,0xC7,0xCF,0xB6,0xDB,0xEA,0xF5,0xFB,0x13}},
	{0x00, 1, {0x00}},
	{0xE6, 40, {0x00,0x04,0x09,0x11,0x2A,0x1D,0x25,0x2C,0x37,0x10,0x40,0x48,0x4E,0x53,0x82,0x58,0x60,0x68,0x6F,0x74,0x75,0x7B,0x83,0x8B,0xE0,0x94,0x9A,0xA0,0xA7,0x41,0xAF,0xB9,0xC6,0xCF,0xA0,0xDA,0xEA,0xF5,0xFB,0xD3}},
	{0x00, 1, {0xB0}},
	{0xF5, 1, {0x00}},
	{0x00, 1, {0xC1}},
	{0xB6, 3, {0x09,0x89,0x68}},
	{0x00, 1, {0x80}},
	{0xB4, 1, {0x0A}},
	{0x00, 1, {0x8C}},
	{0xC3, 1, {0x01}},
	{0x00, 1, {0x8E}},
	{0xC3, 1, {0x10}},
	{0x00, 1, {0x8A}},
	{0xC0, 2, {0x1C,0x05}},
	{0x00, 1, {0xB0}},
	{0xF3, 2, {0x02,0xFD}},
	/* PWM GAIN*/
	{0x00, 1, {0x80}},
	{0xCA, 12, {0xCA, 0xB8, 0xA9, 0x9D, 0x95, 0x8D, 0x87, 0x82, 0x80, 0x80, 0x80, 0x80} },
	/*PWM  DUTY*/
	{0x00, 1, {0x90}},
	{0xCA, 9, {0xFC ,0xFF ,0x00 ,0xF8 ,0xFF ,0x00 ,0xF8 ,0xFF ,0x00} },
	/*PWM DIM*/
	{0x00, 1, {0xb5} },
	{0xca, 1, {0x08} },
	/*CABC DIM*/
	{0x00, 1, {0xA0} },
	{0xCA, 3, {0x08 ,0x08 ,0x08} },

	{0x51, 2, {0xFF, 0xFF} },
	{0x53,1,{0x24}},

	{0x11, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 20, {} },
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}


static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density            = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 1;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active =4; //4;
	params->dsi.vertical_backporch = 26;//50;
	params->dsi.vertical_frontporch = 112;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 6;//4;
	params->dsi.horizontal_backporch = 56;
	params->dsi.horizontal_frontporch = 56;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.HS_PRPR = 10;
	params->dsi.ssc_disable  = 1;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 520;	/* this value must be in MTK suggested table */
#else
	params->dsi.PLL_CLOCK = 569;	/* this value must be in MTK suggested table */
#endif
	params->dsi.PLL_CK_CMD = 520;
	params->dsi.PLL_CK_VDO = 480;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = FRAME_WIDTH;
	params->corner_pattern_height = 16;
#endif
}

static void lcm_init_power(void)
{
	printk("vincent==>%s: panel_reset_state = %d\n", __func__, panel_reset_state);
	if (panel_reset_state == 0) {
		lcm_enp_setting(1);
		MDELAY(6);
		lcm_enn_setting(1);
		MDELAY(5);
		panel_reset_state = 1; /*set reset state*/
	}
}

static void lcm_suspend_power(void)
{
	printk("vincent==>%s:line=%d\n", __func__, __LINE__);

	lcm_enn_setting(0);
	MDELAY(4);
	lcm_enp_setting(0);
	MDELAY(15);


	panel_reset_state = 0; /* clear reset state*/
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}

static void lcm_init(void)
{
	panel_off_state = 0;
	tp_reset_setting(0);
	lcm_reset_setting(0);
	MDELAY(5);
	tp_reset_setting(1);
	lcm_reset_setting(1);
	MDELAY(20);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_LOGI("----lcm_init----lcm mode = cmd mode :%d----\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	printk("vincent==>%s:%d\n", __func__, __LINE__);
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	panel_off_state = 1;
}

static void lcm_resume(void)
{
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;	/* mode control addr */
		lcm_switch_mode_cmd.val[0] = 0x13;	/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[1] = 0x10;	/* disable video mode secondly */
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0] = 0x03;	/* disable GRAM and enable video mode */
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}

/* partial update restrictions:
 * 1. roi width must be 1080 (full lcm width)
 * 2. vertical start (y) must be multiple of 16
 * 3. vertical height (h) must be multiple of 16
 */
static void lcm_validate_roi(int *x, int *y, int *width, int *height)
{
	unsigned int y1 = *y;
	unsigned int y2 = *height + y1 - 1;
	unsigned int x1, w, h;

	x1 = 0;
	w = FRAME_WIDTH;

	y1 = round_down(y1, 16);
	h = y2 - y1 + 1;

	/* in some cases, roi maybe empty. In this case we need to use minimu roi */
	if (h < 16)
		h = 16;

	h = round_up(h, 16);

	/* check height again */
	if (y1 >= FRAME_HEIGHT || y1 + h > FRAME_HEIGHT) {
		/* assign full screen roi */
		pr_warn("%s calc error,assign full roi:y=%d,h=%d\n", __func__, *y, *height);
		y1 = 0;
		h = FRAME_HEIGHT;
	}

	/*pr_err("lcm_validate_roi (%d,%d,%d,%d) to (%d,%d,%d,%d)\n",*/
	/*	*x, *y, *width, *height, x1, y1, w, h);*/

	*x = x1;
	*width = w;
	*y = y1;
	*height = h;
}


static void lcm_reset_for_touch(void)
{
	tp_reset_setting(0);
	lcm_reset_setting(0);
	MDELAY(2);
	tp_reset_setting(1);
	lcm_reset_setting(1);
	MDELAY(3);
	panel_reset_state = 1;
}

/******************************************lcmcabc start**************************************************/
static void lcm_vivo_set_cabc(void *handle, unsigned char levelsetting)
{
	if (levelsetting == 0)
		push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	else if (levelsetting == 1)
		push_table(handle, lcm_cmd_cabc_level1, sizeof(lcm_cmd_cabc_level1) / sizeof(struct LCM_setting_table), 1);
	else if (levelsetting == 2)
		push_table(handle, lcm_cmd_cabc_level2, sizeof(lcm_cmd_cabc_level2) / sizeof(struct LCM_setting_table), 1);
	else if (levelsetting == 3)
		push_table(handle, lcm_cmd_cabc_level3, sizeof(lcm_cmd_cabc_level3) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----lcmcabc----- level:%d\n",  levelsetting);
}
static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	if (cmdtype == 0x81) /*cabc*/
		lcm_vivo_set_cabc(handle, levelsetting);
}
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{

	if (lcm_software_id == 0x7A)
		return 0x42;
	else
		return 0xff;
}

struct LCM_DRIVER pd1831_ft8719_fhdplus_dsi_vdo_dijing_lcm_drv = {
	.name = "pd1831_ft8719_fhdplus_dsi_vdo_dijing",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.update = lcm_update,
	.switch_mode = lcm_switch_mode,
	.validate_roi = lcm_validate_roi,
	.get_id	    = lcm_get_id,
	.lcm_MipiCmd_HS  = lcm_vivo_MipiCmd_HS,
	.lcm_reset = lcm_reset_for_touch,
	.get_project_name = "PD1831",
};
