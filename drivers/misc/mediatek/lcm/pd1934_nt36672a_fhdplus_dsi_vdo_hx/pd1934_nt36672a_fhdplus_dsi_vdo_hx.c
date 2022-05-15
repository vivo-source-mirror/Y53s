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

#include <linux/string.h>
#include <linux/kernel.h>
#include "lcm_drv.h"
#include "mtkfb.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

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

/*static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;*/

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int lcm_software_id;
extern unsigned int bl_brightness_hal;
static unsigned int dimming_enable =1 ;
extern unsigned int lcm_cabc_status;

static struct LCM_setting_table bl_level_12bit[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_enable[] = {
	{0x53, 1, {0x2C} },
};

static struct LCM_setting_table lcm_cmd_backlight_dimming_disable[] = {
	{0x53, 1, {0x24} },
};

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x03} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0x53, 1, {0x2c} },
	{0x55, 1, {0x00} },
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },

	{0x10, 0, {} },
	{REGFLAG_DELAY, 160, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x07, 1, {0x9E} },
	{0x0F, 1, {0xA4} },
	{0x44, 1, {0x50} },
	{0x46, 1, {0x01} },
	{0x62, 1, {0xB0} },
	{0x6D, 1, {0x44} },
	{0x78, 1, {0x01} },
	{0x95, 1, {0xE1} },
	{0x96, 1, {0xE1} },
	{0xFF, 1, {0x24} },
	{0xFB, 1, {0x01} },
	{0x00, 1, {0x1C} },
	{0x01, 1, {0x1C} },
	{0x02, 1, {0x1C} },
	{0x03, 1, {0x1C} },
	{0x04, 1, {0x1C} },
	{0x05, 1, {0x20} },
	{0x06, 1, {0x1C} },
	{0x07, 1, {0x20} },
	{0x08, 1, {0x10} },
	{0x09, 1, {0x12} },
	{0x0A, 1, {0x14} },
	{0x0B, 1, {0x1E} },
	{0x0C, 1, {0x0D} },
	{0x0D, 1, {0x1C} },
	{0x0E, 1, {0x0A} },
	{0x0F, 1, {0x1C} },
	{0x10, 1, {0x01} },
	{0x11, 1, {0x03} },
	{0x12, 1, {0x04} },
	{0x13, 1, {0x05} },
	{0x14, 1, {0x1C} },
	{0x15, 1, {0x09} },
	{0x16, 1, {0x1C} },
	{0x17, 1, {0x1C} },
	{0x18, 1, {0x1C} },
	{0x19, 1, {0x1C} },
	{0x1A, 1, {0x1C} },
	{0x1B, 1, {0x1C} },
	{0x1C, 1, {0x20} },
	{0x1D, 1, {0x1C} },
	{0x1E, 1, {0x0E} },
	{0x1F, 1, {0x10} },
	{0x20, 1, {0x12} },
	{0x21, 1, {0x14} },
	{0x22, 1, {0x1E} },
	{0x23, 1, {0x0D} },
	{0x24, 1, {0x1C} },
	{0x25, 1, {0x0A} },
	{0x26, 1, {0x1C} },
	{0x27, 1, {0x01} },
	{0x28, 1, {0x03} },
	{0x29, 1, {0x04} },
	{0x2A, 1, {0x05} },
	{0x2B, 1, {0x1C} },
	{0x2D, 1, {0x09} },
	{0x2F, 1, {0x1C} },
	{0x31, 1, {0x04} },
	{0x32, 1, {0x08} },
	{0x34, 1, {0x02} },
	{0x35, 1, {0x49} },
	{0x37, 1, {0x03} },
	{0x38, 1, {0x6B} },
	{0x39, 1, {0x6B} },
	{0x3F, 1, {0x6B} },
	{0x41, 1, {0x04} },
	{0x42, 1, {0x08} },
	{0x4C, 1, {0x06} },
	{0x4D, 1, {0x06} },
	{0x60, 1, {0x90} },
	{0x61, 1, {0x24} },
	{0x72, 1, {0x00} },
	{0x73, 1, {0x00} },
	{0x74, 1, {0x00} },
	{0x75, 1, {0x00} },
	{0x79, 1, {0x22} },
	{0x7A, 1, {0x04} },
	{0x7B, 1, {0x9F} },
	{0x7C, 1, {0x80} },
	{0x7D, 1, {0x02} },
	{0x80, 1, {0x41} },
	{0x81, 1, {0x01} },
	{0x82, 1, {0x13} },
	{0x83, 1, {0x22} },
	{0x84, 1, {0x31} },
	{0x85, 1, {0x00} },
	{0x86, 1, {0x00} },
	{0x87, 1, {0x00} },
	{0x88, 1, {0x13} },
	{0x89, 1, {0x22} },
	{0x8A, 1, {0x31} },
	{0x8B, 1, {0x00} },
	{0x8C, 1, {0x00} },
	{0x8D, 1, {0x00} },
	{0x8E, 1, {0xF4} },
	{0x8F, 1, {0x01} },
	{0x90, 1, {0x80} },
	{0x92, 1, {0x72} },
	{0x93, 1, {0x16} },
	{0xB3, 1, {0x0A} },
	{0xB4, 1, {0x04} },
	{0xDC, 1, {0x29} },
	{0xDD, 1, {0x04} },
	{0xDE, 1, {0x03} },
	{0xDF, 1, {0x6F} },
	{0xE0, 1, {0x66} },
	{0xEB, 1, {0x04} },
	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x21, 1, {0x1F} },
	{0x22, 1, {0x1F} },
	{0x24, 1, {0x72} },
	{0x25, 1, {0x72} },
	{0x30, 1, {0x2F} },
	{0x38, 1, {0x2F} },
	{0x3F, 1, {0x21} },
	{0x40, 1, {0x63} },
	{0x4B, 1, {0x21} },
	{0x4C, 1, {0x63} },
	{0x69, 1, {0x10} },
	{0x84, 1, {0x78} },
	{0x85, 1, {0x75} },
	{0x8D, 1, {0x00} },
	{0xBF, 1, {0x00} },
	{0xC2, 1, {0x5B} },
	{0xC3, 1, {0x13} },
	{0xD9, 1, {0x48} },
	{0xDB, 1, {0x22} },
	{0xFF, 1, {0x26} },
	{0xFB, 1, {0x01} },
	{0x06, 1, {0xFF} },
	{0x12, 1, {0x75} },
	{0x1A, 1, {0x1A} },
	{0x1C, 1, {0xAF} },
	{0x1E, 1, {0x9D} },
	{0x98, 1, {0xF1} },
	{0xA9, 1, {0x11} },
	{0xAA, 1, {0x11} },
	{0xAE, 1, {0x8A} },
	{0xFF, 1, {0x27} },
	{0xFB, 1, {0x01} },
	{0x13, 1, {0x00} },
	{0x1E, 1, {0x14} },
	{0x22, 1, {0x00} },
	{0xFF, 1, {0xF0} },
	{0xFB, 1, {0x01} },
	{0xA2, 1, {0x00} },
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00,0x32,0x00,0x4E,0x00,0x7A,0x00,0x9D,0x00,0xBA,0x00,0xD2,0x00,0xE7,0x00,0xFA} },
	{0xB1, 16, {0x01,0x0A,0x01,0x41,0x01,0x68,0x01,0xA5,0x01,0xD1,0x02,0x18,0x02,0x4C,0x02,0x4E} },
	{0xB2, 16, {0x02,0x83,0x02,0xBD,0x02,0xE3,0x03,0x12,0x03,0x32,0x03,0x57,0x03,0x67,0x03,0x76} },
	{0xB3, 12, {0x03,0x8C,0x03,0xA4,0x03,0xBD,0x03,0xCF,0x03,0xDA,0x03,0xDC} },
	{0xB4, 16, {0x00,0x77,0x00,0x87,0x00,0xAC,0x00,0xC7,0x00,0xDB,0x00,0xEF,0x01,0x00,0x01,0x0F} },
	{0xB5, 16, {0x01,0x1E,0x01,0x4E,0x01,0x71,0x01,0xAC,0x01,0xD5,0x02,0x1A,0x02,0x4D,0x02,0x4F} },
	{0xB6, 16, {0x02,0x83,0x02,0xBD,0x02,0xE3,0x03,0x12,0x03,0x32,0x03,0x58,0x03,0x68,0x03,0x77} },
	{0xB7, 12, {0x03,0x8A,0x03,0x98,0x03,0xAE,0x03,0xB4,0x03,0xDA,0x03,0xDC} },
	{0xB8, 16, {0x00,0x08,0x00,0x3B,0x00,0x71,0x00,0x98,0x00,0xB7,0x00,0xD1,0x00,0xE7,0x00,0xFA} },
	{0xB9, 16, {0x01,0x0B,0x01,0x43,0x01,0x68,0x01,0xA6,0x01,0xD2,0x02,0x18,0x02,0x4C,0x02,0x4E} },
	{0xBA, 16, {0x02,0x82,0x02,0xBC,0x02,0xE2,0x03,0x11,0x03,0x30,0x03,0x55,0x03,0x63,0x03,0x71} },
	{0xBB, 12, {0x03,0x8A,0x03,0xA4,0x03,0xBE,0x03,0xD0,0x03,0xDA,0x03,0xDC} },
	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00,0x2A,0x00,0x46,0x00,0x72,0x00,0x95,0x00,0xB2,0x00,0xCA,0x00,0xDF,0x00,0xF2} },
	{0xB1, 16, {0x01,0x02,0x01,0x39,0x01,0x60,0x01,0x9D,0x01,0xC9,0x02,0x10,0x02,0x44,0x02,0x46} },
	{0xB2, 16, {0x02,0x7B,0x02,0xB5,0x02,0xDB,0x03,0x0A,0x03,0x2A,0x03,0x4F,0x03,0x5F,0x03,0x6E} },
	{0xB3, 12, {0x03,0x84,0x03,0x9C,0x03,0xB5,0x03,0xC7,0x03,0xD2,0x03,0xD4} },
	{0xB4, 16, {0x00,0x6F,0x00,0x7F,0x00,0xA4,0x00,0xBF,0x00,0xD3,0x00,0xE7,0x00,0xF8,0x01,0x07} },
	{0xB5, 16, {0x01,0x16,0x01,0x46,0x01,0x69,0x01,0xA4,0x01,0xCD,0x02,0x12,0x02,0x45,0x02,0x47} },
	{0xB6, 16, {0x02,0x7B,0x02,0xB5,0x02,0xDB,0x03,0x0A,0x03,0x2A,0x03,0x50,0x03,0x60,0x03,0x6F} },
	{0xB7, 12, {0x03,0x82,0x03,0x90,0x03,0xA6,0x03,0xAD,0x03,0xD2,0x03,0xD4} },
	{0xB8, 16, {0x00,0x00,0x00,0x33,0x00,0x69,0x00,0x90,0x00,0xAF,0x00,0xC9,0x00,0xDF,0x00,0xF2} },
	{0xB9, 16, {0x01,0x03,0x01,0x3B,0x01,0x60,0x01,0x9E,0x01,0xCA,0x02,0x10,0x02,0x44,0x02,0x46} },
	{0xBA, 16, {0x02,0x7A,0x02,0xB4,0x02,0xDA,0x03,0x09,0x03,0x28,0x03,0x4D,0x03,0x5B,0x03,0x69} },
	{0xBB, 12, {0x03,0x82,0x03,0x9C,0x03,0xB6,0x03,0xC8,0x03,0xD2,0x03,0xD4} },
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0xFF, 1, {0x10} },

	{0xFB, 1, {0x01} },
	{0x51, 2, {0x00, 0x00} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x01} },
	{0x3B, 5, {0x03, 0x0E, 0x07, 0x04, 0x04} },
	{0x35, 1, {0x00} },

	{0xFF, 1, {0x23} },
	{REGFLAG_DELAY, 5, {} },
	{0xFB, 1, {0x01} },

	/*cabc UI mode*/
	{0x30, 1, {0xFF} },
	{0x31, 1, {0xF0} },
	{0x32, 1, {0xEB} },
	{0x33, 1, {0xE5} },
	{0x34, 1, {0xDD} },
	{0x35, 1, {0xDA} },
	{0x36, 1, {0xD5} },
	{0x37, 1, {0xD0} },
	{0x38, 1, {0xCE} },
	{0x39, 1, {0xCD} },
	{0x3A, 1, {0xCD} },
	{0x3B, 1, {0xCD} },
	{0x3D, 1, {0xCB} },
	{0x3F, 1, {0xCB} },
	{0x40, 1, {0xC6} },
	{0x41, 1, {0xBF} },
	/*Still mode*/
	{0x45, 1, {0xFF} },
	{0x46, 1, {0xF0} },
	{0x47, 1, {0xE8} },
	{0x48, 1, {0xCE} },
	{0x49, 1, {0xBC} },
	{0x4A, 1, {0xB8} },
	{0x4B, 1, {0xB5} },
	{0x4C, 1, {0xB0} },
	{0x4D, 1, {0xA8} },
	{0x4E, 1, {0xA0} },
	{0x4F, 1, {0x9B} },
	{0x50, 1, {0x98} },
	{0x51, 1, {0x98} },
	{0x52, 1, {0x88} },
	{0x53, 1, {0x80} },
	{0x54, 1, {0x7F} },

	/*for cabc dimming, 8step, 1frame per step */
	{0xff, 1, {0x23} },
	{0xfb, 1, {0x01} },
	{0x00, 1, {0x80} },
	{0x04, 1, {0x05} },		/*step 8 ui mode*/
	{0x05, 1, {0x2d} },		/*still mode and mov mode step 8*/
	{0x06, 1, {0x01} },		/*1 frame  53 = 2c*/
	{0x07, 1, {0x00} },
	{0x08, 1, {0x01} },
	{0x09, 1, {0x00} },
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x68, 2, {0x03, 0x01} },	/* step 8  1 frame	53 = 2c 55 = 00*/

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
	LCM_INFO("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 22;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 112;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.HS_PRPR = 10;
	params->dsi.ssc_disable  = 1;
	params->dsi.PLL_CLOCK = 563;	/* this value must be in MTK suggested table */
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

	params->dsi.lcm_esd_check_table[1].cmd = 0x0d;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_width = FRAME_WIDTH;
	params->corner_pattern_height = 16;
#endif
}

static void lcm_init_power(void)
{
	LCM_INFO("panel_reset_state = %d, lcm_software_id = 0x%x\n", panel_reset_state, lcm_software_id);
	if (panel_reset_state == 0) {

		lcm_enp_setting(1);
		MDELAY(2);
		lcm_enn_setting(1);
		MDELAY(10);
		panel_reset_state = 1;
	}
}

static void lcm_suspend_power(void)
{
	lcm_enn_setting(0);
	MDELAY(1);
	lcm_enp_setting(0);
	MDELAY(15);
	panel_reset_state = 0;
	LCM_INFO("\n");
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}

static void lcm_init(void)
{
	panel_off_state = 0;
	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(15);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("----lcm_init----lcm mode = cmd mode :%d----\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	LCM_INFO("lcm_software_id = 0x%x\n", lcm_software_id);
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
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

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level == LED_FULL)
			level =4095;
	else
		level = level * 2;
	if ((level != 0) && (dimming_enable == 0) && (bl_brightness_hal != 0)) {
		push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", bl_brightness_hal, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	}
	if (level == 0) {
		push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 0;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", bl_brightness_hal, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	}
	bl_level_12bit[0].para_list[0] = (unsigned char)((level>>8)&0xFF);
	bl_level_12bit[0].para_list[1] = (unsigned char)(level&0xFFFF);
	LCM_INFO("LastBacklight = %d, level=%d, high_bit=0x%x, low_bit=0x%x\n", bl_brightness_hal, level, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	push_table(handle, bl_level_12bit, sizeof(bl_level_12bit) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_reset_for_touch(void)
{
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(3);
}

/******************************************lcmcabc start**************************************************/
static void lcm_vivo_set_cabc(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_cmd_cabc_off) > 0))
		push_table(handle, lcm_cmd_cabc_off, sizeof(lcm_cmd_cabc_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_cabc_level1) > 0))
		push_table(handle, lcm_cmd_cabc_level1, sizeof(lcm_cmd_cabc_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_cabc_level2) > 0))
		push_table(handle, lcm_cmd_cabc_level2, sizeof(lcm_cmd_cabc_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_cabc_level3) > 0))
		push_table(handle, lcm_cmd_cabc_level3, sizeof(lcm_cmd_cabc_level3) / sizeof(struct LCM_setting_table), 1);
	lcm_cabc_status = levelsetting;
	LCM_INFO("vincent=----lcmcabc----- level:%d\n",  levelsetting);
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	if (cmdtype == 0x81) /*cabc*/
		lcm_vivo_set_cabc(handle, levelsetting);
}
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_software_id == 0x50)
		return 0x81;
	else
		return 0xff;
}

struct LCM_DRIVER pd1934_nt36672a_fhdplus_dsi_vdo_hx_lcm_drv = {
	.name = "pd1934_nt36672a_fhdplus_dsi_vdo_hx",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.update = lcm_update,
	//.switch_mode = lcm_switch_mode,
	//.validate_roi = lcm_validate_roi,
	.get_id	    = lcm_get_id,
	.lcm_MipiCmd_HS  = lcm_vivo_MipiCmd_HS,
	.lcm_reset = lcm_reset_for_touch,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
};
