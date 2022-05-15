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

#define lcm_vci_setting(cmd) \
		lcm_util.lcm_vci_setting(cmd)
#define lcm_reset_setting(cmd) \
		lcm_util.lcm_reset_setting(cmd)

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									1
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2400)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(67932)
#define LCM_PHYSICAL_HEIGHT									(147186)
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

//int oled_dimming_enable = 0;
//int oled_weakup_hbmon;
/*
static int hbm_wait;
static int oled_hbm_on;
extern unsigned int lcm_ldo_vision;
extern atomic_t oled_hbm_status;
extern atomic_t doze_mode_state;
extern unsigned int bl_brightness_hal;
extern unsigned int current_backlight;
extern unsigned int lcm_software_id;
extern unsigned int lcm_especial_id;
int oled_dimming_enable = 0;
int oled_weakup_hbmon;
extern int vivo_colour_gamut;
static void dsi_panel_update_seed_crc(void *handle, unsigned char levelsetting);
*/
#if 0
static struct LCM_setting_table init_setting[] = {
	/* Sleep Out */
	{0x9F,2,{0xA5,0xA5}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 20, {} },
	{0x9F,2,{0x5A,0x5A}},
	/* TE on */
	{0x9F,2,{0xA5,0xA5}},
	{0x35,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},
	/* PCD off */
	{0xF0,2,{0x5A,0x5A}},
	{0xE6,1,{0x40}},
	{0xF0,2,{0xA5,0xA5}},
	/*esd sw work around*/
	{0xF1,2,{0x5A,0x5A}},
	{0xFC,2,{0x5A,0x5A}},
	{0xB0,1,{0x01}},
	{0xE3,1,{0xC8}},
	{0xB0,1,{0x07}},
	{0xED,1,{0x67}},
	{0xD0,1,{0x08}},
	{0xF1,2,{0xA5,0xA5}},
	{0xFC,2,{0xA5,0xA5}},
	/* Brightness Setting */
	{0xF0,2,{0x5A,0x5A}},
	{0xB0,1,{0x07}},
	{0xB7,1,{0x01}},
	{0xF0,2,{0xA5,0xA5}},
	{0x53,1,{0x20}},
	{REGFLAG_DELAY, 100, {} },
	{0x55,1,{0x00}},
	{0x51,2,{0xFF,0xFF}},
	/* Display On */
	{0x9F,2,{0xA5,0xA5}},
	{0x29,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},
	/* Display On p3_mode*/
	{0xF0,2,{0x5A,0x5A}},
	{0x81,1,{0x90}},
	{0xB1,23,{0x00,0x00,0xFE,0x00,0x05,0x00,0xEF,0x00,0x07,0x07,0xFF,0x15,0xFF,0xE2,0xFF,0x04,0xEC,0xEF,0xE7,0x00,0xFF,0xFF,0xFF}},
	{0xF0,2,{0xA5,0xA5}},
	/* ACL dimming setting */
	{0xF0,2,{0x5A,0x5A}},
	{0xB0,1,{0xDE}},
	{0xB9,1,{0x48}},
	{0xF0,2,{0xA5,0xA5}},
};
#endif


static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x07, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
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
	printk("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
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
	params->dsi.vertical_frontporch = 40;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 20;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable  = 1;

	params->dsi.PLL_CLOCK = 569;
	params->dsi.HS_PRPR = 12;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9e;
 #ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->full_content = 1;
	params->corner_pattern_width = FRAME_WIDTH;
	params->corner_pattern_height = 120;
	params->corner_pattern_height_bot = 120;
 #endif
	params->hbm_en_time = 2;
	params->hbm_dis_time = 0;
}

static void lcm_init_power(void)
{
	LCM_INFO("lcm_init_power\n");
}

static void lcm_suspend_power(void)
{
	LCM_INFO("lcm_suspend_power\n");
}

static void lcm_resume_power(void)
{
	LCM_INFO("lcm_resume_power\n");
}




static void lcm_init(void)
{
	LCM_INFO("lcm_init start\n");
	//lcm_reset_setting(1);
	MDELAY(5);
	//lcm_reset_setting(0);
	MDELAY(5);
	//lcm_reset_setting(1);
	MDELAY(6);

	//push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("lcm_init end\n");

}

static void lcm_suspend(void)
{
	LCM_INFO("lcm_suspend\n");
}

static void lcm_resume(void)
{
	LCM_INFO("lcm_resume\n");
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

	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("lcm_setbacklight_cmdq\n");
}



static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	/****************
	21		==> DongGuan Factory
	22		==> DongGuan Factory with Different thickness
	23		==> TianJin Factory
	****************/
	return 0x21;

}



struct LCM_DRIVER pd2062_ams644_s6e3fc2_fhdp_cmd60_lcm_drv = {
	.name = "pd2062_ams644_s6e3fc2_fhdp_cmd60_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.update = lcm_update,

	.get_id	    = lcm_get_id,

};
