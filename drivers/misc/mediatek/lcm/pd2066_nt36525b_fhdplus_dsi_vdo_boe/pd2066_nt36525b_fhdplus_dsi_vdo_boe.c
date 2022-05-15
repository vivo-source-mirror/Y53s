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

extern void lcm_bias_set_avdd_n_avee(int value);
extern unsigned int phone_shutdown_state;
/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT									(1600)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(67932)
#define LCM_PHYSICAL_HEIGHT		(150960)
#define LCM_DENSITY											(480)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

#define WLED_CABC_ENABLE_LEVEL			403

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
static unsigned int dimming_enable = 1;
extern unsigned int bl_lvl;
extern unsigned int cabc_state_for_upper_set;
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

	{0x10, 0, {} },
	{REGFLAG_DELAY, 80, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00, 0x6E, 0x00, 0x7B, 0x00, 0x86, 0x00, 0x93, 0x00, 0xA0, 0x00, 0xAC, 0x00, 0xB6, 0x00, 0xC3} },
	{0xB1, 16, {0x00, 0xCD, 0x00, 0xF0, 0x01, 0x11, 0x01, 0x48, 0x01, 0x74, 0x01, 0xBE, 0x01, 0xFC, 0x01, 0xFE} },
	{0xB2, 16, {0x02, 0x3A, 0x02, 0x7F, 0x02, 0xAC, 0x02, 0xE7, 0x03, 0x0E, 0x03, 0x3C, 0x03, 0x4A, 0x03, 0x59} },
	{0xB3, 12, {0x03, 0x6C, 0x03, 0x82, 0x03, 0x9E, 0x03, 0xBE, 0x03, 0xD9, 0x03, 0xDA} },
	{0xB4, 16, {0x00, 0x38, 0x00, 0x45, 0x00, 0x59, 0x00, 0x6B, 0x00, 0x7A, 0x00, 0x88, 0x00, 0x94, 0x00, 0xA2} },
	{0xB5, 16, {0x00, 0xAE, 0x00, 0xDA, 0x00, 0xFE, 0x01, 0x3B, 0x01, 0x6B, 0x01, 0xB9, 0x01, 0xF7, 0x01, 0xF9} },
	{0xB6, 16, {0x02, 0x36, 0x02, 0x7C, 0x02, 0xA9, 0x02, 0xE4, 0x03, 0x09, 0x03, 0x38, 0x03, 0x47, 0x03, 0x56} },
	{0xB7, 12, {0x03, 0x69, 0x03, 0x7E, 0x03, 0x9A, 0x03, 0xB7, 0x03, 0xD9, 0x03, 0xDA} },
	{0xB8, 16, {0x00, 0x00, 0x00, 0x0E, 0x00, 0x2B, 0x00, 0x41, 0x00, 0x53, 0x00, 0x65, 0x00, 0x74, 0x00, 0x83} },
	{0xB9, 16, {0x00, 0x93, 0x00, 0xC5, 0x00, 0xEE, 0x01, 0x31, 0x01, 0x65, 0x01, 0xB6, 0x01, 0xF5, 0x01, 0xF7} },
	{0xBA, 16, {0x02, 0x34, 0x02, 0x7B, 0x02, 0xA9, 0x02, 0xE6, 0x03, 0x0E, 0x03, 0x40, 0x03, 0x49, 0x03, 0x59} },
	{0xBB, 12, {0x03, 0x65, 0x03, 0x78, 0x03, 0x92, 0x03, 0xB5, 0x03, 0xD8, 0x03, 0xDA} },

	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00, 0x6E, 0x00, 0x73, 0x00, 0x7E, 0x00, 0x8B, 0x00, 0x98, 0x00, 0xA4, 0x00, 0xAE, 0x00, 0xBB} },
	{0xB1, 16, {0x00, 0xC5, 0x00, 0xE8, 0x01, 0x09, 0x01, 0x40, 0x01, 0x6C, 0x01, 0xB6, 0x01, 0xF4, 0x01, 0xF6} },
	{0xB2, 16, {0x02, 0x32, 0x02, 0x77, 0x02, 0xA4, 0x02, 0xDF, 0x03, 0x06, 0x03, 0x34, 0x03, 0x42, 0x03, 0x51} },
	{0xB3, 12, {0x03, 0x64, 0x03, 0x7A, 0x03, 0x96, 0x03, 0xB6, 0x03, 0xD1, 0x03, 0xDA} },
	{0xB4, 16, {0x00, 0x38, 0x00, 0x3D, 0x00, 0x51, 0x00, 0x63, 0x00, 0x72, 0x00, 0x80, 0x00, 0x8E, 0x00, 0x9A} },
	{0xB5, 16, {0x00, 0xA6, 0x00, 0xD2, 0x00, 0xF6, 0x01, 0x33, 0x01, 0x63, 0x01, 0xB2, 0x01, 0xEF, 0x01, 0xF1} },
	{0xB6, 16, {0x02, 0x2E, 0x02, 0x74, 0x02, 0xA1, 0x02, 0xDC, 0x03, 0x01, 0x03, 0x30, 0x03, 0x3F, 0x03, 0x4E} },
	{0xB7, 12, {0x03, 0x61, 0x03, 0x76, 0x03, 0x92, 0x03, 0xAF, 0x03, 0xD1, 0x03, 0xDA} },
	{0xB8, 16, {0x00, 0x00, 0x00, 0x06, 0x00, 0x23, 0x00, 0x39, 0x00, 0x4B, 0x00, 0x5D, 0x00, 0x6C, 0x00, 0x7B} },
	{0xB9, 16, {0x00, 0x8B, 0x00, 0xBD, 0x00, 0xE6, 0x01, 0x29, 0x01, 0x5D, 0x01, 0xAE, 0x01, 0xED, 0x01, 0xEF} },
	{0xBA, 16, {0x02, 0x2C, 0x02, 0x73, 0x02, 0xA1, 0x02, 0xDE, 0x03, 0x06, 0x03, 0x3A, 0x03, 0x41, 0x03, 0x51} },
	{0xBB, 12, {0x03, 0x5D, 0x03, 0x70, 0x03, 0x8A, 0x03, 0xAD, 0x03, 0xD0, 0x03, 0xDA} },

	//LV detect debounce time
	{0xFF, 1, {0xF0} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 1, {0x01} },
	{0xCF, 1, {0x22} },

	//ADD VFP>=12
	{0xFF, 1, {0x24} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 1, {0x01} },
	{0x92, 1, {0xB1} },
	{0xB1, 1, {0xAD} },
	{0x93, 1, {0x0C} },
	{0x94, 1, {0x0C} },

	{0xFF, 1, {0x25} },
	{REGFLAG_DELAY, 1, {} },
	{0xFB, 1, {0x01} },
	{0x0A, 1, {0x82} },
	{0x0B, 1, {0x1F} },
	{0x0C, 1, {0x01} },

	{0xFF, 1, {0x23} },
	{0xFB, 1, {0x01} },
	// add cabc UI mode setting
	/*cabc UI mode*/
	/* black 10% */ //----2020/11/17-----//
	{0x30, 1, {0xFF} },
        {0x31, 1, {0xF8} },
        {0x32, 1, {0xF6} },
        {0x33, 1, {0xF5} },
        {0x34, 1, {0xF4} },
        {0x35, 1, {0xF3} },
        {0x36, 1, {0xF3} },
        {0x37, 1, {0xF3} },
        {0x38, 1, {0xF2} },
        {0x39, 1, {0xF1} },
        {0x3A, 1, {0xED} },
        {0x3B, 1, {0xEB} },
        {0x3D, 1, {0xE9} },
        {0x3F, 1, {0xE7} },
        {0x40, 1, {0xE6} },
        {0x41, 1, {0xE5} },
         /* black 20% */
        {0x45, 1, {0xFF} },
        {0x46, 1, {0xFA} },
        {0x47, 1, {0xF6} },
        {0x48, 1, {0xF5} },
        {0x49, 1, {0xF4} },
        {0x4A, 1, {0xF3} },
        {0x4B, 1, {0xF2} },
        {0x4C, 1, {0xF0} },
        {0x4D, 1, {0xED} },
        {0x4E, 1, {0xEA} },
        {0x4F, 1, {0xE7} },
        {0x50, 1, {0xE2} },
        {0x51, 1, {0xDA} },
        {0x52, 1, {0xD5} },
        {0x53, 1, {0xCF} },
        {0x54, 1, {0xCA} },
	 /*dimming setting*/
	{0xFF, 1, {0x23} },
        {0xFB, 1, {0x01} },
        {0x04, 1, {0x05} },
        {0x05, 1, {0x2D} },
        {0x06, 1, {0x01} },
        {0xFF, 1, {0x23} },
        {0xFB, 1, {0x01} },
        {0x00, 1, {0x80} },/* 12bit brightness resolution */
        {0x07, 1, {0x00} },/* set pwm frequency to 15KHz, 12bit bklt has different rule to calc freq with 8bit bklt*/
        {0x08, 1, {0x01} },     //-----2020/11/17---//

	//Backlight setting
	{0xFF, 1, {0x10} },
	{0xFB, 1, {0x01} },
	{0x35, 1, {0x00} },
	{0x51, 2, {0x07, 0xFF} },
	{0x68, 2, {0x03, 0x01} },/* set backlight and cabc diming */
	{0x53, 1, {0x24} },
        {0x55, 1, {0x00} },
	{0xBA, 1, {0x02} },//02-3lane 03-4lane
	//Sleep out&Display on
	{0x29, 0, {} },
	//{REGFLAG_DELAY, 20, {} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 96, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }

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
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 250;
	params->dsi.vertical_frontporch = 12;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4; //14
	params->dsi.horizontal_backporch = 88;   //24
	params->dsi.horizontal_frontporch = 96;  //36
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.HS_PRPR = 10;
	params->dsi.ssc_disable  = 1;
	params->dsi.data_rate = 860;
	params->dsi.PLL_CLOCK = 430; //300
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
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
		MDELAY(3);
		lcm_enn_setting(1);
		MDELAY(10);
		lcm_bias_set_avdd_n_avee(60);
		panel_reset_state = 1;
	}
}

static void lcm_suspend_power(void)
{
	if (phone_shutdown_state) {
		LCM_INFO("[LCM]lcm_suspend_power reset keep low level\n");
		MDELAY(36);
		lcm_reset_setting(0);
	}
	MDELAY(10);
	lcm_enn_setting(0);
	MDELAY(5);
	lcm_enp_setting(0);
	MDELAY(5);
	LCM_INFO("[LCM]lcm_suspend_power nt36525- vddi keep high level\n");
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
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
	lcm_reset_setting(1);
	MDELAY(3);
	lcm_reset_setting(0);
	MDELAY(3);
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

/******************************************lcmcabc start**************************************************/
static void lcm_vivo_set_cabc(void *handle, unsigned char levelsetting)
{
	if (bl_brightness_hal < WLED_CABC_ENABLE_LEVEL)
		levelsetting = 0;
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

static void vivo_adjust_cabc(void *handle, unsigned int level)
{
	if (level < WLED_CABC_ENABLE_LEVEL && lcm_cabc_status) {
		lcm_vivo_set_cabc(handle, 0);
		LCM_INFO("Close cabc when backlight is less than WLED_CABC_ENABLE_LEVEL brightness:%d\n", level);
	} else if (level > WLED_CABC_ENABLE_LEVEL
		&& cabc_state_for_upper_set != lcm_cabc_status) {
		lcm_vivo_set_cabc(handle, cabc_state_for_upper_set);
		LCM_INFO("Recover cabc when backlight is greater than WLED_CABC_ENABLE_LEVEL brightness:%d\n", level);
	}
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	if (level)
		vivo_adjust_cabc(handle, level);

	if (level == LED_FULL)
			level = 4095;
	else
		level = level * 2;
	if ((level != 0) && (dimming_enable == 0) && (bl_brightness_hal != 0)) {
		push_table(handle, lcm_cmd_backlight_dimming_enable, sizeof(lcm_cmd_backlight_dimming_enable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 1;
		MDELAY(20);	//vivo  add by hehuan for dimming enable
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", bl_brightness_hal, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
	}
	if (level == 0) {
		push_table(handle, lcm_cmd_backlight_dimming_disable, sizeof(lcm_cmd_backlight_dimming_disable) / sizeof(struct LCM_setting_table), 1);
		dimming_enable = 0;
		LCM_INFO("LastBacklight = %d, dimming_enable=%d, high_bit=0x%x, low_bit=0x%x\n", bl_brightness_hal, dimming_enable, bl_level_12bit[0].para_list[0], bl_level_12bit[0].para_list[1]);
		MDELAY(40);
	}
	bl_lvl = level;
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
	MDELAY(12);
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	/*Set CABC when backlight is greater than WLED_CABC_ENABLE_LEVEL*/
	if (cmdtype == 0x81)
		lcm_vivo_set_cabc(handle, levelsetting);
}
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_software_id == 0x7C)
		return 0x31;
	else
		return 0xFF;
}

struct LCM_DRIVER pd2066_nt36525b_fhdplus_dsi_vdo_boe_lcm_drv = {
	.name = "pd2066_nt36525b_fhdplus_dsi_vdo_boe",
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
