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
#define FRAME_HEIGHT									(2340)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(68256)
#define LCM_PHYSICAL_HEIGHT									(144096)
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
#if 0
static int oled_hbm_on;
static int oled_dimming_enable = 1;
extern unsigned int bl_brightness_hal;
extern int oled_hbm_status;
extern unsigned int current_backlight;
#endif
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },

	/*lrz transplant from PD1721, 89% black picture to the problem : x21 flick*/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0C} },
	{0xCB, 1, {0xFF} },
	{0xB0, 1, {0x1A} },
	{0xCB, 1, {0xFB} },
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },

	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 5, {} },
	{0x9F, 2, {0x5A, 0x5A} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x6E} },
	{0xB9, 1, {0x81} },
	{0xB0, 1, {0x09} },
	{0xEF, 1, {0x31} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x07} },
	{0xCD, 1, {0x12} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x9F, 2, {0xA5, 0xA5} },
	{0x35, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{0xFC, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x01} },
	{0xE3, 1, {0x88} },
	{0xB0, 1, {0x07} },
	{0xED, 1, {0x67} },
	{0xFC, 2, {0xA5, 0xA5} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x07} },
	{0xB7, 1, {0x01} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x20} },
	{REGFLAG_DELAY, 100, {} },
	{0x55, 1, {0x00} },
	{0x9F, 2, {0xA5, 0xA5} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0xDE} },
	{0xB9, 1, {0x48} },
	{0x81, 1,{0x90}},
	{0xB1, 23,{0x00,0x00,0xFF,0x03,0x00,0x15,0xFF,0x02,0x03,0x0D,0xFF,0x1D,0xFF,0xEE,0xFF,0x0F,0xF1,0xFF,0xF8,0x01,0xFF,0xFF,0xFF}},
	{0xB6, 2,{0x00,0x02}},
	{0xB3, 45,{0x00,0xC0,0x0F,0x02,0x9A,0x00,0x5C,0x02,0x89,0x00,0x62,0x01,0x84,0x00,0xAF,0x01,0xD2,0x00,0xD3,0x05,0x00,0x13,0x63,0x07,0x79,0x0C,0xD9,0x64,0x00,0x70,0x94,0x00,0x32,0xDC,0x88,0x91,0x00,0xA3,0xAB,0x06,0xDA,0x8D,0x65,0x72,0x49}},
	{0x83, 1,{0x80}},
	{0xF0, 2,{0xA5,0xA5}},
	{0x51, 2, {0x00, 0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x03, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};
#if 0

static struct LCM_setting_table oled_aor_on[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x51, 2, {0x03, 0xFF} },
	{0xB1, 2, {0x08, 0xF8} },
	{0xB0, 1, {0x1A} },
	{0xCB, 1, {0x00} },
	{0xB0, 1, {0x07} },
	{0xCB, 2, {0x01, 0x11} },
	{0xB0, 1, {0x14} },
	{0xCB, 2, {0x41, 0x51} },
	{0xB0, 1, {0x21} },
	{0xCB, 1, {0x3F} },
	{0xB0, 1, {0x26} },
	{0xCB, 1, {0x1F} },
	{0xB0, 1, {0x4E} },
	{0xCB, 2, {0x60, 0x29} },
	{0xB0, 1, {0x5F} },
	{0xCB, 1, {0x1F} },
	{0xF7, 1, {0x03} },
	{0xF4, 1, {0xC0} },
	{0xF0, 2, {0xA5, 0xA5} },

	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table oled_aor_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x51, 2, {0x03, 0xFF} },
	{0xF4, 1, {0xCB} },
	{0xB0, 1, {0x07} },
	{0xCB, 2, {0x61, 0x81} },
	{0xB0, 1, {0x14} },
	{0xCB, 2, {0x46, 0x4B} },
	{0xB0, 1, {0x21} },
	{0xCB, 1, {0x76} },
	{0xB0, 1, {0x26} },
	{0xCB, 1, {0x25} },
	{0xB0, 1, {0x4E} },
	{0xCB, 2, {0x62, 0x26} },
	{0xB0, 1, {0x5F} },
	{0xCB, 1, {0x88} },
	{0xB0, 1, {0x1a} },
	{0xCB, 1, {0x03} },
	{0xB1, 2, {0x00, 0x10} },
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },

	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table brightness_dimming_on[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x05} },
	{0xB1, 1, {0x28} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table brightness_dimming_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x05} },
	{0xB1, 1, {0x01} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table oled_alpm_on[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 17, {} },
};
static struct LCM_setting_table oled_alpm_off[] = {
	{0x22, 0, {} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 32, {} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0x53, 1, {0x28} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_DELAY, 16, {} },
	{0x13, 0, {} },
	{0x29, 0, {} },
};

static struct LCM_setting_table oled_alpm_hbrightness[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0F} },
	{0xF4, 1, {0x50} },
	{0x53, 1, {0x02} },
	{0xBB, 1, {0x0F} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_DELAY, 17, {} },
	{0x29, 0, {} },
};
static struct LCM_setting_table oled_alpm_lbrightness[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0F} },
	{0xF4, 1, {0x50} },
	{0x53, 1, {0x03} },
	{0xBB, 1, {0x0F} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_DELAY, 17, {} },
	{0x29, 0, {} },
};

static struct LCM_setting_table oled_hlpm_hbrightness[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0F} },
	{0xF4, 1, {0x50} },
	{0x53, 1, {0x02} },
	{0xBB, 1, {0x0D} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_DELAY, 17, {} },
	{0x29, 0, {} },
};
static struct LCM_setting_table oled_hlpm_lbrightness[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0F} },
	{0xF4, 1, {0x50} },
	{0x53, 1, {0x03} },
	{0xBB, 1, {0x0D} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_DELAY, 17, {} },
	{0x29, 0, {} },
};

static struct LCM_setting_table oled_cmd_hbm_level1[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB1, 3, {0x10, 0x10, 0xB3} }, /*520 nit*/
	{0xB0, 1, {0x05} },
	{0xB1, 1, {0x28} },
	{0x53, 1, {0xE8} }, /*HBM on*/
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table oled_cmd_hbm_level2[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB1, 3, {0x00, 0x10, 0x10} }, /*600 nit*/
	{0xB0, 1, {0x05} },
	{0xB1, 1, {0x28} },
	{0x53, 1, {0xE8} },
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table oled_cmd_hbm_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x53, 1, {0x28} }, /*HBM off*/
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table lcm_cmd_acl_level1[] = {
	{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_acl_level2[] = {
	{0x55, 1, {0x02} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x0E} },
	{0xB4, 1, {0x20} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table lcm_cmd_acl_level3[] = {
	{0x55, 1, {0x03} },
};

static struct LCM_setting_table lcm_cmd_acl_off[] = {
	{0x55, 1, {0x00} },
};
static struct LCM_setting_table lcm_temperature_compensation_minus_five[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB5, 2, {0x85, 0xCD} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table lcm_temperature_compensation_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x01} },
	{0xB5, 1, {0xC9} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table oled_seed[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xBC, 2, {0x01, 0x12} },
	{0xB0, 1, {0x02} },
	{0xBC, 63, {0xB6, 0x0B, 0x04, 0x39, 0xCB, 0x12, 0x06, 0x06, 0xB4, 0x44, 0xE4, 0xCA, 0xCA, 0x11, 0xC2, 0xE9, 0xE3, 0x18,
		0xF7, 0xF5, 0xFB, 0xB6, 0x0B, 0x04, 0x39, 0xCB, 0x12, 0x06, 0x06, 0xB4, 0x44, 0xE4, 0xCA, 0xCA, 0x11, 0xC2, 0xE9,
			0xE3, 0x18, 0xFF, 0xF0, 0xDB, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
				0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF} },
	{0xF0, 2, {0xA5, 0xA5} },
};
#endif
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
	LCD_INFO("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
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
	/*params->dsi.data_rate = 1029;*/

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
	params->corner_pattern_height = 100;
	params->corner_pattern_height_bot = 100;
 #endif
}

extern struct regulator *vddi_reg;
//extern unsigned int lcm_software_id;
static void lcm_init_power(void)
{
//	int ret;
//	printk("vincent==>%s:%d\n", __func__, __LINE__);
	//if (lcm_software_id != 0x55)
//		ret = regulator_enable(vddi_reg);
//	if (ret)
//		printk("vincent==>%s:%d, enable vddi failed\n", __func__, __LINE__);
	MDELAY(1);
	//lcm_vci_setting(1);
	MDELAY(11);
}

static void lcm_suspend_power(void)
{
//	int ret;
//	printk("vincent==>%s:lcm_software_id=0x%x\n", __func__, ret);

	//lcm_reset_setting(0);
	MDELAY(10);
	//lcm_vci_setting(0);
	MDELAY(10);
	//if (lcm_software_id != 0x55)
//		ret = regulator_disable(vddi_reg);
//	if (ret)
//		printk("vincent==>%s:%d, disable vddi failed\n", __func__, __LINE__);
	MDELAY(40);
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}



static void lcm_init(void)
{
	//lcm_reset_setting(1);
	MDELAY(5);
	//lcm_reset_setting(0);
	MDELAY(5);
	//lcm_reset_setting(1);
	MDELAY(6);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	LCD_INFO("----lcm_init----lcm mode = cmd mode :%d----\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	printk("vincent==>%s:%d\n", __func__, __LINE__);
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
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

#if 0
static struct LCM_setting_table hbm_on_level1[] = {
	{0x51, 2, {0x03, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0x53, 1, {0xE8} },
	{0xB1, 3, {0x10, 0x10, 0xB3} },
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table hbm_on_level2[] = {
	{0x51, 2, {0x03, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0x53, 1, {0xE8} },
	{0xB1, 3, {0x00, 0x10, 0x10} },
	{0xF7, 1, {0x03} },
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table hbm_on_level0[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x53, 1, {0x28} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x51, 2, {0x03, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};
#endif
static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	#if 0
	 if ((level != 0) && (oled_dimming_enable == 0) && (bl_brightness_hal != 0)) {
		push_table(handle, brightness_dimming_on, sizeof(brightness_dimming_on) / sizeof(struct LCM_setting_table), 1);
		oled_dimming_enable = 1;
		printk("vincent+  set dimming on, level =%d, current_backlight = %d\n", level, current_backlight);
	}
	 if (level == 0) {
		oled_dimming_enable = 0;
	}
	 if ((level < 29) && (level > 0))/*lrz modified 1112, jiajie required */
	 level = 29;

	if (oled_hbm_status != 0 && oled_hbm_status < 3 && current_backlight >= 1022 && oled_hbm_on == 0) {
		printk("vincent+ to set hbm on,level=%d, oled_hbm_status=%d\n", level, oled_hbm_status);
		if (oled_hbm_status == 1) {
			hbm_on_level1[0].para_list[0] = (unsigned char)((level>>8)&0xF);
			hbm_on_level1[0].para_list[1] = (unsigned char)(level&0xFF);
			push_table(handle, hbm_on_level1, sizeof(hbm_on_level1) / sizeof(struct LCM_setting_table), 1);
		} else {
			hbm_on_level2[0].para_list[0] = (unsigned char)((level>>8)&0xF);
			hbm_on_level2[0].para_list[1] = (unsigned char)(level&0xFF);
			push_table(handle, hbm_on_level2, sizeof(hbm_on_level2) / sizeof(struct LCM_setting_table), 1);
		}
		oled_hbm_on = 1;
	} else if (oled_hbm_status != 0 && current_backlight < 1022 && oled_hbm_on == 1) {
		printk("vincent+ to set hbm off,level=%d, oled_hbm_status=%d\n", level, oled_hbm_status);
		hbm_on_level0[3].para_list[0] = (unsigned char)((level>>8)&0xF);
		hbm_on_level0[3].para_list[1] = (unsigned char)(level&0xFF);
		push_table(handle, hbm_on_level0, sizeof(hbm_on_level0) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	} else {
		bl_level[0].para_list[0] = (unsigned char)((level>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(level&0xFF);
		if ((level == 2) || (level == 1023))
			printk("vincent+ %s:level=%d, ==>current_backlight = %d\n", __func__, level, current_backlight);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}
	#else
		bl_level[0].para_list[0] = (unsigned char)((level>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(level&0xFF);
		LCD_INFO("level=%d, ==>current_backlight = %d\n", level, level);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	#endif
}
#if 0
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

static void lcm_vivo_set_hbm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_cmd_hbm_off))) {
		push_table(handle, oled_cmd_hbm_off, sizeof(oled_cmd_hbm_off) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	} else if ((levelsetting == 1) && (sizeof(oled_cmd_hbm_level1))) {
		push_table(handle, oled_cmd_hbm_level1, sizeof(oled_cmd_hbm_level1) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if ((levelsetting == 2) && (sizeof(oled_cmd_hbm_level2))) {
		push_table(handle, oled_cmd_hbm_level2, sizeof(oled_cmd_hbm_level2) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	}
      printk("vincent=----lcmhbm----- level:%d\n",  levelsetting);
}

extern char reg[256];
static void lcm_vivo_write_mipi_reg(void *handle, unsigned char levelsetting)
{
	int i;
	/*echo 0f 04 00 08 b8 01 0f 09 ff 18 ff ea ff 0c F8 fc ff 01 fe f5 de > oled_alpmmode*/
	for (i = 42; i < 63; i++) {
		 oled_seed[3].para_list[i] = reg[i-42];
		/*printk("vincent=----lcmalpm----- oled_alpmmode_store:0x%x\n",  oled_seed[3].para_list[i]);*/
	}
	push_table(handle, oled_seed, sizeof(oled_seed) / sizeof(struct LCM_setting_table), 1);
	printk("vincent=----%s----- level:%d\n",__func__,levelsetting);
}

static void lcm_vivo_set_alpm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_alpm_lbrightness)))
		push_table(handle, oled_alpm_lbrightness, sizeof(oled_alpm_lbrightness) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(oled_alpm_hbrightness)))
		push_table(handle, oled_alpm_hbrightness, sizeof(oled_alpm_hbrightness) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----lcmalpm----- level:%d\n",  levelsetting);
}

static void lcm_vivo_set_hlpm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_hlpm_lbrightness)))
		push_table(handle, oled_hlpm_lbrightness, sizeof(oled_hlpm_lbrightness) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(oled_hlpm_hbrightness)))
		push_table(handle, oled_hlpm_hbrightness, sizeof(oled_hlpm_hbrightness) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----lcmhlpm----- level:%d\n",  levelsetting);
}

/******************************************lcmacl start**************************************************/
static void lcm_vivo_set_acl(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_cmd_acl_off)))
		push_table(handle, lcm_cmd_acl_off, sizeof(lcm_cmd_acl_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_acl_level1)))
		push_table(handle, lcm_cmd_acl_level1, sizeof(lcm_cmd_acl_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_acl_level2)))
		push_table(handle, lcm_cmd_acl_level2, sizeof(lcm_cmd_acl_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_acl_level3)))
		push_table(handle, lcm_cmd_acl_level3, sizeof(lcm_cmd_acl_level3) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----lcmacl----- level:%d\n",  levelsetting);
}
static void set_lcm_temperature(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_temperature_compensation_off)))
		push_table(handle, lcm_temperature_compensation_off, sizeof(lcm_temperature_compensation_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 5) && (sizeof(lcm_temperature_compensation_minus_five)))
		push_table(handle, lcm_temperature_compensation_minus_five, sizeof(lcm_temperature_compensation_minus_five) / sizeof(struct LCM_setting_table), 1);
      printk("vincent=----%s----- level:%d\n",__func__, levelsetting);
}
static void set_brightness_dimming(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 1) && (sizeof(brightness_dimming_on)))
		push_table(handle, brightness_dimming_on, sizeof(brightness_dimming_on) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 0) && (sizeof(brightness_dimming_off)))
		push_table(handle, brightness_dimming_off, sizeof(brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
      printk("vincent+----set_brightness_dimming----- level:%d\n",  levelsetting);
}

static void set_aor_cmd(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 1) && (sizeof(oled_aor_on)))
		push_table(handle, oled_aor_on, sizeof(oled_aor_on) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 0) && (sizeof(oled_aor_off)))
		push_table(handle, oled_aor_off, sizeof(oled_aor_off) / sizeof(struct LCM_setting_table), 1);
      printk("vincent+ lrz add----set_aor_cmd----- level:%d\n",  levelsetting);
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	if (cmdtype == 0x01) /*acl*/
		lcm_vivo_set_acl(handle, levelsetting);
	else if (cmdtype == 0x02) /*hbm*/
		lcm_vivo_set_hbm(handle, levelsetting);
	else if (cmdtype == 0x03) /*alpm*/
		lcm_vivo_set_alpm(handle, levelsetting);
	else if (cmdtype == 0x04) /*hlpm*/
		lcm_vivo_set_hlpm(handle, levelsetting);
	else if (cmdtype == 0x05) /*mipi_reg*/
		lcm_vivo_write_mipi_reg(handle, levelsetting);
	else if (cmdtype == 0x06) /*set lcm temperature compensation*/
		set_lcm_temperature(handle, levelsetting);
	else if (cmdtype == 0x07) /*set brightness dimming*/
		set_brightness_dimming(handle, levelsetting);
	else if (cmdtype == 0x08) /*set aor on or off*/
		set_aor_cmd(handle, levelsetting);
}

static void lcm_vivo_MipiCmd_LP(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	if (sizeof(lcm_cmd_acl_off))
		push_table(handle, lcm_cmd_acl_off, sizeof(lcm_cmd_acl_off) / sizeof(struct LCM_setting_table), 1);
}
/******************************************lcmacl end**************************************************/
#endif
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
/****************
truly		£º00
tianma	£º10
samsung	£º20
BOE		£º30;
****************/
	/*printk("vincent=----lcm_software_id----- =0x%x\n",  lcm_software_id);
	if (lcm_software_id == 0x5d)
		return 0x20;
	else if (lcm_software_id == 0x6a)
		return 0x21;
	else if (lcm_software_id == 0x55)
		return 0x2f;
	else
		*/
		return 0x20;
}
/*
static void oled_lpm_aod(int on)
{
	if ((on == 0) && (sizeof(oled_alpm_off)))
		push_table(NULL, oled_alpm_off, sizeof(oled_alpm_off) / sizeof(struct LCM_setting_table), 1);
	else if ((on == 1) && (sizeof(oled_alpm_on)))
		push_table(NULL, oled_alpm_on, sizeof(oled_alpm_on) / sizeof(struct LCM_setting_table), 1);
	printk("vincent=----lcmaod----- on/off:%d\n",  on);

}
*/
struct LCM_DRIVER td1901_sofeg04_fhdplus_dsi_cmd_samsung_lcm_drv = {
	.name = "td1901_sofeg04_fhdplus_dsi_cmd_samsung",
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
	//.switch_mode = lcm_switch_mode,
	//.validate_roi = lcm_validate_roi,
	.get_id	    = lcm_get_id,
	/**********lcmacl start**************
	.lcm_MipiCmd_HS  = lcm_vivo_MipiCmd_HS,
	.lcm_MipiCmd_LP = lcm_vivo_MipiCmd_LP,
	**********lcmacl end**************/
	//.aod = oled_lpm_aod,
};
