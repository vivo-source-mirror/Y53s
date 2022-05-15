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
#define LCM_DSI_CMD_MODE									1
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2340)

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
extern unsigned int is_atboot;
extern unsigned int lcm_software_id;
extern unsigned int lcm_imei_str[4];

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

/*
struct LCM_setting_table_V3 {
	unsigned char id;
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[128];
};
*/

static struct LCM_setting_table lcm_cmd_cabc_level1[] = {
		{0xB0, 1, {0x00} },
		{0x55, 1, {0x01} },
};

static struct LCM_setting_table lcm_cmd_cabc_level2[] = {
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x02} },
};
static struct LCM_setting_table lcm_cmd_cabc_level3[] = {
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x03} },
};

static struct LCM_setting_table lcm_cmd_cabc_off[] = {
	{0xB0, 1, {0x00} },
	{0x53, 1, {0x2C} },

	{0x55, 1, {0x00} },
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 50, {} },

	{0x10, 0, {} },
	{REGFLAG_DELAY, 160, {} },
};

#ifdef CONFIG_VIVO_LCM_PD1831A_CONTROL
static struct LCM_setting_table init_setting[] = {
	{0xb0, 1, {0x00} },
	{0xB8, 7, {0x45, 0x51, 0x02, 0xBC, 0x00, 0x08, 0x08} },
	{0xb0, 1, {0x03} },
	{0x51, 1, {0xFF} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x01} },
	{0x35, 1, {0x00} },
	{0xB0, 1, {0x00} },
	{0xC8, 65, {0x41, 0x00, 0x00, 0x03, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0xFB, 0x00
	, 0x00, 0x00, 0x03, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00
	, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00
	, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00
	, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF} },
	{0xB0, 1, {0x03} },

	{0x2A, 4, {0x00, 0x00, 0x04, 0x37} },
	{0x2b, 4, {0x00, 0x00, 0x09, 0x23} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#else
static struct LCM_setting_table init_setting[] = {
	{0xb0, 1, {0x00} },
	{0xB8, 7, {0x45, 0x51, 0x02, 0xBC, 0x00, 0x08, 0x08} },
	{0xb0, 1, {0x03} },
	{0x51, 1, {0xFF} },
	{0x53, 1, {0x24} },
	{0x55, 1, {0x01} },
	{0x35, 1, {0x00} },

	{0x2A, 4, {0x00, 0x00, 0x04, 0x37} },
	{0x2b, 4, {0x00, 0x00, 0x09, 0x23} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 6;
	params->dsi.vertical_frontporch = 12;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 188;
	params->dsi.horizontal_frontporch = 20;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.HS_PRPR = 10;
	params->dsi.ssc_disable  = 1;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 569;	/* this value must be in MTK suggested table */
#else
	params->dsi.PLL_CLOCK = 569;	/* this value must be in MTK suggested table */
#endif
	params->dsi.PLL_CK_CMD = 520;
	params->dsi.PLL_CK_VDO = 480;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
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
		MDELAY(2);
		lcm_enn_setting(1);
		MDELAY(10);
		panel_reset_state = 1; /*set reset state*/
	}
}

static void lcm_suspend_power(void)
{
	printk("vincent==>%s:line=%d\n", __func__, __LINE__);
	lcm_reset_setting(0);
	MDELAY(8);
	if (!is_atboot) {
		tp_reset_setting(0);
		MDELAY(5);
	}
	lcm_enn_setting(0);
	MDELAY(2);
	lcm_enp_setting(0);
	MDELAY(15);

	panel_reset_state = 0; /* clear reset state*/
}


static void lcm_reset_for_lcm(void)
{
	lcm_reset_setting(0);
	MDELAY(2);
	lcm_reset_setting(1);
	MDELAY(3);
	panel_reset_state = 1;
}

static void lcm_resume_power(void)
{
	lcm_init_power();
}

static void lcm_init(void)
{
	panel_off_state = 0;
	if (!is_atboot) {
		tp_reset_setting(1);
		MDELAY(2);
		lcm_reset_setting(1);
		MDELAY(2);
	}
	lcm_reset_setting(0);
	MDELAY(2);
	if (!is_atboot) {
		tp_reset_setting(0);
		MDELAY(2);
		tp_reset_setting(1);
		MDELAY(2);
	}
	lcm_reset_setting(1);
	MDELAY(15);
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
      printk("vincent=----lcmcabc----- level:%d\n",  levelsetting);
}
/******************************************imei start**************************************************/


static struct LCM_setting_table_V3 lcm_imei_key_num1[] = {
	{0x15,0xb0, 1, {0x04} },
	{0x39,0xf5, 2, {0x1c, 0x01} },
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 2, {} },
	{0x15,0xb0, 1, {0x04} },

};
static struct LCM_setting_table_V3 lcm_imei_key_num2[] = {
	{0x15,0xb0, 1, {0x04} },
	{0x39,0xf5, 2, {0x1c, 0x02} },
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 2, {} },
	{0x15,0xb0, 1, {0x04} },
};
extern void set_lcm(struct LCM_setting_table_V3 *para_tbl,
			unsigned int size, bool hs);
extern int read_lcm(unsigned char cmd, unsigned char *buf,
		unsigned char buf_size, bool sendhs);

 void lcm_imei_cmdq_read(void )
{
	unsigned int id, j = 0;
	unsigned char temp[4];

	set_lcm(lcm_imei_key_num1, 4, 0);
	for (id = 0; id < 2; id++) {
		read_lcm(0xfA, temp, 4, 0);
		LCM_LOGI("read 0xfa = 0x%x  first times\n", temp[0]);
		if ((temp[0] & 0xff) > 0) {
			LCM_LOGI("read 0xfa enable by %d times\n", id);

			lcm_imei_str[0] = temp[3] | (temp[2] << 8) | (temp[1] << 16) | (temp[0] << 24);
			lcm_imei_str[2]  = lcm_imei_str[0];

	            set_lcm(lcm_imei_key_num2, 4, 0);
			for (j = 0; j < 2; j++) {
				read_lcm(0xfA, temp, 4, 0);
				LCM_LOGI("read 0xfa = 0x%x second times\n", temp[0]);
				if ((temp[0] & 0xff) > 0) {
					LCM_LOGI("read 0xfa enable by %d times\n", j);
					lcm_imei_str[1] = temp[3] | (temp[2] << 8) | (temp[1] << 16) | (temp[0] << 24);
					lcm_imei_str[3]  = lcm_imei_str[1];
					break;

				}
			}
			break;
		}
	}
	printk("first imei read %d times, second read %d\n", id, j);
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	if (cmdtype == 0x81) /*cabc*/
		lcm_vivo_set_cabc(handle, levelsetting);
}
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_software_id == 0x56)
		return 0x40;
	else if (lcm_software_id == 0x68)
		return 0x41;
	else
		return 0x4f;
}

struct LCM_DRIVER pd1831_td4330_fhdplus_dsi_vdo_djn_lcm_drv = {
	.name = "pd1831_td4330_fhdplus_dsi_vdo_djn",
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
	.lcm_reset = lcm_reset_for_lcm,
	.get_project_name = "PD1831",
};
