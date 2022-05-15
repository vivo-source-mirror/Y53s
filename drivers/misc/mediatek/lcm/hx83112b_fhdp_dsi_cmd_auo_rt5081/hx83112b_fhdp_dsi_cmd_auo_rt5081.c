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
#  include <linux/string.h>
#  include <linux/kernel.h>
#endif

#include "lcm_drv.h"
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/kernel.h>

#ifdef BUILD_LK
#  include <platform/upmu_common.h>
#  include <platform/mt_gpio.h>
#  include <platform/mt_i2c.h>
#  include <platform/mt_pmic.h>
#  include <string.h>
#elif defined(BUILD_UBOOT)
#  include <asm/arch/mt_gpio.h>
#endif

#ifdef BUILD_LK
#  define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#  define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#  define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#  define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
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
#define read_reg(cmd)	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#  include <linux/kernel.h>
#  include <linux/module.h>
#  include <linux/fs.h>
#  include <linux/slab.h>
#  include <linux/init.h>
#  include <linux/list.h>
#  include <linux/i2c.h>
#  include <linux/irq.h>
#  include <linux/uaccess.h>
#  include <linux/interrupt.h>
#  include <linux/io.h>
#  include <linux/platform_device.h>
#endif

#define LCM_DSI_CMD_MODE		0
#define FRAME_WIDTH			(1080)
#define FRAME_HEIGHT			(2340)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH		(69500)
#define LCM_PHYSICAL_HEIGHT		(150580)
#define LCM_DENSITY			(480)

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE		0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF

#define LCM_ID_HX83112B 0x83

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;
static int regulator_inited;
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {

};

static struct LCM_setting_table init_setting_cmd[] = {
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00, 0x1D, 0x00, 0x35, 0x00, 0x5C, 0x00, 0x7C, 0x00, 0x97, 0x00, 0xAE, 0x00, 0xC3, 0x00, 0xD5} },
	{0xB1, 16, {0x00, 0xE6, 0x01, 0x1E, 0x01, 0x46, 0x01, 0x86, 0x01, 0xB4, 0x01, 0xFD, 0x02, 0x34, 0x02, 0x35} },
	{0xB2, 16, {0x02, 0x6C, 0x02, 0xA9, 0x02, 0xD3, 0x03, 0x03, 0x03, 0x27, 0x03, 0x4E, 0x03, 0x5E, 0x03, 0x6C} },
	{0xB3, 14, {0x03, 0x7E, 0x03, 0x94, 0x03, 0xAF, 0x03, 0xC8, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00 } },
	{0xB4, 16, {0x00, 0x77, 0x00, 0x86, 0x00, 0xA0, 0x00, 0xB7, 0x00, 0xCB, 0x00, 0xDC, 0x00, 0xED, 0x00, 0xFB} },
	{0xB5, 16, {0x01, 0x09, 0x01, 0x37, 0x01, 0x5A, 0x01, 0x94, 0x01, 0xBE, 0x02, 0x03, 0x02, 0x39, 0x02, 0x3A} },
	{0xB6, 16, {0x02, 0x70, 0x02, 0xAC, 0x02, 0xD6, 0x03, 0x06, 0x03, 0x2A, 0x03, 0x51, 0x03, 0x61, 0x03, 0x6F} },
	{0xB7, 14, {0x03, 0x80, 0x03, 0x96, 0x03, 0x9F, 0x03, 0xC7, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00 } },
	{0xB8, 16, {0x00, 0x00, 0x00, 0x28, 0x00, 0x57, 0x00, 0x7B, 0x00, 0x98, 0x00, 0xB1, 0x00, 0xC7, 0x00, 0xDA} },
	{0xB9, 16, {0x00, 0xEB, 0x01, 0x24, 0x01, 0x4C, 0x01, 0x8C, 0x01, 0xB9, 0x02, 0x01, 0x02, 0x38, 0x02, 0x39} },
	{0xBA, 16, {0x02, 0x6F, 0x02, 0xAB, 0x02, 0xD5, 0x03, 0x06, 0x03, 0x2A, 0x03, 0x50, 0x03, 0x60, 0x03, 0x70} },
	{0xBB, 14, {0x03, 0x83, 0x03, 0x9B, 0x03, 0xB6, 0x03, 0xCA, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00 } },
	{0xFF, 1, {0x21} },
	{0xFB, 1, {0x01} },
	{0xB0, 16, {0x00, 0x15, 0x00, 0x2D, 0x00, 0x54, 0x00, 0x74, 0x00, 0x8F, 0x00, 0xA6, 0x00, 0xBB, 0x00, 0xCD} },
	{0xB1, 16, {0x00, 0xDE, 0x01, 0x16, 0x01, 0x3E, 0x01, 0x7E, 0x01, 0xAC, 0x01, 0xF5, 0x02, 0x2C, 0x02, 0x2D} },
	{0xB2, 16, {0x02, 0x64, 0x02, 0xA1, 0x02, 0xCB, 0x02, 0xFB, 0x03, 0x1F, 0x03, 0x46, 0x03, 0x56, 0x03, 0x64} },
	{0xB3, 14, {0x03, 0x76, 0x03, 0x8C, 0x03, 0xA7, 0x03, 0xC0, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00} },
	{0xB4, 16, {0x00, 0x6F, 0x00, 0x7E, 0x00, 0x98, 0x00, 0xAF, 0x00, 0xC3, 0x00, 0xD4, 0x00, 0xE5, 0x00, 0xF3} },
	{0xB5, 16, {0x01, 0x01, 0x01, 0x2F, 0x01, 0x52, 0x01, 0x8C, 0x01, 0xB6, 0x01, 0xFB, 0x02, 0x31, 0x02, 0x32} },
	{0xB6, 16, {0x02, 0x68, 0x02, 0xA4, 0x02, 0xCE, 0x02, 0xFE, 0x03, 0x22, 0x03, 0x49, 0x03, 0x59, 0x03, 0x67} },
	{0xB7, 14, {0x03, 0x78, 0x03, 0x8E, 0x03, 0x97, 0x03, 0xBF, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00} },
	{0xB8, 16, {0x00, 0x00, 0x00, 0x20, 0x00, 0x4F, 0x00, 0x73, 0x00, 0x90, 0x00, 0xA9, 0x00, 0xBF, 0x00, 0xD2} },
	{0xB9, 16, {0x00, 0xE3, 0x01, 0x1C, 0x01, 0x44, 0x01, 0x84, 0x01, 0xB1, 0x01, 0xF9, 0x02, 0x30, 0x02, 0x31} },
	{0xBA, 16, {0x02, 0x67, 0x02, 0xA3, 0x02, 0xCD, 0x02, 0xFE, 0x03, 0x22, 0x03, 0x48, 0x03, 0x58, 0x03, 0x68} },
	{0xBB, 14, {0x03, 0x7B, 0x03, 0x93, 0x03, 0xAE, 0x03, 0xC2, 0x03, 0xD2, 0x03, 0xD4, 0x00, 0x00} },

	{0xFF, 1, {0x25} },
	{0xFB, 1, {0x01} },
	{0x05, 1, {0x04} },
	{0xFF, 1, {0x20} },
	{0xFB, 1, {0x01} },
	{0x62, 1, {0xB0} },
	{0xFF, 1, {0x23} },
	{0xFB, 1, {0x01} },
	{0x08, 1, {0x04} },
	{0x11, 1, {0x01} },
	{0x12, 1, {0x7E} },
	{0x15, 1, {0x6E} },
	{0x16, 1, {0x0B} },
	{0x45, 1, {0xFF} },
	{0x46, 1, {0xF6} },
	{0x47, 1, {0xED} },
	{0x48, 1, {0xE6} },
	{0x49, 1, {0xDF} },
	{0x4A, 1, {0xD8} },
	{0x4B, 1, {0xD3} },
	{0x4C, 1, {0xCE} },
	{0x4D, 1, {0xC9} },
	{0x4E, 1, {0xC4} },
	{0x4F, 1, {0xC1} },
	{0x50, 1, {0xBE} },
	{0x51, 1, {0xBB} },
	{0x52, 1, {0xB8} },
	{0x53, 1, {0xB6} },
	{0x54, 1, {0xB5} },
	{0x58, 1, {0xFF} },
	{0x59, 1, {0xF6} },
	{0x5A, 1, {0xED} },
	{0x5B, 1, {0xE6} },
	{0x5C, 1, {0xDF} },
	{0x5D, 1, {0xD8} },
	{0x5E, 1, {0xD3} },
	{0x5F, 1, {0xCE} },
	{0x60, 1, {0xC9} },
	{0x61, 1, {0xC4} },
	{0x62, 1, {0xC1} },
	{0x63, 1, {0xBE} },
	{0x64, 1, {0xBB} },
	{0x65, 1, {0xB8} },
	{0x66, 1, {0xB6} },
	{0x67, 1, {0xB5} },
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

static struct LCM_setting_table
__maybe_unused lcm_deep_sleep_mode_in_setting[] = {
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 150, {} },
};

static struct LCM_setting_table __maybe_unused lcm_sleep_out_setting[] = {
	{0x11, 1, {0x00} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 1, {0x00} },
	{REGFLAG_DELAY, 50, {} },
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
		       unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

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
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
					 table[i].para_list, force_update);
			break;
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
	params->physical_width = LCM_PHYSICAL_WIDTH / 1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT / 1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	LCM_LOGI("%s: lcm_dsi_mode %d\n", __func__, lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

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
	params->dsi.vertical_frontporch_for_low_power = 620;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 112;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable = 1; */
	params->dsi.PLL_CLOCK = 563;	/* this value must be in MTK suggested table */
	params->dsi.data_rate = 1126;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

	/* for ARR 2.0 */
	params->max_refresh_rate = 60;
	params->min_refresh_rate = 45;

}
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
int lcm_bias_regulator_init(void)
{
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_info("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_info("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

int lcm_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5500000, 5500000);
	if (ret < 0)
		pr_info("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5500000, 5500000);
	if (ret < 0)
		pr_info("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

#if 0
	/* get voltage */
	ret = mtk_regulator_get_voltage(&disp_bias_pos);
	if (ret < 0)
		pr_info("get voltage disp_bias_pos fail\n");
	pr_debug("pos voltage = %d\n", ret);

	ret = mtk_regulator_get_voltage(&disp_bias_neg);
	if (ret < 0)
		pr_info("get voltage disp_bias_neg fail\n");
	pr_debug("neg voltage = %d\n", ret);
#endif
	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_info("enable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_info("enable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}


int lcm_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_info("disable regulator disp_bias_neg fail, ret = %d\n",
			ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_info("disable regulator disp_bias_pos fail, ret = %d\n",
			ret);
	retval |= ret;

	return retval;
}

#else
int lcm_bias_regulator_init(void)
{
	return 0;
}

int lcm_bias_enable(void)
{
	return 0;
}

int lcm_bias_disable(void)
{
	return 0;
}
#endif

/* turn on gate ic & control voltage to 5.5V */
/* equle display_bais_enable ,mt6768 need +/-5.5V */
static void lcm_init_power(void)
{
	if (0)
	lcm_bias_enable();
}

static void lcm_suspend_power(void)
{
	SET_RESET_PIN(0);
	if (0)
	lcm_bias_disable();
}

/* turn on gate ic & control voltage to 5.5V */
static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	lcm_init_power();
}

static void lcm_init(void)
{
/*
	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
*/
	push_table(NULL, init_setting_cmd, ARRAY_SIZE(init_setting_cmd), 1);
	LCM_LOGI("hx83112b_fhdp----tps6132----lcm mode = vdo mode :%d----\n",
		 lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	if (0) {
	push_table(NULL, lcm_suspend_setting,
		   ARRAY_SIZE(lcm_suspend_setting), 1);
	}
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int id[3] = {0x83, 0x11, 0x2B};
	unsigned int data_array[3];
	unsigned char read_buf[3];

	data_array[0] = 0x00033700; /* set max return size = 3 */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, read_buf, 3); /* read lcm id */

	LCM_LOGI("ATA read = 0x%x, 0x%x, 0x%x\n",
		 read_buf[0], read_buf[1], read_buf[2]);

	if ((read_buf[0] == id[0]) &&
	    (read_buf[1] == id[1]) &&
	    (read_buf[2] == id[2]))
		ret = 1;
	else
		ret = 0;

	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	LCM_LOGI("%s,hx83112b backlight: level = %d\n", __func__, level);
	level = 2047;
	bl_level[0].para_list[0] = level;

	push_table(handle, bl_level, ARRAY_SIZE(bl_level), 1);
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width,
	unsigned int height)
{
	unsigned int y0 = y;
	unsigned int y1 = y0 + height - 1;
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

#ifdef LCM_SET_DISPLAY_ON_DELAY
	lcm_set_display_on();
#endif

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x30;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00123909;
	dsi_set_cmdq(data_array, 1, 0);
}

/* partial update restrictions:
 * 1. roi width must be 1080 (full lcm width)
 */
static void lcm_validate_roi(int *x, int *y, int *width, int *height)
{
	unsigned int x1, w;

	x1 = 0;
	w = FRAME_WIDTH;

	*x = x1;
	*width = w;
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[1];
	unsigned int array[16];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);

	SET_RESET_PIN(1);
	MDELAY(20);

	array[0] = 0x00013700;  /* read id return 1byte */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xDA, buffer, 1);
	id = buffer[0];     /* we only need ID */

	LCM_LOGI("%s,hx83112b id = 0x%08x\n", __func__, id);

	if (id == LCM_ID_HX83112B)
		return 1;
	else
		return 0;

}

struct LCM_DRIVER hx83112b_fhdp_dsi_cmd_auo_rt5081_lcm_drv = {
	.name = "hx83112b_fhdp_dsi_cmd_auo_rt5081_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.compare_id = lcm_compare_id,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.validate_roi = lcm_validate_roi,
};

