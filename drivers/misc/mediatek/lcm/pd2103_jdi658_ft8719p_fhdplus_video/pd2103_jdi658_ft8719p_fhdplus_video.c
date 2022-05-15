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
#include "mtk_boot_common.h"

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
extern void lm3697_brightness_set(int value);
extern unsigned int islcmconnected;
extern unsigned int phone_shutdown_state;
/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0


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


struct LCD_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#include "../../../../../vivo/system/display/vivosrc/fb/pd2103_jdi658_ft8719p_fhdplus_video_cmd.h"

extern unsigned int panel_reset_state;
extern unsigned int panel_off_state;
extern unsigned int lcm_software_id;
extern unsigned int bl_brightness_hal;
extern unsigned int cabc_state_for_upper_set;
extern unsigned int lcm_cabc_status;
extern unsigned int bl_lvl;

static void push_table(void *cmdq, struct LCD_setting_table *table,
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
			dsi_set_cmdq_V2(cmd, table[i].count-1, &table[i].para_list[1], force_update);
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
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density            = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
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

	params->dsi.vertical_sync_active = VSA;
	params->dsi.vertical_backporch = VBP;
	params->dsi.vertical_frontporch = VFP;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = HSA;
	params->dsi.horizontal_backporch = HBP;
	params->dsi.horizontal_frontporch = HFP;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
	params->dsi.PLL_CLOCK = VIVO_PLL_CLOCK;
	params->dsi.data_rate = VIVO_DATA_RATE;	/* this value must be in MTK suggested table */
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x53;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

#ifdef NT35695_LANESWAP
	params->dsi.lane_swap_en = 1;

	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_CK;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_2;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_0;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_1;
	params->dsi.lane_swap[MIPITX_PHY_PORT_0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_1;
#endif

#ifdef MTK_ROUND_CORNER_SUPPORT
	params->round_corner_params.round_corner_en = 1;
	params->round_corner_params.full_content = 0;
	params->round_corner_params.w = ROUND_CORNER_W;
	params->round_corner_params.h = ROUND_CORNER_H;
	params->round_corner_params.lt_addr = left_top;
	params->round_corner_params.lb_addr = left_bottom;
	params->round_corner_params.rt_addr = right_top;
	params->round_corner_params.rb_addr = right_bottom;
#endif
}

static void lcm_init_power(void)
{
	LCM_INFO("panel_reset_state = %d, lcm_software_id = 0x%x\n", panel_reset_state, lcm_software_id);
	if (panel_reset_state == 0) {
		lcm_enp_setting(1);
		MDELAY(6);
		lcm_enn_setting(1);
		MDELAY(10);
		lcm_bias_set_avdd_n_avee(55);
		MDELAY(3);
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
	LCM_INFO("[LCM]lcm_suspend_power ft8719p- vddi keep high level\n");
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
	MDELAY(6);
	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(6);
	lcm_reset_setting(1);
	MDELAY(15);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCD_setting_table), 1);
	LCM_INFO("----lcm_init----lcm mode = cmd mode :%d----\n", lcm_dsi_mode);
}

static void lcm_suspend(void)
{
	LCM_INFO("lcm_software_id = 0x%x, islcmconnected = %d boot mode = %d \n", lcm_software_id,islcmconnected,get_boot_mode());
	if(get_boot_mode() != KERNEL_POWER_OFF_CHARGING_BOOT)
		lcm_bkg_setting(0);
	MDELAY(5);
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCD_setting_table), 1);
	panel_off_state = 1;
	if(islcmconnected == 0){
		lcm_enn_setting(0);
		MDELAY(5);
		lcm_enp_setting(0);
	}
}

static void lcm_resume(void)
{
	lcm_bkg_setting(1);
	MDELAY(5);
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
	static unsigned int last_brightness_level = 1600;
	LCM_INFO("LastBacklight = %d, level=%d\n", last_brightness_level, level);
	if (level && !last_brightness_level) {
		LCD_INFO("set all pixel on\n");
		push_table(NULL, lcm_all_pixel_on_setting, sizeof(lcm_all_pixel_on_setting) / sizeof(struct LCD_setting_table), 1);
	} else if (!level && last_brightness_level){
		LCD_INFO("set all pixel off\n");
		push_table(NULL, lcm_all_pixel_off_setting, sizeof(lcm_all_pixel_off_setting) / sizeof(struct LCD_setting_table), 1);
		MDELAY(40);
	}

	if(level > 2033)
		level = 2033;
	last_brightness_level = level;
	bl_lvl = level;
	bl_brightness_hal = level;

	lm3697_brightness_set(level);
}
static void lcm_reset_for_touch(void)
{
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(5);
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(12);
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned char levelsetting)
{
	
}
static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	if (lcm_software_id == 0x78)
		return 0x41;
	else
		return 0xFF;
}

struct LCM_DRIVER pd2103_jdi658_ft8719p_fhdplus_lcm_drv = {
	.name = "pd2103_jdi658_ft8719p_fhdplus_drv",
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
