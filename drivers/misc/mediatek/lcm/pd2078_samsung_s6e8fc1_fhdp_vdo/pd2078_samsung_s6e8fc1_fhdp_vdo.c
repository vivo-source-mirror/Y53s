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

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/upmu_hw.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#else
	#include <linux/string.h>
	#include <linux/kernel.h>
	#include <linux/module.h>
	#include <linux/fs.h>
	#include <linux/init.h>
	#include <linux/platform_device.h>
	#include "mtkfb.h"
	#include "vivo_smart_aod.h"
//	#include <mt-plat/mtk_gpio_core.h>
//	#include "mt-plat/upmu_common.h"
//	#include "mt-plat/mtk_gpio.h"
	#include "disp_dts_gpio.h"
//	#include <linux/i2c.h> /*lcm power is provided by i2c*/
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../../../gpu/drm/mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern_PD2078.h"
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Define Print Log Level
// ---------------------------------------------------------------------------
#ifdef BUILD_LK
	#define LCM_LOGI(string, args...)   dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
	#define LCM_LOGD(string, args...)   dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
	#define LCM_LOGE(string, args...)   dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#else
	#define LCM_LOGI(fmt, args...)      pr_notice("[KERNEL/"LOG_TAG"]"fmt, ##args)
	#define LCM_LOGD(fmt, args...)      pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
	#define LCM_LOGE(fmt, args...)      pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE                0

#define FRAME_WIDTH                     (1080)
#define FRAME_HEIGHT                    (2400)
/* physical size in um */

#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW		0xFFFE
#define REGFLAG_RESET_HIGH		0xFFFF


#ifndef TRUE
 #define TRUE 1
#endif

#ifndef FALSE
 #define FALSE 0
#endif

/**
 *  Local Variables
 */
static struct LCM_UTIL_FUNCS lcm_util = {0};

#define MDELAY(n)                       (lcm_util.mdelay(n))
#define UDELAY(n)                       (lcm_util.udelay(n))
#define SET_RESET_PIN(v)                (lcm_util.set_reset_pin((v)))

/**
 * Local Functions
 */
#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
		lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define lcm_reset_setting(cmd) \
		lcm_util.lcm_reset_setting(cmd)

#define lcm_vddr_setting(cmd) \
		lcm_util.lcm_enp_setting(cmd)
#define lcm_vddi_setting(cmd) \
		lcm_util.lcm_enn_setting(cmd)
#define dsi_set_cmdq_V4(para_tbl, size, hs) \
		lcm_util.dsi_set_cmdq_V4(para_tbl, size, hs)
		
#define dfps_dsi_send_cmd( \
		cmdq, cmd, count, para_list, force_update, sendmode) \
		lcm_util.dsi_dynfps_send_cmd( \
		cmdq, cmd, count, para_list, force_update, sendmode)

#define LCM_PHYSICAL_WIDTH									(67000)
#define LCM_PHYSICAL_HEIGHT									(149000)
#define LCM_DENSITY											(480)
//#define REGFLAG_DELAY		0xFE
//#define REGFLAG_END_OF_TABLE	0xFF /* END OF REGISTERS MARKER */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};
#define cmdq_v3 0
#if 1
static int oled_hbm_on;
extern unsigned int lcm_ldo_vision;
extern atomic_t oled_hbm_status;
extern atomic_t doze_mode_state;
extern unsigned int bl_brightness_hal;
extern unsigned int current_backlight;
extern unsigned int lcm_software_id;
extern unsigned int lcm_especial_id;
extern int oled_dimming_enable;
extern int oled_weakup_hbmon;
extern int lcmpr_debug_enabled;
extern int vivo_colour_gamut;
extern struct aod_set_data g_aod_data;
extern atomic_t display_on_control;
extern atomic_t aod_control;
extern int oled_ud_flag;
extern bool set_hbm_on_aod_exit;
extern int notify_udfp_cyan_make_sure_delay;
extern struct mutex hbm_lock;
#endif

static void dsi_panel_update_seed_crc(void *handle, unsigned char levelsetting);

struct regulator *lcm_vci;
static int hbm_wait;
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 15, {} },

	{0x10, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 160, {} },
};

static struct LCM_setting_table_V3 lcm_suspend_setting_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x9F, 0x02, {0xA5, 0xA5} },
	{REGFLAG_ESCAPE_ID, 0x28, 0x01, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 15, {} },

	{REGFLAG_ESCAPE_ID, 0x10, 0x01, {0x00} },
	{REGFLAG_ESCAPE_ID, 0x9F, 0x02, {0x5A, 0x5A} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 160, {} },
};

static struct LCM_setting_table cmd_init_pre_on[] = {
	{0x9F,2,{0xA5,0xA5}},
	{0x11,1,{0x00}},
	{0x35,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},
	{REGFLAG_DELAY, 22, {} },

	{0xF0,2,{0x5A,0x5A}},
	{0xF2,1,{0x8F}},
	{0xEA,1,{0x40}},
	{0xF0,2,{0xA5,0xA5}},

	{0x51,2,{0x00,0x00}},
	{0x53,1,{0x20}},
	{0x55,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},

	{REGFLAG_DELAY, 102, {} },
	{0x9F,2,{0xA5,0xA5}},
	{0x29,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},
	{0xF0,2,{0x5A,0x5A}},
	{0x9F,2,{0xA5,0xA5}},
	{0xB0,2,{0xB1,0xB5}},
	{0xB5,1,{0x48}},

	{0xB0,2,{0x01,0xB1}},
	{0xB1,21,{0xFF,0x00,0x04,0x10,0xFF,0x00,0x07,0x07,0xFF,0x15,0xFF,0xE2,0xFF,0x04,0xEc,0xFF,0xF7,0x00,0xFF,0xFF,0xFF}},

	{0x80,1,{0x90}},
	{0xB0,2,{0x55,0xB1}},
	{0xB1,1,{0x80}},
	{0xB1,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},
	{0xF0,2,{0xA5,0xA5}},
};

static struct LCM_setting_table cmd_brightness_dimming_off[] = {
	{0x53, 1, {0x20} },
};
static struct LCM_setting_table cmd_brightness_dimming_aod[] = {
	{0x51, 2, {0x03,0xE9} },
};

static struct LCM_setting_table cmd_brightness_dimming_on[] = {
	{0x53, 1, {0x28} },
};

static struct LCM_setting_table cmd_elvss_dimming_on[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB3, 6, {0x11, 0x4B, 0x28, 0x09, 0x80, 0x87} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table cmd_elvss_dimming_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB3, 6, {0x11, 0x4B, 0x28, 0x09, 0x80, 0x07} },
	{0xF0, 2, {0xA5, 0xA5} },
};


/* samsung OLED CRC mode P3 */
static struct LCM_setting_table oled_seed_crc_p3[] = {
	{0xF0,2,{0x5A,0x5A} },
	{0xB1,22,{0x00,0xFF,0x00,0x04,0x10,0xFF,0x00,0x07,0x07,0xFF,0x15,0xFF,0xE2,0xFF,0x04,0xEc,0xFF,0xF7,0x00,0xFF,0xFF,0xFF} },
	{0xF0,2,{0xA5,0xA5} },
};

/* samsung OLED CRC mode SRGB*/
static struct LCM_setting_table oled_seed_crc_srgb[] = {
	{0xF0,2,{0x5A,0x5A} },
	{0xB1,22,{0x00,0xB8,0x01,0x05,0x48,0xE2,0x16,0x06,0x09,0xBA,0x5B,0xFF,0xE5,0xCF,0x11,0xCE,0xF6,0xEE,0x1E,0xFF,0xFB,0xE0} },
	{0xF0,2,{0xA5,0xA5} },
};

/* samsung OLED CRC mode off*/
static struct LCM_setting_table oled_seed_crc_off[] = {
	{0xF0,2,{0x5A,0x5A} },
	{0xB1,2,{0x01} },
	{0xF0,2,{0xA5,0xA5} },
};

static struct LCM_setting_table cmd_hbm_off[] = {
	{0x53, 1, {0x28} },
	{0x51, 2, {0x07, 0xff} },
};

static struct LCM_setting_table cmd_hbm_level1[] = {
	{0x53, 1, {0xE8} },
	{0x51, 2, {0x01, 0x7B} },
};
static struct LCM_setting_table cmd_hbm_level2[] = {
	{0x53, 1, {0xE8} },
	{0x51, 2, {0x02, 0x83} },
};

static struct LCM_setting_table cmd_hbm_level3[] = {
	{0x53, 1, {0xE8} },
	{0x51, 2, {0x02, 0x83} },
};

static struct LCM_setting_table cmd_hbm_ud_on[] = {
	{0x53, 1, {0xE0} },
	{0x51, 2, {0x03, 0x29} },   /*600 nit*/
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x07, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
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


/**
 *  LCM Driver Implementations
 */

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}



static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	//params->dsi.IsCphy = 0;

	params->width  = FRAME_WIDTH;
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
	params->dsi.switch_mode_enable = 1;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM		= LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	/* Video mode setting */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.packet_size = 256;
	params->dsi.ssc_disable = 1;
	/* params->dsi.ssc_range = 3; */

	params->dsi.vertical_sync_active	= 2;
	params->dsi.vertical_backporch		= 9;
	params->dsi.vertical_frontporch		= 21;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 16;
	params->dsi.horizontal_backporch	  = 46;
	params->dsi.horizontal_frontporch	  = 48;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

	params->dsi.HS_TRAIL = 16;
	/* Bit rate calculation */
	params->dsi.PLL_CLOCK = 554;
	/* this value must be in MTK suggested table */
	params->dsi.data_rate = 1108;


	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 1;
	params->corner_pattern_height = ROUND_CORNER_H_TOP;
	params->corner_pattern_height_bot = ROUND_CORNER_H_BOT;
	params->corner_pattern_tp_size = sizeof(top_rc_pattern);
	params->corner_pattern_lt_addr = (void *)top_rc_pattern;
#endif

}

int lcm_vci_enable(bool enable){
	int ret;
	static bool is_initvci = 1;
	if(is_initvci){
		lcm_vci = regulator_get(NULL, "lcm_vci");
		is_initvci = 0;
		if (IS_ERR(lcm_vci)){
			ret = PTR_ERR(lcm_vci);
			LCM_ERR("get lcm_vci fail, error: %d\n", ret);
		}

	}
	if(enable){
		ret = regulator_set_voltage(lcm_vci, 3100000, 3100000);
		if (ret < 0)
			LCM_ERR("set regulator lcm_vci fail, ret = %d\n", ret);

		ret = regulator_enable(lcm_vci);
		if (ret < 0)
			LCM_ERR("enable regulator lcm_vci fail, ret = %d\n", ret);
	}else{
		ret = regulator_disable(lcm_vci);
		if(ret < 0)
			LCM_ERR("disable regulator lcm_vci fail, ret = %d\n", ret);
	}
	LCM_INFO("\n");
	return ret;
}

static void aod_area_init(struct aod_set_data *aod_data);
static void lcm_init_power(void)
{
	static bool is_initaod = 1;

	if (is_initaod) {
		aod_area_init(&g_aod_data);
		is_initaod = 0;
	}

	/*vddi*/
	lcm_vddi_setting(1);
	MDELAY(1);

	/*vddr*/
	lcm_vddr_setting(1);
	MDELAY(12);
	lcm_vci_enable(1);
	MDELAY(12);
	lcm_reset_setting(0);
	MDELAY(1);
	lcm_reset_setting(1);
	MDELAY(8);

	LCM_INFO("\n");
}

static void lcm_suspend_power(void)
{
	MDELAY(12);
	/*reset low*/
	lcm_reset_setting(0);
	MDELAY(2);
	/*vddr*/
	lcm_vddr_setting(0);
	MDELAY(12);
	lcm_vddi_setting(0);
	MDELAY(1);
	lcm_vci_enable(0);
	LCM_INFO("\n");
}

static void lcm_resume_power(void)
{
	//lcm_reset_setting(1);
	lcm_init_power();
}

static void lcm_init(void)
{
	lcm_reset_setting(1);
	MDELAY(2);
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_reset_setting(1);
	MDELAY(15);

	atomic_set(&display_on_control, 1);
	push_table(NULL, cmd_init_pre_on, sizeof(cmd_init_pre_on) /
		   sizeof(struct LCM_setting_table), 1);

	if(set_hbm_on_aod_exit){
		push_table(NULL, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(NULL, cmd_elvss_dimming_off, sizeof(cmd_elvss_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(NULL, cmd_hbm_ud_on, sizeof(cmd_hbm_ud_on) / sizeof(struct LCM_setting_table), 1);
		set_hbm_on_aod_exit = false;
		oled_hbm_on = 1;
		atomic_set(&oled_hbm_status, HBM_UDFP_AOD_MODE_ON);
		notify_udfp_cyan_make_sure_delay = 90;
		LCM_INFO("enable hbm in lcm init\n");
	}
	if (vivo_colour_gamut > 0)
		dsi_panel_update_seed_crc(NULL, vivo_colour_gamut);
	atomic_set(&doze_mode_state,LCM_PANEL_ON);
	LCM_INFO("cmd_init_pre_on & set display_on_control 1\n");
}

static void lcm_enable(void *handle)
{
	LCM_INFO("\n");
}

static void lcm_suspend(void)
{
	if (0)
		push_table(NULL, lcm_suspend_setting,
			sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
	atomic_set(&doze_mode_state,LCM_PANEL_OFF);
	MDELAY(10);
	//lcm_reset_setting(0);
	LCM_INFO("\n");
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
static struct LCM_setting_table cmd_aod_on[] = {
	{0xF0,2,{0x5A,0x5A}},
	{0xFC,2,{0x5A,0x5A}},
	/* Internal VDO Packet generation enable*/
	{0xB0,2,{0x14,0xFE}},
	{0xFE,1,{0x12}},
	/* MIPI Mode cmd */
	{0xF2,1,{0x03}},
	{0xFC,2,{0xA5,0xA5}},
	{0xF0,2,{0xA5,0xA5}},

	{REGFLAG_DELAY, 10, {}},

	/* TE vsync ON */
	{0x9F,2,{0xA5,0xA5}},
	{0x35,1,{0x00}},
	{0x9F,2,{0x5A,0x5A}},

	{REGFLAG_DELAY, 10, {}},

	/*AOD IP Setting*/
	{0xF0,2,{0x5A,0x5A}},
	{0xB0,2,{0x03,0xC2}},
	{0xC2,1,{0x04}},
	{0xF0,2,{0xA5,0xA5}},

	/* AOD AMP ON */
	{0xFC,2,{0x5A,0x5A}},
	{0xB0,2,{0x06,0xFD}},
	{0xFD,1,{0x85}},
	{0xFC,2,{0xA5,0xA5}},

	/* set AOD HLPM_ON */
	{0xF0,2,{0x5A,0x5A}},
	{0xB0,2,{0x0B,0xF4}},
	{0xF4,1,{0x8C}},
	{0xB0,1,{0xB3}},
	{0xB8,4,{0x04,0x81,0x20,0x67}},
	{0xB0,2,{0x0E,0xF4}},
	{0xF4,1,{0x7C}},
	{0x53,1,{0x22}},
	{0xF0,2,{0xA5,0xA5}},

	/* Internal VDO Packet generation enable*/
	{0xF0,2,{0x5A,0x5A}},
	{0xFC,2,{0x5A,0x5A}},
	{0xB0,2,{0x14,0xFE}},
	{0xFE,1,{0x10}},
	{0xF0,2,{0xA5,0xA5}},
	{0xFC,2,{0xA5,0xA5}},

	{REGFLAG_DELAY, 34, {}},
};
#endif

static struct LCM_setting_table_V3 cmd_aod_on_v3[] = {
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0x5A,0x5A}},
	/* Internal VDO Packet generation enable*/
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x14,0xFE}},
	{REGFLAG_ESCAPE_ID,0xFE,0x01,{0x12}},
	/* MIPI Mode cmd */
	{REGFLAG_ESCAPE_ID,0xF2,0x01,{0x03}},
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0xA5,0xA5}},

	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },

	/* TE vsync ON */
	{REGFLAG_ESCAPE_ID,0x9F,0x02,{0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID,0x35,0x01,{0x00}},
	{REGFLAG_ESCAPE_ID,0x9F,0x02,{0x5A,0x5A}},

	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },

	/*AOD IP Setting*/
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x03,0xC2}},
	{REGFLAG_ESCAPE_ID,0xC2,0x01,{0x04}},
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0xA5,0xA5}},

	/* AOD AMP ON */
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x06,0xFD}},
	{REGFLAG_ESCAPE_ID,0xFD,0x01,{0x85}},
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0xA5,0xA5}},

	/* set AOD HLPM_ON */
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x0B,0xF4}},
	{REGFLAG_ESCAPE_ID,0xF4,0x01,{0x8C}},
	{REGFLAG_ESCAPE_ID,0xB0,0x01,{0xB3}},
	{REGFLAG_ESCAPE_ID,0xB8,0x04,{0x04,0x81,0x20,0x67}},
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x0E,0xF4}},
	{REGFLAG_ESCAPE_ID,0xF4,0x01,{0x7C}},
	{REGFLAG_ESCAPE_ID,0x53,0x01,{0x22}},
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0xA5,0xA5}},

	/* Internal VDO Packet generation enable*/
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID,0xB0,0x02,{0x14,0xFE}},
	{REGFLAG_ESCAPE_ID,0xFE,0x01,{0x10}},
	{REGFLAG_ESCAPE_ID,0xF0,0x02,{0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID,0xFC,0x02,{0xA5,0xA5}},

	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },
};

static struct LCM_setting_table aod_display_on[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 10, {} },
};

static struct LCM_setting_table_V3 aod_display_on_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x9F,0x02,{0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID, 0x29, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, 0x9F,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },
};

static struct LCM_setting_table_V3 aod_display_off_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x9F, 0x02, {0xA5, 0xA5} },
	{REGFLAG_ESCAPE_ID, 0x28, 0x01, {0x00}},
	{REGFLAG_ESCAPE_ID, 0x9F, 0x02, {0x5A,0x5A}},
};

static struct LCM_setting_table_V3 cmd_aod_off_v3[] = {
	/* AOD Mode off Setting */
	{REGFLAG_ESCAPE_ID, 0x9F,0x02,{0xA5, 0xA5}},
	{REGFLAG_ESCAPE_ID, 0x28,0x01,{0x00}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 35, {}},
	{REGFLAG_ESCAPE_ID, 0x53,0x01,{0x20}},

	/* AOD AMP OFF */
	{REGFLAG_ESCAPE_ID, 0xFC,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xB0,0x02,{0x06,0xFD}},
	{REGFLAG_ESCAPE_ID, 0xFD,0x01,{0x05}},
	{REGFLAG_ESCAPE_ID, 0xFC,0x02,{0xA5,0xA5}},

	/* MIPI Video mode*/
	{REGFLAG_ESCAPE_ID, 0xF0,0x02,{0x5A,0x5A}},
	{REGFLAG_ESCAPE_ID, 0xF2,0x01,{0x0F}},
	{REGFLAG_ESCAPE_ID, 0xF0,0x02,{0xA5,0xA5}},
	{REGFLAG_ESCAPE_ID, 0x9F,0x02,{0x5A,0x5A}},
};

static struct LCM_setting_table_V3 cmd_doze_area_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x9F,0x02,{0xA5,0xA5}},

	/* (x,y,w*h) total size = 931200 Bytes */
	{REGFLAG_ESCAPE_ID, 0x81, 0x2C,{
	0x00, /* cmd id, enabled area */
	0x0C, 0x07, 0x03, 0x21, 0x90, /* area 0 (192,50,672*350) 		,byte[2~6]*/
	0x0C, 0x07, 0x19, 0x02, 0xEE, /* area 1 (192,400,672*350) 		,byte[7~11]*/
	0x18, 0x06, 0x38, 0x45, 0x14, /* area 2 (384,960,576*400) 		,byte[12~16]*/
	0x00, 0x06, 0x51, 0x46, 0xA4, /* area 3 (0,1300,576*400) 		,byte[17~21]*/
	0x00, 0x00, 0x00, 0x00, 0x00, /* area 4 - not use 				,byte[22~26]*/
	0x00, 0x00, 0x00, 0x00, 0x00, /* area 5 - not use 				,byte[27~31]*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* color_sel, depth, color 	,byte[32~37]*/
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00 /* gray, dummy 		,byte[38~44]*/
	}},

	{REGFLAG_ESCAPE_ID,0x9F,0x02,{0x5A,0x5A}},
};


static struct LCM_setting_table cmd_doze_area[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	/* (x,y,w*h) total size = 931200 Bytes */
	{0x81, 44, {
	0x00, /* cmd id, enabled area */
	0x0C, 0x07, 0x03, 0x21, 0x90, /* area 0 (192,50,672*350) 		,byte[2~6]*/
	0x0C, 0x07, 0x19, 0x02, 0xEE, /* area 1 (192,400,672*350) 		,byte[7~11]*/
	0x18, 0x06, 0x38, 0x45, 0x14, /* area 2 (384,960,576*400) 		,byte[12~16]*/
	0x00, 0x06, 0x51, 0x46, 0xA4, /* area 3 (0,1300,576*400) 		,byte[17~21]*/
	0x00, 0x00, 0x00, 0x00, 0x00, /* area 4 - not use 				,byte[22~26]*/
	0x00, 0x00, 0x00, 0x00, 0x00, /* area 5 - not use 				,byte[27~31]*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* color_sel, depth, color 	,byte[32~37]*/
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00 /* gray, dummy 		,byte[38~44]*/
	}},

	{0x9F, 2, {0x5A, 0x5A} },
};

static struct LCM_setting_table_V3 aod_off_n_hbm_on_v3[] = {
	{REGFLAG_ESCAPE_ID, 0x53, 0x01, {0x20} },

	{REGFLAG_ESCAPE_ID, 0xF0, 0x02, {0x5A, 0x5A} },
	{REGFLAG_ESCAPE_ID, 0xB3, 0x06, {0x11, 0x4B, 0x28, 0x09, 0x80, 0x07} },
	{REGFLAG_ESCAPE_ID, 0xF0, 0x02, {0xA5, 0xA5} },

	{REGFLAG_ESCAPE_ID, 0x53, 0x01, {0xE0} },
	{REGFLAG_ESCAPE_ID, 0x51, 0x02, {0x03, 0x29} },   /*600 nit*/
};

static void set_aod_displayon(void *handle, unsigned char levelsetting)
{
	dsi_set_cmdq_V4(aod_display_on_v3, ARRAY_SIZE(aod_display_on_v3), 1);
	LCM_INFO("----aod display on----- level:%d\n",  levelsetting);
}

static void lcm_setbacklight(void *handle, unsigned int level)
{
	unsigned int temp;

	temp = atomic_read(&oled_hbm_status);
	LCM_INFO("hbm status = %d, hbm on = %d, level =%d\n", temp, oled_hbm_on,level);
	if ((level != 0) && (bl_brightness_hal == 0)) {
		if (oled_ud_flag == 1) {
			bl_brightness_hal = level;
			LCD_INFO("can't set bl to level=%d, udfp is running\n", level);
			return;
		} else if (temp == HBM_UDFP_AOD_MODE_ON){
			push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
			oled_hbm_on = 0;
			atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
			LCD_INFO("turn off hbm because udfp flag is off\n");
		}
	}

	if ((level != 0) && (bl_brightness_hal != 0)) {
		if (oled_hbm_on == 1) {
			bl_brightness_hal = level;
			if ((0 == oled_ud_flag) && (temp == HBM_UDFP_RESUME_MODE_ON)) {
				push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
				oled_hbm_on = 0;
				atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
				LCD_INFO("turn off hbm because udfp flag is off\n");
			} else {
				LCD_INFO("-->can not set backlight level =%d, because udfp hbm is on\n", level);
				return;
			}
		}

		if (oled_dimming_enable == 4) {
			oled_dimming_enable = 0;
			MDELAY(20);
		}

		if (oled_dimming_enable == 0) {
			push_table(handle, cmd_brightness_dimming_on, sizeof(cmd_brightness_dimming_on) / sizeof(struct LCM_setting_table), 1);
			oled_dimming_enable = 1;
			MDELAY(20);
			LCD_INFO("-->set dimming on<---, level =%d, current_backlight = %d\n", level, current_backlight);
		}
	}
	if (level == 0) {
		oled_dimming_enable = 0;
	}
	if (temp != HBM_USER_AUTO_MODE_LEVEL0 && temp < HBM_USER_AUTO_MODE_LEVEL3 && current_backlight >= 2046 && oled_hbm_on == 0) {
		LCD_INFO("vincent+ to set hbm on,level=%d, oled_hbm_status=%d\n", level, temp);
		if (temp == HBM_USER_AUTO_MODE_LEVEL1) {
			push_table(handle, cmd_elvss_dimming_on, sizeof(cmd_elvss_dimming_on) / sizeof(struct LCM_setting_table), 1);
			push_table(handle, cmd_hbm_level1, sizeof(cmd_hbm_level1) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL1);
		} else {
			push_table(handle, cmd_elvss_dimming_on, sizeof(cmd_elvss_dimming_on) / sizeof(struct LCM_setting_table), 1);
			push_table(handle, cmd_hbm_level2, sizeof(cmd_hbm_level2) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL2);
		}
		oled_hbm_on = 1;
	} else {
		if (temp != HBM_USER_AUTO_MODE_LEVEL0 && temp < HBM_USER_AUTO_MODE_LEVEL3 && current_backlight < 2046 && oled_hbm_on == 1) {
			LCD_INFO("vincent+ to set hbm off,level=%d, oled_hbm_status=%d\n", level, temp);
			push_table(handle, cmd_hbm_off, sizeof(cmd_hbm_off) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL0);
			oled_hbm_on = 0;
		}
		bl_level[0].para_list[0] = (unsigned char)((level>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(level&0xFF);
		LCD_INFO("=>level=%d, dimming = %d, hal = %d, udfp = %d\n", level, oled_dimming_enable, current_backlight, oled_ud_flag);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}

	if(level == 0 && bl_brightness_hal != 0){
		LCM_INFO("set 28 10  high speed\n");
		//push_table(handle, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
		//atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
	}
	return ;
}

static void aod_area_init(struct aod_set_data *aod_data)
{
	aod_data->aod_area_support = true;
	/* area[0] is AOD display of clock */
	aod_data->area[0].rgb_enable = 0;
	aod_data->area[0].depth_bit = 2;
	aod_data->area[1].rgb_enable = 0;
	aod_data->area[1].depth_bit = 2;

	/* area[1] is AOD display of fingerprint */
	aod_data->area[5].enable = 1;
	aod_data->area[5].area_id = 5;
	aod_data->area[5].rgb_enable = 1;
	aod_data->area[5].x_start = 445;
	aod_data->area[5].y_start = 2075;
	aod_data->area[5].width = 192;
	aod_data->area[5].height = 190;
	aod_data->area[5].depth_bit = 4;
}
static int lcm_get_hbm_wait(void)
{
	return hbm_wait;
}
static int lcm_set_hbm_wait(int wait)
{
	int old = hbm_wait;
	hbm_wait = wait;
	return old;
}

static int lcm_set_hbm_cmdq(int en, void *handle)
{
	int old = atomic_read(&oled_hbm_status);
	LCM_INFO("----prev hbm = %d,will set hbm = %d----bl level = %d to %d\n", old, en, bl_brightness_hal, current_backlight);
	if ((old == en) && !((en == HBM_UDFP_RESUME_MODE_OFF) && (oled_weakup_hbmon == 1)))
		goto done;
	if (en == HBM_UDFP_RESUME_MODE_ON) {
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_elvss_dimming_off, sizeof(cmd_elvss_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_ud_on, sizeof(cmd_hbm_ud_on) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
		notify_udfp_cyan_make_sure_delay = 28;
	} else if (en == HBM_UDFP_RESUME_MODE_OFF)  {
		if((old == HBM_UDFP_RESUME_MODE_ON) || (old == HBM_UDFP_AOD_MODE_ON)){
			push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
			bl_level[0].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
			bl_level[0].para_list[1] = (unsigned char)(current_backlight&0xFF);
			push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		} else if ((old == HBM_UDFP_AOD_MODE_OFF) || (old == HBM_UDFP_RESUME_MODE_OFF)) {
			bl_level[0].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
			bl_level[0].para_list[1] = (unsigned char)(current_backlight&0xFF);
			push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		}
		oled_hbm_on = 0;
		oled_dimming_enable = 4;
	} else if (en == 13){
		if(oled_dimming_enable == 1){
			push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
			LCM_INFO("delay 40ms oled_dimming_enable:%d\n",oled_dimming_enable);
		}
		goto done;
	}
	lcm_set_hbm_wait(true);
done:
	return old;
}

static int lcm_get_hbm_state(void)
{
	int ret;
	ret = atomic_read(&oled_hbm_status);
	if (ret <= HBM_USER_AUTO_MODE_LEVEL3)
		return HBM_UDFP_RESUME_MODE_OFF;
	else
		return ret;
}

static void lcm_vivo_set_hbm(void *handle, unsigned int levelsetting)
{
	if (levelsetting == HBM_USER_AUTO_MODE_LEVEL0) {
		push_table(handle, cmd_hbm_off, sizeof(cmd_hbm_off) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	} else if (levelsetting == HBM_USER_AUTO_MODE_LEVEL1) {
		push_table(handle, cmd_elvss_dimming_on, sizeof(cmd_elvss_dimming_on) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_level1, sizeof(cmd_hbm_level1) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if (levelsetting == HBM_USER_AUTO_MODE_LEVEL2) {
		push_table(handle, cmd_elvss_dimming_on, sizeof(cmd_elvss_dimming_on) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_level2, sizeof(cmd_hbm_level2) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if (levelsetting == HBM_USER_AUTO_MODE_LEVEL3) {
		push_table(handle, cmd_elvss_dimming_on, sizeof(cmd_elvss_dimming_on) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_level3, sizeof(cmd_hbm_level3) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if (levelsetting == HBM_UDFP_RESUME_MODE_ON) {
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_elvss_dimming_off, sizeof(cmd_elvss_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_ud_on, sizeof(cmd_hbm_ud_on) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if (levelsetting == HBM_UDFP_RESUME_MODE_OFF) {
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		bl_level[0].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(current_backlight&0xFF);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
		oled_dimming_enable = 0;
	} else if (levelsetting == HBM_UDFP_AOD_MODE_ON) {
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_elvss_dimming_off, sizeof(cmd_elvss_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_hbm_ud_on, sizeof(cmd_hbm_ud_on) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
		notify_udfp_cyan_make_sure_delay = 28;
	} else if (levelsetting == HBM_UDFP_AOD_MODE_OFF) {
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		push_table(handle, cmd_brightness_dimming_aod, sizeof(cmd_brightness_dimming_aod) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	}else if ((levelsetting&0xFF) == HBM_AOD_UD_OFF_PERF_V3) {
		unsigned int level = (levelsetting>>8)&0x0FFF;
		push_table(handle, cmd_brightness_dimming_off, sizeof(cmd_brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
		bl_level[0].para_list[0] = (unsigned char)((level>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(level&0xFF);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		oled_ud_flag = 2;
		oled_hbm_on = 0;
		bl_brightness_hal = level;
		oled_dimming_enable = 4;
	}

	if((levelsetting&0xFF) == HBM_AOD_UD_OFF_PERF_V3){
		atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
		LCD_INFO("set hbm status to:%d-----bl level = %d---", HBM_UDFP_RESUME_MODE_OFF, bl_brightness_hal);
	} else {
		atomic_set(&oled_hbm_status, (unsigned char)levelsetting);
		LCD_INFO("set hbm status to:%d-----bl level = %d---",  levelsetting, bl_brightness_hal);
	}
}

static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned int levelsetting)
{
	if (cmdtype == 0x08)
		set_aod_displayon(handle, levelsetting);
	else if (cmdtype == 0x02)
		lcm_vivo_set_hbm(handle, levelsetting);
	else if (cmdtype == 0x09) /*set aor on or off*/
		dsi_panel_update_seed_crc(handle, levelsetting);
}

static void dsi_panel_update_seed_crc(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_seed_crc_p3) > 0))
		push_table(handle, oled_seed_crc_p3, sizeof(oled_seed_crc_p3) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(oled_seed_crc_srgb) > 0))
		push_table(handle, oled_seed_crc_srgb, sizeof(oled_seed_crc_srgb) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(oled_seed_crc_off) > 0))
		push_table(handle, oled_seed_crc_off, sizeof(oled_seed_crc_off) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----dsi_panel_update_seed_crc----- level:%d\n",  levelsetting);
}

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	/****************
	21		==> DongGuan Factory
	22		==> DongGuan Factory with Different thickness
	23		==> TianJin Factory
	****************/
	return 0x22;

}

#define SMART_AOD_MAX_AREA_NUM 6

enum color_depth bit_to_depth(unsigned int depth_bit)
{
	switch (depth_bit) {
	case 1:
		return AOD_1_BIT;
	case 2:
		return AOD_2_BIT;
	case 4:
		return AOD_4_BIT;
	case 8:
		return AOD_8_BIT;
	default:
		return AOD_2_BIT;
	}
}

static void lcm_set_aod_area_params(char * para_list)
{
	int i = 0;
	for (i = 0; i < AOD_MAX_AREA_NUM; i++) {
		struct aod_area_data *area = &g_aod_data.area[i];
		unsigned int x_start = area->x_start - 1;
		unsigned int y_start = area->y_start - 1;
		unsigned int y_end = area->y_start + area->height - 1;
		if (area->enable) {
			/* area enable*/
			para_list[0]   |= 1 << (5 - i);
			para_list[1 + i * 5] = x_start >> 4;
			para_list[2 + i * 5] = ((x_start & 0x0F) << 4 | (area->width / 96));
			para_list[3 + i * 5] = y_start >> 4;
			para_list[4 + i * 5] = ((y_start & 0x0F) << 4) | (y_end >> 8);
			para_list[5 + i * 5] = y_end & 0xFF;

			/* color_sel */
			if (i < 3)
				para_list[31] |= area->rgb_enable << (6 - i);
			else
				para_list[31] |= area->rgb_enable << (5 - i);

			/* area_depth */
			area->depth = bit_to_depth(area->depth_bit);
			if (i < 4)
				para_list[32] |= area->depth << (6 - i * 2);
			else
				para_list[33] |= (area->depth << (5 - i) * 2) << 4;

			/* set area_color only when mono */
			if (!area->rgb_enable) {
				if (i < 2)
				para_list[34]  |= area->color << (4-4 * i);
			else if (i < 4)
				para_list[35]  |= area->color << ((3-i) * 4);
			else
				para_list[36]  |= area->color << ((5-i) * 4);
			}

			/* set gray only when area_depth is 1 bit */
			if (area->depth == AOD_1_BIT)
			para_list[37 + i] = area->gray;
		}
	}
}

static void lcm_set_aod_area(void *handle, unsigned char * data)
{
	if (handle) {
		LCM_INFO("use push table handle to send aod area code\n");
		lcm_set_aod_area_params(cmd_doze_area[1].para_list);
		push_table(handle, cmd_doze_area, sizeof(cmd_doze_area) / sizeof(struct LCM_setting_table), 1);
	} else {
		LCM_INFO("use cmdq_V4 to send aod area code\n");
		lcm_set_aod_area_params(cmd_doze_area_v3[1].para_list);
		dsi_set_cmdq_V4(cmd_doze_area_v3, ARRAY_SIZE(cmd_doze_area_v3), 0);
	}

	LCM_INFO("lcm_set_aod_area end\n");
}

#if 0
static struct LCM_setting_table cmd_short[] = {
	{0x28, 1, {0x00} },
};
#endif

static void oled_lpm_aod(int on, void *handle)
{
	if ((on == 4) && (sizeof(aod_display_on_v3)) > 0) {
		//dsi_set_cmdq_V4(aod_display_on_v3, ARRAY_SIZE(aod_display_on_v3), 1);
		push_table(handle, aod_display_on, sizeof(aod_display_on) / sizeof(struct LCM_setting_table), 1);
		LCM_INFO("on == 4\n");
	} else if (on == 1){
		dsi_set_cmdq_V4(cmd_aod_on_v3, ARRAY_SIZE(cmd_aod_on_v3), 0);
		atomic_set(&doze_mode_state,LCM_PANEL_AOD);
		LCM_INFO(" on == 1\n");
	} else if (on == 0) {
		mutex_lock(&hbm_lock);
		LCM_INFO("on == 0 \n");
		dsi_set_cmdq_V4(cmd_aod_off_v3, ARRAY_SIZE(cmd_aod_off_v3), 0);
		if(set_hbm_on_aod_exit){
			dsi_set_cmdq_V4(aod_off_n_hbm_on_v3, ARRAY_SIZE(aod_off_n_hbm_on_v3), 0);
			atomic_set(&oled_hbm_status, HBM_UDFP_AOD_MODE_ON);
			set_hbm_on_aod_exit = false;
			oled_hbm_on = 1;
			notify_udfp_cyan_make_sure_delay = 90;
			LCM_INFO("setting aod_off_n_hbm_on_v3 \n");
		}
		atomic_set(&aod_control, 1);
		atomic_set(&doze_mode_state,LCM_PANEL_ON);
		mutex_unlock(&hbm_lock);
	} else if (on == 7){
		dsi_set_cmdq_V4(aod_display_off_v3, ARRAY_SIZE(aod_display_off_v3), 1);
		LCM_INFO("on == 7 \n");
	} else if (on == 8) {
		/*use cmdq v4 to send suspend cmd before LP11*/
		dsi_set_cmdq_V4(lcm_suspend_setting_v3, ARRAY_SIZE(lcm_suspend_setting_v3), 1);
		LCM_INFO("on == 8 \n");
	}
	LCM_INFO("on:%d\n",on);
}

#define lcm_doze_delay 3
static int lcm_get_doze_delay(void)
{

	return lcm_doze_delay;
}

struct LCM_DRIVER pd2078_samsung_s6e8fc1_fhdp_vdo_lcm_drv = {
	.name				= "pd2078_samsung_s6e8fc1_fhdp_vdo_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.enable = lcm_enable,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.set_backlight_cmdq = lcm_setbacklight,
	.update = lcm_update,
	.get_id	    = lcm_get_id,
	.get_hbm_state = lcm_get_hbm_state,
	.set_hbm_cmdq = lcm_set_hbm_cmdq,
	.get_hbm_wait = lcm_get_hbm_wait,
	.set_hbm_wait = lcm_set_hbm_wait,
	.set_aod_area_cmdq = lcm_set_aod_area,
	.lcm_MipiCmd_HS  = lcm_vivo_MipiCmd_HS,
	.aod = oled_lpm_aod,
	.get_doze_delay = lcm_get_doze_delay,
};
