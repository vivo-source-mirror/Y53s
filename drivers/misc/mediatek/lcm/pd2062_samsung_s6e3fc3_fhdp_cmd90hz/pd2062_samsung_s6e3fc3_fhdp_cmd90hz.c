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
//	#include <mt-plat/mtk_gpio_core.h>
//	#include "mt-plat/upmu_common.h"
//	#include "mt-plat/mtk_gpio.h"
	#include "disp_dts_gpio.h"
//	#include <linux/i2c.h> /*lcm power is provided by i2c*/
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
#define LCM_DSI_CMD_MODE                1

#define FRAME_WIDTH                     (1080)
#define FRAME_HEIGHT                    (2404)
/* physical size in um */
#define PHYSICAL_WIDTH                  (64)
#define PHYSICAL_HEIGHT                 (129)

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
#define dfps_dsi_send_cmd( \
		cmdq, cmd, count, para_list, force_update, sendmode) \
		lcm_util.dsi_dynfps_send_cmd( \
		cmdq, cmd, count, para_list, force_update, sendmode)

//#define REGFLAG_DELAY		0xFE
//#define REGFLAG_END_OF_TABLE	0xFF /* END OF REGISTERS MARKER */

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static int hbm_wait;
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
static void dsi_panel_update_seed_crc(void *handle, unsigned char levelsetting);

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 15, {} },

	{0x10, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 160, {} },
};

#if 0
static struct LCM_setting_table cmd_bl_level[] = {
	{0x51, 2, {0x0F, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

#endif

static struct LCM_setting_table init_setting[] =  {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x11, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 100, {} },

	/*Frequency Control Setting*/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 2, {0x27, 0xF2} },
	{0xF2, 1, {0x00} },
	{0xF0, 2, {0xA5, 0xA5} },

	/*Frequency Select*/
	{0xF0, 2, {0x5A, 0x5A} },
	{0x60, 2, {0x00, 0x00} },
	{0xF7, 1, {0x0F} },
	{0xF0, 2, {0xA5, 0xA5} },

	/*Dimming Setting*/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 2, {0x92, 0x63} },
	{0x63, 1, {0x28} },
	{0xB0, 2, {0x91, 0x63} },
	{0x63, 1, {0x20} },
	{0xF0, 2, {0xA5, 0xA5} },

	/*TE ON*/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x35, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },

	/*Brightness Setting*/
	{0x51, 2, {0x01, 0x72} },
	{0x53, 1, {0x20} },
	{0x55, 1, {0x00} },

	/*Display On*/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 20, {} },
};


#ifdef CONFIG_MTK_HIGH_FRAME_RATE

#define DFPS_MAX_CMD_NUM 10

struct LCM_dfps_cmd_table {
	bool need_send_cmd;
	enum LCM_Send_Cmd_Mode sendmode;
	struct LCM_setting_table switch_fps_table[DFPS_MAX_CMD_NUM];
};

static struct LCM_dfps_cmd_table
	dfps_cmd_table[DFPS_LEVELNUM][DFPS_LEVELNUM] = {

	/**********level 0 to 0*********************/
	[DFPS_LEVEL0][DFPS_LEVEL0] = {
		false,
		LCM_SEND_IN_CMD,
		{
		{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},
	/**********level 0 to 1*********************/
	[DFPS_LEVEL0][DFPS_LEVEL1] = {
		true,
		LCM_SEND_IN_CMD,
		{
			{0xF0, 2, {0x5A, 0x5A} },
			{0x60, 2, {0x08, 0x00} },
			{0xF7, 1, {0x0F} },
			{0xF0, 2, {0xA5, 0xA5} },
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},

	/**********level 1 to 0*********************/
	/*90->60*/
	[DFPS_LEVEL1][DFPS_LEVEL0] = {
		true,
		LCM_SEND_IN_CMD,
		{
			{0xF0, 2, {0x5A, 0x5A} },
			{0x60, 2, {0x00, 0x00} },
			{0xF7, 1, {0x0F} },
			{0xF0, 2, {0xA5, 0xA5} },
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},
	},
	/**********level 1 to 1*********************/
	[DFPS_LEVEL1][DFPS_LEVEL1] = {
		false,
		LCM_SEND_IN_CMD,
		{
			{REGFLAG_END_OF_TABLE, 0x00, {} },
		},

	},


};
#endif
static struct LCM_setting_table bl_level[] = {
	{0x51, 2, {0x07, 0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table brightness_dimming_on[] = {
	{0xF0, 2,{0x5A,0x5A}},
	{0xB0, 1,{0x07}},
	{0xB7, 1,{0x28}},
	{0xF0, 2,{0xA5,0xA5}},
	{0x53, 1, {0x28} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table brightness_dimming_off[] = {
	{0x53, 1, {0x20} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};

static struct LCM_setting_table oled_alpm_on[] = {
	/********Display off*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 17, {} },
	{0x9F, 2, {0x5A, 0x5A} },

	/********AOD mode on*********/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0xDF} },
	{0xB8, 4, {0x02, 0xC1, 0x00, 0x5D} },
	{0xB0, 1, {0x69} },
	{0xB9, 5, {0x08, 0x08, 0xEB, 0x08, 0x0E} }, /*Dimming: 50 nit 8cyc/8cyc*/
	{0xB0, 1, {0x0B} },
	{0xF4, 1, {0x6C} },
	{0x53, 1, {0x22} },
	{0xB0, 1, {0xA5} },
	{0xC7, 1, {0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
};
static struct LCM_setting_table aod_display_on[] = {
	/********AOD Display on*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x13, 1, {0x00} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
};


static struct LCM_setting_table oled_udfp_aod_on[] = {
	/********Display off*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 17, {} },
	{0x9F, 2, {0x5A, 0x5A} },

	/********AOD mode on*********/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0xDF} },
	{0xB8, 4, {0x02, 0xC1, 0x00, 0x5D} },
	{0xB0, 1, {0x69} },
	{0xB9, 5, {0x08, 0x08, 0xEB, 0x08, 0x0E} }, /*Dimming: 50 nit 8cyc/8cyc*/
	{0xB0, 1, {0x0B} },
	{0xF4, 1, {0x6C} },
	{0x53, 1, {0x22} },
	{0xB0, 1, {0xA5} },
	{0xC7, 1, {0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
	/********AOD Display on*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x13, 1, {0x00} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
};

static struct LCM_setting_table oled_hbm_off_n_aod_on[] = {
	/********hbm off**************/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xBD, 2, {0x00, 0x02} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x20} },
	{0x51, 2, {0x01, 0x90} },

	/********Display off*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 17, {} },
	{0x9F, 2, {0x5A, 0x5A} },

	/********AOD mode on*********/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0xDF} },
	{0xB8, 4, {0x02, 0xC1, 0x00, 0x5D} },
	{0xB0, 1, {0x69} },
	{0xB9, 5, {0x08, 0x08, 0xEB, 0x08, 0x0E} }, /*Dimming: 50 nit 8cyc/8cyc*/
	{0xB0, 1, {0x0B} },
	{0xF4, 1, {0x6C} },
	{0x53, 1, {0x22} },
	{0xB0, 1, {0xA5} },
	{0xC7, 1, {0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
	/********AOD Display on*********/
	{0x9F, 2, {0xA5, 0xA5} },
	{0x13, 1, {0x00} },
	{0x29, 1, {0x00} },
	{0x9F, 2, {0x5A, 0x5A} },
};

static struct LCM_setting_table oled_alpm_off[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x22, 0, {} },
	{REGFLAG_DELAY, 0, {} },
	{0x28, 0, {} },
	{REGFLAG_DELAY, 17, {} },
	{0x9F, 2, {0x5A, 0x5A} },
	{0x53, 1, {0x28} },

	{0x9F, 2, {0xA5, 0xA5} },
	{0x13, 0, {} },
	{0x29, 0, {} },
	{0x9F, 2, {0x5A, 0x5A} },
};


static struct LCM_setting_table aod_off_n_hbm_on[] = {
	{0x9F, 2, {0xA5, 0xA5} },
	{0x22, 0, {} },
	{0x28, 0, {} },
	{0x9F, 2, {0x5A, 0x5A} },
	{REGFLAG_DELAY, 17, {} },

	{0x53, 1, {0x28} },

	{0x9F, 2, {0xA5, 0xA5} },
	{0x13, 0, {} },
	{0x29, 0, {} },
	{0x9F, 2, {0x5A, 0x5A} },

	{0x53, 1, {0x20} },

	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x08} },
	{0xB7, 1, {0x10} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0xE0} },
	{0x51, 2, {0x07, 0xFF} },
	{0xF0, 2, {0x5A, 0x5A} },
	{0xBD, 2, {0x00, 0x00} },
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
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

static struct LCM_setting_table oled_cmd_hbm_fingerprint[] = {
	{0x53, 1, {0x20} },						/*DIM OFF*/
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x08} },
	{0xB7, 1, {0x10} },						/*ELVSS DIM OFF*/
	{0xF0, 2, {0xA5, 0xA5} },

	{0x53, 1, {0xE0} },
	{0x51, 2, {0x07, 0xFF} },

	{0xF0, 2, {0x5A, 0x5A} },
	{0xBD, 2, {0x00, 0x00} },					/*PS OFF*/
	{0xF0, 2, {0xA5, 0xA5} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table oled_cmd_ud_resume_hbm_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xBD, 2, {0x00, 0x02} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x20} },
	{0x51, 2, {0x07, 0xFF} },
};

static struct LCM_setting_table oled_cmd_ud_aod_hbm_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xBD, 2, {0x00, 0x02} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0x53, 1, {0x20} },
	{0x51, 2, {0x01, 0x90} },
};

static struct LCM_setting_table oled_cmd_hbm_level1[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x08} },
	{0xB7, 1, {0x90} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0xF0, 2,{0x5A,0x5A}},
	{0xB0, 1,{0x07}},
	{0xB7, 1,{0x28}},
	{0xF0, 2,{0xA5,0xA5}},
	{0x53, 1, {0x28} },
	{REGFLAG_DELAY, 17, {} },
	{0x53, 1, {0xE8} },
	{0x51, 2, {0x01, 0x94} },   /*520 nit*/
};
static struct LCM_setting_table oled_cmd_hbm_level2[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0xB0, 1, {0x08} },
	{0xB7, 1, {0x90} },
	{0xF0, 2, {0xA5, 0xA5} },
	{0xF0, 2,{0x5A,0x5A}},
	{0xB0, 1,{0x07}},
	{0xB7, 1,{0x28}},
	{0xF0, 2,{0xA5,0xA5}},
	{0x53, 1, {0x28} },
	{REGFLAG_DELAY, 17, {} },
	{0x53, 1, {0xE8} },
	{0x51, 2, {0x02, 0xc4} },  /*580 nit*/
};

static struct LCM_setting_table oled_cmd_hbm_off[] = {
	{0x53, 1, {0x28} },
	{0x51, 2, {0x07, 0xff} },
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
	{0xB0, 1, {0xDE} },
	{0xB9, 1, {0x48} },
	/* seed setting*/
	{0x81, 1, {0x90} },
	{0xB1, 23, {0x00,0x00,0xD6,0x00,0x00,0x20,0xF5,0x00,0x00,0x05,0xFF,0x25,0xFF,0xE4,0xFF,0x06,0xF1,0xFF,0xF1,0x00,0xFF,0xFC,0xFF}},
	{0xB6, 2, {0x00, 0x02} },
	{0xB0, 1, {0x1C} },
	{0xB3, 1, {0x00} },
	{0xB0, 1, {0x23} },
	{0xB3, 1, {0x91} },
	{0xB0, 1, {0x21} },
	{0xB3, 1, {0xF8} },
	{0x83, 1, {0x80} },
	{0xB3, 2, {0x00, 0xC0} },
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table oled_seed_crc_p3[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x81, 1, {0x90} },
	{0xB1, 23,{0x00,0x00,0xD6,0x00,0x00,0x20,0xF5,0x00,0x00,0x05,0xFF,0x25,0xFF,0xE4,0xFF,0x06,0xF1,0xFF,0xF1,0x00,0xFF,0xFC,0xFF}},
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table oled_seed_crc_srgb[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x81, 1, {0x90} },
	{0xB1, 23,{0x00,0x00,0xA8,0x03,0x04,0x44,0xC8,0x12,0x05,0x09,0xB3,0x59,0xE7,0xD0,0xBC,0x11,0xBE,0xE5,0xD8,0x1A,0xFF,0xF8,0xE0}},
	{0xF0, 2, {0xA5, 0xA5} },
};

static struct LCM_setting_table oled_seed_crc_off[] = {
	{0xF0, 2, {0x5A, 0x5A} },
	{0x81, 1, {0x00} },
	{0xB1, 2, {0x00,0x01}},
	{0xF0, 2, {0xA5, 0xA5} },
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

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void lcm_dfps_int(struct LCM_DSI_PARAMS *dsi)
{
	struct dfps_info *dfps_params = dsi->dfps_params;

	dsi->dfps_enable = 1;
	dsi->dfps_default_fps = 9000;/*real fps * 100, to support float*/
	dsi->dfps_def_vact_tim_fps = 9000;/*real vact timing fps * 100*/

	/*traversing array must less than DFPS_LEVELS*/
	/*DPFS_LEVEL0*/
	dfps_params[0].level = DFPS_LEVEL0;
	dfps_params[0].fps = 6000;/*real fps * 100, to support float*/
	dfps_params[0].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[0].PLL_CLOCK = xx;*/
	/*dfps_params[0].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[0].horizontal_frontporch = xx;*/


	/*if need mipi hopping params add here*/
	dfps_params[0].dynamic_switch_mipi = 0;
	dfps_params[0].PLL_CLOCK_dyn = 550;


	/*DPFS_LEVEL1*/
	dfps_params[1].level = DFPS_LEVEL1;
	dfps_params[1].fps = 9000;/*real fps * 100, to support float*/
	dfps_params[1].vact_timing_fps = 9000;/*real vact timing fps * 100*/
	/*if mipi clock solution*/
	/*dfps_params[1].PLL_CLOCK = xx;*/
	/*dfps_params[1].data_rate = xx; */
	/*if HFP solution*/
	/*dfps_params[1].horizontal_frontporch = xx;*/


	/*if need mipi hopping params add here*/
	dfps_params[1].dynamic_switch_mipi = 0;
	dfps_params[1].PLL_CLOCK_dyn = 550;
	dfps_params[1].horizontal_frontporch_dyn = 288;
	dfps_params[1].vertical_frontporch_dyn = 54;
	dfps_params[1].vertical_frontporch_for_low_power_dyn = 2500;

	dsi->dfps_num = 2;
}
#endif

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->dsi.IsCphy = 0;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;

	params->physical_width          = PHYSICAL_WIDTH;
	params->physical_height         = PHYSICAL_HEIGHT;

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
	params->dsi.vertical_backporch		= 16;
	params->dsi.vertical_frontporch		= 10;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 20;
	params->dsi.horizontal_backporch	  = 40;
	params->dsi.horizontal_frontporch	  = 40;
	params->dsi.horizontal_active_pixel	= FRAME_WIDTH;


	/* Bit rate calculation */
	params->dsi.PLL_CLOCK = 840;

	/* this value must be in MTK suggested table */
	params->dsi.data_rate = 1680;


	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	lcm_dfps_int(&(params->dsi));
#endif

}

static void lcm_init_power(void)
{
	LCM_INFO("\n");
}

static void lcm_suspend_power(void)
{
	//SET_RESET_PIN(0);
	LCM_INFO("\n");
}

static void lcm_resume_power(void)
{
	lcm_reset_setting(0);
	lcm_init_power();
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
static int lcm_set_hbm_cmdq(int en, void *qhandle)
{
	int old = atomic_read(&oled_hbm_status);
	 LCM_INFO("----prev hbm = %d,will set hbm = %d----bl level = %d to %d\n", old, en, bl_brightness_hal, current_backlight);
	 if ((old == en) && !((en == HBM_UDFP_RESUME_MODE_OFF) && (oled_weakup_hbmon == 1)))
		goto done;
	 if (en == HBM_UDFP_RESUME_MODE_ON) {
	 	oled_cmd_hbm_fingerprint[3].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
		push_table(qhandle, oled_cmd_hbm_fingerprint, sizeof(oled_cmd_hbm_fingerprint) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
 	} else if (en == HBM_UDFP_RESUME_MODE_OFF)  {
	 	if ((old == HBM_UDFP_RESUME_MODE_ON) || (old == HBM_UDFP_AOD_MODE_ON)){
			oled_cmd_ud_resume_hbm_off[4].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
			oled_cmd_ud_resume_hbm_off[4].para_list[1] = (unsigned char)(current_backlight&0xFF);
			push_table(qhandle, oled_cmd_ud_resume_hbm_off, sizeof(oled_cmd_ud_resume_hbm_off) / sizeof(struct LCM_setting_table), 1);
 		} else if ((old == HBM_UDFP_AOD_MODE_OFF) || (old == HBM_UDFP_RESUME_MODE_OFF)) {
			bl_level[0].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
			bl_level[0].para_list[1] = (unsigned char)(current_backlight&0xFF);
			push_table(qhandle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
		}
		oled_dimming_enable = 4;
		oled_hbm_on = 0;
	}
	lcm_set_hbm_wait(true);
done:
	return old;
}
static void lcm_init(void)
{
	lcm_reset_setting(0);
	MDELAY(5);
	lcm_reset_setting(1);
	MDELAY(1);
	lcm_reset_setting(0);
	MDELAY(10);
	lcm_reset_setting(1);
	MDELAY(5);

	push_table(NULL, init_setting, sizeof(init_setting) /
		   sizeof(struct LCM_setting_table), 1);
	LCM_INFO("\n");
	atomic_set(&doze_mode_state, LCM_PANEL_ON);
}


static void lcm_suspend(void)
{
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) /
		   sizeof(struct LCM_setting_table), 1);

	lcm_reset_setting(0);
	MDELAY(10);
	LCM_INFO("\n");
	atomic_set(&oled_hbm_status, HBM_UDFP_RESUME_MODE_OFF);
	atomic_set(&doze_mode_state, LCM_PANEL_OFF);
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



static void lcm_setbacklight(void *handle, unsigned int level)
{
	unsigned int temp;

	if ((level != 0) && (bl_brightness_hal == 0) && (oled_weakup_hbmon == 1)) {
		bl_brightness_hal = 0;
		LCM_INFO("can't set bl to level=%d, udfp is running\n", level);
		return;
	}

	temp = atomic_read(&oled_hbm_status);
	LCM_DEBUG("hbm status = %d, hbm on = %d, level =%d\n", temp, oled_hbm_on,level);
	if ((level != 0) && (bl_brightness_hal != 0)) {
		if ((temp == HBM_UDFP_RESUME_MODE_ON) || (temp == HBM_UDFP_AOD_MODE_ON) || (temp == HBM_UDFP_CALI_MODE_ON)) {
			LCM_INFO("-->can not set backlight level =%d, because hbm is on\n", level);
			return;
		}

		if (oled_dimming_enable == 4) {
			oled_dimming_enable = 0;
			MDELAY(20);
		}

		if (oled_dimming_enable == 0) {
			push_table(handle, brightness_dimming_on, sizeof(brightness_dimming_on) / sizeof(struct LCM_setting_table), 1);
			oled_dimming_enable = 1;
			MDELAY(20);
			LCM_INFO("-->set dimming on<---, level =%d, current_backlight = %d\n", level, current_backlight);
		}
	}

	if (level == 0) {
		oled_dimming_enable = 0;
	}

	if (temp != HBM_USER_AUTO_MODE_LEVEL0 && temp < HBM_USER_AUTO_MODE_LEVEL3 && current_backlight >= 2046 && oled_hbm_on == 0) {
		LCM_INFO("vincent+ to set hbm on,level=%d, oled_hbm_status=%d\n", level, temp);
		if (temp == HBM_USER_AUTO_MODE_LEVEL1) {
			oled_cmd_hbm_level1[2].para_list[0] = (unsigned char)(lcm_especial_id |0x80);
			push_table(handle, oled_cmd_hbm_level1, sizeof(oled_cmd_hbm_level1) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL1);
		} else {
			oled_cmd_hbm_level2[2].para_list[0] = (unsigned char)(lcm_especial_id |0x80);
			push_table(handle, oled_cmd_hbm_level2, sizeof(oled_cmd_hbm_level2) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL2);
		}
		oled_hbm_on = 1;
	} else {
		if (temp != HBM_USER_AUTO_MODE_LEVEL0 && temp < HBM_USER_AUTO_MODE_LEVEL3 && current_backlight < 2046 && oled_hbm_on == 1) {
			LCM_INFO("vincent+ to set hbm off,level=%d, oled_hbm_status=%d\n", level, temp);
			push_table(handle, oled_cmd_hbm_off, sizeof(oled_cmd_hbm_off) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_USER_AUTO_MODE_LEVEL0);
			oled_hbm_on = 0;
		}
		bl_level[0].para_list[0] = (unsigned char)((level>>8)&0xF);
		bl_level[0].para_list[1] = (unsigned char)(level&0xFF);
		LCM_INFO("=>level=%d, dimming = %d, hal = %d, udfp = %d\n", level, oled_dimming_enable, current_backlight, oled_weakup_hbmon);
		push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
	}
}

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
static void dfps_dsi_push_table(
	void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update, enum LCM_Send_Cmd_Mode sendmode)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_END_OF_TABLE:
			return;
		default:
			dfps_dsi_send_cmd(
				cmdq, cmd, table[i].count,
				table[i].para_list, force_update, sendmode);
			break;
		}
	}

}
static bool lcm_dfps_need_inform_lcm(
	unsigned int from_level, unsigned int to_level, struct LCM_PARAMS *params)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;

	LCM_INFO("%s,from_level:%d to_level:%d \n", __func__,from_level,to_level);
	if (from_level == to_level) {
		LCM_INFO("%s,same level\n", __func__);
		return false;
	}
	p_dfps_cmds =
		&(dfps_cmd_table[from_level][to_level]);
	params->sendmode = p_dfps_cmds->sendmode;

	return p_dfps_cmds->need_send_cmd;
}

static void lcm_dfps_inform_lcm(void *cmdq_handle,
unsigned int from_level, unsigned int to_level, struct LCM_PARAMS *params)
{
	struct LCM_dfps_cmd_table *p_dfps_cmds = NULL;
	enum LCM_Send_Cmd_Mode sendmode = LCM_SEND_IN_CMD;

	if (from_level == to_level) {
		LCM_INFO("%s,same level\n", __func__);
		goto done;
	}
	p_dfps_cmds =
		&(dfps_cmd_table[from_level][to_level]);
	LCM_INFO("%s,no cmd[L%d->L%d]\n",
		__func__, from_level, to_level);

	if (p_dfps_cmds &&
		!(p_dfps_cmds->need_send_cmd)) {
		LCM_INFO("%s,no cmd[L%d->L%d]\n",
			__func__, from_level, to_level);
		goto done;
	}

	sendmode = params->sendmode;

	dfps_dsi_push_table(
		cmdq_handle, p_dfps_cmds->switch_fps_table,
		ARRAY_SIZE(p_dfps_cmds->switch_fps_table), 1, sendmode);
done:
	LCM_INFO("%s,done %d->%d\n",
		__func__, from_level, to_level, sendmode);

}
#endif
static void lcm_vivo_set_hbm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == HBM_USER_AUTO_MODE_LEVEL0) && (sizeof(oled_cmd_hbm_off) > 0)) {
		push_table(handle, oled_cmd_hbm_off, sizeof(oled_cmd_hbm_off) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	} else if ((levelsetting == HBM_USER_AUTO_MODE_LEVEL1) && (sizeof(oled_cmd_hbm_level1) > 0)) {
		oled_cmd_hbm_level1[2].para_list[0] = (unsigned char)(lcm_especial_id |0x80);
		push_table(handle, oled_cmd_hbm_level1, sizeof(oled_cmd_hbm_level1) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if ((levelsetting == HBM_USER_AUTO_MODE_LEVEL2) && (sizeof(oled_cmd_hbm_level2) > 0)) {
		oled_cmd_hbm_level2[2].para_list[0] = (unsigned char)(lcm_especial_id |0x80);
		push_table(handle, oled_cmd_hbm_level2, sizeof(oled_cmd_hbm_level2) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if ((levelsetting == HBM_UDFP_RESUME_MODE_ON) && (sizeof(oled_cmd_hbm_fingerprint) > 0)) {
		oled_cmd_hbm_fingerprint[3].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
		push_table(handle, oled_cmd_hbm_fingerprint, sizeof(oled_cmd_hbm_fingerprint) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 1;
	} else if ((levelsetting == HBM_UDFP_RESUME_MODE_OFF) && (sizeof(oled_cmd_ud_resume_hbm_off) > 0)) {
		oled_cmd_ud_resume_hbm_off[4].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
		oled_cmd_ud_resume_hbm_off[4].para_list[1] = (unsigned char)(current_backlight&0xFF);
		push_table(handle, oled_cmd_ud_resume_hbm_off, sizeof(oled_cmd_ud_resume_hbm_off) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
		oled_dimming_enable = 0;
	} else if ((levelsetting == HBM_UDFP_AOD_MODE_ON) && (sizeof(oled_cmd_hbm_fingerprint) > 0)) {
		if (atomic_read(&doze_mode_state) == LCM_PANEL_AOD) {
			aod_off_n_hbm_on[13].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
			push_table(handle, aod_off_n_hbm_on, sizeof(aod_off_n_hbm_on) / sizeof(struct LCM_setting_table), 1);
			LCM_INFO("<--aod off and hbm on together--->, --hbm on = %d, elvss = 0x%x\n", levelsetting, aod_off_n_hbm_on[13].para_list[0]);
			atomic_set(&doze_mode_state, LCM_PANEL_ON);
		} else {
			oled_cmd_hbm_fingerprint[3].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
			push_table(handle, oled_cmd_hbm_fingerprint, sizeof(oled_cmd_hbm_fingerprint) / sizeof(struct LCM_setting_table), 1);
		}
		oled_hbm_on = 1;
	} else if ((levelsetting == HBM_UDFP_AOD_MODE_OFF) && (sizeof(oled_cmd_ud_aod_hbm_off) > 0)) {
		push_table(handle, oled_cmd_ud_aod_hbm_off, sizeof(oled_cmd_ud_aod_hbm_off) / sizeof(struct LCM_setting_table), 1);
		atomic_set(&doze_mode_state, LCM_PANEL_AOD);
		oled_hbm_on = 0;
	} else if ((levelsetting == HBM_UDFP_CALI_MODE_ON) && (sizeof(oled_cmd_hbm_fingerprint) > 0)) {
		oled_cmd_hbm_fingerprint[3].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
	        push_table(handle, oled_cmd_hbm_fingerprint, sizeof(oled_cmd_hbm_fingerprint) / sizeof(struct LCM_setting_table), 1);
	        oled_hbm_on = 1;
	} else if ((levelsetting == HBM_UDFP_CALI_MODE_OFF) && (sizeof(oled_cmd_ud_resume_hbm_off) > 0)) {
		oled_cmd_ud_resume_hbm_off[4].para_list[0] = (unsigned char)((current_backlight>>8)&0xF);
		oled_cmd_ud_resume_hbm_off[4].para_list[1] = (unsigned char)(current_backlight&0xFF);
		push_table(handle, oled_cmd_ud_resume_hbm_off, sizeof(oled_cmd_ud_resume_hbm_off) / sizeof(struct LCM_setting_table), 1);
		oled_hbm_on = 0;
	}
	atomic_set(&oled_hbm_status, levelsetting);
	LCM_INFO("set hbm status to:%d-----bl level = %d---, elvss dim on = 0x%x, off = 0x%x\n",  levelsetting, bl_brightness_hal, oled_cmd_hbm_level1[2].para_list[0], oled_cmd_hbm_fingerprint[3].para_list[0]);
}

extern char reg[256];
static void lcm_vivo_write_mipi_reg(void *handle, unsigned char levelsetting)
{
	int i;
	/*echo 0f 04 00 08 b8 01 0f 09 ff 18 ff ea ff 0c F8 fc ff 01 fe f5 de > oled_alpmmode*/
	for (i = 0; i < 23; i++) {
	oled_seed[4].para_list[i] = reg[i];
		/*LCM_INFO("----lcmalpm----- oled_alpmmode_store:0x%x\n",  oled_seed[3].para_list[i]);*/
	}
	push_table(handle, oled_seed, sizeof(oled_seed) / sizeof(struct LCM_setting_table), 1);
	LCM_INFO("----lcm_vivo_write_mipi_reg----- level:%d\n",  levelsetting);
}

static void lcm_vivo_set_alpm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_alpm_lbrightness) > 0))
		push_table(handle, oled_alpm_lbrightness, sizeof(oled_alpm_lbrightness) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(oled_alpm_hbrightness) > 0))
		push_table(handle, oled_alpm_hbrightness, sizeof(oled_alpm_hbrightness) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----lcmalpm----- level:%d\n",  levelsetting);
}

static void lcm_vivo_set_hlpm(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(oled_hlpm_lbrightness) > 0))
		push_table(handle, oled_hlpm_lbrightness, sizeof(oled_hlpm_lbrightness) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(oled_hlpm_hbrightness)>0))
		push_table(handle, oled_hlpm_hbrightness, sizeof(oled_hlpm_hbrightness) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----lcmhlpm----- level:%d\n",  levelsetting);
}

/******************************************lcmacl start**************************************************/
static void lcm_vivo_set_acl(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_cmd_acl_off) > 0))
		push_table(handle, lcm_cmd_acl_off, sizeof(lcm_cmd_acl_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 1) && (sizeof(lcm_cmd_acl_level1) > 0))
		push_table(handle, lcm_cmd_acl_level1, sizeof(lcm_cmd_acl_level1) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 2) && (sizeof(lcm_cmd_acl_level2) > 0))
		push_table(handle, lcm_cmd_acl_level2, sizeof(lcm_cmd_acl_level2) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 3) && (sizeof(lcm_cmd_acl_level3) > 0))
		push_table(handle, lcm_cmd_acl_level3, sizeof(lcm_cmd_acl_level3) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----lcmacl----- level:%d\n",  levelsetting);
}
static void set_lcm_temperature(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 0) && (sizeof(lcm_temperature_compensation_off) > 0))
		push_table(handle, lcm_temperature_compensation_off, sizeof(lcm_temperature_compensation_off) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 5) && (sizeof(lcm_temperature_compensation_minus_five) > 0))
		push_table(handle, lcm_temperature_compensation_minus_five, sizeof(lcm_temperature_compensation_minus_five) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----lcm_vivo_set_lcm_temperature----- level:%d\n",  levelsetting);
}
static void set_brightness_dimming(void *handle, unsigned char levelsetting)
{
	if ((levelsetting == 1) && (sizeof(brightness_dimming_on) > 0))
		push_table(handle, brightness_dimming_on, sizeof(brightness_dimming_on) / sizeof(struct LCM_setting_table), 1);
	else if ((levelsetting == 0) || (levelsetting == 3))
		push_table(handle, brightness_dimming_off, sizeof(brightness_dimming_off) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----set_brightness_dimming----- level:%d\n",  levelsetting);
}

static void set_aod_displayon(void *handle, unsigned char levelsetting)
{
	push_table(handle, aod_display_on, sizeof(aod_display_on) / sizeof(struct LCM_setting_table), 1);
      LCM_INFO("----aod display on----- level:%d\n",  levelsetting);
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
static void lcm_vivo_MipiCmd_HS(void *handle, unsigned char cmdtype, unsigned int levelsetting)
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
	else if (cmdtype == 0x08) /*set aod display on*/
		set_aod_displayon(handle, levelsetting);
	else if (cmdtype == 0x09) /*set aor on or off*/
		dsi_panel_update_seed_crc(handle, levelsetting);

}

/******************************************lcmacl end**************************************************/

static unsigned int lcm_get_id(unsigned char *lcm_flag)
{
	/****************
	21		==> DongGuan Factory
	22		==> DongGuan Factory with Different thickness
	23		==> TianJin Factory
	****************/
	return 0x22;

}
static void oled_lpm_aod(int on, void *handle)
{
	if ((on == 0) && (sizeof(oled_alpm_off)) > 0) {
		if (atomic_read(&doze_mode_state) == LCM_PANEL_AOD_SUSPEND) {
			aod_off_n_hbm_on[13].para_list[0] = (unsigned char)(lcm_especial_id &0x7F);
			push_table(handle, aod_off_n_hbm_on, sizeof(aod_off_n_hbm_on) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_UDFP_AOD_MODE_ON);
			LCM_INFO("<--aod off and hbm on together--->, --aod on = %d, elvss = 0x%x\n", on, aod_off_n_hbm_on[13].para_list[0]);
		} else {
			if (oled_weakup_hbmon == 1) {
				oled_alpm_off[6].para_list[0] = 0x28;
			} else {
				oled_alpm_off[6].para_list[0] = 0x20;
			}
			push_table(handle, oled_alpm_off, sizeof(oled_alpm_off) / sizeof(struct LCM_setting_table), 1);
			LCM_INFO("--aod on = %d, udfp = %d\n",  on, oled_weakup_hbmon);
		}
		atomic_set(&doze_mode_state, LCM_PANEL_ON);
	} else if ((on == 1) && (sizeof(oled_alpm_on)) > 0) {
		push_table(handle, oled_alpm_on, sizeof(oled_alpm_on) / sizeof(struct LCM_setting_table), 1);
		atomic_set(&doze_mode_state, LCM_PANEL_AOD);
		LCM_INFO("--aod on = %d\n",  on);
	} else if ((on == 2) && (sizeof(oled_udfp_aod_on)) > 0) {
		if (atomic_read(&oled_hbm_status) == HBM_UDFP_AOD_MODE_ON)  {
			LCM_INFO("can not set --aod on = %d, cause of hbm is on\n",  on);
		} else {
			push_table(handle, oled_udfp_aod_on, sizeof(oled_udfp_aod_on) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&doze_mode_state, LCM_PANEL_AOD);
			LCM_INFO("--aod on = %d\n",  on);
		}
	} else if ((on == 3) && (sizeof(oled_hbm_off_n_aod_on)) > 0) {
		if (atomic_read(&oled_hbm_status) == HBM_UDFP_AOD_MODE_ON)  {
			push_table(handle, oled_hbm_off_n_aod_on, sizeof(oled_hbm_off_n_aod_on) / sizeof(struct LCM_setting_table), 1);
			atomic_set(&oled_hbm_status, HBM_UDFP_AOD_MODE_OFF);
			LCM_INFO("<--hbm off and aod on together--->, --aod on = %d\n", on);
		} else {
			push_table(handle, oled_udfp_aod_on, sizeof(oled_udfp_aod_on) / sizeof(struct LCM_setting_table), 1);
			LCM_INFO("--aod on = %d\n",  on);
		}
		atomic_set(&doze_mode_state, LCM_PANEL_AOD);
	} else if ((on == 4) && (sizeof(aod_display_on)) > 0) {
		push_table(handle, aod_display_on, sizeof(aod_display_on) / sizeof(struct LCM_setting_table), 1);
		LCM_INFO("set aod display on\n");
	}
}
struct LCM_DRIVER pd2062_samsung_s6e3fc3_fhdp_cmd90hz_lcm_drv = {
	.name				= "pd2062_samsung_s6e3fc3_fhdp_cmd90hz_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
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
	.lcm_MipiCmd_HS  = lcm_vivo_MipiCmd_HS,
	.aod = oled_lpm_aod,
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/*DynFPS*/
	.dfps_send_lcm_cmd = lcm_dfps_inform_lcm,
	.dfps_need_send_cmd = lcm_dfps_need_inform_lcm,
#endif
};
