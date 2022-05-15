/* drivers/input/touchscreen/sec_ts.h
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as	
 * published by the Free Software Foundation.
 */

#ifndef __SEC_TS_H__
#define __SEC_TS_H__

#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
//#include <linux/input/sec_cmd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include "../vts_core.h"


#ifdef CONFIG_INPUT_BOOSTER
#include <linux/input/input_booster.h>
#endif

#define SEC_TS_I2C_NAME		"sec_ts"
#define SEC_TS_DEVICE_NAME	"SEC_TS"
#define I2C_DRIVER_NAME "synaptics_3706"

#undef USE_OPEN_CLOSE
#undef USE_RESET_DURING_POWER_ON
#undef USE_RESET_EXIT_LPM
#define USE_POR_AFTER_I2C_RETRY
#undef USER_OPEN_DWORK
#define USE_PRESSURE_SENSOR
#define PAT_CONTROL

#define CONFIG_VIRTUAL_PROX_ON_TOUCH

#if defined(USE_RESET_DURING_POWER_ON) || defined(USE_POR_AFTER_I2C_RETRY) || defined(USE_RESET_EXIT_LPM)
#define USE_POWER_RESET_WORK
#define RESET_MAX 5
#endif

#define TOUCH_RESET_DWORK_TIME		10
#define BRUSH_Z_DATA		63	/* for ArtCanvas */

#define MASK_1_BITS			0x0001
#define MASK_2_BITS			0x0003
#define MASK_3_BITS			0x0007
#define MASK_4_BITS			0x000F
#define MASK_5_BITS			0x001F
#define MASK_6_BITS			0x003F
#define MASK_7_BITS			0x007F
#define MASK_8_BITS			0x00FF

/* support feature */
//#define SEC_TS_SUPPORT_CUSTOMLIB	/* support user defined library */

#define TYPE_STATUS_EVENT_CMD_DRIVEN	0
#define TYPE_STATUS_EVENT_ERR		1
#define TYPE_STATUS_EVENT_INFO		2
#define TYPE_STATUS_EVENT_USER_INPUT	3
#define TYPE_STATUS_EVENT_CUSTOMLIB_INFO	6
#define TYPE_STATUS_EVENT_VENDOR_INFO	7
#define TYPE_STATUS_CODE_SAR	0x28

/*SEC_TS VIVO CALIBRATION STATUS CAMMAND */
#define SEC_TS_VIVO_STATUS_COMMAND   0X1F
#define SEC_TS_VIVO_SPECIAL_CALI_STATUS_COMMAND   0X1E

#define SEC_TS_VIVO_VSYNC_FREQ 0x2F

#define BIT_STATUS_EVENT_CMD_DRIVEN(a)	(a << TYPE_STATUS_EVENT_CMD_DRIVEN)
#define BIT_STATUS_EVENT_ERR(a)		(a << TYPE_STATUS_EVENT_ERR)
#define BIT_STATUS_EVENT_INFO(a)	(a << TYPE_STATUS_EVENT_INFO)
#define BIT_STATUS_EVENT_USER_INPUT(a)	(a << TYPE_STATUS_EVENT_USER_INPUT)
#define BIT_STATUS_EVENT_VENDOR_INFO(a)	(a << TYPE_STATUS_EVENT_VENDOR_INFO)

#define DO_FW_CHECKSUM			(1 << 0)
#define DO_PARA_CHECKSUM		(1 << 1)
#define MAX_SUPPORT_TOUCH_COUNT		10
#define MAX_SUPPORT_HOVER_COUNT		1

#define SEC_TS_EVENTID_HOVER		10

#define SEC_TS_DEFAULT_FW_NAME		"tsp_sec/sec_hero.fw"
#define SEC_TS_DEFAULT_BL_NAME		"tsp_sec/s6smc41_blupdate_img_REL.bin"
#define SEC_TS_DEFAULT_PARA_NAME	"tsp_sec/s6smc41_para_REL_DGA0_V0106_150114_193317.bin"
#define SEC_TS_DEFAULT_UMS_FW		"/sdcard/Firmware/TSP/lsi.bin"
#define SEC_TS_DEFAULT_FFU_FW		"ffu_tsp.bin"
#define SEC_TS_MAX_FW_PATH		64
#define SEC_TS_FW_BLK_SIZE_MAX		(512)
#define SEC_TS_FW_BLK_SIZE_DEFAULT	(512)
#define SEC_TS_SELFTEST_REPORT_SIZE	80

#define I2C_WRITE_BUFFER_SIZE		(256 - 1)//10

#define SEC_TS_FW_HEADER_SIGN		0x53494654
#define SEC_TS_FW_CHUNK_SIGN		0x53434654

#define SEC_TS_FW_UPDATE_ON_PROBE

#define AMBIENT_CAL			0
#define OFFSET_CAL_SDC			1
#define OFFSET_CAL_SET			2
#define PRESSURE_CAL			3

#define SEC_TS_SKIPTSP_DUTY		100

#define SEC_TS_NVM_OFFSET_FAC_RESULT		0
#define SEC_TS_NVM_OFFSET_CAL_COUNT		1
#define SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT	2
#define SEC_TS_NVM_OFFSET_TUNE_VERSION		3
#define SEC_TS_NVM_OFFSET_TUNE_VERSION_LENGTH	2

#define SEC_TS_NVM_OFFSET_LENGTH		(SEC_TS_NVM_LAST_BLOCK_OFFSET + SEC_TS_NVM_LAST_BLOCK_SIZE + 1)

/* SEC_TS READ REGISTER ADDRESS */
#define SEC_TS_CMD_SENSE_ON			0x10
#define SEC_TS_CMD_SENSE_OFF			0x11
#define SEC_TS_CMD_SW_RESET			0x12
#define SEC_TS_CMD_CALIBRATION_SEC		0x13	// send it to touch ic, but toucu ic works nothing.
#define SEC_TS_CMD_FACTORY_PANELCALIBRATION	0x14
#define SEC_TS_CMD_NG_PANEL         0x18

#define SEC_TS_READ_GPIO_STATUS			0x20	// not support
#define SEC_TS_READ_FIRMWARE_INTEGRITY		0x21
#define SEC_TS_READ_DEVICE_ID			0x22
#define SEC_TS_READ_PANEL_INFO			0x23
#define SEC_TS_READ_CORE_CONFIG_VERSION		0x24
#define SEC_TS_READ_POINT_BYTE_FLAG			0x26

#define SEC_TS_CMD_SET_TOUCHFUNCTION		0x30
#define SEC_TS_CMD_SET_TSC_MODE			0x31
#define SET_TS_CMD_SET_CHARGER_MODE		0x32
#define SET_TS_CMD_SET_NOISE_MODE		0x33
#define SET_TS_CMD_SET_REPORT_RATE		0x34
#define SEC_TS_CMD_TOUCH_MODE_FOR_THRESHOLD	0x35
#define SEC_TS_CMD_TOUCH_THRESHOLD		0x36
#define SET_TS_CMD_KEY_THRESHOLD		0x37
#define SEC_TS_CMD_SET_COVERTYPE		0x38
#define SEC_TS_CMD_WAKEUP_GESTURE_MODE		0x39
#define SEC_TS_WRITE_POSITION_FILTER		0x3A
#define SEC_TS_CMD_WET_MODE			0x3B
#define SEC_TS_CMD_SET_FOD_DETECTION           0x43
#define SEC_TS_CMD_ERASE_FLASH			0x45
#define SEC_TS_CMD_SELF_SENSING_MODE	0x47
#define SEC_TS_CMD_FREQ_SEND		0x48
#define SEC_TS_CMD_VK_LONGPRESS			0x49
#define SEC_TS_CMD_INPUT_METHOD			0x49
#define SEC_TS_CMD_LANDSCAPE_GAMEMODE	0x4A
#define SEC_TS_CMD_ENTER_GAMEMODE		0x4A
#define SEC_TS_CMD_FOD_MODE				0x4B
#define SEC_TS_CMD_IDLE_TIME			0x4C
#define SEC_TS_CMD_AOI_INTPIN_TEST		0x4D
#define SEC_TS_CMD_GAME_OUT_IDLE_TIME		0x4E
#define SEC_TS_CMD_HIGH_FREQ            0x4F
#define SEC_TS_READ_ID				0x52
#define SEC_TS_READ_BOOT_STATUS			0x55
#define SEC_TS_CMD_ENTER_FW_MODE		0x57
#define SEC_TS_CMD_SET_FOD_RANGE		0x5E
#define SEC_TS_CMD_FOD_ENABLE			0x5F
#define SEC_TS_READ_ONE_EVENT			0x60
#define SEC_TS_READ_ALL_EVENT			0x61
#define SEC_TS_CMD_CLEAR_EVENT_STACK	0x62
#define SEC_TS_CMD_VGK_DEADZONE			0x6E
#define SEC_TS_CMD_VIRTUAL_GAMEKEY		0x6F
#define SEC_TS_CMD_MUTU_RAW_TYPE		0x70
#define SEC_TS_CMD_SELF_RAW_TYPE		0x71
#define SEC_TS_READ_TOUCH_RAWDATA		0x72
#define SEC_TS_READ_TOUCH_SELF_RAWDATA		0x73
#define SEC_TS_READ_SELFTEST_RESULT		0x80
#define SEC_TS_CMD_CALIBRATION_AMBIENT		0x81
#define SEC_TS_CMD_P2P_TEST			0x82
#define SEC_TS_CMD_P2P_MODE			0x83
#define SEC_TS_CMD_NVM				0x85
#define SEC_TS_CMD_STATEMANAGE_ON		0x8E
#define SEC_TS_CMD_CALIBRATION_OFFSET_SDC	0x8F

/* SEC_TS CUSTOMLIB OPCODE COMMAND */
#define SEC_TS_CMD_GESTURECOORD		0x63
//#define SEC_TS_CMD_CUSTOMLIB_GET_INFO			0x90
#define SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM			0x91
#define SEC_TS_CMD_CUSTOMLIB_READ_PARAM			0x92
#define SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET			0x93

#define SEC_TS_CMD_STATUS_EVENT_TYPE	0xA0
#define SEC_TS_READ_FW_INFO		0xA2
#define SEC_TS_READ_FW_VERSION		0xA3
#define SEC_TS_READ_PARA_VERSION	0xA4
#define SEC_TS_READ_IMG_VERSION		0xA5
#define SEC_TS_CMD_GET_CHECKSUM		0xA6
#define SEC_TS_CMD_MIS_CAL_CHECK	0xA7
#define SEC_TS_CMD_MIS_CAL_READ		0xA8
#define SEC_TS_CMD_MIS_CAL_SPEC		0xA9

#define SEC_TS_CMD_DEADZONE_RANGE	0xAA
#define SEC_TS_CMD_LONGPRESSZONE_RANGE	0xAB
#define SEC_TS_CMD_EDGE_REJECT_RANGE	0xAC
#define SEC_TS_CMD_GRIP_MODE_RANGE	0xAD

#define SEC_TS_READ_TS_STATUS		0xAF
#define SEC_TS_CMD_SELFTEST		0xAE
#define SEC_TS_READ_FORCE_RECAL_COUNT	0xB0
#define SEC_TS_READ_FORCE_SIG_MAX_VAL	0xB1
#define SEC_TS_READ_IS_NORMAL_MODE	0x02
#define SEC_TS_READ_IS_GESTURE_MODE	0x05
#define SEC_TS_CMD_FINGER_CENTER 0xBF

/* SEC_TS FLASH COMMAND */
#define SEC_TS_CMD_FLASH_READ_ADDR	0xD0
#define SEC_TS_CMD_FLASH_READ_SIZE	0xD1
#define SEC_TS_CMD_FLASH_READ_DATA	0xD2
#define SEC_TS_CMD_FW_RESOLUTION    0xD4
#define SEC_TS_CMD_CHG_SYSMODE		0xD7
#define SEC_TS_CMD_FLASH_ERASE		0xD8
#define SEC_TS_CMD_FLASH_WRITE		0xD9
#define SEC_TS_CMD_FLASH_PADDING	0xDA

#define SEC_TS_READ_BL_UPDATE_STATUS	0xDB
#define SEC_TS_CMD_SET_POWER_MODE	0xE4
#define SEC_TS_CMD_EDGE_DEADZONE	0xE5
#define SEC_TS_CMD_SET_DEX_MODE		0xE7
#define SEC_TS_CMD_CALIBRATION_PRESSURE		0xE9
/* Have to need delay 30msec after writing 0xEA command */
/* Do not write Zero with 0xEA command */
#define SEC_TS_CMD_SET_GET_PRESSURE		0xEA
#define SEC_TS_CMD_SET_USER_PRESSURE		0xEB
#define SEC_TS_CMD_SET_TEMPERATURE_COMP_MODE	0xEC
#define SEC_TS_CMD_SET_TOUCHABLE_AREA		0xED
#define SEC_TS_CMD_SET_BRUSH_MODE		0xEF
#define SEC_TS_CMD_SET_CALLMODE                        0xEA

#define SEC_TS_READ_CALIBRATION_REPORT		0xF1
#define SEC_TS_CMD_SET_VENDOR_EVENT_LEVEL	0xF2
#define SEC_TS_CMD_SET_SPENMODE			0xF3
#define SEC_TS_CMD_SELECT_PRESSURE_TYPE		0xF5
#define SEC_TS_CMD_READ_PRESSURE_DATA		0xF6

#define SEC_TS_FLASH_SIZE_64		64
#define SEC_TS_FLASH_SIZE_128		128
#define SEC_TS_FLASH_SIZE_256		256

#define SEC_TS_FLASH_SIZE_CMD		1
#define SEC_TS_FLASH_SIZE_ADDR		2
#define SEC_TS_FLASH_SIZE_CHECKSUM	1

#define SEC_TS_STATUS_BOOT_MODE		0x10
#define SEC_TS_STATUS_APP_MODE		0x20

#define SEC_TS_FIRMWARE_PAGE_SIZE_256	256
#define SEC_TS_FIRMWARE_PAGE_SIZE_128	128

/* SEC status event id */
#define SEC_TS_COORDINATE_EVENT		0
#define SEC_TS_STATUS_EVENT		1
#define SEC_TS_GESTURE_EVENT		2
#define SEC_TS_EMPTY_EVENT		3

#define SEC_TS_EVENT_BUFF_SIZE		8
#define SEC_TS_EVENT_DOUBLE_BUFF_SIZE		16
#define SEC_TS_SID_GESTURE		0x14
#define SEC_TS_GESTURE_CODE_SPAY		0x00
#define SEC_TS_GESTURE_CODE_DOUBLE_TAP		0x01

#define SEC_TS_REPORT_8_BYTE	0
#define SEC_TS_REPORT_16_BYTE	2

typedef enum gesture_code {
   TOUCH_GESTURE_DOUBLTAP  = 0xF,
   TOUCH_GESTURE_CLOCK = 0x10,
   TOUCH_GESTURE_W                 = 0x13,
   TOUCH_GESTURE_O                 = 0xB,
   TOUCH_GESTURE_M                 = 9,
   TOUCH_GESTURE_E                 = 5,
   TOUCH_GESTURE_F                 = 6,
   TOUCH_GESTURE_AT                        = 0x15,
   TOUCH_GESTURE_C                 = 3,
   TOUCH_GESTURE_UP                        = 0x1E,
   TOUCH_GESTURE_DOWN              = 0x1F,
   TOUCH_GESTURE_LEFT              = 0x20,
   TOUCH_GESTURE_RIGHT             = 0x21,
   TOUCH_GESTURE_DOUBLTAP_EDGE		= 0x22,
}TOUCH_GESTURE_CODE;

enum gesture_id {
	GESTURE_DOUBLE_CLICK = 0,
	GESTURE_UP_SLIDE,
	GESTURE_DOWN_SLIDE,
	GESTURE_LEFT_SLIDE,
	GESTURE_RIGHT_SLIDE,
	GESTURE_E,
	GESTURE_F,
	GESTURE_W,
	GESTURE_O,
	GESTURE_C,
	GESTURE_A,		// GESTURE @
	GESTURE_M = 11
};

#define SEC_TS_COORDINATE_ACTION_NONE		0
#define SEC_TS_COORDINATE_ACTION_PRESS		1
#define SEC_TS_COORDINATE_ACTION_MOVE		2
#define SEC_TS_COORDINATE_ACTION_RELEASE	3

#define SEC_TS_TOUCHTYPE_NORMAL		0
#define SEC_TS_TOUCHTYPE_HOVER		1
#define SEC_TS_TOUCHTYPE_FLIPCOVER	2
#define SEC_TS_TOUCHTYPE_GLOVE		3
#define SEC_TS_TOUCHTYPE_STYLUS		4
#define SEC_TS_TOUCHTYPE_PALM		5
#define SEC_TS_TOUCHTYPE_WET		6
#define SEC_TS_TOUCHTYPE_PROXIMITY	7
#define SEC_TS_TOUCHTYPE_JIG		8

/* SEC_TS_INFO : Info acknowledge event */
#define SEC_TS_ACK_BOOT_COMPLETE		0x00
#define SEC_TS_ACK_WET_MODE				0x1
#define SEC_TS_ACK_CALL_DETECT			0x01
#define SEC_TS_ACK_CALL_RELEASE			0x00
#define SEC_TS_ACK_FOD_DETECT			0x01
#define SEC_TS_ACK_FOD_RELEASE			0x00
#define SEC_TS_ACK_PALM_DETECT			0x01
#define SEC_TS_ACK_PALM_RELEASE			0x00
#define SEC_TS_ACK_INSIDE_SLIDE_DOWN	0x01
#define SEC_TS_ACK_INSIDE_SLIDE_RELEASE	0x00
#define SEC_TS_ACK_INSIDE_SLIDE_LEFT	0x01
#define SEC_TS_ACK_INSIDE_SLIDE_RIGHT	0x02
#define SEC_TS_ACK_FOD_AREA_INFO_X		0x00
#define SEC_TS_ACK_FOD_AREA_INFO_Y		0x01


/* SEC_TS_VENDOR_INFO : Vendor acknowledge event */
#define SEC_TS_VENDOR_ACK_HSYNC_IMMIGRATE	0x30
#define SEC_TS_VENDOR_ACK_OFFSET_CAL_DONE	0x40
#define SEC_TS_VENDOR_ACK_SELF_TEST_DONE	0x41
#define SEC_TS_VENDOR_ACK_CMR_TEST_DONE			0x42	/* mutual */

#define SEC_TS_VENDOR_ACK_NOISE_STATUS_NOTI		0x64
#define SEC_TS_VENDOR_ACK_CALLMODE_EVENT               0x6A
#define SEC_TS_VENDOR_ACK_FOD_EVENT                    0x6B
#define SEC_TS_VENDOR_ACK_LONGPRESS_EVENT		0x6C
#define SEC_TS_VENDOR_ACK_QUIT_ACTIVE_EVENT		0x6E
#define SEC_TS_VENDOR_ACK_FOD_AREA_INFO			0x6F
#define SEC_TS_VENDOR_ACK_PALM_EVENT			0x70
#define SEC_TS_VENDOR_ACK_FOD_DETECT_INFO		0x71


/* SEC_TS_STATUS_EVENT_USER_INPUT */
#define SEC_TS_EVENT_FORCE_KEY	0x1

/* SEC_TS_STATUS_EVENT_CUSTOMLIB_INFO */
#define SEC_TS_EVENT_CUSTOMLIB_FORCE_KEY	0x00

/* SEC_TS_ERROR : Error event */
#define SEC_TS_ERR_EVNET_CORE_ERR	0x0
#define SEC_TS_ERR_EVENT_QUEUE_FULL	0x01
#define SEC_TS_ERR_EVENT_ESD		0x2

/* SEC_TS_DEBUG : Print event contents */
#define SEC_TS_DEBUG_PRINT_ALLEVENT	0x01
#define SEC_TS_DEBUG_PRINT_ONEEVENT	0x02
#define SEC_TS_DEBUG_PRINT_I2C_READ_CMD		0x04
#define SEC_TS_DEBUG_PRINT_I2C_WRITE_CMD	0x08
#define SEC_TS_DEBUG_SEND_UEVENT	0x80

#define SEC_TS_BIT_SETFUNC_TOUCH		(1 << 0)
#define SEC_TS_BIT_SETFUNC_MUTUAL		(1 << 0)
#define SEC_TS_BIT_SETFUNC_HOVER		(1 << 1)
#define SEC_TS_BIT_SETFUNC_COVER		(1 << 2)
#define SEC_TS_BIT_SETFUNC_GLOVE		(1 << 3)
#define SEC_TS_BIT_SETFUNC_STYLUS		(1 << 4)
#define SEC_TS_BIT_SETFUNC_PALM			(1 << 5)
#define SEC_TS_BIT_SETFUNC_WET			(1 << 6)
#define SEC_TS_BIT_SETFUNC_PROXIMITY		(1 << 7)

#define SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC	(SEC_TS_BIT_SETFUNC_TOUCH | SEC_TS_BIT_SETFUNC_PALM | SEC_TS_BIT_SETFUNC_WET)

#define SEC_TS_BIT_CHARGER_MODE_NO			(0x1 << 0)
#define SEC_TS_BIT_CHARGER_MODE_WIRE_CHARGER		(0x1 << 1)
#define SEC_TS_BIT_CHARGER_MODE_WIRELESS_CHARGER	(0x1 << 2)
#define SEC_TS_BIT_CHARGER_MODE_WIRELESS_BATTERY_PACK	(0x1 << 3)

#define STATE_MANAGE_ON			1
#define STATE_MANAGE_OFF		0

#define SEC_TS_STATUS_NOT_CALIBRATION	0x50
#define SEC_TS_STATUS_CALIBRATION_SDC	0xA1
#define SEC_TS_STATUS_CALIBRATION_SEC	0xA2

#define SET_TS_CMD_EDGE_ZONE_SELETE		0x64
#define SET_TS_CMD_EDGE_ZONE_SET		0x66
#define SET_TS_CMD_EDGE_ZONE_DEAD		0x67
#define SET_TS_CMD_EDGE_ZONE_LP			0x68
#define SET_TS_CMD_ACTIVE_ZONE_AREA		0x6A
#define SET_TS_CMD_SCREEN_CLOCK_AREA	0x6B
#define SET_TS_CMD_SCREEN_CLOCK_ENABLE	0x6C

#define SEC_TS_CMD_DEADZONE_RANGE        0xAA
#define SEC_TS_CMD_LONGPRESS_RANGE       0xAB
#define SEC_TS_CMD_ONEHAND_RANGE         0xAC
#define SEC_TS_CMD_GRIP_ORIENTATION      0xAD


enum grip_write_mode {
	G_NONE				= 0,
	G_SET_EDGE_HANDLER		= 1,
	G_SET_EDGE_ZONE			= 2,
	G_SET_NORMAL_MODE		= 4,
	G_SET_LANDSCAPE_MODE	= 8,
	G_CLR_LANDSCAPE_MODE	= 16,
};
enum grip_set_data {
	ONLY_EDGE_HANDLER		= 0,
	GRIP_ALL_DATA			= 1,
};

enum edge_area_for_func {
	EDGE_FUNC_DEBUG = 0,
	EDGE_FUNC_NAIL,
	EDGE_FUNC_VIRTUAL_KEY,
	EDGE_FUNC_VIRTUAL_KEY_SINGLE,
	EDGE_FUNC_VIRTUAL_KEY_DOUBLE,
	EDGE_FUNC_REPORT_POINT_LEFT,
	EDGE_FUNC_REPORT_POINT_RIGHT,
	EDGE_FUNC_REPORT_POINT_DOUBLE,
	EDGE_FUNC_NUM
};

enum edge_area_for_save {
	EDGE_SAVE_DEBUG = 0,
	EDGE_SAVE_NAIL,
	EDGE_SAVE_VIRTUAL_KEY,
	EDGE_SAVE_VIRTUAL_KEY_GESTURE,
	EDGE_SAVE_REPORT_POINT,
	EDGE_SAVE_NUM
};
	
enum edge_area_setting {
	EDGE_SCREEN_OFF = 0,
	EDGE_SCREEN_APP1,
	EDGE_SCREEN_APP2,
	EDGE_SCREEN_APPALL
};

typedef enum {
	SEC_TS_STATE_POWER_OFF = 0,
	SEC_TS_STATE_LPM,
	SEC_TS_STATE_POWER_ON
} TOUCH_POWER_MODE;

typedef enum {
	TOUCH_SYSTEM_MODE_BOOT		= 0,
	TOUCH_SYSTEM_MODE_CALIBRATION	= 1,
	TOUCH_SYSTEM_MODE_TOUCH		= 2,
	TOUCH_SYSTEM_MODE_SELFTEST	= 3,
	TOUCH_SYSTEM_MODE_FLASH		= 4,
	TOUCH_SYSTEM_MODE_LOWPOWER	= 5,
	TOUCH_SYSTEM_MODE_LISTEN
} TOUCH_SYSTEM_MODE;

typedef enum {
	TOUCH_MODE_STATE_IDLE		= 0,
	TOUCH_MODE_STATE_HOVER		= 1,
	TOUCH_MODE_STATE_TOUCH		= 2,
	TOUCH_MODE_STATE_NOISY		= 3,
	TOUCH_MODE_STATE_CAL		= 4,
	TOUCH_MODE_STATE_CAL2		= 5,
	TOUCH_MODE_STATE_WAKEUP		= 10
} TOUCH_MODE_STATE;

enum switch_system_mode {
	TO_TOUCH_MODE			= 0,
	TO_LOWPOWER_MODE		= 1,
	TO_SELFTEST_MODE		= 2,
	TO_FLASH_MODE			= 3,
};

enum {
	TYPE_RAW_DATA			= 0,	/* Total - Offset : delta data */
	TYPE_SIGNAL_DATA		= 1,	/* Signal - Filtering & Normalization */
	TYPE_AMBIENT_BASELINE	= 2,	/* Cap Baseline */
	TYPE_AMBIENT_DATA		= 3,	/* Cap Ambient */
	TYPE_REMV_BASELINE_DATA	= 4,
	TYPE_DECODED_DATA		= 5,	/* Raw */
	TYPE_REMV_AMB_DATA		= 6,	/*  TYPE_RAW_DATA - TYPE_AMBIENT_DATA */
	TYPE_OFFSET_DATA_SET	= 19,	/* Cap Offset in SET Manufacturing Line */
	TYPE_OFFSET_DATA_SDC	= 29,	/* Cap Offset in SDC Manufacturing Line */
	TYPE_RAW_DATA_P2P_MIN		= 30,	/* Raw min data for 100 frame */
	TYPE_RAW_DATA_P2P_MAX		= 31,	/* Raw max data for 100 frame */
	TYPE_RAWDATA_MAX,
	TYPE_INVALID_DATA		= 0xFF,	/* Invalid data type for release factory mode */
};

typedef enum {
	CUSTOMLIB_EVENT_TYPE_SPAY			= 0x04,
	CUSTOMLIB_EVENT_TYPE_PRESSURE_TOUCHED = 0x05,
	CUSTOMLIB_EVENT_TYPE_PRESSURE_RELEASED	= 0x06,
	CUSTOMLIB_EVENT_TYPE_AOD			= 0x08,
	CUSTOMLIB_EVENT_TYPE_AOD_PRESS		= 0x09,
	CUSTOMLIB_EVENT_TYPE_AOD_LONGPRESS		= 0x0A,
	CUSTOMLIB_EVENT_TYPE_AOD_DOUBLETAB		= 0x0B,
	CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_PRESS	= 0x0C,
	CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RELEASE	= 0x0D,
	CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RELEASE_NO_HAPTIC	= 0x0E
} CUSTOMLIB_EVENT_TYPE;

#define CMD_RESULT_WORD_LEN		10
#define SEC_CMD_STR_LEN			256

#define SEC_TS_I2C_RETRY_CNT		3
#define SEC_TS_WAIT_RETRY_CNT		150
#define SEC_TS_WAIT_RESET_RETRY_CNT		5

#define SEC_TS_MODE_CUSTOMLIB_SPAY			(1 << 1)
#define SEC_TS_MODE_CUSTOMLIB_AOD			(1 << 2)
#define SEC_TS_MODE_CUSTOMLIB_FORCE_KEY	(1 << 6)

#define SEC_TS_MODE_LOWPOWER_FLAG			(SEC_TS_MODE_CUSTOMLIB_SPAY | SEC_TS_MODE_CUSTOMLIB_AOD \
											| SEC_TS_MODE_CUSTOMLIB_FORCE_KEY)

#define SEC_TS_CUSTOMLIB_EVENT_PRESSURE_TOUCHED		(1 << 6)
#define SEC_TS_CUSTOMLIB_EVENT_PRESSURE_RELEASED		(1 << 7)

enum sec_ts_cover_id {
	SEC_TS_FLIP_WALLET = 0,
	SEC_TS_VIEW_COVER,
	SEC_TS_COVER_NOTHING1,
	SEC_TS_VIEW_WIRELESS,
	SEC_TS_COVER_NOTHING2,
	SEC_TS_CHARGER_COVER,
	SEC_TS_VIEW_WALLET,
	SEC_TS_LED_COVER,
	SEC_TS_CLEAR_FLIP_COVER,
	SEC_TS_QWERTY_KEYBOARD_EUR,
	SEC_TS_QWERTY_KEYBOARD_KOR,
	SEC_TS_MONTBLANC_COVER = 100,
};

enum sec_fw_update_status {
	SEC_NOT_UPDATE = 0,
	SEC_NEED_FW_UPDATE,
	SEC_NEED_CALIBRATION_ONLY,
	SEC_NEED_FW_UPDATE_N_CALIBRATION,
};

enum tsp_hw_parameter {
	TSP_ITO_CHECK		= 1,
	TSP_RAW_CHECK		= 2,
	TSP_MULTI_COUNT		= 3,
	TSP_WET_MODE		= 4,
	TSP_COMM_ERR_COUNT	= 5,
	TSP_MODULE_ID		= 6,
};

#define TEST_MODE_MIN_MAX		false
#define TEST_MODE_ALL_NODE		true
#define TEST_MODE_READ_FRAME		false
#define TEST_MODE_READ_CHANNEL		true

/* factory test mode */
struct sec_ts_test_mode {
	u8 type;
	short min;
	short max;
	bool allnode;
	bool frame_channel;
};

struct sec_ts_fw_file {
	u8 *data;
	u32 pos;
	size_t size;
};

/*
 * write 0xE4 [ 11 | 10 | 01 | 00 ]
 * MSB <-------------------> LSB
 * read 0xE4
 * mapping sequnce : LSB -> MSB
 * struct sec_ts_test_result {
 * * assy : front + OCTA assay
 * * module : only OCTA
 *	 union {
 *		 struct {
 *			 u8 assy_count:2;		-> 00
 *			 u8 assy_result:2;		-> 01
 *			 u8 module_count:2;	-> 10
 *			 u8 module_result:2;	-> 11
 *		 } __attribute__ ((packed));
 *		 unsigned char data[1];
 *	 };
 *};
 */
struct sec_ts_test_result {
	union {
		struct {
			u8 assy_count:2;
			u8 assy_result:2;
			u8 module_count:2;
			u8 module_result:2;
		} __attribute__ ((packed));
		unsigned char data[1];
	};
};

/* 8 byte */
struct sec_ts_gesture_status {
	u8 eid:2;
	u8 stype:4;
	u8 sf:2;
	u8 gesture_id;
	u8 gesture_point_num;
	u8 gesture_data_2;
	u8 gesture_data_3;
	u8 gesture_data_4;
	u8 reserved_1;
	u8 left_event_5_0:6;
	u8 reserved_2:2;
} __attribute__ ((packed));

/* 8 byte */
struct sec_ts_event_status {
	u8 eid:2;
	u8 stype:4;
	u8 sf:2;
	u8 status_id;
	u8 status_data_1;
	u8 status_data_2;
	u8 status_data_3;
	u8 status_data_4;
	u8 status_data_5;
	u8 left_event_5_0:6;
	u8 reserved_2:2;
} __attribute__ ((packed));

/* 8 byte */
struct sec_ts_event_coordinate {
	u8 eid:2;
	u8 tid:4;
	u8 tchsta:2;
	u8 x_11_4;
	u8 y_11_4;
	u8 y_3_0:4;
	u8 x_3_0:4;
	u8 major;
	u8 minor;
	u8 z:6;
	u8 ttype_3_2:2;
	u8 left_event:6;
	u8 ttype_1_0:2;
	u8 noise_level:8;
	u8 max_sense:8;
	u8 hover_id_3_0:4;
	u8 reverved_0:4;
	u8 x_15_12:4;
	u8 reverved_1:4;
	u8 y_15_12:4;
	u8 reverved_2:4;
	u8 tx_freq_0_7:8;
	u8 tx_freq_8_15:8;
} __attribute__ ((packed));

/* not fixed */
struct sec_ts_coordinate {
	u8 id;
	u8 ttype;
	u8 action;
	u16 x;
	u16 y;
	u8 z;
	u8 hover_flag;
	u8 glove_flag;
	u8 touch_height;
	u16 mcount;
	u8 major;
	u8 minor;
	bool palm;
	int palm_count;
	u8 left_event;
};

enum sec_ts_edge_reject_para_type {
	EDGE_NORMAL = 0,
	EDGE_NAIL,
	EDGE_VIRTUAL_KEY,
	EDGE_VIRTUAL_KEY_SINGLE,
	EDGE_VIRTUAL_KEY_DOUBLE,

	EDGE_PARA_NUM
};

struct sec_ts_edge_reject_para {
	int reject_height_max;
	int reject_top;
	int reject_buttom;
};

enum sec_ts_pinctrl {
	VTS_INT_ACTIVE,
	VTS_INT_SUSPEND,
	VTS_RESET_ACTIVE,
	VTS_RESET_SUSPEND,
	VTS_PINCTRL_STATE_MAX,
};

#define MAX_EVENT_COUNT 128

struct sec_ts_data {
	u32 isr_pin;

	u32 flash_page_size;
	u8 boot_ver[3];
	struct vts_device *vtsdev;

	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *input_dev_pad;
	struct input_dev *input_dev_touch;
	struct sec_ts_plat_data *plat_data;
	struct sec_ts_coordinate coord[MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT];
	struct timespec64 time_pressed[MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT];
	struct timespec64 time_released[MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT];

	long time_longest;

	u8 ic_info[8];

	u8 lowpower_mode;
	u8 lowpower_status;
	u8 dex_mode;
	char *dex_name;
	u8 brush_mode;
	u8 touchable_area;
	volatile u8 touch_noise_status;
	volatile bool input_closed;

	int touch_count;
	int tx_count;
	int rx_count;
	int i2c_burstmax;
	int ta_status;
	volatile int power_status;
	int raw_status;
	int touchkey_glove_mode_status;
	u16 touch_functions;
	u8 charger_mode;
	u8 irq_wake_flag;
	struct sec_ts_event_coordinate touchtype;
	bool touched[11];
	u8 gesture_status[6];
	u8 read_gesture_point_num;
	u8 cal_status;
	struct mutex lock;
	struct mutex device_mutex;
	struct mutex i2c_mutex;
	struct mutex eventlock;
	struct mutex modechange;
	u8 sec_test_flag;
	u8 mode_change_retry_flag;
	
	struct vts_edge_cmd cmd_store[EDGE_SAVE_NUM];
	struct sec_ts_edge_reject_para edge_para[EDGE_PARA_NUM];
	struct pinctrl_state *pin_state[VTS_PINCTRL_STATE_MAX];
	u8 edge_app_flag;

	u8 self_sensing_mode;

	int gamekey_size[2];
	u8 vgk_deadzone[2];

	int fod_area_info[4];  // x1,x2,y1,y2
	int fod_area_size[2];  // in,out
	int fod_count;
	int fod_index;
	int fod_touch_id;
	int fod_area_percent;
	int fod_avg_signal;
	int fod_finger_count;

	struct delayed_work work_read_info;
#ifdef USE_POWER_RESET_WORK
	struct delayed_work reset_work;
	volatile bool reset_is_on_going;
	struct wakeup_source *reset_wakelock;
#endif
	struct completion resume_done;
	//struct wake_lock wakelock;
	//struct sec_cmd_data sec;
	short *pFrame;

	bool probe_done;
	bool reinit_done;
	bool flip_enable;
	bool info_work_done;
	bool large_press;
	bool longpress_gesture;
	int inside_slide;
	int cover_type;
	u8 cover_cmd;
	u16 rect_data[4];

	int tspid_val;
	int tspicid_val;

	unsigned int scrub_id;
	unsigned int scrub_x;
	unsigned int scrub_y;

	u8 grip_edgehandler_direction;
	int grip_edgehandler_start_y;
	int grip_edgehandler_end_y;
	u16 grip_edge_range;
	u8 grip_deadzone_up_x;
	u8 grip_deadzone_dn_x;
	int grip_deadzone_y;
	u8 grip_landscape_mode;
	int grip_landscape_edge;
	u16 grip_landscape_deadzone;

#if 1//def CONFIG_TOUCHSCREEN_DUMP_MODE
	struct delayed_work ghost_check;
	u8 tsp_dump_lock;
#endif

	int nv;
	int cal_count;
	int tune_fix_ver;
	bool external_factory;

	int wet_mode;

	unsigned char ito_test[4];		/* ito panel tx/rx chanel */
	unsigned char check_multi;
	unsigned int multi_count;		/* multi touch count */
	unsigned int wet_count;			/* wet mode count */
	unsigned int noise_count;		/* noise mode count */
	unsigned int dive_count;		/* dive mode count */
	unsigned int comm_err_count;	/* i2c comm error count */
	unsigned int large_press_count; /* large press count */
	unsigned int checksum_result;	/* checksum result */
	unsigned char module_id[4];
	unsigned int all_finger_count;
	unsigned int all_force_count;
	unsigned int all_aod_tap_count;
	unsigned int all_spay_count;
	unsigned int max_z_value;
	unsigned int min_z_value;
	unsigned int sum_z_value;
	unsigned char pressure_cal_base;
	unsigned char pressure_cal_delta;


	/* for factory - factory_cmd_result_all() */
	short cm_raw_set_p2p_min;			//CM_RAW_SET_P2P
	short cm_raw_set_p2p_max;			//CM_RAW_SET_P2P
	short cm_raw_set_p2p_diff;		//CM_RAW_SET_P2P_DIFF

#ifdef USE_PRESSURE_SENSOR
	short pressure_left;
	short pressure_center;
	short pressure_right;
	u8 pressure_user_level;
#endif
	int debug_flag;

	u8 report_byte_flag;
	int event_buf_size;
	int dbclick_x;
	int dbclick_y;

	int game_mode_in;
	int tx_freq_index;

	unsigned int reset_count;
	struct class *sec_class;
	int last_state;
	int (*sec_ts_i2c_write)(struct sec_ts_data *ts, u8 reg, u8 *data, int len);
	int (*sec_ts_i2c_read)(struct sec_ts_data *ts, u8 reg, u8 *data, int len);
	int (*sec_ts_i2c_write_burst)(struct sec_ts_data *ts, u8 *data, int len);
	int (*sec_ts_i2c_read_bulk)(struct sec_ts_data *ts, u8 *data, int len);
};

struct sec_ts_plat_data {
	int max_x;
	int max_y;
	unsigned irq_gpio;
	unsigned reset_gpio;
	unsigned sda_gpio;
	unsigned scl_gpio;
	u32 irq_flags;
	int irq_type;
	int i2c_burstmax;
	int always_lpmode;
	int bringup;
	int mis_cal_check;

	const char *firmware_name;
	const char *model_name;
	const char *project_name;
	//const char *regulator_dvdd;
	//const char *regulator_avdd;
	struct regulator *regulator_dvdd;
	struct regulator *regulator_avdd;

	unsigned int power_gpio;
	unsigned int iovcc_gpio;

	u32 panel_revision;
	u8 core_version_of_ic[4];
	u8 core_version_of_bin[4];
	u8 config_version_of_ic[4];
	u8 config_version_of_bin[4];
	u8 img_version_of_ic[4];
	u8 img_version_of_bin[4];

	struct pinctrl *pinctrl;

	int (*power)(void *data, bool on);
	void (*enable_sync)(bool on);
	int tsp_icid;
	int tsp_id;
	int tsp_vsync;

	bool regulator_boot_on;
	bool support_mt_pressure;
	bool support_dex;
	bool support_sidegesture;
	bool enabled;
};

typedef struct {
	u32 signature;			/* signature */
	u32 version;			/* version */
	u32 totalsize;			/* total size */
	u32 checksum;			/* checksum */
	u32 img_ver;			/* image file version */
	u32 img_date;			/* image file date */
	u32 img_description;		/* image file description */
	u32 fw_ver;			/* firmware version */
	u32 fw_date;			/* firmware date */
	u32 fw_description;		/* firmware description */
	u32 para_ver;			/* parameter version */
	u32 para_date;			/* parameter date */
	u32 para_description;		/* parameter description */
	u32 num_chunk;			/* number of chunk */
	u32 reserved1;
	u32 reserved2;
} fw_header;

typedef struct {
	u32 signature;
	u32 addr;
	u32 size;
	u32 reserved;
} fw_chunk;

int sec_ts_power(void *data, bool on);
int sec_ts_stop_device(struct sec_ts_data *ts, bool hardware);
int sec_ts_start_device(struct sec_ts_data *ts, bool hardware);
int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode);
int sec_ts_firmware_update_on_probe(struct sec_ts_data *ts, bool force_update);
int sec_ts_firmware_update_on_hidden_menu(struct sec_ts_data *ts, int update_type);
int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode);
int sec_ts_set_charger(struct sec_ts_data *ts, int charge_bit);
int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable);
int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack);
int sec_ts_function(int (*func_init)(void *device_data), void (*func_remove)(void));
int sec_ts_fn_init(struct sec_ts_data *ts);
int sec_ts_execute_force_calibration(struct sec_ts_data *ts, int cal_mode);
int sec_ts_fix_tmode(struct sec_ts_data *ts, u8 mode, u8 state);
int sec_ts_release_tmode(struct sec_ts_data *ts);
int get_user_nvm_data(struct sec_ts_data *ts, u8 *data);
int set_user_nvm_data(struct sec_ts_data *ts, u8 *data);
void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts);
void sec_ts_locked_release_all_finger(struct sec_ts_data *ts);
void sec_ts_fn_remove(struct sec_ts_data *ts);
void sec_ts_delay(unsigned int ms);
int sec_ts_read_information(struct sec_ts_data *ts);

int sec_ts_run_rawdata_all(struct sec_ts_data *ts);

void sec_ts_reinit(struct sec_ts_data *ts);

extern int dead_zone_enable(struct vts_device *vtsdev, u8 orient);
extern int get_fw_ver_ic(void *device_data);
extern int get_config_ver(void *device_data);

extern void run_trx_short_test(void *device_data, u8 *data);
extern int sec_ts_firmware_update(struct sec_ts_data *ts, const u8 *data,
							size_t size, int bl_update, int restore_cal, int retry);
extern int sec_ts_save_version_of_ic(struct sec_ts_data *ts);
extern int sec_ts_charger_config(struct sec_ts_data *ts, int status);
//extern int sec_ts_i2c_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len);

extern int run_rawdata_read_all(void *device_data);
extern int run_delta_read_all(void *device_data);
extern int run_reference_read_all(void *device_data);
extern int run_ambient_read_all(void *device_data);
extern int run_self_rawdata_read_all(void *device_data);
extern int run_self_delta_read_all(void *device_data);
extern int run_self_reference_read_all(void *device_data);
extern int run_self_ambient_read_all(void *device_data);
extern int sec_ts_get_special_cal_status_on_probe(struct vts_device *vtsdev);


int bbk_slsi_sec_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size);
int bbk_slsi_fw_update(struct vts_device *vtsdev, const struct firmware *firmware);
int bbk_slsi_set_auto_idle(struct vts_device *vtsdev, int state);

int bbk_slsi_mode_change(struct vts_device *vtsdev, int which);
int bbk_slsi_get_fw_version(struct vts_device *vtsdev, u64 *version);
int bbk_slsi_gesture_point_handle(struct vts_device *vtsdev, u8 id);
int bbk_slsi_set_charger_bit(struct vts_device *vtsdev, int state);
int bbk_slsi_read_charger_bit(void);
int bbk_slsi_get_module_id(void);
int bbk_slsi_sensor_test(struct vts_device *vtsdev, char *buf,  int at_sensor_test_cmd, void *pdata, int tmp);
int bbk_slsi_setEdgeRestainSwitch(struct vts_device *vtsdev, int on);
#if (1)//!defined(CONFIG_SAMSUNG_PRODUCT_SHIP)

int sec_ts_raw_device_init(struct sec_ts_data *ts);
#endif

extern bool tsp_init_done;

extern struct sec_ts_data *g_ts_data;

#ifdef CONFIG_BATTERY_SAMSUNG
extern unsigned int lpcharge;
#endif

extern void set_grip_data_to_ic(struct sec_ts_data *ts, u8 flag);
extern void sec_ts_set_grip_type(struct sec_ts_data *ts, u8 set_type);


#endif
