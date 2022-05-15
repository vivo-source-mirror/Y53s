/*  Himax Android Driver Sample Code for common functions

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#ifndef HIMAX_COMMON_H
#define HIMAX_COMMON_H

#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
#include <asm/segment.h>
#endif
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include "himax_platform.h"


/*
#if defined(CONFIG_FB)
	#include <linux/notifier.h>
	#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	#include <linux/earlysuspend.h>
#endif
*/
#ifdef CONFIG_OF
	#include <linux/of_gpio.h>
#endif

#define HIMAX_DRIVER_VER "1.2.2.63_VIVO113A_01.02"

/*#define HX_TP_SELF_TEST_DRIVER*/ /*if enable, selftest works in driver*/

/*===========Himax Option function=============*/

#define HX_VIVO_DEBUG_NODE
#define HX_RST_PIN_FUNC
#define HX_RESUME_SEND_CMD
#define HX_ESD_RECOVERY
#define HX_TP_PROC_GUEST_INFO
#define HX_SMART_WAKEUP
#define HX_GESTURE_TRACK
/*#define HX_HIGH_SENSE*/
/*#define HX_PROTOCOL_A*/	/* for MTK special platform.If turning on,it will report to system by using specific format. */
/*#define HX_RESUME_HW_RESET*/
#define HX_PROTOCOL_B_3PA
#define HX_FIX_TOUCH_INFO	/* if open, you need to change the touch info in the fix_touch_info*/
#define HX_ZERO_FLASH
/*#undef CONFIG_FB*/	/* Enable it if driver go into suspend/resume twice */

#define HX_KEY_MAX_COUNT             4
#define DEFAULT_RETRY_CNT            3

#define HX_CHIP_NOT_DETECT		0x00
#define HX_85XX_H_SERIES_PWON		0x85
#define HX_83112A_SERIES_PWON		0x2a
#define HX_83112B_SERIES_PWON		0x2b
#define HX_83112F_SERIES_PWON		0x2f
#define HX_83113A_SERIES_PWON		0x3a


#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

#define SHIFTBITS 5

#define  FW_SIZE_64k 65536
#define HX_SPI_MTK_MAX_WRITE_SZ (64*1024 + 4)

#define NO_ERR 0
#define READY_TO_SERVE 1
#define WORK_OUT	2
#define XFER_FAIL -1
#define MEM_ALLOC_FAIL -2
#define CHECKSUM_FAIL -3
#define GESTURE_DETECT_FAIL -4
#define INPUT_REGISTER_FAIL -5
#define FW_NOT_READY -6
#define LENGTH_FAIL -7
#define OPEN_FILE_FAIL -8
#define ERR_WORK_OUT	-10
#define ERR_STS_WRONG	-11
#define ERR_TEST_FAIL -12
#define HW_CRC_FAIL 1

#define HX_FINGER_ON	1
#define HX_FINGER_LEAVE	2

#define MINUS_ONE -1

#define HX_MP_FW 	 "Himax_mptest_firmware.bin"
#define HX_NORMAL_FW "Himax_firmware.bin"



enum HX_TS_PATH {
	HX_REPORT_COORD = 1,
	HX_REPORT_SMWP_EVENT,
	HX_REPORT_COORD_RAWDATA,
};

enum HX_TS_STATUS {
	HX_TS_GET_DATA_FAIL = -4,
	HX_ESD_EVENT,
	HX_CHKSUM_FAIL,
	HX_PATH_FAIL,
	HX_TS_NORMAL_END = 0,
	HX_ESD_REC_OK,
	HX_READY_SERVE,
	HX_REPORT_DATA,
	HX_ESD_WARNING,
	HX_IC_RUNNING,
	HX_ZERO_EVENT_COUNT,
	HX_RST_OK,
};

enum cell_type {
	CHIP_IS_ON_CELL,
	CHIP_IS_IN_CELL
};
#ifdef HX_FIX_TOUCH_INFO
enum fix_touch_info {
	FIX_HX_RX_NUM = 36,
	FIX_HX_TX_NUM = 18,
	FIX_HX_BT_NUM = 0,
	FIX_HX_X_RES = 1080,
	FIX_HX_Y_RES = 2340,
	FIX_HX_MAX_PT = 10,
	FIX_HX_XY_REVERSE = true,
	FIX_HX_INT_IS_EDGE = true,
};
#endif
#define TOUCH_MAX_FINGER_NUM 10
#ifdef HX_ZERO_FLASH
	#define HX_SPI_OPERATION
	#define HX_0F_DEBUG
#endif
struct himax_ic_data {
	int vendor_fw_ver;
	int vendor_config_ver;
	int vendor_touch_cfg_ver;
	int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
	int vendor_sensor_id;
	uint8_t vendor_cus_info[12];
	uint8_t vendor_proj_info[12];
	int		HX_RX_NUM;
	int		HX_TX_NUM;
	int		HX_BT_NUM;
	int		HX_X_RES;
	int		HX_Y_RES;
	int		HX_MAX_PT;
	bool	HX_XY_REVERSE;
	bool	HX_INT_IS_EDGE;
};

struct himax_target_report_data {
	int *x;
	int *y;
	int *w;
	int *finger_id;
	int finger_on;
	int finger_num;
#ifdef HX_PLATFORM_DEFINE_KEY
	int key_size;
	int *key_x;
	int *key_y;
	int *key_w;
#endif
#ifdef HX_SMART_WAKEUP
	int SMWP_event_chk;
#endif
};

struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t	finger_num;
	uint8_t	finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[2];
#if defined(HX_SMART_WAKEUP)
	int event_size;
	uint8_t *hx_event_buf;
#endif

	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	uint8_t rawdata_frame_size;
};

struct himax_ts_data {
	bool suspended;
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_cmd;
	int chip_name;
	uint8_t chip_cell_type;
	struct device_node *node;
	struct vts_device *vtsdev;

	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t coordInfoSize;
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];

	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;
	uint16_t old_finger;
	int hx_point_num;


	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	
	int rst_gpio;
	int use_irq;
	int pre_finger_data[10][2];

	struct device *dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct i2c_client *client;
	struct himax_platform_data *pdata;
	struct mutex rw_lock;

	bool cs_bootup;
	bool vddi_poweroff;
	struct pinctrl *pinctrl;
	struct pinctrl_state *spi_cs_active;
	struct pinctrl_state *spi_cs_sleep_pulllow;

/******* SPI-start *******/
	struct mutex	spi_lock;
	struct spi_device	*spi;
	int hx_irq;
	struct mutex w_fw_lock;
/******* SPI-end *******/

	int in_self_test;
/*
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *himax_att_wq;
	struct delayed_work work_att;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
*/
	uint8_t press_id[TOUCH_MAX_FINGER_NUM];
	int finger_cnt;
	int finger_cntbk;


#ifdef HX_ZERO_FLASH
	struct workqueue_struct *himax_0f_update_wq;
	struct delayed_work work_0f_update;
#endif

	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_wrok;

#ifdef HX_SMART_WAKEUP
	uint8_t SMWP_enable;
	uint8_t gesture_cust_en[26];
#endif

#ifdef HX_HIGH_SENSE
	uint8_t HSEN_enable;
#endif

#if defined(HX_USB_DETECT_CALLBACK) || defined(HX_USB_DETECT_GLOBAL) || (1)
	uint8_t usb_connected;
	uint8_t *cable_config;
#endif

#ifdef HX_TP_PROC_GUEST_INFO
	struct workqueue_struct	*guest_info_wq;
	struct work_struct	guest_info_work;
#endif

	int have_update_firmware; // Solve the problem of reboot no function, 0: no, 1: yes
};

struct himax_debug {
	void (*fp_ts_dbg_func)(struct himax_ts_data *ts, int start);
	int (*fp_set_diag_cmd)(struct himax_ic_data *ic_data, struct himax_report_data *hx_touch_data);
};

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

#ifdef HX_HIGH_SENSE
	void himax_set_HSEN_func(uint8_t HSEN_enable);
#endif

#ifdef HX_SMART_WAKEUP
void himax_set_SMWP_func(uint8_t SMWP_enable);

#define GEST_PTLG_ID_LEN	(4)
#define GEST_PTLG_HDR_LEN	(4)
#define GEST_PTLG_HDR_ID1	(0xCC)
#define GEST_PTLG_HDR_ID2	(0x44)
#define GEST_PT_MAX_NUM		(20)
#endif

//extern int irq_enable_count;

/*void himax_HW_reset(uint8_t loadconfig,uint8_t int_off);*/

extern int himax_chip_common_suspend(struct himax_ts_data *ts);
extern int himax_chip_common_resume(struct himax_ts_data *ts);

#endif

/*VIVO start*/
int bbk_himax_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size);
int bbk_himax_gesture_point_get(uint16_t *data);
int himax_report_palm_status(void);
int bbk_himax_get_header_file_version(int which);
int bbk_himax_fw_update(struct vts_device *vtsdev, const struct firmware *firmware);
/*VIVO end*/
