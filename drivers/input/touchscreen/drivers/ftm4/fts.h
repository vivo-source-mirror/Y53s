/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2016, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group)
 *
 * 		marco.cali@st.com
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/sysfs.h>
#include "../vts_core.h"
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsGesture.h"


/*daixiang add for algo-prox sensor start*/
#include	<linux/sensors.h>
/*daixiang add for algo-prox sensor end*/

#define FTS_POWER_ON     1
#define FTS_POWER_OFF    0

/****************** CONFIGURATION SECTION ******************/
/* #define PHONE_KEY */

#define PHONE_GESTURE

#define SCRIPTLESS

#define DRIVER_TEST

#define FW_H_FILE

#ifdef SCRIPTLESS
/*#define SCRIPTLESS_DEBUG */
/* uncomment this macro definition to print debug
message for script less  support
 */
#endif



#define FTS_TS_DRV_NAME                     "stm,ftm4"
#define FTS_TS_DRV_VERSION                  "4.1.2 update 2"


#define X_AXIS_MAX                          1080
#define X_AXIS_MIN                          0
#define Y_AXIS_MAX                          2340
#define Y_AXIS_MIN                          0

#define PRESSURE_MIN                        0
#define PRESSURE_MAX                        127

#define FINGER_MAX                          10
#define STYLUS_MAX                          1
#define TOUCH_ID_MAX                        (FINGER_MAX + STYLUS_MAX)

#define AREA_MIN                            PRESSURE_MIN
#define AREA_MAX                            PRESSURE_MAX

/*********************************************************/
/* Flash programming */

#define INIT_FLAG_CNT				3

/* KEYS */
#define KEY1		0x02
#define KEY2		0x01
#define KEY3		0x04

/*
 * Configuration mode
 */
#define MODE_NORMAL 	                        0x00000000
#define MODE_GESTURE				0x00000001
#define MODE_GLOVE				0x00000002
#define MODE_SENSEOFF				0x00000003

/*
 * Status Event Field:
 *     id of command that triggered the event
 */

#define FTS_FLASH_WRITE_CONFIG              0x03
#define FTS_FLASH_WRITE_COMP_MEMORY         0x04
#define FTS_FORCE_CAL_SELF_MUTUAL           0x05
#define FTS_FORCE_CAL_SELF                  0x06
#define FTS_WATER_MODE_ON                   0x07
#define FTS_WATER_MODE_OFF                  0x08


#define EXP_FN_WORK_DELAY_MS  		1000/*ms */

#define CMD_STR_LEN			32


#ifdef SCRIPTLESS
/*
 * I2C Command Read / Write Function
 */

#define CMD_RESULT_STR_LEN     2048
#endif

#define TSP_BUF_SIZE           4096

struct fts_i2c_platform_data {
	int (*power) (bool on);
	int irq_gpio;
	int reset_gpio;
	int vcc_gpio;
	int power_gpio;
	const char *pwr_reg_name;
	const char *bus_reg_name;

};

/*
 * Forward declaration
 */
struct fts_ts_info;

/*
 * Dispatch event handler
 */
typedef unsigned char *(*event_dispatch_handler_t)(struct fts_ts_info *info, unsigned char *data, ktime_t kt);

/*
 * struct fts_ts_info - FTS capacitive touch screen device information
 * @dev:                  Pointer to the structure device
 * @client:               I2C client structure
 * @input_dev             Input device structure
 * @work                  Work thread
 * @event_wq              Event queue for work thread
 * @cmd_done              Asyncronous command notification
 * @event_dispatch_table  Event dispatch table handlers
 * @fw_version            Firmware version
 * @attrs                 SysFS attributes
 * @mode                  Device operating mode
 * @touch_id              Bitmask for touch id (mapped to input slots)
 * @buttons               Bitmask for buttons status
 * @timer                 Timer when operating in polling mode
 * @early_suspend         Structure for early suspend functions
 * @power                 Power on / off routine
 */
struct fts_ts_info{
	   struct vts_device   *vtsdev;
       struct device            *dev;
       struct i2c_client        *client;
	   chipInfo chipinfo;
	   struct fts_gesture gesture;
	   int byte_count_read;
	   char Out_buff[TSP_BUF_SIZE];
      /*sunsl struct input_dev         *input_dev; */

     /*sunsl  struct work_struct        work; */
     /*sunsl  struct workqueue_struct  *event_wq; */

       struct delayed_work        fwu_work;
       struct workqueue_struct  *fwu_workqueue;
       struct completion         cmd_done;

       event_dispatch_handler_t *event_dispatch_table;

       unsigned int              fw_version;
       unsigned int              config_id;

       struct attribute_group    attrs;
	   struct mutex i2cResetMutex;
       struct mutex esdErrorMutex;
	   int esd_error_count;
	   int esd_error_state;
	   int ktime_start;
	   int ktime_middl;
       int ktime_endof;
	   bool wait_cond;
	   wait_queue_head_t wait;
	   struct task_struct *task;
	   struct wakeup_source *error_wlock;
	   char wakelock_name[32];
	   u32 broken_disable;
	   int irq_state;
	   int irq_wake_state;
	   int gama_mode_state;
	   int nr_err_events;

	   struct class *fts_cmd_class;
	   struct device *i2c_cmd_dev;

       unsigned int              mode;
       unsigned long             touch_id;
       unsigned int              buttons;
	/*sunsl add */
		int large_press_touch_id;
	/*sunsl add end */
#ifdef FTS_USE_POLLING_MODE
		struct hrtimer timer;
#endif

#ifdef DRIVER_TEST
       u32 *functionToTest;
       int numberParam;
#endif

#ifdef SCRIPTLESS
	char cmd_read_result[CMD_RESULT_STR_LEN];
	char cmd_wr_result[CMD_RESULT_STR_LEN];
	char cmd_write_result[20];
#endif

	int (*power)(bool on);

	struct fts_i2c_platform_data bdata;
	struct regulator *pwr_reg;
	int pwr_voltage;
	struct regulator *bus_reg;

	bool fw_force;
	int debug_enable;

	int resume_bit;
	int fwupdate_stat;
	int touch_debug;

	struct notifier_block notifier;
	bool sensor_sleep;
	bool stay_awake;

	/* input lock */
	/*struct mutex input_report_mutex; */

	/*switches */
	int gesture_enabled;
	int glove_enabled;
	int edge_suppress_enabled;
	int edge_pix_reject;/*sunsl */
	int edge_corner_reject;/*sunsl */
	int edge_L_reject;/*sunsl */
/*	int edge_reject;//sunsl */
	int charger_enabled;

};

typedef enum {
    ERR_ITO_NO_ERR,                 /* / < 0 No ITO Error */
    ERR_ITO_PANEL_OPEN_FORCE,       /* / < 1 Panel Open Force */
    ERR_ITO_PANEL_OPEN_SENSE,       /* / < 2 Panel Open Sense */
    ERR_ITO_F2G,                    /* / < 3 Force short to ground */
    ERR_ITO_S2G,                    /* / < 4 Sense short to ground */
    ERR_ITO_F2VDD,                  /* / < 5 Force short to VDD */
    ERR_ITO_S2VDD,                  /* / < 6 Sense short to VDD */
    ERR_ITO_P2P_FORCE,              /* / < 7 Pin to Pin short (Force) */
    ERR_ITO_P2P_SENSE,              /* / < 8 Pin to Pin short (Sense) */
} errItoSubTypes_t;



int ftm4_chip_powercycle(struct fts_ts_info *info);
int fts_get_fw_version(struct fts_ts_info *info);
extern unsigned int ftm4_le_to_uint(const unsigned char *ptr);
extern unsigned int ftm4_be_to_uint(const unsigned char *ptr);
extern int input_register_notifier_client(struct notifier_block *nb);
extern int input_unregister_notifier_client(struct notifier_block *nb);

#ifdef DRIVER_TEST
extern struct attribute_group ftm4_test_cmd_attr_group;
#endif

#ifdef SCRIPTLESS
int ftm4_gui_init(struct fts_ts_info *info);
void ftm4_gui_deinit(struct fts_ts_info *info);
#endif

int ftm4_driver_test_init(struct fts_ts_info *info);
void ftm4_driver_test_deinit(struct fts_ts_info *info);

#define fts_err(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_err(info->vtsdev, fmt, ##param); \
		else \
			VTE(fmt, ##param); \
	} while (0)

#define fts_info(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_info(info->vtsdev, fmt, ##param); \
		else \
			VTI(fmt, ##param); \
	} while (0)

#define fts_dbg(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_dbg(info->vtsdev, fmt, ##param); \
		else \
			VTD(fmt, ##param); \
	} while (0)

#endif
