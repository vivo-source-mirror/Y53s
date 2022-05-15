/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include "focaltech_common.h"
#include "../vts_core.h"
#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT              10	/* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_DIM                         10
#define FTS_ONE_TCH_LEN                     6
#define FTS_TOUCH_DATA_LEN  (FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 3)

#define FTS_GESTURE_POINTS_MAX              6
#define FTS_GESTURE_DATA_LEN               (FTS_GESTURE_POINTS_MAX * 4 + 4)

#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_X_H_POS                   3
#define FTS_TOUCH_X_L_POS                   4
#define FTS_TOUCH_Y_H_POS                   5
#define FTS_TOUCH_Y_L_POS                   6
#define FTS_TOUCH_PRE_POS                   7
#define FTS_TOUCH_AREA_POS                  8
#define FTS_TOUCH_POINT_NUM                 2
#define FTS_TOUCH_EVENT_POS                 3
#define FTS_TOUCH_ID_POS                    5
#define FTS_COORDS_ARR_SIZE                 4

#define FTS_TOUCH_DOWN                      0
#define FTS_TOUCH_UP                        1
#define FTS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((FTS_TOUCH_DOWN == flag) || (FTS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (FTS_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 (!data->point_num)

#define FTS_MAX_COMPATIBLE_TYPE             4
#define FTS_MAX_COMMMAND_LENGTH             16

/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * For commnication error in PM(deep sleep) state
 */
#define FTS_PATCH_COMERR_PM                     0
#define FTS_TIMEOUT_COMERR_PM                   700

#define FTS_HIGH_REPORT                         0
#define FTS_SIZE_DEFAULT                        15

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FTS_MAX_COMMMAND_LENGTH];
};

struct fts_ts_platform_data {
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	u32 max_touch_number;
};

struct ts_event {
	int x;			/*x coordinate */
	int y;			/*y coordinate */
	int p;			/* pressure */
	int flag;		/* touch event flag: 0 -- down; 1-- up; 2 -- contact */
	int id;			/*touch ID */
	int area;
};

struct pen_event {
	int inrange;
	int tip;
	int x;			/*x coordinate */
	int y;			/*y coordinate */
	int p;			/* pressure */
	int flag;		/* touch event flag: 0 -- down; 1-- up; 2 -- contact */
	int id;			/*touch ID */
	int tilt_x;
	int tilt_y;
	int tool_type;
};

struct fts_fod_info {
    u8 fp_id;
    u8 event_type;
    u8 fp_area_rate;
    u8 tp_area;
    u16 fp_x;
    u16 fp_y;
    u8 fp_down;
    u8 fp_down_report;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct spi_device *spi;
	struct device *dev;

	struct fts_ts_platform_data *pdata;
	struct vts_device *vtsdev;
	struct ts_ic_info ic_info;
	struct workqueue_struct *ts_workqueue;
	struct work_struct fwupg_work;
	struct delayed_work esdcheck_work;
	struct delayed_work prc_work;
	struct work_struct resume_work;
	struct ftxxxx_proc proc;
	struct fts_fod_info fod_info;
	spinlock_t irq_lock;
	struct mutex report_mutex;
	struct mutex bus_lock;
	unsigned long intr_jiffies;
	int irq;
	int fw_is_running;	/* confirm fw is running when using spi:default 0 */
	int dummy_byte;
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	struct completion pm_completion;
	bool pm_suspend;
#endif
	bool suspended;
	bool fw_loading;
	bool irq_disabled;
	bool power_disabled;
	bool glove_mode;
	bool cover_mode;
	bool charger_mode;
	bool gesture_mode;	/* gesture enable or disable, default: disable */
	bool prc_mode;
	bool fp_en;
	struct pen_event pevent;
	/* multi-touch */
	struct ts_event *events;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	u8 *point_buf;
	int pnt_buf_size;
	int touchs;
	int key_state;
	int touch_point;
	int point_num;
	struct regulator *vdd;
	struct regulator *vcc_i2c;

#ifdef CONFIG_SPI_MT65XX
	struct mtk_chip_config spi_ctrl;
#endif

#if FTS_PINCTRL_EN
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	struct pinctrl_state *pins_release;
#endif
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	u8 reset_reason;
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *ft3518u_fts_data;

/* communication interface */
int ft3518u_fts_read(u8 * cmd, u32 cmdlen, u8 * data, u32 datalen);
int ft3518u_fts_read_reg(u8 addr, u8 * value);
int ft3518u_fts_write(u8 * writebuf, u32 writelen);
int ft3518u_fts_write_reg(u8 addr, u8 value);
int ft3518u_write_then_read(u8 addr, u8 value);

void ft3518u_fts_hid2std(void);
int ft3518u_fts_bus_init(struct fts_ts_data *ts_data);
int ft3518u_fts_bus_exit(struct fts_ts_data *ts_data);
int ft3518u_fts_spi_transfer_direct(u8 * writebuf, u32 writelen, u8 * readbuf,
				    u32 readlen);

/* Gesture functions */
int ft3518u_fts_gesture_init(struct fts_ts_data *ts_data);
int ft3518u_fts_gesture_exit(struct fts_ts_data *ts_data);
void ft3518u_fts_gesture_recovery(struct fts_ts_data *ts_data);
int ft3518u_fts_gesture_readdata(struct fts_ts_data *ts_data, u8 * data);
int ft3518u_fts_gesture_suspend(struct fts_ts_data *ts_data);
int ft3518u_fts_gesture_resume(struct fts_ts_data *ts_data);

/* Apk and functions */
int ft3518u_fts_create_apk_debug_channel(struct fts_ts_data *);
void ft3518u_fts_release_apk_debug_channel(struct fts_ts_data *);

/* ADB functions */
int ft3518u_fts_create_sysfs(struct fts_ts_data *ts_data);
int ft3518u_fts_remove_sysfs(struct fts_ts_data *ts_data);

/* ESD */
#if FTS_ESDCHECK_EN
int ft3518u_fts_esdcheck_init(struct fts_ts_data *ts_data);
int ft3518u_fts_esdcheck_exit(struct fts_ts_data *ts_data);
int ft3518u_fts_esdcheck_switch(bool enable);
int ft3518u_fts_esdcheck_proc_busy(bool proc_debug);
int ft3518u_fts_esdcheck_set_intr(bool intr);
int ft3518u_fts_esdcheck_suspend(void);
int ft3518u_fts_esdcheck_resume(void);
#endif

/* Point Report Check*/
#if FTS_POINT_REPORT_CHECK_EN
int ft3518u_fts_point_report_check_init(struct fts_ts_data *ts_data);
int ft3518u_fts_point_report_check_exit(struct fts_ts_data *ts_data);
void ft3518u_fts_prc_queue_work(struct fts_ts_data *ts_data);
#endif

/* FW upgrade */
int ft3518u_fts_fwupg_init(struct fts_ts_data *ts_data);
int ft3518u_fts_fwupg_exit(struct fts_ts_data *ts_data);
int ft3518u_fts_fw_download(const u8 *buf, u32 len, bool force);
//int ft3518u_fts_upgrade_bin(char *fw_name, bool force);
int ft3518u_fts_enter_test_environment(bool test_state);

/* Other */
int ft3518u_fts_reset_proc(int hdelayms);
int ft3518u_fts_check_cid(struct fts_ts_data *ts_data, u8 id_h);
int ft3518u_fts_wait_tp_to_valid(void);
void ft3518u_ft3518u_fts_release_all_finger(void);
void ft3518u_fts_tp_state_recovery(struct fts_ts_data *ts_data);
int ft3518u_fts_ex_mode_init(struct fts_ts_data *ts_data);
int ft3518u_fts_ex_mode_exit(struct fts_ts_data *ts_data);
int ft3518u_fts_ex_mode_recovery(struct fts_ts_data *ts_data);

void ft3518u_fts_irq_disable(void);
void ft3518u_fts_irq_enable(void);
#endif /* __LINUX_FOCALTECH_CORE_H__ */
