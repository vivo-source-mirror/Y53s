/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, Focaltech Ltd. All rights reserved.
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
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/vmalloc.h>
#include "focaltech_common.h"
#include "../vts_core.h"
#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT              10 /* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_WIDTH                       50
#define FTS_ONE_TCH_LEN                     6
#define FTS_SPI_BUFSIZ_MAX                  (4 * 1024)
#define FTS_SPI_CLK_MAX                     12000000

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

#define FTS_GESTURE_POINT_MAX               6
#define FTS_GESTRUE_POINTS_HEADER           4 /* Enable + Reserve + Header */
#define FTS_GESTURE_DATA_LEN                (FTS_GESTRUE_POINTS_HEADER + FTS_GESTURE_POINT_MAX * 4)
#define FTS_GESTURE_OFF                     63

#define FTS_TOUCH_DOWN                      0
#define FTS_TOUCH_UP                        1
#define FTS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((FTS_TOUCH_DOWN == flag) || (FTS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (FTS_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 ((data != NULL) ? (!data->point_num) : 1)
#define KEY_EN(data)                        ((data != NULL) ? (data->pdata.have_key) : 0)
#define TOUCH_IS_KEY(y, key_y)              (y == key_y)
#define TOUCH_IN_RANGE(val, key_val, half)  ((val > (key_val - half)) && (val < (key_val + half)))
#define TOUCH_IN_KEY(x, key_x)              TOUCH_IN_RANGE(x, key_x, FTS_KEY_WIDTH)

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct ftxxxx_proc {
    struct proc_dir_entry *proc;
    u8 opmode;
    u8 cmd_len;
    u8 cmd[SPI_MAX_COMMAND_LENGTH];
};

struct fts_ts_platform_data {
    int32_t irq_gpio;
    u32 irq_gpio_flags;
    int32_t reset_gpio;
    u32 reset_gpio_flags;
	int32_t cs_gpio;
	u32 cs_gpio_flags;
    bool have_key;
    u32 key_number;
    u32 keys[FTS_MAX_KEYS];
    u32 key_y_coord;
    u32 key_x_coords[FTS_MAX_KEYS];
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 max_touch_number;
	u32 spi_frequency;
};

struct ts_event {
    int x; /*x coordinate */
    int y; /*y coordinate */
    int p; /* pressure */
    int flag; /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;   /*touch ID */
    int area;
};

struct fts_ts_data {
    struct spi_device   *spi;
    struct fts_ts_platform_data pdata;
	struct vts_device *vtsdev;
    struct ts_ic_info ic_info;
    struct delayed_work esdcheck_work;
    struct ftxxxx_proc *proc;
    spinlock_t irq_lock;
    struct mutex report_mutex;
    struct mutex spilock;
	struct mutex mutex;
    int irq;
    bool suspended;
    bool fw_loading;
    bool irq_disabled;
    bool glove_mode;
    bool cover_mode;
    bool charger_mode;
    int fw_is_running;      /* confirm fw is running when using spi:default 0 */
    /* multi-touch */
    struct ts_event *events;
    u8 *point_buf;
    int pnt_buf_size;
    int touchs;
    bool key_down;
    int touch_point;
    int point_num;
	u8 *spi_buf;
	u8 *fw_buf;
	u8 *fw_spi;
	u8 *spi_cmd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *spi_active;
	struct pinctrl_state *spi_sleep_pulllow;
	u8 *dbg_write_buf;
	u8 *dbg_read_buf;
	char *dbg_temp_buf;
#ifdef CONFIG_SPI_MT65XX
	struct mtk_chip_config spi_ctrl;
#endif
	u8 custom_data[2];
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *fts_data_V2;

/* spi interface communication*/
int fts_read_V2(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int fts_read_reg_byte_V2(u8 regaddr, u8 *regvalue);
int fts_write_V2(u8 *writebuf, u32 writelen);
int fts_write_reg_byte_V2(u8 regaddr, u8 regvalue);
int fts_read_status(u8 *status);
int fts_write_then_read(u8 addr, u8 value);

/* Gesture functions */
#if FTS_GESTURE_EN
int fts_gesture_init_V2(struct fts_ts_data *ts_data);
int fts_gesture_exit_V2(struct fts_ts_data *ts_data);
void fts_gesture_recovery_V2(void);
int fts_gesture_buf_vivo_V2(u16 *data);
void fts_gesture_mode_set_V2(bool mode);
int fts_gesture_readdata_V2(struct fts_ts_data *ts_data, u8 *);
int fts_gesture_suspend_V2(struct fts_ts_data *ts_data);
int fts_gesture_resume_V2(struct fts_ts_data *ts_data);
#endif

/* Apk and functions */
#if FTS_APK_NODE_EN
int fts_create_apk_debug_channel_V2(struct fts_ts_data *);
void fts_release_apk_debug_channel_V2(struct fts_ts_data *);
#endif

/* ADB functions */
#if FTS_SYSFS_NODE_EN
int fts_create_sysfs_V2(struct device *dev);
int fts_remove_sysfs_V2(struct device *dev);
#endif

/* ESD */
#if FTS_ESDCHECK_EN
int fts_esdcheck_init(struct fts_ts_data *ts_data);
int fts_esdcheck_exit(struct fts_ts_data *ts_data);
int fts_esdcheck_switch(bool enable);
int fts_esdcheck_proc_busy(bool proc_debug);
int fts_esdcheck_set_intr(bool intr);
int fts_esdcheck_suspend(void);
int fts_esdcheck_resume(void);
#endif

int fts_fw_resume_V2(void);
int focal_fw_recovery(void);
int fts_upgrade_bin_V2(char *fw_name, bool force);
int fts_fw_download_V2(const u8 *buf, u32 len);
int fts_fw_enter_test_environment_V2(int test_state);

/* focal test */
/*int focaltech_get_rawordiff_data_V2(int which, int *data);*/

/* Other */
int fts_reset_proc_V2(int hdelayms);
int fts_wait_tp_to_valid_V2(void);
void fts_tp_state_recovery_V2(void);

void fts_irq_disable_V2(void);
void fts_irq_enable_V2(void);

/* test */
int focaltech_get_rawordiff_data_V2(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size);

#endif /* __LINUX_FOCALTECH_CORE_H__ */
