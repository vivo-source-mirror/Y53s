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
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#if FTS_GESTURE_EN
/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define KEY_GESTURE_U                           KEY_U
#define KEY_GESTURE_UP                          VTS_EVENT_GESTURE_PATTERN_UP
#define KEY_GESTURE_DOWN                        KEY_DOWN
#define KEY_GESTURE_LEFT                        VTS_EVENT_GESTURE_PATTERN_LEFT
#define KEY_GESTURE_RIGHT                       VTS_EVENT_GESTURE_PATTERN_RIGHT
#define KEY_GESTURE_O                           VTS_EVENT_GESTURE_PATTERN_O
#define KEY_GESTURE_E                           VTS_EVENT_GESTURE_PATTERN_E
#define KEY_GESTURE_M                           VTS_EVENT_GESTURE_PATTERN_M
#define KEY_GESTURE_L                           KEY_L
#define KEY_GESTURE_W                           VTS_EVENT_GESTURE_PATTERN_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_V
#define KEY_GESTURE_C                           VTS_EVENT_GESTURE_PATTERN_C
#define KEY_GESTURE_Z                           KEY_Z
#define KEY_GESTURE_A                           VTS_EVENT_GESTURE_PATTERN_A
#define KEY_GESTURE_F                           VTS_EVENT_GESTURE_PATTERN_F

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34
#define GESTURE_O_CW                            0x57
#define GESTURE_A                               0x50
#define GESTURE_F                               0x74

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* header        -   byte0:gesture id
*                   byte1:pointnum
*                   byte2~7:reserved
* coordinate_x  -   All gesture point x coordinate
* coordinate_y  -   All gesture point y coordinate
* mode          -   1:enable gesture function(default)
*               -   0:disable
* active        -   1:enter into gesture(suspend)
*                   0:gesture disable or resume
*/
struct fts_gesture_st {
    u8 header[FTS_GESTRUE_POINTS_HEADER];
    u16 coordinate_x[FTS_GESTURE_POINT_MAX];
    u16 coordinate_y[FTS_GESTURE_POINT_MAX];
    u8 mode;   /*host driver enable gesture flag*/
    u8 active;  /*gesture actutally work*/
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode        ---read gesture mode
 *   write example:echo 0/1 > fts_gesture_mode   ---disable/enable gesture mode
 *
 */
static DEVICE_ATTR (fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show, fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        ---read gesture buf
 */
static DEVICE_ATTR (fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);
static struct attribute *fts_gesture_mode_attrs[] = {
    &dev_attr_fts_gesture_mode.attr,
    &dev_attr_fts_gesture_buf.attr,
    NULL,
};

static struct attribute_group fts_gesture_group = {
    .attrs = fts_gesture_mode_attrs,
};

static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&fts8756_fts_data->mutex);
    fts8756_fts8756_fts_read_reg_byte(FTS_REG_GESTURE_EN, &val);
    count = snprintf(buf, PAGE_SIZE, "Gesture Mode: %s\n", fts_gesture_data.mode ? "On" : "Off");
    count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0) = %d\n", val);
    mutex_unlock(&fts8756_fts_data->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    mutex_lock(&fts8756_fts_data->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        VTI("enable gesture");
        fts_gesture_data.mode = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        VTI("disable gesture");
        fts_gesture_data.mode = DISABLE;
    }
    mutex_unlock(&fts8756_fts_data->mutex);

    return count;
}

static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&fts8756_fts_data->mutex);
    count = snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n", fts_gesture_data.header[0]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n", fts_gesture_data.header[1]);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
    for (i = 0; i < fts_gesture_data.header[1]; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i, fts_gesture_data.coordinate_x[i], fts_gesture_data.coordinate_y[i]);
        if ((i + 1) % 4 == 0)
            count += snprintf(buf + count, PAGE_SIZE, "\n");
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&fts8756_fts_data->mutex);
    FTS_FUNC_EXIT();
    return count;
}

static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

int fts8756_fts_create_gesture_sysfs(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_gesture_group);
    if (ret) {
        VTE("gesture sys node create fail");
        sysfs_remove_group(&dev->kobj, &fts_gesture_group);
        return ret;
    }

    return 0;
}

static void fts_gesture_report(int gesture_id)
{
    int gesture;

    FTS_FUNC_ENTER();
    VTI("fts gesture_id==0x%x ", gesture_id);
    switch (gesture_id) {
    case GESTURE_LEFT:
        gesture = KEY_GESTURE_LEFT;
        break;
    case GESTURE_RIGHT:
        gesture = KEY_GESTURE_RIGHT;
        break;
    case GESTURE_UP:
        gesture = KEY_GESTURE_UP;
        break;
    case GESTURE_DOWN:
        gesture = VTS_EVENT_GESTURE_PATTERN_DOWN;
        break;
    case GESTURE_DOUBLECLICK:
        gesture = VTS_EVENT_GESTURE_DOUBLE_CLICK;
        break;
    case GESTURE_O:
	case GESTURE_O_CW:
        gesture = KEY_GESTURE_O;
        break;
    case GESTURE_W:
        gesture = KEY_GESTURE_W;
        break;
    case GESTURE_M:
        gesture = KEY_GESTURE_M;
        break;
    case GESTURE_E:
        gesture = KEY_GESTURE_E;
        break;
    case GESTURE_L:
        gesture = KEY_GESTURE_L;
        break;
    case GESTURE_S:
        gesture = KEY_GESTURE_S;
        break;
    case GESTURE_V:
        gesture = KEY_GESTURE_V;
        break;
    case GESTURE_Z:
        gesture = KEY_GESTURE_Z;
        break;
    case  GESTURE_C:
        gesture = KEY_GESTURE_C;
        break;
	case GESTURE_A:
        gesture = KEY_GESTURE_A;
        break;
	case GESTURE_F:
        gesture = KEY_GESTURE_F;
        break;
    default:
        gesture = -1;
        break;
    }
    /* report event key */
    if (gesture != -1) { 
		vts_report_event_down(fts8756_fts_data->vtsdev, gesture);
		vts_report_event_up(fts8756_fts_data->vtsdev, gesture);
    }

    FTS_FUNC_EXIT();
}

int fts8756_fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
    int i = 0;
    int gestrue_id = 0;
    u8 pointnum = 0;

    if (!ts_data->suspended) {
        return -EINVAL;
    }

    /* gesture invalid */
    if (data[0] != 0x01) {
        return 1;
    }

    /* init variable before read gesture point */
    memset(fts_gesture_data.coordinate_x, 0, FTS_GESTURE_POINT_MAX * sizeof(u16));
    memset(fts_gesture_data.coordinate_y, 0, FTS_GESTURE_POINT_MAX * sizeof(u16));

    gestrue_id = data[2];
    pointnum = data[3];
    VTD("[GESTURE]PointNum=%d", pointnum);
    if (pointnum > FTS_GESTURE_POINT_MAX) {
        VTE("gesture pointnum(%d) fail", pointnum);
        return -EIO;
    }

	for (i = 0; i < FTS_GESTURE_POINT_MAX; i++)
		fts_gesture_data.coordinate_x[i] = fts_gesture_data.coordinate_y[i] = 65535;

    fts_gesture_report(gestrue_id);
	fts_gesture_data.header[1] = pointnum;
    for (i = 0; i < pointnum; i++) {
        fts_gesture_data.coordinate_x[i] = (((s16) data[0 + (4 * i + 4)]) & 0xFF) << 8
                                           | (((s16) data[1 + (4 * i + 4)]) & 0xFF);
        fts_gesture_data.coordinate_y[i] = (((s16) data[2 + (4 * i + 4)]) & 0xFF) << 8
                                           | (((s16) data[3 + (4 * i + 4)]) & 0xFF);
    }

	pointnum = FTS_GESTURE_POINT_MAX;
	if (GESTURE_O_CW == gestrue_id) {
		fts_gesture_data.coordinate_y[FTS_GESTURE_POINT_MAX-1] = 16;
	} else if (GESTURE_O == gestrue_id) {
		fts_gesture_data.coordinate_y[FTS_GESTURE_POINT_MAX-1] = 32;
	}
	vts_report_coordinates_set(ts_data->vtsdev, fts_gesture_data.coordinate_x, fts_gesture_data.coordinate_y, pointnum);

    return 0;
}

void fts8756_fts_gesture_recovery(void)
{
    if ((ENABLE == fts_gesture_data.mode) && (ENABLE == fts_gesture_data.active)) {
        VTD("enter fts8756_fts_gesture_recovery");
        fts8756_fts8756_fts_write_reg_byte(0xD1, 0xff);
        fts8756_fts8756_fts_write_reg_byte(0xD2, 0xff);
        fts8756_fts8756_fts_write_reg_byte(0xD5, 0xff);
        fts8756_fts8756_fts_write_reg_byte(0xD6, 0xff);
        fts8756_fts8756_fts_write_reg_byte(0xD7, 0xff);
        fts8756_fts8756_fts_write_reg_byte(0xD8, 0xff);
        fts8756_fts8756_fts_write_reg_byte(FTS_REG_GESTURE_EN, ENABLE);
    }
}

int fts8756_fts_gesture_buf_vivo(u16 *data)
{
	int i = 0;

	for(i = 0; i < fts_gesture_data.header[1]; i++) {
		data[2*i] = fts_gesture_data.coordinate_x[i];
		data[2*i+1] = fts_gesture_data.coordinate_y[i];
	}

	return fts_gesture_data.header[1];
}

void fts8756_fts_gesture_mode_set(bool mode)
{
	fts_gesture_data.mode = mode;
}


int fts8756_fts_gesture_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        VTD("gesture is disabled");
        return -EINVAL;
    }

    for (i = 0; i < 5; i++) {
		/*
        fts8756_fts8756_fts_write_reg_byte(0xd1, 0xFF);
        fts8756_fts8756_fts_write_reg_byte(0xd2, 0xFF);
        fts8756_fts8756_fts_write_reg_byte(0xd5, 0xFF);
        fts8756_fts8756_fts_write_reg_byte(0xd6, 0xFF);
        fts8756_fts8756_fts_write_reg_byte(0xd7, 0xFF);
        fts8756_fts8756_fts_write_reg_byte(0xd8, 0xFF);
        */
        fts8756_fts8756_fts_write_reg_byte(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts8756_fts8756_fts_read_reg_byte(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }

    if (i >= 5) {
        VTE("Enter into gesture(suspend) fail");
        fts_gesture_data.active = DISABLE;
        return -EIO;
    }

    ret = enable_irq_wake(ts_data->irq);
    if (ret) {
        VTD("enable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    fts_gesture_data.active = ENABLE;
    VTI("Enter into gesture(suspend) success");
    FTS_FUNC_EXIT();
    return 0;
}

int fts8756_fts_gesture_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 state = 0xFF;

    FTS_FUNC_ENTER();
    /* gesture not enable, return immediately */
    if (fts_gesture_data.mode == DISABLE) {
        VTD("gesture is disabled");
        return -EINVAL;
    }

    if (fts_gesture_data.active == DISABLE) {
        VTD("gesture in suspend is failed, no running fts8756_fts_gesture_resume");
        return -EINVAL;
    }

    fts_gesture_data.active = DISABLE;
    for (i = 0; i < 5; i++) {
        fts8756_fts8756_fts_write_reg_byte(FTS_REG_GESTURE_EN, DISABLE);
        msleep(1);
        fts8756_fts8756_fts_read_reg_byte(FTS_REG_GESTURE_EN, &state);
        if (state == DISABLE)
            break;
    }

    if (i >= 5) {
        VTE("exit gesture(resume) fail");
        return -EIO;
    }

    ret = disable_irq_wake(ts_data->irq);
    if (ret) {
        VTD("disable_irq_wake(irq:%d) fail", ts_data->irq);
    }

    VTD("exit gesture(resume) success");
    return 0;
}

int fts8756_fts_gesture_init(struct fts_ts_data *ts_data)
{ 
    fts8756_fts_create_gesture_sysfs(&ts_data->spi->dev);
    fts_gesture_data.mode = ENABLE; 
    return 0;
}

int fts8756_fts_gesture_exit(struct fts_ts_data *ts_data)
{ 
    sysfs_remove_group(&ts_data->spi->dev.kobj, &fts_gesture_group); 
    return 0;
}
#endif
