/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2018, FocalTech Systems, Ltd., all rights reserved.
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
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-11-06
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   100  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */

const unsigned char VIVO_CUST_FW_FT8719_SPI_JDI[] =
{
#include FTS_UPGRADE_FW_FILE
};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
struct vivo_ts_struct *g_vts_data_0flash = NULL;

extern unsigned int is_atboot;
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);
#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
#else
static int mdss_dsi_panel_reset_and_powerctl(int enable)
{
	return 0;
}
#endif


/*****************************************************************************
* static variable
*****************************************************************************/
static int last_ts_state = TOUCHSCREEN_NORMAL;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);


/*
 * description: wait tp to run fw normal(Timeout: TIMEOUT_READ_REG),
 *              need call when reset/power on/resume...
 *
 * param -
 *
 * return 0 if tp valid, otherwise return error code
 */
int fts_wait_tp_to_valid_0flash(void)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;
    u8 chip_id = fts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg_byte(FTS_REG_CHIP_ID, &reg_value);
        if ((ret < 0) || (reg_value != chip_id)) {
            VTD("TP Not Ready, ReadData = 0x%x", reg_value);
        } else if (reg_value == chip_id) {
            VTI("TP Ready, Device ID = 0x%x", reg_value);
            return 0;
        }
        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    int ctype_num = sizeof(ctype) / sizeof(struct ft_chip_t);

    for (i = 0; i < ctype_num; i++) {
        VTI("%d %d", IC_SERIALS, (int)ctype[i].type);
        if (IC_SERIALS == ctype[i].type)
            break;
    }

    if (i >= ctype_num) {
        VTE("get ic type fail, pls check FTS_CHIP_TYPE_MAPPING");
        return -ENODATA;
    }

    ts_data->ic_info.ids = ctype[i];
    VTI("CHIP TYPE ID in driver = 0x%02x%02x",
             ts_data->ic_info.ids.chip_idh,
             ts_data->ic_info.ids.chip_idl);

    return 0;
}

/*
 * description: recovery tp state: gesture/cover/glove...
 */
void fts_tp_state_recovery_0flash(void)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid_0flash();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */

#if FTS_GESTURE_EN
    fts_gesture_recovery_0flash();
#endif
    FTS_FUNC_EXIT();
}

/*
 * description: Execute hw reset operation
 */
int fts_reset_proc_0flash(int hdelayms)
{
	int ret = 0;
	
    FTS_FUNC_ENTER();
	if (gpio_is_valid(fts_data->pdata->reset_gpio)) {
    	gpio_direction_output(fts_data->pdata->reset_gpio, 0);
		mdelay(5);
    	gpio_direction_output(fts_data->pdata->reset_gpio, 1);
	    if (hdelayms) {
			mdelay(hdelayms);
	    }
		ret = 1;
	} else {
		ret = -1;
	}

    FTS_FUNC_EXIT();
    return ret;
}

void fts_irq_disable_0flash(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable_0flash(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = {0};
static void fts_show_touch_buffer(u8 *buf, int point_num)
{
    int len = point_num * FTS_ONE_TCH_LEN;
    int count = 0;
    int i;

    memset(g_sz_debug, 0, 1024);
    if (len > (fts_data->pnt_buf_size - 3)) {
        len = fts_data->pnt_buf_size - 3;
    } else if (len == 0) {
        len += FTS_ONE_TCH_LEN;
    }
    count += snprintf(g_sz_debug, PAGE_SIZE, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
    for (i = 0; i < len; i++) {
        count += snprintf(g_sz_debug + count, PAGE_SIZE, ",%02X", buf[i + 3]);
    }
    VTD("buffer: %s", g_sz_debug);
}
#endif

void fts_release_all_finger(void)
{
    struct input_dev *input_dev = fts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
#endif

    FTS_FUNC_ENTER();
    mutex_lock(&fts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < fts_data->pdata->max_touch_number; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

    mutex_unlock(&fts_data->report_mutex);
    FTS_FUNC_EXIT();
}

static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    u32 ik;
    int id = data->events[index].id;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int flag = data->events[index].flag;
    u32 key_num = data->pdata->key_number;

    if (!KEY_EN(data)) {
        return -EINVAL;
    }
    for (ik = 0; ik < key_num; ik++) {
        if (TOUCH_IN_KEY(x, data->pdata->key_x_coords[ik])) {
            if (EVENT_DOWN(flag)) {
                data->key_down = true;
                input_report_key(data->input_dev, data->pdata->keys[ik], 1);
                VTD("Key%d(%d, %d) DOWN!", ik, x, y);
            } else {
                data->key_down = false;
                input_report_key(data->input_dev, data->pdata->keys[ik], 0);
                VTD("Key%d(%d, %d) Up!", ik, x, y);
            }
            return 0;
        }
    }

    VTE("invalid touch for key, [%d](%d, %d)", id, x, y);
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    u32 key_y_coor = data->pdata->key_y_coord;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
            fts_input_report_key(data, i);
            continue;
        }

        if (events[i].id >= max_touch_num)
            break;

        va_reported = true;

        if (EVENT_DOWN(events[i].flag)) {
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);

			vivoTsInputReport(VTS_TOUCH_DOWN, events[i].id, events[i].x, events[i].y, events[i].area);

#if FTS_REPORT_PRESSURE_EN
            VTD("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", events[i].id, events[i].x, events[i].y, events[i].p, events[i].area);
#else
            VTD("[B]P%d(%d, %d)[tm:%d] DOWN!", events[i].id, events[i].x, events[i].y, events[i].area);
#endif
        } else {
            uppoint++;
			data->touchs &= ~BIT(events[i].id);
            vivoTsInputReport(VTS_TOUCH_UP, events[i].id, events[i].x, events[i].y, events[i].area);
        }
    }
    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                VTI("[B1]P%d UP!", i);
                va_reported = true;
				vivoTsInputReport(VTS_TOUCH_UP, i, events[i].x, events[i].y, events[i].area);
            }
        }
    }
    data->touchs = touchs;
    if (va_reported) {
        /* touchs==0, there's no point but key */
		if (EVENT_NO_DOWN(data) || (!touchs)) {
        	vivoTsReleasePoints();
		}
    }

    return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 key_y_coor = data->pdata->key_y_coord;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (KEY_EN(data) && TOUCH_IS_KEY(events[i].y, key_y_coor)) {
            fts_input_report_key(data, i);
            continue;
        }

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            VTD("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!", events[i].id, events[i].x,
                      events[i].y, events[i].p, events[i].area);
            touchs++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
            VTD("[A]Points All Up!");
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif
static void fts_show_touch_buffer_c(u8 *data, int datalen)
{
    int i = 0;
    int count = 0;
    char *tmpbuf = NULL;
    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        FTS_ERROR("c_tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    FTS_INFO("c_buffer:%s", tmpbuf);
    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

static int fts_read_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;	
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;
    u8 flag = 0;
    u8 tmp_flag = 0;
	static int large_flag;

#if FTS_POINT_REPORT_CHECK_EN
    fts_prc_queue_work(data);
#endif

    data->point_num = 0;
    data->touch_point = 0;

    memset(buf, 0xFF, data->pnt_buf_size);
    buf[0] = 0x01;
    ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
	if (ret < 0) {
        /*read agin after error,and show log*/
    	memset(buf, 0xFF, data->pnt_buf_size);
		buf[0] = 0x01;
		ret = fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
		fts_show_touch_buffer_c(buf, data->pnt_buf_size);
    }
    flag = buf[1] & 0xF0;
    if ((ret < 0) && (flag != 0x90)) {
        /* check if need recovery fw */
        fts_fw_recovery();
        return ret;
    } else if ((ret < 0) || (flag != 0x90)) {
        FTS_ERROR("touch data(%x) fail,ret:%d", buf[1], ret);
        return -EIO;
    }

#if FTS_GESTURE_EN
    ret = fts_gesture_readdata_0flash(data, buf + FTS_GESTURE_OFF);
    if (0 == ret) {
        VTI("succuss to get gesture data in irq handler");
        return 1;
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    if ((data->point_num == 0x0F) && (buf[2] == 0xFF)
        && (buf[3] == 0xFF) && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
        VTD("touch buff is 0xff, need recovery state");
		vivoTsReleasePoints();
        fts_tp_state_recovery_0flash();
        return -EIO;
    }

    if (data->point_num > max_touch_num) {
        VTE("invalid point_num(%d)", data->point_num);
        return -EIO;
    }

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
    fts_show_touch_buffer(buf, data->point_num);
#endif

    for (i = 0; i < max_touch_num; i++) {
        base = FTS_ONE_TCH_LEN * i;

        pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else if (pointid >= max_touch_num) {
            VTE("ID(%d) beyond max_touch_number", pointid);
            return -EINVAL;
        }

        data->touch_point++;

        events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
        events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
        events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
        events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
        events[i].p =  buf[FTS_TOUCH_PRE_POS + base];

        if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
            VTE("abnormal touch data from fw");
            return -EIO;
        }
    }
	
#if 1	
		tmp_flag = ((buf[1] & 0xFF) == 0x91);
		if (data->point_num == 0 && tmp_flag && large_flag == 0) {
			g_vts_data_0flash->largePressNum++;
			//VTI("got PalmOn");
			large_flag = 1;
		} else if (data->point_num == 0 && (!tmp_flag)){
			large_flag = 0;			
			//VTI("got PalmOFF");	
		}
	
		/*when event->point_num == 0, also call it, because we need set AA release time */
	/*	if (event->point_num || 	buf[1]) { */
			ret = vivoTsCoverMute(data->point_num, tmp_flag);	/*this buf[1] is always 0, because this firmware not support */
			if (ret < 0) {
				if (-2 == ret) {
					VTI("enter larger or 3 finger mode");
					return 0;
				}
	
				if (-3 == ret) {
					VTI("lcd shutoff,not report points");
					return ret;
				}
	
			}
	
#endif

    if (data->touch_point == 0) {
        VTD("no touch point information");
        return -EIO;
    }

    return 0;
}

static void fts_report_event(struct fts_ts_data *data)
{
#if FTS_MT_PROTOCOL_B_EN
    fts_input_report_b(data);
#else
    fts_input_report_a(data);
#endif
}

static void fts_ts_work_func(void *dev_id)
{
	int ret = -1;

	VTD("======enter=");

	if (atomic_read(&vivoTsGetVtsData()->tsState) == TOUCHSCREEN_GESTURE) {	//avoid spi sleep cause gesture and firmware update no function
		int retry = 0;
		VTD("wait spi bus wakeup");
		while(fts_data->spi_suspended) {
			msleep(10);
			if (++retry > 50) {
				VTI("after 500ms delay, device is stil in suspend mode!\n");
				return;
			}
		}
	}
	
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr_pd1831(1);
#endif

	ret = fts_read_touchdata(fts_data);

	if (ret == 0) {
		mutex_lock(&fts_data->report_mutex);
		fts_report_event(fts_data);
		mutex_unlock(&fts_data->report_mutex);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr_pd1831(0);
#endif

	VTD("=======exit=");
	return;

}


static irqreturn_t fts_ts_interrupt(int irq, void *data)
{
    //int ret = 0;
    struct fts_ts_data *ts_data = (struct fts_ts_data *)data;

    if (!ts_data) {
        VTE("[INTR]: Invalid fts_ts_data");
        return IRQ_HANDLED;
    }
#if 0

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr_pd1831(1);
#endif

    ret = fts_read_touchdata(ts_data);
    if (ret == 0) {
        mutex_lock(&ts_data->report_mutex);
        fts_report_event(ts_data);
        mutex_unlock(&ts_data->report_mutex);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr_pd1831(0);
#endif

#endif

	vtsIrqBtmThreadWake();

    return IRQ_HANDLED;
}

/*
 * description: register irq
 */
static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    VTI("irq in ts_data:%d irq in client:%d", ts_data->irq, ts_data->spi->irq);
    if (ts_data->irq != ts_data->spi->irq)
        VTE("IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");

    if (0 == pdata->irq_gpio_flags)
        pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;
    VTI("irq flag:%x", pdata->irq_gpio_flags);
	/*
    ret = request_threaded_irq(ts_data->irq, NULL, fts_ts_interrupt,
                               pdata->irq_gpio_flags | IRQF_ONESHOT,
                               FTS_DRIVER_NAME, ts_data);
	*/
	ret = vivoTsInterruptRegister(ts_data->irq, NULL, fts_ts_interrupt, pdata->irq_gpio_flags | IRQF_TRIGGER_FALLING, ts_data);
	if (ret) {
			VTE("Request irq failed!");
	} else {
			VTI("request irq %d succeed", ts_data->irq);
	}
    return ret;
}

static int fts_input_alloc(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int point_num;

	FTS_FUNC_ENTER();
	
	point_num = FTS_MAX_POINTS_SUPPORT;
    ts_data->pnt_buf_size = point_num * FTS_ONE_TCH_LEN + 3;
#if FTS_GESTURE_EN
    ts_data->pnt_buf_size += FTS_GESTURE_DATA_LEN;
#endif
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        VTE("failed to alloc memory for point buf!");
        ret = -ENOMEM;
        goto err_point_buf;
    }

    ts_data->events = (struct ts_event *)kzalloc(point_num * sizeof(struct ts_event), GFP_KERNEL);
    if (!ts_data->events) {

        VTE("failed to alloc memory for point events!");
        ret = -ENOMEM;
        goto err_event_buf;
    }

	FTS_FUNC_EXIT();
    return 0;

err_event_buf:
    kfree_safe(ts_data->point_buf);

err_point_buf:
	FTS_FUNC_EXIT();
	return ret;

}

/*
 * description: configure reset & irq gpio
 */
static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            VTE("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            VTE("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
        if (ret) {
            VTE("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 1);
        if (ret) {
            VTE("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        VTE("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret && (ret != -EINVAL)) {
        VTE("Unable to read %s", name);
        return -ENODATA;
    }

    if (!strcmp(name, "focaltech,display-coords")) {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    } else {
        VTE("unsupported property %s", name);
        return -EINVAL;
    }

    VTI("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

/*
 * description: parse ts data from dts
 */
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;

    FTS_FUNC_ENTER();

	pdata->pinctrl_gpios = devm_pinctrl_get(dev);
	if (IS_ERR(pdata->pinctrl_gpios)) {
		ret = PTR_ERR(pdata->pinctrl_gpios);
		VTE("can not find touch pintrl1");
		return ret;
	}
	pdata->spi_cs_set = pinctrl_lookup_state(pdata->pinctrl_gpios, "spi_cs_set");
	if (IS_ERR(pdata->spi_cs_set))	{
		ret = PTR_ERR(pdata->spi_cs_set);
		VTE("spi_cs_set pinctrl");		
	}
	pinctrl_select_state(pdata->pinctrl_gpios, pdata->spi_cs_set);
	
	pdata->pin_cs_pulllow = pinctrl_lookup_state(pdata->pinctrl_gpios,  "spi_cs_pulllow");
	if (IS_ERR(pdata->pin_cs_pulllow))	{
		ret = PTR_ERR(pdata->pin_cs_pulllow);
		VTE("pin_cs_pulllow pinctrl");		
	}

    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        VTE("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret)
            VTE("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret)
            VTE("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32(np, "focaltech,key-y-coord", &pdata->key_y_coord);
        if (ret)
            VTE("Key Y Coord undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords, pdata->key_number);
        if (ret)
            VTE("Key X Coords undefined!");

        VTI("VK(%d): (%d, %d, %d), [%d, %d, %d][%d]",
                 pdata->key_number, pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_x_coords[1], pdata->key_x_coords[2],
                 pdata->key_y_coord);
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio", 0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        VTE("Unable to get reset_gpio");

    pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio", 0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        VTE("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (0 == ret) {
        if (temp_val < 2)
            pdata->max_touch_number = 2;
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    } else {
        VTE("Unable to get max-touch-number");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    }
	ret = of_property_read_u32(np, "focaltech,spi-max-frequency", &temp_val);
	if (0 == ret && temp_val > 0 && temp_val <= 9600000) {
	  pdata->spi_frequency = temp_val;
	} else {
	  /*default frequency*/
	  pdata->spi_frequency = 6000000;
	}

    VTI("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

    FTS_FUNC_EXIT();
    return 0;
}
#if 0
#if defined(CONFIG_FB)
/* FB notifier callback from LCD driver */
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct fts_ts_data *ts_data =
        container_of(self, struct fts_ts_data, fb_notif);

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        VTI("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    VTI("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            VTI("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            fts_ts_resume(&ts_data->spi->dev);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            fts_ts_suspend(&ts_data->spi->dev);
        } else if (FB_EVENT_BLANK == event) {
            VTI("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        VTI("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* early_suspend/resume */
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *data = container_of(handler,
                                            struct fts_ts_data,
                                            early_suspend);

    fts_ts_suspend(&data->spi->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *data = container_of(handler,
                                            struct fts_ts_data,
                                            early_suspend);

    fts_ts_resume(&data->spi->dev);
}
#endif
#endif

extern unsigned int mdss_report_lcm_id(void);
static unsigned int report_lcm_id(void)
{
	int lcm_id = 0;
	lcm_id = mdss_report_lcm_id();
	VTI("lcm_id get from mdss is %d", lcm_id);
	
	return lcm_id;
}
static int bbk_xxx_get_lcm_id(void)
{
	return report_lcm_id();
}

static int focaltech_mode_change(int which)
{
	int ret = 0;
	int i = 0;	

	VTI("fts: +++0:normal,1:sleep,2:gesture+++, which = %d", which);

	if (which == TOUCHSCREEN_NORMAL) {
		VTI("change to *****normal mode*****");

		//mdelay(240);
		ret = fts_ts_resume(&fts_data->client->dev);
#if FTS_GESTURE_EN
		fts_gesture_mode_set(false);
#endif

		atomic_set(&g_vts_data_0flash->tsCallBackRTState, CALLBACK_FINISH);

		//fts_irq_enable_0flash();

#if FTS_ESDCHECK_EN
		fts_esdcheck_resume_pd1831();
#endif

	}

	if (which == TOUCHSCREEN_GESTURE) {
		VTI("change to *****gesture mode*****");
		if (TOUCHSCREEN_SLEEP == last_ts_state) {
			mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			mdss_dsi_panel_reset_and_powerctl(1);
			mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));

			VTI("change to gesture mode========= mdelay 290");
			mdelay(290);

			fts_wait_tp_to_valid_0flash();

#if 1
			for( i = 0 ; i < 2;i++) {
				if(atomic_read(&g_vts_data_0flash->tsCallBackRTState)!=CALLBACK_RESUME){
					ret = fts_write_reg_byte(0xfd, 0x5a);
					if(ret < 0){
						VTI("fail to wirte 0xfd register " );
						msleep(10);
						continue;
					}else{
						VTI("wirte 0xfd success GGGGGGG" );
					}					

				} else {
					VTI("--------panel on, no need write FD--------");
					return 0;
				}
			}
#endif
		}

#if FTS_GESTURE_EN
		fts_gesture_mode_set(true);
#endif
		fts_data->suspended = 0;
		fts_data->touchs = 0;
		ret = fts_ts_suspend(&fts_data->client->dev);
		//fts_irq_enable_0flash();

#if FTS_ESDCHECK_EN
		fts_esdcheck_suspend_pd1831();
#endif

	}
	if (which == TOUCHSCREEN_SLEEP) {
		VTI("change to *****sleep mode*****");

#if FTS_ESDCHECK_EN
		fts_esdcheck_suspend_pd1831();
#endif

#if FTS_GESTURE_EN
		fts_gesture_mode_set(false);
#endif

		fts_data->suspended = true;
		fts_data->touchs = 0;
		//fts_irq_disable_0flash();

		if (last_ts_state == TOUCHSCREEN_NORMAL) {
			/* TP enter sleep mode */
			ret = fts_write_reg_byte(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
			if (ret < 0) {
				VTI("Set TP to sleep mode fail, ret=%d!", ret);
			}
			msleep(80);   /*A5 write 03 need delay 80ms before AVDD&AVEE lost*/
		} else if (last_ts_state == TOUCHSCREEN_GESTURE) {
			mdss_dsi_panel_reset_and_powerctl(4);
			mdelay(5);
			mdss_dsi_panel_reset_and_powerctl(3);
			
			msleep(100);
			
			fts_wait_tp_to_valid_0flash();
			ret = fts_write_reg_byte(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
			if (ret < 0) {
				VTI("Set TP to sleep mode fail, ret=%d!", ret);
			}
			msleep(250);			
		}

		/*to protect "close to prox" - "resume", if we lock, mdss(2) -> mdss (0);  mdss(0) will prtect by mdss */
		/* early event -> lcm power on (mdss (2)) -> proxi_switch (0) */
		mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		mdss_dsi_panel_reset_and_powerctl(0);
		mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
	}

	last_ts_state = which;

	VTI("fts: ----mode_change sucessfully----");

	return 0;
}

static int bbk_focaltech_fw_version(int which)
{
	unsigned char ver = 0;
	int ret = 0;

	if (CONFIG_VERSION == which) {
		/*show PR id */

	}
	if (FW_VERSION == which) {
		ret = fts_read_reg_byte(0xA6, &ver);
		if (ver < 0) {
			VTI("read fw version fail.");
		}
	}

	return ver;
}

static int focaltech_module_id(void)
{
	/*just for display in *#225# view */
	if (vivoTsGetLcmId() == 0x00) {
		return VTS_MVC_JDI;	/* JDI */
	}

	return VTS_MVC_JDI; /* default JDi */

}

#if FTS_GESTURE_EN
static int focaltech_gesture_point(u16 *data)
{
	return fts_gesture_buf_vivo_0flash(data);
}
#endif

static int focaltech_charger_state_write(int state)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 charge_state = (u8)(state?1:0);

	VTI("write charger state: %d ", charge_state);	

	for (i = 0; i < 10; i++) {
		ret = fts_write_reg_byte(0x8b, charge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8b register %d", charge_state);

		mdelay(5);

		ret = fts_read_reg_byte(0x8b, &temp_data);
		if (ret < 0)
			VTI("fail to read 0x8b register %d", temp_data);

		if (temp_data != charge_state) {
			VTI("Fail to write 0x8b.data = %d", charge_state);
			msleep(10);
			continue;
		} else {
			VTI("FTS : Success to write 0x8b.data = %d", charge_state);
			break;
		}
	}
	return 0;
}

static int focaltech_idle_state_write(int state)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 idle_state = (u8)(state?1:0);

	VTI("write idle state: %d ", idle_state);

	for (i = 0; i < 10; i++) {
		ret = fts_write_reg_byte(0x86, idle_state);
		if (ret < 0)
			VTI("fail to wirte 0x86 register %d", idle_state);

		mdelay(5);

		ret = fts_read_reg_byte(0x86, &temp_data);
		if (ret < 0)
			VTI("fail to read 0x86 register %d", temp_data);

		if (temp_data != idle_state) {
			VTI("Fail to write 0x86.data = %d", idle_state);
			msleep(10);
			continue;
		} else {
			VTI("FTS : Success to write 0x86.data = %d", idle_state);
			break;
		}
	}

	if (i == 10) {
		//return MINUS_ONE;
		return -EPERM;
	} else {
		return 0;
	}
}

static int focaltech_edge_state_write(int state)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 edge_state = 0;
	
	switch (state) {
	case 0:
		edge_state = 1;
	break;
	case 1:
		edge_state = 0;
	break;
	case 2:
		edge_state = 2;
	break;
	default:
		VTI("Unknown argument");
		edge_state = 0;
	}

	VTI("write edge state: %d", edge_state);
	for (i = 0; i < 10; i++) {
		ret = fts_write_reg_byte(0x8c, edge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8a register %d", edge_state);

		mdelay(5);

		ret = fts_read_reg_byte(0x8c, &temp_data);
		if (ret < 0)
			VTI("fail to read 0x8a register %d", temp_data);

		if (temp_data != edge_state) {
			VTI("Fail to write 0x8a.data = %d", edge_state);
			msleep(10);
			continue;
		} else {
			VTI("FTS : Success to write 0x8a.data = %d", edge_state);
			break;
		}
	}
	return 0;
}

static int print_info(unsigned char *buf)
{

	int ret;
	u8 initcode_ver = 0;
	u8 auc_i2c_write_buf[10] = {0};
	u8 reg_val[4] = {0};

	ret = fts_read_reg_byte(0xE4, &initcode_ver);
	if (ret < 0)
		VTI("fail to wirte 0x8a register %d", initcode_ver);
	else
		VTI("read init code version = %d", initcode_ver);

	auc_i2c_write_buf[0] = 0x7D;
	ret = fts_read(auc_i2c_write_buf, 1, reg_val, 3);
	if (ret < 0)
		VTI("fail to read 0x02 register ");
	else
		VTI("flash manufactory id:%02x device id:%02x%02x", reg_val[0], reg_val[1], reg_val[2]);

	return 0;

}

static int bbk_fw_update(const struct firmware *firmware)
{
	int ret = 0;

	unsigned char *Image = (unsigned char *)kzalloc(firmware->size, GFP_KERNEL);

	if (!Image)
		return -ENOMEM;

	memcpy(Image, firmware->data, firmware->size);

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch_pd1831 (DISABLE);
#endif
	fts_fw_download(Image, firmware->size);
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch_pd1831 (ENABLE);
#endif

	kfree(Image);
	Image = NULL;

	return ret;

}

extern int fts_imei_read_8719_noflash(u8 *imei);
/*
 * description: spi driver probe
 */
static int fts_ts_probe(struct spi_device *spi)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata;
    struct fts_ts_data *ts_data;
	struct vivo_ts_struct *vts_data = NULL;

    FTS_FUNC_ENTER();
    if (spi->dev.of_node) {
        VTI("focaltech dts confige...");
        pdata = devm_kzalloc(&spi->dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
            VTE("allocate memory for platform data fail");
            return -ENOMEM;
        }
        ret = fts_parse_dt(&spi->dev, pdata);
        if (ret)
            VTE("[DTS]DT parsing failed");
    } else {
        pdata = spi->dev.platform_data;
    }

    if (!pdata) {
        VTE("no ts platform data found");
        return -EINVAL;
    }
#if 0
    if (spi->irq <= 0) {
        VTE("spi device has no irq");
        return -ENODEV;
    }
#endif
    
    ts_data = devm_kzalloc(&spi->dev, sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        VTE("allocate memory for fts_data fail");
        return -ENOMEM;
    }

	vts_data = vivoTsAlloc();
	g_vts_data_0flash = vts_data;
	if (vts_data == NULL) {
		VTE("vts_data alloc error!");
		goto free_ftsdata;
	}

	vts_data->busType = VTS_BUS_SPI;
	vts_data->imeiWriteSupport = UNSUPPORT;
	vts_data->spiClient = spi;
	vts_data->tsDimensionX = pdata->x_max;
	vts_data->tsDimensionY = pdata->y_max;
	vivoTsSetTxRxNum(16, 36);
	/*must set for key logic */
	/*vts_data->vkeyString = NULL; */
	vts_data->keyType = VTS_KEY_NOKEY;
	vts_data->isIndependentKeyIc = 0;
	vts_data->hasFingerPrint = 1;
	vts_data->hasHomeKey = 0;
	vts_data->hasMenuKey = 0;
	vts_data->hasBackKey = 0;	

	vts_data->icNum = VTS_IC_FT8719;
	vts_data->mtkOrQcom = VTS_PLATFORM_MTK_NEW;
	vts_data->noFlash = 1;	

	vts_data->icModeChange = focaltech_mode_change;

	vts_data->updateFirmware = bbk_fw_update;
	vts_data->setGestureSwitch = NULL;/*focaltech_set_gesture_switch_to_chip;		//if all gusture switch default on, this function no use */
	vts_data->getRawOrDiffData = focaltech_get_rawordiff_data;
	vts_data->getIcFirmwareOrConfigVersion = bbk_focaltech_fw_version;
	vts_data->getModuleId = focaltech_module_id;
#if FTS_GESTURE_EN
	vts_data->getGesturePointData = focaltech_gesture_point;
#endif
	vts_data->setChargerFlagSwitch = focaltech_charger_state_write;
	vts_data->getTsPinValue = NULL;
	vts_data->sensorTest = NULL;
	vts_data->setEdgeRestainSwitch = focaltech_edge_state_write;
	vts_data->getLcmId = bbk_xxx_get_lcm_id;
	vts_data->otherInfo = print_info;
	vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
	vts_data->irqBtmThreadHandler = fts_ts_work_func;
	vts_data->idleEnableOrDisable = focaltech_idle_state_write;
	vts_data->proFcFilterSwitch = 1;
	vts_data->readImei= fts_imei_read_8719_noflash;
	//vts_data->callbackType = 1;
	vts_data->sensorTestKey = "com.focaltouchscreen.sensortest:MainActivity:com.focaltouchscreen.sensortest:0:focal_test_result";
	vts_data->lcmNoiseTestKey = "com.focaltech.deltadiff:MainActivity:null:null:null";
	vts_data->rawdataTestKey = "com.focaltech.deltadiff:DataActivity:null:null:null";
	vivoTsFwAdd(VTS_FW_TYPE_FW, VIVO_CUST_FW_FT8719_SPI_JDI, sizeof(VIVO_CUST_FW_FT8719_SPI_JDI), "PD1831F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_JDI, 0x42);

	ret = vivoTsInit((void *)spi, NULL, -1);
	if (ret < 0) {
		goto free_vivo_ts;

	}
	/*this must be set up after vivoTsInit */
	vts_data->suspendEventBlank = FB_EVENT_BLANK;
	vts_data->resumeEventBlank = FB_EVENT_BLANK;

	ts_data->input_dev = vts_data->inputDev;/*change to our dev */
	ts_data->spi_buf = kzalloc(SPI_BUF_LENGTH, GFP_DMA);
	if (!ts_data->spi_buf) {
		VTE("allocate memory for spi_buf is failed!");	
		goto free_vivo_ts;
	}

    fts_data = ts_data;
	
    ts_data->spi = spi;
    ts_data->client = spi;
    ts_data->pdata = pdata;
    spi_set_drvdata(spi, ts_data);

	ts_data->spi_suspended = false;

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (NULL == ts_data->ts_workqueue) {
        VTE("failed to create fts workqueue");
    }

	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	spi->max_speed_hz = pdata->spi_frequency;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = FTS_SPI_CLK_MAX;
	
	ret = spi_setup(spi);
	if (ret) {
		VTE("spi setup fail");
		goto err_input_init;
	}

	VTI("mode=%d, max_speed_hz=%d\n", spi->mode, spi->max_speed_hz);

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->spilock);

    ret = fts_input_alloc(ts_data);
    if (ret) {
        VTE("input initialize fail");
        goto err_input_init;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        VTE("configure the gpios fail");
        goto err_gpio_config;
    }

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        VTE("ic type fail, please check driver setting");
        goto err_irq_req;
    }

#if FTS_APK_NODE_EN
    ret = fts_create_apk_debug_channel_0flash(ts_data);
    if (ret) {
        VTE("create apk debug node fail");
    }
#endif

#if FTS_SYSFS_NODE_EN
    ret = fts_create_sysfs_0flash(&spi->dev);
    if (ret) {
        VTE("create sysfs node fail");
    }
#endif

#if FTS_POINT_REPORT_CHECK_EN
    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        VTE("init point report check fail");
    }
#endif

    ret = fts_ex_mode_init_0flash(&spi->dev);
    if (ret) {
        VTE("init glove/cover/charger fail");
    }

#if FTS_GESTURE_EN
    ret = fts_gesture_init_0flash(ts_data);
    if (ret) {
        VTE("init gesture fail");
    }
#endif


#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init_pd1831(ts_data);
    if (ret) {
        VTE("init esd check fail");
    }
#endif

    ret = fts_irq_registration(ts_data);
    if (ret) {
        VTE("request irq failed");
        goto err_irq_req;
    }

    ret = fts_fw_init();
    if (ret) {
        VTE("init fw fail, tp fw not run");
    }

#if 0
#if defined(CONFIG_FB)
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        VTE("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif
#endif

	vivoTsAfterProbeCompleteCall(NULL, NULL, -1);

    FTS_FUNC_EXIT();
    return 0;

err_irq_req:
    if (gpio_is_valid(pdata->reset_gpio))
        gpio_free(pdata->reset_gpio);
    if (gpio_is_valid(pdata->irq_gpio))
        gpio_free(pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
    input_unregister_device(ts_data->input_dev);
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
    devm_kfree(&spi->dev, ts_data);

	kfree_safe(ts_data->spi_buf);
free_vivo_ts:
	vivoTsFree();
	VTE("free_vivo_ts");

free_ftsdata:

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove(struct spi_device *spi)
{
    struct fts_ts_data *ts_data = spi_get_drvdata(spi);

    FTS_FUNC_ENTER();

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit(ts_data);
#endif

#if FTS_APK_NODE_EN
    fts_release_apk_debug_channel_0flash(ts_data);
#endif

#if FTS_SYSFS_NODE_EN
    fts_remove_sysfs_0flash(&spi->dev);
#endif

#if FTS_GESTURE_EN
    fts_gesture_exit_0flash(ts_data);
#endif

    fts_ex_mode_exit_0flash(&spi->dev);


#if FTS_ESDCHECK_EN
    fts_esdcheck_exit_pd1831(ts_data);
#endif

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        VTE("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

    free_irq(ts_data->irq, ts_data);
    input_unregister_device(ts_data->input_dev);

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
	
	kfree_safe(ts_data->spi_buf);

    devm_kfree(&spi->dev, ts_data);
	
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
    //int ret = 0;
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_FUNC_ENTER();
    if (ts_data->suspended) {
        VTI("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        VTI("fw upgrade in process, can't suspend");
        return 0;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend_pd1831();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_suspend_0flash(ts_data) == 0) {
        ts_data->suspended = true;
        return 0;
    }
#endif
#if 0
    /* TP enter sleep mode */
    ret = fts_write_reg_byte(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
    if (ret < 0)
        VTE("set TP to sleep mode fail, ret=%d", ret);
#endif
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        VTD("Already in awake state");
        return 0;
    }

    fts_release_all_finger();
    fts_tp_state_recovery_0flash();

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume_pd1831();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_resume_0flash(ts_data) == 0) {
        ts_data->suspended = false;
        return 0;
    }
#endif

    ts_data->suspended = false;

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
* SPI Driver
*****************************************************************************/
static const struct spi_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(spi, fts_ts_id);

static struct of_device_id fts_match_table[] = {
    { .compatible = "vivo,ts-spi_v1", },
    { },
};

static int fts_ts_dev_suspend(struct device *dev)
{
	struct fts_ts_data *ts_spi = dev_get_drvdata(dev);
	ts_spi->spi_suspended = true;
	return 0;
}

static int fts_ts_dev_resume(struct device *dev)
{
	struct fts_ts_data *ts_spi = dev_get_drvdata(dev);
	ts_spi->spi_suspended = false;
	return 0;
}

static const struct dev_pm_ops fts_ts_pm = {
	.suspend = fts_ts_dev_suspend,
	.resume = fts_ts_dev_resume,
};


static struct spi_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = fts_match_table,
        .pm = &fts_ts_pm,
    },
    .id_table = fts_ts_id,
};

static void vivo_ts_fts_spi_driver_init(void)
{
	int ret = 0;
	ret = spi_register_driver(&fts_ts_driver);
    if (ret != 0) {
		VTE("Focaltech touch screen driver init failed!");
		return;
	}
	VTI("spi register success");
	return ;
}

static int __init fts_ts_init(void)
{
    enum boot_mode_t boot_mode;
	int ret;
	VTI("fts_ts_init enter 8719");
	
	if (0x42 != report_lcm_id()) {
		VTI("not 8719 0-flash");
		return 0;
	}
	
	boot_mode = get_boot_mode();
	VTI("boot_mode = %d", boot_mode);
	
	if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT ||
		boot_mode == LOW_POWER_OFF_CHARGING_BOOT ||
		boot_mode == META_BOOT ||
		boot_mode == FACTORY_BOOT ||
		boot_mode == ADVMETA_BOOT) {
		VTI("in (%d) mode, do not load driver", boot_mode);
		boot_mode_normal = false;
	} else {
		boot_mode_normal = true;
	}
	
	if (is_atboot == 1 || !boot_mode_normal) {
		VTI("TS is in at mode of power off charging mode.");
		return 0;
	}

	ret = vivo_touchscreen_new_module_init(vivo_ts_fts_spi_driver_init, "vivo_ts_fts_spi_driver_init");

    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    FTS_FUNC_ENTER();
    spi_unregister_driver(&fts_ts_driver);
    FTS_FUNC_EXIT();
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
