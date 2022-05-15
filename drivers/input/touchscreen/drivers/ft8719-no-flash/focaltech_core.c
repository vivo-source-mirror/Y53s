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

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define INTERVAL_READ_REG                   100  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data_V2;

/*****************************************************************************
* static variable
*****************************************************************************/
static int last_ts_state = VTS_ST_NORMAL;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct fts_ts_data *ts_data);
static int fts_ts_resume(struct fts_ts_data *ts_data);

#ifdef CONFIG_SPI_MT65XX
const struct mtk_chip_config ft_spi_ctrl = {
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.cs_pol = 0,
	.sample_sel = 0,
	.cs_setuptime = 25,
	.cs_holdtime = 12,
	.cs_idletime = 12,
};
#endif

/*
 * description: wait tp to run fw normal(Timeout: TIMEOUT_READ_REG),
 *              need call when reset/power on/resume...
 *
 * param -
 *
 * return 0 if tp valid, otherwise return error code
 */
int fts_wait_tp_to_valid_V2(void)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;
    u8 chip_id = fts_data_V2->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg_byte_V2(FTS_REG_CHIP_ID, &reg_value);
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
void fts_tp_state_recovery_V2(void)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid_V2();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */

#if FTS_GESTURE_EN
    fts_gesture_recovery_V2();
#endif
    FTS_FUNC_EXIT();
}

/*
 * description: Execute hw reset operation
 */
int fts_reset_proc_V2(int hdelayms)
{
	int ret = 0;
	
    FTS_FUNC_ENTER();
	if (gpio_is_valid(fts_data_V2->pdata.reset_gpio)) {
    	gpio_direction_output(fts_data_V2->pdata.reset_gpio, 0);
		mdelay(5);
    	gpio_direction_output(fts_data_V2->pdata.reset_gpio, 1);
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

void fts_irq_disable_V2(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data_V2->irq_lock, irqflags);

    if (!fts_data_V2->irq_disabled) {
        disable_irq_nosync(fts_data_V2->irq);
        fts_data_V2->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data_V2->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable_V2(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data_V2->irq_lock, irqflags);

    if (fts_data_V2->irq_disabled) {
        enable_irq(fts_data_V2->irq);
        fts_data_V2->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data_V2->irq_lock, irqflags);
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
    if (len > (fts_data_V2->pnt_buf_size - 3)) {
        len = fts_data_V2->pnt_buf_size - 3;
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

static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    u32 ik;
    int id = data->events[index].id;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int flag = data->events[index].flag;
    u32 key_num = data->pdata.key_number;

    if (!KEY_EN(data)) {
        return -EINVAL;
    }
    for (ik = 0; ik < key_num; ik++) {
        if (TOUCH_IN_KEY(x, data->pdata.key_x_coords[ik])) {
            if (EVENT_DOWN(flag)) {
                data->key_down = true;
				vts_report_event_down(fts_data_V2->vtsdev, data->pdata.keys[ik]);
                VTD("Key%d(%d, %d) DOWN!", ik, x, y);
            } else {
                data->key_down = false;
				vts_report_event_up(fts_data_V2->vtsdev, data->pdata.keys[ik]);
                VTD("Key%d(%d, %d) Up!", ik, x, y);
            }
            return 0;
        }
    }

    VTE("invalid touch for key, [%d](%d, %d)", id, x, y);
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int get_touch_count(int touch_bitmap)
{
	int i;
	int count = 0;

	for (i = 0; i < touch_bitmap; i++) {
		if (BIT(i) & touch_bitmap)
			count++;
	}

	return count;
}
static int fts_input_report_b(struct fts_ts_data *data, ktime_t kt)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata.max_touch_number;
    u32 key_y_coor = data->pdata.key_y_coord;
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
			vts_report_point_down(fts_data_V2->vtsdev, events[i].id, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, data->custom_data, sizeof(data->custom_data), kt);
        } else {
            uppoint++;
			data->touchs &= ~BIT(events[i].id);
			vts_report_point_up(fts_data_V2->vtsdev, events[i].id, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, kt);
        }
    }
    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                VTI("[B1]P%d UP!", i);
                va_reported = true;
				vts_report_point_up(fts_data_V2->vtsdev, i, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, kt);
            }
        }
    }
    data->touchs = touchs;
    if (va_reported) {
        /* touchs==0, there's no point but key */
		if (EVENT_NO_DOWN(data) || (!touchs)) {
        	vts_report_release(fts_data_V2->vtsdev);
		}
    }

	vts_report_point_sync(fts_data_V2->vtsdev);
	
    return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *data, ktime_t kt)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata.max_touch_number;
    u8 *buf = data->point_buf;
	u8 flag = 0;

    data->point_num = 0;
    data->touch_point = 0;

    memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x01;
    ret = fts_read_V2(NULL, 0, buf + 1, data->pnt_buf_size - 1);
    flag = buf[1] & 0xF0;
    if ((ret < 0) && (flag != 0x90)) {
        /* check if need recovery fw */
		vts_report_release(data->vtsdev);
        focal_fw_recovery();
        return ret;
    }  else if ((ret < 0) || (flag != 0x90)) {
        FTS_ERROR("touch data(%x) fail,ret:%d", buf[1], ret);
		 return -EIO;
    }
		data->custom_data[0] = (buf[91] & 0x04) >> 2; //charge bit
		data->custom_data[1] = buf[91] & 0x03; //frequency

#if FTS_GESTURE_EN
    ret = fts_gesture_readdata_V2(data, buf + FTS_GESTURE_OFF);
    if (0 == ret) {
        VTI("succuss to get gesture data in irq handler");
        return 1;
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    if ((data->point_num == 0x0F) && (buf[2] == 0xFF)
        && (buf[3] == 0xFF) && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
        VTD("touch buff is 0xff, need recovery state");
		vts_report_release(data->vtsdev);
        fts_tp_state_recovery_V2();
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
    if (data->touch_point == 0) {
        VTE("no touch point information");
		if ((buf[1] & 0xFF) == 0x91) {
			VTI("got Palm state");
			vts_report_release(data->vtsdev);
			vts_report_event_down(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
			vts_report_event_up(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
		}
        return -EIO;
    }

    return 0;
}

static void fts_report_event(struct fts_ts_data *data, ktime_t kt)
{
#if FTS_MT_PROTOCOL_B_EN
    fts_input_report_b(data, kt);
#endif
}

static irqreturn_t fts_ts_work_func(int irq, void *handle, ktime_t kt)
{
	int ret = -1;

	VTD("======enter=");

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif

	ret = fts_read_touchdata(fts_data_V2, kt);

	if (ret == 0) {
		mutex_lock(&fts_data_V2->report_mutex);
		fts_report_event(fts_data_V2, kt);
		mutex_unlock(&fts_data_V2->report_mutex);
	}
	
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif

	VTD("=======exit=");
	return IRQ_HANDLED;

}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = &ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    VTI("irq in ts_data:%d irq in client:%d", ts_data->irq, ts_data->spi->irq);
    if (ts_data->irq != ts_data->spi->irq)
        VTE("IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");

    if (0 == pdata->irq_gpio_flags)
        pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING;
    VTI("irq flag:%x", pdata->irq_gpio_flags);;
	ret = vts_interrupt_register(ts_data->vtsdev, ts_data->irq, fts_ts_work_func, pdata->irq_gpio_flags | IRQF_TRIGGER_FALLING, ts_data);
	if (ret) {
			VTE("Request irq failed!");
	} else {
			VTI("request irq %d succeed", ts_data->irq);
	}
    return ret;
}

static int fts_irq_unregistration(struct fts_ts_data *ts_data)
{
	vts_interrupt_unregister(ts_data->vtsdev);
    return 0;
}

static int fts_input_alloc(struct fts_ts_data *ts_data)
{
	int ret = 0;

    ts_data->pnt_buf_size = FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 5 + FTS_GESTURE_DATA_LEN;
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        VTE("failed to alloc memory for point buf!");
        ret = -ENOMEM;
        goto err_point_buf;
    }

    ts_data->events = (struct ts_event *)kzalloc(FTS_MAX_POINTS_SUPPORT * sizeof(struct ts_event), GFP_KERNEL);
    if (!ts_data->events) {
        VTE("failed to alloc memory for point events!");
        ret = -ENOMEM;
        goto err_event_buf;
    }

    return 0;

err_event_buf:
    kfree_safe(ts_data->point_buf);

err_point_buf:
	return ret;

}

static void fts_input_free(struct fts_ts_data *ts_data)
{
	kfree_safe(ts_data->events);
	kfree_safe(ts_data->point_buf);
}

/*
 * description: configure reset & irq gpio
 */
static int fts_gpio_init(struct fts_ts_data *data)
{
    int ret = 0;
    /* request irq gpio */
    if (gpio_is_valid(data->pdata.irq_gpio)) {
        ret = gpio_request(data->pdata.irq_gpio, "fts_irq_gpio");
        if (ret) {
            VTE("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata.irq_gpio);
        if (ret) {
            VTE("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata.reset_gpio)) {
        ret = gpio_request(data->pdata.reset_gpio, "fts_reset_gpio");
        if (ret) {
            VTE("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata.reset_gpio, 1);
        if (ret) {
            VTE("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
	}

#if defined(CONFIG_ARCH_QCOM)
	/* request cs gpio */
    if (gpio_is_valid(data->pdata.cs_gpio)) {
        ret = gpio_request(data->pdata.cs_gpio, "fts_cs_gpio");
        if (ret) {
            VTE("[GPIO]cs gpio request failed");
            goto err_reset_gpio_dir;
        }

	    ret = gpio_direction_output(data->pdata.cs_gpio, 1);
	    if (ret) {
	        VTE("[GPIO]set_direction for cs gpio failed");
	        goto err_cs_gpio_dir;
	    }
   }
#endif

    return 0;
#if defined(CONFIG_ARCH_QCOM)
err_cs_gpio_dir:
	if (gpio_is_valid(data->pdata.cs_gpio))
        gpio_free(data->pdata.cs_gpio);
#endif
err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata.reset_gpio))
        gpio_free(data->pdata.reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata.irq_gpio))
        gpio_free(data->pdata.irq_gpio);
err_irq_gpio_req:
    return ret;
}

static void fts_gpio_exit(struct fts_ts_data *data)
{
	if (gpio_is_valid(data->pdata.reset_gpio))
        gpio_free(data->pdata.reset_gpio);
	if (gpio_is_valid(data->pdata.irq_gpio))
        gpio_free(data->pdata.irq_gpio);
}

static int fts_get_dt_coords(struct device_node *np, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
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
static int fts_parse_dt(struct device_node *np, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 temp_val = 0;

    FTS_FUNC_ENTER();

    ret = fts_get_dt_coords(np, "focaltech,display-coords", pdata);
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
	#if defined(CONFIG_ARCH_QCOM)
	pdata->cs_gpio = of_get_named_gpio_flags(np, "focaltech,cs-gpio", 0, &pdata->cs_gpio_flags);
    if (pdata->cs_gpio < 0)
        VTE("Unable to get cs_gpio");
    #endif
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
	ret = of_property_read_u32(np, "spi-max-frequency", &temp_val);
	if (0 == ret && temp_val > 0 && temp_val <= 9600000) {
	  pdata->spi_frequency = temp_val;
	} else {
	  /*default frequency*/
	  pdata->spi_frequency = 6000000;
	}

    VTI("max touch number:%d, irq gpio:%d, reset gpio:%d, spi frequency:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio, pdata->spi_frequency);

    FTS_FUNC_EXIT();
    return 0;
}

static void fts_fw_recovery_if_needed(struct fts_ts_data *data)
{
	u8 *buf = data->point_buf;
	int ret;
	u8 flag = 0;

	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x01;
    ret = fts_read_V2(buf, 1, buf + 1, data->pnt_buf_size - 1);
    flag = buf[1] & 0xF0;
    if ((ret < 0) && (flag != 0x90))
        focal_fw_recovery();
}

static int focaltech_mode_change(struct vts_device *vtsdev, int which)
{
	int ret = 0;
	int retry = 2;
      u8  type = 0;

	VTI("fts: +++0:normal,1:sleep,2:gesture+++, which = %d", which);

	if (which == VTS_ST_NORMAL) {
		VTI("change to *****normal mode*****");
#if defined(CONFIG_ARCH_QCOM)
		if (gpio_is_valid(fts_data_V2->pdata.cs_gpio))
			gpio_set_value(fts_data_V2->pdata.cs_gpio, 1);
#endif

		if (VTS_ST_SLEEP == last_ts_state) {
			mdelay(5);
			fts_fw_recovery_if_needed(fts_data_V2);
			fts_wait_tp_to_valid_V2();
		} else {
		    fts_read_reg_byte_V2(FTS_REG_FW_TYPE,&type);
			if(type != 0xAA){
				VTE("touch fw is not in normal, need download fw !!!");
				ret = fts_fw_resume_V2();
				if(ret < 0){
					VTE("fts_fw_resume_V2 fail");
					return ret;
					}
				}
			}

		ret = fts_ts_resume(fts_data_V2);
#if FTS_GESTURE_EN
		fts_gesture_mode_set_V2(false);
#endif
		//fts_irq_enable_V2();
	}

	if (which == VTS_ST_GESTURE) {
		VTI("change to *****gesture mode*****");
		if (VTS_ST_SLEEP == last_ts_state) {
			vts_dsi_panel_reset_power_ctrl(1);
#if defined(CONFIG_ARCH_QCOM)
		if (gpio_is_valid(fts_data_V2->pdata.cs_gpio))
			gpio_set_value(fts_data_V2->pdata.cs_gpio, 1);
#endif
			//VTI("change to gesture mode========= mdelay 290");
			mdelay(5);
			fts_fw_recovery_if_needed(fts_data_V2);
			fts_wait_tp_to_valid_V2();
			
			 while (retry--) {
				ret = fts_write_reg_byte_V2(0xfd, 0x5a);
				if(ret < 0) {
					VTI("fail to wirte 0xfd register " );
					msleep(10);
				} else {
					VTI("wirte 0xfd success" );
					break;
				}	
			} /*cmd for lcd deep standby when TP sleep to gesture*/
		}

#if FTS_GESTURE_EN
		fts_gesture_mode_set_V2(true);
#endif
		fts_data_V2->suspended = 0;
		fts_data_V2->touchs = 0;
		ret = fts_ts_suspend(fts_data_V2);
		//fts_irq_enable_V2();
	}
	if (which == VTS_ST_SLEEP) {
		VTI("change to *****sleep mode*****");

#if FTS_ESDCHECK_EN
		fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
		/*fts_gesture_mode_set_V2(false);*/
#endif

		fts_data_V2->suspended = true;
		fts_data_V2->touchs = 0;
		//fts_irq_disable_V2();

#if 1
		if (last_ts_state == VTS_ST_NORMAL) {
			VTI("Set TP to sleep mode");
			/* TP enter sleep mode */
			ret = fts_write_reg_byte_V2(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
			if (ret < 0) {
				VTI("Set TP to sleep mode fail, ret=%d!", ret);
			}
			//msleep(80);   /*A5 write 03 need delay 80ms before AVDD&AVEE lost*/
		} else if (last_ts_state == VTS_ST_GESTURE) { 
			if (gpio_is_valid(fts_data_V2->pdata.reset_gpio)) {
				gpio_direction_output(fts_data_V2->pdata.reset_gpio, 0);
				mdelay(5);
				gpio_direction_output(fts_data_V2->pdata.reset_gpio, 1);
			}else {
				VTE("fts reset gpio is not valid");
			}
			
			msleep(100);
			
			fts_wait_tp_to_valid_V2();
			VTI("Set TP to sleep mode...");
			ret = fts_write_reg_byte_V2(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
			if (ret < 0) {
				VTI("Set TP to sleep mode fail, ret=%d!", ret);
			}
			//msleep(250);			
		}
#endif
	

#if defined(CONFIG_ARCH_QCOM)
		msleep(250);
		if (gpio_is_valid(fts_data_V2->pdata.cs_gpio))
			gpio_set_value(fts_data_V2->pdata.cs_gpio, 0);
#elif defined(CONFIG_MEDIATEK_SOLUTION)
		msleep(80);
#endif

		/*to protect "close to prox" - "resume", if we lock, mdss(2) -> mdss (0);  mdss(0) will prtect by mdss */
		/* early event -> lcm power on (mdss (2)) -> proxi_switch (0) */
		vts_dsi_panel_reset_power_ctrl(0);

		msleep(50);
	}

	last_ts_state = which;

	VTI("fts: ----mode_change sucessfully----");

	return ret;
}

static int bbk_focaltech_fw_version(struct vts_device *vtsdev, u64 *version)
{
	unsigned char ver = 0;
	int ret = 0;

	ret = fts_read_reg_byte_V2(0xA6, &ver);
	if (ver < 0) {
		VTE("read fw version fail.");
	}

	*version = (u64)ver;
	return ret;
}
static int focaltech_get_touch_ic_mode(struct vts_device *vtsdev)
{
	u8 ver = 0;
	int ret = 0;

	ret = fts_read_reg_byte_V2(0xD0, &ver);
	if (ver < 0) {
		VTE("read touch_ic_mode fail.");
		return 0;
	}

	ver = !(ver&0x01);
	return ver;
}
static int focaltech_charger_state_write(struct vts_device *vtsdev, int state)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 charge_state = (u8)(state?1:0);

	VTI("write charger state: %d ", charge_state);

	for (i = 0; i < 10; i++) {
		ret = fts_write_reg_byte_V2(FTS_REG_CHARGER_MODE_EN, charge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8b register %d", charge_state);

		mdelay(5);

		ret = fts_read_reg_byte_V2(FTS_REG_CHARGER_MODE_EN, &temp_data);
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

static int focaltech_idle_state_write(struct vts_device *vtsdev, int state)
{
	int i = 0;
	int ret = 0;
	u8 idle_state = (u8)(state?1:0);
	u32 idle_time = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);
	VTI("write idle state: %d, idle time: %d ", idle_state, idle_time);

	if (idle_time != 0) {
		VTI("new idle addr");
		for (i = 0; i < 10; i++) {
			if (idle_state == 1) {//idle_state 1-normal 0-game
				ret = fts_write_then_read(0xB8, 0);
				if (ret < 0)
					continue;
				break;
			} else {
				ret = fts_write_then_read(0xB9, idle_time);
				if (ret < 0)
					continue;

				ret = fts_write_then_read(0xB8, 1);
				if (ret < 0)
					continue;
				break;
			}
		}
	} else {
		VTI("old idle addr");
		for (i = 0; i < 10; i++) {
			ret = fts_write_then_read(0x86, idle_state);
			if (ret < 0)
				continue;
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

static int focaltech_edge_state_write(struct vts_device *vtsdev, int on)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 edge_state = 0;

	switch (on) {
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
		ret = fts_write_reg_byte_V2(0x8c, edge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8a register %d", edge_state);

		mdelay(5);

		ret = fts_read_reg_byte_V2(0x8c, &temp_data);
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

static int print_info(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{

	int ret;
	u8 initcode_ver = 0;
	u8 auc_i2c_write_buf[10] = {0};
	u8 reg_val[4] = {0};

	ret = fts_read_reg_byte_V2(0xE4, &initcode_ver);
	if (ret < 0)
		VTI("fail to wirte 0x8a register %d", initcode_ver);
	else
		VTI("read init code version = %d", initcode_ver);

	auc_i2c_write_buf[0] = 0x7D;
	ret = fts_read_V2(auc_i2c_write_buf, 1, reg_val, 3);
	if (ret < 0)
		VTI("fail to read 0x02 register ");
	else
		VTI("flash manufactory id:%02x device id:%02x%02x", reg_val[0], reg_val[1], reg_val[2]);

	return 0;

}

static int bbk_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)
{
	int ret = 0;

	if (!firmware->data)
		return -ENOMEM;

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (DISABLE);
#endif
	fts_fw_download_V2(firmware->data, firmware->size);
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (ENABLE);
#endif
	return ret;

}

static int fts_hw_init(struct vts_device *vtsdev)
{
	struct fts_ts_data *ts_data = vts_get_drvdata(vtsdev);
	int ret;

	ret = fts_gpio_init(ts_data);
    if (ret) {
        VTE("configure the gpios fail");
        return ret;
    }

#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init(ts_data);
    if (ret) {
		fts_gpio_exit(ts_data);
        VTE("init esd check fail");
		return ret;
    }
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		VTE("request irq failed");
#if FTS_ESDCHECK_EN
		fts_esdcheck_exit(ts_data);
#endif
		fts_gpio_exit(ts_data);
		return ret;
	}

	return 0;
}

static int fts_hw_exit(struct vts_device *vtsdev)
{
	struct fts_ts_data *ts_data = vts_get_drvdata(vtsdev);

	fts_irq_unregistration(ts_data);
#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif
	fts_gpio_exit(ts_data);
	return 0;
}

static int fts_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 15;
	return 0;
}

static ssize_t fts_flash_read(struct vts_device *vtsdev, u8*buf, size_t nbytes)
{
	int ret = 0;
	u8 data[9] = { 0 };
	u8 cmd = 0x92;
	u8 mode = 0;
	u32 unique_code_new = 0;

	ret = fts_write_reg_byte_V2(0x00,0x40);
	msleep(200);

	ret = fts_read_reg_byte_V2(0x00, &mode);
	if ((ret >=0 ) && (0x40 == mode)) {
		VTI("enter factory mode success");
	} else {
		VTE("enter factory mode failed");
		goto error_end;
	}

	ret = fts_write_reg_byte_V2(0x92,0x00);
	msleep(10);
	ret = fts_read_V2(&cmd, 1, data, 9);

	memset(buf, 0, nbytes);
	vts_property_get(vtsdev, VTS_PROPERTY_NO_FLASH_UNIQUE_CODE, &unique_code_new);
	if (unique_code_new != 0) {

		VTI("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);

		memcpy(buf, data, nbytes>9? 9:nbytes);
	} else {
		data[0] = data[0] & 0x07;
		data[1] = data[1] & 0x0F;
		data[2] = data[2] & 0x1F;
		data[4] = data[4] & 0x0F;
		data[5] = data[5] & 0x1F;

		VTI("%x,%x,%x,%x,%x,%x,%x,%x,%x\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);

		memcpy(buf, data, nbytes>9? 9:nbytes);
	}

	ret = fts_write_reg_byte_V2(0x00, 0x00);
	msleep(200);

	ret = fts_read_reg_byte_V2(0x00, &mode);
	if ((ret >=0 ) && (0x00 == mode)) {
		VTI("enter work mode success");
	} else {
		VTE("enter work mode failed");
		goto error_end;
	}

	return nbytes;

error_end:
	fts_write_reg_byte_V2(0x00,0x00);
	msleep(200);
	nbytes = -1;
	return nbytes;

}

static int fts_set_gesture(struct vts_device *vtsdev,int enable)
{
	int gesture_type = vts_state_get(vtsdev,VTS_STA_GESTURE);
	u8 gest_switch = 1;//0xd0
	u8 char_switch = 0;//0xd2 bit0-bit4 : O W M E C
	u8 LR_switch = 0;//0xd1 : left right up down doubleclick
	u8 at_switch = 1;//0xd6 : @  0x01-on 0x00-off
	u8 f_switch= 0x10;//0xd8 : F 0x10-on 0x00-off

	VTI("set fts switch gesture_type:%04X", gesture_type);
	char_switch = (gesture_type >> 2) & 0x1f;
	LR_switch |= (gesture_type & VTS_GESTURE_LR);
	LR_switch |= (gesture_type & VTS_GESTURE_LR) << 1;
	LR_switch |= (gesture_type & VTS_GESTURE_UP) << 1;
	LR_switch |= (gesture_type & VTS_GESTURE_DOWN) >> 4;
	LR_switch |= (gesture_type & VTS_GESTURE_DCLICK) >> 4;
	at_switch = (gesture_type & VTS_GESTURE_A) >> 9;
	f_switch = (gesture_type & VTS_GESTURE_F) >> 6;

	fts_write_then_read(0xd0,gest_switch);
	fts_write_then_read(0xd1,LR_switch);
	fts_write_then_read(0xd2,char_switch);
	fts_write_then_read(0xd6,at_switch);
	fts_write_then_read(0xd8,f_switch);
	return 0;
}

static const struct vts_operations fts_vts_ops = {
	.init = fts_hw_init,
	.exit = fts_hw_exit,
	.change_mode = focaltech_mode_change,
	.update_firmware = bbk_fw_update,
	.get_frame = focaltech_get_rawordiff_data_V2,
	.get_fw_version = bbk_focaltech_fw_version,
	.get_ic_mode = focaltech_get_touch_ic_mode,
	.set_charging = focaltech_charger_state_write,
	.set_rotation = focaltech_edge_state_write,
	.otherInfo = print_info,
	.set_auto_idle = focaltech_idle_state_write,
	.rom_size = fts_get_flash_size,
	.rom_read = fts_flash_read,
	.set_gesture = fts_set_gesture,
};

static int fts_ts_probe(struct spi_device *spi, struct device_node *np)
{
    int ret = 0;
    struct fts_ts_data *ts_data;
	struct vts_device *vtsdev = NULL;
    
    ts_data = devm_kzalloc(&spi->dev, sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        VTE("allocate memory for fts_data_V2 fail");
        return -ENOMEM;
    }

	ts_data->spi_buf = devm_kzalloc(&spi->dev, SPI_BUF_LENGTH, GFP_KERNEL | GFP_DMA);
	if (!ts_data->spi_buf) {
		VTE("allocate memory for spi_buf is failed!");
		ret = -ENOMEM;
		goto errorcode1;
	}

	ts_data->spi_cmd = devm_kzalloc(&spi->dev, SPI_MAX_COMMAND_LENGTH, GFP_KERNEL | GFP_DMA);
	if (!ts_data->spi_cmd) {
		VTE("allocate memory for spi_cmd is failed!");
		ret = -ENOMEM;
		goto errorcode2;
	}
	ts_data->dbg_read_buf = (u8 *)kzalloc(FTS_DBUG_BUF_MAX_LEN, GFP_KERNEL);
	if(!ts_data->dbg_read_buf)	{
		VTE("allocate memory for dbg_read_buf is failed!");
		ret = -ENOMEM;
		goto errorcode3;
	}
	ts_data->dbg_write_buf = (u8 *)kzalloc(FTS_DBUG_BUF_MAX_LEN, GFP_KERNEL);
	if(!ts_data->dbg_write_buf)	{
		VTE("allocate memory for dbg_write_buf is failed!");
		ret = -ENOMEM;
		goto errorcodedbgr;
	}

	ts_data->dbg_temp_buf = (char *)kzalloc(FTS_DBUG_BUF_MAX_LEN, GFP_KERNEL);
	if(!ts_data->dbg_temp_buf)	{
		VTE("allocate memory for dbg_temp_buf is failed!");
		ret = -ENOMEM;
		goto errorcodedbgw;
	}

	ret = fts_parse_dt(np, &ts_data->pdata);
    if (ret) {
        VTE("[DTS]DT parsing failed");
		goto errorcodedbgt;
    }

    fts_data_V2 = ts_data;
    ts_data->spi = spi;
    spi_set_drvdata(spi, ts_data);
#ifdef CONFIG_SPI_MT65XX
	memcpy(&ts_data->spi_ctrl, &ft_spi_ctrl, sizeof(struct mtk_chip_config));
	ts_data->spi->controller_data = (void *)&ts_data->spi_ctrl;
#endif
	spi->mode = SPI_MODE_1;
	spi->max_speed_hz = ts_data->pdata.spi_frequency;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = FTS_SPI_CLK_MAX;
	
	ret = spi_setup(spi);
	if (ret) {
		VTE("spi setup fail");
		goto errorcode4;
	}

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->spilock);
	mutex_init(&ts_data->mutex);

    ret = fts_input_alloc(ts_data);
    if (ret) {
        VTE("input initialize fail");
        goto errorcode5;
    }

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        VTE("ic type fail, please check driver setting");
        goto errorcode6;
    }

#if FTS_APK_NODE_EN
    ret = fts_create_apk_debug_channel_V2(ts_data);
    if (ret) {
        VTE("create apk debug node fail");
		goto errorcode7;
    }
#endif

#if FTS_SYSFS_NODE_EN
    ret = fts_create_sysfs_V2(&spi->dev);
    if (ret) {
        VTE("create sysfs node fail");
		goto errorcode8;
    }
#endif

#if FTS_GESTURE_EN
    ret = fts_gesture_init_V2(ts_data);
    if (ret) {
        VTE("init gesture fail");
		goto errorcode9;
    }
#endif

	vtsdev = vts_device_alloc();
	if (!vtsdev) {
		ret = -ENOMEM;
		goto errorcode10;
	}

	vtsdev->ops = &fts_vts_ops;
	vtsdev->busType = BUS_SPI;
	ts_data->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, np);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto errorcode11;
	}
	vts_set_drvdata(vtsdev, ts_data);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {		
		VTE("vts_register_driver failed");
		goto errorcode11;
	}

    return 0;
errorcode11:
	vts_device_free(vtsdev);
errorcode10:
#if FTS_GESTURE_EN
	fts_gesture_exit_V2(ts_data);
errorcode9:
#endif
#if FTS_SYSFS_NODE_EN
	fts_remove_sysfs_V2(&spi->dev);
errorcode8:
#endif
#if FTS_APK_NODE_EN
	fts_release_apk_debug_channel_V2(ts_data);
errorcode7:
#endif
errorcode6:
	fts_input_free(ts_data);
errorcode5:
	mutex_destroy(&ts_data->mutex);
	mutex_destroy(&ts_data->spilock);
	mutex_destroy(&ts_data->report_mutex);
errorcode4:
	fts_data_V2 = NULL;
	spi_set_drvdata(spi, NULL);
errorcodedbgt:	
	kfree_safe(ts_data->dbg_temp_buf);
errorcodedbgw:	
	kfree_safe(ts_data->dbg_write_buf); 
errorcodedbgr:
	kfree_safe(ts_data->dbg_read_buf);
errorcode3:
	devm_kfree(&spi->dev, ts_data->spi_cmd);
errorcode2:
	devm_kfree(&spi->dev, ts_data->spi_buf);
errorcode1:
	devm_kfree(&spi->dev, ts_data);
	return ret;
}

static int fts_ts_remove(struct spi_device *spi, struct device_node *np)
{
    struct fts_ts_data *ts_data = spi_get_drvdata(spi);
	struct vts_device *vtsdev = ts_data->vtsdev;

	vts_unregister_driver(vtsdev);
	vts_device_free(vtsdev);
#if FTS_GESTURE_EN
	fts_gesture_exit_V2(ts_data);
#endif
#if FTS_SYSFS_NODE_EN
	fts_remove_sysfs_V2(&spi->dev);
#endif
#if FTS_APK_NODE_EN
	fts_release_apk_debug_channel_V2(ts_data);
#endif
	fts_input_free(ts_data);
	mutex_destroy(&ts_data->mutex);
	mutex_destroy(&ts_data->spilock);
	mutex_destroy(&ts_data->report_mutex);
	fts_data_V2 = NULL;
	spi_set_drvdata(spi, NULL);
	devm_kfree(&spi->dev, ts_data->spi_buf);
	devm_kfree(&spi->dev, ts_data->spi_cmd);
	devm_kfree(&spi->dev, ts_data);
    return 0;
}

static int fts_ts_suspend(struct fts_ts_data *ts_data)
{
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
    fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_suspend_V2(ts_data) == 0) {
        ts_data->suspended = true;
        return 0;
    }
#endif
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_resume(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        VTD("Already in awake state");
        return 0;
    }

	vts_report_release(ts_data->vtsdev);
    fts_tp_state_recovery_V2();

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

#if FTS_GESTURE_EN
    if (fts_gesture_resume_V2(ts_data) == 0) {
        ts_data->suspended = false;
        return 0;
    }
#endif

    ts_data->suspended = false;

    FTS_FUNC_EXIT();
    return 0;
}

static struct vts_spi_driver fts_ts_driver = {
	.probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .compatible = "focaltech,ft8719_v2",
};

static const int ic_number[1] = {VTS_IC_FT8719};
module_vts_driver(ft8719_no_flash, ic_number, vts_spi_drv_reigster(&fts_ts_driver),  vts_spi_drv_unreigster(&fts_ts_driver));

