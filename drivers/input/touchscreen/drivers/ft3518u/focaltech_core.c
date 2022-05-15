/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
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
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define FTS_DRIVER_PEN_NAME                 "fts_ts,pen"
#define INTERVAL_READ_REG                   200	/* unit:ms */
#define TIMEOUT_READ_REG                    1000	/* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      3000000
#define FTS_VTG_MAX_UV                      3000000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif

/**vivo_ts start*/
#define FTS_VIVO_IDLE_CTL_REG       0x86
#define FTS_VIVO_EDGE_CTL_REG       0x8C
#define FTS_VIVO_SCANRATE_REG       0x88
#define FTS_VIVO_WP_REG             0xFD
#define FTS_VIVO_ACTIVE_REG         0xA5
#define FTS_VIVO_PALM_REG           0x01
#define FTS_VIVO_FOD_EN_REG         0xCF
#define FTS_VIVO_FOD_INFO_REG       0xE1
#define FTS_FOD_INFO_LEN            9
#define FTS_VIVO_FOD_MODE_REG       0x87

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *ft3518u_fts_data;
//char g_debug_buf[DEBUG_BUF_SIZE];

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

#ifdef CONFIG_SPI_MT65XX
static const struct mtk_chip_config spi_ctrldata = {
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.cs_pol = 0,
	.sample_sel = 0,
	.cs_setuptime = 25,
	.cs_holdtime = 12,
	.cs_idletime = 12,
};
#endif

int ft3518u_fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
	int i = 0;
	struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
	u8 cid_h = 0x0;

	if (cid->type == 0)
		return -ENODATA;

	for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
		cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
		if (cid_h && (id_h == cid_h)) {
			return 0;
		}
	}

	return -ENODATA;
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int ft3518u_fts_wait_tp_to_valid(void)
{
	int ret = 0;
	int cnt = 0;
	u8 idh = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;
	u8 chip_idh = ts_data->ic_info.ids.chip_idh;

	do {
		ret = ft3518u_fts_read_reg(FTS_REG_CHIP_ID, &idh);
		if ((idh == chip_idh) || (ft3518u_fts_check_cid(ts_data, idh) == 0)) {
			FTS_INFO("TP Ready,Device ID:0x%02x", idh);
			return 0;
		} else
			FTS_DEBUG("TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void ft3518u_fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	ft3518u_fts_wait_tp_to_valid();
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	ft3518u_fts_ex_mode_recovery(ts_data);
	/* recover TP gesture state 0xD0 */
	ft3518u_fts_gesture_recovery(ts_data);

	ft3518u_fts_write_reg(0x92, ts_data->reset_reason);
	ts_data->reset_reason = 1; // passive

	FTS_FUNC_EXIT();
}

int ft3518u_fts_reset_proc(int hdelayms)
{
	FTS_DEBUG("tp reset");
	ft3518u_fts_data->reset_reason = 0; // initiative
	gpio_direction_output(ft3518u_fts_data->pdata->reset_gpio, 0);
	msleep(1);
	gpio_direction_output(ft3518u_fts_data->pdata->reset_gpio, 1);
	if (hdelayms) {
		msleep(hdelayms);
	}

	return 0;
}

void ft3518u_fts_irq_disable(void)
{
	unsigned long irqflags;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&ft3518u_fts_data->irq_lock, irqflags);

	if (!ft3518u_fts_data->irq_disabled) {
		disable_irq_nosync(ft3518u_fts_data->irq);
		ft3518u_fts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&ft3518u_fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void ft3518u_fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&ft3518u_fts_data->irq_lock, irqflags);

	if (ft3518u_fts_data->irq_disabled) {
		enable_irq(ft3518u_fts_data->irq);
		ft3518u_fts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&ft3518u_fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void ft3518u_fts_hid2std(void)
{
}

static int fts_match_cid(struct fts_ts_data *ts_data,
			 u16 type, u8 id_h, u8 id_l, bool force)
{
#ifdef FTS_CHIP_ID_MAPPING
	u32 i = 0;
	u32 j = 0;
	struct ft_chip_id_t chip_id_list[] = FTS_CHIP_ID_MAPPING;
	u32 cid_entries = sizeof(chip_id_list) / sizeof(struct ft_chip_id_t);
	u16 id = (id_h << 8) + id_l;

	memset(&ts_data->ic_info.cid, 0, sizeof(struct ft_chip_id_t));
	for (i = 0; i < cid_entries; i++) {
		if (!force && (type == chip_id_list[i].type)) {
			break;
		} else if (force && (type == chip_id_list[i].type)) {
			FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
			ts_data->ic_info.cid = chip_id_list[i];
			return 0;
		}
	}

	if (i >= cid_entries) {
		return -ENODATA;
	}

	for (j = 0; j < FTS_MAX_CHIP_IDS; j++) {
		if (id == chip_id_list[i].chip_ids[j]) {
			FTS_DEBUG("cid:%x==%x", id, chip_id_list[i].chip_ids[j]);
			FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
			ts_data->ic_info.cid = chip_id_list[i];
			return 0;
		}
	}

	return -ENODATA;
#else
	return -EINVAL;
#endif
}

static int fts_get_chip_types(struct fts_ts_data *ts_data,
				  u8 id_h, u8 id_l, bool fw_valid)
{
	u32 i = 0;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((0x0 == id_h) || (0x0 == id_l)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (VALID == fw_valid) {
			if (((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
				|| (!fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 0)))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
				|| ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
				|| ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
				break;
			}
		}
	}

	if (i >= ctype_entries) {
		return -ENODATA;
	}

	fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 1);
	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 * id)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = ft3518u_fts_write(id_cmd, 2);
	if (ret < 0) {
		FTS_ERROR("start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	ret = ft3518u_fts_read(id_cmd, 1, chip_id, 2);
	if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
		FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };

	ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

	do {
		ret = ft3518u_fts_read_reg(FTS_REG_CHIP_ID, &chip_id[0]);
		ret = ft3518u_fts_read_reg(FTS_REG_CHIP_ID2, &chip_id[1]);
		if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
			FTS_DEBUG("chip id read invalid, read:0x%02x%02x",
				  chip_id[0], chip_id[1]);
		} else {
			ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
			if (!ret)
				break;
			else
				FTS_DEBUG("TP not ready, read:0x%02x%02x",
					  chip_id[0], chip_id[1]);
		}

		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
		FTS_INFO("fw is invalid, need read boot id");
		if (ts_data->ic_info.hid_supported) {
			ft3518u_fts_hid2std();
		}

		ret = fts_read_bootid(ts_data, &chip_id[0]);
		if (ret < 0) {
			FTS_ERROR("read boot id fail");
			return ret;
		}

		ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
		if (ret < 0) {
			FTS_ERROR("can't get ic informaton");
			return ret;
		}
	}

	FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
		 ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
		 ts_data->ic_info.cid.type);

	return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
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
	u32 max_touch_num = data->pdata->max_touch_number;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
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
			vts_report_point_down(data->vtsdev, events[i].id, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, NULL, 0, kt);

			if (FTS_TOUCH_DOWN == events[i].flag) {
				FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
					  events[i].id,
					  events[i].x, events[i].y,
					  events[i].p, events[i].area);
			}
		} else {
			uppoint++;
			data->touchs &= ~BIT(events[i].id);
			vts_report_point_up(data->vtsdev, events[i].id, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, kt);
			FTS_DEBUG("[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < max_touch_num; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				FTS_DEBUG("[B]P%d UP!", i);
				va_reported = true;
				vts_report_point_up(data->vtsdev, i, get_touch_count(touchs), events[i].x,events[i].y,events[i].area,events[i].area, false, kt);
			}
		}
	}
	data->touchs = touchs;

	if (va_reported) {
		/* touchs==0, there's no point but key */
		if (EVENT_NO_DOWN(data) || (!touchs)) {
			FTS_DEBUG("[B]Points All Up!");
			vts_report_release(data->vtsdev);
		} else {
			//input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	vts_report_point_sync(data->vtsdev);
	return 0;
}
#endif


/*注意手指按在FOD区域，会不间断上报down事件*/
void fts_read_fod_info(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 cmd = FTS_VIVO_FOD_INFO_REG;
	u8 val[FTS_FOD_INFO_LEN] = { 0 };

	ret = ft3518u_fts_read(&cmd, 1, val, FTS_FOD_INFO_LEN);
	if (ret < 0) {
		FTS_INFO("%s:read FOD info fail", __func__);
		return ;
	}

	FTS_DEBUG("FOD info buffer:%x %x %x %x %x %x %x %x %x", val[0],
		val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
	ts_data->fod_info.fp_id = val[0];
	ts_data->fod_info.event_type = val[1];
	ts_data->fod_info.fp_down = !val[8];

	ts_data->fod_info.fp_area_rate = val[2];
	ts_data->fod_info.fp_x = (val[4] << 8) + val[5];
	ts_data->fod_info.fp_y = (val[6] << 8) + val[7];
}

void fts_report_fod(struct fts_ts_data *ts_data)
{

	//if (vts_get_run_mode(ts_data->vtsdev) == VTS_ST_NORMAL &&
	//	!vts_state_get(ts_data->vtsdev, VTS_STA_FACE_HIGHLIGHT) &&
	//	!vts_state_get(ts_data->vtsdev, VTS_STA_FINGER_HIGHLIGHT))
	//	return;

	/*注意手指按在FOD区域，会不间断上报down事件,如果系统只需要一帧down和up，驱动需要过滤*/
	fts_read_fod_info(ts_data);
	if ((ts_data->fod_info.event_type == 0x26) && (ts_data->fod_info.fp_down)) {
		vts_report_event_down(ts_data->vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
	} else if ((ts_data->fod_info.event_type == 0x26) && (!ts_data->fod_info.fp_down)) {
		vts_report_event_up(ts_data->vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
	}
}

static int fts_read_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	u8 *buf = data->point_buf;

	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x01;

	if (data->gesture_mode) {
		if (0 == ft3518u_fts_gesture_readdata(data, NULL)) {
			FTS_INFO("succuss to get gesture data in irq handler");
			return 1;
		}
	}

	//fts_report_fod(data);

	ret = ft3518u_fts_read(buf, 1, buf + 1, data->pnt_buf_size - 1);
	if (ret < 0) {
		FTS_ERROR("read touchdata failed, ret:%d", ret);
		return ret;
	}

	VTD_BUF("touchdata", buf, data->pnt_buf_size);

	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
	int ret = 0;
	int i = 0;
	u8 pointid = 0;
	int base = 0;
	static u8 large_press = 0;
	struct ts_event *events = data->events;
	int max_touch_num = data->pdata->max_touch_number;
	u8 *buf = data->point_buf;

	ret = fts_read_touchdata(data);
	if (ret) {
		return ret;
	}

	data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	data->touch_point = 0;

	if ((data->point_num == 0x0F) && (buf[2] == 0xFF) && (buf[3] == 0xFF)
		&& (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
		VTI("touch buff is 0xff, need recovery state");
		vts_report_release(data->vtsdev);
		ft3518u_fts_tp_state_recovery(data);
		vts_reset(data->vtsdev);
		data->point_num = 0;
		return -EIO;
	}


	if ((buf[1] >> 6 & 0x03) == 0x02) {
		vts_proxminity_report(data->vtsdev, 0, 0, 0);
	} else	if ((buf[1] >> 6 & 0x03) == 0x01) {
		vts_proxminity_report(data->vtsdev, 1, 0, 0);
	}


	if (buf[1] & 0x01) {//
		large_press = 1;
		vts_report_event_down(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
		vts_report_release(data->vtsdev);
	} else if (1 == large_press) {
		large_press = 0;
		vts_report_event_up(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
	}

	//if ((buf[1] >> 5 & 0x01))
		fts_report_fod(data);

	if (data->point_num > max_touch_num) {
		FTS_INFO("invalid point_num(%d)", data->point_num);
		data->point_num = 0;
		return -EIO;
	}

	for (i = 0; i < max_touch_num; i++) {
		base = FTS_ONE_TCH_LEN * i;
		pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else if (pointid >= max_touch_num) {
			FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
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
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
			FTS_INFO("abnormal touch data from fw");
			return -EIO;
		}
	}

	if (data->touch_point == 0) {
		FTS_INFO("no touch point information(%02x)", buf[2]);
		return -EIO;
	}

	return 0;
}

static void fts_irq_read_report(ktime_t kt)
{
	int ret = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;

#if FTS_ESDCHECK_EN
	ft3518u_fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	ft3518u_fts_prc_queue_work(ts_data);
#endif

	ret = fts_read_parse_touchdata(ts_data);
	if (ret == 0) {
		mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
		fts_input_report_b(ts_data, kt);
#endif
		mutex_unlock(&ts_data->report_mutex);
	}
#if FTS_ESDCHECK_EN
	ft3518u_fts_esdcheck_set_intr(0);
#endif
}

static irqreturn_t fts_irq_handler(int irq, void *data, ktime_t kt)
{
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	int ret = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;

	if ((ts_data->suspended) && (ts_data->pm_suspend)) {
		ret = wait_for_completion_timeout(&ts_data->pm_completion,
						  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
		if (!ret) {
			FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
			return IRQ_HANDLED;
		}
	}
#endif

	fts_irq_read_report(kt);
	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	VTI("irq in ts_data:%d irq in client:%d", ts_data->irq, ts_data->spi->irq);
	if (ts_data->irq != ts_data->spi->irq)
		VTE("IRQs are inconsistent, please check <interrupts> & <focaltech,irq-gpio> in DTS");
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	VTI("irq flag:%x", pdata->irq_gpio_flags);

	ret = vts_interrupt_register(ts_data->vtsdev, ts_data->irq, fts_irq_handler,
					pdata->irq_gpio_flags, ts_data);
	if (ret) {
			VTE("Request irq failed!");
	} else {
			VTI("request irq %d succeed", ts_data->irq);
	}
	return ret;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
	int point_num = 0;
	int events_num = 0;

	point_num = ts_data->pdata->max_touch_number;
	ts_data->pnt_buf_size = point_num * FTS_ONE_TCH_LEN + 3;

	ts_data->point_buf = (u8 *) kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
	if (!ts_data->point_buf) {
		FTS_ERROR("failed to alloc memory for point buf");
		return -ENOMEM;
	}

	events_num = point_num * sizeof(struct ts_event);
	ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
	if (!ts_data->events) {
		FTS_ERROR("failed to alloc memory for point events");
		kfree_safe(ts_data->point_buf);
		return -ENOMEM;
	}

	return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl) {
		devm_pinctrl_put(ts->pinctrl);
	}
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			FTS_ERROR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0) {
			FTS_ERROR("Set suspend pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d", ret);
		}
	}

	return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ts_data->vdd)) {
		FTS_ERROR("vdd is invalid");
		return -EINVAL;
	}

	FTS_FUNC_ENTER();
	if (enable) {
		if (ts_data->power_disabled) {
			FTS_DEBUG("regulator enable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			ret = regulator_enable(ts_data->vdd);
			if (ret) {
				FTS_ERROR("enable vdd regulator failed,ret=%d", ret);
			}

			ts_data->power_disabled = false;
		}
	} else {
		if (!ts_data->power_disabled) {
			FTS_DEBUG("regulator disable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);
			ret = regulator_disable(ts_data->vdd);
			if (ret) {
				FTS_ERROR("disable vdd regulator failed,ret=%d", ret);
			}
			msleep(10); /* Power-off to power-on need delay */
			ts_data->power_disabled = true;
		}
	}

	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ts_data->vdd = regulator_get(ts_data->dev, "vdd");
	if (IS_ERR_OR_NULL(ts_data->vdd)) {
		ret = PTR_ERR(ts_data->vdd);
		FTS_ERROR("get vdd regulator failed,ret=%d", ret);
		return ret;
	}

	if (regulator_count_voltages(ts_data->vdd) > 0) {
		ret = regulator_set_voltage(ts_data->vdd, FTS_VTG_MIN_UV,
						FTS_VTG_MAX_UV);
		if (ret) {
			FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
			regulator_put(ts_data->vdd);
			return ret;
		}
	}

#if FTS_PINCTRL_EN
	fts_pinctrl_init(ts_data);
	fts_pinctrl_select_normal(ts_data);
#endif

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret) {
		FTS_ERROR("fail to enable power(regulator)");
	}

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif

	fts_power_source_ctrl(ts_data, DISABLE);

	if (!IS_ERR_OR_NULL(ts_data->vdd)) {
		if (regulator_count_voltages(ts_data->vdd) > 0)
			regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
		regulator_put(ts_data->vdd);
	}

	return 0;
}

int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
	int ret = 0;

#if FTS_PINCTRL_EN
	fts_pinctrl_select_suspend(ts_data);
#endif

	ret = fts_power_source_ctrl(ts_data, DISABLE);
	if (ret < 0) {
		FTS_ERROR("power off fail, ret=%d", ret);
	}

	return ret;
}

int fts_power_source_resume(struct fts_ts_data *ts_data)
{
	int ret = 0;

#if FTS_PINCTRL_EN
	fts_pinctrl_select_normal(ts_data);
#endif

	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret < 0) {
		FTS_ERROR("power on fail, ret=%d", ret);
	}

	return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->pdata->irq_gpio);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
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

static int fts_parse_dt(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct device_node *np = ts_data->dev->of_node;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	u32 temp_val = 0;

	FTS_FUNC_ENTER();

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
							0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		FTS_ERROR("Unable to get reset_gpio");

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
						  0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		FTS_ERROR("Unable to get irq_gpio");

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get max-touch-number, please check dts");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		if (temp_val < 2)
			pdata->max_touch_number = 2;	/* max_touch_number must >= 2 */
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

	/* spi info */
	ret = of_property_read_u32(np, "focaltech,spi-max-frequency",	&ts_data->spi->max_speed_hz);
	if (ret < 0) {
		FTS_ERROR("Unable to get spi-max-frequency, please check dts");
		ts_data->spi->max_speed_hz = 6000000;
	}

	ret = of_property_read_u32(np, "focaltech,spi-mode", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get spi-mode, please check dts");
		ts_data->spi->mode = SPI_MODE_0;
	} else {
		ts_data->spi->mode = temp_val;
	}

	ret = of_property_read_u32(np, "focaltech,spi-bits", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get spi-bits, please check dts");
		ts_data->spi->bits_per_word = 8;
	} else {
		ts_data->spi->bits_per_word = temp_val;
	}

	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d, spi-max-frequency:%d, spi-mode:%d, spi-bits:%d",
		pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio, 
		ts_data->spi->max_speed_hz, ts_data->spi->mode, ts_data->spi->bits_per_word);

	FTS_FUNC_EXIT();
	return 0;
}

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int pdata_size = sizeof(struct fts_ts_platform_data);

	FTS_FUNC_ENTER();
	FTS_INFO("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
	if (!ts_data->pdata) {
		FTS_ERROR("allocate memory for platform_data fail");
		return -ENOMEM;
	}

	if (ts_data->dev->of_node) {
		ret = fts_parse_dt(ts_data);
		if (ret)
			FTS_ERROR("device-tree parse fail");
	} 

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue) {
		FTS_ERROR("create fts workqueue fail");
	}

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);

	/* Init communication interface */
	ret = ft3518u_fts_bus_init(ts_data);
	if (ret) {
		FTS_ERROR("bus initialize fail");
		goto err_bus_init;
	}

	ret = fts_report_buffer_init(ts_data);
	if (ret) {
		FTS_ERROR("report buffer init fail");
		goto err_report_buffer;
	}

	ret = ft3518u_fts_create_apk_debug_channel(ts_data);
	if (ret) {
		FTS_ERROR("create apk debug node fail");
	}

	ret = ft3518u_fts_create_sysfs(ts_data);
	if (ret) {
		FTS_ERROR("create sysfs node fail");
	}
#if FTS_POINT_REPORT_CHECK_EN
	ret = ft3518u_fts_point_report_check_init(ts_data);
	if (ret) {
		FTS_ERROR("init point report check fail");
	}
#endif

	ret = ft3518u_fts_ex_mode_init(ts_data);
	if (ret) {
		FTS_ERROR("init glove/cover/charger fail");
	}

	ret = ft3518u_fts_gesture_init(ts_data);
	if (ret) {
		FTS_ERROR("init gesture fail");
	}

#if FTS_ESDCHECK_EN
	ret = ft3518u_fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
	}
#endif

	ret = ft3518u_fts_fwupg_init(ts_data);
	if (ret) {
		FTS_ERROR("init fw upgrade fail");
	}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	init_completion(&ts_data->pm_completion);
	ts_data->pm_suspend = false;
#endif

	FTS_FUNC_EXIT();
	return 0;

err_report_buffer:

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
	kfree_safe(ts_data->pdata);

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

#if FTS_POINT_REPORT_CHECK_EN
	ft3518u_fts_point_report_check_exit(ts_data);
#endif

	ft3518u_fts_release_apk_debug_channel(ts_data);
	ft3518u_fts_remove_sysfs(ts_data);
	ft3518u_fts_ex_mode_exit(ts_data);

	ft3518u_fts_fwupg_exit(ts_data);

#if FTS_ESDCHECK_EN
	ft3518u_fts_esdcheck_exit(ts_data);
#endif

	ft3518u_fts_gesture_exit(ts_data);
	ft3518u_fts_bus_exit(ts_data);

	free_irq(ts_data->irq, ts_data);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
#endif

	kfree_safe(ts_data->point_buf);
	kfree_safe(ts_data->events);

	kfree_safe(ts_data->pdata);
	kfree_safe(ts_data);

	FTS_FUNC_EXIT();

	return 0;
}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	FTS_INFO("system enters into pm_suspend");
	ts_data->pm_suspend = true;
	reinit_completion(&ts_data->pm_completion);
	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	FTS_INFO("system resumes from pm_suspend");
	ts_data->pm_suspend = false;
	complete(&ts_data->pm_completion);
	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

static int ft3518u_init(struct vts_device *vtsdev)
{
	int ret = 0;
	// struct fts_ts_data *ts_data = container_of(vtsdev, struct fts_ts_data, vtsdev);
	struct fts_ts_data *ts_data = ft3518u_fts_data;

	ret = fts_gpio_configure(ts_data);
	if (ret) {
		FTS_ERROR("configure the gpios fail");
		goto err_gpio_config;
	}
#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_init(ts_data);
	if (ret) {
		FTS_ERROR("fail to get power(regulator)");
		goto err_power_init;
	}
#endif

#if (!FTS_CHIP_IDC)
	ft3518u_fts_reset_proc(200);
#endif

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		FTS_ERROR("not focal IC, unregister driver");
		goto err_power_init;
	}

#if FTS_ESDCHECK_EN
	ret = ft3518u_fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
	}
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_irq_req;
	}

	return 0;


err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
	vts_unregister_driver(vtsdev);
	vts_device_free(vtsdev);
	fts_power_source_exit(ts_data);
#endif
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:

	return ret;
}

static int ft3518u_exit(struct vts_device *vtsdev)
{
	return 0;
}

static int ft3518u_update_firmware(struct vts_device *vtsdev, const struct firmware *firmware)
{
	int ret = 0;
	FTS_FUNC_ENTER();

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (DISABLE);
#endif
	ret = ft3518u_fts_fw_download(firmware->data, firmware->size, 0);
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (ENABLE);
#endif
	FTS_FUNC_EXIT();
	return ret;
}

static int ft3518u_get_fw_version(struct vts_device *vtsdev, u64 *version)
{
	unsigned char ver = 0;
	int ret = 0;
	FTS_FUNC_ENTER();
	ret = ft3518u_fts_read_reg(FTS_REG_FW_VER, &ver);
	if (ret < 0) {
		VTE("read fw version fail.");
	}

	*version = (u64)ver;
	FTS_FUNC_EXIT();
	return 0;
}

static int ft3518u_set_auto_idle(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	u8 idle_state = (u8)(state ? 1 : 0);
	u32 idle_time = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);
	VTI("write idle state: %d, idle time: %d ", idle_state, idle_time);

	if (idle_time != 0) {
		VTI("new idle addr");
		if (idle_state == 1) {//idle_state 1-normal 0-game
			if (ft3518u_write_then_read(0xB8, 0) 
			|| ft3518u_write_then_read(0x88, 0x0C)){
				VTE("write idle state normal error");
				ret = -1;
			}
		} else {
			if (ft3518u_write_then_read(0xB9, idle_time) 
			|| ft3518u_write_then_read(0xB8, 1) 
			|| ft3518u_write_then_read(0x88, 0x12)){
				VTE("write idle state game error");
				ret = -1;
			}
		}
	} else {
		VTI("old idle addr");
		ret = ft3518u_write_then_read(0x86, idle_state);
	}

	return ret;
}

static int ft3518u_set_face_detect(struct vts_device *vtsdev, int cmd) {
	// 接近感应的调试，0xB0是开关，B0写1是打开接近感应，写0是关闭接近感应。
	// 打开接近感应之后，接近，有int，读0x1寄存器，用bit7和bit6来表示接近和离开，bit7为1，bit6为0是接近，bit7为0，bit6为1是离开
	u8 feedback;
	int i = 0;
	for (i = 0; i < 3; i++) {
		ft3518u_fts_write_reg(0xB0, (u8)cmd);
		ft3518u_fts_read_reg(0xB0, &feedback);
		if(cmd == feedback){
			VTI("write  success, cmd =:%02x",cmd);
			return 0;
		}
	}
	VTI("ft3518u_set_face_detect failed");
	return -1;
}


static int ft3518u_rom_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 15;
	return 0;
}

static ssize_t ft3518u_rom_read(struct vts_device *vtsdev, u8*buf, size_t nbytes)
{
	int ret = 0;
	u8 data[12] = { 0 };
	u8 cmd = 0x60;
	u8 mode = 0;
	u8 data_len = 12;

	ret = ft3518u_fts_write_reg(0x00,0x40);
	msleep(200);

	ret = ft3518u_fts_read_reg(0x00, &mode);
	if ((ret >=0 ) && (0x40 == mode)) {
		VTI("enter factory mode success");
	} else {
		VTE("enter factory mode failed");
		goto error_end;
	}

	ret = ft3518u_fts_write_reg(cmd,0x00);
	msleep(10);
	ret = ft3518u_fts_read(&cmd, 1, data, data_len);

	data[0] = data[0] & 0x07;
	data[1] = data[1] & 0x0F;
	data[2] = data[2] & 0x1F;
	data[4] = data[4] & 0x0F;
	data[5] = data[5] & 0x1F;

	VTI("%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
		data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]);

	memset(buf, 0, nbytes);
	memcpy(buf, data, nbytes > data_len ? data_len : nbytes);

	ret = ft3518u_fts_write_reg(0x00, 0x00);
	msleep(200);

	ret = ft3518u_fts_read_reg(0x00, &mode);
	if ((ret >=0 ) && (0x00 == mode)) {
		VTI("enter work mode success");
	} else {
		VTE("enter work mode failed");
		goto error_end;
	}

	return nbytes;

error_end:
	ft3518u_fts_write_reg(0x00,0x00);
	msleep(200);
	return nbytes;
}

/* 0xD0 bit0 -> 手势功能总开关
 * 0xD0 bit1 -> 熄屏触摸区域使能
 * 0xD3  连续写入： 时钟报点区域(8bytes，高字节在前)
 */
static int ft3518u_set_screen_clock_area(struct vts_device *vtsdev, int state)
{
	int ret;
	u8 buf[9] = { 0 };
	u8 cmd = 0;
	struct vts_screen_clock_cmd  clock_area;
	vts_get_screen_clock_zone(&clock_area, &vtsdev->screen_clock_zone);
	VTI("state = %d, clock_area.x = %d, clock_area.y = %d, clock_area.width = %d, clock_area.height = %d",
		state, clock_area.x, clock_area.y, clock_area.width, clock_area.height);
	ft3518u_fts_read_reg(FTS_REG_GESTURE_EN, &cmd);

	if(0 != clock_area.width && 0 != clock_area.height) {
		cmd |= 0x03;
		buf[0] = FTS_REG_GESTURE_OUTPUT_ADDRESS; // addr
		buf[1] = clock_area.x >> 8 & 0xFF;
		buf[2] = clock_area.x & 0xFF;
		buf[3] = clock_area.y >> 8 & 0xFF;
		buf[4] = clock_area.y & 0xFF;
		buf[5] = clock_area.width >> 8 & 0xFF;
		buf[6] = clock_area.width & 0xFF;
		buf[7] = clock_area.height >> 8 & 0xFF;
		buf[8] = clock_area.height & 0xFF;
		ret = ft3518u_fts_write(buf, 9);
		VTI("return %d, buf = %2x %2x %2x %2x %2x %2x %2x %2x %2x",
			ret, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8]);
	} else {
		cmd &= ~0x02;
	}

	ret = ft3518u_fts_write_reg(FTS_REG_GESTURE_EN, cmd);

	return ret;
}

static int ft3518u_set_screen_clock_report_abs(struct vts_device *vtsdev,int report_enable)
{
	VTI("report_enable = %#x", report_enable);
	return ft3518u_set_screen_clock_area(vtsdev, report_enable);
}

static int set_bit_zero(u8 *psrc ,u8 bit_num)
{
	*psrc &= (~(0x01 << bit_num));
	return 0;
}

static int ft3518u_set_gesture(struct vts_device *vtsdev,int enable)
{
	#if 1
	u8 i = 0;
	int geture_type = vts_state_get(vtsdev,VTS_STA_GESTURE);
	//int gesture_bit_map = 0xFFFF;
	u8 gest_switch = 1;//0xd0
	u8 char_switch = 0;//0xd2 bit0-bit4 : O W M E C
	u8 LR_switch = 0;//0xd1 : left right up down doubleclick
	u8 at_switch = 1;//0xd6 : @  0x01-on 0x00-off
	u8 f_switch= 0x10;//0xd8 : F 0x10-on 0x00-off
	u8 feedback = 0;
	VTI("SET FTS GETUREBIT MAP 1");
	VTI("geture_type:%04X",geture_type);
	ft3518u_fts_read_reg(0xd2,&char_switch);
	ft3518u_fts_read_reg(0xd1,&LR_switch);
	//focal firmware  default off, should on in driver
	char_switch |= 0x1f;
	LR_switch |= 0x1f;
	#if 0
	for(i = 0; i < 12; i++){
		if(0 == ((0x01 << i) & geture_type)){
		set_bit_zero(&gesture_bit_map,fts_ges_map[i]);
		}
	}
	#endif
	#if 1
	VTI("SET FTS GETUREBIT MAP 2");

	if(0 == (geture_type & VTS_GESTURE_C)){
		set_bit_zero(&char_switch,4);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_E)){
		set_bit_zero(&char_switch,3);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_M)){
		set_bit_zero(&char_switch,2);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_W)){
		set_bit_zero(&char_switch,1);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_O)){
		set_bit_zero(&char_switch,0);
		i++;
	}
	//
	if(0 == (geture_type & VTS_GESTURE_DCLICK)){
		set_bit_zero(&LR_switch,4);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_DOWN)){
		set_bit_zero(&LR_switch,3);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_UP)){
		set_bit_zero(&LR_switch,2);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_LR)){
		set_bit_zero(&LR_switch,1);
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_LR)){
		set_bit_zero(&LR_switch,0);
		i++;
	}
	//
	if(0 == (geture_type & VTS_GESTURE_A)){
		at_switch = 0;
		i++;
	}
	if(0 == (geture_type & VTS_GESTURE_F)){
		f_switch = 0;
		i++;
	}
	#endif
	VTI("SET FTS GETUREBIT MAP 3");
	VTI("I== %d",i);

	if(i){
		VTI("set fts switch");
		ft3518u_fts_write_reg(FTS_REG_GESTURE_EN, gest_switch);
		ft3518u_fts_read_reg(FTS_REG_GESTURE_EN,& feedback);
		if(gest_switch == feedback)
			VTI("write FTS_REG_GESTURE_EN success :%02x", gest_switch);
		ft3518u_fts_write_reg(0xd1, LR_switch);
		ft3518u_fts_read_reg(0xd1, &feedback);
		if(LR_switch == feedback)
			VTI("write 0xd1 success :%02x",LR_switch);
		ft3518u_fts_write_reg(0xd2,char_switch);
		ft3518u_fts_read_reg(0xd2,&feedback);
		if(char_switch == feedback)
			VTI("write 0xd2 success :%02x",char_switch);
		ft3518u_fts_write_reg(0xd6,at_switch);
		ft3518u_fts_read_reg(0xd6,&feedback);
		if(at_switch == feedback)
			VTI("write 0xd6 success :%02x",at_switch);
		ft3518u_fts_write_reg(0xd8,f_switch);
		ft3518u_fts_read_reg(0xd8,&feedback);
		if(f_switch == feedback)
			VTI("write 0xd8 success :%02x",f_switch);

	}

	#endif
	return 0;
}

static int fts_ts_suspend(bool power_en)
{
	int ret = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;

	FTS_FUNC_ENTER();
	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif

	if (ts_data->gesture_mode) {
		ft3518u_fts_gesture_suspend(ts_data);
	} else {
		ft3518u_fts_irq_disable();

		FTS_INFO("make TP enter into sleep mode");
		ret = ft3518u_fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
		if (ret < 0)
			FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

		if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
			/*关闭power*/
			//if (!power_en)
			//	ret = fts_power_source_suspend(ts_data);

			if (ret < 0)
				FTS_ERROR("power enter suspend fail");
#endif
		}
	}

	vts_report_release(ts_data->vtsdev);
	ts_data->suspended = true;
	FTS_FUNC_EXIT();
	return 0;
}

static int fts_ts_resume(bool power_en)
{
	struct fts_ts_data *ts_data = ft3518u_fts_data;

	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_DEBUG("Already in awake state");
		return 0;
	}

	vts_report_release(ts_data->vtsdev);

#if FTS_POWER_SOURCE_CUST_EN
	/*开启power*/
	//if (power_en)
	//    fts_power_source_resume(ts_data);
#endif
	ft3518u_fts_reset_proc(70);

	ft3518u_fts_wait_tp_to_valid();
	ft3518u_fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

	if (ts_data->gesture_mode) {
		ft3518u_fts_gesture_resume(ts_data);
	} else {
		ft3518u_fts_irq_enable();
	}

	ts_data->suspended = false;
	FTS_FUNC_EXIT();
	return 0;
}


#define BYTES_PER_TIME          128
static int read_mass_data(u8 addr, int byte_num, short *buf)
{
	int ret = 0;
	int i = 0;
	int packet_length = 0;
	int packet_num = 0;
	int packet_remainder = 0;
	int offset = 0;
	u8 *data = NULL;

	data = (u8 *)kzalloc(byte_num * sizeof(u8), GFP_KERNEL);
	if (NULL == data) {
		FTS_ERROR("mass data buffer malloc fail\n");
		return -ENOMEM;
	}

	/* read rawdata buffer */
	FTS_INFO("mass data len:%d", byte_num);
	packet_num = byte_num / BYTES_PER_TIME;
	packet_remainder = byte_num % BYTES_PER_TIME;
	if (packet_remainder)
		packet_num++;

	if (byte_num < BYTES_PER_TIME) {
		packet_length = byte_num;
	} else {
		packet_length = BYTES_PER_TIME;
	}

	ret = ft3518u_fts_read(&addr, 1, &data[offset], packet_length);
	if (ret < 0) {
		FTS_ERROR("read buffer fail");
		goto read_massdata_err;
	}

	for (i = 1; i < packet_num; i++) {
		offset += packet_length;
		if ((i == (packet_num - 1)) && packet_remainder) {
			packet_length = packet_remainder;
		}

		ret = ft3518u_fts_read(NULL, 0, &data[offset], packet_length);
		if (ret < 0) {
			FTS_ERROR("read buffer fail");
			goto read_massdata_err;
		}
	}

	for (i = 0; i < byte_num; i = i + 2) {
		buf[i >> 1] = (short)((data[i] << 8) + data[i + 1]);
	}

	ret = 0;
read_massdata_err:
	kfree(data);
	return ret;
}

static int fts_get_rawordiff_data(u8 is_diff, short *data)
{
	int ret = 0;
	int i = 0;
	u8 state = 0xFF;
	u8 tx = 0;
	u8 rx = 0;

	/*enter factory mode*/
	ret = ft3518u_fts_write_reg(0x00, 0x40);
	if (ret < 0) {
		FTS_ERROR("write 0x40 to reg0x00 fails");
		return ret;
	}

	for (i = 0; i < 20; i++) {
		msleep(20);
		ft3518u_fts_read_reg(0x00, &state);
		if (state == 0x40) {
			FTS_INFO("enter factory mode successfully");
			msleep(200);
			break;
		} else
			FTS_DEBUG("reg%x=%x,retry:%d", 0x00, state, i);
	}

	if (i >= 20) {
		FTS_ERROR("enter factory mode fails(timeout)");
		goto enter_work_mode;
	}

	ret = ft3518u_fts_read_reg(0x02, &tx);
	if (ret < 0) {
		FTS_ERROR("read tx fails");
		goto enter_work_mode;
	}
	ret = ft3518u_fts_read_reg(0x03, &rx);
	if (ret < 0) {
		FTS_ERROR("read rx fails");
		goto enter_work_mode;
	}
	FTS_INFO("tx:%d, rx:%d", tx, rx);

	/*choose rawdata or diff*/
	if (is_diff) {
		ret = ft3518u_fts_write_reg(0x06, 0x01);
		if (ret < 0) {
			FTS_ERROR("write 0x01 to reg0x06 fails");
			goto enter_work_mode;
		}
	}

	/*start scan*/
	ret = ft3518u_fts_write_reg(0x00, 0xC0);
	if (ret < 0) {
		FTS_ERROR("write 0xC0 to reg0x00 fails");
		goto enter_work_mode;
	}

	for (i = 0; i < 50; i++) {
		msleep(20);
		ft3518u_fts_read_reg(0x00, &state);
		if (state == 0x40) {
			break;
		} else
			FTS_DEBUG("reg%x=%x,retry:%d", 0x00, state, i);
	}

	if (i >= 50) {
		FTS_ERROR("scan frame fails(timeout)");
		goto enter_work_mode;
	}

	/*read raw data*/
	ret = ft3518u_fts_write_reg(0x01, 0xAD);
	if (ret < 0) {
		FTS_ERROR("write 0xAD to reg0x01 fails");
		goto enter_work_mode;
	}

	ret = read_mass_data(0x36, tx * rx * 2, data);
	if (ret < 0) {
		FTS_ERROR("read rawdata fail\n");
		goto enter_work_mode;
	}

	ret = 0;
enter_work_mode:
	ft3518u_fts_write_reg(0x06, 0x00);
	ret = ft3518u_fts_write_reg(0x00, 0x00);
	if (ret < 0) {
		FTS_ERROR("write 0x00 to reg0x00 fails");
	}

	for (i = 0; i < 20; i++) {
		msleep(20);
		ft3518u_fts_read_reg(0x00, &state);
		if (state == 0x00) {
			FTS_INFO("enter work mode successfully");
			break;
		}
	}

	return ret;
}


/*1. rawdata数据读取和diff/delta数据读取接口以及baseline读取接口
如果为1表示要读取diff/delta数据.如果为0表示要读取rawdata数据,2表示要读取baseline数据*/
/*验证ok:固件只有rawdata和delta值*/
static int ft3518u_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size)
{
	int ret = 0;

	ret = ft3518u_fts_write_reg(0xEE, 0x01);
	if (ret < 0) {
		FTS_ERROR("write 0x01 to reg0xEE fails");
	}

	ret = fts_get_rawordiff_data(which, data);
	if (ret < 0) {
		FTS_ERROR("read raw or diff data fails");
	}

	ret = ft3518u_fts_write_reg(0xEE, 0x00);
	if (ret < 0) {
		FTS_ERROR("write 0x00 to reg0xEE fails");
	}

	return ret;
}

/*5. 正常工作模式/睡眠模式/手势模式切换接口---验证ok，建议参考IDC的代码，这一块差不多(除了屏下指纹)*/
static int ft3518u_change_mode(struct vts_device *vtsdev, int which)
{
	static int last_mode = 0;

	FTS_INFO("switch mode to %d from %d", which, last_mode);
	switch (which) {
	case VTS_ST_NORMAL:
		fts_ts_resume(last_mode == VTS_ST_SLEEP);
		break;
	case VTS_ST_SLEEP:
		/**/
		if (last_mode == VTS_ST_GESTURE) {
			fts_ts_resume(false); /* don't poweroff */
		}
		ft3518u_fts_data->gesture_mode = 0;
		fts_ts_suspend(false);
		break;
	case VTS_ST_GESTURE:
		/*注意:从sleep回到手势，需要resume(上电拉tprst)后再执行suspend*/
		if (last_mode == VTS_ST_SLEEP) {
			fts_ts_resume(true);
		}
		ft3518u_fts_data->gesture_mode = 1;
		fts_ts_suspend(true);
		break;
	}
	last_mode = which;
	return 0;
}

/*8. 充电状态写入到芯片接口  ----验证ok*/
static int ft3518u_set_charging(struct vts_device *vtsdev, int state)
{
	FTS_INFO("set charge to %s", !!state ? "ENABLE" : "DISABLE");
	return ft3518u_fts_write_reg(FTS_REG_CHARGER_MODE_EN, !!state);
}

/*9.No use*/

/*10.边缘抑制开关 --- 验证ok*/
static int ft3518u_set_rotation(struct vts_device *vtsdev, int on)
{
	/*1:竖屏状态 0:横屏状态(左上右下) 2:横屏状态(左下右上)*/
	int ret = 0;
	u8 value = 0;

	if (on == 1)
		value = 0;
	else if (on == 0)
		value = 1;
	else if (on == 2)
		value = 2;
	FTS_INFO("edge switch, on:%d, fw_value:%d", on, value);
	ret = ft3518u_write_then_read(FTS_VIVO_EDGE_CTL_REG, value);
	if (ret < 0)
		FTS_ERROR("write edge reg %d to %d fail", FTS_VIVO_EDGE_CTL_REG, value);

	return ret;
}

/*12.fw debug信息输出接口 --- 验证ok*/
// static int vivo_get_fw_debug_info(unsigned char *buf)
// {
// 	int cnt = 0;
// 	u8 state = 0;

// 	ft3518u_fts_read_reg(FTS_VIVO_WP_REG, &state);
// 	cnt += snprintf(buf + cnt, 256, "Water Reject Mode:%d\n", !!(state & 0x01));
// 	ft3518u_fts_read_reg(FTS_VIVO_ACTIVE_REG, &state);
// 	cnt += snprintf(buf + cnt, 256, "Active Mode:%d\n", (state == 0));
// 	ft3518u_fts_read_reg(FTS_REG_CHARGER_MODE_EN, &state);
// 	cnt += snprintf(buf + cnt, 256, "Charge Mode:%d\n", state);
// 	ft3518u_fts_read_reg(FTS_VIVO_PALM_REG, &state);
// 	cnt += snprintf(buf + cnt, 256, "Palm Mode:%d\n", !!(state & 0x02));
// 	ft3518u_fts_read_reg(FTS_REG_GESTURE_EN, &state);
// 	cnt += snprintf(buf + cnt, 256, "Gesture Mode:%d\n", state);
// 	return cnt;
// }

/*15.返回值：
0：表示当前处于normal模式
1：表示当前处于sleep模式
2：表示当前处于gesture模式*/
static int ft3518u_get_ic_work_mode(struct vts_device *vtsdev)
{
	int ret = 0;
	u8 status = 0xFF;

	ret = ft3518u_fts_read_reg((u8)FTS_REG_GESTURE_EN, &status);
	if (ret < 0) {
		FTS_ERROR("read gesture reg fail");
		return 1;/*通信不通，表示进入sleep*/
	}

	FTS_INFO("work mode:%d", status);
	if (status == 1)
		return 2;
	else if (status == 0)
		return 0;
	return -1;
}

static int ft3518u_set_long_press(struct vts_device *vtsdev, int enable)
{
	int ret = 0;
	u8 value = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;

	FTS_INFO("set fod to %d", enable);
	ret = ft3518u_fts_read_reg(FTS_VIVO_FOD_EN_REG, &value);
	if (ret < 0) {
		FTS_ERROR("read fod enable register fail");
		return ret;
	}

	FTS_INFO("fp_en=%d, fp_down=%d, event_type=%d", ts_data->fp_en,
		ts_data->fod_info.fp_down, ts_data->fod_info.event_type);
	if (enable) {
		value |= 0x02;
		ts_data->fp_en = 1;
	} else {
		value &= 0xFD;
		ts_data->fp_en = 0;
		ts_data->fod_info.fp_down = 0;
		ts_data->fod_info.event_type = 0;
	}

	FTS_INFO("write %x=%x.", FTS_VIVO_FOD_EN_REG, value);
	ret = ft3518u_fts_write_reg(FTS_VIVO_FOD_EN_REG, value);
	if (ret < 0) {
		FTS_ERROR("write FOD enable(%x=%x) fail", FTS_VIVO_FOD_EN_REG, value);
		return ret;
	}

	return 0;
}

static int ft3518u_set_finger_mode(struct vts_device *vtsdev, int mode)
{
	FTS_INFO("set fod mode to %d", mode);
	return ft3518u_fts_write_reg(FTS_VIVO_FOD_MODE_REG, !!mode);
}

/**vivo_ts end*/

static const struct vts_operations fts_vts_ops = {
	.init = ft3518u_init,
	.exit = ft3518u_exit,
	.get_frame = ft3518u_get_rawordiff_data,
	.get_fw_version = ft3518u_get_fw_version,
	.get_ic_mode = ft3518u_get_ic_work_mode,
	.set_rotation = ft3518u_set_rotation,
	.set_charging = ft3518u_set_charging,
	.set_auto_idle = ft3518u_set_auto_idle,

	.set_virtual_prox = ft3518u_set_face_detect,
	.set_long_press = ft3518u_set_long_press,
	.set_finger_mode = ft3518u_set_finger_mode,
	.set_gesture = ft3518u_set_gesture,

	.change_mode = ft3518u_change_mode,
	.update_firmware = ft3518u_update_firmware,

	.rom_size = ft3518u_rom_size,
	.rom_read = ft3518u_rom_read,

	.set_screen_clock_report_abs = ft3518u_set_screen_clock_report_abs,
	.set_screen_clock_area = ft3518u_set_screen_clock_area,
};

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_probe(struct spi_device *spi, struct device_node *np)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;
	struct vts_device *vtsdev = NULL;

	FTS_INFO("Touch Screen(SPI BUS) driver prboe...");
	/* malloc memory for global struct variable */
	ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("allocate memory for ft3518u_fts_data fail");
		return -ENOMEM;
	}

	ft3518u_fts_data = ts_data;
	ts_data->spi = spi;
	ts_data->dev = &spi->dev;
	ts_data->dev->of_node = np;

	spi_set_drvdata(spi, ts_data);
#ifdef CONFIG_SPI_MT65XX
	memcpy(&ts_data->spi_ctrl, &spi_ctrldata, sizeof(struct mtk_chip_config));
	ts_data->spi->controller_data = (void *)&ts_data->spi_ctrl;
#endif

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
		kfree_safe(ts_data);
		return ret;
	}

	ret = spi_setup(ts_data->spi);
	if (ret) {
		FTS_ERROR("spi setup fail");
		return ret;
	}

	vtsdev = vts_device_alloc();
	if (!vtsdev) {
		ret = -ENOMEM;
		goto errorcode9;
	}

	vtsdev->ops = &fts_vts_ops;
	vtsdev->busType = BUS_SPI;
	ts_data->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, np);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto errorcode10;
	}

	vts_set_drvdata(vtsdev, ts_data);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {
		VTE("vts_register_driver failed");
		goto errorcode10;
	}

	FTS_INFO("Touch Screen(SPI BUS) driver prboe successfully");

	return 0;
errorcode10:
	vts_device_free(vtsdev);
	FTS_INFO("errorcode10");

errorcode9:
	FTS_INFO("errorcode9");
	return ret;
}

static int fts_ts_remove(struct spi_device *spi, struct device_node *np)
{
	struct fts_ts_data *ts_data = spi_get_drvdata(spi);
	struct vts_device *vtsdev = ts_data->vtsdev;

	vts_unregister_driver(vtsdev);
	vts_device_free(vtsdev);

	return fts_ts_remove_entry(spi_get_drvdata(spi));
}

static void fts_shut_down(struct spi_device *spi)
{
	VTI("fts shut down");
}

static struct vts_spi_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.compatible = "focaltech,ft3518u_v2",
	.shutdown = fts_shut_down,
};

static const int ic_number[] = { VTS_IC_FT3518U };
module_vts_driver(ft3518u, ic_number, vts_spi_drv_reigster(&fts_ts_driver), vts_spi_drv_unreigster(&fts_ts_driver));
