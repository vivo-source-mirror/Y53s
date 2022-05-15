/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010 -2016, FocalTech Systems, Ltd., all rights reserved.
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
 * Created: 2016 -08 -08
 *
 * Abstract:
 *
 * Reference:
 *
 *****************************************************************************/

/*****************************************************************************
 * Included header files
 *****************************************************************************/
#include "focaltech_core.h"

#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"


#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early - suspend level */
#endif

/*****************************************************************************
 * Private constant and macro definitions using #define
 *****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      2600000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif
#define FTS_READ_TOUCH_BUFFER_DIVIDED       0
#define MINUS_ONE -1
/*****************************************************************************
 * Global variable or extern global variabls / functions
 ******************************************************************************/
struct i2c_client *fts_i2c_client;
struct fts_ts_data *fts_wq_data;
struct input_dev *fts_input_dev;
static int qup_i2c_suspended;

struct fts_global g_global = {
	.ic_type = FTS_CHIP_TYPE,
	.is_idc = 1,
	.chip_id = 0x00,
	.index = 0,
};

#if FTS_DEBUG_EN
int g_show_log = 1;
#else
int g_show_log = 0;
#endif

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = {0};
#endif

struct vivo_ts_struct *g_vts_data = NULL;


/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/
/*static void fts_release_all_finger(void); */
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);


/*****************************************************************************
 *  Name: fts_wait_tp_to_valid
 *  Brief:   Read chip id until TP FW become valid, need call when reset / power on / resume...
 *           1. Read Chip ID per INTERVAL_READ_REG(20ms)
 *           2. Timeout: TIMEOUT_READ_REG(300ms)
 *  Input:
 *  Output:
 *  Return: 0 - Get correct Device ID
 *****************************************************************************/
int fts_wait_tp_to_valid(struct i2c_client *client)
{
	int ret = 0;
	int cnt = 0;
	u8  reg_value = 0;

	VTI("fts_wait_tp_to_valid enter ");

	do {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
		if ((ret < 0) || (reg_value != chip_types.chip_idh)) {
			VTI("TP Not Ready, ReadData = 0x%x", reg_value);
		} else if (reg_value == chip_types.chip_idh) {
			VTI("TP Ready, Device ID = 0x%x", reg_value);
			return 0;
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	VTI("fts_wait_tp_to_valid  exit");

	/* error: not get correct reg data */
	return MINUS_ONE;
}

/*add by wu.lc*/
void vivo_tp_state_recovery(void)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();
	VTI("enter");
	if (vivoTsGetState() == TOUCHSCREEN_NORMAL) {
		/*free fingers and key*/
		vivoTsReleasePointsAndKeys();

		ret = vivoTsSetChargerFlagToChip(vtsData->usbChargerFlag);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetChargerFlagToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetChargerFlagToChip fail.");
		}
		ret = vivoTsSetEdgeRestainToChip(vtsData->edgeRestainSwitch);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetEdgeRestainToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetEdgeRestainToChip fail.");
		}
	}
}

/*****************************************************************************
 *  Name: fts_recover_state
 *  Brief: Need execute this function when reset
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_tp_state_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	fts_wait_tp_to_valid(client);
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(client);
	/* recover TP gesture state 0xD0 */
#if FTS_GESTURE_EN
	fts_gesture_recovery(client);
#endif
	vivo_tp_state_recovery();

	/*fts_release_all_finger(); */
}


/*****************************************************************************
 *  Name: fts_reset_proc
 *  Brief: Execute reset operation
 *  Input: hdelayms - delay time unit:ms
 *  Output:
 *  Return:
 *****************************************************************************/
#if FTS_ESDCHECK_EN
extern void set_flow_cnt(void);
#endif

#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern void incell_tp_reset_output(int status);
#else
static void incell_tp_reset_output(int status)
{
	return;
}
#endif

int fts_reset_proc(int hdelayms)
{
	VTI("====start=====");
	/*gpio_direction_output(fts_wq_data->pdata->reset_gpio, 0); */
	incell_tp_reset_output(0);
	msleep(20);
	incell_tp_reset_output(1);
	if (0 != hdelayms)
		msleep(hdelayms);
#if FTS_ESDCHECK_EN
	set_flow_cnt();
#endif
	return 0;
}

/*****************************************************************************
 *  Name: fts_irq_disable
 *  Brief: disable irq
 *  Input:
 *   sync:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_irq_disable(void)
{
	unsigned long irqflags;

	spin_lock_irqsave(&fts_wq_data->irq_lock, irqflags);

	VTD("====disable===%d==", fts_wq_data->irq_disable);

	if (!fts_wq_data->irq_disable) {
		/*lgj_0210 - disable_irq(fts_wq_data->client->irq); */
		disable_irq_nosync(fts_wq_data->client->irq); /* + lgj_0210 */
		fts_wq_data->irq_disable = 1;
	}

   spin_unlock_irqrestore(&fts_wq_data->irq_lock, irqflags);

}

/*****************************************************************************
 *  Name: fts_irq_enable
 *  Brief: enable irq
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	VTD("====enable===%d==", fts_wq_data->irq_disable);

	spin_lock_irqsave(&fts_wq_data->irq_lock, irqflags);


	if (fts_wq_data->irq_disable) {
		enable_irq(fts_wq_data->client->irq);
		fts_wq_data->irq_disable = 0;
	}


	spin_unlock_irqrestore(&fts_wq_data->irq_lock, irqflags);

}

/*****************************************************************************
 *  Name: fts_input_dev_init
 *  Brief: input dev init
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
#if 0
static int fts_input_dev_init(struct i2c_client *client, struct fts_ts_data *data,  struct input_dev *input_dev, struct fts_ts_platform_data *pdata)
{
	int  err, len;

	FTS_FUNC_ENTER();

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	if (data->pdata->have_key) {
		VTD("set key capabilities");
		for (len = 0; len < data->pdata->key_number; len++) {
			input_set_capability(input_dev, EV_KEY, data->pdata->keys[len]);
		}
	}
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if FTS_MT_PROTOCOL_B_EN
	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0f, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	err = input_register_device(input_dev);
	if (err) {
		VTE("Input device registration failed");
		goto free_inputdev;
	}

	FTS_FUNC_EXIT();

	return 0;

free_inputdev:
	input_free_device(input_dev);
	FTS_FUNC_EXIT();
	return err;

}
#endif
/*****************************************************************************
 * Power Control
 *****************************************************************************/
#if FTS_POWER_SOURCE_CUST_EN
static int fts_power_source_init(struct fts_ts_data *data)
{
	int rc;

	FTS_FUNC_ENTER();

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		VTE("Regulator get failed vdd rc=%d", rc);
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FTS_VTG_MIN_UV, FTS_VTG_MAX_UV);
		if (rc) {
			VTE("Regulator set_vtg failed vdd rc=%d", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		VTE("Regulator get failed vcc_i2c rc=%d", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MAX_UV);
		if (rc) {
			VTE("Regulator set_vtg failed vcc_i2c rc=%d", rc);
			goto reg_vcc_i2c_put;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FTS_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	FTS_FUNC_EXIT();
	return rc;
}

static int fts_power_source_ctrl(struct fts_ts_data *data, int enable)
{
	int rc;

	FTS_FUNC_ENTER();
	if (enable) {
	  if (0 == strcmp(global_vdd_mode, "gpio_mode")) {
			/*zhj add for vdd gpio */
			if (gpio_is_valid(data->pdata->power_gpio)) {
				ret = gpio_direction_output(data->pdata->power_gpio, 1);
				if (ret) {
					VIVO_TS_LOG_ERR("Unable to set direction out to 1 for vdd gpio [%d]\n", data->pdata->power_gpio);
					return ret;
				}
			} else {
				VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n", data->pdata->power_gpio);
				ret = -EINVAL;
			}
			/*zhj add end */
		} else {
			rc = regulator_enable(data->vdd);
			if (rc) {
				VTE("Regulator vdd enable failed rc=%d", rc);
			}

			rc = regulator_enable(data->vcc_i2c);
			if (rc) {
				VTE("Regulator vcc_i2c enable failed rc=%d", rc);
			}
		}
	} else {
	 if (0 == strcmp(global_vdd_mode, "gpio_mode")) {
			/*zhj add for vdd gpio */
			if (gpio_is_valid(data->pdata->power_gpio)) {
				ret = gpio_direction_output(data->pdata->power_gpio, 0);
				if (ret) {
					VIVO_TS_LOG_ERR("Unable to set direction out to 1 for vdd gpio [%d]\n", data->pdata->power_gpio);
					return ret;
				}
			} else {
				VIVO_TS_LOG_ERR("Invalid irq gpio [%d]!\n", data->pdata->power_gpio);
				ret = -EINVAL;
			}
			/*zhj add end */
		} else {
		rc = regulator_disable(data->vdd);
		if (rc) {
			VTE("Regulator vdd disable failed rc=%d", rc);
		}
		rc = regulator_disable(data->vcc_i2c);
		if (rc) {
			VTE("Regulator vcc_i2c disable failed rc=%d", rc);
		}
				}
	}
	FTS_FUNC_EXIT();
	return 0;
}

#endif


/*****************************************************************************
 *  Reprot related
 *****************************************************************************/
/*****************************************************************************
 *  Name: fts_release_all_finger
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
#if 0
static void fts_release_all_finger(void)
{
#if FTS_MT_PROTOCOL_B_EN
	unsigned int finger_count = 0;
	fts_irq_disable();
	VTI("###########");

	for (finger_count = 0; finger_count < fts_wq_data->pdata->max_touch_number; finger_count++) {
		input_mt_slot(fts_input_dev, finger_count);
		input_mt_report_slot_state(fts_input_dev, MT_TOOL_FINGER, false);
	}
#else
	fts_irq_disable();
	input_mt_sync(fts_input_dev);
#endif
	input_report_key(fts_input_dev, BTN_TOUCH, 0);
	input_sync(fts_input_dev);
	fts_irq_enable();
	VTI("###########");
}
#endif

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
static void fts_show_touch_buffer(u8 *buf, int point_num)
{
	int len = point_num * FTS_ONE_TCH_LEN;
	int count = 0;
	int i;

	memset(g_sz_debug, 0, 1024);
	if (len > (POINT_READ_BUF - 3)) {
		len = POINT_READ_BUF - 3;
	} else if (len == 0) {
		len += FTS_ONE_TCH_LEN;
	}
	count += snprintf(g_sz_debug, 1023, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
	for (i = 0; i < len; i++) {
		count += snprintf(g_sz_debug + count, 1023, ",%02X", buf[i + 3]);
	}
	VTI("buffer: %s", g_sz_debug);
}
#endif

static int fts_input_dev_report_key_event(struct ts_event *event, struct fts_ts_data *data)
{
	int i;

	if (data->pdata->have_key) {
		if ((1 == event->touch_point || 1 == event->point_num) &&
			 (event->au16_y[0] == data->pdata->key_y_coord)) {

			if (event->point_num == 0) {
				VTI("Keys All Up!");
				for (i = 0; i < data->pdata->key_number; i++) {
					input_report_key(data->input_dev, data->pdata->keys[i], 0);
				}
			} else {
				for (i = 0; i < data->pdata->key_number; i++) {
					if (event->au16_x[0] > (data->pdata->key_x_coords[i] - FTS_KEY_WIDTH) &&
						event->au16_x[0] < (data->pdata->key_x_coords[i] + FTS_KEY_WIDTH)) {

						if (event->au8_touch_event[i] == 0 ||
							event->au8_touch_event[i] == 2) {
							input_report_key(data->input_dev, data->pdata->keys[i], 1);
							VTI("Key%d(%d, %d) DOWN!", i, event->au16_x[0], event->au16_y[0]);
						} else {
							input_report_key(data->input_dev, data->pdata->keys[i], 0);
							VTI("Key%d(%d, %d) Up!", i, event->au16_x[0], event->au16_y[0]);
						}
						break;
					}
				}
			}
			input_sync(data->input_dev);
			return 0;
		}
	}

	return MINUS_ONE;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_dev_report_b(struct ts_event *event, struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;

	for (i = 0; i < event->touch_point; i++) {
		/*input_mt_slot(data->input_dev, event->au8_finger_id[i]); */

		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN || event->au8_touch_event[i] == FTS_TOUCH_CONTACT) {
			/*input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true); */

#if FTS_REPORT_PRESSURE_EN
#if FTS_FORCE_TOUCH_EN
			if (event->pressure[i] <= 0) {
				VTE("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
#else
			event->pressure[i] = 0x3f;
#endif
			/*input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]); */
#endif

			if (event->area[i] <= 0) {
				VTE("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			/*input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]); */

			/*input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]); */
			/*input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]); */
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);

			vivoTsInputReport(VTS_TOUCH_DOWN, event->au8_finger_id[i], event->au16_x[i], event->au16_y[i], event->area[i]);

#if FTS_REPORT_PRESSURE_EN
			VTD("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i],
					  event->au16_y[i], event->pressure[i], event->area[i]);
#else
			VTD("[B]P%d(%d, %d)[tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i], event->au16_y[i], event->area[i]);
#endif
		} else {
			uppoint++;
			/*input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false); */
#if FTS_REPORT_PRESSURE_EN
			/*input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0); */
#endif
			data->touchs &= ~BIT(event->au8_finger_id[i]);
			vivoTsInputReport(VTS_TOUCH_UP, event->au8_finger_id[i], event->au16_x[i], event->au16_y[i], event->area[i]);
			/*VTI("[B]P%d UP!", event->au8_finger_id[i]); */
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < data->pdata->max_touch_number; i++) {
			if (BIT(i) & (data->touchs ^ touchs)) {
				VTI("[B1]P%d UP!", i);
				/*input_mt_slot(data->input_dev, i); */
				/*input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false); */
#if FTS_REPORT_PRESSURE_EN
				/*input_report_abs(data->input_dev, ABS_MT_PRESSURE, 0); */
#endif
				vivoTsInputReport(VTS_TOUCH_UP, i, event->au16_x[i], event->au16_y[i], event->area[i]);
			}
		}
	}

	data->touchs = touchs;

	if (event->touch_point == uppoint) {
		/*VTI("Points All Up!"); */
		vivoTsReleasePoints();
		/* - lgj_0210 input_report_key(data->input_dev, BTN_TOUCH, 0); */
	}
#if 0 /*lgj_0210 Done in vivoTsInputReport function */
	else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);
#endif /*lgj_0210 */
	return 0;

}

#else
static int fts_input_dev_report_a(struct ts_event *event, struct fts_ts_data *data)
{
	int i = 0;
	int uppoint = 0;
	int touchs = 0;

	for (i = 0; i < event->touch_point; i++) {

		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN || event->au8_touch_event[i] == FTS_TOUCH_CONTACT) {
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
#if FTS_REPORT_PRESSURE_EN
#if FTS_FORCE_TOUCH_EN
			if (event->pressure[i] <= 0) {
				VTE("[B]Illegal pressure: %d", event->pressure[i]);
				event->pressure[i] = 1;
			}
#else
			event->pressure[i] = 0x3f;
#endif
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
#endif

			if (event->area[i] <= 0) {
				VTE("[B]Illegal touch-major: %d", event->area[i]);
				event->area[i] = 1;
			}
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);

			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);

			input_mt_sync(data->input_dev);

#if FTS_REPORT_PRESSURE_EN
			VTD("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i],
					  event->au16_y[i], event->pressure[i], event->area[i]);
#else
			VTD("[B]P%d(%d, %d)[tm:%d] DOWN!", event->au8_finger_id[i], event->au16_x[i], event->au16_y[i], event->area[i]);
#endif
		} else {
			uppoint++;
		}
	}

	data->touchs = touchs;
	if (event->touch_point == uppoint) {
		VTD("Points All Up!");
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		input_mt_sync(data->input_dev);
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);

	return 0;
}
#endif

/*****************************************************************************
 *  Name: fts_read_touchdata
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static u8 irq_count_num;
static int fts_read_touchdata(struct fts_ts_data *data)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	u8 pointid = FTS_MAX_ID;
	int ret = -1;
	int i;
	static int large_flag;
	struct ts_event *event = &(data->event);

	VTD("======enter");

#if FTS_GESTURE_EN
	{
		u8 state;
		if (data->suspended) {
			fts_i2c_read_reg(data->client, FTS_REG_GESTURE_EN, &state);
			if (state == 1) {
				fts_gesture_readdata(data->client);
				return 1;
			}
		}
	}
#endif


#if FTS_PSENSOR_EN
	if ((fts_sensor_read_data(data) != 0) && (data->suspended == 1)) {
		return 1;
	}
#endif


#if FTS_READ_TOUCH_BUFFER_DIVIDED
	memset(buf, 0xFF, POINT_READ_BUF);
	memset(event, 0, sizeof(struct ts_event));

	buf[0] = 0x00;
	ret = fts_i2c_read(data->client, buf, 1, buf, (3 + FTS_ONE_TCH_LEN));
	if (ret < 0) {
		VTE("%s read touchdata failed.", __func__);
		return ret;
	}
	event->touch_point = 0;
	event->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	if (event->point_num > data->pdata->max_touch_number)
		event->point_num = data->pdata->max_touch_number;

	if (event->point_num > 1) {
		buf[9] = 0x09;
		fts_i2c_read(data->client, buf + 9, 1, buf + 9, (event->point_num - 1) * FTS_ONE_TCH_LEN);
	}
#else
	ret = fts_i2c_read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		VTE("[B]Read touchdata failed, ret: %d", ret);
		fts_reset_proc(300);
		return ret;
	}


#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_queue_work();
#endif

	memset(event, 0, sizeof(struct ts_event));
	event->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
		fts_show_touch_buffer(buf, event->point_num);
#endif

	/* ESD IC reset  */
	if ((event->point_num == 0x0F) && (buf[1] == 0xFF) && (buf[2] == 0xFF) && (buf[3] == 0xFF)
		 && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
		VTI("======enter====irq_count:%d", irq_count_num++);
		fts_tp_state_recovery(fts_i2c_client);
		vivoTsReleasePoints();
		return MINUS_ONE;
	}
	/**/

	if (event->point_num > data->pdata->max_touch_number)
		event->point_num = data->pdata->max_touch_number;
	event->touch_point = 0;
#endif

	for (i = 0; i < data->pdata->max_touch_number; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			event->touch_point++;

		event->au16_x[i] =
			(s16) (buf[FTS_TOUCH_X_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
			8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_ONE_TCH_LEN * i];
		event->au16_y[i] =
			(s16) (buf[FTS_TOUCH_Y_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
			8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_ONE_TCH_LEN * i];
		event->au8_touch_event[i] =
			buf[FTS_TOUCH_EVENT_POS + FTS_ONE_TCH_LEN * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->area[i] =
			(buf[FTS_TOUCH_AREA_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->pressure[i] =
			(s16) buf[FTS_TOUCH_PRE_POS + FTS_ONE_TCH_LEN * i];

		if (0 == event->area[i])
			event->area[i] = 0x09;

		if (0 == event->pressure[i])
			event->pressure[i] = 0x3f;

		if ((event->au8_touch_event[i] == 0 || event->au8_touch_event[i] == 2) && (event->point_num == 0))
			break;
	}
#if 1

	if (event->point_num == 0 && buf[1] == 1 && large_flag == 0) {
		g_vts_data->largePressNum++;
		large_flag = 1;
	} else if (event->point_num == 0 && buf[1] == 0) {
		large_flag = 0;
	}

	/*when event->point_num == 0, also call it, because we need set AA release time */
/*	if (event->point_num || 	buf[1]) { */
		ret = vivoTsCoverMute(event->point_num, buf[1]);	/*this buf[1] is always 0, because this firmware not support */
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
/*	} else if ((0 == buf[1]) && (event->point_num < 3)) { */
/*		//if it's not large mode, clear the status */
/*		vivoTsCoverMuteRelease(); */
/*	} */
#endif
	return 0;
}

/*****************************************************************************
 *  Name: fts_report_value
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;

	VTD("point number: %d, touch point: %d, irq_count_num:%d", event->point_num,
			  event->touch_point, irq_count_num);

	if (0 == fts_input_dev_report_key_event(event, data)) {
		return;
	}

#if FTS_MT_PROTOCOL_B_EN
	fts_input_dev_report_b(event, data);
#else
	fts_input_dev_report_a(event, data);
#endif

	return;

}


static void fts_ts_work_func(void *dev_id)
{
	int ret = -1;
/*	#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif*/
	VTD("======enter");

	ret = fts_read_touchdata(fts_wq_data);

	if (ret == 0) {
		fts_report_value(fts_wq_data);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif

	VTD("=======exit=");
/*	enable_irq(fts_ts_int->client->irq); */
	fts_irq_enable();
	return;

}

/*****************************************************************************
 *  Name: fts_ts_interrupt
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_ts = dev_id;
#if 0
	int ret = -1;
#endif
	if (!fts_ts) {
		VTE("[INTR]: Invalid fts_ts");
		return IRQ_HANDLED;
	}
/*	mutex_lock(&fts_wq_data->suspend_mutex); */
/*	disable_irq_nosync(fts_ts_int->client->irq); */	
	fts_irq_disable();
	
#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif
	VTD("======enter");
#if 0
	ret = fts_read_touchdata(fts_wq_data);

	if (ret == 0) {
		fts_report_value(fts_wq_data);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif
#endif
	vtsIrqBtmThreadWake();
	VTD("=======exit=");
/*	mutex_unlock(&fts_wq_data->suspend_mutex); */

	/*enable_irq(fts_ts->client->irq); */

	return IRQ_HANDLED;
}

/*****************************************************************************
 *  Name: fts_gpio_configure
 *  Brief: Configure IRQ&RESET GPIO
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_gpio_configure(struct fts_ts_data *data)
{
	int err = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		err = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (err) {
			VTE("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		err = gpio_direction_input(data->pdata->irq_gpio);
		if (err) {
			VTE("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}
	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		err = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (err) {
			VTE("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		err = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (err) {
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
	return err;
}


/*****************************************************************************
 *  Name: fts_get_dt_coords
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_get_dt_coords(struct device *dev, char *name,
							 struct fts_ts_platform_data *pdata)
{
	u32 coords[FTS_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;


	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		VTE("invalid %s", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		VTE("Unable to read %s", name);
		return rc;
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

	return 0;
}

/*****************************************************************************
 *  Name: fts_parse_dt
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	FTS_FUNC_ENTER();

	rc = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		VTE("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		rc = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
		if (rc) {
			VTE("Key number undefined!");
		}
		rc = of_property_read_u32_array(np, "focaltech,keys",
										pdata->keys, pdata->key_number);
		if (rc) {
			VTE("Keys undefined!");
		}
		rc = of_property_read_u32(np, "focaltech,key-y-coord", &pdata->key_y_coord);
		if (rc) {
			VTE("Key Y Coord undefined!");
		}
		rc = of_property_read_u32_array(np, "focaltech,key-x-coords",
										pdata->key_x_coords, pdata->key_number);
		if (rc) {
			VTE("Key X Coords undefined!");
		}
		VTD("%d: (%d, %d, %d), [%d, %d, %d][%d]",
				  pdata->key_number, pdata->keys[0], pdata->keys[1], pdata->keys[2],
				  pdata->key_x_coords[0], pdata->key_x_coords[1], pdata->key_x_coords[2],
				  pdata->key_y_coord);
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0) {
		VTE("Unable to get reset_gpio");
	}

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio", 0, &pdata->irq_gpio_flags);
	VTI("irq_gpio is %d", pdata->irq_gpio);
	if (pdata->irq_gpio < 0) {
		VTE("Unable to get irq_gpio");
	}
	/*zhj add power gpio */
	pdata->power_gpio = of_get_named_gpio_flags(np, "focaltech,power-gpio", 0, &pdata->power_gpio_flags);
	if (pdata->power_gpio < 0) {
		VTE("Unable to get power_gpio");
	}
	/*zhj add end */

	rc = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (!rc) {
		pdata->max_touch_number = temp_val;
		VTD("max_touch_number=%d", pdata->max_touch_number);
	} else {
		VTE("Unable to get max-touch-number");
		pdata->max_touch_number = FTS_MAX_POINTS;
	}



	FTS_FUNC_EXIT();
	return 0;
}

#if 0
#if defined(CONFIG_FB)
/*****************************************************************************
 *  Name: fb_notifier_callback
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fb_notifier_callback(struct notifier_block *self,
								unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct fts_ts_data *fts_data =
		container_of(self, struct fts_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		fts_data && fts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			fts_ts_resume(&fts_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			fts_ts_suspend(&fts_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*****************************************************************************
 *  Name: fts_ts_early_suspend
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
											struct fts_ts_data,
											early_suspend);

	fts_ts_suspend(&data->client->dev);
}

/*****************************************************************************
 *  Name: fts_ts_late_resume
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
											struct fts_ts_data,
											early_suspend);

	fts_ts_resume(&data->client->dev);
}
#endif
#endif
#if 0
static int focaltech_set_gesture_switch_to_chip(unsigned int gesture_switch)
{
	u8 sign_enable = ((gesture_switch >> 24)&0x01) | ((gesture_switch >> 23)&0x02) |
		((gesture_switch >> 23)&0x04) | ((gesture_switch >> 13)&0x08) | ((gesture_switch)&0x01 << 4);

	u8 letter_enable = (gesture_switch >> 26) & 0x1f;
	u8 udf_gesture = (gesture_switch >> 8) & 0xff;

	int ret = 0;
	sign_enable = 0x3f;
	letter_enable = 0x1f;

/* get
 * byte 0:dclick
 * byte 1:udf switch
 * byte 2:swip switch
 * byte 4:c e m w o up LR(bit 0)
 */

	/*
	符号使能 0xd1//bit0 - bit4:L, R, U, D, double click,
	字母使能 0xd2//bit0 - bit4:o, w, m, e, c,
	 */
	ret = fts_i2c_write_reg(fts_i2c_client, ID_G_SPEC_GESTURE_ENABLE_SIGN, sign_enable);
	if (ret < 0) {
		VTI("write sign_enable fail");
	}

	ret = fts_i2c_write_reg(fts_i2c_client, ID_G_SPEC_GESTURE_ENABLE_CHAR, letter_enable);
	if (ret < 0) {
		VTI("write letter_enable fail");
	}

	ret = fts_i2c_write_reg(fts_i2c_client, ID_G_SPEC_GESTURE_EXPORT_ENABLE_A, 0x1);
	if (ret < 0) {
		VTI("write A enbale fail");
	}

	ret = fts_i2c_write_reg(fts_i2c_client, ID_G_SPEC_GESTURE_EXPORT_ENABLE_F, 0x10);
	if (ret < 0) {
		VTI("write F enbale fail");
	}

	if (udf_gesture == 0) {
	/*we also used 0xd8 to enable f gesture */
	/*	ret = ft5x0x_write_reg(0xd8, 0x20);
		if (ret < 0) {
			VTI("write udf_gesture fail");
		} */
	}
	return 0;
}
#endif
static int bbk_fw_update(const struct firmware *firmware)
{
	int ret = 0;

	unsigned char *Image = (unsigned char *)kzalloc(firmware->size, GFP_KERNEL);

	if (!Image)
		return -ENOMEM;

	memcpy(Image, firmware->data, firmware->size);

	fts_irq_disable();
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (DISABLE);
#endif
	fts_ctpm_fw_upgrade_bin_vivo(fts_i2c_client, Image, firmware->size);
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (ENABLE);
#endif
	fts_irq_enable();

	kfree(Image);
	Image = NULL;

	return ret;

}


/*---------------------------------------------------------- -
Error Code for Comm
---------------------------------------------------------- - */
#define ERROR_CODE_OK                           0x00
#define ERROR_CODE_CHECKSUM_ERROR               0x01
#define ERROR_CODE_INVALID_COMMAND              0x02
#define ERROR_CODE_INVALID_PARAM                0x03
#define ERROR_CODE_IIC_WRITE_ERROR              0x04
#define ERROR_CODE_IIC_READ_ERROR               0x05
#define ERROR_CODE_WRITE_USB_ERROR              0x06
#define ERROR_CODE_WAIT_RESPONSE_TIMEOUT        0x07
#define ERROR_CODE_PACKET_RE_ERROR              0x08
#define ERROR_CODE_NO_DEVICE                    0x09
#define ERROR_CODE_WAIT_WRITE_TIMEOUT           0x0a
#define ERROR_CODE_READ_USB_ERROR               0x0b
#define ERROR_CODE_COMM_ERROR                   0x0c
#define ERROR_CODE_ALLOCATE_BUFFER_ERROR        0x0d
#define ERROR_CODE_DEVICE_OPENED                0x0e
#define ERROR_CODE_DEVICE_CLOSED                0x0f

#define DEVIDE_MODE_ADDR    0x00
#define REG_LINE_NUM    0x01
#define REG_TX_NUM  0x02
#define REG_RX_NUM  0x03
#define FT8736_LEFT_KEY_REG    0X1E
#define FT8736_RIGHT_KEY_REG   0X1F
#define TP_MAX_TX		50
#define TP_MAX_RX		50
#define BYTES_PER_TIME      128
#define REG_RawBuf0         0x6A
#define REG_RawBuf1         0x6B

static int TP_TX;
static int TP_RX;

static int m_RawData[TP_MAX_TX][TP_MAX_TX] = {{0, 0} };
static int m_DiffData[TP_MAX_TX][TP_MAX_TX] = {{0, 0} };
/*static unsigned char m_ucTempData[TP_MAX_TX * TP_MAX_TX * 2] = {0}; */
static int m_iTempRawData[TP_MAX_TX * TP_MAX_RX] = {0};
static int m_iTempDiffData[TP_MAX_TX * TP_MAX_RX] = {0};

/************************************************************************
 * Name: EnterFactory
 * Brief:  enter Fcatory Mode
 * Input: null
 * Output: null
 * Return: Comm Code. Code = 0 is OK, else fail.
 ***********************************************************************/
static unsigned char EnterFactory(void)
{
	unsigned char RunState = 0;
	int index = 0;
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = fts_i2c_read_reg(fts_i2c_client, DEVIDE_MODE_ADDR, &RunState);
	VTI("ReCode=%d", ReCode);
	if (ReCode >= ERROR_CODE_OK) {
		if (((RunState >> 4)&0x07) == 0x04) {  /*factory */
			ReCode = ERROR_CODE_OK;
		} else {
			ReCode = fts_i2c_write_reg(fts_i2c_client, DEVIDE_MODE_ADDR, 0x40);
			if (ReCode >= ERROR_CODE_OK) {
				for (index = 0; index < 20; ++index) {
					ReCode = fts_i2c_read_reg(fts_i2c_client, DEVIDE_MODE_ADDR, &RunState);
					if (ReCode >= ERROR_CODE_OK) {
						if (((RunState >> 4)&0x07) == 0x04) {
							ReCode = ERROR_CODE_OK;
							break;
						} else {
							ReCode = ERROR_CODE_COMM_ERROR;
						}
					}
					msleep(50);
				}
				if (ReCode != ERROR_CODE_OK)
					FTS_ERROR("EnterFactory read DEVIDE_MODE_ADDR error 3.");
			} else
				FTS_ERROR("EnterFactory write DEVIDE_MODE_ADDR error 2.");
		}
	} else
		FTS_ERROR("EnterFactory read DEVIDE_MODE_ADDR error 1.");

	return ReCode;
}


/*************************************************/
/*获取PanelRows */
/*************************************************/
static unsigned char GetPanelRows(unsigned char *pPanelRows)
{
	return fts_i2c_read_reg(fts_i2c_client, REG_TX_NUM, pPanelRows);
}

/*************************************************/
/*获取PanelCols */
/*************************************************/
static unsigned char GetPanelCols(unsigned char *pPanelCols)
{
	return fts_i2c_read_reg(fts_i2c_client, REG_RX_NUM, pPanelCols);
}


/************************************************************************
 * Name: GetChannelNum
 * Brief:  Get Num of Ch_X, Ch_Y and key
 * Input: none
 * Output: none
 * Return: Comm Code. Code = 0x00 is OK, else fail.
 ***********************************************************************/
static unsigned char GetChannelNum(void)
{
	unsigned char ReCode;
	/*int TxNum, RxNum; */
	int i;
	unsigned char rBuffer[1]; /* = new unsigned char; */

	/*--------------------------------------------"Get Channel X Num..."; */
	for (i = 0; i < 3; i++) {
		ReCode = GetPanelRows(rBuffer);
		if (ReCode >= ERROR_CODE_OK) {
			if (0 < rBuffer[0] && rBuffer[0] < 80) {
				TP_TX = rBuffer[0];
				if (TP_TX > TP_MAX_TX) {
					FTS_ERROR("Failed to get Channel X number, Get num = %d, UsedMaxNum = %d",
								   TP_TX, TP_MAX_TX);
				   TP_TX = 0;
					return ERROR_CODE_INVALID_PARAM;
				}
				break;
			} else {
				msleep(150);
				continue;
			}
		} else {
			FTS_ERROR("Failed to get Channel X number");
			msleep(150);
		}
	}

	/*--------------------------------------------"Get Channel Y Num..."; */
	for (i = 0; i < 3; i++) {
		ReCode = GetPanelCols(rBuffer);
		if (ReCode >= ERROR_CODE_OK) {
			if (0 < rBuffer[0] && rBuffer[0] < 80) {
				TP_RX = rBuffer[0];
				if (TP_RX > TP_MAX_RX) {
					FTS_ERROR("Failed to get Channel Y number, Get num = %d, UsedMaxNum = %d",
								  TP_RX, TP_MAX_RX);
					TP_RX = 0;
					return ERROR_CODE_INVALID_PARAM;
				}
				break;
			} else {
				msleep(150);
				continue;
			}
		} else {
			FTS_ERROR("Failed to get Channel Y number");
			msleep(150);
		}
	}

	VTI("CH_X = %d, CH_Y = %d", TP_TX, TP_RX);
	return ReCode;
}


/************************************************************************
 * Name: StartScan
 * Brief:  Scan TP, do it before read Raw Data
 * Input: none
 * Output: none
 * Return: Comm Code. Code = 0x00 is OK, else fail.
 ***********************************************************************/
static int StartScan(void)
{
	unsigned char RegVal = 0x00;
	unsigned char times = 0;
	const unsigned char MaxTimes = 20;  /*The longest wait 160ms */
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = fts_i2c_read_reg(fts_i2c_client, DEVIDE_MODE_ADDR, &RegVal);
	if (ReCode >= ERROR_CODE_OK) {
		RegVal |= 0x80;     /*Top bit position 1, start scan */
		ReCode = fts_i2c_write_reg(fts_i2c_client, DEVIDE_MODE_ADDR, RegVal);
		if (ReCode >= ERROR_CODE_OK) {
			while (times++ < MaxTimes) {     /*Wait for the scan to complete */
				msleep(8);    /*8ms */
				ReCode = fts_i2c_read_reg(fts_i2c_client, DEVIDE_MODE_ADDR, &RegVal);
				if (ReCode >= ERROR_CODE_OK) {
					if ((RegVal >> 7) == 0)
						break;
				} else {
					break;
				}
			}
			if (times < MaxTimes)
				ReCode = ERROR_CODE_OK;
			else
				ReCode = ERROR_CODE_COMM_ERROR;
		}
	}
	return ReCode;

}

/************************************************************************
 * Name: ReadRawData
 * Brief:  read Raw Data
 * Input: Freq(No longer used, reserved), LineNum, ByteNum
 * Output: pRevBuffer
 * Return: Comm Code. Code = 0x00 is OK, else fail.
 ***********************************************************************/
static unsigned char ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3] = {0};
	unsigned char pReadData[ByteNum];
	int i, iReadNum;
	unsigned short BytesNumInTestMode1 = 0;

	iReadNum = ByteNum / BYTES_PER_TIME;

	if (0 != (ByteNum%BYTES_PER_TIME))
		iReadNum++;

	if (ByteNum <= BYTES_PER_TIME) {
		BytesNumInTestMode1 = ByteNum;
	} else {
		BytesNumInTestMode1 = BYTES_PER_TIME;
	}

	ReCode = fts_i2c_write_reg(fts_i2c_client, REG_LINE_NUM, LineNum);/*Set row addr; */


	/************************************************************Read raw data in test mode1 */
	msleep(10);
	I2C_wBuffer[0] = REG_RawBuf0;   /*set begin address */
	ReCode = fts_i2c_read(fts_i2c_client, I2C_wBuffer, 1, pReadData, ByteNum);
	/*
	if (ReCode >= ERROR_CODE_OK) {
		msleep(10);
		ReCode = fts_i2c_read(fts_i2c_client, I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
	}

	for (i = 1; i < iReadNum; i++) {
		if (ReCode != ERROR_CODE_OK) break;

		if (i == iReadNum -1) //last packet
		{
			msleep(10);
			ReCode = fts_i2c_read(fts_i2c_client, NULL, 0, pReadData + BYTES_PER_TIME *i, ByteNum - BYTES_PER_TIME *i);
		} else {
			msleep(10);
			ReCode = fts_i2c_read(fts_i2c_client, NULL, 0, pReadData + BYTES_PER_TIME *i, BYTES_PER_TIME);
		}

	}
 */
	VTI("ReadRawData :%d", ByteNum);
	for (i = 0; i < ByteNum; i++) {

		printk("%x ", pReadData[i]);
	}

	if (ReCode >= ERROR_CODE_OK) {
		for (i = 0; i < (ByteNum >> 1); i++) {
			pRevBuffer[i] = (pReadData[i << 1] << 8) + pReadData[(i << 1) + 1];

		}
	}


	return ReCode;

}


/************************************************************************
 * Name: GetRawData
 * Brief:  Get Raw Data of FT8736
 * Input: none
 * Output: none
 * Return: Comm Code. Code = 0x00 is OK, else fail.
 ***********************************************************************/
static unsigned char focaltech_getRawdataAndDiffdata(int mode)
{
	unsigned char ReCode = 0;
	int iRow = 0;
	int iCol = 0;
	unsigned char val = 0;

	VTI("EnterFactory start");

	ReCode = EnterFactory();
	if (ReCode < 0) {
		VTI("Failed to Enter Factory Mode");
		return ReCode;
	}

	VTI("EnterFactory end");

	if (0 == (TP_TX + TP_RX)) {
		VTI("start get channel num");
		ReCode = GetChannelNum();
		if (ReCode < 0) {
			VTI("Error Channel Num");
			return ERROR_CODE_INVALID_PARAM;
		}
		VTI("get channel num end");
	}
	VTI("GetRawData  GetChannelNum end ...TP_TX %d TP_RX%d", TP_TX, TP_RX);
	/*--------------------------------------------Start Scanning */


	if (mode == 0) {

		VTI("set  read  rawdata");
		ReCode = fts_i2c_write_reg(fts_i2c_client, 0x06, 0);
		if (ReCode < 0) {
				VTI("read  rawdata failed");

		} else {
				VTI("read  rawdata done");

				ReCode = fts_i2c_read_reg(fts_i2c_client, 0x06, &val);
				if (ReCode < 0) {
					VTI("check read  rawdata failed");

				} else {

					if (val == 0)	{
							VTI("could read rawdata");
							ReCode = 0;
						} else {
								VTI("could not read rawdata");
							ReCode = -1;
				}
			}

		}
	} else {

		VTI("set  read  diffdata");
		ReCode = fts_i2c_write_reg(fts_i2c_client, 0x06, 1);
		if (ReCode < 0) {
			VTI("read  diffdata failed");
		} else {
			VTI("read  diffdata done");

			ReCode = fts_i2c_read_reg(fts_i2c_client, 0x06, &val);
			if (ReCode < 0) {
				VTI("check read  diffdata failed");
			} else {
				if (val == 1)	{
					VTI("could read diffdata");
					ReCode = 0;
				} else {
					VTI("could not read diffdata");
					ReCode = -1;
				}
			}
		}
	}

	VTI("Start Scan");
	ReCode = StartScan();
	if (ReCode < 0) {
		VTI("Failed to Scan");
		return ReCode;
	}

	/*--------------------------------------------Read RawData */

	if (mode == 0) {
		memset(m_RawData, 0, sizeof(m_RawData));
		memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
		ReCode = ReadRawData(0, 0xAD, (TP_TX * TP_RX) * 2, m_iTempRawData);
		for (iRow = 0; iRow < TP_TX; iRow++) {
			for (iCol = 0; iCol < TP_RX; iCol++) {
				m_RawData[iRow][iCol] = m_iTempRawData[iRow * TP_RX + iCol];
			}
		}
	} else {

		memset(m_DiffData, 0, sizeof(m_DiffData));
		memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
		ReCode = ReadRawData(1, 0xAD, (TP_TX * TP_RX) * 2, m_iTempDiffData);
		for (iRow = 0; iRow < TP_TX; iRow++) {
			for (iCol = 0; iCol < TP_RX; iCol++) {
				m_DiffData[iRow][iCol] = (signed int)m_iTempDiffData[iRow * TP_RX + iCol];
			}
		}
	}
	return ReCode;
}


static int focaltech_get_rawordiff_data(int which, int *data)
{
	int TP_TX = 0xff & vivoTsGetTxRxNum();
	int iRow = 0;
	int TP_RX = 0xff & (vivoTsGetTxRxNum() >> 8);
	int iCol = 0;
	unsigned char ReCode = 0;
	unsigned char val = 0;
	int *tmp_data = NULL;
	int i;
for (i = 0; i < 3; i++) {
	VTI("get data %d(0:rawdata 1:diffdata)", which);
	if (which == 0) {	/*rawdata */
		focaltech_getRawdataAndDiffdata(0);
		tmp_data = &m_RawData[0][0];
	}
	if (which == 1) {	/*diffdata */
		focaltech_getRawdataAndDiffdata(1);
		tmp_data = &m_DiffData[0][0];
	}
}
	VTI("TP_TX=%d, TP_RX=%d", TP_TX, TP_RX);
	for (iRow = 0; iRow < TP_TX; iRow++) {
		for (iCol = 0; iCol < TP_RX; iCol++) {
			data[iRow * TP_RX + iCol] = tmp_data[iRow * TP_MAX_TX + iCol];
			VTI("%x ", data[iRow * TP_RX + iCol]);	/*data[iRow *TP_RX + iCol]); */
		}

	}

	ReCode = fts_i2c_write_reg(fts_i2c_client, 0x00, 0);
	if (ReCode < 0) {
		VTI("exit factary mode failed");
	} else {
		VTI("read factary mode done");
		ReCode = fts_i2c_read_reg(fts_i2c_client, 0x00, &val);
		if (ReCode < 0) {
			VTI("exit factary mode failed");
		} else {
			VTI("exit factary mode sucess");
		}
	}

	return 0;
}
static int last_ts_state = TOUCHSCREEN_NORMAL;

#ifdef CONFIG_LCM_PANEL_TYPE_TFT
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
#else
static int mdss_dsi_panel_reset_and_powerctl(int enable)
{
	return 0;
}
#endif

static int focaltech_mode_change(int which)
{

	unsigned char tmp = 0;

	int ret = 0;
	int i;

	VTI("fts: +++0:normal,1:sleep,2:gesture+++, which = %d", which);

	if (which == TOUCHSCREEN_NORMAL) {
		VTI("change to *****normal mode*****");

		mdelay(240);
		atomic_set(&fts_wq_data->ft_apk_open, 0);
		ret = fts_ts_resume(&fts_wq_data->client->dev);
		fts_gesture_data.mode = 0;	/*eleven + */

		atomic_set(&g_vts_data->tsCallBackRTState, CALLBACK_FINISH);

		fts_irq_enable();

#if FTS_ESDCHECK_EN
		fts_esdcheck_resume();
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

			fts_wait_tp_to_valid(fts_i2c_client);
		}

			/*  special for 8006M*/
		for (i = 0 ; i < 2; i++) {
			if (atomic_read(&g_vts_data->tsCallBackRTState) != CALLBACK_RESUME) {
				ret = fts_i2c_write_reg(fts_i2c_client, 0xfd, 0x5a);
				if (ret < 0) {
					VTI("fail to wirte 0xfd register");
					msleep(10); 
					continue;
				} else {
					VTI("wirte 0xfd success GGGGGGG");
				}

				ret = fts_i2c_read_reg(fts_i2c_client, 0xfd, &tmp);
				if (ret < 0) {
					VTI("read fd fail");
				} else {
					if (tmp == 0x5a) {
						VTI("read 0xfd success");
						break;
					} else {
						VTI("0xfd not correct : 0x%x", tmp);
					}
				}

				msleep(5);
				continue;

			} else {
				VTI("--------panel on, no need write FD--------");
				return 0;
			}
		}
		fts_gesture_data.mode = 1;						/* + eleven */
		fts_wq_data->suspended = 0;
		ret = fts_ts_suspend(&fts_wq_data->client->dev);
		fts_irq_enable();

#if FTS_ESDCHECK_EN
		fts_esdcheck_suspend();
#endif

	}
	if (which == TOUCHSCREEN_SLEEP) {
		VTI("change to *****sleep mode*****");

#if FTS_ESDCHECK_EN
		fts_esdcheck_suspend();
#endif

		fts_gesture_data.mode = 0;
		fts_wq_data->suspended = true;

		fts_irq_disable();

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
		ret = fts_i2c_read_reg(fts_i2c_client, 0xA6, &ver);
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
		return VTS_MVC_TRY;	/*TRY */
	}

	return VTS_MVC_TRY; /* default TRY */

}

static int focaltech_gesture_point(u16 *data)
{
	return fts_gesture_buf_vivo(data);
}

static int focaltech_charger_state_write(int state)
{
	int i = 0;
	u8 temp_data = 0;
	int ret = 0;
	u8 charge_state = (u8)(state?1:0);

	VTI("write charger state: %d ", charge_state);

	if (atomic_read(&fts_wq_data->ft_apk_open) == 1) {
		VTI("ft_apk_open is running, forbid write charger state to TP !");
		return 0;
	}

	for (i = 0; i < 10; i++) {
		ret = fts_i2c_write_reg(fts_i2c_client, 0x8b, charge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8b register %d", charge_state);

		mdelay(5);

		ret = fts_i2c_read_reg(fts_i2c_client, 0x8b, &temp_data);
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
		ret = fts_i2c_write_reg(fts_i2c_client, 0x86, idle_state);
		if (ret < 0)
			VTI("fail to wirte 0x86 register %d", idle_state);

		mdelay(5);

		ret = fts_i2c_read_reg(fts_i2c_client, 0x86, &temp_data);
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
		return MINUS_ONE;
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
		ret = fts_i2c_write_reg(fts_i2c_client, 0x8c, edge_state);
		if (ret < 0)
			VTI("fail to wirte 0x8a register %d", edge_state);

		mdelay(5);

		ret = fts_i2c_read_reg(fts_i2c_client, 0x8c, &temp_data);
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


/*
static int get_module_pin_value(void)
{
		int ret_0 = 0;
		int ret_1 = 0;

	//ret_0 = gpio_get_value(PANEL_ID_PIN0);
	//	ret_1 = gpio_get_value(PANEL_ID_PIN1);
	return (ret_1 << 1) | ret_0;
}
 */

static int print_info(unsigned char *buf)
{

	int ret;
	u8 initcode_ver = 0;
	u8 auc_i2c_write_buf[10] = {0};
	u8 reg_val[4] = {0};

	ret = fts_i2c_read_reg(fts_i2c_client, 0xE4, &initcode_ver);
	if (ret < 0)
		VTI("fail to wirte 0x8a register %d", initcode_ver);
	else
		VTI("read init code version = %d", initcode_ver);

	auc_i2c_write_buf[0] = 0x7D;
	ret = fts_i2c_read(fts_i2c_client, auc_i2c_write_buf, 1, reg_val, 3);
	if (ret < 0)
		VTI("fail to read 0x02 register ");
	else
		VTI("flash manufactory id:%02x device id:%02x%02x", reg_val[0], reg_val[1], reg_val[2]);

	return 0;

}

static int get_iic_bus_state(void)
{
	return qup_i2c_suspended;
}

extern unsigned int mdss_report_lcm_id(void);
static int get_lcm_id(void)
{

	int lcm_id;

	lcm_id = mdss_report_lcm_id();

	VTI("lcm id: 0x%x", lcm_id);

	return lcm_id;
}

/*AUO TRULY  lcm id:0x00 */
const unsigned char VIVO_CUST_FW_8719_TRULY[] =
{
#include "include/firmware/Vivo_CPD1811_FT8719_AUO0xE8_TRULY0x5A_6P3_VER0x25_20180927_all.i"
};

/*AUO TRULY  lcm id:0x01 */
const unsigned char VIVO_CUST_FW_8719_TRULY_PD1813E[] =
{
#include "include/firmware/Vivo_CPD1811_FT8719_AUO0xE8_TRULY0x5A_6P3_VER0x21_20190107_all.i"
};


const unsigned char VIVO_CUST_FW_8719_TRULY_FEX[] = {
#include "include/firmware/Vivo_CPD1811_FT8719_AUO0xE8_TRULY0x5A_6P3_VER0x42_20180724_all.i"
};
/*
//AUO  lcm id:0x00
const unsigned char VIVO_CUST_FW_8006_AUO[] =
{
#include "include/firmware/VIVO_CP1708_FT8006M_VID0xE8_PID0xB0_5P99_VER0x71_L0x02_20170703_all.i"
};

//BOE
const unsigned char VIVO_CUST_FW_8006_BOE[] =
{
#include "include/firmware/VIVO_CP1708_FT8006M_VID0xDA_PID0xC3_5P99_VER0x14_L0x09_20170905_all.i"
};

//PD1708A 6000  lcm id:0x30
const unsigned char VIVO_CUST_FW_8006_BOE_6000[] =
{
#include "include/firmware/VIVO_CP1708_FT8006M_VID0xDA_PID0xC3_5P99_VER0xE2_L0x09_20170629_all.i"
};


const unsigned char VIVO_CUST_FW_8006_TIANMA[] =
{

#include "include/firmware/VIVO_CP1708_FT8006M_VID0x8D_PID0x8D_5P99_VER0x5A_L0x51_20170907_all.i"

};

//4500  lcm id:0x32
const unsigned char VIVO_CUST_FW_8006_BOE_FEX[] =
{

#include "include/firmware/VIVO_CP1708F_FT8006M_VID0xDA_PID0xC3_5P99_VER0x41_L0x09_20170905_all.i"

};

//PD1708F_EX 6000  lcm id:0x30
const unsigned char VIVO_CUST_FW_8006_BOE_FEX_6000[] =
{

#include "include/firmware/VIVO_CP1708F_FT8006M_VID0xDA_PID0xC3_5P99_VER0xE1_L0x09_20170629_all.i"

};

const unsigned char VIVO_CUST_FW_CP1745_BOE[] =
{
#include "include/firmware/VIVO_CP1745F_FT8006M_VID0x67_PID0xC3_5P7_VER0x35_L0x06_20171017_all.i"
};

const unsigned char VIVO_CUST_FW_CP1745_TIANMA[] =
{
#include "include/firmware/VIVO_CP1745_FT8006M_VID0x8D_PID0x8D_5P7_VER0x72_L0x51_20171013_all.i"
};

//BOE ChongQing
const unsigned char VIVO_CUST_FW_CP1745_BOE_CQ[] =
{
#include "include/firmware/VIVO_CP1745_FT8006M_VID0x67_PID0xC8_5P7_VER0x90_L0x04_20170811_all.i"
};
*/

/*****************************************************************************
 *  Name: fts_ts_probe
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/

static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *data;
/*    struct input_dev *input_dev; */
	int err;
	int retval;
	struct vivo_ts_struct *vts_data = NULL;

	FTS_FUNC_ENTER();
	/* 1. Get Platform data */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
							 sizeof(struct fts_ts_platform_data),
							 GFP_KERNEL);
		if (!pdata) {
			VTE("[MEMORY]Failed to allocate memory");
			FTS_FUNC_EXIT();
			err =  -ENOMEM;
			goto err_pdata;


		}
		err = fts_parse_dt(&client->dev, pdata);
		if (err) {
			VTE("[DTS]DT parsing failed");
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		VTE("Invalid pdata");
		FTS_FUNC_EXIT();
		err =  -EINVAL;
		goto free_pdata;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VTE("I2C not supported");
		FTS_FUNC_EXIT();
		err =  -ENODEV;
		goto free_pdata;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct fts_ts_data), GFP_KERNEL);
	if (!data) {
		VTE("[MEMORY]Failed to allocate memory");
		FTS_FUNC_EXIT();
		err =  -ENOMEM;
		goto free_pdata;
	}

/*
	input_dev = input_allocate_device();
	if (!input_dev) {
		VTE("[INPUT]Failed to allocate input device");
		FTS_FUNC_EXIT();
		err =  -ENOMEM;
		goto free_ftsdata;
	}
 */
	vts_data = vivoTsAlloc();
	g_vts_data = vts_data;
	if (vts_data == NULL) {
		err =  -ENOMEM;
		goto free_ftsdata;
	}

	vts_data->client = client;
	vts_data->tsDimensionX = 1080;//pdata->x_max;
	vts_data->tsDimensionY = 2280;//pdata->y_max;
	vivoTsSetTxRxNum(18, 36);
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

	vts_data->writeImei = fts_imei_write;
	vts_data->readImei = fts_imei_read;

	switch (pdata->chip_id) {
	case VTS_IC_TD4322:
		vts_data->icModeChange = NULL;
		break;
	case VTS_IC_TD3320:
		vts_data->icModeChange = NULL;
		break;
	default:
		vts_data->icModeChange = focaltech_mode_change;
		VTI("unknown chip id");
	}

	vts_data->updateFirmware = bbk_fw_update;
	vts_data->setGestureSwitch = NULL;/*focaltech_set_gesture_switch_to_chip;		//if all gusture switch default on, this function no use */
	vts_data->getRawOrDiffData = focaltech_get_rawordiff_data;
	vts_data->getIcFirmwareOrConfigVersion = bbk_focaltech_fw_version;
	vts_data->getModuleId = focaltech_module_id;
	vts_data->getGesturePointData = focaltech_gesture_point;
	vts_data->setChargerFlagSwitch = focaltech_charger_state_write;
	vts_data->getTsPinValue = NULL;
	vts_data->sensorTest = NULL;
	vts_data->setEdgeRestainSwitch = focaltech_edge_state_write;
	vts_data->getLcmId = get_lcm_id;
	vts_data->getI2cBusState = get_iic_bus_state;
	vts_data->otherInfo = print_info;
	vts_data->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
	vts_data->irqBtmThreadHandler = fts_ts_work_func;
	vts_data->idleEnableOrDisable = focaltech_idle_state_write;
	vts_data->proFcFilterSwitch = 1;
	vts_data->sensorTestKey = "com.focaltouchscreen.sensortest:MainActivity:com.focaltouchscreen.sensortest:0:focal_test_result";
	vts_data->lcmNoiseTestKey = "com.focaltech.deltadiff:MainActivity:null:null:null";
	//vts_data->bspLcmNoiseTestKey = "com.focaltech.deltadiff:AutoActivity:com.focaltech.deltadiff:0:testResult";
	vts_data->rawdataTestKey = "com.focaltech.deltadiff:DataActivity:null:null:null";
/*
	vivoTsFwAdd(VTS_FW_TYPE_FW, 	VIVO_CUST_FW_8719_BOE_BMDT,			sizeof(VIVO_CUST_FW_8719_BOE_BMDT),			"PD1730F_EX",		VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x31);
	vivoTsFwAdd(VTS_FW_TYPE_FW, 	VIVO_CUST_FW_8719_TM,			sizeof(VIVO_CUST_FW_8719_TM),			"PD1730F_EX",		VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x11);
*/
	vivoTsFwAdd(VTS_FW_TYPE_FW, VIVO_CUST_FW_8719_TRULY, sizeof(VIVO_CUST_FW_8719_TRULY), "PD1813", VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x00);
	vivoTsFwAdd(VTS_FW_TYPE_FW, VIVO_CUST_FW_8719_TRULY_PD1813E, sizeof(VIVO_CUST_FW_8719_TRULY_PD1813E), "PD1813", VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x01);

	vivoTsFwAdd(VTS_FW_TYPE_FW, VIVO_CUST_FW_8719_TRULY_FEX, sizeof(VIVO_CUST_FW_8719_TRULY_FEX), "PD1813F_EX", VTS_FW_NOPIN_VALUE, VTS_MVC_TRY, 0x00);

	retval = vivoTsInit(client, NULL, -1);
	if (retval < 0) {
		err = -ENOMEM;
		goto free_vivo_ts;

	}
	/*this must be set up after vivoTsInit */
	vts_data->suspendEventBlank = FB_EVENT_BLANK;
	vts_data->resumeEventBlank = FB_EVENT_BLANK;

	data->input_dev = vts_data->inputDev;/*change to our dev */
	data->client = client;
	data->pdata = pdata;

	fts_wq_data = data;
	fts_i2c_client = client;
	fts_input_dev = vts_data->inputDev;

	spin_lock_init(&fts_wq_data->irq_lock);
/*	mutex_init(&fts_wq_data->suspend_mutex); */

/*    fts_input_dev_init(client, data, input_dev, pdata); */


#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_init(data);
	fts_power_source_ctrl(data, 1);
#endif

	err = fts_gpio_configure(data);
	if (err < 0) {
		VTE("[GPIO]Failed to configure the gpios");
		goto free_vivotsinit;
	}
FTS_INFO("luoguojin probe");
	VTE("vivo probe");

	/*Since rst is control by LCM LK, this reset in no need */
	/*fts_reset_proc(200); */
	fts_ctpm_get_upgrade_array();

/*	fts_wait_tp_to_valid(client);  already excute in fts_ctpm_get_upgrade_array */

#if 0
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
							   pdata->irq_gpio_flags | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
							   client->dev.driver->name, data);
#else
		client->irq = gpio_to_irq(data->pdata->irq_gpio);
		VTI("client->irq is %d", client->irq);
		err = vivoTsInterruptRegister(client->irq, NULL, fts_ts_interrupt, pdata->irq_gpio_flags /*| IRQF_ONESHOT*/ | IRQF_TRIGGER_FALLING, data);
#endif
	if (err) {
		VTE("Request irq failed!");
		goto free_gpio;
    } else {
		VTI("request irq %d succeed", client->irq);
	}

	fts_irq_disable();

#if FTS_PSENSOR_EN
	if (fts_sensor_init(data) != 0) {
		VTE("fts_sensor_init failed!");
		FTS_FUNC_EXIT();
		err =  -EINVAL;
		goto free_irq;
	}
#endif

#if FTS_APK_NODE_EN
	fts_create_apk_debug_channel(client);
#endif

#if FTS_SYSFS_NODE_EN
	fts_create_sysfs(client);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_init();
#endif

	fts_ex_mode_init(client);

#if FTS_GESTURE_EN
	fts_gesture_init(fts_input_dev, client);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_init();
#endif

	fts_irq_enable();

#if FTS_AUTO_UPGRADE_EN
	/* fw / lcm init code update entrance */
	fts_ctpm_upgrade_init();
#endif

#if FTS_TEST_EN
	fts_test_init(client);
#endif

/*
#if defined(CONFIG_FB)
	if (0) {
		data->fb_notif.notifier_call = fb_notifier_callback;
		err = fb_register_client(&data->fb_notif);
		if (err)
			VTE("[FB]Unable to register fb_notifier: %d", err);
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	data->early_suspend.suspend = fts_ts_early_suspend;
	data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
 */
	vivoTsAfterProbeCompleteCall(client, NULL, -1);

	FTS_FUNC_EXIT();
	return 0;

#if FTS_PSENSOR_EN
free_irq:
	vivoTsInterruptUnregister();
#endif

free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

free_vivotsinit:
	vivoTsDeInit();
	VTE("free_vivotsinit");
free_vivo_ts:
	vivoTsFree();
	VTE("free_vivo_ts");

/*free_input_fts: */
	/*input_free_device(input_dev); */

free_ftsdata:
	/*kfree(data);*/
	VTE("free_ftsdata");

free_pdata:
	/*kfree(pdata);*/
	VTE("free_pdata");

err_pdata:
	VTE("fts_ts_probe faild");
	return err;


}

/*****************************************************************************
 *  Name: fts_ts_remove
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *data = fts_wq_data;/*i2c_get_clientdata(client); */

	FTS_FUNC_ENTER();
	cancel_work_sync(&data->touch_event_work);

#if FTS_PSENSOR_EN
	fts_sensor_remove(data);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit();
#endif

#if FTS_APK_NODE_EN
	fts_release_apk_debug_channel();
#endif

#if FTS_SYSFS_NODE_EN
	fts_remove_sysfs(client);
#endif

	fts_ex_mode_exit(client);

#if FTS_AUTO_UPGRADE_EN
    cancel_work_sync(&fw_update_work);
    if(g_global.ic_type == _FT8719) {
             fts_fwupg_exit();
    }

#endif

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		VTE("Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	input_unregister_device(data->input_dev);

#if FTS_TEST_EN
	fts_test_exit(client);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit();
#endif

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
 *  Name: fts_ts_suspend
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	struct fts_ts_data *data = fts_wq_data;/*dev_get_drvdata(dev); */
	int retval = 0;

	FTS_FUNC_ENTER();
	if (data->suspended) {
		VTI("Already in suspend state");
		FTS_FUNC_EXIT();
		return MINUS_ONE;
	}


	irq_count_num = 0;
	/*fts_release_all_finger(); */

#if FTS_GESTURE_EN
	retval = fts_gesture_suspend(data->client);
	if (retval == 0) {
		/* Enter into gesture mode(suspend) */
		retval = enable_irq_wake(fts_wq_data->client->irq);
		if (retval)
			VTE("%s: set_irq_wake failed", __func__);
		data->suspended = true;
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

#if FTS_PSENSOR_EN
	if (fts_sensor_suspend(data) != 0) {
		enable_irq_wake(data->client->irq);
		data->suspended = true;
		return 0;
	}
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif

	fts_irq_disable();

#if 0
	/* TP enter sleep mode */
	retval = fts_i2c_write_reg(data->client, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
	if (retval < 0) {
		VTE("Set TP to sleep mode fail, ret=%d!", retval);
	}
#endif
	data->suspended = true;

	FTS_FUNC_EXIT();

	return 0;
}

/*****************************************************************************
 *  Name: fts_ts_resume
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *data = fts_wq_data;/*dev_get_drvdata(dev); */

	FTS_FUNC_ENTER();
	if (!data->suspended) {
		VTD("Already in awake state");
		FTS_FUNC_EXIT();
		return MINUS_ONE;
	}

	if (!g_global.is_idc) {
		fts_reset_proc(200);
	}
#if FTS_ESDCHECK_EN
	set_flow_cnt();
#endif

	fts_tp_state_recovery(data->client);

#if FTS_GESTURE_EN
	if (fts_gesture_resume(data->client) == 0) {
		int err;
		err = disable_irq_wake(data->client->irq);
		if (err)
			VTE("%s: disable_irq_wake failed", __func__);
		data->suspended = false;
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

#if FTS_PSENSOR_EN
	if (fts_sensor_resume(data) != 0) {
		disable_irq_wake(data->client->irq);
		data->suspended = false;
		FTS_FUNC_EXIT();
		return 0;
	}
#endif

	data->suspended = false;

	fts_irq_enable();

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif
	FTS_FUNC_EXIT();
	return 0;
}

void fts_ts_ng_check(void)
{
	int ret;
	u8 temp_data = 0;
	ret = fts_i2c_read_reg(fts_i2c_client, 0x02, &temp_data);
	if (ret < 0)
		VTI("fail to read 0x02 register ");
	else
		VTI("*********** 0x02 register 0x%x*********** ", temp_data);
}

/*****************************************************************************
 * I2C Driver
 *****************************************************************************/
static const struct i2c_device_id fts_ts_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct of_device_id fts_match_table[] = {
	{ .compatible = "focaltech,fts", },
	{ },
};

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fts_match_table,
	},
	.id_table = fts_ts_id,
};

static void i2c_register_handler(void)
{
	int ret = 0;
	VIVO_TS_LOG_INF("%s enter\n", __func__);

	ret = i2c_add_driver(&fts_ts_driver);
	if (ret) {
		VIVO_TS_LOG_INF("[%s]i2c add driver fail.\n", __func__);
	}
}

/*extern unsigned int is_atboot;
extern unsigned int power_off_charging_mode;*/

extern unsigned int is_atboot;
static bool boot_mode_normal = true;
extern enum boot_mode_t get_boot_mode(void);

/*****************************************************************************
 *  Name: fts_ts_init
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static int __init fts_ts_init(void)
{
	enum boot_mode_t boot_mode;
	VTI("fts_ts_init enter.");
	
	if ((0x00 != get_lcm_id()) && (0x01 != get_lcm_id()) ) {
		VTI("not 8719");
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
	
	vivo_touchscreen_new_module_init(i2c_register_handler, "i2c_register_handler");
	
	return 0;
}

/*****************************************************************************
 *  Name: fts_ts_exit
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
