/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "regulator.h"

#if defined(CONFIG_MTK_CAM_PD1986)
#include <linux/gpio.h>
#include <linux/of_gpio.h>
extern int GPIO64;
extern int gpio64_value;
#endif 


static const int regulator_voltage[] = {
	REGULATOR_VOLTAGE_0,
	REGULATOR_VOLTAGE_1000,
	REGULATOR_VOLTAGE_1050,
	REGULATOR_VOLTAGE_1100,
	REGULATOR_VOLTAGE_1200,
	REGULATOR_VOLTAGE_1210,
	REGULATOR_VOLTAGE_1220,
	REGULATOR_VOLTAGE_1500,
	REGULATOR_VOLTAGE_1800,
	REGULATOR_VOLTAGE_2500,
	REGULATOR_VOLTAGE_2800,
	REGULATOR_VOLTAGE_2900,
};

struct REGULATOR_CTRL regulator_control[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamois"},
	{"vmois"},
#ifdef CONFIG_REGULATOR_FAN53870
	{"plugic_en"},
#endif
	{"vcamd"},
	{"vcamio"},
};

#if defined(CONFIG_MTK_CAM_PD1986)
struct REGULATOR_CTRL regulator_control_b[REGULATOR_TYPE_MAX_NUM] = {
	{"vcama"},
	{"vcamois"},
	{"vmois"},
	{"vcamd"},
	{"vcamio_b"},
};
#endif

static struct REGULATOR reg_instance;

static enum IMGSENSOR_RETURN regulator_init(
	void *pinstance,
	struct IMGSENSOR_HW_DEVICE_COMMON *pcommon)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	char str_regulator_name[LENGTH_FOR_SNPRINTF];

#if defined(CONFIG_MTK_CAM_PD1986)
	struct REGULATOR_CTRL *regulator_c = NULL;
	gpio64_value = gpio_get_value(GPIO64);
	regulator_c = gpio64_value == 1 ? regulator_control:regulator_control_b;
#endif

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {
		for (type = 0;
			type < REGULATOR_TYPE_MAX_NUM;
			type++) {
			memset(str_regulator_name, 0,
				sizeof(str_regulator_name));
			snprintf(str_regulator_name,
				sizeof(str_regulator_name),
				"cam%d_%s",
				idx,
#if defined(CONFIG_MTK_CAM_PD1986)
				regulator_c[type].pregulator_type);
#else
				regulator_control[type].pregulator_type);
#endif
			preg->pregulator[idx][type] = regulator_get(
					&pcommon->pplatform_device->dev,
					str_regulator_name);
			if (IS_ERR(preg->pregulator[idx][type])) {
				//add  use fan53870  boot up then  reboot by changcheng@vivo.com
				preg->pregulator[idx][type] = NULL;
				//add  use fan53870  boot up then  reboot by changcheng@vivo.com
				pr_err("ERROR: regulator[%d][%d]  %s fail!\n",
						idx, type, str_regulator_name);
				}
			atomic_set(&preg->enable_cnt[idx][type], 0);
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_release(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int type, idx;
	struct regulator *pregulator = NULL;
	atomic_t *enable_cnt = NULL;

	for (idx = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		idx++) {

		for (type = 0; type < REGULATOR_TYPE_MAX_NUM; type++) {
			pregulator = preg->pregulator[idx][type];
			enable_cnt = &preg->enable_cnt[idx][type];
			if (pregulator != NULL) {
				for (; atomic_read(enable_cnt) > 0; ) {
					regulator_disable(pregulator);
					atomic_dec(enable_cnt);
				}
			}
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_set(
	void *pinstance,
	enum IMGSENSOR_SENSOR_IDX   sensor_idx,
	enum IMGSENSOR_HW_PIN       pin,
	enum IMGSENSOR_HW_PIN_STATE pin_state)
{
	struct regulator     *pregulator;
	struct REGULATOR     *preg = (struct REGULATOR *)pinstance;
	int reg_type_offset;
	atomic_t             *enable_cnt;


	if (pin > IMGSENSOR_HW_PIN_DOVDD   ||
	    pin < IMGSENSOR_HW_PIN_AVDD    ||
	    pin_state < IMGSENSOR_HW_PIN_STATE_LEVEL_0 ||
	    pin_state >= IMGSENSOR_HW_PIN_STATE_LEVEL_HIGH ||
	    sensor_idx < 0)
		return IMGSENSOR_RETURN_ERROR;

	reg_type_offset = REGULATOR_TYPE_VCAMA;

	pregulator = preg->pregulator[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	enable_cnt = &preg->enable_cnt[(unsigned int)sensor_idx][
		reg_type_offset + pin - IMGSENSOR_HW_PIN_AVDD];

	if (pregulator) {
		if (pin_state != IMGSENSOR_HW_PIN_STATE_LEVEL_0) {
			if (regulator_set_voltage(pregulator,
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0],
				regulator_voltage[
				pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0])) {

				PK_PR_ERR(
				  "[regulator]fail to regulator_set_voltage, powertype:%d powerId:%d\n",
				  pin,
				  regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
			}
			if (regulator_enable(pregulator)) {
				PK_PR_ERR(
				"[regulator]fail to regulator_enable, powertype:%d powerId:%d\n",
				pin,
				regulator_voltage[
				  pin_state - IMGSENSOR_HW_PIN_STATE_LEVEL_0]);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_inc(enable_cnt);
		} else {
			if (regulator_is_enabled(pregulator))
				PK_DBG("[regulator]%d is enabled\n", pin);

			if (regulator_disable(pregulator)) {
				PK_PR_ERR(
					"[regulator]fail to regulator_disable, powertype: %d\n",
					pin);
				return IMGSENSOR_RETURN_ERROR;
			}
			atomic_dec(enable_cnt);
		}
	} else {
		PK_PR_ERR("regulator == NULL %d %d %d\n",
				reg_type_offset,
				pin,
				IMGSENSOR_HW_PIN_AVDD);
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN regulator_dump(void *pinstance)
{
	struct REGULATOR *preg = (struct REGULATOR *)pinstance;
	int i, j;

	for (j = IMGSENSOR_SENSOR_IDX_MIN_NUM;
		j < IMGSENSOR_SENSOR_IDX_MAX_NUM;
		j++) {

		for (i = REGULATOR_TYPE_VCAMA;
		i < REGULATOR_TYPE_MAX_NUM;
		i++) {
			if (!preg->pregulator[j][i])
				continue;
			if (regulator_is_enabled(preg->pregulator[j][i]) &&
				atomic_read(&preg->enable_cnt[j][i]) != 0)
				PK_DBG("index= %d %s = %d\n",
					j,
					regulator_control[i].pregulator_type,
					regulator_get_voltage(
						preg->pregulator[j][i]));
		}
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static struct IMGSENSOR_HW_DEVICE device = {
	.id        = IMGSENSOR_HW_ID_REGULATOR,
	.pinstance = (void *)&reg_instance,
	.init      = regulator_init,
	.set       = regulator_set,
	.release   = regulator_release,
	.dump      = regulator_dump
};

enum IMGSENSOR_RETURN imgsensor_hw_regulator_open(
	struct IMGSENSOR_HW_DEVICE **pdevice)
{
	*pdevice = &device;
	return IMGSENSOR_RETURN_SUCCESS;
}
