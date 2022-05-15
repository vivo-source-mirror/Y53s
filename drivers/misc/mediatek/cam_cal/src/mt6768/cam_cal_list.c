/*
 * Copyright (C) 2018 MediaTek Inc.
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
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{S5KGW3SP13PD2103_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3P9SPPD2103_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1PD2103_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1V1PD2103_SENSOR_ID, 0xA4, Common_read_region},
	{S5KGM1STPD2104_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499PD1913_SENSOR_ID, 0xA6, Common_read_region},
	{IMX471PD1913_SENSOR_ID, 0xA2, Common_read_region},
	{S5KGD1SPPD1913_SENSOR_ID, 0xA2, Common_read_region},
	{HI846PD1913_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856PD1913_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499PD1934_SENSOR_ID, 0xA6, Common_read_region},
	{OV16B10PD1934_SENSOR_ID, 0xA4, Common_read_region},
	{GC2375HPD1934_SENSOR_ID, 0xA4, Common_read_region},
	{S5KGD1SPPD1934_SENSOR_ID, 0xA2, Common_read_region},
	{HI846PD1934_SENSOR_ID, 0xA0, Common_read_region},
	{HI846PD1934V1_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856PD1934_SENSOR_ID, 0xA0, Common_read_region},
    {S5K3P9SP04PD1934_SENSOR_ID, 0xA2, Common_read_region},
	{S5K3P9SP04PD1934V1_SENSOR_ID, 0xA2, Common_read_region},
	{S5K3P9SP04PD1934V2_SENSOR_ID, 0xA2, Common_read_region},
	{OV13B10_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V1_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10PD2066BA_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V1PD2066BA_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10PD2066A_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10PD2143A_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V1PD2066A_SENSOR_ID, 0xA4, Common_read_region},
	{S5K4H7YX_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXV1_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXPD2066BA_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXV1PD2066BA_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXPD2066A_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXPD2143A_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7YXV1PD2066A_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*B+B. No Cal data for main2 OV8856*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
	{OV02B10PD2066_SENSOR_ID, 0xA4, Common_read_region},
    {OV02B10V1PD2066_SENSOR_ID, 0xA4, Common_read_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}