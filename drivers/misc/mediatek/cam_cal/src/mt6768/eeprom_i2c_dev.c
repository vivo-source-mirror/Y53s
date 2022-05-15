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
#include "eeprom_i2c_dev.h"

//+vivo zhouyikuan modify for read eeprom fail
#if defined(CONFIG_MTK_CAM_PD1934)
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /* main */
	I2C_DEV_IDX_2, /* sub */
	I2C_DEV_IDX_3, /* main2 */
	I2C_DEV_IDX_1, /* sub2 */
	I2C_DEV_IDX_3, /* main3 */
};
#else
static enum EEPROM_I2C_DEV_IDX gi2c_dev_sel[IMGSENSOR_SENSOR_IDX_MAX_NUM] = {
	I2C_DEV_IDX_1, /*  main */
	I2C_DEV_IDX_2, /*  sub  */
	I2C_DEV_IDX_3, /*  main2 */
	I2C_DEV_IDX_1, /*  sub2 */
	I2C_DEV_IDX_1, /*  main3 */
};
#endif
//-vivo zhouyikuan modify for read eeprom fail

enum EEPROM_I2C_DEV_IDX get_i2c_dev_sel(enum IMGSENSOR_SENSOR_IDX idx)
{
	if (idx >= IMGSENSOR_SENSOR_IDX_MIN_NUM &&
		idx < IMGSENSOR_SENSOR_IDX_MAX_NUM)
		return gi2c_dev_sel[idx];
	return I2C_DEV_IDX_1;
}

int gi2c_dev_timing[I2C_DEV_IDX_MAX] = {
	400, /* dev1, 400k */
	400, /* dev2, 400k */
	400, /* dev3, 400k */
};

