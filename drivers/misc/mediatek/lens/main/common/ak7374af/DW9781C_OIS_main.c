/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define OIS_DEBUG
#ifdef OIS_DEBUG
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif

/* #include <stdio.h> */
#include "DW9781C_OIS_head.h"

#ifdef OIS_DEBUG
#define OIS_DRVNAME "DW9781C_OIS"
#define LOG_INF(format, args...)                                               \
	pr_info(OIS_DRVNAME " [%s] " format, __func__, ##args)
#endif


void DW9781C_setOISMode(int Disable)
{
	//DW9781C_I2C_OIS_per_write(0x7015, Disable); /*ois OFF servo on ,*/

	LOG_INF("PosX = 0x%x \n",DW9781C_I2C_OIS_per_read(0x7049));
	LOG_INF("PosY = 0x%x \n",DW9781C_I2C_OIS_per_read(0x704A));
}


void DW9781C_OIS_Standby(void)
{

}

/* MAIN OIS */
void DW9781C_Main_OIS(void)
{
	LOG_INF("ois reset start\n");
	DW9781C_I2C_OIS_per_write(0xD002, 0x0001); /*reset*/
	mdelay(4);
	DW9781C_I2C_OIS_per_write(0xD001, 0x0001);/*Active mode(DSP ON)*/
	mdelay(25);/*ST gyro - over wait 25ms,default Servo on*/
	DW9781C_I2C_OIS_per_write(0xEBF1, 0x56FA);/*User protection release*/
	mdelay(10);
	/*User protection release*/
	LOG_INF("ois reset end, chip id = 0x%x\n",DW9781C_I2C_OIS_per_read(0x7000));
	DW9781C_I2C_OIS_per_write(0x7015, 0x0000); /* OIS ON */

}

