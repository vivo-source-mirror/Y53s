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

#ifndef OIS_USER_C
#define OIS_USER_C
#endif

#include "DW9781C_OIS_head.h"



void DW9781C_WR_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat)
{
	s4AF_WriteReg_DW9781C(slvadr, dat, size);
}

unsigned short int DW9781C_RD_I2C(unsigned char slvadr, unsigned char size,unsigned char *dat)
{
	unsigned short int read_data = 0;
	unsigned short int read_data_h = 0;

	s4AF_ReadReg_DW9781C(slvadr, dat, 2,(unsigned char *)&read_data, 2);

	read_data_h = read_data >> 8;
	read_data = read_data << 8;
	read_data = read_data | read_data_h;

	return read_data;
}

