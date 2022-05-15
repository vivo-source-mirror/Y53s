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

#include "DW9781C_OIS_func.h"
#include "DW9781C_OIS_coef.h"
#include "DW9781C_OIS_defi.h"
#include "DW9781C_OIS_head.h"
#include "DW9781C_OIS_prog.h"


void DW9781C_I2C_OIS_per_write(unsigned short int u16_adr, unsigned short int u16_dat)
{
	unsigned char out[4];

	out[0] = (char)(u16_adr >> 8);
	out[1] = (char)(u16_adr & 0xFF);
	out[2] = (char)(u16_dat >> 8);
	out[3] = (char)(u16_dat & 0xFF);

	DW9781C_WR_I2C(_SLV_OIS_, 4, out);

}

unsigned short int DW9781C_I2C_OIS_per_read(unsigned short int u16_adr)
{

	unsigned char u08_dat[2];

	u08_dat[0] = (char)(u16_adr >> 8);; /*  */
	u08_dat[1] = (char)(u16_adr & 0xFF);	/* target address */
	
	return DW9781C_RD_I2C(_SLV_OIS_, 2, u08_dat);
}


