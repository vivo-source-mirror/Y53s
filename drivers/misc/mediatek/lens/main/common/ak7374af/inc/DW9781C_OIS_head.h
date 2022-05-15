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


#ifndef OIS_MAIN_H
#define OIS_MAIN_H


#include "DW9781C_OIS_defi.h"
/* #include	"windef.h" */


void DW9781C_I2C_OIS_per_write(unsigned short int u16_adr, unsigned short int u16_dat);
unsigned short int DW9781C_I2C_OIS_per_read(unsigned short int u16_adr);

void DW9781C_WR_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat);

unsigned short int DW9781C_RD_I2C(unsigned char slvadr, unsigned char size,unsigned char *dat);

extern void DW9781C_Main_OIS(void);

extern void DW9781C_OIS_Standby(void);

extern void DW9781C_setOISMode(int Disable);

extern int s4AF_WriteReg_DW9781C(unsigned short i2c_id,
				   unsigned char *a_pSendData,
				   unsigned short a_sizeSendData);

extern int s4AF_ReadReg_DW9781C(unsigned short i2c_id,
				  unsigned char *a_pSendData,
				  unsigned short a_sizeSendData,
				  unsigned char *a_pRecvData,
				  unsigned short a_sizeRecvData);





#endif /* OIS_MAIN_H */
