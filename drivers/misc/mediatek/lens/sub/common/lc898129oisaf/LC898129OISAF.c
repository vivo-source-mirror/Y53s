/*
 * Copyright (C) 2016 MediaTek Inc.
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

/*
 * LC898129OISAF voice coil motor driver
 *
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>

#include "lens_info.h"

#define AF_DRVNAME "LC898129OISAF_DRV"
#define AF_I2C_SLAVE_ADDR 0x76
#define _SLV_OIS_ 0x76

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...)                                               \
	pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;

static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4CurrPosition;
static unsigned long g_u4TargetPosition;

#if 0
static int s4AF_ReadReg(u8 a_uAddr, u8 *a_uData)
{
	g_pstAF_I2Cclient->addr = (AF_I2C_SLAVE_ADDR) >> 1;

	if (i2c_master_send(g_pstAF_I2Cclient, &a_uAddr, 1) < 0) {
		LOG_INF("ReadI2C send failed!!\n");
		return -1;
	}

	if (i2c_master_recv(g_pstAF_I2Cclient, a_uData, 1) < 0) {
		LOG_INF("ReadI2C recv failed!!\n");
		return -1;
	}

	/* LOG_INF("ReadI2C 0x%x, 0x%x\n", a_uAddr, *a_uData); */

	return 0;
}
#endif
#if 0
static int s4AF_WriteReg(u8 a_uLength, u8 a_uAddr, u16 a_u2Data)
{
	u8 puSendCmd[2] = {a_uAddr, (u8)(a_u2Data & 0xFF)};
	u8 puSendCmd2[3] = {a_uAddr, (u8)(((a_u2Data << 6) >> 8) & 0xFF),(u8)((a_u2Data << 6) & 0xFF)};
	//LOG_INF("a_u2Data =0x%x , puSendCmd2[0] = 0x%x, puSendCmd2[1] = 0x%x,  puSendCmd2[2] = 0x%x \n", a_u2Data, puSendCmd2[0],  puSendCmd2[1], puSendCmd2[2]);
	
	g_pstAF_I2Cclient->addr = (AF_I2C_SLAVE_ADDR) >> 1;

	if (a_uLength == 0) {
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2) < 0) {
			LOG_INF("WriteI2C failed!!\n");
			return -1;
		}
	} else if (a_uLength == 1) {
		if (i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 3) < 0) {
			LOG_INF("WriteI2C 2 failed!!\n");
			return -1;
		}
	}

	return 0;
}
#endif

/* LC898129AF OIS CODE START*/
int s4AF_WriteReg_LC898129OISAF(unsigned short i2c_id, unsigned char *a_pSendData,unsigned short a_sizeSendData)
{
	int i4RetValue = 0;

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	i4RetValue =
		i2c_master_send(g_pstAF_I2Cclient, a_pSendData, a_sizeSendData);

	if (i4RetValue != a_sizeSendData) {
		LOG_INF("I2C send failed!!, Addr = 0x%x, Data = 0x%x\n",
			a_pSendData[0], a_pSendData[1]);
		return -1;
	}

	return 0;
}

int s4AF_ReadReg_LC898129OISAF(unsigned short i2c_id, unsigned char *a_pSendData,
			   unsigned short a_sizeSendData,
			   unsigned char *a_pRecvData,
			   unsigned short a_sizeRecvData)
{
	int i4RetValue;
	struct i2c_msg msg[2];

	spin_lock(g_pAF_SpinLock);
	g_pstAF_I2Cclient->addr = i2c_id >> 1;
	spin_unlock(g_pAF_SpinLock);

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue =
		i2c_transfer(g_pstAF_I2Cclient->adapter, msg, ARRAY_SIZE(msg));

	if (i4RetValue != 2) {
		LOG_INF("I2C Read failed!!\n");
		return -1;
	}
	return 0;
}


int LC898129OISAF_WR_I2C(unsigned char slvadr, unsigned char size, unsigned char *dat)
{
	return s4AF_WriteReg_LC898129OISAF(slvadr, dat, size);
}

unsigned  int LC898129OISAF_RD_I2C(unsigned char slvadr, unsigned char size,unsigned char *dat)
{
	unsigned  int read_data = 0;
	unsigned  int re_read_data = 0;

	s4AF_ReadReg_LC898129OISAF(slvadr, dat, 2,(unsigned char *)&read_data, 4);

	//LOG_INF("read_data[0] = 0x%x, read_data[1] = 0x%x,read_data[2] = 0x%x,read_data[3] = 0x%x\n",read_data & 0xFF, (read_data >> 8) & 0xFF,( read_data >> 16) & 0xFF, (read_data >> 24) & 0xFF);

	re_read_data = ((read_data & 0xFF) <<24) |(((read_data >> 8) & 0xFF) <<16)|(((read_data >> 16) & 0xFF) <<8) |((read_data >> 24) & 0xFF);

	return re_read_data;
}

int LC898129OISAF_I2C_OIS_per_write(unsigned short int u16_adr, unsigned  int u32_dat)
{
	unsigned char out[6];

	out[0] = (char)(u16_adr >> 8);
	out[1] = (char)(u16_adr & 0xFF);
	out[2] = (char)(u32_dat >> 24);
	out[3] = (char)(u32_dat >> 16 & 0xFF);
	out[4] = (char)(u32_dat >> 8 & 0xFF);
	out[5] = (char)(u32_dat & 0xFF);
	
	//LOG_INF("out[0] = 0x%x, out[1] = 0x%x,out[2] = 0x%x,out[3] = 0x%x,out[4] = 0x%x,out[5] = 0x%x\n",out[0], out[1], out[2], out[3], out[4], out[5]);
	return LC898129OISAF_WR_I2C(_SLV_OIS_, 6, out);

}

unsigned  int LC898129OISAF_I2C_OIS_per_read(unsigned short int u16_adr)
{

	unsigned char u08_dat[2];

	u08_dat[0] = (char)(u16_adr >> 8);; /*  */
	u08_dat[1] = (char)(u16_adr & 0xFF);	/* target address */
	
	return LC898129OISAF_RD_I2C(_SLV_OIS_, 2, u08_dat);
}


void LC898129OISAF_setOISMode(int Disable)
{

	LOG_INF("PosX = 0x%x \n",LC898129OISAF_I2C_OIS_per_read(0x0178));
	LOG_INF("PosY = 0x%x \n",LC898129OISAF_I2C_OIS_per_read(0x017C));
	
	LOG_INF("Gyro X = 0x%x \n",LC898129OISAF_I2C_OIS_per_read(0x0220));
	LOG_INF("Gyro Y = 0x%x \n",LC898129OISAF_I2C_OIS_per_read(0x0224));

}


void LC898129OISAF_OIS_Standby(void)
{

}

/* MAIN OIS */
void LC898129OISAF_Main_OIS(void)
{
	int value = 0, i=0;
	LOG_INF("ois reset start\n");
	LOG_INF("0xF010 x:1 y:2 z:4 AF enble  = 0x%x \n", LC898129OISAF_I2C_OIS_per_read(0xF010));  /*0x0x000007 AF enble*/
	LC898129OISAF_I2C_OIS_per_write(0xF010, 0X00000007);
	mdelay(30);
	do{
		value = LC898129OISAF_I2C_OIS_per_read(0xF100);
		LOG_INF("ois init 0xF100= 0x%x\n",LC898129OISAF_I2C_OIS_per_read(0xF100));
		mdelay(10);
		i++;
	}while( (value != 0) && (i <= 5));
	
	//LC898129OISAF_I2C_OIS_per_write(0xF015, 0x00000004); /*Gyro sett*/
#if 0
	do{
		value = LC898129OISAF_I2C_OIS_per_read(0xF100);
		LOG_INF("ois init 0xF100= 0x%x\n",LC898129OISAF_I2C_OIS_per_read(0xF100));
		mdelay(10);
		i++;
	}while( (value != 0) && (i <= 5));
#endif	
	//LC898129OISAF_I2C_OIS_per_write(0xF012, 0x00000001);/*OIS ON*/

	LC898129OISAF_I2C_OIS_per_write(0xC000, 0x00d00100); /*Gyro sett*/
	LOG_INF("WHO AM I  = 0x%x \n", LC898129OISAF_I2C_OIS_per_read(0xD000));  /*0x00000145 chip id*/
	pr_info(" 0xF010 x:1 y:2 z:4 AF enble  = 0x%x \n", LC898129OISAF_I2C_OIS_per_read(0xF010));  /*0x0x000007 AF enble*/


}

static inline int setAFPara(__user struct stAF_MotorCmd *pstMotorCmd)
{
	struct stAF_MotorCmd stMotorCmd;

	if (copy_from_user(&stMotorCmd, pstMotorCmd, sizeof(stMotorCmd)))
		LOG_INF("copy to user failed when getting motor command\n");

	LOG_INF("Motor CmdID : %x\n", stMotorCmd.u4CmdID);

	LOG_INF("Motor Param : %x\n", stMotorCmd.u4Param);

	switch (stMotorCmd.u4CmdID) {
	case 1:
		LC898129OISAF_setOISMode(stMotorCmd.u4Param); /* 1 : disable */
		break;
	case 2:

		break;
	}

	return 0;
}

/* LC898129AF OIS CODE END*/


static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo,
			 sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

/* initAF include driver initialization and standby mode */
static int initAF(void)
{
	LOG_INF("+\n");

	LC898129OISAF_Main_OIS();
	LOG_INF("The device is opened\n");
	/*chenhan add for ois open*/
#ifdef CONFIG_MTK_CAM_PD2083F_EX
	ois_interface_dispatcher(AFIOC_X_OIS_INIT, NULL, OISDRV_LC89129);
#endif
	/*add end*/

	LOG_INF("-\n");

	return 0;
}

/* moveAF only use to control moving the motor */
static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	
	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); 
	if (LC898129OISAF_I2C_OIS_per_write(0xF01A, (unsigned long)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}
#if 0
	if (s4AF_WriteReg(1, 0x50, (unsigned short)a_u4Position) == 0) {
		g_u4CurrPosition = a_u4Position;
		ret = 0;
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
		ret = -1;
	}
#endif
	//mdelay(10);
	//LOG_INF("read af code  = %ld \n",LC898129OISAF_I2C_OIS_per_read(0xF01A) & 0x3FF); /* chip code */
	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long LC898129OISAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command,
		      unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue =
			getAFInfo((__user struct stAF_MotorInfo *)(a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;
	case AFIOC_S_SETPARA:
		i4RetValue =
			setAFPara((__user struct stAF_MotorCmd *)(a_u4Param));
		break;
	default:
		//LOG_INF("No CMD\n");
		/*chenhan add for ois per frame operation check*/
#ifdef CONFIG_MTK_CAM_PD2083F_EX
		i4RetValue = ois_interface_dispatcher(a_u4Command, (__user void *)(a_u4Param), OISDRV_LC89129);
#else
		i4RetValue = -EPERM;
#endif
		/*add end*/

		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int LC898129OISAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		
		LC898129OISAF_OIS_Standby();
		LOG_INF("Close\n");
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");
#ifdef CONFIG_MTK_CAM_PD2083F_EX
	ois_interface_dispatcher(AFIOC_X_OIS_DEINIT, NULL, OISDRV_LC89129);
#endif
	return 0;
}

int LC898129OISAF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient,
			    spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	initAF();
	

	return 1;
}

int LC898129OISAF_GetFileName(unsigned char *pFileName)
{
	#if SUPPORT_GETTING_LENS_FOLDER_NAME
	char FilePath[256];
	char *FileString;

	sprintf(FilePath, "%s", __FILE__);
	FileString = strrchr(FilePath, '/');
	*FileString = '\0';
	FileString = (strrchr(FilePath, '/') + 1);
	strncpy(pFileName, FileString, AF_MOTOR_NAME);
	LOG_INF("FileName : %s\n", pFileName);
	#else
	pFileName[0] = '\0';
	#endif
	return 1;
}
