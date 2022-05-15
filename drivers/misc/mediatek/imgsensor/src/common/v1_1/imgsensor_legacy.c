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

#include "kd_camera_typedef.h"
#include "imgsensor_i2c.h"

#ifdef IMGSENSOR_LEGACY_COMPAT
void kdSetI2CSpeed(u16 i2cSpeed)
{

}

int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData,
		u16 i2cId)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_read(
			imgsensor_i2c_get_device(),
			a_pSendData,
			a_sizeSendData,
			a_pRecvData,
			a_sizeRecvData,
			i2cId,
			IMGSENSOR_I2C_SPEED);
}

//IIC is grounded, the camera can be opend ,check iic timeout
    int check_i2c_timeout(u16 addr, u16 i2cid)
    {
    int ret, temp;
    u8 pu_send_cmd[2];
    u16 get_byte;
    struct IMGSENSOR_I2C_CFG *pdevice = imgsensor_i2c_get_device();
    if ( pdevice == NULL||pdevice->pinst->pi2c_client == NULL){
    	pr_err("[%s] check_i2c_timeout pdevice =null \n",__FUNCTION__);
        return IMGSENSOR_RETURN_ERROR;
    	}
    
    temp = pdevice->pinst->pi2c_client->adapter->timeout;
    pdevice->pinst->pi2c_client->adapter->timeout = HZ/20;

    if(addr&0xff00){
        get_byte = 2;
        pu_send_cmd[0] = (addr>> 8);
        pu_send_cmd[1] = (addr & 0xff); 
    } else {
        get_byte = 1;
        pu_send_cmd[0] = (addr & 0xff);
    }
    ret = iReadRegI2C(pu_send_cmd,get_byte,(u8*)&get_byte,1,i2cid);
    	//iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    pdevice->pinst->pi2c_client->adapter->timeout = temp;
	pr_info("[%s]  read get_byte = 0x%x  ret =%d \n",__FUNCTION__,get_byte,ret);
    return (ret==-2)?1:0;
    }
//IIC is grounded, the camera can be opend ,check iic timeout

int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData,
			u16 a_sizeRecvData, u16 i2cId, u16 timing)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_read(
			imgsensor_i2c_get_device(),
			a_pSendData,
			a_sizeSendData,
			a_pRecvData,
			a_sizeRecvData,
			i2cId,
			timing);
}

int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_write(
			imgsensor_i2c_get_device(),
			a_pSendData,
			a_sizeSendData,
			a_sizeSendData,
			i2cId,
			IMGSENSOR_I2C_SPEED);
}

int iWriteRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
			u16 i2cId, u16 timing)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_write(
			imgsensor_i2c_get_device(),
			a_pSendData,
			a_sizeSendData,
			a_sizeSendData,
			i2cId,
			timing);
}

int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_write(
			imgsensor_i2c_get_device(),
			pData,
			bytes,
			bytes,
			i2cId,
			IMGSENSOR_I2C_SPEED);
}

int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
				u16 transfer_length, u16 timing)
{
	if (imgsensor_i2c_get_device() == NULL)
		return IMGSENSOR_RETURN_ERROR;

	return imgsensor_i2c_write(
			imgsensor_i2c_get_device(),
			pData,
			bytes,
			transfer_length,
			i2cId,
			timing);
}

#endif
