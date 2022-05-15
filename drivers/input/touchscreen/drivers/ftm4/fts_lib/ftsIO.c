/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                     I2C / SPI Comunication				 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */



#include "ftsSoftware.h"
#include "ftsCrossCompile.h"

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
/*#include <linux/sec_sysfs.h>*/
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsTool.h"

static int ftm4_i2c_transfer(struct fts_ts_info *info, struct i2c_msg *msgs, int num)
{
	int ret;

	ret = i2c_transfer(info->client->adapter, msgs, num);
	if (ret < 0)
		vts_report_ic_exception(info->vtsdev, VTS_EXCEPTION_I2C_ERR, ret);

	return ret;
}

int ftm4_readCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;
	struct i2c_msg                I2CMsg[2];

	/*write msg */
	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;

	/*read msg */
	I2CMsg[1].addr = (__u16)info->client->addr;
	I2CMsg[1].flags = I2C_M_RD;
	I2CMsg[1].len = byteToRead;
	I2CMsg[1].buf = (__u8 *)outBuf;
	/*sunsl add */
	mutex_lock(&info->i2cResetMutex);
	while (retry < I2C_RETRY && ret < OK) {
		ret = ftm4_i2c_transfer(info, I2CMsg, 2);
		retry++;
		if (ret < OK)
			mdelay(I2C_WAIT_BEFORE_RETRY);
	}
	mutex_unlock(&info->i2cResetMutex);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}
	return OK;
}

int ftm4_writeCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int retry = 0;
	struct i2c_msg                I2CMsg[2];

	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;
	/*sunsl add */
	mutex_lock(&info->i2cResetMutex);
	while (retry < I2C_RETRY && ret < OK) {
		ret = ftm4_i2c_transfer(info, I2CMsg, 1);
		retry++;
		if (ret < OK)
			mdelay(I2C_WAIT_BEFORE_RETRY);
		/*fts_err(info, " ftm4_writeCmd: attempt %d\n", retry); */
	}
	mutex_unlock(&info->i2cResetMutex);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}
	return OK;
}

int ftm4_writeFwCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int ret2 = -1;
	int retry = 0;
	struct i2c_msg                I2CMsg[2];

	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;
	
	while (retry < I2C_RETRY && (ret < OK || ret2 < OK)) {
		mutex_lock(&info->i2cResetMutex);
		ret = ftm4_i2c_transfer(info, I2CMsg, 1);
		mutex_unlock(&info->i2cResetMutex);
		retry++;
		if (ret >= 0)
			ret2 = ftm4_checkEcho(info, cmd, cmdLength);
		if (ret < OK || ret2 < OK)
			mdelay(I2C_WAIT_BEFORE_RETRY);
	}
	
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}
	if (ret2 < OK) {
		fts_err(info, "check echo ERROR %02X\n", ret2);
		return (ret | ERROR_I2C_W);
	}
	return OK;
}


int ftm4_writeReadCmd(struct fts_ts_info *info, u8 *writeCmd1, int writeCmdLength, u8 *readCmd1, int readCmdLength, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;
	struct i2c_msg                I2CMsg[3];

	/*write msg */
	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)writeCmdLength;
	I2CMsg[0].buf = (__u8 *)writeCmd1;

	/*write msg */
	I2CMsg[1].addr = (__u16)info->client->addr;
	I2CMsg[1].flags = (__u16)0;
	I2CMsg[1].len = (__u16)readCmdLength;
	I2CMsg[1].buf = (__u8 *)readCmd1;

	/*read msg */
	I2CMsg[2].addr = (__u16)info->client->addr;
	I2CMsg[2].flags = I2C_M_RD;
	I2CMsg[2].len = byteToRead;
	I2CMsg[2].buf = (__u8 *)outBuf;

	/*sunsl add */
	mutex_lock(&info->i2cResetMutex);
	while (retry < I2C_RETRY && ret < OK) {
		ret = ftm4_i2c_transfer(info, I2CMsg, 3);
		retry++;
		if (ret < OK)
			mdelay(I2C_WAIT_BEFORE_RETRY);
	}
	mutex_unlock(&info->i2cResetMutex);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_WR);
			return ERROR_I2C_WR;
	}
	return OK;


}


int ftm4_readCmdU16(struct fts_ts_info *info, u8 cmd, u16 address, u8 *outBuf, int byteToRead, int hasDummyByte)
{

	int remaining = byteToRead;
	int toRead = 0;
	u8 rCmd[3] = { cmd, 0x00, 0x00 };

	u8 *buff = (u8 *)kmalloc((READ_CHUNK + 1) * sizeof(u8), GFP_KERNEL);
	if (buff == NULL) {
		fts_err(info, "ERROR %02X \n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}

		rCmd[1] = (u8)((address & 0xFF00) >> 8);
		rCmd[2] = (u8)(address & 0xFF);

		if (hasDummyByte) {
			if (ftm4_readCmd(info, rCmd, 3, buff, toRead + 1) < 0) {
				fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
				kfree(buff);
				return ERROR_I2C_R;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (ftm4_readCmd(info, rCmd, 3, buff, toRead) < 0)
				return ERROR_I2C_R;
			memcpy(outBuf, buff, toRead);
		}


		address += toRead;

		outBuf += toRead;



	}
	kfree(buff);

	return OK;
}


int ftm4_writeCmdU16(struct fts_ts_info *info, u8 WriteCmd, u16 address, u8 *dataToWrite, int byteToWrite)
{

	int remaining = byteToWrite;
	int toWrite = 0;

	u8 *buff = (u8 *)kmalloc((WRITE_CHUNK + 3) * sizeof(u8), GFP_KERNEL);
	if (buff == NULL) {
		fts_err(info, "ERROR %02X \n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	buff[0] = WriteCmd;


	while (remaining > 0) {
		if (remaining >= WRITE_CHUNK) {
			toWrite = WRITE_CHUNK;
			remaining -= WRITE_CHUNK;
		} else {
			toWrite = remaining;
			remaining = 0;
		}

		buff[1] = (u8)((address & 0xFF00) >> 8);
		buff[2] = (u8)(address & 0xFF);
		memcpy(buff + 3, dataToWrite, toWrite);
		if (ftm4_writeCmd(info, buff, 3 + toWrite) < 0) {
			fts_err(info, "ERROR %02x\n", ERROR_I2C_W);
			kfree(buff);
			return ERROR_I2C_W;
		}
		address += toWrite;
		dataToWrite += toWrite;

	}

	kfree(buff);
	return OK;
}





int ftm4_writeCmdU32(struct fts_ts_info *info, u8 writeCmd1, u8 writeCmd2, u32 address, u8 *dataToWrite, int byteToWrite)
{

	int remaining = byteToWrite;
	int toWrite = 0;
	int ret;

	u8 buff1[3] = { writeCmd1, 0x00, 0x00 };
	u8 *buff2 = (u8 *)kmalloc((WRITE_CHUNK + 3) * sizeof(u8), GFP_KERNEL);
	if (buff2 == NULL) {
		fts_err(info, "ERROR %02X \n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}
	buff2[0] = writeCmd2;


	while (remaining > 0) {
		if (remaining >= WRITE_CHUNK) {
			toWrite = WRITE_CHUNK;
			remaining -= WRITE_CHUNK;
		} else {
			toWrite = remaining;
			remaining = 0;
		}

		buff1[1] = (u8)((address & 0xFF000000) >> 24);
		buff1[2] = (u8)((address & 0x00FF0000) >> 16);
		buff2[1] = (u8)((address & 0x0000FF00) >> 8);
		buff2[2] = (u8)(address & 0xFF);
		memcpy(buff2 + 3, dataToWrite, toWrite);

		if (ftm4_writeCmd(info, buff1, 3) < 0) {
			fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
			ret = ERROR_I2C_W;
			goto END;
		}
		if (ftm4_writeCmd(info, buff2, 3 + toWrite) < 0) {
			fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
			ret = ERROR_I2C_W;
			goto END;
		}

		address += toWrite;
		dataToWrite += toWrite;

	}

	ret = OK;
END:
	kfree(buff2);
	return ret;
}


int ftm4_writeReadCmdU32(struct fts_ts_info *info, u8 wCmd, u8 rCmd, u32 address, u8 *outBuf, int byteToRead, int hasDummyByte)
{

	int remaining = byteToRead;
	int toRead = 0;
	u8 reaCmd[3];
	u8 wriCmd[3];


	u8 *buff = (u8 *)kmalloc((READ_CHUNK + 1) * sizeof(u8), GFP_KERNEL);
	if (buff == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	reaCmd[0] = rCmd;
	wriCmd[0] = wCmd;



	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}

		wriCmd[1] = (u8)((address & 0xFF000000) >> 24);
		wriCmd[2] = (u8)((address & 0x00FF0000) >> 16);

		reaCmd[1] = (u8)((address & 0x0000FF00) >> 8);
		reaCmd[2] = (u8)(address & 0x000000FF);

		if (hasDummyByte) {
			if (ftm4_writeReadCmd(info, wriCmd, 3, reaCmd, 3, buff, toRead + 1) < 0) {
				fts_err(info, "ERROR %02X \n", ERROR_I2C_WR);
				kfree(buff);
				return ERROR_I2C_WR;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (ftm4_writeReadCmd(info, wriCmd, 3, reaCmd, 3, buff, toRead) < 0)
				return ERROR_I2C_WR;
			memcpy(outBuf, buff, toRead);
		}


		address += toRead;

		outBuf += toRead;



	}

	kfree(buff);
	return OK;
}


