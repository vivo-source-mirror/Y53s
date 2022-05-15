/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     I2C/SPI Communication				  *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsIO.c
  * \brief Contains all the functions which handle with the I2C/SPI
  *communication
  */


#include "ftsSoftware.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

#include "ftsCore.h"
#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"

/****************** New I2C API *********************/

/**
  * Perform a direct bus read
  * @param outBuf pointer of a byte array which should contain the byte read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int ftm5_read(struct fts_ts_info *info, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;

#ifdef I2C_INTERFACE
	struct i2c_msg I2CMsg[1];

	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)I2C_M_RD;
	I2CMsg[0].len = (__u16)byteToRead;
	I2CMsg[0].buf = (__u8 *)outBuf;
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	spi_message_init(&msg);

	transfer[0].len = byteToRead;
	transfer[0].delay_usecs = SPI_DELAY_CS;
	transfer[0].tx_buf = NULL;
	transfer[0].rx_buf = outBuf;
	spi_message_add_tail(&transfer[0], &msg);
#endif

	while (retry < I2C_RETRY && ret < OK) {
#ifdef I2C_INTERFACE
		ret = i2c_transfer(info->client->adapter, I2CMsg, 1);
#else
		ret = spi_sync(info->client, &msg);
#endif

		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError_ftm5(1,"%s fts_writeCmd: attempt %d\n", tag_ftm5, retry); */
	}
	if (ret < 0) {
		logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5, __func__, ERROR_BUS_R);
		return ERROR_BUS_R;
	}
	return OK;
}


/**
  * Perform a bus write followed by a bus read without a stop condition
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @param outBuf pointer of a byte array which should contain the bytes read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int ftm5_writeRead(struct fts_ts_info *info, u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;

#ifdef I2C_INTERFACE
	struct i2c_msg I2CMsg[2];

	/* write msg */
	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;

	/* read msg */
	I2CMsg[1].addr = (__u16)info->client->addr;
	I2CMsg[1].flags = I2C_M_RD;
	I2CMsg[1].len = byteToRead;
	I2CMsg[1].buf = (__u8 *)outBuf;

#else
	struct spi_message msg;
	struct spi_transfer transfer[2] = { { 0 }, { 0 } };

	spi_message_init(&msg);

	transfer[0].len = cmdLength;
	transfer[0].tx_buf = cmd;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);

	transfer[1].len = byteToRead;
	transfer[1].delay_usecs = SPI_DELAY_CS;
	transfer[1].tx_buf = NULL;
	transfer[1].rx_buf = outBuf;
	spi_message_add_tail(&transfer[1], &msg);

#endif

	while (retry < I2C_RETRY && ret < OK) {
#ifdef I2C_INTERFACE
		ret = i2c_transfer(info->client->adapter, I2CMsg, 2);
#else
		ret = spi_sync(info->client, &msg);
#endif

		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
	}
	if (ret < 0) {
		logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5, __func__, ERROR_BUS_WR);
		return ERROR_BUS_WR;
	}
	return OK;
}


/**
  * Perform a bus write
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @return OK if success or an error code which specify the type of error
  */
int ftm5_write(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int retry = 0;

#ifdef I2C_INTERFACE
	struct i2c_msg I2CMsg[1];

	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	spi_message_init(&msg);

	transfer[0].len = cmdLength;
	transfer[0].delay_usecs = SPI_DELAY_CS;
	transfer[0].tx_buf = cmd;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);
#endif

	while (retry < I2C_RETRY && ret < OK) {
#ifdef I2C_INTERFACE
		ret = i2c_transfer(info->client->adapter, I2CMsg, 1);
#else
		ret = spi_sync(info->client, &msg);
#endif

		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError_ftm5(1,"%s fts_writeCmd: attempt %d\n", tag_ftm5, retry); */
	}
	if (ret < 0) {
		logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5, __func__, ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	return OK;
}

/**
  * Write a FW command to the IC and check automatically the echo event
  * @param cmd byte array containing the command to send
  * @param cmdLength size of cmd
  * @return OK if success, or an error code which specify the type of error
  */
int ftm5_writeFwCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength)
{
	int ret = -1;
	int ret2 = -1;
	int retry = 0;

#ifdef I2C_INTERFACE
	struct i2c_msg I2CMsg[1];

	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)cmdLength;
	I2CMsg[0].buf = (__u8 *)cmd;
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };

	spi_message_init(&msg);

	transfer[0].len = cmdLength;
	transfer[0].delay_usecs = SPI_DELAY_CS;
	transfer[0].tx_buf = cmd;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);
#endif
	resetErrorList(info);
	while (retry < I2C_RETRY && (ret < OK || ret2 < OK)) {
#ifdef I2C_INTERFACE
		ret = i2c_transfer(info->client->adapter, I2CMsg, 1);
#else
		ret = spi_sync(info->client, &msg);
#endif
		retry++;
		if (ret >= 0)
			ret2 = checkEcho_ftm5(info, cmd, cmdLength);
		if (ret < OK || ret2 < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
		/* logError_ftm5(1,"%s fts_writeCmd: attempt %d\n", tag_ftm5, retry); */
	}
	if (ret < 0) {
		logError_ftm5(1, "%s ftm5_writeFwCmd: ERROR %08X\n", tag_ftm5,
			 ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	if (ret2 < OK) {
		logError_ftm5(1, "%s ftm5_writeFwCmd: check echo ERROR %08X\n", tag_ftm5,
			 ret2);
		return ret2;
	}
	return OK;
}


/**
  * Perform two bus write and one bus read without any stop condition
  * In case of FTI this function is not supported and the same sequence
  * can be achieved calling ftm5_writeRead followed by an fts_writeRead.
  * @param writeCmd1 byte array containing the first command to write
  * @param writeCmdLength size of writeCmd1
  * @param readCmd1 byte array containing the second command to write
  * @param readCmdLength size of readCmd1
  * @param outBuf pointer of a byte array which should contain the bytes read
  * from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int ftm5_writeThenWriteRead(struct fts_ts_info *info, u8 *writeCmd1, int writeCmdLength, u8 *readCmd1, int
			   readCmdLength, u8 *outBuf, int byteToRead)
{
	int ret = -1;
	int retry = 0;

#ifdef I2C_INTERFACE
	struct i2c_msg I2CMsg[3];

	/* write msg */
	I2CMsg[0].addr = (__u16)info->client->addr;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)writeCmdLength;
	I2CMsg[0].buf = (__u8 *)writeCmd1;

	/* write msg */
	I2CMsg[1].addr = (__u16)info->client->addr;
	I2CMsg[1].flags = (__u16)0;
	I2CMsg[1].len = (__u16)readCmdLength;
	I2CMsg[1].buf = (__u8 *)readCmd1;

	/* read msg */
	I2CMsg[2].addr = (__u16)info->client->addr;
	I2CMsg[2].flags = I2C_M_RD;
	I2CMsg[2].len = byteToRead;
	I2CMsg[2].buf = (__u8 *)outBuf;
#else
	struct spi_message msg;
	struct spi_transfer transfer[3] = { { 0 }, { 0 }, { 0 } };

	spi_message_init(&msg);

	transfer[0].len = writeCmdLength;
	transfer[0].tx_buf = writeCmd1;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);

	transfer[1].len = readCmdLength;
	transfer[1].tx_buf = readCmd1;
	transfer[1].rx_buf = NULL;
	spi_message_add_tail(&transfer[1], &msg);

	transfer[2].len = byteToRead;
	transfer[2].delay_usecs = SPI_DELAY_CS;
	transfer[2].tx_buf = NULL;
	transfer[2].rx_buf = outBuf;
	spi_message_add_tail(&transfer[2], &msg);
#endif

	while (retry < I2C_RETRY && ret < OK) {
#ifdef I2C_INTERFACE
		ret = i2c_transfer(info->client->adapter, I2CMsg, 3);
#else
		ret = spi_sync(info->client, &msg);
#endif
		retry++;
		if (ret < OK)
			msleep(I2C_WAIT_BEFORE_RETRY);
	}

	if (ret < 0) {
		logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5, __func__, ERROR_BUS_WR);
		return ERROR_BUS_WR;
	}
	return OK;
}



/**
  * Perform a chunked write with one byte op code and 1 to 8 bytes address
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param data pointer of a byte array which contain the bytes to write
  * @param dataSize size of data
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the address is max 8 bytes */
int ftm5_writeU8UX(struct fts_ts_info *info, u8 cmd, AddrSize addrSize, u64 address, u8 *data, int
		  dataSize)
{
	u8 finalCmd[1 + 8 + WRITE_CHUNK];//addrSize max  8
	int remaining = dataSize;
	int toWrite = 0, i = 0;

	if (addrSize <= sizeof(u64)) {
		while (remaining > 0) {
			if (remaining >= WRITE_CHUNK) {
				toWrite = WRITE_CHUNK;
				remaining -= WRITE_CHUNK;
			} else {
				toWrite = remaining;
				remaining = 0;
			}

			finalCmd[0] = cmd;
			logError_ftm5(0, "%s %s: addrSize = %d\n", tag_ftm5, __func__,
				 addrSize);
			for (i = 0; i < addrSize; i++) {
				finalCmd[i + 1] = (u8)((address >> ((addrSize -
								     1 - i) *
								    8)) & 0xFF);
				logError_ftm5(1, "%s %s: cmd[%d] = %02X\n", tag_ftm5,
					 __func__, i + 1, finalCmd[i + 1]);
			}

			memcpy(&finalCmd[addrSize + 1], data, toWrite);

			if (ftm5_write(info, finalCmd, 1 + addrSize + toWrite) < OK) {
				logError_ftm5(1, "%s %s: ERROR %08X\n", tag_ftm5,
					 __func__, ERROR_BUS_W);
				return ERROR_BUS_W;
			}

			address += toWrite;

			data += toWrite;
		}
	} else
		logError_ftm5(1,
			 "%s %s: address size bigger than max allowed %ld... ERROR %08X\n",
			 tag_ftm5, __func__, sizeof(u64), ERROR_OP_NOT_ALLOW);

	return OK;
}

/**
  * Perform a chunked write read with one byte op code and 1 to 8 bytes address
  * and dummy byte support.
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped)
  * set to 1, otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
int ftm5_writeReadU8UX(struct fts_ts_info *info, u8 cmd, AddrSize addrSize, u64 address, u8 *outBuf, int
		      byteToRead, int hasDummyByte)
{
	u8 finalCmd[1 + 8];//
	u8 buff[READ_CHUNK + 1];/* worst case has dummy byte */
	int remaining = byteToRead;
	int toRead = 0, i = 0;

	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}

		finalCmd[0] = cmd;
		for (i = 0; i < addrSize; i++)
			finalCmd[i + 1] = (u8)((address >> ((addrSize - 1 - i) *
							    8)) & 0xFF);

		if (hasDummyByte == 1) {
			if (ftm5_writeRead(info, finalCmd, 1 + addrSize, buff, toRead +
					  1) < OK) {
				logError_ftm5(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag_ftm5,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (ftm5_writeRead(info, finalCmd, 1 + addrSize, buff,
					  toRead) < OK) {
				logError_ftm5(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag_ftm5,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff, toRead);
		}

		address += toRead;

		outBuf += toRead;
	}

	return OK;
}

/**
  * Perform a chunked write followed by a second write with one byte op code
  * for each write and 1 to 8 bytes address (the sum of the 2 address size of
  * the two writes can not exceed 8 bytes)
  * @param cmd1 byte containing the op code of first write
  * @param addrSize1 address size in byte of first write
  * @param cmd2 byte containing the op code of second write
  * @param addrSize2 address size in byte of second write
  * @param address the starting address
  * @param data pointer of a byte array which contain the bytes to write
  * @param dataSize size of data
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes */
int ftm5_writeU8UXthenWriteU8UX(struct fts_ts_info *info, u8 cmd1, AddrSize addrSize1, u8 cmd2, AddrSize
			       addrSize2, u64 address, u8 *data, int dataSize)
{
	u8 finalCmd1[1 + 8];//addrSize1
	u8 finalCmd2[1 + 8 + WRITE_CHUNK];
	int remaining = dataSize;
	int toWrite = 0, i = 0;

	while (remaining > 0) {
		if (remaining >= WRITE_CHUNK) {
			toWrite = WRITE_CHUNK;
			remaining -= WRITE_CHUNK;
		} else {
			toWrite = remaining;
			remaining = 0;
		}

		finalCmd1[0] = cmd1;
		for (i = 0; i < addrSize1; i++)
			finalCmd1[i + 1] = (u8)((address >> ((addrSize1 +
							      addrSize2 - 1 -
							      i) * 8)) & 0xFF);

		finalCmd2[0] = cmd2;
		for (i = addrSize1; i < addrSize1 + addrSize2; i++)
			finalCmd2[i - addrSize1 + 1] = (u8)((address >>
							     ((addrSize1 +
							       addrSize2 - 1 -
							       i) * 8)) & 0xFF);

		memcpy(&finalCmd2[addrSize2 + 1], data, toWrite);

		if (ftm5_write(info, finalCmd1, 1 + addrSize1) < OK) {
			logError_ftm5(1, "%s %s: first write error... ERROR %08X\n",
				 tag_ftm5, __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		if (ftm5_write(info, finalCmd2, 1 + addrSize2 + toWrite) < OK) {
			logError_ftm5(1,
				 "%s %s: second write error... ERROR %08X\n",
				 tag_ftm5,
				 __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		address += toWrite;

		data += toWrite;
	}

	return OK;
}

/**
  * Perform a chunked write  followed by a write read with one byte op code
  * and 1 to 8 bytes address for each write and dummy byte support.
  * @param cmd1 byte containing the op code of first write
  * @param addrSize1 address size in byte of first write
  * @param cmd2 byte containing the op code of second write read
  * @param addrSize2 address size in byte of second write	read
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped) set to 1,
  *  otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes */
int ftm5_writeU8UXthenWriteReadU8UX(struct fts_ts_info *info, u8 cmd1, AddrSize addrSize1, u8 cmd2,
				   AddrSize addrSize2, u64 address, u8 *outBuf,
				   int byteToRead, int hasDummyByte)
{
	u8 finalCmd1[1 + 8];//
	u8 finalCmd2[1 + 8];
	u8 buff[READ_CHUNK + 1];/* worst case has dummy byte */
	int remaining = byteToRead;
	int toRead = 0, i = 0;


	while (remaining > 0) {
		if (remaining >= READ_CHUNK) {
			toRead = READ_CHUNK;
			remaining -= READ_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}


		finalCmd1[0] = cmd1;
		for (i = 0; i < addrSize1; i++)
			finalCmd1[i + 1] = (u8)((address >> ((addrSize1 +
							      addrSize2 - 1 -
							      i) * 8)) & 0xFF);
		/* logError_ftm5(1, "%s %s: finalCmd1[%d] =  %02X\n",
		  *	tag_ftm5, __func__, i+1, finalCmd1[i + 1]); */

		finalCmd2[0] = cmd2;
		for (i = addrSize1; i < addrSize1 + addrSize2; i++)
			finalCmd2[i - addrSize1 + 1] = (u8)((address >>
							     ((addrSize1 +
							       addrSize2 - 1 -
							       i) * 8)) & 0xFF);

		if (ftm5_write(info, finalCmd1, 1 + addrSize1) < OK) {
			logError_ftm5(1, "%s %s: first write error... ERROR %08X\n",
				 tag_ftm5, __func__, ERROR_BUS_W);
			return ERROR_BUS_W;
		}

		if (hasDummyByte == 1) {
			if (ftm5_writeRead(info, finalCmd2, 1 + addrSize2, buff,
					  toRead + 1) < OK) {
				logError_ftm5(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag_ftm5,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff + 1, toRead);
		} else {
			if (ftm5_writeRead(info, finalCmd2, 1 + addrSize2, buff,
					  toRead) < OK) {
				logError_ftm5(1,
					 "%s %s: read error... ERROR %08X\n",
					 tag_ftm5,
					 __func__, ERROR_BUS_WR);
				return ERROR_BUS_WR;
			}
			memcpy(outBuf, buff, toRead);
		}

		address += toRead;

		outBuf += toRead;
	}

	return OK;
}
