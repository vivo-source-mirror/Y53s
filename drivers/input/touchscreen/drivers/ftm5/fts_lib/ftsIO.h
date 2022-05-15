/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     I2C/SPI Communication				*
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */
/*!
  * \file ftsIO.h
  * \brief Contains all the definitions and prototypes used and implemented in
  * ftsIO.c
  */

#ifndef FTS_IO_H
#define FTS_IO_H

#include "ftsSoftware.h"

#define I2C_RETRY		3	/* /< number of retry in case of i2c
					 * failure */
#define I2C_WAIT_BEFORE_RETRY	2	/* /< wait in ms before retry an i2c
					 * transaction */
int ftm5_read(struct fts_ts_info *info, u8 *outBuf, int byteToRead);
int ftm5_writeRead(struct fts_ts_info *info, u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead);
int ftm5_write(struct fts_ts_info *info, u8 *cmd, int cmdLength);
int ftm5_writeFwCmd(struct fts_ts_info *info, u8 *cmd, int cmdLenght);
int ftm5_writeThenWriteRead(struct fts_ts_info *info, u8 *writeCmd1, int writeCmdLength, u8 *readCmd1, int
			   readCmdLength, u8 *outBuf, int byteToRead);
int ftm5_writeU8UX(struct fts_ts_info *info, u8 cmd, AddrSize addrSize, u64 address, u8 *data, int
		  dataSize);
int ftm5_writeReadU8UX(struct fts_ts_info *info, u8 cmd, AddrSize addrSize, u64 address, u8 *outBuf, int
		      byteToRead, int hasDummyByte);
int ftm5_writeU8UXthenWriteU8UX(struct fts_ts_info *info, u8 cmd1, AddrSize addrSize1, u8 cmd2, AddrSize
			       addrSize2, u64 address, u8 *data, int dataSize);
int ftm5_writeU8UXthenWriteReadU8UX(struct fts_ts_info *info, u8 cmd1, AddrSize addrSize1, u8 cmd2,
				   AddrSize addrSize2, u64 address, u8 *outBuf,
				   int count, int hasDummyByte);
#endif
