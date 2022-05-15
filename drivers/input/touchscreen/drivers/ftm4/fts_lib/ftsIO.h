/*

 **************************************************************************
 **                        STMicroelectronics 		                **
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

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_RETRY			3	/*number */
#define I2C_WAIT_BEFORE_RETRY		2 /*ms */

int ftm4_readCmd(struct fts_ts_info *info, u8 *cmd, int cmdLenght, u8 *outBuf, int byteToRead);
int ftm4_writeCmd(struct fts_ts_info *info, u8 *cmd, int cmdLenght);
int ftm4_writeFwCmd(struct fts_ts_info *info, u8 *cmd, int cmdLength);
int ftm4_writeReadCmd(struct fts_ts_info *info, u8 *writeCmd, int writeCmdLenght, u8 *readCmd, int readCmdLenght, u8 *outBuf, int byteToRead);
int ftm4_readCmdU16(struct fts_ts_info *info, u8 cmd, u16 address, u8 *outBuf, int byteToRead, int hasDummyByte);
int ftm4_writeCmdU16(struct fts_ts_info *info, u8 WriteCmd, u16 address, u8 *dataToWrite, int byteToWrite);
int ftm4_writeCmdU32(struct fts_ts_info *info, u8 writeCmd1, u8 writeCmd2, u32 address, u8 *dataToWrite, int byteToWrite);
int ftm4_writeReadCmdU32(struct fts_ts_info *info, u8 wCmd, u8 rCmd, u32 address, u8 *outBuf, int byteToRead, int hasDummyByte);
