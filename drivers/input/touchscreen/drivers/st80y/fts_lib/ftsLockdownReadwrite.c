#include <linux/delay.h>

#include "../fts.h"
#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsIO.h"
#include "ftsError.h"
#include "ftsFlash.h"
#include "ftsFrame.h"
#include "ftsGesture.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"

#define LOCKDOWN_LENGTH				384	
#define LOCKDOWN_HEAD_LENGTH			4
#define LOCKDOWN_DATA_OFFSET			20
#define LOCKDOWN_SIGNATURE			0x5A

//REMAP REGISTER
#define ADDR_LOCKDOWN				(u64)0x0000000000000000	
#define LOCKDOWN_WRITEREAD_CMD			0xA6
#define EVT_TYPE_ERROR_LOCKDOWN_FLASH		0x30			///< Error in record(s) in flash,FW shall not proceed with any flash write/read 
#define EVT_TYPE_ERROR_LOCKDOWN_CRC		0x31		///< CRC error in record,FW shall discard the record and do not write to flash 
#define EVT_TYPE_ERROR_LOCKDOWN_NO_DATA		0x32			///< No data of this type exisitng in flash,No data of this type exisitng in flash 
#define EVT_TYPE_ERROR_LOCKDOWN_WRITE_FULL	0x33			///< Not enough space in flash for new record,FW shall not write this new record to flash 
#define ERROR_LOCKDOWN_CODE			(int)0x80001600		//unable to write/rewrite/read lockdown code in the IC
#define WRITE_LOCKDOWN_HEAD_SIZE 7
#define WRITE_LOCKDOWN_MAX_TIMES 15


static int calculateCRC8_V2(u8* u8_srcBuff, int size, u8 *crc) {
	u8 u8_remainder; 
	u8 bit;
	int i=0;
	u8_remainder = 0x00;
	
	logError_st80y(0, "%s %s: Start CRC computing...\n", tag_st80y,__func__);
	if (size!=0 && u8_srcBuff!=NULL) {
		// Perform modulo-2 division, a byte at a time. 
		for ( i = 0; i < size; i++) {//Bring the next byte into the remainder. 
			u8_remainder ^= u8_srcBuff[i]; //Perform modulo-2 division, a bit at a time. 		
			for (bit = 8; bit > 0; --bit) {		//Try to divide the current data bit. 
				if (u8_remainder & (0x1 << 7)) {
					u8_remainder = (u8_remainder << 1) ^ 0x9B;
				} else {
					u8_remainder = (u8_remainder << 1);
				} 
			} 
		} // The final remainder is the CRC result.
		*crc = u8_remainder;
		logError_st80y(0, "%s %s: CRC value = %02X\n", tag_st80y,__func__, *crc);
		return OK;
	}else{
		logError_st80y(1, "%s %s: Arguments passed not valid! Data pointer = NULL or size = 0 (%d) ERROR %08X\n", tag_st80y,__func__,size, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}


int st80y_writeLockDownInfo(struct fts_ts_info *info, u8 *data, int size, u8 lock_id)
{
	int ret = 0;
	char write_cmd[WRITE_LOCKDOWN_HEAD_SIZE] = {0xA6,0x00,0x00,0x5A,0x00,0x00,0x00};//A6 00 00 5A host_data_mem_id 00 00
	char write_cmd_head[WRITE_LOCKDOWN_HEAD_SIZE] = {0xA6,0x00,0x10,0x00,0x00,0x00,0x00};//A6 00 10 payload_len payload_crc rec_id hdr_crc
	char write_flash_cmd[3] = {0xA4, 0x00, 0x04 };
	char *write_buf = NULL;
	if (info->flash_write_times > WRITE_LOCKDOWN_MAX_TIMES)
	{	
		logError_st80y(1,"FTS flash write times ex \n");
		return -1;
	}
	write_cmd[4] = lock_id;
	write_buf = (char *)kzalloc((WRITE_LOCKDOWN_HEAD_SIZE + size)*sizeof(char), GFP_KERNEL);
	if (write_buf == NULL) {
		logError_st80y(1,"FTS temp alloc memory failed \n");
		return -ENOMEM;
	}
	
	st80y_disableInterrupt(info);

	st80y_write(info, write_cmd, WRITE_LOCKDOWN_HEAD_SIZE);//A6 00 00 5A host_data_mem_id 00 00
	mdelay(10);
	write_cmd_head[3] = size;
	calculateCRC8_V2(data, size, &write_cmd_head[4]);//payload_crc
	write_cmd_head[5] = lock_id;
	calculateCRC8_V2(write_cmd_head+3, 3, &write_cmd_head[6]);//hdr_crc
	
	memcpy(write_buf, write_cmd_head, WRITE_LOCKDOWN_HEAD_SIZE);
	memcpy(write_buf + WRITE_LOCKDOWN_HEAD_SIZE, data, size);
	st80y_write(info, write_buf, WRITE_LOCKDOWN_HEAD_SIZE+size);
	mdelay(10);
	st80y_write(info, write_flash_cmd, 3);//A4 00 04
	mdelay(10);
	ret = st80y_checkEcho(info, write_flash_cmd,3);
	if(ret < OK) 
		logError_st80y(1, "%s No Echo received.. ERROR %08X !\n", tag_st80y, ret);
	else
		logError_st80y(1, "%s Echo FOUND... OK!\n", tag_st80y, ret);
	
	st80y_enableInterrupt(info);
	kfree(write_buf);

	return ret;
}


int st80y_readLockDownInfo(struct fts_ts_info *info, u8 *lockData,u8 lock_id,int size)
{
	int ret = 0;
	u8 *temp = NULL;
	char datatemp[100] = { 0 };
	u8 cmd_lockdown[3]={0xA4, 0x06, 0x00};

	logError_st80y(0, "%s %s:enter", tag_st80y, __func__);
	if (lock_id < 0x70 || lock_id > 0x77 || size <= 0 || size > LOCKDOWN_LENGTH-20) {
		logError_st80y(1,"%s the lock id type is:%02X not support\n", tag_st80y, lock_id);
		return ERROR_LOCKDOWN_CODE;
	}

	temp = (u8*)kmalloc(LOCKDOWN_LENGTH*sizeof(u8), GFP_KERNEL);
	if (temp == NULL) {
		logError_st80y(1,"FTS temp alloc  memory failed \n");
		return -ENOMEM;
	}
	memset(temp, 0, LOCKDOWN_LENGTH*sizeof(u8));
	
	st80y_disableInterrupt(info);
	
	cmd_lockdown[2] = lock_id;
	st80y_write(info, cmd_lockdown, 3);
	mdelay(10);
	ret = st80y_checkEcho(info, cmd_lockdown, 3);
	if (ret < OK)
		logError_st80y(1, "%s No Echo received.. ERROR %08X !\n", tag_st80y, ret);
	else
		logError_st80y(1, "%s Echo FOUND... OK!\n", tag_st80y);
	
	
	ret = st80y_writeReadU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN, temp, size+LOCKDOWN_DATA_OFFSET, DUMMY_CONFIG);
	if (ret < OK) {
		logError_st80y(1,"%s %s: error while reading data ERROR %08X \n", tag_st80y, __func__, ret);
		goto END;
	}
	info->flash_write_times = (temp[5] & 0xFF);
	if (temp[4] == EVT_TYPE_ERROR_LOCKDOWN_FLASH) {
		logError_st80y(1,"%s %s: There is flash memory error in record, ERROR type:%02X\n", tag_st80y, __func__, temp[4]);
		ret = 0;
		goto END;
	}
	if (temp[4] == EVT_TYPE_ERROR_LOCKDOWN_NO_DATA) {
		logError_st80y(1,"%s %s: Lockdown code read no data, ERROR type:%02X\n", tag_st80y, __func__, temp[4]);
		ret = 0;
		goto END;
	}
		
	logError_st80y(1,"%s %s signature:%02X id:%02X %02X write cnt:%d\n", tag_st80y, __func__, temp[0], temp[1], lock_id, info->flash_write_times);	
	memcpy(lockData, &temp[LOCKDOWN_DATA_OFFSET], size);	
	
	logError_st80y(1,"%s %s", tag_st80y, st80y_printHex("Lockdown Code = ", lockData, size, datatemp));
	 
END:
	
	st80y_enableInterrupt(info);
	kfree(temp);
	return ret;
}

