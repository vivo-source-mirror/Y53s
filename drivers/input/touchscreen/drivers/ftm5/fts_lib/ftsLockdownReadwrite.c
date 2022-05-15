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

int calculateCRC8_V2(u8* u8_srcBuff, int size, u8 *crc) {
	u8 u8_remainder; 
	u8 bit;
	int i=0;
	u8_remainder = 0x00;
	
	logError_ftm5(0, "%s %s: Start CRC computing...\n", tag_ftm5,__func__);
	if(size!=0 && u8_srcBuff!=NULL){
		// Perform modulo-2 division, a byte at a time. 
		for ( i = 0; i < size; i++) {//Bring the next byte into the remainder. 
			u8_remainder ^= u8_srcBuff[i]; //Perform modulo-2 division, a bit at a time. 		
			for (bit = 8; bit > 0; --bit) {		//Try to divide the current data bit. 
				if (u8_remainder & (0x1 << 7)){
					u8_remainder = (u8_remainder << 1) ^ 0x9B;
				} else {
					u8_remainder = (u8_remainder << 1);
				} 
			} 
		} // The final remainder is the CRC result.
		*crc = u8_remainder;
		logError_ftm5(0, "%s %s: CRC value = %02X\n", tag_ftm5,__func__, *crc);
		return OK;
	}else{
		logError_ftm5(1, "%s %s: Arguments passed not valid! Data pointer = NULL or size = 0 (%d) ERROR %08X\n", tag_ftm5,__func__,size, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}


int ftm5_writeLockDownInfo(struct fts_ts_info *info, u8 *data, int size, u8 lock_id)
{
	int ret,i;
	u8 crc_data=0;
	u8 crc_head=0;
	u8 cmd_lockdown_prepare[8]={LOCKDOWN_SIGNATURE,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 cmd_lockdown_crc[4]={0x00};
	u8 lockdown_save[3]={0xA4,0x00,0x04};
	char temp[100] = { 0 };
	u8 error_to_search[4] = {EVT_TYPE_ERROR_LOCKDOWN_FLASH,EVT_TYPE_ERROR_LOCKDOWN_CRC,EVT_TYPE_ERROR_LOCKDOWN_NO_DATA,EVT_TYPE_ERROR_LOCKDOWN_WRITE_FULL};
	
	logError_ftm5(0,"%s %s:enter",tag_ftm5,__func__);
	if(lock_id< 0x70 || lock_id > 0x77 || size <=0 || size > LOCKDOWN_LENGTH-20){
		logError_ftm5(1,"%s %s the lock id type is:%02X size:%d not support\n",tag_ftm5,__func__,lock_id,size);
		return ERROR_LOCKDOWN_CODE;
	}
	
	printHex_ftm5("Lockdown Code = ",data, size, temp);
	logError_ftm5(1,"%s: %s",  __func__, temp);

	logError_ftm5(0,"%s: Writing Lockdown code into the IC ...\n", __func__);
	fts_disableInterrupt_ftm5(info);
	for(i=0;i<3;i++){
		cmd_lockdown_prepare[1]= lock_id;
		ret = calculateCRC8_V2(data,size,&crc_data);
		if (ret < OK) {
			logError_ftm5(1, "%s %s: Unable to compute data CRC.. ERROR %08X\n", tag_ftm5,__func__, ret);
			ret = (ret | ERROR_LOCKDOWN_CODE);
			continue;
		}
		logError_ftm5(0, "%s %s: Get the data CRC value:%02X\n", tag_ftm5, __func__,crc_data);
		ret = ftm5_writeU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN,cmd_lockdown_prepare,ARRAY_SIZE(cmd_lockdown_prepare));
		if (ret < OK) {
			logError_ftm5(1,"%s %s: Unable to write Lockdown data prepare at %d iteration.. ERROR %08X\n",tag_ftm5, __func__,i, ret);
			ret = (ret | ERROR_LOCKDOWN_CODE);
			continue;
		}
		logError_ftm5(0, "%s %s: Compute 8bit header CRC...\n", tag_ftm5, __func__);

		cmd_lockdown_crc[0]= (u8)size;	
		cmd_lockdown_crc[1] = crc_data;
		cmd_lockdown_crc[2] = lock_id;
		ret = calculateCRC8_V2(cmd_lockdown_crc,3,&crc_head);
		if (ret < OK) {
			logError_ftm5(1, "%s %s: Unable to compute head CRC.. ERROR %08X\n", tag_ftm5,__func__, ret);
			ret = (ret | ERROR_LOCKDOWN_CODE);
			continue;
		}
		cmd_lockdown_crc[3] = crc_head;
		logError_ftm5(0, "%s %s: Get the header CRC value:%02X\n", tag_ftm5, __func__,crc_head);
		
		ret = ftm5_writeU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN+LOCKDOWN_DATA_OFFSET-LOCKDOWN_HEAD_LENGTH,
			cmd_lockdown_crc,ARRAY_SIZE(cmd_lockdown_crc));
		if (ret < OK) {
			logError_ftm5(1,"%s %s: Unable to write Lockdown  head at %d iteration.. ERROR %08X\n",tag_ftm5, __func__,i, ret);
			ret = (ret | ERROR_LOCKDOWN_CODE);
			continue;
		}
		mdelay(10);
		ret = ftm5_writeU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN+LOCKDOWN_DATA_OFFSET,data,size);
		if (ret < OK) {
			logError_ftm5(1,"%s %s: Unable to write Lockdown  head at %d iteration.. ERROR %08X\n",tag_ftm5, __func__,i, ret);
			ret = (ret | ERROR_LOCKDOWN_CODE);
			continue;
		}
		mdelay(10);
		ret = ftm5_write(info, lockdown_save,3);
		mdelay(5);
		ret = checkEcho_ftm5(info, lockdown_save,3);
		if(ret<OK){
			logError_ftm5(1, "%s No Echo received.. ERROR %08X !\n", tag_ftm5, ret);
			continue;
		}else{
			logError_ftm5(1, "%s Echo FOUND... OK!\n", tag_ftm5, ret);
			ret = pollForErrorType(info, error_to_search,4);
			if(ret<OK){
				logError_ftm5(1, "%s %s: No Error Found! \n", tag_ftm5, __func__);
				ret=OK;
			}else{
				logError_ftm5(1, "%s %s: have error when write lockdown ERROR = %02X\n", tag_ftm5, __func__, ret);
				ret = ERROR_LOCKDOWN_CODE;
			}
			break;
		}
	}
	if(ret<OK)
		logError_ftm5(1, "%s %s end, write lockdown failed\n", tag_ftm5,__func__, ret);
	else
		logError_ftm5(1, "%s %s end, write lockdown success\n", tag_ftm5,__func__, ret);
	
	fts_enableInterrupt_ftm5(info);
	return ret;
}


int ftm5_readLockDownInfo(struct fts_ts_info *info, u8 *lockData,u8 lock_id,int size)
{
	int ret =0,i;
	int loaded_cnt=0;
	int loaded_cnt_after=0;
	u8 *temp =NULL;
	char datatemp[100] = { 0 };
	u8 cmd_lockdown[3]={0xA4,0x06,0x00};

	logError_ftm5(0,"%s %s:enter",tag_ftm5,__func__);
	if(lock_id< 0x70 || lock_id > 0x77 || size <=0 || size > LOCKDOWN_LENGTH-20){
		logError_ftm5(1,"%s the lock id type is:%02X not support\n",tag_ftm5,lock_id);
		return ERROR_LOCKDOWN_CODE;
	}

	temp = (u8*)kmalloc(LOCKDOWN_LENGTH*sizeof(u8), GFP_KERNEL);
	if(temp==NULL){
		logError_ftm5(1,"FTS temp alloc  memory failed \n");
		return -ENOMEM;
	}
	memset(temp,0,LOCKDOWN_LENGTH*sizeof(u8));
	
	fts_disableInterrupt_ftm5(info);
	for(i=0;i<3;i++){
		ret = ftm5_writeReadU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN, temp,LOCKDOWN_HEAD_LENGTH, DUMMY_CONFIG);
		if ( ret < OK) {
			logError_ftm5(1,"%s %s: error while reading data ERROR %08X \n", tag_ftm5, __func__, ret);
			goto END;
		}
		loaded_cnt = (int)((temp[3] & 0xFF) << 8) + (temp[2] & 0xFF);
		cmd_lockdown[2] = lock_id;
		ftm5_write(info, cmd_lockdown,3);
		mdelay(10);
		ret = checkEcho_ftm5(info, cmd_lockdown,3);
		if(ret<OK){
			logError_ftm5(1, "%s No Echo received.. ERROR %08X !\n", tag_ftm5, ret);
			continue;
		}else{
			logError_ftm5(1, "%s Echo FOUND... OK!\n", tag_ftm5, ret);
		}
		ret = ftm5_writeReadU8UX(info, LOCKDOWN_WRITEREAD_CMD, BITS_16, ADDR_LOCKDOWN, temp, size+LOCKDOWN_DATA_OFFSET, DUMMY_CONFIG);
		if ( ret < OK) {
			logError_ftm5(1,"%s %s: error while reading data ERROR %08X \n", tag_ftm5, __func__, ret);
			goto END;
		}
		
		loaded_cnt_after = (int)((temp[3] & 0xFF) << 8) + (temp[2] & 0xFF);
		if(temp[4] == EVT_TYPE_ERROR_LOCKDOWN_FLASH || temp[4] == EVT_TYPE_ERROR_LOCKDOWN_NO_DATA){
			logError_ftm5(1,"%s %s: can not read the lockdown code ERROR type:%02X\n", tag_ftm5, __func__, temp[4]);
			ret = ERROR_LOCKDOWN_CODE;
			goto END;
		}
			
		logError_ftm5(1,"%s %s signature:%02X id:%02X %02X beforecnt:%d,aftercnt:%d\n", tag_ftm5, __func__,temp[0],temp[1],lock_id,loaded_cnt,loaded_cnt_after);
		if( loaded_cnt_after == loaded_cnt + 1){
			ret = OK;
			memcpy(lockData, &temp[LOCKDOWN_DATA_OFFSET], size);
			break;
		}
		
	}

	printHex_ftm5("Lockdown Code = ", lockData, size, datatemp);
	logError_ftm5(1,"%s %s", tag_ftm5,datatemp);
	 
END:
	
	fts_enableInterrupt_ftm5(info);
	kfree(temp);
	return ret;
}

