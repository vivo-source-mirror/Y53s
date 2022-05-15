/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *               	FTS API for Flashing the IC			 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsCrossCompile.h"
#include "ftsCompensation.h"
#include "ftsError.h"
#include "ftsFlash.h"
#include "ftsFrame.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"					/*needed for including the define FW_H_FILE  */

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
#include <linux/time.h>
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

/*static int * firmware_updating_flag_ptr; */

/*
void initFtsFlash(int * flag) {
	//firmware_updating_flag_ptr = flag;
}
 */
int ftm4_getFirmwareVersion(struct fts_ts_info *info, u16 *fw_vers, u16 *config_id)
{
	u8 fwvers[DCHIP_FW_VER_BYTE];
	u8 confid[CONFIG_ID_BYTE];
	int res;

	res = ftm4_readCmdU16(info, FTS_CMD_HW_REG_R, DCHIP_FW_VER_ADDR, fwvers, DCHIP_FW_VER_BYTE, DUMMY_HW_REG);
	if (res < OK) {
		fts_err(info, "unable to read fw_version ERROR %02X\n", ERROR_FW_VER_READ);
		return (res | ERROR_FW_VER_READ);
	}

	fts_u8ToU16(fwvers, fw_vers); /*fw version use big endian */
	if (*fw_vers != 0) { /* if fw_version is 00 00 means that there is no firmware running in the chip therefore will be impossible find the config_id */
		res = ftm4_readB2(info, CONFIG_ID_ADDR, confid, CONFIG_ID_BYTE);
		if (res < OK) {
			fts_err(info, "unable to read config_id ERROR %02X\n", ERROR_FW_VER_READ);
			return (res | ERROR_FW_VER_READ);
		}
		fts_u8ToU16(confid, config_id); /*config id use little endian */
	} else {
		*config_id = 0x0000;
	}

	fts_err(info, "FW VERS = %04X\n", *fw_vers);
	fts_err(info, "CONFIG ID = %04X\n", *config_id);
	return OK;

}

int ftm4_getFWdata(struct fts_ts_info *info, const char *pathToFile, u8 **data, int *size, int from)
{
	fts_info(info, "Starting...");
	switch (from) {
	case 1:
		fts_info(info, "Read FW from .h file!");
		 //*size = FW_SIZE_NAME;
		 *data = (u8 *) kmalloc((*size) * sizeof (u8), GFP_KERNEL);
		if (*data == NULL) {
			fts_info(info, "Impossible to allocate memory! ERROR %08X", ERROR_ALLOC);
			return ERROR_ALLOC;
		}
		//memcpy(*data, (u8 *) FW_ARRAY_NAME, (*size));

		break;
	default:
		fts_info(info, "Read FW from BIN file!");
		break;
	}

	fts_info(info, "Finshed!");
	return OK;
}

int ftm4_readFwFile(struct fts_ts_info *info, const char *path, Firmware *fw, int keep_cx)
{
	int res = 0;
	int fw_size = 0;
	unsigned char *fw_data = NULL;
if (0) {
	res = ftm4_getFWdata(info, path, &fw_data, &fw_size, 1);
	if (res < OK) {
		fts_info(info, "impossible retrieve FW... ERROR %08X", ERROR_MEMH_READ);
		return (res | ERROR_MEMH_READ);
	}
}

	fw_data = vts_fw_data_get(info->vtsdev, VTS_FW_TYPE_FW, &fw_size);
	if (fw_data == NULL || fw_size == 0) {
		fts_info(info, "vivoTsGerFw fail");
		return (res | ERROR_MEMH_READ);
	}

	res = ftm4_parseBinFile(info, fw_data, fw_size, fw, keep_cx);
	if (res < OK) {
		fts_info(info, "ftm4_readFwFile: impossible parse ERROR %08X", ERROR_MEMH_READ);
		vts_fw_data_put(info->vtsdev, VTS_FW_TYPE_FW);
		return (res | ERROR_MEMH_READ);
	}

	vts_fw_data_put(info->vtsdev, VTS_FW_TYPE_FW);
	return OK;
}

int ftm4_flashProcedure(struct fts_ts_info *info, const char *path, int force, int keep_cx)
{
	Firmware fw;
	int res;

	fw.data = NULL;
	fts_info(info, "Reading Fw file...");
	res = ftm4_readFwFile(info, path, &fw, keep_cx);
	if (res < OK) {
		fts_info(info, "ERROR %02X", (res | ERROR_FLASH_PROCEDURE));
		kfree(fw.data);
		return (res | ERROR_FLASH_PROCEDURE);
	}
	fts_info(info, "Fw file read COMPLETED!");

	fts_info(info, "Starting flashing procedure...");
	res = ftm4_flash_burn(info, fw, force, keep_cx);
	if (res < OK && res != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
		fts_info(info, "ERROR %02X", ERROR_FLASH_PROCEDURE);
		kfree(fw.data);
		return (res | ERROR_FLASH_PROCEDURE);
	}
	fts_info(info, "flashing procedure Finished!");
	kfree(fw.data);

	return res;
}

int ftm4_wait_for_flash_ready(struct fts_ts_info *info, u8 type)
{
	u8 cmd[2] = {FLASH_CMD_READ_REGISTER, type};
	u8 readData;
	int i, res = -1;

	fts_err(info, "Waiting for flash ready ... \n");
	for (i = 0; i < FLASH_RETRY_COUNT && res != 0; i++) {
		if (ftm4_readCmd(info, cmd, sizeof (cmd), &readData, 1) < 0) {
			fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		} else {
			res = readData & 0x80;
		/*fts_err(info, " flash status = %d  \n", res); */
	}
		mdelay(FLASH_WAIT_BEFORE_RETRY);
	}

	if (i == FLASH_RETRY_COUNT && res != 0) {
		fts_err(info, "Wait for flash TIMEOUT! ERROR %02X \n", ERROR_TIMEOUT);
		return ERROR_TIMEOUT;
	}

	fts_err(info, "Flash READY! \n");
	return OK;
}

int ftm4_warm_boot(struct fts_ts_info *info)
{

	u8 cmd[4] = {FTS_CMD_HW_REG_W, 0x00, 0x00, WARM_BOOT_VALUE}; /*write the command to perform the warm boot											 */
	ftm4_u16ToU8_be(ADDR_WARM_BOOT, &cmd[1]);

	fts_err(info, "Command warm boot ... \n");
	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	fts_err(info, "Warm boot DONE! \n");

	return OK;
}

int ftm4_parseBinFile(struct fts_ts_info *info, const u8 *data, int fw_size, Firmware *fwData, int keep_cx)
{

	int dimension, index = 0;
	u32 temp;
	int res, i;

	/*the file should contain at least the header plus the content_crc */
	if (fw_size < FW_HEADER_SIZE + FW_BYTES_ALLIGN || data == NULL) {
		fts_info(info, "Read only %d instead of %d... ERROR %02X", fw_size, FW_HEADER_SIZE + FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
		res = ERROR_FILE_PARSE;
		goto END;
	} else {
		/*start parsing of bytes */
		ftm4_u8ToU32(&data[index], &temp);
		if (temp != FW_HEADER_SIGNATURE) {
			fts_info(info, "Wrong Signature %08X ... ERROR %02X", temp, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		fts_info(info, "Fw Signature OK!");
		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		if (temp != FW_FTB_VER) {
			fts_info(info, "Wrong ftb_version %08X ... ERROR %02X", temp, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		fts_info(info, "ftb_version OK!");
		index += FW_BYTES_ALLIGN;
		if (data[index] != DCHIP_ID_0 || data[index + 1] != DCHIP_ID_1) {
			fts_info(info, "Wrong target %02X != %02X  %02X != %02X ... ERROR %08X", data[index], DCHIP_ID_0, data[index + 1], DCHIP_ID_1, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}
		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fts_info(info, "Fw ID = %08X", temp);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fwData->fw_ver = temp;
		fts_info(info, "FILE Fw Version = %04X", fwData->fw_ver);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fwData->config_id = temp;
		fts_info(info, "FILE Config ID = %08X", temp);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fts_info(info, "Config Version = %08X", temp);

		index += FW_BYTES_ALLIGN * 2;			/*skip reserved data */

		index += FW_BYTES_ALLIGN;
		fts_info(info, "File External Release");
		for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++) {
			fwData->externalRelease[i] = data[index++];
			fts_info(info, "%02X ", fwData->externalRelease[i]);
		}

		/*index += FW_BYTES_ALLIGN; */
		ftm4_u8ToU32(&data[index], &temp);
		fwData->sec0_size = temp;
		fts_info(info, "sec0_size = %08X (%d bytes)", fwData->sec0_size, fwData->sec0_size);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fwData->sec1_size = temp;
		fts_info(info, "sec1_size = %08X (%d bytes)", fwData->sec1_size, fwData->sec1_size);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fwData->sec2_size = temp;
		fts_info(info, "sec2_size = %08X (%d bytes)", fwData->sec2_size, fwData->sec2_size);

		index += FW_BYTES_ALLIGN;
		ftm4_u8ToU32(&data[index], &temp);
		fwData->sec3_size = temp;
		fts_info(info, "sec3_size = %08X (%d bytes)", fwData->sec3_size, fwData->sec3_size);

		index += FW_BYTES_ALLIGN;		/* skip header crc */

		if (!keep_cx) {
			dimension = fwData->sec0_size + fwData->sec1_size + fwData->sec2_size + fwData->sec3_size;
			temp = fw_size;
		} else {
			dimension = fwData->sec0_size + fwData->sec1_size;		/* sec2 may contain cx data (future implementation) sec3 atm not used */
			temp = fw_size - fwData->sec2_size - fwData->sec3_size;
		}

		if (dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN != temp) {
			fts_info(info, "Read only %d instead of %d... ERROR %02X", fw_size, dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
			res = ERROR_FILE_PARSE;
			goto END;
		}

		fwData->data = (u8 *) kmalloc(dimension * sizeof (u8), GFP_KERNEL);
		if (fwData->data == NULL) {
			fts_info(info, "ERROR %02X", ERROR_ALLOC);
			res = ERROR_ALLOC;
			goto END;
		}

		index += FW_BYTES_ALLIGN;
		memcpy(fwData->data, &data[index], dimension);
		fwData->data_size = dimension;

		fts_info(info, "READ FW DONE %d bytes!", fwData->data_size);
		res = OK;
		goto END;
	}

END:
	return res;
}

int ftm4_flash_unlock(struct fts_ts_info *info)
{
	u8 cmd[3] = {FLASH_CMD_UNLOCK, FLASH_UNLOCK_CODE0, FLASH_UNLOCK_CODE1}; /*write the command to perform the unlock									 */

	fts_err(info, "Command unlock ... \n");
	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	/*mdelay(FLASH_WAIT_TIME); */
	fts_err(info, "Unlock flash DONE! \n");

	return OK;

}

int ftm4_flash_erase_unlock(struct fts_ts_info *info)
{
	u8 cmd[3] = {FLASH_CMD_WRITE_REGISTER, FLASH_ERASE_UNLOCK_CODE0, FLASH_ERASE_UNLOCK_CODE1}; /*write the command to perform the unlock for erasing the flash */

	fts_err(info, "Try to erase unlock flash... \n");

	fts_err(info, "Command erase unlock ... \n");
	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	fts_err(info, "Erase Unlock flash DONE! \n");

	return OK;

}

int ftm4_flash_full_erase(struct fts_ts_info *info)
{

	int status;
	u8 cmd[3] = {FLASH_CMD_WRITE_REGISTER, FLASH_ERASE_CODE0, FLASH_ERASE_CODE1}; /*write the command to erase the flash */


	fts_err(info, "Command full erase sent ... \n");
	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	status = ftm4_wait_for_flash_ready(info, FLASH_ERASE_CODE0);

	if (status != OK) {
		fts_err(info, "ERROR %02X\n", ERROR_FLASH_NOT_READY);
		return (status | ERROR_FLASH_NOT_READY); /*Flash not ready within the chosen time, better exit! */
	}

	fts_err(info, "Full Erase flash DONE! \n");

	return OK;

}

int ftm4_start_flash_dma(struct fts_ts_info *info)
{
	int status;
	u8 cmd[3] = {FLASH_CMD_WRITE_REGISTER, FLASH_DMA_CODE0, FLASH_DMA_CODE1}; /*write the command to erase the flash */


	fts_err(info, "Command flash DMA ... \n");
	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	status = ftm4_wait_for_flash_ready(info, FLASH_DMA_CODE0);

	if (status != OK) {
		fts_err(info, "ERROR %02X\n", ERROR_FLASH_NOT_READY);
		return (status | ERROR_FLASH_NOT_READY); /*Flash not ready within the chosen time, better exit! */
	}

	fts_err(info, "flash DMA DONE! \n");

	return OK;
}

int ftm4_fillFlash(struct fts_ts_info *info, u32 address, u8 *data, int size)
{
	int remaining = size;
	int toWrite = 0;
	int byteBlock = 0;
	int wheel = 0;
	u32 addr = 0;
	int res;
	int delta;
	u8 *buff = NULL;
	u8 buff2[9] = {0};

	buff = (u8 *) kmalloc((DMA_CHUNK + 3) * sizeof (u8), GFP_KERNEL);
	if (buff == NULL) {
		fts_info(info, "kmalloc fail:%02X", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	while (remaining > 0) {
		byteBlock = 0;
		addr = 0;
		while (byteBlock < FLASH_CHUNK && remaining > 0) {
			fts_info(info, "write flash info:byteBlock=%d, remaining=%d", byteBlock, remaining);
			buff[0] = FLASH_CMD_WRITE_64K;
			if (remaining >= DMA_CHUNK) {
				if ((byteBlock + DMA_CHUNK) <= FLASH_CHUNK) {
					/*fts_err(info, " ftm4_fillFlash: 1\n"); */
					toWrite = DMA_CHUNK;
					remaining -= DMA_CHUNK;
					byteBlock += DMA_CHUNK;
				} else {
					/*fts_err(info, " ftm4_fillFlash: 2\n"); */
					delta = FLASH_CHUNK - byteBlock;
					toWrite = delta;
					remaining -= delta;
					byteBlock += delta;
				}
			} else {
				if ((byteBlock + remaining) <= FLASH_CHUNK) {
					/*fts_err(info, " ftm4_fillFlash: 3\n"); */
					toWrite = remaining;
					byteBlock += remaining;
					remaining = 0;

				} else {
					/*fts_err(info, " ftm4_fillFlash: 4\n"); */
					delta = FLASH_CHUNK - byteBlock;
					toWrite = delta;
					remaining -= delta;
					byteBlock += delta;
				}
			}


			buff[1] = (u8) ((addr & 0x0000FF00) >> 8);
			buff[2] = (u8) (addr & 0x000000FF);
			memcpy(&buff[3], data, toWrite);
			/*fts_err(info, " Command = %02X , address = %02X %02X, bytes = %d, data =  %02X %02X, %02X %02X \n", buff[0], buff[1], buff[2], toWrite, buff[3], buff[4], buff[3 + toWrite -2], buff[3 + toWrite -1]); */
			if (ftm4_writeCmd(info, buff, 3 + toWrite) < 0) {
				fts_info(info, "write cmd error:%02X", ERROR_I2C_W);
				kfree(buff);
				return ERROR_I2C_W;
			}
			addr += toWrite;
			data += toWrite;
		}

		/*configuring the DMA */
		byteBlock = byteBlock / 4 - 1;

		buff2[0] = FLASH_CMD_WRITE_REGISTER;
		buff2[1] = FLASH_DMA_CONFIG;
		buff2[2] = 0x00;
		buff2[3] = 0x00;

		addr = address + ((wheel * FLASH_CHUNK) / 4);
		buff2[4] = (u8) ((addr & 0x000000FF));
		buff2[5] = (u8) ((addr & 0x0000FF00) >> 8);
		buff2[6] = (u8) (byteBlock & 0x000000FF);
		buff2[7] = (u8) ((byteBlock & 0x0000FF00) >> 8);
		buff2[8] = 0x00;

		fts_info(info, "Command = %02X , address = %02X %02X, words =  %02X %02X", buff2[0], buff2[5], buff2[4], buff2[7], buff2[6]);
		if (ftm4_writeCmd(info, buff2, 9) < OK) {
			fts_info(info, "Error during filling Flash! ERROR %02X", ERROR_I2C_W);
			kfree(buff);
			return ERROR_I2C_W;
		}

		/*mdelay(FLASH_WAIT_TIME); */
		res = ftm4_start_flash_dma(info);
		if (res < OK) {
			fts_info(info, "Error during flashing DMA! ERROR %02X", res);
			kfree(buff);
			return res;
		}
		wheel++;
	}
	kfree(buff);
	return OK;
}

static int flash_erase_page_by_page(struct fts_ts_info *info, int keep_cx) {

    u8 status,i=0;
    u8 cmd[4] = {FLASH_CMD_WRITE_REGISTER, FLASH_ERASE_CODE0, 0x00, 0x00}; //write the command to erase the flash

    for(i=0; i<FLASH_NUM_PAGE; i++){
		if (i == GOLDEN_PANEL_INIT_PAGE) {
			fts_err(info, "Skipping panel init erase page %d! \n",i);
			continue;
		}
		
		if(i>=FLASH_CX_PAGE_START && i<=FLASH_CX_PAGE_END && keep_cx==1){
			fts_err(info, "Skipping erase page %d! \n",i);
			continue;
		}

		cmd[2] = (0x3F&i)|FLASH_ERASE_START;
		fts_err(info, "Command erase page %d sent ... %02X %02X %02X %02X \n",i, cmd[0],cmd[1],cmd[2],cmd[3]);
	    	if (ftm4_writeCmd(info, cmd, sizeof (cmd)) < 0) {
	        	fts_err(info, "ERROR %08X\n", ERROR_I2C_W);
	        	return ERROR_I2C_W;
	    	}

	    	status = ftm4_wait_for_flash_ready(info, FLASH_ERASE_CODE0);

	    	if (status != OK) {

	        	fts_err(info, "ERROR %08X\n", ERROR_FLASH_NOT_READY);
	        	return (status | ERROR_FLASH_NOT_READY); //Flash not ready within the chosen time, better exit!
	    	}

     }
    	
     fts_err(info, "Erase flash page by page DONE! \n");

    return OK;

}
int ftm4_flash_burn(struct fts_ts_info *info, Firmware fw, int force_burn, int keep_cx)
{
	int res;

	if (!force_burn) {
		for (res = EXTERNAL_RELEASE_INFO_SIZE - 1; res >= 0; res--) {
			fts_info(info, "header fw externalReleaseInfo[%d]=0x%02X,chip extReleaseInfo[%d]=0x%02X", res, fw.externalRelease[res], res, info->chipinfo.u8_extReleaseInfo[res]);
			if (fw.externalRelease[res] != info->chipinfo.u8_extReleaseInfo[res])
				goto start;
		}
		fts_info(info, "Firmware in the chip newer or equal to the one to burn! NO UPDATE ERROR %02X", ERROR_FW_NO_UPDATE);
		return (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED);
	}

	/*programming procedure start */
start:
	fts_info(info, "Programming Procedure for flashing starting...");

	fts_info(info, "1.system reset");
	res = ftm4_system_reset(info);
	if (res < 0) {
		fts_info(info, "system reset failed");
		if (res != (ERROR_SYSTEM_RESET_FAIL | ERROR_TIMEOUT)) {/*if there is no firmware i will not get the controller ready event and there will be a timeout but i can keep going, but if there is an I2C error i have to exit */
			return (res | ERROR_FLASH_BURN_FAILED);
		}
	} else {
		fts_info(info, "system reset COMPLETED!");
	}

	fts_info(info, "WARM BOOT");
	res = ftm4_warm_boot(info);
	if (res < OK) {
		fts_info(info, "warm boot FAILED!");
		return (res | ERROR_FLASH_BURN_FAILED);
	} else {
		fts_info(info, "warm boot COMPLETED!");
	}

	/*mdelay(FLASH_WAIT_TIME); */
	fts_info(info, "3.FLASH UNLOCK");
	res = ftm4_flash_unlock(info);
	if (res < OK) {
		fts_info(info, "flash unlock FAILED! ERROR %02X", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	} else {
		fts_info(info, "flash unlock COMPLETED!");
	}

   /*mdelay(200); */
	fts_info(info, "4.FLASH ERASE UNLOCK");
	res = ftm4_flash_erase_unlock(info);
	if (res < 0) {
		fts_info(info, "flash unlock FAILED! ERROR %02X", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	} else {
		fts_info(info, "flash unlock COMPLETED!");
	}

	/*mdelay(FLASH_WAIT_TIME); */
	fts_info(info, "5.FLASH ERASE");
	if (keep_cx == 0) {
	res = ftm4_flash_full_erase(info);
		if (res < 0) {
			fts_info(info, "flash erase FAILED! ERROR %02X", ERROR_FLASH_BURN_FAILED);
			return (res | ERROR_FLASH_BURN_FAILED);
		} else {
			fts_info(info, "flash erase COMPLETED!");
	    }
	} else if (keep_cx == 1){
		res = flash_erase_page_by_page(info, 1);
		if (res < 0) {
			fts_info(info, "flash_erase_page_by_page FAILED! ERROR %02X", ERROR_FLASH_BURN_FAILED);
	        return (res | ERROR_FLASH_BURN_FAILED);
	    } else {
			fts_info(info, "flash_erase_page_by_page COMPLETED!");
	    }
	}

	/*mdelay(FLASH_WAIT_TIME); */
	fts_info(info, "6.LOAD PROGRAM");
	res = ftm4_fillFlash(info, FLASH_ADDR_CODE, &fw.data[0], fw.sec0_size);
	if (res < OK) {
		fts_info(info, "load program ERROR %02X", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	fts_info(info, "load program DONE!");

	fts_info(info, "7.LOAD CONFIG");
	res = ftm4_fillFlash(info, FLASH_ADDR_CONFIG, &(fw.data[fw.sec0_size]), fw.sec1_size);
	if (res < OK) {
		fts_info(info, "load config ERROR %02X", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	fts_info(info, "load config DONE!");

	fts_info(info, "Flash burn COMPLETED!");

	fts_info(info, "8.SYSTEM RESET");
	res = ftm4_system_reset(info);
	if (res < 0) {
		fts_info(info, "system reset FAILED! ERROR %02X", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}
	fts_info(info, "system reset COMPLETED!");

	fts_info(info, "9.FINAL CHECK");
	res = fts_readChipInfo(info, 0);
	if (res < 0) {
		fts_info(info, "Unable to retrieve Chip INFO! ERROR %02x", ERROR_FLASH_BURN_FAILED);
		return (res | ERROR_FLASH_BURN_FAILED);
	}

	for (res = 0; res < EXTERNAL_RELEASE_INFO_SIZE; res++) {
		if (fw.externalRelease[res] != info->chipinfo.u8_extReleaseInfo[res]) { /*external release is prined during readChipInfo */
				fts_info(info, "Firmware in the chip different from the one that was burn! fw: %x != %x , conf: %x != %x", info->chipinfo.u16_fwVer, fw.fw_ver, info->chipinfo.u16_cfgId, fw.config_id);
				return ERROR_FLASH_BURN_FAILED;
		}
	}

	fts_info(info, "Final check OK! fw: %02X , conf: %02X \n", info->chipinfo.u16_fwVer, info->chipinfo.u16_cfgId);

	return OK;
}
