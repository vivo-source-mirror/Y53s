/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *               FTS functions for getting Initialization Data		 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "../fts.h"
#include "ftsCrossCompile.h"
#include "ftsCompensation.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTool.h"

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

int ftm4_requestCompensationData(struct fts_ts_info *info, u16 type)
{
	int retry = 0;
	int ret;
	char *temp = NULL;
	u16 answer;

	int event_to_search[3];
	u8 readEvent[FIFO_EVENT_SIZE];

	u8 cmd[3] = { FTS_CMD_REQU_COMP_DATA, 0x00, 0x00 };							/* B8 is the command for asking compensation data */
	ftm4_u16ToU8(type, &cmd[1]);

	event_to_search[0] = (int)EVENTID_COMP_DATA_READ;
	event_to_search[1] = cmd[1];
	event_to_search[2] = cmd[2];

	while (retry < COMP_DATA_READ_RETRY) {
		temp =  ftm4_printHex("Command = ", cmd, 3);
		if (temp != NULL)
			fts_err(info, "%s", temp);
		kfree(temp);
		ret = ftm4_writeFwCmd(info, cmd, 3);										/*send the request to the chip to load in memory the Compensation Data */
		if (ret < OK) {
			fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
			return ERROR_I2C_W;
		}
		ret = ftm4_pollForEvent(info, event_to_search, 3, readEvent, TIMEOUT_REQU_COMP_DATA);
		if (ret < OK) {
			fts_err(info, "Event did not Found at %d attemp! \n", retry + 1);
			retry += 1;
		} else {
			retry = 0;
			break;
		}
	}


	if (retry == COMP_DATA_READ_RETRY) {
		 fts_err(info, "ERROR %02X\n", ERROR_TIMEOUT);
		 return ERROR_TIMEOUT;
	}

	fts_u8ToU16_le(&readEvent[1], &answer);

	if (answer == type)
		return OK;
	else {
		fts_err(info, "The event found has a different type of Compensation data ERROR %02X \n", ERROR_DIFF_COMP_TYPE);
		return ERROR_DIFF_COMP_TYPE;
	}

}


int ftm4_readCompensationDataHeader(struct fts_ts_info *info, u16 type, DataHeader *header, u16 *address)
{

	u16 offset = ADDR_FRAMEBUFFER_DATA;
	u16 answer;
	u8 data[COMP_DATA_HEADER];


	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, offset, data, COMP_DATA_HEADER, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}
	fts_err(info, "Read Data Header done! \n");

	if (data[0] != HEADER_SIGNATURE) {
		fts_err(info, "ERROR %02X The Header Signature was wrong!  %02X != %02X \n", ERROR_WRONG_COMP_SIGN, data[0], HEADER_SIGNATURE);
		return ERROR_WRONG_COMP_SIGN;
	}


	fts_u8ToU16_le(&data[1], &answer);


	if (answer != type) {
		fts_err(info, "ERROR %02X\n", ERROR_DIFF_COMP_TYPE);
		return ERROR_DIFF_COMP_TYPE;
	}

	fts_err(info, "Type of Compensation data OK! \n");

	header->type = type;
	header->force_node = (int)data[4];
	header->sense_node = (int)data[5];

	 *address = offset + COMP_DATA_HEADER;

	return OK;

}

int ftm4_readMutualSenseGlobalData(struct fts_ts_info *info, u16 *address, MutualSenseData *global)
{

	u8 data[COMP_DATA_GLOBAL];

	fts_err(info, "Address for Global data= %02X \n", *address);

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, *address, data, COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}
	fts_err(info, "Global data Read !\n");

	global->tuning_ver = data[0];
	global->cx1 = data[1];

	fts_err(info, "tuning_ver = %d   CX1 = %d \n", global->tuning_ver, global->cx1);

	 *address += COMP_DATA_GLOBAL;
	return OK;

}

int ftm4_readMutualSenseNodeData(struct fts_ts_info *info, u16 address, MutualSenseData *node)
{

	int size = node->header.force_node * node->header.sense_node;

	fts_err(info, "Address for Node data = %02X \n", address);

	node->node_data = (u8 *)kmalloc(size * (sizeof(u8)), GFP_KERNEL);

	if (node->node_data == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	fts_err(info, "Node Data to read %d bytes \n", size);

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, address, node->node_data, size, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		kfree(node->node_data);
		return ERROR_I2C_R;
	}
	node->node_data_size = size;

	fts_err(info, "Read node data ok! \n");

	return size;

}


int ftm4_readMutualSenseCompensationData(struct fts_ts_info *info, u16 type, MutualSenseData *data)
{

	int ret;
	u16 address;
	data->node_data = NULL;

	if (!(type == MS_TOUCH_ACTIVE || type == MS_TOUCH_LOW_POWER || type == MS_TOUCH_ULTRA_LOW_POWER || type == MS_KEY)) {
		fts_err(info, "Choose a MS type of compensation data ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_requestCompensationData(info, type);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_REQU_COMP_DATA);
		return (ret | ERROR_REQU_COMP_DATA);
	}

	ret = ftm4_readCompensationDataHeader(info, type, &(data->header), &address);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return (ret | ERROR_COMP_DATA_HEADER);
	}

	ret = ftm4_readMutualSenseGlobalData(info, &address, data);
	if (ret < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_COMP_DATA_GLOBAL);
		return (ret | ERROR_COMP_DATA_GLOBAL);
	}

	ret = ftm4_readMutualSenseNodeData(info, address, data);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_NODE);
		return (ret | ERROR_COMP_DATA_NODE);
	}

	return OK;

}


int ftm4_readSelfSenseGlobalData(struct fts_ts_info *info, u16 *address, SelfSenseData *global)
{

	u8 data[COMP_DATA_GLOBAL];

	fts_err(info, "Address for Global data= %02X \n", *address);

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, *address, data, COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}

	fts_err(info, "Global data Read !\n");

	global->tuning_ver = data[0];
	global->f_ix1 = data[1];
	global->s_ix1 = data[2];
	global->f_cx1 = data[3];
	global->s_cx1 = data[4];
	global->f_max_n = data[5];
	global->s_max_n = data[6];

	fts_err(info, "tuning_ver = %d   f_ix1 = %d   s_ix1 = %d   f_cx1 = %d   s_cx1 = %d \n", global->tuning_ver, global->f_ix1, global->s_ix1, global->f_cx1, global->s_cx1);
	fts_err(info, "max_n = %d   s_max_n = %d \n", global->f_max_n, global->s_max_n);


	 *address += COMP_DATA_GLOBAL;

	return OK;

}

int ftm4_readSelfSenseNodeData(struct fts_ts_info *info, u16 address, SelfSenseData *node)
{

	int size = node->header.force_node * 2 + node->header.sense_node * 2;
	u8 *data =NULL;
	data = (u8 *)kmalloc(size * (sizeof(u8)), GFP_KERNEL);
   	if (data == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		return ERROR_ALLOC;
	}
	node->ix2_fm = (u8 *)kmalloc(node->header.force_node * (sizeof(u8)), GFP_KERNEL);
	if (node->ix2_fm == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		kfree(data);
		return ERROR_ALLOC;
	}

	node->cx2_fm = (u8 *)kmalloc(node->header.force_node * (sizeof(u8)), GFP_KERNEL);
	if (node->cx2_fm == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->ix2_sn = (u8 *)kmalloc(node->header.sense_node * (sizeof(u8)), GFP_KERNEL);
	if (node->ix2_sn == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->cx2_sn = (u8 *)kmalloc(node->header.sense_node * (sizeof(u8)), GFP_KERNEL);
	if (node->cx2_sn == NULL) {
		fts_err(info, "ERROR %02X", ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(node->ix2_sn);
		kfree(data);
		return ERROR_ALLOC;
	}


	fts_err(info, "Address for Node data = %02X \n", address);

	fts_err(info, "Node Data to read %d bytes \n", size);

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, address, data, size, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_R);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(node->ix2_sn);
		kfree(node->cx2_sn);
		kfree(data);
		return ERROR_I2C_R;
	}

	fts_err(info, "Read node data ok! \n");

	memcpy(node->ix2_fm, data, node->header.force_node);
	memcpy(node->ix2_sn, &data[node->header.force_node], node->header.sense_node);
	memcpy(node->cx2_fm, &data[node->header.force_node + node->header.sense_node], node->header.force_node);
	memcpy(node->cx2_sn, &data[node->header.force_node * 2 + node->header.sense_node], node->header.sense_node);

	kfree(data);
	return OK;

}

int ftm4_readSelfSenseCompensationData(struct fts_ts_info *info, u16 type, SelfSenseData *data)
{

	int ret;
	u16 address;

	data->ix2_fm = NULL;
	data->cx2_fm = NULL;
	data->ix2_sn = NULL;
	data->cx2_sn = NULL;

	if (!(type == SS_TOUCH || type == SS_KEY || type == SS_HOVER || type == SS_PROXIMITY)) {
		fts_err(info, "Choose a SS type of compensation data ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_requestCompensationData(info, type);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_REQU_COMP_DATA);
		return (ret | ERROR_REQU_COMP_DATA);
	}

	ret = ftm4_readCompensationDataHeader(info, type, &(data->header), &address);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return (ret | ERROR_COMP_DATA_HEADER);
	}

	ret = ftm4_readSelfSenseGlobalData(info, &address, data);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_GLOBAL);
		return (ret | ERROR_COMP_DATA_GLOBAL);
	}

	ret = ftm4_readSelfSenseNodeData(info, address, data);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_NODE);
		return (ret | ERROR_COMP_DATA_NODE);
	}

	return OK;

}


int ftm4_readGeneralGlobalData(struct fts_ts_info *info, u16 address, GeneralData *global)
{
	u8 data[COMP_DATA_GLOBAL];

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, address, data, COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}

	global->ftsd_lp_timer_cal0 = data[0];
	global->ftsd_lp_timer_cal1 = data[1];
	global->ftsd_lp_timer_cal2 = data[2];
	global->ftsd_lp_timer_cal3 = data[3];
	global->ftsa_lp_timer_cal0 = data[4];
	global->ftsa_lp_timer_cal1 = data[5];

	return OK;

}

int ftm4_readGeneralCompensationData(struct fts_ts_info *info, u16 type, GeneralData *data)
{

	int ret;
	u16 address;

	if (!(type == GENERAL_TUNING)) {
		fts_err(info, "Choose a GENERAL type of compensation data ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_requestCompensationData(info, type);
	if (ret < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_REQU_COMP_DATA);
		return ERROR_REQU_COMP_DATA;
	}

	ret = ftm4_readCompensationDataHeader(info, type, &(data->header), &address);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return ERROR_COMP_DATA_HEADER;
	}

	ret = ftm4_readGeneralGlobalData(info, address, data);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_GLOBAL);
		return ERROR_COMP_DATA_GLOBAL;
	}

	return OK;

}


int fts_defaultChipInfo(struct fts_ts_info *info, int i2cError)
{
	int i;
	fts_info(info, "Setting default Chip Info... \n");
	info->chipinfo.u32_echoEn = 0x00000000;
	info->chipinfo.u8_msScrConfigTuneVer = 0;
	info->chipinfo.u8_ssTchConfigTuneVer = 0;
	info->chipinfo.u8_msScrCxmemTuneVer = 0;
	info->chipinfo.u8_ssTchCxmemTuneVer = 0;
	if (i2cError == 1) {
		info->chipinfo.u16_fwVer = 0xFFFF;
		info->chipinfo.u16_cfgId = 0xFFFF;
		for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++) {
			info->chipinfo.u8_extReleaseInfo[i] = 0xFF;
		}
	} else {
		info->chipinfo.u16_fwVer = 0x0000;
		info->chipinfo.u16_cfgId = 0x0000;
		for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++) {
			info->chipinfo.u8_extReleaseInfo[i] = 0x00;
		}
	}
	info->chipinfo.u32_mpPassFlag = INIT_FIELD;
	fts_info(info, "default Chip Info DONE! \n");
	return OK;

}

int fts_readChipInfo(struct fts_ts_info *info, int doRequest)
{
	int ret, i;
	u16 answer;
	u8 data[CHIP_INFO_SIZE + 3];				/* + 3 because need to read all the field of the struct plus the signature and 2 address bytes */
	int index = 0;

	fts_info(info, "Starting Read Chip Info... \n");
	if (doRequest == 1) {
		ret = ftm4_requestCompensationData(info, CHIP_INFO);
		if (ret < 0) {
			fts_err(info, "readChipInfo: ERROR %02X\n", ERROR_REQU_COMP_DATA);
			ret = (ret | ERROR_REQU_COMP_DATA);
			goto FAIL;
		}
	}

	fts_info(info, "Byte to read = %d bytes \n", CHIP_INFO_SIZE + 3);

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, ADDR_FRAMEBUFFER_DATA, data, CHIP_INFO_SIZE + 3, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "readChipInfo: ERROR %02X\n", ERROR_I2C_R);
		ret = ERROR_I2C_R;
		goto FAIL;
	}

	fts_info(info, "Read data ok! \n");

	fts_info(info, "Starting parsing of data... \n");

	if (data[0] != HEADER_SIGNATURE) {
		fts_err(info, "readChipInfo: ERROR %02X The Header Signature was wrong!  %02X != %02X \n", ERROR_WRONG_COMP_SIGN, data[0], HEADER_SIGNATURE);
		ret = ERROR_WRONG_COMP_SIGN;
		goto FAIL;
	}


	fts_u8ToU16_le(&data[1], &answer);


	if (answer != CHIP_INFO) {
		fts_err(info, "readChipInfo:  ERROR %02X\n", ERROR_DIFF_COMP_TYPE);
		ret = ERROR_DIFF_COMP_TYPE;
		goto FAIL;
	}

	index += 3;
	info->chipinfo.u8_loadCnt = data[index++];
	info->chipinfo.u8_infoVer = data[index++];
	fts_u8ToU16(&data[index], &info->chipinfo.u16_ftsdId);
	index += 2;
	info->chipinfo.u8_ftsdVer = data[index++];
	info->chipinfo.u8_ftsaId = data[index++];
	info->chipinfo.u8_ftsaVer = data[index++];
	info->chipinfo.u8_tchRptVer = data[index++];

	fts_info(info, "External Release =  ");
	for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++) {
		info->chipinfo.u8_extReleaseInfo[i] = data[index++];
		fts_info(info, "%02X ", info->chipinfo.u8_extReleaseInfo[i]);
	}
	fts_info(info, "\n");

	for (i = 0; i < sizeof(info->chipinfo.u8_custInfo); i++) {
		info->chipinfo.u8_custInfo[i] = data[index++];
	}

	fts_u8ToU16(&data[index], &info->chipinfo.u16_fwVer);
	index += 2;
	fts_info(info, "FW VERSION = %04X \n", info->chipinfo.u16_fwVer);

	fts_u8ToU16(&data[index], &info->chipinfo.u16_cfgId);
	index += 2;
	fts_info(info, "CONFIG ID = %04X \n", info->chipinfo.u16_cfgId);

	info->chipinfo.u32_projId = ((data[index + 3] & 0x000000FF) << 24) + ((data[index + 2] & 0x000000FF) << 16) + ((data[index + 1] & 0x000000FF) << 8) + (data[index] & 0x000000FF);
	index += 4;

	fts_u8ToU16(&data[index], &info->chipinfo.u16_scrXRes);
	index += 2;

	fts_u8ToU16(&data[index], &info->chipinfo.u16_scrYRes);
	index += 2;

	info->chipinfo.u8_scrForceLen = data[index++];
	fts_info(info, "Force Len = %d \n", info->chipinfo.u8_scrForceLen);

	info->chipinfo.u8_scrSenseLen = data[index++];
	fts_info(info, "Sense Len = %d \n", info->chipinfo.u8_scrSenseLen);

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_scrForceEn[i] = data[index++];
	}

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_scrSenseEn[i] = data[index++];
	}

	info->chipinfo.u8_msKeyLen = data[index++];
	fts_info(info, "MS Key Len = %d \n", info->chipinfo.u8_msKeyLen);

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_msKeyForceEn[i] = data[index++];
	}

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_msKeySenseEn[i] = data[index++];
	}

	info->chipinfo.u8_ssKeyLen = data[index++];
	fts_info(info, "SS Key Len = %d \n", info->chipinfo.u8_ssKeyLen);

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_ssKeyForceEn[i] = data[index++];
	}

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_ssKeySenseEn[i] = data[index++];
	}

	info->chipinfo.u8_frcTchXLen = data[index++];

	info->chipinfo.u8_frcTchYLen = data[index++];

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_frcTchForceEn[i] = data[index++];
	}

	for (i = 0; i < 8; i++) {
		info->chipinfo.u64_frcTchSenseEn[i] = data[index++];
	}


	info->chipinfo.u8_msScrConfigTuneVer = data[index++];
	fts_info(info, "CFG MS TUNING VERSION = %02X \n", info->chipinfo.u8_msScrConfigTuneVer);
	info->chipinfo.u8_msScrLpConfigTuneVer = data[index++];
	info->chipinfo.u8_msScrHwulpConfigTuneVer = data[index++];
	info->chipinfo.u8_msKeyConfigTuneVer = data[index++];
	info->chipinfo.u8_ssTchConfigTuneVer = data[index++];
	fts_info(info, "CFG SS TUNING VERSION = %02X \n", info->chipinfo.u8_ssTchConfigTuneVer);
	info->chipinfo.u8_ssKeyConfigTuneVer = data[index++];
	info->chipinfo.u8_ssHvrConfigTuneVer = data[index++];
	info->chipinfo.u8_frcTchConfigTuneVer = data[index++];
	info->chipinfo.u8_msScrCxmemTuneVer = data[index++];
	fts_info(info, "CX MS TUNING VERSION = %02X \n", info->chipinfo.u8_msScrCxmemTuneVer);
	info->chipinfo.u8_msScrLpCxmemTuneVer = data[index++];
	info->chipinfo.u8_msScrHwulpCxmemTuneVer = data[index++];
	info->chipinfo.u8_msKeyCxmemTuneVer = data[index++];
	info->chipinfo.u8_ssTchCxmemTuneVer = data[index++];
	fts_info(info, "CX SS TUNING VERSION = %02X \n", info->chipinfo.u8_ssTchCxmemTuneVer);
	info->chipinfo.u8_ssKeyCxmemTuneVer = data[index++];
	info->chipinfo.u8_ssHvrCxmemTuneVer = data[index++];
	info->chipinfo.u8_frcTchCxmemTuneVer = data[index++];
	info->chipinfo.u8_msScrPnlCfgTuneVer = data[index++];
	fts_info(info, "MS TUNING VWESION PANEL CONFIG = %02X \n", info->chipinfo.u8_msScrPnlCfgTuneVer);
	info->chipinfo.u32_mpPassFlag = ((data[index + 3] & 0x000000FF) << 24) + ((data[index + 2] & 0x000000FF) << 16) + ((data[index + 1] & 0x000000FF) << 8) + (data[index] & 0x000000FF);
	index += 4;
	fts_info(info, "MP SIGNATURE = %08X \n", info->chipinfo.u32_mpPassFlag);
	info->chipinfo.u32_featEn = ((data[index + 3] & 0x000000FF) << 24) + ((data[index + 2] & 0x000000FF) << 16) + ((data[index + 1] & 0x000000FF) << 8) + (data[index] & 0x000000FF);
	index += 4;
	info->chipinfo.u32_echoEn = ((data[index + 3] & 0x000000FF) << 24) + ((data[index + 2] & 0x000000FF) << 16) + ((data[index + 1] & 0x000000FF) << 8) + (data[index] & 0x000000FF);
	index += 4;
	fts_info(info, "FEATURES = %08X \n", info->chipinfo.u32_echoEn);

	fts_info(info, "Parsed %d bytes! \n", index);


	if (index != CHIP_INFO_SIZE + 3) {
		fts_err(info, "readChipInfo: index = %d different from %d ERROR %02X\n", index, CHIP_INFO_SIZE + 3, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	fts_info(info, "Chip Info Read DONE!\n");
	return OK;

FAIL:
	fts_defaultChipInfo(info, ftm4_isI2cError(ret));
	return ret;
}

