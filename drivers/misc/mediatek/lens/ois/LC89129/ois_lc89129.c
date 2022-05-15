/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ois_lc89129.h"
#include "FromCode_01_08.h"

/*
//current specific fw
const struct ois_fw_info EXP_FW = {
				.version = 0x0201,
				.type		= FW_VIVO,
				.date		= 0x1124
};
*/

const CODE_TBL_EXT CdTbl[] = {
	{0x0108, CcUpdataCode129, UpDataCodeSize, UpDataCodeCheckSum, CcFromCode129_01_08, sizeof(CcFromCode129_01_08), FromCheckSum_01_08, FromCheckSumSize_01_08},
	{0xFFFF, 0, 0, 0, 0, 0, 0, 0}
};
/* on Module vendor, Actuator Size,	 on vesion number */
UINT_32 FW_info129[][3] = {
	{0x01, 0x08, VERNUM_01_08}
};
#define VIVO_OTP_DATA_SIZE 0x34AB
extern unsigned char vivo_otp_data_s5kgh1sm24pd2083[VIVO_OTP_DATA_SIZE];
// unsigned char vivo_otp_data_s5kgh1sm24pd2083[VIVO_OTP_DATA_SIZE] = {0,};


static int LC89129_log_control(struct ois *ois, int level)
{
	int ret = 0;

	if (level > OIS_LOG_START && level < OIS_LOG_END)
		log_ois_level = level;

	LOG_OIS_INF("log %d", log_ois_level);
	return ret;
}

static int RamRead32A_CheckIICTimeout(struct ois *ois, UINT_16 addr, UINT_32 *ReadVal)
{
	int re = 0;
	UINT_32 dataTemp = 0, timeoutTemp = 0;
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	timeoutTemp = client->adapter->timeout;
	client->adapter->timeout = HZ/20;

	LOG_OIS_INF("custom iic timeout change timetout= %d", client->adapter->timeout);
	re  = ois_i2c_read_32(client, addr, ReadVal);

	client->adapter->timeout = timeoutTemp;
	LOG_OIS_INF("custom iic timeout ret =%d \n", re);

	dataTemp = *ReadVal;
	*ReadVal = ((dataTemp & 0xFF) << 24) | (((dataTemp >> 8) & 0xFF) << 16) |
		(((dataTemp >> 16) & 0xFF) << 8) | ((dataTemp >> 24) & 0xFF);
	LOG_OIS_INF("custom iic timeout vRcvBuff = %x", *ReadVal);

	return ((re == -2) || (re == -5)) ? 1 : 0;
}

static int RamRead32A(struct ois *ois, UINT_16 addr, UINT_32 *ReadVal)
{
	int re = 0;
	UINT_32 dataTemp = 0;
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	ois_i2c_read_32(client, addr, ReadVal);
	dataTemp = *ReadVal;
	*ReadVal = ((dataTemp & 0xFF) << 24) | (((dataTemp >> 8) & 0xFF) << 16) |
		(((dataTemp >> 16) & 0xFF) << 8) | ((dataTemp >> 24) & 0xFF);
	return re;
}

static int RamWrite32A(struct ois *ois, UINT_16 addr, UINT_32 WriteVal)
{
	int re = 0;
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	re = ois_i2c_write_32(client, addr, WriteVal);
	I2COP_CHECK(re);
p_err:
	return re;
}

static void IORead32A(struct ois *ois, UINT_32 IOadrs, UINT_32 *IOdata)
{
	RamWrite32A(ois, LC89129_REG_OIS_CMD_IO_ADDR, IOadrs);
	RamRead32A(ois, LC89129_REG_OIS_CMD_IO_DATA, IOdata);
}

static void IOWrite32A(struct ois *ois, UINT_32 IOadrs, UINT_32 IOdata)
{
	RamWrite32A(ois, LC89129_REG_OIS_CMD_IO_ADDR, IOadrs);
	RamWrite32A(ois, LC89129_REG_OIS_CMD_IO_DATA, IOdata);
}

// burst write
static int CntWrt(struct ois *ois, UINT_8 *writeval, UINT_16 UcLength)
{
	int ret = 0;
	int writeCnt = 2;
	UINT_8 WriteValtemp[128] = {0};
	UINT_32 writeDataSize = 0;
	struct i2c_client *client = ois->client;
	UINT_16 addr = 0;
	OIS_BUG(!client);
	writeDataSize = sizeof(UINT_8) * (UcLength - 2);
	addr = (writeval[0] << 8) + writeval[1];

	for (writeCnt = 2; writeCnt < UcLength; writeCnt++) {
		WriteValtemp[writeCnt - 2] = writeval[writeCnt];
	}
	ret = ois_i2c_write_block(client, addr, WriteValtemp, writeDataSize);
	return ret;
}

// burst read
static int CntRd(struct ois *ois, UINT_32 addr, UINT_8 *ReadVal, UINT_16 UcLength)
{
	int ret = 0;
	int readCnt = 0;
	UINT_32 readSize = sizeof(UINT_8) * UcLength;
	UINT_8 readValtemp[256] = {0};
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	ret = ois_i2c_read_block(client, addr, readValtemp, readSize);
	for (readCnt = 0; readCnt < UcLength; readCnt++) {
		ReadVal[readCnt] = readValtemp[readCnt];
	}
	return ret;
}

// status check
static UINT_8 RdStatus(struct ois *ois, UINT_8 UcStBitChk)
{
	UINT_32 UlReadVal;
	// int ret = 0;

	RamRead32A(ois, LC89129_REG_OIS_STATUS_READ, &UlReadVal);
	if (UcStBitChk) {
		UlReadVal &= READ_STATUS_INI;
	}
	if (!UlReadVal) {
		return SUCCESS;
	} else {
		return FAILURE;
	}
}

//
static UINT_16 MkInfMATsum(UINT_32 *InfMAT)
{
	UINT_16 UsCkVal = 0;
	UINT_16 i;

	for (i = 0; i < 63; i++) {
		UsCkVal += (UINT_8)(InfMAT[i] >> 0);
		UsCkVal += (UINT_8)(InfMAT[i] >> 8);
		UsCkVal += (UINT_8)(InfMAT[i] >> 16);
		UsCkVal += (UINT_8)(InfMAT[i] >> 24);
	}
	UsCkVal += (UINT_8)(InfMAT[i] >> 16);
	UsCkVal += (UINT_8)(InfMAT[i] >> 24);

	return UsCkVal;
}

// init OIS and enable ois
static int LC89129_init(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	UINT_32 old_version_h = 0;
	UINT_32 old_version_l = 0;
	UINT_32 exp_version_h = 0;
	UINT_32 exp_version_l = 0;
	UINT_8 UcStRd = 1;
	UINT_32 UlStCnt = 0;

	LOG_OIS_INF("E");

	OIS_BUG(!client);

	/*1.get corret i2c addr*/
	ois->client->addr = LC89129_SLAVE_ADDR >> 1;
	/*2.OIS init on*/
	ret = RamWrite32A(ois, LC89129_REG_OIS_SPI_INIT, SPI_6DSOQ_MASTER);
	I2COP_CHECK(ret);
	// ret = RamWrite32A(ois, LC89129_REG_SERVE_CTRL, OIS_SERVE_ON);
	// I2COP_CHECK(ret);
	while (UcStRd && (UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(ois, 1);
	}
	mdelay(5);

	/////////////////////////////////////////////////////////////
	ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_ON);
	I2COP_CHECK(ret);
	ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_ON);
	I2COP_CHECK(ret);
	LOG_OIS_INF("X");
	ret = RamRead32A(ois, LC89129_REG_OIS_FW_VERSION_HIGH, &old_version_h);
	ret = RamRead32A(ois, LC89129_REG_OIS_FW_VERSION_LOW, &old_version_l);
	old_version_l = old_version_l & 0xFFFFFF00;
	{
		exp_version_h =
				(CcFromCode129_01_08[158] << 24) + (CcFromCode129_01_08[159] << 16) +
				(CcFromCode129_01_08[160] << 8) + (CcFromCode129_01_08[161]);
		exp_version_l =
				((CcFromCode129_01_08[163] << 24) + (CcFromCode129_01_08[164] << 16) +
				 (CcFromCode129_01_08[165] << 8) + (CcFromCode129_01_08[166])) &
				0xFFFFFF00;
	}
	LOG_OIS_INF("FirmwareUpdateLC89129 oldh:0x%08x oldl:0x%08x ,newh:0x%08x newl:0x%08x", old_version_h, old_version_l, exp_version_h, exp_version_l);
p_err:
	return ret;
}

static int LC89129_init_slave(struct ois *ois)
{
	int ret = 0;
	return ret;
}

// deinit OIS and disable ois
static int LC89129_deinit(struct ois *ois)
{
	int ret = 0;
	UINT_8 UcStRd = 1;
	UINT_32 UlStCnt = 0;
	struct i2c_client *client = ois->client;

	LOG_OIS_INF("E");

	OIS_BUG(!client);

	/*1.disable af drift compensation*/
	/*2.servo off*/
	ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_OFF);
	I2COP_CHECK(ret);
	// ret = RamWrite32A(ois, LC89129_REG_SERVE_CTRL, OIS_SERVE_OFF);
	// I2COP_CHECK(ret);
	while (UcStRd && (UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(ois, 1);
	}
	ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_OFF);
	I2COP_CHECK(ret);

	LOG_OIS_INF("X(%d)", ret);
p_err:
	return ret;
}

// In development
static int LC89129_stream_on(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int LC89129_stream_off(struct ois *ois)
{
	int ret = 0;
	return ret;
}

// get ois mode
static int LC89129_get_mode(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->mode, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("ois mode(0x%x)", flash_info->mode);

	return ret;
}

static int LC89129_set_mode(struct ois *ois, int mode)
{
	int ret = 0;
	int user_mode = 0;
	u32 exp_mode = 0x00000000;
	u32 old_mode = 0x00000000;
	u32 new_mode = 0x00000000;
	// u32 op_status = OPERATE_DONE;
	struct i2c_client *client = ois->client;
	user_mode = mode;

	OIS_BUG(!client);

	ret = RamRead32A(ois, LC89129_REG_OIS_MOOD, &old_mode);
	// old_mode = ((old_mode & 0xFF) << 24 ) + ((old_mode & 0xFF00) << 8) +
	// ((old_mode & 0xFF0000) >> 8) + ((old_mode & 0xFF000000) >> 24);

	switch (user_mode) {
	case OIS_CENTER_MODE: {
		// exp_mode = LC89129_CENTERING_MODE;
		exp_mode = OIS_SERVE_ON;
		ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_OFF);
		I2COP_CHECK(ret);
		ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_OFF);
		I2COP_CHECK(ret);
		// ret = RamWrite32A(ois, LC89129_REG_SERVE_CTRL, OIS_SERVE_ON);
		// I2COP_CHECK(ret);
		break;
	}
	case OIS_STILL_MODE: {
		// exp_mode = LC89129_STILL_MODE;
		exp_mode = OIS_ACC_ON;
		ret = RamWrite32A(ois, LC89129_REG_OIS_MOOD, OIS_MODE_STILL);
		I2COP_CHECK(ret);
		ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_ON);
		I2COP_CHECK(ret);
		ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_ON);
		I2COP_CHECK(ret);
		break;
	}
	case OIS_VIDEO_MODE: {
		exp_mode = OIS_MODE_MOVIE;
		ret = RamWrite32A(ois, LC89129_REG_OIS_MOOD, OIS_MODE_MOVIE);
		I2COP_CHECK(ret);
		ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_ON);
		I2COP_CHECK(ret);
		ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_ON);
		I2COP_CHECK(ret);
		break;
	}
	case OIS_ZOOM_MODE: {
		exp_mode = OIS_ACC_ON;
		break;
	}
	case OIS_FIX_MODE: {
		break;
	}
	case OIS_SINEWAVE_MODE: {
		exp_mode = OIS_ACC_ON;
		break;
	}
	case OIS_CIRCLEWAVE_MODE: {
		exp_mode = OIS_ACC_ON;
		break;
	}
	default:
		LOG_OIS_INF("unsupport ois mode(%d)", user_mode);
	}

	/*
	if (old_mode != user_mode) {
		ret = ois_i2c_write(client, LC89129_REG_OIS_MODE, exp_mode);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, LC89129_REG_OIS_CMD_STATUS, OPERATE_START);
		I2COP_CHECK(ret);
	}
	*/
	ret = RamRead32A(ois, LC89129_REG_OIS_MOOD, &new_mode);
	// ret = ois_i2c_read(client, LC89129_REG_OIS_CMD_STATUS, &op_status);

	LOG_OIS_INF("old mode(0x%04x), exp mode(0x%04x) new mode(0x%04x) result(%d)",
		old_mode, exp_mode, new_mode, ret);

p_err:
	return ret;
}

// set to unlock code
static UINT_8 UnlockCodeSet(struct ois *ois)
{
	UINT_32 UlReadVal, UlCnt = 0;

	do {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE1, OIS_UNLOCK_CODE_1);
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE2, OIS_UNLOCK_CODE_2);
		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAWP, &UlReadVal);
		if ((UlReadVal & FLASHROM_MAP) != 0)
			return 0;
		mdelay(1);
	} while (UlCnt++ < 10);
	return 1;
}
//
static UINT_8 UnlockCodeClear(struct ois *ois)
{
	UINT_32 UlDataVal, UlCnt = 0;

	do {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAWP, 0x00000010);
		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAWP, &UlDataVal);
		if ((UlDataVal & FLASHROM_MAP) == 0)
			return 0;
		mdelay(1);
	} while (UlCnt++ < 10);
	return 3;
}

static void AdditionalUnlockCodeSet(struct ois *ois)
{
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3, 0x0000ACD5);
}

static void WritePermission(struct ois *ois)
{
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_RSTB_FLA, LC89129_REG_ON);
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CLK_FLAON, 0x00000010);
}
// Erase User Mat
static UINT_8 EraseUserMat129(struct ois *ois, UINT_8 StartBlock, UINT_8 EndBlock)
{
	UINT_32 i;
	UINT_32 UlReadVal, UlCnt;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_SETUP, 0x00000000);

	for (i = StartBlock; i < EndBlock; i++) {
		RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_STARADDR, (i << 10));
		RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, 0x00000020);

		mdelay(5);
		UlCnt = 0;
		do {
			mdelay(5);
			if (UlCnt++ > 100) {
				IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
				return 0x31;
			}
			RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
		} while (UlReadVal != 0);
	}
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
	return 0;
}

//********************************************************************************
// Function Name	: ProgramFlash129_Standard
//********************************************************************************
static UINT_8 ProgramFlash129_Standard(struct ois *ois, CODE_TBL_EXT *ptr)
{
	UINT_32 UlReadVal, UlCnt, UlNum;
	UINT_8 data[(BURST_LENGTH_FC + 3)];
	UINT_32 i, j;

	const UINT_8 *NcFromVal = ptr->FromCode + 64;
	const UINT_8 *NcFromVal1st = ptr->FromCode;
	UINT_8 UcOddEvn = 0;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_SETUP, 0x00000000);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_STARADDR, 0x00000010);
	data[0] = 0xF0;
	data[1] = 0x08;
	data[2] = 0x00;

	for (i = 1; i < (ptr->SizeFromCode / 64); i++) {
		if (++UcOddEvn > 1)
			UcOddEvn = 0;
		if (UcOddEvn == 0)
			data[1] = 0x08;
		else
			data[1] = 0x09;

#if (BURST_LENGTH_FC == 32)
		data[2] = 0x00;
		UlNum = 3;
		for (j = 0; j < BURST_LENGTH_FC; j++) {
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt(ois, data, BURST_LENGTH_FC + 3);

		data[2] = 0x20;
		UlNum = 3;
		for (j = 0; j < BURST_LENGTH_FC; j++) {
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt(ois, data, BURST_LENGTH_FC + 3);

#elif (BURST_LENGTH_FC == 64)
		UlNum = 3;
		for (j = 0; j < BURST_LENGTH_FC; j++) {
			data[UlNum++] = *NcFromVal++;
		}
		CntWrt(ois, data, BURST_LENGTH_FC + 3);
#endif

		RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_LENS, 0x00000010);
		UlCnt = 0;
		if (UcOddEvn == 0) {
			do {
				mdelay(1);
				RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
				if (UlCnt++ > 250) {
					IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
					return 0x41;
				}
			} while (UlReadVal != 0);
			RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, 0x00000004);
		} else {
			do {
				mdelay(1);
				RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
				if (UlCnt++ > 250) {
					IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
					return 0x41;
				}
			} while (UlReadVal != 0);
			RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, 0x00000008);
		}
	}

	UlCnt = 0;
	do {
		mdelay(1);
		RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
		if (UlCnt++ > 250) {
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
			return 0x41;
		}
	} while ((UlReadVal & 0x0000000C) != 0);

	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_STARADDR, 0x00000000);
	data[1] = 0x08;

#if (BURST_LENGTH_FC == 32)
	data[2] = 0x00;
	UlNum = 3;
	for (j = 0; j < BURST_LENGTH_FC; j++) {
		data[UlNum++] = *NcFromVal1st++;
	}
	CntWrt(ois, data, BURST_LENGTH_FC + 3);

	data[2] = 0x20;
	UlNum = 3;
	for (j = 0; j < BURST_LENGTH_FC; j++) {
		data[UlNum++] = *NcFromVal1st++;
	}
	CntWrt(ois, data, BURST_LENGTH_FC + 3);
#elif (BURST_LENGTH_FC == 64)
	data[2] = 0x00;
	UlNum = 3;
	for (j = 0; j < BURST_LENGTH_FC; j++) {
		data[UlNum++] = *NcFromVal1st++;
	}
	CntWrt(ois, data, BURST_LENGTH_FC + 3);
#endif

	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_LENS, 0x00000010);
	UlCnt = 0;
	do {
		mdelay(1);
		RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
		if (UlCnt++ > 250) {
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
			return 0x41;
		}
	} while (UlReadVal != 0);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, 0x00000004);

	UlCnt = 0;
	do {
		mdelay(1);
		RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
		if (UlCnt++ > 250) {
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
			return 0x41;
		}
	} while ((UlReadVal & 0x0000000C) != 0);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
	return 0;
}

// FlashMultiRead129
static UINT_8 FlashMultiRead129(struct ois *ois, UINT_8 SelMat,
	UINT_32 UlAddress, UINT_32 *PulData, UINT_8 UcLength)
{
	UINT_8 i;

	if (SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2)
		return 10;
	if (UlAddress > 0x00003FFF)
		return 9;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_ACSCNT, 0x00000000 | (UINT_32)(UcLength - 1));
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | (UlAddress & 0x00003FFF));
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 0x00000001);
	for (i = 0; i < UcLength; i++) {
		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_RDAT, &PulData[i]);
	}

	return 0;
}

//********************************************************************************
// Function Name	: FlashBlockErase
//********************************************************************************
static UINT_8 FlashBlockErase129(struct ois *ois, UINT_8 SelMat, UINT_32 SetAddress)
{
	UINT_32 UlReadVal, UlCnt;
	UINT_8 ans = 0;

	if (SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2)
		return 10;
	if (SetAddress > 0x00003CFF)
		return 9;

	ans = UnlockCodeSet(ois);
	if (ans != 0)
		return ans;

	WritePermission(ois);
	if (SelMat == TRIM_MAT) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3, 0x00005B29);
	} else if (SelMat != USER_MAT) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3, 0x0000C5AD);
	}
	AdditionalUnlockCodeSet(ois);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | (SetAddress & 0x00003C00));
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 4);

	mdelay(5);

	UlCnt = 0;

	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		};

		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAINT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	ans = UnlockCodeClear(ois);
	if (ans != 0)
		return ans;

	return ans;
}

//
static UINT_8 FlashPageWrite(struct ois *ois, UINT_8 SelMat, UINT_32 SetAddress, UINT_32 *PulData)
{
	UINT_32 UlReadVal, UlCnt;
	UINT_8 ans = 0;
	UINT_8 i;

	if (SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2)
		return 10;

	if (SetAddress > 0x00003FFF)
		return 9;

	ans = UnlockCodeSet(ois);
	if (ans != 0)
		return ans;

	WritePermission(ois);
	if (SelMat != USER_MAT) {
		IOWrite32A(ois, 0xE07CCC, 0x0000C5AD);
	}
	AdditionalUnlockCodeSet(ois);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | (SetAddress & 0x00003FF0));
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 2);

	UlCnt = 0;

	for (i = 0; i < 16; i++) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_WDAT, PulData[i]);
	}
	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		};

		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAINT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 8);
	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		};

		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAINT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	ans = UnlockCodeClear(ois);
	return ans;
}

//
static UINT_8 WrInfMAT(struct ois *ois, UINT_8 SelMat, UINT_32 *InfMat, UINT_16 Length)
{
	UINT_8 ans = 0;
	UINT_32 address;

	for (address = 0; (address < Length) && (ans == 0); address += 16) {
		ans = FlashPageWrite(ois, SelMat, address, &InfMat[address]);
	}
	return ans;
}

//
static UINT_8 WrTempCompData(struct ois *ois, stAdj_Temp_Compensation *TempCompPtr)
{
	UINT_32 UlMAT1[64];
	UINT_8 ans = 0;
	UINT_16 UsCkVal, UsCkVal_Bk;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);

	// ans = RdErInfMAT(ois, INF_MAT1, UlMAT1, 64 );
	ans = FlashMultiRead129(ois, INF_MAT1, 0, UlMAT1, 64);
	if (ans == 0) {
		ans = FlashBlockErase129(ois, INF_MAT1, 0);
		if (ans != 0) {
			ans = 2;
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
			return ans;
		}
	} else {
		ans = 1;
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return ans;
	}

	UlMAT1[0] = TempCompPtr->rcodeX;
	UlMAT1[1] = TempCompPtr->rcodeY;
	UlMAT1[2] = TempCompPtr->rcodeZ;
	UlMAT1[3] = TempCompPtr->shag;
	UlMAT1[4] = TempCompPtr->shbg;
	UlMAT1[5] = TempCompPtr->shcg;
	UlMAT1[6] = TempCompPtr->shoutag;
	UlMAT1[7] = TempCompPtr->shoutbg;
	UlMAT1[8] = TempCompPtr->shoutag1;
	UlMAT1[9] = TempCompPtr->shoutbg1;
	UlMAT1[10] = TempCompPtr->tag;
	UlMAT1[11] = TempCompPtr->tbg;
	UlMAT1[12] = TempCompPtr->shiftg;
	UlMAT1[13] = TempCompPtr->shab;
	UlMAT1[14] = TempCompPtr->shac;
	UlMAT1[15] = TempCompPtr->shaa;
	UlMAT1[16] = TempCompPtr->shbb;
	UlMAT1[17] = TempCompPtr->shbc;
	UlMAT1[18] = TempCompPtr->shba;
	UlMAT1[19] = TempCompPtr->shcb;
	UlMAT1[20] = TempCompPtr->shcc;
	UlMAT1[21] = TempCompPtr->shca;
	UlMAT1[22] = TempCompPtr->tab;
	UlMAT1[23] = TempCompPtr->tac;
	UlMAT1[24] = TempCompPtr->taa;
	UlMAT1[25] = TempCompPtr->tbb;
	UlMAT1[26] = TempCompPtr->tbc;
	UlMAT1[27] = TempCompPtr->tba;
	UlMAT1[28] = TempCompPtr->TEMPOFF;

	UlMAT1[29] = ((UINT_32)TempCompPtr->tcx << 16) |
		((UINT_32)TempCompPtr->tbx << 8) | ((UINT_32)TempCompPtr->tax);

	UsCkVal = MkInfMATsum(UlMAT1);
	UlMAT1[63] &= (UINT_32)0xFFFF0000;
	UlMAT1[63] |= (UINT_32)UsCkVal;

	ans = WrInfMAT(ois, INF_MAT1, UlMAT1, 64);
	if (ans != 0) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return 3;
	}

	UsCkVal_Bk = UsCkVal;
	ans = FlashMultiRead129(ois, INF_MAT1, 0, UlMAT1, 64);
	if (ans) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return 4;
	}
	UsCkVal = MkInfMATsum(UlMAT1);

	if (UsCkVal != UsCkVal_Bk) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return 5;
	}

	IOWrite32A(ois, 0xE0701C, 0x00000002);
	return 0;
}

// program memory update
static UINT_8 PmemUpdate129(struct ois *ois, UINT_8 dist, CODE_TBL_EXT *ptr)
{
	UINT_8 data[BURST_LENGTH_UC + 2];
	UINT_16 Remainder;
	UINT_8 ReadData[8];
	const UINT_8 *NcDataVal;
	UINT_16 SizeofCode;
	UINT_16 SizeofCheck;
	long long CheckSumCode;
	UINT_8 *p;
	UINT_32 i, j;
	UINT_32 UlReadVal, UlCnt, UlNum;

	if (dist != 0) {
		NcDataVal = ptr->FromCode + 32;
		SizeofCode = (UINT_16)ptr->FromCode[9] << 8 | (UINT_16)ptr->FromCode[8];

		CheckSumCode = (long long)ptr->FromCode[19] << 56 |
			(long long)ptr->FromCode[18] << 48 |
			(long long)ptr->FromCode[17] << 40 |
			(long long)ptr->FromCode[16] << 32 |
			(long long)ptr->FromCode[15] << 24 |
			(long long)ptr->FromCode[14] << 16 |
			(long long)ptr->FromCode[13] << 8 |
			(long long)ptr->FromCode[12];

		SizeofCheck = SizeofCode;
	} else {
		NcDataVal = ptr->UpdataCode;
		SizeofCode = ptr->SizeUpdataCode;
		CheckSumCode = ptr->SizeUpdataCodeCksm;
		SizeofCheck = SizeofCode;
	}
	p = (UINT_8 *)(&CheckSumCode);

	//--------------------------------------------------------------------------------
	// 1.
	//--------------------------------------------------------------------------------
	RamWrite32A(ois, LC89129_REG_OIS_PMEM_STARADDR, 0x00080000);

	data[0] = 0x40;
	data[1] = 0x00;

	Remainder = ((SizeofCode * 5) / BURST_LENGTH_UC);
	for (i = 0; i < Remainder; i++) {
		UlNum = 2;
		for (j = 0; j < BURST_LENGTH_UC; j++) {
			data[UlNum] = *NcDataVal++;
			UlNum++;
		}

		CntWrt(ois, data, BURST_LENGTH_UC + 2);
	}
	Remainder = ((SizeofCode * 5) % BURST_LENGTH_UC);
	if (Remainder != 0) {
		UlNum = 2;
		for (j = 0; j < Remainder; j++) {
			data[UlNum++] = *NcDataVal++;
		}
		CntWrt(ois, data, Remainder + 2); // Cmd 2Byte
	}

	//--------------------------------------------------------------------------------
	// 2.
	//--------------------------------------------------------------------------------
	data[0] = 0xF0;
	data[1] = 0x0E;
	data[2] = (unsigned char)((SizeofCheck >> 8) & 0x000000FF);
	data[3] = (unsigned char)(SizeofCheck & 0x000000FF);
	data[4] = 0x00;
	data[5] = 0x00;

	CntWrt(ois, data, 6);

	UlCnt = 0;
	do {
		mdelay(1);
		if (UlCnt++ > 10) {
			return 0x21;
		}
		RamRead32A(ois, 0x0088, &UlReadVal);
	} while (UlReadVal != 0);

	CntRd(ois, LC89129_REG_OIS_PMEM_CHECKSUM, ReadData, 8);

	for (i = 0; i < 8; i++) {
		if (ReadData[7 - i] != *p++) {
			return 0x22;
		}
	}
	if (dist != 0) {
		RamWrite32A(ois, LC89129_REG_OIS_REMAP, 0);
	}

	return 0;
}

// flash
static UINT_8 FlashUpdate129(struct ois *ois, CODE_TBL_EXT *ptr)
{
	UINT_8 ans = 0;
	UINT_32 UlReadVal, UlCnt;
	stAdj_Temp_Compensation *TempCompPtr;

	//--------------------------------------------------------------------------------
	// 1.
	//--------------------------------------------------------------------------------
	ans = PmemUpdate129(ois, 0, ptr);
	if (ans != 0)
		return ans;

	//--------------------------------------------------------------------------------
	// 2.
	//--------------------------------------------------------------------------------
	if (UnlockCodeSet(ois) != 0)
		return 0x33;
	WritePermission(ois);
	AdditionalUnlockCodeSet(ois);

	//	if( chiperase != 0 )
	ans = EraseUserMat129(ois, 0, FLASH_BLOCKS);
	//	else
	//		 ans = EraseUserMat129( 0, ERASE_BLOCKS ) ;

	if (ans != 0) {
		if (UnlockCodeClear(ois) != 0)
			return 0x32;
		else
			return ans;
	}

	//--------------------------------------------------------------------------------
	// 3.
	//--------------------------------------------------------------------------------
	ans = ProgramFlash129_Standard(ois, ptr);

	if (ans != 0) {
		if (UnlockCodeClear(ois) != 0)
			return 0x43;
		else
			return ans;
	}

	if (UnlockCodeClear(ois) != 0)
		return 0x43;

	//--------------------------------------------------------------------------------
	// 4.
	//--------------------------------------------------------------------------------
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_STARADDR, 0x00000000);
	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_CHECKSUM, ptr->SizeFromCodeValid);

	RamWrite32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, 0x00000100);
	mdelay(6);
	UlCnt = 0;
	do {
		RamRead32A(ois, LC89129_REG_OIS_FLASHROM_ACCESS_CTL, &UlReadVal);
		if (UlCnt++ > 100) {
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
			return 0x51;
		}
		mdelay(1);
	} while (UlReadVal != 0);

	RamRead32A(ois, LC89129_REG_OIS_FLASHROM_CHECKSUM, &UlReadVal);

	if (UlReadVal != ptr->SizeFromCodeCksm) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return 0x52;
	}
	if (ptr->Index == 0x0008) {
		TempCompPtr = (stAdj_Temp_Compensation *)SO2692_TempCompParameter;
		ans = WrTempCompData(ois, TempCompPtr);
		if (ans != 0x00) {
			return 0x53;
		}
	}

	IOWrite32A(ois, LC89129_REG_OIS_CMD_SYSDSP_REMAP, 0x00001001);
	mdelay(15);
	IORead32A(ois, LC89129_REG_OIS_CMD_ROMINFO, (UINT_32 *)&UlReadVal);
	if (UlReadVal != 0x0A)
		return 0x53;

	return 0;
}

// update fw
static int LC89129_fw_update(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	UINT_32 old_version_h = 0;
	UINT_32 old_version_l = 0;
	UINT_32 exp_version_h = 0;
	UINT_32 exp_version_l = 0;
	UINT_32 ReadVal = 0;
	struct i2c_client *client = ois->client;
	CODE_TBL_EXT *ptr = (CODE_TBL_EXT *)CdTbl;
	UINT_8 ModuleVendor = FW_info129[0][0];
	UINT_8 ActVer = FW_info129[0][1];

	OIS_BUG(!client);

	LOG_OIS_INF("E");
	ois->client->addr = LC89129_SLAVE_ADDR >> 1;

	//check iic short circuit timeout by changcheng 2021.3.6
	ret = RamRead32A_CheckIICTimeout(ois, LC89129_REG_OIS_FW_VERSION_HIGH, &old_version_h);
	if (ret) {
		pr_err("[%s] iic hardware short circuit timeout ret =%d \n", __FUNCTION__, ret);
		return ret;
	}

	// 1. check fw version & type
	ret = RamRead32A(ois, LC89129_REG_OIS_FW_VERSION_HIGH, &old_version_h);
	ret = RamRead32A(ois, LC89129_REG_OIS_FW_VERSION_LOW, &old_version_l);
	old_version_l = old_version_l & 0xFFFFFF00;
	{
		exp_version_h = (CcFromCode129_01_08[158] << 24) + (CcFromCode129_01_08[159] << 16) +
			(CcFromCode129_01_08[160] << 8) + (CcFromCode129_01_08[161]);
		exp_version_l = ((CcFromCode129_01_08[163] << 24) + (CcFromCode129_01_08[164] << 16) +
			(CcFromCode129_01_08[165] << 8) + (CcFromCode129_01_08[166])) & 0xFFFFFF00;
	}
	LOG_OIS_INF("FirmwareUpdateLC89129 :%x	%x ,%x	%x",
		old_version_h, old_version_l, exp_version_h, exp_version_l);

	if ((old_version_h != exp_version_h) || (old_version_l != exp_version_l)) {
		LOG_OIS_INF("FlashDownload128 start.");
		// flash download start
		do {
			if (ptr->Index == (((UINT_16)ModuleVendor << 8) + ActVer)) {
				// set in boot mode
				IORead32A(ois, LC89129_REG_OIS_CMD_SYSDSP_REMAP, &ReadVal);
				ReadVal = (ReadVal & 0x1) | 0x00001400;
				IOWrite32A(ois, LC89129_REG_OIS_CMD_SYSDSP_REMAP, ReadVal);
				I2COP_CHECK(ret);
				mdelay(15);
				// flash update
				ret = FlashUpdate129(ois, ptr);
				I2COP_CHECK(ret);
				break;
			}
			ptr++;
		} while (ptr->Index != 0xFFFF);
		// param download start
		LOG_OIS_INF("FlashDownload128 end. %d", ret);
		// RamWrite32A(ois, 0xF015, 0x00000004);
	}
p_err:
	return ret;
}

// get_fw_version
static int LC89129_get_fw_version(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->fwInfo.version, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("fw ver 0x%08x", flash_info->fwInfo.version);

	return ret;
}

static int LC89129_get_gyro_offset(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroOffsetX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("offset(%d, %d)", flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY);
	return ret;
}

//
static void SrvOn(struct ois *ois)
{
	UINT_8 UcStRd = 1;
	UINT_32 UlStCnt = 0;

	RamWrite32A(ois, LC89129_REG_SERVE_CTRL, OIS_SERVE_ON);

	while (UcStRd && (UlStCnt++ < CNT200MS)) {
		UcStRd = RdStatus(ois, 1);
	}
	LOG_OIS_INF("SrvOn	status:%d", UcStRd);
}

//
static void MesFil(struct ois *ois)
{
	UINT_32 UlMeasFilaA, UlMeasFilaB, UlMeasFilaC;
	UINT_32 UlMeasFilbA, UlMeasFilbB, UlMeasFilbC;

	UlMeasFilaA = 0x7FFFFFFF;
	UlMeasFilaB = 0x00000000;
	UlMeasFilaC = 0x00000000;
	UlMeasFilbA = 0x7FFFFFFF;
	UlMeasFilbB = 0x00000000;
	UlMeasFilbC = 0x00000000;

	RamWrite32A(ois, 0x8388, UlMeasFilaA);
	RamWrite32A(ois, 0x8380, UlMeasFilaB);
	RamWrite32A(ois, 0x8384, UlMeasFilaC);

	RamWrite32A(ois, 0x8394, UlMeasFilbA);
	RamWrite32A(ois, 0x838C, UlMeasFilbB);
	RamWrite32A(ois, 0x8390, UlMeasFilbC);

	RamWrite32A(ois, 0x83A0, UlMeasFilaA);
	RamWrite32A(ois, 0x8398, UlMeasFilaB);
	RamWrite32A(ois, 0x839C, UlMeasFilaC);

	RamWrite32A(ois, 0x83AC, UlMeasFilbA);
	RamWrite32A(ois, 0x83A4, UlMeasFilbB);
	RamWrite32A(ois, 0x83A8, UlMeasFilbC);
}

//
static void MeasAddressSelection129(UINT_8 mode, INT_32 *measadr_a, INT_32 *measadr_b)
{
	if (mode == 0) {
		*measadr_a = LC89129_REG_OIS_GYROCAL_RAM_GX_ADIDAT;
		*measadr_b = LC89129_REG_OIS_GYROCAL_RAM_GY_ADIDAT;
	} else if (mode == 1) {
		*measadr_a = LC89129_REG_OIS_GYROCAL_ZRAM_GZ_ADIDAT;
		*measadr_b = LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_ADIDAT;
	} else {
		*measadr_a = LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_ADIDAT;
		*measadr_b = LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_ADIDAT;
	}
}

static void MemoryClear(struct ois *ois, UINT_16 UsSourceAddress, UINT_16 UsClearSize)
{
	UINT_16 UsLoopIndex;

	for (UsLoopIndex = 0; UsLoopIndex < UsClearSize;) {
		RamWrite32A(ois, UsSourceAddress, 0x00000000);
		UsSourceAddress += 4;
		UsLoopIndex += 4;
	}
}

//
static void ClrMesFil129(struct ois *ois)
{
	RamWrite32A(ois, 0x02D0, 0);
	RamWrite32A(ois, 0x02D4, 0);

	RamWrite32A(ois, 0x02D8, 0);
	RamWrite32A(ois, 0x02DC, 0);

	RamWrite32A(ois, 0x02E0, 0);
	RamWrite32A(ois, 0x02E4, 0);

	RamWrite32A(ois, 0x02E8, 0);
	RamWrite32A(ois, 0x02EC, 0);
}

//********************************************************************************
// Function Name	: SetWaitTime
//********************************************************************************

static void SetWaitTime129(struct ois *ois, UINT_16 UsWaitTime)
{
	RamWrite32A(ois, 0x0324, 0);
	RamWrite32A(ois, 0x0328, (UINT_32)(ONE_MSEC_COUNT * UsWaitTime));
}

//
static void MeasureStart129(struct ois *ois, INT_32 SlMeasureParameterNum,
	INT_32 SlMeasureParameterA, INT_32 SlMeasureParameterB)
{
	MemoryClear(ois, 0x0278, 18 * 4);
	RamWrite32A(ois, 0x0280, 0x80000000);
	RamWrite32A(ois, 0x02A8, 0x80000000);
	RamWrite32A(ois, 0x0284, 0x7FFFFFFF);
	RamWrite32A(ois, 0x02AC, 0x7FFFFFFF);

	RamWrite32A(ois, 0x02A0, (UINT_32)SlMeasureParameterA);
	RamWrite32A(ois, 0x02C8, (UINT_32)SlMeasureParameterB);

	RamWrite32A(ois, 0x0278, 0);
	ClrMesFil129(ois);
	SetWaitTime129(ois, 50);
	RamWrite32A(ois, 0x027C, SlMeasureParameterNum);
}

//
static void MeasureWait129(struct ois *ois)
{
	UINT_32 SlWaitTimerSt;
	UINT_16 UsTimeOut = 5000;
	do {
		RamRead32A(ois, 0x027C, &SlWaitTimerSt);
		UsTimeOut--;
	} while (SlWaitTimerSt && UsTimeOut);
}

//
static UINT_32 MeasGyAcOffset129(struct ois *ois)
{
	UINT_32 UlRsltSts;
	INT_32 SlMeasureParameterA, SlMeasureParameterB;
	INT_32 SlMeasureParameterNum;
	UnllnVal StMeasValueA, StMeasValueB;
	INT_32 SlMeasureAveValueA[3], SlMeasureAveValueB[3];
	UINT_8 i;
	INT_32 SlMeasureAZ = 0;

	MesFil(ois);

	SlMeasureParameterNum = MESOF_NUM; // Measurement times

	for (i = 0; i < 3; i++) {
		MeasAddressSelection129(i, &SlMeasureParameterA, &SlMeasureParameterB);

		MeasureStart129(ois, SlMeasureParameterNum, SlMeasureParameterA, SlMeasureParameterB);
		MeasureWait129(ois); // Wait complete of measurement
		MeasureWait129(ois); // Wait complete of measurement
		MeasureWait129(ois); // Wait complete of measurement
		MeasureWait129(ois); // Wait complete of measurement

		RamRead32A(ois, 0x0290, &StMeasValueA.StUllnVal.UlLowVal); // X axis
		RamRead32A(ois, 0x0290 + 4, &StMeasValueA.StUllnVal.UlHigVal);
		RamRead32A(ois, 0x02B8, &StMeasValueB.StUllnVal.UlLowVal); // Y axis
		RamRead32A(ois, 0x02B8 + 4, &StMeasValueB.StUllnVal.UlHigVal);

		SlMeasureAveValueA[i] =
				(INT_32)((INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum);
		SlMeasureAveValueB[i] =
				(INT_32)((INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum);
	}

	UlRsltSts = OIS_CALNG_EXE_END;
	LOG_OIS_INF("zycccccccccccc	status:%d", SlMeasureAveValueB[1]);

	if ((SlMeasureAveValueB[1]) >= POSTURETH) {
		SlMeasureAZ = SlMeasureAveValueB[1] - (INT_32)GSENS;
	} else if ((SlMeasureAveValueB[1]) <= -POSTURETH) {
		SlMeasureAZ = SlMeasureAveValueB[1] + (INT_32)GSENS;
	} else {
		UlRsltSts |= OIS_CALNG_EXE_AZADJ;
	}

	if (abs(SlMeasureAveValueA[0]) > GYROFFSET_H)
		UlRsltSts |= OIS_CALNG_EXE_GXADJ;
	if (abs(SlMeasureAveValueB[0]) > GYROFFSET_H)
		UlRsltSts |= OIS_CALNG_EXE_GYADJ;
	if (abs(SlMeasureAveValueA[1]) > GYROFFSET_H)
		UlRsltSts |= OIS_CALNG_EXE_GZADJ;

	if (abs(SlMeasureAveValueA[2]) > ZG_MRGN)
		UlRsltSts |= OIS_CALNG_EXE_AXADJ;
	if (abs(SlMeasureAveValueB[2]) > ZG_MRGN)
		UlRsltSts |= OIS_CALNG_EXE_AYADJ;
	if (abs(SlMeasureAZ) > ZG_MRGN)
		UlRsltSts |= OIS_CALNG_EXE_AZADJ;

	if (UlRsltSts == OIS_CALNG_EXE_END) {
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GXOFFZ,
								SlMeasureAveValueA[0]); // X axis Gyro offset
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GYOFFZ,
								SlMeasureAveValueB[0]); // Y axis Gyro offset
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_ZRAM_GZOFFZ,
								SlMeasureAveValueA[1]); // Z axis Gyro offset
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_OFFSET,
								SlMeasureAveValueA[2]); // X axis Accel offset
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_OFFSET,
								SlMeasureAveValueB[2]); // Y axis Accel offset
		RamWrite32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_OFFSET,
								SlMeasureAZ); // Z axis Accel offset

		RamWrite32A(ois, 0x01D8, 0x00000000);			// X axis Drift Gyro offset
		RamWrite32A(ois, 0x01FC, 0x00000000);			// Y axis Drift Gyro offset
		RamWrite32A(ois, 0x0370, 0x00000000);			// Z axis Drift Gyro offset
		RamWrite32A(ois, 0x019C, 0x00000000);			// X axis H1Z2 Clear
		RamWrite32A(ois, 0x01C4, 0x00000000);			// Y axis H1Z2 Clear
		RamWrite32A(ois, 0x03C0 + 8, 0x00000000);	// X axis Accl LPF Clear
		RamWrite32A(ois, 0x03F0 + 8, 0x00000000);	// Y axis Accl LPF Clear
		RamWrite32A(ois, 0x0420 + 8, 0x00000000);	// Z axis Accl LPF Clear
		RamWrite32A(ois, 0x03C0 + 12, 0x00000000); // X axis Accl LPF Clear
		RamWrite32A(ois, 0x03F0 + 12, 0x00000000); // Y axis Accl LPF Clear
		RamWrite32A(ois, 0x0420 + 12, 0x00000000); // Z axis Accl LPF Clear
		RamWrite32A(ois, 0x03C0 + 16, 0x00000000); // X axis Accl LPF Clear
		RamWrite32A(ois, 0x03F0 + 16, 0x00000000); // Y axis Accl LPF Clear
		RamWrite32A(ois, 0x0420 + 16, 0x00000000); // Z axis Accl LPF Clear
		RamWrite32A(ois, 0x03C0 + 20, 0x00000000); // X axis Accl LPF Clear
		RamWrite32A(ois, 0x03F0 + 20, 0x00000000); // Y axis Accl LPF Clear
		RamWrite32A(ois, 0x0420 + 20, 0x00000000); // Z axis Accl LPF Clear
	}
	return UlRsltSts;
}

//
static UINT_8 RdErInfMAT(struct ois *ois, UINT_8 SelMat, UINT_32 *InfMat, UINT_16 Length)
{
	UINT_8 ans = 0;

	ans = FlashMultiRead129(ois, SelMat, 0, InfMat, Length);

	if (ans == 0) {
		ans = FlashBlockErase129(ois, SelMat, 0);
		if (ans != 0) {
			ans = 2;
		}
	} else {
		ans = 1;
	}
	return ans;
}

//
static UINT_8 WrGyAcOffsetData129(struct ois *ois, UINT_8 UcMode)
{
	UINT_32 UlMAT0[64];
	UINT_32 UlReadGx, UlReadGy, UlReadGz, UlReadAx, UlReadAy, UlReadAz;
	UINT_8 ans = 0;
	UINT_16 UsCkVal, UsCkVal_Bk;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);

	ans = RdErInfMAT(ois, INF_MAT0, UlMAT0, 64);
	if (ans) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return ans;
	}

	if (UcMode) {
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GXOFFZ, &UlReadGx);
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GYOFFZ, &UlReadGy);
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ZRAM_GZ_ADIDAT, &UlReadGz);
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_OFFSET, &UlReadAx);
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_OFFSET, &UlReadAy);
		RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_OFFSET, &UlReadAz);

		UlMAT0[0] &= ~(0x00000100);
		UlMAT0[10] =
				(UINT_32)((UlReadGx & 0xFFFF0000) | ((UlReadAx >> 16) & 0x0000FFFF));
		UlMAT0[11] =
				(UINT_32)((UlReadGy & 0xFFFF0000) | ((UlReadAy >> 16) & 0x0000FFFF));
		UlMAT0[12] =
				(UINT_32)((UlReadGz & 0xFFFF0000) | ((UlReadAz >> 16) & 0x0000FFFF));

	} else {
		UlMAT0[0] |= 0x00000100;
		UlMAT0[10] = 0x00000000;
		UlMAT0[11] = 0x00000000;
		UlMAT0[12] = 0x00000000;
	}

	UsCkVal = MkInfMATsum(UlMAT0);
	UlMAT0[63] &= (UINT_32)0xFFFF0000;
	UlMAT0[63] |= (UINT_32)UsCkVal;

	ans = WrInfMAT(ois, INF_MAT0, UlMAT0, 64);
	if (ans != 0) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return 3; // 0 - 63
	}

	UsCkVal_Bk = UsCkVal;
	ans = FlashMultiRead129(ois, INF_MAT0, 0, UlMAT0, 64);
	if (ans) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return 4;
	}
	UsCkVal = MkInfMATsum(UlMAT0);

	if (UsCkVal != UsCkVal_Bk) {
		IOWrite32A(ois, 0xE0701C, 0x00000002);
		return 5;
	}

	IOWrite32A(ois, 0xE0701C, 0x00000002);
	return 0;
}
//
static int GetGyroOffset(struct ois *ois, UINT_16 *GyroOffsetX, UINT_16 *GyroOffsetY, UINT_16 *GyroOffsetZ)
{
	UINT_32 ReadValX = 0, ReadValY = 0, ReadValZ = 0;
	INT_32 re = RamRead32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GXOFFZ, &ReadValX);
	re |= RamRead32A(ois, LC89129_REG_OIS_GYROCAL_RAM_GYOFFZ, &ReadValY);
	re |= RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ZRAM_GZOFFZ, &ReadValZ);
	*GyroOffsetX = (UINT_16)((ReadValX >> 16) & 0x0000FFFF);
	*GyroOffsetY = (UINT_16)((ReadValY >> 16) & 0x0000FFFF);
	*GyroOffsetZ = (UINT_16)((ReadValZ >> 16) & 0x0000FFFF);
	return re;
}
//

static int GetAcclOffset(struct ois *ois, UINT_16 *AcclOffsetX,
	UINT_16 *AcclOffsetY, UINT_16 *AcclOffsetZ)
{
	UINT_32 ReadValX = 0, ReadValY = 0, ReadValZ = 0;
	int re = RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_OFFSET, &ReadValX);
	re |= RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_OFFSET, &ReadValY);
	re |= RamRead32A(ois, LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_OFFSET, &ReadValZ);
	*AcclOffsetX = (UINT_16)((ReadValX >> 16) & 0x0000FFFF);
	*AcclOffsetY = (UINT_16)((ReadValY >> 16) & 0x0000FFFF);
	*AcclOffsetZ = (UINT_16)((ReadValZ >> 16) & 0x0000FFFF);
	return re;
}

//
static int LC89129_set_offset_calibration(struct ois *ois)
{
	int ret = 0;
	UINT_8 UcStRd = 1;
	UINT_32 UlStCnt = 0;
	UINT_32 res1;
	UINT_32 res2;

	UINT_16 GyroOffsetX = 0, GyroOffsetY = 0, GyroOffsetZ = 0;
	UINT_16 AcclOffsetX = 0, AcclOffsetY = 0, AcclOffsetZ = 0;

	// get offset before cali
	ret = GetGyroOffset(ois, &GyroOffsetX, &GyroOffsetY, &GyroOffsetZ);
	ret = GetAcclOffset(ois, &AcclOffsetX, &AcclOffsetY, &AcclOffsetZ);
	LOG_OIS_INF("before Gyro offset cali, %x, %x, %x, Acc offset cali, %x, %x, %x",
		GyroOffsetX, GyroOffsetY, GyroOffsetZ, AcclOffsetX, AcclOffsetY, AcclOffsetZ);
	ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_OFF);
	// I2COP_CHECK(ret);
	ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_OFF);
	// I2COP_CHECK(ret);

	while (UcStRd && (UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(ois, 1);
	}
	mdelay(100);
	res1 = MeasGyAcOffset129(ois);
	res2 = WrGyAcOffsetData129(ois, 1);

	LOG_OIS_INF("MeasGyAcOffset, %x WrGyAcOffsetData, %x", res1, res2);
	// IOWrite32A(ois, LC89129_REG_OIS_CMD_SYSDSP_REMAP, 0x1000);
	// mdelay(30);

	ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_ON);
	//	I2COP_CHECK(ret);
	ret = RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_ON);
	//	I2COP_CHECK(ret);

	// SrvOn(ois);
	UcStRd = 1;
	UlStCnt = 0;

	// ret = RamWrite32A(ois, LC89129_REG_OIS_CTRL, 0x00000001 ) ;
	while (UcStRd && (UlStCnt++ < CNT050MS)) {
		UcStRd = RdStatus(ois, 1);
	}
	mdelay(100);

	ret = GetGyroOffset(ois, &GyroOffsetX, &GyroOffsetY, &GyroOffsetZ);
	ret = GetAcclOffset(ois, &AcclOffsetX, &AcclOffsetY, &AcclOffsetZ);
	LOG_OIS_INF("before Gyro offset cali, %x, %x, %x, Acc offset cali, %x, %x, %x",
		GyroOffsetX, GyroOffsetY, GyroOffsetZ, AcclOffsetX, AcclOffsetY, AcclOffsetZ);
	if ((res1 != 2) || (res2))
		ret = -1;
	return ret;
}

static float fix2float_16(unsigned int fix)
{
	if ((fix & 0x80000000) > 0) {
		return ((float)fix - (float)0x100000000) / (float)0x7FFFFFFF;
	} else {
		return (float)fix / (float)0x7FFFFFFF;
	}
}

static unsigned int float2fix_16(float f)
{
	if (f < 0) {
		return (unsigned int)(f * (float)0x7FFFFFFF + 0x100000000);
	} else {
		return (unsigned int)(f * (float)0x7FFFFFFF);
	}
}

//
static int LC89129_get_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	UINT_32 ReadVal = 0;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = RamRead32A(ois, GyroFilterTableX_gxzoom, &ReadVal);
	LOG_OIS_INF("gyro gainX(0x%08x , %f)", ReadVal, ReadVal);
	flash_info->imuInfo.gyroGainX = fix2float_16(ReadVal) * 10000;

	ret = RamRead32A(ois, GyroFilterTableY_gyzoom, &ReadVal);
	LOG_OIS_INF("gyro gainY(0x%08x , %f)", ReadVal, ReadVal);
	flash_info->imuInfo.gyroGainY = fix2float_16(ReadVal) * 10000;

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroGainX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("gyro gain(0x%08x, 0x%08x)",
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY);
	return ret;
}

//
static int LC89129_set_gyro_gain(struct ois *ois, __user void *user_buf)
{

	int ret = 0;
	struct i2c_client *client = ois->client;
	UINT_32 writeVal = 0;
	int gain_param[3] = { 0, };
	OIS_BUG(!user_buf);
	OIS_BUG(!client);
	LOG_OIS_INF("SetGyroGainLC89:");
	ret = copy_from_user(gain_param, user_buf, sizeof(gain_param));
	if (ret) {
		LOG_OIS_ERR("copy gain fail(%d)", ret);
		goto p_err;
	}

	if (gain_param[0] > 0) {
		writeVal = float2fix_16(gain_param[2] * 1.0 / 10000);
		ret = RamWrite32A(ois, GyroFilterTableY_gyzoom, writeVal);
		LOG_OIS_INF("Set Gyro Gain axis Y, %x", writeVal);
	} else {
		writeVal = float2fix_16(gain_param[1] * 1.0 / 10000);
		ret = RamWrite32A(ois, GyroFilterTableX_gxzoom, writeVal);
		LOG_OIS_INF("Set Gyro Gain axis X, %x", writeVal);
	}

p_err:
	return ret;
}

/*
static UINT_8 FlashBlockWrite(struct ois *ois, UINT_8 SelMat,
	UINT_32 SetAddress, UINT_32 *PulData)
{
	UINT_32 UlReadVal, UlCnt;
	UINT_8 ans = 0;
	UINT_8 i;

	if (SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2)
		return 10;
	//
	if (SetAddress > 0x000003FF)
		return 9;

	ans = UnlockCodeSet(ois);
	if (ans != 0)
		return ans;

	WritePermission(ois);
	if (SelMat != USER_MAT) {
		if (SelMat == INF_MAT2)
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3, 0x00006A4B);
		else
			IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3, 0x0000C5AD);
	}
	AdditionalUnlockCodeSet(ois);
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_ADR, ((UINT_32)SelMat << 16) | (SetAddress & 0x000010));
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);
	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 2);

	UlCnt = 0;

	for (i = 0; i < 16; i++) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLA_WDAT, PulData[i]);
	}
	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		};

		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAINT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_CMD, 8);

	do {
		if (UlCnt++ > 100) {
			ans = 2;
			break;
		};

		IORead32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAINT, &UlReadVal);
	} while ((UlReadVal & 0x00000080) != 0);

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
	ans = UnlockCodeClear(ois);
	return ans;
}*/

//
static UINT_8 WrReGyroGainData(struct ois *ois)
{
	UINT_32 UlMAT0[64];
	UINT_32 UlReadGxzoom, UlReadGyzoom;
	UINT_8 ans = 0;
	UINT_16 UsCkVal, UsCkVal_Bk ;

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000000);




	ans = RdErInfMAT(ois, INF_MAT0, UlMAT0, 64);
	if (ans) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return ans;
	}


	RamRead32A(ois, 0x8BB4, &UlReadGxzoom);
	RamRead32A(ois, 0x8BB8, &UlReadGyzoom);

	UlMAT0[15] = UlReadGxzoom;
	UlMAT0[16] = UlReadGyzoom;

	UsCkVal = MkInfMATsum(UlMAT0);
	UlMAT0[63] &= (UINT_32)0xFFFF0000;
	UlMAT0[63] |= (UINT_32)UsCkVal ;


	ans = WrInfMAT(ois, INF_MAT0, UlMAT0, 64);
	if (ans != 0) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return 3;// 0 - 63
	}

	UsCkVal_Bk = UsCkVal;
	ans = FlashMultiRead129(ois, INF_MAT0, 0, UlMAT0, 64);
	if (ans) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return 4;
	}
	UsCkVal = MkInfMATsum(UlMAT0);


	if (UsCkVal != UsCkVal_Bk) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
		return 5;
	}

	IOWrite32A(ois, LC89129_REG_OIS_CMD_FLASHROM_FLAMODE, 0x00000002);
	return 0;
}


//
static int LC89129_flash_save(struct ois *ois)
{

	int ret = 0;
	UINT_8 UcStRd = 1;
	UINT_32 UlStCnt = 0;
	UINT_32 ReadVal5 = 0, ReadVal6 = 0;
	LOG_OIS_INF("Save CalibrationData LC89.");
	ret = WrReGyroGainData(ois);
	if (0 == ret) {
		IOWrite32A(ois, LC89129_REG_OIS_CMD_SYSDSP_REMAP, 0x1000);
		mdelay(30);
		RamWrite32A(ois, LC89129_REG_OIS_SPI_INIT, 0x00000004);
		while (UcStRd && (UlStCnt++ < CNT050MS)) {
			UcStRd = RdStatus(ois, 1);
		}

		SrvOn(ois);
		UcStRd = 1;
		UlStCnt = 0;

		RamWrite32A(ois, LC89129_REG_OIS_CTRL, 0x00000003);
		while (UcStRd && (UlStCnt++ < CNT050MS)) {
			UcStRd = RdStatus(ois, 1);
		}
		mdelay(100);
		RamRead32A(ois, GyroFilterTableX_gxzoom, &ReadVal5);
		RamRead32A(ois, GyroFilterTableY_gyzoom, &ReadVal6);
		LOG_OIS_INF("Save CalibrationData sucess Gyro_gain(zoom)_x = 0x%x,Gyro_gain(zoom)_y = 0x%x.",
			ReadVal5, ReadVal6);
		return ret;
	}
	LOG_OIS_ERR("Save CalibrationData falied.");
	return 1;
}

// update distance
static int LC89129_set_acc(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_acc_param acc_info = {0};
	// u16 acc_gainx = 0x0000, acc_gainy = 0x0000;
	struct i2c_client *client = ois->client;
	u32 tx_data = 0x00000000;
	// int idx = 0;
	UINT_32 UlStCnt = 0;
	UINT_8 UcStRd = 1;
	// int midtemp;
	UINT_32 testmidtemp;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&acc_info, user_buf, sizeof(struct ois_acc_param));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	// ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &acc_gainx);

	tx_data = (u32)acc_info.currFocusDac;
	LOG_OIS_INF("UpdateSubjectDistance: m_accOn %d ", acc_info.accOn);

	if (acc_info.accOn) {
		ret = RamWrite32A(ois, 0xF01C, 0x00000001);
		while (UcStRd && (UlStCnt++ < CNT050MS)) {
			UcStRd = RdStatus(ois, 1);
		}
		LOG_OIS_INF("UpdateSubjectDistance: %d %d %x", tx_data, tx_data, tx_data);
		ret = RamWrite32A(ois, 0x8684, (UINT_32)tx_data);
		while (UcStRd && (UlStCnt++ < CNT050MS)) {
			UcStRd = RdStatus(ois, 1);
		}
		RamRead32A(ois, 0x8684, &testmidtemp);
		LOG_OIS_INF("UpdateSubjectDistance: %d %x, SetAccLC89 on", testmidtemp,
								testmidtemp);
	} else {
		ret = RamWrite32A(ois, 0xF01C, 0x00000000);
		while (UcStRd && (UlStCnt++ < CNT050MS)) {
			UcStRd = RdStatus(ois, 1);
		}
	}

p_err:
	return ret;
}

// fix
static int LC89129_set_target(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_fixmode_parameter fixmode = {
			0,
	};
	int hallx = 0, hally = 0;
	int targetx = 0, targety = 0;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);
	RamWrite32A(ois, LC89129_REG_OIS_ACC_CTRL, OIS_ACC_OFF);
	I2COP_CHECK(ret);
	RamWrite32A(ois, LC89129_REG_OIS_CTRL, OIS_ACC_OFF);
	I2COP_CHECK(ret);
	RamWrite32A(ois, LC89129_REG_SERVE_CTRL, OIS_SERVE_ON);
	I2COP_CHECK(ret);

	ret = copy_from_user(&fixmode, user_buf, sizeof(struct ois_fixmode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy target fail(%d)", ret);
		goto p_err;
	}
	switch (fixmode.target_vertex) {
	case LEFTTOP: {
		fixmode.targetX = ois->ois_otp->hallXMax;
		fixmode.targetY = ois->ois_otp->hallYMax;
		break;
	}
	case LEFTBOTTOM: {
		fixmode.targetX = ois->ois_otp->hallXMin;
		fixmode.targetY = ois->ois_otp->hallYMax;
		break;
	}
	case RIGHTTOP: {
		fixmode.targetX = ois->ois_otp->hallXMax;
		fixmode.targetY = ois->ois_otp->hallYMin;
		break;
	}
	case RIGHTBOTTOM: {
		fixmode.targetX = ois->ois_otp->hallXMin;
		fixmode.targetY = ois->ois_otp->hallYMin;
		break;
	}
	default: {
		LOG_OIS_INF("default target");
	}
	}

	LOG_OIS_INF("target set(%d %d)", fixmode.targetX, fixmode.targetY);
	targetx = fixmode.targetX;
	targety = fixmode.targetY;
	targetx = (targetx & 0x0000FFFF) << 16;
	targety = (targety & 0x0000FFFF) << 16;

	RamWrite32A(ois, 0x00DC, targetx);
	I2COP_CHECK(ret);
	RamWrite32A(ois, 0x012C, targety);
	I2COP_CHECK(ret);
	mdelay(3);
	RamRead32A(ois, 0x00DC, &hallx);
	RamRead32A(ois, 0x012C, &hally);

	LOG_OIS_INF("hall read target(0x%04x 0x%04x) hall(0x%04x 0x%04x)",
		targetx, targety, hallx, hally);

p_err:
	return ret;
}

// init data
static int LC89129_get_init_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	flash_info->readyfalg = 0;
	ret = copy_to_user(user_buf, flash_info, sizeof(struct ois_flash_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy flash info, ret(%d)\n", ret);
		goto p_err;
	}

	LOG_OIS_INF("flash info:fw(ver:0x%04x date:0x%04x type:0x%04x) gyro(type:0x%04x spi:0x%04x readEn:0x%04x) pantilt(%d,limitx:%d,limity:%d) offset(%d,%d),gain(%d,%d,%d,%d) smooth(%d %d %d)",
		flash_info->fwInfo.version, flash_info->fwInfo.date, flash_info->fwInfo.type,
		flash_info->imuInfo.imuType, flash_info->imuInfo.spiMode, flash_info->imuInfo.imuReadEn,
		flash_info->hallInfo.pantiltOn, flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY,
		flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY, flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		flash_info->smoothInfo.on, flash_info->smoothInfo.step, flash_info->smoothInfo.delay);
	LOG_OIS_INF("control info: chip(0x%04x) dsp(0x%04x) writeAthr(0x%04x) reset(0x%04x) tripod(0x%04x) mode(0x%04x) acc(0x%04x) readyflag(0x%04x)",
		flash_info->chipEn, flash_info->dspEn, flash_info->writeAuthority, flash_info->logicReset,
		flash_info->tripodFlag, flash_info->imuInfo.spiMode, flash_info->accOn, flash_info->readyfalg);
p_err:
	return ret;
}

// status check
static int LC89129_status_check(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	INT_32 ReadVal1 = 0, ReadVal2 = 0;
	INT_32 ReadVal3 = 0, ReadVal4 = 0;
	INT_32 ReadVal5 = 0, ReadVal6 = 0;
	s16 gyroX = 0, gyroY = 0, gyroZ = 0;
	s16 accX = 0, accY = 0, accZ = 0;
	s16 hallx_raw = 0, hally_raw = 0;
	s16 hallx = 0, hally = 0;
	ret = RamRead32A(ois, 0xF010, &ReadVal1);
	ret = RamRead32A(ois, 0xF011, &ReadVal2);
	ret = RamRead32A(ois, 0xF012, &ReadVal3);
	ret = RamRead32A(ois, 0xF013, &ReadVal4);
	ret = RamRead32A(ois, 0xF015, &ReadVal5);
	ret = RamRead32A(ois, 0xF01C, &ReadVal6);

	LOG_OIS_INF("OIS Status Check LC89 0xF010:%x, 0xF011:%x, 0xF012:%x, 0xF013:%x ,0xF015:%x, 0xF01C:%x ",
		ReadVal1, ReadVal2, ReadVal3, ReadVal4, ReadVal5, ReadVal6);
	// read gyro raw data
	ret = RamRead32A(ois, 0x0220, &ReadVal1);
	ret = RamRead32A(ois, 0x0224, &ReadVal2);
	ret = RamRead32A(ois, 0x0394, &ReadVal3);
	// read acc raw data
	ret = RamRead32A(ois, 0x0448, &ReadVal4);
	ret = RamRead32A(ois, 0x0474, &ReadVal5);
	ret = RamRead32A(ois, 0x04A0, &ReadVal6);

	gyroX = (s16)((ReadVal1 >> 16) & 0xFFFF);
	gyroY = (s16)((ReadVal2 >> 16) & 0xFFFF);
	gyroZ = (s16)((ReadVal3 >> 16) & 0xFFFF);
	accX = (s16)((ReadVal4 >> 16) & 0xFFFF);
	accY = (s16)((ReadVal5 >> 16) & 0xFFFF);
	accZ = (s16)((ReadVal6 >> 16) & 0xFFFF);

	LOG_OIS_INF("Gyro_raw_x = %d, Gyro_raw_y = %d, Gyro_raw_z = %d, ACC_raw_x = %d, ACC_raw_y = %d, ACC_raw_z = %d",
		gyroX, gyroY, gyroZ, accX, accY, accZ);
	// read hall data
	ret = RamRead32A(ois, 0x0110, &ReadVal1);
	ret = RamRead32A(ois, 0x0160, &ReadVal2);
	// read hall raw data
	ret = RamRead32A(ois, 0x0178, &ReadVal3);
	ret = RamRead32A(ois, 0x017C, &ReadVal4);

	// read gyro gain data
	ret = RamRead32A(ois, 0x82B8, &ReadVal5);
	ret = RamRead32A(ois, 0x8318, &ReadVal6);

	hallx = (s16)((ReadVal1 >> 16) & 0xFFFF);
	hally = (s16)((ReadVal2 >> 16) & 0xFFFF);
	hallx_raw = (s16)((ReadVal3 >> 16) & 0xFFFF);
	hally_raw = (s16)((ReadVal4 >> 16) & 0xFFFF);

	LOG_OIS_INF("hallx = %d, hally = %d, hallx_raw = %d, hally_raw = %d, Gyro_gain(zoom)_x = 0x%x,Gyro_gain(zoom)_y = 0x%x",
		hallx, hally, hallx_raw, hally_raw, ReadVal5, ReadVal6);

	return ret;
}

// otp
static int LC89129_format_otp_data(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u8 *otp_buf = NULL;
	u8 *sn_data = NULL;
	u8 *fuse_id = NULL;
	u8 data_size = 0;
	u8 idx = 0;
	struct ois_otp_info *ois_otp = ois->ois_otp;
	s16 s16_data = 0x0000;
	// INT_16 				int16_data	= 0x0000;

	OIS_BUG(!user_buf);

	ois_otp->inited = 0x00;

	otp_buf = vivo_otp_data_s5kgh1sm24pd2083;

	ois_otp->fwVersion = otp_buf[0x0018];
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x33DB);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x33DF);
	ois_otp->gyroOffsetX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33D3);
	// ois_otp->gyroOffsetX = s16_data;
	ois_otp->gyroOffsetY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33D7);
	// ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E80);
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E82);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33E7);
	ois_otp->hallXMin = s16_data;
	/*if(ois_otp->hallXMin > 0x7FFF)
					ois_otp->hallXMin -= 0xFFFF;*/
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33EF);
	ois_otp->hallYMin = s16_data;
	/*if(ois_otp->hallYMin > 0x7FFF)
					ois_otp->hallYMin -= 0xFFFF;*/
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33E3);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x33EB);
	ois_otp->hallYMax = s16_data;
	// ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E80);
	// ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E82);
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x343B);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x343D);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x343F);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x3441);

	data_size = 0x0032 - 0x0021 + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0021 + idx]);

	data_size = 0x0051 - 0x0046 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
		LOG_OIS_ERR("sn data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0046 + idx]);

	ret = copy_to_user(user_buf, ois_otp, sizeof(struct ois_otp_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy otp info, ret(%d)\n", ret);
		goto p_err;
	}

	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(0x%x,0x%x,%d,0x%x,0x%x,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX,
		ois_otp->gyroGainY, ois_otp->gyroOffsetX, ois_otp->gyroOffsetY,
		ois_otp->hallXMin, ois_otp->hallXMax, ois_otp->hallMechCenterX,
		ois_otp->hallYMin, ois_otp->hallYMax, ois_otp->hallMechCenterY,
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);
p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
	return ret;
}

static int SetSinWaveParam(struct ois *ois, int frequency, int mode)
{
	UINT_32 frequency_t;

	/*if(frequency > 16 )
		frequency = 16 ;
	else if(frequency < 0)
		frequency = 0 ;*/
	frequency_t = frequency_DEC[frequency];

	if (mode == 255) {
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_Phase, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_Phase, 0x20000000);
	} else {
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_Phase, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_Phase, 0x00000000);
	}

	if (frequency_t == 0xFFFFFFFF) {
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_Offset, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_Phase, 0x00000000);

		RamWrite32A(ois, LC89129_REG_OIS_CosWave_Offset, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_Phase, 0x00000000);

		RamWrite32A(ois, LC89129_REG_OIS_SinWaveC_Regsiter, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_OutAddr, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_OutAddr, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_FIX_TARGET_X, 0x00000000);
		RamWrite32A(ois, LC89129_REG_OIS_FIX_TARGET_Y, 0x00000000);
	} else {
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_Offset, frequency_t);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_Offset, frequency_t);

		RamWrite32A(ois, LC89129_REG_OIS_SinWaveC_Regsiter, 0x00000001);
		RamWrite32A(ois, LC89129_REG_OIS_SinWave_OutAddr, 0x000000DC);
		RamWrite32A(ois, LC89129_REG_OIS_CosWave_OutAddr, 0x0000012C);
	}
	return 0;
}

// sin mode
static int LC89129_set_sinewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_sinemode_parameter sine = {0,};
	int frequency = 0x0000;
	int amplitudex = 0;
	int amplitudey = 0;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&sine, user_buf, sizeof(struct ois_sinemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy sine params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("sine set(%d %d %d %d)", sine.axisEn, sine.frequency, sine.amplitudeX, sine.amplitudeY);
	frequency = sine.frequency;
	RamRead32A(ois, LC89129_REG_OIS_SinWave_Gain, &amplitudex);
	RamRead32A(ois, LC89129_REG_OIS_CosWave_Gain, &amplitudey);
	if ((amplitudex) || (amplitudey)) {
		LOG_OIS_ERR("LC89129_set_sinewave skip(%d %d %d %d)", sine.axisEn, sine.frequency, amplitudex, amplitudey);
		return ret;
	}
	// return ret;
	amplitudex = (sine.amplitudeX & 0x0000FFFF) << 16;
	amplitudey = (sine.amplitudeY & 0x0000FFFF) << 16;
	RamWrite32A(ois, LC89129_REG_OIS_SinWave_Gain, amplitudex); // 7fffffff-8000
	RamWrite32A(ois, LC89129_REG_OIS_CosWave_Gain, amplitudey); // 7fffffff-8000
	SetSinWaveParam(ois, frequency, 255);

p_err:
	return ret;
}

// stroke limit
static int LC89129_set_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	return ret;
}

// set pantilt
static int LC89129_set_pantilt(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	return ret;
}

static int LC89129_reset(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int LC89129_ready_check(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int LC89129_set_tripod(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	return ret;
}
static int LC89129_set_smooth(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	return ret;
}

static void LC89129_vsync_process_fn(struct kthread_work *work)
{
	int result = 0;
	struct ois *ois = NULL;
	struct i2c_client *client = NULL;
	// u16 sample_ready = 0x0000;
	struct ois_lens_info *lens_info = NULL;
	u8 insert_idx = 0;
	u8 data[60] = {
			0,
	};
	u16 data_idx = 0, group_idx = 0;
	//	u16 valid_size = 0;
	struct timespec start_time, end_time;

	OIS_BUG_VOID(!work);

	LOG_OIS_VERB("ois vsync processor E");

	ois = container_of(work, struct ois, vsync_work);
	OIS_BUG_VOID(!ois);
	client = ois->client;
	OIS_BUG_VOID(!client);
	OIS_BUG_VOID(!work);
	OIS_BUG_VOID(!(ois->lens_info_buf));

	insert_idx = ois->lens_info_buf->insertidx;
	lens_info = &ois->lens_info_buf->buf[insert_idx];

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	memset(lens_info, 0x00, sizeof(struct ois_lens_info));
	// check if data ready and read first packet(62 bytes)
	/*result = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_READY, &sample_ready);
	result = ois_i2c_read(client, DW9781C_REG_LENS_INFO_START, (u16 *)(&data[0]));
	valid_size = data[0] * 6 + 2;
	if (0x01 != sample_ready || !data[0] || valid_size >
	DW9781C_MAX_LENS_INFO_SIZE) {
					LOG_OIS_INF("skip: sample_ready=%d valid size=%d", sample_ready,
	valid_size);
					goto p_err;
	}*/
	lens_info->validnum = 10;
	lens_info->fcount = ois->vsync_info[0].ois_vsync_cnt;
	lens_info->timestampboot = ois->vsync_info[0].sof_timestamp_boot;
	/*LOG_OIS_INF("info ready=0x%04x, isp vsync %llu fcount=%llu ts=%llu
		 validnum=%d insert=%d",
					sample_ready, ois->vsync_info[0].vsync_cnt, lens_info->fcount,
		 lens_info->timestampboot, lens_info->validnum, insert_idx);*/
	ois->vsync_info[0].ois_vsync_cnt++;

	// if there are more data
	/*while (data_idx < valid_size - 1) {
					result = ois_i2c_read_block(client, DW9781C_REG_LENS_INFO_START,
	&data[data_idx], DW9781C_LENS_PACKET_SIZE);
					result = ois_i2c_write(ois->client, DW9781C_REG_INFO_SAMPLE_READY,
	OFF);//inform ic one packet read done
					I2COP_CHECK(result);
					//mdelay(1);
					data_idx += DW9781C_LENS_PACKET_SIZE;
	}*/
	mdelay(3);
	result = ois_i2c_read_block(client, 0xF111, data, 60);
	I2COP_CHECK(result);

	// result = RegOperation(OperationType::READ, &readSize, 1, 0xF111, 0, 1, 2,
	// (UINT8*)readBuf);
	LOG_OIS_VERB("read done(idx=%d)", data_idx);

	for (group_idx = 0, data_idx = 0; group_idx < 10; group_idx++) {
		lens_info->ic_timecount[group_idx] =
				(s16)((data[data_idx + 1] << 8) | (data[data_idx])); // onvert to us
		if (lens_info->ic_timecount[group_idx] == 0) {
			lens_info->validnum--;
		}
		data_idx += 2;
		lens_info->hallx[group_idx] =
				(s16)((data[data_idx + 1] << 8) | (data[data_idx]));
		data_idx += 2;
		lens_info->hally[group_idx] =
				(s16)((data[data_idx + 1] << 8) | (data[data_idx]));
		data_idx += 2;
		LOG_OIS_INF("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
			lens_info->fcount, group_idx, lens_info->ic_timecount[group_idx],
			lens_info->hallx[group_idx], lens_info->hally[group_idx]);
	}

	if (++ois->lens_info_buf->insertidx == LENS_INFO_FRAMES_MAX)
		ois->lens_info_buf->insertidx = 0;

	ktime_get_ts(&end_time);

	LOG_OIS_VERB("ois vsync processor X %llu ms	vnum:%d",
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)), lens_info->validnum);
p_err:
	mutex_unlock(&ois->op_lock);
	return;
}

static int LC89129_vsync_signal(struct ois *ois, void *buf)
{
	int ret = 0;
	struct timespec curtime;
	struct ois_vsync_info *tempInfo = NULL;

	OIS_BUG(!buf);
	OIS_BUG(!(ois->lens_info_buf));

	tempInfo = (struct ois_vsync_info *)buf;
	curtime = ktime_to_timespec(tempInfo->sof_timestamp_boot);
	ois->vsync_info[0].sof_timestamp_boot = (u64)curtime.tv_sec * 1000000000 + curtime.tv_nsec;
	ois->vsync_info[0].vsync_cnt = tempInfo->vsync_cnt;

	if (ois->vsync_task != NULL) {
		kthread_queue_work(&ois->vsync_worker, &ois->vsync_work);
	}

	LOG_OIS_INF("isp vsync %llu timestamp %llu",
		ois->vsync_info[0].vsync_cnt, ois->vsync_info[0].sof_timestamp_boot);

	return ret;
}

static int LC89129_init_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (ois->vsync_task == NULL) {
		spin_lock_init(&ois->ois_vsync_lock);
		kthread_init_work(&ois->vsync_work, LC89129_vsync_process_fn);
		kthread_init_worker(&ois->vsync_worker);
		ois->vsync_task = kthread_run(kthread_worker_fn, &ois->vsync_worker, "vsync_processor");
		if (NULL == ois->vsync_task) {
			LOG_OIS_ERR("failed to create vsync processor, err(%ld)", PTR_ERR(ois->vsync_task));
			ret = PTR_ERR(ois->vsync_task);
			ois->vsync_task = NULL;
			goto p_err;
		}
	}
	LOG_OIS_INF("start ois vsync processor success");
p_err:
	return ret;
}

static int LC89129_deinit_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (NULL != ois->vsync_task) {
		if (kthread_stop(ois->vsync_task)) {
			LOG_OIS_ERR("vsync processor stop fail");
			goto p_err;
		}
		ois->vsync_task = NULL;
	}
	LOG_OIS_INF("stop ois vsync processor success");
p_err:
	return ret;
}

static int LC89129_get_lens_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u8 idx = 0;
	struct timespec start_time, end_time;
	// struct ois_lens_info_buf info_buf = {0};

	OIS_BUG(!user_buf);

	// get hal frame id
	// ret = copy_from_user(&info_buf, user_buf, sizeof(struct
	// ois_lens_info_buf));
	// LOG_OIS_INF("read hal frame %llu", info_buf.buf[0].frame_id);

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	ret = copy_to_user(user_buf, ois->lens_info_buf, sizeof(struct ois_lens_info_buf));
	if (ret) {
		LOG_OIS_ERR("fail to copy lens info, ret(%d)\n", ret);
	}

	ktime_get_ts(&end_time);
	for (idx = 0, idx = 2; idx < LENS_INFO_GROUPS_MAX; idx++)
		LOG_OIS_INF("fcount %lu data[0] timestamp %lu hallx %d hally %d",
			ois->lens_info_buf->buf[idx].fcount,
			ois->lens_info_buf->buf[idx].ic_timecount[0],
			ois->lens_info_buf->buf[idx].hallx[0],
			ois->lens_info_buf->buf[idx].hally[0]);

	LOG_OIS_INF("lens info copy done %d(%llums)",
		ret, (((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));

	mutex_unlock(&ois->op_lock);
	return ret;
}

static struct ois_core_ops LC89129_ois_ops = {
	.ois_init = LC89129_init,
	.ois_init_slave = LC89129_init_slave,
	.ois_deinit = LC89129_deinit,
	.ois_stream_on = LC89129_stream_on,
	.ois_stream_off = LC89129_stream_off,
	.ois_get_mode = LC89129_get_mode,
	.ois_set_mode = LC89129_set_mode,
	.ois_fw_update = LC89129_fw_update,
	.ois_get_fw_version = LC89129_get_fw_version,
	.ois_get_gyro_offset = LC89129_get_gyro_offset,
	.ois_set_offset_calibration = LC89129_set_offset_calibration,
	.ois_get_gyro_gain = LC89129_get_gyro_gain,
	.ois_set_gyro_gain = LC89129_set_gyro_gain,
	.ois_flash_save = LC89129_flash_save,
	.ois_set_acc = LC89129_set_acc,
	.ois_set_target = LC89129_set_target,
	.ois_get_init_info = LC89129_get_init_info,
	.ois_status_check = LC89129_status_check,
	.ois_init_vsync_thread = LC89129_init_vsync_thread,
	.ois_deinit_vsync_thread = LC89129_deinit_vsync_thread,
	.ois_vsync_signal = LC89129_vsync_signal,
	.ois_get_lens_info = LC89129_get_lens_info,
	.ois_format_otp_data = LC89129_format_otp_data,
	.ois_set_sinewave = LC89129_set_sinewave,
	.ois_set_stroke_limit = LC89129_set_stroke_limit,
	.ois_set_pantilt = LC89129_set_pantilt,
	.ois_reset = LC89129_reset,
	.ois_ready_check = LC89129_ready_check,
	.ois_set_smooth = LC89129_set_smooth,
	.ois_set_tripod = LC89129_set_tripod,
	.ois_log_control = LC89129_log_control,
};

/*ic entry expose to ois_core*/
void LC89129_get_ops(struct ois *ois) { ois->ops = &LC89129_ois_ops; }

