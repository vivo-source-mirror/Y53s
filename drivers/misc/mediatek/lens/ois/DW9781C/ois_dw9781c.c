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

#include "ois_dw9781c.h"

//current specific fw
const struct ois_fw_info EXP_FW = {
	.version = 0x0308,
	.type    = FW_VIVO,
	.date    = 0x0420
};

#define GAIN_TABLE_SIZE_MAX 100
//#define VIVO_OTP_DATA_SIZE 0x2F40
//extern unsigned char vivo_otp_data_s5kgw3sp13[VIVO_OTP_DATA_SIZE];
#define VIVO_OTP_DATA_SIZE 0x2EE8 
extern unsigned char vivo_otp_data_s5kgw3sp13pd2083[VIVO_OTP_DATA_SIZE];
extern int log_ois_level;

//unsigned char vivo_otp_data_s5kgw3sp13pd2062[VIVO_OTP_DATA_SIZE] = {0,};
static int dw9781c_acc_gain_table[GAIN_TABLE_SIZE_MAX] = {
    0,     0,     0,     0,     0,     0,     0,     0,     27000, 24000,
    21600, 19636, 18000, 16615, 15429, 14400, 13500, 12706, 12000, 11368,
    10800, 10286, 9818,  9391,  9000,  8640,  8308,  8000,  7714,  7448,
    7200,  6968,  6750,  6545,  6353,  6171,  6000,  5838,  5684,  5538,
    5400,  5268,  5143,  5023,  4909,  4800,  4696,  4596,  4500,  4408,
    4320,  4235,  4154,  4075,  4000,  3927,  3857,  3789,  3724,  3661,
    3600,  3541,  3484,  3429,  3375,  3323,  3273,  3224,  3176,  3130,
    3086,  3042,  3000,  2959,  2919,  2880,  2842,  2805,  2769,  2734,
    2700,  2667,  2634,  2602,  2571,  2541,  2512,  2483,  2455,  2427,
    2400,  2374,  2348,  2323,  2298,  2274,  2250,  2227,  2204,  2182
};

static int dw9781c_log_control(struct ois *ois, int level)
{
	int ret = 0;

	if (level > OIS_LOG_START && level < OIS_LOG_END)
		log_ois_level = level;

	LOG_OIS_INF("log %d", log_ois_level);
	return ret;
}

static int dw9781c_reset(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	/*1.logic circuit reset*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
	I2COP_CHECK(ret);
	mdelay(4);
	/*2.dsp on, wait 25ms for gyro data stable*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, ON);
	I2COP_CHECK(ret);
	mdelay(25);
	/*3.get user write register authority*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_USER_WRITE_PROTECT, USER_WRITE_EN);
	I2COP_CHECK(ret);
	mdelay(10);

	LOG_OIS_INF("ois reset success(%d)", ret);

p_err:
	return ret;
}

/*temp function*/
static int dw9781c_determine_spi_mode(struct ois *ois)
{
	int ret         = 0;
	u16 spi_mode    = 0x0000;
	u16 fw_type     = 0x0000;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	LOG_OIS_INF("E");
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
	//if (fw_type == FW_VIVO) 
	{
		ret = ois_i2c_write(client, DW9781C_REG_SPI_MODE, OIS_SPI_MASTER);
		mdelay(55);
		I2COP_CHECK(ret);
		/*ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &spi_mode);
		if (OIS_GYRO_INITED != spi_mode) {
			LOG_OIS_ERR("gyro init failed(0x%04x ret=%d)", spi_mode, ret);
			goto p_err;
		}*/
		ret = ois_i2c_write(client, DW9781C_REG_GYRO_READ_CTRL, ON);
		I2COP_CHECK(ret);
	} 
    //else
		//goto p_err;

	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &spi_mode);
	LOG_OIS_INF("X spiMode:0x%04x ret(%d)", spi_mode, ret);

p_err:
	return ret;
}

static int dw9781c_set_mode(struct ois *ois, int mode)
{
	int ret       = 0;
	u16 exp_mode  = 0x0000;
	u16 old_mode  = 0x0000;
	u16 new_mode  = 0x0000;
	u16 servo     = 0x0000;
	u16 op_status = OPERATE_DONE;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &old_mode);

	switch (mode) {
	case OIS_CENTER_MODE: {
		exp_mode = DW9781C_CENTERING_MODE;
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINX, OFF);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINY, OFF);
		I2COP_CHECK(ret);
		break;
	}
	case OIS_STILL_MODE: {
		exp_mode = DW9781C_STILL_MODE;
		I2COP_CHECK(ret);
		break;
	}
	case OIS_VIDEO_MODE: {
		exp_mode = DW9781C_VIDEO_MODE;
		break;
	}
	case OIS_ZOOM_MODE: {
		exp_mode = DW9781C_ZOOM_MODE;
		break;
	}
	default: {
		LOG_OIS_INF("unsupport ois mode(%d)", mode);
		goto p_err;
	}
	}

	if (0x0000 != exp_mode && old_mode != exp_mode) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, exp_mode);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_CMD_STATUS, OPERATE_START);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CMD_STATUS, &op_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &new_mode);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &servo);
	ois->flash_info->mode = new_mode;
	ois->flash_info->serveOn = servo;

	LOG_OIS_INF("mode(old:0x%04x exp:0x%04x new:0x%04x servo:0x%04x), op status(0x%04x) result(%d)",
		old_mode, exp_mode, new_mode, servo, op_status, ret);

p_err:
	return ret;
}

static int dw9781c_start_ois(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	ret = dw9781c_determine_spi_mode(ois);
	I2COP_CHECK(ret);

	/*enable af drift detect
	ret = ois_i2c_write(client, DW9781C_REG_OIS_AF_DRIFT_COMP, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	*/
	//enable tripod detect
	ret = ois_i2c_write(client, DW9781C_REG_OIS_LOOPGAIN_DETECT, ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_TRIPODE_CTRL, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_EN, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, OIS_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, DW9781C_STILL_MODE);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CMD_STATUS, OPERATE_START);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINX, OFF);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINY, OFF);
	LOG_OIS_INF("ois restart success(%d)", ret);

p_err:
	return ret;
}

static int dw9781c_get_init_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	u16 u16_rxdata = 0x0000;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &u16_rxdata);
	flash_info->fwInfo.version = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &u16_rxdata);
	flash_info->fwInfo.type = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &u16_rxdata);
	flash_info->fwInfo.date = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_CTRL, &u16_rxdata);
	flash_info->hallInfo.pantiltOn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &u16_rxdata);
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_IMU_SELECT, &u16_rxdata);
	flash_info->imuInfo.imuType = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINX, &s16_rxdata);
	flash_info->imuInfo.gyroGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINY, &s16_rxdata);
	flash_info->imuInfo.gyroGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_CTRL, &u16_rxdata);
	LOG_OIS_INF("len info sample ctrl 0x%04x", u16_rxdata);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &u16_rxdata);
	flash_info->smoothInfo.on = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_STEP, &u16_rxdata);
	flash_info->smoothInfo.step = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_DELAY, &u16_rxdata);
	flash_info->smoothInfo.delay = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAIN_DETECT, &u16_rxdata);
	flash_info->DetectLoopgainFlag = u16_rxdata;

	if (OFF != flash_info->imuInfo.accGainX)
		flash_info->accOn = ON;
	flash_info->readyfalg = ois->ready_check;

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
	LOG_OIS_INF("control info: chip(0x%04x) dsp(0x%04x) writeAthr(0x%04x) reset(0x%04x) tripod(0x%04x) mode(0x%04x) acc(0x%04x) readyflag(0x%04x) loopgain flag(0x%04x)",
		flash_info->chipEn, flash_info->dspEn, flash_info->writeAuthority, flash_info->logicReset,
		flash_info->tripodFlag, flash_info->imuInfo.spiMode, flash_info->accOn, flash_info->readyfalg, flash_info->DetectLoopgainFlag);

p_err:
	return ret;
}

static int dw9781c_flash_save(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	//save info
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FLASH_SAVE);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(100);

	//reset ois
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);

	//restart ois
	ret = dw9781c_start_ois(ois);

	LOG_OIS_INF("ois flash save success(%d)", ret);
p_err:
	return ret;
}

static int dw9781c_set_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	u16 pantiltx = 0x0000, pantilty = 0x0000;

	OIS_BUG(!client);

	ret = ois_i2c_write(client, 0x725C, 1200);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FLASH_SAVE);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(100);

	//reset ois
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);

	//I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &pantiltx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &pantilty);
	LOG_OIS_INF("limit %d %d\n", pantiltx, pantilty);
	p_err:

	return ret;
}

static int dw9781c_init(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;
	u16 fw_checksum_flag = 0x0000, checksum_cal = 0x0000, checksum_exp = 0x0000;

	LOG_OIS_INF("E");

	OIS_BUG(!client);
	ois->client->addr = DW7981C_SLAVE_ADDR >> 1;
	ois->ready_check = 0;

	//1.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//2.flash checksum flag check
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_FLAG, &fw_checksum_flag);
	if (FW_CHECKSUM_FLAG != fw_checksum_flag) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x", fw_checksum_flag);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		mdelay(4);
		I2COP_CHECK(ret);
		ois->ready_check = 1;
	}

	/*3.get corret i2c addr*/
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	/*4.flash checksum check*/
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_EXP_CHECKSUM, &checksum_exp);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM, &checksum_cal);
	if (checksum_cal != checksum_exp) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x checksum exp 0x%04x checksum cal 0x%04x",
			fw_checksum_flag, checksum_exp, checksum_cal);
		ois->ready_check = 1;
	}

	/*5.logic circuit reset*/
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	/*enable o+e data sample*/
	ret = ois_i2c_write(client, DW9781C_REG_INFO_SAMPLE_CTRL, ((ENABLE_HALL << 8) | DW9871C_DEFAULT_SAMPLE_FREQ));
	I2COP_CHECK(ret);
	/*6.retsart ois*/
	ret = dw9781c_start_ois(ois);
	I2COP_CHECK(ret);
	LOG_OIS_INF("X ready_check 0x%04x", ois->ready_check);

p_err:
	return ret;
}

static int dw9781c_init_slave(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;
	u16 spi_mode	= 0x0000;
	u16 fw_checksum_flag = 0x0000, checksum_cal = 0x0000, checksum_exp = 0x0000;
	u16 u16_rxdata;
	LOG_OIS_INF("E");

	OIS_BUG(!client);

	//1.get corret i2c addr
	ois->client->addr = DW7981C_SLAVE_ADDR >> 1;
	//
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &u16_rxdata);
	if(u16_rxdata) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINX, OFF);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINY, OFF);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, DW9781C_CENTERING_MODE);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_CMD_STATUS, OPERATE_START);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_GYRO_READ_CTRL, 0x0000); /* Stop To Read Gyro*/
		mdelay(10);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_SPI_MODE, 0x0002); /* SPI Input*/
		I2COP_CHECK(ret);
		mdelay(55);
		LOG_OIS_INF("ois dw9781c_init_slave success(%d)", ret);
		return ret;

	}
	ois->ready_check = 0;
	//1.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//2.flash checksum flag check
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_FLAG, &fw_checksum_flag);
	if (FW_CHECKSUM_FLAG != fw_checksum_flag) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x", fw_checksum_flag);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		mdelay(4);
		I2COP_CHECK(ret);
		ois->ready_check = 1;
	}

	//3.get corret i2c addr
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	//4.flash checksum check
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_EXP_CHECKSUM, &checksum_exp);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM, &checksum_cal);
	if (checksum_cal != checksum_exp) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x checksum exp 0x%04x checksum cal 0x%04x",
			fw_checksum_flag, checksum_exp, checksum_cal);
		ois->ready_check = 1;
	}

	//5.logic circuit reset
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	//enable o+e data sample
	ret = ois_i2c_write(client, DW9781C_REG_INFO_SAMPLE_CTRL, ((ENABLE_HALL << 8) | DW9871C_DEFAULT_SAMPLE_FREQ));
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_SPI_MODE, OIS_SPI_INPUT);
	mdelay(55);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &spi_mode);
	LOG_OIS_INF("X spiMode:0x%04x ret(%d)", spi_mode, ret);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, OIS_ON);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINX, OFF);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINY, OFF);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, DW9781C_CENTERING_MODE);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CMD_STATUS, OPERATE_START);
	I2COP_CHECK(ret);

	LOG_OIS_INF("ois dw9781c_init_slave success(%d)", ret);
p_err:
	return ret;
}

static int dw9781c_deinit(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;

	LOG_OIS_INF("E");

	OIS_BUG(!client);

	/*2.servo off*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, SERVO_OFF);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_GYRO_READ_CTRL, 0x0000); /* Stop To Read Gyro*/
	mdelay(10);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_SPI_MODE, 0x0002); /* SPI Input*/
	I2COP_CHECK(ret);
	mdelay(55);

	LOG_OIS_INF("X(%d)", ret);
p_err:
	return ret;
}

static int dw9781c_stream_on(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int dw9781c_stream_off(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static void dw9781c_vsync_process_fn(struct kthread_work *work)
{
	int result = 0;
	struct ois *ois = NULL;
	struct i2c_client *client = NULL;
	u16 sample_ready = 0x0000;
	struct ois_lens_info *lens_info = NULL;
	u8 insert_idx = 0;
	u8 invalidnum = 0;
	u8 data[DW9781C_MAX_LENS_INFO_SIZE] = {0, };
	u16 data_idx = 0, group_idx = 0;
	u16 valid_size = 0;
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
	//check if data ready and read first packet(62 bytes)
	result = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_READY, &sample_ready);
	result = ois_i2c_read(client, DW9781C_REG_LENS_INFO_START, (u16 *)(&data[0]));
	valid_size = data[0] * 6 + 2;
	if (0x01 != sample_ready || !data[0] || valid_size > DW9781C_MAX_LENS_INFO_SIZE) {
		LOG_OIS_INF("skip: sample_ready=%d valid size=%d", sample_ready, valid_size);
		goto p_err;
	}

	lens_info->validnum = (data[0] > LENS_INFO_GROUPS_MAX) ? LENS_INFO_GROUPS_MAX : data[0];
	lens_info->fcount = ois->vsync_info[0].ois_vsync_cnt;
	lens_info->timestampboot = ois->vsync_info[0].sof_timestamp_boot;
	LOG_OIS_VERB("info ready=0x%04x, isp vsync %llu fcount=%llu ts=%llu validnum=%d insert=%d",
		sample_ready, ois->vsync_info[0].vsync_cnt, lens_info->fcount, lens_info->timestampboot, lens_info->validnum, insert_idx);
	ois->vsync_info[0].ois_vsync_cnt++;

	//if there are more data
	while (data_idx < valid_size - 1) {
		result = ois_i2c_read_block(client, DW9781C_REG_LENS_INFO_START, &data[data_idx], DW9781C_LENS_PACKET_SIZE);
		result = ois_i2c_write(ois->client, DW9781C_REG_INFO_SAMPLE_READY, OFF);//inform ic one packet read done
		I2COP_CHECK(result);
		//mdelay(1);
		data_idx += DW9781C_LENS_PACKET_SIZE;
	}

	//LOG_OIS_INF("read done(idx=%d)", data_idx);

	for (group_idx = 0, data_idx = 2; group_idx < lens_info->validnum; group_idx++) {
		lens_info->ic_timecount[group_idx] = (!group_idx) ? (100 * (OIS_BIG_ENDIAN_TRANS2(data, data_idx))) :
			(OIS_BIG_ENDIAN_TRANS2(data, data_idx));//onvert to us
		data_idx += 2;
		lens_info->hallx[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		lens_info->hally[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		if ((lens_info->ic_timecount[group_idx] == 0) && (group_idx != 0)) {
			invalidnum++;
			LOG_OIS_INF("invalid validnum -1");
		}

		LOG_OIS_INF("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
			lens_info->fcount, group_idx,  lens_info->ic_timecount[group_idx],
			lens_info->hallx[group_idx], lens_info->hally[group_idx]);
	}
	lens_info->validnum = lens_info->validnum - invalidnum;

	if (++ois->lens_info_buf->insertidx == LENS_INFO_FRAMES_MAX)
			ois->lens_info_buf->insertidx = 0;

	ktime_get_ts(&end_time);

	LOG_OIS_VERB("ois vsync processor X %llu ms",
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));
p_err:
	mutex_unlock(&ois->op_lock);
	return;
}

static int dw9781c_vsync_signal(struct ois *ois, void *buf)
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

	LOG_OIS_VERB("isp vsync %llu timestamp %llu",
		ois->vsync_info[0].vsync_cnt, ois->vsync_info[0].sof_timestamp_boot);

	return ret;
}

static int dw9781c_init_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (ois->vsync_task == NULL) {
		spin_lock_init(&ois->ois_vsync_lock);
		kthread_init_work(&ois->vsync_work, dw9781c_vsync_process_fn);
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

static int dw9781c_deinit_vsync_thread(struct ois *ois)
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

static int dw9781c_get_lens_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	//u8  idx = 0;
	struct timespec start_time, end_time;
	//struct ois_lens_info_buf info_buf = {0};

	OIS_BUG(!user_buf);

	//get hal frame id
	//ret = copy_from_user(&info_buf, user_buf, sizeof(struct ois_lens_info_buf));
	//LOG_OIS_INF("read hal frame %llu", info_buf.buf[0].frame_id);

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	ret = copy_to_user(user_buf, ois->lens_info_buf, sizeof(struct ois_lens_info_buf));
	if (ret) {
		LOG_OIS_ERR("fail to copy lens info, ret(%d)\n", ret);
	}

	ktime_get_ts(&end_time);
	/*
    for (idx = 0, idx = 2; idx < LENS_INFO_GROUPS_MAX; idx++)
		LOG_OIS_INF("fcount %lu data[0] timestamp %lu hallx %d hally %d",
			ois->lens_info_buf->buf[idx].fcount, ois->lens_info_buf->buf[idx].ic_timecount[0],
			ois->lens_info_buf->buf[idx].hallx[0], ois->lens_info_buf->buf[idx].hally[0]);
	*/
	LOG_OIS_VERB("lens info copy done %d(%llums)", ret,
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));

	mutex_unlock(&ois->op_lock);
	return ret;
}

static int dw9781c_get_fw_version(struct ois *ois, __user void *user_buf)
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

static int dw9781c_get_gyro_offset(struct ois *ois, __user void *user_buf)
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

static int dw9781c_get_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroGainX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("gyro gain(0x%08x, 0x%08x)", flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY);
	return ret;
}

static int dw9781c_get_mode(struct ois *ois, __user void *user_buf)
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
static int dw9781c_set_smooth(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_smooth_info smooth = { 0, };
	u16 on = 0x0000;

	OIS_BUG(!client);

	ret = copy_from_user(&smooth, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &on);
	if (smooth.on != on) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_EN, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_STEP, smooth.step);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_DELAY, smooth.delay);
		I2COP_CHECK(ret);
	}
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &on);

	LOG_OIS_INF("smooth(%d %d %d) read(%d)", smooth.on, smooth.step, smooth.delay, on);
p_err:
	return ret;
}

static int dw9781c_set_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int gain_param[3] =  { 0, };
	u16 tx_data = 0x0000;
	s16 rx_data = 0x0000;
	u16 addr    = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(gain_param, user_buf, sizeof(gain_param));
	if (ret) {
		LOG_OIS_ERR("copy gain fail(%d)", ret);
		goto p_err;
	}

	if (gain_param[0] > 0) {
		addr = DW9781C_REG_OIS_GYRO_GAINY;
		tx_data = (u16)(gain_param[2]);
	} else {
		addr = DW9781C_REG_OIS_GYRO_GAINX;
		tx_data = (u16)(gain_param[1]);
	}

	ret = ois_i2c_write(client, addr, tx_data);
	I2COP_CHECK(ret);
	ret = dw9781c_flash_save(ois);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, addr, &rx_data);

	LOG_OIS_INF("gain param(%d %d %d), set(addr=0x%04x value=%d) read %d(0x%04x)",
		gain_param[0], gain_param[1], gain_param[2], addr, tx_data, rx_data, rx_data);

p_err:
	return ret;
}

static int dw9781c_set_offset_calibration(struct ois *ois)
{
	int ret = 0;
	u8 read_max = 10;
	u8 idx = 0;
	u16 cal_status = 0x0000;
	s16 offsetx = 0x0000;
	s16 offsety = 0x0000;
	s16 gyro_rawx = 0x0000;
	s16 gyro_rawy = 0x0000;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	LOG_OIS_INF("E");

	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &offsety);
	LOG_OIS_INF("before cal offset(0x%04x 0x%04x) ret(%d)", offsetx, offsety, ret);

	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_GYRO_OFFSET_CAL);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(100);

	for (idx = 0, read_max = 10; idx < 2; idx++) {
		while (1) {
			ret = ois_i2c_read(client, DW9781C_REG_OIS_GYROOFFSET_CAL_STS, &cal_status);
			if (REG_DEAFULT_ON & cal_status) {
				LOG_OIS_INF("offset calibration done(0x%04x)", cal_status);
				break;
			} else if (--read_max) {
				LOG_OIS_INF("cal status(0x%04x)  gyro(0x%04x 0x%04x) read retry(%d)", cal_status, gyro_rawx, gyro_rawy, read_max);
				mdelay(50);
			} else {
				LOG_OIS_INF("offset cal timeout(%d,%d)", ret, read_max);
				break;
			}
		}
		//if cal fail,try again
		if (cal_status & REG_DEAFULT_ON) {
			break;
		} else {
			LOG_OIS_INF("offset cal failed(try=%d err_msg=0x%04x)", idx, cal_status);
		}
	}

	if (!(cal_status & REG_DEAFULT_ON))
		goto p_err;

	ret = dw9781c_flash_save(ois);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &offsety);
	flash_info->imuInfo.gyroOffsetX = offsetx;
	flash_info->imuInfo.gyroOffsetY = offsety;

	LOG_OIS_INF("X: after cal offset(%d %d) ret(%d)", offsetx, offsety, ret);

p_err:
	return ret;
}

static int dw9781c_set_acc(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_acc_param acc_info = {0};
	u16 acc_gainx = 0x0000, acc_gainy = 0x0000;
	struct i2c_client *client = ois->client;
	u16 tx_data = 0x0000;
	int idx = 0;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&acc_info, user_buf, sizeof(struct ois_acc_param));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &acc_gainx);

	tx_data = (u16)acc_info.currFocusDistance;
	idx = tx_data / 10;
	if (idx > GAIN_TABLE_SIZE_MAX) {
		LOG_OIS_INF("out of gain table, skip");
		goto p_err;
	}
	tx_data = dw9781c_acc_gain_table[idx];
	//convert to acc gain
	if (1 == acc_info.accOn && acc_gainx != tx_data) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINX, tx_data);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINY, tx_data);
		I2COP_CHECK(ret);
	} else if (0 == acc_info.accOn) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINX, OFF);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINY, OFF);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &acc_gainx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &acc_gainy);

	LOG_OIS_INF("acc info set(%d %d) write(%d) read(%d %d) ret(%d)",
		acc_info.accOn, acc_info.currFocusDistance, tx_data, acc_gainx, acc_gainy, ret);

p_err:
	return ret;
}

static int dw9781c_set_tripod(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int on = 0x0000;
	u16 rx_data = 0x0000;
	u16 tx_data = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&on, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy tripod fail(%d)", ret);
		goto p_err;
	}

	tx_data = (on) ? (REG_DEAFULT_ON) : (OFF);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &rx_data);
	if (tx_data == rx_data) {
		LOG_OIS_INF("config already done(0x%04x)", rx_data);
		goto p_err;
	}

	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_TRIPODE_CTRL, tx_data);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &rx_data);

	LOG_OIS_INF("tripod set(%d 0x%04x) read(0x%04x)", on, tx_data, rx_data);

p_err:
	return ret;
}

static int dw9781c_set_sinewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_sinemode_parameter sine = { 0, };
	u16 mode = 0x0000;
	u16 amp = 0x0000;
	u16 frequency = 0x0000;
	u16 cmd_status = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&sine, user_buf, sizeof(struct ois_sinemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy sine params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("sine set(%d %d %d %d)", sine.axisEn, sine.frequency, sine.amplitudeX, sine.amplitudeY);

	if (1 == sine.axisEn)
		mode = 0x8005;
	else if (2 == sine.axisEn)
		mode = 0x8004;
	else if (3 == sine.axisEn)
		mode = 0x8006;
	else{
		LOG_OIS_INF("invalid axis(%d)", sine.axisEn);
		goto p_err;
	}

	if (sine.amplitudeX > -1 && sine.amplitudeX <= 100)
		amp = (u16)(0xFFFF * sine.amplitudeX / 200);
	else {
		LOG_OIS_INF("invalid amplitude(%d)", sine.amplitudeX);
		goto p_err;
	}

	frequency = (u16)sine.frequency;

	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_MODE, mode);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_AMP, amp);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_FREQ, frequency);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CMD_STATUS, REG_DEAFULT_ON);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CMD_STATUS, &cmd_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_AMP, &amp);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_FREQ, &frequency);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &mode);

	LOG_OIS_INF("sine read(0x%04x 0x%04x) mode(0x%04x) cmd(0x%04x)", frequency, amp, mode, cmd_status);

p_err:
	return ret;
}

static int dw9781c_set_target(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u16 servo = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_fixmode_parameter fixmode = { 0, };
	int min_code = 0, max_code = 0;
	s16 hallx = 0x0000, hally = 0x0000;
	s16 targetx = 0x0000, targety = 0x0000;
	u16 ampX = 0x0000, ampY = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &servo);
	if (SERVO_ON != servo) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CTRL, SERVO_ON);
		I2COP_CHECK(ret);
		LOG_OIS_INF("switch to servo on");
	}

	ret = copy_from_user(&fixmode, user_buf, sizeof(struct ois_fixmode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy target fail(%d)", ret);
		goto p_err;
	}

	switch (fixmode.target_vertex)
	{
		case LEFTTOP:{
			fixmode.targetX = 100;
			fixmode.targetY = -100;
			break;
		}
		case LEFTBOTTOM:{
			fixmode.targetX = -100;
			fixmode.targetY = -100;
			break;
		}
		case RIGHTTOP:{
			fixmode.targetX = 100;
			fixmode.targetY = 100;
			break;
		}
		case RIGHTBOTTOM:{
			fixmode.targetX = -100;
			fixmode.targetY = 100;
			break;
		}
		default:{
			LOG_OIS_INF("default target");
		}
	}

	LOG_OIS_INF("target set(%d %d)", fixmode.targetX, fixmode.targetY);

	ampX = ois->flash_info->hallInfo.totalLimitX;
	ampY = ois->flash_info->hallInfo.totalLimitY;
	targetx = (s16)(ampX * fixmode.targetX / 100);
	targety = (s16)(ampY * fixmode.targetY / 100);

	if (abs(targetx) <= ampX && abs(targety) <= ampY) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETX, targetx);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETY, targety);
		I2COP_CHECK(ret);
	} else {
		LOG_OIS_INF("invalid target(%d, %d)", targetx, targety);
		goto p_err;
	}

	max_code = abs(ois->flash_info->hallInfo.totalLimitX);
	min_code = 0 - max_code;

	mdelay(3);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETX, &targetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETY, &targety);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSX, &hallx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSY, &hally);

	LOG_OIS_INF("hall read target(0x%04x 0x%04x) hall(0x%04x 0x%04x) amp(0x%04x 0x%04x)",
		targetx, targety, hallx, hally, ampX, ampY);

p_err:
	return ret;
}

static int dw9781c_set_pantilt(struct ois *ois, __user void *user_buf)
{
	int ret             = 0;
	u16 pantilt_on      = 0x0000;
	u16 pantilt_degreeX = 0x0000;
	u16 pantilt_degreeY = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_pantilt_param pantilt = { 0, };

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&pantilt, user_buf, sizeof(struct ois_pantilt_param));
	if (ret) {
		LOG_OIS_ERR("copy pantilt fail(%d)", ret);
		goto p_err;
	}

	if (ON == pantilt.on) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_CTRL, PANTILT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_DERGEEX, pantilt.limitX);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_DERGEEY, pantilt.limitY);
		I2COP_CHECK(ret);
	} else if (OFF == pantilt.on) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_CTRL, PANTILT_OFF);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_CTRL, &pantilt_on);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &pantilt_degreeX);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &pantilt_degreeY);

	LOG_OIS_INF("pantilt info(%d %d %d) read(%d %d %d) ret(%d)",
		pantilt.on, pantilt.limitX, pantilt.limitY,
		pantilt_on, pantilt_degreeX, pantilt_degreeY, ret);
p_err:
	return ret;
}

/*write fw to flash*/
static int dw9781c_fw_download(struct ois *ois)
{
	int                   ret              = 0;
	struct i2c_client     *client          = ois->client;
	u16                   idx              = 0;
	u16                   update_flag      = 0;
	const struct firmware *ois_fw          = NULL;
	const u8              *fw_write_data   = NULL;
	u16                   fw_size          = 0;
	size_t                unit_size        = sizeof(u16)/sizeof(u8);
	u8                    *fw_read_data    = NULL;

	OIS_BUG(!client);
	OIS_BUG(!(ois->dev));

	//1.load fw to kernel space
	ret = request_firmware(&ois_fw, DW9781C_OIS_FW_NAME, ois->dev);
	if (ret) {
		LOG_OIS_ERR("load fw failed(%d)", ret);
		goto p_err;
	}
	fw_write_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_write_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	fw_size = ois_fw->size;
	memcpy((void *)fw_write_data, (void *)(ois_fw->data), (ois_fw->size));
	fw_read_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_read_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	release_firmware(ois_fw);
	LOG_OIS_INF("ois_fw buffer %p, size %d unit_size %d", fw_write_data, fw_size, unit_size);
	//3.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//4.i2c level adjust
	ret = ois_i2c_write(client, 0xd005, 0x0001);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, 0xdd03, 0x0002);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, 0xdd04, 0x0002);
	I2COP_CHECK(ret);

	//5.erase current exist data of flash(each unit is 4k bytes)
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_0);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_2);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_3);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_4);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	LOG_OIS_INF("flash erase success!");

	//6.write new fw to flash
	for (idx = 0; idx < fw_size; idx += DW9781C_BLOCK_SIZE) {
		ret = ois_i2c_write_block(client, (MTP_START_ADDR + idx / unit_size), &fw_write_data[idx], DW9781C_BLOCK_SIZE);
		mdelay(10);
		if (ret) {
			update_flag = 1;
			LOG_OIS_ERR("flash[%d] write: addr 0x%04x failed(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
			break;
		} else {
			LOG_OIS_INF("flash[%d] write: addr 0x%04x success(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
		}
	}
	if (update_flag)
		goto p_err;

	//7.verify new fw data in flash
	for (idx = 0; idx < fw_size; idx += DW9781C_BLOCK_SIZE) {
		ret = ois_i2c_read_block(client, (MTP_START_ADDR + idx / unit_size), &fw_read_data[idx], DW9781C_BLOCK_SIZE);
	}
	for (idx = 0; idx < fw_size; idx++) {
		if (fw_write_data[idx] != fw_read_data[idx]) {
			LOG_OIS_ERR("fw data mismatch: idx 0x%04x flash=0x%02x expect=0x%02x\n", idx, fw_read_data[idx], fw_write_data[idx]);
			ret = -1;
		}
	}
	/*
	for (idx = 0; idx < fw_size; idx += 8) {
		LOG_OIS_INF("flash read data[%lu]:0x%02x, 0x%02x, 0x%02x, 0x%02x,0x%02x, 0x%02x, 0x%02x, 0x%02x,\n",
			idx, fw_read_data[idx+0], fw_read_data[idx+1], fw_read_data[idx+2], fw_read_data[idx+3], fw_read_data[idx+4], fw_read_data[idx+5], fw_read_data[idx+6], fw_read_data[idx+7]);
	}
	*/
	if (!ret)
		LOG_OIS_INF("fw download success");
	else
		LOG_OIS_INF("fw download failed(%d)", ret);

p_err:
	if (fw_write_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_write_data);
		kfree(fw_write_data);
		fw_write_data = NULL;
	}
	if (fw_read_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_read_data);
		kfree(fw_read_data);
		fw_read_data = NULL;
	}
	return ret;
}

static int dw9781c_fw_update(struct ois *ois, void __user *user_buf)
{
	int                   ret              = 0;
	u16                   curr_fw_version  = 0x0000;
	u16                   exp_fw_version   = EXP_FW.version;
	u16                   fw_date          = 0x0000;
	u16                   fw_type          = 0x0000;
	u16                   first_chip_id    = 0x0000;
	u16                   second_chip_id   = 0x0000;
	u16                   curr_fw_checksum = 0x0000;
	u16                   exp_fw_checksum  = 0x0000;
	int                   froce_version    = 0;
	struct i2c_client     *client          = ois->client;

	OIS_BUG(!client);

	LOG_OIS_INF("E");

	if (NULL != user_buf) {
		ret = copy_from_user(&froce_version, user_buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy_from_user fail(%d)", ret);
			goto p_err;
		}
		exp_fw_version = froce_version;
		LOG_OIS_INF("force ois update(0x%04x)", froce_version);
	}
	//4.check chip id and download fw
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CHIP_ID, &first_chip_id);
	if (CHIP_ID != first_chip_id) {
		LOG_OIS_ERR("previous ois fw update failed(chip=0x%04x, expect=0x%04x)", first_chip_id, CHIP_ID);
		//ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
		//I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
		I2COP_CHECK(ret);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_SECOND_CHIP_ID, &second_chip_id);
		if (DW9781C_SENCOND_CHIP_ID == second_chip_id) {
			LOG_OIS_INF("try to udpate fw again");
			ret = dw9781c_fw_download(ois);
			if (ret) {//if failed, erase ldt flash and shutdown chip
				ret = dw9781c_fw_download(ois);
				if(ret)
				{
					ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_PAGE);
					I2COP_CHECK(ret);
					ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_PAGE);
					I2COP_CHECK(ret);
					ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, OFF);
					I2COP_CHECK(ret);
					mdelay(10);
					LOG_OIS_ERR("fw download failed");
					goto p_err;
				}
			}
		} else {
			LOG_OIS_ERR("chip id error(0x%04x)", second_chip_id);
			ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_PAGE);
			I2COP_CHECK(ret);
			ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_PAGE);
			I2COP_CHECK(ret);
			ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, OFF);
			I2COP_CHECK(ret);
			goto p_err;
		}
	} else {
		//fw checksum verify
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_EXP_CHECKSUM, &exp_fw_checksum);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
		mdelay(10);
		I2COP_CHECK(ret);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM, &curr_fw_checksum);
		//exp_fw_checksum = (fw_write_data[DW9781C_FW_CHECKSUM_OFFSET] << 8) | fw_write_data[DW9781C_FW_CHECKSUM_OFFSET + 1];
		LOG_OIS_INF("current fw checksum (0x%04x) expect fw checksum(0x%04x)", curr_fw_checksum, exp_fw_checksum);
		if(curr_fw_checksum == exp_fw_checksum)
		{
			// check fw version & type
			ret = dw9781c_reset(ois);
			I2COP_CHECK(ret);
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &curr_fw_version);
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &fw_date);
			if (exp_fw_version == curr_fw_version) {
				LOG_OIS_INF("same fw, skip update(0x%04x)", curr_fw_version);
				goto p_err;
			}
		}
		ret = dw9781c_fw_download(ois);
		if (ret) {//if failed, erase ldt flash and shutdown chip
			ret = dw9781c_fw_download(ois);
			if(ret)
			{
				ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_PAGE);
				I2COP_CHECK(ret);
				ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_PAGE);
				I2COP_CHECK(ret);
				ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, OFF);
				I2COP_CHECK(ret);
				mdelay(10);
				LOG_OIS_ERR("fw download failed");
				goto p_err;
			}
		}
	}
	LOG_OIS_INF("current fw(0x%04x) expect fw(0x%04x)", curr_fw_version, exp_fw_version);

	//5.ic reset & restart ois
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_EXP_CHECKSUM, &exp_fw_checksum);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	mdelay(10);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM, &curr_fw_checksum);
	//exp_fw_checksum = (fw_write_data[DW9781C_FW_CHECKSUM_OFFSET] << 8) | fw_write_data[DW9781C_FW_CHECKSUM_OFFSET + 1];
	LOG_OIS_INF("current fw checksum (0x%04x) expect fw checksum(0x%04x)", curr_fw_checksum, exp_fw_checksum);
	if(curr_fw_checksum != exp_fw_checksum)
	{
		LOG_OIS_ERR("fw download end checksum failed");
		goto p_err;
	}
	//6.check fw basic info
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &curr_fw_version);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &fw_date);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
	if (exp_fw_version != curr_fw_version) {
		LOG_OIS_ERR("after update: fw(ver=0x%04x date=0x%04x type=0x%04x) check failed", curr_fw_version, fw_date, fw_type);
		goto p_err;
	}
	LOG_OIS_INF("X: fw(ver=0x%04x date=0x%04x type=0x%04x) update success(%d)", curr_fw_version, fw_date, fw_type, ret);
p_err:
	return ret;
}

static int dw9781c_status_check(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	u16 u16_rxdata = 0x0000;
	struct i2c_client     *client      = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_RAWX, &s16_rxdata);
	flash_info->imuInfo.gyroRawX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_RAWY, &s16_rxdata);
	flash_info->imuInfo.gyroRawY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_RAWX, &s16_rxdata);
	flash_info->imuInfo.accRawX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_RAWY, &s16_rxdata);
	flash_info->imuInfo.accRawY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSX, &s16_rxdata);
	flash_info->hallInfo.lensPosX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSY, &s16_rxdata);
	flash_info->hallInfo.lensPosY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &u16_rxdata);
	flash_info->serveOn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_STATUS, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &u16_rxdata);
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAIN_DETECT, &u16_rxdata);
	flash_info->DetectLoopgainFlag = u16_rxdata;
	if (SERVO_ON == flash_info->serveOn) {
		ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	} else if (OIS_ON == flash_info->serveOn) {
		ret = ois_i2c_read(client, DW9781C_REG_OIS_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read(client, DW9781C_REG_OIS_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	}


	LOG_OIS_INF("gyro: %d %d acc: %d %d 0x%04x 0x%04x target: %d %d lensPos: %d %d",
		flash_info->imuInfo.gyroRawX, flash_info->imuInfo.gyroRawY, flash_info->imuInfo.accRawX, flash_info->imuInfo.accRawY,
		flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		flash_info->targetInfo.totalTargetX, flash_info->targetInfo.totalTargetY,
		flash_info->hallInfo.lensPosX, flash_info->hallInfo.lensPosY);
	LOG_OIS_INF("control chip:%d servo:%d dsp:%d reset:%d mode:%d tripod:%d spi mode:%d read:0x%04x limit:%d %d loopgain flag:0x%04x",
		flash_info->chipEn, flash_info->serveOn, flash_info->dspEn, flash_info->logicReset, flash_info->mode,
		flash_info->tripodFlag, flash_info->imuInfo.imuReadEn, flash_info->imuInfo.spiMode,
		flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY, flash_info->DetectLoopgainFlag);


	return ret;
}
/*static int dw9781c_EEPROM_ReadReg(u16 addr, u16 *data)
{
	u8 u8data[2];
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	g_pstAF_I2Cclient->addr = (EEPROM_I2C_SLAVE_ADDR) >> 1;
	if (i2c_master_send(g_pstAF_I2Cclient, pu_send_cmd, 2) < 0) {
		LOG_INF("read I2C send failed!!\n");
		return -1;
	}
	if (i2c_master_recv(g_pstAF_I2Cclient, u8data, 2) < 0) {
		LOG_INF("EEPROM_ReadReg failed!!\n");
		return -1;
	}
	LOG_INF("u8data[0] = 0x%x\n", u8data[0]);
	LOG_INF("u8data[1] = 0x%x\n", u8data[1]);

	*data = u8data[1] << 8 | u8data[0];
	LOG_INF("s4EEPROM_ReadReg2 0x%x, 0x%x\n", addr, *data);

	return 0;
}*/
static int dw9781c_format_otp_data(struct ois *ois, void __user *user_buf)
{
	int                 ret       = 0;
	u8                  *otp_buf  = NULL;
	u8                  *sn_data  = NULL;
	u8                  *fuse_id  = NULL;
	u8                  data_size = 0;
	u8                  idx       = 0;
	struct ois_otp_info *ois_otp  = NULL;
	s16                 s16_data  = 0x0000;

	OIS_BUG(!user_buf);
	OIS_BUG(!(ois->ois_otp))

	ois_otp  = ois->ois_otp;

	ois_otp->inited = 0x00;

	otp_buf = vivo_otp_data_s5kgw3sp13pd2083;

	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x2E4E);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E78);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E7A);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E7C);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E7E);
	ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E80);
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E82);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E88);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E8A);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E84);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E86);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E94);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E96);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9C);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9E);

	data_size = 0x0026 - 0x0021 + 1;
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

	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(%d,%d,%d,%d,%d,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX, ois_otp->gyroGainY,
		ois_otp->gyroOffsetX, ois_otp->gyroOffsetY, ois_otp->hallXMin, ois_otp->hallXMax, ois_otp->hallMechCenterX,
		ois_otp->hallYMin, ois_otp->hallYMax, ois_otp->hallMechCenterY,
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);
p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
	return ret;
}

static int dw9781c_ready_check(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;
	u16                   fw_checksum_flag = 0x0000;
	ois->client->addr = DW7981C_SLAVE_ADDR >> 1;

	OIS_BUG(!client);

	/*1.disable dsp and release all protection*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	LOG_OIS_INF("ois reset success(%d)", ret);
	//2.check checksum flag
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_FLAG, &fw_checksum_flag);
	if (FW_CHECKSUM_FLAG == fw_checksum_flag) {
		LOG_OIS_INF("ois checksum flag ok(0x%04x)", fw_checksum_flag);
		ret = dw9781c_reset(ois);
	} else {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		mdelay(4);
		I2COP_CHECK(ret);
		LOG_OIS_INF("ois checksum flag ng(0x%04x)", fw_checksum_flag);
	}
p_err:
	return ret;
}

static int dw9781c_get_loopgain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int loopgain[2] = {0,};
	u16 t_loopgain[2] = {0,};
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	OIS_BUG(!user_buf);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINX, &(t_loopgain[0]));
	loopgain[0] = t_loopgain[0];
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINY, &(t_loopgain[1]));
	loopgain[1] = t_loopgain[1];

	ret = copy_to_user(user_buf, loopgain, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("loop gain(%d, %d)", loopgain[0], loopgain[1]);
	return ret;
}


static struct ois_core_ops dw9781c_ois_ops = {
	.ois_init = dw9781c_init,
	.ois_init_slave = dw9781c_init_slave,
	.ois_deinit = dw9781c_deinit,
	.ois_stream_on = dw9781c_stream_on,
	.ois_stream_off = dw9781c_stream_off,
	.ois_get_mode = dw9781c_get_mode,
	.ois_set_mode = dw9781c_set_mode,
	.ois_fw_update = dw9781c_fw_update,
	.ois_get_fw_version = dw9781c_get_fw_version,
	.ois_get_gyro_offset = dw9781c_get_gyro_offset,
	.ois_set_offset_calibration = dw9781c_set_offset_calibration,
	.ois_get_gyro_gain = dw9781c_get_gyro_gain,
	.ois_set_gyro_gain = dw9781c_set_gyro_gain,
	.ois_flash_save = dw9781c_flash_save,
	.ois_set_acc = dw9781c_set_acc,
	.ois_set_target = dw9781c_set_target,
	.ois_get_init_info = dw9781c_get_init_info,
	.ois_status_check = dw9781c_status_check,
	.ois_init_vsync_thread = dw9781c_init_vsync_thread,
	.ois_deinit_vsync_thread = dw9781c_deinit_vsync_thread,
	.ois_vsync_signal = dw9781c_vsync_signal,
	.ois_get_lens_info = dw9781c_get_lens_info,
	.ois_format_otp_data = dw9781c_format_otp_data,
	.ois_set_sinewave = dw9781c_set_sinewave,
	.ois_set_stroke_limit = dw9781c_set_stroke_limit,
	.ois_set_pantilt = dw9781c_set_pantilt,
	.ois_reset = dw9781c_reset,
	.ois_ready_check = dw9781c_ready_check,
	.ois_set_smooth = dw9781c_set_smooth,
	.ois_set_tripod = dw9781c_set_tripod,
	.ois_log_control = dw9781c_log_control,
	.ois_get_loopgain = dw9781c_get_loopgain,
};

/*ic entry expose to ois_core*/
void dw9781c_get_ops(struct ois *ois)
{
	ois->ops = &dw9781c_ois_ops;
}
