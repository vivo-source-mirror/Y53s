/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010 -2016, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
 *
 * File Name: focaltech_upgrade_ft8006.c
 *
 * Author:    fupeipei
 *
 * Created:    2016 -08 -15
 *
 * Abstract:
 *
 * Reference:
 *
 *****************************************************************************/

/*****************************************************************************
 * 1.Included header files
 *****************************************************************************/
#include "../focaltech_core.h"

#include "../focaltech_flash.h"
#include "focaltech_upgrade_common.h"

/*****************************************************************************
 * Static variables
 *****************************************************************************/
#if UPGRADE_ALL
#define APP_OFFSET                  0x5000
#else
#define APP_OFFSET                  0x0
#endif
#define APP_FILE_MAX_SIZE           (116 * 1024)
#define APP_FILE_MIN_SIZE           (8)
#define APP_FILE_VER_MAPPING        (0x10E + APP_OFFSET)
#define APP_FILE_VENDORID_MAPPING   (0x10C + APP_OFFSET)
#define APP_FILE_CHIPID_MAPPING     (0x11E + APP_OFFSET)
#define CONFIG_START_ADDR           (0xF80)
#define CONFIG_START_ADDR_LEN       (0x80)
#define CONFIG_VENDOR_ID_OFFSET     (0x04)
#define CONFIG_PROJECT_ID_OFFSET    (0x20)
#define CONFIG_VENDOR_ID_ADDR       (CONFIG_START_ADDR + CONFIG_VENDOR_ID_OFFSET)
#define CONFIG_PROJECT_ID_ADDR      (CONFIG_START_ADDR + CONFIG_PROJECT_ID_OFFSET)
#define LCD_CFG_MAX_SIZE            (4 * 1024)
#define LCD_CFG_MIN_SIZE            (8)


#define IMEI_SERCOTR    30

/*****************************************************************************
 * Global variable or extern global variabls / functions
 *****************************************************************************/
static int fts_ctpm_get_app_i_file_ver(void);
static int fts_ctpm_get_app_bin_file_ver(char *firmware_name);
static int fts_ctpm_fw_upgrade_with_app_i_file(struct i2c_client *client);
static int fts_ctpm_fw_upgrade_with_app_bin_file(struct i2c_client *client, char *firmware_name);
static int fts_ctpm_fw_upgrade_with_lcd_cfg_i_file(struct i2c_client *client);
static int fts_ctpm_fw_upgrade_with_lcd_cfg_bin_file(struct i2c_client *client, char *firmware_name);

struct fts_upgrade_fun fts_updatefun_ft8006 = {
	.get_app_bin_file_ver = fts_ctpm_get_app_bin_file_ver,
	.get_app_i_file_ver = fts_ctpm_get_app_i_file_ver,
	.upgrade_with_app_i_file = fts_ctpm_fw_upgrade_with_app_i_file,
	.upgrade_with_app_bin_file = fts_ctpm_fw_upgrade_with_app_bin_file,
	.upgrade_with_lcd_cfg_i_file = fts_ctpm_fw_upgrade_with_lcd_cfg_i_file,
	.upgrade_with_lcd_cfg_bin_file = fts_ctpm_fw_upgrade_with_lcd_cfg_bin_file,
};

/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/

/************************************************************************
 * Name: fts_ctpm_get_app_bin_file_ver
 * Brief:  get .i file version
 * Input: no
 * Output: no
 * Return: fw version
 ***********************************************************************/
static int fts_ctpm_get_app_bin_file_ver(char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int fwsize = fts_GetFirmwareSize(firmware_name);
	int fw_ver = 0;

	FTS_FUNC_ENTER();

	if (fwsize < APP_FILE_MIN_SIZE || fwsize > APP_FILE_MAX_SIZE) {
		FTS_ERROR("[UPGRADE]: FW length(%x) error", fwsize);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		FTS_ERROR("[UPGRADE]: request_firmware failed!!");
		kfree(pbt_buf);
		return -EIO;
	}

	fw_ver = pbt_buf[APP_FILE_VER_MAPPING];

	kfree(pbt_buf);
	FTS_FUNC_EXIT();

	return fw_ver;
}

/************************************************************************
 * Name: fts_ctpm_get_app_i_file_ver
 * Brief:  get .i file version
 * Input: no
 * Output: no
 * Return: fw version
 ***********************************************************************/
static int fts_ctpm_get_app_i_file_ver(void)
{
	int fwsize = fts_getsize(FW_SIZE);

	if (fwsize < APP_FILE_MIN_SIZE || fwsize > APP_FILE_MAX_SIZE) {
		FTS_ERROR("[UPGRADE]: FW length(%x) error", fwsize);
		return -EIO;
	}

	fwsize = comp_upgrade_vars[g_global.index].upgrade_fw[APP_FILE_VER_MAPPING];
	FTS_DEBUG("8006 fw version: %02X", fwsize);
	return fwsize;
}

/************************************************************************
 * Name: fts_ctpm_get_vendor_id_flash
 * Brief:
 * Input:
 * Output:
 * Return:
 ***********************************************************************/
static int fts_ctpm_get_vendor_id_flash(struct i2c_client *client)
{
#if FTS_GET_VENDOR_ID
	int i_ret = 0;
	u8 vendorid[4] = {0};
	u8 auc_i2c_write_buf[10];
	u8 i = 0;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*read vendor id */
		auc_i2c_write_buf[0] = 0x03;
		auc_i2c_write_buf[1] = 0x00;

		auc_i2c_write_buf[2] = (u8)((CONFIG_VENDOR_ID_ADDR - 1) >> 8);
		auc_i2c_write_buf[3] = (u8)(CONFIG_VENDOR_ID_ADDR - 1);
		i_ret = fts_i2c_read(client, auc_i2c_write_buf, 4, vendorid, 2);
		if (i_ret < 0) {
			FTS_DEBUG("[UPGRADE]: read flash : i_ret = %d!!", i_ret);
			continue;
		} else if ((vendorid[1] == FTS_VENDOR_1_ID) || (vendorid[1] == FTS_VENDOR_2_ID))
			break;
	}

	FTS_DEBUG("[UPGRADE]: vendor id from flash rom: 0x%x!!", vendorid[1]);
	if (i >= FTS_UPGRADE_LOOP) {
		FTS_ERROR("[UPGRADE]: read vendor id from flash more than 30 times!!");
		return -EIO;
	}

	return 0;
#else
	return 0;
#endif
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_use_buf
 * Brief: fw upgrade
 * Input: i2c info, file buf, file len
 * Output: no
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_use_buf(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;
	int i_ret = 0;
	bool inbootloader = false;

	FTS_INFO("upgrade fw");
	fts_ctpm_i2c_hid2std(client);

	i_ret = fts_ctpm_start_fw_upgrade(client);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE]: send upgrade cmd to FW error!!");
		goto reset_fw;
	}

	/*Enter upgrade mode */
	fts_ctpm_i2c_hid2std(client);
	msleep(10);

	inbootloader = fts_ctpm_check_run_state(client, FTS_RUN_IN_BOOTLOADER);
	if (!inbootloader) {
		FTS_ERROR("[UPGRADE]: not run in bootloader, upgrade fail!!");
		i_ret = -EIO;
		goto reset_fw;
	}

	/*read vendor id from flash, if vendor id error, can not upgrade */
	i_ret = fts_ctpm_get_vendor_id_flash(client);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE]: read vendor id in flash fail!!");
		goto reset_fw;
	}

	auc_i2c_write_buf[0] = 0x05;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 3);
	FTS_INFO("flash manufactory id:%02x device id:%02x%02x", reg_val[2], reg_val[1], reg_val[0]);
	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download */
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0B;
	fts_i2c_write(client, auc_i2c_write_buf, 2);

	/*
	 * All.bin <= 128K
	 * APP.bin <= 94K
	 * LCD_CFG <= 4K
	 */
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);


	/*erase the app erea in flash */
	i_ret = fts_ctpm_erase_flash(client);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE]: erase flash error!!");
		goto reset_fw;
	}

	/*write FW to ctpm flash */
	upgrade_ecc = 0;
	FTS_DEBUG("[UPGRADE]: write FW to ctpm flash!!");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;

	for (j = 0; j < packet_number; j++) {
		temp = 0x5000 + j * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = 0x5000 + packet_number * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((0x1000 + ((0x5000 + packet_number * FTS_PACKET_LENGTH) / ((dw_lenth) % FTS_PACKET_LENGTH))) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}

	msleep(50);

	/*********Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_DEBUG("[UPGRADE]: read out checksum!!");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0x5000;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (64 * 1024 - 1);
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	temp = (0x5000 + (64 * 1024 - 1));
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (dw_lenth - (64 * 1024 - 1));
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) {
			FTS_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);

	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != upgrade_ecc) {
		FTS_ERROR("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!", reg_val[0], upgrade_ecc);
		i_ret = -EIO;
		goto reset_fw;
	}
	FTS_DEBUG("[UPGRADE]: checksum %x %x!!", reg_val[0], upgrade_ecc);

reset_fw:
	FTS_DEBUG("[UPGRADE]: reset the new FW!!");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1000);

	fts_ctpm_i2c_hid2std(client);

	FTS_INFO("reset fw done");
	return i_ret;
}
#if UPGRADE_ALL
/*3 - gamma begin */
#define MAX_BANK_DATA       0x80
#define MAX_GAMMA_LEN       0x180
u8 gamma_analog[] = { 0x85, 0x00, 0x00, 0x2C, 0x2B };
u8 gamma_digital1[] = { 0x8D, 0x00, 0x00, 0x80, 0x80 };
u8 gamma_digital2[] = { 0x8D, 0x80, 0x00, 0x14, 0x13 };
u8 gamma_enable[] = { 0x91, 0x80, 0x00, 0x19, 0x01 };
union short_bits{
	u16 dshort;
	struct bits{
		u16 bit0:1;
		u16 bit1:1;
		u16 bit2:1;
		u16 bit3:1;
		u16 bit4:1;
		u16 bit5:1;
		u16 bit6:1;
		u16 bit7:1;
		u16 bit8:1;
		u16 bit9:1;
		u16 bit10:1;
		u16 bit11:1;
		u16 bit12:1;
		u16 bit13:1;
		u16 bit14:1;
		u16 bit15:1;
	} bits;
};
static int print_data(u8 *buf, u32 len)
{
	int i = 0;
	int n = 0;
	u8 *p = NULL;

	p = kmalloc(len * 4, GFP_KERNEL);
	memset(p, 0, len * 4);

	for (i = 0; i < len; i++) {
		n += snprintf(p + n, 1023, "%02x ", buf[i]);
	}

	FTS_DEBUG("%s", p);

	kfree(p);
	return 0;
}
static int read_3gamma(struct i2c_client *client, u8 **gamma, u16 *len)
{
	int ret = 0;
	int i = 0;
	int packet_num = 0;
	int packet_len = 0;
	int remainder = 0;
	u8 cmd[4] = { 0 };
	u32 addr = 0x01D000;
	u8 gamma_header[0x20] = { 0 };
	u16 gamma_len = 0;
	u16 gamma_len_n = 0;
	u16 pos = 0;
	bool gamma_has_enable = false;
	u8 *pgamma = NULL;
	int j = 0;
	int back_3gamma_ecc = 0;

	cmd[0] = 0x03;
	cmd[1] = (u8)(addr >> 16);
	cmd[2] = (u8)(addr >> 8);
	cmd[3] = (u8)addr;
	ret = fts_i2c_write(client, cmd, 4);
	msleep(10);
	ret = fts_i2c_read(client, NULL, 0, gamma_header, 0x20);
	if (ret < 0) {
		FTS_ERROR("read 3-gamma header fail");
		return ret;
	}

	gamma_len = (u16)((u16)gamma_header[0] << 8) + gamma_header[1];
	gamma_len_n = (u16)((u16)gamma_header[2] << 8) + gamma_header[3];

	if ((gamma_len + gamma_len_n) != 0xFFFF) {
		FTS_INFO("gamma length check fail:%x %x", gamma_len, gamma_len);
		return -EIO;
	}

	if ((gamma_header[4] + gamma_header[5]) != 0xFF) {
		FTS_INFO("gamma ecc check fail:%x %x", gamma_header[4], gamma_header[5]);
		return -EIO;
	}

	if (gamma_len > MAX_GAMMA_LEN) {
		FTS_ERROR("gamma data len(%d) is too long", gamma_len);
		return -EINVAL;
	}

	*gamma = kmalloc(MAX_GAMMA_LEN, GFP_KERNEL);
	if (NULL == *gamma) {
		FTS_ERROR("malloc gamma memory fail");
		return -ENOMEM;
	}
	pgamma = *gamma;

	packet_num = gamma_len / 256;
	packet_len = 256;
	remainder = gamma_len%256;
	if (remainder)
		packet_num++;
	FTS_INFO("3-gamma len:%d", gamma_len);
	cmd[0] = 0x03;
	addr += 0x20;
	for (i = 0; i < packet_num; i++) {
		addr += i * 256;
		cmd[1] = (u8)(addr >> 16);
		cmd[2] = (u8)(addr >> 8);
		cmd[3] = (u8)addr;
		if ((i == packet_num - 1) && remainder)
			packet_len = remainder;
		ret = fts_i2c_write(client, cmd, 4);
		msleep(10);
		ret = fts_i2c_read(client, NULL, 0, pgamma + i * 256, packet_len);
		if (ret < 0) {
			FTS_ERROR("read 3-gamma data fail");
			return ret;
		}
	}


	/* ecc */
	for (j = 0; j < gamma_len; j++) {
		  back_3gamma_ecc ^= pgamma[j];
	}
	FTS_INFO("back_3gamma_ecc: 0x%x, 0x%x", back_3gamma_ecc, gamma_header[0x04]);
	if (back_3gamma_ecc != gamma_header[0x04]) {
		  FTS_ERROR("back gamma ecc check fail:%x %x", back_3gamma_ecc, gamma_header[0x04]);
		return -EIO;
	}


	/* check last byte is 91 80 00 19 01 */
	pos = gamma_len - 5;
	if ((gamma_enable[0] == pgamma[pos]) && (gamma_enable[1] == pgamma[pos + 1])
		&& (gamma_enable[2] == pgamma[pos + 2]) && (gamma_enable[3] == pgamma[pos + 3])) {
		gamma_has_enable = true;
	}

	if (false == gamma_has_enable) {
		FTS_INFO("3-gamma has no gamma enable info");
		pgamma[gamma_len++] = gamma_enable[0];
		pgamma[gamma_len++] = gamma_enable[1];
		pgamma[gamma_len++] = gamma_enable[2];
		pgamma[gamma_len++] = gamma_enable[3];
		pgamma[gamma_len++] = gamma_enable[4];
	}

	*len = gamma_len;

	FTS_DEBUG("read 3-gamma data:");
	print_data(*gamma, gamma_len);

	return 0;
}
static int replace_3gamma(u8 *initcode, u8 *gamma, u16 gamma_len)
{
	u16 gamma_pos = 0;
/*    u16 gamma_bank_len = 0; */
	u16 initcode_pos = 0x0A;
	u16 initcode_len = 0;
	u16 bank_len = 0;

	initcode_len = (u16)(((u16)initcode[2] << 8) + initcode[3]);

	while (gamma_pos < gamma_len) {
		while (initcode_pos < initcode_len) {
			bank_len = (u16)(((u16)initcode[initcode_pos + 2] << 8) + initcode[initcode_pos + 3]);
			if ((initcode[initcode_pos] == gamma[gamma_pos])
				&& (initcode[initcode_pos + 1] == gamma[gamma_pos + 1])) {
				FTS_INFO("3-gamma bank(%02x %02x) find gamma pos=%x, init pos=%x",
						gamma[gamma_pos], gamma[gamma_pos + 1], gamma_pos, initcode_pos);
				if ((gamma_enable[0] == gamma[gamma_pos]) && (gamma_enable[1] == gamma[gamma_pos + 1])) {
					if (gamma[gamma_pos + 4])
						initcode[initcode_pos + 4 + 15] |= 0x01;
					else
						initcode[initcode_pos + 4 + 15] &= 0xFE;
					gamma_pos += 1 + 4;
				} else if ((gamma_analog[0] == gamma[gamma_pos]) && (gamma_analog[1] == gamma[gamma_pos + 1])) {
					memcpy(initcode + initcode_pos + 4, gamma + gamma_pos + 4, gamma_analog[4]);
					gamma_pos += gamma_analog[4] + 4;
				} else if ((gamma_digital1[0] == gamma[gamma_pos]) && (gamma_digital1[1] == gamma[gamma_pos + 1])) {
					memcpy(initcode + initcode_pos + 4, gamma + gamma_pos + 4, gamma_digital1[4]);
					gamma_pos += gamma_digital1[4] + 4;
				} else if ((gamma_digital2[0] == gamma[gamma_pos]) && (gamma_digital2[1] == gamma[gamma_pos + 1])) {
					memcpy(initcode + initcode_pos + 4, gamma + gamma_pos + 4, gamma_digital2[4]);
					gamma_pos += gamma_digital2[4] + 4;
				}
				break;
			}
			initcode_pos += bank_len + 4;
		}
		if (initcode_pos >= initcode_len) {
			FTS_INFO("3-gamma bank(%02x %02x) not find",
					gamma[gamma_pos], gamma[gamma_pos + 1]);
			return -ENODATA;
		}
/*        gamma_bank_len = (u16)(((u16)gamma[gamma_pos + 2] << 8) + gamma[gamma_pos + 3]); */
/*        gamma_pos += gamma_bank_len + 4; */
	}

	FTS_DEBUG("replace 3-gamma data:");
	print_data(initcode, 1100);

	return 0;
}
/* calculate lcd init code ecc */
static int cal_lcdinitcode_ecc(u8 *buf, u16 *ecc_val)
{
	u32 bank_crc_en = 0;
	u8 bank_data[MAX_BANK_DATA] = { 0 };
	u16 bank_len = 0;
	u16 bank_addr = 0;
	u32 bank_num = 0;
	u16 file_len = 0;
	u16 pos = 0;
	int i = 0;
	union short_bits ecc;
	union short_bits ecc_last;
	union short_bits temp_byte;
	u8 bank_mapping[] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9,
		0xA, 0xB, 0xC, 0xD, 0xE, 0xF, 0x10, 0x11, 0x12, 0x13, 0x14, 0x18,
		0x19, 0x1A, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x22, 0x23, 0x24}; /*Actaul mipi bank */
	u8 banknum_8006 = 0;

	ecc.dshort = 0;
	ecc_last.dshort = 0;
	temp_byte.dshort = 0;

	file_len = (u16)(((u16)buf[2] << 8) + buf[3]);
	bank_crc_en = (u32)(((u32)buf[9] << 24) + ((u32)buf[8] << 16) + \
		((u32)buf[7] << 8) + (u32)buf[6]);
	FTS_INFO("lcd init code len=%x bank en=%x", file_len, bank_crc_en);

	pos = 0x0A; /* addr of first bank */
	while (pos < file_len) {
		bank_addr = (u16)(((u16)buf[pos + 0] << 8) + buf[pos + 1]);
		bank_len = (u16)(((u16)buf[pos + 2] << 8) + buf[pos + 3]);
		FTS_INFO("bank pos=%x bank_addr=%x bank_len=%x", pos, bank_addr, bank_len);
		if (bank_len > MAX_BANK_DATA)
			return -EINVAL;
		memset(bank_data, 0, MAX_BANK_DATA);
		memcpy(bank_data, buf + pos + 4, bank_len);

		bank_num = (bank_addr - 0x8000) / MAX_BANK_DATA;
		FTS_INFO("actual mipi bank number = %x", bank_num);
		for (i = 0; i < sizeof(bank_mapping) / sizeof(u8); i++) {
			if (bank_num == bank_mapping[i]) {
				banknum_8006 = i;
				break;
			}
		}
		if (i >= sizeof(bank_mapping) / sizeof(u8)) {
			FTS_INFO("actual mipi bank(%d) not find in bank mapping, need jump", bank_num);
		} else {
			FTS_INFO("8006 bank number = %d", banknum_8006);
			if ((bank_crc_en >> banknum_8006) & 0x01) {
				for (i = 0; i < MAX_BANK_DATA; i++) {
					temp_byte.dshort = (u16)bank_data[i];
					if (i == 0)
						FTS_INFO("data0=%x, %d %d %d %d %d %d %d %d", temp_byte.dshort, temp_byte.bits.bit0,
							temp_byte.bits.bit1, temp_byte.bits.bit2, temp_byte.bits.bit3, temp_byte.bits.bit4,
							temp_byte.bits.bit5, temp_byte.bits.bit6, temp_byte.bits.bit7);

					ecc.bits.bit0 = ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ ecc_last.bits.bit11
						 ^ ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
						 ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3
						 ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;

					ecc.bits.bit1 = ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ ecc_last.bits.bit11 ^ ecc_last.bits.bit12
						 ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
						 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3 ^ temp_byte.bits.bit4
						 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;

					ecc.bits.bit2 = ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1;

					ecc.bits.bit3 = ecc_last.bits.bit9 ^ ecc_last.bits.bit10 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2;

					ecc.bits.bit4 = ecc_last.bits.bit10 ^ ecc_last.bits.bit11 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3;

					ecc.bits.bit5 = ecc_last.bits.bit11 ^ ecc_last.bits.bit12 ^ temp_byte.bits.bit3 ^ temp_byte.bits.bit4;

					ecc.bits.bit6 = ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5;

					ecc.bits.bit7 = ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6;

					ecc.bits.bit8 = ecc_last.bits.bit0 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;

					ecc.bits.bit9 = ecc_last.bits.bit1 ^ ecc_last.bits.bit15 ^ temp_byte.bits.bit7;

					ecc.bits.bit10 = ecc_last.bits.bit2;

					ecc.bits.bit11 = ecc_last.bits.bit3;

					ecc.bits.bit12 = ecc_last.bits.bit4;

					ecc.bits.bit13 = ecc_last.bits.bit5;

					ecc.bits.bit14 = ecc_last.bits.bit6;

					ecc.bits.bit15 = ecc_last.bits.bit7 ^ ecc_last.bits.bit8 ^ ecc_last.bits.bit9 ^ ecc_last.bits.bit10
						 ^ ecc_last.bits.bit11 ^ ecc_last.bits.bit12 ^ ecc_last.bits.bit13 ^ ecc_last.bits.bit14 ^ ecc_last.bits.bit15
						 ^ temp_byte.bits.bit0 ^ temp_byte.bits.bit1 ^ temp_byte.bits.bit2 ^ temp_byte.bits.bit3
						 ^ temp_byte.bits.bit4 ^ temp_byte.bits.bit5 ^ temp_byte.bits.bit6 ^ temp_byte.bits.bit7;

					ecc_last.dshort = ecc.dshort;

				}
			}
		}
		pos += bank_len + 4;
	}

	*ecc_val = ecc.dshort;
	FTS_INFO("");
	return 0;
}

/* calculate lcd init code checksum */
static unsigned short cal_lcdinitcode_checksum(u8 *ptr, int length)
{
	/*CRC16 */
	u16 cFcs = 0;
	int i, j;

	if (length%2) {
		return 0xFFFF;
	}

	for (i = 0; i < length; i += 2) {
		cFcs ^= ((ptr[i] << 8) + ptr[i + 1]);
		for (j = 0; j < 16; j++) {
			if (cFcs & 1) {
				cFcs = (unsigned short)((cFcs >> 1) ^ ((1 << 15) + (1 << 10) + (1 << 3)));
			} else {
				cFcs >>= 1;
			}
		}
	}
	return cFcs;
}
static int read_replace_3gamma(struct i2c_client *client, u8 *buf)
{
	int ret = 0;
	u16 initcode_ecc = 0;
	u16 initcode_checksum = 0;
	u8 *gamma = NULL;
	u16 gamma_len = 0;

	FTS_FUNC_ENTER();

	ret = read_3gamma(client, &gamma, &gamma_len);
	if (ret < 0) {
		FTS_INFO("no vaid 3-gamma data, not replace");
		return 0;
	}

	ret = replace_3gamma(buf, gamma, gamma_len);
	if (ret < 0) {
		FTS_ERROR("replace 3-gamma fail");
		kfree(gamma);
		return ret;
	}

	ret = cal_lcdinitcode_ecc(buf, &initcode_ecc);
	if (ret < 0) {
		FTS_ERROR("lcd init code ecc calculate fail");
		kfree(gamma);
		return ret;
	}
	FTS_INFO("lcd init code cal ecc:%04x", initcode_ecc);
	buf[4] = (u8)(initcode_ecc >> 8);
	buf[5] = (u8)(initcode_ecc);
	buf[0x43d] = (u8)(initcode_ecc >> 8);
	buf[0x43c] = (u8)(initcode_ecc);

	initcode_checksum = cal_lcdinitcode_checksum(buf + 2, 0x43e - 2);
	FTS_INFO("lcd init code calc checksum:%04x", initcode_checksum);
	buf[0] = (u8)(initcode_checksum >> 8);
	buf[1] = (u8)(initcode_checksum);

	FTS_FUNC_EXIT();

	kfree(gamma);
	return 0;
}
/*3 - gamma end */
#endif
/************************************************************************
 * Name: fts_ctpm_fw_upgrade_use_buf
 * Brief: fw upgrade
 * Input: i2c info, file buf, file len
 * Output: no
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_lcd_cfg_upgrade_use_buf(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
#if !UPGRADE_ALL
	u8 cfg_backup[CONFIG_START_ADDR_LEN + 1] = { 0 };
#endif
	u32 i = 0;
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;
	int i_ret = 0;

	FTS_INFO("upgrade lcd init code");
	fts_ctpm_i2c_hid2std(client);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*write 0xaa to register FTS_RST_CMD_REG1 */
		fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(10);

		/*write 0x55 to register FTS_RST_CMD_REG1 */
		fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
		msleep(200);

		/*Enter upgrade mode */
		fts_ctpm_i2c_hid2std(client);

		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_ERROR("[UPGRADE]: failed writing  0x55 and 0xaa!!");
			continue;
		}

		/*check run in bootloader or not */
		msleep(1);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == chip_types.bootloader_idh
			&& reg_val[1] == chip_types.bootloader_idl) {
			FTS_DEBUG("[UPGRADE]: read bootload id ok!! ID1 = 0x%x,ID2 = 0x%x!!", reg_val[0], reg_val[1]);
			break;
		} else {
			FTS_ERROR("[UPGRADE]: read bootload id fail!! ID1 = 0x%x,ID2 = 0x%x!!", reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
#if !UPGRADE_ALL
	/* Backup FW configuratin area */
	reg_val[0] = 0x03;
	reg_val[1] = (u8)((CONFIG_START_ADDR - 1) >> 16);
	reg_val[2] = (u8)((CONFIG_START_ADDR - 1) >> 8);
	reg_val[3] = (u8)((CONFIG_START_ADDR - 1));
	i_ret = fts_i2c_read(client, reg_val, 4, cfg_backup, CONFIG_START_ADDR_LEN + 1);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE] Read Configuration area error, don't upgrade LCD Code");
		goto reset_fw;
	}
#endif

	i_ret = read_replace_3gamma(client, pbt_buf);
	if (i_ret < 0) {
		FTS_ERROR("replace 3-gamma fail, not upgrade lcd init code");
		FTS_DEBUG("[UPGRADE]: reset the new FW!!");
		auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(1000);

		fts_ctpm_i2c_hid2std(client);

		return i_ret;
	}

	auc_i2c_write_buf[0] = 0x05;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 3);
	FTS_INFO("flash manufactory id:%02x device id:%02x%02x", reg_val[2], reg_val[1], reg_val[0]);
	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download */
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0C;
	fts_i2c_write(client, auc_i2c_write_buf, 2);

	/*Step 4:erase app and panel paramenter area */
	FTS_DEBUG("[UPGRADE]: erase app and panel paramenter area!!");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1000);

	for (i = 0; i < 15; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1]) {
			break;
		}
		msleep(50);
	}
	FTS_DEBUG("[UPGRADE]: erase app area reg_val[0] = %x reg_val[1] = %x!!", reg_val[0], reg_val[1]);

	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = 0;
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);

	/*write FW to ctpm flash */
	upgrade_ecc = 0;
	FTS_DEBUG("[UPGRADE]: write FW to ctpm flash!!");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0;
	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((0x1000 + ((packet_number * FTS_PACKET_LENGTH) / ((dw_lenth) % FTS_PACKET_LENGTH))) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}
#if !UPGRADE_ALL
	/* Write Back FW configuratin area */
#if 0
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = (u8)(CONFIG_START_ADDR >> 16);
	packet_buf[2] = (u8)(CONFIG_START_ADDR >> 8);
	packet_buf[3] = (u8)(CONFIG_START_ADDR);
	packet_buf[4] = (u8)(CONFIG_START_ADDR_LEN >> 8);
	packet_buf[5] = (u8)(CONFIG_START_ADDR_LEN);
	memcpy(&packet_buf[6], &cfg_backup[1], CONFIG_START_ADDR_LEN);
	i_ret = fts_i2c_write(client, packet_buf, CONFIG_START_ADDR_LEN + 6);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE] Write Configuration area error");
	}
#endif
	temp = 0;
	packet_number = (CONFIG_START_ADDR_LEN) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = (u8)(CONFIG_START_ADDR >> 16);
	for (j = 0; j < packet_number; j++) {
		temp = CONFIG_START_ADDR + j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
	}

	if ((CONFIG_START_ADDR_LEN) % FTS_PACKET_LENGTH > 0) {
		temp = CONFIG_START_ADDR + packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
	}
#endif
	msleep(50);

	/*********Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_DEBUG("[UPGRADE]: read out checksum!!");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0x00;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = 0;
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) {
			FTS_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);

	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != upgrade_ecc) {
		FTS_ERROR("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!", reg_val[0], upgrade_ecc);
		i_ret = -EIO;
		goto reset_fw;
	}
	FTS_DEBUG("[UPGRADE]: checksum %x %x!!", reg_val[0], upgrade_ecc);


reset_fw:
	FTS_DEBUG("[UPGRADE]: reset lcd init code!!");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1000);

	fts_ctpm_i2c_hid2std(client);

	FTS_INFO("upgrade lcd init code succuss");
	return i_ret;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_with_app_i_file
 * Brief:  upgrade with *.i file
 * Input: i2c info
 * Output:
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_with_app_i_file(struct i2c_client *client)
{
	int i_ret = 0;
	u32 fw_len;
	u8 *buf;

	FTS_INFO("[UPGRADE]**********start upgrade with app.i**********");

	fw_len = fts_getsize(FW_SIZE);
	if (fw_len < APP_FILE_MIN_SIZE || fw_len > APP_FILE_MAX_SIZE) {
		FTS_ERROR("[UPGRADE]: FW length(%x) error", fw_len);
		return -EIO;
	}
	buf = comp_upgrade_vars[g_global.index].upgrade_fw;
#if UPGRADE_ALL
	if (upgrade_flag[0])
		i_ret = fts_ctpm_lcd_cfg_upgrade_use_buf(client, buf, 4096);
	if (upgrade_flag[1])
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, buf + 0x5000, fw_len - 0x5000);
#else
	i_ret = fts_ctpm_fw_upgrade_use_buf(client, buf, fw_len);
#endif
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE] upgrade app.i failed");
	} else {
		FTS_INFO("[UPGRADE]: upgrade app.i succeed");
	}

	return i_ret;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_with_app_bin_file
 * Brief: upgrade with *.bin file
 * Input: i2c info, file name
 * Output: no
 * Return: success = 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_with_app_bin_file(struct i2c_client *client, char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	bool ecc_ok = false;
	int fwsize = fts_GetFirmwareSize(firmware_name);

	FTS_INFO("[UPGRADE]**********start upgrade with app.bin**********");

	if (fwsize < APP_FILE_MIN_SIZE || fwsize > APP_FILE_MAX_SIZE) {
		FTS_ERROR("[UPGRADE]: app.bin length(%x) error, upgrade fail", fwsize);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (NULL == pbt_buf) {
		FTS_ERROR(" malloc pbt_buf failed ");
		goto ERROR_BIN;
	}

	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		FTS_ERROR("[UPGRADE]: request_firmware failed!!");
		goto ERROR_BIN;
	}

	/*check the app.bin invalid or not */
	ecc_ok = 1;/*fts_check_app_bin_valid_idc(pbt_buf); */

	if (ecc_ok) {
		FTS_INFO("[UPGRADE] app.bin ecc ok");
#if UPGRADE_ALL
		i_ret = fts_ctpm_lcd_cfg_upgrade_use_buf(client, pbt_buf, 4096);
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, pbt_buf + 0x5000, fwsize - 0x5000);
#else
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, pbt_buf, fwsize);
#endif
		if (i_ret < 0) {
			FTS_ERROR("[UPGRADE]: upgrade app.bin failed");
			goto ERROR_BIN;
		} else {
			FTS_INFO("[UPGRADE]: upgrade app.bin succeed");
		}
	} else {
		FTS_ERROR("[UPGRADE] app.bin ecc failed");
		goto ERROR_BIN;
	}

	kfree(pbt_buf);
	return i_ret;
ERROR_BIN:
	kfree(pbt_buf);
	return -EIO;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_with_lcd_cfg_i_file
 * Brief:  upgrade with *.i file
 * Input: i2c info
 * Output: no
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_with_lcd_cfg_i_file(struct i2c_client *client)
{
	int i_ret = 0;
	int lcd_cfg_size = 0;

	FTS_DEBUG("[UPGRADE]**********upgrade with lcd_cfg.i**********");

	lcd_cfg_size = fts_getsize(LCD_CFG_SIZE);
	if (lcd_cfg_size < LCD_CFG_MIN_SIZE || lcd_cfg_size > LCD_CFG_MAX_SIZE) {
		FTS_ERROR("[UPGRADE] lcd_cfg.i length(%x) error", lcd_cfg_size);
		return -EIO;
	}

	/*FW upgrade */
	i_ret = fts_ctpm_lcd_cfg_upgrade_use_buf(client, CTPM_LCD_CFG, lcd_cfg_size);
	if (i_ret != 0) {
		FTS_ERROR("[UPGRADE] lcd_cfg.i upgrade fail, ret=%d", i_ret);
	} else {
		FTS_INFO("[UPGRADE] lcd_cfg.i upgrade succeed");
	}

	return i_ret;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_with_lcd_cfg_bin_file
 * Brief:  upgrade with *.bin file
 * Input: i2c info, file name
 * Output: no
 * Return: success = 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_with_lcd_cfg_bin_file(struct i2c_client *client, char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	bool ecc_ok = false;
	int lcd_cfg_size = fts_GetFirmwareSize(firmware_name);

	FTS_DEBUG("[UPGRADE]**********upgrade with lcd_cfg.bin**********");

	if (lcd_cfg_size < LCD_CFG_MIN_SIZE || lcd_cfg_size > LCD_CFG_MAX_SIZE) {
		FTS_ERROR("[UPGRADE] lcd_cfg.bin length(%x) error", lcd_cfg_size);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(lcd_cfg_size + 1, GFP_KERNEL);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		FTS_ERROR("[UPGRADE]: request_firmware failed!!");
		goto ERROR_LCD_CFG_BIN;
	}

	/*check the app.bin invalid or not */
	ecc_ok = 1;

	if (ecc_ok) {
		FTS_INFO("[UPGRADE]: lcd_cfg.bin ecc ok!!");
		i_ret = fts_ctpm_lcd_cfg_upgrade_use_buf(client, pbt_buf, lcd_cfg_size);
		if (i_ret != 0) {
			FTS_ERROR("[UPGRADE]: lcd_cfg.bin upgrade failed!!");
			goto ERROR_LCD_CFG_BIN;
		} else {
			FTS_INFO("[UPGRADE]: lcd_cfg.bin upgrade succeed!!");
		}
	} else {
		FTS_ERROR("[UPGRADE]: lcd_cfg.bin ecc failed!!");

	}

	kfree(pbt_buf);
	return i_ret;

ERROR_LCD_CFG_BIN:
	kfree(pbt_buf);
	return -EIO;
}


int fts_upgrade_buf_8006(struct i2c_client *client, u8 *fw_buf, u32 fw_size)
{
	int i_ret;

#if UPGRADE_ALL
		i_ret = fts_ctpm_lcd_cfg_upgrade_use_buf(client, fw_buf, 4096);
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, fw_buf + 0x5000, fw_size - 0x5000);
#else
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, fw_buf, fw_size);
#endif

	if (i_ret < 0) {
		VTE("[UPGRADE] upgrade app failed");
	} else {
		VTI("[UPGRADE]: upgrade app succeed");
	}

	return i_ret;
}

int fts_ctpm_write_pramboot_8006(struct i2c_client *client, u32 length, u8 *readbuf)
{
	u32 i = 0;
	u32 j;
	u32 k = 0;
	u32 temp;
	u32 packet_number;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 ecc = 0;

	FTS_INFO("[UPGRADE]**********write pramboot to pram**********");

	temp = 0;
	packet_number = (length) / FTS_PACKET_LENGTH;
	if ((length) % FTS_PACKET_LENGTH > 0) {
		packet_number++;
	}
	packet_buf[0] = 0xae;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		if (j < (packet_number - 1)) {
			temp = FTS_PACKET_LENGTH;
		} else {
			temp = (length) % FTS_PACKET_LENGTH ? (length) % FTS_PACKET_LENGTH : FTS_PACKET_LENGTH;
		}
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp / 4; i++) {
			for (k = 0; k < 4; k++) {
				packet_buf[6 + i * 4 + k] = readbuf[j * FTS_PACKET_LENGTH + i * 4 + 3 - k];
				ecc ^= packet_buf[6 + i * 4 + k];
			}
		}
		fts_i2c_write(client, packet_buf, temp + 6);
	}

	FTS_DEBUG("ecc:%x", ecc);
	return (int)ecc;
}

int fts_ctpm_pramboot_ecc_8006(struct i2c_client *client)
{
	u8 auc_i2c_write_buf[10];
	u8 reg_val[4] = {0};

	FTS_FUNC_ENTER();

	/*read out checksum, if pramboot checksum != host checksum, upgrade fail */
	FTS_INFO("[UPGRADE]******read out pramboot checksum******");
	auc_i2c_write_buf[0] = 0xcc;
	msleep(2);
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);

	FTS_FUNC_EXIT();

	return reg_val[0];
}

int fts_pramboot_write(struct i2c_client *client)
{
	int ecc_in_host = 0;
	int ecc_in_pram = 0;

	/*write pramboot to pram */
	ecc_in_host = fts_ctpm_write_pramboot_8006(client, comp_upgrade_vars[g_global.index].pb_length,
				comp_upgrade_vars[g_global.index].pramboot_fw);

	if (ecc_in_host < 0) {
		FTS_ERROR("[UPGRADE]: write pramboot fail!!");
		return ecc_in_host;
	}

	/*read out checksum */
	ecc_in_pram = fts_ctpm_pramboot_ecc_8006(client);
	if (ecc_in_pram < 0) {
		FTS_ERROR("[UPGRADE]: write pramboot ecc error!!");
		return ecc_in_pram;
	}

	FTS_DEBUG("[UPGRADE]pramboot_ecc = %X, host_ecc = %X!!", ecc_in_pram, ecc_in_host);
	if (ecc_in_pram != ecc_in_host) {/*pramboot checksum != host checksum, upgrade fail */
		FTS_ERROR("[UPGRADE]: checksum fail : pramboot_ecc = %X, host_ecc = %X!!", ecc_in_pram, ecc_in_host);
		return -EIO;
	}

	/*start pram */
	fts_ctpm_start_pramboot(client);

	return 0;
}

int fts_enter_into_pramboot(struct i2c_client *client)
{
	int ret = 0;
	bool state;

	fts_reset_proc(0);
	mdelay(6);

	ret = fts_ctpm_i2c_hid2std(client);
	state = fts_ctpm_check_run_state(client, FTS_RUN_IN_ROM);
	if (!state) {
		FTS_ERROR("[UPGRADE]: not run in romboot");
		return -EIO;
	}

	fts_pramboot_write(client);

	ret = fts_ctpm_i2c_hid2std(client);
	state = fts_ctpm_check_run_state(client, FTS_RUN_IN_PRAM);
	if (!state) {
		FTS_ERROR("[UPGRADE]: not run in pramboot");
		return -EIO;
	}

	return 0;
}

int fts_ctpm_erase_flash_8006(struct i2c_client *client)
{
	u32 i = 0;
	u8 auc_i2c_write_buf[10];
	u8 reg_val[4] = {0};

	FTS_INFO("[UPGRADE]**********erase app now**********");

	/*send to erase flash */
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(30);

	for (i = 0; i < 15; i++) {
		/*get the erase app status, if get 0xF0AA¡ê?erase flash success */
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0xAA == reg_val[1]) {/*erase flash success */
			break;
		}
		msleep(50);
	}

	if ((0xF0 != reg_val[0] || 0xAA != reg_val[1]) && (i >= 15)) {/*erase flash fail */
		FTS_ERROR("[UPGRADE]: erase app error.reset tp and reload FW!!");
		return -EIO;
	}
	FTS_DEBUG("[UPGRADE]: erase app ok!!");

	return 0;
}

int fts_read_imei_flash_8006(u8 *buf, u32 offset, u32 len)
{
	int ret = 0;
	u8 cmd[4] = { 0 };
	u32 addr = 0;
	struct i2c_client *client = fts_i2c_client;

	FTS_INFO("read imei len:%d", len);
	if (len > IMEI_MAX_LEN + 1) {
		FTS_ERROR("write length error");
		return -EINVAL;
	}

	ret = fts_enter_into_pramboot(client);
	if (ret < 0) {
		FTS_ERROR("enter pramboot fail");
		goto boot_reset;
	}

	addr = IMEI_SERCOTR * 4096;
	cmd[0] = 0x03;
	cmd[1] = (u8)((addr >> 16) & 0xFF);
	cmd[2] = (u8)((addr >> 8) & 0xFF);
	cmd[3] = (u8)(addr & 0xFF);
	ret = fts_i2c_write(client, cmd, 4);
	msleep(2);
	ret = fts_i2c_read(client, NULL, 0, buf, len);

	FTS_DEBUG("read imei ok");
boot_reset:
	fts_ctpm_rom_or_pram_reset(client);

	return ret;
}

int fts_write_imei_flash_8006(u8 *buf, u32 offset, u32 len)
{
	int ret = 0;
	u32 i = 0;
	u8 cmd[IMEI_MAX_LEN + 1 + 6] = { 0 };
	u32 addr = 0;
	struct i2c_client *client = fts_i2c_client;

	if (len > IMEI_MAX_LEN + 1) {
		FTS_ERROR("write length error");
		return -EINVAL;
	}
	ret = fts_enter_into_pramboot(client);
	if (ret < 0) {
		FTS_ERROR("enter pramboot fail");
		goto boot_reset;
	}

	cmd[0] = 0x09;
	cmd[1] = 0x80 + IMEI_SERCOTR;
	cmd[2] = 0x01;
	fts_i2c_write(client, cmd, 3);

	fts_ctpm_erase_flash_8006(client);

	addr = IMEI_SERCOTR * 4096;
	cmd[0] = 0xbf;
	cmd[1] = (u8)((addr >> 16) & 0xFF);
	cmd[2] = (u8)((addr >> 8) & 0xFF);
	cmd[3] = (u8)(addr & 0xFF);
	cmd[4] = (u8)((len >> 8) & 0xFF);
	cmd[5] = (u8)(len & 0xFF);
	for (i = 0; i < len; i++) {
		cmd[6 + i] = buf[i];
	}
	ret = fts_i2c_write(client, cmd, len + 6);
	msleep(10);

	memset(cmd, 0, IMEI_MAX_LEN);
	cmd[0] = 0x03;
	cmd[1] = (u8)((addr >> 16) & 0xFF);
	cmd[2] = (u8)((addr >> 8) & 0xFF);
	cmd[3] = (u8)(addr & 0xFF);
	ret = fts_i2c_write(client, cmd, 4);
	msleep(2);
	ret = fts_i2c_read(client, NULL, 0, cmd + 4, len);

	FTS_DEBUG("write:%s read:%s len:%d", buf, cmd + 4, len);
	if (memcmp(buf, cmd + 4, len) != 0) {
		FTS_ERROR("write fail,write:%s read:%s len:%d", buf, cmd + 4, len);
		goto boot_reset;
	}

	FTS_DEBUG("write imei ok");
boot_reset:
	fts_ctpm_rom_or_pram_reset(client);
	return ret;

}


int fts_force_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j = 0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 upgrade_ecc;
	int i_ret = 0;

	FTS_INFO("force upgrade fw");

	fts_reset_proc(0);
	mdelay(6);

	for (i = 0; i < 30; i++) {
	    fts_ctpm_i2c_hid2std(client);
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xAA;
		fts_i2c_write(client, auc_i2c_write_buf, 2);

		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x0;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		FTS_DEBUG("8006 romboot id:%x%x", reg_val[0], reg_val[1]);
		if ((0x80 == reg_val[0]) && (0x06 == reg_val[1]))
			break;
		mdelay(2);
	}

	if (i >= 30) {
		FTS_ERROR("romboot id fail");
		goto reset_fw;
	}

	fts_pramboot_write(client);

	for (i = 0; i < 30; i++) {
	    fts_ctpm_i2c_hid2std(client);
		auc_i2c_write_buf[0] = 0x55;
		auc_i2c_write_buf[1] = 0xAA;
		fts_i2c_write(client, auc_i2c_write_buf, 2);

		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x0;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		FTS_DEBUG("8006 pramboot id:%x%x", reg_val[0], reg_val[1]);
		if ((0x80 == reg_val[0]) && (0xC6 == reg_val[1]))
			break;
		mdelay(2);
	}

	if (i >= 30) {
		FTS_ERROR("pramboot id fail");
		goto reset_fw;
	}

	/*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download */
	auc_i2c_write_buf[0] = 0x09;
	auc_i2c_write_buf[1] = 0x0A;
	fts_i2c_write(client, auc_i2c_write_buf, 2);

	    /*erase the app erea in flash */
	i_ret = fts_ctpm_erase_flash(client);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE]: erase flash error!!");
		goto reset_fw;
	}

	/*write FW to ctpm flash */
	upgrade_ecc = 0;
	FTS_DEBUG("[UPGRADE]: write FW to ctpm flash!!");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000 + (0x0 / FTS_PACKET_LENGTH)) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			upgrade_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		/*msleep(1); */

		for (i = 0; i < 30; i++) {
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((0x1000 + ((0x0 + packet_number * FTS_PACKET_LENGTH) / ((dw_lenth) % FTS_PACKET_LENGTH))) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}

			if (i > 15) {
				msleep(1);
				FTS_DEBUG("[UPGRADE]: write flash: host : %x status : %x!!", (j + 0x1000 + (0x5000 / FTS_PACKET_LENGTH)), (((reg_val[0]) << 8) | reg_val[1]));
			}
			/*msleep(1); */
			fts_ctpm_upgrade_delay(10000);
		}
	}

	msleep(50);

	/*********Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_DEBUG("[UPGRADE]: read out checksum!!");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);

	temp = 0x0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (64 * 1024 - 1);
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	temp = (0x0 + (64 * 1024 - 1));
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = (dw_lenth - (64 * 1024 - 1));
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth / 256);

	for (i = 0; i < 100; i++) {
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) {
			FTS_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);

	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != upgrade_ecc) {
		FTS_ERROR("[UPGRADE]: ecc error! FW=%02x upgrade_ecc=%02x!!", reg_val[0], upgrade_ecc);
		i_ret = -EIO;
		goto reset_fw;
	}
	FTS_DEBUG("[UPGRADE]: checksum %x %x!!", reg_val[0], upgrade_ecc);

reset_fw:
	FTS_DEBUG("[UPGRADE]: reset the new FW!!");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(1000);

	fts_ctpm_i2c_hid2std(client);

	FTS_INFO("force upgrade fw succuss");
	return i_ret;
}

int fts_force_upgrade_bin_8006(struct i2c_client *client, char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret = 0;
	int fwsize = fts_GetFirmwareSize(firmware_name);

	FTS_INFO("[UPGRADE]**********start force upgrade bin**********");

	if (fwsize < APP_FILE_MIN_SIZE || fwsize > APP_FILE_MAX_SIZE) {
		FTS_ERROR("[UPGRADE]: app.bin length(%x) error, upgrade fail", fwsize);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (NULL == pbt_buf) {
		FTS_ERROR(" malloc pbt_buf failed ");
		goto ERROR_BIN;
	}

	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		FTS_ERROR("[UPGRADE]: request_firmware failed!!");
		goto ERROR_BIN;
	}

	i_ret = fts_force_upgrade(client, pbt_buf, fwsize);
	if (i_ret < 0) {
		FTS_ERROR("[UPGRADE]: force upgrade bin failed");
		goto ERROR_BIN;
	} else {
		FTS_INFO("[UPGRADE]: force upgrade bin succeed");
	}


	kfree(pbt_buf);
	return i_ret;
ERROR_BIN:
	kfree(pbt_buf);
	return -EIO;
}


