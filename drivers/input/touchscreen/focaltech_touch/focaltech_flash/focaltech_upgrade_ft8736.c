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
 * File Name: focaltech_upgrade_ft8736.c
 *
 * Author:    fupeipei
 *
 * Created:    2016 -10 -25
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
#define APP_FILE_MAX_SIZE           (64 * 1024)
#define APP_FILE_MIN_SIZE           (8)
#define APP_FILE_VER_MAPPING        (0x10E)
#define APP_FILE_VENDORID_MAPPING   (0x10C)
#define APP_FILE_CHIPID_MAPPING     (0x11E)
#define CONFIG_START_ADDR           (0x0F80)
#define CONFIG_VENDOR_ID_OFFSET     (0x4)
#define CONFIG_PROJECT_ID_OFFSET    (0x20)
#define CONFIG_VENDOR_ID_ADDR       (CONFIG_START_ADDR + CONFIG_VENDOR_ID_OFFSET)
#define CONFIG_PROJECT_ID_ADDR      (CONFIG_START_ADDR + CONFIG_PROJECT_ID_OFFSET)

/*****************************************************************************
 * Global variable or extern global variabls / functions
 *****************************************************************************/
static int fts_ctpm_get_app_i_file_ver(void);
static int fts_ctpm_get_app_bin_file_ver(char *firmware_name);
static int fts_ctpm_fw_upgrade_with_app_i_file(struct i2c_client *client);
static int fts_ctpm_fw_upgrade_with_app_bin_file(struct i2c_client *client, char *firmware_name);

struct fts_upgrade_fun fts_updatefun_ft8736 = {
	.get_app_bin_file_ver = fts_ctpm_get_app_bin_file_ver,
	.get_app_i_file_ver = fts_ctpm_get_app_i_file_ver,
	.upgrade_with_app_i_file = fts_ctpm_fw_upgrade_with_app_i_file,
	.upgrade_with_app_bin_file = fts_ctpm_fw_upgrade_with_app_bin_file,
	.upgrade_with_lcd_cfg_i_file = NULL,
	.upgrade_with_lcd_cfg_bin_file = NULL,
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
		VTE("[UPGRADE]: FW length(%x) error", fwsize);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		VTE("[UPGRADE]: request_firmware failed!!");
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
		VTE("[UPGRADE]: FW length(%x) error", fwsize);
		return -EIO;
	}
	fwsize = comp_upgrade_vars[g_global.index].upgrade_fw[APP_FILE_VER_MAPPING];
	FTS_DEBUG("8736 fw version: %02X", fwsize);
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
		auc_i2c_write_buf[0] = 0x03;
		auc_i2c_write_buf[1] = 0x00;

		auc_i2c_write_buf[2] = (u8)((CONFIG_VENDOR_ID_ADDR - 1) >> 8);
		auc_i2c_write_buf[3] = (u8)(CONFIG_VENDOR_ID_ADDR - 1);
		i_ret = fts_i2c_read(client, auc_i2c_write_buf, 4, vendorid, 2);
		if (i_ret < 0) {
			VTI("[UPGRADE]: read flash : i_ret = %d!!", i_ret);
			continue;
		} else if ((vendorid[1] == FTS_VENDOR_1_ID) || (vendorid[1] == FTS_VENDOR_2_ID))
			break;
	}

	VTI("[UPGRADE]: vendor id from flash rom: 0x%x!!", vendorid[1]);
	if (i >= FTS_UPGRADE_LOOP) {
		VTE("[UPGRADE]: read vendor id from flash more than 30 times!!");
		return -EIO;
	}

	return 0;
#else
	return 0;
#endif
}

/************************************************************************
 * Name: fts_ctpm_write_pram
 * Brief: fw upgrade
 * Input: i2c info, file buf, file len
 * Output: no
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_write_pram(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	int i_ret;
	bool inrom = false;

	FTS_FUNC_ENTER();

	/*check the length of the pramboot */
	if (dw_lenth > APP_FILE_MAX_SIZE || dw_lenth < APP_FILE_MIN_SIZE) {
		VTE("[UPGRADE] pramboot length(%d) fail", dw_lenth);
		return -EIO;
	}

	/*send comond to FW, reset and start write pramboot */
	i_ret = fts_ctpm_start_fw_upgrade(client);
	if (i_ret < 0) {
		VTE("[UPGRADE]: send upgrade cmd to FW error!!");
		return i_ret;
	}

	/*check run in rom or not! if run in rom, will write pramboot */
	inrom = fts_ctpm_check_run_state(client, FTS_RUN_IN_ROM);
	if (!inrom) {
		VTE("[UPGRADE]: not run in rom, write pramboot fail!!");
		return -EIO;
	}

	/*write pramboot to pram */
	i_ret = fts_ctpm_write_pramboot_for_idc(client, dw_lenth,
				comp_upgrade_vars[g_global.index].pramboot_fw);

	if (i_ret < 0) {
		return i_ret;
		VTE("[UPGRADE]: write pramboot fail!!");
	}

	/*read out checksum */
	i_ret = fts_ctpm_pramboot_ecc(client);
	if (i_ret < 0) {
		return i_ret;
		VTE("[UPGRADE]: write pramboot ecc error!!");
	}

	/*start pram */
	fts_ctpm_start_pramboot(client);

	FTS_FUNC_EXIT();

	return 0;
}

/************************************************************************
 * Name: fts_ctpm_write_app
 * Brief:  fw upgrade
 * Input: i2c info, file buf, file len
 * Output: no
 * Return: fail < 0
 ***********************************************************************/
static int fts_ctpm_write_app(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u32 temp;
	int i_ret;
	bool inpram = false;

	FTS_FUNC_ENTER();

	/*check run in pramboot or not! if not rum in pramboot, can not upgrade */
	inpram = fts_ctpm_check_run_state(client, FTS_RUN_IN_PRAM);
	if (!inpram) {
		VTE("[UPGRADE]: not run in pram, upgrade fail!!");
		return -EIO;
	}

	/*upgrade init */
	i_ret = fts_ctpm_upgrade_idc_init(client, dw_lenth);
	if (i_ret < 0) {
		VTE("[UPGRADE]: upgrade init error, upgrade fail!!");
		return i_ret;
	}

	/*read vendor id from flash, if vendor id error, can not upgrade */
	i_ret = fts_ctpm_get_vendor_id_flash(client);
	if (i_ret < 0) {
		VTE("[UPGRADE]: read vendor id in flash fail!!");
		return i_ret;
	}

	/*erase the app erea in flash */
	i_ret = fts_ctpm_erase_flash(client);
	if (i_ret < 0) {
		VTE("[UPGRADE]: erase flash error!!");
		return i_ret;
	}

	/*start to write app */
	i_ret = fts_ctpm_write_app_for_idc(client, dw_lenth, pbt_buf);
	if (i_ret < 0) {
		VTE("[UPGRADE]: write app error!!");
		return i_ret;
	}

	/*read check sum */
	temp = 0x1000;
	i_ret = fts_ctpm_upgrade_ecc(client, temp, dw_lenth);
	if (i_ret < 0) {
		VTE("[UPGRADE]: ecc error!!");
		return i_ret;
	}

	/*upgrade success, reset the FW */
	fts_ctpm_rom_or_pram_reset(client);

	FTS_FUNC_EXIT();

	return 0;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade_use_buf
 * Brief: fw upgrade
 * Input: i2c info, file buf, file len
 * Output: no
 * Return: fail < 0
 *         success = 0
 ***********************************************************************/
static int fts_ctpm_fw_upgrade_use_buf(struct i2c_client *client, u8 *pbt_buf, u32 fwsize)
{
	int i_ret = 0;
	int fw_len;

	FTS_FUNC_ENTER();

	/*write pramboot */
	fw_len = fts_getsize(PRAMBOOT_SIZE);
	FTS_DEBUG("[UPGRADE]: pramboot size : %d!!", fw_len);
	i_ret = fts_ctpm_write_pram(client,
				comp_upgrade_vars[g_global.index].pramboot_fw, fw_len);
	if (i_ret != 0) {
		VTE("[UPGRADE]: write pram failed!!");
		return -EIO;
	}

	/*write app */
	i_ret =  fts_ctpm_write_app(client, pbt_buf, fwsize);

	FTS_FUNC_EXIT();

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

	VTI("[UPGRADE]**********start upgrade with app.i**********");

	fw_len = fts_getsize(FW_SIZE);
	if (fw_len < APP_FILE_MIN_SIZE || fw_len > APP_FILE_MAX_SIZE) {
		VTE("[UPGRADE]: FW length(%x) error", fw_len);
		return -EIO;
	}

	i_ret = fts_ctpm_fw_upgrade_use_buf(client,
		comp_upgrade_vars[g_global.index].upgrade_fw, fw_len);
	if (i_ret != 0) {
		VTE("[UPGRADE] upgrade app.i failed");
	} else {
		VTI("[UPGRADE]: upgrade app.i succeed");
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

	VTI("[UPGRADE]**********start upgrade with app.bin**********");

	if (fwsize < APP_FILE_MIN_SIZE || fwsize > APP_FILE_MAX_SIZE) {
		VTE("[UPGRADE]: app.bin length(%x) error, upgrade fail", fwsize);
		return -EIO;
	}

	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_KERNEL);
	if (NULL == pbt_buf) {
		VTE(" malloc pbt_buf failed ");
		goto ERROR_BIN;
	}

	if (fts_ReadFirmware(firmware_name, pbt_buf)) {
		VTE("[UPGRADE]: request_firmware failed!!");
		goto ERROR_BIN;
	}
#if FTS_GET_VENDOR_ID
	if ((pbt_buf[APP_FILE_VENDORID_MAPPING] != FTS_VENDOR_1_ID) && (pbt_buf[APP_FILE_VENDORID_MAPPING] != FTS_VENDOR_2_ID)) {
		VTE("[UPGRADE]: vendor id is error, app.bin upgrade failed!!");
		goto ERROR_BIN;
	}
#endif
	if ((pbt_buf[APP_FILE_CHIPID_MAPPING] != chip_types.pramboot_idh)
		 || (pbt_buf[APP_FILE_CHIPID_MAPPING + 1] != chip_types.pramboot_idl)) {
		VTE("[UPGRADE]: chip id is error, app.bin upgrade failed!!");
		goto ERROR_BIN;
	}

	/*check the app.bin invalid or not */
	ecc_ok = fts_check_app_bin_valid_idc(pbt_buf);

	if (ecc_ok) {
		VTI("[UPGRADE] app.bin ecc ok");
		i_ret = fts_ctpm_fw_upgrade_use_buf(client, pbt_buf, fwsize);
		if (i_ret != 0) {
			VTE("[UPGRADE]: upgrade app.bin failed");
			goto ERROR_BIN;
		} else {
			VTI("[UPGRADE]: upgrade app.bin succeed");
		}
	} else {
		VTE("[UPGRADE] app.bin ecc failed");
		goto ERROR_BIN;
	}

	kfree(pbt_buf);
	return i_ret;
ERROR_BIN:
	kfree(pbt_buf);
	return -EIO;
}

int fts_upgrade_buf_8736(struct i2c_client *client, u8 *fw_buf, u32 fw_size)
{
	int i_ret;

	i_ret = fts_ctpm_fw_upgrade_use_buf(client, fw_buf, fw_size);
	if (i_ret != 0) {
		VTE("[UPGRADE] upgrade app failed");
	} else {
		VTI("[UPGRADE]: upgrade app succeed");
	}

	return i_ret;
}

static int fts_ctpm_erase_flash_less(struct i2c_client *client)
{
	u32 i = 0;
	u8 auc_i2c_write_buf[10];
	u8 reg_val[4] = {0};

	VTI("[IMEI]**********erase app now**********");

	/*send to erase flash */
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(50);

	for (i = 0; i < 15; i++) {
		/*get the erase app status, if get 0xF0AA£¬erase flash success */
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		VTI("[IMEI] Erase ret=%02x%02x", reg_val[0], reg_val[1]);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1]) {/*erase flash success */
			break;
		}
		msleep(50);
	}

	if ((0xF0 != reg_val[0] || 0xAA != reg_val[1]) && (i >= 15)) {/*erase flash fail */
		VTE("[UPGRADE]: erase app error.reset tp and reload FW!!");
		return -EIO;
	}
	VTI("[UPGRADE]: erase app ok!!");

	return 0;
}


int fts_write_imei_flash_8736(u8 *buf, u32 offset, u32 len)
{
	struct i2c_client *client = fts_i2c_client;
	int fw_status = 0;
	int ret = 0;
	int fw_len;
	bool inpram = false;
	u8 rwbuf[32] = { 0 };
	u32 sector = 0x1E;
	int i;

	fw_status = fts_ctpm_check_fw_status(client);
	VTI("[IMEI]fw_status = %d", fw_status);
	if (fw_status < 0) {
		/* I2C no ACK, return immediately */
		VTE("[IMEI]******I2C NO ACK,exit upgrade******");
		return -EIO;
	} else if (fw_status == FTS_RUN_IN_ERROR) {
		VTE("[IMEI]******IC Type Fail******");
		return -EIO;
	} else if (fw_status == FTS_RUN_IN_APP) {
		VTE("[IMEI]******APP valid******");
	} else {
		VTI("[IMEI]**********FW APP invalid**********");
		fts_ctpm_rom_or_pram_reset(client);
	}

	/*write pramboot */
	fw_len = fts_getsize(PRAMBOOT_SIZE);
	VTI("[IMEI]: pramboot size : %d!!", fw_len);
	ret = fts_ctpm_write_pram(client, comp_upgrade_vars[g_global.index].pramboot_fw, fw_len);
	if (ret != 0) {
		VTE("[IMEI]: write pram failed!!");
		ret = -EIO;
		goto imei_write_err;
	}

	/*check run in pramboot or not! if not rum in pramboot, can not upgrade */
	inpram = fts_ctpm_check_run_state(client, FTS_RUN_IN_PRAM);
	if (!inpram) {
		VTE("[IMEI]: not run in pram, upgrade fail!!");
		ret = -EIO;
		goto imei_write_err;
	}

	/*upgrade init */
	ret = fts_ctpm_upgrade_idc_init(client, fts_getsize(FW_SIZE));
	if (ret < 0) {
		VTE("[IMEI]: upgrade init error, upgrade fail!!");
		ret = -EIO;
		goto imei_write_err;
	}

	/*send upgrade type to reg 0x09 for erase command */
	rwbuf[0] = 0x09;
	rwbuf[1] = sector + 0x80;
	rwbuf[2] = 0x01;
	fts_i2c_write(client, rwbuf, 3);

	/*erase the app erea in flash */
	ret = fts_ctpm_erase_flash_less(client);
	if (ret < 0) {
		VTE("[IMEI]: erase flash error!!");
		ret = -EIO;
		goto imei_write_err;
	}

	VTI("[IMEI] Write Data");
	rwbuf[0] = 0xbf;
	rwbuf[1] = (u8) ((sector * 0x1000) >> 16);
	rwbuf[2] = (u8) ((sector * 0x1000) >> 8);
	rwbuf[3] = (u8) (sector * 0x1000);
	rwbuf[4] = (u8) (len >> 8);
	rwbuf[5] = (u8) len;

	for (i = 0; i < len; i++)
		rwbuf[6 + i] = buf[i];

	fts_i2c_write(client, rwbuf, len + 6);
	msleep(1);
	for (i = 0; i < 30; i++) {
		/*read status and check if the app writting is finished */
		rwbuf[0] = 0x6a;
		fts_i2c_read(client, rwbuf, 1, rwbuf, 2);
		VTI("[IMEI] command Write ret=%02x%02x", rwbuf[0], rwbuf[1]);
		if ((0x1000 + (sector * 0x1000) / len) == (((rwbuf[0]) << 8) | rwbuf[1])) {
			break;
		}
		/*msleep(1); */
		fts_ctpm_upgrade_delay(1000);
	}

#if 0
	u8 rwbuf1[4] = { 0 };
	u8 rwbuf2[32] = { 0 };
	rwbuf1[0] = 0x03;
	rwbuf1[1] = (u8)((sector * 0x1000) >> 16);
	rwbuf1[2] = (u8)((sector * 0x1000) >> 8);
	rwbuf1[3] = (u8)(sector * 0x1000);
	fts_i2c_write(client, rwbuf1, 4);
	msleep(10);
	fts_i2c_read(client, NULL, 0, rwbuf2, len);

	VTI("[IMEI]Write - Read:%s  %x %c %x", rwbuf2, rwbuf2[15], rwbuf2[15], rwbuf2[16]);
#endif
	ret = len;
imei_write_err:
	/*upgrade success, reset the FW */
	fts_ctpm_rom_or_pram_reset(client);

	return ret;
}

int fts_read_imei_flash_8736(u8 *buf, u32 offset, u32 len)
{

	struct i2c_client *client = fts_i2c_client;
	int fw_status = 0;
	int ret = 0;
	int fw_len;
	bool inpram = false;
	u8 rwbuf[4] = { 0 };
	u32 sector = 0x1E;

	fw_status = fts_ctpm_check_fw_status(client);
	VTI("[IMEI]fw_status = %d", fw_status);
	if (fw_status < 0) {
		/* I2C no ACK, return immediately */
		VTE("[IMEI]******I2C NO ACK,exit upgrade******");
		return -EIO;
	} else if (fw_status == FTS_RUN_IN_ERROR) {
		VTE("[IMEI]******IC Type Fail******");
		return -EIO;
	} else if (fw_status == FTS_RUN_IN_APP) {
		VTE("[IMEI]******APP valid******");
/*		fts_ctpm_start_fw_upgrade(client); */
	} else {
		VTI("[IMEI]**********FW APP invalid**********");
		fts_ctpm_rom_or_pram_reset(client);
	}

	/*write pramboot */
	fw_len = fts_getsize(PRAMBOOT_SIZE);
	VTI("[IMEI]: pramboot size : %d!!", fw_len);
	ret = fts_ctpm_write_pram(client, comp_upgrade_vars[g_global.index].pramboot_fw, fw_len);
	if (ret != 0) {
		VTE("[IMEI]: write pram failed!!");
		ret = -EIO;
		goto imei_read_err;
	}

	/*check run in pramboot or not! if not rum in pramboot, can not upgrade */
	inpram = fts_ctpm_check_run_state(client, FTS_RUN_IN_PRAM);
	if (!inpram) {
		VTE("[IMEI]: not run in pram, upgrade fail!!");
		ret = -EIO;
		goto imei_read_err;
	}

	/*upgrade init */
	ret = fts_ctpm_upgrade_idc_init(client, fts_getsize(FW_SIZE));
	if (ret < 0) {
		VTE("[IMEI]: upgrade init error, upgrade fail!!");
		ret = -EIO;
		goto imei_read_err;
	}

	rwbuf[0] = 0x03;
	rwbuf[1] = (u8)((sector * 0x1000) >> 16);
	rwbuf[2] = (u8)((sector * 0x1000) >> 8);
	rwbuf[3] = (u8)(sector * 0x1000);
	fts_i2c_write(client, rwbuf, 4);
	msleep(10);
	fts_i2c_read(client, NULL, 0, buf, len);

	VTI("[IMEI]Read:%s", buf);

	ret = len;
imei_read_err:
	/*upgrade success, reset the FW */
	fts_ctpm_rom_or_pram_reset(client);

	return ret;
}


