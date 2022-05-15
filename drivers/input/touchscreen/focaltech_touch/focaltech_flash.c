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
 * File Name: focaltech_flash.c
 *
 * Author:    fupeipei
 *
 * Created:    2016 -08 -08
 *
 * Abstract:
 *
 * Reference:
 *
 *****************************************************************************/

/*****************************************************************************
 * 1.Included header files
 *****************************************************************************/
#include "focaltech_core.h"
#include "focaltech_flash.h"

/*****************************************************************************
 * Static variables
 *****************************************************************************/
struct ft_chip_t chip_types;



unsigned char CTPM_FW[] = {
#include FTS_UPGRADE_FW_APP
};


unsigned char CTPM_FW_8006[] = {
#ifdef FTS_UPGRADE_FW_APP2
#include FTS_UPGRADE_FW_APP2
#endif
};


unsigned char aucFW_PRAM_BOOT[] = {
#ifdef FTS_UPGRADE_PRAMBOOT
#include FTS_UPGRADE_PRAMBOOT
#endif
};

u8 pramboot_8006[] = {
	#include FTS_PRAMBOOT_8006
};

unsigned char CTPM_LCD_CFG[] = {
#ifdef FTS_UPGRADE_LCD_CFG
#include FTS_UPGRADE_LCD_CFG
#endif
};

extern struct fts_upgrade_fun fts_updatefun_ft8736;
extern struct fts_upgrade_fun fts_updatefun_ft8006;
struct ft_chip_upgrade_func_t comp_upgrade_vars[] =
{
	[0] = {
		.type = _FT8006, 
		.is_idc = 1,
		.funcs = &fts_updatefun_ft8006,
		.pramboot_fw = pramboot_8006,
		.pb_length = sizeof(pramboot_8006),
		.upgrade_fw = CTPM_FW_8006,
		.fw_length = sizeof(CTPM_FW_8006),
	},
	[1] = {
		.type = _FT8736,
		.is_idc = 1,
		.funcs = &fts_updatefun_ft8736,
		.pramboot_fw = aucFW_PRAM_BOOT,
		.pb_length = sizeof(aucFW_PRAM_BOOT),
		.upgrade_fw = CTPM_FW,
		.fw_length = sizeof(CTPM_FW),
	},
	[2] = {
		.type = _FT8719, 
		.is_idc = 1,
	},
};
/* xlg end */

/*****************************************************************************
 * Global variable or extern global variabls / functions
 *****************************************************************************/
struct fts_upgrade_fun  fts_updatefun_curr;

struct workqueue_struct *touch_wq;
struct work_struct fw_update_work;
#if 0
static void fts_show_struct(struct ft_chip_upgrade_func_t *fcuf)
{
	FTS_DEBUG("type: %lX, is_idc: %d; upfunc: %p; pramboot_fw[0]: %02X,"
		"pramboot_len: %ld; app_fw[0]:%02X, app_len: %ld",
		fcuf->type, fcuf->is_idc, fcuf->funcs,
		(fcuf->pramboot_fw ? fcuf->pramboot_fw[0] : 0xEF), fcuf->pb_length, fcuf->upgrade_fw[0], fcuf->fw_length);
}
#endif

/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/

/************************************************************************
 * Name: fts_ctpm_upgrade_delay
 * Brief: 0
 * Input: 0
 * Output: 0
 * Return: 0
 ***********************************************************************/
void fts_ctpm_upgrade_delay(u32 i)
{
	do {
		i--;
	} while (i > 0);
}

/************************************************************************
 * Name: fts_ctpm_i2c_hid2std
 * Brief:  HID to I2C
 * Input: i2c info
 * Output: no
 * Return: fail = 0
 ***********************************************************************/
int fts_ctpm_i2c_hid2std(struct i2c_client *client)
{
	u8 buf[5] = {0};
	int bRet = 0;

	if (g_global.is_idc) {
		return 0;
	}

	buf[0] = 0xeb;
	buf[1] = 0xaa;
	buf[2] = 0x09;
	bRet = fts_i2c_write(client, buf, 3);
	msleep(10);
	buf[0] = buf[1] = buf[2] = 0;
	fts_i2c_read(client, buf, 0, buf, 3);

	if ((0xeb == buf[0]) && (0xaa == buf[1]) && (0x08 == buf[2])) {
		VTD("hidi2c change to stdi2c successful!!");
		bRet = 1;
	} else {
		VTE("hidi2c change to stdi2c error!!");
		bRet = 0;
	}

	return bRet;
}



/************************************************************************
 * Name: fts_get_chip_types
 * Brief: get correct chip id
 * Input:
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
int fts_get_chip_types(u8 id_h, u8 id_l, bool fw_valid)
{
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	int i = 0;

	if ((0x0 == id_h) || (0x0 == id_l)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	for (i = 0; i < ctype_entries; i++) {
		if (fw_valid == VALID) {
			if ((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
				|| ((id_h == ctype[i].pramboot_idh) && (id_l == ctype[i].pramboot_idl))
				|| ((id_h == ctype[i].bootloader_idh) && (id_l == ctype[i].bootloader_idl))) {
				break;
			}
		}
	}

	if (i >= ctype_entries) {
		FTS_ERROR("Can't get correct chip types, please check FTS_CHIP_TYPE_MAPPING");
		return -ENODATA;
	}

	chip_types = ctype[i];
	return 0;
}

/*****************************************************************************
 *  Name: fts_get_ic_information
 *  Brief:
 *  Input:
 *  Output:
 *  Return: return 0 if success, otherwise return error code
 *****************************************************************************/
static int fts_get_ic_information(struct i2c_client *client)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };
    u8 id_cmd[4] = { 0 };
	
    do {
		msleep(INTERVAL_READ_REG);
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &chip_id[0]);
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID2, &chip_id[1]);
		if (ret < 0) {
			FTS_ERROR("i2c read error:%d", ret);
		} else if ((chip_id[0] == 0x0) || (chip_id[0] == 0xFF) || (chip_id[0] == 0xEB)
			|| (chip_id[1] == 0x0) || (chip_id[1] == 0xFF) || (chip_id[1] == 0xEB)) {
			FTS_DEBUG("TP read error or fw invalid, read:0x%02x%02x", chip_id[1], chip_id[0]);
		} else {
			ret = fts_get_chip_types(chip_id[0], chip_id[1], VALID);
			if (ret == 0)
				break;
			else
				FTS_DEBUG(" TP not ready, read:0x%02x%02x", chip_id[1], chip_id[0]);
		}

		cnt++;
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    if((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG)
    {
        FTS_INFO("fw is invalid, need read boot id");
		id_cmd[0] = FTS_UPGRADE_55;
		id_cmd[1] = FTS_UPGRADE_AA;
		ret = fts_i2c_write(client, id_cmd, 2);
		if (ret < 0) {
			FTS_ERROR("start cmd write fail");
			return ret;
		}
		msleep(10);

        id_cmd[0] = FTS_CMD_READ_ID;
		ret = fts_i2c_read(client, id_cmd, 1, chip_id, 2);
        if (ret < 0)
        {
            FTS_ERROR("i2c read error:%d", ret);
            return -EIO;
        }
		FTS_INFO("read boot id = 0x%02x%02x", chip_id[0], chip_id[1]);

        ret = fts_get_chip_types(chip_id[0], chip_id[1], INVALID);
        if(ret != 0)
        {
            FTS_ERROR("can't get ic informaton");
            return -ENODATA;
        }
    }

	return 0;
}


#if 0
static u8 fts_get_chip_id(struct i2c_client *client)
{
	u8  reg_value = 0;
	int ret;

	ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);


	return reg_value;
}
#endif
/************************************************************************
 * Name: fts_get_chip_types
 * Brief: get correct chip information
 * Input:
 * Output:
 * Return:
 ***********************************************************************/
void fts_init_chip_types(void)
{

	int i;
	int len;
#if 0
	/* init chip_types datas from true chip type */
	/* TODO: more conditions to recognize specified ic */
	/* if fw failed, how to confirm chip type */
	u8 chip_id = fts_get_chip_id(fts_i2c_client);

	FTS_DEBUG("comp_types size: %d", (int)(sizeof(comp_types) / sizeof(unsigned long)));

	len = sizeof(ctype) / sizeof(struct ft_chip_t);

	for (i = 0; i < len; i++) {
		FTS_DEBUG("chipid: %02X, type: %lX(FT%lX)", chip_id,
			(ctype[i].type), (comp_types[i]));

		if (chip_id == ctype[i].chip_idh)  {
			FTS_DEBUG("type selected!");
			ic_type = i;
			break;
		}
	}

	chip_types = ctype[ic_type];
#endif

	fts_get_ic_information(fts_i2c_client);

	g_global.index = 0;
	g_global.ic_type = comp_upgrade_vars[0].type;
	g_global.is_idc = comp_upgrade_vars[0].is_idc;
	memcpy(&fts_updatefun_curr, comp_upgrade_vars[0].funcs, sizeof(fts_updatefun_curr));


	len = sizeof(comp_upgrade_vars) / sizeof(struct ft_chip_upgrade_func_t);

	FTS_DEBUG("comp_upgrade_vars size: %d", len);
	for (i = 0; i < len; i++) {
		//fts_show_struct(&comp_upgrade_vars[i]);
		FTS_DEBUG("type comp: %lX(FT%lX), type selected: %lX", 
			TO_IC_SER(comp_upgrade_vars[i].type), 
			(comp_upgrade_vars[i].type),
			TO_IC_SER(chip_types.type));

		if (TO_IC_SER(comp_upgrade_vars[i].type) == chip_types.type) {
			FTS_DEBUG("upgrade funcs copied");

			g_global.index = i;
			g_global.ic_type = comp_upgrade_vars[i].type;
			g_global.is_idc = comp_upgrade_vars[i].is_idc;
			if(g_global.ic_type != _FT8719) {
				memcpy(&fts_updatefun_curr, comp_upgrade_vars[i].funcs, sizeof(fts_updatefun_curr));
			}
			break;
		}
	}


	FTS_INFO("CHIP TYPE ID = 0x%02x%02x", chip_types.chip_idh, chip_types.chip_idl);
}

/************************************************************************
 * Name: fts_ctpm_get_upgrade_array
 * Brief: decide which ic
 * Input: no
 * Output: get ic info in fts_updateinfo_curr
 * Return: no
 ***********************************************************************/
void fts_ctpm_get_upgrade_array(void)
{

	FTS_FUNC_ENTER();

	fts_init_chip_types();

	fts_ctpm_i2c_hid2std(fts_i2c_client);

	FTS_FUNC_EXIT();
}

/************************************************************************
 * Name: fts_ctpm_rom_or_pram_reset
 * Brief: RST CMD(07), reset to romboot(maybe->bootloader)
 * Input:
 * Output:
 * Return:
 ***********************************************************************/
void fts_ctpm_rom_or_pram_reset(struct i2c_client *client)
{
	u8 rst_cmd = FTS_REG_RESET_FW;

	VTI("[UPGRADE]******Reset to romboot/bootloader******");
	fts_i2c_write(client, &rst_cmd, 1);
	/* The delay can't be changed */
	msleep(400);
}

/************************************************************************
 * Name: fts_ctpm_auto_clb
 * Brief:  auto calibration
 * Input: i2c info
 * Output: no
 * Return: 0
 ***********************************************************************/
int fts_ctpm_auto_clb(struct i2c_client *client)
{
#if FTS_AUTO_CLB_EN
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/*start auto CLB */
	msleep(200);

	fts_i2c_write_reg(client, 0, FTS_REG_WORKMODE_FACTORY_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	fts_i2c_write_reg(client, 2, 0x4);
	msleep(300);
	if ((chip_types.chip_idh == 0x11) || (chip_types.chip_idh == 0x12) || (chip_types.chip_idh == 0x13) || (chip_types.chip_idh == 0x14)) { /*5x36, 5x36i */
		for (i = 0; i < 100; i++) {
			fts_i2c_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp ||
				0xFF == uc_temp) {
				break;
			}
			msleep(20);
		}
	} else {
		for (i = 0; i < 100; i++) {
			fts_i2c_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp&0x70) >> 4)) {
				break;
			}
			msleep(20);
		}
	}
	fts_i2c_write_reg(client, 0, 0x40);
	msleep(200);
	fts_i2c_write_reg(client, 2, 0x5);
	msleep(300);
	fts_i2c_write_reg(client, 0, FTS_REG_WORKMODE_WORK_VALUE);
	msleep(300);
#endif

	return 0;
}

/************************************************************************
 * Name: fts_GetFirmwareSize
 * Brief:  get file size
 * Input: file name
 * Output: no
 * Return: file size
 ***********************************************************************/
int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[FILE_NAME_LENGTH];
	struct filename *vts_name;

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, 1023, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile) {
		/*pfile = filp_open(filepath, O_RDONLY, 0);*/
		vts_name = getname_kernel(filepath);
		pfile = file_open_name(vts_name, O_RDONLY, 0);
		putname(vts_name);
	}
	if (IS_ERR(pfile)) {
		VTE("error occured while opening file %s", filepath);
		return -EIO;
	}
	/*inode = pfile->f_dentry->d_inode;*/
	inode = file_inode(pfile);
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/************************************************************************
 * Name: fts_ReadFirmware
 * Brief:  read firmware buf for .bin file.
 * Input: file name, data buf
 * Output: data buf
 * Return: 0
 ***********************************************************************/
int fts_ReadFirmware(char *firmware_name, unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[FILE_NAME_LENGTH];
	loff_t pos;
	mm_segment_t old_fs;
	struct filename *vts_name;

	memset(filepath, 0, sizeof(filepath));
	snprintf(filepath, 1023, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile) {
		/*pfile = filp_open(filepath, O_RDONLY, 0);*/
		vts_name = getname_kernel(filepath);
		pfile = file_open_name(vts_name, O_RDONLY, 0);
		putname(vts_name);
	}
	if (IS_ERR(pfile)) {
		VTE("[UPGRADE] Error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	/*inode = pfile->f_dentry->d_inode;*/
	inode = file_inode(pfile);
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/************************************************************************
 * Name: fts_getsize
 * Brief: 0
 * Input: 0
 * Output: 0
 * Return: 0
 ***********************************************************************/
u32 fts_getsize(u8 fw_type)
{
	u32 fw_len = 0;

	if (fw_type == FW_SIZE) {
		/*fw_len = sizeof(CTPM_FW); */
		fw_len = comp_upgrade_vars[g_global.index].fw_length;
	} else if (fw_type == PRAMBOOT_SIZE) {
		/*fw_len = sizeof(aucFW_PRAM_BOOT); */
		fw_len = comp_upgrade_vars[g_global.index].pb_length;
	} else if (fw_type == LCD_CFG_SIZE)
		fw_len = sizeof(CTPM_LCD_CFG);

	return fw_len;
}

/************************************************************************
 * Name: fts_ctpm_get_pram_or_rom_id
 * Brief: 0
 * Input: 0
 * Output: 0
 * Return: 0
 ***********************************************************************/
enum FW_STATUS fts_ctpm_get_pram_or_rom_id(struct i2c_client *client)
{
	u8 buf[4];
	u8 reg_val[2] = {0};
	enum FW_STATUS inRomBoot = FTS_RUN_IN_ERROR;

	fts_ctpm_i2c_hid2std(client);

	/*Enter upgrade mode */
	/*send 0x55 in time windows */
	buf[0] = FTS_UPGRADE_55;
	buf[1] = FTS_UPGRADE_AA;
	fts_i2c_write(client, buf, 2);

	msleep(20);

	buf[0] = 0x90;
	buf[1] = buf[2] = buf[3] = 0x00;
	fts_i2c_read(client, buf, 4, reg_val, 2);

	VTD("[UPGRADE] ROM/PRAM/Bootloader id:0x%02x%02x", reg_val[0], reg_val[1]);
	if ((reg_val[0] == 0x00) || (reg_val[0] == 0xFF)) {
		inRomBoot = FTS_RUN_IN_ERROR;
	} else if (reg_val[0] == chip_types.pramboot_idh && reg_val[1] == chip_types.pramboot_idl) {
		inRomBoot = FTS_RUN_IN_PRAM;
	} else if (reg_val[0] == chip_types.rom_idh && reg_val[1] == chip_types.rom_idl) {
		inRomBoot = FTS_RUN_IN_ROM;
	} else if (reg_val[0] == chip_types.bootloader_idh && reg_val[1] == chip_types.bootloader_idl) {
		inRomBoot = FTS_RUN_IN_BOOTLOADER;
	}

	return inRomBoot;
}

/************************************************************************
 * Name: fts_ctpm_get_app_ver
 * Brief:  get app file version
 * Input:
 * Output:
 * Return: fw version
 ***********************************************************************/
int fts_ctpm_get_app_ver(void)
{
	int i_ret = 0;

	if (fts_updatefun_curr.get_app_i_file_ver)
		i_ret = fts_updatefun_curr.get_app_i_file_ver();

	if (i_ret < 0)
		i_ret = 0;

	return i_ret;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade
 * Brief:  fw upgrade entry funciotn
 * Input:
 * Output:
 * Return: 0  - upgrade successfully
 *         < 0 - upgrade failed
 ***********************************************************************/
int fts_ctpm_fw_upgrade(struct i2c_client *client)
{
	int i_ret = 0;

	if (fts_updatefun_curr.upgrade_with_app_i_file)
		i_ret = fts_updatefun_curr.upgrade_with_app_i_file(client);

	return i_ret;
}

/************************************************************************
 * Name: fts_ctpm_fw_upgrade
 * Brief:  fw upgrade entry funciotn
 * Input:
 * Output:
 * Return: 0  - upgrade successfully
 *         < 0 - upgrade failed
 ***********************************************************************/
int fts_ctpm_lcd_cfg_upgrade(struct i2c_client *client)
{
	int i_ret = 0;

	if (fts_updatefun_curr.upgrade_with_lcd_cfg_i_file)
		i_ret = fts_updatefun_curr.upgrade_with_lcd_cfg_i_file(client);

	return i_ret;
}

int fts_ctpm_fw_upgrade_bin_vivo(struct i2c_client *client, u8 *fw_buf, u32 fw_size)
{
	int ret = 0;
    if(g_global.ic_type == _FT8736)
    {
        ret = fts_upgrade_buf_8736(client, fw_buf, fw_size);
    }
    else if(g_global.ic_type == _FT8006)
    {
		ret = fts_upgrade_buf_8006(client, fw_buf, fw_size);
    }
	else if(g_global.ic_type == _FT8719)
	{
		ret = fts_ft8719_upgrade(client,fw_buf, fw_size);
	}
	return ret;
}

int fts_imei_write(u8 *imei)
{
	int ret = 0;
	u8 data[IMEI_MAX_LEN + 1] = { 0 };

	memcpy(data, imei, IMEI_MAX_LEN);
	data[IMEI_MAX_LEN] = '\0';
	ret = fts_write_imei_flash(data, 0, IMEI_MAX_LEN + 1);

	return ret;
}

int fts_imei_read(u8 *imei)
{
	int ret = 0;
	u8 data[IMEI_MAX_LEN + 1] = { 0 };

	ret = fts_read_imei_flash(data, 0, IMEI_MAX_LEN + 1);
	memcpy(imei, data, IMEI_MAX_LEN);

	return ret;
}

int fts_write_imei_flash(u8 *buf, u32 offset, u32 len)
{
	int ret = 0;

    fts_irq_disable();
    if(g_global.ic_type == _FT8736)
    {
        ret = fts_write_imei_flash_8736(buf, offset, len);
    }
    else if(g_global.ic_type == _FT8006)
    {
        ret = fts_write_imei_flash_8006(buf, offset, len);
	}
    else if(g_global.ic_type == _FT8719)
    {
        ret = fts_write_imei_flash_8719(buf, offset, len);
	}
    fts_irq_enable();
	return ret;
}

int fts_read_imei_flash(u8 *buf, u32 offset, u32 len)
{
	int ret = 0;
    fts_irq_disable();
    if(g_global.ic_type == _FT8736)
    {
        ret = fts_read_imei_flash_8736(buf, offset, len);
    }
    else if(g_global.ic_type == _FT8006)
    {
        ret = fts_read_imei_flash_8006(buf, offset, len);
	}
    else if(g_global.ic_type == _FT8719)
    {
        ret = fts_read_imei_flash_8719(buf, offset, len);
	}
    fts_irq_enable();
	return ret;
}





#if (!(FTS_UPGRADE_STRESS_TEST))
/************************************************************************
 * Name: fts_ctpm_check_fw_status
 * Brief: Check App is valid or not
 * Input:
 * Output:
 * Return: -EIO - I2C communication error
 *         FTS_RUN_IN_APP - APP valid
 *         0 - APP invalid
 ***********************************************************************/
int fts_ctpm_check_fw_status(struct i2c_client *client)
{
	u8 chip_id1 = 0;
	u8 chip_id2 = 0;
	int fw_status = FTS_RUN_IN_ERROR;
	int i = 0;
	int ret = 0;
	int i2c_noack_retry = 0;

	for (i = 0; i < 5; i++) {
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &chip_id1);
		if (ret < 0) {
			i2c_noack_retry++;
			continue;
		}
		ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID2, &chip_id2);
		if (ret < 0) {
			i2c_noack_retry++;
			continue;
		}

		if ((chip_id1 == chip_types.chip_idh)
/*#if FTS_CHIP_IDC */
			&& (chip_id2 == chip_types.chip_idl)
/*#endif */
) {
			fw_status = FTS_RUN_IN_APP;
			break;
		}
	}

	VTD("[UPGRADE]: chip_id = %02x%02x, chip_types.chip_idh = %02x%02x",
			  chip_id1, chip_id2, chip_types.chip_idh, chip_types.chip_idl);

	/* I2C No ACK 5 times, then return -EIO */
	if (i2c_noack_retry >= 5)
		return -EIO;

	/* I2C communication ok, but not get correct ID, need check pram / rom / bootloader */
	if (i >= 5) {
		fw_status = fts_ctpm_get_pram_or_rom_id(client);
	}

	return fw_status;
}

/************************************************************************
 * Name: fts_ctpm_check_vendorid_fw
 * Brief: Check vendor id is valid or not
 * Input:
 * Output:
 * Return: 1 - vendor id valid
 *         0 - vendor id invalid
 ***********************************************************************/
static int fts_ctpm_check_vendorid_fw(struct i2c_client *client)
{
#if FTS_GET_VENDOR_ID
	u8 vendor_id;
	fts_i2c_read_reg(client, FTS_REG_VENDOR_ID, &vendor_id);

	VTD("[UPGRADE] tp_vendor_id=%x, FTS_VENDOR_1_ID=%x, FTS_VENDOR_2_ID=%x", vendor_id, FTS_VENDOR_1_ID, FTS_VENDOR_2_ID);
	if ((vendor_id == FTS_VENDOR_1_ID) || (vendor_id == FTS_VENDOR_2_ID))
		return 1;
	else
		return 0;
#else
	return 1;
#endif
}

/************************************************************************
 * Name: fts_ctpm_check_fw_ver
 * Brief: Check vendor id is valid or not
 * Input:
 * Output:
 * Return: 1 - vendor id valid
 *         0 - vendor id invalid
 ***********************************************************************/
static int fts_ctpm_check_fw_ver(struct i2c_client *client)
{
	u8 uc_tp_fm_ver;
	u8 uc_host_fm_ver = 0;

	fts_i2c_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_app_ver();

	VTI("[UPGRADE]: uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x!!", uc_tp_fm_ver, uc_host_fm_ver);
	if (uc_tp_fm_ver != uc_host_fm_ver) {
		return 1;
	} else {
		return 0;
	}
}


bool upgrade_flag[2]; /* [0]:init code  [1]:app */
#if UPGRADE_ALL
#if 0

static int fts_get_flashinit_ver(struct i2c_client *client, u8 *ver)
{
	u8 hdr[4] = { 0 };
	bool inbootloader = false;
	u8 cmd[4] = { 0 };
	u32 addr = 0;

	fts_ctpm_i2c_hid2std(client);
	msleep(10);

	inbootloader = fts_ctpm_check_run_state(client, FTS_RUN_IN_BOOTLOADER);
	if (!inbootloader) {
		FTS_ERROR("[UPGRADE]: not run in bootloader, upgrade fail!!");
		return -EIO;
	}

	addr = 0;
	cmd[0] = 0x03;
	cmd[1] = (u8)((addr >> 16) & 0xFF);
	cmd[2] = (u8)((addr >> 8) & 0xFF);
	cmd[3] = (u8)((addr) & 0xFF);
	fts_i2c_write(client, cmd, 4);
	msleep(10);
	fts_i2c_read(client, NULL, 0, hdr, 4);

	addr = (u32)(((u32)hdr[2]) << 8) + hdr[3];
	cmd[0] = 0x03;
	cmd[1] = (u8)((addr >> 16) & 0xFF);
	cmd[2] = (u8)((addr >> 8) & 0xFF);
	cmd[3] = (u8)((addr) & 0xFF);
	fts_i2c_write(client, cmd, 4);
	msleep(10);
	fts_i2c_read(client, NULL, 0, ver, 1);

	return 0;
}
#endif
static int fts_get_fwinit_ver(u8 *ver)
{
	u8 *fwinit_buf = comp_upgrade_vars[g_global.index].upgrade_fw;
	u32 fwinit_len = 0;
	u8 fwinit_ver[2] = { 0 };
	u32 fwlen = comp_upgrade_vars[g_global.index].fw_length;

	if (fwinit_buf == NULL || fwlen == 0)
		return -EIO;

	if (fwlen < 4096) {
		FTS_ERROR("fw len fail");
		return -EIO;
	}
	fwinit_len = (u32)(((u32)fwinit_buf[2]) << 8) + fwinit_buf[3];
	FTS_DEBUG("init code in host len:%x", fwinit_len);
	if (fwinit_len >= fwlen) {
		FTS_ERROR("fw init code len fail");
		return -EIO;
	}

	fwinit_ver[0] = fwinit_buf[fwinit_len];
	fwinit_ver[1] = fwinit_buf[fwinit_len + 1];

	FTS_DEBUG("init code in host ver:%x %x", fwinit_ver[0], fwinit_ver[1]);
	if (0xFF != (fwinit_ver[0] + fwinit_ver[1])) {
		FTS_ERROR("fw init ver checksum error");
		return -EIO;
	}

	*ver = fwinit_ver[0];
	return 0;
}

static bool fts_lcd_need_upgrade(struct i2c_client *client, bool fw_valid)
{
	int ret = 0;
	u8 tp_init_ver = 0;
	u8 fw_init_ver = 0;

	if (g_global.ic_type != _FT8006)
		return false;

	ret = fts_get_fwinit_ver(&fw_init_ver);
	if (ret < 0) {
		FTS_ERROR("init code in host invalid");
		return false;
	}

	if (VALID == fw_valid) {
		fts_i2c_read_reg(client, 0xE4, &tp_init_ver);
	} else {
		FTS_DEBUG("fw invalid, no upgrade init code");
		/*fts_get_flashinit_ver(client,  &tp_init_ver); */
		return false;
	}
	FTS_DEBUG("tp init ver:%x, fw init ver:%x", tp_init_ver, fw_init_ver);
	if ((0x00 == tp_init_ver) || (0xFF == tp_init_ver)) {
		FTS_DEBUG("tp init code ver not valid, need upgrade init code");
		return true;
	} else if (0xA5 == tp_init_ver) {
		FTS_ERROR("fw init ver == 0xA5, no upgrade");
		return false;
	}

	if (tp_init_ver != fw_init_ver)
		return true;
	else
		return false;
}
#endif
/************************************************************************
 * Name: fts_ctpm_check_need_upgrade
 * Brief:
 * Input:
 * Output:
 * Return: 1 - Need upgrade
 *         0 - No upgrade
 ***********************************************************************/
static int fts_ctpm_check_need_upgrade(struct i2c_client *client)
{
	int fw_status = 0;
	int bUpgradeFlag = false;

	FTS_FUNC_ENTER();
	upgrade_flag[0] = false;
	upgrade_flag[1] = false;

	/* 1. veriry FW APP is valid or not */
	fw_status = fts_ctpm_check_fw_status(client);
	VTI("[UPGRADE]: fw_status = %d!!", fw_status);
	if (fw_status < 0) {
		/* I2C no ACK, return immediately */
		VTE("[UPGRADE]******I2C NO ACK,exit upgrade******");
		return -EIO;
	} else if (fw_status == FTS_RUN_IN_ERROR) {
		VTE("[UPGRADE]******IC Type Fail******");
	} else if (fw_status == FTS_RUN_IN_APP) {
		VTI("[UPGRADE]**********FW APP valid**********");
#if UPGRADE_ALL
		if (fts_lcd_need_upgrade(client, VALID) == true) {
			VTI("need upgrade init code");
			upgrade_flag[0] = true;
		}
#endif
		/* 2. check vendor id is valid or not */
		/*Not used yet */
		if (fts_ctpm_check_vendorid_fw(client) == 0) {
			VTI("[UPGRADE]******Vendor ID in FW is invalid******");
			upgrade_flag[1] = false;
		}

		if (fts_ctpm_check_fw_ver(client) == 1) {
			VTI("[UPGRADE]**********need upgrade fw**********");
			upgrade_flag[1] = true;
		} else {
			VTI("[UPGRADE]**********Don't need upgrade fw**********");
			upgrade_flag[1] = false;
		}
	} else {
		/* if app is invalid, reset to run ROM */
		VTI("[UPGRADE]**********FW APP invalid**********");
#if UPGRADE_ALL
		if (fts_lcd_need_upgrade(client, INVALID) == true) {
			FTS_DEBUG("need upgrade init code");
			upgrade_flag[0] = true;
		}
#endif
		fts_ctpm_rom_or_pram_reset(client);
		upgrade_flag[1] = true;
	}

	bUpgradeFlag = upgrade_flag[0] | upgrade_flag[1];

	FTS_FUNC_EXIT();

	return bUpgradeFlag;
}

/************************************************************************
 * Name: fts_ctpm_auto_upgrade
 * Brief:  auto upgrade
 * Input:
 * Output:
 * Return: 0 - no upgrade
 ***********************************************************************/
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_tp_fm_ver;
	int i_ret = 0;
	int bUpgradeFlag = false;
	u8 uc_upgrade_times = 0;

	VTI("[UPGRADE]********************check upgrade need or not********************");
	bUpgradeFlag = fts_ctpm_check_need_upgrade(client);
	VTD("[UPGRADE]**********bUpgradeFlag = 0x%x**********", bUpgradeFlag);

	if (bUpgradeFlag <= 0) {
		VTI("[UPGRADE]**********No Upgrade, exit**********");
		return 0;
	} else {
		/* FW Upgrade */
		do {
			uc_upgrade_times++;
			VTD("[UPGRADE]********************star upgrade(%d)********************", uc_upgrade_times);

			i_ret = fts_ctpm_fw_upgrade(client);
			if (i_ret >= 0) {
				/* upgrade success */
				fts_i2c_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
				VTI("[UPGRADE]********************Success upgrade to new fw version 0x%x********************", uc_tp_fm_ver);

				fts_ctpm_auto_clb(client);
				break;
			} else {
				/* upgrade fail, reset to run ROM BOOT..
				 * if app in flash is ok, TP will work success
				 */
				VTI("[UPGRADE]********************upgrade fail, reset now********************");
				fts_ctpm_rom_or_pram_reset(client);
			}
		} while (uc_upgrade_times < 2);  /* if upgrade fail, upgrade again. then return */
	}

	return i_ret;
}
#endif

#if FTS_AUTO_UPGRADE_EN
#if 0
static void fts_ctpm_update_work_func(struct work_struct *work)
#endif
static void fts_ctpm_update_work_func(void)
{
	int i_ret = 0;

	VTD("[UPGRADE]******************************FTS enter upgrade******************************");
	fts_irq_disable();

	/* esd check */
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (DISABLE);
#endif

	i_ret = fts_ctpm_auto_upgrade(fts_i2c_client);
	if (i_ret < 0)
		VTE("[UPGRADE]**********TP FW upgrade failed**********");

#if FTS_AUTO_UPGRADE_FOR_LCD_CFG_EN  /*not used when we enable upgrade_all */
	msleep(2000);

	/* lcd_cfg upgrade */
	i_ret = fts_ctpm_lcd_cfg_upgrade(fts_i2c_client);
	if (i_ret < 0)
		VTE("[UPGRADE]**********LCD cfg upgrade failed*********");
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch (ENABLE);
#endif
	fts_irq_enable();

	VTD("[UPGRADE]******************************FTS exit upgrade******************************");
}

/*****************************************************************************
 *  Name: fts_ctpm_upgrade_init
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_ctpm_upgrade_init(void)
{
	int fwSize;

	unsigned char *fw_buffer = NULL;

	FTS_FUNC_ENTER();

	fw_buffer = vivoTsGetFw(VTS_FW_TYPE_FW, &fwSize);

	comp_upgrade_vars[g_global.index].fw_length = fwSize;

	comp_upgrade_vars[g_global.index].upgrade_fw = (unsigned char *)kzalloc(fwSize, GFP_KERNEL);
	if (!comp_upgrade_vars[g_global.index].upgrade_fw) {
		VTE("fail kmalloc upgrade_fw.");
		return;
	}

	memcpy(comp_upgrade_vars[g_global.index].upgrade_fw, fw_buffer, fwSize);

    if (g_global.ic_type == _FT8719) {
		fts_fwupg_work();
    }
	else {
		fts_ctpm_update_work_func();
	}

/*
	touch_wq = create_singlethread_workqueue("touch_wq");
	if (touch_wq) {
		INIT_WORK(&fw_update_work, fts_ctpm_update_work_func);
		queue_work(touch_wq, &fw_update_work);
	} else {
		VTE("[UPGRADE]create_singlethread_workqueue failed\n");
	}
 */
	kfree(comp_upgrade_vars[g_global.index].upgrade_fw);
	comp_upgrade_vars[g_global.index].upgrade_fw = NULL;

	FTS_FUNC_EXIT();
}


/*****************************************************************************
 *  Name: fts_ctpm_upgrade_exit
 *  Brief:
 *  Input:
 *  Output:
 *  Return:
 *****************************************************************************/
void fts_ctpm_upgrade_exit(void)
{
	FTS_FUNC_ENTER();
	destroy_workqueue(touch_wq);
	FTS_FUNC_EXIT();
}

#endif  /* #if FTS_AUTO_UPGRADE_EN */
