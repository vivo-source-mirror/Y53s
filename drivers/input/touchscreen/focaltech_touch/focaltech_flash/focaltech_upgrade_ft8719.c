/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_upgrade_ft8719.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-11-22
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_flash_main.h"
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
u8 pb_file_ft8719[] = {
#include "../include/pramboot/FT8719_Pramboot_V0.1_20171102.i"
};

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/************************************************************************
 * fts_ft8719_upgrade_mode -
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_ft8719_upgrade_mode(
    struct i2c_client *client,
    enum FW_FLASH_MODE mode,
    u8 *buf,
    u32 len)
{
    int ret = 0;
    u32 start_addr = 0;
    u8 cmd[4] = { 0 };
    u32 delay = 0;
    int ecc_in_host = 0;
    int ecc_in_tp = 0;

    if ((NULL == buf) || (len < FTS_MIN_LEN)) {
        FTS_ERROR("buffer/len(%x) is invalid", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot(client);
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_UPGRADE_VALUE;
    start_addr = upgrade_func_ft8719.appoff;
    if (FLASH_MODE_LIC == mode) {
        /* lcd initial code upgrade */
        /* not finish 3-gamma yet   */
        cmd[1] = FLASH_MODE_LIC_VALUE;
        start_addr = upgrade_func_ft8719.licoff;
    }
    FTS_INFO("flash mode:0x%02x, start addr=0x%04x", cmd[1], start_addr);

    ret = fts_i2c_write(client, cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto fw_reset;
    }

    delay = FTS_ERASE_SECTOR_DELAY * (len / FTS_MAX_LEN_SECTOR);
    ret = fts_fwupg_erase(client, delay);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto fw_reset;
    }

    /* write app */
    ecc_in_host = fts_flash_write_buf(client, start_addr, buf, len, 1);
    if (ecc_in_host < 0 ) {
        FTS_ERROR("lcd initial code write fail");
        goto fw_reset;
    }

    /* ecc */
    ecc_in_tp = fts_fwupg_ecc_cal(client, start_addr, len);
    if (ecc_in_tp < 0 ) {
        FTS_ERROR("ecc read fail");
        goto fw_reset;
    }

    FTS_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        goto fw_reset;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot(client);
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(400);
    return 0;

fw_reset:
    return -EIO;
}

/************************************************************************
* Name: fts_ft8719_upgrade
* Brief:
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
int fts_ft8719_upgrade(struct i2c_client *client, u8 *buf, u32 len)
{
    int ret = 0;
    u8 *tmpbuf = NULL;
    u32 app_len = 0;

    FTS_INFO("fw app upgrade...");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_FILE)) {
        FTS_ERROR("fw buffer len(%x) fail", len);
        return -EINVAL;
    }

    app_len = len - upgrade_func_ft8719.appoff;
    tmpbuf = buf + upgrade_func_ft8719.appoff;
    ret = fts_ft8719_upgrade_mode(client, FLASH_MODE_APP, tmpbuf, app_len);
    if (ret < 0) {
        FTS_INFO("fw upgrade fail,reset to normal boot");
        if (fts_fwupg_reset_in_boot(client) < 0) {
            FTS_ERROR("reset to normal boot fail");
        }
        return ret;
    }

    return 0;
}
#define IMEI_SERCOTR_8719    30
#define IMEI_MAX_LEN_8719             100
int fts_write_imei_flash_8719(u8 *buf, u32 offset, u32 len)
{
    int ret = 0;
    u32 i = 0;
    u8 cmd[IMEI_MAX_LEN_8719+6] = { 0 };
    u32 addr = 0;
    struct i2c_client *client = fts_i2c_client;

    if( len >= IMEI_MAX_LEN_8719)
    {
        FTS_ERROR("write length error");
        return -EINVAL;
    }
    ret = fts_fwupg_enter_into_boot(client);
    if(ret <0)
    {
        FTS_ERROR("enter pramboot fail");
        goto boot_reset;
    }

    cmd[0] = 0x09;
    cmd[1] = 0x80 + IMEI_SERCOTR_8719;
    cmd[2] = 0x01;
    fts_i2c_write(client, cmd, 3);

    fts_fwupg_erase(client, 30);

    addr = IMEI_SERCOTR_8719 * 4096;
    cmd[0] = 0xbf;
    cmd[1] = (u8)((addr >> 16) & 0xFF);
    cmd[2] = (u8)((addr >> 8) & 0xFF);
    cmd[3] = (u8)(addr & 0xFF);
    cmd[4] = (u8)((len >> 8) & 0xFF);
    cmd[5] = (u8)(len & 0xFF);
    for(i = 0; i < len; i++)
    {
        cmd[6+i] = buf[i];
    }
    ret = fts_i2c_write(client, cmd, len + 6);
    msleep(10);

    memset(cmd, 0, IMEI_MAX_LEN_8719);
    cmd[0] = 0x03;
    cmd[1] = (u8)((addr >> 16) & 0xFF);
    cmd[2] = (u8)((addr >> 8) & 0xFF);
    cmd[3] = (u8)(addr & 0xFF);
    ret = fts_i2c_write(client, cmd, 4);
    msleep(2);
    ret = fts_i2c_read(client, NULL, 0, cmd + 4, len);

    FTS_DEBUG("write:%s read:%s len:%d", buf, cmd+4, len);
    if(memcmp(buf, cmd+4, len) != 0)
    {
        FTS_ERROR("write fail,write:%s read:%s len:%d", buf, cmd+4, len);
        goto boot_reset;
    }

    FTS_DEBUG("write imei ok");
boot_reset:
    fts_fwupg_reset_in_boot(client);
    return ret;
}
int fts_read_imei_flash_8719(u8 *buf, u32 offset, u32 len)
{
    int ret = 0;
    u8 cmd[4] = { 0 };
    u32 addr = 0;
    struct i2c_client *client = fts_i2c_client;

    if( len >= IMEI_MAX_LEN_8719)
    {
        FTS_ERROR("write length error");
        return -EINVAL;
    }

    ret = fts_fwupg_enter_into_boot(client);
    if(ret <0)
    {
        FTS_ERROR("enter pramboot fail");
        goto boot_reset;
    }

    addr = IMEI_SERCOTR_8719 * 4096;
    cmd[0] = 0x03;
    cmd[1] = (u8)((addr >> 16) & 0xFF);
    cmd[2] = (u8)((addr >> 8) & 0xFF);
    cmd[3] = (u8)(addr & 0xFF);
    ret = fts_i2c_write(client, cmd, 4);
    msleep(2);
    ret = fts_i2c_read(client, NULL, 0, buf, len);

    FTS_DEBUG("read imei ok");
boot_reset:
    fts_fwupg_reset_in_boot(client);

    return ret;
}


struct upgrade_func upgrade_func_ft8719 = {
    .ctype = {0x0F},
    .fwveroff = 0x210E,
    .fwcfgoff = 0x1F80,
    .appoff = 0x2000,
    .licoff = 0x0000,
    .pramboot_supported = true,
    .pramboot = pb_file_ft8719,
    .pb_length = sizeof(pb_file_ft8719),
    .hid_supported = false,
    .upgrade = fts_ft8719_upgrade,
};

