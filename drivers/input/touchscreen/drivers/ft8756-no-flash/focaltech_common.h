/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2018, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_common.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-16
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_COMMON_H__
#define __LINUX_FOCALTECH_COMMON_H__

#include "focaltech_config.h"

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_VERSION                  "Focaltech V1.0 20180110"

#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)((x >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)((x >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)((x >> 24) & 0xFF)
#define FLAGBIT(x)              (0x00000001 << (x))
#define FLAGBITS(x, y)          ((0xFFFFFFFF >> (32 - (y) - 1)) << (x))

#define FLAG_ICSERIALS_LEN      8
#define FLAG_IDC_BIT            11

#define IC_SERIALS              (FTS_CHIP_TYPE & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define IC_TO_SERIALS(x)        ((x) & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define FTS_HID_SUPPORTTED      ((FTS_CHIP_TYPE & FLAGBIT(FLAG_HID_BIT)) == FLAGBIT(FLAG_HID_BIT))


//#define READ_WRITE_BUFFER_SIZE              256
#define FILE_NAME_LENGTH                    128
#define SPI_MAX_COMMAND_LENGTH              16
//#define SPI_HEADER_LENGTH                   4 /* must >= 4 */
//#define SPI_MAX_PACKAGE_LENGTH              (4 * 1024)
//#define SPI_BUF_LENGTH_FW                   256   /* must < 512 for fw spi buf */
#define SPI_BUF_LENGTH                      256
#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_READ_ID                     0x90
/*register address*/
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_FLOW_WORK_CNT               0x91
#define FTS_REG_WORKMODE                    0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      0x40
#define FTS_REG_WORKMODE_WORK_VALUE         0x00
#define FTS_REG_ESDCHECK_DISABLE            0x8D
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_CHIP_ID2                    0x9F
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP_VALUE      0x03
#define FTS_REG_FW_VER                      0xA6
#define FTS_REG_VENDOR_ID                   0xA8
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_FACE_DEC_MODE_EN            0xB0
#define FTS_REG_FACE_DEC_MODE_STATUS        0x01
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_GLOVE_MODE_EN               0xC0
#define FTS_REG_COVER_MODE_EN               0xC1
#define FTS_REG_CHARGER_MODE_EN             0x8B
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4
#define FTS_REG_ESD_SATURATE                0xED
#define FTS_REG_FW_TYPE                     0xB4
#define FTS_DBUG_BUF_MAX_LEN                4096
#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

#define FTS_NORMAL_MODE						0
#define FTS_FACTORY_MODE					1

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct ft_chip_t {
    u64 type;
    u8 chip_idh;
    u8 chip_idl;
    u8 rom_idh;
    u8 rom_idl;
    u8 pb_idh;
    u8 pb_idl;
    u8 bl_idh;
    u8 bl_idl;
};

struct ts_ic_info {
    bool is_incell;
    bool hid_supported;
    struct ft_chip_t *ids;
    char *ic_name;
    u64 fw_max_len;//FTS_MAX_LEN_APP
    u8 rst_delay_ms;
    u8 pram_size_k;//pram_app_size = pram_size_k*code_len
    u8 dram_support;
};

#define FT8756_NAME   "ft8756"
#define FT8016_NAME   "ft8016"
#define FT8720_NAME   "ft8720"

static struct ft_chip_t FT8016_chip_info = {
    .type     = 0x1D,
    .chip_idh = 0x80,
    .chip_idl = 0x16,
    .rom_idh  = 0x86,
    .rom_idl  = 0x32,
    .pb_idh   = 0x86,
    .pb_idl   = 0xC2,
    .bl_idh   = 0x00,
    .bl_idl   = 0x00,
};
static struct ft_chip_t FT8756_chip_info = {
    .type     = 0x15,
    .chip_idh = 0x87,
    .chip_idl = 0x56,
    .rom_idh  = 0x87,
    .rom_idl  = 0x56,
    .pb_idh   = 0xF7,
    .pb_idl   = 0xA6,
    .bl_idh   = 0x00,
    .bl_idl   = 0x00,
};
static struct ft_chip_t FT8720_chip_info = {
    .type     = 0x1C,
    .chip_idh = 0x87,
    .chip_idl = 0x20,
    .rom_idh  = 0x87,
    .rom_idl  = 0x20,
    .pb_idh   = 0x87,
    .pb_idl   = 0xA0,
    .bl_idh   = 0x00,
    .bl_idl   = 0x00,
};
static const struct ts_ic_info fts_ic_info_map[] = {
     {.is_incell     = 0, 
      .hid_supported = 0,
      .ids           = &FT8016_chip_info, 
      .ic_name       = FT8016_NAME,
      .fw_max_len    = 64*1024,
      .rst_delay_ms  = 12,
      .pram_size_k   = 1,
      .dram_support  = 0,
     },
     {.is_incell     = 0, 
      .hid_supported = 0,
      .ids           = &FT8756_chip_info, 
      .ic_name       = FT8756_NAME,
      .fw_max_len    = 88*1024,
      .rst_delay_ms  = 8,
      .pram_size_k   = 2,
      .dram_support  = 1,
     },
     {.is_incell     = 0,
      .hid_supported = 0,
      .ids           = &FT8720_chip_info,
      .ic_name       = FT8720_NAME,
      .fw_max_len    = 88*1024,
      .rst_delay_ms  = 8,
      .pram_size_k   = 2,
      .dram_support  = 1,
     }
};

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/
#if FTS_DEBUG_EN
#define FTS_DEBUG_LEVEL     1
#if (FTS_DEBUG_LEVEL == 2)
#define FTS_DEBUG(fmt, args...) printk("[VIVO_TS][%s]"fmt"\n", __func__, ##args)
#else
#define FTS_DEBUG(fmt, args...) printk("[VIVO_TS]"fmt"\n", ##args)
#endif
#define FTS_FUNC_ENTER() printk("VIVO_TS_INF^%d^[FTS]%s: Enter\n", __LINE__, __func__)
#define FTS_FUNC_EXIT()  printk("VIVO_TS_INF^%d^[FTS]%s: Exit\n", __LINE__, __func__)
#else /* #if FTS_DEBUG_EN*/
#define FTS_DEBUG(fmt, args...)
#define FTS_FUNC_ENTER()
#define FTS_FUNC_EXIT()
#endif

#define FTS_INFO(fmt, args...) printk(KERN_INFO "[VIVO_TS][Info]"fmt"\n", ##args)
#define FTS_ERROR(fmt, args...) printk(KERN_ERR "[VIVO_TS][Error]"fmt"\n", ##args)

#endif /* __LINUX_FOCALTECH_COMMON_H__ */
