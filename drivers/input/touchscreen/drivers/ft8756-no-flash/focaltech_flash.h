/************************************************************************
* Copyright (C) 2012-2018, Focaltech Systems (R)��All Rights Reserved.
*
* File Name: focaltech_flash.h
*
*    Author: Driver Team
*
*   Created: 2017-12-06
*
*  Abstract:
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_FLASH_H__
#define __LINUX_FOCALTECH_FLASH_H__

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_CMD_RESET                               0x07
#define FTS_ROMBOOT_CMD_WRITE                       0xAE
#define FTS_CMD_WRITE_LEN                           6
#define FTS_ROMBOOT_CMD_START_APP                   0x08
#define FTS_DELAY_PRAMBOOT_START                    10
#define FTS_ROMBOOT_CMD_ECC                         0xCC
#define FTS_ROMBOOT_CMD_ECC_LEN                     7
#define FTS_ROMBOOT_CMD_ECC_FINISH                  0xCE
#define FTS_ECC_FINISH_TIMEOUT                      100
#define FTS_ROMBOOT_CMD_ECC_READ                    0xCD

#define AL2_FCS_COEF                ((1 << 15) + (1 << 10) + (1 << 3))
#define FTS_READ_BOOT_ID_TIMEOUT                    3

#define FTS_FLASH_PACKET_LENGTH                     (4 * 1024 - 4)
#define FTS_MIN_LEN                                 0x120
#define FTS_MAX_LEN_APP                             (88 * 1024)
#define FTS_MAX_LEN_SECTOR                          (4 * 1024)
#define FTS_REG_UPGRADE                             0xFC
#define FTS_UPGRADE_AA                              0xAA
#define FTS_UPGRADE_55                              0x55
#define FTS_DELAY_FC_AA                             10
#define FTS_UPGRADE_LOOP                            30
#define FTS_FW_BIN_FILEPATH                         "/sdcard/"

#define FTS_APP_INFO_OFFSET                         0x100
#define FTS_FW_NAME									"focaltech_ts_fw.bin"

#define FTS_PRAM_SADDR                              0x000000
#define FTS_DRAM_SADDR                              0xD00000
#define FTS_FLASH_PACKET_LENGTH_SPI                 (32 * 1024 - 16)
#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR               0xAD
#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN           4
#define FTS_CMD_ECC_LENGTH_MAX                      32766
#define FTS_ROMBOOT_CMD_ECC_FINISH_OK               0xA5
#define FTS_MAX_LEN_APP_PARAMS                      (32 * 1024)
#endif /* __LINUX_FOCALTECH_FLASH_H__ */