/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision: 12034 $
 * $Date: 2017 -05 -04 17:49:58 + 0800 (Thu, 04 May 2017) $
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include "../vts_core.h"

#include "nt36xxx.h"

#if BOOT_UPDATE_FIRMWARE

#define FW_BIN_SIZE_116KB 118784
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET 0x1A000
#define FW_BIN_VER_BAR_OFFSET 0x1A001
#define FLASH_SECTOR_SIZE 4096
#define SIZE_64KB 65536
#define BLOCK_64KB_NUM 4

const struct firmware *nt36670_fw_entry;

static struct firmware nt_fw;
/*******************************************************
Description:
	Novatek touchscreen request update firmware function.

return:
	Executive outcomes. 0-- - succeed. -1, -22-- - failed.
 *******************************************************/
int32_t nt36670_update_firmware_request(struct vts_device *vtsdev, char *filename)
{
	int32_t ret = 0;
	int size = 0;

if (0) {
	if (NULL == filename) {
		ret = -1;
		return ret;
	}
}
	NVT_LOG("filename is %s\n", filename);

	/*ret = request_firmware(&nt36670_fw_entry, filename, &nt36670_ts->client->dev); */
	/*if (ret) { */
	/*	NVT_ERR("firmware load failed, ret=%d\n", ret); */
	/*	return ret; */
	/*} */
	nt_fw.data = NULL;
	nt_fw.size = 0;

	nt_fw.data = (u8 *)vts_fw_data_get(vtsdev, VTS_FW_TYPE_FW, &(size));
	if (nt_fw.data == NULL) {
		vts_dev_info(nt36670_ts->vtsdev, "get fw fail");
		ret = -1;
		return ret;
	}
	nt_fw.size = (size_t)size;

	nt36670_fw_entry = &nt_fw;

	/* check bin file size (116kb) */
	if (nt36670_fw_entry->size != FW_BIN_SIZE) {
		NVT_ERR("bin file size not match. (%zu)\n", nt36670_fw_entry->size);
		return -EINVAL;
	}

	/* check if FW version add FW version bar equals 0xFF */
	if (*(nt36670_fw_entry->data + FW_BIN_VER_OFFSET) + *(nt36670_fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
		vts_dev_err(nt36670_ts->vtsdev, "bin file FW_VER + FW_VER_BAR should be 0xFF!");
		vts_dev_err(nt36670_ts->vtsdev, "FW_VER=0x%02X, FW_VER_BAR=0x%02X", *(nt36670_fw_entry->data + FW_BIN_VER_OFFSET), *(nt36670_fw_entry->data + FW_BIN_VER_BAR_OFFSET));
		return -EINVAL;
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen release update firmware function.

return:
	n.a.
 *******************************************************/
void nt36670_update_firmware_release(void)
{
	if (nt36670_fw_entry) {
		release_firmware(nt36670_fw_entry);
	}
	nt36670_fw_entry = NULL;
}

/*******************************************************
Description:
	Novatek touchscreen check firmware version function.

return:
	Executive outcomes. 0-- - need update. 1-- - need not
	update.
 *******************************************************/
static uint8_t ic_fw_version;
int32_t nt36670_Check_FW_Ver(void)
{
	uint8_t buf[16] = {0};
	int32_t ret = 0;

	/*write i2c index to EVENT BUF ADDR */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("i2c write error!(%d)\n", ret);
		return ret;
	}

	/*read Firmware Version */
	buf[0] = EVENT_MAP_FWINFO;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("i2c read error!(%d)\n", ret);
		return ret;
	}

	NVT_LOG("IC FW Ver = 0x%02X, FW Ver Bar = 0x%02X\n", buf[1], buf[2]);
	NVT_LOG("Bin FW Ver = 0x%02X, FW ver Bar = 0x%02X\n",
			nt36670_fw_entry->data[FW_BIN_VER_OFFSET], nt36670_fw_entry->data[FW_BIN_VER_BAR_OFFSET]);

	/* check IC FW_VER + FW_VER_BAR equals 0xFF or not, need to update if not */
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("IC FW_VER + FW_VER_BAR not equals to 0xFF!\n");
		return 0;
	}

	/* compare IC and binary FW version */
	if (buf[1] != nt36670_fw_entry->data[FW_BIN_VER_OFFSET])
		return 0;
	else
		return 1;
}

int nt36670_get_chip_FW_Ver(void)
{
	uint8_t buf[16] = {0};
	int32_t ret = 0;

	/*write i2c index to EVENT BUF ADDR */
	buf[0] = 0xFF;
	buf[1] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (nt36670_ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		vts_dev_err(nt36670_ts->vtsdev, "i2c write error!(%d)", ret);
		return ret;
	}

	/*read Firmware Version */
	buf[0] = EVENT_MAP_FWINFO;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		vts_dev_err(nt36670_ts->vtsdev, "i2c read error!(%d)", ret);
		return ret;
	}

	vts_dev_info(nt36670_ts->vtsdev, "IC FW Ver = 0x%02X, FW Ver Bar = 0x%02X", buf[1], buf[2]);

	/* compare IC and binary FW version */
	return (int)buf[1];

}


int nt36670_get_ic_fw_version(void)
{
	vts_dev_info(nt36670_ts->vtsdev, "ifw version:%d", ic_fw_version);
	return ic_fw_version;
}

/*******************************************************
Description:
	Novatek touchscreen resume from deep power down function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Resume_PD(void)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	/* Resume Command */
	buf[0] = 0x00;
	buf[1] = 0xAB;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("Write Enable error!!(%d)\n", ret);
		return ret;
	}

	/* Check 0xAA (Resume Command) */
	retry = 0;
	while (1) {
		msleep(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Resume Command) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Resume Command) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}
	msleep(10);

	NVT_LOG("Resume PD OK\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen check firmware checksum function.

return:
	Executive outcomes. 0-- - checksum not match.
	1-- - checksum match. -1-- - checksum read failed.
 *******************************************************/
int32_t nt36670_Check_CheckSum(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = nt36670_ts->mmap->READ_FLASH_CHECKSUM_ADDR;
	int32_t ret = 0;
	int32_t i = 0;
	int32_t k = 0;
	uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
	uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
	size_t fw_bin_size = 0;
	size_t len_in_blk = 0;
	int32_t retry = 0;

	if (nt36670_Resume_PD()) {
		NVT_ERR("Resume PD error!!\n");
		ret = -1;
		return ret;
	}

	fw_bin_size = nt36670_fw_entry->size;

	for (i = 0; i < BLOCK_64KB_NUM; i++) {
		if (fw_bin_size > (i * SIZE_64KB)) {
			/* Calculate WR_Filechksum of each 64KB block */
			len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
			WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
			for (k = 0; k < len_in_blk; k++) {
				WR_Filechksum[i] += nt36670_fw_entry->data[k + i * SIZE_64KB];
			}
			WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

			/* Fast Read Command */
			buf[0] = 0x00;
			buf[1] = 0x07;
			buf[2] = i;
			buf[3] = 0x00;
			buf[4] = 0x00;
			buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
			buf[6] = (len_in_blk - 1) & 0xFF;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 7);
			if (ret < 0) {
				NVT_ERR("Fast Read Command error!!(%d)\n", ret);
				return ret;
			}
			/* Check 0xAA (Fast Read Command) */
			retry = 0;
			while (1) {
				msleep(80);
				buf[0] = 0x00;
				buf[1] = 0x00;
				ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
				if (ret < 0) {
					NVT_ERR("Check 0xAA (Fast Read Command) error!!(%d)\n", ret);
					return ret;
				}
				if (buf[1] == 0xAA) {
					break;
				}
				retry++;
				if (unlikely(retry > 5)) {
					NVT_ERR("Check 0xAA (Fast Read Command) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
					ret = -1;
					return ret;
				}
			}
			/* Read Checksum (write addr high byte & middle byte) */
			buf[0] = 0xFF;
			buf[1] = XDATA_Addr >> 16;
			buf[2] = (XDATA_Addr >> 8) & 0xFF;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Read Checksum (write addr high byte & middle byte) error!!(%d)\n", ret);
				return ret;
			}
			/* Read Checksum */
			buf[0] = (XDATA_Addr) & 0xFF;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Read Checksum error!!(%d)\n", ret);
				return ret;
			}

			RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
			if (WR_Filechksum[i] != RD_Filechksum[i]) {
				NVT_ERR("RD_Filechksum[%d]=0x%04X, WR_Filechksum[%d]=0x%04X\n", i, RD_Filechksum[i], i, WR_Filechksum[i]);
				NVT_ERR("firmware checksum not match!!\n");
				return 0;
			}
		}
	}

	NVT_LOG("firmware checksum match\n");
	return 1;
}

/*******************************************************
Description:
	Novatek touchscreen initial bootloader and flash
	block function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Init_BootLoader(void)
{
	uint8_t buf[64] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	/* SW Reset & Idle */
	nt36670_sw_reset_idle();

	/* Initiate Flash Block */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = I2C_FW_Address;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("Inittial Flash Block error!!(%d)\n", ret);
		return ret;
	}

	/* Check 0xAA (Initiate Flash Block) */
	retry = 0;
	while (1) {
		msleep(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Inittial Flash Block) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Inittial Flash Block) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}

	NVT_LOG("Init OK \n");
	msleep(20);

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen erase flash sectors function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Erase_Flash(void)
{
	uint8_t buf[64] = {0};
	int32_t ret = 0;
	int32_t count = 0;
	int32_t i = 0;
	int32_t Flash_Address = 0;
	int32_t retry = 0;

	/* Write Enable */
	buf[0] = 0x00;
	buf[1] = 0x06;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0) {
		NVT_ERR("Write Enable (for Write Status Register) error!!(%d)\n", ret);
		return ret;
	}
	/* Check 0xAA (Write Enable) */
	retry = 0;
	while (1) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Write Enable for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Write Enable for Write Status Register) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}

	/* Write Status Register */
	buf[0] = 0x00;
	buf[1] = 0x01;
	buf[2] = 0x00;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("Write Status Register error!!(%d)\n", ret);
		return ret;
	}
	/* Check 0xAA (Write Status Register) */
	retry = 0;
	while (1) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if (buf[1] == 0xAA) {
			break;
		}
		retry++;
		if (unlikely(retry > 20)) {
			NVT_ERR("Check 0xAA (Write Status Register) error!! status=0x%02X\n", buf[1]);
			ret = -1;
			return ret;
		}
	}

	/* Read Status */
	retry = 0;
	while (1) {
		mdelay(5);
		buf[0] = 0x00;
		buf[1] = 0x05;
		ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Read Status (for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}

		/* Check 0xAA (Read Status) */
		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 3);
		if (ret < 0) {
			NVT_ERR("Check 0xAA (Read Status for Write Status Register) error!!(%d)\n", ret);
			return ret;
		}
		if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
			break;
		}
		retry++;
		if (unlikely(retry > 100)) {
			NVT_ERR("Check 0xAA (Read Status for Write Status Register) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
			ret = -1;
			return ret;
		}
	}

	if (nt36670_fw_entry->size % FLASH_SECTOR_SIZE)
		count = nt36670_fw_entry->size / FLASH_SECTOR_SIZE + 1;
	else
		count = nt36670_fw_entry->size / FLASH_SECTOR_SIZE;

	for (i = 0; i < count; i++) {
		/* Write Enable */
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write Enable error!!(%d,%d)\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Write Enable) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Write Enable) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
				ret = -1;
				return ret;
			}
		}

		Flash_Address = i * FLASH_SECTOR_SIZE;

		/* Sector Erase */
		buf[0] = 0x00;
		buf[1] = 0x20;    /* Command : Sector Erase */
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 5);
		if (ret < 0) {
			NVT_ERR("Sector Erase error!!(%d,%d)\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Sector Erase) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Sector Erase) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Sector Erase) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
				ret = -1;
				return ret;
			}
		}

		/* Read Status */
		retry = 0;
		while (1) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Read Status error!!(%d,%d)\n", ret, i);
				return ret;
			}

			/* Check 0xAA (Read Status) */
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Read Status) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if ((buf[1] == 0xAA) && (buf[2] == 0x00)) {
				break;
			}
			retry++;
			if (unlikely(retry > 100)) {
				NVT_ERR("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
				ret = -1;
				return ret;
			}
		}
	}

	NVT_LOG("Erase OK \n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen write flash sectors function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Write_Flash(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = nt36670_ts->mmap->RW_FLASH_DATA_ADDR;
	uint32_t Flash_Address = 0;
	int32_t i = 0, j = 0, k = 0;
	uint8_t tmpvalue = 0;
	int32_t count = 0;
	int32_t ret = 0;
	int32_t retry = 0;

	/* change I2C buffer index */
	buf[0] = 0xFF;
	buf[1] = XDATA_Addr >> 16;
	buf[2] = (XDATA_Addr >> 8) & 0xFF;
	ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0) {
		NVT_ERR("change I2C buffer index error!!(%d)\n", ret);
		return ret;
	}

	if (nt36670_fw_entry->size % 256)
		count = nt36670_fw_entry->size / 256 + 1;
	else
		count = nt36670_fw_entry->size / 256;

	for (i = 0; i < count; i++) {
		Flash_Address = i * 256;

		/* Write Enable */
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0) {
			NVT_ERR("Write Enable error!!(%d)\n", ret);
			return ret;
		}
		/* Check 0xAA (Write Enable) */
		retry = 0;
		while (1) {
			udelay(100);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Write Enable) error!!(%d,%d)\n", ret, i);
				return ret;
			}
			if (buf[1] == 0xAA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Write Enable) error!! status=0x%02X\n", buf[1]);
				ret = -1;
				return ret;
			}
		}

		/* Write Page : 256 bytes */
		for (j = 0; j < min(nt36670_fw_entry->size - i * 256, (size_t)256); j += 32) {
			buf[0] = (XDATA_Addr + j) & 0xFF;
			for (k = 0; k < 32; k++) {
				buf[1 + k] = nt36670_fw_entry->data[Flash_Address + j + k];
			}
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 33);
			if (ret < 0) {
				NVT_ERR("Write Page error!!(%d), j=%d\n", ret, j);
				return ret;
			}
		}
		if (nt36670_fw_entry->size - Flash_Address >= 256)
			tmpvalue = (Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (255);
		else
			tmpvalue = (Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (nt36670_fw_entry->size - Flash_Address - 1);

		for (k = 0; k < min(nt36670_fw_entry->size - Flash_Address, (size_t)256); k++)
			tmpvalue += nt36670_fw_entry->data[Flash_Address + k];

		tmpvalue = 255 - tmpvalue + 1;

		/* Page Program */
		buf[0] = 0x00;
		buf[1] = 0x02;
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		buf[5] = 0x00;
		buf[6] = min(nt36670_fw_entry->size - Flash_Address, (size_t)256) - 1;
		buf[7] = tmpvalue;
		ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 8);
		if (ret < 0) {
			NVT_ERR("Page Program error!!(%d), i=%d\n", ret, i);
			return ret;
		}
		/* Check 0xAA (Page Program) */
		retry = 0;
		while (1) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Page Program error!!(%d)\n", ret);
				return ret;
			}
			if (buf[1] == 0xAA || buf[1] == 0xEA) {
				break;
			}
			retry++;
			if (unlikely(retry > 20)) {
				NVT_ERR("Check 0xAA (Page Program) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
				ret = -1;
				return ret;
			}
		}
		if (buf[1] == 0xEA) {
			NVT_ERR("Page Program error!! i=%d\n", i);
			ret = -3;
			return ret;
		}

		/* Read Status */
		retry = 0;
		while (1) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0) {
				NVT_ERR("Read Status error!!(%d)\n", ret);
				return ret;
			}

			/* Check 0xAA (Read Status) */
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Check 0xAA (Read Status) error!!(%d)\n", ret);
				return ret;
			}
			if (((buf[1] == 0xAA) && (buf[2] == 0x00)) || (buf[1] == 0xEA)) {
				break;
			}
			retry++;
			if (unlikely(retry > 100)) {
				NVT_ERR("Check 0xAA (Read Status) failed, buf[1]=0x%02X, buf[2]=0x%02X, retry=%d\n", buf[1], buf[2], retry);
				ret = -1;
				return ret;
			}
		}
		if (buf[1] == 0xEA) {
			NVT_ERR("Page Program error!! i=%d\n", i);
			ret = -4;
			return ret;
		}

		NVT_LOG("Programming...%2d%%\r", ((i * 100) / count));
	}

	NVT_LOG("Programming...%2d%%\r", 100);
	NVT_LOG("Program OK         \n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen verify checksum of written
	flash function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Verify_Flash(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = nt36670_ts->mmap->READ_FLASH_CHECKSUM_ADDR;
	int32_t ret = 0;
	int32_t i = 0;
	int32_t k = 0;
	uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
	uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
	size_t fw_bin_size = 0;
	size_t len_in_blk = 0;
	int32_t retry = 0;

	fw_bin_size = nt36670_fw_entry->size;

	for (i = 0; i < BLOCK_64KB_NUM; i++) {
		if (fw_bin_size > (i * SIZE_64KB)) {
			/* Calculate WR_Filechksum of each 64KB block */
			len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
			WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
			for (k = 0; k < len_in_blk; k++) {
				WR_Filechksum[i] += nt36670_fw_entry->data[k + i * SIZE_64KB];
			}
			WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

			/* Fast Read Command */
			buf[0] = 0x00;
			buf[1] = 0x07;
			buf[2] = i;
			buf[3] = 0x00;
			buf[4] = 0x00;
			buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
			buf[6] = (len_in_blk - 1) & 0xFF;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_HW_Address, buf, 7);
			if (ret < 0) {
				NVT_ERR("Fast Read Command error!!(%d)\n", ret);
				return ret;
			}
			/* Check 0xAA (Fast Read Command) */
			retry = 0;
			while (1) {
				msleep(80);
				buf[0] = 0x00;
				buf[1] = 0x00;
				ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_HW_Address, buf, 2);
				if (ret < 0) {
					NVT_ERR("Check 0xAA (Fast Read Command) error!!(%d)\n", ret);
					return ret;
				}
				if (buf[1] == 0xAA) {
					break;
				}
				retry++;
				if (unlikely(retry > 5)) {
					NVT_ERR("Check 0xAA (Fast Read Command) failed, buf[1]=0x%02X, retry=%d\n", buf[1], retry);
					ret = -1;
					return ret;
				}
			}
			/* Read Checksum (write addr high byte & middle byte) */
			buf[0] = 0xFF;
			buf[1] = XDATA_Addr >> 16;
			buf[2] = (XDATA_Addr >> 8) & 0xFF;
			ret = NT36670_CTP_I2C_WRITE(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Read Checksum (write addr high byte & middle byte) error!!(%d)\n", ret);
				return ret;
			}
			/* Read Checksum */
			buf[0] = (XDATA_Addr) & 0xFF;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = NT36670_CTP_I2C_READ(nt36670_ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0) {
				NVT_ERR("Read Checksum error!!(%d)\n", ret);
				return ret;
			}

			RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
			if (WR_Filechksum[i] != RD_Filechksum[i]) {
				NVT_ERR("Verify Fail%d!!\n", i);
				NVT_ERR("RD_Filechksum[%d]=0x%04X, WR_Filechksum[%d]=0x%04X\n", i, RD_Filechksum[i], i, WR_Filechksum[i]);
				ret = -1;
				return ret;
			}
		}
	}

	NVT_LOG("Verify OK \n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware function.

return:
	Executive outcomes. 0-- - succeed. negative-- - failed.
 *******************************************************/
int32_t nt36670_Update_Firmware(void)
{
	int32_t ret = 0;

	/*-- - Stop CRC check to prevent IC auto reboot-- - */
	nt36670_stop_crc_reboot();

	/* Step 1 : initial bootloader */
	ret = nt36670_Init_BootLoader();
	if (ret) {
		return ret;
	}

	/* Step 2 : Resume PD */
	ret = nt36670_Resume_PD();
	if (ret) {
		return ret;
	}

	/* Step 3 : Erase */
	ret = nt36670_Erase_Flash();
	if (ret) {
		return ret;
	}

	/* Step 4 : Program */
	ret = nt36670_Write_Flash();
	if (ret) {
		return ret;
	}

	/* Step 5 : Verify */
	ret = nt36670_Verify_Flash();
	if (ret) {
		return ret;
	}

	/*Step 6 : Bootloader Reset */
	nt36670_bootloader_reset();
	nt36670_check_fw_reset_state(RESET_STATE_INIT);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware when booting
	function.

return:
	n.a.
 *******************************************************/
void nt36670_Boot_Update_Firmware(struct work_struct *work)
{
	int32_t ret = 0;

	char firmware_name[256] = "";
	snprintf(firmware_name, 256, BOOT_UPDATE_FIRMWARE_NAME);

	/* request bin file in "/etc/firmware" */
	ret = nt36670_update_firmware_request(nt36670_ts->vtsdev, firmware_name);
	if (ret) {
		vts_dev_err(nt36670_ts->vtsdev, "nt36670_update_firmware_request failed. (%d)", ret);
		goto out;
	}

	mutex_lock(&nt36670_ts->lock);

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif

	nt36670_sw_reset_idle();
	ret = nt36670_Check_CheckSum();
	if (ret < 0) {	/* read firmware checksum failed */
		vts_dev_info(nt36670_ts->vtsdev, "read firmware checksum failed");
		nt36670_Update_Firmware();
	} else if ((ret == 0) || (nt36670_Check_FW_Ver() == 0)) {	/* (fw checksum not match) || (bin fw version != ic fw version) */
		vts_dev_info(nt36670_ts->vtsdev, "firmware version not match");
		nt36670_Update_Firmware();
	} else {
		/* Bootloader Reset */
		nt36670_bootloader_reset();
		nt36670_check_fw_reset_state(RESET_STATE_INIT);
	}
	mutex_unlock(&nt36670_ts->lock);

out:
	/*update the version to the drivers */
	ic_fw_version = nt36670_get_chip_FW_Ver();
}

int nt36670_hand_update_firmware(const struct firmware *firmware)
{
	const struct firmware *tmp_fw = nt36670_fw_entry;

	vts_dev_info(nt36670_ts->vtsdev, "enter");

	mutex_lock(&nt36670_ts->lock);
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif

	nt36670_fw_entry = firmware;
	nt36670_sw_reset_idle();

	vts_dev_err(nt36670_ts->vtsdev, "firmware version not match");
	nt36670_Update_Firmware();

	nt36670_fw_entry = tmp_fw;
	mutex_unlock(&nt36670_ts->lock);

	/*update the version to the drivers */
	ic_fw_version = nt36670_get_chip_FW_Ver();

	vts_dev_info(nt36670_ts->vtsdev, "end");
	return 0;
}

#endif /* BOOT_UPDATE_FIRMWARE */
