/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
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

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/vivo_ts_function.h>

#include "nt36xxx.h"

#if BOOT_UPDATE_FIRMWARE

#define FW_BIN_SIZE_116KB		(118784)
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET		(0x1A000)
#define FW_BIN_VER_BAR_OFFSET	(0x1A001)
#define FW_BIN_TYPE_OFFSET		(0x1A00D)

#define NVT_DUMP_SRAM   (0)

static struct timeval start, end;
const struct firmware *fw_entry_Spi = NULL;
static uint8_t *fwbuf = NULL;
static struct firmware array_info;

struct nvt_ts_bin_map {
	char name[12];
	uint32_t BIN_addr;
	uint32_t SRAM_addr;
	uint32_t size;
	uint32_t crc;
};

static struct nvt_ts_bin_map *bin_map;

/*******************************************************
Description:
	Novatek touchscreen init variable and allocate buffer
for download firmware function.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_init(void)
{
	/* allocate buffer for transfer firmware */
	//NVT_LOG("NVT_TANSFER_LEN = %ld\n", NVT_TANSFER_LEN);

	if (fwbuf == NULL) {
		fwbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1), GFP_KERNEL);
		if(fwbuf == NULL) {
			NVT_ERR("kzalloc for fwbuf failed!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static uint32_t byte_to_word(const uint8_t *data)
{
	return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

/*******************************************************
Description:
	Novatek touchscreen parsing bin header function.

return:
	n.a.
*******************************************************/
static uint32_t partition = 0;
static uint8_t ilm_dlm_num = 2;
static int32_t nvt_bin_header_parser(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	uint32_t pos = 0x00;
	uint32_t end = 0x00;
	uint8_t info_sec_num = 0;
	uint8_t ovly_sec_num = 0;
	uint8_t ovly_info = 0;

	/* Find the header size */
	end = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
	pos = 0x30;	// info section start at 0x30 offset
	while (pos < end) {
		info_sec_num ++;
		pos += 0x10;	/* each header info is 16 bytes */
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 */
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	NVT_LOG("ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

	/* allocated memory for header info */
	bin_map = (struct nvt_ts_bin_map *)kzalloc((partition+1) * sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if(bin_map == NULL) {
		NVT_ERR("kzalloc for bin_map failed!\n");
		return -ENOMEM;
	}

	for (list = 0; list < partition; list++) {
		/*
		 * [1] parsing ILM & DLM header info
		 * BIN_addr : SRAM_addr : size (12-bytes)
		 * crc located at 0x18 & 0x1C
		 */
		if (list < ilm_dlm_num) {
			bin_map[list].BIN_addr = byte_to_word(&fwdata[0 + list*12]);
			bin_map[list].SRAM_addr = byte_to_word(&fwdata[4 + list*12]);
			bin_map[list].size = byte_to_word(&fwdata[8 + list*12]);
			bin_map[list].crc = byte_to_word(&fwdata[0x18 + list*4]);
			if (list == 0)
				sprintf(bin_map[list].name, "ILM");
			else if (list == 1)
				sprintf(bin_map[list].name, "DLM");
		}

		/*
		 * [2] parsing others header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if ((list >= ilm_dlm_num) && (list < (ilm_dlm_num + info_sec_num))) {
			/* others partition located at 0x30 offset */
			pos = 0x30 + (0x10 * (list - ilm_dlm_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			/* detect header end to protect parser function */
			if ((bin_map[list].BIN_addr == 0) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[1].BIN_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
		}

		/* BIN size error detect */
		if ((bin_map[list].BIN_addr + bin_map[list].size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
			return -EINVAL;
		}

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
//				list, bin_map[list].name,
//				bin_map[list].SRAM_addr, bin_map[list].size,  bin_map[list].BIN_addr, bin_map[list].crc);
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen release update firmware function.

return:
	n.a.
*******************************************************/
static void update_firmware_release(void)
{
	if ((fw_entry_Spi != NULL) && (fw_entry_Spi != &array_info)) {
		release_firmware(fw_entry_Spi);
	}

	fw_entry_Spi = NULL;
}

/*******************************************************
Description:
	Novatek touchscreen request update firmware function.

return:
	Executive outcomes. 0---succeed. -1,-22---failed.
*******************************************************/
static int32_t update_firmware_request(char *filename, int fwtype, int fwRequest)
{
	uint8_t retry = 0;
	int32_t ret = 0;
	int fw_size = 0;

	if (NULL == filename) {
		return -1;
	}	

	while (1) {
		VTI("VIVO_TS loading the firmware\n");

		if (fwtype == FWTYPE_Normal) {
			VTI("VTS_FW_TYPE_FW= %d fwtype = %d", VTS_FW_TYPE_FW, fwtype);
			array_info.data = vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
			array_info.size = fw_size;
			update_firmware_release();
			fw_entry_Spi = &array_info;
		} else { //FWTYPE_MP
			VTI("VTS_FW_TYPE_MP = %d fwtype = %d", VTS_FW_TYPE_MP, fwtype);
			array_info.data = vivoTsGetFw(VTS_FW_TYPE_MP, &fw_size);
			array_info.size = fw_size;
			update_firmware_release();
			fw_entry_Spi = &array_info;
		}
			/*goto request_fail;*/

		// check bin file size (116kb)
		if (fw_entry_Spi->size != FW_BIN_SIZE) {
			NVT_ERR("bin file size not match. (%zu)\n", fw_entry_Spi->size);
			ret = -1;
			goto invalid;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(fw_entry_Spi->data + FW_BIN_VER_OFFSET) + *(fw_entry_Spi->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw_entry_Spi->data+FW_BIN_VER_OFFSET), *(fw_entry_Spi->data+FW_BIN_VER_BAR_OFFSET));
			ret = -1;
			goto invalid;
		}

		NVT_LOG("FW type is 0x%02X\n", *(fw_entry_Spi->data + FW_BIN_TYPE_OFFSET));

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(fw_entry_Spi->data, fw_entry_Spi->size);
		if (ret) {
			NVT_ERR("bin header parser failed\n");
			goto invalid;
		} else {
			break;
		}

invalid:
		update_firmware_release();

/*request_fail:*/
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	return ret;
}

#if NVT_DUMP_SRAM
/*******************************************************
Description:
	Novatek touchscreen dump flash partition function.

return:
	n.a.
*******************************************************/
loff_t file_offset = 0;
static void nvt_read_ram_test(uint32_t addr, uint16_t len, char *name)
{
	char file[256] = "";
	uint8_t *fbufp = NULL;
	int32_t ret = 0;
	struct file *fp = NULL;
	mm_segment_t org_fs;

	sprintf(file, "/data/local/tmp/dump_%s.bin", name);
	NVT_LOG("Dump [%s] from 0x%08X to 0x%08X\n", file, addr, addr+len);

	fbufp = (uint8_t *)kzalloc(len+1, GFP_KERNEL);
	if(fbufp == NULL) {
		NVT_ERR("kzalloc for fbufp failed!\n");
		return;
	}

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		NVT_ERR("open file failed\n");
		goto open_file_fail;
	}

	/* SPI read */
	//---set xdata index to addr---
	nvt_set_page(addr);

	fbufp[0] = addr & 0x7F;	//offset
	CTP_SPI_READ(ts_spi->client, fbufp, len+1);

	/* Write to file */
	ret = vfs_write(fp, (char __user *)fbufp+1, len, &file_offset);
	if (ret <= 0) {
		NVT_ERR("write file failed\n");
		goto open_file_fail;
	}

open_file_fail:
	set_fs(org_fs);
	if (!IS_ERR_OR_NULL(fp)) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	if (!IS_ERR_OR_NULL(fbufp)) {
		kfree(fbufp);
		fbufp = NULL;
	}

	return;
}
#endif

/*******************************************************
Description:
	Novatek touchscreen nvt_write_firmware function to write
firmware into each partition.

return:
	n.a.
*******************************************************/
static int32_t nvt_write_firmware(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	char *name;
	uint32_t BIN_addr, SRAM_addr, size;
	uint32_t i = 0;
	uint32_t len = 0;
	int32_t count = 0;
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	for (list = 0; list < partition; list++) {
		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		BIN_addr = bin_map[list].BIN_addr;
		name = bin_map[list].name;

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X)\n",
//				list, name, SRAM_addr, size, BIN_addr);

		/* Check data size */
		if ((BIN_addr + size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					BIN_addr, BIN_addr + size);
			ret = -1;
			goto out;
		}

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size +1;

		/* write data to SRAM */
		if (size % NVT_TANSFER_LEN)
			count = (size / NVT_TANSFER_LEN) + 1;
		else
			count = (size / NVT_TANSFER_LEN);

		for (i = 0 ; i < count ; i++) {
			len = (size < NVT_TANSFER_LEN) ? size : NVT_TANSFER_LEN;

			//---set xdata index to start address of SRAM---
			nvt_set_page(SRAM_addr);

			//---write data into SRAM---
			fwbuf[0] = SRAM_addr & 0x7F;	//offset
			memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
			CTP_SPI_WRITE(ts_spi->client, fwbuf, len+1);

#if NVT_DUMP_SRAM
			/* dump for debug download firmware */
			nvt_read_ram_test(SRAM_addr, len, name);
#endif
			SRAM_addr += NVT_TANSFER_LEN;
			BIN_addr += NVT_TANSFER_LEN;
			size -= NVT_TANSFER_LEN;
		}

#if NVT_DUMP_SRAM
		file_offset = 0;
#endif
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set bootload crc reg bank function.
This function will set hw crc reg before enable crc function.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_crc_bank(uint32_t DES_ADDR, uint32_t SRAM_ADDR,
		uint32_t LENGTH_ADDR, uint32_t size,
		uint32_t G_CHECKSUM_ADDR, uint32_t crc)
{
	/* write destination address */
	nvt_set_page(DES_ADDR);
	fwbuf[0] = DES_ADDR & 0x7F;
	fwbuf[1] = (SRAM_ADDR) & 0xFF;
	fwbuf[2] = (SRAM_ADDR >> 8) & 0xFF;
	fwbuf[3] = (SRAM_ADDR >> 16) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 4);

	/* write length */
	//nvt_set_page(LENGTH_ADDR);
	fwbuf[0] = LENGTH_ADDR & 0x7F;
	fwbuf[1] = (size) & 0xFF;
	fwbuf[2] = (size >> 8) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 3);

	/* write golden dlm checksum */
	//nvt_set_page(G_CHECKSUM_ADDR);
	fwbuf[0] = G_CHECKSUM_ADDR & 0x7F;
	fwbuf[1] = (crc) & 0xFF;
	fwbuf[2] = (crc >> 8) & 0xFF;
	fwbuf[3] = (crc >> 16) & 0xFF;
	fwbuf[4] = (crc >> 24) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 5);

	return;
}

/*******************************************************
Description:
	Novatek touchscreen check DMA hw crc function.
This function will check hw crc result is pass or not.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_hw_crc(void)
{
	/* [0] ILM */
	/* write register bank */
	nvt_set_bld_crc_bank(ts_spi->mmap->ILM_DES_ADDR, bin_map[0].SRAM_addr,
			ts_spi->mmap->ILM_LENGTH_ADDR, bin_map[0].size,
			ts_spi->mmap->G_ILM_CHECKSUM_ADDR, bin_map[0].crc);

	/* [1] DLM */
	/* write register bank */
	nvt_set_bld_crc_bank(ts_spi->mmap->DLM_DES_ADDR, bin_map[1].SRAM_addr,
			ts_spi->mmap->DLM_LENGTH_ADDR, bin_map[1].size,
			ts_spi->mmap->G_DLM_CHECKSUM_ADDR, bin_map[1].crc);
}

/*******************************************************
Description:
	Novatek touchscreen Download_Firmware with HW CRC
function. It's complete download firmware flow.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_firmware_hw_crc(void)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	do_gettimeofday(&start);

	while (1) {
		/* bootloader reset to reset MCU */
		nvt_bootloader_reset_Spi();

		/* Start to write firmware process */
		ret = nvt_write_firmware(fw_entry_Spi->data, fw_entry_Spi->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}

		/* set ilm & dlm reg bank */
		nvt_set_bld_hw_crc();

		/* enable hw bld crc function */
		nvt_bld_crc_enable();

		/* clear fw reset status & enable fw crc check */
		nvt_fw_crc_enable();

		/* Set Boot Ready Bit */
		nvt_boot_ready();

		ret = nvt_check_fw_reset_state_Spi(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("nvt_check_fw_reset_state_Spi failed. (%d)\n", ret);
			goto fail;
		} else {
			break;
		}

fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	do_gettimeofday(&end);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware main function.

return:
	n.a.
*******************************************************/
void nvt_update_firmware(char *firmware_name, int fwtype, int fwRequest)
{
	int8_t ret = 0;

	// request bin file in "/etc/firmware"
	
	ret = update_firmware_request(firmware_name, fwtype, fwRequest);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init();
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto init_fail;
	}

	/* download firmware process */
	ret = nvt_download_firmware_hw_crc();
	if (ret) {
		NVT_ERR("Download Firmware failed. (%d)\n", ret);
		goto download_fail;
	}

	NVT_LOG("Update firmware success! <%ld.%06ld>\n",
			(end.tv_sec - start.tv_sec), (end.tv_usec - start.tv_usec));

	/* Get FW Info */
	ret = nvt_get_fw_info_Spi();
	if (ret) {
		NVT_ERR("nvt_get_fw_info_Spi failed. (%d)\n", ret);
		goto download_fail;
	}

download_fail:
	kfree(bin_map);
init_fail:
	update_firmware_release();
request_firmware_fail:

	return;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware when booting
	function.

return:
	n.a.
*******************************************************/
void Boot_Update_Firmware_Spi(struct work_struct *work)
{
	mutex_lock(&ts_spi->lock);
	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
	mutex_unlock(&ts_spi->lock);
}
#endif /* BOOT_UPDATE_FIRMWARE */
