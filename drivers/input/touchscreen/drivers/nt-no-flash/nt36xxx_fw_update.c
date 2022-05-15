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

#include "nt36xxx.h"

#if BOOT_UPDATE_FIRMWARE

#if 1 //new modified[20190426,jx]
	#define SIZE_4KB 4096
	#define FLASH_SECTOR_SIZE SIZE_4KB
	#define FW_BIN_VER_OFFSET (fw_need_write_size - SIZE_4KB)
	#define FW_BIN_VER_BAR_OFFSET (FW_BIN_VER_OFFSET + 1)
	#define NVT_FLASH_END_FLAG_LEN 3
	#define NVT_FLASH_END_FLAG_ADDR (fw_need_write_size - NVT_FLASH_END_FLAG_LEN)
#else //Old
	#define FW_BIN_SIZE_116KB		(118784)
	#define FW_BIN_SIZE 			FW_BIN_SIZE_116KB
	#define FW_BIN_VER_OFFSET		(0x1A000)
	#define FW_BIN_VER_BAR_OFFSET	(0x1A001)
	#define FW_BIN_TYPE_OFFSET		(0x1A00D)
#endif

#define NVT_DUMP_SRAM   (0)


s64 start_time; 
s64 end_time;

const struct firmware *ntnf_fw_entry = NULL;
static size_t fw_need_write_size = 0;
static uint8_t *fwbuf = NULL;
static struct firmware array_info;

struct nvt_ts_bin_map {
	char name[12];
	uint32_t BIN_addr;
	uint32_t SRAM_addr;
	uint32_t size;
	uint32_t crc;
	NVT_PARTITION_TYPE type;
};

static struct nvt_ts_bin_map *bin_map;
static int32_t nvt_get_fw_need_write_size(const struct firmware *fw_entry)
{
	int32_t i = 0;
	int32_t total_sectors_to_check = 0;

	total_sectors_to_check = fw_entry->size / FLASH_SECTOR_SIZE;
	/* printk("total_sectors_to_check = %d\n", total_sectors_to_check); */

	for (i = total_sectors_to_check; i > 0; i--) {
		/* printk("current end flag address checked = 0x%X\n", i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN); */
		/* check if there is end flag "NVT" at the end of this sector */
		if (strncmp(&fw_entry->data[i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN], "NVT", NVT_FLASH_END_FLAG_LEN) == 0) {
			fw_need_write_size = i * FLASH_SECTOR_SIZE;
			NVT_LOG("fw_need_write_size = %zu(0x%zx), NVT end flag\n", fw_need_write_size, fw_need_write_size);
			return 0;
		}

		/* check if there is end flag "MOD" at the end of this sector */
		if (strncmp(&fw_entry->data[i * FLASH_SECTOR_SIZE - NVT_FLASH_END_FLAG_LEN], "MOD", NVT_FLASH_END_FLAG_LEN) == 0) {
			fw_need_write_size = i * FLASH_SECTOR_SIZE;
			NVT_LOG("fw_need_write_size = %zu(0x%zx), MOD end flag\n", fw_need_write_size, fw_need_write_size);
			return 0;
		}
	}

	NVT_ERR("end flag \"NVT\" \"MOD\" not found!\n");
	return -1;
}

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
static uint32_t CheckSum(const u8 *data, uint32_t len)
{
	uint32_t i = 0;
	uint32_t checksum = 0;

	for (i = 0 ; i < len+1 ; i++)
		checksum += data[i];

	checksum += len;
	checksum = ~checksum +1;

	return checksum;
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
	uint32_t end_time = 0x00;
	uint8_t info_sec_num = 0;
	uint8_t ovly_sec_num = 0;
	uint8_t ovly_info = 0;

	/* Find the header size */
	end_time = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
	pos = 0x30;	// info section start_time at 0x30 offset
	while (pos < end_time) {
		info_sec_num ++;
		pos += 0x10;	/* each header info is 16 bytes */
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 */
	ntnf_spi->wkg_info = (fwdata[0x28] & 0x20) >> 5;
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	NVT_LOG("wkg_info = %d, ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ntnf_spi->wkg_info, ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

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
			if(ntnf_spi->hw_crc){
				bin_map[list].crc = byte_to_word(&fwdata[0x18 + list*4]);
			}
			else {
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			} //ts->hw_crc
			if (list == 0){
				sprintf(bin_map[list].name, "ILM");
				bin_map[list].type = NVTILM;
				}
			else if (list == 1){
				sprintf(bin_map[list].name, "DLM");
				bin_map[list].type = NVTDLM;
			}
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
			if(ntnf_spi->hw_crc){
				bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			}
			else{
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			}
			/* detect header end_time to protect parser function */
			if ((bin_map[list].BIN_addr == 0) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
				bin_map[list].type = NVTHEADER;
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
				bin_map[list].type = NVTINFO;
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start_time addr */
			pos = bin_map[1].BIN_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			if(ntnf_spi->hw_crc){
				bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			}else{
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			}
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
			/* if wkg_info flag enable, overlay-0 is gesture function */
			if ((list == (ilm_dlm_num + info_sec_num)) && (ntnf_spi->wkg_info)) {
				bin_map[list].type = NVTWKG;
			} else {
				bin_map[list].type = NVTOVLAY;
			}
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
	if ((ntnf_fw_entry != NULL) && (ntnf_fw_entry != &array_info)) {
		release_firmware(ntnf_fw_entry);
	}

	ntnf_fw_entry = NULL;
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
	bool state = true;

	if (NULL == filename) {
		return -1;
	}

	while (1) {	
		NVT_LOG("filename is %s\n", filename);

		if (vts_state_get(ntnf_spi->vtsdev, VTS_STA_FACTORY_SWITCH) && (fwRequest == FWTYPE_REQUEST_YES)) {
			ret = request_firmware(&ntnf_fw_entry, filename, &ntnf_spi->client->dev); /*success 0*/
			state = false;
		}

		if (state || ret) {
			VTI("VIVO_TS loading the firmware\n");

			if (fwtype == FWTYPE_Normal) {
				VTI("VTS_FW_TYPE_FW= %d fwtype = %d", VTS_FW_TYPE_FW, fwtype);
				array_info.data = vts_fw_data_get(ntnf_spi->vtsdev, VTS_FW_TYPE_FW, &fw_size);
				array_info.size = fw_size;
				update_firmware_release();
				ntnf_fw_entry = &array_info;
			} else { //FWTYPE_MP
				VTI("VTS_FW_TYPE_MP = %d fwtype = %d", VTS_FW_TYPE_MP, fwtype);
				array_info.data = vts_fw_data_get(ntnf_spi->vtsdev, VTS_FW_TYPE_MP, &fw_size);
				array_info.size = fw_size;
				update_firmware_release();
				ntnf_fw_entry = &array_info;
			}
		}
		/*goto request_fail;*/

		// check FW need to write size
		if (nvt_get_fw_need_write_size(ntnf_fw_entry)) {
			NVT_ERR("get fw need to write size fail!\n");
			ret = -EINVAL;
			goto invalid;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(ntnf_fw_entry->data + FW_BIN_VER_OFFSET) + *(ntnf_fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(ntnf_fw_entry->data+FW_BIN_VER_OFFSET), *(ntnf_fw_entry->data+FW_BIN_VER_BAR_OFFSET));
			ret = -1;
			goto invalid;
		}

		//Disable it[20190426,jx]
		//NVT_LOG("FW type is 0x%02X\n", *(fw_entry->data + FW_BIN_TYPE_OFFSET));

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(ntnf_fw_entry->data, ntnf_fw_entry->size);
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
	ntnf_set_page(addr);

	fbufp[0] = addr & 0x7F;	//offset
	NTNF_CTP_SPI_READ(ntnf_spi->client, fbufp, len+1);

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

static int32_t nvt_check_fw_checksum(void)
{
	uint32_t fw_checksum = 0;
	uint32_t len = partition*4;
	uint32_t list = 0;
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	//---set xdata index to checksum---
	ntnf_set_page(ntnf_spi->mmap->R_ILM_CHECKSUM_ADDR);

	/* read checksum */
	fwbuf[0] = (ntnf_spi->mmap->R_ILM_CHECKSUM_ADDR) & 0x7F;
	ret = NTNF_CTP_SPI_READ(ntnf_spi->client, fwbuf, len+1);
	if (ret) {
		NVT_ERR("Read fw checksum failed\n");
		return ret;
	}

	/*
	 * Compare each checksum from fw
	 * ILM + DLM + Overlay + Info
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	for (list = 0; list < partition; list++) {
		/* ignore WKG for normally boot code */
		if (bin_map[list].type == NVTWKG) {
			continue;
		}
		fw_checksum = byte_to_word(&fwbuf[1+list*4]);

		/* ignore reserved partition (Reserved Partition size is zero) */
		if(!bin_map[list].size)
			continue;

		if (bin_map[list].crc != fw_checksum) {
			NVT_ERR("[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
					list, bin_map[list].crc, fw_checksum);
			ret = -EIO;
		}
	}

	return ret;
}

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

			//---set xdata index to start_time address of SRAM---
			ntnf_set_page(SRAM_addr);

			//---write data into SRAM---
			fwbuf[0] = SRAM_addr & 0x7F;	//offset
			memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
			NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, len+1);

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
	ntnf_set_page(DES_ADDR);
	fwbuf[0] = DES_ADDR & 0x7F;
	fwbuf[1] = (SRAM_ADDR) & 0xFF;
	fwbuf[2] = (SRAM_ADDR >> 8) & 0xFF;
	fwbuf[3] = (SRAM_ADDR >> 16) & 0xFF;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, 4);

	/* write length */
	//ntnf_set_page(LENGTH_ADDR);
	fwbuf[0] = LENGTH_ADDR & 0x7F;
	fwbuf[1] = (size) & 0xFF;
	fwbuf[2] = (size >> 8) & 0xFF;
	fwbuf[3] = (size >> 16) & 0x01;
	if (ntnf_spi->hw_crc == 1) {
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, 3);
	} else if (ntnf_spi->hw_crc > 1) {
		NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, 4);
	}

	/* write golden dlm checksum */
	//ntnf_set_page(G_CHECKSUM_ADDR);
	fwbuf[0] = G_CHECKSUM_ADDR & 0x7F;
	fwbuf[1] = (crc) & 0xFF;
	fwbuf[2] = (crc >> 8) & 0xFF;
	fwbuf[3] = (crc >> 16) & 0xFF;
	fwbuf[4] = (crc >> 24) & 0xFF;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, 5);

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
	nvt_set_bld_crc_bank(ntnf_spi->mmap->ILM_DES_ADDR, bin_map[0].SRAM_addr,
			ntnf_spi->mmap->ILM_LENGTH_ADDR, bin_map[0].size,
			ntnf_spi->mmap->G_ILM_CHECKSUM_ADDR, bin_map[0].crc);

	/* [1] DLM */
	/* write register bank */
	nvt_set_bld_crc_bank(ntnf_spi->mmap->DLM_DES_ADDR, bin_map[1].SRAM_addr,
			ntnf_spi->mmap->DLM_LENGTH_ADDR, bin_map[1].size,
			ntnf_spi->mmap->G_DLM_CHECKSUM_ADDR, bin_map[1].crc);
}

/*******************************************************
Description:
	Novatek touchscreen read BLD hw crc info function.
This function will check crc results from register.

return:
	n.a.
History:
	[20190919,jx]Add
*******************************************************/
static void nvt_read_bld_hw_crc(void)
{
	uint8_t buf[8] = {0};
	uint32_t g_crc = 0, r_crc = 0;

	/* CRC Flag */
	ntnf_set_page(ntnf_spi->mmap->BLD_ILM_DLM_CRC_ADDR);
	buf[0] = ntnf_spi->mmap->BLD_ILM_DLM_CRC_ADDR & 0x7F;
	buf[1] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);
	NVT_ERR("crc_done = %d, ilm_crc_flag = %d, dlm_crc_flag = %d\n",
			(buf[1] >> 2) & 0x01, (buf[1] >> 0) & 0x01, (buf[1] >> 1) & 0x01);

	/* ILM CRC */
	ntnf_set_page(ntnf_spi->mmap->G_ILM_CHECKSUM_ADDR);
	buf[0] = ntnf_spi->mmap->G_ILM_CHECKSUM_ADDR & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);
	g_crc = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);

	ntnf_set_page(ntnf_spi->mmap->R_ILM_CHECKSUM_ADDR);
	buf[0] = ntnf_spi->mmap->R_ILM_CHECKSUM_ADDR & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);
	r_crc = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);

	NVT_ERR("ilm: bin crc = 0x%08X, golden = 0x%08X, result = 0x%08X\n",
			bin_map[0].crc, g_crc, r_crc);

	/* DLM CRC */
	ntnf_set_page(ntnf_spi->mmap->G_DLM_CHECKSUM_ADDR);
	buf[0] = ntnf_spi->mmap->G_DLM_CHECKSUM_ADDR & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);
	g_crc = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);

	ntnf_set_page(ntnf_spi->mmap->R_DLM_CHECKSUM_ADDR);
	buf[0] = ntnf_spi->mmap->R_DLM_CHECKSUM_ADDR & 0x7F;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);
	r_crc = buf[1] | (buf[2] << 8) | (buf[3] << 16) | (buf[4] << 24);

	NVT_ERR("dlm: bin crc = 0x%08X, golden = 0x%08X, result = 0x%08X\n",
			bin_map[1].crc, g_crc, r_crc);

	return;
}

#if NVT_TOUCH_ESD_DISP_RECOVERY
#define ILM_CRC_FLAG 0x01
#define DLM_CRC_FLAG 0x02
#define CRC_DONE 0x04
//static int32_t nvt_check_crc_done_ilm_err(void)
int32_t nvt_check_crc_done_ilm_err(void)
{
	uint8_t buf[8] = {0};

	ntnf_set_page(ntnf_spi->mmap->BLD_ILM_DLM_CRC_ADDR);
	buf[0] = ntnf_spi->mmap->BLD_ILM_DLM_CRC_ADDR & 0x7F;
	buf[1] = 0x00;
	NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);

	NVT_LOG("CRC DONE, ILM DLM FLAG = 0x%02X\n", buf[1]);
	if (((buf[1] & ILM_CRC_FLAG) && (buf[1] & CRC_DONE)) ||
		((buf[1] & DLM_CRC_FLAG) && (buf[1] & CRC_DONE))||
		(buf[1] == 0xFE) || ((buf[1] & CRC_DONE) == 0x00))
		return 1;
	else
		return 0;
}

#define DISP_OFF_ADDR 0x2800
//static int nvt_f2c_disp_off(void)
int nvt_f2c_disp_off(void)
{
	uint8_t buf[8] = {0};
	int ret = 0;
	uint8_t tmp_val = 0;
	int32_t write_disp_off_retry = 0;
	int32_t retry = 0;

	NVT_LOG("++\n");

	// SW Reset & Idle
	ntnf_sw_reset_idle();

	//Setp1: Set REG CPU_IF_ADDR[15:0]
	ntnf_write_addr(ntnf_spi->mmap->CPU_IF_ADDR_LOW, DISP_OFF_ADDR & 0xFF);
	ntnf_write_addr(ntnf_spi->mmap->CPU_IF_ADDR_HIGH, (DISP_OFF_ADDR >> 8) & 0xFF);

	//Step2: Set REG FFM_ADDR[15:0]
	// set FFM_ADDR to 0x20000
	ntnf_write_addr(ntnf_spi->mmap->FFM_ADDR_LOW, 0x00);
	ntnf_write_addr(ntnf_spi->mmap->FFM_ADDR_MID, 0x00);
	if (ntnf_spi->hw_crc > 1)
		ntnf_write_addr(ntnf_spi->mmap->FFM_ADDR_HIGH, 0x00);

	//Step3: Set REG F2C_LENGT[H7:0]
	ntnf_write_addr(ntnf_spi->mmap->F2C_LENGTH, 1);

nvt_write_disp_off_retry:
	//Step4: Set REG CPU_Polling_En=1, F2C_RW=1, CPU_IF_ADDR_INC=1, F2C_EN=1
	ntnf_set_page(ntnf_spi->mmap->FFM2CPU_CTL);
	buf[0] = ntnf_spi->mmap->FFM2CPU_CTL & 0x7F;
	buf[1] = 0xFF;
	ret = NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 2);
	if (ret) {
		NVT_ERR("Read FFM2CPU control failed!\n");
		return ret;
	}
	tmp_val = buf[1] | 0x27;
	ntnf_write_addr(ntnf_spi->mmap->FFM2CPU_CTL, tmp_val);

	//Step5: wait F2C_EN = 0
	retry = 0;
	while (1) {
		ntnf_set_page(ntnf_spi->mmap->FFM2CPU_CTL);
		buf[0] = ntnf_spi->mmap->FFM2CPU_CTL & 0x7F;
		buf[1] = 0xFF;
		buf[2] = 0xFF;
		ret = NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 3);
		if (ret) {
			NVT_ERR("Read FFM2CPU control failed!\n");
			return ret;
		}

		if ((buf[1] & 0x01) == 0x00)
			break;

		usleep_range(1000, 1000);
		retry++;

		if(unlikely(retry > 1)) {
			NVT_ERR("Wait F2C_EN = 0 failed!\n");
			return -EIO;
		}
	}

	//Step6: Check REG TH_CPU_CHK  status (1: Success,  0: Fail), if 0, can Retry Step4.
	if (((buf[2] & 0x04) >> 2) != 0x01) {
		write_disp_off_retry++;
		if (write_disp_off_retry <= 3) {
			goto nvt_write_disp_off_retry;
		} else {
			NVT_ERR("Write display off failed!, buf[1]=0x%02X, buf[2]=0x%02X\n", buf[1], buf[2]);
			return -EIO;
		}
	}
	NVT_LOG("--\n");

	return ret;
}
#endif /* #if NVT_TOUCH_ESD_DISP_RECOVERY */

static int32_t nvt_download_firmware(void)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	
	start_time = ktime_to_us(ktime_get());

	while (1) {
		/*
		 * Send eng reset cmd before download FW
		 * Keep TP_RESX low when send eng reset cmd
		 */
#if NVT_TOUCH_SUPPORT_HW_RST
	if(gpio_is_valid(ntnf_spi->reset_gpio)){
		gpio_set_value(ntnf_spi->reset_gpio, 0);
		mdelay(1);	//wait 1ms
		nvt_eng_reset_v2();
		gpio_set_value(ntnf_spi->reset_gpio, 1);
		mdelay(10);	//wait tRT2BRST after TP_RST
	}
#endif
		ntnf_bootloader_reset();

		/* clear fw reset status */
		ntnf_write_addr(ntnf_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE, 0x00);

		/* Start to write firmware process */
		ret = nvt_write_firmware(ntnf_fw_entry->data, ntnf_fw_entry->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}


		/* Set Boot Ready Bit */
		ntnf_boot_ready();

		ret = ntnf_check_fw_reset_state(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("nvt_check_fw_reset_state failed. (%d)\n", ret);
			goto fail;
		}
		/* check fw checksum result */
		ret = nvt_check_fw_checksum();
		if (ret) {
			NVT_ERR("firmware checksum not match, retry=%d\n", retry);
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




	end_time = ktime_to_us(ktime_get());
	return ret;
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

	start_time = ktime_to_us(ktime_get());

	while (1) {
		/* bootloader reset to reset MCU */
		ntnf_bootloader_reset();

		/* Start to write firmware process */
		ret = nvt_write_firmware(ntnf_fw_entry->data, ntnf_fw_entry->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}

		/* set ilm & dlm reg bank */
		nvt_set_bld_hw_crc();

		/* enable hw bld crc function */
		ntnf_bld_crc_enable();

		/* clear fw reset status & enable fw crc check */
		ntnf_fw_crc_enable();

		/* Set Boot Ready Bit */
		ntnf_boot_ready();

		ret = ntnf_check_fw_reset_state(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("ntnf_check_fw_reset_state failed. (%d)\n", ret);
			goto fail;
		} else {
			break;
		}

fail:
		retry++;
		if(unlikely(retry > 1)) {
			NVT_ERR("error, retry=%d\n", retry);

			nvt_read_bld_hw_crc();
			#if NVT_TOUCH_ESD_DISP_RECOVERY
				if (nvt_check_crc_done_ilm_err()) {
					NVT_ERR("set display off to trigger display esd recovery.\n");
					nvt_f2c_disp_off();
				}
			#endif /* #if NVT_TOUCH_ESD_DISP_RECOVERY */
			
			break;
		}
	}

	end_time = ktime_to_us(ktime_get());

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware main function.

return:
	n.a.
*******************************************************/
void ntnf_update_firmware(char *firmware_name, int fwtype, int fwRequest)
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
	if(ntnf_spi->hw_crc)
	ret = nvt_download_firmware_hw_crc();
	else
	ret = nvt_download_firmware();
	if (ret) {
		NVT_ERR("Download Firmware failed. (%d)\n", ret);
		goto download_fail;
	}

	NVT_LOG("Update firmware success! <%lldus>\n", end_time - start_time);

	/* Get FW Info */
	ret = ntnf_get_fw_info();
	if (ret) {
		NVT_ERR("ntnf_get_fw_info failed. (%d)\n", ret);
		goto download_fail;
	}

download_fail:
	kfree(bin_map);
init_fail:
	update_firmware_release();
request_firmware_fail:

	return;
}

static int32_t nvt_get_gesture_info(uint32_t *SRAM_addr, uint32_t *size,
		uint32_t *BIN_addr, uint32_t *crc)
{
	uint32_t list = 0;
	int32_t ret = -1;	
	for (list = 0; list < partition; list++) {
		/* get WKG info */
		if (bin_map[list].type == NVTWKG) {
			/* initialize variable */
			*SRAM_addr = bin_map[list].SRAM_addr;
			*size = bin_map[list].size;
			*BIN_addr = bin_map[list].BIN_addr;
			*crc = bin_map[list].crc;

			//NVT_LOG("[WKG] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
			//		*SRAM_addr, *size,  *BIN_addr, *crc);

			ret = 0;
			break;
		} else {
			continue;
		}
	}

	return ret;
}
/*******************************************************
Description:
	Novatek touchscreen write data to sram function.

- fwdata   : The buffer is written
- SRAM_addr: The sram destination address
- size     : Number of data bytes in @fwdata being written
- BIN_addr : The transferred data offset of @fwdata

return:
	Executive outcomes. 0---succeed. else---fail.
*******************************************************/
static int32_t nvt_write_sram(const u8 *fwdata,
		uint32_t SRAM_addr, uint32_t size, uint32_t BIN_addr)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t len = 0;
	int32_t count = 0;

	if (size % NVT_TANSFER_LEN)
		count = (size / NVT_TANSFER_LEN) + 1;
	else
		count = (size / NVT_TANSFER_LEN);

	for (i = 0 ; i < count ; i++) {
		len = (size < NVT_TANSFER_LEN) ? size : NVT_TANSFER_LEN;

		//---set xdata index to start address of SRAM---
		ret = ntnf_set_page(SRAM_addr);
		if (ret) {
			NVT_ERR("set page failed, ret = %d\n", ret);
			return ret;
		}

		//---write data into SRAM---
		fwbuf[0] = SRAM_addr & 0x7F;	//offset
		memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
		ret = NTNF_CTP_SPI_WRITE(ntnf_spi->client, fwbuf, len+1);
		if (ret) {
			NVT_ERR("write to sram failed, ret = %d\n", ret);
			return ret;
		}

		SRAM_addr += NVT_TANSFER_LEN;
		BIN_addr += NVT_TANSFER_LEN;
		size -= NVT_TANSFER_LEN;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen download gesture code function.

- fwdata   : fw data
- fwsize   : fw size
- SRAM_addr: The sram destination address
- size     : Number of data bytes in @fwdata being written
- BIN_addr : The transferred data offset of @fwdata
- crc      : The crc result of transferred data

return:
	Executive outcomes. 0---succeed. else---fail.
*******************************************************/
static int32_t nvt_write_gesture_firmware(const u8 *fwdata, size_t fwsize,
		uint32_t SRAM_addr, uint32_t size, uint32_t BIN_addr, uint32_t crc)
{
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	/* Check data size */
	if ((BIN_addr + size) > fwsize) {
		NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
				BIN_addr, BIN_addr + size);
		ret = -EINVAL;
		goto out;
	}

	size = size +1;

	/* write data to SRAM */
	ret = nvt_write_sram(fwdata, SRAM_addr, size, BIN_addr);
	if (ret) {
		NVT_ERR("sram program failed, ret = %d\n", ret);
		goto out;
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check gesture checksum function.
This function will compare file checksum and fw checksum.

return:
	n.a.
*******************************************************/
static int32_t nvt_check_gesture_checksum(uint32_t SRAM_addr, uint32_t size, uint32_t crc)
{
	uint32_t fw_checksum = 0;
	uint32_t wkg_size = size;
	uint32_t i = 0;
	uint32_t len = 0;
	uint32_t count = 0;
	uint32_t nvt_read_max = NTV_GESTURE_FW_CHECK_LEN;	/* max read 1k data */
	uint32_t offset = 0;
	uint8_t *data  = ntnf_spi->g_data;//nvt_read_max +2 
	int32_t ret = 0;

	if (nvt_read_max >= sizeof(ntnf_spi->rbuf)) {
		NVT_ERR("read len %d is larger than buffer size %ld\n",
				nvt_read_max, sizeof(ntnf_spi->rbuf));
		return -EINVAL;
	} else if (nvt_read_max >= NVT_TANSFER_LEN) {
		NVT_ERR("read len %d is larger than NVT_TANSFER_LEN\n", nvt_read_max);
		return -EINVAL;
	}

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	size = size + 1;

	/* read data from SRAM */
	if (size % nvt_read_max)
		count = (size / nvt_read_max) + 1;
	else
		count = (size / nvt_read_max);

	for (i = 0 ; i < count ; i++) {
		len = (size < nvt_read_max) ? size : nvt_read_max;

		//---set xdata index---
		ntnf_set_page(SRAM_addr);

		data[0] = (SRAM_addr) & 0x7F;
		ret = NTNF_CTP_SPI_READ(ntnf_spi->client, data, len+1);
		if (ret) {
			NVT_ERR("Read fw checksum failed\n");
			return ret;
		}

		memcpy((fwbuf+offset), (data+1), (len));
		SRAM_addr += nvt_read_max;
		size -= nvt_read_max;
		offset += nvt_read_max;
	}

	/* calculate checksum from readback data and compare it! */
	fw_checksum = CheckSum(&fwbuf[0], wkg_size);

	if (crc != fw_checksum) {
		NVT_ERR("[WKG] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
				crc, fw_checksum);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware when booting
	function.

return:
	n.a.
*******************************************************/
void Ntnf_Boot_Update_Firmware(struct work_struct *work)
{
	mutex_lock(&ntnf_spi->lock);
	ntnf_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_YES);
	mutex_unlock(&ntnf_spi->lock);
}
/*******************************************************
Description:
	Novatek touchscreen complete flow for download
gesture firmware code function.

return:
	n/a.
*******************************************************/
void nvt_update_gesture_firmware_V2(char *firmware_name, int fwtype, int fwRequest)
{
	uint32_t BIN_addr, SRAM_addr, size, crc;
	uint32_t i = 0;
	uint32_t retry = 0;
	uint8_t buf[8] = {0};
	int32_t ret = 0;
				

	start_time = ktime_to_us(ktime_get());
	
	// request bin file in "/etc/firmware"
	ret = update_firmware_request(firmware_name, fwtype, fwRequest);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	ret = nvt_get_gesture_info(&SRAM_addr, &size, &BIN_addr, &crc);
	if (ret) {
		NVT_ERR("get gesture info failed. (%d)\n", ret);
		goto download_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init();
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto download_fail;
	}

	/* Step 1: Host Send CMD 0x13, 0xFF, 0xFF, 0x00 to EVENT_MAP_HOST_CMD */
	//---write spi command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	NTNF_CTP_SPI_WRITE(ntnf_spi->client, buf, 5);

	/* Step 2: Host Polling 0xA5 from EVENT_MAP_HOST_CMD */
	for (i=0 ; i<=20 ; i++) {
		usleep_range(10000, 10000);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		buf[2] = 0xFF;
		buf[3] = 0xFF;
		buf[4] = 0xFF;
		NTNF_CTP_SPI_READ(ntnf_spi->client, buf, 5);

		if (buf[4] == 0xA5) {
			break;
		}

		if (i == 20) {
			NVT_LOG("Polling 0x%02X 0x%02X 0x%02X 0x%02X failed, retry %d\n",
					buf[1], buf[2], buf[3], buf[4], i);
			goto download_fail;
		}
	}

	while (1) {
		/* Step 3: Load gesture code */
		ret = nvt_write_gesture_firmware(ntnf_fw_entry->data, ntnf_fw_entry->size,
				SRAM_addr, size, BIN_addr, crc);
		if (ret) {
			NVT_ERR("write gesture firmware failed. (%d)\n", ret);
			goto download_fail;
		}

		/* Step 4: Check FW Checksum */
		ret = nvt_check_gesture_checksum(SRAM_addr, size, crc);
		if (!ret) {
			break;
		}

		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("load gesture code failed, crc = 0x%02X, retry = %d\n", crc, retry);
			goto download_fail;
		}
	}

	/* Step 5: Host Send CMD 0x00 to EVENT_BUF_ADDR offset 0x53 */
	ntnf_write_addr((ntnf_spi->mmap->EVENT_BUF_ADDR | 0x53), 0x00);


	end_time = ktime_to_us(ktime_get());
	NVT_LOG("Update gesture firmware success! <%lldus>\n", start_time - end_time);

download_fail:
	if (!IS_ERR_OR_NULL(bin_map)) {
		kfree(bin_map);
		bin_map = NULL;
	}

	update_firmware_release();
request_firmware_fail:
	return;
}

#endif /* BOOT_UPDATE_FIRMWARE */

