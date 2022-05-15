/* drivers/input/touchscreen/sec_ts_fw.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sec_ts.h"
#include "../vts_core.h"


#define SEC_TS_FW_BLK_SIZE		512

enum {
	BUILT_IN = 0,
	UMS,
	BL,
	FFU,
};

static int sec_ts_enter_fw_mode(struct sec_ts_data *ts)
{
	int ret;
	u8 fw_update_mode_passwd[] = {0x55, 0xAC};
	u8 fw_status;
	u8 id[3];

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_ENTER_FW_MODE, fw_update_mode_passwd, sizeof(fw_update_mode_passwd));
	sec_ts_delay(20);
	if (ret < 0) {
		VTE("%s: write fail, enter_fw_mode\n", __func__);
		return -EIO;
	}

	VTI("%s: write ok, enter_fw_mode - 0x%x 0x%x 0x%x\n",
			__func__, SEC_TS_CMD_ENTER_FW_MODE, fw_update_mode_passwd[0], fw_update_mode_passwd[1]);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &fw_status, 1);
	if (ret < 0) {
		VTE("%s: read fail, read_boot_status\n", __func__);
		return -EIO;
	}
	if (fw_status != SEC_TS_STATUS_BOOT_MODE) {
		VTE("%s: enter fail! read_boot_status = 0x%x\n", __func__, fw_status);
		return -EINVAL;
	}

	VTI("%s: Success! read_boot_status = 0x%x\n", __func__, fw_status);

	sec_ts_delay(10);

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_ID, id, 3);
	if (ret < 0) {
		VTE("%s: read id fail\n", __func__);
		return -EIO;
	}

	ts->boot_ver[0] = id[0];
	ts->boot_ver[1] = id[1];
	ts->boot_ver[2] = id[2];

	ts->flash_page_size = SEC_TS_FW_BLK_SIZE_DEFAULT;

	VTI("%s: read_boot_id = %02X%02X%02X\n", __func__, id[0], id[1], id[2]);

	return 0;
}

static int sec_ts_sw_reset(struct sec_ts_data *ts)
{
	int ret;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);
	if (ret < 0) {
		VTE("%s: write fail, sw_reset\n", __func__);
		return 0;
	}

	sec_ts_delay(100);

	ret = sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	if (ret < 0) {
		VTE("%s: time out\n", __func__);
		return 0;
	}

	VTI("%s: sw_reset\n", __func__);

	/* Sense_on */
	ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		VTE("%s: write fail, Sense_on\n", __func__);
		return 0;
	}

	return ret;
}

void sec_ts_save_version_of_bin(struct sec_ts_data *ts, const fw_header *fw_hd)
{
	ts->plat_data->img_version_of_bin[3] = ((fw_hd->img_ver >> 24) & 0xff);
	ts->plat_data->img_version_of_bin[2] = ((fw_hd->img_ver >> 16) & 0xff);
	ts->plat_data->img_version_of_bin[1] = ((fw_hd->img_ver >> 8) & 0xff);
	ts->plat_data->img_version_of_bin[0] = ((fw_hd->img_ver >> 0) & 0xff);

	ts->plat_data->core_version_of_bin[3] = ((fw_hd->fw_ver >> 24) & 0xff);
	ts->plat_data->core_version_of_bin[2] = ((fw_hd->fw_ver >> 16) & 0xff);
	ts->plat_data->core_version_of_bin[1] = ((fw_hd->fw_ver >> 8) & 0xff);
	ts->plat_data->core_version_of_bin[0] = ((fw_hd->fw_ver >> 0) & 0xff);

	ts->plat_data->config_version_of_bin[3] = ((fw_hd->para_ver >> 24) & 0xff);
	ts->plat_data->config_version_of_bin[2] = ((fw_hd->para_ver >> 16) & 0xff);
	ts->plat_data->config_version_of_bin[1] = ((fw_hd->para_ver >> 8) & 0xff);
	ts->plat_data->config_version_of_bin[0] = ((fw_hd->para_ver >> 0) & 0xff);

	VTI("%s: img_ver of bin = %x.%x.%x.%x\n", __func__,
			ts->plat_data->img_version_of_bin[0],
			ts->plat_data->img_version_of_bin[1],
			ts->plat_data->img_version_of_bin[2],
			ts->plat_data->img_version_of_bin[3]);

	VTI("%s: core_ver of bin = %x.%x.%x.%x\n", __func__,
			ts->plat_data->core_version_of_bin[0],
			ts->plat_data->core_version_of_bin[1],
			ts->plat_data->core_version_of_bin[2],
			ts->plat_data->core_version_of_bin[3]);

	VTI("%s: config_ver of bin = %x.%x.%x.%x\n", __func__,
			ts->plat_data->config_version_of_bin[0],
			ts->plat_data->config_version_of_bin[1],
			ts->plat_data->config_version_of_bin[2],
			ts->plat_data->config_version_of_bin[3]);
}

int sec_ts_save_version_of_ic(struct sec_ts_data *ts)
{
	u8 img_ver[4] = {0,};
	u8 core_ver[4] = {0,};
	u8 config_ver[4] = {0,};
	int ret = 0;

	/* Image ver */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_IMG_VERSION, img_ver, 4);
	if (ret < 0) {
		VTE("%s: Image version read error\n", __func__);
		goto exit;
	}
	VTI("%s: IC Image version info : %x.%x.%x.%x\n",
			__func__, img_ver[0], img_ver[1], img_ver[2], img_ver[3]);

	ts->plat_data->img_version_of_ic[0] = img_ver[0];
	ts->plat_data->img_version_of_ic[1] = img_ver[1];
	ts->plat_data->img_version_of_ic[2] = img_ver[2];
	ts->plat_data->img_version_of_ic[3] = img_ver[3];

	/* Core ver */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_FW_VERSION, core_ver, 4);
	if (ret < 0) {
		VTE("%s: core version read error\n", __func__);
		goto exit;
	}
	VTI("%s: IC Core version info : %x.%x.%x.%x,\n",
			__func__, core_ver[0], core_ver[1], core_ver[2], core_ver[3]);

	ts->plat_data->core_version_of_ic[0] = core_ver[0];
	ts->plat_data->core_version_of_ic[1] = core_ver[1];
	ts->plat_data->core_version_of_ic[2] = core_ver[2];
	ts->plat_data->core_version_of_ic[3] = core_ver[3];

	/* Config ver */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, config_ver, 4);
	if (ret < 0) {
		VTE("%s: config version read error\n", __func__);
		goto exit;
	}
	VTI("%s: IC config version info : %x.%x.%x.%x\n",
			__func__, config_ver[0], config_ver[1], config_ver[2], config_ver[3]);

	ts->plat_data->config_version_of_ic[0] = config_ver[0];
	ts->plat_data->config_version_of_ic[1] = config_ver[1];
	ts->plat_data->config_version_of_ic[2] = config_ver[2];
	ts->plat_data->config_version_of_ic[3] = config_ver[3];

exit:
	return ret;
}

int sec_ts_check_firmware_version(struct sec_ts_data *ts, const u8 *fw_info)
{
	fw_header *fw_hd;
	u8 buff[1];
	int i;
	int ret;
	/*
	 * sec_ts_check_firmware_version
	 * return value = 1 : firmware download needed,
	 * return value = 0 : skip firmware download
	 */

	fw_hd = (fw_header *)fw_info;

	sec_ts_save_version_of_bin(ts, fw_hd);

	/* firmware download if READ_BOOT_STATUS = 0x10 */
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, buff, 1);
	if (ret < 0) {
		VTE("%s: fail to read BootStatus\n", __func__);
		return -EIO;
	}

	if (buff[0] == SEC_TS_STATUS_BOOT_MODE) {
		VTE("%s: ReadBootStatus = 0x%x, Firmware download Start!\n",
				__func__, buff[0]);
		return 1;
	}

	ret = sec_ts_save_version_of_ic(ts);
	if (ret < 0) {
		VTE("%s: fail to read ic version\n", __func__);
		return -EIO;
	}

	/* check f/w version
	 * ver[0] : IC version
	 * ver[1] : Project version
	 */
	for (i = 0; i < 2; i++) {
		if (ts->plat_data->img_version_of_ic[i] != ts->plat_data->img_version_of_bin[i]) {
			if (ts->plat_data->bringup == 3) {
				VTE("%s: bringup. force update\n", __func__);
				return 1;
			}
			VTE("%s: do not matched version info\n", __func__);
			//return 0;
		}
	}

	if (ts->plat_data->img_version_of_ic[2] != ts->plat_data->img_version_of_bin[2])
		return 1;

	if (ts->plat_data->img_version_of_ic[3] != ts->plat_data->img_version_of_bin[3])
		return 1;

	return 0;
}

static u8 sec_ts_checksum(u8 *data, int offset, int size)
{
	int i;
	u8 checksum = 0;

	for (i = 0; i < size; i++)
		checksum += data[i + offset];

	return checksum;
}

static int sec_ts_flashpageerase(struct sec_ts_data *ts, u32 page_idx, u32 page_num)
{
	int ret;
	u8 tCmd[6];

	tCmd[0] = SEC_TS_CMD_FLASH_ERASE;
	tCmd[1] = (u8)((page_idx >> 8) & 0xFF);
	tCmd[2] = (u8)((page_idx >> 0) & 0xFF);
	tCmd[3] = (u8)((page_num >> 8) & 0xFF);
	tCmd[4] = (u8)((page_num >> 0) & 0xFF);
	tCmd[5] = sec_ts_checksum(tCmd, 1, 4);

	ret = ts->sec_ts_i2c_write_burst(ts, tCmd, 6);

	return ret;
}

static int sec_ts_flashpagewrite(struct sec_ts_data *ts, u32 page_idx, u8 *page_data)
{
	int ret;
	u8 tCmd[1 + 2 + SEC_TS_FW_BLK_SIZE_MAX + 1];
	int flash_page_size = (int)ts->flash_page_size;

	tCmd[0] = 0xD9;
	tCmd[1] = (u8)((page_idx >> 8) & 0xFF);
	tCmd[2] = (u8)((page_idx >> 0) & 0xFF);

	memcpy(&tCmd[3], page_data, flash_page_size);
	tCmd[1 + 2 + flash_page_size] = sec_ts_checksum(tCmd, 1, 2 + flash_page_size);

	ret = ts->sec_ts_i2c_write_burst(ts, tCmd, 1 + 2 + flash_page_size + 1);
	return ret;
}

static int sec_ts_limited_flashpagewrite(struct sec_ts_data *ts, u32 page_idx, u8 *page_data)
{
	int ret = 0;
	u8 *tCmd;
	u8 copy_data[3 + SEC_TS_FW_BLK_SIZE_MAX];
	int copy_left = (int)ts->flash_page_size + 3;
	int copy_size = 0;
	int copy_max = ts->i2c_burstmax - 1;
	int flash_page_size = (int)ts->flash_page_size;

	copy_data[0] = (u8)((page_idx >> 8) & 0xFF);	/* addH */
	copy_data[1] = (u8)((page_idx >> 0) & 0xFF);	/* addL */

	memcpy(&copy_data[2], page_data, flash_page_size);	/* DATA */
	copy_data[2 + flash_page_size] = sec_ts_checksum(copy_data, 0, 2 + flash_page_size);	/* CS */
	tCmd = kzalloc(copy_max + 1, GFP_KERNEL);
	if (!tCmd)
		goto err_write;
	
	while (copy_left > 0) {
		int copy_cur = (copy_left > copy_max) ? copy_max : copy_left;

		//tCmd = kzalloc(copy_cur + 1, GFP_KERNEL);
		//if (!tCmd)
		//	goto err_write;
		
		memset(tCmd, 0, copy_max + 1);
		if (copy_size == 0)
			tCmd[0] = SEC_TS_CMD_FLASH_WRITE;
		else
			tCmd[0] = SEC_TS_CMD_FLASH_PADDING;

		memcpy(&tCmd[1], &copy_data[copy_size], copy_cur);

		ret = ts->sec_ts_i2c_write_burst(ts, tCmd, 1 + copy_cur);
		if (ret < 0)
			VTE("%s: failed, ret:%d\n", __func__, ret);

		copy_size += copy_cur;
		copy_left -= copy_cur;
		//kfree(tCmd);
	}
	kfree(tCmd);
	return ret;

err_write:
	VTE("%s: failed to alloc.\n", __func__);
	return -ENOMEM;

}

static int sec_ts_flashwrite(struct sec_ts_data *ts, u32 mem_addr, u8 *mem_data, u32 mem_size, int retry)
{
	int ret;
	u32 page_idx;
	u32 size_copy;
	u32 flash_page_size;
	u32 page_idx_start;
	u32 page_idx_end;
	u32 page_num;
	u8 page_buf[SEC_TS_FW_BLK_SIZE_MAX];

	if (mem_size == 0)
		return 0;
	
	flash_page_size = ts->flash_page_size;
	page_idx_start = mem_addr / flash_page_size;
	page_idx_end = (mem_addr + mem_size - 1) / flash_page_size;
	page_num = page_idx_end - page_idx_start + 1;

	ret = sec_ts_flashpageerase(ts, page_idx_start, page_num);
	if (ret < 0) {
		VTE("%s: fw erase failed, mem_addr= %08X, pagenum = %d\n",
				__func__, mem_addr, page_num);
		return -EIO;
	}

	sec_ts_delay(((page_num / 4) + 3) * 2);

	size_copy = mem_size % flash_page_size;
	if (size_copy == 0)
		size_copy = flash_page_size;

	memset(page_buf, 0, flash_page_size);

	for (page_idx = page_num - 1;; page_idx--) {
		memcpy(page_buf, mem_data + (page_idx * flash_page_size), size_copy);
		if (ts->boot_ver[0] == 0xB2) {
			ret = sec_ts_flashpagewrite(ts, (page_idx + page_idx_start), page_buf);
			if (ret < 0) {
				VTE("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
				goto err;
			}

			if (retry) {
				sec_ts_delay(50);
				ret = sec_ts_flashpagewrite(ts, (page_idx + page_idx_start), page_buf);
				if (ret < 0) {
					VTE("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
					goto err;
				}
			}
		} else {
			ret = sec_ts_limited_flashpagewrite(ts, (page_idx + page_idx_start), page_buf);
			if (ret < 0) {
				VTE("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
				goto err;
			}

			if (retry) {
				sec_ts_delay(50);
				ret = sec_ts_limited_flashpagewrite(ts, (page_idx + page_idx_start), page_buf);
				if (ret < 0) {
					VTE("%s: fw write failed, page_idx = %u\n", __func__, page_idx);
					goto err;
				}
			}

		}

		size_copy = flash_page_size;
		sec_ts_delay(5);

		if (page_idx == 0) /* end condition (page_idx >= 0)   page_idx type unsinged int */
			break;
	}

	return mem_size;
err:
	return -EIO;
}

static int sec_ts_memoryblockread(struct sec_ts_data *ts, u32 mem_addr, int mem_size, u8 *buf)
{
	int ret;
	u8 cmd[5];
	u8 *data;

	if (mem_size >= 64 * 1024) {
		VTE("%s: mem size over 64K\n", __func__);
		return -EIO;
	}

	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_ADDR;
	cmd[1] = (u8)((mem_addr >> 24) & 0xff);
	cmd[2] = (u8)((mem_addr >> 16) & 0xff);
	cmd[3] = (u8)((mem_addr >> 8) & 0xff);
	cmd[4] = (u8)((mem_addr >> 0) & 0xff);

	ret = ts->sec_ts_i2c_write_burst(ts, cmd, 5);
	if (ret < 0) {
		VTE("%s: send command failed, %02X\n", __func__, cmd[0]);
		return -EIO;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_SIZE;
	cmd[1] = (u8)((mem_size >> 8) & 0xff);
	cmd[2] = (u8)((mem_size >> 0) & 0xff);

	ret = ts->sec_ts_i2c_write_burst(ts, cmd, 3);
	if (ret < 0) {
		VTE("%s: send command failed, %02X\n", __func__, cmd[0]);
		return -EIO;
	}

	udelay(10);
	cmd[0] = (u8)SEC_TS_CMD_FLASH_READ_DATA;

	data = buf;


	ret = ts->sec_ts_i2c_read(ts, cmd[0], data, mem_size);
	if (ret < 0) {
		VTE("%s: memory read failed\n", __func__);
		return -EIO;
	}
#if 0
	ret = ts->sec_ts_i2c_write(ts, cmd[0], NULL, 0);
	ret = ts->sec_ts_i2c_read_bulk(ts, data, mem_size);
#endif
	return 0;
}

static int sec_ts_memoryread(struct sec_ts_data *ts, u32 mem_addr, u8 *mem_data, u32 mem_size)
{
	int ret;
	int retry = 3;
	int read_size = 0;
	int unit_size;
	int max_size = 1024;
	int read_left = (int)mem_size;
	u8 *tmp_data;

	tmp_data = kmalloc(max_size, GFP_KERNEL);
	if (!tmp_data) {
		VTE("%s: failed to kmalloc\n", __func__);
		return -ENOMEM;
	}

	while (read_left > 0) {
		unit_size = (read_left > max_size) ? max_size : read_left;
		retry = 3;
		do {
			ret = sec_ts_memoryblockread(ts, mem_addr, unit_size, tmp_data);
			if (retry-- == 0) {
				VTE("%s: fw read fail mem_addr=%08X,unit_size=%d\n",
						__func__, mem_addr, unit_size);
				kfree(tmp_data);
				return -1;
			}

			memcpy(mem_data + read_size, tmp_data, unit_size);
		} while (ret < 0);

		mem_addr += unit_size;
		read_size += unit_size;
		read_left -= unit_size;
	}

	kfree(tmp_data);

	return read_size;
}

static int sec_ts_chunk_update(struct sec_ts_data *ts, u32 addr, u32 size, u8 chunk_count, u8 *data, int retry)
{
	u32 fw_size;
	u32 write_size;
	u8 *mem_rb = NULL;
	int ret = 0;

	fw_size = size;

	write_size = sec_ts_flashwrite(ts, addr, data, fw_size, retry);
	if (write_size != fw_size) {
		VTE("%s: fw write failed\n", __func__);
		ret = -1;
		goto err_write_fail;
	}
	if (chunk_count != 0) {
		mem_rb = vzalloc(fw_size);
		if (!mem_rb) {
			VTE("%s: vzalloc failed\n", __func__);
			ret = -1;
			goto err_write_fail;
		}

		if (sec_ts_memoryread(ts, addr, mem_rb, fw_size) >= 0) {
			u32 ii;

			for (ii = 0; ii < fw_size; ii++) {
				if (data[ii] != mem_rb[ii])
					break;
			}

			if (fw_size != ii) {
				VTE("%s: fw verify fail\n", __func__);
				ret = -1;
				goto out;
			}
		} else {
			ret = -1;
			goto out;
		}
	} else {
		VTI("%s, verify skip %d", __func__, chunk_count);
	}

	VTI("%s: verify done(%d)\n", __func__, ret);

out:
	vfree(mem_rb);
err_write_fail:
	sec_ts_delay(10);

	return ret;
}

static int sec_ts_force_calibration_after_update(struct sec_ts_data *ts);

int sec_ts_firmware_update(struct sec_ts_data *ts, const u8 *data,
						size_t size, int bl_update, int restore_cal, int retry)
{
	int i;
	int ret;
	fw_header *fw_hd;
	fw_chunk *fw_ch;
	u8 fw_status = 0;
	u8 *fd = (u8 *)data;
	int retry_time = 0;
	u8 tBuff[8];
	u8 chunk_count = 0;

	/* Check whether CRC is appended or not.
	 * Enter Firmware Update Mode
	 */
	if (sec_ts_enter_fw_mode(ts) < 0) {
		VTE("%s: firmware mode failed\n", __func__);
		return -1;
	}

	if (bl_update && (ts->boot_ver[0] == 0xB4)) {
		VTI("%s: bootloader is up to date\n", __func__);
		return 0;
	}

	VTI("%s: firmware update retry :%d\n", __func__, retry);

	fw_hd = (fw_header *)fd;
	fd += sizeof(fw_header);

	if (fw_hd->signature != SEC_TS_FW_HEADER_SIGN) {
		VTE("%s: firmware header error = %08X\n", __func__, fw_hd->signature);
		return -1;
	}

	VTI("%s: num_chunk : %d\n", __func__, fw_hd->num_chunk);

	for (i = 0; i < fw_hd->num_chunk; i++) {
		fw_ch = (fw_chunk *)fd;
		chunk_count = i;

		VTI("%s: [%d] 0x%08X, 0x%08X, 0x%08X, 0x%08X\n", __func__, i,
				fw_ch->signature, fw_ch->addr, fw_ch->size, fw_ch->reserved);

		if (fw_ch->signature != SEC_TS_FW_CHUNK_SIGN) {
			VTE("%s: firmware chunk error = %08X\n", __func__, fw_ch->signature);
			return -1;
		}
		fd += sizeof(fw_chunk);
		ret = sec_ts_chunk_update(ts, fw_ch->addr, fw_ch->size, chunk_count, fd, retry);
		if (ret < 0) {
			VTE("%s: firmware chunk write failed, addr=%08X, size = %d\n", __func__, fw_ch->addr, fw_ch->size);
			return -1;
		}
		fd += fw_ch->size;
	}

	sec_ts_sw_reset(ts);

	if (!bl_update) {
		if (restore_cal) {
			VTE("%s: RUN OFFSET CALIBRATION\n", __func__);

			ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SET);
			if (ret < 0)
				VTE("%s: fail to write OFFSET CAL SET!\n", __func__);
		}

		/* calibraion version compare and calibrate */
		while (ts->sec_ts_i2c_read(ts, SEC_TS_READ_PARA_VERSION, tBuff, 4) < 0) {
			retry_time++;
			if (retry_time >= 3) {
				VTI("ic read version failed");
				break;
			}
		}
		if (retry_time < 3) {
			retry_time = 0;
			VTI("after firmware update, the calibration version is %d,%d", tBuff[2], tBuff[3]);
			if (((ts->ic_info[6] != tBuff[2] || ts->ic_info[7] != tBuff[3]) && (ts->ic_info[7] != 0xff))
				|| (ts->ic_info[7] == 0xff && vts_factory_mode_get() == 1)) {
				while (sec_ts_force_calibration_after_update(ts) < 0) {
					retry_time++;
					if (retry_time >= 3) {
						VTE("calibration still fail after 3 times");
						break;
					}
				}
			} else if (ts->ic_info[7] == 0xff && vts_factory_mode_get() == 0) {
				VTE("no factory mode unexpect calibrate version");
			} else {
				VTI("no need to calibrate");
			}
		}

		/* Sense_on */
		ret = ts->sec_ts_i2c_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
		if (ret < 0) {
			VTE("%s: write fail, Sense_on\n", __func__);
			return -EIO;
		}

		if (ts->sec_ts_i2c_read(ts, SEC_TS_READ_BOOT_STATUS, &fw_status, 1) < 0) {
			VTE("%s: read fail, read_boot_status = 0x%x\n", __func__, fw_status);
			return -EIO;
		}

		if (fw_status != SEC_TS_STATUS_APP_MODE) {
			VTE("%s: fw update sequence done, BUT read_boot_status = 0x%x\n", __func__, fw_status);
			return -EIO;
		}

		VTI("%s: fw update Success! read_boot_status = 0x%x\n", __func__, fw_status);

		return 1;
	} else {

		if (ts->sec_ts_i2c_read(ts, SEC_TS_READ_ID, tBuff, 3) < 0) {
			VTE("%s: read device id fail after bl fw download\n", __func__);
			return -EIO;
		}

		if (tBuff[0] == 0xA0) {
			VTI("%s: bl fw download success - device id = %02X\n", __func__, tBuff[0]);
			return 0;
		} else {
			VTE("%s: bl fw id does not match - device id = %02X\n", __func__, tBuff[0]);
			return -EIO;
		}
	}

}

int sec_ts_firmware_update_bl(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int result = -1;

	disable_irq(ts->client->irq);

	snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", SEC_TS_DEFAULT_BL_NAME);

	VTI("%s: initial bl update %s\n", __func__, fw_path);

	/* Loading Firmware------------------------------------------ */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTE("%s: bt is not available\n", __func__);
		goto err_request_fw;
	}
	VTI("%s: request bt done! size = %d\n", __func__, (int)fw_entry->size);

	result = sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 1, false, 0);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);

	return result;
}

int sec_ts_bl_update(struct sec_ts_data *ts)
{
	int ret;
	u8 tCmd[5] = { 0xDE, 0xAD, 0xBE, 0xEF };
	u8 tBuff[3];
	int count = 0;

	ret = ts->sec_ts_i2c_write(ts, SEC_TS_READ_BL_UPDATE_STATUS, tCmd, 4);
	if (ret < 0) {
		VTE("%s: bl update command send fail!\n", __func__);
		goto err;
	}
	sec_ts_delay(10);

	do {
		ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_BL_UPDATE_STATUS, tBuff, 1);
		if (ret < 0) {
			VTE("%s: read bl update status fail!\n", __func__);
			goto err;
		}
		sec_ts_delay(2);
		count++;

	} while (tBuff[0] == 0x1 && count < 100);

	tCmd[0] = 0x55;
	tCmd[1] = 0xAC;
	ret = ts->sec_ts_i2c_write(ts, 0x57, tCmd, 2);
	if (ret < 0) {
		VTE("%s: write passwd fail!\n", __func__);
		goto err;
	}

	ret = ts->sec_ts_i2c_read(ts, SEC_TS_READ_ID, tBuff, 3);

	if (tBuff[0]  == 0xB4) {
		VTI("%s: bl update completed!\n", __func__);
		ret = 1;
	} else {
		VTI("%s: bl updated but bl version not matching, ver=%02X\n", __func__, tBuff[0]);
		goto err;
	}

	return ret;
err:
	return -EIO;
}

static int sec_ts_force_calibration_on_probe(struct sec_ts_data *ts);

int sec_ts_firmware_update_on_probe(struct sec_ts_data *ts, bool force_update)
{
	int result = 0;
	int retry = 0;

	if (ts->plat_data->bringup == 1) {
		VTE("%s: bringup. do not update\n", __func__);
		return 0;
	}

	disable_irq(ts->client->irq);

	sec_ts_save_version_of_ic(ts);

	while((result = sec_ts_force_calibration_on_probe(ts)) < 0) {
		retry++;
		if (retry >= 3) {
			VTI("calibration still fail after 3 times");
			enable_irq(ts->client->irq);
			return result;
		}
	}
	enable_irq(ts->client->irq);
	return result;
}

static int sec_ts_load_fw_from_bin(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	char fw_path[SEC_TS_MAX_FW_PATH];
	int error = 0;
	int restore_cal = 0;

	if (ts->client->irq)
		disable_irq(ts->client->irq);

	if (!ts->plat_data->firmware_name)
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", SEC_TS_DEFAULT_FW_NAME);
	else
		snprintf(fw_path, SEC_TS_MAX_FW_PATH, "%s", ts->plat_data->firmware_name);

	VTI("%s: initial firmware update  %s\n", __func__, fw_path);

	/* Loading Firmware */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTE("%s: firmware is not available\n", __func__);
		error = -1;
		goto err_request_fw;
	}
	VTI("%s: request firmware done! size = %d\n", __func__, (int)fw_entry->size);

	/* use virtual pat_control - magic cal 1 */
	if (sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 0, restore_cal, 0) < 0)
		error = -1;
	else
		error = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	release_firmware(fw_entry);
	if (ts->client->irq)
		enable_irq(ts->client->irq);

	return error;
}

static int sec_ts_load_fw_from_ums(struct sec_ts_data *ts)
{
#if 0
	fw_header *fw_hd;
	struct file *fp;
	long fw_size, nread;
	int error = 0;
	int restore_cal = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	mm_segment_t old_fs;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
#endif
	fp = filp_open(SEC_TS_DEFAULT_UMS_FW, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		VTE("%s: failed to open %s.\n", __func__,
				SEC_TS_DEFAULT_UMS_FW);
		error = -ENOENT;
		goto open_err;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;

	if (fw_size > 0) {
		unsigned char *fw_data;

		fw_data = vzalloc(fw_size);
		if (!fw_data) {
			VTE("%s: failed to alloc mem\n", __func__);
			error = -ENOMEM;
			filp_close(fp, NULL);
			goto open_err;
		}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		nread = vfs_read(fp, (char __user *)fw_data,
				fw_size, &fp->f_pos);
#else
		nread = kernel_read(fp, (char __user *)fw_data,
				fw_size, &fp->f_pos);	
#endif
		VTI("%s: start, file path %s, size %ld Bytes\n",
				__func__, SEC_TS_DEFAULT_UMS_FW, fw_size);

		if (nread != fw_size) {
			VTE("%s: failed to read firmware file, nread %ld Bytes\n",
					__func__, nread);
			error = -EIO;
		} else {
			fw_hd = (fw_header *)fw_data;
#if 0
			sec_ts_check_firmware_version(ts, fw_data);
#endif
			VTI("%s: firmware version %08X\n", __func__, fw_hd->fw_ver);
			VTI("%s: parameter version %08X\n", __func__, fw_hd->para_ver);

			if (ts->client->irq)
				disable_irq(ts->client->irq);
			/* use virtual pat_control - magic cal 1 */
			if (sec_ts_firmware_update(ts, fw_data, fw_size, 0, restore_cal, 0) < 0) {
				error = -1; /* firmware failed */
				goto done;
			}

			sec_ts_save_version_of_ic(ts);
		}

		if (error < 0)
			VTE("%s: failed update firmware\n",
					__func__);

done:
		if (ts->client->irq)
			enable_irq(ts->client->irq);
		vfree(fw_data);
	}

	filp_close(fp, NULL);

open_err:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	set_fs(old_fs);
#endif
	return error;
#else
	return 0;
#endif
}

static int sec_ts_load_fw_from_ffu(struct sec_ts_data *ts)
{
	const struct firmware *fw_entry;
	const char *fw_path = SEC_TS_DEFAULT_FFU_FW;
	int result = -1;

	disable_irq(ts->client->irq);

	VTI("%s: Load firmware : %s\n", __func__, fw_path);

	/* Loading Firmware */
	if (request_firmware(&fw_entry, fw_path, &ts->client->dev) !=  0) {
		VTE("%s: firmware is not available\n", __func__);
		goto err_request_fw;
	}
	VTI("%s: request firmware done! size = %d\n", __func__, (int)fw_entry->size);

	sec_ts_check_firmware_version(ts, fw_entry->data);

	if (sec_ts_firmware_update(ts, fw_entry->data, fw_entry->size, 0, false, 0) < 0)
		result = -1;
	else
		result = 0;

	sec_ts_save_version_of_ic(ts);

err_request_fw:
	release_firmware(fw_entry);
	enable_irq(ts->client->irq);
	return result;
}

extern int bbk_slsi_erase_vivo_cal(struct sec_ts_data *info, u8 val);
extern int bbk_slsi_erase_vivo_specail_cal(struct sec_ts_data *info, u8 val);
extern int bbk_slsi_get_vivo_calibration_status(struct sec_ts_data *info, int *cali_satus);
extern int bbk_slsi_get_vivo_special_calibration_status(struct sec_ts_data *info, int *cali_satus);
extern int bbk_slsi_start_force_calibration(struct sec_ts_data *info);
extern int bbk_slsi_get_vsync_freq(struct sec_ts_data *info, int *vsync_freq);

int sec_ts_get_special_cal_status_on_probe(struct vts_device *vtsdev)
{
	int cali_special = 0;
	u32 calibration_twice_set = 0;
	int ret = 0;
	struct sec_ts_data *ts = vts_get_drvdata(vtsdev);
	vts_property_get(vtsdev, VTS_PROPERTY_CALIBRATION_TWICE, &calibration_twice_set);
	if (calibration_twice_set) {
	
		ret = bbk_slsi_get_vivo_special_calibration_status(ts, &cali_special);
		if (ret < 0) {
			VTI("read 0x1E reg fail");
			return -EPERM;
		}
	}
	return ret;
}

static int sec_ts_force_calibration_on_probe(struct sec_ts_data *ts)
{
	int cali_info = 0;
	int ret = 0;
	
	ret = bbk_slsi_get_vivo_calibration_status(ts, &cali_info);
	if (ret < 0) {
		VTI("get calibration status fail");
		goto END;
	}
	/*0xA5 is for compatible the old software version*/
	if (cali_info != 0xB1 && cali_info != 0xA1) {
		VTI("The chip has not been force calibration or probe calibration before this boot!!!!!!");
		ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SET);
		if (ret < 0) {
			VTI("fail to write OFFSET CAL SEC!");
			msleep(200);
			bbk_slsi_erase_vivo_cal(ts, 0x00);
			msleep(200);
			goto END;
		}else{
			msleep(200);
			bbk_slsi_erase_vivo_cal(ts, 0xB1);
			msleep(200);
			ret = bbk_slsi_get_vivo_calibration_status(ts, &cali_info);
			if ((ret < 0) || (cali_info != 0xB1)) {
				ret = -1;
				VTI("bbk_slsi_erase_vivo_cal 0xB1 fail");
				goto END;
			}
			VTI("force calibration on probe successful!!!");
		}
	} else {
		VTI("chip has been calibration before!");
	}
END:
	return ret;

}

static int sec_ts_force_calibration_after_update(struct sec_ts_data *ts)
{
	int cali_info = 0;
	int ret = 0;
	u8 tBuff[8] = {0};
	u8 cal_check = 0;
	u8 cal_value = 0;
	
	ret = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SET);
	if (ret < 0) {
		VTI("fail to write OFFSET CAL SEC!");
		msleep(200);
		bbk_slsi_erase_vivo_cal(ts, 0x00);
		msleep(200);
		goto END;
	} else {
		msleep(200);
		bbk_slsi_erase_vivo_cal(ts, 0xB1);
		msleep(200);
		ret = bbk_slsi_get_vivo_calibration_status(ts, &cali_info);
		if ((ret < 0) || (cali_info != 0xB1)) {
			ret = -1;
			VTI("bbk_slsi_erase_vivo_cal 0xB1 fail");
			goto END;
		}
		VTI("force calibration after update successful!!!");
	}

	ts->sec_ts_i2c_read(ts, SEC_TS_READ_CALIBRATION_REPORT, tBuff, 8);
	memcpy(ts->ic_info, tBuff, 8);
	VTI("after firmware update, read F1 is %d, %d, %d, %d, %d, %d, %d, %d", ts->ic_info[0], ts->ic_info[1], ts->ic_info[2], ts->ic_info[3],
			ts->ic_info[4], ts->ic_info[5], ts->ic_info[6], ts->ic_info[7]);

	ts->sec_ts_i2c_write(ts, SEC_TS_CMD_MIS_CAL_CHECK, &cal_check, 1);
	ret = ts->sec_ts_i2c_read(ts, SEC_TS_CMD_MIS_CAL_READ, &cal_value, 1);
	VTI("cal result from 0xA8 %d", cal_value);
	if (cal_value != 0 || ts->ic_info[3] == 0x50) 
		ret = -1;						

END:
	return ret;
}

/*
int samsung_at_sensor_test(struct vts_device *vtsdev, char *buf, int at_sensor_test_cmd , void *pdata, int tmp)
{

	int count = 0, ret = 0;
	int cali_satus = 0;

	struct sec_ts_data *info = vts_get_drvdata(vtsdev);
	vts_report_release(info->vtsdev);
	//disable_irq(info->client->irq);
	
	if (VTS_SENSOR_TEST_CALIBRATION == at_sensor_test_cmd) {//calibrate
		//show tag
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return 0;
		}

		//erase tag
		VTI("erase cali tag");
		ret = bbk_slsi_erase_vivo_cal(0);
		msleep(200);
		if (ret < 0) {
			VTI("erase tag fail");
			return 0;
		}

		//show tag
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return 0;
		}

		msleep(100);
		//cali
		ret = bbk_slsi_start_force_calibration();
		if (ret < 0) {
			VTI("cali fail");
			msleep(200);
			bbk_slsi_erase_vivo_cal(0x00);
			msleep(200);
			return 0;
		} else {
			msleep(200);
			bbk_slsi_erase_vivo_cal(0xA1);
		}

		msleep(200);
		//show tag
		ret = bbk_slsi_get_vivo_calibration_status(&cali_satus);
		if (ret >= 0 && (cali_satus == 0xa1)) {
			count = snprintf(buf, 1024, "Pass\nCalibration pass.\n");
			VTI("Calibration pass.");
		} else {
			count = snprintf(buf, 1024, "Failed\nCalibration fail.\n");
			VTI("Calibration fail.");
		}
	} else if (VTS_SENSOR_TEST_CHECK == at_sensor_test_cmd) {
			VTI("MP Flag not set!");
	} else {
		VTI("no cmd ");
	}

	sec_ts_sw_reset(g_ts_data);

	VTI("at_sensor_test result:%s\n", buf);
	at_sensor_test_cmd = 0;
	vts_report_release(info->vtsdev);
	//enable_irq(info->client->irq);

	VTI("count = %d", count);
	return count;
}
*/
/*
int samsung_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result) {
	VTI("touch screen test is done by apk");
	return 0;
}
*/

int samsung_sensor_caliberate(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result) {

	int count = 0, ret = 0;
	int cali_satus = 0;	
	u32 calibration_twice_set = 0;
	int vsync_freq = 0;
	int i = 0, out_range_cnt = 0;

	struct sec_ts_data *info = vts_get_drvdata(vtsdev);
	vts_property_get(vtsdev, VTS_PROPERTY_CALIBRATION_TWICE, &calibration_twice_set);

	*result = VTS_SENSOR_CALIBERATE_FAILED;
	vts_report_release(info->vtsdev);
	//disable_irq(info->client->irq);

	VTI("calibration_twice_set is %d!!!!!", calibration_twice_set);
	
	if (code == 165 || code == 167) {
		VTI("highest refresh rate calibration!!!");
		//disable_irq(info->client->irq);

		if (calibration_twice_set == 1) {
			ret = bbk_slsi_get_vsync_freq(info, &vsync_freq);
			if (ret < 0) {
				VTE("get vsync from TP failed");
				return -EPERM;
			}
			
			for (i = 0; i < vtsdev->module->highestCali_cnt; i++) {
				if (vsync_freq < vtsdev->module->highest_calibration[i] - 10 || vsync_freq > vtsdev->module->highest_calibration[i] + 10)
					out_range_cnt++;
			}
			
			if(out_range_cnt == vtsdev->module->highestCali_cnt) {
				VTI("vsync from TP IC is not equal to highest refresh rate, do not calibration");
				return -EPERM;
			}
			
		}
		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(info, &cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return -EPERM;
		}

		/*erase tag*/
		VTI("erase cali tag");
		ret = bbk_slsi_erase_vivo_cal(info, 0);
		msleep(200);
		if (ret < 0) {
			VTI("erase tag fail");
			return -EPERM;
		}

		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(info, &cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return -EPERM;
		}

		msleep(100);
		/*cali*/
		ret = bbk_slsi_start_force_calibration(info);
		if (ret < 0) {
			VTE("cali fail");
			msleep(200);
			bbk_slsi_erase_vivo_cal(info, 0x00);
			msleep(200);
			return -EPERM;
		} else {
			msleep(200);
			bbk_slsi_erase_vivo_cal(info, 0xA1);
		}

		msleep(200);
		/*show tag*/
		ret = bbk_slsi_get_vivo_calibration_status(info, &cali_satus);
		if (ret >= 0 && (cali_satus == 0xa1)) {
			//count = snprintf(buf, 1024, "Pass\nCalibration pass.\n");
			*result = VTS_SENSOR_CALIBERATE_SUCCESS;
			VTI("Calibration pass.");
		} else {
			//count = snprintf(buf, 1024, "Failed\nCalibration fail.\n");
			*result = VTS_SENSOR_CALIBERATE_FAILED;
			VTI("Calibration fail.");
		}
		sec_ts_sw_reset(info);
	}	else if (code == 169){
		VTI("other calibration!!!");

		if (calibration_twice_set != 1) {
			VTI("not set vts-calibration-twice, do not allow 169 calibration");
			return -EPERM;
		}

		ret = bbk_slsi_get_vsync_freq(info, &vsync_freq);
		if (ret < 0) {
			VTE("get vsync from TP failed");
			return -EPERM;
		}
		
		for (i = 0; i < vtsdev->module->otherCali_cnt; i++) {
			if (vsync_freq < vtsdev->module->other_calibration[i] - 10 || vsync_freq > vtsdev->module->other_calibration[i] + 10)
				out_range_cnt++;
		}
		
		if(out_range_cnt == vtsdev->module->otherCali_cnt) {
			VTI("vsync from TP IC is not equal to other refresh rate, do not calibration");
			return -EPERM;
		}
		
	    /*show tag*/
		ret = bbk_slsi_get_vivo_special_calibration_status(info, &cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return -EPERM;
		}

		/*erase tag*/
		VTI("erase cali tag");
		ret = bbk_slsi_erase_vivo_specail_cal(info, 0);
		msleep(200);
		if (ret < 0) {
			VTI("erase tag fail");
			return -EPERM;
		}

		/*show tag*/
		ret = bbk_slsi_get_vivo_special_calibration_status(info, &cali_satus);
		if (ret < 0) {
			VTI("read tag fail");
			return -EPERM;
		}

		msleep(100);
		/*cali*/		
		ret = bbk_slsi_start_force_calibration(info);
		if (ret < 0) {
			VTE("cali fail");
			msleep(200);
			bbk_slsi_erase_vivo_specail_cal(info, 0x00);
			msleep(200);
			return -EPERM;
		} else {
			msleep(200);
			bbk_slsi_erase_vivo_specail_cal(info, 0xA1);
		}

		msleep(200);

		/*show tag*/
		ret = bbk_slsi_get_vivo_special_calibration_status(info, &cali_satus);
		if (ret >= 0 && (cali_satus == 0xa1)) {
			//count = snprintf(buf, 1024, "Pass\nCalibration pass.\n");
			*result = VTS_SENSOR_CALIBERATE_SUCCESS;
			VTI("special Calibration pass.");
		} else {
			//count = snprintf(buf, 1024, "Failed\nCalibration fail.\n");
			*result = VTS_SENSOR_CALIBERATE_FAILED;
			VTI("special Calibration fail.");
		}

	}

	sec_ts_sw_reset(info);

	//VTI("at_sensor_test result:%s\n", buf);
	vts_report_release(info->vtsdev);
	//enable_irq(info->client->irq);

	//VTI("count = %d", count);
	return count;	
}


int sec_ts_firmware_update_on_hidden_menu(struct sec_ts_data *ts, int update_type)
{
	int ret = 0;

	/* Factory cmd for firmware update
	 * argument represent what is source of firmware like below.
	 *
	 * 0 : [BUILT_IN] Getting firmware which is for user.
	 * 1 : [UMS] Getting firmware from sd card.
	 * 2 : none
	 * 3 : [FFU] Getting firmware from air.
	 */

	switch (update_type) {
	case BUILT_IN:
		ret = sec_ts_load_fw_from_bin(ts);
		break;
	case UMS:
		ret = sec_ts_load_fw_from_ums(ts);
		break;
	case FFU:
		ret = sec_ts_load_fw_from_ffu(ts);
		break;
	case BL:
		ret = sec_ts_firmware_update_bl(ts);
		if (ret < 0) {
			break;
		} else if (!ret) {
			ret = sec_ts_firmware_update_on_probe(ts, false);
			break;
		} else {
			ret = sec_ts_bl_update(ts);
			if (ret < 0)
				break;
			ret = sec_ts_firmware_update_on_probe(ts, false);
			if (ret < 0)
				break;
		}
		break;
	default:
		VTE("%s: Not support command[%d]\n",
				__func__, update_type);
		break;
	}

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	sec_ts_check_custom_library(ts);
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
#endif

	return ret;
}
EXPORT_SYMBOL(sec_ts_firmware_update_on_hidden_menu);
