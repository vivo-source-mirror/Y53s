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
* File Name: focaltech_flash.c
*
* Author: Focaltech Driver Team
*
* Created: 2017-12-06
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
static bool download_all = false;

/*****************************************************************************
* Static function
*****************************************************************************/

static int fts_check_bootid(void)
{
    int ret = 0;
    int i = 0;
    u8 cmd = 0;
    u8 id[2] = { 0 };
    struct ft_chip_t *chip_id = &fts_data->ic_info.ids;

    for (i = 0; i < FTS_READ_BOOT_ID_TIMEOUT; i++) {
        cmd = FTS_CMD_START1;
        ret = fts_write(&cmd, 1);
        if (ret < 0) {
            VTE("write cmd 0x55 fail");
            return ret;
        }

        mdelay(8);
        cmd = FTS_CMD_READ_ID;
        ret = fts_read(&cmd, 1, id, 2);
        if (ret < 0) {
            VTE("read boot id fail");
            return ret;
        }

        VTI("read boot id:0x%02x 0x%02x", id[0], id[1]);
        if ((chip_id->chip_idh == id[0]) && (chip_id->chip_idl == id[1])) {
            return 0;
        }
    }

    return -EIO;
}

static int fts_fwupg_hardware_reset_to_boot(void)
{
    if (fts_reset_proc_0flash(0) > 0)
		mdelay(8);

    return 0;
}

static int fts_enter_into_boot(void)
{
    int ret = 0;
    int i = 0;
	u8 cmd[6] = { 0xF2, 0x00, 0x78, 0x0A, 0x00, 0x02};
	u8 value = 0;
    u8 value1[2] = { 0 };

    VTI("enter into boot environment");
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /* hardware tp reset to boot */
       	fts_fwupg_hardware_reset_to_boot();

        /* check boot id */
        ret = fts_check_bootid();
        if (0 == ret) {
            VTI("boot id check pass, retry=%d", i);
			
			ret = fts_read_reg_byte(0xdb, &value);
            if (ret < 0) {
                VTE("read 0xdb fail");
                return ret;
            }
            ret = fts_read(cmd, 6, value1, 2);
            if (ret < 0) {
                VTE("read 0xF2 fail");
                return ret;
            }
            VTI("0xdb = 0x%x, 0xF2 = 0x%x", value, value1[0]);
            if(value == 0x18 && value1[0] == 0x55) {
                VTI("IC can download once time");
                download_all = true;           
            }
            return 0;
        }
    }

    return -EIO;
}

static int fts_pram_write(u32 saddr, const u8 *buf, u32 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u32 addr = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number;
    u32 packet_len = 0;
    u32 packet_size = FTS_FLASH_PACKET_LENGTH;
	struct fts_ts_data *ts_data = fts_data;

    VTI("pram write");
    if (NULL == buf) {
        VTE("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_APP)) {
        VTE("fw length(%d) fail", len);
        return -EINVAL;
    }

	if (download_all == true) {
        packet_size = 64 * 1024;
    }

	if (!ts_data->fw_buf) {
		ts_data->fw_buf = kzalloc(packet_size + FTS_CMD_WRITE_LEN, GFP_KERNEL);
		if (!ts_data->fw_buf) {
			VTE("allocate memory for fw_buf is failed!");
			return -ENOMEM;
		}
	}

    packet_number = len / packet_size;
    remainder = len % packet_size;
    if (remainder > 0)
        packet_number++;
    packet_len = packet_size;

    ts_data->fw_buf[0] = FTS_ROMBOOT_CMD_WRITE;
	VTI("packet_number = %d.", packet_number);
    for (i = 0; i < packet_number; i++) {
        offset = i * packet_size;
        addr = saddr + offset;
        ts_data->fw_buf[1] = BYTE_OFF_16(addr);
        ts_data->fw_buf[2] = BYTE_OFF_8(addr);
        ts_data->fw_buf[3] = BYTE_OFF_0(addr);

        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;
        ts_data->fw_buf[4] = BYTE_OFF_8(packet_len);
        ts_data->fw_buf[5] = BYTE_OFF_0(packet_len);

        for (j = 0; j < packet_len; j++) {
            ts_data->fw_buf[FTS_CMD_WRITE_LEN + j] = buf[offset + j];
        }
        ret = fts_write(ts_data->fw_buf, FTS_CMD_WRITE_LEN + packet_len);
        if (ret < 0) {
            VTE("write fw to pram(%d) fail", i);
            goto write_pram_err;
        }

        mdelay(3);
	}

write_pram_err:
    return ret;
}

static int fts_ecc_cal_tp(u32 ecc_saddr, u32 ecc_len, u16 *ecc_value)
{
    int ret = 0;
    int i = 0;
    u8 cmd[FTS_ROMBOOT_CMD_ECC_LEN] = { 0 };
    u8 value[2] = { 0 };

    VTI("ecc calc in tp");
    cmd[0] = FTS_ROMBOOT_CMD_ECC;
    cmd[1] = BYTE_OFF_16(ecc_saddr);
    cmd[2] = BYTE_OFF_8(ecc_saddr);
    cmd[3] = BYTE_OFF_0(ecc_saddr);
    cmd[4] = BYTE_OFF_16(ecc_len);
    cmd[5] = BYTE_OFF_8(ecc_len);
    cmd[6] = BYTE_OFF_0(ecc_len);

    /* make boot to calculate ecc in pram */
    ret = fts_write(cmd, FTS_ROMBOOT_CMD_ECC_LEN);
    if (ret < 0) {
        VTE("ecc calc cmd fail");
        return ret;
    }
    mdelay(3);

    /* wait boot calculate ecc finish */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        ret = fts_read(cmd, 1, value, 1);
        if (ret < 0) {
            VTE("ecc finish cmd fail");
            return ret;
        }
        if (0 == value[0])
            break;
        mdelay(1);
		VTI("waiting for romboot calculate ecc time = %d", i);
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        VTE("wait ecc finish timeout");
        return -EIO;
    }

    /* get ecc value calculate in boot */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
    ret = fts_read(cmd, 1, value, 2);
    if (ret < 0) {
        VTE("ecc read cmd fail");
        return ret;
    }

    *ecc_value = ((u16)(value[0] << 8) + value[1]) & 0x0000FFFF;
    return 0;
}

static int fts_ecc_cal_host(const u8 *data, u32 data_len, u16 *ecc_value)
{
    u16 ecc = 0;
    u16 i = 0;
    u16 j = 0;
    u16 al2_fcs_coef = AL2_FCS_COEF;

    for (i = 0; i < data_len; i += 2 ) {
        ecc ^= ((data[i] << 8) | (data[i + 1]));
        for (j = 0; j < 16; j ++) {
            if (ecc & 0x01)
                ecc = (u16)((ecc >> 1) ^ al2_fcs_coef);
            else
                ecc >>= 1;
        }
    }

    *ecc_value = ecc & 0x0000FFFF;
    return 0;
}

static int fts_pram_start(void)
{
    int ret = 0;
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;

    VTI("remap to start pram");
    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        VTE("write start pram cmd fail");
        return ret;
    }

    return 0;
}

/*
 * description: download fw to IC and run
 *
 * param - buf: const, fw data buffer
 *         len: length of fw
 *
 * return 0 if success, otherwise return error code
 */
static int fts_fw_write_start(const u8 *buf, u32 len)
{
    int ret = 0;
    u16 ecc_in_host = 0;
    u16 ecc_in_tp = 0;
    u32 fw_start_addr = 0;
	u32 fw_len = 0;
    u16 code_len = 0;
    u16 code_len_n = 0;

	/* get app length */
	VTI("begin to write and start fw(fwlen:%x)", len);
    code_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
               + buf[FTS_APP_INFO_OFFSET + 1];
    code_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 2] << 8)
                 + buf[FTS_APP_INFO_OFFSET + 3];
    if ((code_len + code_len_n) != 0xFFFF) {
        VTE("code len(%x %x) fail", code_len, code_len_n);
        return -EINVAL;
    }

    fw_len = (u32)code_len;
    if ((fw_len < FTS_MIN_LEN) || (fw_len > FTS_MAX_LEN_APP)) {
        VTE("fw length(%d) is invalid", fw_len);
        return -EINVAL;
    }
    VTI("fw length in fact:0x%x", fw_len);
    fts_data->fw_is_running = false;

    /* enter into boot environment */
    ret = fts_enter_into_boot();
    if (ret < 0) {
        VTE("enter into boot environment fail");
        return ret;
    }

    /* write pram */
    ret = fts_pram_write(fw_start_addr, buf, fw_len);
    if (ret < 0) {
        VTE("write pram fail");
        return ret;
    }

    /* ecc check */
    ret = fts_ecc_cal_host(buf, fw_len, &ecc_in_host);
    if (ret < 0) {
        VTE("ecc in host calc fail");
        return ret;
    }

    ret = fts_ecc_cal_tp(fw_start_addr, fw_len, &ecc_in_tp);
    if (ret < 0) {
        VTE("ecc in tp calc fail");
        return ret;
    }

    VTI("ecc in tp:%04x host:%04x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        VTE("ecc check fail");
        return -EIO;
    }

    /* remap pram and run fw */
    ret = fts_pram_start();
    if (ret < 0) {
        VTE("pram start fail");
        return ret;
    }

    fts_data->fw_is_running = true;
    VTI("fw download successfully");
    return 0;
}

static int fts_caltimediff(struct timeval *diff, struct timeval *start, struct timeval *end)   
{
	if (start->tv_sec > end->tv_sec) {
		return -EINVAL;
	}

	if ((start->tv_sec == end->tv_sec) && (start->tv_usec > end->tv_usec)) {
		return -EINVAL;
	}

	diff->tv_sec = end->tv_sec - start->tv_sec;
	diff->tv_usec = end->tv_usec - start->tv_usec;
	if (diff->tv_usec < 0) {
		diff->tv_sec--;
		diff->tv_usec += 1000000;
	}

	return 0;
} 

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
int fts_fw_download(const u8 *buf, u32 len)
{
    int ret = 0;
    int i = 0;
    struct fts_ts_data *ts_data = fts_data;
	struct timeval start, end;
	struct timeval diff = {0, };

	do_gettimeofday(&start);

    VTI("fw upgrade download function");
    if (NULL == buf) {
        VTE("fw buf is null");
        return -EINVAL;
    }

    if (len < FTS_MIN_LEN) {
        VTE("fw length(%d) is invalid", len);
        return -EINVAL;
    }

    ts_data->fw_loading = 1;
    fts_irq_disable_0flash();
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch_pd1831(DISABLE);
#endif

    for (i = 0; i < 3; i++) {
        VTI("fw download times:%d", i + 1);
        ret = fts_fw_write_start(buf, len);
        if (0 == ret)
            break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_switch_pd1831(ENABLE);
#endif
    fts_irq_enable_0flash();
    ts_data->fw_loading = 0;

    if (i >= 3) {
        VTE("fw download fail");
        return -EIO;
    }

	do_gettimeofday(&end);

	fts_caltimediff(&diff, &start, &end);
	VTI("Update firmware success! <%ld.%06ld>\n", diff.tv_sec, diff.tv_usec);

    return 0;
}

int fts_read_file_0flash(char *file_name, u8 **file_buf)
{
    int ret = 0;
    char file_path[FILE_NAME_LENGTH] = { 0 };
    struct file *filp = NULL;
    struct inode *inode;
    mm_segment_t old_fs;
    loff_t pos;
    loff_t file_len = 0;

    if ((NULL == file_name) || (NULL == file_buf)) {
        VTE("filename/filebuf is NULL");
        return -EINVAL;
    }

    snprintf(file_path, FILE_NAME_LENGTH, "%s%s", FTS_FW_BIN_FILEPATH, file_name);
    filp = filp_open(file_path, O_RDONLY, 0);
    if (IS_ERR(filp)) {
        VTE("open %s file fail", file_path);
        return -ENOENT;
    }

#if 1
    inode = filp->f_inode;
#else
    /* reserved for linux earlier verion */
    inode = filp->f_dentry->d_inode;
#endif

    file_len = inode->i_size;
    *file_buf = (u8 *)vmalloc(file_len);
    if (NULL == *file_buf) {
        VTE("file buf malloc fail");
        filp_close(filp, NULL);
        return -ENOMEM;
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    ret = vfs_read(filp, *file_buf, file_len , &pos);
    if (ret < 0)
        VTE("read file fail");
    VTI("file len:%x read len:%x pos:%x", (u32)file_len, ret, (u32)pos);
    filp_close(filp, NULL);
    set_fs(old_fs);

    return ret;
}

int fts_upgrade_bin_0flash(char *fw_name, bool force)
{
    int ret = 0;
    u32 fw_file_len = 0;
    u8 *fw_file_buf = NULL;

    VTI("start upgrade with fw bin");
    if (fts_data->fw_loading) {
        VTI("fw is loading, not download again");
        return -EINVAL;
    }

    ret = fts_read_file_0flash(fw_name, &fw_file_buf);
    if ((ret < 0) || (ret < FTS_MIN_LEN)) {
        VTE("read fw bin file(sdcard) fail, len:%d", fw_file_len);
        goto err_bin;
    }

    fw_file_len = ret;
    VTI("fw bin file len:%x", fw_file_len);
    ret = fts_fw_download(fw_file_buf, fw_file_len);
    if (ret < 0) {
        VTE("upgrade fw bin failed");
        goto err_bin;
    }

    VTI("upgrade fw bin success");

err_bin:
    if (fw_file_buf) {
        vfree(fw_file_buf);
        fw_file_buf = NULL;
    }
    return ret;
}

static void fts_fwupg_work(struct work_struct *work)
{
    int ret = 0;
    u8 ver = 0;
	const struct firmware *fw = NULL;
	struct firmware fw_download;
	//struct device *dev = &fts_data->client->dev;
	int fw_size = 0;

    VTI("fw upgrade work function");
    if (fts_data->fw_loading) {
        VTI("fw is loading, not download again");
        return ;
    }
	
    /* 1. request firmware */	
	VTI("VIVO_TS loading the firmware\n");
	fw_download.data = vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
	fw_download.size = fw_size;
	if (fw_download.data == NULL) {
		VTE("firmware get fail! ");
		return;
	} 

	/* 2. firmware download */
    ret = fts_fw_download(fw_download.data, fw_download.size);
    if (ret < 0) {
        VTE("fw auto download failed");
    } else {
        msleep(10);
        ret = fts_read_reg_byte(FTS_REG_FW_VER, &ver);
        VTI("read firmware version : 0x%02x", ver);
    }

	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}
}

int fts_fw_enter_test_environment(int test_state)
{
    int ret = 0;
    u8 fw_type = 0;
	const struct firmware *fw = NULL;
	struct firmware fw_download;
	struct device *dev = &fts_data->input_dev->dev;
	int fw_size = 0;
    
    VTI("fw test download function in %d mode (0 - normal, 1 - factory)", test_state);
    
    if (fts_data->fw_loading) {
		VTE("fw is loading, not download again");
        return -EINVAL;
    }

	/* 1. request firmware */
	ret = request_firmware(&fw, FTS_FW_NAME, dev);
	if (ret != 0) {
		VTI("VIVO_TS loading the firmware\n");
		fw_download.data = vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
		fw_download.size = fw_size;
		if (fw_download.data == NULL) {
			VTE("firmware get fail! ");
			return -EINVAL;
		}
	} else {
		fw_download.data = fw->data;
		fw_download.size = fw->size;
	}

    if (fw_download.size <= FTS_MAX_LEN_APP) {
		VTE("not multi-app");
        return -EINVAL;
    }

	/* 2. firmware download */
    if (test_state == FTS_FACTORY_MODE) {
        ret = fts_fw_download(fw_download.data + FTS_MAX_LEN_APP, fw_download.size);
    } else {
        ret = fts_fw_download(fw_download.data, fw_download.size);
    }
    if (ret < 0) {
        FTS_ERROR("fw(app2) download fail");
        return ret;
    }

	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}

    msleep(50);
    ret = fts_read_reg_byte(FTS_REG_FW_TYPE, &fw_type);
    VTI("read fw type:0x%02x", fw_type);

    return 0;
}


int fts_fw_resume(void)
{
    int ret = 0;
    const struct firmware *fw = NULL;
	struct firmware fw_download;
	struct device *dev = &fts_data->input_dev->dev;
	int fw_size = 0;

	VTI("fw upgrade resume function");
	
    /* 1. request firmware */
	ret = request_firmware(&fw, FTS_FW_NAME, dev);
	if (ret != 0) {
		VTI("VIVO_TS loading the firmware\n");
		fw_download.data = vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
		fw_download.size = fw_size;
		if (fw_download.data == NULL) {
			VTE("firmware get fail! ");
			return -ENODATA;
		}
	} else {
		fw_download.data = fw->data;
		fw_download.size = fw->size;
	}

	/* 2. firmware download */
    ret = fts_fw_download(fw_download.data, fw_download.size);
    if (ret < 0) {
        VTE("fw resume download failed");
    }

	if (fw != NULL) {
		release_firmware(fw);
		fw = NULL;
	}

    return ret;
}

int fts_fw_recovery(void)
{
    int ret = 0;
    u8 boot_state = 0;
    u8 chip_id = 0;
	
    VTI("check if boot recovery");
    if (fts_data->fw_loading) {
        VTI("fw is loading, not download again");
        return -EINVAL;
    }

    fts_data->fw_is_running = false;
    ret = fts_check_bootid();
    if (ret < 0) {
        FTS_ERROR("check boot id fail");
        fts_data->fw_is_running = true;
        return ret;
    }
    ret = fts_read_reg_byte(0xD0, &boot_state);
    if (ret < 0) {
        VTE("read boot state failed, ret=%d", ret);
		fts_data->fw_is_running = true;
        return ret;
    }

    if (boot_state != 0x02) {
        VTI("not in boot mode(0x%x),exit", boot_state);
        fts_data->fw_is_running = true;
        return -EIO;
    }

    VTI("abnormal situation,need download fw");
    ret = fts_fw_resume();
    if (ret < 0) {
        VTE("fts_fw_resume fail");
        return ret;
    }
    msleep(10);
    ret = fts_read_reg_byte(FTS_REG_CHIP_ID, &chip_id);
    VTI("read chip id:0x%02x", chip_id);

    fts_tp_state_recovery_0flash();
    return ret;
}

int fts_fw_init(void)
{
    VTI("fw upgrade init function");

    if (NULL == fts_data->ts_workqueue) {
        VTE("fts workqueue is NULL, can't run upgrade function");
        return -EINVAL;
    }

    /*INIT_WORK(&fts_data->fwupg_work, fts_fwupg_work);
    queue_work(fts_data->ts_workqueue, &fts_data->fwupg_work);*/
	fts_fwupg_work(NULL);
	
    return 0;
}


int fts_imei_read_8719_noflash(u8 *imei)
{
	int ret = 0;
	u8 data[9] = { 0 };
    u8 cmd = 0x92;
	u8 mode = 0;

	ret = fts_write_reg_byte(0x00,0x40);
	msleep(200);

	ret = fts_read_reg_byte(0x00, &mode);
	if ((ret >=0 ) && (0x40 == mode)) {
		VTI("enter factory mode success");
	} else {
		VTI("enter factory mode failed");
		goto error_end;
	}
	
	ret = fts_write_reg_byte(0x92,0x00);
	msleep(10);
	ret = fts_read(&cmd, 1, data, 9);


	data[0] = data[0] & 0x07;
	data[1] = data[1] & 0x0F;
	data[2] = data[2] & 0x1F;
	data[4] = data[4] & 0x0F;
	data[5] = data[5] & 0x1F;

	
	VTI("%x,%x,%x,%x,%x,%x,%x,%x,%x",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
	
	memcpy(imei, data, 9);

	ret = fts_write_reg_byte(0x00, 0x00);
	msleep(200);

	ret = fts_read_reg_byte(0x00, &mode);
	if ((ret >=0 ) && (0x00 == mode)) {
		VTI("enter work mode success");
	} else {
		VTI("enter work mode failed");
		goto error_end;
	}	

	return 0;

error_end:
	fts_write_reg_byte(0x00,0x00);
	msleep(200);
	return -EIO;

}


