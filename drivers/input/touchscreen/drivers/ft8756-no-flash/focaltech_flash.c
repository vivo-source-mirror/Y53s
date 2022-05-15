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
* Static function
*****************************************************************************/
#define FTS_MANUAL_FW_REQUEST						1

static int fts_check_bootid(void)
{
    int ret = 0;
    u8 cmd = 0;
    u8 id[2] = { 0 };
    struct ft_chip_t *chip_id = fts8756_fts_data->ic_info.ids;

    cmd = FTS_CMD_READ_ID;
    ret = fts8756_fts_read(&cmd, 1, id, 2);
    if (ret < 0) {
        FTS_ERROR("read boot id fail");
        return ret;
    }

    FTS_INFO("read boot id:0x%02x 0x%02x", id[0], id[1]);
    if ((chip_id->rom_idh == id[0]) && (chip_id->rom_idl == id[1])) {
        return 0;
    }

    return -EIO;
}

static int fts_fwupg_hardware_reset_to_boot(void)
{
    if (fts8756_fts_reset_proc(0) > 0)
		mdelay(fts8756_fts_data->ic_info.rst_delay_ms);

    return 0;
}

static int fts_enter_into_boot(void)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 cmd[2] = { 0 };

    FTS_INFO("enter into boot environment");
    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        /* hardware tp reset to boot */
        fts_fwupg_hardware_reset_to_boot();

        /* enter into boot & check boot id*/
        for (j = 0; j < FTS_READ_BOOT_ID_TIMEOUT; j++) {
            cmd[0] = FTS_CMD_START1;
            ret = fts8756_fts_write(cmd, 1);
            if (ret >= 0) {
                mdelay(fts8756_fts_data->ic_info.rst_delay_ms);
                ret = fts_check_bootid();
                if (0 == ret) {
                    FTS_INFO("boot id check pass, retry=%d", i);
                    return 0;
                }
            }
        }
    }

    return -EIO;
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
    ret = fts8756_fts_write(cmd, FTS_ROMBOOT_CMD_ECC_LEN);
    if (ret < 0) {
        VTE("ecc calc cmd fail");
        return ret;
    }
    mdelay(3);

    /* wait boot calculate ecc finish */
    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        ret = fts8756_fts_read(cmd, 1, value, 1);
        if (ret < 0) {
            VTE("ecc finish cmd fail");
            return ret;
        }
        if (FTS_ROMBOOT_CMD_ECC_FINISH_OK == value[0])
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
    ret = fts8756_fts_read(cmd, 1, value, 2);
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

static int fts_ecc_check(const u8 *buf, u32 len, u32 ecc_saddr)
{
    int ret = 0;
    int i = 0;
    u16 ecc_in_host = 0;
    u16 ecc_in_tp = 0;
    int packet_length = 0;
    int packet_number = 0;
    int packet_remainder = 0;
    int offset = 0;
    u32 packet_size = FTS_CMD_ECC_LENGTH_MAX;

    packet_number = len / packet_size;
    packet_remainder = len % packet_size;
    if (packet_remainder)
        packet_number++;
    packet_length = packet_size;

    for (i = 0; i < packet_number; i++) {
        /* last packet */
        if ((i == (packet_number - 1)) && packet_remainder)
            packet_length = packet_remainder;

        ret = fts_ecc_cal_host(buf + offset, packet_length, &ecc_in_host);
        if (ret < 0) {
            FTS_ERROR("ecc in host calc fail");
            return ret;
        }

        ret = fts_ecc_cal_tp(ecc_saddr + offset, packet_length, &ecc_in_tp);
        if (ret < 0) {
            FTS_ERROR("ecc in tp calc fail");
            return ret;
        }

        FTS_DEBUG("ecc in tp:%04x,host:%04x,i:%d", ecc_in_tp, ecc_in_host, i);
        if (ecc_in_tp != ecc_in_host) {
            FTS_ERROR("ecc_in_tp(%x) != ecc_in_host(%x), ecc check fail",
                      ecc_in_tp, ecc_in_host);
            return -EIO;
        }

        offset += packet_length;
    }

    return 0;
}

static int fts_pram_start(void)
{
    int ret = 0;
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;

    VTI("remap to start pram");
    ret = fts8756_fts_write(&cmd, 1);
    if (ret < 0) {
        VTE("write start pram cmd fail");
        return ret;
    }

    return 0;
}

static int fts_dpram_write(u32 saddr, const u8 *buf, u32 len, bool wpram)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u8 *cmd = NULL;
    u32 addr = 0;
    u32 baseaddr = wpram ? FTS_PRAM_SADDR : FTS_DRAM_SADDR;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number = 0;
    u32 packet_len = 0;
    u32 packet_size = FTS_FLASH_PACKET_LENGTH_SPI;

    FTS_INFO("dpram write");
    if (NULL == buf) {
        FTS_ERROR("fw buf is null");
        return -EINVAL;
    }

    if ((len < FTS_MIN_LEN) || (len > FTS_MAX_LEN_APP)) {
        FTS_ERROR("fw length(%d) fail", len);
        return -EINVAL;
    }

    cmd = vmalloc(packet_size + FTS_CMD_WRITE_LEN);
    if (NULL == cmd) {
        FTS_ERROR("malloc memory for pram write buffer fail");
        return -ENOMEM;
    }

    packet_number = len / packet_size;
    remainder = len % packet_size;
    if (remainder > 0)
        packet_number++;
    packet_len = packet_size;

    for (i = 0; i < packet_number; i++) {
        offset = i * packet_size;
        addr = saddr + offset + baseaddr;
        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;

        /* set pram address */
        cmd[0] = FTS_ROMBOOT_CMD_SET_PRAM_ADDR;
        cmd[1] = BYTE_OFF_16(addr);
        cmd[2] = BYTE_OFF_8(addr);
        cmd[3] = BYTE_OFF_0(addr);
        ret = fts8756_fts_write(&cmd[0], FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN);
        if (ret < 0) {
            FTS_ERROR("set pram(%d) addr(%d) fail", i, addr);
            goto write_pram_err;
        }

        /* write pram data */
        cmd[0] = FTS_ROMBOOT_CMD_WRITE;
        for (j = 0; j < packet_len; j++) {
            cmd[1 + j] = buf[offset + j];
        }
        ret = fts8756_fts_write(&cmd[0], 1 + packet_len);
        if (ret < 0) {
            FTS_ERROR("write fw to pram(%d) fail", i);
            goto write_pram_err;
        }
    }

write_pram_err:
    if (cmd) {
        vfree(cmd);
        cmd = NULL;
    }
    return ret;
}

static int fts_pram_write_ecc(const u8 *buf, u32 len)
{
    int ret = 0;
    u32 pram_app_size = 0;
    u16 code_len = 0;
    u16 code_len_n = 0;
    u32 pram_start_addr = 0;

    FTS_INFO("begin to write pram app(bin len:0x%x)", len);
    /* get pram app length */
    code_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
               + buf[FTS_APP_INFO_OFFSET + 1];
    code_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 2] << 8)
                 + buf[FTS_APP_INFO_OFFSET + 3];
    if ((code_len + code_len_n) != 0xFFFF) {
        FTS_ERROR("pram code len(%x %x) fail", code_len, code_len_n);
        return -EINVAL;
    }
	
   pram_app_size = (u32)(code_len * fts8756_fts_data->ic_info.pram_size_k);

    if ((pram_app_size < FTS_MIN_LEN) || (pram_app_size > FTS_MAX_LEN_APP)) {
        FTS_ERROR("pram app length(%d) is invalid", pram_app_size);
        return -EINVAL;
    }

    FTS_INFO("pram app length in fact:%d", pram_app_size);
    /* write pram */
    ret = fts_dpram_write(pram_start_addr, buf, pram_app_size, true);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }

    /* check ecc */
    ret = fts_ecc_check(buf, pram_app_size, pram_start_addr);
    if (ret < 0) {
        FTS_ERROR("pram ecc check fail");
        return ret;
    }

    FTS_INFO("pram app write successfully");
    return 0;
}

static int fts_dram_write_ecc(const u8 *buf, u32 len)
{
    int ret = 0;
    u32 dram_size = 0;
    u32 pram_app_size = 0;
    u32 dram_start_addr = 0;
    u16 const_len = 0;
    u16 const_len_n = 0;
    const u8 *dram_buf = NULL;

    FTS_INFO("begin to write dram data(bin len:0x%x)", len);
    /* get dram data length */
    const_len = ((u16)buf[FTS_APP_INFO_OFFSET + 0x8] << 8)
                + buf[FTS_APP_INFO_OFFSET + 0x9];
    const_len_n = ((u16)buf[FTS_APP_INFO_OFFSET + 0x0A] << 8)
                  + buf[FTS_APP_INFO_OFFSET + 0x0B];
    if (((const_len + const_len_n) != 0xFFFF) || (const_len == 0)) {
        FTS_INFO("no support dram,const len(%x %x)", const_len, const_len_n);
        return 0;
    }

    dram_size = ((u32)const_len) * 2;
    if ((dram_size <= 0) || (dram_size > FTS_MAX_LEN_APP_PARAMS)) {
        FTS_ERROR("dram data length(%d) is invalid", dram_size);
        return -EINVAL;
    }

    pram_app_size = ((u32)(((u16)buf[FTS_APP_INFO_OFFSET + 0] << 8)
                           + buf[FTS_APP_INFO_OFFSET + 1])) * 2;

    dram_buf = buf + pram_app_size;
    FTS_INFO("dram buf length in fact:%d,offset:%d", dram_size, pram_app_size);
    /* write pram */
    ret = fts_dpram_write(dram_start_addr, dram_buf, dram_size, false);
    if (ret < 0) {
        FTS_ERROR("write dram fail");
        return ret;
    }

    /* check ecc */
    ret = fts_ecc_check(dram_buf, dram_size, dram_start_addr);
    if (ret < 0) {
        FTS_ERROR("dram ecc check fail");
        return ret;
    }

    FTS_INFO("dram data write successfully");
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

    FTS_INFO("begin to write and start fw(bin len:0x%x)", len);
    fts8756_fts_data->fw_is_running = false;

    /* enter into boot environment */
    ret = fts_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into boot environment fail");
        return ret;
    }

    /* write pram */
    ret = fts_pram_write_ecc(buf, len);
    if (ret < 0) {
        FTS_ERROR("write pram fail");
        return ret;
    }
	
if(fts8756_fts_data->ic_info.dram_support){
    /* write dram */
    ret = fts_dram_write_ecc(buf, len);
    if (ret < 0) {
        FTS_ERROR("write dram fail");
        return ret;
    }
}

    /* remap pram and run fw */
    ret = fts_pram_start();
    if (ret < 0) {
        FTS_ERROR("pram start fail");
        return ret;
    }
    
    fts8756_fts_data->fw_is_running = true;
    FTS_INFO("fw download successfully");
    return 0;
}

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
int fts8756_fts_fw_download(const u8 *buf, u32 len)
{
	int ret = 0;
	int i = 0;
	ktime_t kt;
	struct fts_ts_data *ts_data = fts8756_fts_data;

	VTI("fw upgrade download function");
	if (NULL == buf) {
		VTE("fw buf is null");
		return -EINVAL;
	}

	if (len < FTS_MIN_LEN) {
		VTE("fw length(%d) is invalid", len);
		return -EINVAL;
	}

	kt = ktime_get();
	ts_data->fw_loading = 1;
	fts8756_fts_irq_disable();
#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(DISABLE);
#endif

	for (i = 0; i < 3; i++) {
		VTI("fw download times:%d", i + 1);
		ret = fts_fw_write_start(buf, len);
		if (0 == ret)
			break;
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_switch(ENABLE);
#endif
	fts8756_fts_irq_enable();
	ts_data->fw_loading = 0;

	if (i >= 3) {
		VTE("fw download fail");
		return -EIO;
	}
	VTI("Update firmware success! cost <%lld> us", ktime_to_us(ktime_get()) - ktime_to_us(kt));

	return 0;
}

int fts8756_fts8756_fts_read_file(char *file_name, u8 **file_buf)
{
    int ret = 0;
#if FTS_MANUAL_FW_REQUEST
    	const struct firmware *fw = NULL;
    	char fwname[FILE_NAME_LENGTH] = { 0 };
		
		snprintf(fwname, FILE_NAME_LENGTH, "%s", \
	             file_name);

	   ret = request_firmware(&fw, fwname, &fts8756_fts_data->spi->dev);
	    if (0 == ret) {
	        VTI("firmware(%s) request successfully", fwname);
			*file_buf = (u8 *)vmalloc(fw->size);
	        if (NULL == *file_buf) {
	            VTE("fw buffer vmalloc fail");
	            ret = -ENOMEM;
	        } else {
	            memcpy(*file_buf, fw->data, fw->size);
				ret = fw->size;
	        }
	    } else {
	        VTI("firmware(%s) request fail,ret=%d", fwname, ret);
	    }

	    if (fw != NULL) {
	        release_firmware(fw);
	        fw = NULL;
	    }
    

#else
    char file_path[FILE_NAME_LENGTH] = { 0 };
    struct file *filp = NULL;
    struct inode *inode;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
    mm_segment_t old_fs;
#endif
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
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    ret = vfs_read(filp, *file_buf, file_len , &pos);
    if (ret < 0)
        VTE("read file fail");
    VTI("file len:%x read len:%x pos:%x", (u32)file_len, ret, (u32)pos);
    filp_close(filp, NULL);
    set_fs(old_fs);
#else
    pos = 0;
    ret = kernel_read(filp, *file_buf, file_len , &pos);
    if (ret < 0)
        VTE("read file fail");
    VTI("file len:%x read len:%x pos:%x", (u32)file_len, ret, (u32)pos);
    filp_close(filp, NULL);
#endif
#endif
    return ret;
}

int fts8756_fts_upgrade_bin(char *fw_name, bool force)
{
    int ret = 0;
    u32 fw_file_len = 0;
    u8 *fw_file_buf = NULL;

    VTI("start upgrade with fw bin");
    if (fts8756_fts_data->fw_loading) {
        VTI("fw is loading, not download again");
        return -EINVAL;
    }

    ret = fts8756_fts8756_fts_read_file(fw_name, &fw_file_buf);
    if ((ret < 0) || (ret < FTS_MIN_LEN) || (ret > FTS_MAX_LEN_APP)) {
        VTE("read fw bin file(sdcard) fail, len:%d", fw_file_len);
        goto err_bin;
    }

    fw_file_len = ret;
    VTI("fw bin file len:%x", fw_file_len);
    ret = fts8756_fts_fw_download(fw_file_buf, fw_file_len);
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

int fts8756_fts_fw_enter_test_environment(int test_state)
{
    int ret = 0;
    u8 fw_type = 0;
	struct firmware fw_download;
	int fw_size = 0;
    
    VTI("fw test download function in %d mode (0 - normal, 1 - factory)", test_state);
    
    if (fts8756_fts_data->fw_loading) {
		VTE("fw is loading, not download again");
        return -EINVAL;
    }

	fw_download.data = vts_fw_data_get(fts8756_fts_data->vtsdev, VTS_FW_TYPE_FW, &fw_size);
	fw_download.size = fw_size;
	if (fw_download.data == NULL) {
		VTE("firmware get fail! ");
		return -EINVAL;
	}

    if (fw_download.size <= fts8756_fts_data->ic_info.fw_max_len) {
		VTE("not multi-app");
        return -EINVAL;
    }

	/* 2. firmware download */
    if (test_state == FTS_FACTORY_MODE) {
        ret = fts8756_fts_fw_download(fw_download.data + fts8756_fts_data->ic_info.fw_max_len, fw_download.size);
    } else {
        ret = fts8756_fts_fw_download(fw_download.data, fw_download.size);
    }
    if (ret < 0) {
        FTS_ERROR("fw(app2) download fail");
        return ret;
    }

    msleep(50);
    ret = fts8756_fts8756_fts_read_reg_byte(FTS_REG_FW_TYPE, &fw_type);
    VTI("read fw type:0x%02x", fw_type);

    return 0;
}


int fts8756_fts_fw_resume(int fw_type)
{
	int ret = 0;
	struct firmware fw_download;
	int fw_size = 0;
	int fw_offset = 0;

	VTI("fw upgrade resume function");
	
	fw_download.data = vts_fw_data_get(fts8756_fts_data->vtsdev, VTS_FW_TYPE_FW, &fw_size);
	fw_download.size = fw_size;
	if (fw_download.data == NULL) {
		VTE("firmware get fail! ");
		return -ENODATA;
	}

	if (fw_type == VTS_ST_GESTURE && fts8756_fts_data->gesture_separate ){
		fw_offset = fts8756_fts_data->ic_info.fw_max_len * 2;
		VTI("*****************gesture_separate!!!*****************");
	}

	if (fw_download.size < fw_offset) {
		VTE("gesture_separate firmware Invalid!!! fw_download.size = %d, < fw_offset = %d ", (int)fw_download.size, fw_offset);
		return -ENODATA;
	}

	/* 2. firmware download */
	ret = fts8756_fts_fw_download(fw_download.data + fw_offset, fw_download.size - fw_offset);
	if (ret < 0) {
		VTE("fw resume download failed");
	}

	return ret;
}

int fts8756_focal_fw_recovery(int fw_type)
{
    int ret = 0;
    u8 boot_state = 0;
    u8 chip_id = 0;
	
    VTI("check if boot recovery");
    if (fts8756_fts_data->fw_loading) {
        VTI("fw is loading, not download again");
        return -EINVAL;
    }

    fts8756_fts_data->fw_is_running = false;
    ret = fts_check_bootid();
    if (ret < 0) {
        FTS_ERROR("check boot id fail");
        fts8756_fts_data->fw_is_running = true;
        return ret;
    }
    ret = fts8756_fts8756_fts_read_reg_byte(0xD0, &boot_state);
    if (ret < 0) {
        VTE("read boot state failed, ret=%d", ret);
		fts8756_fts_data->fw_is_running = true;
        return ret;
    }

    if (boot_state != 0x01) {
        VTI("not in boot mode(0x%x),exit", boot_state);
        fts8756_fts_data->fw_is_running = true;
        return -EIO;
    }

    VTI("abnormal situation,need download fw");
    ret = fts8756_fts_fw_resume(fw_type);
    if (ret < 0) {
        VTE("fts8756_fts_fw_resume fail");
        return ret;
    }
    msleep(10);
    ret = fts8756_fts8756_fts_read_reg_byte(FTS_REG_CHIP_ID, &chip_id);
    VTI("read chip id:0x%02x", chip_id);

    fts8756_fts_tp_state_recovery();
    return ret;
}

