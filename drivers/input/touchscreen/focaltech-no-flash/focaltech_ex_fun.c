/*
 *
 * FocalTech TouchScreen driver.
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
* File Name: Focaltech_ex_fun.c
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

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
/*create apk debug channel*/
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_HW_RESET                           11
#define PROC_READ_STATUS                        12
#define PROC_SET_BOOT_MODE                      13
#define PROC_ENTER_TEST_ENVIRONMENT             14
#define PROC_NAME                               "ftxxxx-debug"
#define PROC_WRITE_BUF_SIZE                     256
#define PROC_READ_BUF_SIZE                      256

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/
enum {
    RWREG_OP_READ = 0,
    RWREG_OP_WRITE = 1,
};
static struct rwreg_operation_t {
    int type;         // 0: read, 1: write
    int reg;        // register
    int len;        // read/write length
    int val;      // length = 1; read: return value, write: op return
    int res;     // 0: success, otherwise: fail
    char *opbuf;        // length >= 1, read return value, write: op return
} rw_op;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    u8 writebuf[PROC_WRITE_BUF_SIZE] = { 0 };
    int buflen = count;
    int writelen = 0;
    int ret = 0;
    char tmp[PROC_WRITE_BUF_SIZE] = { 0 };
    struct fts_ts_data *ts_data = fts_data;

    if ((count == 0) || (count > PROC_WRITE_BUF_SIZE)) {
        VTE("apk proc wirte count(%d) fail", (int)count);
        return -EINVAL;
    }

    if (copy_from_user(&writebuf, buff, count)) {
        VTE("copy from user error");
        return -EFAULT;
    }

    ts_data->proc->opmode = writebuf[0];
    switch (ts_data->proc->opmode) {
    case PROC_SET_TEST_FLAG:
        VTI("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
#if FTS_ESDCHECK_EN
        if (writebuf[1] == 0) {
			fts_esdcheck_switch_pd1831(ENABLE);
        } else {
			fts_esdcheck_switch_pd1831(DISABLE);
        }
#endif
        break;

    case PROC_HW_RESET:
        sprintf(tmp, "%s", writebuf + 1);
        tmp[buflen - 1] = '\0';
        if (strncmp(tmp, "focal_driver", 12) == 0) {
            VTI("APK execute HW Reset");
            fts_reset_proc_0flash(1);
        }
        break;

    case PROC_READ_REGISTER:
        ts_data->proc->cmd[0] = writebuf[1];
        break;
    case PROC_WRITE_REGISTER:
        writelen = 2;
        ret = fts_write(writebuf + 1, writelen);
        if (ret < 0) {
            VTE("PROC_WRITE_REGISTER write error");
        }
        break;

    case PROC_READ_DATA:
        if (buflen - 1 > SPI_MAX_COMMAND_LENGTH) {
            VTE("PROC_READ_DATA,buflen(%d) invalid", buflen);
            ret = -EINVAL;
        } else {
            memcpy(ts_data->proc->cmd, writebuf + 1, buflen - 1);
            ts_data->proc->cmd_len = buflen - 1;
        }
        break;
    case PROC_WRITE_DATA:
        writelen = buflen - 1;
        if (writelen > 0) {
            ret = fts_write(writebuf + 1, writelen);
            if (ret < 0) {
                VTE("PROC_WRITE_DATA write error");
            }
        }
        break;
    case PROC_READ_STATUS:
        break;

    case PROC_SET_BOOT_MODE:
        if (0 == writebuf[1]) {
            ts_data->fw_is_running = true;
        } else {
            ts_data->fw_is_running = false;
        }
        break;

	case PROC_ENTER_TEST_ENVIRONMENT:  
        if (FTS_NORMAL_MODE == writebuf[1]) {
            ret = fts_fw_enter_test_environment(FTS_NORMAL_MODE);
        } else {
            ret = fts_fw_enter_test_environment(FTS_FACTORY_MODE);
        }
        break;

    default:
        break;
    }

    if (ret < 0) {
        return ret;
    } else {
        return count;
    }
}

/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    int num_read_chars = 0;
    int readlen = 0;
    u8 buf[PROC_READ_BUF_SIZE] = { 0 };
    struct fts_ts_data *ts_data = fts_data;

    if ((count == 0) || (count > PROC_READ_BUF_SIZE)) {
        VTE("apk proc read count(%d) fail", (int)count);
        return -EINVAL;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(1);
#endif

    switch (ts_data->proc->opmode) {
    case PROC_READ_REGISTER:
        readlen = 1;
        ret = fts_read(ts_data->proc->cmd, 1, buf, 1);
        if (ret < 0) {
            VTE("PROC_READ_REGISTER read error");
        }
        num_read_chars = 1;
        break;
    case PROC_WRITE_REGISTER:
        break;

    case PROC_READ_DATA:
        readlen = count;
        ret = fts_read(ts_data->proc->cmd, ts_data->proc->cmd_len, buf, readlen);
        if (ret < 0) {
            VTE("PROC_READ_DATA read error");
        }
        num_read_chars = readlen;
        break;
    case PROC_WRITE_DATA:
        break;

    case PROC_READ_STATUS:
        /*
        ret = fts_read_status(buf);
        if (ret < 0) {
            FTS_ERROR("read status error");
        }
        num_read_chars = 1;
        */
        break;
    default:
        break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(0);
#endif

    if (ret < 0)
        return ret;

    if (copy_to_user(buff, buf, num_read_chars)) {
        VTE("copy to user error");
        return -EFAULT;
    }

    return num_read_chars;
}

static const struct file_operations fts_proc_fops = {
    .owner  = THIS_MODULE,
    .read   = fts_debug_read,
    .write  = fts_debug_write,
};

int fts_create_apk_debug_channel_0flash(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    ts_data->proc = kzalloc(sizeof(struct ftxxxx_proc), GFP_KERNEL);
    if (NULL == ts_data->proc) {
        VTE("allocate memory for proc fail");
        return -ENOMEM;
    }
    ts_data->proc->proc = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
    if (NULL == ts_data->proc->proc) {
        VTE("create proc entry fail");
        kfree(ts_data->proc);
        ts_data->proc = NULL;
        return -ENOMEM;
    }

    FTS_FUNC_EXIT();
    return 0;
}

void fts_release_apk_debug_channel_0flash(struct fts_ts_data *ts_data)
{
    if (ts_data->proc) {
        if (ts_data->proc->proc) {
            proc_remove(ts_data->proc->proc);
        }

        kfree(ts_data->proc);
        ts_data->proc = NULL;
    }
}

/************************************************************************
 * sysfs interface
 ***********************************************************************/
/* fts_hw_reset interface */
static ssize_t fts_hw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_hw_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t count = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    fts_reset_proc_0flash(0);
    count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_irq interface */
static ssize_t fts_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        VTI("[EX-FUN]enable irq");
        fts_irq_enable_0flash();
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        VTI("[EX-FUN]disable irq");
        fts_irq_disable_0flash();
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

static ssize_t fts_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_boot_mode interface */
static ssize_t fts_bootmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        VTI("[EX-FUN]set to boot mode");
        fts_data->fw_is_running = false;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        VTI("[EX-FUN]set to fw mode");
        fts_data->fw_is_running = true;
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

static ssize_t fts_bootmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t count = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (true == fts_data->fw_is_running) {
        count = snprintf(buf, PAGE_SIZE, "tp is in fw mode\n");
    } else {
        count = snprintf(buf, PAGE_SIZE, "tp is in boot mode\n");
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_tpfwver interface */
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
    ssize_t num_read_chars = 0;
    u8 fwver = 0;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(1);
#endif
    if (fts_read_reg_byte(FTS_REG_FW_VER, &fwver) < 0) {
        num_read_chars = snprintf(buf, PAGE_SIZE, "SPI transfer error!\n");
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(0);
#endif
    if ((fwver == 0xFF) || (fwver == 0x00))
        num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
    else
        num_read_chars = snprintf(buf, PAGE_SIZE, "%02x\n", fwver);

    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return num_read_chars;
}

/* fts_rw_reg */
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);

    if (rw_op.len < 0) {
        count = snprintf(buf, PAGE_SIZE, "Invalid cmd line\n");
    } else if (rw_op.len == 1) {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Read %02X: %02X\n", rw_op.reg, rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Read %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        } else {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Write %02X, %02X success\n", rw_op.reg,  rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Write %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        }
    } else {
        if (RWREG_OP_READ == rw_op.type) {
            count = snprintf(buf, PAGE_SIZE, "Read Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len);
            count += snprintf(buf + count, PAGE_SIZE, "Result: ");
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "failed, ret: %d\n", rw_op.res);
            } else {
                if (rw_op.opbuf) {
                    for (i = 0; i < rw_op.len; i++) {
                        count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                    }
                    count += snprintf(buf + count, PAGE_SIZE, "\n");
                }
            }
        } else {
            ;
            count = snprintf(buf, PAGE_SIZE, "Write Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len - 1);
            count += snprintf(buf + count, PAGE_SIZE, "Write Data: ");
            if (rw_op.opbuf) {
                for (i = 1; i < rw_op.len; i++) {
                    count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                }
                count += snprintf(buf + count, PAGE_SIZE, "\n");
            }
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "Result: failed, ret: %d\n", rw_op.res);
            } else {
                count += snprintf(buf + count, PAGE_SIZE, "Result: success\n");
            }
        }
        /*if (rw_op.opbuf) {
            kfree(rw_op.opbuf);
            rw_op.opbuf = NULL;
        }*/
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static int shex_to_int(const char *hex_buf, int size)
{
    int i;
    int base = 1;
    int value = 0;
    char single;

    for (i = size - 1; i >= 0; i--) {
        single = hex_buf[i];

        if ((single >= '0') && (single <= '9')) {
            value += (single - '0') * base;
        } else if ((single >= 'a') && (single <= 'z')) {
            value += (single - 'a' + 10) * base;
        } else if ((single >= 'A') && (single <= 'Z')) {
            value += (single - 'A' + 10) * base;
        } else {
            return -EINVAL;
        }

        base *= 16;
    }

    return value;
}


static u8 shex_to_u8(const char *hex_buf, int size)
{
    return (u8)shex_to_int(hex_buf, size);
}
/*
 * Format buf:
 * [0]: '0' write, '1' read(reserved)
 * [1-2]: addr, hex
 * [3-4]: length, hex
 * [5-6]...[n-(n+1)]: data, hex
 */
static int fts_parse_buf(const char *buf, size_t cmd_len)
{
    int length;
    int i;
    char *tmpbuf;

    rw_op.reg = shex_to_u8(buf + 1, 2);
    length = shex_to_int(buf + 3, 2);

    if (buf[0] == '1') {
        rw_op.len = length;
        rw_op.type = RWREG_OP_READ;
        VTD("read %02X, %d bytes", rw_op.reg, rw_op.len);
    } else {
        if (cmd_len < (length * 2 + 5)) {
            pr_err("data invalided!\n");
            return -EINVAL;
        }
        VTD("write %02X, %d bytes", rw_op.reg, length);

        /* first byte is the register addr */
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = length + 1;
    }

    if (rw_op.len > 0) {
        tmpbuf = (char *)kzalloc(rw_op.len, GFP_KERNEL);
        if (!tmpbuf) {
            VTE("allocate memory failed!\n");
            return -ENOMEM;
        }

        if (RWREG_OP_WRITE == rw_op.type) {
            tmpbuf[0] = rw_op.reg & 0xFF;
            VTD("write buffer: ");
            for (i = 1; i < rw_op.len; i++) {
                tmpbuf[i] = shex_to_u8(buf + 5 + i * 2 - 2, 2);
                VTD("buf[%d]: %02X", i, tmpbuf[i] & 0xFF);
            }
        }
        rw_op.opbuf = tmpbuf;
    }

    return rw_op.len;
}



/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t cmd_length = 0;

    mutex_lock(&input_dev->mutex);
    cmd_length = count - 1;

    if (rw_op.opbuf) {
        kfree(rw_op.opbuf);
        rw_op.opbuf = NULL;
    }

    VTD("cmd len: %d, buf: %s", (int)cmd_length, buf);
    /* compatible old ops */
    if (2 == cmd_length) {
        rw_op.type = RWREG_OP_READ;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
    } else if (4 == cmd_length) {
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
        rw_op.val = shex_to_int(buf + 2, 2);
    } else if (cmd_length < 5) {
        VTE("Invalid cmd buffer");
        mutex_unlock(&input_dev->mutex);
        return -EINVAL;
    } else {
        rw_op.len = fts_parse_buf(buf, cmd_length);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(1);
#endif
    if (rw_op.len < 0) {
        VTE("cmd buffer error!");

    } else {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                rw_op.res = fts_read_reg_byte(reg, &val);
                rw_op.val = val;
            } else {
                char reg;
                reg = rw_op.reg & 0xFF;

                rw_op.res = fts_read(&reg, 1, rw_op.opbuf, rw_op.len);
            }

            if (rw_op.res < 0) {
                VTE("Could not read 0x%02x", rw_op.reg);
            } else {
                VTI("read 0x%02x, %d bytes successful", rw_op.reg, rw_op.len);
                rw_op.res = 0;
            }

        } else {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                val = rw_op.val & 0xFF;
                rw_op.res = fts_write_reg_byte(reg, val);
            } else {
                rw_op.res = fts_write(rw_op.opbuf, rw_op.len);
            }
            if (rw_op.res < 0) {
                VTE("Could not write 0x%02x", rw_op.reg);

            } else {
                VTI("Write 0x%02x, %d bytes successful", rw_op.val, rw_op.len);
                rw_op.res = 0;
            }
        }
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(0);
#endif
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_upgrade_bin_0flash interface */
static ssize_t fts_fwupgradebin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[FILE_NAME_LENGTH] = { 0 };
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
        VTE("fw bin name's length(%d) fail", (int)count);
        return -EINVAL;
    }
    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count - 1] = '\0';

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    VTI("upgrade with bin file through sysfs node");
    fts_upgrade_bin_0flash(fwname, 0);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_fwupgradebin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_force_upgrade interface */
static ssize_t fts_fwforceupg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_fwforceupg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

/* fts_driver_version interface */
static ssize_t fts_driverversion_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_driverversion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    count = sprintf(buf, FTS_DRIVER_VERSION "\n");
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_dump_reg interface */
static ssize_t fts_dumpreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_dumpreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(1);
#endif
    fts_read_reg_byte(FTS_REG_POWER_MODE, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Power Mode:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_FW_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "FW Ver:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_LIC_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "LCD Initcode Ver:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_IDE_PARA_VER_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param Ver:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_IDE_PARA_STATUS, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param status:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_VENDOR_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Vendor ID:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_LCD_BUSY_NUM, &val);
    count += snprintf(buf + count, PAGE_SIZE, "LCD Busy Number:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_GESTURE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Mode:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_CHARGER_MODE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "charge stat:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_INT_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "INT count:0x%02x\n", val);

    fts_read_reg_byte( FTS_REG_FLOW_WORK_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "ESD count:0x%02x\n", val);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy_pd1831(0);
#endif

    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_dump_reg interface */
static ssize_t fts_tpbuf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

static ssize_t fts_tpbuf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count += snprintf(buf + count, PAGE_SIZE, "touch point buffer:\n");
    for (i = 0; i < fts_data->pnt_buf_size; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%02x ", fts_data->point_buf[i]);
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* get the fw version  example:cat fw_version */
static DEVICE_ATTR(fts_fw_version, S_IRUGO | S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

/* read and write register(s)
*   All data type is **HEX**
*   Single Byte:
*       read:   echo 88 > rw_reg ---read register 0x88
*       write:  echo 8807 > rw_reg ---write 0x07 into register 0x88
*   Multi-bytes:
*       [0:rw-flag][1-2: reg addr, hex][3-4: length, hex][5-6...n-n+1: write data, hex]
*       rw-flag: 0, write; 1, read
*       read:  echo 10005           > rw_reg ---read reg 0x00-0x05
*       write: echo 000050102030405 > rw_reg ---write reg 0x00-0x05 as 01,02,03,04,05
*  Get result:
*       cat rw_reg
*/

static DEVICE_ATTR(fts_rw_reg, S_IRUGO | S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*  upgrade from fw bin file   example:echo "*.bin" > fts_upgrade_bin_0flash */
static DEVICE_ATTR(fts_upgrade_bin, S_IRUGO | S_IWUSR, fts_fwupgradebin_show, fts_fwupgradebin_store);
static DEVICE_ATTR(fts_force_upgrade, S_IRUGO | S_IWUSR, fts_fwforceupg_show, fts_fwforceupg_store);
static DEVICE_ATTR(fts_driver_version, S_IRUGO | S_IWUSR, fts_driverversion_show, fts_driverversion_store);
static DEVICE_ATTR(fts_dump_reg, S_IRUGO | S_IWUSR, fts_dumpreg_show, fts_dumpreg_store);
static DEVICE_ATTR(fts_hw_reset, S_IRUGO | S_IWUSR, fts_hw_reset_show, fts_hw_reset_store);
static DEVICE_ATTR(fts_irq, S_IRUGO | S_IWUSR, fts_irq_show, fts_irq_store);
static DEVICE_ATTR(fts_boot_mode, S_IRUGO | S_IWUSR, fts_bootmode_show, fts_bootmode_store);
static DEVICE_ATTR(fts_touch_point, S_IRUGO | S_IWUSR, fts_tpbuf_show, fts_tpbuf_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_fts_fw_version.attr,
    &dev_attr_fts_rw_reg.attr,
    &dev_attr_fts_dump_reg.attr,
    &dev_attr_fts_upgrade_bin.attr,
    &dev_attr_fts_force_upgrade.attr,
    &dev_attr_fts_driver_version.attr,
    &dev_attr_fts_hw_reset.attr,
    &dev_attr_fts_irq.attr,
    &dev_attr_fts_boot_mode.attr,
    &dev_attr_fts_touch_point.attr,
    NULL
};

static struct attribute_group fts_attribute_group = {
    .attrs = fts_attributes
};

int fts_create_sysfs_0flash(struct device *dev)
{
    int ret = 0;

    ret = sysfs_create_group(&dev->kobj, &fts_attribute_group);
    if (ret) {
        VTE("sysfs_create_group() fail");
        sysfs_remove_group(&dev->kobj, &fts_attribute_group);
        return -ENOMEM;
    } else {
        VTI("sysfs_create_group() success");
    }

    return ret;
}

int fts_remove_sysfs_0flash(struct device *dev)
{
    sysfs_remove_group(&dev->kobj, &fts_attribute_group);
    return 0;
}
