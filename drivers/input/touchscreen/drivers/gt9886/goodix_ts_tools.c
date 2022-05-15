/*
 * Goodix GTX5 tools Dirver
 *
 * Copyright (C) 2015 - 2016 Goodix, Inc.
 * Authors:  Wang Yafei <wangyafei@goodix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include "goodix_ts_core.h"

#define GOODIX_TOOLS_NAME		"gtp_tools"
#define GOODIX_TS_IOC_MAGIC		'G'
#define NEGLECT_SIZE_MASK		(~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GTP_IRQ_ENABLE	_IO(GOODIX_TS_IOC_MAGIC, 0)
#define GTP_DEV_RESET	_IO(GOODIX_TS_IOC_MAGIC, 1)
#define GTP_SEND_COMMAND (_IOW(GOODIX_TS_IOC_MAGIC, 2, u8) & NEGLECT_SIZE_MASK)
#define GTP_SEND_CONFIG	(_IOW(GOODIX_TS_IOC_MAGIC, 3, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_READ	(_IOR(GOODIX_TS_IOC_MAGIC, 4, u8) & NEGLECT_SIZE_MASK)
#define GTP_SYNC_READ	(_IOR(GOODIX_TS_IOC_MAGIC, 5, u8) & NEGLECT_SIZE_MASK)
#define GTP_ASYNC_WRITE	(_IOW(GOODIX_TS_IOC_MAGIC, 6, u8) & NEGLECT_SIZE_MASK)
#define GTP_READ_CONFIG	(_IOW(GOODIX_TS_IOC_MAGIC, 7, u8) & NEGLECT_SIZE_MASK)
#define GTP_ESD_ENABLE	_IO(GOODIX_TS_IOC_MAGIC, 8)
#define GTP_DRV_VERSION     (_IOR(GOODIX_TS_IOC_MAGIC, 9, u8) & NEGLECT_SIZE_MASK)
#define GTP_DRV_CMD_ENABLE	_IO(GOODIX_TS_IOC_MAGIC, 10)
#define GTP_MODULE_INFO_READ (_IOR(GOODIX_TS_IOC_MAGIC, 11, u8) & NEGLECT_SIZE_MASK)
#define GOODIX_TS_IOC_MAXNR		12

#define IRQ_FALG	(0x01 << 2)

#define I2C_MSG_HEAD_LEN	20
#define TS_REG_COORDS_BASE	0x4100

/*
 * struct goodix_tools_data - goodix tools data message used in sync read
 * @data: The buffer into which data is written
 * @reg_addr: Slave device register start address to start read data
 * @length: Number of data bytes in @data being read from slave device
 * @filled: When buffer @data be filled will set this flag with 1, outhrwise 0
 * @list_head:Eonnet every goodix_tools_data struct into a list
*/

struct goodix_tools_data {
	u32 reg_addr;
	u32 length;
	u8 *data;
	bool filled;
	struct list_head list;
};


/*
 * struct goodix_tools_dev_V2 - goodix tools device struct
 * @ts_core: The core data struct of ts driver
 * @ops_mode: represent device work mode
 * @rawdiffcmd: Set slave device into rawdata mode
 * @normalcmd: Set slave device into normal mode
 * @wq: Wait queue struct use in synchronous data read
 * @mutex: Protect goodix_tools_dev_V2
*/
struct goodix_tools_dev_V2 {
	struct goodix_ts_core *ts_core;
	struct list_head head;
	unsigned int ops_mode;
	struct goodix_ts_cmd rawdiffcmd, normalcmd;
	wait_queue_head_t wq;
	struct mutex mutex;
	atomic_t t_count;
	struct goodix_ext_module module;
} *goodix_tools_dev_V2;


/* read data from i2c asynchronous,
** success return bytes read, else return <= 0
*/
static int async_read(struct goodix_tools_dev_V2 *dev, void __user *arg)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_ts_device *ts_dev = dev->ts_core->ts_dev;
	const struct goodix_ts_hw_ops *hw_ops = ts_dev->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		ret = -EFAULT;
		goto err_out;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);

	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
			VTE("Alloc memory failed");
			return -ENOMEM;
	}

	if (!hw_ops->read_trans(ts_dev, reg_addr, databuf, length)) {
		if (copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, databuf, length)) {
			ret = -EFAULT;
			VTE("Copy_to_user failed");
		} else {
			ret = length;
		}
	} else {
		ret = -EBUSY;
		VTE("Read i2c failed");
	}
err_out:
	kfree(databuf);
	return ret;
}

/* if success return config data length */
static int read_config_data(struct goodix_ts_device *ts_dev, void __user *arg)
{
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	u8 *tmp_buf;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		VTE("Copy data from user failed");
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
		   + (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
		 + (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	VTI("read config,reg_addr=0x%x, length=%d", reg_addr, length);
	tmp_buf = kzalloc(length, GFP_KERNEL);
	if (!tmp_buf) {
		VTE("failed alloc memory");
		return -ENOMEM;
	}
	/* if reg_addr == 0, read config data with specific flow */
	if (!reg_addr) {
		if (ts_dev->hw_ops->read_config)
			ret = ts_dev->hw_ops->read_config(ts_dev, tmp_buf, 0);
		else
			ret = -EINVAL;
	} else {
		ret = ts_dev->hw_ops->read_trans(ts_dev, reg_addr, tmp_buf, length);
		if (!ret)
			ret = length;
	}
	if (ret <= 0)
		goto err_out;

	if (copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, tmp_buf, ret)) {
			ret = -EFAULT;
			VTE("Copy_to_user failed");
	}

err_out:
	kfree(tmp_buf);
	return ret;
}

/* read data from i2c synchronous,
** success return bytes read, else return <= 0
*/
static int sync_read(struct goodix_tools_dev_V2 *dev, void __user *arg)
{
	int ret = 0;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_tools_data tools_data;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		VTE("Copy data from user failed");
		return -EFAULT;
	}
	tools_data.reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
				+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	tools_data.length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
				+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	tools_data.filled = 0;

	tools_data.data = kzalloc(tools_data.length, GFP_KERNEL);
	if (!tools_data.data) {
			VTE("Alloc memory failed");
			return -ENOMEM;
	}

	mutex_lock(&dev->mutex);
	list_add_tail(&tools_data.list, &dev->head);
	mutex_unlock(&dev->mutex);
	/* wait queue will timeout after 1 seconds */
	wait_event_interruptible_timeout(dev->wq, tools_data.filled == 1, HZ * 3);

	mutex_lock(&dev->mutex);
	list_del(&tools_data.list);
	mutex_unlock(&dev->mutex);
	if (tools_data.filled == 1) {
		if (copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, tools_data.data,
							tools_data.length)) {
			ret = -EFAULT;
			VTE("Copy_to_user failed");
		} else {
			ret = tools_data.length;
		}
	} else {
		ret = -EINVAL;
		VTE("Wait queue timeout");
	}

	kfree(tools_data.data);
	return ret;
}

/* write data to i2c asynchronous,
** success return bytes write, else return <= 0
*/
static int async_write(struct goodix_tools_dev_V2 *dev, void __user *arg)
{
	u8 *databuf;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_ts_device *ts_dev = dev->ts_core->ts_dev;
	const struct goodix_ts_hw_ops *hw_ops = ts_dev->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		VTE("Copy data from user failed");
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);

	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
			VTE("Alloc memory failed");
			return -ENOMEM;
	}
	ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ret = -EFAULT;
		VTE("Copy data from user failed");
		goto err_out;
	}

	if (hw_ops->write_trans(ts_dev, reg_addr, databuf, length)) {
		ret = -EBUSY;
		VTE("Write data to device failed");
	} else {
		ret = length;
	}

err_out:
	kfree(databuf);
	return ret;
}

static int init_cfg_data(struct goodix_ts_config *cfg, void __user *arg)
{
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];

	cfg->initialized = 0;
	mutex_init(&cfg->lock);
	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		VTE("Copy data from user failed");
		return -EFAULT;
	}

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);

	ret = copy_from_user(cfg->data, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ret = -EFAULT;
		VTE("Copy data from user failed");
		goto err_out;
	}
	cfg->reg_base = reg_addr;
	cfg->length = length;
	strlcpy(cfg->name, "tools-send-cfg", sizeof(cfg->name));
	cfg->delay = 50;
	cfg->initialized = true;
	return 0;

err_out:
	return ret;
}
/* get module info
 * success return bytes read, else return <= 0
 */
static int get_module_info(struct goodix_tools_dev_V2*dev, void __user *arg)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 length;
	u8 msg_head[I2C_MSG_HEAD_LEN];
	struct goodix_ts_device *ts_dev = dev->ts_core->ts_dev;
	u8 pid_len = (u8)(sizeof(ts_dev->chip_version.pid));
	//u8 lcmid_len = (u8)(sizeof(dev->ts_core->panel_lcmid));
	//int lcmid = dev->ts_core->panel_lcmid;
	int lcmid = 0xff;
	u8 lcmid_len = (u8)(sizeof(int));
	int info_len = pid_len+lcmid_len+2;
	vts_get_lcmid(ts_dev->vtsdev, &lcmid);
	VTI("lcmid is :%d",lcmid);
	ret = copy_from_user(&msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		ret = -EFAULT;
		goto err_out;
	}
	length = msg_head[0] + (msg_head[1] << 8)
			+ (msg_head[2] << 16) + (msg_head[3] << 24);
	if (length < info_len +I2C_MSG_HEAD_LEN) {
		VTE("module len is short");
		return -EINVAL;
	}
	
	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		VTE("Alloc memory failed");
		return -ENOMEM;
	}
	databuf[0] = pid_len;
	memcpy(&databuf[1], ts_dev->chip_version.pid, pid_len);
	databuf[pid_len + 1] = lcmid_len;
	
	databuf[pid_len + 2] = lcmid&0xff;
	databuf[pid_len + 3] = (lcmid>> 8)&0xff;
	databuf[pid_len + 4] = (lcmid>> 16)&0xff;
	databuf[pid_len + 5] = (lcmid>> 24)&0xff;

	ret = copy_to_user((u8 *)arg+ I2C_MSG_HEAD_LEN, databuf, info_len);
	if (ret) {
		ret = -EFAULT;
		VTE("Copy_to_user failed");
	} else {
		ret = info_len;
	}

err_out:
	kfree(databuf);
	return ret;
}

/**
 * goodix_tools_ioctl - ioctl implementation
 *
 * @filp: Pointer to file opened
 * @cmd: Ioctl opertion command
 * @arg: Command data
 * Returns >=0 - succeed, else failed
 */
static long goodix_tools_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	int ret = 0;
	struct goodix_tools_dev_V2 *dev = filp->private_data;
	struct goodix_ts_device *ts_dev;
	const struct goodix_ts_hw_ops *hw_ops;
	struct goodix_ts_cmd temp_cmd;
	struct goodix_ts_config *temp_cfg = NULL;

	if (dev->ts_core == NULL) {
		VTE("Tools module not register");
		return -EINVAL;
	}
	ts_dev = dev->ts_core->ts_dev;
	hw_ops = ts_dev->hw_ops;

	if (_IOC_TYPE(cmd) != GOODIX_TS_IOC_MAGIC) {
		VTE("Bad magic num:%c", _IOC_TYPE(cmd));
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > GOODIX_TS_IOC_MAXNR) {
		VTE("Bad cmd num:%d > %d",
		       _IOC_NR(cmd), GOODIX_TS_IOC_MAXNR);
		return -ENOTTY;
	}

	switch (cmd & NEGLECT_SIZE_MASK) {
	case GTP_IRQ_ENABLE:
		if (arg == 1) {
			goodix_ts_irq_enable_V2(dev->ts_core, true);
			mutex_lock(&dev->mutex);
			dev->ops_mode |= IRQ_FALG;
			mutex_unlock(&dev->mutex);
			VTI("IRQ enabled");
		} else if (arg == 0) {
			goodix_ts_irq_enable_V2(dev->ts_core, false);
			mutex_lock(&dev->mutex);
			dev->ops_mode &= ~IRQ_FALG;
			mutex_unlock(&dev->mutex);
			VTI("IRQ disabled");
		} else {
			VTI("Irq aready set with, arg = %ld", arg);
		}
		ret = 0;
		break;
	case GTP_ESD_ENABLE:
		if (arg == 0)
			goodix_ts_blocking_notify_V2(NOTIFY_ESD_OFF, NULL);
		else
			goodix_ts_blocking_notify_V2(NOTIFY_ESD_ON, NULL);
		break;
	case GTP_DEV_RESET:
		hw_ops->reset(ts_dev);
		break;
	case GTP_SEND_COMMAND:
		ret = copy_from_user(&temp_cmd, (void __user *)arg,
					sizeof(struct goodix_ts_cmd));
		if (ret) {
			ret = -EINVAL;
			goto err_out;
		}

		ret = hw_ops->send_cmd(ts_dev, &temp_cmd);
		if (ret) {
			VTE("Send command failed");
			ret = -EINVAL;
		}
		break;
	case GTP_SEND_CONFIG:
		temp_cfg = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
		if (temp_cfg == NULL) {
			VTE("Memory allco err");
			ret = -ENOMEM;
			goto err_out;
		}

		ret = init_cfg_data(temp_cfg, (void __user *)arg);

		if (!ret && hw_ops->send_config) {
			ret = hw_ops->send_config(ts_dev, temp_cfg);
			if (ret) {
				VTE("Failed send config");
				ret = -EINVAL;
			} else {
				VTI("Send config success");
				ret = 0;
			}
		}
		break;
	case GTP_READ_CONFIG:
		ret = read_config_data(ts_dev, (void __user *)arg);
		if (ret > 0)
			VTI("success read config:len=%d", ret);
		else
			VTE("failed read config:ret=0x%x", ret);
		break;
	case GTP_ASYNC_READ:
		ret = async_read(dev, (void __user *)arg);
		if (ret < 0)
			VTE("Async data read failed");
		break;
	case GTP_SYNC_READ:
		if (filp->f_flags & O_NONBLOCK) {
			VTE("Goodix tools now worked in sync_bus mode");
			ret = -EINVAL;
			goto err_out;
		}
		ret = sync_read(dev, (void __user *)arg);
		if (ret < 0)
			VTE("Sync data read failed");
		break;
	case GTP_ASYNC_WRITE:
		ret = async_write(dev, (void __user *)arg);
		if (ret < 0)
			VTE("Async data write failed");
		break;
	case GTP_DRV_VERSION:
		ret = copy_to_user((u8 *)arg, GOODIX_DRIVER_VERSION,
				   sizeof(GOODIX_DRIVER_VERSION));
		if (ret)
			VTE("failed copy driver version info to user");
		break;
	case GTP_DRV_CMD_ENABLE:
		if(0 == arg){//disable drv cmd
           ts_dev->goodix_sensor_test = 1;
		   VTI("goodix sensor test start");
		}else{
		   ts_dev->goodix_sensor_test = 0;
		    VTI("goodix sensor test end");
		}
		break;
		//Modify by yzt for ruohua vivo
	case GTP_MODULE_INFO_READ: //_IOR(GOODIX_TS_IOC_MAGIC, 11)
		ret = get_module_info(dev, (void __user *)arg);
		if (ret < 0)
			VTE("failed get module info!");
		break;
	default:
		VTI("Invalid cmd");
		ret = -ENOTTY;
		break;
	}

err_out:
	if (!temp_cfg) {
		kfree(temp_cfg);
		temp_cfg = NULL;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long goodix_tools_compat_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static int goodix_tools_open(struct inode *inode, struct file *filp)
{
	filp->private_data = goodix_tools_dev_V2;
	
	VTI("tools open");

	return 0;
}

static int goodix_tools_release(struct inode *inode, struct file *filp)
{
	VTI("tools closed");
	
	return 0;
}

/**
 * goodix_tools_module_irq - goodix tools Irq handle
 * This functions is excuted when interrupt happended
 *
 * @core_data: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: EVT_CONTINUE let other module handle this irq
 */
static int goodix_tools_module_irq(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	struct goodix_tools_dev_V2 *dev = module->priv_data;
	struct goodix_ts_device *ts_dev = dev->ts_core->ts_dev;
	const struct goodix_ts_hw_ops *hw_ops = ts_dev->hw_ops;
	struct goodix_tools_data *tools_data;
	int r = 0;
	u8 evt_sta = 0;

	if (!list_empty(&dev->head)) {
		r = hw_ops->read_trans(ts_dev, ts_dev->reg.coor/* TS_REG_COORDS_BASE*/, &evt_sta, 1);
		if (r < 0 || ((evt_sta & 0x80) == 0)) {
			VTD("data not ready:0x%x, read ret =%d", evt_sta, r);
			return EVT_CONTINUE;
		}

		mutex_lock(&dev->mutex);
		list_for_each_entry(tools_data, &dev->head, list) {
			if (!hw_ops->read_trans(ts_dev, tools_data->reg_addr,
					tools_data->data, tools_data->length)) {
				tools_data->filled = 1;
			}
		}
		mutex_unlock(&dev->mutex);
		wake_up(&dev->wq);
	}
	return EVT_CONTINUE;
}

static int goodix_tools_module_init(struct goodix_ts_core *core_data,
			struct goodix_ext_module *module)
{
	struct goodix_tools_dev_V2 *tools_dev = module->priv_data;

	if (core_data)
		tools_dev->ts_core = core_data;
	else
		return -ENODEV;

	return 0;
}

static const struct file_operations goodix_tools_fops = {
	.owner		= THIS_MODULE,
	.open		= goodix_tools_open,
	.release	= goodix_tools_release,
	.unlocked_ioctl	= goodix_tools_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = goodix_tools_compat_ioctl,
#endif
};

static struct miscdevice goodix_tools_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= GOODIX_TOOLS_NAME,
	.fops	= &goodix_tools_fops,
};

static struct goodix_ext_module_funcs goodix_tools_module_funcs = {
	.irq_event = goodix_tools_module_irq,
	.init = goodix_tools_module_init,
};

/**
 * goodix_tools_init - init goodix tools device and register a miscdevice
 *
 * return: 0 success, else failed
 */
int goodix_tools_V2_init(void)
{
	int ret;

	goodix_tools_dev_V2 = kzalloc(sizeof(struct goodix_tools_dev_V2), GFP_KERNEL);
	if (goodix_tools_dev_V2 == NULL) {
		VTE("Memory allco err");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&goodix_tools_dev_V2->head);
	goodix_tools_dev_V2->ops_mode = 0;
	goodix_tools_dev_V2->ops_mode |= IRQ_FALG;
	init_waitqueue_head(&goodix_tools_dev_V2->wq);
	mutex_init(&goodix_tools_dev_V2->mutex);
	atomic_set(&goodix_tools_dev_V2->t_count, 0);

	goodix_tools_dev_V2->module.funcs = &goodix_tools_module_funcs;
	goodix_tools_dev_V2->module.name = GOODIX_TOOLS_NAME;
	goodix_tools_dev_V2->module.priv_data = goodix_tools_dev_V2;
	goodix_tools_dev_V2->module.priority = EXTMOD_PRIO_DBGTOOL;

	ret = misc_register(&goodix_tools_miscdev);
	if (ret)
		VTE("Debug tools miscdev register failed");

	ret = goodix_register_ext_module_V2(&goodix_tools_dev_V2->module);
	if (ret)
		VTE("failed register to core module");

	VTI("Goodix tools miscdev init sucessfully");

	return ret;
}

void goodix_tools_V2_exit(void)
{
	misc_deregister(&goodix_tools_miscdev);
	kfree(goodix_tools_dev_V2);
	VTI("Goodix tools miscdev exit");
}
