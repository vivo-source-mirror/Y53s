/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#define pr_fmt(fmt) "<SAR_FAC>" fmt

#include "sar_secondary_factory.h"

struct sar_secondary_factory_private {
	uint32_t gain;
	uint32_t sensitivity;
	struct sar_secondary_factory_fops *fops;
};

static struct sar_secondary_factory_private sar_secondary_factory;

static int sar_secondary_factory_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int sar_secondary_factory_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long sar_secondary_factory_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int data = 0;
	uint32_t flag = 0;
	int32_t data_buf[6] = {0};
	uint32_t cmd_args[VSEN_COMMAND_ARGS_SIZE] = {0};
	pr_err("sar_secondary_factory_unlocked_ioctl cmd:%d\n", cmd);

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (err) {
		pr_err("access error: %08X, (%2d, %2d)\n", cmd,
			    _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case SAR_B_IOCTL_INIT:
		pr_err("SAR_B_IOCTL_INIT\n");
		if (copy_from_user(&flag, ptr, sizeof(flag)))
			return -EFAULT;
		if (sar_secondary_factory.fops != NULL &&
		    sar_secondary_factory.fops->enable_sensor != NULL) {
			err = sar_secondary_factory.fops->enable_sensor(flag, 200);
			if (err < 0) {
				pr_err("SAR_B_IOCTL_INIT fail!\n");
				return -EINVAL;
			}
			pr_debug(
				"SAR_B_IOCTL_INIT, enable: %d, sample_period:%dms\n",
				flag, 200);
		} else {
			pr_err("SAR_B_IOCTL_INIT NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_READ_SENSORDATA:
		pr_err("SAR_IOCTL_READ_SENSORDATA\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_GET_DIFF_OFFSET;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, ARRAY_SIZE(cmd_args));
			if (err < 0) {
				pr_err("SAR_B_IOCTL_READ_SENSORDATA fail!\n");
				return -EINVAL;
			}
			data_buf[0] = cmd_args[1];
			data_buf[1] = cmd_args[2];
			data_buf[2] = cmd_args[3];
			data_buf[3] = cmd_args[4];
			data_buf[4] = cmd_args[5];
			data_buf[5] = cmd_args[6];
			pr_err("SENSOR_COMMAND_SAR_B_GET_DIFF_OFFSET %d %d %d %d %d %d\n", cmd_args[1], cmd_args[2], cmd_args[3],
				cmd_args[4], cmd_args[5], cmd_args[6]);
			if (copy_to_user(ptr, &data_buf, sizeof(data_buf)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_GET_DIFF_OFFSET NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_ENABLE_CALI: /*compensation cali*/
		pr_err("SAR_B_IOCTL_ENABLE_CALI\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			if (copy_from_user(&data, ptr, sizeof(data)))
				return -EFAULT;
			cmd_args[0] = SENSOR_COMMAND_SAR_B_ENABLE_CALI;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_ENABLE_CALI fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("SENSOR_COMMAND_SAR_B_ENABLE_CALI %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_ENABLE_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_GET_CALI: /*get compensation cali result*/
		pr_err("SAR_B_IOCTL_GET_CALI\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_GET_CALI;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_GET_CALI fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("SENSOR_COMMAND_SAR_B_GET_CALI %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_GET_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_READ_REG: /*read sar sensor registers*/
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_READ_REG;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_READ_REG fail!\n");
				return -EINVAL;
			}
			pr_info("SENSOR_COMMAND_SAR_B_READ_REG result reg: 0x%x val: 0x%x\n", cmd_args[1], cmd_args[2]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_READ_REG NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_WRITE_REG: /*read sar sensor registers*/
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_WRITE_REG;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_WRITE_REG fail!\n");
				return -EINVAL;
			}
			pr_info("SENSOR_COMMAND_SAR_B_WRITE_REG result reg: 0x%x val: 0x%x\n", cmd_args[1], cmd_args[2]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_WRITE_REG NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_GET_NEAR_FAR: /*read sar sensor registers*/
		pr_err("SAR_B_IOCTL_GET_NEAR_FAR\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_GET_NEAR_FAR;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_GET_NEAR_FAR fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("SENSOR_COMMAND_SAR_B_GET_NEAR_FAR %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_GET_NEAR_FAR NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_FORCE_TO_NEAR: /*read sar sensor registers*/
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_err("SAR_B_IOCTL_FORCE_TO_NEAR type:%d\n", cmd_args[1]);
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_FORCE_TO_NEAR;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NEAR fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NEAR %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NEAR NULL\n");
			return -EINVAL;
		}
		return 0;
	case SAR_B_IOCTL_FORCE_TO_NOSAR:
		pr_err("SAR_B_IOCTL_FORCE_TO_NOSAR\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_SAR_B_FORCE_TO_NOSAR;
			err = sar_secondary_factory.fops->do_vsen_commands(ID_SAR_SECONDARY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NOSAR fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NOSAR %d\n", data);
		} else {
			pr_err("SENSOR_COMMAND_SAR_B_FORCE_TO_NOSAR NULL\n");
			return -EINVAL;
		}
		return 0;

	case SAR_B_IOCTL_SELF_TEST:
		pr_err("SAR_B_IOCTL_SELF_TEST\n");
		if (sar_secondary_factory.fops != NULL &&
			sar_secondary_factory.fops->do_self_test != NULL) {
			data = sar_secondary_factory.fops->do_self_test();
			if (data < 0) {
				pr_err(
					"SAR_B_IOCTL_SELF_TEST FAIL!\n");
				return -EINVAL;
			}
			pr_info("SAR_B_IOCTL_SELF_TEST result %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("SAR_B_IOCTL_SELF_TEST NULL\n");
			return -EINVAL;
		}
		return 0;
	default:
		pr_err("unknown IOCTL: 0x%08x\n", cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_sar_secondary_factory_unlocked_ioctl(struct file *filp,
					       unsigned int cmd,
					       unsigned long arg)
{
	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		pr_err(
			"compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_SAR_B_IOCTL_INIT:
	case COMPAT_SAR_B_IOCTL_READ_SENSORDATA:
	case COMPAT_SAR_B_IOCTL_ENABLE_CALI:
	case COMPAT_SAR_B_IOCTL_GET_CALI: {
		pr_debug(
			"compat_ion_ioctl : SAR_B_IOCTL_XXX command is 0x%x\n",
			cmd);
		return filp->f_op->unlocked_ioctl(
			filp, cmd, (unsigned long)compat_ptr(arg));
	}
	default:
		pr_err("compat_ion_ioctl : No such command!! 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations _sar_secondary_factory_fops = {
	.open = sar_secondary_factory_open,
	.release = sar_secondary_factory_release,
	.unlocked_ioctl = sar_secondary_factory_unlocked_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_sar_secondary_factory_unlocked_ioctl,
#endif
};

static struct miscdevice _sar_secondary_factory_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sarsecondary",
	.fops = &_sar_secondary_factory_fops,
};

int sar_secondary_factory_device_register(struct sar_secondary_factory_public *dev)
{
	int err = 0;

	if (!dev || !dev->fops)
		return -1;
	sar_secondary_factory.gain = dev->gain;
	sar_secondary_factory.sensitivity = dev->sensitivity;
	sar_secondary_factory.fops = dev->fops;
	err = misc_register(&_sar_secondary_factory_device);
	if (err) {
		pr_err("sar_secondary_factory_device register failed\n");
		err = -1;
	}
	return err;
}

int sar_secondary_factory_device_deregister(struct sar_secondary_factory_public *dev)
{
	sar_secondary_factory.fops = NULL;
	misc_deregister(&_sar_secondary_factory_device);
	return 0;
}
