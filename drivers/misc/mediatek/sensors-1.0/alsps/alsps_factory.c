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

#define pr_fmt(fmt) "<ALS/PS> " fmt

#include "inc/alsps_factory.h"

struct alsps_factory_private {
	uint32_t gain;
	uint32_t sensitivity;
	struct alsps_factory_fops *fops;
};

static struct alsps_factory_private alsps_factory;

static int alsps_factory_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int alsps_factory_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long alsps_factory_unlocked_ioctl(struct file *file, unsigned int cmd,
					 unsigned long arg)
{
	long err = 0;
	void __user *ptr = (void __user *)arg;
	int data = 0;
	int psData[3] = {0};
	uint32_t enable = 0;
	int threshold_data[2] = {0, 0};
	int als_cali = 0;
	uint32_t cmd_args[VSEN_COMMAND_ARGS_SIZE] = {0};
	int brightness = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg,
				 _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg,
				 _IOC_SIZE(cmd));

	if (err) {
		pr_debug("access error: %08X, (%2d, %2d)\n", cmd,
			  _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_enable_sensor != NULL) {
			err = alsps_factory.fops->ps_enable_sensor(enable, 200);
			if (err < 0) {
				pr_err("ALSPS_SET_PS_MODE fail!\n");
				return -EINVAL;
			}
			pr_debug(
				"ALSPS_SET_PS_MODE, enable: %d, sample_period:%dms\n",
				enable, 200);
		} else {
			pr_err("ALSPS_SET_PS_MODE NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_GET_PS_RAW_DATA:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_get_raw_data != NULL) {
			err = alsps_factory.fops->ps_get_raw_data(psData);
			if (err < 0) {
				pr_err(
					"ALSPS_GET_PS_RAW_DATA read data fail!\n");
				return -EINVAL;
			}
			if (copy_to_user(ptr, &psData, sizeof(psData)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_GET_PS_RAW_DATA NULL\n");
			return -EINVAL;
		}
		pr_err("ALSPS_GET_PS_RAW_DATA: %d %d %d\n", psData[0], psData[1], psData[2]);
		return 0;
	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->als_enable_sensor != NULL) {
			err = alsps_factory.fops->als_enable_sensor(enable,
								    200);
			if (err < 0) {
				pr_err("ALSPS_SET_ALS_MODE fail!\n");
				return -EINVAL;
			}
			pr_debug(
				"ALSPS_SET_ALS_MODE, enable: %d, sample_period:%dms\n",
				enable, 200);
		} else {
			pr_err("ALSPS_SET_ALS_MODE NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_GET_ALS_RAW_DATA:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->als_get_raw_data != NULL) {
			err = alsps_factory.fops->als_get_raw_data(&data);
			if (err < 0) {
				pr_err(
					"ALSPS_GET_ALS_RAW_DATA read data fail!\n");
				return -EINVAL;
			}

			pr_err("alsps ALSPS_GET_ALS_RAW_DATA data = %d\n", data);

			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_GET_ALS_RAW_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_ALS_ENABLE_CALI:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->als_enable_calibration != NULL) {
			err = alsps_factory.fops->als_enable_calibration();
			if (err < 0) {
				pr_err("ALSPS_ALS_ENABLE_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_ALS_ENABLE_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_ALS_SET_CALI:
		if (copy_from_user(&als_cali, ptr, sizeof(als_cali)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->als_set_cali != NULL) {
			err = alsps_factory.fops->als_set_cali(als_cali);
			if (err < 0) {
				pr_err("ALSPS_ALS_SET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_ALS_SET_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_ALS_REG_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_GET_ALS_REG_DATA reg 0x%x\n", cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_READ_REG_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_ALS_REG_DATA read data fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_GET_ALS_REG_DATA reg: 0x%x val: 0x%x\n", cmd_args[1], cmd_args[2]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_ALS_REG_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_ALS_REG_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_SET_ALS_REG_DATA reg 0x%x val 0x%x\n", cmd_args[1], cmd_args[2]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_WRITE_REG_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_ALS_REG_DATA read data fail!\n");
				return -EINVAL;
			}
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_SET_ALS_REG_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_GET_PS_TEST_RESULT:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_get_data != NULL) {
			err = alsps_factory.fops->ps_get_data(&data);
			if (err < 0) {
				pr_err(
					"ALSPS_GET_PS_TEST_RESULT read data fail!\n");
				return -EINVAL;
			}
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_GET_PS_TEST_RESULT NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_GET_PS_THRESHOLD_HIGH:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_get_threshold != NULL) {
			err = alsps_factory.fops->ps_get_threshold(
				threshold_data);
			if (err < 0) {
				pr_err(
					"ALSPS_GET_PS_THRESHOLD_HIGH read data fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_GET_PS_THRESHOLD_HIGH NULL\n");
			return -EINVAL;
		}
		if (copy_to_user(ptr, &threshold_data[0],
				 sizeof(threshold_data[0])))
			return -EFAULT;
		return 0;
	case ALSPS_GET_PS_THRESHOLD_LOW:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_get_threshold != NULL) {
			err = alsps_factory.fops->ps_get_threshold(
				threshold_data);
			if (err < 0) {
				pr_err(
					"ALSPS_GET_PS_THRESHOLD_HIGH read data fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_GET_PS_THRESHOLD_HIGH NULL\n");
			return -EINVAL;
		}
		if (copy_to_user(ptr, &threshold_data[1],
				 sizeof(threshold_data[1])))
			return -EFAULT;
		return 0;
	case ALSPS_SET_PS_THRESHOLD:
		if (copy_from_user(threshold_data, ptr, sizeof(threshold_data)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_set_threshold != NULL) {
			err = alsps_factory.fops->ps_set_threshold(
				threshold_data);
			if (err < 0) {
				pr_err("ALSPS_SET_PS_THRESHOLD fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_SET_PS_THRESHOLD NULL\n");
			return -EINVAL;
		}
		return 0;
	/* Remove by vsen team */
	/* case ALSPS_IOCTL_SET_CALI:
		if (copy_from_user(&data, ptr, sizeof(data)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_set_cali != NULL) {
			err = alsps_factory.fops->ps_set_cali(data);
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_CALI fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_IOCTL_SET_CALI NULL\n");
			return -EINVAL;
		}
		return 0; */
	/* Remove by vsen team */
	case ALSPS_IOCTL_GET_CALI:
		if (alsps_factory.fops != NULL &&
		    alsps_factory.fops->ps_get_cali != NULL) {
			err = alsps_factory.fops->ps_get_cali(&data);
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_IOCTL_GET_CALI NULL\n");
			return -EINVAL;
		}
		if (copy_to_user(ptr, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	case ALSPS_IOCTL_ALS_GET_CALI:
		if (alsps_factory.fops != NULL &&
			alsps_factory.fops->als_get_cali != NULL) {
			err = alsps_factory.fops->als_get_cali(&data);
			if (err < 0) {
				pr_err("ALSPS_IOCTL_ALS_GET_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_IOCTL_ALS_GET_CALI NULL\n");
			return -EINVAL;
		}
		if (copy_to_user(ptr, &data, sizeof(data)))
			return -EFAULT;
		return 0;
	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&data, ptr, sizeof(data)))
			return -EFAULT;
		if (alsps_factory.fops != NULL &&
			alsps_factory.fops->ps_clear_cali != NULL) {
			err = alsps_factory.fops->ps_clear_cali();
			if (err < 0) {
				pr_err("ALSPS_IOCTL_CLR_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_IOCTL_CLR_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_PS_ENABLE_CALI:
		if (alsps_factory.fops != NULL &&
			alsps_factory.fops->ps_enable_calibration != NULL) {
			err = alsps_factory.fops->ps_enable_calibration();
			if (err < 0) {
				pr_err("ALSPS_PS_ENABLE_CALI FAIL!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_PS_ENABLE_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	/* ADD by vsen team begain*/
	case ALSPS_IOCTL_CHECK_PS_INT:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_CHECK_INT;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_CHECK_PS_INT read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_CHECK_PS_INT NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_READ_PS_INT:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_READ_INT;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_READ_PS_INT read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_info("ALSPS_IOCTL_READ_PS_INT %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_READ_PS_INT NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_PS_DATA_RANGE:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_GET_DATA_RANGE;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_PS_DATA_RANGE read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_info("ALSPS_IOCTL_GET_PS_DATA_RANGE %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_PS_DATA_RANGE NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_PS_STATUS:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_GET_STATUS;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_PS_STATUS read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_info("ALSPS_IOCTL_GET_PS_STATUS %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_PS_STATUS NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_PS_REG_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_GET_PS_REG_DATA reg 0x%x\n", cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_READ_REG_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_PS_REG_DATA read data fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_GET_PS_REG_DATA reg: 0x%x val: 0x%x\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_PS_REG_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_PS_REG_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_SET_PS_REG_DATA reg 0x%x val 0x%x\n", cmd_args[1], cmd_args[2]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_WRITE_REG_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_PS_REG_DATA read data fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_SET_PS_REG_DATA reg: 0x%x val: 0x%x\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_SET_PS_REG_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_PS_ENG_CALI_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_SET_PS_ENG_CALI_DATA %d\n", cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_ENG_CALI_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_PS_ENG_CALI_DATA fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_SET_PS_ENG_CALI_DATA args:(%d ,%d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_SET_PS_ENG_CALI_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_NOTIFY_PS_THRES_LEVEL:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_NOTIFY_PS_THRES_LEVEL %d\n", cmd_args[0]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_NOTIFY_THRES_LEVEL;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_NOTIFY_PS_THRES_LEVEL fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_NOTIFY_PS_THRES_LEVEL args:(%d ,%d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_NOTIFY_PS_THRES_LEVEL NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_PS_PARA_INDEX:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			if (copy_from_user(&data, ptr, sizeof(data)))
				return -EFAULT;
			cmd_args[0] = SENSOR_COMMAND_PROX_GET_PARA_INDEX;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_PS_PARA_INDEX read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("ALSPS_IOCTL_GET_PS_PARA_INDEX %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_PS_PARA_INDEX NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_ALS_PARA_INDEX:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_GET_PARA_INDEX;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_ALS_PARA_INDEX read data fail!\n");
				return -EINVAL;
			}
			data = cmd_args[1];
			pr_err("ALSPS_IOCTL_GET_ALS_PARA_INDEX %d\n", data);
			if (copy_to_user(ptr, &data, sizeof(data)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_ALS_PARA_INDEX NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_BOOST_ALS_REPORT:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_BOOST_ALS_REPORT %d\n", cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_BOOST_NOTIRY;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_BOOST_ALS_REPORT fail!\n");
				return -EINVAL;
			}
			pr_err("ALSPS_IOCTL_BOOST_ALS_REPORT args:(%d ,%d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_BOOST_ALS_REPORT NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_PS_CHIP_RESET:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_CHIP_RESET;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_PROX_CHIP_RESET read data fail!\n");
				return -EINVAL;
			}
			pr_err("SENSOR_COMMAND_PROX_CHIP_RESET %d\n", data);
		} else {
			pr_err("SENSOR_COMMAND_PROX_CHIP_RESET NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_PS_PRE_CALIBRATION:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_PRE_CALIBRATION;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_PS_PRE_CALIBRATION read data fail!\n");
				return -EINVAL;
			}
			pr_err("ALSPS_IOCTL_SET_PS_PRE_CALIBRATION %d\n", data);
		} else {
			pr_err("ALSPS_IOCTL_SET_PS_PRE_CALIBRATION NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_PS_PRE_CALIBRATION:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_GET_PS_PRE_CALIBRATION %d\n", cmd_args[0]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_GET_PRE_CALIBRATION;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_GET_PS_PRE_CALIBRATION fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_GET_PS_PRE_CALIBRATION args:(%d,0x%x)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_GET_PS_PRE_CALIBRATION NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_PS_CALI_OFFSET_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_SET_PS_CALI_OFFSET_DATA %d %d\n", cmd_args[0], cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_CALI_OFFSET_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_PS_CALI_OFFSET_DATA fail!\n");
				return -EINVAL;
			}
			pr_info("ALSPS_IOCTL_SET_PS_CALI_OFFSET_DATA args:(%d,%d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_IOCTL_SET_PS_CALI_OFFSET_DATA NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_SET_CALI:
		if (copy_from_user(&data, ptr, sizeof(data)))
			return -EFAULT;
		pr_info("ALSPS_IOCTL_SET_CALI cali data %d\n", data);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_CALI_DATA;
			cmd_args[1] = data;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_IOCTL_SET_CALI read data fail!\n");
				return -EINVAL;
			}
		} else {
			pr_err("ALSPS_IOCTL_SET_CALI NULL\n");
			return -EINVAL;
		}
		return 0;
	case ALSPS_IOCTL_GET_ALS_CHANNEL_DATA:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_RAW_DATA;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("ALSPS_GET_ALS_RAW_DATA fail!\n");
				return -EINVAL;
			}
			pr_err("ALSPS_GET_ALS_RAW_DATA args:(%d ,%d, %d, %d, %d, %d)\n", cmd_args[0], cmd_args[1], cmd_args[2], cmd_args[3], cmd_args[4], cmd_args[5]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_GET_ALS_RAW_DATA NULL\n");
			return -EINVAL;
		}
		return 0;

	case ALSPS_IOCTL_SET_PS_DEFAULT_CHNANEL:
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_DEFAUT_CHANEL;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_PROX_SET_DEFAUT_CHANEL fail!\n");
				return -EINVAL;
			}
			pr_err("SENSOR_COMMAND_PROX_SET_DEFAUT_CHANEL args:(%d %d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("SENSOR_COMMAND_PROX_SET_DEFAUT_CHANEL NULL\n");
			return -EINVAL;
		}

		return 0;

	case ALSPS_IOCTL_SET_PS_PRE_CALI:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_PRE_CALI;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_PROX_SET_PRE_CALI fail!\n");
				return -EINVAL;
			}
			pr_err("SENSOR_COMMAND_PROX_SET_PRE_CALI args:(%d %d)\n", cmd_args[0], cmd_args[1]);
			if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
				return -EFAULT;
		} else {
			pr_err("ALSPS_GET_ALS_RAW_DATA NULL\n");
			return -EINVAL;
		}

		return 0;

	case ALSPS_IOCTL_SET_NOTIFY_BRIGHTNESS:
		if (copy_from_user(&brightness, ptr, sizeof(brightness)))
			return -EFAULT;

		pr_err("ALSPS_IOCTL_SET_NOTIFY_BRIGHTNESS brightness = %d", brightness);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_LIGHT_SET_NOTIFY_BRIGHTNESS;
			cmd_args[1] = brightness;
			err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
			if (err < 0) {
				pr_err("SENSOR_COMMAND_ALS_SET_NOTIFY_BRIGHTNESS fail!\n");
				return -EINVAL;
			}
			pr_err("SENSOR_COMMAND_ALS_SET_NOTIFY_BRIGHTNESS success\n");
		}
		return 0;

	case ALSPS_IOCTL_SET_ENG_MODE:
		if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
			return -EFAULT;
		ALSPS_PR_ERR("ALSPS_IOCTL_SET_ENG_MODE %d %d\n", cmd_args[0], cmd_args[1]);
		if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
			cmd_args[0] = SENSOR_COMMAND_PROX_SET_ENG_MODE;
			err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, ARRAY_SIZE(cmd_args));
			if (err < 0) {
				ALSPS_PR_ERR("SENSOR_COMMAND_PROX_SET_ENG_MODE fail!\n");
				return -EINVAL;
			}
			ALSPS_PR_ERR("SENSOR_COMMAND_PROX_SET_ENG_MODE args:(%d ,%d)\n", cmd_args[0], cmd_args[1]);
		} else {
			ALSPS_PR_ERR("SENSOR_COMMAND_PROX_SET_ENG_MODE NULL\n");
			return -EINVAL;
		}
		return 0;

		case ALSPS_IOCTL_NOTIFY_PS_TEMP_CALI:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_TEMP_CALI %d %d\n", cmd_args[0], cmd_args[1]);
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_PROX_NOTIFY_TEMP_CALI;
				err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, ARRAY_SIZE(cmd_args));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_TEMP_CALI fail!\n");
					return -EINVAL;
				}
				copy_to_user(ptr, &cmd_args, sizeof(cmd_args));
				ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_TEMP_CALI args:(%d ,%d, %d)\n", cmd_args[0], cmd_args[1], cmd_args[2]);
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_TEMP_CALI NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_NOTIFY_PS_BROKEN:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_BROKEN %d %d\n", cmd_args[0], cmd_args[1]);
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_PROX_NOTIFY_PS_BROKEN;
				err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, ARRAY_SIZE(cmd_args));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_BROKEN fail!\n");
					return -EINVAL;
				}
				copy_to_user(ptr, &cmd_args, sizeof(cmd_args));
				ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_BROKEN args:(%d ,%d, %d)\n", cmd_args[0], cmd_args[1], cmd_args[2]);
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_PS_BROKEN NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_SET_ALS_SAMPLE_PARAM_MODE:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			ALSPS_PR_ERR("ALSPS_IOCTL_SET_ALS_SAMPLE_PARAM_MODE %d %d\n", cmd_args[0], cmd_args[1]);
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_LIGHT_SET_SAMPLE_PARAM_MODE;
				err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, ARRAY_SIZE(cmd_args));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_SET_ALS_SAMPLE_PARAM_MODE fail!\n");
					return -EINVAL;
				}
				copy_to_user(ptr, &cmd_args, sizeof(cmd_args));
				ALSPS_PR_ERR("ALSPS_IOCTL_SET_ALS_SAMPLE_PARAM_MODE args:(%d ,%d, %d)\n", cmd_args[0], cmd_args[1]);
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_SET_ALS_SAMPLE_PARAM_MODE NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_GET_ALS_SAMPLE_PARAM_DATA:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			ALSPS_PR_ERR("ALSPS_IOCTL_GET_ALS_SAMPLE_PARAM_DATA %d %d\n", cmd_args[0], cmd_args[1]);
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_LIGHT_GET_SAMPLE_PARAM_DATA;
				err = alsps_factory.fops->do_vsen_commands(ID_LIGHT, cmd_args, ARRAY_SIZE(cmd_args));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_GET_ALS_SAMPLE_PARAM_DATA fail!\n");
					return -EINVAL;
				}
				copy_to_user(ptr, &cmd_args, sizeof(cmd_args));
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_ALS_SAMPLE_PARAM_DATA args:(%d ,%d, %d)\n", cmd_args[0], cmd_args[1], cmd_args[2]);
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_ALS_SAMPLE_PARAM_DATA NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_GET_PS_OFFSET_VALUE:
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_PROXIMITY_GET_CALIOFFSET;
				err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_OFFSET_VALUE read data fail!\n");
					return -EINVAL;
				}
				data = cmd_args[1];
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_OFFSET_VALUE %d\n", data);
				if (copy_to_user(ptr, &data, sizeof(data)))
					return -EFAULT;
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_OFFSET_VALUE NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_NOTIFY_POWER_LEVEL:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			ALSPS_PR_ERR("ALSPS_IOCTL_NOTIFY_POWER_LEVEL %d %d\n", cmd_args[0], cmd_args[1]);
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_PROXIMITY_POWER_LEVEL;
				err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, ARRAY_SIZE(cmd_args));
				if (err < 0) {
					ALSPS_PR_ERR("SENSOR_COMMAND_PROXIMITY_POWER_LEVEL fail!\n");
					return -EINVAL;
				}
				ALSPS_PR_ERR("SENSOR_COMMAND_PROXIMITY_POWER_LEVEL args:(%d ,%d)\n", cmd_args[0], cmd_args[1]);
			} else {
				ALSPS_PR_ERR("SENSOR_COMMAND_PROXIMITY_POWER_LEVEL NULL\n");
				return -EINVAL;
			}
			return 0;

		case ALSPS_IOCTL_GET_PS_RECALI_RESULT:
			if (copy_from_user(&cmd_args, ptr, sizeof(cmd_args)))
				return -EFAULT;
			if (alsps_factory.fops != NULL && alsps_factory.fops->do_vsen_commands != NULL) {
				cmd_args[0] = SENSOR_COMMAND_PROX_GET_RECALI_RESULT;
				err = alsps_factory.fops->do_vsen_commands(ID_PROXIMITY, cmd_args, sizeof(cmd_args)/sizeof(cmd_args[0]));
				if (err < 0) {
					ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_RECALI_RESULT fail!\n");
					return -EINVAL;
				}
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_RECALI_RESULT args:(%d ,%d, %d, %d)\n", cmd_args[0], cmd_args[1], cmd_args[2], cmd_args[3]);
				if (copy_to_user(ptr, &cmd_args, sizeof(cmd_args)))
					return -EFAULT;
			} else {
				ALSPS_PR_ERR("ALSPS_IOCTL_GET_PS_RECALI_RESULT NULL\n");
				return -EINVAL;
			}
			return 0;

	/* ADD by vsen team end */
	default:
		pr_err("unknown IOCTL: 0x%08x\n", cmd);
		return -ENOIOCTLCMD;
	}
	return 0;
}
#ifdef CONFIG_COMPAT
static long alsps_factory_compat_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_ALSPS_SET_PS_MODE:
	case COMPAT_ALSPS_GET_PS_RAW_DATA:
	case COMPAT_ALSPS_SET_ALS_MODE:
	case COMPAT_ALSPS_GET_ALS_RAW_DATA:
	case COMPAT_ALSPS_GET_PS_TEST_RESULT:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
	case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
	case COMPAT_ALSPS_SET_PS_THRESHOLD:
	case COMPAT_ALSPS_IOCTL_SET_CALI:
	case COMPAT_ALSPS_IOCTL_GET_CALI:
	case COMPAT_ALSPS_IOCTL_ALS_GET_CALI:
	case COMPAT_ALSPS_IOCTL_CLR_CALI:
	case COMPAT_ALSPS_ALS_ENABLE_CALI:
	case COMPAT_ALSPS_PS_ENABLE_CALI:
	case COMPAT_ALSPS_IOCTL_ALS_SET_CALI:
		err = file->f_op->unlocked_ioctl(file, cmd,
						 (unsigned long)arg32);
		break;
	default:
		pr_err("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif

static const struct file_operations _alsps_factory_fops = {
	.open = alsps_factory_open,
	.release = alsps_factory_release,
	.unlocked_ioctl = alsps_factory_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = alsps_factory_compat_ioctl,
#endif
};

static struct miscdevice alsps_factory_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &_alsps_factory_fops,
};

int alsps_factory_device_register(struct alsps_factory_public *dev)
{
	int err = 0;

	if (!dev || !dev->fops)
		return -1;
	alsps_factory.gain = dev->gain;
	alsps_factory.sensitivity = dev->sensitivity;
	alsps_factory.fops = dev->fops;
	err = misc_register(&alsps_factory_device);
	if (err) {
		pr_err("alsps_factory_device register failed\n");
		err = -1;
	}
	return err;
}

int alsps_factory_device_deregister(struct alsps_factory_public *dev)
{
	alsps_factory.fops = NULL;
	misc_deregister(&alsps_factory_device);
	return 0;
}
