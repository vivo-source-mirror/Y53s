/*
 * Copyright (C) 2019 VIVO SENSOR TEAM
 *
 */

#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "../mtk_nanohub/mtk_nanohub.h"
#include "vivo_sensorhub.h"

#define CONFIG_VIVO_SENSOR_HUB_SYS_FS_SUPPORT

#define SENSOR_CMD_DUMP

#define VSEN_LOG_TAG              "[vsen_sensorhub] "

#define VIVO_ERR(fmt, args...)     pr_err(VSEN_LOG_TAG fmt, ##args)
#define VIVO_INFO(fmt, args...)    pr_info(VSEN_LOG_TAG fmt, ##args)
#define VIVO_DEBUG(fmt, args...)   pr_debug(VSEN_LOG_TAG fmt, ##args)

static DEFINE_MUTEX(ioctrl_mutex);

struct vivo_sensorhub_data {
	bool log_print;
};

static struct vivo_sensorhub_data *local_sensorhub_data;
extern int vivo_nanohub_selftest(int sensor_type);

static void vivo_sensorhub_dump_request(struct SCP_SENSOR_HUB_VIVO_CMD_REQ *req)
{
#ifdef SENSOR_CMD_DUMP
	VIVO_INFO("dump_cmd sensor_type %d cmd 0x%x\n",
						req->sensorType, req->cmd);
	VIVO_INFO("dump_cmd data: %d %d %d %d %d %d %d %d %d\n",
				req->data[0], req->data[1], req->data[2],
				req->data[3], req->data[4], req->data[5],
				req->data[6], req->data[7], req->data[8]);
#endif
	return;
}

static void vivo_sensorhub_init_request(struct SCP_SENSOR_HUB_VIVO_CMD_REQ *req)
{
	memset(req, 0x00, sizeof(*req));
}

extern int mtk_nanohub_req_send(union SCP_SENSOR_HUB_DATA *data);
static int vivo_sensorhub_send_request(struct SCP_SENSOR_HUB_VIVO_CMD_REQ *req)
{
	int err = 0;
	err = mtk_nanohub_req_send((union SCP_SENSOR_HUB_DATA *)req);
	VIVO_INFO("mtk_nanohub_req_send %d\n", err);
	vivo_sensorhub_dump_request(req);

	return err;
}

static int vivo_sensorhub_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static ssize_t vivo_sensorhub_write(struct file *file, const char *buffer,
				size_t length, loff_t *offset)
{
	return 0;
}

static int vivo_sensorhub_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long vivo_sensorhub_unlocked_ioctl(struct file *file, unsigned int cmd,
					 unsigned long arg)
{
	struct vivo_sensorhub_data *sensorhub_data = local_sensorhub_data;
	void __user *ptr = (void __user *)arg;
	int user_param[VIVO_USER_SPACE_CMD_DATA_SIZE] = {0};
	int ret = 0;
	struct SCP_SENSOR_HUB_VIVO_CMD_REQ req;

	if (sensorhub_data == NULL)
		return -EFAULT;

	mutex_lock(&ioctrl_mutex);
	switch (cmd) {
	case VIVO_SENSOR_HUB_SENSOR_CMD:
		if (copy_from_user(&user_param, ptr, sizeof(user_param))) {
			ret = -EFAULT;
			goto exit_hub_ioctrl;
		}
		VIVO_INFO("VIVO_SENSOR_HUB_SENSOR_CMD begin %d 0x%x\n", user_param[0], user_param[1]);

		vivo_sensorhub_init_request(&req);
		req.sensorType = (uint8_t)(user_param[0]&0xFF);
		req.action = SENSOR_HUB_VSEN_CMD;
		req.cmd =  (uint32_t)(user_param[1]&0xFFFF);

		if ((req.cmd == SENSOR_COMMAND_ACC_SELF_TEST) ||
			(req.cmd == SENSOR_COMMAND_GYRO_SELF_TEST) ||
			(req.cmd == SENSOR_COMMAND_MAG_SELF_TEST)) {
			user_param[2] = vivo_nanohub_selftest(req.sensorType);
			VIVO_ERR("do selftest sensor:%d result:%d\n", req.sensorType, user_param[2]);
		} else {
			memcpy(&(req.data), &(user_param[2]), sizeof(req.data));
			ret = vivo_sensorhub_send_request(&req);
			if (ret < 0) {
				VIVO_ERR("VIVO_SENSOR_HUB_SENSOR_CMD ipc fail (%d) %d 0x%x %d %d %d\n", ret,
						user_param[0], user_param[1], user_param[2],
						user_param[3], user_param[4]);
				goto exit_hub_ioctrl;
			}
			memcpy(&(user_param[2]), &(req.data), sizeof(req.data));
		}

		if (copy_to_user(ptr, &user_param, sizeof(user_param))) {
			ret = -EFAULT;
			goto exit_hub_ioctrl;
		}
		VIVO_INFO("VIVO_SENSOR_HUB_SENSOR_CMD end %d 0x%x %d %d %d\n", user_param[0], user_param[1],
									user_param[2], user_param[3], user_param[4]);
		break;

	default:
		VIVO_ERR("unknown IOCTL: 0x%08x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

exit_hub_ioctrl:
	mutex_unlock(&ioctrl_mutex);
	return ret;
}

#ifdef CONFIG_COMPAT
static long vivo_sensorhub_compat_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}
#endif

static const struct file_operations _vivo_sensorhub_fops = {
	.open = vivo_sensorhub_open,
	.write = vivo_sensorhub_write,
	.release = vivo_sensorhub_release,
	.unlocked_ioctl = vivo_sensorhub_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vivo_sensorhub_compat_ioctl,
#endif
};

static struct miscdevice vivo_sensorhub_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vivo_sensorhub",
	.fops = &_vivo_sensorhub_fops,
};

#ifdef CONFIG_VIVO_SENSOR_HUB_SYS_FS_SUPPORT
static struct class *vivo_sensorhub_class;

static struct attribute *vivo_sensorhub_attrs[] = {
	NULL,
};
ATTRIBUTE_GROUPS(vivo_sensorhub);

ssize_t vivo_sensorhub_info_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct vivo_sensorhub_data *sensorhub_data = local_sensorhub_data;
	struct SCP_SENSOR_HUB_VIVO_CMD_REQ req;

	if (sensorhub_data == NULL)
		return -1;

#define SENS_TYPE_PROX            13
#define SENSOR_COMMAND_PROXIMITY_RAWDATA 0x802
	vivo_sensorhub_init_request(&req);
	req.sensorType = SENS_TYPE_PROX;
	req.action = SENSOR_HUB_VSEN_CMD;
	req.cmd = SENSOR_COMMAND_PROXIMITY_RAWDATA;
	vivo_sensorhub_send_request(&req);

	return snprintf(buf, 20, "%d\n", sensorhub_data->log_print);
}
static CLASS_ATTR_RO(vivo_sensorhub_info);


ssize_t vivo_sensorhub_log_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct vivo_sensorhub_data *sensorhub_data = local_sensorhub_data;
	int log_debug = 0;

	if ((sensorhub_data != NULL) && (sensorhub_data->log_print))
		log_debug = 1;

	return snprintf(buf, 20, "%d\n", log_debug);
}

ssize_t vivo_sensorhub_log_store(struct class *class, struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct vivo_sensorhub_data *sensorhub_data = local_sensorhub_data;
	int log_debug = 0, err;

	if (sensorhub_data == NULL)
		return count;

	err = kstrtoint(buf, 10, &log_debug);
	if (err)
		VIVO_ERR("fail to get log level (%d)\n", err);

	if (log_debug != 0)
		sensorhub_data->log_print = true;
	else
		sensorhub_data->log_print = false;

	VIVO_INFO("debug level %d\n", sensorhub_data->log_print);

	return count;
}

static CLASS_ATTR_RW(vivo_sensorhub_log);

static int create_vivo_sensorhub_class(void)
{
	int ret = 0;

	if (!vivo_sensorhub_class) {
		vivo_sensorhub_class = class_create(THIS_MODULE, "vivo_sensorhub");
		if (IS_ERR(vivo_sensorhub_class))
			return PTR_ERR(vivo_sensorhub_class);
		vivo_sensorhub_class->dev_groups = vivo_sensorhub_groups;
		ret = class_create_file(vivo_sensorhub_class, &class_attr_vivo_sensorhub_info);
		if (ret) {
			VIVO_ERR("create vivo sensorhub info failed, ret=%d\n", ret);
			return ret;
		}
		ret = class_create_file(vivo_sensorhub_class, &class_attr_vivo_sensorhub_log);
		if (ret) {
			VIVO_ERR("create vivo sensorhub log failed, ret=%d\n", ret);
			return ret;
		}
	}
	return 0;
}

static void remove_vivo_sensorhub_class(void)
{
	class_remove_file(vivo_sensorhub_class, &class_attr_vivo_sensorhub_info);
}
#endif

static int __init vivo_sensorhub_init(void)
{
	int ret = 0;
	struct vivo_sensorhub_data *sensorhub_data = NULL;

	sensorhub_data = kzalloc(sizeof(struct vivo_sensorhub_data), GFP_KERNEL);
	if (!sensorhub_data) {
		VIVO_ERR("fail to alloc private data\n");
		return -ENOMEM;
	}
	/* Init private data */
	local_sensorhub_data = sensorhub_data;

	ret = misc_register(&vivo_sensorhub_device);
	if (ret) {
		kfree(sensorhub_data);
		VIVO_ERR("register misc failed\n");
		ret = -1;
	}
	VIVO_INFO("misc %d\n", ret);

#ifdef CONFIG_VIVO_SENSOR_HUB_SYS_FS_SUPPORT
	ret = create_vivo_sensorhub_class();
	VIVO_INFO("sensorhub class %d\n", ret);
#endif

	return ret;
}

static void __exit vivo_sensorhub_cleanup(void)
{
	kfree(local_sensorhub_data);
	local_sensorhub_data = NULL;
	misc_deregister(&vivo_sensorhub_device);
#ifdef CONFIG_VIVO_SENSOR_HUB_SYS_FS_SUPPORT
	remove_vivo_sensorhub_class();
#endif
}

module_init(vivo_sensorhub_init);
module_exit(vivo_sensorhub_cleanup);

MODULE_AUTHOR("Yang Ruibin@VIVO SENSOR TEAM");
MODULE_LICENSE("GPL");
