/*
 *	drivers/sarpower/sarpower_class.c
 *
 * Copyright (C) 2020 vivo Technologies, Inc.
 * Author:Kangkai Deng <dengkangkai@vivo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sarpower.h>

struct class *sarpower_class;
static atomic_t device_count;

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
						  char *buf)
{
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	if (sdev->print_state) {
		int ret = sdev->print_state(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", sdev->state);
}

/*laizhilong add start*/
static ssize_t state_store(struct device *dev, struct device_attribute *attr,
						   const char *buf, size_t length)
{
	unsigned long val;
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	dev_err(NULL, "%s 1\n", __func__);
	if (!attr || !dev || !buf)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	sarpower_set_state(sdev, (int)val);
	dev_err(NULL, "%s 2\n", __func__);
	return length;
}
/*laizhilong add end*/

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
						 char *buf)
{
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	if (sdev->print_name) {
		int ret = sdev->print_name(sdev, buf);
		if (ret >= 0)
			return ret;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", sdev->name);
}

static DEVICE_ATTR(state, S_IRUGO, state_show, state_store);
static DEVICE_ATTR(name, S_IRUGO, name_show, NULL);

void sarpower_set_state(struct sarpower_dev *sdev, int state)
{
	char name_buf[120];
	char state_buf[120];
	char *prop_buf;
	char *envp[3];
	int env_offset = 0;
	int length;

	if (sdev->state != state) {
		sdev->state = state;

		prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
		if (prop_buf) {
			length = name_show(sdev->dev, NULL, prop_buf);
			if (length > 0) {
				if (prop_buf[length - 1] == '\n')
					prop_buf[length - 1] = 0;
				snprintf(name_buf, sizeof(name_buf),
						 "sarpower_NAME=%s", prop_buf);
				envp[env_offset++] = name_buf;
			}
			length = state_show(sdev->dev, NULL, prop_buf);
			if (length > 0) {
				if (prop_buf[length - 1] == '\n')
					prop_buf[length - 1] = 0;
				snprintf(state_buf, sizeof(state_buf),
						 "SWITCH_STATE=%s", prop_buf);
				envp[env_offset++] = state_buf;
			}
			envp[env_offset] = NULL;
			kobject_uevent_env(&sdev->dev->kobj, KOBJ_CHANGE, envp);
			free_page((unsigned long)prop_buf);
		} else {
			printk(KERN_ERR "out of memory in sarpower_set_state\n");
			kobject_uevent(&sdev->dev->kobj, KOBJ_CHANGE);
		}
	}
}
EXPORT_SYMBOL_GPL(sarpower_set_state);

static int create_sarpower_class(void)
{
	if (!sarpower_class) {
		sarpower_class = class_create(THIS_MODULE, "sarpower");
		if (IS_ERR(sarpower_class))
			return PTR_ERR(sarpower_class);
		atomic_set(&device_count, 0);
	}

	return 0;
}

int sarpower_dev_register(struct sarpower_dev *sdev)
{
	int ret;
	if (!sarpower_class) {
		ret = create_sarpower_class();
		if (ret < 0)
			return ret;
	}

	sdev->index = atomic_inc_return(&device_count);
	sdev->dev = device_create(sarpower_class, NULL,
							  MKDEV(0, sdev->index), NULL, sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);

	ret = device_create_file(sdev->dev, &dev_attr_state);
	if (ret < 0)
		goto err_create_file_1;
	ret = device_create_file(sdev->dev, &dev_attr_name);
	if (ret < 0)
		goto err_create_file_2;

	dev_set_drvdata(sdev->dev, sdev);
	sdev->state = 0;
	return 0;

err_create_file_2:
	device_remove_file(sdev->dev, &dev_attr_state);
err_create_file_1:
	device_destroy(sarpower_class, MKDEV(0, sdev->index));
	printk(KERN_ERR "sarpower: Failed to register driver %s\n", sdev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(sarpower_dev_register);

void sarpower_dev_unregister(struct sarpower_dev *sdev)
{
	device_remove_file(sdev->dev, &dev_attr_name);
	device_remove_file(sdev->dev, &dev_attr_state);
	device_destroy(sarpower_class, MKDEV(0, sdev->index));
}
EXPORT_SYMBOL_GPL(sarpower_dev_unregister);

static int __init sarpower_class_init(void)
{
	return create_sarpower_class();
}

static void __exit sarpower_class_exit(void)
{
	class_destroy(sarpower_class);
}

module_init(sarpower_class_init);
module_exit(sarpower_class_exit);

MODULE_AUTHOR("Kangkai Deng <dengkangkai@vivo.com>");
MODULE_DESCRIPTION("sarpower class driver");
MODULE_LICENSE("GPL");
