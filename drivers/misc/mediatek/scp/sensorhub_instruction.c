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

#define DERIVER_NAME "sensorhub_instruction"

#define VIVO_LOG_TAG              "[sensorhub_instruction] "

#define VIVO_ERR(fmt, args...)     pr_err(VIVO_LOG_TAG fmt, ##args)
#define VIVO_INFO(fmt, args...)    pr_info(VIVO_LOG_TAG fmt, ##args)
#define VIVO_DEBUG(fmt, args...)   pr_debug(VIVO_LOG_TAG fmt, ##args)

struct sensorhub_instruction_data {
	uint32_t reset_counts;
};

void __attribute__((weak)) sensorhub_scp_reset(void)
{
	pr_err("%s: not implemented.", __func__);
}

static struct sensorhub_instruction_data *local_sensorhub_data;

static struct class *sensorhub_instruction_class;

static struct attribute *sensorhub_instruction_attrs[] = {
	NULL,
};
ATTRIBUTE_GROUPS(sensorhub_instruction);

ssize_t sensorhub_instruction_reset_show(struct class *class, struct class_attribute *attr, char *buf)
{
	struct sensorhub_instruction_data *sensorhub_data = local_sensorhub_data;
	uint32_t reset_counts = 0;

	if (sensorhub_data != NULL)
		reset_counts = sensorhub_data->reset_counts;

	return snprintf(buf, 20, "%d\n", reset_counts);
}

ssize_t sensorhub_instruction_reset_store(struct class *class, struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct sensorhub_instruction_data *sensorhub_data = local_sensorhub_data;

	if (sensorhub_data == NULL)
		return count;

	/*
	*  DO SENSORHUB RESET
	*/
	sensorhub_scp_reset();

	sensorhub_data->reset_counts++;

	VIVO_INFO("reset_counts %d\n", sensorhub_data->reset_counts);

	return count;
}

static CLASS_ATTR_RW(sensorhub_instruction_reset);

static int create_sensorhub_instruction_class(void)
{
	int ret = 0;

	if (!sensorhub_instruction_class) {
		sensorhub_instruction_class = class_create(THIS_MODULE, DERIVER_NAME);
		if (IS_ERR(sensorhub_instruction_class))
			return PTR_ERR(sensorhub_instruction_class);
		sensorhub_instruction_class->dev_groups = sensorhub_instruction_groups;
		ret = class_create_file(sensorhub_instruction_class, &class_attr_sensorhub_instruction_reset);
		if (ret) {
			VIVO_ERR("create sensorhub_instruction_reset failed, ret=%d\n", ret);
			return ret;
		}
	}
	return 0;
}

static void remove_sensorhub_instruction_class(void)
{
	class_remove_file(sensorhub_instruction_class, &class_attr_sensorhub_instruction_reset);
}

static int __init sensorhub_instruction_init(void)
{
	int ret = 0;
	struct sensorhub_instruction_data *sensorhub_data = NULL;

	sensorhub_data = kzalloc(sizeof(struct sensorhub_instruction_data), GFP_KERNEL);
	if (!sensorhub_data) {
		VIVO_ERR("fail to alloc private data\n");
		return -ENOMEM;
	}
	/* Init private data */
	local_sensorhub_data = sensorhub_data;

	ret = create_sensorhub_instruction_class();
	VIVO_INFO("sensorhub class %d\n", ret);

	return ret;
}

static void __exit sensorhub_instruction_cleanup(void)
{
	kfree(local_sensorhub_data);
	local_sensorhub_data = NULL;
	remove_sensorhub_instruction_class();
}

module_init(sensorhub_instruction_init);
module_exit(sensorhub_instruction_cleanup);

MODULE_AUTHOR("yangruibin@vivo.com");
MODULE_LICENSE("GPL");
