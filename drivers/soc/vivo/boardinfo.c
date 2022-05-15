/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/init.h>

#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 8
#define VIVO_VENDOR_PROJECT_NAME 16

struct board_info_ext {
	char project_name[VIVO_VENDOR_PROJECT_NAME];
	char cpu_type[VIVO_CPU_TYPE_LEN];
	char vendor[VIVO_VENDOR_LEN];
	unsigned int core_num;
	char user_cpu_freq[VIVO_VENDOR_LEN];
} *boardinfo_ext;

/*
 * This is our vivo board info obj that we will create a few of and register them with
 * sysfs.  by lipengkui
 */

struct boardinfo_obj {
	struct kobject kobj;
	int status;
};

#define to_boardinfo_obj(x) container_of(x, struct boardinfo_obj, kobj)

struct boardinfo_attribute {
	struct attribute attr;
	ssize_t (*show)(struct boardinfo_obj *board_info, struct boardinfo_attribute *attr, char *buf);
	ssize_t (*store)(struct boardinfo_obj *board_info, struct boardinfo_attribute *attr, const char *buf, size_t count);
};

#define to_boardinfo_attr(x) container_of(x, struct boardinfo_attribute, attr)

static ssize_t vivo_show_vendor(struct boardinfo_obj *dev, struct boardinfo_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->vendor);
}

static ssize_t vivo_show_cpu_type(struct boardinfo_obj *dev, struct boardinfo_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->cpu_type);
}

static ssize_t vivo_show_core_num(struct boardinfo_obj *dev, struct boardinfo_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_ext->core_num);
}

static ssize_t vivo_show_user_cpu_freq(struct boardinfo_obj *dev, struct boardinfo_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->user_cpu_freq);
}

static ssize_t vivo_show_project_name(struct boardinfo_obj *dev, struct boardinfo_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->project_name);
}

static int vivo_boardinfo_ext_init(void)
{
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return 1;
	}

	boardinfo_ext->core_num = num_possible_cpus();

	strlcpy(boardinfo_ext->cpu_type, "6889", VIVO_CPU_TYPE_LEN);
	strlcpy(boardinfo_ext->vendor, "vivo", VIVO_VENDOR_LEN);
	strlcpy(boardinfo_ext->user_cpu_freq, "2.2", VIVO_VENDOR_LEN);
	strlcpy(boardinfo_ext->project_name, "PD1986", VIVO_VENDOR_PROJECT_NAME);

	return 0;
}

static int board_info_parse_dt(struct platform_device *pdevboard)
{
	int ret = 0;
	const char *temp_boardbuf;

	struct device_node *board_node = pdevboard->dev.of_node;

	if (!board_node) {
		pr_err("board_info device tree missing\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(board_node, "vivo,vendor_project_cpunum", &(boardinfo_ext->core_num));
	if (ret) {
		pr_err("vivo,vendor_project_cpunum property not defined\n");
	}

	ret = of_property_read_string(board_node, "vivo,vendor_project_freq", &temp_boardbuf);
	if (ret)
		pr_err("vivo,vendor_project_freq property not defined\n");
	else
		strlcpy(boardinfo_ext->user_cpu_freq, temp_boardbuf, VIVO_VENDOR_LEN);

	ret = of_property_read_string(board_node, "vivo,vendor_project_cpu", &temp_boardbuf);
	if (ret)
		pr_err("vivo,vendor_project_cpu property not defined\n");
	else
		strlcpy(boardinfo_ext->cpu_type, temp_boardbuf, VIVO_VENDOR_LEN);

	ret = of_property_read_string(board_node, "vivo,vendor_project_name", &temp_boardbuf);
	if (ret)
		pr_err("vivo,vendor_project_name property not defined\n");
	else
		strlcpy(boardinfo_ext->project_name, temp_boardbuf, VIVO_VENDOR_PROJECT_NAME);
	return 0;
}


/*
 * The default show function that must be passed to sysfs.  This will be
 * called by sysfs for whenever a show function is called by the user on a
 * sysfs file associated with the kobjects we have registered.  We need to
 * transpose back from a "default" kobject to our custom struct foo_obj and
 * then call the show function for that specific object.
 */
static ssize_t vivo_boardinfo_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct boardinfo_attribute *attribute;
	struct boardinfo_obj *vivo_boardinfo;

	attribute = to_boardinfo_attr(attr);
	vivo_boardinfo = to_boardinfo_obj(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(vivo_boardinfo, attribute, buf);
}

/*
 * The release function for our object.  This is REQUIRED by the kernel to
 * have.  We free the memory held in our object here.
 *
 * NEVER try to get away with just a "blank" release function to try to be
 * smarter than the kernel.  Turns out, no one ever is...
 */
static void boardinfo_release(struct kobject *kobj)
{
	struct boardinfo_obj *vivo_boardinfo;

	vivo_boardinfo = to_boardinfo_obj(kobj);
	kfree(vivo_boardinfo);
}

/* Sysfs attributes cannot be world-writable. */
static struct boardinfo_attribute vivo_core_num_attribute =
	__ATTR(core_num, S_IRUGO, vivo_show_core_num,  NULL);

static struct boardinfo_attribute vivo_user_cpu_freq_attribute =
	__ATTR(user_cpu_freq, S_IRUGO, vivo_show_user_cpu_freq,  NULL);

static struct boardinfo_attribute vivo_vendor_attribute =
	__ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct boardinfo_attribute vivo_cpu_type_attribute =
	__ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type,  NULL);

static struct boardinfo_attribute vivo_project_name_attribute =
	__ATTR(project_name, S_IRUGO, vivo_show_project_name,  NULL);


/* Our custom sysfs_ops that we will associate with our ktype later on */
static const struct sysfs_ops boardinfo_sysfs_ops = {
	.show = vivo_boardinfo_attr_show,
};

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *boardinfo_default_attrs[] = {
	&vivo_core_num_attribute.attr,
	&vivo_user_cpu_freq_attribute.attr,
	&vivo_vendor_attribute.attr,
	&vivo_cpu_type_attribute.attr,
	&vivo_project_name_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * Our own ktype for our kobjects.  Here we specify our sysfs ops, the
 * release function, and the set of default attributes we want created
 * whenever a kobject of this type is registered with the kernel.
 */
static struct kobj_type boardinfo_ktype = {
	.sysfs_ops = &boardinfo_sysfs_ops,
	.release = boardinfo_release,
	.default_attrs = boardinfo_default_attrs,
};

static struct boardinfo_obj *vivo_boardinfo_obj;

static struct boardinfo_obj *vivo_create_boardinfo_obj(void)
{
	struct boardinfo_obj *board_info_obj;
	int retval;

	/* allocate the memory for the whole object */
	board_info_obj = kzalloc(sizeof(*board_info_obj), GFP_KERNEL);
	if (!board_info_obj)
		return NULL;


	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */
	retval = kobject_init_and_add(&board_info_obj->kobj, &boardinfo_ktype, NULL, "board_info");
	if (retval) {
		kobject_put(&board_info_obj->kobj);
		return NULL;
	}

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	kobject_uevent(&board_info_obj->kobj, KOBJ_ADD);

	return board_info_obj;
}

static int vivo_board_info_probe(struct platform_device *pdev)
{
	int ret;

	ret = vivo_boardinfo_ext_init();
	if (ret) {
		pr_err("vivo_board_info Device alloc failed!\n");
		return -ENOMEM;
	}

	ret = board_info_parse_dt(pdev);
	if (ret) {
		pr_err("Couldn't parse DT nodes ret=%d\n", ret);
		return ret;
	}

	/*
	 * Create board info kobjects
	 */
	vivo_boardinfo_obj = vivo_create_boardinfo_obj();
	if (!vivo_boardinfo_obj) {
		pr_err("Couldn't Create sysfs board_info dir!\n");
		return -EINVAL;
	}

	return 0;
}

static int vivo_board_info_remove(struct platform_device *pdev)
{
	kobject_put(&vivo_boardinfo_obj->kobj);
	pr_err("vivo_board_info  remove.\n");
	return 0;
}

static struct of_device_id boardinfo_match_table[] = {
	{ .compatible = "vivo,vendor_board_info",},
	{},
};

static struct platform_driver boardinfo_driver = {
	.probe      = vivo_board_info_probe,
	.remove     = vivo_board_info_remove,
	.driver = {
		.name   = "board_info",
		.owner  = THIS_MODULE,
		.of_match_table = boardinfo_match_table,
	},
};

static int __init vivo_vendor_info_init(void)
{
	return platform_driver_register(&boardinfo_driver);
}
module_init(vivo_vendor_info_init);

static void __exit vivo_vendor_info_exit(void)
{
	platform_driver_unregister(&boardinfo_driver);
}
module_exit(vivo_vendor_info_exit);



