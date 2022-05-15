/*
 * kernel/rsc/rsc_sys_api.c
 *
 * VIVO Resource Control.
 *
 * add /sys/rsc/ interface for rsc internal use.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#include <linux/proc_fs.h>
#include <linux/vivo_rsc/rsc_internal.h>

#define RSC_MODE_NAME_LEN	16

/* sysfs dir for rsc root dir */
struct kobject *rsc_root_dir;
struct proc_dir_entry *vivo_rsc;

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
unsigned int __read_mostly rsc_debug/* = RSC_VSWAPD*/;/* = RSC_FIX_DEADLOCK*/;
#else
unsigned int __read_mostly rsc_debug/* = RSC_VSWAPD*/;/* = RSC_FIX_SPF;*//*= RSC_APP_LAUNCH_BOOST;*//*= RSC_BINDER|RSC_APP_LAUNCH_BOOST*/;
#endif
unsigned int rsc_func_lv_mask;
#if 0
#define SYSTEM_ID KUIDT_INIT(1000)
#define SYSTEM_GROUP_ID KGIDT_INIT(1000)
#endif
int rsc_chown_to_system(struct kobject *kobj, const struct attribute *attr)
{
	#if 1
	struct kernfs_node *kn;
	struct iattr newattrs;
	int rc;

	kn = kernfs_find_and_get(kobj->sd, attr->name);
	if (!kn)
		return -ENOENT;

	newattrs.ia_uid = SYSTEM_ID;
	newattrs.ia_gid = SYSTEM_GROUP_ID;
	newattrs.ia_valid = ATTR_UID | ATTR_GID;

	rc = kernfs_setattr(kn, &newattrs);

	kernfs_put(kn);
	return rc;
	#else
	return 0;
	#endif
}

static ssize_t show_func_debug(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "rsc func lv debug mask = 0x%x\n", rsc_func_lv_mask);

	return ret;
}

static ssize_t store_func_debug(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int func_dbg_lv;

	ret = sscanf(buf, "%u", &func_dbg_lv);
	if (ret != 1)
		goto fail;

	if (rsc_debug)
		rsc_info("%s func lv debug mask = 0x%x\n", __func__, func_dbg_lv);

	rsc_func_lv_mask = func_dbg_lv;

	return count;

fail:
	rsc_err("usage: echo func_dbg_lv > /sys/rsc/func_debug\n");

	return -EINVAL;
}

static ssize_t show_enabled(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	rsc_lock(&rsc_main_info.lock);
	ret = snprintf(buf, PAGE_SIZE, "%s enable: %u\n", __func__, rsc_main_info.is_enabled);
	rsc_unlock(&rsc_main_info.lock);

	return ret;
}

static ssize_t store_enabled(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int enabled;

	ret = sscanf(buf, "%u", &enabled);
	if (ret != 1)
		goto fail;

	if (rsc_debug)
		rsc_info("%s enable = %d\n", __func__, enabled);

	rsc_lock(&rsc_main_info.lock);
	rsc_main_info.is_enabled = enabled;

	rsc_unlock(&rsc_main_info.lock);

	return count;

fail:
	rsc_err("usage: echo 0 or 1 > /sys/rsc/enabled\n");

	return -EINVAL;
}

static ssize_t show_debug(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "debug: %x\n", rsc_debug);

	return ret;
}

static ssize_t store_debug(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int debug;

	ret = sscanf(buf, "%x", &debug);
	if (ret != 1)
		goto fail;

	rsc_debug = debug;
	rsc_info("%s: debug = %x\n", __func__, rsc_debug);

	return count;

fail:
	rsc_err("usage: echo 0 or 1 > /sys/rsc/debug\n");

	return -EINVAL;
}

define_rsc_one_global_rw(func_debug);
define_rsc_one_global_rw(debug);
define_rsc_one_global_rw(enabled);

static struct attribute *rsc_root_attr[] = {
	&func_debug.attr,
	&debug.attr,
	&enabled.attr,
	NULL,
};

static struct attribute_group rsc_root_group = {
	.attrs = rsc_root_attr,
};

int rsc_sys_api_init(void)
{
	int error;

	rsc_root_dir = kobject_create_and_add("rsc", NULL);
	if (!rsc_root_dir)
		return -ENOMEM;
	error = sysfs_create_group(rsc_root_dir, &rsc_root_group);
	if (error)
		return error;

	vivo_rsc = proc_mkdir("vivo_rsc", NULL);
	if (!vivo_rsc) {
		return -ENOMEM;
	}

	return 0;
}

