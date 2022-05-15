/***************************************************************************************
  wangyuanliang created @ 2013/4/17
***************************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/bbk_drivers_info.h>
#include <linux/gpio.h>

#ifdef CONFIG_VIVO_REGDUMP
#include <linux/vivo-regdump.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#endif

struct bbk_driver_callback {
	struct list_head list;
	struct bbk_drivers_callback_handler *handler;
} drivers_callbacks;
#ifdef CONFIG_VIVO_REGDUMP
static bool do_regdump;
static unsigned long last_press;
static struct delayed_work vivo_regdump_dwork;
#endif

int bbk_drivers_log_switch_register_callback(struct bbk_drivers_callback_handler *handler) 
{
	struct bbk_driver_callback *new_callback = NULL;
	
	new_callback = kzalloc(sizeof(struct bbk_driver_callback), GFP_KERNEL);
	if (!new_callback) {
		printk(KERN_ERR "%s: Failed at allocate callback struct\n", __func__);
		return -ENOMEM;
	}

	new_callback->handler = handler;
	INIT_LIST_HEAD(&new_callback->list);
	list_add_tail(&new_callback->list, &drivers_callbacks.list);

	return 0;
}
EXPORT_SYMBOL_GPL(bbk_drivers_log_switch_register_callback);

void bbk_drivers_log_switch_unregister_callback(char *callback_name)
{
	struct bbk_driver_callback *entry;

	if (!list_empty(&drivers_callbacks.list)) {
		list_for_each_entry(entry, &drivers_callbacks.list, list)
			if (!strcmp(entry->handler->name, callback_name)) {
				list_del(&entry->list);
				kfree(entry);
				return;
			}
	}
}
EXPORT_SYMBOL_GPL(bbk_drivers_log_switch_unregister_callback);

void bbk_drivers_log_switch_do_callback(bool is_siwtch_on)
{
	struct bbk_driver_callback *entry;

	printk(KERN_ERR "%s: called(%d)\n", __func__, is_siwtch_on);
	if (!list_empty(&drivers_callbacks.list)) {
		list_for_each_entry(entry, &drivers_callbacks.list, list)
			entry->handler->callback(is_siwtch_on);
	}
}

static struct volume_key_trace_info {
	int key_traced_number;
	unsigned long first_down_time;
} volume_key_trace;

static bool has_switched_on;

static void bbk_drivers_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	unsigned long now = jiffies;
	unsigned long max_time_length = 3 * HZ; 

#ifdef CONFIG_VIVO_REGDUMP
	if (code == KEY_VOLUMEUP) {
		if (value) {
			do_regdump = true;
			last_press = now;
			schedule_delayed_work(&vivo_regdump_dwork,
								msecs_to_jiffies(3200));
		} else {
			do_regdump = false;
			/*cancel_delayed_work_sync(&vivo_regdump_dwork);*/
		}
	}
#endif

	if (value == 0) {
		return;
	}

	if (now - volume_key_trace.first_down_time < max_time_length) {
		switch (volume_key_trace.key_traced_number) {
		case 0:
			/* should not happen */
			if (code == KEY_VOLUMEUP) {
				volume_key_trace.key_traced_number++;
			} else {
				/* nothing to do */
			}
			break;
		case 1:
			if (code == KEY_VOLUMEUP) {
				if (has_switched_on) {
					bbk_drivers_log_switch_do_callback(false);
					has_switched_on = false;
				}
				volume_key_trace.key_traced_number = 0;
				volume_key_trace.first_down_time = 0;
			} else if (code == KEY_VOLUMEDOWN) {
				volume_key_trace.key_traced_number++;
			} else {
				/* should not happen */
			}
			break;
		case 2:
			if (code == KEY_VOLUMEUP) {
				volume_key_trace.key_traced_number++;
			} else if (code == KEY_VOLUMEDOWN) {
				volume_key_trace.key_traced_number = 0;
				volume_key_trace.first_down_time = 0;
			}
			break;
		case 3:
			if (code == KEY_VOLUMEUP) {
				volume_key_trace.key_traced_number = 1;
				volume_key_trace.first_down_time = now;
			} else {
				if (!has_switched_on) {
					bbk_drivers_log_switch_do_callback(true);
					/* for depend mistouch do not clear trace info
					   so it can't turn off log immediatly in this 3s
					*/
					volume_key_trace.key_traced_number++;
					has_switched_on = true;
				}
			}
			break;
		default:
			/* nothing to do */
			break;
		}
	} else {
		if (code == KEY_VOLUMEUP) {
			volume_key_trace.key_traced_number = 1;
			volume_key_trace.first_down_time = now;
		} else {
			/* nothing to do */
		}
	}
}

static int bbk_drivers_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{	
	struct input_handle *handle;
	int error;

	printk(KERN_ERR "%s: input device %s connecting\n", __func__, dev->name);

	/*if (strcmp(dev->name, "gpio-keys")) {*/
	if (strcmp(dev->name, "mtk-kpd")) {	
		printk(KERN_ERR "%s: %s: Not volume key input device\n", 
					__func__, dev->name);
		return -1;
	}

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "bbk_drivers_log";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void bbk_drivers_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id bbk_drivers_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler bbk_drivers_input_handler = {
	.event		= bbk_drivers_input_event,
	.connect	= bbk_drivers_input_connect,
	.disconnect	= bbk_drivers_input_disconnect,
	.name		= "bbk_drivers_log_siwtch",
	.id_table	= bbk_drivers_ids,
};

static struct bbk_devices_info_list {
	struct list_head list;
	struct bbk_device_info dev_info;
} devices_list;

int bbk_driver_register_device_info(char *device_type, char *device_name) 
{
	struct bbk_devices_info_list *new_info;
	int type_length, name_length;

	type_length = strlen(device_type);
	name_length = strlen(device_name);
	if (type_length > NAME_LENGTH || name_length > NAME_LENGTH) {
		printk(KERN_ERR "%s: Invalide device_type or device_name\n", __func__);
		return -EINVAL;
	}

	new_info = kzalloc(sizeof(struct bbk_devices_info_list), GFP_KERNEL);
	if (!new_info) {
		printk(KERN_ERR "%s: Can't allocat new device info list\n", __func__);
		return -ENOMEM;
	}

	strcpy(new_info->dev_info.device_type, device_type);
	strcpy(new_info->dev_info.device_name, device_name);
	INIT_LIST_HEAD(&new_info->list);
	list_add_tail(&new_info->list, &devices_list.list);

	return 0;
}
EXPORT_SYMBOL_GPL(bbk_driver_register_device_info);

void bbk_driver_unregister_device_info(char *device_type, char *device_name)
{
	struct bbk_devices_info_list *entry;

	if (!list_empty(&devices_list.list)) {
		list_for_each_entry(entry, &devices_list.list, list)
			if (!strcmp(entry->dev_info.device_type, device_type) 
					&& !strcmp(entry->dev_info.device_name, device_name)) {
				list_del(&entry->list);
				kfree(entry);
			}
	}
}
EXPORT_SYMBOL_GPL(bbk_driver_unregister_device_info);

static ssize_t devices_list_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	struct bbk_devices_info_list *entry;
	int count = 0;

	if (!list_empty(&devices_list.list)) {
		/*count += sprintf(&buf[count], "%-16.s%-16.s\n", "Device type", "Device name");*/
		list_for_each_entry(entry, &devices_list.list, list) {
			count += sprintf(&buf[count], "%s = ", entry->dev_info.device_name);
			count += sprintf(&buf[count], "%s\n", entry->dev_info.device_type);
		}
	} else {
		count += sprintf(buf, "Device list is empty\n");
	}

	return count;
}

static struct debug_sysfs_entry bbk_devices_list =
	__ATTR(devices_list, S_IRUGO, 
			devices_list_show, NULL);

static struct attribute *sys_attrs[] = {
	&bbk_devices_list.attr,
	NULL
};

static ssize_t debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops debug_object_sysfs_ops = {
	.show = debug_object_show,
	.store = debug_object_store,
};
static struct kobj_type debug_object_type = {
	.sysfs_ops	= &debug_object_sysfs_ops,
	.release	= debug_object_release,
	.default_attrs = sys_attrs,
};

static struct kobject kobject_debug;

static int creat_sys_files(void) 
{ 
    int ret; 
	
	ret = kobject_init_and_add(&kobject_debug, &debug_object_type,
					NULL, "devs_list");
    if (ret) { 
		printk("%s: Create kobjetct error!\n", __func__); 
		return -1; 
    } 
    return 0; 
} 

int devs_create_sys_files(const struct attribute *attr)
{
	int ret = 0;
	ret = sysfs_create_file(&kobject_debug, attr);
	if (ret) {
		printk("%s: Create %s sys files error!", __func__, attr->name);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(devs_create_sys_files);


#ifdef CONFIG_VIVO_REGDUMP
static void vivo_audio_regdump(struct work_struct *work)
{
	if (do_regdump && (jiffies - last_press >= 3*HZ))
		vivo_audio_regdump_do_callback();
	return;
}
#endif

static int __init bbk_drivers_log_switch_init(void)
{
	int error;

	has_switched_on = false;
#ifdef CONFIG_VIVO_REGDUMP
	do_regdump = false;
	last_press = 0;
	INIT_DELAYED_WORK(&vivo_regdump_dwork, vivo_audio_regdump);
#endif

	INIT_LIST_HEAD(&drivers_callbacks.list);
	volume_key_trace.key_traced_number = 0;
	volume_key_trace.first_down_time = jiffies;
	error = input_register_handler(&bbk_drivers_input_handler);
	if (error) {
		printk(KERN_ERR "%s: register input handler failed\n", __func__);
		return error;
	}
	
	INIT_LIST_HEAD(&devices_list.list);
	error = creat_sys_files();
	if (error) {
		printk(KERN_ERR "%s: creat sysfs files failed\n", __func__);
		return error;
	}
	return 0;
}

static void __exit bbk_drivers_log_switch_exit(void)
{
	/*nothing to do */
}

early_initcall(bbk_drivers_log_switch_init);
module_exit(bbk_drivers_log_switch_exit);
