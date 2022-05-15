#ifndef	_VIVO_KEY_H_
#define	_VIVO_KEY_H_

#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define	UP_SLIDE_DELAY_TIME	70
#define	HOME_KEY_SUPPORTED
#define	INPUT_DEV_NAME	"vivo_virtual_key"

#define	BUTTON_NUM	3

#define	HOME_SHIFT	0
#define	BACK_SHIFT	1
#define	MENU_SHIFT	2

/*************************************************************
在使用vivo_key结构体之前，我们必须填充结构体以下成员：
button_attr：节点数组的首地址。
pdev：input设备的父设备，若不初始化，将可能导致死机问题
vivo_key_standard_suspend：suspend回调函数。
vivo_key_standard_resume：resume回调函数。

for example:
	data->vivo_key_processor.button_attr = cyttsp_attrs;
	data->vivo_key_processor.pdev = &client->dev;
	data->vivo_key_processor.vivo_key_standard_suspend = cyttsp_standard_suspend;
	data->vivo_key_processor.vivo_key_standard_resume = cyttsp_standard_resume;
	error = register_vivo_key(&data->vivo_key_processor);
	if(error)
	{
		VIVO_KEY_LOG_ERR("Failed to register_vivo_key\n");
		goto err_vivo_key;
	}

***********************************************************/


struct touchscreen_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};

struct vivo_key {
	struct work_struct menu_release_work;
#if defined(HOME_KEY_SUPPORTED)
	struct work_struct home_release_work;
#endif
	struct work_struct back_release_work;
	struct timer_list menu_release_timer;
#if defined(HOME_KEY_SUPPORTED)
	struct timer_list home_release_timer;
#endif
	struct timer_list back_release_timer;

	struct kobject key_kobject_debug;
	struct attribute **button_attr;
	struct device *pdev;
	struct input_dev *input_dev;
	unsigned long pre_state;

	struct dentry *i2c_test_entry;		//liukangfei add for AT i2c test
	struct dentry *touchscreen_entry;	//liukangfei add for AT i2c test
	int (*at_i2c_test)(struct vivo_key *);//liukangfei add for AT i2c test

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	void (*vivo_key_standard_suspend)(struct vivo_key *);
	void (*vivo_key_standard_resume)(struct vivo_key *);
	struct work_struct suspend_work;
	struct work_struct resume_work;
	struct workqueue_struct *suspend_workqueue;

	struct mutex suspend_mutex;
};

extern int register_vivo_key(struct vivo_key *pvivo_key);
extern void unregister_vivo_key(struct vivo_key *pvivo_key);
extern void vivo_key_processor(struct vivo_key *pvivo_key, int key_shift, int status);

#endif
