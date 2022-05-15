#include <linux/input/vivo_key.h>
#include <linux/kernel.h>
#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_virtual_key.h>

static struct vivo_key *global_key;

/*************************************************************
*HOME KEY CODE:172
*BACK KEY CODE:158
*MENU KEY CODE:139
*************************************************************/
static int button_map[3] = {172, 158, 139};

/*********************************************************
*result:
*0:means we should report up
*others:means we should report -1 to give up down action
*********************************************************/
int should_key_be_canceled(void)
{
	int finger_when_key_down = 0;

	set_virtual_key_state(0);
	finger_when_key_down = get_finger_when_key_down();
	return (finger_when_key_down == 0)?finger_when_key_down:1;
}

/*********************************************************
*result:
*0:means we should report key down
*others:means this is a swip down action , so we shouldn't report key down
*********************************************************/
int should_report_key_down(void)
{
	long long now = 0;
	long long time_diff = 0;
	long long AA_area_point_release_time;
	bool is_finger_on_2d;
	int result = 0;

	now = jiffies;
	is_finger_on_2d = get_finger_on_2d();
	AA_area_point_release_time = get_AA_release_time();
	time_diff = now - AA_area_point_release_time;
	VIVO_KEY_LOG_DBG("time_diff=%lld now=%lld aa_release_time=%lld\n", time_diff, now, AA_area_point_release_time);
	if (time_diff < (HZ/5) || (is_finger_on_2d)) {
		VIVO_KEY_LOG_INF("Forbidden to report key\n");
		result = 1;
	}
	return result;
}

static void menu_release_work_handler(struct work_struct *pwork)
{
	struct vivo_key *pvivo_key = NULL;
	int ret = 0;
	VIVO_KEY_LOG_INF("enter into menu_release_work_handler\n");
	if (!pwork) {
		VIVO_KEY_LOG_ERR("The pwork pointer is NULL\n");
		return;
	}
	pvivo_key = container_of(pwork, struct vivo_key, menu_release_work);

	ret = should_key_be_canceled();
	clear_bit(MENU_SHIFT, &pvivo_key->pre_state);
	mutex_lock(&pvivo_key->input_dev->mutex);
	if (ret) {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[2], -1);
		VIVO_KEY_LOG_INF("give up former menu down\n");
	} else {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[2], 0);
		VIVO_KEY_LOG_INF("key menu up!\n");
	}
	input_sync(pvivo_key->input_dev);
	mutex_unlock(&pvivo_key->input_dev->mutex);
}

static void menu_release_timer_handler(unsigned long data)
{
	struct vivo_key *pvivo_key = (struct vivo_key *)data;
	if (!pvivo_key) {
		VIVO_KEY_LOG_ERR("The up_slide pointer is NULL\n");
		return;
	}
	schedule_work(&pvivo_key->menu_release_work);
}

void menu_up_processor(struct vivo_key *pvivo_key)
{
	mod_timer(&pvivo_key->menu_release_timer,
						jiffies + msecs_to_jiffies(UP_SLIDE_DELAY_TIME));
	return;
}

static void back_release_work_handler(struct work_struct *pwork)
{
	struct vivo_key *pvivo_key = NULL;
	int ret = 0;
	VIVO_KEY_LOG_INF("enter into back_release_work_handler\n");
	if (!pwork) {
		VIVO_KEY_LOG_ERR("The pwork pointer is NULL\n");
		return;
	}
	pvivo_key = container_of(pwork, struct vivo_key, back_release_work);

	ret = should_key_be_canceled();
	clear_bit(BACK_SHIFT, &pvivo_key->pre_state);
	mutex_lock(&pvivo_key->input_dev->mutex);
	if (ret) {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[1], -1);
		VIVO_KEY_LOG_INF("give up former back down\n");
	} else {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[1], 0);
		VIVO_KEY_LOG_INF("key back up!\n");
	}
	input_sync(pvivo_key->input_dev);
	mutex_unlock(&pvivo_key->input_dev->mutex);
}

static void back_release_timer_handler(unsigned long data)
{
	struct vivo_key *pvivo_key = (struct vivo_key *)data;
	if (!pvivo_key) {
		VIVO_KEY_LOG_ERR("The up_slide pointer is NULL\n");
		return;
	}
	schedule_work(&pvivo_key->back_release_work);
}

void back_up_processor(struct vivo_key *pvivo_key)
{
	mod_timer(&pvivo_key->back_release_timer,
						jiffies + msecs_to_jiffies(UP_SLIDE_DELAY_TIME));
	return;
}

#if defined(HOME_KEY_SUPPORTED)
static void home_release_work_handler(struct work_struct *pwork)
{
	struct vivo_key *pvivo_key = NULL;
	int ret = 0;
	VIVO_KEY_LOG_INF("enter into home_release_work_handler\n");
	if (!pwork) {
		VIVO_KEY_LOG_ERR("The pwork pointer is NULL\n");
		return;
	}
	pvivo_key = container_of(pwork, struct vivo_key, home_release_work);

	ret = should_key_be_canceled();
	clear_bit(HOME_SHIFT, &pvivo_key->pre_state);
	mutex_lock(&pvivo_key->input_dev->mutex);
	if (ret) {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[0], -1);
		VIVO_KEY_LOG_INF("give up former home down\n");
	} else {
		input_event(pvivo_key->input_dev, EV_KEY,
						button_map[0], 0);
		VIVO_KEY_LOG_INF("key HOME up!\n");
	}
	input_sync(pvivo_key->input_dev);
	mutex_unlock(&pvivo_key->input_dev->mutex);
}

static void home_release_timer_handler(unsigned long data)
{
	struct vivo_key *pvivo_key = (struct vivo_key *)data;
	if (!pvivo_key) {
		VIVO_KEY_LOG_ERR("The up_slide pointer is NULL\n");
		return;
	}
	schedule_work(&pvivo_key->home_release_work);
}

void home_up_processor(struct vivo_key *pvivo_key)
{
	mod_timer(&pvivo_key->home_release_timer,
						jiffies + msecs_to_jiffies(UP_SLIDE_DELAY_TIME));
	return;
}
#endif

void vivo_key_processor(struct vivo_key *pvivo_key, int key_shift, int status)
{

	VIVO_TS_LOG_INF("key=%s status %d\n", (key_shift == 0) ? "HOME" : (key_shift == 1) ? "BACK" : "MENU", status);

	if (status) {
		set_bit(key_shift, &pvivo_key->pre_state);
		if (should_report_key_down()) {
			return;
		}
		mutex_lock(&pvivo_key->input_dev->mutex);
		input_event(pvivo_key->input_dev, EV_KEY, button_map[key_shift], 1);
		input_sync(pvivo_key->input_dev);
		mutex_unlock(&pvivo_key->input_dev->mutex);
		set_virtual_key_state(status);
		VIVO_KEY_LOG_INF("key %s down!\n", (key_shift == 0) ? "HOME" : (key_shift == 1) ? "BACK" : "MENU");
	} else {
		switch (key_shift) {
		case 0:
			home_up_processor(pvivo_key);
			break;
		case 1:
			back_up_processor(pvivo_key);
			break;
		case 2:
			menu_up_processor(pvivo_key);
			break;
		default:
			VIVO_KEY_LOG_ERR("unknow key\n");
		}
	}
}

static int input_dev_setup(struct vivo_key *pvivo_key)
{
	int error = 0;
	int i;
	pvivo_key->input_dev = input_allocate_device();
	if (!pvivo_key->input_dev) {
		VIVO_KEY_LOG_ERR("Failed to allocate input device\n");
		return -ENOMEM;
	}
	pvivo_key->input_dev->name = INPUT_DEV_NAME;
	pvivo_key->input_dev->id.bustype = BUS_I2C;
	pvivo_key->input_dev->dev.parent = pvivo_key->pdev;

	for (i = 0; i < BUTTON_NUM; i++) {
		input_set_capability(pvivo_key->input_dev, EV_KEY,
					button_map[i]);
	}

	input_set_drvdata(pvivo_key->input_dev, pvivo_key);
	/*this line maybe cause phone could not start*/
	/*dev_set_drvdata(pvivo_key->pdev, pvivo_key);*/

	__set_bit(EV_SYN, pvivo_key->input_dev->evbit);
	__set_bit(EV_KEY, pvivo_key->input_dev->evbit);

	mutex_init(&pvivo_key->input_dev->mutex);

	error = input_register_device(pvivo_key->input_dev);
	if (error) {
		VIVO_KEY_LOG_ERR("Unable to register input device, error: %d\n", error);
		goto err_free_input;
	}
	return error;
err_free_input:
	input_free_device(pvivo_key->input_dev);
	pvivo_key->input_dev = NULL;
	return -EINVAL;
}

static void vivo_key_release_button(struct vivo_key *pvivo_key)
{
	int i;
	for (i = 0; i < BUTTON_NUM; i++) {
		if (test_bit(i, &pvivo_key->pre_state)) {
			mutex_lock(&pvivo_key->input_dev->mutex);
			input_event(pvivo_key->input_dev, EV_KEY, button_map[i], 0);
			input_sync(pvivo_key->input_dev);
			mutex_unlock(&pvivo_key->input_dev->mutex);
		}
	}
	VIVO_KEY_LOG_INF("release button %d\n", (int)pvivo_key->pre_state);
	pvivo_key->pre_state = 0;
	set_virtual_key_state(0);
	get_finger_when_key_down();		/*just for clear variable finger_when_key_down*/
}

static void suspend_work_func(struct work_struct *pwork)
{
	struct vivo_key *pvivo_key = container_of(pwork, struct vivo_key, suspend_work);
	if (!pvivo_key) {
		VIVO_KEY_LOG_INF("pvivo_key is NULL\n");
		return;
	}
	mutex_lock(&pvivo_key->suspend_mutex);
	vivo_key_release_button(pvivo_key);
	if (pvivo_key->vivo_key_standard_suspend)
		pvivo_key->vivo_key_standard_suspend(pvivo_key);
	mutex_unlock(&pvivo_key->suspend_mutex);
	return;
}

static void resume_work_func(struct work_struct *pwork)
{
	struct vivo_key *pvivo_key = container_of(pwork, struct vivo_key, resume_work);
	if (!pvivo_key) {
		VIVO_KEY_LOG_INF("pvivo_key is NULL\n");
		return;
	}
	mutex_lock(&pvivo_key->suspend_mutex);
	if (pvivo_key->vivo_key_standard_resume)
		pvivo_key->vivo_key_standard_resume(pvivo_key);
	mutex_unlock(&pvivo_key->suspend_mutex);
	return;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void vivo_key_early_suspend(struct early_suspend *h)
{
	int ret;
	struct vivo_key *pvivo_key = container_of(h, struct vivo_key, early_suspend);
	if (!pvivo_key) {
		VIVO_KEY_LOG_INF("%s pvivo_key is NULL \n", __func__);
		return;
	}
	queue_work(pvivo_key->suspend_workqueue, &pvivo_key->suspend_work);
}

static void vivo_key_late_resume(struct early_suspend *h)
{
	int ret;
	struct vivo_key *pvivo_key = container_of(h, struct vivo_key, early_suspend);
	if (!pvivo_key) {
		VIVO_KEY_LOG_INF("%s pvivo_key is NULL \n", __func__);
		return;
	}
	queue_work(pvivo_key->suspend_workqueue, &pvivo_key->resume_work);
}
#elif defined(CONFIG_FB) /*this is only called one time int power on*/
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	struct vivo_key *pvivo_key = NULL;
	VIVO_KEY_LOG_INF("CYTTSP ENTER %s\n", __func__);
	if (!self) {
		VIVO_KEY_LOG_ERR("%s: self is NULL\n ", __func__);
		return 0;
	}
	pvivo_key = container_of(self, struct vivo_key, fb_notif);
	if (!pvivo_key) {
		VIVO_KEY_LOG_INF("%s pvivo_key is NULL \n", __func__);
		return 0;
	}
	VIVO_KEY_LOG_INF(" %s event=%d\n", __func__, (int)event);
	if (evdata && evdata->data &&
			event == FB_EARLY_EVENT_BLANK)
/*			event == FB_EVENT_BLANK)			*/
	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			VIVO_KEY_LOG_INF("UNBLANK SCREEN FOR CYTTSP\n");
			queue_work(pvivo_key->suspend_workqueue, &pvivo_key->resume_work);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			VIVO_KEY_LOG_INF("BLANK SCREEN FOR CYTTSP\n");
			queue_work(pvivo_key->suspend_workqueue, &pvivo_key->suspend_work);
		}
	}

	return 0;
}
#endif


static ssize_t virtual_key_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;
	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t virtual_key_debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void virtual_key_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops virtual_key_debug_object_sysfs_ops = {
	.show = virtual_key_debug_object_show,
	.store = virtual_key_debug_object_store,
};

static struct kobj_type virtual_key_debug_object_type = {
	.sysfs_ops	= &virtual_key_debug_object_sysfs_ops,
	.release	= virtual_key_debug_object_release,
	.default_attrs = NULL,
};

static int touchscreen_debugfs_open(struct inode *inode, struct file *pfile)
{
	return 0;
}

static ssize_t touchscreen_debugfs_read(struct file *pfile, char __user *buf, size_t count, loff_t *pos)
{
	struct vivo_key *pvivo_key = global_key;
	int id = 0;
	char local_buf[30];
	int len = 0;
	id = pvivo_key->at_i2c_test(pvivo_key);
	len += snprintf(local_buf, 30, "%s id=%d", (id != 0) ? "Pass" : "Failed", id);
	return simple_read_from_buffer(buf, count, pos, local_buf, len);
}

struct file_operations touchscreen_debugfs_ops = {
	.open = touchscreen_debugfs_open,
	.read = touchscreen_debugfs_read,
};

extern unsigned int is_atboot;

static int vitualkey_create_our_owner_sys_file(struct vivo_key *pvivo_key)
{
	int ret = 0;
	struct kobject *kobject_debug = get_debug_kobject();
	if (!virtual_key_debug_object_type.default_attrs) {
		VIVO_KEY_LOG_INF("default_attrs or is NULL\n");
		return -EINVAL;
	}
	if (pvivo_key->at_i2c_test) {
		pvivo_key->touchscreen_entry = pvivo_key->i2c_test_entry = NULL;
		pvivo_key->touchscreen_entry = debugfs_create_dir("touchscreen", NULL);
		if (pvivo_key->touchscreen_entry == NULL) {
			VIVO_KEY_LOG_ERR("Failed to create /sys/kernel/debug/touchscreen\n");
			return -EIO;
		}
		pvivo_key->i2c_test_entry = debugfs_create_file("key_i2c_test", 0444, pvivo_key->touchscreen_entry, NULL, &touchscreen_debugfs_ops);
		if (pvivo_key->i2c_test_entry == NULL) {
			VIVO_KEY_LOG_ERR("Failed to create /sys/kernel/debug/touchscreen\n");
			debugfs_remove(pvivo_key->touchscreen_entry);
			pvivo_key->touchscreen_entry = NULL;
			return -EIO;
		}
	}
	if (!kobject_debug) {
		VIVO_KEY_LOG_INF("Failed to get touchscreen debug object\n");
	} else {
		ret = kobject_init_and_add(&pvivo_key->key_kobject_debug, &virtual_key_debug_object_type,
			 kobject_debug, "virtual_key");
		if (ret < 0) {
			VIVO_KEY_LOG_INF("Failed to create debug node \n");
		}
	}
	VIVO_KEY_LOG_INF("Success to create node \n");
	return ret;
}

static void vitualkey_remove_our_owner_sys_file(struct vivo_key *pvivo_key)
{
	if (!virtual_key_debug_object_type.default_attrs) {
		VIVO_KEY_LOG_ERR("default_attrs or is NULL\n");
		return;
	}
	if (pvivo_key->at_i2c_test && pvivo_key->i2c_test_entry) {
		debugfs_remove(pvivo_key->i2c_test_entry);
		debugfs_remove(pvivo_key->touchscreen_entry);
	}

	kobject_del(&pvivo_key->key_kobject_debug);

	return;
}

int register_vivo_key(struct vivo_key *pvivo_key)
{
	int error = 0;
	if (!pvivo_key) {
		VIVO_KEY_LOG_ERR("The up_slide pointer is NULL\n");
		return -EINVAL;
	}

	setup_timer(&pvivo_key->menu_release_timer,
				menu_release_timer_handler, (unsigned long)pvivo_key);
	INIT_WORK(&pvivo_key->menu_release_work, menu_release_work_handler);

	setup_timer(&pvivo_key->back_release_timer,
				back_release_timer_handler, (unsigned long)pvivo_key);
	INIT_WORK(&pvivo_key->back_release_work, back_release_work_handler);
#if defined(HOME_KEY_SUPPORTED)
	setup_timer(&pvivo_key->home_release_timer,
				home_release_timer_handler, (unsigned long)pvivo_key);
	INIT_WORK(&pvivo_key->home_release_work, home_release_work_handler);
#endif

	mutex_init(&pvivo_key->suspend_mutex);

	error = input_dev_setup(pvivo_key);
	if (error < 0) {
		VIVO_KEY_LOG_ERR("Failed to setup input device\n");
		goto delete_timer_err;
	}

	if (pvivo_key->vivo_key_standard_suspend && pvivo_key->vivo_key_standard_resume) {
		INIT_WORK(&pvivo_key->suspend_work, suspend_work_func);
		INIT_WORK(&pvivo_key->resume_work, resume_work_func);
		pvivo_key->suspend_workqueue = create_singlethread_workqueue("vivo_key_workqueue");
		if (!pvivo_key->suspend_workqueue) {
			VIVO_KEY_LOG_ERR("%s: Can't create vivo_key_workqueue\n", __func__);
			error = -EIO;
			goto cancel_suspend_work_err;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		VIVO_KEY_LOG_INF("vivo key supported early_suspend\n");
		pvivo_key->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		pvivo_key->early_suspend.suspend = vivo_key_early_suspend;
		pvivo_key->early_suspend.resume = vivo_key_late_resume;
		register_early_suspend(&pvivo_key->early_suspend);
#elif defined(CONFIG_FB)
		VIVO_KEY_LOG_INF("vivo key supported notifier_callback\n");
		pvivo_key->fb_notif.notifier_call = fb_notifier_callback;
		error = fb_register_client(&pvivo_key->fb_notif);
		if (error) {
			VIVO_KEY_LOG_ERR("Unable to register fb_notifier: %d", error);
			goto destroy_workqueue_err;
		}
#endif
	} else {
		VIVO_KEY_LOG_INF("%s: standard suspend or resume is not initialed\n", __func__);
	}

	virtual_key_debug_object_type.default_attrs = pvivo_key->button_attr;
	vitualkey_create_our_owner_sys_file(pvivo_key);
	global_key = pvivo_key;
	return 0;


destroy_workqueue_err:
	flush_workqueue(pvivo_key->suspend_workqueue);
	destroy_workqueue(pvivo_key->suspend_workqueue);
cancel_suspend_work_err:
	cancel_work_sync(&pvivo_key->resume_work);
	cancel_work_sync(&pvivo_key->suspend_work);

	input_unregister_device(pvivo_key->input_dev);
/*	input_free_device(pvivo_key->input_dev);*/
/************************************************
Don't call input_free_device() after
input_unregister_device()

    input_free_device() should only be used if
    input_register_device() was not called yet or if it failed. Once
    device was unregistered use input_unregister_device() and memory
    will be freed once last reference to the device is dropped.
************************************************/
	pvivo_key->input_dev = NULL;
delete_timer_err:
	del_timer_sync(&pvivo_key->menu_release_timer);
	del_timer_sync(&pvivo_key->back_release_timer);
#if defined(HOME_KEY_SUPPORTED)
	del_timer_sync(&pvivo_key->home_release_timer);
#endif
	cancel_work_sync(&pvivo_key->menu_release_work);
	cancel_work_sync(&pvivo_key->back_release_work);
#if defined(HOME_KEY_SUPPORTED)
	cancel_work_sync(&pvivo_key->home_release_work);
#endif
	return error;
}

void unregister_vivo_key(struct vivo_key *pvivo_key)
{
	global_key = NULL;
	if (!pvivo_key) {
		VIVO_KEY_LOG_ERR("The up_slide pointer is NULL\n");
		return;
	}
	vitualkey_remove_our_owner_sys_file(pvivo_key);

	if (pvivo_key->vivo_key_standard_suspend && pvivo_key->vivo_key_standard_resume) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&pvivo_key->early_suspend);
#elif defined(CONFIG_FB)
		if (fb_unregister_client(&pvivo_key->fb_notif))
			VIVO_KEY_LOG_ERR("falied unregister notifier\n");
#endif
		flush_workqueue(pvivo_key->suspend_workqueue);
		destroy_workqueue(pvivo_key->suspend_workqueue);
		cancel_work_sync(&pvivo_key->resume_work);
		cancel_work_sync(&pvivo_key->suspend_work);
	}

	input_unregister_device(pvivo_key->input_dev);
/*	input_free_device(pvivo_key->input_dev);*/
	pvivo_key->input_dev = NULL;

	del_timer_sync(&pvivo_key->menu_release_timer);
	del_timer_sync(&pvivo_key->back_release_timer);
#if defined(HOME_KEY_SUPPORTED)
	del_timer_sync(&pvivo_key->home_release_timer);
#endif
	cancel_work_sync(&pvivo_key->menu_release_work);
	cancel_work_sync(&pvivo_key->back_release_work);
#if defined(HOME_KEY_SUPPORTED)
	cancel_work_sync(&pvivo_key->home_release_work);
#endif

	return;
}

