#include "vts_log_switch.h"
#include "vts_core.h"

static spinlock_t handler_lock;
static LIST_HEAD(drivers_callbacks);
static spinlock_t screenshot_lock;
static void screenshot_work_func(struct work_struct *work);
DECLARE_WORK(screenshot_work,screenshot_work_func);

int vts_log_switch_register(struct vts_callback_handler *vts_handler)
{
	bool match = false;
	struct vts_callback_handler *entry = NULL;

	spin_lock(&handler_lock);
	if (list_empty(&drivers_callbacks)) {
		list_add_tail(&vts_handler->list, &drivers_callbacks);
		spin_unlock(&handler_lock);
		return 0;
	}

	list_for_each_entry(entry, &drivers_callbacks, list) {
		if (entry == vts_handler)
			match = true;
	}

	if (!match)
		list_add_tail(&vts_handler->list, &drivers_callbacks);
	spin_unlock(&handler_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(vts_log_switch_register);

void vts_log_switch_unregister(struct vts_callback_handler *vts_handler)
{
	struct vts_callback_handler *entry = NULL;
	struct vts_callback_handler *next = NULL;

	spin_lock(&handler_lock);
	list_for_each_entry_safe(entry, next, &drivers_callbacks, list) {
		if (entry == vts_handler) {
			list_del(&entry->list);
			spin_unlock(&handler_lock);
			return;
		}
	}
	spin_unlock(&handler_lock);
}
EXPORT_SYMBOL_GPL(vts_log_switch_unregister);
static bool vts_log_switch_do_callback(bool is_switch_on)
{
	struct vts_callback_handler *entry;

	VTI("called(%d)\n", is_switch_on);
	rcu_read_lock();
	list_for_each_entry_rcu(entry, &drivers_callbacks, list) {
			entry->callback(is_switch_on);
	}
	rcu_read_unlock();
	return is_switch_on;
}

static struct volume_key_trace_info {
	spinlock_t key_lock;
	int key_num;
	unsigned long down_time;
	unsigned int last_key_type;
	bool is_switch_on;
}vol_key_info;

static void vts_set_key(int num, unsigned long time, unsigned int type, bool index)
{
	spin_lock(&vol_key_info.key_lock);
	vol_key_info.key_num = num;
	vol_key_info.down_time = time;
	vol_key_info.last_key_type = type;
	vol_key_info.is_switch_on = index;
	spin_unlock(&vol_key_info.key_lock);
}

static void vts_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	unsigned long now = jiffies;
	unsigned long max_time_length = 3 * HZ;
	bool index = vol_key_info.is_switch_on;

	if (type != EV_KEY)
		return;

	if (!value)/*0-release, 1-press*/
		return;

	if (now - vol_key_info.down_time > max_time_length)
		vts_set_key(0, now, KEY_RESERVED, index);

	switch (vol_key_info.last_key_type) {
	case KEY_RESERVED:
		if (code == KEY_VOLUMEUP && vol_key_info.key_num < 4)
			/* for depend mistouch do not clear trace info
			 * so it can't turn on/off log immediatly in this 3s
			 */
			vts_set_key(1, now, KEY_VOLUMEUP, index);
		break;
	case KEY_VOLUMEUP:
		if (code == KEY_VOLUMEUP) {
			/* press + + */
			index = index ? vts_log_switch_do_callback(false) : index;
			vts_set_key(4, now, KEY_RESERVED, index);
		} else if (code == KEY_VOLUMEDOWN && vol_key_info.key_num == 3) {
			/* press + - +, num is 3, current code is KEY_VOLUMEDOWN */
			index = index ? index : vts_log_switch_do_callback(true);
			vts_set_key(vol_key_info.key_num + 1, now, KEY_RESERVED, index);
		} else {
			/* press +, num is 1, current code is KEY_VOLUMEDOWN, num++ */
			vts_set_key(vol_key_info.key_num + 1, now, KEY_VOLUMEDOWN, index);
		}
		break;
	case KEY_VOLUMEDOWN:
		if (code == KEY_VOLUMEUP) {
			vts_set_key(vol_key_info.key_num + 1, now, KEY_VOLUMEUP, index);
		} else {
			vts_set_key(0, now, KEY_RESERVED, index);
		}
		break;
	default:
		/* add something */
		break;
	}
}

static int vts_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	VTI("input device %s connecting\n", dev->name);

#ifdef CONFIG_MEDIATEK_SOLUTION
	if (strcmp(dev->name, "mtk-kpd")) {
		VTE("%s: Not volume key input device\n", dev->name);
		return -1;
	}
#elif defined CONFIG_ARCH_QCOM
	if ((strcmp(dev->name, "gpio-keys")) && (strcmp(dev->name, "qpnp_pon")))  {
		VTE("%s: Not volume key input device\n", dev->name);
		return -1;
	}
#else
	if (strcmp(dev->name, "gpio_keys")) {
		VTE("%s: Not volume key input device\n", dev->name);
		return -1;
	}
#endif

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vts_log_handle";

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

static void vts_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id vts_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler vts_input_handler = {
	.event		= vts_input_event,
	.connect	= vts_input_connect,
	.disconnect	= vts_input_disconnect,
	.name		= "vts_log_siwtch_handler",
	.id_table	= vts_ids,
};

static int vts_screenshot_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;
	static int handle_index = 0;
	const int name_length = 32;
	char *handle_name = NULL;

	VTI("input device %s connecting\n", dev->name);

#ifdef CONFIG_MEDIATEK_SOLUTION
	if (strcmp(dev->name, "mtk-kpd")) {
		VTE("%s: Not volume key input device\n", dev->name);
		return -1;
	}
#elif ((defined CONFIG_ARCH_QCOM) && !defined(CONFIG_ARCH_EXYNOS))
	if ((strcmp(dev->name, "gpio-keys")) && (strcmp(dev->name, "qpnp_pon")))  {
		VTE("%s: Not volume key input device\n", dev->name);
		return -1;
	}
#else
	if (strcmp(dev->name, "gpio_keys")/* volume key*/ && strcmp(dev->name, "s2mpu13-power-keys")/* power key */) {
		VTE("%s: Not volume key or Poweer key input device\n", dev->name);
		return -1;
	}
#endif

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle_name = kzalloc(name_length, GFP_KERNEL);
	if (!handle_name) {
		error = -ENOMEM;
		goto err3;
	}
	snprintf(handle_name, name_length, "vts_screen_shot_handle%d", handle_index);
	handle->name = handle_name;

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
	kfree(handle->name);
err3:
	kfree(handle);
	return error;
}

static void vts_screenshot_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle->name);
	kfree(handle);
}

static void vts_screenshot_do_callback(unsigned int count)
{
	struct vts_callback_handler *entry;
	VTI("screenshot called");
	rcu_read_lock();
	list_for_each_entry_rcu(entry, &drivers_callbacks, list) {
		if (entry->screenshot_callback)
			entry->screenshot_callback(entry, count);
	}
	rcu_read_unlock();
	return;
}
static void screenshot_work_func(struct work_struct *work)
{
	static unsigned int triger_count = 0;
	vts_screenshot_do_callback(++triger_count);
}

static void vts_screenshot_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	static unsigned long pre_down = 0;
	static unsigned int last_down_key = 0;
	unsigned long now = jiffies;
	const unsigned long max_time_length = HZ / 4;
	bool is_callback_triger = 0;

	if (type != EV_KEY)
		return;

	if (!value)/*0-release, 1-press*/
		return;
		
	switch(code) {
		case KEY_VOLUMEDOWN:
			spin_lock(&screenshot_lock);
			if (value == 0) {
				pre_down = 0;
				last_down_key = 0;
			} else {
				if (last_down_key == KEY_POWER) {
					if ((now - pre_down) < max_time_length) {
						is_callback_triger = 1;
					}
				}
				last_down_key = KEY_VOLUMEDOWN;
				pre_down = now;
			}
			spin_unlock(&screenshot_lock);
			break;
		case KEY_POWER:
			spin_lock(&screenshot_lock);
			if (value == 0) {
				pre_down = 0;
				last_down_key = 0;
			} else {
				if (last_down_key == KEY_VOLUMEDOWN) {
					if ((now - pre_down) < max_time_length) {
						is_callback_triger = 1;
					}
				}
				last_down_key = KEY_POWER;
				pre_down = now;
			}
			spin_unlock(&screenshot_lock);
			break;
		default:
			break;
	}
	if (is_callback_triger) {
		schedule_work(&screenshot_work);
	}
	return;
}

static struct input_handler vts_screenshot_handler = {
	.event		= vts_screenshot_event,
	.connect	= vts_screenshot_connect,
	.disconnect	= vts_screenshot_disconnect,
	.name		= "vts_screenshot_handler",
	.id_table	= vts_ids,
};

int vts_log_switch_init(void)
{
	int error;

	spin_lock_init(&handler_lock);
	spin_lock_init(&vol_key_info.key_lock);
	spin_lock_init(&screenshot_lock);
	vts_set_key(0, jiffies, KEY_RESERVED, false);
	error = input_register_handler(&vts_input_handler);
	if (error) {
		VTE("%s: register input handler failed\n", __func__);
		return error;
	}
	error = input_register_handler(&vts_screenshot_handler);
	if (error) {
		VTE("register screenshot handler failed");
		goto err1;
	}
	return 0;

err1:
	input_unregister_handler(&vts_input_handler);
	return error;
}

void vts_log_switch_exit(void)
{
	input_unregister_handler(&vts_screenshot_handler);
	input_unregister_handler(&vts_input_handler);
}