#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include "vts_core.h"

static const char *name_of_events[] = {
	[VTS_EVENT_GESTURE_FINGERPRINT_DETECT] 		= "KEY_FINGER_DETECT",
	[VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT] 	= "KEY_FINGER_DOUBLE_DETECT",
	[VTS_EVENT_GESTURE_FACE_DETECT] 		= "KEY_FACE_DETECT",
	[VTS_EVENT_GESTURE_LARGE_AREA_PRESS] 		= "KEY_LARGE_SUPPRESSION",
	[VTS_EVENT_GESTURE_DOUBLE_CLICK] 		= "KEY_WAKEUP",
	[VTS_EVENT_GESTURE_TAKE_CAMERA] 		= "KEY_CAMERA",
	[VTS_EVENT_GESTURE_PATTERN_C] 			= "KEY_C",
	[VTS_EVENT_GESTURE_PATTERN_E] 			= "KEY_E",
	[VTS_EVENT_GESTURE_PATTERN_M] 			= "KEY_M",
	[VTS_EVENT_GESTURE_PATTERN_W] 			= "KEY_W",
	[VTS_EVENT_GESTURE_PATTERN_A] 			= "KEY_A",
	[VTS_EVENT_GESTURE_PATTERN_F] 			= "KEY_F",
	[VTS_EVENT_GESTURE_PATTERN_O] 			= "KEY_O",
	[VTS_EVENT_GESTURE_PATTERN_V] 			= "KEY_V",
	[VTS_EVENT_GESTURE_PATTERN_HEART] 		= "KEY_HEART",
	[VTS_EVENT_GESTURE_PATTERN_LEFT] 		= "KEY_LEFT",
	[VTS_EVENT_GESTURE_PATTERN_RIGHT] 		= "KEY_RIGHT",
	[VTS_EVENT_GESTURE_PATTERN_UP] 			= "KEY_UP",
	[VTS_EVENT_GESTURE_PATTERN_DOWN] 		= "KEY_DOWN",
	[VTS_EVENT_GESTURE_PATTERN_SWAP] 		= "KEY_TS_SWIPE",
	[VTS_EVENT_GESTURE_VK_LONG_PRESS] 		= "KEY_LONGPRESS_DOWN",
	[VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE] 	= "KEY_LONGPRESS_UP",
	[VTS_EVENT_GESTURE_VK_DC] 			= "KEY_VIRTUAL_WAKEUP",
	[VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK]		= "KEY_SCREEN_CLOCK_DCLICK",
	[VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT]	= "KEY_VK_INSIDE_SLIDE_LEFT",
	[VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT]	= "KEY_VK_INSIDE_SLIDE_RIGHT",
	[VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE]		= "KEY_VK_QUIT_ACTIVE_MODE",
	[VTS_EVENT_GESTURE_PATTERN_H] 			= "KEY_H",
	[VTS_EVENT_GESTURE_FINGER3_MODE]		= "KEY_FINGER3_MODE",
};

/*
*	TP report point callback
*/

struct tp_point_callback {
	struct list_head list;
	struct drivers_callback_handler *handler;
} point_callbacks;

int tp_report_point_register_callback(struct drivers_callback_handler *handler) {	

	struct tp_point_callback *new_callback = NULL;

	if (!vts_is_module_match()) {
		VTE("no match module!!! not register force callback");
		return -EPERM;
	}

	new_callback = kzalloc(sizeof(struct tp_point_callback), GFP_KERNEL);
	if (!new_callback) {
		pr_err("%s: Failed at allocate callback struct\n", __func__);
		return -ENOMEM;
	}

	new_callback->handler = handler;
	INIT_LIST_HEAD(&new_callback->list);
	list_add_tail(&new_callback->list, &point_callbacks.list);

	return 0;
}
EXPORT_SYMBOL_GPL(tp_report_point_register_callback);

void tp_report_point_unregister_callback(char *callback_name) {
	struct tp_point_callback *entry;
	
	if (!list_empty(&point_callbacks.list)) {
		list_for_each_entry(entry, &point_callbacks.list, list)
			if (!strcmp(entry->handler->name, callback_name)) {
				list_del(&entry->list);
				kfree(entry);
				return;
			}
	}
}
EXPORT_SYMBOL_GPL(tp_report_point_unregister_callback);


static void tp_point_switch_do_callback(struct vts_point *point_info, int is_down)
{
	struct tp_point_callback *entry;
	
	if (!list_empty(&point_callbacks.list)) {
		list_for_each_entry(entry, &point_callbacks.list, list) {			
			entry->handler->point_report(point_info, is_down);
		}
	}
}

static const char *vts_event_name(enum vts_event event)
{
	if (unlikely(event >= ARRAY_SIZE(name_of_events)))
		return "Unnamed state";

	return name_of_events[event];
}

#ifdef CONFIG_INPUT_TIMESTAMP
static void vts_input_event(struct input_dev *dev,
		 unsigned int type, unsigned int code, int value, ktime_t timestamp)
{
	struct vts_device *vtsdev = input_get_drvdata(dev);

	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_event_timestamp(dev, type, code, value, timestamp);
	else
		input_event(dev, type, code, value);
}

static void vts_input_mt_report_slot_state(struct input_dev *dev,
				unsigned int tool_type, bool active, ktime_t timestamp)
{
	struct vts_device *vtsdev = input_get_drvdata(dev);

	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_mt_report_slot_state_timestamp(dev, tool_type, active, timestamp);
	else
		input_mt_report_slot_state(dev, tool_type, active);
}

#else
static void vts_input_event(struct input_dev *dev,
		 unsigned int type, unsigned int code, int value, ktime_t timestamp)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#if (defined(CONFIG_ARCH_QCOM) && IS_ENABLED(CONFIG_QGKI)) || (defined(CONFIG_ARCH_EXYNOS) && !IS_ENABLED(CONFIG_GKI_HIDDEN_VIVO_CONFIGS))
	struct vts_device *vtsdev = input_get_drvdata(dev);
	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_set_timestamp(dev, timestamp);
#endif
#endif
	input_event(dev, type, code, value);
}

static void vts_input_mt_report_slot_state(struct input_dev *dev,
				unsigned int tool_type, bool active, ktime_t timestamp)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#if (defined(CONFIG_ARCH_QCOM) && IS_ENABLED(CONFIG_QGKI)) || (defined(CONFIG_ARCH_EXYNOS) && !IS_ENABLED(CONFIG_GKI_HIDDEN_VIVO_CONFIGS))
	struct vts_device *vtsdev = input_get_drvdata(dev);
	if (vtsdev->report.flags & FLAGS_REPORT_TIMESTAMP)
		input_set_timestamp(dev, timestamp);
#endif
#endif
	input_mt_report_slot_state(dev, tool_type, active);
}
#endif

static void vts_input_report_key(struct input_dev *dev, unsigned int code, int value, ktime_t kt)
{
	vts_input_event(dev, EV_KEY, code, !!value, kt);
}

static void vts_input_sync(struct input_dev *dev, ktime_t kt)
{
	vts_input_event(dev, EV_SYN, SYN_REPORT, 0, kt);
}

static void vts_input_report_abs(struct input_dev *dev, unsigned int code, int value, ktime_t kt)
{
	vts_input_event(dev, EV_ABS, code, value, kt);
}

static void vts_input_mt_slot(struct input_dev *dev, int slot, ktime_t kt)
{
	vts_input_event(dev, EV_ABS, ABS_MT_SLOT, slot, kt);
}

struct vts_report_event {
	struct list_head list;
	int touch_id;
	int nr_touches;
	int x;
	int y;
	int wx;
	int wy;
	int keycode;
	ktime_t kt;
};

static void inline vts_report_init_event(struct vts_report_event *event, int touch_id,
	int nr_touches, int x, int y, int wx, int wy, int keycode, ktime_t kt)
{
	event->touch_id = touch_id;
	event->nr_touches = nr_touches;
	event->x = x;
	event->y = y;
	event->wx = wx;
	event->wy = wy;
	event->keycode = keycode;
	event->kt = kt;
	INIT_LIST_HEAD(&event->list);
}

static int vts_report_add_event(struct vts_report *report, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, int keycode, ktime_t kt, struct list_head *head)
{
	struct vts_report_event *event;
	struct vts_device *vtsdev = container_of(report, struct vts_device, report);

	event = kmem_cache_zalloc(report->km, GFP_ATOMIC);
	if (!event) {
		vts_dev_err(vtsdev, "alloc point mem failed!\n");
		return -ENOMEM;
	}

	vts_report_init_event(event, touch_id, nr_touches, x, y, wx, wy, keycode, kt);
	list_add_tail(&event->list, head);
	return 0;
}

static void vts_report_remove_event(struct vts_report *report, struct vts_report_event *event)
{
	list_del(&event->list);
	kmem_cache_free(report->km, event);
}

static int vts_report_add_key(struct vts_report *report, int keycode)
{
	return vts_report_add_event(report, 0, 0, 0, 0, 0, 0, keycode, ktime_get(), &report->keys);
}

static int vts_report_add_point_down(struct vts_report *report, int touch_id, int nr_touches, int x,int y,int wx,int wy, ktime_t kt)
{
	return vts_report_add_event(report, touch_id, nr_touches, x, y, wx, wy, 0, kt, &report->down_points);
}

static int vts_report_add_point_up(struct vts_report *report, int touch_id, int nr_touches, int x,int y,int wx,int wy, ktime_t kt)
{
	return vts_report_add_event(report, touch_id, nr_touches, x, y, wx, wy, 0, kt, &report->up_points);
}

static struct vts_report_event *vts_report_find_key(struct vts_report *report, int key_code)
{
	struct vts_report_event *key;

	list_for_each_entry(key, &report->keys, list) {
		if (key->keycode == key_code)
			return key;
	}

	return NULL;
}

/* if set 15s lcd off,while press ts not move,maybe lcd will off. */
static void vts_long_press_work(struct work_struct *work)
{
	struct vts_long_press_report *long_press_report = container_of(work, struct vts_long_press_report, long_press_work);
	struct vts_device *vtsdev = container_of(long_press_report, struct vts_device, long_press_report);
	struct vts_report *report = &vtsdev->report;
	ktime_t kt = ktime_get();

	long_press_report->work_status = 0;
	if(mutex_trylock(&report->lock)) {
		vts_input_mt_slot(vtsdev->idev, long_press_report->slot, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MAJOR, long_press_report->major, kt);
		if (list_empty(&report->down_points))
			vts_input_sync(vtsdev->idev, kt);
		VTI("long press for 10s, report TOUCH_SLOT: %d, TOUCH_MAJOR: %d to keep lcd on", long_press_report->slot, long_press_report->major);
		mutex_unlock(&report->lock);
	}
}

static enum hrtimer_restart vts_long_press_report(struct hrtimer *timer)
{
	struct vts_long_press_report *long_press_report = container_of(timer, struct vts_long_press_report, long_press_timer);

	long_press_report->major = long_press_report->major > long_press_report->index ? long_press_report->major - 1 : long_press_report->major + 1;
	schedule_work(&long_press_report->long_press_work);
	long_press_report->work_status = 1;
	hrtimer_forward_now(&long_press_report->long_press_timer, ms_to_ktime(12000));
	return HRTIMER_RESTART;
}

void vts_long_press_timer_init(struct vts_device *vtsdev)
{
	struct vts_long_press_report *report = &vtsdev->long_press_report;

	/* long press report major */
	hrtimer_init(&report->long_press_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	report->long_press_timer.function = vts_long_press_report;
	INIT_WORK(&vtsdev->long_press_report.long_press_work, vts_long_press_work);
}

void vts_long_press_timer_deinit(struct vts_device *vtsdev)
{
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}

	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);
}

int vts_report_coordinates_set(struct vts_device *vtsdev, u16 *x, u16 *y, int nr_points)
{
	struct vts_report *report = &vtsdev->report;
	int i = 0;
	int resolution = 0;
	int display_x = 1;
	int display_y = 1;
	int dimention_x = 1;
	int dimention_y = 1;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);
	}

	for (i = 0; i < nr_points; i++) {
		if (resolution && x[i] <= dimention_x && y[i] <= dimention_y) {
			x[i] = x[i] * display_x / dimention_x;
			y[i] = y[i] * display_y / dimention_y;
		}
		vts_dev_info(vtsdev, "set point[%d] x:%d y:%d\n", i, x[i], y[i]);
	}

	if (nr_points == 0 ||
		nr_points > ARRAY_SIZE(report->coordinates_x) ||
		nr_points > ARRAY_SIZE(report->coordinates_y))
		return -EINVAL;

	mutex_lock(&report->lock);
	report->nr_coordinates = 32;
	memset(report->coordinates_x, 0xff, ARRAY_SIZE(report->coordinates_x) * sizeof(*x));
	memset(report->coordinates_y, 0xff, ARRAY_SIZE(report->coordinates_y) * sizeof(*y));
	memcpy(report->coordinates_x, x, nr_points * sizeof(*x));
	memcpy(report->coordinates_y, y, nr_points * sizeof(*y));
	mutex_unlock(&report->lock);
	return 0;
}

int vts_report_coordinates_get(struct vts_device *vtsdev, u16 *x, u16 *y, size_t size, int *nr_points)
{
	int i = 0;
	struct vts_report *report = &vtsdev->report;

	if (report->nr_coordinates > size)
		return -EINVAL;

	if (report->nr_coordinates == 0)
		return -EBUSY;

	mutex_lock(&report->lock);
	*nr_points = report->nr_coordinates;
	memcpy(x, report->coordinates_x, *nr_points * sizeof(*x));
	memcpy(y, report->coordinates_y, *nr_points * sizeof(*y));
	report->nr_coordinates = 0;
	mutex_unlock(&report->lock);

	for (i = 0; i < *nr_points; i++) {
		if(x[i] != 0xffff && y[i] != 0xffff) {
			vts_dev_info(vtsdev, "get point[%d] x:%d y:%d\n", i, x[i], y[i]);
		}
	}
	return 0;
}

static int vts_report_key_down(struct vts_device *vtsdev, int keycode)
{
	struct vts_report *report = &vtsdev->report;
	int ret;
	ktime_t kt = ktime_get();

	mutex_lock(&report->lock);
	if (vts_report_find_key(report, keycode)) {
		vts_dev_info(vtsdev, "keycode 0x%x already down, can not down again!\n", keycode);
		mutex_unlock(&report->lock);
		return -EBUSY;
	}

	if (VTS_KEY_FINGER_GESTURE == keycode) {
		vts_input_report_key(vtsdev->idev_fp, keycode, 1, kt);
		vts_input_sync(vtsdev->idev_fp, kt);
	} else {
		vts_input_report_key(vtsdev->idev, keycode, 1, kt);
		vts_input_sync(vtsdev->idev, kt);
	}
	ret = vts_report_add_key(report, keycode);
	mutex_unlock(&report->lock);
	return ret;
}

static int vts_report_key_up(struct vts_device *vtsdev, int keycode)
{
	struct vts_report_event *key;
	struct vts_report *report = &vtsdev->report;
	ktime_t kt = ktime_get();

	mutex_lock(&report->lock);
	key = vts_report_find_key(report, keycode);
	if (!key) {
		vts_dev_info(vtsdev, "keycode 0x%x not down, can not up!\n", keycode);
		mutex_unlock(&report->lock);
		return -EBUSY;
	}

	if (VTS_KEY_FINGER_GESTURE == keycode) {
		vts_input_report_key(vtsdev->idev_fp, keycode, 0, kt);
		vts_input_sync(vtsdev->idev_fp, kt);
	} else {
		vts_input_report_key(vtsdev->idev, keycode, 0, kt);
		vts_input_sync(vtsdev->idev, kt);
	}
	vts_report_remove_event(report, key);
	mutex_unlock(&report->lock);
	return 0;
}
int vts_update_dclick_point(struct vts_device *vtsdev,int x ,int y)
{
 	vtsdev->screen_clock_point.realX = x ;
	vtsdev->screen_clock_point.realY = y ;
	return 0;
}
int vts_report_event_down(struct vts_device *vtsdev, enum vts_event event)
{
	struct vts_report *report = &vtsdev->report;
	bool ignore;
	int keycode;
	int ret = 0;
	int fp_notifier = 0;
	int fp_aoi = 0;
	struct vts_touch_event *touch = &vtsdev->event[0];

	vts_dev_info(vtsdev, "event 0x%x: %s\n", event, vts_event_name(event));
	memset(touch, 0, sizeof(struct vts_touch_event));
	touch->type = EVENT_TYPE_DOWN;

	if (event == VTS_EVENT_GESTURE_LARGE_AREA_PRESS) {
		vts_large_press_event(TOUCH_VCODE_LARGE_PRESS_EVENT);
	}

	ignore = report->on_event_down(report, event, &keycode);
	if (ignore) {
		vts_dev_info(vtsdev, "event 0x%x: %s was ignored\n", event, vts_event_name(event));
		return 0;
	}

	if (event == VTS_EVENT_GESTURE_FINGERPRINT_DETECT || event == VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT) {
		vts_property_get(vtsdev, VTS_PROPERTY_FP_NOTIFIER, &fp_notifier);
		vts_property_get(vtsdev, VTS_PROPERTY_AOI, &fp_aoi);
		touch->panel = vtsdev->type;
		if (fp_notifier) {
			vtsdev->aoi_report_enabled = true;
			if (!fp_aoi) {
				vts_notifier_call_chain(EVENT_TYPE_DOWN, (void*)touch);
			}
			return 0;
		}
	}

	ret = vts_report_key_down(vtsdev, keycode);
	report->on_post_key(report,keycode,0);
	return ret;
}

int vts_report_event_up(struct vts_device *vtsdev, enum vts_event event)
{
	struct vts_report *report = &vtsdev->report;
	bool ignore;
	int keycode;
	int fp_notifier = 0;
	int fp_aoi = 0;
	struct vts_touch_event *touch = &vtsdev->event[0];

	vts_dev_info(vtsdev, "event 0x%x: %s\n", event, vts_event_name(event));
	memset(touch, 0, sizeof(struct vts_touch_event));
	touch->type = EVENT_TYPE_UP;

	ignore = report->on_event_up(report, event, &keycode);
	if (ignore) {
		vts_dev_info(vtsdev, "event 0x%x: %s was ignored\n", event, vts_event_name(event));
		return 0;
	}

	if (event == VTS_EVENT_GESTURE_FINGERPRINT_DETECT || event == VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT) {
		vts_property_get(vtsdev, VTS_PROPERTY_FP_NOTIFIER, &fp_notifier);
		vts_property_get(vtsdev, VTS_PROPERTY_AOI, &fp_aoi);
		touch->panel = vtsdev->type;
		if (fp_notifier) {
			vtsdev->aoi_report_enabled = false;
			if (!fp_aoi) {
				vts_notifier_call_chain(EVENT_TYPE_UP, (void*)touch);
			}
			return 0;
		}
	}

	return vts_report_key_up(vtsdev, keycode);
}

int vts_event_trigger(struct vts_device *vtsdev, enum vts_event event)
{
	int ret;

	ret = vts_report_event_down(vtsdev, event);
	if (ret) {
		vts_dev_err(vtsdev, "down event error, event:%d, ret:%d\n", event, ret);
		return ret;
	}

	ret = vts_report_event_up(vtsdev, event);
	if (ret) {
		vts_dev_err(vtsdev, "up event error, event:%d, ret:%d\n", event, ret);
		return ret;
	}

	return 0;
}

void nr_touchs_changed(struct vts_device *vtsdev, enum point_down_or_up report_touch, int nr_touches)
{
	static int last_nr_touches = 0;
	int game_mode_flag = 0;
	int auto_idle_time = 0;

	game_mode_flag = vts_state_get(vtsdev, VTS_STA_GAME_MODE);
	if (game_mode_flag == 0) {
		auto_idle_time = 1000;  //ms
	} else {
		vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &auto_idle_time);   //s
		auto_idle_time *= 10000;   //ms
	}

	if (vtsdev->muti_touch_time[last_nr_touches]) {
		vtsdev->muti_touch_time[last_nr_touches] = ktime_to_ms(ktime_get()) - vtsdev->muti_touch_time[last_nr_touches];
		if (last_nr_touches == 0) {
			if (vtsdev->muti_touch_time[last_nr_touches] <= auto_idle_time) {
				vts_multi_touch(TOUCH_VCODE_MULTI_TOUCH, last_nr_touches, vtsdev->muti_touch_time[last_nr_touches]);
			} else {
				vts_multi_touch(TOUCH_VCODE_MULTI_TOUCH, last_nr_touches, auto_idle_time);
				vts_multi_touch(TOUCH_VCODE_MULTI_TOUCH, 11, vtsdev->muti_touch_time[last_nr_touches] - auto_idle_time);
			}
		} else {
			vts_multi_touch(TOUCH_VCODE_MULTI_TOUCH, last_nr_touches, vtsdev->muti_touch_time[last_nr_touches]);
		}
		vtsdev->muti_touch_time[last_nr_touches] = 0;
	}

	if (report_touch == POINT_DOWN || report_touch == POINT_UP) {
		vtsdev->muti_touch_time[nr_touches] = ktime_to_ms(ktime_get());
	}

	last_nr_touches = nr_touches;
}

int vts_report_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, bool large_press, u8 *custom_data, size_t custom_size, ktime_t kt)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_point point_info;
	int ret = 0;
	u8 buf[64];
	int i;
	int display_x, display_y, dimention_x, dimention_y;
	int reject_pixel = 0;
	int aoi_x = 0;
	int aoi_y = 0;
	int aoi_id = 0;
	int resolution_adjust = 0;
	int resolution_x = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution_adjust);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);
	vts_property_get(vtsdev, VTS_PROPERTY_REJECT_FAIL_COLLECT, &reject_pixel);

	if (resolution_adjust == 0) {
		resolution_x = dimention_x;
	} else {
		resolution_x = display_x;
	}

	point_info.x = x;
	point_info.y = y;
	point_info.wx = wx;
	point_info.touch_id = touch_id;
	point_info.nr_touches = nr_touches;

	vtsdev->point_x[touch_id] = x;
	
	/* if long_press_report timer is running, cancel it when new point down */
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}
	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);

	if (display_x && display_y) {
		if (vtsdev->fw_x == display_x && vtsdev->fw_y == display_y) {
			x = x * dimention_x / vtsdev->fw_x;
			y = y * dimention_y / vtsdev->fw_y;
		}
	
		point_info.physical_x = x * display_x / dimention_x;
		point_info.physical_y = y * display_y / dimention_y;
	} else {
		point_info.physical_x = x;
		point_info.physical_y = y;
	}

	if (vtsdev->resume_end) {
		vtsdev->resume_end = ktime_to_ms(ktime_get()) - vtsdev->resume_end;
		vts_resume_to_touch(TOUCH_VCODE_RESUME_TO_REPORT, vtsdev->resume_end);
		vtsdev->resume_end = 0;
	}
	vtsdev->int_cost_collect_flag = true;

	memset(buf, 0, sizeof(buf));
	snprintf(buf, sizeof(buf), "%s ", "0x:");
	for (i = 0; (i < custom_size) && custom_data; i++)
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), "%x ", custom_data[i]);

	if(!test_bit(touch_id, &report->touchbit)) {
		vtsdev->nr_touches++;
		vts_dev_info(vtsdev, "[%d %04d %04d][%04d %04d]down[%d][%d][%d][%d][%lld %lld][%ld][%s]\n", touch_id, x, y, point_info.physical_x, point_info.physical_y, vtsdev->nr_touches, wx, wy, large_press, ktime_to_us(ktime_sub(ktime_get(),kt)), ktime_to_us(kt), report->flags, buf);
		/*big data collect begin*/
		vts_click_frequency_collect(TOUCH_VCODE_CLICK_EVENT);
		if (vtsdev->nr_touches >= 5) {
			vts_five_more_time(TOUCH_VCODE_FIVE_MORE_TOUCH, vtsdev->nr_touches);
		}
		nr_touchs_changed(vtsdev, POINT_DOWN, nr_touches);
		if (wx <= 7) {
			vts_press_area(TOUCH_VCODE_PRESS_AREA, wx - 1);
		} else if (wx == 8 || wx ==9) {
			vts_press_area(TOUCH_VCODE_PRESS_AREA, AREA_8_9);
		} else if (wx >= 10 && wx <= 12) {
			vts_press_area(TOUCH_VCODE_PRESS_AREA, AREA_10_12);
		} else if (wx >= 13) {
			vts_press_area(TOUCH_VCODE_PRESS_AREA, AREA_13_MORE);
		}
		if ((point_info.physical_x >= 0 && point_info.physical_x < reject_pixel) ||
			(point_info.physical_x > (resolution_x - reject_pixel) && point_info.physical_x < resolution_x)) {
			for (i = 0; i < 10; i++) {
				if (vtsdev->point_x[i] >= reject_pixel && vtsdev->point_x[i] <= resolution_x - reject_pixel) {
					vts_edge_reject_fail(TOUCH_VCODE_EDGE_REJECT_FAIL);
					break;
				}
			}
		}
		/*big data collect end*/
	} else
		vts_dev_dbg(vtsdev, "[%d %04d %04d][%04d %04d]down[%d][%d][%d][%d][%lld %lld]%s\n", touch_id, x, y, point_info.physical_x, point_info.physical_y, vtsdev->nr_touches, wx, wy, large_press, ktime_to_us(ktime_sub(ktime_get(),kt)), ktime_to_us(kt), buf);

	if (report->on_aoi_down && vtsdev->aoi_notify_enabled && vtsdev->aoi_report_enabled) {
		if (vtsdev->aoi_report_type == 0) {
			aoi_x = x;
			aoi_y = y;
			aoi_id = touch_id;
		} else if (vtsdev->aoi_report_type == 1) {
			aoi_x = vtsdev->aoi_real_point[0];
			aoi_y = vtsdev->aoi_real_point[1];
			aoi_id = 0;
		} else if (vtsdev->aoi_report_type == 2) {
			aoi_x = vtsdev->aoi_area_point[0];
			aoi_y = vtsdev->aoi_area_point[1];
			aoi_id = 0;
		}

		if (resolution_adjust) {
			aoi_x = aoi_x * display_x / dimention_x;
			aoi_y = aoi_y * display_y / dimention_y;
		}

		ret = report->on_aoi_down(vtsdev, aoi_x, aoi_y, aoi_id);
	}

	if (report->on_point_down) {
		ret = report->on_point_down(report, touch_id, vtsdev->nr_touches, large_press, &point_info);
	}

	if (ret) {
		vts_dev_info(vtsdev, "down point was ignored![%d][%d][%d][%d][%d][%d][%d]\n", touch_id, vtsdev->nr_touches, x, y, wx, wy, large_press);
		return 0;
	}

	vts_event3(VTS_EVENT_TOUCH_DOWN, touch_id, x, y);
	if (report->touchbit == 0)
		vts_event3(VTS_EVENT_FINGER_FIRST_DOWN, touch_id, x, y);

	mutex_lock(&report->lock);
	ret = vts_report_add_point_down(&vtsdev->report, touch_id, vtsdev->nr_touches, x, y, wx, wy, kt);
	tp_point_switch_do_callback(&point_info, 1);
	set_bit(touch_id, &report->touchbit);
	mutex_unlock(&report->lock);
	return ret;
}

int vts_report_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, int x, int y, int wx, int wy, bool large_press, ktime_t kt)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_point point_info;
	int ret = 0;	
	int display_x, display_y, dimention_x, dimention_y;
	int aoi_x = 0;
	int aoi_y = 0;
	int aoi_id = 0;
	int resolution_adjust = 0;

	point_info.x = x;
	point_info.y = y;
	point_info.wx = wx;
	point_info.touch_id = touch_id;
	point_info.nr_touches = nr_touches;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution_adjust);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);

	if (display_x && display_y) {
		if (vtsdev->fw_x == display_x && vtsdev->fw_y == display_y) {
			x = x * dimention_x / vtsdev->fw_x;
			y = y * dimention_y / vtsdev->fw_y;
		}
		point_info.physical_x = x * display_x / dimention_x;
		point_info.physical_y = y * display_y / dimention_y;
	} else {
		point_info.physical_x = x;
		point_info.physical_y = y;
	}

	vtsdev->point_x[touch_id] = 0;

	if (report->on_aoi_up && vtsdev->aoi_notify_enabled && vtsdev->aoi_report_enabled) {
		if (vtsdev->aoi_report_type == 0) {
			aoi_x = x;
			aoi_y = y;
			aoi_id = touch_id;
		} else if (vtsdev->aoi_report_type == 1) {
			aoi_x = vtsdev->aoi_real_point[0];
			aoi_y = vtsdev->aoi_real_point[1];
			aoi_id = 0;
		} else if (vtsdev->aoi_report_type == 2) {
			aoi_x = vtsdev->aoi_area_point[0];
			aoi_y = vtsdev->aoi_area_point[1];
			aoi_id = 0;
		}

		if (resolution_adjust) {
			aoi_x = aoi_x * display_x / dimention_x;
			aoi_y = aoi_y * display_y / dimention_y;
		}

		ret = report->on_aoi_up(vtsdev, aoi_x, aoi_y, aoi_id);
	}

	if (report->on_point_up)
		ret = report->on_point_up(report, touch_id, nr_touches, large_press, &point_info);

	if (ret)
		return 0;

	vtsdev->nr_touches--;
	if (vtsdev->nr_touches < 0) {
		vtsdev->nr_touches = 0;
	}
	nr_touchs_changed(vtsdev, POINT_UP, nr_touches);
	vtsdev->int_cost_collect_flag = true;

	vts_dev_info(vtsdev, "[%d %04d %04d][%04d %04d]up[%d][%d][%d][%lld]\n", touch_id, x, y, point_info.physical_x, point_info.physical_y, vtsdev->nr_touches, wx, wy, ktime_to_us(kt));
	if(!test_bit(touch_id, &report->touchbit)) {
		vts_dev_info(vtsdev, "unexpected point up[%d %04d %04d][%04d %04d]up\n", touch_id, x, y, point_info.physical_x, point_info.physical_y);
	}

	mutex_lock(&report->lock);
	ret = vts_report_add_point_up(&vtsdev->report, touch_id, vtsdev->nr_touches, x, y, wx, wy, kt);
	tp_point_switch_do_callback(&point_info, 0);
	clear_bit(touch_id, &report->touchbit);
	mutex_unlock(&report->lock);

	vts_event3(VTS_EVENT_TOUCH_UP, touch_id, x, y);
	if (report->touchbit == 0)
		vts_event3(VTS_EVENT_FINGER_ALL_UP, touch_id, x, y);

	return ret;
}

int vts_report_release_aoi_point(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	int i;
	
	for (i = 0 ; vtsdev->finger_pressed > 0 && i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		if (test_bit(i, &vtsdev->finger_pressed)) {
			if (report->on_aoi_up) {
				if (vtsdev->aoi_report_type == 0)
					report->on_aoi_up(vtsdev, 0, 0, i);
				else
					report->on_aoi_up(vtsdev, 0, 0, 0);
			}
		}
	}

	return 0;
}
int vts_report_release(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_report_event *key;
	int i;
	ktime_t kt = ktime_get();

	/* if long_press_report timer is running, cancel it when point release */
	if (vtsdev->long_press_report.work_status == 1) {
		cancel_work_sync(&vtsdev->long_press_report.long_press_work);
		vtsdev->long_press_report.work_status = 0;
	}
	hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);

	while(!list_empty(&report->keys)) {
		key = list_first_entry(&report->keys,struct vts_report_event, list);
		vts_report_key_up(vtsdev, key->keycode);
	}

	memset(vtsdev->point_x, 0, sizeof(vtsdev->point_x));
	vts_report_release_aoi_point(vtsdev);

	if (!report->slotbit)
		return 0;

	mutex_lock(&report->lock);
	for (i = 0 ; i < sizeof(report->slotbit) * 8; i++) {
		if (test_bit(i, &report->slotbit)) {
			vts_dev_info(vtsdev, "up[%d]\n", i);
			vts_input_mt_slot(vtsdev->idev, i, kt);
			vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 0, kt);
			clear_bit(i, &report->slotbit);
			vts_event3(VTS_EVENT_TOUCH_UP, i, 0, 0);
			if (!report->slotbit) {
				vts_event3(VTS_EVENT_FINGER_ALL_UP, i, 0, 0);
				break;
			}
		}
	}

	report->touchbit = 0;
	input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 0);
	input_report_key(vtsdev->idev, BTN_TOUCH, 0);
	input_sync(vtsdev->idev);
	vtsdev->nr_touches = 0;
	mutex_unlock(&report->lock);
	return 0;
}

int vts_report_ic_status(struct vts_device *vtsdev, int status)
{
	vts_event(VTS_EVENT_IC_STATUS, status);
	return 0;
}

int vts_report_set_flags(struct vts_device *vtsdev, unsigned long flags)
{
#if !defined(CONFIG_INPUT_TIMESTAMP) && (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	if(flags & FLAGS_REPORT_TIMESTAMP) {
		vts_dev_info(vtsdev, "not support input event with timestamp\n");
		return -EPERM;
	}
#endif
	vtsdev->report.flags = flags;
	return 0;
}

unsigned long vts_report_flags(struct vts_device *vtsdev)
{
	return vtsdev->report.flags;
}

struct inject_event {
	struct input_event events[16];
	u8 nr_events;
	ktime_t time;
};

struct simulator {
	struct vts_device *vtsdev;
	struct hrtimer hrtimer;
	struct completion finished;

	const u8 *buf;
	size_t size;
	size_t curr_pos;

	struct inject_event event;
};

/*
[    1279.476795] /dev/input/event6: 0003 0035 00000343
*/
static int vts_report_parse_line(u8 *line, size_t size, __kernel_ulong_t *sec, __kernel_ulong_t *nsec, u16 *type, u16 *code, u32 *value)
{
	char devicename[64];
	int count;

	memset(devicename, 0, sizeof(devicename));
	count = sscanf(line, "[%8ld.%09ld] %s %hx %hx %x", sec, nsec, devicename, type, code, value);
	return count == 6 ? 0 : -EINVAL;
}

static int vts_report_get_line(const u8 *buf, size_t buf_size, size_t pos, u8 *line, size_t line_size)
{
	size_t offset = pos;
	size_t length;

	while(offset < buf_size && buf[offset++] != '\n');
	length = offset - pos;

	if (!length)
		return -ENOSPC;

	if (length > line_size)
		return -ENOSPC;

	memcpy(line, &buf[pos], length);
	return length;
}

static bool vts_report_has_next_inject_event(struct simulator *s)
{
	return s->curr_pos < s->size;
}

static void vts_fill_event(struct input_event *e, __kernel_ulong_t sec, __kernel_ulong_t nsec, u16 type, u16 code, u32 value)
{
	e->type = type;
	e->code = code;
	e->value = value;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	e->__sec = sec;
	e->__usec = nsec/NSEC_PER_USEC;
#else
	e->time.tv_sec = sec;
	e->time.tv_usec = nsec/NSEC_PER_USEC;
#endif
}

static int vts_report_parse_next_inject_event(struct simulator *s)
{
	u8 line[128];
	struct timespec64 time = {0, 0};

	memset(&s->event, 0, sizeof(s->event));

	while (s->curr_pos < s->size) {
		int length;
		int ret;
		__kernel_ulong_t sec = 0;
		__kernel_ulong_t nsec = 0;
		u16 type = 0;
		u16 code = 0;
		u32 value = 0;

		memset(line, 0, sizeof(line));
		length = vts_report_get_line(s->buf, s->size, s->curr_pos, line, sizeof(line));
		if (length < 0)
			return length;

		ret = vts_report_parse_line(line, sizeof(line), &sec, &nsec, &type, &code, &value);
		if (ret)
			return ret;

		vts_fill_event(&s->event.events[s->event.nr_events], sec, nsec, type, code, value);
		if (time.tv_sec == 0 && time.tv_nsec == 0) {
			time.tv_sec = sec;
			time.tv_nsec = nsec;
			s->event.nr_events++;
			s->curr_pos += length;
			continue;
		}

		if (time.tv_sec != sec || time.tv_nsec!= nsec)
			break;

		s->event.nr_events++;
		s->curr_pos += length;
	}
	
	if (time.tv_sec == 0 && time.tv_nsec == 0)
		return -ENOSPC;

	s->event.time = timespec64_to_ktime(time);
	
	return 0;
}

static void vts_report_inejct_event(struct vts_device *vtsdev, struct inject_event *event, ktime_t kt)
{
	int i;

	for (i = 0; i < event->nr_events; i++) {
		struct input_event *ie = &event->events[i];
		vts_input_event(vtsdev->idev, ie->type, ie->code, ie->value, kt);
	}
}

static enum hrtimer_restart vts_report_simulator_hrtimer(struct hrtimer *timer)
{
	struct simulator *s = container_of(timer, struct simulator, hrtimer);
	ktime_t curr_kt = hrtimer_get_expires(timer);
	ktime_t event_kt = s->event.time;
	int ret;

	vts_report_inejct_event(s->vtsdev, &s->event, curr_kt);

	if(!vts_report_has_next_inject_event(s)) {
		complete(&s->finished);
		return HRTIMER_NORESTART;
	}

	ret = vts_report_parse_next_inject_event(s);
	if (ret) {
		complete(&s->finished);
		VTE("parse inject event failed!ret = %d\n", ret);
		return HRTIMER_NORESTART;
	}

	hrtimer_forward(timer, curr_kt, ktime_sub(s->event.time, event_kt));
	return HRTIMER_RESTART;
}

static int vts_report_simulator_start(struct simulator *s, struct vts_device *vtsdev, const u8 *buf, size_t size)
{
	int ret;

	s->vtsdev = vtsdev;
	s->buf = buf;
	s->size = size;
	s->curr_pos = 0;

	ret = vts_report_parse_next_inject_event(s);
	if (ret)
		return ret;

	init_completion(&s->finished);
	hrtimer_init(&s->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	s->hrtimer.function = vts_report_simulator_hrtimer;
	hrtimer_start(&s->hrtimer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int vts_report_simulator_wait_for_finish(struct simulator *s)
{
	wait_for_completion(&s->finished);
	return 0;
}

int vts_report_inject_points(struct vts_device *vtsdev, char *filepath)
{
	int ret;
	struct simulator s;
	unsigned long flags = vtsdev->report.flags;
	const struct firmware *fw = NULL;

	vts_dev_info(vtsdev,"path:%s\n", filepath);

	ret = request_firmware(&fw, filepath, vtsdev->dev);
	if (ret) {
		vts_dev_err(vtsdev, "request firmware %s failed! ret = %d\n", filepath, ret);
		return ret;
	}

	vtsdev->report.flags |= FLAGS_REPORT_TIMESTAMP;
	ret = vts_report_simulator_start(&s, vtsdev, fw->data, fw->size);
	if (ret) {
		release_firmware(fw);
		vtsdev->report.flags = flags;
		vts_dev_err(vtsdev, "simulator start error!, ret = %d\n", ret);
		return ret;
	}

	vts_report_simulator_wait_for_finish(&s);
	vtsdev->report.flags = flags;
	release_firmware(fw);
	return 0;
}

int vts_report_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg)
{
	switch(exception) {
		case VTS_EXCEPTION_ESD_ERR:
			vts_event(VTS_EVENT_ERR_ESD, arg);
			break;
		case VTS_EXCEPTION_WATCHDOG_ERR:
			vts_event(VTS_EVENT_ERR_WATCHDOG, arg);
			break;
		case VTS_EXCEPTION_CHECKSUM_ERR:
			vts_event(VTS_EVENT_ERR_CHECKSUM, arg);
			break;
		case VTS_EXCEPTION_I2C_ERR:
			vts_event(VTS_EVENT_ERR_I2C, arg);
			break;
		default:
			break;
	}

	return vts_notify_ic_exception(vtsdev, exception, arg);
}

int vts_report_point_sync(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;
	struct vts_report_event *point;
	ktime_t kt;
	int last_major, last_slot;

	mutex_lock(&report->lock);

	while(!list_empty(&report->down_points)) {
		point = list_first_entry(&report->down_points,struct vts_report_event, list);
		kt = point->kt;

		if (report->slotbit == 0) {
			vts_input_report_key(vtsdev->idev, BTN_TOUCH, 1, kt);
			vts_input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 1, kt);
		}

		set_bit(point->touch_id, &report->slotbit);
		vts_input_mt_slot(vtsdev->idev, point->touch_id, kt);
		vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 1, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_POSITION_X, point->x, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_POSITION_Y, point->y, kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MAJOR, max(point->wx, point->wy), kt);
		vts_input_report_abs(vtsdev->idev, ABS_MT_TOUCH_MINOR, min(point->wx, point->wy), kt);
		last_slot = point->touch_id;
		last_major = max(point->wx, point->wy);
		vts_report_remove_event(report, point);


		if (list_empty(&report->down_points)) {
			vts_input_sync(vtsdev->idev, kt);
			vtsdev->long_press_report.slot = last_slot;
			vtsdev->long_press_report.index = last_major;
			vtsdev->long_press_report.major = last_major;
			if (vts_state_get(vtsdev, VTS_STA_LCD))
				hrtimer_start(&vtsdev->long_press_report.long_press_timer, ms_to_ktime(12000), HRTIMER_MODE_REL);
		}
	}

	while (!list_empty(&report->up_points)) {
		point = list_first_entry(&report->up_points,struct vts_report_event, list);
		kt = point->kt;

		clear_bit(point->touch_id, &report->slotbit);
		vts_input_mt_slot(vtsdev->idev, point->touch_id, kt);
		vts_input_mt_report_slot_state(vtsdev->idev, MT_TOOL_FINGER, 0, kt);
		vts_report_remove_event(report, point);

		if (report->slotbit == 0) {
			vts_input_report_key(vtsdev->idev, BTN_TOOL_FINGER, 0, kt);
			vts_input_report_key(vtsdev->idev, BTN_TOUCH, 0, kt);
		}

		if (list_empty(&report->up_points)) {
			vts_input_sync(vtsdev->idev, kt);
			/* if long_press_report timer is running, cancel it when point up */
			if (vtsdev->long_press_report.work_status == 1) {
				cancel_work_sync(&vtsdev->long_press_report.long_press_work);
				vtsdev->long_press_report.work_status = 0;
			}
			hrtimer_cancel(&vtsdev->long_press_report.long_press_timer);
		}
	}

	mutex_unlock(&report->lock);
	return 0;
}

static int vts_report_input_init(struct vts_device *vtsdev)
{
	int ret;
	u32 x_max = 0;
	u32 y_max = 0;

	vtsdev->idev = input_allocate_device();
	if (!vtsdev->idev) {
		vts_dev_err(vtsdev, "no memory for alloc input device\n");
		ret = -ENOMEM;
		goto return_err;
	}

	vtsdev->idev->name = vts_name(vtsdev);
	vtsdev->idev->id.bustype = vtsdev->busType;;
	vtsdev->idev->dev.parent = NULL;
	vtsdev->idev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_mt_init_slots(vtsdev->idev, VIVO_TS_MAX_TOUCH_NUM, INPUT_MT_DIRECT);
	vtsdev->idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(BTN_TOOL_FINGER, vtsdev->idev->keybit);

	input_set_capability(vtsdev->idev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_WAKEUP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_WAKEUP_SWIPE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LEFT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_RIGHT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_O);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_W);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_E);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_M);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_C);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_F);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_A);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_V);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_H);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_MENU);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_BACK);
	input_set_capability(vtsdev->idev, EV_KEY, VTS_KEY_FINGER_GESTURE);
	input_set_capability(vtsdev->idev, EV_KEY, VTS_KEY_DOUBLE_FINGER_GESTURE);
	input_set_capability(vtsdev->idev, EV_KEY, VTS_KEY_FACE_GESTURE);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_CAMERA);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LONGPRESS_DOWN);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_LONGPRESS_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_VIRTUAL_WAKEUP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_SCREEN_CLOCK_WAKE_UP);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_INSIDE_SLIDE_LEFT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_INSIDE_SLIDE_RIGHT);
	input_set_capability(vtsdev->idev, EV_KEY, KEY_QUIT_ACTIVE_MODE);

	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &x_max);
	input_set_abs_params(vtsdev->idev, ABS_MT_POSITION_X, 0, x_max - 1, 0, 0);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &y_max);
	input_set_abs_params(vtsdev->idev, ABS_MT_POSITION_Y, 0, y_max - 1, 0, 0);
	input_set_abs_params(vtsdev->idev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(vtsdev->idev, ABS_MT_TOUCH_MINOR, 0, 31, 0, 0);
	input_mt_init_slots(vtsdev->idev, VIVO_TS_MAX_TOUCH_NUM, 0);
	input_set_drvdata(vtsdev->idev, vtsdev);

	ret = input_register_device(vtsdev->idev);
	if (ret) {
		vts_dev_err(vtsdev, "register input device failed. ret = %d\n", ret);
		goto free_device;
	}

	vtsdev->idev_fp = input_allocate_device();
	if (!vtsdev->idev_fp) {
		vts_dev_err(vtsdev, "no memory for alloc input device for fingerprint\n");
		goto unregister_device;
	}

	vtsdev->idev_fp->name = vts_name_fp(vtsdev);
	vtsdev->idev_fp->id.bustype = vtsdev->busType;;
	vtsdev->idev_fp->dev.parent = NULL;
	vtsdev->idev_fp->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(vtsdev->idev_fp, EV_KEY, VTS_KEY_FINGER_GESTURE);

	input_set_abs_params(vtsdev->idev_fp, ABS_MT_POSITION_X, 0, x_max - 1, 0, 0);
	input_set_abs_params(vtsdev->idev_fp, ABS_MT_POSITION_Y, 0, y_max - 1, 0, 0);
	input_set_abs_params(vtsdev->idev_fp, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(vtsdev->idev_fp, ABS_MT_TOUCH_MINOR, 0, 31, 0, 0);
	
	input_set_drvdata(vtsdev->idev_fp, vtsdev);
	ret = input_register_device(vtsdev->idev_fp);
	if (ret) {
		vts_dev_err(vtsdev, "register input device failed. ret = %d\n", ret);
		goto free_device_fp;
	}

	return 0;

free_device_fp:
	input_free_device(vtsdev->idev_fp);
	
unregister_device:
	input_unregister_device(vtsdev->idev);
	vtsdev->idev = NULL;
	
free_device:
	if (vtsdev->idev != NULL)
		input_free_device(vtsdev->idev);
	
return_err:
	return ret;

}

static void vts_report_input_deinit(struct vts_device *vtsdev)
{
	input_unregister_device(vtsdev->idev);
	vtsdev->idev = NULL;
	input_unregister_device(vtsdev->idev_fp);
	vtsdev->idev_fp = NULL;
	return ;
}

int vts_report_init(struct vts_device *vtsdev,
	bool (*on_event_down)(struct vts_report *report, enum vts_event event, int *keycode),
	bool (*on_event_up)(struct vts_report *report, enum vts_event event, int *keycode),
	bool (*on_point_down)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point),
	bool (*on_point_up)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point),
	int (*on_post_key)(struct vts_report *report, int keycode ,int value),
	int (*on_aoi_down)(struct vts_device *vtsdev, int x, int y, int touch_id),
	int (*on_aoi_up)(struct vts_device *vtsdev, int x, int y, int touch_id))
{
	struct vts_report *report = &vtsdev->report;
	int ret;
	u32 report_timestamp = 0;

	snprintf(report->km_name, sizeof(report->km_name), "vts_report_km%d", vtsdev->type);
	report->km = kmem_cache_create(report->km_name, sizeof(struct vts_report_event), 0, 0, NULL);
	if (!report->km) {
		vts_dev_err(vtsdev, "create mem cache failed!\n");
		return -ENOMEM;
	}

	ret = vts_report_input_init(vtsdev);
	if (ret) {
		vts_dev_err(vtsdev, "report input init error, ret = %d\n", ret);
		kmem_cache_destroy(report->km);
		report->km = NULL;
		return ret;
	}

	report->on_event_down = on_event_down;
	report->on_event_up = on_event_up;
	report->on_point_down = on_point_down;
	report->on_point_up = on_point_up;
	report->on_post_key = on_post_key;
	report->on_aoi_down = on_aoi_down;
	report->on_aoi_up = on_aoi_up;
	mutex_init(&report->lock);
	report->slotbit = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_REPORT_TIMESTAMP, &report_timestamp);
#if defined(CONFIG_INPUT_TIMESTAMP) || (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	if (report_timestamp)
		report->flags |= FLAGS_REPORT_TIMESTAMP;
#else
	report->flags &= ~FLAGS_REPORT_TIMESTAMP;
#endif
	INIT_LIST_HEAD(&report->down_points);
	INIT_LIST_HEAD(&report->up_points);
	INIT_LIST_HEAD(&report->keys);
	INIT_LIST_HEAD(&point_callbacks.list);
	return 0;
}

int vts_report_deinit(struct vts_device *vtsdev)
{
	struct vts_report *report = &vtsdev->report;

	mutex_destroy(&report->lock);
	vts_report_input_deinit(vtsdev);
	return 0;
}
