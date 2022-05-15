#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include "vts_core.h"
#include "vts_policy_common.h"
#include "vts_policy_iqoo5.h"
#include "vts_policy_gpoint.h"
#include "vts_policy.h"
#include "vts_policy_x60.h"

struct event_key_gesture_map {
	enum vts_event event;
	int keycode;
	char *event_name;
	u32 gesture;
};

#define KG_DEF(e,k,g) { \
		.event = e, \
		.gesture =g, \
		.keycode = k, \
		.event_name = #e \
		}

static const struct event_key_gesture_map kgmaps[] = {
	[VTS_EVENT_GESTURE_FINGERPRINT_DETECT] 		= KG_DEF(VTS_EVENT_GESTURE_FINGERPRINT_DETECT, VTS_KEY_FINGER_GESTURE, 0),
	[VTS_EVENT_GESTURE_FACE_DETECT]			= KG_DEF(VTS_EVENT_GESTURE_FACE_DETECT, VTS_KEY_FACE_GESTURE, 0),
	[VTS_EVENT_GESTURE_LARGE_AREA_PRESS]		= KG_DEF(VTS_EVENT_GESTURE_LARGE_AREA_PRESS, KEY_TS_LARGE_SUPPRESSION, 0),
	[VTS_EVENT_GESTURE_DOUBLE_CLICK]		= KG_DEF(VTS_EVENT_GESTURE_DOUBLE_CLICK, KEY_WAKEUP, VTS_GESTURE_DCLICK),
	[VTS_EVENT_GESTURE_TAKE_CAMERA]			= KG_DEF(VTS_EVENT_GESTURE_TAKE_CAMERA, KEY_CAMERA, 0),
	[VTS_EVENT_GESTURE_PATTERN_C]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_C, KEY_C, VTS_GESTURE_C),
	[VTS_EVENT_GESTURE_PATTERN_E]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_E, KEY_E, VTS_GESTURE_E),
	[VTS_EVENT_GESTURE_PATTERN_M]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_M, KEY_M, VTS_GESTURE_M),
	[VTS_EVENT_GESTURE_PATTERN_W]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_W, KEY_W, VTS_GESTURE_W),
	[VTS_EVENT_GESTURE_PATTERN_A]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_A, KEY_A, VTS_GESTURE_A),
	[VTS_EVENT_GESTURE_PATTERN_F]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_F, KEY_F, VTS_GESTURE_F),
	[VTS_EVENT_GESTURE_PATTERN_O]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_O, KEY_O, VTS_GESTURE_O),
	[VTS_EVENT_GESTURE_PATTERN_V]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_V, KEY_V, VTS_GESTURE_V),
	[VTS_EVENT_GESTURE_PATTERN_HEART]		= KG_DEF(VTS_EVENT_GESTURE_PATTERN_HEART, KEY_H, VTS_GESTURE_HEART),
	[VTS_EVENT_GESTURE_PATTERN_LEFT]		= KG_DEF(VTS_EVENT_GESTURE_PATTERN_LEFT, KEY_LEFT, VTS_GESTURE_LR),
	[VTS_EVENT_GESTURE_PATTERN_RIGHT]		= KG_DEF(VTS_EVENT_GESTURE_PATTERN_RIGHT, KEY_RIGHT, VTS_GESTURE_LR),
	[VTS_EVENT_GESTURE_PATTERN_UP]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_UP, KEY_UP, VTS_GESTURE_UP),
	[VTS_EVENT_GESTURE_PATTERN_DOWN]		= KG_DEF(VTS_EVENT_GESTURE_PATTERN_DOWN, KEY_WAKEUP_SWIPE, VTS_GESTURE_DOWN),
	[VTS_EVENT_GESTURE_PATTERN_SWAP]		= KG_DEF(VTS_EVENT_GESTURE_PATTERN_SWAP, KEY_TS_SWIPE, VTS_GESTURE_SHORT_DOWN_SWIPE),
	[VTS_EVENT_GESTURE_VK_LONG_PRESS]		= KG_DEF(VTS_EVENT_GESTURE_VK_LONG_PRESS, KEY_LONGPRESS_DOWN, 0),
	[VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE]	= KG_DEF(VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE, KEY_LONGPRESS_UP, 0),
	[VTS_EVENT_GESTURE_VK_DC]			= KG_DEF(VTS_EVENT_GESTURE_VK_DC, KEY_VIRTUAL_WAKEUP, 0),
	[VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK]		= KG_DEF(VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK, KEY_SCREEN_CLOCK_WAKE_UP, 0),
	[VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT]	= KG_DEF(VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT, KEY_INSIDE_SLIDE_LEFT, 0),
	[VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT]	= KG_DEF(VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT, KEY_INSIDE_SLIDE_RIGHT, 0),
	[VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE]		= KG_DEF(VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE, KEY_QUIT_ACTIVE_MODE, 0),
	[VTS_EVENT_GESTURE_PATTERN_H]			= KG_DEF(VTS_EVENT_GESTURE_PATTERN_H, KEY_H, VTS_GESTURE_HEART),
	[VTS_EVENT_GESTURE_FINGER3_MODE]		= KG_DEF(VTS_EVENT_GESTURE_LARGE_AREA_PRESS, KEY_TS_LARGE_SUPPRESSION, 0),
};

static bool vts_cover_mute_delay(struct vts_device *vtsdev)
{
	if (!(vts_state_get(vtsdev, VTS_STA_GESTURE) & VTS_GESTURE_COVER_MUTE))	
		return false;

	if (!vts_state_get(vtsdev, VTS_STA_CALLING))
		return false;

	if (vts_state_get(vtsdev, VTS_STA_IN_CALL))
		return false;

	if (vts_state_get(vtsdev, VTS_STA_PROX_STATE))
		return false;

	msleep(VTS_PALM_DELAY);
	return true;
}

int vts_policy_on_post_key(struct vts_report *report, int keycode ,int value)
{
	int ret = 0;
	return ret;
}

bool vts_policy_on_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
{
	bool large = (vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev));

	vtsdev->finger3_mode = (nr_touches >= 3);
	if (large_press)
		vts_add_large_press_point(vtsdev, touch_id);
	else
		vts_remove_large_press_point(vtsdev, touch_id);

	if (!large && vtsdev->finger3_mode)
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGER3_MODE);

	if (!large && vts_is_large_press_mode(vtsdev))
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);

	return vts_is_large_press_mode(vtsdev);
}

bool vts_policy_on_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
{
	bool large = (vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev));
	
	vtsdev->finger3_mode = (nr_touches >= 3);	
	vts_remove_large_press_point(vtsdev, touch_id);

	if (nr_touches == 0)
		vts_clear_large_press_points(vtsdev);

	if (large && !(vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev)))
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);

	return vts_is_large_press_mode(vtsdev);
}
static bool vts_should_report_clock_key(struct vts_device *vtsdev)
{
	struct vts_screen_clock_cmd  sclock_rect;
	vts_get_screen_clock_zone(&sclock_rect, &vtsdev->screen_clock_zone);

 if(( vtsdev->screen_clock_point.realX > sclock_rect.x
	&& vtsdev->screen_clock_point.realX < (sclock_rect.x + sclock_rect.width))
	&& (vtsdev->screen_clock_point.realY > sclock_rect.y 
	&& vtsdev->screen_clock_point.realY < (sclock_rect.y + sclock_rect.height))){
      mutex_lock(&(vtsdev->screen_clock_point.scrclMutex));
	  vtsdev->screen_clock_point.pointX = vtsdev->screen_clock_point.realX;
	  vtsdev->screen_clock_point.pointY = vtsdev->screen_clock_point.realY;
	  mutex_unlock(&(vtsdev->screen_clock_point.scrclMutex));  
	  return true;
   }
 return false;
}

bool vts_policy_on_event_down(struct vts_device *vtsdev, enum vts_event event, int *keycode)
{ 
    u32 screenclock = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_SCREEN_CLOCK, &screenclock);
	
	if (unlikely(event >= ARRAY_SIZE(kgmaps))) {
		vts_dev_err(vtsdev, "invalid event %d\n", event);
		return true;
	}

	*keycode = kgmaps[event].keycode;

	if (event == VTS_EVENT_GESTURE_FINGERPRINT_DETECT)
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FACE_DETECT);

	if (event == VTS_EVENT_GESTURE_DOUBLE_CLICK)
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_TAKE_CAMERA);

	if (event == VTS_EVENT_GESTURE_LARGE_AREA_PRESS && !vts_state_get(vtsdev, VTS_STA_GAME_MODE))
		return false;

	if (event == VTS_EVENT_GESTURE_FINGER3_MODE && vtsdev->finger3_mode && vts_state_get(vtsdev, VTS_STA_CALLING))
		return false;

	if (event == VTS_EVENT_GESTURE_FINGERPRINT_DETECT && vts_state_get(vtsdev, VTS_STA_FINGER_HIGHLIGHT))
		return false;

	if (event == VTS_EVENT_GESTURE_FACE_DETECT && vts_state_get(vtsdev, VTS_STA_FACE_HIGHLIGHT))
		return false;

	if (vts_state_get(vtsdev, VTS_STA_NOTICE_UP) && event == VTS_EVENT_GESTURE_PATTERN_UP)
		return false;

	if (vts_state_get(vtsdev, VTS_STA_FORCE_DOUBLE) && event == VTS_EVENT_GESTURE_TAKE_CAMERA)
		return false;
	
	if (vts_state_get(vtsdev, VTS_STA_VIRTUAL_KEY) && 
		(event == VTS_EVENT_GESTURE_VK_LONG_PRESS || event == VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE || event == VTS_EVENT_GESTURE_VK_DC
		 || event == VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT || event == VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT || event == VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE))
		return false;
	
	if (vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN) 
		&& screenclock && event == VTS_EVENT_GESTURE_DOUBLE_CLICK && vts_should_report_clock_key(vtsdev)){
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK);
		return true;
		}
	
    if(event == VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK)
   	   return false;
	
	if ((vts_state_get(vtsdev, VTS_STA_GESTURE) & kgmaps[event].gesture) && vts_state_get(vtsdev, VTS_STA_PROX_STATE) == 1)
		return false;

	return true;
}

bool vts_policy_on_event_up(struct vts_device *vtsdev,  enum vts_event event, int *keycode)
{
	u32 screenclock = 0;
	vts_property_get(vtsdev, VTS_PROPERTY_SCREEN_CLOCK, &screenclock);

	if (unlikely(event >= ARRAY_SIZE(kgmaps))) {
		vts_dev_err(vtsdev, "invalid event %d\n", event);
		return true;
	}

	*keycode = kgmaps[event].keycode;

	if (event == VTS_EVENT_GESTURE_FINGERPRINT_DETECT)
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FACE_DETECT);

	if (event == VTS_EVENT_GESTURE_DOUBLE_CLICK)
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_TAKE_CAMERA);
	
	if (vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN) 
		&& screenclock && event == VTS_EVENT_GESTURE_DOUBLE_CLICK && vts_should_report_clock_key(vtsdev)){
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK);
		return true;
	}

	return false;
}
enum vts_expect_proc vts_policy_get_expected_proc(struct vts_device *vtsdev, bool mode_change)
{
	
	u32 save_power_in_game = 0;
	enum vts_expect_proc ret = VTS_NULL_PROC;
	static enum vts_expect_proc last_proc_state = VTS_NULL_PROC;
	int game_mode_state = vts_state_get(vtsdev, VTS_STA_GAME_MODE);
	int high_rate_state = vts_state_get(vtsdev, VTS_STA_GAME_HIGH_RATE);
	int game_battle_state = vts_state_get(vtsdev, VTS_STA_GAME_BATTLE);
	
	vts_property_get(vtsdev, VTS_PROPERTY_GAME_SAVE_PWOER_PROC, &save_power_in_game);//not all of game switch to high report rate 
	
	if (0 == save_power_in_game) {
		if (game_mode_state)
			ret = VTS_TRADITIONAL_GAME_PROC;
		else
			ret = VTS_TRADITIONAL_NORMAL_PROC;
			
	}else {
			if (high_rate_state && game_battle_state)
				ret = VTS_HIGH_RATE_10S_IDLE_PROC;
			else if (high_rate_state || game_mode_state)
				ret = VTS_NORMAL_RATE_2S_IDLE_PROC;
			else 
				ret = VTS_NORMAL_RATE_1S_IDLE_PROC;
	}
	if (last_proc_state != ret || mode_change) {
		last_proc_state = ret;
		return ret;
	}
	else { 
		return VTS_NULL_PROC;
	}
	

}

enum vts_run_mode vts_policy_expected_mode(struct vts_device *vtsdev)
{
	u32 tddi = 0;
	u32 screenclock = 0;
	u32 vk = 0;
	u32 screenclock_hight = 0;
	vts_state_dump(vtsdev);
	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);
	vts_property_get(vtsdev, VTS_PROPERTY_SCREEN_CLOCK, &screenclock);
	vts_property_get(vtsdev, VTS_PROPERTY_VIRTUAL_KEY, &vk);
	vts_property_get(vtsdev, VTS_PROPERTY_SCREEN_CLOCK_HILIGHT, &screenclock_hight);

	if (vts_state_get(vtsdev, VTS_STA_FORCE_NORMAL))
		return VTS_ST_NORMAL;

	if (vts_state_get(vtsdev, VTS_STA_VIRTUAL_PROX_STATE))
		return VTS_ST_NORMAL;

	if (!tddi && vts_state_get(vtsdev, VTS_STA_LCD))
		return VTS_ST_NORMAL;

	if (tddi && vts_state_get(vtsdev, VTS_STA_TDDI_LCD))
		return VTS_ST_NORMAL;

	if (vts_state_get(vtsdev, VTS_STA_SAVE_POWER))
		return VTS_ST_SLEEP;

	if (vts_state_get(vtsdev, VTS_STA_FORCE_DOUBLE))
		return VTS_ST_GESTURE;

	if (vts_state_get(vtsdev, VTS_STA_NOTICE_UP))
		return VTS_ST_GESTURE;

	if (vts_state_get(vtsdev, VTS_STA_FINGER_UNLOCK_OPEN) && (vts_state_get(vtsdev, VTS_STA_PROX_STATE) == 1))
		return VTS_ST_GESTURE;

	if (vts_state_get(vtsdev, VTS_STA_FINGER_HIGHLIGHT))
		return VTS_ST_GESTURE;

	if (vts_state_get(vtsdev, VTS_STA_FACE_HIGHLIGHT))
		return VTS_ST_GESTURE;

	if (vts_state_get(vtsdev, VTS_STA_FP_AOI))
		return VTS_ST_GESTURE;

	if (vk && vts_state_get(vtsdev, VTS_STA_VIRTUAL_KEY) &&
		(vts_state_get(vtsdev, VTS_STA_IN_CALL) || (vts_state_get(vtsdev, VTS_STA_PROX_STATE) == 1)))
		return VTS_ST_GESTURE;

	if (!tddi && !vts_state_all_equals(VTS_STA_LCD, 0) && (vts_device_count_get() >= 2))
		return VTS_ST_SLEEP;

	if ((vts_state_get(vtsdev, VTS_STA_GESTURE) & (~(VTS_GESTURE_FG | VTS_GESTURE_FACE_GESTURE | VTS_GESTURE_COVER_MUTE))) && (vts_state_get(vtsdev, VTS_STA_PROX_STATE) == 1))
		return VTS_ST_GESTURE;
	
	if (screenclock_hight){// screen clock hilight state change mode
		if (screenclock && vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN) && vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_HIGHLIGHT))
			return VTS_ST_GESTURE;
	} else {
		if (screenclock && vts_state_get(vtsdev, VTS_STA_SCREEN_CLOCK_OPEN) && (vts_state_get(vtsdev, VTS_STA_PROX_STATE) == 1))
			return VTS_ST_GESTURE;
	}

	return VTS_ST_SLEEP;
}

void vts_policy_on_state_changed(struct vts_device *vtsdev, enum vts_state state, int val)
{
	u32 tddi, vprox = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);
	vts_property_get(vtsdev, VTS_PROPERTY_VIRTUAL_PROXIMINITY, &vprox);

	if ((state == VTS_STA_FINGER_HIGHLIGHT) && val)
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);

	if ((state == VTS_STA_FACE_HIGHLIGHT) && val)
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FACE_DETECT);

	if (state == VTS_STA_LCD)
		vts_state_broadcast(vtsdev, state, val);

	if (!tddi && !vprox && !vts_state_get(vtsdev, VTS_STA_LCD))
		vts_cover_mute_delay(vtsdev);

}

bool vts_policy_modechange_allow(struct vts_device *vtsdev, enum vts_run_mode old,
	enum vts_run_mode new, enum vts_state reason)
{
	u32 tddi;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);
	if (tddi && (reason == VTS_STA_DEFAUTL)) {
		vts_dev_info(vtsdev, "force mode change is not allowd on tddi panel!!\n");
		return false;
	}

	return true;
}

int vts_policy_on_aoi_down(struct vts_device *vtsdev, int x, int y, int touch_id)
{
	struct vts_touch_event *touch = NULL;
	struct vts_aoi_info *info = vtsdev->aoi_info;
	if ((x >= vtsdev->aoi_cmd.aoi_left && x < vtsdev->aoi_cmd.aoi_right)
		&& (y >= vtsdev->aoi_cmd.aoi_top && y < vtsdev->aoi_cmd.aoi_bottom)) {

		touch = &vtsdev->event[touch_id];
		touch->panel = vtsdev->type;
		touch->fid = touch_id;
		touch->type = EVENT_TYPE_DOWN;
		touch->x = x;
		touch->y = y;
		touch->inside = info->aoi_area_in;
		touch->outside = info->aoi_area_out;
		
		if (!test_bit(touch_id, &vtsdev->finger_pressed)) {
			__set_bit(touch_id, &vtsdev->finger_pressed);
		}
		__set_bit(touch_id, &vtsdev->event_mask);
		VTI("press down");
	} else {
		if (test_bit(touch_id, &vtsdev->finger_pressed)) {

			touch = &vtsdev->event[touch_id];
			touch->panel = vtsdev->type;
			touch->type = EVENT_TYPE_UP;
			touch->x = 0;
			touch->y = 0;
			touch->fid = touch_id;

			__clear_bit(touch_id, &vtsdev->finger_pressed);
			__set_bit(touch_id, &vtsdev->event_mask);
			VTI("press up");
		}
	}

	if (vtsdev->event_mask) {
		VTI("press type %d, call notifier panel %d    id %d   x %d    y %d", touch->type, touch->panel, touch->fid, touch->x, touch->y);
		vts_notifier_call_chain(vtsdev->event_mask, (void *)touch);
		vtsdev->event_mask = 0;
	}

	return 0;

}

int vts_policy_on_aoi_up(struct vts_device *vtsdev, int x, int y, int touch_id)
{
	struct vts_touch_event *touch = NULL;
	
	if (test_bit(touch_id, &vtsdev->finger_pressed)) {

		touch = &vtsdev->event[touch_id];
		touch->panel = vtsdev->type;
		touch->fid = touch_id;
		touch->type = EVENT_TYPE_UP;
		touch->x = 0;
		touch->y = 0;
		touch->inside = 0;
		touch->outside = 0;
	
		__clear_bit(touch_id, &vtsdev->finger_pressed);
		__set_bit(touch_id, &vtsdev->event_mask);
		VTI("press up");
	}

	if (vtsdev->event_mask) {
		VTI("press up, call notifier panel %d    id %d   x %d    y %d", touch->panel, touch->fid, touch->x, touch->y);
		vts_notifier_call_chain(vtsdev->event_mask, (void *)touch);
		vtsdev->event_mask = 0;
	}

	return 0;
}

void vts_incell_proc(struct vts_device *vtsdev, unsigned long event, int blank, int display_id)
{
	if (event == VTS_EARLY_EVENT_BLANK && blank == VTS_BLANK_POWERDOWN)
		vts_cover_mute_delay(vtsdev);
}

int vts_policy_init(struct vts_device *vtsdev)
{
	int i = 0;
	struct vts_policy *policy_array[] = {
		&policy_report_restrain,
		&policy_gpoint_dump,
		&policy_default,
		&policy_game_proc_x60,
	};

	vtsdev->policy = &policy_default;
	for (i = 0; i < ARRAY_SIZE(policy_array); i++) {
		if(policy_array[i]->name != NULL) {
			if (strcmp (policy_array[i]->name, vtsdev->module->policy_name) == 0) {
				vtsdev->policy = policy_array[i];
				break;
			}
		}
	}
	VTI("policy name is %s", vtsdev->policy->name);

	if (vtsdev->policy->init)
		vtsdev->policy->init(vtsdev);

	return 0;
}

void vts_policy_deinit(struct vts_device *vtsdev)
{
	vtsdev->policy = NULL;
}
