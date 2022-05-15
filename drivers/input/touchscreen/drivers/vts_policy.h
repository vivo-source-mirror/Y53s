#ifndef __VTS_POLICY_H__
#define __VTS_POLICY_H__

#include <linux/ctype.h>
#include "vts_core.h"

bool vts_policy_on_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
bool vts_policy_on_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
enum vts_run_mode vts_policy_expected_mode(struct vts_device *vtsdev);
bool vts_policy_modechange_allow(struct vts_device *vtsdev, enum vts_run_mode old, enum vts_run_mode new, enum vts_state reason);
bool vts_policy_on_event_down(struct vts_device *vtsdev, enum vts_event event, int *keycode);
bool vts_policy_on_event_up(struct vts_device *vtsdev,  enum vts_event event, int *keycode);
void vts_policy_on_state_changed(struct vts_device *vtsdev, enum vts_state state, int val);
void vts_incell_proc(struct vts_device *vtsdev, unsigned long event, int blank, int display_id);
int vts_policy_on_post_key(struct vts_report *report, int keycode ,int value);
int vts_policy_on_aoi_down(struct vts_device *vtsdev, int x, int y, int touch_id);
int vts_policy_on_aoi_up(struct vts_device *vtsdev, int x, int y, int touch_id);
enum vts_expect_proc vts_policy_get_expected_proc(struct vts_device *vtsdev, bool mode_change);

static inline void vts_clear_large_press_points(struct vts_device *vtsdev)
{
	if (vtsdev->large_touches != 0)
		vts_report_release(vtsdev);

	vtsdev->large_touches = 0;
}

static inline int vts_add_large_press_point(struct vts_device *vtsdev, int touch_id)
{
	int old;
	int new;

	if (touch_id >= (sizeof(vtsdev->large_touches) * 8)) {
		vts_dev_err(vtsdev, "error large press points id! touch_id = %d\n", touch_id);
		return -ENOMEM;
	}

	old = vtsdev->large_touches;
	set_bit(touch_id, &vtsdev->large_touches);
	new = vtsdev->large_touches;

	if (old == 0 && new != 0)
		vts_report_release(vtsdev);

	return 0;
}

static inline int vts_remove_large_press_point(struct vts_device *vtsdev, int touch_id)
{
	unsigned long old;
	unsigned long new;

	if (touch_id >= (sizeof(vtsdev->large_touches) * 8)) {
		vts_dev_err(vtsdev, "error large press points id! touch_id = %d\n", touch_id);
		return -ENOMEM;
	}

	old = vtsdev->large_touches;
	clear_bit(touch_id, &vtsdev->large_touches);
	new = vtsdev->large_touches;

	if (old != 0 && new == 0)
		vts_report_release(vtsdev);

	return 0;
}

static inline bool vts_is_large_press_mode(struct vts_device *vtsdev)
{
	return vtsdev->large_touches != 0;
}


#endif

