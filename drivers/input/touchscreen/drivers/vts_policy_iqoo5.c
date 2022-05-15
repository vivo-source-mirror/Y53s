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
#include "vts_policy.h"

struct vts_filter_area {
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int min_wx;
	int max_wx;
};

struct vts_point last_report_restrain[10];
static struct vts_filter_area area[] = {
	{1086, 1154, 2905, 3088, 62,  85},
	{800,  1440,  200,  600, 70, 255},
	{330,  626,  2918,  3022, 68, 108}
};

static bool vts_is_filter_area(struct vts_point *point)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(area); i++) {
		if (point->x < area[i].min_x || point->x > area[i].max_x)
			continue;

		if (point->y < area[i].min_y || point->y > area[i].max_y)
			continue;

		if (point->wx < area[i].min_wx || point->wx > area[i].max_wx)
			continue;

		return true;
	}

	return false;
}

static bool vts_policy_release_point_region(struct vts_device *vtsdev, struct vts_point *point)
{
	last_report_restrain[point->touch_id].state = 0;
	return 0;
}

static bool vts_policy_report_point_region(struct vts_device *vtsdev, struct vts_point *point)
{	
	int touchid = point->touch_id;

	if (last_report_restrain[touchid].state == 2) { 
		return true;
	} else if (vts_is_filter_area(point)) {
		if (last_report_restrain[touchid].state != 0) {
			if (last_report_restrain[touchid].x == point->x && last_report_restrain[touchid].y == point->y) {
				VTI("Point down in restrain region\n");

				return false;
			} else {
				last_report_restrain[touchid].state = 2;
				return true;
			}
		}
		memcpy (&last_report_restrain[touchid],  point, sizeof(struct vts_point));
		last_report_restrain[touchid].state = 1;
		VTI("Point down in restrain region\n");
		return false;
	}

	last_report_restrain[touchid].state = 2;
	return true;
 }


static bool vts_policy_on_res_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
{
	bool is_report;
	bool large = (vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev));

	vtsdev->finger3_mode = (nr_touches >= 3);

	if (large_press)
		vts_add_large_press_point(vtsdev, touch_id);
	else
		vts_remove_large_press_point(vtsdev, touch_id);
	
	
	is_report = vts_policy_report_point_region(vtsdev, point);
	
	if (!is_report) {
		VTI("%s  %d", __func__, is_report);
		return true;
	}

	if (!large && vtsdev->finger3_mode)
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGER3_MODE);

	if (!large && vts_is_large_press_mode(vtsdev))
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);

	return vts_is_large_press_mode(vtsdev);
}

static bool vts_policy_on_res_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
{
	bool large = (vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev));

	vts_policy_release_point_region(vtsdev, point);

	vtsdev->finger3_mode = (nr_touches >= 3);
	vts_remove_large_press_point(vtsdev, touch_id);

	if (nr_touches == 0)
		vts_clear_large_press_points(vtsdev);

	if (large && !(vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev)))
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);

	return vts_is_large_press_mode(vtsdev);
} 

struct vts_policy policy_report_restrain = {
	.name = "iqoo5",
	.expected_mode = vts_policy_expected_mode,
	.modechange_allow = vts_policy_modechange_allow,
	.on_event_down = vts_policy_on_event_down,
	.on_event_up = vts_policy_on_event_up,
	.on_point_down = vts_policy_on_res_point_down,
	.on_point_up = vts_policy_on_res_point_up,
	.on_state_changed = vts_policy_on_state_changed,
	.incell_proc = vts_incell_proc,
	.on_post_key = vts_policy_on_post_key,
	.get_expect_proc = vts_policy_get_expected_proc,
};

