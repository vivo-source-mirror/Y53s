#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include "vts_core.h"
#include "vts_policy.h"


#define MIN_Y				1800
#define SPECIAL_WX			1
#define MIN_WX				60
#define CHECK_TIMEOUT		250
#define GHOST_MAX_COUNT		5
#define UP					0
#define DOWN				1
#define MOVE				2

struct vts_ghost_point {
	int x;
	int y;
	int state;
};

enum vts_point_state {
	VTS_POINT_NO_EXISTED = 0,
	VTS_POINT_EXISTED,
	VTS_POINT_UNKNOWN
};

struct gpoint_data {
	struct delayed_work dwork;
	struct vts_device *vtsdev;
} local_policy_data_X60;


static struct wakeup_source *gpoint_wakelock;
/*
static DEFINE_MUTEX(vts_gpoint_lock);
static struct vts_ghost_point pre_point[10];
static int ghost_count;
static s64 ktime_start;
static atomic_t point_state = ATOMIC_INIT(VTS_POINT_NO_EXISTED);


static void vts_set_point_state(enum vts_point_state state)
{
	atomic_set(&point_state, state);
}

static bool vts_point_is_existed(void)
{
	if (atomic_read(&point_state) == VTS_POINT_EXISTED)
		return true;

	return false;
}

static void vts_policy_on_dump(struct vts_device *vtsdev)
{
	if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL) {
		if (!vts_point_is_existed()) {
			VTI("no touch this time screen on!");
			vts_call_ic_ops(vtsdev, set_auto_idle, 0);
			vts_call_ic_ops(vtsdev, dump, 0);
			vts_call_ic_ops(vtsdev, set_auto_idle, 1);
		}

		vts_set_point_state(VTS_POINT_NO_EXISTED);
	}
}
*/
static void vts_policy_regular_slience_dump_delay_work(struct work_struct *work)
{
	struct vts_device *vtsdev = local_policy_data_X60.vtsdev;
	int ret = 0;
	ret = vts_call_func_sync(vtsdev, int, vts_check_status_sliently); 
	if (ret) {
		VTE("fail to check status!\n");
	}
	queue_delayed_work(vtsdev->gpoint_wq, &local_policy_data_X60.dwork, msecs_to_jiffies(5000));
}

static int vts_policy_mode_exit_prepare(struct vts_device *vtsdev, enum vts_run_mode mode)
{
	int ret = 0;

	if (mode == VTS_ST_NORMAL) {
		ret = cancel_delayed_work(&local_policy_data_X60.dwork);  
		if (ret < 0) {
			VTE("fail to cancel work!\n");
		}
	}
	return ret;
}

static int vts_policy_mode_enter_after(struct vts_device *vtsdev, enum vts_run_mode mode)
{
	int ret = 0;

	if (mode == VTS_ST_NORMAL) {
		ret = vts_call_ic_ops(vtsdev, check_state_sliently);
		if (ret) {
			VTE("fail to check status!\n");
		}
		queue_delayed_work(vtsdev->gpoint_wq, &local_policy_data_X60.dwork, msecs_to_jiffies(5000));
	}
	return ret;
}
/*
static void vts_policy_gpoint_release(struct vts_device *vtsdev, struct vts_point *point)
{
	s64 ktime_end;
	u64 ktime_diff;
	int id;

	id = point->touch_id;
	mutex_lock(&vts_gpoint_lock);
	if (pre_point[id].x == point->x && pre_point[id].y == point->y) {
		ghost_count ++;
	}

	if (ghost_count > GHOST_MAX_COUNT) {
		ktime_end  = ktime_to_us(ktime_get());
		ktime_diff = (u64)(ktime_end - ktime_start);
		do_div(ktime_diff, 1000);
		if (ktime_diff < CHECK_TIMEOUT) {
			if (work_pending(&vtsdev->gpoint_work)) {
				ghost_count = 0;
				mutex_unlock(&vts_gpoint_lock);
				return;
			}
			queue_work(vtsdev->gpoint_wq, &vtsdev->gpoint_work);
			VTI("found ghost point!");
		} else {
			VTI("found ghost point, timeout!");
		}
		ghost_count = 0;
	}

	pre_point[id].state = UP;
	mutex_unlock(&vts_gpoint_lock);
}

static void vts_policy_gpoint_check(struct vts_device *vtsdev, struct vts_point *point)
{

	int id;

	id = point->touch_id;
	mutex_lock(&vts_gpoint_lock);
	if (pre_point[id].state == MOVE) {
		mutex_unlock(&vts_gpoint_lock);
		return;
	}

	if (point->y >= MIN_Y) {
		if (point->wx == SPECIAL_WX || point->wx >= MIN_WX) {
			if (ghost_count == 0) {
				ktime_start = ktime_to_us(ktime_get());
			}

			if (pre_point[id].state == UP) {
				pre_point[id].x = point->x;
				pre_point[id].y = point->y;
				pre_point[id].state = DOWN;
			} else if (pre_point[id].state == DOWN) {
				if (pre_point[id].x != point->x || pre_point[id].y != point->y) {
					pre_point[id].state = MOVE;
				}
			}
		}
	}
	mutex_unlock(&vts_gpoint_lock);
}

static bool vts_policy_on_gpoint_down(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
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

	if (!vts_is_large_press_mode(vtsdev))
		vts_policy_gpoint_check(vtsdev, point);

	if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL)
		vts_set_point_state(VTS_POINT_EXISTED);

	return vts_is_large_press_mode(vtsdev);
}

static bool vts_policy_on_gpoint_up(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point)
{
	bool large = (vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev));

	vtsdev->finger3_mode = (nr_touches >= 3);
	vts_remove_large_press_point(vtsdev, touch_id);

	if (nr_touches == 0)
		vts_clear_large_press_points(vtsdev);

	if (large && !(vtsdev->finger3_mode || vts_is_large_press_mode(vtsdev)))
		vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);

	if (!vts_is_large_press_mode(vtsdev))
		vts_policy_gpoint_release(vtsdev, point);

	return vts_is_large_press_mode(vtsdev);
}

static void vts_policy_gpoint_work(struct work_struct *work)
{
	int ret;
	s64 ktime_start;
	s64 ktime_end;
	u64 ktime_diff;
	struct vts_device *vtsdev = container_of(work, struct vts_device, gpoint_work);

	ktime_start = ktime_to_us(ktime_get());
	__pm_stay_awake(gpoint_wakelock);
	do {
		ret = vts_call_func_sync(vtsdev, int, vts_dump);
		if (ret) {
			VTE("fail to dump fw data!\n");
			__pm_relax(gpoint_wakelock);
			return;
		}

		ktime_end  = ktime_to_us(ktime_get());
		ktime_diff = (u64)(ktime_end - ktime_start);
		do_div(ktime_diff, 1000);
	} while (ktime_diff < 1500);
	__pm_relax(gpoint_wakelock);
}
*/
static int vts_policy_gpoint_init(struct vts_device *vtsdev)
{
	vtsdev->gpoint_wq = alloc_workqueue("vts_gpoint_wq", WQ_HIGHPRI, 0);
	if (!vtsdev->gpoint_wq) {
		vts_dev_info(vtsdev, "create vts_gpoint_wq fail");
		return -EFAULT;
	}

	gpoint_wakelock = vts_wakelock_register(vtsdev, "vts_gpoint");
	if (!gpoint_wakelock) {
		vts_dev_err(vtsdev, "ghost point wakelock init fail!\n");
		destroy_workqueue(vtsdev->gpoint_wq);
		return -ENOMEM;
	}
/*
	INIT_WORK(&vtsdev->gpoint_work, vts_policy_gpoint_work);
*/
	local_policy_data_X60.vtsdev = vtsdev;
	INIT_DELAYED_WORK(&local_policy_data_X60.dwork, vts_policy_regular_slience_dump_delay_work);
	return 0;
}

enum vts_expect_proc vts_policy_get_expected_proc_x60(struct vts_device *vtsdev, bool mode_change)
{
	
	u32 save_power_in_game = 0;
	int game_mode_state = vts_state_get(vtsdev, VTS_STA_GAME_MODE);
	int high_rate_state = vts_state_get(vtsdev, VTS_STA_GAME_HIGH_RATE);
	enum vts_expect_proc ret = VTS_NULL_PROC;
	static enum vts_expect_proc last_proc = VTS_NULL_PROC;
	vts_property_get(vtsdev, VTS_PROPERTY_GAME_SAVE_PWOER_PROC, &save_power_in_game);
	
	if(0 == save_power_in_game){
		if (game_mode_state)
			ret = VTS_TRADITIONAL_GAME_PROC;
		else
			ret = VTS_TRADITIONAL_NORMAL_PROC;
			
	}else {
			if(high_rate_state)
			   ret = VTS_TRADITIONAL_GAME_PROC;
			else
			   ret = VTS_TRADITIONAL_NORMAL_PROC;
	}
	if(last_proc != ret || mode_change){
		last_proc = ret;
		return ret;
	}
	else{ 
		return VTS_NULL_PROC;
	}

}



struct vts_policy policy_game_proc_x60 = {
		.name = "x60",
		.init = vts_policy_gpoint_init,
		.expected_mode = vts_policy_expected_mode,
		.modechange_allow = vts_policy_modechange_allow,
		.on_event_down = vts_policy_on_event_down,
		.on_event_up = vts_policy_on_event_up,
		.on_point_down = vts_policy_on_point_down,
		.on_point_up = vts_policy_on_point_up,
		.on_state_changed = vts_policy_on_state_changed,
		.incell_proc = vts_incell_proc,
		.on_post_key = vts_policy_on_post_key,
		.mode_exit_prepare = vts_policy_mode_exit_prepare,
		.mode_enter_after = vts_policy_mode_enter_after,
		.get_expect_proc = vts_policy_get_expected_proc_x60,


};

