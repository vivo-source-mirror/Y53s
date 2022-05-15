#include <linux/kobject.h>
#include <linux/jiffies.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/vivo_touchscreen_virtual_key.h>
#include <linux/vivo_touchscreen_common.h>
static struct touchscreen_info *ts_info;
struct kobject *get_debug_kobject(void)
{
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return NULL;
	}
	return local_ts_info->kobject_debug;
}
void set_virtual_key_state(int key_state)
{
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info)	{
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return;
	}
	atomic_set(&local_ts_info->key_state, key_state);
}

int get_finger_when_key_down(void)
{
	int finger_when_key_down = 0;
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return 0;
	}
	finger_when_key_down = atomic_read(&local_ts_info->finger_when_key_down);
	atomic_set(&local_ts_info->finger_when_key_down, 0);		/*clear variable finger_when_key_down*/
	return finger_when_key_down;
}

bool get_finger_on_2d(void)
{
	bool result = false;
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return false;
	}
	result = atomic_read(&local_ts_info->finger_on_2d);
	return result;
}

void set_finger_on_2d(bool is_finger_on_2d)
{
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return;
	}
	atomic_set(&local_ts_info->finger_on_2d, is_finger_on_2d);
	if (atomic_read(&local_ts_info->key_state) && is_finger_on_2d) {
		atomic_set(&local_ts_info->finger_when_key_down, 1);
	}
}

long long  get_AA_release_time(void)
{
	struct touchscreen_info *local_ts_info = ts_info;
	long long tmp_time = 0;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return -EINVAL;
	}
	tmp_time = atomic64_read(&local_ts_info->AA_release_time);
	return tmp_time;
}

void set_AA_release_time(void)
{
	struct touchscreen_info *local_ts_info = ts_info;
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return ;
	}
	atomic64_set(&local_ts_info->AA_release_time, jiffies);
}

int ts_info_register(struct kobject *kobject_debug)
{
	struct touchscreen_info *local_ts_info = NULL;
	if (!kobject_debug) {
		VIVO_TS_LOG_ERR("%s: Debug kobject is NULL\n", __func__);
		return -EINVAL;
	}
	local_ts_info = kzalloc(sizeof(struct touchscreen_info), GFP_KERNEL);
	if (!local_ts_info) {
		VIVO_TS_LOG_ERR("%s: Failed to alloc mem for touchscreen infomation\n", __func__);
		return -ENOMEM;
	}
	ts_info = local_ts_info;
	local_ts_info->kobject_debug = kobject_debug;
	atomic_set(&local_ts_info->finger_on_2d, 0);
	atomic_set(&local_ts_info->finger_when_key_down, 0);
	atomic_set(&local_ts_info->key_state, 0);
	atomic64_set(&local_ts_info->AA_release_time, 0);
	return 0;
}

void ts_info_unregister(void)
{
	if (!ts_info) {
		VIVO_TS_LOG_ERR("%s: ts_info is NULL\n", __func__);
		return;
	}
	kfree(ts_info);
	ts_info = NULL;
	return ;
}