#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include "vts_core.h"

struct vts_node {
	/* touch driver manager */
	struct mutex lock;
	struct list_head tps;

	/* vts core code info */
	int code_version;

	/* sys/touchscreen node use */
	struct kobject kobj;
	int imei_read_mode;
	int tune_cmd;
	int cmd_code;
	int at_sensor_test_cmd;
};

#define TEST_NODE_CMD_MAX 10000
#define VTS_TNUE_DIF_2	95
#define VTS_TNUE_RAW_2	96
#define VTS_TNUE_DIF_1	97
#define VTS_TNUE_RAW_1	98
#define VTS_TNUE_BASE_INFO 99
#define USER_DEFINE_GESTURE_INFO 900
#define VTS_VERSION_INFO 100
#define VTS_ALL_STATE 101
#define VTS_DTS_INFO 102
#define VTS_IC_FUNCTION_STATE 103
#define VTS_KTHREAD_FUNC_STATE 104
#define VTS_CHARGER_TEST 105
#define VTS_TS_PIN_VALUE 107
#define VTS_TS_FW_MANAGER_SHOW 106
#define VTS_SET_IDLE_ENABLE 108
#define VTS_SET_IDLE_DISABLE 109
#define VTS_GET_LARGER_PRESS_COUNT 110
#define VTS_CLEAR_LARGER_PRESS_COUNT 111
#define VTS_GET_I2C_ERR_COUNT 112
#define VTS_CLEAR_I2C_ERR_COUNT 113
#define VTS_OTHER_INFO 114
#define VTS_TEST_COLLECT_BUG 115
#define VTS_GET_GRIP_STATUS 116

struct vts_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count);
};

static ssize_t vts_null_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}

static ssize_t vts_null_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	return -EINVAL;
}

static ssize_t vts_log_switch_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTE("invalide number of parameters passed\n");
		return -EINVAL;
	}

	VTI("FTS parameter is %d\n", val);
	if (val == 0) {
		vts_debug_enable(0);
	} else if (val == 1) {
		vts_debug_enable(1);
	} else{
		VTE("invalide parameter passed:%d\n", val);
		return -EINVAL;
	}

	return count;
}
static ssize_t vts_log_switch_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d\n", vts_is_debug());
}

static ssize_t vts_touchpanel_devices_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	int count = 0;
	int i = 0;
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	u32 ic_number;

	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		vts_property_get(vtsdev, VTS_PROPERTY_IC_NUMBER, &ic_number);
		count += snprintf(&buf[count], 255, "%s:%d ", touscreen_name(vtsdev), ic_number);
		i++;
	}
	mutex_unlock(&vtsc->lock);

	return count;
}

struct vts_cmd_handler {
	const char *tag;
	enum vts_state state_id;
};

const static struct vts_cmd_handler cmd_handlers[] = {
	{"vts_charge", VTS_STA_USB_CHARGE},
	{"vts_rotation", VTS_STA_ROTATION},
	{"vts_lcd", VTS_STA_LCD},
	{"vts_proximity", VTS_STA_PROX_STATE},
	{"vts_gs", VTS_STA_GESTURE},
	{"vts_finger_highlight", VTS_STA_FINGER_HIGHLIGHT},
	{"vts_face_highlight", VTS_STA_FACE_HIGHLIGHT},
	{"vts_save_power", VTS_STA_SAVE_POWER},
	{"vts_force_normal", VTS_STA_FORCE_NORMAL},
	{"vts_notice_up", VTS_STA_NOTICE_UP},
	{"vts_force_double", VTS_STA_FORCE_DOUBLE},
	{"vts_calling", VTS_STA_CALLING},
	{"vts_finger_unlock", VTS_STA_FINGER_UNLOCK_OPEN},
	{"vts_game_mode", VTS_STA_GAME_MODE},
	{"vts_virtual_key", VTS_STA_VIRTUAL_KEY},
	{"vts_vk_longpress", VTS_STA_VK_LONGPRESS},
	{"vts_vk_activemode", VTS_STA_VK_ACTIVEMODE},
	{"vts_band_state", VTS_STA_BAND_STATE},
	{"vts_in_call", VTS_STA_IN_CALL},
	{"vts_factory_switch", VTS_STA_FACTORY_SWITCH},
	{"vts_virtual_gamekey", VTS_STA_VIRTUAL_GAMEKEY},
	{"vts_screen_clock", VTS_STA_SCREEN_CLOCK_OPEN},
	{"vts_clock_report_abs", VTS_STA_SCREEN_CLOCK_REPORT_ABS},
	{"vts_force_disable", VTS_STA_FORCE_DISABLE},
	{"vts_finger_mode", VTS_STA_FINGER_MODE},
	{"vts_input_method",VTS_STA_INPUT_METHOD},
	{"vts-finger-center",VTS_STA_FINGER_CENTER},
	{"vts_finger_mode_support",VTS_STA_FINGER_MODE_SUPPORT},
	{"vts_gamemode_high_rate",VTS_STA_GAME_HIGH_RATE},
	{"vts_game_battle",VTS_STA_GAME_BATTLE},
};

static int vts_state_id_lookup(const char *tag, enum vts_state *state_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cmd_handlers); i++)
		if (!strcmp(cmd_handlers[i].tag, tag)) {
			*state_id =  cmd_handlers[i].state_id;
			return 0;
	}

	return -EINVAL;
}

static int vts_cmd_process(struct vts_device *vtsdev, enum vts_state state_id, int val)
{
	vts_state_set(vtsdev, state_id, val);
	return 0;
}

static bool match_name(struct vts_device *vtsdev, void *data)
{
	return strcmp(vts_name(vtsdev), (const char *)data) == 0 ? true : false;
}

static bool match_type(struct vts_device *vtsdev, void *data)
{
	u32 type = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &type);
	return type == (enum vts_type)data ? true : false;
}

static struct vts_device *vtsdev_lookup(struct vts_node *vtsc, bool (*match)(struct vts_device *vtsdev, void *data), void *data)
{
	struct vts_device *vtsdev;

	mutex_lock(&vtsc->lock);
	list_for_each_entry(vtsdev, &vtsc->tps, tp_list) {
		if (match(vtsdev, data)) {
			mutex_unlock(&vtsc->lock);
			return vtsdev;
		}
	}

	mutex_unlock(&vtsc->lock);
	return NULL;
}

static struct vts_device *vtsdev_lookup_by_type(struct vts_node *vtsc, enum vts_type type)
{
	return vtsdev_lookup(vtsc, match_type, (void *)type);
}

static struct vts_device *vtsdev_lookup_by_name(struct vts_node *vtsc, char *devname)
{
	return vtsdev_lookup(vtsc, match_name, devname);
}

static int vts_cmd_parse_dispatch(struct vts_node *vtsc, const char *buf)
{
	char *ptr = (char *)buf;
	char *tag;
	char *touch;
	char *state;
	int val;
	int type = 0;
	enum vts_state state_id;
	struct vts_device *vtsdev;

	tag = strsep(&ptr, ":");
	if (!tag) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}


	if (vts_state_id_lookup(tag, &state_id)) {
		VTI("invalid tag! tag:%s\n", tag);
		return -EINVAL;
	}

	touch = strsep(&ptr, ":");
	if (!touch) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	state = strsep(&ptr, ":");
	if (!state) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	if (kstrtoint(state, 10, &val)) {
		VTE("invalid command format!\n");
		return -EINVAL;
	}

	if (!strcmp(touch, "ALL")) {
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			if (vtsdev)
				vts_cmd_process(vtsdev, state_id, val);
		}
		return 0;
	}

	vtsdev = vtsdev_lookup_by_name(vtsc, touch);
	if (!vtsdev) {
		VTE("invalid touch name or no related touch was registered! touch name is %s\n", touch);
		return -EINVAL;
	}

	vts_cmd_process(vtsdev, state_id, val);
	return 0;
}

#define PARSE_INT(buf, val) do { \
		char *token; \
		\
		token = strsep(&buf, ","); \
		if (!token) \
			return ERR_PTR(-EINVAL); \
		\
		if(kstrtoint(token, 0, val)) \
			return ERR_PTR(-EINVAL); \
	} while (0)


static char* vts_get_edge_cmd(struct vts_edge_cmd *buf, char *cmd) {
	PARSE_INT(cmd, &(buf->x));
	PARSE_INT(cmd, &(buf->y));
	PARSE_INT(cmd, &(buf->width));
	PARSE_INT(cmd, &(buf->height));
	PARSE_INT(cmd, &(buf->area_type));
	PARSE_INT(cmd, (int*)&(buf->enable));
	PARSE_INT(cmd, &(buf->index));
	return cmd;
}

static char* vts_get_screen_clock_cmd(struct vts_screen_clock_cmd *buf, char *cmd) {
	PARSE_INT(cmd, &(buf->x));
	PARSE_INT(cmd, &(buf->y));
	PARSE_INT(cmd, &(buf->width));
	PARSE_INT(cmd, &(buf->height));
	return cmd;
}

static int vts_update_screen_clock_zone(struct vts_screen_clock_zone *dis, struct vts_screen_clock_cmd *src)
{
	 int ret = 0;
	 static int statcnt = 0;
	 if(dis ==NULL || src == NULL){
	 	VTE("NULL point !!");
	 	return -1;
	 }
	 mutex_lock(&dis->c_Mutex);
	 if(src->x != dis->startPoint.pointX || src->y != dis->startPoint.pointY ||
		src->width != dis->width || src->height != dis->height ){// screen clock appear
		statcnt++;
		if(statcnt < 0)
			statcnt = 1;
	 }
	 if(0 == src->width  && 0 == src->height)// remove screen clock 
	 	statcnt = 0;
	 
	 dis->startPoint.pointX = src->x;
	 dis->startPoint.pointY = src->y;
	 dis->width= src->width;
	 dis->height= src->height;
	 mutex_unlock(&dis->c_Mutex);
	 ret = statcnt;
	 return ret;
}

int vts_get_screen_clock_zone(struct vts_screen_clock_cmd *dis, struct vts_screen_clock_zone *src)
{
 	 int ret = 0;
	 if(dis ==NULL || src == NULL){
	 	VTE("NULL point !!");
	 	return -1;
	 }
	 mutex_lock(&src->c_Mutex);
	 dis->x = src->startPoint.pointX;
	 dis->y = src->startPoint.pointY;
	 dis->width = src->width ;
	 dis->height = src->height; 
	 mutex_unlock(&src->c_Mutex);
	 return ret;
	 
}

static ssize_t vts_service_cmd_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	int ret = 0;
	int type = 0;
	struct vts_device *vtsdev = NULL;
	struct vts_edge_cmd edge_cmd;
	char *tmp = (char *)buf;
	char *tag;
	unsigned char app_name[256];
	struct vts_screen_clock_cmd clock_cmd;
	u32 support,sclock_hilight = 0;
	VTI("buf:%s\n", buf);

	memset(app_name, '\0', 256);
	strcpy(app_name, (unsigned char*)buf);

	if(strstr(tmp, "bbk_screen_disable_card_slide_height_area")){
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			if (vtsdev) {
				tag = strsep(&tmp, ":");
				if (!tag) {
					VTE("tag is NULL\n");
					return -EINVAL;
				}

				tag = strsep(&tmp, ",");
				if (!tag) {
					VTE("tag for y0 is NULL\n");
					return -EINVAL;
				}

				if (kstrtoint(tag, 0, &vtsdev->y0)) {
					VTE("invalid command format!\n");
					return -EINVAL;
				}

				tag = strsep(&tmp, ",");
				if (!tag) {
					VTE("tag for y1 is NULL\n");
					vtsdev->y0 = 0;
					vtsdev->y1 = 0;
					return -EINVAL;
				}

				if (kstrtoint(tag, 0, &vtsdev->y1)) {
					VTE("invalid command format!\n");
					return -EINVAL;
				}
				vtsdev->y1 -= 1;
				return count;
			}
		}
	}

	if(strstr(tmp, "vts_edge_area_cmd")){
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			if (vtsdev) {
				tag = strsep(&tmp, ":");
				if (!tag) {
					VTE("tag is NULL\n");
					return -EINVAL;
				}
				if (IS_ERR(vts_get_edge_cmd(&edge_cmd, tmp))){
					VTE("get edge cmd error\n");
					return -EINVAL;
				}
				VTI("the edge_cmd is x:%d, y:%d, width:%d, height:%d, area_type:%d, enable:%d, index:%d\n",edge_cmd.x, edge_cmd.y, edge_cmd.width,
									edge_cmd.height, edge_cmd.area_type, edge_cmd.enable, edge_cmd.index);
				ret = vts_call_ic_ops_sync(vtsdev, set_edge_reject_area, &edge_cmd);
				if (ret) {
					VTE("call ic ops set_edge_reject_area failed, ret = %d\n", ret);
					return ret;
				}
				return count;
			}
		}
	}

	if(strstr(tmp, "vts_clock_area_cmd")){
		vtsdev = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
		if (vtsdev) {
			tag = strsep(&tmp, ":");
			if (!tag) {
				VTE("tag is NULL\n");
				return -EINVAL;
			}

			if (IS_ERR(vts_get_screen_clock_cmd(&clock_cmd, tmp))){
				VTE("get screen vts_screen_clock_area_cmd  error\n");
				return -EINVAL;
			}

			ret = vts_update_screen_clock_zone(&vtsdev->screen_clock_zone,&clock_cmd);
			if(ret < 0){
				VTE("call ic ops vts_update_screen_clock_zone failed, ret = %d\n", ret);
				return ret;
			}
		
				
			VTI("the screen cmd is x:%d, y:%d, width:%d, height:%d\n",clock_cmd.x, clock_cmd.y, clock_cmd.width,clock_cmd.height);
			vts_property_get(vtsdev, VTS_PROPERTY_SCREEN_CLOCK_HILIGHT, &sclock_hilight);
			if(sclock_hilight){// new screen clock 
																								// SCREEN CLOCK  hilight
					vts_state_set(vtsdev, VTS_STA_SCREEN_CLOCK_HIGHLIGHT, ret);
				
			}else{
				if(VTS_ST_SLEEP == vts_get_run_mode(vtsdev))
					return -EINVAL;
				
				if(vts_state_get(vtsdev,VTS_STA_SCREEN_CLOCK_REPORT_ABS)){// only enable is true ,send area to IC
					ret = vts_call_ic_ops_sync(vtsdev, set_screen_clock_report_abs, vtsdev->screen_clock_zone.enable);
					if (ret) {
						VTE("call ic ops set_edge_reject_area failed, ret = %d\n", ret);
						return ret;
					}
				}

				if (vts_state_get(vtsdev,VTS_STA_SCREEN_CLOCK_OPEN) && vtsdev->screen_clock_zone.enable == 0) {
					ret = vts_call_ic_ops_sync(vtsdev, set_screen_clock_area, 0);
					if (ret) {
						VTE("call ic ops set_screen_clock_area failed, ret = %d\n", ret);
						return ret;
					}
				}
			}

			return count;
		}
	}

	if(strstr(tmp, "vts_rejection_zone")){
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			vts_property_get(vtsdev, VTS_PROPERTY_EDGE_REJECTION, &support);
			if (support != 1) {
				VTI("no support edge setting\n");
				return -EINVAL;
			}
			if((ret = vts_rejection_zone_cmd_set(vtsdev, &tmp)) >= 0)
				return count;
			else
				return ret;
		}
	}
	if (strstr(tmp, "vts_rejection_version")) {
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			vts_property_get(vtsdev, VTS_PROPERTY_EDGE_REJECTION, &support);
			if (support != 1) {
				VTI("no support edge setting\n");
				return -EINVAL;
			}
			if((ret = vts_rejection_version_parse(vtsdev, &tmp)) >= 0)
				return count;
			else
				return ret;
		}
	}

	ret = vts_cmd_parse_dispatch(vtsc, buf);
	if (!ret)
		return count;

	if (strstr(buf, "vts_screenshot")) {
		for (type = 0; type <= VTS_TYPE_SECOND; type++) {
			vtsdev = vtsdev_lookup_by_type(vtsc, type);
			if (vtsdev && vts_is_debug()) {
				vts_call_func_sync(vtsdev, int, vts_frame_data_put, vts_frame_data_get(vtsdev, VTS_FRAME_MUTUAL_DELTA));
			}
		}
		return count;
	}

	for (type = 0; type <= VTS_TYPE_SECOND; type++) {
		vtsdev = vtsdev_lookup_by_type(vtsc, type);
		if (vtsdev && vts_get_run_mode(vtsdev) == VTS_ST_NORMAL)
			vts_call_ic_ops_sync(vtsdev,process_package, app_name);
	}

	return count;
}

static ssize_t vts_cali_support_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	int count = 0;
	struct vts_device *vtsdev_main = NULL;
	struct vts_device *vtsdev_sec = NULL;
	int state = 0;
	u32 type;
	u32 need_cali;

	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &type);
		if (type == VTS_TYPE_MAIN)
			vtsdev_main = vtsdev;		
		if (type == VTS_TYPE_SECOND)
			vtsdev_sec = vtsdev;
	}
	mutex_unlock(&vtsc->lock);

	if (vtsdev_main) {
		vts_property_get(vtsdev_main, VTS_PROPERTY_NEED_CALI, &need_cali);
		state |= need_cali;
	}
	if (vtsdev_sec) {
		vts_property_get(vtsdev_sec, VTS_PROPERTY_NEED_CALI, &need_cali);
		state |= (need_cali << 1);
	}
	
	count += snprintf(&buf[count], 128, "%d", state);

	return count;
}

static ssize_t vts_fwver_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	u64 version;
	int ret;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev;

	mutex_lock(&vtsc->lock);
	list_for_each_entry(vtsdev, &vtsc->tps, tp_list) {
		ret = vts_call_func_sync(vtsdev, int, vts_firmware_version_get, &version);
		if (ret) {
			vts_dev_err(vtsdev, "get firmware version error! ret = %dd\n", ret);
			mutex_unlock(&vtsc->lock);
			return ret;
		}
		count += snprintf(buf + strlen(buf), PAGE_SIZE, "%s:0x%llx ", touscreen_name(vtsdev), version);
	}
	mutex_unlock(&vtsc->lock);
	count += snprintf(buf + strlen(buf), PAGE_SIZE, "\n");
	return count;
}

static ssize_t vts_module_id_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;

	mutex_lock(&vtsc->lock);

	list_for_each_entry(vtsdev, &vtsc->tps, tp_list) {
		u32 vendor;
		vts_property_get(vtsdev, VTS_PROPERTY_VENDOR, &vendor);
		count += snprintf(&buf[count], PAGE_SIZE - count, "%s:0x%x ", touscreen_name(vtsdev), vendor);
	}

	mutex_unlock(&vtsc->lock);
	return count;

}

static ssize_t vts_imei_ctl_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	int i = 0;

	if (count <= 15) {
		VTE("Invalide number of parameters\n");
		return -EINVAL;
	}
	for (i = 0; i < 15; i++) {
		if (buf[i] != '0') {
			break;
		}
	}
	/* if 15 zero, enter into imei read mode */
	if (i == 15) {
		vtsc->imei_read_mode = 1;
		VTI("enter into imei_read_mode\n");
		return count;
	}

	/* if not 15 zero, write it into flash */
	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL)
			vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_write, VTS_ROM_ZONE_IMEI, (const u8 *)buf, count);
	}
	mutex_unlock(&vtsc->lock);

	return count;
}
static ssize_t vts_imei_ctl_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	int count = 0;
	u32 no_flash;
	u32 unique_code_new = 0;

	int ret = 0;
	int i = 0;
	unsigned char read_config_buf[15] = {0};

	if (vtsc->imei_read_mode == 0) {
		VTI("not in touchscreen data read mode\n");
		return snprintf(buf, 255, "not in ID read mode\n");
	}
	vtsc->imei_read_mode = 0;

	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL) {
			memset(read_config_buf, 0, 15);
			ret = vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_read, VTS_ROM_ZONE_IMEI, (u8 *)read_config_buf, sizeof(read_config_buf));
			if (ret !=  sizeof(read_config_buf)) {
				VTE("read imei fail or imei interface not define\n");
			if (ret == -EINVAL) {
					mutex_unlock(&vtsc->lock);
					return snprintf(buf, 255, "Touch not support ID\n");
			}
				count += snprintf(&buf[count], 255, "read ID fail\n");
			} else {
				/* is all 0x00 judge*/
				for (i = 0; i < 15; i++) {
					if (read_config_buf[i] == 0x00) {
						continue;
					} else {
						break;
					}
				}
				if (i == 15) {
					VTI("ID is no flashed,all is 0x00\n");
					count += snprintf(&buf[count], 255, "ID is no flashed\n");
					continue;
				}
				/* is all 0xff judge*/
				for (i = 0; i < 15; i++) {
					if (read_config_buf[i] == 0xff) {
						continue;
					} else {
						break;
					}
				}
				if (i == 15) {
					VTI("ID is no flashed,all is 0xff\n");
					count += snprintf(&buf[count], 255, "ID is no flashed\n");
					continue;
				}

				vts_property_get(vtsdev, VTS_PROPERTY_NO_FLASH, &no_flash);
				if (!no_flash) {
					for (i = 0; i < 15; i++) {
						if (read_config_buf[i] <= '9' && read_config_buf[i] >= '0') {
							continue;
						} else {
							break;
						}
					}

					if (i != 15) {
						VTI("ID is invalid,out of the range 0~9\n");
						count += snprintf(&buf[count], 255, "ID is invalid\n");
						continue;
					}
					count += snprintf(&buf[count], 255, "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n", read_config_buf[0]-0x30, read_config_buf[1]-0x30, read_config_buf[2]-0x30,
					read_config_buf[3]-0x30, read_config_buf[4]-0x30, read_config_buf[5]-0x30,
					read_config_buf[6]-0x30, read_config_buf[7]-0x30, read_config_buf[8]-0x30,
					read_config_buf[9]-0x30, read_config_buf[10]-0x30, read_config_buf[11]-0x30,
					read_config_buf[12]-0x30, read_config_buf[13]-0x30, read_config_buf[14]-0x30);
				} else {
					vts_property_get(vtsdev, VTS_PROPERTY_NO_FLASH_UNIQUE_CODE, &unique_code_new);
					if (unique_code_new != 0) {
						count += snprintf(&buf[count], 255, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", read_config_buf[0], read_config_buf[1], read_config_buf[2],
						read_config_buf[3], read_config_buf[4], read_config_buf[5],
						read_config_buf[6], read_config_buf[7], read_config_buf[8],
						read_config_buf[9], read_config_buf[10], read_config_buf[11],
						read_config_buf[12], read_config_buf[13], read_config_buf[14]);
					} else {
						count += snprintf(&buf[count], 255, "%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\n", read_config_buf[0], read_config_buf[1], read_config_buf[2],
						read_config_buf[3], read_config_buf[4], read_config_buf[5],
						read_config_buf[6], read_config_buf[7], read_config_buf[8],
						read_config_buf[9], read_config_buf[10], read_config_buf[11],
						read_config_buf[12], read_config_buf[13], read_config_buf[14]);
					}
				}
			}
		} else {
			VTI("not in VTS_ST_NORMAL mode,could not read imei\n");
			count += snprintf(&buf[count], 255, "ID is locked\n");
		}
	}
	mutex_unlock(&vtsc->lock);

	return count;
}

static ssize_t vts_trx_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	int touch_state_count = 0;
	int temp = 0;
	u32 tx_sensors;
	u32 rx_sensors;

	/* judge 2 lcd is booth normal, if booth, do nothing */
	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL) {
			touch_state_count++;
		}
	}
	mutex_unlock(&vtsc->lock);

	if (touch_state_count != 1) {
		VTI("0 or 2 lcm is light, touch_state_count:%d\n", touch_state_count);
		return snprintf(buf, 255, "fail! 0 or 2 lcm is light, touch_state_count:%d\n", touch_state_count);
	}

	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL) {
			vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, &tx_sensors);
			vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, &rx_sensors);
			if (rx_sensors >= tx_sensors) {
				temp = (rx_sensors << 8) | tx_sensors;
			} else {
				temp = (tx_sensors << 8) | rx_sensors;
			}
		}
	}
	mutex_unlock(&vtsc->lock);

	return snprintf(buf, 255, "%d\n", temp);
}

static ssize_t vts_gesture_point_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev;
	u16 x[32];
	u16 y[32];
	int points = 0;
	size_t len = 0;
	int i;

	memset(x, 0, sizeof(x));
	memset(y, 0, sizeof(y));
	
	mutex_lock(&vtsc->lock);
	list_for_each_entry(vtsdev, &vtsc->tps, tp_list) {
		if (vts_report_coordinates_get(vtsdev, x, y, ARRAY_SIZE(x), &points) == 0) {
			vts_dev_info(vtsdev, "points count: %d", points);
			for (i = 0; i < points; i++)
				len += sprintf(buf + len, "%d %d ", x[i], y[i]);
			mutex_unlock(&vtsc->lock);
			return len;
		}
	}
	mutex_unlock(&vtsc->lock);
	return 0;
}

static ssize_t vts_ts_super_node_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_grip_area grip_area = {0, 0};
	int type, ret, count;
	u8 buffer[256];
	type = ret = count = 0;
	switch (vtsc->cmd_code) {
		case VTS_GET_GRIP_STATUS:
			for (type = 0; type <= VTS_TYPE_SECOND; type++) {
				vtsdev = vtsdev_lookup_by_type(vtsc, type);
				ret = vts_call_ic_ops_sync(vtsdev, get_grip_status, &grip_area);
				if (ret < 0) {
					count += snprintf(&buf[count], 255, "%d ", -1);
				} else {
					count += snprintf(&buf[count], 255, "%d ", grip_area.area_center);
				}
			}
			break;
		case VTS_OTHER_INFO:
			for (type = 0; type <= VTS_TYPE_SECOND; type++) {
				vtsdev = vtsdev_lookup_by_type(vtsc, type);
				if (!vtsdev) {
					VTI("vtsdev no exist, type:%d\n", type);
					continue;
				}

				if (vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) {
					count += snprintf(&buf[count], PAGE_SIZE - count, "%s:not in normal\n", vts_name(vtsdev));
					continue;
				}

				memset(buffer, 0 ,sizeof(buffer));
				ret = vts_call_ic_ops_sync(vtsdev, otherInfo, buffer, sizeof(buffer));
				if (ret < 0)
					count += snprintf(&buf[count], PAGE_SIZE - count, "%s:get info error!\n", vts_name(vtsdev));
				else
					count += snprintf(&buf[count], PAGE_SIZE - count, "%s:%s\n", vts_name(vtsdev), buffer);
			}
			break;
		default:
			count = snprintf(buf, 255, "%s", "cmd not support");
			break;
	}

	vtsc->cmd_code = 0;	/*erase cmd, cts test will no effect*/
	return count;
}

static ssize_t vts_ts_super_node_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);

	if (sscanf(buf, "%d", &val) != 1) {
		VTE("invalide number of parameters passed\n");
		return -EINVAL;
	}

	VTI("input cmd is %d\n", val);
	if (val > TEST_NODE_CMD_MAX || val <= 0) {
		VTE("input cmd is out of range:%d\n", val);
		return count;
	}

	vtsc->cmd_code = val;

	return count;
}

static ssize_t vts_sensor_test(struct vts_node *vtsc, enum vts_type type, char *buf)
{
	struct vts_device *vtsdev;
	enum vts_sensor_test_result result = VTS_SENSOR_TEST_SUCCESS;
	int ret;

	vtsdev = vtsdev_lookup_by_type(vtsc, type);
	if (!vtsdev) {
		VTE("vtsdev is NULL, type = %d\n", type);
		return -EINVAL;
	}

	if(vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) {
		vts_dev_err(vtsdev, "not in normal mode, can not sensor test!\n");
		return -EINVAL;
	}

	ret = vts_call_ic_ops_sync(vtsdev, sensor_test, &result);
	vts_event(VTS_EVENT_SENSOR_TEST_FINISHED, (int)result);

	if (!ret && result == VTS_SENSOR_TEST_SUCCESS)
		return snprintf(buf, PAGE_SIZE, "Pass");
	else
		return snprintf(buf, PAGE_SIZE, "Failed");
}

static ssize_t vts_sensor_caliberate(struct vts_node *vtsc, int code, enum vts_type type, char *buf)
{
	struct vts_device *vtsdev;
	enum vts_sensor_cali_result result;
	int ret;

	vtsdev = vtsdev_lookup_by_type(vtsc, type);
	if (!vtsdev) {
		VTE("vtsdev is NULL, type = %d\n", type);
		return -EINVAL;
	}

	if(vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) {
		vts_dev_err(vtsdev, "not in normal mode, can not sensor test!\n");
		return -EINVAL;
	}

	ret = vts_call_ic_ops_sync(vtsdev, sensor_caliberate, code, &result);
	if (!ret && result == VTS_SENSOR_CALIBERATE_SUCCESS)
		return snprintf(buf, PAGE_SIZE, "Pass");
	else
		return snprintf(buf, PAGE_SIZE, "Failed");
}


static ssize_t vts_at_sensor_test_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);

	switch (vtsc->at_sensor_test_cmd) {
	case 165:
		count = vts_sensor_caliberate(vtsc,165, VTS_TYPE_MAIN, buf);
		break;
	case 166:
		count = vts_sensor_test(vtsc, VTS_TYPE_MAIN, buf);
		break;
	case 167:
		count = vts_sensor_caliberate(vtsc, 167, VTS_TYPE_SECOND, buf);
		break;
	case 168:
		count = vts_sensor_test(vtsc, VTS_TYPE_SECOND, buf);
		break;
	case 169:
		count = vts_sensor_caliberate(vtsc, 169, VTS_TYPE_MAIN, buf);
		break;	
	
	default:
		VTE("no this cmd:%d\n", vtsc->at_sensor_test_cmd);
		break;
	}
	vtsc->at_sensor_test_cmd = 0;

	return count;
}

static ssize_t vts_at_sensor_test_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTE("invalide number of parameters passed\n");
		return -EINVAL;
	}
	vtsc->at_sensor_test_cmd = val;

	return count;
}

static ssize_t vts_factory_key_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev = NULL;
	struct vts_device *tmp = NULL;
	int count = 0;
	struct vts_device *vtsdev_main = NULL;
	struct vts_device *vtsdev_second = NULL;
	u32 type;

	mutex_lock(&vtsc->lock);
	list_for_each_entry_safe(vtsdev, tmp, &vtsc->tps, tp_list) {
		vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &type);
		if (type == VTS_TYPE_MAIN)
			vtsdev_main = vtsdev;		
		if (type == VTS_TYPE_SECOND)
			vtsdev_second = vtsdev;
	}
	mutex_unlock(&vtsc->lock);
	
	if (vtsdev_main) {
		count += snprintf(&buf[count], 2048, "bad_screen:%s\nlcm_noise:%s\nbsp_lcm_noise:%s\nrawdata_test:%s\nRF_test:%s\n",
			vtsdev_main->activity_path[VTS_TEST_APK_TYPE_SENSOR_TEST],
			vtsdev_main->activity_path[VTS_TEST_APK_TYPE_LCM_NOISE_TEST],
			vtsdev_main->activity_path[VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST],
			vtsdev_main->activity_path[VTS_TEST_APK_TYPE_RAWDATA_TEST],
			vtsdev_main->activity_path[VTS_TEST_APK_TYPE_RF_TEST]);
	}
	if (vtsdev_second) {
		count += snprintf(&buf[count], 2048, "bad_screen_back:%s\nlcm_noise_back:%s\nbsp_lcm_noise_back:%s\nrawdata_test_back:%s\nRF_test_back:%s\n",
			vtsdev_second->activity_path[VTS_TEST_APK_TYPE_SENSOR_TEST],
			vtsdev_second->activity_path[VTS_TEST_APK_TYPE_LCM_NOISE_TEST],
			vtsdev_second->activity_path[VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST],
			vtsdev_second->activity_path[VTS_TEST_APK_TYPE_RAWDATA_TEST],
			vtsdev_second->activity_path[VTS_TEST_APK_TYPE_RF_TEST]);
	}
	
	return count;
}

static ssize_t vts_imei_write_support_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main = NULL;
	struct vts_device *vtsdev_sec = NULL;
	u32 main_no_flash;
	u32 second_no_flash;

	vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	vtsdev_sec = vtsdev_lookup_by_type(vtsc, VTS_TYPE_SECOND);

	if (vtsdev_main)
		vts_property_get(vtsdev_main, VTS_PROPERTY_NO_FLASH, &main_no_flash);

	if (vtsdev_sec)
		vts_property_get(vtsdev_sec, VTS_PROPERTY_NO_FLASH, &second_no_flash);

	if (!vtsdev_main)
		return -ENODEV;

	if (!vtsdev_sec)
		return snprintf(buf, PAGE_SIZE, "%d", !main_no_flash);

	return snprintf(buf, PAGE_SIZE, "%s:%d,%s:%d", vts_name(vtsdev_main), !main_no_flash,
		vts_name(vtsdev_sec), !second_no_flash);
}
static ssize_t touchscreen_screen_clock_point_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main = NULL;
    vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	if (vtsdev_main){
		mutex_lock(&vtsdev_main->screen_clock_point.scrclMutex);
		count = snprintf(buf, PAGE_SIZE, "%04d%04d\n", vtsdev_main->screen_clock_point.pointX, vtsdev_main->screen_clock_point.pointY);
		mutex_unlock(&vtsdev_main->screen_clock_point.scrclMutex);
	}
	return count;


}
static ssize_t touchscreen_screen_clock_support_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int count = 0;
	u32 main_screen_support = 0;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main = NULL;
	
	vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	if (vtsdev_main){
		vts_property_get(vtsdev_main, VTS_PROPERTY_SCREEN_CLOCK, &main_screen_support);
		count = snprintf(buf, PAGE_SIZE, "%d\n", main_screen_support);
		if(vtsdev_main->has_screen_clock){//debug
		 main_screen_support = 1;
		 count = snprintf(buf, PAGE_SIZE, "%d\n", main_screen_support);
		}
	}
	return count;
	  
}

static ssize_t touchscreen_screen_clock_support_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)

{	
    //only for debug
    int val;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main;
	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}
	VTI("paramater is %d", val);
	vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	if(vtsdev_main) {
		mutex_lock(&vtsc->lock);
		if (val == 0) {
			vtsdev_main->has_screen_clock = 0;
		} else if (val == 1) {
			vtsdev_main->has_screen_clock = 1;
		} else{
			VTI("invalide parameter:%d", val);
			mutex_unlock(&vtsc->lock);
			return -EINVAL;
		}
		mutex_unlock(&vtsc->lock);
	}
	//VTI("tsState is %d,---normal is 0,sleep is 1,gesture is 2; hasLcdShutoff is %d", atomic_read(&vivo_ts_data->tsState),vivo_ts_data->hasLcdShutoff);
    return count;

}

static ssize_t vts_cali_twice_support_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
    int count = 0;
	u32 cali_twice_support = 0;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main = NULL;

	vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	if (vtsdev_main){
		vts_property_get(vtsdev_main, VTS_PROPERTY_CALIBRATION_TWICE, &cali_twice_support);
		count = snprintf(buf, PAGE_SIZE, "%d\n", cali_twice_support);
	}
	return count;


}
static ssize_t vts_edge_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{	
	int count = 0;
	u32 edge_cfg_support;
	struct vts_node *vtsc = container_of(kobj, struct vts_node, kobj);
	struct vts_device *vtsdev_main = NULL;

	vtsdev_main = vtsdev_lookup_by_type(vtsc, VTS_TYPE_MAIN);
	if (vtsdev_main){
		vts_property_get(vtsdev_main, VTS_PROPERTY_EDGE_REJECTION, &edge_cfg_support);
		count = snprintf(buf, PAGE_SIZE, "version:%s\n", vtsdev_main->module->edge_cfg_version);
	}
	return count;


}



static struct vts_sysfs_entry vts_cali_twice = __ATTR(cali_twice_support, 0664, vts_cali_twice_support_show, vts_null_store);
static struct vts_sysfs_entry vts_log_switch = __ATTR(ts_log_switch, 0664, vts_log_switch_show, vts_log_switch_store);
static struct vts_sysfs_entry vts_touchpanel_devices = __ATTR(touchpanel_devices, 0644, vts_touchpanel_devices_show, vts_null_store);
static struct vts_sysfs_entry vts_app_name = __ATTR(app_name, 0644, vts_null_show, vts_service_cmd_store);
static struct vts_sysfs_entry vts_cali_support = __ATTR(cali_support, 0644, vts_cali_support_show, vts_null_store);
static struct vts_sysfs_entry vts_firmware_version = __ATTR(firmware_version, 0644, vts_fwver_show, vts_null_store);
static struct vts_sysfs_entry vts_firmware_module_id = __ATTR(firmware_module_id, 0644, vts_module_id_show, vts_null_store);
static struct vts_sysfs_entry vts_imei_ctl = __ATTR(imei_ctl, 0644, vts_imei_ctl_show, vts_imei_ctl_store);
static struct vts_sysfs_entry vts_sensor_rx_tx = __ATTR(sensor_rx_tx, 0644, vts_trx_show, vts_null_store);
static struct vts_sysfs_entry vts_gesture_point = __ATTR(gesture_point, 0644, vts_gesture_point_show, vts_null_store);
static struct vts_sysfs_entry vts_ts_super_node = __ATTR(ts_super_node, 0644, vts_ts_super_node_show, vts_ts_super_node_store);
static struct vts_sysfs_entry vts_at_sensor_test = __ATTR(at_sensor_test, 0644, vts_at_sensor_test_show, vts_at_sensor_test_store);
static struct vts_sysfs_entry vts_factory_key = __ATTR(factory_key, 0644, vts_factory_key_show, vts_null_store);
static struct vts_sysfs_entry vts_imei_write_support = __ATTR(imei_write_support, 0644, vts_imei_write_support_show, vts_null_store);
static struct vts_sysfs_entry vts_screen_clock_point_a = __ATTR(screen_clock_point, 0644,touchscreen_screen_clock_point_show, vts_null_store);
static struct vts_sysfs_entry vts_screen_clock_support=__ATTR(screen_clock_support, 0644,touchscreen_screen_clock_support_show, touchscreen_screen_clock_support_store);
static struct vts_sysfs_entry vts_edge_version =__ATTR(edge_version, 0644,vts_edge_version_show, vts_null_store);


static struct attribute *vts_sys_attrs[] = {
	&vts_log_switch.attr,
	&vts_touchpanel_devices.attr,
	&vts_app_name.attr,
	&vts_cali_support.attr,
	&vts_firmware_version.attr,
	&vts_firmware_module_id.attr,
	&vts_imei_ctl.attr,
	&vts_sensor_rx_tx.attr,
	&vts_gesture_point.attr,
	&vts_ts_super_node.attr,
	&vts_at_sensor_test.attr,
	&vts_factory_key.attr,
	&vts_imei_write_support.attr,
	&vts_screen_clock_point_a.attr,
	&vts_screen_clock_support.attr,
	&vts_cali_twice.attr,
	&vts_edge_version.attr,
	NULL
};

static ssize_t vts_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t vts_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);
	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void vts_object_release(struct kobject *kobj)
{
	return;
}

static const struct sysfs_ops vts_object_sysfs_ops = {
	.show = vts_object_show,
	.store = vts_object_store,
};

static struct kobj_type vts_object_type = {
	.sysfs_ops	= &vts_object_sysfs_ops,
	.release	= vts_object_release,
	.default_attrs = vts_sys_attrs,
};

static struct vts_node *vts = NULL;
static DEFINE_MUTEX(vts_node_mutex);

int vts_node_reset_fw_download(enum vts_type type)
{
	struct vts_device *vtsdev;
	int ret;

	mutex_lock(&vts_node_mutex);

	if (!vts) {
		VTE("no vts\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	vtsdev = vtsdev_lookup_by_type(vts, type);
	if (!vtsdev) {
		VTE("no vts data\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	ret = vts_call_func_sync(vtsdev, int, vts_reset);
	if (ret) {
		VTE("reset to download firmware failed!\n");
		mutex_unlock(&vts_node_mutex);
		return ret;
	}

	mutex_unlock(&vts_node_mutex);
	return ret;
}

ssize_t vts_node_rom_zone_write(enum vts_type type, enum vts_zone zone, const u8 *buf, size_t size)
{
	struct vts_device *vtsdev;
	ssize_t size_write;

	mutex_lock(&vts_node_mutex);

	if (!vts) {
		VTE("no vts\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	vtsdev = vtsdev_lookup_by_type(vts, type);
	if (!vtsdev) {
		VTE("no vts data\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	size_write = vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_write, zone, buf, size);
	mutex_unlock(&vts_node_mutex);
	return size_write;
}

ssize_t vts_node_rom_zone_read(enum vts_type type, enum vts_zone zone, u8 *buf, size_t size)
{
	struct vts_device *vtsdev;
	ssize_t size_read;

	mutex_lock(&vts_node_mutex);

	if (!vts) {
		VTE("no vts\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	vtsdev = vtsdev_lookup_by_type(vts, type);
	if (!vtsdev) {
		VTE("no vts data\n");
		mutex_unlock(&vts_node_mutex);
		return -EINVAL;
	}

	size_read = vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_read, zone, buf, size);
	mutex_unlock(&vts_node_mutex);
	return size_read;
}

static int vts_node_init(void)
{
	int ret;

	if (vts)
		return 0;

	vts = kzalloc(sizeof(*vts), GFP_KERNEL);
	if (!vts) {
		VTE("alloc memory for vts core failed!\n");
		return -ENOMEM;
	}

	ret = kobject_init_and_add(&vts->kobj, &vts_object_type, NULL, "touchscreen");
	if (ret) {
		VTE("create vts node faield!\n");
		kfree(vts);
		vts = NULL;
		return ret;
	}

	INIT_LIST_HEAD(&vts->tps);
	mutex_init(&vts->lock);
	vts->code_version = VTS_CODE_VERSION;
	return 0;
}

static void vts_node_exit(void)
{
	mutex_destroy(&vts->lock);
	kobject_del(&vts->kobj);
	kfree(vts);
	vts = NULL;
}

int vts_node_sysfs_add(struct vts_device *vtsdev)
{
	int ret;

	mutex_lock(&vts_node_mutex);
	ret = vts_node_init();
	if (ret) {
		vts_dev_err(vtsdev, "node init failed!\n");
		mutex_unlock(&vts_node_mutex);
		return ret;
	}

	mutex_lock(&vts->lock);
	list_add_tail(&vtsdev->tp_list, &vts->tps);
	mutex_unlock(&vts->lock);
	mutex_unlock(&vts_node_mutex);
	return 0;
}

void vts_node_sysfs_remove(struct vts_device *vtsdev)
{
	mutex_lock(&vts_node_mutex);

	if (!vts) {
		mutex_unlock(&vts_node_mutex);
		return ;
	}

	mutex_lock(&vts->lock);
	list_del(&vtsdev->tp_list);
	mutex_unlock(&vts->lock);

	if (list_empty(&vts->tps))
		vts_node_exit();

	mutex_unlock(&vts_node_mutex);
}
