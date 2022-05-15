#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/rwsem.h>
#include <linux/sensors.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "vts_core.h"

static struct class *vts_class = NULL;
static atomic_t nr_devices = ATOMIC_INIT(0);

#define VTS_STATE_ATTR(name, state) \
	static ssize_t vts_##name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int val; \
		int ret; \
		\
		if (kstrtoint(buf, 10, &val)) { \
			vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf); \
			return -EINVAL; \
		} \
		\
		ret = vts_state_set(vtsdev, state, val); \
		if (ret) { \
			vts_dev_err(vtsdev, "send kthread state "#state" error, ret = %d\n", ret); \
			return ret; \
		} \
		\
		return size; \
	} \
	\
	static ssize_t vts_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
	 	\
		ret = vts_state_get(vtsdev, state); \
		if (ret < 0) { \
			vts_dev_err(vtsdev, "get state %d error, ret = %d\n", state, ret); \
			return ret; \
		} \
		\
		return snprintf(buf, PAGE_SIZE, "%d\n", ret); \
	} \
	\
	static DEVICE_ATTR(name, 0644, vts_##name##_show, vts_##name##_store)

#define VTS_INFO_ATTR(name, field_name) \
	static ssize_t vts_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		u32 val; \
		int ret; \
	\
		ret = vts_property_get(vtsdev, field_name, &val); \
		if (ret) { \
			vts_dev_err(vtsdev, "get propperty"#field_name" failed, ret = %d\n", ret); \
			return ret; \
		} \
		return snprintf(buf, PAGE_SIZE, "%d\n", val); \
	} \
	\
	static DEVICE_ATTR(name, 0644, vts_##name##_show, NULL)

#define VTS_FW_ATTR(name, type) \
	static ssize_t vts_##name##_show(struct device *dev, \
				struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_get, type, buf, PAGE_SIZE); \
		if (ret) \
			return ret; \
	\
		sprintf(buf + strlen(buf), "\n"); \
		return strlen(buf); \
	} \
	\
	static ssize_t vts_##name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		int ret; \
		char *tmp = (char *)buf; \
		char *path; \
		char *update_s; \
		int update; \
	\
		path = strsep(&tmp, ","); \
		if (!path) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		update_s = strsep(&tmp, ","); \
		if (!update_s) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		if(kstrtoint(update_s, 10, &update)) { \
			vts_dev_err(vtsdev, "invalid args!"); \
			return -EINVAL; \
		} \
	\
		ret = vts_call_func_sync(vtsdev, int, vts_fw_path_set, type, buf, update != 0); \
		if (ret) \
			return ret; \
	\
		return size; \
	} \
	static DEVICE_ATTR(name, 0644, vts_##name##_show, vts_##name##_store)

#define FRAME_DATA_ATTR(name, type) \
	static ssize_t vts_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		struct vts_frame *frame; \
		ssize_t data_bytes; \
		\
		frame = vts_call_func_sync(vtsdev, struct vts_frame *, vts_frame_data_get, type);  \
		if (!frame) { \
			vts_dev_err(vtsdev, "get frame data error! type = %d\n", type); \
			return -EIO; \
		} \
		\
		data_bytes = frame->size * sizeof(*(frame->data)); \
		if (data_bytes >= PAGE_SIZE) { \
			vts_dev_err(vtsdev, "frame size is too large!! size = %ld\n", data_bytes); \
			vts_frame_data_put(vtsdev, frame); \
			return -ENOMEM; \
		} \
		\
		memcpy(buf, frame->data, data_bytes); \
		vts_frame_data_put(vtsdev, frame); \
		return data_bytes; \
	} \
	\
	static DEVICE_ATTR(name, 0644, vts_##name##_show, NULL)

#define VTS_ROM_ZONE_ATTR(name, zone) \
	static ssize_t vts_##name##_store(struct device *dev, \
			struct device_attribute *attr, const char *buf, size_t size) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		return vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_write, zone, buf, size); \
	} \
	\
	static ssize_t vts_##name##_show(struct device *dev, \
			struct device_attribute *attr, char *buf) \
	{ \
		struct vts_device *vtsdev = dev_get_drvdata(dev); \
		ssize_t zone_size; \
		u8 *zone_data; \
		ssize_t read_len; \
		\
		zone_size = vts_rom_zone_size(vtsdev, zone); \
		if (zone_size <= 0) { \
			vts_dev_err(vtsdev, "incorrect zone size %ld\n", zone_size); \
			return zone_size; \
		} \
		\
		zone_data = kzalloc(zone_size, GFP_KERNEL); \
		if (!zone_data) { \
			vts_dev_err(vtsdev, "no memory for zone data!\n"); \
			return -ENOMEM; \
		} \
		\
		read_len = vts_call_func_sync(vtsdev, ssize_t, vts_rom_zone_read, zone, zone_data, zone_size); \
		if (read_len != zone_size) { \
			vts_dev_err(vtsdev, "read zone data error! read_len = %ld, zone = %d, zone_size = %ld\n", \
			read_len, zone, zone_size); \
			kfree(zone_data); \
			return -EIO; \
		} \
		\
		memcpy(buf, zone_data, zone_size); \
		kfree(zone_data); \
		return zone_size; \
	} \
	\
	static DEVICE_ATTR(name, 0644, vts_##name##_show, vts_##name##_store)


static ssize_t vts_factory_key_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "bad_screen:%s\nlcm_noise:%s\nbsp_lcm_noise:%s\nrawdata_test:%s\nRF_test:%s\n",
		vtsdev->activity_path[VTS_TEST_APK_TYPE_SENSOR_TEST],
		vtsdev->activity_path[VTS_TEST_APK_TYPE_LCM_NOISE_TEST],
		vtsdev->activity_path[VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST],
		vtsdev->activity_path[VTS_TEST_APK_TYPE_RAWDATA_TEST],
		vtsdev->activity_path[VTS_TEST_APK_TYPE_RF_TEST]);
}

static ssize_t vts_lcmid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid;
	int ret;

	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid, &lcmid);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", lcmid);
}

static ssize_t vts_lcmid_compatible_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 lcmid[VTS_MODULE_LCMID_NR_MAX];
	int ret;
	ssize_t count = 0;
	size_t sz = ARRAY_SIZE(lcmid);
	int i;

	memset(lcmid, 0, sizeof(lcmid));
	ret = vts_call_func_sync(vtsdev, int, vts_get_lcmid_compatible, lcmid, &sz);
	if (ret)
		return ret;

	if (sz == 0)
		return -ENODEV;

	for (i = 0; i < sz; i++)
		if (i == (sz -1))
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d\n", lcmid[i]);
		else
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "%d ", lcmid[i]);

	return count;
}

/* 获取dts中与cmdline 匹配的 softid */
static ssize_t vts_softid_cmdline_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	struct vts_vendor_module_id *vendor_module_id;
	u32 softid = vtsdev->module->softid_cmdline;
	u32 matched_softid = 0;

	if(softid <= 0)
		return -ENODEV;

	list_for_each_entry(vendor_module_id, &vtsdev->module->vendor_module_list, entry) {
		if (vendor_module_id->softid == softid) {
			matched_softid = vendor_module_id->softid;
			break;
		}
	}
	return snprintf(buf, PAGE_SIZE, "0x%x\n", matched_softid);
}

/* 获取dts中softid对应的module_id */
static ssize_t vts_softid_module_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 module_id[VTS_MODULE_LCMID_NR_MAX];
	int ret;
	ssize_t count = 0;
	size_t sz = ARRAY_SIZE(module_id);
	int i;

	memset(module_id, 0, sizeof(module_id));
	ret = vts_call_func_sync(vtsdev, int, vts_get_module_id_compatible, module_id, &sz);
	if (ret)
		return ret;

	for (i = 0; i < sz; i++)
		if (i == (sz - 1))
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "0x%x\n", module_id[i]);
		else
			count += snprintf(buf + strlen(buf), PAGE_SIZE - strlen(buf), "0x%x ", module_id[i]);

	return count;
}

static ssize_t vts_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u64 version;
	int ret;
	ret = vts_call_func_sync(vtsdev, int, vts_firmware_version_get, &version);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "0x%llx\n", version);
}

static ssize_t vts_data_info_show(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "ESD_ERR : %d, WATCHDOG_ERR: %d, CHECKSUM_ERR : %d, IIC_ERR : %d\n",
					vtsdev->exceptions[VTS_EXCEPTION_ESD_ERR],
					vtsdev->exceptions[VTS_EXCEPTION_WATCHDOG_ERR],
					vtsdev->exceptions[VTS_EXCEPTION_CHECKSUM_ERR],
					vtsdev->exceptions[VTS_EXCEPTION_I2C_ERR]);
	
	return ret; 
}

static ssize_t vts_touch_ic_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int ret;
	ret = vts_call_func_sync(vtsdev, int, vts_touch_ic_mode_get);
	if (ret)
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t vts_special_calibration_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int ret;

	ret = vts_call_func_sync(vtsdev, int, vts_get_calibration_status);
	if (ret)
		return snprintf(buf, PAGE_SIZE, "%d\n", 1);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t vts_gesture_points_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u16 x[32];
	u16 y[32];
	int points = 0;
	int i;
	int ret;
	u16 *points_buf = (u16 *)buf;

	memset(x, 0, sizeof(x));
	memset(y, 0, sizeof(y));

	ret = vts_call_func_sync(vtsdev, int, vts_report_coordinates_get, x, y, ARRAY_SIZE(x), &points);
	if (ret) {
		vts_dev_err(vtsdev, "get gesture points failed! ret = %d\n", ret);
		return ret;
	}

	for (i = 0; i < points; i++) {
		points_buf[2 * i] = x[i];
		points_buf[2 * i + 1] = y[i];
	}

	return points * sizeof(*points_buf) * 2;
}

static ssize_t vts_touch_area_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int ret;
	struct vts_grip_area area;

	ret = vts_call_ic_ops_sync(vtsdev, get_grip_status, &area);
	if (ret) {
		vts_dev_err(vtsdev, "get touch area failed! ret = %d\n", ret);
		return ret;
	}

	return snprintf(buf, PAGE_SIZE, "%d %d\n", area.area_center, area.area_edge);
}

static ssize_t vts_test_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	enum vts_sensor_test_result result = VTS_SENSOR_TEST_SUCCESS;
	int ret;

	if(vts_get_run_mode(vtsdev) != VTS_ST_NORMAL) {
		vts_dev_err(vtsdev, "not in normal mode, can not sensor test!\n");
		return -EINVAL;
	}

	ret = vts_call_ic_ops_sync(vtsdev, sensor_test, &result);
	vts_event(VTS_EVENT_SENSOR_TEST_FINISHED, (int)result);

	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}

static ssize_t vts_idle_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;
	int ret;

	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}

	atomic_set(&vtsdev->idle_state, val);

	ret = vts_call_ic_ops_sync(vtsdev, set_auto_idle, val);
	if (ret)
		return ret;

	return size;
}

static ssize_t vts_idle_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&vtsdev->idle_state));
}

static ssize_t vts_firmware_cache_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}

	atomic_set(&vtsdev->firmware_cache, val);
	return size;
}

static ssize_t vts_firmware_cache_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&vtsdev->firmware_cache));
}

static ssize_t vts_points_inject_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	char path[256];

	memset(path, 0, sizeof(path));
	memcpy(path, buf, size-1);
	vts_report_inject_points(vtsdev, path);
	return size;
}

#define __PARSE_PARAM(type, buf, val) do { \
		char *token; \
		\
		token = strsep(&buf, ","); \
		if (!token) \
			return -EINVAL; \
		\
		if(kstrto##type(token, 0, val)) \
			return -EINVAL; \
	} while (0)
#define PARSE_INT(buf, val) __PARSE_PARAM(int, buf, val)
#define PARSE_U32(buf, val) __PARSE_PARAM(u32, buf, val)
#define PARSE_U64(buf, val) __PARSE_PARAM(u64, buf, val)

static int vts_get_edge_cmd(struct vts_edge_cmd *buf, char *cmd) {
	PARSE_INT(cmd, &buf->x);
	PARSE_INT(cmd, &buf->y);
	PARSE_INT(cmd, &buf->width);
	PARSE_INT(cmd, &buf->height);
	PARSE_INT(cmd, &buf->area_type);
	PARSE_INT(cmd, (int*)&(buf->enable));
	return 0;
}

static ssize_t vts_touch_edge_area_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	u32 x_max = 0;
	u32 y_max = 0;
	u32 resolution = 0;
	int ret = 0;
	struct vts_edge_cmd edge_cmd;
	char *tmp = (char *)buf;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);

	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &x_max);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &y_max);
	} else {
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &x_max);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &y_max);
	}

	if (vts_get_edge_cmd(&edge_cmd, tmp)){
		VTE("get edge cmd error.");
		return -EINVAL;
	}
	if (edge_cmd.x < 0 || edge_cmd.x > x_max || edge_cmd.y < 0 || edge_cmd.y > y_max
				|| edge_cmd.width > x_max || edge_cmd.height > y_max) {
		VTE("edge cmd is out of size\n");
		return -EINVAL;
	}
	VTI("the edge_cmd is x:%d, y:%d, width:%d, height:%d, area_type:%d, enable:%d, index:%d",edge_cmd.x, edge_cmd.y, edge_cmd.width,
								edge_cmd.height, edge_cmd.area_type, edge_cmd.enable, edge_cmd.index);
	ret = vts_call_ic_ops_sync(vtsdev, set_edge_reject_area, &edge_cmd);
	if (ret) {
		VTE("call ic ops set_edge_reject_area failed, ret = %d", ret);
		return ret;
	}
	return size;
}

static ssize_t vts_report_flags_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long flags;
	int ret;
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	if(kstrtoul(buf, 0, &flags)) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	ret = vts_report_set_flags(vtsdev, flags);
	if (ret)
		return ret;

	return size;
}

static ssize_t vts_report_flags_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%lx\n", vts_report_flags(vtsdev));
}

static ssize_t vts_sched_flags_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long flags;
	int ret;
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	if(kstrtoul(buf, 0, &flags)) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	ret = vts_sched_set_flags(vtsdev, flags);
	if (ret)
		return ret;

	return size;
}

static ssize_t vts_sched_flags_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%lx\n", vts_sched_flags(vtsdev));
}

static ssize_t vts_sched_lpm_timeout_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long timeout;
	int ret;
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	if(kstrtoul(buf, 0, &timeout)) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	ret = vts_sched_set_lpm_timeout(vtsdev, timeout);
	if (ret)
		return ret;

	return size;
}

static ssize_t vts_sched_lpm_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%lx\n", vts_sched_lpm_timeout(vtsdev));
}

static ssize_t vts_sched_ddr_timeout_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long timeout;
	int ret;
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	if(kstrtoul(buf, 0, &timeout)) {
		vts_dev_err(vtsdev, "invalid args\n");
		return -EINVAL;
	}

	ret = vts_sched_set_ddr_timeout(vtsdev, timeout);
	if (ret)
		return ret;

	return size;
}

static ssize_t vts_sched_ddr_timeout_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%lx\n", vts_sched_ddr_timeout(vtsdev));
}

/******      add for fingerprint begin         ******/
static int vts_get_aoi_cmd(struct vts_aoi_cmd *buf, char *cmd) {
	PARSE_INT(cmd, &buf->aoi_left);
	PARSE_INT(cmd, &buf->aoi_top);
	PARSE_INT(cmd, &buf->aoi_right);
	PARSE_INT(cmd, &buf->aoi_bottom);
	return 0;
}
			
static ssize_t vts_aoi_set_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int max_x, max_y;
	int ret = 0;
	struct vts_aoi_cmd aoi_cmd;
	char *tmp = (char *)buf;
	int high_resloution = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &high_resloution);
	if (high_resloution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &max_y);
	} else {
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &max_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &max_y);
	}

	if (vts_get_aoi_cmd(&aoi_cmd, tmp)){
		vts_dev_err(vtsdev, "get aoi cmd error.");
		return -EINVAL;
	}
	if (aoi_cmd.aoi_left < 0 || aoi_cmd.aoi_left > max_x || aoi_cmd.aoi_right < 0 || aoi_cmd.aoi_right > max_x) {
		vtsdev->aoi_notify_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_top < 0 || aoi_cmd.aoi_top > max_y || aoi_cmd.aoi_bottom < 0 || aoi_cmd.aoi_bottom > max_y) {
		vtsdev->aoi_notify_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_left >= aoi_cmd.aoi_right || aoi_cmd.aoi_top >= aoi_cmd.aoi_bottom) {
		vtsdev->aoi_notify_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_left == 0 && aoi_cmd.aoi_right == 0 && aoi_cmd.aoi_top == 0 && aoi_cmd.aoi_bottom == 0) {
		vtsdev->aoi_notify_enabled = false;
		goto config_aoi;
	}

	vtsdev->aoi_cmd.aoi_left = aoi_cmd.aoi_left;
	vtsdev->aoi_cmd.aoi_top = aoi_cmd.aoi_top;
	vtsdev->aoi_cmd.aoi_right = aoi_cmd.aoi_right;
	vtsdev->aoi_cmd.aoi_bottom = aoi_cmd.aoi_bottom;

	vtsdev->aoi_notify_enabled = true;

config_aoi:
	VTI("the aoi_cmd is x:%d, y:%d, width:%d, height:%d", aoi_cmd.aoi_left, aoi_cmd.aoi_top, aoi_cmd.aoi_right, aoi_cmd.aoi_bottom);
	ret = vts_state_set(vtsdev, VTS_STA_FP_AOI, (int)vtsdev->aoi_notify_enabled);
	if (ret) {
		VTE("set state is error, ret = %d", ret);
		return ret;
	}
	/*
	ret = vts_call_ic_ops_sync(vtsdev, set_aoi_zone, &aoi_cmd);
	if (ret) {
		vts_dev_err(vtsdev, "call ic ops set_aoi_zone failed, ret = %d", ret);
		return ret;
	}
	*/
	return size;
}

static ssize_t vts_aoi_set_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;

	len = scnprintf(buf + len, PAGE_SIZE,
				"%d %d %d %d, %d",
				vtsdev->aoi_cmd.aoi_left,
				vtsdev->aoi_cmd.aoi_top,
				vtsdev->aoi_cmd.aoi_right,
				vtsdev->aoi_cmd.aoi_bottom,
				(int)vtsdev->aoi_notify_enabled);

	return len;
}

static ssize_t vts_touch_event_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;
	struct vts_touch_event *touch = (struct vts_touch_event *)vtsdev->event;
	struct vts_aoi_info *info = (struct vts_aoi_info *)vtsdev->aoi_info;
	unsigned long finger_pressed = vtsdev->finger_pressed;

	for (i = 0; finger_pressed > 0 && i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		if (test_bit(i, &finger_pressed))
			len += scnprintf((buf + len), PAGE_SIZE - len,
					"%d %d %d %d %d %d %d ",
					touch->x,
					touch->y,
					info->aoi_area_in,
					info->aoi_area_out,
					info->aoi_area_width,
					info->aoi_area_height,
					info->aoi_avg_signal);
		touch++;
		info++;
	}

	return len;
}

static ssize_t vts_aoi_int_pin_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int ret, val;

	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}
	if (vts_get_run_mode(vtsdev) == VTS_ST_SLEEP) {
		vts_dev_err(vtsdev, "IC in sleep mode !");
		return -EINVAL;
	}

	ret = vts_call_ic_ops_sync(vtsdev, set_aoi_intpin, val);
	if (ret) {
		vts_dev_err(vtsdev, "call ic ops set_aoi_intpin failed, ret = %d", ret);
		return ret;
	}

	vtsdev->aoi_intpin = val;

	return size;
}

static ssize_t vts_aoi_int_pin_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;

	len = scnprintf(buf + len, PAGE_SIZE, "%d", vtsdev->aoi_intpin);

	return len;
}

static ssize_t vts_aoi_report_type_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)  //real point/signal max point/center point
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}

	if (val > 2) val = 0;

	vtsdev->aoi_report_type = val;

	return size;
}

static ssize_t vts_aoi_report_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;

	len = scnprintf(buf + len, PAGE_SIZE, "%d", vtsdev->aoi_report_type);

	return len;
}

static ssize_t vts_aoi_intpin_ctrl_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int max_x, max_y;
	int ret = 0;
	struct vts_aoi_cmd aoi_cmd;
	char *tmp = (char *)buf;

	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &max_x);
	vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &max_y);

	if (vts_get_aoi_cmd(&aoi_cmd, tmp)){
		vts_dev_err(vtsdev, "get aoi cmd error.");
		return -EINVAL;
	}
	if (aoi_cmd.aoi_left < 0 || aoi_cmd.aoi_left > max_x || aoi_cmd.aoi_right < 0 || aoi_cmd.aoi_right > max_x) {
		vtsdev->aoi_int_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_top < 0 || aoi_cmd.aoi_top > max_y || aoi_cmd.aoi_bottom < 0 || aoi_cmd.aoi_bottom > max_y) {
		vtsdev->aoi_int_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_left >= aoi_cmd.aoi_right || aoi_cmd.aoi_top >= aoi_cmd.aoi_bottom) {
		vtsdev->aoi_int_enabled = false;
		goto config_aoi;
	}

	if (aoi_cmd.aoi_left == 0 && aoi_cmd.aoi_right == 0 && aoi_cmd.aoi_top == 0 && aoi_cmd.aoi_bottom == 0) {
		vtsdev->aoi_int_enabled = false;
		goto config_aoi;
	}

	vtsdev->aoi_cmd.aoi_left = aoi_cmd.aoi_left;
	vtsdev->aoi_cmd.aoi_top = aoi_cmd.aoi_top;
	vtsdev->aoi_cmd.aoi_right = aoi_cmd.aoi_right;
	vtsdev->aoi_cmd.aoi_bottom = aoi_cmd.aoi_bottom;

	vtsdev->aoi_int_enabled = true;

config_aoi:
	VTI("the aoi_cmd is x:%d, y:%d, width:%d, height:%d", aoi_cmd.aoi_left, aoi_cmd.aoi_top, aoi_cmd.aoi_right, aoi_cmd.aoi_bottom);
	ret = vts_state_set(vtsdev, VTS_STA_FP_AOI_INT, (int)vtsdev->aoi_int_enabled);
	if (ret) {
		VTE("set state is error, ret = %d", ret);
		return ret;
	}
	/*
	ret = vts_call_ic_ops_sync(vtsdev, set_aoi_int_zone, (int)vtsdev->aoi_int_enabled);
	if (ret) {
		vts_dev_err(vtsdev, "call ic ops set_aoi_int_zone failed, ret = %d", ret);
		return ret;
	}
	*/
	return size;
}

static ssize_t vts_aoi_intpin_ctrl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;

	len = scnprintf(buf + len, PAGE_SIZE,
				"%d %d %d %d, %d",
				vtsdev->aoi_cmd.aoi_left,
				vtsdev->aoi_cmd.aoi_top,
				vtsdev->aoi_cmd.aoi_right,
				vtsdev->aoi_cmd.aoi_bottom,
				(int)vtsdev->aoi_int_enabled);

	return len;
}
/******      add for fingerprint end         ******/

/*card slide region begin*/
static ssize_t vts_card_region_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	if (sscanf(buf, "%d %d %d", &vtsdev->y0, &vtsdev->y1, &vtsdev->width) != 3) {
		VTE("invalide number of parameters passed\n");
		return -EINVAL;
	}
	VTI("y0 = %d y1 = %d width = %d", vtsdev->y0, vtsdev->y1, vtsdev->width);
	return size;
}
static ssize_t vts_card_region_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	size_t len = 0;

	len = scnprintf(buf + len, PAGE_SIZE, "%d %d %d",
		vtsdev->y0, vtsdev->y1, vtsdev->width);

	return len;
}
/*card slide region end*/
static ssize_t vts_ops_game_mode_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;
	int ret;
	if (vts_get_run_mode(vtsdev) == VTS_ST_SLEEP) {
		vts_dev_err(vtsdev, "can not call ic ops in sleep mode ! \n");
		return -EINVAL;
	}
		
	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}
	vts_dev_info(vtsdev, "param data: %d", val);
	ret = vts_call_ic_ops_sync(vtsdev, set_game_mode, val);
	if (ret)
		return ret;

	return size;
}
static ssize_t vts_ops_idle_time_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;
	int ret;
	if (vts_get_run_mode(vtsdev) == VTS_ST_SLEEP) {
		vts_dev_err(vtsdev, "can not call ic ops in sleep mode ! \n");
		return -EINVAL;
	}
		
	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}
	vts_dev_info(vtsdev, "param data: %d", val);
	ret = vts_call_ic_ops_sync(vtsdev, set_idle_time, val);
	if (ret)
		return ret;

	return size;
}
static ssize_t vts_ops_high_rate_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);
	int val;
	int ret;
	if (vts_get_run_mode(vtsdev) == VTS_ST_SLEEP) {
		vts_dev_err(vtsdev, "can not call ic ops in sleep mode ! \n");
		return -EINVAL;
	}
	
	if (kstrtoint(buf, 10, &val)) {
		vts_dev_err(vtsdev, "string to int error! buf: %s\n", buf);
		return -EINVAL;
	}
	vts_dev_info(vtsdev, "param data: %d", val);
	ret = vts_call_ic_ops_sync(vtsdev, set_high_report_rate, val);
	if (ret)
		return ret;

	return size;
}




VTS_STATE_ATTR(lcd_on, VTS_STA_LCD);
VTS_STATE_ATTR(proximity, VTS_STA_PROX_STATE);
VTS_STATE_ATTR(charging, VTS_STA_USB_CHARGE);
VTS_STATE_ATTR(force_normal, VTS_STA_FORCE_NORMAL);
VTS_STATE_ATTR(finger_up, VTS_STA_NOTICE_UP);
VTS_STATE_ATTR(faceprint, VTS_STA_FACE_HIGHLIGHT);
VTS_STATE_ATTR(fingerprint, VTS_STA_FINGER_HIGHLIGHT);
VTS_STATE_ATTR(fingerunlock, VTS_STA_FINGER_UNLOCK_OPEN);
VTS_STATE_ATTR(rotation, VTS_STA_ROTATION);
VTS_STATE_ATTR(game_mode, VTS_STA_GAME_MODE);
VTS_STATE_ATTR(gesture, VTS_STA_GESTURE);
VTS_STATE_ATTR(virt_prox_enable, VTS_STA_VIRTUAL_PROX_STATE);
VTS_STATE_ATTR(calling, VTS_STA_CALLING);
VTS_STATE_ATTR(virtual_key, VTS_STA_VIRTUAL_KEY);
VTS_INFO_ATTR(screen_clock_support, VTS_PROPERTY_SCREEN_CLOCK);
VTS_INFO_ATTR(tx_sensors, VTS_PROPERTY_SENSOR_TX_NUM);
VTS_INFO_ATTR(rx_sensors, VTS_PROPERTY_SENSOR_RX_NUM);
VTS_INFO_ATTR(ic, VTS_PROPERTY_IC_NUMBER);
VTS_INFO_ATTR(incell, VTS_PROPERTY_TDDI);
VTS_INFO_ATTR(dimension_x, VTS_PROPERTY_DIMENTION_X);
VTS_INFO_ATTR(dimension_y, VTS_PROPERTY_DIMENTION_Y);
VTS_INFO_ATTR(display_x, VTS_PROPERTY_DISPLAY_X);
VTS_INFO_ATTR(display_y, VTS_PROPERTY_DISPLAY_Y);
VTS_INFO_ATTR(resolution_adjust, VTS_PROPERTY_RESOLUTION_ADJUST);
VTS_INFO_ATTR(need_calibration, VTS_PROPERTY_NEED_CALI);
VTS_INFO_ATTR(type, VTS_PROPERTY_PANEL_TYPE);
VTS_INFO_ATTR(virt_prox, VTS_PROPERTY_VIRTUAL_PROXIMINITY);
VTS_INFO_ATTR(long_press, VTS_PROPERTY_LONG_PRESS);
VTS_INFO_ATTR(no_flash, VTS_PROPERTY_NO_FLASH);
VTS_INFO_ATTR(policy, VTS_PROPERTY_POLICY);
VTS_INFO_ATTR(vendor, VTS_PROPERTY_VENDOR);
VTS_FW_ATTR(firmware, VTS_FW_TYPE_FW);
VTS_FW_ATTR(firmware_config, VTS_FW_TYPE_CONFIG);
VTS_FW_ATTR(firmware_factory, VTS_FW_TYPE_FACTORY);
VTS_FW_ATTR(threshold, VTS_FW_TYPE_LIMIT);
VTS_FW_ATTR(firmware_mp, VTS_FW_TYPE_MP);
FRAME_DATA_ATTR(mutual_raw, VTS_FRAME_MUTUAL_RAW);
FRAME_DATA_ATTR(mutual_delta, VTS_FRAME_MUTUAL_DELTA);
FRAME_DATA_ATTR(self_raw, VTS_FRAME_SELF_RAW);
FRAME_DATA_ATTR(self_delta, VTS_FRAME_SELF_DELTA);
FRAME_DATA_ATTR(mutual_self_raw, VTS_FRAME_MUTUAL_SELF_RAW);
FRAME_DATA_ATTR(mutual_self_delta, VTS_FRAME_MUTUAL_SELF_DELTA);
VTS_ROM_ZONE_ATTR(imei,VTS_ROM_ZONE_IMEI);
VTS_ROM_ZONE_ATTR(lcm,VTS_ROM_ZONE_LCM);
static DEVICE_ATTR(factory_key, 0644, vts_factory_key_show, NULL);
static DEVICE_ATTR(lcmid, 0644, vts_lcmid_show, NULL);
static DEVICE_ATTR(lcmid_compatible, 0644, vts_lcmid_compatible_show, NULL);
static DEVICE_ATTR(softid_cmdline, 0644, vts_softid_cmdline_show, NULL);
static DEVICE_ATTR(softid_module, 0644, vts_softid_module_show, NULL);
static DEVICE_ATTR(version, 0644, vts_version_show, NULL);
static DEVICE_ATTR(touch_ic_mode, 0644, vts_touch_ic_mode_show, NULL);
static DEVICE_ATTR(special_calibration_status, 0644, vts_special_calibration_status_show, NULL);
static DEVICE_ATTR(gesture_points, 0644, vts_gesture_points_show, NULL);
static DEVICE_ATTR(touch_area, 0644, vts_touch_area_show, NULL);
static DEVICE_ATTR(sensor_test, 0644, vts_test_show, NULL);
static DEVICE_ATTR(idle, 0644, vts_idle_show, vts_idle_store);
static DEVICE_ATTR(firmware_cache, 0644, vts_firmware_cache_show, vts_firmware_cache_store);
static DEVICE_ATTR(touch_edge_area, 0644, NULL, vts_touch_edge_area_store);
static DEVICE_ATTR(report_flags, 0644, vts_report_flags_show, vts_report_flags_store);
static DEVICE_ATTR(ts_data_info, 0644, vts_data_info_show, NULL);
static DEVICE_ATTR(sched_flags, 0644, vts_sched_flags_show, vts_sched_flags_store);
static DEVICE_ATTR(lpm_timeout, 0644, vts_sched_lpm_timeout_show, vts_sched_lpm_timeout_store);
static DEVICE_ATTR(ddr_timeout, 0644, vts_sched_ddr_timeout_show, vts_sched_ddr_timeout_store);
static DEVICE_ATTR(points_inject, 0644, NULL, vts_points_inject_store);
static DEVICE_ATTR(aoi_set, 0644, vts_aoi_set_show, vts_aoi_set_store);
static DEVICE_ATTR(touch_event, 0644, vts_touch_event_show, NULL);
static DEVICE_ATTR(aoi_int_pin, 0644, vts_aoi_int_pin_show, vts_aoi_int_pin_store);
static DEVICE_ATTR(aoi_report_type, 0644, vts_aoi_report_type_show, vts_aoi_report_type_store);
static DEVICE_ATTR(touch_int, 0644, vts_aoi_intpin_ctrl_show, vts_aoi_intpin_ctrl_store);
static DEVICE_ATTR(card_region, 0644, vts_card_region_show, vts_card_region_store);
static DEVICE_ATTR(game_state, 0644, NULL, vts_ops_game_mode_store);
static DEVICE_ATTR(high_rate, 0644, NULL, vts_ops_high_rate_store);
static DEVICE_ATTR(idle_time, 0644, NULL, vts_ops_idle_time_store);




static struct attribute *vts_dev_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_touch_ic_mode.attr,
	&dev_attr_special_calibration_status.attr,
	&dev_attr_firmware.attr,
	&dev_attr_firmware_mp.attr,
	&dev_attr_firmware_config.attr,
	&dev_attr_firmware_factory.attr,
	&dev_attr_threshold.attr,
	&dev_attr_gesture_points.attr,
	&dev_attr_touch_area.attr,
	&dev_attr_sensor_test.attr,
	&dev_attr_factory_key.attr,
	&dev_attr_firmware_cache.attr,
	&dev_attr_touch_edge_area.attr,
	&dev_attr_report_flags.attr,
	&dev_attr_ts_data_info.attr,
	&dev_attr_sched_flags.attr,
	&dev_attr_lpm_timeout.attr,
	&dev_attr_ddr_timeout.attr,
	&dev_attr_points_inject.attr,
	&dev_attr_card_region.attr,
	NULL,
};

static struct attribute_group vts_attr_group = {
	.name	= NULL,
	.attrs	= vts_dev_attrs,
};

static struct attribute *vts_frame_attrs[] = {
	&dev_attr_mutual_raw.attr,
	&dev_attr_mutual_delta.attr,
	&dev_attr_self_raw.attr,
	&dev_attr_self_delta.attr,
	&dev_attr_mutual_self_raw.attr,
	&dev_attr_mutual_self_delta.attr,
	NULL,
};

static struct attribute_group frames_attr_group = {
	.name	= "frames",
	.attrs	= vts_frame_attrs,
};

static struct attribute *vts_rom_attrs[] = {
	&dev_attr_imei.attr,
	&dev_attr_lcm.attr,
	NULL,
};

static struct attribute_group rom_attr_group = {
	.name	= "rom",
	.attrs	= vts_rom_attrs,
};

static struct attribute *vts_property_attrs[] = {
	&dev_attr_tx_sensors.attr,
	&dev_attr_rx_sensors.attr,
	&dev_attr_ic.attr,
	&dev_attr_incell.attr,
	&dev_attr_dimension_x.attr,
	&dev_attr_dimension_y.attr,
	&dev_attr_display_x.attr,
	&dev_attr_display_y.attr,
	&dev_attr_resolution_adjust.attr,
	&dev_attr_need_calibration.attr,
	&dev_attr_type.attr,
	&dev_attr_virt_prox.attr,
	&dev_attr_long_press.attr,
	&dev_attr_no_flash.attr,
	&dev_attr_policy.attr,
	&dev_attr_vendor.attr,
	&dev_attr_lcmid.attr,
	&dev_attr_lcmid_compatible.attr,
	&dev_attr_softid_cmdline.attr,
	&dev_attr_softid_module.attr,
	NULL,
};

static struct attribute_group property_attr_group = {
	.name	= "properties",
	.attrs	= vts_property_attrs,
};

static struct attribute *vts_status_attrs[] = {
	&dev_attr_lcd_on.attr,
	&dev_attr_proximity.attr,
	&dev_attr_charging.attr,
	&dev_attr_force_normal.attr,
	&dev_attr_finger_up.attr,
	&dev_attr_faceprint.attr,
	&dev_attr_fingerunlock.attr,
	&dev_attr_fingerprint.attr,
	&dev_attr_rotation.attr,
	&dev_attr_gesture.attr,
	&dev_attr_virt_prox_enable.attr,
	&dev_attr_calling.attr,
	&dev_attr_idle.attr,
	&dev_attr_game_mode.attr,
	&dev_attr_virtual_key.attr,
	&dev_attr_screen_clock_support.attr,
	NULL,
};


static struct attribute_group status_attr_group = {
	.name	= "status",
	.attrs	= vts_status_attrs,
};

static struct attribute *touch_aoi_attrs[] = {
	&dev_attr_aoi_set.attr,
	&dev_attr_touch_event.attr,
	&dev_attr_aoi_int_pin.attr,
	&dev_attr_aoi_report_type.attr,
	&dev_attr_touch_int.attr,
	NULL,
};

static struct attribute_group touch_aoi_attr_group = {
	.name   = "touch_aoi",
	.attrs  = touch_aoi_attrs,
};
static struct attribute *ic_ops_attrs[] = {
	&dev_attr_game_state.attr,
	&dev_attr_high_rate.attr,
	&dev_attr_idle_time.attr,
	NULL,
};

static struct attribute_group ic_ops_attr_group = {
	.name   = "ic_ops",
	.attrs  = ic_ops_attrs,
};


const struct attribute_group *vts_class_groups[] = {
	&frames_attr_group,
	&rom_attr_group,
	&status_attr_group,
	&property_attr_group,
	&vts_attr_group,
	&touch_aoi_attr_group,
	&ic_ops_attr_group,
	NULL,
};

static int vts_dev_suspend(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "suspended\n");
	vts_device_lock(vtsdev);
	return 0;
}

static int vts_dev_resume(struct device *dev)
{
	struct vts_device *vtsdev = dev_get_drvdata(dev);

	vts_dev_info(vtsdev, "resumed\n");
	vts_device_unlock(vtsdev);
	return 0;
}

struct dev_pm_ops vts_dev_pm = {
	.suspend = vts_dev_suspend,
	.resume = vts_dev_resume,
};

static int vts_class_init(void)
{
	int ret = 0;

	if(vts_class)
		return 0;

	vts_class = class_create(THIS_MODULE, "vts");
	if (IS_ERR_OR_NULL(vts_class)) {
		VTE("create class vts failed ret = %ld\n", PTR_ERR(vts_class));
		ret = PTR_ERR(vts_class);
		vts_class = NULL;
		return ret;
	}
	vts_class->dev_groups = vts_class_groups;
	vts_class->pm = &vts_dev_pm;
	return 0;
}

static void vts_class_exit(void)
{
	if (!vts_class)
		return ;

	class_destroy(vts_class);
	vts_class = NULL;
}

int vts_classdev_register(struct device *parent,
				struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ret;
	}

	if (atomic_inc_return(&nr_devices) == 1) {
		ret = vts_class_init();
		if (ret)
			return ret;
	}
	
	vtsdev->dev = device_create(vts_class, parent, val,
				      vtsdev, "%s", vts_name(vtsdev));
	if (IS_ERR(vtsdev->dev)) {
		vts_dev_err(vtsdev, "device create failed! ret = %ld\n", PTR_ERR(vtsdev->dev));
		if (atomic_dec_return(&nr_devices) == 0)
			vts_class_exit();
		return PTR_ERR(vtsdev->dev);
	}

	vts_dev_info(vtsdev, "class device registered\n");
	return 0;
}

void vts_classdev_unregister(struct vts_device *vtsdev)
{
	int ret;
	u32 val;

	ret = vts_property_get(vtsdev, VTS_PROPERTY_PANEL_TYPE, &val);
	if (ret) {
		vts_dev_err(vtsdev, "get panle type error, ret = %d\n", ret);
		return ;
	}

	device_destroy(vts_class, val);
	vts_dev_info(vtsdev, "class device unregistered\n");
	if (atomic_dec_return(&nr_devices) == 0)
		vts_class_exit();
}

