/*
 * Goodix Gesture Module
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include "goodix_ts_core.h"

#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

#define GSX_GESTURE_TYPE_LEN	32
#define GSX_INFO_HEAD_LEN	42
#define GSX_BUFFER_DATA_LEN	514

/*
 * struct gesture_module - gesture module data
 * @registered: module register state
 * @sysfs_node_created: sysfs node state
 * @gesture_type: valid gesture type, each bit represent one gesture type
 * @gesture_data: store latest gesture code get from irq event
 * @gesture_ts_cmd: gesture command data
 */
struct gesture_module {
	atomic_t registered;
	rwlock_t rwlock;
	u8 gesture_type[GSX_GESTURE_TYPE_LEN];
	u8 gesture_buffer_data[GSX_BUFFER_DATA_LEN];
	u8 gesture_data;
	struct goodix_ext_module module;
	/*Jarvis 2020-7-8*/
	u8 gesture_switch[2];
};

static struct gesture_module *gsx_gesture; /*allocated in gesture init module*/

/**
 * gsx_gesture_type_show - show valid gesture type
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to output buffer
 * Returns >=0 - succeed,< 0 - failed
 */
static ssize_t gsx_gesture_type_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = 0, i, ret = 0;
	unsigned char *type;

	type = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!type)
		return -ENOMEM;
	read_lock(&gsx_gesture->rwlock);
	for (i = 0; i < 256; i++) {
		if (QUERYBIT(gsx_gesture->gesture_type, i)) {
			count += scnprintf(type + count,
					   PAGE_SIZE, "%02x,", i);
		}
	}
	if (count > 0)
		ret = scnprintf(buf, PAGE_SIZE, "%s\n", type);
	read_unlock(&gsx_gesture->rwlock);

	kfree(type);
	return ret;
}

/**
 * gsx_gesture_type_store - set vailed gesture
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to valid gesture type
 * @count: length of buf
 * Returns >0 - valid gestures, < 0 - failed
 */
static ssize_t gsx_gesture_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	int i;

	if (count <= 0 || count > 256 || buf == NULL) {
		ts_err("Parameter error");
		return -EINVAL;
	}

	write_lock(&gsx_gesture->rwlock);
	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	for (i = 0; i < count; i++)
		gsx_gesture->gesture_type[buf[i]/8] |= (0x1 << buf[i]%8);
	write_unlock(&gsx_gesture->rwlock);

	return count;
}

static ssize_t gsx_gesture_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&gsx_gesture->registered));
}

static ssize_t gsx_gesture_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	bool val;
	int ret;

	ret = strtobool(buf, &val);
	if (ret < 0)
		return ret;

	if (val) {
		ret = gt9897_register_ext_module_no_wait(&gsx_gesture->module);
		return ret ? ret : count;
	} else {
		ret = gt9897_unregister_ext_module(&gsx_gesture->module);
		return ret ? ret : count;
	}
}

static ssize_t gsx_gesture_data_show(struct goodix_ext_module *module,
				char *buf)
{
	ssize_t count;

	read_lock(&gsx_gesture->rwlock);
	count = scnprintf(buf, PAGE_SIZE, "gesture type code:0x%x\n",
			  gsx_gesture->gesture_data);
	read_unlock(&gsx_gesture->rwlock);

	return count;
}

const struct goodix_ext_attribute gt9897_ges_attrs[] = {
	__EXTMOD_ATTR(type, 0666, gsx_gesture_type_show,
		gsx_gesture_type_store),
	__EXTMOD_ATTR(enable, 0666, gsx_gesture_enable_show,
		gsx_gesture_enable_store),
	__EXTMOD_ATTR(data, 0444, gsx_gesture_data_show, NULL)
};

static int gsx_gesture_init(struct goodix_ts_core *cd,
		struct goodix_ext_module *module)
{
	int i, ret = -EINVAL;

	if (!cd || !cd->hw_ops->gesture) {
		ts_err("gesture unsupported");
		return -EINVAL;
	}

	if (atomic_read(&gsx_gesture->registered))
		return 0;

	ts_debug("enable all gesture type");
	/* set all bit to 1 to enable all gesture wakeup */
	memset(gsx_gesture->gesture_type, 0xff, GSX_GESTURE_TYPE_LEN);
	memset(gsx_gesture->gesture_buffer_data, 0xff, GSX_BUFFER_DATA_LEN);
	module->priv_data = cd;

	ret = kobject_init_and_add(&module->kobj, gt9897_get_default_ktype(),
			&cd->pdev->dev.kobj, "gesture");
	if (ret) {
		ts_err("failed create gesture sysfs node!");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(gt9897_ges_attrs) && !ret; i++)
		ret = sysfs_create_file(&module->kobj, &gt9897_ges_attrs[i].attr);
	if (ret) {
		ts_err("failed create gst sysfs files");
		while (--i >= 0)
			sysfs_remove_file(&module->kobj, &gt9897_ges_attrs[i].attr);

		kobject_put(&module->kobj);
		return ret;
	}

	atomic_set(&gsx_gesture->registered, 1);
	return 0;
}

static int gsx_gesture_exit(struct goodix_ts_core *cd,
		struct goodix_ext_module *module)
{
	return 0;
}
int bbk_xxx_gesture_point_get(struct goodix_ts_core *core_data, u16 *data)
{
	int i;
	u8 gesture_type = gsx_gesture->gesture_data;
	int ges_num = 0;
	u16 coordinate_x[9];
	u16 coordinate_y[9];

	switch (gesture_type) {
	case 0xBA:
	case 0xAB:
		ges_num = 2;
		data[0] = (gsx_gesture->gesture_buffer_data[1] << 8) | gsx_gesture->gesture_buffer_data[0];
		data[1] = (gsx_gesture->gesture_buffer_data[3] << 8) | gsx_gesture->gesture_buffer_data[2];
		data[2] = (gsx_gesture->gesture_buffer_data[5] << 8) | gsx_gesture->gesture_buffer_data[4];
		data[3] = (gsx_gesture->gesture_buffer_data[7] << 8) | gsx_gesture->gesture_buffer_data[6];
		break;
	case 0x65:
	case 0x40:
	case 0x66:
	case 0x6F:
		ges_num = 6;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture->gesture_buffer_data[2 * i];
		break;
	case 0x77:
		ges_num = 5;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture->gesture_buffer_data[2 * i];
		break;
	case 0x63:
		ges_num = 6;
		for (i = 0; i < 10; i++)
			data[i] = (gsx_gesture->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture->gesture_buffer_data[2 * i];

		data[10] = data[8];
		data[11] = data[9];
		break;
	default:
		ges_num = 0;
		break;
	}

	//for (i = 0; i < 12; i++)
	//	ts_info("data[i]:%d", data[i]);
	
	for (i = 0; i < ges_num; i++){
		coordinate_x[i] = data[2*i] ;
		coordinate_y[i] = data[2*i+1] ;
		VTI("x[i]:%d, y[i]:%d", data[2*i], data[2*i+1]);
	}
	vts_report_coordinates_set(core_data->vtsdev, coordinate_x, coordinate_y, ges_num); 

	return ges_num;
}

/**
 * gsx_gesture_ist - Gesture Irq handle
 * This functions is excuted when interrupt happended and
 * ic in doze mode.
 *
 * @cd: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_CANCEL_IRQEVT  stop execute
 */
static int gsx_gesture_ist(struct goodix_ts_core *cd,
	struct goodix_ext_module *module)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ts_event gs_event = {0};
	struct vts_device *vtsdev = cd->vtsdev;
	u8 tmp_buf[GSX_INFO_HEAD_LEN + GSX_BUFFER_DATA_LEN] = {0};
	int ges_buffer_len = 0;
	int ret;
	/*Jarvis 2020-7-8*/
	bool skip = 0;
	int x,y;
	u16 gestur_point[30];
	int resolution_adjust = 0;
	int dimension_x, dimension_y;
	int display_x, display_y;
	
	x=0;
	y =0;
	if (atomic_read(&cd->gestured) == 0)
		return EVT_CONTINUE;
	VTD("ENTER gsx_gesture_ist");
	ret = hw_ops->event_handler(cd, &gs_event);
	if (ret) {
		ts_err("failed get gesture data");
		goto re_send_ges_cmd;
	}
	
	if (!(gs_event.event_type & EVENT_GESTURE)) {
		ts_err("invalid event type: 0x%x",
			cd->ts_event.event_type);
		goto re_send_ges_cmd;
	}
	VTI("gesture type:0x%x", gs_event.gesture_type);
	ret = hw_ops->read(cd, misc->touch_data_addr,
			   tmp_buf, sizeof(tmp_buf));
	if (ret) {
		ts_err("failed read gesture point trach data");
		goto re_send_ges_cmd;
	}

	if (checksum_cmp(tmp_buf, 8, CHECKSUM_MODE_U8_LE)) {
		ts_err("failed gesture data head check failed");
		ts_err("%*ph", 8, tmp_buf);
		goto re_send_ges_cmd;
	}
	ges_buffer_len = tmp_buf[3] * 4 + 2;
	if (ges_buffer_len > sizeof(tmp_buf) - GSX_INFO_HEAD_LEN) {
		ts_err("gesture track point data len exceed limit %d", ges_buffer_len);
		goto re_send_ges_cmd;
	}
	if (checksum_cmp(&tmp_buf[GSX_INFO_HEAD_LEN],
			 ges_buffer_len, CHECKSUM_MODE_U8_LE)) {
		ts_err("gesture track point data checksum err, %d", ges_buffer_len);
		goto re_send_ges_cmd;
	}
	memcpy(gsx_gesture->gesture_buffer_data, &tmp_buf[GSX_INFO_HEAD_LEN], ges_buffer_len);
	

	if (QUERYBIT(gsx_gesture->gesture_type,
		     gs_event.gesture_type)) {
		gsx_gesture->gesture_data = gs_event.gesture_type;
		/* do resume routine */
		ts_info("got valid gesture type 0x%x",
			gs_event.gesture_type);
/*Jarvis 2020-7-8 end*/
#if 1	
		bbk_xxx_gesture_point_get(cd, gestur_point);

		/* do resume routine */
		VTI("Gesture match success, resume IC");
		{	/*"c" e w m*/
			VTI("gesture type:0x%x", gs_event.gesture_type);
			if (gs_event.gesture_type == 0xcc) {
				vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution_adjust);
				if (resolution_adjust) {
					vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimension_x);
					vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimension_y);
					vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
					vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
					x = (((gsx_gesture->gesture_buffer_data[1] << 8) | gsx_gesture->gesture_buffer_data[0]) * display_x) / dimension_x;
					y = (((gsx_gesture->gesture_buffer_data[3] << 8) | gsx_gesture->gesture_buffer_data[2]) * display_y) / dimension_y;
				} else {
					x = (gsx_gesture->gesture_buffer_data[1] << 8) | gsx_gesture->gesture_buffer_data[0];
					y = (gsx_gesture->gesture_buffer_data[3] << 8) | gsx_gesture->gesture_buffer_data[2];
				}
				VTI("DBLICK POINT X = %d, Y= %d", x, y);
				vts_update_dclick_point(vtsdev, x, y);
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_DOUBLE_CLICK);
			}	
			if (gs_event.gesture_type == 0x63) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_C);
			}
			if (gs_event.gesture_type == 0x65) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_E);
			}
			if (gs_event.gesture_type == 0x6d) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_M);
			}
			if (gs_event.gesture_type == 0x77) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_W);
			}
			if (gs_event.gesture_type == 0x40) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_A);
			}
			if (gs_event.gesture_type == 0x66) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_F);
			}
			if (gs_event.gesture_type == 0x6f) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_O);
			}
			if (gs_event.gesture_type == 0xaa) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_RIGHT);
			}
			if (gs_event.gesture_type == 0xbb) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_LEFT);
			}
			if (gs_event.gesture_type == 0xba) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_UP);
			}
			if (gs_event.gesture_type == 0xab) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_PATTERN_DOWN);
			}
			if (gs_event.gesture_type == 0x46) {
				vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
				skip = 1;
			}
			if (gs_event.gesture_type == 0x55) {
				vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
				skip = 1;
			}
		}	
#else
		input_report_key(cd->input_dev, KEY_POWER, 1);
		input_sync(cd->input_dev);
		input_report_key(cd->input_dev, KEY_POWER, 0);
		input_sync(cd->input_dev);
#endif
/*Jarvis 2020-7-8 end*/
//		goto gesture_ist_exit;
	} else {
		ts_info("unsupported gesture:%x", gs_event.gesture_type);
	}

re_send_ges_cmd:
	if (!skip) {
		if (hw_ops->gesture(cd, 0))
			ts_info("warning: failed re_send gesture cmd\n");
	}
	if (!cd->tools_ctrl_sync)
		hw_ops->after_event_handler(cd);
	return EVT_CANCEL_IRQEVT;
}

/**
 * gsx_gesture_before_suspend - execute gesture suspend routine
 * This functions is excuted to set ic into doze mode
 *
 * @cd: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_IRQCANCLED  stop execute
 */
static int gsx_gesture_before_suspend(struct goodix_ts_core *cd,
	struct goodix_ext_module *module)
{
	int ret;
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	// atomic_set(&cd->suspended, 1);
	ret = hw_ops->gesture(cd, 0);
	if (ret) {
		ts_err("failed enter gesture mode");
	} else {
		ts_info("enter gesture mode");
		atomic_set(&cd->suspended, 0);
		atomic_set(&cd->gestured, 1);
	}

	return EVT_CANCEL_SUSPEND;
}

static struct goodix_ext_module_funcs gsx_gesture_funcs = {
	.irq_event = gsx_gesture_ist,
	.init = gsx_gesture_init,
	.exit = gsx_gesture_exit,
	.before_suspend = gsx_gesture_before_suspend
};

int gt9897_gsx_gesture_init(void)
{
	ts_info("gesture module init");
	gsx_gesture = kzalloc(sizeof(struct gesture_module), GFP_KERNEL);
	if (!gsx_gesture)
		return -ENOMEM;
	gsx_gesture->module.funcs = &gsx_gesture_funcs;
	gsx_gesture->module.priority = EXTMOD_PRIO_GESTURE;
	gsx_gesture->module.name = "Goodix_gsx_gesture";
	gsx_gesture->module.priv_data = gsx_gesture;

	atomic_set(&gsx_gesture->registered, 0);
	rwlock_init(&gsx_gesture->rwlock);
	gt9897_register_ext_module(&(gsx_gesture->module));
	return 0;
}

void  gt9897_gsx_gesture_exit(void)
{
	int i;

	ts_info("gesture module exit");
	if (atomic_read(&gsx_gesture->registered)) {
		gt9897_unregister_ext_module(&gsx_gesture->module);
		atomic_set(&gsx_gesture->registered, 0);

		for (i = 0; i < ARRAY_SIZE(gt9897_ges_attrs); i++)
			sysfs_remove_file(&gsx_gesture->module.kobj,
					  &gt9897_ges_attrs[i].attr);

		kobject_put(&gsx_gesture->module.kobj);
	}

	kfree(gsx_gesture);
}

