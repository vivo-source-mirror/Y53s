/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2016, STMicroelectronics Limited.
 * Authors: AMG(Analog Mems Group)
 *ftm4_edge_restain_switch
 * 		marco.cali@st.com
 *
 * This program is free software; you can redistribute it and / or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/timekeeping.h>
#include <linux/ktime.h>

#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsGesture.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"

#include "../vts_core.h"

#define LINK_KOBJ_NAME "tp"

struct regulator;
struct regulator_dev;
struct fts_ts_info;

#define event_id(_e)     EVENTID_##_e
#define handler_name(_h) fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd) \
do { \
		_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd); \
} while (0)


/*
 * Asyncronouns command helper
 */
#define WAIT_WITH_TIMEOUT(_info, _timeout, _command) \
do { \
		if (wait_for_completion_timeout(&_info->cmd_done, _timeout) == 0) { \
				dev_warn(_info->dev, "Waiting for %s command: timeout\n", \
				#_command); \
		} \
} while (0)

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

static int fts_init_hw(struct fts_ts_info *info);
static int fts_mode_handler(struct fts_ts_info *info, int force);
static int fts_command(struct fts_ts_info *info, unsigned char cmd);
static int fts_power_on(struct fts_ts_info *info, bool onoff);
void ftm4_reset_pin_ctl(struct fts_ts_info *info, int state);


static int ftm4_irq_enable_disable(struct fts_ts_info *info, int ctl, int nosync_choose)
{
	if (info == NULL) {
		fts_err(info, "info is NULL");
		return 0;
	}
	
	if (ctl == 1 && info->irq_state == 0) {
		info->irq_state = 1;
		enable_irq(info->client->irq);
		fts_info(info, "do irq enable");
	} else if (ctl == 0 && info->irq_state == 1) {
		info->irq_state = 0;
		if (nosync_choose == 1) {
			disable_irq_nosync(info->client->irq);
		} else if (nosync_choose == 0) {
			disable_irq(info->client->irq);
		}
		fts_info(info, "do irq diaable");
	} else {
		fts_info(info, "do irq nothing.irq_state:%d ctl:%d", info->irq_state, ctl);
	}

	return 0;
}
static int fts_chip_initialization(struct fts_ts_info *info);
unsigned int ftm4_le_to_uint(const unsigned char *ptr)
{
	return (unsigned int) ptr[0] + (unsigned int) ptr[1] * 0x100;
}

unsigned int ftm4_be_to_uint(const unsigned char *ptr)
{
	return (unsigned int) ptr[1] + (unsigned int) ptr[0] * 0x100;
}

static int fts_command(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd;
	int ret;

	regAdd = cmd;

	ret = ftm4_writeCmd(info, &regAdd, sizeof (regAdd)); /* 0 = ok */

	fts_dbg(info, "Issued command 0x%02x, return value %08X", cmd, ret);

	return ret;
}

static inline unsigned char *fts_next_event(unsigned char *evt)
{
	/* Nothing to do with this event, moving to the next one */
	evt += FIFO_EVENT_SIZE;

	/* the previous one was the last event ?  */
	return (evt[-1] & 0x1F) ? evt : NULL;
}

/* EventId : 0x00 */
static unsigned char *fts_nop_event_handler(struct fts_ts_info *info,
		unsigned char *event, ktime_t kt)
{
	fts_info(info, "Doing nothing for event = %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
	return fts_next_event(event);
}

static int get_ear_detect_result(struct fts_ts_info *info, u32 index)
{
	int ret;
	int event_to_search[2] = {0xC1,0x0E};
	u8 cmd[6];
	u8 readEvent[FIFO_EVENT_SIZE];
	cmd[0] = 0xCD;
	cmd[1] = 0x0E;

	fts_u32ToU8(index, &cmd[2]);

	fts_info(info, "the index to set is %ud ", index);

	//fts_disableInterrupt();
	if (ftm4_writeCmd(info, cmd, 6) <0) {
			fts_info(info, "fts_disableInterrupt: ERROR!");
			return ERROR_I2C_W;
		}
	
	fts_info(info, "write cmd CD 0E finished! ");
		ret = ftm4_pollForEvent(info, event_to_search, 2, &readEvent[0], GENERAL_TIMEOUT); //start the polling for reading the reply
		if(ret < 0) {
			fts_info(info, "Can not found the event 0xC1 0x0E");
			return ret;
		}
	fts_info(info, "has found the event 0xC1 0x0E! ");
	//ftm4_enableInterrupt();
	return 0;

}

static unsigned char *fts_ear_detect_event_handler(struct fts_ts_info *info,
        unsigned char *event, ktime_t kt)
{
	unsigned char touchId;
	int value;

	int pick[3] = {0};
	int down[3] = {0};
	down[0] = 1;

	fts_info(info, "ear_detect data: %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
	/* always use touchId zero */
	touchId = 0;
	//__set_bit(touchId, &info->touch_id);

	if (event[1] == 0xA1) {	
		fts_info(info, "---------------------TOUCHICALG_MOTION_PICK");
		value = VTS_EVENT_GESTURE_DOUBLE_CLICK;
		vts_proxminity_report(info->vtsdev, down[0], down[1], down[2]);
		fts_info(info, "report the param---1");

	} else if(event[1] == 0xA3) {
		fts_info(info, "---------------------TOUCHICALG_MOTION_DOWN_TIMEOVER");
		value = VTS_EVENT_GESTURE_DOUBLE_CLICK;
	if (0) {
		get_ear_detect_result(info, 1);
	}
		vts_proxminity_report(info->vtsdev, pick[0], pick[1], pick[2]);
		fts_info(info, "report the param---0");
			
	} else {
		fts_info(info, "---------------------OTHER EVENT");
		goto END;
	}

	//vivoTsInputReport(VTS_GESTURE_EVENT, value, -1, -1, -1);
END:
	//__clear_bit(touchId, &info->touch_id);
	return fts_next_event(event);
}

static unsigned char *fts_ear_debug_event_handler(struct fts_ts_info *info,
		unsigned char *event, ktime_t kt)
{

	fts_info(info, "-------------ear_debug event: %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);


	return fts_next_event(event);

}

static void fts_ic_status(struct fts_ts_info *info, u8 data)
{
	int ic_status = 0;

	if (data & BIT(0))
		ic_status |= VTS_IC_STATUS_WATER_MODE;

	if (data & BIT(1))
		ic_status |= VTS_IC_STATUS_NOISE_MODE;

	if (data & BIT(2))
		ic_status |= VTS_IC_STATUS_DOZE_MODE;

	if (data & BIT(3))
		ic_status |= VTS_IC_STATUS_GESTURE_MODE;

	if (data & BIT(4))
		ic_status |= VTS_IC_STATUS_ACTIVE_MODE;

	if (data & BIT(5))
		ic_status |= VTS_IC_STATUS_BATTLE_MODE;

	ic_status |= (VTS_IC_STATUS_SCAN_FREQ_INDEX & (BIT(6) | BIT(7)));
	vts_report_ic_status(info->vtsdev, ic_status);
}

/* EventId : 0x03 */
static unsigned char *fts_enter_pointer_event_handler(struct fts_ts_info *info, unsigned char *event, ktime_t kt)
{
	unsigned char touchId, touchcount;
	int x, y, z;
	u8 touchsize;
	if (!info->resume_bit)
	    goto no_report;

	touchId = event[1] & 0x0F;
	touchcount = (event[1] & 0xF0) >> 4;
	touchsize = (event[5] & 0xC0) >> 6;

	fts_dbg(info, "touchsize=%d touchcount=%d touchId=%d", touchsize, touchcount, touchId);
	/*this is a palm touch, no report! sunshal */

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0x3F);
	fts_ic_status(info, event[6]);

	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;

	vts_report_point_down(info->vtsdev, touchId, touchcount, x,  y, z, z, touchsize == 3, NULL, 0, kt);
	fts_dbg(info, "Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)", *event, touchId, x, y, z);

no_report:
	return fts_next_event(event);
}

/* EventId : 0x04 */
static unsigned char *fts_leave_pointer_event_handler(struct fts_ts_info *info,
		unsigned char *event, ktime_t kt)
{
	unsigned char touchId, touchcount;
	int x, y, z;

	touchId = event[1] & 0x0F;
	touchcount = (event[1] & 0xF0) >> 4;

	fts_dbg(info, "leave point touchcount=%d", touchcount);
	__clear_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0x3F);


	if (x == X_AXIS_MAX)
	    x--;

	if (y == Y_AXIS_MAX)
	    y--;

	vts_report_point_up(info->vtsdev, touchId, touchcount, x, y, z, z, false, kt);
	return fts_next_event(event);
}

/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler

#ifdef PHONE_KEY
/* EventId : 0x0E */
/*sunsl
static unsigned char *fts_key_status_event_handler(struct fts_ts_info *info, unsigned char *event) {
	int value;
	fts_dbg(info, "Received event %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
	//TODO: the customer should handle the events coming from the keys according his needs (this is an example that report only the single pressure of one key at time)
	if (event[2] != 0) {			//event[2] contain the bitmask of the keys that are actually pressed
		switch (event[2]) {
			case KEY1:
				value = KEY_HOMEPAGE;
				fts_info(info, "Button HOME !");
			break;

			case KEY2:
				value = KEY_BACK;
				fts_info(info, "Button Back !");
				break;

			case KEY3:
				value = KEY_MENU;
				fts_info(info, "Button Menu !");
				break;

			default:
				fts_info(info, " No valid Button ID or more than one key pressed!");
				goto done;
		}

	//sunsl		fts_input_report_key(info, value);
	} else {
		fts_dbg(info, "All buttons released!");
	} 
done:
	return fts_next_event(event);
}
 */
#endif

static int do_fts_esd_error_thread(struct fts_ts_info *info)
{
	int error = 0;

	mutex_lock(&(info->esdErrorMutex));

	fts_info(info, "disable ftm4 backscreen.");

	if (!(info->gesture_enabled == 1 && info->mode == MODE_GESTURE)) {
		fts_info(info, "not in gesture mode, not disable.");
		mutex_unlock(&(info->esdErrorMutex));
		error = -1;
		return error;
	}

	mutex_lock(&(info->i2cResetMutex));
	error = fts_power_on(info, 0);
	if (error)
		fts_err(info, "fts_power_off error.");
	ftm4_reset_pin_ctl(info, 0);
	msleep(10);
	mutex_unlock(&(info->i2cResetMutex));
	ftm4_irq_enable_disable(info, 0, 1);

	mutex_unlock(&(info->esdErrorMutex));

	return error;
}

static int fts_esd_error_thread(void *dev)
{
	struct fts_ts_info *info = (struct fts_ts_info *)dev;
	static struct sched_param para = {
		.sched_priority = 40,
	};

	init_waitqueue_head(&info->wait);
	sched_setscheduler(current, SCHED_FIFO, &para);
	while (likely(!kthread_should_stop()))  {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(info->wait, info->wait_cond);

		__pm_stay_awake(info->error_wlock);
		info->wait_cond = false;
		set_current_state(TASK_RUNNING);
		do_fts_esd_error_thread(info);

		__pm_relax(info->error_wlock);
	}

	vts_wakelock_unregister(info->error_wlock);
	info->error_wlock = NULL;
	return 0;
}

static int fts_check_vaild_tiemstamp(struct fts_ts_info *info)
{
	int error = 0;

	switch (++(info->esd_error_count)) {
	case 1:
		info->ktime_start = (int)ktime_to_ms(ktime_get());
		break;
	case 2:
		info->ktime_middl = (int)ktime_to_ms(ktime_get());
		if ((info->ktime_middl - info->ktime_start) > 3000) {
			fts_info(info, "ktime_middl - ktime_start = %d", info->ktime_middl - info->ktime_start);
			info->ktime_start = info->ktime_middl;
			info->esd_error_count = 1;
			error = -1;
		}
		break;
	case 3:
		info->ktime_endof = (int)ktime_to_ms(ktime_get());
		if ((info->ktime_endof - info->ktime_middl) > 3000) {
			fts_info(info, "ktime_endof - ktime_middl = %d", info->ktime_endof - info->ktime_middl);
			info->ktime_start = info->ktime_endof;
			info->esd_error_count = 1;
			error = -1;
		}
		break;
	default:
		fts_info(info, "invaild count, need again get timestamp.");
		info->esd_error_count = 0;
		error = -1;
		break;
	}

	return error;
}

static void fts_esd_error_disable(struct fts_ts_info *info)
{
	int error = 0;

	if (!(info->gesture_enabled == 1 && info->mode == MODE_GESTURE)) {
		fts_info(info, "not in gesture mode ignoring esd error.");
		info->esd_error_count = 0;
		return;
	}

	error = fts_check_vaild_tiemstamp(info);
	if (error < OK) {
		fts_err(info, "timestamp is abnormal, need again.");
		return;
	}

	if (info->esd_error_count > 2) {
		fts_info(info, "After 3 times esd retry disable host interrupt.");
		info->esd_error_count = 0;
		info->wait_cond = true;
		info->esd_error_state = 1;
		wake_up_interruptible(&info->wait);
	}
}

int vts_esd_error_thread_register(struct fts_ts_info *info)
{
	struct vts_device *vtsdev = info->vtsdev;
	init_waitqueue_head(&info->wait);

	snprintf(info->wakelock_name, sizeof(info->wakelock_name), "error_esd_%d", vtsdev->type);
	info->error_wlock = vts_wakelock_register(vtsdev, info->wakelock_name);
	if (!info->error_wlock) {
		VTE("error_wlock init fail");
		return -ENOMEM;
	}

	info->task = kthread_run(fts_esd_error_thread, info, "esd_thread_%d", vtsdev->type);
	if (IS_ERR(info->task)) {
		VTE("create irq task error!, %s", "error_esd");
		vts_wakelock_unregister(info->error_wlock);
		info->error_wlock = NULL;
		return PTR_ERR(info->task);
	}

	return 0;
}

/* EventId : 0x0F */
static unsigned char *fts_error_event_handler(struct fts_ts_info *info,
		unsigned char *event, ktime_t kt)
{
	int error = 0;
	/*  fts_info(info, "Received event 0x%02x 0x%02x", event[0], event[1]); */
	fts_info(info, "Received event = %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);

	switch (event[1]) {
	case EVENT_TYPE_ESD_ERROR: /*esd */
		fts_err(info, "esd error! power reset and resume");

		mutex_lock(&(info->esdErrorMutex));
		ftm4_irq_enable_disable(info, 0, 1);
		ftm4_chip_powercycle(info);
		error = ftm4_system_reset(info);
		error |= fts_mode_handler(info, 0);
		error |= ftm4_enableInterrupt(info);
		ftm4_irq_enable_disable(info, 1, 0);
		if (error < OK) {
			fts_err(info, "Cannot restore the device ERROR %08X", error);
		}

		if (info->broken_disable)
			fts_esd_error_disable(info);

		mutex_unlock(&(info->esdErrorMutex));
		break;
	case EVENT_TYPE_WATCHDOG_ERROR: /*watch dog timer */
		if (event[2] == 0) {
			error = ftm4_system_reset(info);
			error |= fts_mode_handler(info, 0);
			error |= ftm4_enableInterrupt(info);
			if (error < OK) {
				fts_info(info, "Cannot reset the device ERROR %08X", error);
			}
		}
		break;
	}
	return fts_next_event(event);
}

/* EventId : 0x10 */
static unsigned char *fts_controller_ready_event_handler(
		struct fts_ts_info *info, unsigned char *event, ktime_t kt)
{
	int error = 0;
	fts_info(info, "Received event 0x%02x", event[0]);
	info->touch_id = 0;
   /*sunsl input_sync(info->input_dev); */
   /*
	setSystemResettedUp(1);
	setSystemResettedDown(1); */
	error = fts_mode_handler(info, 0);
	if (error < OK) {
		  fts_err(info, "Cannot restore the device status ERROR %08X", error);
	}
	return fts_next_event(event);
}

/* EventId : 0x16 */
static unsigned char *fts_status_event_handler(
		struct fts_ts_info *info, unsigned char *event, ktime_t kt)
{
	switch (event[1]) {
	case EVENT_TYPE_MS_TUNING_CMPL:
	case EVENT_TYPE_SS_TUNING_CMPL:
	case FTS_FORCE_CAL_SELF_MUTUAL:
	case FTS_FLASH_WRITE_CONFIG:
	case FTS_FLASH_WRITE_COMP_MEMORY:
	case FTS_FORCE_CAL_SELF:
	case FTS_WATER_MODE_ON:
	case FTS_WATER_MODE_OFF:
	default:
		fts_info(info, "Received unhandled status event = %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
		break;
	}

	return fts_next_event(event);
}

#ifdef PHONE_GESTURE
static int fts_get_gesture_coordinates(struct fts_ts_info *info, u8 *event, u16 *point_x, u16 *point_y, size_t size, int *nr_points)
{

	int i = 0;
	u8 rCmd[3] = {FTS_CMD_FRAMEBUFFER_R, 0x00, 0x00 };
	int len = 0;
	  int error = OK;

	unsigned char val[130];
	/*u16 x_min = 65535; */
	/*u16 x_max = 0; */
	/*u16 y_min = 65535; */
	/*u16 y_max = 0; */


	if (event[1] == 0x02) {


		rCmd[1] = event[4];    /* Ofset address \A1\C1?\A1\C0\A8\BA??8 */
		rCmd[2] = event[3];    /* Ofset address  \A1\C1?\A1\C0\A8\BA\A6?\AA8 */
		len = event[6];/*\A6\CC?\A8\BAy??8 */
		len = (len << 8) | event[5]; /*\A6\CC?\A8\BAy\A6?\AA8 */
		*nr_points = len;
		if (*nr_points > size) {

			*nr_points = size;
		}
		error = ftm4_readCmd(info, rCmd, 3, (unsigned char *)val, 1 + (len << 2));
		if (error < OK) {
			fts_dbg(info, "Cannot read reg!%08X", error);
			return error;
		}

		fts_info(info, "FTS fts gesture coordinate address : %02X %02X %02X len = %02X", rCmd[0], rCmd[1], rCmd[2], len); /* add by terry 150312 for default enable user defind gesture */
		/*all the points of the gesture in the val */
		if (event[2] != 0x11) {	/*default gesture */
			for (i = 0; i < *nr_points; i++) {	/*ignore first byte data because it is a harware commond data */
				point_x[i] =  (((u16) val[i * 2 + 1 + 1]) & 0x0F) << 8 | (((u16) val[i * 2 + 1]) & 0xFF);
				point_y[i] =  (((u16) val[*nr_points * 2 + i * 2 + 1 + 1]) & 0x0F) << 8 | (((u16) val[*nr_points * 2 + i * 2 + 1]) & 0xFF);

				/*if (coordinate_x[i] < x_min)
					x_min = gesture_point[i];
				if (coordinate_x[i] > x_max)
					x_max = gesture_point[i];
				if (coordinate_y[i] < y_min)
					y_min = gesture_point[i];
				if (coordinate_y[i] > y_max)
					y_max = gesture_point[i];
				printk("gmz FTS pointnum = %d  coordinate_x[%d]=%d	coordinate_y[%d]=%d \n", ftm4_point_num, i, coordinate_x[i], i, coordinate_y[i]); */
			}
			*nr_points = 10;
			for (i = 0; i < *nr_points; i++) {
				point_x[i] = point_x[3 * i];
				point_y[i] = point_y[3 * i];
			}


		} else {
		/*	printk("Get Customer  define gesture coordinate\n"); */
			fts_dbg(info, "do not get coordinate!");

		}
	}
	return OK;

}
static unsigned char *fts_gesture_event_handler(struct fts_ts_info *info, unsigned char *event, ktime_t kt)
{
	unsigned char touchId = 0;
	int value = 0;
	bool get_coord = true;
	int finger_gesture_type = 0;

	fts_info(info, "gesture event data: %02X %02X %02X %02X %02X %02X %02X %02X", event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
	if (event[1] == 0x03) {
		fts_info(info, "Gesture ID %02X enable_status = %02X", event[2], event[3]);
	}
	if (event[1] == EVENT_TYPE_ENB && event[2] == 0x00) {
		switch (event[3]) {
		case GESTURE_ENABLE:
			fts_info(info, "Gesture Enabled! res = %02X", event[4]);
			break;

		case GESTURE_DISABLE:
			fts_info(info, "Gesture Disabled! res = %02X", event[4]);
			break;

		default:
			fts_info(info, "Event not Valid");
		}
	}

	/* always use touchId zero */
	touchId = 0;
	__set_bit(touchId, &info->touch_id);

	if (event[0] == EVENTID_GESTURE && (event[1] == EVENT_TYPE_GESTURE_DTC1 || event[1] == EVENT_TYPE_GESTURE_DTC2)) {
		switch (event[2]) {
		case GES_ID_DBLTAP:
			value = VTS_EVENT_GESTURE_DOUBLE_CLICK;
			get_coord = false;
			fts_info(info, "double tap !");
		break;

		case GES_ID_AT:
			value = VTS_EVENT_GESTURE_PATTERN_A;
			fts_info(info, " @ !");
			break;

		case GES_ID_C:
			value = VTS_EVENT_GESTURE_PATTERN_C;
			fts_info(info, "C ! ");
			break;

		case GES_ID_E:
			value = VTS_EVENT_GESTURE_PATTERN_E;
			fts_info(info, " e  ");
			break;

		case GES_ID_F:
			value = VTS_EVENT_GESTURE_PATTERN_F;
			fts_info(info, "F !");
			break;

		case GES_ID_L:
			value = KEY_L;
			fts_info(info, "L !");
			break;

		case GES_ID_M:
			value = VTS_EVENT_GESTURE_PATTERN_M;
			fts_info(info, "M !");
			break;

		case GES_ID_O:
			value = VTS_EVENT_GESTURE_PATTERN_O;
			fts_info(info, "O !");
			break;

		case GES_ID_S:
			value = KEY_S;
			fts_info(info, "S !");
			break;

		case GES_ID_V:
			value = VTS_EVENT_GESTURE_PATTERN_V;
			fts_info(info, "V !");
			break;

		case GES_ID_H:
			value = VTS_EVENT_GESTURE_PATTERN_H;
			fts_info(info, "H !");
			break;

		case GES_ID_W:
			value = VTS_EVENT_GESTURE_PATTERN_W;
			fts_info(info, "W !");
			break;

		case GES_ID_Z:
			value = KEY_Z;
			fts_info(info, "Z !");
			break;

		case GES_ID_HFLIP_L2R:
			value = VTS_EVENT_GESTURE_PATTERN_RIGHT;
			fts_info(info, "-> !");
			break;

		case GES_ID_HFLIP_R2L:
			value = VTS_EVENT_GESTURE_PATTERN_LEFT;
			fts_info(info, "<- !");
			break;

		case GES_ID_VFLIP_D2T:
			value = VTS_EVENT_GESTURE_PATTERN_UP;
			fts_info(info, "UP !");
			break;

		case GES_ID_VFLIP_T2D:
			value = VTS_EVENT_GESTURE_PATTERN_DOWN;/*KEY_DOWN; */
			fts_info(info, "DOWN !");
			break;

		case GES_ID_CUST1:
			value = KEY_F1;
			fts_info(info, "F1 !");
			break;

		case GES_ID_CUST2:
			value = KEY_F1;
			fts_info(info, "F2 !");
			break;

		case GES_ID_CUST3:
			value = KEY_F3;
			fts_info(info, "F3 !");
			break;

		case GES_ID_CUST4:
			value = KEY_F1;
			fts_info(info, "F4 !");
			break;

		case GES_ID_CUST5:
			value = KEY_F1;
			fts_info(info, "F5 !");
			break;

		case GES_ID_LEFTBRACE:
			value = KEY_LEFTBRACE;
			fts_info(info, "< !");
			break;

		case GES_ID_RIGHTBRACE:
			value = KEY_RIGHTBRACE;
			fts_info(info, "> !");
			break;
		case GES_ID_LONG_PRESS_DOWN:
			fts_info(info, "finger gesture down");
			value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
			finger_gesture_type = 1;
			get_coord = false;
			break;
		case GES_ID_LONG_PRESS_UP:
			fts_info(info, "finger gesture up");
			value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
			finger_gesture_type = 0;
			get_coord = false;
			break;
		default:
			fts_info(info, "No valid GestureID!");
			goto gesture_done;

		}

		/*fts_input_report_key(info, value); */
		if (get_coord == true) {
			u16 point_x[32];
			u16 point_y[32];
			int nr_points = 0;

			memset(point_x, 0xff, sizeof(point_x));
			memset(point_y, 0xff, sizeof(point_y));
			if(fts_get_gesture_coordinates(info, event, point_x, point_y, ARRAY_SIZE(point_x), &nr_points) == OK)
				vts_report_coordinates_set(info->vtsdev, point_x, point_y, nr_points);
		}

		if (value == VTS_EVENT_GESTURE_FINGERPRINT_DETECT) {
			if (finger_gesture_type == 1) {
				vts_report_event_down(info->vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
			} else if (finger_gesture_type == 0) {
				vts_report_event_up(info->vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
			}
		} else {
			vts_report_event_down(info->vtsdev, value);
			vts_report_event_up(info->vtsdev, value);
		}

	gesture_done:
		/* Done with gesture event, clear bit. */
		__clear_bit(touchId, &info->touch_id);
	}

	return fts_next_event(event);
}
#endif

/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler

/*
 * This handler is called each time there is at least
 * one new event in the FIFO
 */

static int cx_crc_check(struct fts_ts_info *info)
{
	unsigned char regAdd1[3] = {FTS_CMD_HW_REG_R, ADDR_CRC_BYTE0, ADDR_CRC_BYTE1};
	unsigned char val[2];
	unsigned char crc_status;
	int res;
	u8 cmd[4] = { FTS_CMD_HW_REG_W, 0x00, 0x00, SYSTEM_RESET_VALUE };
	int event_to_search[2] = {(int)EVENTID_ERROR_EVENT, (int)EVENT_TYPE_CHECKSUM_ERROR};
	u8 readData[FIFO_EVENT_SIZE];

	res = ftm4_readCmd(info, regAdd1, sizeof (regAdd1), val, 2);/*read 2 bytes because the first one is a dummy byte! */
	if (res < OK) {
		fts_dbg(info, "Cannot read crc status ERROR %08X\n", res);
		return res;
	}

	crc_status = val[1] & CRC_MASK;
	if (crc_status != OK) {
		fts_dbg(info, "CRC ERROR = %X", crc_status);
	return crc_status;
	}

	fts_info(info, "Verifying if Config CRC Error...");
	ftm4_u16ToU8_be(SYSTEM_RESET_ADDRESS, &cmd[1]);
	res = ftm4_writeCmd(info, cmd, 4);
	if (res < OK) {
		fts_dbg(info, "Cannot send system resest command ERROR %08X", res);
		return res;
	}
	/*
	setSystemResettedDown(1);
	setSystemResettedUp(1); */
	res = ftm4_pollForEvent(info, event_to_search, 2, readData, GENERAL_TIMEOUT);
	if (res < OK) {
	fts_dbg(info, "cx_crc_check: No Config CRC Found!");
	} else {
	if (readData[2] == CRC_CONFIG_SIGNATURE || readData[2] == CRC_CONFIG) {
		fts_dbg(info, "cx_crc_check: CRC Error for config found! CRC ERROR = %02X", readData[2]);
		return readData[2];
	}
	}

	return OK; /*return OK if no CRC error, or a number > OK if crc error */
}

static bool fts_need_caliberate(struct fts_ts_info *info)
{
	u8 cmd[4] = { FTS_CMD_HW_REG_W, 0x00, 0x00, SYSTEM_RESET_VALUE };
	int event_to_search[2] = {(int)EVENTID_ERROR_EVENT, (int)EVENT_TYPE_CHECKSUM_ERROR};
	int flag_init = 0;
	u8 readData[FIFO_EVENT_SIZE];
	int ret;

	fts_info(info, "Verifying if CX CRC Error...");
	ftm4_u16ToU8_be(SYSTEM_RESET_ADDRESS, &cmd[1]);
	ret = ftm4_writeCmd(info, cmd, 4);
	if (ret < OK) {
		fts_dbg(info, "Cannot send system reset command ERROR %08X", ret);
	} else {
	/*
		setSystemResettedDown(1);
		setSystemResettedUp(1);
		 */
		ret = ftm4_pollForEvent(info, event_to_search, 2, readData, GENERAL_TIMEOUT);
		if (ret < OK) {
			fts_dbg(info, "No CX CRC Found!");
		} else {
			if (readData[2] == CRC_CX_MEMORY) {
				fts_err(info, "CRC Error for CX found! CRC ERROR = %02X", readData[2]);
				flag_init = 1;
			}
		}
	}

	if (((info->chipinfo.u32_mpPassFlag != INIT_MP) && (info->chipinfo.u32_mpPassFlag != INIT_FIELD)) || 
		(info->chipinfo.u8_msScrConfigTuneVer != info->chipinfo.u8_msScrCxmemTuneVer || info->chipinfo.u8_ssTchConfigTuneVer != info->chipinfo.u8_ssTchCxmemTuneVer
		|| info->chipinfo.u8_msScrLpConfigTuneVer != info->chipinfo.u8_msScrLpCxmemTuneVer)
		|| flag_init == 1) {
		fts_info(info, "need calibration\n");
		return true;
	}

	fts_info(info, "no need calibration\n");
	return false;
}

static int fts_fw_update_auto(struct fts_ts_info *info)
{
	int retval = 0;
	int retval1 = 0;
	int ret;
	int crc_status = 0;
	fts_info(info, "Fw Auto Update is starting... ");

	/* check CRC status */
	ret = cx_crc_check(info);
	if (ret > OK && info->chipinfo.u16_fwVer == 0x0000) {
		fts_info(info, "CRC Error or NO FW!");
		crc_status = 1;
	} else {
		crc_status = 0;
		fts_info(info, "NO CRC Error or Impossible to read CRC register!");
	}

	retval = ftm4_flashProcedure(info, PATH_FILE_FW, crc_status, 1);
	if ((retval & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
		fts_info(info, "firmware update failed and retry! ERROR %08X", retval);
		ftm4_chip_powercycle(info); /* power reset */
		retval1 = ftm4_flashProcedure(info, PATH_FILE_FW, crc_status, 1);
		if ((retval1 & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
			fts_info(info, "firmware update failed again!  ERROR %08X", retval1);
		}
	}

	fts_info(info, "Fw Auto Update Finished!");
	if (fts_need_caliberate(info))
		return ERROR_GET_INIT_STATUS;

	return OK;
}

static int fts_chip_initialization(struct fts_ts_info *info)
{
	int ret2 = 0;
	int retry;
	int initretrycnt = 0;
	TestToDo todoDefault;
	enum vts_sensor_test_result result;

	todoDefault.MutualRaw = 1;
	todoDefault.MutualRawGap = 0;
	todoDefault.MutualCx1 = 0;
	todoDefault.MutualCx2 = 0;
	todoDefault.MutualCx2Adj = 0;
	todoDefault.MutualCxTotal = 1;
	todoDefault.MutualCxTotalAdj = 1;

	todoDefault.MutualKeyRaw = 0;
	todoDefault.MutualKeyCx1 = 0;
	todoDefault.MutualKeyCx2 = 0;
	todoDefault.MutualKeyCxTotal = 0;

	todoDefault.SelfForceRaw = 1;
	todoDefault.SelfForceRawGap = 0;
	todoDefault.SelfForceIx1 = 0;
	todoDefault.SelfForceIx2 = 0;
	todoDefault.SelfForceIx2Adj = 0;
	todoDefault.SelfForceIxTotal = 1;
	todoDefault.SelfForceIxTotalAdj = 0;
	todoDefault.SelfForceCx1 = 0;
	todoDefault.SelfForceCx2 = 0;
	todoDefault.SelfForceCx2Adj = 0;
	todoDefault.SelfForceCxTotal = 0;
	todoDefault.SelfForceCxTotalAdj = 0;

	todoDefault.SelfSenseRaw = 1;
	todoDefault.SelfSenseRawGap = 0;
	todoDefault.SelfSenseIx1 = 0;
	todoDefault.SelfSenseIx2 = 0;
	todoDefault.SelfSenseIx2Adj = 0;
	todoDefault.SelfSenseIxTotal = 1;
	todoDefault.SelfSenseIxTotalAdj = 0;
	todoDefault.SelfSenseCx1 = 0;
	todoDefault.SelfSenseCx2 = 0;
	todoDefault.SelfSenseCx2Adj = 0;
	todoDefault.SelfSenseCxTotal = 0;
	todoDefault.SelfSenseCxTotalAdj = 0;

	/*initialization error, retry initialization */
	for (retry = 0; retry <= INIT_FLAG_CNT; retry++) {
		ret2 = ftm4_production_test_main(info, LIMITS_FILE, 1, 1, &todoDefault, INIT_FIELD, &result);
		if (ret2 == OK)
			break;
		initretrycnt++;
		fts_info(info, "initialization cycle count = %04d - ERROR %08X ", initretrycnt, ret2);
	ftm4_chip_powercycle(info);
	}
	if (ret2 < OK) {
		fts_err(info, "fts initialization failed 3 times");
	}


	return ret2;
}

#ifdef FTS_USE_POLLING_MODE

static enum hrtimer_restart fts_timer_func(struct hrtimer *timer)
{
	struct fts_ts_info *info =
			container_of(timer, struct fts_ts_info, timer);

	/*sunsl queue_work(info->event_wq, &info->work); */
	return HRTIMER_NORESTART;
}
#else
static void fts_event_handler(struct fts_ts_info *info, ktime_t kt)
{
	int error, error1;
	int left_events;
	unsigned char regAdd;
	unsigned char data[FIFO_EVENT_SIZE * (FIFO_DEPTH)] = {0};
	unsigned char *event = NULL;
	unsigned char eventId;
	event_dispatch_handler_t event_handler;

	fts_dbg(info, "enter interrupt work handler !");
	/*
	 * to avoid reading all FIFO, we read the first event and
	 * then check how many events left in the FIFO
	 */

	regAdd = FIFO_CMD_READONE;
	error = ftm4_readCmd(info, &regAdd,
			sizeof (regAdd), data, FIFO_EVENT_SIZE);

	if (!error) {

		left_events = data[7] & 0x1F;
		if ((left_events > 0) && (left_events < FIFO_DEPTH)) {
			/*
			 * Read remaining events.
			 */
			regAdd = FIFO_CMD_READALL;

			error1 = ftm4_readCmd(info, &regAdd, sizeof (regAdd),
					&data[FIFO_EVENT_SIZE],
					left_events * FIFO_EVENT_SIZE);
			/*
			 * Got an error reading remaining events,
			 * process at least * the first one that was
			 * reading fine.
			 */
			if (error1)
				data[7] &= 0xE0;
		}

		/* At least one event is available */
		event = data;
		do {
			eventId = *event;

			if (eventId < EVENTID_LAST) {
				event_handler = info->event_dispatch_table[eventId];
				event = event_handler(info, (event), kt);
			} else {
				event = fts_next_event(event);
			}
		/*sunsl    input_sync(info->input_dev); */
		} while (event);
		vts_report_point_sync(info->vtsdev);

	}

	/*
	 * re - enable interrupts
	 */
	/*fts_interrupt_enable(info); */

	fts_dbg(info, "interrupt work handler complete !");
}
static irqreturn_t fts_interrupt_handler(int irq, void *handle, ktime_t kt)
{
	struct fts_ts_info *info = handle;

	/*disable_irq_nosync(info->client->irq); */
	/*queue_work(info->event_wq, &info->work); */
	fts_event_handler(info, kt);
	return IRQ_HANDLED;
}
#endif

static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;
	int ret = 0;

	info->event_dispatch_table = kzalloc(
			sizeof (event_dispatch_handler_t) * EVENTID_LAST, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		fts_err(info, "OOM allocating event dispatch table");
		return -ENOMEM;
	}

	for (i = 0; i < EVENTID_LAST; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINTER, enter_pointer);
	install_handler(info, LEAVE_POINTER, leave_pointer);
	install_handler(info, MOTION_POINTER, motion_pointer);
	install_handler(info, ERROR_EVENT, error);
	install_handler(info, CONTROL_READY, controller_ready);
	install_handler(info, STATUS_UPDATE, status);
	/*add for ear detect*/
	install_handler(info, EAR_DETECT, ear_detect);
	install_handler(info, EAR_DEBUG, ear_debug);
#ifdef PHONE_GESTURE
	install_handler(info, GESTURE, gesture);
#endif
#ifdef PHONE_KEY
  /*sunsl  install_handler(info, KEY_STATUS, key_status); */
#endif

	/* disable interrupts in any case */
	error = ftm4_disableInterrupt(info);

#ifdef FTS_USE_POLLING_MODE
	fts_dbg(info, "Polling Mode");
	hrtimer_init(&info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	info->timer.function = fts_timer_func;
	hrtimer_start(&info->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
	fts_info(info, " Interrupt Mode");
	ret = vts_interrupt_register(info->vtsdev, info->client->irq,fts_interrupt_handler, IRQF_ONESHOT | IRQF_TRIGGER_LOW, info);
	if (ret < 0) {
		fts_info(info, "request register irq failed");
		kfree(info->event_dispatch_table);
		error = -EBUSY;
	} /*else {
		error = ftm4_enableInterrupt(info);
	} */
#endif

	return error;
}

static void fts_interrupt_uninstall(struct fts_ts_info *info)
{

	ftm4_disableInterrupt(info);

	kfree(info->event_dispatch_table);
#ifdef FTS_USE_POLLING_MODE
	hrtimer_cancel(&info->timer);
#else
	vts_interrupt_unregister(info->vtsdev);
#endif
}

static int fts_init(struct fts_ts_info *info)
{
	int error = 0;
	vts_property_get(info->vtsdev, VTS_PROPERTY_BROKEN_DISABLE, &info->broken_disable);

	error = ftm4_system_reset(info);
	if (error < OK && error != (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
		fts_err(info, "Cannot reset the device! ERROR %08X", error);
		return error;
	} else {
		if (error == (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
			fts_err(info, "Setting default Chip INFO!");
			fts_defaultChipInfo(info, 0);
		} else {
			error = fts_readChipInfo(info, 0);/*system reset OK */
			if (error < OK) {
				fts_err(info, "Cannot read Chip Info! ERROR %08X\n", error);
		    }
		}
	}

   error = fts_interrupt_install(info);
	if (error != OK)
		fts_err(info, "Init (1) error (ERROR  = %08X)", error);

	if (info->broken_disable) {
		error = vts_esd_error_thread_register(info);
		if (error != OK)
			VTE("Init (2) error (ERROR  = %08X)", error);
	}

	return error;
}

/*sunsl add fts_power_on */
static int fts_power_on(struct fts_ts_info *info, bool onoff)
{
	int rc;
	struct fts_i2c_platform_data *bdata = &info->bdata;


	/* power on */
	if (onoff) {
		/* PMIC power control */
		fts_info(info, "enable1.8 and 3.3v");
		/*return 0; */
		if (gpio_is_valid(bdata->power_gpio)) {
			gpio_direction_output(bdata->power_gpio, 1);
		} else {
		if (regulator_count_voltages(info->bus_reg) > 0) {
			rc = regulator_set_voltage(info->bus_reg, 3000000,
								3000000);
			if (rc) {
				fts_info(info, "regulator set_vtg:bus_reg vcc_ana failed rc=%d", rc);
				regulator_put(info->bus_reg);
				return rc;
			}
		}
		rc = regulator_enable(info->bus_reg);
		if (rc) {
			fts_info(info, "Regulator bus_reg vcc_ana enable failed rc=%d", rc);
			return rc;
		}
		}

		//msleep(10);
		if (gpio_is_valid(bdata->vcc_gpio)) {
			fts_info(info, "vcc_gpio power on");
			gpio_direction_output(bdata->vcc_gpio, 1);
		} else {
		//__1.8v
		if (regulator_count_voltages(info->pwr_reg) > 0) {
			rc = regulator_set_voltage(info->pwr_reg, info->pwr_voltage ? info->pwr_voltage : 1800000,
								info->pwr_voltage ? info->pwr_voltage : 1800000);
			if (rc) {
				fts_info(info, "regulator set_vtg:pwr_regvcc_i2c failed rc=%d", rc);
				regulator_put(info->pwr_reg);
				return rc;
			}
		}
		rc = regulator_enable(info->pwr_reg);
		if (rc) {
			fts_info(info, "Regulator pwr_reg vcc_i2c enable failed rc=%d", rc);
			return rc;
		}
		}
	} else {
	/* power off */
	/* PMIC power control */
		fts_info(info, "disable1.8 and 3.3v");
		if (gpio_is_valid(bdata->vcc_gpio)) {
			gpio_direction_output(bdata->vcc_gpio, 0);
		} else {
		/* _1.8v must first power down, else will get big problem in resume and esd event handler */
		rc = regulator_disable(info->pwr_reg);
		if (rc) {
			fts_info(info, "Regulator info->pwr_reg vcc_i2c enable failed rc=%d", rc);
			return rc;
		}
		}
 
		msleep(10);
		/* _3.3v */
		if (gpio_is_valid(bdata->power_gpio)) {
			gpio_direction_output(bdata->power_gpio, 0);
		} else {
		rc = regulator_disable(info->bus_reg);
		if (rc) {
			fts_info(info, "Regulator bus_reg vcc_ana enable failed rc=%d", rc);
			return rc;
		}
		/*sunsl add */
		}
		msleep(20);
	}

	return 0;
}

void ftm4_reset_pin_ctl(struct fts_ts_info *info, int state)
{
	struct fts_i2c_platform_data *data	 =  &info->bdata;
	int error = 0;

	if (gpio_is_valid(data->reset_gpio)) {
		error = gpio_direction_output(data->reset_gpio, state);
		if (error) {
			fts_info(info, "unable to set direction out to 0 for gpio [%d]", data->reset_gpio);
		} else {
			fts_info(info, "set rst_gpio to %d succeed.", state);
		}
	} else {
		fts_info(info, "rst_gpio not valid.");
	}
}

void  fts_power_reset(struct fts_ts_info *info)
{
	int error = 0;

	mutex_lock(&(info->i2cResetMutex));

	/*power down and power on */
	error = fts_power_on(info, 0);
	if (error) {
		fts_info(info, "fts_power_off error");
	}
	ftm4_reset_pin_ctl(info, 0);/*must low rst pin, because low temputure no function */

	msleep(10);
	error = fts_power_on(info, 1);
	if (error) {
		fts_info(info, "fts_power_off error");
	}

	msleep(10);
	ftm4_reset_pin_ctl(info, 1);/*must, because low temputure no function	 */
	msleep(10);

	mutex_unlock(&(info->i2cResetMutex));
}

/*sunsl add end */



int ftm4_chip_powercycle(struct fts_ts_info *info)
{
	int error = 0;
	struct fts_i2c_platform_data *bdata = &info->bdata;

	fts_info(info, "enter");
	mutex_lock(&(info->i2cResetMutex));
	if (gpio_is_valid(bdata->vcc_gpio)) {
		gpio_direction_output(bdata->vcc_gpio, 0);
	} else {
		if (info->pwr_reg) {
			error = regulator_disable(info->pwr_reg);
			if (error < 0) {
				fts_info(info, "Failed to disable DVDD regulator");
			}
		}
	}

	if (gpio_is_valid(bdata->power_gpio)) {
		gpio_direction_output(bdata->power_gpio, 0);
	} else {
	if (info->bus_reg) {
		error = regulator_disable(info->bus_reg);
		if (error < 0) {
			fts_err(info, "Failed to disable AVDD regulator");
			}
		}
	}

	if (bdata->reset_gpio != GPIO_NOT_DEFINED)
		gpio_set_value(bdata->reset_gpio, 0);

	mdelay(300);
	if (gpio_is_valid(bdata->power_gpio)) {
		gpio_direction_output(bdata->power_gpio, 1);
	} else {
		if (info->bus_reg) {
		error = regulator_enable(info->bus_reg);
		if (error < 0) {
			fts_err(info, "Failed to enable AVDD regulator");
			}
		}
	}
	if (gpio_is_valid(bdata->vcc_gpio)) {
		gpio_direction_output(bdata->vcc_gpio, 1);
	} else {
		if (info->pwr_reg) {
			error = regulator_enable(info->pwr_reg);
			if (error < 0) {
				fts_err(info, "Failed to enable DVDD regulator\n");
			}
		}
	}
	mdelay(300); /*time needed by the regulators for reaching the regime values */


	if (bdata->reset_gpio != GPIO_NOT_DEFINED) {
		mdelay(10); /*time to wait before bring up the reset gpio after the power up of the regulators */
		gpio_set_value(bdata->reset_gpio, 1);
		/*mdelay(300); */
	}
	mutex_unlock(&(info->i2cResetMutex));
	fts_info(info, "end");
	return error;
}

static int fts_init_hw(struct fts_ts_info *info)
{
	int error = 0;
	u8 guesturemask[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	ftm4_updateGestureMask(info, guesturemask, 8, FEAT_ENABLE);  /*enable by default all the gestures */

	info->edge_suppress_enabled = 1;        /*enable by default as per Vivo request; */
	info->edge_pix_reject = 1;
	info->edge_corner_reject = 1;
	info->edge_L_reject = 1;
	/*error = ftm4_cleanUp(1); */

	error = ftm4_system_reset(info);
	error |= fts_mode_handler(info, 0);
	error |= ftm4_enableInterrupt(info);

	if (error < OK)
		fts_dbg(info, "Init (2) error (ERROR = %08X)", error);

	info->mode = MODE_NORMAL;

	return error;
}

/*TODO: change this function according with the needs of customer in temrs of feature to enable / disable */
static int fts_mode_handler(struct fts_ts_info *info, int force)
{
	int res = OK;
	/*int ret = OK; */

	fts_info(info, "Mode Handler starting...");

	switch (info->resume_bit) {
	case 0:/*screen down */
		fts_info(info, "Screen OFF... ");
		res |= fts_command(info, FTS_CMD_MS_MT_SENSE_OFF);/*we need to use fts_command for speed reason (no need to check echo in this case and interrupt can be enabled) */

	#ifdef PHONE_KEY
		fts_info(info, "Key OFF!");
		res |= fts_command(info, FTS_CMD_MS_KEY_OFF);
	#endif

	#ifdef PHONE_GESTURE
		if (info->gesture_enabled == 1) {
			fts_dbg(info, "enter in gesture mode !");
			res = ftm4_enterGestureMode(info, 1);/*isSystemResettedDown()); sunsl */
			if (res >= OK) {
				info->mode = MODE_GESTURE;
				/*return OK; */
			} else {
				fts_err(info, "ftm4_enterGestureMode failed! ERROR %08X recovery in ftm4_senseOff...\n", res);
			}
		}
	#endif
		if (info->mode != MODE_GESTURE || info->gesture_enabled == 0) {
			fts_info(info, "Sense OFF!");
			info->mode = MODE_SENSEOFF;

		}
		/*setSystemResettedDown(0); */
		break;

	case 1:	/*screen up */
		/*ftm4_logError(0, "%s %s: Edge Rejection setting... \n", tag, __func__);//BBK want it always on
		res |= ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_REJECTION);
		if (ret < OK) {
			ftm4_logError(1, "%s %s: error during setting Edge Rejection! ERROR %08X\n", tag, __func__, ret);
		} */

		info->mode = MODE_NORMAL;
		fts_info(info, "Sense ON! ");
		res |= fts_command(info, FTS_CMD_MS_MT_SENSE_ON);
#ifdef PHONE_KEY
		fts_dbg(info, "Key ON! ");
		res |= fts_command(info, FTS_CMD_MS_KEY_ON);
#endif

	/*setSystemResettedUp(0); */
	break;

	default:
		fts_info(info, "invalid resume_bit value = %d! ERROR %08X ", info->resume_bit, ERROR_OP_NOT_ALLOW);
		res = ERROR_OP_NOT_ALLOW;
	}


	fts_info(info, "Mode Handler finished! res = %08X \n", res);
	return res;

}

static int fts_get_reg(struct fts_ts_info *info,
		bool get)
{
	int retval;
	const struct fts_i2c_platform_data *bdata =
			&info->bdata;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}
	fts_info(info, "pwr_reg_name is %s", bdata->pwr_reg_name);
	if ((bdata->pwr_reg_name != NULL) && (*bdata->pwr_reg_name != 0)) {

		info->pwr_reg = regulator_get(info->dev,
				bdata->pwr_reg_name);
		fts_info(info, "pwr_reg get");
		if (IS_ERR(info->pwr_reg)) {
			fts_info(info, "Failed to get power regulator");
			retval = PTR_ERR(info->pwr_reg);
			goto regulator_put;
		}
	}

	fts_info(info, "bus_reg_name is %s", bdata->bus_reg_name);
	if ((bdata->bus_reg_name != NULL) && (*bdata->bus_reg_name != 0)) {
		info->bus_reg = regulator_get(info->dev,
				bdata->bus_reg_name);
		if (IS_ERR(info->bus_reg)) {
			fts_err(info, "Failed to get bus pullup regulator");
			retval = PTR_ERR(info->bus_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (info->pwr_reg) {
		regulator_put(info->pwr_reg);
		info->pwr_reg = NULL;
	}

	if (info->bus_reg) {
		regulator_put(info->bus_reg);
		info->bus_reg = NULL;
	}

	return retval;
}

static int fts_enable_reg(struct fts_ts_info *info, bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (info->bus_reg) {
		retval = regulator_enable(info->bus_reg);
		if (retval < 0) {
			fts_err(info, "Failed to enable bus regulator");
			goto exit;
		}
	}

	if (info->pwr_reg) {
		retval = regulator_enable(info->pwr_reg);
		if (retval < 0) {
			fts_err(info, "Failed to enable power regulator");
			goto disable_bus_reg;
		}
	}

	return OK;

disable_pwr_reg:
	if (info->pwr_reg)
		regulator_disable(info->pwr_reg);

disable_bus_reg:
	if (info->bus_reg)
		regulator_disable(info->bus_reg);

exit:
	return retval;
}
static int fts_mix_set_power_state(struct fts_ts_info *info, bool enable)
{
	int retval = 0;
	const struct fts_i2c_platform_data *bdata =
			&info->bdata;
	if (bdata->power_gpio >= 0) {
		retval = gpio_direction_output(bdata->power_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			fts_info(info, "Fail to set power gpio %s", (enable) ? "on" : "off");
		}
	}
	if (bdata->vcc_gpio >= 0) {
		retval = gpio_direction_output(bdata->vcc_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			fts_info(info, "Fail to set vcc gpio %s", (enable) ? "on" : "off");
		}
	}
	retval = fts_enable_reg(info, enable);
	if (retval < 0) {
		fts_info(info, "Fail to set regulator %s", (enable) ? "on" : "off");
	}
	return retval;
}

static int fts_gpio_setup(struct fts_ts_info *info, int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		snprintf(buf, 16, "fts_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			fts_err(info, "Failed to get gpio %d (code: %d)", gpio, retval);
			return retval;
		}

		if (dir == 0) {
			retval = gpio_direction_input(gpio);
		} else {
			retval = gpio_direction_output(gpio, state);
		}
		if (retval) {
			fts_err(info, "Failed to set gpio %d direction", gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return retval;
}

static int fts_set_gpio(struct fts_ts_info *info, bool enable)
{
	int retval;
	const struct fts_i2c_platform_data *bdata =
			&info->bdata;

	if (!enable) {
		retval = 0;
		goto err_gpio_both;
	}

	retval = fts_gpio_setup(info, bdata->irq_gpio, true, 0, 0);
	if (retval < 0) {
		fts_err(info, "Failed to configure irq GPIO");
		goto err_gpio_irq;
	}

	if (bdata->reset_gpio >= 0) {
		retval = fts_gpio_setup(info, bdata->reset_gpio, true, 1, 0);
		if (retval < 0) {
			fts_err(info, "Failed to configure reset GPIO");
			goto err_gpio_reset;
		}
	}

	if (bdata->power_gpio >= 0) {
		retval = fts_gpio_setup(info, bdata->power_gpio, true, 1, 0);
		if (retval < 0) {
			fts_err(info, "Failed to configure power GPIO");
			goto err_gpio_power;
		}
	}

	if (bdata->vcc_gpio >= 0) {
		retval = fts_gpio_setup(info, bdata->vcc_gpio, true, 1, 0);
		if (retval < 0) {
			fts_err(info, "Failed to configure vcc GPIO");
			goto err_gpio_vcc;
		}
	}

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		mdelay(10);
		/*gpio_set_value(bdata->reset_gpio, 1); */
	}

	return OK;

err_gpio_vcc:
	fts_gpio_setup(info, bdata->power_gpio, false, 0, 0);
err_gpio_power:
	fts_gpio_setup(info, bdata->reset_gpio, false, 0, 0);
err_gpio_reset:
	fts_gpio_setup(info, bdata->irq_gpio, false, 0, 0);
err_gpio_irq:
err_gpio_both:
	return retval;
}

static int parse_dt(struct device *dev, struct fts_ts_info *info)
{
	int retval;
	const char *name;
	struct device_node *np = dev->of_node;
	 struct fts_i2c_platform_data *bdata = &info->bdata;

	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, NULL);/*st, irq - gpio */
	fts_info(info, "irq_gpio = %d", bdata->irq_gpio);

	of_property_read_u32(np, "st,pwr_voltage", &info->pwr_voltage);
	fts_info(info, "power voltage:%d", info->pwr_voltage);

	if (of_property_read_bool(np, "synaptics,reset-gpio")) {	/*st, reset - gpio */
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);/*st, reset - gpio */
		fts_info(info, "reset_gpio = %d", bdata->reset_gpio);
	} else {
		bdata->reset_gpio = GPIO_NOT_DEFINED;
	}

	if (of_property_read_bool(np, "fts,power-gpio")) {	/*st,power-gpio*/
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"fts,power-gpio", 0, NULL);	/*st,power-gpio*/
		fts_info(info, "power_gpio =%d", bdata->power_gpio);
	} else {
		bdata->power_gpio = GPIO_NOT_DEFINED;
	}

	if (bdata->power_gpio == GPIO_NOT_DEFINED) {
		retval = of_property_read_string(np, "st,regulator_avdd", &name);
		if (retval == -EINVAL)
			bdata->bus_reg_name = "vdd_ana";
		else if (retval < 0)
			return retval;
		else {
			bdata->bus_reg_name = name;
			fts_info(info, "bus_reg_name = %s", name);
		}
	} else {
		bdata->bus_reg_name = NULL;
	}


	if (of_property_read_bool(np, "synaptics,vcc-gpio")) {	/*_1.8 */
		bdata->vcc_gpio = of_get_named_gpio_flags(np,
				"synaptics,vcc-gpio", 0, NULL);/*_1.8 */
		fts_info(info, "vcc_gpio = %d", bdata->vcc_gpio);
	} else {
		bdata->vcc_gpio = GPIO_NOT_DEFINED;
	}

	if (bdata->vcc_gpio == GPIO_NOT_DEFINED) {
		retval = of_property_read_string(np, "st,regulator_dvdd", &name);
		if (retval == -EINVAL)
			bdata->pwr_reg_name = "vcc_i2c";
		else if (retval < 0)
			return retval;
		else {
			bdata->pwr_reg_name = name;
			fts_info(info, "pwr_reg_name = %s", name);
		}
	} else {
		bdata->pwr_reg_name = NULL;
	}

	return OK;
}

static int ftm4_idle_switch_write(struct vts_device *vtsdev, int val);
static int irq_wake_ctl(struct fts_ts_info *info, int state)
{
	fts_info(info, "enter");
	if (state == 1 && info->irq_wake_state == 0) {
		fts_info(info, "enable irq wake");
		enable_irq_wake(info->client->irq);
		info->irq_wake_state = 1;
	} else if (state == 0 && info->irq_wake_state == 1) {
		fts_info(info, "disable irq wake");
		disable_irq_wake(info->client->irq);
		info->irq_wake_state = 0;
	}
	return 0;
}

static int ftm4_mode_change(struct vts_device *vtsdev, int which)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int power_reset_count = 0;
	int error = 0;
	int ret = 0;

	fts_info(info, "main, mode change:%d\n", which);

	if (which == VTS_ST_NORMAL) {
		mutex_lock(&(info->esdErrorMutex));
		fts_info(info, "change to normal mode");
		info->esd_error_state = 0;
		//disable_irq_wake(info->client->irq);
		irq_wake_ctl(info, 0);
		info->resume_bit = 1;
		//ftm4_readNoiseParameters(info, noise_params);
		/*ftm4_system_reset(info); */

power_reset_tag:
		if (power_reset_count > 0) {
			fts_info(info, "[%s]power reset error!power_reset_count=%d.error=%d\n", __func__, power_reset_count, error);
		}
		error = 0;
		fts_info(info, "step 1.power down reset.\n");

		fts_power_reset(info);

		/*error += ftm4_system_reset(info); */
		ftm4_irq_enable_disable(info, 0, 0);	//writeNoiseParameters will fail if not;
		ftm4_flushFIFO(info);
		//ftm4_writeNoiseParameters(info, noise_params);


		fts_info(info, "step 2.sense on and key on.\n");
		error |= fts_command(info, FTS_CMD_MS_MT_SENSE_ON);
		mdelay(10);
		fts_mode_handler(info, 0);
		ftm4_irq_enable_disable(info, 1, 0);	//writeNoiseParameters will fail if not;
		error += ftm4_enableInterrupt(info);
		if (error < 0) {
			power_reset_count++;
			if (power_reset_count > 2) {
				fts_info(info, "[%s]3 times power down reset fail.\n", __func__);
				mutex_unlock(&(info->esdErrorMutex));
				ret = -1;
				return ret;
			} else {
				msleep(50);
				goto power_reset_tag;
			}
		}
		mutex_unlock(&(info->esdErrorMutex));
	}

	if (which == VTS_ST_GESTURE) {
		fts_info(info, "change to gesture mode");
		irq_wake_ctl(info, 1);
		info->resume_bit =0;
		info->gesture_enabled =1;
		fts_mode_handler(info, 0);
		mutex_lock(&(info->esdErrorMutex));
		if (!info->esd_error_state)
			ftm4_irq_enable_disable(info, 1, 0);
		mutex_unlock(&(info->esdErrorMutex));
	}
	if (which == VTS_ST_SLEEP) {
		fts_info(info, "change to sleep mode");
		//disable_irq_wake(info->client->irq);
		irq_wake_ctl(info, 0);
		info->resume_bit =0;
		info->gesture_enabled=0;
		fts_mode_handler(info, 0);
		ftm4_irq_enable_disable(info, 0, 0);
	}

	return ret;
}

static int ftm4_write_charger_flag(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	fts_info(info, "write usb charger flag,state=%d", state);
	info->charger_enabled = !!state;
	ret = ftm4_featureEnableDisable(info, !!state, FEAT_CHARGER);
	if (ret < 0) {
		fts_info(info, "write usb charger flag fail");
	}
	return ret;
}
static int ftm4_edge_restain_switch(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	u8 cmd1[6] = {0xc9, 0x02, 0xD5, 0xff, 0xef, 0xff};
	u8 cmd2[7] = {0};
	u8 horizontal_screen_cmd[2] = {0xC6, 0x01};
	u8 vertical_screen_cmd[2] = {0xC6, 0x00};

	fts_info(info, "get screen state:%d", state);

	ftm4_writeCmd(info, (state == 0 || state == 2) ? horizontal_screen_cmd : vertical_screen_cmd, 2);

	if (0 == state) {
		fts_info(info, "enabling edge features 0 degree");
		//ftm4_featureEnableDisable(FEAT_DISABLE, FEAT_EDGE_CORNER_REJECTION);
		//ftm4_featureEnableDisable(FEAT_DISABLE, FEAT_EDGE_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_L_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_PIX_REJECT);
		ftm4_writeCmd(info, cmd1, 6);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x01;
		cmd2[2] = 0x50;
		cmd2[3] = 0x5A;
		cmd2[4] = 0x20;
		cmd2[5] = 0x00;
		ftm4_writeCmd(info, cmd2, 6);

		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x09;
		cmd2[3] = 0x64;
		cmd2[4] = 0x00;
		cmd2[5] = 0x64;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0B;
		cmd2[3] = 0x64;
		cmd2[4] = 0x00;
		cmd2[5] = 0x64;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0C;
		cmd2[3] = 0x2C;
		cmd2[4] = 0x01;
		cmd2[5] = 0x96;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0E;
		cmd2[3] = 0x2C;
		cmd2[4] = 0x01;
		cmd2[5] = 0x96;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		fts_info(info, "disabling edge regection");
		info->edge_suppress_enabled = 0;
	} else if (1 == state) {	/* 90 degree*/
		fts_info(info, "enabling edge features 90 degree");
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_PIX_REJECT);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_CORNER_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_L_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_REJECTION); /*sunsl */
		ftm4_writeCmd(info, cmd1, 6);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x01;
		cmd2[2] = 0x00;
		cmd2[3] = 0xC0;
		cmd2[4] = 0x00;
		cmd2[5] = 0x00;
		ftm4_writeCmd(info, cmd2, 6);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0E;
		cmd2[3] = 0x96;
		cmd2[4] = 0x00;
		cmd2[5] = 0x5E;
		cmd2[6] = 0x01;
		ftm4_writeCmd(info, cmd2, 7);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0F;
		cmd2[3] = 0x96;
		cmd2[4] = 0x00;
		cmd2[5] = 0x5E;
		cmd2[6] = 0x01;
		ftm4_writeCmd(info, cmd2, 7);
	
		
		info->edge_suppress_enabled = 1;
		info->edge_pix_reject = 1;
		info->edge_corner_reject = 1;
		info->edge_L_reject = 1;
	} else if (2 == state) {
		fts_info(info, "enabling edge features 180 degree");
		//ftm4_featureEnableDisable(FEAT_DISABLE, FEAT_EDGE_CORNER_REJECTION);
		//ftm4_featureEnableDisable(FEAT_DISABLE, FEAT_EDGE_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_L_REJECTION);
		//ftm4_featureEnableDisable(FEAT_ENABLE, FEAT_EDGE_PIX_REJECT);
		ftm4_writeCmd(info, cmd1, 6);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x01;
		cmd2[2] = 0x50;
		cmd2[3] = 0xA5;
		cmd2[4] = 0x20;
		cmd2[5] = 0x00;
		ftm4_writeCmd(info, cmd2, 6);
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x08;
		cmd2[3] = 0x64;
		cmd2[4] = 0x00;
		cmd2[5] = 0x64;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0A;
		cmd2[3] = 0x64;
		cmd2[4] = 0x00;
		cmd2[5] = 0x64;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0D;
		cmd2[3] = 0x2C;
		cmd2[4] = 0x01;
		cmd2[5] = 0x96;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		
		cmd2[0] = 0xc9;
		cmd2[1] = 0x03;
		cmd2[2] = 0x0F;
		cmd2[3] = 0x2C;
		cmd2[4] = 0x01;
		cmd2[5] = 0x96;
		cmd2[6] = 0x00;
		ftm4_writeCmd(info, cmd2, 7);
		fts_info(info, "disabling edge regection");
		info->edge_suppress_enabled = 0;
	}
	return ret;
}

static int ftm4_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;
	int error;
	short *mutual_data = NULL;
	short *tx_self_data = NULL;
	int tx_size;
	short *rx_self_data = NULL;
	int rx_size;
	int size_out;
	bool int_enabled = false;

	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled) {
		error = ftm4_disableInterrupt(info);
		if (error < OK) {
			fts_err(info, "disable int error! %d\n", error);
			return error;
		}
	}

	switch (type) {
		case VTS_FRAME_MUTUAL_RAW:
			{
				error = ftm4_getMSFrame2(info, MS_TOUCH_ACTIVE, &frameMS);
				if (error < 0) {
					fts_err(info, "get mutual raw error, %d\n", error);
					goto end;
				}
				mutual_data = frameMS.node_data;
				size_out = error;
			}
			break;
		case VTS_FRAME_MUTUAL_DELTA:
			{
				error = ftm4_getMSFrame(info, ADDR_NORM_TOUCH, &mutual_data, 0);
				if (error < OK) {
					fts_err(info, "get mutual delta error, %d\n", error);
					goto end;
				}
				size_out = error;
			}
			break;
		case VTS_FRAME_SELF_RAW:
			{
				error = ftm4_getSSFrame2(info, SS_TOUCH, &frameSS);
				if (error < 0) {
					fts_err(info, "get self raw error, %d\n", error);
					goto end;
				}			

				tx_self_data = frameSS.force_data;
				tx_size = frameSS.header.force_node;
				rx_self_data = frameSS.sense_data;
				rx_size = frameSS.header.sense_node;
				size_out = tx_size + rx_size;
			}
			break;
		case VTS_FRAME_SELF_DELTA:
			{
				error = ftm4_getSSFrame(info, ADDR_NORM_PRX_FORCE, &tx_self_data);
				if (error < 0) {
					fts_err(info, "get self delta tx error,%d\n", error);
					goto end;
				}

				tx_size = error;
			
				error = ftm4_getSSFrame(info, ADDR_NORM_PRX_SENSE, &rx_self_data);
				if (error < 0) {
					fts_err(info, "get self delta rx error,%d\n", error);
					goto end;
				}

				rx_size = error;
				size_out = tx_size + rx_size;
			}
			break;
		default:
			fts_err(info, "invalid type %d\n", type);
			goto end;
			break;
	}

	if (size < size_out) {
		fts_err(info, "no enough mem to store data, actual:%d, expected:%d\n", size, size_out);
		goto end;
	}

	if (type == VTS_FRAME_MUTUAL_RAW || type == VTS_FRAME_MUTUAL_DELTA) {
		memcpy(data, mutual_data, size_out * sizeof(short));
	} else {
		memcpy(data, tx_self_data, tx_size * sizeof(short));
		memcpy(((u8 *)data) +  tx_size * sizeof(short), rx_self_data, rx_size * sizeof(short));
	}
end:
	if (int_enabled)
		ftm4_enableInterrupt(info);
	if (mutual_data != NULL)
		kfree(mutual_data);
	if (tx_self_data != NULL)
		kfree(tx_self_data);
	if (rx_self_data != NULL)
		kfree(rx_self_data);
	return 0;
}

static int ftm4_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	TestToDo todoDefault;

	todoDefault.MutualRaw = 1;
	todoDefault.MutualRawGap = 0;
	todoDefault.MutualCx1 = 0;
	todoDefault.MutualCx2 = 0;
	todoDefault.MutualCx2Adj = 0;
	todoDefault.MutualCxTotal = 1;
	todoDefault.MutualCxTotalAdj = 1;
	todoDefault.MutualKeyRaw = 0;
	todoDefault.MutualKeyCx1 = 0;
	todoDefault.MutualKeyCx2 = 0;
	todoDefault.MutualKeyCxTotal = 0;

	todoDefault.SelfForceRaw = 1;
	todoDefault.SelfForceRawGap = 0;
	todoDefault.SelfForceIx1 = 0;
	todoDefault.SelfForceIx2 = 0;
	todoDefault.SelfForceIx2Adj = 0;
	todoDefault.SelfForceIxTotal = 1;
	todoDefault.SelfForceIxTotalAdj = 0;
	todoDefault.SelfForceCx1 = 0;
	todoDefault.SelfForceCx2 = 0;
	todoDefault.SelfForceCx2Adj = 0;
	todoDefault.SelfForceCxTotal = 0;
	todoDefault.SelfForceCxTotalAdj = 0;

	todoDefault.SelfSenseRaw = 1;
	todoDefault.SelfSenseRawGap = 0;
	todoDefault.SelfSenseIx1 = 0;
	todoDefault.SelfSenseIx2 = 0;
	todoDefault.SelfSenseIx2Adj = 0;
	todoDefault.SelfSenseIxTotal = 1;
	todoDefault.SelfSenseIxTotalAdj = 0;
	todoDefault.SelfSenseCx1 = 0;
	todoDefault.SelfSenseCx2 = 0;
	todoDefault.SelfSenseCx2Adj = 0;
	todoDefault.SelfSenseCxTotal = 0;
	todoDefault.SelfSenseCxTotalAdj = 0;

	vts_report_release(info->vtsdev);

	if (info->chipinfo.u32_mpPassFlag != INIT_MP) {
		fts_info(info, "MP Flag not set!");
			ret = ftm4_production_test_main(info, LIMITS_FILE, 1, 1, &todoDefault, INIT_MP, result);
	} else {
		fts_info(info, "MP Flag set!");
		ret = ftm4_production_test_main(info, LIMITS_FILE, 1, 0, &todoDefault, INIT_MP, result);
	}
	fts_info(info, "at_sensor_test result:%d\n", *result);

	fts_mode_handler(info, 0);
	vts_report_release(info->vtsdev);
	ftm4_enableInterrupt(info);

	return ret;
}

static int ftm4_sensor_caliberate(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	*result = VTS_SENSOR_CALIBERATE_SUCCESS;

	vts_report_release(info->vtsdev);

	if (fts_chip_initialization(info) < 0) {
		*result |= VTS_SENSOR_CALIBERATE_FAILED;
		fts_info(info, "Pass.Calibration fail.");
	}

	ret = fts_save_mp_flag(info, INIT_MP);
	fts_info(info, "save mp");
	if (ret < 0) {
		fts_info(info, "[%s]Fail! Error while saving the MP flag!\n", __func__);
	}
	fts_readChipInfo(info, 1);

	fts_info(info, "at_sensor_test result:%d\n", *result);
	fts_mode_handler(info, 0);
	vts_report_release(info->vtsdev);
	ftm4_enableInterrupt(info);
	return 0;
}


static int ftm4_idle_switch_write(struct vts_device *vtsdev, int val)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 openCmd[] = {0xC2, 0x00, 0x00, 0x01};
	u8 closeCmd[] = {0xC1, 0x00, 0x00, 0x01};

	if (val)
		ret = ftm4_writeCmd(info, openCmd, sizeof (openCmd));
	else
		ret = ftm4_writeCmd(info, closeCmd, sizeof (closeCmd));
	return ret;
}

static int ftm4_fw_version(struct vts_device *vtsdev, u64 *version)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	*version = (((u64)info->chipinfo.u16_fwVer) << 16) | ((u64)info->chipinfo.u16_cfgId);
	return 0;
}
/*
static int ftm4_gesture_point(u16 *data)
{
	int i = 0;
	int pointnum = ftm4_point_num;
	for (i = 0; i < pointnum; i++) {
		data[2 * i] = ftm4_gesture_point_x[i];
		data[2 * i + 1] = ftm4_gesture_point_y[i];
	}
	printk("\n");

	return pointnum;
}
*/
/**************************************************************************
 * Calculates CRC8 value of an array of bytes & return 8 - bit CRC value
 * @param u8_srcBuff[] : input array of bytes
 * @param u32_len : the number of input bytes
 * @param u8_polynomial: polynomial for CRC calculation
 * @param u8_initCRCVal: Initial CRC value
 * @return CRC8 result
 **************************************************************************/
uint8_t ftm4_CalculateCRC8(uint8_t *u8_srcBuff, uint32_t u32_len, uint8_t u8_polynomial, uint8_t u8_initCRCVal)
{
	u8 u8_remainder;
	u8 bit;
	int i = 0;
	u8_remainder = u8_initCRCVal;

	/* Perform modulo -2 division, a byte at a time. */
	for (i = 0; i < u32_len; i++) {/*Bring the next byte into the remainder. */
		u8_remainder ^= u8_srcBuff[i]; /*Perform modulo -2 division, a bit at a time. 		 */
		for (bit = 8; bit > 0; --bit) {		/*Try to divide the current data bit. */
			if (u8_remainder & (0x1 << 7)) {
				u8_remainder = (u8_remainder << 1) ^ u8_polynomial;
			} else {
				u8_remainder = (u8_remainder << 1);
			}
		}
	} /* The final remainder is the CRC result. */
	return u8_remainder;
}

static int ftm4_read_user_data(struct fts_ts_info *info, unsigned char *user_data)
{
	int ret, res;
	u8 readdatacmd = 0x85;
	u8 readdata[8] = {0};
	int i = 0;
	unsigned char start_read = 0xcc;
	int loop_count = 0;
	bool int_enabled = false;

	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled) {
		ret = ftm4_disableInterrupt(info);
		if (ret < OK) {
			fts_err(info, "ERROR %02X\n", ret);
			goto ERROR;
		}
	}

	ret = ftm4_writeCmd(info, &start_read, 1);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		goto ERROR;
	}

	for (i = 0; i < 5; i++) {
		ret = ftm4_readCmd(info, (u8 *)(&readdatacmd), 1, readdata, 8);
		if (ret < OK) {
			fts_info(info, "impossible write command... ERROR %02X", ret);
			goto ERROR;
		}
		if (readdata[0] != 0x1E) {
			loop_count++;
			if (loop_count == 100)
				break;
			i--;
			fts_info(info, "not user_data event");
			continue;
		}
		loop_count = 0;

		if (i < 4) {
			user_data[4 * i] = readdata[3];
			user_data[4 * i + 1] = readdata[4];
			user_data[4 * i + 2] = readdata[5];
			user_data[4 * i + 3] = readdata[6];
			fts_info(info, "user_data:%d %d %d %d", user_data[4 * i], user_data[4 * i + 1], user_data[4 * i + 2], user_data[4 * i + 3]);
		} else {
			user_data[4 * i] = readdata[3];
			user_data[4 * i + 1] = readdata[4];
			user_data[4 * i + 2] = readdata[5];
			fts_info(info, "user_data:%d %d %d", user_data[4 * i], user_data[4 * i + 1], user_data[4 * i + 2]);
		}
	}
	fts_info(info, "udd read %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u", user_data[0], user_data[1], user_data[2], user_data[3], user_data[4],
		user_data[5], user_data[6], user_data[7], user_data[8], user_data[9], user_data[10],
		user_data[11], user_data[12], user_data[13], user_data[14], user_data[15], user_data[16], user_data[17], user_data[18]);
	ret = 19;
ERROR:
	if (int_enabled) {
		res = ftm4_enableInterrupt(info);
		if (res < OK) {
			fts_info(info, "ftm4_enableInterrupt ERROR %08X \n", res | ERROR_ENABLE_INTER);
			return res;
		}
	}

	return ret;

}

static int ftm4_write_user_data(struct fts_ts_info *info, unsigned char *udd)
{
	int ret, res;
	u8 writedatacmd6byte1[8] = {0XCA, 00};
	u8 writedatacmd6byte2[8] = {0XCA, 06};
	u8 writedatacmd6byte3[8] = {0XCA, 12};
	u8 writedatacmd1byte[3] = {0XCA, 18};
	u8 writedatacmdtoflash[4] = {0xCB, 0x00, 19};
	u8 polynomial = 0x9B;
	u8 initCRCValue = 0x00;
	int i = 0;
	bool int_enabled = false;

	ftm4_isInterruptEnabled(info, &int_enabled);
	
	fts_info(info, "write udd: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u", udd[0], udd[1], udd[2], udd[3], udd[4], 
		udd[5], udd[6], udd[7], udd[8], udd[9], 
		udd[10], udd[11], udd[12], udd[13], udd[14], udd[15], udd[16], udd[17], udd[18]);
	
	writedatacmdtoflash[3] = ftm4_CalculateCRC8(udd, 19, polynomial, initCRCValue);

	for (i = 0; i < 6; i++) {
		writedatacmd6byte1[i+2] = udd[i];
	}
	for (i = 0; i < 6; i++) {
		writedatacmd6byte2[i+2] = udd[i+6];
	}
	for (i = 0; i < 6; i++) {
		writedatacmd6byte3[i+2] = udd[i+12];
	}
	for (i = 0; i < 1; i++) {
		writedatacmd1byte[i+2] = udd[i+18];
	}
	
	
	fts_info(info, "udd to write %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u", writedatacmd6byte1[2], writedatacmd6byte1[3], writedatacmd6byte1[4],
		writedatacmd6byte1[5], writedatacmd6byte1[6], writedatacmd6byte1[7], writedatacmd6byte2[2], writedatacmd6byte2[3], writedatacmd6byte2[4],
		writedatacmd6byte2[5], writedatacmd6byte2[6], writedatacmd6byte2[7], writedatacmd6byte3[2], writedatacmd6byte3[3], writedatacmd6byte3[4],
		writedatacmd6byte3[5], writedatacmd6byte3[6], writedatacmd6byte3[7], writedatacmd1byte[2]);

	if (int_enabled) {
		ret = ftm4_disableInterrupt(info);
		if (ret < OK) {
			fts_info(info, "ERROR %d", ret);
			goto ERROR;
		}
	}

	ret = ftm4_writeCmd(info, writedatacmd6byte1, 8);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		goto ERROR;
	}
	ret = ftm4_writeCmd(info, writedatacmd6byte2, 8);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		goto ERROR;
	}
	ret = ftm4_writeCmd(info, writedatacmd6byte3, 8);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		goto ERROR;
	}
	ret = ftm4_writeCmd(info, writedatacmd1byte, 3);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		goto ERROR;
	}
	ret = ftm4_writeCmd(info, writedatacmdtoflash, 4);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		//goto ERROR;
	}
	writedatacmdtoflash[1] = 01;
	ret = ftm4_writeCmd(info, writedatacmdtoflash, 4);
	if (ret < OK) {
		fts_info(info, "impossible write command... ERROR %02X", ret);
		//goto ERROR;
	}

	ret = 19;
ERROR:
	if (int_enabled) {
		res = ftm4_enableInterrupt(info);
		if (res < OK) {
			fts_err(info, " ftm4_enterGestureMode: ftm4_enableInterrupt ERROR %08X \n", res | ERROR_ENABLE_INTER);
			return res;
		}
	}

	return ret;
}


static int ftm4_update_firmware(struct vts_device *vtsdev, const struct firmware *firmware)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	Firmware ftm4_fw;
	ftm4_fw.data = NULL;
	
	if (!firmware->size || !firmware->data) {
		fts_err(info, "invalid args\n");
		return -EINVAL;
	}
	
	ftm4_irq_enable_disable(info, 0, 0);

	ret = ftm4_parseBinFile(info, firmware->data, firmware->size, &ftm4_fw, 1);
	if (ret < 0) {
		fts_err(info, "impossible parse ERROR");
		ftm4_irq_enable_disable(info, 1, 0);
		return (ret | ERROR_MEMH_READ);
	}

	ret = ftm4_flash_burn(info, ftm4_fw, 1, 1);
	if (ret < 0 && ret != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
		fts_err(info, "ERROR ");
		/*return (res | ERROR_FLASH_PROCEDURE);*/
		ret = -1;
	}

	fts_info(info, "flashing procedure Finished!\n");
	if (ftm4_fw.data != NULL)
		kfree(ftm4_fw.data);

	if (fts_need_caliberate(info)) {
		if (fts_chip_initialization(info) >= 0)
			fts_info(info, "Pass.Calibration pass.");
		else
			fts_err(info, "Fail.Calibration fail.");

		ret = fts_save_mp_flag(info, INIT_MP);
		if (ret < 0)
			fts_err(info, "Fail! Error while saving the MP flag!\n");
	}

	ftm4_senseOn(info);
	ftm4_irq_enable_disable(info, 1, 0);
	ftm4_enableInterrupt(info);
	return ret;
}
static int ftm4_get_touch_ic_mode(struct vts_device *vtsdev)
{
		struct fts_ts_info *info = vts_get_drvdata(vtsdev);
		int ret = 0;
		unsigned char cmd[2] = {0xc8, 0x02};
		unsigned char result[16] = {0};
		u8 readdatacmd = 0x85;
		int i = 0;
		bool int_enabled = false;
		ftm4_isInterruptEnabled(info, &int_enabled);
		if (int_enabled) {
			ret = ftm4_disableInterrupt(info);
			if (ret < OK) {
				fts_err(info, "fts_disableInterrupt fail");
				goto Fail;
			}	
		}
	
		ret = ftm4_writeCmd(info, cmd, 2);
		if (ret < OK) {
			fts_err(info, "impossible write command...");
			goto Fail;
		}
	
		for (i = 0; i < 32; i++) {
			ret = ftm4_readCmd(info, (u8 *)(&readdatacmd), 1, result, 8);
			if (ret < OK) {
				fts_err(info, "impossible read data...");
				break;
			}
			fts_info(info, "get data:0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", result[0], result[1], result[2], result[3], result[4], result[5], result[6]);
	
			if (result[0] == 0x17 && result[1] == 2) {
			if(result[2] & BIT(4)){
				VTI("NORMAL MODE ");
				ret = 1;
				break;
				
			}
			if(result[2] & BIT(3)){
				VTI("GESTURE MODE ");
				ret = 0;
				break;
			}				
			}
		}
	
	Fail:
		if (int_enabled)
			ftm4_enableInterrupt(info);
		return ret;
}

static int ftm4_get_ic_status(struct vts_device *vtsdev, int *status)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	unsigned char cmd[2] = {0xc8, 0x02};
	unsigned char result[16] = {0};
	u8 readdatacmd = 0x85;
	int i = 0;
	bool int_enabled = false;
	*status = 0;

	ftm4_isInterruptEnabled(info, &int_enabled);
	if (int_enabled) {
		ret = ftm4_disableInterrupt(info);
		if (ret < OK) {
			fts_err(info, "fts_disableInterrupt fail");
			goto Fail;
		}	
	}

	ret = ftm4_writeCmd(info, cmd, 2);
	if (ret < OK) {
		fts_err(info, "impossible write command...");
		goto Fail;
	}

	for (i = 0; i < 32; i++) {
		ret = ftm4_readCmd(info, (u8 *)(&readdatacmd), 1, result, 8);
		if (ret < OK) {
			fts_err(info, "impossible read data...");
			break;
		}
		fts_info(info, "get data:0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", result[0], result[1], result[2], result[3], result[4], result[5], result[6]);
		if (result[0] == 0x17 && result[1] == 2) {
			if (result[2] & BIT(0))
				*status |= VTS_IC_STATUS_WATER_MODE;
			if (result[2] & BIT(1))
				*status |= VTS_IC_STATUS_NOISE_MODE;
			if (result[2] & BIT(2))
				*status |= VTS_IC_STATUS_DOZE_MODE;
			break;
		}
	}

Fail:
	if (int_enabled)
		ftm4_enableInterrupt(info);
	return ret;
}


static int ftm4_other_info(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	int status;
	int ret;

	ret = ftm4_get_ic_status(vtsdev, &status);
	if (ret)
		return ret;

	return snprintf(buf, nbytes, "active:gesture:idle:charger:water(bit0)\n0x%x\n", status);
}

static int get_grip_area(struct vts_device *vtsdev, struct vts_grip_area *grip_area)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int i, ret = 0;
	u8 readdata[FIFO_EVENT_SIZE*4] = {0x00};
	u8 cmd_get[2] = {0xCF, 0x01};
	u8 cmd_reset[2] = {0xCF, 0x02};
	u16 address = 0x7FF0;

	if (ftm4_writeCmd(info, cmd_get, sizeof(cmd_get)) < 0) {
		fts_err(info, "send CF 01 failed ERROR %02X\n", ERROR_I2C_W);
		ret = ERROR_I2C_W;
		goto END;
    }

	for (i = 0; i < 10; i++) {
		mdelay(5);
		ret= ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, address, readdata, /*size*/sizeof(readdata), DUMMY_FRAMEBUFFER);
		if (ret < OK) {
			fts_err(info, "getData failed: ERROR %02X\n", ERROR_I2C_R);
			ret = ERROR_I2C_R;
			goto END;
		}
		if ((readdata[0] != 0xFD) ||  (readdata[1] != 0xFD) || (readdata[2] != 0xFD) || (readdata[3] != 0xFD)) {
			fts_info(info, "getData right data\n");
			ret =OK;
			break;
		}
	}

	if(i >= 10){
		fts_err(info, "can not get the data\n");
		ret = -1;
		goto END;
	}else{
		grip_area->area_edge = (readdata[1] << 8) + readdata[0];
		grip_area->area_center = (readdata[3] << 8) + readdata[2];
		fts_info(info, "alredy get grip_area_edge: %d, grip_area_center: %d\n", grip_area->area_edge, grip_area->area_center);
	}

END:
	if (ftm4_writeCmd(info, cmd_reset, sizeof (cmd_reset)) < 0) {
        fts_err(info, "send CF 02 failed ERROR %02X\n", ERROR_I2C_W);
        ret= ERROR_I2C_W;
    }
	return ret;
}

static int ftm4_game_mode_ctl(struct fts_ts_info *info, int state)
{
	unsigned char enable_cmd[3] = {0xc1, 0x00, 0x04};
	unsigned char disable_cmd[3] = {0xc2, 0x00, 0x04};
	int ret = 0;

	if (state == 1) {
		fts_info(info, "enter game mode");
		info->gama_mode_state = 1;
		ret = ftm4_writeCmd(info, enable_cmd, 3);
		if (ret < OK) {
			fts_info(info, "impossible write command...");
			return ret;
		}
	} else if (state == 0) {
		fts_info(info, "exit game mode");
		info->gama_mode_state = 0;
		ret = ftm4_writeCmd(info, disable_cmd, 3);
		if (ret < OK) {
			fts_info(info, "impossible write command...");
			return ret;
		}
	}
	return 0;
}

static int fts_process_by_package(struct vts_device *vtsdev, unsigned char *cmd)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	fts_info(info, "get cmd:%s", cmd);

	if (strcmp(cmd, "VivoGameMode:0") == 0) {
		ftm4_game_mode_ctl(info, 0);
	}
	if (strcmp(cmd, "VivoGameMode:1") == 0) {
		ftm4_game_mode_ctl(info, 1);
	}

	return 0;	
}

static int ftm4_long_press_enable(struct vts_device *vtsdev, int enable)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	u8 enable_long_press_cmd[10] = {0xc3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00};
	u8 disable_long_press_cmd[10] = {0xc3, 0x02, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00};
	u8 enable_fast_mode_cmd[3] = {0xC3, 0x06, 0x01};
	u8 disable_fast_mode_cmd[3] = {0xC3, 0x06, 0x00};
	int ret;
	bool int_enable = false;

	fts_info(info, "enable:%d", enable);
	ftm4_isInterruptEnabled(info, &int_enable);

	if (int_enable)
		ftm4_disableInterrupt(info);

	/* enable or disable long press gesture */
	ret = ftm4_writeCmd(info, enable ? enable_long_press_cmd : disable_long_press_cmd, 10);
	if (ret < 0) {
			fts_info(info, "writeCmd  fail!");
			ret = ERROR_I2C_W;
			goto END;
	}

	/* enable or disable long press fast mode */
	ret = ftm4_writeCmd(info, enable ? enable_fast_mode_cmd : disable_fast_mode_cmd, 3);
	if (ret < 0) {
			fts_info(info, "writeCmd  fail!");
			ret = ERROR_I2C_W;
			goto END;
	}

END:
	if (int_enable)
		ftm4_enableInterrupt(info);
	return ret;
}

/*for ear detect*/
static int ftm4_earDetect_start_end(struct vts_device *vtsdev, int state)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	u8 cmd1[2] = {0xCD, 0x0C};/*cmd for start*/
	u8 cmd2[2] = {0xCD, 0x0F};/*cmd for end*/
	bool int_enabled = false;

	fts_info(info, "get cmd:%d,---1 is start, 0 is end", state);
	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled)
		ftm4_disableInterrupt(info);
	if (state == 1) {
		/*set cmd for start*/
		if (ftm4_writeCmd(info, cmd1, 2) < 0) {
				fts_info(info, "writeCmd 0xCD,0x0C fail!");
				ret = ERROR_I2C_W;
				goto END;
		}

		ftm4_idle_switch_write(vtsdev, 0);
	} else if (state == 0) {
		/*set cmd 0 for disable*/
		if (ftm4_writeCmd(info, cmd2, 2) < 0) {
				fts_info(info, "writeCmd 0xCD,0x0F fail!");
				ret = ERROR_I2C_W;
				goto END;
		}

		ftm4_idle_switch_write(vtsdev, 1);
	} else {
		fts_info(info, "The input param is invalid");
	}
	
END:
	if (int_enabled)
		ftm4_enableInterrupt(info);
	return ret;

}

static int ftm4_hw_init(struct vts_device *vtsdev)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int retval;
	int error;

	retval = fts_set_gpio(info, true);
	if (retval < 0) {
		fts_err(info, "ERROR Failed to set up GPIO's");
		return retval;
	}

	retval = fts_mix_set_power_state(info, true);
	if (retval < 0) {
		fts_err(info, "ERROR Failed to enable regulators");
		fts_set_gpio(info, false);
		return retval;
	}

	retval = fts_init(info);
	if (retval < OK) {
		fts_err(info, "Cannot initialize the device ERROR %08X", retval);
		fts_mix_set_power_state(info, false);
		fts_set_gpio(info, false);
		return retval;
	}

	retval = fts_fw_update_auto(info);
	if (retval == ERROR_GET_INIT_STATUS)	{ /* initialization status not correct or after FW complete update, do initialization. */
		error = fts_chip_initialization(info);
		if (error < OK) {
			fts_err(info, "Cannot initialize the chip ERROR %08X", error);
		}
	}
	error = fts_init_hw(info);
	if (error < OK) {
		fts_err(info, "Cannot initialize the hardware device ERROR %08X", error);
	}

	return 0;
}

static int ftm4_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 19;
	return 0;
}

static ssize_t ftm4_flash_read(struct vts_device *vtsdev, u8*buf, size_t nbytes)
{
	unsigned char udd[19] = {0};
	int i = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	ftm4_read_user_data(info, udd);

	for (i = 0; i < 19; i++) {
		buf[i] = udd[i];
	}

	return nbytes;
}

static ssize_t ftm4_flash_write(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	unsigned char udd[19] = {0};
	int i;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	for (i = 0; i < 19; i++) {
		udd[i] = buf[i];
	}

	ftm4_write_user_data(info, udd);
	return nbytes;
}

static int ftm4_broken_disable(struct vts_device *vtsdev, int state)
{
	int error = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	mutex_lock(&(info->esdErrorMutex));
	fts_info(info, "broken disable ftm4.");

	mutex_lock(&(info->i2cResetMutex));
	error = fts_power_on(info, 0);
	if (error)
		fts_err(info, "fts_power_off error.");
	ftm4_reset_pin_ctl(info, 0);
	msleep(10);
	mutex_unlock(&(info->i2cResetMutex));

	ftm4_irq_enable_disable(info, 0, 1);
	mutex_unlock(&(info->esdErrorMutex));

	return error;
}

static const struct vts_operations fts_vts_ops = {
	.init = ftm4_hw_init,
	.update_firmware = ftm4_update_firmware,
	.change_mode = ftm4_mode_change,
	.get_frame = ftm4_get_rawordiff_data,
	.get_fw_version = ftm4_fw_version,
	.set_charging = ftm4_write_charger_flag,
	.set_rotation = ftm4_edge_restain_switch,
	.sensor_test = ftm4_sensor_test,
	.sensor_caliberate = ftm4_sensor_caliberate,
	.set_auto_idle = ftm4_idle_switch_write,
	.process_package = fts_process_by_package,
	.get_ic_status = ftm4_get_ic_status,
	.otherInfo = ftm4_other_info,
	.get_grip_status = get_grip_area,
	.set_virtual_prox = ftm4_earDetect_start_end,
	.set_long_press = ftm4_long_press_enable,
	.rom_size = ftm4_get_flash_size,
	.rom_read = ftm4_flash_read,
	.rom_write = ftm4_flash_write,
	.broken_disable = ftm4_broken_disable,
	.get_ic_mode = ftm4_get_touch_ic_mode,
};

static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct fts_ts_info *info = NULL;
	int error = 0;
	struct vts_device *vtsdev = NULL;

	fts_info(info, "driver probe begin!");

	info = kzalloc(sizeof (struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		fts_info(info, "no memory");
		return -ENOMEM;
	}

	mutex_init(&info->i2cResetMutex);
	mutex_init(&info->esdErrorMutex);
	info->touch_id = 0;
	info->gesture_enabled = 0;
	info->glove_enabled = 0;
	info->edge_suppress_enabled = 1;
	info->charger_enabled = 0;
	info->edge_pix_reject = 1;
	info->edge_corner_reject = 1;
	info->edge_L_reject = 1;
	info->resume_bit = 1;
	info->irq_state = 1;
	info->irq_wake_state = 0;
	info->client = client;
	i2c_set_clientdata(client, info);
	info->dev = &info->client->dev;
	ftm4_error_init(info);

	ftm4_InitGesture(info);
	ftm4_driver_test_init(info);
	parse_dt(&client->dev, info);
	info->client->irq = gpio_to_irq(info->bdata.irq_gpio);
	error = fts_get_reg(info, true);
	if (error < 0) {
		fts_err(info, "ERROR: Failed to get regulators");
		goto error1;
	}

	vtsdev = vts_device_alloc();
	if(!vtsdev) {
		fts_err(info, "vivoTsAlloc fail");
		error = -ENOMEM;
		goto error2;
	}

	vtsdev->ops = &fts_vts_ops;
	vtsdev->busType = BUS_I2C;
	info->vtsdev = vtsdev;
	vts_parse_dt_property(vtsdev, client->dev.of_node);
	vts_set_drvdata(vtsdev, info);
	ftm4_gui_init(info);
	error = vts_register_driver(vtsdev);
	if(error < 0) {		
		fts_err(info, "vts_register_driver failed");
		goto error3;
	}	

	fts_info(info, "Probe Finished!");
	return 0;

error3:
	ftm4_gui_deinit(info);
	vts_device_free(vtsdev);
error2:
	fts_get_reg(info, false);
	ftm4_driver_test_deinit(info);
error1:
	kfree(info);
	return error;
}

static int fts_remove(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);
/*sunsl
#ifdef DRIVER_TEST
	sysfs_remove_group(&info->test_cmd_dev->kobj,
			&test_cmd_attr_group);
#endif
 */
	/* sysfs stuff */
  /*  sysfs_remove_group(&client->dev.kobj, &info->attrs); */

	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);

	/*fb_unregister_client(&info->notifier); */

	/* unregister the device */
	/*sunsl input_unregister_device(info->input_dev); */

	/*intput_free_device(info->input_dev); */

	/* Empty the FIFO buffer */
	fts_command(info, FIFO_CMD_FLUSH);
	/*ftm4_flushFIFO(info); */

	/* Remove the work thread */
	/*destroy_workqueue(info->event_wq); */
	destroy_workqueue(info->fwu_workqueue);

	vts_unregister_driver(info->vtsdev);
	vts_device_free(info->vtsdev);
	fts_get_reg(info, false);
	ftm4_gui_deinit(info);
	ftm4_driver_test_deinit(info);
	kfree(info);
	return OK;
}

static struct of_device_id fts_of_match_table[] = {
	{
		.compatible = "stm,ftm4",
	},
	{},
};
static const struct i2c_device_id fts_device_id[] = {
	{FTS_TS_DRV_NAME, 0},
	{}
};

static struct i2c_driver fts_i2c_driver = {
	.driver = {
		.name = FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
	},
	.probe = fts_probe,
	.remove = fts_remove,
	.id_table = fts_device_id,
};
 
static const int ic_number[1] = {VTS_IC_FTM4};
module_vts_driver(ftm4, ic_number, i2c_add_driver(&fts_i2c_driver), i2c_del_driver(&fts_i2c_driver));

