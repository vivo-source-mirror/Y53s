#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/interrupt.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/wait.h>
//#include <linux/sched.h>
#include <uapi/linux/sched/types.h>


#include <linux/vivo_ts_function.h>
#include <linux/vivo_ts_ic_ctl.h>
#include <linux/vivo_ts_node.h>

#include <linux/bbk_drivers_info.h>
#include <linux/vivo_touchscreen_virtual_key.h>
#include <linux/input/vivo_key.h>
//#include <linux/msm_drm_notify.h>

static struct vivo_ts_struct *vivoTsData;

static unsigned int allGestureSwitchIntoBitmap(void);
static void vivoTsInputBLongPressAvoidLcdoff(int x, int y, int *z);
static int vtsIrqBtmThreadRegister(void);
static int vtsIrqBtmThreadStop(void);

int vivoTsGetState(void)
{
	return atomic_read(&vivoTsData->tsState);
}
//add by wanglanbin
int vivoTsGetLcdState(void)
{
	if (vivoTsData)
	   return vivoTsData->hasLcdShutoff;
	else
	   return 0;
}
//add end
struct vivo_ts_struct *vivoTsGetVtsData(void)
{
	return vivoTsData;
}

struct input_dev *vivoTsGetInputDev(void)
{
	if (vivoTsData->inputDev)
		return vivoTsData->inputDev;
	else
		VTI("inputDev is not alloc,please call vivoTsInit first.");

	return NULL;
}

int vivoTsReleasePoints(void)
{
	int i;

	VTI("release point.");

	mutex_lock(&vivoTsData->inputReportMutex);
	for (i = 0; i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		input_mt_slot(vivoTsData->inputDev, i);
		/* must add this line, else cause some exception points after resume */
		input_report_abs(vivoTsData->inputDev, ABS_MT_TRACKING_ID, -1);
	}
	input_report_key(vivoTsData->inputDev, BTN_TOUCH, 0);
	input_report_key(vivoTsData->inputDev, BTN_TOOL_FINGER, 0);
	input_sync(vivoTsData->inputDev);

	/* if suspend and resume,ic no free point,we need free and print point upEvent in suspend's end. */
	for (i = 0; i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		if (vivoTsData->fingerPressRecord[i][2] > 0) {
			vivoTsData->fingerPressRecord[i][2] = 0;
			VTI("TouchInfo[%d][0][%d][%d][%04x]", i,
				vivoTsData->fingerPressRecord[i][0], vivoTsData->fingerPressRecord[i][1],
				vivoTsData->fingerPressRecord[i][3]);
		}
	}

	mutex_unlock(&vivoTsData->inputReportMutex);

	/* add for fingerprint and independent key ic */
	if (vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		if (vivoTsData->keyType != VTS_KEY_NOKEY) {
			set_finger_on_2d(false);
			set_AA_release_time();
		}
	}
	if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
		if (vivoTsData->keyType != VTS_KEY_NOKEY) {
			set_finger_on_2d(false);
			set_AA_release_time();
		}
	}
	return 0;
}
int vivoTsReleasePointsAndKeys(void)
{
	int i = 0;

	VTI("release point and keys.");

	mutex_lock(&vivoTsData->inputReportMutex);
	for (i = 0; i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		input_mt_slot(vivoTsData->inputDev, i);
		/* must add this line, else cause some exception points after resume */
		input_report_abs(vivoTsData->inputDev, ABS_MT_TRACKING_ID, -1);
	}
	input_report_key(vivoTsData->inputDev, BTN_TOUCH, 0);
	input_report_key(vivoTsData->inputDev, BTN_TOOL_FINGER, 0);
	input_sync(vivoTsData->inputDev);
	
	/* release the finger gesture */
	if (vivoTsData->hasFingerGesture) {
		VTI("release finger gesture UP");
		input_report_key(vivoTsData->inputDevFp, 254, 0);
		input_sync(vivoTsData->inputDevFp);
	}
	mutex_unlock(&vivoTsData->inputReportMutex);

	/* just 0d key and not independent key need free keys */
	if ((VTS_KEY_0D == vivoTsData->keyType) && (0 == vivoTsData->isIndependentKeyIc)) {
		/* release key */
		if (vivoTsData->hasMenuKey)
			vivoTsInputReport(VTS_VIRTUAL_KEY, KEY_MENU, 0, -1, -1);
		if (vivoTsData->hasHomeKey)
			vivoTsInputReport(VTS_VIRTUAL_KEY, KEY_HOMEPAGE, 0, -1, -1);
		if (vivoTsData->hasBackKey)
			vivoTsInputReport(VTS_VIRTUAL_KEY, KEY_BACK, 0, -1, -1);
	}

	/* if suspend and resume,ic no free point,we need free and print point upEvent in suspend's end. */
	for (i = 0; i < VIVO_TS_MAX_TOUCH_NUM; i++) {
		if (vivoTsData->fingerPressRecord[i][2] > 0) {
			vivoTsData->fingerPressRecord[i][2] = 0;
			VTI("TouchInfo[%d][0][%d][%d][%d]", i,
				vivoTsData->fingerPressRecord[i][0], vivoTsData->fingerPressRecord[i][1],
				vivoTsData->fingerPressRecord[i][3]);
		}
	}

	/* add for fingerprint and independent key ic */
	if (vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		if (vivoTsData->keyType != VTS_KEY_NOKEY) {
			set_finger_on_2d(false);
			set_AA_release_time();
		}
	}
	if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
		if (vivoTsData->keyType != VTS_KEY_NOKEY) {
			set_finger_on_2d(false);
			set_AA_release_time();
		}
	}

	return 0;
}

/*
 * just call this function to print touch info,By imitate this logic,we could achieve TouchInfo print easy
 * is_down:means press(1) or not press(0)
 *
 * desc:
 * find report down pleace and insert vivo_ts_touch_info_print(touch_id, x, y, 1);
 * find report up place and insert vivo_ts_touch_info_print(touch_id, NC, NC, 0);
 */
static void vivoTsTouchInfoPrint(int touchId, int x, int y, int z, int isDown)
{
	if (touchId < 0 || touchId >= VIVO_TS_MAX_TOUCH_NUM) {
		VTI("out of VIVO_TS_MAX_TOUCH_NUM=10");
		return;
	}

	if (1 == isDown) {
		vivoTsData->fingerPressRecord[touchId][0] = x;
		vivoTsData->fingerPressRecord[touchId][1] = y;
		vivoTsData->fingerPressRecord[touchId][3] = z;
		/* just print log while first report this finger */
		vivoTsData->fingerPressRecord[touchId][2]++;		/* report rate=80,so 80*60*60=288000timers/hour */
		if (vivoTsData->fingerPressRecord[touchId][2] == 1) {
			VTI("TouchInfo[%d][1][%4d][%4d][%04x]", touchId,
				vivoTsData->fingerPressRecord[touchId][0], vivoTsData->fingerPressRecord[touchId][1],
				vivoTsData->fingerPressRecord[touchId][3]);
		} else {
			VTD("TouchInfo[%d][2][%4d][%4d][%04x]", touchId,
				vivoTsData->fingerPressRecord[touchId][0], vivoTsData->fingerPressRecord[touchId][1],
				vivoTsData->fingerPressRecord[touchId][3]);
		}
	}
	if (0 == isDown) {
		if (vivoTsData->fingerPressRecord[touchId][2] > 0) {
			vivoTsData->fingerPressRecord[touchId][2] = 0;		/* clear count,report rate=80,so 80*60*60=288000timers/hour */
			VTI("TouchInfo[%d][0][%4d][%4d][%04x]", touchId,
				vivoTsData->fingerPressRecord[touchId][0], vivoTsData->fingerPressRecord[touchId][1],
				vivoTsData->fingerPressRecord[touchId][3]);
		}
	}
}

static int x_width, y_width;
static int vivoTsInputReport2Called;
int vivoTsInputReport2(int event, int touchIdOrGestureEvent, int x, int y, int z, int x_size, int y_size)
{
	int ret = 0;
	x_width = x_size;
	y_width = y_size;
	vivoTsInputReport2Called = 1;
	ret = vivoTsInputReport(event, touchIdOrGestureEvent, x, y, z);
	vivoTsInputReport2Called = 0;

	return ret;
}
/*
 * #define VTS_TOUCH_UP 0
 * #define VTS_TOUCH_DOWN 1
 * #define VTS_GESTURE_EVENT 2
 * #define VTS_VIRTUAL_KEY  5
 * paramater formate
 * VTS_TOUCH_DOWN touchId x y z
 * VTS_TOUCH_UP touchId nc nc nc	(nc recommend 0 0 0)
 * VTS_GESTURE_EVENT:whichGesture nc nc nc  (nc recommend -1 -1 -1)
 * VTS_VIRTUAL_KEY:whichKey down/up(1/0) nc nc
 *****/
static int firstDownY;
void vivoTsInputSync(void)
{
	input_sync(vivoTsData->inputDev);
}
int vivoTsInputReport(int event, int touchIdOrGestureEvent, int x, int y, int z)
{
	int needReportGesture = 0;

	/* VTD("event:%d touchIdOrGestureEvent:%d x:%d y:%d z:%d", event, touchIdOrGestureEvent, x, y, z); */

	mutex_lock(&vivoTsData->inputReportMutex);
	switch (event) {
	case VTS_TOUCH_UP:
		if (touchIdOrGestureEvent >= VIVO_TS_MAX_TOUCH_NUM) {
			VTI("error!touch id out of range");
			break;
		}
		input_mt_slot(vivoTsData->inputDev, touchIdOrGestureEvent);
		input_report_abs(vivoTsData->inputDev, ABS_MT_TRACKING_ID, -1);
		if (vivoTsData->syncFlag == 0) {
			input_sync(vivoTsData->inputDev);
		}
		vivoTsTouchInfoPrint(touchIdOrGestureEvent, x, y, z, 0);
		if (touchIdOrGestureEvent == 0) {
			if (vivoTsData->keyType != VTS_KEY_NOKEY) {
				VTD("key exception:one finger up.Y=%d", y);
				if ((y > (vivoTsData->tsDimensionY - VTS_VKEY_OPTIMIZE_WIDTH)) && (firstDownY > (vivoTsData->tsDimensionY - VTS_VKEY_OPTIMIZE_WIDTH))) {
					VTD("key exception:down and up is all local into 5 pixel, set_finger_on_2d:false");
					set_finger_on_2d(false);
					set_AA_release_time();
					firstDownY = 0;
				}
			}
		}

		break;
	case VTS_TOUCH_DOWN:
		if (touchIdOrGestureEvent >= VIVO_TS_MAX_TOUCH_NUM) {
			VTI("error!touch id out of range");
			break;
		}
		vivoTsInputBLongPressAvoidLcdoff(x, y, &z);
		input_mt_slot(vivoTsData->inputDev, touchIdOrGestureEvent);
		input_report_key(vivoTsData->inputDev, BTN_TOUCH, 1);
		input_report_key(vivoTsData->inputDev, BTN_TOOL_FINGER, 1);

		input_report_abs(vivoTsData->inputDev, ABS_MT_TRACKING_ID, touchIdOrGestureEvent);
		input_report_abs(vivoTsData->inputDev, ABS_MT_POSITION_X, x);
		input_report_abs(vivoTsData->inputDev, ABS_MT_POSITION_Y, y);
			if (vivoTsInputReport2Called) {
				input_report_abs(vivoTsData->inputDev, ABS_MT_TOUCH_MAJOR, (y_width<<16) | (x_width & 0X0000FFFF));
				input_report_abs(vivoTsData->inputDev, ABS_MT_WIDTH_MAJOR, y_width);
			} else {
				input_report_abs(vivoTsData->inputDev, ABS_MT_TOUCH_MAJOR, z & 0xff);
			}
		input_report_abs(vivoTsData->inputDev, ABS_MT_PRESSURE, z & 0xff);
		if (vivoTsData->syncFlag == 0) {
			input_sync(vivoTsData->inputDev);
		}
		vivoTsTouchInfoPrint(touchIdOrGestureEvent, x, y, z, 1);
		if (touchIdOrGestureEvent == 0) {
			if (vivoTsData->keyType != VTS_KEY_NOKEY) {
				VTD("key exception:one finger down.Y=%d", y);
				if (y < (vivoTsData->tsDimensionY - VTS_VKEY_OPTIMIZE_WIDTH)) {
					set_finger_on_2d(true);
				}
				if (firstDownY == 0) {
					firstDownY = y;
				}
			}
		}
		break;
	case VTS_GESTURE_EVENT:
/*
#define  KEY_GESTURE_U		KEY_WAKEUP
#define  KEY_GESTURE_UP		KEY_UP
#define  KEY_GESTURE_DOWN		KEY_WAKEUP_SWIPE
#define  KEY_GESTURE_LEFT		KEY_LEFT
#define  KEY_GESTURE_RIGHT		KEY_RIGHT
#define  KEY_GESTURE_O		KEY_O
#define  KEY_GESTURE_E		KEY_E
#define  KEY_GESTURE_M		KEY_M
#define  KEY_GESTURE_L		KEY_L
#define  KEY_GESTURE_W		KEY_W
#define  KEY_GESTURE_S		KEY_S
#define  KEY_GESTURE_V		KEY_V
#define  KEY_GESTURE_Z		KEY_Z
#define	 KEY_GESTURE_C		KEY_C
*/
/* get
* byte 0:dclick
* byte 1:udf switch
* byte 2:swip switch
* byte 4:c e m w o up LR(bit 0)
*/

		if (KEY_WAKEUP == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_WAKEUP");
		if (KEY_UP == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_UP");
		if (KEY_WAKEUP_SWIPE == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_WAKEUP_SWIPE");
		if (KEY_LEFT == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_LEFT");
		if (KEY_RIGHT == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_RIGHT");
		if (KEY_O == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_O");
		if (KEY_E == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_E");
		if (KEY_M == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_M");
		if (KEY_W == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_W");
		if (KEY_C == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_C");
		if (KEY_A == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_A");
		if (KEY_F == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_F");
		if (KEY_CUSTOM_GESTURE == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_CUSTOM_GESTURE");
		if (KEY_TS_SWIPE == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_TS_SWIPE");
		if (KEY_TS_LARGE_SUPPRESSION == touchIdOrGestureEvent)
			VTI("ic report gesture:KEY_TS_LARGE_SUPPRESSION");
		if (vivoTsData->hasFingerGesture) {
			if (245 == touchIdOrGestureEvent)
				VTI("ic report gesture:FINGER_GESTURE:%d", x);
		}

		if ((KEY_WAKEUP == touchIdOrGestureEvent) && vivoTsData->dClickSwitch)
			needReportGesture = 1;
		if ((KEY_UP == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x02))
			needReportGesture = 1;
		/*¡¡for long down swipe,camera */
		if ((KEY_WAKEUP_SWIPE == touchIdOrGestureEvent) && vivoTsData->swipeSwitch)
			needReportGesture = 1;
		if ((KEY_LEFT == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x01))
			needReportGesture = 1;
		if ((KEY_RIGHT == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x01))
			needReportGesture = 1;
		if ((KEY_O == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x04))
			needReportGesture = 1;
		if ((KEY_E == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x20))
			needReportGesture = 1;
		if ((KEY_M == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x10))
			needReportGesture = 1;
		if ((KEY_W == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x08))
			needReportGesture = 1;
		if ((KEY_C == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x40))
			needReportGesture = 1;
		if ((KEY_A == touchIdOrGestureEvent) && (vivoTsData->exportSwitch & 0x02))
			needReportGesture = 1;
		if ((KEY_F == touchIdOrGestureEvent) && (vivoTsData->exportSwitch & 0x04))
			needReportGesture = 1;
		if ((KEY_CUSTOM_GESTURE == touchIdOrGestureEvent) && vivoTsData->udfGestureSwitch)
			needReportGesture = 1;
		/* for short down swipe */
		if ((KEY_TS_SWIPE == touchIdOrGestureEvent) && (vivoTsData->gestureSwitch & 0x80)) {
			needReportGesture = 1;
		}
		if (KEY_TS_LARGE_SUPPRESSION == touchIdOrGestureEvent) {
			needReportGesture = 1;
		}
		/* for finger gesture */
		if (vivoTsData->hasFingerGesture) {
			if (254 == touchIdOrGestureEvent) {
				if (vivoTsGetState() == TOUCHSCREEN_NORMAL) {	/* normal mode:always report finger gesture,no gesture switch filter*/
					if (0 == x) {
						input_report_key(vivoTsData->inputDevFp, 254, 0);
						input_sync(vivoTsData->inputDevFp);
						VTI("input report finger gesture UP");
					} else if (1 == x) {
						input_report_key(vivoTsData->inputDevFp, 254, 1);
						input_sync(vivoTsData->inputDevFp);
						VTI("input report finger gesture DOWN");
					}
				} else if (vivoTsGetState() == TOUCHSCREEN_GESTURE) {
					if (vivoTsData->exportSwitch & 0x01) {
						if (0 == x) {
							input_report_key(vivoTsData->inputDevFp, 254, 0);
							input_sync(vivoTsData->inputDevFp);
							VTI("input report finger gesture UP");
						} else if (1 == x) {
							input_report_key(vivoTsData->inputDevFp, 254, 1);
							input_sync(vivoTsData->inputDevFp);
							VTI("input report finger gesture DOWN");
						}
					}
				}
			}
		}
		
		if (needReportGesture && (vivoTsData->commonGestureOff == 0)) {
			input_report_key(vivoTsData->inputDev, touchIdOrGestureEvent, 1);
			input_sync(vivoTsData->inputDev);
			input_report_key(vivoTsData->inputDev, touchIdOrGestureEvent, 0);
			input_sync(vivoTsData->inputDev);
			VTI("input report gesture:%d", touchIdOrGestureEvent);
		}
		break;
	case VTS_VIRTUAL_KEY:
		break;
	default:
		VTI("no this event:%d", event);
		break;
	}
	mutex_unlock(&vivoTsData->inputReportMutex);

	if (VTS_VIRTUAL_KEY == event) {
		if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
			if (touchIdOrGestureEvent == KEY_BACK) {
				if (1 == x) {	/* down */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 1,  1);	/* down */
				} else {	/* up */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 1,  0);	/* up */
				}
			}
			if (touchIdOrGestureEvent == KEY_MENU) {
				if (1 == x) {	/* down */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 2,  1);	/* down */
				} else {		/* up */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 2,  0);
				}
			}
			if (touchIdOrGestureEvent == KEY_HOMEPAGE) {
				if (1 == x) {	/* down */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 0,  1);
				} else {		/* up */
					vivo_key_processor(&(vivoTsData->virtualKeyProcessor), 0,  0);
				}
			}
		}
	}

	return 0;
}

/*
 * just call this function to achieve cover_mute function,By imitate this logic,we could achieve TouchInfo print easy
 * point_state:include touch finger number and larger press state
 *
 * note:
 * 1.gt9286 must insert in 2 place,and we can serch by "vivo_ts_cover_mute" in this driver
 * para:
 *	pointNum:touch finger num
 * 	isLargePressMode:1,large 0,not large
 */
int vivoTsCoverMute(u8 pointNum, u8 isLargePressMode)
{
	int ret = 0;
	VTD("pointNum=%d RT_isLargePressMode=%d", pointNum, isLargePressMode);

	/* avoid after ts_suspend's touch irq make 3finger of largertouch flag been set to 1 */
	if (vivoTsGetState() != TOUCHSCREEN_NORMAL) {
		return 0;
	}

	if ((0 == isLargePressMode) && (pointNum < 3)) {
		vivoTsCoverMuteRelease();
	}

	/* add for finger print and independent ic virtual key */
/*	if(vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		if(pointNum) {
			set_finger_on_2d(true);
		} else {
			set_finger_on_2d(false);
			set_AA_release_time();
		}
	}
*/

	if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || vivoTsData->icNum == VTS_IC_S3706 || vivoTsData->icNum == VTS_IC_NT36525
		|| vivoTsData->icNum == VTS_IC_SEC_TDDI) {
		if (vivoTsData->isCallingSave && isLargePressMode) {
			vivoTsData->isCalling = vivoTsData->isCallingSave;
		}
	}

	if (pointNum >= 3) {
		vivoTsData->is3FingerMode = 1;

		/* here if largpressmode impl in fw,must add ic macro here */
		if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || vivoTsData->icNum == VTS_IC_S3706 || vivoTsData->icNum == VTS_IC_NT36525
			|| vivoTsData->icNum == VTS_IC_SEC_TDDI) {
			vivoTsData->isLargePressMode = 0;
		}
	} else {
		vivoTsData->is3FingerMode = 0;
	}

	if ((isLargePressMode == 0) && (vivoTsData->isLargePressMode == 1) && (pointNum == 0)) {
		vivoTsData->isLargePressMode = 0;
	}

	if (isLargePressMode && vivoTsData->isLargePressMode == 0) {
		vivoTsData->isLargePressMode = 1;
	}

	if (vivoTsData->isCalling == 1) {
		VTI("is3FingerMode=%d isLargePressMode=%d",
			vivoTsData->is3FingerMode, vivoTsData->isLargePressMode);
		if (vivoTsData->is3FingerMode == 1 || vivoTsData->isLargePressMode == 1) {
			VIVO_TS_LOG_INF("[%s]report large press event.\n", __func__);
			vivoTsReleasePoints();
			vivoTsInputReport(VTS_GESTURE_EVENT, KEY_TS_LARGE_SUPPRESSION, -1, -1, -1);
			vivoTsData->isCalling = 0;
			ret = -1;
		return ret;
		}
	}

	if ((vivoTsData->isLargePressMode == 1) && (vivoTsData->icNum != VTS_IC_FT5446) && (vivoTsData->icNum != VTS_IC_FT8006) && (vivoTsData->icNum != VTS_IC_FT8719)) {
		vivoTsReleasePoints();
		ret = -2;
		return ret;
	}

	/* add for proximity and is calling, lcd shutoff,ts not report */
	if (vivoTsData->hasLcdShutoff) {
		/* there must not call vivoTsReleasePoints(),else cause lock UI exceptions. */
		ret = -3;
		return ret;
	}

	return 0;
}
void vivoTsCoverMuteRelease(void)
{
	VTD("enter.");
	vivoTsData->isLargePressMode = 0;
	vivoTsData->is3FingerMode = 0;
	vivoTsData->isCalling = vivoTsData->isCallingSave;
/*	vivoTsReleasePoints();	//must not release.else cause mutex wait mutex unlock dead lock */
}

/*
 * in wechat,if set 15s lcd off,while press ts not move,maybe lcd will off.
 * usage:
 * 	this function just inset before input system report down event.it's easy to use!
 * note:
 * 	paramater z must trans a int point
 */
static void vivoTsInputBLongPressAvoidLcdoff(int x, int y, int *z)
{
	static int report_data_sum;
	int cur_report_data_sum = 0;
	static long long diff_time;

	VIVO_TS_LOG_DBG("[%s]enter.", __func__);
	cur_report_data_sum = x + y + *z;
	if (report_data_sum == cur_report_data_sum) {
		if (jiffies-diff_time > (7 * HZ)) {
			*z = *z + 1;
			VIVO_TS_LOG_DBG("[%s]7 sec no move,so z++", __func__);
		}
	} else {
		diff_time = jiffies;
	}
	report_data_sum = x + y + *z;
}

#if defined(CONFIG_FB)
extern struct mutex NotifyTsThreadMutex;

#if 0
static int vivoTsFbNotifierCallback2(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank = NULL;
	static int fb_suspend_flag;
	int ret = 0;
	int id = 0;

	VTD("enter");
	if (evdata == NULL) {
		VTI("evdata is NULL.");
		return 0;
	}
	if (evdata->data == NULL) {
		VTI("evdata->data is NULL.");
		return 0;
	}

	id = evdata->id;
	if (id != 0) {
		VTD("not real callback");
		return 0;	
	}

	/* resume */
	if (event == 1) {	/*wake_p*/
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == 0 && fb_suspend_flag == 1) {
			VTI("blank=%d event=%ld", *blank, event);
			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_FT8006 || vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || vivoTsData->icNum == VTS_IC_FT8719) {
				VTI("lcmResumeAndProximityMutex unlock");
				mutex_unlock(&(vivoTsData->lcmResumeAndProximityMutex));
				if (vivoTsData->notifyMutexSwitch == 1) {
					VTI("NotifyTsThreadMutex unlock");
					mutex_unlock(&NotifyTsThreadMutex);
				}
			}
			fb_suspend_flag = 0;
			vivoTsData->suspendProximityChangeTimes = 0;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 1);
		}
	} else if (event == 2) {		/* early */
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == 0 && fb_suspend_flag == 1) {
			/* add for proximity power down and resume iic access at the same time.ref vivo driver document-0002 */
			atomic_set(&vivoTsData->tsState, TOUCHSCREEN_NORMAL);
			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_FT8006 || vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || vivoTsData->icNum == VTS_IC_FT8719) {
				VTI("blank=%d event=%ld lcmResumeAndProximityMutex lock", *blank, event);
				atomic_set(&vivoTsData->tsCallBackRTState, CALLBACK_RESUME);
				if (vivoTsData->notifyMutexSwitch == 1) {
					mutex_lock(&NotifyTsThreadMutex);
					VTI("NotifyTsThreadMutex lock");
				}
				mutex_lock(&(vivoTsData->lcmResumeAndProximityMutex));
			}
		}
	}
	/* suspend */
	if (event == 1) {	/* wake_p */
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == 1 && fb_suspend_flag == 0) {
			VTI("blank=%d event=%ld", *blank, event);
			fb_suspend_flag = 1;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 0);

			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || vivoTsData->icNum == VTS_IC_FT8006 || vivoTsData->icNum == VTS_IC_NT36772 || vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || vivoTsData->icNum == VTS_IC_FT8719) {
				VTI("wait tp suspend done");
				ret = wait_event_interruptible_timeout(vivoTsData->lcmTpSyncWaitQueue, vivoTsData->lcmTpSyncFlag != VTS_RESUME_WAIT, 2000);
				if (ret == 0) {
					VTI("wait timeout");
#ifdef	VIVO_TOUCH_COLLECT_LOG
					collectTouchException(1, 3);
#endif
				}
				vivoTsData->lcmTpSyncFlag = VTS_RESUME_WAIT;
				VTI("get tp wake up event");
			}
		}
	} else if (event == 2) {	/* eraly suspend */
		if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || vivoTsData->icNum == VTS_IC_NT36772 || vivoTsData->icNum == VTS_IC_NT36525) {
			blank = evdata->data;		/* 0:wakeup 4:suspend */
			if (*blank == 1 && fb_suspend_flag == 0) {
				if (vivoTsData->earlySuspendRun) {
					if (vivoTsData->notifyMutexSwitch == 1) {
						mutex_lock(&NotifyTsThreadMutex);
						VTI("NotifyTsThreadMutex lock");
					}
					vivoTsData->earlySuspendRun();
					if (vivoTsData->notifyMutexSwitch == 1) {
						VTI("NotifyTsThreadMutex unlock");
						mutex_unlock(&NotifyTsThreadMutex);
					}
				}

				/* add for proximity and is_calling */
				if (vivoTsData->isCallingSave) {
					VTI("is callingSave=1,wait 500ms to change mode");
					msleep(300);
				}
			}
		}
	}

	return 0;
}

#endif 

static int vivoTsFbNotifierCallback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	static int fb_suspend_flag;
	int ret = 0;

	VTD("enter");
	if (evdata == NULL) {
		VTI("evdata is NULL.");
		return 0;
	}
	if (evdata->data == NULL) {
		VTI("evdata->data is NULL.");
		return 0;
	}

	/* resume */
	if (event == vivoTsData->resumeEventBlank) {	/*wake_p*/
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == FB_BLANK_UNBLANK && fb_suspend_flag == 1) {
			VTI("blank=%d event=%ld", *blank, event);
			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_FT8006 || 
				vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || 
				vivoTsData->icNum == VTS_IC_FT8719 || vivoTsData->icNum == VTS_IC_SEC_TDDI || vivoTsData->icNum == VTS_IC_TD4330_N) {
				VTI("lcmResumeAndProximityMutex unlock");
				mutex_unlock(&(vivoTsData->lcmResumeAndProximityMutex));
				if (vivoTsData->notifyMutexSwitch == 1) {
					VTI("NotifyTsThreadMutex unlock");
					mutex_unlock(&NotifyTsThreadMutex);
				}
			}
			fb_suspend_flag = 0;
			vivoTsData->suspendProximityChangeTimes = 0;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 1);
		}
	} else if (event == 0x10) {		/* early */
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == FB_BLANK_UNBLANK && fb_suspend_flag == 1) {
			/* add for proximity power down and resume iic access at the same time.ref vivo driver document-0002 */
			atomic_set(&vivoTsData->tsState, TOUCHSCREEN_NORMAL);
			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_FT8006 || 
				vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || 
				vivoTsData->icNum == VTS_IC_FT8719 || vivoTsData->icNum == VTS_IC_SEC_TDDI || vivoTsData->icNum == VTS_IC_TD4330_N) {
				VTI("blank=%d event=%ld lcmResumeAndProximityMutex lock", *blank, event);
				atomic_set(&vivoTsData->tsCallBackRTState, CALLBACK_RESUME);
				if (vivoTsData->notifyMutexSwitch == 1) {
					mutex_lock(&NotifyTsThreadMutex);
					VTI("NotifyTsThreadMutex lock");
				}
				if (vivoTsData->icNum == VTS_IC_TD4330_N) {					
					vivoTsData->irqEnableOrDisable(false);
				}
				mutex_lock(&(vivoTsData->lcmResumeAndProximityMutex));
				VTI("blank=%d event=%ld lcmResumeAndProximityMutex lock", *blank, event);
			}
		}
	}
	/* suspend */
	if (event == vivoTsData->suspendEventBlank) {	/* wake_p */
		blank = evdata->data;		/* 0:wakeup 4:suspend */
		if (*blank == FB_BLANK_POWERDOWN && fb_suspend_flag == 0) {
			VTI("blank=%d event=%ld", *blank, event);
			fb_suspend_flag = 1;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 0);

			if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || 
				vivoTsData->icNum == VTS_IC_FT8006 || vivoTsData->icNum == VTS_IC_NT36772 || 
				vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || 
				vivoTsData->icNum == VTS_IC_FT8719 || vivoTsData->icNum == VTS_IC_SEC_TDDI
				|| vivoTsData->icNum == VTS_IC_NT36670 || vivoTsData->icNum == VTS_IC_TD4330_N) {
				VTI("wait tp suspend done");
				ret = wait_event_interruptible_timeout(vivoTsData->lcmTpSyncWaitQueue, vivoTsData->lcmTpSyncFlag != VTS_RESUME_WAIT, 2000);
				if (ret == 0) {
					VTI("wait timeout");
#ifdef	VIVO_TOUCH_COLLECT_LOG
					collectTouchException(1, 3);
#endif
				}
				vivoTsData->lcmTpSyncFlag = VTS_RESUME_WAIT;
				VTI("get tp wake up event");
			}
		}
	} else if (event == FB_EARLY_EVENT_BLANK) {	/* eraly suspend */
		if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || 
			vivoTsData->icNum == VTS_IC_NT36772 || vivoTsData->icNum == VTS_IC_NT36525 ||
			vivoTsData->icNum == VTS_IC_SEC_TDDI || vivoTsData->icNum == VTS_IC_NT36670) {
			blank = evdata->data;		/* 0:wakeup 4:suspend */
			if (*blank == FB_BLANK_POWERDOWN && fb_suspend_flag == 0) {
				if (vivoTsData->earlySuspendRun) {
					if (vivoTsData->notifyMutexSwitch == 1) {
						mutex_lock(&NotifyTsThreadMutex);
						VTI("NotifyTsThreadMutex lock");
					}
					vivoTsData->earlySuspendRun();
					if (vivoTsData->notifyMutexSwitch == 1) {
						VTI("NotifyTsThreadMutex unlock");
						mutex_unlock(&NotifyTsThreadMutex);
					}
				}

				/* add for proximity and is_calling */
				if (vivoTsData->isCallingSave) {
					VTI("is callingSave=1,wait 500ms to change mode");
					msleep(300);
				}
			}
		}
	}

	return 0;
}
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void vivoTsEarlySuspend(struct early_suspend *handler)
{
	touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 0);
	return;
}
static void vivoTsLateResume(struct early_suspend *handler)
{
	touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 1);
	return;
}
#endif

void vivoTsSetTxRxNum(int txNum, int rxNum)
{
	vivoTsData->rxNum = rxNum;
	vivoTsData->txNum = txNum;
}
int vivoTsGetTxRxNum(void)
{
	int ret = 0;
	int temp = 0;

	ret = vivoTsGetRxTxNumFromIc();
	if (-FUN_NOT_REG == ret) {
		temp = (vivoTsData->rxNum << 8) | vivoTsData->txNum;
	} else if (-1 == ret) {
		VTI("read rx/tx num from ic fail.");
		temp = (vivoTsData->rxNum << 8) | vivoTsData->txNum;
	} else if (ret > 0) {
		temp = ret;
	}

	return temp;
}

int vivoTsGetChargerFlag(void)
{
	return vivoTsData->usbChargerFlag;
}

static void vivoTsSetLogSwitch(bool on)
{
	vivoTsData->logSwitch  = on;
}
static struct bbk_drivers_callback_handler vivoTsLogSwitchHandler = {
	.name = "vivo_ts_function",
	.callback = vivoTsSetLogSwitch,
};

static int vivoTsGetLogSwitch(void)
{
	return vivoTsData->logSwitch;
}
static void VivoTsChargerConnectJudge(char on)
{
	VTI("charger detelct.");
	vivoTsData->usbChargerFlag = (int)on;

	/* add for firmware updating,while updating,not charger state switch. */
	if (vivoTsData->fwUpdatingFlag) {
		VTI("firmware is updating.no charger state switch.");
		return;
	}

	if (vivoTsData->isProbeComplete == 1) {
		schedule_work(&vivoTsData->chargerWork);
	}
}
static vivo_touchscreen_common_data vivoTsCommonData = {
	.driver_name = "vivo_ts_function",
	.get_ts_log_switch = vivoTsGetLogSwitch,
	.charge_connect_judge = VivoTsChargerConnectJudge,
};

static int vivoTsWriteUsbChargerFlag(void)
{
	int ret = 0;
	static int usb_change_count;

	VTI("write charge flag to chip.tsState:%d,usbChargerFlag=%d",
		atomic_read(&vivoTsData->tsState), vivoTsData->usbChargerFlag);

	/* if ts module not loade,but vivo_touchscreen_common.c loade and usb is charge while phone boot.
	 * must include this situation.
	 **/
	if (vivo_get_before_ts_load_usb_charger_flag() == 1) {
		vivoTsData->usbChargerFlag = 1;
		/* must clean after phone booted */
		vivo_clear_before_ts_load_usb_charger_flag();
	}

	/* just write in normal mode */
	if (atomic_read(&vivoTsData->tsState) == TOUCHSCREEN_NORMAL) {/* ts work normal,need to write charger */
		if (vivoTsData->usbChargerFlag == 1) {
			/* sent charger cmd */
			ret = vivoTsSetChargerFlagToChip(1);
		} else if (vivoTsData->usbChargerFlag == 0) {
			/* send no charger cmd */
			ret = vivoTsSetChargerFlagToChip(0);
		}
	}

	if (vivoTsData->usbChargerTestFlag == 0) {
		usb_change_count = 0;
	}
	vivoTsData->usbChargerTestFlag = (~(1 << usb_change_count)) & vivoTsData->usbChargerTestFlag;
	vivoTsData->usbChargerTestFlag = (((unsigned int)vivoTsData->usbChargerFlag) << usb_change_count) | vivoTsData->usbChargerTestFlag;
	usb_change_count++;
	if (16 == usb_change_count) {
		usb_change_count = 0;
	}

	return ret;
}

static void vivoTsChargerWorkHandler(struct work_struct *work)
{
	vivoTsWriteUsbChargerFlag();
}

/* input dev request */
static int vivoTsInputDeviceInit(struct i2c_client *client, void *tmpP, int tmp)
{
	int ret = -1;
	struct input_dev *inputDev = NULL;
	struct input_dev *inputDevFp = NULL;
	inputDev = input_allocate_device();
	if (inputDev == NULL) {
		VIVO_TS_LOG_ERR("[%s]failed to allocate input device.\n", __func__);
	    goto return_err;
	}

	inputDev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	    input_mt_init_slots(inputDev, VIVO_TS_MAX_TOUCH_NUM, INPUT_MT_DIRECT);
	#else
		input_mt_init_slots(inputDev, VIVO_TS_MAX_TOUCH_NUM);
	#endif
	VTI("intput max touch num=%d", VIVO_TS_MAX_TOUCH_NUM);

	/* add for b */
	inputDev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(BTN_TOOL_FINGER, inputDev->keybit);
	set_bit(INPUT_PROP_DIRECT, inputDev->propbit);

	/* register smart key,udf key*/
	input_set_capability(inputDev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);
	input_set_capability(inputDev, EV_KEY, KEY_CUSTOM_GESTURE);
	input_set_capability(inputDev, EV_KEY, KEY_WAKEUP);
	input_set_capability(inputDev, EV_KEY, KEY_WAKEUP_SWIPE);
	input_set_capability(inputDev, EV_KEY, KEY_LEFT);
	input_set_capability(inputDev, EV_KEY, KEY_RIGHT);
	input_set_capability(inputDev, EV_KEY, KEY_UP);
	input_set_capability(inputDev, EV_KEY, KEY_O);
	input_set_capability(inputDev, EV_KEY, KEY_W);
	input_set_capability(inputDev, EV_KEY, KEY_E);
	input_set_capability(inputDev, EV_KEY, KEY_M);
	input_set_capability(inputDev, EV_KEY, KEY_C);
	input_set_capability(inputDev, EV_KEY, KEY_F);
	input_set_capability(inputDev, EV_KEY, KEY_A);
	input_set_capability(inputDev, EV_KEY, KEY_TS_SWIPE);
	/* touch ic support dependent key */
	input_set_capability(inputDev, EV_KEY, KEY_MENU);
	input_set_capability(inputDev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(inputDev, EV_KEY, KEY_BACK);
	/* finger gesture */
	if (vivoTsData->hasFingerGesture) {
		input_set_capability(inputDev, EV_KEY, 254);
	}

	input_set_abs_params(inputDev, ABS_MT_POSITION_X, 0, vivoTsData->tsDimensionX-1, 0, 0);
	input_set_abs_params(inputDev, ABS_MT_POSITION_Y, 0, vivoTsData->tsDimensionY-1, 0, 0);
	input_set_abs_params(inputDev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(inputDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(inputDev, ABS_MT_TRACKING_ID, 0, VIVO_TS_MAX_TOUCH_NUM, 0, 0);
	VTI("intput ABS_MT_POSITION_X=%d", vivoTsData->tsDimensionX-1);
	VTI("intput ABS_MT_POSITION_Y=%d", vivoTsData->tsDimensionY-1);
	VTI("intput ABS_MT_PRESSURE=%d", 255);
	VTI("intput ABS_MT_TOUCH_MAJOR=%d", 255);
	VTI("intput ABS_MT_TRACKING_ID=%d", VIVO_TS_MAX_TOUCH_NUM);

	inputDev->name = "vivo_ts";
	inputDev->id.bustype = BUS_I2C;
	inputDev->id.vendor = vivoTsData->icNum;
	if (vivoTsData->busType == VTS_BUS_I2C) {
		inputDev->dev.parent = &client->dev;
	} else if (vivoTsData->busType == VTS_BUS_SPI) {
		//inputDev->dev.parent = ;	
	}

	ret = input_register_device(inputDev);
	if (ret) {
#ifdef	VIVO_TOUCH_COLLECT_LOG
		collectTouchException(1, 1);
#endif
		VTI("register %s input device failed.", inputDev->name);
		goto free_device;
	}

	vivoTsData->inputDev = inputDev;
	
	/* finger gesture */
	if (vivoTsData->hasFingerGesture) {
		inputDevFp = input_allocate_device();
		if (inputDevFp == NULL) {
			VIVO_TS_LOG_ERR("[%s]failed to allocate input device for fingerprint.\n", __func__);
			goto unregister_device;
		}
		inputDevFp->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
		input_set_capability(inputDevFp, EV_KEY, 254);
	
		inputDevFp->name = "vivo_ts_fp";
		inputDevFp->id.bustype = BUS_I2C;
		inputDevFp->id.vendor = vivoTsData->icNum;
		if (vivoTsData->busType == VTS_BUS_I2C) {
			inputDevFp->dev.parent = &client->dev;
		} else if (vivoTsData->busType == VTS_BUS_SPI) {
			//inputDev->dev.parent = ;	
		}

		ret = input_register_device(inputDevFp);
		if (ret) {
#ifdef	VIVO_TOUCH_COLLECT_LOG
			collectTouchException(1, 1);
#endif
			VTI("register %s input device failed.", inputDevFp->name);
			goto free_device_fp;
		}

		vivoTsData->inputDevFp = inputDevFp;
	}
	return 0;
	
free_device_fp:
	if (vivoTsData->hasFingerGesture) {
		input_free_device(inputDevFp);
	}

unregister_device:
	input_unregister_device(inputDev);
	vivoTsData->inputDev = NULL;
	inputDev = NULL;

free_device:
	if (inputDev != NULL)
		input_free_device(inputDev);

return_err:
	return -EINVAL;
}

static void vivoTsInputDeviceFree(void)
{
	input_unregister_device(vivoTsData->inputDev);
/************************************************
Don't call input_free_device() after
input_unregister_device()

	input_free_device() should only be used if
	input_register_device() was not called yet or if it failed. Once
	device was unregistered use input_unregister_device() and memory
	will be freed once last reference to the device is dropped.
************************************************/

/*	input_free_device(vivoTsData->inputDev); */
	vivoTsData->inputDev = NULL;
	
	if (vivoTsData->hasFingerGesture) {
		input_unregister_device(vivoTsData->inputDevFp);
		vivoTsData->inputDevFp = NULL;
	}
}

/* equal to scan_switch */
static int gestureAndSleepModeExchange(int needChangeToGestureMode)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = vivoTsData;

	// edge status init
	vtsData->edgeRestainSwitch = 1;

    VTI("%s enter", __func__);

	if (needChangeToGestureMode) {
		VTI("change to gesture mode.");
		atomic_set(&vtsData->tsState, TOUCHSCREEN_GESTURE);

		/* while firmeware updating,do not execute resume process */
		if (vtsData->fwUpdatingFlag) {
			VTI("firmware is updating.just set tsState,not suspend");
			return 0;
		}

		ret = vivoTsModeChange(TOUCHSCREEN_GESTURE);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsModeChange no define.");
		} else if (ret < 0) {
			VTI("vivoTsIcEnter GESTURE Mode fail.");
			return ret;
		}

		/* set gesture switch */
		ret = vivoTsSetGestureToChip(allGestureSwitchIntoBitmap());
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetGestureToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetGestureToChip fail.");
		}
		/* glove mode write */
		if (0 == vtsData->usbChargerFlag) {
			ret = vivoTsSetGloveModeToChip(vtsData->gloveModeSwitch);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		} else if (1 == vtsData->usbChargerFlag) {	/* if usb insert, close glove mode */
			ret = vivoTsSetGloveModeToChip(0);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		}
	} else {
		VTI("change to sleep mode.");
		atomic_set(&vtsData->tsState, TOUCHSCREEN_SLEEP);

		if (vtsData->fwUpdatingFlag) {
			VTI("firmware is updating.just set tsState,not suspend");
			return 0;
		}

		ret = vivoTsModeChange(TOUCHSCREEN_SLEEP);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsModeChange no define.");
		} else if (ret < 0) {
			VTI("vivoTsIcEnter SLEEP Mode fail.");
			return ret;
		}
	}

	return 0;
}
/*
 * byte 0:dclick
 * byte 1:udf switch
 * byte 2:swip switch
 * byte 4:c e m w o up LR
 */
static unsigned int allGestureSwitchIntoBitmap(void)
{
	if (vivoTsData == NULL) {	/* samsung TDDI need call this function in mdss,here need return zero if boot phone in AT mode */
		VTI("vivoTsData is NULL");
		return 0;
	}
	return ((vivoTsData->gestureSwitch << 24) | (vivoTsData->swipeSwitch << 16) | (vivoTsData->udfGestureSwitch << 8) | (vivoTsData->dClickSwitch));
}
static int vivoTsKthreadResumeSupend(void *vivoTsData, int which)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;

	if (which) {	/* resume */
		VTI("resume begin.");
		atomic_set(&vtsData->tsState, TOUCHSCREEN_NORMAL);
		/*  clean the finger gesture 2 min timeout flag */
		if (vtsData->hasFingerGesture) {
			vtsData->fg2MinTimeout = 0;
		}

		/* while firmeware updating,do not execute resume process */
		if (vtsData->fwUpdatingFlag) {
			VTI("firmware is updating.just set tsState,not resume");
			return 0;
		}

		/* call ic action */
		ret = vivoTsModeChange(TOUCHSCREEN_NORMAL);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsModeChange no define.");
		} else if (ret < 0) {
			VTI("vivoTsIcEnter NORMAL Mode fail.");
			return ret;
		}

		/* usb state write */
		ret = vivoTsSetChargerFlagToChip(vtsData->usbChargerFlag);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetChargerFlagToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetChargerFlagToChip fail.");
		}
		/*while resume,is sure in edgeRestainSwitch on */
		//vtsData->edgeRestainSwitch = 1;
		mutex_lock(&vtsData->edgeMutex);
		ret = vivoTsSetEdgeRestainToChip(vtsData->edgeRestainSwitch);
		mutex_unlock(&vtsData->edgeMutex);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetEdgeRestainToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetEdgeRestainToChip fail.");
		}

		/* glove mode write */
		if (0 == vtsData->usbChargerFlag) {
			ret = vivoTsSetGloveModeToChip(vtsData->gloveModeSwitch);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		} else if (1 == vtsData->usbChargerFlag) {	/* if usb insert, close glove mode */
			ret = vivoTsSetGloveModeToChip(0);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		}
	} else {		/* suspend */
		/* add for S3706,avoid cause proximity switch(while calling) tp no function*/
		/*if (vtsData->icNum == VTS_IC_S3706) {
			if (vtsData->hasLcdShutoff == 0) {
				VTI("lcd state is 1,no suspend");
				return 0;
			}
		}*/

		VTI("suspend begin.");
		if ((vtsData->icNum != VTS_IC_TD4322) && (vtsData->icNum != VTS_IC_TD3320)) {
			/* add for proximity and is_calling */
			if (vtsData->isCallingSave) {
				VTI("is callingSave=1,wait 500ms to change mode");
				msleep(500);
			}
		}

		if (vtsData->hasLcdShutoff == 1) {
			vtsData->needChangeToGestureMode = vtsData->proximityRtState;
		}

		/* check which mode to enter */
		if (vtsData->dClickSwitch == 0 && vtsData->gestureSwitch == 0 &&
			vtsData->swipeSwitch == 0 && vtsData->udfGestureSwitch == 0 && vtsData->exportSwitch == 0) {
			vtsData->needChangeToGestureMode = 0;
		}

		/* add for finger gesture,force to gesture mode while finger gesture on */
		if (vtsData->hasFingerGesture) {
			if (vtsData->exportSwitch & 0x01) {
				vtsData->needChangeToGestureMode = 1;
			}
		}
		
		ret = gestureAndSleepModeExchange(vtsData->needChangeToGestureMode);
		/* notify lcm waitqueue to run */
		if (vtsData->icNum == VTS_IC_TD4322 || vtsData->icNum == VTS_IC_TD3320 || 
			vtsData->icNum == VTS_IC_FT8006  || vtsData->icNum == VTS_IC_NT36772 || 
			vtsData->icNum == VTS_IC_NT36525 || vtsData->icNum == VTS_IC_TD4330 || 
			vtsData->icNum == VTS_IC_FT8719 || vtsData->icNum == VTS_IC_SEC_TDDI ||
			vtsData->icNum == VTS_IC_NT36670 || vtsData->icNum == VTS_IC_TD4330_N) {
			VTI("wake up to run lcm process");
			vtsData->lcmTpSyncFlag = VTS_RESUME_DONE;
			wake_up(&vtsData->lcmTpSyncWaitQueue);
		}
		if (ret < 0) {
			VTI("gestureAndSleepModeExchange fail");
			return ret;
		}
	}

	/* ts state reinit */
	vivoTsReleasePointsAndKeys();
	vtsData->isLargePressMode = 0;
	vtsData->is3FingerMode = 0;

	return 0;
}
static int vivoTsKthreadLcdState(void *vivoTsData, int state)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;

	VTI("lcd state=%d", state);

	if (state == 0 || state == 1) {
		vtsData->hasLcdShutoff = (!state);	/* high layer write lcd's current state. */
		/* here avoid quick light off and quick on,no resume and suspend,cause point not free */
		if (vtsData->hasLcdShutoff) {
			vivoTsReleasePoints();
		}
	} else {
		VTI("lcd state node set invalide parameter:%d", state);
		return -EINVAL;
	}

	return 0;
}
static int vivoTsKthreadProximityState(void *vivoTsData, int proxState)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	int ret = 0;

	/* here add for finger quick touch cause smart wake no function */
	vtsData->proximityRtState = proxState;

	if (atomic_read(&vtsData->tsState) == TOUCHSCREEN_NORMAL) {
		vtsData->needChangeToGestureMode = proxState;
		VTI("not first set in normal mode");
		return 0;
	}

	/* add for firmware updating,while updating,not change proximity state. */
	if (vtsData->fwUpdatingFlag) {
		VTI("firmware is updating.no proximity switch.");
		return 0;
	}

	/* normal change state */
	ret = gestureAndSleepModeExchange(proxState);
	if (ret < 0) {
		VTI("gestureAndSleepModeExchange fail");
		return ret;
	}

	return 0;
}
/* register request functions */
static int vivoTsRegisterKthreadFuns(struct vivo_ts_struct *vtsData)
{
	touchscreen_set_priv_data((void *)vtsData);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, vivoTsKthreadResumeSupend);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_LCD_STATE, vivoTsKthreadLcdState);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_PROX_STATE, vivoTsKthreadProximityState);
/*	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_USB_CHARGE,NULL); */

	return 0;
}
static int vivoTsGetIicSuspendFlag(void)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	if (!vtsData->getI2cBusState) {
		ret = -500;
		return ret;
	}
	return vtsData->getI2cBusState();

}

static void vivoTsIrqErrWorkFunc(struct work_struct *work)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	int count = 0;

	if (vtsData->hasLcdShutoff) {
		while (vivoTsGetIicSuspendFlag() > 0 && count < 80) {
			msleep(5);
			count++;
		}
		if (count == 80) {
			VIVO_TS_LOG_ERR("[SYNA]The i2c bus still suspend after 100 times try\n");
			return;
		}

	}
	vtsData->irqHandler(vtsData->irqNum, vtsData->irqDev);
}

static irqreturn_t vivoTsInterruptFunc(int irq, void *data)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	//int count = 0;
	VTD("enter,lcd state:%d,i2c_bus_state:%d", !(vtsData->hasLcdShutoff), vivoTsGetIicSuspendFlag());

	if ((VTS_PLATFORM_QCOM == vivoTsData->mtkOrQcom) && (vivoTsGetState() == TOUCHSCREEN_GESTURE)) {
		VTI("wake lock enable 2s");
		__pm_wakeup_event(&vivoTsData->wakeLock, 2*HZ);
	}

	/*
	if (vtsData->hasLcdShutoff) {
		while (vivoTsGetIicSuspendFlag() > 0 && count < 20) {
			msleep(5);
			count++;
		}

		if (count == 20) {
			VTE("the i2c bus still suspend after 20 times try");
			queue_work(vtsData->irqErrWorkqueue, &vtsData->irqErrWork);
			return IRQ_HANDLED;
		}

	}
	*/

	vtsData->irqHandler(irq, data);
	return IRQ_HANDLED;
}

int vivoTsInterruptRegister(unsigned int irq, irq_handler_t handler, irq_handler_t thread_fn, unsigned long flags, void *dev)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	int ret = 0;

	VTI("enter");
	if (NULL == thread_fn) {
		VTE("irq handler func is NULL");
		return -EIO;
	}
	vtsData->irqNum = irq;
	vtsData->irqDev = dev;
	vtsData->irqHandler = thread_fn;
	if (vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_RT_THREAD ||
		vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_WORK) {
		VTI("irqBtmType is VTS_IRQ_BTM_TYPE_RT_THREAD or VTS_IRQ_BTM_TYPE_WORK");
		flags &= (~IRQF_ONESHOT);
		ret = request_irq(irq, vivoTsInterruptFunc, flags, "vivo_ts_irq", dev);
	} else if (vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_ONESHOOT_THREAD) {
		VTI("irqBtmType is VTS_IRQ_BTM_TYPE_ONESHOOT_THREAD");
		ret = request_threaded_irq(irq, handler, vivoTsInterruptFunc, flags | IRQF_ONESHOT, "vivo_ts_irq", dev);
	}
	if (ret < 0) {
		VTE("Failed to create irq thread");
		vtsData->irqNum = -1;
		vtsData->irqDev = NULL;
		vtsData->irqHandler = NULL;
		return -EIO;
	}
	VTI("end");
	return 0;
}

void vivoTsInterruptUnregister(void)
{
	struct vivo_ts_struct *vtsData = (struct vivo_ts_struct *)vivoTsData;
	VTI("enter");
	if (!vtsData->irqHandler) {
		VTE("irq has not been register");
		return;
	}
	free_irq(vtsData->irqNum, vtsData->irqDev);
	vtsData->irqHandler = NULL;
	vtsData->irqDev = NULL;
	vtsData->irqNum = -1;
	VTI("end");
}

struct vivo_ts_struct *vivoTsAlloc(void)
{
	VTI("1.kzalloc");
	vivoTsData = (struct vivo_ts_struct *)kzalloc(sizeof(struct vivo_ts_struct), GFP_KERNEL);
	if (!vivoTsData) {
		VTI("fail kmalloc vivoTsData.");
		return NULL;
	}

	return vivoTsData;
}

int vivoTsFree(void)
{
	if (vivoTsData) {
		kfree(vivoTsData);
		vivoTsData = NULL;
	}
	return 0;
}

/* return 0 success, <0:fail */
int vivoTsInit(struct i2c_client *client, void *tmpP, int tmp)
{
	int ret = 0;

	VTI("1.vivoTsIcCtlInit");
	vivoTsIcCtlInit(vivoTsData);

	/* ts info init */

	/* init wait queue */
	if (vivoTsData->icNum == VTS_IC_TD4322 || vivoTsData->icNum == VTS_IC_TD3320 || 
		vivoTsData->icNum == VTS_IC_FT8006 || vivoTsData->icNum == VTS_IC_NT36772 || 
		vivoTsData->icNum == VTS_IC_NT36525 || vivoTsData->icNum == VTS_IC_TD4330 || 
		vivoTsData->icNum == VTS_IC_FT8719 || vivoTsData->icNum == VTS_IC_SEC_TDDI ||
		vivoTsData->icNum == VTS_IC_NT36670 || vivoTsData->icNum == VTS_IC_TD4330_N) {
		init_waitqueue_head(&(vivoTsData->lcmTpSyncWaitQueue));
	}

	/* set default leavel for resume and suspend process */
	vivoTsData->resumeEventBlank = FB_EVENT_BLANK;
	vivoTsData->suspendEventBlank = FB_EARLY_EVENT_BLANK;

	vivoTsData->edgeRestainSwitch = 1;		/* edge restain must be default on */
	vivoTsData->dClickSwitch = 1;	/* dclick switch must default on */
	mutex_init(&(vivoTsData->i2cResetMutex));
	mutex_init(&(vivoTsData->inputReportMutex));
	mutex_init(&(vivoTsData->sensorTestMutex));
	mutex_init(&(vivoTsData->sensorRawDiffGetMutex));
	mutex_init(&(vivoTsData->lcmResumeAndProximityMutex));
	mutex_init(&(vivoTsData->edgeMutex));
	if (VTS_PLATFORM_QCOM == vivoTsData->mtkOrQcom) {
		wakeup_source_init(&vivoTsData->wakeLock, "smartwake");
	}

	vivoTsData->irqErrWorkqueue = create_singlethread_workqueue("vivo_ts_wq");
	if (!vivoTsData->irqErrWorkqueue) {
		VTE("can't create irq err worqueue");
		ret = -ENOMEM;
		goto create_workqueue_failed;
	}
	INIT_WORK(&vivoTsData->irqErrWork, vivoTsIrqErrWorkFunc);
	VTI("3.init chargerWork");
	/* charger work */
	INIT_WORK(&vivoTsData->chargerWork, vivoTsChargerWorkHandler);

	/* register log_switch */
	VTI("4.register_touchscreen_common_interface");
	ret = register_touchscreen_common_interface(&vivoTsCommonData);
	if (ret == -1) {
		VTE("fail to register touchscreen_common_interface.");
		goto register_touchscreen_common_interface_fail;
	}

	/* register log_switch volum key */
	VTI("4.bbk_drivers_log_switch_register_callback");
	ret = bbk_drivers_log_switch_register_callback(&vivoTsLogSwitchHandler);
	if (ret < 0) {
		VTE("fail set log switch bbk  ret=%d", ret);
		goto bbk_drivers_log_switch_register_callback_fail;
	}

	ret = vivoTsCreatSysFsFile(vivoTsData);
	if (ret < 0) {
		VTE("fail vivoTsCreatSysFsFile = %d", ret);
		goto vivoTsCreatSysFsFileFail;
	}

	if (vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		/* Register for TS info */
		ret = ts_info_register(&(vivoTsData->kobjectDebug));
		if (ret < 0) {
			VTE("fail to register ts info = %d", ret);
			goto vivoTsInfoRegisterFail;
		}
	}

	if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
		vivoTsData->virtualKeyProcessor.vivo_key_standard_suspend = NULL;
		vivoTsData->virtualKeyProcessor.vivo_key_standard_resume = NULL;
		vivoTsData->virtualKeyProcessor.button_attr = NULL;
		vivoTsData->virtualKeyProcessor.pdev = &client->dev;
		register_vivo_key(&vivoTsData->virtualKeyProcessor);
	}

	ret = vivoTsInputDeviceInit(client, NULL, -1);
	if (ret < 0) {
		VTE("fail vivoTsInputDeviceInit = %d", ret);
		goto vivoTsInputDeviceInitFail;
	}

	ret = vtsIrqBtmThreadRegister();
	if (ret < 0) {
		VTI("start the vts irq btm rt thread fail");
		goto vivoTsIrqBtmThreadRegisterFail;
	}

	ret = vtsInitProFcFilter();
	if (ret < 0) {
		VTI("proFcFilter init fail");
		goto vtsInitProFcFilterFail;
	}

	/* for finger gesture */
	if (vivoTsData->hasFingerGesture) {
		vivoTsData->lcdFcFilterSwitch = 1;
		vivoTsData->fgProximityRtState = 1;
	}
	ret = vtsInitLcdFcFilter();
	if (ret < 0) {
		VTI("vtsInitLcdFcFilter init fail");
		goto vtsInitLcdFcFilterFail;
	}
	
	return 0;

vtsInitLcdFcFilterFail:
	vtsFreeProFcFilter();
vtsInitProFcFilterFail:
	vtsIrqBtmThreadStop();
vivoTsIrqBtmThreadRegisterFail:
	vivoTsInputDeviceFree();
vivoTsInputDeviceInitFail:
	if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
		unregister_vivo_key(&vivoTsData->virtualKeyProcessor);
	}
	if (vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		ts_info_unregister();
	}
vivoTsInfoRegisterFail:
	vivoTsRemoveSysFsFile(vivoTsData);
vivoTsCreatSysFsFileFail:
	bbk_drivers_log_switch_unregister_callback(vivoTsLogSwitchHandler.name);
bbk_drivers_log_switch_register_callback_fail:
	unregister_touchscreen_common_interface(&vivoTsCommonData);
register_touchscreen_common_interface_fail:
	flush_workqueue(vivoTsData->irqErrWorkqueue);
	destroy_workqueue(vivoTsData->irqErrWorkqueue);
create_workqueue_failed:
	return -EIO;
}

void vivoTsDeInit(void)
{
	vtsFreeLcdFcFilter();
	vtsFreeProFcFilter();
	vtsIrqBtmThreadStop();

	if (VTS_PLATFORM_MTK_OLD == vivoTsData->mtkOrQcom) {
#if defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&vivoTsData->early_suspend)
#endif
	}
	vivoTsInputDeviceFree();
	if (vivoTsData->keyType == VTS_KEY_0D && vivoTsData->isIndependentKeyIc == 0) {
		unregister_vivo_key(&vivoTsData->virtualKeyProcessor);
	}

	if (vivoTsData->hasFingerPrint || vivoTsData->isIndependentKeyIc) {
		ts_info_unregister();
	}
	vivoTsRemoveSysFsFile(vivoTsData);
	bbk_drivers_log_switch_unregister_callback(vivoTsLogSwitchHandler.name);
	unregister_touchscreen_common_interface(&vivoTsCommonData);
	flush_workqueue(vivoTsData->irqErrWorkqueue);
	destroy_workqueue(vivoTsData->irqErrWorkqueue);
	if (VTS_PLATFORM_QCOM == vivoTsData->mtkOrQcom) {
		wakeup_source_trash(&vivoTsData->wakeLock);
	}
}

/*
* after probe complete,we need set charge state,edge restain,glove mode,and other init state.
*
* return value not need fail,because set init state fail,no need return probe -1,this will cause ts no function
*
*/
int vivoTsAfterProbeCompleteCall(struct i2c_client *client, void *tmpP, int tmp)
{
	int ret = 0;

	vivoTsData->isProbeComplete = 1;
/*
	//must,else gesture maybe no function
	//ret = enable_irq_wake(client->irq);
	//if (ret) {
	//	VTI("set_irq_wake failed for irq %d", client->irq);
	//}
*/
	vivoTsRegisterKthreadFuns(vivoTsData);

	ret = vivoTsSetEdgeRestainToChip(vivoTsData->edgeRestainSwitch);
	if (ret) {
		VTI("vivoTsSetEdgeRestainToChip fail.");
	}

	/* if ts module not loade,but vivo_touchscreen_common.c loade and usb is charge while phone boot.
	 * must include this situation.
	 **/
	if (vivo_get_before_ts_load_usb_charger_flag() == 1) {
		vivoTsData->usbChargerFlag = 1;
		/* must clean after phone booted */
		vivo_clear_before_ts_load_usb_charger_flag();
	}
	/* just write in normal mode */
	if (atomic_read(&vivoTsData->tsState) == TOUCHSCREEN_NORMAL) {/*ts work normal,need to write charger */
		if (vivoTsData->usbChargerFlag == 1) {
			/* sent charger cmd */
			ret = vivoTsSetChargerFlagToChip(1);
			/* close glove mode while charger */
		} else if (vivoTsData->usbChargerFlag == 0) {
			/* send no charger cmd */
			ret = vivoTsSetChargerFlagToChip(0);
			/* open glove mode while charger */
		}
		if (ret < 0) {
			VTI("vivoTsSetChargerFlagToChip fail.");
		}
	}

	if (vivoTsData->mtkOrQcom == VTS_PLATFORM_QCOM
		|| vivoTsData->mtkOrQcom == VTS_PLATFORM_MTK_NEW) {
		VTI("register notifier callback for qcom or mtk_new.");
#if defined(CONFIG_FB)
		if (0 == vivoTsData->lcdFcFilterSwitch) {
			if (vivoTsData->callbackType == 0) {
				vivoTsData->fbNotif.notifier_call = vivoTsFbNotifierCallback;
				ret = fb_register_client(&vivoTsData->fbNotif);
			} /*else if (vivoTsData->callbackType == 1) {
				vivoTsData->fbNotif.notifier_call = vivoTsFbNotifierCallback2;
				ret = msm_drm_register_client(&vivoTsData->fbNotif);
			}*/
			if (ret) {
				VTE("Unable to register fb_notifier.");
				ret = -1;
				return ret;
			}
		}
#endif
	} else if (vivoTsData->mtkOrQcom == VTS_PLATFORM_MTK_OLD) {
		VTI("register notifier callback for mtk old.");
#if defined(CONFIG_HAS_EARLYSUSPEND)
		vivoTsData->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + VTS_SUSPEND_LEVEL;
		vivoTsData->early_suspend.suspend = vivoTsEarlySuspend;
		vivoTsData->early_suspend.resume = vivoTsLateResume;
		register_early_suspend(&vivoTsData->early_suspend);
#endif
	}

	return 0;
}

int vivoTsFwAdd(char dataFlag, const unsigned char *data, int dataSize, unsigned char *productName, int modulePinValue, int moduleVendor, int lcmId)
{
	int ret = 0;

	if (vivoTsData->vtsFwManager.curFwNum >= VTS_FW_MAX_FW_NUM) {
		VTI("fw num more than manage array, this firmware not been add");
		ret = -1;
		return ret;
	}

	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].dataFlag = dataFlag;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].data = (unsigned char *)data;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].dataSize = dataSize;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].modulePinValue = modulePinValue;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].moduleVendor = moduleVendor;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].productName = productName;
	vivoTsData->vtsFwManager.vtsFwDesc[vivoTsData->vtsFwManager.curFwNum].lcmId = lcmId;
	vivoTsData->vtsFwManager.curFwNum++;

	VTI("add fw success[curNum:%d]:DF:%d PN:%s MPV:%d MV:%d LCMID:%d",
		vivoTsData->vtsFwManager.curFwNum, dataFlag,
		productName, modulePinValue, moduleVendor, lcmId);

	return 0;
}

unsigned char *vivoTsGetFw(char dataType, int *fwSize)
{
	const char *productName = NULL;
	int modulePinValue = VTS_FW_NOPIN_VALUE;
	int lcmId = VTS_FW_NOLCM_ID;
	int i = 0;

	vivo_touchscreen_get_product_name(&productName);
	if (NULL == productName) {
		VTI("no product name, please check device tree");
		*fwSize = 0;
		return NULL;
	}

	modulePinValue = vivoTsGetTsPinValue();
	if (modulePinValue < 0) {
		if (modulePinValue != -FUN_NOT_REG) {
			VTI("vivoTsGetTsPinValue read error");
			*fwSize = 0;
			return NULL;
		}
	}

	lcmId = vivoTsGetLcmId();
	if (lcmId < 0) {
		if (modulePinValue != -FUN_NOT_REG) {
			VTI("vivoTsGetLcmId read error");
			*fwSize = 0;
			return NULL;
		}
	}

	for (i = 0; i < vivoTsData->vtsFwManager.curFwNum; i++) {
		/* find product */
		if (!strcmp(vivoTsData->vtsFwManager.vtsFwDesc[i].productName, productName)) {
			/* judge module pin value */
			if (vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue == modulePinValue) {
				/* judge lcm id */
				if (vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId == lcmId) {
					/* judge dataType */
					if (vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag == dataType) {
						*fwSize = vivoTsData->vtsFwManager.vtsFwDesc[i].dataSize;
						VTI("find dataType:%d dataSize:%d", dataType, *fwSize);
						return vivoTsData->vtsFwManager.vtsFwDesc[i].data;
					} else {
						VTI("dataType:%d not find", dataType);
					}
				} else {
					VTI("lcmId:%d not find", lcmId);
				}
			} else {
				VTI("modulePinValue:%d not find", modulePinValue);
			}
		} else {
			VTI("productName:%s not find", productName);
		}
	}

	VTI("not find dataType:%d", dataType);
	return NULL;
}

int vivoTsFwManagerShow(char *buf)
{
	const char *productName = NULL;
	int modulePinValue = VTS_FW_NOPIN_VALUE;
	int lcmId = VTS_FW_NOLCM_ID;

	int fwVersion = 0;
	int configVersion = 0;

	int ret = 0;
	int i = 0;

	ret += snprintf(&buf[ret], 255, "fw num:[%d]\n", vivoTsData->vtsFwManager.curFwNum);
	for (i = 0; i < vivoTsData->vtsFwManager.curFwNum; i++) {
		if (vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag == VTS_FW_TYPE_FW) {
			fwVersion = vivoTsGetHeaderFileFWVersionOrConfig(FW_VERSION, vivoTsData->vtsFwManager.vtsFwDesc[i].data);
			configVersion = vivoTsGetHeaderFileFWVersionOrConfig(CONFIG_VERSION, vivoTsData->vtsFwManager.vtsFwDesc[i].data);
			if (fwVersion == -500 || configVersion == -500) {
				ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[noif,noif]\n", i,
					vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
					vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
					vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
					vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
					vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor);
			} else if (configVersion < 0 && fwVersion >= 0) {
				ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[0x%x]\n", i,
					vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
					vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
					vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
					vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
					vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor,
					fwVersion);
			} else if (fwVersion >= 0 && configVersion >= 0) {
				ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[0x%x0x%x]\n", i,
					vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
					vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
					vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
					vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
					vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor,
					fwVersion,
					configVersion);
			}
		} else {
			ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[%d]\n", i,
				vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
				vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
				vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
				vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
				vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor
			);
		}
	}

	ret += snprintf(&buf[ret], 255, "current product file:\n");
	/* show cur product fw file */
	vivo_touchscreen_get_product_name(&productName);
	if (NULL == productName) {
		ret += snprintf(&buf[ret], 255, "get cur product name fail\n");
		VTI("no product name, please check device tree");
		ret = -1;
		return ret;
	}

	modulePinValue = vivoTsGetTsPinValue();
	if (modulePinValue < 0) {
		if (modulePinValue != -FUN_NOT_REG) {
			ret += snprintf(&buf[ret], 255, "get ts pin value fail\n");
			VTI("vivoTsGetTsPinValue read error");
			ret = -1;
			return ret;
		}
	}

	lcmId = vivoTsGetLcmId();
	if (lcmId < 0) {
		if (modulePinValue != -FUN_NOT_REG) {
			ret += snprintf(&buf[ret], 255, "get lcm id fail\n");
			VTI("vivoTsGetLcmId read error");
			ret = -1;
			return ret;
		}
	}

	for (i = 0; i < vivoTsData->vtsFwManager.curFwNum; i++) {
		/*find product*/
		if (!strcmp(vivoTsData->vtsFwManager.vtsFwDesc[i].productName, productName)) {
			/*judge module pin value*/
			if (vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue == modulePinValue) {
				/*judge lcm id*/
				if (vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId == lcmId) {
					if (vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag == VTS_FW_TYPE_FW) {
						fwVersion = vivoTsGetHeaderFileFWVersionOrConfig(FW_VERSION, vivoTsData->vtsFwManager.vtsFwDesc[i].data);
						configVersion = vivoTsGetHeaderFileFWVersionOrConfig(CONFIG_VERSION, vivoTsData->vtsFwManager.vtsFwDesc[i].data);
						if (fwVersion == -500 || configVersion == -500) {
							ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[noif,noif]\n", i,
								vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
								vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
								vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
								vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
								vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor);
						} else if (configVersion < 0 && fwVersion >= 0) {
							ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[0x%x]\n", i,
								vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
								vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
								vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
								vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
								vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor,
								fwVersion);
						} else if (fwVersion >= 0 && configVersion >= 0) {
							ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x] fwVersion[0x%x0x%x]\n", i,
								vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
								vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
								vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
								vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
								vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor,
								fwVersion,
								configVersion);
						}
					} else {
						ret += snprintf(&buf[ret], 255, "    vtsFwDesc[%d] dataType[%d] productName[%s] modulePinValue[%d] lcmId[%d] moduleVendorCode[0x%x]\n", i,
							vivoTsData->vtsFwManager.vtsFwDesc[i].dataFlag,
							vivoTsData->vtsFwManager.vtsFwDesc[i].productName,
							vivoTsData->vtsFwManager.vtsFwDesc[i].modulePinValue,
							vivoTsData->vtsFwManager.vtsFwDesc[i].lcmId,
							vivoTsData->vtsFwManager.vtsFwDesc[i].moduleVendor
						);
					}
				}
			}
		}
	}

	return ret;
}

/* this function add for chip reset exception,after reset exception,need to rewrite our function switch to chip*/
int vivoTsChipStateReset(void)
{
	int ret = 0;
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();
	if (vivoTsGetState() == TOUCHSCREEN_NORMAL) {
		/*free fingers and key*/
		vivoTsReleasePointsAndKeys();

		ret = vivoTsSetChargerFlagToChip(vtsData->usbChargerFlag);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetChargerFlagToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetChargerFlagToChip fail.");
		}
		/* while resume,is sure in edgeRestainSwitch on */
		vtsData->edgeRestainSwitch = 1;
		ret = vivoTsSetEdgeRestainToChip(vtsData->edgeRestainSwitch);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetEdgeRestainToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetEdgeRestainToChip fail.");
		}

		/*glove mode write*/
		if (0 == vtsData->usbChargerFlag) {
			ret = vivoTsSetGloveModeToChip(vtsData->gloveModeSwitch);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		} else if (1 == vtsData->usbChargerFlag) {	/*if usb insert, close glove mode*/
			ret = vivoTsSetGloveModeToChip(0);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		}
	} else if (vivoTsGetState() == TOUCHSCREEN_GESTURE) {
		ret = vivoTsModeChange(TOUCHSCREEN_GESTURE);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsModeChange no define.");
		} else if (ret < 0) {
			VTI("vivoTsIcEnter GESTURE Mode fail.");
			return ret;
		}
		/*set gesture switch*/
		ret = vivoTsSetGestureToChip(allGestureSwitchIntoBitmap());
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetGestureToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetGestureToChip fail.");
		}
		/*glove mode write*/
		if (0 == vtsData->usbChargerFlag) {
			ret = vivoTsSetGloveModeToChip(vtsData->gloveModeSwitch);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		} else if (1 == vtsData->usbChargerFlag) {	/*if usb insert, close glove mode*/
			ret = vivoTsSetGloveModeToChip(0);
			if (-FUN_NOT_REG == ret) {
				VTI("vivoTsSetGloveModeToChip no define.");
			} else if (ret < 0) {
				VTI("vivoTsSetGloveModeToChip fail.");
			}
		}
	} else if (vivoTsGetState() == TOUCHSCREEN_SLEEP) {
		ret = vivoTsModeChange(TOUCHSCREEN_SLEEP);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsModeChange no define.");
		} else if (ret < 0) {
			VTI("vivoTsIcEnter SLEEP Mode fail.");
			return ret;
		}
	}
	return (ret == -FUN_NOT_REG)?0:ret;
}

#ifdef	VIVO_TOUCH_COLLECT_LOG
extern int writeData(char *modelId, char *filename, char *data);
extern int writeDatas(char *modelId, char *filename, char *fmt, ...);
int collectTouchException(int exception_type, int reason_num)
{
	struct vivo_ts_struct *vtsData = vivoTsGetVtsData();
	int module_id;
	int chip_id;
	char data[100];
	int fw_version = 0;
	int config_version = 0;
	int tmp = 0;

	module_id = vivoTsGetModuleId();

	chip_id = vtsData->icNum;

	tmp = vivoTsGetIcFirmwareOrConfigVersion(CONFIG_VERSION);
	if (-FUN_NOT_REG == tmp) {
		VTI("vivoTsGetIcFirmwareOrConfigVersion no defined.\n");
	} else if (tmp < 0) {
		VTI("read config version fail.\n");
	}
	config_version = tmp;

	tmp = vivoTsGetIcFirmwareOrConfigVersion(FW_VERSION);
	if (-FUN_NOT_REG == tmp) {
		VTI("vivoTsGetIcFirmwareOrConfigVersion no defined.\n");
	} else if (tmp < 0) {
		VTI("read firmware version fail.\n");
	}
	fw_version = tmp;
	if (-FUN_NOT_REG == config_version) {
		config_version = fw_version;
		fw_version = 0;
		if (config_version < 0) {
			config_version = 0;
		}
	} else {
		config_version = (config_version < 0) ? 0 : config_version;
		fw_version = (fw_version < 0) ? 0 : fw_version;
	}

	snprintf(data, sizeof(data), "%d,0x%x0x%x,0x%x,%d,%d", exception_type,
				fw_version, config_version, module_id, chip_id, reason_num);
	VTI("%s", data);
	writeDatas("1402", "1402_4", data);
	return 0;
}

#endif

/*
 * interrupt bottom thread
 * to avoid report rate esception
 * add 17.08.23
 * useage
 * 1.set the vivoTsData->irqBtmType = VTS_IRQ_BTM_TYPE_RT_THREAD;
 * 2.change the chip's driver irq function,and add vtsIrqBtmThreadWake();in it
 */
static wait_queue_head_t vtsIrqBtmThreadWait;
static int vtsIrqBtmThreadWaitCondition;
static struct task_struct *vtsIrqBtmKthread;
static int vtsIrqBtmThreadFlag;
extern void ts_thread_call_func(void);
static int vtsIrqBtmThread(void *ignored)
{
	static struct sched_param para = {
		.sched_priority = VTS_IRQ_BTM_THREAD_DEFAULT_PRIORITY,	/*DEFAULT_PRIO,		//Max RT priority*/
	};
	VTI("vts irq bottom thread start,priority is %d", para.sched_priority);

	sched_setscheduler(current, SCHED_FIFO, &para);

	while (1)  {
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(vtsIrqBtmThreadWait, vtsIrqBtmThreadWaitCondition || kthread_should_stop());
		vtsIrqBtmThreadWaitCondition = 0;
		set_current_state(TASK_RUNNING);
		VTD("thread waked up.");
		if (vtsIrqBtmThreadFlag & VTS_IRQ_BTM_KTHREAD_ZOMBIE) {
			goto out;
		}

		if (vivoTsData->irqBtmThreadHandler) {
			vivoTsData->irqBtmThreadHandler(NULL);
		} else {
			VTI("vivoTsData->irqBtmThreadHandler is NULL");
		}
	}

out:
	return 0;
}

int vtsIrqBtmThreadRegister(void)
{
	int rc = 0;

	VTI("start the vts irq btm rt thread");

	if (vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_RT_THREAD) {
		init_waitqueue_head(&vtsIrqBtmThreadWait);
		vtsIrqBtmKthread = kthread_run(&vtsIrqBtmThread, NULL, "vtsIrqKthread");
		if (IS_ERR(vtsIrqBtmKthread)) {
			rc = PTR_ERR(vtsIrqBtmKthread);
			VTE("Failed to create kernel thread; rc = [%d]", rc);
			rc = -1;
			return rc;
		}
	} else {
		VTI("vivoTsData->irqBtmType != VTS_IRQ_BTM_TYPE_RT_THREAD,so, not create thread");
	}

	return 0;
}

void vtsIrqBtmThreadWake(void)
{
	if (vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_RT_THREAD) {
		vtsIrqBtmThreadWaitCondition = 1;
		wake_up_interruptible(&vtsIrqBtmThreadWait);
	} else {
		VTI("vivoTsData->irqBtmType != VTS_IRQ_BTM_TYPE_RT_THREAD, irq bottom rt thread not create,so, not wake");
	}
}

int vtsIrqBtmThreadStop(void)
{
	VTI("stop the vts irq btm rt thread");

	if (vivoTsData->irqBtmType == VTS_IRQ_BTM_TYPE_RT_THREAD) {
		vtsIrqBtmThreadFlag |= VTS_IRQ_BTM_KTHREAD_ZOMBIE;
		kthread_stop(vtsIrqBtmKthread);
		wake_up(&vtsIrqBtmThreadWait);
	} else {
		VTI("vivoTsData->irqBtmType != VTS_IRQ_BTM_TYPE_RT_THREAD,so,not to stop the thread");
	}
	return 0;
}
static u16 vtsGesturePoints[VTS_GESTURE_ARRAY_LEN] = {65535};
int vtsGetGesturePointNum(void)
{
	int num = 0;
	int i = 0;

	for (i = VTS_GESTURE_ARRAY_LEN, num = VTS_GESTURE_ARRAY_LEN; i > 0; i--, num--) {
		if (vtsGesturePoints[i - 1] != 65535) {
			break;
		}
	}

	/* show all points */
	for (i = 0; i < VTS_GESTURE_ARRAY_LEN; i++) {
		VTI("gesturePoint %d", vtsGesturePoints[i]);
	}

	return num / 2;
}
u16 *vtsGesturePointsGet(void)
{
	return vtsGesturePoints;
}
void vtsGesturePointsClean(void)
{
	memset(vtsGesturePoints, 0xff, sizeof(vtsGesturePoints));
}
/*
 * event
 * 0,gesture point
 * 1,set "o" gesture dirction
 * 		index:1(clock dir), 0(negative clock dir)
 */
int vtsGesturePointsReport(int event, int index, u16 x, u16 y)
{
	int ret = 0;
	if (event == VTS_GESTURE_POINT) {
		if (index >= VTS_GESTURE_ARRAY_LEN) {
			VTI("out of VTS_GESTURE_ARRAY_LEN:%d", VTS_GESTURE_ARRAY_LEN);
			ret = -1;
			return ret;
		}
		vtsGesturePoints[index * 2] = x;
		vtsGesturePoints[index * 2 + 1] = y;
	}

	if (event == VTS_GESTURE_O_DIR) {
		if (index == 1) {	/*sun shizhen*/
			vtsGesturePoints[19] = 16;	/*19 index is o gesture dir setting*/
		} else if (index == 0) {
			vtsGesturePoints[19] = 32;
		}
	}
	return 0;
}

/* fast change filter */
static void lcdFcfWorkHandler(struct work_struct *work)
{
	VTI("enter");
	msleep(vivoTsData->lcdFcFilter->delayTime);
	mutex_lock(&(vivoTsData->lcdFcFilter->fcfMutex));
	if (vivoTsData->lcdFcFilter->realTimeState == 1) {
		VTI("go resume");
		touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 1);
		touchscreen_request_send(TOUCHSCREEN_REQ_ID_LCD_STATE, 1);
	} else if (vivoTsData->lcdFcFilter->realTimeState == 0) {
		VTI("go suspend");
		touchscreen_request_send(TOUCHSCREEN_REQ_ID_LCD_STATE, 0);
		touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, 0);
	}
	vivoTsData->lcdFcFilter->isWorkRun = 0;
	mutex_unlock(&(vivoTsData->lcdFcFilter->fcfMutex));
	VTI("end");
}
static void proFcfWorkHandler(struct work_struct *work)
{
	VTI("enter");
	msleep(vivoTsData->proFcFilter->delayTime);
	mutex_lock(&(vivoTsData->proFcFilter->fcfMutex));
	touchscreen_request_send(TOUCHSCREEN_REQ_ID_PROX_STATE, vivoTsData->proFcFilter->realTimeState);
	vivoTsData->proFcFilter->isWorkRun = 0;
	mutex_unlock(&(vivoTsData->proFcFilter->fcfMutex));
	VTI("end");
}

/* alloc a filter */
int vtsInitLcdFcFilter(void)
{
	int ret = 0;
	VTI("enter");
	if (vivoTsData->lcdFcFilterSwitch == 0) {
		VTI("proFcFilter is not on,not do init");
		return 0;
	}
	vivoTsData->lcdFcFilter = (struct VtsFcFilter *)kzalloc(sizeof(struct VtsFcFilter), GFP_KERNEL);
	if (vivoTsData->lcdFcFilter == NULL) {
		VTI("alloc lcdFcFilter fail");
		ret = -1;
		return ret;
	}
	mutex_init(&(vivoTsData->lcdFcFilter->fcfMutex));
	INIT_WORK(&(vivoTsData->lcdFcFilter->fcfWork), lcdFcfWorkHandler);
	vivoTsData->lcdFcFilter->delayTime = 50;
	return 0;
}
int vtsFreeLcdFcFilter(void)
{
	if (vivoTsData->lcdFcFilterSwitch == 0) {
		VTI("lcdFcFilter is not on,not do free");
		return 0;
	}
	kfree(vivoTsData->lcdFcFilter);
	return 0;
}
int vtsInitProFcFilter(void)
{
	int ret = 0;
	VTI("enter");
	if (vivoTsData->proFcFilterSwitch == 0) {
		VTI("proFcFilter is not on,not do init");
		return 0;
	}
	vivoTsData->proFcFilter = (struct VtsFcFilter *)kzalloc(sizeof(struct VtsFcFilter), GFP_KERNEL);
	if (vivoTsData->proFcFilter == NULL) {
		VTI("alloc proFcFilter fail");
		ret = -1;
		return ret;
	}
	mutex_init(&(vivoTsData->proFcFilter->fcfMutex));
	INIT_WORK(&(vivoTsData->proFcFilter->fcfWork), proFcfWorkHandler);
	vivoTsData->proFcFilter->delayTime = 50;
	return 0;
}
int vtsFreeProFcFilter(void)
{
	if (vivoTsData->proFcFilterSwitch == 0) {
		VTI("proFcFilter is not on,not do free");
		return 0;
	}
	kfree(vivoTsData->proFcFilter);
	return 0;
}
void vtsLcdFcFilter(int state)
{
	VTI("enter");
	mutex_lock(&(vivoTsData->lcdFcFilter->fcfMutex));
	vivoTsData->lcdFcFilter->realTimeState = state;
	mutex_unlock(&(vivoTsData->lcdFcFilter->fcfMutex));
	if (vivoTsData->lcdFcFilter->isWorkRun == 0) {
		VTI("schedule lcd fcfilter work");
		vivoTsData->lcdFcFilter->isWorkRun = 1;
		schedule_work(&(vivoTsData->lcdFcFilter->fcfWork));
	}
	VTI("end");
}
void vtsProFcFilter(int state)
{
	VTI("enter");
	mutex_lock(&(vivoTsData->proFcFilter->fcfMutex));
	vivoTsData->proFcFilter->realTimeState = state;
	mutex_unlock(&(vivoTsData->proFcFilter->fcfMutex));
	if (vivoTsData->proFcFilter->isWorkRun == 0) {
		VTI("schedule pro fcfilter work");
		vivoTsData->proFcFilter->isWorkRun = 1;
		schedule_work(&(vivoTsData->proFcFilter->fcfWork));
	}
	VTI("end");
}

/***************************************
*
* caculate the nub of bits with value 1
*
*****************************************/
int count_one_bits(unsigned int value)
{
	int count = 0;
	while (value) { 
		value = value & (value - 1);
		count++;
	}
	return count;
}

/***************************************
*
* INDEX:0 water mode; 1 doze mode; 2 charge mode; 3 exception handler mode
*
*****************************************/

void vtsSetTouchData(int index, int state)
{
	unsigned int count = 0;
	
	if (vivoTsData->statisticsCount[index] < 0x20) {
		vivoTsData->statisticsCount[index] += 1;
	}

	vivoTsData->statusBit[index] <<= 1;
	vivoTsData->statusBit[index] &= 0xffffffff;
	vivoTsData->statusBit[index] |= (state ? 1 : 0);
	count = count_one_bits(vivoTsData->statusBit[index]);
	vivoTsData->stateCount[index] = (vivoTsData->statisticsCount[index] << 8) | count;
	return;
}

