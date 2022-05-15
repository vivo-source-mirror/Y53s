#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>

#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_ts_function.h>
#include <linux/vivo_ts_ic_ctl.h>
#include <linux/vivo_touchscreen_virtual_key.h>

#define TEST_NODE_CMD_MAX 10000
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
#define VTS_TOUCH_INFO_COLLECT 116
#define VTS_TOUCH_INFO_COLLECT_DUMP 117

static struct vivo_ts_struct *vivo_ts_data;

#ifndef	_VIVO_KEY_H_
struct touchscreen_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};
#endif

static ssize_t touchscreen_null_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}
static ssize_t touchscreen_null_store(struct kobject *kobj,
							struct kobj_attribute *attr,  const char *buf, size_t count)
{
		return -EINVAL;
}

static ssize_t touchscreen_log_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{

	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("[%s]invalide number of parameters passed\n", __func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("FTS parameter is %d\n", val);
	if (val == 0) {
		vivo_ts_data->logSwitch = 0;
	} else if (val == 1) {
		vivo_ts_data->logSwitch = 1;
	} else{
		VIVO_TS_LOG_INF("[%s]invalide parameter passed:%d\n", __func__, val);
		return -EINVAL;
	}

	return count;
}
static ssize_t touchscreen_log_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "log_switch = %d\n", vivo_ts_data->logSwitch);
}

static ssize_t touchscreen_ic_mode_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	if (atomic_read(&vivo_ts_data->tsState) == TOUCHSCREEN_SLEEP) {
		return snprintf(buf, 255, "sleep mode, could not read.\n");
	}
	return snprintf(buf, 255, "%d\n", vivoTsReadIcMode());
}

/*
 * #define RAW_DATA 0
 * #define DIFF_DATA 1
 *
 *
 *
 *
 */
static int getDiffOrRawData(int which, char *buf)
{
	int *rawOrDiffData = NULL;
	int i = 0;
	int j = 0;
	int ret = 0;
	unsigned int txNum = 0;
	unsigned int rxNum = 0;
	unsigned char log_buf[1023];
	int offset = 0;

	vivoTsReleasePointsAndKeys();

	ret = vivoTsGetTxRxNum();	/*there reget data, because sometime, we get tx rx from ic*/
	txNum = ret & 0xff;
	rxNum = (ret>>8) & 0xff;

	rawOrDiffData = (int *)kzalloc(rxNum*txNum*sizeof(int), GFP_KERNEL);
	if (rawOrDiffData == NULL) {
		VTI("kzalloc mem fail.");
		ret = -1;
		return ret;
	}

	ret = vivoTsGetRawOrDiffDataFromChip(which, rawOrDiffData);
	if (-FUN_NOT_REG == ret) {
		VTI("vivoTsGetRawOrDiffDataFromChip no define.");
	}
	if (ret < 0) {
		VTI("get raw or diff data from chip fail.");
		kfree(rawOrDiffData);
		ret = -1;
		return ret;
	}

	ret = snprintf(buf, 255, "tp channel: %u * %u\n", txNum, rxNum);
	for (i = 0; i < txNum; i++) {
		for (j = 0; j < rxNum; j++) {
			ret += snprintf(&buf[ret], 255, "%5d ", (signed short)rawOrDiffData[i*rxNum + j]);
			if (ret > (4095 - 7)) {
				goto exit;
			}
		}
		ret += snprintf(&buf[ret], 255, "\n");
	}
exit:
	if (ret > (4095 - 7)) {
		for (i = 0; i < txNum; i++) {
			memset(log_buf, 0, sizeof(log_buf));
			offset = 0;
			for (j = 0; j < rxNum; j++) {
				offset += snprintf(&log_buf[offset], 255, "%6d ", (signed short)rawOrDiffData[i*rxNum + j]);
			}
			VIVO_TS_LOG_INF("%s\n", log_buf);
		}
		ret = -EINVAL;
		VIVO_TS_LOG_INF("Unexpect data\n");
	}
	kfree(rawOrDiffData);
	vivoTsReleasePointsAndKeys();

	return ret;
}

static ssize_t touchscreen_rawdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VTI("get sensor rawdata.");

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
	}

	if (!vivo_ts_data->getRawOrDiffData) {
		return snprintf(buf, 255, "getRawOrDiffData not define in ic driver.\n");
	}

	mutex_lock(&(vivo_ts_data->sensorTestMutex));
	ret = getDiffOrRawData(RAW_DATA, buf);
	mutex_unlock(&(vivo_ts_data->sensorTestMutex));
	if (ret < 0) {
		VTI("get rawdata fail.");
		return ret;
	}

	return ret;
}

static ssize_t touchscreen_delta_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	VTI("get sensor diffdata.");

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
	}

	if (!vivo_ts_data->getRawOrDiffData) {
		return snprintf(buf, 255, "getRawOrDiffData not define in ic driver.\n");
	}

	mutex_lock(&(vivo_ts_data->sensorTestMutex));
	ret = getDiffOrRawData(DIFF_DATA, buf);
	mutex_unlock(&(vivo_ts_data->sensorTestMutex));
	if (ret < 0) {
		VTI("get delta fail.");
		return ret;
	}

	return ret;
}

static ssize_t touchscreen_sensor_test_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not test.\n");
	}

	if (!vivo_ts_data->sensorTest) {
		VTI("sensorTest not define in ic driver.");
		return snprintf(buf, 255, "sensorTest not define in ic driver.\n");
	}

	msleep(1000);
	mutex_lock(&(vivo_ts_data->sensorTestMutex));
	vivoTsReleasePointsAndKeys();

	VTI("sensor test begin");
	ret = vivoSensorTest(buf, NULL, -1);
	if (ret < 0) {
		ret = snprintf(buf, 255, "fail\n");
		VTI("sensor test function exception");
	} else {
		VTI("sensor test function no exception");
	}

	vivoTsReleasePointsAndKeys();
	mutex_unlock(&(vivo_ts_data->sensorTestMutex));

	vivo_ts_data->isLargePressMode = 0;
	vivo_ts_data->is3FingerMode = 0;

	return ret;
}

static ssize_t touchscreen_touch_ic_name_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d", vivo_ts_data->icNum);
}

static ssize_t touchscreen_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
	}

	ret = getFirmwareaVersion(buf);
	ret += snprintf(&buf[ret], 255, " VTSCCV:0x%x\n", VIVO_TS_FUNCTION_VERSION);

	/*release key and point, avoid cause up event lost, such as st's ftm3*/
	vivoTsReleasePointsAndKeys();

	return ret;
}

static ssize_t touchscreen_firmware_module_id_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int moduleId = 0;

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not read.\n");
	}

	moduleId = vivoTsGetModuleId();
	if (-FUN_NOT_REG == moduleId) {	/*no func to read module id*/
		return snprintf(buf, 255, "0x%x\n", vivo_ts_data->moduleId);
	}

	if (moduleId < 0) {
		return snprintf(buf, 255, "read module id from ic fail.\n");
	}

	return snprintf(buf, 255, "0x%x\n", moduleId);
}

static ssize_t touchscreen_sensor_rx_tx_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int temp = 0;
	int ret = 0;
	int tmp_tx = 0, tmp_rx = 0;

	ret = vivoTsGetRxTxNumFromIc();
	if (-FUN_NOT_REG == ret) {
		if (vivo_ts_data->rxNum >= vivo_ts_data->txNum) {
			temp = (vivo_ts_data->rxNum<<8) | vivo_ts_data->txNum;
		} else {
			temp = (vivo_ts_data->txNum<<8) | vivo_ts_data->rxNum;
		}
	} else if (-1 == ret) {
		VTI("read rx/tx num from ic fail.");
		if (vivo_ts_data->rxNum >= vivo_ts_data->txNum) {
			temp = (vivo_ts_data->rxNum<<8) | vivo_ts_data->txNum;
		} else {
			temp = (vivo_ts_data->txNum<<8) | vivo_ts_data->rxNum;
		}
	} else if (ret > 0) {
		tmp_tx = temp & 0xff;
		tmp_rx = (temp>>8) & 0xff;
		if (tmp_tx >= tmp_rx) {
			temp = (tmp_tx<<8) | tmp_rx;
		} else {
			temp = (tmp_rx<<8) | tmp_tx;
		}
	}

	return snprintf(buf, 255, "%d\n", temp);
}

static ssize_t fts_is_calling_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d\n", vivo_ts_data->isCalling);
}
static ssize_t fts_is_calling_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	unsigned int val;

	if (sscanf(buf, "%d", &val) != 1) {
		return -EINVAL;
	}

	VTI("val is %d", val);
	if (val != 0 && val != 1) {
		return -EINVAL;
	}

	VTI("is_large_press_mode=%d is_3_finger_touch=%d", vivo_ts_data->isLargePressMode, vivo_ts_data->is3FingerMode);

	if (val == 0) {
		vivo_ts_data->isCalling = 0;
		vivo_ts_data->isCallingSave = 0;
		/* if large press, phone call should ring */
	}

#if 0
	if (val == 1 && ((vivo_ts_data->isLargePressMode == 0) && (vivo_ts_data->is3FingerMode == 0))) {
		vivo_ts_data->isCalling = 1;
		vivo_ts_data->isCallingSave = 1;
	} else {
		if (val == 1) {	/*must, else while val==0, it will set is_calling_save=1;*/
			vivo_ts_data->isCallingSave = 1;
		}
	}
#endif
	/* upper code equal to :*/
	if (1 == val) {
		vivo_ts_data->isCallingSave = 1;
		if ((vivo_ts_data->isLargePressMode == 0) && (vivo_ts_data->is3FingerMode == 0)) {
			vivo_ts_data->isCalling = 1;
		}
	}

	return count;
}

static ssize_t touchscreen_dclick_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("parameter is %d", val);
	if (val == 0) {
		vivo_ts_data->dClickSwitch = 0;
	} else if (val == 1) {
		vivo_ts_data->dClickSwitch = 1;
	} else {
		VTI("invalide parameter passed:%d", val);
		return -EINVAL;
	}

	return count;
}
static ssize_t touchscreen_dclick_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "dclick_switch = %d, 0--off, 1---on\n", vivo_ts_data->dClickSwitch);
}

static ssize_t touchscreen_swipe_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("parameter is %d", val);
	if (val == 0) {
		vivo_ts_data->swipeSwitch = 0;
	} else if (val == 1) {
		vivo_ts_data->swipeSwitch = 1;
	} else {
		VTI("invalide parameter passed:%d", val);
		return -EINVAL;
	}

	return count;
}
static ssize_t touchscreen_swipe_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "swipe_switch = %d, 0--off, 1---on\n", vivo_ts_data->swipeSwitch);
}

/* if return 0,no filter
 * if return 1,filter
 * */
int fingerGestureTimeoutFilter(void)
{
	if (vivo_ts_data->fg2MinTimeout == 0) {	/* in 2 minites */
		VTI("fg2MinTimeout = 0");
		if ((vivo_ts_data->exportSwitch & 0x01) == 0) {	/* finger gesture off */
			VTI("finger gesture off, do nothing");
			return 0;
		} else if ((vivo_ts_data->exportSwitch & 0x01) == 0x01) {	/* finger gesture on */
			VTI("finger gesture on,do filter judge");
			return 1;
		}
	} else if (vivo_ts_data->fg2MinTimeout == 1) {
		VTI("fg2MinTimeout = 1");
		if (vivo_ts_data->fgProximityRtState == 1) {
			vivo_ts_data->fg2MinTimeout = 0;
		} else {
			VTI("fptimeout=1 is early than prox=0 not change fg2MinTimeout");
		}
		return 0;
	}

	return 0;
}
static ssize_t touchscreen_dclick_proximity_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("dclick_proxi_switch val is %d", val);
	// by tp team 
	/*
	if (allGestureSwitchIntoBitmap() == 0 && (vivo_ts_data->exportSwitch & 0x01) == 0 ) {
		VTI("all gesture is off, no need proximity state");
		return -EINVAL;
	}
	*/
	/* just fingure gesture product run */
	if (vivo_ts_data->hasFingerGesture) {
		/* add proximity real time state just for finger gesture */
		vivo_ts_data->fgProximityRtState = val;
		if (fingerGestureTimeoutFilter() == 1) {
			vivo_ts_data->proximityRtState = 1;	/* while filter,gesture mode must be on */
			if (val == 0) {
				vivo_ts_data->commonGestureOff = 1;	
			} else {
				vivo_ts_data->commonGestureOff = 0;	
			}
			return count;
		}
	}

	vivo_ts_data->commonGestureOff = 0;
	if (val == 0 || val == 1) {
		if (vivo_ts_data->proFcFilterSwitch == 0 || vivo_ts_data->suspendProximityChangeTimes < 2) {
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_PROX_STATE, val);
		} else if (vivo_ts_data->proFcFilterSwitch == 1) {
			vtsProFcFilter(val);
		}
		vivo_ts_data->suspendProximityChangeTimes++;
		VTI("suspendProximityChangeTimes = %d", vivo_ts_data->suspendProximityChangeTimes);
	}
	return count;
}

static ssize_t touchscreen_gesture_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("parameter is 0x%x", val);
	if (val < 0x100) {
		vivo_ts_data->gestureSwitch = val;
	} else {
		VTI("invalide parameter passed:%d", val);
		return -EINVAL;
	}

	return count;
}
static ssize_t touchscreen_gesture_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "0x01:LR 0x02:up 0x04:O 0x08:W 0x10:M 0x20:e 0x40:C 0x80:shortDown gesture_switch = 0x%x\n", vivo_ts_data->gestureSwitch);
}

static ssize_t touchscreen_dclick_lcd_state_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("parameter is %d", val);
	if (val == 0 || val == 1) {
		vivo_ts_data->hasLcdShutoff = !(val);
		vivo_ts_data->lcdRtState = val;
		if (vivo_ts_data->lcdFcFilterSwitch) {
			vtsLcdFcFilter(val);
		} else if (vivo_ts_data->lcdFcFilterSwitch == 0) {
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_LCD_STATE, val);
		}
	}
	
	return count;
}
static ssize_t touchscreen_dclick_lcd_state_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "has_lcd_shutoff = %d, 0--lcd on, 1---lcd off\n", vivo_ts_data->hasLcdShutoff);
}

static ssize_t touchscreen_gesture_point_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	u16 gesturePoint[130];	/*gesture coordinate */
	int gesturePointNum = 0;
	int i = 0;
	int len = 0;

	memset(gesturePoint, 0xff, sizeof(gesturePoint));

	/*get gesture data*/
	gesturePointNum = vivoTsGetGesturePointDataFromChip(gesturePoint);
	if (gesturePointNum < 0) {
		/*construct an error line to show that read data error*/
		gesturePoint[0] = 10;
		gesturePoint[1] = 10;
		gesturePoint[2] = 200;
		gesturePoint[3] = 200;
		if (gesturePointNum != -FUN_NOT_REG) {
			VTI("fail to read gesture point data.");
			gesturePointNum = 2;
		}
	}

	if (gesturePointNum == -FUN_NOT_REG) {
		memset(gesturePoint, 0xff, sizeof(gesturePoint));
		memcpy(gesturePoint, vtsGesturePointsGet(), VTS_GESTURE_ARRAY_LEN * 2);
		gesturePointNum = vtsGetGesturePointNum();
	}
	VTI("gesturePointNum %d", gesturePointNum);
	for (i = 0; i < gesturePointNum * 2; i = i + 2) {
		VTI("x:%d y:%d", gesturePoint[i], gesturePoint[i+1]);
		len += snprintf(&buf[len], 255, "%d ", gesturePoint[i]);
		len += snprintf(&buf[len], 255, "%d ", gesturePoint[i+1]);
	}
	len += snprintf(&buf[len], 255, "%d ", 65535);	/*here must not have '\n', or gesture will not draw normally, app layer BUG*/

	return len;
}

static int cmd_code;
static ssize_t touchscreen_ts_super_node_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	int tmp = 0;
	char tmp_buf[100];
	memset(tmp_buf, 0, sizeof(tmp_buf));

	switch (cmd_code) {
	case USER_DEFINE_GESTURE_INFO:
		ret = snprintf(buf, 255, "cmd:%d get.\n", USER_DEFINE_GESTURE_INFO);
		break;
	case VTS_VERSION_INFO:
		ret = snprintf(buf, 255, "vivo ts function version:%x.%x.%x\n", (unsigned char)(VIVO_TS_FUNCTION_VERSION>>16),
			(unsigned char)(VIVO_TS_FUNCTION_VERSION>>8), (unsigned char)VIVO_TS_FUNCTION_VERSION);
		break;
	case VTS_ALL_STATE:
		ret += snprintf(buf, 255, "All ts state:\n");
		ret += snprintf(buf+ret, 255, "	txNum=%d\n", vivo_ts_data->txNum);
		ret += snprintf(buf+ret, 255, "	rxNum=%d\n", vivo_ts_data->rxNum);
		ret += snprintf(buf+ret, 255, "	icNum=%d\n", vivo_ts_data->icNum);
		ret += snprintf(buf+ret, 255, "	moduleId=%d\n", vivo_ts_data->moduleId);
		ret += snprintf(buf+ret, 255, "	tsState=%d\n", atomic_read(&vivo_ts_data->tsState));
		ret += snprintf(buf+ret, 255, "	hasLcdShutoff=%d\n", vivo_ts_data->hasLcdShutoff);

		ret += snprintf(buf+ret, 255, "	logSwitch=%d\n", vivo_ts_data->logSwitch);
		ret += snprintf(buf+ret, 255, "	isProbeComplete=%d\n", vivo_ts_data->isProbeComplete);
		ret += snprintf(buf+ret, 255, "	usbChargerFlag=%d\n", vivo_ts_data->usbChargerFlag);
		ret += snprintf(buf+ret, 255, "	fwUpdatingFlag=%d\n", vivo_ts_data->fwUpdatingFlag);
		ret += snprintf(buf+ret, 255, "	isCalling=%d\n", vivo_ts_data->isCalling);
		ret += snprintf(buf+ret, 255, "	isCallingSave=%d\n", vivo_ts_data->isCallingSave);
		ret += snprintf(buf+ret, 255, "	isLargePressMode=%d\n", vivo_ts_data->isLargePressMode);
		ret += snprintf(buf+ret, 255, "	is3FingerMode=%d\n", vivo_ts_data->is3FingerMode);
		ret += snprintf(buf+ret, 255, "	dClickSwitch=%d\n", vivo_ts_data->dClickSwitch);
		ret += snprintf(buf+ret, 255, "	swipeSwitch=%d\n", vivo_ts_data->swipeSwitch);
		ret += snprintf(buf+ret, 255, "	gestureSwitch=%d\n", vivo_ts_data->gestureSwitch);
		ret += snprintf(buf+ret, 255, "	udfGestureSwitch=%d\n", vivo_ts_data->udfGestureSwitch);
		ret += snprintf(buf+ret, 255, "	exportSwitch=%d\n", vivo_ts_data->exportSwitch);
		ret += snprintf(buf+ret, 255, "	edgeRestainSwitch=%d\n", vivo_ts_data->edgeRestainSwitch);
		ret += snprintf(buf+ret, 255, "	gloveModeSwitch=%d\n", vivo_ts_data->gloveModeSwitch);
		ret += snprintf(buf+ret, 255, "	isIndependentKeyIc=%d\n", vivo_ts_data->isIndependentKeyIc);
		ret += snprintf(buf+ret, 255, "	hasFingerPrint=%d\n", vivo_ts_data->hasFingerPrint);
		ret += snprintf(buf+ret, 255, "	keyType=%d\n", vivo_ts_data->keyType);
		ret += snprintf(buf+ret, 255, "	mtkOrQcom=%d\n", vivo_ts_data->mtkOrQcom);
		ret += snprintf(buf+ret, 255, "	fingerOn2d=%d\n", (int)get_finger_on_2d());
		ret += snprintf(buf+ret, 255, "	hasHomeKey=%d\n", vivo_ts_data->hasHomeKey);
		ret += snprintf(buf+ret, 255, "	hasMenuKey=%d\n", vivo_ts_data->hasMenuKey);
		ret += snprintf(buf+ret, 255, "	hasBackKey=%d\n", vivo_ts_data->hasBackKey);
		break;
	case VTS_DTS_INFO:
		ret += snprintf(buf, 255, "Ts dts info:\n");
		ret += snprintf(buf+ret, 255, "	tsDimensionX=%d\n", vivo_ts_data->tsDimensionX);
		ret += snprintf(buf+ret, 255, "	tsDimensionY=%d\n", vivo_ts_data->tsDimensionY);
		ret += snprintf(buf+ret, 255, "	lcdDimensionX=%d\n", vivo_ts_data->lcdDimensionX);
		ret += snprintf(buf+ret, 255, "	lcdDimensionY=%d\n", vivo_ts_data->lcdDimensionY);
		break;
	case VTS_IC_FUNCTION_STATE:
		ret += snprintf(buf, 255, "Ts ic function state:\n");
		ret += snprintf(buf+ret, 255, "	getRawOrDiffData=%d\n", vivo_ts_data->getRawOrDiffData?1:0);
		ret += snprintf(buf+ret, 255, "	getGesturePointData=%d\n", vivo_ts_data->getGesturePointData?1:0);
		ret += snprintf(buf+ret, 255, "	getIcFirmwareOrConfigVersion=%d\n", vivo_ts_data->getIcFirmwareOrConfigVersion?1:0);
		ret += snprintf(buf+ret, 255, "	getModuleId=%d\n", vivo_ts_data->getModuleId?1:0);
		ret += snprintf(buf+ret, 255, "	setEdgeRestainSwitch=%d\n", vivo_ts_data->setEdgeRestainSwitch?1:0);
		ret += snprintf(buf+ret, 255, "	setGloveModeSwitch=%d\n", vivo_ts_data->setGloveModeSwitch?1:0);
		ret += snprintf(buf+ret, 255, "	setGestureSwitch=%d\n", vivo_ts_data->setGestureSwitch?1:0);
		ret += snprintf(buf+ret, 255, "	setChargerFlagSwitch=%d\n", vivo_ts_data->setChargerFlagSwitch?1:0);
		ret += snprintf(buf+ret, 255, "	getRxTxNumFromIc=%d\n", vivo_ts_data->getRxTxNumFromIc?1:0);
		ret += snprintf(buf+ret, 255, "	getTsPinValue=%d\n", vivo_ts_data->getTsPinValue?1:0);
		ret += snprintf(buf+ret, 255, "	setIcPower=%d\n", vivo_ts_data->setIcPower?1:0);
		ret += snprintf(buf+ret, 255, "	hardwareReset=%d\n", vivo_ts_data->hardwareReset?1:0);
		ret += snprintf(buf+ret, 255, "	softwareReset=%d\n", vivo_ts_data->softwareReset?1:0);
		ret += snprintf(buf+ret, 255, "	icModeChange=%d\n", vivo_ts_data->icModeChange?1:0);
		ret += snprintf(buf+ret, 255, "	updateFirmware=%d\n", vivo_ts_data->updateFirmware?1:0);
		ret += snprintf(buf+ret, 255, "	readRegister=%d\n", vivo_ts_data->readRegister?1:0);
		ret += snprintf(buf+ret, 255, "	writeRegister=%d\n", vivo_ts_data->writeRegister?1:0);
		ret += snprintf(buf+ret, 255, "	sensorTest=%d\n", vivo_ts_data->sensorTest?1:0);
		ret += snprintf(buf+ret, 255, "	atSensorTest=%d\n", vivo_ts_data->atSensorTest?1:0);
		break;
	case VTS_KTHREAD_FUNC_STATE:
		ret += snprintf(buf, 255, "Ts kthread function state:\n");
		ret += snprintf(buf+ret, 255, "	todo...\n");
		break;
	case VTS_TS_PIN_VALUE:
		tmp = vivoTsGetTsPinValue();
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "no ts pin to get.\n");
		} else {
			ret = snprintf(buf, 255, "0x%x\n", tmp);
		}
		break;
	case VTS_CHARGER_TEST:
		ret = snprintf(buf, 255, "0x%x\n", vivo_ts_data->usbChargerTestFlag);
		vivo_ts_data->usbChargerTestFlag = 0;
		break;
	case VTS_TS_FW_MANAGER_SHOW:
		ret = vivoTsFwManagerShow(buf);
		break;

	case VTS_SET_IDLE_ENABLE:
		ret = vivoTsIdleEnableOrDisable(1);
		if (ret == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "fail, idle ctl function not define\n");
			VTI("fail, idle ctl function not define");
		} else if (ret < 0) {
			ret = snprintf(buf, 255, "fail, idle ctl error\n");
			VTI("fail, idle ctl error");
		} else {
			ret = snprintf(buf, 255, "succ\n");
			VTI("succ");
		}
		break;
	case VTS_SET_IDLE_DISABLE:
		ret = vivoTsIdleEnableOrDisable(0);
		break;
	case VTS_GET_LARGER_PRESS_COUNT:	/*get large press count*/
		tmp = vivoTsGetOrSet(VTS_GET_LARGER_PRESS_COUNT);
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "vivoTsGetOrSet not define\n");
		} else {
			ret = snprintf(buf, 255, "%d", tmp < 0 ? 0 : tmp);
		}
		break;
	case VTS_CLEAR_LARGER_PRESS_COUNT:	/*clear large press count*/
		tmp = vivoTsGetOrSet(VTS_CLEAR_LARGER_PRESS_COUNT);
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "vivoTsGetOrSet not define\n");
		} else {
			ret = snprintf(buf, 255, "%d", tmp < 0 ? 0 : tmp);
		}
		break;
	case VTS_GET_I2C_ERR_COUNT:	/*clear large press count*/
		tmp = vivoTsGetOrSet(VTS_GET_I2C_ERR_COUNT);
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "vivoTsGetOrSet not define\n");
		} else {
			ret = snprintf(buf, 255, "%d", tmp < 0 ? 0 : tmp);
		}
		break;
	case VTS_CLEAR_I2C_ERR_COUNT:	/*clear large press count*/
		tmp = vivoTsGetOrSet(VTS_CLEAR_I2C_ERR_COUNT);
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "vivoTsGetOrSet not define\n");
		} else {
			ret = snprintf(buf, 255, "%d", tmp < 0 ? 0 : tmp);
		}
		break;
	case VTS_OTHER_INFO:
		tmp = vivoTsOtherInfo(buf);
		if (tmp == -FUN_NOT_REG) {
			ret = snprintf(buf, 255, "vivoTsOtherInfo not define\n");
		} else {
			ret = tmp;
		}
		break;
	case VTS_TEST_COLLECT_BUG:
#ifdef	VIVO_TOUCH_COLLECT_LOG
		collectTouchException(1, 999);
		ret = snprintf(buf, 255, "Exception report success\n");
#else
		ret = snprintf(buf, 255, "collectTouchException is undefined\n");
#endif
		break;
	case VTS_TOUCH_INFO_COLLECT:
		getFirmwareaVersion(tmp_buf);
		ret = snprintf(buf, 255, "lcmId:%x,fwVersion:%s,statusCount:%x,%x,%x,%x\n", vivoTsGetLcmId(), tmp_buf, vivo_ts_data->stateCount[0],
					vivo_ts_data->stateCount[1], vivo_ts_data->stateCount[2], vivo_ts_data->stateCount[3]);
	break;
	case VTS_TOUCH_INFO_COLLECT_DUMP:
		ret = snprintf(buf, 255, "statusCount:%02x,%02x,%02x,%02x\nstatusBits:%08x,%08x,%08x,%08x", vivo_ts_data->statisticsCount[0], vivo_ts_data->statisticsCount[1],
						vivo_ts_data->statisticsCount[2], vivo_ts_data->statisticsCount[3], vivo_ts_data->statusBit[0],
						vivo_ts_data->statusBit[1], vivo_ts_data->statusBit[2], vivo_ts_data->statusBit[3]);
	break;
	
	default:		/*for goodix udf gesture*/
		ret = snprintf(buf, 255, "%s", "GT9P 0xA684 0x95D4 ");
		break;
	}

	cmd_code = 0;	/*erase cmd, cts test will no effect*/
	return ret;
}
static ssize_t touchscreen_ts_super_node_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("input cmd is %d", val);
	if (val > TEST_NODE_CMD_MAX || val <= 0) {
		VTI("input cmd is out of range:%d", val);
		return count;
	}

	cmd_code = val;

	return count;
}

static ssize_t touchscreen_user_defined_gesture_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d", vivo_ts_data->udfGestureSwitch);
}

static ssize_t touchscreen_user_defined_gesture_enable_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	if (val < 0 || val > 1) {
		VTI("invalide number of parameters passed.");
		return  -EINVAL;
	}

	VTI("set udfGesture:%d", val);
	vivo_ts_data->udfGestureSwitch = val;

	return count;
}

static ssize_t touchscreen_edge_suppress_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d\n", vivo_ts_data->edgeRestainSwitch);
}

static ssize_t touchscreen_edge_suppress_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;
	int ret = 0;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("paramater is %d", val);
	VTI("tsState is %d,---normal is 0,sleep is 1,gesture is 2; hasLcdShutoff is %d", atomic_read(&vivo_ts_data->tsState),vivo_ts_data->hasLcdShutoff);
	mutex_lock(&vivo_ts_data->edgeMutex);
	if (val == 0) {
		vivo_ts_data->edgeRestainSwitch = 0;
	} else if (val == 1) {
		vivo_ts_data->edgeRestainSwitch = 1;
	} else if (val == 2) {
		vivo_ts_data->edgeRestainSwitch = 2;
	} else{
		VTI("invalide parameter:%d", val);
		mutex_unlock(&vivo_ts_data->edgeMutex);
		return -EINVAL;
	}

	if ((atomic_read(&vivo_ts_data->tsState) == TOUCHSCREEN_NORMAL) && (vivo_ts_data->hasLcdShutoff == 0)) {
		ret = vivoTsSetEdgeRestainToChip(vivo_ts_data->edgeRestainSwitch);
		if (ret < 0) {
			VTI("vivoTsSetEdgeRestainToChip fail.");
		}
	}
	mutex_unlock(&vivo_ts_data->edgeMutex);

	return count;
}

static ssize_t touchscreen_fw_update_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	char binFileName[128];
	int ret = 0;
	const struct firmware *firmware = NULL;

	if (!vivo_ts_data->updateFirmware) {
		VTI("ic's update fw function not define.");
		return count;
	}

	/* get bin fw file name */
	if (sscanf(buf, "%s", binFileName) != 1) {
		VTI("invalide parameter");
		return -EINVAL;
	}
	VTI("firmware name:%s", binFileName);

	vivoTsReleasePointsAndKeys();

	ret = request_firmware(&firmware, binFileName, vivo_ts_data->client->dev.parent);
	if (ret != 0) {
		VTI("Firmware %s request fail", binFileName);
		 return -EINVAL;
	}

	vivo_ts_data->fwUpdatingFlag = 1;

	ret = vivoTsUpdateFirmware(firmware);
	if (-FUN_NOT_REG == ret) {
		VTI("vivoTsUpdateFirmware not defined");
	} else if (ret < 0) {
		VTI("vivoTsUpdateFirmware fail, please try again.");
	} else {
		VTI("vivoTsUpdateFirmware success.");
	}

	vivoTsReleasePointsAndKeys();

	vivo_ts_data->fwUpdatingFlag = 0;

	vivo_ts_data->isLargePressMode = 0;
	vivo_ts_data->is3FingerMode = 0;

	release_firmware(firmware);
/*update node just for need, Don't need to be so complicated*/
/*
	//because updating fw, maybe phone suspend and resume, because this situation, we do not real resume or
	//suspend, just set tsState, so here we need reset chip state according tsState
	if (atomic_read(&vivoTsData->tsState) == TOUCHSCREEN_NORMAL) {
		//usb state write
		ret = vivoTsSetChargerFlagToChip(vivoTsData->usbChargerFlag);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetChargerFlagToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetChargerFlagToChip fail.");
		}
		//here just set edge restain by vivoTsData->edgeRestainSwitch
		ret = vivoTsSetEdgeRestainToChip(vivoTsData->edgeRestainSwitch);
		if (-FUN_NOT_REG == ret) {
			VTI("vivoTsSetEdgeRestainToChip no define.");
		} else if (ret < 0) {
			VTI("vivoTsSetEdgeRestainToChip fail.");
		}
		//...glove mode.ref resume process
	} else {
		ret = gestureAndSleepModeExchange(vivoTsData->needChangeToGestureMode);
		if (ret < 0) {
			VTI("gestureAndSleepModeExchange fail");
			return ret;
		}
	}
*/
	return count;
}

static ssize_t touchscreen_gesture_switch_export_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "%d\n", vivo_ts_data->exportSwitch);
}

static ssize_t touchscreen_gesture_switch_export_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		VTI("invalide number of parameters passed.");
		return -EINVAL;
	}

	VTI("set exportSwitch:%d", val);
	vivo_ts_data->exportSwitch = val;

	return count;
}

static int caliOrCheck;
static ssize_t touchscreen_at_sensor_test_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int val = 0;
	int ret = 0;

	ret = sscanf(buf, "%d", &val);
	if (ret != 1) {
		VTI("Invalide number of parameters passed");
		return -EINVAL;
	}

	mutex_lock(&(vivo_ts_data->sensorTestMutex));
	VTI("FTS parameter is %d", val);
	if (val == VTS_SENSOR_TEST_CALIBRATION) {	/*sensor cali*/
		caliOrCheck = VTS_SENSOR_TEST_CALIBRATION;
	} else if (val == VTS_SENSOR_TEST_CHECK) {	/*sensor test*/
		caliOrCheck = VTS_SENSOR_TEST_CHECK;
	} else {
		VTI("Invalide parameter passed:%d", val);
		caliOrCheck = 0;
	}
	mutex_unlock(&(vivo_ts_data->sensorTestMutex));

	return count;
}
static ssize_t touchscreen_at_sensor_test_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not test.\n");
	}

	if (!vivo_ts_data->atSensorTest) {
		VTI("atSensorTest not defined in ic driver.");
		return snprintf(buf, 255, "atSensorTest not defined in ic driver.\n");
	}

	mutex_lock(&(vivo_ts_data->sensorTestMutex));
	if (caliOrCheck == VTS_SENSOR_TEST_CALIBRATION || caliOrCheck == VTS_SENSOR_TEST_CHECK) {
		vivoTsReleasePointsAndKeys();

		VTI("atSensorTest begin");
		ret = vivoAtSensorTest(buf, caliOrCheck, NULL, -1);
		if (ret < 0) {
			ret = snprintf(buf, 255, "fail, function exception\n");
			VTI("at sensor test function exception");
		} else {
			VTI("at sensor test function no exception");
		}

		vivoTsReleasePointsAndKeys();
	}
	caliOrCheck = 0;
	mutex_unlock(&(vivo_ts_data->sensorTestMutex));

	vivo_ts_data->isLargePressMode = 0;
	vivo_ts_data->is3FingerMode = 0;

	return ret;
}

static  int imei_read_mode;
static ssize_t imei_ctl_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int ret = 0;
	int i = 0;

	if (vivoTsGetState() != TOUCHSCREEN_NORMAL) {
		VTI("not in TOUCHSCREEN_NORMAL mode, could not write imei");
		return -EINVAL;
	}

	if (count <= 15) {
		VTE("Invalide number of parameters.");
		return -EINVAL;
	}

	for (i = 0; i < 15; i++) {
		if (buf[i] != '0') {
			break;
		}
	}

	if (i == 15) {
		imei_read_mode = 1;
		VTI("check into imei_read_mode");
		return count;
	}

	ret = vivoTsWriteImei((unsigned char *)buf);
	if (ret < 0) {
		VTI("write imei fail or imei interface not define");
		return -EINVAL;
	}

	return count;
}
static ssize_t imei_ctl_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	int i = 0;
	unsigned char read_config_buf[16] = {0};

	if (vivoTsGetState() != TOUCHSCREEN_NORMAL) {
		VTI("not in TOUCHSCREEN_NORMAL mode,could not read imei");
		return snprintf(buf, 255, "ID is locked\n");
	}

	if (imei_read_mode == 0) {
		VTI("not in touchscreen data read mode");
		return snprintf(buf, 255, "not in ID read mode\n");
	}
	imei_read_mode = 0;

	if (vivoTsGetVtsData()->isProbeComplete == 0 || vivoTsGetVtsData()->fwUpdatingFlag == 1) {
		VTI("probe is not complete of fw is update\n");
		return snprintf(buf, 255, "Touch firwmare update");
	}

	ret = vivoTsReadImei((unsigned char *)read_config_buf);
	if (ret < 0) {
		VTI("read imei fail or imei interface not define");
		if (ret == -FUN_NOT_REG) {
			return snprintf(buf, 255, "Touch not support ID\n");
		}
		return snprintf(buf, 255, "read ID fail\n");
	}

	/* ID is no flashed */
	/* is all 0x00 judge*/
	for (i=0; i<15; i++) {
		if (read_config_buf[i] == 0x00) {
			continue;
		} else {
			break;
		}
	}
	if (i == 15) {
		VTI("ID is no flashed,all is 0x00");
		return snprintf(buf, 255, "ID is no flashed\n");
	}
	/* is all 0xff judge*/
	for (i=0; i<15; i++) {
		if (read_config_buf[i] == 0xff) {
			continue;
		} else {
			break;
		}
	}
	if (i == 15) {
		VTI("ID is no flashed,all is 0xff");
		return snprintf(buf, 255, "ID is no flashed\n");
	}

	/* ID is invalid */
	for (i=0; i<15; i++) {
		if (read_config_buf[i] <= '9' && read_config_buf[i] >= '0') {
			continue;
		} else {
			break;
		}
	}
	if (i != 15) {
		VTI("ID is invalid,out of the range 0~9");
		return snprintf(buf, 255, "ID is invalid\n");
	}
	
	return snprintf(buf, 255, "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n",
			read_config_buf[0]-0x30,
			read_config_buf[1]-0x30,
			read_config_buf[2]-0x30,
			read_config_buf[3]-0x30,
			read_config_buf[4]-0x30,
			read_config_buf[5]-0x30,
			read_config_buf[6]-0x30,
			read_config_buf[7]-0x30,
			read_config_buf[8]-0x30,
			read_config_buf[9]-0x30,
			read_config_buf[10]-0x30,
			read_config_buf[11]-0x30,
			read_config_buf[12]-0x30,
			read_config_buf[13]-0x30,
			read_config_buf[14]-0x30

		);
}

static ssize_t touchscreen_cali_support_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{	
	return snprintf(buf, 128, "%d", vivo_ts_data->caliSupport);
}

static ssize_t touchscreen_imei_write_support_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{	
	return snprintf(buf, 128, "%d", vivo_ts_data->imeiWriteSupport);
}				

static ssize_t touchscreen_factory_key_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	if (!vivo_ts_data->sensorTestKey) {
		vivo_ts_data->sensorTestKey = "null:null:null:null:null";
	}
	if (!vivo_ts_data->lcmNoiseTestKey) {
		vivo_ts_data->lcmNoiseTestKey = "null:null:null:null:null";
	}
	if (!vivo_ts_data->bspLcmNoiseTestKey) {
		vivo_ts_data->bspLcmNoiseTestKey = "null:null:null:null:null";
	}
	if (!vivo_ts_data->rawdataTestKey) {
		vivo_ts_data->rawdataTestKey = "null:null:null:null:null";
	}
	if (!vivo_ts_data->RFTestKey) {
		vivo_ts_data->RFTestKey = "null:null:null:null:null";
	}
	
	return snprintf(buf, 2048, "bad_screen:%s\nlcm_noise:%s\nbsp_lcm_noise:%s\nrawdata_test:%s\nRF_test:%s\n", vivo_ts_data->sensorTestKey, 
		vivo_ts_data->lcmNoiseTestKey, vivo_ts_data->bspLcmNoiseTestKey, vivo_ts_data->rawdataTestKey, vivo_ts_data->RFTestKey);
}
static ssize_t touchscreen_app_name_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{

	if (atomic_read(&vivo_ts_data->tsState) != TOUCHSCREEN_NORMAL) {
		return snprintf(buf, 255, "not in ts normal mode, could not test.\n");
	}
	if (vivo_ts_data->app_name[0] == '\0') {
		return snprintf(buf, 255, "no valid app name write to driver\n");
	}
	return snprintf(buf, 255, "%s\n", vivo_ts_data->app_name);
}
static ssize_t touchscreen_app_name_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	int i = 0;

	if (vivoTsGetState() != TOUCHSCREEN_NORMAL) {
		VTI("not in TOUCHSCREEN_NORMAL mode,could not write app name");
		return -EINVAL;
	}

	if (count >= 256) {
		VIVO_TS_LOG_ERR("app name to long, out of 256 bytes\n");
		return  -EINVAL;
	}

	memset(vivo_ts_data->app_name, '\0', 256);
	for (i = 0; i < count; i++) {
		vivo_ts_data->app_name[i] = (unsigned char)buf[i];
	}

	if (vivo_ts_data->processByPackage) {
		vivo_ts_data->processByPackage(vivo_ts_data->app_name);
	}

	VIVO_TS_LOG_INF("cur app name:%s\n", vivo_ts_data->app_name);
	VIVO_TS_LOG_INF("last app name:%s\n", vivo_ts_data->last_app_name);

	memset(vivo_ts_data->last_app_name, '\0', 256);
	for (i = 0; i < count; i++) {
		vivo_ts_data->last_app_name[i] = (unsigned char)vivo_ts_data->app_name[i];
	}
	return count;
}

static ssize_t touchscreen_data_info_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, 255, "large_press_num:%d,iic_error_num:%d,probe_ito_test_result:%d\n", vivo_ts_data->largePressNum, vivo_ts_data->iicErrorNum, vivo_ts_data->itoTestResult);

	VIVO_TS_LOG_INF("large_press_num:%d,iic_error_num:%d,probe_ito_test_result:%d\n", vivo_ts_data->largePressNum, vivo_ts_data->iicErrorNum, vivo_ts_data->itoTestResult);
	vivo_ts_data->largePressNum = 0;
	vivo_ts_data->iicErrorNum = 0;

	return ret;
}

static ssize_t touchscreen_finger_gesture_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val = 0;

	if (vivo_ts_data->hasFingerGesture) {
		if (sscanf(buf, "%d", &val) != 1) {
			VTI("Invalide number of parameters passed");
			return -EINVAL;
		}
		VTI("finger gesture switch store, parameter is %d", val);
		if (val < 0x80) {
			if (val == 2 || val == 3) {
				if (val == 2) {
					VTI("2 min timeout");
					vivo_ts_data->fg2MinTimeout = 1;
					if (vivo_ts_data->fgProximityRtState == 0) {
						touchscreen_request_send(TOUCHSCREEN_REQ_ID_PROX_STATE, 0);
					} else {
						VTI("proximity has been move away,not need change to sleep mode");
						vivo_ts_data->fg2MinTimeout = 0;	
					}
					return count;
				} else if (val == 3) {
					VTI("clean 2 min timeout,do nothing");
					return count;
				}
			} else if (val == 6 || val == 7) {
				if (val == 6) {
					VTI("The Fingerprint icon is light");
					atomic_set(&vivo_ts_data->fingerIcoState, 1);
					vivoTsScanRate(1);
				} else if (val == 7) {
					VTI("The Fingerprint icon is extinguish");
					atomic_set(&vivo_ts_data->fingerIcoState, 0);
					vivoTsScanRate(3);
				}
			}
		} else {
			VTI("Invalide parameter passed(%d)", val);
			return -EINVAL;
		}
	}

	return count;
}

/*for face detect start start*/
static ssize_t touchscreen_faceDetect_enableDisable_store(struct kobject *kobj,
					struct kobj_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int ret;
	if (sscanf(buf, "%d", &val) != 1) {
		VTI("Invalide number of parameters passed");
		return -EINVAL;
	}
		VTI("faceDetect enableDisable store, parameter is %d", val);
		
	ret = vivoFaceDetectStartEnd(val);
		if (ret <= 0) {
			VTI("vivoFaceDetectStartEnd fail.");
		}

	return count;
}
static struct touchscreen_driver_sysfs_entry touchscreen_gesture_switch_export =
	__ATTR(gesture_switch_export, 0644,
			touchscreen_gesture_switch_export_show, touchscreen_gesture_switch_export_store);
static struct touchscreen_driver_sysfs_entry touchscreen_gesture_point =
	__ATTR(gesture_point, 0644,
			touchscreen_gesture_point_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_lcd_state =
	__ATTR(dclick_lcd_state, 0644,
			touchscreen_dclick_lcd_state_show, touchscreen_dclick_lcd_state_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_proximity_switch =
	__ATTR(dclick_proximity_switch, 0644,
			touchscreen_null_show, touchscreen_dclick_proximity_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_gesture_switch =
	__ATTR(gesture_switch, 0644,
			touchscreen_gesture_switch_show, touchscreen_gesture_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_swipe_switch =
	__ATTR(swipe_switch, 0644,
			touchscreen_swipe_switch_show, touchscreen_swipe_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_switch =
	__ATTR(dclick_switch, 0644,
			touchscreen_dclick_switch_show, touchscreen_dclick_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_fts_is_calling =
	__ATTR(ts_is_calling, 0644,
			fts_is_calling_show, fts_is_calling_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_rx_tx =
	__ATTR(sensor_rx_tx, 0644,
			touchscreen_sensor_rx_tx_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_firmware_module_id =
	__ATTR(firmware_module_id, 0644,
			touchscreen_firmware_module_id_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_firmware_version =
	__ATTR(firmware_version, 0644,
			touchscreen_version_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_touchpanel_device =
	__ATTR(touchpanel_devices, 0644,
			touchscreen_touch_ic_name_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_delta =
	__ATTR(sensor_delta, 0644,
			touchscreen_delta_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_test =
	__ATTR(sensor_test, 0644,
			touchscreen_sensor_test_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_rawdata =
	__ATTR(sensor_rawdata, 0644,
			touchscreen_rawdata_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_ic_mode =
	__ATTR(touch_ic_mode, 0644, touchscreen_ic_mode_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_ts_log_switch =
	__ATTR(ts_log_switch, 0644, touchscreen_log_switch_show, touchscreen_log_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_ts_super_node =
	__ATTR(ts_super_node, 0644, touchscreen_ts_super_node_show, touchscreen_ts_super_node_store);
static struct touchscreen_driver_sysfs_entry touchscreen_user_defined_gesture_enable =
	__ATTR(user_defined_gesture_enable, 0644,
			touchscreen_user_defined_gesture_enable_show, touchscreen_user_defined_gesture_enable_store);
static struct touchscreen_driver_sysfs_entry touchscreen_edge_suppress =
	__ATTR(edge_suppress_switch, 0644, touchscreen_edge_suppress_show, touchscreen_edge_suppress_store);
static struct touchscreen_driver_sysfs_entry touchscreen_data_info =
	__ATTR(ts_data_info, 0644, touchscreen_data_info_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_firmware_update =
	__ATTR(firmware_update, 0644, touchscreen_null_show, touchscreen_fw_update_store);
static struct touchscreen_driver_sysfs_entry touchscreen_at_sensor_test =
	__ATTR(at_sensor_test, 0644, touchscreen_at_sensor_test_show, touchscreen_at_sensor_test_store);
static struct touchscreen_driver_sysfs_entry touchscreen_imei_ctl =
	__ATTR(imei_ctl, 0644, imei_ctl_show, imei_ctl_store);
static struct touchscreen_driver_sysfs_entry touchscreen_app_name =
	__ATTR(app_name, 0644, touchscreen_app_name_show, touchscreen_app_name_store);
static struct touchscreen_driver_sysfs_entry touchscreen_finger_gesture_switch =
	__ATTR(finger_gesture_switch, 0644,
			touchscreen_null_show, touchscreen_finger_gesture_switch_store);

static struct touchscreen_driver_sysfs_entry touchscreen_factory_key =
	__ATTR(factory_key, 0644,
			touchscreen_factory_key_show, touchscreen_null_store);

static struct touchscreen_driver_sysfs_entry touchscreen_cali_support =
	__ATTR(cali_support, 0644,
			touchscreen_cali_support_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_imei_write_support =
	__ATTR(imei_write_support, 0644,
			touchscreen_imei_write_support_show, touchscreen_null_store);
/*for face detect start*/
static struct touchscreen_driver_sysfs_entry touchscreen_faceDetect_enableDisable =
	__ATTR(faceDetect_enableDisable, 0644,
			touchscreen_null_show, touchscreen_faceDetect_enableDisable_store);
static struct attribute *our_own_sys_attrs[] = {

	&touchscreen_ic_mode.attr,
	&touchscreen_ts_log_switch.attr,
	&touchscreen_sensor_rawdata.attr,

	&touchscreen_sensor_test.attr,
	&touchscreen_at_sensor_test.attr,
	&touchscreen_imei_ctl.attr,
	&touchscreen_app_name.attr,

	&touchscreen_sensor_delta.attr,
	&touchscreen_touchpanel_device.attr,
	&touchscreen_firmware_version.attr,
	&touchscreen_firmware_module_id.attr,
	&touchscreen_sensor_rx_tx.attr,
	&touchscreen_fts_is_calling.attr,

	&touchscreen_dclick_switch.attr,
	&touchscreen_swipe_switch.attr,
	&touchscreen_gesture_switch.attr,
	&touchscreen_gesture_switch_export.attr,
	&touchscreen_dclick_proximity_switch.attr,
	&touchscreen_dclick_lcd_state.attr,

	&touchscreen_gesture_point.attr,
	&touchscreen_ts_super_node.attr,
	&touchscreen_user_defined_gesture_enable.attr,
	&touchscreen_edge_suppress.attr,
	&touchscreen_data_info.attr,
	&touchscreen_firmware_update.attr,
	&touchscreen_finger_gesture_switch.attr,
	&touchscreen_factory_key.attr,

	&touchscreen_cali_support.attr,
	&touchscreen_imei_write_support.attr,
	/*for face detect*/
	&touchscreen_faceDetect_enableDisable.attr,
	/*
	&touchscreen_sensor_self_rawdata.attr,
	&touchscreen_gesture_index.attr,
	&touchscreen_user_defined_gesture_enable.attr,

	//&touchscreen_power_state.attr,
	//&touchscreen_gloves_mode_switch.attr,
	//&touchscreen_interrupt_enable.attr,

	&touchscreen_dclick_simulate_switch.attr,
*/
	NULL
};
static ssize_t touchscreen_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t touchscreen_debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void touchscreen_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops touchscreen_debug_object_sysfs_ops = {
	.show = touchscreen_debug_object_show,
	.store = touchscreen_debug_object_store,
};
static struct kobj_type touchscreen_debug_object_type = {
	.sysfs_ops	= &touchscreen_debug_object_sysfs_ops,
	.release	= touchscreen_debug_object_release,
	.default_attrs = our_own_sys_attrs,
};

#define USER_DEFINE_GESTURE_NODE 0
#if USER_DEFINE_GESTURE_NODE
/*user define gesture node*/
static struct attribute *touchscreen_guest_attrs[] = {
	/*user define gesture*/
	/*
	&touchscreen_userdefine_gesture_template.attr,
	&touchscreen_userdefine_gesture_remove.attr,
	&touchscreen_userdefine_gesture_template_valid.attr,
	*/
	NULL
};

static ssize_t touchscreen_guest_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t touchscreen_guest_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}
static void touchscreen_guest_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops touchscreen_guest_object_sysfs_ops = {
	.show = touchscreen_guest_object_show,
	.store = touchscreen_guest_object_store,
};
static struct kobj_type touchscreen_guest_object_type = {
	.sysfs_ops	= &touchscreen_guest_object_sysfs_ops,
	.release	= touchscreen_guest_object_release,
	.default_attrs = touchscreen_guest_attrs,
};
#endif

static ssize_t vivoTsvkeysShow(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	int rc = -EINVAL;
	if (vivo_ts_data->vkeyString != NULL) {
		VTI("VKEY= %s", vivo_ts_data->vkeyString);
		rc = snprintf(buf, 255, "%s\n", vivo_ts_data->vkeyString);
	} else {
		VTI("VKEY support failed or please check dts");
	}

	return rc;
}
static struct kobj_attribute vivoTsVkeysAttr = {
	.attr = {
		.name = "virtualkeys.vivo_ts", /*shijianxing add:virtualkeys + input devices name*/
		.mode = S_IRUGO,
	},
	.show = &vivoTsvkeysShow,
};

static struct attribute *vivoTsPropertiesAttrs[] = {
	&vivoTsVkeysAttr.attr,
	NULL
};
static struct attribute_group vivoTsPropertiesAttrGroup = {
	.attrs = vivoTsPropertiesAttrs,
};

int vivoTsCreatSysFsFile(struct vivo_ts_struct *vtsData)
{
	int ret;

	vivo_ts_data = vtsData;

	VTI("create vivo ts sys/touchscreen node");
	ret = kobject_init_and_add(&vtsData->kobjectDebug, &touchscreen_debug_object_type,
					NULL, "touchscreen");
	if (ret) {
		VTE("create kobjetct error!");
		ret = -1;
		return ret;
	}

#if USER_DEFINE_GESTURE_NODE
	/*add guest gesture dir*/
	ret = kobject_init_and_add(&vtsData->kobjectGuest, &touchscreen_guest_object_type, &vtsData->kobject_debug, "guest");
	if (ret) {
		VTE("create kobjetct guest error!");
		ret = -1;
		return ret;
	}
#endif

	if (VTS_KEY_2D == vtsData->keyType) {
		VTI("2D key, create vivo ts vkey node");
		if (vtsData->vkeyString != NULL) {
			vtsData->propertiesKobj = kobject_create_and_add("board_properties", NULL);
			if (vtsData->propertiesKobj) {
				ret = sysfs_create_group(vtsData->propertiesKobj, &vivoTsPropertiesAttrGroup);
			}
			if (!vtsData->propertiesKobj || ret) {
				VTD("failed to create board_properties");
			}
		} else {
			VTI("create vivo ts vkey node fail, vkeyString is NULL, if 2d key, will no function.");
		}
	}

	return 0;
}

void vivoTsRemoveSysFsFile(struct vivo_ts_struct *vtsData)
{

	VTI("remove vivo ts sys/touchscreen node");

	if (VTS_KEY_2D == vtsData->keyType) {
		VTI("2D key, remove vivo ts vkey node");
		if (vtsData->vkeyString != NULL) {
			if (vtsData->propertiesKobj) {
				sysfs_remove_group(vtsData->propertiesKobj, &vivoTsPropertiesAttrGroup);
				kobject_del(vtsData->propertiesKobj);
				vtsData->propertiesKobj = NULL;
			}
		} else {
			VTI("remove vivo ts vkey node fail, vkeyString is NULL, if 2d key, will no function.");
		}
	}

#if USER_DEFINE_GESTURE_NODE
	/*add guest gesture dir*/
	kobject_del(&vtsData->kobjectGuest);
#endif

	kobject_del(&vtsData->kobjectDebug);

	vivo_ts_data = NULL;

	return;
}
