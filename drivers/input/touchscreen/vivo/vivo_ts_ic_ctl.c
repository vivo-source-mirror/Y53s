#include <linux/vivo_ts_function.h>

static struct vivo_ts_struct *vivoTsData;

int vivoTsIcCtlInit(struct vivo_ts_struct *vtsData)
{
	vivoTsData = vtsData;
	return 0;
}

/*
 * which:
 *  	1:diffdata
 * 	2:rawdata
 *	3:keydiff
 *	4:keyraw
 * return 0:success
 */
int vivoTsGetRawOrDiffDataFromChip(int which, int *data)
{
	if (vivoTsData->getRawOrDiffData)
		return vivoTsData->getRawOrDiffData(which, data);
	else
		return -FUN_NOT_REG;
}
/*
 * return:
 * 	<0:fail
 * 	>0:gesture point number
 * data pkg formate:
 * 	u16 u16 u16 u16...
 * 	x1   y1   x2   y2....
 */
int vivoTsGetGesturePointDataFromChip(u16 *data)
{
	if (vivoTsData->getGesturePointData)
		return vivoTsData->getGesturePointData(data);
	else
		return -FUN_NOT_REG;
}

/*
 * FW_VERSION 0
 * CONFIG_VERSION 1
 *
 *
 */
int vivoTsGetIcFirmwareOrConfigVersion(int FwOrConfig)
{
	if (vivoTsData->getIcFirmwareOrConfigVersion)
		return vivoTsData->getIcFirmwareOrConfigVersion(FwOrConfig);
	else
		return -FUN_NOT_REG;
}
/*
*Fomate firmware into string
*max length 20
*
*
*
*/
int getFirmwareaVersion(char *buf)
{
	int fwVersion = -1;
	int configVersion = -1;
	int ret = 0;

	fwVersion = vivoTsGetIcFirmwareOrConfigVersion(FW_VERSION);
	if (-FUN_NOT_REG == fwVersion) {
		VTI("vivoTsGetIcFirmwareOrConfigVersion no defined.\n");
	} else if (fwVersion < 0) {
		VTI("read firmware version fail.\n");
	}

	configVersion = vivoTsGetIcFirmwareOrConfigVersion(CONFIG_VERSION);
	if (-FUN_NOT_REG == configVersion) {
		VTI("vivoTsGetIcFirmwareOrConfigVersion no defined.\n");
	} else if (configVersion < 0) {
		VTI("read config version fail.\n");
	}

	if (fwVersion < 0) {
		ret = snprintf(buf, 20, "Read Fail");
	} else if ((fwVersion > 0) && (configVersion > 0)) {
		ret = snprintf(buf, 20, "0x%04x0x%04x", fwVersion, configVersion);
	} else if ((fwVersion > 0) && (configVersion <= 0)) {
		ret = snprintf(buf, 20, "0x%04x", fwVersion);
	}
	return ret;
}

/*
 * return:
 * 	>=0:success
 *	<0:fail
 */
int vivoTsGetModuleId(void)
{
	if (vivoTsData->getModuleId)
		return vivoTsData->getModuleId();
	else
		return -FUN_NOT_REG;
}

int vivoTsGetLcmId(void)
{
	if (vivoTsData->getLcmId)
		return vivoTsData->getLcmId();
	else
		return -FUN_NOT_REG;
}

/*
 * parameter:1(screen v) or 0(screen H)
 * return:
 * 0:success
 * <0:fail
 */
int vivoTsSetEdgeRestainToChip(int on)
{
	if (vivoTsData->setEdgeRestainSwitch)
		return vivoTsData->setEdgeRestainSwitch(on);
	else
		return -FUN_NOT_REG;
}

int vivoTsSetGloveModeToChip(int on)
{
	if (vivoTsData->setGloveModeSwitch)
		return vivoTsData->setGloveModeSwitch(on);
	else
		return -FUN_NOT_REG;
}

int vivoTsSetGestureToChip(unsigned int gesture_bit_state)
{
	if (vivoTsData->setGestureSwitch)
		return vivoTsData->setGestureSwitch(gesture_bit_state);
	else
		return -FUN_NOT_REG;
}

/*return 0,success*/
int vivoTsSetChargerFlagToChip(int state)
{
	if (vivoTsData->setChargerFlagSwitch)
		return vivoTsData->setChargerFlagSwitch(state);
	else
		return -FUN_NOT_REG;
}

/*
 * return
 * 	-FUN_NOT_REG:rx/tx not read from ic,so no this function defined
 *	-1:get from ic fail
 * 	>0:success,return value is:[rxNum<<8 | txNum]
 ***/
int vivoTsGetRxTxNumFromIc(void)
{
	if (vivoTsData->getRxTxNumFromIc)
		return vivoTsData->getRxTxNumFromIc();
	else
		return -FUN_NOT_REG;
}

/*
 * get ts pin value to show which module we should select
 * format:id1<1 | id0
 */
int vivoTsGetTsPinValue(void)
{
	if (vivoTsData->getTsPinValue)
		return vivoTsData->getTsPinValue();
	else
		return -FUN_NOT_REG;
}

int vivoTsSetIcPower(int on)
{
	if (vivoTsData->setIcPower)
		return vivoTsData->setIcPower(on);
	else
		return -FUN_NOT_REG;
}

int vivoTsHardwareReset(int tmp)
{
	if (vivoTsData->hardwareReset)
		return vivoTsData->hardwareReset(tmp);
	else
		return -FUN_NOT_REG;
}
int vivoTsSoftwareReset(int tmp)
{
	if (vivoTsData->softwareReset)
		return vivoTsData->softwareReset(tmp);
	else
		return -FUN_NOT_REG;
}

/* ic mode switch
 * TOUCHSCREEN_NORMAL TOUCHSCREEN_SLEEP TOUCHSCREEN_GESTURE
 * return 0,success
 */
int vivoTsModeChange(int which)
{
	if (vivoTsData->icModeChange)
		return vivoTsData->icModeChange(which);
	else
		return -FUN_NOT_REG;
}

/*
 * name:firmware bin file name,length must than 128 bytes
 *
 ****/
int vivoTsUpdateFirmware(const struct firmware *firmware)
{
	if (vivoTsData->updateFirmware) {
		return vivoTsData->updateFirmware(firmware);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
 * read data from ic
 *
 */
int vivoTsReadRegister(unsigned int addr)
{
	if (vivoTsData->readRegister) {
		return vivoTsData->readRegister(addr);
	} else {
		return -FUN_NOT_REG;
	}
}

int vivoTsReadIcMode(void)
{
	if (vivoTsData->getIcMode) {
		return vivoTsData->getIcMode();
	} else {
		return -FUN_NOT_REG;
	}
}

/*
 * write data to ic
 *
 */
int vivoTsWriteRegister(unsigned int addr, unsigned int data)
{
	if (vivoTsData->writeRegister) {
		return vivoTsData->writeRegister(addr, data);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
 * sensor test
 * return <0:run fail	>0:run success,return buf data count(include test success or fail)
 */
int vivoSensorTest(char *buf, void *pdata, int tmp)
{
	if (vivoTsData->sensorTest) {
		return vivoTsData->sensorTest(buf, pdata, tmp);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
 * at sensor test
 * return <0:run fail	>0:run success,return buf data count(include test success or fail)
 */
int vivoAtSensorTest(char *buf, int caliOrCheck, void *pdata, int tmp)
{
	if (vivoTsData->atSensorTest) {
		return vivoTsData->atSensorTest(buf, caliOrCheck, pdata, tmp);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
 * while power on,use this interface could enable or disable the idle mode
 */
int vivoTsIdleEnableOrDisable(int state)
{
	if (vivoTsData->idleEnableOrDisable) {
		return vivoTsData->idleEnableOrDisable(state);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
*
*/
int vivoTsReadImei(unsigned char *imei)
{
	if (vivoTsData->readImei) {
		return vivoTsData->readImei(imei);
	} else {
		return -FUN_NOT_REG;
	}
}
int vivoTsWriteImei(unsigned char *imei)
{
	if (vivoTsData->writeImei) {
		return vivoTsData->writeImei(imei);
	} else {
		return -FUN_NOT_REG;
	}
}

int vivoTsGetHeaderFileFWVersionOrConfig(int which, unsigned char *fw)
{
	 if (vivoTsData->getHeaderFileFWVersionOrConfig) {
		return vivoTsData->getHeaderFileFWVersionOrConfig(which, fw);
	} else {
		return -FUN_NOT_REG;
	}
}

int vivoTsGetOrSet(int which)
{
	if (vivoTsData->getOrSet) {
		return vivoTsData->getOrSet(which);
	} else {
		return -FUN_NOT_REG;
	}
}

int vivoTsOtherInfo(unsigned char *buf)
{
	if (vivoTsData->otherInfo) {
		return vivoTsData->otherInfo(buf);
	} else {
		return -FUN_NOT_REG;
	}
}

/*
* return 0: this function not useble
* return 1: success
* return -1: fail
* return -500: not add this interface
* state: 0-->low,1-->hi
*/
int vivoTsTsIn(int state)
{
	if (!vivoTsData) {
		VTI("vivoTsData is NULL");
		return 0;
	}
	
	if (vivoTsData->tsInCtl) {
		return vivoTsData->tsInCtl(state);
	} else {
		return -FUN_NOT_REG;
	}
	
	return 0;
}

int vivoTsScanRate(int rate)
{
	if (!vivoTsData) {
		VTI("vivoTsData is NULL");
		return 0;
	}
	
	if (vivoTsData->scanRateChange) {
		return vivoTsData->scanRateChange(rate);
	} else {
		return -FUN_NOT_REG;
	}
	
	return 0;
}

/*for face detect*/
int vivoFaceDetectStartEnd(int state)
{
	if (vivoTsData->faceStartEnd) {
		return vivoTsData->faceStartEnd(state);
	} else {
		return -FUN_NOT_REG;
	}
	
	return 0;

}