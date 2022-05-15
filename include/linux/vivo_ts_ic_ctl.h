#ifndef _VIVO_TS_IC_CTL_H_
#define _VIVO_TS_IC_CTL_H_
#define RAW_DATA 0
#define DIFF_DATA 1

#define FW_VERSION 0
#define CONFIG_VERSION 1

#define VTS_EDGE_V	1
#define VTS_EDGE_H	0

extern int vivoTsIcCtlInit(struct vivo_ts_struct *vtsData);
extern int vivoTsGetRawOrDiffDataFromChip(int which, int *data);
extern int vivoTsGetIcFirmwareOrConfigVersion(int FwOrConfig);
extern int vivoTsGetGesturePointDataFromChip(u16 *data);
extern int vivoTsGetModuleId(void);
extern int vivoTsSetEdgeRestainToChip(int on);
extern int vivoTsSetGloveModeToChip(int on);
extern int vivoTsSetGestureToChip(unsigned int gesture_bit_state);
extern int vivoTsSetChargerFlagToChip(int state);
extern int vivoTsGetRxTxNumFromIc(void);
extern int vivoTsGetTsPinValue(void);
extern int vivoTsSetIcPower(int on);
extern int vivoTsHardwareReset(int tmp);
extern int vivoTsSoftwareReset(int tmp);
extern int vivoTsModeChange(int which);
extern int vivoTsUpdateFirmware(const struct firmware *firmware);
extern int vivoSensorTest(char *buf, void *pdata, int tmp);
extern int vivoAtSensorTest(char *buf, int caliOrCheck, void *pdata, int tmp);
extern int getFirmwareaVersion(char *buf);
extern int vivoTsIdleEnableOrDisable(int state);
extern int vivoTsReadImei(unsigned char *imei);
extern int vivoTsWriteImei(unsigned char *imei);
extern int vivoTsGetHeaderFileFWVersionOrConfig(int which, unsigned char *fw);
extern int vivoTsGetOrSet(int which);
extern int vivoTsOtherInfo(unsigned char *buf);
extern int vivoTsScanRate(int rate);
extern int vivoFaceDetectStartEnd(int state);
extern int vivoTsReadIcMode(void);
#endif
