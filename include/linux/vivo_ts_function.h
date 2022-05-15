#ifndef _VIVO_TS_FUNCTION_H_
#define _VIVO_TS_FUNCTION_H_

#include <linux/wait.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define VTS_SUSPEND_LEVEL 1
#endif
#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include <linux/input/vivo_key.h>
#include <linux/vivo_ts_prox.h>

#define 	VIVO_TS_FUNCTION_VERSION 0x180625

#define 	VTI(fmt, arg...)		VIVO_TS_LOG_INF("[%s]"fmt"\n", __func__, ##arg)
#define 	VTE(fmt, arg...)		VIVO_TS_LOG_ERR("[%s]"fmt"\n", __func__, ##arg)
#define 	VTD(fmt, arg...)		VIVO_TS_LOG_DBG("[%s]"fmt"\n", __func__, ##arg)

#define	VIVO_TS_MAX_TOUCH_NUM 10

/*4322 lcm and tp process sync*/
#define 	VTS_RESUME_WAIT	0
#define 	VTS_RESUME_DONE	1

/*use for vtsInterruptBottomThread*/
#define VTS_IRQ_BTM_KTHREAD_ZOMBIE 0x00000001

/*use for vivoTsModeChange*/
#define 	TOUCHSCREEN_NORMAL  	0
#define	TOUCHSCREEN_SLEEP   	1
#define	TOUCHSCREEN_GESTURE 	2

/*use for vivoTsModeChange*/
#define	CALLBACK_FINISH   0
#define	CALLBACK_RESUME   1
#define	CALLBACK_SUSPEND  2

/* add for bus type define */
#define VTS_BUS_I2C		0
#define VTS_BUS_SPI		1

/*add for fupport write imei*/
#define UNSUPPORT 0 
#define SUPPORT 1

/*use for vivoTsInputReport*/
#define 	VTS_TOUCH_UP 		0
#define 	VTS_TOUCH_DOWN 	1
#define 	VTS_GESTURE_EVENT 	2
#define 	VTS_RELEASE_POINTS	3
#define 	VTS_RELEASE_KEYS		4
#define 	VTS_VIRTUAL_KEY	5

#define	FUN_NOT_REG	500

#define VTS_PLATFORM_QCOM 0
#define VTS_PLATFORM_MTK_NEW 1	/*mtk new driver software framewark*/
#define VTS_PLATFORM_MTK_OLD 2

#define VTS_KEY_NOKEY	0
#define VTS_KEY_0D	1
#define VTS_KEY_2D	2

#define VTS_SENSOR_TEST_CALIBRATION	165
#define VTS_SENSOR_TEST_CHECK			166

/*ic code */
#define VTS_IC_FTS2		11
#define VTS_IC_FTS3		12
#define VTS_IC_FTM4		13

#define VTS_IC_S3202	21
#define VTS_IC_S3203	22
#define VTS_IC_S3508	23
#define VTS_IC_S3501	24
#define VTS_IC_S3528	25
#define VTS_IC_S3310	26
#define VTS_IC_S3320	124
#define VTS_IC_S3718	28
#define VTS_IC_S3502	29
#define VTS_IC_S3606	61
#define VTS_IC_S1302	62
#define VTS_IC_S3706	63
#define VTS_IC_TD4322	123
#define VTS_IC_TD3320	124
#define VTS_IC_TD4330	125

#define VTS_IC_FT5316	31
#define VTS_IC_FT5336	32
#define VTS_IC_FT5446	33
#define VTS_IC_FT8006	34
#define	VTS_IC_FT8736	35
#define	VTS_IC_FT8719	36

#define VTS_IC_GT970	41
#define VTS_IC_GT9159	42
#define VTS_IC_GT1151	43
#define VTS_IC_GT9286	44
#define VTS_IC_GT5688	45
#define VTS_IC_GT9886	46

#define VTS_IC_CYTMA568	51
#define VTS_IC_CYTMA545	52
#define VTS_IC_CP3155	53

#define VTS_IC_NT36772	81
#define VTS_IC_NT36525	82
#define VTS_IC_NT36670	83

#define VTS_IC_SEC_Y661 70
#define VTS_IC_SEC_TDDI 71

#define	VTS_IC_CSA37F60	90

#define VTS_IC_TD4330_N 126


/*module vendor code--->MVC*/
#define VTS_MVC_LG	0xB0
#define VTS_MVC_BIE	0x3B
#define VTS_MVC_TRY	0x70
#define VTS_MVC_ECH	0x80
#define VTS_MVC_LNS	0x10
#define VTS_MVC_BYD	0x59
#define VTS_MVC_SAM	0x90
#define VTS_MVC_JDI	0xC0
#define VTS_MVC_BOE	0xD0
#define VTS_MVC_SAP	0xE9
#define VTS_MVC_TMA	0xF0
#define VTS_MVC_AUO	0x30

/*irq botton type*/
#define VTS_IRQ_BTM_TYPE_ONESHOOT_THREAD 	0
#define VTS_IRQ_BTM_TYPE_RT_THREAD 			1
#define VTS_IRQ_BTM_TYPE_WORK 				2
#define VTS_IRQ_BTM_THREAD_DEFAULT_PRIORITY 40	/*rt priority*/

#define VTS_NONE 	-1

#ifndef KEY_TS_SWIPE
#define KEY_TS_SWIPE	746
#endif

#define VTS_VKEY_OPTIMIZE_WIDTH 6
#define VTS_GESTURE_ARRAY_LEN 40
#define VTS_GESTURE_POINT 0
#define VTS_GESTURE_O_DIR 1

/* Add for vivo touchscreen data collect*/
#define SCREEN_ON_DURATION		(10 * HZ)

#define VTS_FW_MAX_FW_NUM	30
#define VTS_FW_NOPIN_VALUE	(-500)
#define VTS_FW_PIN_0		0
#define VTS_FW_PIN_1		1
#define VTS_FW_NOLCM_ID		(-500)
#define VTS_FW_TYPE_FW	0
#define VTS_FW_TYPE_IHEX	1
#define VTS_FW_TYPE_CONFIG	2
#define VTS_FW_TYPE_LIMIT	3
#define VTS_FW_TYPE_MP  4

struct VtsFwDesc {
	unsigned char *productName;
	int modulePinValue;
	int moduleVendor;
	int lcmId;

	unsigned char *data;
	int dataSize;
	char dataFlag;
};
struct VtsFwManager {
	struct VtsFwDesc vtsFwDesc[VTS_FW_MAX_FW_NUM];
	int curFwNum;
};

struct VtsFcFilter {
	/* Fast Change Filter --> FCF */
	struct mutex fcfMutex;
	int realTimeState;
	struct work_struct fcfWork;
	int delayTime;
	int isWorkRun;
};

struct vivo_ts_struct {
	int (*getRawOrDiffData)(int which, int *data);
	int (*getGesturePointData)(u16 *data);
	int (*getIcFirmwareOrConfigVersion)(int FwOrConfig);
	int (*getModuleId)(void);
	int (*getLcmId)(void);

	int (*getHeaderFileFWVersionOrConfig)(int FwOrConfig, unsigned char *fw);

	/*ic switch set*/
	int (*setEdgeRestainSwitch)(int on);
	int (*setGloveModeSwitch)(unsigned int gesture_bit_state);
	int (*setGestureSwitch)(unsigned int gestureBitMap);		/*all gesture set to ic by this function*/
	int (*setChargerFlagSwitch)(int state);
	int (*idleEnableOrDisable)(int state);
	int (*readImei)(unsigned char *imei);
	int (*writeImei)(unsigned char *imei);
	int (*processByPackage)(unsigned char *package_name);
	int (*earlySuspendRun)(void);

	int (*getOrSet)(int which);
	int (*otherInfo)(unsigned char *buf);

	/*get info from ic*/
	int (*getRxTxNumFromIc)(void);
	int (*getIcMode)(void);
	/*get tp ts pin value*/
	int (*getTsPinValue)(void);
	/*read or write data to ic*/
	int (*readRegister)(unsigned int addr);
	int (*writeRegister)(unsigned int addr, unsigned int data);

	/*ic reset,power*/
	int (*setIcPower)(int on);
	int (*hardwareReset)(int tmp);	/*reset pin reset*/
	int (*softwareReset)(int tmp);	/*cmd reset*/

	/*ic mode switch*/
	int (*icModeChange)(int which);
	/*ic firmware update*/
	int (*updateFirmware)(const struct firmware *firmware);

	/*sensor test*/
	int (*sensorTest)(char *buf, void *pdata, int tmp);
	int (*atSensorTest)(char *buf, int caliOrCheck, void *pdata, int tmp);
	
	/* vivo finger_gesture  */
	int (*tsInCtl)(int state);
	int (*scanRateChange)(int rate);
	/*for face detect*/
	int (*faceStartEnd)(int state);
	
	/*irq handler*/
	irq_handler_t irqHandler;
	int (*getI2cBusState)(void);
	/*product config info*/
	unsigned int txNum;
	unsigned int rxNum;
	int icNum;
	int moduleId;
	int suspendEventBlank;
	int resumeEventBlank;

#if defined(CONFIG_FB)
	struct notifier_block fbNotif;
	/*this just for mtk old driver framework*/
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct i2c_client *client;
	struct spi_device *spiClient;
	struct input_dev *inputDev;
	struct input_dev *inputDevFp;
	/* input lock */
	struct mutex inputReportMutex;
	/* iic and reset lock */
	struct mutex i2cResetMutex;
	/*sensor test mutex*/
	struct mutex sensorTestMutex;
	struct mutex sensorRawDiffGetMutex;
	/* edge mutex */
	struct mutex edgeMutex;

	struct wakeup_source wakeLock;
	struct kobject kobjectDebug;
	struct kobject kobjectGuest;
	struct kobject *propertiesKobj;	/*for vkey node in sys/board_properties/virtualkeys.(input dev name)*/
	const char *vkeyString;

	void *privateData1;
	void *privateData2;

	atomic_t tsState;
	atomic_t tsCallBackRTState;	/*callback real time states(RESUME SUSPEND)*/
	int hasLcdShutoff;
	int busType;

	int iicErrorNum;
	int largePressNum;
	int itoTestResult;
	/*normal mode*/
	int metalEvent;
	int esdEvent;
	int metalEsdEvent;
	/*guesture mode*/
	int metalGuestureEvent;
	int esdGuestureEvent;
	int metalEsdGuestureEvent;
	
	int logSwitch;
	int isProbeComplete;
	int syncFlag;
	int caliSupport;
	int imeiWriteSupport;

	int usbChargerFlag;
	unsigned int usbChargerTestFlag;
	struct work_struct chargerWork;

	int irqNum;
	void *irqDev;
	struct work_struct irqErrWork;
	struct workqueue_struct *irqErrWorkqueue;
	struct work_struct FaceDetectWork;

    int (*irqEnableOrDisable)(bool en);
	/*firmware updating flag,1 means updating*/
	int fwUpdatingFlag;
	/*fw manager*/
	struct VtsFwManager vtsFwManager;

	int isCalling;
	int isCallingSave;
	int isLargePressMode;
	int is3FingerMode;

	int proximityRtState;

	unsigned int needChangeToGestureMode;
	unsigned int dClickSwitch;
	unsigned int swipeSwitch;
	unsigned int gestureSwitch;
	unsigned int exportSwitch;
	unsigned int udfGestureSwitch;
	unsigned int edgeRestainSwitch;
	unsigned int gloveModeSwitch;

	/* user mode finger print */
	unsigned int fingerPressRecord[VIVO_TS_MAX_TOUCH_NUM][4];
	unsigned char app_name[256];
	unsigned char last_app_name[256];

	/*for incell 4322 sync process between tp and lcm*/
	wait_queue_head_t lcmTpSyncWaitQueue;
	int lcmTpSyncFlag;
	struct mutex lcmResumeAndProximityMutex;	/*infact,this mutex as same as i2cResetMutex,4322 not use i2cResetMutex,this is a Careless*/

	/*vts irq bottom thread */
	int irqBtmType;
	void (*irqBtmThreadHandler)(void *data);

	/*config info*/
	int hasFingerPrint;
	int keyType;
	int isIndependentKeyIc;
	int hasHomeKey;
	int hasMenuKey;
	int hasBackKey;
	struct vivo_key virtualKeyProcessor;
	int mtkOrQcom;	/*qcom or mtk*/
	/* config for finger gesture product */
	int hasFingerGesture;
	int fg2MinTimeout;
	int fgProximityRtState;
	int commonGestureOff;
	atomic_t fingerIcoState;

	/*dt/product config_file info*/
	int tsDimensionX;
	int tsDimensionY;
	int lcdDimensionX;
	int lcdDimensionY;
	int rstGpio;
	int irqGpio;
	int v33Gpio;
	int v18Gpio;

	/* fast change filter */
	struct VtsFcFilter *lcdFcFilter;
	struct VtsFcFilter *proFcFilter;
	int lcdFcFilterSwitch;
	int proFcFilterSwitch;
	int notifyMutexSwitch;
	int suspendProximityChangeTimes;

	/* add for factory apk */
	unsigned char *sensorTestKey;
	unsigned char *lcmNoiseTestKey;
	unsigned char *bspLcmNoiseTestKey;
	unsigned char *rawdataTestKey;
	unsigned char *RFTestKey;

	/*add for lcd RT state*/
	int lcdRtState;
	int callbackType;
	int resume_suspend_type;
	unsigned int statusBit[12];		/* for example statusBit[0] with 32 bits,the num of bits with value 1 mean triger times of this mode */
	unsigned char statisticsCount[12]; /* show the total valid statistics count of each mode */
	unsigned short stateCount[12]; /*index:state  0:water  1:doze 2:noise 3:rescue*/
	int noFlash;
};
extern void vtsIrqBtmThreadWake(void);
extern int vivoTsChipStateReset(void);
extern int vivoTsInputReport(int event, int touchIdOrGestureEvent, int x, int y, int z);
extern int vivoTsCoverMute(u8 pointNum, u8 isLargePressMode);
extern void vivoTsCoverMuteRelease(void);
extern int vivoTsGetState(void);
extern int vivoTsReleasePoints(void);
extern int vivoTsReleasePointsAndKeys(void);
extern void vivoTsSetTxRxNum(int txNum, int rxNum);
extern int vivoTsGetTxRxNum(void);
extern int vivoTsGetChargerFlag(void);
extern struct vivo_ts_struct *vivoTsGetVtsData(void);
extern struct input_dev *vivoTsGetInputDev(void);
extern struct vivo_ts_struct *vivoTsAlloc(void);
extern int vivoTsFree(void);
extern int vivoTsInit(struct i2c_client *client, void *tmpP, int tmp);
extern void vivoTsDeInit(void);
extern int vivoTsAfterProbeCompleteCall(struct i2c_client *client, void *tmpP, int tmp);
extern int vivoTsFwAdd(char dataFlag, const unsigned char *data, int dataSize, unsigned char *productName, int modulePinValue, int moduleVendor, int lcmId);
extern unsigned char *vivoTsGetFw(char dataType, int *fwSize);
extern int vivoTsFwManagerShow(char *buf);
extern int vivoTsGetLcmId(void);
extern int vivoTsInterruptRegister(unsigned int irq, irq_handler_t handler, irq_handler_t thread_fn, unsigned long flags, void *dev);
extern void vivoTsInterruptUnregister(void);
extern u16 *vtsGesturePointsGet(void);
extern int vtsGetGesturePointNum(void);
extern void vtsGesturePointsClean(void);
extern int vtsGesturePointsReport(int event, int index, u16 x, u16 y);
extern int vtsInitProFcFilter(void);
extern void vtsProFcFilter(int state);
extern int vtsInitLcdFcFilter(void);
extern void vtsLcdFcFilter(int state);
extern int vtsFreeProFcFilter(void);
extern int vtsFreeLcdFcFilter(void);
extern int vivoTsInputReport2(int event, int touchIdOrGestureEvent, int x, int y, int z, int x_size, int y_size);

extern int vivoTsGetLcdState(void);//add by wanglanbin
/* #define	VIVO_TOUCH_COLLECT_LOG */
#ifdef	VIVO_TOUCH_COLLECT_LOG
extern int collectTouchException(int exception_type, int reason_num);
#endif
extern void vivoTsInputSync(void);
extern void vtsSetTouchData(int index, int state);
/* extern unsigned int allGestureSwitchIntoBitmap(void); */

#include <linux/vivo_ts_ic_ctl.h>

#endif
