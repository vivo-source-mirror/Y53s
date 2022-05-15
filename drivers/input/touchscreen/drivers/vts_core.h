#ifndef __VTS_CORE_H__
#define __VTS_CORE_H__

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/sensors.h>
#include <linux/mobileevent.h>
#include <linux/pm_wakeup.h>
#include "vts_op.h"
#include "vts_incell.h"
#include "vts_spi_drv.h"
#include <linux/vts_state.h>
#include <linux/vivo_touch_write_datas.h>
#include <linux/moduleparam.h>
#include <asm/div64.h>
#include "vts_log_switch.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <uapi/linux/sched/types.h>
#endif

#define VTS_CODE_VERSION 0x02180710

enum vts_type {
	VTS_TYPE_MAIN,
	VTS_TYPE_SECOND
};

#define	VIVO_TS_MAX_TOUCH_NUM 10

#define	FUN_NOT_REG	500
#define VTS_SENSOR_TEST_CALIBRATION	165
#define VTS_SENSOR_TEST_CHECK		166
#define VTS_PALM_DELAY	300

/* ic number */
/* st */
#define VTS_IC_FTS2		11
#define VTS_IC_FTS3		12
#define VTS_IC_FTM4		13
#define VTS_IC_FTM5		14
#define VTS_IC_ST80Y		15

/* syna */
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
#define VTS_IC_S3908	64
/* ft */
#define VTS_IC_FT5316	31
#define VTS_IC_FT5336	32
#define VTS_IC_FT5446	33
#define VTS_IC_FT8006	34
#define	VTS_IC_FT8736	35
#define	VTS_IC_FT8719	36
#define	VTS_IC_FT8756	37
#define VTS_IC_FT8720   38
#define	VTS_IC_FT8656   39
#define	VTS_IC_FT3518U	40

/* goodix */
#define VTS_IC_GT970	41
#define VTS_IC_GT9159	42
#define VTS_IC_GT1151	43
#define VTS_IC_GT9286	44
#define VTS_IC_GT5688	45
#define VTS_IC_GT9886	46
#define VTS_IC_GT9885TS 47
#define VTS_IC_GT9897   48
#define VTS_IC_GT9897_I2C   49

/* cypress */
#define VTS_IC_CYTMA568	51
#define VTS_IC_CYTMA545	52
#define VTS_IC_CP3155	53
/* nt */
#define VTS_IC_NT36672	81
#define VTS_IC_NT36525	82
#define VTS_IC_NT36670	83
#define VTS_IC_NT36675  84
/* samsung */
#define VTS_IC_SEC_Y661 70
#define VTS_IC_SEC_TDDI 71
#define VTS_IC_SEC_Y761 72
/*ilitek*/
#define VTS_IC_ILI_9881H 90
#define VTS_IC_ILI_9882N 91

/* himax */
#define VTS_IC_HIMAX83113	100
#define VTS_IC_HIMAX83112   101

/*chipone*/
#define VTS_IC_CHIPONE_NL9911C 110


/* module vendor code:MVC*/
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

#ifndef KEY_TS_SWIPE
#define KEY_TS_SWIPE	746
#endif

#define KEY_VIRTUAL_WAKEUP		0x2f2
#define KEY_LONGPRESS_DOWN		0x2f3
#define KEY_LONGPRESS_UP		0x2f4

#define KEY_INSIDE_SLIDE_LEFT   0x2f5
#define KEY_INSIDE_SLIDE_RIGHT  0x2f6
#define KEY_QUIT_ACTIVE_MODE    0x2f7

#define VTS_KEY_FINGER_GESTURE 254
#define VTS_KEY_DOUBLE_FINGER_GESTURE 760
#define VTS_KEY_FACE_GESTURE 605
#define VTS_KEY_V_GESTURE 606
#define VTS_KEY_HEART_GESTURE 607

#define KEY_SCREEN_CLOCK_WAKE_UP    0x279 //633

#define VTS_GESTURE_ARRAY_LEN 40
#define VTS_GESTURE_POINT 0
#define VTS_GESTURE_O_DIR 1

#define VTS_EDGE_AREA_ABS_TYPE BIT(0)
#define VTS_EDGE_AREA_LONG_PRESS_TYPE BIT(1)
#define VTS_EDGE_AREA_CHARACTER_TYPE BIT(2)
#define VTS_ACTIVE_AREA_GESTURE_TYPE BIT(3)

enum vts_event {
	VTS_EVENT_GESTURE_FINGERPRINT_DETECT = 0,
	VTS_EVENT_GESTURE_FINGERPRINT_DOUBLE_DETECT,
	VTS_EVENT_GESTURE_FACE_DETECT,
	VTS_EVENT_GESTURE_LARGE_AREA_PRESS,
	VTS_EVENT_GESTURE_DOUBLE_CLICK,
	VTS_EVENT_GESTURE_TAKE_CAMERA,
	VTS_EVENT_GESTURE_PATTERN_C,
	VTS_EVENT_GESTURE_PATTERN_E,
	VTS_EVENT_GESTURE_PATTERN_M,
	VTS_EVENT_GESTURE_PATTERN_W,
	VTS_EVENT_GESTURE_PATTERN_A,
	VTS_EVENT_GESTURE_PATTERN_F,
	VTS_EVENT_GESTURE_PATTERN_O,
	VTS_EVENT_GESTURE_PATTERN_V,
	VTS_EVENT_GESTURE_PATTERN_HEART,
	VTS_EVENT_GESTURE_PATTERN_LEFT,
	VTS_EVENT_GESTURE_PATTERN_RIGHT,
	VTS_EVENT_GESTURE_PATTERN_UP,
	VTS_EVENT_GESTURE_PATTERN_DOWN,
	VTS_EVENT_GESTURE_PATTERN_SWAP,
	VTS_EVENT_GESTURE_VK_LONG_PRESS,
	VTS_EVENT_GESTURE_VK_LONG_PRESS_RELEASE,
	VTS_EVENT_GESTURE_VK_DC,
	VTS_EVENT_GESTURE_SCREEN_CLOCK_DCLICK,
	VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_LEFT,
	VTS_EVENT_GESTURE_VK_INSIDE_SLIDE_RIGHT,
	VTS_EVENT_GESTURE_VK_QUIT_ACTIVE_MODE,
	VTS_EVENT_GESTURE_PATTERN_H,
	VTS_EVENT_GESTURE_FINGER3_MODE
};

enum vts_frame_type {
	VTS_FRAME_MUTUAL_RAW,
	VTS_FRAME_MUTUAL_DELTA,
	VTS_FRAME_SELF_RAW,
	VTS_FRAME_SELF_DELTA,
	VTS_FRAME_AMBIENT_BASELINE,
	VTS_FRAME_MUTUAL_SELF_RAW,
	VTS_FRAME_MUTUAL_SELF_DELTA
};

enum vts_test_apk_type {
	VTS_TEST_APK_TYPE_SENSOR_TEST,
	VTS_TEST_APK_TYPE_LCM_NOISE_TEST,
	VTS_TEST_APK_TYPE_BSP_LCM_NOISE_TEST,
	VTS_TEST_APK_TYPE_RAWDATA_TEST,
	VTS_TEST_APK_TYPE_RF_TEST,
	VTS_TEST_APK_TYPE_MAX
};

enum vts_property {
	VTS_PROPERTY_PANEL_TYPE,
	VTS_PROPERTY_SENSOR_TX_NUM,
	VTS_PROPERTY_SENSOR_RX_NUM,
	VTS_PROPERTY_DIMENTION_X,
	VTS_PROPERTY_DIMENTION_Y,
	VTS_PROPERTY_DISPLAY_X,
	VTS_PROPERTY_DISPLAY_Y,
	VTS_PROPERTY_RESOLUTION_ADJUST,
	VTS_PROPERTY_IC_NUMBER,
	VTS_PROPERTY_TDDI,
	VTS_PROPERTY_VENDOR,
	VTS_PROPERTY_POLICY,
	VTS_PROPERTY_NO_FLASH,
	VTS_PROPERTY_LONG_PRESS,
	VTS_PROPERTY_VIRTUAL_PROXIMINITY,
	VTS_PROPERTY_NEED_CALI,
	VTS_PROPERTY_GAME_MODE,
	VTS_PROPERTY_BAND,
	VTS_PROPERTY_ROTATION,
	VTS_PROPERTY_CHARGE,
	VTS_PROPERTY_GESTURE,
	VTS_PROPERTY_VIRTUAL_KEY,
	VTS_PROPERTY_LANDSCAPE_GAMEMODE,
	VTS_PROPERTY_REPORT_TIMESTAMP,
	VTS_PROPERYT_VIRTUAL_GAMEKEY,
	VTS_PROPERTY_SCREEN_CLOCK,
	VTS_PROPERTY_BROKEN_DISABLE,
	VTS_PROPERTY_GAME_HIGH_RATE,
	VTS_PROPERTY_GAME_IDLE_TIME,
	VTS_PROPERTY_FINGER_MODE,
	VTS_PROPERTY_FD_FEED_BACK,
	VTS_PROPERTY_INPUT_METHOD,
	VTS_PROPERTY_TP_CHANNEL_COMP,
	VTS_PROPERTY_SET_BUS_STATE,
	VTS_PROPERTY_I2C_EVENT,
	VTS_PROPERTY_BOOST,
	VTS_PROPERTY_FINGER_CENTER,
	VTS_PROPERTY_FINGER_CENTER_X,
	VTS_PROPERTY_FINGER_CENTER_Y,
	VTS_PROPERTY_CALIBRATION_TWICE,
	VTS_PROPERTY_SCREEN_CLOCK_REPORT_ABS,
	VTS_PROPERTY_VIRTUAL_LCMID,
	VTS_PROPERTY_SHARE_LCMID,
	VTS_PROPERTY_DEF_LCMID,
	VTS_PROPERTY_SLEEP_TIME,
	VTS_PROPERTY_EDGE_REJECTION,
	VTS_PROPERTY_SCREEN_CLOCK_HILIGHT,
	VTS_PROPERTY_SCREENSHOT_TRIGER,
	VTS_PROPERTY_HSYNC_COLLECT,
	VTS_PROPERTY_HSYNC_LOWER,
	VTS_PROPERTY_HSYNC_UPPER,
	VTS_PROPERTY_NG_PANEL,
	VTS_PROPERTY_SET_BLANK,
	VTS_PROPERTY_DELAY_AVEE,
	VTS_PROPERTY_AOI,
	VTS_PROPERTY_SET_CARD_REGION,
	VTS_PROPERTY_CARD_REGION_WIDTH,
	VTS_PROPERTY_NO_FLASH_UNIQUE_CODE,
	VTS_PROPERTY_FP_NOTIFIER,
	VTS_PROPERTY_GAMEMODE_FREQ,
	VTS_PROPERTY_REJECT_FAIL_COLLECT,
	VTS_PROPERTY_GAME_SAVE_PWOER_PROC,
	VTS_PROPERTY_RESET_DISCONECT,
	VTS_PROPERTY_TX_FREQ_WRITE,
	VTS_PROPERTY_ESD_CHECK,
	VTS_PROPERTY_MAX
};

enum point_down_or_up {
	POINT_UP = 0,
	POINT_DOWN,
	POINT_RELEASE,
};

struct vts_cmdline_mode {
	const char *mode;
	const char *pre_deli;
	const char *next_deli;
	unsigned int *modeval;
};

struct vts_edge_cmd {
	int x;
	int y;
	int width;
	int height;
	int area_type;
	bool enable;
	int index;
};
struct vts_screen_clock_point{
     struct mutex  scrclMutex;
	 unsigned int  pointX;
	 unsigned int  pointY;
	 unsigned int  realX;
	 unsigned int  realY;
};

struct vts_screen_clock_cmd {
	int x;
	int y;
	int width;
	int height;
};

struct vts_screen_clock_zone{
	 struct vts_screen_clock_point startPoint;
	 int width;
	 int height;
	 int enable;
	 struct mutex c_Mutex;
	};

struct vts_aoi_cmd {
	int aoi_left;
	int aoi_top;
	int aoi_right;
	int aoi_bottom;
};
struct vts_frame {
	enum vts_frame_type type;
	short *data;
	int size;
	struct list_head node;
};

struct vts_device;
struct vts_incell;

struct vts_incell_ops {
	int (*early_suspend)(struct vts_incell *incell, int display_id);
	int (*suspend)(struct vts_incell *incell, int display_id);
	int (*early_resume)(struct vts_incell *incell, int display_id);
	int (*resume)(struct vts_incell *incell, int display_id);
};

enum vts_report_flags_e {
	FLAGS_REPORT_TIMESTAMP = BIT(0),
};

struct vts_report {
	unsigned long slotbit;
	unsigned long touchbit;
	struct mutex lock;
	char km_name[32];
	struct kmem_cache *km;
	struct list_head down_points;
	struct list_head up_points;
	struct list_head keys;
	int nr_coordinates;
	unsigned long flags;
	u16 coordinates_x[32];
	u16 coordinates_y[32];
	bool (*on_event_down)(struct vts_report *report, enum vts_event event, int *keycode);
	bool (*on_event_up)(struct vts_report *report, enum vts_event event, int *keycode);
	bool (*on_point_down)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	bool (*on_point_up)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	int (*on_post_key)(struct vts_report *report,int keycode, int value);
	int (*on_aoi_down)(struct vts_device *vtsdev, int x, int y, int touch_id);
	int (*on_aoi_up)(struct vts_device *vtsdev, int x, int y, int touch_id);
	
};

struct vts_virtual_proxminity {
	struct sensors_classdev cdev;
	struct input_dev *idev;
	struct delayed_work dwork ;
	int values[3];
	int nr_remain_report_times;
	spinlock_t lock;
	bool inited;
};

enum vts_fw_type {
	VTS_FW_TYPE_FW,
	VTS_FW_TYPE_IHEX,
	VTS_FW_TYPE_CONFIG,
	VTS_FW_TYPE_LIMIT,
	VTS_FW_TYPE_MP,
	VTS_FW_TYPE_FACTORY,
	VTS_FW_TYPE_COUNT
};

enum vts_zone {
	VTS_ROM_ZONE_IMEI,
	VTS_ROM_ZONE_LCM
};

struct vts_firmware {
	char path[128];
	struct firmware *firmware;
};

struct vts_grip_area {
	int area_edge;
	int area_center;
};
enum vts_rejection_scene {
	NORMAL_PORTRAIT_SCENE,
	INPUT_METHOD_PORTRAIT_SCENE,
	GAME_PORTRAIT_SCENE,
	NORMAL_LANDSCAPE_SCENE,
	GAME_LANDSCAPE_SCENE,
	VIDEO_LANDSCAPE_SCENE,
	SPECIAL_NORMAL_SCENE,
	SPECIAL_LYING_SCENE,
	EDGE_CONFIG_MAX
};

struct vts_rejection_data {
	int type;	//0 :deadzone  1: longshort_side
	int mode;	//0 :portrait  1:landscape
	int block;	//0 :not corner area 1: corner area (only in portrait)   2:upper_corner  3:bottom_corner
	int x1;		
	int y1;
	int x2;
	int y2;
};

struct vts_rejection_config {
	int scene;
	int config_num;
	int checksum;
	struct list_head list;
	struct vts_rejection_data *data;
};

enum vts_sensor_test_result {
	VTS_SENSOR_TEST_SUCCESS = 0,
	VTS_SENSOR_TEST_DELTA_FAILED = BIT(0),
	VTS_SENSOR_TEST_ITO_FAILED = BIT(1),
	VTS_SENSOR_TEST_DATA_FAILED = BIT(2),
	VTS_SENSOR_TEST_INIT_FAILED = BIT(3),
	VTS_SENSOR_TEST_SHORT_FAILED = BIT(4),
	VTS_SENSOR_TEST_OPEN_FAILED = BIT(5),
	VTS_SENSOR_TEST_NOISE_FAILED = BIT(6),
	VTS_SENSOR_TEST_DOZECC_FAILED = BIT(7),
	VTS_SENSOR_TEST_DIGITAL_FAILED = BIT(8),
	VTS_SENSOR_TEST_SLP_FAILED = BIT(9),
};

enum vts_sensor_cali_result {
	VTS_SENSOR_CALIBERATE_SUCCESS = 0,
	VTS_SENSOR_CALIBERATE_FAILED = BIT(0),
};

struct vts_operations {
	int (*init)(struct vts_device *vtsdev);
	int (*exit)(struct vts_device *vtsdev);
	int (*get_frame)(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size);
	int (*get_fw_version)(struct vts_device *vtsdev, u64 *version);
	int (*get_ic_mode)(struct vts_device *vtsdev);
	int (*set_rotation)(struct vts_device *vtsdev, int on);;
	int (*set_charging)(struct vts_device *vtsdev, int state);
	int (*set_auto_idle)(struct vts_device *vtsdev, int state);
	int (*set_bandstate)(struct vts_device *vtsdev, int state);
	int (*process_package)(struct vts_device *vtsdev, unsigned char *package_name);
	int (*early_suspend)(struct vts_device *vtsdev);
	int (*early_resume)(struct vts_device *vtsdev);
	int (*set_virtual_prox)(struct vts_device *vtsdev, int enable);
	int (*set_long_press)(struct vts_device *vtsdev, int enable);
	int (*set_finger_mode)(struct vts_device *vtsdev, int mode);
	int (*set_gesture)(struct vts_device *vtsdev, int enable);
	int (*otherInfo)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	int (*get_grip_status)(struct vts_device *vtsdev, struct vts_grip_area *grip_area);
	int (*change_mode)(struct vts_device *vtsdev, int which);
	int (*update_firmware)(struct vts_device *vtsdev, const struct firmware *firmware);
	int (*reset)(struct vts_device *vtsdev);
	int (*broken_disable)(struct vts_device *vtsdev, int state);
	int (*sensor_test)(struct vts_device *vtsdev, enum vts_sensor_test_result *result);
	int (*sensor_caliberate)(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result);
	int (*get_ic_status)(struct vts_device *vtsdev, int *status);
	int (*set_edge_reject_area)(struct vts_device *vtsdev,  struct vts_edge_cmd *cmd);
	int (*set_virtual_key)(struct vts_device *vtsdev, int state);
	int (*set_vk_longpress)(struct vts_device *vtsdev, int state);
	int (*set_vk_activemode)(struct vts_device *vtsdev, int state);
	int (*set_landscape_gamemode)(struct vts_device *vtsdev, int state);
	int (*set_virtual_gamekey)(struct vts_device *vtsdev, int state);
	int (*set_aoi_zone)(struct vts_device *vtsdev, int state);
	int (*set_aoi_int_zone)(struct vts_device *vtsdev, int state);
	int (*set_aoi_intpin)(struct vts_device *vtsdev, int state);
	int (*rom_size)(struct vts_device *vtsdev, u32 *size);
	ssize_t (*rom_read)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	ssize_t (*rom_write)(struct vts_device *vtsdev, u8 *buf, size_t nbytes);
	int (*set_screen_clock_report_abs)(struct vts_device *vtsdev,int report_enable);
	int (*set_screen_clock_area)(struct vts_device *vtsdev ,int state);
	int (*update_gesture_bit)(struct vts_device *vtsdev,int state);
	int (*set_input_method)(struct vts_device *vtsdev,int state);
	int (*get_tp_channel_comp_data)(struct vts_device *vtsdev ,bool true_false);
	int (*set_bus_state)(struct vts_device *vtsdev,int state);
	int (*set_finger_center)(struct vts_device *vtsdev,int state);
	int (*get_calibration_status)(struct vts_device *vtsdev);
	int (*get_fw_resolution)(struct vts_device *vtsdev, int enable);
	int (*set_rejection_zone)(struct vts_device *vtsdev, int scene);
	int (*dump)(struct vts_device *vtsdev, int state);
	int (*check_state_sliently)(struct vts_device *vtsdev);
	int (*set_card_region)(struct vts_device *vtsdev, int enable);
	int (*set_game_mode)(struct vts_device *vtsdev, int state);
	int (*set_high_report_rate)(struct vts_device *vtsdev, int rate);
	int (*set_idle_time)(struct vts_device *vtsdev, int times);
	
};

#define VTS_MODULE_LCMID_NR_MAX 32
#define VTS_MODULE_CALI_MAX 5

/* <softid, moduleid> in tp_module */
struct vts_vendor_module_id {
	struct list_head entry;
	u32 module_id;
	u32 softid;
};

struct vts_panel_module {
	struct list_head list;
	u32 properties[VTS_PROPERTY_MAX];
	u32 lcmid[VTS_MODULE_LCMID_NR_MAX];
	u32 softid_cmdline;
	int lcmid_cnt;
	int softid_cnt;
	u32 highest_calibration[VTS_MODULE_CALI_MAX];
	u32 other_calibration[VTS_MODULE_CALI_MAX];
	int highestCali_cnt;
	int otherCali_cnt;
	#if defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	struct drm_panel *active_panel_v2;
	#endif
	bool vts_lcm_module_active;			/* dts 配置vts-lcm-module时使用 */
	struct list_head module_list;
	struct list_head vendor_module_list;		/* dts 配置vts-softid-module_id时保存<softid, module_id> */
	const char *policy_name;
	const char *edge_config[EDGE_CONFIG_MAX];
	char edge_cfg_version[256];
	bool upper_cfg;
};

#ifdef VTS_TEMP_ENABLE
struct vts_fw_collection {
	int lcm_id;
	enum vts_fw_type fw_type;
	const unsigned char *fw_data;
	unsigned int fw_size;
};
#endif

struct vts_policy {
	enum vts_run_mode (*expected_mode)(struct vts_device *vtsdev);
	bool (*modechange_allow)(struct vts_device *vtsdev, enum vts_run_mode old, enum vts_run_mode new, enum vts_state reason);
	bool (*on_event_down)(struct vts_device *vtsdev, enum vts_event event, int *keycode);
	bool (*on_event_up)(struct vts_device *vtsdev, enum vts_event event, int *keycode);
	bool (*on_point_down)(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	bool (*on_point_up)(struct vts_device *vtsdev, int touch_id, int nr_touches, bool large_press, struct vts_point *point);
	void (*on_state_changed)(struct vts_device *vtsdev, enum vts_state state, int val);
	void (*incell_proc)(struct vts_device *vtsdev, unsigned long event, int blank, int display_id);
	int  (*on_post_key)(struct vts_report *report, int keycode ,int value);
	int  (*init)(struct vts_device *vtsdev);
	int (*mode_exit_prepare)(struct vts_device *vtsdev, enum vts_run_mode mode);
	int (*mode_enter_after)(struct vts_device *vtsdev, enum vts_run_mode mode);
	void (*on_dump)(struct vts_device *vtsdev);
	int  (*on_aoi_down)(struct vts_device *vtsdev, int x, int y, int touch_id);
	int  (*on_aoi_up)(struct vts_device *vtsdev, int x, int y, int touch_id);
	enum vts_expect_proc (*get_expect_proc)(struct vts_device *vtsdev, bool mode_change);
	const char *name;
	void *private;
};

enum vts_ic_exception {
	VTS_EXCEPTION_ESD_ERR,
	VTS_EXCEPTION_WATCHDOG_ERR,
	VTS_EXCEPTION_CHECKSUM_ERR,
	VTS_EXCEPTION_I2C_ERR,
	VTS_EXCEPTION_COUNT,
};

#define VTS_MAX_CPU_NR 2
struct vts_sched {
	struct task_struct *idle[VTS_MAX_CPU_NR];
	struct semaphore sem0;
	struct semaphore sem1;
	int max_cpu_num;
	int idle_num;
	unsigned long flags;
	struct hrtimer timer;
	struct work_struct ddr_work;
	struct delayed_work ddr_work_rem;
	struct workqueue_struct *wq;
	int ddr_timeout;
	int lpm_timeout;
	struct wakeup_source *idle_wakelock0;
	struct wakeup_source *idle_wakelock1;
	char idle_wakelock_name0[32];
	char idle_wakelock_name1[32];
};

enum vts_boost_state {
	VTS_BOOST_ENABLE = 0,
	VTS_BOOST_DISABLE
};

struct vts_long_press_report {
	/*long press report timer*/
	struct hrtimer long_press_timer;
	struct work_struct long_press_work;
	int slot;
	int major;
	int index;
	int work_status;
};

#define vts_event(ev, val) do {} while(0)
#define vts_event2(ev, val1, val2) do {} while(0)
#define vts_event3(ev, val1, val2, val3) do {} while(0)

typedef irqreturn_t (*vts_irq_handler_t)(int, void *, ktime_t);

struct vts_aoi_info {
	int aoi_area_in;
	int aoi_area_out;
	int aoi_area_width;
	int aoi_area_height;
	int aoi_avg_signal;
};

int vts_notifier_chain_register(struct notifier_block * nb);
int vts_notifier_chain_unregister(struct notifier_block * nb);
void vts_notifier_call_chain(unsigned long action, void *data);

#define EVENT_TYPE_DOWN    0
#define EVENT_TYPE_UP      1
#define EVENT_TYPE_MOVE    2
#define RESERVED_LENGTH 8
struct vts_touch_event {
	int panel;    /* 0-main  1-second */
	int fid;       /* Finger ID */
	int type;      /* 0-down 1-move 2-up */
 	int x;
 	int y;
	int inside;
	int outside;
	int major_axis;
	int minor_axis;
	int pressure;
	int signal;
	int reserved[RESERVED_LENGTH];
};



struct vts_device {
	struct device *dev;
	struct list_head entry;
	struct list_head tp_list;
	struct input_dev *idev;
	struct input_dev *idev_fp;
	const struct vts_operations *ops;
	void *private;
	struct list_head frames;

	char msg_km_name[32];
	struct kmem_cache *msg_km;
	struct list_head messages;
	spinlock_t msg_lock;
	struct semaphore msg_sem;

	struct task_struct *t;
	struct semaphore sem;

	int irq;
	void *irqdata;
	unsigned long flags;
	vts_irq_handler_t irq_thread_fn;

	struct wakeup_source *irq_wakelock;
	struct wakeup_source *k_wakelock;

	int (*exception_handler)(struct vts_device *vtsdev, enum vts_ic_exception e, int arg);

	struct vts_firmware firmwares[VTS_FW_TYPE_COUNT];
	atomic_t firmware_cache;
	struct vts_panel_module *module;
	u32 ic_number;
	enum vts_type type;
	struct vts_policy *policy;

	atomic_t run_mode;
	spinlock_t mslock;
	atomic_t mobile_state[VTS_STA_MAX];
	unsigned char busType;

	const char *activity_path[VTS_TEST_APK_TYPE_MAX];
	
	struct vts_incell incell;
	struct vts_report report;
	unsigned long large_touches;
	bool finger3_mode;
	struct vts_virtual_proxminity virtual_prox;
	struct vts_screen_clock_zone screen_clock_zone;
	struct vts_screen_clock_point  screen_clock_point;
	unsigned int has_screen_clock;
	atomic_t idle_state;
	struct vts_long_press_report long_press_report;

	u8 *rom_data;
	u32 rom_size;
	u32 exceptions[VTS_EXCEPTION_COUNT];
#if defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	struct drm_panel *active_panel_v2;
#endif
#ifdef VTS_TEMP_ENABLE
	struct delayed_work fw_update_work;
#endif
	atomic_t boost_state;
	struct vts_sched sched;
	char wakelock_name[32];
	struct workqueue_struct *gpoint_wq;
	struct work_struct gpoint_work;

	int fw_x;
	int fw_y;

	struct vts_callback_handler vts_dbg_catch_handler;
	atomic_t special_lying_state;
	u8 default_cfg[EDGE_CONFIG_MAX];

	/****data collect begin****/
	int t_to_k;
	int resume_end;
	int state_sync_time;
	int mode_change_time;
	int fw_update_time;
	int irq_wakelock_time;
	int lcd_wakelock_time;
	int total_handler_time;
	int muti_touch_time[10];
	int nr_touches;
	bool int_cost_collect_flag;
	int point_x[10];
	/****data collect end****/

	/*fingerprint begin*/
	struct vts_aoi_cmd aoi_cmd;
	bool aoi_notify_enabled;
	bool aoi_report_enabled;
	bool aoi_int_enabled;
	unsigned long event_mask;
	unsigned long finger_pressed;
	struct vts_touch_event event[10];
	struct vts_aoi_info aoi_info[10];
	int aoi_intpin;
	int aoi_report_type;
	int aoi_real_point[2];  //x,y
	int aoi_area_point[2];  //x,y
	/*fingerprint end*/

	/*card slide region begin*/
	unsigned int y0;
	unsigned int y1;
	unsigned int width;
	/*card slide region end*/

	struct vts_rejection_config *rejection_config;
	int config_length;
};

static inline void *vts_get_drvdata(struct vts_device *vtsdev)
{
	return vtsdev->private;
}

static inline void vts_set_drvdata(struct vts_device *vtsdev, void *data)
{
	vtsdev->private = data;
}

static inline const char *vts_name(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vivo_ts";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vivo_ts_second";
	else
		return "invalid type";
}

static inline const char *touscreen_name(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "touchscreen";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "touchscreen_second";
	else
		return "invalid type";
}


static inline const char *vts_name_fp(struct vts_device *vtsdev)
{
	if (vtsdev->type == VTS_TYPE_MAIN)
		return "vivo_ts_fp";
	else if (vtsdev->type == VTS_TYPE_SECOND)
		return "vivo_ts_second_fp";
	else
		return "invalid type";
}

#define incell_to_vts(x) container_of(x, struct vts_device, incell)

/* gesture switch bit mask, sync this bit with touchscreen service*/
#define VTS_GESTURE_C		(0x01 << 6)
#define VTS_GESTURE_E		(0x01 << 5)
#define VTS_GESTURE_UP		(0x01 << 1)
#define VTS_GESTURE_DOWN	(0x01 << 7)
#define VTS_GESTURE_M		(0x01 << 4)
#define VTS_GESTURE_LR		(0x01 << 0)
#define VTS_GESTURE_W		(0x01 << 3)
#define VTS_GESTURE_DCLICK	(0x01 << 8)
#define VTS_GESTURE_F		(0x01 << 10)
#define VTS_GESTURE_A		(0x01 << 9)
#define VTS_GESTURE_FG		(0x01 << 12)
#define VTS_GESTURE_O		(0x01 << 2)
#define VTS_GESTURE_SHORT_DOWN_SWIPE	(0x01 << 11)
#define VTS_GESTURE_FACE_GESTURE		(0x01 << 13)
#define VTS_GESTURE_V		(0x01 << 14)
#define VTS_GESTURE_HEART	(0x01 << 15)
#define VTS_GESTURE_COVER_MUTE			(0x01 << 16)

#define VTS_GESTURE_ENABLE(bitmap, gesture) 	((unsigned int)(bitmap | (unsigned int)gesture))
#define VTS_GESTURE_DISABLE(bitmap, gesture) 	((unsigned int)(bitmap & (~(unsigned int)gesture))

enum vts_run_mode {
	VTS_ST_NORMAL = 0,
	VTS_ST_SLEEP,
	VTS_ST_GESTURE,
	VTS_ST_UNKNOWN
};

enum vts_expect_proc{
	VTS_NULL_PROC = 0,
	VTS_TRADITIONAL_NORMAL_PROC,
	VTS_TRADITIONAL_GAME_PROC,
	VTS_HIGH_RATE_10S_IDLE_PROC,
	VTS_NORMAL_RATE_2S_IDLE_PROC,
	VTS_NORMAL_RATE_1S_IDLE_PROC	
};

enum vts_report_rate_proc{
	RATE_NORMAL = 0,
	RATE_HIGH
};
enum vts_idle_times_proc{
	IDLE_TIME_1S = 1,
	IDLE_TIME_2S, 
	IDLE_TIME_10S
};

enum vts_game_proc_types{
	PROC_OUT_GAME = 0,
	PROC_IN_GAME
};


#define vts_dev_log(vtsdev, force, flag, fmt, param...) do { \
		if(likely(force)) \
			printk(KERN_ERR "VIVO_TS_"flag"[%s]%s:%d:"fmt, vtsdev ? vts_name(vtsdev) : "common", __func__, __LINE__, ##param); \
	} while (0)
#define vts_dev_err(vtsdev, fmt, param...) do { \
		vts_dev_log(vtsdev, 1, "ERR", fmt, ##param); \
	} while(0)
#define vts_debug_code() if(vts_is_debug())

#define vts_dev_info(vtsdev, fmt, param...) vts_dev_log(vtsdev, 1, "INF", fmt, ##param)
#define vts_dev_dbg(vtsdev, fmt, param...) vts_dev_log(vtsdev, vts_is_debug(), "DBG", fmt, ##param)

#define VTI(fmt, param...) vts_dev_info(NULL, fmt, ##param)
#define VTE(fmt, param...) vts_dev_err(NULL, fmt, ##param)
#define VTD(fmt, param...) vts_dev_dbg(NULL, fmt, ##param)

#define VTS_GESTURE_ARRAY_LEN 40
#define VTS_GESTURE_POINT 0
#define VTS_GESTURE_O_DIR 1

extern struct vts_device *vts_device_alloc(void);
extern int vts_device_free(struct vts_device *vtsdev);
extern int vts_parse_dt_property(struct vts_device *vtsdev, struct device_node *np);
extern int vts_register_driver(struct vts_device *vtsdev);
extern int vts_unregister_driver(struct vts_device *vtsdev);
extern void vts_device_lock(struct vts_device *vtsdev);
extern int vts_device_lock_timeout(struct vts_device *vtsdev, long jiffies);
extern void vts_device_unlock(struct vts_device *vtsdev);
extern u8 *vts_fw_data_get(struct vts_device *vtsdev, enum vts_fw_type type, int *size);
extern void vts_fw_data_put(struct vts_device *vtsdev,  enum vts_fw_type type);
extern int vts_fw_path_set(struct vts_device *vtsdev, enum vts_fw_type type, const char *path, bool update);
extern int vts_fw_path_get(struct vts_device *vtsdev, enum vts_fw_type type, char *path, size_t size);
extern struct vts_frame *vts_frame_data_get(struct vts_device *vtsdev, enum vts_frame_type type);
extern int vts_frame_data_put(struct vts_device *vtsdev, struct vts_frame *frame);
extern int vts_get_lcmid(struct vts_device *vtsdev, u32 *lcmid);
extern int vts_get_lcmid_compatible(struct vts_device *vtsdev, u32 *lcmid, size_t *size);
int vts_get_module_id_compatible(struct vts_device *vtsdev, u32 *module_id, size_t *size);
extern int vts_property_get(struct vts_device *vtsdev, enum vts_property prop, u32 *val);
extern int vts_state_set(struct vts_device *vtsdev, enum vts_state id, int val);
extern int vts_state_set_sync(struct vts_device *vtsdev, enum vts_state id, int val);
extern int vts_state_broadcast(struct vts_device *vtsdev, enum vts_state id, int val);
extern bool vts_state_all_equals(enum vts_state state, int val);
extern enum vts_run_mode vts_get_run_mode(struct vts_device *vtsdev);
extern int vts_state_get(struct vts_device *vtsdev, enum vts_state state);
extern void vts_state_dump(struct vts_device *vtsdev);
int vts_notify_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg);
extern bool vts_is_module_match(void);
extern bool vts_is_debug(void);
extern void vts_debug_enable(bool enable);
extern int vts_interrupt_register(struct vts_device *vtsdev, unsigned int irq, vts_irq_handler_t irq_thread_fn, unsigned long flags, void *data);
extern void vts_interrupt_unregister(struct vts_device *vtsdev);
extern int vts_incell_init(struct vts_device *vtsdev);
extern int vts_incell_deinit(struct vts_device *vtsdev);
extern int vts_policy_init(struct vts_device *vtsdev);
extern void vts_policy_deinit(struct vts_device *vtsdev);
extern int vts_report_coordinates_set(struct vts_device *vtsdev, u16 *x, u16 *y, int nr_points);
extern int vts_report_coordinates_get(struct vts_device *vtsdev, u16 *x, u16 *y, size_t size, int *nr_points);
extern int vts_report_point_down(struct vts_device *vtsdev, int touch_id, int nr_touches,
	int x, int y, int wx, int wy, bool large_press, u8 *data, size_t size, ktime_t kt);
extern int vts_report_point_up(struct vts_device *vtsdev, int touch_id, int nr_touches, int x, int y, int wx, int wy, bool large_press, ktime_t kt);
extern int vts_report_release(struct vts_device *vtsdev);
extern int vts_report_release_aoi_point(struct vts_device *vtsdev);
extern int vts_report_set_flags(struct vts_device *vtsdev, unsigned long flags);
extern int vts_report_inject_points(struct vts_device *vtsdev, char *filepath);
extern unsigned long vts_report_flags(struct vts_device *vtsdev);
extern int vts_report_event_down(struct vts_device *vtsdev, enum vts_event event);
extern int vts_report_event_up(struct vts_device *vtsdev, enum vts_event event);
extern int vts_event_trigger(struct vts_device *vtsdev, enum vts_event event);
extern int vts_report_point_sync(struct vts_device *vtsdev);
extern int vts_report_ic_status(struct vts_device *vtsdev, int status);
extern int vts_report_ic_exception(struct vts_device *vtsdev, enum vts_ic_exception exception, int arg);
extern int vts_proxminity_init(struct vts_device *vtsdev);
extern void vts_proxminity_deinit(struct vts_device *vtsdev);
extern void vts_proxminity_report(struct vts_device *vtsdev, int x, int y, int z);
extern int vts_firmware_version_get(struct vts_device *vtsdev, u64 *version);
extern int vts_touch_ic_mode_get(struct vts_device *vtsdev);
extern int vts_get_calibration_status(struct vts_device *vtsdev);
extern int vts_classdev_register(struct device *parent, struct vts_device *vtsdev);
extern void vts_classdev_unregister(struct vts_device *vtsdev);
extern int vts_update_dclick_point(struct vts_device *vtsdev, int x, int y);
extern unsigned int vts_get_msdd_report_lcm_id(enum vts_type type);
extern struct wakeup_source *vts_wakelock_register(struct vts_device *vtsdev, const char *wakelock_name);
extern void vts_wakelock_unregister(struct wakeup_source *wakelock);
extern void vts_long_press_timer_init(struct vts_device *vtsdev);
extern void vts_long_press_timer_deinit(struct vts_device *vtsdev);
extern int vts_dump(struct vts_device *vtsdev);
extern int vts_check_status_sliently(struct vts_device *vtsdev);
extern int vts_channel_broken_collect(int type, u8 Txnum, u8 Rxnum ,u16 Txbit, long long Rxbit);
extern int vts_firmwware_version_collect(int type, int lcmid, u64 fwversion);
extern int vts_virtual_prox_enable_collect(int type, unsigned int enable);
extern int vts_communication_abnormal_collect(int type);
extern int vts_click_frequency_collect(int type);
extern int vts_ng_panel_collect(int type);
extern int vts_hsync_state_collect(int type);
extern int vts_abnormal_reset_collect(int type);
extern int vts_ic_mode_collect(int type, int mode, int mode_in);
extern int vts_int_cost_time(int type, int t_to_k, int int_bottom);
extern int vts_large_press_event(int type);
extern int vts_multi_touch(int type, int finger_type, int finger_cost_time);
extern int vts_press_area(int type, int area_type);
extern int vts_sensor_test_collect(int type, int pass, int fail);
extern int vts_edge_reject_fail(int type);
extern int vts_resume_to_touch(int type, int min_time);
extern int vts_handler_cost(int type, enum touch_vcode_lock_type lock_type, int lock_time);
extern int vts_five_more_time(int type, int nr_touchs);
extern void nr_touchs_changed(struct vts_device *vtsdev, enum point_down_or_up report_touch, int nr_touches);


int vts_report_init(struct vts_device *vtsdev,
bool (*on_event_down)(struct vts_report *report, enum vts_event event, int *keycode),
bool (*on_event_up)(struct vts_report *report, enum vts_event event, int *keycode),
bool (*on_point_down)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point),
bool (*on_point_up)(struct vts_report *report, int touch_id, int nr_touches, bool large_press, struct vts_point *point),
int  (*on_post_key)(struct vts_report *report, int keycode ,int value),
int  (*on_aoi_down)(struct vts_device *vtsdev, int x, int y, int touch_id),
int  (*on_aoi_up)(struct vts_device *vtsdev, int x, int y, int touch_id));
extern int vts_report_deinit(struct vts_device *vtsdev);
extern int vts_node_sysfs_add(struct vts_device *vtsdev);
extern void vts_node_sysfs_remove(struct vts_device *vtsdev);
extern ssize_t vts_rom_zone_read(struct vts_device *vtsdev, enum vts_zone zone, u8 *buf, size_t size);
extern ssize_t vts_rom_zone_write(struct vts_device *vtsdev, enum vts_zone zone, const u8 *buf, size_t size);
extern ssize_t vts_rom_zone_size(struct vts_device *vtsdev, enum vts_zone zone);
extern bool vts_is_load_driver(char *driver_name, const int *ic_numbers, size_t size);
extern int vts_reset(struct vts_device *vtsdev);
extern int vts_device_count_get(void);
int vts_sched_set_flags(struct vts_device *vtsdev, unsigned long flags);
unsigned long vts_sched_flags(struct vts_device *vtsdev);
int vts_sched_set_lpm_timeout(struct vts_device *vtsdev, unsigned long timeout);
unsigned long vts_sched_lpm_timeout(struct vts_device *vtsdev);
int vts_sched_set_ddr_timeout(struct vts_device *vtsdev, unsigned long timeout);
unsigned long vts_sched_ddr_timeout(struct vts_device *vtsdev);
extern int vts_get_screen_clock_zone(struct vts_screen_clock_cmd *dis, struct vts_screen_clock_zone *src);
extern unsigned int vts_factory_mode_get(void);
extern struct vts_rejection_config* vts_rejection_zone_config_get(struct vts_device *vtsdev, int scene);
extern int vts_rejection_zone_cmd_set(struct vts_device *vtsdev, char **tmp);
extern int vts_rejection_zone_config_get_from_dts(struct vts_device * vtsdev, char ** tmp);
extern int vts_rejection_version_parse(struct vts_device *vtsdev, char **tmp);

#if IS_ENABLED(CONFIG_INPUT_BOOST)
int vts_boost_init(struct vts_device *vtsdev);
void vts_boost_deinit(struct vts_device *vtsdev);
void vts_boost_enable(struct vts_device *vtsdev, int nr_touches);
void vts_boost(struct vts_device *vtsdev);
#else
static inline int vts_boost_init(struct vts_device *vtsdev)
{
	return 0;
}
static inline void vts_boost_deinit(struct vts_device *vtsdev) {}
static inline void vts_boost_enable(struct vts_device *vtsdev, int nr_touches) {}
static inline void vts_boost(struct vts_device *vtsdev) {}
#endif
int vts_down_interruptible(struct semaphore *sem);
void vts_up(struct semaphore *sem);
void vts_kthread_stop(struct semaphore *sem, struct task_struct *task);
bool vts_dev_match_module_ic_number(struct device_node * node);


#define vts_call_func_sync(vts, return_type, func, arg...) ({ \
		return_type ret; \
		vts_device_lock(vts); \
		ret = func(vts, ##arg); \
		vts_device_unlock(vts); \
		ret; \
	})

#define __vts_call_ic_ops(vts, func, sync, arg...) ({ \
		int ret = 0; \
		s64 ktime_start; \
		s64 ktime_end; \
		u64 ktime_diff; \
		u64 ktime_mod; \
		\
		if (vts && vts->ops && vts->ops->func) { \
			if (sync) \
				vts_device_lock(vts); \
			vts_dev_info(vts, "call ic ops "#func" start\n"); \
			ktime_start = ktime_to_us(ktime_get()); \
			ret = vts->ops->func(vts, ##arg); \
			ktime_end = ktime_to_us(ktime_get()); \
			ktime_diff = (u64)(ktime_end - ktime_start); \
			ktime_mod = do_div(ktime_diff, 1000); \
			vts_dev_info(vts, "call ic ops "#func" end, took %lld.%lldms, ret = %d\n", ktime_diff, ktime_mod, ret); \
			if (sync) \
				vts_device_unlock(vts); \
		} else { \
			vts_dev_info(vts, "ic didn't implement "#func", no need call\n"); \
		} \
		\
		ret; \
	})

#define vts_call_ic_ops(vts, func, arg...) __vts_call_ic_ops(vts, func, false, ##arg)
#define vts_call_ic_ops_sync(vts, func, arg...) __vts_call_ic_ops(vts, func, true, ##arg)

#define module_vts_driver(name, ic_numbers, init, exit) \
	int vts_driver_##name##_init(int *load) \
	{ \
		bool ret; \
		ret = vts_is_load_driver(#name, ic_numbers, ARRAY_SIZE(ic_numbers)); \
		*load = ret; \
		if (!ret) \
			return -ENODEV; \
	\
		return init; \
	} \
	void vts_driver_##name##_exit(void) \
	{ \
		exit; \
	} \

#endif
