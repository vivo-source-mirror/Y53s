#ifndef __FUEL_SUMMARY_H
#define __FUEL_SUMMARY_H


#define QCOM_PLATFORM			0


#if QCOM_PLATFORM
#include <linux/power_supply.h>
#else
#include <linux/power/vivo/power_supply_lite.h>
#endif


/************************************************************
 *
 *  [Fuelsummary exception code]
 *
 ***********************************************************/
#define EX_HEAD_LIMIT_BIT		24
#define EX_WARNING			(0x1<<EX_HEAD_LIMIT_BIT)
typedef enum {
	WARNING_CHG_OV = 0,
	WARNING_BAT_OT,                 /* 电池过温 */
	WARNING_CHG_OC,
	WARNING_BAT_OV,		        /* 电池过压 */
	WARNING_CHG_TO,		        /* 充电超时 */
	WARNING_BAT_LT,		        /* 电池低温 */
	WARNING_BID_MS,		        /* 电池丢失 */
	WARNING_BAT_TEMP_OFFSET,
	WARNING_BAT_NORIGIN,		/* 电池非标 */
	WARNING_BAT_UNPLUGED,	        /* 电池拔出 */
	WARNING_CHG_EX,
	WARNING_BAT_HTCCC,		/* 基于安全 */
	WARNING_USB_OT,		        /* USB过温 */
	WARNING_USB_MT,		        /* USB潮湿 */
	WARNING_ADP_OT,		        /* 充电器过温 */
	WARNING_BBOARD_OT,		/* 电池板过温 */
	WARNING_BCONN_OT,		/* 电池接口过温 */
	WARNING_USB_2GEAR,		/* USB二挡过温 */
	WARNING_BBOARD_2GEAR,	        /* 电池板二挡过温 */
	WARNING_BCONN_2GEAR,		/* 电池接口二挡过温 */
	WARNING_ADPCONN_OT,		/* 充电器接口过温 */
	WARNING_DIE_OT,			/* DIE过温 */
	WARNING_LCM_OT,		        /* LCM过温 */
} EX_WARNING_ENUM;

#define EX_SOC				(0x2<<EX_HEAD_LIMIT_BIT)
typedef enum {
#define DEVIATE_INTERVAL		20
	SOC_DEVIATE = 0,                /* 电量偏差 */
	SOC_NEVER_FULL,		        /* 充不满 */
#define COEF_1P			        4
	SOC_CLIMB,			/* 电量上升慢 */
	SOC_HANG,			/* 电量下降慢 */
	SOC_HOP,			/* 电量跳变 */
	SOC_DROP,			/* 电量下降快 */
	SOC_FAST,			/* 电量生命持续10s以内 */
	SOC_FIRST_HOP,		        /* 开机电量跳变 */
	SOC_CYCLE,		        /* 电池放电循环次数 */
	SOC_VCELL_BALANCE,		/* 双电芯平衡 */
} EX_SOC_ENUM;

#define EX_CHG				(0x3<<EX_HEAD_LIMIT_BIT)
typedef enum {
#define READY_TIME			90
	CHG_RERUN_ERR = 0,		/* 类型出错 */
	CHG_WEAK,			/* 功率弱载 */
#define HOT_INTERVAL			1800
	CHG_HOT_PLUG,		        /* 快速充停 */
#define VALID_TIME			300
#define STIME				14400
#define ISLOW				-800
#define SNCHG_P				75
#define SFCHG_P				65
	CHG_SLOW,			/* 充电慢 */
#define TYPE_USB			POWER_SUPPLY_TYPE_USB
#define TYPE_DCP			POWER_SUPPLY_TYPE_USB_DCP
#define TYPE_HVDCP			POWER_SUPPLY_TYPE_USB_HVDCP
	CHG_NSLOW,		        /* 标配充电慢 */
	CHG_VFCHG_EX,		        /* vivo快充退出异常 */
	CHG_BQIC_EX,		        /* vivo快充IC异常 */
	CHG_OUT,		        /* vivo快充拔出 */
} EX_CHG_ENUM;

#define EX_CHGIC			(0x4<<EX_HEAD_LIMIT_BIT)
typedef enum {
	CHGIC_I2C_VENDOR = 0,
	CHGIC_M_I2C_ERR,		/* I2C出错 */
	CHGIC_S_I2C_ERR,
	CHGIC_M_DPM,		        /* 触发DPM */
	CHGIC_S_DPM,
	CHGIC_M_FAULT,		        /* FAULT状态 */
	CHGIC_S_FAULT,
	CHGIC_M_TERM_ERR,		/* 截至状态异常 */
	CHGIC_S_TERM_ERR,
	CHGIC_M_STAT_ERR,		/* 充电状态异常 */
	CHGIC_S_STAT_ERR,
	CHGIC_BOOST_INVERT_ERR,		/* 充电倒灌异常 */
	CHGIC_SAFE_REG_ERR,		/* 安全寄存器异常 */
	CHGIC_ICO_ERR,		        /* ICO失效 */
} EX_CHGIC_ENUM;

#define EX_FG				(0x5<<EX_HEAD_LIMIT_BIT)
typedef enum {
	FG_I2C_ERR = 0,			/* I2C出错 */
	FG_I2C_INIT_ERR,
	FG_FFC_OVP_ERR,
	FG_CHECKSUM_ERR,
	FG_FFC_COUT_ERR,
	FG_FW_UPDATE,
	FG_VENDOR,
	FG_RESET,
	FG_LOADER_SW,
#define FG_DUMP_INTERVAL		300
	FG_DUMP,
	FG_DISCHG_RST,
	FG_CHG_RST,
	FG_BALANCE,
} EX_FG_ENUM;

#define EX_OTHER			(0x6<<EX_HEAD_LIMIT_BIT)
typedef enum {
	OTHER_RESERVED = 0,
	OTG_USING,
	LCM_MOS_GPIO_ERR,
	BCL_VADC_ERR,
	VBUS_SMPL_DETECT,
	CC_ERR,
	VBUS_PULL,
	BALANCING_CHG,
} EX_OTHER_ENUM;


/* ***********************************************************
 *
 *  [Fuelsummary exception structure & function]
 *
 ************************************************************/
typedef enum {
	CHGIC_UNKNOWN = 0,
	CHGIC_BQ24157,
	CHGIC_BQ24192,
	CHGIC_BQ24296,
	CHGIC_BQ25890,
	CHGIC_BQ25892,
	CHGIC_BQ25601D,
	CHGIC_BQ25890H,
	CHGIC_BQ25910,
	CHGIC_BQ25970,
	CHGIC_RT9467,
	CHGIC_RT9466,
	CHGIC_PCA9486,
	CHGIC_BQ25980,
	CHGIC_MT6360,
	CHGIC_BQ25970_DUAL,
	PM8916 = 20,
	PM8917,
	PM8926,
	PMI8952,
	PM8939,
	PMI8994,
	PMI632,
	PM660,
	PM670,
	PMI8998,
	PM6150,
	PM7150,
	PM8150,
} CHGIC_VENDOR_ENUM;
typedef enum {
	FG_STATE_IIC_ERR = 1,
	FG_STATE_CHECKSUM_ERR,
	FG_STATE_FFC_OVP_ERR,
} FG_STATE_ENUM;
typedef enum {
	COUL_NOT = 0,
	COUL_NORMAL,
	COUL_NEGATIVE,
} COUL_ENUM;
typedef enum {
	SCHEME_VIVO_FASTCHG = 0,
	SCHEME_QCOM_DUALCHG,
	SCHEME_MTK_DUALCHG,
	SCHEME_QCOM_NORMCHG,
	SCHEME_MTK_NORMCHG,
} CHARGINE_SCHEME_ENUM;
typedef enum {
	SCENE_INIT = 0,
	SCENE_10STHD,
	SCENE_CHG,
	SCENE_USBIN,
	SCENE_USBOUT,
	SCENE_SUSPEND,
	SCENE_RESUME,
	SCENE_SOC_CHANGE,
} SCENE_ENUM;
struct fuelsummary_ex {
	unsigned long code;
	unsigned long ex_code;
	unsigned long scenes;
	bool valid;
	void *private;
	void (*analyze)(struct fuelsummary_ex *fex, SCENE_ENUM scene);
	struct mutex fmutex;
	struct list_head list;
};
typedef void (*FEX_ANALYZE)(struct fuelsummary_ex *fex, SCENE_ENUM scene);
extern void set_exbit(unsigned long *ex_code, int bit, bool status);
extern struct fuelsummary_ex *fex_create(unsigned long ex_enum, void *owned,
		FEX_ANALYZE func);


/* ***********************************************************
 *
 *  [Fuelsummary exception info]
 *
 ************************************************************/
struct mark_int {
	bool state;
	int true_cnt;
	int false_cnt;
};

struct both_int {
	bool start;
	int start_val;
	int cur_val;
};

struct comb_int {
	int num;
	int sum;
	int cur_val;
	int avg_val;
	int max_val;
	int min_val;
};

typedef enum {
	ID_BIT__TERM = 0,
	ID_BIT__FCHG,
	ID_BIT__CALLING,
	ID_BIT__BALANCE_CHG,
	ID_BIT__FACTORY,
	ID_BIT__BSPTEST,
	ID_BIT__EXHIBITION,
	ID_BIT__APSD_ERR,
	ID_BIT__PD_9V,
	ID_BIT__VBUS_SMPL,
	ID_BIT__USB_WATER,
	ID_BIT__CC_ERR,
	ID_BIT__VBUS_PULL,
	ID_BIT__VBCL_ERR,
	ID_BIT__M_I2C_ERR,
	ID_BIT__M_DPM,
	ID_BIT__M_FAULT,
	ID_BIT__M_BUS_OCP,
	ID_BIT__M_BUS_RCP,
	ID_BIT__M_BUS_SCP,
	ID_BIT__M_CFLY_SHORT,
	ID_BIT__M_CONV_OCP,
	ID_BIT__S_I2C_ERR,
	ID_BIT__S_DPM,
	ID_BIT__S_FAULT,
	ID_BIT__S_BUS_OCP,
	ID_BIT__S_BUS_RCP,
	ID_BIT__S_BUS_SCP,
	ID_BIT__S_CFLY_SHORT,
	ID_BIT__S_CONV_OCP,
	ID_BIT__MAX,

	ID_BYTE__BATID,
	ID_BYTE__MIC_VENDOR,
	ID_BYTE__SIC_VENDOR,
	ID_BYTE__INPUT_SCALE,
	ID_BYTE__INBAT_SCALE,
	ID_BYTE__CHGT,
	ID_BYTE__CAB_ID,
	ID_BYTE__SAFE_REG,
	ID_BYTE__CAM_VDROP,
	ID_BYTE__IRQA,
	ID_BYTE__IRQB,
	ID_BYTE__IRQC,
	ID_BYTE__IRQD,
	ID_BYTE__IRQ4,
	ID_BYTE__IRQ5,
	ID_BYTE__FC_BL_REV,
	ID_BYTE__FC_APP_REV,
	ID_BYTE__FC_REV_M,
	ID_BYTE__FC_REV_S,
	ID_BYTE__FC_BYPASS,
	ID_BYTE__FC_IMBALANCE,
	ID_BYTE__FC_BOTH_FAULT,
	ID_BYTE__FC_SLAVE_FAULT,
	ID_BYTE__ADPT_STAT,
	ID_BYTE__ADPT_POWER,
	ID_BYTE__ADPT_MCU_VER,
	ID_BYTE__ADPT_VENDOR,
	ID_BYTE__FG_STATE,
	ID_BYTE__FG_EX_STEP,
	ID_BYTE__FG_BAT_FACTORY,
	ID_BYTE__FG_DEV_NUM,
	ID_BYTE__FG_VER,
	ID_BYTE__FG_FW_VER,
	ID_BYTE__FG_CLK_STAT,
	ID_BYTE__FG_SDA_STAT,
	ID_BYTE__FG_UPDATE_STAT,
	ID_BYTE__FG_ERR_CODE,
	ID_BYTE__FG_DELTA_DIFF,
	ID_BYTE__MAX,

	ID_INT__CYCLE,
	ID_INT__FCC,
	ID_INT__SOH,
	ID_INT__ICO,
	ID_INT__STAY_SEC,
	ID_INT__PLUG_SEC,
	ID_INT__CHG_TIME,
	ID_INT__CHG_TCOST,
	ID_INT__FG_VCELL1,
	ID_INT__FG_VCELL2,
	ID_INT__FG_COUT_CNT,
	ID_INT__FG_BALANCE_CNT,
	ID_INT__FG_FCC_RST_CNT,
	ID_INT__FG_WDT_RST_CNT,
	ID_INT__FG_IT_RST_CNT,
	ID_INT__FG_CHG_RST_CNT,
	ID_INT__FG_DISCHG_RST_CNT,
	ID_INT__FG_LOADER_RST_CNT,
	ID_INT__BACKFLOW_CNT,
	ID_INT__BACKFLOW_ONE_ROUND,
	ID_INT__OTG_CNT,
	ID_INT__VBCL_ERR_CNT,
	ID_INT__LCM_MOS_ERR_CNT,
	ID_INT__MAX,

	ID_MARK__FBON,
	ID_MARK__MAX,

	ID_BOTH__STEP_SOC,
	ID_BOTH__STEP_VBAT,
	ID_BOTH__STEP_COUL,
	ID_BOTH__SOC,
	ID_BOTH__VBAT,
	ID_BOTH__VCHG,
	ID_BOTH__COUL,
	ID_BOTH__MAS_1P,
	ID_BOTH__MAX,

	ID_COMB__STEP_IBAT,
	ID_COMB__STEP_TBAT,
	ID_COMB__STEP_TMBOARD,
	ID_COMB__ISYS,
	ID_COMB__IBAT,
	ID_COMB__TBAT,
	ID_COMB__TMBOARD,
	ID_COMB__TSBOARD,
	ID_COMB__FC_IBUS,
	ID_COMB__FC_REQ_IBUS,
	ID_COMB__FC_VBAT,
	ID_COMB__FC_VBUS,
	ID_COMB__FC_CAB_MOHM,
	ID_COMB__FC_TADPT,
	ID_COMB__FC_TADPT_CONN,
	ID_COMB__FC_TUSB,
	ID_COMB__FC_TDIE,
	ID_COMB__FC_TBAT_CONN,
	ID_COMB__FC_MTBAT_CONN,
	ID_COMB__FC_TBAT_BOARD,
	ID_COMB__FC_VDP,
	ID_COMB__FC_VDM,
	ID_COMB__MAX,

	ID_BUFF__FG_INTERVAL,
	ID_BUFF__FG_DISCHG_RST,
	ID_BUFF__FG_CHG_RST,
	ID_BUFF__MAX,

	ID_LONG__HEALTH,
} RUN_INFO_INDEX_ENUM;


#define ARRAYSIZE_RUN_BIT		((ID_BIT__MAX + 32) / 32)
#define ARRAYSIZE_RUN_BYTE		(ID_BYTE__MAX - ID_BIT__MAX)
#define ARRAYSIZE_RUN_INT		(ID_INT__MAX - ID_BYTE__MAX)
#define ARRAYSIZE_RUN_MARK		(ID_MARK__MAX - ID_INT__MAX)
#define ARRAYSIZE_RUN_BOTH		(ID_BOTH__MAX - ID_MARK__MAX)
#define ARRAYSIZE_RUN_COMB		(ID_COMB__MAX - ID_BOTH__MAX)
#define ARRAYSIZE_RUN_BUFF		(ID_BUFF__MAX - ID_COMB__MAX)
#define INDEX_BYTE(i)			(i - ID_BIT__MAX - 1)
#define INDEX_INT(i)			(i - ID_BYTE__MAX - 1)
#define INDEX_MARK(i)			(i - ID_INT__MAX - 1)
#define INDEX_BOTH(i)			(i - ID_MARK__MAX - 1)
#define INDEX_COMB(i)			(i - ID_BOTH__MAX - 1)
#define INDEX_BUFF(i)			(i - ID_COMB__MAX - 1)
#define BETWEEN_BYTE(i)			(i > ID_BIT__MAX && i < ID_BYTE__MAX)
#define BETWEEN_INT(i)			(i > ID_BYTE__MAX && i < ID_INT__MAX)
#define BETWEEN_MARK(i)			(i > ID_INT__MAX && i < ID_MARK__MAX)
#define BETWEEN_BOTH(i)			(i > ID_MARK__MAX && i < ID_BOTH__MAX)
#define BETWEEN_COMB(i)			(i > ID_BOTH__MAX && i < ID_COMB__MAX)
#define BETWEEN_BUFF(i)			(i > ID_COMB__MAX && i < ID_BUFF__MAX)
#define FAULT_CUMULATIVE		6
struct fuel_info {
	uint32_t	run_bit[ARRAYSIZE_RUN_BIT];
	uint8_t		run_byte[ARRAYSIZE_RUN_BYTE];
	int		run_int[ARRAYSIZE_RUN_INT];
	struct mark_int	run_mark[ARRAYSIZE_RUN_MARK];
	struct both_int	run_both[ARRAYSIZE_RUN_BOTH];
	struct comb_int	run_comb[ARRAYSIZE_RUN_COMB];
	uint8_t*	run_buff[ARRAYSIZE_RUN_BUFF];
	unsigned long health;
	int chg_time;
	uint32_t insert_time;
	uint32_t unplug_time;
};

extern void fuelsummary_collect_value(RUN_INFO_INDEX_ENUM index, int value);
extern void fuelsummary_collect_data(RUN_INFO_INDEX_ENUM index, void *data);
extern void fuelsummary_clear_value(RUN_INFO_INDEX_ENUM index);
extern void fuelsummary_clear_data(const RUN_INFO_INDEX_ENUM *ids, int size);
typedef void (*func_profile_update_t)(void);
extern void fuelsummary_init_func_profile_update(func_profile_update_t func);


/* ***********************************************************
 *
 *  [Fuelsummary config params structure & function]
 *
 ************************************************************/
#define BYTE_SWAP_32(x) (int)((*(uint8_t *)(x)<<24) | (*(uint8_t *)(x+1)<<16) | (*(uint8_t *)(x+2)<<8) | (*(uint8_t *)(x+3)))
typedef enum {
	CONFIG_TYPE_NORMAL = 0,
	CONFIG_TYPE_DRIVER,
	CONFIG_TYPE_CHARGING,
	CONFIG_TYPE_BATTERY_CURVE,
} CONFIG_TYPE;
#define CONFIG_HEADER_SIZE		30
struct config_work {
	struct work_struct cwork;
	uint8_t type;
	uint8_t chksum;
	int page;
	int row;
	int column;
	int name_offset;
	int name_size;
	int buff_offset;
	int buff_size;
	int total;
	uint8_t *params;
};
typedef enum {
	STATIC_FIXED = false,
	EXTENSIBLE = true,
} VARIABLE_STATE;
typedef enum {
	PARAMS_TYPE_BOOL = 0,
	PARAMS_TYPE_BYTE,
	PARAMS_TYPE_INT = 4,
} PARAMS_TYPE;
struct config_params {
	bool extensible;
	uint8_t ptype;
	int page;
	int row;
	int column;
	size_t name_size;
	char *name;
	void *value;
	struct list_head list;
};
extern int fuelsummary_of_property_put(const char *propname, uint8_t ptype, void *value);
extern int fuelsummary_of_property_puts(const char *propname, bool extensible, uint8_t ptype, void *value, int row, int column);
extern int fuelsummary_of_property_put_matrix(const char *propname, bool extensible, uint8_t ptype, void *value, int page, int row, int column);

typedef enum {
	AGING_UNSUPPORTED = 0,
	DRIVER_SUPPORTED,
	USPACE_SUPPORTED,
} AGING_SUPPORT;


#endif/* #ifndef __FUEL_SUMMARY_H */
