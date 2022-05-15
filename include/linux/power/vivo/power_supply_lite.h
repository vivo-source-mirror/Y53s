#ifndef __POWER_SUPPLY_LITE_H
#define __POWER_SUPPLY_LITE_H

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#define PSYL_IO_DEBUG			0

#define GETBIT(x,n)                     (x & (0x1<<n))
#define SETBIT(x,n)                     (x = (x | (0x1<<n)))
#define CLRBIT(x,n)                     (x = (x & ~(0x1<<n)))

#define GETARRAYNUM(array)		(sizeof(array)/sizeof(array[0]))
#define BETWEEN(val, min, max)		((val) >= (min) && (val) <= (max))

#define DUMP_INFO_SIZE			1024
#define	SPM_PERIOD				(30*60)


/************************************************************
 *
 *   [power_supply_lite define]
 *
 ***********************************************************/
enum direction_io {
	SET_IN = 0,
	GET_OUT = 1,
};

enum power_supply_lite_num {
	PSYL_NUM_UNKNOWN = 0,
	PSYL_MAIN,
	PSYL_BATTERY_ID,
	PSYL_BATTERY,
	PSYL_METER,
	PSYL_CHARGE,
	PSYL_CMS,
#define PSL_IS_READY(stat)		((stat & 0x7E) == 0x7E)
	PSYL_MCU,
	PSYL_FUELSUMMARY,
	PSYL_CHGIC_PRIMARY,
	PSYL_CHGIC_PARALLEL,
	PSYL_FG_EX,
	PSYL_USB,
	PSYL_AC,
	PSYL_WIRELESS,
	PSYL_NUM_AUTO,
};

enum power_supply_lite_status {
	STATUS_UNIQUE = 0,
	STATUS_PRIMARY,
	STATUS_PARALLEL,
	STATUS_MCU,
};

enum parallel_hw_supplier {
	HW_SUPPLIER_PARALLEL_UNKNOWN = 0,
	HW_SUPPLIER_PARALLEL_1,
	HW_SUPPLIER_PARALLEL_2,
	HW_SUPPLIER_PARALLEL_3,
	HW_SUPPLIER_PARALLEL_4,
	HW_SUPPLIER_PARALLEL_MAX,
};

struct power_supply_lite {
	const char *name;
	void *drv_data;
	int status;
	int dev_num;
	bool initialized;
	int (*ioctrl_property)(struct power_supply_lite *psyl,
			enum direction_io io,
			enum power_supply_property psp,
			union power_supply_propval *val);
	struct list_head list;
};

extern void *power_supply_lite_get_drvdata(struct power_supply_lite *psyl);
extern struct power_supply_lite *get_power_supply_lite(const char *name, int dnum);
extern int power_supply_lite_register(struct power_supply_lite *psyl, int dnum);
extern int power_supply_lite_ioctrl_property(struct power_supply_lite *psyl,
		enum direction_io io,
		enum power_supply_property psp,
		union power_supply_propval *val);
extern int power_supply_lite_change(enum direction_io io,
		enum power_supply_property psp, union power_supply_propval *val);
extern int power_supply_lite_get_property(enum power_supply_lite_num psylnum,
		enum power_supply_property psp,
		union power_supply_propval *val);
extern int power_supply_lite_set_property(enum power_supply_lite_num psylnum,
		enum power_supply_property psp,
		union power_supply_propval *val);
extern bool power_supply_lite_is_ready(void);
extern bool power_supply_lite_detect(enum power_supply_lite_num psylnum);



/************************************************************
 *
 *   [enum lists]
 *
 ***********************************************************/
typedef enum {
	ROW = 0,
	COL,
} DIMENSION_ENUM;

typedef enum {
	INPUT_LIMIT = 0,
	LOWER_LIMIT,
	UPPER_LIMIT,
	LIMIT_INDEX_MAX,
} LIMIT_ENUM;

typedef enum {
	BATTERY_ID_NOT_LOADED = -1,
	BATTERY_UNKNOWN_SUPPLIER = 0,
	BATTERY_FIRST_SUPPLIER,
	BATTERY_SECOND_SUPPLIER,
	BATTERY_THIRD_SUPPLIER,
	BATTERY_FOURTH_SUPPLIER,
	BATTERY_FIFTH_SUPPLIER,
	BATTERY_SIXTH_SUPPLIER,
	BATTERY_SUPPLIER_MAX = BATTERY_SIXTH_SUPPLIER,
	BATTERY_DEFAULT_SUPPLIER = 100,
} BATTERY_VENDOR;

typedef enum {
	NORMAL_NO_WARNING = 0,
	CHARGER_VOLTAGE_HIGH,
	BATTERY_TEMPERATURE_HIGH,
	CURRENT_OVER_PROTECTION,
	BATTERY_VOLTAGE_HIGH,
	CHARGING_OVER_TIME,
	BATTERY_TEMPERATURE_LOW,
	BATTERY_ID_ERROR,
	BATTERY_TEMPERATURE_OFFSET,
	BATTERY_NOT_ORIGINAL,
	CHARGING_EXCEPTION,
	BATTERY_VOLTAGE_TEMPERATURE_HIGH,
	TEST_WARNING,
	CHARGE_USB_CONN_HEAT = 15,
	WARNING_NUMBER,
} WARNING_ENUM;

typedef enum {
	VBAT_AVG = 0,
	TBAT_AVG,
	IBAT_AVG,
	VBUS_AVG,
	AVG_NUMBER,
} AVG_ENUM;

typedef enum {
	AUTO_CHARGING = 0,
	NORMAL_CHARGINE,
	SINGLE_ENGIEN_CHARGINE,
	DUAL_ENGIEN_CHARGINE,
	FLASH_CHARGING,
} CHG_SCHEME_ENUM;

typedef enum {
	PMIC_DET_SOURCE = 0,
	CHGIC_DET_SOURCE,
} VBUS_DET_ENUM;

typedef enum {
	PWR_DET_READY = 0,
	PWR_DET_BUSY,
	PWR_DET_DONE,
} PWR_DET_ENUM;


typedef void (*CALL_BACK)(void);
extern int charge_psyl_register(struct power_supply_lite *psyl);
extern bool charge_psyl_has_mcu(void);
extern int get_average_value(AVG_ENUM type, int *buf, int size, int element);



#endif/* #ifndef __POWER_SUPPLY_LITE_H */
