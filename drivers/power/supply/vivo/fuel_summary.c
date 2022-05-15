/*****************************************************************************
 *
 * Filename:
 * ---------
 *  vivo-fuel_summary.c
 *
 * Project:
 * --------
 *  kernel
 *
 * Description:
 * ------------
 *  fuel summary : exception of charge & battery & fg
 *
 * Author:
 * -------
 *  @vivo Sif
 *
 ****************************************************************************/
#define pr_fmt(fmt) "FUELSUMMARY: %s: " fmt, __func__


#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include "fuel_summary.h"


//#undef pr_info
//#define pr_info(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)

#undef __ATTR_RW
#define __ATTR_RW(_name) __ATTR(_name, (S_IWUSR | S_IWGRP | S_IRUGO), _name##_show, _name##_store)

#define OWNER_10THD			0
#define FMEA_BIGDATA			0
#define FUELSUMMARY_TEST		0
#define FG_EX				0
#define DUAL_BAT_CELL			0
#define BUFF_SIZE_256			256
#define ENTRY_LEN_MAX			384
#define INFO_SIZE_MAX			1024
#define TRANSFER_RETRY_TIMES		3
#define BATTERY_CURVE_SUPPLIER		"battery_curve_supplier"
#define AGING_SOH_COMPENSATE		4
#define CUSTOM_INPUT_MAX		8000
#define CUSTOM_IBAT_MAX			20000
#define CUSTOM_VOLT_MAX			20000

#ifdef CONFIG_LIMIT_STACK_SIZE
#define FIFO_SIZE_MAX			1280
#else
#define FIFO_SIZE_MAX			1536
#endif


#if FMEA_BIGDATA
extern int writeData(char *modelId, char *filename, char *data);
extern int writeDatas(char *modelId, char *filename, char *fmt, ...);
#endif

typedef enum {
	BAT_PSY = 0,
	USB_PSY,
	CMS_PSY,
} PSY_NUM;

#define CURRENT_ARRAY_SIZE		32
struct current_roll {
	int cur_index;
	int max_index;
	int min_index;
	int sum;
	int avg;
	int size;
	int array[CURRENT_ARRAY_SIZE];
	//int carray[CURRENT_ARRAY_SIZE];
};

struct fuel_work {
	struct work_struct fwork;
	int running_scene;
};

struct fuelsummary_chip {
	struct device *dev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debug_root;
#endif
	struct class fuelsummary_cls;
	struct power_supply *usb_psy;
	struct power_supply *cms_psy;
	struct power_supply *bat_psy;
#if QCOM_PLATFORM
	struct power_supply *fuelsummary_psy;
#else
	struct power_supply_lite *fuelsummary_psyl;
#endif
	bool initialized;
	bool usb_present;
	bool user_custom;
	bool is_suspend;
	bool uspace_aging;
	bool chg_term;
	int uspace_cycle;
	int uspace_soh;
	int ag_support;
	int soc;
	int cycle_count;
	int chg_scheme;
	int fex_version;
	int fbon_ibat;
	int fboff_ibat;
	int fex_count;
	int custom_input;
	int custom_ibat;
	int custom_volt;
	int coul_support;
	int icurrent;
	int mah;
	int mas_1p_max;
	int mas_1p_min;
	int battery_id;
	uint8_t config_cnt;
	uint8_t config_bc_supplier;
	func_profile_update_t bc_update;
	struct fuelsummary_ex *warning_fex;
	struct fuelsummary_ex *soc_fex;
	struct fuelsummary_ex *chg_fex;
	struct fuelsummary_ex *chgic_fex;
	struct fuelsummary_ex *fg_fex;
	struct fuelsummary_ex *other_fex;
	struct list_head *fex_list;
	struct workqueue_struct *wq;
	struct fuel_work *running_10sthd;
	struct fuel_work *running_chg;
	struct fuel_work *running_usbin;
	struct fuel_work *running_usbout;
	struct fuel_work *running_suspend;
	struct fuel_work *running_resume;
	struct fuel_work *running_soc_change;
	struct list_head *params_list;
#if OWNER_10THD
	struct delayed_work thd_10s;
#endif
	struct delayed_work boot_start;
	struct fuel_info info;
	struct current_roll croll;
	struct kfifo fuel_fifo;
	spinlock_t fuel_fifo_lock;
};
static struct fuelsummary_chip fs_chip = {
	.initialized = false,
	.usb_present = false,
	.user_custom = false,
	.is_suspend = false,
	.chg_term = false,
	.soc = 0,
	.fex_count = 0,
	.custom_input = 2000,
	.custom_ibat = 2048,
	.custom_volt = 5000,
	.coul_support = COUL_NOT,
	.icurrent = 0,
	.warning_fex = NULL,
	.soc_fex = NULL,
	.chg_fex = NULL,
	.chgic_fex = NULL,
	.fg_fex = NULL,
	.other_fex = NULL,
	.fex_list = NULL,
	.battery_id = 0,
	.config_cnt = 0,
	.config_bc_supplier = 0,
	.bc_update = NULL,
	.running_10sthd = NULL,
	.running_chg = NULL,
	.running_usbin = NULL,
	.running_usbout = NULL,
	.running_suspend = NULL,
	.running_resume = NULL,
	.running_soc_change = NULL,
	.params_list = NULL,
	.info = {
		.health = 0,
		.chg_time = 0,
	},
	.croll = {
		.cur_index = 0,
		.max_index = 0,
		.min_index = 0,
		.sum = 0,
		.avg = 0,
		.size = 0,
		.array = {0},
	},
};

static DEFINE_MUTEX(params_list_mutex);



/************************************************************
 *
 *  [Fuelsummary info collect]
 *
 ***********************************************************/
void fuelsummary_collect_value(RUN_INFO_INDEX_ENUM index, int value)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	int bit = 0, offset = 0;
	struct mark_int *mint = NULL;
	struct both_int *bint = NULL;
	struct comb_int *cint = NULL;

	if (index < ID_BIT__MAX) {
		bit = index % 32;
		offset = index / 32;
		if (value == 0)
			info->run_bit[offset] &= ~(0x1 << bit);
		else
			info->run_bit[offset] |= (0x1 << bit);
	} else if (BETWEEN_BYTE(index)) {
		info->run_byte[INDEX_BYTE(index)] = ((value == -1) ? 0 :(value & 0xff));
	} else if (BETWEEN_INT(index)) {
		info->run_int[INDEX_INT(index)] = value;
	} else if (BETWEEN_MARK(index)) {
		mint = &(info->run_mark[INDEX_MARK(index)]);
		mint->state = value ? true : false;
		if (mint->state)
			mint->true_cnt += 1;
		else
			mint->false_cnt += 1;
	} else if (BETWEEN_BOTH(index)) {
		bint = &(info->run_both[INDEX_BOTH(index)]);
		if (!bint->start) {
			bint->start = true;
			bint->start_val = value;
			bint->cur_val = value;
		} else {
			bint->cur_val = value;
		}
	} else if (BETWEEN_COMB(index)) {
		cint = &(info->run_comb[INDEX_COMB(index)]);
		if (cint->num >= 0xffff || cint->num <= 0) {
			cint->num = 1;
			cint->sum = value;
			cint->cur_val = value;
			cint->avg_val = value;
			cint->max_val = value;
			cint->min_val = value;
		} else {
			cint->num++;
			cint->sum += value;
			cint->cur_val = value;
			cint->avg_val = cint->sum / cint->num;
			if (cint->max_val < value)
				cint->max_val = value;
			if (cint->min_val > value)
				cint->min_val = value;
		}
	}
}
EXPORT_SYMBOL(fuelsummary_collect_value);
void fuelsummary_collect_data(RUN_INFO_INDEX_ENUM index, void *data)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);

	if (index < ID_COMB__MAX) {
		fuelsummary_collect_value(index, *(int *)(data));
	} else if (BETWEEN_BUFF(index)) {
		info->run_buff[INDEX_BUFF(index)] = (uint8_t *)(data);
	} else if (index == ID_LONG__HEALTH) {
		info->health = *(unsigned long *)(data);
	}
}
EXPORT_SYMBOL(fuelsummary_collect_data);
void fuelsummary_clear_value(RUN_INFO_INDEX_ENUM index)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	int bit = 0, offset = 0, len = 0;
	uint8_t *buf = NULL;
	struct mark_int *mint = NULL;
	struct both_int *bint = NULL;
	struct comb_int *cint = NULL;

	if (index < ID_BIT__MAX) {
		bit = index % 32;
		offset = index / 32;
		info->run_bit[offset] &= ~(0x1 << bit);
	} else if (index == ID_BIT__MAX) {
		memset(info->run_bit, 0x0, ARRAYSIZE_RUN_BIT * sizeof(uint32_t));
	} else if (BETWEEN_BYTE(index)) {
		info->run_byte[INDEX_BYTE(index)] = 0;
	} else if (BETWEEN_INT(index)) {
		info->run_int[INDEX_INT(index)] = 0;
	} else if (BETWEEN_MARK(index)) {
		mint = &(info->run_mark[INDEX_MARK(index)]);
		mint->state = false;
		mint->true_cnt = 0;
		mint->false_cnt = 0;
	} else if (BETWEEN_BOTH(index)) {
		bint = &(info->run_both[INDEX_BOTH(index)]);
		bint->start = false;
		bint->start_val = 0;
		bint->cur_val = 0;
	} else if (BETWEEN_COMB(index)) {
		cint = &(info->run_comb[INDEX_COMB(index)]);
		cint->num = 0;
		cint->sum = 0;
		cint->cur_val = 0;
		cint->avg_val = 0;
		cint->max_val = 0;
		cint->min_val = 0;
	} else if (BETWEEN_BUFF(index)) {
		buf = info->run_buff[INDEX_BUFF(index)];
		if (buf != NULL) {
			len = strlen(buf);
			if (len > 0) {
				if (len > BUFF_SIZE_256)
					len = BUFF_SIZE_256;
				memset(buf, 0x0, len);
			}
		}
	} else if (index == ID_LONG__HEALTH) {
		info->health = 0x0;
	}
}
EXPORT_SYMBOL(fuelsummary_clear_value);
void fuelsummary_clear_data(const RUN_INFO_INDEX_ENUM *ids, int size)
{
	int i;

	if (size > ID_LONG__HEALTH) {
		pr_info("size=%d out of bounds err\n", size);
		return;
	}

	for (i = 0; i < size; i++) {
		if (ids[i] <= ID_LONG__HEALTH) {
			fuelsummary_clear_value(ids[i]);
		}
	}
}
EXPORT_SYMBOL(fuelsummary_clear_data);
void fuelsummary_init_func_profile_update(func_profile_update_t func)
{
	struct fuelsummary_chip *chip = &fs_chip;
	chip->bc_update = func;
}
EXPORT_SYMBOL(fuelsummary_init_func_profile_update);
int fuelsummary_writeBigData(struct fuelsummary_chip *chip, uint8_t *data)
{
#if FMEA_BIGDATA
	return writeData("1501", "1501_2", data);
#else
	int ret = 0;
	size_t len = 0;
	uint8_t entry[INFO_SIZE_MAX];

	memset(entry, 0, sizeof(entry));
	len = scnprintf(entry, INFO_SIZE_MAX, "%s\n", data);
	if (!len) {
		pr_info("entry is empty!\n");
		return -EFAULT;
	}

	spin_lock_irq(&(chip->fuel_fifo_lock));
	if (entry[len-1] != '\n') {
		pr_info("entry size overflow!\n");
		entry[len-1] = '\n';
	}

	if (kfifo_is_full(&(chip->fuel_fifo)))
		kfifo_reset_out(&(chip->fuel_fifo));

	ret = kfifo_in(&(chip->fuel_fifo), entry, len);
	spin_unlock_irq(&(chip->fuel_fifo_lock));
	return ret;
#endif
}
int fuelsummary_writeBigDatas(struct fuelsummary_chip *chip, char *fmt, ...)
{
	va_list args;
	char data[INFO_SIZE_MAX];

	memset(data, 0, sizeof(data));
	va_start(args, fmt);
	vscnprintf(data, INFO_SIZE_MAX, fmt, args);
	va_end(args);
	return fuelsummary_writeBigData(chip, data);
}

#if QCOM_PLATFORM
static int fuelsummary_get_psy_data(PSY_NUM n, enum power_supply_property psp, int *data)
{
	int ret = 0;
	struct power_supply *psy = NULL;
	union power_supply_propval val = {0,};
	struct fuelsummary_chip *chip = &fs_chip;

	switch (n) {
	case BAT_PSY:
		if (!chip->bat_psy)
			chip->bat_psy = power_supply_get_by_name("battery");
		psy = chip->bat_psy;
		break;
	case USB_PSY:
		if (!chip->usb_psy)
			chip->usb_psy = power_supply_get_by_name("usb");
		psy = chip->usb_psy;
		break;
	case CMS_PSY:
		if (!chip->cms_psy)
			chip->cms_psy = power_supply_get_by_name("cms");
		psy = chip->cms_psy;
		break;
	default:
		break;
	}

	if (psy == NULL) {
		pr_err("psy[%d] not found\n", n);
		ret = -EINVAL;
		goto exit;
	}

	ret = power_supply_get_property(psy, psp, &val);
	if (ret < 0) {
		pr_err("psp[%d] not found\n", psp);
		ret = -EEXIST;
		goto exit;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		*data = (val.intval / 1000);
		break;
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP:
	case POWER_SUPPLY_PROP_PARALLEL_BOARD_TEMP:
		*data = (val.intval / 10);
		break;
	default:
		*data = val.intval;
		break;
	}
exit:
	return ret;
}
#else
static int fuelsummary_get_psy_data(PSY_NUM n, enum power_supply_property psp, int *data)
{
	int ret = 0;
	enum power_supply_lite_num psyl = PSYL_NUM_UNKNOWN;
	union power_supply_propval val = {1,};

	switch (n) {
	case BAT_PSY:
		psyl = PSYL_BATTERY;
		break;
	case USB_PSY:
		psyl = PSYL_CHARGE;
		break;
	case CMS_PSY:
		psyl = PSYL_CMS;
		break;
	default:
		break;
	}

	if (psyl == PSYL_NUM_UNKNOWN) {
		pr_err("psyl[%d] not found\n", n);
		ret = -EINVAL;
		goto exit;
	}

	ret = power_supply_lite_get_property(psyl, psp, &val);
	if (ret < 0) {
		pr_err("psp[%d] not found\n", psp);
		ret = -EEXIST;
		goto exit;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP:
	case POWER_SUPPLY_PROP_PARALLEL_BOARD_TEMP:
		*data = (val.intval / 10);
		break;
	default:
		*data = val.intval;
		break;
	}
exit:
	return ret;
}
#endif

static int fuelsummary_get_stay_sec(uint32_t msecs1, uint32_t msecs2)
{
	return abs(msecs1 - msecs2) / 1000;
}

static void run_reset(RUN_INFO_INDEX_ENUM index)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	struct both_int *bint = NULL;
	struct comb_int *cint = NULL;

	if (index < ID_MARK__MAX && index > ID_COMB__MAX) {
		fuelsummary_clear_value(index);
	} else if  (BETWEEN_BOTH(index)) {
		bint = &(info->run_both[INDEX_BOTH(index)]);
		bint->start = true;
		bint->start_val = bint->cur_val;
	} else if (BETWEEN_COMB(index)) {
		cint = &(info->run_comb[INDEX_COMB(index)]);
		cint->num = 1;
		cint->sum = cint->cur_val;
		cint->avg_val = cint->cur_val;
		cint->max_val = cint->cur_val;
		cint->min_val = cint->cur_val;
	}
}
static int run_val(RUN_INFO_INDEX_ENUM index)
{
	int ret = 0;
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	int bit = 0, offset = 0;

	if (index < ID_BIT__MAX) {
		bit = index % 32;
		offset = index / 32;
		ret = ((info->run_bit[offset] >> bit) & 0x1);
	} else if (BETWEEN_BYTE(index)) {
		ret = info->run_byte[INDEX_BYTE(index)];
	} else if (BETWEEN_INT(index)) {
		ret = info->run_int[INDEX_INT(index)];
	} else if (BETWEEN_MARK(index)) {
		ret = info->run_mark[INDEX_MARK(index)].state;
	} else if (BETWEEN_BOTH(index)) {
		ret = info->run_both[INDEX_BOTH(index)].cur_val;
	} else if (BETWEEN_COMB(index)) {
		ret = info->run_comb[INDEX_COMB(index)].cur_val;
	}

	return ret;
}
static void *run_pval(RUN_INFO_INDEX_ENUM index)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);

	if (BETWEEN_MARK(index)) {
		return &(info->run_mark[INDEX_MARK(index)]);
	} else if (BETWEEN_BOTH(index)) {
		return &(info->run_both[INDEX_BOTH(index)]);
	} else if (BETWEEN_COMB(index)) {
		return &(info->run_comb[INDEX_COMB(index)]);
	} else if (BETWEEN_BUFF(index)) {
		return info->run_buff[INDEX_BUFF(index)];
	} else {
		return NULL;
	}
}
static int fuelsummary_save_run_info(struct fuelsummary_chip *chip, unsigned long excode, const RUN_INFO_INDEX_ENUM *ids, int size)
{
	int ret = 0;
	struct fuel_info *info = &(chip->info);
	int i = 0, j = 0, index = 0;
	uint8_t val[64];
	uint8_t buff[INFO_SIZE_MAX];
	struct mark_int *mint = NULL;
	struct both_int *bint = NULL;
	struct comb_int *cint = NULL;

	if (size > ID_LONG__HEALTH) {
		pr_err("size=%d out of bounds err\n", size);
		return -EINVAL;
	}

	memset(buff, 0x0, INFO_SIZE_MAX);
	for (i = 0; i < size; i++) {
		index = ids[i];
		memset(val, 0x0, 64);
		if (index == ID_BIT__MAX) {
			sprintf(val, ",%x", info->run_bit[0]);
			strcat(buff, val);
			for (j = 1; j < ARRAYSIZE_RUN_BIT; j++) {
				memset(val, 0x0, 64);
				sprintf(val, "|%x", info->run_bit[j]);
				strcat(buff, val);
			}
		} else if (BETWEEN_BYTE(index)) {
			sprintf(val, ",%d", info->run_byte[INDEX_BYTE(index)]);
			strcat(buff, val);
		} else if (BETWEEN_INT(index)) {
			sprintf(val, ",%d", info->run_int[INDEX_INT(index)]);
			strcat(buff, val);
		} else if (BETWEEN_MARK(index)) {
			mint = &(info->run_mark[INDEX_MARK(index)]);
			sprintf(val, ",%d|%d", mint->true_cnt, mint->false_cnt);
			strcat(buff, val);
		} else if (BETWEEN_BOTH(index)) {
			bint = &(info->run_both[INDEX_BOTH(index)]);
			sprintf(val, ",%d|%d", bint->start_val, bint->cur_val);
			strcat(buff, val);
		} else if (BETWEEN_COMB(index)) {
			cint = &(info->run_comb[INDEX_COMB(index)]);
			sprintf(val, ",%d|%d|%d", cint->avg_val, cint->max_val, cint->min_val);
			strcat(buff, val);
		} else if (BETWEEN_BUFF(index)) {
			strcat(buff, info->run_buff[INDEX_BUFF(index)]);
		} else if (index == ID_LONG__HEALTH) {
			sprintf(val, ",%lx", info->health);
			strcat(buff, val);
		}
	}

	fuelsummary_writeBigDatas(chip, "v%d:%lx%s", chip->fex_version, excode, buff);
	return ret;
}



/************************************************************
 *
 *  [Fuelsummary config params founction]
 *
 ***********************************************************/
static int params_list_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct list_head *l;

	if (!chip->params_list) {
		l = kzalloc(sizeof(*l), GFP_KERNEL);
		if (l) {
			chip->params_list = l;
			INIT_LIST_HEAD(chip->params_list);
		} else {
			pr_err("params_list alloc fail\n");
			ret = -ENOMEM;
		}
	}
	return ret;
}
static int params_node_init(const char *propname, bool extensible, uint8_t ptype, void *value, int page, int row, int column)
{
	int ret = 0;
	struct fuelsummary_chip *chip = &fs_chip;
	struct config_params *cp, *temp;
	size_t propname_size = 0;

	mutex_lock(&params_list_mutex);
	if (propname == NULL) {
		pr_err("propname is NULL\n");
		ret = -EFAULT;
		goto exit;
	} else {
		propname_size = strlen(propname);
	}

	if (chip->params_list && !list_empty(chip->params_list)) {
		list_for_each_entry(temp, chip->params_list, list) {
			if (temp != NULL && temp->name != NULL) {
				if ((temp->name_size == propname_size) && !strncmp(temp->name, propname, propname_size)) {
					pr_err("%s: is existed\n", propname);
					ret = -EEXIST;
					goto exit;
				}
			}
		}
	}

	cp = kzalloc(sizeof(*cp), GFP_KERNEL);
	if (!cp) {
		pr_err("couldn't alloc config_params struct\n");
		ret = -ENOMEM;
		goto exit;
	}
	cp->extensible = extensible;
	cp->ptype = ptype;
	cp->page = page;
	cp->row = row;
	cp->column = column;

	cp->name_size = propname_size;
	if (cp->name_size > 255)
		cp->name_size = 255;

	cp->name = kzalloc((cp->name_size + 1), GFP_KERNEL);
	if (!cp->name) {
		pr_err("couldn't alloc name\n");
		kfree(cp);
		ret = -ENOMEM;
		goto exit;
	}
	strncpy(cp->name, propname, cp->name_size);

	cp->value = value;

	INIT_LIST_HEAD(&(cp->list));
	if (!chip->params_list) {
		if (params_list_init(chip)) {
			pr_err("params list init err\n");
			kfree(cp->name);
			kfree(cp);
			ret = -ENOMEM;
			goto exit;
		}
	}
	list_add_tail(&(cp->list), chip->params_list);
exit:
	mutex_unlock(&params_list_mutex);
	return ret;

}
int fuelsummary_of_property_put(const char *propname, uint8_t ptype, void *value)
{
	return params_node_init(propname, STATIC_FIXED, ptype, value, 1, 1, 1);
}
EXPORT_SYMBOL(fuelsummary_of_property_put);
int fuelsummary_of_property_puts(const char *propname, bool extensible, uint8_t ptype, void *value, int row, int column)
{
	return params_node_init(propname, extensible, ptype, value, 1, row, column);
}
EXPORT_SYMBOL(fuelsummary_of_property_puts);
int fuelsummary_of_property_put_matrix(const char *propname, bool extensible, uint8_t ptype, void *value, int page, int row, int column)
{
	return params_node_init(propname, extensible, ptype, value, page, row, column);
}
EXPORT_SYMBOL(fuelsummary_of_property_put_matrix);
static bool config_params_update(struct fuelsummary_chip *chip, struct config_work *cw)
{
	bool ret = false, hit = false;
	int size = 0, cw_size = 0, cp_size = 0, i = 0;
	uint8_t temp = 0;
	uint8_t name[256];
	int *intval, *inttemp;
	uint8_t *byteval, *bytetemp;
	struct config_params *cp;

	if (!chip->params_list || list_empty(chip->params_list)) {
		pr_err("params_list is empty\n");
		goto exit;
	}

	memset(name, 0x0, 256);
	if (cw->name_size > 255)
		memcpy(name, (cw->params + cw->name_offset), 255);
	else
		memcpy(name, (cw->params + cw->name_offset), cw->name_size);

	list_for_each_entry(cp, chip->params_list, list) {
		if ((cp->name_size == cw->name_size) && !strncmp(cp->name, name, cp->name_size)) {
			hit = true;
			break;
		}
	}

	if (hit) {
		cw_size = cw->page * cw->row * cw->column;
		cp_size = cp->page * cp->row * cp->column;
		if (cw_size == cp_size) {
			switch (cp->ptype) {
			case PARAMS_TYPE_BOOL:
				if (!cp->value) {
					pr_err("static_fixed bool values is null\n");
					goto exit;
				}
				temp = *(cw->params + cw->buff_offset);
				if (temp == 1) {
					*(bool *)(cp->value) = true;
					ret = true;
				} else if (temp == 0) {
					*(bool *)(cp->value) = false;
					ret = true;
				} else {
					pr_err("bool args err:%d!\n", temp);
				}
				break;
			case PARAMS_TYPE_BYTE:
				if (cp->extensible) {
					byteval = *(uint8_t **)(cp->value);
					if (!byteval) {
						byteval = kzalloc(cw_size, GFP_KERNEL);
						if (!byteval) {
							pr_err("couldn't alloc extensible byte values\n");
							goto exit;
						}
						*(uint8_t **)(cp->value) = byteval;
					}
				} else {
					byteval = (uint8_t *)(cp->value);
					if (!byteval) {
						pr_err("static_fixed byte values is null\n");
						goto exit;
					}
				}
				memcpy(byteval, (cw->params + cw->buff_offset), cw_size);
				ret = true;
				break;
			case PARAMS_TYPE_INT:
				if (cp->extensible) {
					intval = *(int **)(cp->value);
					if (!intval) {
						intval = kzalloc((cw_size * sizeof(int)), GFP_KERNEL);
						if (!intval) {
							pr_err("couldn't alloc extensible int values\n");
							goto exit;
						}
						*(int **)(cp->value) = intval;
					}
				} else {
					intval = (int *)(cp->value);
					if (!intval) {
						pr_err("static_fixed int values is null\n");
						goto exit;
					}
				}
				for (i = 0; i < cw_size; i++) {
					*(intval + i) = BYTE_SWAP_32(cw->params + cw->buff_offset + i * 4);
				}
				ret = true;
				break;
			default:
				pr_err("params type not found : %d\n", cp->ptype);
				break;
			}
		} else {
			if (cw_size > cp_size)
				size = cw_size;
			else
				size = cp_size;

			if (cp->extensible) {
				switch (cp->ptype) {
				case PARAMS_TYPE_BYTE:
					bytetemp = *(uint8_t **)(cp->value);
					byteval = kzalloc(size, GFP_KERNEL);
					if (!byteval) {
						pr_err("couldn't alloc extensible byte new size\n");
						goto exit;
					}
					*(uint8_t **)(cp->value) = byteval;
					if (cw_size > cp_size) {
						cp->row = cw->row;
						cp->column = cw->column;
					}
					memcpy(byteval, (cw->params + cw->buff_offset), cw_size);
					if (bytetemp)
						kfree(bytetemp);
					ret = true;
					break;
				case PARAMS_TYPE_INT:
					inttemp = *(int **)(cp->value);
					intval = kzalloc((size * sizeof(int)), GFP_KERNEL);
					if (!intval) {
						pr_err("couldn't alloc extensible int new size\\n");
						goto exit;
					}
					*(int **)(cp->value) = intval;
					if (cw_size > cp_size) {
						cp->row = cw->row;
						cp->column = cw->column;
					}
					for (i = 0; i < cw_size; i++) {
						*(intval + i) = BYTE_SWAP_32(cw->params + cw->buff_offset + i * 4);
					}
					if (inttemp)
						kfree(inttemp);
					ret = true;
					break;
				default:
					pr_err("extensible type not found : %d\n", cp->ptype);
					break;
				}
			} else {
				pr_err("type=%d, name=%s data size not extensible!\n", cw->type, name);
			}
		}
	} else {
		pr_err("type=%d, name=%s not found!\n", cw->type, name);
	}

	if (ret && chip->bc_update && !strncmp(name, "qcom,fg-profile-data", 20)) {
		pr_info("update battery curve profile\n");
		chip->bc_update();
	}
exit:
	return ret;
}
static void config_params_dump(void)
{
	struct fuelsummary_chip *chip = &fs_chip;
	int size = 0, i = 0;
	bool err = false;
	int *intval = NULL;
	bool *boolval = NULL;
	uint8_t *byteval = NULL;
	uint8_t val[24];
	uint8_t str[512];
	struct config_params *cp;

	if (!chip->params_list || list_empty(chip->params_list)) {
		pr_err("params_list is empty\n");
		return;
	}

	list_for_each_entry(cp, chip->params_list, list) {
		err = false;
		pr_info("%s: page=%d,row=%d,column=%d\n", cp->name, cp->page, cp->row, cp->column);
		switch (cp->ptype) {
		case PARAMS_TYPE_BOOL:
			boolval =  (bool *)(cp->value);
		case PARAMS_TYPE_BYTE:
			if (cp->extensible) {
				byteval = *(uint8_t **)(cp->value);
			} else {
				byteval = (uint8_t *)(cp->value);
			}
			break;
		case PARAMS_TYPE_INT:
			if (cp->extensible) {
				intval = *(int **)(cp->value);
			} else {
				intval = (int *)(cp->value);
			}
			break;
		default:
			pr_err("type not found : %d\n", cp->ptype);
			err = true;
			break;
		}
		if (err)
			break;

		size = cp->page * cp->row * cp->column;
		memset(str, 0x0, 512);
		for (i = 0; i < size; i++) {
			if (i != 0 && i % cp->column == 0) {
				pr_info("%s\n", str);
				memset(str, 0x0, 512);
			}

			memset(val, 0x0, 24);
			switch (cp->ptype) {
			case PARAMS_TYPE_BOOL:
				if (boolval)
					sprintf(val, "%d", *boolval);
				break;
			case PARAMS_TYPE_BYTE:
				if (byteval)
					sprintf(val, "0x%02x ", *(byteval + i));
				break;
			case PARAMS_TYPE_INT:
				if (intval)
					sprintf(val, "%d ", *(intval + i));
				break;
			}
			strcat(str, val);
		}
		pr_info("%s\n", str);
	}
}
static void config_params_worker(struct work_struct *work)
{
	int i = 0;
	bool isbyte = false;
	uint8_t csum;
	uint8_t name[256];
	struct fuelsummary_chip *chip = &fs_chip;
	struct config_work *cw = container_of(work, struct config_work, cwork);

	if (cw->total == (CONFIG_HEADER_SIZE + cw->name_size + cw->buff_size) && cw->buff_size > 0) {
		isbyte = ((cw->params[0] & 0x80) != 0);
		memset(name, 0x0, 256);
		if (cw->name_size > 255)
			memcpy(name, (cw->params + cw->name_offset), 255);
		else
			memcpy(name, (cw->params + cw->name_offset), cw->name_size);
#if FUELSUMMARY_TEST
		pr_info("type=0x%02x, page=%d, row=%d, column=%d, name=%s\n", cw->type, cw->page, cw->row, cw->column, name);
		if (isbyte) {
			for (i = 0; i < cw->buff_size; i++) {
				pr_info("buff[%d]=0x%02x\n", i, *(cw->params + cw->buff_offset + i));
			}
		} else {
			for (i = 0; i < (cw->buff_size / 4); i++) {
				pr_info("buff[%d]=%d\n", i, BYTE_SWAP_32(cw->params + cw->buff_offset + i * 4));
			}
		}
#endif
		csum = cw->type;
		for (i = 0; i < cw->buff_size; i++) {
			csum += *(cw->params + cw->buff_offset + i);
		}
		if (csum != cw->chksum) {
			pr_err("type=%d, name=%s, chksum=0x%02x !=  csum=0x%02x\n", cw->type, name, cw->chksum, csum);
			goto exit;
		}

		switch (cw->type) {
		case CONFIG_TYPE_DRIVER:
		case CONFIG_TYPE_CHARGING:
			config_params_update(chip, cw);
			chip->config_cnt++;
			break;
		case CONFIG_TYPE_BATTERY_CURVE:
			if (!strncmp(name, BATTERY_CURVE_SUPPLIER, cw->name_size)) {
				if (isbyte && cw->buff_size >= 1) {
					chip->config_bc_supplier = *(uint8_t *)(cw->params + cw->buff_offset);
				} else if (!isbyte && cw->buff_size >= 4) {
					chip->config_bc_supplier = BYTE_SWAP_32(cw->params + cw->buff_offset);
				} else {
					chip->config_bc_supplier = 0;
				}

				pr_info("type=%d, name=%s : %d\n", cw->type, name, chip->config_bc_supplier);
			} else {
				if (chip->battery_id == 0) {
					fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_BATTERY_ID, &(chip->battery_id));
				}

				if (chip->config_bc_supplier == chip->battery_id) {
					config_params_update(chip, cw);
					chip->config_cnt++;
				} else {
					pr_err("type=%d, name=%s, bc_supplier=%d != battery_id=%d\n", cw->type, name, chip->config_bc_supplier, chip->battery_id);
				}
			}
			break;
		default:
			pr_err("config cmd not support\n");
			break;
		}
	} else {
		pr_err("total=%d, name_size=%d, buff_size=%d, total size err\n", cw->total, cw->name_size, cw->buff_size);
	}
exit:
	kfree(cw->params);
	kfree(cw);
}



/************************************************************
 *
 *  [Fuelsummary list]
 *
 ***********************************************************/
static int fex_list_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct list_head *l;

	if (!chip->fex_list) {
		l = kzalloc(sizeof(*l), GFP_KERNEL);
		if (l) {
			chip->fex_list = l;
			INIT_LIST_HEAD(chip->fex_list);
		} else {
			pr_info("fex_list alloc fail\n");
			ret = -ENOMEM;
		}
	}
	return ret;
}



/************************************************************
 *
 *  [Fuelsummary exception create & update]
 *
 ***********************************************************/
struct fuelsummary_ex *fuelsummary_fex_create(unsigned long ex_enum, void *owned,
		FEX_ANALYZE func)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuelsummary_ex *f = kzalloc(sizeof(*f), GFP_KERNEL);

	if (!f) {
		pr_info("couldn't alloc fuelsummary_ex struct\n");
		return NULL;
	}
	f->code = f->ex_code = ex_enum;
	f->scenes = 0x0;
	f->valid = false;
	f->private = owned;
	f->analyze = func;
	mutex_init(&(f->fmutex));
	INIT_LIST_HEAD(&(f->list));

	if (!chip->fex_list) {
		if (fex_list_init(chip)) {
			kfree(f);
			return NULL;
		}
	}
	list_add_tail(&(f->list), chip->fex_list);
	return f;
}
EXPORT_SYMBOL(fuelsummary_fex_create);
void fuelsummary_set_exbit(unsigned long *ex_code, int bit, bool status)
{
	bool update = false;

	if (bit >= EX_HEAD_LIMIT_BIT) {
		pr_info("bit overflow!");
		return;
	}

	if (!status && test_and_clear_bit(bit, ex_code))
		update = true;
	else if (status && !test_and_set_bit(bit, ex_code))
		update = true;

	if (update)
		pr_info("EX[0x%08lx] update bit[%d]\n", *ex_code, bit);
}
EXPORT_SYMBOL(fuelsummary_set_exbit);


/************************************************************
 *
 *  [warning fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM warning_ids[] = {
	ID_BIT__MAX, ID_BYTE__INPUT_SCALE, ID_BYTE__INBAT_SCALE,
	ID_BYTE__CHGT, ID_BOTH__VBAT, ID_BOTH__VCHG, ID_COMB__IBAT,
	ID_COMB__TBAT, ID_COMB__TMBOARD, ID_COMB__TSBOARD,
	ID_COMB__FC_TBAT_BOARD, ID_COMB__FC_TBAT_CONN, ID_COMB__FC_TUSB,
	ID_COMB__FC_TDIE, ID_COMB__FC_TADPT, ID_COMB__FC_TADPT_CONN
};
static int warning_ids_size = ARRAY_SIZE(warning_ids);

struct warning_context {
	bool ready;
};
#if 1//QCOM_PLATFORM
/* [copy from charger-monitor-v2.h for health enum] */
enum{
	HEALTH_STATUS_MIN = 0,
	HEALTH_STATUS_CHG_OV = HEALTH_STATUS_MIN,
	HEALTH_STATUS_BAT_WARM,
	HEALTH_STATUS_CHG_OC,
	HEALTH_STATUS_BAT_OV,
	HEALTH_STATUS_CHG_TIMEOUT,
	HEALTH_STATUS_BAT_COLD,
	HEALTH_STATUS_BAT_MISSING,
	HEALTH_STATUS_CHG_UV,
	HEALTH_STATUS_BAT_INVALID,
	HEALTH_STATUS_CHG_ERR,
	//HEALTH_STATUS_BAT_UV,
	//HEALTH_STATUS_CHG_DONE,
	HEALTH_STATUS_BAT_UNPLUGED = 12,
	HEALTH_STATUS_HTCCC = 13,
	HEALTH_STATUS_USB_CONN_HEAT = 14,
	HEALTH_STATUS_USBID_WATER = 15,
	HEALTH_STATUS_MAX,
};
#endif
static void warning_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	struct warning_context *context = fex->private;

	bool status = false;
	unsigned long code = 0;
	unsigned long health = info->health;

	if (!fex->valid) {
		pr_info("warning_fex not valid\n");
		return;
	}

	switch (scene) {
	case SCENE_CHG:
		if (!context->ready && (run_val(ID_BIT__FCHG) || info->chg_time >= READY_TIME)) {
			context->ready = true;
		}
#if 1//QCOM_PLATFORM
		status = test_bit(HEALTH_STATUS_BAT_WARM, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_OT, status);

		status = test_bit(HEALTH_STATUS_BAT_COLD, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_LT, status);

		status = test_bit(HEALTH_STATUS_CHG_TIMEOUT, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_CHG_TO, status);

		status = test_bit(HEALTH_STATUS_CHG_ERR, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_CHG_EX, status);

		status = test_bit(HEALTH_STATUS_HTCCC, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_HTCCC, status);

		status = test_bit(HEALTH_STATUS_BAT_MISSING, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BID_MS, status);

		status = test_bit(HEALTH_STATUS_BAT_INVALID, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_NORIGIN, status);

		status = test_bit(HEALTH_STATUS_BAT_OV, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_OV, status);

		status = test_bit(HEALTH_STATUS_BAT_UNPLUGED, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_UNPLUGED, status);

		status = test_bit(HEALTH_STATUS_USB_CONN_HEAT, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_USB_OT, status);

		status = test_bit(HEALTH_STATUS_USBID_WATER, &health) || (run_val(ID_BIT__USB_WATER) == 1);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_USB_MT, status);

		if (chip->chg_scheme == SCHEME_VIVO_FASTCHG) {
			status = (run_val(ID_COMB__FC_TUSB) >= 80);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_USB_2GEAR, status);

			//status = (run_val(ID_COMB__FC_TADPT) >= 93);
			//fuelsummary_set_exbit(&(fex->ex_code), WARNING_ADP_OT, status);

			status = (run_val(ID_COMB__FC_TBAT_BOARD) >= 65);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_BBOARD_OT, status);

			status = (run_val(ID_COMB__FC_TBAT_BOARD) >= 75);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_BBOARD_2GEAR, status);

			status = (run_val(ID_COMB__FC_TBAT_CONN) >= 70);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_BCONN_OT, status);

			status = (run_val(ID_COMB__FC_TBAT_CONN) >= 85);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_BCONN_2GEAR, status);

			status = (run_val(ID_COMB__FC_TADPT_CONN) >= 90);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_ADPCONN_OT, status);

			status = (run_val(ID_COMB__FC_TADPT_CONN) >= 120);
			fuelsummary_set_exbit(&(fex->ex_code), WARNING_DIE_OT, status);
		}
#else
		status = test_bit(BATTERY_TEMPERATURE_HIGH, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_OT, status);

		status = test_bit(BATTERY_TEMPERATURE_LOW, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_LT, status);

		status = test_bit(CHARGING_OVER_TIME, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_CHG_TO, status);

		status = test_bit(BATTERY_VOLTAGE_TEMPERATURE_HIGH, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_HTCCC, status);

		status = test_bit(BATTERY_ID_ERROR, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_NORIGIN, status);

		status = test_bit(BATTERY_VOLTAGE_HIGH, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_OV, status);

		status = test_bit(BATTERY_NOT_ORIGINAL, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_BAT_UNPLUGED, status);

		status = test_bit(CHARGE_USB_CONN_HEAT, &health);
		fuelsummary_set_exbit(&(fex->ex_code), WARNING_USB_OT, status);
#endif
		code = fex->code;
		fex->code |= fex->ex_code;
		if (fex->code > code) {
			pr_info("trigger ex_warning\n");
			fuelsummary_save_run_info(chip, fex->ex_code, warning_ids, warning_ids_size);
		}
		break;
	case SCENE_INIT:
	case SCENE_USBOUT:
		fex->code = fex->ex_code = EX_WARNING;
		if (context->ready) {
			context->ready = false;
		}
		break;
	default:
		break;
	}
}
static int warning_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct warning_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("warning_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->warning_fex = fuelsummary_fex_create(EX_WARNING, context, warning_fex_analyze);
	if (!chip->warning_fex) {
		ret = -ENOMEM;
		goto exit2;
	}

	test_and_set_bit(SCENE_CHG, &(chip->warning_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->warning_fex->scenes));

	chip->warning_fex->valid = true;
	pr_info("warning_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [soc fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM soc_ids[] = {
	ID_BIT__MAX,
	ID_BYTE__CHGT, ID_BYTE__FG_DELTA_DIFF,
	ID_INT__STAY_SEC, ID_INT__FG_VCELL1, ID_INT__FG_VCELL2, ID_MARK__FBON,
	ID_BOTH__STEP_SOC, ID_BOTH__STEP_VBAT, ID_BOTH__STEP_COUL, ID_BOTH__MAS_1P,
	ID_COMB__STEP_IBAT, ID_COMB__STEP_TBAT, ID_COMB__STEP_TMBOARD
};
static int soc_ids_size = ARRAY_SIZE(soc_ids);

static const RUN_INFO_INDEX_ENUM aging_ids[] = {
	ID_INT__CYCLE, ID_INT__FCC, ID_INT__SOH
};
static int aging_ids_size = ARRAY_SIZE(aging_ids);

struct soc_context {
	bool full_lifecycle;
	bool usb_present;
	int soc;
	int deviate_cnt;
	int never_full_cnt;
	int drop_cnt;
	uint32_t start_time;
	int once_cnt;
};
static void soc_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct soc_context *context = fex->private;

	bool status = false, soc_change = false;
	unsigned long code = 0;
	int stay_sec = 0, mas_c = 0, delta_c = 0;
	uint32_t now = 0;
	struct both_int *b_soc = NULL, *b_vbat = NULL, *b_coul = NULL;
	struct comb_int *c_ibat = NULL;
	void *p = NULL;

	if (!fex->valid) {
		pr_info("soc_fex not valid\n");
		return;
	}

	now = jiffies_to_msecs(jiffies);
	switch (scene) {
	case SCENE_SOC_CHANGE:
		soc_change = true;
	case SCENE_10STHD:
		p = run_pval(ID_BOTH__STEP_SOC);
		if (IS_ERR(p)) {
			pr_info("ID_BOTH__STEP_SOC not valid\n");
			return;
		}
		b_soc = (struct both_int *)p;

		p = run_pval(ID_BOTH__STEP_VBAT);
		if (IS_ERR(p)) {
			pr_info("ID_BOTH__STEP_VBAT not valid\n");
			return;
		}
		b_vbat = (struct both_int *)p;

		p = run_pval(ID_BOTH__STEP_COUL);
		if (IS_ERR(p)) {
			pr_info("ID_BOTH__STEP_COUL not valid\n");
			return;
		}
		b_coul = (struct both_int *)p;

		p = run_pval(ID_COMB__STEP_IBAT);
		if (IS_ERR(p)) {
			pr_info("ID_COMB__STEP_IBAT not valid\n");
			return;
		}
		c_ibat = (struct comb_int *)p;

		stay_sec = fuelsummary_get_stay_sec(now, context->start_time);
		fuelsummary_collect_value(ID_INT__STAY_SEC, stay_sec);
		mas_c = c_ibat->avg_val * stay_sec;
		if (chip->coul_support == COUL_NORMAL)
			delta_c = (b_coul->cur_val - b_coul->start_val) * 3600;
		else if (chip->coul_support == COUL_NEGATIVE)
			delta_c = (b_coul->start_val - b_coul->cur_val) * 3600;

		if (!test_bit(SOC_DEVIATE, &(fex->code)) && !chip->usb_present && !context->usb_present) {
			if ((run_val(ID_BYTE__FG_DELTA_DIFF) >= DEVIATE_INTERVAL) ||
					(chip->soc < 30 && b_vbat->cur_val > 3800 && c_ibat->cur_val > -500) ||
					(chip->soc > 90 && b_vbat->cur_val < 4000 && c_ibat->cur_val < 500)) {
				context->deviate_cnt++;
			} else {
				context->deviate_cnt = 0;
			}

			if (context->deviate_cnt > FAULT_CUMULATIVE)
				fuelsummary_set_exbit(&(fex->ex_code), SOC_DEVIATE, true);
		}

		if (!test_bit(SOC_NEVER_FULL, &(fex->code)) && chip->usb_present && context->usb_present) {
			if (run_val(ID_BIT__TERM) && chip->soc < 96) {
				context->never_full_cnt++;
			} else {
				context->never_full_cnt = 0;
			}

			if (context->never_full_cnt > FAULT_CUMULATIVE)
				fuelsummary_set_exbit(&(fex->ex_code), SOC_NEVER_FULL, true);
		}

		status = ((b_soc->start_val != 100) && (b_soc->start_val == b_soc->cur_val) && (abs(mas_c) > chip->mas_1p_max));
		if (status) {
			if (c_ibat->cur_val < 0 && chip->usb_present && context->usb_present) {
				if (chip->coul_support && delta_c > 0)
					status = (status && (delta_c > chip->mas_1p_max));
				fuelsummary_set_exbit(&(fex->ex_code), SOC_CLIMB, status);
			} else if (c_ibat->cur_val > 0 && !chip->usb_present && !context->usb_present) {
				if (chip->coul_support && delta_c < 0)
					status = (status && (abs(delta_c) > chip->mas_1p_max));
				fuelsummary_set_exbit(&(fex->ex_code), SOC_HANG, status);
			}
		}

		if (soc_change) {
			if (abs(b_soc->start_val - b_soc->cur_val) > 1 && run_val(ID_MARK__FBON))
				fuelsummary_set_exbit(&(fex->ex_code), SOC_HOP, true);

			if (context->full_lifecycle) {
				status = (b_soc->cur_val < b_soc->start_val);
				if (c_ibat->cur_val > 0)
					status = (status && (abs(mas_c) < chip->mas_1p_min));
				if (chip->coul_support && delta_c < 0)
					status = (status && (abs(delta_c) < chip->mas_1p_min));

				if (status) {
					context->drop_cnt++;
					if (context->drop_cnt >= 3 || (b_soc->start_val == 100 && stay_sec <= 60)) {
						context->drop_cnt = 0;
						fuelsummary_set_exbit(&(fex->ex_code), SOC_DROP, status);
						if (stay_sec <= 10) {
							fuelsummary_set_exbit(&(fex->ex_code), SOC_FAST, true);
						}
					}
				} else {
					context->drop_cnt = 0;
				}
			}

			if (chip->cycle_count < run_val(ID_INT__CYCLE)) {
				chip->cycle_count = run_val(ID_INT__CYCLE);
				code = (EX_SOC | (1<<SOC_CYCLE));
				pr_info("trigger aging\n");
				fuelsummary_save_run_info(chip, code, aging_ids, aging_ids_size);
			}
#if DUAL_BAT_CELL
			if (context->once_cnt >= 5 || chip->soc <= 5) {
				context->once_cnt = 1;
				code = (EX_SOC | (1 << SOC_VCELL_BALANCE));
				pr_info("trigger vcell balance\n");
				fuelsummary_save_run_info(chip, code, soc_ids, soc_ids_size);
			} else {
				context->once_cnt++;
			}
#endif
		}

		code = fex->code;
		fex->code |= fex->ex_code;
		if (fex->code > code) {
			pr_info("trigger ex_soc\n");
			fuelsummary_save_run_info(chip, fex->ex_code, soc_ids, soc_ids_size);
		}

		if (soc_change) {
			context->full_lifecycle = true;
			context->usb_present = chip->usb_present;
			run_reset(ID_BOTH__STEP_SOC);
			run_reset(ID_BOTH__STEP_VBAT);
			run_reset(ID_BOTH__STEP_COUL);
			run_reset(ID_COMB__STEP_IBAT);
			run_reset(ID_COMB__STEP_TBAT);
			run_reset(ID_COMB__STEP_TMBOARD);
			context->deviate_cnt = 0;
			context->never_full_cnt = 0;
			context->start_time = now;

			fex->ex_code = EX_SOC;
			if (context->soc == 100 || context->soc == 1)
				fex->code = EX_SOC;
			else
				fex->code &= (EX_SOC | (1<<SOC_DEVIATE) | (1<<SOC_NEVER_FULL));
		}
		break;
	case SCENE_INIT:
	case SCENE_USBIN:
	case SCENE_USBOUT:
	case SCENE_SUSPEND:
	case SCENE_RESUME:
		if (scene == SCENE_USBOUT && context->soc == 100)
			context->full_lifecycle = true;
		else
			context->full_lifecycle = false;
		context->usb_present = chip->usb_present;
		run_reset(ID_BOTH__STEP_SOC);
		run_reset(ID_BOTH__STEP_VBAT);
		run_reset(ID_BOTH__STEP_COUL);
		run_reset(ID_COMB__STEP_IBAT);
		run_reset(ID_COMB__STEP_TBAT);
		run_reset(ID_COMB__STEP_TMBOARD);
		context->deviate_cnt = 0;
		context->never_full_cnt = 0;
		context->start_time = now;
		context->once_cnt = 1;

		fex->ex_code = EX_SOC;
		if (context->soc == 100 || context->soc == 1)
			fex->code = EX_SOC;
		else
			fex->code &= (EX_SOC | (1<<SOC_DEVIATE) | (1<<SOC_NEVER_FULL));
		break;
	default:
		break;
	}
}
static int soc_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct soc_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("soc_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->soc_fex = fuelsummary_fex_create(EX_SOC, context, soc_fex_analyze);
	if (!chip->soc_fex) {
		pr_info("soc_fex alloc fail\n");
		ret = -ENOMEM;
		goto exit2;
	}

	test_and_set_bit(SCENE_10STHD, &(chip->soc_fex->scenes));
	test_and_set_bit(SCENE_USBIN, &(chip->soc_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->soc_fex->scenes));
	//test_and_set_bit(SCENE_SUSPEND, &(chip->soc_fex->scenes));
	test_and_set_bit(SCENE_RESUME, &(chip->soc_fex->scenes));
	test_and_set_bit(SCENE_SOC_CHANGE, &(chip->soc_fex->scenes));

	chip->soc_fex->valid = true;
	pr_info("soc_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [chg fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM chg_ids[] = {
	ID_BIT__MAX,

	ID_BYTE__INPUT_SCALE, ID_BYTE__INBAT_SCALE, ID_BYTE__CHGT, ID_BYTE__CAB_ID,
	ID_BYTE__SAFE_REG, ID_BYTE__CAM_VDROP, ID_BYTE__IRQA, ID_BYTE__IRQB,
	ID_BYTE__IRQC, ID_BYTE__IRQD, ID_BYTE__IRQ4, ID_BYTE__IRQ5,
	ID_BYTE__FC_REV_M, ID_BYTE__FC_REV_S,
	ID_BYTE__FC_BL_REV, ID_BYTE__FC_APP_REV, ID_BYTE__FC_BYPASS, ID_BYTE__FC_IMBALANCE,
	ID_BYTE__FC_BOTH_FAULT, ID_BYTE__FC_SLAVE_FAULT,
	ID_BYTE__ADPT_STAT, ID_BYTE__ADPT_POWER, ID_BYTE__ADPT_MCU_VER, ID_BYTE__ADPT_VENDOR,

	ID_INT__CHG_TIME, ID_INT__CHG_TCOST, ID_INT__FG_VCELL1, ID_INT__FG_VCELL2,
	ID_INT__FG_COUT_CNT, ID_MARK__FBON,

	ID_BOTH__SOC, ID_BOTH__VBAT, ID_BOTH__VCHG, ID_BOTH__COUL,

	ID_COMB__ISYS, ID_COMB__IBAT, ID_COMB__TBAT, ID_COMB__TMBOARD, ID_COMB__TSBOARD,
	ID_COMB__FC_IBUS, ID_COMB__FC_REQ_IBUS, ID_COMB__FC_VBAT, ID_COMB__FC_VBUS,
	ID_COMB__FC_CAB_MOHM, ID_COMB__FC_TADPT, ID_COMB__FC_TADPT_CONN, ID_COMB__FC_TUSB,
	ID_COMB__FC_TDIE, ID_COMB__FC_TBAT_CONN, ID_COMB__FC_MTBAT_CONN, ID_COMB__FC_TBAT_BOARD,
	ID_COMB__FC_VDP, ID_COMB__FC_VDM
};
static int chg_ids_size = ARRAY_SIZE(chg_ids);

static const RUN_INFO_INDEX_ENUM hotplug_ids[] = {
	ID_BYTE__CHGT, ID_INT__PLUG_SEC,
	ID_BOTH__SOC, ID_BOTH__VCHG, ID_BOTH__VBAT,
	ID_COMB__TBAT
};
static int hotplug_ids_size = ARRAY_SIZE(hotplug_ids);

struct chg_context {
	bool ready;
	int msoc;
	int chg_type;
	int rerun_cnt;
	int hot_plug_cnt;
	uint32_t hot_time;
};
static void chg_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	struct chg_context *context = fex->private;

	bool status = false, fchg = false, term = false;
	unsigned long code;
	uint8_t chg_type = 0;
	uint32_t now = 0, plug_sec = 0;
	struct both_int *b_soc = NULL;
	struct comb_int *c_ibat = NULL;
	void *p = NULL;

	if (!fex->valid) {
		pr_info("warning_fex not valid\n");
		return;
	}

	now = jiffies_to_msecs(jiffies);
	fchg = (run_val(ID_BIT__FCHG) != 0);
	term = (run_val(ID_BIT__TERM) != 0);
	chg_type = run_val(ID_BYTE__CHGT);

	p = run_pval(ID_BOTH__SOC);
	if (IS_ERR(p)) {
		pr_err("ID_BOTH__SOC not valid\n");
		return;
	}
	b_soc = (struct both_int *)p;

	p = run_pval(ID_COMB__IBAT);
	if (IS_ERR(p)) {
		pr_err("ID_COMB__IBAT not valid\n");
		return;
	}
	c_ibat = (struct comb_int *)p;
	switch (scene) {
	case SCENE_INIT:
		fex->code = fex->ex_code = code = EX_CHG;
		if (chip->usb_present) {
			context->msoc = chip->soc;
			run_reset(ID_BOTH__SOC);
			run_reset(ID_BOTH__VBAT);
			run_reset(ID_BOTH__VCHG);
			run_reset(ID_BOTH__COUL);
			run_reset(ID_COMB__ISYS);
			run_reset(ID_COMB__IBAT);
			run_reset(ID_COMB__TBAT);
			run_reset(ID_COMB__TMBOARD);
		}
		break;
	case SCENE_CHG:
		if (!term && b_soc->cur_val < 100)
			fuelsummary_collect_value(ID_INT__CHG_TCOST, info->chg_time);

		if (!context->ready && (fchg || info->chg_time >= READY_TIME)) {
			context->ready = true;
			context->chg_type = chg_type;
			context->rerun_cnt = 0;
		}

		if (context->ready) {
			if (context->msoc < chip->soc)
				context->msoc = chip->soc;

			if (context->chg_type != chg_type) {
				context->chg_type = chg_type;
				context->rerun_cnt++;
				if (context->rerun_cnt > 3)
					fuelsummary_set_exbit(&(fex->ex_code), CHG_RERUN_ERR, true);
			}
		}
		break;
	case SCENE_USBIN:
		fex->code = fex->ex_code = code = EX_CHG;
		context->ready = false;
		context->msoc = chip->soc;
		run_reset(ID_BOTH__SOC);
		run_reset(ID_BOTH__VBAT);
		run_reset(ID_BOTH__VCHG);
		run_reset(ID_BOTH__COUL);
		run_reset(ID_COMB__ISYS);
		run_reset(ID_COMB__IBAT);
		run_reset(ID_COMB__TBAT);
		run_reset(ID_COMB__TMBOARD);

		plug_sec = fuelsummary_get_stay_sec(now, info->unplug_time);
		fuelsummary_collect_value(ID_INT__PLUG_SEC, plug_sec);
		if (plug_sec >= 0 && plug_sec <= 5) {
			if (context->hot_plug_cnt++ > FAULT_CUMULATIVE && context->hot_time != 0 &&
					fuelsummary_get_stay_sec(now, context->hot_time) >= HOT_INTERVAL) {
				context->hot_plug_cnt = 0;
				context->hot_time = now;
				fuelsummary_set_exbit(&code, CHG_HOT_PLUG, true);
				fuelsummary_save_run_info(chip, code, hotplug_ids, hotplug_ids_size);
			}
		} else {
			context->hot_plug_cnt = 0;
		}
		break;
	case SCENE_USBOUT:
		if (fuelsummary_get_stay_sec(now, info->insert_time) > 5)
			context->hot_plug_cnt = 0;

		if (context->ready) {
			context->ready = false;
			status = (info->chg_time >= READY_TIME);
			fuelsummary_set_exbit(&(fex->ex_code), CHG_OUT, status);

			if (context->chg_type != TYPE_USB) {
				status = (info->chg_time >= STIME && (context->msoc < 100 && !term));
				fuelsummary_set_exbit(&(fex->ex_code), CHG_SLOW, status);

				status = (info->chg_time >= VALID_TIME && c_ibat->avg_val > ISLOW && b_soc->start_val <= SNCHG_P);
				fuelsummary_set_exbit(&(fex->ex_code), CHG_WEAK, status);
			}

			code = fex->code;
			fex->code |= fex->ex_code;
			if (fex->code > code) {
				pr_info("trigger ex_chg\n");
				fuelsummary_save_run_info(chip, fex->ex_code, chg_ids, chg_ids_size);
			}
		}
		break;
	default:
		break;
	}
}
static int chg_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct chg_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("chg_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->chg_fex = fuelsummary_fex_create(EX_CHG, context, chg_fex_analyze);
	if (!chip->chg_fex) {
		pr_info("chg_fex alloc fail\n");
		ret = -ENOMEM;
		goto exit2;
	}

	test_and_set_bit(SCENE_CHG, &(chip->chg_fex->scenes));
	test_and_set_bit(SCENE_USBIN, &(chip->chg_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->chg_fex->scenes));

	chip->chg_fex->valid = true;
	pr_info("chg_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [chgic fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM chgic_ids[] = {
	ID_BIT__MAX, ID_BYTE__BATID, ID_BYTE__MIC_VENDOR, ID_BYTE__SIC_VENDOR,
	ID_BYTE__INPUT_SCALE, ID_BYTE__INBAT_SCALE, ID_BYTE__CHGT, 
	ID_INT__CHG_TIME, ID_INT__CHG_TCOST, ID_INT__BACKFLOW_CNT,
	ID_INT__BACKFLOW_ONE_ROUND, ID_INT__ICO,
	ID_BOTH__SOC, ID_BOTH__VBAT, ID_BOTH__VCHG, ID_BOTH__COUL,
	ID_COMB__IBAT, ID_COMB__TBAT, ID_COMB__TMBOARD, ID_COMB__TSBOARD
};
static int chgic_ids_size = ARRAY_SIZE(chgic_ids);

struct chgic_context {
	bool ready;
	int m_i2c_cnt;
	int m_dpm_cnt;
	int m_fault_cnt;
	int s_i2c_cnt;
	int s_dpm_cnt;
	int s_fault_cnt;
	int safe_reg_cnt;
	int boost_invert_cnt;
	int boost_action_cnt;
	int once_cnt;
};
static void chgic_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_info *info = &(chip->info);
	struct chgic_context *context = fex->private;

	bool status = false, chg_enable = false, safe_reg_err = false;
	bool mi2c_err = false, si2c_err = false, mi2c_dpm = false, si2c_dpm = false, mi2c_fault = false, si2c_fault = false;
	int backflow_cnt = 0, backflow_one_round = 0;

	if (!fex->valid) {
		pr_info("chgic_fex not valid\n");
		return;
	}

	backflow_cnt = run_val(ID_INT__BACKFLOW_CNT);
	backflow_one_round = run_val(ID_INT__BACKFLOW_ONE_ROUND);
	switch (scene) {
	case SCENE_INIT:
		fex->code = fex->ex_code = EX_CHGIC;
		context->ready = false;
		context->once_cnt = 1;

		fuelsummary_set_exbit(&(fex->ex_code), CHGIC_I2C_VENDOR, true);
		fuelsummary_save_run_info(chip, fex->ex_code, chgic_ids, chgic_ids_size);
		fex->ex_code = EX_CHGIC;
		break;
	case SCENE_10STHD:
		if (context->once_cnt >= FAULT_CUMULATIVE) {
			context->once_cnt = 1;
			if ((backflow_cnt > context->boost_invert_cnt) || (backflow_one_round > context->boost_action_cnt)) {
				pr_info("trigger ex_chgic boost_invert\n");
				fuelsummary_set_exbit(&(fex->code), CHGIC_BOOST_INVERT_ERR, true);
				context->boost_invert_cnt = backflow_cnt;
				context->boost_action_cnt = backflow_one_round;
				fuelsummary_save_run_info(chip, fex->code, chgic_ids, chgic_ids_size);
			}
		} else {
			context->once_cnt++;
		}
		break;
	case SCENE_CHG:
		if (!context->ready && info->chg_time >= READY_TIME) {
			context->ready = true;
			context->m_i2c_cnt = 0;
			context->m_dpm_cnt = 0;
			context->m_fault_cnt = 0;
			context->s_i2c_cnt = 0;
			context->s_dpm_cnt = 0;
			context->s_fault_cnt = 0;
		}
		if (context->ready) {
			chg_enable = (run_val(ID_BIT__FCHG) || ((run_val(ID_BYTE__INPUT_SCALE) >= 50) && (run_val(ID_BYTE__INBAT_SCALE) >= 50)));
			safe_reg_err = (run_val(ID_BYTE__SAFE_REG) < 0x7a);
			mi2c_err = (run_val(ID_BIT__M_I2C_ERR) != 0);
			si2c_err = (run_val(ID_BIT__S_I2C_ERR) != 0);
			mi2c_dpm = (run_val(ID_BIT__M_DPM) != 0);
			si2c_dpm = (run_val(ID_BIT__S_DPM) != 0);
			mi2c_fault = (run_val(ID_BIT__M_FAULT) != 0);
			si2c_fault = (run_val(ID_BIT__S_FAULT) != 0);

			if (mi2c_err && context->m_i2c_cnt < INT_MAX)
				context->m_i2c_cnt++;

			if (chg_enable && mi2c_dpm && context->m_dpm_cnt < INT_MAX)
				context->m_dpm_cnt++;

			if (chg_enable && mi2c_fault && context->m_fault_cnt < INT_MAX)
				context->m_fault_cnt++;

			if (si2c_err && context->s_i2c_cnt < INT_MAX)
				context->s_i2c_cnt++;

			if (chg_enable && si2c_dpm && context->s_dpm_cnt < INT_MAX)
				context->s_dpm_cnt++;

			if (chg_enable && si2c_fault && context->s_fault_cnt < INT_MAX)
				context->s_fault_cnt++;

			if (safe_reg_err && context->safe_reg_cnt < 128)
				context->safe_reg_cnt++;
		}
		break;
	case SCENE_USBOUT:
		if (context->ready) {
			status = (context->m_i2c_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_M_I2C_ERR, status);

			status = (context->m_dpm_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_M_DPM, status);

			status = (context->m_fault_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_M_FAULT, status);

			status = (context->s_i2c_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_S_I2C_ERR, status);

			status = (context->s_dpm_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_S_DPM, status);

			status = (context->s_fault_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_S_FAULT, status);

			status = (context->safe_reg_cnt >= FAULT_CUMULATIVE);
			fuelsummary_set_exbit(&(fex->ex_code), CHGIC_SAFE_REG_ERR, status);
		}

		if (fex->ex_code > EX_CHGIC) {
			pr_info("trigger ex_chgic\n");
			fuelsummary_save_run_info(chip,fex->ex_code, chgic_ids, chgic_ids_size);
		}
		fex->code = fex->ex_code = EX_CHGIC;
		context->ready = false;
		break;
	default:
		break;
	}
}
static int chgic_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct chgic_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("chgic_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->chgic_fex = fuelsummary_fex_create(EX_CHGIC, context, chgic_fex_analyze);
	if (!chip->chgic_fex) {
		pr_info("chgic_fex alloc fail\n");
		ret = -ENOMEM;
		goto exit2;
	}

	test_and_set_bit(SCENE_10STHD, &(chip->chgic_fex->scenes));
	test_and_set_bit(SCENE_CHG, &(chip->chgic_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->chgic_fex->scenes));

	chip->chgic_fex->valid = true;
	pr_info("chgic_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [fg fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM fg_ids[] = {
	ID_BIT__MAX, ID_BYTE__FG_STATE, ID_BYTE__FG_EX_STEP,
	ID_BYTE__FG_BAT_FACTORY, ID_BYTE__FG_DEV_NUM, ID_BYTE__FG_VER,
	ID_BYTE__FG_FW_VER, ID_BYTE__FG_CLK_STAT, ID_BYTE__FG_SDA_STAT,
	ID_BYTE__FG_UPDATE_STAT, ID_BYTE__FG_ERR_CODE, ID_BYTE__FG_DELTA_DIFF,
	ID_INT__FG_BALANCE_CNT, ID_INT__FG_FCC_RST_CNT, ID_INT__FG_WDT_RST_CNT, ID_INT__FG_IT_RST_CNT,
	ID_INT__FG_CHG_RST_CNT, ID_INT__FG_DISCHG_RST_CNT, ID_INT__FG_LOADER_RST_CNT,
	ID_BOTH__SOC, ID_BOTH__VBAT, ID_BOTH__COUL, ID_COMB__IBAT, ID_COMB__TBAT
};
static int fg_ids_size = ARRAY_SIZE(fg_ids);
#if 0
static const RUN_INFO_INDEX_ENUM fg_interdump_ids[] = {
	ID_BIT__MAX, ID_BYTE__FG_BAT_FACTORY, ID_BYTE__FG_DEV_NUM,
	ID_BYTE__FG_VER, ID_BYTE__FG_FW_VER, ID_BUFF__FG_INTERVAL
};
static int fg_interdump_ids_size = ARRAY_SIZE(fg_interdump_ids);

static const RUN_INFO_INDEX_ENUM fg_dischgdump_ids[] = {
	ID_BIT__MAX, ID_BYTE__FG_BAT_FACTORY, ID_BYTE__FG_DEV_NUM,
	ID_BYTE__FG_VER, ID_BYTE__FG_FW_VER, ID_BUFF__FG_DISCHG_RST
};
static int fg_dischgdump_ids_size = ARRAY_SIZE(fg_dischgdump_ids);

static const RUN_INFO_INDEX_ENUM fg_chgdump_ids[] = {
	ID_BIT__MAX, ID_BYTE__FG_BAT_FACTORY, ID_BYTE__FG_DEV_NUM,
	ID_BYTE__FG_VER, ID_BYTE__FG_FW_VER, ID_BUFF__FG_CHG_RST
};
static int fg_chgdump_ids_size = ARRAY_SIZE(fg_chgdump_ids);
#endif
struct fg_context {
	bool i2c_err;
	bool ffc_ovp;
	bool checksum;
	uint8_t i2c_step;
	int ffc_cout;
	int bat_balance;
	int fcc_rst_cnt;
	int wdt_rst_cnt;
	int it_rst_cnt;
	int chg_rst_cnt;
	int dischg_rst_cnt;
	int loader_rst_cnt;
	int once_cnt;
};
static void fg_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct fg_context *context = fex->private;

	bool status = false, i2c_err = false, ffc_ovp = false, checksum = false;
	int cout_cnt = 0, balance_cnt = 0;
	int fcc_rst_cnt = 0, wdt_rst_cnt = 0, it_rst_cnt = 0, chg_rst_cnt = 0, dischg_rst_cnt = 0, loader_rst_cnt = 0;
	uint8_t err_code = 0x0, ex_step = 0x0;
	unsigned long fg_state;

	if (!fex->valid) {
		pr_info("fg_fex not valid\n");
		return;
	}

	fex->ex_code = EX_FG;
	switch (scene) {
	case SCENE_INIT:
		context->once_cnt = 1;
#if FG_EX
		fuelsummary_set_exbit(&(fex->ex_code), FG_VENDOR, true);
		fuelsummary_save_run_info(chip, fex->ex_code, fg_ids, fg_ids_size);
#endif
		break;
	case SCENE_10STHD:
		if (context->once_cnt >= FAULT_CUMULATIVE) {
			context->once_cnt = 1;
			ex_step = run_val(ID_BYTE__FG_EX_STEP);
			err_code = run_val(ID_BYTE__FG_ERR_CODE);
			fg_state = (unsigned long)run_val(ID_BYTE__FG_STATE);

			cout_cnt = run_val(ID_INT__FG_COUT_CNT);
			balance_cnt = run_val(ID_INT__FG_BALANCE_CNT);
			fcc_rst_cnt = run_val(ID_INT__FG_FCC_RST_CNT);
			wdt_rst_cnt = run_val(ID_INT__FG_WDT_RST_CNT);
			it_rst_cnt = run_val(ID_INT__FG_IT_RST_CNT);
			chg_rst_cnt = run_val(ID_INT__FG_CHG_RST_CNT);
			dischg_rst_cnt = run_val(ID_INT__FG_DISCHG_RST_CNT);
			loader_rst_cnt = run_val(ID_INT__FG_LOADER_RST_CNT);

			i2c_err = test_bit(FG_STATE_IIC_ERR, &fg_state);
			ffc_ovp = test_bit(FG_STATE_FFC_OVP_ERR, &fg_state);
			checksum = test_bit(FG_STATE_CHECKSUM_ERR, &fg_state);

			if (!context->ffc_ovp && ffc_ovp)
				context->ffc_ovp = ffc_ovp;

			if (context->ffc_cout > cout_cnt)
				context->ffc_cout = cout_cnt;

			status = (context->bat_balance < balance_cnt);
			fuelsummary_set_exbit(&(fex->ex_code), FG_BALANCE, status);

			status = err_code && ((context->i2c_err != i2c_err) || (i2c_err && context->i2c_step != ex_step));
			fuelsummary_set_exbit(&(fex->ex_code), FG_I2C_ERR, status);

			status = ((context->checksum != checksum) && checksum);
			fuelsummary_set_exbit(&(fex->ex_code), FG_CHECKSUM_ERR, status);

			status = (fcc_rst_cnt > context->wdt_rst_cnt ||
					wdt_rst_cnt > context->wdt_rst_cnt ||
					it_rst_cnt > context->it_rst_cnt ||
					chg_rst_cnt > context->chg_rst_cnt ||
					dischg_rst_cnt > context->dischg_rst_cnt);
			fuelsummary_set_exbit(&(fex->ex_code), FG_RESET, status);

			status = (loader_rst_cnt > context->loader_rst_cnt);
			fuelsummary_set_exbit(&(fex->ex_code), FG_LOADER_SW, status);

			context->bat_balance = balance_cnt;
			context->i2c_err = i2c_err;
			context->i2c_step = ex_step;
			context->checksum = checksum;
			context->fcc_rst_cnt = fcc_rst_cnt;
			context->wdt_rst_cnt = wdt_rst_cnt;
			context->it_rst_cnt = it_rst_cnt;
			context->chg_rst_cnt = chg_rst_cnt;
			context->dischg_rst_cnt = dischg_rst_cnt;
			context->loader_rst_cnt = loader_rst_cnt;
			if (fex->ex_code > EX_FG) {
				pr_info("trigger ex_fg\n");
				fuelsummary_save_run_info(chip, fex->ex_code, fg_ids, fg_ids_size);
			}
		} else {
			context->once_cnt++;
		}
		break;
	case SCENE_USBOUT:
		status = context->ffc_ovp;
		context->ffc_ovp = false;
		fuelsummary_set_exbit(&(fex->ex_code), FG_FFC_OVP_ERR, status);

		status = (context->ffc_cout >= FAULT_CUMULATIVE);
		context->ffc_cout = 0;
		fuelsummary_set_exbit(&(fex->ex_code), FG_FFC_COUT_ERR, status);
		if (fex->ex_code > EX_FG) {
			pr_info("trigger chg ex_fg\n");
			fuelsummary_save_run_info(chip, fex->ex_code, fg_ids, fg_ids_size);
		}
		break;
	default:
		break;
	}
}
static int fg_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct fg_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("fg_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->fg_fex = fuelsummary_fex_create(EX_FG, context, fg_fex_analyze);
	if (!chip->fg_fex) {
		pr_info("fg_fex alloc fail\n");
		ret = -ENOMEM;
		goto exit2;
	}

#if FG_EX
	test_and_set_bit(SCENE_10STHD, &(chip->fg_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->fg_fex->scenes));

	chip->fg_fex->valid = true;
#endif
	pr_info("fg_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [other fex]
 *
 ***********************************************************/
static const RUN_INFO_INDEX_ENUM other_ids[] = {
	ID_BIT__MAX, ID_INT__OTG_CNT, ID_INT__VBCL_ERR_CNT, ID_INT__LCM_MOS_ERR_CNT,
	ID_BOTH__SOC, ID_BOTH__VBAT, ID_COMB__IBAT, ID_COMB__TBAT
};
static int other_ids_size = ARRAY_SIZE(other_ids);

struct other_context {
	bool otg_use_1hour;
	bool otg_use_2hour;
	bool otg_use_3hour;
	bool balance_chg;
	bool vbus_smpl;
	bool cc_err;
	bool vbus_pull;
	int otg_cnt;
	int vbcl_err_cnt;
	int lcm_mos_err_cnt;
	int once_cnt;
};
static void other_fex_analyze(struct fuelsummary_ex *fex, SCENE_ENUM scene)
{
	struct fuelsummary_chip *chip = &fs_chip;
	struct other_context *context = fex->private;

	bool status = false, balance_chg = false, vbus_smpl = false, cc_err = false, vbus_pull = false;
	int otg_cnt = 0, vbcl_err_cnt = 0, lcm_mos_err_cnt = 0;

	if (!fex->valid) {
		pr_info("other_fex not valid\n");
		return;
	}

	fex->ex_code = EX_OTHER;
	switch (scene) {
	case SCENE_INIT:
		context->once_cnt = 1;
		break;
	case SCENE_10STHD:
		if (context->once_cnt >= FAULT_CUMULATIVE) {
			context->once_cnt = 1;
			balance_chg = (run_val(ID_BIT__BALANCE_CHG) != 0);
			vbus_smpl = (run_val(ID_BIT__VBUS_SMPL) != 0);
			cc_err = (run_val(ID_BIT__CC_ERR) != 0);
			vbus_pull = (run_val(ID_BIT__VBUS_PULL) != 0);
			otg_cnt = run_val(ID_INT__OTG_CNT);
			vbcl_err_cnt = run_val(ID_INT__VBCL_ERR_CNT);
			lcm_mos_err_cnt = run_val(ID_INT__LCM_MOS_ERR_CNT);

			if (otg_cnt < 360) {
				context->otg_use_1hour = false;
				context->otg_use_2hour = false;
				context->otg_use_3hour = false;
				if (otg_cnt == 1) {
					pr_info("trigger otg_using once\n");
					fuelsummary_set_exbit(&(fex->ex_code), OTG_USING, true);
				}
			} else {
				status = false;
				if (!context->otg_use_1hour && otg_cnt >= 360) {
					status = true;
					context->otg_use_1hour = true;
				} else if (!context->otg_use_2hour && otg_cnt >= 720) {
					status = true;
					context->otg_use_2hour = true;
				} else if (!context->otg_use_3hour && otg_cnt >= 1080) {
					status = true;
					context->otg_use_3hour = true;
				}
				fuelsummary_set_exbit(&(fex->ex_code), OTG_USING, status);
			}

			if (context->balance_chg != balance_chg) {
				context->balance_chg = balance_chg;
				fuelsummary_set_exbit(&(fex->ex_code), BALANCING_CHG, true);
			}

			if (!context->vbus_smpl && vbus_smpl)
				fuelsummary_set_exbit(&(fex->ex_code), VBUS_SMPL_DETECT, true);
			context->vbus_smpl = vbus_smpl;

			if (!context->cc_err && cc_err)
				fuelsummary_set_exbit(&(fex->ex_code), CC_ERR, true);
			context->cc_err = cc_err;

			if (!context->vbus_pull && vbus_pull)
				fuelsummary_set_exbit(&(fex->ex_code), VBUS_PULL, true);
			context->vbus_pull = vbus_pull;

			if (context->vbcl_err_cnt < vbcl_err_cnt) {
				context->vbcl_err_cnt = vbcl_err_cnt;
				fuelsummary_set_exbit(&(fex->ex_code), BCL_VADC_ERR, true);
			}

			if (context->lcm_mos_err_cnt < lcm_mos_err_cnt) {
				context->lcm_mos_err_cnt = lcm_mos_err_cnt;
				fuelsummary_set_exbit(&(fex->ex_code), LCM_MOS_GPIO_ERR, true);
			}

			if (fex->ex_code > EX_OTHER) {
				pr_info("trigger other ex_other\n");
				fuelsummary_save_run_info(chip, fex->ex_code, other_ids, other_ids_size);
			}
		} else {
			context->once_cnt++;
		}
		break;
	case SCENE_USBOUT:
		break;
	default:
		break;
	}
}
static int other_fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct other_context *context = kzalloc(sizeof(*context), GFP_KERNEL);

	if (!context) {
		pr_info("other_context alloc fail\n");
		ret = -ENOMEM;
		goto exit1;
	}

	chip->other_fex = fuelsummary_fex_create(EX_OTHER, context, other_fex_analyze);
	if (!chip->other_fex) {
		pr_info("other_fex alloc fail\n");
		ret = -ENOMEM;
		goto exit2;
	}

	test_and_set_bit(SCENE_10STHD, &(chip->other_fex->scenes));
	test_and_set_bit(SCENE_USBOUT, &(chip->other_fex->scenes));

	chip->other_fex->valid = true;
	pr_info("other_fex_init success\n");
	return ret;
exit2:
	kfree(context);
exit1:
	return ret;
}
/************************************************************
 *
 *  [Fuelsummary Exception init]
 *
 ***********************************************************/
static int fex_init(struct fuelsummary_chip *chip)
{
	int ret = 0;

	if (!chip->fex_list) {
		ret = fex_list_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->warning_fex) {
		ret = warning_fex_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->soc_fex) {
		ret = soc_fex_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->chg_fex) {
		ret = chg_fex_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->chgic_fex) {
		ret = chgic_fex_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->fg_fex) {
		ret = fg_fex_init(chip);
		if (ret)
			goto exit;
	}
	if (!chip->other_fex) {
		ret = other_fex_init(chip);
		if (ret)
			goto exit;
	}
exit:
	return ret;
}
/************************************************************
 *
 *  [Fuelsummary local func]
 *
 ***********************************************************/
static void reset_roll_current(struct fuelsummary_chip *chip)
{
	struct current_roll *cr = &(chip->croll);
	cr->cur_index = 0;
	cr->max_index = 0;
	cr->min_index = 0;
	cr->sum = 0;
	cr->avg = chip->icurrent;
	cr->size = 0;
}
static void update_roll_current(struct fuelsummary_chip *chip)
{
	int i, sum, iold, ibat, imax, imin, cur_index, max_index, min_index;
	struct current_roll *cr = &(chip->croll);

	ibat = chip->icurrent;
	cur_index = cr->cur_index;
	max_index = cr->max_index;
	min_index = cr->min_index;

	iold = cr->array[cur_index];
	cr->array[cur_index] = ibat;
	if (cr->size < CURRENT_ARRAY_SIZE) {
		cr->size++;
		if (cr->size == 1) {
			cr->max_index = cr->min_index = cur_index;
		} else {
			if (cr->array[max_index] < ibat)
				cr->max_index = cur_index;

			if (cr->array[min_index] > ibat)
				cr->min_index = cur_index;
		}
		cr->sum += ibat;
		if (cr->size > 0)
			cr->avg = (cr->sum / cr->size);
	} else {
		if (cur_index == max_index) {
			imax = ibat;
			for (i = 1; i < CURRENT_ARRAY_SIZE; i++) {
				if (imax < cr->array[i]) {
					imax = cr->array[i];
					cr->max_index = i;
				}
			}
		} else {
			if (cr->array[max_index] < ibat)
				cr->max_index = cur_index;
		}

		if (cur_index == min_index) {
			imin = ibat;
			for (i = 1; i < CURRENT_ARRAY_SIZE; i++) {
				if (imin > cr->array[i]) {
					imin = cr->array[i];
					cr->min_index = i;
				}
			}
		} else {
			if (cr->array[min_index] > ibat)
				cr->min_index = cur_index;
		}

		cr->sum += (ibat - iold);
		sum = cr->sum - (cr->array[cr->max_index] + cr->array[cr->min_index]);
		cr->avg = (sum / (CURRENT_ARRAY_SIZE - 2));
	}

	cr->cur_index++;
	if (cr->cur_index >= CURRENT_ARRAY_SIZE)
		cr->cur_index = 0;
#if 0
	pr_info("size=%d, sum=%d, cur=%d, val[%d,%d,%d]\n",
			cr->size, cr->sum, ibat, cr->avg,
			cr->array[cr->max_index], cr->array[cr->min_index]);
#endif
}
static void fuelsummary_running_info(struct fuelsummary_chip *chip)
{
	int ret = 0;
	int chg_status = 0, ibat = 0, tbat = 0, vbat = 0, vchg = 0, chg_type = 0;
	int tmboard = 0, tsboard = 0, calling = 0, factory = 0, health_status = 0;
	int coulomb = 0, cycle = 0, balance_chg = 0, exhibition_mode = 0;
#if !QCOM_PLATFORM
	int fbon = 0, input_scale = 0, inbat_scale = 0;
#endif
#if 0
	//int soh = 0, fcc = 0;
	/* add for super charge */
	int capacity = 0;
	/* add for super charge */
#endif
	if (chip->usb_present) {
		ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_STATUS_EX, &chg_status);
		if (!ret) {
			if (chg_status == POWER_SUPPLY_STATUS_FULL) {
				chip->chg_term = true;
				fuelsummary_collect_value(ID_BIT__TERM, 1);
			}
		}

		ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CHG_VOLTAGE, &vchg);
		if (!ret && !chip->chg_term) {
			fuelsummary_collect_value(ID_BOTH__VCHG, vchg);
		}
#if QCOM_PLATFORM
		ret = fuelsummary_get_psy_data(USB_PSY, POWER_SUPPLY_PROP_REAL_TYPE, &chg_type);
#else
		ret = fuelsummary_get_psy_data(USB_PSY, POWER_SUPPLY_PROP_CHARGE_TYPE, &chg_type);
#endif
		if (!ret && !chip->chg_term) {
			fuelsummary_collect_value(ID_BYTE__CHGT, chg_type);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_FACTORY_MODE_STATE, &factory);
		if (!ret && !chip->chg_term) {
			if (factory)
				fuelsummary_collect_value(ID_BIT__FACTORY, 1);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_CALLING_STATE, &calling);
		if (!ret && !chip->chg_term) {
			if (calling)
				fuelsummary_collect_value(ID_BIT__CALLING, 1);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_SWITCH_STATE, &balance_chg);
		if (!ret && !chip->chg_term) {
			if (balance_chg)
				fuelsummary_collect_value(ID_BIT__BALANCE_CHG, 1);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_HEALTH_STATUS, &health_status);
		if (!ret) {
			fuelsummary_collect_value(ID_LONG__HEALTH, health_status);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_EXHIBITION_MODE, &exhibition_mode);
		if (!ret) {
			fuelsummary_collect_value(ID_BIT__EXHIBITION, exhibition_mode);
		}
#if !QCOM_PLATFORM
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_INPUT_SCALE, &input_scale);
		if (!ret) {
			fuelsummary_collect_value(ID_BYTE__INPUT_SCALE, input_scale);
		}
		ret = fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_INBAT_SCALE, &inbat_scale);
		if (!ret) {
			fuelsummary_collect_value(ID_BYTE__INBAT_SCALE, inbat_scale);
		}
#endif
	}

	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CURRENT_NOW, &ibat);
	if (!ret) {
		chip->icurrent = ibat;
		fuelsummary_collect_value(ID_COMB__STEP_IBAT, ibat);
		if (!chip->chg_term)
			fuelsummary_collect_value(ID_COMB__IBAT, ibat);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_TEMP, &tbat);
	if (!ret) {
		fuelsummary_collect_value(ID_COMB__STEP_TBAT, tbat);
		if (!chip->chg_term)
			fuelsummary_collect_value(ID_COMB__TBAT, tbat);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat);
	if (!ret) {
		fuelsummary_collect_value(ID_BOTH__STEP_VBAT, vbat);
		if (!chip->chg_term)
			fuelsummary_collect_value(ID_BOTH__VBAT, vbat);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP, &tmboard);
	if (!ret) {
		fuelsummary_collect_value(ID_COMB__STEP_TMBOARD, tmboard);
		if (!chip->chg_term)
			fuelsummary_collect_value(ID_COMB__TMBOARD, tmboard);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_PARALLEL_BOARD_TEMP, &tsboard);
	if (!ret && !chip->chg_term) {
		fuelsummary_collect_value(ID_COMB__TSBOARD, tsboard);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CHARGE_COUNTER, &coulomb);
	if (!ret) {
		fuelsummary_collect_value(ID_BOTH__STEP_COUL, coulomb);
		if (!chip->chg_term)
			fuelsummary_collect_value(ID_BOTH__COUL, coulomb);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CYCLE_COUNT, &cycle);
	if (!ret) {
		fuelsummary_collect_value(ID_INT__CYCLE, cycle);
	}
#if !QCOM_PLATFORM
	fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_FBON, &fbon);
	if (!ret) {
		fuelsummary_collect_value(ID_MARK__FBON, fbon);
	}
#endif

#if 0
/*
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_SOH, &soh);
	if (!ret) {
		fuelsummary_collect_value(ID_INT__SOH, soh);
	}
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_FCC, &fcc);
	if (!ret) {
		fuelsummary_collect_value(ID_INT__FCC, fcc);
	}
*/

	/* add for super charge */
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (!ret) {
		pr_info("soc=%d,capacity=%d\n", chip->soc, capacity);
		if (chip->soc != capacity) {
			pr_info("SCENE_10STHD detected soc change!\n");
			power_supply_changed(chip->bat_psy);
		}
	}
	/* add for super charge */
#endif
	update_roll_current(chip);
}
static void fuelsummary_boot_start(struct work_struct *work)
{
	int ret = 0, mAh = 1, usb_in = 0;
	struct fuelsummary_chip *chip = container_of(work, struct fuelsummary_chip, boot_start.work);
	struct fuel_info *info = &(chip->info);
	struct fuelsummary_ex *f;
#if QCOM_PLATFORM
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &mAh);
#else
	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CAPACITY_MAH, &mAh);
#endif
	if (ret < 0) {
		pr_err("init data err!!!!!!\n");
		mAh = 4000;
	}
	ret = fuelsummary_get_psy_data(USB_PSY, POWER_SUPPLY_PROP_PRESENT, &usb_in);
	ret += fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CAPACITY, &(chip->soc));
	ret += fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CYCLE_COUNT, &(chip->cycle_count));
	ret += fuelsummary_get_psy_data(CMS_PSY, POWER_SUPPLY_PROP_BATTERY_ID, &(chip->battery_id));
	if (ret < 0) {
		pr_err("some data init err!!!!!!\n");
	}

	chip->mah = mAh;
	chip->mas_1p_max = (mAh * 36 * COEF_1P);
	chip->mas_1p_min = (mAh * 36 / COEF_1P);
	chip->usb_present = !!usb_in;
	chip->coul_support = COUL_NORMAL;

	fuelsummary_collect_value(ID_BOTH__MAS_1P, chip->mas_1p_max);
	fuelsummary_collect_value(ID_BOTH__MAS_1P, chip->mas_1p_min);
	fuelsummary_collect_value(ID_INT__CYCLE, chip->cycle_count);
	fuelsummary_collect_value(ID_BYTE__BATID, chip->battery_id);
	fuelsummary_collect_value(ID_BOTH__SOC, chip->soc);
	fuelsummary_collect_value(ID_BOTH__STEP_SOC, chip->soc);
	fuelsummary_running_info(chip);

	if (chip->usb_present) {
		info->chg_time = 0;
		fuelsummary_collect_value(ID_INT__CHG_TIME, 0);
		fuelsummary_collect_value(ID_INT__CHG_TCOST, 0);
		info->insert_time = jiffies_to_msecs(jiffies);
	} else {
		info->unplug_time = jiffies_to_msecs(jiffies);
	}

	list_for_each_entry(f, chip->fex_list, list) {
		if (f->analyze)
			f->analyze(f, SCENE_INIT);
	}

	chip->initialized = true;
	//config_params_dump();
#if OWNER_10THD
	schedule_delayed_work(&(chip->thd_10s), msecs_to_jiffies(10000));
#endif
}
#if OWNER_10THD
static void fuelsummary_thd_10s(struct work_struct *work)
{
	struct fuelsummary_chip *chip =
		container_of(work, struct fuelsummary_chip, thd_10s.work);

	if (chip->running_10sthd && !chip->is_suspend)
		queue_work(chip->wq, &(chip->running_10sthd->fwork));

	schedule_delayed_work(&(chip->thd_10s), msecs_to_jiffies(10000));
}
#endif
static void fex_running(struct work_struct *work)
{
	int n = 0;
	struct fuelsummary_ex *f;
	struct fuelsummary_chip *chip = &fs_chip;
	struct fuel_work *fw = container_of(work, struct fuel_work, fwork);

	int scene = fw->running_scene;
	struct fuel_info *info = &(chip->info);

	if (!chip->initialized) {
		pr_info("system not ready!\n");
		if (scene == SCENE_10STHD) {
			fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CURRENT_NOW, &(chip->icurrent));
			update_roll_current(chip);
		}
		return;
	}

	pr_info("scene:%d\n", scene);
	if (scene >= SCENE_10STHD && scene <= SCENE_SOC_CHANGE) {
		if (scene == SCENE_10STHD || scene == SCENE_SOC_CHANGE) {
			fuelsummary_running_info(chip);
		} else if (scene == SCENE_USBIN) {
			fuelsummary_collect_value(ID_INT__CHG_TIME, 0);
			fuelsummary_collect_value(ID_INT__CHG_TCOST, 0);
			info->insert_time = jiffies_to_msecs(jiffies);
			fuelsummary_clear_data(chg_ids, chg_ids_size);
			fuelsummary_running_info(chip);
			reset_roll_current(chip);
		} else if (scene == SCENE_USBOUT) {
			info->unplug_time = jiffies_to_msecs(jiffies);
			reset_roll_current(chip);
		} else if (scene == SCENE_CHG) {
			info->chg_time = fuelsummary_get_stay_sec(jiffies_to_msecs(jiffies), info->insert_time);
			fuelsummary_collect_value(ID_INT__CHG_TIME, info->chg_time);
		}

		list_for_each_entry(f, chip->fex_list, list) {
			if (f->analyze && test_bit(scene, &(f->scenes)))
				f->analyze(f, scene);

			if (f->code & 0xffffff)
				n++;
		}
		chip->fex_count = n;
		if (scene == SCENE_USBOUT) {
			fuelsummary_clear_data(chg_ids, chg_ids_size);
		}
	}
}
static struct fuel_work *fuelsummary_init_fwork(int scene)
{
	struct fuel_work *fw = kzalloc(sizeof(*fw), GFP_KERNEL);

	if (!fw) {
		pr_info("couldn't alloc scene(%d) fuel_work struct\n", scene);
		return NULL;
	}

	INIT_WORK(&(fw->fwork), fex_running);
	fw->running_scene = scene;
	return fw;
}



/************************************************************
 *
 *  [Fuelsummary sysfs]
 *
 ***********************************************************/
static ssize_t fuelsummary_access_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int ret = 0, len = 0, index = 0, value = 0;
	char buf[24] = {0};
	char *pvalue = NULL;

	if (count > 24)
		len = 24;
	else
		len = count;
	ret = copy_from_user(buf, ubuf, len);

	if (len >= 3) {
		//index = buf[0];
		//value = ((buf[4]<<24) | (buf[3]<<16) | (buf[2]<<8) | buf[1]);
		index = simple_strtoul(buf, &pvalue, 10);
		value = simple_strtoul((pvalue+1), NULL, 10);

		pr_info("access:[%d,%d]\n", index, value);
		if (index >= 0 && index <= ID_LONG__HEALTH)
			fuelsummary_collect_value(index, value);
	}
	return count;
}
static ssize_t fuelsummary_access_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	size_t len = 0, outlen = 0;
	uint8_t buf[FIFO_SIZE_MAX];
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	memset(buf, 0x0, FIFO_SIZE_MAX);
	spin_lock_irq(&(chip->fuel_fifo_lock));
	if (kfifo_is_empty(&chip->fuel_fifo))
		goto exit;

	len = kfifo_len(&(chip->fuel_fifo));
	if (len > FIFO_SIZE_MAX)
		len = FIFO_SIZE_MAX;

	outlen = kfifo_out(&(chip->fuel_fifo), &buf, len);
exit:
	spin_unlock_irq(&(chip->fuel_fifo_lock));
	return sprintf(ubuf, "%s", buf);
}
static CLASS_ATTR_RW(fuelsummary_access);

static ssize_t fifotest_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int len = 0, i = 0;
	char values[24];
	uint8_t buf[INFO_SIZE_MAX];
#if FUELSUMMARY_TEST
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);
#endif
	if (copy_from_user(values, ubuf, min_t(size_t, sizeof(values), count)))
		return -EFAULT;

	if (1 == sscanf(values, "%d", &len)) {
		if (len > INFO_SIZE_MAX)
			len = INFO_SIZE_MAX;

		memset(buf, 0x0, sizeof(buf));
		for (i = 0; i < len; i++) {
			if ((i + 1) % 0x10 == 0)
				buf[i] = '\n';
			else
				buf[i] = '0';
		}
#if FUELSUMMARY_TEST
		fuelsummary_writeBigData(chip, buf);
#endif
	}
	return count;
}
static ssize_t fifotest_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	size_t len = 0, outlen = 0;
	uint8_t buf[FIFO_SIZE_MAX];
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	memset(buf, 0x0, sizeof(buf));
	if (kfifo_is_empty(&chip->fuel_fifo))
		goto exit;

	len = kfifo_len(&(chip->fuel_fifo));
	if (len > FIFO_SIZE_MAX)
		len = FIFO_SIZE_MAX;

	outlen = kfifo_out_peek(&(chip->fuel_fifo), &buf, len);
exit:
	return sprintf(ubuf, "fifo-size:%d, fifo-len:%d, full:%d\n%s",
			kfifo_size(&(chip->fuel_fifo)), kfifo_len(&(chip->fuel_fifo)), kfifo_is_full(&(chip->fuel_fifo)), buf);
}
static CLASS_ATTR_RW(fifotest);

static ssize_t user_custom_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int val = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (kstrtoint(ubuf, 10, &val)) {
		pr_err("val inval!\n");
		return -EINVAL;
	}

	if (val == 0) {
		chip->user_custom = false;
	} else if (val == 1) {
		chip->user_custom = true;
	} else if (val == 7083) {
		config_params_dump();
	} else if (val > 0xffff) {
		chip->uspace_aging = true;
		chip->uspace_cycle = (val & 0xffff);
		chip->uspace_soh = (val >> 16);
	}
	pr_info("user_custom=%llx\n", val);
	return count;
}
static ssize_t user_custom_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->user_custom);
}
static CLASS_ATTR_RW(user_custom);

static ssize_t custom_input_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int val = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (kstrtoint(ubuf, 10, &val))
		return -EINVAL;

	if (val >= 0 && val <= CUSTOM_INPUT_MAX) {
		chip->custom_input = val;
		pr_info("custom_ibat=%d\n", chip->custom_input);
	}
	return count;
}
static ssize_t custom_input_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->custom_input);
}
static CLASS_ATTR_RW(custom_input);

static ssize_t custom_ibat_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int val = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (kstrtoint(ubuf, 10, &val))
		return -EINVAL;

	if (val >= 0 && val <= CUSTOM_IBAT_MAX) {
		chip->custom_ibat = val;
		pr_info("custom_ibat=%d\n", chip->custom_ibat);
	}
	return count;
}
static ssize_t custom_ibat_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->custom_ibat);
}
static CLASS_ATTR_RW(custom_ibat);

static ssize_t custom_volt_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int val = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (kstrtoint(ubuf, 10, &val))
		return -EINVAL;

	if (val >= 0 && val <= CUSTOM_VOLT_MAX) {
		chip->custom_volt = val;
		pr_info("custom_volt=%d\n", chip->custom_volt);
	}
	return count;
}
static ssize_t custom_volt_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->custom_volt);
}
static CLASS_ATTR_RW(custom_volt);

static ssize_t fex_version_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
	int val = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (kstrtoint(ubuf, 10, &val))
		return -EINVAL;

	if (val >= 0) {
		chip->fex_version = val;
		pr_info("set fex_version=%d\n", chip->fex_version);
	}
	return count;
}
static ssize_t fex_version_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->fex_version);
}
static CLASS_ATTR_RW(fex_version);

static ssize_t fex_header_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int i = 0, len = 0;
	uint8_t buf[INFO_SIZE_MAX];

	memset(buf, 0x0, INFO_SIZE_MAX);
	len += sprintf(buf, "%lx:", EX_WARNING);
	for (i = 0; i < warning_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (warning_ids_size - 1))
				len += sprintf((buf + len), "%d,", warning_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", warning_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_SOC);
	for (i = 0; i < soc_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (soc_ids_size - 1))
				len += sprintf((buf + len), "%d,", soc_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", soc_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 40))
		len += sprintf((buf + len), "%lx:%d,%d,%d\n", (EX_SOC | (1 << SOC_CYCLE)),
				aging_ids[0], aging_ids[1], aging_ids[2]);

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_CHG);
	for (i = 0; i < chg_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (chg_ids_size - 1))
				len += sprintf((buf + len), "%d,", chg_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", chg_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 60))
		len += sprintf((buf + len), "%lx:%d,%d,%d,%d,%d,%d\n", (EX_CHG | (1 << CHG_HOT_PLUG)),
				hotplug_ids[0], hotplug_ids[1], hotplug_ids[2], hotplug_ids[3], hotplug_ids[4], hotplug_ids[4]);

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_CHGIC);
	for (i = 0; i < chgic_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (chgic_ids_size - 1))
				len += sprintf((buf + len), "%d,", chgic_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", chgic_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_FG);
	for (i = 0; i < fg_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (fg_ids_size - 1))
				len += sprintf((buf + len), "%d,", fg_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", fg_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_OTHER);
	for (i = 0; i < other_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (other_ids_size - 1))
				len += sprintf((buf + len), "%d,", other_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", other_ids[i]);
		}
	}

	return sprintf(ubuf, "%s\n", buf);
}
static CLASS_ATTR_RO(fex_header);

static ssize_t roll_current_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);
	struct current_roll *cr = &(chip->croll);

	return sprintf(ubuf, "%d,%d,%d\n", cr->avg, cr->array[cr->max_index], cr->array[cr->min_index]);
}
static CLASS_ATTR_RO(roll_current);

static ssize_t cycle_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int ret = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (chip->ag_support == DRIVER_SUPPORTED)
		ret = run_val(ID_INT__CYCLE);
	else
		ret = chip->uspace_cycle;

	if (ret < 0)
		ret = 0;

	return sprintf(ubuf, "%u\n", ret);
}
static CLASS_ATTR_RO(cycle);

static ssize_t soh_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int ret = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (chip->ag_support == DRIVER_SUPPORTED)
		ret = run_val(ID_INT__SOH) + AGING_SOH_COMPENSATE;
	else
		ret = chip->uspace_soh + AGING_SOH_COMPENSATE;

	if (ret < 0 || ret > 100)
		ret = 100;

	return sprintf(ubuf, "%u\n", ret);
}
static CLASS_ATTR_RO(soh);

static ssize_t aging_support_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int ret = 0;
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	if (chip->ag_support == DRIVER_SUPPORTED || (chip->ag_support == USPACE_SUPPORTED && chip->uspace_aging))
		ret = 1;
	return sprintf(ubuf, "%u\n", ret);
}
static CLASS_ATTR_RO(aging_support);

static ssize_t aging_count_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	return sprintf(ubuf, "%d\n", run_val(ID_INT__FCC));
}
static CLASS_ATTR_RO(aging_count);

static ssize_t chgic_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	return sprintf(ubuf, "%u,%u\n", run_val(ID_BYTE__MIC_VENDOR), run_val(ID_BYTE__SIC_VENDOR));
}
static CLASS_ATTR_RO(chgic);

static ssize_t batid_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	return sprintf(ubuf, "%u\n", run_val(ID_BYTE__BATID));
}
static CLASS_ATTR_RO(batid);

static ssize_t fgid_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	return sprintf(ubuf, "%u\n", run_val(ID_BYTE__FG_DEV_NUM));
}
static CLASS_ATTR_RO(fgid);

static ssize_t config_store(struct class *c, struct class_attribute *attr, const char *ubuf, size_t count)
{
#if 0
	int ret = 0;
#endif
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);
	struct config_work *cw = kzalloc(sizeof(*cw), GFP_KERNEL);

	if (!cw) {
		pr_err("couldn't alloc config_work struct\n");
		return -ENOMEM;
	}

	cw->params = kzalloc(count, GFP_KERNEL);
	if (!cw->params) {
		pr_err("params buffer alloc fail:%d\n", (int)count);
		kfree(cw);
		return -ENOMEM;
	}

	memcpy(cw->params, ubuf, count);
	pr_info("copy config_params from user, size=%d\n", count);
#if 0
	ret = copy_from_user(cw->params, ubuf, count);
	if (ret) {
		pr_err("failed to copy config_params from user\n");
		kfree(cw->params);
		kfree(cw);
		return -EFAULT;
	}
#endif
	if (count > CONFIG_HEADER_SIZE) {
		cw->type = (*(cw->params) & 0x3f);
		cw->chksum = *(cw->params + 1);
		cw->page = BYTE_SWAP_32(cw->params + 2);
		cw->row = BYTE_SWAP_32(cw->params + 6);
		cw->column = BYTE_SWAP_32(cw->params + 10);
		cw->name_offset = BYTE_SWAP_32(cw->params + 14);
		cw->name_size = BYTE_SWAP_32(cw->params + 18);
		cw->buff_offset = BYTE_SWAP_32(cw->params + 22);
		cw->buff_size = BYTE_SWAP_32(cw->params + 26);
		cw->total = count;

		INIT_WORK(&(cw->cwork), config_params_worker);
		queue_work(chip->wq, &(cw->cwork));
	} else {
		pr_err("buff size error\n");
		kfree(cw->params);
		kfree(cw);
	}
	return count;
}
static ssize_t config_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);

	return sprintf(ubuf, "%d\n", chip->config_cnt);
}
static CLASS_ATTR_RW(config);

static ssize_t amount_total_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int val = 0;
#if QCOM_PLATFORM
	fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_BATTERY_CAPACITY_MAH, &val);
#else
	fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CAPACITY_MAH, &val);
#endif
	return sprintf(ubuf, "%d\n", val);
}
static CLASS_ATTR_RO(amount_total);

static ssize_t age_amount_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	struct fuelsummary_chip *chip = container_of(c, struct fuelsummary_chip, fuelsummary_cls);
	int val = 0;

	if (chip->ag_support == DRIVER_SUPPORTED)
		val = chip->mah * run_val(ID_INT__SOH) / 100;
	else
		val = chip->mah * chip->uspace_soh / 100;
	return sprintf(ubuf, "%d\n", val);
}
static CLASS_ATTR_RO(age_amount);

static ssize_t amount_show(struct class *c, struct class_attribute *attr, char *ubuf)
{
	int val = 0;

	fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_FCC, &val);
	return sprintf(ubuf, "%d\n", val);
}
static CLASS_ATTR_RO(amount);

static struct attribute *fuelsummary_class_attrs[] = {
	&class_attr_fuelsummary_access.attr,
	&class_attr_fifotest.attr,
	&class_attr_user_custom.attr,
	&class_attr_custom_input.attr,
	&class_attr_custom_ibat.attr,
	&class_attr_custom_volt.attr,
	&class_attr_fex_version.attr,
	&class_attr_fex_header.attr,
	&class_attr_roll_current.attr,
	&class_attr_cycle.attr,
	&class_attr_soh.attr,
	&class_attr_aging_support.attr,
	&class_attr_aging_count.attr,
	&class_attr_chgic.attr,
	&class_attr_batid.attr,
	&class_attr_fgid.attr,
	&class_attr_config.attr,
	&class_attr_amount_total.attr,
	&class_attr_age_amount.attr,
	&class_attr_amount.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fuelsummary_class);

static int create_class_attrs(struct fuelsummary_chip *chip)
{
	int ret = 0;

	chip->fuelsummary_cls.name = "fuelsummary";
	chip->fuelsummary_cls.class_groups = fuelsummary_class_groups;
	ret = class_register(&(chip->fuelsummary_cls));
	if (ret < 0) {
		pr_err("Failed to register fuelsummary_class ret=%d\n", ret);
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t store_fuelsummary_access(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	int ret = 0, len = 0;
	int index = 0, value = 0;
	char buf[24] = {0};
	char *pvalue = NULL;

	if (count > 24)
		len = 24;
	else
		len = count;
	ret = copy_from_user(buf, ubuf, len);

	if (len >= 3) {
		//index = buf[0];
		//value = ((buf[4]<<24) | (buf[3]<<16) | (buf[2]<<8) | buf[1]);
		index = simple_strtoul(buf, &pvalue, 10);
		value = simple_strtoul((pvalue+1), NULL, 10);

		pr_info("access:[%d,%d]\n", index, value);
		if (index >= ID_BIT__TERM && index < ID_LONG__HEALTH) {
			fuelsummary_collect_value(index, value);
		}
	}
	return len;
}
static int show_fuelsummary_access(struct seq_file *m, void *data)
{
	size_t len = 0, outlen = 0;
	uint8_t buf[FIFO_SIZE_MAX];
	struct fuelsummary_chip *chip = m->private;

	memset(buf, 0x0, sizeof(buf));
	spin_lock_irq(&(chip->fuel_fifo_lock));
	if (kfifo_is_empty(&chip->fuel_fifo))
		goto exit;

	len = kfifo_len(&(chip->fuel_fifo));
	if (len > FIFO_SIZE_MAX)
		len = FIFO_SIZE_MAX;

	outlen = kfifo_out(&(chip->fuel_fifo), &buf, len);
exit:
	spin_unlock_irq(&(chip->fuel_fifo_lock));
	seq_printf(m, "%s", buf);
	return 0;
}
static int fuelsummary_access_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_fuelsummary_access, inode->i_private);
}
static const struct file_operations fuelsummary_access_fops = {
	.owner		= THIS_MODULE,
	.open		= fuelsummary_access_open,
	.write		= store_fuelsummary_access,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static ssize_t store_fifotest(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	int len = 0, i = 0;
	char values[24];
	uint8_t buf[ENTRY_LEN_MAX];
#if FUELSUMMARY_TEST
	struct seq_file *s = file->private_data;
	struct fuelsummary_chip *chip = s->private;
#endif
	if (copy_from_user(values, ubuf, min_t(size_t, sizeof(values), count)))
		return -EFAULT;

	if (1 == sscanf(values, "%d", &len)) {
		if (len > ENTRY_LEN_MAX)
			len = ENTRY_LEN_MAX;

		for (i=0; i<len; i++) {
			if ((i+1) % 0x10 == 0)
				buf[i] = '\n';
			else
				buf[i] = '0';
		}
#if FUELSUMMARY_TEST
		fuelsummary_writeBigData(chip, buf);
#endif
	}

	return len;
}
static int show_fifotest(struct seq_file *m, void *data)
{
	size_t len = 0;
	size_t outlen = 0;
	uint8_t buf[FIFO_SIZE_MAX];
	struct fuelsummary_chip *chip = m->private;

	memset(buf, 0x0, sizeof(buf));
	if (kfifo_is_empty(&chip->fuel_fifo))
		goto exit;

	len = kfifo_len(&(chip->fuel_fifo));
	if (len > FIFO_SIZE_MAX)
		len = FIFO_SIZE_MAX;

	outlen = kfifo_out_peek(&(chip->fuel_fifo), &buf, len);
exit:
	seq_printf(m, "fifo-size:%d, fifo-len:%d, full:%d\n%s",
			kfifo_size(&(chip->fuel_fifo)),
			kfifo_len(&(chip->fuel_fifo)),
			kfifo_is_full(&(chip->fuel_fifo)), buf);

	return 0;
}
static int fifotest_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_fifotest, inode->i_private);
}
static const struct file_operations fifotest_fops = {
	.owner		= THIS_MODULE,
	.open		= fifotest_open,
	.write		= store_fifotest,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int set_user_custom(void *data, u64 val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	if (val == 0) {
		chip->user_custom = false;
	} else if (val == 7083) {
		config_params_dump();
	} else if (val > 0xffff) {
		chip->uspace_aging = true;
		chip->uspace_cycle = (val & 0xffff);
		chip->uspace_soh = (val >> 16);
	} else {
		chip->user_custom = true;
	}
	pr_info("user_custom=%lld\n", val);
	return 0;
}
static int get_user_custom(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	if (chip->user_custom)
		*(u64 *)val = 1;
	else
		*(u64 *)val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(user_custom_fops,
		get_user_custom, set_user_custom, "%01llx\n");

static int set_custom_input(void *data, u64 val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (val >= 0 && val <= CUSTOM_INPUT_MAX) {
		chip->custom_input = val;
		pr_info("custom_ibat=%d\n", chip->custom_input);
	}
	return 0;
}
static int get_custom_input(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	*(u64 *)val = chip->custom_input;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(custom_input_fops,
		get_custom_input, set_custom_input, "%llu\n");

static int set_custom_ibat(void *data, u64 val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (val >= 0 && val <= CUSTOM_IBAT_MAX) {
		chip->custom_ibat = val;
		pr_info("custom_ibat=%d\n", chip->custom_ibat);
	}
	return 0;
}
static int get_custom_ibat(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	*(u64 *)val = chip->custom_ibat;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(custom_ibat_fops,
		get_custom_ibat, set_custom_ibat, "%llu\n");

static int set_custom_volt(void *data, u64 val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (val >= 0 && val <= CUSTOM_VOLT_MAX) {
		chip->custom_volt = val;
		pr_info("custom_volt=%d\n", chip->custom_volt);
	}
	return 0;
}
static int get_custom_volt(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	*(u64 *)val = chip->custom_volt;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(custom_volt_fops,
		get_custom_volt, set_custom_volt, "%llu\n");

static int set_fex_version(void *data, u64 val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	if (val >= 0)
		chip->fex_version = (int)val;

	pr_info("fex_version=%d\n", chip->fex_version);
	return 0;
}
static int get_fex_version(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;
	*(u64 *)val = chip->fex_version;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fex_version_fops,
		get_fex_version, set_fex_version, "%llu\n");

static int get_fex_header(struct seq_file *m, void *data)
{
	int i = 0, len = 0;
	uint8_t buf[INFO_SIZE_MAX];

	memset(buf, 0x0, INFO_SIZE_MAX);
	len += sprintf(buf, "%lx:", EX_WARNING);
	for (i = 0; i < warning_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (warning_ids_size - 1))
				len += sprintf((buf + len), "%d,", warning_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", warning_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_SOC);
	for (i = 0; i < soc_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (soc_ids_size - 1))
				len += sprintf((buf + len), "%d,", soc_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", soc_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 40))
		len += sprintf((buf + len), "%lx:%d,%d,%d\n", (EX_SOC | (1 << SOC_CYCLE)),
				aging_ids[0], aging_ids[1], aging_ids[2]);

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_CHG);
	for (i = 0; i < chg_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (chg_ids_size - 1))
				len += sprintf((buf + len), "%d,", chg_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", chg_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 60))
		len += sprintf((buf + len), "%lx:%d,%d,%d,%d,%d,%d\n", (EX_CHG | (1 << CHG_HOT_PLUG)),
				hotplug_ids[0], hotplug_ids[1], hotplug_ids[2], hotplug_ids[3], hotplug_ids[4], hotplug_ids[4]);

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_CHGIC);
	for (i = 0; i < chgic_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (chgic_ids_size - 1))
				len += sprintf((buf + len), "%d,", chgic_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", chgic_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_FG);
	for (i = 0; i < fg_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (fg_ids_size - 1))
				len += sprintf((buf + len), "%d,", fg_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", fg_ids[i]);
		}
	}

	if (len < (INFO_SIZE_MAX - 10))
		len += sprintf((buf + len), "%lx:", EX_OTHER);
	for (i = 0; i < other_ids_size; i++) {
		if (len < (INFO_SIZE_MAX - 10)) {
			if (i < (other_ids_size - 1))
				len += sprintf((buf + len), "%d,", other_ids[i]);
			else
				len += sprintf((buf + len), "%d\n", other_ids[i]);
		}
	}

	seq_printf(m, "%s\n", buf);
	return 0;
}

static int fex_header_open(struct inode *inode, struct file *file)
{
	struct fuelsummary_chip *chip = inode->i_private;

	return single_open(file, get_fex_header, chip);
}

static const struct file_operations fex_header_fops = {
	.owner		= THIS_MODULE,
	.open		= fex_header_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_roll_current(struct seq_file *m, void *data)
{
	struct fuelsummary_chip *chip = m->private;
	struct current_roll *cr = &(chip->croll);

	seq_printf(m, "%d,%d,%d\n", cr->avg,
			cr->array[cr->max_index], cr->array[cr->min_index]);
	return 0;
}

static int roll_current_open(struct inode *inode, struct file *file)
{
	struct fuelsummary_chip *chip = inode->i_private;

	return single_open(file, get_roll_current, chip);
}

static const struct file_operations roll_current_fops = {
	.owner		= THIS_MODULE,
	.open		= roll_current_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int get_cycle(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (chip->ag_support == DRIVER_SUPPORTED)
		*(u64 *)val = run_val(ID_INT__CYCLE);
	else
		*(u64 *)val = chip->uspace_cycle;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cycle_fops, get_cycle, NULL, "%llu\n");

static int get_soh(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (chip->ag_support == DRIVER_SUPPORTED)
		*(u64 *)val = run_val(ID_INT__SOH);
	else
		*(u64 *)val = chip->uspace_soh;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(soh_fops, get_soh, NULL, "%llu\n");

static int get_aging_support(void *data, u64 *val)
{
	struct fuelsummary_chip *chip = (struct fuelsummary_chip *)data;

	if (chip->ag_support == DRIVER_SUPPORTED || (chip->ag_support == USPACE_SUPPORTED && chip->uspace_aging))
		*(u64 *)val = 1;
	else
		*(u64 *)val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(aging_support_fops, get_aging_support, NULL, "%llu\n");

static int get_aging_count(void *data, u64 *val)
{
	*(u64 *)val = run_val(ID_INT__FCC);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(aging_count_fops, get_aging_count, NULL, "%llu\n");

static int get_chgic(struct seq_file *m, void *data)
{
	seq_printf(m, "%d,%d\n", run_val(ID_BYTE__MIC_VENDOR), run_val(ID_BYTE__SIC_VENDOR));
	return 0;
}
static int chgic_open(struct inode *inode, struct file *file)
{
	struct fuelsummary_chip *chip = inode->i_private;

	return single_open(file, get_chgic, chip);
}
static const struct file_operations chgic_fops = {
	.owner		= THIS_MODULE,
	.open		= chgic_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_batid(void *data, u64 *val)
{
	*(u64 *)val = run_val(ID_BYTE__BATID);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(batid_fops, get_batid, NULL, "%llu\n");

static int get_fgid(void *data, u64 *val)
{
	*(u64 *)val = run_val(ID_BYTE__FG_DEV_NUM);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fgid_fops, get_fgid, NULL, "%llu\n");

static int config_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t config_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct fuelsummary_chip *chip = file->private_data;
	return simple_read_from_buffer(buf, count, ppos, &(chip->config_cnt), 1);
}
static ssize_t config_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	struct fuelsummary_chip *chip = file->private_data;
	struct config_work *cw = kzalloc(sizeof(*cw), GFP_KERNEL);

	if (!cw) {
		pr_err("couldn't alloc config_work struct\n");
		return -ENOMEM;
	}

	cw->params = kzalloc(count, GFP_KERNEL);
	if (!cw->params) {
		pr_err("params buffer alloc fail:%d\n", (int)count);
		kfree(cw);
		return -ENOMEM;
	}

	ret = copy_from_user(cw->params, buf, count);
	if (ret) {
		pr_err("failed to copy config_params from user\n");
		kfree(cw->params);
		kfree(cw);
		return -EFAULT;
	}

	if (count > CONFIG_HEADER_SIZE) {
		cw->type = (*(cw->params) & 0x3f);
		cw->chksum = *(cw->params + 1);
		cw->page = BYTE_SWAP_32(cw->params + 2);
		cw->row = BYTE_SWAP_32(cw->params + 6);
		cw->column = BYTE_SWAP_32(cw->params + 10);
		cw->name_offset = BYTE_SWAP_32(cw->params + 14);
		cw->name_size = BYTE_SWAP_32(cw->params + 18);
		cw->buff_offset = BYTE_SWAP_32(cw->params + 22);
		cw->buff_size = BYTE_SWAP_32(cw->params + 26);
		cw->total = count;

		INIT_WORK(&(cw->cwork), config_params_worker);
		queue_work(chip->wq, &(cw->cwork));
	} else {
		pr_err("buff size error\n");
		kfree(cw->params);
		kfree(cw);
	}
	return count;
}

static const struct file_operations config_fops = {
	.open		= config_open,
	.read		= config_read,
	.write		= config_write,
};


static int create_debugfs_entries(struct fuelsummary_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("fuelsummary", NULL);
	if (!chip->debug_root) {
		pr_info("Couldn't create debug dir\n");
	} else {
		ent = debugfs_create_file("fuelsummary_access", 0664,
				chip->debug_root, chip,
				&fuelsummary_access_fops);
		if (!ent)
			pr_info("Couldn't create fuelsummary_access debug \n");

		ent = debugfs_create_file("fifotest", 0664, chip->debug_root,
				chip, &fifotest_fops);
		if (!ent)
			pr_info("Couldn't create fifotest debug \n");

		ent = debugfs_create_file("user_custom", 0664,
				chip->debug_root, chip, &user_custom_fops);
		if (!ent)
			pr_info("Couldn't create user_custom debug file\n");

		ent = debugfs_create_file("custom_input", 0664,
				chip->debug_root, chip, &custom_input_fops);
		if (!ent)
			pr_info("Couldn't create custom_input debug file\n");

		ent = debugfs_create_file("custom_ibat", 0664,
				chip->debug_root, chip, &custom_ibat_fops);
		if (!ent)
			pr_info("Couldn't create custom_ibat debug file\n");

		ent = debugfs_create_file("custom_volt", 0664,
				chip->debug_root, chip, &custom_volt_fops);
		if (!ent)
			pr_info("Couldn't create custom_volt debug file\n");

		ent = debugfs_create_file("fex_version", 0664,
				chip->debug_root, chip, &fex_version_fops);
		if (!ent)
			pr_info("Couldn't create fex_version debug file\n");

		ent = debugfs_create_file("fex_header", 0664,
				chip->debug_root, chip, &fex_header_fops);
		if (!ent)
			pr_info("Couldn't create fex_header debug file\n");

		ent = debugfs_create_file("roll_current", 0664,
				chip->debug_root, chip, &roll_current_fops);
		if (!ent)
			pr_info("Couldn't create roll_current debug file\n");

		ent = debugfs_create_file("cycle", 0664,
				chip->debug_root, chip, &cycle_fops);
		if (!ent)
			pr_info("Couldn't create cycle debug file\n");

		ent = debugfs_create_file("soh", 0664,
				chip->debug_root, chip, &soh_fops);
		if (!ent)
			pr_info("Couldn't create soh debug file\n");

		ent = debugfs_create_file("aging_support", 0664,
				chip->debug_root, chip, &aging_support_fops);
		if (!ent)
			pr_info("Couldn't create aging_support debug file\n");

		ent = debugfs_create_file("aging_count", 0664,
				chip->debug_root, chip, &aging_count_fops);
		if (!ent)
			pr_info("Couldn't create aging_count debug file\n");

		ent = debugfs_create_file("chgic", 0664,
				chip->debug_root, chip, &chgic_fops);
		if (!ent)
			pr_info("Couldn't create chgic debug file\n");

		ent = debugfs_create_file("batid", 0664,
				chip->debug_root, chip, &batid_fops);
		if (!ent)
			pr_info("Couldn't create batid debug file\n");

		ent = debugfs_create_file("fgid", 0664,
				chip->debug_root, chip, &fgid_fops);
		if (!ent)
			pr_info("Couldn't create fgid debug file\n");

		ent = debugfs_create_file("config", 0664,
				chip->debug_root, chip, &config_fops);
		if (!ent)
			pr_info("Couldn't create config debug file\n");
	}
	return 0;
}
#endif



/************************************************************
 *
 *  [Fuelsummary driver]
 *
 ***********************************************************/
static int fuelsummary_parse_dt(struct fuelsummary_chip *chip)
{
	int ret = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_info("device tree missing\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(node, "chg-scheme", &(chip->chg_scheme));
	if (ret) {
		chip->chg_scheme = SCHEME_VIVO_FASTCHG;
		pr_info("chg_scheme not defined, default vivo-fastchg\n");
	}
	ret = of_property_read_u32(node, "fex-version", &(chip->fex_version));
	if (ret) {
		chip->fex_version = 0x1;
		pr_info("fex_version not defined, default 0x1\n");
	}
	ret = of_property_read_u32(node, "fbon-ibat", &(chip->fbon_ibat));
	if (ret) {
		chip->fbon_ibat = -1000;//-4500 + 650 + 200;
		pr_info("fbon_ibat not defined\n");
	} else {
		chip->fbon_ibat += 200;
	}
	//fuelsummary_of_property_put("fbon-ibat", PARAMS_TYPE_INT, &(chip->fbon_ibat));
	ret = of_property_read_u32(node, "fboff-ibat", &(chip->fboff_ibat));
	if (ret) {
		chip->fboff_ibat = -1000;//-4500 + 350 + 200;
		pr_info("fboff_ibat not defined\n");
	} else {
		chip->fboff_ibat += 200;
	}
	//fuelsummary_of_property_put("fboff-ibat", PARAMS_TYPE_INT, &(chip->fboff_ibat));

	ret = of_property_read_u32(node, "vivo,battery-aging-support", &(chip->ag_support));
	if (ret) {
		chip->ag_support = AGING_UNSUPPORTED;
		pr_info("vivo,battery-aging-support not defined\n");
	}
	//fuelsummary_of_property_put("vivo,battery-aging-support", PARAMS_TYPE_INT, &(chip->ag_support));
	return 0;
}



#if QCOM_PLATFORM
/************************************************************
 *
 *  [Fuelsummary power supply]
 *
 ***********************************************************/
static void fuelsummary_external_power_changed(struct power_supply *psy)
{
	int ret = 0, usb_in = 0, capacity = 0;
	struct fuelsummary_chip *chip = power_supply_get_drvdata(psy);
	bool system_start = (((u64)(local_clock()/1000000000)) <= 90);

	if (system_start) {
		pr_info("system not ready!\n");
		return;
	}
	ret = fuelsummary_get_psy_data(USB_PSY, POWER_SUPPLY_PROP_PRESENT, &usb_in);
	if (ret < 0) {
		pr_err("usb_psy not ready!\n");
		return;
	}
	if (!chip->usb_present && usb_in) {
		chip->usb_present = true;
		chip->chg_term = false;
		if (chip->running_usbin)
			queue_work(chip->wq, &(chip->running_usbin->fwork));
	} else if (chip->usb_present && !usb_in) {
		chip->usb_present = false;
		if (chip->running_usbout)
			queue_work(chip->wq, &(chip->running_usbout->fwork));
	}

	ret = fuelsummary_get_psy_data(BAT_PSY, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (ret < 0) {
		pr_err("bat_psy not ready!\n");
		return;
	}
	if (chip->soc != capacity) {
		chip->soc = capacity;
		fuelsummary_collect_value(ID_BOTH__SOC, capacity);
		fuelsummary_collect_value(ID_BOTH__STEP_SOC, capacity);
		if (chip->running_soc_change)
			queue_work(chip->wq, &(chip->running_soc_change->fwork));
	}
}
static int fuelsummary_get_property(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct fuelsummary_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_FEX_CNT:
		val->intval = chip->fex_count;
		break;
	case POWER_SUPPLY_PROP_FUEL_USER_CUSTOM:
		val->intval = chip->user_custom;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_INPUT:
		val->intval = chip->custom_input;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_CURRENT:
		val->intval = chip->custom_ibat;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_VOLTAGE:
		val->intval = chip->custom_volt;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int fuelsummary_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct fuelsummary_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_FEX_RUNNING:
		if (val->intval == SCENE_10STHD) {
			if (chip->running_10sthd)
				queue_work(chip->wq, &(chip->running_10sthd->fwork));
		} else if (val->intval == SCENE_CHG) {
			if (chip->running_chg)
				queue_work(chip->wq, &(chip->running_chg->fwork));
		} else if (val->intval == SCENE_USBIN) {
			if (chip->running_usbin)
				queue_work(chip->wq, &(chip->running_usbin->fwork));
		} else if (val->intval == SCENE_USBOUT) {
			if (chip->running_usbout)
				queue_work(chip->wq, &(chip->running_usbout->fwork));
		} else if (val->intval == SCENE_SUSPEND) {
			if (chip->running_suspend)
				queue_work(chip->wq, &(chip->running_suspend->fwork));
		} else if (val->intval == SCENE_RESUME) {
			if (chip->running_resume)
				queue_work(chip->wq, &(chip->running_resume->fwork));
		} else if (val->intval == SCENE_SOC_CHANGE) {
			if (chip->running_soc_change)
				queue_work(chip->wq, &(chip->running_soc_change->fwork));
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static char *fuelsummary_supplies[] = {
	"usb",
	"battery",
	"bq_bms",
};
static enum power_supply_property fuelsummary_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc fuelsummary_psy_desc = {
	.name = "fuelsummary",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = fuelsummary_properties,
	.num_properties	= ARRAY_SIZE(fuelsummary_properties),
	.get_property = fuelsummary_get_property,
	.set_property = fuelsummary_set_property,
	.external_power_changed = fuelsummary_external_power_changed,
};



#else
/************************************************************
 *
 *   [fuelsummary power_supply_lite function]
 *
 ***********************************************************/
static int fuelsummary_io_cable(struct fuelsummary_chip *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;

	if (io == SET_IN) {
		if (val->intval) {
			chip->usb_present = true;
			if (chip->running_usbin)
				queue_work(chip->wq, &(chip->running_usbin->fwork));
		} else {
			chip->usb_present = false;
			if (chip->running_usbout)
				queue_work(chip->wq, &(chip->running_usbout->fwork));
		}
	} else {
		pr_err("GET_OUT not support\n");
		ret = -EIO;
	}
	return ret;
}

static int fuelsummary_io_dump(struct fuelsummary_chip *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;
	union power_supply_propval capacity = {0,};

	if (io == GET_OUT) {
		power_supply_lite_get_property(PSYL_BATTERY, POWER_SUPPLY_PROP_CAPACITY, &capacity);
		if (chip->soc != capacity.intval) {
			chip->soc = capacity.intval;
			fuelsummary_collect_value(ID_BOTH__SOC, chip->soc);
			fuelsummary_collect_value(ID_BOTH__STEP_SOC, chip->soc);
			if (chip->running_soc_change)
				queue_work(chip->wq, &(chip->running_soc_change->fwork));
		} else {
			if (chip->running_10sthd)
				queue_work(chip->wq, &(chip->running_10sthd->fwork));

			if (chip->usb_present && chip->running_chg)
				queue_work(chip->wq, &(chip->running_chg->fwork));
		}
	} else {
		pr_err("GET_OUT not support\n");
		ret = -EIO;
	}
	return ret;
}

static int fuelsummary_ioctrl_property(struct power_supply_lite *psyl,
			enum direction_io io,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
        struct fuelsummary_chip *chip = power_supply_lite_get_drvdata(psyl);

	if (!psyl) {
		pr_info("ioctrl[%] not find psyl!!!\n", psp);
		return -EFAULT;
	}
#if PSYL_IO_DEBUG
	pr_debug("ioctrl[%d] io[%d]\n", psp, io);
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_CABLE_PLUG:
		ret = fuelsummary_io_cable(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_DUMP:
		ret = fuelsummary_io_dump(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHECK:
	case POWER_SUPPLY_PROP_RUNNING:
	case POWER_SUPPLY_PROP_COLLECT:
	case POWER_SUPPLY_PROP_UPDATE:
		break;
	case POWER_SUPPLY_PROP_FEX_CNT:
		if (io == GET_OUT)
			val->intval = chip->fex_count;
		break;
	case POWER_SUPPLY_PROP_FUEL_USER_CUSTOM:
		if (io == GET_OUT)
			val->intval = chip->user_custom;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_INPUT:
		if (io == GET_OUT)
			val->intval = chip->custom_input;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_CURRENT:
		if (io == GET_OUT)
			val->intval = chip->custom_ibat;
		break;
	case POWER_SUPPLY_PROP_FUEL_CUSTOM_VOLTAGE:
		if (io == GET_OUT)
			val->intval = chip->custom_volt;
		break;
	case POWER_SUPPLY_PROP_FEX_RUNNING:
		if (io == SET_IN) {
			if (val->intval == SCENE_10STHD) {
				if (chip->running_10sthd)
					queue_work(chip->wq, &(chip->running_10sthd->fwork));
			} else if (val->intval == SCENE_CHG) {
				if (chip->running_chg)
					queue_work(chip->wq, &(chip->running_chg->fwork));
			} else if (val->intval == SCENE_USBIN) {
				if (chip->running_usbin)
					queue_work(chip->wq, &(chip->running_usbin->fwork));
			} else if (val->intval == SCENE_USBOUT) {
				if (chip->running_usbout)
					queue_work(chip->wq, &(chip->running_usbout->fwork));
			} else if (val->intval == SCENE_SUSPEND) {
				if (chip->running_suspend)
					queue_work(chip->wq, &(chip->running_suspend->fwork));
			} else if (val->intval == SCENE_RESUME) {
				if (chip->running_resume)
					queue_work(chip->wq, &(chip->running_resume->fwork));
			} else if (val->intval == SCENE_SOC_CHANGE) {
				if (chip->running_soc_change)
					queue_work(chip->wq, &(chip->running_soc_change->fwork));
			}
		}
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (io == GET_OUT) {
			if (chip->ag_support == DRIVER_SUPPORTED)
				val->intval = run_val(ID_INT__CYCLE);
			else
				val->intval = chip->uspace_cycle;
		}
		break;
	default:
		ret = -EINVAL;
	}
#if PSYL_IO_DEBUG
	pr_info("ioctrl[%d] io[%d] %s\n", psp, io, (ret < 0 ? "error" : "done"));
#else
	if (ret < 0)
		pr_info("ioctrl[%d] io[%d] err\n", psp, io);
#endif
	return ret;
}

static struct power_supply_lite fuel_summary_psyl = {
	.name = "fuelsummary",
	.dev_num = PSYL_FUELSUMMARY,
	.initialized = false,
	.ioctrl_property = fuelsummary_ioctrl_property,
};
#endif

static int fuelsummary_probe(struct platform_device *pdev)
{
	int ret = -1, retry = 0;
#if QCOM_PLATFORM
	struct power_supply_config fuelsummary_cfg = {};
#endif
	struct fuelsummary_chip *chip = &fs_chip;

	chip->dev = &(pdev->dev);
	dev_set_drvdata(&(pdev->dev), chip);

	params_list_init(chip);

	ret = fuelsummary_parse_dt(chip);
	if (ret) {
		pr_info("Couldn't parse DT nodes ret=%d\n", ret);
		goto exit;
	}

	ret = kfifo_alloc(&(chip->fuel_fifo), (FIFO_SIZE_MAX * sizeof(uint8_t)), GFP_KERNEL);
	if (ret) {
		pr_err("could not allocate fuel_fifo\n");
		goto exit;
	}
	spin_lock_init(&(chip->fuel_fifo_lock));

	chip->user_custom = false;
	chip->uspace_aging = false;
	chip->uspace_cycle = 1;
	chip->uspace_soh = 100;
	create_class_attrs(chip);
#ifdef CONFIG_DEBUG_FS
	create_debugfs_entries(chip);
#endif
	while (retry++ < TRANSFER_RETRY_TIMES) {
		if (fex_init(chip))
			pr_info("failed to init fuelsummary ex: %d\n", retry);
		else
			break;
	}
	chip->wq = create_singlethread_workqueue("fuelsummary_wq");
	if (!chip->wq) {
		pr_info("unable to create workqueue for fuelsummary probed\n");
		goto exit_kfifo;
	}
	chip->running_10sthd = fuelsummary_init_fwork(SCENE_10STHD);
	chip->running_chg = fuelsummary_init_fwork(SCENE_CHG);
	chip->running_usbin = fuelsummary_init_fwork(SCENE_USBIN);
	chip->running_usbout = fuelsummary_init_fwork(SCENE_USBOUT);
	chip->running_resume = fuelsummary_init_fwork(SCENE_RESUME);
	chip->running_soc_change = fuelsummary_init_fwork(SCENE_SOC_CHANGE);
#if OWNER_10THD
	INIT_DELAYED_WORK(&(chip->thd_10s), fuelsummary_thd_10s);
#endif
	INIT_DELAYED_WORK(&(chip->boot_start), fuelsummary_boot_start);
	schedule_delayed_work(&(chip->boot_start), msecs_to_jiffies(80000));
#if QCOM_PLATFORM
	fuelsummary_cfg.drv_data = chip;
	fuelsummary_cfg.of_node = chip->dev->of_node;
	chip->fuelsummary_psy = power_supply_register(chip->dev,
			&fuelsummary_psy_desc, &fuelsummary_cfg);
	if (IS_ERR(chip->fuelsummary_psy)) {
		ret = PTR_ERR(chip->fuelsummary_psy);
		pr_info("Couldn't register fuelsummary psy ret=%d\n", ret);
		goto exit_kfifo;
	}
	chip->fuelsummary_psy->supplied_from = fuelsummary_supplies;
	chip->fuelsummary_psy->num_supplies = ARRAY_SIZE(fuelsummary_supplies);
#else
	chip->fuelsummary_psyl = &fuel_summary_psyl;
	chip->fuelsummary_psyl->drv_data = chip;
	ret = power_supply_lite_register(chip->fuelsummary_psyl, chip->fuelsummary_psyl->dev_num);
	if (ret < 0)
		pr_err("fuelsummary register psyl error: %d\n", ret);
#endif
	pr_info("fuelsummary successfully probed\n");
	return 0;

exit_kfifo:
	kfifo_free(&(chip->fuel_fifo));
exit:
	return ret;
}

static int fuelsummary_remove(struct platform_device *pdev)
{
	struct fuelsummary_chip *chip = dev_get_drvdata(&(pdev->dev));
	struct fuelsummary_ex *f, *n;
	struct config_params *cp, *cp_temp;

	pr_info("fex_count=%d\n", chip->fex_count);
	cancel_delayed_work(&(chip->boot_start));
	if (chip->wq)
		destroy_workqueue(chip->wq);
#if QCOM_PLATFORM
	power_supply_unregister(chip->fuelsummary_psy);
#endif
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->debug_root);
#endif
	list_for_each_entry_safe(f, n, chip->fex_list, list) {
		list_del(&(f->list));
		kfree(f);
	}

	if (chip->params_list && !list_empty(chip->params_list)) {
		list_for_each_entry_safe(cp, cp_temp, chip->params_list, list) {
			list_del(&(cp->list));
			kfree(cp->name);
			kfree(cp);
		}
	}

	kfifo_free(&(chip->fuel_fifo));
	return 0;
}

static void fuelsummary_shutdown(struct platform_device *pdev)
{
	struct fuelsummary_chip *chip = dev_get_drvdata(&(pdev->dev));

	pr_info("fex_count=%d\n", chip->fex_count);
}


/************************************************************
 *
 *  [Fuelsummary module function]
 *
 ***********************************************************/
static int fuelsummary_suspend(struct device *dev)
{
	struct fuelsummary_chip *chip = dev_get_drvdata(dev);

	chip->is_suspend = true;
	pr_info("enter sleep fex_count=%d\n", chip->fex_count);
	return 0;
}

static int fuelsummary_resume(struct device *dev)
{
	struct fuelsummary_chip *chip = dev_get_drvdata(dev);

	chip->is_suspend = false;
	pr_info("resume fex_count=%d\n", chip->fex_count);
	if (chip->running_resume)
		queue_work(chip->wq, &(chip->running_resume->fwork));
	return 0;
}

static const struct dev_pm_ops fuelsummary_pm_ops = {
	.suspend	= fuelsummary_suspend,
	.resume		= fuelsummary_resume,
};

static struct of_device_id fuelsummary_table[] = {
	{ .compatible = "vivo,fuelsummary",},
	{ },
};
MODULE_DEVICE_TABLE(of, fuelsummary_table);

static struct platform_driver fuelsummary_driver = {
	.driver		= {
		.name		= "fuelsummary",
		.owner		= THIS_MODULE,
		.of_match_table	= fuelsummary_table,
		.pm		= &fuelsummary_pm_ops,
	},
	.probe		= fuelsummary_probe,
	.remove		= fuelsummary_remove,
	.shutdown	= fuelsummary_shutdown,
};
module_platform_driver(fuelsummary_driver);


MODULE_AUTHOR("Sif");
MODULE_DESCRIPTION("FuelSummary Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("FuelSummary");
