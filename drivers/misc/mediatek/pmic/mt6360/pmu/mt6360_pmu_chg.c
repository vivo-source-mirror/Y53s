/*
 *  drivers/misc/mediatek/pmic/mt6360/mt6360_pmu_chg.c
 *  Driver for MT6360 PMU CHG part
 *
 *  Copyright (C) 2018 Mediatek Technology Inc.
 *  cy_huang <cy_huang@richtek.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../inc/mt6360_pmu.h"
#include "../inc/mt6360_pmu_chg.h"
#include <mt-plat/charger_class.h>
#include <mt-plat/charger_type.h>
/* add notify vbusov, eoc, rechg */
#include <mtk_charger_intf.h>
/* switch USB config */
#include <mt-plat/upmu_common.h>
#include <mt-plat/mtk_boot.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
#include <linux/power/vivo/interact.h>
#include <linux/power/vivo/power_supply_lite.h>
#include <fuel_summary.h>

#undef pr_fmt
#define pr_fmt(fmt) "MT6360_PMU_CHG: %s: " fmt, __func__

//#undef pr_info
//#define pr_info(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)

#define VBUS_HV_MIN			6500000
#define VBUS_MIN			4000000
#define MT6360_PMU_CHG_VENDOR_ID	0x5
#endif

#define MT6360_PMU_CHG_DRV_VERSION	"1.0.7_MTK"


void __attribute__ ((weak)) Charger_Detect_Init(void)
{
}

void __attribute__ ((weak)) Charger_Detect_Release(void)
{
}

enum mt6360_adc_channel {
	MT6360_ADC_VBUSDIV5,
	MT6360_ADC_VSYS,
	MT6360_ADC_VBAT,
	MT6360_ADC_IBUS,
	MT6360_ADC_IBAT,
	MT6360_ADC_NODUMP,
	MT6360_ADC_TEMP_JC = MT6360_ADC_NODUMP,
	MT6360_ADC_USBID,
	MT6360_ADC_TS,
	MT6360_ADC_MAX,
};

static const char * const mt6360_adc_chan_list[] = {
	"VBUSDIV5", "VSYS", "VBAT", "IBUS", "IBAT", "TEMP_JC", "USBID", "TS",
};

struct mt6360_pmu_chg_info {
	struct device *dev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debug_root;
#else
	struct class mt6360_chg_debug;
#endif
	struct mt6360_pmu_info *mpi;
	struct iio_channel *channels[MT6360_ADC_MAX];
	struct power_supply *psy;
	struct charger_device *chg_dev;
	int hidden_mode_cnt;
	struct mutex hidden_mode_lock;
	struct mutex pe_lock;
	struct mutex aicr_lock;
	struct mutex tchg_lock;
	struct mutex ichg_lock;
	int tchg;
	u32 zcv;
	u32 ichg;
	u32 ichg_dis_chg;

	/* Charger type detection */
	struct mutex chgdet_lock;
	bool attach;
	int chg_type;
	bool pwr_rdy;
	bool bc12_en;
#ifdef CONFIG_TCPC_CLASS
	bool tcpc_attach;
#else
	struct work_struct chgdet_work;
#endif /* CONFIG_TCPC_CLASS */

	struct completion aicc_done;
	struct completion pumpx_done;
	atomic_t pe_complete;
	/* mivr */
	atomic_t mivr_cnt;
	wait_queue_head_t waitq;
	struct task_struct *mivr_task;
	/* unfinish pe pattern */
	struct workqueue_struct *pe_wq;
	struct work_struct pe_work;
	u8 ctd_dischg_status;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	struct power_supply_lite *mt6360_psyl;
	void (*callback)(void);
	struct wake_lock bc12_wlock;
	bool exist;
	bool shutdown_ignore_hiz;
	bool otg_reverse_chg;
	int ico;
	int vboost_mv;
	int iboost_ma;
	uint8_t dump_info[DUMP_INFO_SIZE];
	struct work_struct irq_work;
#endif
};

/* for recive bat oc notify */
struct mt6360_pmu_chg_info *g_mpci;

enum mt6360_iinlmtsel {
	MT6360_IINLMTSEL_AICR_3250 = 0,
	MT6360_IINLMTSEL_CHG_TYPE,
	MT6360_IINLMTSEL_AICR,
	MT6360_IINLMTSEL_LOWER_LEVEL,
};

enum mt6360_charging_status {
	MT6360_CHG_STATUS_READY = 0,
	MT6360_CHG_STATUS_PROGRESS,
	MT6360_CHG_STATUS_DONE,
	MT6360_CHG_STATUS_FAULT,
	MT6360_CHG_STATUS_MAX,
};

enum mt6360_usbsw_state {
	MT6360_USBSW_CHG = 0,
	MT6360_USBSW_USB,
};

enum mt6360_pmu_chg_type {
	MT6360_CHG_TYPE_NOVBUS = 0,
	MT6360_CHG_TYPE_UNDER_GOING,
	MT6360_CHG_TYPE_SDP,
	MT6360_CHG_TYPE_SDPNSTD,
	MT6360_CHG_TYPE_DCP,
	MT6360_CHG_TYPE_CDP,
	MT6360_CHG_TYPE_MAX,
};

static const char *mt6360_chg_status_name[MT6360_CHG_STATUS_MAX] = {
	"ready", "progress", "done", "fault",
};

static const struct mt6360_chg_platform_data def_platform_data = {
	.ichg = 2000000,		/* uA */
	.aicr = 500000,			/* uA */
	.mivr = 4400000,		/* uV */
	.cv = 4350000,			/* uA */
	.ieoc = 250000,			/* uA */
	.safety_timer = 12,		/* hour */
#ifdef CONFIG_MTK_BIF_SUPPORT
	.ircmp_resistor = 0,		/* uohm */
	.ircmp_vclamp = 0,		/* uV */
#else
	.ircmp_resistor = 0,	/* uohm */
	.ircmp_vclamp = 0,		/* uV */
#endif
	.en_te = true,
	.en_wdt = true,
	.aicc_once = true,
	.post_aicc = true,
	.batoc_notify = false,
	.chg_name = "primary_chg",
};

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
int charger_dev_notify(struct charger_device *chg_dev, int event)
{
	return 0;
}
#endif

/* ================== */
/* Internal Functions */
/* ================== */
static inline u32 mt6360_trans_sel(u32 target, u32 min_val, u32 step,
				   u32 max_sel)
{
	u32 data = 0;

	if (target >= min_val)
		data = (target - min_val) / step;
	if (data > max_sel)
		data = max_sel;
	return data;
}

static u32 mt6360_trans_ichg_sel(u32 uA)
{
	return mt6360_trans_sel(uA, 100000, 100000, 0x31);
}

static u32 mt6360_trans_aicr_sel(u32 uA)
{
	return mt6360_trans_sel(uA, 100000, 50000, 0x3F);
}

static u32 mt6360_trans_mivr_sel(u32 uV)
{
	return mt6360_trans_sel(uV, 3900000, 100000, 0x5F);
}

static u32 mt6360_trans_cv_sel(u32 uV)
{
	return mt6360_trans_sel(uV, 3900000, 10000, 0x51);
}

static u32 mt6360_trans_ieoc_sel(u32 uA)
{
	return mt6360_trans_sel(uA, 100000, 50000, 0x0F);
}

static u32 mt6360_trans_safety_timer_sel(u32 hr)
{
	u32 mt6360_st_tbl[] = {4, 6, 8, 10, 12, 14, 16, 20};
	u32 cur_val, next_val;
	int i;

	if (hr < mt6360_st_tbl[0])
		return 0;
	for (i = 0; i < ARRAY_SIZE(mt6360_st_tbl) - 1; i++) {
		cur_val = mt6360_st_tbl[i];
		next_val = mt6360_st_tbl[i+1];
		if (hr >= cur_val && hr < next_val)
			return i;
	}
	return ARRAY_SIZE(mt6360_st_tbl) - 1;
}

static u32 mt6360_trans_ircmp_r_sel(u32 uohm)
{
	return mt6360_trans_sel(uohm, 0, 25000, 0x07);
}

static u32 mt6360_trans_ircmp_vclamp_sel(u32 uV)
{
	return mt6360_trans_sel(uV, 0, 32000, 0x07);
}

static const u32 mt6360_usbid_rup[] = {
	500000, 75000, 5000, 1000
};

static inline u32 mt6360_trans_usbid_rup(u32 rup)
{
	int i;
	int maxidx = ARRAY_SIZE(mt6360_usbid_rup) - 1;

	if (rup >= mt6360_usbid_rup[0])
		return 0;
	if (rup <= mt6360_usbid_rup[maxidx])
		return maxidx;

	for (i = 0; i < maxidx; i++) {
		if (rup == mt6360_usbid_rup[i])
			return i;
		if (rup < mt6360_usbid_rup[i] &&
		    rup > mt6360_usbid_rup[i + 1]) {
			if ((mt6360_usbid_rup[i] - rup) <=
			    (rup - mt6360_usbid_rup[i + 1]))
				return i;
			else
				return i + 1;
		}
	}
	return maxidx;
}

static const u32 mt6360_usbid_src_ton[] = {
	400, 1000, 4000, 10000, 40000, 100000, 400000,
};

static inline u32 mt6360_trans_usbid_src_ton(u32 src_ton)
{
	int i;
	int maxidx = ARRAY_SIZE(mt6360_usbid_src_ton) - 1;

	/* There is actually an option, always on, after 400000 */
	if (src_ton == 0)
		return maxidx + 1;
	if (src_ton < mt6360_usbid_src_ton[0])
		return 0;
	if (src_ton > mt6360_usbid_src_ton[maxidx])
		return maxidx;

	for (i = 0; i < maxidx; i++) {
		if (src_ton == mt6360_usbid_src_ton[i])
			return i;
		if (src_ton > mt6360_usbid_src_ton[i] &&
		    src_ton < mt6360_usbid_src_ton[i + 1]) {
			if ((src_ton - mt6360_usbid_src_ton[i]) <=
			    (mt6360_usbid_src_ton[i + 1] - src_ton))
				return i;
			else
				return i + 1;
		}
	}
	return maxidx;
}

static inline int mt6360_get_ieoc(struct mt6360_pmu_chg_info *mpci, u32 *uA)
{
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL9);
	if (ret < 0)
		return ret;
	ret = (ret & MT6360_MASK_IEOC) >> MT6360_SHFT_IEOC;
	*uA = 100000 + (ret * 50000);
	return ret;
}

static inline int mt6360_get_charging_status(
					struct mt6360_pmu_chg_info *mpci,
					enum mt6360_charging_status *chg_stat)
{
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT);
	if (ret < 0)
		return ret;
	*chg_stat = (ret & MT6360_MASK_CHG_STAT) >> MT6360_SHFT_CHG_STAT;
	return 0;
}

static inline int mt6360_is_charger_enabled(struct mt6360_pmu_chg_info *mpci,
					    bool *en)
{
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL2);
	if (ret < 0)
		return ret;
	*en = (ret & MT6360_MASK_CHG_EN) ? true : false;
	return 0;
}

static inline int mt6360_select_input_current_limit(
		struct mt6360_pmu_chg_info *mpci, enum mt6360_iinlmtsel sel)
{
	dev_dbg(mpci->dev,
		"%s: select input current limit = %d\n", __func__, sel);
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL2,
					  MT6360_MASK_IINLMTSEL,
					  sel << MT6360_SHFT_IINLMTSEL);
}

static int mt6360_enable_wdt(struct mt6360_pmu_chg_info *mpci, bool en)
{
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(mpci->dev);

	dev_dbg(mpci->dev, "%s enable wdt, en = %d\n", __func__, en);
	if (!pdata->en_wdt)
		return 0;
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL13,
					  MT6360_MASK_CHG_WDT_EN,
					  en ? 0xff : 0);
}

static inline int mt6360_get_chrdet_ext_stat(struct mt6360_pmu_chg_info *mpci,
					  bool *pwr_rdy)
{
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHRDET_STAT);
	if (ret < 0)
		return ret;
	*pwr_rdy = (ret & BIT(4)) ? true : false;
	return 0;
}

#ifdef CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT
static int mt6360_psy_online_changed(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
#if 1 /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)) */
	union power_supply_propval propval;

	/* Get chg type det power supply */
	if (!mpci->psy)
		mpci->psy = power_supply_get_by_name("charger");
	if (!mpci->psy) {
		dev_notice(mpci->dev,
			"%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

	propval.intval = mpci->attach;
	ret = power_supply_set_property(mpci->psy, POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret < 0)
		dev_err(mpci->dev, "%s: psy online fail(%d)\n", __func__, ret);
	else
		dev_info(mpci->dev,
			 "%s: pwr_rdy = %d\n",  __func__, mpci->attach);
#endif
	return ret;
}

static int mt6360_psy_chg_type_changed(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
#if 1 /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)) */
	union power_supply_propval propval;

	/* Get chg type det power supply */
	if (!mpci->psy)
		mpci->psy = power_supply_get_by_name("charger");
	if (!mpci->psy) {
		dev_notice(mpci->dev,
			"%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

	propval.intval = mpci->chg_type;
	ret = power_supply_set_property(mpci->psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE,
					&propval);
	if (ret < 0)
		dev_err(mpci->dev,
			"%s: psy type failed, ret = %d\n", __func__, ret);
	else
		dev_info(mpci->dev,
			 "%s: chg_type = %d\n", __func__, mpci->chg_type);
#endif
	return ret;
}
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT */

static int mt6360_set_usbsw_state(struct mt6360_pmu_chg_info *mpci, int state)
{
	dev_info(mpci->dev, "%s: state = %d\n", __func__, state);

	/* Switch D+D- to AP/MT6360 */
	if (state == MT6360_USBSW_CHG)
		Charger_Detect_Init();
	else
		Charger_Detect_Release();

	return 0;
}

#ifndef CONFIG_MT6360_DCDTOUT_SUPPORT
static int __maybe_unused mt6360_enable_dcd_tout(
				      struct mt6360_pmu_chg_info *mpci, bool en)
{
	dev_info(mpci->dev, "%s en = %d\n", __func__, en);
	return (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_DEVICE_TYPE, MT6360_MASK_DCDTOUTEN);
}

static int __maybe_unused mt6360_is_dcd_tout_enable(
				     struct mt6360_pmu_chg_info *mpci, bool *en)
{
	int ret;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_DEVICE_TYPE);
	if (ret < 0) {
		*en = false;
		return ret;
	}
	*en = (ret & MT6360_MASK_DCDTOUTEN ? true : false);
	return 0;
}
#endif

static int __mt6360_enable_usbchgen(struct mt6360_pmu_chg_info *mpci, bool en)
{
	int i, ret = 0;
	const int max_wait_cnt = 200;
#ifndef CONFIG_TCPC_CLASS
	bool pwr_rdy = false;
#endif /* !CONFIG_TCPC_CLASS */
	enum mt6360_usbsw_state usbsw =
				       en ? MT6360_USBSW_CHG : MT6360_USBSW_USB;
#ifndef CONFIG_MT6360_DCDTOUT_SUPPORT
	bool dcd_en = false;
#endif /* CONFIG_MT6360_DCDTOUT_SUPPORT */

	dev_info(mpci->dev, "%s: en = %d\n", __func__, en);
	if (en) {
#ifndef CONFIG_MT6360_DCDTOUT_SUPPORT
		ret = mt6360_is_dcd_tout_enable(mpci, &dcd_en);
		if (!dcd_en)
			msleep(180);
#endif /* CONFIG_MT6360_DCDTOUT_SUPPORT */
		/* Workaround for CDP port */
		for (i = 0; i < max_wait_cnt; i++) {
			if (is_usb_rdy())
				break;
			dev_info(mpci->dev, "%s: CDP block\n", __func__);
#ifndef CONFIG_TCPC_CLASS
			/* Check vbus */
			ret = mt6360_get_chrdet_ext_stat(mpci, &pwr_rdy);
			if (ret < 0) {
				dev_err(mpci->dev, "%s: fail, ret = %d\n",
					 __func__, ret);
				return ret;
			}
			dev_info(mpci->dev, "%s: pwr_rdy = %d\n", __func__,
				 pwr_rdy);
			if (!pwr_rdy) {
				dev_info(mpci->dev, "%s: plug out\n", __func__);
				return ret;
			}
#else
			if (!(mpci->tcpc_attach)) {
				dev_info(mpci->dev,
					 "%s: plug out\n", __func__);
				return 0;
			}
#endif /* !CONFIG_TCPC_CLASS */
			msleep(100);
		}
		if (i == max_wait_cnt)
			dev_err(mpci->dev, "%s: CDP timeout\n", __func__);
		else
			dev_info(mpci->dev, "%s: CDP free\n", __func__);
	}
	mt6360_set_usbsw_state(mpci, usbsw);
	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_DEVICE_TYPE,
					 MT6360_MASK_USBCHGEN, en ? 0xff : 0);
	if (ret >= 0)
		mpci->bc12_en = en;
	return ret;
}

static int mt6360_enable_usbchgen(struct mt6360_pmu_chg_info *mpci, bool en)
{
	int ret = 0;

	mutex_lock(&mpci->chgdet_lock);
	ret = __mt6360_enable_usbchgen(mpci, en);
	mutex_unlock(&mpci->chgdet_lock);
	return ret;
}

#ifdef CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT
static int mt6360_chgdet_pre_process(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
	bool attach = false;

#ifdef CONFIG_TCPC_CLASS
	attach = mpci->tcpc_attach;
#else
	attach = mpci->pwr_rdy;
#endif /* CONFIG_TCPC_CLASS */
	if (attach && is_meta_mode()) {
		/* Skip charger type detection to speed up meta boot.*/
		dev_notice(mpci->dev, "%s: force Standard USB Host in meta\n",
			   __func__);
		mpci->attach = attach;
		mpci->chg_type = STANDARD_HOST;
		ret = mt6360_psy_online_changed(mpci);
		if (ret < 0)
			dev_notice(mpci->dev,
				   "%s: set psy online fail\n", __func__);
		return mt6360_psy_chg_type_changed(mpci);
	}
	return __mt6360_enable_usbchgen(mpci, attach);
}

static int mt6360_chgdet_post_process(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
	bool attach = false, inform_psy = true;
	u8 usb_status = CHARGER_UNKNOWN;

#ifdef CONFIG_TCPC_CLASS
	attach = mpci->tcpc_attach;
#else
	attach = mpci->pwr_rdy;
#endif /* CONFIG_TCPC_CLASS */
	if (mpci->attach == attach) {
		dev_info(mpci->dev, "%s: attach(%d) is the same\n",
				    __func__, attach);
		inform_psy = !attach;
		goto out;
	}
	mpci->attach = attach;
	dev_info(mpci->dev, "%s: attach = %d\n", __func__, attach);
	/* Plug out during BC12 */
	if (!attach) {
		mpci->chg_type = CHARGER_UNKNOWN;
		goto out;
	}
	/* Plug in */
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_USB_STATUS1);
	if (ret < 0)
		goto out;
	usb_status = (ret & MT6360_MASK_USB_STATUS) >> MT6360_SHFT_USB_STATUS;
	switch (usb_status) {
	case MT6360_CHG_TYPE_UNDER_GOING:
		dev_info(mpci->dev, "%s: under going...\n", __func__);
		return ret;
	case MT6360_CHG_TYPE_SDP:
		mpci->chg_type = STANDARD_HOST;
		break;
	case MT6360_CHG_TYPE_SDPNSTD:
		mpci->chg_type = APPLE_2_1A_CHARGER;
		break;
	case MT6360_CHG_TYPE_CDP:
		mpci->chg_type = CHARGING_HOST;
		break;
	case MT6360_CHG_TYPE_DCP:
		mpci->chg_type = STANDARD_CHARGER;
		break;
	}
out:
	if (!attach) {
		ret = __mt6360_enable_usbchgen(mpci, false);
		if (ret < 0)
			dev_notice(mpci->dev, "%s: disable chgdet fail\n",
				   __func__);
	} else if (mpci->chg_type != STANDARD_CHARGER)
		mt6360_set_usbsw_state(mpci, MT6360_USBSW_USB);
	if (!inform_psy)
		return ret;
	ret = mt6360_psy_online_changed(mpci);
	if (ret < 0)
		dev_err(mpci->dev, "%s: report psy online fail\n", __func__);
	return mt6360_psy_chg_type_changed(mpci);
}
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT */

static const u32 mt6360_vinovp_list[] = {
	5500000, 6500000, 10500000, 14500000,
};

static int mt6360_select_vinovp(struct mt6360_pmu_chg_info *mpci, u32 uV)
{
	int i;

	if (uV < mt6360_vinovp_list[0])
		return -EINVAL;
	for (i = 1; i < ARRAY_SIZE(mt6360_vinovp_list); i++) {
		if (uV < mt6360_vinovp_list[i])
			break;
	}
	i--;
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL19,
					  MT6360_MASK_CHG_VIN_OVP_VTHSEL,
					  i << MT6360_SHFT_CHG_VIN_OVP_VTHSEL);
}

static inline int mt6360_read_zcv(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
	u8 zcv_data[2] = {0};

	dev_dbg(mpci->dev, "%s\n", __func__);
	/* Read ZCV data */
	ret = mt6360_pmu_reg_block_read(mpci->mpi, MT6360_PMU_ADC_BAT_DATA_H,
					2, zcv_data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read zcv data fail\n", __func__);
		return ret;
	}
	mpci->zcv = 1250 * (zcv_data[0] * 256 + zcv_data[1]);
	dev_info(mpci->dev, "%s: zcv = (0x%02X, 0x%02X, %dmV)\n",
		 __func__, zcv_data[0], zcv_data[1], mpci->zcv/1000);
	/* Disable ZCV */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_ADC_CONFIG,
				      MT6360_MASK_ZCV_EN);
	if (ret < 0)
		dev_err(mpci->dev, "%s: disable zcv fail\n", __func__);
	return ret;
}

/* ================== */
/* External Functions */
/* ================== */
static int __mt6360_set_ichg(struct mt6360_pmu_chg_info *mpci, u32 uA)
{
	int ret = 0;
	u32 data = 0;

	mt_dbg(mpci->dev, "%s\n", __func__);
	data = mt6360_trans_ichg_sel(uA);
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL7,
					 MT6360_MASK_ICHG,
					 data << MT6360_SHFT_ICHG);
	if (ret < 0)
		dev_notice(mpci->dev, "%s: fail\n", __func__);
	else
		mpci->ichg = uA;
	return ret;
}

static int mt6360_set_ichg(struct charger_device *chg_dev, u32 uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	mutex_lock(&mpci->ichg_lock);
	ret = __mt6360_set_ichg(mpci, uA);
	mutex_unlock(&mpci->ichg_lock);
	return ret;
}

static int mt6360_get_ichg(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL7);
	if (ret < 0)
		return ret;
	ret = (ret & MT6360_MASK_ICHG) >> MT6360_SHFT_ICHG;
	*uA = 100000 + (ret * 100000);
	return 0;
}

static int mt6360_enable_hidden_mode(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	static const u8 pascode[] = { 0x69, 0x96, 0x63, 0x72, };
	int ret = 0;

	mutex_lock(&mpci->hidden_mode_lock);
	if (en) {
		if (mpci->hidden_mode_cnt == 0) {
			ret = mt6360_pmu_reg_block_write(mpci->mpi,
					   MT6360_PMU_TM_PAS_CODE1, 4, pascode);
			if (ret < 0)
				goto err;
		}
		mpci->hidden_mode_cnt++;
	} else {
		if (mpci->hidden_mode_cnt == 1)
			ret = mt6360_pmu_reg_write(mpci->mpi,
						 MT6360_PMU_TM_PAS_CODE1, 0x00);
		mpci->hidden_mode_cnt--;
		if (ret < 0)
			goto err;
	}
	mt_dbg(mpci->dev, "%s: en = %d\n", __func__, en);
	goto out;
err:
	dev_err(mpci->dev, "%s failed, en = %d\n", __func__, en);
out:
	mutex_unlock(&mpci->hidden_mode_lock);
	return ret;
}

static int mt6360_enable(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;
	u32 ichg_ramp_t = 0;

	mt_dbg(mpci->dev, "%s: en = %d\n", __func__, en);

	/* Workaround for vsys overshoot */
	mutex_lock(&mpci->ichg_lock);
	if (mpci->ichg < 500000) {
		dev_info(mpci->dev,
			 "%s: ichg < 500mA, bypass vsys wkard\n", __func__);
		goto out;
	}
	if (!en) {
		mpci->ichg_dis_chg = mpci->ichg;
		ichg_ramp_t = (mpci->ichg - 500000) / 50000 * 2;
		/* Set ichg to 500mA */
		ret = mt6360_pmu_reg_update_bits(mpci->mpi,
						 MT6360_PMU_CHG_CTRL7,
						 MT6360_MASK_ICHG,
						 0x04 << MT6360_SHFT_ICHG);
		if (ret < 0) {
			dev_notice(mpci->dev,
				   "%s: set ichg fail\n", __func__);
			goto vsys_wkard_fail;
		}
		mdelay(ichg_ramp_t);
	} else {
		if (mpci->ichg == mpci->ichg_dis_chg) {
			ret = __mt6360_set_ichg(mpci, mpci->ichg);
			if (ret < 0) {
				dev_notice(mpci->dev,
					   "%s: set ichg fail\n", __func__);
				goto out;
			}
		}
	}

out:
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL2,
					 MT6360_MASK_CHG_EN, en ? 0xff : 0);
	if (ret < 0)
		dev_notice(mpci->dev, "%s: fail, en = %d\n", __func__, en);
vsys_wkard_fail:
	mutex_unlock(&mpci->ichg_lock);
	return ret;
}

static int mt6360_get_min_ichg(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 300000;
	return 0;
}

static int mt6360_set_cv(struct charger_device *chg_dev, u32 uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	u8 data = 0;

	dev_dbg(mpci->dev, "%s: cv = %d\n", __func__, uV);
	data = mt6360_trans_cv_sel(uV);
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL4,
					  MT6360_MASK_VOREG,
					  data << MT6360_SHFT_VOREG);
}

static int mt6360_get_cv(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL4);
	if (ret < 0)
		return ret;
	ret = (ret & MT6360_MASK_VOREG) >> MT6360_SHFT_VOREG;
	*uV = 3900000 + (ret * 10000);
	return 0;
}

static int mt6360_toggle_aicc(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
	u8 data = 0;

	mutex_lock(&mpci->mpi->io_lock);
	/* read aicc */
	ret = i2c_smbus_read_i2c_block_data(mpci->mpi->i2c,
					       MT6360_PMU_CHG_CTRL14, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read aicc fail\n", __func__);
		goto out;
	}
	/* aicc off */
	data &= ~MT6360_MASK_RG_EN_AICC;
	ret = i2c_smbus_read_i2c_block_data(mpci->mpi->i2c,
					       MT6360_PMU_CHG_CTRL14, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: aicc off fail\n", __func__);
		goto out;
	}
	/* aicc on */
	data |= MT6360_MASK_RG_EN_AICC;
	ret = i2c_smbus_read_i2c_block_data(mpci->mpi->i2c,
					       MT6360_PMU_CHG_CTRL14, 1, &data);
	if (ret < 0)
		dev_err(mpci->dev, "%s: aicc on fail\n", __func__);
out:
	mutex_unlock(&mpci->mpi->io_lock);
	return ret;
}

static int mt6360_set_aicr(struct charger_device *chg_dev, u32 uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(mpci->dev);
	int ret = 0;
	u8 data = 0;

	dev_dbg(mpci->dev, "%s\n", __func__);
	/* Toggle aicc for auto aicc mode */
	if (!pdata->aicc_once) {
		ret = mt6360_toggle_aicc(mpci);
		if (ret < 0) {
			dev_err(mpci->dev, "%s: toggle aicc fail\n", __func__);
			return ret;
		}
	}
	/* Disable sys drop improvement for download mode */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_CTRL20,
				      MT6360_MASK_EN_SDI);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable en_sdi fail\n", __func__);
		return ret;
	}
	data = mt6360_trans_aicr_sel(uA);
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL3,
					  MT6360_MASK_AICR,
					  data << MT6360_SHFT_AICR);
}

static int mt6360_get_aicr(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL3);
	if (ret < 0)
		return ret;
	ret = (ret & MT6360_MASK_AICR) >> MT6360_SHFT_AICR;
	*uA = 100000 + (ret * 50000);
	return 0;
}

static int mt6360_get_min_aicr(struct charger_device *chg_dev, u32 *uA)
{
	*uA = 100000;
	return 0;
}

static int mt6360_set_ieoc(struct charger_device *chg_dev, u32 uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	u8 data = 0;

	dev_dbg(mpci->dev, "%s: ieoc = %d\n", __func__, uA);
	data = mt6360_trans_ieoc_sel(uA);
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL9,
					  MT6360_MASK_IEOC,
					  data << MT6360_SHFT_IEOC);
}

static int mt6360_set_mivr(struct charger_device *chg_dev, u32 uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	u32 aicc_vth = 0, data = 0;
	u8 aicc_vth_sel = 0;
	int ret = 0;

	mt_dbg(mpci->dev, "%s: mivr = %d\n", __func__, uV);
	if (uV < 3900000 || uV > 13400000) {
		dev_err(mpci->dev,
			"%s: unsuitable mivr val(%d)\n", __func__, uV);
		return -EINVAL;
	}
	/* Check if there's a suitable AICC_VTH */
	aicc_vth = uV;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	if (aicc_vth > 4600000 && aicc_vth < 5000000)
		aicc_vth = 4600000;
#endif
	aicc_vth_sel = (aicc_vth - 3900000) / 100000;
	if (aicc_vth_sel > MT6360_AICC_VTH_MAXVAL) {
		dev_err(mpci->dev, "%s: can't match, aicc_vth_sel = %d\n",
			__func__, aicc_vth_sel);
		return -EINVAL;
	}
	/* Set AICC_VTH threshold */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL16,
					 MT6360_MASK_AICC_VTH,
					 aicc_vth_sel << MT6360_SHFT_AICC_VTH);
	if (ret < 0)
		return ret;
	/* Set MIVR */
	data = mt6360_trans_mivr_sel(uV);
	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL6,
					  MT6360_MASK_MIVR,
					  data << MT6360_SHFT_MIVR);
}

static inline int mt6360_get_mivr(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL6);
	if (ret < 0)
		return ret;
	ret = (ret & MT6360_MASK_MIVR) >> MT6360_SHFT_MIVR;
	*uV = 3900000 + (ret * 100000);
	return 0;
}

static int mt6360_get_mivr_state(struct charger_device *chg_dev, bool *in_loop)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
	if (ret < 0)
		return ret;
	*in_loop = (ret & MT6360_MASK_MIVR_EVT) >> MT6360_SHFT_MIVR_EVT;
	return 0;
}

static int mt6360_enable_te(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(mpci->dev);

	dev_info(mpci->dev, "%s: en = %d\n", __func__, en);
	if (!pdata->en_te)
		return 0;
	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_CHG_CTRL2,
					  MT6360_MASK_TE_EN, en ? 0xff : 0);
}

static int mt6360_enable_pump_express(struct mt6360_pmu_chg_info *mpci,
				      bool pe20)
{
	long timeout, pe_timeout = pe20 ? 1400 : 2800;
	int ret = 0;

	dev_info(mpci->dev, "%s\n", __func__);
	ret = mt6360_set_aicr(mpci->chg_dev, 800000);
	if (ret < 0)
		return ret;
	ret = mt6360_set_ichg(mpci->chg_dev, 2000000);
	if (ret < 0)
		return ret;
	ret = mt6360_enable(mpci->chg_dev, true);
	if (ret < 0)
		return ret;
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_CTRL17,
				      MT6360_MASK_EN_PUMPX);
	if (ret < 0)
		return ret;
	ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL17,
				      MT6360_MASK_EN_PUMPX);
	if (ret < 0)
		return ret;
	reinit_completion(&mpci->pumpx_done);
	atomic_set(&mpci->pe_complete, 1);
	timeout = wait_for_completion_interruptible_timeout(
			       &mpci->pumpx_done, msecs_to_jiffies(pe_timeout));
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = -EINTR;
	else
		ret = 0;
	if (ret < 0)
		dev_err(mpci->dev,
			"%s: wait pumpx timeout, ret = %d\n", __func__, ret);
	return ret;
}

static int mt6360_set_pep_current_pattern(struct charger_device *chg_dev,
					  bool is_inc)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_dbg(mpci->dev, "%s: pe1.0 pump up = %d\n", __func__, is_inc);

	mutex_lock(&mpci->pe_lock);
	/* Set to PE1.0 */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi,
				      MT6360_PMU_CHG_CTRL17,
				      MT6360_MASK_PUMPX_20_10);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: enable pumpx 10 fail\n", __func__);
		goto out;
	}

	/* Set Pump Up/Down */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL17,
					 MT6360_MASK_PUMPX_UP_DN,
					 is_inc ? 0xff : 0);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set pumpx up/down fail\n", __func__);
		goto out;
	}
	ret = mt6360_enable_pump_express(mpci, false);
out:
	mutex_unlock(&mpci->pe_lock);
	return ret;
}

static int mt6360_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	struct charger_manager *chg_mgr = NULL;

	chg_mgr = charger_dev_get_drvdata(chg_dev);
	if (!chg_mgr)
		return -EINVAL;

	chg_mgr->pe2.profile[0].vchr = 8000000;
	chg_mgr->pe2.profile[1].vchr = 8000000;
	chg_mgr->pe2.profile[2].vchr = 8000000;
	chg_mgr->pe2.profile[3].vchr = 8500000;
	chg_mgr->pe2.profile[4].vchr = 8500000;
	chg_mgr->pe2.profile[5].vchr = 8500000;
	chg_mgr->pe2.profile[6].vchr = 9000000;
	chg_mgr->pe2.profile[7].vchr = 9000000;
	chg_mgr->pe2.profile[8].vchr = 9500000;
	chg_mgr->pe2.profile[9].vchr = 9500000;
#endif
	return 0;
}

static int mt6360_set_pep20_current_pattern(struct charger_device *chg_dev,
					    u32 uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;
	u8 data = 0;

	dev_dbg(mpci->dev, "%s: pep2.0 = %d\n", __func__, uV);
	mutex_lock(&mpci->pe_lock);
	if (uV >= 5500000)
		data = (uV - 5500000) / 500000;
	if (data > MT6360_PUMPX_20_MAXVAL)
		data = MT6360_PUMPX_20_MAXVAL;
	/* Set to PE2.0 */
	ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL17,
				      MT6360_MASK_PUMPX_20_10);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: enable pumpx 20 fail\n", __func__);
		goto out;
	}
	/* Set Voltage */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL17,
					 MT6360_MASK_PUMPX_DEC,
					 data << MT6360_SHFT_PUMPX_DEC);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set pumpx voltage fail\n", __func__);
		goto out;
	}
	ret = mt6360_enable_pump_express(mpci, true);
out:
	mutex_unlock(&mpci->pe_lock);
	return ret;
}

static int mt6360_reset_ta(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_dbg(mpci->dev, "%s\n", __func__);
	ret = mt6360_set_mivr(chg_dev, 4600000);
	if (ret < 0)
		return ret;
	ret = mt6360_select_input_current_limit(mpci, MT6360_IINLMTSEL_AICR);
	if (ret < 0)
		return ret;
	ret = mt6360_set_aicr(chg_dev, 100000);
	if (ret < 0)
		return ret;
	msleep(250);
	return mt6360_set_aicr(chg_dev, 500000);
}

static int mt6360_enable_cable_drop_comp(struct charger_device *chg_dev,
					 bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_info(mpci->dev, "%s: en = %d\n", __func__, en);
	if (en)
		return ret;

	/* Set to PE2.0 */
	mutex_lock(&mpci->pe_lock);
	ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL17,
				      MT6360_MASK_PUMPX_20_10);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: enable pumpx 20 fail\n", __func__);
		goto out;
	}
	/* Disable cable drop compensation */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi,
					 MT6360_PMU_CHG_CTRL17,
					 MT6360_MASK_PUMPX_DEC,
					 0x1F << MT6360_SHFT_PUMPX_DEC);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set pumpx voltage fail\n", __func__);
		goto out;
	}
	ret = mt6360_enable_pump_express(mpci, true);
out:
	mutex_unlock(&mpci->pe_lock);
	return ret;
}

static inline int mt6360_get_aicc(struct mt6360_pmu_chg_info *mpci,
				  u32 *aicc_val)
{
	u8 aicc_sel = 0;
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_AICC_RESULT);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read aicc result fail\n", __func__);
		return ret;
	}
	aicc_sel = (ret & MT6360_MASK_RG_AICC_RESULT) >>
						     MT6360_SHFT_RG_AICC_RESULT;
	*aicc_val = (aicc_sel * 50000) + 100000;
	return 0;
}

static inline int mt6360_post_aicc_measure(struct charger_device *chg_dev,
					   u32 start, u32 stop, u32 step,
					   u32 *measure)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int cur, ret;

	mt_dbg(mpci->dev,
		"%s: post_aicc = (%d, %d, %d)\n", __func__, start, stop, step);
	for (cur = start; cur < (stop + step); cur += step) {
		/* set_aicr to cur */
		ret = mt6360_set_aicr(chg_dev, cur + step);
		if (ret < 0)
			return ret;
		usleep_range(150, 200);
		ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
		if (ret < 0)
			return ret;
		/* read mivr stat */
		if (ret & MT6360_MASK_MIVR_EVT)
			break;
	}
	*measure = cur;
	return 0;
}

static int mt6360_run_aicc(struct charger_device *chg_dev, u32 *uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(mpci->dev);
	int ret = 0;
	u32 aicc_val = 0, aicr_val;
	long timeout;
	bool mivr_stat = false;

	mt_dbg(mpci->dev, "%s: aicc_once = %d\n", __func__, pdata->aicc_once);
	/* check MIVR stat is act */
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read mivr stat fail\n", __func__);
		return ret;
	}
	mivr_stat = (ret & MT6360_MASK_MIVR_EVT) ? true : false;
	if (!mivr_stat) {
		dev_err(mpci->dev, "%s: mivr stat not act\n", __func__);
		return ret;
	}

	/* Auto run aicc */
	if (!pdata->aicc_once) {
		if (!try_wait_for_completion(&mpci->aicc_done)) {
			dev_info(mpci->dev, "%s: aicc is not act\n", __func__);
			return 0;
		}

		/* get aicc result */
		ret = mt6360_get_aicc(mpci, &aicc_val);
		if (ret < 0) {
			dev_err(mpci->dev,
				"%s: get aicc fail\n", __func__);
			return ret;
		}
		*uA = aicc_val;
		reinit_completion(&mpci->aicc_done);
		return ret;
	}

	/* Use aicc once method */
	/* Run AICC measure */
	mutex_lock(&mpci->pe_lock);
	ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL14,
				      MT6360_MASK_RG_EN_AICC);
	if (ret < 0)
		goto out;
	/* Clear AICC measurement IRQ */
	reinit_completion(&mpci->aicc_done);
	timeout = wait_for_completion_interruptible_timeout(
				   &mpci->aicc_done, msecs_to_jiffies(3000));
	if (timeout == 0)
		ret = -ETIMEDOUT;
	else if (timeout < 0)
		ret = -EINTR;
	else
		ret = 0;
	if (ret < 0) {
		dev_err(mpci->dev,
			"%s: wait AICC time out, ret = %d\n", __func__, ret);
		goto out;
	}
	/* get aicc_result */
	ret = mt6360_get_aicc(mpci, &aicc_val);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: get aicc result fail\n", __func__);
		goto out;
	}

	if (!pdata->post_aicc)
		goto skip_post_aicc;

	dev_info(mpci->dev, "%s: aicc pre val = %d\n", __func__, aicc_val);
	ret = mt6360_get_aicr(chg_dev, &aicr_val);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: get aicr fail\n", __func__);
		goto out;
	}
	ret = mt6360_set_aicr(chg_dev, aicc_val);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set aicr fail\n", __func__);
		goto out;
	}
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_CTRL14,
				      MT6360_MASK_RG_EN_AICC);
	if (ret < 0)
		goto out;
	/* always start/end aicc_val/aicc_val+200mA */
	ret = mt6360_post_aicc_measure(chg_dev, aicc_val,
				       aicc_val + 200000, 50000, &aicc_val);
	if (ret < 0)
		goto out;
	ret = mt6360_set_aicr(chg_dev, aicr_val);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set aicr fail\n", __func__);
		goto out;
	}
	dev_info(mpci->dev, "%s: aicc post val = %d\n", __func__, aicc_val);
skip_post_aicc:
	*uA = aicc_val;
out:
	/* Clear EN_AICC */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_CTRL14,
				      MT6360_MASK_RG_EN_AICC);
	mutex_unlock(&mpci->pe_lock);
	return ret;
}

static int mt6360_enable_power_path(struct charger_device *chg_dev,
					    bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	dev_dbg(mpci->dev, "%s: en = %d\n", __func__, en);
	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_CHG_CTRL1,
					MT6360_MASK_FORCE_SLEEP, en ? 0 : 0xff);
}

static int mt6360_is_power_path_enabled(struct charger_device *chg_dev,
						bool *en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL1);
	if (ret < 0)
		return ret;
	*en = (ret & MT6360_MASK_FORCE_SLEEP) ? false : true;
	return 0;
}

static int mt6360_enable_safety_timer(struct charger_device *chg_dev,
					      bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	dev_dbg(mpci->dev, "%s: en = %d\n", __func__, en);
	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_CHG_CTRL12,
					  MT6360_MASK_TMR_EN, en ? 0xff : 0);
}

static int mt6360_is_safety_timer_enabled(
				struct charger_device *chg_dev, bool *en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL12);
	if (ret < 0)
		return ret;
	*en = (ret & MT6360_MASK_TMR_EN) ? true : false;
	return 0;
}

static int mt6360_enable_hz(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_info(mpci->dev, "%s: en = %d\n", __func__, en);

	ret = (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_CHG_CTRL1, MT6360_MASK_HZ_EN);

	return ret;
}

static const u32 otg_oc_table[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000
};

static int mt6360_set_otg_current_limit(struct charger_device *chg_dev,
						u32 uA)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int i;

	/* Set higher OC threshold protect */
	for (i = 0; i < ARRAY_SIZE(otg_oc_table); i++) {
		if (uA <= otg_oc_table[i])
			break;
	}
	if (i == ARRAY_SIZE(otg_oc_table))
		i = MT6360_OTG_OC_MAXVAL;
	dev_dbg(mpci->dev,
		"%s: select oc threshold = %d\n", __func__, otg_oc_table[i]);

	return mt6360_pmu_reg_update_bits(mpci->mpi,
					  MT6360_PMU_CHG_CTRL10,
					  MT6360_MASK_OTG_OC,
					  i << MT6360_SHFT_OTG_OC);
}

static int mt6360_enable_otg(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_err(mpci->dev, "%s: en = %d\n", __func__, en);
	ret = mt6360_enable_wdt(mpci, en ? false : true);
	mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_CFO_EN);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set wdt fail, en = %d\n", __func__, en);
		return ret;
	}
	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_CHG_CTRL1,
					  MT6360_MASK_OPA_MODE, en ? 0xff : 0);
}

static int mt6360_enable_discharge(struct charger_device *chg_dev,
					   bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int i, ret = 0;
	const int dischg_retry_cnt = 3;
	bool is_dischg;

	dev_dbg(mpci->dev, "%s: en = %d\n", __func__, en);
	ret = mt6360_enable_hidden_mode(mpci->chg_dev, true);
	if (ret < 0)
		return ret;
	/* Set bit2 of reg[0x31] to 1/0 to enable/disable discharging */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_CHG_HIDDEN_CTRL2,
					 MT6360_MASK_DISCHG, en ? 0xff : 0);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: fail, en = %d\n", __func__, en);
		goto out;
	}

	if (!en) {
		for (i = 0; i < dischg_retry_cnt; i++) {
			ret = mt6360_pmu_reg_read(mpci->mpi,
						  MT6360_PMU_CHG_HIDDEN_CTRL2);
			is_dischg = (ret & MT6360_MASK_DISCHG) ? true : false;
			if (!is_dischg)
				break;
			ret = mt6360_pmu_reg_clr_bits(mpci->mpi,
						MT6360_PMU_CHG_HIDDEN_CTRL2,
						MT6360_MASK_DISCHG);
			if (ret < 0) {
				dev_err(mpci->dev,
					"%s: disable dischg failed\n",
					__func__);
				goto out;
			}
		}
		if (i == dischg_retry_cnt) {
			dev_err(mpci->dev, "%s: dischg failed\n", __func__);
			ret = -EINVAL;
		}
	}
out:
	mt6360_enable_hidden_mode(mpci->chg_dev, false);
	return ret;
}

static int mt6360_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
#if defined(CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT) && defined(CONFIG_TCPC_CLASS)
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	dev_info(mpci->dev, "%s\n", __func__);
	mutex_lock(&mpci->chgdet_lock);
	if (mpci->tcpc_attach == en) {
		dev_info(mpci->dev, "%s attach(%d) is the same\n",
			 __func__, mpci->tcpc_attach);
		goto out;
	}
	mpci->tcpc_attach = en;
	ret = (en ? mt6360_chgdet_pre_process :
		    mt6360_chgdet_post_process)(mpci);
out:
	mutex_unlock(&mpci->chgdet_lock);
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT && CONFIG_TCPC_CLASS */
	return ret;
}

static int mt6360_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	enum mt6360_adc_channel channel;
	int ret = 0;

	switch (chan) {
	case ADC_CHANNEL_VBUS:
		channel = MT6360_ADC_VBUSDIV5;
		break;
	case ADC_CHANNEL_VSYS:
		channel = MT6360_ADC_VSYS;
		break;
	case ADC_CHANNEL_VBAT:
		channel = MT6360_ADC_VBAT;
		break;
	case ADC_CHANNEL_IBUS:
		channel = MT6360_ADC_IBUS;
		break;
	case ADC_CHANNEL_IBAT:
		channel = MT6360_ADC_IBAT;
		break;
	case ADC_CHANNEL_TEMP_JC:
		channel = MT6360_ADC_TEMP_JC;
		break;
	case ADC_CHANNEL_USBID:
		channel = MT6360_ADC_USBID;
		break;
	case ADC_CHANNEL_TS:
		channel = MT6360_ADC_TS;
		break;
	default:
		return -ENOTSUPP;
	}
	ret = iio_read_channel_processed(mpci->channels[channel], min);
	if (ret < 0) {
		dev_info(mpci->dev, "%s: fail(%d)\n", __func__, ret);
		return ret;
	}
	*max = *min;
	return 0;
}

static int mt6360_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	mt_dbg(mpci->dev, "%s\n", __func__);
	return mt6360_get_adc(chg_dev, ADC_CHANNEL_VBUS, vbus, vbus);
}

static int mt6360_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	mt_dbg(mpci->dev, "%s\n", __func__);
	return mt6360_get_adc(chg_dev, ADC_CHANNEL_IBUS, ibus, ibus);
}

static int mt6360_get_ibat(struct charger_device *chg_dev, u32 *ibat)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	mt_dbg(mpci->dev, "%s\n", __func__);
	return mt6360_get_adc(chg_dev, ADC_CHANNEL_IBAT, ibat, ibat);
}

static int mt6360_get_tchg(struct charger_device *chg_dev,
				   int *tchg_min, int *tchg_max)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int temp_jc = 0, ret = 0, retry_cnt = 3;

	mt_dbg(mpci->dev, "%s\n", __func__);
	/* temp abnormal Workaround */
	do {
		ret = mt6360_get_adc(chg_dev, ADC_CHANNEL_TEMP_JC,
				     &temp_jc, &temp_jc);
		if (ret < 0) {
			dev_err(mpci->dev,
				"%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
	} while (temp_jc >= 120 && (retry_cnt--) > 0);
	mutex_lock(&mpci->tchg_lock);
	if (temp_jc >= 120)
		temp_jc = mpci->tchg;
	else
		mpci->tchg = temp_jc;
	mutex_unlock(&mpci->tchg_lock);
	*tchg_min = *tchg_max = temp_jc;
	dev_info(mpci->dev, "%s: tchg = %d\n", __func__, temp_jc);
	return 0;
}

static int mt6360_kick_wdt(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	dev_dbg(mpci->dev, "%s\n", __func__);
	return mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL1);
}

static int mt6360_safety_check(struct charger_device *chg_dev, u32 polling_ieoc)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret, ibat = 0;
	static int eoc_cnt;

	mt_dbg(mpci->dev, "%s\n", __func__);
	ret = iio_read_channel_processed(mpci->channels[MT6360_ADC_IBAT],
					 &ibat);
	if (ret < 0)
		dev_err(mpci->dev, "%s: failed, ret = %d\n", __func__, ret);

	if (ibat <= polling_ieoc)
		eoc_cnt++;
	else
		eoc_cnt = 0;
	/* If ibat is less than polling_ieoc for 3 times, trigger EOC event */
	if (eoc_cnt == 3) {
		dev_info(mpci->dev, "%s: polling_ieoc = %d, ibat = %d\n",
			 __func__, polling_ieoc, ibat);
		charger_dev_notify(mpci->chg_dev, CHARGER_DEV_NOTIFY_EOC);
		eoc_cnt = 0;
	}
	return ret;
}

static int mt6360_reset_eoc_state(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_dbg(mpci->dev, "%s\n", __func__);

	ret = mt6360_enable_hidden_mode(mpci->chg_dev, true);
	if (ret < 0)
		return ret;
	ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_HIDDEN_CTRL1,
				      MT6360_MASK_EOC_RST);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: set failed, ret = %d\n", __func__, ret);
		goto out;
	}
	udelay(100);
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_HIDDEN_CTRL1,
				      MT6360_MASK_EOC_RST);
	if (ret < 0) {
		dev_err(mpci->dev,
			"%s: clear failed, ret = %d\n", __func__, ret);
		goto out;
	}
out:
	mt6360_enable_hidden_mode(mpci->chg_dev, false);
	return ret;
}

static int mt6360_is_charging_done(struct charger_device *chg_dev,
					   bool *done)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	enum mt6360_charging_status chg_stat;
	int ret = 0;

	mt_dbg(mpci->dev, "%s\n", __func__);
	ret = mt6360_get_charging_status(mpci, &chg_stat);
	if (ret < 0)
		return ret;
	*done = (chg_stat == MT6360_CHG_STATUS_DONE) ? true : false;
	return 0;
}

static int mt6360_get_zcv(struct charger_device *chg_dev, u32 *uV)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	dev_info(mpci->dev, "%s: zcv = %dmV\n", __func__, mpci->zcv / 1000);
	*uV = mpci->zcv;
	return 0;
}

static int mt6360_dump_registers(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int i, ret = 0;
	int adc_vals[MT6360_ADC_MAX];
	u32 ichg = 0, aicr = 0, mivr = 0, cv = 0, ieoc = 0;
	enum mt6360_charging_status chg_stat = MT6360_CHG_STATUS_READY;
	bool chg_en = false;
	u8 chg_stat1 = 0, chg_ctrl[2] = {0};

	dev_dbg(mpci->dev, "%s\n", __func__);
	ret = mt6360_get_ichg(chg_dev, &ichg);
	ret |= mt6360_get_aicr(chg_dev, &aicr);
	ret |= mt6360_get_mivr(chg_dev, &mivr);
	ret |= mt6360_get_cv(chg_dev, &cv);
	ret |= mt6360_get_ieoc(mpci, &ieoc);
	ret |= mt6360_get_charging_status(mpci, &chg_stat);
	ret |= mt6360_is_charger_enabled(mpci, &chg_en);
	if (ret < 0) {
		dev_notice(mpci->dev, "%s: parse chg setting fail\n", __func__);
		return ret;
	}
	for (i = 0; i < MT6360_ADC_MAX; i++) {
		/* Skip unnecessary channel */
		if (i >= MT6360_ADC_NODUMP)
			break;
		ret = iio_read_channel_processed(mpci->channels[i],
						 &adc_vals[i]);
		if (ret < 0) {
			dev_err(mpci->dev,
				"%s: read [%s] adc fail(%d)\n",
				__func__, mt6360_adc_chan_list[i], ret);
			return ret;
		}
	}
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
	if (ret < 0)
		return ret;
	chg_stat1 = ret;

	ret = mt6360_pmu_reg_block_read(mpci->mpi, MT6360_PMU_CHG_CTRL1,
					2, chg_ctrl);
	if (ret < 0)
		return ret;
	dev_info(mpci->dev,
		 "%s: ICHG = %dmA, AICR = %dmA, MIVR = %dmV, IEOC = %dmA, CV = %dmV\n",
		 __func__, ichg / 1000, aicr / 1000, mivr / 1000, ieoc / 1000,
		 cv / 1000);
	dev_info(mpci->dev,
		 "%s: VBUS = %dmV, IBUS = %dmA, VSYS = %dmV, VBAT = %dmV, IBAT = %dmA\n",
		 __func__,
		 adc_vals[MT6360_ADC_VBUSDIV5] / 1000,
		 adc_vals[MT6360_ADC_IBUS] / 1000,
		 adc_vals[MT6360_ADC_VSYS] / 1000,
		 adc_vals[MT6360_ADC_VBAT] / 1000,
		 adc_vals[MT6360_ADC_IBAT] / 1000);
	dev_info(mpci->dev, "%s: CHG_EN = %d, CHG_STATUS = %s, CHG_STAT1 = 0x%02X\n",
		 __func__, chg_en, mt6360_chg_status_name[chg_stat], chg_stat1);
	dev_info(mpci->dev, "%s: CHG_CTRL1 = 0x%02X, CHG_CTRL2 = 0x%02X\n",
		 __func__, chg_ctrl[0], chg_ctrl[1]);
	return 0;
}

static int mt6360_do_event(struct charger_device *chg_dev, u32 event,
				   u32 args)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	mt_dbg(mpci->dev, "%s\n", __func__);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}
	return 0;
}

static int mt6360_plug_in(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	union power_supply_propval propval;
	int ret = 0;

	dev_dbg(mpci->dev, "%s\n", __func__);

	ret = mt6360_enable_wdt(mpci, true);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: en wdt failed\n", __func__);
		return ret;
	}
	/* Replace CHG_EN by TE for avoid CV level too low trigger ieoc */
	/* TODO: First select cv, then chg_en, no need ? */
	ret = mt6360_enable_te(chg_dev, true);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: en te failed\n", __func__);
		return ret;
	}

	/* Workaround for ibus stuck in pe/pe20 pattern */
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	if (!mpci->psy)
		mpci->psy = power_supply_get_by_name("charger");
	if (!mpci->psy) {
		dev_notice(mpci->dev,
			"%s: get power supply failed\n", __func__);
		return -EINVAL;
	}
#endif
	ret = power_supply_get_property(mpci->psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE,
					&propval);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: get chg_type fail\n", __func__);
		return ret;
	}
	return ret;
}

static int mt6360_plug_out(struct charger_device *chg_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	int ret = 0;

	dev_dbg(mpci->dev, "%s\n", __func__);
	ret = mt6360_enable_wdt(mpci, false);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable wdt failed\n", __func__);
		return ret;
	}
	ret = mt6360_enable_te(chg_dev, false);
	if (ret < 0)
		dev_err(mpci->dev, "%s: disable te failed\n", __func__);
	return ret;
}

static int mt6360_enable_usbid(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	return (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_USBID_CTRL1, MT6360_MASK_USBID_EN);
}

static int mt6360_set_usbid_rup(struct charger_device *chg_dev, u32 rup)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	u32 data = mt6360_trans_usbid_rup(rup);

	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1,
					  MT6360_MASK_ID_RPULSEL,
					  data << MT6360_SHFT_ID_RPULSEL);
}

static int mt6360_set_usbid_src_ton(struct charger_device *chg_dev, u32 src_ton)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);
	u32 data = mt6360_trans_usbid_src_ton(src_ton);

	return mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1,
					  MT6360_MASK_ISTDET,
					  data << MT6360_SHFT_ISTDET);
}

static int mt6360_enable_usbid_floating(struct charger_device *chg_dev, bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	return (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_USBID_CTRL2, MT6360_MASK_USBID_FLOAT);
}

static int mt6360_enable_force_typec_otp(struct charger_device *chg_dev,
					 bool en)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	return (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_TYPEC_OTP_CTRL,
		 MT6360_MASK_TYPEC_OTP_SWEN);
}

static int mt6360_get_ctd_dischg_status(struct charger_device *chg_dev,
					u8 *status)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(chg_dev);

	*status = mpci->ctd_dischg_status;
	return 0;
}

static const struct charger_ops mt6360_chg_ops = {
	/* cable plug in/out */
	.plug_in = mt6360_plug_in,
	.plug_out = mt6360_plug_out,
	/* enable */
	.enable = mt6360_enable,
	/* charging current */
	.set_charging_current = mt6360_set_ichg,
	.get_charging_current = mt6360_get_ichg,
	.get_min_charging_current = mt6360_get_min_ichg,
	/* charging voltage */
	.set_constant_voltage = mt6360_set_cv,
	.get_constant_voltage = mt6360_get_cv,
	/* charging input current */
	.set_input_current = mt6360_set_aicr,
	.get_input_current = mt6360_get_aicr,
	.get_min_input_current = mt6360_get_min_aicr,
	/* set termination current */
	.set_eoc_current = mt6360_set_ieoc,
	/* charging mivr */
	.set_mivr = mt6360_set_mivr,
	.get_mivr = mt6360_get_mivr,
	.get_mivr_state = mt6360_get_mivr_state,
	/* charing termination */
	.enable_termination = mt6360_enable_te,
	/* PE+/PE+20 */
	.send_ta_current_pattern = mt6360_set_pep_current_pattern,
	.set_pe20_efficiency_table = mt6360_set_pep20_efficiency_table,
	.send_ta20_current_pattern = mt6360_set_pep20_current_pattern,
	.reset_ta = mt6360_reset_ta,
	.enable_cable_drop_comp = mt6360_enable_cable_drop_comp,
	.run_aicl = mt6360_run_aicc,
	/* Power path */
	.enable_powerpath = mt6360_enable_power_path,
	.is_powerpath_enabled = mt6360_is_power_path_enabled,
	/* safety timer */
	.enable_safety_timer = mt6360_enable_safety_timer,
	.is_safety_timer_enabled = mt6360_is_safety_timer_enabled,
	/* OTG */
	.enable_otg = mt6360_enable_otg,
	.set_boost_current_limit = mt6360_set_otg_current_limit,
	.enable_discharge = mt6360_enable_discharge,
	/* Charger type detection */
	.enable_chg_type_det = mt6360_enable_chg_type_det,
	/* ADC */
	.get_adc = mt6360_get_adc,
	.get_vbus_adc = mt6360_get_vbus,
	.get_ibus_adc = mt6360_get_ibus,
	.get_ibat_adc = mt6360_get_ibat,
	.get_tchg_adc = mt6360_get_tchg,
	/* kick wdt */
	.kick_wdt = mt6360_kick_wdt,
	/* misc */
	.safety_check = mt6360_safety_check,
	.reset_eoc_state = mt6360_reset_eoc_state,
	.is_charging_done = mt6360_is_charging_done,
	.get_zcv = mt6360_get_zcv,
	.dump_registers = mt6360_dump_registers,
	/* event */
	.event = mt6360_do_event,
	/* TypeC */
	.enable_usbid = mt6360_enable_usbid,
	.set_usbid_rup = mt6360_set_usbid_rup,
	.set_usbid_src_ton = mt6360_set_usbid_src_ton,
	.enable_usbid_floating = mt6360_enable_usbid_floating,
	.enable_force_typec_otp = mt6360_enable_force_typec_otp,
	.get_ctd_dischg_status = mt6360_get_ctd_dischg_status,
	.enable_hidden_mode = mt6360_enable_hidden_mode,
	.enable_hz = mt6360_enable_hz,
};

static const struct charger_properties mt6360_chg_props = {
	.alias_name = "mt6360_chg",
};

static irqreturn_t mt6360_pmu_chg_treg_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	int ret = 0;

	dev_err(mpci->dev, "%s\n", __func__);
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
	if (ret < 0)
		return ret;
	if ((ret & MT6360_MASK_CHG_TREG) >> MT6360_SHFT_CHG_TREG)
		dev_err(mpci->dev,
			"%s: thermal regulation loop is active\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_aicr_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static void mt6360_pmu_chg_irq_enable(const char *name, int en);
static irqreturn_t mt6360_pmu_chg_mivr_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	mt_dbg(mpci->dev, "%s\n", __func__);
	mt6360_pmu_chg_irq_enable("chg_mivr_evt", 0);
	atomic_inc(&mpci->mivr_cnt);
	wake_up(&mpci->waitq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_pwr_rdy_evt_handler(int irq, void *data)
{
#if 0
	struct mt6360_pmu_chg_info *mpci = data;
	bool pwr_rdy = false;
	int ret = 0;

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
	if (ret < 0)
		return ret;
	pwr_rdy = (ret & MT6360_MASK_PWR_RDY_EVT);
	dev_info(mpci->dev, "%s: pwr_rdy = %d\n", __func__, pwr_rdy);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_batsysuv_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_vsysuv_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_vsysov_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_vbatov_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_vbusov_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	struct chgdev_notify *noti = &(mpci->chg_dev->noti);
#endif
	bool vbusov_stat = false;
	int ret = 0;

	dev_warn(mpci->dev, "%s\n", __func__);
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT2);
	if (ret < 0)
		goto out;
	vbusov_stat = (ret & BIT(7));
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	noti->vbusov_stat = vbusov_stat;
#endif
	dev_info(mpci->dev, "%s: stat = %d\n", __func__, vbusov_stat);
out:
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_wd_pmu_det_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_wd_pmu_done_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_tmri_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	int ret = 0;

	dev_warn(mpci->dev, "%s\n", __func__);
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT4);
	if (ret < 0)
		return IRQ_HANDLED;
	dev_info(mpci->dev, "%s: chg_stat4 = 0x%02x\n", __func__, ret);
	if (!(ret & MT6360_MASK_CHG_TMRI))
		return IRQ_HANDLED;
	charger_dev_notify(mpci->chg_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_adpbadi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_rvpi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_otpi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_aiccmeasl_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	complete(&mpci->aicc_done);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chgdet_donei_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_wdtmri_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	int ret;

	dev_warn(mpci->dev, "%s\n", __func__);
	/* Any I2C R/W can kick watchdog timer */
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_CTRL1);
	if (ret < 0)
		dev_err(mpci->dev, "%s: kick wdt failed\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_ssfinishi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_rechgi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	charger_dev_notify(mpci->chg_dev, CHARGER_DEV_NOTIFY_RECHG);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_termi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chg_ieoci_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	bool ieoc_stat = false;
	int ret = 0;

	dev_info(mpci->dev, "%s\n", __func__);
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT4);
	if (ret < 0)
		goto out;
	ieoc_stat = (ret & BIT(7));
	if (!ieoc_stat)
		goto out;

	charger_dev_notify(mpci->chg_dev, CHARGER_DEV_NOTIFY_EOC);
out:
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_pumpx_donei_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	atomic_set(&mpci->pe_complete, 0);
	complete(&mpci->pumpx_done);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_bst_batuvi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_bst_vbusovi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_bst_olpi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_warn(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_attachi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
#ifdef CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT
	mutex_lock(&mpci->chgdet_lock);
	if (!mpci->bc12_en) {
		dev_err(mpci->dev, "%s: bc12 disabled, ignore irq\n",
			__func__);
		goto out;
	}
	mt6360_chgdet_post_process(mpci);
out:
	mutex_unlock(&mpci->chgdet_lock);
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT */
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_detachi_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_hvdcp_det_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chgdeti_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_dcdti_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;

	dev_info(mpci->dev, "%s\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t mt6360_pmu_chrdet_ext_evt_handler(int irq, void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	int ret = 0;
	bool pwr_rdy = false;

	ret = mt6360_get_chrdet_ext_stat(mpci, &pwr_rdy);
	dev_info(mpci->dev, "%s: pwr_rdy = %d\n", __func__, pwr_rdy);
	if (ret < 0)
		goto out;
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	if (mpci->pwr_rdy == pwr_rdy)
		goto out;
#endif
	mpci->pwr_rdy = pwr_rdy;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	schedule_work(&mpci->irq_work);
	goto out;
#endif
#ifdef CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT
#ifndef CONFIG_TCPC_CLASS
	mutex_lock(&mpci->chgdet_lock);
	(pwr_rdy ? mt6360_chgdet_pre_process :
		   mt6360_chgdet_post_process)(mpci);
	mutex_unlock(&mpci->chgdet_lock);
#endif /* !CONFIG_TCPC_CLASS */
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT */
	if (atomic_read(&mpci->pe_complete) && pwr_rdy == true &&
	    mpci->mpi->chip_rev <= 0x02) {
		dev_info(mpci->dev, "%s: re-trigger pe20 pattern\n", __func__);
		queue_work(mpci->pe_wq, &mpci->pe_work);
	}
out:
	return IRQ_HANDLED;
}

static struct mt6360_pmu_irq_desc mt6360_pmu_chg_irq_desc[] = {
	MT6360_PMU_IRQDESC(chg_treg_evt),
	MT6360_PMU_IRQDESC(chg_aicr_evt),
	MT6360_PMU_IRQDESC(chg_mivr_evt),
	MT6360_PMU_IRQDESC(pwr_rdy_evt),
	MT6360_PMU_IRQDESC(chg_batsysuv_evt),
	MT6360_PMU_IRQDESC(chg_vsysuv_evt),
	MT6360_PMU_IRQDESC(chg_vsysov_evt),
	MT6360_PMU_IRQDESC(chg_vbatov_evt),
	MT6360_PMU_IRQDESC(chg_vbusov_evt),
	MT6360_PMU_IRQDESC(wd_pmu_det),
	MT6360_PMU_IRQDESC(wd_pmu_done),
	MT6360_PMU_IRQDESC(chg_tmri),
	MT6360_PMU_IRQDESC(chg_adpbadi),
	MT6360_PMU_IRQDESC(chg_rvpi),
	MT6360_PMU_IRQDESC(otpi),
	MT6360_PMU_IRQDESC(chg_aiccmeasl),
	MT6360_PMU_IRQDESC(chgdet_donei),
	MT6360_PMU_IRQDESC(wdtmri),
	MT6360_PMU_IRQDESC(ssfinishi),
	MT6360_PMU_IRQDESC(chg_rechgi),
	MT6360_PMU_IRQDESC(chg_termi),
	MT6360_PMU_IRQDESC(chg_ieoci),
	MT6360_PMU_IRQDESC(pumpx_donei),
	MT6360_PMU_IRQDESC(bst_batuvi),
	MT6360_PMU_IRQDESC(bst_vbusovi),
	MT6360_PMU_IRQDESC(bst_olpi),
	MT6360_PMU_IRQDESC(attachi),
	MT6360_PMU_IRQDESC(detachi),
	MT6360_PMU_IRQDESC(hvdcp_det),
	MT6360_PMU_IRQDESC(chgdeti),
	MT6360_PMU_IRQDESC(dcdti),
	MT6360_PMU_IRQDESC(chrdet_ext_evt),
};

static void mt6360_pmu_chg_irq_enable(const char *name, int en)
{
	struct mt6360_pmu_irq_desc *irq_desc;
	int i = 0;

	if (unlikely(!name))
		return;
	for (i = 0; i < ARRAY_SIZE(mt6360_pmu_chg_irq_desc); i++) {
		irq_desc = mt6360_pmu_chg_irq_desc + i;
		if (unlikely(!irq_desc->name))
			continue;
		if (!strcmp(irq_desc->name, name)) {
			if (en)
				enable_irq(irq_desc->irq);
			else
				disable_irq_nosync(irq_desc->irq);
			break;
		}
	}
}

static void mt6360_pmu_chg_irq_register(struct platform_device *pdev)
{
	struct mt6360_pmu_irq_desc *irq_desc;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(mt6360_pmu_chg_irq_desc); i++) {
		irq_desc = mt6360_pmu_chg_irq_desc + i;
		if (unlikely(!irq_desc->name))
			continue;
		ret = platform_get_irq_byname(pdev, irq_desc->name);
		if (ret < 0)
			continue;
		irq_desc->irq = ret;
		ret = devm_request_threaded_irq(&pdev->dev, irq_desc->irq, NULL,
						irq_desc->irq_handler,
						IRQF_TRIGGER_FALLING,
						irq_desc->name,
						platform_get_drvdata(pdev));
		if (ret < 0)
			dev_err(&pdev->dev,
				"request %s irq fail\n", irq_desc->name);
	}
}

static int mt6360_toggle_cfo(struct mt6360_pmu_chg_info *mpci)
{
	int ret = 0;
	u8 data = 0;

	mutex_lock(&mpci->mpi->io_lock);
	/* check if strobe mode */
	ret = i2c_smbus_read_i2c_block_data(mpci->mpi->i2c,
						  MT6360_PMU_FLED_EN, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read cfo fail\n", __func__);
		goto out;
	}
	if (data & MT6360_MASK_STROBE_EN) {
		dev_err(mpci->dev, "%s: fled in strobe mode\n", __func__);
		goto out;
	}
	/* read data */
	ret = i2c_smbus_read_i2c_block_data(mpci->mpi->i2c,
						MT6360_PMU_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read cfo fail\n", __func__);
		goto out;
	}
	/* cfo off */
	data &= ~MT6360_MASK_CFO_EN;
	ret = i2c_smbus_write_i2c_block_data(mpci->mpi->i2c,
						MT6360_PMU_CHG_CTRL2, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: clear cfo fail\n", __func__);
		goto out;
	}
	/* cfo on */
	data |= MT6360_MASK_CFO_EN;
	ret = i2c_smbus_write_i2c_block_data(mpci->mpi->i2c,
						MT6360_PMU_CHG_CTRL2, 1, &data);
	if (ret < 0)
		dev_err(mpci->dev, "%s: set cfo fail\n", __func__);
out:
	mutex_unlock(&mpci->mpi->io_lock);
	return ret;
}

static int mt6360_chg_mivr_task_threadfn(void *data)
{
	struct mt6360_pmu_chg_info *mpci = data;
	u32 ibus;
	int ret;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	return 0;
#endif
	dev_info(mpci->dev, "%s ++\n", __func__);
	while (!kthread_should_stop()) {
		wait_event(mpci->waitq, atomic_read(&mpci->mivr_cnt) > 0);
		mt_dbg(mpci->dev, "%s: enter mivr thread\n", __func__);
		pm_stay_awake(mpci->dev);
		/* check real mivr stat or not */
		ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT1);
		if (ret < 0)
			goto loop_cont;
		if (!(ret & MT6360_MASK_MIVR_EVT)) {
			mt_dbg(mpci->dev, "%s: mivr stat not act\n", __func__);
			goto loop_cont;
		}
		/* read ibus adc */
		ret = mt6360_get_ibus(mpci->chg_dev, &ibus);
		if (ret < 0) {
			dev_err(mpci->dev, "%s: get ibus adc fail\n", __func__);
			goto loop_cont;
		}
		/* if ibus adc value < 100mA), toggle cfo */
		if (ibus < 100000) {
			dev_dbg(mpci->dev, "%s: enter toggle cfo\n", __func__);
			ret = mt6360_toggle_cfo(mpci);
			if (ret < 0)
				dev_err(mpci->dev,
					"%s: toggle cfo fail\n", __func__);
		}
loop_cont:
		pm_relax(mpci->dev);
		atomic_set(&mpci->mivr_cnt, 0);
		mt6360_pmu_chg_irq_enable("chg_mivr_evt", 1);
		msleep(200);
	}
	dev_info(mpci->dev, "%s --\n", __func__);
	return 0;
}

static void mt6360_trigger_pep_work_handler(struct work_struct *work)
{
	struct mt6360_pmu_chg_info *mpci =
		(struct mt6360_pmu_chg_info *)container_of(work,
		struct mt6360_pmu_chg_info, pe_work);
	int ret = 0;

	ret = mt6360_set_pep20_current_pattern(mpci->chg_dev, 5000000);
	if (ret < 0)
		dev_err(mpci->dev, "%s: trigger pe20 pattern fail\n",
			__func__);
}

#if defined(CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT)\
&& !defined(CONFIG_TCPC_CLASS)
static void mt6360_chgdet_work_handler(struct work_struct *work)
{
	int ret = 0;
	bool pwr_rdy = false;
	struct mt6360_pmu_chg_info *mpci =
		(struct mt6360_pmu_chg_info *)container_of(work,
		struct mt6360_pmu_chg_info, chgdet_work);

	mutex_lock(&mpci->chgdet_lock);
	/* Check PWR_RDY_STAT */
	ret = mt6360_get_chrdet_ext_stat(mpci, &pwr_rdy);
	dev_info(info->dev, "pwr_rdy : %d\n", pwr_rdy);
	if (ret < 0)
		goto out;
	/* power not good */
	if (!pwr_rdy)
		goto out;
	/* power good */
	mpci->pwr_rdy = pwr_rdy;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	goto out;
#endif
	/* Turn on USB charger detection */
	ret = __mt6360_enable_usbchgen(mpci, true);
	if (ret < 0)
		dev_err(mpci->dev, "%s: en bc12 fail\n", __func__);
out:
	mutex_unlock(&mpci->chgdet_lock);
}
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT && !CONFIG_TCPC_CLASS */

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
static void mt6360_irq_work(struct work_struct *work)
{
	struct mt6360_pmu_chg_info *mpci =
		(struct mt6360_pmu_chg_info *)container_of(work,
		struct mt6360_pmu_chg_info, irq_work);

	if (!mpci) {
		pr_err("mpci is null, return\n");
	} else if (mpci->callback) {
		pr_err("start\n");
		mpci->callback();
	} else {
		pr_err("mpci->callback is null\n");
	}
}
#endif

static const struct mt6360_pdata_prop mt6360_pdata_props[] = {
	MT6360_PDATA_VALPROP(ichg, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL7, 2, 0xFC,
			     mt6360_trans_ichg_sel, 0),
	MT6360_PDATA_VALPROP(aicr, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL3, 2, 0xFC,
			     mt6360_trans_aicr_sel, 0),
	MT6360_PDATA_VALPROP(mivr, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL6, 1, 0xFE,
			     mt6360_trans_mivr_sel, 0),
	MT6360_PDATA_VALPROP(cv, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL4, 1, 0xFE,
			     mt6360_trans_cv_sel, 0),
	MT6360_PDATA_VALPROP(ieoc, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL9, 4, 0xF0,
			     mt6360_trans_ieoc_sel, 0),
	MT6360_PDATA_VALPROP(safety_timer, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL12, 5, 0xE0,
			     mt6360_trans_safety_timer_sel, 0),
	MT6360_PDATA_VALPROP(ircmp_resistor, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL18, 3, 0x38,
			     mt6360_trans_ircmp_r_sel, 0),
	MT6360_PDATA_VALPROP(ircmp_vclamp, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL18, 0, 0x07,
			     mt6360_trans_ircmp_vclamp_sel, 0),
#if 0
	MT6360_PDATA_VALPROP(en_te, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL2, 4, 0x10, NULL, 0),
	MT6360_PDATA_VALPROP(en_wdt, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL13, 7, 0x80, NULL, 0),
#endif
	MT6360_PDATA_VALPROP(aicc_once, struct mt6360_chg_platform_data,
			     MT6360_PMU_CHG_CTRL14, 0, 0x04, NULL, 0),
};

static int mt6360_chg_apply_pdata(struct mt6360_pmu_chg_info *mpci,
				  struct mt6360_chg_platform_data *pdata)
{
	int ret;

	dev_dbg(mpci->dev, "%s ++\n", __func__);
	ret = mt6360_pdata_apply_helper(mpci->mpi, pdata, mt6360_pdata_props,
					ARRAY_SIZE(mt6360_pdata_props));
	if (ret < 0)
		return ret;
	dev_dbg(mpci->dev, "%s ++\n", __func__);
	return 0;
}

static const struct mt6360_val_prop mt6360_val_props[] = {
	MT6360_DT_VALPROP(ichg, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(aicr, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(mivr, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(cv, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(ieoc, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(safety_timer, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(ircmp_resistor, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(ircmp_vclamp, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(en_te, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(en_wdt, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(aicc_once, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(post_aicc, struct mt6360_chg_platform_data),
	MT6360_DT_VALPROP(batoc_notify, struct mt6360_chg_platform_data),
};

static int mt6360_chg_parse_dt_data(struct device *dev,
				    struct mt6360_chg_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	dev_dbg(dev, "%s ++\n", __func__);
	memcpy(pdata, &def_platform_data, sizeof(*pdata));
	mt6360_dt_parser_helper(np, (void *)pdata,
				mt6360_val_props, ARRAY_SIZE(mt6360_val_props));
	dev_dbg(dev, "%s --\n", __func__);
	return 0;
}

static int mt6360_enable_ilim(struct mt6360_pmu_chg_info *mpci, bool en)
{
	return (en ? mt6360_pmu_reg_set_bits : mt6360_pmu_reg_clr_bits)
		(mpci->mpi, MT6360_PMU_CHG_CTRL3, MT6360_MASK_ILIM_EN);
}

static int mt6360_chg_init_setting(struct mt6360_pmu_chg_info *mpci)
{
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(mpci->dev);
	int ret = 0;
	u32 boot_mode = get_boot_mode();

	dev_info(mpci->dev, "%s\n", __func__);

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHRDET_STAT);
	if (ret >= 0)
		mpci->ctd_dischg_status = ret & 0xE3;
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CTD_CTRL, 0x40);
	if (ret < 0)
		dev_err(mpci->dev, "%s: disable ctd ctrl fail\n", __func__);
	ret = mt6360_select_input_current_limit(mpci, MT6360_IINLMTSEL_AICR);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: select iinlmtsel by aicr fail\n",
			__func__);
		return ret;
	}
	usleep_range(5000, 6000);
	ret = mt6360_enable_ilim(mpci, false);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable ilim fail\n", __func__);
		return ret;
	}
	if (boot_mode == META_BOOT || boot_mode == ADVMETA_BOOT) {
		ret = mt6360_pmu_reg_update_bits(mpci->mpi,
						 MT6360_PMU_CHG_CTRL3,
						 MT6360_MASK_AICR,
						 0x02 << MT6360_SHFT_AICR);
		dev_info(mpci->dev, "%s: set aicr to 200mA in meta mode\n",
			__func__);
	}
	/* disable wdt reduce 1mA power consumption */
	ret = mt6360_enable_wdt(mpci, false);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable wdt fail\n", __func__);
		return ret;
	}
	/* Disable USB charger type detect, no matter use it or not */
	ret = mt6360_enable_usbchgen(mpci, false);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable chg type detect fail\n",
			__func__);
		return ret;
	}
	/* unlock ovp limit for pump express, can be replaced by option */
	ret = mt6360_select_vinovp(mpci, 14500000);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: unlimit vin for pump express\n",
			__func__);
		return ret;
	}
	/* Disable TE, set TE when plug in/out */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_CHG_CTRL2,
				      MT6360_MASK_TE_EN);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: disable te fail\n", __func__);
		return ret;
	}
	/* Read ZCV */
	ret = mt6360_read_zcv(mpci);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read zcv fail\n", __func__);
		return ret;
	}
	/* enable AICC_EN if aicc_once = 0 */
	if (!pdata->aicc_once) {
		ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_CTRL14,
					      MT6360_MASK_RG_EN_AICC);
		if (ret < 0) {
			dev_err(mpci->dev,
				"%s: enable en_aicc fail\n", __func__);
			return ret;
		}
	}
#ifndef CONFIG_MT6360_DCDTOUT_SUPPORT
	/* Disable DCD */
	ret = mt6360_enable_dcd_tout(mpci, false);
	if (ret < 0)
		dev_notice(mpci->dev, "%s disable dcd fail\n", __func__);
#endif
	/* Check BATSYSUV occurred last time boot-on */
	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_CHG_STAT);
	if (ret < 0) {
		dev_err(mpci->dev, "%s: read BATSYSUV fail\n", __func__);
		return ret;
	}
	if (!(ret & MT6360_MASK_CHG_BATSYSUV)) {
		dev_warn(mpci->dev, "%s: BATSYSUV occurred\n", __func__);
		ret = mt6360_pmu_reg_set_bits(mpci->mpi, MT6360_PMU_CHG_STAT,
					      MT6360_MASK_CHG_BATSYSUV);
		if (ret < 0)
			dev_err(mpci->dev,
				"%s: clear BATSYSUV fail\n", __func__);
	}
#ifdef CONFIG_VIVO_USBID_ENABLE
	/*init USBID and enable it*/
	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL2, 0xff, 0x01);
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL2 fail\n", __func__);

	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL3, 0xff, 0xc0);
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL2 fail\n", __func__);

	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1, 0xff, 0xA2);
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL2 fail\n", __func__);
#else
	/* USBID ID_TD = 32T */
	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL2,
					 MT6360_MASK_IDTD |
					 MT6360_MASK_USBID_FLOAT, 0x62);
#endif
	/* Disable TypeC OTP for check EVB version by TS pin */
	ret = mt6360_pmu_reg_clr_bits(mpci->mpi, MT6360_PMU_TYPEC_OTP_CTRL,
				      MT6360_MASK_TYPEC_OTP_EN);
	return ret;
}

static int mt6360_set_shipping_mode(struct mt6360_pmu_chg_info *mpci)
{
	struct mt6360_pmu_info *mpi = mpci->mpi;
	int ret;
	u8 data = 0;
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	return 0;
#endif
	dev_info(mpci->dev, "%s\n", __func__);
	mutex_lock(&mpi->io_lock);
	/* disable shipping mode rst */
	ret = i2c_smbus_read_i2c_block_data(mpi->i2c,
					    MT6360_PMU_CORE_CTRL2, 1, &data);
	if (ret < 0)
		goto out;
	data |= MT6360_MASK_SHIP_RST_DIS;
	dev_info(mpci->dev, "%s: reg[0x06] = 0x%02x\n", __func__, data);
	ret = i2c_smbus_write_i2c_block_data(mpi->i2c,
					     MT6360_PMU_CORE_CTRL2, 1, &data);
	if (ret < 0) {
		dev_err(mpci->dev,
			"%s: fail to disable shipping mode rst\n", __func__);
		goto out;
	}

	data = 0x80;
	/* enter shipping mode and disable cfo_en/chg_en */
	ret = i2c_smbus_write_i2c_block_data(mpi->i2c,
					     MT6360_PMU_CHG_CTRL2, 1, &data);
	if (ret < 0)
		dev_err(mpci->dev,
			"%s: fail to enter shipping mode\n", __func__);
	return 0;
out:
	mutex_unlock(&mpi->io_lock);
	return ret;
}

static ssize_t shipping_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mt6360_pmu_chg_info *mpci = dev_get_drvdata(dev);
	int32_t tmp = 0;
	int ret = 0;

	if (kstrtoint(buf, 10, &tmp) < 0) {
		dev_notice(dev, "parsing number fail\n");
		return -EINVAL;
	}
	if (tmp != 5526789)
		return -EINVAL;
	ret = mt6360_set_shipping_mode(mpci);
	if (ret < 0)
		return ret;
	return count;
}
static const DEVICE_ATTR_WO(shipping_mode);

void mt6360_recv_batoc_callback(BATTERY_OC_LEVEL tag)
{
	int ret, cnt = 0;

	if (tag != BATTERY_OC_LEVEL_1)
		return;
	while (!pmic_get_register_value(PMIC_RG_INT_STATUS_FG_CUR_H)) {
		if (cnt >= 1) {
			ret = mt6360_set_shipping_mode(g_mpci);
			if (ret < 0)
				dev_err(g_mpci->dev,
					"%s: set shipping mode fail\n",
					__func__);
			else
				dev_info(g_mpci->dev,
					 "%s: set shipping mode done\n",
					 __func__);
		}
		mdelay(8);
		cnt++;
	}
	dev_info(g_mpci->dev, "%s exit, cnt = %d, FG_CUR_H = %d\n",
		 __func__, cnt,
		 pmic_get_register_value(PMIC_RG_INT_STATUS_FG_CUR_H));
}

#ifdef CONFIG_DEBUG_FS
static int set_usbid_ctrl1(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL1, 0xff, regval);
	if (ret)
		pr_err("set_usbid_ctrl1 err\n");
	return 0;
}
static int get_usbid_ctrl1(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL1);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usbid_ctrl1_fops, get_usbid_ctrl1, set_usbid_ctrl1, "%01llx\n");

static int set_usbid_ctrl2(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL2, 0xff, regval);
	if (ret)
		pr_err("set_usbid_ctrl2 err\n");
	return 0;
}
static int get_usbid_ctrl2(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL2);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usbid_ctrl2_fops, get_usbid_ctrl2, set_usbid_ctrl2, "%01llx\n");

static int set_usbid_ctrl3(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL3, 0xff, regval);
	if (ret)
		pr_err("set_usbid_ctrl3 err\n");
	return 0;
}
static int get_usbid_ctrl3(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL3);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usbid_ctrl3_fops, get_usbid_ctrl3, set_usbid_ctrl3, "%01llx\n");

static int get_usbid_adc(void *data, u64 *val)
{
	int adc = 0;
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	mt6360_get_adc(chip->chg_dev, ADC_CHANNEL_USBID, &adc, &adc);
	*(u64 *)val = adc;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usbid_adc_fops, get_usbid_adc, NULL, "%01llx\n");

static int set_rst1(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_RST1, 0xff, regval);
	if (ret)
		pr_err("set rst1 err\n");
	return 0;
}
static int get_rst1(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_RST1);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rst1_fops, get_rst1, set_rst1, "%01llx\n");

static int set_adc_en2(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_ADC_EN2, 0xff, regval);
	if (ret)
		pr_err("set adc_en2 err\n");
	return 0;
}
static int get_adc_en2(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_ADC_EN2);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adc_en2_fops, get_adc_en2, set_adc_en2, "%01llx\n");

static int set_base_irq(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_IRQ, 0xff, regval);
	if (ret)
		pr_err("set base_irq err\n");
	return 0;
}
static int get_base_irq(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_IRQ);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(base_irq_fops, get_base_irq, set_base_irq, "%01llx\n");

static int set_base_state(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_STAT, 0xff, regval);
	if (ret)
		pr_err("set base_state err\n");
	return 0;
}
static int get_base_state(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_STAT);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(base_state_fops, get_base_state, set_base_state, "%01llx\n");

static int set_base_mask(void *data, u64 val)
{
	int ret = 0;
	uint8_t regval = (uint8_t)(val & 0xff);
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	pr_info("[%lld,%0x]\n", val, regval);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_MASK, 0xff, regval);
	if (ret)
		pr_err("set base_mask err\n");
	return 0;
}
static int get_base_mask(void *data, u64 *val)
{
	struct mt6360_pmu_chg_info *chip = (struct mt6360_pmu_chg_info *)data;

	*(u64 *)val = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_MASK);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(base_mask_fops, get_base_mask, set_base_mask, "%01llx\n");

static int create_debugfs_entries(struct mt6360_pmu_chg_info *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("mt6360_chg", NULL);
	if (!chip->debug_root) {
		pr_info("Couldn't create debug dir\n");
	} else {
		ent = debugfs_create_file("usbid_ctrl1_0x6D", 0664, chip->debug_root, chip, &usbid_ctrl1_fops);
		if (!ent)
			pr_info("Couldn't create usbid_ctrl1 debug file\n");
		ent = debugfs_create_file("usbid_ctrl2_0x6E", 0664, chip->debug_root, chip, &usbid_ctrl2_fops);
		if (!ent)
			pr_info("Couldn't create usbid_ctrl2 debug file\n");
		ent = debugfs_create_file("usbid_ctrl3_0x6F", 0664, chip->debug_root, chip, &usbid_ctrl3_fops);
		if (!ent)
			pr_info("Couldn't create usbid_ctrl3 debug file\n");
		ent = debugfs_create_file("usbid_adc_0x6F", 0664, chip->debug_root, chip, &usbid_adc_fops);
		if (!ent)
			pr_info("Couldn't create usbid_adc debug file\n");
		ent = debugfs_create_file("rst1_0x02", 0664, chip->debug_root, chip, &rst1_fops);
		if (!ent)
			pr_info("Couldn't create rst1 debug file\n");
		ent = debugfs_create_file("adc_en2_0x57", 0664, chip->debug_root, chip, &adc_en2_fops);
		if (!ent)
			pr_info("Couldn't create adc_en2 debug file\n");
		ent = debugfs_create_file("base_irq_0xD8", 0664, chip->debug_root, chip, &base_irq_fops);
		if (!ent)
			pr_info("Couldn't create base_irq debug file\n");
		ent = debugfs_create_file("base_state_0xE8", 0664, chip->debug_root, chip, &base_state_fops);
		if (!ent)
			pr_info("Couldn't create base_state debug file\n");
		ent = debugfs_create_file("base_mask_0xF8", 0664, chip->debug_root, chip, &base_mask_fops);
		if (!ent)
			pr_info("Couldn't create base_mask debug file\n");
	}
	return 0;
}

#else

static ssize_t usbid_ctrl1_0x6D_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL1));
}

static ssize_t usbid_ctrl1_0x6D_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL1, 0xff, value);
	if (ret)
		pr_err("set usbid_ctrl1 err\n");
	return count;
}
static CLASS_ATTR_RW(usbid_ctrl1_0x6D);

static ssize_t usbid_ctrl2_0x6E_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL2));
}

static ssize_t usbid_ctrl2_0x6E_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL2, 0xff, value);
	if (ret)
		pr_err("set usbid_ctrl2 err\n");
	return count;
}
static CLASS_ATTR_RW(usbid_ctrl2_0x6E);

static ssize_t usbid_ctrl3_0x6F_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USBID_CTRL3));
}

static ssize_t usbid_ctrl3_0x6F_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_USBID_CTRL3, 0xff, value);
	if (ret)
		pr_err("set usbid_ctrl3 err\n");
	return count;
}
static CLASS_ATTR_RW(usbid_ctrl3_0x6F);

static ssize_t usbid_adc_show(struct class *c, struct class_attribute *attr, char *buf)
{
	int adc = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	mt6360_get_adc(chip->chg_dev, ADC_CHANNEL_USBID, &adc, &adc);
	return scnprintf(buf, PAGE_SIZE, "%x\n", adc);
}
static CLASS_ATTR_RO(usbid_adc);

static ssize_t rst1_0x02_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_RST1));
}

static ssize_t rst1_0x02_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_RST1, 0xff, value);
	if (ret)
		pr_err("set rst1 err\n");
	return count;
}
static CLASS_ATTR_RW(rst1_0x02);

static ssize_t adc_en2_0x57_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_ADC_EN2));
}

static ssize_t adc_en2_0x57_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_ADC_EN2, 0xff, value);
	if (ret)
		pr_err("set adc_en2 err\n");
	return count;
}
static CLASS_ATTR_RW(adc_en2_0x57);

static ssize_t base_irq_0xD8_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_IRQ));
}

static ssize_t base_irq_0xD8_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_IRQ, 0xff, value);
	if (ret)
		pr_err("set base_irq err\n");
	return count;
}
static CLASS_ATTR_RW(base_irq_0xD8);

static ssize_t base_state_0xE8_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_STAT));
}

static ssize_t base_state_0xE8_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_STAT, 0xff, value);
	if (ret)
		pr_err("set base_state err\n");
	return count;
}
static CLASS_ATTR_RW(base_state_0xE8);

static ssize_t base_mask_0xF8_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);
	return scnprintf(buf, PAGE_SIZE, "%x\n", mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_BASE_MASK));
}

static ssize_t base_mask_0xF8_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	u8 value = 0;
	struct mt6360_pmu_chg_info *chip = container_of(c, struct mt6360_pmu_chg_info, mt6360_chg_debug);

	if (kstrtou8(buf, 10, &value))
		return -EINVAL;

	pr_info("buf:%s, value=%x\n", value);
	ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_BASE_MASK, 0xff, value);
	if (ret)
		pr_err("set base_mask err\n");
	return count;
}
static CLASS_ATTR_RW(base_mask_0xF8);

static struct attribute *mt6360_chg_debug_attrs[] = {
	&class_attr_usbid_ctrl1_0x6D.attr,
	&class_attr_usbid_ctrl2_0x6E.attr,
	&class_attr_usbid_ctrl3_0x6F.attr,
	&class_attr_usbid_adc.attr,
	&class_attr_rst1_0x02.attr,
	&class_attr_adc_en2_0x57.attr,
	&class_attr_base_irq_0xD8.attr,
	&class_attr_base_state_0xE8.attr,
	&class_attr_base_mask_0xF8.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mt6360_chg_debug);

static int mt6360_chg_debug_init(struct mt6360_pmu_chg_info *chip)
{
	int ret;

	chip->mt6360_chg_debug.name = "mt6360_chg_debug";
	chip->mt6360_chg_debug.class_groups = mt6360_chg_debug_groups;
	ret = class_register(&chip->mt6360_chg_debug);
	if (ret < 0)
		pr_err("Failed to create mt6360_chg_debug rc=%d\n", ret);
	else
		pr_info("Successfully create mt6360_chg_debug!\n");

	return ret;
}
#endif

#ifdef CONFIG_VIVO_USBID_ENABLE
#ifndef CONFIG_MTK_TYPEC_WATER_DETECT
int vivo_get_usbid_present(struct charger_device *charger_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(charger_dev);
	int ret = 0;

	if (!mpci || !mpci->mpi) {
		pr_err("mpci or mpci->mpi is NULL\n");
		return 0;
	}

	ret = mt6360_pmu_reg_read(mpci->mpi, MT6360_PMU_BASE_STAT);
	pr_err("MT6360_PMU_BASE_STAT = 0x%X\n", ret);
	if (ret < 0) {
		pr_err("read MT6360_PMU_BASE_STAT error\n", ret);
		return 0;
	}

	if (ret & BIT(0))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(vivo_get_usbid_present);

int vivo_usbid_setting_first(struct charger_device *charger_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(charger_dev);
	int ret = 0;

	if (!mpci || !mpci->mpi) {
		pr_err("mpci or mpci->mpi is NULL\n");
		return 0;
	}

	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1, 0xff, 0xA2);
	pr_err("SET MT6360_PMU_USBID_CTRL1 = 0xA2\n");
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL1 fail\n", __func__);

	return ret;
}
EXPORT_SYMBOL(vivo_usbid_setting_first);

int vivo_usbid_setting_second(struct charger_device *charger_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(charger_dev);
	int ret = 0;

	if (!mpci || !mpci->mpi) {
		pr_err("mpci or mpci->mpi is NULL\n");
		return 0;
	}

	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1, 0xff, 0xC2);
	pr_err("SET MT6360_PMU_USBID_CTRL1 = 0xC2\n");
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL1 fail\n", __func__);

	return ret;

}
EXPORT_SYMBOL(vivo_usbid_setting_second);

int vivo_usbid_open_cc(struct charger_device *charger_dev)
{
	struct mt6360_pmu_chg_info *mpci = charger_get_data(charger_dev);
	int ret = 0;

	if (!mpci || !mpci->mpi) {
		pr_err("mpci or mpci->mpi is NULL\n");
		return 0;
	}

	ret = mt6360_pmu_reg_update_bits(mpci->mpi, MT6360_PMU_USBID_CTRL1, 0xff, 0x9E);
	pr_err("SET MT6360_PMU_USBID_CTRL1 = 0x9E\n");
	if (ret < 0)
		dev_err(mpci->dev, "%s: MT6360_PMU_USBID_CTRL1 fail\n", __func__);

	return ret;

}
EXPORT_SYMBOL(vivo_usbid_open_cc);

#endif
#endif

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
static void mt6360_pmu_chg_cfo_check(struct mt6360_pmu_chg_info *chip)
{
	int ctrl2 = 0;

	ctrl2 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL2);
	if (!(ctrl2 & MT6360_MASK_CFO_EN)) {
		pr_info("ctrl2=%x, recover cfo\n", ctrl2);
		mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_CFO_EN);
	}
}

static bool mt6360_pmu_chg_hw_component_detect(struct mt6360_pmu_chg_info *chip)
{
	int regval = 0, vendor = 0, ctrl2 = 0;

	regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_DEV_INFO);
	if (regval >= 0) {
		vendor = ((regval >> 4) & 0xf);
		chip->exist = (vendor == MT6360_PMU_CHG_VENDOR_ID);

		ctrl2 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL2);
		if (!(ctrl2 & MT6360_MASK_CFO_EN)) {
			pr_info("ctrl2=%x, recover cfo\n", ctrl2);
			mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_CFO_EN);
		}
	} else {
		chip->exist = false;
	}
	pr_info("vendor_id=0x%02x,ic_exist=%d\n", vendor, chip->exist);
	return chip->exist;
}

static int mt6360_pmu_chg_io_health(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;

	if (io == GET_OUT) {
		if (mt6360_pmu_chg_hw_component_detect(chip)) {
			val->intval = 1;
			fuelsummary_collect_value(ID_BIT__M_I2C_ERR, 0);
		} else {
			fuelsummary_collect_value(ID_BIT__M_I2C_ERR, 1);
			val->intval = 0;
		}
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
	return ret;
}

static int mt6360_pmu_chg_io_status(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;
	enum mt6360_charging_status chg_stat = MT6360_CHG_STATUS_READY;

	if (io == GET_OUT) {
		ret = mt6360_get_charging_status(chip, &chg_stat);
		if (ret < 0)
			goto exit;

		switch (chg_stat) {
		case MT6360_CHG_STATUS_READY:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case MT6360_CHG_STATUS_PROGRESS:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case MT6360_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case MT6360_CHG_STATUS_FAULT:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			break;
		}
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
exit:
	return ret;
}

static int mt6360_pmu_chg_io_charge_detect(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == GET_OUT) {
#if 1
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHRDET_STAT);
		if (regval < 0)
			goto exit;

		if (regval & BIT(4))
			val->intval = 1;
		else
			val->intval = 0;
#else
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL1);
		if (regval < 0)
			goto exit;
		if (regval & MT6360_MASK_OPA_MODE) {
			val->intval = 0;
		} else {
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHRDET_STAT);
			if (regval < 0)
				goto exit;

			if (regval & BIT(4))
				val->intval = 1;
			else
				val->intval = 0;
		}
#endif
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
exit:
	return ret;
}

static int mt6360_pmu_chg_io_charge_voltage(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, vbus = 0;

	if (io == GET_OUT) {
		ret = mt6360_get_vbus(chip->chg_dev, &vbus);
		if (ret < 0)
			goto exit;

		val->intval = vbus / 1000;
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
exit:
	return ret;
}

static int mt6360_pmu_chg_io_charge_type(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, i = 0, regval = 0;
	bool real_bc12 = true;
	uint8_t usb_status = 0;
	union power_supply_propval usb_attach = {0,};
	const int max_retry_cnt = 8;
	static bool first_in = true;

	if (io == GET_OUT) {
		if (!wake_lock_active(&chip->bc12_wlock))
			wake_lock(&chip->bc12_wlock);

		ret = mt6360_pmu_chg_io_charge_detect(chip, GET_OUT, &usb_attach);
		if (ret < 0 || usb_attach.intval == 0) {
			pr_err("no charging stat or read err\n");
			chip->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
			val->intval = chip->chg_type;
			ret = 0;
			goto exit;
		}

		if (val->intval == 0) {
			val->intval = chip->chg_type;
			goto exit;
		}

		if (first_in) {
			first_in = false;
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USB_STATUS1);
			usb_status = (regval & MT6360_MASK_USB_STATUS) >> MT6360_SHFT_USB_STATUS;
			pr_info("charging start usb_stats=0x%x\n", usb_status);
			if (usb_status > MT6360_CHG_TYPE_UNDER_GOING && usb_status < MT6360_CHG_TYPE_MAX)
				real_bc12 = false;
		}

		if (real_bc12) {
			/* Toggle chgdet flow */
			mt6360_set_usbsw_state(chip, MT6360_USBSW_CHG);
			platform_dpdm_hiz(true);

			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_DEVICE_TYPE, MT6360_MASK_USBCHGEN);
			msleep(10);
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_DEVICE_TYPE, MT6360_MASK_USBCHGEN);
			for (i = 0; i < max_retry_cnt; i++) {
				msleep(50);
				regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_USB_STATUS1);
				usb_status = (regval & MT6360_MASK_USB_STATUS) >> MT6360_SHFT_USB_STATUS;
				pr_info("usb_stats=0x%x\n", usb_status);
				if (usb_status != MT6360_CHG_TYPE_UNDER_GOING)
					break;
			}
			if (i == max_retry_cnt)
				pr_err("bc12 failed\n");

			mt6360_set_usbsw_state(chip, MT6360_USBSW_USB);
			platform_dpdm_hiz(false);
		}

		switch (usb_status) {
		case MT6360_CHG_TYPE_SDP:
			chip->chg_type = POWER_SUPPLY_TYPE_USB;
			break;
		case MT6360_CHG_TYPE_SDPNSTD:
			chip->chg_type = POWER_SUPPLY_TYPE_APPLE_BRICK_ID;
			break;
		case MT6360_CHG_TYPE_CDP:
			chip->chg_type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case MT6360_CHG_TYPE_DCP:
			chip->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		default:
			chip->chg_type = POWER_SUPPLY_TYPE_USB_FLOAT;
			break;
		}
		val->intval = chip->chg_type;
		pr_info("usb_attach=%d,chg_type=%d,usb_stats=0x%x\n", usb_attach, chip->chg_type, usb_status);
	} else {
		if (val->intval == 0)
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_DEVICE_TYPE, MT6360_MASK_USBCHGEN);
		else
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_DEVICE_TYPE, MT6360_MASK_USBCHGEN);
		pr_info("set MT6360_PMU_DEVICE_TYPE: %d, ret:%d\n", val->intval, ret);
		/*disable DPDM register ctrl, set DPDM hiz*/
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x0f, 0);
		ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, BIT(4));
	}
exit:
	if (wake_lock_active(&chip->bc12_wlock))
		wake_unlock(&chip->bc12_wlock);
	return ret;
}

static int mt6360_pmu_chg_io_charging_type(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;
	enum mt6360_charging_status chg_stat = MT6360_CHG_STATUS_READY;

	if (io == GET_OUT) {
		ret = mt6360_get_charging_status(chip, &chg_stat);
		if (ret < 0)
			goto exit;

		switch (chg_stat) {
		case MT6360_CHG_STATUS_PROGRESS:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case MT6360_CHG_STATUS_DONE:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case MT6360_CHG_STATUS_READY:
		case MT6360_CHG_STATUS_FAULT:
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
exit:
	return ret;
}

static int mt6360_pmu_chg_io_force_dpdm(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, i = 0, regval = 0, vbus = 0;
	const int max_wait_cnt = 5;
	union power_supply_propval usb_attach = {0,};

	if (io == SET_IN) {
		if (val->intval != 0) {
			pr_info("to hv\n");
			ret = mt6360_get_vbus(chip->chg_dev, &vbus);
			if (vbus > VBUS_MIN && vbus <= VBUS_HV_MIN) {
				regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_QC_CTRL1);
				if (regval & BIT(7)) {
					ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_QC_CTRL1, BIT(7));
					msleep(50);
				}
				ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_QC_CTRL1, BIT(7));
				msleep(1500);
				for (i = 0; i < max_wait_cnt; i++) {
					ret = mt6360_pmu_chg_io_charge_detect(chip, GET_OUT, &usb_attach);
					if (ret < 0 || usb_attach.intval == 0) {
						pr_err("no charging stat or read err\n");
						chip->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
						ret = 0;
						goto exit;
					}

					regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_DPDM_IRQ);
					pr_info("MT6360_PMU_DPDM_IRQ=0x%02x\n", ret);
					if (regval & BIT(4))
						break;

					msleep(50);
				}
				mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_QC_CTRL1, MT6360_MASK_QC20_REQ_SET, 0x10);
				for (i = 0; i < max_wait_cnt; i++) {
					msleep(80);
					ret = mt6360_get_vbus(chip->chg_dev, &vbus);
					if (vbus > VBUS_HV_MIN) {
						chip->chg_type = POWER_SUPPLY_TYPE_USB_HVDCP;
						pr_err("hv charger[%d] success\n", vbus);
						break;
					} else if (vbus > VBUS_MIN) {
						pr_err("hv charger[%d] fail\n", vbus);
					} else {
						pr_err("vbus = %d, abort\n", vbus);
						break;
					}
				}
				#if 1
				//ret = mt6360_get_vbus(chip->chg_dev, &vbus);
				if (vbus > VBUS_MIN && vbus <= VBUS_HV_MIN) {
					/* it is DCP, set DpDm 0V */
					ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x10, 0x1 << 4);
					ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x0f, 0x3);
					pr_info("fail to 9V, set DpDm 0V\n");
				}
				#endif
			} else {
				chip->chg_type = POWER_SUPPLY_TYPE_USB_HVDCP;
			}
		} else {
			pr_info("hv drop\n");
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_QC_CTRL1, BIT(7));
			if (chip->chg_type == POWER_SUPPLY_TYPE_USB_HVDCP)
				chip->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_QC_CTRL1);
		if (regval < 0) {
			ret = -EIO;
			pr_err("read err\n");
		} else {
			val->intval = regval & BIT(7) >> 7;
		}
	}
exit:
	return ret;
}

static int mt6360_pmu_chg_io_disable_charge(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		mt6360_pmu_chg_cfo_check(chip);
		if (val->intval != 0)
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_CHG_EN);
		else
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_CHG_EN);
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL2);
		if (regval & MT6360_MASK_CHG_EN)
			val->intval = 0;
		else
			val->intval = 1;
	}
	return ret;
}

static int mt6360_pmu_chg_io_input_suspend(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		mt6360_pmu_chg_cfo_check(chip);
		pr_info("input_suspend:%d\n", val->intval);
		if (val->intval != 0) {
			if (!chip->shutdown_ignore_hiz)
				ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL1, MT6360_MASK_FORCE_SLEEP);
			else
				pr_info("shutdown ignore hiz\n");
		} else {
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL1, MT6360_MASK_FORCE_SLEEP);
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL1);
		if (regval & MT6360_MASK_FORCE_SLEEP)
			val->intval = 1;
		else
			val->intval = 0;
	}
	return ret;
}

static int mt6360_pmu_chg_io_inbat(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, inbat_ua = 0;

	if (io == SET_IN) {
		inbat_ua = val->intval * 1000;
		ret = mt6360_set_ichg(chip->chg_dev, inbat_ua);
	} else {
		ret = mt6360_get_ichg(chip->chg_dev, &inbat_ua);
		val->intval = inbat_ua / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_input(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, input_ua = 0;

	if (io == SET_IN) {
		input_ua = val->intval * 1000;
		ret = mt6360_set_aicr(chip->chg_dev, input_ua);
	} else {
		ret = mt6360_get_aicr(chip->chg_dev, &input_ua);
		val->intval = input_ua / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_iterm(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, iterm_ua = 0;

	if (io == SET_IN) {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL2);
		if ((regval & MT6360_MASK_TE_EN) == 0x0) {
			pr_info("iterm enable\n");
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL2, MT6360_MASK_TE_EN);
		}

		iterm_ua = val->intval * 1000;
		ret = mt6360_set_ieoc(chip->chg_dev, iterm_ua);
	} else {
		ret = mt6360_get_ieoc(chip, &iterm_ua);
		val->intval = iterm_ua / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_vreg(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, vreg_uv = 0;

	if (io == SET_IN) {
		vreg_uv = val->intval * 1000;
		ret = mt6360_set_cv(chip->chg_dev, vreg_uv);
	} else {
		ret = mt6360_get_cv(chip->chg_dev, &vreg_uv);
		val->intval = vreg_uv / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_ico(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, ico_ua = 0;

	if (io == SET_IN) {
		if (val->intval == 1) {
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL14);
			if ((regval & BIT(2)) == 0x0) {
				pr_err("not aicc once\n");
				ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL14, BIT(2));
			}
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL14);
			if (regval & MT6360_MASK_RG_EN_AICC) {
				pr_err("aicc reenable\n");
				ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL14, MT6360_MASK_RG_EN_AICC);
				msleep(10);
			}
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL14, MT6360_MASK_RG_EN_AICC);
		} else if (val->intval == 0) {
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL14, MT6360_MASK_RG_EN_AICC);
		}
	} else {
		ret = mt6360_get_aicc(chip, &ico_ua);
		val->intval = ico_ua / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_ico_status(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == GET_OUT) {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT5);
		if (regval & BIT(0)) {
			val->intval = 1;
			//ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL14, MT6360_MASK_RG_EN_AICC);
		} else {
			val->intval = 0;
		}
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
	return ret;
}

static int mt6360_pmu_chg_io_vindpm(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, vindpm_uv = 0;

	if (io == SET_IN) {
		vindpm_uv = val->intval * 1000;
		ret = mt6360_set_mivr(chip->chg_dev, vindpm_uv);
	} else {
		ret = mt6360_get_mivr(chip->chg_dev, &vindpm_uv);
		val->intval = vindpm_uv / 1000;
	}
	return ret;
}

static int mt6360_pmu_chg_io_vindpm_status(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == GET_OUT) {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT1);
		if (regval & MT6360_MASK_MIVR_EVT)
			val->intval = 1;
		else
			val->intval = 0;
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
	return ret;
}

static int mt6360_pmu_chg_io_iindpm_status(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == GET_OUT) {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT1);
		if (regval & BIT(5))
			val->intval = 1;
		else
			val->intval = 0;
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
	return ret;
}

static int mt6360_pmu_chg_io_dump_reg(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, i = 0;
	int adc_vals[MT6360_ADC_MAX];
	int ichg = 0, aicr = 0, mivr = 0, cv = 0, ieoc = 0;
	int chg_stat1 = 0, chg_stat2 = 0, chg_stat3 = 0, chg_stat4 = 0, chg_stat5 = 0, chg_stat6 = 0;
	int chg_ctrl1 = 0, chg_ctrl2 = 0, chg_ctrl3 = 0;
	int chg_ctrl10 = 0, chg_ctrl11 = 0, chg_ctrl12 = 0, chg_ctrl13 = 0, chg_ctrl14 = 0, chg_ctrl15 = 0, chg_ctrl16 = 0;
	int dev_type = 0, qc_ctrl = 0, dpdm_ctrl = 0, shdn_ctrl = 0, chgdet_ctrl1 = 0, chgdet_ctrl2 = 0, chg_ctrl18 = 0, chg_ctrl19 = 0;
	enum mt6360_charging_status chg_stat = MT6360_CHG_STATUS_READY;
	uint8_t str[384];

	if (io == GET_OUT) {
		memset(chip->dump_info, 0x0, DUMP_INFO_SIZE);
		ret = mt6360_get_ichg(chip->chg_dev, &ichg);
		ret |= mt6360_get_aicr(chip->chg_dev, &aicr);
		ret |= mt6360_get_mivr(chip->chg_dev, &mivr);
		ret |= mt6360_get_cv(chip->chg_dev, &cv);
		ret |= mt6360_get_ieoc(chip, &ieoc);
		ret |= mt6360_get_charging_status(chip, &chg_stat);
		for (i = 0; i < MT6360_ADC_MAX; i++) {
			if (i >= MT6360_ADC_NODUMP)
				break;
			ret = iio_read_channel_processed(chip->channels[i], &adc_vals[i]);
		}
		chg_stat1 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT1);
		chg_stat2 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT2);
		chg_stat3 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT3);
		chg_stat4 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT4);
		chg_stat5 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT5);
		chg_stat6 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_STAT6);

		chg_ctrl1 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL1);
		chg_ctrl2 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL2);
		chg_ctrl3 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL3);

		chg_ctrl10 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL10);
		chg_ctrl11 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL11);
		chg_ctrl12 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL12);
		chg_ctrl13 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL13);
		chg_ctrl14 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL14);
		chg_ctrl15 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL15);
		chg_ctrl16 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL16);

		dev_type = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_DEVICE_TYPE);
		qc_ctrl = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_QC_CTRL1);
		dpdm_ctrl = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_DPDM_CTRL);
		shdn_ctrl = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_SHDN_CTRL);
		chgdet_ctrl1 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHRDET_CTRL1);
		chgdet_ctrl2 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHRDET_CTRL2);
		chg_ctrl18 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL18);
		chg_ctrl19 = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL19);

		memset(str, 0x0, sizeof(str));
		sprintf(str, "ichg=%d,aicr=%d,mivr=%d,ieoc=%d,cv=%d,",
				ichg / 1000, aicr / 1000, mivr / 1000, ieoc / 1000, cv / 1000);
		strcat(chip->dump_info, str);

		memset(str, 0x0, sizeof(str));
		sprintf(str, "vbus=%d,ibus=%d,vsys=%d,vbat=%d,ibat=%d,",
				adc_vals[MT6360_ADC_VBUSDIV5] / 1000, adc_vals[MT6360_ADC_IBUS] / 1000,
				adc_vals[MT6360_ADC_VSYS] / 1000, adc_vals[MT6360_ADC_VBAT] / 1000,
				adc_vals[MT6360_ADC_IBAT] / 1000);
		strcat(chip->dump_info, str);

		memset(str, 0, sizeof(str));
		sprintf(str, "chg_status=%s,"
				"chg_stat[0x%02x%02x%02x%02x%02x%02x],"
				"chg_ctrl1[0x%02x%02x%02x],"
				"chg_ctrl10[0x%02x%02x%02x%02x%02x%02x%02x],"
				"dev_type=0x%02x,qc=0x%02x,dpdm=0x%02x,shdn=0x%02x,chgdet=[0x%02x,0x%02x],ctrl18=[0x%02x,0x%02x]",
				mt6360_chg_status_name[chg_stat],
				chg_stat1, chg_stat2, chg_stat3, chg_stat4, chg_stat5, chg_stat6,
				chg_ctrl1, chg_ctrl2, chg_ctrl3,
				chg_ctrl10, chg_ctrl11, chg_ctrl12, chg_ctrl13, chg_ctrl14, chg_ctrl15, chg_ctrl16,
				dev_type, qc_ctrl, dpdm_ctrl, shdn_ctrl, chgdet_ctrl1, chgdet_ctrl2, chg_ctrl18, chg_ctrl19);
		strcat(chip->dump_info, str);
		val->strval = chip->dump_info;
	} else {
		pr_err("SET_IN not support\n");
		ret = -EIO;
	}
	return ret;
}

static int mt6360_pmu_chg_io_charge_timeout(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		if (val->intval <= 0) {
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL12, MT6360_MASK_TMR_EN);
		} else {
			regval = (val->intval <= 4 ? 0x0 : ((val->intval - 4) / 2));
			if (regval > 0x7)
				regval = 0x7;
			ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL12, 0xe0, (regval << 5));
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL12, MT6360_MASK_TMR_EN);
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL12);
		if (regval & MT6360_MASK_TMR_EN)
			val->intval = 0;
		else
			val->intval = ((regval & 0xe0) >> 5) * 2 + 4;
	}
	return ret;
}

static int mt6360_pmu_chg_io_otg_vboost(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, vboost = 0;

	if (io == SET_IN) {
		if (val->intval == -1) {
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL5);
			vboost = ((regval & 0xfc) >> 2);
			if (vboost > 0x11) {
				pr_info("step-down:0x%02x\n", vboost);
				vboost--;
				ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, (vboost << 2));
			}
		} else if (val->intval == 0) {
			ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, (0x11 << 2));/*4850mV*/
		} else if (val->intval == 1) {
			regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL5);
			vboost = ((regval & 0xfc) >> 2);
			if (vboost < 0x38) {
				pr_info("step-up:0x%02x\n", vboost);
				vboost++;
				ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, (vboost << 2));
			}
		} else if (val->intval > 4500) {
			chip->vboost_mv = val->intval;
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL5);
		vboost = ((regval & 0xfc) >> 2);
		if (vboost > 0x11)
			val->intval = 4850 + ((vboost - 0x11) * 25);
		else
			val->intval = 4850;
	}
	return ret;
}

static int mt6360_pmu_chg_io_otg_mode(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, ilimit_ua = 0;
	union power_supply_propval soc = {0,};

	if (io == SET_IN) {
		mt6360_pmu_chg_cfo_check(chip);
		if (val->intval != 0) {
			if (chip->otg_reverse_chg) {
				pr_info("otg reverse chg\n");
				power_supply_lite_get_property(PSYL_BATTERY, POWER_SUPPLY_PROP_CAPACITY, &soc);
				if (soc.intval > 25)
					mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, ((0x11 + 0xc) << 2));
				else
					mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, ((0x11 + 0x7) << 2));

				ret = mt6360_set_otg_current_limit(chip->chg_dev, 2100000);/*2100mA*/

				ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x0f, 0x6);
				ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, BIT(4));
			} else {
				pr_info("nromal otg:vboost=%d,iboost=%d\n", chip->vboost_mv, chip->iboost_ma);
				regval = 0x11 + (chip->vboost_mv <= 4850 ? 0x0 : ((chip->vboost_mv - 4850) / 25));
				if (regval > 0x38)
					regval = 0x38;
				ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL5, 0xfc, (regval << 2));

				ilimit_ua = chip->iboost_ma * 1000;
				ret = mt6360_set_otg_current_limit(chip->chg_dev, ilimit_ua);

				ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x0f, 0x0);
				ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, BIT(4));
			}
			ret = mt6360_enable_otg(chip->chg_dev, true);
		} else {
			ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, 0x0f, 0x0);
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_DPDM_CTRL, BIT(4));
			ret = mt6360_enable_otg(chip->chg_dev, false);
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL1);
		if (regval & MT6360_MASK_OPA_MODE)
			val->intval = 1;
		else
			val->intval = 0;
	}
	return ret;
}

static int mt6360_pmu_chg_io_iprechg(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		regval = (val->intval <= 100 ? 0x0 : ((val->intval - 100) / 50));
		if (regval > 0xf)
			regval = 0xf;
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL8, 0xf, regval);
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL8);
		val->intval = ((regval & 0xf) * 50 + 100);
	}
	return ret;
}

static int mt6360_pmu_chg_io_vprechg(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		regval = (val->intval <= 2000 ? 0x0 : ((val->intval - 2000) / 100));
		if (regval > 0xf)
			regval = 0xf;
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL8, 0xf0, (regval << 4));
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL8);
		val->intval = (((regval >> 4) & 0xf) * 100 + 2000);
	}
	return ret;
}

static int mt6360_pmu_chg_io_vrechg(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		regval = (val->intval <= 100 ? 0x0 : ((val->intval - 100) / 50));
		if (regval > 0x3)
			regval = 0x3;
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL11, 0x3, regval);
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL11);
		val->intval = ((regval & 0x3) * 50 + 100);
	}
	return ret;
}

static int mt6360_pmu_chg_io_vsys_min(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0;

	if (io == SET_IN) {
		regval = (val->intval <= 3300 ? 0x0 : ((val->intval - 3300) / 100));
		if (regval > 0x7)
			regval = 0x7;
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL11, (0x7 << 2), (regval << 2));
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL11);
		val->intval = (((regval >> 2) & 0x7) * 100 + 3300);
	}
	return ret;
}

static int mt6360_pmu_chg_io_ovp(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, vin_ovp = 0, det_ovp = 0;

	if (io == SET_IN) {
		if (val->intval < 6500) {
			vin_ovp = 0x0;
			det_ovp = 0x0;
		} else if (val->intval < 11000) {
			vin_ovp = 0x1;
			det_ovp = 0x6;
		} else if (val->intval < 14500) {
			vin_ovp = 0x2;
			det_ovp = 0x8;
		} else {
			vin_ovp = 0x3;
			det_ovp = 0xf;
		}
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL19,
				MT6360_MASK_CHG_VIN_OVP_VTHSEL, (vin_ovp << MT6360_SHFT_CHG_VIN_OVP_VTHSEL));
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHRDET_CTRL1, 0xf, det_ovp);
		ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL18, 0xff, 0x48);//ircmp
		//ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL18, BIT(6));
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL19);
		regval = ((regval >> 5) & 0x3);
		if (regval == 0x0)
			val->intval = 5500;
		else if (regval == 0x1)
			val->intval = 6500;
		else if (regval == 0x2)
			val->intval = 11000;
		else
			val->intval = 14500;
	}
	return ret;
}

static int mt6360_pmu_chg_io_wdt(struct mt6360_pmu_chg_info *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, regval = 0, wdt_s = 0;

	if (io == SET_IN) {
		if (val->intval == 0) {
			ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL13, MT6360_MASK_CHG_WDT_EN);
		} else {
			if (val->intval <= 8)
				wdt_s = 0x0;
			else if (val->intval <= 40)
				wdt_s = 0x1;
			else if (val->intval <= 80)
				wdt_s = 0x2;
			else
				wdt_s = 0x3;

			ret = mt6360_pmu_reg_update_bits(chip->mpi, MT6360_PMU_CHG_CTRL13, (0x3 << 4), (wdt_s << 4));
			ret = mt6360_pmu_reg_set_bits(chip->mpi, MT6360_PMU_CHG_CTRL13, MT6360_MASK_CHG_WDT_EN);
		}
	} else {
		regval = mt6360_pmu_reg_read(chip->mpi, MT6360_PMU_CHG_CTRL13);
		if (regval & 0x80) {
			val->intval = 0;
		} else {
			regval = ((regval >> 4) & 0x3);
			if (regval == 0x0)
				val->intval = 8;
			else if (regval == 0x1)
				val->intval = 40;
			else if (regval == 0x2)
				val->intval = 80;
			else
				val->intval = 160;
		}
	}
	return ret;
}

static int mt6360_pmu_chg_ioctrl_property(struct power_supply_lite *psyl,
			enum direction_io io,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
        struct mt6360_pmu_chg_info *chip = power_supply_lite_get_drvdata(psyl);

	if (!psyl) {
		pr_info("ioctrl[%] not find psyl!!!\n", psp);
		return -EFAULT;
	}
#if PSYL_IO_DEBUG
	pr_debug("ioctrl[%d] io[%d]\n", psp, io);
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		ret = mt6360_pmu_chg_io_health(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6360_pmu_chg_io_status(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DETECT:
		ret = mt6360_pmu_chg_io_charge_detect(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		ret = mt6360_pmu_chg_io_charge_voltage(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = mt6360_pmu_chg_io_charge_type(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHG_TYPE:
		ret = mt6360_pmu_chg_io_charging_type(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_FORCE_DPDM:
		ret = mt6360_pmu_chg_io_force_dpdm(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_DISABLE_CHARGE:
		ret = mt6360_pmu_chg_io_disable_charge(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		ret = mt6360_pmu_chg_io_input_suspend(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_LIMIT_INBAT_CURRENT:
		ret = mt6360_pmu_chg_io_inbat(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_LIMIT_INPUT_CURRENT:
		ret = mt6360_pmu_chg_io_input(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6360_pmu_chg_io_iterm(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = mt6360_pmu_chg_io_vreg(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_ICO:
		ret = mt6360_pmu_chg_io_ico(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_ICO_STATUS:
		ret = mt6360_pmu_chg_io_ico_status(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_FORCE_VINDPM:
		break;
	case POWER_SUPPLY_PROP_VINDPM:
		ret = mt6360_pmu_chg_io_vindpm(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VINDPM_STATUS:
		ret = mt6360_pmu_chg_io_vindpm_status(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_IINDPM_STATUS:
		ret = mt6360_pmu_chg_io_iindpm_status(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_DUMP_REG:
		ret = mt6360_pmu_chg_io_dump_reg(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TIMEOUT:
		ret = mt6360_pmu_chg_io_charge_timeout(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VBOOST:
		ret = mt6360_pmu_chg_io_otg_vboost(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_IBOOST:
		if (io == SET_IN)
			chip->iboost_ma = val->intval;
		else
			val->intval = chip->iboost_ma;
		break;
	case POWER_SUPPLY_PROP_OTG_REVERSE_CHG:
		if (io == SET_IN)
			chip->otg_reverse_chg = !!val->intval;
		else
			val->intval = chip->otg_reverse_chg ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_OTG_MODE:
		ret = mt6360_pmu_chg_io_otg_mode(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6360_pmu_chg_io_iprechg(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VPRECHG:
		ret = mt6360_pmu_chg_io_vprechg(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VRECHG:
		ret = mt6360_pmu_chg_io_vrechg(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_VSYS_MIN:
		ret = mt6360_pmu_chg_io_vsys_min(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_OVP:
		ret = mt6360_pmu_chg_io_ovp(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHG_WDT:
		ret = mt6360_pmu_chg_io_wdt(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_KICK_WDT:
		if (io == SET_IN) {
			mt6360_pmu_chg_cfo_check(chip);
			ret = mt6360_kick_wdt(chip->chg_dev);
		}
		break;
	case POWER_SUPPLY_PROP_ADC_CONVERT:
		break;
	case POWER_SUPPLY_PROP_INTERRUPT_CALLBACK:
		if (io == SET_IN) {
			if (val->int64val != 0) {
				pr_info("set interrupt callback\n");
				chip->callback = (CALL_BACK)(val->int64val);
			} else {
				pr_info("clr interrupt callback\n");
				chip->callback = NULL;
			}
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
#if PSYL_IO_DEBUG
	pr_info("ioctrl[%d] io[%d] %s\n", psp, io, (ret < 0 ? "error" : "done"));
#else
	if (ret < 0)
		pr_info("ioctrl[%d] io[%d] err\n", psp, io);
#endif
	return ret;
}

static struct power_supply_lite mt6360_pmu_chg_psyl = {
	.name = "mt6360_pmu_chg",
	.status = STATUS_PRIMARY,
	.dev_num = PSYL_NUM_UNKNOWN,
	.initialized = false,
	.ioctrl_property = mt6360_pmu_chg_ioctrl_property,
};

struct charger_device *get_charger_by_name(const char *name)
{
	return g_mpci->chg_dev;
}
EXPORT_SYMBOL(get_charger_by_name);
#endif

static int mt6360_pmu_chg_probe(struct platform_device *pdev)
{
	struct mt6360_chg_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct mt6360_pmu_chg_info *mpci;
	struct iio_channel *channel;
	bool use_dt = pdev->dev.of_node;
	int i, ret = 0;
	char *p;

	dev_info(&pdev->dev, "%s\n", __func__);
	if (use_dt) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		ret = mt6360_chg_parse_dt_data(&pdev->dev, pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "parse dt fail\n");
			return ret;
		}
		pdev->dev.platform_data = pdata;
	}
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}
	mpci = devm_kzalloc(&pdev->dev, sizeof(*mpci), GFP_KERNEL);
	if (!mpci)
		return -ENOMEM;
	mpci->dev = &pdev->dev;
	mpci->mpi = dev_get_drvdata(pdev->dev.parent);
	mpci->hidden_mode_cnt = 0;
	mutex_init(&mpci->hidden_mode_lock);
	mutex_init(&mpci->pe_lock);
	mutex_init(&mpci->aicr_lock);
	mutex_init(&mpci->chgdet_lock);
	mutex_init(&mpci->tchg_lock);
	mutex_init(&mpci->ichg_lock);
	mpci->tchg = 0;
	mpci->ichg = 2000000;
	mpci->ichg_dis_chg = 2000000;
	mpci->attach = false;
	mpci->chg_type = CHARGER_UNKNOWN;
	g_mpci = mpci;
#if defined(CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT)\
&& !defined(CONFIG_TCPC_CLASS)
	INIT_WORK(&mpci->chgdet_work, mt6360_chgdet_work_handler);
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT && !CONFIG_TCPC_CLASS */
	init_completion(&mpci->aicc_done);
	init_completion(&mpci->pumpx_done);
	atomic_set(&mpci->pe_complete, 0);
	atomic_set(&mpci->mivr_cnt, 0);
	init_waitqueue_head(&mpci->waitq);
	platform_set_drvdata(pdev, mpci);

	/* apply platform data */
	ret = mt6360_chg_apply_pdata(mpci, pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "apply pdata fail\n");
		goto err_mutex_init;
	}
	/* Initial Setting */
	ret = mt6360_chg_init_setting(mpci);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: init setting fail\n", __func__);
		goto err_mutex_init;
	}

	/* Get ADC iio channels */
	for (i = 0; i < MT6360_ADC_MAX; i++) {
		channel = devm_iio_channel_get(&pdev->dev,
					       mt6360_adc_chan_list[i]);
		if (IS_ERR(channel)) {
			ret = PTR_ERR(channel);
			goto err_mutex_init;
		}
		mpci->channels[i] = channel;
	}

	/* charger class register */
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	mpci->callback = NULL;
	mpci->chg_dev = kzalloc(sizeof(struct charger_device), GFP_KERNEL);
	if (!mpci->chg_dev) {
		pr_err("chg_dev kzalloc fail\n");
		goto err_mutex_init;
	}
	memcpy(&mpci->chg_dev->props, &mt6360_chg_props, sizeof(struct charger_properties));
	mutex_init(&mpci->chg_dev->ops_lock);
	mpci->chg_dev->dev.parent = mpci->dev;
	dev_set_name(&mpci->chg_dev->dev, pdata->chg_name);
	dev_set_drvdata(&mpci->chg_dev->dev, mpci);
	mpci->chg_dev->ops = &mt6360_chg_ops;
	INIT_WORK(&mpci->irq_work, mt6360_irq_work);
#else
	mpci->chg_dev = charger_device_register(pdata->chg_name, mpci->dev,
						mpci, &mt6360_chg_ops,
						&mt6360_chg_props);
	if (IS_ERR(mpci->chg_dev)) {
		dev_err(mpci->dev, "charger device register fail\n");
		ret = PTR_ERR(mpci->chg_dev);
		goto err_mutex_init;
	}
#endif
	/* irq register */
	mt6360_pmu_chg_irq_register(pdev);
	device_init_wakeup(&pdev->dev, true);
	/* mivr task */
	p = devm_kasprintf(mpci->dev, GFP_KERNEL,
				"mivr_thread.%s", dev_name(mpci->dev));
	if (IS_ERR_OR_NULL(p)) {
		dev_notice(mpci->dev, "devm kasprintf fail\n");
		ret = -EINVAL;
		goto err_register_chg_dev;
	}
	mpci->mivr_task = kthread_run(mt6360_chg_mivr_task_threadfn, mpci, p);
	ret = PTR_ERR_OR_ZERO(mpci->mivr_task);
	if (ret < 0) {
		dev_err(mpci->dev, "create mivr handling thread fail\n");
		goto err_register_chg_dev;
	}
	ret = device_create_file(mpci->dev, &dev_attr_shipping_mode);
	if (ret < 0) {
		dev_notice(&pdev->dev, "create shipping attr fail\n");
		goto err_register_chg_dev;
	}
	/* for trigger unfinish pe pattern */
	mpci->pe_wq = create_singlethread_workqueue("pe_pattern");
	if (!mpci->pe_wq) {
		dev_err(mpci->dev, "%s: create pe_pattern work queue fail\n",
			__func__);
		goto err_shipping_mode_attr;
	}
	INIT_WORK(&mpci->pe_work, mt6360_trigger_pep_work_handler);

	/* register fg bat oc notify */
	if (pdata->batoc_notify)
		register_battery_oc_notify(&mt6360_recv_batoc_callback,
					   BATTERY_OC_PRIO_CHARGER);
	/* Schedule work for microB's BC1.2 */
#if defined(CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT)\
&& !defined(CONFIG_TCPC_CLASS)
	schedule_work(&mpci->chgdet_work);
#endif /* CONFIG_MT6360_PMU_CHARGER_TYPE_DETECT && !CONFIG_TCPC_CLASS */
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	if (mt6360_pmu_chg_hw_component_detect(mpci)) {
		wake_lock_init(&mpci->bc12_wlock, WAKE_LOCK_SUSPEND, "mt6360_bc12_wlock");
		mpci->shutdown_ignore_hiz = false;
		mpci->otg_reverse_chg = false;
		mpci->ico = 2000;
		mpci->vboost_mv = 5150;
		mpci->iboost_ma = 1500;
		mpci->mt6360_psyl = &mt6360_pmu_chg_psyl;
		mpci->mt6360_psyl->drv_data = mpci;
		fuelsummary_collect_value(ID_BYTE__MIC_VENDOR, CHGIC_MT6360);
		ret = charge_psyl_register(mpci->mt6360_psyl);
		if (ret < 0) {
			pr_err("mt6360_pmu_chg register psyl error: %d\n", ret);
		}
	}
#endif
#ifdef CONFIG_DEBUG_FS
	create_debugfs_entries(mpci);
#else
	mt6360_chg_debug_init(mpci);
#endif
	dev_info(&pdev->dev, "%s: successfully probed\n", __func__);
	return 0;
err_shipping_mode_attr:
	device_remove_file(mpci->dev, &dev_attr_shipping_mode);
err_register_chg_dev:
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	charger_device_unregister(mpci->chg_dev);
#endif
err_mutex_init:
	mutex_destroy(&mpci->tchg_lock);
	mutex_destroy(&mpci->chgdet_lock);
	mutex_destroy(&mpci->aicr_lock);
	mutex_destroy(&mpci->pe_lock);
	mutex_destroy(&mpci->hidden_mode_lock);
	return ret;
}

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
static void mt6360_pmu_chg_shutdown(struct platform_device *pdev)
{
	int ret = 0;
	union power_supply_propval vindpm = {4500,};
	struct mt6360_pmu_chg_info *chip = platform_get_drvdata(pdev);

	pr_info("\n");
	chip->shutdown_ignore_hiz = true;
	ret = mt6360_pmu_reg_clr_bits(chip->mpi, MT6360_PMU_CHG_CTRL1, MT6360_MASK_FORCE_SLEEP);
	mt6360_pmu_chg_io_vindpm(chip, SET_IN, &vindpm);
}
#endif

static int mt6360_pmu_chg_remove(struct platform_device *pdev)
{
	struct mt6360_pmu_chg_info *mpci = platform_get_drvdata(pdev);

	dev_dbg(mpci->dev, "%s\n", __func__);
	flush_workqueue(mpci->pe_wq);
	destroy_workqueue(mpci->pe_wq);
	if (mpci->mivr_task) {
		kthread_stop(mpci->mivr_task);
		atomic_inc(&mpci->mivr_cnt);
		wake_up(&mpci->waitq);
	}
	device_remove_file(mpci->dev, &dev_attr_shipping_mode);
#ifndef CONFIG_VIVO_CHARGING_NEW_ARCH
	charger_device_unregister(mpci->chg_dev);
#endif
	mutex_destroy(&mpci->tchg_lock);
	mutex_destroy(&mpci->chgdet_lock);
	mutex_destroy(&mpci->aicr_lock);
	mutex_destroy(&mpci->pe_lock);
	mutex_destroy(&mpci->hidden_mode_lock);
	return 0;
}

static int __maybe_unused mt6360_pmu_chg_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused mt6360_pmu_chg_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(mt6360_pmu_chg_pm_ops,
			 mt6360_pmu_chg_suspend, mt6360_pmu_chg_resume);

static const struct of_device_id __maybe_unused mt6360_pmu_chg_of_id[] = {
	{ .compatible = "mediatek,mt6360_pmu_chg", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6360_pmu_chg_of_id);

static const struct platform_device_id mt6360_pmu_chg_id[] = {
	{ "mt6360_pmu_chg", 0 },
	{},
};
MODULE_DEVICE_TABLE(platform, mt6360_pmu_chg_id);

static struct platform_driver mt6360_pmu_chg_driver = {
	.driver = {
		.name = "mt6360_pmu_chg",
		.owner = THIS_MODULE,
		.pm = &mt6360_pmu_chg_pm_ops,
		.of_match_table = of_match_ptr(mt6360_pmu_chg_of_id),
	},
	.probe = mt6360_pmu_chg_probe,
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
	.shutdown = mt6360_pmu_chg_shutdown,
#endif
	.remove = mt6360_pmu_chg_remove,
	.id_table = mt6360_pmu_chg_id,
};
module_platform_driver(mt6360_pmu_chg_driver);

MODULE_AUTHOR("CY_Huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("MT6360 PMU CHG Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(MT6360_PMU_CHG_DRV_VERSION);

/*
 * Version Note
 * 1.0.7_MTK
 * (1) Fix Unbalanced enable for MIVR IRQ
 * (2) Sleep 200ms before do another iteration in mt6360_chg_mivr_task_threadfn
 *
 * 1.0.6_MTK
 * (1) Fix the usages of charger power supply
 *
 * 1.0.5_MTK
 * (1) Prevent charger type infromed repeatedly
 *
 * 1.0.4_MTK
 * (1) Mask mivr irq until mivr task has run an iteration
 *
 * 1.0.3_MTK
 * (1) fix zcv adc from 5mV to 1.25mV per step
 * (2) add BC12 initial setting dcd timeout disable when unuse dcd
 *
 * 1.0.2_MTK
 * (1) remove eoc, rechg, te irq for evb with phone load
 * (2) report power supply online with chg type detect done
 * (3) remove unused irq event and status
 * (4) add chg termination irq notifier when safety timer timeout
 *
 * 1.0.1_MTK
 * (1) fix dtsi parse attribute about en_te, en_wdt, aicc_once
 * (2) add charger class get vbus adc interface
 * (3) add initial setting about disable en_sdi, and check batsysuv.
 *
 * 1.0.0_MTK
 * (1) Initial Release
 */
