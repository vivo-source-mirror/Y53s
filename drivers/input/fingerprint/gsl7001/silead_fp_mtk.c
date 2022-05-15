/*
 * @file   silead_fp_mtk.c
 * @brief  Contains silead_fp device implements for Mediatek platform.
 *
 *
 * Copyright 2016-2017 Slead Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/5/2    0.1.0      Init version
 * Bill Yu    2018/5/20   0.1.1      Default wait 3ms after reset
 * Bill Yu    2018/6/5    0.1.2      Support chip enter power down
 * Bill Yu    2018/6/27   0.1.3      Expand pwdn I/F
 * Taobb      2019/6/6    0.1.4      Expand feature interface, irq pin set to reset pin
 * Bill Yu    2019/8/10   0.1.5      Fix crash while parse dts fail
 *
 */

#ifdef BSP_SIL_PLAT_MTK

#ifndef __SILEAD_FP_MTK__
#define __SILEAD_FP_MTK__

#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
//#include "nt_smc_call.h"
#include <linux/gpio.h>
#include <mt-plat/upmu_common.h>

#if !defined(CONFIG_MTK_CLKMGR)
#include <linux/clk.h>
#endif	/* !defined(CONFIG_MTK_CLKMGR)*/

#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
//#include "mtk_spi.h"
//#include "mtk_spi_hal.h"

struct mt_spi_t {
	struct platform_device *pdev;
	void __iomem *regs;
	int irq;
	int running;
	u32 pad_macro;
	struct wake_lock wk_lock;
	struct mt_chip_conf *config;
	struct spi_master *master;

	struct spi_transfer *cur_transfer;
	struct spi_transfer *next_transfer;

	spinlock_t lock;
	struct list_head queue;
#if !defined(CONFIG_MTK_CLKMGR)
	struct clk *clk_main;	/* main clock for spi bus */
#endif				/* !defined(CONFIG_MTK_LEGACY)*/
};

extern int mt_spi_enable_master_clk(struct spi_device *spidev);
extern void mt_spi_disable_master_clk(struct spi_device *spidev);
#endif /* !CONFIG_SILEAD_FP_PLATFORM */

#define FP_IRQ_OF  "mediatek,fp_node"
#define FP_PINS_OF "mediatek,fp_node"

const static uint8_t TANAME[] = { 0x51, 0x1E, 0xAD, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static irqreturn_t silfp_irq_handler(int irq, void *dev_id);
static void silfp_work_func(struct work_struct *work);
static int silfp_input_init(struct silfp_data *fp_dev);
static int silfp_pull_down_cs(struct silfp_data *fp_dev, int active);

/* -------------------------------------------------------------------- */
/*                            power supply                              */
/* -------------------------------------------------------------------- */
static void silfp_hw_poweron(struct silfp_data *fp_dev)
{
	int err = 0;
	//LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
	/* Power control by Regulators(LDO)*/
	if (fp_dev->avdd_ldo) {
		err = regulator_set_voltage(fp_dev->avdd_ldo, AVDD_MIN, AVDD_MAX);	/*set 3.3v*/
		err = regulator_enable(fp_dev->avdd_ldo);	/*enable regulator*/
	}
	/*if (fp_dev->vddio_ldo) {
		err = regulator_set_voltage(fp_dev->vddio_ldo, VDDIO_MIN, VDDIO_MAX);	
		err = regulator_enable(fp_dev->vddio_ldo);	
	}*/
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
	/* Power control by GPIOs */
	if (fp_dev->pin.pins_avdd_h) {
		err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_avdd_h);
	}
	if (fp_dev->pin.pins_vddio_h) {
		err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_vddio_h);
	}
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
	if (fp_dev->avdd_port > 0) {
		err = gpio_direction_output(fp_dev->avdd_port, 1);
	}
	if (fp_dev->vddio_port > 0) {
		err = gpio_direction_output(fp_dev->vddio_port, 1);
	}
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
	fp_dev->power_is_off = 0;
	LOG_MSG_DEBUG(INFO_LOG, "%s: power supply ret:%d \n", __func__, err);
}

static void silfp_hw_poweroff(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
	/* Power control by Regulators(LDO)*/
	if (fp_dev->avdd_ldo && (regulator_is_enabled(fp_dev->avdd_ldo) > 0)) {
		regulator_disable(fp_dev->avdd_ldo);    /*disable regulator*/
	}
	/*if (fp_dev->vddio_ldo && (regulator_is_enabled(fp_dev->vddio_ldo) > 0)) {
		regulator_disable(fp_dev->vddio_ldo);   
	}*/
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
	/* Power control by GPIOs */
	//fp_dev->pin.pins_avdd_h = NULL;
	//fp_dev->pin.pins_vddio_h = NULL;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
	if (fp_dev->avdd_port > 0) {
		gpio_direction_output(fp_dev->avdd_port, 0);
	}
	if (fp_dev->vddio_port > 0) {
		gpio_direction_output(fp_dev->vddio_port, 0);
	}
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
	fp_dev->power_is_off = 1;
}

static void silfp_power_deinit(struct silfp_data *fp_dev)
{
	LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
	/* Power control by Regulators(LDO)*/
	if (fp_dev->avdd_ldo) {
		regulator_disable(fp_dev->avdd_ldo);	/*disable regulator*/
		regulator_put(fp_dev->avdd_ldo);
		fp_dev->avdd_ldo = NULL;
	}
	/*if (fp_dev->vddio_ldo) {
		regulator_disable(fp_dev->vddio_ldo);	
		regulator_put(fp_dev->vddio_ldo);
		fp_dev->vddio_ldo = NULL;
	}*/
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
	/* Power control by GPIOs */
	fp_dev->pin.pins_avdd_h = NULL;
	fp_dev->pin.pins_vddio_h = NULL;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
	if (fp_dev->avdd_port > 0) {
		gpio_direction_output(fp_dev->avdd_port, 0);
		gpio_free(fp_dev->avdd_port);
		fp_dev->avdd_port = 0;
	}
	if (fp_dev->vddio_port > 0) {
		gpio_direction_output(fp_dev->vddio_port, 0);
		gpio_free(fp_dev->vddio_port);
		fp_dev->vddio_port = 0;
	}
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
}

/* -------------------------------------------------------------------- */
/*                            hardware reset                            */
/* -------------------------------------------------------------------- */
static void silfp_hw_reset(struct silfp_data *fp_dev, u8 delay)
{
	LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d\n", __func__, fp_dev->rst_port);

	//if(fp_dev->rst_port > 0 ){
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_l);
	if (fp_dev->irq_no_use) {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq_rst_h);
	}
	mdelay((delay?delay:5)*RESET_TIME_MULTIPLE);
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_h);
	if (fp_dev->irq_no_use) {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq_rst_l);
	}
	mdelay((delay?delay:3)*RESET_TIME_MULTIPLE);
	//}
}

/* -------------------------------------------------------------------- */
/*                            power  down                               */
/* -------------------------------------------------------------------- */
static void silfp_pwdn(struct silfp_data *fp_dev, u8 flag_avdd)
{
	LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d\n", __func__, fp_dev->rst_port);

	if (SIFP_PWDN_FLASH == flag_avdd) {
		silfp_hw_poweroff(fp_dev);
		msleep(200*RESET_TIME_MULTIPLE);
		silfp_hw_poweron(fp_dev);
	}
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_l);
	if (fp_dev->irq_no_use) {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq_rst_h);
	}

	if (SIFP_PWDN_POWEROFF == flag_avdd) {
		silfp_pull_down_cs(fp_dev, 1);//power off need pull-down cs first
		silfp_hw_poweroff(fp_dev);
	}
}

/* -------------------------------------------------------------------- */
/*                         init/deinit functions                        */
/* -------------------------------------------------------------------- */
static int silfp_parse_dts(struct silfp_data *fp_dev)
{
#ifdef CONFIG_OF
	struct device_node *node = NULL;
	struct platform_device *pdev = NULL;
	int  ret;

	node = of_find_compatible_node(NULL, NULL, FP_IRQ_OF);
	if (node) {
		fp_dev->int_port = irq_of_parse_and_map(node, 0);
		LOG_MSG_DEBUG(INFO_LOG, "%s, irq = %d\n", __func__, fp_dev->int_port);
	} else {
		LOG_MSG_DEBUG(ERR_LOG, "%s %s compatible device node is null\n", __func__, FP_IRQ_OF);
	}

	node = of_find_compatible_node(NULL, NULL, FP_PINS_OF);
	if (node) {
		LOG_MSG_DEBUG(INFO_LOG, "%s, irq = %d\n", __func__, fp_dev->int_port);
		pdev = of_find_device_by_node(node);
		if (pdev) {
			fp_dev->pin.pinctrl = devm_pinctrl_get(&pdev->dev);
			if (IS_ERR(fp_dev->pin.pinctrl)) {
				ret = PTR_ERR(fp_dev->pin.pinctrl);
				LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp pinctrl\n", __func__);
				return ret;
			}
		} else {
			LOG_MSG_DEBUG(ERR_LOG, "%s platform device is null\n", __func__);
		}
	} else {
		LOG_MSG_DEBUG(ERR_LOG, "%s %s compatible device node is null\n", __func__, FP_PINS_OF);
		return ret;
	}

	fp_dev->pin.pins_irq = pinctrl_lookup_state(fp_dev->pin.pinctrl, "fingerprint_irq");
	if (IS_ERR(fp_dev->pin.pins_irq)) {
		ret = PTR_ERR(fp_dev->pin.pins_irq);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp irq-init\n", __func__);
		return ret;
	}

	fp_dev->pin.pins_rst_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "reset_high");
	if (IS_ERR(fp_dev->pin.pins_rst_h)) {
		ret = PTR_ERR(fp_dev->pin.pins_rst_h);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp rst-high\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_rst_l = pinctrl_lookup_state(fp_dev->pin.pinctrl, "reset_low");
	if (IS_ERR(fp_dev->pin.pins_rst_l)) {
		ret = PTR_ERR(fp_dev->pin.pins_rst_l);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp rst-high\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_mosi_spi = pinctrl_lookup_state(fp_dev->pin.pinctrl, "mosi_spi");
	if (IS_ERR(fp_dev->pin.pins_mosi_spi)) {
		ret = PTR_ERR(fp_dev->pin.pins_mosi_spi);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp mosi_spi\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_mosi_pullhigh = pinctrl_lookup_state(fp_dev->pin.pinctrl, "mosi_pullhigh");
	if (IS_ERR(fp_dev->pin.pins_mosi_pullhigh)) {
		ret = PTR_ERR(fp_dev->pin.pins_mosi_pullhigh);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp mosi_pullhigh\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_mosi_pulllow = pinctrl_lookup_state(fp_dev->pin.pinctrl, "mosi_pulllow");
	if (IS_ERR(fp_dev->pin.pins_mosi_pulllow)) {
		ret = PTR_ERR(fp_dev->pin.pins_mosi_pulllow);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp mosi_pulllow\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_miso_spi = pinctrl_lookup_state(fp_dev->pin.pinctrl, "miso_spi");
	if (IS_ERR(fp_dev->pin.pins_miso_spi)) {
		ret = PTR_ERR(fp_dev->pin.pins_miso_spi);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp miso_spi\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_miso_pullhigh = pinctrl_lookup_state(fp_dev->pin.pinctrl, "miso_pullhigh");
	if (IS_ERR(fp_dev->pin.pins_miso_pullhigh)) {
		ret = PTR_ERR(fp_dev->pin.pins_miso_pullhigh);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp miso_pullhigh\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_miso_pulllow = pinctrl_lookup_state(fp_dev->pin.pinctrl, "miso_pulllow");
	if (IS_ERR(fp_dev->pin.pins_miso_pulllow)) {
		ret = PTR_ERR(fp_dev->pin.pins_miso_pulllow);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp miso_pulllow\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_cs_spi = pinctrl_lookup_state(fp_dev->pin.pinctrl, "cs_spi");
	if (IS_ERR(fp_dev->pin.pins_cs_spi)) {
		ret = PTR_ERR(fp_dev->pin.pins_cs_spi);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp cs_spi\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_cs_pullhigh = pinctrl_lookup_state(fp_dev->pin.pinctrl, "cs_pullhigh");
	if (IS_ERR(fp_dev->pin.pins_cs_pullhigh)) {
		ret = PTR_ERR(fp_dev->pin.pins_cs_pullhigh);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp cs_pullhigh\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_cs_pulllow = pinctrl_lookup_state(fp_dev->pin.pinctrl, "cs_pulllow");
	if (IS_ERR(fp_dev->pin.pins_cs_pulllow)) {
		ret = PTR_ERR(fp_dev->pin.pins_cs_pulllow);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp cs_pulllow\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_clk_spi = pinctrl_lookup_state(fp_dev->pin.pinctrl, "clk_spi");
	if (IS_ERR(fp_dev->pin.pins_clk_spi)) {
		ret = PTR_ERR(fp_dev->pin.pins_clk_spi);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp clk_spi\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_clk_pullhigh = pinctrl_lookup_state(fp_dev->pin.pinctrl, "clk_pullhigh");
	if (IS_ERR(fp_dev->pin.pins_clk_pullhigh)) {
		ret = PTR_ERR(fp_dev->pin.pins_clk_pullhigh);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp clk_pullhigh\n", __func__);
		return ret;
	}
	fp_dev->pin.pins_clk_pulllow = pinctrl_lookup_state(fp_dev->pin.pinctrl, "clk_pulllow");
	if (IS_ERR(fp_dev->pin.pins_clk_pulllow)) {
		ret = PTR_ERR(fp_dev->pin.pins_clk_pulllow);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp clk_pulllow\n", __func__);
		return ret;
	}
#ifdef BSP_SIL_CTRL_SPI
	fp_dev->pin.spi_default = pinctrl_lookup_state(fp_dev->pin.pinctrl, "spi-default");
	if (IS_ERR(fp_dev->pin.spi_default)) {
		ret = PTR_ERR(fp_dev->pin.spi_default);
		pr_info("%s can't find silfp spi-default\n", __func__);
		return ret;
	}
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.spi_default);
#endif /* BSP_SIL_CTRL_SPI */

	/* Get power settings */
#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
	fp_dev->pin.pins_avdd_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "avdd-enable");
	if (IS_ERR_OR_NULL(fp_dev->pin.pins_avdd_h)) {
		fp_dev->pin.pins_avdd_h = NULL;
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp avdd-enable\n", __func__);
		// Ignore error
	}

	fp_dev->pin.pins_vddio_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "vddio-enable");
	if (IS_ERR_OR_NULL(fp_dev->pin.pins_vddio_h)) {
		fp_dev->pin.pins_vddio_h = NULL;
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp vddio-enable\n", __func__);
		// Ignore error
	}
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
	// Todo: use correct settings.
	fp_dev->avdd_ldo = regulator_get(&fp_dev->spi->dev, "vfp");
	//fp_dev->vddio_ldo = regulator_get(&fp_dev->spi->dev, "vddio");
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */
#ifdef BSP_SIL_POWER_SUPPLY_GPIO
	fp_dev->avdd_port = of_get_named_gpio(node, "silead,gpio_vdd_en", 0);
	if (!gpio_is_valid(fp_dev->avdd_port)) {
		LOG_MSG_DEBUG(ERR_LOG, "avdd_port GPIO is invalid.\n");
		//return -ENODEV;
	} else {
	    LOG_MSG_DEBUG(INFO_LOG, "000 avdd_port GPIO is %d.\n", fp_dev->avdd_port);
	}
	ret = gpio_request(fp_dev->avdd_port, "silead_fp_vdd");
	if (ret) {
		LOG_MSG_DEBUG(ERR_LOG, "Failed to request avdd_port GPIO. ret = %d, number=%d\n", ret, fp_dev->avdd_port);
		//return -ENODEV;
	}
	LOG_MSG_DEBUG(INFO_LOG, "111 avdd_port GPIO is %d.\n", fp_dev->avdd_port);
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */

#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
	if (fp_dev->spi->dev.of_node) {
		/* Get the SPI ID(#1-6)*/
		ret = of_property_read_u32(fp_dev->spi->dev.of_node, "spi-id", &fp_dev->pin.spi_id);
		if (ret) {
			fp_dev->pin.spi_id = 0;
			pr_info("Error getting spi_id\n");
		}
		ret = of_property_read_u32(fp_dev->spi->dev.of_node, "spi-irq", &fp_dev->pin.spi_irq);
		if (ret) {
			fp_dev->pin.spi_irq = 0;
			pr_info("Error getting spi_irq\n");
		}
		ret = of_property_read_u32(fp_dev->spi->dev.of_node, "spi-reg", &fp_dev->pin.spi_reg);
		if (ret) {
			fp_dev->pin.spi_reg = 0;
			pr_info("Error getting spi_reg\n");
		}
	}
#endif /* !CONFIG_SILEAD_FP_PLATFORM */
#endif /* CONFIG_OF */
	return 0;
}

/* -------------------------------------------------------------------- */
/*                     set spi to default status                        */
/* -------------------------------------------------------------------- */
static int silfp_set_default_spi_status(struct silfp_data *fp_dev, int enable)
{
	if (!enable) {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_miso_pulllow);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_mosi_pulllow);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_cs_pullhigh);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_clk_pulllow);
	} else {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_miso_spi);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_mosi_spi);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_cs_spi);
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_clk_spi);
	}
	return 0;
}

/* -------------------------------------------------------------------- */
/*                      pull down cs-gpio                               */
/* -------------------------------------------------------------------- */
static int silfp_pull_down_cs(struct silfp_data *fp_dev, int pull_down)
{
	if (pull_down) {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_cs_pulllow);
	} else {
		pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_cs_pullhigh);
	}
	LOG_MSG_DEBUG(INFO_LOG, "%s, pull_down=%d\n", __func__, pull_down);
	mdelay(1);
	return 0;
}

static int silfp_set_spi(struct silfp_data *fp_dev, bool enable)
{
#if (!defined(CONFIG_SILEAD_FP_PLATFORM))
#if (!defined(CONFIG_MT_SPI_FPGA_ENABLE))
#if defined(CONFIG_MTK_CLKMGR)
	if (enable && !atomic_read(&fp_dev->spionoff_count)) {
		atomic_inc(&fp_dev->spionoff_count);
		enable_clock(MT_CG_PERI_SPI0, "spi");
	} else if (atomic_read(&fp_dev->spionoff_count)) {
		atomic_dec(&fp_dev->spionoff_count);
		disable_clock(MT_CG_PERI_SPI0, "spi");
	}
	LOG_MSG_DEBUG(DBG_LOG, "[%s] done\n", __func__);
#else
	int ret = -ENOENT;
	struct mt_spi_t *ms = NULL;
	ms = spi_master_get_devdata(fp_dev->spi->master);

	if (/*!fp_dev->pin.spi_id || */ !ms) {
		LOG_MSG_DEBUG(ERR_LOG, "%s: not support\n", __func__);
		return ret;
	}

	if (enable && !atomic_read(&fp_dev->spionoff_count)) {
		atomic_inc(&fp_dev->spionoff_count);
		//clk_prepare_enable(ms->clk_main);
		//ret = clk_enable(ms->clk_main);
		ret = mt_spi_enable_master_clk(fp_dev->spi);
	} else if (atomic_read(&fp_dev->spionoff_count)) {
		atomic_dec(&fp_dev->spionoff_count);
		//clk_disable_unprepare(ms->clk_main);
		//clk_disable(ms->clk_main);
		mt_spi_disable_master_clk(fp_dev->spi);
		ret = 0;
	}
	LOG_MSG_DEBUG(DBG_LOG, "[%s] done(%d).\n", __func__, ret);
#endif /* CONFIG_MTK_CLKMGR */
#endif /* !CONFIG_MT_SPI_FPGA_ENABLE */
#else
	return -ENOENT;
#endif /* !CONFIG_SILEAD_FP_PLATFORM */
	return 0;
}

static int silfp_irq_to_reset_init(struct silfp_data *fp_dev)
{
	int ret = 0;

	fp_dev->pin.pins_irq_rst_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "irq_rst-high");
	if (IS_ERR(fp_dev->pin.pins_irq_rst_h)) {
		ret = PTR_ERR(fp_dev->pin.pins_irq_rst_h);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp irq_rst-high\n", __func__);
		return ret;
	}

	fp_dev->pin.pins_irq_rst_l = pinctrl_lookup_state(fp_dev->pin.pinctrl, "irq_rst-low");
	if (IS_ERR(fp_dev->pin.pins_irq_rst_l)) {
		ret = PTR_ERR(fp_dev->pin.pins_irq_rst_l);
		LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp irq_rst-low\n", __func__);
		return ret;
	}

	silfp_irq_disable(fp_dev);
	free_irq(fp_dev->irq, fp_dev);
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq_rst_l);

	fp_dev->irq_no_use = 1;

	return ret;
}

static int silfp_set_feature(struct silfp_data *fp_dev, u8 feature)
{
	int ret = 0;

	switch (feature) {
	case FEATURE_FLASH_CS:
		LOG_MSG_DEBUG(INFO_LOG, "%s set feature flash cs\n", __func__);
		ret = silfp_irq_to_reset_init(fp_dev);
		break;

	default:
		break;
	}
	return ret;
}

static int silfp_resource_init(struct silfp_data *fp_dev, struct fp_dev_init_t *dev_info)
{
	int status = 0;
	int ret;

	if (atomic_read(&fp_dev->init)) {
		atomic_inc(&fp_dev->init);
		LOG_MSG_DEBUG(DBG_LOG, "[%s] dev already init(%d).\n", __func__, atomic_read(&fp_dev->init));
		return status;
	}

	fp_dev->irq_no_use = 0;
	silfp_parse_dts(fp_dev);
	//silfp_hw_poweron(fp_dev);
	/*fp_dev->int_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "irq-gpios", 0);
	fp_dev->rst_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "rst-gpios", 0); */
	LOG_MSG_DEBUG(INFO_LOG, "[%s] int_port %d, rst_port %d.\n", __func__, fp_dev->int_port, fp_dev->rst_port);
	/*if(fp_dev->int_port > 0 ){
		gpio_free(fp_dev->int_port);
	}

	if(fp_dev->rst_port > 0 ){
		gpio_free(fp_dev->rst_port);
	}*/
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_irq);
	fp_dev->irq = fp_dev->int_port; //gpio_to_irq(fp_dev->int_port);
	fp_dev->irq_is_disable = 0;

	ret  = request_irq(fp_dev->irq, 
					   silfp_irq_handler, 
					   IRQ_TYPE_EDGE_RISING, //IRQ_TYPE_LEVEL_HIGH, //irq_table[ts->int_trigger_type], 
					   "silfp", 
					   fp_dev);
	if (ret < 0) {
		LOG_MSG_DEBUG(ERR_LOG, "[%s] Filed to request_irq(%d), ert=%d", __func__, fp_dev->irq, ret);
		status = -ENODEV;
		goto err_irq;
	} else {
		LOG_MSG_DEBUG(INFO_LOG, "[%s] Enable_irq_wake.\n", __func__);
		enable_irq_wake(fp_dev->irq);
		silfp_irq_disable(fp_dev);
	}

	if (fp_dev->rst_port > 0) {
		ret = gpio_request(fp_dev->rst_port, "SILFP_RST_PIN");
		if (ret < 0) {
			LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d", __func__, (s32)fp_dev->rst_port, ret);
			status = -ENODEV;
			goto err_rst;
		} else {
			//gpio_direction_output(fp_dev->rst_port, 0);
			pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_l);
		}
	}
	silfp_hw_poweron(fp_dev);
	mdelay(3);
	pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_rst_h);
	silfp_set_default_spi_status(fp_dev, 0);
	
	if (!ret) {
		if (silfp_input_init(fp_dev)) {
			goto err_input;
		}
		atomic_set(&fp_dev->init, 1);
	}

	dev_info->reserve = PKG_SIZE;
	dev_info->reserve <<= 12;

	if (dev_info && fp_dev->pin.spi_id) {
		//LOG_MSG_DEBUG(ERR_LOG, "spi(%d), irq(%d)reg=0x%X\n", fp_dev->pin.spi_id, fp_dev->pin.spi_irq, fp_dev->pin.spi_reg);
		dev_info->dev_id = (uint8_t)fp_dev->pin.spi_id;
		dev_info->reserve |= fp_dev->pin.spi_irq & 0x0FFF;
		dev_info->reg = fp_dev->pin.spi_reg;
		memcpy(dev_info->ta, TANAME, sizeof(dev_info->ta));
	}

	return status;

err_input:
	if (fp_dev->rst_port > 0) {
		//gpio_free(fp_dev->rst_port);
	}

err_rst:
	free_irq(fp_dev->irq, fp_dev);
	gpio_direction_input(fp_dev->int_port);

err_irq:
	//gpio_free(fp_dev->int_port);

//err:
	fp_dev->int_port = 0;
	fp_dev->rst_port = 0;

	return status;
}

#endif /* __SILEAD_FP_MTK__ */

#endif /* BSP_SIL_PLAT_MTK */

/* End of file spilead_fp_mtk.c */
