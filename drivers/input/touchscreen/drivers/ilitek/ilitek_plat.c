/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"
//#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

//static bool boot_mode_normal = true;

void ilitek_plat_tp_reset_V2(void)
{
	ipio_info("edge delay = %d\n", idev_V2->rst_edge_delay);
	gpio_direction_output(idev_V2->tp_rst, 1);
	mdelay(1);
	gpio_set_value(idev_V2->tp_rst, 0);
	mdelay(5);
	gpio_set_value(idev_V2->tp_rst, 1);
	mdelay(idev_V2->rst_edge_delay);
}



void ilitek_plat_regulator_power_on_V2(bool status)
{
	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (idev_V2->vdd) {
			if (regulator_enable(idev_V2->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev_V2->vcc) {
			if (regulator_enable(idev_V2->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	} else {
		if (idev_V2->vdd) {
			if (regulator_disable(idev_V2->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev_V2->vcc) {
			if (regulator_disable(idev_V2->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&idev_V2->ice_stat, DISABLE);
	mdelay(5);
}

static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	idev_V2->vdd = regulator_get(idev_V2->dev, vdd_name);
	if (ERR_ALLOC_MEM(idev_V2->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev_V2->vdd = NULL;
	}
	if (regulator_set_voltage(idev_V2->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ipio_err("Failed to set VDD %d\n", VDD_VOLTAGE);

	idev_V2->vcc = regulator_get(idev_V2->dev, vcc_name);
	if (ERR_ALLOC_MEM(idev_V2->vcc)) {
		ipio_err("regulator_get VCC fail.\n");
		idev_V2->vcc = NULL;
	}
	if (regulator_set_voltage(idev_V2->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ipio_err("Failed to set VCC %d\n", VCC_VOLTAGE);

	ilitek_plat_regulator_power_on_V2(true);
}
#if 0
static int ilitek_parse_dt(device_node *np)
{

}
#endif
static int ilitek_plat_gpio_register(void)
{
	int ret = 0;
	u32 flag;
	struct device_node *dev_node = idev_V2->node;

	idev_V2->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	idev_V2->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);

	ipio_info("TP INT: %d\n", idev_V2->tp_int);
	ipio_info("TP RESET: %d\n", idev_V2->tp_rst);

	if (!gpio_is_valid(idev_V2->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev_V2->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev_V2->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev_V2->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev_V2->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev_V2->tp_int);
		ret = gpio_request(idev_V2->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev_V2->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev_V2->tp_rst);
		ret = gpio_request(idev_V2->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(idev_V2->tp_int);
	return ret;
}

void ilitek_plat_irq_disable_V2(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev_V2->irq_spin, flag);

	if (atomic_read(&idev_V2->irq_stat) == DISABLE)
		goto out;

	if (!idev_V2->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev_V2->irq_num);
		goto out;
	}

	disable_irq_nosync(idev_V2->irq_num);
	atomic_set(&idev_V2->irq_stat, DISABLE);
	VTI("Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev_V2->irq_spin, flag);
}

void ilitek_plat_irq_enable_V2(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev_V2->irq_spin, flag);

	if (atomic_read(&idev_V2->irq_stat) == ENABLE)
		goto out;

	if (!idev_V2->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev_V2->irq_num);
		goto out;
	}

	enable_irq(idev_V2->irq_num);
	atomic_set(&idev_V2->irq_stat, ENABLE);
	VTI("Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev_V2->irq_spin, flag);
}


/*static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&idev_V2->touch_mutex)) {
		ipio_debug("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&idev_V2->touch_mutex);
	ilitek_tddi_report_handler_V2();
	mutex_unlock(&idev_V2->touch_mutex);
	return IRQ_HANDLED;
}*/

 int ilitek_int_state_check(void)
{
 
	if (atomic_read(&idev_V2->mp_int_check) == ENABLE) {
		atomic_set(&idev_V2->mp_int_check, DISABLE);
		ipio_info("Get an INT for mp, ignore\n");
		wake_up(&(idev_V2->inq));
		//return IRQ_HANDLED;
	}
	return 0;
}
static irqreturn_t ilitek_ts_work_func(int irq, void *handle, ktime_t kt)
{
	VTD("Enter interrupter");
	if (irq != idev_V2->irq_num) {
		ipio_err("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}
	ilitek_int_state_check();

	if (mutex_is_locked(&idev_V2->touch_mutex)) {
		ipio_debug("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&idev_V2->touch_mutex);
	ilitek_tddi_report_handler_V2(kt);
	mutex_unlock(&idev_V2->touch_mutex);
	return IRQ_HANDLED;
}

static int ilitek_plat_irq_register(void)
{
	int ret = 0;

	idev_V2->irq_num  = gpio_to_irq(idev_V2->tp_int);

	ipio_info("idev_V2->irq_num = %d\n", idev_V2->irq_num);

	ret = vts_interrupt_register(idev_V2->vtsdev, idev_V2->irq_num, ilitek_ts_work_func, IRQF_TRIGGER_FALLING, idev_V2);
		if (ret) {
				VTE("Request irq failed!");
		} else {
				VTI("request irq %d succeed", idev_V2->irq_num);
		}

	atomic_set(&idev_V2->irq_stat, ENABLE);
	return ret;
}

/*#ifdef CONFIG_FB
static int ilitek_plat_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	
		//FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
		//FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	
	if (evdata && evdata->data) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_POWERDOWN:
			if (TP_SUSPEND_PRIO) {
				if (event != FB_EARLY_EVENT_BLANK)
					return NOTIFY_DONE;
			} else {
				if (event != FB_EVENT_BLANK)
					return NOTIFY_DONE;
			}
			ilitek_tddi_sleep_handler_V2(TP_SUSPEND);
			break;
		case FB_BLANK_UNBLANK:
		case FB_BLANK_NORMAL:
			if (event == FB_EVENT_BLANK)
				ilitek_tddi_sleep_handler_V2(TP_RESUME);
			break;
		default:
			ipio_err("Unknown event, blank = %d\n", *blank);
			break;
		}
	}
	return NOTIFY_OK;
}
#else
static void ilitek_plat_early_suspend(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler_V2(TP_SUSPEND);
}

static void ilitek_plat_late_resume(struct early_suspend *h)
{
	ilitek_tddi_sleep_handler_V2(TP_RESUME);
}
#endif

static void ilitek_plat_sleep_init(void)
{
#ifdef CONFIG_FB
	ipio_info("Init notifier_fb struct\n");
	idev_V2->notifier_fb.notifier_call = ilitek_plat_notifier_fb;

	if (fb_register_client(&idev_V2->notifier_fb))
		ipio_err("Unable to register notifier_fb\n");

#else
	ipio_info("Init eqarly_suspend struct\n");
	idev_V2->early_suspend.suspend = ilitek_plat_early_suspend;
	idev_V2->early_suspend.resume = ilitek_plat_late_resume;
	idev_V2->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&idev_V2->early_suspend);
#endif
}*/
int ilitek_hw_init(struct vts_device *vtsdev)
{

 	if (REGULATOR_POWER)
	 ilitek_plat_regulator_power_init();

	ilitek_plat_gpio_register();

	if (ilitek_tddi_init_V2() < 0) {
		ipio_err("platform probe failed\n");
		return -ENODEV;
	}
	
    idev_V2->ili_suspended = false;
	ilitek_plat_irq_register();
	return 0;
}


//module_init(ilitek_plat_dev_init);

