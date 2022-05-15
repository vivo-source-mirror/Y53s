/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/pm_wakeup.h>
#include <linux/of_platform.h>

#include "extcon_usb.h"

#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
#define ACCDET_TCPC_VOTER "ACCDET_TCPC_VOTER"
extern int get_typec_drp_voter_effective(const char *voter);
#endif

#define HOST_DISABLED_DELAY_TIME  (300*1000)
static BLOCKING_NOTIFIER_HEAD(mt_usb_notifier_list);
extern bool usb_cable_connected(void);

struct usb_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	unsigned int dr; /* data role */
	struct workqueue_struct *extcon_workq;

	bool host_mode_enabled;
	bool typec_hw_det;
	struct delayed_work host_disabled_work;
	struct mutex switch_mutex;
	struct wakeup_source switch_lock;
};

struct mt_usb_work {
	struct delayed_work dwork;
	unsigned int dr; /* data role */
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

enum {
	DUAL_PROP_MODE_UFP = 0,
	DUAL_PROP_MODE_DFP,
	DUAL_PROP_MODE_NONE,
};

enum {
	DUAL_PROP_PR_SRC = 0,
	DUAL_PROP_PR_SNK,
	DUAL_PROP_PR_NONE,
};

enum {
	DUAL_PROP_DR_HOST = 0,
	DUAL_PROP_DR_DEVICE,
	DUAL_PROP_DR_NONE,
};

static struct usb_extcon_info *g_extcon_info;
static void usb_extcon_detect_cable(struct work_struct *work)
{
	struct mt_usb_work *info = container_of(to_delayed_work(work),
						    struct mt_usb_work,
						    dwork);
	unsigned int cur_dr, new_dr;

	if (!g_extcon_info) {
		pr_info("g_extcon_info = NULL\n");
		return;
	}
	cur_dr = g_extcon_info->dr;
	new_dr = info->dr;
	pr_info("cur_dr(%d) new_dr(%d)\n", cur_dr, new_dr);

	g_extcon_info->dr = new_dr;

	/* none -> device */
	if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB, true);
	/* none -> host */
	} else if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_HOST) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled ||
			g_extcon_info->typec_hw_det)
			extcon_set_state_sync(g_extcon_info->edev,
				EXTCON_USB_HOST, true);
		else
			pr_info("%s: %d: host_mode_enabled = %d, typec_hw_det = %d, ignore set host mode\n",
				__func__, __LINE__, g_extcon_info->host_mode_enabled, g_extcon_info->typec_hw_det);
		mutex_unlock(&g_extcon_info->switch_mutex);
	/* device -> none */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB, false);
	/* host -> none */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(g_extcon_info->edev,
			EXTCON_USB_HOST, false);
	/* device -> host */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_HOST) {
		pr_info("device -> host, it's illegal\n");
	/* host -> device */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		pr_info("host -> device, it's illegal\n");
	}

	if (new_dr == DUAL_PROP_DR_HOST)
		cancel_delayed_work(&g_extcon_info->host_disabled_work);
	else {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled)
			queue_delayed_work(g_extcon_info->extcon_workq,
					   &g_extcon_info->host_disabled_work,
					   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
		mutex_unlock(&g_extcon_info->switch_mutex);
	}

	kfree(info);
}

static void issue_connection_work(unsigned int dr)
{
	struct mt_usb_work *work;

	int retry = 20;

	while (retry && !g_extcon_info) {
		pr_info("usb extcon driver is not ready, waiting 100ms, retry: %d\n", retry);
		mdelay(100);
		retry--;
	}

	if (retry == 0 && !g_extcon_info)
		return;

	/* create and prepare worker */
	work = kzalloc(sizeof(struct mt_usb_work), GFP_ATOMIC);
	if (!work)
		return;

	work->dr = dr;
	INIT_DELAYED_WORK(&work->dwork, usb_extcon_detect_cable);
	/* issue connection work */
	queue_delayed_work(g_extcon_info->extcon_workq, &work->dwork, 0);
}

#if !defined(CONFIG_USB_MU3D_DRV)
void mt_usb_connect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_device();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_DEVICE);
}
EXPORT_SYMBOL_GPL(mt_usb_connect);

void mt_usb_disconnect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_none();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_NONE);
}
EXPORT_SYMBOL_GPL(mt_usb_disconnect);
#endif

void mt_usbhost_connect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_host();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_HOST);
}
EXPORT_SYMBOL_GPL(mt_usbhost_connect);

void mt_usbhost_disconnect(void)
{
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_to_none();
#endif
#endif

	pr_info("%s\n", __func__);
	issue_connection_work(DUAL_PROP_DR_NONE);
}
EXPORT_SYMBOL_GPL(mt_usbhost_disconnect);

void mt_usbaudio_connect(void)
{
	if (g_extcon_info->typec_hw_det) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled)
			cancel_delayed_work(&g_extcon_info->host_disabled_work);
		mutex_unlock(&g_extcon_info->switch_mutex);
		pr_info("%s\n", __func__);
	}
}
EXPORT_SYMBOL_GPL(mt_usbaudio_connect);

void mt_usbaudio_disconnect(void)
{
	if (g_extcon_info->typec_hw_det) {
		mutex_lock(&g_extcon_info->switch_mutex);
		if (g_extcon_info->host_mode_enabled)
			queue_delayed_work(g_extcon_info->extcon_workq,
					   &g_extcon_info->host_disabled_work,
					   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
		mutex_unlock(&g_extcon_info->switch_mutex);
		pr_info("%s\n", __func__);
	}
}
EXPORT_SYMBOL_GPL(mt_usbaudio_disconnect);

void mt_vbus_on(void)
{
	usb_otg_set_vbus(true);
}
EXPORT_SYMBOL_GPL(mt_vbus_on);

void mt_vbus_off(void)
{
	usb_otg_set_vbus(false);
}
EXPORT_SYMBOL_GPL(mt_vbus_off);

bool mt_usb_is_host(void)
{
	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return 0;
	}

	return g_extcon_info->dr == DUAL_PROP_DR_HOST;
}
EXPORT_SYMBOL_GPL(mt_usb_is_host);

int mt_usb_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mt_usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mt_usb_register_notify);

void mt_usb_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&mt_usb_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mt_usb_unregister_notify);

static void mt_usb_notify_host_mode(enum host_mode mode)
{
	blocking_notifier_call_chain(&mt_usb_notifier_list, mode, NULL);
}

static ssize_t otg_mode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	ssize_t size;

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

	if (mt_usb_is_host()) {
		pr_info("%s: ----otg enter host mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "host\n");
	} else if (usb_cable_connected()) {
		pr_info("%s: ----otg enter peripheral mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "peripheral\n");
	} else {
		pr_info("%s: ----otg enter none mode----\n", __func__);
		size = snprintf(buf, PAGE_SIZE, "%s", "none\n");
	}

	if (g_extcon_info->typec_hw_det) {
		if (!strncmp(buf, "peripheral", 10) || !strncmp(buf, "none", 4)) {
#ifdef CONFIG_VIVO_CHARGING_NEW_ARCH
			if (get_typec_drp_voter_effective(ACCDET_TCPC_VOTER)) {
				pr_info("%s: ----otg enter ccopened mode----\n", __func__);
				size = snprintf(buf, PAGE_SIZE, "%s", "ccopened\n");
			}
#endif
		}
	}

	return size;
}

static ssize_t otg_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	return -EPERM;
}

static ssize_t host_mode_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	ssize_t size;

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&g_extcon_info->switch_mutex);
	if (g_extcon_info->host_mode_enabled)
		size = snprintf(buf, PAGE_SIZE, "%s", "enabled\n");
	else
		size = snprintf(buf, PAGE_SIZE, "%s", "disabled\n");
	mutex_unlock(&g_extcon_info->switch_mutex);

	return size;
}

static ssize_t host_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	char value[32];
	bool is_enable;

	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return -ENODEV;
	}

	if (sscanf(buf, "%s", value) != 1)
		return -EINVAL;

	if (!strncmp(value, "enabled", 7))
		is_enable = true;
	else if (!strncmp(value, "disabled", 8))
		is_enable = false;
	else
		return -EINVAL;

	mutex_lock(&g_extcon_info->switch_mutex);
	if (is_enable == g_extcon_info->host_mode_enabled) {
		mutex_unlock(&g_extcon_info->switch_mutex);
		return size;
	}

	if (is_enable) {
		queue_delayed_work(g_extcon_info->extcon_workq,
				   &g_extcon_info->host_disabled_work,
				   msecs_to_jiffies(HOST_DISABLED_DELAY_TIME));
		__pm_stay_awake(&g_extcon_info->switch_lock);
		pr_info("%s: ===vivo otg enabled ===\n", __func__);
	} else {
		cancel_delayed_work(&g_extcon_info->host_disabled_work);
		if (!g_extcon_info->host_mode_enabled) {
			pr_info("%s: ===vivo otg already disabled ===\n", __func__);
			mutex_unlock(&g_extcon_info->switch_mutex);
			return size;
		}
		__pm_relax(&g_extcon_info->switch_lock);
		pr_info("%s: ===vivo otg disabled ===\n", __func__);
	}
	g_extcon_info->host_mode_enabled = is_enable;
	if (!g_extcon_info->typec_hw_det)
		mt_usb_notify_host_mode(is_enable ? HOST_MODE_ENABLE : HOST_MODE_DISABLE);
	else
		mt_usb_notify_host_mode(is_enable ? HOST_MODE_ENABLE_HW_DET : HOST_MODE_DISABLE_HW_DET);
	mutex_unlock(&g_extcon_info->switch_mutex);

	return size;
}

DEVICE_ATTR(otg_mode,  0664, otg_mode_show, otg_mode_store);
DEVICE_ATTR(host_mode,	0664, host_mode_show, host_mode_store);

static struct attribute *otg_attributes[] = {
	&dev_attr_otg_mode.attr,
	&dev_attr_host_mode.attr,
	NULL
};

static const struct attribute_group otg_attr_group = {
	.attrs = otg_attributes,
};

static void host_disabled_work(struct work_struct *w)
{
	if (!g_extcon_info) {
		pr_info("%s: g_extcon_info = NULL\n", __func__);
		return;
	}

	mutex_lock(&g_extcon_info->switch_mutex);
	if (!g_extcon_info->host_mode_enabled) {
		pr_info("%s: ===vivo otg already disabled ===\n", __func__);
		mutex_unlock(&g_extcon_info->switch_mutex);
		return;
	}
	g_extcon_info->host_mode_enabled = false;
	__pm_relax(&g_extcon_info->switch_lock);
	pr_info("%s: ===vivo otg disabled ===\n", __func__);
	if (!g_extcon_info->typec_hw_det)
		mt_usb_notify_host_mode(HOST_MODE_DISABLE);
	else
		mt_usb_notify_host_mode(HOST_MODE_DISABLE_HW_DET);
	mutex_unlock(&g_extcon_info->switch_mutex);
}

static int usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_extcon_info *info;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	info->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(info->edev)) {
		dev_info(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, info->edev);
	if (ret < 0) {
		dev_info(dev, "failed to register extcon device\n");
		return ret;
	}
	platform_set_drvdata(pdev, info);
	info->dr = DUAL_PROP_DR_NONE;
	info->extcon_workq = create_singlethread_workqueue("usb_extcon_workq");
	g_extcon_info = info;

#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
	mt_usb_dual_role_init(g_extcon_info->dev);
#endif
#endif
	info->typec_hw_det = of_property_read_bool(dev->of_node, "vivo,typec-hw-det");
	dev_info(&pdev->dev, "typec-hw-det = %d\n", info->typec_hw_det);
	mutex_init(&info->switch_mutex);
	wakeup_source_init(&info->switch_lock, "switch_wakelock");
	INIT_DELAYED_WORK(&info->host_disabled_work, host_disabled_work);
	ret = sysfs_create_group(&pdev->dev.kobj, &otg_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "falied to register sysfs: %d\n", ret);
		return ret;
	}

	/* Perform initial detection */
	/* issue_connection_work(DUAL_PROP_DR_NONE); */
	return 0;
}

static int usb_extcon_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &otg_attr_group);
	cancel_delayed_work_sync(&g_extcon_info->host_disabled_work);

	return 0;
}

static const struct of_device_id usb_extcon_dt_match[] = {
	{ .compatible = "mediatek,extcon-usb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_extcon_dt_match);

static struct platform_driver usb_extcon_driver = {
	.probe		= usb_extcon_probe,
	.remove		= usb_extcon_remove,
	.driver		= {
		.name	= "mediatek,extcon-usb",
		.of_match_table = usb_extcon_dt_match,
	},
};

module_platform_driver(usb_extcon_driver);

