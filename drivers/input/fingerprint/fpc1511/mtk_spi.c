/*
 * FPC Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks.
 * *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 *
 * Copyright (c) 2018 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <linux/input.h>
#include <mtk_ppm_api.h>
#include <linux/workqueue.h>

#if defined(CONFIG_BBK_FP_ID) || defined(CONFIG_BBK_FP_MODULE)
#include "../fp_id.h"
#endif

#define FPC_RESET_LOW_US 5000
#define FPC_RESET_HIGH1_US 100
#define FPC_RESET_HIGH2_US 5000
#define FPC_TTW_HOLD_TIME 1000


static const char * const pctl_names[] = {
	"reset_low",
	"reset_high",
    "fingerprint_irq",
    "miso_spi",
    "mosi_spi",
    "cs_spi",
    "clk_spi",
    "cs_pulllow",
};

struct fpc_data {
	struct device *dev;
	struct spi_device *spidev;
	struct pinctrl *pinctrl_fpc;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	int irq_gpio;
	int rst_gpio;
	bool wakeup_enabled;
	struct wake_lock ttw_wl;
    struct regulator *reg;
	struct input_dev *idev;
	char idev_name[32];
	int event_type;
	int event_code;
};

static DEFINE_MUTEX(spidev_set_gpio_mutex);

extern void mt_spi_disable_master_clk(struct spi_device *spidev);
extern void mt_spi_enable_master_clk(struct spi_device *spidev);

static int select_pin_ctl(struct fpc_data *fpc, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc->dev;
	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			mutex_lock(&spidev_set_gpio_mutex);
			rc = pinctrl_select_state(fpc->pinctrl_fpc, fpc->pinctrl_state[i]);
			mutex_unlock(&spidev_set_gpio_mutex);
			if (rc)
				dev_dbg(fpc->dev, "cannot select '%s'\n", name);
			else
				dev_dbg(fpc->dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static int fpc_perfservice_enable;
static struct work_struct fpc_start_work;
static struct work_struct fpc_stop_work;
static struct hrtimer fpc_perfservice_kthread_timer;

static enum hrtimer_restart fpc_perfservice_kthread_hrtimer_func(struct hrtimer *timer)
{
	fpc_perfservice_enable = 0;
    schedule_work(&fpc_stop_work);

	return HRTIMER_NORESTART;
}

static void fpc_perfservice_kthread_hrtimer_init(void)
{
	static ktime_t ktime;
	if (fpc_perfservice_enable) {
		hrtimer_cancel(&fpc_perfservice_kthread_timer);
	}
	ktime = ktime_set(1, 0);/*1s, 1* 1000 ms*/
	hrtimer_init(&fpc_perfservice_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	fpc_perfservice_kthread_timer.function = fpc_perfservice_kthread_hrtimer_func;
	hrtimer_start(&fpc_perfservice_kthread_timer, ktime, HRTIMER_MODE_REL);
	fpc_perfservice_enable = 1;
}

static void fpc_start_func(struct work_struct *work)
{
	fpc_perfservice_kthread_hrtimer_init();
	/*mt_ppm_perf_boost_enable();*/
	mt_ppm_sysboost_freq(BOOST_BY_PERFSERV, 2002000);
	printk("fpc_start_func\n");
}

static void fpc_stop_func(struct work_struct *work)
{
	mt_ppm_sysboost_freq(BOOST_BY_PERFSERV, 0);
	printk("fpc_stop_func\n");
}

static int fpc_hw_power_enable(struct fpc_data *fpc, bool onoff)
{
	/* TODO: LDO configure */
	static int retval;
	if (!fpc->reg) {
		return -ENODEV;
	}
	if (onoff) {
		if (regulator_is_enabled(fpc->reg)) {
		dev_dbg(fpc->dev, "fpc:%s,power state:on,don't set repeatedly!\n", __func__);
		return retval;
		}
	/* TODO:  set power  according to actual situation  */
		retval = regulator_enable(fpc->reg);
		if (retval) {
			dev_err(fpc->dev, "fpc:error enabling vcc_spi %d\n", retval);
		}
	} else {
		select_pin_ctl(fpc, "reset_low");
		usleep_range(FPC_RESET_LOW_US, FPC_RESET_LOW_US + 100);

		if (fpc->reg) {
			if (regulator_is_enabled(fpc->reg)) {
				regulator_disable(fpc->reg);
			}
			dev_dbg(fpc->dev, "%s, fpc:disable vcc_spi!\n", __func__);
		}
	}
    return retval;
}

static int fpc_hw_get_power_state(struct fpc_data *fpc)
{
	/* TODO: LDO configure */
	static int retval;
	if (!fpc->reg) {
		return -ENODEV;
	} else {
		retval = regulator_is_enabled(fpc->reg);
		return retval;
	}
}

static int set_clks(struct fpc_data *fpc, bool enable)
{
	int rc = 0;

	if (enable) {
		mt_spi_enable_master_clk(fpc->spidev);
		rc = 1;
	} else {
		mt_spi_disable_master_clk(fpc->spidev);
		rc = 0;
	}

	return rc;
}

static int hw_reset(struct  fpc_data *fpc)
{
	int irq_gpio;
	struct device *dev = fpc->dev;

    irq_gpio = gpio_get_value(fpc->irq_gpio);
	dev_info(dev, "IRQ before reset %d\n", irq_gpio);

	select_pin_ctl(fpc, "reset_high");
	usleep_range(FPC_RESET_HIGH1_US, FPC_RESET_HIGH1_US + 100);

	select_pin_ctl(fpc, "reset_low");
	usleep_range(FPC_RESET_LOW_US, FPC_RESET_LOW_US + 100);

	select_pin_ctl(fpc, "reset_high");
	usleep_range(FPC_RESET_HIGH2_US, FPC_RESET_HIGH2_US + 100);

	irq_gpio = gpio_get_value(fpc->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);

	dev_info(dev, "Using GPIO#%d as IRQ.\n", fpc->irq_gpio);
	dev_info(dev, "Using GPIO#%d as RST.\n", fpc->rst_gpio);

	return 0;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc_data *fpc = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		rc = hw_reset(fpc);
		return rc ? rc : count;
	} else {
		return -EINVAL;
	}	
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);
	ssize_t ret = count;

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc->wakeup_enabled = true;
		smp_wmb();
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc->wakeup_enabled = false;
		smp_wmb();
	} else {
		ret = -EINVAL;
	}
	return ret;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			struct device_attribute *attribute,
			char *buffer)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc->irq_gpio);

	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			struct device_attribute *attribute,
			const char *buffer, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	dev_dbg(fpc->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t clk_enable_set(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(device);
	return set_clks(fpc, (*buf == '1')) ? : count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * sysfs node for sending event to make the system interactive,
 * i.e. waking up
 */
static ssize_t do_wakeup_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);

	if (*buf == '1') {
		/* Sending power key event creates a toggling
		   effect that may be desired. It could be
		   replaced by another event such as KEY_WAKEUP. */
		input_report_key(fpc->idev, KEY_FINGERPRINT_WAKE, 1);
		input_report_key(fpc->idev, KEY_FINGERPRINT_WAKE, 0);
		input_sync(fpc->idev);
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(do_wakeup, S_IWUSR, NULL, do_wakeup_set);

static ssize_t fpc_keep_awake_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc_data *fpc = dev_get_drvdata(dev);

	if (*buf == '1') {
		wake_lock_timeout(&fpc->ttw_wl, msecs_to_jiffies(1000));
		dev_info(fpc->dev, "%s\n", __func__);
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(keep_awake, S_IWUSR, NULL, fpc_keep_awake_set);

static ssize_t fpc_perf_lock_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc_data *fpc = dev_get_drvdata(dev);
	schedule_work(&fpc_start_work);
	dev_info(fpc->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(perf_lock, S_IWUSR, NULL, fpc_perf_lock_set);

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct fpc_data *fpc = dev_get_drvdata(dev);
    int rc;

	if (!strncmp(buf, "enable", strlen("enable"))) {
		rc = fpc_hw_power_enable(fpc, true);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		rc = fpc_hw_power_enable(fpc, false);
	} else {
		return -EINVAL;
	}
	dev_info(fpc->dev, "%s rc=%d\n", __func__, rc);
    return rc ? rc : count;
}


static ssize_t regulator_enable_get(struct device *dev,
			struct device_attribute *attr, char *buf)
{
    struct fpc_data *fpc = dev_get_drvdata(dev);
    int rc;
    rc = fpc_hw_get_power_state(fpc);
	dev_info(fpc->dev, "%s rc=%d\n", __func__, rc);
    
    return scnprintf(buf, PAGE_SIZE, "%d\n", rc);

}
static DEVICE_ATTR(regulator_enable, S_IRUSR | S_IWUSR, regulator_enable_get, regulator_enable_set);

static struct attribute *fpc_attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_keep_awake.attr,
	&dev_attr_do_wakeup.attr,
	&dev_attr_perf_lock.attr,
	&dev_attr_regulator_enable.attr,
	NULL
};

static const struct attribute_group const fpc_attribute_group = {
	.attrs = fpc_attributes,
};

static irqreturn_t fpc_irq_handler(int irq, void *handle)
{
	struct fpc_data *fpc = handle;
	struct device *dev = fpc->dev;
	static int current_level; /* We assume low level from start*/
	current_level = !current_level;

	if (current_level) {
		dev_dbg(dev, "Reconfigure irq to trigger in low level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	} else {
		dev_dbg(dev, "Reconfigure irq to trigger in high level\n");
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	smp_rmb();
	if (fpc->wakeup_enabled) {
		wake_lock_timeout(&fpc->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}
	printk("fpc_irq_handler\n");
	sysfs_notify(&fpc->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int mtk6797_probe(struct spi_device *spidev)
{
	struct device *dev = &spidev->dev;
	struct device_node *node_eint;
	struct device_node *np = dev->of_node;
	struct fpc_data *fpc;
	int irqf = 0;
	int irq_num = 0;
	int rc = 0;
	size_t i;
	const char *idev_name;
	u32 val;

	dev_dbg(dev, "%s\n", __func__);
	spidev->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,goodix-fp");
	if (!spidev->dev.of_node) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc) {
		dev_err(dev, "failed to allocate memory for struct fpc_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc->dev = dev;
	dev_set_drvdata(dev, fpc);
	fpc->spidev = spidev;
	fpc->spidev->irq = 0; /*SPI_MODE_0*/

	fpc->pinctrl_fpc = devm_pinctrl_get(&spidev->dev);
	if (IS_ERR(fpc->pinctrl_fpc)) {
		rc = PTR_ERR(fpc->pinctrl_fpc);
		dev_err(fpc->dev, "Cannot find pinctrl_fpc rc = %d.\n", rc);
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state = pinctrl_lookup_state(fpc->pinctrl_fpc, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc->pinctrl_state[i] = state;
	}

    node_eint = of_find_compatible_node(NULL, NULL, "mediatek,fpsensor_fp_eint");
	if (node_eint == NULL) {
		rc = -EINVAL;
		dev_err(fpc->dev, "cannot find node_eint rc = %d.\n", rc);
		goto exit;
	}
	fpc->irq_gpio = of_get_named_gpio(node_eint, "int-gpios", 0);

	/*get regulator*/
	fpc->reg = regulator_get(fpc->dev, "vfp");
	if (IS_ERR(fpc->reg)) {
		rc = -EINVAL;
		dev_err(fpc->dev, "%s, get regulator err.\n", __func__);
		goto exit;
	}
	regulator_set_voltage(fpc->reg, 1800000, 1800000);
	fpc_hw_power_enable(fpc, true);
	dev_dbg(dev, "%s, enable end--->.\n", __func__);

	dev_dbg(dev, "Using GPIO#%d as IRQ.\n", fpc->irq_gpio);
	dev_dbg(dev, "Using GPIO#%d as RST.\n", fpc->rst_gpio);

    select_pin_ctl(fpc, "fingerprint_irq");
    select_pin_ctl(fpc, "clk_spi");
    select_pin_ctl(fpc, "miso_spi");
    select_pin_ctl(fpc, "mosi_spi");
    select_pin_ctl(fpc, "cs_spi");
    node_eint = of_find_compatible_node(NULL, NULL, "mediatek,fpc-fp");
	if (dev->of_node) {
		irq_num = irq_of_parse_and_map(dev->of_node, 0);
		dev_dbg(fpc->dev, "%s, irq_num = %d\n", __func__, irq_num);
	} else {
		rc = -EINVAL;
		dev_err(fpc->dev, "cannot find node_eint rc = %d.\n", rc);
		goto exit;
	}

	rc = of_property_read_u32(np, "fpc,event-type", &val);
	fpc->event_type = rc < 0 ? EV_MSC : val;

	rc = of_property_read_u32(np, "fpc,event-code", &val);
	fpc->event_code = rc < 0 ? MSC_SCAN : val;
	
	fpc->idev = devm_input_allocate_device(dev);
	if (!fpc->idev) {
		dev_err(dev, "failed to allocate input device\n");
		rc = -ENOMEM;
		goto exit;
	}
	input_set_capability(fpc->idev, fpc->event_type,
			fpc->event_code);
	input_set_capability(fpc->idev, fpc->event_type,
			MSC_RAW);

	if (!of_property_read_string(np, "input-device-name", &idev_name)) {
		fpc->idev->name = idev_name;
	} else {
		snprintf(fpc->idev_name, sizeof(fpc->idev_name),
			"fpc@%s", dev_name(dev));
		fpc->idev->name = fpc->idev_name;
	}
	/* Also register the key for wake up */
	set_bit(EV_KEY,	fpc->idev->evbit);
	set_bit(KEY_FINGERPRINT_WAKE, fpc->idev->keybit);
	rc = input_register_device(fpc->idev);
	if (rc) {
		dev_err(dev, "failed to register input device\n");
		goto exit;
	}
	fpc->wakeup_enabled = false;
	INIT_WORK(&fpc_start_work, fpc_start_func);
	INIT_WORK(&fpc_stop_work, fpc_stop_func);

	irqf = IRQF_TRIGGER_HIGH | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

    rc = request_threaded_irq(irq_num, NULL, fpc_irq_handler,
				irqf, dev_name(dev), fpc);
	if (rc) {
		dev_err(dev, "could not request irq %d\n", irq_num);
		goto err;
	}
	dev_dbg(dev, "requested irq %d\n", irq_num);

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(irq_num);
	wake_lock_init(&fpc->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &fpc_attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto err;
	}

	(void)hw_reset(fpc);
	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
err:
    fpc_hw_power_enable(fpc, false);
    return rc;
}

static int mtk6797_remove(struct spi_device *spidev)
{
	struct  fpc_data *fpc = dev_get_drvdata(&spidev->dev);

	sysfs_remove_group(&spidev->dev.kobj, &fpc_attribute_group);
	wake_lock_destroy(&fpc->ttw_wl);
	dev_info(&spidev->dev, "%s\n", __func__);
	return 0;
}

static void mtk6797_shutdown(struct spi_device *spidev)
{
	struct  fpc_data *fpc = dev_get_drvdata(&spidev->dev);
	printk("fpc_shutdown\n");
	select_pin_ctl(fpc, "cs_pulllow");
	fpc_hw_power_enable(fpc, false);
}

static struct of_device_id mt6797_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{ .compatible = "mediatek,goodix-fp", },
	{ .compatible = "goodix,goodix-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6797_of_match);

static struct spi_driver mtk6797_driver = {
	.driver = {
		.name	= "fpc_spi",
		.bus = &spi_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = mt6797_of_match,
	},
	.probe	= mtk6797_probe,
	.remove	= mtk6797_remove,
	.shutdown = mtk6797_shutdown,
};

//vivo duyihang add for puresys_recovery begin
extern unsigned int os_boot_puresys;
//vivo duyihang add for puresys_recovery end
static int __init fpc_sensor_init(void)
{
	int status;
	if ((get_fp_id() != FPC_FPC1511) && (get_fp_id() != FPC_FPC1540)) {
		printk("%s(): wrong fpc1511 id, exit\n", __func__);
		return 0;
	}
	//vivo duyihang add for puresys_recovery begin
	if (os_boot_puresys == 1) {
		printk("%s:boot puresys, not load drm driver!\n", __func__);
		return 0;
	}
	//vivo duyihang add for puresys_recovery end

	status = spi_register_driver(&mtk6797_driver);
	if (status < 0) {
		printk("%s, fpc_sensor_init failed.\n", __func__);
	}
	return status;
}
late_initcall(fpc_sensor_init);

static void __exit fpc_sensor_exit(void)
{
	spi_unregister_driver(&mtk6797_driver);
}
module_exit(fpc_sensor_exit);

MODULE_LICENSE("GPL");
