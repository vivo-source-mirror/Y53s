#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/atomic.h>
#include <extcon_usb.h>

#define VUSB_IDDIG_DEBOUNCE_MS 50
#define VUSB_IDDIG_PU_DEBOUNCE_MS 20
#define VUSB_IDDIG_PU_TIMES 5

struct vusb_iddig_info {
	struct device *dev;
	struct gpio_desc *id_gpiod;
	int id_irq;
	bool id_trigger;
	bool enable;
	unsigned int id_debounce;
	unsigned int id_pu_debounce;
	unsigned int id_pu_times;
	struct mutex mutex;

	struct pinctrl *usb_id_pinctrl;
	struct pinctrl_state *usb_id_in;
	struct pinctrl_state *usb_id_in_pud;
	struct pinctrl_state *usb_id_pu1_in;
	struct pinctrl_state *usb_id_pu1_outh;
	struct pinctrl_state *usb_id_pu2_in;
	struct pinctrl_state *usb_id_pu2_outh;

	struct workqueue_struct *id_workq;
	struct delayed_work id_work;
};
struct vusb_iddig_info *_info;

static void vusb_iddig_work(struct work_struct *work)
{
	struct vusb_iddig_info *info = container_of(work,
						    struct vusb_iddig_info,
						    id_work.work);
	int id_pu_times = info->id_pu_times;
	int id_state;

	mutex_lock(&info->mutex);
	if (!info->enable) {
		pr_info("%s: %d: usb disabled\n", __func__, __LINE__);
		goto end;
	}

	id_state = gpiod_get_value_cansleep(info->id_gpiod);
	pr_info("%s: %d: id_state = %d, id_trigger = %d\n", __func__, __LINE__,
		id_state, info->id_trigger);

	if (id_state == info->id_trigger) {
		int irq_type = IRQF_ONESHOT;

		if (!id_state) {
			pinctrl_select_state(info->usb_id_pinctrl,
					     info->usb_id_pu2_outh);
			pinctrl_select_state(info->usb_id_pinctrl,
					     info->usb_id_pu1_in);
			usleep_range(info->id_debounce, info->id_debounce);

			id_state = gpiod_get_value_cansleep(info->id_gpiod);
			while (!id_state && id_pu_times > 0) {
				usleep_range(info->id_pu_debounce, info->id_pu_debounce);
				id_state = gpiod_get_value_cansleep(info->id_gpiod);
				id_pu_times--;
			}
			if (id_state) {
				pr_info("%s: %d: id_pu_debounce fail: id_state = %d, id_trigger = %d, id_pu_times = %d\n",
					__func__, __LINE__, id_state, info->id_trigger, id_pu_times);
				pinctrl_select_state(info->usb_id_pinctrl,
						     info->usb_id_pu1_outh);
				pinctrl_select_state(info->usb_id_pinctrl,
						     info->usb_id_pu2_in);
				usleep_range(info->id_debounce, info->id_debounce);
				goto end;
			} else {
				mt_usbhost_connect();
				mt_vbus_on();
			}
		} else {
			pinctrl_select_state(info->usb_id_pinctrl,
					     info->usb_id_pu1_outh);
			pinctrl_select_state(info->usb_id_pinctrl,
					     info->usb_id_pu2_in);
			usleep_range(info->id_debounce, info->id_debounce);
			mt_usbhost_disconnect();
			mt_vbus_off();
		}
		info->id_trigger = !id_state;
		if (info->id_trigger)
			irq_type |= IRQ_TYPE_LEVEL_HIGH;
		else
			irq_type |= IRQ_TYPE_LEVEL_LOW;
		irq_set_irq_type(info->id_irq, irq_type);
	}
end:
	mutex_unlock(&info->mutex);
	enable_irq(info->id_irq);
}

static irqreturn_t vusb_iddig_eint_isr(int irqnum, void *data)
{
	struct vusb_iddig_info *info = data;

	disable_irq_nosync(irqnum);

	queue_delayed_work(info->id_workq, &info->id_work,
			   msecs_to_jiffies(info->id_debounce));

	return IRQ_HANDLED;
}

static int vusb_iddig_parse_dts(struct platform_device *pdev,
				struct vusb_iddig_info *info)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	if (of_property_read_u32(node, "id-debounce", &info->id_debounce))
		info->id_debounce = VUSB_IDDIG_DEBOUNCE_MS;
	if (of_property_read_u32(node, "id-pu-debounce", &info->id_pu_debounce))
		info->id_pu_debounce = VUSB_IDDIG_PU_DEBOUNCE_MS;
	if (of_property_read_u32(node, "id-pu-times", &info->id_pu_times))
		info->id_pu_times = VUSB_IDDIG_PU_TIMES;
	pr_info("%s: %d: id-debounce = %d, id-pu-debounce = %d, id-pu-times = %d\n",
		__func__, __LINE__, info->id_debounce, info->id_pu_debounce, info->id_pu_times);

	info->usb_id_in =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_in");
	if (IS_ERR_OR_NULL(info->usb_id_in)) {
		ret = PTR_ERR(info->usb_id_in);
		pr_err("%s: %d: failed to get usb_id_in state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	info->usb_id_in_pud =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_in_pud");
	if (IS_ERR_OR_NULL(info->usb_id_in_pud)) {
		ret = PTR_ERR(info->usb_id_in_pud);
		pr_err("%s: %d: failed to get usb_id_in_pud state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	info->usb_id_pu1_in =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_pu1_in");
	if (IS_ERR_OR_NULL(info->usb_id_pu1_in)) {
		ret = PTR_ERR(info->usb_id_pu1_in);
		pr_err("%s: %d: failed to get usb_id_pu1_in state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	info->usb_id_pu1_outh =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_pu1_outh");
	if (IS_ERR_OR_NULL(info->usb_id_pu1_outh)) {
		ret = PTR_ERR(info->usb_id_pu1_outh);
		pr_err("%s: %d: failed to get usb_id_pu1_outh state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	info->usb_id_pu2_in =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_pu2_in");
	if (IS_ERR_OR_NULL(info->usb_id_pu2_in)) {
		ret = PTR_ERR(info->usb_id_pu2_in);
		pr_err("%s: %d: failed to get usb_id_pu2_in state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	info->usb_id_pu2_outh =
		pinctrl_lookup_state(info->usb_id_pinctrl, "usb_id_pu2_outh");
	if (IS_ERR_OR_NULL(info->usb_id_pu2_outh)) {
		ret = PTR_ERR(info->usb_id_pu2_outh);
		pr_err("%s: %d: failed to get usb_id_pu2_outh state: %d\n", __func__, __LINE__, ret);
		return ret;
	}

	return ret;
}

static int mt_usb_notify(struct notifier_block *self,
			 unsigned long is_host, void *data)
{
	struct vusb_iddig_info *info;

	info = _info;
	if (!info) {
		pr_err("%s: %d: vusb_iddig_info is null", __func__, __LINE__);
		goto end;
	}

	mutex_lock(&info->mutex);
	pr_info("%s: %d: %d\n", __func__, __LINE__, is_host);

	switch (is_host) {
	case HOST_MODE_DISABLE:
		disable_irq(info->id_irq);
		if (info->id_trigger) {
			mt_usbhost_disconnect();
			mt_vbus_off();
		}
		pinctrl_select_state(info->usb_id_pinctrl,
				     info->usb_id_pu1_in);
		pinctrl_select_state(info->usb_id_pinctrl,
				     info->usb_id_pu2_in);
		pinctrl_select_state(info->usb_id_pinctrl,
				     info->usb_id_in_pud);
		usleep_range(info->id_debounce, info->id_debounce);
		info->id_trigger = true;
		info->enable = false;
		break;
	case HOST_MODE_ENABLE:
		pinctrl_select_state(info->usb_id_pinctrl,
				     info->usb_id_in);
		pinctrl_select_state(info->usb_id_pinctrl,
				     info->usb_id_pu1_outh);
		usleep_range(info->id_debounce, info->id_debounce);
		irq_set_irq_type(info->id_irq, IRQF_TRIGGER_LOW | IRQF_ONESHOT);
		info->id_trigger = false;
		info->enable = true;
		enable_irq(info->id_irq);
		break;
	default:
		break;
	}
	mutex_unlock(&info->mutex);
end:
	return NOTIFY_OK;
}

static struct notifier_block mt_usb_nb = {
	.notifier_call = mt_usb_notify,
};

static int vusb_iddig_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct vusb_iddig_info *info;
	int ret = 0;

	if (!node)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;

	info->id_gpiod = devm_gpiod_get_optional(dev, "id", GPIOD_IN);
	if (IS_ERR_OR_NULL(info->id_gpiod)) {
		ret = PTR_ERR(info->id_gpiod);
		pr_err("%s: %d: failed to get id gpio: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	info->usb_id_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(info->usb_id_pinctrl)) {
		ret = PTR_ERR(info->usb_id_pinctrl);
		pr_err("%s: %d: failed to get usb id pinctrl: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	ret = vusb_iddig_parse_dts(pdev, info);
	if (ret) {
		pr_err("%s: %d: failed to parse dts: %d\n", __func__, __LINE__, ret);
		goto err;
	}

	info->id_workq = create_singlethread_workqueue("id_workq");
	if (!info->id_workq) {
		ret = -ENOMEM;
		goto err;
	}
	INIT_DELAYED_WORK(&info->id_work, vusb_iddig_work);
	mutex_init(&info->mutex);

	info->id_irq = irq_of_parse_and_map(node, 0);
	if (info->id_irq < 0) {
		ret = info->id_irq;
		pr_err("%s: %d: failed to get id irq: %d\n", __func__, __LINE__, ret);
		goto err1;
	}

	ret = devm_request_irq(dev, info->id_irq,
			       vusb_iddig_eint_isr,
			       IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			       pdev->name, info);
	if (ret < 0) {
		pr_err("%s: %d: failed to request id irq: %d\n", __func__, __LINE__, ret);
		goto err1;
	}
	disable_irq(info->id_irq);

	platform_set_drvdata(pdev, info);

	_info = info;
	mt_usb_register_notify(&mt_usb_nb);

	pr_info("%s: %d: probe finish\n", __func__, __LINE__);

	return 0;
err1:
	destroy_workqueue(info->id_workq);
err:
	return ret;
}

static int vusb_iddig_remove(struct platform_device *pdev)
{
	struct vusb_iddig_info *info = platform_get_drvdata(pdev);

	mt_usb_unregister_notify(&mt_usb_nb);

	cancel_delayed_work_sync(&info->id_work);
	destroy_workqueue(info->id_workq);
	_info = NULL;

	return 0;
}

static const struct of_device_id vusb_iddig_of_match[] = {
	{.compatible = "vivo,usb_iddig_bi_eint"},
	{},
};

static struct platform_driver vusb_iddig_driver = {
	.probe = vusb_iddig_probe,
	.remove = vusb_iddig_remove,
	.driver = {
		.name = "vusb_iddig",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vusb_iddig_of_match),
	},
};

static int __init vusb_iddig_init(void)
{
	return platform_driver_register(&vusb_iddig_driver);
}
late_initcall(vusb_iddig_init);

static void __exit vusb_iddig_cleanup(void)
{
	platform_driver_unregister(&vusb_iddig_driver);
}
module_exit(vusb_iddig_cleanup);

