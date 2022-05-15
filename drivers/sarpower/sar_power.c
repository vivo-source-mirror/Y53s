/*
 *	drivers/sarpower/sar_power.c
 *
 * Copyright (C) 2020 vivo Technologies, Inc.
 * Author: Kangkai Deng<dengkangkai@vivo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sarpower.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/wakelock.h>
#define TAG "SAR_POWER"
#define sarpower_ERROR -1
struct sar_power_sarpower_data {
	struct sarpower_dev sdev;
	int rf_detect_gpio;
	int rf_detect_gpio_1;
	int rf_detect_gpio_2;
	int rf_detect_gpio_3;
	int rf_detect_gpio_4;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int irq_1;
	int irq_2;
	int irq_3;
	int irq_4;
	int enable;
	struct work_struct work;
	struct work_struct sarpower_work;
};

static unsigned long sarpower_enable = -1;

static void sar_power_irq_work(struct work_struct *work)
{
	int state;
	int state_1 = 0;
	int state_2 = 0;
	int state_3 = 0;
	int state_4 = 0;
	struct sar_power_sarpower_data	*data =
		container_of(work, struct sar_power_sarpower_data, work);

	state = gpio_get_value(data->rf_detect_gpio);


	if (data->rf_detect_gpio_1 >= 0) {
		state_1 = gpio_get_value(data->rf_detect_gpio_1);
	}
	if (data->rf_detect_gpio_2 >= 0) {
		state_2 = gpio_get_value(data->rf_detect_gpio_2);
	}
	if (data->rf_detect_gpio_3 >= 0) {
		state_3 = gpio_get_value(data->rf_detect_gpio_3);
	}
	if (data->rf_detect_gpio_4 >= 0) {
		state_4 = gpio_get_value(data->rf_detect_gpio_4);
	}
	printk(KERN_ERR "[%s]:[%s] state1=%d, state2=%d state3=%d state4=%d state5=%d\n", TAG, __func__, state, state_1, state_2, state_3, state_4);
	state = state || state_1 || state_2 || state_3 || state_4;
	printk(KERN_ERR "[%s]:[%s] state is %d \n", TAG, __func__, state);
	sarpower_set_state(&data->sdev, state);
}

static irqreturn_t sar_power_irq_handler(int irq, void *dev_id)
{
	struct sar_power_sarpower_data *sarpower_data =
		(struct sar_power_sarpower_data *)dev_id;

	printk(KERN_ERR "[%s]:[%s] irq had triggered \n", TAG, __func__);
	schedule_work(&sarpower_data->work);
	return IRQ_HANDLED;
}

static void sar_power_sarpower_work(struct work_struct *sarpower_work)
{
	unsigned long sarpower_state = sarpower_enable;
	int ret = -1;
	struct sar_power_sarpower_data	*sarpower_data =
		container_of(sarpower_work, struct sar_power_sarpower_data, sarpower_work);

	if (sarpower_state == 1) {
		if (sarpower_data->enable == 1) {
			return;
		}

		ret = request_irq(sarpower_data->irq, sar_power_irq_handler,
						  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, sarpower_data->sdev.name, sarpower_data);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] request irq1 %d err %d\n", TAG, __func__, sarpower_data->irq, ret);
			return;
		}

		if (sarpower_data->rf_detect_gpio_1 >= 0) {
			ret = request_irq(sarpower_data->irq_1, sar_power_irq_handler,
							  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, sarpower_data->sdev.name, sarpower_data);
			if (ret < 0) {
				printk(KERN_ERR "[%s]:[%s] request irq2 %d err %d\n", TAG, __func__, sarpower_data->irq_1, ret);
				return;
			}
		}
		if (sarpower_data->rf_detect_gpio_2 >= 0) {
			ret = request_irq(sarpower_data->irq_2, sar_power_irq_handler,
							  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, sarpower_data->sdev.name, sarpower_data);
			if (ret < 0) {
				printk(KERN_ERR "[%s]:[%s] request irq3 %d err %d\n", TAG, __func__, sarpower_data->irq_2, ret);
				return;
			}
		}
		if (sarpower_data->rf_detect_gpio_3 >= 0) {
			ret = request_irq(sarpower_data->irq_3, sar_power_irq_handler,
							  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, sarpower_data->sdev.name, sarpower_data);
			if (ret < 0) {
				printk(KERN_ERR "[%s]:[%s] request irq4 %d err %d\n", TAG, __func__, sarpower_data->irq_3, ret);
				return;
			}
		}
		if (sarpower_data->rf_detect_gpio_4 >= 0) {
			ret = request_irq(sarpower_data->irq_4, sar_power_irq_handler,
							  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, sarpower_data->sdev.name, sarpower_data);
			if (ret < 0) {
				printk(KERN_ERR "[%s]:[%s] request irq5 %d err %d\n", TAG, __func__, sarpower_data->irq_4, ret);
				return;
			}
		}

		schedule_work(&sarpower_data->work);

		sarpower_data->enable = 1;
		printk(KERN_ERR "[%s]:[%s] sarpower enable is %d \n", TAG, __func__, sarpower_data->enable);
	} else {
		if (sarpower_data->enable == 0 || sarpower_data->enable == -1) {
			sarpower_set_state(&sarpower_data->sdev, 1);  /* ensure disable state is 1 */
			return;
		}

		disable_irq(sarpower_data->irq);
		free_irq(sarpower_data->irq, sarpower_data);

		if (sarpower_data->rf_detect_gpio_1 >= 0) {
			disable_irq(sarpower_data->irq_1);
			free_irq(sarpower_data->irq_1, sarpower_data);
		}
		if (sarpower_data->rf_detect_gpio_2 >= 0) {
			disable_irq(sarpower_data->irq_2);
			free_irq(sarpower_data->irq_2, sarpower_data);
		}
		if (sarpower_data->rf_detect_gpio_3 >= 0) {
			disable_irq(sarpower_data->irq_3);
			free_irq(sarpower_data->irq_3, sarpower_data);
		}
		if (sarpower_data->rf_detect_gpio_4 >= 0) {
			disable_irq(sarpower_data->irq_4);
			free_irq(sarpower_data->irq_4, sarpower_data);
		}
		sarpower_set_state(&sarpower_data->sdev, 1);
		sarpower_data->enable = 0;
		printk(KERN_ERR "[%s]:[%s] sarpower enable is %d \n", TAG, __func__, sarpower_data->enable);
	}
}

static ssize_t sarpower_gpio_print_state(struct sarpower_dev *sdev, char *buf)
{
	struct sar_power_sarpower_data	*sarpower_data =
		container_of(sdev, struct sar_power_sarpower_data, sdev);
	const char *state;
	if (sarpower_get_state(sdev))
		state = sarpower_data->state_on;
	else
		state = sarpower_data->state_off;

	if (state)
		/*return snprintf(buf, "%s\n", state, 10);*/
		return snprintf(buf, 10, "%s\n", state);
	return sarpower_ERROR;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
							const char *buf, size_t length)
{
	unsigned long val;
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	struct sar_power_sarpower_data *sarpower_data =
		container_of(sdev, struct sar_power_sarpower_data, sdev);

	if (!attr || !dev || !buf)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	sarpower_enable = val;
	schedule_work(&sarpower_data->sarpower_work);

	return length;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	struct sar_power_sarpower_data *sarpower_data =
		container_of(sdev, struct sar_power_sarpower_data, sdev);

	return snprintf(buf, 10, "enable=%d\n\r", sarpower_data->enable);
}

static DEVICE_ATTR(enable, S_IRWXU, enable_show, enable_store);

static ssize_t refdetect0_show(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	int state1 = -1;
	int state2 = -1;
	int state3 = -1;
	int state4 = -1;
	int state5 = -1;
	int gpio_1 = -1;
	int gpio_2 = -1;
	int gpio_3 = -1;
	int gpio_4 = -1;
	int gpio_5 = -1;
	struct sarpower_dev *sdev = (struct sarpower_dev *)
								dev_get_drvdata(dev);
	struct sar_power_sarpower_data *sarpower_data =
		container_of(sdev, struct sar_power_sarpower_data, sdev);

	if (sarpower_data->rf_detect_gpio > 0) {
		state1 = gpio_get_value(sarpower_data->rf_detect_gpio);
		gpio_1 = sarpower_data->rf_detect_gpio;
	}

    if (sarpower_data->rf_detect_gpio_1 > 0) {
		gpio_2 = sarpower_data->rf_detect_gpio_1;
		state2 = gpio_get_value(sarpower_data->rf_detect_gpio_1);
	}

	if (sarpower_data->rf_detect_gpio_2 > 0) {
		 state3 = gpio_get_value(sarpower_data->rf_detect_gpio_2);
		 gpio_3 = sarpower_data->rf_detect_gpio_2;
	}
	if (sarpower_data->rf_detect_gpio_3 > 0) {
		 state4 = gpio_get_value(sarpower_data->rf_detect_gpio_3);
		 gpio_4 = sarpower_data->rf_detect_gpio_3;
	}
	if (sarpower_data->rf_detect_gpio_4 > 0) {
		 state5 = gpio_get_value(sarpower_data->rf_detect_gpio_4);
		 gpio_5 = sarpower_data->rf_detect_gpio_4;
	}

	return snprintf(buf, 80, "%d %d %d %d %d %d %d %d\n\r", gpio_1, state1, gpio_2, state2, gpio_3, state3, gpio_4, state4);
}

static DEVICE_ATTR(refdetect0, S_IRWXU, refdetect0_show, NULL);

static int sar_power_sarpower_probe(struct platform_device *pdev)
{

	struct sar_power_sarpower_data *sarpower_data;
	int ret = 0;
	/*enum of_gpio_flags flags;*/

	printk(KERN_ERR "[%s]:[%s] enter\n", TAG, __func__);
	sarpower_data = kzalloc(sizeof(struct sar_power_sarpower_data), GFP_KERNEL);
	if (!sarpower_data) {
		printk(KERN_ERR "[%s]:[%s] error 1\n", TAG, __func__);
		return -ENOMEM;
	}

	sarpower_data->sdev.name = "sar-power";
	sarpower_data->enable = -1;

	sarpower_data->rf_detect_gpio = of_get_named_gpio(pdev->dev.of_node, "sar-power-rf-detect,gpios", 0);
	if (sarpower_data->rf_detect_gpio < 0) {
		printk(KERN_ERR "[%s]:[%s] get rf detect error: %d\n", TAG, __func__, sarpower_data->rf_detect_gpio);
		goto err_sarpower_dev_register;
	}
	sarpower_data->rf_detect_gpio_1 = of_get_named_gpio(pdev->dev.of_node, "sar-power-rf-detect_1,gpios", 0);
	if (sarpower_data->rf_detect_gpio_1 < 0) {
		printk(KERN_ERR "[%s]:[%s] It is not the second gpio, error: %d\n", TAG, __func__, sarpower_data->rf_detect_gpio_1);
	}
	sarpower_data->rf_detect_gpio_2 = of_get_named_gpio(pdev->dev.of_node, "sar-power-rf-detect_2,gpios", 0);
	if (sarpower_data->rf_detect_gpio_2 < 0) {
		printk(KERN_ERR "[%s]:[%s] It is not the third gpio\n", TAG, __func__);
	}
	sarpower_data->rf_detect_gpio_3 = of_get_named_gpio(pdev->dev.of_node, "sar-power-rf-detect_3,gpios", 0);
	if (sarpower_data->rf_detect_gpio_3 < 0) {
		printk(KERN_ERR "[%s]:[%s] It is not the fourth gpio\n", TAG, __func__);
	}
	sarpower_data->rf_detect_gpio_4 = of_get_named_gpio(pdev->dev.of_node, "sar-power-rf-detect_4,gpios", 0);
	if (sarpower_data->rf_detect_gpio_4 < 0) {
		printk(KERN_ERR "[%s]:[%s] It is not the fifth gpio\n", TAG, __func__);
	}

	sarpower_data->sdev.print_state = sarpower_gpio_print_state;

	ret = sarpower_dev_register(&sarpower_data->sdev);
	if (ret < 0) {
		printk(KERN_ERR "[%s]:[%s] error 2 ret = %d\n", TAG, __func__, ret);
		goto err_sarpower_dev_register;

	}

	ret = device_create_file(sarpower_data->sdev.dev, &dev_attr_enable);
	if (ret < 0) {
		printk(KERN_ERR "[%s]:[%s] error 3 ret = %d\n", TAG, __func__, ret);
		goto err_sarpower_dev_register;

	}
	ret = device_create_file(sarpower_data->sdev.dev, &dev_attr_refdetect0);
	if (ret < 0) {
		printk(KERN_ERR "[%s]:[%s] error 3 ret = %d\n", TAG, __func__, ret);
		goto err_sarpower_dev_register;

	}
	if (sarpower_data->rf_detect_gpio >= 0) {
		ret = gpio_request(sarpower_data->rf_detect_gpio, sarpower_data->sdev.name);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] error 4 ret = %d\n", TAG, __func__, ret);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(sarpower_data->rf_detect_gpio);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] error 5 ret = %d\n", TAG, __func__, ret);
			goto err_set_gpio_input;
		}

		sarpower_data->irq = gpio_to_irq(sarpower_data->rf_detect_gpio);
		if (sarpower_data->irq < 0) {
			ret = sarpower_data->irq;
			printk(KERN_ERR "[%s]:[%s] error 6\n", TAG, __func__);
			goto err_detect_irq_num_failed;
		}
	}

	/*vivo sensor team added for sencond interrupt gpio*/
	if (sarpower_data->rf_detect_gpio_1 >= 0) {
		ret = gpio_request(sarpower_data->rf_detect_gpio_1, sarpower_data->sdev.name);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_2: %d request error\n", TAG, __func__, sarpower_data->rf_detect_gpio_1);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(sarpower_data->rf_detect_gpio_1);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_2: %d set input error\n", TAG, __func__, sarpower_data->rf_detect_gpio_1);
			goto err_set_gpio_input;
		}
		sarpower_data->irq_1 = gpio_to_irq(sarpower_data->rf_detect_gpio_1);

		if (sarpower_data->irq_1 < 0) {
			ret = sarpower_data->irq_1;
			printk(KERN_ERR "[%s]:[%s] gpio_2 %d set irq error\n", TAG, __func__, sarpower_data->rf_detect_gpio_1);
			goto err_detect_irq_num_failed;
		}
	}

	if (sarpower_data->rf_detect_gpio_2 >= 0) {
		ret = gpio_request(sarpower_data->rf_detect_gpio_2, sarpower_data->sdev.name);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_3: %d request error\n", TAG, __func__, sarpower_data->rf_detect_gpio_2);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(sarpower_data->rf_detect_gpio_2);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_3: %d set input error\n", TAG, __func__, sarpower_data->rf_detect_gpio_2);
			goto err_set_gpio_input;
		}
		sarpower_data->irq_2 = gpio_to_irq(sarpower_data->rf_detect_gpio_2);

		if (sarpower_data->irq_2 < 0) {
			ret = sarpower_data->irq_2;
			printk(KERN_ERR "[%s]:[%s] gpio_3 %d set irq error\n", TAG, __func__, sarpower_data->rf_detect_gpio_2);
			goto err_detect_irq_num_failed;
		}
	}

	if (sarpower_data->rf_detect_gpio_3 >= 0) {
		ret = gpio_request(sarpower_data->rf_detect_gpio_3, sarpower_data->sdev.name);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_4: %d request error\n", TAG, __func__, sarpower_data->rf_detect_gpio_3);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(sarpower_data->rf_detect_gpio_3);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_4: %d set input error\n", TAG, __func__, sarpower_data->rf_detect_gpio_3);
			goto err_set_gpio_input;
		}
		sarpower_data->irq_3 = gpio_to_irq(sarpower_data->rf_detect_gpio_3);

		if (sarpower_data->irq_3 < 0) {
			ret = sarpower_data->irq_3;
			printk(KERN_ERR "[%s]:[%s] gpio_4 %d set irq error\n", TAG, __func__, sarpower_data->rf_detect_gpio_3);
			goto err_detect_irq_num_failed;
		}
	}

	if (sarpower_data->rf_detect_gpio_4 >= 0) {
		ret = gpio_request(sarpower_data->rf_detect_gpio_4, sarpower_data->sdev.name);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_5: %d request error\n", TAG, __func__, sarpower_data->rf_detect_gpio_4);
			goto err_request_gpio;
		}

		ret = gpio_direction_input(sarpower_data->rf_detect_gpio_4);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] gpio_5: %d set input error\n", TAG, __func__, sarpower_data->rf_detect_gpio_4);
			goto err_set_gpio_input;
		}
		sarpower_data->irq_4 = gpio_to_irq(sarpower_data->rf_detect_gpio_4);

		if (sarpower_data->irq_4 < 0) {
			ret = sarpower_data->irq_4;
			printk(KERN_ERR "[%s]:[%s] gpio_5 %d set irq error\n", TAG, __func__, sarpower_data->rf_detect_gpio_4);
			goto err_detect_irq_num_failed;
		}
	}

	/*added end*/

	INIT_WORK(&sarpower_data->work, sar_power_irq_work);
	INIT_WORK(&sarpower_data->sarpower_work, sar_power_sarpower_work);
	sar_power_irq_work(&sarpower_data->work);
	printk(KERN_ERR "[%s]:[%s] success\n", TAG, __func__);

	return 0;

err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(sarpower_data->rf_detect_gpio);
err_request_gpio:
	sarpower_dev_unregister(&sarpower_data->sdev);
err_sarpower_dev_register:
	kfree(sarpower_data);

	return ret;
}

static int sar_power_sarpower_remove(struct platform_device *pdev)
{
	struct sar_power_sarpower_data *sarpower_data = platform_get_drvdata(pdev);

	cancel_work_sync(&sarpower_data->work);
	gpio_free(sarpower_data->rf_detect_gpio);
	sarpower_dev_unregister(&sarpower_data->sdev);
	kfree(sarpower_data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id sar_power_match_table[] = {
	 { .compatible = "sar-power",},
	 {},
};
#endif

static struct platform_driver sar_power_sarpower_driver = {
	.probe		= sar_power_sarpower_probe,
	.remove		= sar_power_sarpower_remove,
	.driver		=  {
		.name		= "sar-power",
		.owner		= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sar_power_match_table,
#endif
	},
};

static int __init sar_power_sarpower_init(void)
{
	return platform_driver_register(&sar_power_sarpower_driver);
}

static void __exit sar_power_sarpower_exit(void)
{
	platform_driver_unregister(&sar_power_sarpower_driver);
}

module_init(sar_power_sarpower_init);
/*late_initcall(sar_power_sarpower_init);*/
module_exit(sar_power_sarpower_exit);

MODULE_AUTHOR("Kangkai Deng <dengkangkai@vivo.com>");
MODULE_DESCRIPTION("SAR POWER sarpower driver");
MODULE_LICENSE("GPL v2");
