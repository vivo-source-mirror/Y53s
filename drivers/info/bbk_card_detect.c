/*
 *  bbk_card_detect.c
 *
 * Copyright (C) 2016 Vivo, Inc.
 * Author: WangLe <wangle@vivo.com.cn>
 *
 * Description: detect SIM1 SIM2 TF-CARD inster remove state 
 *
 */
#include <linux/module.h>
#include <linux/kernel.h> 
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/bbk_drivers_info.h>

#define TAG "CARD-DETECT"

struct card_detect_data {
	bool sim1_card;
	bool sim2_card;
	bool tf_card;
	bool new_card_detect;
	
	unsigned sim1_gpio;
	unsigned sim2_gpio;
	unsigned tf_card_gpio;
	unsigned sim1_gpio_flags;
	unsigned sim2_gpio_flags;
	unsigned tf_card_gpio_flags;
};

struct card_detect_data *cd_data;

static int card_detect_gpio_state(int gpio_num)
{
	return gpio_get_value(gpio_num);
}

static ssize_t card_slot_inserted_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;
	int value = 1;

	if (cd_data->new_card_detect)
		value = 0;

	if (!(cd_data->sim1_card || cd_data->sim2_card || cd_data->tf_card)) {
		count += sprintf(&buf[count], "can not config shrapnel detection ERROR\n");
		return count;
	}
	
	if (cd_data->sim1_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "sim1 gpio %d is invalid\n", cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "sim1 gpio %d is open drain\n", cd_data->sim1_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->sim2_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "sim2 gpio %d is invalid\n", cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "sim2 gpio %d is open drain\n", cd_data->sim2_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->tf_card) {
		gpio_state = card_detect_gpio_state(cd_data->tf_card_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "tf card gpio %d is invalid\n", cd_data->tf_card_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "tf card gpio %d is open drain\n", cd_data->tf_card_gpio);
			flag = -1;
		}	
	}

	if (flag == 0) {
		count += sprintf(&buf[count], "OK\n");
	} else {
		count += sprintf(&buf[count], "ERROR\n");
	}
	
	return count;
}

static ssize_t card_slot_removed_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	int gpio_state = -1;
	int count = 0;
	int flag = 0;
	int value = 0;

	if (cd_data->new_card_detect)
		value = 1;

	if (!(cd_data->sim1_card || cd_data->sim2_card || cd_data->tf_card)) {
		count += sprintf(&buf[count], "can not config shrapnel detection ERROR\n");
		return count;
	}
	
	if (cd_data->sim1_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim1_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "sim1 gpio %d is invalid\n", cd_data->sim1_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "sim1 gpio %d is short\n", cd_data->sim1_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->sim2_card) {
		gpio_state = card_detect_gpio_state(cd_data->sim2_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "sim2 gpio %d is invalid\n", cd_data->sim2_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "sim2 gpio %d is short\n", cd_data->sim2_gpio);
			flag = -1;
		}	
	}
	
	if (cd_data->tf_card) {
		gpio_state = card_detect_gpio_state(cd_data->tf_card_gpio);
		if (gpio_state < 0) {
			count += sprintf(&buf[count], "tf card gpio %d is invalid\n", cd_data->tf_card_gpio);
			return -EIO;
		} else if (gpio_state == value) {
			count += sprintf(&buf[count], "tf card gpio %d is short\n", cd_data->tf_card_gpio);
			flag = -1;
		}	
	}

	if (flag == 0) {
		count += sprintf(&buf[count], "OK\n");
	} else {
		count += sprintf(&buf[count], "ERROR\n");
	}
	
	return count;
}

static struct debug_sysfs_entry card_slot_inserted_check = 
		__ATTR(card_slot_inserted_check, S_IRUGO, card_slot_inserted_show, NULL);
static struct debug_sysfs_entry card_slot_removed_check = 
		__ATTR(card_slot_removed_check, S_IRUGO, card_slot_removed_show, NULL);


static int card_detect_parse_dt(struct device *dev, struct card_detect_data *cdata)
{
	struct device_node *np = dev->of_node;
	int gpio;
	cdata->sim1_card = of_property_read_bool(np, "factory-test,sim1-card");
	cdata->sim2_card = of_property_read_bool(np, "factory-test,sim2-card");
	cdata->tf_card = of_property_read_bool(np, "factory-test,tf-card");
	cdata->new_card_detect = of_property_read_bool(np, "vivo,new_card_detect");

	if (cdata->sim1_card) {
		//if (of_property_read_u32(np, "card-detect-sim1,gpios",&cdata->sim1_gpio))
		gpio = of_get_named_gpio(np, "card-detect-sim1,gpios", 0);
		if (gpio_is_valid(gpio)) {
			cdata->sim1_gpio = gpio;
		} else {
			printk(KERN_ERR "%s: sim1 detect gpio %d is not valid\n", TAG, cdata->sim1_gpio);
			return -1;
		}
		//cdata->sim1_gpio=of_get_named_gpio(np, "card-detect-sim1,gpios", 0);
	}
	
	if (cdata->sim2_card) {
		//if (of_property_read_u32(np, "card-detect-sim2,gpios",&cdata->sim2_gpio))
		gpio = of_get_named_gpio(np, "card-detect-sim2,gpios", 0);
		if (gpio_is_valid(gpio)) {
			cdata->sim2_gpio = gpio;
		} else {
			printk(KERN_ERR "%s: sim2 detect gpio %d is not valid\n", TAG, cdata->sim2_gpio);
			return -1;
		}
		//cdata->sim2_gpio= of_get_named_gpio(np, "card-detect-sim2,gpios", 0);
	}
	if (cdata->tf_card) {
		//if (of_property_read_u32(np, "card-detect-tf-card,gpios",&cdata->tf_card_gpio))
		gpio = of_get_named_gpio(np, "card-detect-tf-card,gpios", 0);
		if (gpio_is_valid(gpio)) {
			cdata->tf_card_gpio = gpio;
		} else {
			printk(KERN_ERR "%s: tf detect gpio %d is not valid\n", TAG, cdata->tf_card_gpio);
			return -1;
		}
		//cdata->tf_card_gpio= of_get_named_gpio(np, "card-detect-tf-card,gpios", 0);
	}

	return 0;
}

static int card_detect_probe(struct platform_device *pdev)
{

	struct card_detect_data *card_detect_data;
	int ret = 0;

	card_detect_data = kzalloc(sizeof(struct card_detect_data), GFP_KERNEL);
	if (!card_detect_data)
		return -ENOMEM;
		
	cd_data = card_detect_data;

	ret = card_detect_parse_dt(&pdev->dev, card_detect_data);
	if (ret < 0) {
		printk(KERN_ERR "%s: card detect parse dt fail \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&card_slot_inserted_check.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: card inserted sys files create error \n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&card_slot_removed_check.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: card removed sys files create error \n", TAG);
		goto free_pdata;
	}

	return 0;

free_pdata:
	kfree(card_detect_data);
	
	return ret;
}

static int card_detect_remove(struct platform_device *pdev)
{
	struct card_detect_data *card_detect_data = platform_get_drvdata(pdev);

	gpio_free(card_detect_data->sim1_gpio);
	gpio_free(card_detect_data->sim2_gpio);
	gpio_free(card_detect_data->tf_card_gpio);
	kfree(card_detect_data);

	return 0;
}

#ifdef CONFIG_OF              
static struct of_device_id card_match_table[] = {
    { .compatible = "card-detect",},
    {},
};
#endif 

static struct platform_driver card_detect_driver = {
    .probe      = card_detect_probe,
    .remove     = card_detect_remove,
    .driver     = {
		.name   = "card-detect",       
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = card_match_table,
#endif
	},
};

static int __init card_detect_init(void)
{
	return platform_driver_register(&card_detect_driver);
}

static void __exit card_detect_exit(void)
{
	platform_driver_unregister(&card_detect_driver);
}

module_init(card_detect_init);
module_exit(card_detect_exit);

MODULE_AUTHOR("WangLe <wangle@vivo.com.cn>");
MODULE_DESCRIPTION("CARD state detect");
MODULE_LICENSE("GPL");

