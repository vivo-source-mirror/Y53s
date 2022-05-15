/*
 *  bbk_boards_version.c
 *
 * Copyright (C) 2016 Vivo, Inc.
 * Author: WangLe <wangle@vivo.com.cn>
 *
 * Description: cali the hardware board version and show it
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

#define TAG "BOARD_VERSION"

#define ID_GPIO_MAX_LEN		20
#define PCB_GPIO_MAX_LEN	12
static char pcb_version_map[PCB_GPIO_MAX_LEN];

struct boards_version_data {
	unsigned int gpio_nums;
	unsigned int gpios[ID_GPIO_MAX_LEN];
	char board_version[ID_GPIO_MAX_LEN];
	char model_value[ID_GPIO_MAX_LEN*11+20];
};

struct boards_version_data *bv_data;

static ssize_t board_version_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->board_version);
}

char *get_board_version(void)
{
	return bv_data->board_version;
}

EXPORT_SYMBOL_GPL(get_board_version);

static ssize_t model_value_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, bv_data->model_value);
}

static ssize_t pcb_version_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, pcb_version_map);
}

static struct debug_sysfs_entry board_version = 
		__ATTR(board_version, S_IRUGO, board_version_show, NULL);
static struct debug_sysfs_entry model_value = 
		__ATTR(model_value, S_IRUGO, model_value_show, NULL);
static struct debug_sysfs_entry pcb_version = 
		__ATTR(pcb_version, S_IRUGO, pcb_version_show, NULL);


static int boards_version_parse_dt(struct device *dev, struct boards_version_data *bvdata)
{
	struct device_node *np = dev->of_node;
	int gpio, i;

	if (of_property_read_u32(np, "gpio_nums", &bvdata->gpio_nums)) {
		printk(KERN_ERR "%s: boards version gpio nums(%d) is not valid\n", TAG, bvdata->gpio_nums);
		return -1;
	}
	
	if (of_property_read_u32_array(np, "gpios", bvdata->gpios, bvdata->gpio_nums)) {
		printk(KERN_ERR "%s: boards version gpios get fail\n", TAG);
		return -1;
	}

	gpio = of_get_named_gpio(np, "gpios_start", 0);
	if (!gpio_is_valid(gpio)) {
		printk(KERN_ERR "%s: boards version gpios get fail\n", TAG);
		return -1;
	}

	for (i = 0; i < bvdata->gpio_nums-1; i++) {
		bvdata->gpios[i+1] = bvdata->gpios[i+1] - bvdata->gpios[0] + gpio;
	}
	bvdata->gpios[0] = gpio;

	return 0;
}

extern char *saved_command_line;
static void boards_version_set(struct boards_version_data *bvdata)
{
	unsigned int gpio_nums = bvdata->gpio_nums;
	unsigned int count = 0;
	unsigned int i = 0;
	char keyword[] = "androidboot.product.hardware.sku=";
	char *ptr = NULL, *ptr_e = NULL;
	int size = 0;
	char board_version[ID_GPIO_MAX_LEN] = {0};

	ptr = strstr(saved_command_line, keyword);
	if (ptr != NULL) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e != NULL)
			size = ptr_e - (ptr + strlen(keyword));

		if (size > 0) {
			strncpy(board_version, ptr + strlen(keyword), size);
			board_version[size] = '\0';
		}
	}

	for (i = 0; i < gpio_nums; i++) {
		bvdata->board_version[i] = board_version[i];
		count += sprintf(&bvdata->model_value[count], "GPIO-%d-%c,", bvdata->gpios[i], bvdata->board_version[i]);
	}
	bvdata->board_version[gpio_nums] = '\0';
	bvdata->model_value[count - 1] = '\0';
}

static void pcb_version_set(void)
{
	char keyword[] = "bbk_pcb_version=";
	char *ptr = NULL, *ptr_e = NULL;
	int size = 0;

	ptr = strstr(saved_command_line, keyword);
	if (ptr != NULL) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e != NULL)
			size = ptr_e - (ptr + strlen(keyword));

		if (size > 0) {
			strncpy(pcb_version_map, ptr + strlen(keyword), size);
			pcb_version_map[size] = '\0';
		}
	}
}

static int boards_version_probe(struct platform_device *pdev)
{

	struct boards_version_data *boards_version_data;
	int ret = 0;

	boards_version_data = kzalloc(sizeof(struct boards_version_data), GFP_KERNEL);
	if (!boards_version_data)
		return -ENOMEM;
		
	bv_data = boards_version_data;

	ret = boards_version_parse_dt(&pdev->dev, boards_version_data);
	if (ret < 0) {
		printk(KERN_ERR "%s: boards version parse dt fail\n", TAG);
		goto free_pdata;
	}

	boards_version_set(boards_version_data);
	pcb_version_set();

	ret = devs_create_sys_files(&board_version.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: board version sys files create error\n", TAG);
		goto free_pdata;
	}
	
	ret = devs_create_sys_files(&model_value.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: model value sys files create error\n", TAG);
		goto free_pdata;
	}

	ret = devs_create_sys_files(&pcb_version.attr);
	if (ret < 0) {
		printk(KERN_ERR "%s: model value sys files create error\n", TAG);
		goto free_pdata;
	}

	return 0;

free_pdata:
	kfree(boards_version_data);
	
	return ret;
}

static int boards_version_remove(struct platform_device *pdev)
{
	struct boards_version_data *boards_version_data = platform_get_drvdata(pdev);

	kfree(boards_version_data);

	return 0;
}

#ifdef CONFIG_OF              
static struct of_device_id board_match_table[] = {
    { .compatible = "board-version",},
    {},
};
#endif 

static struct platform_driver boards_version_driver = {
    .probe      = boards_version_probe,
    .remove     = boards_version_remove,
    .driver     = {
		.name   = "board-version",       
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = board_match_table,
#endif
	},
};

static int __init boards_version_init(void)
{
	return platform_driver_register(&boards_version_driver);
}

static void __exit boards_version_exit(void)
{
	platform_driver_unregister(&boards_version_driver);
}

arch_initcall(boards_version_init);
module_exit(boards_version_exit);

MODULE_AUTHOR("WangLe <wangle@vivo.com.cn>");
MODULE_DESCRIPTION("Hardware Boards Version");
MODULE_LICENSE("GPL");

