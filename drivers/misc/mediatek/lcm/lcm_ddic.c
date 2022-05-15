/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/delay.h>
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/upmu_hw.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>	/* hwPowerOn */
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#include "lcm_define.h"
#include "lcm_drv.h"
#include "lcm_i2c.h"



/*****************************************************************************
 * Define
 *****************************************************************************/
#define LCM_I2C_ADDR 0x7C
#define LCM_I2C_BUSNUM  6
#define LCM_I2C_ID_NAME "SM5109"
#define QRD_LCD_VPOS_ADDRESS 0x00
#define QRD_LCD_VNEG_ADDRESS 0x01
#define QRD_LCD_DIS_ADDRESS 0x03

/*add for detect wrong bias devices*/

static DEFINE_MUTEX(sm5109_mutex);

/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info _lcm_i2c_board_info __initdata = {
	I2C_BOARD_INFO(LCM_I2C_ID_NAME, LCM_I2C_ADDR)
};

static struct i2c_client *_lcm_i2c_client;


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int _lcm_i2c_remove(struct i2c_client *client);


/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct _lcm_i2c_dev {
	struct i2c_client *client;

};


static const struct i2c_device_id _lcm_i2c_id[] = {
	{LCM_I2C_ID_NAME, 0},
	{}
};

static struct i2c_driver _lcm_i2c_driver = {
	.id_table = _lcm_i2c_id,
	.probe = _lcm_i2c_probe,
	.remove = _lcm_i2c_remove,
	/* .detect               = _lcm_i2c_detect, */
	.driver = {
		   .owner = THIS_MODULE,
		   .name = LCM_I2C_ID_NAME,
		   },

};

/*add for detect wrong bias devices*/
enum bl_bias_ic_smt_detect_type {
	BL_BIAS_IC_SMT_DETECT_SUCESS,
	BL_IC_SMT_DETECT_FAIL,
	BIAS_IC_SMT_DETECT_FAIL = 0x10,
};
int bk_lcmic_id;
static int sm5109_read_reg (struct i2c_client *client,
    unsigned reg, uint8_t *value)
{
    uint8_t buf[1];
    int ret = 0;

	if (!client) {
		printk ("%s: client is NULL\n", __func__);
		return -EINVAL;
	}

    if (!value)
	    return -EINVAL;
    buf[0] = reg;
    ret = i2c_master_send (client, buf, 1);
    if (ret > 0) {
	    msleep_interruptible (1);
	    ret = i2c_master_recv (client, buf, 1);
	    if (ret > 0)
		    *value = buf[0];
    }
    return ret;
}

static int bias_ic_id_read (struct i2c_client *client)
{
	uint8_t buf[1];
	int ret = 0;
	mutex_lock (&sm5109_mutex);
	ret = sm5109_read_reg(client, QRD_LCD_VPOS_ADDRESS, buf);
	if (ret < 0)  {
		bk_lcmic_id = bk_lcmic_id | BIAS_IC_SMT_DETECT_FAIL;
	}
	pr_info("[LCM]read id ret =%d, bias ic pos =0x%x, bk_lcmic_id = 0x%x\n", ret, buf[0], bk_lcmic_id);
	mutex_unlock (&sm5109_mutex);
	return ret;
}


/*****************************************************************************
 * Function
 *****************************************************************************/
static int _lcm_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	pr_info("[LCM]%s: enter, I2C address = 0x%x, flags = 0x%x\n", client->name, client->addr, client->flags);
	_lcm_i2c_client = client;
	ret = bias_ic_id_read(client);
	pr_info("[LCM]result_temp=%#x\n", ret);
	return 0;
}


static int _lcm_i2c_remove(struct i2c_client *client)
{
	pr_debug("[LCM][I2C] _lcm_i2c_remove\n");
	_lcm_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = _lcm_i2c_client;
	char write_data[2] = { 0 };

	if (client == NULL) {
		pr_debug("ERROR!! _lcm_i2c_client is null\n");
		return 0;
	}

	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		pr_info("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	return ret;
}

void lcm_bias_set_control(int level)
{
	int control = 0x53;
	int result;
	/******************************/

	switch (level) {
	case 40:
		control = 0x03;
		break;
	case 80:
	default:
		control = 0x53;
		break;
	}
	result = _lcm_i2c_write_bytes (QRD_LCD_DIS_ADDRESS, control);
	pr_err("%s: result_temp=%#x\n", __func__, result);
	return;
}

void lcm_bias_set_avdd_n_avee(int level)
{
	int avdd = 0x0f;
	int avee = 0x0f;
	int result;
/******************************/

	switch (level) {
	case 50:
		avdd = 0x0a;
		avee = 0x0a;
		break;
	case 58:
		avdd = 0x12;
		avee = 0x12;
		break;
	case 60:
		avdd = 0x14;
		avee = 0x14;
		break;
	case 55:
	default:
		avdd = 0x0f;
		avee = 0x0f;
		break;
	}
	result = _lcm_i2c_write_bytes (QRD_LCD_VPOS_ADDRESS, avdd);
	result = _lcm_i2c_write_bytes (QRD_LCD_VNEG_ADDRESS, avee);
	pr_err("%s: result_temp=%#x\n", __func__, result);
	lcm_bias_set_control(80);
	return;
}

/*
 * module load/unload record keeping
 */
static int __init _lcm_i2c_init(void)
{
	pr_debug("[LCM][I2C] _lcm_i2c_init\n");
	if (0)
	i2c_register_board_info(LCM_I2C_BUSNUM, &_lcm_i2c_board_info, 1);
	pr_debug("[LCM][I2C] _lcm_i2c_init2\n");
	i2c_add_driver(&_lcm_i2c_driver);
	pr_debug("[LCM][I2C] _lcm_i2c_init success\n");

	return 0;
}


static void __exit _lcm_i2c_exit(void)
{
	pr_debug("[LCM][I2C] _lcm_i2c_exit\n");
	i2c_del_driver(&_lcm_i2c_driver);
}

module_init(_lcm_i2c_init);
module_exit(_lcm_i2c_exit);

MODULE_AUTHOR("alen keli");
MODULE_DESCRIPTION("MTK LCM I2C Driver");
MODULE_LICENSE("GPL");
