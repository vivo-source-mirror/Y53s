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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>

/*#define HOPE_ADD_IRQ_FLASH*/
#ifdef HOPE_ADD_IRQ_FLASH
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef LM3644_DTNAME
#define LM3644_DTNAME "mediatek,flashlights_lm3644"
#endif
#ifndef LM3644_DTNAME_I2C
#define LM3644_DTNAME_I2C "mediatek,flashlights_lm3644_i2c"
#endif

#define LM3644_NAME "flashlights-lm3644"

/* define registers */
#define LM3644_REG_ENABLE (0x01)
#define LM3644_MASK_ENABLE_LED1 (0x01)
#define LM3644_MASK_ENABLE_LED2 (0x02)
#define LM3644_DISABLE (0x00)
#define LM3644_ENABLE_LED1 (0x01)
#define LM3644_ENABLE_LED1_TORCH (0x09)
#define LM3644_ENABLE_LED1_FLASH (0x0D)
#define LM3644_ENABLE_LED2 (0x02)
#define LM3644_ENABLE_LED2_TORCH (0x0A)
#define LM3644_ENABLE_LED2_FLASH (0x0E)

#define LM3644_REG_TORCH_LEVEL_LED1 (0x05)
#define LM3644_REG_FLASH_LEVEL_LED1 (0x03)
#define LM3644_REG_TORCH_LEVEL_LED2 (0x06)
#define LM3644_REG_FLASH_LEVEL_LED2 (0x04)
#define LM3644_REG_FLAGS_1 (0x0A)
#define LM3644_REG_FLAGS_2 (0x0B)
#define LM3644_DEVICE_ID (0x0C)

#define LM3644_REG_TIMING_CONF (0x08)
#define LM3644_TORCH_RAMP_TIME (0x10)
#define LM3644_FLASH_TIMEOUT   (0x0F)
#define AW36413_FLASH_TIMEOUT_160MS   (0x03)
#define LM3644_FLASH_TIMEOUT_150MS   (0x0A)
#define LM3644TT_FLASH_TIMEOUT (0x09)

#define LM3644_REG_DEVICE_ID (0x0C)
#define LM3644_REG_BOOST (0x07)
#define LM3644_REG_BOOST_CRRRENT_LIMIT_2_8A (0x09)
#define LM3644_REG_BOOST_CRRRENT_LIMIT_1_8A (0x08)


/* define channel, level */
#define LM3644_CHANNEL_NUM 2
#define LM3644_CHANNEL_CH1 0
#define LM3644_CHANNEL_CH2 1

#define LM3644_LEVEL_NUM 26
#define LM3644_LEVEL_TORCH 7

#define LM3644_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(lm3644_mutex);
static DEFINE_MUTEX(pin_set_mutex);
static struct work_struct lm3644_work_ch1;
static struct work_struct lm3644_work_ch2;

/* define pinctrl */
#define LM3644_PINCTRL_PIN_HWEN 0
#define LM3644_PINCTRL_PINSTATE_LOW 0
#define LM3644_PINCTRL_PINSTATE_HIGH 1
#define LM3644_PINCTRL_STATE_HWEN_HIGH "hwen_high"
#define LM3644_PINCTRL_STATE_HWEN_LOW  "hwen_low"
#ifdef HOPE_ADD_IRQ_FLASH
#define LM3644_PINCTRL_STATE_FLASH_IRQ "flash_irq"
#endif

static struct pinctrl *lm3644_pinctrl;
static struct pinctrl_state *lm3644_hwen_high;
static struct pinctrl_state *lm3644_hwen_low;
#ifdef HOPE_ADD_IRQ_FLASH
static int g_flash_irq_num;
static  unsigned int g_gpio_pin;
static unsigned int g_gpio_headset_deb;
static struct pinctrl_state *lm3644_flash_irq;
static unsigned int g_accdet_eint_type = IRQ_TYPE_LEVEL_LOW;
static struct delayed_work ir_delayed_work;
static unsigned int led_count;
static unsigned int irq_enable_count;
static unsigned int first_led_on;
static ktime_t StartTime;
static struct timer_list flash_delay_off_timer;
static struct work_struct flash_delay_off_work;
static void flash_delay_off_func(struct work_struct *work);
static void set_flash_led_delay_off(unsigned long arg);
unsigned int sof_flag;
#endif


/* define usage count */
static int use_count;
static int lock_touch;  /*hope add*/
static int lock_touch_sub; /*hope add*/
static int device_id;
/* define i2c */
static struct i2c_client *lm3644_i2c_client;

/* platform data */
struct lm3644_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* lm3644 chip data */
struct lm3644_chip_data {
	struct i2c_client *client;
	struct lm3644_platform_data *pdata;
	struct mutex lock;
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int lm3644_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	lm3644_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lm3644_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(lm3644_pinctrl);
	}

	/* Flashlight HWEN pin initialization */
	lm3644_hwen_high = pinctrl_lookup_state(lm3644_pinctrl, LM3644_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(lm3644_hwen_high)) {
		pr_err("Failed to init (%s)\n", LM3644_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(lm3644_hwen_high);
	}
	lm3644_hwen_low = pinctrl_lookup_state(lm3644_pinctrl, LM3644_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(lm3644_hwen_low)) {
		pr_err("Failed to init (%s)\n", LM3644_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(lm3644_hwen_low);
	}
	
#ifdef HOPE_ADD_IRQ_FLASH
	lm3644_flash_irq = pinctrl_lookup_state(lm3644_pinctrl, LM3644_PINCTRL_STATE_FLASH_IRQ);
	if (IS_ERR(lm3644_flash_irq)) {
		pr_err("Failed to init (%s)\n", LM3644_PINCTRL_STATE_FLASH_IRQ);
		ret = PTR_ERR(lm3644_flash_irq);
	}
#endif
	return ret;
}

static int lm3644_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(lm3644_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case LM3644_PINCTRL_PIN_HWEN:
		mutex_lock(&pin_set_mutex);
		if (state == LM3644_PINCTRL_PINSTATE_LOW && !IS_ERR(lm3644_hwen_low))
			pinctrl_select_state(lm3644_pinctrl, lm3644_hwen_low);
		else if (state == LM3644_PINCTRL_PINSTATE_HIGH && !IS_ERR(lm3644_hwen_high))
			pinctrl_select_state(lm3644_pinctrl, lm3644_hwen_high);
		else
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		mutex_unlock(&pin_set_mutex);
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	/*pr_debug("pin(%d) state(%d)\n", pin, state);*/
	/*pr_debug("lm3644_i2c_client->addr = 0x%x, device_id =0x%x\n", lm3644_i2c_client->addr, device_id);*/
	return ret;
}


/******************************************************************************
 * lm3644 operations
 *****************************************************************************/
#if defined  (CONFIG_MTK_CAM_PD2083F_EX)
static const int lm3644_current[LM3644_LEVEL_NUM] = {  /*current:mA */
	 33,  51,  65,  75,  100, 120, 140, 198,  246, 304,
	351, 398, 445, 503, 550, 597, 656, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100};


static const unsigned char lm3644_torch_level[LM3644_LEVEL_NUM] = {
	0x17, 0x24, 0x2E, 0x35, 0x46, 0x55, 0x63, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36413_torch_level[LM3644_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x22, 0x27, 0x2F, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

static const unsigned char lm3644_flash_level[LM3644_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};
	
#elif defined(CONFIG_MTK_CAM_PD2066A) || defined(CONFIG_MTK_CAM_PD2066BA) || defined(CONFIG_MTK_CAM_PD2066F_EX) || defined(CONFIG_MTK_CAM_PD2103F_EX) || defined(CONFIG_MTK_CAM_PD2104F_EX)


static const int lm3644_current[LM3644_LEVEL_NUM] = {  /*current:mA */
	 33,  51,  65,  75,  93, 100, 140, 198,  246, 304,
	351, 398, 445, 503, 550, 597, 656, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100};


static const unsigned char lm3644_torch_level[LM3644_LEVEL_NUM] = {
	0x17, 0x24, 0x2E, 0x35, 0x42, 0x46, 0x63, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36413_torch_level[LM3644_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x1F, 0x22, 0x2F, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

static const unsigned char lm3644_flash_level[LM3644_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

#else

static const int lm3644_current[LM3644_LEVEL_NUM] = {  /*current:mA */
	 33,  51,  65,  75,  93, 116, 140, 198,  246, 304,
	351, 398, 445, 503, 550, 597, 656, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100};


static const unsigned char lm3644_torch_level[LM3644_LEVEL_NUM] = {
	0x17, 0x24, 0x2E, 0x35, 0x42, 0x52, 0x63, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36413_torch_level[LM3644_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x1F, 0x27, 0x2F, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

static const unsigned char lm3644_flash_level[LM3644_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};
#endif

static unsigned char lm3644_reg_enable;
static int lm3644_level_ch1 = -1;
static int lm3644_level_ch2 = -1;

static int lm3644_is_torch(int level)
{
	if (level >= LM3644_LEVEL_TORCH)
		return -1;

	return 0;
}

static int lm3644_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= LM3644_LEVEL_NUM)
		level = LM3644_LEVEL_NUM - 1;

	return level;
}

static int lm3644_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_HIGH);
	mdelay(2);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

/* i2c wrapper function */
static int state_pin_short = 0;
static int lm3644_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char  flags_1_val, flags_2_val, timimg_conf_val;
	
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	pr_info("state_pin_short =%d\n", state_pin_short);
 	if(state_pin_short == 1){
	pr_info("state_pin_short =%d\n", state_pin_short);
 	return 0;
 	}
	
	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_HIGH);
	mdelay(2);
	timimg_conf_val = lm3644_read_reg(lm3644_i2c_client, LM3644_REG_TIMING_CONF);
	flags_1_val = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_FLAGS_1);
	flags_2_val = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_FLAGS_2);
	pr_debug("flags_1_val =0x%x, flags_2_val=0x%x, timimg_conf_val = 0x%x\n", flags_1_val, flags_2_val, timimg_conf_val);
	if((flags_1_val & 0x7E)||(flags_2_val & 0x1E))
		pr_err("flashlight err flags_1_val = 0x%x, flags_2_val = 0x%x, lm3644_i2c_client->addr = 0x%x, device_id =0x%x\n", flags_1_val, flags_2_val, lm3644_i2c_client->addr, device_id);

	ret = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_TORCH_LEVEL_LED1);
	/*pr_debug("LM3644_REG_TORCH_LEVEL_LED1 =0x%x\n",ret);*/

	ret = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_FLASH_LEVEL_LED1);
	/*pr_debug("LM3644_REG_FLASH_LEVEL_LED1 =0x%x\n", ret);*/

	ret = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_TORCH_LEVEL_LED2);
	/*pr_debug("LM3644_REG_TORCH_LEVEL_LED2 =0x%x\n",ret);*/

	ret = lm3644_read_reg(lm3644_i2c_client,LM3644_REG_FLASH_LEVEL_LED2);
	/*pr_debug("LM3644_REG_FLASH_LEVEL_LED2 =0x%x\n", ret);*/
	
	/*pr_debug("===hope lm3644_write_reg  reg =%d,val = %d\n",reg, val);*/
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if(ret == EIO){
	pr_err("EIO state_pin_short =%d\n", state_pin_short);
	state_pin_short = 1;
	}
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* flashlight enable function */
static int lm3644_enable_ch1(void)
{
	unsigned char reg, val;

	reg = LM3644_REG_ENABLE;
	if (!lm3644_is_torch(lm3644_level_ch1)) {
		/* torch mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED1_FLASH;
	}
	val = lm3644_reg_enable;

	return lm3644_write_reg(lm3644_i2c_client, reg, val);
}

static int lm3644_enable_ch2(void)
{
#ifdef HOPE_ADD_IRQ_FLASH

	unsigned char reg, val;
	if (g_flash_irq_num > 0 && irq_enable_count == 0) {
		pr_debug("RED FLASH ON  g_flash_irq_num =%d, device_id = 0x%x\n", g_flash_irq_num, device_id);
		irq_enable_count++;
		led_count = 0;
		StartTime = ktime_get();
		first_led_on = 0;
		sof_flag = 0;
		reg = LM3644_REG_TIMING_CONF;
		
		if (device_id == 0x12)
			val = LM3644_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT_160MS; /*setting time_out = 160 ms*/
		else
			val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT_150MS; /*setting time_out = 150 ms*/
		 lm3644_write_reg(lm3644_i2c_client, reg, val);
		 enable_irq(g_flash_irq_num);
		 return 0;
	}else{
		pr_err("lm3644_enable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d, device_id = %d\n", g_flash_irq_num, irq_enable_count, device_id);
		return -1;
	}
#else
	
	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;
	if (!lm3644_is_torch(lm3644_level_ch2)) {
		/* torch mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED2_FLASH;
	}
	val = lm3644_reg_enable;
	return lm3644_write_reg(lm3644_i2c_client, reg, val);
	
#endif
}

static int lm3644_enable_ch2_store(void)
{
	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;
	if (!lm3644_is_torch(lm3644_level_ch2)) {
		/* torch mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		lm3644_reg_enable |= LM3644_ENABLE_LED2_FLASH;
	}
	val = lm3644_reg_enable;
	return lm3644_write_reg(lm3644_i2c_client, reg, val);
}


static int lm3644_enable(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		lm3644_enable_ch1();
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_enable_store(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		lm3644_enable_ch1();
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_enable_ch2_store();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int lm3644_disable_ch1(void)
{
	unsigned char reg, val;

	reg = LM3644_REG_ENABLE;
	if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
	} else {
		/* if LED 2 is disable, disable LED 1 and clear mode */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED1_FLASH);
	}
	val = lm3644_reg_enable;

	return lm3644_write_reg(lm3644_i2c_client, reg, val);
}

static int lm3644_disable_ch2(void)
{
	
#ifdef HOPE_ADD_IRQ_FLASH
	unsigned char reg, val;
	if (g_flash_irq_num > 0 && irq_enable_count == 1) {
		pr_debug("RED FLASH OFF  g_flash_irq_num =%d\n", g_flash_irq_num);
		disable_irq(g_flash_irq_num);
		irq_enable_count--;
		led_count = 0;
		StartTime = ktime_get();
		first_led_on = 0;
		sof_flag = 0;
		del_timer(&flash_delay_off_timer);
		/* set torch current ramp time and flash timeout */
		reg = LM3644_REG_TIMING_CONF;
		val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;  /*setting time_out = max default*/
		return lm3644_write_reg(lm3644_i2c_client, reg, val);
	}else{
		pr_err("lm3644_disable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d\n", g_flash_irq_num, irq_enable_count);
		return -1;
	}
#else

	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;
	if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = lm3644_reg_enable;
	return lm3644_write_reg(lm3644_i2c_client, reg, val);
#endif
}


static int lm3644_disable_ch2_store(void)
{
	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;
	if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = lm3644_reg_enable;
	return lm3644_write_reg(lm3644_i2c_client, reg, val);
}
static int lm3644_disable(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		lm3644_disable_ch1();
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_disable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_disable_store(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		lm3644_disable_ch1();
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_disable_ch2_store();
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int lm3644_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = lm3644_verify_level(level);

	/* set torch brightness level */
	reg = LM3644_REG_TORCH_LEVEL_LED1;
	if (device_id == 0x12)
		val = aw36413_torch_level[level];
	else
		val = lm3644_torch_level[level];
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);

	lm3644_level_ch1 = level;

	/* set flash brightness level */
	reg = LM3644_REG_FLASH_LEVEL_LED1;
	val = lm3644_flash_level[level];
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);

	return ret;
}

static int lm3644_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = lm3644_verify_level(level);

	/* set torch brightness level */
	reg = LM3644_REG_TORCH_LEVEL_LED2;

	if (device_id == 0x12)
		val = aw36413_torch_level[level];
	else
		val = lm3644_torch_level[level];
	
#ifdef CONFIG_MTK_CAM_PD2083F_EX
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);
#else
	ret = lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);
#endif
	
	lm3644_level_ch2 = level;

	/* set flash brightness level */
	reg = LM3644_REG_FLASH_LEVEL_LED2;
	val = lm3644_flash_level[level];
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);

	return ret;
}

#ifdef HOPE_ADD_IRQ_FLASH
static int lm3644_set_red_flash_level_and_enabe_ch2(void)
{
	int ret;
	//unsigned char reg, val;
	/* set torch current ramp time and flash timeout */
	//reg = LM3644_REG_TIMING_CONF;
	//val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT_90MS;
	//ret = lm3644_write_reg(lm3644_i2c_client, reg, val);
	
	/* set flash brightness level */
	lm3644_reg_enable |= LM3644_ENABLE_LED2_FLASH;
	
	ret = lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*write reg5 bit 7 to 0*/
	ret = lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED2, 0x6E);/*6E = 1.3A*/
	ret = lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
	del_timer(&flash_delay_off_timer);
	flash_delay_off_timer.expires = jiffies + msecs_to_jiffies(100);  /*100ms*/
	flash_delay_off_timer.data = 0;
	flash_delay_off_timer.function = set_flash_led_delay_off;
	add_timer(&flash_delay_off_timer);
	return ret;
}
#endif

static int lm3644_set_level(int channel, int level)
{
	int ret;
	unsigned char reg, val;
	/* set torch current ramp time and flash timeout */
	reg = LM3644_REG_TIMING_CONF;
	val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);
	
	if (channel == LM3644_CHANNEL_CH1)
		lm3644_set_level_ch1(level);
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int lm3644_init(void)
{
	int ret;
	unsigned char reg, val;

	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_HIGH);
	/*msleep(20);*/
	if (lock_touch == 0 && lock_touch_sub == 0) {
	/* clear enable register */
	reg = LM3644_REG_ENABLE;
	val = LM3644_DISABLE;
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);

	lm3644_reg_enable = val;
	}

	/* set torch current ramp time and flash timeout */
	reg = LM3644_REG_TIMING_CONF;
	val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);

#ifdef CONFIG_MTK_CAM_PD2083F_EX
	/* set boost current limit config */
	reg = LM3644_REG_BOOST;
	val = LM3644_REG_BOOST_CRRRENT_LIMIT_1_8A;
	ret = lm3644_write_reg(lm3644_i2c_client, reg, val);
	//mdelay(2);	
	//boost_val = lm3644_read_reg(lm3644_i2c_client, LM3644_REG_BOOST);
	//pr_err("lm3644_init LM3644_REG_BOOST =0x%x\n",boost_val);
#endif
	return ret;
}

/* flashlight uninit */
int lm3644_uninit(void)
{
	if (lock_touch == 0 && lock_touch_sub == 0) {
	lm3644_disable(LM3644_CHANNEL_CH1);
	lm3644_disable(LM3644_CHANNEL_CH2);
	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
	pr_debug("lm3644_uninit LM3644_PINCTRL_PINSTATE_LOW\n");
	}
	return 0;
}

#ifdef HOPE_ADD_IRQ_FLASH
static irqreturn_t vivo_subflash_ISR(int irq, void *dev_id)
{
	/*pr_debug("vivo_subflash_ISR \n");*/
	schedule_delayed_work(&ir_delayed_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}
static void ir_delayed_func(struct work_struct *work)
{
	
	/*int ret;*/
	unsigned char reg, flags_1_val, flags_2_val;
	u64 deltaTime = ktime_us_delta(ktime_get(), StartTime);
	pr_debug("hope  led_count = %d, first_led_on =%d sof_flag = %d\n", led_count, first_led_on, sof_flag);
	pr_debug("hope  deltaTime %lluus\n", ktime_us_delta(ktime_get(), StartTime));
	if(first_led_on == 0 && sof_flag ==1 ){
			first_led_on = 2;
	}
	
	if(!lm3644_i2c_client ){
		pr_err("i2c client is NULL.\n");
	}else{
		if ((deltaTime > 480000 && led_count < 8) || (first_led_on == 2)) {/*380000*/
			StartTime = ktime_get();
			led_count++;
			
			reg = LM3644_REG_FLAGS_1;
			flags_1_val = lm3644_read_reg(lm3644_i2c_client,reg);
			
			reg = LM3644_REG_FLAGS_2;
			flags_2_val = lm3644_read_reg(lm3644_i2c_client,reg);

			if((flags_1_val & 0x7E)||(flags_2_val & 0x1E))
				pr_err("flashlight err flags_1_val = 0x%x, flags_2_val = 0x%x, lm3644_i2c_client->addr = 0x%x, device_id =0x%x\n", flags_1_val, flags_2_val, lm3644_i2c_client->addr, device_id);
			
				pr_debug("sub flash mode start on \n");
				lm3644_set_red_flash_level_and_enabe_ch2();
		}
		if(first_led_on != 0)
			first_led_on++;
	}
}

static void set_flash_led_delay_off(unsigned long arg)
{
	pr_debug("set_flash_led_delay_off \n");
	schedule_work(&flash_delay_off_work);
	del_timer(&flash_delay_off_timer);
}

static void flash_delay_off_func(struct work_struct *work)
 {
	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;
	
	pr_err("flash_delay_off_func start\n");
	
	if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = lm3644_reg_enable;
	 lm3644_write_reg(lm3644_i2c_client, reg, val);
	 
	 pr_err("flash_delay_off_func end\n");
}

#endif

static int set_flashlight_state(int state)
{
	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_HIGH);
	pr_info("set_flashlight_state check state:%d \n", state);
	switch (state) {
	
	case FRONT_TORCH_3rd_ONE_ON:
		
		pr_info("FRONT_TORCH_3rd_ONE_ON\n");
		lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
#if defined  (CONFIG_MTK_CAM_PD2083F_EX)
		if (device_id == 0x12)
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x19);/*(Brightnees code x 1.4mA)+0.997ma Torch 0x1A = 78mA*/
		else
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*0x47 = 100mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA(Brightnees code x 1.4mA)+0.997ma Torch*/
#else
		if (device_id == 0x12)
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x22);/*(Brightnees code x 1.4mA)+0.997ma Torch 0x1A = 78mA*/
		else
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*0x47 = 100mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA(Brightnees code x 1.4mA)+0.997ma Torch*/
#endif
		lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		lock_touch = 1;
		break;
	case FRONT_TORCH_3rd_ONE_OFF:
		if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED2) {
			/* if LED 2 is enable, disable LED 1 */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED1_FLASH);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
			lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
		}
		lock_touch = 0;
		pr_info("FRONT_TORCH_3rd_ONE_OFF\n");
		break;
	case FRONT_TORCH_3rd_TWO_ON:
		pr_info("FRONT_TORCH_3rd_TWO_ON\n");
		lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
#if defined  (CONFIG_MTK_CAM_PD2083F_EX)
		if (device_id == 0x12){
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x19);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x19);/*default 0x33 = 150mA 0x1A = 78mA (Brightness Code*2.91mA)+2.55mA Torch*/
		}else{
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x35);/*default 2E == 65mA 0x47 = 100mA 0x6A = 150mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA (Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#else
		if (device_id == 0x12){
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x22);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x22);/*default 0x33 = 150mA 0x1A = 78mA (Brightness Code*2.91mA)+2.55mA Torch*/
		}else{
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x35);/*default 2E == 65mA 0x47 = 100mA 0x6A = 150mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA (Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#endif
		lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		lock_touch_sub = 1;  
		break;
	case FRONT_TORCH_3rd_TWO_OFF:
		if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
			/* if LED 1 is enable, disable LED 2 */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
			lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
		}
		pr_info("FRONT_TORCH_3rd_TWO_OFF\n");
		lock_touch_sub = 0;  
		break;
	case BBK_TORCH_LOW:
		lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
#if defined  (CONFIG_MTK_CAM_PD2083F_EX)
		if (device_id == 0x12) {
			pr_info("PD2083 BBK_TORCH_LOW, LM3644_REG_TORCH_LEVEL_LED1 = 0x19\n");
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x19);/*(Brightness Code*2.91mA)+2.55mA Torch*/
		} else {
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*(Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#else
		if (device_id == 0x12) {
			pr_info("BBK_TORCH_LOW, AW36413_REG_TORCH_LEVEL_LED1 = 0x19\n");
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x19);/*aw36413 0x19 = 75mA (Brightness Code x 2.91mA)+2.55mA Torch*/
		} else {
			pr_info("BBK_TORCH_LOW, LM3644_REG_TORCH_LEVEL_LED1 = 0x35\n");
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*KTD2687 0x35 = 75mA (Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#endif
		lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		lock_touch = 1;
		break;
	case BBK_TORCH_OFF:
		if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED2) {
			/* if LED 2 is enable, disable LED 1 */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED1_FLASH);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
			lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
		}
		lock_touch = 0;
		break;
	case FRONT_TORCH_ON:
		pr_info("FRONT_TORCH_ON device_id=%d\n",device_id);
		lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
#if defined  (CONFIG_MTK_CAM_PD2083F_EX)
		if (device_id == 0x12){
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x19);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x19);/*default 0x33 = 150mA (Brightness Code*2.91mA)+2.55mA Torch*/
		}else{
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x6A);/*default 2E == 65mA  0x6A = 150mA(Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#else
		if (device_id == 0x12){
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x22);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x22);/*default 0x33 = 150mA (Brightness Code*2.91mA)+2.55mA Torch*/
		}else{
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED2, 0x6A);/*default 2E == 65mA  0x6A = 150mA(Brightnees code x 1.4mA)+0.997ma Torch*/
		}
#endif
		lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		lock_touch_sub = 1;  
		break;
	case FRONT_TORCH_OFF:
		if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
			/* if LED 1 is enable, disable LED 2 */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
		} else {
			/* disable LED 1 LED 2 and clear mode */
			lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, lm3644_reg_enable);
			lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
		}
		pr_info("FRONT_TORCH_OFF\n");
		lock_touch_sub = 0;  
		break;
	case BBK_FLASH_AT_TEST:
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TIMING_CONF, 0x1f);/*vivo liuguangwei change flash time out from 150ms to 400ms*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED1, 0x54);
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, LM3644_ENABLE_LED1_FLASH);
		break;
	case BBK_FLASH_AT_OFF:
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_ENABLE, (~LM3644_ENABLE_LED1_FLASH));
		break;
	default:
		pr_info("set_flashlight_state No such command and arg\n");
		return -ENOTTY;
	}
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer lm3644_timer_ch1;
static struct hrtimer lm3644_timer_ch2;
static unsigned int lm3644_timeout_ms[LM3644_CHANNEL_NUM];

static void lm3644_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	lm3644_disable_ch1();
}

static void lm3644_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	lm3644_disable_ch2();
}

static enum hrtimer_restart lm3644_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&lm3644_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart lm3644_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&lm3644_work_ch2);
	return HRTIMER_NORESTART;
}

static int lm3644_timer_start(int channel, ktime_t ktime)
{
	if (channel == LM3644_CHANNEL_CH1)
		hrtimer_start(&lm3644_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == LM3644_CHANNEL_CH2)
		hrtimer_start(&lm3644_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_timer_cancel(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		hrtimer_cancel(&lm3644_timer_ch1);
	else if (channel == LM3644_CHANNEL_CH2)
		hrtimer_cancel(&lm3644_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int lm3644_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int led_state;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= LM3644_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		lm3644_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d lm3644_current[fl_arg->arg]=%d\n",
				channel, (int)fl_arg->arg,lm3644_current[fl_arg->arg]);
		lm3644_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (lm3644_timeout_ms[channel]) {
				ktime = ktime_set(lm3644_timeout_ms[channel] / 1000,
						(lm3644_timeout_ms[channel] % 1000) * 1000000);
				lm3644_timer_start(channel, ktime);
			}
			lm3644_enable(channel);
		} else {
		if (lock_touch == 0 && lock_touch_sub == 0) {
			lm3644_disable(channel);
			lm3644_timer_cancel(channel);
			}
		}
		break;

    case FLASH_IOCTL_SET_LED_STATE:
		pr_info("FLASH_IOCTL_SET_LED_STATE(channel %d): arg: %d\n",
				channel, (int)fl_arg->arg);
		led_state = (int)fl_arg->arg;
		set_flashlight_state(led_state);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = LM3644_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = LM3644_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = lm3644_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = lm3644_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = LM3644_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm3644_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3644_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3644_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&lm3644_mutex);
	if (set) {
		if (!use_count)
			ret = lm3644_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = lm3644_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&lm3644_mutex);

	return ret;
}

static ssize_t lm3644_strobe_store(struct flashlight_arg arg)
{
	int i;

	if (arg.channel == LM3644_CHANNEL_CH2 && arg.level == 27){
		if(arg.dur == 200)
			set_flashlight_state(10);
		if(arg.dur == 300)
			set_flashlight_state(11);
	}else{
		lm3644_set_driver(1);
		if(arg.channel == LM3644_CHANNEL_CH2 && arg.level == 26){
			pr_debug("====hope arg.channel = %d, arg.level = %d\n", arg.channel, arg.level);
			lm3644_level_ch2 = arg.level;
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/	
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED2, 0x6E);/*0x6E = 1.3A,0x65 = 1.2A*/
			for (i = 0; i < 10; i++){
				lm3644_enable_store(arg.channel);
				msleep(arg.dur);
				lm3644_disable_store(arg.channel);
				msleep(330);
				pr_debug("====hope arg.dur = %d, disable = 330\n", arg.dur);
			}
		}else{
			lm3644_set_level(arg.channel, arg.level);
		}
#if 0
		if(arg.channel == LM3644_CHANNEL_CH1 && arg.level == 26){
			pr_debug("====hope arg.channel = %d, arg.level = %d\n", arg.channel, arg.level);
			lm3644_level_ch1 = arg.level;
			/*lm3644_write_reg(lm3644_i2c_client, LM3644_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));default value is 0xbf,bit7 should set to 0*/
			/*lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));default value is 0xbf,bit7 should set to 0*/	
			lm3644_write_reg(lm3644_i2c_client, LM3644_REG_FLASH_LEVEL_LED1, 0x6E);
		}else{
			lm3644_set_level(arg.channel, arg.level);
		}
#endif
		lm3644_timeout_ms[arg.channel] = 0;
		lm3644_enable_store(arg.channel);
		msleep(arg.dur);
		lm3644_disable_store(arg.channel);
		lm3644_set_driver(0);
		}
	return 0;
}

static struct flashlight_operations lm3644_ops = {
	lm3644_open,
	lm3644_release,
	lm3644_ioctl,
	lm3644_strobe_store,
	lm3644_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int lm3644_chip_init(struct lm3644_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * lm3644_init();
	 */

	return 0;
}

static int lm3644_parse_dt(struct device *dev,
		struct lm3644_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num * sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE, LM3644_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel, pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int lm3644_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3644_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3644_chip_data *chip;
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct lm3644_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = lm3644_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	lm3644_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&lm3644_work_ch1, lm3644_work_disable_ch1);
	INIT_WORK(&lm3644_work_ch2, lm3644_work_disable_ch2);

	/* init timer */
	hrtimer_init(&lm3644_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3644_timer_ch1.function = lm3644_timer_func_ch1;
	hrtimer_init(&lm3644_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3644_timer_ch2.function = lm3644_timer_func_ch2;
	lm3644_timeout_ms[LM3644_CHANNEL_CH1] = 100;
	lm3644_timeout_ms[LM3644_CHANNEL_CH2] = 100;

	/* init chip hw */
	lm3644_chip_init(chip);

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	pr_debug("===hope enter chip id select\n");
	device_id = lm3644_read_reg(lm3644_i2c_client, LM3644_DEVICE_ID);
	pr_info("lm3644_i2c_probe lm3644 DEVICE_ID = 0x%x\n", device_id);
	if(device_id  != 0x02){
		lm3644_i2c_client->addr = 0x6B;
		device_id = lm3644_read_reg(lm3644_i2c_client, LM3644_DEVICE_ID);
		pr_debug("lm3644_i2c_probe lm36413 DEVICE_ID = 0x%x\n", device_id);
	}
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(&pdata->dev_id[i], &lm3644_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(LM3644_NAME, &lm3644_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}
	/*set_flashlight_state(11); hope for test use */
	lm3644_pinctrl_set(LM3644_PINCTRL_PIN_HWEN, LM3644_PINCTRL_PINSTATE_LOW);
	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int lm3644_i2c_remove(struct i2c_client *client)
{
	struct lm3644_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(LM3644_NAME);

	/* flush work queue */
	flush_work(&lm3644_work_ch1);
	flush_work(&lm3644_work_ch2);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id lm3644_i2c_id[] = {
	{LM3644_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id lm3644_i2c_of_match[] = {
	{.compatible = LM3644_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver lm3644_i2c_driver = {
	.driver = {
		.name = LM3644_NAME,
#ifdef CONFIG_OF
		.of_match_table = lm3644_i2c_of_match,
#endif
	},
	.probe = lm3644_i2c_probe,
	.remove = lm3644_i2c_remove,
	.id_table = lm3644_i2c_id,
};

#ifdef CONFIG_OF
static const struct of_device_id lm3644_of_match[] = {
	{.compatible = LM3644_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);
#else
static struct platform_device lm3644_platform_device[] = {
	{
		.name = LM3644_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, lm3644_platform_device);
#endif

/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int lm3644_probe(struct platform_device *dev)
{
#ifdef HOPE_ADD_IRQ_FLASH
	int ret=0;
	struct device_node *node = NULL;
	u32 ints1[4] = { 0 };
#endif
	pr_debug("Probe start.\n");

	/* init pinctrl */
	if (lm3644_pinctrl_init(dev)) {
		pr_debug("Failed to init pinctrl.\n");
		return -1;
	}
	
	if (i2c_add_driver(&lm3644_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

#ifdef HOPE_ADD_IRQ_FLASH
	pinctrl_select_state(lm3644_pinctrl, lm3644_flash_irq);
	node = of_find_matching_node(node, lm3644_of_match);
	if (node) {
		g_gpio_pin = of_get_named_gpio(node, "deb-gpios", 0);
		ret = of_property_read_u32(node, "debounce", &g_gpio_headset_deb);
		if (ret < 0) {
			pr_debug("debounce time not found\n");
			return ret;
		}
		/*gpio_set_debounce(g_gpio_pin, g_gpio_headset_deb);*/
		
		g_flash_irq_num = irq_of_parse_and_map(node, 0);
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		g_accdet_eint_type = ints1[1];
		pr_debug("[flash] gpiopin:%d debounce:%d accdet_irq:%d accdet_eint_type:%d\n",
				g_gpio_pin, g_gpio_headset_deb, g_flash_irq_num, g_accdet_eint_type);
		ret = request_irq(g_flash_irq_num, vivo_subflash_ISR, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "flash_irq", NULL);
		if (ret != 0) {
			pr_debug("[flash]EINT IRQ LINE NOT AVAILABLE\n");
			goto ir_irq_err;
		} else {
			pr_debug("[flash]accdet set EINT finished, accdet_irq=%d, headsetdebounce=%d\n",
				     g_flash_irq_num, g_gpio_headset_deb);
		}
		
		INIT_DELAYED_WORK(&ir_delayed_work, ir_delayed_func);
		INIT_WORK(&flash_delay_off_work, flash_delay_off_func);
		init_timer(&flash_delay_off_timer); // add for flash 100ms off control
		
		disable_irq_nosync(g_flash_irq_num);
				
		pr_debug("hope \n");
	} else {
		pr_debug("[flash]%s can't find compatible node\n", __func__);
	}
	
	return 0;
	
ir_irq_err:
	free_irq(g_flash_irq_num, NULL);
	return -1;
#endif
	pr_debug("Probe done.\n");

	return 0;
}

static int lm3644_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&lm3644_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}


static struct platform_driver lm3644_platform_driver = {
	.probe = lm3644_probe,
	.remove = lm3644_remove,
	.driver = {
		.name = LM3644_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3644_of_match,
#endif
	},
};

static int __init flashlight_lm3644_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&lm3644_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&lm3644_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_lm3644_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&lm3644_platform_driver);

	pr_debug("Exit done.\n");
}

late_initcall(flashlight_lm3644_init);
module_exit(flashlight_lm3644_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ziyu Jiang <jiangziyu@meizu.com>");
MODULE_DESCRIPTION("MTK Flashlight LM3644 Driver");
