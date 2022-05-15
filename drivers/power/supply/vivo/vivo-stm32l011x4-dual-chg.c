/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "STM32: %s: " fmt, __func__

#include <linux/i2c.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>

#include <max20328-i2c.h>
#include <linux/power/vivo/power_supply_lite.h>
#include <linux/power/vivo/interact.h>
#include "vivo-stm32l011x4-dual-chg.h"
#include "voter.h"
#include "fuel_summary.h"


#define STM32L011_DEBUG_BIN		"/data/DirectCharger.bin"

//#undef pr_info
//#define pr_info(fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)

#define vivo_wake_lock(lock)	__pm_stay_awake(lock)
#define vivo_wake_unlock(lock)	__pm_relax(lock)

#define MONITOR_WORK_DELAY_MS		1380


/*inner-var statement */
static struct irq_handler_info handlers[2];
static struct stm32l011_mcu *this_chip = NULL;


/* function statement */
static void stm32l011_check_dchg_error_state(struct stm32l011_mcu *chip);
static bool fw_check_version(struct stm32l011_mcu *chip);
static int stm32l011_sbu_cable_id_detect(struct stm32l011_mcu *chip);
static int stm32l011_check_battery_high(struct stm32l011_mcu *chip);
static int stm32l011_adc_read(struct stm32l011_mcu *chip, struct iio_channel *chan, int *data);
static void stm32l011_reset_voters(struct stm32l011_mcu *chip);



int __attribute__ ((weak)) max20328_switch_mode_event(enum max_function event)
{
	return -ENODEV;
}

/*function*/
static int is_between(int left, int right, int value)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;

	return 0;
}

static u32 get_crc32(u32 crc32, u8 *p, u32 len)
{
	u8 remain, compensate;
	remain = len % BATCH_ALIGN;

	while (len--)
		crc32 = crc32_table[(crc32 >> 24) ^  *p++] ^ (crc32 << 8);

	if (0 != remain) {
		compensate = BATCH_ALIGN - remain;
		pr_debug("crc need align remain=%d, compensate=%d\n", remain, compensate);
		while (compensate--)
			crc32 = crc32_table[(crc32 >> 24) ^  0xFF] ^ (crc32 << 8);
	}

	return crc32;
}

static int stm32l011_get_usbsel_state(struct stm32l011_mcu *chip)
{
	int ret = 0;

	if (chip->usbsel_gpio > 0)
		ret = gpio_get_value(chip->usbsel_gpio);
	else
		ret = chip->usbsel_state;

	return ret;
}

static int stm32l011_get_power_gpio_state(struct stm32l011_mcu *chip)
{
	int ret = 0;

	if (chip->power_gpio > 0) {
		ret = gpio_get_value(chip->power_gpio);
		//pr_info("power_gpio:%d\n", ret);
	} else if (!IS_ERR_OR_NULL(chip->power_ldo)) {
		ret = regulator_is_enabled(chip->power_ldo);
		//pr_info("power_ldo:%d\n", ret);
	} else {
		pr_err("not found mcu power ctl !!!\n");
	}
	return ret;
}

static int stm32l011_pinctrl_power_gpio_state(struct stm32l011_mcu *chip, int state)
{
#if 1
	int ret = stm32l011_get_power_gpio_state(chip);

	if (state == ret) {
		pr_info("mcu_power is already %d, state=%d\n", ret, state);
		return 0;
	}

	switch (state) {
	case PINCFG_LOW:
		if (chip->power_gpio > 0) {
			gpio_direction_output(chip->power_gpio, 0);
		} else if (!IS_ERR_OR_NULL(chip->power_ldo)) {
			ret = regulator_disable(chip->power_ldo);
			pr_info("power_ldo disable, ret=%d\n", ret);
		} else {
			pr_err("not found mcu power ctl !!!\n");
		}
		break;
	case PINCFG_HIGH:
		if (chip->power_gpio > 0) {
			gpio_direction_output(chip->power_gpio, 1);
		} else if (!IS_ERR_OR_NULL(chip->power_ldo)) {
			ret = regulator_enable(chip->power_ldo);
			pr_info("power_ldo enable, ret=%d\n", ret);
		} else {
			pr_err("not found mcu power ctl !!!\n");
		}
		break;
	case PINCFG_DEFAULT:
		pr_warn("PINCFG_DEFAULT\n");
		break;
	}
#else
	if (!chip->pinctrl) {
		pr_err("pinctrl is null\n");
		return -EINVAL;
	}

	if (!chip->power_gpio_default || !chip->power_gpio_high || !chip->power_gpio_low) {
		pr_err("power gpio pinctrl is null\n");
		return -EINVAL;
	}

	switch (state) {
	case PINCFG_LOW:
		if (pinctrl_select_state(chip->pinctrl, chip->power_gpio_low))
			pr_err("failed to pinctrl_select_state: low\n");
		break;
	case PINCFG_HIGH:
		if (pinctrl_select_state(chip->pinctrl, chip->power_gpio_high))
			pr_err("failed to pinctrl_select_state: high\n");
		break;
	case PINCFG_DEFAULT:
		if (pinctrl_select_state(chip->pinctrl, chip->power_gpio_default))
			pr_err("failed to pinctrl_select_state: default\n");
		break;
	}
#endif
	return 0;
}

static int stm32l011_shift_en_enable(struct stm32l011_mcu *chip,bool enable)
{
	if (chip->shift_en_gpio <= 0) {
		pr_err("shift_en_gpio not configged\n");
		return 0;
	}
	if (enable) {
		gpio_direction_output(chip->shift_en_gpio,1);
	} else {
		gpio_direction_output(chip->shift_en_gpio,0);
	}
	pr_info("enable=%d,shift_en_gpio=%d\n",enable,gpio_get_value(chip->shift_en_gpio));
	return 0;
}

static int stm32l011_read_reg(struct stm32l011_mcu *chip, u8 reg, u8 *val)
{
	s32 ret;
	bool enable;

	enable = stm32l011_get_power_gpio_state(chip);
	if (!enable) {
		pr_err("mcu not power\n");
		return 0;
	}
	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		if (chip->ap_i2c_read_exception<0x1FFFFFFF)
			chip->ap_i2c_read_exception++;
		if (!chip->update_done && chip->burn_handling)
			chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__I2C_ERROR;
		pr_err("i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	//pr_info("Reading 0x%02x=0x%02x\n", reg, *val);
	return 0;
}

static int stm32l011_write_reg(struct stm32l011_mcu *chip, int reg, u8 val)
{
	s32 ret;
	bool enable;

	enable = stm32l011_get_power_gpio_state(chip);
	if (!enable)
		return 0;
	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		if (chip->ap_i2c_write_exception<0x1FFFFFFF)
			chip->ap_i2c_write_exception++;
		if (!chip->update_done && chip->burn_handling)
			chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__I2C_ERROR;
		pr_err("i2c write fail: can't write %02x to %02x: %d\n",
				val, reg, ret);
		return ret;
	}
	pr_info("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int stm32l011_masked_write(struct stm32l011_mcu *chip, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = stm32l011_read_reg(chip, reg, &temp);
	if (rc) {
		pr_err("read failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = stm32l011_write_reg(chip, reg, temp);
	if (rc) {
		pr_err("write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}
static void stm32l011_stay_awake(struct stm32l011_mcu *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_info("staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		//pm_stay_awake(chip->dev);
		//if (!wake_lock_active(&chip->stm32l011_wake_lock)) {
		//	wake_lock(&chip->stm32l011_wake_lock);
		//	pr_info("get wakelock\n");
		//}
		vivo_wake_lock(&chip->stm32l011_wake_lock);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void stm32l011_relax(struct stm32l011_mcu *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_info("relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		//pm_relax(chip->dev);
		//if (wake_lock_active(&chip->stm32l011_wake_lock)) {
		//	pr_info("releasing wakelock\n");
		//	wake_unlock(&chip->stm32l011_wake_lock);
		//}
		vivo_wake_unlock(&chip->stm32l011_wake_lock);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};
#if 0
static int stm32l011_write_block(struct stm32l011_mcu *chip, u8 reg,
		u8 len,u8 *buf)
{
#if 0

	int rc;

	rc = i2c_smbus_write_block_data(chip->client, reg,len, buf);
	if (rc) {
		pr_err("write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;

#else
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msg[1];
	int ret;
	bool enable;

	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = len;

	mutex_lock(&chip->stm32l011_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&chip->stm32l011_i2c_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
#endif
}
static int stm32l011_read_block(struct stm32l011_mcu *chip, u8 reg,
		u8 len,char *buf)
{
#if 0

	int rc;
	rc = i2c_smbus_read_block_data(chip->client, reg, buf);
	if (rc < 0) {
		pr_err("Failed to read Manufacturer ID\n");
		return rc;
	}
	return 0;
#else
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msg[2];
	int ret;
	bool enable;

	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	if (!client->adapter)
		return -ENODEV;
	if (len > I2C_SMBUS_BLOCK_MAX -1)
		len = I2C_SMBUS_BLOCK_MAX - 1;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	mutex_lock(&chip->stm32l011_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&chip->stm32l011_i2c_lock);

	if (ret < 0) {
		pr_err("i2c failed to read ret=%d\n",ret);
		return ret;
	}
	return ret;
#endif
}
#endif
#if 0
static int stm32l011_hw_deinit(struct stm32l011_mcu *chip);
static int stm32l011_hw_init(struct stm32l011_mcu *chip);
static int stm32l011_usbsel_enable(struct stm32l011_mcu *chip,bool enable);
static int stm32l011_power_enable(struct stm32l011_mcu *chip,bool enable);
#endif
static int stm32l011_temp_from_voltage(struct stm32l011_mcu *chip,unsigned long voltage,int type) {
	int index;
	int temp = 0;

	//voltage = voltage * 1000;
	switch(type) {
	case MCU_TEMP_PCB_BTB:
	case MCU_TEMP_BAT_BTB:
		voltage = voltage * 1000;
		for (index = ARRAY_SIZE(adc_map_temp_table1) - 1; index >= 1; index--) {
			if (is_between(adc_map_temp_table1[index].voltage, adc_map_temp_table1[index-1].voltage, voltage)) {
				temp = adc_map_temp_table1[index].temp;
				//pr_warn("type=%d voltage=%ld temp=%d \n",type,voltage,temp);
				break;
			}
		}
		break;
	case MCU_TEMP_USB_CONN:
	case MCU_TEMP_BAT:
		if (chip->dchg_version == NXP_PCA9486_DCHG) {
			voltage = voltage * 1000;
			for (index = ARRAY_SIZE(adc_map_temp_table2_2) - 1; index >= 1; index--) {
				if (is_between(adc_map_temp_table2_2[index].voltage, adc_map_temp_table2_2[index-1].voltage, voltage)) {
					temp = adc_map_temp_table2_2[index].temp;
					break;
				}
			}
		} else if (chip->dchg_version == LN8000_DCHG) {
			voltage = voltage * 1000;
			for (index = ARRAY_SIZE(adc_map_temp_table2_2) - 1; index >= 1; index--) {
				if (is_between(adc_map_temp_table2_2[index].voltage, adc_map_temp_table2_2[index-1].voltage, voltage)) {
					temp = adc_map_temp_table2_2[index].temp;
					break;
				}
			}
		} else {
			/*bq25970 persent read from regs,not voltage */
			voltage = voltage * 97;
			for (index = ARRAY_SIZE(adc_map_temp_table2) - 1; index >= 1; index--) {
				if (is_between(adc_map_temp_table2[index].voltage, adc_map_temp_table2[index-1].voltage, voltage)) {
					temp = adc_map_temp_table2[index].temp;
					//pr_warn("bat voltage=%ld temp=%d \n",voltage,temp);
					break;
				}
			}
		}
		break;
	case MCU_TEMP_ADAPTER:
	case MCU_TEMP_ADAPTER_CONN:
		voltage = voltage * 1000;
		for (index = ARRAY_SIZE(adc_map_temp_table4) - 1; index >= 1; index--) {
			if (is_between(adc_map_temp_table4[index].voltage, adc_map_temp_table4[index-1].voltage, voltage)) {
				temp = adc_map_temp_table4[index].temp;
				//pr_warn("adapter voltage=%ld temp=%d \n",voltage,temp);
				break;
			}
		}
		break;
	default:
		pr_warn("no ntc temp map table\n");
		break;
	}
	pr_warn("type=%d voltage=%ld temp=%d \n",type,voltage,temp);
	//temp = temp/10;
	return temp;
}

static uint16_t stm32l011_get_int_status_flag(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	if (chip->dchg_version == BQ25970_SINGLE_DCHG || chip->dchg_version == BQ25970_DUAL_DCHG) {
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x04, &chip->int_stat_08);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x05, &chip->int_stat_0A);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x11, &chip->dchg_status);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x12, &chip->int_stat_0B);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x17, &chip->int_stat_0E);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x18, &chip->int_stat_11);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x19, &chip->int_stat_2D);
		if (chip->enable_slave_charger) {
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x2E, &chip->slave_int_stat_08);
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x2F, &chip->slave_int_stat_0A);
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x30, &chip->slave_int_stat_0E);
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x31, &chip->slave_int_stat_11);
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x32, &chip->slave_int_stat_2D);
		}
	} else if (chip->dchg_version == LN8000_DCHG) {
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x04, &chip->int_stat_ln8000_01);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x05, &chip->int_stat_ln8000_03);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x11, &chip->dchg_status);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x12, &chip->int_stat_ln8000_04);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x17, &chip->int_stat_ln8000_05);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x18, &chip->int_stat_ln8000_06);
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS_START + 0x19, &chip->int_stat_ln8000_08);
	}
	return ret;
}

static uint16_t stm32l011_get_ibusADC(struct stm32l011_mcu *chip)
{

	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t ibus_ma = 0;

	ret = stm32l011_read_reg(chip,BQ25970_REG_15,&high);
	ret = stm32l011_read_reg(chip,BQ25970_REG_16,&low);
	ibus_ma = (high & BQ25970_IBUS_ADC_MASK) << 8 | low;
	//	pr_info("ibus_ma=%d\n",ibus_ma);
	return ibus_ma;
}
static uint16_t stm32l011_get_slave_ibusADC(struct stm32l011_mcu *chip)
{

	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t ibus_ma = 0;

	ret = stm32l011_read_reg(chip,BQ25970_REG_2B,&high);
	ret = stm32l011_read_reg(chip,BQ25970_REG_2C,&low);
	ibus_ma = (high & BQ25970_IBUS_ADC_MASK) << 8 | low;
	//	pr_info("slave ibus_ma=%d\n",ibus_ma);
	return ibus_ma;
}
//uint8_t low,high;
static uint16_t stm32l011_get_vbusADC(struct stm32l011_mcu *chip)
{

	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t vbus_mv = 0;

	ret = stm32l011_read_reg(chip,BQ25970_REG_13,&high);
	ret = stm32l011_read_reg(chip,BQ25970_REG_14,&low);
	vbus_mv = (high & BQ25970_VBUS_ADC_MASK) << 8 | low;
	//pr_info("vbus_mv=%d\n",vbus_mv);
	return vbus_mv;
}
static uint16_t stm32l011_get_vbatADC(struct stm32l011_mcu *chip)
{

	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t vbat_mv = 0;

	ret = stm32l011_read_reg(chip,BQ25970_REG_1B,&high);
	ret = stm32l011_read_reg(chip,BQ25970_REG_1C,&low);
	vbat_mv = (high & BQ25970_VBAT_ADC_MASK) << 8 | low;
	//pr_info("vbat_mv=%d\n",vbat_mv);
	return vbat_mv;
}
static uint16_t stm32l011_get_cable_mohm(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t cable_mohm = 0;

	ret = stm32l011_read_reg(chip,MCU_CABLEROM,&low);
	ret = stm32l011_read_reg(chip,MCU_CABLEROM + 1,&high);
	cable_mohm = high << 8 | low;
	//pr_info("cable_mohm=%d\n",cable_mohm);
	return cable_mohm;

}
static uint16_t stm32l011_get_reqIbus(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t req_ibus = 0;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x7,&high);
	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x8,&low);
	req_ibus = high << 8 | low;

	return req_ibus;

}
static uint16_t stm32l011_get_req_vbus(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t req_vbus = 0;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x24,&high);
	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x23,&low);
	req_vbus = high << 8 | low;

	return req_vbus;

}
static uint16_t stm32l011_get_ldo_count(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t ldo_count = 0;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x9,&ldo_count);

	return ldo_count;
}
static int stm32l011_get_handshakeid(struct stm32l011_mcu *chip)
{
	uint8_t ret;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x0D,&chip->handshakeid[0]);
	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x0E,&chip->handshakeid[1]);
	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x0F,&chip->handshakeid[2]);
	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0x10,&chip->handshakeid[3]);

	return ret;
}
/*
   static uint16_t stm32l011_get_ptmValue(struct stm32l011_mcu *chip)
   {
   uint8_t ret;
   uint8_t ptmValue = 0;

   ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0xA,&ptmValue);

   return ptmValue;
   }*/
static uint16_t stm32l011_get_adjust_stop(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t adjust_stop;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0xB,&adjust_stop);

	return adjust_stop;
}
static uint16_t stm32l011_get_is_ldo(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t is_ldo;

	ret = stm32l011_read_reg(chip,MCU_BQ25970_REGS_START+0xC,&is_ldo);

	return is_ldo;
}

#define BAT_TEMP_NO_BATTERY (-400)
#define DEFAULT_BOARD_TEMP 250
#define NTCV_STEP			2346
static int stm32l011_get_bat_tempADC(struct stm32l011_mcu *chip)
{
	int ret;
	uint8_t low = 0,high = 0;
	uint16_t vbat_mv = 0;
	int temp = 0;
	int raw_adc = 0;
	static int pre_raw_adc;
	union power_supply_propval bat_board_temp = {0,};

	if (chip->bat_board_temp_enable ==  NTC_NONSUPPORT) {
		temp = BAT_TEMP_NO_BATTERY;
		pr_info("bat board temp nonsupport,use default 250\n");
	} else if (chip->bat_board_temp_enable == NTC_FROM_EX_FG) {
		if (get_atboot()) {
			temp = DEFAULT_BOARD_TEMP;
			return temp;
		}
		ret = power_supply_lite_get_property(PSYL_BATTERY, POWER_SUPPLY_PROP_TEMP, &bat_board_temp);
		if (ret < 0) {
			pr_info("Couldn't get bat board temp fail rc = %d, use default value\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = bat_board_temp.intval;
		}
	} else if (chip->bat_board_temp_enable == NTC_FROM_BQ25970) {
		if (chip->dchg_version == NXP_PCA9486_DCHG) {
			ret = stm32l011_read_reg(chip, BQ25970_REG_21, &high);
			ret = stm32l011_read_reg(chip, BQ25970_REG_22, &low);
			vbat_mv = high << 8 | low;
			pr_info("PAC9468 bat_temp not used,ibus_pca=%d\n", vbat_mv);
			temp = BAT_TEMP_NO_BATTERY;
		} else if (chip->dchg_version == LN8000_DCHG) {
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x10, &low);
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x11, &high);

			raw_adc = ((low & 0xF0) >> 4) | ((high & 0x3F) << 4);
			if (raw_adc) {
				pre_raw_adc = raw_adc;
			} else {
				/*pca9468 adc have not done,the raw_adc value sometimes read 0*/
				raw_adc = pre_raw_adc;
			}
			vbat_mv = raw_adc * NTCV_STEP / 1000;
			temp = stm32l011_temp_from_voltage(chip, vbat_mv, MCU_TEMP_BAT);
			pr_info("LN8000 bat_temp vbat_mv=%d\n", vbat_mv);
		} else {
			ret = stm32l011_read_reg(chip, BQ25970_REG_21, &high);
			ret = stm32l011_read_reg(chip, BQ25970_REG_22, &low);
			vbat_mv = (high & BQ25970_TBAT_ADC_MASK) << 8 | low;
			temp = stm32l011_temp_from_voltage(chip, vbat_mv, MCU_TEMP_BAT);
		}
	} else {/*maybe last one from pmi*/
		ret = stm32l011_adc_read(chip, chip->iio.bat_board_temp_chan, &temp);
		if (ret < 0) {
			pr_err("bat_board_temp_chan read fail, rc=%d\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = temp / 100;
		}
	}

	/*Backup NTC value When Dchg Start.*/
	if (chip->bat_temp_backup == 0 && is_between(200, 450, temp)) {
		chip->bat_temp_backup = temp;
	}
	chip->bat_temp_mv = temp;

	return temp;
}
static int stm32l011_get_bat_conn_temp(struct stm32l011_mcu *chip)
{
	int ret;
	uint8_t low = 0,high = 0;
	uint16_t bat_conn_temp = 0;
	int temp = 0;

	if (chip->bat_conn_temp_enable ==  NTC_NONSUPPORT) {
		temp = DEFAULT_BOARD_TEMP;
		pr_info("bat conn temp nonsupport,use default 250\n");
	} else if (chip->bat_conn_temp_enable ==  NTC_FROM_BQ25970) {
		ret = stm32l011_read_reg(chip, MCU_BAT_CONN_TMEP, &low);
		ret = stm32l011_read_reg(chip, MCU_BAT_CONN_TMEP + 1, &high);
		bat_conn_temp = high << 8 | low;
		temp = stm32l011_temp_from_voltage(chip, bat_conn_temp, MCU_TEMP_BAT_BTB);
	} else {/*maybe last one from pmi*/
		ret = stm32l011_adc_read(chip, chip->iio.bat_conn_temp_chan, &temp);
		if (ret < 0) {
			pr_err("bat_conn_temp_chan read fail, rc=%d\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = temp / 100;
		}
	}

	/*Backup NTC value When Dchg Start.*/
	if (chip->bat_conn_temp_backup == 0 && is_between(200, 450, temp)) {
		chip->bat_conn_temp_backup = temp;
	}

	return temp;
}
static int stm32l011_get_master_bat_conn_temp(struct stm32l011_mcu *chip)
{
	int ret;
#if 0
	uint8_t low = 0,high = 0;
	uint16_t master_bat_conn_temp = 0;
#endif
	int temp = 0;

	if (chip->master_bat_conn_temp_enable ==  NTC_NONSUPPORT) {
		/*single bq25970 no master bat conn temp, use default*/
		temp = -40;
		pr_info("master bat conn temp nonsupport,use default 250\n");
	} else if (chip->master_bat_conn_temp_enable ==  NTC_FROM_BQ25970) {
#if 0
		ret = stm32l011_read_reg(chip,MCU_PCB_CONN_TMEP,&low);
		ret = stm32l011_read_reg(chip,MCU_PCB_CONN_TMEP + 1,&high);
		master_bat_conn_temp = high << 8 | low;
		temp = stm32l011_temp_from_voltage(chip,master_bat_conn_temp,MCU_TEMP_PCB_BTB);
#endif
	} else {/*maybe last one from pmi*/
		ret = stm32l011_adc_read(chip, chip->iio.master_bat_conn_temp_chan, &temp);
		if (ret < 0) {
			pr_err("master_bat_conn_temp_chan read fail, rc=%d\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = temp / 100;
		}
	}

	/*Backup NTC value When Dchg Start.*/
	if (chip->master_bat_conn_temp_backup == 0 && is_between(200, 450, temp)) {
		chip->master_bat_conn_temp_backup = temp;
	}

	return temp;
}

#define PCA9468_BIT_ADC_NTCV9_4			0x3F
#define PCA9468_BIT_ADC_NTCV3_0			0xF0
static int stm32l011_get_usb_conn_temp(struct stm32l011_mcu *chip)
{
	int ret;
	uint8_t low = 0,high = 0;
	uint16_t usb_conn_mv = 0;
	int raw_adc = 0;
	int temp = 0;
	static int pre_raw_adc;
	union power_supply_propval usb_conn_temp = {0,};

	if (chip->usb_conn_temp_enable == NTC_NONSUPPORT) {
		temp = DEFAULT_BOARD_TEMP;
		pr_info("usb conn temp nonsupport,use default 250\n");
	} else if (chip->usb_conn_temp_enable == NTC_FROM_EX_FG) {
		ret = power_supply_lite_get_property(PSYL_BATTERY, POWER_SUPPLY_PROP_USB_CONN_TEMP, &usb_conn_temp);
		if (ret < 0) {
			pr_info("Couldn't get usb conn temp fail rc = %d, use default value\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = usb_conn_temp.intval;
		}
	} else if (chip->usb_conn_temp_enable == NTC_FROM_BQ25970) {
		if (chip->dchg_version == NXP_PCA9486_DCHG) {
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x0F, &high);
			ret |= stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x10, &low);
			raw_adc = ((low & PCA9468_BIT_ADC_NTCV9_4)<<4) | ((high & PCA9468_BIT_ADC_NTCV3_0)>>4);
			if (raw_adc) {
				pre_raw_adc = raw_adc;
			} else {
				/*pca9468 adc have not done,the raw_adc value sometimes read 0*/
				raw_adc = pre_raw_adc;
			}
			usb_conn_mv = raw_adc * NTCV_STEP / 1000;

			temp = stm32l011_temp_from_voltage(chip, usb_conn_mv, MCU_TEMP_USB_CONN);
		} else if (chip->dchg_version == LN8000_DCHG) {
			ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x11, &low);
			ret |= stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x12, &high);
			raw_adc = ((low & 0xC0) >> 6) | ((high & 0xFF) << 2);
			if (raw_adc) {
				pre_raw_adc = raw_adc;
			} else {
				/*pca9468 adc have not done,the raw_adc value sometimes read 0*/
				raw_adc = pre_raw_adc;
			}
			usb_conn_mv = raw_adc * NTCV_STEP / 1000;
			temp = stm32l011_temp_from_voltage(chip, usb_conn_mv, MCU_TEMP_USB_CONN);
		} else {
			ret = stm32l011_read_reg(chip, BQ25970_REG_1F, &high);
			ret = stm32l011_read_reg(chip, BQ25970_REG_20, &low);
			usb_conn_mv = (high & BQ25970_TBUS_ADC_MASK) << 8 | low;

			temp = stm32l011_temp_from_voltage(chip, usb_conn_mv, MCU_TEMP_USB_CONN);
		}
	} else {/*maybe last one from pmi*/
		ret = stm32l011_adc_read(chip, chip->iio.usb_conn_temp_chan, &temp);
		if (ret < 0) {
			pr_err("usb_conn_temp_chan read fail, rc=%d\n", ret);
			temp = DEFAULT_BOARD_TEMP;
		} else {
			temp = temp / 100;
		}
	}

	/*Backup NTC value When Dchg Start.*/
	if (chip->usb_conn_temp_backup == 0 && is_between(200, 450, temp)) {
		chip->usb_conn_temp_backup = temp;
	}

	return temp;
}

static int stm32l011_get_adapter_temp(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t adapter_temp = 0;
	int temp = 0;

	ret = stm32l011_read_reg(chip,MCU_ADAPTER_TEMP,&low);
	ret = stm32l011_read_reg(chip,MCU_ADAPTER_TEMP + 1,&high);
	adapter_temp = high << 8 | low;
	temp = adapter_temp * 10;
	return temp;

	if (VENDOR_VIVO == chip->adapter_vendor)
		temp = stm32l011_temp_from_voltage(chip,adapter_temp,MCU_TEMP_ADAPTER);
	else
		temp = adapter_temp * 10;

	return temp;
}
static int stm32l011_get_adapter_conn_temp(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t adapter_conn_temp = 0;
	int temp = 0;

	ret = stm32l011_read_reg(chip,MCU_ADAPTER_CONN_TEMP,&low);
	ret = stm32l011_read_reg(chip,MCU_ADAPTER_CONN_TEMP + 1,&high);
	adapter_conn_temp = high << 8 | low;
	temp = adapter_conn_temp * 10;
	return temp;

	if (VENDOR_VIVO == chip->adapter_vendor)
		temp = stm32l011_temp_from_voltage(chip,adapter_conn_temp,MCU_TEMP_ADAPTER_CONN);
	else
		temp = adapter_conn_temp * 10;

	return temp;
}
#define DIETEMP_STEP  	435		// 0.435C LSB, Range(-25C ~ 160C)
#define DIETEMP_DENOM	1000	// 1000, denominator
#define DIETEMP_MIN 	-25  	// -25C
#define DIETEMP_MAX		160		// 160C
static int stm32l011_get_bq_die_temp(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	int16_t reg_val = 0;
	int bq_die_temp = 0;
	int rawadc = 0;

	if (chip->dchg_version == NXP_PCA9486_DCHG) {
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x0E, &high);
		ret |= stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x0F, &low);

		if (ret != 0 || chip->analog_i2c_read_exception > 0 || chip->analog_i2c_write_exception > 0)
			bq_die_temp = 350;	/*ap+mcu or mcu+bq iic error, retrun 35degree*/
		else {
			/* Convert ADC, Temp = (935-rawadc)*0.435, unit - C*/
			rawadc = ((low & 0x0F)<<6) | ((high & 0xFC)>>2);
			rawadc = (935 - rawadc) * DIETEMP_STEP / DIETEMP_DENOM;
			if (rawadc > DIETEMP_MAX) {
				rawadc = DIETEMP_MAX;
			} else if (rawadc < DIETEMP_MIN) {
				rawadc = DIETEMP_MIN;
			}
			bq_die_temp = rawadc;
		}
	} else if (chip->dchg_version == LN8000_DCHG) {
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x0F, &low);
		ret |= stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START + 0x10, &high);
		if (ret != 0 || chip->analog_i2c_read_exception > 0 || chip->analog_i2c_write_exception > 0) {
			bq_die_temp = 350;/*ap+mcu or mcu+bq iic error, retrun 35degree*/
		} else {
			 rawadc = ((high & 0x0F) << 6) | ((low & 0xFC) >> 2);
			 rawadc = ((935 - rawadc) * 10) / 23;
			 if (rawadc > DIETEMP_MAX) {
				rawadc = DIETEMP_MAX;
			 } else if (rawadc < DIETEMP_MIN) {
				rawadc = DIETEMP_MIN;
			 }
			 bq_die_temp = rawadc;
		}
	} else {
		ret = stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x26, &high);
		ret |= stm32l011_read_reg(chip, MCU_BQ25970_REGS2_START+0x27, &low);

		if (ret != 0 || chip->analog_i2c_read_exception > 0 || chip->analog_i2c_write_exception > 0)
			bq_die_temp = 350;	/*ap+mcu or mcu+bq iic error, retrun 35degree*/
		else {
			reg_val = high << 8 | low;
			if (reg_val > 0)
				reg_val &= 0x1FF;
			bq_die_temp = 50 + (reg_val) * 5;
		}
	}
	return bq_die_temp;
}

static uint16_t stm32l011_get_dp_mv(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t dp_mv = 0;

	ret = stm32l011_read_reg(chip,MCU_DP_MV,&low);
	ret = stm32l011_read_reg(chip,MCU_DP_MV + 1,&high);
	dp_mv = high << 8 | low;
	dp_mv = dp_mv * 1047/47;

	if (dp_mv > 3800)
		chip->dp_dm_ovp = true;
	return dp_mv;
}
static uint16_t stm32l011_get_dm_mv(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t dm_mv = 0;

	ret = stm32l011_read_reg(chip,MCU_DM_MV,&low);
	ret = stm32l011_read_reg(chip,MCU_DM_MV + 1,&high);
	dm_mv = high << 8 | low;
	dm_mv = dm_mv * 1047/47;

	if (dm_mv > 3800)
		chip->dp_dm_ovp = true;
	return dm_mv;
}
/*
   static bool stm32l011_get_cable_matched(struct stm32l011_mcu *chip)
   {
   uint8_t ret;
   uint8_t low = 0;
   uint8_t cable_matched = 0;

   return 0;
   ret = stm32l011_read_reg(chip,MCU_CABLE_MATCHED,&low);
//ret = stm32l011_read_reg(chip,MCU_CABLE_MATCHED + 1,&high);
//cable_matched = high << 8 | low;
cable_matched = low;

return cable_matched;
}*/
static uint16_t stm32l011_get_mcu_exception_count(struct stm32l011_mcu *chip,uint8_t reg)
{
	uint8_t ret;
	uint8_t low = 0,high = 0;
	uint16_t exp_count = 0;
	uint16_t static last_exp_count = 0;

	ret = stm32l011_read_reg(chip,reg,&low);
	ret = stm32l011_read_reg(chip,reg + 1,&high);
	exp_count = high << 8 | low;
	if (exp_count > last_exp_count) {
		last_exp_count = exp_count;
	}
	return exp_count;
	//return last_exp_count;
}

static uint32_t stm32l011_get_adapter_powerderate(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t first = 0,second= 0,third = 0,fourth= 0;
	uint32_t status = 0;

	ret = stm32l011_read_reg(chip,MCU_ADAPTER_POWERDERATE,&first);
	ret = stm32l011_read_reg(chip,MCU_ADAPTER_POWERDERATE + 1,&second);
	ret = stm32l011_read_reg(chip,MCU_ADAPTER_POWERDERATE + 2,&third);
	ret = stm32l011_read_reg(chip,MCU_ADAPTER_POWERDERATE + 3,&fourth);
	status = first| second<<8|third<<16|fourth<<24;

	if(status == 0xffffffff){
		pr_err("adapter power derate involid, set default 0\n");
		status = 0;
	}

	return status;
}

static uint32_t stm32l011_get_adapter_status(struct stm32l011_mcu *chip)
{
	uint8_t ret;
	uint8_t first = 0,second= 0,third = 0,fourth= 0;
	uint32_t status = 0;

	ret = stm32l011_read_reg(chip,ADAPTER_STATUS,&first);
	ret = stm32l011_read_reg(chip,ADAPTER_STATUS + 1,&second);
	ret = stm32l011_read_reg(chip,ADAPTER_STATUS + 2,&third);
	ret = stm32l011_read_reg(chip,ADAPTER_STATUS + 3,&fourth);
	status = first| second<<8|third<<16|fourth<<24;


	return status;
}

static uint8_t stm32l011_chg_is_enable(struct stm32l011_mcu *chip)
{

	uint8_t ret = 0, rc = 0;
	uint8_t value = 0;
	bool need_rerun = false;
	ret = stm32l011_read_reg(chip,BQ25970_REG_06,&value);
	ret  = value & BQ25970_CHGEN_MASK;

	if (ret > 0 || chip->handshake_success) {
		if (chip->dchg_start_time == 0) {
			chip->dchg_start_time = jiffies;
			pr_err("direct charging enable, dchg_start_time = %ld.\n", chip->dchg_start_time/HZ);
		}
	}

	if (!get_atboot() && (!chip->chg_enable || chip->user_vote_lose) && ret > 0) {
		pr_err("direct charging enable, initialize current to %d mA.\n", chip->user_dchg_current);

		if (get_effective_result_locked(chip->dchg_fcc_votable) == chip->user_dchg_current)
			need_rerun = true;

		rc = vote(chip->dchg_fcc_votable, USER_FCC_VOTER, true,
				chip->user_dchg_current);
		if (rc < 0)
			pr_err("Couldn't vote fastchg ma rc = %d\n", rc);

		if (need_rerun) {
			pr_err("rerun_election dchg_fcc_votable...\n");
			rerun_election(chip->dchg_fcc_votable);
		}

		if (chip->user_vote_lose)
			chip->user_vote_lose = false;
	}

	return !!ret;
}
static uint8_t stm32l011_slave_chg_is_enable(struct stm32l011_mcu *chip)
{
	uint8_t ret = 0;
	uint8_t value = 0;

	ret = stm32l011_read_reg(chip,BQ25970_REG_2D,&value);
	//	pr_info("ret=%d,value=%d\n", ret, value);
	ret  = value & BQ25970_CHGEN_MASK;
	return !!ret;
}

#ifdef CONFIG_SPECIAL_PROP_FOR_PD2083
extern char *get_board_version(void);
#endif

static int stm32l011_usbsel_enable(struct stm32l011_mcu *chip, bool enable)
{
	int ret = 0;
/*Board version rule: when IO value is low, usbsel enable; otherwise, usbsel disable */
#ifdef CONFIG_SPECIAL_PROP_FOR_PD2083
	char *version_state = NULL;
	version_state = get_board_version();
	pr_info("board version = %s\n", version_state);
	if (chip->usbsel_gpio > 0 && (version_state[5] == '0')) {
#else
	if (chip->usbsel_gpio > 0) {
#endif
		if (enable)
			gpio_direction_output(chip->usbsel_gpio, 1);
		else
			gpio_direction_output(chip->usbsel_gpio, 0);

		chip->usbsel_state = gpio_get_value(chip->usbsel_gpio);
		pr_info("usbsel gpio, enable=%d,usbsel_state=%d\n", enable, chip->usbsel_state);
	} else {
		if (enable)
			ret = max20328_switch_mode_event(MAX_USBC_FAST_CHARGE_SELECT);
		else
			ret = max20328_switch_mode_event(MAX_USBC_FAST_CHARGE_EXIT);

		if (ret) {
			pr_err("switch_mode set usbsel_state=%d failed!", enable);
		} else {
			chip->usbsel_state = enable;
		}
		pr_info("not usbsel gpio, enable=%d,usbsel_state=%d\n", enable, chip->usbsel_state);
	}
	return 0;
}

#ifdef TEST_BY_FXI_FOR_BURN
static int stm32l011_i2c_read(struct stm32l011_mcu *chip, u8 *buf, int len)
{
	int ret = -1;
	int retry = 0;
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msgs[2];
	bool enable;

	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	msgs[0].flags   = !I2C_M_RD;
	msgs[0].addr    = client->addr;
	msgs[0].len     = 1;
	msgs[0].buf     = &buf[0];

	msgs[1].flags   = I2C_M_RD;
	msgs[1].addr    = client->addr;
	msgs[1].len     = len - 1;
	msgs[1].buf     = &buf[1];

	while (retry++ < TRANSFER_RETRY_TIME) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		else {
			if (!chip->update_done && chip->burn_handling)
				chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__I2C_ERROR;
			pr_info("failed: ret=%d\n", ret);
		}
	}

	return ret;
}

static int stm32l011_i2c_write(struct stm32l011_mcu *chip, u8 *buf, int len)
{
	int ret = -1;
	int retry = 0;
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msg;
	bool enable;

	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	msg.flags   = !I2C_M_RD;
	msg.addr    = client->addr;
	msg.len     = len;
	msg.buf     = buf;

	while (retry++ < TRANSFER_RETRY_TIME) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		else {
			if (!chip->update_done && chip->burn_handling)
				chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__I2C_ERROR;
			pr_info("failed: ret=%d\n", ret);
		}
	}

	return ret;
}

static u8 *get_erase_cmd_param(struct stm32l011_mcu *chip, u32 shift, u16 page)
{
	mutex_lock(&chip->cmd_lock);
	memset(chip->erase_cmd,0,6);
	chip->erase_cmd[0] = shift & 0xFF;
	chip->erase_cmd[1] = (shift>>8) & 0xFF;
	chip->erase_cmd[2] = (shift>>16) & 0xFF;
	chip->erase_cmd[3] = (shift>>24) & 0xFF;
	chip->erase_cmd[4] = page & 0xFF;
	chip->erase_cmd[5] = (page>>8) & 0xFF;
	mutex_unlock(&chip->cmd_lock);
	return chip->erase_cmd;
}

static u8 *get_crc_cmd_param(struct stm32l011_mcu *chip, u32 crc, u32 shift, u32 size)
{
	mutex_lock(&chip->cmd_lock);
	memset(chip->crc_cmd,0,12);
	chip->crc_cmd[0] = crc & 0xFF;
	chip->crc_cmd[1] = (crc>>8) & 0xFF;
	chip->crc_cmd[2] = (crc>>16) & 0xFF;
	chip->crc_cmd[3] = (crc>>24) & 0xFF;
	chip->crc_cmd[4] = shift & 0xFF;
	chip->crc_cmd[5] = (shift>>8) & 0xFF;
	chip->crc_cmd[6] = (shift>>16) & 0xFF;
	chip->crc_cmd[7] = (shift>>24) & 0xFF;
	chip->crc_cmd[8] = size & 0xFF;
	chip->crc_cmd[9] = (size>>8) & 0xFF;
	chip->crc_cmd[10] = (size>>16) & 0xFF;
	chip->crc_cmd[11] = (size>>24) & 0xFF;
	mutex_unlock(&chip->cmd_lock);
	return chip->crc_cmd;
}

static void init_burn_wrap_command(struct burn_wrap *b_wrap, BURN_CMD cmd, u8 *parameter, u16 size)
{
	u8 i, sum;

	b_wrap->head.reg = I2C_BURN_REG;

	if (BURN_CMD_APP_VERSION == cmd || BURN_CMD_APP_UPDATE == cmd)
		b_wrap->head.start = PROTOCOL_APP_START;
	else
		b_wrap->head.start = PROTOCOL_BOOT_START;

	b_wrap->head.length = COMMAND_WIDTH + size;
	b_wrap->command = cmd;
	b_wrap->data = parameter;

	sum = b_wrap->head.length + b_wrap->command;
	for (i=0; i<size; i++)
		sum += *(parameter + i);

	b_wrap->chksum = (u8)sum;
}

static void init_burn_wrap_write(struct stm32l011_mcu *chip, struct burn_wrap *b_wrap, u8 *data, u32 offset, u16 batch)
{
	u8 i, sum, remain, compensate;

	b_wrap->head.reg = I2C_BURN_REG;
	b_wrap->head.start = PROTOCOL_BOOT_START;
	b_wrap->head.length = COMMAND_WIDTH + FLASH_ADDR_WIDTH + FLASH_BATCH_WIDTH + batch;
	b_wrap->command = BURN_CMD_WRITE;
	b_wrap->shift = offset;
	b_wrap->batch = batch & 0xFF;
	b_wrap->data = data;

	sum = b_wrap->head.length + b_wrap->command;

	sum += (offset & 0xFF) + ((offset>>8) & 0xFF) +
		((offset>>16) & 0xFF) + ((offset>>24) & 0xFF);

	sum += (b_wrap->batch & 0xFF);
	for (i=0; i<batch; i++)
		sum += *(data + i);

	remain = batch % BATCH_ALIGN;
	if (0 != remain) {
		pr_info("size need align compensate\n");
		compensate = BATCH_ALIGN - remain;
		b_wrap->batch += compensate;
		b_wrap->head.length += compensate;
		for (i=0; i<compensate; i++)
			sum += 0xFF;
	}

	b_wrap->chksum = (u8)sum;
}

static u8 get_reply_wrap_size(struct stm32l011_mcu *chip)
{
	int ret;
	u8 size = 0, retry;
	u8 buf[I2C_BURN_HEAD_SIZE];

	for (retry=0; retry<TRANSFER_RETRY_TIME; retry++) {
		buf[0] = I2C_BURN_REG;
		ret = stm32l011_i2c_read(chip, buf, I2C_BURN_HEAD_SIZE);
		if (ret <= 0) {
			pr_info("read reply head data i2c error\n");
			continue;
		}
		if ((PROTOCOL_BOOT_START == buf[1] || PROTOCOL_APP_START == buf[1])
				&& buf[2] >= REPLY_DEF_LEN) {
			size = I2C_BURN_HEAD_SIZE + buf[2] + CHKSUM_WIDTH;
			pr_info("read reply size=%d\n", size);
			break;
		}
		if (retry >= TRANSFER_RETRY_TIME) {
			pr_info("read reply head data time out, exit\n");
			size = -EAGAIN;
			break;
		}
		pr_info("0x%02x,0x%02x,0x%02x\n", buf[0], buf[1], buf[2]);
	}

	pr_info("size=%d\n", size);
	return size;
}

static void convert_to_reply_wrap(struct reply_wrap *r_wrap, u8 *buf)
{
	r_wrap->head.reg = buf[0];
	r_wrap->head.start = buf[1];
	r_wrap->head.length = buf[2];
	r_wrap->reply = buf[3];
	r_wrap->command = buf[4];
	r_wrap->status = buf[5];
	if (r_wrap->head.length > REPLY_DEF_LEN) {
		r_wrap->data = &buf[6];
		r_wrap->chksum = buf[I2C_BURN_HEAD_SIZE+r_wrap->head.length];
	} else {
		r_wrap->data = NULL;
		r_wrap->chksum = buf[6];
	}
}

static void dump_burn_wrap(struct stm32l011_mcu *chip, struct burn_wrap *b_wrap)
{
	u8 i;
	u16 data_size = b_wrap->head.length - COMMAND_WIDTH;

	pr_info("write_wrap:0x%02x,0x%02x,0x%02x,",
			b_wrap->head.start, b_wrap->head.length, b_wrap->command);

	if (BURN_CMD_WRITE == b_wrap->command) {
		data_size -= (FLASH_ADDR_WIDTH + FLASH_BATCH_WIDTH);
		pr_info("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,",
				(b_wrap->shift & 0xFF), ((b_wrap->shift>>8) & 0xFF),
				((b_wrap->shift>>16) & 0xFF), ((b_wrap->shift>>24) & 0xFF),
				(b_wrap->batch & 0xFF));
	}

	if (b_wrap->data) {
		for (i=0; i<data_size; i++)
			pr_info("0x%02x%c", *(b_wrap->data+i), (((i+1)%16 == 0)?'\n':','));
	}

	pr_info("0x%02x\n", b_wrap->chksum);
}

#if 0
static void dump_origin_data(u8 *buf, u32 len)
{
	u32 i = 0;
	for (i=1; i<len; i++)
		pr_info("0x%02x%c", buf[i], ((i==(len-1))?'\n':','));
}
#endif

static void dump_reply_wrap(struct stm32l011_mcu *chip, struct reply_wrap *r_wrap)
{
	u8 i;
	u16 data_size = r_wrap->head.length - REPLY_DEF_LEN;

	pr_info("read_wrap:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,",
			r_wrap->head.start, r_wrap->head.length, r_wrap->reply,
			r_wrap->command, r_wrap->status);

	if (r_wrap->data) {
		for (i=0; i<data_size; i++)
			pr_info("0x%02x%c", *(r_wrap->data+i), (((i+1)%16 == 0)?'\n':','));
	}

	pr_info("0x%02x\n", r_wrap->chksum);
}

static bool check_reply_wrap(struct stm32l011_mcu *chip, struct reply_wrap *r_wrap, BURN_CMD cmd, u16 size)
{
	bool ret = false;
	u8 i, sum;

	if (PROTOCOL_BOOT_START != r_wrap->head.start
			&& PROTOCOL_APP_START != r_wrap->head.start) {
		pr_info("reply_wrap start byte error(0x%02x)\n", r_wrap->head.start);
		goto exit;
	}

	if ((BURN_CMD_READ == cmd && (size+REPLY_DEF_LEN) != r_wrap->head.length)
			|| (BURN_CMD_READ != cmd && REPLY_DEF_LEN != r_wrap->head.length)) {
		pr_info("reply_wrap length byte error(0x%02x)\n", r_wrap->head.length);
		goto exit;
	}

	if (BURN_CMD_REPLY != r_wrap->reply) {
		pr_info("reply_wrap reply byte error(0x%02x)\n", r_wrap->reply);
		goto exit;
	}

	if (cmd != r_wrap->command) {
		pr_info("reply_wrap command byte error(0x%02x)\n", r_wrap->command);
		goto exit;
	}

	if (REPLY_STATUS_OK != r_wrap->status) {
		pr_info("reply_wrap status byte error(0x%02x)\n", r_wrap->status);
		goto exit;
	}

	sum = r_wrap->head.length + r_wrap->reply + r_wrap->command;
	for (i=0; i<(r_wrap->head.length-REPLY_DEF_LEN); i++)
		sum += *(r_wrap->data + i);

	if ((u8)sum != r_wrap->chksum) {
		pr_info("reply_wrap chksum byte error(0x%02x)\n", r_wrap->chksum);
		goto exit;
	}

	ret = true;
exit:
	return ret;
}

static int stm32l011_i2c_burn_command(struct stm32l011_mcu *chip, BURN_CMD cmd,
		u8 *parameter, u16 size)
{
	int ret = -1;
	u16 total_burn, total_reply;
	u8 retry, reply_size, data_size;
	u8 *w_buf = NULL;//u8 w_buf[TRANSFER_BATCH_SIZE];
	u8 *r_buf = NULL;//u8 r_buf[TRANSFER_BATCH_SIZE];
	struct burn_wrap b_wrap;
	struct reply_wrap r_wrap;

	w_buf = kzalloc(sizeof(u8) * TRANSFER_BATCH_SIZE, GFP_ATOMIC  | GFP_DMA);
	r_buf = kzalloc(sizeof(u8) * TRANSFER_BATCH_SIZE, GFP_ATOMIC  | GFP_DMA);
	memset(&b_wrap, 0x0, sizeof(struct burn_wrap));
	memset(&r_wrap, 0x0, sizeof(struct reply_wrap));
	reply_size = I2C_BURN_HEAD_SIZE + REPLY_DEF_LEN + CHKSUM_WIDTH;

	init_burn_wrap_command(&b_wrap, cmd, parameter, size);
	w_buf[0] = r_buf[0] = b_wrap.head.reg;
	w_buf[1] = b_wrap.head.start;
	w_buf[2] = b_wrap.head.length;
	w_buf[3] = b_wrap.command;
	data_size = (b_wrap.head.length-COMMAND_WIDTH);
	if (data_size > 0)
		memcpy(&w_buf[4], b_wrap.data, data_size);

	w_buf[4+data_size] = b_wrap.chksum;
	total_burn = (I2C_BURN_HEAD_SIZE + COMMAND_WIDTH + data_size + CHKSUM_WIDTH);

	pr_info("burn_cmd=0x%02x, total=%d\n", cmd, total_burn);
	for (retry=0; retry<TRANSFER_RETRY_TIME; retry++) {
		ret = stm32l011_i2c_write(chip, w_buf, total_burn);
		if (ret <= 0) {
			pr_info("send command i2c error\n");
			continue;
		}

		dump_burn_wrap(chip, &b_wrap);
		msleep(500);
		total_reply = get_reply_wrap_size(chip);
		if (total_reply < reply_size) {
			msleep(500);
			total_reply = get_reply_wrap_size(chip);
			if (total_reply < reply_size) {
				pr_info("read reply wrap size error\n");
				continue;
			}
		}

		ret = stm32l011_i2c_read(chip, r_buf, total_reply);
		if (ret <= 0) {
			pr_info("fetch reply i2c error\n");
			continue;
		}
		convert_to_reply_wrap(&r_wrap, r_buf);
		dump_reply_wrap(chip, &r_wrap);
		if (BURN_CMD_BOOT_VERSION == cmd) {
			//lierda_Bootloader: 0x55,0x04,0x08,0x0c,0x00,0x21,0x39
			//vivo_Bootloader: 0x55,0x03,0x08,0x0c,0x31,0x48
			if (r_buf[3] == 0x08 && r_buf[4] == 0x0C) {
				if (r_buf[2] == 0x04 && r_buf[5] == 0x00 && r_buf[6] == 0x21 && r_buf[7] == 0x39)
					chip->mcu_bl_version = lierda_MCU_Bootloader_Version;
				else if (r_buf[2] == 0x03 && r_buf[5] == 0x31 && r_buf[6] == 0x48)
					chip->mcu_bl_version = vivo_MCU_Bootloader_Version;
			}
			//pr_info("check MCU BL version=0x%x, MCU BL vendor=0x%x\n", chip->mcu_bl_version, chip->mcu_bl_vendor);
			break;
		} else if (!check_reply_wrap(chip, &r_wrap, cmd, size)) {
			pr_info("check reply wrap fail\n");
			continue;
		} else {
			if (r_wrap.data != NULL)
				ret = r_buf[total_reply - 2];
			break;
		}
	}

	if (retry >= TRANSFER_RETRY_TIME) {
		pr_info("burn command time out, exit\n");
		ret = -EAGAIN;
	}

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

static int stm32l011_i2c_burn_command_noparam(struct stm32l011_mcu *chip, BURN_CMD cmd)
{
	return stm32l011_i2c_burn_command(chip, cmd, NULL, CMD_NO_PARAM);
}

static u32 stm32l011_i2c_burn_write(struct stm32l011_mcu *chip, bool new_crc,
		u8 *data, u32 offset, u32 size, u32 format)
{
	int ret = 0;
	u32 crc = 0xFFFFFFFF;
	u32 piece = 0, piece_align_page = 0, flash_offset;
	u16 reply_size, one_batch, batch_format, total_burn, total_reply;
	u8 retry = 0, align_retry = 0;
	u8 *w_buf = NULL;//u8 w_buf[TRANSFER_BATCH_SIZE];
	u8 *r_buf = NULL;//u8 r_buf[TRANSFER_BATCH_SIZE];
	u8 *p;
	struct burn_wrap b_wrap;
	struct reply_wrap r_wrap;
	ERROR_TYPE err = NO_ERROR;

	w_buf = kzalloc(sizeof(u8) * TRANSFER_BATCH_SIZE, GFP_ATOMIC  | GFP_DMA);
	r_buf = kzalloc(sizeof(u8) * TRANSFER_BATCH_SIZE, GFP_ATOMIC  | GFP_DMA);

	memset(&b_wrap, 0x0, sizeof(struct burn_wrap));
	memset(&r_wrap, 0x0, sizeof(struct reply_wrap));

	flash_offset = offset;
	if (format > FLASH_PAGE_SIZE || format <= 0)
		batch_format = FLASH_PAGE_SIZE;
	else
		batch_format = (format / BATCH_ALIGN) * BATCH_ALIGN;

	reply_size = I2C_BURN_HEAD_SIZE + REPLY_DEF_LEN + CHKSUM_WIDTH;

	pr_info("BURN_CMD_WRITE size=%d\n", size);
	while (piece < size) {
		one_batch = ((size-piece)>batch_format) ? batch_format : (size-piece);
		if (new_crc)
			crc = get_crc32(crc, (data+piece), one_batch);

		init_burn_wrap_write(chip, &b_wrap, (data+piece), flash_offset, one_batch);
		memset(w_buf, 0xFF, TRANSFER_BATCH_SIZE);
		w_buf[0] = r_buf[0] = b_wrap.head.reg;
		w_buf[1] = b_wrap.head.start;
		w_buf[2] = b_wrap.head.length;
		w_buf[3] = b_wrap.command;
		w_buf[4] = b_wrap.shift & 0xFF;
		w_buf[5] = (b_wrap.shift>>8) & 0xFF;
		w_buf[6] = (b_wrap.shift>>16) & 0xFF;
		w_buf[7] = (b_wrap.shift>>24) & 0xFF;
		w_buf[8] = b_wrap.batch & 0xFF;
		memcpy(&w_buf[9], b_wrap.data, one_batch);
		w_buf[I2C_BURN_HEAD_SIZE + b_wrap.head.length] = b_wrap.chksum;
		total_burn = (I2C_BURN_HEAD_SIZE + b_wrap.head.length + CHKSUM_WIDTH);

		if (chip->burn_handling == false) {
			pr_info("burn_handling = false, skip it 1 ...\n");
			break;
		}

		for (retry=0; retry<TRANSFER_RETRY_TIME; retry++) {
			if (chip->burn_handling == false) {
				pr_info("burn_handling = false, skip it 2...\n");
				break;
			}

			err = NO_ERROR;
			ret = stm32l011_i2c_write(chip, w_buf, total_burn);
			if (ret <= 0) {
				pr_info("send command i2c error\n");
				err = WRITE_ERROR;
			}

			if (NO_ERROR == err) {
				if (chip->debug_fw_updating)
					msleep(100);
				else
					msleep(100);
				total_reply = get_reply_wrap_size(chip);
				if (total_reply < reply_size) {
					msleep(500);
					total_reply = get_reply_wrap_size(chip);
					if (total_reply < reply_size) {
						pr_info("read reply wrap size error\n");
						err = READ_ERROR;
					}
				}
			}

			if (NO_ERROR == err) {
				ret = stm32l011_i2c_read(chip, r_buf, total_reply);
				if (ret <= 0) {
					pr_info("fetch reply i2c error\n");
					err = READ_ERROR;
				}
			}

			if (NO_ERROR == err) {
				convert_to_reply_wrap(&r_wrap, r_buf);
				dump_reply_wrap(chip, &r_wrap);
				if (!check_reply_wrap(chip, &r_wrap, BURN_CMD_WRITE, one_batch)) {
					pr_info("check reply wrap fail\n");
					err = REPLY_ERROR;
				}
			}

			if (NO_ERROR == err) {
				piece += one_batch;
				flash_offset = offset + piece;
				break;
			} else {
				if (0 == (piece % FLASH_PAGE_SIZE)) {
					pr_info("burn write retry (0x%08x,%d)\n", flash_offset, piece);
					p = get_erase_cmd_param(chip, flash_offset, 1);
					ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
					msleep(1000);
					continue;
				} else {
					pr_info("burn write align_retry (0x%08x,%d)\n", flash_offset, piece);
					align_retry++;
					piece_align_page = (piece / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
					piece = piece_align_page;
					flash_offset = offset + piece;
					p = get_erase_cmd_param(chip, flash_offset, 1);
					ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
					msleep(1000);
					break;
				}
			}
		}

		if (retry >= TRANSFER_RETRY_TIME || align_retry >= TRANSFER_RETRY_TIME) {
			pr_info("burn write data time out, exit\n");
			ret = -EAGAIN;
			break;
		}
	}

	if (ret > 0)
		pr_info("~~~ burn write data success\n");
	else
		pr_err("burn write data fail\n");
	kfree(w_buf);
	kfree(r_buf);

	return crc;
}
#endif

static int stm32l011_power_restore_enable(struct stm32l011_mcu *chip)
{
	stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
	msleep(100);
	stm32l011_pinctrl_power_gpio_state(chip, PINCFG_HIGH);

	return 0;
}

static int stm32l011_power_enable(struct stm32l011_mcu *chip,bool enable)
{
	if (chip->power_gpio <= 0 && IS_ERR_OR_NULL(chip->power_ldo)) {
		pr_err("power_gpio not configged\n");
		return 0;
	}

	if (chip->burn_handling) {
		pr_warn("burn_handling\n");
		return 0;
	}

	if (enable) {
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_HIGH);
	} else {
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
	}
	pr_info("enable=%d,mcu_power=%d\n", enable, stm32l011_get_power_gpio_state(chip));
	return 0;
}

static int stm32l011_hw_deinit(struct stm32l011_mcu *chip)
{
	int i;

	pr_info(" deinit\n");
	mutex_lock(&chip->parameters_lock);
	chip->init_done = false;
	chip->pmi_suspend = false;
	chip->pmi_suspended = false;
	chip->exit = false;
	chip->handshake_success = false;
	chip->handshake_failed = false;
	chip->dchg_exit_status = 0;
	chip->cable_matched_event = false;
	chip->high_vol_mode_event = false;
	chip->time_count = 0;
	memset(chip->handshakeid,0,4);
	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		handlers[i].prev_val = 0;
		handlers[i].val = 0;
	}
	for (i = 0; i < chip->cable_r_limit_table_rc[ROW]; i++) {
		chip->cable_r_limit_table[i].limit_count = 0;
	}

	stm32l011_shift_en_enable(chip,false);
	stm32l011_usbsel_enable(chip,false);
	stm32l011_power_enable(chip,false);
	stm32l011_reset_voters(chip);
	chip->irqA = 0;
	chip->irqB = 0;
	chip->irqC = 0;
	chip->irqD = 0;
	chip->reg04 = 0;
	chip->reg05 = 0;
	chip->int_stat_08 = 0;
	chip->int_stat_0A = 0;
	chip->int_stat_0B = 0;
	chip->int_stat_0E = 0;
	chip->int_stat_11 = 0;
	chip->int_stat_2D = 0;
	chip->int_stat_ln8000_01 = 0;
	chip->int_stat_ln8000_03 = 0;
	chip->int_stat_ln8000_04 = 0;
	chip->int_stat_ln8000_05 = 0;
	chip->int_stat_ln8000_06 = 0;
	chip->int_stat_ln8000_08 = 0;
	chip->ibus_ma = 0;
	chip->vbus_mv = 0;
	chip->vbat_mv = 0;
	chip->cable_mohm = 0;
	chip->bat_temp = 0;
	chip->bat_temp_backup = 0;
	chip->bat_conn_temp = 0;
	chip->bat_conn_temp_backup = 0;
	chip->pcb_conn_temp = 0;
	chip->usb_conn_temp = 0;
	chip->usb_conn_temp_backup = 0;
	chip->adapter_temp = 0;
	chip->adapter_conn_temp = 0;
	chip->bq_die_temp = 0;
	chip->dp_mv = 0;
	chip->dm_mv = 0;
	chip->cable_matched = 0;
	chip->chg_enable = 0;
	chip->user_dchg_current = chip->total_dchg_limit_max_ma;
	chip->cable_r_current_limit_max = chip->total_dchg_limit_max_ma;
	chip->uart_tx_exception = 0;
	chip->uart_rx_exception = 0;
	chip->analog_i2c_read_exception = 0;
	chip->analog_i2c_write_exception  = 0;
	chip->adapter_i2c_read_exception = 0;
	chip->adapter_i2c_write_exception = 0;
	chip->adapter_uart_tx_exception = 0;
	chip->adapter_uart_rx_exception = 0;
	chip->adapter_status = 0;
	chip->adapter_powerderate = 0;
	chip->adapter_powerderate_ready = false;
	chip->ap_i2c_write_exception = 0;
	chip->ap_i2c_read_exception = 0;
	chip->adapter_power = 0;
	chip->adapter_mcu_version = 0;
	chip->adapter_vendor = VENDOR_VIVO;
	chip->dchg_enabled = 0xFF;
	chip->vbus_exception = 0;
	chip->low_temp_retries = 0;
	chip->medial_temp_retries = 0;
	chip->normal_temp_retries = 0;
	chip->high_temp_retries = 0;
	chip->setting_curr_too_low_for_dchg = false;
	chip->reg_exception = false;
	chip->reg_exception_count = 0;
	chip->reset_retries = 0;
	chip->usb_type = 0;
	scnprintf(chip->dchg_state, sizeof(chip->dchg_state), "waiting..");
	/* add for dual charging */
	chip->slave_ibus_ma = 0;
	chip->dual_chg_step = HCHG_WAIT_MASTER_CHG;
	chip->dual_chg_cur_threshold = 3800;
	chip->dchg_master_max_current = 4500;
	chip->slave_ic_enable_fail = 0;
	chip->dual_chg_to_standalone_req = false;
	chip->in_standalone_mode = false;
	chip->both_charger_fault_counter = 0;
	chip->slave_charger_fault_counter = 0;
	chip->master_slave_charger_current_miss_match = 0;
	chip->set_cur_too_low_for_dual_chg = false;
	chip->master_bat_conn_temp = 0;
	chip->master_bat_conn_temp_backup = 0;
	chip->at_mode_slave_chg_enable_req = 0;
	chip->factory_mode_state = 0;
	chip->real_vbat_mv = 0;
	chip->slave_chg_enable = 0;
	chip->batt_capacity = 0;
	chip->slave_charger_should_disable = false;
	/*dual end*/
	chip->factory_10W_charge_test_enabe = 0;
	chip->sbu1_adc = -1;
	chip->sbu2_adc = -1;
	chip->sbu_cable_id = SBU_CABLE_ID_UNKNOW;
	chip->main_chg_sm_error_in_ap_side = 0;
	chip->cable_r_disable_dchg = 0;
	//chip->dp_dm_ovp = false;	//cable disconnect,  no need reset it..
	chip->vivo_flash_charge_status = POWER_SUPPLY_ENGINE_NORMAL;
	chip->dchg_start_time = 0;
	chip->dchg_continue_time = 0;
	chip->cable_id_detect_done = 0;
	chip->sbu_detect_busy = false;
	chip->user_vote_lose = false;
	chip->disable_vote_miss = false;
	mutex_unlock(&chip->parameters_lock);
	return 0;
}
static int stm32l011_hw_init(struct stm32l011_mcu *chip)
{
	int i;

	pr_info(" init\n");
	mutex_lock(&chip->parameters_lock);
	chip->init_done = false;
	chip->pmi_suspend = false;
	chip->pmi_suspended = false;
	chip->exit = false;
	chip->handshake_success = false;
	chip->handshake_failed = false;
	chip->dchg_exit_status = 0;
	chip->cable_matched_event = false;
	chip->high_vol_mode_event = false;
	chip->time_count = 0;
	memset(chip->handshakeid,0,4);
	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		handlers[i].prev_val = 0;
		handlers[i].val = 0;
	}
	for (i = 0; i < chip->cable_r_limit_table_rc[ROW]; i++) {
		chip->cable_r_limit_table[i].limit_count = 0;
	}
	stm32l011_reset_voters(chip);

	if (stm32l011_get_power_gpio_state(chip)) {
		stm32l011_power_enable(chip, false);
		msleep(100);
	}
	stm32l011_power_enable(chip,true);
	msleep(10);
	stm32l011_shift_en_enable(chip,true);
	chip->irqA = 0;
	chip->irqB = 0;
	chip->irqC = 0;
	chip->irqD = 0;
	chip->reg04 = 0;
	chip->reg05 = 0;
	chip->int_stat_08 = 0;
	chip->int_stat_0A = 0;
	chip->int_stat_0B = 0;
	chip->int_stat_0E = 0;
	chip->int_stat_11 = 0;
	chip->int_stat_2D = 0;
	chip->int_stat_ln8000_01 = 0;
	chip->int_stat_ln8000_03 = 0;
	chip->int_stat_ln8000_04 = 0;
	chip->int_stat_ln8000_05 = 0;
	chip->int_stat_ln8000_06 = 0;
	chip->int_stat_ln8000_08 = 0;
	chip->ibus_ma = 0;
	chip->vbus_mv = 0;
	chip->vbat_mv = 0;
	chip->cable_mohm = 0;
	chip->bat_temp = 0;
	chip->bat_temp_backup = 0;
	chip->bat_conn_temp = 0;
	chip->bat_conn_temp_backup = 0;
	chip->pcb_conn_temp = 0;
	chip->usb_conn_temp = 0;
	chip->usb_conn_temp_backup = 0;
	chip->adapter_temp = 0;
	chip->adapter_conn_temp = 0;
	chip->bq_die_temp = 0;
	chip->dp_mv = 0;
	chip->dm_mv = 0;
	chip->cable_matched = 0;
	chip->chg_enable = 0;
	chip->user_dchg_current = chip->total_dchg_limit_max_ma;
	chip->cable_r_current_limit_max = chip->total_dchg_limit_max_ma;
	chip->uart_tx_exception = 0;
	chip->uart_rx_exception = 0;
	chip->analog_i2c_read_exception = 0;
	chip->analog_i2c_write_exception  = 0;
	chip->adapter_i2c_read_exception = 0;
	chip->adapter_i2c_write_exception = 0;
	chip->adapter_uart_tx_exception = 0;
	chip->adapter_uart_rx_exception = 0;
	chip->adapter_status = 0;
	chip->adapter_powerderate = 0;
	chip->adapter_powerderate_ready = false;
	chip->ap_i2c_write_exception = 0;
	chip->ap_i2c_read_exception = 0;
	chip->dchg_enabled = 0xFF;
	chip->vbus_exception = 0;
	chip->low_temp_retries = 0;
	chip->medial_temp_retries = 0;
	chip->normal_temp_retries = 0;
	chip->high_temp_retries = 0;
	chip->setting_curr_too_low_for_dchg = false;
	chip->reg_exception = false;
	chip->reg_exception_count = 0;
	chip->reset_retries = 0;
	chip->usb_type = 0;
	scnprintf(chip->dchg_state, sizeof(chip->dchg_state), "waiting..");

	/* add for dual charging */
	chip->slave_ibus_ma = 0;
	chip->dual_chg_step = HCHG_WAIT_MASTER_CHG;
	chip->dual_chg_cur_threshold = 3800;
	chip->dchg_master_max_current = 4500;
	chip->slave_ic_enable_fail = 0;
	chip->dual_chg_to_standalone_req = false;
	chip->in_standalone_mode = false;
	chip->both_charger_fault_counter = 0;
	chip->slave_charger_fault_counter = 0;
	chip->master_slave_charger_current_miss_match = 0;
	chip->set_cur_too_low_for_dual_chg = false;
	chip->master_bat_conn_temp = 0;
	chip->master_bat_conn_temp_backup = 0;
	chip->at_mode_slave_chg_enable_req = 0;
	//chip->factory_mode_state = 0;
	chip->real_vbat_mv = 0;
	chip->slave_chg_enable = 0;
	chip->batt_capacity = 0;
	chip->slave_charger_should_disable = false;
	/*dual end*/
	chip->factory_10W_charge_test_enabe = 0;
	chip->sbu1_adc = -1;
	chip->sbu2_adc = -1;
	chip->sbu_cable_id = SBU_CABLE_ID_UNKNOW;
	chip->main_chg_sm_error_in_ap_side = 0;
	chip->cable_r_disable_dchg = 0;
	if ((jiffies - chip->last_cable_disconnect_start_time)/HZ >= 1)	//1s
		chip->dp_dm_ovp = false;
	chip->vivo_flash_charge_status = POWER_SUPPLY_ENGINE_NORMAL;
	chip->dchg_start_time = 0;
	chip->dchg_continue_time = 0;
	mutex_unlock(&chip->parameters_lock);

	chip->chg_update = true;
	chip->cable_id_detect_done = 0;
	chip->sbu_detect_busy = false;
	chip->user_vote_lose = false;
	chip->disable_vote_miss = false;
	//fuelsummary_clear_data(ID_FCHG, ID_ADPT_VER);
	return 0;
}
static int stm32l011_dchg_enable(struct stm32l011_mcu *chip, bool enable)
{
	union power_supply_propval batt_soc = {0,};
	union power_supply_propval usb_type = {0,};

	if (chip->burn_handling) {
		pr_warn("burn_handling\n");
		return 0;
	}

	/*dchg enbale/disable not trigger usb plugged*/
	if (!chip->usb_present) {
		pr_warn("!usb_present\n");
		return 0;
	}

	/*dchg_enabled default value:0xFF*/
	if (chip->dchg_enabled != 0xFF && chip->dchg_enabled == enable && !chip->disable_vote_miss) {
		pr_warn("dchg status not changed,return\n");
		return 0;
	}

	power_supply_lite_get_property(PSYL_BATTERY, POWER_SUPPLY_PROP_CAPACITY, &batt_soc);
	if (((chip->irqB & EXIT_HIGH_VBAT) || batt_soc.intval >= 99) && enable) {
		pr_warn("dchg exit because in HIGH Vbat, skip dchg_enable.\n");
		return 0;
	}

	mutex_lock(&chip->dchg_enable_lock);
	if (enable) {
		//chip->dchg_enabled = true;
		power_supply_lite_get_property(PSYL_CHARGE, POWER_SUPPLY_PROP_CHARGE_TYPE, &usb_type);
		chip->usb_type = usb_type.intval;
#if OLD_CAM_NOTIFY
		if (chip->usb_type == POWER_SUPPLY_TYPE_USB_DCP && (chip->exit || chip->cam_exit_restore))
#else
		if ((chip->usb_type == POWER_SUPPLY_TYPE_USB_DCP || chip->usb_type == POWER_SUPPLY_TYPE_USB_FLOAT) &&
				(chip->exit || chip->dchg_enabled == false))
#endif
		{
#if OLD_CAM_NOTIFY
			chip->cam_exit_restore = false;
#endif
			pr_info("enable dchg\n");
			chip->dchg_pon_reset = true;
			cancel_delayed_work(&chip->chg_monitor_work);
			msleep(1000);

			/*power off/on reset mcu*/
			stm32l011_shift_en_enable(chip, false);
			stm32l011_usbsel_enable(chip, false);
			stm32l011_power_enable(chip, false);
			/*power off success*/
			chip->exit = false;

			msleep(1000);
			stm32l011_power_enable(chip, true);
			msleep(10);
			stm32l011_shift_en_enable(chip, true);
			msleep(2000);
			stm32l011_usbsel_enable(chip, true);

			/*dual chg used*/
			chip->dual_chg_step = HCHG_WAIT_MASTER_CHG;
			chip->setting_curr_too_low_for_dchg = false;
			chip->set_cur_too_low_for_dual_chg = false;
			chip->dual_chg_to_standalone_req = false;
			/*dual end*/

			chip->time_count = 0;
			schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(1000));

			chip->dchg_pon_reset = false;
			chip->dchg_enabled = true;
		} else {
#if OLD_CAM_NOTIFY
			if (chip->cam_exit_restore) {
				cancel_delayed_work(&chip->chg_monitor_work);
				schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(1000));
			}
#else
			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(1000));
#endif
			pr_info("enable dchg failed,dchg not exit or not charging, usb_type=%d\n", chip->usb_type);
		}
	} else {
		pr_info("disable dchg\n");
		chip->dchg_enabled = false;
		/*write possiblly switch bit when dchg exit and usb plugged*/
		stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
	}
	mutex_unlock(&chip->dchg_enable_lock);
	chip->disable_vote_miss = false;
	return 0;
}
static int stm32l011_restore_pmi_charing(struct stm32l011_mcu *chip,u8 status)
{
	union power_supply_propval enable = {1,};
	union power_supply_propval direct_charger_status = {DIRECT_CHARGER_END,};
	union power_supply_propval charge_status = {0,};
	int rc;

	if (!chip->usb_present) {
		pr_warn("!usb_present\n");
		return 0;
	}

	if (!!status) {
		stm32l011_usbsel_enable(chip,false);
	}

#if 0 //TEST_CYCLE_CHARGE_DISCHARGE
	if (chip->pmi_suspended && chip->chg_switch)
#else
		if (chip->pmi_suspended)
#endif
		{
			pr_warn("enable pmi charging\n");
			power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_DCHG_CHARGING_ENABLED, &enable);
			msleep(500);
			power_supply_lite_get_property(PSYL_CHARGE, POWER_SUPPLY_PROP_STATUS_EX, &charge_status);

			chip->pmi_suspended = false;
			rc = stm32l011_masked_write(chip, MCU_REG_04, MCU_PMI_SUSPENDED_Msk, 0);
			if (rc) {
				pr_err("Couldn't read IRQ_G_REG rc = %d\n", rc);
			}

			power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_DCHG_STATUS, &direct_charger_status);
			msleep(200);
			power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_RERUN_AICL, &enable);
		}

	pr_warn("charge type=%d status = 0x%02x\n", charge_status.intval, status);
	return 0;
}

#if 0
static bool stm32l011_is_inited(struct stm32l011_mcu *chip)
{
	int rc;
	u8 value = 0;
	bool inited = 0;

	rc = stm32l011_read_reg(chip, MCU_IRQ_A_REG, &value);
	if (rc < 0) {
		pr_err("fail to read reg=%d\n",MCU_IRQ_A_REG);
		return 0;
	}

	inited = !!(value & MCU_IRA_INIT_DONE);
	return inited;
}
static bool stm32l011_is_pmisuspend(struct stm32l011_mcu *chip)
{
	int rc;
	u8 value = 0;
	bool pmisuspend = 0;

	rc = stm32l011_read_reg(chip, MCU_IRQ_A_REG, &value);
	if (rc < 0) {
		pr_err("fail to read reg=%d\n",MCU_IRQ_A_REG);
		return 0;
	}

	pmisuspend = !!(value & MCU_IRA_PMI_SUSPEND);
	return pmisuspend;
}
#endif

static bool get_fuel_user_custom(struct stm32l011_mcu *chip)
{
	union power_supply_propval user_custom = {0,};

	power_supply_lite_get_property(PSYL_FUELSUMMARY, POWER_SUPPLY_PROP_FUEL_USER_CUSTOM, &user_custom);
	return (0 != user_custom.intval);
}

static int stm32l011_set_current(struct stm32l011_mcu *chip, int cur)
{
	u8 value = 0, regvalue = 0, reg04 = 0;
	int ret = 0;

	bool force = (get_atboot() || get_fuel_user_custom(chip));

	if (!force && (chip->burn_handling || chip->dchg_pon_reset)) {
		chip->user_vote_lose = true;
		goto exit;
	}

	ret = stm32l011_read_reg(chip, MCU_REG_04, &reg04);
	if (ret < 0) {
		pr_err("fail to read reg=%d\n", MCU_REG_04);
		goto exit;
	}

	ret = stm32l011_read_reg(chip, MCU_ADAPTER_CURRENT, &regvalue);
	if (ret < 0) {
		pr_err("fail to read reg=%d\n", MCU_ADAPTER_CURRENT);
		goto exit;
	}
	if (cur > chip->total_dchg_limit_max_ma) {
		pr_err("cur(%d mA) more than total_dchg_limit_max_ma(%d mA)\n", cur, chip->total_dchg_limit_max_ma);
		cur = chip->total_dchg_limit_max_ma;
	}

	if (cur > chip->cable_r_current_limit_max) {
		pr_err("cur(%d mA) more than cable_r_current_limit_max(%d mA)\n", cur, chip->cable_r_current_limit_max);
		cur = chip->cable_r_current_limit_max;
	}
	value = (cur / 100);

	if (cur < 0 || cur > chip->total_dchg_limit_max_ma) {
		pr_info("dchg set icurrent(%d) overflow!!!\n", cur);
		if (reg04 & MCU_SET_CURRENT)
			ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_CURRENT, 0);

	} else if (cur < chip->low_current_exit_threshold && !force) {
		pr_info("exception: dchg set low icurrent(%dmA) exit\n", cur);
		chip->setting_curr_too_low_for_dchg = true;
		if (!(reg04 & MCU_DCHG_SWITCH))
			ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);

	} else {
		pr_info("dchg set icurrent(%dmA), value=%d, regvalue=%d\n", cur, value, regvalue);
		//chip->setting_curr_too_low_for_dchg = false;
		if ((value != regvalue) || !(reg04 & MCU_SET_CURRENT)) {
			ret = stm32l011_write_reg(chip, MCU_ADAPTER_CURRENT, value);
			ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_CURRENT, MCU_SET_CURRENT);
		}
	}
exit:
	return ret;
}

static void stm32l011_set_battery_voltage(struct stm32l011_mcu *chip, int bat_vol)
{
	u8 value_high = 0;
	u8 value_low = 0;
	int ret = 0;

	if (bat_vol < 0) {
		pr_err("bat_vol=%d\n", bat_vol);
		return;
	}
	value_high = bat_vol >> 8 & 0xFF;
	value_low = bat_vol & 0xFF;
	pr_err("value_high=0x%x,value_low=0x%x\n", value_high, value_low);
	ret = stm32l011_write_reg(chip, MCU_FG_VBAT_HIGH, value_high);
	ret = stm32l011_write_reg(chip, MCU_FG_VBAT_LOW, value_low);
	return;
}
static int stm32l011_cable_matched_handler(struct stm32l011_mcu *chip, u8 status)
{
	union power_supply_propval engine = {POWER_SUPPLY_ENGINE_NORMAL,};

	stm32l011_read_reg(chip, MCU_ADAPTER_POWER, &chip->adapter_power);
	stm32l011_read_reg(chip, MCU_ADAPTER_VERSION, &chip->adapter_mcu_version);
	stm32l011_read_reg(chip, MCU_ADAPTER_VENDOR, &chip->adapter_vendor);
	pr_info("status = 0x%02x\n", status);
	if (!!status) {
		chip->cable_matched_event = true;
	}
	fuelsummary_collect_value(ID_BYTE__ADPT_MCU_VER, chip->adapter_mcu_version);
	fuelsummary_collect_value(ID_BYTE__ADPT_VENDOR, chip->adapter_vendor);
	fuelsummary_collect_value(ID_BYTE__ADPT_POWER, chip->adapter_power);

	if (chip->cable_matched_event) {
		if (chip->dchg_supported_type >= DCHG_SINGLE_TYPE_44W) {
			if ((chip->adapter_power & 0x0F) == 0x01 || (chip->adapter_power & 0x0F) == 0x04 || (chip->adapter_power & 0x0F) == 0x05 || (chip->adapter_power & 0x0F) == 0x06 || (chip->adapter_power & 0x0F) == 0x09 || (chip->adapter_power & 0x0F) == 0x0A) {
				if (chip->sbu_cable_id == SBU_CABLE_ID_4A || chip->sbu_cable_id == SBU_CABLE_ID_5A || chip->sbu_cable_id == SBU_CABLE_ID_6A || chip->sbu_cable_id == SBU_CABLE_ID_8A)
					chip->vivo_flash_charge_status = POWER_SUPPLY_ENGINE_SUPER;
			}
		}
		if (chip->vivo_flash_charge_status < POWER_SUPPLY_ENGINE_DUAL)
			chip->vivo_flash_charge_status = POWER_SUPPLY_ENGINE_DUAL;
	}
#if OLD_CAM_NOTIFY
	if (chip->cam_running && chip->cable_matched_event) {
		pr_info("cam running dchg disable\n");
		chip->dchg_enabled = false;
		stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
	}
#endif
	engine.intval = chip->vivo_flash_charge_status;
	power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_ENGINE, &engine);
	return 0;
}

static int stm32l011_vbat_good_handler(struct stm32l011_mcu *chip, u8 status)
{
	union power_supply_propval direct_charger_status = {DIRECT_CHARGER_PREPARE,};

	//adapter power derate
	if (chip->adapter_power_derate_enable) {
		chip->adapter_powerderate = (int)stm32l011_get_adapter_powerderate(chip);
		chip->adapter_powerderate_ready = true;
		pr_info("adapter power derate=%d ready\n", chip->adapter_powerderate);
	}

	pr_info("status = 0x%02x\n", status);
	if (status) {
		chip->vbat_good = true;
		power_supply_lite_set_property(PSYL_CHARGE,
				POWER_SUPPLY_PROP_DCHG_STATUS, &direct_charger_status);
	}
	return 0;
}
static u8 stm32l011_dchg_ic_deviceid(struct stm32l011_mcu *chip)
{
	u8 reg = 0;

	stm32l011_read_reg(chip, (MCU_BQ25970_REGS2_START + 0x13), &reg);
	if (reg == RT9759_DEVICE_INFO)
		chip->chgic_rt9759 = true;
	else
		chip->chgic_rt9759 = false;
	pr_info("chgic_deviceid:reg13=0x%02x,chgic_rt9759=%d\n", reg, chip->chgic_rt9759);
	return reg;
}
static int stm32l011_handshake_failed_handler(struct stm32l011_mcu *chip, u8 status)
{
	pr_info("status=0x%02x,dchg_ic=0x%02x\n", status, stm32l011_dchg_ic_deviceid(chip));
	if (status) {
		chip->handshake_failed = true;
		if (!!status) {
			stm32l011_usbsel_enable(chip,false);
		}
	}
	return 0;
}

static int stm32l011_handshake_success_handler(struct stm32l011_mcu *chip, u8 status)
{
	union power_supply_propval val = {0,};

	pr_info("status=0x%02x,dchg_ic=0x%02x\n", status, stm32l011_dchg_ic_deviceid(chip));
	if (!!status) {
		chip->handshake_success = true;
	}

	if (chip->handshake_success && chip->usb_present && !chip->sbu_detect_busy)
		stm32l011_sbu_cable_id_detect(chip);

	power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_WAKEUP, &val);
	return 0;
}
static int stm32l011_init_done_handler(struct stm32l011_mcu *chip, u8 status)
{
	int rc = 0;
	u8 reg04 = 0x0;

	pr_info("status = 0x%02x\n", status);
	chip->init_done = !!status;

	rc = stm32l011_read_reg(chip, MCU_REG_04, &reg04);
	if (get_effective_result_locked(chip->dchg_disable_votable) && !(reg04 & MCU_DCHG_SWITCH)) {
		pr_err("dchg_disable_votable mis-match, try to vote again\n");
		chip->disable_vote_miss = true;
		rerun_election(chip->dchg_disable_votable);
	}

	if (get_atboot()) {
		pr_warn("at mode set ibus max to 2800\n");
		stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_AT_BOOT_MODE, MCU_SET_AT_BOOT_MODE);

		rc = vote(chip->dchg_fcc_votable, FACTORY_FCC_VOTER, true, 2800);
		if (rc < 0) {
			pr_err("Couldn't vote fastchg ma rc = %d\n", rc);
		}
	} else {
		if (chip->enable_slave_charger) {
			//stm32l011_set_current(chip, chip->total_dchg_limit_max_ma);	//init current first
			rc = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, true,
					chip->dual_chg_cur_threshold);
			if (rc < 0) {
				pr_err("Couldn't vote fastchg ma rc = %d\n", rc);
			}
		}
	}

	stm32l011_check_battery_high(chip);
	return 0;
}
static int stm32l011_pmi_suspend_handler(struct stm32l011_mcu *chip, u8 status)
{
	union power_supply_propval enable = {0,};
	union power_supply_propval direct_charger_status = {DIRECT_CHARGER_IS_CHARGERING,};
	union power_supply_propval charge_status = {0,};
	union power_supply_propval vbat_mv = {0,};
	int rc;
#if OLD_CAM_NOTIFY
	if (chip->cam_running) {
		pr_warn("!cam running dchg skip pmi_suspend\n");
		return 0;
	}
#endif
	power_supply_lite_get_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_STATUS_EX, &charge_status);

	pr_warn("charge status=%d status=0x%02X\n", charge_status.intval, status);
	if (!status) {
		pr_warn("pmi exit event recevied\n");
		return 0;
	}

	power_supply_lite_set_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_DCHG_STATUS, &direct_charger_status);

	chip->pmi_suspend = true;
	power_supply_lite_set_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_DCHG_CHARGING_ENABLED, &enable);
	msleep(1000);
	power_supply_lite_get_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_STATUS_EX, &charge_status);
	if (charge_status.intval != POWER_SUPPLY_STATUS_CHARGING &&
			charge_status.intval != POWER_SUPPLY_STATUS_FULL) {
		/*dual chg used*/
		power_supply_lite_get_property(PSYL_BATTERY,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_mv);
		chip->real_vbat_mv = vbat_mv.intval;
		pr_warn("pmi suspended, real_vbat_mv=%d\n", chip->real_vbat_mv);
		/*dual end*/
		chip->pmi_suspended = true;
		rc = stm32l011_masked_write(chip, MCU_REG_04, MCU_PMI_SUSPENDED_Msk, MCU_PMI_SUSPENDED);
		if (rc) {
			pr_err("Couldn't read IRQ_G_REG rc = %d\n", rc);
		}
	} else {
		pr_warn("pmi not suspended\n");
	}
	return 0;

}

static int stm32l011_dchg_exit_handler(struct stm32l011_mcu *chip, u8 status)
{
	union power_supply_propval usbin_mv = {0,};
	union power_supply_propval value = {0,};
	int vbus_mv = 0;

	power_supply_lite_get_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_CHARGE_VOLTAGE, &usbin_mv);
	vbus_mv = usbin_mv.intval;
	pr_warn("vbus_mv = %d,status=0x%02X\n", vbus_mv, status);
	chip->dchg_exit_status = status;
	if (!!status) {
		chip->exit = true;
		//vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, true, 0);
		chip->dchg_enabled = 0xFF;
	}
	if (vbus_mv > 4500) {
		stm32l011_restore_pmi_charing(chip, status);

		if (!chip->chg_switch) {
			value.intval = 0;
			power_supply_lite_set_property(PSYL_CHARGE,
					POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
		}
	} else {
		cancel_delayed_work(&chip->restore_pmi_work);
		schedule_delayed_work(&chip->restore_pmi_work, msecs_to_jiffies(2000));
	}

	chip->report_big_data = true;
	return 0;
}
static struct irq_handler_info handlers[2] = {
	[0] = {
		.stat_reg	= MCU_IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "cold_soft",
				//.smb_irq = stm32l011_init_done_handler,
			},
			{	.name	 = "hot_soft",
				.smb_irq = stm32l011_cable_matched_handler,
			},
			{	.name	 = "cold_hard",
				//.smb_irq = stm32l011_init_done_handler,
			},
			{	.name	 = "vbat_good",
				.smb_irq = stm32l011_vbat_good_handler,
			},
			{	.name	 = "pmi_suspend",
				.smb_irq = stm32l011_pmi_suspend_handler,
			},
			{	.name	 = "handshake_failed",
				.smb_irq = stm32l011_handshake_failed_handler,
			},
			{	.name	 = "handshake_success",
				.smb_irq = stm32l011_handshake_success_handler,
			},
			{	.name	 = "init_done",
				.smb_irq = stm32l011_init_done_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= MCU_IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "dchg_exit",
				.smb_irq = stm32l011_dchg_exit_handler,
			},
			{	.name	 = "battery_missing",
				//.smb_irq = stm32l011_battery_missing_handler,
			},
			{	.name	 = "batt_therm_removed",
				//.smb_irq = stm32l011_battery_missing_handler,
			},
		},
	},
};

#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		1
static irqreturn_t stm32l011_chg_stat_handler(int irq, void *dev_id)
{
	struct stm32l011_mcu *chip = dev_id;
	int i, j;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;
	bool enable = 0;

	mutex_lock(&chip->irq_complete);
	pr_info("IRQ triggered\n");
	//mutex_unlock(&chip->irq_complete);
	//return IRQ_HANDLED;
	if (chip->burn_handling || chip->dchg_pon_reset) {
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		pr_info("IRQ triggered before device-resume\n");
		//disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	if (chip->shift_en_gpio > 0) {
		enable = gpio_get_value(chip->shift_en_gpio);
		if (!enable) {
			pr_info("shit_en not enabled\n");
			stm32l011_shift_en_enable(chip,true);
		}
	}

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = stm32l011_read_reg(chip, handlers[i].stat_reg,
				&handlers[i].val);
		if (rc) {
			pr_err("Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;
			//pr_info("reg=0x%02X value=0x%02X rt_stat=0x%02X prev_rt_stat=0x%02X\n",
			//handlers[i].stat_reg,handlers[i].val,rt_stat,prev_rt_stat);
			if (changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
					handlers[i].irq_info[j].low++;

			if (changed	&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
						rt_stat);
				if (rc)
					pr_err("Couldn't handle %d irq for reg 0x%02x rc = %d\n",
							j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_info("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_info("batt psy changed\n");

	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static void stm32l011_chg_deinit_work(struct work_struct *work)
{
	struct stm32l011_mcu *chip = container_of(work,
			struct stm32l011_mcu, chg_deinit_work.work);
	stm32l011_hw_deinit(chip);
	stm32l011_relax(chip,PM_DEINIT);
}
static void  stm32l011_restore_pmi_work(struct work_struct *work) {
	struct stm32l011_mcu *chip = container_of(work,
			struct stm32l011_mcu, restore_pmi_work.work);
	stm32l011_restore_pmi_charing(chip,chip->dchg_exit_status);
	//stm32l011_relax(chip,PM_DEINIT);
}
static bool stm32l011_ex_fg_i2c_is_exception(struct stm32l011_mcu *chip)
{
	/*bq27750_IIC_ERROR = BIT(1) */
#if 0
	if ((ex_fg_i2c_hand_step == 4 || ex_fg_i2c_hand_step >= 7
				|| chip->ex_fg_i2c_error_counter > 20) && (ex_fg_state & BIT(1))) {
		pr_info("ex fg i2c is exception!\n");
		return true;
	}else {
		return false;
	}
#else
	return false;
#endif
}

static int stm32l011_ffc_update_param(struct stm32l011_mcu *chip, int bat_temp, int *ffc_cc_to_cv_ibat_threshold)
{
	int index;

	/*get ffc cc to cv ibat threshold*/
	for (index = 0; index < chip->ffc_cc_to_cv_ibat_thr_rc[ROW]; index++) {
		if (is_between(chip->ffc_cc_to_cv_ibat_thr[index].tmin, chip->ffc_cc_to_cv_ibat_thr[index].tmax,
					bat_temp)) {
			*ffc_cc_to_cv_ibat_threshold = chip->ffc_cc_to_cv_ibat_thr[index].ibat;
			break;
		}
	}

	/* compatible ex fg and qcom fg */
	switch (chip->ffc_param_tuning_enable) {
	case 1963:
		/*for project:1963*/
		if (!chip->ex_fg_ffc_support) {
			if (chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].fg_cv_mv != chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv) {
				chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].fg_cv_mv = chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv;
				chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].fg_cv_mv = chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].btb_cv_mv;
			}
		}
		break;
	default:
		pr_info("nothing to do.\n");
	}

	return 0;
}

#define DCHG_BASE_CV_MV	4300
/* Adjust const charging voltage threshold by temperature and charing status */
static void stm32l011_adjust_dchg_ffc_cv_threshold(struct stm32l011_mcu *chip)
{
	union power_supply_propval batt_temp_now = {0,};
	u8 dela_cv = 0, regvalue = 0, regvalue2 = 0;
	int ret = 0, index = 0, ffc_cc_to_cv_ibat_threshold = 4000;

	/*get data*/
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_TEMP, &batt_temp_now);

	/*adjust ffc param*/
	stm32l011_ffc_update_param(chip, batt_temp_now.intval, &ffc_cc_to_cv_ibat_threshold);

	if (batt_temp_now.intval <= chip->ffc_temperature_range[0] || batt_temp_now.intval >= chip->ffc_temperature_range[1]
			|| stm32l011_ex_fg_i2c_is_exception(chip)) {
		index = FFC_DCHG_STEP_CV_DEFAULT;
	} else {
		if (chip->req_ibus * 2 > ffc_cc_to_cv_ibat_threshold || !chip->chg_enable) {/*step 1: default step*/
			index = FFC_DCHG_STEP_CV_DEFAULT;
		} else if (chip->chg_enable && chip->slave_chg_enable) {
			index = FFC_DCHG_STEP_CV_DUAL;
		} else {
			index = FFC_DCHG_STEP_CV_SINGLE;
		}
	}

	dela_cv = chip->ffc_dchg_cv_mv[index].btb_cv_mv - DCHG_BASE_CV_MV;
	ret = stm32l011_read_reg(chip, MCU_BTB_CV_VOLTAGE, &regvalue);
	if (ret < 0) {
		pr_err("fail to read btb cv voltage reg=%d\n", MCU_BTB_CV_VOLTAGE);
	}
	if (dela_cv != regvalue) {
		ret = stm32l011_write_reg(chip, MCU_BTB_CV_VOLTAGE, dela_cv);
		if (ret) {
			pr_err("btb cv voltage reg write failed!\n");
		}
	}

	dela_cv = chip->ffc_dchg_cv_mv[index].fg_cv_mv - DCHG_BASE_CV_MV;
	ret = stm32l011_read_reg(chip, MCU_FG_VBAT_CV_VOLTAGE, &regvalue2);
	if (ret < 0) {
		pr_err("fail to read fg cv voltage reg=%d\n", MCU_FG_VBAT_CV_VOLTAGE);
	}
	if (dela_cv != regvalue2) {
		ret = stm32l011_write_reg(chip, MCU_FG_VBAT_CV_VOLTAGE, dela_cv);
		if (ret) {
			pr_err("fg cv voltage reg write failed!\n");
		}
	}
	pr_info("MCU_BTB_CV=%d,MCU_FG_VBAT_CV=%d\n", (regvalue + DCHG_BASE_CV_MV), (regvalue2 + DCHG_BASE_CV_MV));

	pr_info("bat_temp=%d,chg_enable=%d,slave_chg_enable=%d,req_ibus=%d,index=%d,ffc_cc_to_cv_ibat_threshold=%dmA,default[%d,%d],single[%d,%d],dual[%d,%d]\n",
			batt_temp_now.intval, chip->chg_enable, chip->slave_chg_enable, chip->req_ibus, index, ffc_cc_to_cv_ibat_threshold,
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DEFAULT].btb_cv_mv, chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DEFAULT].fg_cv_mv,
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv, chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].fg_cv_mv,
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].btb_cv_mv, chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].fg_cv_mv);
}

static void stm32l011_check_cout_status(struct stm32l011_mcu *chip)
{
	if (get_atboot()) {
		pr_warn("at mode skip status monitor\n");
		return;
	}

	mutex_lock(&chip->ffc_lock);
	if (!chip->chg_enable) {
		pr_info("dchg not enable!\n");
		mutex_unlock(&chip->ffc_lock);
		return;
	}

	if (chip->cout_gpio > 0)
		chip->battery_cout_value = gpio_get_value(chip->cout_gpio);
	if (chip->battery_cout_value)
		stm32l011_masked_write(chip, MCU_REG_08, MCU_SET_CV_MODE, MCU_SET_CV_MODE);
	mutex_unlock(&chip->ffc_lock);
	return;
}

static void stm32l011_dump_regs(struct stm32l011_mcu *chip)
{
	int rc;
	u8 reg = 0,i;

	if (1)
		return;//add for avoid i2c interrupt storm
#if 0
	for (i=MCU_REGESTER_START; i<=MCU_REGESTER_END; i++) {
		rc = stm32l011_read_reg(chip, i, &reg);
		pr_info("MCU:reg=0x%02X,value=0x%02X\n",i,reg);
	}

	for (i=MCU_VERSION_START; i<=MCU_VERSION_END; i++) {
		rc = stm32l011_read_reg(chip, i, &reg);
		pr_info("MCU:version=0x%02X,value=0x%02X\n",i,reg);
	}

	for (i=MCU_ADC_START; i<=MCU_ADC_END; i++) {
		rc = stm32l011_read_reg(chip, i, &reg);
		pr_info("ADC:reg=0x%02X,value=0x%02X\n",i,reg);
	}
	for (i=MCU_LOG1_START; i<=MCU_LOG1_END; i++) {
		rc = stm32l011_read_reg(chip, i, &reg);
		pr_info("LOG1:reg=0x%02X,value=0x%02X\n",i,reg);
	}
#endif
#if 1
	for (i=MCU_BQ25970_REGS2_START; i<=MCU_BQ25970_REGS2_END; i++) {
		if (chip->dchg_version == NXP_PCA9486_DCHG) {
			if (i-MCU_BQ25970_REGS2_START > 0x10 && i-MCU_BQ25970_REGS2_START < 0x20)
				continue;
			if (i-MCU_BQ25970_REGS2_START > 0x2A && i-MCU_BQ25970_REGS2_START < 0x31)
				continue;
			rc = stm32l011_read_reg(chip, i, &reg);
			pr_info("PCA9468:reg=0x%02X,value=0x%02X\n", i-MCU_BQ25970_REGS2_START, reg);
		} else if (chip->dchg_version == LN8000_DCHG) {
			if ((i - MCU_BQ25970_REGS2_START) <= 0x12) {
				rc = stm32l011_read_reg(chip, i, &reg);
				pr_info("LN8000:reg=0x%02X,value=0x%02X\n", i-MCU_BQ25970_REGS2_START, reg);
			} else if ((i - MCU_BQ25970_REGS2_START) < 0x1B) {
				continue;
			} else if (i - MCU_BQ25970_REGS2_START <=  0x29) {
				rc = stm32l011_read_reg(chip, i, &reg);
				pr_info("LN8000:reg=0x%02X,value=0x%02X\n", i-MCU_BQ25970_REGS2_START, reg);
			} else {
				continue;
			}
			//rc = stm32l011_read_reg(chip, i, &reg);
			//pr_info("LN8000:reg=0x%02X,value=0x%02X\n", i-MCU_BQ25970_REGS2_START, reg);
		} else {
			rc = stm32l011_read_reg(chip, i, &reg);
			pr_info("BQ25970:reg=0x%02X,value=0x%02X\n", i-MCU_BQ25970_REGS2_START, reg);
		}
	}

	if (chip->enable_slave_charger) {
		for (i=MCU_BQ25970_REGS3_START; i<=MCU_BQ25970_REGS3_END; i++) {
			rc = stm32l011_read_reg(chip, i, &reg);
			pr_info("Slave BQ25970:reg=0x%02X,value=0x%02X\n",i-MCU_BQ25970_REGS3_START,reg);
		}
	}
#endif
}
static void stm32l011_check_usb_present(struct stm32l011_mcu *chip)
{
	union power_supply_propval usbin_mv = {0,};
	union power_supply_propval ibat_ma = {0,};
	union power_supply_propval vbat_mv = {0,};

	power_supply_lite_get_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_CHARGE_VOLTAGE, &usbin_mv);
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ma);
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_mv);
	usbin_mv.intval = usbin_mv.intval;
	vbat_mv.intval = vbat_mv.intval;
	ibat_ma.intval = ibat_ma.intval / 10;
	pr_info("usbin_mv=%d,vbat_mv=%d ibat_ma=%d\n",
			usbin_mv.intval, vbat_mv.intval, ibat_ma.intval);

	/*delete because of not used in bq25970*/
#if 0
	if (usbin_mv.intval - vbat_mv.intval < 20 && ibat_ma.intval > 0 &&
			usbin_mv.intval > 0) {
		pr_info("usb unpresent\n");
		//chip->usb_present = false;
	}
#endif

}
static bool stm32l011_check_dpdm_status(struct stm32l011_mcu *chip)
{

	int dp_mv = 0,dm_mv = 0;
	bool exception = false;

	dp_mv = stm32l011_get_dp_mv(chip);
	dm_mv = stm32l011_get_dm_mv(chip);
	if (dp_mv > 3800 || dm_mv > 3800) {
		exception = true;
	}
	if (exception)
		pr_info("dp_mv=%d dm_mv=%d exception=%d\n", dp_mv,dm_mv, exception);
	return exception;
}

static int stm32l011_check_battery_high(struct stm32l011_mcu *chip)
{
	bool bat_high = false;
	union power_supply_propval bat_soc = {0,};
	union power_supply_propval vbat_uv = {0,};
	int vbat_high_threshold = chip->ex_fg_ffc_support ? 4380 : 4340;

	if (!chip->ex_fg_ffc_support) {
		return 0;
	}

	if (get_atboot()) {
		pr_warn("at mode skip..\n");
		return 0;
	}

	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_CAPACITY, &bat_soc);

	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);

	if ((bat_soc.intval >= 100) || (vbat_uv.intval > vbat_high_threshold)) {
		pr_err("soc(%d) / voltage(%d) too high, skip direct-charging....\n", bat_soc.intval, vbat_uv.intval);
		bat_high = true;;
	}

	if (bat_high == true)
		stm32l011_masked_write(chip, MCU_REG_08, MCU_FFC_BAT_HIGH, MCU_FFC_BAT_HIGH);
	else
		stm32l011_masked_write(chip, MCU_REG_08, MCU_FFC_BAT_HIGH, 0);

	return 0;
}

static bool stm32l011_check_battery_status(struct stm32l011_mcu *chip)
{
	union power_supply_propval vbat_mv = {0,};
	union power_supply_propval bat_id = {0,};
	bool exception = false;

	/* at mode*/
	if (get_atboot()) {
		pr_err("at mode,return\n");
		return exception;
	}

	/*dual-charger normal mode*/
#if 0
	if (chip->enable_slave_charger && !ex_fg_detect_done) {
		pr_err("ex_fg_detect_done not finish!\n");
		return exception;
	}
#endif
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_BATTERY_ID, &bat_id);
	if (bat_id.intval <= 0) {
		pr_err("exception: bat_id error.\n");
		exception = true;
		return exception;
	}

	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_mv);
	if (vbat_mv.intval < 3000) {
		pr_err("exception: vbat_mv low 3v,error.\n");
		exception = true;
		return exception;
	}

	return exception;
}
static bool stm32l011_check_ibat_high(struct stm32l011_mcu *chip)
{
	union power_supply_propval ibat = {0,};
	int ibat_ma = 0;
	bool exception = false;

	/*get ibat from bq27546*/
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat);
	ibat_ma = ibat.intval / 10;
	ibat_ma = abs(ibat_ma);

	if (ibat_ma > chip->total_dchg_limit_max_ma + 100) {
		exception = true;
	}
	pr_info("ibat_ma=%d\n", ibat_ma);
	return exception;
}

static bool stm32l011_check_vbat_high(struct stm32l011_mcu *chip)
{
	union power_supply_propval pval = {0,};
	int vbat_mv = 0, vbat_ocv_mv = 0;
	bool exception = false;
	int vbat_high_thershold = 0;

	/*get vbat from bms*/
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	vbat_mv = pval.intval;

	/*get ocv from bms*/
	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_VOLTAGE_OCV, &pval);
	vbat_ocv_mv = pval.intval;

	/* get set vbat_high_thershold, and bms vbat to mcu */
	if (chip->ex_fg_ffc_support) {
		stm32l011_set_battery_voltage(chip, vbat_mv);
		vbat_high_thershold = 4600;
	} else {
		vbat_high_thershold = 4550;
	}

	if (vbat_mv >= vbat_high_thershold || vbat_ocv_mv > vbat_high_thershold) {
		exception = true;
	}

	pr_info("vbat_mv=%d,vbat_ocv_mv=%d,vbat_high_thershold=%d\n",
			vbat_mv, vbat_ocv_mv, vbat_high_thershold);

	return exception;
}

#define VBUS_QC2P0_THISREHOLD_MV 6500
#define VBUS_QC2P0_CHECK_COUNT_MAX 2
/*monitor vbus more than 6.5V, because of
 * QC2.0 charger before handshake successful
 */
static int stm32l011_check_vbus_status(struct stm32l011_mcu *chip)
{
	union power_supply_propval usbin_mv = {0,};
	static int exception_count;
	int enable = 0;

	enable = stm32l011_get_usbsel_state(chip);
	if (!enable) {
		pr_info("usbsel_state not switched,not to check vbus\n");
		return 0;
	}

	power_supply_lite_get_property(PSYL_CHARGE,
			POWER_SUPPLY_PROP_CHARGE_VOLTAGE, &usbin_mv);
	usbin_mv.intval = usbin_mv.intval;
	if (usbin_mv.intval > VBUS_QC2P0_THISREHOLD_MV) {
		if (++exception_count > VBUS_QC2P0_CHECK_COUNT_MAX) {
			pr_err("vbus more than 6.5V because of QC2.0\n");
			stm32l011_usbsel_enable(chip, false);
			chip->vbus_exception = true;
			exception_count = 0;
		}
	} else {
		exception_count = 0;
	}
	return 0;
}

int stm32l011_ap_control_mcu_UART_TRx(struct stm32l011_mcu *chip, u8 cmd, u8 tx_data_lenght, u8 *tx_data, u8 * rx_data, u8 rx_data_lenght);
#define ADAPTER_SHA_ENCRYPTION 1
#if ADAPTER_SHA_ENCRYPTION
uint32_t Wwdata[16];
uint32_t HASH_Data[5];
static void DChg_Sha1_Init(void)
{
	//Sha1 Key value
	Wwdata[4]=0x56724e49;
	Wwdata[5]=0x6f487074;
	Wwdata[6]=0x49695857;
	Wwdata[7]=0x76437061;
	Wwdata[8]=0x56635041;
	Wwdata[9]=0x49497877;
	Wwdata[10]=0x4f685054;
	Wwdata[11]=0x76526e69;
}
/*
   uint32_t rol32(uint32_t word, uint32_t shift)
   {
   return (word << shift) | (word >> (32 - shift));
   }
   */
int Sha_Transform(void)
{
	int ret = 0;
	int i = 0;
	uint32_t temp=0;
	uint32_t a=0x67452301,b=0xefcdab89,c=0x98badcfe,d=0x10325476,e=0xc3d2e1f0,f,k;

	uint32_t Wdata[80];

	for (i=0;i<16;i++)
		Wdata[i]=Wwdata[i];

	for (i=16;i<80;i++)
	{
		Wdata[i]=rol32((uint32_t)(Wdata[i-3] ^ Wdata[i-8]^ Wdata[i-14]^ Wdata[i-16]),1);
	}

	for (i=0;i<=79;i++)
	{

		if ((i>=0)&&(i<=19))
		{
			f=((b&c) | ((~b)^d)) & (c|d);
			k=0x5a827999;
		}

		else if ((i>=20)&&(i<=39))
		{
			f=(((~b)&d) & (c|(~d)))|(b^c);
			k=0x6ed9eba1;
		}
		else if ((i>=40)&&(i<=59))
		{
			f=((c&d)|(b&d))&(b^(~c));
			k=0x8f1bbcdc;
		}
		else if ((i>=60)&&(i<=79))
		{
			f=((b&(~c))|(b^(~d)))|(b&d);
			k=0xca62c1d6;
		}

		temp=rol32(a,5)+f+e+k+Wdata[i];
		e=d;
		d=c;
		c=rol32(b,30);
		b=a;
		a=temp;
	}
	a+=0x67452301;
	b+=0xefcdab89;
	c+=0x98badcfe;
	d+=0x10325476;
	e+=0xc3d2e1f0;
	/*
	   hash[0] +=a;
	   hash[1] +=b;
	   hash[2] +=c;
	   hash[3] +=d;
	   hash[4] +=e;

	   hash[0] = rol32(hash[0], 7) + 0x70667873;
	   hash[1] = rol32(hash[1], 6) + 0x70667873;
	   hash[2] = rol32(hash[2], 6) + 0x70667873;
	   hash[3] = rol32(hash[3], 12) + 0x70667873;
	   hash[4] = rol32(hash[4], 5) + 0x70667873;
	   */
	a = rol32(a, 7) + 0x70667873;
	b = rol32(b, 6) + 0x70667873;
	c = rol32(c, 6) + 0x70667873;
	d = rol32(d, 12) + 0x70667873;
	e = rol32(e, 5) + 0x70667873;

	//if ((e==0x05ece771)&&(c==0x304a762c)&&(a==0x2f8ccc85)&&(b==0x627507ca)&&(d==0x15915420))
	//	ret=0;
	//if ((hash[4]==HASH_Data[0])&&(hash[2]==HASH_Data[1])&&(hash[0]==HASH_Data[2])&&(hash[1]==HASH_Data[3])&&(hash[3]==HASH_Data[4]))

	if ((e==HASH_Data[0])&&(c==HASH_Data[1])&&(a==HASH_Data[2])&&(b==HASH_Data[3])&&(d==HASH_Data[4]))
		//if ((a==HASH_Data[0])&&(b==HASH_Data[1])&&(c==HASH_Data[2])&&(d==HASH_Data[3])&&(e==HASH_Data[4]))
		ret=1;	//success~!!!

	return ret;
}

int DChg_Check_Adapter_Hash(struct stm32l011_mcu *chip)
{

	uint8_t cmd;
	int ret = 0;
	uint8_t txBuffer[16];
	uint8_t rxBuffer[36];
	uint32_t rand_data = 0;
	uint8_t i = 0;

	for (i=0;i<4;i++)
	{
		get_random_bytes(&rand_data, sizeof(uint32_t));
		Wwdata[i]=rand_data;

		txBuffer[4*i+3] = (uint8_t)((Wwdata[i]>>24)&(0x000000ff));
		txBuffer[4*i+2] = (uint8_t)((Wwdata[i]>>16)&(0x000000ff));
		txBuffer[4*i+1] = (uint8_t)((Wwdata[i]>>8)&(0x000000ff));
		txBuffer[4*i] = (uint8_t)((Wwdata[i])&(0x000000ff));
	}

	cmd = 0x0C;
	ret = stm32l011_ap_control_mcu_UART_TRx(chip, cmd, 16, txBuffer, rxBuffer, 36);
	//pr_err(" cmd=0x0C: ret=%d. random_data[0x%x, 0x%x, 0x%x, 0x%x]\n", ret, Wwdata[0], Wwdata[1], Wwdata[2], Wwdata[3]);

	if (ret <= 0) {
		pr_err(" Adapter get Hash error!!!\n");
		return ret;
	}

	for (i=0;i<16;i=i+4)
		Wwdata[i/4+12] = (uint32_t)(rxBuffer[i+3]<<24 | rxBuffer[i+2]<<16| rxBuffer[i+1]<<8 | rxBuffer[i]);
	/*
	   Wwdata[12] = (uint32_t)(rxBuffer[3]<<24 | rxBuffer[2]<<16| rxBuffer[1]<<8 | rxBuffer[0]);
	   Wwdata[13] = (uint32_t)(rxBuffer[7]<<24 | rxBuffer[6]<<16| rxBuffer[5]<<8 | rxBuffer[4]);
	   Wwdata[14] = (uint32_t)(rxBuffer[11]<<24 | rxBuffer[10]<<16| rxBuffer[9]<<8 | rxBuffer[8]);
	   Wwdata[15] = (uint32_t)(rxBuffer[15]<<24 | rxBuffer[14]<<16| rxBuffer[13]<<8 | rxBuffer[12]);
	   */
	for (i=16;i<36;i=i+4)
		HASH_Data[(i-16)/4] = (uint32_t)(rxBuffer[i+3]<<24 | rxBuffer[i+2]<<16| rxBuffer[i+1]<<8 | rxBuffer[i]);	//HASH[4]  HASH[2]  HASH[0]  HASH[1]  HASH[3]
	/*
	   HASH_Data[0] = (uint32_t)(rxBuffer[19]<<24 | rxBuffer[18]<<16| rxBuffer[17]<<8 | rxBuffer[16]);
	   HASH_Data[1] = (uint32_t)(rxBuffer[23]<<24 | rxBuffer[22]<<16| rxBuffer[21]<<8 | rxBuffer[20]);
	   HASH_Data[2] = (uint32_t)(rxBuffer[27]<<24 | rxBuffer[26]<<16| rxBuffer[25]<<8 | rxBuffer[24]);
	   HASH_Data[3] = (uint32_t)(rxBuffer[31]<<24 | rxBuffer[30]<<16| rxBuffer[29]<<8 | rxBuffer[28]);
	   HASH_Data[4] = (uint32_t)(rxBuffer[35]<<24 | rxBuffer[34]<<16| rxBuffer[33]<<8 | rxBuffer[32]);
	   */

	ret = Sha_Transform();
	if (ret <= 0)
		pr_err(" Adapter Hash check error!!!\n");
	else
		pr_err(" Adapter Hash check Success!!!\n");

	return ret;
}
#endif

int stm32l011_ap_control_mcu_UART_TRx(struct stm32l011_mcu *chip, u8 cmd, u8 tx_data_lenght, u8 *tx_data, u8 * rx_data, u8 rx_data_lenght)
{
	u8 *tx_buff = &chip->tx_buff[0], tx_len;
	u8 *rx_buff = &chip->rx_buff[0], rx_len;
	u8 trx_status = 0;
	int ret = 0, status_try = 0;
	char data[800] = {0,};
	int len = 0, i = 0;

	//TX_DATA:
	tx_buff[0] = AP_CONTROL_MCU_UART_TX_DATA_REG;
	tx_buff[1] = cmd & 0xFF;
	tx_buff[2] = tx_data_lenght & 0xFF;
	memcpy(&tx_buff[3], tx_data, tx_data_lenght);
	tx_len = tx_data_lenght + 3;
	ret = stm32l011_i2c_write(chip, tx_buff, tx_len);
	if (ret <= 0) {
		pr_err("TX send command i2c error(reg=0x%02x, ret=%d)\n", tx_buff[0], ret);
		goto OUT;
	}

	//CHECK_TRx_STATUS:
	while(status_try++ < 20) {
		if (!chip->usb_present)
			break;
		msleep(20);
		ret = stm32l011_read_reg(chip, AP_CONTROL_MCU_UART_STATUS_REG, &trx_status);
		if (ret < 0) {
			pr_err("fail to read TRx_status_reg[0x%x], ret = %d\n", AP_CONTROL_MCU_UART_STATUS_REG, ret);
			goto OUT;
		} else {
			if (trx_status == cmd) {
				pr_err("read TRx_status_reg[0x%02x]=0x%02x, Success!!!\n", AP_CONTROL_MCU_UART_STATUS_REG, trx_status);
				rx_buff[0] = AP_CONTROL_MCU_UART_RX_DATA_REG;
				rx_len = rx_data_lenght + 1;
				goto RX_DATA;
			}
		}
		pr_err("read TRx_status_reg[0x%02x]=0x%02x,  status_try=%d.\n", AP_CONTROL_MCU_UART_STATUS_REG, trx_status, status_try);
	}

	pr_err("read TRx_status_reg[0x%02x]=0x%02x,  status_try=%d, usb_present=%d, Fail!!!\n", AP_CONTROL_MCU_UART_STATUS_REG, trx_status, status_try, chip->usb_present);
	ret = -EINVAL;
	goto OUT;

RX_DATA:
	ret = stm32l011_i2c_read(chip, rx_buff, rx_len);
	if (ret <= 0) {
		pr_err("RX read command i2c error(reg=0x%02x, ret=%d)\n", rx_buff[0], ret);
		goto OUT;
	}

	if (rx_data != NULL) {
		if (trx_status == cmd) {
			memcpy(rx_data, &rx_buff[1], rx_data_lenght);
			ret = 1;	//success!!
		} else
			ret = -EINVAL;
	}

	//TRx_LOG:
	memset(data, 0, sizeof(data));
	len = 0;

	len += snprintf(data+len, sizeof(data) - len, "TX_send: cmd=0x%02x, tx_data_len=%d, tx_data=", cmd, tx_data_lenght);
	for (i = 0; i < tx_data_lenght; i++)
		len += snprintf(data+len, sizeof(data) - len, "0x%02x ", tx_buff[3+i]);

	len += snprintf(data+len, sizeof(data) - len, ", TRx_status=0x%02x(%s),  ", trx_status, (trx_status == cmd) ? "Success" : "Error");

	len += snprintf(data+len, sizeof(data) - len, "Rx_value: ");
	for (i = 1; i < rx_len; i++)
		len += snprintf(data+len, sizeof(data) - len, "0x%02x ", rx_buff[i]);

	pr_err(" ret=%d, %s.\n", ret, data);
OUT:
	return ret;
}

//#define EXIT_ADAPTER_IDENTIFICATION_ERROR	BIT(2)
void stm32l011_AP_MCU_main_sm(struct stm32l011_mcu *chip)
{
	u8 value;
	//u8 cmd, tx_data_lenght, tx_data[40], rx_data[60], rx_data_lenght;
	static int retry = 0;
	int rc = 0;
	union power_supply_propval val = {0,};
	union power_supply_propval engine = {POWER_SUPPLY_ENGINE_NORMAL,};

	rc = stm32l011_read_reg(chip, MCU_MAIN_CHG_STATE_MACHINE, &value);
	if (rc < 0) {
		pr_err("fail to read reg=%d\n", MCU_MAIN_CHG_STATE_MACHINE);
		return;
	}
	if (value != chip->mcu_main_chg_sm)
		retry = 0;

	if (chip->mcu_main_chg_sm > 0 || value > 0)
		pr_err("Start: AP side sm=%d, MCU side sm=%d\n", chip->mcu_main_chg_sm, value);
	chip->mcu_main_chg_sm = value;

	switch(chip->mcu_main_chg_sm)
	{
	case CHARGER_IDENTIFICATION:	//this state machine control in AP side. so in MCU side, need skip it.
		rc = DChg_Check_Adapter_Hash(chip);
		if (rc > 0) {
			chip->mcu_main_chg_sm = CHARGER_HIGH_VOL_MODE_REQUEST;
			rc = stm32l011_write_reg(chip,MCU_MAIN_CHG_STATE_MACHINE,chip->mcu_main_chg_sm);
			pr_err("CHARGER_IDENTIFICATION Success. mcu_main_chg_sm change: %d --> %d (%s).\n",
					CHARGER_IDENTIFICATION, chip->mcu_main_chg_sm, (rc == 0) ? "success" : "error");
		} else {
			if (++retry > 6) {
				chip->mcu_main_chg_sm = CHARGER_EXIT_DIRECT_CHARGING;
				rc = stm32l011_write_reg(chip,MCU_MAIN_CHG_STATE_MACHINE,chip->mcu_main_chg_sm);
				pr_err("CHARGER_IDENTIFICATION Fail. mcu_main_chg_sm change: %d --> %d (%s).\n",
						CHARGER_IDENTIFICATION, chip->mcu_main_chg_sm, (rc == 0) ? "success" : "error");
				chip->main_chg_sm_error_in_ap_side |= EXIT_ADAPTER_IDENTIFICATION_ERROR;//same with hdchg.Instance->R05
				engine.intval = POWER_SUPPLY_ENGINE_NORMAL;
				power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_ENGINE, &engine);
				power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_WAKEUP, &val);
			}
		}
		break;
	default:
		break;
	}

	if (chip->mcu_main_chg_sm > 0 || value > 0)
		pr_err("End: AP side sm=%d.\n", chip->mcu_main_chg_sm);
}

static void dchg_temp_retry_flag_deinit(struct stm32l011_mcu *chip)
{
	chip->low_temp_retries = 0;
	chip->medial_temp_retries = 0;
	chip->high_temp_retries = 0;
	chip->normal_temp_retries = 0;
}
#define EXIT_AP_CLOSE_MCU		BIT(0)
#define EXIT_CHG_PROTECTED 		BIT(6)
#define DCHG_RESTART_TEMP_THRESHOLD_LOW		100
#define DCHG_RESTART_TEMP_THRESHOLD_MEDIAL		150
#define DCHG_RESTART_TEMP_THRESHOLD_NORMAL		200
#define DCHG_RESTART_TEMP_THRESHOLD_HIGH		350
#define DCHG_RESTART_RETIES_MAX		4
static int stm32l011_check_dchg_restart(struct stm32l011_mcu *chip)
{

	union power_supply_propval primary_board_temp_now = {0,};
	bool need_rerun_election = false;
	int i = 0;

	if (get_atboot()) {
		pr_warn("at mode dchg not restart\n");
		return 0;
	}

	if (!chip->usb_present) {
		pr_info("!usb_present\n");
		return 0;
	}

	if(chip->factory_10W_charge_test_enabe != 0) {
		pr_info("factory 10w charging test,not to restart\n");
		return 0;
	}

	if (!chip->exit) {
		pr_info("dchg not exit\n");
		return 0;
	}

	if (chip->dchg_version == NXP_PCA9486_DCHG && chip->reg_exception) {
		pr_info("pca9468 register error,not to restart\n");
		return 0;
	}

	if (chip->dchg_version == LN8000_DCHG && chip->reg_exception) {
		pr_info("ln8000 register error,not to restart\n");
		return 0;
	}

	if (chip->cable_r_disable_dchg) {
		pr_info("cable_r_disable_dchg = 1, skip restart...\n");
		return 0;
	}

	if (chip->dp_dm_ovp) {
		pr_info("dp_dm_ovp = true, skip restart (dp=%d, dm=%d)...\n", chip->dp_mv, chip->dm_mv);
		return 0;
	}

	if (chip->uart_tx_exception > 200 || chip->uart_rx_exception > 200) {
		pr_info("uart_trx_exception skip restart (uart_tx_exception=%d, uart_rx_exception=%d)...\n", chip->uart_tx_exception, chip->uart_rx_exception);
		return 0;
	}

	if (!get_effective_result_locked(chip->dchg_disable_votable)) {
		need_rerun_election = true;
	}

	power_supply_lite_get_property(PSYL_BATTERY,
			POWER_SUPPLY_PROP_PRIMARY_BOARD_TEMP, &primary_board_temp_now);

	/* @@ Restart Case: bq259790 Error, retry more times in different Temp.*/
	if (chip->irqB & EXIT_CHG_PROTECTED && !(chip->irqD & EXIT_AP_CLOSE_MCU) && !(chip->irqB & EXIT_HIGH_VBAT)) {
		if (chip->dchg_version == NXP_PCA9486_DCHG) {
			pr_info("pca9468,not to restart\n");
			return 0;
		}

		if (chip->dchg_version == LN8000_DCHG) {
			pr_info("ln8000,not to restart\n");
			return 0;
		}

		pr_err("dchg exit, seem bq259790 not charing\n");

		if (chip->int_stat_11 & BIT(7)) {//BQ25970 BAT_OVP_FLT_FLAG
			pr_err("not dchg restart,BQ25970 BAT OVP alreadly!\n");
			return 0;
		}

		if (primary_board_temp_now.intval < DCHG_RESTART_TEMP_THRESHOLD_LOW) {
			pr_err("not dchg restart,temp too low\n");
			return 0;
		}

		if (primary_board_temp_now.intval < DCHG_RESTART_TEMP_THRESHOLD_MEDIAL) {
			/*10dec-15dec*/
			pr_err("low temp dchg restart\n");
			if (++chip->low_temp_retries > DCHG_RESTART_RETIES_MAX) {
				pr_err("dchg restart low_temp_retries max\n");
			} else {
				if (chip->enable_slave_charger) {
					if (chip->low_temp_retries >=2)
						chip->slave_charger_should_disable = true;
					else
						chip->slave_charger_should_disable = false;
				}
				/*chip->irqB &= ~EXIT_CHG_PROTECTED; clear bit?? mcu power off,no need clear bit*/
				vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
				if (need_rerun_election)
					rerun_election(chip->dchg_disable_votable);

			}
		} else if (primary_board_temp_now.intval < DCHG_RESTART_TEMP_THRESHOLD_NORMAL) {
			/*15dec-20dec*/
			pr_err("medial temp dchg restart\n");
			if (++chip->medial_temp_retries > DCHG_RESTART_RETIES_MAX) {
				pr_err("dchg restart medial_temp_retries max\n");
			} else {
				if (chip->enable_slave_charger) {
					if (chip->medial_temp_retries >=2)
						chip->slave_charger_should_disable = true;
					else
						chip->slave_charger_should_disable = false;
				}
				/*chip->irqB &= ~EXIT_CHG_PROTECTED; clear bit?? */
				vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
				if (need_rerun_election)
					rerun_election(chip->dchg_disable_votable);
			}
		} else if (primary_board_temp_now.intval < DCHG_RESTART_TEMP_THRESHOLD_HIGH) {
			/*20dec-35dec*/
			pr_err("normal temp dchg restart\n");
			if (++chip->normal_temp_retries > DCHG_RESTART_RETIES_MAX) {
				pr_err("dchg restart normal_temp_retries max\n");
			} else {
				if (chip->enable_slave_charger) {
					if (chip->normal_temp_retries >=2)
						chip->slave_charger_should_disable = true;
					else
						chip->slave_charger_should_disable = false;
				}
				/*chip->irqB &= ~EXIT_CHG_PROTECTED; clear bit?? */
				vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
				if (need_rerun_election)
					rerun_election(chip->dchg_disable_votable);
			}
		} else {
			/*35dec-0xFFFFdec*/
			pr_err("high temp dchg restart\n");
			if (++chip->high_temp_retries > DCHG_RESTART_RETIES_MAX) {
				pr_err("dchg restart high_temp_retries max\n");
			} else {
				if (chip->enable_slave_charger) {
					if (chip->high_temp_retries >=2)
						chip->slave_charger_should_disable = true;
					else
						chip->slave_charger_should_disable = false;
				}
				/*chip->irqB &= ~EXIT_CHG_PROTECTED; clear bit?? */
				vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
				if (need_rerun_election)
					rerun_election(chip->dchg_disable_votable);
			}
		}
		/*retries hanpen between low and medial??? both increase*/
	}
	/* @@ Restart Case: AP intell Charge Temp Control Exit Dchg, retry after Temp recovery.*/
	else if ((chip->irqD & EXIT_AP_CLOSE_MCU) && (chip->irqB == 0x01) &&
			(chip->setting_curr_too_low_for_dchg ||chip->dual_chg_to_standalone_req ||chip->set_cur_too_low_for_dual_chg)\
			&& chip->handshake_success && !chip->handshake_failed && chip->cable_matched_event && !chip->cable_r_disable_dchg \
			&& chip->user_dchg_current >= chip->low_current_exit_threshold \
			&& primary_board_temp_now.intval > 0 && primary_board_temp_now.intval < 550) {
		pr_err("dchg restart because AP request in Normal Case. temp=%d.\n", primary_board_temp_now.intval);
		/*reset the value****************************/
		for (i = 0; i < ARRAY_SIZE(handlers); i++) {
			handlers[i].prev_val = 0;
			handlers[i].val = 0;
		}
		chip->irqA = 0;
		chip->irqB = 0;
		chip->irqC = 0;
		chip->irqD = 0;
		chip->reg04 = 0;
		chip->reg05 = 0;
		chip->pmi_suspend = false;

		chip->setting_curr_too_low_for_dchg = false;
		/*reset the value****************************/
		if (chip->enable_slave_charger)
			dchg_temp_retry_flag_deinit(chip);
		vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
		if (need_rerun_election)
			rerun_election(chip->dchg_disable_votable);
	}

	pr_err("primary_board_temp=%d, setting_curr_too_low_for_dchg=%d, user_dchg_current=%d, handshake_success=%d, "
			"handshake_failed=%d, cable_matched=%d, cable_r_disable_dchg=%d, dual_chg_to_standalone_req=%d,set_cur_too_low_for_dual_chg=%d\n",
			primary_board_temp_now.intval, chip->setting_curr_too_low_for_dchg, chip->user_dchg_current, chip->handshake_success,
			chip->handshake_failed, chip->cable_matched_event, chip->cable_r_disable_dchg, chip->dual_chg_to_standalone_req, chip->set_cur_too_low_for_dual_chg);
	return 0;
}
#define PCA9468_RESET_RETRIES_MAX 3
static int stm32l011_reset_pca9468(struct stm32l011_mcu *chip)
{
#if 0
	int rc;
	u8 reg04 = 0;
	bool need_rerun_election = false;

	/*notify mcu to reset the pca9468*/
	rc = stm32l011_masked_write(chip, MCU_REG_04, MCU_RESET_PCA9468, MCU_RESET_PCA9468);
	if (rc) {
		pr_err("Couldn't write MCU_REG_04 rc = %d\n", rc);
		return rc;
	}

	msleep(700);
	/*check mcu reset pca9468 or not*/
	rc = stm32l011_read_reg(chip, MCU_REG_04, &reg04);
	if (rc) {
		pr_err("Couldn't read MCU_REG_04 rc = %d\n", rc);
		return rc;
	}

	pr_warn("MCU_REG_04 = %d\n", reg04);
	if ((reg04 & MCU_RESET_PCA9468) == 0) {
		if (!get_effective_result_locked(chip->dchg_disable_votable))
			need_rerun_election = true;
		vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
		if (need_rerun_election) {
			rerun_election(chip->dchg_disable_votable);
		}
		chip->reg_exception = false;
		chip->reset_retries++;
		pr_warn("pca reset completed reg04=%d reset_retries=%d\n", reg04, chip->reset_retries);
	}
#endif
	return 0;
}

/*reset pca9468 in handshake failed/pmi suspend/dchg running*/
#define PCA9468_DEVICE_INFO 0x18
#define PCA9468_CHECK_REG_LONG 6
#define PCA9468_CHECK_REG_COUNT 5
static int stm32l011_check_pca9468_reset(struct stm32l011_mcu *chip)
{
	int rc;
	u8 reg;
	char values[PCA9468_CHECK_REG_LONG];

	if (get_atboot()) {
		pr_warn("at mode pca9468 not restart\n");
		return 0;
	}

	if (!chip->usb_present) {
		pr_info("!usb_present\n");
		return 0;
	}


	if (chip->dchg_version != NXP_PCA9486_DCHG) {
		//pr_warn("not PCA9468 restart\n");
		return 0;
	}

	memset(values, 0, sizeof(values));
	for (reg = 0; reg < PCA9468_CHECK_REG_LONG; reg++) {
		rc = stm32l011_read_reg(chip, reg + MCU_BQ25970_REGS2_START, &values[reg]);
		if (rc < 0) {
			pr_warn("iic error,not to check\n");
			return 0;
		}
	}

	if (values[0] == 0 && values[1] == 0 &&
			values[2] == 0 && values[3] == 0 &&
			values[4] == 0 && values[5] == 0) {
		pr_warn("pca registers not updated,not to check\n");
		return 0;
	}

	/*device info may be not 0x18*/
	if (values[0] == PCA9468_DEVICE_INFO &&
			values[3] == 0 && values[4] == 0 && values[5] == 0) {
		chip->reg_exception_count++;
		if (chip->reg_exception_count > PCA9468_CHECK_REG_COUNT) {
			pr_warn("pca registers exception\n");
			chip->reg_exception = true;
		}

	} else {
		chip->reg_exception_count = 0;
	}

	/*reset and restart when uv/ov/not enable/ibus<200mA/cableR>580mR*/
	if (chip->irqB & EXIT_CHG_PROTECTED) {
		pr_warn("pca charging exception\n");
		chip->reg_exception = true;
	}

	/*check,not to reset when  handshaking or handshake success,but not exit*/
	if (!chip->handshake_failed && !chip->exit) {
		pr_info("dchg not exit,not to reset\n");
		return 0;
	}


	if (chip->reset_retries > PCA9468_RESET_RETRIES_MAX) {
		pr_warn("reset_retries max reset_retries=%d\n", chip->reset_retries);
		return 0;
	}

	if (chip->reg_exception) {
		stm32l011_reset_pca9468(chip);
	}

	return 0;
}

static int stm32l011_update_cable_R(struct stm32l011_mcu *chip)
{
	uint16_t cable_R = 0;
	uint8_t ret, reg;
	uint8_t low = 0, high = 0;

	if (chip->dchg_version != NXP_PCA9486_DCHG) {
		//pr_warn("PCA9468,not to update cable r\n");
		return 0;
	}

	reg = ADAPTER_I2C_READ_COUNT;
	ret = stm32l011_read_reg(chip, reg, &low);
	ret = stm32l011_read_reg(chip, reg + 1, &high);
	chip->adapter_vbus = high << 8 | low;

	low = 0;
	high = 0;
	reg = ADAPTER_I2C_WRITE_COUNT;
	ret = stm32l011_read_reg(chip, reg, &low);
	ret = stm32l011_read_reg(chip, reg + 1, &high);
	chip->pca_vbus = high << 8 | low;

	chip->ibus_ma = stm32l011_get_ibusADC(chip);

	if (chip->ibus_ma > 0)
		cable_R = (chip->adapter_vbus - chip->pca_vbus) * 1000 / chip->ibus_ma;

	pr_warn("cable ibus_ma =%d adapter_vbus=%d pca_vbus=%d cable_R=%d dchg_status=%d\n",
			chip->ibus_ma, chip->adapter_vbus, chip->pca_vbus, cable_R, chip->dchg_status);

	return 0;
}

static int dchg_get_ap_req_ibat(struct stm32l011_mcu *chip)
{
	u8 value = 0;
	int vote_current = 0;
	int ibat = 0;

	vote_current = get_effective_result_locked(chip->dchg_fcc_votable);
	value = vote_current / 100;
	ibat = value * 100;
	pr_info("vote_current=%d,ibat=%d\n",  vote_current, ibat);
	return ibat;
}

#define CABLE_R_LIMIT_MAX_COUNT	5
static void dchg_config_current_by_cableR(struct stm32l011_mcu *chip)
{
	int rc = 0, i = 0;
	int cable_mohm = 0;
	int cable_id_type = 0;

	/*AT mode skip CableR detect*/
	if (get_atboot())
		return;

	/*nonsupport sbu cable id detect*/
	if (!chip->sbu_cable_id_detect_enable)
		return;

	/*sbu cable id unknown*/
	if (SBU_CABLE_ID_UNKNOW == chip->sbu_cable_id)
		return;

	cable_mohm = stm32l011_get_cable_mohm(chip);

	/* check effective cable id type */
	if (chip->sbu_cable_id == SBU_CABLE_ID_DEFAULT) {
		cable_id_type = 1;
	} else {
		cable_id_type = 0;
	}

	/*check cableR limit count*/
	for (i = 0; i < chip->cable_r_limit_table_rc[ROW]; i++) {
		if (is_between(chip->cable_r_limit_table[i].rmin, chip->cable_r_limit_table[i].rmax, cable_mohm) &&
				(cable_id_type == chip->cable_r_limit_table[i].cable_id_type)) {
			chip->cable_r_limit_table[i].limit_count++;

			pr_info("CableR=%dmO, cable_r_limit_table[%d]={%dmO,%dmO,%dmA,%d,%d}, sbu_cable_id=%d\n",
					cable_mohm, i, chip->cable_r_limit_table[i].rmin, chip->cable_r_limit_table[i].rmax, chip->cable_r_limit_table[i].dchg_ibus_max,
					chip->cable_r_limit_table[i].cable_id_type, chip->cable_r_limit_table[i].limit_count, chip->sbu_cable_id);
		} else {
			chip->cable_r_limit_table[i].limit_count = 0;
		}
	}

	/*get cableR limit current*/
	for (i = 0; i < chip->cable_r_limit_table_rc[ROW]; i++) {
		if (chip->cable_r_limit_table[i].limit_count >= CABLE_R_LIMIT_MAX_COUNT) {
			chip->cable_r_limit_table[i].limit_count = 0;
			chip->cable_r_current_limit_max = chip->cable_r_limit_table[i].dchg_ibus_max * 2;

			if (0 == chip->cable_r_current_limit_max) {
				chip->cable_r_disable_dchg = 1;
				pr_warn("exception: CableR(%d) too high,exit dchg\n", cable_mohm);
				stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				msleep(100);
				vote(chip->dchg_fcc_votable, CABLE_R_FCC_VOTER, false, 0);
				break;
			}

			rc = vote(chip->dchg_fcc_votable, CABLE_R_FCC_VOTER, true, chip->cable_r_current_limit_max);
			if (rc < 0)
				pr_err("Couldn't vote fastchg ma by cable rc = %d\n", rc);
			else
				pr_info("cable mohm=%d, current=%d\n", cable_mohm, chip->cable_r_current_limit_max);
			break;
		}
	}
}

static void dchg_config_current_by_cableID(struct stm32l011_mcu *chip)
{
	int ret;
	int dchg_ibus_limit = 3000;//default typec cable current limit in 3000mA
	int ibat_limit = dchg_ibus_limit * 2;

	/*AT mode skip CableR detect*/
	if (get_atboot())
		return;

	if (!chip->init_done || !chip->handshake_success) {
		return;
	}

	switch (chip->sbu_cable_id) {
	case SBU_CABLE_ID_4A:
		dchg_ibus_limit = 4000;
		break;
	case SBU_CABLE_ID_5A:
		dchg_ibus_limit = 5000;
		break;
	case SBU_CABLE_ID_6A:
		dchg_ibus_limit = 6000;
		break;
	case SBU_CABLE_ID_8A:
		dchg_ibus_limit = 8000;
		break;
	case SBU_CABLE_ID_DEFAULT:
		dchg_ibus_limit = 3000;/*default id 3a*/
		break;
	case SBU_CABLE_ID_UNKNOW:
	default:
		dchg_ibus_limit = 2700;/*change to 2700mA for support Other Vendor Cable*/
		break;

	}

	//ibat_limit = min(dchg_ibus_limit * 2, chip->total_dchg_limit_max_ma);
	ibat_limit = dchg_ibus_limit * 2;

	ret = vote(chip->dchg_fcc_votable, CABLE_ID_FCC_VOTER, true, ibat_limit);
	if (ret < 0)
		pr_err("Couldn't vote fastchg ma by cableID rc = %d\n", ret);

	pr_err("MCU Cable_id vote ibat_max=%dmA,sbu_cable_id=%d\n", ibat_limit, chip->sbu_cable_id);
}

static void stm32l011_check_dchg_fcc_vote(struct stm32l011_mcu *chip)
{
	uint8_t regvalue;
	int vote_value = 0;

	if (!chip->init_done || !chip->chg_enable) {
		return;
	}

	vote_value = get_effective_result_locked(chip->dchg_fcc_votable);
	if (vote_value > 0) {
		stm32l011_read_reg(chip, MCU_ADAPTER_CURRENT, &regvalue);
		if ((vote_value / 100) != regvalue) {
			pr_info("ibat vote mis-match, try again, vote_value=%d, mcu_ibat_value=%d\n",  vote_value, regvalue);
			stm32l011_set_current(chip, vote_value);
		}
	}
}

static void stm32l011_check_dchg_disable_vote(struct stm32l011_mcu *chip)
{
	int vote_value = 0;
	union power_supply_propval chg_time = {0,};

	power_supply_lite_get_property(PSYL_CHARGE, POWER_SUPPLY_PROP_CHARGE_TIME, &chg_time);
	if (!chip->init_done || chg_time.intval > 6) {
		return;
	}

	vote_value = get_effective_result_locked(chip->dchg_disable_votable);
	if (!vote_value != chip->chg_enable && (chip->factory_10W_charge_test_enabe == 0)) {
		pr_info("dchg_disable mis-match, try again. vote_value=%d, chg_enable=%d\n", vote_value, chip->chg_enable);
		chip->disable_vote_miss = true;
		rerun_election(chip->dchg_disable_votable);
	}
}

static void stm32l011_reset_voters(struct stm32l011_mcu *chip)
{
	if (chip->enable_slave_charger)
		vote(chip->dchg_fcc_votable, USER_FCC_VOTER, true, chip->dual_chg_cur_threshold);
	else
		vote(chip->dchg_fcc_votable, USER_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, STANDALONE_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, FACTORY_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, CABLE_R_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, CHG_EXCETION_FCC_VOTER, false, 0);
	vote(chip->dchg_fcc_votable, CABLE_ID_FCC_VOTER, false, 0);
	force_active_votable(chip->dchg_fcc_votable, FACTORY_FCC_VOTER, false, 0);

	vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, false, 0);
#if OLD_CAM_NOTIFY
	if (!chip->cam_running) {
		vote(chip->dchg_disable_votable, DCHG_CAM_EVENT, false, 0);
	}
#endif
}

static void dual_engine_chg_monitor(struct stm32l011_mcu *chip)
{
	u8 reg04 = 0, reg06 =0, mcu_current_value = 0;
	int ret = 0, rc = 0;
	int dual_chg_input_cur_threshold = chip->dual_chg_cur_threshold /2;
	int ap_req_ibat = 0;
	int mcu_req_ibat = 0;
	int delta_ibus = 0;
	static int delta_ibus_count = 0;

	ret = stm32l011_read_reg(chip, MCU_ADAPTER_CURRENT, &mcu_current_value);
	if (ret < 0) {
		pr_err("fail to read reg=%d\n", MCU_ADAPTER_CURRENT);
	}
	mcu_req_ibat = mcu_current_value * 100;
	ap_req_ibat = dchg_get_ap_req_ibat(chip);
	if (ap_req_ibat != mcu_req_ibat) {
		stm32l011_set_current(chip, ap_req_ibat);
	}
	pr_info("dual_chg_step=%d,chg_enable=%d,slave_chg_enable=%d,ap_req_ibat=%d,mcu_req_ibat=%d,ibus_ma=%d,factory_enable_flag=%d,in_standalone_mode=%d\n",
			chip->dual_chg_step, chip->chg_enable, chip->slave_chg_enable, ap_req_ibat, mcu_req_ibat, chip->ibus_ma, chip->at_mode_slave_chg_enable_req, chip->in_standalone_mode);
	switch(chip->dual_chg_step) {
	case HCHG_WAIT_MASTER_CHG:
		/* vote for master charger*/
		if (get_atboot())
			ret = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, true,
					chip->dchg_master_max_current);
		else
			ret = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, true,
					chip->dual_chg_cur_threshold);
		if (ret < 0)
			pr_err("Couldn't vote fastchg ma ret = %d\n", ret);

		/*get master charger status*/
		chip->chg_enable  = stm32l011_chg_is_enable(chip);
		if (chip->chg_enable) {
			if ((!chip->in_standalone_mode && !chip->slave_charger_should_disable)
					|| get_atboot())
				chip->dual_chg_step = HCHG_MASTER_CHG;
			else
				chip->dual_chg_step = HCHG_STANDALONE_MODE;
			/* if cable impedance is larger, goto standlone mode */
			//if (chip->cable_mohm > 400 || chip->both_charger_fault_counter >= 2)
			//	chip->dual_chg_step = HCHG_STANDALONE_MODE;
		}
		pr_info("cable_mohm=%d,both_charger_fault_counter=%d\n", chip->cable_mohm, chip->both_charger_fault_counter);
		break;
	case HCHG_MASTER_CHG:
		if (get_atboot() && !chip->at_mode_slave_chg_enable_req)
			break;
		/*get master charger status*/
		chip->chg_enable  = stm32l011_chg_is_enable(chip);
		if (chip->chg_enable) {
			if (((ap_req_ibat == mcu_req_ibat) && mcu_req_ibat >= chip->dual_chg_cur_threshold
						&& chip->ibus_ma >= (dual_chg_input_cur_threshold - 150)) || (get_atboot() && chip->ibus_ma > 1000))
				chip->dual_chg_step = HCHG_DUAL_MODE_REQUEST;
		}else {
			if (!(reg04 & MCU_DCHG_SWITCH)) {
				chip->both_charger_fault_counter++;
				chip->dual_chg_step = HCHG_EXIT;
			}
		}
		break;
	case HCHG_DUAL_MODE_REQUEST:
		/* try to enable slave charger */
		ret = stm32l011_read_reg(chip, MCU_REG_04, &reg04);
		if (ret< 0) {
			pr_err("fail to read reg=%d\n",MCU_REG_04);
		}
		if (!(reg04 & MCU_SET_ENABLE_SLAVE_CHG)) {
			ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_ENABLE_SLAVE_CHG, MCU_SET_ENABLE_SLAVE_CHG);
			if (ret)
				pr_err("Couldn't write reg_04 ret = %d\n", ret);
		}
		/* get enable state */
		ret = stm32l011_read_reg(chip, MCU_REG_06, &reg06);
		if (ret< 0)
			pr_err("fail to read reg=%d\n", MCU_REG_06);

		pr_info("reg04=0x%02x, reg06=0x%02x\n", reg04, reg06);
		if (reg06 & EVENT_SLAVE_CHG_ENABLE_FAIL
				|| reg06 & EVENT_SLAVE_CHG_ENABLE_SUCCESS) {
			/*get dual charger enable status*/
			chip->chg_enable  = stm32l011_chg_is_enable(chip);
			chip->slave_chg_enable  = stm32l011_slave_chg_is_enable(chip);
			pr_info("chg_enable=%d, slave_chg_enable=%d\n", chip->chg_enable, chip->slave_chg_enable);
			if (get_atboot())
				break;

			/*when dual charger enable sucess,enter into dual_mode */
			if (chip->chg_enable && chip->slave_chg_enable) {
				ret = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, false, 0);
				if (ret < 0) {
					pr_err("Couldn't vote fastchg ma ret = %d\n", ret);
				}
				chip->dual_chg_step = HCHG_DUAL_MODE;
			}else{
				/* master enable sucess, slave enable fail, go to standalone mode*/
				if (chip->chg_enable && !chip->slave_chg_enable) {
					chip->dual_chg_step = HCHG_STANDALONE_MODE;
					break;
				}
				/* dual charger enable fail */
				if (!chip->chg_enable && !chip->slave_chg_enable && !(reg04 & MCU_DCHG_SWITCH)) {
					chip->both_charger_fault_counter++;
					chip->dual_chg_step = HCHG_EXIT;
				}
			}
		}
		break;
	case HCHG_DUAL_MODE:
		/*get dual charger enable status*/
		chip->chg_enable  = stm32l011_chg_is_enable(chip);
		chip->slave_chg_enable  = stm32l011_slave_chg_is_enable(chip);
		/* master enable sucess, slave enable fail, go to standalone mode*/
		if (chip->chg_enable && !chip->slave_chg_enable) {
			chip->dual_chg_step = HCHG_STANDALONE_MODE;
			chip->slave_charger_fault_counter++;
			break;
		}
		/* dual charger enable fail */
		if (!chip->chg_enable && !chip->slave_chg_enable && !(reg04 & MCU_DCHG_SWITCH)) {
			chip->both_charger_fault_counter++;
			chip->dual_chg_step = HCHG_EXIT;
			pr_info("both charger exit charging\n");
			break;
		}
		/* master and slave ibus allocate control*/
		if (chip->chg_enable && chip->slave_chg_enable && chip->slave_ibus_ma > 200) {
			delta_ibus = abs(chip->ibus_ma - 2 * chip->slave_ibus_ma);
			pr_info("delta_ibus=%d\n", delta_ibus);
			if (delta_ibus > 2000 &&  !get_effective_result_locked(chip->dchg_disable_votable)) {
				if (delta_ibus_count++ >= 2) {
					delta_ibus_count = 2;
					vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, true, 0);
					chip->dual_chg_to_standalone_req = true;
					chip->master_slave_charger_current_miss_match = 2;
					chip->in_standalone_mode = true;
					chip->dual_chg_step = HCHG_EXIT;
					break;
				}
			} else if (delta_ibus >= 1000 &&  !get_effective_result_locked(chip->dchg_disable_votable)) {
				if (delta_ibus_count++ >= 2) {
					delta_ibus_count = 2;
					chip->master_slave_charger_current_miss_match = 1;
					rc = vote(chip->dchg_fcc_votable, CHG_EXCETION_FCC_VOTER, true, 6000);
					if (rc < 0)
						pr_err("Couldn't vote fastchg ma by dual chg diff value rc = %d\n", rc);
				}
			} else
				delta_ibus_count = 0;	//reset to zero..
		}
		/* if dual charging enter into cv mode, goto standalone mode*/
		if (chip->chg_enable && chip->slave_chg_enable
				&& (ap_req_ibat == mcu_req_ibat)
				&& chip->req_ibus <= dual_chg_input_cur_threshold
				&& chip->adjust_stop
				&& chip->ldo_count >=1
				&& !get_effective_result_locked(chip->dchg_disable_votable)) {
			pr_info("dchg exit in dual charing mode in cv mode\n");
			vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, true, 0);
			chip->dual_chg_to_standalone_req = true;
			chip->in_standalone_mode = true;
			chip->dual_chg_step = HCHG_EXIT;
			break;
		}

		ap_req_ibat = dchg_get_ap_req_ibat(chip);
		pr_info("in_standalone_mode=%d ap_req_ibat=%d\n", chip->in_standalone_mode, ap_req_ibat);
		/* if request current is lower, exit hchg*/
		if (chip->chg_enable
				&& chip->slave_chg_enable
				&& ap_req_ibat >= chip->low_current_exit_threshold
				&& ap_req_ibat < chip->dual_chg_cur_threshold
				&& !(reg04 & MCU_DCHG_SWITCH)) {
			if (!get_effective_result_locked(chip->dchg_disable_votable)) {
				vote(chip->dchg_disable_votable, DCHG_RESTART_EVENT, true, 0);
				chip->set_cur_too_low_for_dual_chg = true;
				chip->in_standalone_mode = false;
				chip->dual_chg_step = HCHG_EXIT;
				pr_info("user set low current exit in dual charing mode\n");
			}
		}
		break;
	case HCHG_STANDALONE_MODE:
		/*get dual charger enable status*/
		chip->chg_enable  = stm32l011_chg_is_enable(chip);
		if (!chip->chg_enable) {
			chip->dual_chg_step = HCHG_EXIT;
			pr_info("master charger exit charging\n");
			break;
		}
		/* vote  current in standlone mode */
		ret = vote(chip->dchg_fcc_votable, STANDALONE_FCC_VOTER, true,
				chip->dchg_master_max_current);
		if (ret < 0) {
			pr_err("Couldn't vote fastchg ma ret = %d\n", ret);
		}
		/* disable MASTER_FCC_VOTER*/
		ret = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, false,
				0);
		if (ret < 0) {
			pr_err("Couldn't vote fastchg ma ret = %d\n", ret);
		}
		break;
	case HCHG_EXIT:
		/* MASTER_FCC_VOTER*/
		ret = vote(chip->dchg_fcc_votable, MASTER_FCC_VOTER, true,
				chip->dual_chg_cur_threshold);
		if (ret < 0) {
			pr_err("Couldn't vote fastchg ma ret = %d\n", ret);
		}
		break;
	default:
		pr_info("cmd not support\n");
		break;
	}

}
#define SECOND_CHECK_DELAY	1000
static void stm32l011_chg_monitor_work(struct work_struct *work)
{
	struct stm32l011_mcu *chip = container_of(work,
			struct stm32l011_mcu, chg_monitor_work.work);
	union power_supply_propval usb_type = {0,};
	union power_supply_propval usb_in = {0,};
	union power_supply_propval batt_soc = {0,};
	union power_supply_propval cms_health_now = {0,};
	int rc = 0, enable = 0;
	bool pmi_suspend = 0, inited = 0, exit = 0, handshake_success = 0;
	bool vbat_good = 0, handshake_failed = 0, cable_matched_event = 0, high_vol_mode_event = 0;
	u8 reg = 0, value = 0, coefficient = 0;
	static u8 value_irqA, value_irqB, value_irqC, value_irqD, value_04, value_05;
	bool dpdm_exception = false, bat_exception = false, vbat_high = false, ibat_high = false;
	static int dpdm_exception_count = 0;
	static int bat_exception_count = 0;
	static int vbat_exception_count = 0;
	static int ibat_exception_count = 0;
	static int dchg_error_time_count = 0;
	static int bq_die_temp_error_count = 0;
	//static int usb_conn_temp_error_count = 0;
	static int bat_temp_error_count = 0;
	static int master_bat_conn_temp_error_count = 0;
	static bool last_chg_enable = false;
	bool polling_quickly = true;

	if (chip->suspend) {
		pr_err("suspend stop monitor work\n");
		return;
	}

	if (!power_supply_lite_is_ready()) {
		pr_err("psyl not ready,return\n");
		return;
	}

	if (chip->burn_handling || chip->dchg_pon_reset) {
		pr_warn("burn_handling\n");
		goto restart;
	}

	if (chip->time_count % 2 == 0) {
		power_supply_lite_get_property(PSYL_CHARGE,
				POWER_SUPPLY_PROP_CHARGE_DETECT, &usb_in);
		chip->usb_present = usb_in.intval;
		pr_info("usb_present=%d\n", chip->usb_present);
	}

	if (!chip->usb_present) {
		pr_warn("!usb_present\n");
		stm32l011_hw_deinit(chip);
		return;
	}
#if OLD_CAM_NOTIFY
	if (chip->cam_running && chip->cable_matched_event && chip->mcu_main_chg_sm > CHARGER_IDENTIFICATION) {
		pr_warn("cam is running & cable matched then return monitor!!!\n");
		return;
	}
#endif
	if (chip->dchg_start_time != 0) {
		chip->dchg_continue_time = (jiffies - chip->dchg_start_time) / HZ;
	}

	/*get cable id value*/
	if (chip->init_done && chip->handshake_success && chip->usb_present) {
		if (SBU_CABLE_ID_UNKNOW == chip->sbu_cable_id && !chip->sbu_detect_busy)
			stm32l011_sbu_cable_id_detect(chip);
	}

	/*polling event from mcu*/
	if (chip->time_count % 1 == 0) {
		rc = stm32l011_read_reg(chip, MCU_IRQ_A_REG, &value);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_IRQ_A_REG);
			goto restart;
		}
		value_irqA = value;

		/*polling  inited event*/
		inited = !!(value & MCU_IRA_INIT_DONE);
		if (chip->init_done != inited && inited) {
			pr_info("init done reg=0x%02X value=0x%02X\n",MCU_IRQ_A_REG,value);
			if (0xFF == value) {
				pr_err("irqa value error, skip for next check!!! \n");
				goto restart;
			}

			stm32l011_init_done_handler(chip,inited);
			chip->init_done = inited;
		}
		if (!chip->init_done) {
			pr_info("mcu not inited\n");
			goto restart;
		}

		/*check dpdm  before handshake*/
		if (!chip->handshake_success && !chip->handshake_failed) {
			dpdm_exception = stm32l011_check_dpdm_status(chip);
			if (dpdm_exception) {
				if (++dpdm_exception_count > 3) {
					pr_info("exception: dpdm status,not entry dchg\n");
					dpdm_exception_count = 0;
					return;
				}
				goto restart;
			} else {
				dpdm_exception_count = 0;
			}

			bat_exception = stm32l011_check_battery_status(chip);
			if (bat_exception) {
				if (++bat_exception_count > 3) {
					pr_info("exception: bat status,not entry dchg\n");
					bat_exception_count = 0;
					return;
				}
				goto restart;
			} else {
				bat_exception_count = 0;
			}
		}

		stm32l011_AP_MCU_main_sm(chip);

		if (!chip->handshake_success && !chip->handshake_failed) {
			/*switch usb  and start handshake*/
			power_supply_lite_get_property(PSYL_CHARGE,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &usb_type);
			chip->usb_type = usb_type.intval;
			power_supply_lite_get_property(PSYL_CMS,
					POWER_SUPPLY_PROP_HEALTH_STATUS, &cms_health_now);
			enable = stm32l011_get_usbsel_state(chip);
			pr_info("vbus_exception=%d, dp_dm_ovp=%d, usb_type=%d", chip->vbus_exception, chip->dp_dm_ovp, usb_type.intval);
			if (usb_type.intval == POWER_SUPPLY_TYPE_USB_DCP && !enable &&
					!chip->vbus_exception && !(cms_health_now.intval & 0x4000) && !chip->dp_dm_ovp) {
				pr_info("enable usbsel\n");
				stm32l011_usbsel_enable(chip,true);
			}
			stm32l011_check_vbus_status(chip);
		}

		/*polling  handshake success event*/
		handshake_success = !!(value & MCU_IRA_HANDSHAKE_SUCCESS);
		if (chip->handshake_success != handshake_success && handshake_success) {
			pr_info("handshake_success reg=0x%02X,value=0x%02X\n", MCU_IRQ_A_REG, value);
			stm32l011_handshake_success_handler(chip,handshake_success);
			chip->handshake_success = handshake_success;
		}
		/*polling  handshake failed event*/
		handshake_failed = !!(value & EVENT_HANDSHAKE_FAIL);
		if (chip->handshake_failed != handshake_failed && handshake_failed) {
			pr_info("handshake_failed reg=0x%02X,value=0x%02X\n",MCU_IRQ_A_REG,value);
			stm32l011_handshake_failed_handler(chip,handshake_failed);
			chip->handshake_failed = handshake_failed;
		}

		/*polling  adapter power matched / cable matched event*/
		cable_matched_event = !!(value & MCU_IRA_POWER_MATCHED);
		if (chip->cable_matched_event != cable_matched_event && cable_matched_event) {
			pr_info("adapter_power_matched_event reg=0x%02X,value=0x%02X\n", MCU_IRQ_A_REG, value);
			stm32l011_cable_matched_handler(chip, cable_matched_event);
			chip->cable_matched_event = cable_matched_event;
		}

		/*polling adapter high voltage mode request event*/
		high_vol_mode_event = !!(value & MCU_IRA_HIGH_VOL_REQUEST_SUCCESS);
		if (chip->high_vol_mode_event != high_vol_mode_event && high_vol_mode_event) {
			pr_info("adapter switch to high vol mode sucess reg=0x%02X,value=0x%02X\n", MCU_IRQ_A_REG, value);
			chip->high_vol_mode_event = high_vol_mode_event;
		}

		/*polling  vbat_good event*/
		vbat_good = !!(value & MCU_IRA_VBAT_GOOD);
		if (chip->vbat_good != vbat_good && vbat_good) {
			pr_info("vbat_good reg=0x%02X,value=0x%02X\n",MCU_IRQ_A_REG,value);
			stm32l011_vbat_good_handler(chip,vbat_good);
			chip->vbat_good = vbat_good;
		}
		/*polling  pmi suspend event*/
		pmi_suspend = !!(value & MCU_IRA_PMI_SUSPEND);
#if 0 //TEST_CYCLE_CHARGE_DISCHARGE
		if (!chip->chg_switch)
			pmi_suspend = true;
#endif
		if (chip->pmi_suspend != pmi_suspend && pmi_suspend) {
			pr_info("pmi_suspend reg=0x%02X,value=0x%02X\n",MCU_IRQ_A_REG,value);
			stm32l011_pmi_suspend_handler(chip,pmi_suspend);
			chip->pmi_suspend = pmi_suspend;
		}

		if (chip->pmi_suspend) {
			/*kick watchdog,iic read error not kick*/
			stm32l011_write_reg(chip,MCU_REG_07,0x01);

			/*monitor vbat by FG/BQ27546,too high exit dchg*/
			vbat_high = stm32l011_check_vbat_high(chip);
			if (vbat_high) {
				if (++vbat_exception_count > 3) {
					vbat_exception_count = 0;
					pr_warn("exception: vbat too high,exit dchg\n");
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				vbat_exception_count = 0;
			}
			/*monitor ibat by BQ27546,too high exit dchg*/
			ibat_high = stm32l011_check_ibat_high(chip);
			if (ibat_high) {
				if (++ibat_exception_count > 3) {
					ibat_exception_count = 0;
					pr_warn("exception: ibat too high,exit dchg\n");
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				ibat_exception_count = 0;
			}

			/*monitor  BQ25970 Die temp(default:1200) too high ,exit dchg*/
			if (chip->chg_enable && chip->bq_die_temp > chip->dchg_exit_ntc_threshold[DENT_BQ_DIE_TEMP]) {
				if (++bq_die_temp_error_count > 3) {
					bq_die_temp_error_count = 0;
					pr_warn("exception: BQ25970 Die temp too high(%d),exit dchg\n", chip->bq_die_temp);
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				bq_die_temp_error_count = 0;
			}

#if 0 /*monitor usb conn temp(default:650) too high, exit dchg*/
			/* monitor ibat by USB conn temp too high exit dchg */
			if (chip->chg_enable && (chip->usb_conn_ntc_connect_to != USB_CONN_NTC_CONNECT_TO_DCHG_IC) &&
					chip->usb_conn_temp > chip->dchg_exit_ntc_threshold[DENT_USB_CONN_TEMP]) {
				if (++usb_conn_temp_error_count > 3) {
					usb_conn_temp_error_count = 0;
					pr_warn("USB conn temp too high(%d),exit dchg\n", chip->usb_conn_temp);
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				usb_conn_temp_error_count = 0;
			}
#endif

			/*monitor bat board temp(default:650) too high, exit dchg*/
			if (chip->chg_enable && chip->bat_temp > chip->dchg_exit_ntc_threshold[DENT_BAT_BOARD_TEMP]) {
				if (++bat_temp_error_count > 3) {
					bat_temp_error_count = 0;
					pr_warn("exception: bat board temp too high(%d),exit dchg\n", chip->bat_temp);
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				bat_temp_error_count = 0;
			}

			/*monitor master bat conn temp(default:700) too high, exit dchg*/
			if (chip->chg_enable && chip->master_bat_conn_temp > chip->dchg_exit_ntc_threshold[DENT_MASTER_BAT_CONN_TEMP]) {
				if (++master_bat_conn_temp_error_count > 3) {
					master_bat_conn_temp_error_count = 0;
					pr_warn("exception: master bat conn temp too high(%d),exit dchg\n", chip->master_bat_conn_temp);
					stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
				}
			} else {
				master_bat_conn_temp_error_count = 0;
			}

#if 0
			if (chip->ex_fg_i2c_error_counter > 20 && !get_atboot()) {
				pr_warn("exception: external fuel gauge exception(%d),exit dchg\n", chip->ex_fg_i2c_error_counter);
				stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
			}
#endif
		}

		/*polling  exit event*/
		rc = stm32l011_read_reg(chip, MCU_IRQ_B_REG, &value);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_IRQ_A_REG);
			goto restart;
		}

		value_irqB = value;
		exit = !!(value & MCU_IRB_DCHG_EXIT);

#if TEST_CYCLE_CHARGE_DISCHARGE
		rc = stm32l011_read_reg(chip, MCU_REG_04, &value_04);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_REG_04);
		}

		if (!(value_04 & MCU_DCHG_SWITCH) && !chip->chg_switch) {

			if (!(value_04 & MCU_DCHG_SWITCH)) {
				pr_warn("exception: user request to set_chg_switch(false), exit dchg\n");
				stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
			}

			rc = stm32l011_read_reg(chip, MCU_REG_04, &value_04);
			if (rc < 0) {
				pr_err("fail to read reg=%d\n",MCU_REG_04);
			}
		}
		if (chip->enable_slave_charger) {
			power_supply_lite_get_property(PSYL_BATTERY,
					POWER_SUPPLY_PROP_CAPACITY, &batt_soc);
			chip->batt_capacity = batt_soc.intval;
		}

		if (chip->dchg_version == LN8000_DCHG) {
			if ((value_irqB & EXIT_HIGH_VBAT) || (value_04 & MCU_DCHG_SWITCH) ||
					(chip->int_stat_ln8000_05 & 0x40 && (batt_soc.intval > 90)) ||
					chip->dual_chg_to_standalone_req || chip->chg_enable) {
				chip->exit_event = 1;//dchg normal ongoing  or  dchg normal exit.
				dchg_error_time_count = 0;
			} else if (dchg_error_time_count++ >= 4) {
				dchg_error_time_count = 4;
				chip->exit_event = 0;
			}
		} else {
			if ((value_irqB & EXIT_HIGH_VBAT) || (value_04 & MCU_DCHG_SWITCH) ||
					(chip->int_stat_11 & 0x80 && (batt_soc.intval > 90)) ||
					chip->dual_chg_to_standalone_req || chip->chg_enable) {
				chip->exit_event = 1;//dchg normal ongoing  or  dchg normal exit.
				dchg_error_time_count = 0;
			} else if (dchg_error_time_count++ >= 4) {
				dchg_error_time_count = 4;
				chip->exit_event = 0;
			}
		}

		// if (!chip->chg_switch)
		//     exit = true;
#endif
		if (chip->exit != exit && exit) {
			pr_info("exit reg=0x%02X,value=0x%02X\n",MCU_IRQ_B_REG,value);
			stm32l011_dchg_exit_handler(chip,exit);
			chip->exit = exit;
		}

		stm32l011_check_usb_present(chip);
		if (!chip->usb_present) {
			pr_warn("!usb_present\n");
			stm32l011_hw_deinit(chip);
			return;
		}
		chip->irqA = value_irqA;
		chip->irqB = value_irqB;
		stm32l011_update_cable_R(chip);
	}

	/*engineer mode parameters and log updated*/
	chip->usbsel_state = stm32l011_get_usbsel_state(chip);
	polling_quickly = (chip->usbsel_state || chip->handshake_success || !chip->handshake_failed || chip->chg_enable);
	if ((chip->time_count % 2 == 0 && polling_quickly) ||
			(chip->time_count % 31 == 0 && !polling_quickly)) {
		rc = stm32l011_read_reg(chip, MCU_IRQ_C_REG, &value_irqC);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_IRQ_C_REG);
		}

		rc = stm32l011_read_reg(chip, MCU_IRQ_D_REG, &value_irqD);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_IRQ_D_REG);
		}

		rc = stm32l011_read_reg(chip, MCU_REG_04, &value_04);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n",MCU_REG_04);
		}
		rc = stm32l011_read_reg(chip, MCU_REG_05, &value_05);
		if (rc < 0) {
			pr_err("fail to read reg=%d\n", MCU_REG_05);
		}
		chip->ibus_ma = stm32l011_get_ibusADC(chip);
		if (chip->enable_slave_charger)
			chip->slave_ibus_ma = stm32l011_get_slave_ibusADC(chip);
		chip->vbus_mv = stm32l011_get_vbusADC(chip);
		chip->vbat_mv = stm32l011_get_vbatADC(chip);
		chip->cable_mohm = stm32l011_get_cable_mohm(chip);
		chip->req_vbus = stm32l011_get_req_vbus(chip);
		chip->chg_enable = stm32l011_chg_is_enable(chip);
		if (!chip->handshake_success && !chip->handshake_failed) {
			chip->bat_temp = -10;
			chip->bat_conn_temp = -10;
			chip->master_bat_conn_temp = -10;
			chip->usb_conn_temp = -10;
			chip->adapter_temp = -10;
			chip->adapter_conn_temp = -10;
			chip->bq_die_temp = -10;
		} else {
			chip->bat_temp = stm32l011_get_bat_tempADC(chip);
			chip->bat_conn_temp = stm32l011_get_bat_conn_temp(chip);
			chip->master_bat_conn_temp = stm32l011_get_master_bat_conn_temp(chip);
			chip->usb_conn_temp = stm32l011_get_usb_conn_temp(chip);
			chip->adapter_temp = stm32l011_get_adapter_temp(chip);
			chip->adapter_conn_temp = stm32l011_get_adapter_conn_temp(chip);
			chip->bq_die_temp = stm32l011_get_bq_die_temp(chip);
		}
		chip->dp_mv = stm32l011_get_dp_mv(chip);
		chip->dm_mv = stm32l011_get_dm_mv(chip);
		//chip->cable_matched = stm32l011_get_cable_matched(chip);
		if (chip->enable_slave_charger)
			chip->slave_chg_enable = stm32l011_slave_chg_is_enable(chip);
		chip->req_ibus = stm32l011_get_reqIbus(chip);
		chip->ldo_count = stm32l011_get_ldo_count(chip);
		//chip->ptmValue = stm32l011_get_ptmValue(chip);
		chip->adjust_stop = stm32l011_get_adjust_stop(chip);
		chip->is_ldo = stm32l011_get_is_ldo(chip);
		stm32l011_get_handshakeid(chip);
		stm32l011_get_int_status_flag(chip);
		chip->irqA = value_irqA;
		chip->irqB = value_irqB;
		chip->irqC = value_irqC;
		chip->irqD = value_irqD;
		chip->reg04 = value_04;
		chip->reg05 = value_05;
		stm32l011_check_dchg_error_state(chip);
		if (chip->dchg_version != LN8000_DCHG) {
			pr_info("irqA:0x%02X irqB:0x%02X irqC:0x%02X irqD:0x%02X reg04:0x%02X reg05:0x%02X(0x%02X) "
					"ibus:%d req_ibus:%d req_vbus:%d vbus:%d vbat:%d cable:%d batTem:%d "
					"batConnTem:%d masterBatConnTem:%d usbConnTem:%d adapterTem:%d adapterConnTem:%d BqDieTemp:%d "
					"dp:%d dm:%d dpdmovp:%d cableID:%d chgEnable:%d ptmValue:%d adjust_stop:%d is_ldo:%d "
					"ldo_count:%d 08:0x%02X 0A:0x%02X 0B:0x%02X 0E:0x%02X 11:0x%02X 2D:0x%02X "
					"dchg_status:%d handshakeid:[0x%02X 0x%02X 0x%02X 0x%02X] dchg_state:[%s] cable_id:%s[%d,%d] "
					"IcableR=%d time_count:%ld(%ds) exit_event:%d(%d) usbsel_state=%d cout_count=%d\n",
					value_irqA, value_irqB, value_irqC, value_irqD, value_04, value_05, chip->main_chg_sm_error_in_ap_side, chip->ibus_ma,
					chip->req_ibus, chip->req_vbus, chip->vbus_mv, chip->vbat_mv, chip->cable_mohm,
					chip->bat_temp, chip->bat_conn_temp, chip->master_bat_conn_temp, chip->usb_conn_temp,
					chip->adapter_temp, chip->adapter_conn_temp, chip->bq_die_temp, chip->dp_mv, chip->dm_mv, chip->dp_dm_ovp,
					chip->cable_matched, chip->chg_enable, chip->ptmValue, chip->adjust_stop,
					chip->is_ldo, chip->ldo_count, chip->int_stat_08, chip->int_stat_0A,
					chip->int_stat_0B, chip->int_stat_0E, chip->int_stat_11, chip->int_stat_2D,
					chip->dchg_status, chip->handshakeid[0], chip->handshakeid[1],
					chip->handshakeid[2], chip->handshakeid[3], chip->dchg_state, sbu_cable_id_type_strings[chip->sbu_cable_id],
					chip->sbu1_adc, chip->sbu2_adc, chip->cable_r_current_limit_max, chip->time_count, chip->dchg_continue_time,
					chip->exit_event, dchg_error_time_count, chip->usbsel_state, chip->battery_cout_counter);
		} else {
			pr_info("irqA:0x%02X irqB:0x%02X irqC:0x%02X irqD:0x%02X reg04:0x%02X reg05:0x%02X(0x%02X) "
					"ibus:%d req_ibus:%d req_vbus:%d vbus:%d vbat:%d cable:%d batTem:%d "
					"batConnTem:%d masterBatConnTem:%d usbConnTem:%d adapterTem:%d adapterConnTem:%d BqDieTemp:%d "
					"dp:%d dm:%d dpdmovp:%d cableID:%d chgEnable:%d ptmValue:%d adjust_stop:%d is_ldo:%d "
					"ldo_count:%d 01:0x%02X 03:0x%02X 04:0x%02X 05:0x%02X 06:0x%02X 08:0x%02X "
					"dchg_status:%d handshakeid:[0x%02X 0x%02X 0x%02X 0x%02X] dchg_state:[%s] cable_id:%s[%d,%d] "
					"IcableR=%d time_count:%ld(%lds) exit_event:%d(%d) usbsel_state=%d cout_count=%d\n",
					value_irqA, value_irqB, value_irqC, value_irqD, value_04, value_05, chip->main_chg_sm_error_in_ap_side, chip->ibus_ma,
					chip->req_ibus, chip->req_vbus, chip->vbus_mv, chip->vbat_mv, chip->cable_mohm,
					chip->bat_temp, chip->bat_conn_temp, chip->master_bat_conn_temp, chip->usb_conn_temp,
					chip->adapter_temp, chip->adapter_conn_temp, chip->bq_die_temp, chip->dp_mv, chip->dm_mv, chip->dp_dm_ovp,
					chip->cable_matched, chip->chg_enable, chip->ptmValue, chip->adjust_stop,
					chip->is_ldo, chip->ldo_count, chip->int_stat_ln8000_01, chip->int_stat_ln8000_03,
					chip->int_stat_ln8000_04, chip->int_stat_ln8000_05, chip->int_stat_ln8000_06, chip->int_stat_ln8000_08,
					chip->dchg_status, chip->handshakeid[0], chip->handshakeid[1],
					chip->handshakeid[2], chip->handshakeid[3], chip->dchg_state, sbu_cable_id_type_strings[chip->sbu_cable_id],
					chip->sbu1_adc, chip->sbu2_adc, chip->cable_r_current_limit_max, chip->time_count, chip->dchg_continue_time,
					chip->exit_event, dchg_error_time_count, chip->usbsel_state, chip->battery_cout_counter);
		}
		if (chip->enable_slave_charger)
			pr_info("slave_ibus_ma:%d, slave_chg_enable=%d, 08:0x%02X 0A:0x%02X 0E:0x%02X 11:0x%02X 2D:0x%02X\n",
					chip->slave_ibus_ma, chip->slave_chg_enable, chip->slave_int_stat_08, chip->slave_int_stat_0A,
					chip->slave_int_stat_0E, chip->slave_int_stat_11, chip->slave_int_stat_2D);

		if (chip->usb_present && chip->chg_update) {
			fuelsummary_collect_value(ID_BIT__FCHG, chip->chg_enable);
			fuelsummary_collect_value(ID_BYTE__IRQA, value_irqA);
			fuelsummary_collect_value(ID_BYTE__IRQB, value_irqB);
			fuelsummary_collect_value(ID_BYTE__IRQC, value_irqC);
			fuelsummary_collect_value(ID_BYTE__IRQD, value_irqD);
			fuelsummary_collect_value(ID_BYTE__IRQ4, value_04);
			fuelsummary_collect_value(ID_BYTE__IRQ5, value_05 | chip->main_chg_sm_error_in_ap_side);
			fuelsummary_collect_value(ID_COMB__FC_IBUS, chip->ibus_ma);
			fuelsummary_collect_value(ID_COMB__FC_REQ_IBUS, chip->req_ibus);
			fuelsummary_collect_value(ID_COMB__FC_VBUS, chip->vbus_mv);
			fuelsummary_collect_value(ID_COMB__FC_VBAT, chip->vbat_mv);
			fuelsummary_collect_value(ID_COMB__FC_CAB_MOHM, chip->cable_mohm);
			fuelsummary_collect_value(ID_BYTE__CAB_ID, chip->sbu_cable_id);
			fuelsummary_collect_value(ID_COMB__FC_TADPT, chip->adapter_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_TADPT_CONN, chip->adapter_conn_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_TUSB, chip->usb_conn_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_TDIE, chip->bq_die_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_TBAT_CONN, chip->bat_conn_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_MTBAT_CONN, chip->master_bat_conn_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_TBAT_BOARD, chip->bat_temp / 10);
			fuelsummary_collect_value(ID_COMB__FC_VDP, chip->dp_mv);
			fuelsummary_collect_value(ID_COMB__FC_VDM, chip->dm_mv);

			fuelsummary_collect_value(ID_BYTE__FC_IMBALANCE, chip->master_slave_charger_current_miss_match);
			fuelsummary_collect_value(ID_BYTE__FC_BOTH_FAULT, chip->both_charger_fault_counter);
			fuelsummary_collect_value(ID_BYTE__FC_SLAVE_FAULT, chip->slave_charger_fault_counter);

			if (chip->handshake_success && chip->vbat_mv > 0 && chip->vbat_mv < 1000)
				fuelsummary_collect_value(ID_BIT__M_FAULT, 1);
			else
				fuelsummary_collect_value(ID_BIT__M_FAULT, 0);
		}
		if (!chip->chg_enable && last_chg_enable)
			chip->chg_update = false;
		else if (chip->chg_enable)
			chip->chg_update = true;

		last_chg_enable = chip->chg_enable;
	}

	if (chip->time_count % 5 == 0) {
		stm32l011_check_dchg_fcc_vote(chip);
		stm32l011_check_dchg_disable_vote(chip);

		chip->analog_i2c_read_exception = stm32l011_get_mcu_exception_count(chip,ANALOG_I2C_READ_COUNT);
		chip->analog_i2c_write_exception = stm32l011_get_mcu_exception_count(chip,ANALOG_I2C_WRITE_COUNT);
		chip->uart_tx_exception = stm32l011_get_mcu_exception_count(chip,UART_TX_COUNT);
		chip->uart_rx_exception = stm32l011_get_mcu_exception_count(chip,UART_RX_COUNT);
		chip->adapter_i2c_read_exception = stm32l011_get_mcu_exception_count(chip,ADAPTER_I2C_READ_COUNT);
		chip->adapter_i2c_write_exception = stm32l011_get_mcu_exception_count(chip,ADAPTER_I2C_WRITE_COUNT);
		chip->adapter_uart_tx_exception = stm32l011_get_mcu_exception_count(chip,ADAPTER_UART_TX_COUNT);
		chip->adapter_uart_rx_exception = stm32l011_get_mcu_exception_count(chip,ADAPTER_UART_RX_COUNT);
		chip->adapter_status = stm32l011_get_adapter_status(chip);
		pr_err("mcu expception monitor=%d,%d,%d,%d,%d,%d,%d,%d,0x%X,%d,%s | fw=0x%x, burn_result=%d, bl=%d, adapter_power=(0x%02x, 0x%02x, %d), VFC=%d,\n",
				chip->analog_i2c_read_exception, chip->analog_i2c_write_exception, chip->uart_tx_exception, chip->uart_rx_exception, chip->adapter_i2c_read_exception, chip->adapter_i2c_write_exception,
				chip->adapter_uart_tx_exception, chip->adapter_uart_rx_exception, chip->adapter_status, chip->adapter_mcu_version, adapter_vendor[chip->adapter_vendor], chip->mcu_current_fw,
				chip->inside_burn_result, chip->mcu_bl_vendor, chip->adapter_power, chip->adapter_powerderate, chip->adapter_powerderate_ready, chip->vivo_flash_charge_status);

		if (chip->usb_present) {
			fuelsummary_collect_value(ID_BYTE__ADPT_STAT, chip->adapter_status);
			//fuelsummary_collect_value(ID_ADPT_I2C_R, chip->adapter_i2c_read_exception);
			//fuelsummary_collect_value(ID_ADPT_I2C_W, chip->adapter_i2c_write_exception);
			//fuelsummary_collect_value(ID_ADPT_U_T, chip->adapter_uart_tx_exception);
			//fuelsummary_collect_value(ID_ADPT_U_R, chip->adapter_uart_rx_exception);
		}
	}

	if (!chip->chg_enable && chip->handshake_success)
	{
		chip->chg_enable = stm32l011_chg_is_enable(chip);
		//rc = stm32l011_read_reg(chip, 0x0C+MCU_BQ25970_REGS2_START, &reg);
		//pr_info(" BQ25970_reg=0x%02X,value=0x%02X;  chg_enable=%d.\n",0x0C,reg,chip->chg_enable);
	}

	if (chip->time_count % 2 == 0 && !chip->mcu_current_fw) {
		stm32l011_read_reg(chip, MCU_VERSION_END, &reg);
		chip->mcu_current_fw = reg;
	}

	if ((chip->time_count % 4 == 0 && get_atboot()) ||
			(chip->time_count % 10 == 0 && !get_atboot())) {
		get_votable_info(chip->dchg_disable_votable);
		get_votable_info(chip->dchg_fcc_votable);
		stm32l011_dump_regs(chip);
	}

	if (get_atboot() || chip->factory_mode_state) {
		stm32l011_masked_write(chip, MCU_REG_08, MCU_SET_FACTORY_MODE, MCU_SET_FACTORY_MODE);
		coefficient = 1;
	} else {
		coefficient = 2;
	}

	pr_info("factory_mode_state=%d,enable_slave_charger=%d\n", chip->factory_mode_state, chip->enable_slave_charger);

	if (chip->time_count % coefficient == 0) {
		dchg_config_current_by_cableR(chip);
		if (chip->enable_slave_charger) {
			dual_engine_chg_monitor(chip);
		}
		dchg_config_current_by_cableID(chip);
		/* add for ffc */
		if (chip->ex_fg_ffc_support) {
			stm32l011_check_cout_status(chip);
			stm32l011_adjust_dchg_ffc_cv_threshold(chip);
		}

		if (get_effective_result_locked(chip->dchg_disable_votable) && chip->chg_enable) {
			pr_err("dchg_disable_votable mis-match, try to vote again\n");
			chip->disable_vote_miss = true;
			rerun_election(chip->dchg_disable_votable);
		}

		stm32l011_check_dchg_restart(chip);
		stm32l011_check_pca9468_reset(chip);
	}

	if (chip->time_count++ > 36000) {
		chip->time_count = 0;
	}
restart:
	if (chip->report_big_data) {
		chip->report_big_data = false;
	}
	cancel_delayed_work(&chip->chg_monitor_work);
	schedule_delayed_work(&chip->chg_monitor_work,
			msecs_to_jiffies(SECOND_CHECK_DELAY));

}
#if OLD_CAM_NOTIFY
static void stm32l011_cam_exit_work(struct work_struct *work)
{
	struct stm32l011_mcu *chip = container_of(work, struct stm32l011_mcu, cam_exit_work.work);

	chip->cam_running = false;
	chip->cam_exit_restore = true;
	vote(chip->dchg_disable_votable, DCHG_CAM_EVENT, false, 0);
	pr_info("cam real stop\n");
}

void cam_notify_status_to_charge(bool runing)
{
	bool system_start = (((u64)(local_clock()/1000000000)) >= 32);

	if (this_chip && system_start) {
		if (runing) {
			pr_info("cam running notify!\n");
			cancel_delayed_work(&this_chip->cam_exit_work);
			this_chip->cam_running = true;
			vote(this_chip->dchg_disable_votable, DCHG_CAM_EVENT, true, 0);
		} else {
			if (this_chip->cam_running) {
				cancel_delayed_work(&this_chip->cam_exit_work);
				schedule_delayed_work(&this_chip->cam_exit_work, msecs_to_jiffies(this_chip->cam_exit_second * 1000));
				pr_info("cam will stop notify!\n");
			} else {
				pr_info("cam not runing, don't need stop!\n");
			}
		}
	} else {
		pr_info("driver init or not ready\n");
	}
}
EXPORT_SYMBOL(cam_notify_status_to_charge);
#endif

#ifdef CONFIG_DEBUG_FS
#define LAST_CNFG_REG	0x16
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x34
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x36
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
					handlers[i].irq_info[j].name,
					handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low,
					handlers[i].irq_info[j].high,
					handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#if 0
static int get_reg(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = data;
	int rc;
	u8 temp;

	rc = stm32l011_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
				chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct stm32l011_mcu *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = stm32l011_write_reg(chip, chip->peek_poke_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
				temp, chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");
#endif
static int force_irq_set(void *data, u64 val)
{
	struct stm32l011_mcu *chip = data;

	stm32l011_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

#ifdef TEST_BY_FXI_FOR_BURN
static int set_power_usbsel(void *data, u64 val)
{
	int ret = 0;
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;


	if (1 == val) {
		pr_info("setting usb psy dp=f dm=f\n");
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_HIGH);
		stm32l011_usbsel_enable(chip, 1);
	} else {
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
		stm32l011_usbsel_enable(chip, 0);
		pr_info("setting usb psy dp=r dm=r\n");
	}

	return ret;
}

static int get_power_usbsel(void *data, u64 *val)
{
	int ret = 0, power_state = 0, usbsel_state = 0;
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	power_state = stm32l011_get_power_gpio_state(chip);
	usbsel_state = stm32l011_get_usbsel_state(chip);

	*(int *)val = (power_state << 4) | usbsel_state;

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(power_usbsel_fops, get_power_usbsel, set_power_usbsel, "0x%02llx\n");

static int burn_ctl_open(struct inode *inode, struct file *file)
{
	struct test_data *data;

	pr_info("open\n");
	if (file->private_data == NULL) {
		data = kzalloc(sizeof(struct test_data), GFP_KERNEL);
		if (!data) {
			pr_err("Unable to allocate memory for test_data\n");
			return -ENOMEM;
		}

		pr_info("new open\n");
		data->chip = (struct stm32l011_mcu *)inode->i_private;
		if (!get_atboot())
			data->chip->burn_handling = true;
		data->chip->burn_ctl_work_state = false;
		data->type = I2C_BURN;
		data->dir_chg_switch = true;
		data->burn_from_user = false;
		data->voltage = 5080;
		data->icurrent = 2000;
		data->impedance = 300;
		data->dchg_switch = 1;
		data->crc32 = 0xFFFFFFFF;
		data->size = TRANSFER_BATCH_SIZE;
		memset(data->buffer, 0x0, TRANSFER_BATCH_SIZE);

		file->private_data = data;
	}
	return 0;
}
static int burn_ctl_close(struct inode *inode, struct file *file)
{
	struct test_data *data = file->private_data;

	pr_info("close\n");
	if (data->chip) {
		if (!get_atboot())
			data->chip->burn_handling = false;
	}

	if (data)
		kfree(data);

	return 0;
}
static ssize_t burn_ctl_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	//u32 len;
	//u32 ret;
	u8 result;
	struct test_data *data = file->private_data;

	pr_info("read : %d\n", data->opt_ret);
#if 1
	if (data->opt_ret < 0) {
		result = 0xFF;
	} else {
		result = (data->opt_ret & 0xFF);
	}
	//len = 1;
	//ret = copy_to_user(buf, &result, 1);
#else
	len = min((u32)count, data->size);
	ret = copy_to_user(buf, data->buffer, len);
#endif
	/*
	   if (ret) {
	   pr_err("error copy values to user\n");
	   return -EFAULT;
	   }
	   */
	return simple_read_from_buffer(buf, count, ppos, &result, 1);
}

static ssize_t burn_ctl_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	u8 cmd = BURN_CMD_UNKNOW, reg_value = 0x0;
	u32 ret, len, i;
	u16 erase_size = 0;
	u8 *p;
	struct test_data *data = file->private_data;
	struct stm32l011_mcu *chip = data->chip;

	len = min((u32)count, MAX_FW_SIZE);
	ret = copy_from_user(data->buffer, buf, len);
	if (ret) {
		pr_err("failed to copy data from user\n");
		return -EFAULT;
	}

	if (len >= OUTSIDE_BATCH) {
		data->size = len;
		data->burn_from_user = true;
		cmd = BURN_CMD_WRITE;
		pr_info("write select\n");
	} else {
		data->burn_from_user = false;
		cmd = data->buffer[0];
		pr_info("cmd:0x%02x", cmd);
		for (i=1; i<len; i++)
			pr_info(",0x%02x", data->buffer[i]);

		pr_info("\n");
	}

	switch (cmd) {
	case BURN_CMD_WRITE:
		pr_info("test write, type(%d)\n", data->type);
		if (data->burn_from_user) {
			if (I2C_BURN == data->type) {
				data->crc32 = stm32l011_i2c_burn_write(chip, true,
						data->buffer, FLASH_APP_ADDR_SHIFT, data->size, INSIDE_BATCH);
			} else {
				data->crc32 = stm32l011_i2c_burn_write(chip, true,
						data->buffer, FLASH_APP_ADDR_SHIFT, data->size, OUTSIDE_BATCH);
			}
		} else {
			if (I2C_BURN == data->type) {
#if 0
				data->opt_ret = stm32l011_i2c_burn_write(chip, false,
						chip->fw.data, FLASH_APP_ADDR_SHIFT, chip->fw.size, INSIDE_BATCH);
#else
				cancel_delayed_work_sync(&chip->burn_ctl_work);
				chip->burn_ctl_work_state = false;
				schedule_delayed_work(&chip->burn_ctl_work, 0);
#endif
				data->crc32 = chip->fw.crc;
				data->size = chip->fw.size;
			}
		}
		break;
	case BURN_CMD_ERASE:
		if (len >= 5) {
			erase_size = ((data->buffer[4]<<24) | (data->buffer[3]<<16)
					| (data->buffer[2]<<8) | data->buffer[1]);
			erase_size = (erase_size / FLASH_PAGE_SIZE + ((erase_size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
		} else {
			if (I2C_BURN == data->type)
				erase_size = (chip->fw.size / FLASH_PAGE_SIZE + ((chip->fw.size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
		}
		pr_info("test erase[%d], type(%d)\n", erase_size, data->type);
		p = get_erase_cmd_param(chip, FLASH_APP_ADDR_SHIFT, erase_size);
		data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
		break;
	case BURN_CMD_JUMP:
		pr_info("test jump, type(%d)\n", data->type);
		data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_JUMP);
		/* reset value after JUMP */
		mutex_lock(&chip->parameters_lock);
		chip->init_done = false;
		chip->exit = false;
		chip->handshake_success = false;
		chip->handshake_failed = false;
		mutex_unlock(&chip->parameters_lock);

		if (chip->usb_present) {
			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, 0);
		}
		break;
	case BURN_CMD_CRC:
		pr_info("test crc, type(%d)\n", data->type);
		p = get_crc_cmd_param(chip, data->crc32, FLASH_APP_ADDR_SHIFT, data->size);
		data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_CRC, p, CMD_CRC_PARAM);
		if (data->opt_ret < 0) {
			chip->burn_success = false;
			pr_info("crc check error\n");
		} else {
			chip->burn_success = true;
			pr_info("crc check success\n");
		}
		break;
	case BURN_CMD_BOOT_VERSION:
		pr_info("test read boot_version, type(%d)\n", data->type);
		data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_BOOT_VERSION, &data->type, CMD_ONE_PARAM);
		pr_info("check MCU BL version=0x%x, MCU BL vendor=0x%x\n", chip->mcu_bl_version, chip->mcu_bl_vendor);
		break;
	case BURN_CMD_UPDATE:
		pr_info("test update, type(%d)\n", data->type);
		cancel_delayed_work_sync(&chip->chg_monitor_work);
		if (I2C_BURN == data->type) {
			stm32l011_power_restore_enable(chip);
			msleep(100);
		}
		data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_UPDATE, &data->type, CMD_ONE_PARAM);
		break;
	case BURN_CMD_APP_UPDATE:
		pr_info("test app_update, type(%d)\n", data->type);
		data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_APP_UPDATE);
		break;
	case BURN_CMD_APP_VERSION:
		pr_info("test app_version, type(%d)\n", data->type);
		data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_APP_VERSION);
		break;
	case BURN_CMD_TEST_I2C:
		pr_info("test set i2c_burn, type(%d)\n", data->type);
		data->opt_ret = data->type = I2C_BURN;
		stm32l011_usbsel_enable(chip, 0);
		break;
	case BURN_CMD_TEST_SERIAL:
		pr_info("test set serial_burn, type(%d)\n", data->type);
		data->opt_ret = data->type = SERIAL_BURN;
		stm32l011_usbsel_enable(chip, 1);
		break;
	case TEST_DIRECT_CHARGE_SWITCH:
		if (len >= 2) {
			data->dchg_switch = data->buffer[1];
			if (0 == data->dchg_switch)
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
			else
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, 0);
		} else {
			data->opt_ret = -1;
		}
		break;
	case TEST_SET_VOLTAGE:
#if 1
		pr_info("skip set voltage\n");
#else
		if (len >= 5) {
			data->voltage = ((data->buffer[4]<<24) | (data->buffer[3]<<16)
					| (data->buffer[2]<<8) | data->buffer[1]);
			if (data->voltage < 3500 || data->voltage > 5500) {
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_VOLTAGE, 0);
			} else {
				reg_value = (data->voltage / 500);
				pr_info("test set voltage(%d), type(%d)\n", data->voltage, data->type);
				data->opt_ret = stm32l011_write_reg(chip, MCU_ADAPTER_VOLTAGE, reg_value);
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_VOLTAGE, MCU_SET_VOLTAGE);
			}
		} else {
			data->opt_ret = -1;
		}
#endif
		break;
	case TEST_SET_CURRENT:
		if (len >= 5) {
			data->icurrent = ((data->buffer[4]<<24) | (data->buffer[3]<<16)
					| (data->buffer[2]<<8) | data->buffer[1]);

			/* 产线“直充主充输出电流2.5A”测试项:
			 * 因AT模式下，半成品测试（有些器件没扣全），系统端耗电偏大（高端项目、5G项目），进入电池端电流偏小（接近2A）,测试数据与“普通充电2A”容易混淆。
			 * 此处优化：将2.5A档位 增加 100mA余量用于系统端消耗。*/
			if (data->icurrent == 2500) {
				pr_err("get_atboot()=%d, AT request ibat=%dmA, change it to 2600mA (add 100mA margin for System Current Consume).\n", get_atboot(), data->icurrent);
				data->icurrent = 2600;
			}

			data->opt_ret = force_active_votable(chip->dchg_fcc_votable, FACTORY_FCC_VOTER, true, data->icurrent);
			if (data->opt_ret < 0)
				pr_err("Couldn't vote fastchg ma ret = %d\n", data->opt_ret);
		} else {
			data->opt_ret = -1;
		}
		break;
	case TEST_SET_IMPEDANCE:
		if (len >= 5) {
			data->impedance = ((data->buffer[4]<<24) | (data->buffer[3]<<16)
					| (data->buffer[2]<<8) | data->buffer[1]);
			if (data->impedance <= 300 || data->impedance >= 1000) {
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_IMPEDANCE, 0);
			} else {
				reg_value = (data->impedance / 100);
				pr_info("test set impedance(%d), type(%d)\n", data->impedance, data->type);
				data->opt_ret = stm32l011_write_reg(chip, MCU_ADAPTER_IMPEDANCE, reg_value);
				data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_IMPEDANCE, MCU_SET_IMPEDANCE);
			}
		} else {
			data->opt_ret = -1;
		}
		break;
	default:
		pr_info("burn cmd not support\n");
		break;
	}

	return len;
}

static const struct file_operations burn_ctl_fops = {
	.open		= burn_ctl_open,
	.release	= burn_ctl_close,
	.read		= burn_ctl_read,
	.write		= burn_ctl_write,
};

static int get_burn_ctl_result(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	*val = chip->burn_ctl_work_state;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(burn_ctl_result_ops, get_burn_ctl_result, NULL, "%lld\n");

#if TEST_CYCLE_CHARGE_DISCHARGE
static int set_chg_switch(void *data, u64 val)
{
	int ret = 0;
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	union power_supply_propval value = {0,};

	if (chip->burn_handling || chip->dchg_pon_reset) {
		pr_warn("burn_handling\n");
		return 0;
	}

	//chip->burn_handling = true;
	if (0 == val) {
		pr_info("suspend charging\n");
		chip->chg_switch = false;
		stm32l011_dchg_enable(chip, false);

		value.intval = 0;
		power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
	} else if (1 == val) {
		pr_info("resume charging\n");
		cancel_delayed_work(&chip->chg_monitor_work);
		chip->chg_switch = true;

		value.intval = 1;
		power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
		stm32l011_dchg_enable(chip, true);
	}
	//chip->burn_handling = false;
	return ret;
}
static int get_chg_switch(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	*val = chip->chg_switch;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_switch_fops, get_chg_switch, set_chg_switch, "%lld\n");

static int get_dchg_exit_event(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	*val = chip->exit_event;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dchg_exit_event_fops, get_dchg_exit_event, NULL, "%lld\n");

static int get_burn_success(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	if (chip->burn_success) {
		*val = 1;
	} else {
		*val = 0;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(burn_success_fops, get_burn_success, NULL, "%lld\n");

static int get_handshake_success(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	if (chip->handshake_success) {
		*val = 1;
	} else {
		*val = 0;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(handshake_success_fops, get_handshake_success, NULL, "%lld\n");
#endif

static int get_inside_version(void *data, u64 *val)
{
	u8 reg_value = 0x0;
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	stm32l011_read_reg(chip, MCU_VERSION_END, &reg_value);
	*val = reg_value;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(inside_version_fops, get_inside_version, NULL, "%lld\n");

static int get_outside_version(void *data, u64 *val)
{
	u8 reg_value = 0x0;
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	stm32l011_read_reg(chip, MCU_ADAPTER_VERSION, &reg_value);
	*val = reg_value;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(outside_version_fops, get_outside_version, NULL, "%lld\n");

static int get_ibus(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	if (chip->chg_enable)
		*val = chip->ibus_ma;
	else
		*val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ibus_fops, get_ibus, NULL, "%lld\n");
static int get_master_ibus(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	pr_info("%d,%d,%d,%d\n", chip->chg_enable, chip->slave_chg_enable, chip->ibus_ma, chip->slave_ibus_ma);
	if (chip->chg_enable) {
		if (!chip->slave_chg_enable)
			*val = chip->ibus_ma;
		else
			*val = chip->ibus_ma - chip->slave_ibus_ma;
	}else
		*val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(master_ibus_fops, get_master_ibus, NULL, "%lld\n");
static int get_slave_ibus(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	pr_info("%d,%d\n", chip->slave_chg_enable, chip->slave_ibus_ma);
	if (chip->slave_chg_enable)
		*val = chip->slave_ibus_ma;
	else
		*val = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(slave_ibus_fops, get_slave_ibus, NULL, "%lld\n");
static int get_slave_chg_enable(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	pr_info("%d,%d\n", chip->slave_chg_enable, chip->slave_ibus_ma);
	*val = chip->slave_chg_enable;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(slave_chg_enable_fops, get_slave_chg_enable, NULL, "%lld\n");
static int get_master_chg_enable(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	pr_info("%d,%d\n", chip->chg_enable, chip->ibus_ma);
	*val = chip->chg_enable;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(master_chg_enable_fops, get_master_chg_enable, NULL, "%lld\n");
static int get_vbus(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->vbus_mv;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vbus_fops, get_vbus, NULL, "%lld\n");

static int get_vbat(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->vbat_mv;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vbat_fops, get_vbat, NULL, "%lld\n");

static int get_cable_mohm(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->cable_mohm;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cable_mohm_fops, get_cable_mohm, NULL, "%lld\n");

static int get_req_ibus(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->req_ibus;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(req_ibus_fops, get_req_ibus, NULL, "%lld\n");

static int get_ldo_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->ldo_count;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ldo_count_fops, get_ldo_count, NULL, "%lld\n");

static int get_ptmValue(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->ptmValue;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ptmValue_fops, get_ptmValue, NULL, "%lld\n");

static int get_adjust_stop(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adjust_stop;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adjust_stop_fops, get_adjust_stop, NULL, "%lld\n");

static int get_is_ldo(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->is_ldo;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(is_ldo_fops, get_is_ldo, NULL, "%lld\n");

static int get_bat_temp_mv(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->bat_temp_mv;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bat_temp_mv_fops, get_bat_temp_mv, NULL, "%lld\n");
#define BACKUP_TEMP_TIMER	60	//s
static int get_bat_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	int ret = chip->bat_temp;
	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		ret = chip->bat_temp;
	} else if (chip->bat_temp_backup > 0)
		ret = chip->bat_temp_backup;

	pr_err("bat_temp_backup=%d, bat_temp=%d, ret=%d. dchg_continue_time=%ld\n", chip->bat_temp_backup, chip->bat_temp, ret, chip->dchg_continue_time);
#if 1//
	*val = DEFAULT_BOARD_TEMP;
#else
	*val = ret;
#endif
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bat_temp_fops, get_bat_temp, NULL, "%lld\n");

static int get_bat_conn_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	int ret = chip->bat_conn_temp;
	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		ret = chip->bat_conn_temp;
	} else if (chip->bat_conn_temp_backup > 0)
		ret = chip->bat_conn_temp_backup;

	pr_err("bat_conn_temp_backup=%d, bat_conn_temp=%d, ret=%d. dchg_continue_time=%ld\n", chip->bat_conn_temp_backup, chip->bat_conn_temp, ret, chip->dchg_continue_time);
	*val = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(bat_conn_temp_fops, get_bat_conn_temp, NULL, "%lld\n");

static int get_master_bat_conn_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	int ret = chip->master_bat_conn_temp;
	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		ret = chip->master_bat_conn_temp;
	} else if (chip->master_bat_conn_temp_backup > 0)
		ret = chip->master_bat_conn_temp_backup;

	pr_err("master_bat_conn_temp_backup=%d, master_bat_conn_temp=%d, ret=%d. dchg_continue_time=%ld\n", chip->master_bat_conn_temp_backup, chip->master_bat_conn_temp, ret, chip->dchg_continue_time);
	*val = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(master_bat_conn_temp_fops, get_master_bat_conn_temp, NULL, "%lld\n");

static int get_usb_conn_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	int ret = chip->usb_conn_temp;
	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		ret = chip->usb_conn_temp;
	} else if (chip->usb_conn_temp_backup > 0)
		ret = chip->usb_conn_temp_backup;

	pr_err("usb_conn_temp_backup=%d, usb_conn_temp=%d, ret=%d. dchg_continue_time=%ld\n", chip->usb_conn_temp_backup, chip->usb_conn_temp, ret, chip->dchg_continue_time);
	*val = ret;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(usb_conn_temp_fops, get_usb_conn_temp, NULL, "%lld\n");
static int get_adapter_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adapter_temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_temp_fops, get_adapter_temp, NULL, "%lld\n");
static int get_adapter_conn_temp(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adapter_conn_temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_conn_temp_fops, get_adapter_conn_temp, NULL, "%lld\n");

static int get_dp_mv(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->dp_mv;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dp_mv_fops, get_dp_mv, NULL, "%lld\n");

static int get_dm_mv(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->dm_mv;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dm_mv_fops, get_dm_mv, NULL, "%lld\n");
static int get_cable_matched(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;

	seq_printf(m, "%s.sbu1:%d,sbu2:%d. FFC:%d, Cout:%d\n",
			sbu_cable_id_type_strings[chip->sbu_cable_id], chip->sbu1_adc, chip->sbu2_adc, chip->ex_fg_ffc_support, chip->battery_cout_counter);

	return 0;
}

static int cable_matched_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, get_cable_matched, chip);
}

static const struct file_operations cable_matched_fops = {
	.owner		= THIS_MODULE,
	.open		= cable_matched_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_chg_enable(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	bool	chg_enable = 0;

	pr_info("real_vbat_mv=%d, batt_capacity=%d, user_dchg_current=%d\n",
			chip->real_vbat_mv, chip->batt_capacity, chip->user_dchg_current);

	if (chip->enable_slave_charger) {
		if ((get_atboot() && !chip->at_mode_slave_chg_enable_req) || chip->real_vbat_mv > 4200
				|| chip->batt_capacity >=80 || chip->user_dchg_current < 3800)
			chg_enable = chip->chg_enable;
		else
			chg_enable = chip->chg_enable && chip->slave_chg_enable;
	}else {
		chg_enable = chip->chg_enable;
	}
	*val = chg_enable;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(chg_enable_fops, get_chg_enable, NULL, "%lld\n");
static int set_slave_chg_enable_flag(void *data, u64 val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	pr_info("val=%lld", val);
	if (1 == val)
		chip->at_mode_slave_chg_enable_req = 1;
	else
		chip->at_mode_slave_chg_enable_req = 0;
	return 0;
}
static int get_slave_chg_enable_flag(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->at_mode_slave_chg_enable_req;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(slave_chg_enable_flag_fops, get_slave_chg_enable_flag, set_slave_chg_enable_flag, "%lld\n");
static int get_analog_i2c_read_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->analog_i2c_read_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(analog_i2c_read_count_fops, get_analog_i2c_read_count, NULL, "%lld\n");
static int get_analog_i2c_write_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->analog_i2c_write_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(analog_i2c_write_count_fops, get_analog_i2c_write_count, NULL, "%lld\n");
static int get_uart_tx_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->uart_tx_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(uart_tx_count_fops, get_uart_tx_count, NULL, "%lld\n");
static int get_uart_rx_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->uart_rx_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(uart_rx_count_fops, get_uart_rx_count, NULL, "%lld\n");
static int get_ap_i2c_read_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->ap_i2c_read_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ap_i2c_read_count_fops, get_ap_i2c_read_count, NULL, "%lld\n");
static int get_ap_i2c_write_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->ap_i2c_write_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ap_i2c_write_count_fops, get_ap_i2c_write_count, NULL, "%lld\n");
static int get_adapter_i2c_read_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	//*val = chip->adapter_i2c_read_exception;
	*val = chip->req_vbus;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_i2c_read_count_fops, get_adapter_i2c_read_count, NULL, "%lld\n");
static int get_adapter_i2c_write_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	//*val = chip->adapter_i2c_write_exception;
	*val = chip->req_ibus;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_i2c_write_count_fops, get_adapter_i2c_write_count, NULL, "%lld\n");
static int get_adapter_uart_tx_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adapter_uart_tx_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_uart_tx_count_fops, get_adapter_uart_tx_count, NULL, "%lld\n");
static int get_adapter_uart_rx_count(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adapter_uart_rx_exception;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_uart_rx_count_fops, get_adapter_uart_rx_count, NULL, "%lld\n");
static int get_adapter_status_monitor(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;
	*val = chip->adapter_status;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adapter_status_monitor_fops, get_adapter_status_monitor, NULL, "0x%08llx\n");
#if OLD_CAM_NOTIFY
static int set_flashlight_test(void *data, u64 val)
{
	//struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	pr_info("val=%lld", val);
	if (1 == val)
		cam_notify_status_to_charge(true);
	else
		cam_notify_status_to_charge(false);
	return 0;
}

static int get_flashlight_test(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	pr_info("val=%d", *(int *)val);
	if (chip->cam_running)
		*(int *)val = 1;
	else
		*(int *)val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(flashlight_test_ops, get_flashlight_test, set_flashlight_test, "%lld\n");
#endif
static int set_sbu_gpio(void *data, u64 val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	pr_info("val=%lld", val);
	if (1 == val) {
		max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_DIRECT_CONNECT);
		gpio_direction_output(chip->sbu_pwr_gpio, 1);
	} else {
		max20328_switch_mode_event(MAX_USBC_FAST_CHARGE_SELECT);
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
	}
	return 0;
}

static int get_sbu_gpio(void *data, u64 *val)
{
	struct stm32l011_mcu *chip = (struct stm32l011_mcu *)data;

	if (gpio_get_value(chip->sbu_pwr_gpio))
		*(int *)val = 1;
	else
		*(int *)val = 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(sbu_gpio_ops, get_sbu_gpio, set_sbu_gpio, "%lld\n");

static int get_sbu_info(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;
	int cabel_A = 0;
	const struct sbu_r_adc * sub_normal_radc = sub_normal_radc_rely_switch;
	const struct sbu_r_adc * sub_error_radc = sub_error_radc_rely_switch;

	if (!chip->sbu_rely_switch) {
		sub_normal_radc = sub_normal_radc_no_switch;
		sub_error_radc = sub_error_radc_no_switch;
	}

	switch (chip->sbu_cable_id) {
	case SBU_CABLE_ID_DEFAULT:
		cabel_A = 3;
		break;
	case SBU_CABLE_ID_4A:
		cabel_A = 4;
		break;
	case SBU_CABLE_ID_5A:
		cabel_A = 5;
		break;
	case SBU_CABLE_ID_6A:
		cabel_A = 6;
		break;
	case SBU_CABLE_ID_8A:
		cabel_A = 8;
		break;
	case SBU_CABLE_ID_UNKNOW:
	default:
		cabel_A = 0;
		break;
	}

	if (chip->cable_id_detect_done == 1) {
		seq_printf(m, "%d,%d,%d,%d,%d,%d,%d", cabel_A, chip->sbu1_adc, chip->sbu2_adc,
				sub_normal_radc[chip->sbu1_range_index].vmin, sub_normal_radc[chip->sbu1_range_index].vmax,
				sub_normal_radc[chip->sbu2_range_index].vmin, sub_normal_radc[chip->sbu2_range_index].vmax);
	} else if (chip->cable_id_detect_done == 2) {
		seq_printf(m, "%d,%d,%d,%d,%d,%d,%d", cabel_A, chip->sbu1_adc, chip->sbu2_adc,
				sub_error_radc[chip->sbu1_range_index].vmin, sub_error_radc[chip->sbu1_range_index].vmax,
				sub_error_radc[chip->sbu2_range_index].vmin, sub_error_radc[chip->sbu2_range_index].vmax);
	} else {
		seq_printf(m, "unknown");
	}
	return 0;
}

static int sbu_info_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;
	return single_open(file, get_sbu_info, chip);
}

static const struct file_operations sbu_info_fops = {
	.owner		= THIS_MODULE,
	.open		= sbu_info_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif
#endif

#if 0//def DEBUG
static void dump_regs(struct stm32l011_mcu *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_info("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_info("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = stm32l011_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_info("0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct stm32l011_mcu *chip)
{
}
#endif

static void *stm32l011_of_get_property_data(struct stm32l011_mcu *chip, struct device_node *node, const char * prop_name,
		int rows, int cols, int type, char *tip)
{
	struct cable_r_limit_table *ori_cable_r_data;
	struct ffc_cc_to_cv_ibat_thr_table *ori_ffc_cc_to_cv_data;
	struct ffc_dchg_cv_mv_table *ori_ffc_dchg_cv_data;
	struct property *prop;
	const __be32 *data;
	int *prop_data = NULL;
	char *default_tip = "default";
	int size, i;

	if (!tip) {
		tip = default_tip;
	}

	if (rows*cols) {
		prop = of_find_property(node, prop_name, NULL);
		if (!prop) {
			pr_err("prop %s is not found\n", prop_name);
			goto out;
		}
		data = prop->value;
		size = prop->length/sizeof(int);
		if (size <= 0 || size != cols * rows) {
			pr_err("%s: %s data size mismatch, %dx%d != %d\n",
					node->name, prop_name, cols, rows, size);
			goto out;
		}

		prop_data = kzalloc((sizeof(int) * size), GFP_KERNEL);
		if (!prop_data) {
			pr_err("kzalloc failed %d\n", __LINE__);
			goto out;
		}

		if (type == STRUCT_CABLE_R_LIMIT_TABLE) {
			ori_cable_r_data = (struct cable_r_limit_table *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_cable_r_data[i].rmin = be32_to_cpup(data++);
				ori_cable_r_data[i].rmax = be32_to_cpup(data++);
				ori_cable_r_data[i].dchg_ibus_max = be32_to_cpup(data++);
				ori_cable_r_data[i].cable_id_type = be32_to_cpup(data++);
				ori_cable_r_data[i].limit_count = be32_to_cpup(data++);
				pr_info("%s :rmin=%d, rmax=%d, dchg_ibus_max=%d, cable_id_type=%d, limit_count=%d\n", tip, ori_cable_r_data[i].rmin, \
						ori_cable_r_data[i].rmax, ori_cable_r_data[i].dchg_ibus_max, ori_cable_r_data[i].cable_id_type, ori_cable_r_data[i].limit_count);
			}
		} else if (type == STRUCT_FFC_CC_TO_CV_IBAT_THR_TABLE) {
			ori_ffc_cc_to_cv_data = (struct ffc_cc_to_cv_ibat_thr_table *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_ffc_cc_to_cv_data[i].tmin = be32_to_cpup(data++);
				ori_ffc_cc_to_cv_data[i].tmax = be32_to_cpup(data++);
				ori_ffc_cc_to_cv_data[i].ibat = be32_to_cpup(data++);
				pr_info("%s :tmin=%d, tmax=%d, ibat=%d\n", tip, ori_ffc_cc_to_cv_data[i].tmin, \
						ori_ffc_cc_to_cv_data[i].tmax, ori_ffc_cc_to_cv_data[i].ibat);
			}
		} else if (type == STRUCT_FFC_DCHG_CV_MV_TABLE) {
			ori_ffc_dchg_cv_data = (struct ffc_dchg_cv_mv_table *)prop_data;
			for (i = 0; i < rows; i++) {
				ori_ffc_dchg_cv_data[i].btb_cv_mv = be32_to_cpup(data++);
				ori_ffc_dchg_cv_data[i].fg_cv_mv = be32_to_cpup(data++);
				pr_info("%s :btb_cv_mv=%d, fg_cv_mv=%d\n", tip, ori_ffc_dchg_cv_data[i].btb_cv_mv, \
						ori_ffc_dchg_cv_data[i].fg_cv_mv);
			}
		} else if (type == STRUCT_INT_ARRAY) {
			for (i = 0; i < cols; i++) {
				prop_data[i] = be32_to_cpup(data++);
				pr_info("%s :data = %d\n", tip, prop_data[i]);
			}
		} else {
			pr_err("%s :type out of range\n", tip);
			kfree(prop_data);
			prop_data = NULL;
		}
	}else{
		pr_err("%s :rows or lows is error\n", tip);
	}

out:
	return prop_data;
}

static void stm_release_property_data(struct stm32l011_mcu *chip)
{
	if (chip->dchg_exit_ntc_threshold != default_dchg_exit_ntc_threshold) {
		kfree(chip->dchg_exit_ntc_threshold);
	}
	if (chip->cable_r_limit_table != default_cable_r_limit_table) {
		kfree(chip->cable_r_limit_table);
		kfree(chip->cable_r_limit_table_rc);
	}
	if (chip->ffc_dchg_cv_mv != default_ffc_dchg_cv_mv_table) {
		kfree(chip->ffc_dchg_cv_mv);
		kfree(chip->ffc_dchg_cv_mv_rc);
	}
	if (chip->ffc_cc_to_cv_ibat_thr != default_ffc_cc_to_cv_ibat_thr_table) {
		kfree(chip->ffc_cc_to_cv_ibat_thr);
		kfree(chip->ffc_cc_to_cv_ibat_thr_rc);
	}
}

static int stm32l011_gpio_parse_dt(struct stm32l011_mcu *chip)
{
	int ret = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_err("device tree info. missing\n");
		return -EINVAL;
	}
	chip->power_gpio = of_get_named_gpio(node, "vivo,power-gpio", 0);
	if (chip->power_gpio < 0) {
		pr_err("failed to get power-gpio.\n");
		chip->power_gpio = 0;
		chip->power_ldo = devm_regulator_get(chip->dev, "vpower-ldo");
		if (!IS_ERR_OR_NULL(chip->power_ldo)) {
			ret = regulator_set_voltage(chip->power_ldo, 3300000, 3300000);
			if (ret < 0) {
				pr_err("regulator_set_voltage failed %d\n", ret);
			}
		} else {
			pr_err("Failed to get regulator vpower-ldo:%d\n", PTR_ERR(chip->power_ldo));
			chip->power_ldo = NULL;
		}
	}
	chip->int_gpio = of_get_named_gpio(node, "vivo,int-gpio", 0);
	if (chip->int_gpio < 0) {
		pr_err("failed to get int-gpio.\n");
	}

	chip->usbsel_gpio = of_get_named_gpio(node, "vivo,usbsel-gpio", 0);
	if (chip->usbsel_gpio < 0) {
		pr_err("failed to get usbsel-gpio.\n");
		chip->usbsel_gpio = 0;
	}

	chip->shift_en_gpio = of_get_named_gpio(node, "vivo,shift-en-gpio", 0);
	if (chip->shift_en_gpio < 0) {
		pr_err("failed to get shift_en-gpio.\n");
		chip->shift_en_gpio = 0;
	}

	chip->chg_version_gpio = of_get_named_gpio(node, "vivo,chg-version-gpio", 0);
	if (chip->chg_version_gpio < 0) {
		pr_err("failed to get chg_version_gpio.\n");
		chip->chg_version_gpio = 0;
	}

	chip->sbu_pwr_gpio = of_get_named_gpio(node, "vivo,sbu-pwr-gpio", 0);
	if (chip->sbu_pwr_gpio < 0) {
		pr_err("failed to get sbu-pwr-gpio.\n");
		chip->sbu_pwr_gpio = 0;
	} else {
		pr_err("init sbu-pwr-gpio low.\n");
		gpio_request(chip->sbu_pwr_gpio, "sbu_pwr_gpio");
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
	}
	return 0;
}

static int stm32l011_parse_dt(struct stm32l011_mcu *chip)
{
	int rc = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_err("device tree info. missing\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "vivo,total-limit-max-ma", &chip->total_dchg_limit_max_ma);
	if (rc)
		chip->total_dchg_limit_max_ma = 3000;
	pr_info("total_dchg_limit_max_ma=%d\n", chip->total_dchg_limit_max_ma);

	rc = of_property_read_u32(node, "vivo,bigdata-collect-time-limit", &chip->bigdata_collect_time_limit);
	if (rc) {
		chip->bigdata_collect_time_limit = 95;
		pr_err("failed to get vivo,bigdata-collect-time-limit. (default : %d sec) \n", chip->bigdata_collect_time_limit);
	}

	chip->ffc_support = of_property_read_bool(node, "vivo,ffc-support");
	pr_info("ffc_support=%d, ex_fg_ffc_sup=%d\n", chip->ffc_support, chip->ex_fg_ffc_support);
	if (!chip->ex_fg_ffc_support)
		chip->ex_fg_ffc_support = chip->ffc_support;

	rc = of_property_read_u32(node, "vivo,dchg-supported-type", &chip->dchg_supported_type);
	if (rc < 0) {
		chip->dchg_supported_type = DCHG_SINGLE_TYPE_33W;
	}
	pr_info("dchg-supported-type=%d\n", chip->dchg_supported_type);

	chip->ln8000_supported = of_property_read_bool(node, "vivo,ln8000-support");
	pr_info("ln8000-support=%d\n", chip->ln8000_supported);

	chip->enable_slave_charger = false;//(chip->dchg_supported_type == DCHG_DUAL_TYPE_44W);
	pr_info("enable_slave_charger=%d\n", chip->enable_slave_charger);

	/*battery board temp ntc enable state*/
	rc = of_property_read_u32(node, "vivo,bat-board-temp-enable", &chip->bat_board_temp_enable);
	if (rc < 0) {
		chip->bat_board_temp_enable = NTC_NONSUPPORT;
	}
	pr_info("bat_board_temp_enable=%d\n", chip->bat_board_temp_enable);

	/*usb conn temp ntc enable state*/
	rc = of_property_read_u32(node, "vivo,usb-conn-temp-enable", &chip->usb_conn_temp_enable);
	if (rc < 0) {
		chip->usb_conn_temp_enable = NTC_NONSUPPORT;
	}
	pr_info("usb_conn_temp_enable=%d\n", chip->usb_conn_temp_enable);

	/*bat conn temp ntc enable state*/
	rc = of_property_read_u32(node, "vivo,bat-conn-temp-enable", &chip->bat_conn_temp_enable);
	if (rc < 0) {
		chip->bat_conn_temp_enable = NTC_FROM_BQ25970;
	}
	pr_info("bat_conn_temp_enable=%d\n", chip->bat_conn_temp_enable);

	/*master bat conn temp ntc enable state*/
	rc = of_property_read_u32(node, "vivo,master-bat-conn-temp-enable", &chip->master_bat_conn_temp_enable);
	if (rc < 0) {
		chip->master_bat_conn_temp_enable = NTC_NONSUPPORT;
	}
	pr_info("master_bat_conn_temp_enable=%d\n", chip->master_bat_conn_temp_enable);

	rc = of_property_read_string(node, "vivo,bms-psy-name", &chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = "bms";

	/*FFC PARAM start*/
	chip->cout_gpio = of_get_named_gpio(node, "vivo,cout-int-gpio", 0);
	if (chip->cout_gpio < 0) {
		pr_err("failed to get vivo,cout-int-gpio.\n");
		chip->cout_gpio = 0;
	}

	/*ffc cc to cv ibat threshold param*/
	chip->ffc_cc_to_cv_ibat_thr_rc = (int*)stm32l011_of_get_property_data(chip, node,
			"vivo,ffc-cc-to-cv-ibat-thr-rc", 1, 2, STRUCT_INT_ARRAY, NULL);
	if (chip->ffc_cc_to_cv_ibat_thr_rc) {
		chip->ffc_cc_to_cv_ibat_thr = (struct ffc_cc_to_cv_ibat_thr_table *)stm32l011_of_get_property_data(chip, node,
				"vivo,ffc-cc-to-cv-ibat-thr-table", chip->ffc_cc_to_cv_ibat_thr_rc[ROW], chip->ffc_cc_to_cv_ibat_thr_rc[COL],
				STRUCT_FFC_CC_TO_CV_IBAT_THR_TABLE, "ffc-cc-to-cv-ibat-thr-table");
	}
	if (!chip->ffc_cc_to_cv_ibat_thr_rc || !chip->ffc_cc_to_cv_ibat_thr) {
		chip->ffc_cc_to_cv_ibat_thr = default_ffc_cc_to_cv_ibat_thr_table;
		chip->ffc_cc_to_cv_ibat_thr_rc = default_ffc_cc_to_cv_ibat_thr_rc;
	}

	/*ffc dchg cv mv param*/
	chip->ffc_dchg_cv_mv_rc = (int*)stm32l011_of_get_property_data(chip, node,
			"vivo,ffc-dchg-cv-mv-rc", 1, 2, STRUCT_INT_ARRAY, NULL);
	if (chip->ffc_dchg_cv_mv_rc) {
		chip->ffc_dchg_cv_mv = (struct ffc_dchg_cv_mv_table *)stm32l011_of_get_property_data(chip, node,
				"vivo,ffc-dchg-cv-mv-table", chip->ffc_dchg_cv_mv_rc[ROW], chip->ffc_dchg_cv_mv_rc[COL],
				STRUCT_FFC_DCHG_CV_MV_TABLE, "ffc-dchg-cv-mv-table");
	}
	if (!chip->ffc_dchg_cv_mv_rc || !chip->ffc_dchg_cv_mv) {
		chip->ffc_dchg_cv_mv = default_ffc_dchg_cv_mv_table;
		chip->ffc_dchg_cv_mv_rc = default_ffc_dchg_cv_mv_rc;
	}

	/*ffc temp range*/
	rc = of_property_read_u32_array(node, "vivo,ffc-temperature-range", chip->ffc_temperature_range, 2);
	if (rc < 0) {
		chip->ffc_temperature_range[0] = 150;
		chip->ffc_temperature_range[1] = 450;
	}

	rc = of_property_read_u32(node, "vivo,ffc-param-tuning-enable", &chip->ffc_param_tuning_enable);
	if (rc < 0) {
		chip->ffc_param_tuning_enable = 0;
	}

	pr_info("ffc_temperature_range=[%d, %d],ffc_param_tuning_enable=%d\n",
			chip->ffc_temperature_range[0], chip->ffc_temperature_range[1], chip->ffc_param_tuning_enable);
	/*FFC PARAM  end*/

	chip->bat_temp_exist = of_property_read_bool(node, "vivo,bat-temp-exist");
	pr_info("bat_temp_exist=%d\n", chip->bat_temp_exist);

	chip->sbu_cable_id_detect_enable = of_property_read_bool(node, "vivo,sbu-cable-id-detect-enable");
	chip->sbu_rely_switch = of_property_read_bool(node, "vivo,sbu-detect-rely-usb-switch");
	pr_info("sbu_cable_id_detect_enable=%d,sbu_rely_switch=%d\n",
			chip->sbu_cable_id_detect_enable, chip->sbu_rely_switch);

	/* cable R limit table */
	chip->cable_r_limit_table_rc = (int*)stm32l011_of_get_property_data(chip, node,
			"vivo,cable-r-limit-table-rc", 1, 2, STRUCT_INT_ARRAY, NULL);
	if (chip->cable_r_limit_table_rc) {
		chip->cable_r_limit_table = (struct cable_r_limit_table *)stm32l011_of_get_property_data(chip, node,
				"vivo,cable-r-limit-table", chip->cable_r_limit_table_rc[ROW], chip->cable_r_limit_table_rc[COL],
				STRUCT_CABLE_R_LIMIT_TABLE, "cable-r-limit-table");
	}
	if (!chip->cable_r_limit_table_rc || !chip->cable_r_limit_table) {
		chip->cable_r_limit_table = default_cable_r_limit_table;
		chip->cable_r_limit_table_rc = default_cable_r_limit_table_rc;
	}

	chip->adapter_power_derate_enable = of_property_read_bool(node, "vivo,adapter-power-derate-enable");

	/*bat_baord, usb_conn, pcb_conn(master_bat_conn), bat_conn*/
	chip->dchg_exit_ntc_threshold = (int*)stm32l011_of_get_property_data(chip, node,
			"vivo,dchg-exit-ntc-threshold", 1, 5, STRUCT_INT_ARRAY, "dchg-exit-ntc-threshold");
	if (!chip->dchg_exit_ntc_threshold) {
		chip->dchg_exit_ntc_threshold = default_dchg_exit_ntc_threshold;
	}

	rc = of_property_read_u32(node, "vivo,fw-id-num", &chip->fw_id_num);
	if (rc < 0) {
		chip->fw_id_num = 100;
	}
	return 0;
}

struct device_node *of_fw_get_best_profile(
		const struct device_node *fwdata_container_node,
		int fw_id_num, const char *fw_type)
{
	struct device_node *node, *best_node = NULL;
	const char *fw_temp_type = NULL;
	int delta = 0, best_delta = 0, best_id_num = 0, fw_temp_id = 0, id_range_pct,
	    rc = 0, limit = 0;
	bool in_range = false;

	/* read fw id range percentage for best profile */
	rc = of_property_read_u32(fwdata_container_node,
			"vivo,fw-id-range-pct", &id_range_pct);

	if (rc) {
		if (rc == -EINVAL) {
			id_range_pct = 0;
		} else {
			pr_err("failed to read fw id range\n");
			return ERR_PTR(-ENXIO);
		}
	}

	/*
	 * Find the fw data with a fw id resistor closest to this one
	 */
	for_each_child_of_node(fwdata_container_node, node) {
		if (fw_type != NULL) {
			rc = of_property_read_string(node, "vivo,fw-type",
					&fw_temp_type);
			if (!rc && strcmp(fw_temp_type, fw_type) == 0) {
				best_node = node;
				best_id_num = fw_id_num;
				break;
			}
		} else {
			rc = of_property_read_u32(node, "vivo,fw-id", &fw_temp_id);
			if (rc)
				continue;

			delta = abs(fw_temp_id - fw_id_num);
			limit = (fw_temp_id * id_range_pct) / 100;
			in_range = (delta <= limit);
			/*
			 * Check if the delta is the lowest one
			 * and also if the limits are in range
			 * before selecting the best node.
			 */
			if ((delta < best_delta || !best_node)
					&& in_range) {
				best_node = node;
				best_delta = delta;
				best_id_num = fw_temp_id;
			}
		}
	}

	if (best_node == NULL) {
		pr_err("No fw data found\n");
		return best_node;
	}

	/* check that profile id is in range of the measured fw_id */
	if (abs(best_id_num - fw_id_num) >
			((best_id_num * id_range_pct) / 100)) {
		pr_err("out of range: profile id %d fw id %d pct %d\n",
				best_id_num, fw_id_num, id_range_pct);
		return NULL;
	}

	rc = of_property_read_string(best_node, "vivo,fw-type",
			&fw_temp_type);
	if (!rc)
		pr_info("%s found\n", fw_temp_type);
	else
		pr_info("%s found\n", best_node->name);

	return best_node;
}

static int of_fw_read_data(const struct device_node *np,
		const char *propname, struct mcu_fw *fw)
{
	struct property *prop;
	const __be32 *data;
	u8 *prop_data = NULL;
	int size, i;

	prop = of_find_property(np, propname, NULL);
	if (!prop) {
		pr_err("%s: No fw data found.\n", np->name);
		return -EINVAL;
	} else if (!prop->value || prop->length < sizeof(__be32)) {
		pr_err("%s: No fw data value found.\n", np->name);
		return -ENODATA;
	}

	data = prop->value;
	size = prop->length/sizeof(__be32);

	prop_data = kzalloc((sizeof(u8) * size), GFP_KERNEL);
	if (!prop_data) {
		pr_err("kzalloc failed %d\n", __LINE__);
		return -ENOMEM;
	}


	for (i = 0; i < size; i++) {
		prop_data[i] = be32_to_cpup(data++);
		/*if (i%10 == 0)
		  pr_err("fw data 0x%02x\n", prop_data[i]);*/
	}

	fw->data = prop_data;
	fw->size = size;
	return 0;
}

static int mcu_load_fw_profile(struct stm32l011_mcu *chip)
{
	struct device_node *node = chip->dev->of_node;
	//struct device_node *fw_node;
	struct device_node *profile_node;
	struct mcu_fw *fw = &chip->fw;
	int rc = 0;

	if (chip->chg_version_gpio > 0) {
		if (gpio_get_value(chip->chg_version_gpio) == 1) {
			chip->ln8000_supported = false;
			chip->enable_slave_charger = false;
			pr_info("chg_version Detect load 44w\n");
			profile_node = of_fw_get_best_profile(node, chip->fw_id_num, "stm_mcu_firmware_data_44w");
		} else {
			chip->ln8000_supported = true;
			chip->enable_slave_charger = false;
			pr_info("chg_version Detect load LN8000\n");
			profile_node = of_fw_get_best_profile(node, chip->fw_id_num, "stm_mcu_firmware_data_ln8000");
		}
	} else {
		chip->enable_slave_charger = false;
		pr_info("chg_version not define, defalut load 44w\n");
		if (chip->ln8000_supported)
			profile_node = of_fw_get_best_profile(node, chip->fw_id_num, "stm_mcu_firmware_data_ln8000");
		else
			profile_node = of_fw_get_best_profile(node, chip->fw_id_num, "stm_mcu_firmware_data_44w");
	}

	if (IS_ERR(profile_node)) {
		rc = PTR_ERR(profile_node);
		pr_err("Failed to detect valid QG battery profile %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(profile_node, "vivo,fw-id", &fw->id);
	if (rc) {
		pr_err("Failed to read fw id,rc=%d\n", rc);
		return rc;
	}

	rc = of_fw_read_data(profile_node, "vivo,fw-data", fw);
	if (rc) {
		pr_err("Failed to read fw data,rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(profile_node, "vivo,fw-crc", &fw->crc);
	if (rc) {
		pr_err("Failed to read fw crc,rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32(profile_node, "vivo,fw-version", &fw->version);
	if (rc) {
		pr_err("Failed to read fw version,rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_string(profile_node, "vivo,fw-type", &fw->type);
	if (rc)
		fw->type = "default";

	pr_info("fw_id=%d,fw_crc=0x%02x,fw_version=0x%02x,fw_size=%d,fw_type=%s\n",
			fw->id, fw->crc, fw->version, fw->size, fw->type);

	return rc;
}
static int stm32l011_determine_initial_state(struct stm32l011_mcu *chip)
{
	int rc = 0;
	//u8 reg = 0;

	rc = mcu_load_fw_profile(chip);

	return rc;
}

static void stm32l011_check_dchg_error_state(struct stm32l011_mcu *chip)
{
	int pos = 0;

	/*init.*/
	if (chip->at_mode_slave_chg_enable_req) {
		if (chip->chg_enable && chip->slave_chg_enable) {
			memset(chip->dchg_state, 0, sizeof(chip->dchg_state));
			scnprintf(chip->dchg_state, sizeof(chip->dchg_state), "dual chg normal.");
		}
	}else{
		if (chip->chg_enable) {
			memset(chip->dchg_state, 0, sizeof(chip->dchg_state));
			scnprintf(chip->dchg_state, sizeof(chip->dchg_state), "master chg normal.");
		}
	}

	if (chip->dchg_version == BQ25970_SINGLE_DCHG || chip->dchg_version == BQ25970_DUAL_DCHG) {
		/* Get master charger exception information */
		if ((chip->int_stat_0A & 0x02))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:Conv_OCP_0x%02x|", chip->irqB); //irqB=0x41:after vac pull down; irqb=0x51:before vac pull down

		if ((chip->int_stat_0A & 0x01))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) - pos, "M:pin_diag_fail|");

		if ((chip->int_stat_0A & 0x08))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:SS_Timeout|");

		if ((chip->int_stat_0A & 0x20))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:vbus_too_low|");

		if ((chip->int_stat_0A & 0x10))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:vbus_too_high|");

		if ((chip->int_stat_11 & 0x0C))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:TSBAT_Temp_fault|");

		if ((chip->int_stat_11 & 0x0A))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:TSBUS_Temp_fault|");

		if ((chip->int_stat_0A))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:0A:0x%02x|", chip->int_stat_0A);

		/* Get slave charger exception information */
		if (chip->at_mode_slave_chg_enable_req || (chip->dchg_version == BQ25970_DUAL_DCHG && !get_atboot())) {
			if ((chip->slave_int_stat_0A & 0x02))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:Conv_OCP_0x%02x|", chip->irqB); //irqB=0x41:after vac pull down; irqb=0x51:before vac pull down

			if ((chip->slave_int_stat_0A & 0x01))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:pin_diag_fail|");

			if ((chip->slave_int_stat_0A & 0x08))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:SS_Timeout|");

			if ((chip->slave_int_stat_0A & 0x20))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:vbus_too_low|");

			if ((chip->slave_int_stat_0A & 0x10))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:vbus_too_high|");

			if ((chip->slave_int_stat_11 & 0x0C))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:TSBAT_Temp_fault|");

			if ((chip->slave_int_stat_11 & 0x0A))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:TSBUS_Temp_fault|");

			if ((chip->slave_int_stat_0A))
				pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "S:0A:0x%02x|", chip->slave_int_stat_0A);
		}
	} else if (chip->dchg_version == NXP_PCA9486_DCHG) {
		//nothing, please check it later...
	} else if(chip->dchg_version == LN8000_DCHG) {
		/* Get master charger exception information */
		if ((chip->int_stat_ln8000_06& 0x80))//irqB=0x41:after vac pull down; irqb=0x51:before vac pull down
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:Conv_OCP_0x%02x|", chip->irqB);

		if ((chip->int_stat_0A & 0x01) != 0)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) - pos, "M:pin_diag_fail|");

		if ((chip->int_stat_0A & 0x08) != 0)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:SS_Timeout|");

		if ((chip->int_stat_ln8000_05 & 0x04))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:vbus_too_low|");

		if ((chip->int_stat_ln8000_05 & 0x08))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:vbus_too_high|");

		if ((chip->int_stat_11 & 0x0C) != 0)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:TSBAT_Temp_fault|");

		if ((chip->int_stat_11 & 0x0A) != 0)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:TSBUS_Temp_fault|");

		if ((chip->int_stat_ln8000_05))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "M:05:0x%02x|", chip->int_stat_ln8000_05);

	}

	if (!chip->chg_enable) {
		if (chip->uart_tx_exception > 1 || chip->uart_rx_exception > 1)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "phone_uart_error|");

		if (chip->analog_i2c_read_exception || chip->analog_i2c_write_exception)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "MCU_Dchg_IIC_error|");

		if (chip->ap_i2c_read_exception || chip->ap_i2c_write_exception)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "AP_MCU_IIC_error|");

		if (chip->handshake_failed)
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "handshake_failed|");


		/*NOTE: keep this in the last line.*/
		if (!strncmp(chip->dchg_state, "waiting..", 9))
			pos += scnprintf(chip->dchg_state + pos, sizeof(chip->dchg_state) -pos, "waiting..%d_0x%02x_0x%02x_0x%02x_0x%02x_0x%02x_0x%02x",
					chip->usb_type, chip->irqA, chip->irqB, chip->irqC, chip->irqD, chip->reg04, chip->reg05);
	}

	pr_info("chg_enable=%d,slave_chg_enable=%d,dchg_state=%s.\n", chip->chg_enable, chip->slave_chg_enable, chip->dchg_state);
}

static void stm32l011_debug_fw_update_work(struct work_struct *work)
{
	union power_supply_propval enable = {1,};
	int ret = 0;
	u16 erase_size;
	u8 *p;
	u8 update_type = I2C_BURN;
	u32 new_crc = 0xFFFFFFFF;

	struct stm32l011_mcu *chip = container_of(work, struct stm32l011_mcu, mcu_debug_fw_update.work);

	mm_segment_t old_fs;
	struct file *fp;
	long fw_size, nread;

	pr_info("~~~\n");
	if (chip->burn_handling)
		goto RESTART;

	stm32l011_stay_awake(chip, PM_BURN);
	mutex_lock(&chip->burn_lock);
	chip->burn_handling = true;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(chip->mcu_debug_fw_name, O_RDONLY, S_IRUGO);
	if (IS_ERR(fp)) {
		pr_info("failed to open %s (%ld).\n", chip->mcu_debug_fw_name, PTR_ERR(fp));
		chip->mcu_debug_fw_update_result = mcu_debug_fw_update_result_no_such_file;
		goto out_unlock;
	}

	chip->debug_fw_updating = true;
	chip->mcu_debug_fw_update_result = mcu_debug_fw_update_result_waiting;
	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (0 < fw_size) {
		chip->mcu_debug_fw = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)chip->mcu_debug_fw, fw_size, &fp->f_pos);
		pr_info("start, file path %s, size %ld Bytes\n", chip->mcu_debug_fw_name, fw_size);

		if (nread != fw_size) {
			pr_info("failed to read firmware file, nread %ld Bytes, but fw_size %ld Bytes.\n", nread, fw_size);
			ret = -EIO;
		} else {
			if (chip->usb_present) {
				cancel_delayed_work_sync(&chip->chg_monitor_work);
				if (chip->pmi_suspend) {
					pr_err("PMI suspend durning MCU FW update, restart PMI charging...\n");
					power_supply_lite_set_property(PSYL_CHARGE,
							POWER_SUPPLY_PROP_DCHG_CHARGING_ENABLED, &enable);
				}
				stm32l011_hw_deinit(chip);
			}

			stm32l011_power_restore_enable(chip);
			msleep(100);
			stm32l011_shift_en_enable(chip, true);

			ret = stm32l011_i2c_burn_command(chip, BURN_CMD_UPDATE, &update_type, CMD_ONE_PARAM);
			if (ret < 0) {
				pr_err("burn command=0x%02X time out\n",BURN_CMD_UPDATE);
				goto OUT;
			}

			if (fw_size > FW_SIZE_MAX_KB * 1024)
				pr_err(" [Error]: MCU bin > %dK !!!!!\n", FW_SIZE_MAX_KB);
			else
				pr_info("MCU bin = %dK\n", chip->fw.size / 1024);

			erase_size = (fw_size / FLASH_PAGE_SIZE + ((fw_size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
			p = get_erase_cmd_param(chip, FLASH_APP_ADDR_SHIFT, erase_size);
			ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
			if (ret < 0) {
				pr_err("burn command=0x%02X time out\n",BURN_CMD_ERASE);
				goto OUT;
			}

			new_crc = stm32l011_i2c_burn_write(chip, true, chip->mcu_debug_fw, FLASH_APP_ADDR_SHIFT, fw_size, INSIDE_BATCH);

			p = get_crc_cmd_param(chip, new_crc, FLASH_APP_ADDR_SHIFT, fw_size);	//sxs need check CRC
			ret = stm32l011_i2c_burn_command(chip, BURN_CMD_CRC, p, CMD_CRC_PARAM);
			if (ret < 0) {
				pr_err("burn CRC time out.(crc=0x%08x)\n", new_crc);
				goto OUT;
			}
			stm32l011_power_restore_enable(chip);
			stm32l011_shift_en_enable(chip,true);
			msleep(1100);
			fw_check_version(chip);

			chip->mcu_debug_fw_update_result = mcu_debug_fw_update_result_success;
			pr_info(" MCU FW update Success (crc=0x%08x), fw_version=0x%02x !!!\n", new_crc, chip->mcu_current_fw);

			if (!chip->usb_present) {
				stm32l011_shift_en_enable(chip,false);
				stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
			} else {
				stm32l011_power_restore_enable(chip);
				stm32l011_shift_en_enable(chip,true);

				cancel_delayed_work(&chip->chg_monitor_work);
				schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(1000));
			}
		}

OUT:
		kfree(chip->mcu_debug_fw);
	}

	filp_close(fp, NULL);
	set_fs(old_fs);

out_unlock:
	if (chip->mcu_debug_fw_update_result == mcu_debug_fw_update_result_waiting)
		chip->mcu_debug_fw_update_result = mcu_debug_fw_update_result_error;
	chip->burn_handling = false;
	chip->debug_fw_updating = false;
	mutex_unlock(&chip->burn_lock);
	stm32l011_relax(chip, PM_BURN);
	return;

RESTART:
	chip->mcu_debug_fw_update_result = mcu_debug_fw_update_result_waiting;
	pr_info("Restart debug fw update after 10s, because burn_handling.\n");
	schedule_delayed_work(&chip->mcu_debug_fw_update, msecs_to_jiffies(10000));
}

static void stm32l011_burn_ctl_work(struct work_struct *work)
{
	struct stm32l011_mcu *chip = container_of(work, struct stm32l011_mcu, burn_ctl_work.work);

	stm32l011_i2c_burn_write(chip, false, chip->fw.data, FLASH_APP_ADDR_SHIFT, chip->fw.size, INSIDE_BATCH);
	chip->burn_ctl_work_state = true;
}

#ifdef CONFIG_DEBUG_FS
static int dchg_state_get(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;

	stm32l011_check_dchg_error_state(chip);
	seq_printf(m, "%s\n", chip->dchg_state);
	return 0;
}

static int dchg_state_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, dchg_state_get, chip);
}

/*
   AT+BKCHARGE=2,2
   "1,Msg/ErrorCode"
   /d/stm32l011/chg_enable, /d/stm32l011/dchg_state
   */
static const struct file_operations dchg_state_ops = {
	.owner		= THIS_MODULE,
	.open		= dchg_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mcu_fw_inside_burn_result_get(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;

	if (chip->update_done)
		seq_printf(m, "%s\n", mcu_fw_niside_burn_result_str[chip->inside_burn_result]);
	else
		seq_printf(m, "0\n");

	pr_info("update_done=%d, burn_handling=%d, inside_burn_result=%d.\n", chip->update_done, chip->burn_handling, chip->inside_burn_result);
	return 0;
}

static int mcu_fw_inside_burn_result_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, mcu_fw_inside_burn_result_get, chip);
}

/* for : AT+BK_CHG_UPD=1 */
static const struct file_operations mcu_fw_inside_burn_result_ops = {
	.owner		= THIS_MODULE,
	.open		= mcu_fw_inside_burn_result_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mcu_fw_update_show(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;

	seq_printf(m, "fw_path=%s[%s],fw_version=0x%02X\n", chip->mcu_debug_fw_name, mcu_debug_fw_update_result_str[chip->mcu_debug_fw_update_result], chip->mcu_current_fw);

	return 0;
}

static ssize_t mcu_fw_update_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct stm32l011_mcu *chip = s->private;

	int ret = count;
	char buf[100];
	int value;
	int i = 0;

	memset(buf, 0x00, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	buf[ret] = '\0';
	while(i <= ret) {
		if (buf[i] == '\n') {
			buf[i] = '\0';
			break;
		}
		i++;
	}

	if (!strncmp(buf, "/", 1)) {
		scnprintf(chip->mcu_debug_fw_name, sizeof(chip->mcu_debug_fw_name), "%s", buf);
	} else if (sscanf(buf, "%d", &value) == 1 && (value == 1)) {
		scnprintf(chip->mcu_debug_fw_name, sizeof(chip->mcu_debug_fw_name), STM32L011_DEBUG_BIN);
	} else {
		pr_info("unknow FW name (%s), skip update.\n", buf);
		return -ENOENT;
	}

	pr_info("get FW name (%s)..\n", chip->mcu_debug_fw_name);
	schedule_delayed_work(&chip->mcu_debug_fw_update, 0);
	return ret;
}

static int mcu_fw_update_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, mcu_fw_update_show, chip);
}

static const struct file_operations mcu_fw_update_ops = {
	.owner		= THIS_MODULE,
	.open		= mcu_fw_update_open,
	.read		= seq_read,
	.write		= mcu_fw_update_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dchg_ic_vendor_get(struct seq_file *m, void *data)
{
	struct stm32l011_mcu *chip = m->private;

	seq_printf(m, "%s\n", DCHG_IC_VENDOR[chip->dchg_version]);

	pr_info("dchg_ic_vendor=%s.\n", DCHG_IC_VENDOR[chip->dchg_version]);
	return 0;
}

static int dchg_ic_vendor_open(struct inode *inode, struct file *file)
{
	struct stm32l011_mcu *chip = inode->i_private;

	return single_open(file, dchg_ic_vendor_get, chip);
}

/* for : AT+BK_CHG_IC=1 //check the dchg IC vendor name.
   "ICName"  //NXP/TI
   ATOK */
static const struct file_operations dchg_ic_vendor_ops = {
	.owner		= THIS_MODULE,
	.open		= dchg_ic_vendor_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_debugfs_entries(struct stm32l011_mcu *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("stm32l011", NULL);
	if (!chip->debug_root) {
		pr_err("Couldn't create debug dir\n");
	} else {
#if 0
		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
				chip->debug_root, chip,
				&cnfg_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cnfg debug file\n");

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
				chip->debug_root, chip,
				&status_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create status debug file\n");

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
				chip->debug_root, chip,
				&cmd_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cmd debug file\n");

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
				chip->debug_root, chip,
				&irq_count_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create count debug file\n");
#endif
		ent = debugfs_create_file("mcu_fw_inside_burn_result", 0664,
				chip->debug_root, chip,
				&mcu_fw_inside_burn_result_ops);
		ent = debugfs_create_file("dchg_state", 0664,
				chip->debug_root, chip,
				&dchg_state_ops);
		ent = debugfs_create_file("fw_update", 0664,
				chip->debug_root, chip,
				&mcu_fw_update_ops);
		ent = debugfs_create_file("dchg_ic_vendor", 0664,
				chip->debug_root, chip,
				&dchg_ic_vendor_ops);
#ifdef TEST_BY_FXI_FOR_BURN
		ent = debugfs_create_file("burn_ctl", S_IRUSR | S_IWUSR,
				chip->debug_root, chip, &burn_ctl_fops);
		if (!ent)
			pr_err("Couldn't create count debug file\n");

		ent = debugfs_create_file("burn_ctl_result", 0644,
				chip->debug_root, chip,
				&burn_ctl_result_ops);
		if (!ent)
			pr_err("Couldn't create burn_ctl_result debug file\n");
#endif
#if 1
		ent = debugfs_create_file("power_usbsel", 0644,
				chip->debug_root, chip,
				&power_usbsel_fops);
		if (!ent)
			pr_err("Couldn't create power_usbsel debug file\n");

		ent = debugfs_create_file("chg_switch", 0644,
				chip->debug_root, chip,
				&chg_switch_fops);
		if (!ent)
			pr_err("Couldn't create chg_switch debug file\n");

		ent = debugfs_create_file("dchg_exit_event", 0444,
				chip->debug_root, chip,
				&dchg_exit_event_fops);
		if (!ent)
			pr_err("Couldn't create dchg_exit_event debug file\n");

		ent = debugfs_create_file("burn_success", 0444,
				chip->debug_root, chip,
				&burn_success_fops);
		if (!ent)
			pr_err("Couldn't create burn_success debug file\n");

		ent = debugfs_create_file("handshake_success", 0444,
				chip->debug_root, chip,
				&handshake_success_fops);
		if (!ent)
			pr_err("Couldn't create handshake_success debug file\n");

		ent = debugfs_create_file("in_version", 0444,
				chip->debug_root, chip,
				&inside_version_fops);
		if (!ent)
			pr_err("Couldn't create in_version debug file\n");

		ent = debugfs_create_file("out_version", 0444,
				chip->debug_root, chip,
				&outside_version_fops);
		if (!ent)
			pr_err("Couldn't create out_version debug file\n");

		ent = debugfs_create_file("ibus", 0444,
				chip->debug_root, chip,
				&ibus_fops);
		if (!ent)
			pr_err("Couldn't create ibus debug file\n");
		if (chip->enable_slave_charger) {
			ent = debugfs_create_file("slave_ibus", 0444,
					chip->debug_root, chip,
					&slave_ibus_fops);
			if (!ent)
				pr_err("Couldn't create slave ibus debug file\n");

			ent = debugfs_create_file("master_ibus", 0444,
					chip->debug_root, chip,
					&master_ibus_fops);
			if (!ent)
				pr_err("Couldn't create master ibus debug file\n");
		}
		ent = debugfs_create_file("vbus", 0444,
				chip->debug_root, chip,
				&vbus_fops);
		if (!ent)
			pr_err("Couldn't create vbus debug file\n");

		ent = debugfs_create_file("vbat", 0444,
				chip->debug_root, chip,
				&vbat_fops);
		if (!ent)
			pr_err("Couldn't create vbat debug file\n");

		ent = debugfs_create_file("cable_mohm", 0444,
				chip->debug_root, chip,
				&cable_mohm_fops);
		if (!ent)
			pr_err("Couldn't create cable_mohm debug file\n");

		ent = debugfs_create_file("req_ibus", 0444,
				chip->debug_root, chip,
				&req_ibus_fops);
		if (!ent)
			pr_err("Couldn't create req_ibus debug file\n");

		ent = debugfs_create_file("ldo_count", 0444,
				chip->debug_root, chip,
				&ldo_count_fops);
		if (!ent)
			pr_err("Couldn't create ldo_count debug file\n");


		ent = debugfs_create_file("ptmValue", 0444,
				chip->debug_root, chip,
				&ptmValue_fops);
		if (!ent)
			pr_err("Couldn't create ptmValue debug file\n");


		ent = debugfs_create_file("adjust_stop", 0444,
				chip->debug_root, chip,
				&adjust_stop_fops);
		if (!ent)
			pr_err("Couldn't create adjust_stop debug file\n");

		ent = debugfs_create_file("is_ldo", 0444,
				chip->debug_root, chip,
				&is_ldo_fops);
		if (!ent)
			pr_err("Couldn't create is_ldo debug file\n");

		ent = debugfs_create_file("bat_temp_mv", 0444,
				chip->debug_root, chip,
				&bat_temp_mv_fops);
		if (!ent)
			pr_err("Couldn't create bat_temp_mv debug file\n");


		if (chip->bat_temp_exist) {
			ent = debugfs_create_file("bat_temp", 0444,
					chip->debug_root, chip,
					&bat_temp_fops);
			if (!ent)
				pr_err("Couldn't create bat_temp debug file\n");
		}

		ent = debugfs_create_file("bat_conn_temp", 0444,
				chip->debug_root, chip,
				&bat_conn_temp_fops);
		if (!ent)
			pr_err("Couldn't create bat_conn_temp debug file\n");

		if (chip->enable_slave_charger) {
			ent = debugfs_create_file("master_bat_conn_temp", 0444,
					chip->debug_root, chip,
					&master_bat_conn_temp_fops);
			if (!ent)
				pr_err("Couldn't create master_bat_conn_temp debug file\n");
		}

		ent = debugfs_create_file("usb_conn_temp", 0444,
				chip->debug_root, chip,
				&usb_conn_temp_fops);
		if (!ent)
			pr_err("Couldn't create usb_conn_temp debug file\n");

		ent = debugfs_create_file("adapter_temp", 0444,
				chip->debug_root, chip,
				&adapter_temp_fops);
		if (!ent)
			pr_err("Couldn't create adapter_temp debug file\n");

		ent = debugfs_create_file("adapter_conn_temp", 0444,
				chip->debug_root, chip,
				&adapter_conn_temp_fops);
		if (!ent)
			pr_err("Couldn't create adapter_conn_temp debug file\n");

		ent = debugfs_create_file("dp", 0444,
				chip->debug_root, chip,
				&dp_mv_fops);
		if (!ent)
			pr_err("Couldn't create dp_mv debug file\n");

		ent = debugfs_create_file("dm", 0444,
				chip->debug_root, chip,
				&dm_mv_fops);
		if (!ent)
			pr_err("Couldn't create dm_mv debug file\n");

		ent = debugfs_create_file("cable_matched", 0444,
				chip->debug_root, chip,
				&cable_matched_fops);
		if (!ent)
			pr_err("Couldn't create cable_matched debug file\n");

		ent = debugfs_create_file("chg_enable", 0444,
				chip->debug_root, chip,
				&chg_enable_fops);
		if (!ent)
			pr_err("Couldn't create chg_enable debug file\n");
		if (chip->enable_slave_charger) {
			ent = debugfs_create_file("slave_chg_enable", 0444,
					chip->debug_root, chip,
					&slave_chg_enable_fops);
			if (!ent)
				pr_err("Couldn't create slave chg enable debug file\n");

			ent = debugfs_create_file("master_chg_enable", 0444,
					chip->debug_root, chip,
					&master_chg_enable_fops);
			if (!ent)
				pr_err("Couldn't create master chg enable debug file\n");

			ent = debugfs_create_file("slave_chg_enable_flag", 0644,
					chip->debug_root, chip,
					&slave_chg_enable_flag_fops);
			if (!ent)
				pr_err("Couldn't create slave_chg_enable_flag debug file\n");
		}
		ent = debugfs_create_file("analog_i2c_read_count", 0444,
				chip->debug_root, chip,
				&analog_i2c_read_count_fops);
		if (!ent)
			pr_err("Couldn't create analog_i2c_read_count debug file\n");
		ent = debugfs_create_file("analog_i2c_write_count", 0444,
				chip->debug_root, chip,
				&analog_i2c_write_count_fops);
		if (!ent)
			pr_err("Couldn't create analog_i2c_write_count debug file\n");
		ent = debugfs_create_file("uart_tx_count", 0444,
				chip->debug_root, chip,
				&uart_tx_count_fops);
		if (!ent)
			pr_err("Couldn't create uart_tx_count debug file\n");
		ent = debugfs_create_file("uart_rx_count", 0444,
				chip->debug_root, chip,
				&uart_rx_count_fops);
		if (!ent)
			pr_err("Couldn't create uart_rx_count debug file\n");
		ent = debugfs_create_file("ap_i2c_read_count", 0444,
				chip->debug_root, chip,
				&ap_i2c_read_count_fops);
		if (!ent)
			pr_err("Couldn't create ap_i2c_read_count debug file\n");
		ent = debugfs_create_file("ap_i2c_write_count", 0444,
				chip->debug_root, chip,
				&ap_i2c_write_count_fops);
		if (!ent)
			pr_err("Couldn't create ap_i2c_write_count debug file\n");
		ent = debugfs_create_file("adapter_i2c_read_count", 0444,
				chip->debug_root, chip,
				&adapter_i2c_read_count_fops);
		if (!ent)
			pr_err("Couldn't create adapter_i2c_read_count debug file\n");
		ent = debugfs_create_file("adapter_i2c_write_count", 0444,
				chip->debug_root, chip,
				&adapter_i2c_write_count_fops);
		if (!ent)
			pr_err("Couldn't create adapter_i2c_write_count debug file\n");
		ent = debugfs_create_file("adapter_uart_tx_count", 0444,
				chip->debug_root, chip,
				&adapter_uart_tx_count_fops);
		if (!ent)
			pr_err("Couldn't create adapter_uart_tx_count debug file\n");
		ent = debugfs_create_file("adapter_uart_rx_count", 0444,
				chip->debug_root, chip,
				&adapter_uart_rx_count_fops);
		if (!ent)
			pr_err("Couldn't create adapter_uart_rx_count debug file\n");
		ent = debugfs_create_file("adapter_status_monitor", 0444,
				chip->debug_root, chip,
				&adapter_status_monitor_fops);
		if (!ent)
			pr_err("Couldn't create adapter_status_monitor debug file\n");
#if OLD_CAM_NOTIFY
		ent = debugfs_create_file("flashlight_test", 0664, chip->debug_root, chip, &flashlight_test_ops);
		if (!ent)
			pr_err("Couldn't create flashlight_test debug file\n");
#endif
		ent = debugfs_create_file("sbu_gpio", 0664, chip->debug_root, chip, &sbu_gpio_ops);
		if (!ent)
			pr_err("Couldn't create sbu_gpio debug file\n");

		ent = debugfs_create_file("sbu_info", 0444, chip->debug_root, chip, &sbu_info_fops);
		if (!ent)
			pr_err("Couldn't create sbu_info debug file\n");
#endif
	}
	return 0;
}
#endif

#if 1//stm32 debug class
static int burn_ctl_init(struct stm32l011_mcu *chip)
{
	int rc = 0;

	if (chip->test_data) {
		pr_info("init.\n");
		if (!get_atboot())
			chip->burn_handling = true;
		chip->burn_ctl_work_state = false;
		chip->test_data->type = I2C_BURN;
		chip->test_data->dir_chg_switch = true;
		chip->test_data->burn_from_user = false;
		chip->test_data->voltage = 5080;
		chip->test_data->icurrent = 2000;
		chip->test_data->impedance = 300;
		chip->test_data->dchg_switch = 1;
		chip->test_data->crc32 = 0xFFFFFFFF;
		chip->test_data->size = TRANSFER_BATCH_SIZE;
		memset(chip->test_data->buffer, 0x0, TRANSFER_BATCH_SIZE);
	} else {
		rc = -ENODEV;
	}

	return rc;
}
static int burn_ctl_deinit(struct stm32l011_mcu *chip)
{
	pr_info("deinit.\n");

	if (chip) {
		if (!get_atboot())
			chip->burn_handling = false;
	}

	return 0;
}

static ssize_t mcu_fw_inside_burn_result_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("update_done=%d,burn_handling=%d,inside_burn_result=%d.\n",
			chip->update_done, chip->burn_handling, chip->inside_burn_result);

	if (chip->update_done)
		return scnprintf(buf, PAGE_SIZE, "%s\n", mcu_fw_niside_burn_result_str[chip->inside_burn_result]);
	else
		return scnprintf(buf, PAGE_SIZE, "0\n");
}
static CLASS_ATTR_RO(mcu_fw_inside_burn_result);

static ssize_t dchg_state_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	stm32l011_check_dchg_error_state(chip);

	return scnprintf(buf, PAGE_SIZE, "%s\n", chip->dchg_state);
}
static CLASS_ATTR_RO(dchg_state);

static ssize_t fw_update_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "fw_path=%s[%s],fw_version=0x%02X\n",
			chip->mcu_debug_fw_name, mcu_debug_fw_update_result_str[chip->mcu_debug_fw_update_result],
			chip->mcu_current_fw);
}
static ssize_t fw_update_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int rc = count;
	int value, i = 0;
	char buffer[100];

	memset(buffer, 0x00, sizeof(buffer));
	memcpy(buffer, buf, min_t(size_t, sizeof(buffer) - 1, count));

	buffer[rc] = '\0';
	while (i <= rc) {
		if (buffer[i] == '\n') {
			buffer[i] = '\0';
			break;
		}
		i++;
	}

	if (!strncmp(buffer, "/", 1)) {
		scnprintf(chip->mcu_debug_fw_name, sizeof(chip->mcu_debug_fw_name), "%s", buffer);
	} else if (sscanf(buffer, "%d", &value) == 1 && (value == 1)) {
		scnprintf(chip->mcu_debug_fw_name, sizeof(chip->mcu_debug_fw_name), STM32L011_DEBUG_BIN);
	} else {
		pr_info("unknow FW name (%s), skip update.\n", buffer);
		return -ENOENT;
	}

	pr_info("get FW name (%s)..\n", chip->mcu_debug_fw_name);
	schedule_delayed_work(&chip->mcu_debug_fw_update, 0);

	return rc;
}
static CLASS_ATTR_RW(fw_update);

static ssize_t dchg_ic_vendor_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("dchg_ic_vendor=%s.\n", DCHG_IC_VENDOR[chip->dchg_version]);

	return scnprintf(buf, PAGE_SIZE, "%s\n", DCHG_IC_VENDOR[chip->dchg_version]);
}
static CLASS_ATTR_RO(dchg_ic_vendor);

static ssize_t burn_ctl_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	u8 result;
	int rc;

	rc = burn_ctl_init(chip);
	if (rc) {
		pr_err("data null, rc = %d\n", rc);
		return 0;
	}

	pr_info("read : %d\n", chip->test_data->opt_ret);

	if (chip->test_data->opt_ret < 0)
		result = 0xFF;
	else
		result = (chip->test_data->opt_ret & 0xFF);

	rc = burn_ctl_deinit(chip);
	if (rc)
		pr_err("failed, rc = %d\n", rc);

	return scnprintf(buf, PAGE_SIZE, "%u\n", result);
}
static ssize_t burn_ctl_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	u8 cmd = BURN_CMD_UNKNOW, reg_value = 0x0;
	u32 len, i;
	u16 erase_size = 0;
	u8 *p;
	int rc;

	rc = burn_ctl_init(chip);
	if (rc) {
		pr_err("data null, rc = %d\n", rc);
		return 0;
	}

	len = min((u32)count, MAX_FW_SIZE);
	memcpy(chip->test_data->buffer, buf, len);

	if (len >= OUTSIDE_BATCH) {
		chip->test_data->size = len;
		chip->test_data->burn_from_user = true;
		cmd = BURN_CMD_WRITE;
		pr_info("write select\n");
	} else {
		chip->test_data->burn_from_user = false;
		cmd = chip->test_data->buffer[0];
		pr_info("cmd:0x%02x", cmd);
		for (i=1; i<len; i++)
			pr_info(",0x%02x", chip->test_data->buffer[i]);
		pr_info("\n");
	}

	switch (cmd) {
	case BURN_CMD_WRITE:
		pr_info("test write, type(%d)\n", chip->test_data->type);
		if (chip->test_data->burn_from_user) {
			if (I2C_BURN == chip->test_data->type) {
				chip->test_data->crc32 = stm32l011_i2c_burn_write(chip, true,
						chip->test_data->buffer, FLASH_APP_ADDR_SHIFT, chip->test_data->size, INSIDE_BATCH);
			} else {
				chip->test_data->crc32 = stm32l011_i2c_burn_write(chip, true,
						chip->test_data->buffer, FLASH_APP_ADDR_SHIFT, chip->test_data->size, OUTSIDE_BATCH);
			}
		} else {
			if (I2C_BURN == chip->test_data->type) {
#if 0
				chip->test_data->opt_ret = stm32l011_i2c_burn_write(chip, false,
						chip->fw.data, FLASH_APP_ADDR_SHIFT, chip->fw.size, INSIDE_BATCH);
#else
				cancel_delayed_work_sync(&chip->burn_ctl_work);
				chip->burn_ctl_work_state = false;
				schedule_delayed_work(&chip->burn_ctl_work, 0);
#endif
				chip->test_data->crc32 = chip->fw.crc;
				chip->test_data->size = chip->fw.size;
			}
		}
		break;
	case BURN_CMD_ERASE:
		if (len >= 5) {
			erase_size = ((chip->test_data->buffer[4]<<24) | (chip->test_data->buffer[3]<<16)
					| (chip->test_data->buffer[2]<<8) | chip->test_data->buffer[1]);
			erase_size = (erase_size / FLASH_PAGE_SIZE + ((erase_size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
		} else {
			if (I2C_BURN == chip->test_data->type)
				erase_size = (chip->fw.size / FLASH_PAGE_SIZE + ((chip->fw.size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
		}
		pr_info("test erase[%d], type(%d)\n", erase_size, chip->test_data->type);
		p = get_erase_cmd_param(chip, FLASH_APP_ADDR_SHIFT, erase_size);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
		break;
	case BURN_CMD_JUMP:
		pr_info("test jump, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_JUMP);
		/* reset value after JUMP */
		mutex_lock(&chip->parameters_lock);
		chip->init_done = false;
		chip->exit = false;
		chip->handshake_success = false;
		chip->handshake_failed = false;
		mutex_unlock(&chip->parameters_lock);

		if (chip->usb_present) {
			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, 0);
		}
		break;
	case BURN_CMD_CRC:
		pr_info("test crc, type(%d)\n", chip->test_data->type);
		p = get_crc_cmd_param(chip, chip->test_data->crc32, FLASH_APP_ADDR_SHIFT, chip->test_data->size);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_CRC, p, CMD_CRC_PARAM);
		if (chip->test_data->opt_ret < 0) {
			chip->burn_success = false;
			pr_info("crc check error\n");
		} else {
			chip->burn_success = true;
			pr_info("crc check success\n");
		}
		break;
	case BURN_CMD_BOOT_VERSION:
		pr_info("test read boot_version, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_BOOT_VERSION, &chip->test_data->type, CMD_ONE_PARAM);
		pr_info("check MCU BL version=0x%x, MCU BL vendor=0x%x\n", chip->mcu_bl_version, chip->mcu_bl_vendor);
		break;
	case BURN_CMD_UPDATE:
		pr_info("test update, type(%d)\n", chip->test_data->type);
		cancel_delayed_work_sync(&chip->chg_monitor_work);
		if (I2C_BURN == chip->test_data->type) {
			stm32l011_power_restore_enable(chip);
			msleep(100);
		}
		chip->test_data->opt_ret = stm32l011_i2c_burn_command(chip, BURN_CMD_UPDATE, &chip->test_data->type, CMD_ONE_PARAM);
		break;
	case BURN_CMD_APP_UPDATE:
		pr_info("test app_update, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_APP_UPDATE);
		break;
	case BURN_CMD_APP_VERSION:
		pr_info("test app_version, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_APP_VERSION);
		break;
	case BURN_CMD_TEST_I2C:
		pr_info("test set i2c_burn, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = chip->test_data->type = I2C_BURN;
		stm32l011_usbsel_enable(chip, false);
		break;
	case BURN_CMD_TEST_SERIAL:
		pr_info("test set serial_burn, type(%d)\n", chip->test_data->type);
		chip->test_data->opt_ret = chip->test_data->type = SERIAL_BURN;
		stm32l011_usbsel_enable(chip, true);
		break;
	case TEST_DIRECT_CHARGE_SWITCH:
		if (len >= 2) {
			chip->test_data->dchg_switch = chip->test_data->buffer[1];
			if (0 == chip->test_data->dchg_switch)
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, MCU_DCHG_SWITCH);
			else
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_DCHG_SWITCH, 0);
		} else {
			chip->test_data->opt_ret = -1;
		}
		break;
	case TEST_SET_VOLTAGE:
#if 1
		pr_info("skip set voltage\n");
#else
		if (len >= 5) {
			chip->test_data->voltage = ((chip->test_data->buffer[4]<<24) | (chip->test_data->buffer[3]<<16)
					| (chip->test_data->buffer[2]<<8) | chip->test_data->buffer[1]);
			if (chip->test_data->voltage < 3500 || chip->test_data->voltage > 5500) {
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_VOLTAGE, 0);
			} else {
				reg_value = (chip->test_data->voltage / 500);
				pr_info("test set voltage(%d), type(%d)\n", chip->test_data->voltage, chip->test_data->type);
				chip->test_data->opt_ret = stm32l011_write_reg(chip, MCU_ADAPTER_VOLTAGE, reg_value);
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_VOLTAGE, MCU_SET_VOLTAGE);
			}
		} else {
			chip->test_data->opt_ret = -1;
		}
#endif
		break;
	case TEST_SET_CURRENT:
		if (len >= 5) {
			chip->test_data->icurrent = ((chip->test_data->buffer[4]<<24) | (chip->test_data->buffer[3]<<16)
					| (chip->test_data->buffer[2]<<8) | chip->test_data->buffer[1]);

			/*
			 * Factory test -- direct charging outputs 2.5A:
			 * In AT mode, the device is usually incompelete, which causes higher system power consumption.
			 * Then ibat is low(close to 2A), the results can be confused with "normal 5v/2a charging".
			 * Therefore, here we request a bit more current to counterbalance system power consumption.
			 * If "vivo,test-set-current" isn't defined is dtsi, the default current is set to 2600ma.
			 */
			if (chip->test_data->icurrent == 2500) {
				pr_err("atboot=%d, AT request ibat=%dmA, change it to 2700mA (add 200mA margin for System Current Consume).\n",
						get_atboot(), chip->test_data->icurrent);
				chip->test_data->icurrent = 2600;
			}

			chip->test_data->opt_ret = force_active_votable(chip->dchg_fcc_votable, FACTORY_FCC_VOTER, true, chip->test_data->icurrent);
			if (chip->test_data->opt_ret < 0)
				pr_err("Couldn't vote fastchg ma ret = %d\n", chip->test_data->opt_ret);
		} else {
			chip->test_data->opt_ret = -1;
		}
		break;
	case TEST_SET_IMPEDANCE:
		if (len >= 5) {
			chip->test_data->impedance = ((chip->test_data->buffer[4]<<24) | (chip->test_data->buffer[3]<<16)
					| (chip->test_data->buffer[2]<<8) | chip->test_data->buffer[1]);
			if (chip->test_data->impedance <= 300 || chip->test_data->impedance >= 1000) {
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_IMPEDANCE, 0);
			} else {
				reg_value = (chip->test_data->impedance / 100);
				pr_info("test set impedance(%d), type(%d)\n", chip->test_data->impedance, chip->test_data->type);
				chip->test_data->opt_ret = stm32l011_write_reg(chip, MCU_ADAPTER_IMPEDANCE, reg_value);
				chip->test_data->opt_ret = stm32l011_masked_write(chip, MCU_REG_04, MCU_SET_IMPEDANCE, MCU_SET_IMPEDANCE);
			}
		} else {
			chip->test_data->opt_ret = -1;
		}
		break;
	default:
		pr_info("burn cmd not support\n");
		break;
	}

	rc = burn_ctl_deinit(chip);
	if (rc)
		pr_err("failed, ret = %d\n", rc);

	return len;
}
static CLASS_ATTR_RW(burn_ctl);

static ssize_t burn_ctl_result_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->burn_ctl_work_state);
}
static CLASS_ATTR_RO(burn_ctl_result);

static ssize_t power_usbsel_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int power_state = 0, usbsel_state = 0;

	power_state = stm32l011_get_power_gpio_state(chip);
	usbsel_state = stm32l011_get_usbsel_state(chip);

	return scnprintf(buf, PAGE_SIZE, "0x%02llx\n", (int)((power_state << 4) | usbsel_state));
}
static ssize_t power_usbsel_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (1 == val) {
		pr_info("setting usb psy dp=f dm=f\n");
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_HIGH);
		stm32l011_usbsel_enable(chip, 1);
	} else {
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
		stm32l011_usbsel_enable(chip, 0);
		pr_info("setting usb psy dp=r dm=r\n");
	}

	return count;
}
static CLASS_ATTR_RW(power_usbsel);

static ssize_t chg_switch_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->chg_switch);
}
static ssize_t chg_switch_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int val;
	union power_supply_propval value = {0,};

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (chip->burn_handling || chip->dchg_pon_reset) {
		pr_err("burn_handling\n");
		return 0;
	}

	//chip->burn_handling = true;
	if (0 == val) {
		pr_info("suspend charging\n");
		chip->chg_switch = false;
		stm32l011_dchg_enable(chip, false);

		value.intval = 0;
		power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
	} else if (1 == val) {
		pr_info("resume charging\n");
		cancel_delayed_work(&chip->chg_monitor_work);
		chip->chg_switch = true;

		value.intval = 1;
		power_supply_lite_set_property(PSYL_CHARGE, POWER_SUPPLY_PROP_BSP_CHG_CUTOFF, &value);
		stm32l011_dchg_enable(chip, true);
	}
	//chip->burn_handling = false;
	return count;
}
static CLASS_ATTR_RW(chg_switch);

static ssize_t dchg_exit_event_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->exit_event);
}
static CLASS_ATTR_RO(dchg_exit_event);

static ssize_t burn_success_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!chip->burn_success);
}
static CLASS_ATTR_RO(burn_success);

static ssize_t handshake_success_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->handshake_success ? 1 : 0);
}
static CLASS_ATTR_RO(handshake_success);

static ssize_t in_version_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	u8 reg_value = 0x0;

	stm32l011_read_reg(chip, MCU_VERSION_END, &reg_value);

	return scnprintf(buf, PAGE_SIZE, "%u\n", reg_value);
}
static CLASS_ATTR_RO(in_version);

static ssize_t out_version_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	u8 reg_value = 0x0;

	stm32l011_read_reg(chip, MCU_ADAPTER_VERSION, &reg_value);

	return scnprintf(buf, PAGE_SIZE, "%u\n", reg_value);
}
static CLASS_ATTR_RO(out_version);

static ssize_t ibus_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->chg_enable ? chip->ibus_ma : 0);
}
static CLASS_ATTR_RO(ibus);

static ssize_t vbus_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->vbus_mv);
}
static CLASS_ATTR_RO(vbus);

static ssize_t vbat_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->vbat_mv);
}
static CLASS_ATTR_RO(vbat);

static ssize_t cable_mohm_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->cable_mohm);
}
static CLASS_ATTR_RO(cable_mohm);

static ssize_t req_ibus_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->req_ibus);
}
static CLASS_ATTR_RO(req_ibus);

static ssize_t ldo_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->ldo_count);
}
static CLASS_ATTR_RO(ldo_count);

static ssize_t ptmValue_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->ptmValue);
}
static CLASS_ATTR_RO(ptmValue);

static ssize_t adjust_stop_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->adjust_stop);
}
static CLASS_ATTR_RO(adjust_stop);

static ssize_t is_ldo_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->is_ldo);
}
static CLASS_ATTR_RO(is_ldo);

static ssize_t bat_temp_mv_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->bat_temp_mv);
}
static CLASS_ATTR_RO(bat_temp_mv);

#define BACKUP_TEMP_TIMER	60	//s
static ssize_t bat_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int temp = chip->bat_temp;

	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		temp = chip->bat_temp;
	} else if (chip->bat_temp_backup > 0)
		temp = chip->bat_temp_backup;

	pr_err("bat_temp_backup=%d,bat_temp=%d,temp=%d.dchg_continue_time=%d\n",
			chip->bat_temp_backup, chip->bat_temp, temp, chip->dchg_continue_time);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static CLASS_ATTR_RO(bat_temp);

static ssize_t bat_conn_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int temp = chip->bat_conn_temp;

	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		temp = chip->bat_conn_temp;
	} else if (chip->bat_conn_temp_backup > 0)
		temp = chip->bat_conn_temp_backup;

	pr_err("bat_conn_temp_backup=%d,bat_conn_temp=%d,temp=%d.dchg_continue_time=%d\n",
			chip->bat_conn_temp_backup, chip->bat_conn_temp, temp, chip->dchg_continue_time);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static CLASS_ATTR_RO(bat_conn_temp);

static ssize_t master_bat_conn_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int temp = chip->master_bat_conn_temp;

	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		temp = chip->master_bat_conn_temp;
	} else if (chip->master_bat_conn_temp_backup > 0)
		temp = chip->master_bat_conn_temp_backup;

	pr_err("master_bat_conn_temp_backup=%d,master_bat_conn_temp=%d,temp=%d.dchg_continue_time=%d\n",
			chip->master_bat_conn_temp_backup, chip->master_bat_conn_temp, temp, chip->dchg_continue_time);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static CLASS_ATTR_RO(master_bat_conn_temp);

static ssize_t usb_conn_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int temp = chip->usb_conn_temp;

	if (chip->dchg_continue_time > BACKUP_TEMP_TIMER) {
		temp = chip->usb_conn_temp;
	} else if (chip->usb_conn_temp_backup > 0)
		temp = chip->usb_conn_temp_backup;

	pr_err("usb_conn_temp_backup=%d,usb_conn_temp=%d,temp=%d.dchg_continue_time=%d\n",
			chip->usb_conn_temp_backup, chip->usb_conn_temp, temp, chip->dchg_continue_time);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp);
}
static CLASS_ATTR_RO(usb_conn_temp);

static ssize_t adapter_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->adapter_temp);
}
static CLASS_ATTR_RO(adapter_temp);

static ssize_t adapter_conn_temp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->adapter_conn_temp);
}
static CLASS_ATTR_RO(adapter_conn_temp);

static ssize_t dp_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->dp_mv);
}
static CLASS_ATTR_RO(dp);

static ssize_t dm_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->dm_mv);
}
static CLASS_ATTR_RO(dm);

static ssize_t cable_matched_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%s.sbu1:%d,sbu2:%d. FFC:%d, Cout:%d\n",
			sbu_cable_id_type_strings[chip->sbu_cable_id], chip->sbu1_adc, chip->sbu2_adc, chip->ex_fg_ffc_support, chip->battery_cout_counter);
}
static CLASS_ATTR_RO(cable_matched);

static ssize_t chg_enable_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	bool chg_enable = 0;

	pr_info("real_vbat_mv=%d, batt_capacity=%d, user_dchg_current=%d\n",
			chip->real_vbat_mv, chip->batt_capacity, chip->user_dchg_current);

	if (chip->enable_slave_charger) {
		if ((get_atboot() && !chip->at_mode_slave_chg_enable_req) || chip->real_vbat_mv > 4200
				|| chip->batt_capacity >=80 || chip->user_dchg_current < 3800)
			chg_enable = chip->chg_enable;
		else
			chg_enable = chip->chg_enable && chip->slave_chg_enable;
	} else {
		chg_enable = chip->chg_enable;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_enable);
}
static CLASS_ATTR_RO(chg_enable);

static ssize_t analog_i2c_read_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->analog_i2c_read_exception);
}
static CLASS_ATTR_RO(analog_i2c_read_count);

static ssize_t analog_i2c_write_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->analog_i2c_write_exception);
}
static CLASS_ATTR_RO(analog_i2c_write_count);

static ssize_t uart_tx_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->uart_tx_exception);
}
static CLASS_ATTR_RO(uart_tx_count);

static ssize_t uart_rx_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->uart_rx_exception);
}
static CLASS_ATTR_RO(uart_rx_count);

static ssize_t ap_i2c_read_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->ap_i2c_read_exception);
}
static CLASS_ATTR_RO(ap_i2c_read_count);

static ssize_t ap_i2c_write_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->ap_i2c_write_exception);
}
static CLASS_ATTR_RO(ap_i2c_write_count);

static ssize_t adapter_i2c_read_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	// return scnprintf(buf, PAGE_SIZE, "%d\n", chip->adapter_i2c_read_exception);
	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->req_vbus);
}
static CLASS_ATTR_RO(adapter_i2c_read_count);

static ssize_t adapter_i2c_write_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	// return scnprintf(buf, PAGE_SIZE, "%d\n", chip->adapter_i2c_write_exception);
	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->req_ibus);
}
static CLASS_ATTR_RO(adapter_i2c_write_count);

static ssize_t adapter_uart_tx_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->adapter_uart_tx_exception);
}
static CLASS_ATTR_RO(adapter_uart_tx_count);

static ssize_t adapter_uart_rx_count_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->adapter_uart_rx_exception);
}
static CLASS_ATTR_RO(adapter_uart_rx_count);

static ssize_t adapter_status_monitor_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->adapter_status);
}
static CLASS_ATTR_RO(adapter_status_monitor);

/* extra attributes: may be added to sysfs only in certain condition */
/* if (enable_slave_charger) */
static ssize_t slave_ibus_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("%d,%d\n", chip->slave_chg_enable, chip->slave_ibus_ma);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			chip->slave_chg_enable ? chip->slave_ibus_ma : 0);
}
static CLASS_ATTR_RO(slave_ibus);

static ssize_t master_ibus_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("%d, %d, %d, %d\n",
			chip->chg_enable, chip->slave_chg_enable, chip->ibus_ma, chip->slave_ibus_ma);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->chg_enable ?
			(chip->slave_chg_enable ? chip->ibus_ma - chip->slave_ibus_ma : chip->ibus_ma) : 0);
}
static CLASS_ATTR_RO(master_ibus);

static ssize_t slave_chg_enable_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("%d,%d\n", chip->slave_chg_enable, chip->slave_ibus_ma);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->slave_chg_enable);
}
static CLASS_ATTR_RO(slave_chg_enable);

static ssize_t master_chg_enable_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	pr_info("%d, %d\n", chip->chg_enable, chip->ibus_ma);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->chg_enable);
}
static CLASS_ATTR_RO(master_chg_enable);

static ssize_t slave_chg_enable_flag_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	pr_info("val = %d\n", val);

	if (1 == val)
		chip->at_mode_slave_chg_enable_req = 1;
	else
		chip->at_mode_slave_chg_enable_req = 0;

	return count;
}
static ssize_t slave_chg_enable_flag_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);

	return scnprintf(buf, PAGE_SIZE, "%u\n", chip->at_mode_slave_chg_enable_req);
}
static CLASS_ATTR_RW(slave_chg_enable_flag);

static ssize_t update_sbu_adc_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int i = 0;
	ssize_t size = 0;

	for (i = 0; i < SBU_TEST_CASE_MAX; i++) {
		size += scnprintf((buf+size), PAGE_SIZE, "%s:[%d,%d]\n",
				sbu_switcher_test_case_str[i], chip->sbu_adc[i].sbu1, chip->sbu_adc[i].sbu2);
	}

	return size;
}
static CLASS_ATTR_RO(update_sbu_adc);

static ssize_t sbu_off_adc_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int rc = 0, adc_result = 0;
	ssize_t size = 0;

	if (!chip->sbu_cable_id_detect_enable) {
		goto out;
	}

	max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_HIZ);

	if (gpio_get_value(chip->sbu_pwr_gpio) == 0) {
		pr_info("sbu_pwr_gpio ctrl output high!\n");
		gpio_direction_output(chip->sbu_pwr_gpio, 1);
		msleep(300);
	}

	rc = stm32l011_adc_read(chip, chip->iio.sbu1_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu1_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_OFF].sbu1 = 0;
		goto out;
	}
	chip->sbu_adc[SBU_OFF].sbu1 = adc_result / 1000;
	msleep(50);

	rc = stm32l011_adc_read(chip, chip->iio.sbu2_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu2_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_OFF].sbu2= 0;
		goto out;
	}
	chip->sbu_adc[SBU_OFF].sbu2= adc_result / 1000;
	gpio_direction_output(chip->sbu_pwr_gpio, 0);

	size = scnprintf(buf, PAGE_SIZE, "%d,%d\n", chip->sbu_adc[SBU_OFF].sbu1, chip->sbu_adc[SBU_OFF].sbu2);
	pr_debug("%s: %s. usb_present=%d\n", sbu_switcher_test_case_str[SBU_OFF], buf, chip->usb_present);
out:
	return size;
}
static CLASS_ATTR_RO(sbu_off_adc);

static ssize_t sbu_direct_on_adc_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int rc = 0, adc_result = 0;
	ssize_t size = 0;

	if (!chip->sbu_cable_id_detect_enable) {
		goto out1;
	}

	max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_DIRECT_CONNECT);
	msleep(100);

	if (gpio_get_value(chip->sbu_pwr_gpio) == 0) {
		pr_info("sbu_pwr_gpio ctrl output high!\n");
		gpio_direction_output(chip->sbu_pwr_gpio, 1);
		msleep(300);
	}

	rc = stm32l011_adc_read(chip, chip->iio.sbu1_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu1_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_DIRECT_ON].sbu1 = 0;
		goto out2;
	}
	chip->sbu_adc[SBU_DIRECT_ON].sbu1 = adc_result / 1000;
	msleep(50);

	rc = stm32l011_adc_read(chip, chip->iio.sbu2_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu2_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_DIRECT_ON].sbu2= 0;
		goto out2;
	}
	chip->sbu_adc[SBU_DIRECT_ON].sbu2= adc_result / 1000;
	gpio_direction_output(chip->sbu_pwr_gpio, 0);

	size = scnprintf(buf, PAGE_SIZE, "%d,%d\n", chip->sbu_adc[SBU_DIRECT_ON].sbu1, chip->sbu_adc[SBU_DIRECT_ON].sbu2);
out2:
	max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_HIZ);
	pr_debug("%s: %s. usb_present=%d\n", sbu_switcher_test_case_str[SBU_DIRECT_ON], buf, chip->usb_present);
out1:
	return size;
}
static CLASS_ATTR_RO(sbu_direct_on_adc);

static ssize_t sbu_flip_on_adc_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int rc = 0, adc_result = 0;
	ssize_t size = 0;

	if (!chip->sbu_cable_id_detect_enable) {
		goto out1;
	}

	max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_DIRECT_CONNECT);
	msleep(100);

	if (gpio_get_value(chip->sbu_pwr_gpio) == 0) {
		pr_info("sbu_pwr_gpio ctrl output high!\n");
		gpio_direction_output(chip->sbu_pwr_gpio, 1);
		msleep(300);
	}

	rc = stm32l011_adc_read(chip, chip->iio.sbu1_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu1_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_FLIP_ON].sbu1 = 0;
		goto out2;
	}
	chip->sbu_adc[SBU_FLIP_ON].sbu1 = adc_result / 1000;
	msleep(10);

	rc = stm32l011_adc_read(chip, chip->iio.sbu2_adc_chan, &adc_result);
	if (rc < 0) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		pr_err("sbu2_cable_id_adc read fail, rc=%d\n", rc);
		chip->sbu_adc[SBU_FLIP_ON].sbu2= 0;
		goto out2;
	}
	chip->sbu_adc[SBU_FLIP_ON].sbu2= adc_result / 1000;
	gpio_direction_output(chip->sbu_pwr_gpio, 0);

	size = scnprintf(buf, PAGE_SIZE, "%d,%d\n", chip->sbu_adc[SBU_FLIP_ON].sbu1, chip->sbu_adc[SBU_FLIP_ON].sbu2);
out2:
	max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_HIZ);
	pr_debug("%s: %s. usb_present=%d\n", sbu_switcher_test_case_str[SBU_FLIP_ON], buf, chip->usb_present);
out1:
	return size;
}
static CLASS_ATTR_RO(sbu_flip_on_adc);

static ssize_t sbu_info_show(struct class *c, struct class_attribute *attr, char *buf)
{
	struct stm32l011_mcu *chip = container_of(c, struct stm32l011_mcu, stm32_debug);
	int cabel_A = 0;
	ssize_t size = 0;
	const struct sbu_r_adc * sub_normal_radc = sub_normal_radc_rely_switch;
	const struct sbu_r_adc * sub_error_radc = sub_error_radc_rely_switch;

	if (!chip->sbu_rely_switch) {
		sub_normal_radc = sub_normal_radc_no_switch;
		sub_error_radc = sub_error_radc_no_switch;
	}

	switch (chip->sbu_cable_id) {
	case SBU_CABLE_ID_DEFAULT:
		cabel_A = 3;
		break;
	case SBU_CABLE_ID_4A:
		cabel_A = 4;
		break;
	case SBU_CABLE_ID_5A:
		cabel_A = 5;
		break;
	case SBU_CABLE_ID_6A:
		cabel_A = 6;
		break;
	case SBU_CABLE_ID_8A:
		cabel_A = 8;
		break;
	case SBU_CABLE_ID_UNKNOW:
	default:
		cabel_A = 0;
		break;
	}

	if (chip->cable_id_detect_done == 1) {
		size = scnprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d", cabel_A, chip->sbu1_adc, chip->sbu2_adc,
				sub_normal_radc[chip->sbu1_range_index].vmin, sub_normal_radc[chip->sbu1_range_index].vmax,
				sub_normal_radc[chip->sbu2_range_index].vmin, sub_normal_radc[chip->sbu2_range_index].vmax);
	} else if (chip->cable_id_detect_done == 2) {
		size = scnprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d", cabel_A, chip->sbu1_adc, chip->sbu2_adc,
				sub_error_radc[chip->sbu1_range_index].vmin, sub_error_radc[chip->sbu1_range_index].vmax,
				sub_error_radc[chip->sbu2_range_index].vmin, sub_error_radc[chip->sbu2_range_index].vmax);
	} else {
		size = scnprintf(buf, PAGE_SIZE, "unknown");
	}
	return size;
}
static CLASS_ATTR_RO(sbu_info);

static struct attribute *stm32_debug_attrs[] = {
	&class_attr_mcu_fw_inside_burn_result.attr,
	&class_attr_dchg_state.attr,
	&class_attr_fw_update.attr,
	&class_attr_dchg_ic_vendor.attr,
	&class_attr_burn_ctl.attr,
	&class_attr_burn_ctl_result.attr,
	&class_attr_power_usbsel.attr,
	&class_attr_chg_switch.attr,
	&class_attr_dchg_exit_event.attr,
	&class_attr_burn_success.attr,
	&class_attr_handshake_success.attr,
	&class_attr_in_version.attr,
	&class_attr_out_version.attr,
	&class_attr_ibus.attr,
	&class_attr_vbus.attr,
	&class_attr_vbat.attr,
	&class_attr_cable_mohm.attr,
	&class_attr_req_ibus.attr,
	&class_attr_ldo_count.attr,
	&class_attr_ptmValue.attr,
	&class_attr_adjust_stop.attr,
	&class_attr_is_ldo.attr,
	&class_attr_bat_temp_mv.attr,
	&class_attr_bat_conn_temp.attr,
	&class_attr_master_bat_conn_temp.attr,
	&class_attr_usb_conn_temp.attr,
	&class_attr_adapter_temp.attr,
	&class_attr_adapter_conn_temp.attr,
	&class_attr_dp.attr,
	&class_attr_dm.attr,
	&class_attr_cable_matched.attr,
	&class_attr_chg_enable.attr,
	&class_attr_analog_i2c_read_count.attr,
	&class_attr_analog_i2c_write_count.attr,
	&class_attr_uart_tx_count.attr,
	&class_attr_uart_rx_count.attr,
	&class_attr_ap_i2c_read_count.attr,
	&class_attr_ap_i2c_write_count.attr,
	&class_attr_adapter_i2c_read_count.attr,
	&class_attr_adapter_i2c_write_count.attr,
	&class_attr_adapter_uart_tx_count.attr,
	&class_attr_adapter_uart_rx_count.attr,
	&class_attr_adapter_status_monitor.attr,
	&class_attr_slave_ibus.attr,
	&class_attr_master_ibus.attr,
	&class_attr_slave_chg_enable.attr,
	&class_attr_master_chg_enable.attr,
	&class_attr_slave_chg_enable_flag.attr,
	&class_attr_bat_temp.attr,
	&class_attr_update_sbu_adc.attr,
	&class_attr_sbu_off_adc.attr,
	&class_attr_sbu_direct_on_adc.attr,
	&class_attr_sbu_flip_on_adc.attr,
	&class_attr_sbu_info.attr,
	NULL,
};
ATTRIBUTE_GROUPS(stm32_debug);

static int stm32_debug_init(struct stm32l011_mcu *chip)
{
	int rc;

	chip->stm32_debug.name = "stm32_debug";
	chip->stm32_debug.class_groups = stm32_debug_groups;
	rc = class_register(&chip->stm32_debug);

	if (rc < 0)
		pr_err("Failed to create stm32_debug, rc=%d\n", rc);
	else {
		pr_info("Successfully create stm32_debug!\n");
		chip->test_data = kzalloc(sizeof(struct test_data), GFP_KERNEL);
		if (!chip->test_data) {
			pr_err("Unable to allocate memory for test_data\n");
		}
	}

	return rc;
}
#endif

#ifdef TEST_BY_FXI_FOR_BURN
static bool fw_check_version(struct stm32l011_mcu *chip)
{
	char reg[2];
	bool status = false;
	u8 version = 0x0;
	reg[0] = 0;
	reg[1] = 0;
	stm32l011_read_reg(chip, 0x0D, &reg[0]);
	stm32l011_read_reg(chip, 0x0E, &reg[1]);
	stm32l011_read_reg(chip, 0x0F, &version);

	if (reg[0] == 0 && reg[1] == 0 && version == 0) {
		pr_info("recheck version after 2s\n");
		msleep(2000);
		stm32l011_read_reg(chip, 0x0D, &reg[0]);
		stm32l011_read_reg(chip, 0x0E, &reg[1]);
		stm32l011_read_reg(chip, 0x0F, &version);
	}

	switch (chip->dchg_version) {
	case BQ25970_DUAL_DCHG:
		/* version scope is 0x81-0xFF */
		if ((version <= 0 || version != chip->fw.version) && chip->fw.data)
			status = true;
		break;
	case BQ25970_SINGLE_DCHG:
	case LN8000_DCHG:
	default:
		/* version scope is 0x01-0x80 */
		if ((version <= 0 || version != chip->fw.version) && chip->fw.data)
			status = true;
		break;
	}
	fuelsummary_collect_value(ID_BYTE__FC_BL_REV, chip->mcu_bl_vendor);
	fuelsummary_collect_value(ID_BYTE__FC_APP_REV, chip->fw.version);

	chip->mcu_current_fw = version;

	/*for debug test ,always burn new fw*/
	//status = true;

	pr_info("PD%d%d_QC_%x.%x, dchg_version[0x%x], fw_version[0x%x], status[%d], fw_data[%d]\n",
			reg[0], reg[1], ((version&0xF0)>>4), (version&0xF),
			chip->dchg_version, chip->fw.version, status, !!chip->fw.data);
	return status;
}
static void stm32l011_inside_burn_work(struct work_struct *work)
{
	union power_supply_propval enable = {1,};
	int ret = 0;
	u16 erase_size;
	u8 *p;
	u8 update_type = I2C_BURN;
	static bool first = true;
	bool	update = false;
	struct stm32l011_mcu *chip = container_of(work,
			struct stm32l011_mcu, inside_burn_work.work);

	pr_info("~~~\n");
	stm32l011_stay_awake(chip, PM_BURN);
	mutex_lock(&chip->burn_lock);
	chip->burn_handling = true;
	if (!chip->usb_present || !first) {
		stm32l011_power_restore_enable(chip);
		msleep(1500);
		stm32l011_shift_en_enable(chip,true);
		update = fw_check_version(chip);
		if (!update) {
			if (!chip->usb_present) {
				stm32l011_shift_en_enable(chip,false);
				stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
			}
		}
	} else {
		/*mcu inited check*/
		msleep(1500);
		update = fw_check_version(chip);
	}

	if (!update) {
		pr_info("no update\n");
		chip->update_done = true;
		//if (chip->inside_burn_result == MCU_FW_INSIDE_BURN_RESULT__NONE)
		chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__SUCCESS;
		chip->burn_handling = false;
		mutex_unlock(&chip->burn_lock);
		stm32l011_relax(chip, PM_BURN);
		return;
	}
	if (chip->update_done && chip->inside_burn_result == MCU_FW_INSIDE_BURN_RESULT__SUCCESS) {
		pr_info("update completed\n");
		chip->burn_handling = false;
		mutex_unlock(&chip->burn_lock);
		stm32l011_relax(chip, PM_BURN);
		return;
	}

	if (chip->usb_present) {
		cancel_delayed_work_sync(&chip->chg_monitor_work);
		if (chip->pmi_suspend) {
			pr_err("PMI suspend durning MCU FW update, restart PMI charging...\n");
			power_supply_lite_set_property(PSYL_CHARGE,
					POWER_SUPPLY_PROP_DCHG_CHARGING_ENABLED, &enable);
		}
		stm32l011_hw_deinit(chip);
	}

	stm32l011_power_restore_enable(chip);
	msleep(100);
	stm32l011_shift_en_enable(chip,true);

	ret = stm32l011_i2c_burn_command(chip, BURN_CMD_UPDATE, &update_type, CMD_ONE_PARAM);
	if (ret < 0) {
		pr_err("burn command=0x%02X time out\n",BURN_CMD_UPDATE);
		goto out;
	}

	if (chip->fw.size > FW_SIZE_MAX_KB * 1024)
		pr_err(" [Error]: MCU bin > %dK !!!!!\n", FW_SIZE_MAX_KB);
	else
		pr_info("MCU bin = %dK\n", (chip->fw.size / 1024));

	erase_size = (chip->fw.size / FLASH_PAGE_SIZE + ((chip->fw.size % FLASH_PAGE_SIZE) > 0 ? 1 : 0));
	p = get_erase_cmd_param(chip, FLASH_APP_ADDR_SHIFT, erase_size);
	ret = stm32l011_i2c_burn_command(chip, BURN_CMD_ERASE, p, CMD_ERASE_PARAM);
	if (ret < 0) {
		pr_err("burn command=0x%02X time out\n",BURN_CMD_ERASE);
		goto out;
	}

	stm32l011_i2c_burn_write(chip, false, chip->fw.data, FLASH_APP_ADDR_SHIFT, chip->fw.size, INSIDE_BATCH);

	p = get_crc_cmd_param(chip, chip->fw.crc, FLASH_APP_ADDR_SHIFT, chip->fw.size);

	ret = stm32l011_i2c_burn_command(chip, BURN_CMD_CRC, p, CMD_CRC_PARAM);
	if (ret < 0) {
		if (chip->inside_burn_result == MCU_FW_INSIDE_BURN_RESULT__NONE)
			chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__FW_CRC_ERROR;
		pr_err("burn command=0x%02x time out\n", BURN_CMD_CRC);
		goto out;
	}

	//ret = stm32l011_i2c_burn_command_noparam(chip, BURN_CMD_APP_VERSION);
	//pr_info("burn fw crc success app version : %d\n", ret);
	pr_info("burn fw right\n");
	if (!chip->usb_present) {
		stm32l011_shift_en_enable(chip,false);
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
	} else {
		stm32l011_power_restore_enable(chip);
		stm32l011_shift_en_enable(chip,true);

		cancel_delayed_work(&chip->chg_monitor_work);
		schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
	}
	chip->update_done = true;
	chip->burn_handling = false;
	chip->mcu_current_fw = chip->fw.version;
	chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__SUCCESS;
	goto out_unlock;
	//mutex_unlock(&chip->burn_lock);
	//stm32l011_relax(chip, PM_BURN);
	//return;
out:
	if (first) {
		first = false;
		pr_warn("burn fw first error,retry again\n");
		schedule_delayed_work(&chip->inside_burn_work, msecs_to_jiffies(3000));
	} else {
		pr_warn("burn fw error\n");
		if (!chip->usb_present) {
			stm32l011_shift_en_enable(chip,false);
			stm32l011_pinctrl_power_gpio_state(chip, PINCFG_LOW);
		} else {
			stm32l011_power_restore_enable(chip);
			stm32l011_shift_en_enable(chip,true);

			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
		}
		chip->update_done = true;
		chip->burn_handling = false;
		if (chip->inside_burn_result == MCU_FW_INSIDE_BURN_RESULT__NONE)
			chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__UNKNOW_ERROR;
	}

out_unlock:
	pr_info("update_done=%d, burn_handling=%d, inside_burn_result=%d.\n", chip->update_done, chip->burn_handling, chip->inside_burn_result);
	mutex_unlock(&chip->burn_lock);
	stm32l011_relax(chip, PM_BURN);
}
#endif
extern char* get_bbk_board_version(void);
static int stm32l011_dchg_disable_vote_callback(struct votable *votable,
		void *data,
		int dchg_disable, const char *client)
{
	struct stm32l011_mcu *chip = data;

	pr_err("dchg_disable=%d, vote_client=%s\n", dchg_disable, get_effective_client_locked(votable));

	stm32l011_dchg_enable(chip, !dchg_disable);

	return 0;
}
static int set_fastchg_current_vote_callback(struct votable *votable, void *data,
		int fcc_ma, const char *client)
{
	struct stm32l011_mcu *chip = data;

	pr_info("fcc_ma=%d, vote_client=%s\n", fcc_ma, get_effective_client_locked(votable));
	if (fcc_ma < 0) {
		pr_err("fcc_ma=%d\n", fcc_ma);
		return 0;
	}
	stm32l011_set_current(chip, fcc_ma);

	return 0;
}
static int stm32l011_create_votables(struct stm32l011_mcu *chip)
{
	int rc = 0;

	pr_info("votable start\n");

	chip->dchg_disable_votable = create_votable("DCHG_DISABLE",
			VOTE_SET_ANY, stm32l011_dchg_disable_vote_callback, chip);
	if (IS_ERR(chip->dchg_disable_votable)) {
		rc = PTR_ERR(chip->dchg_disable_votable);
		return rc;
	}

	chip->dchg_fcc_votable = create_votable("DCHG_FCC",
			VOTE_MIN, set_fastchg_current_vote_callback, chip);
	if (IS_ERR(chip->dchg_fcc_votable))
		return PTR_ERR(chip->dchg_fcc_votable);

	/*init vote value*/
	vote(chip->dchg_fcc_votable, TOTAL_DCHG_FCC_LIMIT_MAX, true, chip->total_dchg_limit_max_ma);
	return 0;
}

static int stm32l011_adc_read(struct stm32l011_mcu *chip, struct iio_channel *chan, int *data)
{
	int rc = 0;
	int value = 0;

	*data = -ENODATA;

	if (chan) {
		rc = iio_read_channel_processed(chan, &value);
		if (rc < 0) {
			pr_err("Error in reading IIO channel data, rc=%d\n", rc);
			return rc;
		}

		/*val * 1500 / 4096*/
		value = (value * 1500) >> 12;
		*data = value * 1000;
	} else {
		rc = -1;
		pr_err("chan null, rc=%d\n", rc);
	}

	return rc;
}

/*Vadc default pull 100K + 1875mV */
static int stm32l011_sbu_cable_id_detect(struct stm32l011_mcu *chip)
{
	int rc = 0, i = 0;
	static int detect_count = 0;
	int adc_result = 0, sbu1_id = -1, sbu2_id = -1;
	int sub_normal_radc_size = ARRAY_SIZE(sub_normal_radc_rely_switch);
	int sub_error_radc_size = ARRAY_SIZE(sub_error_radc_rely_switch);
	const struct sbu_r_adc * sub_normal_radc = sub_normal_radc_rely_switch;
	const struct sbu_r_adc * sub_error_radc = sub_error_radc_rely_switch;


	if (!chip->sbu_cable_id_detect_enable)
		goto out;

	if (SBU_CABLE_ID_UNKNOW != chip->sbu_cable_id)
		return 0;

	chip->sbu_detect_busy = true;
	if (!chip->sbu_rely_switch) {
		sub_normal_radc_size = ARRAY_SIZE(sub_normal_radc_no_switch);
		sub_error_radc_size = ARRAY_SIZE(sub_error_radc_no_switch);
		sub_normal_radc = sub_normal_radc_no_switch;
		sub_error_radc = sub_error_radc_no_switch;
	}

	rc = max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_DIRECT_CONNECT);
	if (gpio_get_value(chip->sbu_pwr_gpio) == 0) {
		pr_info("sbu_pwr_gpio ctrl output high!\n");
		gpio_direction_output(chip->sbu_pwr_gpio, 1);
		msleep(300);
	}

	rc = stm32l011_adc_read(chip, chip->iio.sbu1_adc_chan, &adc_result);
	if (rc < 0) {
		pr_err("sbu1_cable_id_adc read fail, rc=%d\n", rc);
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		chip->sbu_detect_busy = false;
		goto out;
	}
	chip->sbu1_adc = adc_result / 1000;
	msleep(10);

	rc = stm32l011_adc_read(chip, chip->iio.sbu2_adc_chan, &adc_result);
	if (rc < 0) {
		pr_err("sbu2_cable_id_adc read fail, rc=%d\n", rc);
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		chip->sbu_detect_busy = false;
		goto out;
	}
	chip->sbu2_adc = adc_result / 1000;

	for (i = 0; i < sub_normal_radc_size; i++) {
		if (is_between(sub_normal_radc[i].vmin, sub_normal_radc[i].vmax, chip->sbu1_adc)) {
			sbu1_id = sub_normal_radc[i].id;
			chip->sbu1_range_index = i;
		}
		if (is_between(sub_normal_radc[i].vmin, sub_normal_radc[i].vmax, chip->sbu2_adc)) {
			sbu2_id = sub_normal_radc[i].id;
			chip->sbu2_range_index = i;
		}
	}

	if (sbu1_id != -1 && sbu2_id != -1 && sbu1_id == sbu2_id) {
		gpio_direction_output(chip->sbu_pwr_gpio, 0);
		chip->sbu_cable_id = sbu1_id;
		detect_count = 0;
		chip->cable_id_detect_done = 1;
		rc = max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_HIZ);
	} else {
		if (detect_count++ >= 5) {
			if ((sbu1_id == -1 && sbu2_id == -1) || (sbu1_id == -1 && sbu2_id == SBU_CABLE_ID_DEFAULT) || (sbu2_id == -1 && sbu1_id == SBU_CABLE_ID_DEFAULT)) {
				chip->sbu_cable_id = SBU_CABLE_ID_DEFAULT;
			} else {
				for (i = 0; i < sub_error_radc_size; i++) {
					if (is_between(sub_error_radc[i].vmin, sub_error_radc[i].vmax, chip->sbu1_adc)) {
						sbu1_id = sub_error_radc[i].id;
						chip->sbu1_range_index = i;
					}

					if (is_between(sub_error_radc[i].vmin, sub_error_radc[i].vmax, chip->sbu2_adc)) {
						sbu2_id = sub_error_radc[i].id;
						chip->sbu2_range_index = i;
					}
				}

				if (sbu1_id != -1 && sbu2_id != -1)
					chip->sbu_cable_id = min(sbu1_id, sbu2_id);
			}

			detect_count = 0;
			if (chip->sbu_cable_id <= SBU_CABLE_ID_UNKNOW || chip->sbu_cable_id >= SBU_CABLE_ID_ERROR)
				chip->sbu_cable_id = SBU_CABLE_ID_DEFAULT;
			gpio_direction_output(chip->sbu_pwr_gpio, 0);
			chip->cable_id_detect_done = 2;
			rc = max20328_switch_mode_event(MAX_USBC_SWITCH_SBU_HIZ);
		} else {
			chip->sbu_cable_id = SBU_CABLE_ID_UNKNOW;//detect next time.
		}
	}

	chip->sbu_detect_busy = false;
	pr_info("sbu1_adc=%d, sbu2_adc=%d. cable_id=%d(%d,%d), cable_id_str=%s, detect_count=%d.\n",
			chip->sbu1_adc, chip->sbu2_adc, chip->sbu_cable_id, sbu1_id, sbu2_id, sbu_cable_id_type_strings[chip->sbu_cable_id], detect_count);
	return 0;
out:
	chip->sbu_cable_id = SBU_CABLE_ID_DEFAULT;
	return rc;
}

static int stm32l011_get_adc(struct stm32l011_mcu *chip, const char *propname,
		struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chip->dev->of_node,
			"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chip->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		//if (rc != -EPROBE_DEFER)
		pr_err("%s channel unavailable, %d\n", propname, rc);
		*chan = NULL;
	}

	return rc;
}

static irqreturn_t battery_cout_interrupt(int irq, void *dev_id)
{
	struct stm32l011_mcu *chip = dev_id;
	int last_cout_value = chip->battery_cout_value;

	pr_info(" ~ \n");
	mutex_lock(&chip->cout_irq_complete);
	chip->battery_cout_value = gpio_get_value(chip->cout_gpio);
	pr_err("cout_gpio=%d,bat_cout_counter=%d\n", chip->battery_cout_value, chip->battery_cout_counter);
	if (chip->battery_cout_counter <= 10000 && chip->battery_cout_value)
		chip->battery_cout_counter++;

	fuelsummary_collect_value(ID_INT__FG_COUT_CNT, chip->battery_cout_counter);
	if (chip->battery_cout_value && !last_cout_value)
		stm32l011_masked_write(chip, MCU_REG_08, MCU_SET_CV_MODE, MCU_SET_CV_MODE);
	mutex_unlock(&chip->cout_irq_complete);

	return IRQ_HANDLED;
}

static int stm32l011_init_adc(struct stm32l011_mcu *chip)
{
	int rc = 0;

	if (chip->sbu_cable_id_detect_enable) {
		rc = stm32l011_get_adc(chip, "sbu1_cable_id_adc", &chip->iio.sbu1_adc_chan);
		if (rc < 0)
			return rc;

		rc = stm32l011_get_adc(chip, "sbu2_cable_id_adc", &chip->iio.sbu2_adc_chan);
		if (rc < 0)
			return rc;
	}

	rc = stm32l011_get_adc(chip, "bat_board_temp", &chip->iio.bat_board_temp_chan);
	if (rc < 0)
		return rc;

	rc = stm32l011_get_adc(chip, "usb_conn_temp", &chip->iio.usb_conn_temp_chan);
	if (rc < 0)
		return rc;

	rc = stm32l011_get_adc(chip, "bat_conn_temp", &chip->iio.bat_conn_temp_chan);
	if (rc < 0)
		return rc;

	rc = stm32l011_get_adc(chip, "master_bat_conn_temp", &chip->iio.master_bat_conn_temp_chan);
	if (rc < 0)
		return rc;

	return 0;
}

static int stm32l011_pinctrl_init(struct stm32l011_mcu *chip)
{
	int rc = 0;

	/*get pincrtl*/
	chip->pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		pr_err("devm_pinctrl_get error.\n");
		chip->pinctrl = NULL;
		return 0;
	}

	/*get power gpio config*/
	if (chip->power_gpio > 0) {
		chip->power_gpio_default = pinctrl_lookup_state(chip->pinctrl, "power_gpio_default");
		if (IS_ERR_OR_NULL(chip->power_gpio_default)) {
			pr_err("power_gpio_default error.\n");
			chip->power_gpio_default = NULL;
		}

		chip->power_gpio_high = pinctrl_lookup_state(chip->pinctrl, "power_gpio_high");
		if (IS_ERR_OR_NULL(chip->power_gpio_high)) {
			pr_err("power_gpio_high error.\n");
			chip->power_gpio_high = NULL;
		}

		chip->power_gpio_low = pinctrl_lookup_state(chip->pinctrl, "power_gpio_low");
		if (IS_ERR_OR_NULL(chip->power_gpio_low)) {
			pr_err("power_gpio_low error.\n");
			chip->power_gpio_low = NULL;
		}

		/*init power gpio state*/
		stm32l011_pinctrl_power_gpio_state(chip, PINCFG_DEFAULT);
	}

	pr_info("power_gpio pinctrl init completely!\n");

	return rc;
}



/************************************************************
 *
 *   [mcu power_supply_lite function]
 *
 ***********************************************************/
static bool stm32l011_hw_component_detect(struct stm32l011_mcu *chip)
{
	int ret = 0;
	uint8_t update_type = I2C_BURN;

	stm32l011_power_restore_enable(chip);
	msleep(100);
	stm32l011_shift_en_enable(chip,true);

	ret = stm32l011_i2c_burn_command(chip, BURN_CMD_BOOT_VERSION, &update_type, CMD_ONE_PARAM);
	if (ret < 0)
		chip->exist = false;
	else
		chip->exist = true;

	pr_info("ret=%d,ic_exist=%d\n", ret, chip->exist);
	return chip->exist;
}

static int stm32l011_io_cable(struct stm32l011_mcu *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0, i = 0;
	u8 mcu_reg08 = 0x0, chgic_reg14 = 0x0;
	struct timespec time_now;

	if (io == SET_IN) {
		/* add for ffc */
		if (chip->ex_fg_ffc_support)
			stm32l011_check_cout_status(chip);

		mutex_lock(&chip->status_lock);
		if (!chip->usb_present && val->intval) {
			pr_info("plugged = %d\n", val->intval);
			chip->usb_present = true;
			chip->battery_cout_counter = 0;
			get_monotonic_boottime(&time_now);
			chip->cable_connect_start = time_now.tv_sec;
			if (chip->chgic_rt9759 && get_atboot()) {
				if (stm32l011_get_power_gpio_state(chip))
					stm32l011_hw_deinit(chip);
			}
			stm32l011_hw_init(chip);
			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
		} else if (chip->usb_present && !val->intval) {
			pr_info("unplugged = %d\n", val->intval);
			chip->usb_present = false;
			chip->battery_cout_counter = 0;
			//cancel_delayed_work(&chip->chg_monitor_work);
			cancel_delayed_work_sync(&chip->chg_monitor_work);
			ret = stm32l011_masked_write(chip, MCU_REG_08, MCU_SET_SHUTDOWN_POWER, MCU_SET_SHUTDOWN_POWER);
			if (ret)
				pr_err("Couldn't write MCU_REG_08 ret = %d\n", ret);
			if (get_atboot()) {
				for (i = 0; i < 5; i++) {
					msleep(600);
					ret = stm32l011_read_reg(chip, MCU_REG_08, &mcu_reg08);
					ret = stm32l011_read_reg(chip, (MCU_BQ25970_REGS2_START + 0x14), &chgic_reg14);
					pr_info("read[%d] MCU_REG08=0x%02x,CHGIC_REG14=0x%02x\n", i, mcu_reg08, chgic_reg14);
					if (!(chgic_reg14 & BIT(7))) {
						break;
					}
				}
				if (!chip->usb_present) {
					stm32l011_hw_deinit(chip);
				}
			} else {
				stm32l011_hw_deinit(chip);
			}
#if TEST_CYCLE_CHARGE_DISCHARGE
			chip->chg_switch = true;
#endif
		} else if (chip->check_pwr_reset_after_chg_remove && val->intval) {
			pr_info("mcu power reset here(%d)...\n", val->intval);
			/*power down*/
			cancel_delayed_work_sync(&chip->chg_monitor_work);
			stm32l011_hw_deinit(chip);
#if TEST_CYCLE_CHARGE_DISCHARGE
			chip->chg_switch = true;
#endif
			msleep(100);
			/*power up*/
			get_monotonic_boottime(&time_now);
			chip->cable_connect_start = time_now.tv_sec;
			stm32l011_hw_init(chip);
			cancel_delayed_work(&chip->chg_monitor_work);
			schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
			chip->usb_present = !!val->intval;
		}
		chip->check_pwr_reset_after_chg_remove = false;
		mutex_unlock(&chip->status_lock);
	} else {
		pr_err("GET_OUT not support\n");
		ret = -EIO;
	}
	return ret;
}

static int stm32l011_io_dump(struct stm32l011_mcu *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;

	if (io == GET_OUT) {
		//get_votable_info(chip->dchg_disable_votable);
		//get_votable_info(chip->dchg_fcc_votable);
	} else {
		pr_err("GET_OUT not support\n");
		ret = -EIO;
	}
	return ret;
}

static int stm32l011_io_ffc_cv(struct stm32l011_mcu *chip, enum direction_io io, union power_supply_propval *val)
{
	int ret = 0;

	if (io == SET_IN) {
		if (val->intval != 0) {
			pr_info("btbcv=%d, btb_cv_mv=%d,fg_cv_mv=%d\n", val->intval,
					chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv,
					chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].fg_cv_mv);
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv = val->intval;
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].fg_cv_mv = val->intval;
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].btb_cv_mv = val->intval;
			chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_DUAL].fg_cv_mv = val->intval;
		}
	} else {
		val->intval = chip->ffc_dchg_cv_mv[FFC_DCHG_STEP_CV_SINGLE].btb_cv_mv;
	}
	return ret;
}

static int stm32l011_ioctrl_property(struct power_supply_lite *psyl,
		enum direction_io io,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0, usr_current = 0;
	struct stm32l011_mcu *chip = power_supply_lite_get_drvdata(psyl);

	if (!psyl) {
		pr_info("ioctrl[%] not find psyl!!!\n", psp);
		return -EFAULT;
	}
#if PSYL_IO_DEBUG
	pr_debug("ioctrl[%d] io[%d]\n", psp, io);
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_CABLE_PLUG:
		ret = stm32l011_io_cable(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_DUMP:
		if (chip->usb_present)
			ret = stm32l011_io_dump(chip, io, val);
		break;
	case POWER_SUPPLY_PROP_CHECK:
	case POWER_SUPPLY_PROP_RUNNING:
	case POWER_SUPPLY_PROP_COLLECT:
	case POWER_SUPPLY_PROP_UPDATE:
		break;
	case POWER_SUPPLY_PROP_USBSEL:
		if (io == SET_IN) {
			pr_info("usbsel=%d\n", val->intval);
			if (!chip->vbus_exception)
				stm32l011_usbsel_enable(chip, val->intval);
		} else {
			val->intval = stm32l011_get_usbsel_state(chip);
		}
		break;
	case POWER_SUPPLY_PROP_MCU_POWER:
		if (io == SET_IN) {
			pr_info("power=%d\n", val->intval);
			stm32l011_power_enable(chip, val->intval);
		}
		break;
	case POWER_SUPPLY_PROP_DCHG_CURRENT:
		if (io == SET_IN) {
			usr_current = min(chip->total_dchg_limit_max_ma, val->intval);
			chip->user_dchg_current = usr_current;
			pr_info("current=%d, limit_max_ma=%d,usr_current=%d\n",
					val->intval, chip->total_dchg_limit_max_ma, usr_current);
			if (chip->chg_enable) {
				if (!get_atboot()) {
					ret = vote(chip->dchg_fcc_votable, USER_FCC_VOTER, true, usr_current);
					if (ret < 0)
						pr_err("Couldn't vote fastchg ma ret=%d\n", ret);
				}
			} else {
				pr_err("direct charging no enable, skip setting current (%d mA)...\n", chip->user_dchg_current);
			}
		}
		break;
	case POWER_SUPPLY_PROP_CHARGER_REMOVE:
		if (io == SET_IN) {
			pr_info("charger remove now (%d).\n", chip->usb_present);
			chip->check_pwr_reset_after_chg_remove = chip->usb_present;
			chip->last_cable_disconnect_start_time = jiffies;
		}
		break;
	case POWER_SUPPLY_PROP_DCHG_ENABLE:
		if (io == SET_IN) {
			pr_info("dchg enable=%d\n", val->intval);
			stm32l011_dchg_enable(chip, val->intval);
		} else {
			val->intval = chip->chg_enable;
		}
		break;
	case POWER_SUPPLY_PROP_DCHG_CALL_START:
		if (io == SET_IN) {
			if (chip->enable_slave_charger)
				dchg_temp_retry_flag_deinit(chip);
		}
		break;
	case POWER_SUPPLY_PROP_FACTORY_MODE_STATE:
		if (io == SET_IN) {
			chip->factory_mode_state = val->intval;
			pr_info("### vivo factory_mode_state=%d ###\n", chip->factory_mode_state);
		}
		break;
	case POWER_SUPPLY_PROP_FACTORY_10W_CHARGE_TEST:
		if (io == SET_IN) {
			chip->factory_10W_charge_test_enabe = val->intval;
			stm32l011_dchg_enable(chip, 0);
			pr_debug("### factory_10W_charge_test_enabe=%d ###\n", chip->factory_10W_charge_test_enabe);
		}
		break;
	case POWER_SUPPLY_PROP_HANDSHAKE_STATUS:
		if (io == GET_OUT) {
			if (chip->cable_matched_event) {
				if (chip->dchg_supported_type >= DCHG_SINGLE_TYPE_44W) {
					if ((chip->adapter_power & 0x0F) == 0x01 ||
							(chip->adapter_power & 0x0F) == 0x04 ||
							(chip->adapter_power & 0x0F) == 0x05) {
						if (chip->sbu_cable_id == SBU_CABLE_ID_4A ||
								chip->sbu_cable_id == SBU_CABLE_ID_5A ||
								chip->sbu_cable_id == SBU_CABLE_ID_6A ||
								chip->sbu_cable_id == SBU_CABLE_ID_8A ||
								(chip->cable_mohm > 0 && chip->cable_mohm <= 278) ||
								((chip->irqB & EXIT_HIGH_VBAT) && chip->cable_mohm == 0)) {
							chip->vivo_flash_charge_status = POWER_SUPPLY_ENGINE_SUPER;
						}
					}
				}
			}
			if (chip->vivo_flash_charge_status > POWER_SUPPLY_ENGINE_NORMAL &&
					(chip->main_chg_sm_error_in_ap_side & EXIT_ADAPTER_IDENTIFICATION_ERROR || chip->reg05 & EXIT_ADAPTER_IDENTIFICATION_ERROR)) {
				pr_err("adapter_identification_error:: vivo_flash_charge_status=%d, reg05=0x%02x,"
						"main_chg_sm_error_in_ap_side=0x%02x.. Force POWER_SUPPLY_ENGINE_NORMAL\n",
						chip->vivo_flash_charge_status, chip->reg05, chip->main_chg_sm_error_in_ap_side);
				val->intval = POWER_SUPPLY_ENGINE_NORMAL;
			} else
				val->intval = chip->vivo_flash_charge_status;
		}
		break;
	case POWER_SUPPLY_PROP_DCHG_LIMIT_MA:
		if (io == GET_OUT)
			val->intval = chip->total_dchg_limit_max_ma;
		break;
	case POWER_SUPPLY_PROP_USB_CONN_TEMP:
		if (io == GET_OUT)
			val->intval = chip->usb_conn_temp;
		break;
	case POWER_SUPPLY_PROP_BAT_BOARD_TEMP:
		if (io == GET_OUT)
			val->intval = chip->bat_temp;
		break;
	case POWER_SUPPLY_PROP_ADAPTER_TEMP:
		if (io == GET_OUT)
			val->intval = chip->adapter_temp;
		break;
	case POWER_SUPPLY_PROP_ADAPTER_POWER:
		if (io == GET_OUT)
			val->intval = chip->adapter_power;
		break;
	case POWER_SUPPLY_PROP_ADAPTER_POWER_DERATE:
		if (io == GET_OUT)
			val->intval = chip->adapter_powerderate;
		break;
	case POWER_SUPPLY_PROP_ADAPTER_POWER_DERATE_ALREADY:
		if (io == GET_OUT)
			val->intval = chip->adapter_powerderate_ready;
		break;
	case POWER_SUPPLY_PROP_MASTER_BAT_CONN_TEMP:
		if (io == GET_OUT)
			val->intval = chip->master_bat_conn_temp;
		break;
	case POWER_SUPPLY_PROP_BAT_CONN_TEMP:
		if (io == GET_OUT)
			val->intval = chip->bat_conn_temp;
		break;
	case POWER_SUPPLY_PROP_DCHG_CABLER:
		if (io == GET_OUT)
			val->intval = chip->cable_mohm;
		break;
	case POWER_SUPPLY_PROP_DCHG_COUT:
		if (io == GET_OUT)
			val->intval = chip->battery_cout_value;
		break;
	case POWER_SUPPLY_PROP_DCHG_COUT_COUNT:
		if (io == GET_OUT)
			val->intval = chip->battery_cout_counter;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		if (io == GET_OUT) {
			if (chip->pmi_suspend) {
				chip->vbus_mv = stm32l011_get_vbusADC(chip);
				pr_debug("chip->vbus_mv=%d\n", chip->vbus_mv);
				val->intval = chip->vbus_mv;
			} else {
				power_supply_lite_get_property(PSYL_CHARGE, psp, val);
			}
		}
		break;
	case POWER_SUPPLY_PROP_HALF_CHG_MASTER_IBUS:
		if (io == GET_OUT)
			val->intval = chip->ibus_ma;
		break;
	case POWER_SUPPLY_PROP_HALF_CHG_SLAVE_IBUS:
		if (io == GET_OUT)
			val->intval = chip->slave_ibus_ma;
		break;
	case POWER_SUPPLY_PROP_LIMIT_INPUT_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_VINDPM:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		break;
		case POWER_SUPPLY_PROP_FFC_CV:
			ret = stm32l011_io_ffc_cv(chip, io, val);

		break;
	default:
		ret = -EINVAL;
	}
#if PSYL_IO_DEBUG
	pr_info("ioctrl[%d] io[%d] %s\n", psp, io, (ret < 0 ? "error" : "done"));
#else
	if (ret < 0)
		pr_info("ioctrl[%d] io[%d] err\n", psp, io);
#endif
	return ret;
}

static struct power_supply_lite stm32l011_psyl = {
	.name = "mcu",
	.status = STATUS_MCU,
	.dev_num = PSYL_MCU,
	.initialized = false,
	.ioctrl_property = stm32l011_ioctrl_property,
};

static int stm32l011_mcu_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct stm32l011_mcu *chip;
	int rc;

	pr_info("stm32l011_mcu_probe start\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("stm32l011 mcu,No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->name = "STM32";
	chip->client = client;
	chip->dev = &client->dev;
	chip->exist = false;
	chip->update_done = false;
	chip->debug_fw_updating = false;
	chip->resume_completed = true;
	chip->report_big_data = false;
	chip->dchg_enabled = 0xFF;
	chip->time_count = 0;
	chip->mcu_current_fw = 0;
	chip->inside_burn_result = MCU_FW_INSIDE_BURN_RESULT__NONE;
	chip->setting_curr_too_low_for_dchg = false;
	chip->sbu1_adc = -1;
	chip->sbu2_adc = -1;
	chip->sbu_cable_id = SBU_CABLE_ID_UNKNOW;
	chip->cable_r_disable_dchg = 0;
	chip->dp_dm_ovp = false;
	chip->suspend = false;
	chip->user_vote_lose = false;
	chip->disable_vote_miss = false;
	chip->chgic_rt9759 = false;
#if OLD_CAM_NOTIFY
	chip->cam_running = false;
	chip->cam_exit_restore = false;
	chip->cam_exit_second = 10;
#endif
	chip->cable_id_detect_done = 0;
	chip->sbu_detect_busy = false;
	this_chip = chip;

#if ADAPTER_SHA_ENCRYPTION
	DChg_Sha1_Init();
#endif

	scnprintf(chip->dchg_state, sizeof(chip->dchg_state), "waiting..");
	scnprintf(chip->mcu_debug_fw_name, sizeof(chip->mcu_debug_fw_name), "unknow..");
	i2c_set_clientdata(client, chip);
	dev_set_drvdata(&(client->dev), chip);

	rc = stm32l011_gpio_parse_dt(chip);
	if (rc) {
		pr_err("Couldn't parse gpio DT nodes rc=%d\n", rc);
		goto init_err;
	}

	if (chip->power_gpio > 0) {
		rc = gpio_request(chip->power_gpio,"power_gpio");
		if (rc) {
			pr_err("failed to request power_gpio,rc=%d\n",rc);
		}
	}

	if (chip->int_gpio > 0) {
		rc = gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,"mcu_int");
		if (rc) {
			pr_err("failed to request int_gpio\n");
		}
		chip->irq = gpio_to_irq(chip->int_gpio);
	}

	if (chip->usbsel_gpio > 0) {
		rc = gpio_request(chip->usbsel_gpio,"usbsel_gpio");
		if (rc) {
			pr_err("failed to request usbsel_gpio,rc=%d\n",rc);
		}
		/*Init usb switcher's sel/csb-pin state.*/
		stm32l011_usbsel_enable(chip, false);
		pr_info("usbsel_gpio=%d\n", gpio_get_value(chip->usbsel_gpio));
	}

	if (chip->shift_en_gpio > 0) {
		rc = gpio_request(chip->shift_en_gpio,"shift_en_gpio");
		if (rc) {
			pr_err("failed to request shift_en_gpio,rc=%d\n",rc);
		}
		pr_info("shift_en_gpio=%d\n", gpio_get_value(chip->shift_en_gpio));
	}

	pr_info("power_gpio=%d,int_gpio=%d,usbsel_gpio=%d,shift_en_gpio=%d\n",
			chip->power_gpio, chip->int_gpio, chip->usbsel_gpio, chip->shift_en_gpio);
	if (!stm32l011_hw_component_detect(chip)) {
		pr_err("mcu not exist\n");
		rc = -ENODEV;
		goto not_exist_err;
	}

	mutex_init(&chip->pm_lock);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->cout_irq_complete);
	mutex_init(&chip->stm32l011_i2c_lock);
	mutex_init(&chip->status_lock);
	mutex_init(&chip->burn_lock);
	mutex_init(&chip->cmd_lock);
	mutex_init(&chip->parameters_lock);
	mutex_init(&chip->dchg_enable_lock);
	mutex_init(&chip->ffc_lock);

	INIT_DELAYED_WORK(&chip->chg_monitor_work, stm32l011_chg_monitor_work);
	INIT_DELAYED_WORK(&chip->chg_deinit_work, stm32l011_chg_deinit_work);
	INIT_DELAYED_WORK(&chip->restore_pmi_work, stm32l011_restore_pmi_work);
#ifdef TEST_BY_FXI_FOR_BURN
	INIT_DELAYED_WORK(&chip->inside_burn_work, stm32l011_inside_burn_work);
#endif
	INIT_DELAYED_WORK(&chip->mcu_debug_fw_update, stm32l011_debug_fw_update_work);
	INIT_DELAYED_WORK(&chip->burn_ctl_work, stm32l011_burn_ctl_work);
#if OLD_CAM_NOTIFY
	INIT_DELAYED_WORK(&chip->cam_exit_work, stm32l011_cam_exit_work);
#endif
#if TEST_CYCLE_CHARGE_DISCHARGE
	chip->chg_switch = true;
	chip->burn_success = false;
#endif
	//wake_lock_init(&chip->stm32l011_wake_lock, WAKE_LOCK_SUSPEND, "stm32l011_wk");
	wakeup_source_init(&(chip->stm32l011_wake_lock), "stm32l011_wk");

	rc = stm32l011_parse_dt(chip);
	if (rc) {
		pr_err("Couldn't parse DT nodes rc=%d\n", rc);
		goto init_err;
	}

	rc = stm32l011_init_adc(chip);
	if (rc < 0) {
		pr_err("adc missing rc=%d\n", rc);
		//goto parse_err;
	}

	if (chip->enable_slave_charger) {
		chip->low_current_exit_threshold = 1000;
		fuelsummary_collect_value(ID_BYTE__SIC_VENDOR, CHGIC_BQ25970_DUAL);
	} else {
		chip->low_current_exit_threshold = 500;
		fuelsummary_collect_value(ID_BYTE__SIC_VENDOR, CHGIC_BQ25970);
	}

	chip->cable_r_current_limit_max = chip->total_dchg_limit_max_ma;
	chip->user_dchg_current = chip->total_dchg_limit_max_ma;

	rc = stm32l011_pinctrl_init(chip);
	if (rc) {
		pr_err("Couldn't init pinctrl rc=%d\n", rc);
		//return rc;
	}

	rc = stm32l011_create_votables(chip);
	if(rc < 0){
		pr_err("Couldn't create votable rc=%d\n", rc);
		goto parse_err;
	}

	dump_regs(chip);

	rc = stm32l011_determine_initial_state(chip);
	if (rc) {
		pr_err("Couldn't determine initial state rc=%d\n", rc);
		//goto fail_stm32l011_hw_init;
	}

	/* STAT irq configuration */
	if (chip->irq) {
		rc = devm_request_threaded_irq(&client->dev, chip->irq, NULL,
				stm32l011_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"stm32l011_chg_stat_irq", chip);
		if (rc) {
			pr_err("Failed STAT irq=%d request rc = %d\n", chip->irq, rc);
			goto vote_err;
		}
		enable_irq_wake(client->irq);
	}

	if (chip->cout_gpio > 0) {
		rc = gpio_request_one(chip->cout_gpio, GPIOF_DIR_IN, "battery_cout_irq");
		if (rc) {
			pr_err("failed to request battery_cout_irq\n");
		} else
			chip->cout_irq = gpio_to_irq(chip->cout_gpio);
	}

	if (chip->cout_irq) {
		rc = devm_request_threaded_irq(&client->dev, chip->cout_irq,
				NULL,
				battery_cout_interrupt,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"battery cout IRQ", chip);
		if (rc < 0) {
			pr_err("request battery cout irq for irq=%d failed, ret =%d\n", chip->cout_irq, rc);
		} else {
			chip->battery_cout_value = gpio_get_value(chip->cout_gpio);
			pr_debug("cout_gpio=%d\n", chip->battery_cout_value);
			enable_irq_wake(chip->cout_irq);
		}
	}
#ifdef CONFIG_DEBUG_FS
	create_debugfs_entries(chip);
#endif
#if 1//stm32 debug class
	stm32_debug_init(chip);
#endif
	dump_regs(chip);

	if (chip->enable_slave_charger) {
		chip->dchg_version = BQ25970_DUAL_DCHG;
		chip->mcu_bl_vendor = lierda_Bootloader;
	} else if (chip->ln8000_supported) {
		chip->dchg_version = LN8000_DCHG;
		chip->mcu_bl_vendor = lierda_Bootloader;
	} else {
		chip->dchg_version = BQ25970_SINGLE_DCHG;
		chip->mcu_bl_vendor = lierda_Bootloader;
	}

#ifdef TEST_BY_FXI_FOR_BURN
	if (!(/*get_atboot() ||*/ get_charging_mode()))
		schedule_delayed_work(&chip->inside_burn_work, msecs_to_jiffies(5000));
#endif
	schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(SECOND_CHECK_DELAY));

	chip->mcu_psyl = &stm32l011_psyl;
	chip->mcu_psyl->drv_data = chip;
	rc = charge_psyl_register(chip->mcu_psyl);
	rc = power_supply_lite_register(chip->mcu_psyl, chip->mcu_psyl->dev_num);
	if (rc < 0)
		pr_err("mcu_psyl register psyl error: %d\n", rc);

	pr_info("stm32l011 high-voltage Dchg successfully probed dchg_version=%d.\n", chip->dchg_version);
	return 0;
vote_err:
	if (chip->dchg_disable_votable)
		destroy_votable(chip->dchg_disable_votable);
	if (chip->dchg_fcc_votable)
		destroy_votable(chip->dchg_fcc_votable);
parse_err:
	stm_release_property_data(chip);
init_err:
	mutex_destroy(&chip->pm_lock);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->cout_irq_complete);
	mutex_destroy(&chip->stm32l011_i2c_lock);
	mutex_destroy(&chip->status_lock);
	mutex_destroy(&chip->burn_lock);
	mutex_destroy(&chip->cmd_lock);
	mutex_destroy(&chip->parameters_lock);
	mutex_destroy(&chip->dchg_enable_lock);
	mutex_destroy(&chip->ffc_lock);
not_exist_err:
	if (chip->shift_en_gpio > 0)
		gpio_free(chip->shift_en_gpio);
	if (chip->usbsel_gpio > 0)
		gpio_free(chip->usbsel_gpio);
	if (chip->int_gpio > 0)
		gpio_free(chip->int_gpio);
	if (chip->power_gpio > 0)
		gpio_free(chip->power_gpio);
#if 1//stm32 debug class
	if (chip->test_data)
		kfree(chip->test_data);
#endif
	kfree(chip);
	return rc;
}
static int stm32l011_mcu_remove(struct i2c_client *client)
{
	struct stm32l011_mcu *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->chg_monitor_work);
	cancel_delayed_work(&chip->chg_deinit_work);
	cancel_delayed_work(&chip->restore_pmi_work);

	if (chip->dchg_disable_votable)
		destroy_votable(chip->dchg_disable_votable);
	if (chip->dchg_fcc_votable)
		destroy_votable(chip->dchg_fcc_votable);

	if (chip->shift_en_gpio > 0)
		gpio_free(chip->shift_en_gpio);
	if (chip->usbsel_gpio > 0)
		gpio_free(chip->usbsel_gpio);
	if (chip->int_gpio > 0)
		gpio_free(chip->int_gpio);
	if (chip->power_gpio > 0)
		gpio_free(chip->power_gpio);

	stm_release_property_data(chip);
	mutex_destroy(&chip->pm_lock);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->cout_irq_complete);
	mutex_destroy(&chip->stm32l011_i2c_lock);
	mutex_destroy(&chip->status_lock);
	mutex_destroy(&chip->burn_lock);
	mutex_destroy(&chip->cmd_lock);
	mutex_destroy(&chip->parameters_lock);
	mutex_destroy(&chip->dchg_enable_lock);
	mutex_destroy(&chip->ffc_lock);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->debug_root);
#endif
	return 0;
}

static int stm32l011_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stm32l011_mcu *chip = i2c_get_clientdata(client);
	/*cancel work and dchg exit*/
	chip->suspend = true;
	cancel_delayed_work_sync(&chip->chg_monitor_work);
	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int stm32l011_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stm32l011_mcu *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel mcu */

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int stm32l011_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct stm32l011_mcu *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	pr_debug("irq_waiting=%d\n", chip->irq_waiting);
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		stm32l011_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	chip->suspend = false;
	if (chip->usb_present) {
		cancel_delayed_work(&chip->chg_monitor_work);
		schedule_delayed_work(&chip->chg_monitor_work, msecs_to_jiffies(MONITOR_WORK_DELAY_MS));
	}
	return 0;
}

static const struct dev_pm_ops stm32l011_pm_ops = {
	.suspend	= stm32l011_suspend,
	.suspend_noirq	= stm32l011_suspend_noirq,
	.resume		= stm32l011_resume,
};

static struct of_device_id stm32l011_match_table[] = {
	{ .compatible = "st,st32l011-mcu",},
	{ },
};

static const struct i2c_device_id stm32l011_mcu_id[] = {
	{"st32l011-mcu", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, stm32l011_mcu_id);

static struct i2c_driver stm32l011_mcu_driver = {
	.driver		= {
		.name		= "st32l011-mcu",
		.owner		= THIS_MODULE,
		.of_match_table	= stm32l011_match_table,
		.pm		= &stm32l011_pm_ops,
	},
	.probe		= stm32l011_mcu_probe,
	.remove		= stm32l011_mcu_remove,
	.id_table	= stm32l011_mcu_id,
};

module_i2c_driver(stm32l011_mcu_driver);

MODULE_DESCRIPTION("st32l011 mcu High Voltage DirectCharging");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:chg-st32l011-mcu");
