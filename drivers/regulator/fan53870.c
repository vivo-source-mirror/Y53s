/*
 * FAN53870 ON Semiconductor LDO PMIC Driver.
 *
 * Copyright (c) 2019 On Semiconducto.
 * Zhongming Yang <bright.yang@onsemi.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>



#define fan53870_err(reg, message, ...)                                        \
	pr_err("%s: " message, (reg)->rdesc.name, ##__VA_ARGS__)
#define fan53870_debug(reg, message, ...)                                      \
	pr_debug("%s: " message, (reg)->rdesc.name, ##__VA_ARGS__)

#define FAN53870_REG_PID 0x00
#define FAN53870_REG_RID 0x01
#define FAN53870_REG_IOUT 0x02
#define FAN53870_REG_ENABLE 0x03
#define FAN53870_REG_LDO0 0x04

#define FAN53870_REG_INTERRUPT1 0x14

#define LDO_VSET_REG(offset) ((offset) + FAN53870_REG_LDO0)

#define VSET_BASE_12 800
#define VSET_BASE_34567 1500
#define VSET_STEP_MV 8

#define MAX_REG_NAME 20
#define FAN53870_MAX_LDO 7

//Interrupt 1
#define LDO1_UVP_INT BIT(0)
#define LDO2_UVP_INT BIT(1)
#define LDO3_UVP_INT BIT(2)
#define LDO4_UVP_INT BIT(3)
#define LDO5_UVP_INT BIT(4)
#define LDO6_UVP_INT BIT(5)
#define LDO7_UVP_INT BIT(6)

//Interrupt 2
#define LDO1_OCP_INT BIT(0)
#define LDO2_OCP_INT BIT(1)
#define LDO3_OCP_INT BIT(2)
#define LDO4_OCP_INT BIT(3)
#define LDO5_OCP_INT BIT(4)
#define LDO6_OCP_INT BIT(5)
#define LDO7_OCP_INT BIT(6)

//Interrupt 3
#define LDO12_UVLO_INT BIT(0)
#define LDO34_UVLO_INT BIT(1)
#define LDO5_UVLO_INT BIT(2)
#define LDO6_UVLO_INT BIT(3)
#define LDO7_UVLO_INT BIT(4)
#define VSYS_UVLO_INT BIT(5)
#define TSD_WRN_INT BIT(6)
#define TSD_INT BIT(7)

struct fan53870_i2c_reg_array {
	uint32_t reg_addr;
	uint32_t reg_data;
};

struct fan53870_init_settings {
	uint32_t size;
	struct fan53870_i2c_reg_array *reg_settings;
};

struct fan53870_chip {
	struct device *dev;
	struct regmap *regmap;
	unsigned int gpio_reset;
	unsigned int gpio_int;
	unsigned int gpio_int_irq;
	struct fan53870_init_settings init_settings;
};

struct fan53870_regulator {
	struct device *dev;
	struct regmap *regmap;
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
	struct regulator *parent_supply;
	struct regulator *en_supply;
	struct device_node *of_node;
	unsigned int gpio_reset;
	u16 offset;
	int min_dropout_uv;
	int iout_ua;
};

struct regulator_data {
	char *name;
	char *supply_name;
	int default_mv;
	int min_dropout_uv;
	int iout_ua;
};


#if defined(CONFIG_MTK_CAM_PD2083F_EX)
//zhenrong add for fan53870 reset if i2c can not work
struct fan53870_reset_data {
	unsigned int reset_gpio;
	struct mutex		lock;
	bool is_probe_successed;
};
//zhenrong add for fan53870 reset if i2c can not work
#endif


static struct regulator_data reg_data[] = {
	/* name,  parent,   headroom */
	{ "fan53870_l1", "vdd_l1_l2", 1048, 225000, 650000 },
	{ "fan53870_l2", "vdd_l1_l2", 1048, 225000, 650000 },
	{ "fan53870_l3", "vdd_l3_l4", 2804, 200000, 650000 },
	{ "fan53870_l4", "vdd_l3_l4", 2804, 200000, 650000 },
	{ "fan53870_l5", "vdd_l5", 1804, 300000, 650000 },
	{ "fan53870_l6", "vdd_l6", 2804, 300000, 650000 },
	{ "fan53870_l7", "vdd_l7", 2804, 300000, 650000 },
};

static const struct regmap_config fan53870_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static u8 fan53870_ldo_comp0;
static u8 fan53870_ldo_comp1;

#if defined(CONFIG_MTK_CAM_PD2083F_EX)
//zhenrong add for fan53870 reset if i2c can not work
static struct fan53870_reset_data reset_fan53870 = {0};

static int fan53870_reset(void)
{
	pr_info("Doing reset fan53870\n");
	mutex_lock(&reset_fan53870.lock);
	gpio_direction_output(reset_fan53870.reset_gpio, 0);
	pr_err("fan53870:pull down reset_gpio\n");
	mdelay(2);
	gpio_direction_output(reset_fan53870.reset_gpio, 1);
	pr_err("fan53870:pull up reset_gpio\n");
	mdelay(2);
	mutex_unlock(&reset_fan53870.lock);
	return 0;
}
#endif
/*common functions*/
static int fan53870_read(struct regmap *regmap, u16 reg, u8 *val, int count)
{
	int rc = 0;	
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
	retry:
		mutex_lock(&reset_fan53870.lock);
#endif
		rc = regmap_bulk_read(regmap, reg, val, count);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
		mutex_unlock(&reset_fan53870.lock);
#endif
		if (rc < 0) {
			pr_err("failed to read 0x%04x\n", reg);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
				if (reset_fan53870.is_probe_successed == true && fan53870_reset() == 0)
					goto retry;
#endif
		}
	return rc;	
}

static int fan53870_write(struct regmap *regmap, u16 reg, u8 *val, int count)
{
	int rc = 0;	
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
	retry:
		mutex_lock(&reset_fan53870.lock);
#endif
		pr_debug("Writing 0x%02x to 0x%02x\n", *val, reg);
		rc = regmap_bulk_write(regmap, reg, val, count);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
		mutex_unlock(&reset_fan53870.lock);
#endif
		if (rc < 0) {
			pr_err("failed to write 0x%04x\n", reg);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
			if (reset_fan53870.is_probe_successed == true && fan53870_reset() == 0)
				goto retry;
#endif
		}
	return rc;
}

static int fan53870_masked_write(struct regmap *regmap, u16 reg, u8 mask,
				 u8 val)
{
	int rc = 0;
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
	retry:
		mutex_lock(&reset_fan53870.lock);
#endif
		pr_debug("Writing 0x%02x to 0x%04x with mask 0x%02x\n", val, reg, mask);
		rc = regmap_update_bits(regmap, reg, mask, val);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
		mutex_unlock(&reset_fan53870.lock);
#endif
		if (rc < 0) {
			pr_err("failed to write 0x%02x to 0x%04x with mask 0x%02x\n",
			       val, reg, mask);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)
			if (reset_fan53870.is_probe_successed == true && fan53870_reset() == 0)
					goto retry;
#endif
		}	
	return rc;	
}
//zhenrong add for fan53870 reset if i2c can not work


static int fan53870_enable(struct fan53870_regulator *fan_reg)
{
	 if (!gpio_is_valid(fan_reg->gpio_reset)) {
		 pr_info("FAN53870 gpio reset is invalid\n");
	 } else {
		 if (gpio_get_value(fan_reg->gpio_reset) == 0) {
			 gpio_direction_output(fan_reg->gpio_reset, 1);
			 usleep_range(500, 600);
			 pr_info("fan53870_enable gpio reset to high\n");
		 }
	 }
	 return 0;
}

static int fan53870_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct fan53870_regulator *fan_reg = rdev_get_drvdata(rdev);
	int rc;
	u8 reg;
	fan53870_enable(fan_reg);
	rc = fan53870_read(fan_reg->regmap, FAN53870_REG_ENABLE, &reg, 1);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to read enable reg rc = %d\n",
			     rc);
		return rc;
	}
	return !!(reg & (1u << fan_reg->offset));
}

static int fan53870_regulator_enable(struct regulator_dev *rdev)
{
	struct fan53870_regulator *fan_reg = rdev_get_drvdata(rdev);
	int rc;

	fan53870_enable(fan_reg);
	if (fan_reg->parent_supply) {
		rc = regulator_enable(fan_reg->parent_supply);
		if (rc < 0) {
			fan53870_err(fan_reg, "failed to enable parent rc=%d\n",
				     rc);
			return rc;
		}
	}

	rc = fan53870_write(fan_reg->regmap, 0x13, &fan53870_ldo_comp0, 1);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to write LDO_COMP0 rc=%d\n", rc);
	}

	rc = fan53870_write(fan_reg->regmap, 0x14, &fan53870_ldo_comp1, 1);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to write LDO_COMP1 rc=%d\n", rc);
	}

	rc = fan53870_masked_write(fan_reg->regmap, FAN53870_REG_ENABLE,
				   1u << fan_reg->offset,
				   1u << fan_reg->offset);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to enable regulator rc=%d\n", rc);
		goto remove_vote;
	}

	return 0;

remove_vote:
	if (fan_reg->parent_supply)
		rc = regulator_disable(fan_reg->parent_supply);
	if (rc < 0)
		fan53870_err(fan_reg,
			     "failed to disable parent regulator rc=%d\n", rc);
	return -ETIME;
}

static int fan53870_disable(struct fan53870_regulator *fan_reg)
{
	int rc;
	u8 reg;

	rc = fan53870_read(fan_reg->regmap, FAN53870_REG_ENABLE, &reg, 1);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to read enable reg rc = %d\n",
			     rc);
		return rc;
	}

	if (reg & 0x7F)
		return 0;

	if (!gpio_is_valid(fan_reg->gpio_reset)) {
		pr_info("FAN53870 gpio reset is invalid\n");
	} else {
		gpio_set_value(fan_reg->gpio_reset, 0);
		pr_info("fan53870_disable reset pin have set low\n");
	}
	return 0;
}

static int fan53870_regulator_disable(struct regulator_dev *rdev)
{
	struct fan53870_regulator *fan_reg = rdev_get_drvdata(rdev);
	int rc;

	rc = fan53870_masked_write(fan_reg->regmap, FAN53870_REG_ENABLE,
				   1u << fan_reg->offset, 0);

	if (rc < 0) {
		fan53870_err(fan_reg, "failed to disable regulator rc=%d\n",
			     rc);
		return rc;
	}

	/*remove voltage vote from parent regulator */
	if (fan_reg->parent_supply) {
		rc = regulator_set_voltage(fan_reg->parent_supply, 0, INT_MAX);
		if (rc < 0) {
			fan53870_err(fan_reg,
				     "failed to remove parent voltage rc=%d\n",
				     rc);
			return rc;
		}
		rc = regulator_disable(fan_reg->parent_supply);
		if (rc < 0) {
			fan53870_err(fan_reg,
				     "failed to disable parent rc=%d\n", rc);
			return rc;
		}
	}

	fan53870_disable(fan_reg);

	fan53870_debug(fan_reg, "regulator disabled\n");
	return 0;
}

static int fan53870_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct fan53870_regulator *fan_reg = rdev_get_drvdata(rdev);
	u8 vset;
	int rc;
	int uv;
	fan53870_enable(fan_reg);
	rc = fan53870_read(fan_reg->regmap, LDO_VSET_REG(fan_reg->offset),
			   &vset, 1);
	if (rc < 0) {
		fan53870_err(fan_reg,
			     "failed to read regulator voltage rc = %d\n", rc);
		return rc;
	}

	if (vset == 0)
		uv = reg_data[fan_reg->offset].default_mv * 1000;
	else {
		fan53870_debug(fan_reg, "voltage read [%x]\n", vset);
		if (fan_reg->offset == 0 || fan_reg->offset == 1)
			uv = (VSET_BASE_12 + (vset - 99) * VSET_STEP_MV) * 1000;
		else
			uv = (VSET_BASE_34567 + (vset - 16) * VSET_STEP_MV) *
			     1000;
	}
	return uv;
}

static int fan53870_write_voltage(struct fan53870_regulator *fan_reg,
				  int min_uv, int max_uv)
{
	int rc = 0, mv;
	u8 vset;

	mv = DIV_ROUND_UP(min_uv, 1000);
	if (mv * 1000 > max_uv) {
		fan53870_err(fan_reg, "requestd voltage above maximum limit\n");
		return -EINVAL;
	}

	if (fan_reg->offset == 0 || fan_reg->offset == 1)
		vset = DIV_ROUND_UP(mv - VSET_BASE_12, VSET_STEP_MV) + 99;
	else
		vset = DIV_ROUND_UP(mv - VSET_BASE_34567, VSET_STEP_MV) + 16;

	rc = fan53870_write(fan_reg->regmap, LDO_VSET_REG(fan_reg->offset),
			    &vset, 1);
	if (rc < 0) {
		fan53870_err(fan_reg, "failed to write voltage rc = %d\n", rc);
		return rc;
	}

	fan53870_debug(fan_reg, "VSET=[%2x]\n", vset);
	return 0;
}

static int fan53870_regulator_set_voltage(struct regulator_dev *rdev,
					  int min_uv, int max_uv,
					  unsigned int *selector)
{
	struct fan53870_regulator *fan_reg = rdev_get_drvdata(rdev);
	int rc;
	fan53870_enable(fan_reg);
	if (fan_reg->parent_supply) {
		rc = regulator_set_voltage(fan_reg->parent_supply,
					   fan_reg->min_dropout_uv + min_uv,
					   INT_MAX);
		if (rc < 0) {
			fan53870_err(
				fan_reg,
				"failed to request parent supply voltage rc=%d\n",
				rc);
			return rc;
		}
	}

	rc = fan53870_write_voltage(fan_reg, min_uv, max_uv);
	if (rc < 0) {
		/* remove parentn's voltage vote */
		if (fan_reg->parent_supply)
			regulator_set_voltage(fan_reg->parent_supply, 0,
					      INT_MAX);
		if (!gpio_is_valid(fan_reg->gpio_reset)) {
			pr_info("FAN53870 gpio reset is invalid\n");
			} else {
				gpio_set_value(fan_reg->gpio_reset, 0);
				pr_info("fan53870_disable reset pin have set low\n");
			}
			fan53870_err(fan_reg, "failed to set voltage rc = %d\n", rc);
			return rc;
	}
	fan53870_debug(fan_reg, "voltage set to %d\n", min_uv);
	return rc;
}

static struct regulator_ops fan53870_regulator_ops = {
	.enable = fan53870_regulator_enable,
	.disable = fan53870_regulator_disable,
	.is_enabled = fan53870_regulator_is_enabled,
	.set_voltage = fan53870_regulator_set_voltage,
	.get_voltage = fan53870_regulator_get_voltage,
	//    .set_mode = fan53870_regulator_set_mode,
	//    .get_mode = fan53870_regulator_get_mode,
	//    .set_load = fan53870_regulator_set_load,
	//    .set_voltage_time = fan53870_regulator_set_voltage_time,
};

static int fan53870_register_ldo(struct fan53870_regulator *fan53870_reg,
				 const char *name)
{
	struct regulator_config reg_config = {};
	struct regulator_init_data *init_data;

	struct device_node *reg_node = fan53870_reg->of_node;
	struct device *dev = fan53870_reg->dev;
	int rc, i, init_voltage;
	char buff[MAX_REG_NAME];

	/* try to find ldo pre-defined in the regulator table */
	for (i = 0; i < FAN53870_MAX_LDO; i++)
		if (!strcmp(reg_data[i].name, name))
			break;

	if (i == FAN53870_MAX_LDO) {
		pr_err("Invalid regulator name %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u16(reg_node, "offset", &fan53870_reg->offset);
	if (rc < 0) {
		pr_err("%s:failed to get regulator offset rc = %d\n", name, rc);
		return rc;
	}

	//assign default value defined in code.
	fan53870_reg->min_dropout_uv = reg_data[i].min_dropout_uv;
	of_property_read_u32(reg_node, "min-dropout-voltage",
			     &fan53870_reg->min_dropout_uv);

	fan53870_reg->iout_ua = reg_data[i].iout_ua;
	of_property_read_u32(reg_node, "iout_ua", &fan53870_reg->iout_ua);

	init_voltage = -EINVAL;
	of_property_read_u32(reg_node, "init-voltage", &init_voltage);

	scnprintf(buff, MAX_REG_NAME, "%s-supply", reg_data[i].supply_name);
	if (of_find_property(dev->of_node, buff, NULL)) {
		fan53870_reg->parent_supply =
			devm_regulator_get(dev, reg_data[i].supply_name);
		if (IS_ERR(fan53870_reg->parent_supply)) {
			rc = PTR_ERR(fan53870_reg->parent_supply);
			if (rc != EPROBE_DEFER)
				pr_err("%s: failed to get parent regulator rc = %d\n",
				       name, rc);
			return rc;
		}
	}

	init_data =
		of_get_regulator_init_data(dev, reg_node, &fan53870_reg->rdesc);
	if (init_data == NULL) {
		pr_err("%s: failed to get regulator data\n", name);
		return -ENODATA;
	}

	if (!init_data->constraints.name) {
		pr_err("%s: regulator name missing\n", name);
		return -EINVAL;
	}

	/* configure the initial voltage for the regulator */
	if (init_voltage > 0) {
		rc = fan53870_write_voltage(fan53870_reg, init_voltage,
					    init_data->constraints.max_uV);
		if (rc < 0)
			pr_err("%s:failed to set initial voltage rc = %d\n",
			       name, rc);
	}

	init_data->constraints.input_uV = init_data->constraints.max_uV;
	init_data->constraints.valid_ops_mask |=
		REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE;

	reg_config.dev = dev;
	reg_config.init_data = init_data;
	reg_config.driver_data = fan53870_reg;
	reg_config.of_node = reg_node;

	fan53870_reg->rdesc.owner = THIS_MODULE;
	fan53870_reg->rdesc.type = REGULATOR_VOLTAGE;
	fan53870_reg->rdesc.ops = &fan53870_regulator_ops;
	fan53870_reg->rdesc.name = init_data->constraints.name;
	fan53870_reg->rdesc.n_voltages = 1;

	fan53870_reg->rdev =
		devm_regulator_register(dev, &fan53870_reg->rdesc, &reg_config);
	if (IS_ERR(fan53870_reg->rdev)) {
		rc = PTR_ERR(fan53870_reg->rdev);
		pr_err("%s: failed to register regulator rc =%d\n",
		       fan53870_reg->rdesc.name, rc);
		return rc;
	}

	pr_info("%s regulator register\n", name);
	return 0;
}

static int fan53870_parse_regulator(struct regmap *regmap,
				    unsigned int gpio_reset, struct device *dev)
{
	int rc = 0;
	const char *name;
	struct device_node *child;
	struct fan53870_regulator *fan53870_reg;

	/* parse each regulator */
	for_each_available_child_of_node (dev->of_node, child) {
		fan53870_reg =
			devm_kzalloc(dev, sizeof(*fan53870_reg), GFP_KERNEL);
		if (!fan53870_reg)
			return -ENOMEM;

		fan53870_reg->regmap = regmap;
		fan53870_reg->gpio_reset = gpio_reset;
		fan53870_reg->of_node = child;
		fan53870_reg->dev = dev;

		rc = of_property_read_string(child, "regulator-name", &name);
		if (rc)
			continue;

		rc = fan53870_register_ldo(fan53870_reg, name);
		if (rc < 0) {
			pr_err("failed to register regulator %s rc = %d\n",
			       name, rc);
			return rc;
		}
	}
	return 0;
}

static int fan53870_initializeGPIO(struct fan53870_chip *chip)
{
	int rc;
	struct device_node *node;

	node = chip->dev->of_node;

	//Reset Pin
	if (of_parse_phandle(node, "fan53870,reset", 0) != NULL) {
		chip->gpio_reset = of_get_named_gpio(node, "fan53870,reset", 0);
		if (!gpio_is_valid(chip->gpio_reset)) {
			pr_err("FAN53870: failed to get named reset gpio\n");
		} else {
			rc = gpio_request(chip->gpio_reset, "fan53870 reset");
			if (rc < 0) {
				pr_err("FAN53870 could not request GPIO for reset\n");
				return rc;
			}
#if defined(CONFIG_MTK_CAM_PD2083F_EX)			
			//zhenrong add for fan53870 reset beg
			reset_fan53870.reset_gpio = chip->gpio_reset;
			//zhenrong add for fan53870 reset end
#endif			
			gpio_set_value(chip->gpio_reset, 1);
		}
	} else {
		pr_info("FAN53870: failed to find  fan53870,reset configuration\n");
	}

	//Interrupt Pin
	if (of_parse_phandle(node, "fan53870,irq", 0) != NULL) {
		chip->gpio_int = of_get_named_gpio(node, "fan53870,irq", 0);
		if (!gpio_is_valid(chip->gpio_int)) {
			pr_err("FAN53870:failed to get irq named int.\n");
			//return chip->gpio_int;
		} else {
			rc = gpio_request(chip->gpio_int, "fan53870 int");
			if (rc < 0) {
				pr_err("FAN53870 could not request GPIO for interrupt!\n");
				return rc;
			}
		}
	} else {
		pr_info("FAN53870: failed to find  fan53870,irq configuration\n");
	}
	return 0;
}
static void fan53870_write_init_settings(struct fan53870_chip *chip)
{
	int rc, i;
	u16 reg;
	u8 val;
	for (i = 0; i < chip->init_settings.size; i++) {
		reg = (u16)(chip->init_settings.reg_settings[i].reg_addr & 0xFF);
		val = (u8)(chip->init_settings.reg_settings[i].reg_data & 0xFF);
		rc = fan53870_write(chip->regmap, reg, &val, 1);
		if (rc < 0) {
			pr_info("FAN53870: failed to write reg:0x%02x data:0x%02x\n", reg, val);
		}
	}
}
static void fan53870_init_reg_settings(struct fan53870_chip *chip)
{
	int rc, i;
	struct device_node *node;
	int num_of_regs;
	uint32_t temp;

	node = chip->dev->of_node;
	chip->init_settings.size = 0;
	fan53870_ldo_comp0 = 0;
	fan53870_ldo_comp1 = 0;
	if (of_parse_phandle(node, "fan53870,init-settings", 0) != NULL) {
		num_of_regs = of_property_count_u32_elems(node, "fan53870,init-settings");
		if (num_of_regs > 0 && num_of_regs % 2 == 0) {
			num_of_regs = num_of_regs / 2;
			chip->init_settings.size = num_of_regs;
			chip->init_settings.reg_settings = (struct fan53870_i2c_reg_array *)
					vzalloc(sizeof(struct fan53870_i2c_reg_array) * num_of_regs);
			if (chip->init_settings.reg_settings != NULL) {
				for (i = 0; i < num_of_regs; i++) {
					rc = of_property_read_u32_index(node, "fan53870,init-settings",
						(2 * i), &temp);
					chip->init_settings.reg_settings[i].reg_addr = temp;
					rc = of_property_read_u32_index(node, "fan53870,init-settings",
						(2 * i + 1), &temp);
					chip->init_settings.reg_settings[i].reg_data = temp;
					if (chip->init_settings.reg_settings[i].reg_addr == 0x13)
						fan53870_ldo_comp0 = chip->init_settings.reg_settings[i].reg_data;
					else if (chip->init_settings.reg_settings[i].reg_addr == 0x14)
						fan53870_ldo_comp1 = chip->init_settings.reg_settings[i].reg_data;
					pr_info("FAN53870: addr:0x%02x data:0x%02x\n",
						chip->init_settings.reg_settings[i].reg_addr, 
						chip->init_settings.reg_settings[i].reg_data);
				}
			} else {
				chip->init_settings.size = 0;
				pr_info("FAN53870: failed to alloc fan53870_i2c_reg_array\n");
			}
		}
	}
	fan53870_write_init_settings(chip);
}
static int fan53870_cleanGPIO(struct fan53870_chip *chip)
{
	if (gpio_is_valid(chip->gpio_int) && chip->gpio_int_irq != -1)
		devm_free_irq(chip->dev, chip->gpio_int_irq, chip);

	if (gpio_is_valid(chip->gpio_int))
		gpio_free(chip->gpio_int);

	if (gpio_is_valid(chip->gpio_reset)) {
		//pinctrl_gpio_free(chip->gpio_reset);
		gpio_free(chip->gpio_reset);
	}
	return 0;
}

static irqreturn_t fan53870_irq_handler(int irq, void *dev_id)
{
	struct fan53870_chip *chip = (struct fan53870_chip *)dev_id;
	u8 ints[3];
	u8 int1, int2, int3;
	int rc;

	rc = fan53870_read(chip->regmap, FAN53870_REG_INTERRUPT1, ints, 3);
	int1 = ints[0];
	int2 = ints[1];
	int3 = ints[2];

	if (int1 != 0) {
		//some ldo uvp
	}

	if (int2 != 0) {
		//some ldo ocp
	}

	if (int3 != 0) {
		//themal or input uvp
	}

	return IRQ_HANDLED;
}

static int fan53870_initialzieInterrupt(struct fan53870_chip *chip)
{
	int rc;
	rc = gpio_to_irq(chip->gpio_int);
	if (rc < 0) {
		pr_err("FAN53870: Unable to request IRQ for INT GPIO!\n");
		chip->gpio_int_irq = -1;
		fan53870_cleanGPIO(chip);
		return rc;
	}

	chip->gpio_int_irq = rc;

	pr_info("FAN53870: Got INT IRQ!\n");

	rc = devm_request_threaded_irq(chip->dev, chip->gpio_int_irq, NULL,
				       fan53870_irq_handler,
				       IRQF_ONESHOT | IRQF_TRIGGER_RISING,
				       "fan53870 ldo int", chip);
	if (rc) {
		pr_err("Unable to request thread IRQ for INTB GPIO!\n");
		fan53870_cleanGPIO(chip);
		return rc;
	}
	return 0;
}

static int fan53870_regulator_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int rc = 0;
	unsigned int val = 0;
	struct regmap *regmap;
	struct fan53870_chip *chip;
	
#if defined(CONFIG_MTK_CAM_PD2083F_EX)		
	//zhenrong add for fan53870 reset beg
	reset_fan53870.is_probe_successed = false;
	mutex_init(&reset_fan53870.lock);
	//zhenrong add for fan53870 reset end
#endif

	regmap = devm_regmap_init_i2c(client, &fan53870_regmap_config);
	if (IS_ERR(regmap)) {
		pr_err("FAN53870 failed to allocate regmap\n");
		return PTR_ERR(regmap);
	}

	/*fan53870 chip*/
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("FAN53870 failed to alloc fan53870_chip\n");
		return -ENOMEM;
	}

	chip->dev = &client->dev;
	chip->regmap = regmap;
	i2c_set_clientdata(client, chip);
	rc = fan53870_initializeGPIO(chip);
#if defined(CONFIG_MTK_CAM_PD2083F_EX)	
	if (rc == -EBUSY) {
		pr_err("FAN53870: Unable to initialize GPIO\n");
		//zhenrong add for pmic compatilbe beg
		return rc;
		//zhenrong add for pmic compatilbe end
	}
#else
	if (rc < 0) {
		pr_err("FAN53870: Unable to initialize GPIO\n");
	}
#endif	


	fan53870_init_reg_settings(chip);

	rc = regmap_read(regmap, 0x00, &val);
	if (rc < 0) {
		pr_err("FAN53870 failed to get PID\n");
		fan53870_cleanGPIO(chip);
		return rc;
	} else {
		pr_info("FAN53870 get Product ID: [%02x]\n", val);
	}

	if (gpio_is_valid(chip->gpio_int)) {
		rc = fan53870_initialzieInterrupt(chip);
		if (rc < 0) {
			return -EIO;
		}
	}
	rc = fan53870_parse_regulator(regmap, chip->gpio_reset, &client->dev);
	if (rc < 0) {
		pr_err("FAN53870 failed to parse device tree rc=%d\n", rc);
		return rc;
	}
#if defined(CONFIG_MTK_CAM_PD2083F_EX)	
	//zhenrong add for fan53870 reset beg
	reset_fan53870.is_probe_successed = true;
	//zhenrong add for fan53870 reset end
#endif
	pr_info("FAN53870: probe done!\n");
	return 0;
}

static int fan53870_remove(struct i2c_client *client)
{
	struct fan53870_chip *chip =
		(struct fan53870_chip *)i2c_get_clientdata(client);
	if (chip != NULL) {
		gpio_free(chip->gpio_reset);
	}
	return 0;
}

static const struct of_device_id fan53870_dt_ids[] = {
	{
		.compatible = "onsemi,fan53870",
	},
	{}
};
MODULE_DEVICE_TABLE(of, fan53870_dt_ids);

static const struct i2c_device_id fan53870_id[] = {
	{
		.name = "fan53870-regulator",
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(i2c, fan53870_id);

static struct i2c_driver fan53870_regulator_driver = {
		.driver = {
			.name = "fan53870-regulator",
			.of_match_table = of_match_ptr(fan53870_dt_ids),
		},
		.probe = fan53870_regulator_probe,
		.remove = fan53870_remove,
		.id_table = fan53870_id,
};

module_i2c_driver(fan53870_regulator_driver);

MODULE_DESCRIPTION("Fan53870 PMIC Regulator Driver");
MODULE_LICENSE("GPL v2");
