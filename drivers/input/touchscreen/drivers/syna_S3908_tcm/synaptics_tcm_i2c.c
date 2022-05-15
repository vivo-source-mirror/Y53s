/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include "synaptics_tcm_core.h"

#define XFER_ATTEMPTS 3
#define RESET_RETRY_TIMES 5

static unsigned char *buf;

static struct syna_tcm_bus_io bus_io;

static struct syna_tcm_hw_interface hw_if;

static struct platform_device *syna_tcm_i2c_device;

#ifdef CONFIG_OF
static int parse_dt(struct device *dev, struct syna_tcm_board_data *bdata)
{
	int retval;
	u32 value;
	struct property *prop;
	struct device_node *np = dev->of_node;
	const char *name;

	prop = of_find_property(np, "synaptics,irq-gpio", NULL);
	if (prop && prop->length) {
		bdata->irq_gpio = of_get_named_gpio_flags(np,
				"synaptics,irq-gpio", 0,
				(enum of_gpio_flags *)&bdata->irq_flags);
	} else {
		bdata->irq_gpio = -1;
	}

	retval = of_property_read_u32(np, "synaptics,irq-on-state", &value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	retval = of_property_read_string(np, "synaptics,pwr-reg-name", &name);
	if (retval < 0)
		bdata->pwr_reg_name = NULL;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval < 0)
		bdata->bus_reg_name = NULL;
	else
		bdata->bus_reg_name = name;

	/*gpio supply 3V*/
	prop = of_find_property(np, "synaptics,power-gpio", NULL);
	if (prop && prop->length) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
	} else {
		bdata->power_gpio = -1;
	}

	/*gpio supply 1.8V*/
	prop = of_find_property(np, "synaptics,vddi-gpio", NULL);
	if (prop && prop->length) {
		bdata->vddi_gpio = of_get_named_gpio_flags(np,
				"synaptics,vddi-gpio", 0, NULL);
	} else {
		bdata->vddi_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,power-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read synaptics,power-on-state property\n");
			return retval;
		} else {
			bdata->power_on_state = value;
		}
	} else {
		bdata->power_on_state = 0;
	}

	prop = of_find_property(np, "synaptics,power-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read synaptics,power-delay-ms property\n");
			return retval;
		} else {
			bdata->power_delay_ms = value;
		}
	} else {
		bdata->power_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-gpio", NULL);
	if (prop && prop->length) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
	} else {
		bdata->reset_gpio = -1;
	}

	prop = of_find_property(np, "synaptics,reset-on-state", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read synaptics,reset-on-state property\n");
			return retval;
		} else {
			bdata->reset_on_state = value;
		}
	} else {
		bdata->reset_on_state = 0;
	}

	prop = of_find_property(np, "synaptics,reset-active-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Failed to read synaptics,reset-active-ms property\n");
			return retval;
		} else {
			bdata->reset_active_ms = value;
		}
	} else {
		bdata->reset_active_ms = 0;
	}

	prop = of_find_property(np, "synaptics,reset-delay-ms", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read synaptics,reset-delay-ms property\n");
			return retval;
		} else {
			bdata->reset_delay_ms = value;
		}
	} else {
		bdata->reset_delay_ms = 0;
	}

	prop = of_find_property(np, "synaptics,x-flip", NULL);
	bdata->x_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,y-flip", NULL);
	bdata->y_flip = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,swap-axes", NULL);
	bdata->swap_axes = prop > 0 ? true : false;

	prop = of_find_property(np, "synaptics,ubl-i2c-addr", NULL);
	if (prop && prop->length) {
		retval = of_property_read_u32(np, "synaptics,ubl-i2c-addr",
				&value);
		if (retval < 0) {
			LOGE(dev,
					"Unable to read synaptics,ubl-i2c-addr property\n");
			return retval;
		} else {
			bdata->ubl_i2c_addr = value;
			LOGE(dev,
					"synaptics,ubl-i2c-addr = %02x\n", bdata->ubl_i2c_addr);
		}
	} else {
		bdata->ubl_i2c_addr = 0;
		LOGE(dev,
				"Unable to read synaptics,ubl-i2c-addr property, bdata->ubl_i2c_addr = 0\n");
	}

	bdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(bdata->pinctrl)) {
		VTE("Failed to get pinctrl");
		return PTR_ERR(bdata->pinctrl);
	}

	bdata->pin_active = pinctrl_lookup_state(bdata->pinctrl, "ts_pinctrl_active");
	if (IS_ERR_OR_NULL(bdata->pin_active)) {
		VTE("Failed to look up active state");
		return PTR_ERR(bdata->pin_active);
	}

	bdata->pin_suspend = pinctrl_lookup_state(bdata->pinctrl, "ts_pinctrl_suspend");
	if (IS_ERR_OR_NULL(bdata->pin_suspend)) {
		VTE("Failed to look up suspend state");
		return PTR_ERR(bdata->pin_suspend);
	}

	return 0;
}
#endif

static int syna_tcm_i2c_alloc_mem(struct syna_tcm_hcd *tcm_hcd,
		unsigned int size)
{
	static unsigned int buf_size;
	struct i2c_client *i2c = to_i2c_client(tcm_hcd->pdev->dev.parent);

	if (size > buf_size) {
		if (buf_size)
			kfree(buf);
		buf = kmalloc(size, GFP_KERNEL);
		if (!buf) {
			LOGE(&i2c->dev,
					"Failed to allocate memory for buf\n");
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = size;
	}

	return 0;
}

static int syna_tcm_i2c_rmi_read(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned char address;
	unsigned int attempt;
	struct i2c_msg msg[2];
	struct i2c_client *i2c = to_i2c_client(tcm_hcd->pdev->dev.parent);
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	address = (unsigned char)addr;

	msg[0].addr = bdata->ubl_i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &address;

	msg[1].addr = bdata->ubl_i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		retval = i2c_transfer(i2c->adapter, msg, 2);

		if (retval == 2) {
			retval = length;
			goto exit;
		}
		LOGE(&i2c->dev,
				"Transfer attempt %d failed, retval = %d\n",
				attempt + 1, retval);

		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int syna_tcm_i2c_rmi_write(struct syna_tcm_hcd *tcm_hcd,
		unsigned short addr, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int attempt;
	unsigned int byte_count;
	struct i2c_msg msg;
	struct i2c_client *i2c = to_i2c_client(tcm_hcd->pdev->dev.parent);
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	byte_count = length + 1;

	retval = syna_tcm_i2c_alloc_mem(tcm_hcd, byte_count);
	if (retval < 0) {
		LOGE(&i2c->dev,
				"Failed to allocate memory\n");
		goto exit;
	}

	buf[0] = (unsigned char)addr;
	retval = secure_memcpy(&buf[1],
			length,
			data,
			length,
			length);
	if (retval < 0) {
		LOGE(&i2c->dev,
				"Failed to copy write data\n");
		goto exit;
	}

	msg.addr = bdata->ubl_i2c_addr;
	msg.flags = 0;
	msg.len = byte_count;
	msg.buf = buf;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			goto exit;
		}
		LOGE(&i2c->dev,
				"Transfer attempt %d failed\n",
				attempt + 1);

		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto exit;
		}

		msleep(20);
	}

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int i2c_fail_times = 0;
static int syna_tcm_i2c_read(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = 0;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = to_i2c_client(tcm_hcd->pdev->dev.parent);

	mutex_lock(&tcm_hcd->io_ctrl_mutex);

	msg.addr = i2c->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			i2c_fail_times = 0;
			goto exit;
		}
		LOGE(&i2c->dev,
				"Transfer attempt %d failed\n",
				attempt + 1);

		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto hw_reset;
		}

		msleep(20);
	}

hw_reset:
	if (i2c_fail_times >= RESET_RETRY_TIMES) {
		VTE("reset over 5 times, not reset again!");
		goto exit;
	}
	i2c_fail_times ++;
	schedule_work(&tcm_hcd->syna_reset_work);


exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int syna_tcm_i2c_write(struct syna_tcm_hcd *tcm_hcd, unsigned char *data,
		unsigned int length)
{
	int retval = 0;
	unsigned int attempt;
	struct i2c_msg msg;
	struct i2c_client *i2c = to_i2c_client(tcm_hcd->pdev->dev.parent);

	mutex_lock(&tcm_hcd->io_ctrl_mutex);
	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = length;
	msg.buf = data;

	for (attempt = 0; attempt < XFER_ATTEMPTS; attempt++) {
		if (i2c_transfer(i2c->adapter, &msg, 1) == 1) {
			retval = length;
			i2c_fail_times = 0;
			goto exit;
		}
		LOGE(&i2c->dev,
				"Transfer attempt %d failed, addr=0x%x\n",
				attempt + 1, msg.addr);

		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);

		if (attempt + 1 == XFER_ATTEMPTS) {
			retval = -EIO;
			goto hw_reset;
		}

		msleep(20);
	}

hw_reset:
	if (i2c_fail_times >= RESET_RETRY_TIMES) {
		VTE("reset over 5 times, not reset again!");
		goto exit;
	}
	i2c_fail_times ++;
	schedule_work(&tcm_hcd->syna_reset_work);

exit:
	mutex_unlock(&tcm_hcd->io_ctrl_mutex);

	return retval;
}

static int syna_tcm_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *dev_id)
{
	int retval;
	VTI("I2C probe start !");
	syna_tcm_i2c_device = platform_device_alloc(PLATFORM_DRIVER_NAME, 0);
	if (!syna_tcm_i2c_device) {
		LOGE(&i2c->dev,
				"Failed to allocate platform device\n");
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	hw_if.bdata = devm_kzalloc(&i2c->dev, sizeof(*hw_if.bdata), GFP_KERNEL);
	if (!hw_if.bdata) {
		LOGE(&i2c->dev,
				"Failed to allocate memory for board data\n");
		return -ENOMEM;
	}
	parse_dt(&i2c->dev, hw_if.bdata);
#else
	hw_if.bdata = i2c->dev.platform_data;
#endif

	bus_io.type = BUS_I2C;
	bus_io.read = syna_tcm_i2c_read;
	bus_io.write = syna_tcm_i2c_write;
	bus_io.rmi_read = syna_tcm_i2c_rmi_read;
	bus_io.rmi_write = syna_tcm_i2c_rmi_write;

	hw_if.bus_io = &bus_io;

	syna_tcm_i2c_device->dev.parent = &i2c->dev;
	syna_tcm_i2c_device->dev.platform_data = &hw_if;

	retval = platform_device_add(syna_tcm_i2c_device);
	if (retval < 0) {
		LOGE(&i2c->dev,
				"Failed to add platform device\n");
		return retval;
	}

	return 0;
}

static int syna_tcm_i2c_remove(struct i2c_client *i2c)
{
	syna_tcm_i2c_device->dev.platform_data = NULL;

	platform_device_unregister(syna_tcm_i2c_device);

	return 0;
}

static const struct i2c_device_id syna_tcm_id_table[] = {
	{I2C_MODULE_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, syna_tcm_id_table);

#ifdef CONFIG_OF
static struct of_device_id syna_tcm_of_match_table[] = {
	{
		.compatible = "synaptics, synaptics-3618",
	},
	{},
};
MODULE_DEVICE_TABLE(of, syna_tcm_of_match_table);
#else
#define syna_tcm_of_match_table NULL
#endif

static struct i2c_driver syna_tcm_i2c_driver = {
	.driver = {
		.name = I2C_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = syna_tcm_of_match_table,
	},
	.probe = syna_tcm_i2c_probe,
	.remove = syna_tcm_i2c_remove,
	.id_table = syna_tcm_id_table,
};

static int module_bit = 0x00;

int syna_module_init(void)
{
	if (device_module_init()) {
		VTE("fail to execute device_module_init");
	} else {
		module_bit |= (0x01 << 1);
	}
	
	if (recovery_module_init()) {
		VTE("fail to execute recovery_module_init");
	} else {
		module_bit |= (0x01 << 2);
	}
	
	if (reflash_module_init()) {
		VTE("fail to execute reflash_module_init");
	} else {
		module_bit |= (0x01 << 3);
	}
	
	if (testing_module_init()) {
		VTE("fail to execute testing_module_init");
	} else {
		module_bit |= (0x01 << 4);
	}
	
	if (touch_module_init()) {
		VTE("fail to execute touch_module_init");
		return -1;
	}
	module_bit |= (0x01 << 5);
	
	if (vivo_intf_module_init()) {
		VTE("fail to execute vivo_intf_module_init");
		return -1;
	}
	module_bit |= (0x01 << 6);
	
	if (diag_module_init()) {
		VTE("fail to execute diag_module_init");
	} else {
		module_bit |= (0x01 << 7);
	}
	return 0;
}

static int S3908_driver_init(void)
{
	if (i2c_add_driver(&syna_tcm_i2c_driver))
		return -1;
	if (syna_tcm_module_init() < 0) {
		VTE("fail to execute syna_tcm_module_init");
		return -1;
	}
	module_bit |= 0x01;
	return 0;
}

static void S3908_driver_exit(void)
{
	i2c_del_driver(&syna_tcm_i2c_driver);
	if (module_bit & (0x01))
		syna_tcm_module_exit();
	if (module_bit & (0x01 << 1))
		device_module_exit();
	if (module_bit & (0x01 << 2))
		recovery_module_exit();
	if (module_bit & (0x01 << 3))
		reflash_module_exit();
	if (module_bit & (0x01 << 4))
		testing_module_exit();
	if (module_bit & (0x01 << 5))
		touch_module_exit();
	if (module_bit & (0x01 << 6))
		vivo_intf_module_exit();
	if (module_bit & (0x01 << 7))
		diag_module_exit();
}


static const int ic_number[1] = {VTS_IC_S3908};
module_vts_driver(synaptics_S3908, ic_number, S3908_driver_init(), S3908_driver_exit());
//module_vts_driver(synaptics_S3908, ic_number, i2c_add_driver(&syna_tcm_i2c_driver), i2c_del_driver(&syna_tcm_i2c_driver));

