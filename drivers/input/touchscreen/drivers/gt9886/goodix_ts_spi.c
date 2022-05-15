/*
 * Goodix GTx5 I2C Dirver
 * Hardware interface layer of touchdriver architecture.
 *
 * Copyright (C) 2015 - 2016 Goodix, Inc.
 * Authors:  Yulong Cai <caiyulong@goodix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include "goodix_ts_core.h"
#include "goodix_cfg_bin.h"


/*#include "../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"*/

#define TS_DT_COMPATIBLE "goodix,gt9886_V2"
#define TS_DRIVER_NAME "goodix_spi"
#define I2C_MAX_TRANSFER_SIZE	256

#define I2C_READ_MAX_DEAFULT_SIZE  256
#define TS_ADDR_LENGTH	2

#define TS_REG_COORDS_BASE	0x824E
#define TS_REG_CMD	0x8040
#define TS_REG_REQUEST	0x8044
#define TS_REG_VERSION 0x8240
#define TS_REG_CFG_BASE	0x8050
#define TS_REG_DOZE_CTRL		0x30F0
#define TS_REG_DOZE_STAT		0x3100

#define CFG_XMAX_OFFSET (0x8052 - 0x8050)
#define CFG_YMAX_OFFSET	(0x8054 - 0x8050)

#define REQUEST_HANDLED	0x00
#define REQUEST_CONFIG	0x01
#define REQUEST_BAKREF	0x02
#define REQUEST_RESET	0x03
#define REQUEST_MAINCLK	0x04
#define REQUEST_IDLE	0x05

#define COMMAND_SLEEP		0X05
#define COMMAND_CLOSE_HID	0Xaa
#define COMMAND_START_SEND_CFG	0x80
#define COMMAND_END_SEND_CFG	0x83
#define COMMAND_SEND_SMALL_CFG	0x81

#define BYTES_PER_COORD 8
#define TS_MAX_SENSORID	5
#define TS_CFG_MAX_LEN	495
#define FPXStart 450
#define FPYStart 1900
#define FPXEnd 635
#define FPYEnd 2100

/* flag */
#define SPI_FLAG_WR			0xF0
#define SPI_FLAG_RD			0xF1
#define MASK_8BIT			0xFF
/* times*/
#define TS_WAIT_CMD_FREE_RETRY_TIMES	10
#define TS_WAIT_CFG_READY_RETRY_TIMES	30
#define TS_RESET_IC_INIT_RETRY_TIMES	100
#define GOODIX_SPI_BUFF_MAX_SIZE	(8 * 1024 + 16)

#define TS_DOZE_DISABLE_DATA		0xAA
#define TS_DOZE_CLOSE_OK_DATA		0xBB
#define TS_DOZE_ENABLE_DATA		0xCC


/* times*/
#define TS_DOZE_ENABLE_RETRY_TIMES	3
#define TS_DOZE_DISABLE_RETRY_TIMES	6


#if TS_CFG_MAX_LEN > GOODIX_CFG_MAX_SIZE
#error GOODIX_CFG_MAX_SIZE too small, please fix.
#endif

#define FORCE_SEND_CFG 1

static struct platform_device *goodix_pdev;

#ifdef CONFIG_OF
/**
 * goodix_parse_dt_resolution - parse resolution from dt
 * @node: devicetree node
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
*/
static int goodix_parse_dt_resolution(struct device_node *node,
		struct goodix_ts_board_data *board_data)
{
	int r, err;

	r = of_property_read_u32(node, "goodix,panel-max-id",
				&board_data->panel_max_id);
	if (r) {
		err = -ENOENT;
	} else {
		if (board_data->panel_max_id > GOODIX_MAX_TOUCH)
			board_data->panel_max_id = GOODIX_MAX_TOUCH;
	}

	r = of_property_read_u32(node, "goodix,panel-max-x",
				 &board_data->panel_max_x);
	if (r)
		err = -ENOENT;

	r = of_property_read_u32(node, "goodix,panel-max-y",
				&board_data->panel_max_y);
	if (r)
		err = -ENOENT;

	r = of_property_read_u32(node, "goodix,panel-max-w",
				&board_data->panel_max_w);
	if (r)
		err = -ENOENT;

	r = of_property_read_u32(node, "goodix,panel-max-p",
				&board_data->panel_max_p);
	if (r)
		err = -ENOENT;

	board_data->swap_axis = of_property_read_bool(node,
			"goodix,swap-axis");

	board_data->x2x = of_property_read_bool(node,
			"goodix,x2x");

	board_data->y2y = of_property_read_bool(node,
			"goodix,y2y");

	return 0;
}

/**
 * goodix_parse_dt - parse board data from dt
 * @dev: pointer to device
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
*/
static int goodix_parse_dt(struct device_node *node,
	struct goodix_ts_board_data *board_data)
{
	struct property *prop;
	int r;

	if (!board_data) {
		VTE("Invalid board data");
		return -EINVAL;
	}

	/* get spi property */
	r = of_property_read_u32(node, "spi-max-frequency",
			&board_data->spi_max_speed);
	if (r) {
		VTE("set default spi-max-frequency 6M");
		board_data->spi_max_speed = 6*1000*1000;
		//return -EINVAL;
	}

	r = of_get_named_gpio(node, "goodix,reset-gpio", 0);
	if (r < 0) {
		VTE("Invalid reset-gpio in dt: %d", r);
		return -EINVAL;
	} else {
		VTI("Parse reset-gpio[%d] from dt", r);
		board_data->reset_gpio = r;
	}

	r = of_get_named_gpio(node, "goodix,irq-gpio", 0);
	if (r < 0) {
		VTE("Invalid irq-gpio in dt: %d", r);
		return -EINVAL;
	} else {
		VTI("Parse irq-gpio[%d] from dt", r);
		board_data->irq_gpio = r;
	}

	r = of_property_read_u32(node, "goodix,irq-flags",
			&board_data->irq_flags);
	if (r) {
		VTE("Invalid irq-flags");
		return -EINVAL;
	}
	prop = of_find_property(node, "goodix,power-gpio", NULL);
	if (prop && prop->length) {
		VTI("dt find:goodix, power-gpio");
		board_data->power_gpio = of_get_named_gpio_flags(node,
				"goodix,power-gpio", 0, NULL);
		VTI("dt find:goodix,power-gpio = %d", board_data->power_gpio);
	} else {
		board_data->power_gpio = -1;
	}
	
	if (board_data->power_gpio == -1) {
		board_data->avdd_name = "avddvtouch";
	} else {
		board_data->avdd_name = NULL;
	}

	prop = of_find_property(node, "goodix,dvdd-gpio", NULL);
	if (prop && prop->length) {
		VTI("dt find:goodix, dvdd-gpio");
		board_data->dvdd_gpio = of_get_named_gpio_flags(node,
				"goodix,dvdd-gpio", 0, NULL);
		VTI("dt find:goodix,dvdd-gpio = %d", board_data->dvdd_gpio);
	} else {
		board_data->dvdd_gpio = -1;
	}

	if (board_data->dvdd_gpio == -1) {
		board_data->dvdd_name = "vtouch";
	} else {
		board_data->dvdd_name = NULL;
	}

	r = of_property_read_u32(node, "goodix,i2c-read-max-size",
				&board_data->i2c_read_max_size);
	
    if(r < 0 || board_data->i2c_read_max_size <= 0){	
		board_data->i2c_read_max_size = I2C_READ_MAX_DEAFULT_SIZE;
		VTE("i2c_read_max_size not set in dtsi,set default value :%d",board_data->i2c_read_max_size);
		
    }else{
    
    	 VTI("i2c_read_max_size in dtsi is :%d",board_data->i2c_read_max_size);
    }
	
	
	r = of_property_read_u32(node, "goodix,power-on-delay-us",
				&board_data->power_on_delay_us);
	if (!r) {
		/* 1000ms is too large, maybe you have pass
		 * a wrong value */
		if (board_data->power_on_delay_us > 1000 * 1000) {
			VTE("Power on delay time exceed 1s, please check");
			board_data->power_on_delay_us = 0;
		}
	}

	r = of_property_read_u32(node, "goodix,power-off-delay-us",
				&board_data->power_off_delay_us);
	if (!r) {
		/* 1000ms is too large, maybe you have pass
		 * a wrong value */
		if (board_data->power_off_delay_us > 1000 * 1000) {
			VTE("Power off delay time exceed 1s, please check");
			board_data->power_off_delay_us = 0;
		}
	}

	/* get xyz resolutions */
	r = goodix_parse_dt_resolution(node, board_data);
	if (r < 0) {
		VTE("Failed to parse resolutions:%d", r);
		return r;
	}


	/* key map */
	prop = of_find_property(node, "goodix,panel-key-map", NULL);
	if (prop && prop->length) {
		if (prop->length / sizeof(u32) > GOODIX_MAX_KEY) {
			VTE("Size of panel-key-map is invalid");
			return r;
		}

		board_data->panel_max_key = prop->length / sizeof(u32);
		board_data->tp_key_num = prop->length / sizeof(u32);
		r = of_property_read_u32_array(node,
				"goodix,panel-key-map",
				&board_data->panel_key_map[0],
				board_data->panel_max_key);
		if (r)
			return r;
	}

	/*get pen-enable switch and pen keys, must after "key map"*/
	board_data->pen_enable = of_property_read_bool(node, "goodix,pen-enable");
	if (board_data->pen_enable) {
		prop = of_find_property(node, "goodix,key-of-pen", NULL);
		if (prop && prop->length) {
			if (prop->length / sizeof(u32) > GOODIX_PEN_MAX_KEY) {
				VTE("Size of key-of-pen is invalid");
				return r;
			}
			r = of_property_read_u32_array(node,
				"goodix,key-of-pen",
				&board_data->panel_key_map[board_data->panel_max_key],
				prop->length / sizeof(u32));
			if (r)
				return r;

			board_data->panel_max_key += (prop->length / sizeof(u32));
		}
	}
	VTI("***key:%d, %d, %d, %d, %d",
			board_data->panel_key_map[0],
			board_data->panel_key_map[1],
			board_data->panel_key_map[2],
			board_data->panel_key_map[3],
			board_data->panel_key_map[4]);
	/*add end*/


	VTD("[DT]id:%d, x:%d, y:%d, w:%d, p:%d",
			board_data->panel_max_id,
			board_data->panel_max_x,
			board_data->panel_max_y,
			board_data->panel_max_w,
			board_data->panel_max_p);
	return 0;
}


#if 0
/**
 * goodix_parse_dt_cfg - pares config data from devicetree node
 * @dev: pointer to device
 * @cfg_type: config type such as normal_config, highsense_cfg ...
 * @config: pointer to config data structure
 * @sensor_id: sensor id
 * return: 0 - no error, <0 error
*/
static int goodix_parse_dt_cfg(struct goodix_ts_device *dev,
		char *cfg_type, struct goodix_ts_config *config,
		unsigned int sensor_id)
{
	struct device_node *node = dev->dev->of_node;
	struct goodix_ts_board_data *ts_bdata = dev->board_data;
	struct property *prop = NULL;
	char of_node_name[24];
	unsigned int len = 0;
	u16 checksum;

	BUG_ON(config == NULL);
	if (sensor_id > TS_MAX_SENSORID) {
		VTE("Invalid sensor id");
		return -EINVAL;
	}

	if (config->initialized) {
		VTI("Config already initialized");
		return 0;
	}

	/*
	 * config data are located in child node called
	 * 'sensorx', x is the sensor ID got from touch
	 * device.
	 */
	snprintf(of_node_name, sizeof(of_node_name),
			"sensor%u", sensor_id);
	node = of_get_child_by_name(node, of_node_name);
	if (!node) {
		VTE("Child property[%s] not found",
				of_node_name);
		return -EINVAL;
	}

	prop = of_find_property(node, cfg_type, &len);
	if (!prop || !prop->value || len == 0
			|| len > TS_CFG_MAX_LEN || len % 2 != 1) {
		VTE("Invalid cfg type%s, size:%u", cfg_type, len);
		return -EINVAL;
	}

	config->length = len;

	mutex_init(&config->lock);
	mutex_lock(&config->lock);

	memcpy(config->data, prop->value, len);

	/* modify max-x max-y resolution, little-endian */
	config->data[CFG_XMAX_OFFSET] = (u8)ts_bdata->panel_max_x;
	config->data[CFG_XMAX_OFFSET + 1] = (u8)(ts_bdata->panel_max_x >> 8);
	config->data[CFG_YMAX_OFFSET] = (u8)ts_bdata->panel_max_y;
	config->data[CFG_YMAX_OFFSET + 1] = (u8)(ts_bdata->panel_max_y >> 8);

	/*
	 * checksum: u16 little-endian format
	 * the last byte of config is the config update flag
	 */
	checksum = checksum_le16(config->data, len - 3);
	checksum = 0 - checksum;
	config->data[len - 3] = (u8)checksum;
	config->data[len - 2] = (u8)(checksum >> 8 & 0xff);
	config->data[len - 1] = 0x01;

	strlcpy(config->name, cfg_type, sizeof(config->name));
	config->reg_base = TS_REG_CFG_BASE;
	config->delay = 0;
	config->initialized = true;
	mutex_unlock(&config->lock);

	VTI("Config name:%s,ver:%02xh,size:%d,checksum:%04xh",
			config->name, config->data[0],
			config->length, checksum);
	return 0;
}
#endif


/**
 * goodix_parse_customize_params - parse sensor independent params
 * @dev: pointer to device data
 * @board_data: board data
 * @sensor_id: sensor ID
 * return: 0 - read ok, < 0 - i2c transter error
*/
static int goodix_parse_customize_params(struct goodix_ts_device *dev,
				struct goodix_ts_board_data *board_data,
				unsigned int sensor_id)
{
	struct device_node *node = dev->dev->of_node;
	char of_node_name[24];
	int r;

	if (sensor_id > TS_MAX_SENSORID || node == NULL) {
		VTE("Invalid sensor id");
		return -EINVAL;
	}

	/* parse sensor independent parameters */
	snprintf(of_node_name, sizeof(of_node_name),
			"sensor%u", sensor_id);
	node = of_find_node_by_name(dev->dev->of_node, of_node_name);
	if (!node) {
		VTE("Child property[%s] not found", of_node_name);
		return -EINVAL;
	}

	/* sensor independent resolutions */
	r = goodix_parse_dt_resolution(node, board_data);
	return r;
}
#endif

#if 0 /*def CONFIG_ACPI*/
static int goodix_parse_acpi(struct acpi_device *dev,
		struct goodix_ts_board_data *bdata)
{
	return 0;
}

static int goodix_parse_acpi_cfg(struct acpi_device *dev,
		char *cfg_type, struct goodix_ts_config *config,
		unsigned int sensor_id)
{
	return 0;
}
#endif
/**
 * goodix_spi_read- read device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - spi transter error
 */
#if 1
int goodix_spi_read_trans(struct goodix_ts_device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = dev->spi_dev;
	u8 *rx_buf = dev->rx_buff;
	u8 *tx_buf = dev->tx_buff;
	u8 start_cmd_buf[3];
	struct spi_transfer xfers[2];
	struct spi_message spi_msg;
	int ret = 0;

	mutex_lock(&dev->spi_mutex);
	spi_message_init(&spi_msg);
	memset(xfers, 0, sizeof(xfers));
	if(addr > 0)
		addr -= 1;
	start_cmd_buf[0] = SPI_FLAG_WR; //0xF0 start write flag
	start_cmd_buf[1] = (addr >> 8) & MASK_8BIT;
	start_cmd_buf[2] = addr & MASK_8BIT;

	xfers[0].tx_buf = start_cmd_buf;
	xfers[0].len = 3;
	xfers[0].cs_change = 1;
	spi_message_add_tail(&xfers[0], &spi_msg);

	tx_buf[0] = SPI_FLAG_RD; //0xF1 start read flag
	xfers[1].tx_buf = tx_buf;
	xfers[1].rx_buf = rx_buf;
	xfers[1].len = len + 2;
	#if defined(CONFIG_ARCH_QCOM)
	xfers[1].cs_change = 1;
	#else
	xfers[1].cs_change = 0;
	#endif
	spi_message_add_tail(&xfers[1], &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		VTE("Spi transfer error:%d\n",ret);
		goto exit;
	}
	memcpy(data, &rx_buf[2], len);
    //VTI("goodix SPI RXBUF:%02x,%02x,%02x",rx_buf[0],rx_buf[1],rx_buf[2]);
exit:
	mutex_unlock(&dev->spi_mutex);
	return ret;
}
#else
int goodix_spi_read_trans(struct goodix_ts_device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = dev->spi_dev;
	u8 *rx_buf = dev->rx_buff;
	u8 start_cmd_buf[4];
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	mutex_lock(&dev->spi_mutex);
	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));
	memset(rx_buf, 0, 1024);

	start_cmd_buf[0] = SPI_FLAG_WR; //0xF0 start write flag
	start_cmd_buf[1] = (addr >> 8) & MASK_8BIT;
	start_cmd_buf[2] = addr & MASK_8BIT;
	start_cmd_buf[3] = SPI_FLAG_RD;
	xfers.tx_buf = start_cmd_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = 4 + len;
	xfers.cs_change = 1;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		VTE("Spi transfer error:%d\n",ret);
		goto exit;
	}
	VTI("goodix SPI RXBUF:%02x,%02x",rx_buf[0],rx_buf[1]);
	memcpy(data, &rx_buf[1], len);
    
exit:
	mutex_unlock(&dev->spi_mutex);
	return ret;
}
#endif
/**
 * goodix_spi_write- write device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - spi transter error.
 */
int goodix_spi_write_trans(struct goodix_ts_device *dev, unsigned int addr,
		unsigned char *data, unsigned int len)
{
	struct spi_device *spi = dev->spi_dev;
	u8 *tx_buf = dev->tx_buff;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
#if 0
	tx_buf = kzalloc(len + 3, GFP_KERNEL);
	if (!tx_buf) {
		VTE("alloc tx_buf failed, size:%d\n", len + 3);
		return -ENOMEM;
	}
#endif
	mutex_lock(&dev->spi_mutex);
	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	tx_buf[0] = SPI_FLAG_WR; //0xF1 start read flag
	tx_buf[1] = (addr >> 8) & MASK_8BIT;
	tx_buf[2] = addr & MASK_8BIT;
	memcpy(&tx_buf[3], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = len + 3;
	#if defined(CONFIG_ARCH_QCOM)
	xfers.cs_change = 1;
	#else
	xfers.cs_change = 0;
	#endif
	
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		VTE("Spi transfer error:%d\n", ret);
	}
	mutex_unlock(&dev->spi_mutex);

	return ret;
}
#define IDLE_WAKE_FLAG   0xF0
int goodix_spi_active(struct goodix_ts_device *dev)
{
	struct spi_device *spi = dev->spi_dev;
	u8 wake_flag = IDLE_WAKE_FLAG;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	mutex_lock(&dev->spi_mutex);
	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	xfers.tx_buf = &wake_flag;
	xfers.len = 1;
	#if defined(CONFIG_ARCH_QCOM)
	xfers.cs_change = 1;
	#else
	xfers.cs_change = 0;
	#endif
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		VTE("Spi transfer error:%d\n", ret);
	}
	mutex_unlock(&dev->spi_mutex);

	udelay(900);
	return ret;
}

static int goodix_set_spi_speed(struct goodix_ts_device *dev, u32 speed)
{
	struct spi_device *spi = dev->spi_dev;
	int ret = 0;
	if(1){
		spi->max_speed_hz = speed;
		return ret;
	
	}else{
		return 0;
	}
}

/**
 * goodix_set_doze_mode - disable or enable doze mode
 * @dev: pointer to device data
 * @enable: true/flase
 * return: 0 - ok; < 0 - error.
 * This func must be used in pairs, when you disable doze
 * mode, then you must enable it again.
 * Between set_doze_false and set_doze_true, do not reset
 * IC!
 */
static int goodix_set_spi_doze_mode(struct goodix_ts_device *dev, int enable)
{
	static DEFINE_MUTEX(doze_mode_lock);
	static int doze_mode_set_count;
	int result = -EINVAL;
	int i = 0, j = 0;
	u8 w_data, r_data;
	//if (dev->ic_type != IC_TYPE_NORMANDY_SPI)
		//return 0;
	if(dev->int_state){
		VTD("in int state skip doze set");
		return 0;
		}
	mutex_lock(&doze_mode_lock);
	
	if (enable) {
		if (doze_mode_set_count != 0)
			doze_mode_set_count--;

		/*when count equal 0, allow ic enter doze mode*/
		if (doze_mode_set_count == 0) {
			w_data = TS_DOZE_ENABLE_DATA;
			for (i = 0; i < TS_DOZE_ENABLE_RETRY_TIMES; i++) {
				result = goodix_spi_write_trans(dev,
						TS_REG_DOZE_CTRL, &w_data, 1);
				if (!result) {
					result = 0;
					goto exit;
				}
				usleep_range(1000, 1100);
			}
			if (i >= TS_DOZE_ENABLE_RETRY_TIMES)
				VTE("spi doze mode enable failed");
		} else {
			/*ts_info("doze count not euqal 0,
			 * so skip doze mode enable");
			 */
			result = 0;
			goto exit;
		}
	} else {
		doze_mode_set_count++;

		if (doze_mode_set_count == 1) {
			goodix_set_spi_speed(dev, 3500);
			goodix_spi_active(dev);

			w_data = TS_DOZE_DISABLE_DATA;
			goodix_spi_write_trans(dev, TS_REG_DOZE_CTRL,
					       &w_data, 1);
			goodix_set_spi_speed(dev, dev->board_data->spi_max_speed);
			usleep_range(11000, 12000);
			for (i = 0; i < TS_DOZE_DISABLE_RETRY_TIMES; i++) {
				goodix_spi_read_trans(dev,
						TS_REG_DOZE_CTRL, &r_data, 1);
				if (TS_DOZE_CLOSE_OK_DATA == r_data) {
					result = 0;
					goto exit;
				} else if (0xAA != r_data) {
					VTE("read back data illegal, 0x%X, retry_times %d",
							r_data, j);
					for (j = 1; j < TS_DOZE_DISABLE_RETRY_TIMES; j++) {
						usleep_range(11000, 12000);
						goodix_spi_read_trans(dev,
								TS_REG_DOZE_CTRL, &r_data, 1);
						if (TS_DOZE_CLOSE_OK_DATA == r_data) {
							result = 0;
							goto exit;
						} else {
							VTE("read back data illegal, 0x%X, retry_times %d",
									r_data, j);
						}
					}

					if (j == TS_DOZE_DISABLE_RETRY_TIMES) {
						goodix_set_spi_speed(dev, 3500);
						goodix_spi_active(dev);
						w_data = TS_DOZE_DISABLE_DATA;
						goodix_spi_write_trans(dev,
							TS_REG_DOZE_CTRL, &w_data, 1);
						goodix_set_spi_speed(dev, dev->board_data->spi_max_speed);
					}
				}
				usleep_range(11000, 12000);
			}
			VTE("doze mode disable FAILED");
		} else {
			result = 0;
			goto exit;
		}
	}

exit:
	mutex_unlock(&doze_mode_lock);
	return result;
}


/**
 * goodix_i2c_write_V2 - write device register through i2c bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - i2c transter error.
*/
int goodix_spi_write(struct goodix_ts_device *dev, unsigned int reg,
		unsigned char *data, unsigned int len)
{
	int r;
	
	if (goodix_set_spi_doze_mode(dev, false) != 0)
		VTE("gtx8 spi write:0x%04x ERROR, disable doze mode FAILED",
				reg);
	
	r = goodix_spi_write_trans(dev, reg, data, len);
	
	if (goodix_set_spi_doze_mode(dev, true) != 0)
		VTE("gtx8 spi write:0x%04x ERROR, enable doze mode FAILED",
				reg);
	return r;
}


/**
 * goodix_i2c_read_V2 - read device register through i2c bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - i2c transter error
*/
int goodix_spi_read(struct goodix_ts_device *dev, unsigned int reg,
	unsigned char *data, unsigned int len)
{
	int r;
	if (goodix_set_spi_doze_mode(dev, false) != 0)
		VTE("gtx8 spi read:0x%04x ERROR, disable doze mode FAILED",
				reg);
	
	r = goodix_spi_read_trans(dev, reg, data, len);
	
	if (goodix_set_spi_doze_mode(dev, true) != 0)
		VTE("gtx8 spi read:0x%04x ERROR, enable doze mode FAILED",
				reg);
	return r;
}


static void goodix_cmds_init(struct goodix_ts_cmd *ts_cmd,
					     u8 cmds, u8 cmd_data, u32 reg_addr)
{
	if (reg_addr) {
		ts_cmd->cmd_reg = reg_addr;
		ts_cmd->length = 3;
		ts_cmd->cmds[0] = cmds;
		ts_cmd->cmds[1] = cmd_data;
		ts_cmd->cmds[2] = 0 - cmds - cmd_data;
		ts_cmd->initialized = true;
		} else {
			ts_cmd->initialized = false;
	}
}

/**
 * goodix_send_command_V2 - seng cmd to firmware
 *
 * @dev: pointer to device
 * @cmd: pointer to command struct which cotain command data
 * Returns 0 - succeed,<0 - failed
 */
int goodix_send_command_spi(struct goodix_ts_device *dev,
		struct goodix_ts_cmd *cmd)
{
	int ret;

	if (!cmd || !cmd->initialized)
		return -EINVAL;
	mutex_lock(&dev->i2c_reset_mutex);
	
	if (0x08 == cmd->cmds[0] || 0x19 == cmd->cmds[0] || 0x1A == cmd->cmds[0])
		usleep_range(15000, 16000);
	VTI("send cmd ++");
	ret = goodix_spi_write(dev, cmd->cmd_reg, cmd->cmds,
			cmd->length);
	
	if (0x08 == cmd->cmds[0] || 0x19 == cmd->cmds[0] || 0x1A == cmd->cmds[0])
	usleep_range(15000, 16000);
	
	mutex_unlock(&dev->i2c_reset_mutex);
	return ret;
}


static int goodix_read_version(struct goodix_ts_device *dev,
		struct goodix_ts_version *version)
{
	u8 buffer[12];
	u8 temp_buf[256], checksum;
	int r;
	u8 pid_read_len = dev->reg.pid_len;
	u8 vid_read_len = dev->reg.vid_len;
	u8 sensor_id_mask = dev->reg.sensor_id_mask;
#define IS_CHAR(c)	(((c) >= 'A' && (c) <= 'Z')\
		|| ((c) >= 'a' && (c) <= 'z')\
		|| ((c) >= '0' && (c) <= '9'))

	if (!version) {
		VTE("pointer of version is NULL");
		return -EINVAL;
	}

	version->valid = false;

	/*check reg info valid*/
	if (!dev->reg.pid || !dev->reg.sensor_id || !dev->reg.vid) {
		VTE("reg is NULL, pid:0x%04x, vid:0x%04x, sensor_id:0x%04x",
				dev->reg.pid, dev->reg.vid, dev->reg.sensor_id);
		return -EINVAL;
	}
	if (!pid_read_len || pid_read_len > 8 || !vid_read_len || vid_read_len > 8) {
		VTE("pid vid read len ERROR, pid_read_len:%d, vid_read_len:%d",
				pid_read_len, vid_read_len);
		return -EINVAL;
	}

	/*disable doze mode, just valid for normandy
	 * this func must be used in pairs*/
	goodix_set_spi_doze_mode(dev, false);

	/*check checksum*/
	if (dev->reg.version_base && dev->reg.version_len < 256) {
		r = goodix_spi_read_trans(dev, dev->reg.version_base,
				temp_buf, dev->reg.version_len);

		if (r < 0) {
			VTE("Read version base failed, reg:0x%02x, len:%d",
					dev->reg.version_base, dev->reg.version_len);
			if (version)
				version->valid = false;
			goto exit;
		}

		checksum = checksum_u8(temp_buf, dev->reg.version_len);
		if (checksum) {
			VTE("checksum error:0x%02x, base:0x%02x, len:%d",
					checksum, dev->reg.version_base, dev->reg.version_len);
			VTE("%*ph", (int)(dev->reg.version_len / 2), temp_buf);
			VTE("%*ph", (int)(dev->reg.version_len - dev->reg.version_len / 2),
					&temp_buf[dev->reg.version_len / 2]);
			vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);

			if (version)
				version->valid = false;
			r = -EINVAL;
			goto exit;
		}
	}

	/*read pid*/
	memset(buffer, 0, sizeof(buffer));
	memset(version->pid, 0, sizeof(version->pid));
	r = goodix_spi_read_trans(dev, dev->reg.pid, buffer, pid_read_len);
	if (r < 0) {
		VTE("Read pid failed");
		if (version)
			version->valid = false;
		goto exit;
	}
	memcpy(version->pid, buffer, pid_read_len);


	/*read vid*/
	memset(buffer, 0, sizeof(buffer));
	memset(version->vid, 0, sizeof(version->vid));
	r = goodix_spi_read_trans(dev, dev->reg.vid, buffer, vid_read_len);
	if (r < 0) {
		VTE("Read vid failed");
		if (version)
			version->valid = false;
		goto exit;
	}
	memcpy(version->vid, buffer, vid_read_len);

	/*read sensor_id*/
	memset(buffer, 0, sizeof(buffer));
	r = goodix_spi_read_trans(dev, dev->reg.sensor_id, buffer, 1);
	if (r < 0) {
		VTE("Read sensor_id failed");
		if (version)
			version->valid = false;
		goto exit;
	}
	if (sensor_id_mask != 0) {
		version->sensor_id = buffer[0] & sensor_id_mask;
		VTI("sensor_id_mask:0x%02x, sensor_id:0x%02x",
				sensor_id_mask, version->sensor_id);
	} else {
		version->sensor_id = buffer[0];
	}

	version->valid = true;

	VTI("PID:%s,SensorID:%d, VID:%*ph",
						version->pid,
						version->sensor_id,
						(int)sizeof(version->vid), version->vid);
exit:
	/*enable doze mode, just valid for normandy
	 * this func must be used in pairs*/
	goodix_set_spi_doze_mode(dev, true);

	return r;
}


#if 0
static int goodix_read_version(struct goodix_ts_device *dev,
		struct goodix_ts_version *version)
{
	u8 buffer[12];
	int r;
#define IS_CHAR(c)	(((c) >= 'A' && (c) <= 'Z')\
		|| ((c) >= 'a' && (c) <= 'z')\
		|| ((c) >= '0' && (c) <= '9'))

	r = goodix_i2c_read_V2(dev, dev->reg.pid,/*TS_REG_VERSION*/
			buffer, sizeof(buffer));
	if (r < 0) {
		VTE("Read chip version failed");
		if (version)
			version->valid = false;
		return r;
	}

	/* if checksum is right and first 4 bytes are
	 * not invalid value */
	if (checksum_u8(buffer, sizeof(buffer)) == 0 &&
			IS_CHAR(buffer[0]) && IS_CHAR(buffer[1]) &&
			IS_CHAR(buffer[2]) && IS_CHAR(buffer[3])) {
		if (version) {
			memcpy(&version->pid[0], buffer, 4);
			version->pid[4] = '\0';
			version->cid = buffer[4];
			/* vid = main version + minor version */
			version->vid = (buffer[5] << 8) + buffer[6];
			version->sensor_id = buffer[10] & 0x0F;
			version->valid = true;

			if (version->cid)
				VTI("PID:%s,CID: %c,VID:%04x,SensorID:%u",
						version->pid, version->cid + 'A' - 1,
						version->vid, version->sensor_id);
			else
				VTI("PID:%s,VID:%04x,SensorID:%u",
						version->pid, version->vid,
						version->sensor_id);
		}
	} else {
		VTE("Checksum error:%*ph", (int)sizeof(buffer), buffer);
		/* mark this version is invalid */
		if (version)
			version->valid = false;
		r = -EINVAL;
	}

	return r;
}
#endif

static int goodix_send_small_config(struct goodix_ts_device *dev,
		struct goodix_ts_config *config)
{
	int r = 0;
	int try_times = 0;
	u8 buf = 0;
	u16 command_reg = dev->reg.command;
	u16 cfg_reg = dev->reg.cfg_addr;
	struct goodix_ts_cmd ts_cmd;

	/*1. Inquire command_reg until it's free*/
	for (try_times = 0; try_times < 10; try_times++) {
		if (!goodix_spi_read(dev, command_reg, &buf, 1) && buf == 0xff)
			break;
		msleep(100);
	}
	if (try_times >= 10) {
		VTE("Send small cfg FAILED, before send, reg:0x%04x is not 0xff", command_reg);
		r = -EINVAL;
		goto exit;
	}

	/*2. write cfg data*/
	if (goodix_spi_write(dev, cfg_reg, config->data, config->length)) {
		VTE("Send small cfg FAILED, write cfg to fw ERROR");
		r = -EINVAL;
		goto exit;
	}

	/*3. send 0x81 command*/
	goodix_cmds_init(&ts_cmd, COMMAND_SEND_SMALL_CFG, 0, dev->reg.command);
	if (goodix_send_command_spi(dev, &ts_cmd)) {
		VTE("Send large cfg FAILED, send COMMAND_SEND_SMALL_CFG ERROR");
		r = -EINVAL;
		goto exit;
	}

	r = 0;
	VTI("send small cfg SUCCESS");

exit:
	return r;
}

static int goodix_send_large_config(struct goodix_ts_device *dev,
		struct goodix_ts_config *config)
{
	int r = 0;
	int try_times = 0;
	u8 buf = 0;
	u16 command_reg = dev->reg.command;
	u16 cfg_reg = dev->reg.cfg_addr;
	struct goodix_ts_cmd ts_cmd;

	/*1. Inquire command_reg until it's free*/
	for (try_times = 0; try_times < 10; try_times++) {
		if (!goodix_spi_read(dev, command_reg, &buf, 1) && buf == 0xff)
			break;
		usleep_range(10000, 11000);
	}
	if (try_times >= 10) {
		VTE("Send large cfg FAILED, before send, reg:0x%04x is not 0xff", command_reg);
		r = -EINVAL;
		goto exit;
	}

	/*2. send "start write cfg" command*/
	goodix_cmds_init(&ts_cmd, COMMAND_START_SEND_CFG, 0, dev->reg.command);
	if (goodix_send_command_spi(dev, &ts_cmd)) {
		VTE("Send large cfg FAILED, send COMMAND_START_SEND_CFG ERROR");
		r = -EINVAL;
		goto exit;
	}

	/*3. wait ic set command_reg to 0x82*/
	for (try_times = 0; try_times < 10; try_times++) {
		if (!goodix_spi_read(dev, command_reg, &buf, 1) && buf == 0x82)
			break;
		usleep_range(10000, 11000);
	}
	if (try_times >= 10) {
		VTE("Send large cfg FAILED, reg:0x%04x is not 0x82", command_reg);
		r = -EINVAL;
		goto exit;
	}

	/*4. write cfg*/
	if (goodix_spi_write(dev, cfg_reg, config->data, config->length)) {
		VTE("Send large cfg FAILED, write cfg to fw ERROR");
		r = -EINVAL;
		goto exit;
	}

	/*5. send "end send cfg" command*/
	goodix_cmds_init(&ts_cmd, COMMAND_END_SEND_CFG, 0, dev->reg.command);
	if (goodix_send_command_spi(dev, &ts_cmd)) {
		VTE("Send large cfg FAILED, send COMMAND_END_SEND_CFG ERROR");
		r = -EINVAL;
		goto exit;
	}

	/*6. wait ic set command_reg to 0xff*/
	for (try_times = 0; try_times < 10; try_times++) {
		if (!goodix_spi_read(dev, command_reg, &buf, 1) && buf == 0xff)
			break;
		usleep_range(10000, 11000);
	}
	if (try_times >= 10) {
		VTE("Send large cfg FAILED, after send, reg:0x%04x is not 0xff", command_reg);
		r = -EINVAL;
		goto exit;
	}

	VTI("Send large cfg SUCCESS");
	r = 0;

exit:
	return r;
}

#if FORCE_SEND_CFG
static int goodix_cfg_tmp_checksum_fix(struct goodix_ts_device *dev, u8 *cfg, u32 length)
{
	int ret;
	u8 checksum;
	int i;

	if (!cfg || length < 4) {
		VTE("cfg is INVALID, len:%d", length);
		ret = -EINVAL;
		goto exit;
	}

	if (dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		/*set cfg version to 0xff and fix the checksum*/
		cfg[0] = 0xff;
		checksum = 0;
		for (i = 0; i < 3; i++)
			checksum -= cfg[i];
		cfg[3] = checksum;
	} else {
		VTE("cfg check FAILED, is not Normandy ic_type");
		ret = -EINVAL;
		goto exit;
	}

	VTI("fix the tmp_cfg SUCCESS");
	ret = 0;

exit:
	return ret;
}
#endif

static int goodix_check_cfg_valid(struct goodix_ts_device *dev, u8 *cfg, u32 length)
{
	int ret;
	u8 bag_num;
	u8 checksum;
	int i, j;
	int bag_start = 0;
	int bag_end = 0;
	if (!cfg || length < 4) {
		VTE("cfg is INVALID, len:%d", length);
		ret = -EINVAL;
		goto exit;
	}

	if (dev->ic_type == IC_TYPE_NANJING) {
		/*check configuration head checksum*/
		checksum = 0;
		for (i = 0; i < 3; i++)
			checksum += cfg[i];

		if (checksum != 0) {
			VTE("cfg head checksum ERROR, ic type:nanjing, checksum:0x%02x",
					checksum);
			ret = -EINVAL;
			goto exit;
		}
		bag_num = cfg[1];
		bag_start = 3;
	} else if (dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		checksum = 0;
		for (i = 0; i < 4; i++)
			checksum += cfg[i];
		if (checksum != 0) {
			VTE("cfg head checksum ERROR, ic type:normandy, checksum:0x%02x",
					checksum);
			ret = -EINVAL;
			goto exit;
		}
		bag_num = cfg[2];
		bag_start = 4;
	} else {
		VTE("cfg check FAILED, unkonw ic_type");
		ret = -EINVAL;
		goto exit;
	}

	VTI("cfg bag_num:%d, cfg length:%d", bag_num, length);

	/*check each bag's checksum*/
	for (j = 0; j < bag_num; j++) {
		if (bag_start >= length - 1) {
			VTE("ERROR, overflow!!bag_start:%d, cfg_len:%d", bag_start, length);
			ret = -EINVAL;
			goto exit;
		}

		bag_end = bag_start + cfg[bag_start + 1] + 3;
		if ((j == 0) && (dev->ic_type == IC_TYPE_NANJING))
			/*the first bag of nanjing cfg is different!*/
			bag_end = 336;

		checksum = 0;
		if (bag_end > length) {
			VTE("ERROR, overflow!!bag:%d, bag_start:%d,  bag_end:%d, cfg length:%d",
					j, bag_start, bag_end, length);
			ret = -EINVAL;
			goto exit;
		}
		for (i = bag_start; i < bag_end; i++)
			checksum += cfg[i];
		if (checksum != 0) {
			VTE("cfg INVALID, bag:%d checksum ERROR:0x%02x", j, checksum);
			ret = -EINVAL;
			goto exit;
		}
		bag_start = bag_end;
	}

	ret = 0;
	VTI("configuration check SUCCESS");

exit:
	return ret;
}
static int goodix_send_config(struct goodix_ts_device *dev,
		struct goodix_ts_config *config)
{
	int r = 0;
#if FORCE_SEND_CFG		
	struct goodix_ts_config *cfg_tmp = NULL;  
#endif

	/*check reg valid*/
	/*	if (!dev->reg.cfg_addr) {
			VTE("cfg register is NULL");
			return -EINVAL;
		}*/
	if (!config) {
		VTE("Null config data");
		return -EINVAL;
	}

	/*check configuration valid*/
	r = goodix_check_cfg_valid(dev, config->data, config->length);
	if (r != 0) {
		VTE("cfg check FAILED");
		return -EINVAL;
	}

#if FORCE_SEND_CFG
	/*set cfg_tmp version to 0xff and fix the cfg_tmp checksum*/
	cfg_tmp = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (cfg_tmp == NULL) {
		VTE("Memory allco err");		
		return -ENOMEM;
	}
	memcpy(cfg_tmp, config, sizeof(struct goodix_ts_config));
	r = goodix_cfg_tmp_checksum_fix(dev, cfg_tmp->data, cfg_tmp->length);
	if (r != 0) {
		VTE("cfg_tmp checksum fixed FAILED");
		r = -EINVAL;
		goto exit;
	}	
#endif

	VTI("ver:%02xh,size:%d",
		config->data[0],
		config->length);

	mutex_lock(&config->lock);

	if (dev->ic_type == IC_TYPE_NANJING)
		r = goodix_send_large_config(dev, config);
	else if (dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		/*close doze mode*/
		goodix_set_spi_doze_mode(dev, false);
		
		if (config->length > 32) {
#if FORCE_SEND_CFG				
			r = goodix_send_large_config(dev, cfg_tmp);
			msleep(100);
#endif			
			r = goodix_send_large_config(dev, config);
		} else {
			r = goodix_send_small_config(dev, config);
		}
		/*open doze mode*/
		msleep(100);
		goodix_set_spi_doze_mode(dev, true);
	}

	if (r != 0)
		VTE("send_cfg FAILED, ic_type:%d, cfg_len:%d",
				dev->ic_type, config->length);

	mutex_unlock(&config->lock);
#if FORCE_SEND_CFG	
exit:

	if (cfg_tmp) {
		kfree(cfg_tmp);
		cfg_tmp = NULL;
	}
#endif	
	return r;
}



#if 0
/**
 * goodix_send_config - send config data to device.
 * @dev: pointer to device
 * @config: pointer to config data struct to be send
 * @return: 0 - succeed, < 0 - failed
 */
static int goodix_send_config(struct goodix_ts_device *dev,
		struct goodix_ts_config *config)
{
	int r = 0;

	if (!config || !config->data) {
		VTE("Null config data");
		return -EINVAL;
	}

	VTI("Send %s,ver:%02xh,size:%d",
		config->name, config->data[0],
		config->length);

	mutex_lock(&config->lock);
	r = goodix_i2c_write_V2(dev, config->reg_base,
			config->data, config->length);
	if (r)
		goto exit;

	/* make sure the firmware accept the config data*/
	if (config->delay)
		msleep(config->delay);
exit:
	mutex_unlock(&config->lock);
	return r;
}
#endif



/* success return config length else return -1 */
static int _goodix_do_read_config(struct goodix_ts_device *dev, u32 base_addr, u8 *buf)
{
	int sub_bags = 0;
	int offset = 0;
	int subbag_len;
	u8 checksum;
	int i;
	int ret;

	/*disable doze mode*/
	if (dev->ic_type == IC_TYPE_NORMANDY_SPI)
		goodix_set_spi_doze_mode(dev, false);

	ret = goodix_spi_read(dev, base_addr, buf, 4);
	if (ret)
		goto err_out;

	if (dev->ic_type == IC_TYPE_NANJING) {
		offset = 3;
		sub_bags = buf[1];
		checksum = checksum_u8(buf, 3);
	} else {
		offset = 4;
		sub_bags = buf[2];
		checksum = checksum_u8(buf, 4);
	}
	if (checksum) {
		VTE("Config head checksum err:0x%x,data:%*ph",
				checksum, 4, buf);
		ret = -EINVAL;
		goto err_out;
	}

	VTI("config_version:%u, vub_bags:%u",
			buf[0], sub_bags);
	for (i = 0; i < sub_bags; i++) {
		/* read sub head [0]: sub bag num, [1]: sub bag length */
		ret = goodix_spi_read(dev, base_addr + offset, buf + offset, 2);
		if (ret)
			goto err_out;

		/* read sub bag data */
		if (dev->ic_type == IC_TYPE_NANJING && i == 0)
			subbag_len = buf[offset + 1] + 256;
		else
			subbag_len = buf[offset + 1];

		VTI("sub bag num:%u,sub bag length:%u", buf[offset], subbag_len);
		ret = goodix_spi_read(dev, base_addr + offset + 2,
							buf + offset + 2,
							subbag_len + 1);
		if (ret)
			goto err_out;
		checksum = checksum_u8(buf + offset, subbag_len + 3);
		if (checksum) {
			VTE("sub bag checksum err:0x%x", checksum);
			ret = -EINVAL;
			goto err_out;
		}
		offset += subbag_len + 3;
		VTD("sub bag %d, data:%*ph", buf[offset], buf[offset + 1] + 3,
				 buf + offset);
	}
	ret = offset;

err_out:
	/*enable doze mode*/
	if (dev->ic_type == IC_TYPE_NORMANDY_SPI)
		goodix_set_spi_doze_mode(dev, true);

	return ret;
}

/* success return config_len, <= 0 failed */
static int goodix_read_config(struct goodix_ts_device *dev,
					      u8 *config_data, u32 config_len)
{
	struct goodix_ts_cmd ts_cmd;
	u8 cmd_flag;
	u32 cmd_reg = dev->reg.command;
	int r = 0;
	int i;

	if (!config_data || config_len > TS_CFG_MAX_LEN) {
		VTE("Illegal params");
		return -EINVAL;
	}
	if (!dev->reg.command) {
		VTE("command register ERROR:0x%04x", dev->reg.command);
		return -EINVAL;
	}

	/*close doze mode*/
	if (dev->ic_type == IC_TYPE_NORMANDY_SPI)
		goodix_set_spi_doze_mode(dev, false);

	/* wait for IC in IDLE state */
	for (i = 0; i < 20; i++) {
		cmd_flag = 0;
		r = goodix_spi_read(dev, cmd_reg, &cmd_flag, 1);
		if (r < 0 || cmd_flag == 0xFF)
			break;
		usleep_range(5000, 5200);
	}
	if (cmd_flag != 0xFF) {
		VTE("Wait for IC ready IDEL state timeout:addr 0x%x\n",
		       cmd_reg);
		r = -EINVAL;
		goto exit;
	}
	/* 0x86 read config command */
	goodix_cmds_init(&ts_cmd, 0x86, 0, cmd_reg);
	r = goodix_send_command_spi(dev, &ts_cmd);
	if (r) {
		VTE("Failed send read config command");
		goto exit;
	}
	/* wait for config data ready */
	for (i = 0; i < 20; i++) {
		cmd_flag = 0;
		r = goodix_spi_read(dev, cmd_reg, &cmd_flag, 1);
		if (r < 0 || cmd_flag == 0x85)
			break;
		usleep_range(5000, 5200);
	}
	if (cmd_flag != 0x85) {
		VTE("Wait for config data ready timeout");
		r = -EINVAL;
		goto exit;
	}
	if (config_len) {
		r = goodix_spi_read(dev, cmd_reg + 16, config_data, config_len);
		if (r)
			VTE("Failed read config data");
		else
			r = config_len;
	} else {
		r = _goodix_do_read_config(dev, cmd_reg + 16, config_data);
		if (r < 0)
			VTE("Failed read config data");
	}
	if (r > 0)
		VTI("success read config, len:%d", r);
	/* clear command */
	goodix_cmds_init(&ts_cmd, 0xFF, 0, cmd_reg);
	goodix_send_command_spi(dev, &ts_cmd);
	
exit:
	/*open doze mode*/
	if (dev->ic_type == IC_TYPE_NORMANDY_SPI)
		goodix_set_spi_doze_mode(dev, true);


	return r;
}

/**
 * goodix_hw_init - hardware initialize
 *   Called by touch core module when bootup
 * @ts_dev: pointer to touch device
 * return: 0 - no error, <0 error
*/
static int goodix_hw_init(struct goodix_ts_device *ts_dev)
{
	int r;

	BUG_ON(!ts_dev);

	/* goodix_hw_init may be called many times */
	if (!ts_dev->normal_cfg) {
		ts_dev->normal_cfg = devm_kzalloc(ts_dev->dev,
				sizeof(*ts_dev->normal_cfg), GFP_KERNEL);
		if (!ts_dev->normal_cfg) {
			VTE("Failed to alloc memory for normal cfg");
			return -ENOMEM;
		}
		mutex_init(&ts_dev->normal_cfg->lock);
	}
	if (!ts_dev->highsense_cfg) {
		ts_dev->highsense_cfg = devm_kzalloc(ts_dev->dev,
				sizeof(*ts_dev->highsense_cfg), GFP_KERNEL);
		if (!ts_dev->highsense_cfg) {
			VTE("Failed to alloc memory for high sense cfg");
			return -ENOMEM;
		}
		mutex_init(&ts_dev->highsense_cfg->lock);
	}


	/* read chip version: PID/VID/sensor ID,etc.*/
	r = goodix_read_version(ts_dev, &ts_dev->chip_version);
	if (r < 0)
		return r;

	/* devicetree property like resolution(panel_max_xxx)
	 * may be different between sensors, here we try to parse
	 * parameters form sensor child node */
	r = goodix_parse_customize_params(ts_dev,
			ts_dev->board_data,
			ts_dev->chip_version.sensor_id);
	if (r < 0)
		VTI("Cann't find customized parameters");
#if 0
	/* lonzo debug*/
	ts_dev->chip_version.sensor_id = 0;

	/* parse normal-cfg from devicetree node */
	r = goodix_parse_dt_cfg(ts_dev, "normal-cfg",
			ts_dev->normal_cfg,
			ts_dev->chip_version.sensor_id);
	if (r < 0) {
		VTE("Failed to obtain normal-cfg");
		return r;
	}
#endif
	ts_dev->normal_cfg->delay = 500;
	/* send normal-cfg to firmware */
/*	if (!goodix_cfg_send_judge(ts_dev, ts_dev->normal_cfg))  //By Jarvis,Plz add in formal drivers*/
		r = goodix_send_config(ts_dev, ts_dev->normal_cfg);
	VTI("goodix_send_config end!!!!!!!!!");
	return r;
}

/**
 * goodix_hw_reset_V2 - reset device
 *
 * @dev: pointer to touch device
 * Returns 0 - succeed,<0 - failed
 */
int goodix_hw_reset_spi(struct goodix_ts_device *dev)
{
	u8 data[2] = {0x00};
	int r = 0;

	VTI("HW reset");

	if (dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		VTI("11 enter in goodix_hw_reset_V2");
		//no need to check state 
		if (1) {
			VTI("normandy reset");
			mutex_lock(&dev->i2c_reset_mutex);
			gpio_direction_output(dev->board_data->reset_gpio, 0);
			udelay(2000);
			gpio_direction_output(dev->board_data->reset_gpio, 1);
			msleep(60);
			mutex_unlock(&dev->i2c_reset_mutex);
		} 
		/*else {
			VTI("ear detect open, do not reset ic");
		}
		*/

	} 

	/*init static esd*/
	data[0] = 0xaa;

	/*init dynamic esd*/
	if (dev->reg.esd) {
		r = goodix_spi_write_trans(dev,
				dev->reg.esd,
				data, 1);
		if (r < 0)
			VTE("IC reset, init dynamic esd FAILED, i2c write ERROR");
	} else
		VTI("reg.esd is NULL, skip dynamic esd init");

	return 0;
}

/**
 * goodix_request_handler - handle firmware request
 *
 * @dev: pointer to touch device
 * @request_data: requset information
 * Returns 0 - succeed,<0 - failed
 */
static int goodix_request_handler(struct goodix_ts_device *dev,
		struct goodix_request_data *request_data)
{
	unsigned char buffer[1];
	int r;

	r = goodix_spi_read_trans(dev, dev->reg.fw_request, buffer, 1);/*TS_REG_REQUEST*/
	if (r < 0)
		return r;

	switch (buffer[0]) {
	case REQUEST_CONFIG:
		VTI("HW request config");
		goodix_send_config(dev, dev->normal_cfg);
		goto clear_requ;
		break;
	case REQUEST_BAKREF:
		VTI("HW request bakref");
		goto clear_requ;
		break;
	case REQUEST_RESET:
		VTI("HW requset reset");
		goto clear_requ;
		break;
	case REQUEST_MAINCLK:
		VTI("HW request mainclk");
		goto clear_requ;
		break;
	default:
		VTI("Unknown hw request:%d", buffer[0]);
		return 0;
	}

clear_requ:
	buffer[0] = 0x00;
	r = goodix_spi_write_trans(dev, dev->reg.fw_request, buffer, 1);/*TS_REG_REQUEST*/
	return r;
}

/*goodix_swap_coords - swap coord
 */

static void goodix_swap_coords(struct goodix_ts_device *dev,
		struct goodix_ts_coords *coords,
		int touch_num)
{
	int i, temp;
	struct goodix_ts_board_data *bdata = dev->board_data;
	u32 panel_max_x,panel_max_y;
	for (i = 0; i < touch_num; i++) {
		if (bdata->swap_axis) {
			temp = coords->x;
			coords->x = coords->y;
			coords->y = temp;
		}
		if (bdata->x2x){
			vts_property_get(dev->vtsdev, VTS_PROPERTY_DIMENTION_X, &panel_max_x);
			coords->x = panel_max_x - coords->x;
		}
		if (bdata->y2y){
			vts_property_get(dev->vtsdev, VTS_PROPERTY_DIMENTION_Y, &panel_max_y);
			coords->y = panel_max_y - coords->y;
		}
		coords++;
	}
}

extern int FPPointCnt_V2;
static void goodix_FPreport(struct goodix_ts_core *data, struct goodix_ts_device *dev,
		unsigned char buf)
{

	struct vts_device *vtsdev = data->vtsdev;
	if (0x88 == (buf & 0x88)) {
		if (FPPointCnt_V2 <= 1)
			FPPointCnt_V2++;
	} else { 
		if (FPPointCnt_V2 >= 1) {
			//vivoTsInputReport(VTS_GESTURE_EVENT, 254, 0, -1, -1);
			vts_report_event_up(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
		}
		FPPointCnt_V2 = 0;
	}

	if(1 == FPPointCnt_V2)
		//vivoTsInputReport(VTS_GESTURE_EVENT, 254, 1, -1, -1);
		vts_report_event_down(vtsdev, VTS_EVENT_GESTURE_FINGERPRINT_DETECT);
}


/**
 * goodix_event_handler - handle firmware event
 *
 * @dev: pointer to touch device
 * @ts_event: pointer to touch event structure
 * Returns 0 - succeed,<0 - failed
 */
#define CONFIG_VIRTUAL_PROX_GOODIX

static int goodix_touch_handler(struct goodix_ts_core *data, struct goodix_ts_device *dev,
		struct goodix_ts_event *ts_event,
		u8 *pre_buf, u32 pre_buf_len, ktime_t kt)
{
	struct goodix_touch_data *touch_data = &ts_event->event_data.touch_data;
	struct goodix_ts_coords *coords = &(touch_data->coords[0]);
	int max_touch_num = dev->board_data->panel_max_id;
	unsigned char buffer[4 + BYTES_PER_COORD * 15];

	unsigned char coord_sta;
	int touch_num = 0;
	int i;
	int r = 0;
	unsigned char chksum = 0;
	int large_status = 0;
#ifdef CONFIG_VIRTUAL_PROX_GOODIX
	int pick[3] = {0};
    int cur_state = 0;
    static int goodix_last_prox_state = 0;
#endif

	VTD("===enter===");

	if (!pre_buf || pre_buf_len != (4 + BYTES_PER_COORD)) {
		r = -EINVAL;
		return r;
	}

	/*copy data to buffer*/
	touch_data->custom_data_buf[0] = pre_buf[8];
	touch_data->custom_data_buf[1] = pre_buf[9];
	memcpy(buffer, pre_buf, pre_buf_len);

	/* buffer[1]: touch state */
	coord_sta = buffer[1];

	touch_num = coord_sta & 0x0F;
	#ifdef CONFIG_VIRTUAL_PROX_GOODIX
	cur_state = coord_sta & 0x40;
    #endif
	large_status = coord_sta & 0x20;
	if (0 != large_status) {
		VTI(" Palm Detect, in large status");
		data->large_press = true;
		vts_report_event_down(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
	} else {
		if (data->large_press) {
			VTI(" Palm release, out large status");
			data->large_press = false;
			vts_report_event_up(data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
		}
	}

	if (unlikely(touch_num > max_touch_num)) {
		touch_num = -EINVAL;
		goto exit_clean_sta;
	} else if (unlikely(touch_num > 1)) {
		r = goodix_spi_read_trans(dev,
				dev->reg.coor + 4 + BYTES_PER_COORD,/*TS_REG_COORDS_BASE*/
				&buffer[4 + BYTES_PER_COORD],
				(touch_num - 1) * BYTES_PER_COORD);
		if (unlikely(r < 0))
			goto exit_clean_sta;
	}
/*
	r = goodix_i2c_read_V2_trans_V2(dev,
				0x4108, &(touch_data->custom_data_buf[0]), 1);
	if (r != 0)
		VTE("read register 0x4108 fail");

	r = goodix_i2c_read_V2_trans_V2(dev,
				0x4109,&(touch_data->custom_data_buf[1]), 1);
	if (r != 0)
		VTE("read register 0x4109 fail");
*/

	/* touch_num * BYTES_PER_COORD + 1(touch event state)
	 * + 1(checksum) + 1(key value) */
	if (dev->ic_type == IC_TYPE_NANJING) {
		chksum = checksum_u8(&buffer[1],
				touch_num * BYTES_PER_COORD + 3);
	} else {
		chksum = checksum_u8(&buffer[0],
				touch_num * BYTES_PER_COORD + 4);
	}
	if (unlikely(chksum != 0)) {
		VTE("Checksum error:%X, ic_type:%d", chksum, dev->ic_type);
		vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
		r = -EINVAL;
		goto exit_clean_sta;
	}

	touch_data->have_key = false;/*clear variable*/
	touch_data->key_value = 0;/*clear variable*/
	touch_data->have_key = (coord_sta >> 4) & 0x01;
	if (touch_data->have_key) {
		touch_data->key_value = buffer[touch_num * BYTES_PER_COORD + 2];
		if (dev->board_data->pen_enable)
			touch_data->key_value = (touch_data->key_value & 0x0f) |
				((touch_data->key_value & 0xf0) >> (4 - dev->board_data->tp_key_num));
	}
	/*VTI("$$$$$$coord_sta:0x%02x, have_key:%d, key_value:0x%02x",
			coord_sta, touch_data->have_key, touch_data->key_value);*/

	/*for (i = 0; i < touch_num; i++) {
		VTI("data%d:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",
				i, buffer[i * 8 + 2], buffer[i * 8 + 3],
				buffer[i * 8 + 4], buffer[i * 8 + 5],
				buffer[i * 8 + 6], buffer[i * 8 + 7],
				buffer[i * 8 + 8], buffer[i * 8 + 9]);
	}*/

	/*add end*/

	/*remap trace id*/
	/*goodix_remap_trace_id(dev, &buffer[2],
			pre_buf_len - 2, touch_num);*/


	/*clear buffer*/
	memset(touch_data->coords, 0x00, sizeof(touch_data->coords));
	memset(touch_data->pen_coords, 0x00, sizeof(touch_data->pen_coords));

	/*"0 ~ touch_num - 2" is finger, "touch_num - 1" may be a finger or a pen*/
	/*process "0 ~ touch_num -2"*/
	if(touch_num > 0)
	{
	for (i = 0; i < touch_num - 1; i++) {
		/*coords->id = buffer[i * BYTES_PER_COORD + 2] & 0x0f;*/
		coords->id = buffer[i * BYTES_PER_COORD + 2] & 0x0f;
		coords->x = buffer[i * BYTES_PER_COORD + 3] |
						(buffer[i * BYTES_PER_COORD + 4] << 8);
		coords->y = buffer[i * BYTES_PER_COORD + 5] |
						(buffer[i * BYTES_PER_COORD + 6] << 8);
		coords->w = buffer[i * BYTES_PER_COORD + 7];
		coords->p = coords->w;
		coords->custom_data = touch_data->custom_data_buf;


		VTD("D:[%d](%d, %d)[%d]", coords->id, coords->x, coords->y,
				coords->w);
		coords++;
	}

	/*process "touch_num - 1", it may be a finger or a pen*/
	/*it's a pen*/
	i = touch_num - 1;
	if (touch_num >= 1 && buffer[i * BYTES_PER_COORD + 2] >= 0x80) {
		if (dev->board_data->pen_enable) {/*pen_enable*/
			touch_data->pen_down = true;

			/*change pen's trace ID, let it equal to "panel_max_id - 1"*/
			/*touch_data->pen_coords[0].id = dev->board_data->panel_max_id - 1;*/
			touch_data->pen_coords[0].id = dev->board_data->panel_max_id * 2;
			touch_data->pen_coords[0].x = buffer[i * BYTES_PER_COORD + 3] |
				(buffer[i * BYTES_PER_COORD + 4] << 8);
			touch_data->pen_coords[0].y = buffer[i * BYTES_PER_COORD + 5] |
				(buffer[i * BYTES_PER_COORD + 6] << 8);
			touch_data->pen_coords[0].w = buffer[i * BYTES_PER_COORD + 7];
			touch_data->pen_coords[0].p = touch_data->pen_coords[0].w;

			VTD("EP:[%d](%d, %d)", touch_data->pen_coords[0].id,
					touch_data->pen_coords[0].x, touch_data->pen_coords[0].y);

		}
	} else {/*it's a finger*/
		coords->id = buffer[i * BYTES_PER_COORD + 2] & 0x0f;
		coords->x = buffer[i * BYTES_PER_COORD + 3] |
						(buffer[i * BYTES_PER_COORD + 4] << 8);
		coords->y = buffer[i * BYTES_PER_COORD + 5] |
						(buffer[i * BYTES_PER_COORD + 6] << 8);
		coords->w = buffer[i * BYTES_PER_COORD + 7];
		coords->p = coords->w;
		coords->custom_data = touch_data->custom_data_buf;

		/*VTD("EF:[%d](%d, %d)", coords->id, coords->x, coords->y);*/
		if (touch_data->pen_down == true) {
			touch_data->pen_down = false;
			VTI("***pen leave");
		}
	}
	}

	/*swap coord*/
	goodix_swap_coords(dev, &touch_data->coords[0], touch_num);
	goodix_swap_coords(dev, &touch_data->pen_coords[0], 1);

    VTD("buffer[0]=0x%2x,FPPointCnt_V2=0x%2x", buffer[0], FPPointCnt_V2);
	goodix_FPreport(data, dev, buffer[0]);
	#ifdef CONFIG_VIRTUAL_PROX_GOODIX
	if(cur_state != goodix_last_prox_state){
		
		goodix_last_prox_state = cur_state;
		if((cur_state & 0x40) == 0x40){//0X40 
		 pick[0] = 0;
		 VTI("goodix virtual prox detec near !!!!");
		 VTI("goodix custom data[0]:0x%2x,data[1]:0x%2x",touch_data->custom_data_buf[0],touch_data->custom_data_buf[1]);
		 
		}
		else{
		 pick[0] = 1;
		 VTI("goodix virtual prox detec far!!!! ");	
		 VTI("goodix custom data[0]:0x%2x,data[1]:0x%2x",touch_data->custom_data_buf[0],touch_data->custom_data_buf[1]);
		}
		vts_proxminity_report(data->vtsdev, pick[0], pick[1], pick[2]);
	}
	#endif
	touch_data->touch_num = touch_num;
	/* mark this event as touch event */
	ts_event->event_type = EVENT_TOUCH;

	r = 0;

exit_clean_sta:
	/* handshake */
	buffer[0] = 0x00;
	goodix_spi_write_trans(dev, dev->reg.coor, buffer, 1);/*TS_REG_COORDS_BASE*/
/*
	if (1) {
	r = vivoTsCoverMute(touch_num, large_status);
	if (r < 0) {
		if (-2 == r) {
			VTI("enter larger or 3 finger mode");
			return 0;
		}
		if (-3 == r) {
			VTI("lcd shutoff, not report points");
			//return 0;
		}
	}
}
*/		
	return r;
}

static int goodix_event_handler(struct goodix_ts_core *data, struct goodix_ts_device *dev,
		struct goodix_ts_event *ts_event, ktime_t kt)
{
	unsigned char pre_buf[4 + BYTES_PER_COORD];
	unsigned char event_sta;
	int r = 0;

	memset(pre_buf, 0, sizeof(pre_buf));

	r = goodix_spi_read_trans(dev, dev->reg.coor,
			pre_buf, 4 + BYTES_PER_COORD);
	if (unlikely(r < 0))
		return r;

	/* buffer[0]: event state */
	event_sta = pre_buf[0];
	if (likely((event_sta & 0x80) == 0x80)) {
		/*handle touch event*/
		goodix_touch_handler(data, dev,
				ts_event,
				pre_buf,
				4 + BYTES_PER_COORD, kt);
	} else if (unlikely((event_sta & 0x40) == 0x40)) {
		/* handle request event */
		ts_event->event_type = EVENT_REQUEST;
		goodix_request_handler(dev,
				&ts_event->event_data.request_data);
	} else if ((event_sta & 0x20) == 0x20) {
		/* handle gesture event */
		VTI("Gesture event");
	} else if ((event_sta & 0x10) == 0x10) {
		/* handle hotknot event */
		VTI("Hotknot event");
	} else {
		VTI(" unknow event type, event_sta is 0x%x", event_sta);
		r = -EINVAL;
	}

	return r;
}

/**
 * goodix_hw_suspend - Let touch deivce stay in lowpower mode.
 * @dev: pointer to goodix touch device
 * @return: 0 - succeed, < 0 - failed
 */
static int goodix_hw_suspend(struct goodix_ts_device *dev)
{
	struct goodix_ts_cmd sleep_cmd;
	struct goodix_ts_core *core_data = platform_get_drvdata(goodix_pdev);
	int r = 0;
	if(1){//in charge state ,enter into sleep mode instead of power off
		goodix_cmds_init(&sleep_cmd, COMMAND_SLEEP, 0, dev->reg.command);
		if (sleep_cmd.initialized) {
			r = goodix_send_command_spi(dev, &sleep_cmd);
			if (!r)
				VTI("Chip in sleep mode");
		} else
			VTE("Uninitialized sleep command");
		return r;
	}
	VTI("power off sleep mode");
	r = goodix_ts_power_off_V2(core_data);
	if (r) {
		VTE("%s: power off failed, ret %d", __func__, r);
		goto exit;
	}

	core_data->power_off_sleep = true;
exit:
	return r;

}

/**
 * goodix_hw_resume - Let touch deivce stay in active  mode.
 * @dev: pointer to goodix touch device
 * @return: 0 - succeed, < 0 - failed
 */
static int goodix_hw_resume(struct goodix_ts_device *dev)
{
	int r = 0;
	int	retry = GOODIX_BUS_RETRY_TIMES;
	struct goodix_ts_core *core_data = platform_get_drvdata(goodix_pdev);
	u8 temp_buf[256], checksum;
	static struct firmware temp_firmware;
	int  fw_size;
	if (core_data->power_off_sleep) {
		r = goodix_ts_power_on_V2(core_data);
		if (r) {
			VTE("%s: power on failed, ret %d", __func__, r);
			goto exit;
		}
		core_data->power_off_sleep = false;
	}
	for (; retry--;) {
		goodix_hw_reset_spi(dev);

		/*read version and check checksum*/
		if (dev->reg.version_base && dev->reg.version_len < 256) {
			r = goodix_spi_read(dev, dev->reg.version_base,
					temp_buf, dev->reg.version_len);
			if (r < 0)
				continue;

			checksum = checksum_u8(temp_buf, dev->reg.version_len);
			if (!checksum) {
				VTI("read version SUCCESS");
				VTI("VID:%02x, %02x, %02x, %02x",temp_buf[17],temp_buf[18],temp_buf[19],temp_buf[20]);
				if(0x00 == temp_buf[17] && 
					0x01 == temp_buf[18] && 
					0x00 == temp_buf[19] &&
					0x01 == temp_buf[20] ){
					VTE("Need upgrade FW");
					temp_firmware.data = vts_fw_data_get(dev->vtsdev, VTS_FW_TYPE_FW, &fw_size);
					temp_firmware.size = fw_size;
					r =  bbk_goodix_fw_update_V2(NULL, &temp_firmware);
				}
				break;
			}else {
				vts_communication_abnormal_collect(TOUCH_VCODE_I2C_EVENT);
			}
		}
	}
exit:
	return r;
}

static int goodix_esd_check(struct goodix_ts_device *dev)
{
	int r = 0;
	u8 data = 0;

	if (dev->reg.esd == 0) {
		VTE("esd reg is NULL");
		return 0;
	}

	/*check dynamic esd*/
	if (dev->ic_type == IC_TYPE_NORMANDY_SPI)
		r = dev->hw_ops->read_trans(dev,
				0x3103, &data, 1);
	else
		r = dev->hw_ops->read_trans(dev,
				dev->reg.esd, &data, 1);

	if (r < 0 || (data == 0xaa)) {
		VTI("dynamic esd occur, r:%d, data:0x%02x", r, data);
		r = -EINVAL;
		goto exit;
	}

	/*check static esd*/
	if (dev->ic_type == IC_TYPE_NANJING) {
		r = dev->hw_ops->read_trans(dev,
				0x8043, &data, 1);

		if (r < 0 || (data != 0xaa)) {
			VTI("static esd occur, r:%d, data:0x%02x", r, data);
			r = -EINVAL;
			goto exit;
		}
	}

exit:
	return r;
}

static int goodix_spi_transfer_test(struct goodix_ts_device *dev)
{   
    struct spi_device *spi = dev->spi_dev;
 	u8 buf[2];
	int r = 0;
	u32 speed = 3500;
     msleep(5);

	while(speed < 6000000){
	VTI("set spi speed %dhz\n", speed);
	//spi->mode = SPI_MODE_0;
	spi->max_speed_hz = speed;
	//spi->bits_per_word = 8;
	 buf[0] = 0xaa;
	 buf[1] = 0xbb;
	 r = dev->hw_ops->write_trans(dev, 0x452c, &buf[0], 2);
	 msleep(5);

	 //spi->max_speed_hz = 60000;

	 buf[0] = 0xcc;
	 buf[1] = 0xdd;
	 r = dev->hw_ops->read_trans(dev, 0x452c, &buf[0], 2);
	 VTI("452c:%02x--%02x",buf[0],buf[1]);
	 if(buf[0] == 0xaa && buf[1] == 0xbb){
	 	VTI("spi speed %dhz\n is OK", speed);
	 	//break;
	 }
	 speed += 10000;
	}
	 VTI("set spi speed %dhz\n", 6000000);
	spi->max_speed_hz = 6000000;

	return 0;
}
/* hardware opeation funstions */
static const struct goodix_ts_hw_ops hw_spi_ops = {
	.init = goodix_hw_init,
	.read = goodix_spi_read,
	.write = goodix_spi_write,
	.read_trans = goodix_spi_read_trans,
	.write_trans = goodix_spi_write_trans,
	.reset = goodix_hw_reset_spi,
	.event_handler = goodix_event_handler,
	.send_config = goodix_send_config,
	.read_config = goodix_read_config,
	.send_cmd = goodix_send_command_spi,
	.read_version = goodix_read_version,
	.suspend = goodix_hw_suspend,
	.resume = goodix_hw_resume,
	.check_hw = goodix_esd_check,
	.set_doze_mode = goodix_set_spi_doze_mode,
	.spi_transfer_test = goodix_spi_transfer_test,
};


static void goodix_pdev_release(struct device *dev)
{
	VTI("goodix pdev released");
}

static int goodix_spi_probe(struct spi_device *spi ,struct device_node *np)
{
	struct goodix_ts_device *ts_device = NULL;
	struct goodix_ts_board_data *ts_bdata = NULL;
	int r = 0;

	VTI("%s IN", __func__);

	/* init spi_device */
	spi->mode            = SPI_MODE_0;
	spi->bits_per_word   = 8;
	/* board data */
	ts_bdata = devm_kzalloc(&(spi->dev),
			sizeof(struct goodix_ts_board_data), GFP_KERNEL);
	if (!ts_bdata)
		return -ENOMEM;
    VTI("%s IN :1", __func__);
	/* init ts device data */
	ts_device = devm_kzalloc(&spi->dev,
		sizeof(struct goodix_ts_device), GFP_KERNEL);
	if (!ts_device)
		return -ENOMEM;

	/* alloc memory for spi transfer buffer */
	ts_device->tx_buff = kzalloc(GOODIX_SPI_BUFF_MAX_SIZE, GFP_KERNEL|GFP_DMA);
	ts_device->rx_buff = kzalloc(GOODIX_SPI_BUFF_MAX_SIZE, GFP_KERNEL|GFP_DMA);
	if (!ts_device->tx_buff || !ts_device->rx_buff) {
		VTE("%s: out of memory\n", __func__);
		r = -ENOMEM;
		goto err_spi_buf;
	}

	ts_device->name = "Goodix TouchDevcie";
	ts_device->spi_dev= spi;
	ts_device->dev = &spi->dev;
	ts_device->dev->of_node = np;
	ts_device->hw_ops = &hw_spi_ops;
	ts_device->board_data = ts_bdata;
	ts_device->int_state = false;

	mutex_init(&ts_device->spi_mutex);

	/* parse devicetree property */
	if (IS_ENABLED(CONFIG_OF) && np) {
		r = goodix_parse_dt(np,
				    ts_device->board_data);
		if (r < 0) {
			VTE("failed parse device info form dts, %d", r);
			r = -EINVAL;
			goto err_spi_buf;
		}
	} else {
		VTE("no valid device tree node found");
		r = -ENODEV;
		goto err_spi_buf;
	}

	/* init spi_device */
	spi->max_speed_hz  = ts_device->board_data->spi_max_speed;


	/* init ts core device */
	goodix_pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (!goodix_pdev) {
		VTE("kzalloc core dev goodix_pdev faliled!");
		r = -ENOMEM;
		goto err_spi_buf;
	}

	/*
	 * you can find this platform dev in
	 * /sys/devices/platfrom/goodix_ts.0
	 * goodix_pdev->dev.parent = &spi->dev;
	 */
	goodix_pdev->name = GOODIX_CORE_DRIVER_NAME;
	goodix_pdev->id = 0;
	goodix_pdev->num_resources = 0;
	goodix_pdev->dev.platform_data = ts_device;
	goodix_pdev->dev.release = goodix_pdev_release;

	/* register platform device, then the goodix_ts_core
	 * module will probe the touch deivce.
	 */
	r = platform_device_register(goodix_pdev);
	if (r) {
		VTE("failed register goodix platform device, %d", r);
		goto err_pdev;
	}

	VTI("%s OUT", __func__);
	return r;

err_pdev:
	if (goodix_pdev) {
		kfree(goodix_pdev);
		goodix_pdev = NULL;
	}
err_spi_buf:
	if(ts_device->tx_buff){
		kfree(ts_device->tx_buff);
		ts_device->tx_buff = NULL;
	}
	if(ts_device->rx_buff){
		kfree(ts_device->rx_buff);
		ts_device->rx_buff = NULL;
	}

	VTI("%s OUT, %d", __func__, r);
	return r;
}

static int goodix_spi_remove(struct spi_device *spi, struct device_node *np)
{
	if (goodix_pdev) {
		platform_device_unregister(goodix_pdev);
		kfree(goodix_pdev);
		goodix_pdev = NULL;
	}

	return 0;
}
#if defined(CONFIG_MEDIATEK_SOLUTION)
static void goodix_spi_shutdown(struct spi_device *spi)
{	
	
	int r = 0;
	struct goodix_ts_core *core_data = platform_get_drvdata(goodix_pdev);
	if (core_data){
	   VTI("power off shutdown");
	   r = goodix_ts_power_off_V2(core_data);
	   if (r) 
          VTE("%s: power off failed, ret %d", __func__, r);
	}
	
}
#endif

static struct vts_spi_driver goodix_spi_driver = {
	.probe		= goodix_spi_probe,
	.remove		= goodix_spi_remove,
	.compatible = "goodix,gt9885-spi",
	#if defined(CONFIG_MEDIATEK_SOLUTION)
	.shutdown = goodix_spi_shutdown,
	#endif
};

static unsigned long gt9885_flags;
enum gt9885_flags {
	GT9885_SPI_INIT = BIT(0),
	GT9885_CORE_INIT = BIT(1),
	GT9885_TOOLS_INIT = BIT(2),
	GT9885_GESTURE_INIT = BIT(3)
};

int goodix_gt9885_init(void)
{
	if (vts_spi_drv_reigster(&goodix_spi_driver))
		return -1;

	gt9885_flags |= GT9885_SPI_INIT;
	if (goodix_ts_core_V2_init())
		return -1;

	gt9885_flags |= GT9885_CORE_INIT;
	if (goodix_tools_V2_init())
		return -1;

	gt9885_flags |= GT9885_TOOLS_INIT;
	if (goodix_gsx_gesture_V2_init())
		return -1;

	gt9885_flags |= GT9885_GESTURE_INIT;

	return 0;
}

void goodix_gt9885_exit(void)
{
	if (gt9885_flags & GT9885_GESTURE_INIT)
		goodix_gsx_gesture_V2_exit();

	if (gt9885_flags & GT9885_TOOLS_INIT)
		goodix_tools_V2_exit();

	if (gt9885_flags & GT9885_CORE_INIT)
		goodix_ts_core_V2_exit();

	if (gt9885_flags & GT9885_SPI_INIT)
		vts_spi_drv_unreigster(&goodix_spi_driver);
}

static const int ic_numbers[] = {VTS_IC_GT9885TS};
module_vts_driver(goodix_g9885, ic_numbers, goodix_gt9885_init(), goodix_gt9885_exit());

