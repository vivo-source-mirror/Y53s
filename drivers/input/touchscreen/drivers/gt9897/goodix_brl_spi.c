 /*
  * Goodix Touchscreen Driver
  * Copyright (C) 2020 - 2021 Goodix, Inc.
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
  *
  */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "goodix_ts_core.h"
#define TS_DRIVER_NAME		"gtx8_spi"

#define SPI_TRANS_PREFIX_LEN    1
#define REGISTER_WIDTH          4
#define SPI_READ_DUMMY_LEN      4
#define SPI_READ_PREFIX_LEN  (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH + SPI_READ_DUMMY_LEN)
#define SPI_WRITE_PREFIX_LEN (SPI_TRANS_PREFIX_LEN + REGISTER_WIDTH)

#define SPI_WRITE_FLAG  0xF0
#define SPI_READ_FLAG   0xF1

static struct platform_device *goodix_pdev;
struct goodix_bus_interface goodix_spi_bus;
static struct mutex bus_mutex;

/**
 * goodix_spi_read- read device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: read buffer
 * @len: bytes to read
 * return: 0 - read ok, < 0 - spi transter error
 */
static int goodix_spi_read(struct device *dev, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 *rx_buf = NULL;
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;

	rx_buf = goodix_spi_bus.rx_buf;
	tx_buf = goodix_spi_bus.tx_buf;
	if (!rx_buf || !tx_buf) {
		ts_err("alloc tx/rx_buf failed, size:%d\n",
			SPI_READ_PREFIX_LEN + len);
		return -ENOMEM;
	}
	mutex_lock(&bus_mutex);
	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	/*spi_read tx_buf format: 0xF1 + addr(2bytes) + data*/
	tx_buf[0] = SPI_READ_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	tx_buf[5] = 0xFF;
	tx_buf[6] = 0xFF;
	tx_buf[7] = 0xFF;
	tx_buf[8] = 0xFF;

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = rx_buf;
	xfers.len = SPI_READ_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0) {
		ts_err("spi transfer error:%d\n",ret);
		goto exit;
	}
	memcpy(data, &rx_buf[SPI_READ_PREFIX_LEN], len);

exit:
	mutex_unlock(&bus_mutex);
	return ret;
}

/**
 * goodix_spi_write- write device register through spi bus
 * @dev: pointer to device data
 * @addr: register address
 * @data: write buffer
 * @len: bytes to write
 * return: 0 - write ok; < 0 - spi transter error.
 */
static int goodix_spi_write(struct device *dev, unsigned int addr,
		unsigned char *data, unsigned int len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 *tx_buf = NULL;
	struct spi_transfer xfers;
	struct spi_message spi_msg;
	int ret = 0;
	
	mutex_lock(&bus_mutex);
	tx_buf = goodix_spi_bus.tx_buf;


	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	tx_buf[0] = SPI_WRITE_FLAG;
	tx_buf[1] = (addr >> 24) & 0xFF;
	tx_buf[2] = (addr >> 16) & 0xFF;
	tx_buf[3] = (addr >> 8) & 0xFF;
	tx_buf[4] = addr & 0xFF;
	memcpy(&tx_buf[SPI_WRITE_PREFIX_LEN], data, len);
	xfers.tx_buf = tx_buf;
	xfers.len = SPI_WRITE_PREFIX_LEN + len;
	xfers.cs_change = 0;
	spi_message_add_tail(&xfers, &spi_msg);
	ret = spi_sync(spi, &spi_msg);
	if (ret < 0)
		ts_err("spi transfer error:%d\n",ret);
	
	mutex_unlock(&bus_mutex);
	return ret;
}

static void goodix_pdev_release(struct device *dev)
{
	ts_info("goodix pdev released");
	kfree(goodix_pdev);
}
#define GOODIX_SPI_BUFF_MAX_SIZE_BERLI	(8 * 1024 + 16)

static int goodix_spi_probe(struct spi_device *spi, struct device_node *np)
{
	int ret = 0;

	ts_info("goodix spi probe in");

	/* init spi_device */
	spi->mode            = SPI_MODE_0;
	spi->bits_per_word   = 8;
	spi->max_speed_hz    = 8000000;
	mutex_init(&bus_mutex);

	ret = spi_setup(spi);
	if (ret) {
		ts_err("failed set spi mode, %d", ret);
		return ret;
	}
	/* alloc memory for spi transfer buffer */
	goodix_spi_bus.tx_buf = kzalloc(GOODIX_SPI_BUFF_MAX_SIZE_BERLI, GFP_KERNEL|GFP_DMA);
	goodix_spi_bus.rx_buf = kzalloc(GOODIX_SPI_BUFF_MAX_SIZE_BERLI, GFP_KERNEL|GFP_DMA);
	if (!goodix_spi_bus.tx_buf || !goodix_spi_bus.rx_buf) {
		ts_err("%s: out of memory\n", __func__);
		ret = -ENOMEM;
		goto err_spi_buf;
	}

	goodix_spi_bus.bus_type = GOODIX_BUS_TYPE_SPI;
	goodix_spi_bus.dev = &spi->dev;
	goodix_spi_bus.dev->of_node = np;
	goodix_spi_bus.read = goodix_spi_read;
	goodix_spi_bus.write = goodix_spi_write;
	
	/* ts core device */
	goodix_pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (!goodix_pdev)
		return -ENOMEM;

	goodix_pdev->name = GOODIX_CORE_DRIVER_NAME;
	goodix_pdev->id = 0;
	goodix_pdev->num_resources = 0;
	/*
	 * you can find this platform dev in
	 * /sys/devices/platfrom/goodix_ts.0
	 * goodix_pdev->dev.parent = &client->dev;
	 */
	goodix_pdev->dev.platform_data = &goodix_spi_bus;
	goodix_pdev->dev.release = goodix_pdev_release;

	/* register platform device, then the goodix_ts_core
	 * module will probe the touch deivce.
	 */
	ret = platform_device_register(goodix_pdev);
	if (ret) {
		ts_err("failed register goodix platform device, %d", ret);
		goto err_pdev;
	}
	//ret = gt9897_ts_core_init();
	//if (ret) {
		//ts_err("failed register goodix driver, %d", ret);
		//goto err_pdriver;
	//}
	ts_info("spi probe out");
	return 0;
	
err_spi_buf:
		
	if(goodix_spi_bus.tx_buf){
		kfree(goodix_spi_bus.tx_buf);
		goodix_spi_bus.tx_buf = NULL;
	}
	if(goodix_spi_bus.rx_buf){
		kfree(goodix_spi_bus.rx_buf);
		goodix_spi_bus.rx_buf = NULL;
	}	
err_pdev:
	kfree(goodix_pdev);
	goodix_pdev = NULL;
	ts_info("spi probe out, %d", ret);
	return ret;
}

static int goodix_spi_remove(struct spi_device *spi, struct device_node *np)
{
	platform_device_unregister(goodix_pdev);
	return 0;
}

static struct vts_spi_driver goodix_gt9897_driver = {
	.probe		= goodix_spi_probe,
	.remove		= goodix_spi_remove,
	.compatible = "goodix,gt9897-spi",
};

static unsigned long gt9897_flags;
enum GT9897_FLAGS {
	GT9897_SPI_INIT = BIT(0),
	GT9897_CORE_INIT = BIT(1),
	GT9897_TOOLS_INIT = BIT(2),
	GT9897_GESTURE_INIT = BIT(3)
};

int goodix_gt9897_init(void)
{
	if (vts_spi_drv_reigster(&goodix_gt9897_driver))
		return -1;

	gt9897_flags |= GT9897_SPI_INIT;
	if (gt9897_ts_core_init())
		return -1;

	gt9897_flags |= GT9897_CORE_INIT;
	if (gt9897_tools_init())
		return -1;

	gt9897_flags |= GT9897_TOOLS_INIT;
	if (gt9897_gsx_gesture_init())
		return -1;

	gt9897_flags |= GT9897_GESTURE_INIT;

	return 0;
}

void goodix_gt9897_exit(void)
{
	if (gt9897_flags & GT9897_GESTURE_INIT)
		gt9897_gsx_gesture_exit();

	if (gt9897_flags & GT9897_TOOLS_INIT)
		gt9897_tools_exit();

	if (gt9897_flags & GT9897_CORE_INIT)
		gt9897_ts_core_exit();

	if (gt9897_flags & GT9897_SPI_INIT)
		vts_spi_drv_unreigster(&goodix_gt9897_driver);
}

static const int ic_numbers[] = {VTS_IC_GT9897};
module_vts_driver(goodix_gt9897, ic_numbers, goodix_gt9897_init(), goodix_gt9897_exit());

