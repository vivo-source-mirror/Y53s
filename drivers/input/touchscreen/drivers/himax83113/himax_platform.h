/*  Himax Android Driver Sample Code for QCT platform

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#ifndef HIMAX_PLATFORM_H
#define HIMAX_PLATFORM_H

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#if defined(CONFIG_HMX_DB)
	#include <linux/regulator/consumer.h>
#endif
#include "../vts_core.h"

#define HIMAX_XFER_RETRY_TIMES 10

#define D(fmt, x...) printk("[VIVO_TS_DBG][%s]:%d: "fmt , __func__, __LINE__, ##x)
#define I(fmt, x...) printk("[VIVO_TS_INF][%s]:%d: "fmt , __func__, __LINE__, ##x)
#define W(fmt, x...) printk("[VIVO_TS_INF][WARNING][%s]:%d: "fmt , __func__, __LINE__, ##x)
#define E(fmt, x...) printk("[VIVO_TS_INF][ERROR][%s]:%d: "fmt , __func__, __LINE__, ##x)
#define DIF(x...) \
do { \
	if (debug_flag) \
		printk("[HXTP][DEBUG] " x) \
	} while (0)

#if defined(CONFIG_HMX_DB)
	/* Analog voltage @2.7 V */
	#define HX_VTG_MIN_UV			2700000
	#define HX_VTG_MAX_UV			3300000
	#define HX_ACTIVE_LOAD_UA		15000
	#define HX_LPM_LOAD_UA			10
	/* Digital voltage @1.8 V */
	#define HX_VTG_DIG_MIN_UV		1800000
	#define HX_VTG_DIG_MAX_UV		1800000
	#define HX_ACTIVE_LOAD_DIG_UA		10000
	#define HX_LPM_LOAD_DIG_UA		10
#endif

#define HIMAX_COMMON_NAME			"himax_tp"

struct himax_platform_data {
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	int screenWidth;
	int screenHeight;
	int tx;
	int rx;
	uint8_t fw_version;
	uint8_t tw_id;
	uint8_t cable_config[2];
	uint8_t protocol_type;
	int gpio_irq;
	int gpio_reset;
	int gpio_cs;
	int gpio_3v3_en;
	void (*reset)(void);
	u32 spi_frequency;
	int hx_config_size;
#if defined(CONFIG_HMX_DB)	
	bool	digital_pwr_regulator;
	int reset_gpio;
	u32 reset_gpio_flags;
	int irq_gpio;
	u32 irq_gpio_flags;
	
	struct regulator *vcc_ana; /* For Dragon Board */
	struct regulator *vcc_dig; /* For Dragon Board */	
#endif
};


//extern int irq_enable_count;
extern int himax_bus_read(uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry);
extern int himax_bus_write(uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry);
extern int himax_bus_write_command(uint8_t command, uint8_t toRetry);
extern int himax_bus_master_write(uint8_t *data, uint32_t length, uint8_t toRetry);
extern void himax_int_enable(int enable);
extern int himax_ts_register_interrupt(void);
extern uint8_t himax_int_gpio_read(int pinnum);

extern int himax_gpio_power_config(struct himax_platform_data *pdata);
extern int himax_hx83112_init(void);
extern void himax_hx83112_exit(void);

/*
#if defined(CONFIG_FB)
	extern int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
*/
#endif
