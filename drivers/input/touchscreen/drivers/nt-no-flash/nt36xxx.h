/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT_TOUCH_H
#define		_LINUX_NVT_TOUCH_H

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "nt36xxx_mem_map.h"
#include "../vts_core.h"

#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#elif defined CONFIG_MTK_SPI
#include "mtk_spi.h"
#endif
#ifdef CONFIG_ARCH_BENGAL
#include <linux/spi/spi-geni-qcom.h>
#endif
#define NVT_DEBUG 1

//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
//#define IRQ_TYPE_EDGE_FALLING 2
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_RISING


//---SPI driver info.---
#define NVT_SPI_NAME "NVT-ts-spi-v2"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    VTI("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    VTI("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    VTE("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"


//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 1080
#define TOUCH_DEFAULT_MAX_HEIGHT 2400
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_FORCE_NUM 1000

//---Customerized Define.---//[20180525,jx]	
	#define _CustomerFunction_ 	(1)
	#define POINT_DATA_CHECKSUM	(1)	//[20180606]Add for Gesture
#define POINT_DATA_CHECKSUM_LEN 65

/* Enable only when module have tp reset pin and connected to host */
#define NVT_TOUCH_SUPPORT_HW_RST 1

//---Customerized func.---
#define NVT_TOUCH_PROC 1
#define NVT_TOUCH_EXT_PROC 1
#define NVT_TOUCH_MP 0
#define WAKEUP_GESTURE 1
#define BOOT_UPDATE_FIRMWARE 1
#define BOOT_UPDATE_FIRMWARE_NAME "vivo_novatek_ts_fw.bin"
#define MP_UPDATE_FIRMWARE_NAME "vivo_novatek_ts_mp.bin"
#define FWTYPE_MP     (0)
#define FWTYPE_Normal (1)

#define FWTYPE_REQUEST_YES		1
#define FWTYPE_REQUEST_NO		0

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */
#define NVT_TOUCH_WDT_RECOVERY 1
#define NVT_TOUCH_ESD_DISP_RECOVERY 1

struct nvt_ts_gesture{
u8 need_set;
u8 ges_bit[2];
};

struct nvt_ts_data {
	struct spi_device *client;
	struct device_node *node;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	struct device *dev;
	struct vts_device *vtsdev;
	int rotation;
	int charging;
	uint16_t addr;
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
	int32_t irq_gpio;
	uint32_t irq_flags;
	atomic_t irq_enabled;
	int32_t reset_gpio;
	uint32_t reset_flags;
	int32_t clk_gpio;
	uint32_t clk_flags;
	int32_t cs_gpio;
	uint32_t cs_flags;
	uint32_t spi_frequency;
	uint32_t down_x[10];
	uint32_t down_y[10];
	uint32_t down_z[10];
	struct mutex lock;
	const uint8_t *trimid;	//[20180524]For APK
	const struct nvt_ts_mem_map *mmap;
	uint8_t carrier_system;
	uint8_t hw_crc;
	uint16_t nvt_pid;
	uint8_t rbuf[1025];	
    uint8_t wkg_info;
	uint8_t *xbuf;
#ifdef CONFIG_SPI_MT65XX
	struct mtk_chip_config spi_ctrl;
#elif defined CONFIG_MTK_SPI
	struct mt_chip_conf spi_ctrl;
#endif

	int fm_switch;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM];
	uint8_t press_id_pre[TOUCH_MAX_FINGER_NUM];
	int finger_cnt;
	uint8_t custom_data[2];

	bool cs_bootup;
	bool vddi_poweroff;
	bool nt_4power;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl_state *spi_cs_active;
	struct pinctrl_state *spi_cs_sleep_pulllow;
	struct pinctrl_state *spi_miso_active;
	struct pinctrl_state *spi_miso_sleep_pulllow;
	struct pinctrl_state *spi_clk_active;
	struct pinctrl_state *spi_mosi_active;
	struct mutex xbuf_lock;
	int32_t miso_gpio;
	uint32_t miso_flags;
	struct nvt_ts_gesture gesture_set;
	#ifdef CONFIG_ARCH_BENGAL
	struct spi_geni_qcom_ctrl_data qcom_4250_spi_ctrl;
	#endif
	uint8_t *g_data;	
};

#if NVT_TOUCH_PROC
struct nvt_flash_data{
	rwlock_t lock;
};

//[20180904,jx]Change to read node (from write node) for SELinux
struct nvt_fwupdate_data{
	rwlock_t lock;
};

#endif
typedef enum {
	NVTILM    = 0,
	NVTDLM    = 1,
	NVTINFO   = 2,
	NVTHEADER = 3,
	NVTOVLAY  = 4,
	NVTWKG    = 5
} NVT_PARTITION_TYPE;

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
#define NVT_TANSFER_LEN		(63*1024)	//[20190508,jx]New; Old(128*1024)
#define NTV_GESTURE_FW_CHECK_LEN   1024 
#define NVTFLASH_WORK_PROTECT (1)
#if (NVTFLASH_WORK_PROTECT)
	extern atomic_t u8_NT36xxx_flashWorking;
#endif

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ntnf_spi;

//---extern functions---
int32_t NTNF_CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t NTNF_CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void ntnf_bootloader_reset(void);
void ntnf_sw_reset(void);
void ntnf_sw_reset_idle(void);
void ntnf_boot_ready(void);
void ntnf_bld_crc_enable(void);
void ntnf_fw_crc_enable(void);
void ntnf_update_firmware(char *firmware_name, int fwtype, int fwRequest);
int32_t ntnf_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t ntnf_get_fw_info(void);
int32_t ntnf_clear_fw_status(void);
int32_t ntnf_check_fw_status(void);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable_v2(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

int32_t ntnf_set_page(uint32_t addr);
int32_t ntnf_write_addr(uint32_t addr, uint8_t data);
void nvt_eng_reset_v2(void);



#endif /* _LINUX_NVT_TOUCH_H */
