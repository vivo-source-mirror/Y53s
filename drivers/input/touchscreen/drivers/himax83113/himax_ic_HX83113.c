/* SPDX-License-Identifier: GPL-2.0 */
/*  Himax Android Driver Sample Code for HX83113 chipset
 *
 *  Copyright (C) 2019 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include "himax_ic_HX83113.h"
extern struct himax_ts_data *private_ts;
extern struct himax_platform_data *hv_pdata;

extern struct himax_core_fp g_core_fp;
extern struct ic_operation *pic_op;
extern struct fw_operation *pfw_op;
#ifdef HX_ZERO_FLASH
extern struct zf_operation *pzf_op;
#endif

extern struct himax_chip_detect *g_core_chip_dt;

extern uint8_t g_hx_ic_dt_num;
extern int xfer_error_count;
extern unsigned char IC_TYPE;
extern unsigned char IC_CHECKSUM;

extern unsigned long FW_VER_MAJ_FLASH_ADDR;
extern unsigned long FW_VER_MIN_FLASH_ADDR;
extern unsigned long CFG_VER_MAJ_FLASH_ADDR;
extern unsigned long CFG_VER_MIN_FLASH_ADDR;
extern unsigned long CID_VER_MAJ_FLASH_ADDR;
extern unsigned long CID_VER_MIN_FLASH_ADDR;

extern unsigned long FW_VER_MAJ_FLASH_LENG;
extern unsigned long FW_VER_MIN_FLASH_LENG;
extern unsigned long CFG_VER_MAJ_FLASH_LENG;
extern unsigned long CFG_VER_MIN_FLASH_LENG;
extern unsigned long CID_VER_MAJ_FLASH_LENG;
extern unsigned long CID_VER_MIN_FLASH_LENG;

static void hx83113_chip_init(void)
{

	(private_ts)->chip_cell_type = CHIP_IS_IN_CELL;
	I("%s:IC cell type = %d\n",  __func__,  (private_ts)->chip_cell_type);
	IC_CHECKSUM			= HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	FW_VER_MAJ_FLASH_ADDR   = 49157;  /*0x00C005*/
	FW_VER_MAJ_FLASH_LENG   = 1;
	FW_VER_MIN_FLASH_ADDR   = 49158;  /*0x00C006*/
	FW_VER_MIN_FLASH_LENG   = 1;
	CFG_VER_MAJ_FLASH_ADDR	= 49408;  /*0x00C100*/
	CFG_VER_MAJ_FLASH_LENG	= 1;
	CFG_VER_MIN_FLASH_ADDR	= 49409;  /*0x00C101*/
	CFG_VER_MIN_FLASH_LENG	= 1;
	CID_VER_MAJ_FLASH_ADDR	= 49154;  /*0x00C002*/
	CID_VER_MAJ_FLASH_LENG	= 1;
	CID_VER_MIN_FLASH_ADDR	= 49155;  /*0x00C003*/
	CID_VER_MIN_FLASH_LENG	= 1;
}

static bool hx83113_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4];

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 && tmp_data[0] != 0x87))
			g_core_fp.fp_register_write((pfw_op)->addr_ctrl_fw_isr, DATA_LEN_4, (pfw_op)->data_fw_stop, 0);

		usleep_range(10000, 11000);
		/* check fw status */
		g_core_fp.fp_register_read((pic_op)->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);

		if (tmp_data[0] != 0x05) {
			I("%s: Do not need wait FW, Status = 0x%02X!\n", __func__, tmp_data[0]);
			break;
		}

		g_core_fp.fp_register_read((pfw_op)->addr_ctrl_fw_isr, 4, tmp_data, false);
		I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__, cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 10) && check_en == true);

	cnt = 0;

	do {
		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 *===========================================
		 */
		tmp_data[0] = (pic_op)->data_sonoff_psw_lb[0];

		if (himax_bus_write((pic_op)->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/*===========================================
		 password[15:8] set Enter safe mode :0x32 ==> 0x95
		 *===========================================
		 */
		tmp_data[0] = (pic_op)->data_sonoff_psw_ub[0];

		if (himax_bus_write((pic_op)->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x00
		 *===========================================
		 */
		tmp_data[0] = 0x00;

		if (himax_bus_write((pic_op)->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x27
		 *===========================================
		 */
		tmp_data[0] = (pic_op)->data_sonoff_psw_lb[0];

		if (himax_bus_write((pic_op)->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/*===========================================
		 password[15:8] set Enter safe mode :0x32 ==> 0x95
		 *===========================================
		 */
		tmp_data[0] = (pic_op)->data_sonoff_psw_ub[0];

		if (himax_bus_write((pic_op)->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/* ======================
		 *Check enter_save_mode
		 *======================
		 */
		g_core_fp.fp_register_read((pic_op)->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);
		I("%s: Check enter_save_mode data[0]=%X\n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*=====================================
			 *Reset TCON
			 *=====================================
			 */
			g_core_fp.fp_register_write((pic_op)->addr_tcon_on_rst, DATA_LEN_4, (pic_op)->data_rst, 0);
			usleep_range(1000, 1100);
			tmp_data[3] = (pic_op)->data_rst[3];
			tmp_data[2] = (pic_op)->data_rst[2];
			tmp_data[1] = (pic_op)->data_rst[1];
			tmp_data[0] = (pic_op)->data_rst[0] | 0x01;
			g_core_fp.fp_register_write((pic_op)->addr_tcon_on_rst, DATA_LEN_4, tmp_data, 0);
			/*=====================================
			 *Reset ADC
			 *=====================================
			 */
			g_core_fp.fp_register_write((pic_op)->addr_adc_on_rst, DATA_LEN_4, (pic_op)->data_rst, 0);
			usleep_range(1000, 1100);
			tmp_data[3] = (pic_op)->data_rst[3];
			tmp_data[2] = (pic_op)->data_rst[2];
			tmp_data[1] = (pic_op)->data_rst[1];
			tmp_data[0] = (pic_op)->data_rst[0] | 0x01;
			g_core_fp.fp_register_write((pic_op)->addr_adc_on_rst, DATA_LEN_4, tmp_data, 0);
			goto SUCCED;
		} else {
			/*msleep(10);*/
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		}
	} while (cnt++ < 5);

	return false;
SUCCED:
	return true;
}

#if defined(HX_ZERO_FLASH)
static void himax_hx83113_reload_to_active(void)
{
	uint8_t addr[DATA_LEN_4] = {0};
	uint8_t data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	addr[3] = 0x90;
	addr[2] = 0x00;
	addr[1] = 0x00;
	addr[0] = 0x48;

	do {
		data[3] = 0x00;
		data[2] = 0x00;
		data[1] = 0x00;
		data[0] = 0xEC;
		g_core_fp.fp_register_write(addr, DATA_LEN_4, data, 0);
		usleep_range(1000, 1100);
		g_core_fp.fp_register_read(addr, DATA_LEN_4, data, 0);
		I("%s: data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__, data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01 || data[0] != 0xEC) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_hx83113_resume_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83113_reload_to_active();
#endif
}

static void himax_hx83113_suspend_ic_action(void)
{
#ifndef HX_RESUME_HW_RESET
	himax_hx83113_reload_to_active();
#endif
}

static void himax_hx83113_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int retry = 0;
	I("Enter %s \n", __func__);
	g_core_fp.fp_interface_on();

	if (!FlashMode) {
#ifdef HX_RST_PIN_FUNC
		/*g_core_fp.fp_ic_reset(false, false);*/
		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_lb[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("%s: xfer fail!\n", __func__);

		/*===========================================
		 password[15:8] set Enter safe mode :0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_ub[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("%s: xfer fail!\n", __func__);

		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x00
		===========================================*/
		tmp_data[0] = 0x00;

		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("%s: xfer fail!\n", __func__);
#else
		g_core_fp.fp_system_reset();
#endif
		himax_hx83113_reload_to_active();
	} else {
		himax_hx83113_reload_to_active();
		do {
			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_active), pfw_op->data_safe_mode_release_pw_active, 0);

			g_core_fp.fp_register_read(pfw_op->addr_flag_reset_event, DATA_LEN_4, tmp_data, 0);
			I("%s:Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01 || tmp_data[0] != 0x00) && retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
			himax_hx83113_reload_to_active();
		} else {
			I("%s:OK and Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
			}

			if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
			}

			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_reset), pfw_op->data_safe_mode_release_pw_reset, 0);
		}
	}
}
#endif

static void hx83113_func_re_init(void)
{
	g_core_fp.fp_sense_off = hx83113_sense_off;
	g_core_fp.fp_chip_init = hx83113_chip_init;
#if defined(HX_ZERO_FLASH)
	g_core_fp.fp_resume_ic_action = himax_hx83113_resume_ic_action;
	g_core_fp.fp_suspend_ic_action = himax_hx83113_suspend_ic_action;
	g_core_fp.fp_sense_on = himax_hx83113_sense_on;
	g_core_fp.fp_0f_reload_to_active = himax_hx83113_reload_to_active;
#endif
}

static bool hx83113_chip_detect(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	ret = himax_mcu_in_cmd_struct_init();
	if (ret < 0) {
		ret_data = false;
		E("%s:cmd_struct_init Fail:\n", __func__);
		return ret_data;
	}

	himax_mcu_in_cmd_init();
	hx83113_func_re_init();
	ret = himax_bus_read((pic_op)->addr_conti[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES);
	I("%s:addr=%8x,tmp_data0=0x%2x,tmp_data1=0x%2x,tmp_data2=0x%2x,tmp_data3=0x%2x\n", __func__, (pic_op)->addr_conti[0], tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	if (ret < 0) {
		E("%s: xfer fail!\n", __func__);
		return false;
	}

	if (g_core_fp.fp_sense_off(false) == false) {
		ret_data = false;
		E("%s:fp_sense_off Fail:\n", __func__);
		return ret_data;
	}
	for (i = 0; i < 5; i++) {
		if (g_core_fp.fp_register_read((pfw_op)->addr_icid_addr,  DATA_LEN_4,  tmp_data,  false) != 0) {
			ret_data = false;
			E("%s:fp_register_read Fail:\n", __func__);
			return ret_data;
		}
		I("%s:Read driver IC ID = %X, %X, %X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x11) && (tmp_data[1] == 0x3a)) {
			if (tmp_data[1] == 0x3a)
				strlcpy((private_ts)->chip_name,  HX_83113A_SERIES_PWON,  30);

			I("%s:IC name = %s\n", __func__, (private_ts)->chip_name);

			I("Himax IC package %x%x%x in\n",  tmp_data[3],  tmp_data[2],  tmp_data[1]);
			ret_data = true;
			goto FINAL;
		} else {
			ret_data = false;
			E("%s:Read driver ID register Fail:\n", __func__);
			E("Could NOT find Himax Chipset\n");
			E("Please check 1.VCCD,VCCA,VSP,VSN\n");
			E("2. LCM_RST,TP_RST\n");
			E("3. Power On Sequence\n");
			kfree(g_core_chip_dt);
		}
	}

FINAL:
	return ret_data;
}

static int himax_hx83113_probe(void)
{
	I("%s:Enter\n", __func__);

	if (g_core_chip_dt == NULL) {
		g_core_chip_dt = kzalloc(sizeof(struct himax_chip_detect) * HX_DRIVER_MAX_IC_NUM, GFP_KERNEL);
		if (!g_core_chip_dt)
			return -ENOMEM;
		I("%s:1st alloc g_core_chip_dt \n", __func__);
	}
	if (g_hx_ic_dt_num < HX_DRIVER_MAX_IC_NUM) {
		g_core_chip_dt[g_hx_ic_dt_num].fp_chip_detect = hx83113_chip_detect;
		g_hx_ic_dt_num++;
	}

	return 0;
}

static int himax_hx83113_remove(void)
{
	g_core_chip_dt[g_hx_ic_dt_num].fp_chip_detect = NULL;
	g_core_fp.fp_chip_init = NULL;
	return 0;
}

static int __init himax_hx83113_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83113_probe();
	return ret;
}

static void __exit himax_hx83113_exit(void)
{
	himax_hx83113_remove();
}

arch_initcall(himax_hx83113_init);
//module_init(himax_hx83113_init);
module_exit(himax_hx83113_exit);

MODULE_DESCRIPTION("HIMAX HX83113 touch driver");
MODULE_LICENSE("GPL");


