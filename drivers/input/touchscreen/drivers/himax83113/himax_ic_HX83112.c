/*  Himax Android Driver Sample Code for HX83112 chipset

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2,  as published by the Free Software Foundation,  and
    may be copied,  distributed,  and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#include "himax_ic_HX83112.h"
extern struct himax_ts_data *private_ts;
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

#ifdef HX_TP_PROC_2T2R
	extern bool Is_2T2R;
#endif

static void hx83112_chip_init (void)
{

	private_ts->chip_cell_type = CHIP_IS_IN_CELL;
	I ("%s:IC cell type = %d\n",  __func__,  private_ts->chip_cell_type);
	IC_CHECKSUM 			= HX_TP_BIN_CHECKSUM_CRC;
	/*Himax: Set FW and CFG Flash Address*/
	FW_VER_MAJ_FLASH_ADDR = 49157;  /*0x00C005*/
	FW_VER_MAJ_FLASH_LENG = 1;
	FW_VER_MIN_FLASH_ADDR = 49158;  /*0x00C006*/
	FW_VER_MIN_FLASH_LENG = 1;
	CFG_VER_MAJ_FLASH_ADDR = 49408;  /*0x00C100*/
	CFG_VER_MAJ_FLASH_LENG = 1;
	CFG_VER_MIN_FLASH_ADDR = 49409;  /*0x00C101*/
	CFG_VER_MIN_FLASH_LENG = 1;
	CID_VER_MAJ_FLASH_ADDR = 49154;  /*0x00C002*/
	CID_VER_MAJ_FLASH_LENG = 1;
	CID_VER_MIN_FLASH_ADDR = 49155;  /*0x00C003*/
	CID_VER_MIN_FLASH_LENG = 1;
}

static bool hx83112_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4] = {0};

	do {
		if (cnt == 0 || (tmp_data[0] != 0xA5 && tmp_data[0] != 0x00 && tmp_data[0] != 0x87)) {
		 g_core_fp.fp_register_write(pfw_op->addr_ctrl_fw_isr, DATA_LEN_4, pfw_op->data_fw_stop, 0);
		}
		udelay(1000);

		/* check fw status */
		g_core_fp.fp_register_read(pic_op->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);

		if (tmp_data[0] != 0x05) {
		 I("%s: Do not need wait FW, Status = 0x%02X!\n", __func__, tmp_data[0]);
		 break;
		}

		g_core_fp.fp_register_read(pfw_op->addr_ctrl_fw_isr, 4, tmp_data, false);
		I("%s: cnt = %d, data[0] = 0x%02X!\n", __func__, cnt, tmp_data[0]);
	} while (tmp_data[0] != 0x87 && (++cnt < 50) && check_en == true);

	cnt = 0;

	do {
		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_lb[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		/*===========================================
		 password[15:8] set Enter safe mode :0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_ub[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}
		if (private_ts->chip_name == HX_83112A_SERIES_PWON) {

			udelay(500);
			/*===========================================
			 password[7:0] set Enter safe mode : 0x31 ==> 0x00
			===========================================*/
			tmp_data[0] = 0x00;

			if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
				return false;
			}
			udelay(500);
			/*===========================================
			 password[7:0] set Enter safe mode : 0x31 ==> 0x27
			===========================================*/
			tmp_data[0] = pic_op->data_sonoff_psw_lb[0];

			if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
				return false;
			}

			/*===========================================
			 password[15:8] set Enter safe mode :0x32 ==> 0x95
			===========================================*/
			tmp_data[0] = pic_op->data_sonoff_psw_ub[0];

			if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
				return false;
			}

		}

		/* ======================
		 Check enter_save_mode
		 ======================*/
		g_core_fp.fp_register_read(pic_op->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);
		I("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			/*=====================================
			 Reset TCON
			=====================================*/
			g_core_fp.fp_register_write(pic_op->addr_tcon_on_rst, DATA_LEN_4, pic_op->data_rst, 0);
			msleep(1);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(pic_op->addr_tcon_on_rst, DATA_LEN_4, tmp_data, 0);
			/*=====================================
			 Reset ADC
			=====================================*/
			g_core_fp.fp_register_write(pic_op->addr_adc_on_rst, DATA_LEN_4, pic_op->data_rst, 0);
			msleep(1);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(pic_op->addr_adc_on_rst, DATA_LEN_4, tmp_data, 0);
			return true;
		} else {
			msleep(10);
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		}
	} while (cnt++ < 15);

	return false;
}

#define hx83112_ic_osc_en (0x900880A8)
#define hx83112_ic_osc_en_data (0x01)
#define hx83112_ic_osc_pw (0x900880E0)
#define hx83112_ic_osc_pw_data (0xA5)
#define hx83112_ic_eb_en (0x300EB000)
#define hx83112_ic_eb_en_data (0x00AA5500)
#ifdef HX_ESD_RECOVERY
static void himax_hx83112f_esd_dd_ic_reset(void)
{
	uint8_t addr[4] = {0};
	uint8_t data[4] = {0};

	I("%s: Entering\n", __func__);
	himax_in_parse_assign_cmd(0x9000001C, addr, sizeof(addr));
	himax_in_parse_assign_cmd(0x49, data, sizeof(data));
	g_core_fp.fp_register_write(addr, 4, data, 0);
	usleep_range(49000, 49000);
}
#endif

#if defined(HX_ZERO_FLASH)
static void himax_hx83112f_reload_to_active(void)
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
		I("%s: data[1]=%d, data[0]=%d, retry_cnt=%d\n", __func__,
				data[1], data[0], retry_cnt);
		retry_cnt++;
	} while ((data[1] != 0x01
		|| data[0] != 0xEC)
		&& retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_hx83112f_resume_ic_action(void)
{
#if !defined(HX_RESUME_HW_RESET)
	himax_hx83112f_reload_to_active();
#endif
}

static void himax_hx83112f_suspend_ic_action(void)
{
//#if !defined(HX_RESUME_HW_RESET)
//	himax_hx83112f_reload_to_active();
//#endif
}

static void himax_hx83112f_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;
	int ret = 0;

	I("Enter %s\n", __func__);

	g_core_fp.fp_interface_on();
	g_core_fp.fp_register_write(pfw_op->addr_ctrl_fw_isr,
		sizeof(pfw_op->data_clear), pfw_op->data_clear, 0);
	/*msleep(20);*/
	usleep_range(10000, 10001);
	if (!FlashMode) {
#if defined(HX_RST_PIN_FUNC)
		g_core_fp.fp_ic_reset(false, false);
#else
		g_core_fp.fp_system_reset();
#endif
		himax_hx83112f_reload_to_active();
	} else {
		himax_hx83112f_reload_to_active();
		do {
			g_core_fp.fp_register_write(
				pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->
				data_safe_mode_release_pw_active),
				pfw_op->data_safe_mode_release_pw_active,
				0);

			g_core_fp.fp_register_read(
				pfw_op->addr_flag_reset_event,
				DATA_LEN_4, tmp_data, 0);
			I("%s:Read status from IC = %X,%X\n", __func__,
					tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01
			|| tmp_data[0] != 0x00)
			&& retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#if defined(HX_RST_PIN_FUNC)
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
			himax_hx83112f_reload_to_active();
		} else {
			I("%s:OK and Read status from IC = %X,%X\n", __func__,
				tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			ret = himax_bus_write(
				pic_op->adr_sonoff_psw_lb[0],
				tmp_data, 1, HIMAX_XFER_RETRY_TIMES);
			if (ret < 0)
				E("%s: i2c access fail!\n", __func__);

				ret = himax_bus_write(
					pic_op->adr_sonoff_psw_ub[0],
					tmp_data, 1, HIMAX_XFER_RETRY_TIMES);
			if (ret < 0)
				E("%s: i2c access fail!\n", __func__);

			g_core_fp.fp_register_write(
				pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->
				data_safe_mode_release_pw_reset),
				pfw_op->data_safe_mode_release_pw_reset,
				0);
		}
	}
}

#endif
/* File node for SMWP and HSEN - End*/

void himax_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*I("%s: Entering!\n", __func__);*/

	switch (len) {
	case 1:
		cmd[0] = addr;
		/*I("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n",*/
		/*	__func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,*/
		/*cmd[2] = 0x%02X,cmd[3] = 0x%02X\n", */
		/* __func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		E("%s: input length fault,len = %d!\n", __func__, len);
	}
}


static void hx83112f_reg_re_init(void)
{
	himax_parse_assign_cmd(hx83112f_fw_addr_raw_out_sel,
		pfw_op->addr_raw_out_sel,
		sizeof(pfw_op->addr_raw_out_sel));
}

static void hx83112f_func_re_init(void)
{
#if defined(HX_ZERO_FLASH)
	g_core_fp.fp_resume_ic_action = himax_hx83112f_resume_ic_action;
	g_core_fp.fp_suspend_ic_action = himax_hx83112f_suspend_ic_action;
	g_core_fp.fp_sense_on = himax_hx83112f_sense_on;
	g_core_fp.fp_0f_reload_to_active = himax_hx83112f_reload_to_active;
#endif
#ifdef HX_ESD_RECOVERY
	g_core_fp._esd_ic_dd_reset = himax_hx83112f_esd_dd_ic_reset;
#endif
}

static void hx83112_func_re_init (void)
{
	g_core_fp.fp_sense_off = hx83112_sense_off;
	g_core_fp.fp_chip_init = hx83112_chip_init;
}

static void hx83112_reg_re_init (void)
{
}

static bool hx83112_chip_detect (void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	bool ret_data = false;
	int ret = 0;
	int i = 0;

	I("%s\n", __func__);
	private_ts->chip_name = HX_CHIP_NOT_DETECT;
	ret = himax_mcu_in_cmd_struct_init();
	if (ret < 0) {
		ret_data = false;
		E("%s:cmd_struct_init Fail:\n", __func__);
		return ret_data;
	}
	himax_mcu_in_cmd_init();

	hx83112_reg_re_init();
	hx83112_func_re_init();
	if (himax_bus_read((pic_op)->addr_conti[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return false;
	}

	if (g_core_fp.fp_sense_off(false) == false) {
		ret_data = false;
		E("%s:fp_sense_off Fail:\n", __func__);
		return ret_data;
	}
	for (i = 0; i < 5; i++) {
		if (g_core_fp.fp_register_read(pfw_op->addr_icid_addr,  DATA_LEN_4,  tmp_data,  false) != 0) {
			ret_data = false;
			E("%s:fp_register_read Fail:\n", __func__);
			return ret_data;
		}
		I("%s:Read driver IC ID = %X, %X, %X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x11) && ((tmp_data[1] == 0x2a) || (tmp_data[1] == 0x2b) || (tmp_data[1] == 0x2e) 
			|| (tmp_data[1] == 0x2f))) {
			if (tmp_data[1] == 0x2a) {
				private_ts->chip_name = HX_83112A_SERIES_PWON;
				I("0x%02X\n", private_ts->chip_name);
			} else if (tmp_data[1] == 0x2b) {
				private_ts->chip_name = HX_83112B_SERIES_PWON;
				I("0x%02X\n", private_ts->chip_name);
			} else if (tmp_data[1] == 0x2f) {
				private_ts->chip_name = HX_83112F_SERIES_PWON;
				I("0x%02X\n", private_ts->chip_name);
				hx83112f_reg_re_init();
				hx83112f_func_re_init();
			}

			I("%s:IC name = 0x%02X\n", __func__, private_ts->chip_name);
			I("Himax IC package %x%x%x in\n",  tmp_data[3],  tmp_data[2],  tmp_data[1]);
			ret_data = true;
			goto FINAL;
		} else {
			ret_data = false;
			E("%s:Read driver ID register Fail:\n", __func__);
			E("Could NOT find Himax Chipset \n");
			E("Please check 1.VCCD,VCCA,VSP,VSN \n");
			E("2. LCM_RST,TP_RST \n");
			E("3. Power On Sequence \n");
			kfree(g_core_chip_dt);
			g_core_chip_dt = NULL;
		}
	}
FINAL:
	return ret_data;
}

static int himax_hx83112_probe(void)
{
	I("%s:Enter\n", __func__);
	if (g_core_chip_dt == NULL) {
		g_core_chip_dt = kzalloc(sizeof(struct himax_chip_detect) * HX_DRIVER_MAX_IC_NUM, GFP_KERNEL);
		I("%s:1st alloc g_core_chip_dt \n", __func__);
		if (!g_core_chip_dt)
			return -ENOMEM;
	}
	if (g_hx_ic_dt_num < HX_DRIVER_MAX_IC_NUM) {
		g_core_chip_dt[g_hx_ic_dt_num].fp_chip_detect = hx83112_chip_detect;
		g_hx_ic_dt_num++;
	}

	return 0;

}

static int himax_hx83112_remove(void)
{
	g_core_chip_dt[g_hx_ic_dt_num].fp_chip_detect = NULL;
	g_core_fp.fp_chip_init = NULL;
	return 0;
}

int himax_hx83112_init(void)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = himax_hx83112_probe();
	return ret;
}

void himax_hx83112_exit(void)
{
	himax_hx83112_remove();
}
