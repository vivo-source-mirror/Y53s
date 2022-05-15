/*	Himax Android Driver Sample Code for debug nodes

	Copyright (C) 2018 Himax Corporation.

	This software is licensed under the terms of the GNU General Public
	License version 2, as published by the Free Software Foundation, and
	may be copied, distributed, and modified under those terms.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

*/


#include "himax_debug.h"
#include "himax_ic_core.h"

extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern struct himax_platform_data *hv_pdata;

extern struct himax_core_fp g_core_fp;
extern struct himax_debug *debug_data;
extern unsigned char	IC_CHECKSUM;
extern int xfer_error_count;
extern struct proc_dir_entry *himax_touch_proc_dir;
extern int mdss_dsi_panel_reset_and_powerctl(int enable);
extern bool g_DSRAM_Flag;//27

#ifdef HX_TP_PROC_GUEST_INFO
extern struct hx_guest_info *g_guest_info_data;
#endif

extern int himax_input_register(struct himax_ts_data *ts);
extern int bbk_himax_set_charger_bit(struct vts_device *vtsdev,int state);
extern int bbk_himax_gesture_point_get(uint16_t *data);


#ifdef HX_RST_PIN_FUNC
	extern void himax_rst_gpio_set(int pinnum, uint8_t value);//29
	extern void himax_ic_reset(uint8_t loadconfig, uint8_t int_off);
#endif

#if defined(HX_ZERO_FLASH)
extern char *i_CTPM_firmware_name;
#endif

uint8_t g_diag_arr_num;
int gest_pt_data[GEST_DATA_NUM];



extern uint8_t HX_PROC_SEND_FLAG;

int g_max_mutual;
int g_min_mutual;
int g_max_self;
int g_min_self;
s64 timeStart, timeEnd;

int g_switch_mode;

/* =============================================================================================================

	Segment : Himax PROC Debug Function

============================================================================================================= */
/*==============================================VIVO======================================================*/
#ifdef HX_VIVO_DEBUG_NODE
extern int g_f_0f_updat;
static int which_last = VTS_ST_NORMAL;

int idle_enable_or_disable(struct vts_device *vtsdev,int state)  /*#3 ++09-26++*/
{
	int retry = 10;
	uint8_t value[4] = {0};
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};

	I("%s:entering\n", __func__);

	do {
		I("%s,now %d times\n!", __func__, retry);
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x7f;
		tmp_addr[0] = 0xd4;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, 0);

		if (state == 0) {
			value[0] = 0xcd;
			value[1] = 0xcd;
			value[2] = 0xcd;
			value[3] = 0xcd;
		} else {
			value[0] = 0x00;
			value[1] = 0x00;
			value[2] = 0x00;
			value[3] = 0x00;
		}

		I(" tmp_data[0] = 0x%2X, value[0] = 0x%2X\n", tmp_data[0], state);

		g_core_fp.fp_flash_write_burst_lenth(tmp_addr, value, 4);

		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, 0);

		I("%s:After turn ON/OFF IDLE Mode [0] = 0x%02X,[1] = 0x%02X,\
				[2] = 0x%02X,[3] = 0x%02X\n", __func__, tmp_data[0],
				tmp_data[1], tmp_data[2], tmp_data[3]);

		retry--;
		msleep(10);

	} while ((tmp_data[0] != value[0]) && retry > 0);

	if ((tmp_data[0] == value[0]) && retry > 0)
		return 0;
	else
		return MINUS_ONE;
}

/*0 is disable and 1 is enable*/
static void enable_or_disable_irq_wake(int enable)
{
	static int state_of_irq = 2;
	if (state_of_irq == enable) {
		return;
	}
	if (enable) {
		enable_irq_wake(private_ts->hx_irq);
		VTI("enable_irq_wake");
	} else {
		disable_irq_wake(private_ts->hx_irq);
		VTI("disable_irq_wake");
	}
	state_of_irq = enable;
	return;
}


int bbk_himax_mode_change(struct vts_device *vtsdev,int which) /*#5*/
{
	int status = 0;
	struct himax_ts_data *ts = NULL;
	struct himax_platform_data *pdata=hv_pdata;
	int times = 3;
	ts = private_ts;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	if (which == VTS_ST_NORMAL) {
		VTI("change to normal mode");
		//himax_rst_gpio_set(private_ts->rst_gpio, 1);
#if defined(CONFIG_MEDIATEK_SOLUTION)
		if (private_ts->vddi_poweroff && !IS_ERR_OR_NULL(private_ts->spi_cs_active))
			pinctrl_select_state(private_ts->pinctrl, private_ts->spi_cs_active);
#endif
		if (gpio_is_valid(pdata->gpio_cs)) {
			gpio_set_value(pdata->gpio_cs, 1);
		}
		if (gpio_is_valid(pdata->gpio_reset)) {
			gpio_set_value(pdata->gpio_reset, 1);
		}
		status = himax_chip_common_resume(ts);
		enable_or_disable_irq_wake(0);
	}

	if (which == VTS_ST_GESTURE) {
		VTI("change to gesture mode");
		if (which_last == VTS_ST_SLEEP) {
			ts->suspended = false;
#if defined(CONFIG_MEDIATEK_SOLUTION)
			if (private_ts->vddi_poweroff && !IS_ERR_OR_NULL(private_ts->spi_cs_active))
				pinctrl_select_state(private_ts->pinctrl, private_ts->spi_cs_active);
#endif
			vts_dsi_panel_reset_power_ctrl(1);
			if (gpio_is_valid(pdata->gpio_cs)) {
				gpio_set_value(pdata->gpio_cs, 1);
			}
			if (gpio_is_valid(pdata->gpio_reset)) {
				gpio_set_value(pdata->gpio_reset, 1);
			}
			//himax_rst_gpio_set(private_ts->rst_gpio, 1);//control tp_rst pin to high//29
		retry:
			g_core_fp.fp_0f_operation_dirly();			
			g_core_fp.fp_reload_disable(0);
			g_core_fp.fp_reload_fw_clear_register();
			g_core_fp.fp_sense_on(0x00);
			usleep_range(15000, 15001);
			if (g_core_fp.fp_check_remapping()) {
				if (!times) {
					return status;
				}
				times--;
				goto retry;
			}
			himax_int_enable (1);
		}
		ts->SMWP_enable = 0x01;
		g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, 0);
		status = himax_chip_common_suspend(ts);
		enable_or_disable_irq_wake(1);
		I("SMWP1=%d\n", ts->SMWP_enable);
	}

	if (which == VTS_ST_SLEEP) {
		VTI("change to sleep mode");
		ts->SMWP_enable = 0x00;
		/*himax_set_SMWP_enable(ts->client, ts->SMWP_enable, 0);*/
		if (which_last == VTS_ST_GESTURE)
			ts->suspended = false;
		status = himax_chip_common_suspend(ts);
		I("SMWP2=%d\n", ts->SMWP_enable);
#if defined(CONFIG_MEDIATEK_SOLUTION)
		if (private_ts->vddi_poweroff && !IS_ERR_OR_NULL(private_ts->spi_cs_sleep_pulllow))
			pinctrl_select_state(private_ts->pinctrl, private_ts->spi_cs_sleep_pulllow);
#endif
		//himax_rst_gpio_set(private_ts->rst_gpio, 0);//control tp_rst pin to low//29
		if (gpio_is_valid(pdata->gpio_cs)) {
			gpio_set_value(pdata->gpio_cs, 0);
		}
		if (gpio_is_valid(pdata->gpio_reset)) {
			gpio_set_value(pdata->gpio_reset, 0);
		}
		vts_dsi_panel_reset_power_ctrl(0);
		mdelay(20);
		enable_or_disable_irq_wake(0);
	}
	which_last = which;


	return status;
}

int bbk_himax_get_fw_version(struct vts_device *vtsdev, u64 *version) /*#6*/
{
	struct himax_ts_data *ts = private_ts;
	uint8_t cmd[4] = {0};
	uint8_t data[4] = {0};
	uint8_t data_2[4] = {0};
	int value1 = 0;
	int value2 = 0;
	int retry = 3;
	int reload_status = 0;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	if (!ts->have_update_firmware) {
		*version = 0;
		VTI("should update firmware first, set version 0 !");
		return 0;
	}

	while (reload_status == 0 && retry != 0) {
		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x7f;
		cmd[0] = 0x00;
		g_core_fp.fp_register_read(cmd, 4, data, 0);

		cmd[3] = 0x10;
		cmd[2] = 0x00;
		cmd[1] = 0x72;
		cmd[0] = 0xc0;
		g_core_fp.fp_register_read(cmd, 4, data_2, 0);

		if ((data[2] == 0x9a && data[3] == 0xa9)
			|| (data_2[1] == 0x72 && data_2[0] == 0xC0)) {
			I("reload OK! \n");
			reload_status = 1;
			break;
		} else {
			retry--;
			msleep(10);
			I("reload fail ,delay 10ms retry=%d\n", retry);
		}
	}
	I("%s : data[2]=0x%2.2X,data[3]=0x%2.2X,data_2[0]=0x%2.2X,data_2[1]=0x%2.2X\n"
	, __func__, data[2], data[3], data_2[0], data_2[1]);
	I("reload_status=%d\n", reload_status);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x84;

	data[3] = 0xff;
	data[2] = 0xff;
	data[1] = 0xff;
	data[0] = 0xff;
	g_core_fp.fp_register_read(cmd, 4, data, 0);

	if ((data[2] == 0xff) && (data[3] == 0xff)) {
		value1 = MINUS_ONE;
	} else {
		value1 = data[1] << 8 | data[2];
	}
	*version =(u64)value1;
	I("firmware version: %X\n", value1);

	cmd[3] = 0x10;
	cmd[2] = 0x00;
	cmd[1] = 0x70;
	cmd[0] = 0x04;

	data[3] = 0xff;
	data[2] = 0xff;
	data[1] = 0xff;
	data[0] = 0xff;
	g_core_fp.fp_register_read(cmd, 4, data, 0);

	if ((data[2] == 0xff) && (data[3] == 0xff)) {
		value2 = MINUS_ONE;
	} else {
		value2 = data[1] << 8 | data[2];
	}

	I("config version is %X", value2);
	return 0;
}

int set_edge_restain_switch(struct vts_device *vtsdev,int on) /*#10*/
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t back_data[4] = {0};
	uint8_t retry_cnt = 0;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	/* Enable:0x10007F3C = 0xA55AA55A */
	do {
		if (on == 1) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x3C;

			tmp_data[3] = 0xA5;
			tmp_data[2] = 0x5A;
			tmp_data[1] = 0xA5;
			tmp_data[0] = 0x5A;
			g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			back_data[3] = 0xA5;
			back_data[2] = 0x5A;
			back_data[1] = 0xA5;
			back_data[0] = 0x5A;
		} else if (on == 0) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x3C;

			tmp_data[3] = 0xA3;
			tmp_data[2] = 0x3A;
			tmp_data[1] = 0xA3;
			tmp_data[0] = 0x3A;
			g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			back_data[3] = 0xA3;
			back_data[2] = 0x3A;
			back_data[1] = 0xA3;
			back_data[0] = 0x3A;
		} else if (on == 2) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x7F;
			tmp_addr[0] = 0x3C;

			tmp_data[3] = 0xA1;
			tmp_data[2] = 0x1A;
			tmp_data[1] = 0xA1;
			tmp_data[0] = 0x1A;
			g_core_fp.fp_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			back_data[3] = 0xA1;
			back_data[2] = 0x1A;
			back_data[1] = 0xA1;
			back_data[0] = 0x1A;
		}
		msleep (50);
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, 0);

		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2]
			|| tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0])
			&& retry_cnt < HIMAX_REG_RETRY_TIMES);

	if (retry_cnt < HIMAX_REG_RETRY_TIMES) {
		return 0;
	} else {
		return MINUS_ONE;
	}
}

int bbk_himax_set_gesture(struct vts_device *vtsdev, int enable) {
	int gesture_state = vts_state_get(vtsdev, VTS_STA_GESTURE);
	uint8_t addr[4];
	uint8_t data[4];
	
	addr[3] = 0x10;
	addr[2] = 0x00;
	addr[1] = 0x7f;
	addr[0] = 0x24;

	data[3] = 0x00;
	data[2] = 0x00;
	data[1] = gesture_state >> 8;
	data[0] = gesture_state & 0xff;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	I("current geature state is 0x%x    0x%x    0x%x", gesture_state, data[1], data[0]);
	g_core_fp.fp_register_write(addr, DATA_LEN_4, data, 0);
	
	return 0;
}

int bbk_himax_set_idle(struct vts_device *vtsdev, int enable) {
	u32 idle_time = 0;
	int retry = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t rec_data[DATA_LEN_4] = {0};
	uint32_t cmd = 0;
	I("set game mode with value %d", enable);
	if(enable == 0){
		vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);
		if(idle_time == 0) {
			cmd = 0xCDCDCDCD;
			do {
				himax_in_parse_assign_cmd(0x10007fd4, tmp_addr, sizeof(tmp_addr));
				himax_in_parse_assign_cmd(cmd, tmp_data, sizeof(tmp_data));
				g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
				usleep_range(1000, 1001);

				g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
				I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
					rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
			} while((retry++ < 10) && (rec_data[3] != 0x00 && rec_data[2] != 0x00 && rec_data[1] != 0x00 && rec_data[0] != 0x00 ));
			
		} else if(idle_time <= 0xff) {	
			cmd = 0xA55A0000 | idle_time;
			do {
				himax_in_parse_assign_cmd(0x10007fd4, tmp_addr, sizeof(tmp_addr));
				himax_in_parse_assign_cmd(cmd, tmp_data, sizeof(tmp_data));
				g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
				usleep_range(1000, 1001);

				g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
				I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
					rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
			} while((retry++ < 10) && (rec_data[3] != 0x00 && rec_data[2] != 0x00 && rec_data[1] != 0x00 && rec_data[0] != 0x00 ));
		} else {
			E("idle time is not valid !");
		}
		
	} else {
		do {
			himax_in_parse_assign_cmd(0x10007fd4, tmp_addr, sizeof(tmp_addr));
			himax_in_parse_assign_cmd(0x00000000, tmp_data, sizeof(tmp_data));
			g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
			usleep_range(1000, 1001);
		
			g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
			I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
				rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
		} while((retry++ < 10) && (rec_data[3] != 0x00 && rec_data[2] != 0x00 && rec_data[1] != 0x00 && rec_data[0] != 0x00 ));
	}

	return 0;

}

int bbk_himax_get_touch_ic_mdoe(struct vts_device *vtsdev)
{
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t rec_data[DATA_LEN_4] = {0};
	uint32_t result = 0;

	himax_in_parse_assign_cmd(0x10007fd0, tmp_addr, sizeof(tmp_addr));
	g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
	VTI("data=0x%02X%02X%02X%02X\n", rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
	result |= (uint32_t)rec_data[0];
	result |= (uint32_t)rec_data[1] << 8;
	result |= (uint32_t)rec_data[2] << 16;
	result |= (uint32_t)rec_data[3] << 24;
	VTI("read from 0x10007fd0 is 0x%x", result);
	if (result == 0xA55AA55A)
		return 0;
	else if(result == 0xA33AA33A)
		return 1;
	else
		return -1;
}

extern struct himax_report_data *hx_touch_data;
int bbk_himax_get_fw_debug_info(unsigned char *buf) /*#12 ++09-26++*/
{
	size_t ret = 0;
	size_t len = 1024;

	ret += snprintf(buf + ret, len - ret, "ReCal = %d\t", hx_touch_data->hx_state_info[0] & 0x03);
	ret += snprintf(buf + ret, len - ret, "Palm = %d\t", hx_touch_data->hx_state_info[0] >> 3 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Water = %d\n", hx_touch_data->hx_state_info[0] >> 5 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Glove = %d\t", hx_touch_data->hx_state_info[1] & 0x01);
	ret += snprintf(buf + ret, len - ret, "TX Hop = %d\t", hx_touch_data->hx_state_info[0] >> 6 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Base Line = %d\t", hx_touch_data->hx_state_info[0] >> 2 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Noise mode = %d\n", hx_touch_data->hx_state_info[0] >> 7 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Border = %d\t", hx_touch_data->hx_state_info[1] >> 1 & 0x01);
	ret += snprintf(buf + ret, len - ret, "VR = %d\t", hx_touch_data->hx_state_info[1] >> 2 & 0x01);
	ret += snprintf(buf + ret, len - ret, "One Block = %d\t", hx_touch_data->hx_state_info[1] >> 4 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Blewing = %d\n", hx_touch_data->hx_state_info[1] >> 5 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Thumb flying = %d\t", hx_touch_data->hx_state_info[1] >> 6 & 0x01);
	ret += snprintf(buf + ret, len - ret, "Border extend = %d\n", hx_touch_data->hx_state_info[1] >> 7 & 0x01);

	return ret;
}
/* VIVO start */
int bbbk_himax_get_rawordiff_data(int which, int *data) /*#1 ++09-26++*/
{

	int32_t *mutual_data = NULL;
	int32_t *mutual_data_buf = NULL;
	uint8_t command = 0x00;
	uint8_t storage_type = 0x00;
	int i;
	mutual_data_buf = kzalloc(ic_data->HX_RX_NUM * ic_data->HX_TX_NUM
			* sizeof(int32_t), GFP_KERNEL);
	if (!mutual_data_buf)
		return -ENOMEM;

	if (which == 0) {
		/* read rawdata */
		private_ts->diag_cmd = 2;
		command = 0x02;
	} else if (which == 1) {
		/* read delta/diff data */
		private_ts->diag_cmd = 1;
		command = 0x01;
	} else if (which == 2) {
		private_ts->diag_cmd = 3;
		command = 0x03;

	}
	storage_type = g_core_fp.fp_determin_diag_storage(private_ts->diag_cmd);

	g_core_fp.fp_diag_register_set(command, storage_type);
	msleep(650);
	command = 0x00;
	g_core_fp.fp_diag_register_set(command, storage_type);
	msleep(20);
	private_ts->diag_cmd = 0;

	/* 1. Copy rawdata from global buffer */
	mutual_data = getMutualBuffer();
	memcpy(mutual_data_buf, mutual_data, ic_data->HX_RX_NUM
			* ic_data->HX_TX_NUM * sizeof(int32_t));

	/* 2. Transfer and copy rawdata to vivo system */
	for (i = 0 ; i < ic_data->HX_RX_NUM * ic_data->HX_TX_NUM; i++) {
		data[i] = mutual_data_buf[i];
		I("data[%d] = %d", i, data[i]);
		if ((i%8) == 0)
			I("\n");
	}

	kfree(mutual_data_buf);
	mutual_data_buf = NULL;
	return 0;
}
static ssize_t himax_get_rawordiff_data(struct file *file, char *buf,
										size_t len, loff_t *pos)  /*#1*/
{
	ssize_t ret = 0;
	int i = 0;
	int *data = NULL;
	char *temp_buf = NULL;
	if (!HX_PROC_SEND_FLAG) {
		data = kzalloc(ic_data->HX_RX_NUM * ic_data->HX_TX_NUM
			* sizeof(int32_t), GFP_KERNEL);
		if (!data) {
			ret = -1;
			goto null_data;
		}
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf) {
			ret = -1;
			goto null_temp;
		}

		msleep(20);
		bbbk_himax_get_rawordiff_data(0, data);
		for (i = 0 ; i < ic_data->HX_RX_NUM * ic_data->HX_TX_NUM; i++) {
			if (i % ic_data->HX_RX_NUM == 0)
				ret += snprintf(&temp_buf[ret], 255, "\n");
			ret += snprintf(&temp_buf[ret], 255, "%5d ", (signed short)data[i]);
		}
		ret += snprintf(&temp_buf[ret], 255, "\n");
		if (copy_to_user(buf, temp_buf, len)) {
			I("%s,here:%d\n", __func__, __LINE__);
		}
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	kfree(temp_buf);
	temp_buf = NULL;
null_temp:
	kfree(data);
	data = NULL;
null_data:
	return ret;
}

static ssize_t himax_check_debug_info_read(struct file *file, char *buf, size_t len, loff_t *pos)

{
	ssize_t ret = 0;
	char *temp_buf = NULL;
	char *ret_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {

		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf) {
			ret = -1;
			goto null_temp;
		}
		ret_buf = kzalloc(1024, GFP_KERNEL);
		if (!ret_buf) {
			ret = -1;
			goto null_ret;
		}
		msleep(20);
		bbk_himax_get_fw_debug_info(ret_buf);
		I("ret_buf = %s", ret_buf);
		ret += snprintf(temp_buf + ret, len - ret, ret_buf);


		if (copy_to_user(buf, temp_buf, len)) {
			I("%s,here:%d\n", __func__, __LINE__);
		}		

		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
		goto null_temp;
	}

	kfree(ret_buf);
	ret_buf = NULL;
null_ret:
	kfree(temp_buf);
	temp_buf = NULL;
null_temp:
	return ret;
}
extern struct himax_ts_data *private_ts;
static ssize_t himax_check_debug_info_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf[80] = {0};
	u64 version;
	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len)) {
		return -EFAULT;
	}

	if (buf[0] == 'e') {
		if (buf[1] == '0') {
			set_edge_restain_switch(private_ts->vtsdev,0x00);
			I("run mode 0\n");
		} else if (buf[1] == '1') {
			set_edge_restain_switch(private_ts->vtsdev,0x01);
			I("run mode 1\n");
		} else if (buf[1] == '2') {
			set_edge_restain_switch(private_ts->vtsdev,0x02);
			I("run mode 2\n");
		} else {
			I("Do nothing\n");
		}
	} else if (buf[0] == 'h') {
		if (buf[1] == '0') {
			bbk_himax_get_header_file_version(0);
			I("run bbk_himax_get_header_file_version(0)\n");
		} else if (buf[1] == '1') {
			bbk_himax_get_header_file_version(1);
			I("run bbk_himax_get_header_file_version(1)\n");
		} else {
			I("Do nothing\n");
		}
	} else if (buf[0] == 'f') {
		if (buf[1] == '0') {
			bbk_himax_get_fw_version(private_ts->vtsdev,&version);
			I("run get_fw_version(0)\n");
		} else if (buf[1] == '1') {
			bbk_himax_get_fw_version(private_ts->vtsdev,&version);
			I("run get_fw_version(1)\n");
		} else {
			I("Do nothing\n");
		} 
	}else if (buf[0] == 'm') {
		if (buf[1] == '1') {
			bbk_himax_mode_change(private_ts->vtsdev,VTS_ST_NORMAL);
			I("run TOUCHSCREEN_NORMAL\n");
		} else if (buf[1] == '2') {
			bbk_himax_mode_change(private_ts->vtsdev,VTS_ST_GESTURE);
			I("run TOUCHSCREEN_GESTURE\n");
		} else if (buf[1] == '3') {
			bbk_himax_mode_change(private_ts->vtsdev,VTS_ST_SLEEP);
			I("run TOUCHSCREEN_SLEEP\n");
		}
	} else if (buf[0] == 'i') {
		if (buf[1] == '0') {
			idle_enable_or_disable(private_ts->vtsdev,0x00);
			I("run disable idle\n");
		} else if (buf[1] == '1') {
			idle_enable_or_disable(private_ts->vtsdev,0x01);
			I("run enable idle\n");
		} else {
			I("Do nothing\n");
		}
	} 
	else if (buf[0] == 'p') {
		if (buf[1] == '0') {
			bbk_himax_set_charger_bit(private_ts->vtsdev,0x00);
			I("run charger disable\n");
		} else if (buf[1] == '1') {
			bbk_himax_set_charger_bit(private_ts->vtsdev,0x01);
			I("run charger enable\n");
		} else {
			I("Do nothing\n");
		}
	} 
	else if (buf[0] == 'w') {
		bbk_himax_gesture_point_get((uint16_t *) gest_pt_data);
		I("run gesture enable\n");
	}

	return len;
}
#endif

/*==============================================VIVO======================================================*/

static ssize_t himax_CRC_test_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	uint8_t result = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if(!temp_buf){
			return -1;
		}
		g_core_fp.fp_sense_off(true);
		msleep(20);
		result = g_core_fp.fp_calculateChecksum(false);
		g_core_fp.fp_sense_on(0x01);

		if (result) {
			ret += snprintf(temp_buf + ret, len - ret, "CRC test is Pass! \n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "CRC test is Fail! \n");
		}

		if (copy_to_user(buf, temp_buf, len)) {
			I("%s,here:%d\n", __func__, __LINE__);
		}

		kfree(temp_buf);
		temp_buf = NULL;
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_vendor_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	char *temp_buf = NULL;
	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf) 
			return -ENOMEM;
		ret += snprintf(temp_buf + ret, len - ret, "FW_VER = 0x%2.2X \n", ic_data->vendor_fw_ver);

		if (private_ts->chip_cell_type == CHIP_IS_ON_CELL) {
			ret += snprintf(temp_buf + ret, len - ret, "CONFIG_VER = 0x%2.2X \n", ic_data->vendor_config_ver);
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "TOUCH_VER = 0x%2.2X \n", ic_data->vendor_touch_cfg_ver);
			ret += snprintf(temp_buf + ret, len - ret, "DISPLAY_VER = 0x%2.2X \n", ic_data->vendor_display_cfg_ver);
		}

		if (ic_data->vendor_cid_maj_ver < 0 && ic_data->vendor_cid_min_ver < 0) {
			ret += snprintf(temp_buf + ret, len - ret, "CID_VER = NULL\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "CID_VER = 0x%2.2X \n", (ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));
		}

		if (ic_data->vendor_panel_ver < 0) {
			ret += snprintf(temp_buf + ret, len - ret, "PANEL_VER = NULL\n");
		} else {
			ret += snprintf(temp_buf + ret, len - ret, "PANEL_VER = 0x%2.2X \n", ic_data->vendor_panel_ver);
		}
		if (private_ts->chip_cell_type == CHIP_IS_IN_CELL) {
			ret += snprintf(temp_buf + ret, len - ret, "Cusomer = %s \n", ic_data->vendor_cus_info);
			ret += snprintf(temp_buf + ret, len - ret, "Project = %s \n", ic_data->vendor_proj_info);
		}
		ret += snprintf(temp_buf + ret, len - ret, "\n");
		ret += snprintf(temp_buf + ret, len - ret, "Himax Touch Driver Version:\n");
		ret += snprintf(temp_buf + ret, len - ret, "%s \n", HIMAX_DRIVER_VER);
		HX_PROC_SEND_FLAG = 1;

		if (copy_to_user(buf, temp_buf, len)) {
			I("%s,here:%d\n", __func__, __LINE__);
		}

		kfree(temp_buf);
		temp_buf = NULL;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}
									
static ssize_t himax_attn_read(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	char *temp_buf = NULL;
	ts_data = private_ts;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf)
			return -1;
		ret += snprintf(temp_buf + ret, len - ret, "attn = %x\n", himax_int_gpio_read(ts_data->pdata->gpio_irq));

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		temp_buf = NULL;
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_int_en_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf)
			return -ENOMEM;
		ret += snprintf(temp_buf + ret, len - ret, "%d ", ts->irq_enabled);
		ret += snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_int_en_write(struct file *file, const char *buff,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[12] = {0};
	int value = 0;

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len)) {
		return -EFAULT;
	}

	if (buf_tmp[0] == '0') {
		value = false;
	} else if (buf_tmp[0] == '1') {
		value = true;
	} else {
		return -EINVAL;
	}
	if (value) {
		ts->irq_enabled = 1;
		himax_int_enable(1);
	} else {
		himax_int_enable(0);
		ts->irq_enabled = 0;
	}

	return len;
}

static ssize_t himax_layout_read(struct file *file, char *buf,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t ret = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf) 
			return -1;
		ret += snprintf(temp_buf + ret, len - ret, "%d ", ts->pdata->abs_x_min);
		ret += snprintf(temp_buf + ret, len - ret, "%d ", ts->pdata->abs_x_max);
		ret += snprintf(temp_buf + ret, len - ret, "%d ", ts->pdata->abs_y_min);
		ret += snprintf(temp_buf + ret, len - ret, "%d ", ts->pdata->abs_y_max);
		ret += snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		temp_buf = NULL;
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_layout_write(struct file *file, const char *buff,
									size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[5] = {0};
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;


	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));

			if (i - j <= 5) {
				memcpy(buf_tmp, buf + j, i - j);
			} else {
				I("buffer size is over 5 char\n");
				return len;
			}

			j = i + 1;

			if (k < 4) {
				ret = kstrtoul(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}

	if (k == 4) {
		ts->pdata->abs_x_min = layout[0];
		ts->pdata->abs_x_max = (layout[1] - 1);
		ts->pdata->abs_y_min = layout[2];
		ts->pdata->abs_y_max = (layout[3] - 1);
		I("%d, %d, %d, %d\n",
		  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		input_unregister_device(ts->input_dev);
		himax_input_register(ts);
	} else {
		I("ERR@%d, %d, %d, %d\n",
		  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	}

	return len;
}

static ssize_t himax_debug_level_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	struct himax_ts_data *ts_data = NULL;
	size_t ret = 0;
	char *temp_buf = NULL;
	ts_data = private_ts;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf)
			return -ENOMEM;
		ret += snprintf(temp_buf + ret, len - ret, "%d\n", ts_data->debug_log_level);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_debug_level_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = NULL;
	char buf_tmp[11] = {0};
	int i;
	ts = private_ts;

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len))
		return -EFAULT;

	ts->debug_log_level = 0;

	for (i = 0; i < len - 1; i++) {
		if (buf_tmp[i] >= '0' && buf_tmp[i] <= '9')
			ts->debug_log_level |= (buf_tmp[i] - '0');
		else if (buf_tmp[i] >= 'A' && buf_tmp[i] <= 'F')
			ts->debug_log_level |= (buf_tmp[i] - 'A' + 10);
		else if (buf_tmp[i] >= 'a' && buf_tmp[i] <= 'f')
			ts->debug_log_level |= (buf_tmp[i] - 'a' + 10);

		if (i != len - 2)
			ts->debug_log_level <<= 4;
	}

	if (ts->debug_log_level & BIT(4)) {
		I("Turn on/Enable Debug Mode for Inspection!\n");
		goto END_FUNC;
	}

	if (ts->debug_log_level & BIT(3)) {
		if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
			(ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
			(ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
			ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS) / (ts->pdata->abs_x_max - ts->pdata->abs_x_min);
			ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS) / (ts->pdata->abs_y_max - ts->pdata->abs_y_min);

			if (ts->widthFactor > 0 && ts->heightFactor > 0) {
				ts->useScreenRes = 1;
			} else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else {
			I("Enable finger debug with raw position mode!\n");
		}
	} else {
		ts->useScreenRes = 0;
		ts->widthFactor = 0;
		ts->heightFactor = 0;
	}
END_FUNC:
	return len;
}

static ssize_t himax_proc_register_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	int ret = 0;
	uint16_t loop_i;
	uint8_t *data = NULL;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		data = kzalloc(128 * sizeof(uint8_t), GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;
			goto null_data;
		}
		temp_buf = kzalloc(len, GFP_KERNEL);
		if (!temp_buf) {
			ret = -ENOMEM;
			goto null_temp;
		}
		I("himax_register_show: %02X,%02X,%02X,%02X\n", register_command[3], register_command[2], register_command[1], register_command[0]);
		g_core_fp.fp_register_read(register_command, 128, data, cfg_flag);
		ret += snprintf(temp_buf + ret, len - ret, "command:  %02X,%02X,%02X,%02X\n", register_command[3], register_command[2], register_command[1], register_command[0]);

		for (loop_i = 0; loop_i < 128; loop_i++) {
			ret += snprintf(temp_buf + ret, len - ret, "0x%2.2X ", data[loop_i]);
			if ((loop_i % 16) == 15)
				ret += snprintf(temp_buf + ret, len - ret, "\n");
		}

		ret += snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	kfree(temp_buf);
null_temp:
	kfree(data);
null_data:
	return ret;
}

static ssize_t himax_proc_register_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	char buf[80] = {0};
	char buf_tmp[16];
	uint8_t length = 0;
	unsigned long result	= 0;
	uint8_t loop_i			= 0;
	uint16_t base			= 2;
	char *data_str = NULL;
	uint8_t w_data[20];
	uint8_t x_pos[20];
	uint8_t count = 0;

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(w_data, 0x0, sizeof(w_data));
	memset(x_pos, 0x0, sizeof(x_pos));
	memset(register_command, 0x0, sizeof(register_command));

	I("himax %s\n", buf);

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' && buf[2] == 'x') {
		length = strlen(buf);

		/* I("%s: length = %d.\n", __func__,length); */
		for (loop_i = 0; loop_i < length; loop_i++) { /* find postion of 'x' */
			if (buf[loop_i] == 'x') {
				x_pos[count] = loop_i;
				count++;
			}
		}

		data_str = strrchr(buf, 'x');
		I("%s: %s.\n", __func__, data_str);
		length = strlen(data_str + 1) - 1;

		if (buf[0] == 'r') {
			if (buf[3] == 'F' && buf[4] == 'E' && length == 4) {
				length = length - base;
				cfg_flag = 1;
				memcpy(buf_tmp, data_str + base + 1, length);
			} else {
				cfg_flag = 0;
				memcpy(buf_tmp, data_str + 1, length);
			}

			byte_length = length / 2;

			if (!kstrtoul(buf_tmp, 16, &result)) {
				for (loop_i = 0 ; loop_i < byte_length ; loop_i++)
					register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
			}

			if ((private_ts->chip_name == HX_85XX_H_SERIES_PWON) && cfg_flag == 0)
				cfg_flag = 2;
		} else if (buf[0] == 'w') {
			if (buf[3] == 'F' && buf[4] == 'E') {
				cfg_flag = 1;
				memcpy(buf_tmp, buf + base + 3, length);
			} else {
				cfg_flag = 0;
				memcpy(buf_tmp, buf + 3, length);
			}

			if (count < 3) {
				byte_length = length / 2;

				if (!kstrtoul(buf_tmp, 16, &result)) { /* command */
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++)
						register_command[loop_i] = (uint8_t)(result >> loop_i * 8);
				}

				if (!kstrtoul(data_str + 1, 16, &result)) { /* data */
					for (loop_i = 0 ; loop_i < byte_length ; loop_i++)
						w_data[loop_i] = (uint8_t)(result >> loop_i * 8);
				}

				g_core_fp.fp_register_write(register_command, byte_length, w_data, cfg_flag);
			} else {
				for (loop_i = 0; loop_i < count; loop_i++) { /* parsing addr after 'x' */
					memset(buf_tmp, 0x0, sizeof(buf_tmp));
					if (cfg_flag != 0 && loop_i != 0)
						byte_length = 2;
					else
						byte_length = x_pos[1] - x_pos[0] - 2; /* original */

					memcpy(buf_tmp, buf + x_pos[loop_i] + 1, byte_length);

					/* I("%s: buf_tmp = %s\n", __func__,buf_tmp); */
					if (!kstrtoul(buf_tmp, 16, &result)) {
						if (loop_i == 0) {
							register_command[loop_i] = (uint8_t)(result);
							/* I("%s: register_command = %X\n", __func__,register_command[0]); */
						} else {
							w_data[loop_i - 1] = (uint8_t)(result);
							/* I("%s: w_data[%d] = %2X\n", __func__,loop_i - 1,w_data[loop_i - 1]); */
						}
					}
				}

				byte_length = count - 1;
				if ((private_ts->chip_name == HX_85XX_H_SERIES_PWON) && cfg_flag == 0)
					cfg_flag = 2;
				g_core_fp.fp_register_write(register_command, byte_length, &w_data[0], cfg_flag);
			}
		} else {
			return len;
		}
	}

	return len;
}

int32_t *getMutualBuffer(void)
{
	return diag_mutual;
}
int32_t *getMutualNewBuffer(void)
{
	return diag_mutual_new;
}
int32_t *getMutualOldBuffer(void)
{
	return diag_mutual_old;
}
int32_t *getSelfBuffer(void)
{
	return &diag_self[0];
}
int32_t *getSelfNewBuffer(void)
{
	return &diag_self_new[0];
}
int32_t *getSelfOldBuffer(void)
{
	return &diag_self_old[0];
}
void setMutualBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualNewBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_new = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}
void setMutualOldBuffer(uint8_t x_num, uint8_t y_num)
{
	diag_mutual_old = kzalloc(x_num * y_num * sizeof(int32_t), GFP_KERNEL);
}

int himax_set_diag_cmd(struct himax_ic_data *ic_data, struct himax_report_data *hx_touch_data)
{
	struct himax_ts_data *ts = private_ts;
	int32_t *mutual_data;
	int32_t *self_data;
	int mul_num = 0;
	int self_num = 0;
	/* int RawDataLen = 0; */
	hx_touch_data->diag_cmd = ts->diag_cmd;

	if (hx_touch_data->diag_cmd >= 1 && hx_touch_data->diag_cmd <= 7) {
		/* Check event stack CRC */
		if (!g_core_fp.fp_diag_check_sum(hx_touch_data))
			goto bypass_checksum_failed_packet;

		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();
		/*	initiallize the block number of mutual and self */
		mul_num = ic_data->HX_RX_NUM * ic_data->HX_TX_NUM;
		self_num = ic_data->HX_RX_NUM + ic_data->HX_TX_NUM;
		g_core_fp.fp_diag_parse_raw_data(hx_touch_data, mul_num, self_num, hx_touch_data->diag_cmd, mutual_data, self_data);
	} else if (hx_touch_data->diag_cmd == 8) {
		memset(diag_coor, 0x00, sizeof(diag_coor));
		memcpy(&(diag_coor[0]), &hx_touch_data->hx_coord_buf[0], hx_touch_data->touch_info_size);
	}

	/* assign state info data */
	memcpy(&(hx_state_info[0]), &hx_touch_data->hx_state_info[0], 2);
	return NO_ERR;
bypass_checksum_failed_packet:
	return 1;
}

/* #if defined(HX_DEBUG_LEVEL) */
extern struct himax_target_report_data *g_target_report_data;
//extern struct himax_report_data *hx_touch_data;
void himax_log_touch_data(int start)
{
	int loop_i = 0;
	int print_size = 0;
	uint8_t *buf;

	if (start == 1)
		return; /* report data when end of ts_work*/

	if (hx_touch_data->diag_cmd == 0) {
		print_size = hx_touch_data->touch_info_size;
		buf = kzalloc(print_size, GFP_KERNEL);
		if (!buf) {
			E("fail to kzalloc buf!");
			return;
		}
		memcpy(buf, hx_touch_data->hx_coord_buf, hx_touch_data->touch_info_size);
	} else if (hx_touch_data->diag_cmd > 0) {
		print_size = hx_touch_data->touch_all_size;
		buf = kzalloc(print_size, GFP_KERNEL);
		if (!buf) {
			E("fail to kzalloc buf!");
			return;
		}
		memcpy(buf, hx_touch_data->hx_coord_buf, hx_touch_data->touch_info_size);
		memcpy(&buf[hx_touch_data->touch_info_size], hx_touch_data->hx_rawdata_buf, print_size - hx_touch_data->touch_info_size);
	} 

	#if defined(HX_SMART_WAKEUP)
	else if (private_ts->SMWP_enable > 0 && private_ts->suspended) {
		print_size = hx_touch_data->event_size;
		buf = kcalloc(print_size, sizeof(uint8_t), GFP_KERNEL);
		if (buf == NULL) {
			E("%s, Failed to allocate memory\n", __func__);
			return;
		}

		memcpy(buf, hx_touch_data->hx_event_buf, print_size);
	}
	#endif

	else {
		E("%s:cmd fault\n", __func__);
		return;
	}

	for (loop_i = 0; loop_i < print_size; loop_i += 8) {
		if ((loop_i + 7) >= print_size) {
			I("%s: over flow\n", __func__);
			break;
		}

		I("P %2d = 0x%2.2X P %2d = 0x%2.2X ", loop_i, buf[loop_i], loop_i + 1, buf[loop_i + 1]);
		I("P %2d = 0x%2.2X P %2d = 0x%2.2X ", loop_i + 2, buf[loop_i + 2], loop_i + 3, buf[loop_i + 3]);
		I("P %2d = 0x%2.2X P %2d = 0x%2.2X ", loop_i + 4, buf[loop_i + 4], loop_i + 5, buf[loop_i + 5]);
		I("P %2d = 0x%2.2X P %2d = 0x%2.2X ", loop_i + 6, buf[loop_i + 6], loop_i + 7, buf[loop_i + 7]);
		I("\n");
	}
	kfree(buf);
}
void himax_log_touch_event(struct himax_ts_data *ts, int start)
{
	int loop_i = 0;
	if (g_target_report_data->finger_on > 0 && g_target_report_data->finger_num > 0) {
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
			if (g_target_report_data->x[loop_i] >= 0 && g_target_report_data->x[loop_i] <= ts->pdata->abs_x_max && g_target_report_data->y[loop_i] >= 0 && g_target_report_data->y[loop_i] <= ts->pdata->abs_y_max) {
				I("Finger %d=> X:%d, Y:%d W:%d, Z:%d, F:%d\n", loop_i + 1,
				g_target_report_data->x[loop_i],
				g_target_report_data->y[loop_i],
				g_target_report_data->w[loop_i],
				g_target_report_data->w[loop_i],
				loop_i + 1);
			}
		}
	} else if (g_target_report_data->finger_on == 0 && g_target_report_data->finger_num == 0) {
		I("All Finger leave\n");
	} else {
		I("%s : wrong input!\n", __func__);
	}
}
void himax_log_touch_int_devation(int touched)
{
	if (touched == HX_FINGER_ON) {
		timeStart = ktime_to_us(ktime_get());
		/*  I(" Irq start time = %ld.%06ld s\n",
		timeStart.tv_sec, timeStart.tv_nsec/1000); */
	} else if (touched == HX_FINGER_LEAVE) {
		timeEnd = ktime_to_us(ktime_get());
		/*  I("Irq finish time = %ld.%06ld s\n",
			timeEnd.tv_sec, timeEnd.tv_nsec/1000);*/
		I("Touch latency = %lldms\n", timeEnd - timeStart);
	} else {
		I("%s : wrong input!\n", __func__);
	}
}
void himax_log_touch_event_detail(struct himax_ts_data *ts, int start)
{
	int loop_i = 0;

	if (start == HX_FINGER_LEAVE) {
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
			if (((ts->old_finger >> loop_i & 1) == 0) && ((ts->pre_finger_mask >> loop_i & 1) == 1)) {
				if (g_target_report_data->x[loop_i] >= 0 && g_target_report_data->x[loop_i] <= ts->pdata->abs_x_max && g_target_report_data->y[loop_i] >= 0 && g_target_report_data->y[loop_i] <= ts->pdata->abs_y_max)
					I("status: Raw:F:%02d Down, X:%d, Y:%d, W:%d\n", loop_i + 1, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]);
			} else if ((((ts->old_finger >> loop_i & 1) == 1) && ((ts->pre_finger_mask >> loop_i & 1) == 0))) {
				I("status: Raw:F:%02d Up, X:%d, Y:%d\n", loop_i + 1, ts->pre_finger_data[loop_i][0], ts->pre_finger_data[loop_i][1]);
			} else {
				/* I("dbg hx_point_num=%d,old_finger=0x%02X,pre_finger_mask=0x%02X\n",ts->hx_point_num,ts->old_finger,ts->pre_finger_mask);*/
			}
		}
	}
}

void himax_ts_dbg_func(struct himax_ts_data *ts, int start)
{
	switch (ts->debug_log_level) {
	case 1:
		himax_log_touch_data(start);
		break;
	case 2:
		himax_log_touch_event(ts, start);
		break;
	case 4:
		himax_log_touch_int_devation(start);
		break;
	case 8:
		himax_log_touch_event_detail(ts, start);
		break;
	}
}

/* #endif */
static ssize_t himax_diag_arrange_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	g_diag_arr_num = buf[0] - '0';
	I("%s: g_diag_arr_num = %d\n", __func__, g_diag_arr_num);
	return len;
}

void himax_get_mutual_edge(void)
{
	int i = 0;

	for (i = 0; i < (ic_data->HX_RX_NUM * ic_data->HX_TX_NUM); i++) {
		if (diag_mutual[i] > g_max_mutual)
			g_max_mutual = diag_mutual[i];

		if (diag_mutual[i] < g_min_mutual)
			g_min_mutual = diag_mutual[i];
	}
}

void himax_get_self_edge(void)
{
	int i = 0;

	for (i = 0; i < (ic_data->HX_RX_NUM + ic_data->HX_TX_NUM); i++) {
		if (diag_self[i] > g_max_self)
			g_max_self = diag_self[i];

		if (diag_self[i] < g_min_self)
			g_min_self = diag_self[i];
	}
}

/* print first step which is row */
static void print_state_info(struct seq_file *s)
{
	seq_printf(s, "ReCal=%d;\t", hx_touch_data->hx_state_info[0] & 0x03);
	seq_printf(s, "Palm=%d;\t", hx_touch_data->hx_state_info[0] >> 3 & 0x01);
	seq_printf(s, "Water=%d;\t", hx_touch_data->hx_state_info[0] >> 5 & 0x01);
	seq_printf(s, "TXHop=%d;\t", hx_touch_data->hx_state_info[0] >> 6 & 0x01);
	seq_printf(s, "BaseLine=%d;\t", hx_touch_data->hx_state_info[0] >> 2 & 0x01);
	seq_printf(s, "NoiseMd=%d;\n", hx_touch_data->hx_state_info[0] >> 7 & 0x01);
	seq_printf(s, "Border=%d;\t", hx_touch_data->hx_state_info[1] >> 1 & 0x01);
	seq_printf(s, "VR=%d;\t", hx_touch_data->hx_state_info[1] >> 2 & 0x01);
	seq_printf(s, "OneBlk=%d;\t", hx_touch_data->hx_state_info[1] >> 4 & 0x01);
	seq_printf(s, "Blew=%d;\t", hx_touch_data->hx_state_info[1] >> 5 & 0x01);
	seq_printf(s, "Thumb=%d;\t", hx_touch_data->hx_state_info[1] >> 6 & 0x01);
	seq_printf(s, "BorderEx=%d;\n", hx_touch_data->hx_state_info[1] >> 7 & 0x01);
}

static void himax_diag_arrange_print(struct seq_file *s, int i, int j, int transpose)
{
	if (transpose)
		seq_printf(s, "%6d", diag_mutual[j + i * ic_data->HX_RX_NUM]);
	else
		seq_printf(s, "%6d", diag_mutual[i + j * ic_data->HX_RX_NUM]);
}

/* ready to print second step which is column*/
static void himax_diag_arrange_inloop(struct seq_file *s, int in_init, int out_init, bool transpose, int j)
{
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int i;
	int in_max = 0;

	if (transpose)
		in_max = y_channel;
	else
		in_max = x_channel;

	if (in_init > 0) { /* bit0 = 1 */
		for (i = in_init - 1; i >= 0; i--)
			himax_diag_arrange_print(s, i, j, transpose);

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n", diag_self[j]);
			else
				seq_printf(s, " %5d\n", diag_self[x_channel - j - 1]);
		}
	} else {	/* bit0 = 0 */
		for (i = 0; i < in_max; i++)
			himax_diag_arrange_print(s, i, j, transpose);

		if (transpose) {
			if (out_init > 0)
				seq_printf(s, " %5d\n", diag_self[x_channel - j - 1]);
			else
				seq_printf(s, " %5d\n", diag_self[j]);
		}
	}
}

/* print first step which is row */
static void himax_diag_arrange_outloop(struct seq_file *s, int transpose, int out_init, int in_init)
{
	int j;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int out_max = 0;
	int self_cnt = 0;

	if (transpose)
		out_max = x_channel;
	else
		out_max = y_channel;

	if (out_init > 0) { /* bit1 = 1 */
		self_cnt = 1;

		for (j = out_init - 1; j >= 0; j--) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			himax_diag_arrange_inloop(s, in_init, out_init, transpose, j);

			if (!transpose) {
				seq_printf(s, " %5d\n", diag_self[y_channel + x_channel - self_cnt]);
				self_cnt++;
			}
		}
	} else {	/* bit1 = 0 */
		/* self_cnt = x_channel; */
		for (j = 0; j < out_max; j++) {
			seq_printf(s, "%3c%02d%c", '[', j + 1, ']');
			himax_diag_arrange_inloop(s, in_init, out_init, transpose, j);

			if (!transpose)
				seq_printf(s, " %5d\n", diag_self[j + x_channel]);
		}
	}
}

/* determin the output format of diag */
static void himax_diag_arrange(struct seq_file *s)
{
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int bit2, bit1, bit0;
	int i;
	/* rotate bit */
	bit2 = g_diag_arr_num >> 2;
	/* reverse Y */
	bit1 = g_diag_arr_num >> 1 & 0x1;
	/* reverse X */
	bit0 = g_diag_arr_num & 0x1;

	if (g_diag_arr_num < 4) {
		for (i = 0 ; i <= x_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_printf(s, "\n");
		himax_diag_arrange_outloop(s, bit2, bit1 * y_channel, bit0 * x_channel);
		seq_printf(s, "%6c", ' ');

		if (bit0 == 1) {
			for (i = x_channel - 1; i >= 0; i--)
				seq_printf(s, "%6d", diag_self[i]);
		} else {
			for (i = 0; i < x_channel; i++)
				seq_printf(s, "%6d", diag_self[i]);
		}
	} else {
		for (i = 0 ; i <= y_channel; i++)
			seq_printf(s, "%3c%02d%c", '[', i, ']');

		seq_printf(s, "\n");
		himax_diag_arrange_outloop(s, bit2, bit1 * x_channel, bit0 * y_channel);
		seq_printf(s, "%6c", ' ');

		if (bit1 == 1) {
			for (i = x_channel + y_channel - 1; i >= x_channel; i--)
				seq_printf(s, "%6d", diag_self[i]);
		} else {
			for (i = x_channel; i < x_channel + y_channel; i++)
				seq_printf(s, "%6d", diag_self[i]);
		}
	}
}

static void *himax_diag_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;
	return (void *)((unsigned long) *pos + 1);
}

static void *himax_diag_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void himax_diag_seq_stop(struct seq_file *s, void *v)
{
}

static int himax_diag_seq_read(struct seq_file *s, void *v)
{
	struct himax_ts_data *ts = private_ts;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	size_t ret = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	int dsram_type = 0;
	dsram_type = ts->diag_cmd / 10;

	mutual_num	= x_channel * y_channel;
	self_num	= x_channel + y_channel; /* don't add KEY_COUNT */
	width		= x_channel;
	seq_printf(s, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);

	/*	start to show out the raw data in adb shell */
	if ((ts->diag_cmd >= 1 && ts->diag_cmd <= 3) || (ts->diag_cmd == 7)) {
		himax_diag_arrange(s);
		seq_printf(s, "\n");
		seq_printf(s, "ChannelEnd");
		seq_printf(s, "\n");
	} else if (ts->diag_cmd == 8) {
		for (loop_i = 0; loop_i < 128 ; loop_i++) {
			if ((loop_i % 16) == 0)
				seq_printf(s, "LineStart:");

			seq_printf(s, "%4x", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
				seq_printf(s, "\n");
		}
	} else if (dsram_type > 0 && dsram_type <= 8) {
		himax_diag_arrange(s);
		seq_printf(s, "\n ChannelEnd");
		seq_printf(s, "\n");
	}

	if ((ts->diag_cmd >= 1 && ts->diag_cmd <= 7) || dsram_type > 0) {
		/* print Mutual/Slef Maximum and Minimum */
		himax_get_mutual_edge();
		himax_get_self_edge();
		seq_printf(s, "Mutual Max:%3d, Min:%3d\n", g_max_mutual, g_min_mutual);
		seq_printf(s, "Self Max:%3d, Min:%3d\n", g_max_self, g_min_self);
		/* recovery status after print*/
		g_max_mutual = 0;
		g_min_mutual = 0xFFFF;
		g_max_self = 0;
		g_min_self = 0xFFFF;
	}

	/*pring state info*/
	print_state_info(s);
	return ret;
}
static struct seq_operations himax_diag_seq_ops = {
	.start	= himax_diag_seq_start,
	.next	= himax_diag_seq_next,
	.stop	= himax_diag_seq_stop,
	.show	= himax_diag_seq_read,
};
static int himax_diag_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_diag_seq_ops);
};

/* DSRAM thread */
bool himax_ts_diag_func(void)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0, j = 0, ret = true;
	unsigned int index = 0;
	int x_channel = ic_data->HX_RX_NUM;
	int y_channel = ic_data->HX_TX_NUM;
	int total_size = (y_channel * x_channel + y_channel + x_channel) * 2;
	uint8_t *info_data;
	int32_t *mutual_data = NULL;
	int32_t *mutual_data_new = NULL;
	int32_t *mutual_data_old = NULL;
	int32_t *self_data = NULL;
	int32_t *self_data_new = NULL;
	int32_t *self_data_old = NULL;
	int32_t new_data;
	/* 1:common dsram,2:100 frame Max,3:N-(N-1)frame */
	int dsram_type = 0;
    int retry = 3;

	info_data = kzalloc(total_size * sizeof(uint8_t), GFP_KERNEL);
	if (info_data == NULL) {
		ret = false;
		goto error1;
	}
		
	//memset(write_buf, 0x00, sizeof(write_buf) * total_size * 3);
	//memset(info_data, 0x00, total_size * sizeof(uint8_t));
	dsram_type = ts->diag_cmd / 10;
	I("%s:Entering ts->diag_cmd=%d!\n", __func__, ts->diag_cmd);

	if (dsram_type == 8) {
		dsram_type = 1;
		I("%s Sorting Mode run sram type1 !\n", __func__);
	}

	g_core_fp.fp_burst_enable(1);

	if (dsram_type == 1 || dsram_type == 2) {
		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();
	} else if (dsram_type == 3) {
		mutual_data = getMutualBuffer();
		mutual_data_new = getMutualNewBuffer();
		mutual_data_old = getMutualOldBuffer();
		self_data = getSelfBuffer();
		self_data_new = getSelfNewBuffer();
		self_data_old = getSelfOldBuffer();
	}
	
	while(g_core_fp.fp_get_DSRAM_data(info_data, g_DSRAM_Flag) == false && retry > 0){
         if (private_ts->diag_cmd == 0) {
			 I("%s:get rawdata fail,but FORCE to leave,because diag_cmd=%d!\n", __func__, private_ts->diag_cmd);
			 ret = false;
			 goto error2;
			 }
		 E("%s: Now is %d times!\n", __func__, retry);
		 retry--;
	}

	if (retry < 0) {
		ret = false;
		goto error2;
	}
/*
	if (g_core_fp.fp_get_DSRAM_data(info_data, g_DSRAM_Flag) == false) {
        ret = false;
		goto error3;
	}*/

	index = 0;

	for (i = 0; i < y_channel; i++) { /*mutual data*/
		for (j = 0; j < x_channel; j++) {
			new_data = (((int8_t)info_data[index + 1] << 8) | info_data[index]);

			if (dsram_type == 1) {
				mutual_data[i * x_channel + j] = new_data;
			} else if (dsram_type == 2) { /* Keep max data */
				if (mutual_data[i * x_channel + j] < new_data)
					mutual_data[i * x_channel + j] = new_data;
			} else if (dsram_type == 3) { /* Cal data for [N]-[N-1] frame */
				mutual_data_new[i * x_channel + j] = new_data;
				mutual_data[i * x_channel + j] = mutual_data_new[i * x_channel + j] - mutual_data_old[i * x_channel + j];
			}
			index += 2;
		}
	}

	for (i = 0; i < x_channel + y_channel; i++) { /*self data*/
		new_data = (info_data[index + 1] << 8 | info_data[index]);
		if (dsram_type == 1) {
			self_data[i] = new_data;
		} else if (dsram_type == 2) { /* Keep max data */
			if (self_data[i] < new_data)
				self_data[i] = new_data;
		} else if (dsram_type == 3) { /* Cal data for [N]-[N-1] frame */
			self_data_new[i] = new_data;
			self_data[i] = self_data_new[i] - self_data_old[i];
		}
		index += 2;
	}

	if (dsram_type == 3) {
		memcpy(mutual_data_old, mutual_data_new, x_channel * y_channel * sizeof(int32_t)); /* copy N data to N-1 array */
		memcpy(self_data_old, self_data_new, (x_channel + y_channel) * sizeof(int32_t)); /* copy N data to N-1 array */
	}

	diag_max_cnt++;

	if (dsram_type >= 1 && dsram_type <= 3) {
		queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 1 / 10 * HZ);
	}

error2:
	kfree(info_data);
	info_data = NULL;
error1:
	return ret;
}

static ssize_t himax_diag_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	struct himax_ts_data *ts = private_ts;
	char messages[80] = {0};
	uint8_t command[2] = {0x00, 0x00};
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	/* 0: common , other: dsram*/
	int storage_type = 0;
	/* 1:IIR,2:DC,3:Bank,4:IIR2,5:IIR2_N,6:FIR2,7:Baseline,8:dump coord */
	int rawdata_type = 0;

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	I("%s:g_switch_mode = %d\n", __func__, g_switch_mode);

	if (messages[1] == 0x0A) {
		ts->diag_cmd = messages[0] - '0';
	} else {
		ts->diag_cmd = (messages[0] - '0') * 10 + (messages[1] - '0');
	}

	storage_type = g_core_fp.fp_determin_diag_storage(ts->diag_cmd);
	rawdata_type = g_core_fp.fp_determin_diag_rawdata(ts->diag_cmd);

	if (ts->diag_cmd > 0 && rawdata_type == 0) {
		I("[Himax]ts->diag_cmd=0x%x ,storage_type=%d, rawdata_type=%d! Maybe no support!\n"
		  , ts->diag_cmd, storage_type, rawdata_type);
		ts->diag_cmd = 0x00;
	} else {
		I("[Himax]ts->diag_cmd=0x%x ,storage_type=%d, rawdata_type=%d\n", ts->diag_cmd, storage_type, rawdata_type);
	}

	I("-----before diag_mutual memset\n");
	memset(diag_mutual, 0x00, ic_data->HX_RX_NUM * ic_data->HX_TX_NUM * sizeof(int32_t)); /* Set data 0 */
	I("-----before diag_self memset\n");
	memset(diag_self, 0x00, sizeof(diag_self));
	I("-----after diag_self memset\n");
	if (storage_type == 0 && rawdata_type > 0 && rawdata_type < 8) {
		I("%s,common\n", __func__);

		if (g_DSRAM_Flag) {
			/* 1. Clear DSRAM flag */
			g_DSRAM_Flag = false;
			/* 2. Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* 3. Enable ISR */
			/* himax_int_enable(1); */
			/*(4) FW leave sram and return to event stack*/
			g_core_fp.fp_return_event_stack();
		}

		if (g_switch_mode == 2) {
			g_core_fp.fp_idle_mode(0);
			g_switch_mode = g_core_fp.fp_switch_mode(0);
		}

		if (ts->diag_cmd == 0x04) {
			ts->diag_cmd = 0x00;
			command[0] = 0x00;
		} else {
			command[0] = ts->diag_cmd;
		}

		g_core_fp.fp_diag_register_set(command[0], storage_type);
	} else if (storage_type > 0 && storage_type < 8 && rawdata_type > 0 && rawdata_type < 8) { /*DSRAM*/
		I("%s,dsram\n", __func__);
		diag_max_cnt = 0;

		
		if (g_DSRAM_Flag) {
			// (1) Clear DSRAM flag
			g_DSRAM_Flag = false;
			// (2) Stop DSRAM thread
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			// (3) Enable ISR
			// himax_int_enable(1);
			// (4) FW leave sram and return to event stack
			g_core_fp.fp_return_event_stack();
		}

		/* close sorting if turn on*/
		if (g_switch_mode == 2) {
			g_core_fp.fp_idle_mode(0);
			g_switch_mode = g_core_fp.fp_switch_mode(0);
		}

		command[0] = rawdata_type;/* ts->diag_cmd; */
		g_core_fp.fp_diag_register_set(command[0], storage_type);
		/* 1. Disable ISR */
		/* himax_int_enable(0);*/

		/* 2. Start DSRAM thread */
		if (!g_DSRAM_Flag)
			queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 2 * HZ / 100);
		I("%s: Start get raw data in DSRAM\n", __func__);

		if (storage_type == 4)
			msleep(6000);

		/* 3. Set DSRAM flag */
		g_DSRAM_Flag = true;
	} else if (storage_type == 8) { /*Sorting*/
		I("Soritng mode!\n");
		
		/*if (g_DSRAM_Flag) {
			1. Clear DSRAM flag
			g_DSRAM_Flag = false;
			2. Stop DSRAM thread
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			3. Enable ISR
			himax_int_enable(1);
			4. FW leave sram and return to event stack
			g_core_fp.fp_return_event_stack();
		}*/

		g_core_fp.fp_idle_mode(1);
		g_switch_mode = g_core_fp.fp_switch_mode(1);

		if (g_switch_mode == 2)
			g_core_fp.fp_diag_register_set(command[0], storage_type);
		if (!g_DSRAM_Flag)
			queue_delayed_work(private_ts->himax_diag_wq, &private_ts->himax_diag_delay_wrok, 2 * HZ / 100);
		g_DSRAM_Flag = true;
	} else {  /*Normal Mode*/
		/* set diag flag */
		if (g_DSRAM_Flag) {
			tmp_addr[3] = 0x10;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x00;
			
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
			I("return and cancel sram thread!\n");
			/* (1) Clear DSRAM flag */
			g_DSRAM_Flag = false;
			/* (2) Stop DSRAM thread */
			cancel_delayed_work(&private_ts->himax_diag_delay_wrok);
			/* (3) Enable ISR */
			/* himax_int_enable(1); */
			/*(4) FW leave sram and return to event stack*/
			g_core_fp.fp_return_event_stack();
		}

		if (g_switch_mode == 2) {
			g_core_fp.fp_idle_mode(0);
			g_switch_mode = g_core_fp.fp_switch_mode(0);
		}

		if (ts->diag_cmd != 0x00) {
			E("[Himax]ts->diag_cmd error!diag_command=0x%x so reset\n", ts->diag_cmd);
			command[0] = 0x00;

			if (ts->diag_cmd != 0x08)
				ts->diag_cmd = 0x00;

			g_core_fp.fp_diag_register_set(command[0], storage_type);
		} else {
			command[0] = 0x00;
			ts->diag_cmd = 0x00;
			g_core_fp.fp_diag_register_set(command[0], storage_type);
			I("return to normal ts->diag_cmd=0x%x\n", ts->diag_cmd);
		}
	}
	I("%s,g_DSRAM_Flag = %d\n", __func__, g_DSRAM_Flag);//28
	
	return len;
}

static ssize_t himax_reset_write(struct file *file, const char *buff,
								 size_t len, loff_t *pos)
{
	char buf_tmp[12];

	if (len >= 12) {
		I("%s: no command exceeds 12 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len))
		return -EFAULT;

#ifdef HX_RST_PIN_FUNC

	if (buf_tmp[0] == '1') {
		g_core_fp.fp_ic_reset(false, false);
	} else if (buf_tmp[0] == '2') {
		g_core_fp.fp_ic_reset(false, true);
	} else if (buf_tmp[0] == '3') {
		g_core_fp.fp_ic_reset(true, false);
	} else if (buf_tmp[0] == '4') {
		g_core_fp.fp_ic_reset(true, true);
	}
	/* else if (buf_tmp[0] == '5') */
	/*	ESD_HW_REST(); */
#else
	g_core_fp.fp_system_reset();
#endif
#ifdef HX_ZERO_FLASH
	if (g_core_fp.fp_0f_reload_to_active)
		g_core_fp.fp_0f_reload_to_active();
#endif
	return len;
}

static ssize_t himax_debug_read(struct file *file, char *buf,
								size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *temp_buf = NULL;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		if (debug_level_cmd == 't') {
			if (fw_update_complete) {
				ret += snprintf(temp_buf + ret, len - ret, "FW Update Complete ");
			} else {
				ret += snprintf(temp_buf + ret, len - ret, "FW Update Fail ");
			}
		} else if (debug_level_cmd == 'h') {
			if (handshaking_result == 0) {
				ret += snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (MCU Running)\n", handshaking_result);
			} else if (handshaking_result == 1) {
				ret += snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (MCU Stop)\n", handshaking_result);
			} else if (handshaking_result == 2) {
				ret += snprintf(temp_buf + ret, len - ret, "Handshaking Result = %d (xfer Error)\n", handshaking_result);
			} else {
				ret += snprintf(temp_buf + ret, len - ret, "Handshaking Result = error\n");
			}
		} else if (debug_level_cmd == 'v') {
			ret += snprintf(temp_buf + ret, len - ret, "FW_VER = 0x%2.2X\n", ic_data->vendor_fw_ver);

			if (private_ts->chip_cell_type == CHIP_IS_ON_CELL) {
				ret += snprintf(temp_buf + ret, len - ret, "CONFIG_VER = 0x%2.2X\n", ic_data->vendor_config_ver);
			} else {
				ret += snprintf(temp_buf + ret, len - ret, "TOUCH_VER = 0x%2.2X\n", ic_data->vendor_touch_cfg_ver);
				ret += snprintf(temp_buf + ret, len - ret, "DISPLAY_VER = 0x%2.2X\n", ic_data->vendor_display_cfg_ver);
			}
			if (ic_data->vendor_cid_maj_ver < 0 && ic_data->vendor_cid_min_ver < 0)
				ret += snprintf(temp_buf + ret, len - ret, "CID_VER = NULL\n");
			else
				ret += snprintf(temp_buf + ret, len - ret, "CID_VER = 0x%2.2X\n", (ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));

			if (ic_data->vendor_panel_ver < 0)
				ret += snprintf(temp_buf + ret, len - ret, "PANEL_VER = NULL\n");
			else
				ret += snprintf(temp_buf + ret, len - ret, "PANEL_VER = 0x%2.2X\n", ic_data->vendor_panel_ver);
			if (private_ts->chip_cell_type == CHIP_IS_IN_CELL) {
				ret += snprintf(temp_buf + ret, len - ret, "Cusomer = %s\n", ic_data->vendor_cus_info);
				ret += snprintf(temp_buf + ret, len - ret, "Project = %s\n", ic_data->vendor_proj_info);
			}
			ret += snprintf(temp_buf + ret, len - ret, "g_DSRAM_Flag = %d\n", g_DSRAM_Flag);//28
			
			ret += snprintf(temp_buf + ret, len - ret, "\n");
			ret += snprintf(temp_buf + ret, len - ret, "Himax Touch Driver Version:\n");
			ret += snprintf(temp_buf + ret, len - ret, "%s\n", HIMAX_DRIVER_VER);
		} else if (debug_level_cmd == 'd') {
			ret += snprintf(temp_buf + ret, len - ret, "Himax Touch IC Information :\n");
			ret += snprintf(temp_buf + ret, len - ret, "%d\n", private_ts->chip_name);

			switch (IC_CHECKSUM) {
			case HX_TP_BIN_CHECKSUM_SW:
				ret += snprintf(temp_buf + ret, len - ret, "IC Checksum : SW\n");
				break;

			case HX_TP_BIN_CHECKSUM_HW:
				ret += snprintf(temp_buf + ret, len - ret, "IC Checksum : HW\n");
				break;

			case HX_TP_BIN_CHECKSUM_CRC:
				ret += snprintf(temp_buf + ret, len - ret, "IC Checksum : CRC\n");
				break;

			default:
				ret += snprintf(temp_buf + ret, len - ret, "IC Checksum error.\n");
			}

			if (ic_data->HX_INT_IS_EDGE) {
				ret += snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : EDGE TIRGGER\n");
			} else {
				ret += snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : LEVEL TRIGGER\n");
			}
			if (private_ts->protocol_type == PROTOCOL_TYPE_A) {
				ret += snprintf(temp_buf + ret, len - ret, "Protocol : TYPE_A\n");
			} else {
				ret += snprintf(temp_buf + ret, len - ret, "Protocol : TYPE_B\n");
			}

			ret += snprintf(temp_buf + ret, len - ret, "RX Num : %d\n", ic_data->HX_RX_NUM);
			ret += snprintf(temp_buf + ret, len - ret, "TX Num : %d\n", ic_data->HX_TX_NUM);
			ret += snprintf(temp_buf + ret, len - ret, "BT Num : %d\n", ic_data->HX_BT_NUM);
			ret += snprintf(temp_buf + ret, len - ret, "X Resolution : %d\n", ic_data->HX_X_RES);
			ret += snprintf(temp_buf + ret, len - ret, "Y Resolution : %d\n", ic_data->HX_Y_RES);
			ret += snprintf(temp_buf + ret, len - ret, "Max Point : %d\n", ic_data->HX_MAX_PT);
			ret += snprintf(temp_buf + ret, len - ret, "XY reverse : %d\n", ic_data->HX_XY_REVERSE);
		} else if (debug_level_cmd == 'i') {
			if (g_core_fp.fp_read_xfer_status())
				ret += snprintf(temp_buf + ret, len - ret, "xfer communication is bad.\n");
			else
				ret += snprintf(temp_buf + ret, len - ret, "xfer communication is good.\n");
		} else if (debug_level_cmd == 'n') {
			if (g_core_fp.fp_read_ic_trigger_type() == 1) /* Edgd = 1, Level = 0 */
				ret += snprintf(temp_buf + ret, len - ret, "IC Interrupt type is edge trigger.\n");
			else if (g_core_fp.fp_read_ic_trigger_type() == 0)
				ret += snprintf(temp_buf + ret, len - ret, "IC Interrupt type is level trigger.\n");
			else
				ret += snprintf(temp_buf + ret, len - ret, "Unkown IC trigger type.\n");

			if (ic_data->HX_INT_IS_EDGE)
				ret += snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : EDGE TIRGGER\n");
			else
				ret += snprintf(temp_buf + ret, len - ret, "Driver register Interrupt : LEVEL TRIGGER\n");
		}

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		temp_buf = NULL;
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

extern int g_ts_dbg;
static ssize_t himax_debug_write(struct file *file, const char *buff,
								 size_t len, loff_t *pos)
{
	char *fileName;
	char buf[80] = {0};
	int result = 0;
#ifndef HX_ZERO_FLASH
	int fw_type = 0;
	const struct firmware *fw = NULL;
#endif

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == 'h') { /* handshaking */
		debug_level_cmd = buf[0];
		himax_int_enable(0);
		handshaking_result = g_core_fp.fp_hand_shaking(); /* 0:Running, 1:Stop, 2:xfer Fail */
		himax_int_enable(1);
		return len;
	} else if (buf[0] == 'v') { /* firmware version */
		himax_int_enable(0);
#ifdef HX_RST_PIN_FUNC
		g_core_fp.fp_ic_reset(false, false);
#endif
		debug_level_cmd = buf[0];
		g_core_fp.fp_read_FW_ver();
#ifdef HX_RST_PIN_FUNC
		g_core_fp.fp_ic_reset(true, false);
#else
		g_core_fp.fp_system_reset();
#endif
#ifdef HX_ZERO_FLASH
		if (g_core_fp.fp_0f_reload_to_active)
			g_core_fp.fp_0f_reload_to_active();
#endif
		himax_int_enable(1);
		/* himax_check_chip_version(); */
		return len;
	} else if (buf[0] == 'd') { /* ic information */
		debug_level_cmd = buf[0];
		return len;
	} else if (buf[0] == 't') {
		if (buf[1] == 's' &&
		    buf[2] == 'd' &&
		    buf[3] == 'b' &&
		    buf[4] == 'g'
		) {
			if (buf[5] == '1') {
				I("Open Ts Debug!\n");
				g_ts_dbg = 1;
			} else if (buf[5] == '0') {
				I("Close Ts Debug!\n");
				g_ts_dbg = 0;
			} else {
				E("Parameter fault for ts debug\n");
			}
			goto ENDFUCTION;
		}
		himax_int_enable(0);
		debug_level_cmd			= buf[0];
		fw_update_complete		= false;
		fileName = kzalloc(128 * sizeof(char), GFP_KERNEL);
		if (!fileName)
			return -ENOMEM;
		//memset(fileName, 0, 128);
		/* parse the file name */
		snprintf(fileName, len - 2, "%s", &buf[2]);

#ifdef HX_ZERO_FLASH
		I("NOW Running Zero flash update!\n");
		I("%s: upgrade from file(%s) start!\n", __func__, fileName);
#ifndef HX_VIVO_DEBUG_NODE
		g_core_fp.fp_0f_op_file_dirly(fileName);
#else
		result = g_core_fp.fp_0f_operation_dirly();
#endif
		if (result) {
			fw_update_complete = false;
			I("Zero flash update fail!\n");
			kfree(fileName);
			fileName = NULL;
			goto ENDFUCTION;
		} else {
			fw_update_complete = true;
			I("Zero flash update complete!\n");
		}
		goto firmware_upgrade_done;
#else
		I("NOW Running common flow update!\n");
		I("%s: upgrade from file(%s) start!\n", __func__, fileName);
		result = request_firmware(&fw, fileName, private_ts->dev);
		if (result < 0) {
			I("fail to request_firmware fwpath: %s (ret:%d)\n", fileName, result);
			return result;
		}

		I("%s: FW image: %02X, %02X, %02X, %02X\n", __func__, fw->data[0], fw->data[1], fw->data[2], fw->data[3]);
		fw_type = (fw->size) / 1024;
		/*	start to upgrade */
		himax_int_enable(0);
		I("Now FW size is : %dk\n", fw_type);

		switch (fw_type) {
		case 32:
			if (g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k((unsigned char *)fw->data, fw->size, false) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		case 60:
			if (g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k((unsigned char *)fw->data, fw->size, false) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		case 64:
			if (g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k((unsigned char *)fw->data, fw->size, false) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		case 124:
			if (g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k((unsigned char *)fw->data, fw->size, false) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		case 128:
			if (g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k((unsigned char *)fw->data, fw->size, false) == 0) {
				E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
				fw_update_complete = false;
			} else {
				I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
				fw_update_complete = true;
			}
			break;

		default:
			E("%s: Flash command fail: %d\n", __func__, __LINE__);
			fw_update_complete = false;
			break;
		}
		kfree(fileName);
		release_firmware(fw);
		goto firmware_upgrade_done;
#endif
	} else if (buf[0] == 'i' && buf[1] == '2' && buf[2] == 'c') { /* i2c commutation */
		debug_level_cmd = 'i';
		return len;
	} else if (buf[0] == 'i' && buf[1] == 'n' && buf[2] == 't') { /* INT trigger */
		debug_level_cmd = 'n';
		return len;
#ifdef HX_ZERO_FLASH
	} else if (buf[0] == 'z') {
		result = buf[1] - '0';
		I("check type = %d\n", result);
		g_core_fp.fp_0f_operation_check(result);
		return len;
	} else if (buf[0] == 'p') {
		I("NOW debug echo r!\n");
		/* himax_program_sram(); */
		private_ts->himax_0f_update_wq = create_singlethread_workqueue("HMX_update_0f_reuqest_write");

		if (!private_ts->himax_0f_update_wq)
			E(" allocate syn_update_wq failed\n");

		INIT_DELAYED_WORK(&private_ts->work_0f_update, g_core_fp.fp_0f_operation);
		queue_delayed_work(private_ts->himax_0f_update_wq, &private_ts->work_0f_update, msecs_to_jiffies(100));
		return len;
	} else if (buf[0] == 'x') {
		g_core_fp.fp_system_reset();
		if (g_core_fp.fp_0f_reload_to_active)
			g_core_fp.fp_0f_reload_to_active();
		return len;
#endif
	} else { /* others,do nothing */
		debug_level_cmd = 0;
		return len;
	}
	
firmware_upgrade_done:

	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_touch_information();
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, false);
#ifdef HX_ZERO_FLASH
	if (g_core_fp.fp_0f_reload_to_active)
		g_core_fp.fp_0f_reload_to_active();
#endif
#else
	g_core_fp.fp_sense_on(0x00);
#endif

	himax_int_enable(1);
/*	todo himax_chip->tp_firmware_upgrade_proceed = 0;
	todo himax_chip->suspend_state = 0;
	todo enable_irq(himax_chip->irq); */
	
	if(fileName != NULL){
		kfree(fileName);
		fileName = NULL;
	}
ENDFUCTION:
	return len;
}

static ssize_t himax_proc_FW_debug_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	uint8_t loop_i = 0;
	uint8_t tmp_data[64];
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		cmd_set[0] = 0x01;

		if (g_core_fp.fp_read_FW_status(cmd_set, tmp_data) == NO_ERR) {
			ret += snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t", cmd_set[5], cmd_set[4], cmd_set[3], cmd_set[2]);

			for (loop_i = 0; loop_i < cmd_set[1]; loop_i++)
				ret += snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i]);

			ret += snprintf(temp_buf + ret, len - ret, "\n");
		}

		cmd_set[0] = 0x02;

		if (g_core_fp.fp_read_FW_status(cmd_set, tmp_data) == NO_ERR) {
			for (loop_i = 0; loop_i < cmd_set[1]; loop_i = loop_i + 2) {
				if ((loop_i % 16) == 0)
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X%02X%02X%02X :\t",
									cmd_set[5], cmd_set[4], cmd_set[3] + (((cmd_set[2] + loop_i) >> 8) & 0xFF), (cmd_set[2] + loop_i) & 0xFF);

				ret += snprintf(temp_buf + ret, len - ret, "%5d\t", tmp_data[loop_i] + (tmp_data[loop_i + 1] << 8));

				if ((loop_i % 16) == 14)
					ret += snprintf(temp_buf + ret, len - ret, "\n");
			}
		}

		ret += snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_proc_DD_debug_read(struct file *file, char *buf,
										size_t len, loff_t *pos)
{
	ssize_t ret = 0;
	uint8_t tmp_data[64];
	uint8_t loop_i = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		if (mutual_set_flag == 1) {
			if (g_core_fp.fp_read_DD_status(cmd_set, tmp_data) == NO_ERR) {
				for (loop_i = 0; loop_i < cmd_set[0]; loop_i++) {
					if ((loop_i % 8) == 0)
						ret += snprintf(temp_buf + ret, len - ret, "0x%02X : ", loop_i);

					ret += snprintf(temp_buf + ret, len - ret, "0x%02X ", tmp_data[loop_i]);

					if ((loop_i % 8) == 7)
						ret += snprintf(temp_buf + ret, len - ret, "\n");
				}
			}
		}

		ret += snprintf(temp_buf + ret, len - ret, "\n");

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_proc_DD_debug_write(struct file *file, const char *buff,
		size_t len, loff_t *pos)
{
	uint8_t i = 0;
	uint8_t cnt = 2;
	unsigned long result = 0;
	char buf_tmp[20];
	char buf_tmp2[4];

	if (len >= 20) {
		I("%s: no command exceeds 20 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf_tmp, buff, len))
		return -EFAULT;

	memset(buf_tmp2, 0x0, sizeof(buf_tmp2));

	if (buf_tmp[2] == 'x' && buf_tmp[6] == 'x' && buf_tmp[10] == 'x') {
		mutual_set_flag = 1;

		for (i = 3; i < 12; i = i + 4) {
			memcpy(buf_tmp2, buf_tmp + i, 2);

			if (!kstrtoul(buf_tmp2, 16, &result))
				cmd_set[cnt] = (uint8_t)result;
			else
				I("String to oul is fail in cnt = %d, buf_tmp2 = %s\n", cnt, buf_tmp2);

			cnt--;
		}

		I("cmd_set[2] = %02X, cmd_set[1] = %02X, cmd_set[0] = %02X\n", cmd_set[2], cmd_set[1], cmd_set[0]);
	} else {
		mutual_set_flag = 0;
	}

	return len;
}

static ssize_t himax_sense_on_off_write(struct file *file, const char *buff,
										size_t len, loff_t *pos)
{
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0') {
		g_core_fp.fp_sense_off(true);
		I("Sense off\n");
	} else if (buf[0] == '1') {
		if (buf[1] == 's') {
			g_core_fp.fp_sense_on(0x00);
			I("Sense on re-map on, run sram\n");
		} else {
			g_core_fp.fp_sense_on(0x01);
			I("Sense on re-map off, run flash\n");
		}
	} else {
		I("Do nothing\n");
	}

	return len;
}

uint8_t *g_criteria_buf;
uint32_t g_criteria_sze;
uint32_t g_criteria_offset;
struct firmware ext_criteria;

static ssize_t himax_criteria_write(struct file *file, const char *buff, size_t len, loff_t *ops)
{
	mutex_lock(&(private_ts->rw_lock));
	if (g_criteria_sze == 0 || g_criteria_offset == g_criteria_sze) {
		mutex_unlock(&(private_ts->rw_lock));
		return len;
	}
	
	if (copy_from_user(g_criteria_buf + g_criteria_offset, buff, len)) {
		mutex_unlock(&(private_ts->rw_lock));
		return -EFAULT;
	}
	
	if (len > 0) {
		I("Read %ld bytes of hx_criteria at: %d\n", len, g_criteria_offset);
		g_criteria_offset += len;
		if (g_criteria_offset == g_criteria_sze) {
			I("All contents read.\n");
			ext_criteria.data = g_criteria_buf;
			ext_criteria.size = g_criteria_sze;
		}
	} else {
		I("Do nothing\n");
	}
	mutex_unlock(&(private_ts->rw_lock));
	return len;
}

static ssize_t himax_criteria_size_write(struct file *file, const char *buff, size_t len, loff_t *ops)
{
	char buf[20] = {0};
	uint32_t fileSze = 0;

	mutex_lock(&(private_ts->rw_lock));
	if (copy_from_user(buf, buff, len>sizeof(buf)?sizeof(buf):len)) {
		mutex_unlock(&(private_ts->rw_lock));
		return -EFAULT;
	}
	
	sscanf(buf, "%u", &fileSze);
	
	if (fileSze > 0) {
		if (g_criteria_sze != fileSze) {
			kfree(g_criteria_buf);
			g_criteria_buf = kzalloc(fileSze * sizeof(uint8_t), GFP_KERNEL);
			g_criteria_sze = fileSze;
			g_criteria_offset = 0;
		}
		I("criteria file is %d bytes\n", g_criteria_sze);
	} else {
		I("Do nothing\n");
	}
	mutex_unlock(&(private_ts->rw_lock));
	return len;
}

static ssize_t himax_criteria_size_read(struct file *file, char *buf, size_t len, loff_t *ops)
{
	size_t ret = 0;
	char *temp_buf;
	I("%s:enter, %d\n", __func__, __LINE__);
	
	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		ret += snprintf(temp_buf + ret, len - ret, "%d, %d\n", g_criteria_sze, g_criteria_offset);
		
		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);
		
		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}
	return ret;
}

#ifdef HX_ESD_RECOVERY
static ssize_t himax_esd_cnt_read(struct file *file, char *buf,
		size_t len, loff_t *pos)
{
	size_t ret = 0;
	char *temp_buf;
	I("%s: enter, %d\n", __func__, __LINE__);

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		ret += snprintf(temp_buf + ret, len - ret, "EB_cnt = %d, EC_cnt = %d, ED_cnt = %d\n", hx_EB_event_flag, hx_EC_event_flag, hx_ED_event_flag);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return ret;
}

static ssize_t himax_esd_cnt_write(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	int i = 0;
	char buf[12] = {0};

	if (len >= 12) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	I("Clear ESD Flag\n");

	if (buf[i] == '0') {
		hx_EB_event_flag = 0;
		hx_EC_event_flag = 0;
		hx_ED_event_flag = 0;
	}

	return len;
}
#endif

#ifdef HX_TP_PROC_GUEST_INFO
extern char *g_guest_info_item[];


static ssize_t himax_proc_guest_info_read(struct file *file, char *buf, size_t len, loff_t *pos)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	int max_size = 128;
	char *temp_buf;

    I("guest info progress\n");
    if (!HX_PROC_SEND_FLAG) {
	temp_buf = kzalloc(len, GFP_KERNEL);
	if (g_core_fp.guest_info_get_status()) {
		ret += snprintf(temp_buf + ret, len - ret, "Not Ready\n");
		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);
		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
		return ret;
	} else {
			if (g_guest_info_data->g_guest_info_type == 1) {
				for (j = 0 ; j < 3 ; j++) {
					ret += snprintf(temp_buf + ret, len - ret, "%s:\n", g_guest_info_item[j]);
					for (i = 0 ; i < max_size ; i++) {
						if (i % 16 == 0 && i > 0)
							ret += snprintf(temp_buf + ret, len - ret, "\n");
						ret += snprintf(temp_buf + ret, len - ret, "0x%02X\t", g_guest_info_data->g_guest_str[j][i]);
					}
					ret += snprintf(temp_buf + ret, len - ret, "\n");
					I("str[%d] %s\n", j, g_guest_info_data->g_guest_str[j]);
				}
				ret += snprintf(temp_buf + ret, len - ret, "%s:\n", g_guest_info_item[8]);
				for (i = 0 ; i < max_size ; i++) {
					if (i % 16 == 0 && i > 0)
						ret += snprintf(temp_buf + ret, len - ret, "\n");
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X\t", g_guest_info_data->g_guest_str[8][i]);
				}

				ret += snprintf(temp_buf + ret, len - ret, "\n");
				I("str[8] %s\n", g_guest_info_data->g_guest_str[8]);
				ret += snprintf(temp_buf + ret, len - ret, "%s:\n", g_guest_info_item[9]);

				for (i = 0 ; i < max_size ; i++) {
					if (i % 16 == 0 && i > 0)
						ret += snprintf(temp_buf + ret, len - ret, "\n");
					ret += snprintf(temp_buf + ret, len - ret, "0x%02X\t", g_guest_info_data->g_guest_str[9][i]);
				}
				ret += snprintf(temp_buf + ret, len - ret, "\n");
				I("str[8] %s\n", g_guest_info_data->g_guest_str[9]);
			} else if (g_guest_info_data->g_guest_info_type == 0) {
				for (j = 0 ; j < 10 ; j++) {
					if (j == 3)
						j = 8;
					ret += snprintf(temp_buf + ret, len - ret, "%s:\n", g_guest_info_item[j]);
					if (g_guest_info_data->g_guest_data_type[j] == 0)
						ret += snprintf(temp_buf + ret, len - ret, "%s", g_guest_info_data->g_guest_str_in_format[j]);
					else {
						for (i = 0 ; i < g_guest_info_data->g_guest_data_len[j] ; i++) {
							if (i % 16 == 0 && i > 0)
								ret += snprintf(temp_buf + ret, len - ret, "\n");
							ret += snprintf(temp_buf + ret, len - ret, "0x%02X\t", g_guest_info_data->g_guest_str_in_format[j][i]);
						}
					}
					ret += snprintf(temp_buf + ret, len - ret, "\n");
				}
			}
		}
	if (copy_to_user(buf, temp_buf, len))
		I("%s,here:%d\n", __func__, __LINE__);
	kfree(temp_buf);
	HX_PROC_SEND_FLAG = 1;
    } else {
		HX_PROC_SEND_FLAG = 0;
	}
	return ret;
}

static ssize_t himax_proc_guest_info_write(struct file *file, const char *buff, size_t len, loff_t *pos)
{

	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(buf, buff, len))
		return -EFAULT;
		I("%s: buf = %s\n", __func__, buf);
	if (buf[0] == 'r') {
		I("%s,Test to get", __func__);
		queue_work(private_ts->guest_info_wq, &private_ts->guest_info_work);
	}

	return len;
}
#endif
static void himax_himax_data_init(void)
{
	debug_data->fp_ts_dbg_func = himax_ts_dbg_func;
	debug_data->fp_set_diag_cmd = himax_set_diag_cmd;
	g_DSRAM_Flag = false; //28
}

#ifdef HX_TP_PROC_GUEST_INFO
static void himax_ts_guest_info_work_func(struct work_struct *work)
{
	g_core_fp.read_guest_info();
}
#endif

static void himax_ts_diag_work_func(struct work_struct *work)
{
	himax_ts_diag_func();
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
#ifdef HX_VIVO_DEBUG_NODE
static struct file_operations himax_proc_get_rawordiff_data_ops = {
	.owner = THIS_MODULE,
	.read = himax_get_rawordiff_data,
};
static struct file_operations himax_check_debug_info_ops = { /*#show*/
	.owner = THIS_MODULE,
	.read = himax_check_debug_info_read,
	.write = himax_check_debug_info_write,
};
#endif

static struct file_operations himax_proc_CRC_test_ops = {
	.owner = THIS_MODULE,
	.read = himax_CRC_test_read,
};

static struct file_operations himax_proc_vendor_ops = {
	.owner = THIS_MODULE,
	.read = himax_vendor_read,
};

static struct file_operations himax_proc_attn_ops = {
	.owner = THIS_MODULE,
	.read = himax_attn_read,
};

static struct file_operations himax_proc_int_en_ops = {
	.owner = THIS_MODULE,
	.read = himax_int_en_read,
	.write = himax_int_en_write,
};

static struct file_operations himax_proc_layout_ops = {
	.owner = THIS_MODULE,
	.read = himax_layout_read,
	.write = himax_layout_write,
};

static struct file_operations himax_proc_debug_level_ops = {
	.owner = THIS_MODULE,
	.read = himax_debug_level_read,
	.write = himax_debug_level_write,
};

static struct file_operations himax_proc_register_ops = {
	.owner = THIS_MODULE,
	.read = himax_proc_register_read,
	.write = himax_proc_register_write,
};

static struct file_operations himax_proc_diag_arrange_ops = {
	.owner = THIS_MODULE,
	.write = himax_diag_arrange_write,
};

static struct file_operations himax_proc_diag_ops = {
	.owner = THIS_MODULE,
	.open = himax_diag_proc_open,
	.read = seq_read,
	.write = himax_diag_write,
};

static struct file_operations himax_proc_reset_ops = {
	.owner = THIS_MODULE,
	.write = himax_reset_write,
};

static struct file_operations himax_proc_debug_ops = {
	.owner = THIS_MODULE,
	.read = himax_debug_read,
	.write = himax_debug_write,
};


static struct file_operations himax_proc_fw_debug_ops = {
	.owner = THIS_MODULE,
	.read = himax_proc_FW_debug_read,
};

static struct file_operations himax_proc_dd_debug_ops = {
	.owner = THIS_MODULE,
	.read = himax_proc_DD_debug_read,
	.write = himax_proc_DD_debug_write,
};

static struct file_operations himax_proc_sense_on_off_ops = {
	.owner = THIS_MODULE,
	.write = himax_sense_on_off_write,
};

static struct file_operations himax_proc_criteria_ops = {
	.owner = THIS_MODULE,
	.write = himax_criteria_write,
};

static struct file_operations himax_proc_criteria_size_ops = {
	.owner = THIS_MODULE,
	.write = himax_criteria_size_write,
	.read = himax_criteria_size_read,
};

#ifdef HX_ESD_RECOVERY
static struct file_operations himax_proc_esd_cnt_ops = {
	.owner = THIS_MODULE,
	.read = himax_esd_cnt_read,
	.write = himax_esd_cnt_write,
};
#endif

#ifdef HX_TP_PROC_GUEST_INFO
static struct file_operations himax_proc_guest_info_ops = {
	.owner = THIS_MODULE,
	.read = himax_proc_guest_info_read,
	.write = himax_proc_guest_info_write,
};
#endif

#else  /*kernel-5.10*/
#ifdef HX_VIVO_DEBUG_NODE
static struct proc_ops himax_proc_get_rawordiff_data_ops = {
	.proc_read = himax_get_rawordiff_data,
};
static struct proc_ops himax_check_debug_info_ops = { /*#show*/
	.proc_read = himax_check_debug_info_read,
	.proc_write = himax_check_debug_info_write,
};
#endif

static struct proc_ops himax_proc_CRC_test_ops = {
	.proc_read = himax_CRC_test_read,
};

static struct proc_ops himax_proc_vendor_ops = {
	.proc_read = himax_vendor_read,
};

static struct proc_ops himax_proc_attn_ops = {
	.proc_read = himax_attn_read,
};

static struct proc_ops himax_proc_int_en_ops = {
	.proc_read = himax_int_en_read,
	.proc_write = himax_int_en_write,
};

static struct proc_ops himax_proc_layout_ops = {
	.proc_read = himax_layout_read,
	.proc_write = himax_layout_write,
};

static struct proc_ops himax_proc_debug_level_ops = {
	.proc_read = himax_debug_level_read,
	.proc_write = himax_debug_level_write,
};

static struct proc_ops himax_proc_register_ops = {
	.proc_read = himax_proc_register_read,
	.proc_write = himax_proc_register_write,
};

static struct proc_ops himax_proc_diag_arrange_ops = {
	.proc_write = himax_diag_arrange_write,
};

static struct proc_ops himax_proc_diag_ops = {
	.proc_open = himax_diag_proc_open,
	.proc_read = seq_read,
	.proc_write = himax_diag_write,
};

static struct proc_ops himax_proc_reset_ops = {
	.proc_write = himax_reset_write,
};

static struct proc_ops himax_proc_debug_ops = {
	.proc_read = himax_debug_read,
	.proc_write = himax_debug_write,
};

static struct proc_ops himax_proc_fw_debug_ops = {
	.proc_read = himax_proc_FW_debug_read,
};

static struct proc_ops himax_proc_dd_debug_ops = {
	.proc_read = himax_proc_DD_debug_read,
	.proc_write = himax_proc_DD_debug_write,
};

static struct proc_ops himax_proc_sense_on_off_ops = {
	.proc_write = himax_sense_on_off_write,
};

static struct proc_ops himax_proc_criteria_ops = {
	.proc_write = himax_criteria_write,
};

static struct proc_ops himax_proc_criteria_size_ops = {
	.proc_write = himax_criteria_size_write,
	.proc_read = himax_criteria_size_read,
};

#ifdef HX_ESD_RECOVERY
static struct proc_ops himax_proc_esd_cnt_ops = {
	.proc_read = himax_esd_cnt_read,
	.proc_write = himax_esd_cnt_write,
};
#endif

#ifdef HX_TP_PROC_GUEST_INFO
static struct proc_ops himax_proc_guest_info_ops = {
	.proc_read = himax_proc_guest_info_read,
	.proc_write = himax_proc_guest_info_write,
};
#endif

#endif


int himax_touch_proc_init(void)
{
	himax_proc_debug_level_file = proc_create(HIMAX_PROC_DEBUG_LEVEL_FILE, (S_IWUSR | S_IRUGO), himax_touch_proc_dir, &himax_proc_debug_level_ops);
	if (himax_proc_debug_level_file == NULL) {
		E(" %s: proc debug_level file create failed!\n", __func__);
		goto fail_1;
	}

	himax_proc_vendor_file = proc_create(HIMAX_PROC_VENDOR_FILE, (S_IRUGO), himax_touch_proc_dir, &himax_proc_vendor_ops);
	if (himax_proc_vendor_file == NULL) {
		E(" %s: proc vendor file create failed!\n", __func__);
		goto fail_2;
	}

	himax_proc_attn_file = proc_create(HIMAX_PROC_ATTN_FILE, (S_IRUGO), himax_touch_proc_dir, &himax_proc_attn_ops);
	if (himax_proc_attn_file == NULL) {
		E(" %s: proc attn file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_int_en_file = proc_create(HIMAX_PROC_INT_EN_FILE, (S_IWUSR | S_IRUGO),
										 himax_touch_proc_dir, &himax_proc_int_en_ops);
	if (himax_proc_int_en_file == NULL) {
		E(" %s: proc int en file create failed!\n", __func__);
		goto fail_4;
	}

	himax_proc_layout_file = proc_create(HIMAX_PROC_LAYOUT_FILE, (S_IWUSR | S_IRUGO),
										 himax_touch_proc_dir, &himax_proc_layout_ops);
	if (himax_proc_layout_file == NULL) {
		E(" %s: proc layout file create failed!\n", __func__);
		goto fail_5;
	}

	himax_proc_reset_file = proc_create(HIMAX_PROC_RESET_FILE, (S_IWUSR),
										himax_touch_proc_dir, &himax_proc_reset_ops);
	if (himax_proc_reset_file == NULL) {
		E(" %s: proc reset file create failed!\n", __func__);
		goto fail_6;
	}

	himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_diag_ops);
	if (himax_proc_diag_file == NULL) {
		E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7;
	}

	himax_proc_diag_arrange_file = proc_create(HIMAX_PROC_DIAG_ARR_FILE, (S_IWUSR | S_IRUGO),
								   himax_touch_proc_dir, &himax_proc_diag_arrange_ops);
	if (himax_proc_diag_arrange_file == NULL) {
		E(" %s: proc diag file create failed!\n", __func__);
		goto fail_7_1;
	}

	himax_proc_register_file = proc_create(HIMAX_PROC_REGISTER_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										   himax_touch_proc_dir, &himax_proc_register_ops);
	if (himax_proc_register_file == NULL) {
		E(" %s: proc register file create failed!\n", __func__);
		goto fail_8;
	}

	himax_proc_debug_file = proc_create(HIMAX_PROC_DEBUG_FILE, (S_IWUSR | S_IRUGO), himax_touch_proc_dir, &himax_proc_debug_ops);
	if (himax_proc_debug_file == NULL) {
		E(" %s: proc debug file create failed!\n", __func__);
		goto fail_9;
	}

	himax_proc_fw_debug_file = proc_create(HIMAX_PROC_FW_DEBUG_FILE, (S_IWUSR | S_IRUGO),
										   himax_touch_proc_dir, &himax_proc_fw_debug_ops);
	if (himax_proc_fw_debug_file == NULL) {
		E(" %s: proc fw debug file create failed!\n", __func__);
		goto fail_9_1;
	}

	himax_proc_dd_debug_file = proc_create(HIMAX_PROC_DD_DEBUG_FILE, (S_IWUSR | S_IRUGO),
										   himax_touch_proc_dir, &himax_proc_dd_debug_ops);
	if (himax_proc_dd_debug_file == NULL) {
		E(" %s: proc DD debug file create failed!\n", __func__);
		goto fail_9_2;
	}

	himax_proc_SENSE_ON_OFF_file = proc_create(HIMAX_PROC_SENSE_ON_OFF_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
								   himax_touch_proc_dir, &himax_proc_sense_on_off_ops);
	if (himax_proc_SENSE_ON_OFF_file == NULL) {
		E(" %s: proc SENSE_ON_OFF file create failed!\n", __func__);
		goto fail_16;
	}

	himax_proc_HX_CRITERIA_file = proc_create(HIMAX_PROC_HX_CRITERIA_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
											himax_touch_proc_dir, &himax_proc_criteria_ops);
	if (himax_proc_HX_CRITERIA_file == NULL) {
		E("%s: proc himax_proc_HX_CRITERIA_file file create failed\n", __func__);
		goto fail_16_c0;
	}
	
	himax_proc_HX_CRITERIA_SIZE_file = proc_create(HIMAX_PROC_HX_CRITERIA_SIZE_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
											himax_touch_proc_dir, &himax_proc_criteria_size_ops);
	if (himax_proc_HX_CRITERIA_SIZE_file == NULL) {
		E("%s: proc himax_proc_HX_CRITERIA_SIZE_file file create failed\n", __func__);
		goto fail_16_c1;
	}

#ifdef HX_ESD_RECOVERY
	himax_proc_ESD_cnt_file = proc_create(HIMAX_PROC_ESD_CNT_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										  himax_touch_proc_dir, &himax_proc_esd_cnt_ops);

	if (himax_proc_ESD_cnt_file == NULL) {
		E(" %s: proc ESD cnt file create failed!\n", __func__);
		goto fail_17;
	}

#endif
	himax_proc_CRC_test_file = proc_create(HIMAX_PROC_CRC_TEST_FILE, (S_IWUSR | S_IRUGO | S_IWUGO), himax_touch_proc_dir, &himax_proc_CRC_test_ops);

	if (himax_proc_CRC_test_file == NULL) {
		E(" %s: proc CRC test file create failed!\n", __func__);
		goto fail_18;
	}
#ifdef HX_TP_PROC_GUEST_INFO
	himax_proc_guest_info_file = proc_create(HIMAX_PROC_GUEST_INFO_FILE, (S_IWUSR|S_IRUGO|S_IWUGO), himax_touch_proc_dir, &himax_proc_guest_info_ops);
	if (himax_proc_guest_info_file == NULL) {
		E(" %s: proc guest information file create failed!\n", __func__);
		goto fail_19;
	}
#endif
#ifdef HX_VIVO_DEBUG_NODE
	himax_proc_get_rawordiff_data_file = proc_create(HIMAX_PROC_GET_RAWORDIFF_DATA_FILE, (S_IWUSR | S_IRUGO | S_IWUGO), himax_touch_proc_dir, &himax_proc_get_rawordiff_data_ops);
	if (himax_proc_get_rawordiff_data_file == NULL) {
		E(" %s: himax_proc_get_rawordiff_data  create failed!\n", __func__);
		goto fail_20;
	}
	himax_check_debug_info_file = proc_create(HIMAX_CHECK_DEBUG_INFO_FILE, (S_IWUSR | S_IRUGO | S_IWUGO), himax_touch_proc_dir, &himax_check_debug_info_ops);
	if (himax_check_debug_info_file == NULL) {
		E(" %s: himax_proc_get_rawordiff_data create failed!\n", __func__);
		goto fail_21;
	}

#endif
	return 0;
#ifdef HX_VIVO_DEBUG_NODE

fail_21:
	remove_proc_entry(HIMAX_PROC_GET_RAWORDIFF_DATA_FILE, himax_touch_proc_dir); /*#12*/
fail_20:
#endif

#ifdef HX_TP_PROC_GUEST_INFO
	remove_proc_entry(HIMAX_PROC_GUEST_INFO_FILE, himax_touch_proc_dir);
fail_19:
#endif
remove_proc_entry(HIMAX_PROC_CRC_TEST_FILE, himax_touch_proc_dir);

fail_18:
#ifdef HX_ESD_RECOVERY
	remove_proc_entry(HIMAX_PROC_ESD_CNT_FILE, himax_touch_proc_dir);
fail_17:
#endif
	remove_proc_entry(HIMAX_PROC_HX_CRITERIA_SIZE_FILE, himax_touch_proc_dir);
fail_16_c1:remove_proc_entry(HIMAX_PROC_HX_CRITERIA_FILE, himax_touch_proc_dir);
fail_16_c0:remove_proc_entry(HIMAX_PROC_SENSE_ON_OFF_FILE, himax_touch_proc_dir);
fail_16: remove_proc_entry(HIMAX_PROC_DD_DEBUG_FILE, himax_touch_proc_dir);
fail_9_2:remove_proc_entry(HIMAX_PROC_FW_DEBUG_FILE, himax_touch_proc_dir);
fail_9_1: remove_proc_entry(HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir);
fail_9: remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
fail_8:	remove_proc_entry(HIMAX_PROC_DIAG_ARR_FILE, himax_touch_proc_dir);
fail_7_1:remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
fail_7: remove_proc_entry(HIMAX_PROC_RESET_FILE, himax_touch_proc_dir);
fail_6:	remove_proc_entry(HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir);
fail_5: remove_proc_entry(HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir);
fail_4: remove_proc_entry(HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir);
fail_3: remove_proc_entry(HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir);
fail_2: remove_proc_entry(HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir);

fail_1:
	return -ENOMEM;
}

void himax_touch_proc_deinit(void)
{
	remove_proc_entry(HIMAX_PROC_CRC_TEST_FILE, himax_touch_proc_dir);
#ifdef HX_ESD_RECOVERY
	remove_proc_entry(HIMAX_PROC_ESD_CNT_FILE, himax_touch_proc_dir);
#endif
	remove_proc_entry(HIMAX_PROC_HX_CRITERIA_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SENSE_ON_OFF_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_FW_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DD_DEBUG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_REGISTER_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DIAG_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_RESET_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_LAYOUT_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_INT_EN_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_ATTN_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_VENDOR_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_DEBUG_LEVEL_FILE, himax_touch_proc_dir);
#ifdef HX_TP_PROC_GUEST_INFO
	remove_proc_entry(HIMAX_PROC_GUEST_INFO_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_VIVO_DEBUG_NODE
	remove_proc_entry(HIMAX_PROC_GET_RAWORDIFF_DATA_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_CHECK_DEBUG_INFO_FILE, himax_touch_proc_dir); /*#12*/
#endif
}

int himax_debug_init(void)
{
	struct himax_ts_data *ts = private_ts;
	int err = 0;

	I("%s:Enter\n", __func__);

	if (ts == NULL) {
		E("%s: ts struct is NULL\n", __func__);
		return -EPROBE_DEFER;
	}

	debug_data = kzalloc(sizeof(*debug_data), GFP_KERNEL);
	if (debug_data == NULL) { /*Allocate debug data space*/
		err = -ENOMEM;
		goto err_alloc_debug_data_fail;
	}

	himax_himax_data_init();

#ifdef HX_TP_PROC_GUEST_INFO

	if (g_guest_info_data == NULL) {
		g_guest_info_data = kzalloc(sizeof(struct hx_guest_info), GFP_KERNEL);
		g_guest_info_data->g_guest_info_ongoing = 0;
		g_guest_info_data->g_guest_info_type = 0;
	}

	ts->guest_info_wq = create_singlethread_workqueue("himax_guest_info_wq");
	if (!ts->guest_info_wq) {
		E("%s: create guest info workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_guest_info_wq_failed;
	}
	INIT_WORK(&ts->guest_info_work, himax_ts_guest_info_work_func);
#endif

	ts->himax_diag_wq = create_singlethread_workqueue("himax_diag");

	if (!ts->himax_diag_wq) {
		E("%s: create diag workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_debug_data_fail;
	}

	INIT_DELAYED_WORK(&ts->himax_diag_delay_wrok, himax_ts_diag_work_func);

	setMutualBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);
	setMutualNewBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);
	setMutualOldBuffer(ic_data->HX_RX_NUM, ic_data->HX_TX_NUM);

	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate fail failed\n", __func__);
		err = MEM_ALLOC_FAIL;
		goto null_getMutualBuffer;
	}

	return 0;

null_getMutualBuffer:
	cancel_delayed_work_sync(&ts->himax_diag_delay_wrok);
	destroy_workqueue(ts->himax_diag_wq);
#ifdef HX_TP_PROC_GUEST_INFO
err_alloc_debug_data_fail:
	destroy_workqueue(ts->guest_info_wq);
#endif
err_create_guest_info_wq_failed:

	return err;
}

int himax_debug_remove(void)
{
	struct himax_ts_data *ts = private_ts;

	himax_touch_proc_deinit();

	cancel_delayed_work_sync(&ts->himax_diag_delay_wrok);
#ifdef HX_TP_PROC_GUEST_INFO
	destroy_workqueue(ts->guest_info_wq);
	if (g_guest_info_data != NULL)
		kfree(g_guest_info_data);
#endif
	destroy_workqueue(ts->himax_diag_wq);

	kfree(debug_data);

	return 0;
}
