/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ili9881x.h"

enum {
	GET_FW_VER = 0,
	GET_FW_CONFIG_VER,
};

enum {
	CHARGER_OFF = 0,
	CHARGER_ON,
};

enum {
	NORMAL_MODE = 0,
	DEEP_SLEEP_MODE,
	GESTURE_MODE,
};

enum {
	EDGE_PALM_LEFT_UP_RIGHT_DOWN = 0,
	EDGE_PALM_STRAIGHT,
	EDGE_PALM_LEFT_DOWN_RIGHT_UP,
};

enum {
	ILI_RAWDATA = 0,
	ILI_DIFFDATA,
	ILI_BASEDATA,
};

int SetFwState(void)
{
	int ret = 0;
	ILI_INFO("charger = %d , EdgeRestain = %d , gamemode = %d \n",ilits->vivo_charger, ilits->vivo_EdgeRestain, !ilits->vivo_gamemode);
	switch (ilits->vivo_charger) {
	case CHARGER_OFF:
		ILI_INFO("set charger off\n");
		ret = ili_ic_func_ctrl("plug", ENABLE);
		break;
	case CHARGER_ON:
		ILI_INFO("set charger on\n");
		ret = ili_ic_func_ctrl("plug", DISABLE);
		break;
	default:
		ILI_ERR("Unknown type, %d\n", ilits->vivo_charger);
		ret = -1;
		break;
	}

	switch (ilits->vivo_EdgeRestain) {
	case EDGE_PALM_LEFT_UP_RIGHT_DOWN:
		ILI_INFO("Edge: left rotation\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x2);
		break;
	case EDGE_PALM_STRAIGHT:
		ILI_INFO("Edge: straight\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x1);
		break;
	case EDGE_PALM_LEFT_DOWN_RIGHT_UP:
		ILI_INFO("Edge: right rotation\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x0);
		break;
	default:
		ILI_ERR("Unknown type, %d\n", ilits->vivo_EdgeRestain);
		ret = -1;
		break;
	}	

	if (ilits->vivo_gamemode){
		ili_ic_func_ctrl("lock_point", 1) ;
	}else{
		ili_ic_func_ctrl("lock_point", 0) ;
	}
	return ret;
}


int bbk_ili_get_rawordiff_data_v2_9882n(struct vts_device *vtsdev, enum vts_frame_type which, int *data)
{
	int i,	ret,	rlen = 0;
	u8 cmd[2] = { 0 }, row, col;
	u8 checksum = 0;
	s16 temp;
	int retry = 5;
	u16 self_key = 2;

	switch (which) {
	case ILI_RAWDATA:
		ILI_INFO("get raw data");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		break;
	case ILI_DIFFDATA:
		ILI_INFO("get delta data\n");
		cmd[1] = P5_X_FW_DELTA_DATA_MODE;
		break;
	case ILI_BASEDATA:
		ILI_INFO("get baseline data\n");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		break;
	default:
		ILI_ERR("Unknown type, %d\n", which);
		return -1;
	}

	rlen = (2 * ilits->xch_num * ilits->ych_num) + (ilits->stx * 2) + (ilits->srx * 2);
	rlen += 2 * self_key + (8 * 2) + 1 + 35;

	row = ilits->ych_num;
	col = ilits->xch_num;
	mutex_lock(&ilits->touch_mutex);
	ili_set_tp_data_len(DATA_FORMAT_DEBUG, false, NULL) ;
	mdelay(10);
	cmd[0] = 0xFA;
	ret = ilits->wrapper(cmd, 2, NULL, 0, ON, OFF);
	mdelay(10);
	if (ilitek_debug_node_buff_control(ENABLE) < 0) {
		ILI_ERR("Failed to allocate debug buf\n");
		ret = -1;
		mutex_unlock(&ilits->touch_mutex);
		goto out;
	}

       while (retry--) {
		mdelay(50);
		memset(ilits->p_rawdata, 0, 2048);
		ilits->wrapper(NULL, 0, ilits->p_rawdata, rlen, OFF, OFF);
		checksum = ili_calc_packet_checksum(ilits->p_rawdata, rlen - 1);
		if (ilits->p_rawdata[0] == P5_X_DEBUG_PACKET_ID && checksum == ilits->p_rawdata[rlen-1]) {
			ILI_INFO("debug packet checksum success !\n");
			break;
		}
       }
	mutex_unlock(&ilits->touch_mutex);

	ilits->report = DISABLE;
	for (i = 0; i < row * col; i++, ilits->p_rawdata += 2) {
		temp = (*ilits->p_rawdata << 8) + *(ilits->p_rawdata + 1);
		if (which != P5_X_FW_RAW_DATA_MODE)
			temp -= 0x10000;

		*(data + i) = temp;
	}

	mdelay(10);
out:
	ili_set_tp_data_len(DATA_FORMAT_DEMO, false, NULL);
	return ret;
}

int bbk_ili_get_rawordiff_data_v2(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size)
{
	int i,	ret,	rlen = 0;
	u8 cmd[2] = { 0 }, row, col;
	u8 checksum = 0;
	s16 temp;
	int retry = 5;
	u16 self_key = 2;
	u8 *ptr;

	switch (which) {
	case ILI_RAWDATA:
		ILI_INFO("get raw data");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		break;
	case ILI_DIFFDATA:
		ILI_INFO("get delta data\n");
		cmd[1] = P5_X_FW_DELTA_DATA_MODE;
		break;
	case ILI_BASEDATA:
		ILI_INFO("get baseline data\n");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		break;
	default:
		ILI_ERR("Unknown type, %d\n", which);
		return -1;
	}

	rlen = (2 * ilits->xch_num * ilits->ych_num) + (ilits->stx * 2) + (ilits->srx * 2);
	rlen += 2 * self_key + (8 * 2) + 1 + 35;

	row = ilits->ych_num;
	col = ilits->xch_num;
	mutex_lock(&ilits->touch_mutex);
	ili_set_tp_data_len(DATA_FORMAT_DEBUG, false, NULL) ;
	mdelay(20);
	cmd[0] = 0xFA;
	ret = ilits->wrapper(cmd, 2, NULL, 0, ON, OFF);
	mdelay(20);
	ilits->wrapper(NULL, 0, ilits->p_rawdata, rlen, OFF, OFF);	

       while (retry--) {
		mdelay(20);
		memset(ilits->p_rawdata, 0, 2048);
		ilits->wrapper(NULL, 0, ilits->p_rawdata, rlen, OFF, OFF);
		checksum = ili_calc_packet_checksum(ilits->p_rawdata, rlen - 1);
		if (ilits->p_rawdata[0] == P5_X_DEBUG_PACKET_ID && checksum == ilits->p_rawdata[rlen-1]) {
			ILI_INFO("debug packet checksum success !\n");
			break;
		}
       }
	mutex_unlock(&ilits->touch_mutex);

	ptr = &ilits->p_rawdata[35];
	for (i = 0; i < row * col; i++, ptr += 2) {
		temp = (*ptr << 8) + *(ptr + 1);
		if (which != P5_X_FW_RAW_DATA_MODE)
			temp -= 0x10000;

		*((data + (row*(i % col))) + (i /col)) = temp;
	}

	mdelay(10);

	ili_set_tp_data_len(DATA_FORMAT_DEMO, false, NULL);
	return ret;
}


int bbk_ili_fw_update_v2_9882n(struct vts_device *vtsdev, const struct firmware *fw)
{
	int ret = 0;

	if (!fw) {
		ILI_ERR("firmware is null\n");
		return -ENOMEM;
	}

	if (atomic_read(&ilits->fw_stat)) {
		ILI_ERR("fw update is still running\n");
		return -1;
	}

	ILI_INFO("----------------------------11");
	ilits->force_fw_update = ENABLE;
	ret = ili_fw_upgrade_handler(NULL);
	ilits->force_fw_update = DISABLE;
	SetFwState();
	return ret;
}

int idleEnableOrDisable_v2_9882n(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	ilits->vivo_gamemode = state;
	ILI_INFO("game mode %s\n", (state == ENABLE) ? "OFF" : "ON");
	mutex_lock(&ilits->touch_mutex);

	if (state == ENABLE) {
		ili_ic_func_ctrl ("lock_point", 0x01);
	} else {
		ili_ic_func_ctrl ("lock_point", 0x00);
	}
	
	mutex_unlock(&ilits->touch_mutex);
	return ret;
}

int bbk_ili_readUdd_v2_9882n(unsigned char *udd)
{
	return 0;
}

int bbk_ili_writeUdd_v2_9882n(unsigned char *udd)
{
	return 0;
}
static int last_state = VTS_ST_NORMAL;
int bbk_ili_mode_change_v2_9882n(struct vts_device *vtsdev, int which)
{
	int ret = 0;
#ifdef CONFIG_ARCH_QCOM
	int cs_retry = 0;
#endif
	ILI_INFO("which = %d\n", which);

	switch (which) {
	case DEEP_SLEEP_MODE:
		ILI_INFO("deep sleep mode\n");
		if (last_state == NORMAL_MODE) {
			ilits->gesture = DISABLE;
			ret = ili_sleep_handler(TP_DEEP_SLEEP);
		} else {
			ilits->gesture = DISABLE;
			ilits->tp_suspend = true;
			ili_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
			ilits->tp_suspend = false;
		}

#ifdef CONFIG_ARCH_QCOM
		msleep(240);
		if (gpio_is_valid(ilits->tp_cs)) {
			do {
				msleep(10);
				VTI("cs still high!!!");
				gpio_set_value(ilits->tp_cs, 0);
				cs_retry++;
			} while (gpio_get_value(ilits->tp_cs) && (cs_retry < 5));
		}
		if(ilits->reset_poweroff) {
			VTI("reset power off");
			if (gpio_is_valid(ilits->tp_rst))
				gpio_set_value(ilits->tp_rst, 0);
		}
#endif
		mdelay(50);
		vts_dsi_panel_reset_power_ctrl(0);
		ili_irq_disable();
		mdelay(35);
		/* power off */
		last_state = DEEP_SLEEP_MODE;
		break;
	case NORMAL_MODE:
		ILI_INFO("resume to ap\n");
		/* power on */
		msleep(30);
		if (gpio_is_valid(ilits->tp_cs))
			gpio_set_value(ilits->tp_cs, 1);
		if (gpio_is_valid(ilits->tp_rst))
			gpio_set_value(ilits->tp_rst, 1);
		ret = ili_sleep_handler(TP_RESUME);
		last_state = NORMAL_MODE;
		break;
	case GESTURE_MODE:
		ILI_INFO("suspend with gesture\n");
		if (last_state == GESTURE_MODE) {
			ILI_INFO("gesture break\n");
			break;
		}
		if (last_state == NORMAL_MODE) {
			ilits->gesture = ENABLE;
			ret = ili_sleep_handler(TP_SUSPEND);
			msleep(120);
		} else {
			/* power on */
			vts_dsi_panel_reset_power_ctrl(1);
			mdelay(5);
			if (gpio_is_valid(ilits->tp_rst)) 
				gpio_set_value(ilits->tp_rst, 1);
			if (gpio_is_valid(ilits->tp_cs))
				gpio_set_value(ilits->tp_cs, 1);
			mdelay(50);
			ilits->gesture = ENABLE;
			ili_gesture_recovery();
			enable_irq_wake(ilits->irq_num);
			ili_irq_enable();
			msleep(180);
		}
		last_state = GESTURE_MODE;
		break;
	}
	return ret;
}

int bbk_ili_get_fw_version_v2_9882n(int which)
{
	int ret = 0;

	ILI_INFO("which = %d\n", which);

	switch (which) {
	case GET_FW_CONFIG_VER:
		ILI_INFO("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ILI_INFO("get fw version\n");
		mutex_lock(&ilits->touch_mutex);
		ili_ic_get_fw_ver();
		mutex_unlock(&ilits->touch_mutex);
		ret = ilits->chip->fw_ver & 0xffff;
		break;
	default:
		ILI_ERR("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ILI_INFO("firmware version = %x\n", ret);
	return ret;
}

int bbk_ili_get_fw_version_v2(struct vts_device *vtsdev, u64 *version)
{
	int ret = 0;
	ILI_INFO("get fw version\n");
	mutex_lock(&ilits->touch_mutex);
	ili_ic_get_fw_ver();
	mutex_unlock(&ilits->touch_mutex);
	ret = ilits->chip->fw_ver & 0xffff;
	ILI_INFO("firmware version = %x\n", ret);
	*version = (u64)ret;
	return 0;
}


struct point_xy {
	u16 x;
	u16 y;
};

struct gesture_buf {
	struct point_xy Point_start;
	struct point_xy Point_end;
	struct point_xy Point_1st;
	struct point_xy Point_2st;
	struct point_xy Point_3st;
	struct point_xy Point_4st;
};

int bbk_ili_gesture_point_get_v2_9882n(u16 *data)
{
	int ret = 0;
	u16 xx[6];
	u16 yy[6];
	uint8_t gesture_id = 0;
	uint8_t score = 0;
	int nlength = 0;
	u16 x1, y1, x2, y2;
	struct gesture_buf gesture;

	if (!data) {
		ILI_ERR("data is null\n");
		return -1;
	}

	ili_dump_data(ilits->vivo_ges_data, 8, P5_X_GESTURE_INFO_LENGTH + 1, 0, "VIVO_TS gesture before");

	gesture_id = ilits->vivo_ges_data[1];
	score = ilits->vivo_ges_data[36];

	x1 = (((ilits->vivo_ges_data[4] & 0xF0) << 4) | (ilits->vivo_ges_data[5]));
	y1 = (((ilits->vivo_ges_data[4] & 0x0F) << 8) | (ilits->vivo_ges_data[6]));
	x2 = (((ilits->vivo_ges_data[7] & 0xF0) << 4) | (ilits->vivo_ges_data[8]));
	y2 = (((ilits->vivo_ges_data[7] & 0x0F) << 8) | (ilits->vivo_ges_data[9]));

	xx[0] = (((ilits->vivo_ges_data[28] & 0xF0) << 4) | (ilits->vivo_ges_data[29]));
	yy[0] = (((ilits->vivo_ges_data[28] & 0x0F) << 8) | (ilits->vivo_ges_data[30]));
	xx[1] = (((ilits->vivo_ges_data[31] & 0xF0) << 4) | (ilits->vivo_ges_data[32]));
	yy[1] = (((ilits->vivo_ges_data[31] & 0x0F) << 8) | (ilits->vivo_ges_data[33]));

	xx[2] = xx[0] + ((xx[1] - xx[0])/2);
	yy[2] = yy[0];
	xx[3] = xx[0];
	yy[3] = yy[0] + ((yy[1] - yy[0])/2);
	xx[4] = xx[0] + ((xx[1] - xx[0])/2);
	yy[4] = yy[1];
	xx[5] = xx[1];
	yy[5] = yy[0] + ((yy[1] - yy[0])/2);

	gesture.Point_start.x = x1*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_start.y = y1*ilits->panel_hei/TPD_HEIGHT;
	gesture.Point_end.x  = x2*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_end.y  = y2*ilits->panel_hei/TPD_HEIGHT;
	gesture.Point_1st.x   = xx[2]*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_1st.y   = yy[2]*ilits->panel_hei/TPD_HEIGHT;
	gesture.Point_2st.x   = xx[3]*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_2st.y   = yy[3]*ilits->panel_hei/TPD_HEIGHT;
	gesture.Point_3st.x   = xx[4]*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_3st.y   = yy[4]*ilits->panel_hei/TPD_HEIGHT;
	gesture.Point_4st.x   = xx[5]*ilits->panel_wid/TPD_WIDTH;
	gesture.Point_4st.y   = yy[5]*ilits->panel_hei/TPD_HEIGHT;

/*
	gesture.Point_4st.x   = (((point_data[25] & 0xF0) << 4) | (point_data[26]));
	gesture.Point_4st.y   = (((point_data[25] & 0x0F) << 8) | (point_data[27]));
*/
     /*judge gesture type  */
	switch (gesture_id) {
	case GESTURE_UP:
		*data       = gesture.Point_start.x;
		*(data + 1) = gesture.Point_start.y;
		*(data + 2) = gesture.Point_end.x;
		*(data + 3) = gesture.Point_end.y;
		nlength = 2;
		break;
	case GESTURE_W:
		gesture.Point_1st.x = (((ilits->vivo_ges_data[16] & 0xF0) << 4) | (ilits->vivo_ges_data[17]))*ilits->panel_wid/TPD_WIDTH;
		gesture.Point_1st.y = (((ilits->vivo_ges_data[16] & 0x0F) << 8) | (ilits->vivo_ges_data[18]))*ilits->panel_hei/TPD_HEIGHT;
		gesture.Point_2st.x = (((ilits->vivo_ges_data[19] & 0xF0) << 4) | (ilits->vivo_ges_data[20]))*ilits->panel_wid/TPD_WIDTH;
		gesture.Point_2st.y = (((ilits->vivo_ges_data[19] & 0x0F) << 8) | (ilits->vivo_ges_data[21]))*ilits->panel_hei/TPD_HEIGHT;
		gesture.Point_3st.x = (((ilits->vivo_ges_data[22] & 0xF0) << 4) | (ilits->vivo_ges_data[23]))*ilits->panel_wid/TPD_WIDTH;
		gesture.Point_3st.y = (((ilits->vivo_ges_data[22] & 0x0F) << 8) | (ilits->vivo_ges_data[24]))*ilits->panel_hei/TPD_HEIGHT;
		*data     = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_1st.x;
		*(data + 3)  = gesture.Point_1st.y;
		*(data + 4)  = gesture.Point_2st.x;
		*(data + 5)  = gesture.Point_2st.y;
		*(data + 6)  = gesture.Point_3st.x;
		*(data + 7)  = gesture.Point_3st.y;
		*(data + 8)  = gesture.Point_end.x;
		*(data + 9)  = gesture.Point_end.y;
		nlength = 5;
		break;
	case GESTURE_E:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_C:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_2st.x;
		*(data + 3)  = gesture.Point_2st.y;
		*(data + 4)  = gesture.Point_end.x;
		*(data + 5)  = gesture.Point_end.y;
		*(data + 6)  = gesture.Point_1st.x;
		*(data + 7)  = gesture.Point_1st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 3;
		break;
	case GESTURE_O:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		if (ilits->vivo_ges_data[34]) {/*point_data[34] = 1 clockwise*/
			/*vtsGesturePointsReport(VTS_GESTURE_O_DIR,1,-1,-1); */
		}
		nlength = 6;
		break;
	case GESTURE_F:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_A:
		*data        = gesture.Point_start.x;
		*(data + 1)  = gesture.Point_start.y;
		*(data + 2)  = gesture.Point_end.x;
		*(data + 3)  = gesture.Point_end.y;
		*(data + 4)  = gesture.Point_1st.x;
		*(data + 5)  = gesture.Point_1st.y;
		*(data + 6)  = gesture.Point_2st.x;
		*(data + 7)  = gesture.Point_2st.y;
		*(data + 8)  = gesture.Point_3st.x;
		*(data + 9)  = gesture.Point_3st.y;
		*(data + 10) = gesture.Point_4st.x;
		*(data + 11) = gesture.Point_4st.y;
		nlength = 6;
		break;
	default:
		nlength = -1;
		break;
	}
	ILI_INFO("Point_start (x = %d,y = %d),Point_end (x = %d,y = %d)\n", gesture.Point_start.x, gesture.Point_start.y, gesture.Point_end.x, gesture.Point_end.y);
	ILI_INFO("Point_1st (x = %d,y = %d),Point_2st (x = %d,y = %d)\n", gesture.Point_1st.x, gesture.Point_1st.y, gesture.Point_2st.x, gesture.Point_2st.y);
	ILI_INFO("Point_3st (x = %d,y = %d),Point_4st (x = %d,y = %d)\n", gesture.Point_3st.x, gesture.Point_3st.y, gesture.Point_4st.x, gesture.Point_4st.y);
	ILI_INFO("idev->panel_wid = %d ,  idev->panel_hei = %d \n", ilits->panel_wid, ilits->panel_hei);
	ret = nlength;
	return ret;
}

int bbk_ili_set_charger_bit_v2_9882n(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	if(ilits->mpstatus){
	    ILI_INFO("mp mode ,skip send cmd");		
		return 0;
	}
	ilits->vivo_charger = state;
	ILI_INFO("state = %d\n", state);
	mutex_lock(&ilits->touch_mutex);
	switch (state) {
	case CHARGER_OFF:
		ILI_INFO("set charger off\n");
		ret = ili_ic_func_ctrl("plug", ENABLE);/* plug out*/
		break;
	case CHARGER_ON:
		ILI_INFO("set charger on\n");
		ret = ili_ic_func_ctrl("plug", DISABLE);/* plug in*/
		break;
	default:
		ILI_ERR("Unknown type, %d\n", state);
		ret = -1;
		break;
	}
	mutex_unlock(&ilits->touch_mutex);
	return ret;
}

int bbk_ili_read_charger_bit_v2_9882n(void)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };
	uint8_t data[2] = { 0 };
	mutex_lock(&ilits->touch_mutex);
	cmd[0] = 0x48;
	res = ilits->wrapper(cmd, sizeof(u8), data, sizeof(data), OFF, OFF);
	mutex_unlock(&ilits->touch_mutex);
	if (res < 0) {
		ILI_ERR("Failed to read fw charger %d\n", res);
		return -1;
	}
	if (data[1]) {
		ILI_INFO("data[1] = 1\n");
		return 1;
	} else {
		ILI_INFO("data[1] = 0\n");
		return 0;
	}
}

int bbk_ili_read_ic_mode(struct vts_device *vtsdev)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };
	uint8_t data[3] = { 0 };
	mutex_lock(&ilits->touch_mutex);
	cmd[0] = 0x4b;
	res = ilits->wrapper(cmd, sizeof(u8), data, sizeof(data), OFF, OFF);
	mutex_unlock(&ilits->touch_mutex);
	if (res < 0) {
		ILI_ERR("Failed to read ic mode \n");
		return -1;
	}
	if (data[1]) {
		ILI_INFO("gesutre mode = 1\n");
		return 1;
	} else {
		ILI_INFO("normal mode = 0\n");
		return 0;
	}
}

int setEdgeRestainSwitch_v2_9882n(struct vts_device *vtsdev, int on)
{
	int ret = 0;
	if(ilits->mpstatus){
	    ILI_INFO("mp mode ,skip send cmd");		
		return 0;
	}
	ilits->vivo_EdgeRestain = on;
	ILI_INFO("on = %d\n", on);
	mutex_lock(&ilits->touch_mutex);
	switch (on) {
	case EDGE_PALM_LEFT_UP_RIGHT_DOWN:
		ILI_INFO("Edge: left rotation\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x2);/*ilitek_tddi_edge_palm_ctrl(2);*/
		break;
	case EDGE_PALM_STRAIGHT:
		ILI_INFO("Edge: straight\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x1);/*ilitek_tddi_edge_palm_ctrl(1);*/
		break;
	case EDGE_PALM_LEFT_DOWN_RIGHT_UP:
		ILI_INFO("Edge: right rotation\n");
		ret = ili_ic_func_ctrl("edge_palm", 0x0);/*ilitek_tddi_edge_palm_ctrl(0);*/
		break;
	default:
		ILI_ERR("Unknown type, %d\n", on);
		ret = -1;
		break;
	}
	mutex_unlock(&ilits->touch_mutex);
	return ret;
}


int bbk_ili_set_gesture(struct vts_device *vtsdev, int gesture_state)
{
	ilits->ges_sym.alphabet_line_2_left = (gesture_state & (VTS_GESTURE_LR)) ? 1 : 0;
	ilits->ges_sym.alphabet_line_2_right = (gesture_state & (VTS_GESTURE_LR)) ? 1 : 0;
	ilits->ges_sym.alphabet_line_2_top = (gesture_state & (VTS_GESTURE_UP)) ? 1 : 0;
	ilits->ges_sym.alphabet_O = (gesture_state & (VTS_GESTURE_O)) ? 1 : 0;
	ilits->ges_sym.alphabet_w = (gesture_state & (VTS_GESTURE_W)) ? 1 : 0;
	ilits->ges_sym.alphabet_m = (gesture_state & (VTS_GESTURE_M)) ? 1 : 0;
	ilits->ges_sym.alphabet_E = (gesture_state & (VTS_GESTURE_E)) ? 1 : 0;
	ilits->ges_sym.alphabet_c = (gesture_state & (VTS_GESTURE_C)) ? 1 : 0;
	ilits->ges_sym.alphabet_line_2_bottom = (gesture_state & (VTS_GESTURE_DOWN)) ? 1 : 0;
	ilits->ges_sym.double_tap = (gesture_state & (VTS_GESTURE_DCLICK)) ? 1 : 0;
	ilits->ges_sym.alphabet_AT = (gesture_state & (VTS_GESTURE_A)) ? 1 : 0;
	ilits->ges_sym.alphabet_F = (gesture_state & (VTS_GESTURE_F)) ? 1 : 0;
	
	ili_set_gesture_symbol();
	return 0;
}



int bbk_ili_get_header_file_version_v2_9882n(int which, unsigned char *fw)
{
	int ret = 0;

	switch (which) {
	case GET_FW_CONFIG_VER:
		ILI_INFO("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ILI_INFO("get header fw version\n");
		ret = ((fw[FW_VER_ADDR] << 24) | (fw[FW_VER_ADDR + 1] << 16) |
				(fw[FW_VER_ADDR + 2] << 8) | (fw[FW_VER_ADDR + 3])) & 0x7fffffff ;
		break;
	default:
		ILI_ERR("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ILI_INFO("Header firmware version = %x\n", ret);
	return ret;
}

struct fw_debug_info {
	u8 id;
	u8 app_sys_powr_state_e : 3;
	u8 app_sys_state_e : 3;
	u8 tp_state_e : 2;
	u8 touch_palm_state_e : 2;
	u8 app_an_statu_e : 3;
	u8 app_sys_check_bg_abnormal : 1;
	u8 g_b_wrong_bg: 1;
	u8 reserved0 : 1;

	u8 status_of_dynamic_normal : 1;
	u8 status_of_dynamic_charger : 1;
	u8 status_of_dynamic_noise : 1;
	u8 reserved1 : 5;

	u32 algo_pt_status0 : 3;
	u32 algo_pt_status1 : 3;
	u32 algo_pt_status2 : 3;
	u32 algo_pt_status3 : 3;
	u32 algo_pt_status4 : 3;
	u32 algo_pt_status5 : 3;
	u32 algo_pt_status6 : 3;
	u32 algo_pt_status7 : 3;
	u32 algo_pt_status8 : 3;
	u32 algo_pt_status9 : 3;
	u32 reserved2 : 2;

	u8 nodp : 5;
	u8 high_byte_frequecy : 3;

	u8 low_byte_freqeucy;
};

int bbk_ili_get_fw_debug_info_v2_9882n(unsigned char *buf)
{
	int ret = 0, len = 0;
	struct fw_debug_info fdi;
	u8 cmd[1] = {0};
	u8 fw_buf[10] = {0};

	memset(&fdi, 0x0, sizeof(fdi));

	mutex_lock(&ilits->touch_mutex);

	cmd[0] = 0x47;

	ret = ilits->wrapper(cmd, sizeof(u8), fw_buf, sizeof(fw_buf), OFF, OFF);
	if (ret < 0) {
		ILI_ERR("read fw debug info error\n");
		ret = -1;
		goto out;
	}
	ili_dump_data(fw_buf, 8, sizeof(fw_buf), 0, "fw debug info");

	memcpy(&fdi, fw_buf, sizeof(fdi));

	len += sprintf(buf, "header = %d\n", fdi.id);
	len += sprintf(buf + len, "app_sys_powr_state_e = %d\n", fdi.app_sys_powr_state_e);
	len += sprintf(buf + len, "app_sys_state_e = %d\n", fdi.app_sys_state_e);
	len += sprintf(buf + len, "tp_state_e = %d\n", fdi.tp_state_e);
	len += sprintf(buf + len, "touch_palm_state_e = %d\n", fdi.touch_palm_state_e);
	len += sprintf(buf + len, "app_an_statu_e = %d\n", fdi.app_an_statu_e);
	len += sprintf(buf + len, "app_sys_check_bg_abnormal = %d\n", fdi.app_sys_check_bg_abnormal);
	len += sprintf(buf + len, "g_b_wrong_bg = %d\n", fdi.g_b_wrong_bg);

	len += sprintf(buf + len, "status_of_dynamic_normal = %d\n", fdi.status_of_dynamic_normal);
	len += sprintf(buf + len, "status_of_dynamic_charger = %d\n", fdi.status_of_dynamic_charger);
	len += sprintf(buf + len, "status_of_dynamic_noise = %d\n", fdi.status_of_dynamic_noise);

	len += sprintf(buf + len, "algo_pt_status0 = %d\n", fdi.algo_pt_status0);
	len += sprintf(buf + len, "algo_pt_status1 = %d\n", fdi.algo_pt_status1);
	len += sprintf(buf + len, "algo_pt_status2 = %d\n", fdi.algo_pt_status2);
	len += sprintf(buf + len, "algo_pt_status3 = %d\n", fdi.algo_pt_status3);
	len += sprintf(buf + len, "algo_pt_status4 = %d\n", fdi.algo_pt_status4);
	len += sprintf(buf + len, "algo_pt_status5 = %d\n", fdi.algo_pt_status5);
	len += sprintf(buf + len, "algo_pt_status6 = %d\n", fdi.algo_pt_status6);
	len += sprintf(buf + len, "algo_pt_status7 = %d\n", fdi.algo_pt_status7);
	len += sprintf(buf + len, "algo_pt_status8 = %d\n", fdi.algo_pt_status8);
	len += sprintf(buf + len, "algo_pt_status9 = %d\n", fdi.algo_pt_status9);

	len += sprintf(buf + len, "nodp = %d\n", fdi.nodp);
	len += sprintf(buf + len, "high_byte_frequecy = %d\n", fdi.high_byte_frequecy);
	len += sprintf(buf + len, "low_byte_freqeucy = %d\n", fdi.low_byte_freqeucy);

	ILI_INFO("id = %d\n", fdi.id);
	ILI_INFO("app_sys_powr_state_e = %d\n", fdi.app_sys_powr_state_e);
	ILI_INFO("app_sys_state_e = %d\n", fdi.app_sys_state_e);
	ILI_INFO("tp_state_e = %d\n", fdi.tp_state_e);
	ILI_INFO("touch_palm_state_e = %d\n", fdi.touch_palm_state_e);
	ILI_INFO("app_an_statu_e = %d\n", fdi.app_an_statu_e);
	ILI_INFO("app_sys_check_bg_abnormal = %d\n", fdi.app_sys_check_bg_abnormal);
	ILI_INFO("g_b_wrong_bg = %d\n", fdi.g_b_wrong_bg);
	ILI_INFO("status_of_dynamic_normal = %d\n", fdi.status_of_dynamic_normal);
	ILI_INFO("status_of_dynamic_charger = %d\n", fdi.status_of_dynamic_charger);
	ILI_INFO("status_of_dynamic_noise = %d\n", fdi.status_of_dynamic_noise);
	ILI_INFO("algo_pt_status0 = %d\n", fdi.algo_pt_status0);
	ILI_INFO("algo_pt_status1 = %d\n", fdi.algo_pt_status1);
	ILI_INFO("algo_pt_status2 = %d\n", fdi.algo_pt_status2);
	ILI_INFO("algo_pt_status3 = %d\n", fdi.algo_pt_status3);
	ILI_INFO("algo_pt_status4 = %d\n", fdi.algo_pt_status4);
	ILI_INFO("algo_pt_status5 = %d\n", fdi.algo_pt_status5);
	ILI_INFO("algo_pt_status6 = %d\n", fdi.algo_pt_status6);
	ILI_INFO("algo_pt_status7 = %d\n", fdi.algo_pt_status7);
	ILI_INFO("algo_pt_status8 = %d\n", fdi.algo_pt_status8);
	ILI_INFO("algo_pt_status9 = %d\n", fdi.algo_pt_status9);
	ILI_INFO("node = %d\n", fdi.nodp);
	ILI_INFO("low byte frequecy = %d\n", fdi.high_byte_frequecy);
	ILI_INFO("high byte frequecy = %d\n", fdi.low_byte_freqeucy);

out:
	mutex_unlock(&ilits->touch_mutex);
	return (ret < 0) ? ret : len;
}
