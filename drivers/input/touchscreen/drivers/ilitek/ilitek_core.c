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

#include "ilitek.h"


extern int mdss_dsi_panel_reset_and_powerctl(int enable);

enum {
	GET_FW_VER = 0,
	GET_FW_CONFIG_VER,
};

enum {
	CHARGER_OFF = 0,
	CHARGER_ON,
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

struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_QCOM,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
};

int bbk_ili_get_rawordiff_data_V2(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size)
{
	int i, row, col;
	int ret = 0,rlen = 0;
	int frame_count = 0;
	u8 cmd[2] = {0xFA, 0x0};
	u8 tp_mode = 0,checksum = 0;
	s16 temp;
	int retry = 5; 
	u16 self_key = 2;
	u8 *ptr;

	switch (which) {
	case ILI_RAWDATA:
		ipio_info("get raw data");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		frame_count = 1;
		break;
	case ILI_DIFFDATA:
		ipio_info("get delta data\n");
		cmd[1] = P5_X_FW_DELTA_DATA_MODE;
		frame_count = 1;
		break;
	case ILI_BASEDATA:
		ipio_info("get baseline data\n");
		cmd[1] = P5_X_FW_RAW_DATA_MODE;
		frame_count = 1;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		return -1;
	}
	//ilitek_tddi_fw_upgrade_handler_V2(NULL);//debug 
	rlen = (2 * idev_V2->xch_num * idev_V2->ych_num) + (idev_V2->stx * 2) + (idev_V2->srx * 2);
	rlen += 2 * self_key + (8 * 2) + 1 + 35;
	idev_V2->report = DISABLE;
	
	tp_mode = P5_X_FW_DEBUG_MODE;
	ret = ilitek_tddi_switch_mode_V2(&tp_mode);
	if (ret < 0)
		goto out;

	mdelay(50);
    idev_V2->debug_node_open = DISABLE;

	row = idev_V2->ych_num;
	col = idev_V2->xch_num;

	ipio_info("row = %d, col = %d\n", row, col);
	ipio_info("cmd = 0x%x, 0x%x\n", cmd[0], cmd[1]);

	mutex_lock(&idev_V2->touch_mutex);
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	// ret = idev->write(cmd, sizeof(cmd));
	ret = ilitek_thp_send_cmd(cmd, sizeof(cmd), NULL, 0);
	//ret = idev_V2->write(cmd, sizeof(cmd));
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	
	idev_V2->debug_data_frame = 0;
	idev_V2->debug_node_open = ENABLE;

   	while(retry--) {
   		mdelay(100);
		memset(idev_V2->p_rawdata, 0, 2048);
		idev_V2->read(idev_V2->p_rawdata, rlen);
		checksum = ilitek_calc_packet_checksum_V2(idev_V2->p_rawdata, rlen - 1);
		if(idev_V2->p_rawdata[0] == P5_X_DEBUG_PACKET_ID && checksum == idev_V2->p_rawdata[rlen-1] ) {
			ipio_info("debug packet checksum success !\n");
			break;
		}
   	}
	mutex_unlock(&idev_V2->touch_mutex);
	idev_V2->report = DISABLE;
	ptr = &idev_V2->p_rawdata[35];
	for (i = 0; i < row * col; i++, ptr += 2) {
		temp = (*ptr << 8) + *(ptr + 1);
		if (which != P5_X_FW_RAW_DATA_MODE)
			temp -= 0x10000;
		*((data + (row*(i % col))) + (i /col)) = temp;
	}

	mdelay(10);
out:
	idev_V2->debug_node_open = DISABLE;
	tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	mutex_lock(&idev_V2->touch_mutex);
	ilitek_tddi_switch_mode_V2(&tp_mode);
	mutex_unlock(&idev_V2->touch_mutex);	
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	return ret;
}
int bbk_ili_fw_update_for_V2(struct vts_device * vtsdev,const struct firmware *firmware)
{
  return bbk_ili_fw_update_V2(firmware);
}

int bbk_ili_fw_update_V2(const struct firmware *firmware)
{
	int ret = 0;

	if (!firmware) {
		ipio_err("firmware is null\n");
		return -ENOMEM;
	}

	if (atomic_read(&idev_V2->fw_stat)) {
		ipio_err("fw update is still running\n");
		return -1;
	}
	ret = ilitek_tddi_fw_upgrade_handler_V2(NULL);
	if(0){//100 == idev_V2->fw_update_stat
		ilitek_tddi_ic_get_tp_info_V2();
		ilitek_tddi_ic_get_panel_info_V2();
	}
	return ret;
}

int iliIdleEnableOrDisable_V2(struct vts_device *vtsdev,int state)
{
	int ret = 0;
	u8 cmd[2] = {0};

	ipio_info("idle %s\n", (state == ENABLE) ? "ON" : "OFF");
	mutex_lock(&idev_V2->touch_mutex);
	cmd[0] = 0x14;
	cmd[1] = 0xFF & state;
	ipio_info("idle cmd = 0x%x, 0x%x\n", cmd[0], cmd[1]);
	// ret = idev->write(cmd, sizeof(cmd));
	ret = ilitek_thp_send_cmd(cmd, sizeof(cmd), NULL, 0);
	mutex_unlock(&idev_V2->touch_mutex);
	return ret;
}

int bbk_ili_readUdd_V2(unsigned char *udd)
{
	return 0;
}

int bbk_ili_writeUdd_V2(unsigned char *udd)
{
	return 0;
}

int bbk_ili_mode_change_V2(struct vts_device *vtsdev,int which)
{
	int ret = 0;
	static int mode;	
	ipio_info("which = %d\n", which);
	switch (which) {
	case VTS_ST_SLEEP:
		VTI("deep sleep mode");
		if (mode == VTS_ST_NORMAL) {
			idev_V2->gesture_V2 = DISABLE;
			ret = ilitek_tddi_sleep_handler_V2(TP_DEEP_SLEEP);
		} else {
			idev_V2->gesture_V2 = DISABLE;
			idev_V2->tp_suspend = true;	
			ilitek_tddi_ic_func_ctrl_V2("sleep", DEEP_SLEEP_IN);
			idev_V2->tp_suspend = false;	
		}
		ilitek_plat_irq_disable_V2();
		mdelay(35);
		
	//	mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
		//mdss_dsi_panel_reset_and_powerctl(0);
		vts_dsi_panel_reset_power_ctrl(0);
	//	mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));		
		mode = VTS_ST_SLEEP;
		//idev_V2->tp_suspend = true;
		mdelay(10);
		break;
	case VTS_ST_NORMAL:
		//idev_V2->tp_suspend = false;
		VTI("resume to normal mode");		
		ret = ilitek_tddi_sleep_handler_V2(TP_RESUME);
		mode = VTS_ST_NORMAL;
		break;
	case VTS_ST_GESTURE:
		VTI("suspend with gesture_V2 mode");
		if (mode ==  VTS_ST_GESTURE) {
			VTI("Already in gesture_V2 mode break");
			break;
		}
		if (mode == VTS_ST_NORMAL) {
			idev_V2->gesture_V2 = ENABLE;
			ret = ilitek_tddi_sleep_handler_V2(TP_SUSPEND);
		} else {			
		//	mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
			vts_dsi_panel_reset_power_ctrl(1);
			mdelay(50);
		//	mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));		
			idev_V2->gesture_V2 = ENABLE;
			ilitek_tddi_gesture_recovery_V2();
			enable_irq_wake(idev_V2->irq_num);
			ilitek_plat_irq_enable_V2();
		}
		mode = VTS_ST_GESTURE;
		//idev_V2->tp_suspend = true;
		break;
	}
	return ret;
}
int bbk_ili_get_fw_version_for_V2(struct vts_device *vtsdev, u64 *version)
{  
   int fw_ver = 0;
   fw_ver = bbk_ili_get_fw_version_V2(GET_FW_VER);
   *version = fw_ver;
   return 0;
}
int bbk_ili_get_fw_version_V2(int which)
{
	int ret = 0;

	ipio_info("which = %d\n", which);

	switch (which) {
	case GET_FW_CONFIG_VER:
		ipio_info("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ipio_info("get fw version\n");
		mutex_lock(&idev_V2->touch_mutex);
		ilitek_tddi_ic_get_fw_ver_V2();
		mutex_unlock(&idev_V2->touch_mutex);				
		ret = idev_V2->chip->fw_ver & 0xffff;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ipio_info("firmware version = %x\n", ret);
	return ret;
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

int bbk_ili_gesture_point_get_V2(u16 *data)
{
	int ret = 0;
	u16 xx[6];
	u16 yy[6];
	uint8_t gesture_id = 0;
	uint8_t score = 0;
	int nlength = 0;
	u16 x1, y1, x2, y2;
	struct gesture_buf gesture_V2;

	if (!data) {
		ipio_err("data is null\n");
		return -1;
	}

	if (idev_V2->debug_buf == NULL) {
		VTI("idev_V2->debug_buf NULL");
		ilitek_dump_data_V2(idev_V2->debug_buf, 8, P5_X_GESTURE_INFO_LENGTH + 1, 0, "VIVO_TS gesture_V2 before");
		return 0;
	}

	gesture_id = idev_V2->debug_buf[0][1];
	score = idev_V2->debug_buf[0][36];

	x1 = (((idev_V2->debug_buf[0][4] & 0xF0) << 4) | (idev_V2->debug_buf[0][5]));
	y1 = (((idev_V2->debug_buf[0][4] & 0x0F) << 8) | (idev_V2->debug_buf[0][6]));
	x2 = (((idev_V2->debug_buf[0][7] & 0xF0) << 4) | (idev_V2->debug_buf[0][8]));
	y2 = (((idev_V2->debug_buf[0][7] & 0x0F) << 8) | (idev_V2->debug_buf[0][9]));

	xx[0] = (((idev_V2->debug_buf[0][28] & 0xF0) << 4) | (idev_V2->debug_buf[0][29]));
	yy[0] = (((idev_V2->debug_buf[0][28] & 0x0F) << 8) | (idev_V2->debug_buf[0][30]));
	xx[1] = (((idev_V2->debug_buf[0][31] & 0xF0) << 4) | (idev_V2->debug_buf[0][32]));
	yy[1] = (((idev_V2->debug_buf[0][31] & 0x0F) << 8) | (idev_V2->debug_buf[0][33]));

	xx[2] = xx[0] + ((xx[1] - xx[0])/2);
	yy[2] = yy[0];
	xx[3] = xx[0];
	yy[3] = yy[0] + ((yy[1] - yy[0])/2);
	xx[4] = xx[0] + ((xx[1] - xx[0])/2);
	yy[4] = yy[1];
	xx[5] = xx[1];
	yy[5] = yy[0] + ((yy[1] - yy[0])/2);

	gesture_V2.Point_start.x = x1*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_start.y = y1*idev_V2->panel_hei/TPD_HEIGHT;
	gesture_V2.Point_end.x  = x2*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_end.y  = y2*idev_V2->panel_hei/TPD_HEIGHT;
	gesture_V2.Point_1st.x   = xx[2]*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_1st.y   = yy[2]*idev_V2->panel_hei/TPD_HEIGHT;
	gesture_V2.Point_2st.x   = xx[3]*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_2st.y   = yy[3]*idev_V2->panel_hei/TPD_HEIGHT;
	gesture_V2.Point_3st.x   = xx[4]*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_3st.y   = yy[4]*idev_V2->panel_hei/TPD_HEIGHT;
	gesture_V2.Point_4st.x   = xx[5]*idev_V2->panel_wid/TPD_WIDTH;
	gesture_V2.Point_4st.y   = yy[5]*idev_V2->panel_hei/TPD_HEIGHT;

/*
	gesture_V2.Point_4st.x   = (((point_data[25] & 0xF0) << 4) | (point_data[26]));
	gesture_V2.Point_4st.y   = (((point_data[25] & 0x0F) << 8) | (point_data[27]));
*/
     /*judge gesture_V2 type  */
	switch (gesture_id) {
	case GESTURE_UP:
		*data       = gesture_V2.Point_start.x;
		*(data + 1) = gesture_V2.Point_start.y;
		*(data + 2) = gesture_V2.Point_end.x;
		*(data + 3) = gesture_V2.Point_end.y;
		nlength = 2;
		break;
	case GESTURE_W:
		gesture_V2.Point_1st.x = (((idev_V2->debug_buf[0][16] & 0xF0) << 4) | (idev_V2->debug_buf[0][17]))*idev_V2->panel_wid/TPD_WIDTH;
		gesture_V2.Point_1st.y = (((idev_V2->debug_buf[0][16] & 0x0F) << 8) | (idev_V2->debug_buf[0][18]))*idev_V2->panel_hei/TPD_HEIGHT;
		gesture_V2.Point_2st.x = (((idev_V2->debug_buf[0][19] & 0xF0) << 4) | (idev_V2->debug_buf[0][20]))*idev_V2->panel_wid/TPD_WIDTH;
		gesture_V2.Point_2st.y = (((idev_V2->debug_buf[0][19] & 0x0F) << 8) | (idev_V2->debug_buf[0][21]))*idev_V2->panel_hei/TPD_HEIGHT;
		gesture_V2.Point_3st.x = (((idev_V2->debug_buf[0][22] & 0xF0) << 4) | (idev_V2->debug_buf[0][23]))*idev_V2->panel_wid/TPD_WIDTH;
		gesture_V2.Point_3st.y = (((idev_V2->debug_buf[0][22] & 0x0F) << 8) | (idev_V2->debug_buf[0][24]))*idev_V2->panel_hei/TPD_HEIGHT;
		*data     = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_1st.x;
		*(data + 3)  = gesture_V2.Point_1st.y;
		*(data + 4)  = gesture_V2.Point_2st.x;
		*(data + 5)  = gesture_V2.Point_2st.y;
		*(data + 6)  = gesture_V2.Point_3st.x;
		*(data + 7)  = gesture_V2.Point_3st.y;
		*(data + 8)  = gesture_V2.Point_end.x;
		*(data + 9)  = gesture_V2.Point_end.y;
		nlength = 5;
		break;
	case GESTURE_E:
		*data        = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_end.x;
		*(data + 3)  = gesture_V2.Point_end.y;
		*(data + 4)  = gesture_V2.Point_1st.x;
		*(data + 5)  = gesture_V2.Point_1st.y;
		*(data + 6)  = gesture_V2.Point_2st.x;
		*(data + 7)  = gesture_V2.Point_2st.y;
		*(data + 8)  = gesture_V2.Point_3st.x;
		*(data + 9)  = gesture_V2.Point_3st.y;
		*(data + 10) = gesture_V2.Point_4st.x;
		*(data + 11) = gesture_V2.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_C:
		*data        = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_2st.x;
		*(data + 3)  = gesture_V2.Point_2st.y;
		*(data + 4)  = gesture_V2.Point_end.x;
		*(data + 5)  = gesture_V2.Point_end.y;
		*(data + 6)  = gesture_V2.Point_1st.x;
		*(data + 7)  = gesture_V2.Point_1st.y;
		*(data + 8)  = gesture_V2.Point_3st.x;
		*(data + 9)  = gesture_V2.Point_3st.y;
		*(data + 10) = gesture_V2.Point_4st.x;
		*(data + 11) = gesture_V2.Point_4st.y;
		nlength = 3;
		break;
	case GESTURE_O:
		*data        = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_end.x;
		*(data + 3)  = gesture_V2.Point_end.y;
		*(data + 4)  = gesture_V2.Point_1st.x;
		*(data + 5)  = gesture_V2.Point_1st.y;
		*(data + 6)  = gesture_V2.Point_2st.x;
		*(data + 7)  = gesture_V2.Point_2st.y;
		*(data + 8)  = gesture_V2.Point_3st.x;
		*(data + 9)  = gesture_V2.Point_3st.y;
		*(data + 10) = gesture_V2.Point_4st.x;
		*(data + 11) = gesture_V2.Point_4st.y;
		if (idev_V2->debug_buf[0][34]) {/*point_data[34] = 1 clockwise*/
			//
			//vtsGesturePointsReport(VTS_GESTURE_O_DIR,1,-1,-1); 
		}
		nlength = 6;
		break;
	case GESTURE_F:
		*data        = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_end.x;
		*(data + 3)  = gesture_V2.Point_end.y;
		*(data + 4)  = gesture_V2.Point_1st.x;
		*(data + 5)  = gesture_V2.Point_1st.y;
		*(data + 6)  = gesture_V2.Point_2st.x;
		*(data + 7)  = gesture_V2.Point_2st.y;
		*(data + 8)  = gesture_V2.Point_3st.x;
		*(data + 9)  = gesture_V2.Point_3st.y;
		*(data + 10) = gesture_V2.Point_4st.x;
		*(data + 11) = gesture_V2.Point_4st.y;
		nlength = 6;
		break;
	case GESTURE_A:
		*data        = gesture_V2.Point_start.x;
		*(data + 1)  = gesture_V2.Point_start.y;
		*(data + 2)  = gesture_V2.Point_end.x;
		*(data + 3)  = gesture_V2.Point_end.y;
		*(data + 4)  = gesture_V2.Point_1st.x;
		*(data + 5)  = gesture_V2.Point_1st.y;
		*(data + 6)  = gesture_V2.Point_2st.x;
		*(data + 7)  = gesture_V2.Point_2st.y;
		*(data + 8)  = gesture_V2.Point_3st.x;
		*(data + 9)  = gesture_V2.Point_3st.y;
		*(data + 10) = gesture_V2.Point_4st.x;
		*(data + 11) = gesture_V2.Point_4st.y;
		nlength = 6;
		break;
	default:
		nlength = -1;
		break;
	}
	ipio_info("Point_start (x = %d,y = %d),Point_end (x = %d,y = %d)\n", gesture_V2.Point_start.x, gesture_V2.Point_start.y, gesture_V2.Point_end.x, gesture_V2.Point_end.y);
	ipio_info("Point_1st (x = %d,y = %d),Point_2st (x = %d,y = %d)\n", gesture_V2.Point_1st.x, gesture_V2.Point_1st.y, gesture_V2.Point_2st.x, gesture_V2.Point_2st.y);
	ipio_info("Point_3st (x = %d,y = %d),Point_4st (x = %d,y = %d)\n", gesture_V2.Point_3st.x, gesture_V2.Point_3st.y, gesture_V2.Point_4st.x, gesture_V2.Point_4st.y);
	ipio_info("idev_V2->panel_wid = %d ,  idev_V2->panel_hei = %d \n", idev_V2->panel_wid, idev_V2->panel_hei);
	idev_V2->debug_node_open = DISABLE;
	ret = nlength;
	return ret;
}

int bbk_ili_set_charger_bit_V2(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	if(idev_V2->mp_mode){
	    ipio_info("mp mode ,skip send cmd");		
		return 0;
	}

	ipio_info("state = %d\n", state);	
	mutex_lock(&idev_V2->touch_mutex);

	switch (state) {
	case CHARGER_OFF:
		ipio_info("set charger off\n");
		ret = ilitek_tddi_ic_func_ctrl_V2("plug", ENABLE);/* plug out*/
		break;
	case CHARGER_ON:
		ipio_info("set charger on\n");
		ret = ilitek_tddi_ic_func_ctrl_V2("plug", DISABLE);/* plug in*/
		break;
	default:
		ipio_err("Unknown type, %d\n", state);
		ret = -1;
		break;
	}	
	
	mutex_unlock(&idev_V2->touch_mutex);
	return ret;
}

int bbk_ili_read_charger_bit_V2(void)
{
		int res = 0;
		uint8_t cmd[2] = { 0 };
		uint8_t data[2] = { 0 };
		mutex_lock(&idev_V2->touch_mutex);
		cmd[0] = 0x48;
		// idev->write(cmd, 1);
	
		// mdelay(10);
	
		// res = idev->read(cmd, 2);
		res = ilitek_thp_send_cmd(cmd, sizeof(u8), data, sizeof(data));
		mutex_unlock(&idev_V2->touch_mutex);
		if (res < 0) {
			ipio_err("Failed to read fw charger %d\n", res);
			return -1;
		}
		if (data[1]) {
			ipio_info("data[1] = 1\n");
			return 1;
		} else {
			ipio_info("data[1] = 0\n");
			return 0;
		}
}


int iliSetEdgeRestainSwitch_V2(struct vts_device *vtsdev,int on)
{
	int ret = 0;
	if(idev_V2->mp_mode){
	    ipio_info("mp mode ,skip send cmd");		
		return 0;
	}

	ipio_info("on = %d\n", on);	
	mutex_lock(&idev_V2->touch_mutex);
	
	switch (on) {
	case EDGE_PALM_LEFT_UP_RIGHT_DOWN:
		ipio_info("Edge: left rotation\n");
		ret = ilitek_tddi_ic_func_ctrl_V2("edge_palm", 0x2);/*ilitek_tddi_edge_palm_ctrl(2);*/
		break;
	case EDGE_PALM_STRAIGHT:
		ipio_info("Edge: straight\n");
		ret = ilitek_tddi_ic_func_ctrl_V2("edge_palm", 0x1);/*ilitek_tddi_edge_palm_ctrl(1);*/
		break;
	case EDGE_PALM_LEFT_DOWN_RIGHT_UP:
		ipio_info("Edge: right rotation\n");
		ret = ilitek_tddi_ic_func_ctrl_V2("edge_palm", 0x0);/*ilitek_tddi_edge_palm_ctrl(0);*/
		break;
	default:
		ipio_err("Unknown type, %d\n", on);
		ret = -1;
		break;
	}
	
	mutex_unlock(&idev_V2->touch_mutex);
	return ret;
}

int bbk_ili_get_header_file_version_V2(int which, unsigned char *fw)
{
	int ret = 0;

	switch (which) {
	case GET_FW_CONFIG_VER:
		ipio_info("no config version\n");
		ret = 0;
		break;
	case GET_FW_VER:
		ipio_info("get header fw version\n");
		ret = ((fw[FW_VER_ADDR] << 24) | (fw[FW_VER_ADDR + 1] << 16) |
				(fw[FW_VER_ADDR + 2] << 8) | (fw[FW_VER_ADDR + 3])) & 0x7fffffff ;
		break;
	default:
		ipio_err("Unknown type, %d\n", which);
		ret = -1;
		break;
	}
	ipio_info("Header firmware version = %x\n", ret);
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
int bbk_ili_get_fw_debug_info_V2(unsigned char *buf)
{
		int ret = 0, len = 0;
        struct fw_debug_info fdi;
        u8 cmd[1] = {0};
        u8 fw_buf[10] = {0};

        memset(&fdi, 0x0, sizeof(fdi));

        mutex_lock(&idev_V2->touch_mutex);

        cmd[0] = 0x47;
		#if 0
        ret = idev_V2->write(cmd, 1);
        if (ret < 0) {
                ipio_err("write 0x47 cmd error\n");
                ret = -1;
                goto out;
        }

        ret = idev_V2->read(fw_buf, sizeof(fw_buf));
        if (ret < 0) {
                ipio_err("read fw debug_V2 info error\n");
                ret = -1;
                goto out;
        }
		#endif
		ret = ilitek_thp_send_cmd(cmd, sizeof(u8), fw_buf, sizeof(fw_buf));
        if (ret < 0) {
                ipio_err("read fw debug info error\n");
                ret = -1;
                goto out;
        }
		
        ilitek_dump_data_V2(fw_buf, 8, sizeof(fw_buf), 0, "fw debug_V2 info");

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

	ipio_info("id = %d\n", fdi.id);
	ipio_info("app_sys_powr_state_e = %d\n", fdi.app_sys_powr_state_e);
	ipio_info("app_sys_state_e = %d\n", fdi.app_sys_state_e);
	ipio_info("tp_state_e = %d\n", fdi.tp_state_e);
	ipio_info("touch_palm_state_e = %d\n", fdi.touch_palm_state_e);
	ipio_info("app_an_statu_e = %d\n", fdi.app_an_statu_e);
	ipio_info("app_sys_check_bg_abnormal = %d\n", fdi.app_sys_check_bg_abnormal);
	ipio_info("g_b_wrong_bg = %d\n", fdi.g_b_wrong_bg);
    ipio_info("status_of_dynamic_normal = %d\n", fdi.status_of_dynamic_normal);
    ipio_info("status_of_dynamic_charger = %d\n", fdi.status_of_dynamic_charger);
    ipio_info("status_of_dynamic_noise = %d\n", fdi.status_of_dynamic_noise);
	ipio_info("algo_pt_status0 = %d\n", fdi.algo_pt_status0);
	ipio_info("algo_pt_status1 = %d\n", fdi.algo_pt_status1);
	ipio_info("algo_pt_status2 = %d\n", fdi.algo_pt_status2);
	ipio_info("algo_pt_status3 = %d\n", fdi.algo_pt_status3);
	ipio_info("algo_pt_status4 = %d\n", fdi.algo_pt_status4);
	ipio_info("algo_pt_status5 = %d\n", fdi.algo_pt_status5);
	ipio_info("algo_pt_status6 = %d\n", fdi.algo_pt_status6);
	ipio_info("algo_pt_status7 = %d\n", fdi.algo_pt_status7);
	ipio_info("algo_pt_status8 = %d\n", fdi.algo_pt_status8);
	ipio_info("algo_pt_status9 = %d\n", fdi.algo_pt_status9);
	ipio_info("node = %d\n", fdi.nodp);
	ipio_info("low byte frequecy = %d\n", fdi.high_byte_frequecy);
	ipio_info("high byte frequecy = %d\n", fdi.low_byte_freqeucy);

out:
        mutex_unlock(&idev_V2->touch_mutex);
        return (ret < 0) ? ret : len;

}


int vivo_ili_module_id_V2(struct vts_device *vtsdev)
{
	/*just for display in *#225# view*/
	return VTS_MVC_TRY; 
}
int bbk_ili_read_ic_mode_V2(struct vts_device *vtsdev)
{
	int res = 0;
	uint8_t cmd[2] = { 0 };
	uint8_t data[2] = { 0 };
	mutex_lock(&idev_V2->touch_mutex);
	cmd[0] = 0x4b;
	res = ilitek_thp_send_cmd(cmd, sizeof(u8), data, sizeof(data));
	mutex_unlock(&idev_V2->touch_mutex);
	if (res < 0) {
		ipio_err("Failed to read ic mode \n");
		return -1;
	}
	if (data[1]) {
		ipio_info("gesutre mode = 0\n");
		return 0;
	} else {
		ipio_info("normal mode = 1\n");
		return 1;
	}
}

static const struct vts_operations ili_vts_ops = {
	.init = ilitek_hw_init,
	.change_mode = bbk_ili_mode_change_V2,
	.get_frame = bbk_ili_get_rawordiff_data_V2,
	.get_fw_version = bbk_ili_get_fw_version_for_V2,
	.get_ic_mode = bbk_ili_read_ic_mode_V2,
	.set_charging = bbk_ili_set_charger_bit_V2,
	.set_auto_idle = iliIdleEnableOrDisable_V2,
	.update_firmware = bbk_ili_fw_update_for_V2,
	.set_rotation = iliSetEdgeRestainSwitch_V2,
};
int ilitek_plat_probe(void)
{
	struct vts_device *vtsdev = NULL;
	int ret;
	idev_V2->hwif = &hwif;
	VTI("start..");
	vtsdev = vts_device_alloc();
	if(!vtsdev) {
		VTE("vivoTsAlloc fail");
		ret = -ENOMEM;
		goto errocode9;
	}

	vtsdev->ops = &ili_vts_ops;
	vtsdev->busType = BUS_SPI;
	idev_V2->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, idev_V2->node);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto errocode9;
	}
	vts_set_drvdata(vtsdev, idev_V2);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {		
		VTE("vts_register_driver failed");
		
		goto errorcode10;
	}

	return 0;

	
errorcode10:
	vts_unregister_driver(vtsdev);
errocode9:
	vts_device_free(vtsdev);
	return ret;
}

/*
extern unsigned int report_lcm_id(void);
int get_lcm_id(void)
{

	int lcm_id;

	lcm_id = report_lcm_id();

	VTI("lcm id: 0x%x", lcm_id);

	return lcm_id;
} */

