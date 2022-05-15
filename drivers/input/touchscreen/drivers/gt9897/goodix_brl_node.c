#include "goodix_ts_core.h"

#define BBK_USER_DATA_MAX_LEN			64
#define BBK_USER_DATA_ADDR			0x10028




extern struct goodix_module goodix_modules_gt9897;

#define CFG_VER 0x1017f
int bbk_gdix_brl_get_fw_version(struct vts_device *vtsdev, u64 *version)
{
	int ret = -1;
	u8 cfg_ver;
    int firmware_version = 0;
	struct goodix_ts_core *cd = goodix_modules_gt9897.core_data;
	struct goodix_fw_version chip_ver;
	if(0 == cd->cfg_has_parsed){
		ts_err("cfg has not parsed ,return");
		return -EPERM;

	}
		if (cd->hw_ops->read_version) {
			ret = cd->hw_ops->read_version(cd, &chip_ver);
			if (ret) {
				ts_err("Read fw version fail.");
				return -EPERM;
			}
			ts_info("VID:%02x %02x %02x %02x", chip_ver.patch_vid[0], chip_ver.patch_vid[1],
						chip_ver.patch_vid[2], chip_ver.patch_vid[3]);
			firmware_version = (chip_ver.patch_vid[0] << 24) | (chip_ver.patch_vid[1] << 16) 
					| (chip_ver.patch_vid[2] << 8) | chip_ver.patch_vid[3];

		ret = cd->hw_ops->read(cd, CFG_VER,
				&cfg_ver, 1);
		if (ret) {
				ts_err("Read fg version fail.");
				return -EPERM;
			}
		ts_info("cfg version:0x%x",cfg_ver);
		*version =(((u64)firmware_version) << 16) | ((u64)cfg_ver);
		} 
	
	return ret;
}
/*Jarvis 2020-7-6 end*/
/*1 = auto enter idle;0 = keep active*/
#define GOODIX_AUTOIDLE_CMD 0x1B
#define GOODIX_KEEPACTIVE_CMD 0x1A
int bbk_gdix_brl_idleEnableOrDisable(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;

	if (1 == state) {   //geme out
		cmd.cmd = GOODIX_AUTOIDLE_CMD;
		cmd.len = 4;
		ret = core_data->hw_ops->send_cmd(core_data, &cmd);
		if (ret)
			ts_err("failed send auto idle cmd");
	} else if (0 == state) {    //game in
		cmd.cmd = GOODIX_KEEPACTIVE_CMD;
		cmd.len = 7;
		cmd.data[0] = 0x01;
		cmd.data[1] = 0x0A;
		cmd.data[2] = 0x02;
		cmd.data[3] = 0x0A;
		cmd.data[4] = 0x2C;
		cmd.data[5] = 0x01;
		ret = core_data->hw_ops->send_cmd(core_data, &cmd);
		if (ret)
			ts_err("failed send keep active cmd");
	} else {
		ts_err("Invalid set idle params");
		return -EPERM;
	}

	return ret;
}

/* get resoluton while screen on */
static int goodix_get_resolution(struct goodix_ts_core *cd)
{
	struct vts_device *vtsdev = cd->vtsdev;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int resolution = 0;
	int display_x = 1;
	int display_y = 1;
	int dimention_x = 1;
	int dimention_y = 1;

	vts_property_get(vtsdev, VTS_PROPERTY_RESOLUTION_ADJUST, &resolution);
	if (resolution) {
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_X, &display_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DISPLAY_Y, &display_y);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_X, &dimention_x);
		vts_property_get(vtsdev, VTS_PROPERTY_DIMENTION_Y, &dimention_y);
		hw_ops->read_version(cd, &cd->fw_version);
		VTI("vid[0]: %d", cd->fw_version.patch_vid[0]);
		if(cd->fw_version.patch_vid[0] == 0x00) {
			VTI("normal resolution");
			vtsdev->fw_x = display_x;
			vtsdev->fw_y = display_y;
		} else {
			VTI("high resolution");
			vtsdev->fw_x = dimention_x;
			vtsdev->fw_y = dimention_y;
		}
		VTI("fw resolution is %d * %d", vtsdev->fw_x, vtsdev->fw_y);
	} else {
		VTI("not high resolution");
	}

	return 0;
}

int bbk_gdix_brl_get_fw_resolution(struct vts_device *vtsdev, int enable)
{
	struct goodix_ts_core *core_data = vts_get_drvdata(vtsdev);

	if (!enable) {
		VTI("log switch not open !");
		return 0;
	}
	goodix_get_resolution(core_data);
	return 0;
}

ssize_t bbk_gdix_brl_readUdd(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	int ret = 0;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
	ret = core_data->hw_ops->read(core_data, BBK_USER_DATA_ADDR, buf, 15);
	if (ret) {
		ts_err("User data spi read failed");
		ret = -1;
	} else {
		ret = nbytes;
	}

	return ret;
}

#define GOODIX_WRITE_UDD_CMD 0x80
ssize_t bbk_gdix_brl_writeUdd(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	int ret = 0;
	int i;
	u16 checksumU16 = 0;
	unsigned char buf_temp[BBK_USER_DATA_MAX_LEN] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;

	memcpy(buf_temp, buf, 15);
	buf_temp[BBK_USER_DATA_MAX_LEN - 3] = 0xAA;
	for (i = 0; i < BBK_USER_DATA_MAX_LEN - 2; i++) {
		checksumU16 += buf_temp[i];
	}
	buf_temp[BBK_USER_DATA_MAX_LEN - 2] = checksumU16 & 0xFF;
	buf_temp[BBK_USER_DATA_MAX_LEN - 1] = (checksumU16 >> 8) & 0xFF;

	ret = core_data->hw_ops->write(core_data, BBK_USER_DATA_ADDR,
					buf_temp, BBK_USER_DATA_MAX_LEN);
	if (ret) {
		ts_err("User data i2c write failed");
		return -EPERM;
	}

	cmd.cmd = GOODIX_WRITE_UDD_CMD;
	cmd.len = 4;
	ret = core_data->hw_ops->send_cmd(core_data, &cmd);
	if (ret) {
		ts_err("Send write udd command error");
		return -EPERM;
	}

	return ret;
}
/*Jarvis 2020-7-7 end*/

/*Jarvis 2020-7-9 Start*/
/*0-usb at right;1-usb at bottom;2-usb at left*/
#define GOODIX_DIRECTION_CMD 0x17
int bbk_gdix_setEdgeRestainSwitch(struct vts_device *vtsdev, int on)
{
	int ret = 0;
	struct goodix_ts_cmd direction_cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;

	if (1 == on) {
		direction_cmd.cmd = GOODIX_DIRECTION_CMD;
		direction_cmd.data[0] = 0x00;
		direction_cmd.len = 5;
		ret = core_data->hw_ops->send_cmd(core_data, &direction_cmd);
	} else if (0 == on) {
		direction_cmd.cmd = GOODIX_DIRECTION_CMD;
		direction_cmd.data[0] = 0x02;
		direction_cmd.len = 5;
		ret = core_data->hw_ops->send_cmd(core_data, &direction_cmd);
	} else if (2 == on) {
		direction_cmd.cmd = GOODIX_DIRECTION_CMD;
		direction_cmd.data[0] = 0x01;
		direction_cmd.len = 5;
		ret = core_data->hw_ops->send_cmd(core_data, &direction_cmd);
	} else {
		ts_err("Invalid setEdgeRestainSwitch params");
		return -EPERM;
	}

	return ret;
}

/*Mao 2020-8-3 Start*/
/*0x14,0x00-enable;0x14,0x01-disable;0x1E,0xFF-finger mode;0x1E,0x00-finger mode off*/
#define BBK_USER_FingerCmd_ADDR	    0x10180
#define GOODIX_FP_OPNCLS_CMD 0x14
#define GOODIX_FP_MVNSTC_CMD 0x1E
int setFingerGesture(int state)  
{
	int ret = 0;
	struct goodix_ts_cmd fp_cmd; /*0-enable fp; 1-disable fp;3--move; 4-static*/
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
 
    switch (state) {
		case 1:
		    fp_cmd.cmd = GOODIX_FP_OPNCLS_CMD;
		    fp_cmd.data[0] = 0x00;
		    fp_cmd.len = 5;
		    ret = core_data->hw_ops->send_cmd(core_data, &fp_cmd);
		    break;
        case 0:
			fp_cmd.cmd = GOODIX_FP_OPNCLS_CMD;
		    fp_cmd.data[0] = 0x01;
		    fp_cmd.len = 5;
		    ret = core_data->hw_ops->send_cmd(core_data, &fp_cmd);
			break;
		case 2://move
			fp_cmd.cmd = GOODIX_FP_MVNSTC_CMD;
		    fp_cmd.data[0] = 0xFF;
		    fp_cmd.len = 5;
		    ret = core_data->hw_ops->send_cmd(core_data, &fp_cmd);
			break;
		case 3://static
			fp_cmd.cmd = GOODIX_FP_MVNSTC_CMD;
		    fp_cmd.data[0] = 0x00;
		    fp_cmd.len = 5;
		    ret = core_data->hw_ops->send_cmd(core_data, &fp_cmd);
			break;
		default :
			VTE("invalid param in setFingerGesture");
			break;
}
	return ret;
}

/*Mao 2020-8-3 End*/


#define GOODIX_COOR_MODE_CMD 0x00
int bbk_gdix_brl_be_normal(void)
{
	int ret = -1;
	u8 sync_clean = 0x00;
	struct goodix_ts_cmd normal_cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
	struct goodix_ic_info_misc *misc = &core_data->ic_info.misc;

	ret = core_data->hw_ops->write(core_data, misc->touch_data_addr,
		&sync_clean, 1);
	if (ret) {
		ts_err("Before be_normal,Set zero to interrupt status reg fail.");
		ret = -EINVAL;
	}

	normal_cmd.cmd = GOODIX_COOR_MODE_CMD;
	normal_cmd.len = 4;
	ret = core_data->hw_ops->send_cmd(core_data, &normal_cmd);
	if (ret)
		ts_err("Send enter normal mode command fail.");

	usleep_range(10000, 11000);

	ret = core_data->hw_ops->write(core_data, misc->touch_data_addr,
		&sync_clean, 1);
	if (ret) {
		ts_err("After be_normal,Set zero to interrupt status reg fail.");
		ret = -EINVAL;
	}

	return ret;
}

#define GOODIX_DATA_MODE_CMD 0x01
int bbk_gdix_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	int i, wait_cnt, ret = 0;
	int sen_num = 0, drv_num = 0;
	u8 sync_clean = 0x00;
	u8 *temp_data = NULL;
	s16 *result_data = NULL;
	int data_size = 0, data_len = 0;
	u8 buf[2] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_gt9897.core_data;
	struct goodix_ic_info_misc *misc = &core_data->ic_info.misc;
	if(0 == core_data->ic_info.misc.cmd_addr) {
		ts_err("misc is null");
		return -EINVAL;
	}


	if ((type != 0) && (type != 1) && (type != 2)) {
		ts_err("Invalid get rawdiff data params.");
		return -EINVAL;
	}

	//TODO: get sen_num/drv_num from dts!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//you can also get from:core_data->ic_info->parm->drv_num

	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, (u32 *)&drv_num);
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, (u32 *)&sen_num);
	if (ret) {
		ts_err("Get channel num fail.");
		return -EINVAL;
	}
	VTI("txNum:%d--rxNum:%d", drv_num, sen_num);
	data_size = sen_num * drv_num;
	data_len = data_size * sizeof(u16);
	if (data == NULL) {
		ts_err("User data format error.");
		return -EINVAL;
	}

	if(!misc->touch_data_addr || !misc->mutual_diffdata_addr
	  || !misc->mutual_rawdata_addr || !misc->mutual_refdata_addr) {
		ts_err("touch_data_addr mutual_rawdata_addr mutual_rawdata_addr or mutual_refdata_addr is null");
		return -EINVAL;
	}

	temp_data = kzalloc(data_len, GFP_KERNEL);
	if (!temp_data) {
		ts_err("Alloc rawdiff data mem fail.");
		goto free_data;
	}

	result_data = kzalloc(data_len, GFP_KERNEL);
	if (!result_data) {
		ts_err("Alloc result data mem fail.");
		ret = -EINVAL;
		goto free_data;
	}

	cmd.cmd = GOODIX_DATA_MODE_CMD;
	cmd.len = 4;
	ret = core_data->hw_ops->send_cmd(core_data, &cmd);
	if (ret) {
		ts_err("Send enter rawdiff mode fail.");
		ret = -EINVAL;
		goto free_data;
	}

	usleep_range(30000, 30100);

	ret = core_data->hw_ops->write(core_data, misc->touch_data_addr,
		&sync_clean, 1);
	if (ret) {
		ts_err("Set zero to interrupt status reg before read data fail.");
		ret = -EINVAL;
		goto be_normal;
	}

	for (wait_cnt = 0; wait_cnt < 100; wait_cnt++) {
		usleep_range(10000, 10100);
		ret = core_data->hw_ops->read(core_data, misc->touch_data_addr, buf, 2);
		if ((0 == ret) && (buf[0] & 0x80))
			break;
	}


	if (wait_cnt < 100) {
		switch (type) {
		case VTS_FRAME_MUTUAL_RAW:
			ret = core_data->hw_ops->read(core_data, misc->mutual_rawdata_addr,
						temp_data, data_len);
			if (ret) {
				ts_err("Read rawdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		case VTS_FRAME_MUTUAL_DELTA:
			ret = core_data->hw_ops->read(core_data, misc->mutual_diffdata_addr,
						temp_data, data_len);
			if (ret) {
				ts_err("Read diffdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		case VTS_FRAME_SELF_RAW:
			ret = core_data->hw_ops->read(core_data, misc->mutual_refdata_addr,
						temp_data, data_len);
			if (ret) {
				ts_err("Read ref data fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
			
		default:
			VTI("%s: Invalid data type", __func__);
			break;
			
		}

		for (i = 0; i < data_size; i++) {
			result_data[i] = (temp_data[2 * i +1] << 8) | temp_data[2 * i];
			data[i] = result_data[i];
		}

		ret = core_data->hw_ops->write(core_data, misc->touch_data_addr,
			&sync_clean, 1);
		if (ret) {
			ts_err("Set zero to interrupt status reg after read data fail.");
			ret = -EINVAL;
			goto be_normal;
		}
	} else {
		ts_err("Wait for data ready timeout.");
		ret = -EINVAL;
		goto be_normal;
	}

be_normal:

	bbk_gdix_brl_be_normal();
	msleep(30);

free_data:

	kfree(temp_data);
	temp_data = NULL;
	kfree(result_data);
	result_data = NULL;

	return ret;
}



/*Jarvis 2020-7-9 End*/























