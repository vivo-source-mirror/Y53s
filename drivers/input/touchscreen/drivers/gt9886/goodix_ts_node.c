#include "goodix_ts_core.h"
#include "../vts_core.h"

#define FW_SUBSYS_MAX_NUM			28
#define BBK_USER_DATA_MAX_LEN			30
#define BBK_USER_DATA_ADDR			0xBDB4
#define BBK_SENSOR_NUM_ADDR			0x5473
#define BBK_DRIVER_GROUP_A_NUM_ADDR		0x5477
#define BBK_DRIVER_GROUP_B_NUM_ADDR		0x5478
#define BBK_READ_COOR_ADDR			0x4100
#define BBK_RAW_DATA_BASE_ADDR			0x8FA0
#define BBK_DIFF_DATA_BASE_ADDR			0x9D20
#define BBK_REF_DATA_BASE_ADDR			0xA980
#define BBK_CMD_ADDR				0x3101
#define BBK_CMD_ADDR_SPI    		0x30F1 

#define GSX_KEY_DATA_LEN			37
#define GSX_BUFFER_DATA_LEN			513

struct fw_subsys_info {
	u8 type;
	u32 size;
	u16 flash_addr;
	const u8 *data;
};

struct firmware_info {
	u32 size;
	u16 checksum;
	u8 hw_pid[6];
	u8 hw_vid[3];
	u8 fw_pid[8];
	u8 fw_vid[4];
	u8 subsys_num;
	u8 chip_type;
	u8 protocol_ver;
	u8 reserved[2];
	struct fw_subsys_info subsys[FW_SUBSYS_MAX_NUM];
};


struct firmware_data {
	struct firmware_info fw_info;
	const struct firmware *firmware;
};




enum update_status {
	UPSTA_NOTWORK = 0,
	UPSTA_PREPARING,
	UPSTA_UPDATING,
	UPSTA_ABORT,
	UPSTA_SUCCESS,
	UPSTA_FAILED
};

struct fw_update_ctrl {
	enum update_status  status;
	unsigned int progress;
	bool force_update;

	bool allow_reset;
	bool allow_irq;
	bool allow_suspend;
	bool allow_resume;

	struct firmware_data fw_data;
	struct goodix_ts_device *ts_dev;
	struct goodix_ts_core *core_data;

	char fw_name[32];
	struct bin_attribute attr_fwimage;
	bool fw_from_sysfs;
	int mode;
};

struct gesture_module {
	atomic_t registered;
	unsigned int kobj_initialized;
	rwlock_t rwlock;
	unsigned char gesture_type[32];
	unsigned char gesture_data[GSX_KEY_DATA_LEN];
	unsigned char gesture_buffer_data[GSX_BUFFER_DATA_LEN];
	struct goodix_ext_module module;
	struct goodix_ts_cmd cmd;
};
extern struct goodix_module goodix_modules_V2;
extern struct gesture_module *gsx_gesture_V2;
extern int goodix_fw_update_proc_V2(struct fw_update_ctrl *fwu_ctrl);

extern int goodix_ts_irq_enable_V2(struct goodix_ts_core *core_data,
			bool enable);


void bbk_goodix_cmds_init_V2(struct goodix_ts_cmd *ts_cmd,
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

int bbk_goodix_get_channel_num_V2(u32 *sen_num, u32 *drv_num)
{
	int ret = -1;
	u8 buf[2] = {0};
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;
	
	VTI("===enter===");

	ret = ts_dev->hw_ops->read(ts_dev, BBK_SENSOR_NUM_ADDR, buf, 1);
	if (ret) {
		VTE("Read sen_num fail.");
		return ret;
	}

	*sen_num = buf[0];

	ret = ts_dev->hw_ops->read(ts_dev, BBK_DRIVER_GROUP_A_NUM_ADDR, buf, 2);
	if (ret) {
		VTE("Read drv_num fail.");
		return ret;
	}

	*drv_num = buf[0] + buf[1];
	VTI("sen_num : %d, drv_num : %d", *sen_num, *drv_num);

	return 0;
}

int bbk_goodix_be_normal_V2(void)
{
	int ret = -1;
	struct goodix_ts_cmd normal_cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;
	unsigned char buf[2] = {0x00, 0x00};

	VTI("===enter===");

	bbk_goodix_cmds_init_V2(&normal_cmd, 0x00, 0, ts_dev->reg.command);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &normal_cmd);
	if (ret)
		VTE("Send enter normal mode command fail.");
	
	buf[0] = buf[1] = 0x00;
	ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
	if (ret) {
		VTE("After be_normal, Set zero to interrupt status reg fail.");
		ret = -EINVAL;
	}
	msleep(30);

	return ret;
}

int bbk_goodix_get_rawordiff_data_V2(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	int i, j, wait_cnt, ret = 0;
	u32 sen_num = 0, drv_num = 0;
	u8 *temp_data = NULL;
	u16 *result_data = NULL;
	int data_size = 0, data_len = 0;
	u8 buf[2] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;	
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	VTI("get data %d(0:rawdata 1:diffdata)", type);

	if ((type != VTS_FRAME_MUTUAL_RAW) && (type != VTS_FRAME_MUTUAL_DELTA)) {
		VTE("Invalid get rawdiff data params.");
		return -EINVAL;
	}

	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, &sen_num);
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, &drv_num);
	VTI("sen_num is %d, drv_num is %d", sen_num, drv_num);

if (0){
	ret = bbk_goodix_get_channel_num_V2(&sen_num, &drv_num);
	if (ret) {
		VTE("Get channel num fail.");
		return -EINVAL;
	}
}
	data_size = sen_num * drv_num;
	data_len = data_size * sizeof(u16);
	if (data == NULL) {
		VTE("User data format error.");
		return -EINVAL;
	}

	temp_data = kzalloc(data_len, GFP_KERNEL);
	if (!temp_data) {
		VTE("Alloc rawdiff data mem fail.");
		return -EINVAL;
	}

	result_data = kzalloc(data_len, GFP_KERNEL);
	if (!result_data) {
		VTE("Alloc result data mem fail.");
		ret = -EINVAL;
		goto free_data;
	}

	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.	*/
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0) {  
		VTE("disable doze mode FAILE");		
		ret = -EINVAL;
		goto enable_doze;
	}

	if (bbk_goodix_set_auto_idle_V2(vtsdev, VTS_FRAME_MUTUAL_RAW)) {
		VTE("disable idle mode fail.");
		ret = -EINVAL;
		goto enable_doze;
	}  
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/


	bbk_goodix_cmds_init_V2(&cmd, 0x01, 0, ts_dev->reg.command);

	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		VTE("Send enter rawdiff mode fail.");
		ret = -EINVAL;
		goto idle_enable;
	}
	
	usleep_range(18000, 18100);

	goodix_ts_irq_enable_V2(core_data, false);

	ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
	if (ret) {
		VTE("Set zero to interrupt status reg fail.");
		ret = -EINVAL;
		goto be_normal;
	}

	msleep(30);
	for (wait_cnt = 0; wait_cnt < 3; wait_cnt++) {
		ret = ts_dev->hw_ops->read(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if ((0 == ret) && (buf[0] & 0x80))
			break;
		msleep(150);
	}

	if (wait_cnt < 3) {
		switch (type) {
		case VTS_FRAME_MUTUAL_RAW:/*rawdata*/
			ret = ts_dev->hw_ops->read(ts_dev, BBK_RAW_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read rawdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		case VTS_FRAME_MUTUAL_DELTA:/*diffdata	*/
			ret = ts_dev->hw_ops->read(ts_dev, BBK_DIFF_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read diffdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		/*
		case 2:
			ret = goodix_i2c_read_V2(ts_dev, BBK_REF_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read ref data fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		*/
		default:
			VTI("%s: Invalid data type", __func__);
		}

		/*Jarvis 2018060401 start,fix the data array .*/
		for (i = 0; i < data_size; i++) {  
			result_data[i] = (temp_data[2 * i] << 8) | temp_data[2 * i + 1];
		}

		for (i = 0; i < drv_num ; i++) {
			for (j = 0; j < sen_num; j++) {
				data[j * drv_num + i] = result_data[j + i * sen_num];
			}
		} 
		/*Jarvis 2018060401 end,fix the data array.*/

		buf[0] = buf[1] = 0x00;
		ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if (ret) {
			VTE("Set zero to interrupt status reg fail.");
			ret = -EINVAL;
			goto be_normal;
		}
	} else {
		VTE("Wait for data ready timeout.");
		ret = -EINVAL;
		goto be_normal;
	}

be_normal:
	bbk_goodix_be_normal_V2();	
	goodix_ts_irq_enable_V2(core_data, true);

idle_enable:
	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.*/
	if (bbk_goodix_set_auto_idle_V2(vtsdev, VTS_FRAME_MUTUAL_DELTA)) {
		VTE("ensable idle mode fail.");
		ret = -EINVAL;
	}

enable_doze:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0) {
		VTE("enable doze mode FAILE");
		ret = -EINVAL;
	}
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/
free_data:
	kfree(result_data);
	result_data = NULL;
	kfree(temp_data);
	temp_data = NULL;

	return ret;
}

int bbk_goodix_get_rawordiff_data_V2_v1(int which, int *data)
{
	int i, j, wait_cnt, ret = 0;
	u32 sen_num = 0, drv_num = 0;
	u8 *temp_data = NULL;
	u16 *result_data = NULL;
	int data_size = 0, data_len = 0;
	u8 buf[2] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;	
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	VTI("get data %d(0:rawdata 1:diffdata)", which);

	if ((which != 0) && (which != 1) && (which != 2)) {
		VTE("Invalid get rawdiff data params.");
		return -EINVAL;
	}

	ret = bbk_goodix_get_channel_num_V2(&sen_num, &drv_num);
	if (ret) {
		VTE("Get channel num fail.");
		return -EINVAL;
	}

	data_size = sen_num * drv_num;
	data_len = data_size * sizeof(u16);
	if (data == NULL) {
		VTE("User data format error.");
		return -EINVAL;
	}

	temp_data = kzalloc(data_len, GFP_KERNEL);
	if (!temp_data) {
		VTE("Alloc rawdiff data mem fail.");
		return -EINVAL;
	}

	result_data = kzalloc(data_len, GFP_KERNEL);
	if (!result_data) {
		VTE("Alloc result data mem fail.");
		ret = -EINVAL;
		goto free_data;
	}

	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.	*/
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0) {  
		VTE("disable doze mode FAILE");		
		ret = -EINVAL;
		goto free_data;
	}

	if (bbk_goodix_set_auto_idle_V2(NULL, 0)) {
		VTE("disable idle mode fail.");
		ret = -EINVAL;
		goto enable_doze;
	}  
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/


	bbk_goodix_cmds_init_V2(&cmd, 0x01, 0, ts_dev->reg.command);

	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		VTE("Send enter rawdiff mode fail.");
		ret = -EINVAL;
		goto idle_enable;
	}
	
	usleep_range(18000, 18100);

	goodix_ts_irq_enable_V2(core_data, false);

	ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
	if (ret) {
		VTE("Set zero to interrupt status reg fail.");
		ret = -EINVAL;
		goto be_normal;
	}

	msleep(30);
	for (wait_cnt = 0; wait_cnt < 3; wait_cnt++) {
		ret = ts_dev->hw_ops->read(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if ((0 == ret) && (buf[0] & 0x80))
			break;
		msleep(150);
	}

	if (wait_cnt < 3) {
		switch (which) {
		case 0:/*rawdata*/
			ret = ts_dev->hw_ops->read(ts_dev, BBK_RAW_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read rawdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		case 1:/*diffdata	*/
			ret = ts_dev->hw_ops->read(ts_dev, BBK_DIFF_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read diffdata fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		case 2:
			ret = ts_dev->hw_ops->read(ts_dev, BBK_REF_DATA_BASE_ADDR,
						temp_data, data_len);
			if (ret) {
				VTE("Read ref data fail.");
				ret = -EINVAL;
				goto be_normal;
			}
			break;
		}

		/*Jarvis 2018060401 start,fix the data array .*/
		for (i = 0; i < data_size; i++) {  
			result_data[i] = (temp_data[2 * i] << 8) | temp_data[2 * i + 1];
		}

		for (i = 0; i < drv_num ; i++) {
			for (j = 0; j < sen_num; j++) {
				data[j * drv_num + i] = result_data[j + i * sen_num];
			}
		} 
		/*Jarvis 2018060401 end,fix the data array.*/

		buf[0] = buf[1] = 0x00;
		ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if (ret) {
			VTE("Set zero to interrupt status reg fail.");
			ret = -EINVAL;
			goto be_normal;
		}
	} else {
		VTE("Wait for data ready timeout.");
		ret = -EINVAL;
		goto be_normal;
	}

be_normal:
	bbk_goodix_be_normal_V2();	
	goodix_ts_irq_enable_V2(core_data, true);

idle_enable:
	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.*/
	if (bbk_goodix_set_auto_idle_V2(NULL, 1)) {
		VTE("ensable idle mode fail.");
		ret = -EINVAL;
	}

enable_doze:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0) {
		VTE("enable doze mode FAILE");
		ret = -EINVAL;
	}
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/
free_data:
	kfree(result_data);
	result_data = NULL;
	kfree(temp_data);
	temp_data = NULL;

	return ret;
}

int bbk_goodix_dump_fw_data(struct vts_device *vtsdev, int state)
{
	int i, j, wait_cnt, ret = 0;
	u32 sen_num = 0, drv_num = 0;
	u8 *temp_data = NULL;
	u16 *result_data = NULL;
	int data_size = 0, data_len = 0;
	u8 buf[2] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u8 *fw_buf;
	u32 fw_buf_len;
	short *fw_data;

	if (!vts_is_debug())
		return 0;

	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_TX_NUM, &sen_num);
	vts_property_get(vtsdev, VTS_PROPERTY_SENSOR_RX_NUM, &drv_num);
	VTI("sen_num is %d, drv_num is %d", sen_num, drv_num);

	fw_buf_len = drv_num * 16;
	fw_buf = kzalloc(fw_buf_len, GFP_KERNEL);
	if (!fw_buf) {
		vts_dev_err(vtsdev, "alloc fw_buf for data failed\n");
		return -EINVAL;
	}

	fw_data = kzalloc(sen_num * drv_num * sizeof(short), GFP_KERNEL);
	if (!fw_data) {
		vts_dev_err(vtsdev, "alloc fw_data for data failed\n");
		kfree(fw_buf);
		return -EINVAL;
	}

	data_size = sen_num * drv_num;
	data_len = data_size * sizeof(u16);

	temp_data = kzalloc(data_len, GFP_KERNEL);
	if (!temp_data) {
		VTE("Alloc rawdiff data mem fail.");
		kfree(fw_buf);
		kfree(fw_data);
		return -EINVAL;
	}

	result_data = kzalloc(data_len, GFP_KERNEL);
	if (!result_data) {
		VTE("Alloc result data mem fail.");
		ret = -EINVAL;
		goto free_data;
	}

	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.	*/
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, false) != 0) {
		VTE("disable doze mode FAILE");
		ret = -EINVAL;
		goto enable_doze;
	}

	if (bbk_goodix_set_auto_idle_V2(vtsdev, VTS_FRAME_MUTUAL_RAW)) {
		VTE("disable idle mode fail.");
		ret = -EINVAL;
		goto enable_doze;
	}
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/


	bbk_goodix_cmds_init_V2(&cmd, 0x01, 0, ts_dev->reg.command);

	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		VTE("Send enter rawdiff mode fail.");
		ret = -EINVAL;
		goto idle_enable;
	}

	usleep_range(18000, 18100);

	goodix_ts_irq_enable_V2(core_data, false);

	ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
	if (ret) {
		VTE("Set zero to interrupt status reg fail.");
		ret = -EINVAL;
		goto be_normal;
	}

	msleep(30);
	for (wait_cnt = 0; wait_cnt < 3; wait_cnt++) {
		ret = ts_dev->hw_ops->read(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if ((0 == ret) && (buf[0] & 0x80))
			break;
		msleep(150);
	}

	if (wait_cnt < 3) {
		ret = ts_dev->hw_ops->read(ts_dev, BBK_DIFF_DATA_BASE_ADDR,
					temp_data, data_len);
		if (ret) {
			VTE("Read diffdata fail.");
			ret = -EINVAL;
			goto be_normal;
		}

		/*Jarvis 2018060401 start,fix the data array .*/
		for (i = 0; i < data_size; i++) {
			result_data[i] = (temp_data[2 * i] << 8) | temp_data[2 * i + 1];
		}

		for (i = 0; i < drv_num ; i++) {
			for (j = 0; j < sen_num; j++) {
				fw_data[j * drv_num + i] = result_data[j + i * sen_num];
			}
		}

		for (i = 0; i < sen_num; i++) {
			memset(fw_buf, 0, fw_buf_len);
			for (j = 0; j < drv_num ; j++)
				snprintf(fw_buf + strlen(fw_buf), fw_buf_len - strlen(fw_buf), "%4d ", fw_data[i * drv_num  + j]);
			vts_dev_info(vtsdev, "%s\n", fw_buf);
		}

		/*Jarvis 2018060401 end,fix the data array.*/

		buf[0] = buf[1] = 0x00;
		ret = ts_dev->hw_ops->write(ts_dev, BBK_READ_COOR_ADDR, buf, 2);
		if (ret) {
			VTE("Set zero to interrupt status reg fail.");
			ret = -EINVAL;
			goto be_normal;
		}
	} else {
		VTE("Wait for data ready timeout.");
		ret = -EINVAL;
		goto be_normal;
	}

be_normal:
	bbk_goodix_be_normal_V2();
	goodix_ts_irq_enable_V2(core_data, true);

idle_enable:
	/*Jarvis 2018060402 start, disable doze and get in active mode before get data.*/
	if (bbk_goodix_set_auto_idle_V2(vtsdev, VTS_FRAME_MUTUAL_DELTA)) {
		VTE("ensable idle mode fail.");
		ret = -EINVAL;
	}

enable_doze:
	if (ts_dev->hw_ops->set_doze_mode(ts_dev, true) != 0) {
		VTE("enable doze mode FAILE");
		ret = -EINVAL;
	}
	/*Jarvis 2018060402 end, disable doze and get in active mode before get data.*/
	kfree(result_data);
	result_data = NULL;
free_data:
	kfree(temp_data);
	temp_data = NULL;
	kfree(fw_data);
	fw_data = NULL;
	kfree(fw_buf);
	fw_buf = NULL;

	return ret;
}

int bbk_goodix_fw_update_V2(struct vts_device *vtsdev, const struct firmware *firmware)
{
	struct fw_update_ctrl *fwu_ctrl;
	static DEFINE_MUTEX(bbk_fwu_lock);
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	VTI("===enter===");

	fwu_ctrl = kzalloc(sizeof(struct fw_update_ctrl), GFP_KERNEL);
	if (!fwu_ctrl) {
		VTE("Failed to alloc memory for fwu_ctrl");
		return -EPERM;
	}

	fwu_ctrl->ts_dev = ts_dev;
	fwu_ctrl->allow_reset = true;
	fwu_ctrl->allow_irq = true;
	fwu_ctrl->allow_suspend = true;
	fwu_ctrl->allow_resume = true;
	fwu_ctrl->core_data = core_data;

	mutex_lock(&bbk_fwu_lock);
	fwu_ctrl->fw_data.firmware = firmware;
	
	/* DONT allow reset/irq/suspend/resume during update */
	fwu_ctrl->allow_irq = false;
	fwu_ctrl->allow_suspend = false;
	fwu_ctrl->allow_resume = false;
	fwu_ctrl->allow_reset = false;
	goodix_ts_blocking_notify_V2(NOTIFY_FWUPDATE_START, NULL);
	
	/* ready to update */
	goodix_fw_update_proc_V2(fwu_ctrl);

	goodix_ts_blocking_notify_V2(NOTIFY_FWUPDATE_END, NULL);
	fwu_ctrl->allow_reset = true;
	fwu_ctrl->allow_irq = true;
	fwu_ctrl->allow_suspend = true;
	fwu_ctrl->allow_resume = true;
	
	mutex_unlock(&bbk_fwu_lock);

	kfree(fwu_ctrl);

	return 0;
}

int bbk_goodix_set_auto_idle_V2(struct vts_device *vtsdev, int state)
{
	int i = 0;
	int ret = 0;
	u8 cmd_data = 0;
	u8 read_val = 0;
	struct goodix_ts_cmd idle_cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;
	u32 high_rate = 0;
	u32 idle_time = 0;
	u16 feed_reg;
	VTI("===enter===,state:%d (1:idle, 0:active)", state); 

	if (1 == state) {
		bbk_goodix_cmds_init_V2(&idle_cmd, 0x11, 0, ts_dev->reg.command);
	} else if (0 == state) {
	    vts_property_get(vtsdev, VTS_PROPERTY_GAME_HIGH_RATE, &high_rate);
	    vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);
		
		if(0 == idle_time){//dtsi not set  ,keep active 
		   cmd_data |= 0x7F;
		}
		else if(0xFF == idle_time){//dtsi set 0xff ->  not change FW state 
		   cmd_data = 0x00;
		}
		else{// dtsi set avaliable value ,set idle enter time
			cmd_data = idle_time;
		}
		
		if(1 == high_rate){//hight rate set 
			cmd_data |= 0x80;
		}
		else if(0 == high_rate){// dtsi not set high rate or set 0,send 10 ff ,keep the same as prev 
			cmd_data = 0xFF;
		}
		else if(2 == high_rate){// dtsi not set high rate 2, send normal report rate cmd
			cmd_data |= 0x00;
		}

		bbk_goodix_cmds_init_V2(&idle_cmd, 0x10, cmd_data, ts_dev->reg.command);
		VTI("set idle cmd data:0x10--%x",cmd_data);
		
	} else {
		VTE("Invalid set idle params");
		return -EPERM;
	}

	if(ts_dev->ic_type == IC_TYPE_NORMANDY_SPI)
		feed_reg = BBK_CMD_ADDR_SPI;
	else
		feed_reg = BBK_CMD_ADDR;

	for (i = 0; i < 5; i++) {
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &idle_cmd);
		if (ret) {
			VTE("Send idle command error");
			ret = -1;
		}
		mdelay(10);
		if (ts_dev->hw_ops->read(ts_dev, feed_reg, &read_val, 1))
			VTE("Read cmd reg fail.");
		else
			VTI("Read cmd reg value:%02x.", read_val);

		VTI("read_val:%d", read_val);
		if (state) {
			if (read_val & 0x04) {
				VTI("success in idle mode");
				ret = 0;
				break;
			}
		} else {
			if (!(read_val & 0x04)) {
				VTI("success out idle mode");
				ret = 0;
				break;
			}
		}
	}

	return ret;
}

int bbk_goodix_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 30;
	return 0;
}


ssize_t bbk_goodix_readUdd_V2(struct vts_device *vtsdev, u8 *udd, size_t nbytes)
{
	int ret = 0;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	VTI("===enter===");

	ret = ts_dev->hw_ops->read(ts_dev, BBK_USER_DATA_ADDR, udd, 15);
	if (ret) {
		VTE("User data i2c read failed");
		ret = -1;
	}

	return nbytes;
}

int bbk_goodix_readUdd_V2_v1(unsigned char *udd)
{
	int ret = 0;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	VTI("===enter===");

	ret = ts_dev->hw_ops->read(ts_dev, BBK_USER_DATA_ADDR, udd, 15);
	if (ret) {
		VTE("User data i2c read failed");
		ret = -1;
	}

	return ret;
}

ssize_t bbk_goodix_writeUdd_V2(struct vts_device *vtsdev, u8 *udd, size_t nbytes)
{
	int ret = 0;
	unsigned char buf[BBK_USER_DATA_MAX_LEN] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	VTI("===enter===");

	memcpy(buf, udd, 15);
	buf[BBK_USER_DATA_MAX_LEN - 2] = 0xAA;
	buf[BBK_USER_DATA_MAX_LEN - 1] = 0x100 - checksum_u8(buf, BBK_USER_DATA_MAX_LEN - 1);

	ret = ts_dev->hw_ops->write(ts_dev, BBK_USER_DATA_ADDR,
					buf, BBK_USER_DATA_MAX_LEN);
	if (ret) {
		VTE("User data i2c write failed");
		return -EPERM;
	}

	bbk_goodix_cmds_init_V2(&cmd, 0x0A, 0, ts_dev->reg.command);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		VTE("Send write udd command error");
		return -EPERM;
	}

	return nbytes;
}

int bbk_goodix_writeUdd_V2_v1(unsigned char *udd)
{
	int ret = 0;
	unsigned char buf[BBK_USER_DATA_MAX_LEN] = {0};
	struct goodix_ts_cmd cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	VTI("===enter===");

	memcpy(buf, udd, 15);
	buf[BBK_USER_DATA_MAX_LEN - 2] = 0xAA;
	buf[BBK_USER_DATA_MAX_LEN - 1] = 0x100 - checksum_u8(buf, BBK_USER_DATA_MAX_LEN - 1);

	ret = ts_dev->hw_ops->write(ts_dev, BBK_USER_DATA_ADDR,
					buf, BBK_USER_DATA_MAX_LEN);
	if (ret) {
		VTE("User data i2c write failed");
		return -EPERM;
	}

	bbk_goodix_cmds_init_V2(&cmd, 0x0A, 0, ts_dev->reg.command);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &cmd);
	if (ret) {
		VTE("Send write udd command error");
		return -EPERM;
	}

	return ret;
}


void bbk_goodix_enter_gesture_V2(void)
{
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int r = 0;
	//struct goodix_ts_cmd finger_dclick_cmd;
	//struct goodix_ts_cmd finger_icon_cmd;
   // int ret = 0;
	VTI("===enter===core_data->suspended:%d", atomic_read(&core_data->suspended));
	
#ifdef CONFIG_PINCTRL
		if (!IS_ERR_OR_NULL(core_data->pinctrl) && !IS_ERR_OR_NULL(core_data->pin_sta_active)) {
			VTI("select active pinstate");
			r = pinctrl_select_state(core_data->pinctrl,
						core_data->pin_sta_active);
			if (r < 0)
				VTE("Failed to select active pinstate, r:%d", r);
		}
#endif

	if (atomic_read(&core_data->suspended) == 1) {
		if (ts_dev && ts_dev->hw_ops->resume)
			ts_dev->hw_ops->resume(ts_dev);

		goodix_ts_irq_enable_V2(core_data, true);
	}
	enable_irq_wake(core_data->irq);
/*
	if (0 == (vivoTsGetVtsData()->exportSwitch & 0x01)) {
		bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x01, ts_dev->reg.command);
	} else {
		bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x00, ts_dev->reg.command);
	}
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
	if (ret) {
		VTE("Send finger dclick command error");
		ret = -1;
	}
	if (1 == (vivoTsGetVtsData()->exportSwitch & 0x01)) {
		bbk_goodix_cmds_init_V2(&finger_icon_cmd, 0x1a, 0x01, ts_dev->reg.command);
		VTD("%02X, %02X, %02X", finger_icon_cmd.cmds[0], finger_icon_cmd.cmds[1], finger_icon_cmd.cmds[2]);
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_icon_cmd);
		if (ret) {
			VTE("Send finger dclick command error");
			ret = -1;
		}	
	}

	bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x01, ts_dev->reg.command);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
	if (ret) {
		VTE("Send finger dclick command error");
		ret = -1;
	}
*/
	goodix_ts_suspend_V2(core_data, 0);
}


void bbk_goodix_set_ic_enter_gesture_V2(void)
{
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	//struct goodix_ts_device *ts_dev = core_data->ts_dev;
	//struct goodix_ts_cmd finger_dclick_cmd;
	//struct goodix_ts_cmd finger_icon_cmd;
	struct goodix_ext_module *ext_module;
	int ret = 0;

	VTI("===enter===core_data->suspended:%d", atomic_read(&core_data->suspended));
	
	enable_irq_wake(core_data->irq);
/*
	if (0 == (vivoTsGetVtsData()->exportSwitch & 0x01)) {
		bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x01, ts_dev->reg.command);
	} else {
		bbk_goodix_cmds_init_V2(&finger_dclick_cmd, 0x19, 0x00, ts_dev->reg.command);
	}
	

	VTD("%02X, %02X, %02X", finger_dclick_cmd.cmds[0], finger_dclick_cmd.cmds[1], finger_dclick_cmd.cmds[2]);
	ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_dclick_cmd);
	if (ret) {
		VTE("Send finger dclick command error");
	}
	mdelay(20);
	
	if (1 == (vivoTsGetVtsData()->exportSwitch & 0x01)) {
		bbk_goodix_cmds_init_V2(&finger_icon_cmd, 0x1a, 0x01, ts_dev->reg.command);
		VTD("%02X, %02X, %02X", finger_icon_cmd.cmds[0], finger_icon_cmd.cmds[1], finger_icon_cmd.cmds[2]);
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &finger_icon_cmd);
		if (ret) {
			VTE("Send finger dclick command error");
		}	
	}
	mdelay(20);
*/
	mutex_lock(&goodix_modules_V2.mutex);
	if (!list_empty(&goodix_modules_V2.head)) {
		list_for_each_entry(ext_module, &goodix_modules_V2.head, list) {
			if (!ext_module->funcs || !ext_module->funcs->before_suspend)
				continue;

			ret = ext_module->funcs->before_suspend(core_data, ext_module);
			if (ret == EVT_CANCEL_SUSPEND) {
				VTI("Canceled by module:%s", ext_module->name);
			}
		}
	}
	mutex_unlock(&goodix_modules_V2.mutex);
}

int touch_state = 0;

int bbk_goodix_mode_change_V2(struct vts_device *vtsdev, int which)
{
	int ret = 0;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;

	VTI("====start====, which:%d (0:Normal, 1:Sleep, 2:Gesture)", which);
	switch (which) {
	case VTS_ST_NORMAL:
		ret = goodix_ts_resume_V2(core_data);
		if (ret) {
			VTE("Change normal mode fail.");
			ret = -1;
		}
		touch_state = VTS_ST_NORMAL;
		break;
	case VTS_ST_SLEEP:
		ret = goodix_ts_suspend_V2(core_data, 1);
		if (ret) {
			VTE("Change sleep mode fail.");
			ret = -1;
		}
		touch_state = VTS_ST_SLEEP;
		break;
	case VTS_ST_GESTURE:
		touch_state = VTS_ST_GESTURE;
		bbk_goodix_enter_gesture_V2();
		break;
	default:
		VTE("Invalid mode change params");
		ret = -1;
		break;
	}

	return ret;
}

int bbk_goodix_get_fw_version_V2(struct vts_device *vtsdev, u64 *version)
{
	u8 ver_buf[4] = {0};
	u8 *cfg_buf;
	int ret = -1;
	int firmware_version = 0;
	int config_version = 0;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	
	/*read firmware verison*/
	ret = ts_dev->hw_ops->read(ts_dev, ts_dev->reg.vid,
				ver_buf, ts_dev->reg.vid_len);
	if (ret) {
		VTE("Read fw version fail.");
		return -EPERM;
	}
	VTI("VID:%02x %02x %02x %02x", ver_buf[0], ver_buf[1],
				ver_buf[2], ver_buf[3]);
	firmware_version = (ver_buf[0] << 24) | (ver_buf[1] << 16) | (ver_buf[2] << 8) | ver_buf[3];
	
	/*read config version*/
	cfg_buf = kzalloc(4096, GFP_KERNEL | GFP_DMA);
	if (!cfg_buf) {
		VTE("Alloc config buffer mem fail.");
		return -EPERM;
	}

	disable_irq(core_data->irq);
	if (ts_dev->hw_ops->read_config) {
		ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf, 0);
		if (ret > 0) {
			VTI("config version : %d", cfg_buf[0]);
			config_version = cfg_buf[0];
		} else {
			VTE("Read config version fail.");
			ret = -1;
		}
	} else {
		ret = -1;
	}
	enable_irq(core_data->irq);
	*version =(((u64)firmware_version) << 16) | ((u64)config_version);

	kfree(cfg_buf);
	
/*
	ret = goodix_i2c_read_V2(ts_dev, ts_dev->reg.vid,
					ver_buf, ts_dev->reg.vid_len);
	if (ret) {
		VTE("Read fw version fail.");
		return -EPERM;
	}
	VTI("VID:%02x %02x %02x %02x", ver_buf[0], ver_buf[1],
				ver_buf[2], ver_buf[3]);
	*version = (ver_buf[0] << 24) | (ver_buf[1] << 16) | (ver_buf[2] << 8) | ver_buf[3];
*/
	return 0;
}

int bbk_goodix_get_fw_version_V2_v1(int which)
{
	u8 ver_buf[4] = {0};
	u8 *cfg_buf;
	int ret = -1;
	struct goodix_ts_core *core_data = goodix_modules_V2.core_data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	switch (which) {
	case 0:
		ret = ts_dev->hw_ops->read(ts_dev, ts_dev->reg.vid,
					ver_buf, ts_dev->reg.vid_len);
		if (ret) {
			VTE("Read fw version fail.");
			return -EPERM;
		}
		VTI("VID:%02x %02x %02x %02x", ver_buf[0], ver_buf[1],
					ver_buf[2], ver_buf[3]);
		ret = (ver_buf[0] << 24) | (ver_buf[1] << 16) | (ver_buf[2] << 8) | ver_buf[3];
		break;
	case 1:
		cfg_buf = kzalloc(4096, GFP_KERNEL);
		if (!cfg_buf) {
			VTE("Alloc config buffer mem fail.");
			return -EPERM;
		}

		disable_irq(core_data->irq);
		if (ts_dev->hw_ops->read_config) {
			ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf, 0);
			if (ret > 0) {
				VTI("config version : %d", cfg_buf[0]);
				ret = cfg_buf[0];
			} else {
				VTE("Read config version fail.");
				ret = -1;
			}
		} else {
			ret = -1;
		}
		enable_irq(core_data->irq);

		kfree(cfg_buf);
		break;
	default:
		VTE("Invalid get fw version params");
		break;
	}

	return ret;
}

int bbk_goodix_gesture_point_get_V2_v1(u16 *data)
{
	int i;
	u8 gesture_type = gsx_gesture_V2->gesture_data[2];
	int ges_num = 0;

	switch (gesture_type) {
	case 0xBA://VTS_EVENT_GESTURE_PATTERN_UP
	case 0xAB://VTS_EVENT_GESTURE_PATTERN_DOWN
		ges_num = 2;
		data[0] = (gsx_gesture_V2->gesture_buffer_data[1] << 8) | gsx_gesture_V2->gesture_buffer_data[0];
		data[1] = (gsx_gesture_V2->gesture_buffer_data[3] << 8) | gsx_gesture_V2->gesture_buffer_data[2];
		data[2] = (gsx_gesture_V2->gesture_buffer_data[5] << 8) | gsx_gesture_V2->gesture_buffer_data[4];
		data[3] = (gsx_gesture_V2->gesture_buffer_data[7] << 8) | gsx_gesture_V2->gesture_buffer_data[6];
		break;
	case 0x65://VTS_EVENT_GESTURE_PATTERN_E
	case 0x40://VTS_EVENT_GESTURE_PATTERN_A
	case 0x66://VTS_EVENT_GESTURE_PATTERN_F
	case 0x6F://VTS_EVENT_GESTURE_PATTERN_O
		ges_num = 6;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];
		break;
	case 0x77://VTS_EVENT_GESTURE_PATTERN_W
		ges_num = 5;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];
		break;
	case 0x63://VTS_EVENT_GESTURE_PATTERN_C
		ges_num = 6;
		for (i = 0; i < 10; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];

		data[10] = data[8];
		data[11] = data[9];
		break;
	default:
		ges_num = 0;
		break;
	}

	for (i = 0; i < 12; i++)
		VTI("data[i]:%d", data[i]);

	return ges_num;
}



int bbk_goodix_gesture_point_get_V2(struct goodix_ts_core *core_data, u16 *data)
{
	int i;
	u8 gesture_type = gsx_gesture_V2->gesture_data[2];
	int ges_num = 0;
	u16 coordinate_x[9];
	u16 coordinate_y[9];

	switch (gesture_type) {
	case 0xBA://VTS_EVENT_GESTURE_PATTERN_UP
	case 0xAB://VTS_EVENT_GESTURE_PATTERN_DOWN
		ges_num = 2;
		data[0] = (gsx_gesture_V2->gesture_buffer_data[1] << 8) | gsx_gesture_V2->gesture_buffer_data[0];
		data[1] = (gsx_gesture_V2->gesture_buffer_data[3] << 8) | gsx_gesture_V2->gesture_buffer_data[2];
		data[2] = (gsx_gesture_V2->gesture_buffer_data[5] << 8) | gsx_gesture_V2->gesture_buffer_data[4];
		data[3] = (gsx_gesture_V2->gesture_buffer_data[7] << 8) | gsx_gesture_V2->gesture_buffer_data[6];
		break;
	case 0x65://VTS_EVENT_GESTURE_PATTERN_E
	case 0x40://VTS_EVENT_GESTURE_PATTERN_A
	case 0x66://VTS_EVENT_GESTURE_PATTERN_F
	case 0x6F://VTS_EVENT_GESTURE_PATTERN_O
		ges_num = 6;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];
		break;
	case 0x77://VTS_EVENT_GESTURE_PATTERN_W
		ges_num = 5;
		for (i = 0; i < 2 * ges_num; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];
		break;
	case 0x63://VTS_EVENT_GESTURE_PATTERN_C
		ges_num = 6;
		for (i = 0; i < 10; i++)
			data[i] = (gsx_gesture_V2->gesture_buffer_data[2 * i + 1] << 8) | gsx_gesture_V2->gesture_buffer_data[2 * i];

		data[10] = data[8];
		data[11] = data[9];
		break;
	default:
		ges_num = 0;
		break;
	}

	for (i = 0; i < ges_num; i++){
		coordinate_x[i] = data[2*i] ;
		coordinate_y[i] = data[2*i+1];
		VTI("x[i]:%d, y[i]:%d", coordinate_x[i], coordinate_y[i]);
	}
	//for(i = ges_num; i < 9; i++){
	//	coordinate_x[i] = 65535;
	//	coordinate_y[i] = 65535;
	//}
	vts_report_coordinates_set(core_data->vtsdev, coordinate_x, coordinate_y, ges_num);	

	return ges_num;
}

int bbk_goodix_set_charger_bit_V2(struct vts_device *vtsdev, int state)
{
	int ret = 0, i;
	u8 read_val = 0;
	u16 feed_reg;
	struct goodix_ts_cmd charger_cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	if(ts_dev == NULL){
		VTE("goodix_modules_V2.core_date is not init");
		return -1;
	}

    if(ts_dev->goodix_sensor_test){
       VTI("goodix sensor testing, can not send charge bit");
	   VTI("return !!!");
	   return 0;
	}
	VTI("===enter===,state:%d (0:no charger, 1:has charger)", state);

	if (1 == state) {
		bbk_goodix_cmds_init_V2(&charger_cmd, 0x06, 0, ts_dev->reg.command);
		goodix_modules_V2.core_data->charge_sta = true;
	} else if (0 == state) {
		bbk_goodix_cmds_init_V2(&charger_cmd, 0x07, 0, ts_dev->reg.command);
		goodix_modules_V2.core_data->charge_sta = false;
	} else {
		VTE("Invalid set charger bit params");
		return -1;
	}

	if(ts_dev->ic_type == IC_TYPE_NORMANDY_SPI)
		feed_reg = BBK_CMD_ADDR_SPI;
	else
		feed_reg = BBK_CMD_ADDR;

	for (i = 0; i < 5; i++) {
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &charger_cmd);
		if (ret) {
			VTE("Send charger command error");
			ret = -1;
		}
		mdelay(10);
		if (ts_dev->hw_ops->read(ts_dev, feed_reg, &read_val, 1))
			VTE("Read cmd reg fail.");
		else
			VTI("Read cmd reg value:%02x.", read_val);
		VTI("read_val:%d", read_val);

		if (state) {
			if (read_val & 0x02) {
				VTI("success in charger mode");
				ret = 0;
				break;
			}
		} else {
			if (!(read_val & 0x02)) {
				VTI("success out charger mode");
				ret = 0;
				break;
			}
		}
	}

	return ret;
}


int bbk_goodix_set_Edge_Switch_V2(struct vts_device *vtsdev, int on)
{
	int ret = 0;
	int i;
	u8 read_val = 0;
	u16 feed_reg;
	struct goodix_ts_cmd direction_cmd;
	struct goodix_ts_device *ts_dev = goodix_modules_V2.core_data->ts_dev;

	if (ts_dev == NULL) {
		VTE("goodix_modules_V2.core_date is not init");
		return -1;
	}

	VTI("===enter===,state:%d (1:normal,0:left,2:right)", on);

	if (1 == on) {
		bbk_goodix_cmds_init_V2(&direction_cmd, 0x0F, 0, ts_dev->reg.command);
	} else if (0 == on) {
		bbk_goodix_cmds_init_V2(&direction_cmd, 0x0E, 0, ts_dev->reg.command); /*pls combine the code in formally driver and del this massage. Jarvis in 2018-4-25*/
	} else if (2 == on) {
		bbk_goodix_cmds_init_V2(&direction_cmd, 0x0E, 1, ts_dev->reg.command);
	} else {
		VTE("Invalid setEdgeRestainSwitch params");
		return -EPERM;
	}

	if(ts_dev->ic_type == IC_TYPE_NORMANDY_SPI) {
		feed_reg = BBK_CMD_ADDR_SPI;
		for (i = 0; i < 5; i++) {
			ret = ts_dev->hw_ops->send_cmd(ts_dev, &direction_cmd);
			if (ret) {
				VTE("Send setEdgeRestainSwitch command error");
				ret = -1;
			}
			mdelay(10);
			if (ts_dev->hw_ops->read(ts_dev, feed_reg, &read_val, 1))
				VTE("Read cmd reg fail.");
			else
				VTI("Read cmd reg value:%02x.", read_val);

			VTI("read_val:%d", read_val);
			if (on) {
				if (!(read_val & 0x08)) {
					VTI("success out horizontal screen mode");
					ret = 0;
					break;
				}
			} else {
				if (read_val & 0x08) {
					VTI("success in horizontal screen mode");
					ret = 0;
					break;
				}
			}
		}
	} else {
		ret = ts_dev->hw_ops->send_cmd(ts_dev, &direction_cmd);
		if (ret) {
			VTE("Send setEdgeRestainSwitch command error");
			ret = -1;
		}
		mdelay(10);
	}

	return ret;
}
