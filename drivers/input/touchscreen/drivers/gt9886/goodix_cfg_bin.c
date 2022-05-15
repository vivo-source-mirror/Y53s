#include "goodix_cfg_bin.h"

#define GOODIX_DEFAULT_CFG
#define GOODIX_DEFAULT_FW
extern int goodix_create_fw_sys_V2(struct goodix_ts_core *core_data, struct goodix_ext_module *module);
extern struct goodix_module goodix_modules_V2;
extern struct goodix_ext_module goodix_fwu_module_V2;

/*
int goodix_start_cfg_bin(struct goodix_ts_core *ts_core)
{
	struct task_struct *cfg_bin_thrd;
	// create and run update thread 
	cfg_bin_thrd = kthread_run(goodix_cfg_bin_proc_V2,
							ts_core, "goodix-parse_cfg_bin");
	if (IS_ERR_OR_NULL(cfg_bin_thrd)) {
		VTE("Failed to create update thread:%ld",
				PTR_ERR(cfg_bin_thrd));
		return -EFAULT;
	}
	VTI("%s", __func__);
	return 0;
}
*/

int goodix_parse_cfg_bin_V2(struct goodix_cfg_bin *cfg_bin)
{
	u8 checksum;
	int i, r;
	u16 offset1, offset2;
	if (!cfg_bin->bin_data || cfg_bin->bin_data_len == 0) {
		VTE("NO cfg_bin data, cfg_bin data length:%d", cfg_bin->bin_data_len);
		r = -EINVAL;
		goto exit;
	}

	/* copy cfg_bin head info */
	VTI("cfg_bin->bin_data_len is %d, sizeof(struct goodix_cfg_bin_head) is %ld", cfg_bin->bin_data_len, sizeof(struct goodix_cfg_bin_head));
	if (cfg_bin->bin_data_len < sizeof(struct goodix_cfg_bin_head)) {
		VTE("Invalid cfg_bin size:%d", cfg_bin->bin_data_len);
		r = -EINVAL;
		goto exit;
	}
	memcpy(&cfg_bin->head, cfg_bin->bin_data, sizeof(struct goodix_cfg_bin_head));
	cfg_bin->head.bin_len = le32_to_cpu(cfg_bin->head.bin_len);


	/*check length*/
	if (cfg_bin->bin_data_len != cfg_bin->head.bin_len) {
		VTE("cfg_bin length error,len in cfg_bin:%d, len of firmware:%d",
				cfg_bin->head.bin_len, cfg_bin->bin_data_len);
		r = -EINVAL;
		goto exit;
	}

	/*check cfg_bin valid*/
	checksum = 0;
	for (i = 5; i < cfg_bin->bin_data_len; i++) {
		checksum += cfg_bin->bin_data[i];
	}
	if (checksum != cfg_bin->head.checksum) {
		VTE("cfg_bin checksum ERROR, checksum in cfg_bin:0x%02x, checksum caculate:0x%02x",
				cfg_bin->head.checksum, checksum);
		r = -EINVAL;
		goto exit;
	}

	/*allocate memory for cfg packages*/
	cfg_bin->cfg_pkgs = kzalloc(sizeof(struct goodix_cfg_package) * cfg_bin->head.pkg_num, GFP_KERNEL);
	if (!cfg_bin->cfg_pkgs) {
		VTE("cfg_pkgs, allocate memory ERROR");
		r = -ENOMEM;
		goto exit;
	}
	/*get cfg_pkg's info*/
	for (i = 0; i < cfg_bin->head.pkg_num; i++) {
		/*get cfg pkg length*/
		if (i == cfg_bin->head.pkg_num - 1) {
			offset1 = cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2] +
				(cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2 + 1] << 8);

			cfg_bin->cfg_pkgs[i].pkg_len = cfg_bin->bin_data_len - offset1;
		} else {
			offset1 = cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2] +
				(cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2 + 1] << 8);

			offset2 = cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2 + 2] +
				(cfg_bin->bin_data[TS_CFG_BIN_HEAD_LEN + i * 2 + 3] << 8);

			if (offset2 <= offset1) {
				VTE("offset error,pkg:%d, offset1:%d, offset2:%d", i, offset1, offset2);
				r = -EINVAL;
				goto exit;
			}

			cfg_bin->cfg_pkgs[i].pkg_len = offset2 - offset1;
		}
		/*get cfg pkg head*/
		memcpy(&cfg_bin->cfg_pkgs[i].cnst_info,
				&cfg_bin->bin_data[offset1],
				TS_PKG_CONST_INFO_LEN);
		memcpy(&cfg_bin->cfg_pkgs[i].reg_info,
				&cfg_bin->bin_data[offset1 + TS_PKG_CONST_INFO_LEN],
				TS_PKG_REG_INFO_LEN);
		/*compatible little edition and big edition*/
		goodix_cfg_pkg_leToCpu_V2(&cfg_bin->cfg_pkgs[i]);

		/*get configuration data*/
		cfg_bin->cfg_pkgs[i].cfg = &cfg_bin->bin_data[offset1 + TS_PKG_HEAD_LEN];
	}

	/*debug, print pkg information*/
	for (i = 0; i < cfg_bin->head.pkg_num; i++) {
		VTI("---------------------------------------------");
		VTI("------package:%d------", i + 1);
		VTI("package len:%04x", cfg_bin->cfg_pkgs[i].cnst_info.pkg_len);
		VTI("package ic_type:%s", cfg_bin->cfg_pkgs[i].cnst_info.ic_type);
		VTI("package cfg_type:%01x", cfg_bin->cfg_pkgs[i].cnst_info.cfg_type);
		VTI("package sensor_id:%01x", cfg_bin->cfg_pkgs[i].cnst_info.sensor_id);
		VTI("package hw_pid:%s", cfg_bin->cfg_pkgs[i].cnst_info.hw_pid);
		VTI("package hw_vid:%s", cfg_bin->cfg_pkgs[i].cnst_info.hw_vid);
		VTI("package fw_mask_version:%s", cfg_bin->cfg_pkgs[i].cnst_info.fw_mask);
		VTI("package fw_patch_version:%s", cfg_bin->cfg_pkgs[i].cnst_info.fw_patch);
		VTI("package x_res_offset:%02x", cfg_bin->cfg_pkgs[i].cnst_info.x_res_offset);
		VTI("package y_res_offset:%02x", cfg_bin->cfg_pkgs[i].cnst_info.y_res_offset);
		VTI("package trigger_offset:%02x", cfg_bin->cfg_pkgs[i].cnst_info.trigger_offset);

		VTI("");
		VTI("send_cfg_flag reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.cfg_send_flag.addr);
		VTI("version base reg:%02x, len:%d",
				cfg_bin->cfg_pkgs[i].reg_info.version_base.addr,
				cfg_bin->cfg_pkgs[i].reg_info.version_base.reserved1);
		VTI("pid reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.pid.addr);
		VTI("vid reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.vid.addr);
		VTI("sensor_id reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.sensor_id.addr);
		VTI("fw_status reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.fw_status.addr);
		VTI("cfg_addr reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.cfg_addr.addr);
		VTI("esd reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.esd.addr);
		VTI("command reg:%02x", cfg_bin->cfg_pkgs[i].reg_info.command.addr);
		VTI("coor:%02x", cfg_bin->cfg_pkgs[i].reg_info.coor.addr);
		VTI("gesture:%02x", cfg_bin->cfg_pkgs[i].reg_info.gesture.addr);
		VTI("fw_request:%02x", cfg_bin->cfg_pkgs[i].reg_info.fw_request.addr);
		VTI("proximity:%02x", cfg_bin->cfg_pkgs[i].reg_info.proximity.addr);

		VTI("--------------------------------------------");
	}
	r = 0;
exit:
	return r;
}

//int goodix_cfg_bin_proc_V2(void *data)
int goodix_cfg_bin_proc_V2(struct vts_device *vtsdev, const struct firmware *firmware)
{
	struct goodix_ts_core *core_data = vts_get_drvdata(vtsdev);
	//struct goodix_ts_core *core_data = data;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	struct device *dev = ts_dev->dev;
	int r = 0;
	struct goodix_cfg_bin *cfg_bin = NULL;
	//struct firmware *temp_firmware = NULL;


	VTI("enter in goodix_cfg_bin_proc_V2");

	/*update fw*/
	VTI("Firmware header update start.");

    goodix_modules_V2.core_data = core_data;
	r =  bbk_goodix_fw_update_V2(NULL, firmware);
	if (r < 0) {
		VTE("fw update FAILED");
		goto exit;
	}
	goodix_create_fw_sys_V2(core_data, &goodix_fwu_module_V2);

	
	cfg_bin = kzalloc(sizeof(struct goodix_cfg_bin), GFP_KERNEL);
	if (!cfg_bin) {
		VTE("Failed to alloc memory for cfg_bin");
		r = -ENOMEM;
		goto exit;
	}
	/*get cfg_bin from file system*/
	r = goodix_read_cfg_bin_V2(core_data, dev, cfg_bin);
	if (r < 0) {
		VTE("read cfg_bin from /etc/firmware FAILED");
		goto exit;
	}
	VTI("%s", __func__);
	/*parse cfg bin*/
	r = goodix_parse_cfg_bin_V2(cfg_bin);
	if (!r) {
		VTI("parse cfg bin SUCCESS");
	} else {
		VTE("parse cfg bin FAILED");
		goto exit;
	}



	/*get register address and configuration from cfg bin*/
	r = goodix_get_reg_and_cfg_V2(ts_dev, cfg_bin);
	if (!r) {
		VTI("get reg and cfg from cfg_bin SUCCESS");
	} else {
		if (r != -EBUS) {
			VTE("get reg and cfg from cfg_bin FAILED");
			goto exit;
		} else {
			VTE("get reg and cfg from cfg_bin FAILED, I2C com ERROR");
			goto exit;
		}
	}

	/*debug*/
	VTI("@@@@@@@@@");
	VTI("cfg_send_flag:0x%04x", ts_dev->reg.cfg_send_flag);
	VTI("pid:0x%04x", ts_dev->reg.pid);
	VTI("vid:0x%04x", ts_dev->reg.vid);
	VTI("sensor_id:0x%04x", ts_dev->reg.sensor_id);
	VTI("fw_mask:0x%04x", ts_dev->reg.fw_mask);
	VTI("fw_status:0x%04x", ts_dev->reg.fw_status);
	VTI("cfg_addr:0x%04x", ts_dev->reg.cfg_addr);
	VTI("esd:0x%04x", ts_dev->reg.esd);
	VTI("command:0x%04x", ts_dev->reg.command);
	VTI("coor:0x%04x", ts_dev->reg.coor);
	VTI("gesture:0x%04x", ts_dev->reg.gesture);
	VTI("fw_request:0x%04x", ts_dev->reg.fw_request);
	VTI("proximity:0x%04x", ts_dev->reg.proximity);
	VTI("@@@@@@@@@");

	
	/* initialize firmware */
	r =	goodix_ts_hw_init_V2(core_data);
	if (r < 0)
		goto exit;
	if (0) {
		/* alloc/config/register input device */
		r = goodix_ts_input_dev_config_V2(core_data);
		if (r < 0)
			goto exit;
	}
	/* request irq line */
	r = goodix_ts_irq_setup_V2(core_data);
	if (r < 0)
		goto exit;

	/*set flag, prevent fwupdate module parse cfg_group again*/
	core_data->cfg_group_parsed = true;

	/* inform the external module manager that
	 * touch core layer is ready now */

/*#ifdef CONFIG_FB
	core_data->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback_V2;
	if (fb_register_client(&core_data->fb_notifier))
		VTE("Failed to register fb notifier client:%d", r);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	core_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	core_data->early_suspend.resume = goodix_ts_lateresume;
	core_data->early_suspend.suspend = goodix_ts_earlysuspend;
	register_early_suspend(&core_data->early_suspend);
#endif*/

	/* esd protector */
	goodix_ts_esd_init_V2(core_data);

	/* generic notifier callback */
	core_data->ts_notifier.notifier_call = goodix_generic_noti_callback_V2;
	goodix_ts_register_notifier_V2(&core_data->ts_notifier);

exit:

	if (cfg_bin && cfg_bin->cfg_pkgs) {
		kfree(cfg_bin->cfg_pkgs);
		cfg_bin->cfg_pkgs = NULL;
	}

	if (cfg_bin && cfg_bin->bin_data) {
		kfree(cfg_bin->bin_data);
		cfg_bin->bin_data = NULL;
	}
	if (cfg_bin) {
		kfree(cfg_bin);
		cfg_bin = NULL;
	}
/*	
	if (temp_firmware) {
		kfree(temp_firmware);
		temp_firmware = NULL;
	}	
*/
	return r;
}


int goodix_get_reg_and_cfg_V2(struct goodix_ts_device *ts_dev, struct goodix_cfg_bin *cfg_bin)
{
	int i;
	u16 addr;
	u8 read_len;
	struct goodix_cfg_package *normal_pkg, *high_sense_pkg;
	char temp_sensor_id = -1;
	u8 temp_fw_mask[9] = {0x00};
	u8 temp_pid[8] = {0x00};
	int r = -EINVAL;
	normal_pkg = NULL;
	high_sense_pkg = NULL;

	if (!cfg_bin->head.pkg_num || !cfg_bin->cfg_pkgs) {
		VTE("there is none cfg package, pkg_num:%d", cfg_bin->head.pkg_num);
		r = -EINVAL;
		goto exit;
	}

	/*select suitable cfg packages*/
	for (i = 0; i < cfg_bin->head.pkg_num; i++) {
		/*get ic type*/
		if (!strncmp(cfg_bin->cfg_pkgs[i].cnst_info.ic_type, "nanjing",
					sizeof(cfg_bin->cfg_pkgs[i].cnst_info.ic_type)))
			ts_dev->ic_type = IC_TYPE_NANJING;
		else if (!strncmp(cfg_bin->cfg_pkgs[i].cnst_info.ic_type, "normandy",
					sizeof(cfg_bin->cfg_pkgs[i].cnst_info.ic_type)))
			ts_dev->ic_type = IC_TYPE_NORMANDY;
		else if (!strncmp(cfg_bin->cfg_pkgs[i].cnst_info.ic_type,"normandy_s", 
					sizeof(cfg_bin->cfg_pkgs[i].cnst_info.ic_type))) {
			ts_dev->ic_type = IC_TYPE_NORMANDY_SPI;
		} 
		else
			VTE("get ic type FAILED, unknow ic type from cfg_bin:%s",
					cfg_bin->cfg_pkgs[i].cnst_info.ic_type);

		VTI("ic_type:%d", ts_dev->ic_type);
		/*reset IC*/
		if(ts_dev->hw_ops->reset)
			r = ts_dev->hw_ops->reset(ts_dev);
        else
			VTE("rest fail");
		
		VTI("%s", __func__);
		/*contrast sensor id*/
		addr = cfg_bin->cfg_pkgs[i].reg_info.sensor_id.addr;
		if (!addr) {
			VTI("pkg:%d, sensor_id reg is NULL", i);
			continue;
		}
		VTI("%s", __func__);
		if(ts_dev->hw_ops->read)
		r = ts_dev->hw_ops->read(ts_dev, addr,
				&temp_sensor_id, 1);
		else
			r = -1;
		if (r < 0) {
			VTE("read sensor id FAILED,I2C ERROR, pkg:%d, sensor_id reg:0x%02x", i, addr);
			goto exit;
		} else {
			VTI("%s", __func__);
			/*sensor.reserved1 is a mask, if it's not ZERO, use it*/
			if (cfg_bin->cfg_pkgs[i].reg_info.sensor_id.reserved1 != 0)
				temp_sensor_id &= cfg_bin->cfg_pkgs[i].reg_info.sensor_id.reserved1;

			if (temp_sensor_id != cfg_bin->cfg_pkgs[i].cnst_info.sensor_id) {
				VTE("pkg:%d, sensor id contrast FAILED, reg:0x%02x", i, addr);
				VTE("sensor_id from i2c:%d, sensor_id of cfg bin:%d",
						temp_sensor_id,
						cfg_bin->cfg_pkgs[i].cnst_info.sensor_id);
				//continue;
			}
		}

		VTI("%s", __func__);
		/*contrast fw_mask, if this reg is null, skip this step*/
		addr = cfg_bin->cfg_pkgs[i].reg_info.fw_mask.addr;
		if (!addr || !cfg_bin->cfg_pkgs[i].cnst_info.fw_mask[0]) {
			VTE("pkg:%d, fw_mask of cfg bin is NULL, Skip!!", i);
		} else {
			if(ts_dev->hw_ops->read)
				r = ts_dev->hw_ops->read(ts_dev, addr, temp_fw_mask, sizeof(temp_fw_mask));
			else
				r = -1;
			if (r < 0) {
				VTE("read fw_mask FAILED, I2C ERROR, pkg: %d, fw_mask reg:0x%02x", i, addr);
				goto exit;
			} else if (strncmp(temp_fw_mask, cfg_bin->cfg_pkgs[i].cnst_info.fw_mask,
						sizeof(temp_fw_mask))) {
				VTE("pkg:%d, fw_mask contrast FAILED, reg:0x%02x,", i, addr);
				VTE("mask from i2c:%s, mask of cfg bin:%s",
						temp_fw_mask,
						cfg_bin->cfg_pkgs[i].cnst_info.fw_mask);
				continue;
			}
		}

		/*contrast pid*/
		addr = cfg_bin->cfg_pkgs[i].reg_info.pid.addr;
		read_len = cfg_bin->cfg_pkgs[i].reg_info.pid.reserved1;
		if (!addr) {
			VTE("pkg:%d, pid reg is NULL", i);
			continue;
		}
		if (read_len <= 0 || read_len > 8) {
			VTE("pkg:%d, hw_pid length ERROR, len:%d", i, read_len);
			continue;
		}
		if(ts_dev->hw_ops->read)
			r = ts_dev->hw_ops->read(ts_dev, addr, temp_pid, read_len);
		else
			r = -1;
		if (r < 0) {
			VTE("read pid FAILED, I2C ERROR, pkg: %d, pid reg:0x%02x", i, addr);
			goto exit;
		} else if (strncmp(temp_pid, cfg_bin->cfg_pkgs[i].cnst_info.hw_pid, read_len)) {
				VTE("pkg:%d, pid contrast FAILED, reg:0x%02x", i, addr);
				VTE("pid from i2c:%s, pid of cfg bin:%s",
						temp_pid,
						cfg_bin->cfg_pkgs[i].cnst_info.hw_pid);
				continue;
		}

		/*contrast success, cfg_type*/
		if (cfg_bin->cfg_pkgs[i].cnst_info.cfg_type == 0x01) {
			VTI("find normal cfg_pkg SUCCESS");
			r = 0;
			normal_pkg = &cfg_bin->cfg_pkgs[i];
		}
		if (cfg_bin->cfg_pkgs[i].cnst_info.cfg_type == 0x03) {
			VTI("find high sense cfg_pkg SUCCESS");
			high_sense_pkg = &cfg_bin->cfg_pkgs[i];
		}
	}

	/*get register address from normal_pkg*/
	if (!normal_pkg) {
		VTE("ERROR, none suitable normal_pkg exist in cfg_bin");
		/*ts_dev->ic_type = IC_TYPE_NONE;*/
		r = -EINVAL;
		goto exit;
	} else {
		/*get ic type*/
		if (!strncmp(normal_pkg->cnst_info.ic_type, "nanjing",
					sizeof(normal_pkg->cnst_info.ic_type)))
			ts_dev->ic_type = IC_TYPE_NANJING;
		else if (!strncmp(normal_pkg->cnst_info.ic_type, "normandy",
					sizeof(normal_pkg->cnst_info.ic_type)))
			ts_dev->ic_type = IC_TYPE_NORMANDY;
		else if (!strncmp(normal_pkg->cnst_info.ic_type, "normandy_s",
					sizeof(normal_pkg->cnst_info.ic_type)))
			ts_dev->ic_type = IC_TYPE_NORMANDY_SPI;
		else
			VTE("get ic type FAILED, unknow ic type from cfg_bin:%s",
					normal_pkg->cnst_info.ic_type);

		/*get register info*/
		ts_dev->reg.cfg_send_flag = normal_pkg->reg_info.cfg_send_flag.addr;

		ts_dev->reg.version_base = normal_pkg->reg_info.version_base.addr;
		ts_dev->reg.version_len = normal_pkg->reg_info.version_base.reserved1;

		ts_dev->reg.pid = normal_pkg->reg_info.pid.addr;
		ts_dev->reg.pid_len = normal_pkg->reg_info.pid.reserved1;

		ts_dev->reg.vid = normal_pkg->reg_info.vid.addr;
		ts_dev->reg.vid_len = normal_pkg->reg_info.vid.reserved1;

		ts_dev->reg.sensor_id = normal_pkg->reg_info.sensor_id.addr;
		ts_dev->reg.sensor_id_mask = normal_pkg->reg_info.sensor_id.reserved1;

		ts_dev->reg.fw_mask = normal_pkg->reg_info.fw_mask.addr;
		ts_dev->reg.fw_status = normal_pkg->reg_info.fw_status.addr;
		ts_dev->reg.cfg_addr = normal_pkg->reg_info.cfg_addr.addr;
		ts_dev->reg.esd = normal_pkg->reg_info.esd.addr;
		ts_dev->reg.command = normal_pkg->reg_info.command.addr;
		ts_dev->reg.coor = normal_pkg->reg_info.coor.addr;
		ts_dev->reg.gesture = normal_pkg->reg_info.gesture.addr;
		ts_dev->reg.fw_request = normal_pkg->reg_info.fw_request.addr;
		ts_dev->reg.proximity = normal_pkg->reg_info.proximity.addr;
	}

	/*get configuration from pkgs*/
	if (normal_pkg) {
		VTI("normal cfg is found!");
		if (!ts_dev->normal_cfg) {
			ts_dev->normal_cfg = devm_kzalloc(ts_dev->dev,
					sizeof(*ts_dev->normal_cfg), GFP_KERNEL);
			if (!ts_dev->normal_cfg) {
				VTE("Failed to alloc memory for normal cfg");
				return -ENOMEM;
			}
			mutex_init(&ts_dev->normal_cfg->lock);
		}

		ts_dev->normal_cfg->length = normal_pkg->pkg_len -
			TS_PKG_CONST_INFO_LEN - TS_PKG_REG_INFO_LEN;
		memcpy(ts_dev->normal_cfg->data,
				normal_pkg->cfg,
				ts_dev->normal_cfg->length);
	}

	if (high_sense_pkg) {
		VTI("high sense cfg is found!");
		if (!ts_dev->highsense_cfg) {
			ts_dev->highsense_cfg = devm_kzalloc(ts_dev->dev,
					sizeof(*ts_dev->highsense_cfg), GFP_KERNEL);
			if (!ts_dev->highsense_cfg) {
				VTE("Failed to alloc memory for high sense cfg");
				return -ENOMEM;
			}
			mutex_init(&ts_dev->highsense_cfg->lock);
		}

		ts_dev->highsense_cfg->length = high_sense_pkg->pkg_len -
			TS_PKG_CONST_INFO_LEN - TS_PKG_REG_INFO_LEN;
		memcpy(ts_dev->highsense_cfg->data,
				high_sense_pkg->cfg,
				ts_dev->highsense_cfg->length);
	}
	VTI("get cfg success");
	

exit:
	return r;
}

int goodix_read_cfg_bin_V2(struct goodix_ts_core *coredata, struct device *dev, struct goodix_cfg_bin *cfg_bin)
{
	int r;
	struct firmware *firmware = NULL;
	char *cfg_data;
	int size = 0;

	firmware = kzalloc(sizeof(struct firmware), GFP_KERNEL);
	if (!firmware) {
		VTE("Failed to alloc memory for firmware");
		r = -ENOMEM;
		goto exit;
	}

	cfg_data= vts_fw_data_get(coredata->vtsdev, VTS_FW_TYPE_CONFIG, &size);
	if (cfg_data == NULL || size == 0) {
		VTE("vivoTsGet config data fail");
		r = -1;
		goto exit;
	}

	firmware->size = size;
	firmware->data = cfg_data;
	VTI("firmware->size is %zu", firmware->size);

	//firmware->size = sizeof(goodix_default_CFG);
	//firmware->data = goodix_default_CFG;
	
	VTI("11 enter in goodix_read_cfg_bin_V2!!!!!!!");
	
	if (firmware->size <= 0) {
		VTE("request_firmware, cfg_bin length ERROR,len:%zu", firmware->size);
		r = -EINVAL;
		goto exit;
	}

	cfg_bin->bin_data_len = firmware->size;
	/*allocate memory for cfg_bin->bin_data*/
	cfg_bin->bin_data = kzalloc(cfg_bin->bin_data_len, GFP_KERNEL);
	if (!cfg_bin->bin_data) {
		VTE("Allocate memory for cfg_bin->bin_data FAILED");
		r = -ENOMEM;
		goto exit;
	}
	memcpy(cfg_bin->bin_data, firmware->data, cfg_bin->bin_data_len);

	r = 0;
exit:
	if (firmware) {
		kfree(firmware);
		firmware = NULL;
	}
	return r;
}

int goodix_read_cfg_bin_V2_from_dts_V2(struct device_node *node, struct goodix_cfg_bin *cfg_bin)
{
	unsigned int len = 0;
	struct property *prop = NULL;

	prop = of_find_property(node, "goodix_cfg_bin", &len);
	if (!prop || !prop->value || len == 0) {
		VTE("Invalid cfg type, size:%u",  len);
		return -EINVAL;
	}

	cfg_bin->bin_data_len = len;
	/*allocate memory for cfg_bin->bin_data*/
	cfg_bin->bin_data = kzalloc(cfg_bin->bin_data_len, GFP_KERNEL);
	if (!cfg_bin->bin_data) {
		VTE("Allocate memory for cfg_bin->bin_data FAILED");
		return -ENOMEM;
	}
	memcpy(cfg_bin->bin_data, prop->value, cfg_bin->bin_data_len);

	return 0;
}

void goodix_cfg_pkg_leToCpu_V2(struct goodix_cfg_package *pkg)
{
	if (!pkg) {
		VTE("cfg package is NULL");
		return;
	}
	/*package const_info*/
	pkg->cnst_info.pkg_len = le32_to_cpu(pkg->cnst_info.pkg_len);
	pkg->cnst_info.x_res_offset = le16_to_cpu(pkg->cnst_info.x_res_offset);
	pkg->cnst_info.y_res_offset = le16_to_cpu(pkg->cnst_info.y_res_offset);
	pkg->cnst_info.trigger_offset = le16_to_cpu(pkg->cnst_info.trigger_offset);

	/*package reg_info*/
	pkg->reg_info.cfg_send_flag.addr = le16_to_cpu(pkg->reg_info.cfg_send_flag.addr);
	pkg->reg_info.pid.addr = le16_to_cpu(pkg->reg_info.pid.addr);
	pkg->reg_info.vid.addr = le16_to_cpu(pkg->reg_info.vid.addr);
	pkg->reg_info.sensor_id.addr = le16_to_cpu(pkg->reg_info.sensor_id.addr);
	pkg->reg_info.fw_status.addr = le16_to_cpu(pkg->reg_info.fw_status.addr);
	pkg->reg_info.cfg_addr.addr = le16_to_cpu(pkg->reg_info.cfg_addr.addr);
	pkg->reg_info.esd.addr = le16_to_cpu(pkg->reg_info.esd.addr);
	pkg->reg_info.command.addr = le16_to_cpu(pkg->reg_info.command.addr);
	pkg->reg_info.coor.addr = le16_to_cpu(pkg->reg_info.coor.addr);
	pkg->reg_info.gesture.addr = le16_to_cpu(pkg->reg_info.gesture.addr);
	pkg->reg_info.fw_request.addr = le16_to_cpu(pkg->reg_info.fw_request.addr);
	pkg->reg_info.proximity.addr = le16_to_cpu(pkg->reg_info.proximity.addr);
}
