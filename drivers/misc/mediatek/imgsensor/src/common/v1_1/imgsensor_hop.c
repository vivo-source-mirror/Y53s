#define ERROR_NONE 0

/*************************************************************************
 * GLOBALS
 *	available_clk[4]
 *
 * DESCRIPTION
 *	This global stores current available clk of each sensor (as bits).
 *        available_clk[0]: sub_sim
 *        available_clk[1]: data_sim
 *        available_clk[2]: wifi
 *        available_clk[3]: 5G-NSA&SA-----
 *
 *************************************************************************/
static kal_uint8 available_clk[4] = {((1 << ARRAY_SIZE(sensor_freq_list[0])) - 1)};

/*************************************************************************
 * FUNCTION
 *	hop
 *
 * DESCRIPTION
 *	This function compares current rat and arfcn to the target
 *        freq_setting.
 *
 * PARAMETERS
 *      index: current simid
 *      rat:   current modem operation mode
 *      arfcn: current modem operation band/channel
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *      available_clk
 *
 *************************************************************************/
static kal_uint32 hop(int simid, kal_uint8 rat, kal_uint32 arfcn)	/*card,rat,frequnce*/
{
	int i = 0;
	int j = 0;
	int p = rat - 1;
	struct SENSOR_FREQ_LIST *cur;
	struct FREQ *freq;

/*ARRAY_SIZE(sensor_freq_list[0]) = 4, ARRAY_SIZE(sensor_freq_list) = 5*/
	pr_debug("changefreq: [%s(%d)] arfcn[%d]=%d\n", __func__, __LINE__, simid, arfcn);
	available_clk[simid] = (1 << ARRAY_SIZE(sensor_freq_list[0])) - 1;
	pr_debug("changefreq: [%s(%d)] sensor_mode=%d\n", __func__, __LINE__, imgsensor.sensor_mode);
	if (rat == MODEM_OPERATION_RAT_NULL)
		return ERROR_NONE;
	if (imgsensor.sensor_mode > IMGSENSOR_MODE_INIT &&
	    imgsensor.sensor_mode < (ARRAY_SIZE(sensor_freq_list) + 1)) {

		for (j = 0; j < ARRAY_SIZE(sensor_freq_list[0]); j++) {
			cur =&sensor_freq_list[imgsensor.sensor_mode - 1][j];
			freq = cur->freq_list[p];
			i = 0;
			while (freq[i].max != 0) {
				if ((arfcn >= freq[i].min) && (arfcn < freq[i].max)) {
					available_clk[simid] &= ~(1 << j); /* 1 1 1 1 & 0 0 0 0 default = 0xF,if not support is 0*/
					/*available_clk[simid] representative .setting = preview_setting_0*/
					break;
				}
				i++;
			}
		}

		pr_debug("changefreq: [%s(%d)] available_clk[%d]=%x\n", __func__, __LINE__, simid, available_clk[simid]);
	}

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	do_hop
 *
 * DESCRIPTION
 *	This function sets the present_freq_setting as freq_setting and
 *        decides whether to do a camera hop.
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *      imgsensor.present_freq_setting
 *
 *************************************************************************/
static kal_uint32 do_hop(int flag, void *psensor)
{
	struct SENSOR_FREQ_LIST *cur;
	kal_uint16 *setting;
	kal_uint32 setting_size;
	int i;
	kal_uint8 tmp_clk;
	struct IMGSENSOR_SENSOR *ppsensor; 


	if (flag & (1 << 1)) {
		imgsensor.present_freq_setting = 0;
	}

	pr_debug("changefreq: [%s(%d)] imgsensor.present_freq_setting=%d,\n",
	         __func__, __LINE__, imgsensor.present_freq_setting);

	pr_debug("changefreq: [%s(%d)] sensor_mode=%d\n", __func__, __LINE__, imgsensor.sensor_mode);
	if (imgsensor.sensor_mode > IMGSENSOR_MODE_INIT &&
	    imgsensor.sensor_mode < (ARRAY_SIZE(sensor_freq_list) + 1)) {

		if (flag & 1) {
			tmp_clk = available_clk[0];
		} else {
			tmp_clk = available_clk[1];
		}

		if (!tmp_clk) {
			pr_err("changefreq: [%s(%d)] no available clk\n", __func__, __LINE__);
			return ERROR_NONE;
		}

		for (i = 0; (i < 4) && (tmp_clk & available_clk[i]); i++) {
			tmp_clk &= available_clk[i];
		}

		pr_debug("changefreq: [%s(%d)] tmp_clk=%x,\n", __func__, __LINE__, tmp_clk);

		if (tmp_clk & (1 << imgsensor.present_freq_setting))
			return ERROR_NONE;

		i = 0;
		while (!(tmp_clk & (1 << i))) {
			i++;
		}

		imgsensor.freq_setting = i;
		
		pr_debug("changefreq: [%s(%d)] sensor_freq_list[%d][%d]\n", __func__, __LINE__, (imgsensor.sensor_mode-1),(imgsensor.freq_setting));
		
		cur = &sensor_freq_list[imgsensor.sensor_mode-1][imgsensor.freq_setting];
		setting = cur->setting;
		setting_size = cur->setting_size;
		
		for (i = 0; i < setting_size; i++) {
			pr_debug("changefreq: [%s] %p, writing setting, 0x%x\n", __func__, setting, setting[i]);
			}

		ppsensor = (struct IMGSENSOR_SENSOR *)psensor;

#if defined(CONFIG_MTK_CAM_PD2048)
		imgsensor_i2c_set_device(&(ppsensor->inst.i2c_cfg));
		table_write_cmos_sensor(setting, setting_size);
		imgsensor_i2c_set_device(NULL);
#endif	


#if 0
		for (i = 0; i < setting_size; i++) {
			pr_debug("changefreq: [%s] writing setting, 0x%04x, 0x%02x\n", __func__, setting[i * 2], setting[i * 2 + 1]);
			write_cmos_sensor_16_16(setting[i * 2], setting[i * 2 + 1]);
		}
#endif
		
		imgsensor.present_freq_setting = imgsensor.freq_setting;

	}
	pr_debug("changefreq: [%s(%d)] imgsensor.present_freq_setting=%d\n", __func__, __LINE__, imgsensor.present_freq_setting);
	return ERROR_NONE;
}
