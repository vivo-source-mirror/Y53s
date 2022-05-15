
#include "himax_inspection.h"


extern struct himax_core_fp g_core_fp;
extern struct himax_ts_data *private_ts;
extern struct himax_platform_data *hv_pdata;

extern struct himax_ic_data *ic_data;

void himax_inspection_init(void);

void (*fp_himax_self_test_init)(void) = himax_inspection_init;

#ifdef HX_ESD_RECOVERY
	extern u8 HX_ESD_RESET_ACTIVATE;
#endif

static uint8_t	NOISEMAX;
static uint8_t	g_recal_thx;

static int hx_test_data_get(int RAW[], char *start_log, char *result, int now_item)
{
	uint32_t i;

	ssize_t len = 0;
	char *testdata = NULL;
	uint32_t SZ_SIZE = 0x1000;
	int now_size = (int)strlen(g_rslt_data);
	int max_size = SZ_SIZE * HX_CRITERIA_ITEM;

	I("%s: Entering, Now type=%s!\n", __func__, g_himax_inspection_mode[now_item]);

	testdata = kzalloc(sizeof(char) * SZ_SIZE, GFP_KERNEL);
	if (!testdata) {
		E("%s: testdata alloc failed\n", __func__);
		return -ENOMEM;
	}

	len += snprintf((testdata + len), SZ_SIZE - len, "%s\n", start_log);
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++) {
		if (i > 1 && ((i + 1) % ic_data->HX_RX_NUM) == 0)
			len += snprintf((testdata + len), SZ_SIZE - len, "%5d,\n", RAW[i]);
		else
			len += snprintf((testdata + len), SZ_SIZE - len, "%5d,", RAW[i]);
	}
	len += snprintf((testdata + len), SZ_SIZE - len, "\n%s", result);

	//memcpy(&g_rslt_data[now_item * SZ_SIZE], testdata, SZ_SIZE);
	I("%s:=======Now buffer used length=%d, this item size=%d\n", __func__, now_size, (int)len);
	if(max_size > now_size + len)
		memcpy(&g_rslt_data[now_size], testdata, len);
	else
		E("%s: Over the buffer size!\n", __func__);


	kfree(testdata);
	I("%s: End!\n", __func__);
	return NO_ERR;

}

static int himax_switch_mode_inspection(int mode)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	I("%s: Entering\n", __func__);

	/*Stop Handshaking*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0x00;
	tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00; tmp_data[0] = 0x00;
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

	/*Swtich Mode*/
	switch (mode) {
	case HIMAX_INSPECTION_SORTING:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SORTING_START; tmp_data[0] = PWD_SORTING_START;
		break;
	case HIMAX_INSPECTION_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_OPEN_START; tmp_data[0] = PWD_OPEN_START;
		break;
	case HIMAX_INSPECTION_SHORT:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_SHORT_START; tmp_data[0] = PWD_SHORT_START;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:

	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_RAWDATA_START; tmp_data[0] = PWD_RAWDATA_START;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_NOISE_START; tmp_data[0] = PWD_NOISE_START;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_ACT_IDLE_START; tmp_data[0] = PWD_ACT_IDLE_START;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWUG_START; tmp_data[0] = PWD_LPWUG_START;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = PWD_LPWUG_IDLE_START; tmp_data[0] = PWD_LPWUG_IDLE_START;
		break;

	default:
		I("%s,Nothing to be done!\n", __func__);
		break;
	}

	if (g_core_fp.fp_assign_sorting_mode != NULL)
		g_core_fp.fp_assign_sorting_mode(tmp_data);
	I("%s: End of setting!\n", __func__);

	return 0;

}

int himax_get_rawdata(int RAW[], uint32_t datalen)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t *tmp_rawdata;
	uint8_t retry = 0;
	uint16_t checksum_cal;
	uint32_t i = 0;

	uint8_t max_xfer_size = 128;
	int address = 0;
	int total_read_times = 0;
	int total_size = datalen * 2 + 4;
	int total_size_temp;

	uint32_t j = 0;
	uint32_t index = 0;
	uint32_t Min_DATA = 99999;
	uint32_t Max_DATA = -99999;


	tmp_rawdata = kzalloc(sizeof(uint8_t)*(total_size+8), GFP_KERNEL);
	if (!tmp_rawdata) {
		E("%s: tmp_rawdata alloc failed\n", __func__);
		return -ENOMEM;
	}

	/*1 Set Data Ready PWD*/
	while (retry < 300) {
		tmp_addr[3] = 0x10;
		tmp_addr[2] = 0x00;
		tmp_addr[1] = 0x00;
		tmp_addr[0] = 0x00;
		tmp_data[3] = 0x00;
		tmp_data[2] = 0x00;
		tmp_data[1] = Data_PWD1;
		tmp_data[0] = Data_PWD0;
		g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		if ((tmp_data[0] == Data_PWD0 && tmp_data[1] == Data_PWD1) ||
			(tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0)) {
			break;
		}

		retry++;
		msleep(1);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	while (retry < 200) {
		if (tmp_data[0] == Data_PWD1 && tmp_data[1] == Data_PWD0)
			break;

		retry++;
		msleep(1);
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	}

	if (retry >= 200) {
		kfree(tmp_rawdata);
		return 1;
	} else {
		retry = 0;
	}

	/*2 Read Data from SRAM*/
	while (retry < 10) {
		checksum_cal = 0;
		total_size_temp = total_size;
		tmp_addr[3] = 0x10;	tmp_addr[2] = 0x00;	tmp_addr[1] = 0x00;	tmp_addr[0] = 0x00;

		if (total_size % max_xfer_size == 0)
			total_read_times = total_size / max_xfer_size;
		else
			total_read_times = total_size / max_xfer_size + 1;

		for (i = 0; i < (total_read_times); i++) {
			if (total_size_temp >= max_xfer_size) {
				g_core_fp.fp_register_read(tmp_addr, max_xfer_size, &tmp_rawdata[i*max_xfer_size], false);
				total_size_temp = total_size_temp - max_xfer_size;
			} else {
				/*I("last total_size_temp=%d\n", total_size_temp);*/
				g_core_fp.fp_register_read(tmp_addr, total_size_temp % max_xfer_size, &tmp_rawdata[i*max_xfer_size], false);
			}

			address = ((i+1)*max_xfer_size);
			tmp_addr[1] = (uint8_t)((address>>8)&0x00FF);
			tmp_addr[0] = (uint8_t)((address)&0x00FF);
		}

		/*3 Check Checksum*/
		for (i = 2; i < datalen * 2 + 4; i = i + 2)
			checksum_cal += tmp_rawdata[i + 1] * 256 + tmp_rawdata[i];

		if (checksum_cal == 0)
			break;

		retry++;
	}

	if (checksum_cal != 0) {
		E("%s: Get rawdata checksum fail!\n", __func__);
		kfree(tmp_rawdata);
		return HX_CHKSUM_FAIL;
	}

	/*4 Copy Data*/
	for (i = 0; i < ic_data->HX_TX_NUM*ic_data->HX_RX_NUM; i++)
		RAW[i] = tmp_rawdata[(i * 2) + 1 + 4] * 256 + tmp_rawdata[(i * 2) + 4];

	//for (j = 0; j < ic_data->HX_RX_NUM; j++) {
	//	if (j == 0)
	//		printk("      RX%2d", j + 1);
	//	else
	//		printk("  RX%2d", j + 1);
	//}
	//printk("\n");

	for (i = 0; i < ic_data->HX_TX_NUM; i++) {
		//printk("TX%2d", i + 1);
		for (j = 0; j < ic_data->HX_RX_NUM; j++) {
			//printk("%5d ", RAW[index]);
			if (RAW[index] > Max_DATA)
				Max_DATA = RAW[index];
			if (RAW[index] < Min_DATA)
				Min_DATA = RAW[index];
			index++;
		}
		printk("\n");
	}
	I("Max = %5d, Min = %5d \n", Max_DATA, Min_DATA);

	kfree(tmp_rawdata);
	return HX_INSPECT_OK;
}

static void himax_switch_data_type(uint8_t checktype)
{
	uint8_t datatype = 0x00;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		datatype = DATA_SORTING;
		break;
	case HIMAX_INSPECTION_OPEN:
		datatype = DATA_OPEN;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		datatype = DATA_MICRO_OPEN;
		break;
	case HIMAX_INSPECTION_SHORT:
		datatype = DATA_SHORT;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		datatype = DATA_RAWDATA;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		datatype = DATA_NOISE;
		break;
	case HIMAX_INSPECTION_BACK_NORMAL:
		datatype = DATA_BACK_NORMAL;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		datatype = DATA_RAWDATA;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		datatype = DATA_ACT_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		datatype = DATA_ACT_IDLE_NOISE;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		datatype = DATA_LPWUG_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		datatype = DATA_LPWUG_NOISE;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		datatype = DATA_LPWUG_IDLE_RAWDATA;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		datatype = DATA_LPWUG_IDLE_NOISE;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}
	g_core_fp.fp_diag_register_set(datatype, 0x00);
}

static void himax_bank_search_set(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	/*skip frame*/
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0xF4;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	switch (checktype) {

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		tmp_data[0] = BS_ACT_IDLE;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		tmp_data[0] = BS_LPWUG;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		tmp_data[0] = BS_LPWUG_dile;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		tmp_data[0] = BS_RAWDATA;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		tmp_data[0] = BS_NOISE;
		break;
	default:
		tmp_data[0] = BS_OPENSHORT;
		break;
	}
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
}

static void himax_neg_noise_sup(uint8_t *data)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	/*0x10007FD8 Check support negative value or not */
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F; tmp_addr[0] = 0xD8;
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	if ((tmp_data[3] & 0x04) == 0x04) {
		data[2] = 0x0C; data[3] = 0x7F;
	} else
		I("%s Not support negative noise\n", __func__);
}

static void himax_set_N_frame(uint16_t Nframe, uint8_t checktype)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	himax_bank_search_set(Nframe, checktype);

	/*IIR MAX*/
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72;
	tmp_addr[0] = 0x94;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = (uint8_t)((Nframe & 0xFF00) >> 8);
	tmp_data[0] = (uint8_t)(Nframe & 0x00FF);
	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);

	if (checktype == HIMAX_INSPECTION_WEIGHT_NOISE ||
		checktype == HIMAX_INSPECTION_ABS_NOISE)
		himax_neg_noise_sup(tmp_data);

	g_core_fp.fp_register_write(tmp_addr, 4, tmp_data, 0);
}

static void himax_get_noise_base(void)/*Normal Threshold*/
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x8C;/*0x1000708F*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	NOISEMAX = tmp_data[3];

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x90;/*0x10007092*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);

	g_recal_thx = tmp_data[2];
	I("%s: NOISEMAX=%d, g_recal_thx = %d\n", __func__,
		NOISEMAX, g_recal_thx);
}

static uint16_t himax_get_palm_num(void)/*Palm Number*/
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint16_t palm_num;

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0xA8;/*0x100070AB*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	palm_num = tmp_data[3];
	I("%s: palm_num = %d ", __func__, palm_num);

	return palm_num;
}

static int himax_get_noise_weight_test(void)
{
	uint8_t tmp_addr[4];
	uint8_t tmp_data[4];
	uint16_t weight = 0;
	uint16_t value = 0;

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x72; tmp_addr[0] = 0xC8;/*0x100072C8 weighting value*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	if (tmp_data[3] != 0x72 || tmp_data[2] != 0xC8)
		return FW_NOT_READY;

	value = (tmp_data[1] << 8) | tmp_data[0];
	I("%s: value = %d, %d, %d ", __func__, value, tmp_data[2], tmp_data[3]);

	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x9C;/*0x1000709C weighting threshold*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	weight = tmp_data[0];
	tmp_addr[3] = 0x10; tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x70; tmp_addr[0] = 0x94;/*0x10007095 weighting threshold*/
	g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
	weight = tmp_data[1] * weight;
	I("%s: weight = %d ", __func__, weight);

	if (value > weight)
		return ERR_TEST_FAIL;
	else
		return 0;
}

static uint32_t himax_check_mode(uint8_t checktype)
{
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	if (g_core_fp.fp_check_sorting_mode != NULL)
		g_core_fp.fp_check_sorting_mode(tmp_data);

	if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1])) {
		I("Change to mode=%s\n", g_himax_inspection_mode[checktype]);
		return 0;
	} else {
		return 1;
	}
}

static uint32_t himax_wait_sorting_mode(uint8_t checktype)
{
	uint8_t tmp_addr[4] = {0};
	uint8_t tmp_data[4] = {0};
	uint8_t wait_pwd[2] = {0};
	int count = 0;

	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		wait_pwd[0] = PWD_SORTING_END;
		wait_pwd[1] = PWD_SORTING_END;
		break;
	case HIMAX_INSPECTION_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_MICRO_OPEN:
		wait_pwd[0] = PWD_OPEN_END;
		wait_pwd[1] = PWD_OPEN_END;
		break;
	case HIMAX_INSPECTION_SHORT:
		wait_pwd[0] = PWD_SHORT_END;
		wait_pwd[1] = PWD_SHORT_END;
		break;
	case HIMAX_INSPECTION_RAWDATA:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;
	case HIMAX_INSPECTION_WEIGHT_NOISE:
	case HIMAX_INSPECTION_ABS_NOISE:
		wait_pwd[0] = PWD_NOISE_END;
		wait_pwd[1] = PWD_NOISE_END;
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
		wait_pwd[0] = PWD_RAWDATA_END;
		wait_pwd[1] = PWD_RAWDATA_END;
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		wait_pwd[0] = PWD_ACT_IDLE_END;
		wait_pwd[1] = PWD_ACT_IDLE_END;
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_NOISE:
		wait_pwd[0] = PWD_LPWUG_END;
		wait_pwd[1] = PWD_LPWUG_END;
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		wait_pwd[0] = PWD_LPWUG_IDLE_END;
		wait_pwd[1] = PWD_LPWUG_IDLE_END;
		break;

	default:
		I("No Change Mode and now type=%d\n", checktype);
		break;
	}

	do {
		if (g_core_fp.fp_check_sorting_mode != NULL)
			g_core_fp.fp_check_sorting_mode(tmp_data);
		if ((wait_pwd[0] == tmp_data[0]) && (wait_pwd[1] == tmp_data[1]))
			return 0;

		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xA8;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000A8, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xE4;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000E4, tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		tmp_addr[3] = 0x10; tmp_addr[2] = 0x00; tmp_addr[1] = 0x7F; tmp_addr[0] = 0x40;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x10007F40,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		tmp_addr[3] = 0x90; tmp_addr[2] = 0x00; tmp_addr[1] = 0x00; tmp_addr[0] = 0xE8;
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, false);
		I("%s: 0x900000E8,tmp_data[0]=%x,tmp_data[1]=%x,tmp_data[2]=%x,tmp_data[3]=%x \n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		I("Now retry %d times!\n", count++);
		msleep(50);
	} while (count < 50);

	return 1;
}


#if 0
static int himax_check_notch(int index)
{
	if (SKIP_NOTCH_START < 0 && SKIP_NOTCH_END < 0 && SKIP_DUMMY_START < 0 && SKIP_DUMMY_START < 0) {
		/* no support notch */
		return 0;
	}
	if ((index >= SKIP_NOTCH_START) && (index <= SKIP_NOTCH_END))
		return 1;
	else if ((index >= SKIP_DUMMY_START) && (index <= SKIP_DUMMY_END))
		return 1;
	else
		return 0;
}
#endif

#define FAIL_IN_INDEX "%s: %s FAIL in index %d\n"
static uint32_t mpTestFunc(uint8_t checktype, uint32_t datalen)
{
	uint32_t i/*, j*/, ret = 0;
	uint32_t *RAW = NULL;
	char *rslt_log = NULL;
	char *start_log = NULL;
	uint16_t palm_num = 0;
	uint16_t noise_count = 0;
	int ret_val;

	RAW = kzalloc(sizeof(uint32_t) * datalen, GFP_KERNEL);
	if (!RAW) {
		E("%s: RAW alloc failed\n", __func__);
		return -ENOMEM;
	}
	/*uint16_t* pInspectGridData = &gInspectGridData[0];*/
	/*uint16_t* pInspectNoiseData = &gInspectNoiseData[0];*/
	I("Now Check type = %d\n", checktype);

	I("%s: Init OK, start to test!\n", __func__);
	rslt_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!rslt_log) {
		E("%s: rslt_log alloc failed\n", __func__);
		kfree(RAW);
		RAW = NULL;
		return -ENOMEM;
	}
	start_log = kzalloc(256 * sizeof(char), GFP_KERNEL);
	if (!start_log) {
		E("%s: start_log alloc failed\n", __func__);
		kfree(rslt_log);
		kfree(RAW);
		rslt_log = NULL;
		return -ENOMEM;
	}

	if (himax_check_mode(checktype)) {
		I("Need Change Mode ,target=%s\n", g_himax_inspection_mode[checktype]);

		g_core_fp.fp_sense_off(true);

#ifndef HX_ZERO_FLASH
		if (g_core_fp.fp_reload_disable != NULL)
			g_core_fp.fp_reload_disable(1);
#endif

		himax_switch_mode_inspection(checktype);

		if (checktype == HIMAX_INSPECTION_WEIGHT_NOISE ||
		checktype == HIMAX_INSPECTION_ABS_NOISE) {
			himax_set_N_frame(NOISEFRAME, checktype);
		} else if (checktype == HIMAX_INSPECTION_ACT_IDLE_RAWDATA || checktype == HIMAX_INSPECTION_ACT_IDLE_NOISE) {
			I("N frame = %d\n", 10);
			himax_set_N_frame(10, checktype);

		} else if (checktype >= HIMAX_INSPECTION_LPWUG_RAWDATA) {
			I("N frame = %d\n", 1);
			himax_set_N_frame(1, checktype);

		} else {
			himax_set_N_frame(NOISEFRAME, checktype);
		}

		g_core_fp.fp_sense_on(1);

		ret = himax_wait_sorting_mode(checktype);
		if (ret) {
			E("%s: himax_wait_sorting_mode FAIL\n", __func__);
			ret_val = ret;
			goto BROKEN_END;
		}
	}

	himax_switch_data_type(checktype);

	ret = himax_get_rawdata(RAW, datalen);
	if (ret) {
		E("%s: himax_get_rawdata FAIL\n", __func__);
		ret_val = ret;
		kfree(RAW);
		RAW = NULL;
		goto BROKEN_END;
	}

	/* back to normal */
	himax_switch_data_type(HIMAX_INSPECTION_BACK_NORMAL);

	snprintf(start_log, 256 * sizeof(char), "\n%s%s\n", g_himax_inspection_mode[checktype], ": data as follow!\n");

	/*Check Data*/
	switch (checktype) {
	case HIMAX_INSPECTION_SORTING:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] < g_inspection_criteria[IDX_SORTMIN][i]) {
				E("%s: sorting mode open test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: sorting mode open test PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_OPENMAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_OPENMIN][i]) {
				E("%s: open test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EOPEN;
				goto FAIL_END;
			}
		}
		I("%s: open test PASS\n", __func__);

		break;

	case HIMAX_INSPECTION_MICRO_OPEN:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_M_OPENMAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_M_OPENMIN][i]) {
				E("%s: micro open test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EMOPEN;
				goto FAIL_END;
			}
		}
		I("%s: micro open test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_SHORT:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_SHORTMAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_SHORTMIN][i]) {
				E("%s: short test FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ESHORT;
				goto FAIL_END;
			}
		}
		I("%s: short test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/* I("Now new compare, datalen=%d!\n",ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); */
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_RAWMAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_RAWMIN][i]) {
				E("%s: rawdata test FAIL:RAW[%d]=%d\n", __func__, i, RAW[i]);
				I("%s: Now Criteria max=%d,min=%d\n", __func__, g_inspection_criteria[IDX_RAWMAX][i], g_inspection_criteria[IDX_RAWMIN][i]);
				ret_val = HX_INSPECT_ERAW;
				goto FAIL_END;
			}
		}
		I("%s: rawdata test PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_WEIGHT_NOISE:
		/*I("NOISEMAX=%d\n", NOISEMAX);*/
		noise_count = 0;
		himax_get_noise_base();
		palm_num = himax_get_palm_num();
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > NOISEMAX)
				noise_count++;
		}
		I("noise_count=%d\n", noise_count);
		if (noise_count > palm_num) {
			E("%s: noise test FAIL\n", __func__);
			ret_val = HX_INSPECT_WT_ENOISE;
			goto FAIL_END;
		}
		/*snprintf(start_log, 256 * sizeof(char), "\n Threshold = %d\n", NOISEMAX);*/
		/*Check weightingt*/
		ret_val = himax_get_noise_weight_test();
		if (ret_val < 0) {
			I("%s: noise test FAIL %d\n", __func__, ret_val);
			ret_val = HX_INSPECT_WT_ENOISE;
			goto FAIL_END;
		}

		/*Check negative side noise*/
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] >	(g_inspection_criteria[IDX_WT_NOISEMAX][i] * g_recal_thx / 100) ||
			(int)RAW[i] < (g_inspection_criteria[IDX_WT_NOISEMIN][i] * g_recal_thx / 100)) {
				E(FAIL_IN_INDEX, __func__,
				"HIMAX_INSPECTION_WEIGHT_NOISE", i);
				ret_val = HX_INSPECT_WT_ENOISE;
				goto FAIL_END;
			}
		}
		I("%s: noise test(weight) PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_ABS_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			if ((int)RAW[i] > g_inspection_criteria[IDX_ABS_NOISEMAX][i]
			|| (int)RAW[i] < g_inspection_criteria[IDX_ABS_NOISEMIN][i]) {
				E(FAIL_IN_INDEX, __func__,
				"HIMAX_INSPECTION_ABS_NOISE", i);
				I("%s: Now Criteria max=%d,min=%d\n", __func__,
				  g_inspection_criteria[IDX_ABS_NOISEMAX][i],
				  g_inspection_criteria[IDX_ABS_NOISEMIN][i]);
				ret_val = HX_INSPECT_ABS_ENOISE;
				goto FAIL_END;
			}
		}
		I("%s: noise test(absolute) PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_GAPTEST_RAW:
#ifdef HX_GAP
		if (himax_gap_test_vertical_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
		if (himax_gap_test_honrizontal_raw(HIMAX_INSPECTION_GAPTEST_RAW, RAW) != NO_ERR) {
			E("%s: HIMAX_INSPECTION_GAPTEST_RAW FAIL\n", __func__);
			ret_val = HX_INSPECT_EGAP_RAW;
			goto FAIL_END;
		}
#endif
		break;

	case HIMAX_INSPECTION_ACT_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_RAWDATA_MIN][i]) {
				E("%s: HIMAX_INSPECTION_ACT_IDLE_RAWDATA FAIL  in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EACT_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_ACT_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_ACT_IDLE_NOISE_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_ACT_IDLE_NOISE_MIN][i]) {
				E("%s: HIMAX_INSPECTION_ACT_IDLE_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_EACT_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_ACT_IDLE_NOISE PASS\n", __func__);
		break;

	case HIMAX_INSPECTION_LPWUG_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_RAWDATA_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_RAWDATA_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_RAWDATA FAIL  in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_NOISE_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_NOISE_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_NOISE PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_RAWDATA_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_RAW;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_RAWDATA PASS\n", __func__);
		break;
	case HIMAX_INSPECTION_LPWUG_IDLE_NOISE:
		for (i = 0; i < (ic_data->HX_TX_NUM*ic_data->HX_RX_NUM); i++) {
			/*if (himax_check_notch(i)) {
				continue;
			}*/
			if ((int)RAW[i] > g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MAX][i] || (int)RAW[i] < g_inspection_criteria[IDX_LPWUG_IDLE_NOISE_MIN][i]) {
				E("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE FAIL in index %d\n", __func__, i);
				ret_val = HX_INSPECT_ELPWUG_IDLE_NOISE;
				goto FAIL_END;
			}
		}
		I("%s: HIMAX_INSPECTION_LPWUG_IDLE_NOISE PASS\n", __func__);
		break;

	default:
		E("Wrong type=%d\n", checktype);
		break;
	}

	ret_val = HX_INSPECT_OK;
	snprintf(rslt_log, 256 * sizeof(char), "\n%s%s\n", g_himax_inspection_mode[checktype], " Test Pass!\n");
	I("pass write log\n");
	goto END_FUNC;

BROKEN_END:
	I("%s: fail flow/Init... start to log!\n", __func__);
	snprintf(start_log, 256 * sizeof(char), "\n%s%s\n", g_himax_inspection_mode[checktype], ": data as follow!\n");
	snprintf(rslt_log, 256 * sizeof(char),	"\nChange mode | Get data Fail:%s!\n", g_himax_inspection_mode[checktype]);
	I("Flow/operation fail log\n");
	goto END_FUNC;

FAIL_END:
	snprintf(rslt_log, 256 * sizeof(char),  "\n%s%s\n", g_himax_inspection_mode[checktype], " Test Fail!\n");
	I("fail write log\n");
END_FUNC:
	hx_test_data_get(RAW, start_log, rslt_log, checktype);
	kfree(RAW);
	kfree(start_log);
	kfree(rslt_log);
	return ret_val;
}

/* parsing Criteria start */
static int himax_get_criteria_size(void)
{
	int result = 0;

	result = HX_CRITERIA_SIZE;

	return result;
}

/* claculate 10's power function */
static int himax_power_cal(int pow, int number)
{
	int i = 0;
	int result = 1;

	for (i = 0; i < pow; i++)
		result *= 10;
	result = result * number;

	return result;

}

/* String to int */
static int hiamx_parse_str2int(char *str)
{
	int i = 0;
	int temp_cal = 0;
	int result = 0;
	int str_len = strlen(str);
	int negtive_flag = 0;
	for (i = 0; i < strlen(str); i++) {
		if (str[i] != '-' && str[i] > '9' && str[i] < '0') {
			E("%s: Parsing fail!\n", __func__);
			result = -9487;
			negtive_flag = 0;
			break;
		}
		if (str[i] == '-') {
			negtive_flag = 1;
			continue;
		}
		temp_cal = str[i] - '0';
		result += himax_power_cal(str_len-i-1, temp_cal); /* str's the lowest char is the number's the highest number
															So we should reverse this number before using the power function
															-1: starting number is from 0 ex:10^0 = 1,10^1=10*/
	}

	if (negtive_flag == 1)
		result = 0 - result;
	return result;
}


/* Get sub-string from original string by using some charaters return size of result*/
static int himax_saperate_comma(const struct firmware *file_entry, char **result, int str_size)
{
	int count = 0;
	int str_count = 0; /* now string*/
	int char_count = 0; /* now char count in string*/

	do {
		switch (file_entry->data[count]) {
		case ASCII_COMMA:
		case ACSII_SPACE:
		case ASCII_CR:
		case ASCII_LF:
			count++;
			/* If end of line as above condifiton, differencing the count of char.
				If char_count != 0 it's meaning this string is parsing over .
				The Next char is belong to next string */
			if (char_count != 0) {
				char_count = 0;
				str_count++;
			}
			break;
		default:
			result[str_count][char_count++] = file_entry->data[count];
			count++;
			break;
		}
	} while (count < file_entry->size && str_count < str_size);

	return str_count;
}

static int hx_diff_str(char *str1, char *str2)
{
	int i = 0;
	int result = 0; /* zero is all same, non-zero is not same index*/
	int str1_len = strlen(str1);
	int str2_len = strlen(str2);

	if (str1_len != str2_len) {
		I("%s:Size different!\n", __func__);
		return LENGTH_FAIL;
	}

	for (i = 0; i < str1_len; i++) {
		if (str1[i] != str2[i]) {
			result = i+1;
			I("%s: different in %d!\n", __func__, result);
			return result;
		}
	}

	return result;
}

/* get idx of criteria whe parsing file */
int hx_find_crtra_id(char *input)
{
	int i = 0;
	int result = 0;

	for (i = 0 ; i < HX_CRITERIA_SIZE ; i++) {
		if (hx_diff_str(g_hx_inspt_crtra_name[i], input) == 0) {
			result = i;
			I("find the str=%s,idx=%d\n", g_hx_inspt_crtra_name[i], i);
			break;
		}
	}
	if (i > (HX_CRITERIA_SIZE - 1)) {
		E("%s: find Fail!\n", __func__);
		result = -1;
		return result;
	}

	return result;
}

int hx_print_crtra_after_parsing(void)
{
	int i = 0, j = 0;
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;

	for (i = 0; i < HX_CRITERIA_SIZE; i++) {
		I("Now is %s\n", g_hx_inspt_crtra_name[i]);
		if (g_inspt_crtra_flag[i] == 1) {
			for (j = 0; j < all_mut_len; j++) {
				I("%d, ", g_inspection_criteria[i][j]);
				if (j % 16 == 15)
					printk("\n");
			}
		} else {
			I("No this Item in this criteria file!\n");
		}
		printk("\n");
	}

	return 0;
}

static int hx_get_crtra_by_name(char **result, int size_of_result_str)
{
	int i = 0;
	/* count of criteria type */
	int count_type = 0;
	/* count of criteria data */
	int count_data = 0;
	int err = HX_INSPECT_OK;
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;
	int temp = 0;

	/* get criteria and assign to a global array(2-Dimensional/int) */
	/* basiclly the size of criteria will be (crtra_count * (all_mut_len) + crtra_count)
		but we use file size to be the end of counter*/
	for (i = 0; i < size_of_result_str && result[i] != NULL; i++) {
		/* It have get one page(all mutual) criteria data!
			And we should skip the string of criteria name!
		*/
		if (i == 0 || i == ((i / (all_mut_len))+(i / (all_mut_len) * (all_mut_len)))) {
			count_data = 0;

			if (private_ts->debug_log_level & BIT(4))
				I("Now find str=%s ,idx=%d\n", result[i], i);
			/* check the item of criteria is in criteria file or not*/
			count_type = hx_find_crtra_id(result[i]);
			if (count_type < 0) {
				E("1. %s:Name Not match!\n", __func__);
				/* E("can recognize[%d]=%s\n", count_type, g_hx_inspt_crtra_name[count_type]); */
				E("get from file[%d]=%s\n", i, result[i]);
				E("Please check criteria file again!\n");
				err = HX_INSPECT_EFILE;
				return err;
			} else {
				I("Now str=%s, idx=%d\n", g_hx_inspt_crtra_name[count_type], count_type);
				g_inspt_crtra_flag[count_type] = 1;
			}
			continue;
		}
		/* change string to int*/
		temp = hiamx_parse_str2int(result[i]);
		if (temp != -9487)
			g_inspection_criteria[count_type][count_data] = temp;
		else {
			E("%s: Parsing Fail in %d\n", __func__, i);
			E("in range:[%d]=%s\n", count_type, g_hx_inspt_crtra_name[count_type]);
			E("btw, get from file[%d]=%s\n", i, result[i]);
			break;
		}
		/* dbg
		I("[%d]g_inspection_criteria[%d][%d]=%d\n", i, count_type, count_data, g_inspection_criteria[count_type][count_data]);
		*/
		count_data++;

	}

	if (private_ts->debug_log_level & BIT(4)) {
		/* dbg:print all of criteria from parsing file */
		hx_print_crtra_after_parsing();
	}

	I("Total loop=%d\n", i);

	return err;
}

extern uint8_t *g_criteria_buf;
extern uint32_t g_criteria_sze;
extern uint32_t g_criteria_offset;
extern struct firmware ext_criteria;

static int himax_parse_criteria_file(void)
{
	int err = HX_INSPECT_OK;
	const struct firmware *file_entry = NULL;
	char *file_name = "hx_criteria.csv";
	static char **result = NULL;
	int i = 0;

	int crtra_count = himax_get_criteria_size();
	int data_size = 0; /* The maximum of number Data*/
	int all_mut_len = ic_data->HX_TX_NUM*ic_data->HX_RX_NUM;
	int str_max_len = 32;
	int result_all_len = 0;
	int file_size = 0;
	int size_of_result_str = 0;
	int use_ext_criteria = 0;

	I("%s,Entering \n", __func__);
	I("file name = %s\n", file_name);

	/* default path is /system/etc/firmware */
	err = request_firmware(&file_entry, file_name, private_ts->dev);
	if (err < 0) {
		if (g_criteria_buf != NULL) {
			I("Can't locate hx_criteria, use data in criteria node.\n");
			if (g_criteria_sze != g_criteria_offset) {
				I("File not ready, g_criteria_offset/g_criteria_sze: %u/%u", g_criteria_offset, g_criteria_sze);
				err = HX_INSPECT_EFILE;
				goto END_FUNC_REQ_FAIL;
			}
			file_entry = &ext_criteria;
			use_ext_criteria = 1;
			err = HX_INSPECT_OK;
		} else {
			E("%s,fail in line%d error code=%d\n", __func__, __LINE__, err);
			err = HX_INSPECT_EFILE;
			goto END_FUNC_REQ_FAIL;
		}
	}

	/* size of criteria include name string */
	data_size = ((all_mut_len) * crtra_count) + crtra_count;

	/* init the array which store original criteria and include name string*/
	if (result == NULL) {
		I("%s: result is NULL, alloc memory.\n", __func__);
		result = kzalloc(data_size * sizeof(char *), GFP_KERNEL);
		if (result != NULL) {
			for (i = 0 ; i < data_size; i++) {
				result[i] = kzalloc(str_max_len * sizeof(char), GFP_KERNEL);
				if (!result[i]) {
					err = -ENOMEM;
					E("%s: result[%d] alloc failed\n", __func__, i);
					goto END_FUNC;
				}
			}
			
		} else {
			err = -ENOMEM;
			E("%s: result alloc failed\n", __func__);
			goto END_FUNC_REQ_FAIL;
		}
	} else {
	
        for (i = 0 ; i < data_size; i++) {
			memset(result[i], 0x00, str_max_len * sizeof(char));
		}      
    }

	result_all_len = data_size;
	file_size =	file_entry->size;
	I("Now result_all_len=%d\n", result_all_len);
	I("Now file_size=%d\n", file_size);

	/* dbg */
	if (private_ts->debug_log_level & BIT(4))
		I("first 4 bytes 0x%2X,0x%2X,0x%2X,0x%2X !\n", file_entry->data[0], file_entry->data[1], file_entry->data[2], file_entry->data[3]);

	/* parse value in to result array(1-Dimensional/String) */
	size_of_result_str = himax_saperate_comma(file_entry, result, data_size);

	I("%s: now size_of_result_str=%d\n", __func__, size_of_result_str);

	err = hx_get_crtra_by_name(result, size_of_result_str);
	if (err != HX_INSPECT_OK) {
		E("%s:Load criteria from file fail, go end!\n", __func__);
		goto END_FUNC;
	}
	goto ALLOC_END_FUNC;


END_FUNC:
	for (i = 0 ; i < data_size; i++)
		kfree(result[i]);
	kfree(result);
	result = NULL;
ALLOC_END_FUNC:
	if (use_ext_criteria == 0)
		release_firmware(file_entry);
END_FUNC_REQ_FAIL:
	I("%s,END \n", __func__);
	return err;
	/* parsing Criteria end */
}


int hx_get_size_str_arr(char **input)
{
	int i = 0;
	int result = 0;

	while (input[i] != NULL)
		i++;
	result = i;
	if (private_ts->debug_log_level & BIT(4))
		I("There is %d in [0]=%s\n", result, input[0]);
	return result;
}

static int himax_self_test_data_init(void)
{
	int ret = HX_INSPECT_OK;
	int i = 0;

	/* get test item and its items of criteria*/
	HX_CRITERIA_ITEM = hx_get_size_str_arr(g_himax_inspection_mode);
	HX_CRITERIA_SIZE = hx_get_size_str_arr(g_hx_inspt_crtra_name);
	I("There is %d HX_CRITERIA_ITEM and %d HX_CRITERIA_SIZE\n", HX_CRITERIA_ITEM, HX_CRITERIA_SIZE);

	/* init criteria data*/
	g_inspt_crtra_flag = kzalloc(HX_CRITERIA_SIZE*sizeof(int), GFP_KERNEL);
	if (!g_inspt_crtra_flag) {
		E("%s: g_inspt_crtra_flag alloc failed\n", __func__);
		return -ENOMEM;
	}
	g_inspection_criteria = kzalloc(sizeof(int *)*HX_CRITERIA_SIZE, GFP_KERNEL);
	if (!g_inspection_criteria) {
		E("%s: g_inspection_criteria alloc failed\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < HX_CRITERIA_SIZE; i++){
		g_inspection_criteria[i] = kzalloc(sizeof(int)*(ic_data->HX_TX_NUM*ic_data->HX_RX_NUM), GFP_KERNEL);
		if (!g_inspection_criteria[i]) {
			E("%s: g_inspection_criteria[%d] alloc failed\n", __func__, i);
			return -ENOMEM;
		}
	}
	/* parsing criteria from file*/
	ret = himax_parse_criteria_file();

	if (private_ts->debug_log_level & BIT(4)) {
		/* print get criteria string */
		for (i = 0 ; i < HX_CRITERIA_SIZE ; i++) {
			if (g_inspt_crtra_flag[i] != 0)
				I("%s: [%d]There is String=%s\n", __func__, i, g_hx_inspt_crtra_name[i]);
		}
	}

	g_rslt_data = kzalloc(0x1000 * HX_CRITERIA_ITEM * sizeof(char), GFP_KERNEL);
	if (!g_rslt_data) {
		E("%s: g_rslt_data alloc failed\n", __func__);
		return -ENOMEM;
	}
	return ret;
}

static void himax_self_test_data_deinit(void)
{
	int i = 0;

	/*dbg*/
	/*for (i = 0; i < HX_CRITERIA_ITEM; i++)
		I("%s:[%d]%d\n", __func__, i, g_inspection_criteria[i]);*/
	if (g_inspection_criteria != NULL) {
		for (i = 0; i < HX_CRITERIA_SIZE; i++)
			kfree(g_inspection_criteria[i]);
		kfree(g_inspection_criteria);
		I("Now it have free the g_inspection_criteria!\n");
	} else {
		I("No Need to free g_inspection_criteria!\n");
	}

	kfree(g_inspt_crtra_flag);
	kfree(g_rslt_data);

}

static int himax_chip_self_test(struct seq_file *s, void *v) /*#9 ++09-27++*/
{
	uint32_t ret = HX_INSPECT_OK;
	char g_test_flag[10] = {0};
	int i = 0;
	int j = 0;
	uint32_t test_size = ic_data->HX_TX_NUM	* ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM;

	I("%s:Selftest Enter\n", __func__);

	I("%s, s->size = %d\n", __func__, (int)s->size);
	I("%s, sizeof(s->buf) = %d\n", __func__, (int)sizeof(s->buf));
	I("%s\n", s->buf);
	
	s->size = 24*1024;
	
	s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
	if (s->buf == NULL) {
		E("Memory allocation falied!\n");
		ret = -ENOMEM;

		s->size = 50;
		s->buf = kcalloc(s->size, sizeof(char), GFP_KERNEL);
		if (s->buf == NULL) {
			E("Memory allocation falied!\n");
		} else {

			seq_printf(s, "%s\n", "Memory allocation Fail");
		}
		return ret;
	}
	
	memset(s->buf, 0, s->size*sizeof(char));

	ret = himax_self_test_data_init();
	if (ret != HX_INSPECT_OK) {
		E("%s: himax_self_test_data_init fail!\n", __func__);
		seq_printf(s, "%s\n", "init Memory allocation Fail");
		goto END_FUNC;
	}


	for (i = 0; i < HX_CRITERIA_SIZE; i = i + 2 ) {
		
		if (g_inspt_crtra_flag[i] == 1 && g_inspt_crtra_flag[i+1] == 1) {
			
			ret += mpTestFunc(j, test_size);
			if (ret == HX_INSPECT_OK)
				g_test_flag[j] = 1;
			j++;
			
		} else {
			I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[i], g_inspt_crtra_flag[i]);
			I("Now %s :flag=%d\n", g_hx_inspt_crtra_name[i+1], g_inspt_crtra_flag[i+1]);
		}
	}

	if (ret == HX_INSPECT_OK)
		seq_puts(s, "Self_Test Pass:\n");
	else
		seq_puts(s, "Self_Test Fail:\n");

	for (i = 0; i < 5; i++) {
		
		if (g_test_flag[i] == 1) {
			
			seq_printf(s, "%s : %s\n", g_himax_inspection_mode[i], "OK");
		} else {

			seq_printf(s, "%s : %s\n", g_himax_inspection_mode[i], "Fail");
		}
		
		if (g_himax_inspection_mode[i] == NULL) {

			break;
		}
	}
	
	seq_printf(s, g_rslt_data);
	
	g_core_fp.fp_sense_off(true);
	himax_set_N_frame(1, HIMAX_INSPECTION_WEIGHT_NOISE);
#ifndef HX_ZERO_FLASH
	if (g_core_fp.fp_reload_disable != NULL)
		g_core_fp.fp_reload_disable(0);
#endif
	g_core_fp.fp_sense_on(0);

END_FUNC:
	himax_self_test_data_deinit();

	I("running status = %d \n", ret);

	if (ret != 0)
		ret = 1;

	I("%s:OUT\n", __func__);
	return ret;
}


void himax_inspection_init(void)
{
	I("%s: enter, %d \n", __func__, __LINE__);

	g_core_fp.fp_chip_self_test = himax_chip_self_test;

	return;
}
