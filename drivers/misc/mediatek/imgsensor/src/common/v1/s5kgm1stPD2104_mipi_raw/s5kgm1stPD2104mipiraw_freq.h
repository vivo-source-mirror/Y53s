
/*******************preview_interference ARFCN start********************************************************/
static struct FREQ LTE_preview_0_interference[] = {//mipi-clk:600MHz
	{9265,9465},//B28:CH:936,  773.5 MHz
	{2450,2649},//B5:CH:2550,  884 MHz
	{38755,38955},//B40:CH:38855,  2320.5 MHz
	{40005,40205},//B41:CH:40105,  2541.5 MHz
	{41110,41310},//B41:CH:41210,  2652 MHz
	{2970,3170},//B7:CH:3070,  2652 MHz
	{716666,723200},//B79
	{0, 0}
};

static struct FREQ LTE_preview_1_interference[] = {//mipi-clk:612MHz
	{1421,1621},//  B3:CH:1521,  1837.0625 MHz
	{412,599},//    B1:CH:512,  2161.25 MHz
	{39324,39524},//B40:CH:39424,  2377.375 MHz
	{37885,38085},//B38:CH:37985,  2593.5 MHz
	{40525,40725},//B38:CH:40625,  2593.5 MHz
	{723200,733333},//B79
	{0, 0}
};

static struct FREQ LTE_preview_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ LTE_preview_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ UMTS_preview_0_interference[] = {//mipi-clk:442.00MHz
	{4355,4385},//WCDMA:B5:CH:4370,  884 MHz
	{50020,50021},//for-AT command-clk0
	{0, 0}
};

static struct FREQ UMTS_preview_1_interference[] = {//mipi-clk:432.25MHz
	{10793,10819},//WCDMA:B1:CH:10806,  2161.25 MHz
	{50021,50022},//for-AT command-clk1
	{0, 0}
};

static struct FREQ UMTS_preview_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ UMTS_preview_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ GSM_preview_0_interference[] = {//mipi-clk:442.00MHz
	{142,162},//GSM850:CH:152,  884 MHz
	{872,883},//GSM1800:CH:878,  1878.5 MHz
	{0, 0}
};

static struct FREQ GSM_preview_1_interference[] = {//mipi-clk:432.25MHz
	{666,676},//GSM1800:CH:671,  1837.0625 MHz
	{0, 0}
};

static struct FREQ GSM_preview_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ GSM_preview_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ WIFI_preview_0_interference[] = {//mipi-clk:442.00MHz
	{59,63},//WIFI:CH:61,  5304 MHz
	{148,152},//WIFI:CH:149,  5746 MHz
	{14,15},//WIFI:CH:14,  2485.4375 MHz
	{50020,50021},//for-AT command-clk0
	{0, 0}
};

static struct FREQ WIFI_preview_1_interference[] = {//mipi-clk:432.25MHz
	{36,40},//WIFI:CH:37,  5304 MHz
	{122,126},//WIFI:CH:124,  5619.25 MHz
	{3,7},//WIFI:CH:5,  2431 MHz
	{50021,50022},//for-AT command-clk1
	{0, 0}
};

static struct FREQ WIFI_preview_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ WIFI_preview_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};
/*******************preview_interference ARFCN end********************************************************/

/*******************capture_interference ARFCN start********************************************************/
static struct FREQ LTE_capture_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_capture_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_capture_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_capture_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_capture_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_capture_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_capture_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_capture_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_capture_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_capture_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_capture_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_capture_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_capture_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_capture_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_capture_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ WIFI_capture_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};
/*******************capture_interference ARFCN end********************************************************/

/*******************video_interference ARFCN start********************************************************/
static struct FREQ LTE_video_0_interference[] = {//mipi-clk:600 MHz
	{9232,9432},//--B28:CH:9332,  770.25 MHz
	{36219,36349},//B34:CH:36319,  1829.34375 MHz
	{0,182},//-------B1:CH:82,  2118.1875 MHz
	{38658,38858},//B40:CH:38758,  2310.75 MHz
	{37946,38146},//B38:CH:38046,  2599.59375 MHz
	{40586,40786},//B41:CH:40686,  2599.59375 MHz
	{716666,723200},//B79
	{0, 0}
};

static struct FREQ LTE_video_1_interference[] = {//mipi-clk:612 MHz
	{3566,3799},//---B8:CH:3666,  946.5625 MHz
	{39214,39414},//B40:CH:39314,  2366.40625 MHz
	{40147,40347},//B41:CH:40247,  2555.71875 MHz
	{41094,41294},//B41:CH:41194,  2650.375 MHz
	{2954,3154},//---B7:CH:3054,  2650.375 MHz
	{723200,733333},//B79
	{0, 0}
};

static struct FREQ LTE_video_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ LTE_video_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};
////////
static struct FREQ UMTS_video_0_interference[] = {//mipi-clk:385.125 MHz
	{10578,10604},//WCDMA:B1:CH:10591,  2118.1875 MHz
	{10105,10115},//TDSCDMA:B34:CH:10110,  2021.90625 MHz
	{50020,50021},//for-AT command-clk0
	{0, 0}
};

static struct FREQ UMTS_video_1_interference[] = {//mipi-clk:378.625 MHz
	{3020,3046},//WCDMA:B8:CH:3033,  946.5625 MHz
	{50021,50022},//for-AT command-clk1
	{0, 0}
};

static struct FREQ UMTS_video_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ UMTS_video_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};
////////
static struct FREQ GSM_video_0_interference[] = {//mipi-clk:385.125 MHz
	{628,638},//GSM1800:CH:633,  1829.34375 MHz
	{0, 0}
};

static struct FREQ GSM_video_1_interference[] = {//mipi-clk:378.625 MHz
	{52,63},//GSM900:CH:58,  946.5625 MHz
	{0, 0}
};

static struct FREQ GSM_video_2_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};

static struct FREQ GSM_video_3_interference[] = {
	{0, 0Xffffffff},//all
	{0, 0}
};
////////
static struct FREQ WIFI_video_0_interference[] = {//mipi-clk:385.125 MHz
	{154,158},//WIFI:CH:155,  5776.875 MHz
	{1,3},//WIFI:CH:1,  2407.03125 MHz
	{50020,50021},//for-AT command-clk1
	{0, 0}
};

static struct FREQ WIFI_video_1_interference[] = {//mipi-clk:378.625 MHz
	{134,138},//WIFI:CH:136,  5679.375 MHz
	{59,63},//  WIFI:CH:60,  5300.75 MHz
	{9,13},//   WIFI:CH:11,  2461.0625 MHz
	{50021,50022},//for-AT command-clk1
	{0, 0}
};

static struct FREQ WIFI_video_2_interference[] = {
	{0, 0Xffffffff},//WIFI
	{0, 0}
};

static struct FREQ WIFI_video_3_interference[] = {
	{0, 0Xffffffff},//WIFI
	{0, 0}
};
/*******************video_interference ARFCN end********************************************************/

/*******************high_video_speed_interference ARFCN start********************************************************/
static struct FREQ LTE_high_speed_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_high_speed_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_high_speed_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_high_speed_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_high_speed_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_high_speed_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_high_speed_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_high_speed_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_high_speed_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_high_speed_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_high_speed_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_high_speed_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_high_speed_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_high_speed_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_high_speed_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_high_speed_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

/*******************high_speed_video_interference ARFCN end********************************************************/

/*******************slim_video_interference ARFCN start********************************************************/

static struct FREQ LTE_slim_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_slim_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_slim_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ LTE_slim_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};
////////
static struct FREQ UMTS_slim_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_slim_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_slim_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ UMTS_slim_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_slim_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_slim_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_slim_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ GSM_slim_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_slim_video_0_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_slim_video_1_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_slim_video_2_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

static struct FREQ WIFI_slim_video_3_interference[] = {
	{0, 0Xffffffff},//-all
	{0, 0}
};

/*******************slim_video_interference ARFCN end********************************************************/

/***************************preview_setting start*********************************************/
static kal_uint16 preview_setting_0[] = {
	0x0310, 0x0064,//600MHz~1200Mbps
};

static kal_uint16 preview_setting_1[] = {
	0x0310, 0x0066,//612MHz~1224Mbps
};

static kal_uint16 preview_setting_2[] = {
};

static kal_uint16 preview_setting_3[] = {
};
/***************************preview_setting end*********************************************/

/***************************capture_setting start*********************************************/
static kal_uint16 capture_setting_0[] = {

};

static kal_uint16 capture_setting_1[] = {
};

static kal_uint16 capture_setting_2[] = {
};

static kal_uint16 capture_setting_3[] = {
};
/***************************capture_setting end*********************************************/

/***************************video_setting start*********************************************/
static kal_uint16 video_setting_0[] = {
	0x0310, 0x0064,//600MHz~1200Mbps
};
	
static kal_uint16 video_setting_1[] = {
	0x0310, 0x0066,//612MHz~1224Mbps
};

static kal_uint16 video_setting_2[] = {
};

static kal_uint16 video_setting_3[] = {
};
/***************************video_setting end*********************************************/

/***************************high_speed_video_setting start*********************************************/
static kal_uint16 high_speed_video_setting_0[] = {
};

static kal_uint16 high_speed_video_setting_1[] = {
};

static kal_uint16 high_speed_video_setting_2[] = {
};

static kal_uint16 high_speed_video_setting_3[] = {
};
/***************************high_speed_video_setting end*********************************************/

/***************************slim_video_setting start*********************************************/
static kal_uint16 slim_video_setting_0[] = {
};

static kal_uint16 slim_video_setting_1[] = {
};

static kal_uint16 slim_video_setting_2[] = {
};

static kal_uint16 slim_video_setting_3[] = {
};
/***************************slim_video_setting end*********************************************/


static struct SENSOR_FREQ_LIST sensor_freq_list[5][4] = {
	{ /* preview */
		{
			.freq_list = {GSM_preview_0_interference,
										UMTS_preview_0_interference,
										WIFI_preview_0_interference,
										LTE_preview_0_interference
			},
			.setting = preview_setting_0,
			.setting_size = sizeof(preview_setting_0) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_preview_1_interference,
										UMTS_preview_1_interference,
										WIFI_preview_1_interference,
										LTE_preview_1_interference
			},
			.setting = preview_setting_1,
			.setting_size = sizeof(preview_setting_1) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_preview_2_interference,
										UMTS_preview_2_interference,
										WIFI_preview_2_interference,
										LTE_preview_2_interference
			},
			.setting = preview_setting_2,
			.setting_size = sizeof(preview_setting_2) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_preview_3_interference,
										UMTS_preview_3_interference,
										WIFI_preview_3_interference,
										LTE_preview_3_interference
			},
			.setting = preview_setting_3,
			.setting_size = sizeof(preview_setting_3) / sizeof(kal_uint16)
		},
	},
	{ /* capture */

		{
			.freq_list = {GSM_capture_0_interference,
										UMTS_capture_0_interference,
										WIFI_capture_0_interference,
										LTE_capture_0_interference
			},
			.setting = capture_setting_0,
			.setting_size = sizeof(capture_setting_0) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_capture_1_interference,
										UMTS_capture_1_interference,
										WIFI_capture_1_interference,
										LTE_capture_1_interference
			},
			.setting = capture_setting_1,
			.setting_size = sizeof(capture_setting_1) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_capture_2_interference,
										UMTS_capture_2_interference,
										WIFI_capture_2_interference,
										LTE_capture_2_interference
			},
			.setting = capture_setting_2,
			.setting_size = sizeof(capture_setting_2) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_capture_3_interference,
										UMTS_capture_3_interference,
										WIFI_capture_3_interference,
										LTE_capture_3_interference
			},
			.setting = capture_setting_3,
			.setting_size = sizeof(capture_setting_3) / sizeof(kal_uint16)
		},
	},
	{ /* video */

		{
			.freq_list = {GSM_video_0_interference,
										UMTS_video_0_interference,
										WIFI_video_0_interference,
										LTE_video_0_interference
			},
			.setting = video_setting_0,
			.setting_size = sizeof(video_setting_0) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_video_1_interference,
										UMTS_video_1_interference,
										WIFI_video_1_interference,
										LTE_video_1_interference
			},
			.setting = video_setting_1,
			.setting_size = sizeof(video_setting_1) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_video_2_interference,
										UMTS_video_2_interference,
										WIFI_video_2_interference,
										LTE_video_2_interference
			},
			.setting = video_setting_2,
			.setting_size = sizeof(video_setting_2) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_video_3_interference,
										UMTS_video_3_interference,
										WIFI_video_3_interference,
										LTE_video_3_interference
			},
			.setting = video_setting_3,
			.setting_size = sizeof(video_setting_3) / sizeof(kal_uint16)
		},
	},
	{ /* high_speed_video */

		{
			.freq_list = {GSM_high_speed_video_0_interference,
										UMTS_high_speed_video_0_interference,
										WIFI_high_speed_video_0_interference,
										LTE_high_speed_video_0_interference
			},
			.setting = high_speed_video_setting_0,
			.setting_size = sizeof(high_speed_video_setting_0) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_high_speed_video_1_interference,
										UMTS_high_speed_video_1_interference,
										WIFI_high_speed_video_1_interference,
										LTE_high_speed_video_1_interference
			},
			.setting = high_speed_video_setting_1,
			.setting_size = sizeof(high_speed_video_setting_1) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_high_speed_video_2_interference,
										UMTS_high_speed_video_2_interference,
										WIFI_high_speed_video_2_interference,
										LTE_high_speed_video_2_interference
			},
			.setting = high_speed_video_setting_2,
			.setting_size = sizeof(high_speed_video_setting_2) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_high_speed_video_3_interference,
										UMTS_high_speed_video_3_interference,
										WIFI_high_speed_video_3_interference,
										LTE_high_speed_video_3_interference
			},
			.setting = high_speed_video_setting_3,
			.setting_size = sizeof(high_speed_video_setting_3) / sizeof(kal_uint16)
		},
	},
	{ /* slim video */

		{
			.freq_list = {GSM_slim_video_0_interference,
										UMTS_slim_video_0_interference,
										WIFI_slim_video_0_interference,
										LTE_slim_video_0_interference
			},
			.setting = slim_video_setting_0,
			.setting_size = sizeof(slim_video_setting_0) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_slim_video_1_interference,
										UMTS_slim_video_1_interference,
										WIFI_slim_video_1_interference,
										LTE_slim_video_1_interference
			},
			.setting = slim_video_setting_1,
			.setting_size = sizeof(slim_video_setting_1) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_slim_video_2_interference,
										UMTS_slim_video_2_interference,
										WIFI_slim_video_2_interference,
										LTE_slim_video_2_interference
			},
			.setting = slim_video_setting_2,
			.setting_size = sizeof(slim_video_setting_2) / sizeof(kal_uint16)
		},
		{
			.freq_list = {GSM_slim_video_3_interference,
										UMTS_slim_video_3_interference,
										WIFI_slim_video_3_interference,
										LTE_slim_video_3_interference
			},
			.setting = slim_video_setting_3,
			.setting_size = sizeof(slim_video_setting_3) / sizeof(kal_uint16)
		}
	}
};
