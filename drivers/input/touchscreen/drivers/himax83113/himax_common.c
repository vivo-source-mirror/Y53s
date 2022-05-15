/*  Himax Android Driver Sample Code for common functions

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/
#include "himax_common.h"
#include "himax_ic_core.h"
#ifdef HX_SMART_WAKEUP
#define GEST_SUP_NUM 26
/*Setting cust key define (DF = double finger)*/
/*{Double Tap, Up, Down, Left, Rright, C, Z, M,
	O, S, V, W, e, m, @, (reserve),
	Finger gesture, ^, >, <, f(R), f(L), Up(DF), Down(DF),
	Left(DF), Right(DF)}*/

	uint8_t gest_event[GEST_SUP_NUM] = {
	0x80, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x81, 0x1D, 0x2D, 0x3D, 0x1F, 0x2F, 0x51, 0x52,
	0x53, 0x54};

/*gest_event mapping to gest_key_def*/
	uint16_t gest_key_def[GEST_SUP_NUM] = {
	KEY_WAKEUP, KEY_UP, KEY_WAKEUP_SWIPE, KEY_LEFT, KEY_RIGHT, KEY_C, 256, KEY_M,
	KEY_O, 259, 260, KEY_W, KEY_E, 263, KEY_A, 265,
	266, 267, 268, 269, KEY_F, KEY_F, 272, 273,
	274, 275};
#endif

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2000)
#define FRAME_COUNT 5

#if defined(HX_ZERO_FLASH)

char i_CTPM_firmware_name[256];

bool g_auto_update_flag;
#endif
#ifdef HX_ZERO_FLASH
int g_f_0f_updat;
#endif

struct himax_ts_data *private_ts;
struct himax_platform_data *hv_pdata;

struct himax_ic_data *ic_data;
struct himax_report_data *hx_touch_data;
struct himax_core_fp g_core_fp;
struct himax_debug *debug_data;
bool g_DSRAM_Flag;//27
uint8_t *event_buf = NULL;


struct proc_dir_entry *himax_touch_proc_dir;
//static struct task_struct *hx_thread;
//extern int hx_ts_work_handler(void *unused);
//extern int tpd_flag;

uint8_t g_hx_ic_dt_num;
struct himax_chip_detect *g_core_chip_dt;

#define HIMAX_PROC_TOUCH_FOLDER "android_touch"

extern int himax_debug_init(void);
extern int himax_debug_remove(void);

uint8_t *g_point_buf;


/*ts_work about start*/
struct himax_target_report_data *g_target_report_data;
int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status, ktime_t kt);
static void himax_report_all_leave_event(struct himax_ts_data *ts);
/*ts_work about end*/
static int		HX_TOUCH_INFO_POINT_CNT;

unsigned long	FW_VER_MAJ_FLASH_ADDR;
unsigned long FW_VER_MIN_FLASH_ADDR;
unsigned long CFG_VER_MAJ_FLASH_ADDR;
unsigned long CFG_VER_MIN_FLASH_ADDR;
unsigned long CID_VER_MAJ_FLASH_ADDR;
unsigned long CID_VER_MIN_FLASH_ADDR;
/*unsigned long	PANEL_VERSION_ADDR;*/

unsigned long FW_VER_MAJ_FLASH_LENG;
unsigned long FW_VER_MIN_FLASH_LENG;
unsigned long CFG_VER_MAJ_FLASH_LENG;
unsigned long CFG_VER_MIN_FLASH_LENG;
unsigned long CID_VER_MAJ_FLASH_LENG;
unsigned long CID_VER_MIN_FLASH_LENG;
/*unsigned long	PANEL_VERSION_LENG;*/

unsigned long FW_CFG_VER_FLASH_ADDR;

unsigned char IC_CHECKSUM;

#ifdef HX_ESD_RECOVERY
	u8 HX_ESD_RESET_ACTIVATE = 0;
	int hx_EB_event_flag = 0;
	int hx_EC_event_flag = 0;
	int hx_ED_event_flag = 0;
	int g_zero_event_count = 0;
#endif
u8 HX_HW_RESET_ACTIVATE;

extern int himax_parse_dt(struct device_node *np,struct himax_platform_data *pdata);

static uint8_t AA_press;
static uint8_t EN_NoiseFilter;
static uint8_t Last_EN_NoiseFilter;

static int	p_point_num	= 0xFFFF;
static int	probe_fail_flag;
/*
#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
						unsigned long event, void *data);

#endif
*/
#ifdef HX_GESTURE_TRACK
	static int gest_pt_cnt;
	static u16 gest_pt_x[GEST_PT_MAX_NUM];
	static u16 gest_pt_y[GEST_PT_MAX_NUM];
	static int gest_start_x, gest_start_y, gest_end_x, gest_end_y;
	static int gest_width, gest_height, gest_mid_x, gest_mid_y;
	static int hx_gesture_coor[16];
#endif

int himax_report_data_init(void);

extern int himax_dev_set(struct himax_ts_data *ts);
extern int himax_input_register_device(struct input_dev *input_dev);

int g_ts_dbg;

/* File node for Selftest, SMWP and HSEN - Start*/
#define HIMAX_PROC_SELF_TEST_FILE	"self_test"
struct proc_dir_entry *himax_proc_self_test_file;

uint8_t HX_PROC_SEND_FLAG;
#ifdef HX_SMART_WAKEUP
	#define HIMAX_PROC_SMWP_FILE "SMWP"
	struct proc_dir_entry *himax_proc_SMWP_file = NULL;
	#define HIMAX_PROC_GESTURE_FILE "GESTURE"
	struct proc_dir_entry *himax_proc_GESTURE_file = NULL;
	uint8_t HX_SMWP_EN = 0;
#endif

bool FAKE_POWER_KEY_SEND = true;


#ifdef HX_HIGH_SENSE
	#define HIMAX_PROC_HSEN_FILE "HSEN"
	struct proc_dir_entry *himax_proc_HSEN_file = NULL;
#endif

extern int32_t *getMutualBuffer(void);
extern int himax_get_rawdata(uint32_t RAW[], uint32_t datalen);
/* VIVO start */
int bbk_himax_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size) /*#1 ++09-26++*/
{
	uint8_t *temp_data = NULL;
	int data_len = ic_data->HX_RX_NUM * ic_data->HX_TX_NUM;
	int ret = 0;
	int i;
	int retry = 3;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	himax_int_enable(0);
	temp_data = kzalloc(data_len * 2 * sizeof(uint8_t), GFP_KERNEL);
	if (!temp_data) {
		ret = -ENOMEM;
		goto out;
	}
	if (which == VTS_FRAME_MUTUAL_RAW) {
		I("Now is rawdata!\n");
		g_core_fp.fp_diag_register_set(0x0a, 0x00);
		//himax_get_rawdata(temp_data, data_len);
		//g_core_fp.fp_diag_register_set(0x00, 0x00);
	} else if (which == VTS_FRAME_MUTUAL_DELTA) {
		I("Now is delta!\n");
		g_core_fp.fp_diag_register_set(0x09, 0x00);
		//himax_get_rawdata(temp_data, data_len);
		//g_core_fp.fp_diag_register_set(0x00, 0x00);
	} else {
		E("cmd is not valid !!!");
		goto out;
	}

	while(g_core_fp.fp_get_DSRAM_data(temp_data, false) == false && retry > 0){
		I("%s:get rawdata fail,Now if %d time!\n", __func__, retry--);
	}

	if (retry <= 0) {
		E("%s:Retry fail!GET FAIL!\n", __func__);
		goto out;
	}
	
	for(i = 0; i < data_len; i++) {
		data[i] = temp_data[i * 2] + (temp_data[i * 2 + 1] * 256);
		I("data[%d] = %d", i, data[i]);
	}
	
out:
	kfree(temp_data);
	himax_int_enable(1);
	return ret;

#if 0
	int32_t *mutual_data;
	int32_t *mutual_data_buf;
	uint8_t command = 0x00;
	uint8_t storage_type = 0x00;
	int i;

	mutual_data_buf = kzalloc(ic_data->HX_RX_NUM * ic_data->HX_TX_NUM
			* sizeof(int32_t), GFP_KERNEL);

	if (which == VTS_FRAME_MUTUAL_RAW) {
		/* read rawdata */	
		private_ts->diag_cmd = 2;
		command = 0x02;
	} else if (which == VTS_FRAME_MUTUAL_DELTA) {
		/* read delta/diff data */	
		private_ts->diag_cmd = 1;
		command = 0x01;
	} else {
		goto error;
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
		I("mutual_data_buf[%d] = %d", i, mutual_data_buf[i]);
		if ((i%8) == 0)
			I("\n");
	}
error:
	kfree(mutual_data_buf);
	return 0;
#endif
}

int bbk_himax_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)  /*#2*/
{
	int result = 0;
	struct himax_ts_data *ts = private_ts;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	himax_int_enable(0);
#ifdef HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif

	/*result = g_core_fp.fp_0f_op_file_dirly(fileName);*/

	if (g_f_0f_updat) {
		I("%s:[Warning]Other thread is updating now!\n", __func__);
		result = 0;
		return result;
	} else {
		I("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}
	g_core_fp.fp_firmware_update_0f(firmware);
	//release_firmware(fw_entry);

	g_f_0f_updat = 0;
	ts->have_update_firmware = 1;
/*
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#endif
	g_core_fp.fp_sense_on(0x00);
	msleep(120);
#ifdef HX_ESD_RECOVERY
	HX_ESD_RESET_ACTIVATE = 1;
#endif
	himax_int_enable(1);
	return result;
*/
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_touch_information();

//#ifdef HX_ZERO_FLASH
//	if (g_core_fp.fp_0f_reload_to_active)
//		g_core_fp.fp_0f_reload_to_active();
//#endif

	g_core_fp.fp_sense_on(0x00);

	himax_int_enable(1);


#ifdef HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;
	queue_delayed_work(private_ts->himax_chip_monitor_wq,
			&private_ts->himax_chip_monitor,
			g_chip_monitor_data->HX_POLLING_TIMES*HZ);
#endif

	return result;
}

int bbk_himax_gesture_point_get(uint16_t *data) /*#7*/
{
	int status = -1;
	uint8_t i = 0;
	uint8_t pt_num = 0;

	for (pt_num = 0; pt_num < GEST_PT_MAX_NUM; pt_num++) {
		if (gest_pt_x[pt_num] != 0) {
			*(data + i) = (uint16_t)(gest_pt_x[pt_num] & 0x0000ffff);
			*(data + i + 1) = (uint16_t)(gest_pt_y[pt_num] & 0x0000ffff);
			i += 2;
		} else {
			break;
		}
		if (pt_num < 12) {

			VTI("bbk_gesture_data[%d] = %d\n", pt_num, *(data + pt_num));
			/*VTI("bbk_gesture_data[%d] = %d\n", (pt_num + 1), *(data + pt_num + 1));*/
		}

	}
	if ((i > 0) && (i % 2 == 0))

		status = i/2;
	VTI("bbk_gesture_data status is %d \n", status);

	return status;
}

int bbk_himax_sensor_test(struct seq_file *s, void *v) /*#9*/
{
	size_t ret = 0;

/*==========================Update FW Start==========================================*/

	I("NOW Running Zero flash update!\n");
	//himax_int_enable(0);
	memset(i_CTPM_firmware_name, 0x00, sizeof(i_CTPM_firmware_name));
	memcpy(i_CTPM_firmware_name, HX_MP_FW, sizeof(char)*strlen(HX_MP_FW));
	I("%s: start! FW file name(%s)\n", __func__, i_CTPM_firmware_name);
	g_core_fp.fp_0f_operation_dirly();
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	g_core_fp.fp_read_FW_ver();
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#else
	g_core_fp.fp_sense_on(0x00);
#endif
/*==========================Update FW End==========================================*/

	private_ts->in_self_test = 1;
	himax_int_enable(0);
	ret = g_core_fp.fp_chip_self_test(s, v);
	himax_int_enable(1);


#ifdef HX_ESD_RECOVERY
	HX_ESD_RESET_ACTIVATE = 1;
#endif
	//himax_int_enable(1);/* enable irq */

	private_ts->in_self_test = 0;
/*==========================Update FW Start==========================================*/
	I("NOW Running Zero flash update!\n");
	memset(i_CTPM_firmware_name, 0x00, sizeof(i_CTPM_firmware_name));
	memcpy(i_CTPM_firmware_name, HX_NORMAL_FW, sizeof(char)*strlen(HX_NORMAL_FW));
	I("%s: end! FW file name(%s)\n", __func__, i_CTPM_firmware_name);
	g_core_fp.fp_0f_operation_dirly();
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	g_core_fp.fp_read_FW_ver();
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(true, false);
#else
	g_core_fp.fp_sense_on(0x00);
#endif
	himax_int_enable(1);
/*==========================Update FW End==========================================*/

	return ret;

}

int bbk_himax_get_header_file_version(int which) /*#11*/
{

	int err = NO_ERR;
	int i_FW_VER = 0;
	int i_CFG_VER = 0;
	const struct firmware *fw_entry = NULL;
	static struct firmware array_info;
	int fw_size = 0;
	int flag_boot_h = 0;

	I("%s, Entering\n", __func__);
	memset(i_CTPM_firmware_name, 0x00, sizeof(i_CTPM_firmware_name));
	memcpy(i_CTPM_firmware_name, HX_NORMAL_FW, sizeof(char)*strlen(HX_NORMAL_FW));
	I("file name = %s\n", i_CTPM_firmware_name);
	
	err = request_firmware(&fw_entry,  i_CTPM_firmware_name, private_ts->dev);	
	if (err < 0) {
		VTI("VTS_FW_TYPE_FW = %d",VTS_FW_TYPE_FW);
		array_info.data = vts_fw_data_get(private_ts->vtsdev, VTS_FW_TYPE_FW, &fw_size);
		array_info.size = fw_size;
		release_firmware(fw_entry);
		fw_entry = &array_info;
		flag_boot_h = 1;
		I("%s, in line %d code = %d,bin file not load\n", __func__, __LINE__, err);
	}
	

	i_FW_VER = (fw_entry->data[FW_VER_MAJ_FLASH_ADDR] << 8) | fw_entry->data[FW_VER_MIN_FLASH_ADDR];
	i_CFG_VER = (fw_entry->data[CFG_VER_MAJ_FLASH_ADDR] << 8) | fw_entry->data[CFG_VER_MIN_FLASH_ADDR];

	I("i_FW_VER = %x\n", i_FW_VER);
	I("i_CFG_VER = %x\n", i_CFG_VER);

	if (0 == flag_boot_h) {
		release_firmware(fw_entry);
	}
	I("%s, END\n", __func__);
	return (which == 1) ? i_FW_VER : i_CFG_VER;
}

/* VIVO end */
static int himax_self_test_seq_read(struct seq_file *s, void *v)
{
	int val = 0x00;
	size_t ret = 0;
	I("%s: enter, %d\n", __func__, __LINE__);
	if (!HX_PROC_SEND_FLAG) {		
		HX_PROC_SEND_FLAG = 1;		
		val = bbk_himax_sensor_test(s, v);
		HX_PROC_SEND_FLAG = 0;	
	}
	
	return ret;
}

static void *himax_self_test_seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= 1)
		return NULL;
	return (void *)((unsigned long) *pos + 1);
}

static void *himax_self_test_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	return NULL;
}

static void himax_self_test_seq_stop(struct seq_file *s, void *v)
{
}


static const struct seq_operations himax_self_test_seq_ops = {
	.start	= himax_self_test_seq_start,
	.next	= himax_self_test_seq_next,
	.stop	= himax_self_test_seq_stop,
	.show	= himax_self_test_seq_read,
};

static int himax_self_test_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &himax_self_test_seq_ops);
};


static int himax_release (struct inode *inode, struct file *file)
{
	int ret = 0;

	I("%s\n", __func__);
	ret = seq_release(inode, file);

	return ret;
}

#ifdef HX_HIGH_SENSE
static ssize_t himax_HSEN_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->HSEN_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_HSEN_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->HSEN_enable = 0;
	else if (buf[0] == '1')
		ts->HSEN_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_HSEN_enable(ts->HSEN_enable, ts->suspended);
	I("%s: HSEN_enable = %d.\n", __func__, ts->HSEN_enable);
	return len;
}
#endif

#ifdef HX_SMART_WAKEUP
static ssize_t himax_SMWP_read(struct file *file, char *buf,
							   size_t len, loff_t *pos)
{
	size_t count = 0;
	struct himax_ts_data *ts = private_ts;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);
		count = snprintf(temp_buf, PAGE_SIZE, "%d\n", ts->SMWP_enable);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
	}

	return count;
}

static ssize_t himax_SMWP_write(struct file *file, const char *buff,
								size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	if (buf[0] == '0')
		ts->SMWP_enable = 0;
	else if (buf[0] == '1')
		ts->SMWP_enable = 1;
	else
		return -EINVAL;

	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, ts->suspended);
	HX_SMWP_EN = ts->SMWP_enable;
	I("%s: SMART_WAKEUP_enable = %d.\n", __func__, HX_SMWP_EN);
	return len;
}

static ssize_t himax_GESTURE_read(struct file *file, char *buf,
								  size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	size_t ret = 0;
	char *temp_buf;

	if (!HX_PROC_SEND_FLAG) {
		temp_buf = kzalloc(len, GFP_KERNEL);

		for (i = 0; i < GEST_SUP_NUM; i++)
			ret += snprintf(temp_buf + ret, len - ret, "ges_en[%d]=%d\n", i, ts->gesture_cust_en[i]);

		if (copy_to_user(buf, temp_buf, len))
			I("%s,here:%d\n", __func__, __LINE__);

		kfree(temp_buf);
		HX_PROC_SEND_FLAG = 1;
	} else {
		HX_PROC_SEND_FLAG = 0;
		ret = 0;
	}

	return ret;
}

static ssize_t himax_GESTURE_write(struct file *file, const char *buff,
								   size_t len, loff_t *pos)
{
	struct himax_ts_data *ts = private_ts;
	int i = 0;
	int j = 0;
	char buf[80] = {0};

	if (len >= 80) {
		I("%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(buf, buff, len))
		return -EFAULT;

	I("himax_GESTURE_store= %s, len = %d\n", buf, (int)len);

	for (i = 0; i < len; i++) {
		if (buf[i] == '0' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 0;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		}	else if (buf[i] == '1' && j < GEST_SUP_NUM) {
			ts->gesture_cust_en[j] = 1;
			I("gesture en[%d]=%d\n", j, ts->gesture_cust_en[j]);
			j++;
		}	else
			I("Not 0/1 or >=GEST_SUP_NUM : buf[%d] = %c\n", i, buf[i]);
	}

	return len;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static struct file_operations himax_proc_self_test_ops = {
	.owner = THIS_MODULE,
	.open = himax_self_test_proc_open,
	.read = seq_read,
	.release = himax_release,
};

#ifdef HX_HIGH_SENSE
static struct file_operations himax_proc_HSEN_ops = {
	.owner = THIS_MODULE,
	.read = himax_HSEN_read,
	.write = himax_HSEN_write,
};
#endif

#ifdef HX_SMART_WAKEUP
static struct file_operations himax_proc_SMWP_ops = {
	.owner = THIS_MODULE,
	.read = himax_SMWP_read,
	.write = himax_SMWP_write,
};
static struct file_operations himax_proc_Gesture_ops = {
	.owner = THIS_MODULE,
	.read = himax_GESTURE_read,
	.write = himax_GESTURE_write,
};
#endif
#else /*kernel-5.10*/
static struct proc_ops himax_proc_self_test_ops = {
	.proc_open = himax_self_test_proc_open,
	.proc_read = seq_read,
	.proc_release = himax_release,
};

#ifdef HX_HIGH_SENSE
static struct proc_ops himax_proc_HSEN_ops = {
	.proc_read = himax_HSEN_read,
	.proc_write = himax_HSEN_write,
};
#endif

#ifdef HX_SMART_WAKEUP
static struct proc_ops himax_proc_SMWP_ops = {
	.proc_read = himax_SMWP_read,
	.proc_write = himax_SMWP_write,
};

static struct proc_ops himax_proc_Gesture_ops = {
	.proc_read = himax_GESTURE_read,
	.proc_write = himax_GESTURE_write,
};
#endif
#endif

extern void (*fp_himax_self_test_init)(void);


int himax_common_proc_init(void)
{
	himax_touch_proc_dir = proc_mkdir(HIMAX_PROC_TOUCH_FOLDER, NULL);

	if (himax_touch_proc_dir == NULL) {
		E(" %s: himax_touch_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
	if (fp_himax_self_test_init != NULL)
		fp_himax_self_test_init();

	himax_proc_self_test_file = proc_create(HIMAX_PROC_SELF_TEST_FILE, (S_IRUGO), himax_touch_proc_dir, &himax_proc_self_test_ops);
	if (himax_proc_self_test_file == NULL) {
		E(" %s: proc self_test file create failed!\n", __func__);
		goto fail_1;
	}

#ifdef HX_HIGH_SENSE
	himax_proc_HSEN_file = proc_create(HIMAX_PROC_HSEN_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_HSEN_ops);

	if (himax_proc_HSEN_file == NULL) {
		E(" %s: proc HSEN file create failed!\n", __func__);
		goto fail_2;
	}

#endif
#ifdef HX_SMART_WAKEUP
	himax_proc_SMWP_file = proc_create(HIMAX_PROC_SMWP_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
									   himax_touch_proc_dir, &himax_proc_SMWP_ops);

	if (himax_proc_SMWP_file == NULL) {
		E(" %s: proc SMWP file create failed!\n", __func__);
		goto fail_3;
	}

	himax_proc_GESTURE_file = proc_create(HIMAX_PROC_GESTURE_FILE, (S_IWUSR | S_IRUGO | S_IWUGO),
										  himax_touch_proc_dir, &himax_proc_Gesture_ops);

	if (himax_proc_GESTURE_file == NULL) {
		E(" %s: proc GESTURE file create failed!\n", __func__);
		goto fail_4;
	}
#endif
	return 0;

fail_4:
#ifdef HX_SMART_WAKEUP
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
#endif
fail_3:
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
fail_2:
	remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
#endif
fail_1:
	return -ENOMEM;
}

void himax_common_proc_deinit(void)
{
	remove_proc_entry(HIMAX_PROC_SELF_TEST_FILE, himax_touch_proc_dir);
#ifdef HX_SMART_WAKEUP
	remove_proc_entry(HIMAX_PROC_GESTURE_FILE, himax_touch_proc_dir);
	remove_proc_entry(HIMAX_PROC_SMWP_FILE, himax_touch_proc_dir);
#endif
#ifdef HX_HIGH_SENSE
	remove_proc_entry(HIMAX_PROC_HSEN_FILE, himax_touch_proc_dir);
#endif
}

/* File node for SMWP and HSEN - End*/

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;
#if defined(HX_SMART_WAKEUP)
	int i = 0;
#endif
	ret = himax_dev_set(ts);

	if (ret < 0)
		goto input_device_fail;

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

#if defined(HX_SMART_WAKEUP)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP)
	for (i = 1; i < GEST_SUP_NUM; i++)
		set_bit(gest_key_def[i], ts->input_dev->keybit);
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#ifdef	HX_PROTOCOL_A
	/*ts->input_dev->mtsize = ts->nFinger_support;*/
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 3, 0, 0);
#else
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
#if defined(HX_PROTOCOL_B_3PA)
	input_mt_init_slots(ts->input_dev, ts->nFinger_support, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(ts->input_dev, ts->nFinger_support);
#endif
#endif
	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
	  ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
#ifndef	HX_PROTOCOL_A
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	/*input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);*/
#endif
/*	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);*/

	if (himax_input_register_device(ts->input_dev) == 0)
		return NO_ERR;
	else
		ret = INPUT_REGISTER_FAIL;


input_device_fail:
	I("%s, input device register fail!\n", __func__);
	return ret;
}

static void calcDataSize(void)
{
	struct himax_ts_data *ts_data = private_ts;

	ts_data->x_channel = ic_data->HX_RX_NUM;
	ts_data->y_channel = ic_data->HX_TX_NUM;
	ts_data->nFinger_support = ic_data->HX_MAX_PT;

	ts_data->coord_data_size = 4 * ts_data->nFinger_support;
	ts_data->area_data_size = ((ts_data->nFinger_support / 4) + (ts_data->nFinger_support % 4 ? 1 : 0)) * 4;
	ts_data->coordInfoSize = ts_data->coord_data_size + ts_data->area_data_size + 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;

	if (ts_data->raw_data_frame_size == 0) {
		E("%s: could NOT calculate!\n", __func__);
		return;
	}

	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
									(((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size) ? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d\n", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4;

	if ((ic_data->HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4;
	else
		HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) + 1) * 4;
}

#ifdef HX_ESD_RECOVERY
static void himax_esd_hw_reset(void)
{
	if (g_ts_dbg != 0)
		I("%s: Entering\n", __func__);

	I("START_Himax TP: ESD - Reset\n");

	if (private_ts->in_self_test == 1) {
		I("In self test , not  TP: ESD - Reset\n");
		return;
	}

	g_core_fp.fp_esd_ic_reset();
#ifdef HX_ZERO_FLASH
	I("It will update fw after esd event in zero flash mode!\n");
	g_core_fp.fp_0f_operation_dirly();
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	g_core_fp.fp_sense_on(0x00);
	g_core_fp.fp_check_remapping();
	himax_report_all_leave_event(private_ts);
	himax_int_enable(1);
#endif
}
#endif

#ifdef HX_SMART_WAKEUP
#ifdef HX_GESTURE_TRACK
static void gest_pt_log_coordinate(int rx, int tx)
{
	/*driver report x y with range 0 - 255 , we scale it up to x/y pixel*/
	gest_pt_x[gest_pt_cnt] = rx * (ic_data->HX_X_RES) / 255;
	gest_pt_y[gest_pt_cnt] = tx * (ic_data->HX_Y_RES) / 255;
}
#endif

static int himax_wake_event_parse(struct himax_ts_data *ts, int ts_status)
{
#ifdef HX_GESTURE_TRACK
	int tmp_max_x = 0x00, tmp_min_x = 0xFFFF, tmp_max_y = 0x00, tmp_min_y = 0xFFFF;
	int gest_len = 0;
#endif
	int i = 0, check_FC = 0;
	int j = 0, gesture_pos = 0, gesture_flag = 0;

	memset(gest_pt_x, 0x00, sizeof(gest_pt_x));
	memset(gest_pt_y, 0x00, sizeof(gest_pt_y));

	if (g_ts_dbg != 0)
		I("%s: Entering!, ts_status=%d\n", __func__, ts_status);

	if(!event_buf){
		event_buf = kzalloc(hx_touch_data->event_size * sizeof(uint8_t), GFP_KERNEL);
		if (event_buf == NULL) {
			return -ENOMEM;
		}
	}
	memcpy(event_buf, hx_touch_data->hx_event_buf, hx_touch_data->event_size);

	for (i = 0; i < GEST_PTLG_ID_LEN; i++) {
		for (j = 0; j < GEST_SUP_NUM; j++) {
			if (event_buf[i] == gest_event[j]) {
				gesture_flag = event_buf[i];
				gesture_pos = j;
				break;
			}
		}
		I("0x%2.2X ", event_buf[i]);
		if (event_buf[i] == gesture_flag) {
			check_FC++;
		} else {
			I("ID START at %x , value = 0x%2X skip the event\n", i, event_buf[i]);
			break;
		}
	}

	I("Himax gesture_flag= %x\n", gesture_flag);
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC != GEST_PTLG_ID_LEN) {
		kfree(event_buf);
		event_buf = NULL;
		return 0;
	}

	if (event_buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
		event_buf[GEST_PTLG_ID_LEN + 1] != GEST_PTLG_HDR_ID2) {
		kfree(event_buf);
		event_buf = NULL;
		return 0;
	}

#ifdef HX_GESTURE_TRACK

	if (event_buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
		event_buf[GEST_PTLG_ID_LEN + 1] == GEST_PTLG_HDR_ID2) {
		gest_len = event_buf[GEST_PTLG_ID_LEN + 2];
		I("gest_len = %d\n", gest_len);
		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start\n %s", __func__);

		if (gesture_flag == 8) {
			gest_len = 19;
		}

		while (i < (gest_len + 1) / 2) {
			gest_pt_log_coordinate(event_buf[GEST_PTLG_ID_LEN + 4 + i * 2], event_buf[GEST_PTLG_ID_LEN + 4 + i * 2 + 1]);
			i++;
			I("gest_pt_x[%d]=%d\n", gest_pt_cnt, gest_pt_x[gest_pt_cnt]);
			I("gest_pt_y[%d]=%d\n", gest_pt_cnt, gest_pt_y[gest_pt_cnt]);
			gest_pt_cnt += 1;
		}

		if (gesture_flag == 8) {
			if (gest_pt_x[6] == 0) {   //the sixth byte is direction
				gest_pt_y[9] = 32;
			}
			if (gest_pt_x[6] == 4) {
				gest_pt_y[9] = 16;
			}
			
			for (i = 6; i < 9; i ++) {
				gest_pt_x[i] = 0xFFFF;
				gest_pt_y[i] = 0xFFFF;
			}
			gest_pt_x[i] = 0xFFFF;
		}

		if (gest_pt_cnt) {
			for (i = 0; i < gest_pt_cnt; i++) {
				if (tmp_max_x < gest_pt_x[i])
					tmp_max_x = gest_pt_x[i];
				if (tmp_min_x > gest_pt_x[i])
					tmp_min_x = gest_pt_x[i];
				if (tmp_max_y < gest_pt_y[i])
					tmp_max_y = gest_pt_y[i];
				if (tmp_min_y > gest_pt_y[i])
					tmp_min_y = gest_pt_y[i];
			}

			I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n", tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y);
			gest_start_x = gest_pt_x[0];
			hx_gesture_coor[0] = gest_start_x;
			gest_start_y = gest_pt_y[0];
			hx_gesture_coor[1] = gest_start_y;
			gest_end_x = gest_pt_x[gest_pt_cnt - 1];
			hx_gesture_coor[2] = gest_end_x;
			gest_end_y = gest_pt_y[gest_pt_cnt - 1];
			hx_gesture_coor[3] = gest_end_y;
			gest_width = tmp_max_x - tmp_min_x;
			hx_gesture_coor[4] = gest_width;
			gest_height = tmp_max_y - tmp_min_y;
			hx_gesture_coor[5] = gest_height;
			gest_mid_x = (tmp_max_x + tmp_min_x) / 2;
			hx_gesture_coor[6] = gest_mid_x;
			gest_mid_y = (tmp_max_y + tmp_min_y) / 2;
			hx_gesture_coor[7] = gest_mid_y;
			hx_gesture_coor[8] = gest_mid_x;/*gest_up_x*/
			hx_gesture_coor[9] = gest_mid_y - gest_height / 2; /*gest_up_y*/
			hx_gesture_coor[10] = gest_mid_x;/*gest_down_x*/
			hx_gesture_coor[11] = gest_mid_y + gest_height / 2;	/*gest_down_y*/
			hx_gesture_coor[12] = gest_mid_x - gest_width / 2;	/*gest_left_x*/
			hx_gesture_coor[13] = gest_mid_y;	/*gest_left_y*/
			hx_gesture_coor[14] = gest_mid_x + gest_width / 2;	/*gest_right_x*/
			hx_gesture_coor[15] = gest_mid_y; /*gest_right_y*/
		}
	}

#endif

	g_target_report_data->SMWP_event_chk = gest_key_def[gesture_pos];
	return gesture_pos;
}

static void himax_wake_event_report(void)
{
	int KEY_EVENT = g_target_report_data->SMWP_event_chk;
	uint32_t keycode = 0;

	if (g_ts_dbg != 0)
		I("%s: Entering!\n", __func__);
	switch (KEY_EVENT) {
		case KEY_C:
			VTI("Gesture : Word-C.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_C;
			break;
		case KEY_W:
			VTI("Gesture : Word-W.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_W;
			break;
		case KEY_WAKEUP:
			VTI("Gesture : Double Click.\n");
			keycode = VTS_EVENT_GESTURE_DOUBLE_CLICK;
			break;
		case KEY_M:
			VTI("Gesture : Word-M.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_M;
			break;
		case KEY_O:
			VTI("Gesture : Word-O.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_O;
			break;
		case KEY_E:
			VTI("Gesture : Word-e.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_E;
			break;
		case KEY_UP:
			VTI("Gesture : Slide UP.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_UP;
			break;
		case KEY_WAKEUP_SWIPE:
			VTI("Gesture : Slide DOWN.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_DOWN;
			break;
		case KEY_LEFT:
			VTI("Gesture : Slide LEFT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_LEFT;
			break;
		case KEY_RIGHT:
			VTI("Gesture : Slide RIGHT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_RIGHT;
			break;
		case KEY_A:
			VTI("Gesture : Custom_Word-AT.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_A;
			break;
		case KEY_F:
			VTI("Gesture : Custom_Word-F.\n");
			keycode = VTS_EVENT_GESTURE_PATTERN_F;
			break;
		default:
			VTI("not used gesture event");
			keycode = 0;
			break;
	}

	if (keycode>0) {
		VTI(" %s SMART WAKEUP KEY event %d press\n", __func__, KEY_EVENT);
		//input_report_key(private_ts->input_dev, KEY_EVENT, 1);
		//input_sync(private_ts->input_dev);
		//VTI(" %s SMART WAKEUP KEY event %d release\n", __func__, KEY_EVENT);
		//input_report_key(private_ts->input_dev, KEY_EVENT, 0);
		//input_sync(private_ts->input_dev);
		vts_report_event_down(private_ts->vtsdev,keycode);
		vts_report_event_up(private_ts->vtsdev, keycode);
#ifdef HX_GESTURE_TRACK
		VTI("gest_start_x= %d, gest_start_y= %d, gest_end_x= %d, gest_end_y= %d\n", gest_start_x, gest_start_y,
		  gest_end_x, gest_end_y);
		VTI("gest_width= %d, gest_height= %d, gest_mid_x= %d, gest_mid_y= %d\n", gest_width, gest_height,
		  gest_mid_x, gest_mid_y);
		VTI("gest_up_x= %d, gest_up_y= %d, gest_down_x= %d, gest_down_y= %d\n", hx_gesture_coor[8], hx_gesture_coor[9],
		  hx_gesture_coor[10], hx_gesture_coor[11]);
		VTI("gest_left_x= %d, gest_left_y= %d, gest_right_x= %d, gest_right_y= %d\n", hx_gesture_coor[12], hx_gesture_coor[13],
		  hx_gesture_coor[14], hx_gesture_coor[15]);

		vts_report_coordinates_set(private_ts->vtsdev, gest_pt_x, gest_pt_y, gest_pt_cnt);
#endif
		g_target_report_data->SMWP_event_chk = 0;
	}
}

#endif

int himax_report_data_init(void)
{
	if (hx_touch_data->hx_coord_buf != NULL){
		kfree(hx_touch_data->hx_coord_buf);
		hx_touch_data->hx_coord_buf = NULL;
		}

	if (hx_touch_data->hx_rawdata_buf != NULL){
		kfree(hx_touch_data->hx_rawdata_buf);
		hx_touch_data->hx_rawdata_buf = NULL;
		}
#if defined(HX_SMART_WAKEUP)
	hx_touch_data->event_size = g_core_fp.fp_get_touch_data_size();

	if (hx_touch_data->hx_event_buf != NULL){
		kfree(hx_touch_data->hx_event_buf);
		hx_touch_data->hx_event_buf = NULL;
		}

#endif
	hx_touch_data->touch_all_size = g_core_fp.fp_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT / 4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT % 4;
	/* more than 4 fingers */
	if (hx_touch_data->raw_cnt_rmd != 0x00) {
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 2) * 4;
	} else { /* less than 4 fingers */
		hx_touch_data->rawdata_size = g_core_fp.fp_cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT + hx_touch_data->raw_cnt_max + 1) * 4;
	}

	if ((ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) % hx_touch_data->rawdata_size == 0)
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size;
	else
		hx_touch_data->rawdata_frame_size = (ic_data->HX_TX_NUM * ic_data->HX_RX_NUM + ic_data->HX_TX_NUM + ic_data->HX_RX_NUM) / hx_touch_data->rawdata_size + 1;


	I("%s: rawdata_frame_size = %d\n", __func__, hx_touch_data->rawdata_frame_size);
	I("%s: ic_data->HX_MAX_PT:%d, hx_raw_cnt_max:%d, hx_raw_cnt_rmd:%d, g_hx_rawdata_size:%d, hx_touch_data->touch_info_size:%d\n", __func__, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max, hx_touch_data->raw_cnt_rmd, hx_touch_data->rawdata_size, hx_touch_data->touch_info_size);
	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_coord_buf == NULL)
		goto mem_alloc_fail;

	if (g_target_report_data == NULL) {
		g_target_report_data = kzalloc(sizeof(struct himax_target_report_data), GFP_KERNEL);
		if (g_target_report_data == NULL)
			goto mem_alloc_fail;
		g_target_report_data->x = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->x == NULL)
			goto mem_alloc_fail;
		g_target_report_data->y = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->y == NULL)
			goto mem_alloc_fail;
		g_target_report_data->w = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->w == NULL)
			goto mem_alloc_fail;
		g_target_report_data->finger_id = kzalloc(sizeof(int)*(ic_data->HX_MAX_PT), GFP_KERNEL);
		if (g_target_report_data->finger_id == NULL)
			goto mem_alloc_fail;
	}
#ifdef HX_SMART_WAKEUP
	g_target_report_data->SMWP_event_chk = 0;
#endif

	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size), GFP_KERNEL);

	if (hx_touch_data->hx_rawdata_buf == NULL)
		goto mem_alloc_fail;


#if defined(HX_SMART_WAKEUP)
	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t) * (hx_touch_data->event_size), GFP_KERNEL);

	if (hx_touch_data->hx_event_buf == NULL)
		goto mem_alloc_fail;
#endif
	return NO_ERR;
mem_alloc_fail:
	kfree(g_target_report_data->x);
	g_target_report_data->x = NULL;
	kfree(g_target_report_data->y);
	g_target_report_data->y = NULL;
	kfree(g_target_report_data->w);
	g_target_report_data->w = NULL;
	kfree(g_target_report_data->finger_id);
	g_target_report_data->finger_id = NULL;
	kfree(g_target_report_data);
	g_target_report_data = NULL;
	kfree(hx_touch_data->hx_coord_buf);
	hx_touch_data->hx_coord_buf = NULL;
	kfree(hx_touch_data->hx_rawdata_buf);
	hx_touch_data->hx_rawdata_buf = NULL;
#if defined(HX_SMART_WAKEUP)
	kfree(hx_touch_data->hx_event_buf);
#endif
	I("%s: Memory allocate fail!\n", __func__);
	return MEM_ALLOC_FAIL;
}

/*start ts_work*/
static int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD;

	hx_touch_data->diag_cmd = ts->diag_cmd;
	if (hx_touch_data->diag_cmd)
		result = HX_REPORT_COORD_RAWDATA;

#ifdef HX_SMART_WAKEUP
	if (atomic_read(&ts->suspend_mode) && (ts->SMWP_enable) && (!hx_touch_data->diag_cmd))
		result = HX_REPORT_SMWP_EVENT;
#endif
	/* I("Now Status is %d\n", result); */
	return result;
}

static int himax_touch_get(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	switch (ts_path) {
	/*normal*/
	case HX_REPORT_COORD:
		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
			|| (HX_ESD_RESET_ACTIVATE)
#endif
			) {
			if (!g_core_fp.fp_read_event_stack(buf, 128)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
				goto END_FUNCTION;
			}
			break;
		} else {
			if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->touch_info_size)) {
				E("%s: can't read data from chip!\n", __func__);
				ts_status = HX_TS_GET_DATA_FAIL;
				goto END_FUNCTION;
			}
			break;
		}
#if defined(HX_SMART_WAKEUP)

	/*SMWP*/
	case HX_REPORT_SMWP_EVENT:
		g_core_fp.fp_burst_enable(0);

		if (!g_core_fp.fp_read_event_stack(buf, hx_touch_data->event_size)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		if (!g_core_fp.fp_read_event_stack(buf, 128)) {
			E("%s: can't read data from chip!\n", __func__);
			ts_status = HX_TS_GET_DATA_FAIL;
			goto END_FUNCTION;
		}
		break;
	default:
		break;
	}

END_FUNCTION:
	return ts_status;
}

/* start error_control*/
static int himax_checksum_cal(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	uint16_t check_sum_cal = 0;
	int32_t	i = 0;
	int length = 0;
	int zero_cnt = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	for (i = 0; i < length; i++) {
		check_sum_cal += buf[i];
		if (buf[i] == 0x00)
			zero_cnt++;
	}

	if (check_sum_cal % 0x100 != 0) {
		I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		ret_val = HX_CHKSUM_FAIL;
	} else if (zero_cnt == length) {
		I("[HIMAX TP MSG] All Zero event\n");
		ret_val = HX_CHKSUM_FAIL;
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);
	return ret_val;
}

#ifdef HX_ESD_RECOVERY
#ifdef HX_ZERO_FLASH
void hx_update_dirly_0f(void)
{
	I("It will update fw after esd event in zero flash mode!\n");
	g_core_fp.fp_0f_operation_dirly();
}
#endif
static int himax_ts_event_check(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
	int hx_EB_event = 0;
	int hx_EC_event = 0;
	int hx_ED_event = 0;
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;

	int32_t	loop_i = 0;
	int length = 0;
	int ret_val = ts_status;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	/* Normal */
	switch (ts_path) {
	case HX_REPORT_COORD:
		length = hx_touch_data->touch_info_size;
		break;
#if defined(HX_SMART_WAKEUP)
/* SMWP */
	case HX_REPORT_SMWP_EVENT:
		length = (GEST_PTLG_ID_LEN + GEST_PTLG_HDR_LEN);
		break;
#endif
	case HX_REPORT_COORD_RAWDATA:
		length = hx_touch_data->touch_info_size;
		break;
	default:
		I("%s, Neither Normal Nor SMWP error!\n", __func__);
		ret_val = HX_PATH_FAIL;
		goto END_FUNCTION;
	}

	if (g_ts_dbg != 0)
		I("Now Path=%d, Now status=%d, length=%d\n", ts_path, ts_status, length);

	for (loop_i = 0; loop_i < length; loop_i++) {
		if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xEB) {
				hx_EB_event++;
			} else if (buf[loop_i] == 0xEC) {
				hx_EC_event++;
			} else if (buf[loop_i] == 0xED) {
				hx_ED_event++;
			} else if (buf[loop_i] == 0x00) { /* case 2 ESD recovery flow-Disable */
				hx_zero_event++;
			} else {
				hx_EB_event = 0;
				hx_EC_event = 0;
				hx_ED_event = 0;
				hx_zero_event = 0;
				g_zero_event_count = 0;
			}
		}
	}

	if (hx_EB_event == length) {
		hx_esd_event = length;
		hx_EB_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEB.\n");
	} else if (hx_EC_event == length) {
		hx_esd_event = length;
		hx_EC_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xEC.\n");
	} else if (hx_ED_event == length) {
		hx_esd_event = length;
		hx_ED_event_flag++;
		I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
	}
#ifdef HX_ZERO_FLASH
		else if (hx_zero_event == length) {
		/*check zero flash status*/
		if (g_core_fp.fp_0f_esd_check() < 0) {
			g_zero_event_count = 6;
			I("[HIMAX TP MSG]: ESD event checked - ALL Zero in ZF.\n");
		} else {
			I("[HIMAX TP MSG]: Status check pass in ZF.\n");
		}
	}
#endif
	else
		hx_esd_event = 0;
	if ((hx_esd_event == length || hx_zero_event == length)
		&& (HX_HW_RESET_ACTIVATE == 0)
		&& (HX_ESD_RESET_ACTIVATE == 0)
		&& (hx_touch_data->diag_cmd == 0)
		&& (ts->in_self_test == 0)) {
		shaking_ret = g_core_fp.fp_ic_esd_recovery(hx_esd_event, hx_zero_event, length);

		if (shaking_ret == HX_ESD_EVENT) {
			himax_esd_hw_reset();
			ret_val = HX_ESD_EVENT;
		} else if (shaking_ret == HX_ZERO_EVENT_COUNT) {
			ret_val = HX_ZERO_EVENT_COUNT;
		} else {
			I("MCU running. Nothing to be done!\n");
			ret_val = HX_IC_RUNNING;
		}
	} else if (HX_ESD_RESET_ACTIVATE) { /* drop 1st interrupts after chip reset */
		HX_ESD_RESET_ACTIVATE = 0;
		I("[HX_ESD_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ret_val = HX_ESD_REC_OK;
	}

END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ret_val=%d!\n", __func__, ret_val);

	return ret_val;
}
#endif

static int himax_err_ctrl(struct himax_ts_data *ts, uint8_t *buf, int ts_path, int ts_status)
{
#ifdef HX_RST_PIN_FUNC
	if (HX_HW_RESET_ACTIVATE) {
		/* drop 1st interrupts after chip reset */
		HX_HW_RESET_ACTIVATE = 0;
		I("[HX_HW_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
		ts_status = HX_RST_OK;
		goto END_FUNCTION;
	}
#endif

	ts_status = himax_checksum_cal(ts, buf, ts_path, ts_status);
	if (ts_status == HX_CHKSUM_FAIL) {
		goto CHK_FAIL;
	} else {
#ifdef HX_ESD_RECOVERY
		g_zero_event_count = 0;
#endif
		goto END_FUNCTION;
	}

CHK_FAIL:
#ifdef HX_ESD_RECOVERY
	ts_status = himax_ts_event_check(ts, buf, ts_path, ts_status);
#endif


END_FUNCTION:
	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end error_control*/

/* start distribute_data*/
static int himax_distribute_touch_data(uint8_t *buf, int ts_path, int ts_status)
{
	uint8_t hx_state_info_pos = hx_touch_data->touch_info_size - 3;

	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);
		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF) {
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		} else {
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));
		}

		if ((HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
		|| (HX_ESD_RESET_ACTIVATE)
#endif
		) {
			memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
		}
	} else if (ts_path == HX_REPORT_COORD_RAWDATA) {
		memcpy(hx_touch_data->hx_coord_buf, &buf[0], hx_touch_data->touch_info_size);

		if (buf[hx_state_info_pos] != 0xFF && buf[hx_state_info_pos + 1] != 0xFF)
			memcpy(hx_touch_data->hx_state_info, &buf[hx_state_info_pos], 2);
		else
			memset(hx_touch_data->hx_state_info, 0x00, sizeof(hx_touch_data->hx_state_info));


		memcpy(hx_touch_data->hx_rawdata_buf, &buf[hx_touch_data->touch_info_size], hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
#if defined(HX_SMART_WAKEUP)
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		memcpy(hx_touch_data->hx_event_buf, buf, hx_touch_data->event_size);
#endif
	} else {
		E("%s, Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: End, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end assign_data*/
static void himax_bak_leave(struct himax_ts_data *ts,int loop_i, ktime_t kt)
{
	vts_report_point_up(ts->vtsdev, loop_i,--ts->finger_cntbk,ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1],0,0, false, kt);
	//vts_report_point_sync(ts->vtsdev);

	return;
}

/* start parse_report_data*/
int himax_parse_report_points(struct himax_ts_data *ts, int ts_path, int ts_status, ktime_t kt)
{
	int x = 0;
	int y = 0;
	int w = 0;
	int base = 0;
	int32_t	loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	ts->old_finger = ts->pre_finger_mask;
	if (ts->hx_point_num == 0){
		return ts_status;
	}
	ts->pre_finger_mask = 0;
	hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
	hx_touch_data->finger_on = 1;
	AA_press = 1;

	g_target_report_data->finger_num = hx_touch_data->finger_num;
	g_target_report_data->finger_on = hx_touch_data->finger_on;

	if (g_ts_dbg != 0)
		I("%s:finger_num = 0x%2X, finger_on = %d\n", __func__, g_target_report_data->finger_num, g_target_report_data->finger_on);

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		base = loop_i * 4;
		x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
		y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
		w = hx_touch_data->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];

		if (g_ts_dbg != 0)
			D("%s: now parsing[%d]:x=%d, y=%d, w=%d\n", __func__, loop_i, x, y, w);

		if (x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 && y <= ts->pdata->abs_y_max) {
			hx_touch_data->finger_num--;

			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 1;

			/* I("%s: g_target_report_data->x[loop_i]=%d, g_target_report_data->y[loop_i]=%d, g_target_report_data->w[loop_i]=%d", */
			/* __func__, g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->w[loop_i]); */


			if (!ts->first_pressed) {
				ts->first_pressed = 1;
				I("S1@%d, %d\n", x, y);
			}

			ts->pre_finger_data[loop_i][0] = x;
			ts->pre_finger_data[loop_i][1] = y;

			ts->pre_finger_mask = ts->pre_finger_mask + (1 << loop_i);
			ts->press_id[loop_i] = 1;

		} else {/* report coordinates */
			if(ts->press_id[loop_i] ==1) 
				himax_bak_leave(ts,loop_i,kt);

			ts->press_id[loop_i] =0;
			g_target_report_data->x[loop_i] = x;
			g_target_report_data->y[loop_i] = y;
			g_target_report_data->w[loop_i] = w;
			g_target_report_data->finger_id[loop_i] = 0;

			if (loop_i == 0 && ts->first_pressed == 1) {
				ts->first_pressed = 2;
				I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
			}
		}
	}

	if (g_ts_dbg != 0) {
		for (loop_i = 0; loop_i < 10; loop_i++)
			D("DBG X=%d  Y=%d ID=%d\n", g_target_report_data->x[loop_i], g_target_report_data->y[loop_i], g_target_report_data->finger_id[loop_i]);
		D("DBG finger number %d\n", g_target_report_data->finger_num);
	}

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
	return ts_status;
}

static int himax_parse_report_data(struct himax_ts_data *ts, int ts_path, int ts_status, ktime_t kt)
{
	int palm = 0;
	bool large_press = false;
	if (g_ts_dbg != 0)
		I("%s: start now_status=%d!\n", __func__, ts_status);


	EN_NoiseFilter = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 3);
	/* I("EN_NoiseFilter=%d\n", EN_NoiseFilter); */
	EN_NoiseFilter = EN_NoiseFilter & 0x01;
	/* I("EN_NoiseFilter2=%d\n", EN_NoiseFilter); */
	p_point_num = ts->hx_point_num;

	if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff) {
		ts->hx_point_num = 0;
	} else {
		ts->hx_point_num = hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
	}

	if(!ts->suspended) {
		palm = hx_touch_data->hx_state_info[0] >> 3 & 0x01;
		if (palm && (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f) == 0x00) {
			VTI("got PalmOn\n");
			//<VIVO can do action for Palm On> large press 
			large_press = true;
			vts_report_point_down(ts->vtsdev,0,0,0,0,0,0,true, NULL, 0, kt);
			return 0;
		} else if (!palm && (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f) == 0x00) {
			VTI("got PalmOff\n");
			large_press = false;
			vts_report_release(ts->vtsdev);
			vts_report_point_up(ts->vtsdev,0,0,0,0,0,0,true,kt);
			return 0;
		}
	}

	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_parse_report_points(ts, ts_path, ts_status, kt);
		break;
	case HX_REPORT_COORD_RAWDATA:
		/* touch monitor rawdata */
		if (debug_data != NULL) {
			if (debug_data->fp_set_diag_cmd(ic_data, hx_touch_data))
				I("%s: coordinate dump fail and bypass with checksum err\n", __func__);
		} else {
			E("%s,There is no init set_diag_cmd\n", __func__);
		}
		ts_status = himax_parse_report_points(ts, ts_path, ts_status, kt);
		break;
#ifdef HX_SMART_WAKEUP
	case HX_REPORT_SMWP_EVENT:
		himax_wake_event_parse(ts, ts_status);
		break;
#endif
	default:
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
		break;
	}
	if (g_ts_dbg != 0)
		I("%s: end now_status=%d!\n", __func__, ts_status);
	return ts_status;
}

/* end parse_report_data*/

static void himax_report_all_leave_event(struct himax_ts_data *ts)
{
#if 0
	int loop_i = 0;

	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
#ifndef	HX_PROTOCOL_A
		input_mt_slot(ts->input_dev, loop_i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		/*input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);*/
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
#endif
	}
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
#endif
	vts_report_release(ts->vtsdev);
vts_report_point_sync(ts->vtsdev);

}

/* start report_point*/
static void himax_finger_report(struct himax_ts_data *ts, ktime_t kt)
{
	int i = 0;
	bool valid = false;
	//uint32_t weight = 0;
	//int ret = 0;
	ts->finger_cnt=0;
	ts->finger_cntbk=0;
	if (g_ts_dbg != 0) {
		I("%s:start\n", __func__);
		I("hx_touch_data->finger_num=%d\n", hx_touch_data->finger_num);
	}

	for (i = 0; i < ts->nFinger_support; i++) {
		if (g_target_report_data->x[i] >= 0 && g_target_report_data->x[i] <= ts->pdata->abs_x_max && g_target_report_data->y[i] >= 0 && g_target_report_data->y[i] <= ts->pdata->abs_y_max)
			valid = true;
		else
			valid = false;
		if (g_ts_dbg != 0)
			I("valid=%d\n", valid);

		if (valid) {
			if (g_ts_dbg != 0)
				I("g_target_report_data->x[i]=%d, g_target_report_data->y[i]=%d, g_target_report_data->w[i]=%d\n", g_target_report_data->x[i], g_target_report_data->y[i], (g_target_report_data->w[i]/10));

		//weight = (uint32_t)g_target_report_data->w[i]/10;
		//weight |=  (hx_touch_data->hx_state_info[0] << 8) | (hx_touch_data->hx_state_info[1] << 16);

		ts->finger_cnt++;
		ts->finger_cntbk++;
		/*	vivoTsInputReport(VTS_TOUCH_DOWN, i, g_target_report_data->x[i], g_target_report_data->y[i], (g_target_report_data->w[i]/10));*/
		vts_report_point_down(ts->vtsdev, i,ts->finger_cnt , g_target_report_data->x[i],g_target_report_data->y[i],g_target_report_data->w[i],g_target_report_data->w[i], false, NULL, 0, kt);
		/*
		Noise : hx_state_info[0] >> 2 & 0x01;
		water : hx_state_info[0] >> 3 & 0x01;
		TX Hop: hx_state_info[0] >> 5 & 0x01;
		*/
#if 0
#ifndef	HX_PROTOCOL_A
			input_mt_slot(ts->input_dev, i);
#endif
			input_report_key(ts->input_dev, BTN_TOUCH, g_target_report_data->finger_on);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_target_report_data->w[i]);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
#ifndef	HX_PROTOCOL_A
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_target_report_data->w[i]);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_target_report_data->w[i]);
#endif
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_target_report_data->x[i]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_target_report_data->y[i]);
#ifndef	HX_PROTOCOL_A
			ts->last_slot = i;
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
#else
			input_mt_sync(ts->input_dev);
#endif
#endif

		}
	}
	
	vts_report_point_sync(ts->vtsdev);

	//input_report_key(ts->input_dev, BTN_TOUCH, g_target_report_data->finger_on);
	//input_sync(ts->input_dev);
	if (g_ts_dbg != 0)
		I("%s:end\n", __func__);
}

static void himax_finger_leave(struct himax_ts_data *ts, ktime_t kt)
{
	int32_t loop_i = 0;

	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);
	hx_touch_data->finger_on = 0;
	g_target_report_data->finger_on  = 0;
	g_target_report_data->finger_num = 0;
	AA_press = 0;	

#ifdef HX_PROTOCOL_A
	input_mt_sync(ts->input_dev);
#endif
	for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
		/*++09-26 Start++*/
		if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
		/*++09-26 End++*/
			 vts_report_point_up(ts->vtsdev, loop_i,--ts->finger_cntbk,g_target_report_data->x[loop_i],g_target_report_data->y[loop_i],0,0, false, kt);
			/*input_mt_slot(ts->input_dev, loop_i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);*/
		}
	}
	memset(ts->press_id, 0 ,sizeof(ts->press_id));
		ts->finger_cntbk=0;
	if (ts->pre_finger_mask > 0)
		ts->pre_finger_mask = 0;
	if (ts->first_pressed == 1) {
		ts->first_pressed = 2;
		I("E1@%d, %d\n", ts->pre_finger_data[0][0], ts->pre_finger_data[0][1]);
	}
#if 0
	if (ts->debug_log_level & BIT(1))
		himax_log_touch_event(x, y, w, loop_i, EN_NoiseFilter, HX_FINGER_LEAVE);

	input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
	input_sync(ts->input_dev);
#endif

	vts_report_point_sync(ts->vtsdev);

	vts_report_release(ts->vtsdev);

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
	return;
}

static void himax_report_points(struct himax_ts_data *ts, ktime_t kt)
{
	if (g_ts_dbg != 0)
		I("%s: start!\n", __func__);

	if (ts->hx_point_num != 0)
		himax_finger_report(ts, kt);
	else
		himax_finger_leave(ts, kt);

	Last_EN_NoiseFilter = EN_NoiseFilter;

	if (g_ts_dbg != 0)
		I("%s: end!\n", __func__);
}
/* end report_points*/

int himax_report_data(struct himax_ts_data *ts, int ts_path, int ts_status, ktime_t kt)
{
	if (g_ts_dbg != 0)
		I("%s: Entering, ts_status=%d!\n", __func__, ts_status);

	if (ts_path == HX_REPORT_COORD || ts_path == HX_REPORT_COORD_RAWDATA) {

		/* Touch Point information */
		himax_report_points(ts, kt);

#ifdef HX_SMART_WAKEUP
	} else if (ts_path == HX_REPORT_SMWP_EVENT) {
		himax_wake_event_report();
#endif
	} else {
		E("%s:Fail Path!\n", __func__);
		ts_status = HX_PATH_FAIL;
	}

	if (g_ts_dbg != 0)
		I("%s: END, ts_status=%d!\n", __func__, ts_status);
	return ts_status;
}
/* end report_data */

static int himax_ts_operation(struct himax_ts_data *ts, int ts_path, int ts_status, ktime_t kt)
{
	uint8_t hw_reset_check[2];
	//uint8_t *buf;
	//buf = kzalloc(128 * sizeof(uint8_t), GFP_KERNEL);

	memset(g_point_buf, 0x00, sizeof(uint8_t) * 128);
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	ts_status = himax_touch_get(ts, g_point_buf, ts_path, ts_status);
	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto END_FUNCTION;

	ts_status = himax_err_ctrl(ts, g_point_buf, ts_path, ts_status);
	if (ts_status == HX_REPORT_DATA || ts_status == HX_TS_NORMAL_END) {
		ts_status = himax_distribute_touch_data(g_point_buf, ts_path, ts_status);
		ts_status = himax_parse_report_data(ts, ts_path, ts_status, kt);
	} else {
		goto END_FUNCTION;
	}
	ts_status = himax_report_data(ts, ts_path, ts_status, kt);
END_FUNCTION:
	//kfree(buf);
	return ts_status;
}

void himax_ts_work(struct himax_ts_data *ts, ktime_t kt)
{
	int ts_status = HX_TS_NORMAL_END;
	int ts_path = 0;
	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_ON);
	ts_path = himax_ts_work_status(ts);
	switch (ts_path) {
	case HX_REPORT_COORD:
		ts_status = himax_ts_operation(ts, ts_path, ts_status, kt);
		break;
	case HX_REPORT_SMWP_EVENT:
		ts_status = himax_ts_operation(ts, ts_path, ts_status, kt);
		break;
	case HX_REPORT_COORD_RAWDATA:
		ts_status = himax_ts_operation(ts, ts_path, ts_status, kt);
		break;
	default:
		E("%s:Path Fault! value=%d\n", __func__, ts_path);
		goto END_FUNCTION;
		break;
	}

	if (ts_status == HX_TS_GET_DATA_FAIL)
		goto GET_TOUCH_FAIL;
	else
		goto END_FUNCTION;

GET_TOUCH_FAIL:
	I("%s: Now reset the Touch chip.\n", __func__);
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, true);
#else
	g_core_fp.fp_system_reset();
#endif
#ifdef HX_ZERO_FLASH
	if (g_core_fp.fp_0f_reload_to_active)
		g_core_fp.fp_0f_reload_to_active();
#endif
END_FUNCTION:
	if (debug_data != NULL)
		debug_data->fp_ts_dbg_func(ts, HX_FINGER_LEAVE);

	return;
}
/*end ts_work*/
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
/*
#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work_att.work);

	I(" %s in\n", __func__);
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);

	if (ret)
		E(" Unable to register fb_notifier: %d\n", ret);
}
#endif
*/
int himax_chip_common_init(void)
{

	int i = 0, err = 0;
	struct himax_ts_data *ts = private_ts;
	struct himax_platform_data *pdata=hv_pdata;
	I("PDATA START\n");

	I("ic_data START\n");
	g_point_buf = kzalloc(128 * sizeof(uint8_t), GFP_KERNEL);
	if (!g_point_buf) {
		err = -ENOMEM;
		goto err_dt_g_point_buf_fail;
	}
	ic_data = kzalloc(sizeof(*ic_data), GFP_KERNEL);
	if (ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
		goto err_dt_ic_data_fail;
	}

	/* allocate report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data), GFP_KERNEL);
	if (hx_touch_data == NULL) {
		err = -ENOMEM;
		goto err_alloc_touch_data_failed;
	}

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif
	himax_gpio_power_config(pdata);

	for (i = 0; i < g_hx_ic_dt_num; i++) {
	if (g_core_chip_dt[i].fp_chip_detect != NULL) {
			if (g_core_chip_dt[i].fp_chip_detect() == true) {
				I("%s: chip detect found! list_num=%d\n", __func__, i);
				goto found_hx_chip;
			}	else {
				I("%s: num=%d chip detect NOT found! goto Next\n", __func__, i);
				continue;
			}
		}
	}
	 
	if (i == g_hx_ic_dt_num) {
			E("%s: chip detect failed!\n", __func__);
			goto error_ic_detect_failed;
		}
found_hx_chip:
	if (g_core_fp.fp_chip_init != NULL) {
		g_core_fp.fp_chip_init();
	} else {
		E("%s: function point of chip_init is NULL!\n", __func__);
		goto error_ic_detect_failed;
	}

#ifdef HX_ZERO_FLASH
	g_auto_update_flag = true;
	memset(i_CTPM_firmware_name, 0x00, sizeof(i_CTPM_firmware_name));
	memcpy(i_CTPM_firmware_name, HX_NORMAL_FW, sizeof(char)*strlen(HX_NORMAL_FW));
	I("%s: end! FW file name(%s)\n", __func__, i_CTPM_firmware_name);
/*
	ts->himax_0f_update_wq = create_singlethread_workqueue("HMX_0f_update_reuqest");
	INIT_DELAYED_WORK(&ts->work_0f_update, g_core_fp.fp_0f_operation);
	queue_delayed_work(ts->himax_0f_update_wq, &ts->work_0f_update, msecs_to_jiffies(2000));
*/
#endif

	g_core_fp.fp_power_on_init();
	calculate_point_number();

	ts->pdata = pdata;
	/*calculate the data size*/
	calcDataSize();
	I("%s: calcDataSize complete\n", __func__);
#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif
	ts->suspended                      = false;
#ifdef	HX_PROTOCOL_A
	ts->protocol_type = PROTOCOL_TYPE_A;
#else
	ts->protocol_type = PROTOCOL_TYPE_B;
#endif
	I("%s: Use Protocol Type %c\n", __func__,
	  ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');


/*
#ifdef CONFIG_FB
	ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");

	if (!ts->himax_att_wq) {
		E(" allocate syn_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}

	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#endif
*/
#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable = 0;
#endif
#ifdef HX_HIGH_SENSE
	ts->HSEN_enable = 0;
#endif

	/*touch data init*/
	err = himax_report_data_init();

	if (err)
		goto error_ic_detect_failed;

	/*hx_thread = kthread_run(hx_ts_work_handler, 0, "himax-dev");
	if (IS_ERR(hx_thread)) {
		err = PTR_ERR(hx_thread);
		E(" failed to create kernel thread: %d\n", err);
	}*/


	err = himax_ts_register_interrupt();

	if (err)
		goto err_register_interrupt_failed;

	if (himax_debug_init())
		E(" %s: debug initial failed!\n", __func__);

#if defined(HX_ZERO_FLASH)

	if (g_auto_update_flag)
		himax_int_enable(0);
#endif
	return 0;
err_register_interrupt_failed:
	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
/*
#ifdef CONFIG_FB
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
err_get_intr_bit_failed:
#endif
*/
/*
err_input_register_device_failed:
	input_free_device(ts->input_dev);
*/
error_ic_detect_failed:
	if (gpio_is_valid(pdata->gpio_irq))
		gpio_free(pdata->gpio_irq);
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
#endif

err_alloc_touch_data_failed:
	kfree(ic_data);
	ic_data = NULL;
	kfree(g_point_buf);
	g_point_buf = NULL;
err_dt_ic_data_fail:
	kfree(pdata);
	kfree(ts);
err_dt_g_point_buf_fail:
	probe_fail_flag = 1;
	return err;
}

void himax_chip_common_deinit(void)
{
	struct himax_ts_data *ts = private_ts;

	himax_debug_remove();


	remove_proc_entry(HIMAX_PROC_TOUCH_FOLDER, NULL);
	himax_common_proc_deinit();

	if (!ts->use_irq) {
		hrtimer_cancel(&ts->timer);
		destroy_workqueue(ts->himax_wq);
	}
/*
#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		E("Error occurred while unregistering fb_notifier.\n");
	cancel_delayed_work_sync(&ts->work_att);
	destroy_workqueue(ts->himax_att_wq);
#endif
*/

#ifdef HX_ZERO_FLASH
	cancel_delayed_work_sync(&ts->work_0f_update);
	destroy_workqueue(ts->himax_0f_update_wq);
#endif
	if (gpio_is_valid(ts->pdata->gpio_irq))
		gpio_free(ts->pdata->gpio_irq);
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(ts->pdata->gpio_reset))
		gpio_free(ts->pdata->gpio_reset);
#endif

	kfree(hx_touch_data);
	kfree(ic_data);
	kfree(ts->pdata);
	kfree(ts);
	probe_fail_flag = 0;

	return;
}
int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	int ret;
	
#ifdef HX_SMART_WAKEUP
	int retry = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t rec_data[DATA_LEN_4] = {0};
#endif

	if (ts->suspended) {
		I("%s: Already suspended. Skipped.\n", __func__);
		return 0;
	} else {
		ts->suspended = true;
		I("%s: enter\n", __func__);
	}

#ifdef HX_SMART_WAKEUP

	if (ts->SMWP_enable) {
		I("SMWP Condition!\n");
		himax_int_enable(0);

#ifdef HX_RST_PIN_FUNC
	if(!(private_ts->chip_name == HX_83112A_SERIES_PWON))
		g_core_fp.fp_ic_reset(false, false);
#endif
I("%s:0x7fD0<-0xA55A\n", __func__);
	do {
		himax_in_parse_assign_cmd(0x10007fd0, tmp_addr, sizeof(tmp_addr));
		himax_in_parse_assign_cmd(0xA55AA55A, tmp_data, sizeof(tmp_data));
		g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
		usleep_range(1000, 1001);

		g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
			I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
		rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
	} while((retry++ < 10) && (rec_data[3] != 0xA5 || rec_data[2] != 0x5A || rec_data[1] != 0xA5 || rec_data[0] != 0x5A ));
	#if defined(HX_ZERO_FLASH)
			if(g_core_fp.fp_0f_reload_to_active)
				g_core_fp.fp_0f_reload_to_active();	
	#endif

#if defined(HX_HIGH_SENSE)
#ifndef HX_RESUME_SEND_CMD
	g_core_fp.fp_resend_cmd_func(ts->suspended);
#endif
#endif

		himax_int_enable(1);
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n", __func__);
		return 0;
		}

#endif
	himax_int_enable(0);
	g_core_fp.fp_suspend_ic_action();

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);

		if (ret)
			himax_int_enable(1);
	}

	/*ts->first_pressed = 0;*/
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;

	I("%s: END\n", __func__);
	return 0;
}
int himax_chip_common_resume(struct himax_ts_data *ts)
{
	int retry = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t rec_data[DATA_LEN_4] = {0};

	g_zero_event_count = 0;

#if defined(HX_ZERO_FLASH) && defined(HX_SMART_WAKEUP)
#endif
	I("%s: enter\n", __func__);

	if (ts->suspended == false) {
		I("%s: It had entered resume, skip this step\n", __func__);
		return 0;
	} else {
		ts->suspended = false;
	}
#if 0
	tmp_addr[3] = 0x10;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x7F;
	tmp_addr[0] = 0xD0;
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = 0x00;
	tmp_data[0] = 0x00;
	g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
#endif
	atomic_set(&ts->suspend_mode, 0);

#if defined(HX_RST_PIN_FUNC) && defined(HX_RESUME_HW_RESET)
	g_core_fp.fp_ic_reset(false, false);
#endif

	I("%s:0x7fD0<-0xA33AA33A\n", __func__);
	do {
		himax_in_parse_assign_cmd(0x10007fd0, tmp_addr, sizeof(tmp_addr));
		himax_in_parse_assign_cmd(0xA33AA33A, tmp_data, sizeof(tmp_data));
		g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
		usleep_range(1000, 1001);

		g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);
		I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
			rec_data[3], rec_data[2], rec_data[1], rec_data[0]);
	} while((retry++ < 10) && (rec_data[3] != 0xA3 || rec_data[2] != 0x3A || rec_data[1] != 0xA3 || rec_data[0] != 0x3A ));

#if defined(HX_ZERO_FLASH) && defined(HX_SMART_WAKEUP)
	if (!ts->SMWP_enable) {
		I("It will update fw after esd event in zero flash mode!\n");
		g_core_fp.fp_0f_operation_dirly();
		g_core_fp.fp_reload_disable(0);
		g_core_fp.fp_reload_fw_clear_register();
		g_core_fp.fp_sense_on(0x00);
		g_core_fp.fp_check_remapping();
	}
#endif

#if defined(HX_HIGH_SENSE) || defined(HX_SMART_WAKEUP)
	g_core_fp.fp_resend_cmd_func(ts->suspended);
#endif

	himax_report_all_leave_event(ts);

	g_core_fp.fp_resume_ic_action();
	himax_int_enable(1);
	return 0;
}
