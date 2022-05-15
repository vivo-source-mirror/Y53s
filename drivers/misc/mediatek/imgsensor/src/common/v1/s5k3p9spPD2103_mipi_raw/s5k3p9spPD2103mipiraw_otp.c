#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "s5k3p9spPD2103mipiraw_Sensor.h"

#define PFX "SUB[3109]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x1c41/*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA2
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x1c40
#define VIVO_VENDOR_SUNNY 0x01
#define VIVO_VENDOR_TRULY 0x02	
/*#define VIVO_VENDOR_QTECH 0x05*/
#define VIVO_VENDOR_OFILM 0x09
#define VIVO_VENDOR_LENS_ID 0x01
/*second vendor for Lens*/
/*#define VIVO_VENDOR_LENS_ID2 0x01*/
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03

static unsigned char vivo_otp_data[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x0001f;
static unsigned const int Sensor_fuse_id_addr = 0x0020;
static unsigned const int Sensor_fuse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
/*static unsigned const int Af_addr = 0x0020;*/
/*static unsigned const int Af_checksum_addr = 0x003e;*/
static unsigned const int Awb_addr = 0x0cab;
static unsigned const int Awb_checksum_addr = 0x0ce0;
/*static unsigned const int Awb_low_addr = 0x0cb8;*/
/*static unsigned const int Awb_low_checksum_addr = 0x008b;*/
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143e;
//static unsigned const int QSC_addr = 0x0F8E;
//static unsigned const int QSC_checksum_addr = 0x179F;
static unsigned const int XTLK_addr = 0x143F;
static unsigned const int XTLK_checksum_addr = 0x1c40;
/*static unsigned const int PDAF_addr = 0x07b3;*/
/*static unsigned const int PDAF_checksum_addr = 0x0cd2;*/
#ifdef AF_RANGE_GOT
static unsigned const int AF_inf_golden = 0;/*320;*/
static unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32 crosstalk_data[2049];  /*0 flag   1-606 data*/
extern MUINT32  sn_inf_sub_s5k3p9sp04[13];
extern MUINT32  material_inf_sub_s5k3p9sp04[4];  

static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
	if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);
	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}

/*extern unsigned int is_atboot;*/
/*guojunzheng add*/
int read_sn_id(void)
{
   int i = 0;
   int offset = 0x46;
   unsigned char sn_id[4] ={0x00};
   for(i = 0; i < 4; i++ ){
   	if (!vivo_read_eeprom(offset,  &sn_id[i])) {
			printk("read_vivo_eeprom 0x%0x %d fail \n", offset,  sn_id[i]);
			return 0;
	}
		/*LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data[i]);*/
	offset++;	
	}
	return sn_id[0]<<24|sn_id[1]<<16|sn_id[2]<<8|sn_id[3];
}
int SUB3109_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	//int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;
	
	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;

	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	/*if (is_atboot == 1) {
		LOG_INF("AT mode skip S5K3P9SP04_otp_read\n");
		return 1;
	}
	*/
	/* guojunzheng add end */

	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset,  &vivo_otp_data[i])) {
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n", offset,  vivo_otp_data[i]);
			return 0;
		}
		/*LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data[i]);*/
		offset++;
	}
	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data[ModuleInfo_addr] | vivo_otp_data[Awb_addr] | vivo_otp_data[Af_addr] | vivo_otp_data[Lsc_addr] | vivo_otp_data[PD_Proc1_addr] | vivo_otp_data[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data[ModuleInfo_addr]) && 
		(0x01 == vivo_otp_data[Sensor_fuse_id_addr]) && 
		(0x01 == vivo_otp_data[SN_addr]) && 
		(0x01 == vivo_otp_data[Awb_addr]) && 
		(0x01 == vivo_otp_data[Lsc_addr])) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		/*if ((VIVO_VENDOR_SUNNY != vivo_otp_data[0x01]) && (VIVO_VENDOR_QTECH != vivo_otp_data[0x01])) {*/
		if ((VIVO_VENDOR_SUNNY != vivo_otp_data[0x01])){
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[ModuleInfo_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
			}

		/****material info start****/
			material_inf_sub_s5k3p9sp04[0] = (MUINT32)vivo_otp_data[0x0F];
			material_inf_sub_s5k3p9sp04[1] = (MUINT32)vivo_otp_data[0x10];
			material_inf_sub_s5k3p9sp04[2] = (MUINT32)vivo_otp_data[0x11];
			material_inf_sub_s5k3p9sp04[3] = (MUINT32)vivo_otp_data[0x12];
			LOG_INF("material_inf_sub_s5k3p9sp04[0] = 0x%x, material_inf_sub_s5k3p9sp04[1] = 0x%x, material_inf_sub_s5k3p9sp04[2] = 0x%x,material_inf_sub_s5k3p9sp04[3] = 0x%x\n",
				material_inf_sub_s5k3p9sp04[0]  , material_inf_sub_s5k3p9sp04[1],  material_inf_sub_s5k3p9sp04[2], material_inf_sub_s5k3p9sp04[3]);
		/****material info end****/


		/****SENSOR Fuse ID_checksum****/
		checksum = 0;
		for (i = Sensor_fuse_id_addr+1; i < Sensor_fuse_id_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[Sensor_fuse_id_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SENSOR Fuse ID_checksum error!!!	otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		
		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data[0x%x] = 0x%x\n", i, vivo_otp_data[i]);*/
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[SN_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!	otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_sub_s5k3p9sp04[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_sub_s5k3p9sp04[i +1] = (MUINT32)vivo_otp_data[i + SN_addr+1];
			/*LOG_INF("sn_inf_sub_s5k3p9sp04[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i+1  , sn_inf_sub_s5k3p9sp04[i +1],  i +SN_addr+1, vivo_otp_data[i + SN_addr+1]);*/
		}
       
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[Awb_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_5100K_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
			}
#if 0
		/****AWB_checksum 3100K****/
		checksum = 0;
		for (i = Awb_low_addr+1; i < Awb_low_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[Awb_low_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_3100K_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
			}
		#if 0
		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[Af_checksum_addr] != checksum) {
			OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AF_checksum error!!!   otp_error_code:%d\n", OTP_ERROR_CODE);
			return 0;
			}
		#endif
#endif
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[Lsc_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}


		/****XTLK_checksum****/
		checksum = 0;
		for (i = XTLK_addr+1; i < XTLK_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[XTLK_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("XTLK_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		 for (i = 0; i < 2049; i++) {	
			crosstalk_data[i] = (MUINT32)vivo_otp_data[i + XTLK_addr];
			LOG_INF("crosstalk_data[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i, crosstalk_data[i],	i + XTLK_addr, vivo_otp_data[i + XTLK_addr]);
		 }				
#if 0
		/****QSC_checksum****/
		checksum = 0;
		for (i = QSC_addr+1; i < QSC_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[QSC_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("QSC_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
#endif
		
#if 0
		/****PDAF_checksum****/
		checksum = 0;
		for (i = PDAF_addr+1; i < PDAF_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data[PDAF_checksum_addr] != checksum) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
			}
		#endif
		
		/****check if awb out of range[high cct]****/
		R_unit = vivo_otp_data[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data[Awb_addr+2]);
		B_unit = vivo_otp_data[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data[Awb_addr+4]);
		G_unit = vivo_otp_data[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data[Awb_addr+6]);

		R_golden = vivo_otp_data[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data[Awb_addr+8]);
		B_golden = vivo_otp_data[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data[Awb_addr+10]);
		G_golden = vivo_otp_data[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		#if 0
		/****check if awb out of range[low cct]****/
		R_unit_low = vivo_otp_data[Awb_low_addr];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data[Awb_low_addr+1]);
		B_unit_low = vivo_otp_data[Awb_low_addr+2];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data[Awb_low_addr+3]);
		G_unit_low = vivo_otp_data[Awb_low_addr+4];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data[Awb_low_addr+5]);

		R_golden_low = vivo_otp_data[Awb_low_addr+6];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data[Awb_low_addr+7]);
		B_golden_low = vivo_otp_data[Awb_low_addr+8];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data[Awb_low_addr+9]);
		G_golden_low = vivo_otp_data[Awb_low_addr+10];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data[Awb_low_addr+11]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		#endif
		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data[Af_addr+2];
		AF_inf = (AF_inf << 8) | (vivo_otp_data[Af_addr+3]);

		AF_mac = vivo_otp_data[Af_addr+4];
		AF_mac = (AF_mac << 8) | (vivo_otp_data[Af_addr+5]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
		if (diff_inf > 70 || diff_mac > 80 || diff_inf_macro > 450 || diff_inf_macro < 250) {  /*AF code out of range*/
			S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("AF out of range error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
			return 0;
		}
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("read_S5K2L8_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/

		return 1;
	} else {
		S5K3P9SP04_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", S5K3P9SP04_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
