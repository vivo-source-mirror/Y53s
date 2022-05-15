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
#include "hi846mipiraw_Sensor.h"

#define PFX "HI846_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_MAIN3_OTP_DATA_SIZE 0x143F /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_MAIN3_EEPROM_WRITE_ID 0xA8
#define VIVO_MAIN3_I2C_SPEED 400
#define VIVO_MAIN3_MAX_OFFSET 0x143e
#define VIVO_MAIN3_VENDOR_SUNNY 0x01
#define VIVO_MAIN3_VENDOR_TRULY 0x02
/*#define VIVO_MAIN3_VENDOR_QTECH 0x05*/
/*#define VIVO_MAIN3_VENDOR_OFILM 0x09*/
#define VIVO_MAIN3_VENDOR_LENS_ID 0x07
#define VIVO_MAIN3_VENDOR_VCM_ID 0x00
#define VIVO_MAIN3_VENDOR_DRIVERIC_ID 0x00
#define VIVO_MAIN3_VENDOR_PLATFORM_ID 0x03

static unsigned char main3_otp_data_hi846[VIVO_MAIN3_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001F;
static unsigned const int Fuse_id_addr = 0x0020;
static unsigned const int Fuse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
static unsigned const int Af_addr = 0x0066;
static unsigned const int Awb_addr = 0xCAB;
static unsigned const int Awb_checksum_addr = 0x0CE0;
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143E;

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main2_hi846[13];
extern MUINT32  material_inf_main_hi846[4];  
extern MUINT32  af_calib_inf_main2_hi846[6]; 

static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAIN3_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_MAIN3_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_MAIN3_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}

extern unsigned int is_atboot;
/*guojunzheng add*/
int vivo_main3_otp_read(void)
{
	int i = 0;
	int offset = 0x0000;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_NORMAL;

	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	if (is_atboot == 1) {
		LOG_INF("AT mode skip vivo_otp_read\n");
		return 1;
	}

	for (i = 0; i < VIVO_MAIN3_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset,  &main3_otp_data_hi846[i])) {
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n", offset,  main3_otp_data_hi846[i]);
			return 0;
		}
		/*LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  main3_otp_data_hi846[i]);*/
		offset++;
	}
	/*for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  main3_otp_data_hi846[i]);
	}*/

	/*check_if_group_valid = main3_otp_data_hi846[ModuleInfo_addr] | main3_otp_data_hi846[Awb_addr] | main3_otp_data_hi846[Af_addr] | main3_otp_data_hi846[Lsc_addr] | main3_otp_data_hi846[PD_Proc1_addr] | main3_otp_data_hi846[PD_Proc2_addr];*/
	if ((0x01 == main3_otp_data_hi846[ModuleInfo_addr]) && (0x01 == main3_otp_data_hi846[Fuse_id_addr]) && (0x01 == main3_otp_data_hi846[SN_addr]) && (0x01 == main3_otp_data_hi846[Awb_addr]) && (0x01 == main3_otp_data_hi846[Lsc_addr])  ) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if (  (0x02	!= main3_otp_data_hi846[0x01]) && (VIVO_MAIN3_VENDOR_TRULY != main3_otp_data_hi846[0x01])) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		} else if ((0x07 != main3_otp_data_hi846[0x0008]) || (0x11 != main3_otp_data_hi846[0x0009]) || (0x03 != main3_otp_data_hi846[0x000A]) || (VIVO_MAIN3_VENDOR_PLATFORM_ID != main3_otp_data_hi846[0x0002])) {
           
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		} else if ((0xff != main3_otp_data_hi846[0x000B]) || (0x00 != main3_otp_data_hi846[0x000C]) || (0x0b != main3_otp_data_hi846[0x000D]) || (0x01 != main3_otp_data_hi846[0x000E])) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", main3_otp_data_hi846[0x000B], main3_otp_data_hi846[0x000C], main3_otp_data_hi846[0x000D], main3_otp_data_hi846[0x000E]);
			return 0;
		}
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[ModuleInfo_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
			}
		
		/****material info start****/
			material_inf_main_hi846[0] = (MUINT32)main3_otp_data_hi846[0xF];
			material_inf_main_hi846[1] = (MUINT32)main3_otp_data_hi846[0x10];
			material_inf_main_hi846[2] = (MUINT32)main3_otp_data_hi846[0x11];
			material_inf_main_hi846[3] = (MUINT32)main3_otp_data_hi846[0x12];
			LOG_INF("material_inf_main_ov32alq[0] = 0x%x, material_inf_main_ov32alq[1] = 0x%x, material_inf_main_ov32alq[2] = 0x%x,material_inf_main_ov32alq[3] = 0x%x\n",
				material_inf_main_hi846[0]  , material_inf_main_hi846[1],  material_inf_main_hi846[2], material_inf_main_hi846[3]);
		/****material info end****/
		/****Fuse id_checksum****/
		checksum = 0;
		for (i = Fuse_id_addr+1; i < Fuse_id_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[Fuse_id_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Fuse id_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
		
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[Awb_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
			}

		#if 0
		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[Af_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AF_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
			}
		#endif
		
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[Lsc_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
        af_calib_inf_main2_hi846[0] = (main3_otp_data_hi846[Af_addr + 7]<<8| main3_otp_data_hi846[Af_addr + 8])/256;
		af_calib_inf_main2_hi846[1] = (main3_otp_data_hi846[Af_addr + 7]<<8| main3_otp_data_hi846[Af_addr + 8])%256;
		af_calib_inf_main2_hi846[2] = (main3_otp_data_hi846[Af_addr+ 9]<<8| main3_otp_data_hi846[Af_addr + 10])/256;
		af_calib_inf_main2_hi846[3] = (main3_otp_data_hi846[Af_addr+ 9]<<8| main3_otp_data_hi846[Af_addr + 10])%256;
		af_calib_inf_main2_hi846[4] = (main3_otp_data_hi846[Af_addr+ 11]<<8| main3_otp_data_hi846[Af_addr + 12])/256;
		af_calib_inf_main2_hi846[5] = (main3_otp_data_hi846[Af_addr+ 11]<<8| main3_otp_data_hi846[Af_addr + 12])%256;
        LOG_INF("af1 = %d, af2= %d af3= %d af4= %d af5= %d af5= %d\n",af_calib_inf_main2_hi846[0],af_calib_inf_main2_hi846[1],af_calib_inf_main2_hi846[2],af_calib_inf_main2_hi846[3],af_calib_inf_main2_hi846[4],af_calib_inf_main2_hi846[5]);

		#if 0
		/****PDAF_proc1_checksum****/
		checksum = 0;
		for (i = PDAF_proc1_addr+1; i < PDAF_proc1_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
		}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[PDAF_proc1_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
			}

		/****PDAF_proc2_checksum****/
		checksum = 0;
		for (i = PDAF_proc2_addr+1; i < PDAF_proc2_checksum_addr; i++) {
			checksum += main3_otp_data_hi846[i];
		}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[PDAF_proc2_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
		#endif
		
		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			LOG_INF("main3_otp_data_hi846[0x%x] = 0x%x\n", i, main3_otp_data_hi846[i]);
			checksum += main3_otp_data_hi846[i];
			}
			checksum = checksum % 0xff+1;
		if (main3_otp_data_hi846[SN_checksum_addr] != checksum) {
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
		sn_inf_main2_hi846[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main2_hi846[i +1] = (MUINT32)main3_otp_data_hi846[i + SN_addr+1];
			/*LOG_INF("sn_inf_main2_hi846[%d] = 0x%x, main3_otp_data_hi846[0x%x] = 0x%x\n", i+1  , sn_inf_main2_hi846[i +1],  i +SN_addr+1, main3_otp_data_hi846[i + SN_addr+1]);*/
		}


		/****check if awb out of range[high cct]****/
		R_unit = main3_otp_data_hi846[Awb_addr+1];
		R_unit = (R_unit << 8) | (main3_otp_data_hi846[Awb_addr+2]);
		B_unit = main3_otp_data_hi846[Awb_addr+3];
		B_unit = (B_unit << 8) | (main3_otp_data_hi846[Awb_addr+4]);
		G_unit = main3_otp_data_hi846[Awb_addr+5];
		G_unit = (G_unit << 8) | (main3_otp_data_hi846[Awb_addr+6]);

		R_golden = main3_otp_data_hi846[Awb_addr+7];
		R_golden = (R_golden << 8) | (main3_otp_data_hi846[Awb_addr+8]);
		B_golden = main3_otp_data_hi846[Awb_addr+9];
		B_golden = (B_golden << 8) | (main3_otp_data_hi846[Awb_addr+10]);
		G_golden = main3_otp_data_hi846[Awb_addr+11];
		G_golden = (G_golden << 8) | (main3_otp_data_hi846[Awb_addr+12]);

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
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
		/****check if awb out of range[low cct]****/
		R_unit_low = main3_otp_data_hi846[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (main3_otp_data_hi846[Awb_addr+14]);
		B_unit_low = main3_otp_data_hi846[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (main3_otp_data_hi846[Awb_addr+16]);
		G_unit_low = main3_otp_data_hi846[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (main3_otp_data_hi846[Awb_addr+18]);

		R_golden_low = main3_otp_data_hi846[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (main3_otp_data_hi846[Awb_addr+20]);
		B_golden_low = main3_otp_data_hi846[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (main3_otp_data_hi846[Awb_addr+22]);
		G_golden_low = main3_otp_data_hi846[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (main3_otp_data_hi846[Awb_addr+24]);

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
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}

		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  main3_otp_data_hi846[Af_addr+2];
		AF_inf = (AF_inf << 8) | (main3_otp_data_hi846[Af_addr+3]);

		AF_mac = main3_otp_data_hi846[Af_addr+4];
		AF_mac = (AF_mac << 8) | (main3_otp_data_hi846[Af_addr+5]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
		if (diff_inf > 70 || diff_mac > 80 || diff_inf_macro > 450 || diff_inf_macro < 250) {  /*AF code out of range*/
			MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("AF out of range error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
			return 0;
		}
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF(" read_S5K2L8_PDAF_data");
			if(i < 496)
				PDAF_data[i] = main3_otp_data_hi846[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = main3_otp_data_hi846[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &main3_otp_data_hi846[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/
		MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		MAIN3_OTP_ERROR_CODE_HI846 = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF(" invalid otp data. error!!!   otp_error_code:%d\n", MAIN3_OTP_ERROR_CODE_HI846);
		return 0;
	}
}
/*vivo cfx add end*/
