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
#include "s5kgd1sppd1934mipiraw_Sensor.h"

#define PFX "S5KGD1SPPD1934_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo cfx add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x143F	 /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
/*#define AF_RANGE_GOT*/  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA2
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x143E
/*#define VIVO_VENDOR_SUNNY 0x01*/
#define VIVO_VENDOR_TRULY 0x02
/*#define VIVO_VENDOR_QTECH 0x05*/
#define VIVO_VENDOR_OFILM 0x09
#define VIVO_VENDOR_LENS_ID 0x01
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03

static unsigned char vivo_otp_data_s5kgd1sppd1934[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001F;
static unsigned const int Fuse_id_addr = 0x0020;
static unsigned const int Fuse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
static unsigned const int Awb_addr = 0x0CAB;
static unsigned const int Awb_checksum_addr = 0x0CE0;
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143E;

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_sub_s5kgd1sppd1934[13];
extern MUINT32  material_inf_sub_s5kgd1sppd1934[4];

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

extern unsigned int is_atboot;
/*guojunzheng add*/
int s5kgd1sppd1934_otp_read(void)
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
	S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	if (is_atboot == 1) {
		LOG_INF("AT mode skip S5KGD1SPPD1934_otp_read\n");
		return 1;
	}
	
	/* guojunzheng add end */

	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset,  &vivo_otp_data_s5kgd1sppd1934[i])) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_EMPTY;
			LOG_INF("read_vivo_eeprom 0x%0x %d fail \n", offset,  vivo_otp_data_s5kgd1sppd1934[i]);
			return 0;
		}
		/*LOG_INF("hope read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data_s5kgd1sppd1934[i]);*/
		offset++;
	}
	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_s5kgd1sppd1934[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data_s5kgd1sppd1934[ModuleInfo_addr] | vivo_otp_data_s5kgd1sppd1934[Awb_addr] | vivo_otp_data_s5kgd1sppd1934[Af_addr] | vivo_otp_data_s5kgd1sppd1934[Lsc_addr] | vivo_otp_data_s5kgd1sppd1934[PD_Proc1_addr] | vivo_otp_data_s5kgd1sppd1934[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data_s5kgd1sppd1934[ModuleInfo_addr]) &&
	      (0x01 == vivo_otp_data_s5kgd1sppd1934[Fuse_id_addr]) && 
	      (0x01 == vivo_otp_data_s5kgd1sppd1934[SN_addr]) && 
	      (0x01 == vivo_otp_data_s5kgd1sppd1934[Awb_addr]) && 
	      (0x01 == vivo_otp_data_s5kgd1sppd1934[Lsc_addr])) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		if (  (VIVO_VENDOR_OFILM != vivo_otp_data_s5kgd1sppd1934[0x01]) && 
		        (VIVO_VENDOR_TRULY != vivo_otp_data_s5kgd1sppd1934[0x01])) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Module ID error!!!    otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		} 

		if ((VIVO_VENDOR_LENS_ID != vivo_otp_data_s5kgd1sppd1934[0x0008]) || 
		      (VIVO_VENDOR_VCM_ID != vivo_otp_data_s5kgd1sppd1934[0x0009]) || 
		      (VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_s5kgd1sppd1934[0x000A]) || 
		      (VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_s5kgd1sppd1934[0x0002])) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		} 

		if ((0xff != vivo_otp_data_s5kgd1sppd1934[0x000B]) || 
		      (0x00 != vivo_otp_data_s5kgd1sppd1934[0x000C]) || 
		      (0x0b != vivo_otp_data_s5kgd1sppd1934[0x000D]) || 
		      (0x01 != vivo_otp_data_s5kgd1sppd1934[0x000E])) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF(": calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", vivo_otp_data_s5kgd1sppd1934[0x000B], vivo_otp_data_s5kgd1sppd1934[0x000C], vivo_otp_data_s5kgd1sppd1934[0x000D], vivo_otp_data_s5kgd1sppd1934[0x000E]);
			return 0;
		}
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[ModuleInfo_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
			}

		/****Fuse id_checksum****/
		checksum = 0;
		for (i = Fuse_id_addr+1; i < Fuse_id_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[Fuse_id_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Fuse id_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
		
		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[Awb_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AWB_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
			}
#if 0
		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[Af_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("AF_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
			}
#endif
		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[Lsc_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("LSC_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
#if 0
		/****PDAF_proc1_checksum****/
		checksum = 0;
		for (i = PDAF_proc1_addr+1; i < PDAF_proc1_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[PDAF_proc1_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF_proc2_checksum****/
		checksum = 0;
		for (i = PDAF_proc2_addr+1; i < PDAF_proc2_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[PDAF_proc2_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
#endif
		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			LOG_INF("vivo_otp_data_s5kgd1sppd1934[0x%x] = 0x%x\n", i, vivo_otp_data_s5kgd1sppd1934[i]);
			checksum += vivo_otp_data_s5kgd1sppd1934[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kgd1sppd1934[SN_checksum_addr] != checksum) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_sub_s5kgd1sppd1934[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_sub_s5kgd1sppd1934[i +1] = (MUINT32)vivo_otp_data_s5kgd1sppd1934[i + SN_addr+1];
			/*LOG_INF("sn_inf_sub_s5kgd1sppd1934[%d] = 0x%x, vivo_otp_data_s5kgd1sppd1934[0x%x] = 0x%x\n", i+1  , sn_inf_sub_s5kgd1sppd1934[i +1],  i +SN_addr+1, vivo_otp_data_s5kgd1sppd1934[i + SN_addr+1]);*/
		}

        /****material info start****/
        material_inf_sub_s5kgd1sppd1934[0] = (MUINT32)vivo_otp_data_s5kgd1sppd1934[0x0F];
        material_inf_sub_s5kgd1sppd1934[1] = (MUINT32)vivo_otp_data_s5kgd1sppd1934[0x10];
        material_inf_sub_s5kgd1sppd1934[2] = (MUINT32)vivo_otp_data_s5kgd1sppd1934[0x11];
        material_inf_sub_s5kgd1sppd1934[3] = (MUINT32)vivo_otp_data_s5kgd1sppd1934[0x12];
        LOG_INF("material_inf_sub_s5kgd1sppd1934[0] = 0x%x, material_inf_sub_s5kgd1sppd1934[1] = 0x%x, material_inf_sub_s5kgd1sppd1934[2] = 0x%x,material_inf_sub_s5kgd1sppd1934[3] = 0x%x\n",
            material_inf_sub_s5kgd1sppd1934[0]  , material_inf_sub_s5kgd1sppd1934[1],  material_inf_sub_s5kgd1sppd1934[2], material_inf_sub_s5kgd1sppd1934[3]);
        /****material info end****/

		/****check if awb out of range[high cct]****/
		R_unit = vivo_otp_data_s5kgd1sppd1934[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+2]);
		B_unit = vivo_otp_data_s5kgd1sppd1934[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+4]);
		G_unit = vivo_otp_data_s5kgd1sppd1934[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+6]);

		R_golden = vivo_otp_data_s5kgd1sppd1934[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+8]);
		B_golden = vivo_otp_data_s5kgd1sppd1934[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+10]);
		G_golden = vivo_otp_data_s5kgd1sppd1934[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
		/****check if awb out of range[low cct]****/
		R_unit_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+14]);
		B_unit_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+16]);
		G_unit_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+18]);

		R_golden_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+20]);
		B_golden_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+22]);
		G_golden_low = vivo_otp_data_s5kgd1sppd1934[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_s5kgd1sppd1934[Awb_addr+24]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}

		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_s5kgd1sppd1934[Af_addr+2];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_s5kgd1sppd1934[Af_addr+3]);

		AF_mac = vivo_otp_data_s5kgd1sppd1934[Af_addr+4];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_s5kgd1sppd1934[Af_addr+5]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
		if (diff_inf > 70 || diff_mac > 80 || diff_inf_macro > 450 || diff_inf_macro < 250) {  /*AF code out of range*/
			S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("AF out of range error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
			return 0;
		}
		#endif

		/*cfx add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF(" read_S5KGD1SPPD1934_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_s5kgd1sppd1934[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_s5kgd1sppd1934[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_s5kgd1sppd1934[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*cfx add for pdaf data end*/
		S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		S5KGD1SPPD1934_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF(" invalid otp data. error!!!   otp_error_code:%d\n", S5KGD1SPPD1934_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo cfx add end*/
