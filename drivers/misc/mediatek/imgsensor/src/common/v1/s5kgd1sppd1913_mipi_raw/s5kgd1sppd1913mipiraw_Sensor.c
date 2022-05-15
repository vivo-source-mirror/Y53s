/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5KGD1SPPD1913mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

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
#include "s5kgd1sppd1913mipiraw_Sensor.h"

#define PFX "S5KGD1SPPD1913_camera_sensor"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static kal_uint16 hdr_le, hdr_me, hdr_se;

static imgsensor_info_struct imgsensor_info = {
		.sensor_id = S5KGD1SPPD1913_SENSOR_ID,

		.checksum_value = 0xffb1ec31,

		.pre = {
		.pclk = 1144000000, //960000000
		.linelength = 14528,  //14528
		.framelength = 2624,  //2624
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 345800000, //479700000
		},
	
		.cap = {
		.pclk = 1144000000,//960000000
		.linelength = 13728,
		.framelength = 5556,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 6528,//6560
		.grabwindow_height = 4896,//4928
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,
		.mipi_pixel_rate = 605800000,//479700000  6048
		},
	
		.normal_video = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1840,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 302900000, //479700000
		},
		.hs_video = {
		.pclk = 1144000000,
		.linelength = 13880,
		.framelength = 2746,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
		.slim_video = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		},
		.custom1 = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300, 
        .mipi_pixel_rate = 345800000,		
		},
		.custom2 = {
		.pclk = 1144000000,
		.linelength = 14528,
		.framelength = 2624,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1840,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,   
		.mipi_pixel_rate = 302900000,
	},
        .custom3 = {
		.pclk = 1144000000,
		.linelength = 13880,
		.framelength = 2746,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,   
		.mipi_pixel_rate = 172900000, //mipi speed  x 4
	},
		.custom4 = {
			.pclk = 1144000000,
			.linelength = 14528,
			.framelength = 2624,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			.max_framerate = 300, 
			.mipi_pixel_rate = 345800000,		
		},
		.custom5 = {
			.pclk = 1144000000,
			.linelength = 14528,
			.framelength = 2624,
			.startx = 0,
			.starty = 0,
			.grabwindow_width = 3264,
			.grabwindow_height = 2448,
			.mipi_data_lp2hs_settle_dc = 85,
			.max_framerate = 300, 
			.mipi_pixel_rate = 345800000,		
		},		
		.margin = 5,
		.min_shutter = 4,
		.max_frame_length = 0xffff,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first; 0, se first*/
		.sensor_mode_num = 10,	  /*support sensor mode num*/

		.cap_delay_frame = 2,/*3 guanjd modify for cts*/
		.pre_delay_frame = 2,/*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
	    .custom1_delay_frame = 3,
	    .custom2_delay_frame = 3,
	    .custom3_delay_frame = 3,
		.custom4_delay_frame = 2,
		.custom5_delay_frame = 2,
	
		.isp_driving_current = ISP_DRIVING_4MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
		//.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
		.mclk = 26,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 1000,
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x3D0,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{ 6560, 4944,	  16,	16, 6528, 4912, 3264,  2456, 0000, 0004, 3264,  2448,	  0,	0, 3264, 2448}, /*Preview*/
	{ 6560, 4944,	  0,	8, 6560, 4928, 6560, 4928, 0000, 0000, 6560, 4928,	  0,	0, 6528, 4896}, //hw-remosiac capture
	
	{ 6560, 4944,	  16,	624, 6528, 3696, 3264,  1848, 0000, 0004, 3264,  1840,	  0,	0, 3264, 1840}, /*video*/
	{ 6560, 4944,	  0,	0, 6560, 3024, 1344,  756, 0000, 0000, 1344,  756,	  0,	0, 1632, 1224}, /*hs_video,don't use*/
	{ 6560, 4944,	  16,	16, 6528, 4912, 3264,  2456, 0000, 0004, 3264,  2448,	  0,	0, 3264, 2448}, /* slim video*/
	{ 6560, 4944,	  16,	16, 6528, 4912, 3264,  2456, 0000, 0004, 3264,  2448,	  0,	0, 3264, 2448}, /* custom1*/
	{ 6560, 4944,	  16,	624, 6528, 3696, 3264,  1848, 0000, 0004, 3264,  1840,	  0,	0, 3264, 1840}, /* custom2*/
	{ 6560, 4944,	  0,	0, 6560, 3024, 1344,  756, 0000, 0000, 1344,  756,	  0,	0, 1632, 1224}, /* custom3*/
	{ 6560, 4944,	  16,	16, 6528, 4912, 3264,  2456, 0000, 0004, 3264,  2448,	  0,	0, 3264, 2448}, /*custom4*/
	{ 6560, 4944,	  16,	16, 6528, 4912, 3264,  2456, 0000, 0004, 3264,	2448,	  0,	0, 3264, 2448}, /*custom5*/
}; /*cpy from preview*/

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x05, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0B40, 0x0002, /*VC0:raw, VC1:Embedded header, RAW8*/
	 0x00, 0x30, 0x0B40, 0x0018, 0x00, 0x00, 0x0E10, 0x0001},/*VC2:embedded footer, RAW8*/
	/* Capture mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1680, 0x10D8, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x00, 0x0000, 0x0000, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0B40, 0x0002,
	 0x00, 0x30, 0x0B40, 0x0018, 0x00, 0x00, 0x0000, 0x0000},
};
/*no mirror flip*/

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/*extern void kdSetI2CSpeed(u16 i2cSpeed);*/
/*extern bool read_2l9_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);*/
/****hope  add for CameraEM otp errorcode****/
extern int s5kgd1sppd1913_otp_read(void);
MUINT32 vivo_s5kgd1sppd1913otp_read_when_power_on;
extern otp_error_code_t S5KGD1SPPD1913_OTP_ERROR_CODE;
MUINT32  sn_inf_sub_s5kgd1sppd1913[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_sub_s5kgd1sppd1913[4];
static int fps_count;
/*extern u32 sensor_temperature[10];*/
/****hope add end****/



static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	 /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define USE_TNP_BURST	0  /*samsung*/
#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 225	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4

#endif

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;
	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								4, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;

#endif
	}
	return 0;
}



static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	 write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	 write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{

	kal_uint16 realtime_fps = 0;

	if (fps_count < 5)
		fps_count ++;
	if (fps_count == 5){
		LOG_INF("Recovery voltage\n");
		write_cmos_sensor_16_16(0x6028, 0x4000);
		write_cmos_sensor_16_16(0xF44A, 0x000E);
		fps_count ++;
	}

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		/* Extend frame length*/
	        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);

	    }
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);

	}
	/* Update Shutter*/
	#if 0
	if (imgsensor.sensor_mode != IMGSENSOR_MODE_CAPTURE)
		shutter = shutter >>1;
	#endif
	
	write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);
	LOG_INF("sensor_mode=%d\n", imgsensor.sensor_mode);
	LOG_INF("shutter = %d, framelength = %d\n", shutter, imgsensor.frame_length);

}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

	LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);

}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

	reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/*gain= 1024;for test*/
	/*return; for test*/

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
	    LOG_INF("Error gain setting");

	    if (gain < BASEGAIN)
	        gain = BASEGAIN;
	    else if (gain > 32 * BASEGAIN)
	        gain = 32 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_16(0x0204,reg_gain);
	/*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
	/*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/

	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {

	    case IMAGE_NORMAL:

	        write_cmos_sensor_16_8(0x0101,0x00);   /* Gr*/
	        break;

	    case IMAGE_H_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x01);
	        break;

	    case IMAGE_V_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x02);
	        break;

	    case IMAGE_HV_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x03);/*Gb*/
	        break;
	    default:
	    LOG_INF("Error image_mirror setting\n");
	}
}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0x01);
		mdelay(10);
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			if (framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt= %d.\n", framecnt);
	}
	return ERROR_NONE;
}

#if USE_TNP_BURST
static const u16 uTnpArrayA[] = {
};
	
static const u16 uTnpArrayB[] = {
};
static const u16 uTnpArrayC[] = {
};
static const u16 uTnpArray_Face_A[] = {
};
static const u16 uTnpArray_Face_B[] = {
};  
#endif
	
static kal_uint16 addr_data_pair_init_otp[] = {
0x6028, 0x2001,    //Swpage
0x602A, 0x518C,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0549,
0x6F12, 0x0448,
0x6F12, 0x054A,
0x6F12, 0xC1F8,
0x6F12, 0xD006,
0x6F12, 0x101A,
0x6F12, 0xA1F8,
0x6F12, 0xD406,
0x6F12, 0x00F0,
0x6F12, 0x79B9,
0x6F12, 0x2001,
0x6F12, 0x5628,
0x6F12, 0x2000,
0x6F12, 0x68C0,
0x6F12, 0x2001,
0x6F12, 0xF800,
0x6F12, 0x70B5,
0x6F12, 0xC94D,
0x6F12, 0xC94C,
0x6F12, 0x2888,
0x6F12, 0x2081,
0x6F12, 0xA081,
0x6F12, 0x00F0,
0x6F12, 0xBAF9,
0x6F12, 0x2080,
0x6F12, 0x2888,
0x6F12, 0xE080,
0x6F12, 0x0320,
0x6F12, 0x00F0,
0x6F12, 0xB9F9,
0x6F12, 0x0820,
0x6F12, 0x00F0,
0x6F12, 0xBBF9,
0x6F12, 0xC34D,
0x6F12, 0xA080,
0x6F12, 0x287D,
0x6F12, 0x0328,
0x6F12, 0x06D1,
0x6F12, 0x0020,
0x6F12, 0x2875,
0x6F12, 0xC048,
0x6F12, 0x00F0,
0x6F12, 0xB6F9,
0x6F12, 0x00F0,
0x6F12, 0xB9F9,
0x6F12, 0x287D,
0x6F12, 0x58BB,
0x6F12, 0xBE4C,
0x6F12, 0x94F8,
0x6F12, 0x6709,
0x6F12, 0x68B9,
0x6F12, 0xBD48,
0x6F12, 0x0078,
0x6F12, 0x50B9,
0x6F12, 0x94F8,
0x6F12, 0x6C09,
0x6F12, 0x38B9,
0x6F12, 0xBB48,
0x6F12, 0x007A,
0x6F12, 0x10B1,
0x6F12, 0x94F8,
0x6F12, 0x5C09,
0x6F12, 0x08B9,
0x6F12, 0x687D,
0x6F12, 0x18B1,
0x6F12, 0x1021,
0x6F12, 0xB348,
0x6F12, 0x00F0,
0x6F12, 0xA6F9,
0x6F12, 0x94F8,
0x6F12, 0x6909,
0x6F12, 0x08B9,
0x6F12, 0x687D,
0x6F12, 0x18B1,
0x6F12, 0x4021,
0x6F12, 0xAF48,
0x6F12, 0x00F0,
0x6F12, 0x9DF9,
0x6F12, 0x00F0,
0x6F12, 0xA0F9,
0x6F12, 0x08B9,
0x6F12, 0x687D,
0x6F12, 0x38B1,
0x6F12, 0x0C21,
0x6F12, 0xAA48,
0x6F12, 0x00F0,
0x6F12, 0x94F9,
0x6F12, 0x0121,
0x6F12, 0xA848,
0x6F12, 0x00F0,
0x6F12, 0x90F9,
0x6F12, 0x00F0,
0x6F12, 0x98F9,
0x6F12, 0x687D,
0x6F12, 0x0028,
0x6F12, 0x04D0,
0x6F12, 0xBDE8,
0x6F12, 0x7040,
0x6F12, 0xA748,
0x6F12, 0x00F0,
0x6F12, 0x95B9,
0x6F12, 0x70BD,
0x6F12, 0x8908,
0x6F12, 0x8900,
0x6F12, 0x41EA,
0x6F12, 0x4000,
0x6F12, 0x82B2,
0x6F12, 0x3E21,
0x6F12, 0x46F2,
0x6F12, 0x4420,
0x6F12, 0x00F0,
0x6F12, 0x8FB9,
0x6F12, 0x2DE9,
0x6F12, 0xFE4F,
0x6F12, 0x00F0,
0x6F12, 0x90F9,
0x6F12, 0x00F0,
0x6F12, 0x93F9,
0x6F12, 0x0128,
0x6F12, 0x7DD1,
0x6F12, 0x9D4C,
0x6F12, 0x94F8,
0x6F12, 0xA700,
0x6F12, 0x0090,
0x6F12, 0x94F8,
0x6F12, 0xA800,
0x6F12, 0xDF34,
0x6F12, 0x0190,
0x6F12, 0x00F0,
0x6F12, 0x8CF9,
0x6F12, 0x994F,
0x6F12, 0x0646,
0x6F12, 0x994D,
0x6F12, 0x97F8,
0x6F12, 0xA800,
0x6F12, 0x4000,
0x6F12, 0x40F4,
0x6F12, 0x0070,
0x6F12, 0xA5F8,
0x6F12, 0xB201,
0x6F12, 0xDFF8,
0x6F12, 0x44A2,
0x6F12, 0x6B46,
0x6F12, 0x3246,
0x6F12, 0xBAF8,
0x6F12, 0xEC11,
0x6F12, 0x4FF2,
0x6F12, 0xB010,
0x6F12, 0x05F5,
0x6F12, 0xC275,
0x6F12, 0x9E37,
0x6F12, 0x00F0,
0x6F12, 0x7AF9,
0x6F12, 0x0E20,
0x6F12, 0xA077,
0x6F12, 0xA885,
0x6F12, 0xB97A,
0x6F12, 0x4FF4,
0x6F12, 0x0078,
0x6F12, 0x48EA,
0x6F12, 0x4100,
0x6F12, 0xE887,
0x6F12, 0xB988,
0x6F12, 0x6B46,
0x6F12, 0x3246,
0x6F12, 0x4FF2,
0x6F12, 0xC010,
0x6F12, 0x00F0,
0x6F12, 0x6AF9,
0x6F12, 0x0026,
0x6F12, 0xA677,
0x6F12, 0xAE87,
0x6F12, 0x25F8,
0x6F12, 0x128C,
0x6F12, 0x4FF0,
0x6F12, 0x0109,
0x6F12, 0x84F8,
0x6F12, 0x1D90,
0x6F12, 0x84F8,
0x6F12, 0x1F90,
0x6F12, 0xBAF8,
0x6F12, 0xEA21,
0x6F12, 0x0198,
0x6F12, 0x3988,
0x6F12, 0x4243,
0x6F12, 0x6B46,
0x6F12, 0x4FF2,
0x6F12, 0x7010,
0x6F12, 0x00F0,
0x6F12, 0x55F9,
0x6F12, 0x25F8,
0x6F12, 0x149C,
0x6F12, 0x7C4C,
0x6F12, 0x7D48,
0x6F12, 0x84F8,
0x6F12, 0xCE6C,
0x6F12, 0x0680,
0x6F12, 0x7C48,
0x6F12, 0x0680,
0x6F12, 0xA5F8,
0x6F12, 0x0E80,
0x6F12, 0x7B4E,
0x6F12, 0x6B46,
0x6F12, 0x2821,
0x6F12, 0x7288,
0x6F12, 0x4FF2,
0x6F12, 0x9010,
0x6F12, 0x00F0,
0x6F12, 0x42F9,
0x6F12, 0x0220,
0x6F12, 0xA881,
0x6F12, 0xA5F8,
0x6F12, 0x1E80,
0x6F12, 0x94F8,
0x6F12, 0xCE0C,
0x6F12, 0xF021,
0x6F12, 0x0201,
0x6F12, 0x4BF2,
0x6F12, 0x0600,
0x6F12, 0x00F0,
0x6F12, 0x22F9,
0x6F12, 0xA4F5,
0x6F12, 0x8045,
0x6F12, 0x2878,
0x6F12, 0xE8B1,
0x6F12, 0x94F8,
0x6F12, 0xCE0C,
0x6F12, 0xF021,
0x6F12, 0x0201,
0x6F12, 0x4BF2,
0x6F12, 0x0800,
0x6F12, 0x04F6,
0x6F12, 0xCE44,
0x6F12, 0x00F0,
0x6F12, 0x14F9,
0x6F12, 0x2078,
0x6F12, 0xF021,
0x6F12, 0x0201,
0x6F12, 0x4BF2,
0x6F12, 0x0A00,
0x6F12, 0x00F0,
0x6F12, 0x0DF9,
0x6F12, 0xA079,
0x6F12, 0xE179,
0x6F12, 0x0001,
0x6F12, 0x00E0,
0x6F12, 0x0EE0,
0x6F12, 0x40EA,
0x6F12, 0x0130,
0x6F12, 0xB5F8,
0x6F12, 0xDC12,
0x6F12, 0x0843,
0x6F12, 0x6249,
0x6F12, 0xA1F8,
0x6F12, 0x0601,
0x6F12, 0x6248,
0x6F12, 0x7288,
0x6F12, 0xB0F8,
0x6F12, 0xF412,
0x6F12, 0x1144,
0x6F12, 0xA0F8,
0x6F12, 0xF412,
0x6F12, 0xBDE8,
0x6F12, 0xFE8F,
0x6F12, 0x10B5,
0x6F12, 0x5D4C,
0x6F12, 0x94F8,
0x6F12, 0xFC10,
0x6F12, 0x94F8,
0x6F12, 0xFA00,
0x6F12, 0x00F0,
0x6F12, 0x08F9,
0x6F12, 0x81B2,
0x6F12, 0xA4F8,
0x6F12, 0x2A12,
0x6F12, 0x8807,
0x6F12, 0x02D0,
0x6F12, 0x0E20,
0x6F12, 0x00F0,
0x6F12, 0x05F9,
0x6F12, 0x0020,
0x6F12, 0xA4F8,
0x6F12, 0x2C02,
0x6F12, 0x94F8,
0x6F12, 0x1612,
0x6F12, 0xA4F8,
0x6F12, 0x2612,
0x6F12, 0xC807,
0x6F12, 0x02D0,
0x6F12, 0x4520,
0x6F12, 0x00F0,
0x6F12, 0xF9F8,
0x6F12, 0x00F0,
0x6F12, 0xFCF8,
0x6F12, 0xA4F8,
0x6F12, 0x2802,
0x6F12, 0x00F0,
0x6F12, 0xDFF8,
0x6F12, 0x0128,
0x6F12, 0x16D1,
0x6F12, 0x4C48,
0x6F12, 0x0078,
0x6F12, 0x98B1,
0x6F12, 0x4848,
0x6F12, 0x94F8,
0x6F12, 0x1622,
0x6F12, 0x4188,
0x6F12, 0x4048,
0x6F12, 0x90F8,
0x6F12, 0xC000,
0x6F12, 0xC140,
0x6F12, 0x1144,
0x6F12, 0xA4F8,
0x6F12, 0x2612,
0x6F12, 0xB4F8,
0x6F12, 0x2812,
0x6F12, 0x4FF6,
0x6F12, 0xFF72,
0x6F12, 0x02EA,
0x6F12, 0x4101,
0x6F12, 0xC140,
0x6F12, 0xA4F8,
0x6F12, 0x2812,
0x6F12, 0xB4F8,
0x6F12, 0x2C32,
0x6F12, 0xB4F8,
0x6F12, 0x2A22,
0x6F12, 0xB4F8,
0x6F12, 0x2812,
0x6F12, 0xB4F8,
0x6F12, 0x2602,
0x6F12, 0xBDE8,
0x6F12, 0x1040,
0x6F12, 0x00F0,
0x6F12, 0xD8B8,
0x6F12, 0x70B5,
0x6F12, 0x3748,
0x6F12, 0x0022,
0x6F12, 0x4068,
0x6F12, 0x86B2,
0x6F12, 0x040C,
0x6F12, 0x3146,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0xD3F8,
0x6F12, 0x00F0,
0x6F12, 0xD6F8,
0x6F12, 0x0546,
0x6F12, 0x0122,
0x6F12, 0x3146,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0xCBF8,
0x6F12, 0x00F0,
0x6F12, 0xA6F8,
0x6F12, 0x0128,
0x6F12, 0x01D0,
0x6F12, 0x2846,
0x6F12, 0x70BD,
0x6F12, 0x2E48,
0x6F12, 0x90F8,
0x6F12, 0xD512,
0x6F12, 0x2D48,
0x6F12, 0x0078,
0x6F12, 0x00B1,
0x6F12, 0x1821,
0x6F12, 0x284C,
0x6F12, 0x6180,
0x6F12, 0x00F0,
0x6F12, 0xC4F8,
0x6F12, 0x6188,
0x6F12, 0xC140,
0x6F12, 0x4FF6,
0x6F12, 0xF870,
0x6F12, 0xC91D,
0x6F12, 0x0140,
0x6F12, 0x1D48,
0x6F12, 0x6180,
0x6F12, 0x90F8,
0x6F12, 0xC000,
0x6F12, 0xC140,
0x6F12, 0x4819,
0x6F12, 0x1849,
0x6F12, 0xB1F8,
0x6F12, 0xEA11,
0x6F12, 0x0844,
0x6F12, 0x70BD,
0x6F12, 0x10B5,
0x6F12, 0x0122,
0x6F12, 0xAFF2,
0x6F12, 0xEB21,
0x6F12, 0x2048,
0x6F12, 0x00F0,
0x6F12, 0xB1F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x4B21,
0x6F12, 0x1E48,
0x6F12, 0x00F0,
0x6F12, 0xABF8,
0x6F12, 0xAFF2,
0x6F12, 0x4320,
0x6F12, 0x1C49,
0x6F12, 0x0022,
0x6F12, 0x8864,
0x6F12, 0xAFF2,
0x6F12, 0x1B11,
0x6F12, 0x1B48,
0x6F12, 0x00F0,
0x6F12, 0xA1F8,
0x6F12, 0x0122,
0x6F12, 0xAFF2,
0x6F12, 0x9901,
0x6F12, 0x1948,
0x6F12, 0x00F0,
0x6F12, 0x9BF8,
0x6F12, 0x0F49,
0x6F12, 0x4860,
0x6F12, 0x10BD,
0x6F12, 0x0000,
0x6F12, 0x2000,
0x6F12, 0xED50,
0x6F12, 0x2000,
0x6F12, 0x61F0,
0x6F12, 0x2000,
0x6F12, 0x2390,
0x6F12, 0x2000,
0x6F12, 0x3A70,
0x6F12, 0x2000,
0x6F12, 0xBA00,
0x6F12, 0x2000,
0x6F12, 0xED60,
0x6F12, 0x2001,
0x6F12, 0xF000,
0x6F12, 0x2000,
0x6F12, 0x9420,
0x6F12, 0x2001,
0x6F12, 0x45B0,
0x6F12, 0x2000,
0x6F12, 0x12F0,
0x6F12, 0x4000,
0x6F12, 0xF000,
0x6F12, 0x2000,
0x6F12, 0xDA00,
0x6F12, 0x4000,
0x6F12, 0xF4F2,
0x6F12, 0x4000,
0x6F12, 0xB40C,
0x6F12, 0x2001,
0x6F12, 0x5620,
0x6F12, 0x4000,
0x6F12, 0xE000,
0x6F12, 0x2000,
0x6F12, 0x68C0,
0x6F12, 0x2000,
0x6F12, 0x9A00,
0x6F12, 0x0001,
0x6F12, 0x7E15,
0x6F12, 0x0000,
0x6F12, 0x6D83,
0x6F12, 0x2000,
0x6F12, 0x5D90,
0x6F12, 0x0000,
0x6F12, 0x0ACB,
0x6F12, 0x0000,
0x6F12, 0xA89B,
0x6F12, 0x41F2,
0x6F12, 0x331C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F2,
0x6F12, 0xE53C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x48F2,
0x6F12, 0x554C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x40F2,
0x6F12, 0x6D7C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x47F6,
0x6F12, 0x975C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x40F2,
0x6F12, 0x476C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x40F6,
0x6F12, 0xBF6C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x47F6,
0x6F12, 0xC55C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x43F6,
0x6F12, 0xBB1C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x48F2,
0x6F12, 0x551C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x45F2,
0x6F12, 0xC70C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x40F6,
0x6F12, 0x692C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F2,
0x6F12, 0x8F6C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0x451C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x44F2,
0x6F12, 0x3F6C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x42F2,
0x6F12, 0x113C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F2,
0x6F12, 0xC36C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x43F2,
0x6F12, 0x374C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x48F2,
0x6F12, 0x391C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x4AF6,
0x6F12, 0x9B0C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0x290C,
0x6F12, 0xC0F2,
0x6F12, 0x020C,
0x6F12, 0x6047,
0x6F12, 0x4BF6,
0x6F12, 0x152C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x0841,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x00B0,
0x602A, 0xF1EA,
0x6F12, 0x0008,
0x6F12, 0x13C0,
0x602A, 0xF008,
0x6F12, 0x0000,

0x6028, 0x2000,  //Global
0x602A, 0x2500,
0x6F12, 0x0080,
0x602A, 0x10B8,
0x6F12, 0x0020,
0x602A, 0x1EE0,
0x6F12, 0x0078,
0x602A, 0x2870,
0x6F12, 0x0100,
0x602A, 0x250A,
0x6F12, 0x0000,
0x602A, 0x23A0,
0x6F12, 0x0001,
0x602A, 0x3022,
0x6F12, 0x1281,
0x602A, 0x32E8,
0x6F12, 0x0100,
0x602A, 0x54A2,
0x6F12, 0x0000,
0x602A, 0x120E,
0x6F12, 0x0000,
0x602A, 0x1212,
0x6F12, 0x0000,
0x602A, 0x2860,
0x6F12, 0x0001,
0x602A, 0x3220,
0x6F12, 0x0000,
0x602A, 0x1226,
0x6F12, 0x0301,
0x602A, 0x29C8,
0x6F12, 0x0000,
0x602A, 0x32EC,
0x6F12, 0x0000,
0x602A, 0x12BE,
0x6F12, 0x0101,
0x602A, 0x3034,
0x6F12, 0x049B,
0x602A, 0x1230,
0x6F12, 0x0100,
0x602A, 0x1232,
0x6F12, 0x00F0,
0x602A, 0x1236,
0x6F12, 0x01FF,
0x602A, 0x123A,
0x6F12, 0x0004,
0x602A, 0x123E,
0x6F12, 0xF45A,
0x602A, 0x1EE2,
0x6F12, 0x19CD,
0x602A, 0x115E,
0x6F12, 0x0048,
0x602A, 0x131C,
0x6F12, 0x2400,
0x602A, 0x2872,
0x6F12, 0x0001,
0x602A, 0x1314,
0x6F12, 0x0100,
0x602A, 0x20DE,
0x6F12, 0x0003,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x602A, 0x2108,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x6F12, 0x0022,
0x6F12, 0x0011,
0x602A, 0x1EDC,
0x6F12, 0x5008,
0x602A, 0x138E,
0x6F12, 0x13C0,
0x602A, 0x1392,
0x6F12, 0x0038,
0x602A, 0x21B6,
0x6F12, 0x0002,
0x6F12, 0x0000,
0x602A, 0x2550,
0x6F12, 0x193C,
0x6028, 0x4000,
0x0BC0, 0x0040,
0x0FE8, 0x49C1,
0x0FEA, 0x0040,
0x0BC8, 0x0001,
0x0B0A, 0x0101,
0x0BC6, 0x0000,
0x0B06, 0x0101,
0xF446, 0x000C,
0xF448, 0x0018,
0xF450, 0x0010,
0xF44E, 0x0000,
0xF468, 0xE000,

0x6028, 0x2000,  //20190121 add for remosaic 
0x602A, 0x3778,
0x6F12, 0x0100,
0x602A, 0x37FC,
0x6F12, 0x0000,
0x602A, 0x4BFC,
0x6F12, 0xD2D2,
0x6F12, 0xD2D2,
0x6F12, 0xD2D2,
0x602A, 0x465C,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x602A, 0x4652,
0x6F12, 0x1023,
0x6F12, 0x2323,
0x6F12, 0x2323,
0x6F12, 0x2300,
0x602A, 0x466E,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x602A, 0x469A,
0x6F12, 0x1014,
0x6F12, 0x1414,
0x6F12, 0x1414,
0x6F12, 0x1400,
0x602A, 0x46AC,
0x6F12, 0x1013,
0x6F12, 0x1313,
0x6F12, 0x1313,
0x6F12, 0x1300,
0x602A, 0x4676,
0x6F12, 0x100A,
0x6F12, 0x0A0A,
0x6F12, 0x0A0A,
0x6F12, 0x0A00,
0x602A, 0x4688,
0x6F12, 0x101D,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x6F12, 0x1D00,
0x602A, 0x4C0E,
0x6F12, 0x7878,
0x6F12, 0x7878,
0x6F12, 0x7878,
0x602A, 0x3B1E,
0x6F12, 0x008C,
0x602A, 0x4C20,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x6F12, 0x1D1D,
0x602A, 0x3B12,
0x6F12, 0x0002,
0x602A, 0x3AF2,
0x6F12, 0x0002,
0x602A, 0x3AF6,
0x6F12, 0x0005,
0x602A, 0x3AFA,
0x6F12, 0x0007,
0x602A, 0x3AFE,
0x6F12, 0x0064,
0x602A, 0x3B02,
0x6F12, 0x00AF,
0x602A, 0x3B06,
0x6F12, 0x00C8,
0x602A, 0x46BE,
0x6F12, 0x10D4,
0x6F12, 0xD4D4,
0x6F12, 0xD4D4,
0x6F12, 0xD400,
0x602A, 0x46C8,
0x6F12, 0xFAFA,
0x6F12, 0xFAFA,
0x6F12, 0xFAFA,
0x602A, 0x3B2E,
0x6F12, 0x0008,
0x602A, 0x3B32,
0x6F12, 0x0070,
0x602A, 0x4C28,
0x6F12, 0x1033,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x6F12, 0x3300,
0x602A, 0x4C32,
0x6F12, 0x1919,
0x6F12, 0x1919,
0x6F12, 0x1919,
0x602A, 0x4C3A,
0x6F12, 0x10CC,
0x6F12, 0xCCCC,
0x6F12, 0xCCCC,
0x6F12, 0xCC00,
0x602A, 0x4C44,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x6F12, 0x3333,
0x602A, 0x4C4C,
0x6F12, 0x1066,
0x6F12, 0x6666,
0x6F12, 0x6666,
0x6F12, 0x6600,
0x602A, 0x4C56,
0x6F12, 0x2222,
0x6F12, 0x2222,
0x6F12, 0x2222,
0x602A, 0x3E06,
0x6F12, 0x0000,
0x602A, 0x3E0A,
0x6F12, 0x0000,
0x602A, 0x3E2E,
0x6F12, 0x0060,
0x602A, 0x37FE,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x6F12, 0x0001,
0x602A, 0x3E0E,
0x6F12, 0x0019,
0x602A, 0x3E12,
0x6F12, 0x00FE,
0x602A, 0x3E16,
0x6F12, 0x0019,
0x602A, 0x3E1A,
0x6F12, 0x00FE,
0x602A, 0x3E1E,
0x6F12, 0x001E,
0x602A, 0x3E22,
0x6F12, 0x00FF,
0x602A, 0x3E26,
0x6F12, 0x0014,
0x602A, 0x3E2A,
0x6F12, 0x00DA,
0x602A, 0x3CB2,
0x6F12, 0x0000,
0x602A, 0x3BA2,
0x6F12, 0x0000,
0x602A, 0x4C5E,
0x6F12, 0x4078,
0x6F12, 0x785E,
0x6F12, 0x4236,
0x6F12, 0x3601,
0x602A, 0x4C68,
0x6F12, 0x7878,
0x6F12, 0x5E42,
0x6F12, 0x3636,
0x602A, 0x4C70,
0x6F12, 0x405A,
0x6F12, 0x5A78,
0x6F12, 0x96B4,
0x6F12, 0xB401,
0x602A, 0x4C7A,
0x6F12, 0x6464,
0x6F12, 0x85A7,
0x602A, 0x4C82,
0x6F12, 0x4053,
0x6F12, 0x5370,
0x6F12, 0x8BA7,
0x6F12, 0xA701,
0x602A, 0x4C8C,
0x6F12, 0x5353,
0x6F12, 0x708B,
0x6F12, 0xA7A7,
0x602A, 0x4C94,
0x6F12, 0x4064,
0x6F12, 0x6486,
0x6F12, 0xA7C8,
0x6F12, 0xC801,
0x602A, 0x4C9E,
0x6F12, 0x1414,
0x6F12, 0x1B21,
0x6F12, 0x2828,
0x602A, 0x4CA6,
0x6F12, 0x4014,
0x6F12, 0x141B,
0x6F12, 0x2128,
0x6F12, 0x2801,
0x602A, 0x4CB0,
0x6F12, 0x1B1B,
0x6F12, 0x232D,
0x6F12, 0x3636,
0x602A, 0x4CB8,
0x6F12, 0x403C,
0x6F12, 0x3C50,
0x6F12, 0x6478,
0x6F12, 0x7801,
0x602A, 0x4CC2,
0x6F12, 0x3C3C,
0x6F12, 0x5064,
0x6F12, 0x7878,
0x602A, 0x3DA6,
0x6F12, 0x0035,
0x602A, 0x3DAA,
0x6F12, 0x0028,
0x602A, 0x3DB0,
0x6F12, 0x01AB,
0x6F12, 0x0001,
0x6F12, 0x01AC,
0x6F12, 0x0050,
0x6F12, 0x01AD,
0x6F12, 0x0064,
0x6F12, 0x01AE,
0x6F12, 0x0064,
0x6F12, 0x01AF,
0x6F12, 0x00C8,
0x6F12, 0x01B0,
0x6F12, 0x00C8,
0x602A, 0x3DD4,
0x6F12, 0x01B4,
0x6F12, 0x0032,
0x6F12, 0x01B5,
0x6F12, 0x0050,
0x6F12, 0x01B6,
0x6F12, 0x0050,
0x6F12, 0x01B7,
0x6F12, 0x00C8,
0x6F12, 0x01B8,
0x6F12, 0x00C8,
0x6F12, 0x01B9,
0x6F12, 0x0081,

0x6028, 0x2000,/*TMC*/
0x602A, 0x2554,
0x6F12, 0x0B50,
0x6F12, 0x02DE,
0x6F12, 0x0334,

0x6028, 0x4000, 
0x6B76, 0x0180, /*Tr Tf*/
0x6B5E, 0x4000, /*Tr Tf*/
0x6028, 0x4000, 
0xF44A, 0x001F, 
};	
/*4:3 binning mode*/
static kal_uint16 addr_data_pair_preview[] = {
 // 2sum2Ave mode 3264 x 2448
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0018,
0x0346, 0x0010,
0x0348, 0x1997,
0x034A, 0x133E,
0x034C, 0x0CC0,
0x034E, 0x0990,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x010A,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
};

#if 1/*4:3 hw-remosiac mode*/
static kal_uint16 addr_data_pair_capture[] = {
//fullsize 6528x4896
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0000,
0x602A, 0x1EB8,
0x6F12, 0x0300,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0200,
0x6F12, 0x0098,
0x602A, 0x11AE,
0x6F12, 0x0088,
0x602A, 0x13FC,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0101,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0101,
0x602A, 0x1EC8,
0x6F12, 0x0603,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0004,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0FFD,
0x0328, 0x0100,
0x0344, 0x0018,
0x0346, 0x0010,
0x0348, 0x1997,
0x034A, 0x133F,
0x034C, 0x1980,
0x034E, 0x1320,
0x0350, 0x0000,
0x0352, 0x0008,
0x0900, 0x0111,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0001,
0x0386, 0x0001,
0x0400, 0x1010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00E9,
0x0312, 0x0001,
0x0340, 0x15B4,
0x0342, 0x35A0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0001,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
};
#endif
/*16:9 binning mode*/
static kal_uint16 addr_data_pair_normal_video[] = {
// 2Sum2Ave 3264 x 1840
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0018,
0x0346, 0x0270,
0x0348, 0x1997,
0x034A, 0x10DE,
0x034C, 0x0CC0,
0x034E, 0x0730,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00E9,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,

};

//addr_data_pair_hs_video  for face mode
static kal_uint16 addr_data_pair_hs_video[] = {
//1632x1224 
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0104,
0x602A, 0x1ED2,
0x6F12, 0x0807,
0x602A, 0x1ED6,
0x6F12, 0x0207,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0008,
0x0346, 0x0010,
0x0348, 0x19A7,
0x034A, 0x133F,
0x034C, 0x0660,
0x034E, 0x04C8,
0x0350, 0x0000,
0x0352, 0x0002,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x2000,
0x0402, 0x1020,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x010A,
0x0312, 0x0003,
0x0340, 0x0ABA,
0x0342, 0x3638,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
};

static kal_uint16 addr_data_pair_slim_video[] = {
};
/*4:3 3d-hdr binning mode */
static kal_uint16 addr_data_pair_custom1[] = {
 // 2sum2Ave mode 3264 x 2448
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0018,
0x0346, 0x0010,
0x0348, 0x1997,
0x034A, 0x133E,
0x034C, 0x0CC0,
0x034E, 0x0990,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x010A,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
};
/*16:9 3d-hdr binning mode */
static kal_uint16 addr_data_pair_custom2[] = {
// 2Sum2Ave 3264 x 1840
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0504,
0x602A, 0x1ED2,
0x6F12, 0x080F,
0x602A, 0x1ED6,
0x6F12, 0x0307,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0018,
0x0346, 0x0270,
0x0348, 0x1997,
0x034A, 0x10DE,
0x034C, 0x0CC0,
0x034E, 0x0730,
0x0350, 0x0000,
0x0352, 0x0004,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x1000,
0x0402, 0x1010,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x00E9,
0x0312, 0x0002,
0x0340, 0x0A40,
0x0342, 0x38C0,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
	};
	
//SUM 1632 X 1224 30fps
static kal_uint16 addr_data_pair_custom3[] = {
//1632x1224 
0x6028, 0x4000,
0x6214, 0xF9F0,
0x6218, 0xE150,
0x6242, 0x0E00,
0x6028, 0x2000,
0x602A, 0x12F2,
0x6F12, 0x0D10,
0x6F12, 0x0A18,
0x6F12, 0x19B0,
0x6F12, 0x1350,
0x602A, 0x1EB6,
0x6F12, 0x0206,
0x602A, 0x3770,
0x6F12, 0x0100,
0x602A, 0x1EB8,
0x6F12, 0x0301,
0x602A, 0x131E,
0x6F12, 0x0100,
0x602A, 0x3DEA,
0x6F12, 0x0081,
0x602A, 0x11A6,
0x6F12, 0x0000,
0x6F12, 0x0004,
0x602A, 0x11AE,
0x6F12, 0x0003,
0x602A, 0x13FC,
0x6F12, 0x0044,
0x6F12, 0x0064,
0x6F12, 0x0044,
0x602A, 0x3302,
0x6F12, 0x0100,
0x6F12, 0x0100,
0x6F12, 0x0001,
0x602A, 0x27D2,
0x6F12, 0x0203,
0x602A, 0x1EC8,
0x6F12, 0x0503,
0x6F12, 0x0104,
0x602A, 0x1ED2,
0x6F12, 0x0807,
0x602A, 0x1ED6,
0x6F12, 0x0207,
0x602A, 0x123C,
0x6F12, 0x0009,
0x602A, 0x21BE,
0x6F12, 0x04D2,
0x6F12, 0x41A6,
0x602A, 0x1EE0,
0x6F12, 0x006C,
0x602A, 0x145C,
0x6F12, 0x0035,
0x6F12, 0x0049,
0x6F12, 0x0035,
0x602A, 0x140E,
0x6F12, 0x0000,
0x6F12, 0x0001,
0x6F12, 0x0000,
0x602A, 0x1224,
0x6F12, 0x014B,
0x6028, 0x4000,
0xF466, 0x0E0D,
0x0328, 0x0100,
0x0344, 0x0008,
0x0346, 0x0010,
0x0348, 0x19A7,
0x034A, 0x133F,
0x034C, 0x0660,
0x034E, 0x04C8,
0x0350, 0x0000,
0x0352, 0x0002,
0x0900, 0x0112,
0x0380, 0x0001,
0x0382, 0x0001,
0x0384, 0x0002,
0x0386, 0x0002,
0x0400, 0x2010,
0x0404, 0x2000,
0x0402, 0x1020,
0x0114, 0x0300,
0x0116, 0x3000,
0x0110, 0x1002,
0x011C, 0x0100,
0x0136, 0x1A00,
0x0300, 0x0002,
0x0302, 0x0003,
0x0304, 0x0004,
0x0306, 0x0108,
0x0308, 0x0008,
0x030A, 0x0002,
0x030C, 0x0000,
0x030E, 0x0004,
0x0310, 0x010A,
0x0312, 0x0003,
0x0340, 0x0ABA,
0x0342, 0x3638,
0x0202, 0x0100,
0x0200, 0x0100,
0x022C, 0x0100,
0x0226, 0x0100,
0x021E, 0x0000,
0x6028, 0x2000,
0x602A, 0x3020,
0x6F12, 0x0000,
0x6028, 0x4000,
0x0B00, 0x0080,
0x0B08, 0x0000,
0x0D00, 0x0000,
0x0D02, 0x0000,
0x0D04, 0x0000,
	};	
static void sensor_init(void)
{
	/*Global setting */
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x0000, 0x0010);
	write_cmos_sensor_16_16(0x0000, 0x0841);
	write_cmos_sensor_16_16(0x6010, 0x0001);
	mdelay(30);
	write_cmos_sensor_16_16(0x6214, 0xE9F0);
	write_cmos_sensor_16_16(0x6218, 0xE150); 
	write_cmos_sensor_16_16(0x6242, 0x0E00);

	table_write_cmos_sensor(addr_data_pair_init_otp,
			   sizeof(addr_data_pair_init_otp) / sizeof(kal_uint16));
}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	LOG_INF("4:3 binning size end\n");
}	/*	preview_setting  */

/* Pll Setting - VCO = 280Mhz*/

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("full size start\n");
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	LOG_INF("full size  end\n");
}
static void normal_video_setting(void)
{
	LOG_INF("16:9 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	LOG_INF("16:9 binning size end\n");
}
static void hs_video_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	LOG_INF("4:3 HDR binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	LOG_INF("4:3 HDR binning size end\n");
}

static void custom2_setting(void)
{
	LOG_INF("16:9 HDR binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	LOG_INF("16:9 HDR binning size end\n");
}

static void custom3_setting(void)
{
	LOG_INF("4:3 1632X1224 size start\n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	LOG_INF("4:3 1632X1224 size end\n");
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	int I2C_BUS = -1;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	LOG_INF("s5kgd1sppd1913mipiraw_Sensor I2C_BUS = %d\n", I2C_BUS);	
	if(I2C_BUS != 4){
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = read_cmos_sensor_16_8(0x0000) << 8 | read_cmos_sensor_16_8(0x0001);
			LOG_INF("read out sensor id 0x%x \n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				/*vivo hope  add for CameraEM otp errorcode*/
				LOG_INF("cfx_add:start read eeprom ---vivo_s5kgd1sppd1913otp_read_when_power_on = %d\n", vivo_s5kgd1sppd1913otp_read_when_power_on);
				vivo_s5kgd1sppd1913otp_read_when_power_on = s5kgd1sppd1913_otp_read();
				LOG_INF("cfx_add:end read eeprom ---vivo_s5kgd1sppd1913otp_read_when_power_on = %d,S5KGD1SPPD1913_OTP_ERROR_CODE =%d\n", vivo_s5kgd1sppd1913otp_read_when_power_on, S5KGD1SPPD1913_OTP_ERROR_CODE);
				/*vivo hope  add end*/
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id !=  imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	LOG_INF("PLATFORM:MT6750,MIPI 4LANE\n");
	LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = read_cmos_sensor_16_8(0x0000) << 8 | read_cmos_sensor_16_8(0x0001);
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id){
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/* initail sequence write in  */
	sensor_init();
	fps_count = 0;
	/*sensor_temperature[1] = 0;*/
	/*LOG_INF("sensor_temperature[1] = %d\n", sensor_temperature[1]);*/
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/*sensor_temperature[1] = 0;*/
	/*LOG_INF("sensor_temperature[1] = %d\n", sensor_temperature[1]);*/

	/*No Need to implement this function*/

	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E imgsensor.hdr_mode=%d\n", imgsensor.hdr_mode);

	if (imgsensor.hdr_mode) { /* use cutom1 HDR setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		custom1_setting();  /* 4:3 3HDR binning size */
	} else { /* use preview setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
		imgsensor.pclk = imgsensor_info.pre.pclk;
		imgsensor.line_length = imgsensor_info.pre.linelength;
		imgsensor.frame_length = imgsensor_info.pre.framelength;
		imgsensor.min_frame_length = imgsensor_info.pre.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		preview_setting();  /* 4:3 normal binning size */
	}
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);  /*hw-remosiac*/
	
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E imgsensor.hdr_mode=%d\n", imgsensor.hdr_mode);

	if (imgsensor.hdr_mode) { /* use cutom2 HDR setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
		imgsensor.pclk = imgsensor_info.custom2.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		custom2_setting();  /* 16:9 3HDR binning size */
	} else { /* use normal_video setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
		imgsensor.pclk = imgsensor_info.normal_video.pclk;
		imgsensor.line_length = imgsensor_info.normal_video.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		normal_video_setting();  /* 16:9 normal binning size */
	}

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("Don't use, no setting E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E imgsensor.hdr_mode=%d\n", imgsensor.hdr_mode);

	if (imgsensor.hdr_mode) { /* use cutom1 HDR setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		custom1_setting();  /* 4:3 3HDR binning size */
	} else { /* use preview setting */
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
		imgsensor.pclk = imgsensor_info.pre.pclk;
		imgsensor.line_length = imgsensor_info.pre.linelength;
		imgsensor.frame_length = imgsensor_info.pre.framelength;
		imgsensor.min_frame_length = imgsensor_info.pre.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);

		preview_setting();  /* 4:3 normal binning size */
	}

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom1	 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2	 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom3	 */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom4   */

static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom5   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;
	
	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;
	
	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet*/
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;


	
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 0;
	sensor_info->HDR_Support = 4; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR, 4:four-cell mVHDR*/
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x*/
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x*/
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
						imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM4:
			sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
		
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 
				imgsensor_info.custom4.mipi_data_lp2hs_settle_dc; 
		
			break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
	
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc; 
	
		break;

		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = 
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			custom1(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			custom2(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			custom3(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			Custom4(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			Custom5(image_window, sensor_config_data);
			break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0)
		/* Dynamic frame rate*/
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		if (imgsensor.hdr_mode) {
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
						(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
						(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		if (imgsensor.hdr_mode) {
			frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ?
						(frame_length - imgsensor_info.custom2.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10
						/ imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
						(frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					framerate, imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
						(frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
						(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
						(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1: /*same as preview */
		if (imgsensor.hdr_mode) {
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
						(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
						(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ?
					(frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ?
					(frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	/* coding with  preview scenario by default */
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
					(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;	
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;	
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
	    *framerate = imgsensor_info.custom4.max_framerate;
	    break;
	case MSDK_SCENARIO_ID_CUSTOM5:
	    *framerate = imgsensor_info.custom5.max_framerate;
	    break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600, 0x0002);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable*/
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK*/
		write_cmos_sensor_16_16(0x0600,0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
	kal_uint16 realtime_fps = 0;

	LOG_INF("E! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	} else {
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	hdr_le = le;
	hdr_me = me;
	hdr_se = se;
	///* Long exposure */
	//write_cmos_sensor_16_16(0x0226, le);
	///* Middle exposure */
	//write_cmos_sensor_16_16(0x022c, me);
	///* Short exposure */
	//write_cmos_sensor_16_16(0x0202, se);

	LOG_INF("imgsensor.frame_length:0x%x\n", imgsensor.frame_length);
	LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);

}

static kal_uint16 gain2reg_dig(kal_uint16 gain, kal_uint16 g_gain)
{
	kal_uint16 dig_gain = 0x0;

	/* gain's base is 64, dig_gain's base is 256 */
	dig_gain = ((kal_uint32)gain*256/g_gain)&0xffff;
	LOG_INF("gain:%d,g_gain:%d,dig_gain=%d\n", gain, g_gain, dig_gain);
	return (kal_uint16)dig_gain;
}

static void hdr_write_tri_gain(kal_uint16 lg, kal_uint16 mg, kal_uint16 sg)
{
	kal_uint16 reg_lg_dig, reg_mg_dig, reg_sg_dig;
	kal_uint16 global_gain, reg_global_gain;

	/* should use analog(global) gain first */
	/* nosie would be obviously if totally use digital gain */
	global_gain = lg < mg?lg:mg;
	global_gain = global_gain < sg?global_gain:sg;

	if (global_gain < BASEGAIN || global_gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (global_gain < BASEGAIN)
			global_gain = BASEGAIN;
		else if (global_gain > 16 * BASEGAIN)
			global_gain = 16 * BASEGAIN;
	}
	write_cmos_sensor_16_8(0x0104, 0x01);
	reg_global_gain = gain2reg(global_gain);
	reg_lg_dig = gain2reg_dig(lg, global_gain);
	reg_mg_dig = gain2reg_dig(mg, global_gain);
	reg_sg_dig = gain2reg_dig(sg, global_gain);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_global_gain;
	spin_unlock(&imgsensor_drv_lock);
	
	/* Long expo Gian- digital gain, step=256 */
	write_cmos_sensor_16_16(0x0230, reg_lg_dig);
	/* Medium expo Gian */
	write_cmos_sensor_16_16(0x0238, reg_mg_dig);
	/* Short expo Gian */
	write_cmos_sensor_16_16(0x020E, reg_sg_dig);
	/* global gain - step=32 */
	write_cmos_sensor_16_16(0x0204, reg_global_gain);
	
	write_cmos_sensor_16_16(0x0226, hdr_le);
	write_cmos_sensor_16_16(0x022c, hdr_me);
	write_cmos_sensor_16_16(0x0202, hdr_se);
	write_cmos_sensor_16_8(0x0104, 0x00);
	
	LOG_INF("lg:0x%x, reg_lg_dig:0x%x, mg:0x%x, reg_mg_dig:0x%x, sg:0x%x, reg_sg_dig:0x%x\n",
			lg, reg_lg_dig, mg, reg_mg_dig, sg, reg_sg_dig);
	LOG_INF("reg_global_gain:0x%x\n", reg_global_gain);

}


static kal_uint32 set_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32, ggain_32;
	UINT32 BASE_HDR = 2184;
	UINT32 BASE_WB = 256;
	UINT16 reg_rgain, reg_ggain, reg_bgain;

	grgain_32 = pSetSensorAWB->ABS_GAIN_GR;
	rgain_32  = pSetSensorAWB->ABS_GAIN_R;
	bgain_32  = pSetSensorAWB->ABS_GAIN_B;
	gbgain_32 = pSetSensorAWB->ABS_GAIN_GB;
	ggain_32  = (grgain_32 + gbgain_32) >> 1; /*Gr_gain = Gb_gain */
	LOG_INF("[set_awb_gain] rgain:%d, ggain:%d, bgain:%d\n",
				rgain_32, ggain_32, bgain_32);

	/* set WB gain when HDR/remosaic*/
	reg_rgain = (rgain_32*BASE_WB/512)&0xffff;
	reg_ggain = (ggain_32*BASE_WB/512)&0xffff;
	reg_bgain = (bgain_32*BASE_WB/512)&0xffff;
	LOG_INF("[BASE_WB=256] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
					reg_rgain, reg_ggain, reg_bgain);

	write_cmos_sensor_16_16(0x0D82, reg_rgain);
	write_cmos_sensor_16_16(0x0D84, reg_ggain);
	write_cmos_sensor_16_16(0x0D86, reg_bgain);

	/*set weight gain when HDR*/
	if (imgsensor.sensor_mode != IMGSENSOR_MODE_CAPTURE) {
		reg_rgain = (rgain_32*BASE_HDR*5/16/512)&0x1fff;  /*max value is 8192 */
		reg_ggain = (ggain_32*BASE_HDR*9/16/512)&0x1fff;
		reg_bgain = (bgain_32*BASE_HDR*2/16/512)&0x1fff;
		LOG_INF("[BASE_HDR=2184] reg_rgain:%d, reg_ggain:%d, reg_bgain:%d\n",
					reg_rgain, reg_ggain, reg_bgain);

		write_cmos_sensor_16_16(0x6028, 0x2000);
		write_cmos_sensor_16_16(0x602A, 0x4B9C);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* short expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BAA);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* long expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BB8);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* medium expo's gain */
		write_cmos_sensor_16_16(0x602A, 0x4BC6);
		write_cmos_sensor_16_16(0x6F12, reg_rgain);
		write_cmos_sensor_16_16(0x6F12, reg_ggain);
		write_cmos_sensor_16_16(0x6F12, reg_bgain); /* mixed expo's gain */
	}

	return ERROR_NONE;
}
#if 0
static kal_uint32 get_sensor_temperature(void)
{
	kal_uint32	TMC_000A_Value=0,TMC_Value =0; 
	
	write_cmos_sensor_16_16(0x6028, 0x4000);
	TMC_000A_Value=read_cmos_sensor_16_16(0x000A);
	/*LOG_INF("GD1 TMC 0X000A Value=0x%x\n", TMC_000A_Value);*/
	if (TMC_000A_Value >0x0000 && TMC_000A_Value<0x5600){
	  	/*LOG_INF("GD1 TMC is available \n");*/
	}
	else{
		 LOG_INF("GD1 TMC is not available, the sensor is not calibration \n");
	}
	TMC_Value=(TMC_000A_Value>>8)&0xFF;
	/*sensor_temperature[1] = TMC_Value;*/
	/*LOG_INF("GD1 TMC 0X000A temperature Value=0x%x, sensor_temperature[1] =%d\n", TMC_Value, sensor_temperature[1]);*/
	
	return TMC_Value;
}
#endif
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/*INT32 *feature_return_para_i32 = (INT32 *) feature_para;*/
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (struct SET_SENSOR_AWB_GAIN *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
	             set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	             /*night_mode((BOOL) *feature_data);*/
		break;
	case SENSOR_FEATURE_SET_GAIN:
	             set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE*/
		/* if EEPROM does not exist in camera module.*/
		*feature_return_para_32= LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
	            set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
	             set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
	             get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	/*case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		read_2L9_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		break;*/

	case SENSOR_FEATURE_SET_TEST_PATTERN:
	               set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing*/
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len= 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
	             imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
	    LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data_32);
	    wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,	(void *)&imgsensor_winsize_info[8],	sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,	(void *)&imgsensor_winsize_info[9],	sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				//memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* video & capture use same setting*/
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:	
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
		default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
#if 1	
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	#if 1
	    LOG_INF("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  S5KGD1SPPD1913_OTP_ERROR_CODE:%d \n", *feature_data,S5KGD1SPPD1913_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			LOG_INF("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = S5KGD1SPPD1913_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_sub_s5kgd1sppd1913, sizeof(MUINT32)*13); 
				memcpy( feature_data+10, material_inf_sub_s5kgd1sppd1913, sizeof(MUINT32)*4);
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						LOG_INF("sn_inf_sub_s5kgd1sppd1913[%d]= 0x%x\n", i, sn_inf_sub_s5kgd1sppd1913[i]);
						}
				#endif
			}
				break;
		}
		break;
    #endif		
#endif
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		/**feature_return_para_i32 = get_sensor_temperature();*/
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	
	case SENSOR_FEATURE_GET_PIXEL_RATE:

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;

			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					rate = imgsensor_info.cap.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					if (imgsensor.hdr_mode)
						rate = imgsensor_info.custom2.mipi_pixel_rate;
					else
						rate = imgsensor_info.normal_video.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					rate = imgsensor_info.hs_video.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
					rate = imgsensor_info.slim_video.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM1:
					if (imgsensor.hdr_mode)
						rate = imgsensor_info.custom1.mipi_pixel_rate;
					else
						rate = imgsensor_info.pre.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM2:
					rate = imgsensor_info.custom2.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM3:
					rate = imgsensor_info.custom3.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM4:
					rate = imgsensor_info.custom4.mipi_pixel_rate;
					break;
			case MSDK_SCENARIO_ID_CUSTOM5:
					rate = imgsensor_info.custom5.mipi_pixel_rate;
					break;
			default:
					rate = 0;
					break;
			}
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
	}
	break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("hdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, SE=%d, ME=%d\n",(UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
		hdr_write_tri_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",(UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
		hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		set_awb_gain(pSetSensorAWB);
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		/*
		  * SENSOR_VHDR_MODE_NONE  = 0x0,
		  * SENSOR_VHDR_MODE_IVHDR = 0x01,
		  * SENSOR_VHDR_MODE_MVHDR = 0x02,
		  * SENSOR_VHDR_MODE_ZVHDR = 0x09
		  * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
		*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x02;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n", *feature_data, *(feature_data+1));
		break;

	default:
	    break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 S5KGD1SPPD1913_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!= NULL)
		*pfFunc= &sensor_func;
	return ERROR_NONE;
}	/*	S5KGD1SPPD1913_MIPI_RAW_SensorInit	*/
