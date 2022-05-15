/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 ov8856pd2078mipiraw_Sensor.c
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


#define PFX "MAIN2[885A]_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ov8856pd2078mipiraw_Sensor.h"
#include "../imgsensor_common.h"

#include "ov8856pd2078mipiraw_freq.h"
#include "../imgsensor_sensor.h"

/*#define PK_DBG(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)*/
 
#undef CONFIG_MTK_CAM_TD1909

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV8856PD2078_SENSOR_ID,
	.checksum_value = 0xb1893b4f, /*checksum value for Camera Auto Test*/

	.pre = {
		.pclk = 143870000,	/*record different mode's pclk*/
		.linelength  = 1932,	/*record different mode's linelength*/
		.framelength = 2482,	/*record different mode's framelength*/
		.startx = 0, /*record different mode's startx of grabwindow*/
		.starty = 0,	/*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width  = 3264,

		/*record different mode's height of grabwindow*/
		.grabwindow_height = 2448,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * nby different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
	},
	.cap = {
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
	},
	.normal_video = {
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
	},
	.hs_video = { /*hs_video 120fps*/
		.pclk = 143870000, /*record different mode's pclk*/
		.linelength  = 1932, /*record different mode's linelength*/
		.framelength = 1242, /*record different mode's framelength*/
		.startx = 0, /*record different mode's startx of grabwindow*/
		.starty = 0, /*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width  = 1920,
		/*record different mode's height of grabwindow*/
		.grabwindow_height = 1080,

		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 152530000,
		},
	.slim_video = { /* hs_video 240fps*/
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
		},
	.custom1 = { /*user for dualcam*/
		.pclk = 143870000, /*record different mode's pclk*/
		.linelength  = 1932, /*record different mode's linelength*/
		.framelength = 2482, /*record different mode's framelength*/
		.startx = 0, /*record different mode's startx of grabwindow*/
		.starty = 0, /*record different mode's starty of grabwindow*/

		/*record different mode's width of grabwindow*/
		.grabwindow_width  = 2080,
		/*record different mode's height of grabwindow*/
		.grabwindow_height = 1560,

		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
		 * by different scenario
		 */
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
		.mipi_pixel_rate = 162930000,
		},
	.custom2 = { /*back setting*/
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
		},
	 .custom3 = { /* 4608*2592@60fps*/
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
		},
	.custom4 = { /* hs_video 480fps*/
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
	},
	.custom5 = {
		.pclk = 143870000,
		.linelength  = 1932,
		.framelength = 2482,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 273870000,
	},
   
		.margin = 8,
		.min_shutter = 6,
		.min_gain = 64,
		.max_gain = 992,
		.min_gain_iso = 100,
		.gain_step = 4,
		.gain_type = 1,
		.max_frame_length = 0x90f7,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.frame_time_delay_frame = 2,	/*sony sensor must be 3,non-sony sensor must be 2 , The delay frame of setting frame length  */
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/
		.sensor_mode_num =10,

	.cap_delay_frame = 3,		/*enter capture delay frame num*/
	.pre_delay_frame = 3,		/*enter preview delay frame num*/
	.video_delay_frame = 3,		/*enter video delay frame num*/
	.hs_video_delay_frame = 3, /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 3,/*enter slim video delay frame num*/
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_4MA, /*mclk driving current*/

	/*Sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,

	/*sensor output first pixel color*/
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,

	.mclk = 26,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_2_LANE,/*mipi lane num*/

/*record sensor support all write id addr, only supprt 4must end with 0xff*/
	.i2c_addr_table = {0x6c, 0xff},
	.i2c_speed = 400, // i2c read/write speed
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x4C00,			/*current shutter*/
	.gain = 0x200,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 30,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,

	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x6c,
	.current_ae_effective_frame = 1,
	.freq_setting = 0,
  	.present_freq_setting = 0,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // Preview	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // capture	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // video		
	{ 3264, 2448, 	672,   684,	 1920, 1080, 1920, 1080, 0, 0, 1920, 1080,  0,  0, 1920, 1080}, //hight speed video	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // slime video	  
	{ 3264, 2448, 	592,	444, 2080, 1560, 2080, 1560, 0, 0, 2080, 1560,  0,  0, 2080, 1560}, // custom1	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // custom2	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // custom3
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // custom4
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // custom5

};


/*hope add otp check start*/
static int vivo_otp_read_when_power_on;
extern int MAIN2_885A_otp_read(void);
extern otp_error_code_t OV8856PD2078_OTP_ERROR_CODE;
MUINT32  sn_inf_main2_ov8856pd2078[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main2_ov8856pd2078[4];  
/*hope add otp check end*/

#if 0
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
#endif

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

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
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
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}

#if MULTI_WRITE
	if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 3, imgsensor_info.i2c_speed);
			tosend = 0;
	}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif

	}
	return 0;
}



static void set_dummy(void)
{
	if (imgsensor.frame_length%2 != 0)
		imgsensor.frame_length =
			imgsensor.frame_length - imgsensor.frame_length%2;

	pr_debug("imgsensor.frame_length = %d\n", imgsensor.frame_length);
	write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_16_8(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor_16_8(0x380d, imgsensor.line_length & 0xFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	PK_DBG("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

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

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_16_8(0x0100, 0X01);
	else
		write_cmos_sensor_16_8(0x0100, 0x00);
	return ERROR_NONE;
}




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
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK
	 * to get exposure larger than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode,
	 * thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution*/
/* if shutter bigger than frame_length, should extend frame length first*/

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		 ? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter =
		    (imgsensor_info.max_frame_length - imgsensor_info.margin);

	imgsensor.frame_length =
		imgsensor.frame_length - imgsensor.frame_length%2;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
			write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x380f,imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
	}

	if(shutter > 65528) {  //linetime=10160/960000000<< maxshutter=3023622-line=32s
		/*enter long exposure mode */
		kal_uint32 new_line_length;
		PK_DBG("enter long exposure mode\n");
		PK_DBG("Calc long exposure  +\n");
				
		PK_DBG("enter long exposure mode shutter = %d, imgsensor.line_length =%d\n", shutter, imgsensor.line_length);
		
		new_line_length = shutter * imgsensor.line_length / 65528 ;
		shutter = 65528;
		write_cmos_sensor_16_8(0x380C, (new_line_length >> 8)& 0xFF);
		write_cmos_sensor_16_8(0x380D, new_line_length & 0xFF);
		write_cmos_sensor_16_8(0x380e, 0xFF);
		write_cmos_sensor_16_8(0x380f, 0xFF);
		write_cmos_sensor_16_8(0x3502, (shutter << 4) & 0xFF);
		write_cmos_sensor_16_8(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor_16_8(0x3500, (shutter >> 12) & 0x0F);
		
		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 1;
		
		PK_DBG("long exposure  stream on-\n");
		PK_DBG("Exit! shutter =%d, framelength =%d\n",shutter, imgsensor.frame_length);
		}else{
		/* Update Shutter*/
		write_cmos_sensor_16_8(0x380C, (imgsensor.line_length >> 8)& 0xFF);
		write_cmos_sensor_16_8(0x380D, imgsensor.line_length & 0xFF);		
		write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_16_8(0x3502, (shutter << 4) & 0xFF);
		write_cmos_sensor_16_8(0x3501, (shutter >> 4) & 0xFF);
		write_cmos_sensor_16_8(0x3500, (shutter >> 12) & 0x0F);

		pr_debug("Exit! shutter =%d, framelength =%d\n",shutter, imgsensor.frame_length);
}

	
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter =
		    (imgsensor_info.max_frame_length - imgsensor_info.margin);

	imgsensor.frame_length =
			imgsensor.frame_length - imgsensor.frame_length%2;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
			write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x380f,
					  imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_8(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter*/
	write_cmos_sensor_16_8(0x3502, (shutter << 4) & 0xFF);
	write_cmos_sensor_16_8(0x3501, (shutter >> 4) & 0xFF);
	write_cmos_sensor_16_8(0x3500, (shutter >> 12) & 0x0F);

	pr_debug("Add for N3D! shutterlzl =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

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

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_debug("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain*2;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_8(0x3508, (reg_gain>>8));
	write_cmos_sensor_16_8(0x3509, (reg_gain&0xFF));
	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_16_8(0x3820, ((read_cmos_sensor_16_8(0x3820) & 0xB9) | 0x00));
		write_cmos_sensor_16_8(0x3821, ((read_cmos_sensor_16_8(0x3821) & 0xF9) | 0x06));
		write_cmos_sensor_16_8(0x502e, ((read_cmos_sensor_16_8(0x502e) & 0xFC) | 0x03));
		write_cmos_sensor_16_8(0x5001, ((read_cmos_sensor_16_8(0x5001) & 0xFB) | 0x00));
		write_cmos_sensor_16_8(0x5004, ((read_cmos_sensor_16_8(0x5004) & 0xFB) | 0x04));
		write_cmos_sensor_16_8(0x376b, 0x30);
	
	break;

	case IMAGE_H_MIRROR:
		write_cmos_sensor_16_8(0x3820, ((read_cmos_sensor_16_8(0x3820) & 0xB9) | 0x00));
		write_cmos_sensor_16_8(0x3821, ((read_cmos_sensor_16_8(0x3821) & 0xF9) | 0x00));
		write_cmos_sensor_16_8(0x502e, ((read_cmos_sensor_16_8(0x502e) & 0xFC) | 0x03));
		write_cmos_sensor_16_8(0x5001, ((read_cmos_sensor_16_8(0x5001) & 0xFB) | 0x00));
		write_cmos_sensor_16_8(0x5004, ((read_cmos_sensor_16_8(0x5004) & 0xFB) | 0x00));
		write_cmos_sensor_16_8(0x376b, 0x30);
	break;

	case IMAGE_V_MIRROR:
		write_cmos_sensor_16_8(0x3820, ((read_cmos_sensor_16_8(0x3820) & 0xB9) | 0x46));
		write_cmos_sensor_16_8(0x3821, ((read_cmos_sensor_16_8(0x3821) & 0xF9) | 0x06));
		write_cmos_sensor_16_8(0x502e, ((read_cmos_sensor_16_8(0x502e) & 0xFC) | 0x00));
		write_cmos_sensor_16_8(0x5001, ((read_cmos_sensor_16_8(0x5001) & 0xFB) | 0x04));
		write_cmos_sensor_16_8(0x5004, ((read_cmos_sensor_16_8(0x5004) & 0xFB) | 0x04));
		write_cmos_sensor_16_8(0x376b, 0x36);
	break;

	case IMAGE_HV_MIRROR:
		write_cmos_sensor_16_8(0x3820, ((read_cmos_sensor_16_8(0x3820) & 0xB9) | 0x46));
		write_cmos_sensor_16_8(0x3821, ((read_cmos_sensor_16_8(0x3821) & 0xF9) | 0x00));
		write_cmos_sensor_16_8(0x502e, ((read_cmos_sensor_16_8(0x502e) & 0xFC) | 0x00));
		write_cmos_sensor_16_8(0x5001, ((read_cmos_sensor_16_8(0x5001) & 0xFB) | 0x04));
		write_cmos_sensor_16_8(0x5004, ((read_cmos_sensor_16_8(0x5004) & 0xFB) | 0x00));
		write_cmos_sensor_16_8(0x376b, 0x36);
	break;

	default:
			pr_debug("Error image_mirror setting\n");
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

static kal_uint16 addr_data_pair_init[] = {
	0x0103, 0x01,
	0x0300, 0x01,
	0x0302, 0x4f,
	0x0303, 0x00,
	0x030b, 0x04,
	0x030d, 0x53,
	0x031e, 0x0c,
	0x3000, 0x00,
	0x300e, 0x00,
	0x3010, 0x00,
	0x3015, 0x84,
	0x3018, 0x32,
	0x3021, 0x23,
	0x3033, 0x24,
	0x3500, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x3503, 0x08,
	0x3505, 0x83,
	0x3508, 0x01,
	0x3509, 0x80,
	0x350c, 0x00,
	0x350d, 0x80,
	0x350e, 0x04,
	0x350f, 0x00,
	0x3510, 0x00,
	0x3511, 0x02,
	0x3512, 0x00,
	0x3600, 0x72,
	0x3601, 0x40,
	0x3602, 0x30,
	0x3610, 0xc5,
	0x3611, 0x58,
	0x3612, 0x5c,
	0x3613, 0xca,
	0x3614, 0x50,
	0x3628, 0xff,
	0x3629, 0xff,
	0x362a, 0xff,
	0x3633, 0x10,
	0x3634, 0x10,
	0x3635, 0x10,
	0x3636, 0x10,
	0x3663, 0x08,
	0x3669, 0x34,
	0x366e, 0x10,
	0x3706, 0x86,
	0x370b, 0x7e,
	0x3714, 0x23,
	0x3730, 0x12,
	0x3733, 0x10,
	0x3764, 0x00,
	0x3765, 0x00,
	0x3769, 0x62,
	0x376a, 0x2a,
	0x376b, 0x30,
	0x3780, 0x00,
	0x3781, 0x24,
	0x3782, 0x00,
	0x3783, 0x23,
	0x3798, 0x2f,
	0x37a1, 0x60,
	0x37a8, 0x6a,
	0x37ab, 0x3f,
	0x37c2, 0x04,
	0x37c3, 0xf1,
	0x37c9, 0x80,
	0x37cb, 0x06,
	0x37cc, 0x16,
	0x37cd, 0x16,
	0x37ce, 0x16,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380c, 0x07,
	0x380d, 0x8c,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3810, 0x00,
	0x3811, 0x10,
	0x3812, 0x00,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3815, 0x01,
	0x3816, 0x00,
	0x3817, 0x00,
	0x3818, 0x00,
	0x3819, 0x00,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x382b, 0x01,
	0x3830, 0x06,
	0x3836, 0x02,
	0x3837, 0x10,
	0x3862, 0x04,
	0x3863, 0x08,
	0x3cc0, 0x33,
	0x3d85, 0x14,
	0x3d8c, 0x73,
	0x3d8d, 0xde,
	0x4001, 0xe0,
	0x4003, 0x40,
	0x4008, 0x00,
	0x4009, 0x0b,
	0x400a, 0x00,
	0x400b, 0x84,
	0x400f, 0x80,
	0x4010, 0xf0,
	0x4011, 0xff,
	0x4012, 0x02,
	0x4013, 0x01,
	0x4014, 0x01,
	0x4015, 0x01,
	0x4042, 0x00,
	0x4043, 0x80,
	0x4044, 0x00,
	0x4045, 0x80,
	0x4046, 0x00,
	0x4047, 0x80,
	0x4048, 0x00,
	0x4049, 0x80,
	0x4041, 0x03,
	0x404c, 0x20,
	0x404d, 0x00,
	0x404e, 0x00,
	0x4203, 0x81,
	0x4307, 0x30,
	0x4317, 0x00,
	0x4503, 0x08,
	0x4601, 0x80,
	0x4800, 0x44,
	0x4816, 0x53,
	0x481b, 0x58,
	0x481f, 0x27,
	0x4837, 0x0c,
	0x483c, 0x0f,
	0x484b, 0x05,
	0x5000, 0x77,
	0x5001, 0x0a,
	0x5004, 0x04,
	0x502e, 0x03,
	0x5030, 0x41,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579a, 0x00,
	0x579b, 0x50,
	0x579c, 0x00,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x5780, 0x14,
	0x5781, 0x0f,
	0x5782, 0x44,
	0x5783, 0x02,
	0x5784, 0x01,
	0x5785, 0x01,
	0x5786, 0x00,
	0x5787, 0x04,
	0x5788, 0x02,
	0x5789, 0x0f,
	0x578a, 0xfd,
	0x578b, 0xf5,
	0x578c, 0xf5,
	0x578d, 0x03,
	0x578e, 0x08,
	0x578f, 0x0c,
	0x5790, 0x08,
	0x5791, 0x04,
	0x5792, 0x00,
	0x5793, 0x52,
	0x5794, 0xa3,
	0x59f8, 0x3d,
	0x5a08, 0x02,
	0x5b00, 0x02,
	0x5b01, 0x10,
	0x5b02, 0x03,
	0x5b03, 0xcf,
	0x5b05, 0x6c,
	0x5e00, 0x00,
};

static kal_uint16 addr_data_pair_preview[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,

};

static kal_uint16 addr_data_pair_capture[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
};

static kal_uint16 addr_data_pair_normal_video[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
};

static kal_uint16 addr_data_pair_hs_video[] = {
	0x0302, 0x2c,
	0x0303, 0x00,
	0x3501, 0x4c,
	0x3502, 0xa0,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x02,
	0x3801, 0xa0,
	0x3802, 0x02,
	0x3803, 0xb8,
	0x3804, 0x0a,
	0x3805, 0x3f,
	0x3806, 0x06,
	0x3807, 0xf7,
	0x3808, 0x07,
	0x3809, 0x80,
	0x380a, 0x04,
	0x380b, 0x38,
	0x380e, 0x04,
	0x380f, 0xda,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x14,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x00,
	0x579d, 0x00,
	0x579e, 0x07,
	0x579f, 0xa0,
	0x57a0, 0x04,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
};

static kal_uint16 addr_data_pair_slim_video[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
	
};
static kal_uint16 addr_data_pair_custom1[] = {
	0x0302, 0x2f,
	0x0303, 0x00,
	0x3501, 0xc1,
	0x3502, 0x00,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x02,
	0x3801, 0x50,
	0x3802, 0x01,
	0x3803, 0xc8,
	0x3804, 0x0a,
	0x3805, 0x8f,
	0x3806, 0x07,
	0x3807, 0xe7,
	0x3808, 0x08,
	0x3809, 0x20,
	0x380a, 0x06,
	0x380b, 0x18,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x13,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x00,
	0x579d, 0x00,
	0x579e, 0x07,
	0x579f, 0xa0,
	0x57a0, 0x04,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
};

static kal_uint16 addr_data_pair_custom2[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,
};
static kal_uint16 addr_data_pair_custom3[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,

};
static kal_uint16 addr_data_pair_custom4[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,

};
#if 0
static kal_uint16 addr_data_pair_custom5[] = {
	0x0302, 0x4f,
	0x0303, 0x00,
	0x3501, 0x9a,
	0x3502, 0x20,
	0x366e, 0x10,
	0x3714, 0x23,
	0x37c2, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x0c,
	0x3804, 0x0c,
	0x3805, 0xdf,
	0x3806, 0x09,
	0x3807, 0xa3,
	0x3808, 0x0c,
	0x3809, 0xc0,
	0x380a, 0x09,
	0x380b, 0x90,
	0x380e, 0x09,
	0x380f, 0xb2,
	0x3811, 0x10,
	0x3813, 0x04,
	0x3814, 0x01,
	0x3820, 0x80,
	0x3821, 0x46,
	0x382a, 0x01,
	0x4009, 0x0b,
	0x4601, 0x80,
	0x4837, 0x0c,
	0x5795, 0x02,
	0x5796, 0x20,
	0x5797, 0x20,
	0x5798, 0xd5,
	0x5799, 0xd5,
	0x579b, 0x50,
	0x579d, 0x2c,
	0x579e, 0x0c,
	0x579f, 0x40,
	0x57a0, 0x09,
	0x57a1, 0x40,
	0x366d, 0x00,
	0x5003, 0xc8,
	0x5006, 0x00,
	0x5007, 0x00,
	0x5e10, 0xfc,

};
#endif

static void sensor_init(void)
{
	pr_debug(" E\n");
	table_write_cmos_sensor(addr_data_pair_init,
		    sizeof(addr_data_pair_init) / sizeof(kal_uint16));
 
}	/*	sensor_init  */

static void preview_setting(void)
{
	pr_debug("E\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		sizeof(addr_data_pair_preview) / sizeof(kal_uint16));

}	/*	preview_setting  */

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_capture, 
		sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void normal_video_setting(kal_uint16 currefps)
{
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}


static void hs_video_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void slim_video_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom1_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom2_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom3_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom4_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	PK_DBG("end \n");
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
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_16_8(0x300B) << 8) | read_cmos_sensor_16_8(0x300C));
			if (*sensor_id == imgsensor_info.sensor_id) {
				//if ((read_cmos_sensor_16_8(0x302A)) == 0XB0) {
					PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n",imgsensor.i2c_write_id,*sensor_id);
				
				pr_debug("start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = MAIN2_885A_otp_read();
				pr_debug("read eeprom ---vivo_otp_read_when_power_on = %d,MAIN2_885A_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, OV8856PD2078_OTP_ERROR_CODE);
				
					return ERROR_NONE;
			//	}
			}
			pr_debug("Read sensor id fail,write_id:0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
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

	PK_DBG("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_16_8(0x300B) << 8) | read_cmos_sensor_16_8(0x300C));

			if (sensor_id == imgsensor_info.sensor_id) {

				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			pr_debug(
			    "Read sensor id fail, write: 0x%x, sensor: 0x%x\n",
			    imgsensor.i2c_write_id, sensor_id);

			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	sensor_init();
	mdelay(10);
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
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
	PK_DBG("E\n");

	return ERROR_NONE;
}				/*      close  */


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
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
	return ERROR_NONE;
}				/*      preview   */

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
	PK_DBG("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	
	capture_setting(imgsensor.current_fps);
	mdelay(10);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,      
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)                       
{                                                                                
	PK_DBG("E\n");                                                                
                                                                                 
	spin_lock(&imgsensor_drv_lock);                                                
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;                                
	imgsensor.pclk = imgsensor_info.custom3.pclk;                                  
	imgsensor.line_length = imgsensor_info.custom3.linelength;                     
	imgsensor.frame_length = imgsensor_info.custom3.framelength;                   
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;               
	imgsensor.autoflicker_en = KAL_FALSE;                                          
	spin_unlock(&imgsensor_drv_lock);                                              
                                                                                 
	custom3_setting();                                    
	set_mirror_flip(imgsensor.mirror);                                             
                                                                                 
	return ERROR_NONE;                                                             
}	/*	custom3   */                                                               

 
 static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,	  
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)						
 {																				  
	 PK_DBG("E\n"); 															   
																				  
	 spin_lock(&imgsensor_drv_lock);												
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;								
	 imgsensor.pclk = imgsensor_info.custom4.pclk;									
	 imgsensor.line_length = imgsensor_info.custom4.linelength; 					
	 imgsensor.frame_length = imgsensor_info.custom4.framelength;					
	 imgsensor.min_frame_length = imgsensor_info.custom4.framelength;				
	 imgsensor.autoflicker_en = KAL_FALSE;											
	 spin_unlock(&imgsensor_drv_lock);												
																				  
	 custom4_setting();									
	 set_mirror_flip(imgsensor.mirror); 											
																				  
	 return ERROR_NONE; 															
 }	 /*  custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	PK_DBG("%s E\n", __func__);
	
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

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
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*PK_DBG("get_info -> scenario_id = %d\n", scenario_id);*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;

	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;

	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* change pdaf support mode to pdaf VC mode */
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
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

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

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
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	    case MSDK_SCENARIO_ID_CUSTOM1:
	        sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx; 
	        sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc; 

	        break;
	    case MSDK_SCENARIO_ID_CUSTOM2:
	        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx; 
	        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc; 

	        break;   
	        
	     case MSDK_SCENARIO_ID_CUSTOM3:                                                                                    
	         sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;                                                 
	         sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;                                                 
	                                                                                                                        
	         sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;      
	                                                                                                                        
	         break;                                                                                                         
	 case MSDK_SCENARIO_ID_CUSTOM4: 																				   
		 sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx; 												
		 sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty; 												
																														
		 sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;		
																														
		 break; 																										
	case MSDK_SCENARIO_ID_CUSTOM5:
	        sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx; 
	        sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom5.mipi_data_lp2hs_settle_dc; 

	        break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
	        Custom1(image_window, sensor_config_data);
	        break;
	    case MSDK_SCENARIO_ID_CUSTOM2:
	        Custom2(image_window, sensor_config_data);
	        break;  
	    case MSDK_SCENARIO_ID_CUSTOM3:                     
	        Custom3(image_window, sensor_config_data);      
	        break;  
	case MSDK_SCENARIO_ID_CUSTOM4:					   
		Custom4(image_window, sensor_config_data);		
		break;		
	case MSDK_SCENARIO_ID_CUSTOM5:
	        Custom5(image_window, sensor_config_data);
	        break;
	default:
		PK_DBG("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //PK_DBG("framerate = %d\n ", framerate); */
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(
	kal_bool enable, UINT16 framerate)
{
	PK_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	PK_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);

		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
		    / framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
	    (frame_length > imgsensor_info.normal_video.framelength)
	  ? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.slim_video.framelength)
		? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;  
	case MSDK_SCENARIO_ID_CUSTOM3:                                                                                                              
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;                                          	
		spin_lock(&imgsensor_drv_lock);                                                                                                           	
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;     	
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
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		PK_DBG("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*PK_DBG("scenario_id = %d\n", scenario_id);*/

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
	pr_debug("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor_16_8(0x5000, 0x57);
		write_cmos_sensor_16_8(0x5001, 0x02);
		write_cmos_sensor_16_8(0x5e00, 0x80);
	} else {
		write_cmos_sensor_16_8(0x5000, 0x77);
		write_cmos_sensor_16_8(0x5001, 0x0a);
		write_cmos_sensor_16_8(0x5e00, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/*struct SET_PD_BLOCK_INFO_T *PDAFinfo;*/
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    	/*struct SENSOR_VC_INFO_STRUCT *pvcinfo;*/

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*PK_DBG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ imgsensor_info.custom4.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
		PK_DBG(
			"feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
#endif
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		PK_DBG("SENSOR_FEATURE_SET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		write_cmos_sensor_16_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
		PK_DBG("SENSOR_FEATURE_GET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		PK_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:	
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/ 
			break;
		}
		PK_DBG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* PK_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
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
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT)); 	  
			 break; 																							  
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
#if 0
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		PK_DBG("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* video & capture use same setting*/
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;

			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;
/*
    case SENSOR_FEATURE_SET_PDAF:
        PK_DBG("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode= *feature_data_16;
        break;
    */
    case SENSOR_FEATURE_GET_VC_INFO:
        PK_DBG("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		default:
		    memcpy((void *)pvcinfo, (void *) &SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		}
		break;  
#endif
	#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN2_885A_OTP_ERROR_CODE:%d \n", *feature_data,OV8856PD2078_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			PK_DBG("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = OV8856PD2078_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_main2_ov8856pd2078, sizeof(MUINT32)*13); 
				memcpy( feature_data+10, material_inf_main2_ov8856pd2078, sizeof(MUINT32)*4); 
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_main2_ov8856pd2078[%d]= 0x%x\n", i, sn_inf_main2_ov8856pd2078[i]);
						}
				#endif
			}
				break;
			}
			break;
	#endif

	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
		break;

	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	PK_DBG("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		PK_DBG("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
		*feature_return_para_32 =  imgsensor.current_ae_effective_frame;
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
		
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom2.pclk /
			(imgsensor_info.custom2.linelength - 80))*
			imgsensor_info.custom2.grabwindow_width;
			break;
			
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom3.pclk /
			(imgsensor_info.custom3.linelength - 80))*
			imgsensor_info.custom3.grabwindow_width;
			break;
	
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom4.pclk /
			(imgsensor_info.custom4.linelength - 80))*
			imgsensor_info.custom4.grabwindow_width;
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
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom3.mipi_pixel_rate;
			break;
		
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 
				imgsensor_info.custom5.mipi_pixel_rate;
			break;					
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control()  */

//#include "../imgsensor_hop.c"

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
//	hop,
// 	do_hop,
};

UINT32 OV8856PD2078_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
