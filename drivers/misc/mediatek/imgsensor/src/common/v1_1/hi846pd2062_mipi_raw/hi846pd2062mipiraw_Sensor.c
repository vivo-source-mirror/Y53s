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
 *	 hi846pd2062mipiraw_Sensor.c
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


#define PFX "MAIN3[0846]_camera_sensor"
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
#include "../imgsensor_i2c.h"
#include "hi846pd2062mipiraw_Sensor.h"


/*
 * #define PK_DBG(format, args...) pr_debug(
 * PFX "[%s] " format, __func__, ##args)
 */
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI846PD2062_SENSOR_ID,
	.checksum_value = 0x55e2a82f,
	.pre = {
		.pclk = 287700000,				//record different mode's pclk
		.linelength  = 3800,				//record different mode's linelength
		.framelength = 2524,			/*record different mode's framelength*/
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		/*1296 record different mode's width of grabwindow*/
		.grabwindow_height = 1224,		/*972 record different mode's height of grabwindow*/
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 719300000,	
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 287700000,				//record different mode's pclk
		.linelength  = 3800,				//record different mode's linelength
		.framelength = 2524,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 3264,   /*2592 record different mode's width of grabwindow*/
		.grabwindow_height = 2448,	 /*1944record different mode's height of grabwindow*/
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,	
		.mipi_pixel_rate = 287700000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 287700000,				//record different mode's pclk
		.linelength = 3800,				//record different mode's linelength
		.framelength = 2524,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 3264,		/*record different mode's width of grabwindow*/
		.grabwindow_height = 1840,		/*record different mode's height of grabwindow*/
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 287700000,	
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 287700000,
		.linelength = 3800,				//record different mode's linelength
		.framelength = 631,				//record different mode's framelength
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640 ,		//record different mode's width of grabwindow
		.grabwindow_height = 480 ,
		.mipi_data_lp2hs_settle_dc = 14,//unit , ns	
		.mipi_pixel_rate = 72000000,	
		.max_framerate = 1200,
	},
	    .slim_video = {
		.pclk = 287700000,		
		.linelength = 3800,		
		.framelength = 2526,	
		.startx = 0,			
		.starty = 0,			
		.grabwindow_width = 1632,
		.grabwindow_height = 1224,
		.mipi_data_lp2hs_settle_dc = 14,
		.mipi_pixel_rate = 144000000,	
		.max_framerate = 300,	
	},
	
	.custom1 = {
		.pclk = 287700000,				//record different mode's pclk
		.linelength  = 3800,				//record different mode's linelength
		.framelength = 2524,			/*record different mode's framelength*/
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2080,   /*2592 record different mode's width of grabwindow*/
		.grabwindow_height = 1560,	 /*1944record different mode's height of grabwindow*/
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,	
		.mipi_pixel_rate = 287700000,
		.max_framerate = 300,
	},
	.custom2 = {
		.pclk = 287700000,				//record different mode's pclk
		.linelength  = 3800,				//record different mode's linelength
		.framelength = 2526,			/*record different mode's framelength*/
		.startx= 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1632,		/*1296 record different mode's width of grabwindow*/
		.grabwindow_height = 1224,		/*972 record different mode's height of grabwindow*/
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 14,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 136000000,	
		.max_framerate = 300,
	},

	.margin = 6,
	.min_shutter = 6,
	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.gain_step = 64,
	.gain_type = 3,
	.max_frame_length = 0xffff9,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 1,	/* The delay frame of setting frame length  */

	.ihdr_support = 0,      //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 7,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
   	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
  	.mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
	.mclk = 26,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
	.i2c_addr_table = {0x42, /*0x20,*/0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
	.i2c_speed = 400, // i2c read/write speed
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_H_MIRROR,				//mirrorflip information IMAGE_NORMAL
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x0100,					//current shutter
	.gain = 0xe0,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	 .test_pattern = KAL_FALSE, 
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	 .ihdr_mode = 0, 
	.i2c_write_id = 0x40,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] =
{
	{ 3264, 2448,	0,		0,	 3264, 2448, 1632, 1224, 0, 0, 1632, 1224,  0,  0, 1632, 1224}, // Preview	  
	{ 3264, 2448, 	0,		0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // capture	  
	{ 3264, 2448, 	0,	  304,	 3264, 1840, 3264, 1840, 0, 0, 3264, 1840,  0,  0, 3264, 1840}, // video		
	{ 3264, 2448,	0,	  264,   3264, 1920, 816,  480,  88, 0, 640,480,	0,	0, 640,  480}, //hight speed video	  
	{ 3264, 2448,	0,		0,	 3264, 2448, 1632, 1224, 0, 0, 1632, 1224,  0,  0, 1632, 1224}, // slime video	  
	{ 3264, 2448, 	592,  444,	 2080, 1560, 2080, 1560, 0, 0, 2080, 1560,  0,  0, 2080, 1560}, // custom1	  
	{ 3264, 2448,	0,		0,	 3264, 2448, 1632, 1224, 0, 0, 1632, 1224,  0,  0, 1632, 1224}, // custom2	  
};

/****vivo caofeixiang add for CameraEM otp errorcode****/
extern int vivo_main3_otp_read(void);
int vivo_main3_otp_read_when_power_on;
extern otp_error_code_t MAIN3_OTP_ERROR_CODE_HI846PD2062;
MUINT32  sn_inf_main2_hi846pd2062[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main_hi846pd2062[4];  
/****vivo caofeixiang add end****/
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

#if 1
static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}
#endif
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
	PK_DBG("dummyline = %d, dummypixels = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel);

	/* return; //for test */
	write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}				/*      set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_16_8(0x0F17) << 8) | read_cmos_sensor_16_8(0x0F16));	
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	//kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	PK_DBG("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;

		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*      set_max_framerate  */

static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	//kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	//printk("pangfei shutter %d line %d\n",shutter,__LINE__);
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
      write_cmos_sensor_16_16(0x0006, imgsensor.frame_length);

		}
	} else {
		/* Extend frame length */
    write_cmos_sensor_16_16(0x0006, imgsensor.frame_length);
		PK_DBG("(else)imgsensor.frame_length = %d\n",
			imgsensor.frame_length);

	}
	/* Update Shutter */
	write_cmos_sensor_16_16(0x0073, ((shutter & 0xFF0000) >> 16));	  
	write_cmos_sensor_16_16(0x0074, shutter & 0x00FFFF);	  
	PK_DBG("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

}				/*      write_shutter  */



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

	PK_DBG("set_shutter");
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */
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
			write_cmos_sensor_16_16(0x0006, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_16(0x0006, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_16_16(0X0074, shutter & 0xFFFF);

	PK_DBG("shutter =%d, framelength =%d/%d, dummy_line=%d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);

}				/*       set_shutter_frame_length  */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
  	kal_uint16 reg_gain = 0x0000;
	reg_gain = gain / 4 - 16;

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

	/* gain=1024;//for test */
	/* return; //for test */

	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		PK_DBG("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	PK_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_8(0x0077,reg_gain&0xff);/* max = 0xf0*/

    return gain;
		
}	/*	set_gain  */
#if 0
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 0
	PK_DBG("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
#endif
}
#endif

static void set_mirror_flip(kal_uint8 image_mirror)
{
   switch (image_mirror) {
 
        case IMAGE_NORMAL:
            write_cmos_sensor_16_16(0x000e,0x0000);   /*B*/ 
            break;

        case IMAGE_H_MIRROR:
            write_cmos_sensor_16_16(0x000e,0x0100); /*Gb*/ 
            break;

        case IMAGE_V_MIRROR:
            write_cmos_sensor_16_16(0x000e,0x0200);/*Gr*/ 
            break;

        case IMAGE_HV_MIRROR:
            write_cmos_sensor_16_16(0x000e,0x0300);/*R*/
            break;
        default:
        PK_DBG("Error image_mirror setting\n");
    }
}





/*************************************************************************
 * FUNCTION
 *	check_stremoff
 *
 * DESCRIPTION
 *	waiting function until sensor streaming finish.
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
static kal_uint16 addr_data_pair_init[] = {
0x2000, 0x9D7A,
0x2002, 0x00EF,
0x2004, 0x0047,
0x2006, 0x3FFF,
0x2008, 0x3FFF,
0x200A, 0xC216,
0x200C, 0x1292,
0x200E, 0xC01A,
0x2010, 0x403D,
0x2012, 0x000E,
0x2014, 0x403E,
0x2016, 0x0B80,
0x2018, 0x403F,
0x201A, 0x82AE,
0x201C, 0x1292,
0x201E, 0xC00C,
0x2020, 0x4130,
0x2022, 0x43E2,
0x2024, 0x0180,
0x2026, 0x4130,
0x2028, 0x7400,
0x202A, 0x5000,
0x202C, 0x0253,
0x202E, 0x0AD1,
0x2030, 0x2360,
0x2032, 0x0009,
0x2034, 0x5020,
0x2036, 0x000B,
0x2038, 0x0002,
0x203A, 0x0044,
0x203C, 0x0016,
0x203E, 0x1792,
0x2040, 0x7002,
0x2042, 0x154F,
0x2044, 0x00D5,
0x2046, 0x000B,
0x2048, 0x0019,
0x204A, 0x1698,
0x204C, 0x000E,
0x204E, 0x099A,
0x2050, 0x0058,
0x2052, 0x7000,
0x2054, 0x1799,
0x2056, 0x0310,
0x2058, 0x03C3,
0x205A, 0x004C,
0x205C, 0x064A,
0x205E, 0x0001,
0x2060, 0x0007,
0x2062, 0x0BC7,
0x2064, 0x0055,
0x2066, 0x7000,
0x2068, 0x1550,
0x206A, 0x158A,
0x206C, 0x0004,
0x206E, 0x1488,
0x2070, 0x7010,
0x2072, 0x1508,
0x2074, 0x0004,
0x2076, 0x0016,
0x2078, 0x03D5,
0x207A, 0x0055,
0x207C, 0x08CA,
0x207E, 0x2019,
0x2080, 0x0007,
0x2082, 0x7057,
0x2084, 0x0FC7,
0x2086, 0x5041,
0x2088, 0x12C8,
0x208A, 0x5060,
0x208C, 0x5080,
0x208E, 0x2084,
0x2090, 0x12C8,
0x2092, 0x7800,
0x2094, 0x0802,
0x2096, 0x040F,
0x2098, 0x1007,
0x209A, 0x0803,
0x209C, 0x080B,
0x209E, 0x3803,
0x20A0, 0x0807,
0x20A2, 0x0404,
0x20A4, 0x0400,
0x20A6, 0xFFFF,
0x20A8, 0xF0B2,
0x20AA, 0xFFEF,
0x20AC, 0x0A84,
0x20AE, 0x1292,
0x20B0, 0xC02E,
0x20B2, 0x4130,
0x20B4, 0xF0B2,
0x20B6, 0xFFBF,
0x20B8, 0x2004,
0x20BA, 0x403F,
0x20BC, 0x00C3,
0x20BE, 0x4FE2,
0x20C0, 0x831E,
0x20C2, 0x43CF,
0x20C4, 0x0000,
0x20C6, 0x9382,
0x20C8, 0xC314,
0x20CA, 0x2003,
0x20CC, 0x12B0,
0x20CE, 0xCAB0,
0x20D0, 0x4130,
0x20D2, 0x12B0,
0x20D4, 0xC90A,
0x20D6, 0x4130,
0x20D8, 0x42D2,
0x20DA, 0x831E,
0x20DC, 0x00C3,
0x20DE, 0x9382,
0x20E0, 0xC314,
0x20E2, 0x2009,
0x20E4, 0x120B,
0x20E6, 0x120A,
0x20E8, 0x1209,
0x20EA, 0x1208,
0x20EC, 0x1207,
0x20EE, 0x1206,
0x20F0, 0x4030,
0x20F2, 0xC15E,
0x20F4, 0x4130,
0x20F6, 0x1292,
0x20F8, 0xC008,
0x20FA, 0x4130,
0x20FC, 0x430E,
0x20FE, 0x930A,
0x2100, 0x2407,
0x2102, 0xC312,
0x2104, 0x100C,
0x2106, 0x2801,
0x2108, 0x5A0E,
0x210A, 0x5A0A,
0x210C, 0x930C,
0x210E, 0x23F7,
0x2110, 0x4130,
0x2112, 0x42D2,
0x2114, 0x82A1,
0x2116, 0x00C2,
0x2118, 0x421A,
0x211A, 0x0B80,
0x211C, 0x903A,
0x2120, 0x2809,
0x2122, 0x503A,
0x2126, 0x421C,
0x2128, 0x8318,
0x212A, 0x12B0,
0x212C, 0xFCFC,
0x212E, 0x4E82,
0x2130, 0x831C,
0x2132, 0x3C02,
0x2134, 0x4382,
0x2136, 0x831C,
0x2138, 0x1292,
0x213A, 0xC040,
0x213C, 0xD292,
0x213E, 0x8322,
0x2140, 0x0A84,
0x2142, 0x4130,
0x2144, 0x1292,
0x2146, 0xC006,
0x2148, 0x42A2,
0x214A, 0x7324,
0x214C, 0x9382,
0x214E, 0xC314,
0x2150, 0x2011,
0x2152, 0x425F,
0x2154, 0x82A1,
0x2156, 0xF25F,
0x2158, 0x00C1,
0x215A, 0xF35F,
0x215C, 0x2406,
0x215E, 0x425F,
0x2160, 0x00C0,
0x2162, 0xF37F,
0x2164, 0x522F,
0x2166, 0x4F82,
0x2168, 0x7324,
0x216A, 0x425F,
0x216C, 0x82D4,
0x216E, 0xF35F,
0x2170, 0x4FC2,
0x2172, 0x01B3,
0x2174, 0x93C2,
0x2176, 0x829F,
0x2178, 0x2421,
0x217A, 0x403E,
0x217C, 0xFFFE,
0x217E, 0x40B2,
0x2180, 0xEC78,
0x2182, 0x8326,
0x2184, 0x40B2,
0x2186, 0xEC78,
0x2188, 0x8328,
0x218A, 0x40B2,
0x218C, 0xEC78,
0x218E, 0x832A,
0x2190, 0xB3D2,
0x2192, 0x008C,
0x2194, 0x2405,
0x2196, 0x4E0F,
0x2198, 0x503F,
0x219A, 0xFFD8,
0x219C, 0x4F82,
0x219E, 0x8326,
0x21A0, 0x90F2,
0x21A2, 0x0003,
0x21A4, 0x008C,
0x21A6, 0x2401,
0x21A8, 0x4130,
0x21AA, 0x421F,
0x21AC, 0x8326,
0x21AE, 0x5E0F,
0x21B0, 0x4F82,
0x21B2, 0x8328,
0x21B4, 0x5E0F,
0x21B6, 0x4F82,
0x21B8, 0x832A,
0x21BA, 0x3FF6,
0x21BC, 0x432E,
0x21BE, 0x3FDF,
0x21C0, 0x421F,
0x21C2, 0x7100,
0x21C4, 0x4F0E,
0x21C6, 0x503E,
0x21C8, 0xFFD8,
0x21CA, 0x4E82,
0x21CC, 0x7A04,
0x21CE, 0x421E,
0x21D0, 0x8326,
0x21D2, 0x5F0E,
0x21D4, 0x4E82,
0x21D6, 0x7A06,
0x21D8, 0x0B00,
0x21DA, 0x7304,
0x21DC, 0x0050,
0x21DE, 0x40B2,
0x21E0, 0xD081,
0x21E2, 0x0B88,
0x21E4, 0x421E,
0x21E6, 0x8328,
0x21E8, 0x5F0E,
0x21EA, 0x4E82,
0x21EC, 0x7A0E,
0x21EE, 0x521F,
0x21F0, 0x832A,
0x21F2, 0x4F82,
0x21F4, 0x7A10,
0x21F6, 0x0B00,
0x21F8, 0x7304,
0x21FA, 0x007A,
0x21FC, 0x40B2,
0x21FE, 0x0081,
0x2200, 0x0B88,
0x2202, 0x4392,
0x2204, 0x7A0A,
0x2206, 0x0800,
0x2208, 0x7A0C,
0x220A, 0x0B00,
0x220C, 0x7304,
0x220E, 0x022B,
0x2210, 0x40B2,
0x2212, 0xD081,
0x2214, 0x0B88,
0x2216, 0x0B00,
0x2218, 0x7304,
0x221A, 0x0255,
0x221C, 0x40B2,
0x221E, 0x0081,
0x2220, 0x0B88,
0x2222, 0x9382,
0x2224, 0x7112,
0x2226, 0x2403,
0x2228, 0xF0B2,
0x222A, 0xFFEF,
0x222C, 0x0A84,
0x222E, 0x4130,
0x2230, 0x421E,
0x2232, 0x831C,
0x2234, 0x430F,
0x2236, 0x4130,
0x2238, 0x120B,
0x223A, 0x430B,
0x223C, 0x1292,
0x223E, 0xC03C,
0x2240, 0x421E,
0x2242, 0x0B80,
0x2244, 0x93C2,
0x2246, 0x829A,
0x2248, 0x2403,
0x224A, 0x9392,
0x224C, 0x826E,
0x224E, 0x241B,
0x2250, 0x4292,
0x2252, 0x8320,
0x2254, 0x8322,
0x2256, 0xB3A2,
0x2258, 0x8258,
0x225A, 0x241C,
0x225C, 0x4B0F,
0x225E, 0x5F0F,
0x2260, 0x9F1E,
0x2262, 0xFF90,
0x2264, 0x2804,
0x2266, 0x531B,
0x2268, 0x903B,
0x226A, 0x0006,
0x226C, 0x2BF7,
0x226E, 0x903B,
0x2270, 0x0006,
0x2272, 0x2406,
0x2274, 0x903B,
0x2276, 0x0006,
0x2278, 0x280D,
0x227A, 0x403B,
0x227C, 0x0005,
0x227E, 0x3C0A,
0x2280, 0x4382,
0x2282, 0x8322,
0x2284, 0x3FF7,
0x2286, 0x421F,
0x2288, 0x0A84,
0x228A, 0xF03F,
0x228C, 0x0010,
0x228E, 0x4F82,
0x2290, 0x8320,
0x2292, 0x3FDE,
0x2294, 0x4B0F,
0x2296, 0x5F0F,
0x2298, 0x5F0F,
0x229A, 0x503F,
0x229C, 0xFEE6,
0x229E, 0x4FA2,
0x22A0, 0x0482,
0x22A2, 0x4F92,
0x22A4, 0x0002,
0x22A6, 0x0484,
0x22A8, 0x4B0E,
0x22AA, 0x5E0E,
0x22AC, 0x5E0E,
0x22AE, 0x5E0E,
0x22B0, 0x4B0F,
0x22B2, 0x5F0F,
0x22B4, 0x5F0F,
0x22B6, 0x5F0E,
0x22B8, 0x4E92,
0x22BA, 0xFF46,
0x22BC, 0x0496,
0x22BE, 0x4B0F,
0x22C0, 0x5F0F,
0x22C2, 0x5B0F,
0x22C4, 0x5F0F,
0x22C6, 0x521F,
0x22C8, 0x831A,
0x22CA, 0x5F0F,
0x22CC, 0x4F92,
0x22CE, 0xFF46,
0x22D0, 0x0498,
0x22D2, 0x503E,
0x22D4, 0xFEFE,
0x22D6, 0x403D,
0x22D8, 0x0006,
0x22DA, 0x403F,
0x22DC, 0x048A,
0x22DE, 0x1292,
0x22E0, 0xC00C,
0x22E2, 0x413B,
0x22E4, 0x4130,
0x22E6, 0x0110,
0x22E8, 0x00F4,
0x22EA, 0x0118,
0x22EC, 0x00FA,
0x22EE, 0x0124,
0x22F0, 0x0102,
0x22F2, 0x0140,
0x22F4, 0x0110,
0x22F6, 0x0170,
0x22F8, 0x012A,
0x22FA, 0x01DC,
0x22FC, 0x015A,
0x22FE, 0x01C6,
0x2300, 0x038B,
0x2302, 0xF0F1,
0x2304, 0x008D,
0x2306, 0x011A,
0x2308, 0x864C,
0x230A, 0x01B8,
0x230C, 0x0371,
0x230E, 0xEA0F,
0x2310, 0x0086,
0x2312, 0x010C,
0x2314, 0x8312,
0x2316, 0x018A,
0x2318, 0x0313,
0x231A, 0xE070,
0x231C, 0x007E,
0x231E, 0x00FC,
0x2320, 0x7F02,
0x2322, 0x0148,
0x2324, 0x028F,
0x2326, 0xCCCD,
0x2328, 0x0071,
0x232A, 0x00E3,
0x232C, 0x7878,
0x232E, 0x00F8,
0x2330, 0x01F0,
0x2332, 0xB216,
0x2334, 0x005E,
0x2336, 0x00BD,
0x2338, 0x6DF6,
0x233A, 0x0094,
0x233C, 0x0128,
0x233E, 0x89AE,
0x2340, 0x0046,
0x2342, 0x008C,
0x2344, 0x5EB5,
0x2346, 0x0000,
0x2348, 0x0000,
0x234A, 0x023A,
0x234C, 0x0214,
0x234E, 0x0410,
0x2350, 0x04C4,
0x2352, 0x0030,
0x2354, 0x001A,
0x2356, 0x0252,
0x2358, 0x0228,
0x235A, 0x041E,
0x235C, 0x04D2,
0x235E, 0x0078,
0x2360, 0x0042,
0x2362, 0x0272,
0x2364, 0x0248,
0x2366, 0x0436,
0x2368, 0x04EA,
0x236A, 0x0120,
0x236C, 0x0088,
0x236E, 0x02AA,
0x2370, 0x0280,
0x2372, 0x0460,
0x2374, 0x0604,
0x2376, 0x02D0,
0x2378, 0x010A,
0x237A, 0x0312,
0x237C, 0x02E8,
0x237E, 0x04AE,
0x2380, 0x0638,
0x2382, 0x0510,
0x2384, 0x02A0,
0x2386, 0x0478,
0x2388, 0x044E,
0x238A, 0x053E,
0x238C, 0x0698,
0x238E, 0x00C0,
0x2390, 0x00C8,
0x2392, 0x00D0,
0x2394, 0x00D8,
0x2396, 0x00E0,
0x2398, 0x00E8,
0x239A, 0x00F0,
0x23FE, 0xC056,
0x3232, 0xFC0C,
0x3236, 0xFC22,
0x3238, 0xFD12,
0x323A, 0xFDC0,
0x323C, 0xFD44,
0x3240, 0xFE38,
0x3244, 0xFE30,
0x3246, 0xFCD8,
0x3248, 0xFCA8,
0x324E, 0xFCB4,
0x326A, 0x8302,
0x326C, 0x830A,
0x326E, 0x0000,
0x32CA, 0xFC28,
0x32CC, 0xC3BC,
0x32CE, 0xC34C,
0x32D0, 0xC35A,
0x32D2, 0xC368,
0x32D4, 0xC376,
0x32D6, 0xC3C2,
0x32D8, 0xC3E6,
0x32DA, 0x0003,
0x32DC, 0x0003,
0x32DE, 0x00C7,
0x32E0, 0x0031,
0x32E2, 0x0031,
0x32E4, 0x0031,
0x32E6, 0xFC28,
0x32E8, 0xC3BC,
0x32EA, 0xC384,
0x32EC, 0xC392,
0x32EE, 0xC3A0,
0x32F0, 0xC3AE,
0x32F2, 0xC3C4,
0x32F4, 0xC3E6,
0x32F6, 0x0003,
0x32F8, 0x0003,
0x32FA, 0x00C7,
0x32FC, 0x0031,
0x32FE, 0x0031,
0x3300, 0x0031,
0x3302, 0x82CA,
0x3304, 0xC164,
0x3306, 0x82E6,
0x3308, 0xC19C,
0x330A, 0x001F,
0x330C, 0x001A,
0x330E, 0x0034,
0x3310, 0x0000,
0x3312, 0x0000,
0x3314, 0xFC94,
0x3316, 0xC3D8,
0x211E, 0x0070,
0x2124, 0xFF90,
0x3318, 0x0000,
0x331A, 0x0001,
0x3258, 0x0000,
0x0A00, 0x0000,
0x0E04, 0x0012,
0x002E, 0x1111,
0x0032, 0x1111,
0x0022, 0x0008,
0x0026, 0x0040,
0x0028, 0x0017,
0x002C, 0x09CF,
0x005C, 0x2101,
0x0006, 0x09BC,
0x0008, 0x0ED8,
0x000E, 0x0100,
0x000C, 0x0022,
0x0A22, 0x0000,
0x0A24, 0x0000,
0x0804, 0x0000,
0x0A12, 0x0CC0,
0x0A14, 0x0990,
0x0074, 0x09B6,
0x0076, 0x0000,
0x051E, 0x0000,
0x0200, 0x0400,
0x0A1A, 0x0C00,
0x0A0C, 0x0010,
0x0A1E, 0x0CCF,
0x0402, 0x0110,
0x0404, 0x00F4,
0x0408, 0x0000,
0x0410, 0x008D,
0x0412, 0x011A,
0x0414, 0x864C,
0x021C, 0x0003,
0x021E, 0x0235,
0x0C00, 0x9950,
0x0C06, 0x0021,
0x0C10, 0x0040,
0x0C12, 0x0040,
0x0C14, 0x0040,
0x0C16, 0x0040,
0x0A02, 0x0100,
0x0A04, 0x015A,
0x0418, 0x0000,
0x0128, 0x0027,
0x012A, 0xFFFF,
0x0120, 0x0044,
0x0122, 0x0374,
0x012C, 0x001F,
0x012E, 0xFFFF,
0x0124, 0x003E,
0x0126, 0x0376,
0x0746, 0x0050,
0x0748, 0x01D5,
0x074A, 0x022B,
0x074C, 0x03B0,
0x0756, 0x043F,
0x0758, 0x3F1D,
0x0B02, 0xE04D,
0x0B10, 0x6821,
0x0B12, 0x0120,
0x0B14, 0x0001,
0x2008, 0x38FD,
0x326E, 0x0000,
0x0900, 0x0320,
0x0902, 0xC31A,
0x0914, 0xC109,
0x0916, 0x061A,
0x0918, 0x0406,
0x091A, 0x0A09,
0x091C, 0x0C07,
0x091E, 0x0A00,
0x090C, 0x03FC,
0x090E, 0x0028,
0x0954, 0x0089,
0x0956, 0x0000,
0x0958, 0xCA00,
0x095A, 0x9240,
0x0F08, 0x2F04,
0x0F30, 0x001F,
0x0F36, 0x001F,
0x0F04, 0x3A00,
0x0F32, 0x025A,
0x0F38, 0x0373,
0x0F2A, 0x0024,
0x006A, 0x0100,
0x004C, 0x0100,
0x0044, 0x0001,	
};

static kal_uint16 addr_data_pair_preview[] = {
	0x0A00, 0x0000, 
	0x002E, 0x3311, 
	0x0032, 0x3311, 
	0x0026, 0x0040, 
	0x002C, 0x09CF, 
	0x005C, 0x4202, 
	0x0006, 0x09dc, 
	0x0008, 0x0ED8, 
	0x000C, 0x0122, 
	0x0A22, 0x0100, 
	0x0A24, 0x0000, 
	0x0804, 0x0000, 
	0x0A12, 0x0660, 
	0x0A14, 0x04C8, 
	0x0074, 0x09B6, 
	0x021C, 0x0001,
	0x0A04, 0x016A, 
	0x0418, 0x0000, 
	0x0B02, 0xE04D, 
	0x0B10, 0x6C21, 
	0x0B12, 0x0120, 
	0x0B14, 0x0005, 
	0x2008, 0x38FD, 
	0x326E, 0x0000, 
	0x0900, 0x0300, 
	0x0902, 0x4319, 
	0x0914, 0xC106, 
	0x0916, 0x0616, 
	0x0918, 0x0306, 
	0x091A, 0x0A0B, 
	0x091C, 0x0E08, 
	0x091E, 0x0A00, 
	0x090C, 0x03FB, 
	0x090E, 0x002B, 
	0x0954, 0x0089, 
	0x0956, 0x0000, 
	0x0958, 0xCA80, 
	0x095A, 0x9240, 
	0x0F32, 0x0253,
    0x0F38, 0x0253,
	0x0F2A, 0x4124, 
	0x004C, 0x0100, 
	0x331A, 0x0001,
	0x0A00, 0x0100,
};

static kal_uint16 addr_data_pair_capture[] = {
    0x0A00, 0x0000,
    0x002E, 0x1111,
    0x0032, 0x1111,
    0x0026, 0x0040,
    0x002C, 0x09CF,
    0x005C, 0x2101,
    0x0006, 0x09DC,
    0x0008, 0x0ED8,
    0x000C, 0x0022,
    0x0A22, 0x0000,
    0x0A24, 0x0000,
    0x0804, 0x0000,
    0x0A12, 0x0CC0,
    0x0A14, 0x0990,
    0x0074, 0x09D8,
    0x021C, 0x0001,
    0x0A04, 0x014A,
    0x0418, 0x0000,
    0x0B02, 0xE04D,
    0x0B10, 0x6821,
    0x0B12, 0x0120,
    0x0B14, 0x0001,
    0x2008, 0x38FD,
    0x326E, 0x0000,
    0x0900, 0x0320,
    0x0902, 0xC31A,
    0x0914, 0xC109,
    0x0916, 0x061A,
    0x0918, 0x0406,
    0x091A, 0x0A09,
    0x091C, 0x0C07,
    0x091E, 0x0A00,
    0x090C, 0x03FC,
    0x090E, 0x0028,
    0x0954, 0x0089,
    0x0956, 0x0000,
    0x0958, 0xCA00,
    0x095A, 0x9240,
    0x0F32, 0x0253,
    0x0F38, 0x0253,
    0x0F2A, 0x0024,
    0x004C, 0x0100,
    0x331A, 0x0001,
};

static kal_uint16 addr_data_pair_normal_video[] = {
    0x0A00, 0x0000,
    0x002E, 0x1111,
    0x0032, 0x1111,
    0x0026, 0x0170,
    0x002C, 0x089F,
    0x005C, 0x2101,
    0x0006, 0x09dc,
    0x0008, 0x0ED8,
    0x000C, 0x0022,
    0x0A22, 0x0000,
    0x0A24, 0x0000,
    0x0804, 0x0000,
    0x0A12, 0x0CC0,
    0x0A14, 0x0730,
    0x0074, 0x09D8,
    0x021C, 0x0001,
    0x0A04, 0x014A,
    0x0418, 0x0000,
    0x0B02, 0xE04D,
    0x0B10, 0x6821,
    0x0B12, 0x0120,
    0x0B14, 0x0001,
    0x2008, 0x38FD,
    0x326E, 0x0000,
    0x0900, 0x0320,
    0x0902, 0xC31A,
    0x0914, 0xC109,
    0x0916, 0x061A,
    0x0918, 0x0406,
    0x091A, 0x0A09,
    0x091C, 0x0C07,
    0x091E, 0x0A00,
    0x090C, 0x03FC,
    0x090E, 0x0028,
    0x0954, 0x0089,
    0x0956, 0x0000,
    0x0958, 0xCA00,
    0x095A, 0x9240,
    0x0F32, 0x0253,
    0x0F38, 0x0253,
    0x0F2A, 0x0024,
    0x004C, 0x0100,
    0x331A, 0x0001,
	
};

static kal_uint16 addr_data_pair_hs_video[] = {
	0x0A00, 0x0000, 
	0x002E, 0x7711, 
	0x0032, 0x7711, 
	0x0026, 0x0148, 
	0x002C, 0x08C7, 
	0x005C, 0x4404, 
	0x0006, 0x0277, 
	0x0008, 0x0ED8, 
	0x000C, 0x0322, 
	0x0A22, 0x0200, 
	0x0A24, 0x0000, 
	0x0804, 0x0058, 
	0x0A12, 0x0280, 
	0x0A14, 0x01E0, 
	0x0074, 0x0271,
	0x021C, 0x0001, 
	0x0A04, 0x016A, 
	0x0418, 0x0210, 
	0x0B02, 0xE04D, 
	0x0B10, 0x7021, 
	0x0B12, 0x0120, 
	0x0B14, 0x0001, 
	0x2008, 0x38FD, 
	0x326E, 0x0000, 
	0x0900, 0x0300, 
	0x0902, 0x4319, 
	0x0914, 0xC105, 
	0x0916, 0x030C, 
	0x0918, 0x0304, 
	0x091A, 0x0708, 
	0x091C, 0x0B04, 
	0x091E, 0x0500, 
	0x090C, 0x0208, 
	0x090E, 0x008A, 
	0x0954, 0x0089, 
	0x0956, 0x0000, 
	0x0958, 0xCA80, 
	0x095A, 0x9240, 
	0x0F2A, 0x4924, 
	0x004C, 0x0100, 
	0x0A00, 0x0100,
};

static kal_uint16 addr_data_pair_slim_video[] = {
	0x0A00, 0x0000, 
	0x002E, 0x3311, 
	0x0032, 0x3311, 
	0x0026, 0x0040, 
	0x002C, 0x09CF, 
	0x005C, 0x4202, 
	0x0006, 0x09de, 
	0x0008, 0x0ED8, 
	0x000C, 0x0122, 
	0x0A22, 0x0100, 
	0x0A24, 0x0000, 
	0x0804, 0x0000, 
	0x0A12, 0x0660, 
	0x0A14, 0x04C8, 
	0x0074, 0x09B6, 
	0x021C, 0x0001,
	0x0A04, 0x016A, 
	0x0418, 0x0000, 
	0x0B02, 0xE04D, 
	0x0B10, 0x6C21, 
	0x0B12, 0x0120, 
	0x0B14, 0x0005, 
	0x2008, 0x38FD, 
	0x326E, 0x0000, 
	0x0900, 0x0300, 
	0x0902, 0x4319, 
	0x0914, 0xC109, 
	0x0916, 0x061A, 
	0x0918, 0x0407, 
	0x091A, 0x0A0B, 
	0x091C, 0x0E08, 
	0x091E, 0x0A00, 
	0x090C, 0x0427, 
	0x090E, 0x0059, 
	0x0954, 0x0089, 
	0x0956, 0x0000, 
	0x0958, 0xCA80, 
	0x095A, 0x9240, 
	0x0F32, 0x0253,
    0x0F38, 0x0253,
	0x0F2A, 0x4124, 
	0x004C, 0x0100, 
	0x0A00, 0x0100,	
};

static kal_uint16 addr_data_pair_custom1[] = {  /* 2080x1560@30fps for dual camera*/
    0x0A00, 0x0000,
    0x002E, 0x1111,
    0x0032, 0x1111,
    0x0026, 0x01FC,
    0x002C, 0x0813,
    0x005C, 0x2101,
    0x0006, 0x09DC,
    0x0008, 0x0ED8,
    0x000C, 0x0022,
    0x0A22, 0x0000,
    0x0A24, 0x0000,
    0x0804, 0x0250,
    0x0A12, 0x0820,
    0x0A14, 0x0618,
    0x0074, 0x09D8,
    0x021C, 0x0001,
    0x0A04, 0x014A,
    0x0418, 0x02C8,
    0x0B02, 0xE04D,
    0x0B10, 0x6821,
    0x0B12, 0x0120,
    0x0B14, 0x0001,
    0x2008, 0x38FD,
    0x326E, 0x0000,
    0x0900, 0x0320,
    0x0902, 0xC31A,
    0x0914, 0xC109,
    0x0916, 0x061A,
    0x0918, 0x0406,
    0x091A, 0x0A09,
    0x091C, 0x0C07,
    0x091E, 0x0A00,
    0x090C, 0x03FC,
    0x090E, 0x0028,
    0x0954, 0x0089,
    0x0956, 0x0000,
    0x0958, 0xCA00,
    0x095A, 0x9240,
    0x0F32, 0x0253,
    0x0F38, 0x0253,
    0x0F2A, 0x0024,
    0x004C, 0x0100,
    0x331A, 0x0001,
};


static kal_uint16 addr_data_pair_custom2[] = { /* 1632x2448@30fps for dual camera*/
	0x0A00, 0x0000, 
	0x002E, 0x3311, 
	0x0032, 0x3311, 
	0x0026, 0x0040, 
	0x002C, 0x09CF, 
	0x005C, 0x4202, 
	0x0006, 0x09de, 
	0x0008, 0x0ED8, 
	0x000C, 0x0122, 
	0x0A22, 0x0100, 
	0x0A24, 0x0000, 
	0x0804, 0x0000, 
	0x0A12, 0x0660, 
	0x0A14, 0x04C8, 
	0x0074, 0x09B6, 
	0x021C, 0x0001,
	0x0A04, 0x016A, 
	0x0418, 0x0000, 
	0x0B02, 0xE04D, 
	0x0B10, 0x6C21, 
	0x0B12, 0x0120, 
	0x0B14, 0x0005, 
	0x2008, 0x38FD, 
	0x326E, 0x0000, 
	0x0900, 0x0300, 
	0x0902, 0x4319, 
	0x0914, 0xC106, 
	0x0916, 0x0616, 
	0x0918, 0x0306, 
	0x091A, 0x0A0B, 
	0x091C, 0x0E08, 
	0x091E, 0x0A00, 
	0x090C, 0x03FB, 
	0x090E, 0x002B, 
	0x0954, 0x0089, 
	0x0956, 0x0000, 
	0x0958, 0xCA80, 
	0x095A, 0x9240, 
	0x0F32, 0x025A,
    	0x0F38, 0x05AA,
	0x0F2A, 0x4124, 
	0x004C, 0x0100, 
	0x331A, 0x0001,
	0x0A00, 0x0100,
};

#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
#endif
static void sensor_init(void)
{
	PK_DBG("addr_data_pair_init\n");
	table_write_cmos_sensor(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
}


static void preview_setting(void)
{
	PK_DBG("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	PK_DBG("4:3 binning size end\n");
}


static void capture_setting(kal_uint16 currefps)
{
	PK_DBG("full size start\n");
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	PK_DBG("full size  end\n");
}

static void normal_video_setting(void)
{
	PK_DBG("16:9 full size start\n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	PK_DBG("16:9 full size end\n");
}

static void hs_video_setting(void)
{
	PK_DBG("\n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	PK_DBG("\n");
    table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	PK_DBG("\n");
    table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
}

static void custom2_setting(void)
{
	PK_DBG("\n");
    table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
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
#if 0
	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	PK_DBG(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 2){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
#endif
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
				*sensor_id = return_sensor_id();
				if (*sensor_id == imgsensor_info.sensor_id) {				
					PK_DBG("i2c write id  : 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
					sensor_init();
					/*vivo caofeixiang add for CameraEM otp errorcode*/
					PK_DBG("cfx_add:start read eeprom ---vivo_main3_otp_read_when_power_on = %d\n", vivo_main3_otp_read_when_power_on);
					vivo_main3_otp_read_when_power_on = vivo_main3_otp_read();
					PK_DBG("cfx_add:end read eeprom ---vivo_main3_otp_read_when_power_on = %d,MAIN3_OTP_ERROR_CODE_HI846PD2062 =%d\n", vivo_main3_otp_read_when_power_on,MAIN3_OTP_ERROR_CODE_HI846PD2062);
					/*vivo caofeixiang add end*/
					return ERROR_NONE;
				}	
				PK_DBG("get_imgsensor_id Read sensor id fail, i2c write id: 0x%x,sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
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
	PK_DBG("[open]: PLATFORM:VIVO,MIPI 24LANE\n");
	PK_DBG("preview 1280*960@30fps,360Mbps/lane; capture 1560*1920@30fps,880Mbps/lane\n");
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			PK_DBG("open:Read sensor id fail open i2c write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
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

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
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
}				/*      open  */



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
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

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

	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;  
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	PK_DBG("Caputre fps:%d\n",imgsensor.current_fps);
	capture_setting(imgsensor.current_fps); 
	
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
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting();
	/*capture_setting(imgsensor.current_fps);*/
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
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
//	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
//	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

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

	custom1_setting();
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

	custom2_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2   */




static kal_uint32 get_resolution(
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
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
	PK_DBG("get_info -> scenario_id = %d\n", scenario_id);

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

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
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
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX =
			imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

		break;
		
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    PK_DBG("scenario_id = %d\n", scenario_id);
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
			PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate,
				imgsensor_info.cap.max_framerate / 10);

		frame_length = imgsensor_info.cap.pclk
			/ framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.cap.framelength + imgsensor.dummy_line;
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
		frame_length = imgsensor_info.custom1.pclk
			/ framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.custom1.framelength)
		? (frame_length - imgsensor_info.custom1.  framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.custom1.framelength + imgsensor.dummy_line;

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
		case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk
			/ framerate * 10 / imgsensor_info.custom2.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.custom2.framelength)
		? (frame_length - imgsensor_info.custom2.  framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.custom2.framelength + imgsensor.dummy_line;

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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	PK_DBG("enable: %d", enable);

	if (enable) { 
		  PK_DBG("enter color bar");            
		// 0x5E00[8]: 1 enable,  0 disable               
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor_16_16(0x0a04, 0x0141); 
		write_cmos_sensor_16_16(0x020a, 0x0002);	             
	} else {               
		// 0x5E00[8]: 1 enable,  0 disable               
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor_16_16(0x0a04, 0x0142);               
		write_cmos_sensor_16_16(0x020a, 0x0000);               
	}	 
	spin_lock(&imgsensor_drv_lock);
  imgsensor.test_pattern = enable;
  spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	PK_DBG("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_16(0x0a00, 0x0100);
	}
	else
		write_cmos_sensor_16_16(0x0a00, 0x0000);
	mdelay(5);
	return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	/*INT32 *feature_return_para_i32 = (INT32 *) feature_para;*/
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/* SET_PD_BLOCK_INFO_T *PDAFinfo; */
	/* SENSOR_VC_INFO_STRUCT *pvcinfo; */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

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
            write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
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
	case SENSOR_FEATURE_SET_HDR:
		PK_DBG("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
#if 1
        case SENSOR_FEATURE_GET_CUSTOM_INFO:
		    PK_DBG("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN3_OTP_ERROR_CODE_HI846PD2062:%d \n", *feature_data,MAIN3_OTP_ERROR_CODE_HI846PD2062);
			switch (*feature_data) {
				case 0:    //info type: otp state
				PK_DBG("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
				if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
				    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = MAIN3_OTP_ERROR_CODE_HI846PD2062;//otp_state
					memcpy( feature_data+2, sn_inf_main2_hi846pd2062, sizeof(MUINT32)*13); 
				    memcpy( feature_data+10, material_inf_main_hi846pd2062, sizeof(MUINT32)*4); 
					#if 0
							for (i = 0 ; i<12 ; i++ ){
							printk("sn_inf_main2_hi846pd2062[%d]= 0x%x\n", i, sn_inf_main2_hi846pd2062[i]);
							}
						
					#endif
					}
				break;
			}
			break;
#endif
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
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

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 HI846PD2062_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
