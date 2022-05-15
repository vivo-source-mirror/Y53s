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

/************************************************************************
 *
 * Filename:
 * ---------
 *     IMX499PD1913mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *-----------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *=====================================================
 ************************************************************************/
#define PFX "IMX499PD1913_camera_sensor"

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
#include "imx499pd1913mipiraw_Sensor.h"



static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX499PD1913_SENSOR_ID,	/* record sensor id defined in Kd_imgsensor.h */

	.checksum_value = 0xd6650427,	/*0xf86cfdf4, checksum value for Camera Auto Test */
	.pre = {
		.pclk = 277710000,	/*record different mode's pclk */
		.linelength = 5120,	/*record different mode's linelength */
		.framelength = 1808,	/*record different mode's framelength */
		.startx = 0,	/*record different mode's startx of grabwindow */
		.starty = 0,	/*record different mode's starty of grabwindow */
		.grabwindow_width = 2304,	/*2096,record different mode's width of grabwindow */
		.grabwindow_height = 1728,	/*1568 */
		.mipi_data_lp2hs_settle_dc = 40,	/*unit , ns */
		/*     following for GetDefaultFramerateByScenario()    */
		.mipi_pixel_rate = 333260000,
		.max_framerate = 300,
		},
	.cap = {
		.pclk = 570000000,
		.linelength = 5120,
		.framelength = 3710,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,	/* 4192, */
		.grabwindow_height = 3456,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 684000000,
		.max_framerate = 300,
		},
	.normal_video = {
		.pclk = 488000000,
		.linelength = 5120,
		.framelength = 3176,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 2592,
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 585600000,
		.max_framerate = 300,
		},
	.hs_video = {
		.pclk = 419200000,
		.linelength = 2560,	/* 2968, */
		.framelength = 1364,	/* 674, */
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,	/* 640 */
		.grabwindow_height = 1080,	/* 480 */
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 503040000,
		.max_framerate = 1200,
		},
	.slim_video = {
		.pclk = 90000000,
		.linelength = 720,	/* 9600,//2400, */
		.framelength = 4166,	/* 834,//3328, */
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 585600000,
		.max_framerate = 300,
		},
	.custom1 = { /*use cap_setting for stereo camera*/
		.pclk = 432000000,
		.linelength = 5120,
		.framelength = 3514,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,	/* 4192, */
		.grabwindow_height = 3456,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 518400000,
		.max_framerate = 240,
		},
	.custom2 = { /*cpy from preview*/
		.pclk = 432000000,
		.linelength = 5120,
		.framelength = 3514,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,	/* 4192, */
		.grabwindow_height = 3456,	/* 3104, */
		.mipi_data_lp2hs_settle_dc = 40,	/* unit , ns */
		.mipi_pixel_rate = 518400000,
		.max_framerate = 240,
		},
	
	.custom3 = {
		.pclk = 277710000,	/*record different mode's pclk */
		.linelength = 5120, /*record different mode's linelength */
		.framelength = 1808,	/*record different mode's framelength */
		.startx = 0,	/*record different mode's startx of grabwindow */
		.starty = 0,	/*record different mode's starty of grabwindow */
		.grabwindow_width = 2304,	/*2096,record different mode's width of grabwindow */
		.grabwindow_height = 1728,	/*1568 */
		.mipi_data_lp2hs_settle_dc = 40,	/*unit , ns */
		/*	   following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 333260000,
		.max_framerate = 300,
		},
	
	.custom4 = {
		.pclk = 277710000,	/*record different mode's pclk */
		.linelength = 5120, /*record different mode's linelength */
		.framelength = 1808,	/*record different mode's framelength */
		.startx = 0,	/*record different mode's startx of grabwindow */
		.starty = 0,	/*record different mode's starty of grabwindow */
		.grabwindow_width = 2304,	/*2096,record different mode's width of grabwindow */
		.grabwindow_height = 1728,	/*1568 */
		.mipi_data_lp2hs_settle_dc = 40,	/*unit , ns */
		/*	   following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 333260000,
		.max_framerate = 300,
		},
	.custom5 = {
		.pclk = 277710000,	/*record different mode's pclk */
		.linelength = 5120, /*record different mode's linelength */
		.framelength = 1808,	/*record different mode's framelength */
		.startx = 0,	/*record different mode's startx of grabwindow */
		.starty = 0,	/*record different mode's starty of grabwindow */
		.grabwindow_width = 2304,	/*2096,record different mode's width of grabwindow */
		.grabwindow_height = 1728,	/*1568 */
		.mipi_data_lp2hs_settle_dc = 40,	/*unit , ns */
		/*	   following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 333260000,
		.max_framerate = 300,
		},
	.margin = 18,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */

	/* max framelength by sensor register's limitation */
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	/* shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,
	/* sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.frame_time_delay_frame = 2,	/* The delay frame of setting frame length  */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 10,	  /*support sensor mode num*/

	.cap_delay_frame = 3,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,	/* enter high speed video  delay frame num */
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,



	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x34, 0xff},
	.i2c_speed = 1000,
	/* record sensor support all write id addr, only supprt 4must end with 0xff */
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,
	 * such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,

	/* current scenario id */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.hdr_mode = 0,/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  0, 0, 2304, 1728, 0, 0, 2304, 1728 }, /* Preview*/
{ 4608, 3456, 0, 0, 4608, 3456, 4608, 3456,  0, 0, 4608, 3456, 0, 0, 4608, 3456 },/*capture*/
{ 4608, 3456, 0, 432, 4608, 2592, 4608, 2592,  0, 0, 4608, 2592, 0, 0, 4608, 2592 },/*normal-video*/
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  192, 324, 1920, 1080, 0, 0, 1920, 1080},/*hs-video*/
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  0, 0, 2304, 1728, 0, 0, 2304, 1728 },/*slim-video*/
{ 4608, 3456, 0, 0, 4608, 3456, 4608, 3456,  0, 0, 4608, 3456, 0, 0, 4608, 3456 },/*custom1*/
{ 4608, 3456, 0, 0, 4608, 3456, 4608, 3456,  0, 0, 4608, 3456, 0, 0, 4608, 3456 },/*custom2*/
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  0, 0, 2304, 1728, 0, 0, 2304, 1728 }, /* custom3*/
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  0, 0, 2304, 1728, 0, 0, 2304, 1728 }, /* custom4*/
{ 4608, 3456, 0, 0, 4608, 3456, 2304, 1728,  0, 0, 2304, 1728, 0, 0, 2304, 1728 }, /* custom5*/
};

#define MAX_CIT_LSHIFT 7

/* add for imx499pd1913 pdaf */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 66,
	.i4OffsetY = 52,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,/*PD_DENSITY_X=8;*/
	.i4SubBlkH = 8,
	.i4PosL = {
		{68, 59}, {76, 59}, {84, 59}, {92, 59},
		{72, 67}, {80, 67}, {88, 67}, {96, 67},
		{68, 75}, {76, 75}, {84, 75}, {92, 75},
		{72, 83}, {80, 83}, {88, 83}, {96, 83}
		},
	.i4PosR = {
		{67, 59}, {75, 59}, {83, 59}, {91, 59},
		{71, 67}, {79, 67}, {87, 67}, {95, 67},
		{67, 75}, {75, 75}, {83, 75}, {91, 75},
		{71, 83}, {79, 83}, {87, 83}, {95, 83}
		},
	.i4BlockNumX = 140,
	.i4BlockNumY = 104,
	.i4LeFirst = 0,
	.i4Crop = {
		{0, 0}, {0, 0}, {0, 432}, {0, 0}, {0, 0},
		{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
	},
	.iMirrorFlip = 3,
};
 /*VC1 for L-PD(DT=0X34) , VC2 for R-PD(DT=0X31), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	 /* Preview mode setting */
	 {0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	  0x00, 0x2b, 0x0918, 0x06D2,/*VC0*/
	  0x00, 0x00, 0x000, 0x000,/*VC1*/
	  0x00, 0x31, 0x02BC, 0x019F*2,/*VC2 LPD+RPD*/
	  0x03, 0x00, 0x0000, 0x0000},/*VC3*/
	 /* Capture mode setting */
	 {0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	  0x00, 0x2b, 0x1230, 0x0DA8,/*VC0*/
	  0x00, 0x00, 0x000, 0x000,/*VC1*/
	  0x00, 0x31, 0x02BC, 0x01A0*2,/*VC2 LPD+RPD*/
	  0x03, 0x00, 0x0000, 0x0000},/*VC3*/
	 /* Video mode setting */
	 {0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	  0x00, 0x2b, 0x1230, 0x0DA8,/*VC0*/
	  0x00, 0x00, 0x000, 0x000,/*VC1*/
	  0x00, 0x31, 0x02BC, 0x0284,/*VC2 LPD+RPD*/
	  0x03, 0x00, 0x0000, 0x0000},/*VC3*/
};





static int vivo_otp_read_when_power_on;
extern int imx499pd1913_otp_read(void);
extern otp_error_code_t IMX499PD1913_OTP_ERROR_CODE;

MUINT32  sn_inf_main_imx499pd1913[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main_imx499pd1913[4];



static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
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
		/* Write when remain buffer size is less than 3 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id, 3,
					     imgsensor_info.i2c_speed);
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
	pr_debug("frame_length = %d, line_length = %d\n",
	    imgsensor.frame_length,
	    imgsensor.line_length);

	write_cmos_sensor_16_8(0x0104, 0x01);

	write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_16_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_16_8(0x0343, imgsensor.line_length & 0xFF);

	write_cmos_sensor_16_8(0x0104, 0x00);
} /* set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	pr_debug("framerate = %d, min framelength should enable %d\n",
			framerate,
			min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
	    ? frame_length : imgsensor.min_frame_length;

	imgsensor.dummy_line =
	    imgsensor.frame_length - imgsensor.min_frame_length;

	/* dummy_line = frame_length - imgsensor.min_frame_length; */
	/* if (dummy_line < 0) */
	/* imgsensor.dummy_line = 0; */
	/* else */
	/* imgsensor.dummy_line = dummy_line; */
	/* imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
		    imgsensor.frame_length - imgsensor.min_frame_length;

	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}				/*    set_max_framerate  */



/************************************************************************
 * FUNCTION
 *    set_shutter
 *
 * DESCRIPTION
 *    This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *    iShutter : exposured lines
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint16 l_shift = 1;

	pr_debug("Enter! shutter =%d, framelength =%d\n",shutter,imgsensor.frame_length);

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);


	/* if shutter bigger than frame_length, extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	/* long expsoure */
	if (shutter >(imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		pr_debug(" === hope enter long shutter shutter = %d\n", shutter); 
		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift)< (imgsensor_info.max_frame_length - imgsensor_info.margin))
				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			pr_debug("Unable to set such a long exposure %d, set to max\n",shutter);
			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		imgsensor.frame_length = shutter + imgsensor_info.margin;
		write_cmos_sensor_16_8(0x3100,read_cmos_sensor_16_8(0x3100) | (l_shift & 0x7));

		 pr_debug("0x3028 0x%x\n", read_cmos_sensor_16_8(0x3028)); 

	} else {
		write_cmos_sensor_16_8(0x3100, read_cmos_sensor_16_8(0x3100) & 0xf8);
	}

	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps =imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 237 && realtime_fps <= 243)
			set_max_framerate(236, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_8(0x0104, 0x01);
			write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor_16_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_8(0x0104, 0x01);
		write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_16_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_16_8(0x0104, 0x01);
	write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0203, shutter & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);
	pr_debug("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

} /* set_shutter */

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
			write_cmos_sensor_16_8(0x0104, 0x01);
			write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_16_8(0x0341,imgsensor.frame_length & 0xFF);
			write_cmos_sensor_16_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_8(0x0104, 0x01);
		write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_16_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_16_8(0x0104, 0x01);
	write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0203, shutter & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);
	pr_debug("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain;
	reg_gain = 1024 - (1024*64)/gain;
	pr_debug("imx499pd1913 gain =%d, reg_gain =%d\n", gain, reg_gain);
	return reg_gain;
}

/************************************************************************
 * FUNCTION
 *    set_gain
 *
 * DESCRIPTION
 *    This function is to set global gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/*  */
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		pr_debug("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	pr_debug("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_8(0x0104, 0x01);
	/* Global analog Gain for Long expo */
	write_cmos_sensor_16_8(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_16_8(0x0104, 0x00);


	return gain;
}				/*    set_gain  */



static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_V_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_H_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;

	case IMAGE_HV_MIRROR:
		write_cmos_sensor_16_8(0x0101, image_mirror);
		break;
	}
}
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
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
	pr_debug("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
	{
		write_cmos_sensor_16_8(0x0100, 0x01);
		mdelay(10);
	}
	else
	{
		write_cmos_sensor_16_8(0x0100, 0x00);
		mdelay(10);
	}
	return ERROR_NONE;
}

static kal_uint16 addr_data_pair_init[] = {
	0x0136, 0x18,
	0x0137, 0x00,	
	0x3C7E, 0x04,
	0x3C7F, 0x01,
	0x3F7F, 0x01,
	0x4D44, 0x00,
	0x4D45, 0x27,
	0x531C, 0x01,
	0x531D, 0x02,
	0x531E, 0x04,
	0x5928, 0x00,
	0x5929, 0x28,
	0x592A, 0x00,
	0x592B, 0x7E,
	0x592C, 0x00,
	0x592D, 0x3A,
	0x592E, 0x00,
	0x592F, 0x90,
	0x5930, 0x00,
	0x5931, 0x3F,
	0x5932, 0x00,
	0x5933, 0x95,
	0x5938, 0x00,
	0x5939, 0x20,
	0x593A, 0x00,
	0x593B, 0x76,
	0x5B38, 0x00,
	0x5B79, 0x02,
	0x5B7A, 0x07,
	0x5B88, 0x05,
	0x5B8D, 0x05,
	0x5C2E, 0x00,
	0x5C54, 0x00,
	0x6F6D, 0x01,
	0x79A0, 0x01,
	0x79A8, 0x00,
	0x79A9, 0x46,
	0x79AA, 0x01,
	0x79AD, 0x00,
	0x8169, 0x01,
	0x8359, 0x01,
	0x9004, 0x02,
	0x9200, 0x6A,
	0x9201, 0x22,
	0x9202, 0x6A,
	0x9203, 0x23,
	0x9302, 0x23,
	0x9312, 0x37,
	0x9316, 0x37,
	0xB046, 0x01,
	0xB048, 0x01,
	0xAA06,	0x3F,
	0xAA07,	0x05,
	0xAA08,	0x04,
	0xAA12,	0x3F,
	0xAA13,	0x04,
	0xAA14,	0x03,
	0xAB55,	0x02,
	0xAB57,	0x01,
	0xAB59,	0x01,
	0xABB4,	0x00,
	0xABB5,	0x01,
	0xABB6,	0x00,
	0xABB7,	0x01,
	0xABB8,	0x00,
	0xABB9,	0x01,
	0xE186, 0x31,	/*data id for L-PD */
	0xE1A6, 0x31,	/*data id for R-PD */
};


static kal_uint16 addr_data_pair_preview[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x14,
	0x0343, 0x00,
	0x0340, 0x07,
	0x0341, 0x10,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x02,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0xA5,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x0A,
	0x3F4C, 0x05,
	0x3F4D, 0x03,
	0x0408, 0x00,
	0x0409, 0x0C,
	0x040A, 0x00,
	0x040B, 0x08,
	0x040C, 0x09,
	0x040D, 0x00,
	0x040E, 0x06,
	0x040F, 0xC0,
	0x034C, 0x09,
	0x034D, 0x00,
	0x034E, 0x06,
	0x034F, 0xC0,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x07,
	0x0306, 0x00,
	0x0307, 0xF3,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x0D,
	0x0821, 0x04,
	0x0822, 0x92,
	0x0823, 0x49,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x02,
	0x4435, 0x30,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x75,
	0x3F78, 0x00,
	0x3F79, 0xF9,
	0x0202, 0x06,
	0x0203, 0xFE,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
};

static kal_uint16 addr_data_pair_capture[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x14,
	0x0343, 0x00,
	0x0340, 0x0E,
	0x0341, 0x7E,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x14,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0x93,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x0408, 0x00,
	0x0409, 0x18,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0D,
	0x040F, 0x80,
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0D,
	0x034F, 0x80,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x1D,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x1A,
	0x0821, 0xB8,
	0x0822, 0x00,
	0x0823, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x02,
	0x4435, 0x30,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x38,
	0x3F78, 0x01,
	0x3F79, 0x20,
	0x0202, 0x0E,
	0x0203, 0x6C,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
};

static kal_uint16 addr_data_pair_normal_video[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x14,
	0x0343, 0x00,
	0x0340, 0x0C,
	0x0341, 0x68,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xC4,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0B,
	0x034B, 0xE3,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x0408, 0x00,
	0x0409, 0x18,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0A,
	0x040F, 0x20,
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0A,
	0x034F, 0x20,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x02,
	0x0306, 0x00,
	0x0307, 0x7A,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x16,
	0x0821, 0xE0,
	0x0822, 0x00,
	0x0823, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x02,
	0x4435, 0x30,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x38,
	0x3F78, 0x01,
	0x3F79, 0x20,
	0x0202, 0x0C,
	0x0203, 0x56,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
};

static kal_uint16 addr_data_pair_hs_video[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x0A,
	0x0343, 0x00,
	0x0340, 0x05,
	0x0341, 0x54,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x01,
	0x0347, 0xB8,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0B,
	0x034B, 0xE7,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x0B,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x0408, 0x00,
	0x0409, 0xCC,
	0x040A, 0x00,
	0x040B, 0x70,
	0x040C, 0x07,
	0x040D, 0x80,
	0x040E, 0x04,
	0x040F, 0x38,
	0x034C, 0x07,
	0x034D, 0x80,
	0x034E, 0x04,
	0x034F, 0x38,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x05,
	0x0306, 0x01,
	0x0307, 0x06,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x13,
	0x0821, 0xA6,
	0x0822, 0x66,
	0x0823, 0x66,
	0x3E20, 0x02,
	0x3E3B, 0x00,
	0x4434, 0x00,
	0x4435, 0x00,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x88,
	0x3F78, 0x01,
	0x3F79, 0x1E,
	0x0202, 0x05,
	0x0203, 0x42,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
};

static kal_uint16 addr_data_pair_custom1[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x14,
	0x0343, 0x00,
	0x0340, 0x0D,
	0x0341, 0xBA,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x14,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0x93,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x0408, 0x00,
	0x0409, 0x18,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0D,
	0x040F, 0x80,
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0D,
	0x034F, 0x80,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xD8,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x14,
	0x0821, 0x40,
	0x0822, 0x00,
	0x0823, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x02,
	0x4435, 0x30,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x38,
	0x3F78, 0x01,
	0x3F79, 0x20,
	0x0202, 0x0D,
	0x0203, 0xA8,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,

};

static kal_uint16 addr_data_pair_custom2[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x14,
	0x0343, 0x00,
	0x0340, 0x0D,
	0x0341, 0xBA,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x14,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0x93,
	0x0381, 0x01,
	0x0383, 0x01,
	0x0385, 0x01,
	0x0387, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3F4C, 0x01,
	0x3F4D, 0x01,
	0x0408, 0x00,
	0x0409, 0x18,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x12,
	0x040D, 0x00,
	0x040E, 0x0D,
	0x040F, 0x80,
	0x034C, 0x12,
	0x034D, 0x00,
	0x034E, 0x0D,
	0x034F, 0x80,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x00,
	0x0307, 0xD8,
	0x030B, 0x01,
	0x030D, 0x02,
	0x030E, 0x01,
	0x030F, 0x22,
	0x0310, 0x00,
	0x0820, 0x14,
	0x0821, 0x40,
	0x0822, 0x00,
	0x0823, 0x00,
	0x3E20, 0x02,
	0x3E3B, 0x01,
	0x4434, 0x02,
	0x4435, 0x30,
	0x8271, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3C00, 0x00,
	0x3C01, 0x38,
	0x3F78, 0x01,
	0x3F79, 0x20,
	0x0202, 0x0D,
	0x0203, 0xA8,
	0x0204, 0x00,
	0x0205, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,

};

static void sensor_init(void)
{
	pr_debug("E\n");
	table_write_cmos_sensor(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
}				/*    sensor_init  */

static void preview_setting(void)
{
	pr_debug("2304x1728_30fps E\n");
	if(bIsLongExposure){
		write_cmos_sensor_16_8(0x3208, 0x03);
		write_cmos_sensor_16_8(0x3400, 0x04); 
		write_cmos_sensor_16_8(0x3410, 0x00); //;[0]long_exposure_mode_en
		
		write_cmos_sensor_16_8(0x3501, 0x0e);
		write_cmos_sensor_16_8(0x3502, 0x70);
		write_cmos_sensor_16_8(0x3508, 0x0f);
		write_cmos_sensor_16_8(0x3509, 0x80);
		write_cmos_sensor_16_8(0x350a, 0x01);
		write_cmos_sensor_16_8(0x350b, 0x00);
		write_cmos_sensor_16_8(0x350c, 0x00);		
		write_cmos_sensor_16_8(0x380e, 0x0e);
		write_cmos_sensor_16_8(0x380f, 0x78);
		write_cmos_sensor_16_8(0x3208, 0x13);
		write_cmos_sensor_16_8(0x3208, 0xa3);
			
		bIsLongExposure = KAL_FALSE;
	}
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
		
}				/*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	pr_debug("E! 4608x3456_30fps currefps:%d\n", currefps);
	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));

}

static void normal_video_setting(kal_uint16 currefps)
{
	pr_debug("E! 4608x2592_30fps  currefps:%d\n", currefps);
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
	pr_debug("E 1920x1080_120fps \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	pr_debug("E\n");
}

static void custom1_setting(void)
{
	pr_debug("4:3 full size start\n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	pr_debug("4:3 full size end\n");
}

static void custom2_setting(void)
{
	pr_debug("4:3 binning size start\n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	pr_debug("4:3 binning size end\n");
}

#if 0
static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		//write_cmos_sensor_16_8(0x5000, 0xdb);	/* disable lenc and otp_dpc */
		write_cmos_sensor_16_8(0x5081, 0x81);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		//write_cmos_sensor_16_8(0x5000, 0xff);	/* enable otp_dpc */
		write_cmos_sensor_16_8(0x5081, 0x80);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
#endif


static kal_uint32 return_sensor_id(void)
{
	return read_cmos_sensor_16_16(0x0016);
}


/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	pr_debug(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 2){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
		if ((*sensor_id == imgsensor_info.sensor_id)||((*sensor_id + 1) == imgsensor_info.sensor_id)) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				
				/*vivo lxd add for CameraEM otp errorcode*/
				pr_debug("lxd_add:start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = imx499pd1913_otp_read();
				pr_debug("lxd_add:end read eeprom ---vivo_otp_read_when_power_on = %d,IMX499PD1913_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, IMX499PD1913_OTP_ERROR_CODE);
				/*vivo lxd add end*/
				return ERROR_NONE;
			}
			pr_debug("Read sensor id fail:0x%x, id: 0x%x\n", imgsensor.i2c_write_id,
				*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
if ((*sensor_id != imgsensor_info.sensor_id)&&((*sensor_id + 1) != imgsensor_info.sensor_id)) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/************************************************************************
 * FUNCTION
 *    open
 *
 * DESCRIPTION
 *    This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if ((sensor_id == imgsensor_info.sensor_id)||(((sensor_id + 1) == imgsensor_info.sensor_id))) {
				pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			pr_debug("Read sensor id fail: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,
				sensor_id);
			retry--;
		} while (retry > 0);
		i++;
	if ((sensor_id == imgsensor_info.sensor_id)||((sensor_id + 1) == imgsensor_info.sensor_id))
			break;
		retry = 2;
	}
   if ((imgsensor_info.sensor_id != sensor_id)&&(imgsensor_info.sensor_id != (sensor_id + 1)))
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();
	
	/* for PD gain curve */

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.hdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
} /*    open  */



/************************************************************************
 * FUNCTION
 *    close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *    None
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static kal_uint32 close(void)
{
	write_cmos_sensor_16_8(0x0100, 0x00);/*stream off */
	return ERROR_NONE;
} /*    close  */


/************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *    This function start the sensor preview.
 *
 * PARAMETERS
 *    *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
} /*    preview   */

/************************************************************************
 * FUNCTION
 *    capture
 *
 * DESCRIPTION
 *    This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *    None
 *
 * GLOBALS AFFECTED
 *
 ************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	pr_debug("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	 capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
} /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/*preview_setting();*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
} /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
	pr_debug("E\n");

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

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom3   */

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


static kal_uint32 get_resolution(
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT * sensor_resolution)
{

	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;

	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;

	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;

	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;

	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;

	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = 
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;
		
	sensor_resolution->SensorCustom2Width = 
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width = 
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width = 
		imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = 
		imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =
		imgsensor_info.custom5.grabwindow_height;

	return ERROR_NONE;
}				/*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id); 



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

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */
		sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;

	sensor_info->SensorHorFOV = 63;
	sensor_info->SensorVerFOV = 49;

	sensor_info->HDR_Support = 0;/*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR */

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

	sensor_info->SensorMIPIDeskew = 1;

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

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("scenario_id = %d\n", scenario_id);
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
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
} /* control() */



/* This Function not used after ROME */
static kal_uint32 set_video_mode(UINT16 framerate)
{
	pr_debug("framerate = %d\n ", framerate);
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

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	pr_debug("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	 enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_debug("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
			pr_debug("frame_length %d < shutter %d",
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
		 ? (frame_length - imgsensor_info.normal_video.framelength) : 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			pr_debug("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps
			!= imgsensor_info.cap.max_framerate)
			pr_debug(
				"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
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
			pr_debug("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length > imgsensor_info.hs_video.framelength)
		    ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			pr_debug("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length > imgsensor_info.slim_video.framelength)
		   ? (frame_length - imgsensor_info.slim_video.framelength) : 0;

		imgsensor.frame_length =
		   imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			pr_debug("frame_length %d < shutter %d",
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
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	/* coding with  preview scenario by default */
	default:
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
			pr_debug("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		pr_debug("error scenario_id = %d, we use preview scenario\n",
			scenario_id);

		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*pr_debug("scenario_id = %d\n", scenario_id); */

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


static kal_uint32 imx499pd1913_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	pr_debug("%s\n", __func__);

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	pr_debug(
		"[%s] ABS_GAIN_GR:%d, grgain_32:%d\n, ABS_GAIN_R:%d, rgain_32:%d\n, ABS_GAIN_B:%d, bgain_32:%d,ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR, grgain_32,
		pSetSensorAWB->ABS_GAIN_R, rgain_32,
		pSetSensorAWB->ABS_GAIN_B, bgain_32,
		pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

	write_cmos_sensor_16_8(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor_16_8(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor_16_8(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor_16_8(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor_16_8(0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}

#if 0
static kal_uint32 get_sensor_temperature(void)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor_16_16(0x013a);

	if (temperature >= 0x0 && temperature <= 0x4F)
		temperature_convert = temperature;
	else if (temperature >= 0x50 && temperature <= 0x7F)
		temperature_convert = 80;
	else if (temperature >= 0x80 && temperature <= 0xEC)
		temperature_convert = -20;
	else
		temperature_convert = (INT8) temperature;

	/* pr_debug("temp_c(%d), read_reg(%d)\n",*/
	/*	temperature_convert, temperature); */

	return temperature_convert;
}
#endif

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	/*INT32 *feature_return_para_i32 = (INT32 *) feature_para;*/
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
		(struct SET_SENSOR_AWB_GAIN *) feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*pr_debug("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		/*night_mode((BOOL) (*feature_data));*/
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_16_8(
			sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:

		/* get the lens driver ID from EEPROM or
		 *just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
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
		set_auto_flicker_mode(
		    (BOOL) (*feature_data_16),
		    *(feature_data_16 + 1));

		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
		    (enum MSDK_SCENARIO_ID_ENUM) *feature_data,
		    *(feature_data + 1));

		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
		    (enum MSDK_SCENARIO_ID_ENUM) *feature_data,
		    (MUINT32 *) (uintptr_t) (*(feature_data + 1)));

		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		pr_debug("Please use EEPROM function\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		/*set_test_pattern_mode((BOOL) (*feature_data));*/
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32) *feature_data);

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy(
			    (void *)wininfo,
			    (void *)&imgsensor_winsize_info[1],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy(
			    (void *)wininfo,
			    (void *)&imgsensor_winsize_info[2],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy(
			    (void *)wininfo,
			    (void *)&imgsensor_winsize_info[3],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy(
			    (void *)wininfo,
			    (void *)&imgsensor_winsize_info[4],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;	
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[8],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy(
				(void *)wininfo,
				(void *)&imgsensor_winsize_info[9],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy(
			    (void *)wininfo,
			    (void *)&imgsensor_winsize_info[0],
			    sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));

			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("GET_PDAF_INFO :%lld\n", *feature_data);
		PDAFinfo = (struct SET_PD_BLOCK_INFO_T *) (uintptr_t)(*(feature_data + 1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_SET_PDAF_TYPE:
		break;


	case SENSOR_FEATURE_GET_VC_INFO:
		pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n",
			(UINT16) *feature_data);

		pvcinfo =
	    (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:			
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:	
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx499pd1913_awb_gain(pSetSensorAWB);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		pr_debug(
		    "SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu\n",
		    *feature_data);
		*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
		break;

		/*END OF HDR CMD */
		/*PDAF CMD */
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			imgsensor_pd_info. i4BlockNumY=80;
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *) (uintptr_t)(*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		
		case MSDK_SCENARIO_ID_CUSTOM3:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			 imgsensor_pd_info. i4BlockNumY=104;
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}

		pr_debug(
		    "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%llu=%d\n",
		    *feature_data,
		    *(MUINT32 *)(uintptr_t)(*(feature_data + 1)));

		break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
		
		case SENSOR_FEATURE_GET_CUSTOM_INFO:
			
		    pr_debug("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  IMX499PD1913_OTP_ERROR_CODE:%d \n", *feature_data,IMX499PD1913_OTP_ERROR_CODE);
			switch (*feature_data) {
				case 0:    //info type: otp state
				pr_debug("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
				if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
				    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = IMX499PD1913_OTP_ERROR_CODE;//otp_state
					memcpy(feature_data+2, sn_inf_main_imx499pd1913, sizeof(MUINT32)*13); 
					memcpy( feature_data+10, material_inf_main_imx499pd1913, sizeof(MUINT32)*4);
					#if 0
							for (i = 0 ; i<13 ; i++ ){
							printk("sn_inf_main_imx499pd1913[%d]= 0x%x\n", i, sn_inf_main_imx499pd1913[i]);
							}
						
					#endif
					}
					break;
			}
			
			break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) *feature_data, (UINT16) *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		/**feature_return_para_i32 = get_sensor_temperature();*/
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug(
		    "SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
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
}				/*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 IMX499PD1913_MIPI_RAW_SensorInit(
			struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    IMX499PD1913_MIPI_RAW_SensorInit    */
