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
 *	 ov13b10mipi_Sensor.c
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
 *----------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define PFX "OV13B10_camera_sensor"


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
#include "ov13b10mipiraw_Sensor.h"
#define LOG_INF(format, args...)    \
	pr_debug(PFX "[%s] " format, __func__, ##args)



#define MULTI_WRITE 1
#define ORINGNAL_VERSION 0

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020	/* trans# max is 255, each 4 bytes */
#else
#define I2C_BUFFER_LEN 4
#endif


static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV13B10_SENSOR_ID,

	.checksum_value = 0x3acb7e3a,

	.pre = {
		.pclk = 112310000,
		.linelength =  1176,
		.framelength = 3184,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 19,//unit , ns , 85
		.mipi_pixel_rate = 223600000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 3184,//3196,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 3120,
		.mipi_data_lp2hs_settle_dc = 19,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 300,
	},
#if 0	
	.normal_video = {
		.pclk = 112310000,
		.linelength =  1176,
		.framelength = 3184,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2104,
		.grabwindow_height = 1560,
		.mipi_data_lp2hs_settle_dc = 19,//unit , ns , 85
		.mipi_pixel_rate = 223600000,
		.max_framerate = 300,
	},
#endif
	.normal_video = {
		.pclk = 112310000,
		.linelength =  1176,
		.framelength = 3184,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4208,
		.grabwindow_height = 2368,
		.mipi_data_lp2hs_settle_dc = 19,//unit , ns , 85
		.mipi_pixel_rate = 447200000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 112310000,    
		.linelength = 1176,
		.framelength = 798,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 223600000,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 112310000,    
		.linelength = 1176,
		.framelength = 798,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 223600000,
		.max_framerate = 1200,
	},
	.custom1 = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 3978,//3240,3184
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 240,
	},
	.custom2 = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 4776,//3240,3184
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 200,
	},
	.custom3 = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 3980,//3240,3184
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 240,
	},
	.custom4 = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 4776,//3240,3184
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 200,
	},
	.custom5 = {
		.pclk = 112310000,
		.linelength = 1176,
		.framelength = 4776,//3240,3184
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3264,//4224,
		.grabwindow_height = 2448,//3136,
		.mipi_data_lp2hs_settle_dc = 90,
		.mipi_pixel_rate = 447200000,
		.max_framerate = 200,
	},
	.margin = 8,                    /*sensor framelength & shutter margin*/
	.min_shutter = 6,
	.min_gain = 64,
	.max_gain = 992,
	.min_gain_iso = 100,
	.gain_step = 4,
	.gain_type = 1,
	.exp_step = 2,
	/*max framelength by sensor register's limitation*/
	.max_frame_length = 0x7fff,
	/*shutter delay frame for AE cycle, 2 frame*/
	.ae_shut_delay_frame = 0,
	/*sensor gain delay frame for AE cycle,2 frame*/
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,    /*isp gain delay frame for AE cycle*/
	.frame_time_delay_frame = 2,	/* The delay frame of setting frame length  */
	.ihdr_support = 0,	            /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,         /*1,le first ; 0, se first*/
	.sensor_mode_num = 10,	        /*support sensor mode num*/

	.cap_delay_frame = 2,           /*enter capture delay frame num*/
	.pre_delay_frame = 2,           /*enter preview delay frame num*/
	.video_delay_frame = 2,         /*enter video delay frame num*/
	.hs_video_delay_frame = 2,   /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 2, /*enter slim video delay frame num*/
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,
	
	.isp_driving_current = ISP_DRIVING_2MA,     /*mclk driving current*/

	/*sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 26,         /*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,

	/*record sensor support all write id addr*/
	.i2c_addr_table = {0x6c, 0x20,0xff},
	.i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/

	/*IMGSENSOR_MODE enum value,record current sensor mode*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,			/*current shutter*/
	.gain = 0x100,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 300,
	/*auto flicker enable: KAL_FALSE for disable auto flicker*/
	.autoflicker_en = KAL_FALSE,
	/*test pattern mode or not. KAL_FALSE for in test pattern mode*/
	.test_pattern = KAL_FALSE,
	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	/*sensor need support LE, SE with HDR feature*/
	.ihdr_mode = 0,
	.pdaf_mode = 0,
	.i2c_write_id = 0x6c,  /*record current sensor's i2c write id*/

};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568,   4,   4, 2104, 1560, 0, 0, 2104, 1560}, //preview
	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   8,   8, 4208, 3120, 0, 0, 4208, 3120}, //capture
	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   8,   384, 4208, 2368, 0, 0, 4208, 2368},  //normal video
//	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568,   4,   4, 2104, 1560, 0, 0, 2104, 1560},  //normal video
//	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568, 544, 400, 1024,  768, 0, 0, 1024,  768}, //hs_video

//	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568, 544, 400, 1024,  768, 0, 0, 1024,  768}, //slim_video

	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568, 416, 424, 1280,  720, 0, 0, 1280,  720}, //hs_video

	{4224, 3136,  0,  0,  4224, 3136, 2112, 1568, 416, 424, 1280,  720, 0, 0, 1280,  720}, //slim_video


	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   480, 344,  3264, 2448, 0,   0,   3264, 2448},//custom1
	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   480, 344,  3264, 2448, 0,   0,   3264, 2448},//custom2
 	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   480, 344,  3264, 2448, 0,   0,   3264, 2448},//custom3
	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   480, 344,  3264, 2448, 0,   0,   3264, 2448},//custom4
	{4224, 3136,  0,  0,  4224, 3136, 4224, 3136,   480, 344,  3264, 2448, 0,   0,   3264, 2448},//custom5	
};

/*PD information update*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
	 .i4OffsetX = 8,
	 .i4OffsetY = 8,
	 .i4PitchX	= 32,
	 .i4PitchY	= 32,
	 .i4PairNum  = 8,//32*32 is cropped into 16*8 sub block of 8 pairnum, per sub block includes just only one pd pair pixels
	 .i4SubBlkW  =16,
	 .i4SubBlkH  =8,
	 .i4PosL = {{18, 10}, {34, 10}, {10, 22}, {26, 22},
		{18, 26}, {34, 26}, {10, 38}, {26, 38} },
	 .i4PosR = {{18, 14}, {34, 14}, {10, 18}, {26, 18},
		{18, 30}, {34, 30}, {10, 34}, {26, 34} },
	 .iMirrorFlip = 0,
	 .i4BlockNumX = 131,
	 .i4BlockNumY = 97,

};

/*vivo hope add for Camera otp errorcode*/
extern int ov13b10_vivo_otp_read(void);
extern bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data);
static int vivo_otp_read_when_power_on = 0;
MUINT32  sn_inf_main_ov13b10[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main_ov13b10[4];  
extern otp_error_code_t OV13B10_OTP_ERROR_CODE;
extern unsigned int is_atboot;
extern unsigned  int ov13b10_flag;

/*vivo hope add end*/
#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF),
		(char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}
#endif
static kal_uint16 read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	pr_info(PFX "[%s] dummyline = %d, dummypixels = %d frame_length =\n",
		 __FUNCTION__,imgsensor.dummy_line, imgsensor.dummy_pixel,imgsensor.frame_length);
	/*return; //for test*/
    write_cmos_sensor_8(0x380c, imgsensor.line_length >> 8);
    write_cmos_sensor_8(0x380d, imgsensor.line_length & 0xFF);
    write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0x7f);
    write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
}	/*	set_dummy  */

static kal_uint16 ov13b10_table_write_cmos_sensor(
					kal_uint16 *para, kal_uint32 len)
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
		if ((I2C_BUFFER_LEN - tosend) < 3 ||
			len == IDX ||
			addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend,
				imgsensor.i2c_write_id,
				3, imgsensor_info.i2c_speed);

			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
			frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length -
		imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length -
			imgsensor.min_frame_length;
	}
	if (min_framelength_en){
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}

static void write_shutter(kal_uint32 shutter)
{
    //check
#if 0
	kal_uint16  a_0x3500 =0;
	kal_uint16  b_0x3501 =0;
	kal_uint16  c_0x3502 =0;
	kal_uint16  d_0x3822 =0;
	kal_uint16  e_0x380e =0;
	kal_uint16  f_0x380f =0;
#endif

	kal_uint16 realtime_fps = 0;
    pr_info(PFX "[%s] write shutter :%d\n",__FUNCTION__,shutter);

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin){
        imgsensor.frame_length = shutter + imgsensor_info.margin;//increase current frame_length that makes shutter <= frame_length - margin.
    }
	else{
        imgsensor.frame_length = imgsensor.min_frame_length;
	}
	
	if (imgsensor.frame_length > imgsensor_info.max_frame_length){
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;

	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	
   	pr_info(PFX "[%s] write shutter :%d frame_length =%d  autoflicker_en =%d \n",__FUNCTION__,shutter,imgsensor.frame_length,imgsensor.autoflicker_en);
	/* ###groupon### */
 	write_cmos_sensor_8(0x3208, 0x01);
	/* ###groupon### */

	if(shutter > 32759) {
		/*enter long exposure mode */
		pr_info("enter long exposure mode\n");
		LOG_INF("Calc long exposure  +\n");

 		write_cmos_sensor_8(0x3822, 0x04);
		write_cmos_sensor_8(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor_8(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter)  & 0xFF);
		bIsLongExposure = KAL_TRUE;
	} else {

		pr_info("normal exposure mode  +\n");
		shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
		
		/*Warning : shutter must be even. Odd might happen Unexpected Results */

    		write_cmos_sensor_8(0x3822, 0x14);

		write_cmos_sensor_8(0x3500, (shutter >> 16) & 0xFF);
		write_cmos_sensor_8(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter)  & 0xFF);

		if (imgsensor.autoflicker_en == KAL_TRUE) {
			realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
			pr_info(PFX "[%s] realtime_fps :%d\n",__FUNCTION__,realtime_fps);
			if (realtime_fps >= 297 && realtime_fps <= 305) {
				realtime_fps = 296;
				set_max_framerate(realtime_fps, 0);
			} else if (realtime_fps >= 147 && realtime_fps <= 150) {
				realtime_fps = 146;
				set_max_framerate(realtime_fps, 0);
			} else	{
				/* Extend frame length */
				write_cmos_sensor_8(0x380e,
					(imgsensor.frame_length >> 8) & 0x7f);
				write_cmos_sensor_8(0x380f,
					imgsensor.frame_length & 0xFF);
			}
		} else	{
			write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0x7f);
			write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
		}


	}
	/* ###groupoff### */
 	write_cmos_sensor_8(0x3208, 0x11);
	write_cmos_sensor_8(0x3208, 0xA1);
	/* ###groupoff### */

	LOG_INF("Add for N3D! shutterlzl =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);

#if 0
	a_0x3500 =	read_cmos_sensor_8(0x3500);
	b_0x3501 =	read_cmos_sensor_8(0x3501);
	c_0x3502 =	read_cmos_sensor_8(0x3502);
	d_0x3822 =	read_cmos_sensor_8(0x3822);
	e_0x380e =	read_cmos_sensor_8(0x380e);
	f_0x380f =	read_cmos_sensor_8(0x380f);
    	pr_info(PFX "[%s] read  a_0x3500=%d  b_0x3501=%d c_0x3502 =%d shutter=%d  0x%x\n",__FUNCTION__,a_0x3500,b_0x3501,c_0x3502,((a_0x3500<<16)|(b_0x3501<<8)|c_0x3502),((a_0x3500<<16)|(b_0x3501<<8)|c_0x3502));
	pr_info(PFX "[%s] read  d_0x3822=%d  ", __FUNCTION__,d_0x3822);
	pr_info(PFX "[%s] read  frame_length =%d e_0x380e=%d  e_0x380f=%d", __FUNCTION__,(e_0x380e<<8|f_0x380f),e_0x380e,f_0x380f);
#endif
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

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;

	/* platform 1xgain = 64, sensor driver 1*gain = 0x80 */
	iReg = gain*256 / BASEGAIN;

	/* sensor 1xGain */
	if (iReg < 0x100){
		iReg = 0x100;
	}

	/* sensor 15.5xGain */
	if (iReg > 0xf80){
		iReg = 0xf80;
	}
	
	return iReg;
}

/*************************************************************************
 * FUNCTION
 * set_gain
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
	unsigned long flags;
#if 0
	kal_uint16 reg_gain_3508;
	kal_uint16 reg_gain_3509;
#endif

	reg_gain = gain2reg(gain);
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.gain = reg_gain;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    pr_info(PFX "[%s]:start gain = %d , reg_gain = 0x%x\n",__FUNCTION__,gain, reg_gain);

	write_cmos_sensor_8(0x3508, (reg_gain >> 8));
	write_cmos_sensor_8(0x3509, (reg_gain&0xff));

#if 0
	reg_gain_3508 =read_cmos_sensor_8(0x3508);
	reg_gain_3509 =read_cmos_sensor_8(0x3509);
    pr_info(PFX "[%s]: read  reg_gain_3508 = %d , reg_gain_3509 = %d reg_gain = 0x%x\n",__FUNCTION__,reg_gain_3508, reg_gain_3509,(reg_gain_3508<<8|reg_gain_3509));
#endif
	return gain;
}

static void set_mirror_flip(kal_uint8 image_mirror)
{

/*	kal_uint8 itemp;*/

	LOG_INF("image_mirror = %d\n", image_mirror);

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
#if MULTI_WRITE
kal_uint16 addr_data_pair_init_ov13b10[] = {
0x0103, 0x01,
0x0303, 0x02,
0x0305, 0x56,
0x0321, 0x00,
0x0323, 0x04,
0x0324, 0x01,
0x0325, 0x37,
0x0326, 0x81,
0x0327, 0x04,
0x3011, 0x7c,
0x3012, 0x07,
0x3013, 0x32,
0x3107, 0x23,
0x3501, 0x0c,
0x3502, 0x10,
0x3504, 0x08,
0x3508, 0x07,
0x3509, 0xc0,
0x3600, 0x16,
0x3601, 0x54,
0x3612, 0x4e,
0x3620, 0x00,
0x3621, 0x68,
0x3622, 0x66,
0x3623, 0x03,
0x3662, 0x92,
0x3666, 0xbb,
0x3667, 0x44,
0x366e, 0xff,
0x366f, 0xf3,
0x3675, 0x44,
0x3676, 0x00,
0x367f, 0xe9,
0x3681, 0x32,
0x3682, 0x1f,
0x3683, 0x0b,
0x3684, 0x0b,
0x3704, 0x0f,
0x3706, 0x40,
0x3708, 0x3b,
0x3709, 0x72,
0x370b, 0xa2,
0x3714, 0x24,
0x371a, 0x3e,
0x3725, 0x42,
0x3739, 0x12,
0x3767, 0x00,
0x377a, 0x0d,
0x3789, 0x18,
0x3790, 0x40,
0x3791, 0xa2,
0x37c2, 0x04,
0x37c3, 0xf1,
0x37d9, 0x0c,
0x37da, 0x02,
0x37dc, 0x02,
0x37e1, 0x04,
0x37e2, 0x0a,
0x3800, 0x00,
0x3801, 0x00,
0x3802, 0x00,
0x3803, 0x08,
0x3804, 0x10,
0x3805, 0x8f,
0x3806, 0x0c,
0x3807, 0x47,
0x3808, 0x10,
0x3809, 0x70,
0x380a, 0x0c,
0x380b, 0x30,
0x380c, 0x04,
0x380d, 0x98,
0x380e, 0x0c,
0x380f, 0x70,
0x3811, 0x0f,
0x3813, 0x08,
0x3814, 0x01,
0x3815, 0x01,
0x3816, 0x01,
0x3817, 0x01,
0x381f, 0x08,
0x3820, 0x88,
0x3821, 0x00,
0x3822, 0x14,
0x3823, 0x18,
0x3827, 0x01,
0x382e, 0xe6,
0x3c80, 0x00,
0x3c87, 0x01,
0x3c8c, 0x19,
0x3c8d, 0x1c,
0x3ca0, 0x00,
0x3ca1, 0x00,
0x3ca2, 0x00,
0x3ca3, 0x00,
0x3ca4, 0x50,
0x3ca5, 0x11,
0x3ca6, 0x01,
0x3ca7, 0x00,
0x3ca8, 0x00,
0x4008, 0x02,
0x4009, 0x0f,
0x400a, 0x01,
0x400b, 0x19,
0x4011, 0x21,
0x4017, 0x08,
0x4019, 0x04,
0x401a, 0x58,
0x4032, 0x1e,
0x4050, 0x02,
0x4051, 0x09,
0x405e, 0x00,
0x4066, 0x02,
0x4501, 0x00,
0x4502, 0x10,
0x4505, 0x00,
0x4800, 0x64,
0x481b, 0x3e,
0x481f, 0x30,
0x4825, 0x34,
0x4837, 0x0e,
0x484b, 0x01,
0x4883, 0x02,
0x5000, 0xff,
0x5001, 0x0f,
0x5045, 0x20,
0x5046, 0x20,
0x5047, 0xa4,
0x5048, 0x20,
0x5049, 0xa4, 
0x0100, 0x01
};
#endif

static kal_uint32 streaming_control(kal_bool enable)
{
	pr_info(PFX "[%s] streaming_enable(0=Sw Standby,1=streaming): %d\n", __FUNCTION__,enable);
	if (enable) {
		write_cmos_sensor_8(0x0100, 0x01);
	}
	else{
		write_cmos_sensor_8(0x0100, 0x00);
	}
	
	mdelay(10);
	return ERROR_NONE;
}
static void set_shutter_frame_length(kal_uint32 shutter,
			kal_uint32 frame_length)
{
	unsigned long flags;
	kal_uint32 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	    pr_info(PFX "[%s]shutter =%d, framelength =%d, realtime_fps =%d \n",__FUNCTION__,shutter, imgsensor.frame_length, realtime_fps);
	/*Change frame time*/
	if (frame_length > 1){
		imgsensor.frame_length = frame_length;
	}

	if (shutter > imgsensor.frame_length - imgsensor_info.margin){
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	}

	if (imgsensor.frame_length > imgsensor_info.max_frame_length){
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)){
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);
	}
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305){
			set_max_framerate(296, 0);
		}
		else if (realtime_fps >= 147 && realtime_fps <= 150){
			set_max_framerate(146, 0);
		}
		else {
			/* Extend frame length*/
            write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0x7f);
            write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
	
		/* Extend frame length*/
            write_cmos_sensor_8(0x380e, (imgsensor.frame_length >> 8) & 0x7f);
            write_cmos_sensor_8(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter*/
		write_cmos_sensor_8(0x3500, (shutter >> 16) & 0xF);
		write_cmos_sensor_8(0x3501, (shutter >> 8) & 0xFF);
		write_cmos_sensor_8(0x3502, (shutter)  & 0xFF);


	    pr_info(PFX "[%s]shutter =%d, framelength =%d, realtime_fps =%d \n",__FUNCTION__,shutter, imgsensor.frame_length, realtime_fps);
}	

static void sensor_init(void)
{
	LOG_INF("sensor_init() E\n");
	mdelay(10);
	LOG_INF("MULTI_WRITE\n");
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_init_ov13b10,
		sizeof(addr_data_pair_init_ov13b10) / sizeof(kal_uint16));
	LOG_INF("sensor_init() end\n");
}	/*	sensor_init  */

#if MULTI_WRITE
kal_uint16 addr_data_pair_preview_ov13b10[] = {
	0x0305, 0x2b,
	0x0327, 0x04,
	0x3501, 0x06,
	0x3502, 0x10,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x68,
	0x3622, 0x66,
	0x3623, 0x03,
	0x3662, 0x88,
	0x3667, 0x44,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x3789, 0x18,
	0x37c2, 0x14,
	0x37c3, 0xf1,
	0x37d9, 0x06,
	0x37e2, 0x0c,
	0x37e4, 0x00,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x08,
	0x3809, 0x38,
	0x380a, 0x06,
	0x380b, 0x18,
	0x380e, 0x0c,
	0x380f, 0x70,
	0x3811, 0x08,
	0x3813, 0x03,
	0x3814, 0x03,
	0x3816, 0x03,
	0x381f, 0x08,
	0x3820, 0xb3,//0x8b
	0x3c8c, 0x18,
	0x3f02, 0x0f,
	0x3f03, 0x00,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4066, 0x02,
	0x4501, 0x08,
	0x4502, 0x10,
	0x4505, 0x04,
	0x4837, 0x1d,
	0x5000, 0xfd,
	0x5001, 0x0d,
	0x5049, 0xa4,


};
#endif
static void preview_setting(void)
{
	/*Preview 2320*1744 30fps 24M MCLK 4lane 1488Mbps/lane*/
	/*preview 30.01fps*/
	LOG_INF("preview_setting() E\n");
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_preview_ov13b10,
		sizeof(addr_data_pair_preview_ov13b10) / sizeof(kal_uint16));

	LOG_INF("preview_setting() end\n");
}	/*	preview_setting  */

#if MULTI_WRITE
kal_uint16 addr_data_pair_capture_15fps_ov13b10[] = {
	0x0305, 0x56,
	0x0327, 0x05,
	0x3501, 0x0c,
	0x3502, 0x10,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x28,
	0x3622, 0xe6,
	0x3623, 0x00,
	0x3662, 0x92,
	0x3667, 0x44,
	0x3714, 0x24,
	0x371a, 0x3e,
	0x3739, 0x12,
	0x3789, 0x18,
	0x37c2, 0x04,
	0x37c3, 0xf1,
	0x37d9, 0x0c,
	0x37e2, 0x0a,
	0x37e4, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x10,
	0x3809, 0x70,
	0x380a, 0x0c,
	0x380b, 0x30,
	0x380e, 0x18,
	0x380f, 0xde,
	0x3811, 0x10,
	0x3813, 0x07,
	0x3814, 0x01,
	0x3816, 0x01,
	0x381f, 0x08,
	0x3820, 0xb0,//0x88
	0x3c8c, 0x19,
	0x3f02, 0x2a,
	0x3f03, 0x10,
	0x4008, 0x02,
	0x4009, 0x0f,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x02,
	0x4051, 0x09,
	0x4066, 0x02,
	0x4501, 0x00,
	0x4502, 0x10,
	0x4505, 0x00,
	0x4837, 0x0e,
	0x5000, 0xff,
	0x5001, 0x0f,
	0x5049, 0xa4,
};

kal_uint16 addr_data_pair_capture_30fps_ov13b10[] = {
	0x0305, 0x56,
	0x0327, 0x05,
	0x3501, 0x0c,
	0x3502, 0x10,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x28,
	0x3622, 0xe6,
	0x3623, 0x00,
	0x3662, 0x92,
	0x3667, 0x44,
	0x3714, 0x24,
	0x371a, 0x3e,
	0x3739, 0x12,
	0x3789, 0x18,
	0x37c2, 0x04,
	0x37c3, 0xf1,
	0x37d9, 0x0c,
	0x37e2, 0x0a,
	0x37e4, 0x04,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x10,
	0x3809, 0x70,
	0x380a, 0x0c,
	0x380b, 0x30,
	0x380e, 0x0c,
	0x380f, 0x70,
	0x3811, 0x10,
	0x3813, 0x07,
	0x3814, 0x01,
	0x3816, 0x01,
	0x381f, 0x08,
	0x3820, 0xb0,//0x88
	0x3c8c, 0x19,
	0x3f02, 0x2a,
	0x3f03, 0x10,
	0x4008, 0x02,
	0x4009, 0x0f,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x02,
	0x4051, 0x09,
	0x4066, 0x02,
	0x4501, 0x00,
	0x4502, 0x10,
	0x4505, 0x00,
	0x4837, 0x0e,
	0x5000, 0xff,
	0x5001, 0x0f,
	0x5049, 0xa4,
};
#endif
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture_setting() E! currefps:%d\n", currefps);
	if (currefps == 150) {
		ov13b10_table_write_cmos_sensor(
			addr_data_pair_capture_15fps_ov13b10,
			sizeof(addr_data_pair_capture_15fps_ov13b10) /
			sizeof(kal_uint16));
	} else {
		ov13b10_table_write_cmos_sensor(
			addr_data_pair_capture_30fps_ov13b10,
			sizeof(addr_data_pair_capture_30fps_ov13b10) /
			sizeof(kal_uint16));
	}
}

#if MULTI_WRITE
#if 0
kal_uint16 addr_data_pair_video_ov13b10[] = {//4208x3120
	0x0305, 0x2b,
	0x0327, 0x04,
	0x3501, 0x06,
	0x3502, 0x10,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x68,
	0x3622, 0x66,
	0x3623, 0x03,
	0x3662, 0x88,
	0x3667, 0x44,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x3789, 0x18,
	0x37c2, 0x14,
	0x37c3, 0xf1,
	0x37d9, 0x06,
	0x37e2, 0x0c,
	0x37e4, 0x00,
	0x3800, 0x00,
	0x3801, 0x00,
	0x3802, 0x00,
	0x3803, 0x08,
	0x3804, 0x10,
	0x3805, 0x8f,
	0x3806, 0x0c,
	0x3807, 0x47,
	0x3808, 0x08,
	0x3809, 0x38,
	0x380a, 0x06,
	0x380b, 0x18,
	0x380e, 0x0c,
	0x380f, 0x70,
	0x3811, 0x08,
	0x3813, 0x03,
	0x3814, 0x03,
	0x3816, 0x03,
	0x381f, 0x08,
	0x3820, 0xb3,//0x8b
	0x3c8c, 0x18,
	0x3f02, 0x0f,
	0x3f03, 0x00,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4066, 0x02,
	0x4501, 0x08,
	0x4502, 0x10,
	0x4505, 0x04,
	0x4837, 0x1d,
	0x5000, 0xfd,
	0x5001, 0x0d,
	0x5049, 0xa4
};
#endif
kal_uint16 addr_data_pair_video_ov13b10[] = {//4208x2368
0x0305, 0x56, 
0x0327, 0x05, 
0x3501, 0x0c, 
0x3502, 0x10, 
0x3612, 0x4e,
0x3620, 0x00, 
0x3621, 0x28, 
0x3622, 0xe6, 
0x3623, 0x00, 
0x3662, 0x92,
0x3667, 0x44, 
0x3714, 0x24, 
0x371a, 0x3e,
0x3739, 0x12, 
0x3789, 0x18,
0x37c2, 0x04, 
0x37c3, 0xf1, 
0x37d9, 0x0c, 
0x37e2, 0x0a,
0x37e4, 0x04,
0x3800, 0x00, 
0x3801, 0x00,  //00
0x3802, 0x01,  //01
0x3803, 0x78,  //78 //376
0x3804, 0x10, 
0x3805, 0x8f,  // 4239
0x3806, 0x0a,  //0a
0x3807, 0xd7,  //d7 //2775
0x3808, 0x10, 
0x3809, 0x70,  // 4208
0x380a, 0x09,  //09
0x380b, 0x40,  //40 //2368
0x380e, 0x0c, 
0x380f, 0x70, 
0x3811, 0x10, 
0x3813, 0x0f,   //0f
0x3814, 0x01, 
0x3816, 0x01, 
0x381f, 0x08, 
0x3820, 0xb0,
0x3c8c, 0x19, 
0x3f02, 0x2a,
0x3f03, 0x10,
0x4008, 0x02, 
0x4009, 0x0f, 
0x4011, 0x21, 
0x4032, 0x1e, 
0x4050, 0x02, 
0x4051, 0x09, 
0x4066, 0x02,
0x4501, 0x00,
0x4502, 0x10, 
0x4505, 0x00, 
0x4837, 0x0e, 
0x5000, 0xff, 
0x5001, 0x0f,
0x5049, 0xa4, 
};
#endif
static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video cc\n");
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_video_ov13b10,
		sizeof(addr_data_pair_video_ov13b10) /
		sizeof(kal_uint16));
}
 

 
#if MULTI_WRITE
kal_uint16 addr_data_pair_hs_video_ov13b10[] = {
	0x0305, 0x2b,
	0x0327, 0x04,
	0x3501, 0x03,
	0x3502, 0x00,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x68,
	0x3622, 0x66,
	0x3623, 0x03,
	0x3662, 0x88,
	0x3667, 0x44,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x3789, 0x18,
	0x37c2, 0x14,
	0x37c3, 0xf1,
	0x37d9, 0x06,
	0x37e2, 0x0c,
	0x37e4, 0x00,
	0x3800, 0x03,
	0x3801, 0x30,
	0x3802, 0x03,
	0x3803, 0x48,
	0x3804, 0x0d,
	0x3805, 0x5f,
	0x3806, 0x09,
	0x3807, 0x07,
	0x3808, 0x05,
	0x3809, 0x00,
	0x380a, 0x02,
	0x380b, 0xd0,
	0x380e, 0x03,
	0x380f, 0x1e,
	0x3811, 0x0c,
	0x3813, 0x07,
	0x3814, 0x03,
	0x3816, 0x03,
	0x381f, 0x08,
	0x3820, 0xb3,//8b  b4
	0x3c8c, 0x18,
	0x3f02, 0x0f,
	0x3f02, 0x00,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4066, 0x02,
	0x4501, 0x08,
	0x4502, 0x10,
	0x4505, 0x04,
	0x4837, 0x1d,
	0x5000, 0xfd,
	0x5001, 0x0d,
	0x5049, 0xa4
};
#endif
#if MULTI_WRITE
kal_uint16 addr_data_pair_slim_video_ov13b10[] = {
	0x0305, 0x2b,
	0x0327, 0x04,
	0x3501, 0x03,
	0x3502, 0x00,
	0x3612, 0x4e,
	0x3620, 0x00,
	0x3621, 0x68,
	0x3622, 0x66,
	0x3623, 0x03,
	0x3662, 0x88,
	0x3667, 0x44,
	0x3714, 0x28,
	0x371a, 0x3e,
	0x3739, 0x10,
	0x3789, 0x18,
	0x37c2, 0x14,
	0x37c3, 0xf1,
	0x37d9, 0x06,
	0x37e2, 0x0c,
	0x37e4, 0x00,
	0x3800, 0x03,
	0x3801, 0x30,
	0x3802, 0x03,
	0x3803, 0x48,
	0x3804, 0x0d,
	0x3805, 0x5f,
	0x3806, 0x09,
	0x3807, 0x07,
	0x3808, 0x05,
	0x3809, 0x00,
	0x380a, 0x02,
	0x380b, 0xd0,
	0x380e, 0x03,
	0x380f, 0x1e,
	0x3811, 0x0c,
	0x3813, 0x07,
	0x3814, 0x03,
	0x3816, 0x03,
	0x381f, 0x08,
	0x3820, 0xb3,//0x8b
	0x3c8c, 0x18,
	0x3f02, 0x0f,
	0x3f02, 0x00,
	0x4008, 0x00,
	0x4009, 0x05,
	0x4011, 0x21,
	0x4032, 0x1e,
	0x4050, 0x00,
	0x4051, 0x05,
	0x4066, 0x02,
	0x4501, 0x08,
	0x4502, 0x10,
	0x4505, 0x04,
	0x4837, 0x1d,
	0x5000, 0xfd,
	0x5001, 0x0d,
	0x5049, 0xa4
};
#endif


static void hs_video_setting(void)
{
	LOG_INF("hs_video_setting() E\n");
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_hs_video_ov13b10,
		sizeof(addr_data_pair_hs_video_ov13b10) /
		sizeof(kal_uint16));
}


static void slim_video_setting(void)
{
	LOG_INF("slim_video_setting() E\n");
	ov13b10_table_write_cmos_sensor(
		addr_data_pair_slim_video_ov13b10,
		sizeof(addr_data_pair_slim_video_ov13b10) /
		sizeof(kal_uint16));
	LOG_INF("slim_video_setting() end\n");
}

static void custom1_setting(kal_uint16 currefps)
{

	LOG_INF("custom1_setting() start! \n");
	write_cmos_sensor_8(0x0305, 0x56); 
	write_cmos_sensor_8(0x0327, 0x05); 
	write_cmos_sensor_8(0x3501, 0x0c); 
	write_cmos_sensor_8(0x3502, 0x10); 
	write_cmos_sensor_8(0x3612, 0x4e);
	write_cmos_sensor_8(0x3620, 0x00); 
	write_cmos_sensor_8(0x3621, 0x28); 
	write_cmos_sensor_8(0x3622, 0xe6); 
	write_cmos_sensor_8(0x3623, 0x00); 
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3667, 0x44); 
	write_cmos_sensor_8(0x3714, 0x24); 
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x12); 
	write_cmos_sensor_8(0x3789, 0x18);
	write_cmos_sensor_8(0x37c2, 0x04); 
	write_cmos_sensor_8(0x37c3, 0xf1); 
	write_cmos_sensor_8(0x37d9, 0x0c); 
	write_cmos_sensor_8(0x37e2, 0x0a);
	write_cmos_sensor_8(0x37e4, 0x04);
	write_cmos_sensor_8(0x3800, 0x01); 
	write_cmos_sensor_8(0x3801, 0xe0); 
	write_cmos_sensor_8(0x3802, 0x01); 
	write_cmos_sensor_8(0x3803, 0x48); 
	write_cmos_sensor_8(0x3804, 0x0e); 
	write_cmos_sensor_8(0x3805, 0xaf); 
	write_cmos_sensor_8(0x3806, 0x0a); 
	write_cmos_sensor_8(0x3807, 0xf7); 
	write_cmos_sensor_8(0x3808, 0x0C); 
	write_cmos_sensor_8(0x3809, 0xC0); 
	write_cmos_sensor_8(0x380a, 0x09); 
	write_cmos_sensor_8(0x380b, 0x90); 
	write_cmos_sensor_8(0x380e, 0x0f); 
	write_cmos_sensor_8(0x380f, 0x8a); 
	write_cmos_sensor_8(0x3811, 0x10); 
	write_cmos_sensor_8(0x3813, 0x07); 
	write_cmos_sensor_8(0x3814, 0x01); 
	write_cmos_sensor_8(0x3816, 0x01); 
	write_cmos_sensor_8(0x381f, 0x08); 
	write_cmos_sensor_8(0x3820, 0xb0); 
	write_cmos_sensor_8(0x3c8c, 0x19); 
	write_cmos_sensor_8(0x3f02, 0x2a);
	write_cmos_sensor_8(0x3f03, 0x10);
	write_cmos_sensor_8(0x4008, 0x02); 
	write_cmos_sensor_8(0x4009, 0x0f); 
	write_cmos_sensor_8(0x4011, 0x21); 
	write_cmos_sensor_8(0x4032, 0x1e); 
	write_cmos_sensor_8(0x4050, 0x02); 
	write_cmos_sensor_8(0x4051, 0x09); 
	write_cmos_sensor_8(0x4066, 0x02);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4502, 0x10); 
	write_cmos_sensor_8(0x4505, 0x00); 
	write_cmos_sensor_8(0x4837, 0x0e); 
	write_cmos_sensor_8(0x5000, 0xff); 
	write_cmos_sensor_8(0x5001, 0x0f);
	write_cmos_sensor_8(0x5049, 0xa4); 
	LOG_INF("custom1_setting() end! \n");

}

static void custom2_setting(kal_uint16 currefps)
{
//	capture_setting(currefps);
	LOG_INF("custom2_setting() start! \n");

	write_cmos_sensor_8(0x0305, 0x41);
	write_cmos_sensor_8(0x0327, 0x05); 
	write_cmos_sensor_8(0x3501, 0x0c);
	write_cmos_sensor_8(0x3502, 0x00);
	write_cmos_sensor_8(0x3612, 0x4e);
	write_cmos_sensor_8(0x3621, 0x28);
	write_cmos_sensor_8(0x3622, 0xe6);
	write_cmos_sensor_8(0x3623, 0x00);
	write_cmos_sensor_8(0x3662, 0x92);
	write_cmos_sensor_8(0x3714, 0x24);
	write_cmos_sensor_8(0x371a, 0x3e);
	write_cmos_sensor_8(0x3739, 0x12);
	write_cmos_sensor_8(0x3789, 0x18);
	write_cmos_sensor_8(0x37c2, 0x04);
	write_cmos_sensor_8(0x37d9, 0x0c);
	write_cmos_sensor_8(0x37e2, 0x0a);
	write_cmos_sensor_8(0x37e4, 0x04);
	write_cmos_sensor_8(0x3800, 0x00);
	write_cmos_sensor_8(0x3801, 0x00);
	write_cmos_sensor_8(0x3802, 0x00);
	write_cmos_sensor_8(0x3803, 0x08);
	write_cmos_sensor_8(0x3804, 0x10);
	write_cmos_sensor_8(0x3805, 0x8f);
	write_cmos_sensor_8(0x3806, 0x0c);
	write_cmos_sensor_8(0x3807, 0x47);
	write_cmos_sensor_8(0x3808, 0x0c);
	write_cmos_sensor_8(0x3809, 0xc0);
	write_cmos_sensor_8(0x380a, 0x09);
	write_cmos_sensor_8(0x380b, 0x90);
	write_cmos_sensor_8(0x380e, 0x12);
	write_cmos_sensor_8(0x380f, 0xA8);
	write_cmos_sensor_8(0x3810, 0x01);
	write_cmos_sensor_8(0x3811, 0xd0);
	write_cmos_sensor_8(0x3812, 0x01);
	write_cmos_sensor_8(0x3813, 0x47);
	write_cmos_sensor_8(0x3814, 0x01);
	write_cmos_sensor_8(0x3816, 0x01);
	write_cmos_sensor_8(0x3820, 0xb0);
	write_cmos_sensor_8(0x3c8c, 0x19);
	write_cmos_sensor_8(0x3f02, 0x2a);
	write_cmos_sensor_8(0x3f03, 0x10);
	write_cmos_sensor_8(0x4008, 0x02);
	write_cmos_sensor_8(0x4009, 0x0f);
	write_cmos_sensor_8(0x4032, 0x1e);
	write_cmos_sensor_8(0x4050, 0x02);
	write_cmos_sensor_8(0x4051, 0x09);
	write_cmos_sensor_8(0x4066, 0x02);
	write_cmos_sensor_8(0x4501, 0x00);
	write_cmos_sensor_8(0x4502, 0x10);
	write_cmos_sensor_8(0x4505, 0x00);
	write_cmos_sensor_8(0x481b, 0x3e);
	write_cmos_sensor_8(0x4821, 0x3e);
	write_cmos_sensor_8(0x4837, 0x0e);
	write_cmos_sensor_8(0x484b, 0x01);
	write_cmos_sensor_8(0x5000, 0xff);
	write_cmos_sensor_8(0x5001, 0x0f);
	write_cmos_sensor_8(0x5049, 0xa4);

	LOG_INF("custom2_setting() end! \n");
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_8(0x300a) << 16) | (read_cmos_sensor_8(0x300b) << 8) | read_cmos_sensor_8(0x300c));
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
	kal_uint8 i = 0, j = 0;
	kal_uint8 retry = 2;
 	int offset = 0x10;
	unsigned char material_inf_ov13b10[4];
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	LOG_INF(" I2C_BUS = %d\n",I2C_BUS);
	if(I2C_BUS != 2){	
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
				*sensor_id = return_sensor_id();
				for(j = 0;j < 4;j++){
					vivo_read_eeprom(offset,&material_inf_ov13b10[j]);	
					offset++;
				}			
				if((int)material_inf_ov13b10[3] == 55) {
					if (*sensor_id == imgsensor_info.sensor_id) {
					pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
					if (is_atboot == 1) {
						pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
						pr_info("AT mode skip now return\n");
						return ERROR_NONE;
					}
					vivo_otp_read_when_power_on = ov13b10_vivo_otp_read();
					return ERROR_NONE;
				}
			}
			pr_err("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
				retry--;
		} while (retry > 0);
		i++;
		retry = 1;
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
	kal_uint8 retry = 1;
	kal_uint32 sensor_id = 0;

    LOG_INF("PLATFORM:MT6765,MIPI 4LANE +++++ ++++ \n");
    pr_info(PFX "[%s ]preview 2112*1568@30fps,1080Mbps/lane; video 1024*768@120fps,864Mbps/lane; capture 4224*3136@30fps,1080Mbps/lane\n",__FUNCTION__);
	
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			LOG_INF("if before i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			pr_err("Read sensor id fail: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id){
			break;
		}
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id){
		return ERROR_SENSOR_CONNECT_FAIL;
	}

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
    pr_info(PFX "[%s] exit \n",__FUNCTION__);
	
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
	pr_info(PFX "[%s] E \n",__FUNCTION__);

	/*No Need to implement this function*/
	streaming_control(KAL_FALSE);
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
 *  *image_window : address pointer of pixel numbers in one period of HSYNC
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
	LOG_INF("E\n");
	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor.current_fps;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
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
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	 capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
 
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
	LOG_INF("E\n");	 

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use custom1's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom1.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom2.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use custom2's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom2.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");	 

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom3.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use custom3's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom3.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.custom3.pclk;
		imgsensor.line_length = imgsensor_info.custom3.linelength;
		imgsensor.frame_length = imgsensor_info.custom3.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom4.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use custom4's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom4.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.custom4.pclk;
		imgsensor.line_length = imgsensor_info.custom4.linelength;
		imgsensor.frame_length = imgsensor_info.custom4.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom5.max_framerate){
			LOG_INF(
		"Warning: current_fps %d fps is not support, so use custom5's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom5.max_framerate/10);
		}
		imgsensor.pclk = imgsensor_info.custom5.pclk;
		imgsensor.line_length = imgsensor_info.custom5.linelength;
		imgsensor.frame_length = imgsensor_info.custom5.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}

static kal_uint32 get_resolution(
	MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
	LOG_INF("E\n");
 
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

	sensor_resolution->SensorSlimVideoWidth	 =
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
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				  MSDK_SENSOR_INFO_STRUCT *sensor_info,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
 
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	/* inverse with datasheet*/
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

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
	sensor_info->Custom1DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom5_delay_frame;


	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 1; /*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode*/
	/* 0: NO PDAF, 1: PDAF Raw Data mode,
	 * 2:PDAF VC mode(Full),
	 * 3:PDAF VC mode(Binning)
	 */
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

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

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

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

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
}	/* get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_info(PFX "[%s] scenario_id = %d\n", __FUNCTION__,scenario_id);
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
			custom4(image_window, sensor_config_data);
			break;
    case MSDK_SCENARIO_ID_CUSTOM5:
			custom5(image_window, sensor_config_data);
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
	pr_info("set_video_mode framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate*/
	if (framerate == 0)
		/* Dynamic frame rate*/
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)){
		imgsensor.current_fps = 296;
	}
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)){
		imgsensor.current_fps = 146;
	}
	else{
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	pr_info(" set_auto_flicker_mode enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {/*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	}
	else {/*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	pr_info(PFX "[%s] sscenario_id = %d, framerate = %d\n", __FUNCTION__,scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength){
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0){
			return ERROR_NONE;
		}

		frame_length = imgsensor_info.normal_video.pclk /
			framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.normal_video.framelength){
			imgsensor.dummy_line =
		      (frame_length - imgsensor_info.normal_video.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}
		
		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if (imgsensor.current_fps !=
				imgsensor_info.cap.max_framerate){
				LOG_INF(
					"current_fps %d is not support, so use cap's setting: %d fps!\n",
					framerate,
					imgsensor_info.cap.max_framerate/10);
			}
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock(&imgsensor_drv_lock);

			if (frame_length > imgsensor_info.cap.framelength){
				imgsensor.dummy_line = frame_length - imgsensor_info.cap.framelength;
			}
			else{
				imgsensor.dummy_line = 0;
			}

			imgsensor.frame_length =
			 imgsensor_info.cap.framelength + imgsensor.dummy_line;

			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;

		spin_lock(&imgsensor_drv_lock);
			LOG_INF("frame_length = %d, imgsensor_info.hs_video.framelength = %d\n", frame_length,imgsensor_info.hs_video.framelength);
		if (frame_length > imgsensor_info.hs_video.framelength){
			imgsensor.dummy_line =  (frame_length - imgsensor_info.hs_video.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
	LOG_INF("imgsensor.frame_length = %d, imgsensor.shutter = %d\n", imgsensor.frame_length, imgsensor.shutter);
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		//if (imgsensor.frame_length > imgsensor.shutter){ //changcheng@vivo.com  slow  frame_length and exp  register does not fit
		//	set_dummy();
		//}
		
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.slim_video.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.slim_video.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom1.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.custom1.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk /
			framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom2.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.custom2.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk /
			framerate * 10 / imgsensor_info.custom3.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom3.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.custom3.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk /
			framerate * 10 / imgsensor_info.custom4.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom4.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.custom4.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk /
			framerate * 10 / imgsensor_info.custom5.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom5.framelength){
			imgsensor.dummy_line = (frame_length - imgsensor_info.custom5.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		
		break;
	default:  /*coding with  preview scenario by default*/

		frame_length = imgsensor_info.pre.pclk /
			framerate * 10 / imgsensor_info.pre.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.pre.framelength){
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.pre.framelength);
		}
		else{
			imgsensor.dummy_line = 0;
		}

		imgsensor.frame_length =
		  imgsensor_info.pre.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter){
			set_dummy();
		}
		LOG_INF(
		    "error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
		write_cmos_sensor_8(0x5000, 0x81);
		write_cmos_sensor_8(0x5080, 0x80);
	} else {
		write_cmos_sensor_8(0x5000, 0xff);
		write_cmos_sensor_8(0x5080, 0x00);
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
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	/*struct SENSOR_VC_INFO_STRUCT *pvcinfo;*/
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;


	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
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
		*(feature_data + 2) = imgsensor_info.exp_step;
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
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len = 4;
			break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
			break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	    /*night_mode((BOOL) *feature_data); no need to implement this mode*/
			break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
			break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
	case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor_8(
			    sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
	case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData =
				read_cmos_sensor_8(sensor_reg_data->RegAddr);
			break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */

		/* if EEPROM does not exist in camera module.*/
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
			(BOOL)*feature_data_16, *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;

		/*for factory mode auto testing*/
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len = 4;
			break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
			imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
			break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			*feature_data_32);

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
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[9],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1), (UINT16)*(feature_data+2));

		/* ihdr_write_shutter_gain((UINT16)*feature_data,
		 * (UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		 */
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",(UINT16)*feature_data, (UINT16)*(feature_data+1));
		/* ihdr_write_shutter(
		 * (UINT16)*feature_data,(UINT16)*(feature_data+1));
		 */
		break;
		/******************** PDAF START >>> *********/
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: /*full*/
     		case MSDK_SCENARIO_ID_CUSTOM1:
     		case MSDK_SCENARIO_ID_CUSTOM2:
     		case MSDK_SCENARIO_ID_CUSTOM3:
     		case MSDK_SCENARIO_ID_CUSTOM4:
     		case MSDK_SCENARIO_ID_CUSTOM5:						
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
     		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
     		case MSDK_SCENARIO_ID_SLIM_VIDEO:
     		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

			default:
				break;
		}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		//PDAF capacity enable or not, 2p8 only full size support PDAF
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CUSTOM1:
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CUSTOM3:
			case MSDK_SCENARIO_ID_CUSTOM4:
			case MSDK_SCENARIO_ID_CUSTOM5:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;// xxx is not supported
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
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
		}
		break;

	case SENSOR_FEATURE_GET_PDAF_DATA:	/*get cal data from eeprom*/
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
			LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA success\n");
			
			break;
	
	case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", imgsensor.pdaf_mode);
			imgsensor.pdaf_mode = *feature_data_16;
			break;
		
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:/*lzl*/
			set_shutter_frame_length((UINT16)*feature_data,(UINT16)*(feature_data+1));
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
	
	
	
	
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
    	printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  OV13B10_OTP_ERROR_CODE:%d \n", *feature_data,OV13B10_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
				printk("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
				if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
					*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = OV13B10_OTP_ERROR_CODE;//otp_state
					memcpy( feature_data+2, sn_inf_main_ov13b10, sizeof(MUINT32)*13); 	
					memcpy( feature_data+10, material_inf_main_ov13b10, sizeof(MUINT32)*4); 
					#if 0
						for (i = 0 ; i<13 ; i++ ){
						printk("sn_inf_main_ov13b10[%d]= 0x%x\n", i, sn_inf_main_ov13b10[i]);
						}
								
					#endif
					}
				break;
				}
			break;
	/******************** STREAMING RESUME/SUSPEND *********/
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_info("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CUSTOM3:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM4:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

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
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 OV13B10_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL){
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
}
