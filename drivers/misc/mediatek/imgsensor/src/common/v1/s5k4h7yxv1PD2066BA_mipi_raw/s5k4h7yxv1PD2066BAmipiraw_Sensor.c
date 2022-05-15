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
 *	 s5k4h7yxv1PD2066BAmipiraw_Sensor.c
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


#define PFX "S5K4H7YXV1PD2066BA_camera_sensor"
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

#include "s5k4h7yxv1PD2066BAmipiraw_Sensor.h"
#include "../imgsensor_i2c.h"
/*
 * #define pr_debug(format, args...) pr_debug(
 * PFX "[%s] " format, __func__, ##args)
 */
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
      .sensor_id = S5K4H7YXV1PD2066BA_SENSOR_ID,
      .checksum_value = 0x91f55fe8,//0x82256eb5,

      .pre = {
              .pclk = 269750000,
              .linelength = 3560,
              .framelength = 2526,
              .startx = 0,
              .starty = 0,
              .grabwindow_width = 1632, /*//0x0A20*/
              .grabwindow_height = 1224, /*//0x0794*/
              //grabwindow_height should be 16's N times
              .mipi_data_lp2hs_settle_dc = 85,
              .mipi_pixel_rate = 196800000,  //205400000
              .max_framerate = 300,
              },
      .cap = {
              .pclk = 269750000,
              .linelength = 3560,
              .framelength = 2526,
              .startx = 0,
              .starty = 0,
              .grabwindow_width = 3264, /*//0x1440*/
              .grabwindow_height = 2448, /*//0x0F28*/
              /*//grabwindow_height should be 16's N times*/
              .mipi_data_lp2hs_settle_dc = 85,
              .mipi_pixel_rate = 303200000,
              .max_framerate = 300,
              },
      .normal_video = {
                       .pclk = 269750000,
                       .linelength = 3560,
                       .framelength = 2526,
                       .startx = 0,
                       .starty = 0,
                       .grabwindow_width = 3264,
                       .grabwindow_height = 1836,
                       .mipi_data_lp2hs_settle_dc = 85,
                       .mipi_pixel_rate = 303200000,
                       .max_framerate = 300,
                       },
      .hs_video = {
                   .pclk = 269750000,
                   .linelength = 3556,
                   .framelength = 632,
                   .startx = 0,
                   .starty = 0,
                   .grabwindow_width = 640,
                   .grabwindow_height = 480,
                   .mipi_data_lp2hs_settle_dc = 85,
                   .max_framerate = 1200,
                   },
      .slim_video = {
                     .pclk = 269750000,
                     .linelength = 7112,
                     .framelength = 1264,
                     .startx = 0,
                     .starty = 0,
                     .grabwindow_width = 1280, /*//0x1440*/
                     .grabwindow_height = 720, /*//0x0F28*/
                     /*//grabwindow_height should be 16's N times*/
                     .mipi_data_lp2hs_settle_dc = 85,
                     .max_framerate = 300,
                     },
	.custom1 = {
		 .pclk = 269750000,
		 .linelength = 3560,
		 .framelength = 2526,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 1632, /*//0x0A20*/
		 .grabwindow_height = 1224, /*//0x0794*/
		 //grabwindow_height should be 16's N times
		 .mipi_data_lp2hs_settle_dc = 85,
		 .mipi_pixel_rate = 196800000,	//205400000
		 .max_framerate = 300,
	},
	.custom2 = {
		 .pclk = 269750000,
		 .linelength = 3560,
		 .framelength = 2526,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 1632, /*//0x0A20*/
		 .grabwindow_height = 1224, /*//0x0794*/
		 //grabwindow_height should be 16's N times
		 .mipi_data_lp2hs_settle_dc = 85,
		 .mipi_pixel_rate = 196800000,	//205400000
		 .max_framerate = 300,
	},
	.custom3 = {
		 .pclk = 269750000,
		 .linelength = 3560,
		 .framelength = 2526,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 1632, /*//0x0A20*/
		 .grabwindow_height = 1224, /*//0x0794*/
		 //grabwindow_height should be 16's N times
		 .mipi_data_lp2hs_settle_dc = 85,
		 .mipi_pixel_rate = 196800000,	//205400000
		 .max_framerate = 300,
	},
	.custom4 = {
		 .pclk = 269750000,
		 .linelength = 3560,
		 .framelength = 2526,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 1632, /*//0x0A20*/
		 .grabwindow_height = 1224, /*//0x0794*/
		 //grabwindow_height should be 16's N times
		 .mipi_data_lp2hs_settle_dc = 85,
		 .mipi_pixel_rate = 196800000,	//205400000
		 .max_framerate = 300,
	},
	.custom5 = {
		 .pclk = 269750000,
		 .linelength = 3560,
		 .framelength = 2526,
		 .startx = 0,
		 .starty = 0,
		 .grabwindow_width = 1632, /*//0x0A20*/
		 .grabwindow_height = 1224, /*//0x0794*/
		 //grabwindow_height should be 16's N times
		 .mipi_data_lp2hs_settle_dc = 85,
		 .mipi_pixel_rate = 196800000,	//205400000
		 .max_framerate = 300,
	},
      .margin = 5,
      .min_shutter = 4,
      .max_frame_length = 0xffff,

      /* shutter delay frame for AE cycle,
       * 2 frame with ispGain_delay-shut_delay=2-0=2
       */
      .ae_shut_delay_frame = 0,

      /* sensor gain delay frame for AE cycle,
       * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
       */
      .ae_sensor_gain_delay_frame = 0,

      .ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
      .ihdr_support = 0,	/* 1, support; 0,not support */
      .ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
      .sensor_mode_num = 10,	/* support sensor mode num */

      .cap_delay_frame = 3,	/* enter capture delay frame num */
      .pre_delay_frame = 3,	/* enter preview delay frame num */
      .video_delay_frame = 3,	/* enter video delay frame num */

      /* enter high speed video  delay frame num */
      .hs_video_delay_frame = 3,

      .slim_video_delay_frame = 3,	/* enter slim video delay frame num */
	  
	  .custom1_delay_frame = 2,
	  .custom2_delay_frame = 2,
	  .custom3_delay_frame = 2,
	  .custom4_delay_frame = 2,
	  .custom4_delay_frame = 2,

      .isp_driving_current = ISP_DRIVING_2MA,	/* mclk driving current */

      /* sensor_interface_type */
      .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

      /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
      .mipi_sensor_type = MIPI_OPHY_NCSI2,

      /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
      .mipi_settle_delay_mode = 0,
      .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
      .mclk = 26,	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
      .mipi_lane_num = SENSOR_MIPI_4_LANE,

      /* record sensor support all write id addr,
       * only supprt 4 must end with 0xff
       */
      .i2c_addr_table = { 0x20, 0xff},
      .i2c_speed = 400,
};


static struct imgsensor_struct imgsensor = {
        .mirror = IMAGE_HV_MIRROR,	//TODO			//mirrorflip information

        /* IMGSENSOR_MODE enum value,record current sensor mode,such as:
         * INIT, Preview, Capture, Video,High Speed Video, Slim Video
         */
        .sensor_mode = IMGSENSOR_MODE_INIT,

        .shutter = 0x3D0,	/* current shutter */
        .gain = 0x100,		/* current gain */
        .dummy_pixel = 0,	/* current dummypixel */
        .dummy_line = 0,	/* current dummyline */
        .current_fps = 0,	/* full size current fps : 24fps for PIP,
                           * 30fps for Normal or ZSD
                           */

        /* auto flicker enable: KAL_FALSE for disable auto flicker,
         * KAL_TRUE for enable auto flicker
         */
        .autoflicker_en = KAL_FALSE,

        /* test pattern mode or not.
         * KAL_FALSE for in test pattern mode,
         * KAL_TRUE for normal output
         */
        .test_pattern = KAL_FALSE,

        /* current scenario id */
        .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,

        /* sensor need support LE, SE with HDR feature */
        .ihdr_mode = KAL_FALSE,
        .i2c_write_id = 0x20,	/* record current sensor's i2c write id */

};


//int chip_id;
/* VC_Num, VC_PixelNum, ModeSelect, EXPO_Ratio, ODValue, RG_STATSMODE */
/* VC0_ID, VC0_DataType, VC0_SIZEH, VC0_SIZE,
 * VC1_ID, VC1_DataType, VC1_SIZEH, VC1_SIZEV
 */
/* VC2_ID, VC2_DataType, VC2_SIZEH, VC2_SIZE,
 * VC3_ID, VC3_DataType, VC3_SIZEH, VC3_SIZEV
 */

/*static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
 *  {// Preview mode setting
 *  {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *  0x00, 0x2B, 0x0910, 0x06D0, 0x01, 0x00, 0x0000, 0x0000,
 *  0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
 * // Video mode setting
 *{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
 *0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
 * // Capture mode setting
 *{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
 *0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000}};
 */

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
      { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // Preview
      { 3264, 2448,	  0,	  0,   3264, 2448, 3264, 2448, 0000, 0000, 3264, 2448,	0,	0, 3264, 2448}, // capture
      { 3264, 2448,	  0,	  306,   3264, 1836, 3264, 1836, 0000, 0000, 3264, 1836,	0,	0, 3264, 1836}, // video
      { 3264, 2448,	  0,    0,   3264, 2448, 640,  480,  0000, 0000, 640,  480,	  0,	0, 640,  480}, //hight speed video
      { 3264, 2448,	  351,  514, 2562, 1440, 1280, 720,  0000, 0000, 1280, 720,	  0,	0, 1280, 720},
      { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // custom1
	  { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // custom2
	  { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // custom3
	  { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // custom4
	  { 3264, 2448,	  0,	  0,   3264, 2448, 1632, 1224, 0000, 0000, 1632, 1224,	0,	0, 1632, 1224}, // custom5
};// slim video


/* no mirror flip, and no binning -revised by dj */
/* static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
 * .i4OffsetX = 16,
 * .i4OffsetY = 16,
 * .i4PitchX  = 64,
 * .i4PitchY  = 64,
 * .i4PairNum  =16,
 * .i4SubBlkW  =16,
 * .i4SubBlkH  =16,
 * .i4PosL = {{20,23},{72,23},{36,27},{56,27},{24,43},{68,43},{40,47},
 * {52,47},{40,55},{52,55},{24,59},{68,59},{36,75},{56,75},{20,79},{72,79}},
 * .i4PosR = {{20,27},{72,27},{36,31},{56,31},{24,39},{68,39},{40,43},{52,43},
 * {40,59},{52,59},{24,63},{68,63},{36,71},{56,71},{20,75},{72,75}},
 * .iMirrorFlip = 0,
 * .i4BlockNumX = 72,
 * .i4BlockNumY = 54,
 * };
 */

#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
  kal_uint16 get_byte = 0;

  iReadRegI2C((u16) addr, (u8 *) &get_byte, imgsensor.i2c_write_id);
  return get_byte;
}

#define write_cmos_sensor(addr, para) iWriteRegI2C(                     \
                                                   (u16) addr, (u32) para, 1,  imgsensor.i2c_write_id)
#endif
//vivo hope add for Camera otp errorcode
extern int S5K4H7YXV1PD2066BA_otp_read(void);
int vivo_sub_otp_read_when_power_on_s5k4h7yxv1PD2066BA;
extern otp_error_code_t S5K4H7YXV1PD2066BA_OTP_ERROR_CODE;
extern unsigned int is_atboot;
MUINT32  sn_inf_sub_s5k4h7yxv1PD2066BA[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_sub_s5k4h7yxv1PD2066BA[4];  /*0 flag   1-12 data*/
extern MUINT32  ctidFlag;
//vivo hope add end
#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
  kal_uint16 get_byte = 0;
  char pusendcmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

  iReadRegI2C(pusendcmd, 2, (u8 *) &get_byte, 2, imgsensor.i2c_write_id);
  return ((get_byte << 8) & 0xff00) | ((get_byte >> 8) & 0x00ff);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
  char pusendcmd[4] = {
                       (char)(addr >> 8), (char)(addr & 0xFF),
                       (char)(para >> 8), (char)(para & 0xFF) };
  iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}
#endif


static void write_cmos_sensor_16_8(kal_uint32 addr, kal_uint32 para)
{
  char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
  //	kdSetI2CSpeed(400); 
  iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
  kal_uint16 get_byte = 0;
  char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

  iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
  return get_byte;
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
  pr_debug("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

  write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
  write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
  write_cmos_sensor_16_8(0x0342, imgsensor.line_length >> 8);
  write_cmos_sensor_16_8(0x0343, imgsensor.line_length & 0xFF);
}


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
  kal_uint32 frame_length = imgsensor.frame_length;
  //unsigned long flags;

  pr_debug("framerate = %d, min_framelength_en=%d\n", framerate,min_framelength_en);
  frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
  pr_debug("frame_length =%d\n", frame_length);
  spin_lock(&imgsensor_drv_lock);
  imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
  imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
  //dummy_line = frame_length - imgsensor.min_frame_length;
  //if (dummy_line < 0)
  //imgsensor.dummy_line = 0;
  //else
  //imgsensor.dummy_line = dummy_line;
  //imgsensor.frame_length = frame_length + imgsensor.dummy_line;
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


static void write_shutter(kal_uint32 shutter)
{
  kal_uint16 realtime_fps = 0;
  pr_info(PFX "[%s] write shutter :%d\n",__FUNCTION__,shutter);
  // shutter=2512;//add for debug capture framerate
  /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
  /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

  // OV Recommend Solution
  // if shutter bigger than frame_length, should extend frame length first

  if(imgsensor.sensor_mode == IMGSENSOR_MODE_HIGH_SPEED_VIDEO)
    {
      if(shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        shutter = imgsensor.min_frame_length - imgsensor_info.margin;
      write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
      write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
      write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
      write_cmos_sensor_16_8(0x0203, shutter  & 0xFF);
      pr_debug("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
      return;
    }
  spin_lock(&imgsensor_drv_lock);
  if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
    {
      imgsensor.frame_length = shutter + imgsensor_info.margin;
    }
  else
    {
      imgsensor.frame_length = imgsensor.min_frame_length;
    }
  if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
      imgsensor.frame_length = imgsensor_info.max_frame_length;
    }
  spin_unlock(&imgsensor_drv_lock);
  if (shutter < imgsensor_info.min_shutter)
    shutter = imgsensor_info.min_shutter;

  if (imgsensor.autoflicker_en == KAL_TRUE) {
    realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
    if(realtime_fps >= 297 && realtime_fps <= 305)
      {
        set_max_framerate(296,0);
        //set_dummy();
      }
    else if(realtime_fps >= 147 && realtime_fps <= 150)
      {
        set_max_framerate(146,0);
        //set_dummy();
      }
    else{
      write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
      write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
    }
  } else {
    // Extend frame length
    write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
    write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
  }

  // Update Shutter
  write_cmos_sensor_16_8(0x0340, imgsensor.frame_length >> 8);
  write_cmos_sensor_16_8(0x0341, imgsensor.frame_length & 0xFF);
  write_cmos_sensor_16_8(0x0202, (shutter >> 8) & 0xFF);
  write_cmos_sensor_16_8(0x0203, shutter  & 0xFF);
  pr_debug("realtime_fps =%d\n", realtime_fps);
  pr_debug("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

  //pr_debug("frame_length = %d ", frame_length);

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
}				/*      set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
  kal_uint16 reg_gain = 0x0;

  reg_gain = gain / 2;
  return (kal_uint16) reg_gain;
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

  if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
    pr_debug("Error gain setting");

    if (gain < BASEGAIN)
      gain = BASEGAIN;
    else if (gain > 32 * BASEGAIN)
      gain = 32 * BASEGAIN;
  }

  reg_gain = gain2reg(gain);
  spin_lock(&imgsensor_drv_lock);
  imgsensor.gain = reg_gain;
  spin_unlock(&imgsensor_drv_lock);

  pr_info(PFX "[%s]:gain = %d , reg_gain = 0x%x\n",__FUNCTION__,gain, reg_gain);
  //write_cmos_sensor(0x0204,reg_gain);
  write_cmos_sensor_16_8(0x0204,(reg_gain>>8));
  write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));

  return gain;
}				/*      set_gain  */

#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
  pr_debug("image_mirror = %d\n", image_mirror);

  /********************************************************
   *
   *   0x3820[2] ISP Vertical flip
   *   0x3820[1] Sensor Vertical flip
   *
   *   0x3821[2] ISP Horizontal mirror
   *   0x3821[1] Sensor Horizontal mirror
   *
   *   ISP and Sensor flip or mirror register bit should be the same!!
   *
   ********************************************************/

  switch (image_mirror)
    {
    case IMAGE_NORMAL: //B
      write_cmos_sensor_16_8(0x0101, 0x00);	//Set normal
      break;
    case IMAGE_V_MIRROR: //Gr X
      write_cmos_sensor_16_8(0x0101, 0x01);	//Set flip
      break;
    case IMAGE_H_MIRROR: //Gb
      write_cmos_sensor_16_8(0x0101, 0x02);	//Set mirror
      break;
    case IMAGE_HV_MIRROR: //R
      write_cmos_sensor_16_8(0x0101, 0x03);	//Set mirror and flip
      break;
    }

}
#endif
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
}				/*      night_mode      */
#endif



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
static kal_uint32 streaming_control(kal_bool enable)
{
  int timeout = 100;
  int i = 0;
  int framecnt = 0;

  pr_info(PFX "[%s] streaming_enable(0=Sw Standby,1=streaming): %d\n", __FUNCTION__,enable);
  if (enable) {
    //write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor_16_8(0x0100, 0X01);
    mDELAY(33);
  } else {
    //write_cmos_sensor(0x6028,0x4000);
    write_cmos_sensor_16_8(0x0100, 0x00);
    for (i = 0; i < timeout; i++) {
      mDELAY(10);
      framecnt = read_cmos_sensor_16_8(0x0005);
      if ( framecnt == 0xFF) {
        pr_debug(" Stream Off OK at i=%d.\n", i);
        return ERROR_NONE;
      }
    }
    pr_debug("Stream Off Fail! framecnt=%d.\n", framecnt);
  }
  return ERROR_NONE;
}


kal_uint16 addr_data_pair_init_s5k4h7yxv1PD2066BA[] = {
0x0100, 0x00,
0x0B05, 0x01,
0x3074, 0x06,
0x3075, 0x2F,
0x308A, 0x20,
0x308B, 0x08,
0x308C, 0x0B,
0x3081, 0x07,
0x307B, 0x85,
0x307A, 0x0A,
0x3079, 0x0A,
0x306E, 0x71,
0x306F, 0x28,
0x301F, 0x20,
0x306B, 0x9A,
0x3091, 0x1F,
0x30C4, 0x06,
0x3200, 0x09,
0x306A, 0x79,
0x30B0, 0xFF,
0x306D, 0x08,
0x3080, 0x00,
0x3929, 0x3F,
0x3084, 0x16,
0x3070, 0x0F,
0x3B45, 0x01,
0x30C2, 0x05,
0x3069, 0x87,
0x3924, 0x7F,
0x3925, 0xFD,
0x3C08, 0xFF,
0x3C09, 0xFF,
0x3C31, 0xFF,
0x3C32, 0xFF,
0x300A, 0x52,
0x3012, 0x52,
0x3013, 0x36,
0x3019, 0x5F,
0x301A, 0x57,
0x3024, 0x10,
0x3025, 0x4E,
0x3026, 0x9A,
0x302D, 0x0B,
0x302E, 0x09,
0x392F, 0x01, /* 20180907 set tf/tr */
0x3930, 0x80, /* 20180907 set tf/tr */
0x3C03, 0x3F, /* 20181008 Dual vsync */
0x0807, 0x09, //Carl 20210129
};

kal_uint16 addr_data_pair_preview_s5k4h7yxv1PD2066BA[] = {
0x0136, 0x1A,
0x0137, 0x00,
0x0305, 0x04,
0x0306, 0x00,
0x0307, 0x53,
0x030D, 0x06,  //0x06
0x030E, 0x00,
0x030F, 0xE3,  //0xA6
0x3C1F, 0x00,
0x3C17, 0x01,
0x3C1C, 0x05,
0x3C1D, 0x15,
0x0301, 0x04,
0x0820, 0x01,  //0x02
0x0821, 0xEC,  //0xD0
0x0822, 0x00,
0x0823, 0x00,
0x0112, 0x0A,
0x0113, 0x0A,
0x0114, 0x03,
0x3906, 0x00,
0x0344, 0x00,
0x0345, 0x08,
0x0346, 0x00,
0x0347, 0x08,
0x0348, 0x0C,
0x0349, 0xC7,
0x034A, 0x09,
0x034B, 0x97,
0x034C, 0x06,
0x034D, 0x60,
0x034E, 0x04,
0x034F, 0xC8,
0x0900, 0x01,
0x0901, 0x22,
0x0381, 0x01,
0x0383, 0x01,
0x0385, 0x01,
0x0387, 0x03,
0x0101, 0x03,
0x0340, 0x09,
0x0341, 0xDE,
0x0342, 0x0D,
0x0343, 0xE8,
0x0200, 0x0D,
0x0201, 0x58,
0x0202, 0x00,
0x0203, 0x02,
0x3400, 0x01,
};


kal_uint16 addr_data_pair_capture_s5k4h7yxv1PD2066BA[] = {
0x0136, 0x1A,
0x0137, 0x00,
0x0305, 0x04,
0x0306, 0x00,
0x0307, 0x54,
0x030D, 0x06,
0x030E, 0x00,
0x030F, 0xAF,
0x3C1F, 0x00,
0x3C17, 0x00,
0x3C1C, 0x05,
0x3C1D, 0x15,
0x0301, 0x04,
0x0820, 0x02,
0x0821, 0xF6,
0x0822, 0x00,
0x0823, 0x00,
0x0112, 0x0A,
0x0113, 0x0A,
0x0114, 0x03,
0x3906, 0x00,
0x0344, 0x00,
0x0345, 0x08,
0x0346, 0x00,
0x0347, 0x08,
0x0348, 0x0C,
0x0349, 0xC7,
0x034A, 0x09,
0x034B, 0x97,
0x034C, 0x0C,
0x034D, 0xC0,
0x034E, 0x09,
0x034F, 0x90,
0x0900, 0x00,
0x0901, 0x00,
0x0381, 0x01,
0x0383, 0x01,
0x0385, 0x01,
0x0387, 0x01,
0x0101, 0x03,
0x0340, 0x09,
0x0341, 0xDE,
0x0342, 0x0D,
0x0343, 0xE8,
0x0200, 0x0D,
0x0201, 0x58,
0x0202, 0x00,
0x0203, 0x02,
0x3400, 0x01,
};


kal_uint16 addr_data_pair_normal_video_s5k4h7yxv1PD2066BA[] = {
0x0136, 0x1A,
0x0137, 0x00,
0x0305, 0x04,
0x0306, 0x00,
0x0307, 0x54,
0x030D, 0x06,
0x030E, 0x00,
0x030F, 0xAF,
0x3C1F, 0x00,
0x3C17, 0x00,
0x3C1C, 0x05,
0x3C1D, 0x15,
0x0301, 0x04,
0x0820, 0x02,
0x0821, 0xF6,
0x0822, 0x00,
0x0823, 0x00,
0x0112, 0x0A,
0x0113, 0x0A,
0x0114, 0x03,
0x3906, 0x00,
0x0344, 0x00,
0x0345, 0x08,
0x0346, 0x01,
0x0347, 0x3A,
0x0348, 0x0C,
0x0349, 0xC7,
0x034A, 0x08,
0x034B, 0x65,
0x034C, 0x0C,
0x034D, 0xC0,
0x034E, 0x07,
0x034F, 0x2C,
0x0900, 0x00,
0x0901, 0x00,
0x0381, 0x01,
0x0383, 0x01,
0x0385, 0x01,
0x0387, 0x01,
0x0101, 0x03,
0x0340, 0x09,
0x0341, 0xDE,
0x0342, 0x0D,
0x0343, 0xE8,
0x0200, 0x0D,
0x0201, 0x58,
0x0202, 0x00,
0x0203, 0x02,
0x3400, 0x01,
};


kal_uint16 addr_data_pair_hs_video_s5k4h7yxv1PD2066BA[] = {
0x0136, 0x1A,
0x0137, 0x00,
0x0305, 0x04,
0x0306, 0x00,
0x0307, 0x53,
0x030D, 0x06,
0x030E, 0x00,
0x030F, 0xA2,
0x3C1F, 0x00,
0x3C17, 0x00,
0x3C1C, 0x05,
0x3C1D, 0x15,
0x0301, 0x04,
0x0820, 0x02,
0x0821, 0xBE,
0x0822, 0x00,
0x0823, 0x00,
0x0112, 0x0A,
0x0113, 0x0A,
0x0114, 0x03,
0x3906, 0x00,
0x0344, 0x01,
0x0345, 0x68,
0x0346, 0x01,
0x0347, 0x10,
0x0348, 0x0B,
0x0349, 0x67,
0x034A, 0x08,
0x034B, 0x8F,
0x034C, 0x02,
0x034D, 0x80,
0x034E, 0x01,
0x034F, 0xE0,
0x0900, 0x01,
0x0901, 0x44,
0x0381, 0x01,
0x0383, 0x01,
0x0385, 0x01,
0x0387, 0x07,
0x0101, 0x00,
0x0340, 0x02,
0x0341, 0x78,
0x0342, 0x0D,
0x0343, 0xE4,
0x0200, 0x0D,
0x0201, 0x54,
0x0202, 0x02,
0x0203, 0x08,
0x3400, 0x01,
};


kal_uint16 addr_data_pair_slim_video_s5k4h7yxv1PD2066BA[] = {
0x0136, 0x1A,
0x0137, 0x00,
0x0305, 0x04,
0x0306, 0x00,
0x0307, 0x53,
0x030D, 0x06,
0x030E, 0x00,
0x030F, 0xA2,
0x3C1F, 0x00,
0x3C17, 0x00,
0x3C1C, 0x05,
0x3C1D, 0x15,
0x0301, 0x04,
0x0820, 0x02,
0x0821, 0xBE,
0x0822, 0x00,
0x0823, 0x00,
0x0112, 0x0A,
0x0113, 0x0A,
0x0114, 0x03,
0x3906, 0x00,
0x0344, 0x01,
0x0345, 0x68,
0x0346, 0x02,
0x0347, 0x00,
0x0348, 0x0B,
0x0349, 0x67,
0x034A, 0x07,
0x034B, 0x9F,
0x034C, 0x05,
0x034D, 0x00,
0x034E, 0x02,
0x034F, 0xD0,
0x0900, 0x01,
0x0901, 0x22,
0x0381, 0x01,
0x0383, 0x01,
0x0385, 0x01,
0x0387, 0x03,
0x0101, 0x03,
0x0340, 0x04,
0x0341, 0xF0,
0x0342, 0x1B,
0x0343, 0xC8,
0x0200, 0x1B,
0x0201, 0x38,
0x0202, 0x00,
0x0203, 0x02,
0x3400, 0x01,
};

static void sensor_init(void)
{	
  /*kal_uint16 chip_id = 0;
    chip_id = read_cmos_sensor(0x0002);	
    pr_debug("-- sensor_init, chip id = 0x%x\n", chip_id);*/
  table_write_cmos_sensor(addr_data_pair_init_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_init_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));
}				/*      sensor_init  */

static void preview_setting(void)
{	
  pr_debug("preview_setting() E\n");
  table_write_cmos_sensor(addr_data_pair_preview_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_preview_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));
}				/*      preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
  pr_debug("capture_setting() E! currefps:%d\n", currefps);

  table_write_cmos_sensor(addr_data_pair_capture_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_capture_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));
}

static void normal_video_setting(kal_uint16 currefps)
{

  pr_debug("E! currefps:%d\n",currefps);	
  table_write_cmos_sensor(addr_data_pair_normal_video_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_normal_video_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
  pr_debug("hs_video_setting() E\n");
  table_write_cmos_sensor(addr_data_pair_hs_video_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_hs_video_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
  pr_debug("slim_video_setting() E\n");
  table_write_cmos_sensor(addr_data_pair_slim_video_s5k4h7yxv1PD2066BA,
                          sizeof(addr_data_pair_slim_video_s5k4h7yxv1PD2066BA) / sizeof(kal_uint16));

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
  //	kal_uint16 sp8spFlag = 0;

  /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
   *we should detect the module used i2c address
   */
#if 0
   	 int I2C_BUS = -1 ;
	I2C_BUS = i2c_adapter_id(pgi2c_cfg_legacy->pinst->pi2c_client->adapter);
	pr_debug(" I2C_BUS = %d\n",I2C_BUS);
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
      *sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001)) + 1;
      pr_debug("read_0x0000=0x%x, 0x0001=0x%x,0x0000_0001=0x%x\n",read_cmos_sensor_16_8(0x0000),read_cmos_sensor_16_8(0x0001),read_cmos_sensor_16_8(0x0000));
      pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
      //*sensor_id = imgsensor_info.sensor_id;
      if (*sensor_id == imgsensor_info.sensor_id) {
          if(1 == ctidFlag){
      
        /*vivo hope add for AT cammand start*/
        if (is_atboot == 1) {
          pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
          pr_info("AT mode skip now return\n");
          return ERROR_NONE;
        }
        /*vivo hope add for AT cammand end*/
  
        //vivo hope add for CameraEM otp errorcode
        pr_debug("add:start read eeprom  - when_power_on  = %d\n",vivo_sub_otp_read_when_power_on_s5k4h7yxv1PD2066BA);
       sensor_init();
        vivo_sub_otp_read_when_power_on_s5k4h7yxv1PD2066BA=S5K4H7YXV1PD2066BA_otp_read();
        pr_info("add:end read eeprom - when_power_on = %d\n",vivo_sub_otp_read_when_power_on_s5k4h7yxv1PD2066BA);
        //vivo hope add end
        pr_debug("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
        return ERROR_NONE;
				} else {
        /* 4Cell version check, 2T7 And 2T7's checking is differet
         *	sp8spFlag = (((read_cmos_sensor(0x000C) & 0xFF) << 8)
         *		|((read_cmos_sensor(0x000E) >> 8) & 0xFF));
         *	pr_debug(
         *	"sp8Flag(0x%x),0x5003 used by s5k4h7yxv1PD2066BA\n", sp8spFlag);
         *
         *	if (sp8spFlag == 0x5003) {
         *		pr_debug("it is s5k4h7yxv1PD2066BA\n");
         *		return ERROR_NONE;
         *	}
         *
         *		pr_debug(
         *	"2t7 type is 0x(%x),0x000C(0x%x),0x000E(0x%x)\n",
         *		sp8spFlag,
         *		read_cmos_sensor(0x000C),
         *		read_cmos_sensor(0x000E));
         *
         *		*sensor_id = 0xFFFFFFFF;
         *	return ERROR_SENSOR_CONNECT_FAIL;
         */
     		*sensor_id = 0xFFFFFFFF;
				}
			}
      pr_err("Read sensor id fail, id: 0x%x\n",
               imgsensor.i2c_write_id);
      retry--;
    } while (retry > 0);
    i++;
    retry = 2;
  }
  if (*sensor_id != imgsensor_info.sensor_id) {
    /* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
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

  pr_debug("%s", __func__);
  pr_info(PFX "[%s ] E \n",__FUNCTION__);
  /* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
   * we should detect the module used i2c address
   */
  while (imgsensor_info.i2c_addr_table[i] != 0xff) {
    spin_lock(&imgsensor_drv_lock);
    imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
    spin_unlock(&imgsensor_drv_lock);
    do {
      sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001))+1;
      if (sensor_id == imgsensor_info.sensor_id) {
        pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
        break;
      }
      pr_err("Read sensor id fail, id: 0x%x,0x%x\n", imgsensor.i2c_write_id,sensor_id);
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
  pr_info(PFX "[%s] exit \n",__FUNCTION__);
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
  pr_info(PFX "[%s] E \n",__FUNCTION__);

  /*No Need to implement this function */
  streaming_control(KAL_FALSE);
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
  pr_debug("preview E\n");

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
  pr_debug("capture E\n");
  spin_lock(&imgsensor_drv_lock);
  imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

  imgsensor.pclk = imgsensor_info.cap.pclk;
  imgsensor.line_length = imgsensor_info.cap.linelength;
  imgsensor.frame_length = imgsensor_info.cap.framelength;
  imgsensor.min_frame_length = imgsensor_info.cap.framelength;
  imgsensor.autoflicker_en = KAL_FALSE;

  spin_unlock(&imgsensor_drv_lock);

  capture_setting(imgsensor.current_fps);
  set_mirror_flip(imgsensor.mirror);

  return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
                               MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  pr_debug("normal_video E\n");

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
  pr_debug("hs_video E\n");

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
  pr_debug("slim_video E\n");

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

  return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom1.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom1's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom1.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom2.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom2's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom2.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom2.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom3.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom3's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom3.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom3.pclk;
		imgsensor.line_length = imgsensor_info.custom3.linelength;
		imgsensor.frame_length = imgsensor_info.custom3.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom4.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom4's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom4.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom4.pclk;
		imgsensor.line_length = imgsensor_info.custom4.linelength;
		imgsensor.frame_length = imgsensor_info.custom4.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M*/
		if (imgsensor.current_fps != imgsensor_info.custom5.max_framerate)
			pr_debug(
		"Warning: current_fps %d fps is not support, so use custom5's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.custom5.max_framerate/10);

		imgsensor.pclk = imgsensor_info.custom5.pclk;
		imgsensor.line_length = imgsensor_info.custom5.linelength;
		imgsensor.frame_length = imgsensor_info.custom5.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}
static kal_uint32 get_resolution(
                                 MSDK_SENSOR_RESOLUTION_INFO_STRUCT(*sensor_resolution))
{
  pr_debug("get_resolution E\n");
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
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;
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
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                           MSDK_SENSOR_INFO_STRUCT *sensor_info,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
  pr_debug("get_info -> scenario_id = %d\n", scenario_id);

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
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
    imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
    imgsensor_info.ae_ispGain_delay_frame;

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
    case MSDK_SCENARIO_ID_CUSTOM5:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;

			break;
    case MSDK_SCENARIO_ID_CUSTOM4:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;

			break;
	case MSDK_SCENARIO_ID_CUSTOM3:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
	    sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
	    sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

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
		pr_debug("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //pr_debug("framerate = %d\n ", framerate); */
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
                                                enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
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
		set_dummy();
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
		set_dummy();
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
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
		set_dummy();
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
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk /
			framerate * 10 / imgsensor_info.custom1.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom1.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom1.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom1.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk /
			framerate * 10 / imgsensor_info.custom2.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom2.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom2.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom2.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk /
			framerate * 10 / imgsensor_info.custom3.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom3.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom3.framelength);
		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom3.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk /
			framerate * 10 / imgsensor_info.custom4.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom4.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom4.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom4.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
    case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk /
			framerate * 10 / imgsensor_info.custom5.linelength;

		spin_lock(&imgsensor_drv_lock);
		if (frame_length > imgsensor_info.custom5.framelength)
			imgsensor.dummy_line =
			(frame_length - imgsensor_info.custom5.framelength);

		else
			imgsensor.dummy_line = 0;

		imgsensor.frame_length =
		   imgsensor_info.custom5.framelength + imgsensor.dummy_line;

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
		set_dummy();
		pr_debug("error scenario_id = %d, we use preview scenario\n",
             scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
                                                    enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	pr_debug("scenario_id = %d\n", scenario_id);

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
#if 0
	if (enable) {
    /* 0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9 */
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
#endif
	return ERROR_NONE;
}
static kal_uint32 get_sensor_temperature(void)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor_16_8(0x013a);

	if (temperature >= 0x0 && temperature <= 0x78)
		temperature_convert = temperature;
	else
		temperature_convert = -1;

	/*pr_info("temp_c(%d), read_reg(%d), enable %d\n",
	 *	temperature_convert, temperature, read_cmos_sensor_16_8(0x0138));
	 */

	return temperature_convert;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                                  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;


	/* SET_PD_BLOCK_INFO_T *PDAFinfo; */
	/* SENSOR_VC_INFO_STRUCT *pvcinfo; */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

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
#if 0
		pr_debug(
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
		write_cmos_sensor_16_8(sensor_reg_data->RegAddr,
                           sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
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
		pr_debug("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		pr_debug("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
             (UINT32) *feature_data);

		wininfo =
      (struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
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
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[5],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		 case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[6],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		  case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[7],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		  case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[8],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		   case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[9],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
             (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
             (UINT16) *feature_data,
             (UINT16) *(feature_data + 1),
             (UINT16) *(feature_data + 2));

    /* ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),
     * (UINT16)*(feature_data+2));
     */
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
             (UINT16) *feature_data,
             (UINT16) *(feature_data + 1));
    /* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
		break;

    /*	case SENSOR_FEATURE_GET_PDAF_DATA:	//get cal data from eeprom
     *	pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
     *	s5k2t7_read_otp_pdaf_data((kal_uint16)(*feature_data),
     *				(BYTE *)(uintptr_t)(*(feature_data+1)),
     *					(kal_uint32)(*(feature_data+2)));
     *		pr_debug("SENSOR_FEATURE_GET_PDAF_DATA success\n");
     *		break;
     */

		/******************** PDAF START >>> *********/
		/*
		 * case SENSOR_FEATURE_GET_PDAF_INFO:
		 * pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
		 * (UINT16)*feature_data);
		 * PDAFinfo =
		 * (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		 * switch (*feature_data) {
		 * case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG: //full
		 * case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 * case MSDK_SCENARIO_ID_CAMERA_PREVIEW: //2x2 binning
		 * memcpy((void *)PDAFinfo,
		 * (void *)&imgsensor_pd_info,
		 * sizeof(SET_PD_BLOCK_INFO_T)); //need to check
		 * break;
		 * case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		 * case MSDK_SCENARIO_ID_SLIM_VIDEO:
		 * default:
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_VC_INFO:
		 * pr_debug("SENSOR_FEATURE_GET_VC_INFO %d\n",
		 * (UINT16)*feature_data);
		 * pvcinfo =
		 * (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		 * switch (*feature_data_32) {
		 * case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		 * default:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		 * pr_debug(
		 * "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
		 * (UINT16)*feature_data);
		 * //PDAF capacity enable or not
		 * switch (*feature_data) {
		 * case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		 * (MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * break;
		 * case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * // video & capture use same setting
		 * break;
		 * case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * case MSDK_SCENARIO_ID_SLIM_VIDEO:
		 * //need to check
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * break;
		 * default:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_PDAF_DATA: //get cal data from eeprom
		 * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA\n");
		 * read_2T7_eeprom((kal_uint16 )(*feature_data),
		 * (char*)(uintptr_t)(*(feature_data+1)),
		 * (kal_uint32)(*(feature_data+2)));
		 * pr_debug("SENSOR_FEATURE_GET_PDAF_DATA success\n");
		 * break;
		 * case SENSOR_FEATURE_SET_PDAF:
		 * pr_debug("PDAF mode :%d\n", *feature_data_16);
		 * imgsensor.pdaf_mode= *feature_data_16;
		 * break;
		 */
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
		pr_debug("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld\n", *feature_data);
		switch (*feature_data) {
		case 0:    //info type: otp state
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = S5K4H7YXV1PD2066BA_OTP_ERROR_CODE;//otp_state
			memcpy(feature_data+2, sn_inf_sub_s5k4h7yxv1PD2066BA, sizeof(MUINT32)*13);
			memcpy( feature_data+10, material_inf_sub_s5k4h7yxv1PD2066BA, sizeof(MUINT32)*4); 
			break;
		}
		break;
		/******************** PDAF END   <<< *********/
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
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
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom4.mipi_pixel_rate;
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

UINT32 S5K4H7YXV1PD2066BA_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
