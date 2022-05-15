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
 *     GC02M1BPD2066A_mipi_mono_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef __GC02M1BPD2066AMIPI_SENSOR_H__
#define __GC02M1BPD2066AMIPI_SENSOR_H__
//#define IMAGE_NORMAL_MIRROR
//#define IMAGE_H_MIRROR
//#define IMAGE_V_MIRROR
#define IMAGE_HV_MIRROR

//#ifdef IMAGE_NORMAL_MIRROR
//#define MIRROR 0x80
//#endif

//#ifdef IMAGE_H_MIRROR
//#define MIRROR 0x81
//#endif

//#ifdef IMAGE_V_MIRROR
//#define MIRROR 0x82
//#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 0x83
#endif

/* SENSOR PRIVATE INFO FOR GAIN SETTING */
#define GC02M0_SENSOR_GAIN_BASE             0x400
#define GC02M0_SENSOR_GAIN_MAX              (16 * GC02M0_SENSOR_GAIN_BASE)
#define GC02M0_SENSOR_GAIN_MAX_VALID_INDEX  16
#define GC02M0_SENSOR_GAIN_MAP_SIZE         16
#define GC02M0_SENSOR_DGAIN_BASE            0x400

enum{
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
    IMGSENSOR_MODE_CUSTOM1,
    IMGSENSOR_MODE_CUSTOM2,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;
	kal_uint32 linelength;
	kal_uint32 framelength;
	kal_uint8 startx;
	kal_uint8 starty;
	kal_uint16 grabwindow_width;
	kal_uint16 grabwindow_height;
	kal_uint8 mipi_data_lp2hs_settle_dc;
	kal_uint32 mipi_pixel_rate;
	kal_uint16 max_framerate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES */
struct imgsensor_struct {
	kal_uint8 mirror;
	kal_uint8 sensor_mode;
	kal_uint32 shutter;
	kal_uint16 gain;
	kal_uint32 pclk;
	kal_uint32 frame_length;
	kal_uint32 line_length;
	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel;
	kal_uint16 dummy_line;
	kal_uint16 current_fps;
	kal_bool   autoflicker_en;
	kal_bool   test_pattern;
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;
	kal_uint8  ihdr_en;
	kal_uint8 i2c_write_id;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT */
struct imgsensor_info_struct {
	kal_uint16 sensor_id;	/*record sensor id defined in Kd_imgsensor.h*/
	kal_uint32 checksum_value;	/*checksum value for Camera Auto Test*/

	/*preview scenario relative information*/
	struct imgsensor_mode_struct pre;
	struct imgsensor_mode_struct cap;
	struct imgsensor_mode_struct cap1;
	struct imgsensor_mode_struct normal_video;
	struct imgsensor_mode_struct hs_video;
	struct imgsensor_mode_struct slim_video;
    struct imgsensor_mode_struct custom1;	//custom1 scenario relative information
    struct imgsensor_mode_struct custom2;	//custom2 scenario relative information
	kal_uint8  ae_shut_delay_frame;	/*shutter delay frame for AE cycle*/

	/*sensor gain delay frame for AE cycle*/
	kal_uint8  ae_sensor_gain_delay_frame;

	kal_uint8  ae_ispGain_delay_frame; /*isp gain delay frame for AE cycle*/
	kal_uint8 frame_time_delay_frame;
	kal_uint8  ihdr_support;	/*1, support; 0,not support*/
	kal_uint8  ihdr_le_firstline;	/*1,le first ; 0, se first*/
	kal_uint8  sensor_mode_num;	/*support sensor mode num*/

	kal_uint8  cap_delay_frame;	/*enter capture delay frame num*/
	kal_uint8  pre_delay_frame;	/*enter preview delay frame num*/
	kal_uint8  video_delay_frame;	/*enter video delay frame num*/

	/*enter high speed video  delay frame num*/
	kal_uint8  hs_video_delay_frame;
    kal_uint8  custom1_delay_frame;
    kal_uint8  custom2_delay_frame;
	/*enter slim video delay frame num*/
	kal_uint8  slim_video_delay_frame;
	kal_uint8  margin;
	kal_uint32 min_shutter;
	kal_uint32 max_frame_length;
	kal_uint32 min_gain;
	kal_uint32 max_gain;
	kal_uint32 min_gain_iso;
	kal_uint32 gain_step;
	kal_uint32 gain_type;
	kal_uint8 isp_driving_current; /* mclk driving current */
	kal_uint8 sensor_interface_type; /* sensor_interface_type */
	kal_uint8 mipi_sensor_type;
	/* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2,
	 * default is NCSI2, don't modify this para
	 */

	kal_uint8 mipi_settle_delay_mode;
	/* 0, high speed signal auto detect;
	 * 1, use settle delay,unit is ns,
	 * default is auto detect, don't modify this para
	 */
	kal_uint8  sensor_output_dataformat;/*sensor output first pixel color*/
	kal_uint8  mclk; /*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	kal_uint32 i2c_speed;
	kal_uint8  mipi_lane_num;		/*mipi lane num*/

	/*record sensor support all write id addr,
	 * only supprt 4must end with 0xff
	 */
	kal_uint8  i2c_addr_table[5];
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);

#endif
