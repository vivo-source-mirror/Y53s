/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#ifndef OIS_DW9781C_H
#define OIS_DW9781C_H

#include "ois_core.h"

#define DW9781C_OIS_FW_NAME "dw9781cFW.bin"

/* common define */
#define DW7981C_SLAVE_ADDR          0xE4
#define OFF                         0x0000
#define ON                          0x0001
#define OIS_ON                      0x0000  //servo on and ois on
#define SERVO_ON                    0x0001  //servo on and ois locked
#define SERVO_OFF                   0x0002
#define REG_DEAFULT_ON              0x8000
#define PANTILT_OFF                 0x8000
#define PANTILT_ON                  0x0000
#define USER_WRITE_EN               0x56FA
#define OIS_SPI_MASTER              0x0001
#define OIS_SPI_INPUT               0x0002
#define OIS_GYRO_INITED             0x8001
#define OIS_GYRO_STOPPED            0x8002
#define ALL_RELEASE1                0x98AC
#define ALL_RELEASE2                0x70BD
#define FW_CHECKSUM_FLAG            0xCC33
#define CHIP_ID                     0x9781
#define DW9781C_ERASE_PAGE          0x0008
#define DW9781C_ERASE_START         0x0002
#define MTP_START_ADDR              0x8000
#define DW9781C_FW_SIZE             20480   //20kb
#define DW9781C_BLOCK_SIZE          512     //512bytes
#define DW9781C_FW_CHECKSUM_OFFSET  20466
#define DW9781C_SENCOND_CHIP_ID     0x0020
#define DW9871C_DEFAULT_SAMPLE_FREQ 0x04
#define DW9781C_LENS_PACKET_SIZE    62      //byte
#define DW9781C_PACKET_READ_DONE    0x0000
#define DW9781C_MAX_LENS_INFO_SIZE  310      //62*5 bytes

/* registers define*/
#define DW9781C_REG_OIS_CHIP_ID             0x7000
#define DW9781C_REG_OIS_FW_VERSION          0x7001
#define DW9781C_REG_OIS_FW_DATE             0x7002
#define DW9781C_REG_OIS_FW_CHECKSUM         0x7005  // fw checksum calculated value restored
#define DW9781C_REG_OIS_FW_EXP_CHECKSUM     0x7007  // fw checksum calculated value expext
#define DW9781C_REG_OIS_FW_TYPE             0x700D  //module factory(0x8000) or vivo(0x8001) use
#define DW9781C_REG_OPERATION_ENABLE        0x7010
#define DW9781C_REG_OPERATION_MODE          0x7011
#define DW9781C_REG_IMU_SELECT              0x7012
#define DW9781C_REG_SPI_MODE                0x7013  //gyro multiplexing use,0x0001 for spi master,0x0002 for spi input
#define DW9781C_REG_OIS_MODE                0x7014
#define DW9781C_REG_OIS_CTRL                0x7015
#define DW9781C_REG_GYRO_READ_CTRL          0x7017  //0 stop 1 start
#define DW9781C_REG_OIS_OL_TARGETX          0x7020  // target code for servo off(current DAC)
#define DW9781C_REG_OIS_OL_TARGETY          0x7021
#define DW9781C_REG_OIS_CL_TARGETX          0x7025  // target code for servo on & ois off(direct code,no need gyro&acc info)
#define DW9781C_REG_OIS_CL_TARGETY          0x7026
#define DW9781C_REG_OIS_SMOOTH_EN           0x702A
#define DW9781C_REG_OIS_TARGETX             0x7105  // target code for ois on(code calculated via gyro&acc info)
#define DW9781C_REG_OIS_TARGETY             0x7106
#define DW9781C_REG_OIS_GYROOFFSET_CAL_STS  0x7036  // gyro offset calibration operation status
#define DW9781C_REG_OIS_LOOPGAIN_DETECT     0x7042  // 8000 normal 8001 X 8002 Y 8003 X&Y
#define DW9781C_REG_OIS_HALL_RAWX           0x7046  // raw hall DAC code
#define DW9781C_REG_OIS_HALL_RAWY           0x7047
#define DW9781C_REG_OIS_LENS_POSX           0x7049  // lens position
#define DW9781C_REG_OIS_LENS_POSY           0x704A
#define DW9781C_REG_OIS_ACC_GAINX           0x70E4  // 0x1DD9 for 30cm, 0x0000 for off
#define DW9781C_REG_OIS_ACC_GAINY           0x70E5
#define DW9781C_REG_OIS_GYRO_RAWX           0x70F0
#define DW9781C_REG_OIS_GYRO_RAWY           0x70F1
#define DW9781C_REG_LENS_INFO_START         0x70B0  //lens info start addr
#define DW9781C_REG_GYRO_INFO_START         0x70CF
#define DW9781C_REG_INFO_SAMPLE_CTRL        0x70D9  //[15:8] for hall and gyro data sample enable,refer to DW9781C_SAMPLE_CTRL;[7:0] for sample frequecy
#define DW9781C_REG_INFO_SAMPLE_READY       0x70DA  //indicate if lens/gyro data is ready
#define DW9781C_REG_OIS_ACC_RAWX            0x70DE
#define DW9781C_REG_OIS_ACC_RAWY            0x70DF
#define DW9781C_REG_OIS_GYRO_OFFSETX        0x70F8  // gyro offset
#define DW9781C_REG_OIS_GYRO_OFFSETY        0x70F9
#define DW9781C_REG_OIS_GYRO_GAINX          0x70FA
#define DW9781C_REG_OIS_GYRO_GAINY          0x70FB
#define DW9781C_REG_OIS_PANTILT_CTRL        0x7029  //0x8000 off 0x0000 on
#define DW9781C_REG_OIS_PANTILT_DERGEEX     0x725C  //pantilt degree
#define DW9781C_REG_OIS_PANTILT_DERGEEY     0x725D
#define DW9781C_REG_OIS_LOOPGAINX           0x7293  //pantilt degree
#define DW9781C_REG_OIS_LOOPGAINY           0x72A3
#define DW9781C_REG_OIS_AF_DRIFT_COMP       0x7328  // AF drift compensation
#define DW9781C_REG_AF_POSITION             0x7329  // AF postion
#define DW9781C_REG_OIS_TRIPODE_CTRL        0x73D4
#define DW9781C_REG_OIS_TRIPODE_STATUS      0x73D8
#define DW9781C_REG_OIS_CMD_STATUS          0x73E5  // operation command status
#define DW9781C_REG_OIS_SMOOTH_DELAY        0x73E7  // 1 for 0.4ms, ther larger the smoother
#define DW9781C_REG_OIS_SMOOTH_STEP         0x73E8  // 1024 for 100%, the smaller the smoother
#define DW9781C_REG_OIS_SINEWAVE_AMP        0x73E9  // radius for sinewave mode(0~32767, default 16383,60um)
#define DW9781C_REG_OIS_SINEWAVE_FREQ       0x73EA  // frequency for sinewave mode(support 1/2/4/8/16, default 4)
#define DW9781C_REG_OIS_CIRCLE_STOP         0x73F0  // to stop circle wave
#define DW9781C_REG_OIS_FW_CHECKSUM_FLAG    0xA7F9  // fw update checksum flag
#define DW9781C_REG_CHIP_CTRL               0xD000
#define DW9781C_REG_OIS_DSP_CTRL            0xD001
#define DW9781C_REG_OIS_LOGIC_RESET         0xD002  // logical circuit reset
#define DW9781C_REG_OIS_SECOND_CHIP_ID      0xD060  // fixed value from ic factory
#define DW9781C_REG_FLASH_SECTOR_SELECT     0xDE03  // select the flash sector of ic
#define DW9781C_REG_FLASH_ERASE_CTRL        0xDE04  // erase start control register
#define DW9781C_REG_OIS_USER_WRITE_PROTECT  0xEBF1  // only if set to 0x56FA can user have right to write ic register
#define DW9781C_REG_OIS_ALL_RELEASE1        0xFAFA  // ic all protection release1
#define DW9781C_REG_OIS_ALL_RELEASE2        0xF053  // ic all protection release2


/*struct define*/
enum DW9781C_mode {
	DW9781C_STILL_MODE       = 0x8000,
	DW9781C_VIDEO_MODE       = 0x8001,
	DW9781C_ZOOM_MODE        = 0x8002,
	DW9781C_CENTERING_MODE   = 0x8003,
	DW9781C_SINEX_WAVE_MODE  = 0x8004,
	DW9781C_SINEY_WAVE_MODE  = 0x8005,
	DW9781C_CIRCLE_WAVE_MODE = 0x8006,
};

enum DW9781C_IMU {
	ICM20690 = 0x0000,
	LSM6DSM  = 0x0002,
	BMI260   = 0x0006,
	LSM6DSOQ = 0x0007,
};

enum DW9781C_CMD_STATUS {
	OPERATE_START = 0x8000,
	OPERATE_ING   = 0x8001,
	OPERATE_DONE  = 0x8002,
};

enum DW9781C_OPREATION_MODE {
	OP_FLASH_SAVE      = 0x00AA,
	OP_CAL_CHECKSUM    = 0x1000,
	OP_FW_CHECKSUM     = 0x2000,
	OP_OIS_MODE        = 0x8000,  //default
	OP_HALL_CAL        = 0x4012,
	OP_LOOP_GAIN_CAL   = 0x4013,
	OP_LENS_OFFSET_CAL = 0x4014,
	OP_GYRO_OFFSET_CAL = 0x4015,
};

enum DW9781C_FW_TYPE {
	FW_MODULE = 0x8000,
	FW_VIVO   = 0x8001
};

enum DW9781C_FLASH_SECTOR {
	SECTOR_0    = 0x0000,
	SECTOR_1    = 0x0008,
	SECTOR_2    = 0x0010,
	SECTOR_3    = 0x0018,
	SECTOR_4    = 0x0020,
	SECTOR_PAGE = 0x0027
};

enum DW9781C_SAMPLE_CTRL {//8bit
	DISABLE_All = 0x00,
	ENABLE_HALL = 0x01,
	ENABLE_GYRO = 0x10,
	ENABLE_ALL  = 0x11
};

struct DW9781C_LENS_INFO {
	u16 timeStamp;
	u16 hallX;
	u16 hallY;
};

#endif /* OIS_MAIN_H */

