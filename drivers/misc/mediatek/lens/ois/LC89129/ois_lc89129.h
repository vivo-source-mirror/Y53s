#ifndef PHONEUPDATE_H_
#define PHONEUPDATE_H_

#include "ois_core.h"

//==============================================================================
//
//==============================================================================
#define MODULE_VENDOR   0
#define MDL_VER         4

typedef signed char          INT_8;
typedef short                INT_16;
typedef int                  INT_32;
typedef long long            INT_64;
typedef unsigned char       UINT_8;
typedef unsigned short      UINT_16;
typedef unsigned int        UINT_32;
typedef unsigned long long	UINT_64;

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct
{
	UINT_16             Index;
	const UINT_8*       UpdataCode;
	UINT_32             SizeUpdataCode;
	UINT_64             SizeUpdataCodeCksm;
	const UINT_8*       FromCode;
	UINT_32             SizeFromCode;
	UINT_64             SizeFromCodeCksm;
	UINT_32             SizeFromCodeValid;
}CODE_TBL_EXT;
typedef struct ADJ_TEMP_COMPENSATION
{
	UINT_32 rcodeX;
	UINT_32 rcodeY;
	UINT_32 rcodeZ;
	UINT_32 shag;
	UINT_32 shbg;
	UINT_32 shcg;
	UINT_32 shoutag;
	UINT_32 shoutbg;
	UINT_32 shab;
	UINT_32 shac;
	UINT_32 shaa;
	UINT_32 shbb;
	UINT_32 shbc;
	UINT_32 shba;
	UINT_32 shcb;
	UINT_32 shcc;
	UINT_32 shca;
	UINT_32 tab;
	UINT_32 tac;
	UINT_32 taa;
	UINT_32 tbb;
	UINT_32 tbc;
	UINT_32 tba;
	UINT_32 TEMPOFF;
	UINT_32 tag;
	UINT_32 tbg;
	UINT_32 shiftg;
	UINT_32 shoutag1;
	UINT_32 shoutbg1;
	UINT_8  tcx;
	UINT_8  tbx;
	UINT_8  tax;
} stAdj_Temp_Compensation;

/*** caution [little-endian] ***/
#ifdef _BIG_ENDIAN_
/*typedef union DWDVAL
{
	UINT_32 UlDwdVal ;
	UINT_16 UsDwdVal[ 2 ] ;
	struct
	{
		UINT_16 UsHigVal ;
		UINT_16 UsLowVal ;
	} StDwdVal ;
	struct
	{
		UINT_8 UcRamVa3 ;
		UINT_8 UcRamVa2 ;
		UINT_8 UcRamVa1 ;
		UINT_8 UcRamVa0 ;
	} StCdwVal ;
}UnDwdVal ;*/
/*typedef union ULLNAL
{
	UINT_64 UllnValue ;
	UINT_32 UlnValue[ 2 ] ;
	struct
	{
		UINT_32 UlHigVal ;	// [63:32]
		UINT_32 UlLowVal ;	// [31:0]
	} StUllnVal ;
} UnllnVal ;*/

#else // BIG_ENDDIAN
/*typedef union DWDVAL
{
	UINT_32 UlDwdVal ;
	UINT_16 UsDwdVal[ 2 ] ;
	struct
	{
		UINT_16 UsLowVal ;
		UINT_16 UsHigVal ;
	} StDwdVal ;
	struct
	{
		UINT_8 UcRamVa0 ;
		UINT_8 UcRamVa1 ;
		UINT_8 UcRamVa2 ;
		UINT_8 UcRamVa3 ;
	} StCdwVal ;
}UnDwdVal ;*/
typedef union ULLNVAL
{
	UINT_64 UllnValue ;
	UINT_32 UlnValue[ 2 ] ;
	struct
	{
		UINT_32 UlLowVal ;// [31:0]
		UINT_32 UlHigVal ;// [63:32]
	} StUllnVal ;
}UnllnVal ;

#endif// _BIG_ENDIAN_

#define SUCCESS                 0x00
#define FAILURE                 0x01

//==============================================================================
//zhangyichi add
#define FLASH_BLOCKS                                    14
#define USER_RESERVE                                    2
#define ERASE_BLOCKS                                    ( FLASH_BLOCKS - USER_RESERVE )
#define BURST_LENGTH_UC                                 ( 120 )
#define BURST_LENGTH_FC                                 ( 64 )
#define PMEM_INITIALIZED                                ( 0x706D656D )
#define MESOF_NUM                                       2048
#define GYROFFSET_H                                     ( 0x06D6 << 16 )
#define GSENS                                           ( 16393 << 16 )
#define GSENS_MARG                                      ( GSENS / 4 )
#define POSTURETH                                       ( GSENS - GSENS_MARG )
#define ZG_MRGN                                         ( 1310 << 16 )
#define CNT050MS                                        676
#define CNT100MS                                        1352
#define CNT200MS                                        2703
#define GyroFilterTableX_gxzoom                         0x8BB4
#define GyroFilterTableY_gyzoom                         0x8BB8
#define GYRO_GAIN_X                                     ( 4 )
#define GYRO_GAIN_Y                                     ( 5 )
#define ORIGINAL_GYRO_GAIN_X                            ( 25 )
#define ORIGINAL_GYRO_GAIN_Y                            ( 26 )
#define GYRO_GAIN_FLG                                   0x00004000
#define CALIBRATION_STATUS                              ( 0 )
#define MAT0_CKSM                                       ( 31 )
#define ONE_MSEC_COUNT                                  18
#define FLASHROM_MAP                                    0x00000080
//==============================================================================

#define USER_MAT                                        0
#define INF_MAT0                                        1
#define INF_MAT1                                        2
#define INF_MAT2                                        4
#define TRIM_MAT                                        16

//
#define LC89129_SLAVE_ADDR                              0x76
#define LC89129_REG_ON                                  0x00000001
#define LC89129_REG_OFF                                 0x00000000
#define SPI_6DSOQ_MASTER                                0x00000004
#define OIS_SERVE_ON                                    0x00000007//??
#define OIS_SERVE_OFF                                   0x00000000//??
#define OIS_ACC_ON                                      0x00000001
#define OIS_ACC_OFF                                     0x00000000
#define OIS_MODE_STILL                                  0x00000001
#define OIS_MODE_MOVIE                                  0x00000000
#define READ_STATUS_INI                                 0x01000000

#define OIS_UNLOCK_CODE_1                               0xAAAAAAAA
#define OIS_UNLOCK_CODE_2                               0x55555555
#define OIS_UNLOCK_CODE_3                               0x0000ACD5
#define OIS_FLASHROM_CLK                                0x00000010//??

//CAL result

#define OIS_CALNG_EXE_END                               0x00000002L//!< Execute End (Adjust OK)
#define OIS_CALNG_EXE_HXADJ                             0x00000006L//!< Adjust NG : X Hall NG (Gain or Offset)
#define OIS_CALNG_EXE_HYADJ                             0x0000000AL//!< Adjust NG : Y Hall NG (Gain or Offset)
#define OIS_CALNG_EXE_LXADJ                             0x00000012L//!< Adjust NG : X Loop NG (Gain)
#define OIS_CALNG_EXE_LYADJ                             0x00000022L//!< Adjust NG : Y Loop NG (Gain)
#define OIS_CALNG_EXE_GXADJ                             0x00000042L//!< Adjust NG : X Gyro NG (offset)
#define OIS_CALNG_EXE_GYADJ                             0x00000082L//!< Adjust NG : Y Gyro NG (offset)
#define OIS_CALNG_EXE_GZADJ                             0x00400002L//!< Adjust NG : Z Gyro NG (offset)
#define OIS_CALNG_EXE_AXADJ                             0x00080002L// Adjust NG : X ACCL NG (offset)
#define OIS_CALNG_EXE_AYADJ                             0x00100002L// Adjust NG : Y ACCL NG (offset)
#define OIS_CALNG_EXE_AZADJ                             0x00200002L// Adjust NG : Z ACCL NG (offset)
#define OIS_CALNG_EXE_ERR                               0x00000099L//!< Execute Error End
#define OIS_CALNG_EXE_HZADJ                             0x00100002L//!< Adjust NG : AF Hall NG (Gain or Offset)
#define OIS_CALNG_EXE_LZADJ                             0x00200002L//!< Adjust NG : AF Loop NG (Gain)
#define OIS_CALNG_EXE_LNZADJ                            0x00800000L//!< Adjust NG : AF Hall Linearity adjust


//flash
#define LC89129_REG_OIS_SPI_INIT                        0xF015
#define LC89129_REG_SERVE_CTRL                          0xF010
#define LC89129_REG_OIS_CTRL                            0xF012
#define LC89129_REG_OIS_ACC_CTRL                        0xF01C
#define LC89129_REG_OIS_STATUS_READ                     0xF100
#define LC89129_REG_OIS_MOOD                            0xF103
#define LC89129_REG_OIS_PMEM_CHECKSUM                   0xF00E
#define LC89129_REG_OIS_REMAP                           0xF001
#define LC89129_REG_OIS_FLASHROM_SETUP                  0xF007
#define LC89129_REG_OIS_FLASHROM_ACCESS_CTL             0xF00C
#define LC89129_REG_OIS_FLASHROM_ACCESS_LENS            0xF00B
#define LC89129_REG_OIS_FLASHROM_CHECKSUM               0xF00D
#define LC89129_REG_OIS_FLASHROM_STARADDR               0xF00A//Start address for Flash memory.
#define LC89129_REG_OIS_PMEM_STARADDR                   0x3000//Start Address on Program memory
#define LC89129_REG_OIS_FW_VERSION_HIGH                 0X8000
#define LC89129_REG_OIS_FW_VERSION_LOW                  0X8004
#define LC89129_REG_OIS_FIX_TARGET_X                    0x00DC
#define LC89129_REG_OIS_FIX_TARGET_Y                    0x012C
#define LC89129_REG_OIS_CMD_IO_ADDR                     0xC000
#define LC89129_REG_OIS_CMD_IO_DATA                     0xD000
#define LC89129_REG_OIS_CMD_SYSDSP_REMAP                0xD000AC
#define LC89129_REG_OIS_CMD_SYSDSP_DSPDIV               0xD00014
#define LC89129_REG_OIS_CMD_SYSDSP_SOFTRES              0xD0006C
#define LC89129_REG_OIS_CMD_SYSDSP_CVER                 0xD00100
#define LC89129_REG_OIS_CMD_ROMINFO                     0xE050D4
#define LC89129_REG_OIS_CMD_FLASHROM_129                0xE07000
#define LC89129_REG_OIS_CMD_FLASHROM_FLA_RDAT           (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x00)
#define LC89129_REG_OIS_CMD_FLASHROM_FLA_WDAT           (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x04)
#define LC89129_REG_OIS_CMD_FLASHROM_ACSCNT             (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x08)
#define LC89129_REG_OIS_CMD_FLASHROM_FLA_ADR            (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x0C)
#define LC89129_REG_OIS_CMD_FLASHROM_CMD                (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x10)
#define LC89129_REG_OIS_CMD_FLASHROM_FLAWP              (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x14)//UNLOCK CODE FLAG
#define LC89129_REG_OIS_CMD_FLASHROM_FLAINT             (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x18)
#define LC89129_REG_OIS_CMD_FLASHROM_FLAMODE            (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x1C)
#define LC89129_REG_OIS_CMD_FLASHROM_TPECPW             (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x20)
#define LC89129_REG_OIS_CMD_FLASHROM_TACC               (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x24)
#define LC89129_REG_OIS_CMD_FLASHROM_ERR_FLA            (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x98)
#define LC89129_REG_OIS_CMD_FLASHROM_RSTB_FLA           (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x4CC)//WritePermission1
#define LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE1         (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x554)
#define LC89129_REG_OIS_CMD_FLASHROM_CLK_FLAON          (LC89129_REG_OIS_CMD_FLASHROM_129 + 0x664)//WritePermission2
#define LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE2         (LC89129_REG_OIS_CMD_FLASHROM_129 + 0xAA8)
#define LC89129_REG_OIS_CMD_FLASHROM_UNLK_CODE3         (LC89129_REG_OIS_CMD_FLASHROM_129 + 0xCCC)



//cal
#define LC89129_REG_OIS_GYROCAL_RAM_GX_ADIDAT           0x0220
#define LC89129_REG_OIS_GYROCAL_RAM_GY_ADIDAT           0x0224
#define LC89129_REG_OIS_GYROCAL_RAM_GXOFFZ              0x0240
#define LC89129_REG_OIS_GYROCAL_RAM_GYOFFZ              0x0244
#define LC89129_REG_OIS_GYROCAL_ZRAM_GZ_ADIDAT          0x039C
#define LC89129_REG_OIS_GYROCAL_ZRAM_GZOFFZ             0x03A0
#define LC89129_REG_OIS_GYROCAL_AcclRAM_X               0x0448
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_ADIDAT     0x0448
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_X_AC_OFFSET     0x044C
#define LC89129_REG_OIS_GYROCAL_AcclRAM_Y               0x0474
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_ADIDAT     0x0474
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_Y_AC_OFFSET     0x0478
#define LC89129_REG_OIS_GYROCAL_AcclRAM_Z               0x04A0
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_ADIDAT     0x04A0
#define LC89129_REG_OIS_GYROCAL_ACCLRAM_Z_AC_OFFSET     0x04A4



//********************************************************************************
// temperature compensation parameters : R3W11166
//********************************************************************************
#define TEMP_RCODEX                                     0x00000000
#define TEMP_RCODEY                                     0x00000000
#define TEMP_RCODEZ                                     0x7FFFFFFF
#define TEMP_SHAG                                       0x02C80000
#define TEMP_SHBG                                       0x037A0000
#define TEMP_SHCG                                       0x00000000
#define TEMP_SHOUTAG                                    0x00000000
#define TEMP_SHOUTBG                                    0x00000000
#define TEMP_SHAB                                       0x00003A90
#define TEMP_SHAC                                       0x7FFF8AE0
#define TEMP_SHAA                                       0x00003A90
#define TEMP_SHBB                                       0x0000012B
#define TEMP_SHBC                                       0x7FFFFDAA
#define TEMP_SHBA                                       0x0000012B
#define TEMP_SHCB                                       0x00000000
#define TEMP_SHCC                                       0x00000000
#define TEMP_SHCA                                       0x00000000
#define TEMP_TAB                                        0x00000CE5
#define TEMP_TAC                                        0x7FFFE635
#define TEMP_TAA                                        0x00000CE5
#define TEMP_TBB                                        0x0000520D
#define TEMP_TBC                                        0x7FFF5BE1
#define TEMP_TBA                                        0x0000520D
#define TEMP_TEMPOFF                                    0x25000000
#define TEMP_TAG                                        0x6BD00000
#define TEMP_TBG                                        0xD1200000
#define TEMP_SHIFTG                                     0x3FFFFFFF
#define TEMP_SHOUTAG1                                   0x00000000
#define TEMP_SHOUTBG1                                   0x00000000
#define TEMP_TCX                                        0x08
#define TEMP_TBX                                        0x00
#define TEMP_TAX                                        0x00

//********************************************************************************

#define __OISFLSH__
#ifdef __OISFLSH__
const stAdj_Temp_Compensation SO2692_TempCompParameter[] =
{
	{
		/****** P1 ACT=01 ******/
		/* rcodeX */    TEMP_RCODEX,
		/* rcodeY */    TEMP_RCODEY,
		/* rcodeZ */    TEMP_RCODEZ,
		/* shag */      TEMP_SHAG,
		/* shbg */      TEMP_SHBG,
		/* shcg */      TEMP_SHCG,
		/* shoutag */   TEMP_SHOUTAG,
		/* shoutbg */   TEMP_SHOUTBG,
		/* shab */      TEMP_SHAB,
		/* shac */      TEMP_SHAC,
		/* shaa */      TEMP_SHAA,
		/* shbb */      TEMP_SHBB,
		/* shbc */      TEMP_SHBC,
		/* shba */      TEMP_SHBA,
		/* shcb */      TEMP_SHCB,
		/* shcc */      TEMP_SHCC,
		/* shca */      TEMP_SHCA,
		/* tab */       TEMP_TAB,
		/* tac */       TEMP_TAC,
		/* taa */       TEMP_TAA,
		/* tbb */       TEMP_TBB,
		/* tbc */       TEMP_TBC,
		/* tba */       TEMP_TBA,
		/* TEMPOFF */   TEMP_TEMPOFF,
		/* tag */       TEMP_TAG,
		/* tbg */       TEMP_TBG,
		/* shiftg */    TEMP_SHIFTG,
		/* shoutag1 */  TEMP_SHOUTAG1,
		/* shoutbg1 */  TEMP_SHOUTBG1,
		/* tcx */       TEMP_TCX,
		/* tbx */       TEMP_TBX,
		/* tax */       TEMP_TAX
	}
};
#endif// __OISFLSH__
//sin
const UINT_32 frequency_DEC[ 17 ]	=
{
	0xFFFFFFFF,//  0:  Stop
	0x0001D268,//  1: 1Hz
	0x0003A4d0,//  2: 2Hz
	0x00057738,//  3: 3Hz
	0x000749A0,//  4: 4Hz
	0x00091C08,//  5: 5Hz
	0x000AEE70,//  6: 6Hz
	0x000CC0D8,//  7: 7Hz
	0x000E9340,//  8: 8Hz
	0x001065A8,//  9: 9Hz
	0x00123810,//  A: 10Hz
	0x00140A78,//  B: 11Hz
	0x0015DCE0,//  C: 12Hz
	0x0017AF48,//  D: 13Hz
	0x001981b0,//  E: 14Hz
	0x001B5418,//  F: 15Hz
	0x001D2680// 10: 16Hz
} ;
#define			LC89129_REG_OIS_CIRCWAVE						255
#define			LC89129_REG_OIS_SinWaveC						0x02F0
#define			LC89129_REG_OIS_SinWaveC_Pt						0x0000 + LC89129_REG_OIS_SinWaveC
#define			LC89129_REG_OIS_SinWaveC_Regsiter				0x0004 + LC89129_REG_OIS_SinWaveC


#define			LC89129_REG_OIS_SinWave							0x02FC
// SinGenerator.h SinWave_t
#define			LC89129_REG_OIS_SinWave_Offset					0x0000 + LC89129_REG_OIS_SinWave
#define			LC89129_REG_OIS_SinWave_Phase					0x0004 + LC89129_REG_OIS_SinWave
#define			LC89129_REG_OIS_SinWave_Gain					0x0008 + LC89129_REG_OIS_SinWave
#define			LC89129_REG_OIS_SinWave_Output					0x000C + LC89129_REG_OIS_SinWave
#define			LC89129_REG_OIS_SinWave_OutAddr					0x0010 + LC89129_REG_OIS_SinWave
#define			LC89129_REG_OIS_CosWave							0x0310
// SinGenerator.h SinWave_t
#define			LC89129_REG_OIS_CosWave_Offset					0x0000 + LC89129_REG_OIS_CosWave
#define			LC89129_REG_OIS_CosWave_Phase					0x0004 + LC89129_REG_OIS_CosWave
#define			LC89129_REG_OIS_CosWave_Gain					0x0008 + LC89129_REG_OIS_CosWave
#define			LC89129_REG_OIS_CosWave_Output					0x000C + LC89129_REG_OIS_CosWave
#define			LC89129_REG_OIS_CosWave_OutAddr					0x0010 + LC89129_REG_OIS_CosWave


#undef TEMP_RCODEX
#undef TEMP_RCODEY
#undef TEMP_RCODEZ
#undef TEMP_SHAG
#undef TEMP_SHBG
#undef TEMP_SHCG
#undef TEMP_SHOUTAG
#undef TEMP_SHOUTBG
#undef TEMP_SHAB
#undef TEMP_SHAC
#undef TEMP_SHAA
#undef TEMP_SHBB
#undef TEMP_SHBC
#undef TEMP_SHBA
#undef TEMP_SHCB
#undef TEMP_SHCC
#undef TEMP_SHCA
#undef TEMP_TAB
#undef TEMP_TAC
#undef TEMP_TAA
#undef TEMP_TBB
#undef TEMP_TBC
#undef TEMP_TBA
#undef TEMP_TEMPOFF
#undef TEMP_TAG
#undef TEMP_TBG
#undef TEMP_SHIFTG
#undef TEMP_TCX
#undef TEMP_TBX
#undef TEMP_TAX
#endif /* #ifndef OIS_H_ */

