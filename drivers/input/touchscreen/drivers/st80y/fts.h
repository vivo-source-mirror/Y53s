/*
  * fts.c
  *
  * FTS Capacitive touch screen controller (FingerTipS)
  *
  * Copyright (C) 2017, STMicroelectronics
  * Authors: AMG(Analog Mems Group)
  *
  *             marco.cali@st.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
  * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
  * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
  *THE
  * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
  */

/*!
  * \file fts.h
  * \brief Contains all the definitions and structs used generally by the driver
  */

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/device.h>
#include "../vts_core.h"
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"


/****************** CONFIGURATION SECTION ******************/
/** @defgroup conf_section	 Driver Configuration Section
  * Settings of the driver code in order to suit the HW set up and the
  *application behavior
  * @{
  */
/* **** CODE CONFIGURATION **** */
#define FTS_TS_DRV_NAME		"st80y"	/* /< driver name */
#define FTS_TS_DRV_VERSION	"5.2.19" /* /< driver version string format */
#define FTS_TS_DRV_VER		0x05021300	/* driver version u32 format */

//#define DEBUG	/* /< define to print more logs in the kernel log and better
		 
#define DRIVER_TEST	/* /< if defined allow to use and test special functions
			 * of the driver and fts_lib from comand shell (usefull
			 * for enginering/debug operations) */


/* If both COMPUTE_INIT_METHOD and PRE_SAVED_METHOD are not defined,
 * driver will be automatically configured as GOLDEN_VALUE_METHOD */
#define COMPUTE_INIT_METHOD /* Allow to compute init data on phone during
								production */
#ifndef COMPUTE_INIT_METHOD
		#define PRE_SAVED_METHOD /* Pre-Saved Method used
					  * during production */
#endif

#define DO_AUTO_TUNE_ONLY   1

#ifndef FW_UPDATE_ON_PROBE
//define LIMITS_H_FILE 
/* include the Production Limit File as header file, can be commented to use a
 * .csv file instead */
#ifdef LIMITS_H_FILE
	#define LIMITS_SIZE_NAME	myArray2_size	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * dimension of the
							 * limits data array */
	#define LIMITS_ARRAY_NAME	myArray2	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * limits data array */
#endif
#else
/* if execute fw update in the probe the limit file must be a .h */
#define LIMITS_H_FILE	/* /< include the Production Limit File as header file,
			 * DO NOT COMMENT! */
#define LIMITS_SIZE_NAME		myArray2_size	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * dimension of the
							 * limits data array */
#define LIMITS_ARRAY_NAME		myArray2	/* /< name of the
							 * variable in the
							 * limits header file
							 * which specified the
							 * limits data array */
#endif

#define USE_ONE_FILE_NODE
/* allow to enable/disable all the features just using one file node */

#ifndef FW_UPDATE_ON_PROBE
#define EXP_FN_WORK_DELAY_MS 1000	/* /< time in ms elapsed after the probe
					 * to start the work which execute FW
					 * update and the Initialization of the
					 * IC */
#endif

/* **** END **** */


/* **** FEATURES USED IN THE IC **** */
/* Enable the support of keys */
/* #define PHONE_KEY */

#define GESTURE_MODE	/* /< enable the support of the gestures */
#ifdef GESTURE_MODE
	#define USE_GESTURE_MASK	/* /< the gestures to select are
					 * referred using a gesture bitmask
					 * instead of their gesture IDs */
#endif

#define SLEEP_MODE_POWER_OFF  1 // power off in sleep mode 

#define CHARGER_MODE	/* /< enable the support to charger mode feature
			 * (comment to disable) */

#define GLOVE_MODE	/* /< enable the support to glove mode feature (comment
			 * to disable) */

#define COVER_MODE	/* /< enable the support to cover mode feature (comment
			 * to disable) */

#define STYLUS_MODE	/* /< enable the support to stylus mode feature (comment
			 * to disable) */

#define GRIP_MODE	/* /< enable the support to grip mode feature (comment
			 * to disable) */


/* **** END **** */


/* **** PANEL SPECIFICATION **** */
#define X_AXIS_MAX	1440	/* /< Max X coordinate of the display */
#define X_AXIS_MIN	0	/* /< min X coordinate of the display */
#define Y_AXIS_MAX	2959	/* /< Max Y coordinate of the display */
#define Y_AXIS_MIN	0	/* /< min Y coordinate of the display */

#define PRESSURE_MIN	0	/* /< min value of pressure reported */
#define PRESSURE_MAX	127	/* /< Max value of pressure reported */

#define DISTANCE_MIN	0	/* /< min distance between the tool and the
				 * display */
#define DISTANCE_MAX	127	/* /< Max distance between the tool and the
				 * display */

#define TOUCH_ID_MAX	10	/* /< Max number of simoultaneous touches
				 * reported */

#define AREA_MIN	PRESSURE_MIN	/* /< min value of Major/minor axis
					 * reported */
#define AREA_MAX	PRESSURE_MAX	/* /< Man value of Major/minor axis
					 * reported */
/* **** END **** */
/**@}*/
/*********************************************************/


/*
  * Configuration mode
  *
  * bitmask which can assume the value defined as features in ftsSoftware.h or
  * the following values
  */

/** @defgroup mode_section	 IC Status Mode
  * Bitmask which keeps track of the features and working mode enabled in the
  * IC.
  * The meaning of the the LSB of the bitmask must be interpreted considering
  * that the value defined in @link feat_opt Feature Selection Option @endlink
  * correspond to the position of the corresponding bit in the mask
  * @{
  */
#define MODE_NOTHING 0x00000000	/* /< nothing enabled (sense off) */
#define MODE_ACTIVE(_mask, _sett)	(_mask |= (SCAN_MODE_ACTIVE << 24) | \
						  (_sett << 16))
/* /< store the status of scan mode active and its setting */
#define MODE_LOW_POWER(_mask, _sett)   (_mask |= (SCAN_MODE_LOW_POWER << 24) | \
						  (_sett << 16))
/* /< store the status of scan mode low power and its setting */
#define IS_POWER_MODE(_mask, _mode)	((_mask&(_mode<<24)) != 0x00)
/* /< check the current mode of the IC */

/** @}*/

#define CMD_STR_LEN	32	/* /< max number of parameters that can accept
				 * the MP file node (stm_fts_cmd) */

#define TSP_BUF_SIZE	PAGE_SIZE	/* /< max number of bytes printable on
					 * the shell in the normal file nodes */

/* max number of gestures coordinates pairs reported */
#define GESTURE_MAX_COORDS_PAIRS_REPORT 100

/* number of bytes of the gesture mask */
#define GESTURE_MASK_SIZE 4

/**
  * Struct which contains information about the HW platform and set up
  */
struct fts_hw_platform_data {
	int (*power)(bool on);
	int irq_gpio;	/* /< number of the gpio associated to the interrupt pin
			 * */
	int reset_gpio;	/* /< number of the gpio associated to the reset pin */
	int power_gpio; /* 3.3V */
	int vcc_gpio;   /* 1.8V */
	const char *vdd_reg_name;	/* /< name of the VDD regulator */
	const char *avdd_reg_name;	/* /< name of the AVDD regulator */
};

/**
  * Struct which store an ordered list of the errors events encountered during
  *the polling of a FIFO.
  * The max number of error events that can be stored is equal to FIFO_DEPTH
  */
typedef struct {
	u8 list[FIFO_DEPTH * FIFO_EVENT_SIZE];	/* /< byte array which contains
						 * the series of error events
						 * encountered from the last
						 * reset of the list. */
	int count;	/* /< number of error events stored in the list */
	int last_index;	/* /< index of the list where will be stored the next
			 * error event. Subtract -1 to have the index of the
			 * last error event! */
} ErrorList;

/**
  * Struct used to specify which test perform during the Mass Production Test.
  * For each test item selected in this structure, there should be one or
  * more labels associated in the Limit file from where load the thresholds
  */
typedef struct {
	int MutualRaw;	/* /< MS Raw min/Max test */
	int MutualRawMap;	/* /< MS Raw min/Max test for each node */
	int MutualRawGap;	/* /< MS Raw Gap(max-min) test */
	int MutualRawAdj;	/* /< MS Raw Adjacent test */
	int MutualRawAdjGap;	/* /< MS Raw Adjacent Gap (max-min) test */
	int MutualRawAdjPeak;	/* /< MS Raw Adjacent Peak
				 * max(max(adjv),max(adjh)) test */
	int MutualRawLP;	/* /< MS Low Power Raw min/Max test */
	int MutualRawMapLP;	/* /< MS Low Power Raw min/Max test
				 * for each node */
	int MutualRawGapLP;	/* /< MS Low Power Raw Gap(max-min) test */
	int MutualRawAdjLP;	/* /< MS Low Power Raw Adjacent test */
	int MutualRawAdjITO;	/* /< MS Raw Adjacent test during ITO test */

	int MutualCx1;	/* /< MS Cx1 min/Max test */
	int MutualCx2;	/* /< MS Cx2 min/Max (for each node) test */
	int MutualCx2Adj;	/* /< MS Vertical and Horizontal Adj Cx2 min/Max
				 * (for each node) test */
	int MutualCxTotal;	/* /< MS Total Cx min/Max (for each node) test
				 * */
	int MutualCxTotalAdj;	/* /< MS Total vertical and Horizontal Adj Cx2
				 * min/Max (for each node) test */

	int MutualCx1LP;	/* /< MS LowPower Cx1 min/Max test */
	int MutualCx2LP;	/* /< MS LowPower Cx2 min/Max (for each node) test */
	int MutualCx2AdjLP;	/* /< MS LowPower Vertical and Horizontal Adj Cx2 min/Max
				 * (for each node) test */
	int MutualCxTotalLP;	/* /< MS Total LowPower Cx min/Max (for each node) test
				 * */
	int MutualCxTotalAdjLP;	/* /< MS Total LowPower vertical and Horizontal Adj Cx2
				 * min/Max (for each node) test */

	int MutualKeyRaw;	/* /< MS Raw Key min/Max test */
	int MutualKeyCx1;	/* /< MS Cx1 Key min/Max test */
	int MutualKeyCx2;	/* /< MS Cx2 Key min/Max (for each node) test */
	int MutualKeyCxTotal;	/* /< MS Total Cx Key min/Max (for each node)
				 * test */

	int SelfForceRaw;	/* /< SS Force Raw min/Max test */
	int SelfForceRawGap;	/* /< SS Force Raw Gap(max-min) test */
	int SelfForceRawMap;	/* /< SS Force Raw min/Max Map test */
	int SelfForceRawLP;	/* /< SS Low Power Force Raw min/Max test */
	int SelfForceRawGapLP;	/* /< SS Low Power Force Raw Gap(max-min)test */
	int SelfForceRawMapLP;	/* /< SS Low Power Force Raw min/Max Map test */

	int SelfForceIx1;	/* /< SS Force Ix1 min/Max test */
	int SelfForceIx2;	/* /< SS Force Ix2 min/Max (for each node) test
				 * */
	int SelfForceIx2Adj;	/* /< SS Vertical Adj Force Ix2 min/Max
				  * (for each node) test */
	int SelfForceIxTotal;	/* /< SS Total Force Ix min/Max (for each node)
				 * test */
	int SelfForceIxTotalAdj;	/* /< SS Total Vertical Adj Force Ix
					 * min/Max (for each node) test */
	int SelfForceCx1;	/* /< SS Force Cx1 min/Max test */
	int SelfForceCx2; /* /< SS Force Cx2 min/Max (for each node) test */
	int SelfForceCx2Adj;	/* /< SS Vertical Adj Force Cx2 min/Max (for
				 * each node) test */
	int SelfForceCxTotal;	/* /< SS Total Force Cx min/Max (for each node)
				 * test */
	int SelfForceCxTotalAdj;	/* /< SS Total Vertical Adj Force Cx
					 * min/Max (for each node) test */

	int SelfForceIx1LP;	/* /< SS LP Force Ix1 min/Max test */
	int SelfForceIx2LP;	/* /< SS LP Force Ix2 min/Max (for each node)
				 *  test */
	int SelfForceIx2AdjLP;	/* /< SS LP Vertical Adj Force Ix2 min/Max
				  * (for each node) test */
	int SelfForceIxTotalLP;	/* /< SS LP Total Force Ix min/Max
				 * (for each node) test */
	int SelfForceIxTotalAdjLP;	/* /< SS LP Total Vertical Adj Force Ix
					 * min/Max (for each node) test */
	int SelfForceCx1LP;	/* /< SS LP Force Cx1 min/Max test */
	int SelfForceCx2LP; /* /< SS LP Force Cx2 min/Max (for each node) test */
	int SelfForceCx2AdjLP;	/* /< SS LP Vertical Adj Force Cx2 min/Max (for
				 * each node) test */
	int SelfForceCxTotalLP;	/* /< SS LP Total Force Cx min/Max
				 * (for each node) test */
	int SelfForceCxTotalAdjLP;	/* /< SS LP Total Vertical Adj Force Cx
					 * min/Max (for each node) test */

	int SelfSenseRaw;	/* /< SS Sense Raw min/Max test */
	int SelfSenseRawGap;	/* /< SS Sense Raw Gap(max-min) test */
	int SelfSenseRawMap;	/* /< SS Sense Raw min/Max test for each node */
	int SelfSenseRawLP;	/* /< SS Low Power Sense Raw min/Max test */
	int SelfSenseRawGapLP; /* /< SS Low Power Sense Raw Gap(max-min) test */
	int SelfSenseRawMapLP;	/* /< SS Low Power Sense Raw min/Max test for
				 * each node */

	int SelfSenseIx1;	/* /< SS Sense Ix1 min/Max test */
	int SelfSenseIx2; /* /< SS Sense Ix2 min/Max (for each node) test */
	int SelfSenseIx2Adj;	/* /< SS Horizontal Adj Sense Ix2 min/Max
				  * (for each node) test */
	int SelfSenseIxTotal;	/* /< SS Total Horizontal Sense Ix min/Max
				  * (for each node) test */
	int SelfSenseIxTotalAdj;	/* /< SS Total Horizontal Adj Sense Ix
					 * min/Max (for each node) test */
	int SelfSenseCx1;	/* /< SS Sense Cx1 min/Max test */
	int SelfSenseCx2; /* /< SS Sense Cx2 min/Max (for each node) test */
	int SelfSenseCx2Adj;	/* /< SS Horizontal Adj Sense Cx2 min/Max
				  * (for each node) test */
	int SelfSenseCxTotal;	/* /< SS Total Sense Cx min/Max (for each node)
				 * test */
	int SelfSenseCxTotalAdj;	/* /< SS Total Horizontal Adj Sense Cx
					 * min/Max (for each node) test */

	int SelfSenseIx1LP;	/* /< SS LP Sense Ix1 min/Max test */
	int SelfSenseIx2LP; /* /< SS LP Sense Ix2 min/Max (for each node)
			     * test */
	int SelfSenseIx2AdjLP;	/* /< SS LP Horizontal Adj Sense Ix2 min/Max
				  * (for each node) test */
	int SelfSenseIxTotalLP;	/* /< SS LP Total Horizontal Sense Ix min/Max
				  * (for each node) test */
	int SelfSenseIxTotalAdjLP; /* /< SS LP Total Horizontal Adj Sense Ix
					 * min/Max (for each node) test */
	int SelfSenseCx1LP;	/* /< SS LP Sense Cx1 min/Max test */
	int SelfSenseCx2LP; /* /< SS LP Sense Cx2 min/Max (for each node)
				* test */
	int SelfSenseCx2AdjLP;	/* /< SS LP Horizontal Adj Sense Cx2 min/Max
				  * (for each node) test */
	int SelfSenseCxTotalLP;	/* /< SS LP Total Sense Cx min/Max
				 * (for each node) test */
	int SelfSenseCxTotalAdjLP; /* /< SS LP Total Horizontal Adj Sense Cx
					 * min/Max (for each node) test */
} TestToDo;

#define MAX_LIMIT_FILE_NAME 100	/* /< max number of chars of the limit file name */

/**
  * Struct which store the data coming from a Production Limit File
  */
typedef struct {
	char *data;	/* /< pointer to an array of char which contains
			  * the content of the Production Limit File */
	int size;	/* /< size of data */
	char name[MAX_LIMIT_FILE_NAME];	/* /< identifier of the source from
					 * where the limits data were loaded (if
					 * loaded from a file it will be the
					 * file name, while if loaded from .h
					 * will be "NULL") */
} LimitFile;

/* Size in bytes of System Info data */
#define SYS_INFO_SIZE			216	/* Num bytes of die info */
#define DIE_INFO_SIZE			16	/* Num bytes of external release
						 * in config */
#define EXTERNAL_RELEASE_INFO_SIZE	8	/* Num bytes of release info in
						  * sys info
						  *  (first bytes are external
						  *release) */
#define RELEASE_INFO_SIZE		(EXTERNAL_RELEASE_INFO_SIZE)

/**
  * Struct which contains fundamental informations about the chip and its
  *configuration
  */
typedef struct {
	u16 u16_apiVer_rev;	/* /< API revision version */
	u8 u8_apiVer_minor;	/* /< API minor version */
	u8 u8_apiVer_major;	/* /< API major version */
	u16 u16_chip0Ver;	/* /< Dev0 version */
	u16 u16_chip0Id;	/* /< Dev0 ID */
	u16 u16_chip1Ver;	/* /< Dev1 version */
	u16 u16_chip1Id;	/* /< Dev1 ID */
	u16 u16_fwVer;	/* /< Fw version */
	u16 u16_svnRev;	/* /< SVN Revision */
	u16 u16_cfgVer;	/* /< Config Version */
	u16 u16_cfgProjectId;	/* /< Config Project ID */
	u16 u16_cxVer;	/* /< Cx Version */
	u16 u16_cxProjectId;	/* /< Cx Project ID */
	u8 u8_cfgAfeVer;	/* /< AFE version in Config */
	u8 u8_cxAfeVer;	/* /< AFE version in CX */
	u8 u8_panelCfgAfeVer;	/* /< AFE version in PanelMem */
	u8 u8_protocol;	/* /< Touch Report Protocol */
	u8 u8_dieInfo[DIE_INFO_SIZE];	/* /< Die information */
	u8 u8_releaseInfo[RELEASE_INFO_SIZE];	/* /< Release information */
	u32 u32_fwCrc;	/* /< Crc of FW */
	u32 u32_cfgCrc;	/* /< Crc of config */
	u8 u8_mpFlag; /* /< MP Flag */
	u8 u8_ssDetScanSet; /* /< Type of Detect Scan Selected */

	u16 u16_scrResX;	/* /< X resolution on main screen */
	u16 u16_scrResY;	/* /< Y resolution on main screen */
	u8 u8_scrTxLen;	/* /< Tx length */
	u8 u8_scrRxLen;	/* /< Rx length */
	u8 u8_keyLen;	/* /< Key Len */
	u8 u8_forceLen;	/* /< Force Len */
	u32 u32_productionTimestamp;	/* /< Production Timestamp */

	u16 u16_dbgInfoAddr;	/* /< Offset of debug Info structure */

	u16 u16_msTchRawAddr;	/* /< Offset of MS touch raw frame */
	u16 u16_msTchFilterAddr;	/* /< Offset of MS touch filter frame */
	u16 u16_msTchStrenAddr;	/* /< Offset of MS touch strength frame */
	u16 u16_msTchBaselineAddr;	/* /< Offset of MS touch baseline frame
					 * */

	u16 u16_ssTchTxRawAddr;	/* /< Offset of SS touch force raw frame */
	u16 u16_ssTchTxFilterAddr;	/* /< Offset of SS touch force filter
					 * frame */
	u16 u16_ssTchTxStrenAddr;	/* /< Offset of SS touch force strength
					 * frame */
	u16 u16_ssTchTxBaselineAddr;	/* /< Offset of SS touch force baseline
					 * frame */

	u16 u16_ssTchRxRawAddr;	/* /< Offset of SS touch sense raw frame */
	u16 u16_ssTchRxFilterAddr;	/* /< Offset of SS touch sense filter
					 * frame */
	u16 u16_ssTchRxStrenAddr;	/* /< Offset of SS touch sense strength
					 * frame */
	u16 u16_ssTchRxBaselineAddr;	/* /< Offset of SS touch sense baseline
					 * frame */

	u16 u16_keyRawAddr;	/* /< Offset of key raw frame */
	u16 u16_keyFilterAddr;	/* /< Offset of key filter frame */
	u16 u16_keyStrenAddr;	/* /< Offset of key strength frame */
	u16 u16_keyBaselineAddr;	/* /< Offset of key baseline frame */

	u16 u16_frcRawAddr;	/* /< Offset of force touch raw frame */
	u16 u16_frcFilterAddr;	/* /< Offset of force touch filter frame */
	u16 u16_frcStrenAddr;	/* /< Offset of force touch strength frame */
	u16 u16_frcBaselineAddr;	/* /< Offset of force touch baseline
					 * frame */

	u16 u16_ssHvrTxRawAddr;	/* /< Offset of SS hover Force raw frame */
	u16 u16_ssHvrTxFilterAddr;	/* /< Offset of SS hover Force filter
					 * frame */
	u16 u16_ssHvrTxStrenAddr;	/* /< Offset of SS hover Force strength
					 * frame */
	u16 u16_ssHvrTxBaselineAddr;	/* /< Offset of SS hover Force baseline
					 * frame */

	u16 u16_ssHvrRxRawAddr;	/* /< Offset of SS hover Sense raw frame */
	u16 u16_ssHvrRxFilterAddr;	/* /< Offset of SS hover Sense filter
					 * frame */
	u16 u16_ssHvrRxStrenAddr;	/* /< Offset of SS hover Sense strength
					 * frame */
	u16 u16_ssHvrRxBaselineAddr;	/* /< Offset of SS hover Sense baseline
					 * frame */

	u16 u16_ssPrxTxRawAddr;	/* /< Offset of SS proximity force raw frame */
	u16 u16_ssPrxTxFilterAddr;	/* /< Offset of SS proximity force
					 * filter frame */
	u16 u16_ssPrxTxStrenAddr;	/* /< Offset of SS proximity force
					 * strength frame */
	u16 u16_ssPrxTxBaselineAddr;	/* /< Offset of SS proximity force
					 * baseline frame */

	u16 u16_ssPrxRxRawAddr;	/* /< Offset of SS proximity sense raw frame */
	u16 u16_ssPrxRxFilterAddr;	/* /< Offset of SS proximity sense
					 * filter frame */
	u16 u16_ssPrxRxStrenAddr;	/* /< Offset of SS proximity sense
					 * strength frame */
	u16 u16_ssPrxRxBaselineAddr;	/* /< Offset of SS proximity sense
					 * baseline frame */

	u16 u16_ssDetRawAddr;		/* /< Offset of SS detect raw frame */
	u16 u16_ssDetFilterAddr;	/* /< Offset of SS detect filter
					 * frame */
	u16 u16_ssDetStrenAddr;		/* /< Offset of SS detect strength
					 * frame */
	u16 u16_ssDetBaselineAddr;	/* /< Offset of SS detect baseline
					 * frame */
} SysInfo;

/**
  * Possible actions that can be requested by an host
  */
typedef enum {
	ACTION_WRITE				= (u16) 0x0001,	/* /< Bus Write
								 * */
	ACTION_READ				= (u16) 0x0002,	/* /< Bus Read
								 * */
	ACTION_WRITE_READ			= (u16) 0x0003,	/* /< Bus Write
								 * followed by a
								 * Read */
	ACTION_GET_VERSION			= (u16) 0x0004,	/* /< Get
								 * Version of
								 * the protocol
								 * (equal to the
								 * first 2 bye
								 * of driver
								 * version) */
	ACTION_WRITEU8UX			= (u16) 0x0011,	/* /< Bus Write
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITEREADU8UX			= (u16) 0x0012,	/* /< Bus
								 * writeRead
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITETHENWRITEREAD		= (u16) 0x0013,	/* /< Bus write
								 * followed by a
								 * writeRead */
	ACTION_WRITEU8XTHENWRITEREADU8UX	= (u16) 0x0014,	/* /< Bus write
								 * followed by a
								 * writeRead
								 * with support
								 * to different
								 * address size
								 * */
	ACTION_WRITEU8UXTHENWRITEU8UX		= (u16) 0x0015,	/* /< Bus write
								 * followed by a
								 * write with
								 * support to
								 * different
								 * address size
								 * */
	ACTION_GET_FW				= (u16) 0x1000,	/* /< Get Fw
								 * file content
								 * used by the
								 * driver */
	ACTION_GET_LIMIT			= (u16) 0x1001	/* /< Get Limit
								 * File content
								 * used by the
								 * driver */
} Actions;

/**
  * Struct used to contain info of the message received by the host in
  * Scriptless mode
  */
typedef struct {
	u16 msg_size;	/* /< total size of the message in bytes */
	u16 counter;	/* /< counter ID to identify a message */
	Actions action;	/* /< type of operation requested by the host @see
			 * Actions */
	u8 dummy;	/* /< (optional)in case of any kind of read operations,
			 * specify if the first byte is dummy */
} Message;
#define CHUNK_PROC		1024	/* /< Max info->chunk of data info->printed on the
					 * sequential file in each iteration */

/*
  * Forward declaration
  */
struct fts_ts_info;
extern char tag_st80y[12];	/* /< forward the definition of the label used
			  * to print the log in the kernel log */

/*
  * Dispatch event handler
  */
typedef void (*event_dispatch_handler_t)
	(struct fts_ts_info *info, unsigned char *data, ktime_t kt);

/**
  * FTS capacitive touch screen device information
  * - dev             Pointer to the structure device \n
  * - client          client structure \n
  * - work            Work thread \n
  * - event_dispatch_table  Event dispatch table handlers \n
  * - attrs           SysFS attributes \n
  * - mode            Device operating mode (bitmask) \n
  * - touch_id        Bitmask for touch id (mapped to input slots) \n
  * - stylus_id       Bitmask for tracking the stylus touches (mapped using the
  * touchId) \n
  * - timer           Timer when operating in polling mode \n
  * - power           Power on/off routine \n
  * - board           HW info retrieved from device tree \n
  * - vdd_reg         DVDD power regulator \n
  * - avdd_reg        AVDD power regulator \n
  * - resume_bit      Indicate if screen off/on \n
  * - fwupdate_stat   Store the result of a fw update triggered by the host \n
  * - sensor_sleep    true suspend was called, false resume was called \n
  * - series_of_switches  to store the enabling status of a particular feature
  * from the host \n
  */
struct fts_ts_info {
	struct device            *dev;	/* /< Pointer to the structure device */
	struct vts_device *vtsdev;
#ifdef I2C_INTERFACE
	struct i2c_client        *client;	/* /< I2C client structure */
#else
	struct spi_device        *client;	/* /< SPI client structure */
#endif

	event_dispatch_handler_t *event_dispatch_table;	/* /< Event dispatch
							 * table handlers */

	struct attribute_group attrs;	/* /< SysFS attributes */

	unsigned int mode;	/* /< Device operating mode (bitmask: msb
				 * indicate if active or lpm) */
	unsigned long touch_id;	/* /< Bitmask for touch id (mapped to input
				 * slots) */
#ifdef STYLUS_MODE
	unsigned long stylus_id;	/* /< Bitmask for tracking the stylus
					 * touches (mapped using the touchId) */
#endif
#ifdef PHONE_KEY
	u8 key_mask;	/* /< store the last update of the key mask
					 * published by the IC */
#endif
#ifdef USE_ONE_FILE_NODE
	int feature_feasibility;
#endif
	u32 typeOfComand[CMD_STR_LEN];	/* /< buffer used to store the
							  * command sent from the MP
							  * device file node */
	int numberParameters;	/* /< number of parameter passed through the MP
					  * device file node */

	struct fts_hw_platform_data board;	/* /< HW info retrieved from
						 * device tree */
	struct regulator *vdd_reg;	/* /< DVDD power regulator */
	struct regulator *avdd_reg;	/* /< AVDD power regulator */


	int resume_bit;	/* /< Indicate if screen off/on */
	int fwupdate_stat;	/* /< Store the result of a fw update triggered
				 * by the host */
	bool sensor_sleep;	/* /< if true suspend was called while if false
				 * resume was called */

	/* switches for features */
	int gesture_enabled;	/* /< if set, the gesture mode will be enabled
				 * during the suspend */
	int glove_enabled;	/* /< if set, the glove mode will be enabled
				 * when allowed */
	int charger_enabled;	/* /< if set, the charger mode will be enabled
				 * when allowed */
	int stylus_enabled;	/* /< if set, the stylus mode will be enabled
				 * when allowed */
	int cover_enabled;	/* /< if set, the cover mode will be enabled
				 * when allowed */
	int grip_enabled;	/* /< if set, the grip mode mode will be enabled
				 * when allowed */
		/* /< store the gesture bitmask which the host want to enable.
	  * If bit set 1 the corresponding gesture will be detected in Gesture Mode */
	u8 gesture_mask[GESTURE_MASK_SIZE];
	/* /< store the x coordinates of the points draw by the user
	  * when a gesture is detected */
	u16 gesture_coordinates_x[GESTURE_MAX_COORDS_PAIRS_REPORT];
	/* /< store the y coordinates of the points draw by the user
	  * when a gesture is detected */
	u16 gesture_coordinates_y[GESTURE_MAX_COORDS_PAIRS_REPORT];
	/* /< number of coordinates pairs (points) reported with the detected gesture */
	int gesture_coords_reported;
	u8 refreshGestureMask;	/* /< flag which indicate if there is
					 * the need to set the gesture mask in the FW */
	struct mutex st80y_gestureMask_mutex;	/* /< mutex used to control access on gesture
					 * shared variables */

	ErrorList errors;	/* /< private variable which implement the Error */

	TestToDo tests_st80y;	/* /< global variable that specify the tests to perform during
		 * the Mass Production Test */
	LimitFile limit_file;	/* /< variable which contains the limit file */

	SysInfo systemInfo;	/* /< Global System Info variable, accessible in all the
				 * during test */

	int reset_gpio;	/* /< gpio number of the rest
							 * pin, the value is
							 *  GPIO_NOT_DEFINED if the
							 * reset pin is not connected */
	int system_reseted_up;	/* /< flag checked during resume to
						 * understand if there was a system
						 * reset and restore the proper state */
	int system_reseted_down;	/* /< flag checked during suspend to
						 * understand if there was a system
						 * reset and restore the proper state */
	atomic_t disable_irq_count;	/* /< count the number of call to
						 * disable_irq, start with 1 because at
						 * the boot IRQ are already disabled */
	struct proc_dir_entry *fts_dir;
	Message mess;	/* /< store the information of the Scriptless
				 * message received */
	u8 bin_output;	/* /< Select the output type of the scriptless
				 * protocol (binary = 1  or hex string = 0) */

	int limit; /* /< store the amount of data to print into the shell */
	int chunk;	/* /< store the chuk of data that should be printed in
				 * this iteration */
	int printed;	/* /< store the amount of data already printed in the
				 * shell */
	u8 *driver_test_buff;	/* /< pointer to an array of bytes used
						 * to store the result of the function
						 * executed */
	char buf_chunk[CHUNK_PROC];	/* /< buffer used to store the message
						 * info received */

	int last_mode_state;
	u8 flash_write_times;
};



int st80y_chip_powercycle(struct fts_ts_info *info);

/* export declaration of functions in fts_proc.c */
extern int st80y_proc_init(struct fts_ts_info *info);
extern int st80y_proc_remove(struct fts_ts_info *info);
extern int st80y_tools_dev_init(struct vts_device *vtsdev);
extern int fts_mode_handler_extern(struct fts_ts_info *info, int force);
#define fts_err(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_err(info->vtsdev, fmt, ##param); \
		else \
			VTE(fmt, ##param); \
	} while (0)

#define fts_info(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_info(info->vtsdev, fmt, ##param); \
		else \
			VTI(fmt, ##param); \
	} while (0)

#define fts_dbg(info, fmt, param...) do { \
		if(likely(info && info->vtsdev)) \
			vts_dev_dbg(info->vtsdev, fmt, ##param); \
		else \
			VTD(fmt, ##param); \
	} while (0)

#endif
