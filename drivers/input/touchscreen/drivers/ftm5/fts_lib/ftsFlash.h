/*
  *
  **************************************************************************
  **                        STMicroelectronics                            **
  **************************************************************************
  **                        marco.cali@st.com                             **
  **************************************************************************
  *                                                                        *
  *                       FTS API for Flashing the IC                      *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsFlash.h
  * \brief Contains all the definitions and structs to handle the FW update
  *process
  */

#ifndef FTS_FLASH_H
#define FTS_FLASH_H

#include "ftsSoftware.h"

/* Flash possible status */
#define FLASH_READY		0	/* /< value to indicate that the flash
					 * is ready */
#define FLASH_BUSY		1	/* /< value to indicate that the flash
					 * is busy */
#define FLASH_UNKNOWN		-1	/* /< value to indicate an unknown
					 * status of the flash */

#define FLASH_STATUS_BYTES	1	/* /< number of bytes to check for read
					 * the flash status */



/* Flash timing parameters */
#define FLASH_RETRY_COUNT	200	/* /< number of attemps to read the
					 * flash status */
#define FLASH_WAIT_BEFORE_RETRY 50	/* /< time to wait in ms between status
					  * readings */
#define PATH_FILE_FW		"st_fts.ftb"	/* /< new FW bin file name */

#define FLASH_CHUNK		(32 * 1024)	/* /< Max number of bytes that
						 * the DMA can burn on the flash
						 * in one shot in FTI */
#define DMA_CHUNK		32	/* /< Max number of bytes that can be
					 * written in I2C to the DMA */


/**
  * Define which kind of erase page by page should be performed
  */
typedef enum {
	ERASE_ALL		= 0,	/* /< erase all the pages */
	SKIP_PANEL_INIT		= 1,	/* /< skip erase Panel Init Pages */
	SKIP_PANEL_CX_INIT	= 2 /* /< skip erase Panel Init and CX Pages */
} ErasePage;

/** @addtogroup fw_file
  * @{
  */

/**
  * Struct which contains information and data of the FW that should be burnt
  *into the IC
  */
typedef struct {
	u8 *data;	/* /< pointer to an array of bytes which represent the
			 * FW data */
	u16 fw_ver;	/* /< FW version of the FW file */
	u16 config_id;	/* /< Config ID of the FW file */
	u16 cx_ver;	/* /< Cx version of the FW file */
	u8 externalRelease[EXTERNAL_RELEASE_INFO_SIZE];	/* /< External Release
							 * Info of the FW file
							 * */
	int data_size;	/* /< dimension of data (the actual data to be burnt) */
	u32 sec0_size;	/* /< dimension of section 0 (FW) in .ftb file */
	u32 sec1_size;	/* /< dimension of section 1 (Config) in .ftb file */
	u32 sec2_size;	/* /< dimension of section 2 (Cx) in .ftb file */
	u32 sec3_size;	/* /< dimension of section 3 (TBD) in .ftb file */
} Firmware;

/** @}*/

/** @addtogroup flash_command
  * @{
  */

int wait_for_flash_ready_ftm5(struct fts_ts_info *info, u8 type);
int hold_m3(struct fts_ts_info *info);
int flash_erase_unlock_ftm5(struct fts_ts_info *info);
int flash_full_erase_ftm5(struct fts_ts_info *info);
int flash_erase_page_by_page_V2(struct fts_ts_info *info, ErasePage keep_cx);
int start_flash_dma_ftm5(struct fts_ts_info *info);
int fillFlash_ftm5(struct fts_ts_info *info, u32 address, u8 *data, int size);

int flash_unlock_ftm5(struct fts_ts_info *info);
int getFWdata_ftm5(struct fts_ts_info *info, const char *pathToFile, u8 **data, int *size);
int parseBinFile_ftm5(struct fts_ts_info *info, u8 *fw_data, int fw_size, Firmware *fw, int keep_cx);
int readFwFile_ftm5(struct fts_ts_info *info, const char *path, Firmware *fw, int keep_cx);
int flash_burn_ftm5(struct fts_ts_info *info, Firmware fw, int force_burn, int keep_cx);
int flashProcedure_ftm5(struct fts_ts_info *info, const char *path, int force, int keep_cx);
int flash_enable_uvlo_autopowerdown(struct fts_ts_info *info);

int ftm5_readLockDownInfo(struct fts_ts_info *info, u8 *lockData,u8 lock_id,int size);
int ftm5_writeLockDownInfo(struct fts_ts_info *info, u8 *data, int size, u8 lock_id);

#endif

/** @}*/
