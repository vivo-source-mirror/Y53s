/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *               	FTS API for Flashing the IC			 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsSoftware.h"

/*Flash possible status */
#define FLASH_READY				0
#define FLASH_BUSY				1
#define FLASH_UNKNOWN			-1

#define FLASH_STATUS_BYTES		1

/*Flash timing parameters */
#define FLASH_RETRY_COUNT		1000
#define FLASH_WAIT_BEFORE_RETRY         50			/*ms */

#define FLASH_WAIT_TIME                 200			/*ms */


/*PATHS FW FILES */
#define PATH_FILE_FW			"st_fts.ftb"		/*new bin file structure */

#define FLASH_CHUNK			(64 * 1024)
#define DMA_CHUNK			(2 * 1024)

typedef struct {
	u8 *data;
	u16 fw_ver;
	u16 config_id;
	u8 externalRelease[EXTERNAL_RELEASE_INFO_SIZE];
	int data_size;

	u32 sec0_size;
	u32 sec1_size;
	u32 sec2_size;
	u32 sec3_size;
} Firmware;

int ftm4_wait_for_flash_ready(struct fts_ts_info *info, u8 type);
int ftm4_warm_boot(struct fts_ts_info *info);
int ftm4_flash_erase_unlock(struct fts_ts_info *info);
int ftm4_flash_full_erase(struct fts_ts_info *info);
int ftm4_start_flash_dma(struct fts_ts_info *info);
int ftm4_fillFlash(struct fts_ts_info *info, u32 address, u8 *data, int size);

int ftm4_flash_unlock(struct fts_ts_info *info);
int fillMemory(u32 address, u8 *data, int size);
int ftm4_getFirmwareVersion(struct fts_ts_info *info, u16 *fw_vers, u16 *config_id);
int ftm4_getFWdata(struct fts_ts_info *info, const char *pathToFile, u8 **data, int *size, int from);
int ftm4_parseBinFile(struct fts_ts_info *info, const u8 *fw_data, int fw_size, Firmware *fw, int keep_cx);
int ftm4_readFwFile(struct fts_ts_info *info, const char *path, Firmware *fw, int keep_cx);
int ftm4_flash_burn(struct fts_ts_info *info, Firmware fw, int force_burn, int keep_cx);
int ftm4_flashProcedure(struct fts_ts_info *info, const char *path, int force, int keep_cx);

/*sunsl add */
void initFtsFlash(int *flag);

