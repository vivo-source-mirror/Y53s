/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                     FTS Utility Functions				 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */
#define GPIO_NOT_DEFINED		(-1)

#define TIMEOUT_RESOLUTION		10					/*ms */
#define GENERAL_TIMEOUT			(50 * TIMEOUT_RESOLUTION)			/*ms */
#define RELEASE_INFO_TIMEOUT		(15 * TIMEOUT_RESOLUTION)			/*ms */

#define FEAT_ENABLE				1
#define FEAT_DISABLE			0

#define SYSTEM_RESET_RETRY		3

#define B2_RETRY				2

#define LOCKDOWN_CODE_SIZE		10		/*for FTM4 can not be greater than 13 bytes */

int ftm4_readB2(struct fts_ts_info *info, u16 address, u8 *outBuf, int len);
int ftm4_readB2U16(struct fts_ts_info *info, u16 address, u8 *outBuf, int byteToRead);
int ftm4_releaseInformation(struct fts_ts_info *info);
int ftm4_lockDownInfo(struct fts_ts_info *info, u8 *data);
char *ftm4_printHex(char *label, u8 *buff, int count);
int ftm4_pollForEvent(struct fts_ts_info *info, int *event_to_search, int event_bytes, u8 *readData, int time_to_wait);
int ftm4_disableInterrupt(struct fts_ts_info *info);
int ftm4_isInterruptEnabled(struct fts_ts_info *info, bool *is_enabled);
int ftm4_enableInterrupt(struct fts_ts_info *info);
int fts_u8ToU16(u8 *src, u16 *dst);
int fts_u8ToU16_le(u8 *src, u16 *dst);
int fts_u8ToU16n(u8 *src, int src_length, u16 *dst);
int ftm4_u16ToU8(u16 src, u8 *dst);
int ftm4_u16ToU8_le(u16 src, u8 *dst);
int ftm4_u16ToU8_be(u16 src, u8 *dst);
int ftm4_u16ToU8n(u16 *src, int src_length, u8 *dst);
int ftm4_u8ToU32(const u8 *src, u32 *dst);
int fts_u32ToU8(u32 src, u8 *dst);
int ftm4_attempt_function(int(*code)(void), unsigned long wait_before_retry, int retry_count);
int ftm4_system_reset(struct fts_ts_info *info);
/*
int isSystemResettedUp(void);
int isSystemResettedDown(void);
void setSystemResettedUp(int val);
void setSystemResettedDown(int val);
 */
int ftm4_senseOn(struct fts_ts_info *info);
int ftm4_senseOff(struct fts_ts_info *info);
int ftm4_keyOn(struct fts_ts_info *info);
int keyOff(void);
int ftm4_featureEnableDisable(struct fts_ts_info *info, int on_off, u32 feature);
int ftm4_writeNoiseParameters(struct fts_ts_info *info, u8 *noise);
int ftm4_readNoiseParameters(struct fts_ts_info *info, u8 *noise);
int ftm4_checkEcho(struct fts_ts_info *info, u8 *cmd, int size);
void ftm4_print_frame_short(struct fts_ts_info *info, char *label, short **matrix, int row, int column);
short **ftm4_array1dTo2d_short(short *data, int size, int columns);
u8 **ftm4_array1dTo2d_u8(u8 *data, int size, int columns);
void ftm4_print_frame_u8(struct fts_ts_info *info, char *label, u8 **matrix, int row, int column);
void ftm4_print_frame_u32(struct fts_ts_info *info, char *label, u32 **matrix, int row, int column);
void ftm4_print_frame_int(struct fts_ts_info *info, char *label, int **matrix, int row, int column);
int ftm4_cleanUp(struct fts_ts_info *info, int enableTouch);
int ftm4_flushFIFO(struct fts_ts_info *info);
