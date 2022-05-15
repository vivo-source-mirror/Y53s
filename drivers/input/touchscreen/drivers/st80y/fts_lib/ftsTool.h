/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     FTS Utility Functions				   *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsTool.h
  * \brief Contains all the definitions to support common operations inside the
  * driver
  */

#ifndef FTS_TOOL_H
#define FTS_TOOL_H


char *st80y_printHex(char *label, u8 *buff, int count, u8 *result);
int st80y_u8ToU16(u8 *src, u16 *dst);
int u8ToU16_be(u8 *src, u16 *dst);
int st80y_u8ToU16n(u8 *src, int src_length, u16 *dst);
int st80y_u16ToU8(u16 src, u8 *dst);
int st80y_u16ToU8_be(u16 src, u8 *dst);
int u16ToU8n_be(u16 *src, int src_length, u8 *dst);
int st80y_u8ToU32(u8 *src, u32 *dst);
int u8ToU32_be(u8 *src, u32 *dst);
int st80y_u32ToU8(u32 src, u8 *dst);
int u32ToU8_be(u32 src, u8 *dst);
int u8ToU64_be(u8 *src, u64 *dest, int size);
int u64ToU8_be(u64 src, u8 *dest, int size);
int st80y_attempt_function(int (*code)(void), unsigned long wait_before_retry, int
		     retry_count);
int st80y_senseOn(struct fts_ts_info *info);
int st80y_senseOff(struct fts_ts_info *info);
void st80y_print_frame_short(char *label, short **matrix, int row, int column);
short **st80y_array1dTo2d_short(short *data, int size, int columns);
void print_frame_u16(char *label, u16 **matrix, int row, int column);
u16 **array1dTo2d_u16(u16 *data, int size, int columns);
u8 **st80y_array1dTo2d_u8(u8 *data, int size, int columns);
i8 **array1dTo2d_i8(i8 *data, int size, int columns);
void st80y_print_frame_u8(char *label, u8 **matrix, int row, int column);
void print_frame_i8(char *label, i8 **matrix, int row, int column);
void st80y_print_frame_u32(char *label, u32 **matrix, int row, int column);
void st80y_print_frame_int(char *label, int **matrix, int row, int column);
int st80y_cleanUp(struct fts_ts_info *info, int enableTouch);
int st80y_flushFIFO(struct fts_ts_info *info);

/* New API */
int fromIDtoMask(u8 id, u8 *mask, int size);

#endif
