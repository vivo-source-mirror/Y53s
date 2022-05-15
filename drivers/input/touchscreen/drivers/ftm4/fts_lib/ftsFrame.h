/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                  FTS functions for getting frames			 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */


#include "ftsSoftware.h"



/*Number of data bytes for each node */
#define BYTES_PER_NODE						2

#define OFFSET_LENGTH						2

#define FRAME_DATA_HEADER				8

#define FRAME_HEADER_SIGNATURE			0xB5

#define FRAME_DATA_READ_RETRY			2

typedef struct {
	DataHeader header;
	short *node_data;
	int node_data_size;
} MutualSenseFrame;

typedef struct {
	DataHeader header;
	short *force_data;
	short *sense_data;
} SelfSenseFrame;

int ftm4_getOffsetFrame(struct fts_ts_info *info, u16 address, u16 *offset);
int ftm4_getChannelsLength(struct fts_ts_info *info, int *sense_len, int *force_len);
int ftm4_getFrameData(struct fts_ts_info *info, u16 address, int size, short **frame);
int ftm4_getMSFrame(struct fts_ts_info *info, u16 type, short **frame, int keep_first_row);
/*int getMSKeyFrame(u16 type, short **frame); */
int ftm4_getSSFrame(struct fts_ts_info *info, u16 type, short **frame);
/*int getNmsFrame(u16 type, short ***frames, int * sizes, int keep_first_row, int fs, int n); */
int ftm4_getSenseLen(struct fts_ts_info *info);
int ftm4_getForceLen(struct fts_ts_info *info);
int ftm4_requestFrame(struct fts_ts_info *info, u16 type);
int ftm4_readFrameDataHeader(struct fts_ts_info *info, u16 type, DataHeader *header);
int ftm4_getMSFrame2(struct fts_ts_info *info, u16 type, MutualSenseFrame *frame);
int ftm4_getSSFrame2(struct fts_ts_info *info, u16 type, SelfSenseFrame *frame);

