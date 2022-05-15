/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                  FTS functions for getting frames			 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsCrossCompile.h"
#include "ftsCompensation.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTool.h"
#include "ftsTime.h"

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
/*#include <linux/sec_sysfs.h>*/

int ftm4_getOffsetFrame(struct fts_ts_info *info, u16 address, u16 *offset)
{
	u8 data[2];
	u8 cmd = { FTS_CMD_FRAMEBUFFER_R };
	char *temp = NULL;


	if (ftm4_readCmdU16(info, cmd, address, data, OFFSET_LENGTH, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_R);
		return ERROR_I2C_R;
	} else {
		fts_u8ToU16(data, offset);
		temp = ftm4_printHex("Offest = ", data, OFFSET_LENGTH);
		if (temp != NULL)
			fts_err(info, "%s", temp);
		kfree(temp);
		return OK;
	}

}


int ftm4_getChannelsLength(struct fts_ts_info *info, int *sense_len, int *force_len)
{

	int ret;
	u8 *data = (u8 *)kmalloc(2 * sizeof(u8), GFP_KERNEL);

	if (data == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_readB2(info, ADDR_SENSE_LEN, data, 2);
	if (ret < OK) {
		 fts_err(info, "ERROR %02X\n", ERROR_READ_B2);
		 kfree(data);
		 return (ret | ERROR_READ_B2);
	}

	*sense_len = (int)data[0];
	*force_len = (int)data[1];

	fts_err(info, "Force_len = %d   Sense_Len = %d \n", *force_len, *sense_len);

	kfree(data);

	return OK;
}


int ftm4_getFrameData(struct fts_ts_info *info, u16 address, int size, short **frame)
{
	int i, j, ret;
	u8 *data = (u8 *)kmalloc(size * sizeof(u8), GFP_KERNEL);
	if (data == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, address, data, size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_I2C_R);
				kfree(data);
		return ERROR_I2C_R;
	}
	j = 0;
	for (i = 0; i < size; i += 2) {
		(*frame)[j] = (short)((data[i + 1] << 8) + data[i]);
		j++;
	}
	kfree(data);
	return OK;
}


int ftm4_getMSFrame(struct fts_ts_info *info, u16 type, short **frame, int keep_first_row)
{
	u16 offset;
	int size, ret;
	int sense_len;
	int force_len;

	if (ftm4_getSenseLen(info) == 0 || ftm4_getForceLen(info) == 0) {
		ret = ftm4_getChannelsLength(info, &sense_len, &force_len);
		if (ret < OK) {
			fts_err(info, "ERROR %02X\n", ERROR_CH_LEN);
			return (ret | ERROR_CH_LEN);
		}
	}


	ret = ftm4_getOffsetFrame(info, type, &offset);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_OFFSET);
		return (ret | ERROR_GET_OFFSET);
	}

	switch (type) {
	case ADDR_RAW_TOUCH:
	case ADDR_FILTER_TOUCH:
	case ADDR_NORM_TOUCH:
	case ADDR_CALIB_TOUCH:
		if (keep_first_row == 1)
			size = ((force_len + 1) * sense_len);
		else {
			size = ((force_len) * sense_len);
			offset += (sense_len * BYTES_PER_NODE);
		}
		break;

	default:
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}


	*frame = (short *)kmalloc(size * sizeof(short), GFP_KERNEL);
	if (*frame == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_getFrameData(info, offset, size * BYTES_PER_NODE, frame);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_FRAME_DATA);
		return (ret | ERROR_GET_FRAME_DATA);
	}
	/* if you want to access one node i, j, you should compute the offset like: offset = i *columns + j = > frame[i, j]	 */

	fts_err(info, "Frame acquired! \n");
	return size;																		/*return the number of data put inside frame */

}

int ftm4_getMSFrame3(struct fts_ts_info *info, u16 type, MutualSenseFrame *frame, int keep_first_row) {
	u16 offset;
	int ret;
	int sense_len;
	int force_len;

	if (ftm4_getSenseLen(info) == 0 || ftm4_getForceLen(info) == 0) {
		ret=ftm4_getChannelsLength(info, &sense_len, &force_len);
		if(ret<OK){
			fts_err(info, "ERROR %02X\n",ERROR_CH_LEN);
			return (ret|ERROR_CH_LEN);
		}
	}
 
	ret = ftm4_getOffsetFrame(info, type, &offset);
	if (ret<OK) {
			 fts_err(info, "ERROR %02X\n", ERROR_GET_OFFSET);
			 return (ret | ERROR_GET_OFFSET);
	}

	switch (type) {
		case ADDR_RAW_TOUCH:                                          
		case ADDR_FILTER_TOUCH:                                                  
		case ADDR_NORM_TOUCH:                                                    
		case ADDR_CALIB_TOUCH:
			if(keep_first_row ==1){
				frame->node_data_size = ((force_len+1)*sense_len);
				frame->header.force_node = force_len+1;
			}else {
				frame->node_data_size = ((force_len)*sense_len);
				offset+= (sense_len * BYTES_PER_NODE);
				frame->header.force_node = force_len;
			}
			frame->header.sense_node = sense_len;
			break;

		default:
			fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
			return ERROR_OP_NOT_ALLOW;
	}


	fts_info(info, "##frame->node_data_size:%d, force_len:%d, sense_len:%d\n", frame->node_data_size, force_len, sense_len);
	frame->node_data = (short*)kmalloc(frame->node_data_size*sizeof(short), GFP_KERNEL);
	if(frame->node_data==NULL){
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret=ftm4_getFrameData(info, offset, frame->node_data_size*BYTES_PER_NODE, &(frame->node_data));
	if(ret<OK){
		fts_err(info, "ERROR %02X\n", ERROR_GET_FRAME_DATA);
		kfree(frame->node_data);
		return (ret|ERROR_GET_FRAME_DATA);         
	}
	//if you want to access one node i,j, you should compute the offset like: offset = i*columns + j = > frame[i, j]    

	fts_err(info, "Frame acquired! \n");
	return frame->node_data_size;                                                           //return the number of data put inside frame
}


/*int getMSKeyFrame(u16 type, short **frame) {
	u16 offset;
	int size, ret;
	u16 address;
	MutualSenseData data;

	if (type != ADDR_RAW_MS_KEY) {
		fts_err(info, " getMSKeyFrame: ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_getOffsetFrame(info, type, &offset);
	if (ret < OK) {
		fts_err(info, " getMSKeyFrame: ERROR %02X\n", ERROR_GET_OFFSET);
		return (ret | ERROR_GET_OFFSET);
	}

	ret = ftm4_requestCompensationData(MS_KEY);
	if (ret < OK) {
		fts_err(info, " getMSKeyFrame: ftm4_readMutualSenseCompensationData ERROR %02X\n", ERROR_REQU_COMP_DATA);
		return (ret | ERROR_REQU_COMP_DATA);
	}

	ret = ftm4_readCompensationDataHeader(MS_KEY, &(data.header), &address);
	if (ret < OK) {
		fts_err(info, " getMSKeyFrame: ftm4_readMutualSenseCompensationData ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return (ret | ERROR_COMP_DATA_HEADER);
	}

	if (data.header.force_node > data.header.sense_node)
		size = data.header.force_node;
	else
		size = data.header.sense_node;

	 *frame = (short *)kmalloc(size *sizeof(short), GFP_KERNEL);
	if (frame == NULL) {
		fts_err(info, " getMSKeyFrame: ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_getFrameData(offset, size *BYTES_PER_NODE, frame);
	if (ret < OK) {
		fts_err(info, " getMSKeyFrame: ERROR %02X\n", ERROR_GET_FRAME_DATA);
		return (ret | ERROR_GET_FRAME_DATA);
	}

	fts_err(info, " Frame acquired! \n");
	return size;																		//return the number of data put inside frame

}

*/

int ftm4_getSSFrame(struct fts_ts_info *info, u16 type, short **frame) {
	u16 offset;
	int size, ret;
	int sense_len;
	int force_len;

	if (ftm4_getSenseLen(info) == 0 || ftm4_getForceLen(info) == 0) {
		ret = ftm4_getChannelsLength(info, &sense_len, &force_len);
		if (ret < 0) {
			fts_err(info, "ERROR %02X\n", ERROR_CH_LEN);
			return (ret | ERROR_CH_LEN);
		}
	}

	switch (type) {
	case ADDR_RAW_HOVER_FORCE:
	case ADDR_FILTER_HOVER_FORCE:
	case ADDR_NORM_HOVER_FORCE:
	case ADDR_CALIB_HOVER_FORCE:
	case ADDR_RAW_PRX_FORCE:
	case ADDR_FILTER_PRX_FORCE:
	case ADDR_NORM_PRX_FORCE:
	case ADDR_CALIB_PRX_FORCE:
		size = ((force_len) * 1);
		break;

	case ADDR_RAW_HOVER_SENSE:
	case ADDR_FILTER_HOVER_SENSE:
	case ADDR_NORM_HOVER_SENSE:
	case ADDR_CALIB_HOVER_SENSE:
	case ADDR_RAW_PRX_SENSE:
	case ADDR_FILTER_PRX_SENSE:
	case ADDR_NORM_PRX_SENSE:
	case ADDR_CALIB_PRX_SENSE:
		size = ((1) *sense_len);
		break;

	default:
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;

	}


	ret = ftm4_getOffsetFrame(info, type, &offset);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_OFFSET);
		return (ret | ERROR_GET_OFFSET);
	}


	 *frame = (short *)kmalloc(size *sizeof(short), GFP_KERNEL);
	if (*frame == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_getFrameData(info, offset, size *BYTES_PER_NODE, frame);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_FRAME_DATA);
		return (ret | ERROR_GET_FRAME_DATA);
	}

	fts_err(info, "Frame acquired! \n");
	return size;

}

/*

int getNmsFrame(u16 type, short ***frames, int *size, int keep_first_row, int fs, int n) {
	int i;
	StopWatch global, local;											// structure for computing the time elapsed during the total execution or the single iteration
	int temp;

	 *frames = (short **)kmalloc(n *sizeof(short *), GFP_KERNEL);


	if (*frames == NULL) {
		fts_err(info, " getNmsFrame: ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}


	fs = (1 * 1000 / fs);													//convert the sample frequency[Hz] in ms

	ftm4_startStopWatch(&global);
	for (i = 0; i < n; i++) {

		ftm4_startStopWatch(&local);


		 *size = ftm4_getMSFrame(type, ((*frames) + i), keep_first_row);
		if (*size < OK) {
			fts_err(info, " getNFrame: getFrame failed\n");
			return *size;
		}

		ftm4_stopStopWatch(&local);
		temp = ftm4_elapsedMillisecond(&local);
		fts_err(info, " Iteration %d performed in %d ms... the process wait for %ld ms\n\n", i, temp, (unsigned long)(fs - temp));

		if (temp < fs)
			mdelay((unsigned long)(fs - temp));

	}


	ftm4_stopStopWatch(&global);
	temp = ftm4_elapsedMillisecond(&global);
	fts_err(info, " Global Iteration performed in %d ms \n", temp);
	temp /= n;
	fts_err(info, " Mean Iteration performed in %d ms \n", temp);
	return (1000 / (temp));

} */


int ftm4_getSenseLen(struct fts_ts_info *info)
{
	int ret;
	int sense_len;
	int force_len;

	ret = ftm4_getChannelsLength(info, &sense_len, &force_len);
	return ret;
}

int ftm4_getForceLen(struct fts_ts_info *info)
{
	int ret;
	int sense_len;
	int force_len;

	ret = ftm4_getChannelsLength(info, &sense_len, &force_len);
	return ret;
}

int ftm4_requestFrame(struct fts_ts_info *info, u16 type)
{
	int retry = 0;
	int ret;
	u16 answer;
	char *temp = NULL;

	int event_to_search[1];
	u8 readEvent[FIFO_EVENT_SIZE];

	u8 cmd[3] = { FTS_CMD_REQU_FRAME_DATA, 0x00, 0x00 };							/* B7 is the command for asking frame data */
	event_to_search[0] = (int)EVENTID_FRAME_DATA_READ;


	ftm4_u16ToU8(type, &cmd[1]);

	while (retry < FRAME_DATA_READ_RETRY) {
		temp = ftm4_printHex("Command = ", cmd, 3);
		if (temp != NULL)
			fts_err(info, "%s", temp);
		kfree(temp);
		ret = ftm4_writeFwCmd(info, cmd, 3);										/*send the request to the chip to load in memory the Frame Data */
		if (ret < OK) {
			fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
			return ERROR_I2C_W;
		}
		ret = ftm4_pollForEvent(info, event_to_search, 1, readEvent, TIMEOUT_REQU_COMP_DATA);
		if (ret < OK) {
			fts_err(info, "Event did not Found at %d attemp! \n", retry + 1);
			retry += 1;
		} else {
			retry = 0;
			break;
		}
	}

	if (retry == FRAME_DATA_READ_RETRY) {
		fts_err(info, "ERROR %02X\n", ERROR_TIMEOUT);
		return ERROR_TIMEOUT;
	}

	fts_u8ToU16_le(&readEvent[1], &answer);

	if (answer == type)
		return OK;
	else {
		fts_err(info, "The event found has a different type of Frame data ERROR %02X \n", ERROR_DIFF_COMP_TYPE);
		return ERROR_DIFF_COMP_TYPE;
	}

}


int ftm4_readFrameDataHeader(struct fts_ts_info *info, u16 type, DataHeader *header)
{

	u16 offset = ADDR_FRAMEBUFFER_DATA;
	u16 answer;
	u8 data[FRAME_DATA_HEADER];

	if (ftm4_readCmdU16(info, FTS_CMD_FRAMEBUFFER_R, offset, data, FRAME_DATA_HEADER, DUMMY_FRAMEBUFFER) < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		return ERROR_I2C_R;
	}
	fts_err(info, "Read Data Header done! \n");

	if (data[0] != FRAME_HEADER_SIGNATURE) {
		fts_err(info, "ERROR %02X The Header Signature was wrong!  %02X != %02X \n", ERROR_WRONG_COMP_SIGN, data[0], HEADER_SIGNATURE);
		return ERROR_WRONG_COMP_SIGN;
	}


	fts_u8ToU16_le(&data[1], &answer);


	if (answer != type) {
		fts_err(info, "ERROR %02X\n", ERROR_DIFF_COMP_TYPE);
		return ERROR_DIFF_COMP_TYPE;
	}

	fts_err(info, "Type of Frame data OK! \n");

	header->type = type;
	header->force_node = (int)data[4];
	header->sense_node = (int)data[5];

	return OK;

}

int ftm4_getMSFrame2(struct fts_ts_info *info, u16 type, MutualSenseFrame *frame)
{
	u16 offset = ADDR_FRAMEBUFFER_DATA + FRAME_DATA_HEADER;
	int size, ret;
	frame->node_data = NULL;

	if (!(type == MS_TOUCH_ACTIVE || type == MS_TOUCH_LOW_POWER || type == MS_TOUCH_ULTRA_LOW_POWER || type == MS_KEY)) {
		fts_err(info, "Choose a MS type of frame data ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_requestFrame(info, type);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_REQU_COMP_DATA);
		return (ret | ERROR_REQU_COMP_DATA);
	}

	ret = ftm4_readFrameDataHeader(info, type, &(frame->header));
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return (ret | ERROR_COMP_DATA_HEADER);
	}

	switch (type) {
	case MS_TOUCH_ACTIVE:
	case MS_TOUCH_LOW_POWER:
	case MS_TOUCH_ULTRA_LOW_POWER:
		size = frame->header.force_node * frame->header.sense_node;
		break;
	case MS_KEY:
		if (frame->header.force_node > frame->header.sense_node) /*or use directly the number in the ftsChip */
			size = frame->header.force_node;
		else
			size = frame->header.sense_node;
				frame->header.force_node = 1;
		frame->header.sense_node = size;
		break;

	default:
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}


	frame->node_data = (short *)kmalloc(size * sizeof(short), GFP_KERNEL);
	if (frame->node_data == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_getFrameData(info, offset, size * BYTES_PER_NODE, &(frame->node_data));
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_FRAME_DATA);
		kfree(frame->node_data);
		return (ret | ERROR_GET_FRAME_DATA);
	}
	/* if you want to access one node i, j, you should compute the offset like: offset = i *columns + j = > frame[i, j]	 */

	fts_err(info, "Frame acquired! \n");
	frame->node_data_size = size;
	return size;																		/*return the number of data put inside frame */

}

int ftm4_getSSFrame2(struct fts_ts_info *info, u16 type, SelfSenseFrame *frame)
{
	u16 offset = ADDR_FRAMEBUFFER_DATA + FRAME_DATA_HEADER;
	int size, ret;
	short *temp = NULL;
	frame->force_data = NULL;
	frame->sense_data = NULL;

	if (!(type == SS_TOUCH || type == SS_KEY || type == SS_HOVER || type == SS_PROXIMITY)) {
		fts_err(info, " Choose a SS type of frame data ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = ftm4_requestFrame(info, type);
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_REQU_COMP_DATA);
		return (ret | ERROR_REQU_COMP_DATA);
	}

	ret = ftm4_readFrameDataHeader(info, type, &(frame->header));
	if (ret < 0) {
		fts_err(info, "ERROR %02X\n", ERROR_COMP_DATA_HEADER);
		return (ret | ERROR_COMP_DATA_HEADER);
	}

	switch (type) {
	case SS_TOUCH:
	case SS_HOVER:
	case SS_PROXIMITY:
		size = frame->header.force_node + frame->header.sense_node;
		break;

	default:
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}


	temp = (short *)kmalloc(size * sizeof(short), GFP_KERNEL);
	if (temp == NULL) {
		fts_err(info, "getSSFrame: temp ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	ret = ftm4_getFrameData(info, offset, size * BYTES_PER_NODE, &temp);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ERROR_GET_FRAME_DATA);
		kfree(temp);
		return (ret | ERROR_GET_FRAME_DATA);
	}

	frame->force_data = (short *)kmalloc(frame->header.force_node * sizeof(short), GFP_KERNEL);
	if (frame->force_data == NULL) {
		fts_err(info, "getSSFrame: frame->force_data ERROR %02X\n", ERROR_ALLOC);
		kfree(temp);
		return ERROR_ALLOC;
	}

	memcpy(frame->force_data, temp, frame->header.force_node * sizeof(short));

	frame->sense_data = (short *)kmalloc(frame->header.sense_node * sizeof(short), GFP_KERNEL);
	if (frame->sense_data == NULL) {
		fts_err(info, " getSSFrame: frame->sense_data ERROR %02X\n", ERROR_ALLOC);
		kfree(temp);
		kfree(frame->force_data);
		return ERROR_ALLOC;
	}

	memcpy(frame->sense_data, &temp[frame->header.force_node], frame->header.sense_node * sizeof(short));

	fts_err(info, "Frame acquired! \n");
	kfree(temp);
	return size;														/*return the number of data put inside frame */

}
