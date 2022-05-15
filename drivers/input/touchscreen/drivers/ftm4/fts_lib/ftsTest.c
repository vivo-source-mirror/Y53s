/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *               	FTS API for MP test				 *
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
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"

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

int ftm4_computeAdjHoriz(struct fts_ts_info *info, u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		fts_err(info, "ERROR %02x\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *) kmalloc(size * sizeof (u8), GFP_KERNEL);
	if (*result == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++) {
		for (j = 1; j < column; j++) {
			*(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);
		}
	}

	return OK;

}

int ftm4_computeAdjHorizTotal(struct fts_ts_info *info, u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = row * (column - 1);

	if (column < 2) {
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *) kmalloc(size * sizeof (u16), GFP_KERNEL);
	if (*result == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++) {
		for (j = 1; j < column; j++) {
			*(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);
		}
	}

	return OK;

}

int ftm4_computeAdjVert(struct fts_ts_info *info, u8 *data, int row, int column, u8 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		fts_err(info, "ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u8 *) kmalloc(size * sizeof (u8), GFP_KERNEL);
	if (*result == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++) {
		for (j = 0; j < column; j++) {
			*(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);
		}
	}

	return OK;
}

int ftm4_computeAdjVertTotal(struct fts_ts_info *info, u16 *data, int row, int column, u16 **result)
{
	int i, j;
	int size = (row - 1) * (column);

	if (row < 2) {
		fts_err(info, " ERROR %02X\n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	*result = (u16 *) kmalloc(size * sizeof (u16), GFP_KERNEL);
	if (*result == NULL) {
		fts_err(info, " ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 1; i < row; i++) {
		for (j = 0; j < column; j++) {
			*(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);
		}
	}

	return OK;
}

int ftm4_computeTotal(struct fts_ts_info *info, u8 *data, u8 main, int row, int column, int m, int n, u16 **result)
{
	int i, j;
	int size = (row) * (column);

	*result = (u16 *) kmalloc(size * sizeof (u16), GFP_KERNEL);
	if (*result == NULL) {
		fts_err(info, "ERROR %02X\n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			*(*result + (i * column + j)) = m * main + n * data[i * column + j];
		}
	}

	return OK;
}

int ftm4_checkLimitsMinMax(struct fts_ts_info *info, short *data, int row, int column, int min, int max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min || data[i * column + j] > max) {
				fts_err(info, "Node[%d,%d] = %d exceed limit [%d, %d] \n", i, j, data[i * column + j], min, max);
				count++;
			}
		}
	}

	return count; /*if count is 0 = OK, test completed successfully */
}

int ftm4_checkLimitsGap(struct fts_ts_info *info, short *data, int row, int column, int threshold)
{
	int i, j;
	int min_node;
	int max_node;

	if (row == 0 || column == 0) {
		fts_err(info, " invalid number of rows = %d or columns = %d  ERROR %02x\n", row, column, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	min_node = data[0];
	max_node = data[0];

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min_node) {
				min_node = data[i * column + j];
			} else {
				if (data[i * column + j] > max_node)
					max_node = data[i * column + j];
			}
		}
	}

	if (max_node - min_node > threshold) {
		fts_err(info, " GAP = %d exceed limit  %d \n", max_node - min_node, threshold);
		return ERROR_TEST_CHECK_FAIL;
	} else
		return OK;

}

int ftm4_checkLimitsMap(struct fts_ts_info *info, u8 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
				fts_err(info, " Node[%d,%d] = %d exceed limit [%d, %d] \n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]); count++;
			}
		}
	}

	return count; /*if count is 0 = OK, test completed successfully */
}

int ftm4_checkLimitsMapTotal(struct fts_ts_info *info, u16 *data, int row, int column, int *min, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
				fts_err(info, " Node[%d,%d] = %d exceed limit [%d, %d] \n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count; /*if count is 0 = OK, test completed successfully */
}

int ftm4_checkLimitsMapAdj(struct fts_ts_info *info, u8 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				fts_err(info, " Node[%d,%d] = %d exceed limit > %d \n", i, j, data[i * column + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count; /*if count is 0 = OK, test completed successfully */
}

int ftm4_checkLimitsMapAdjTotal(struct fts_ts_info *info, u16 *data, int row, int column, int *max)
{
	int i, j;
	int count = 0;

	for (i = 0; i < row; i++) {
		for (j = 0; j < column; j++) {
			if (data[i * column + j] > max[i * column + j]) {
				fts_err(info, "Node[%d,%d] = %d exceed limit > %d \n", i, j, data[i * column + j], max[i * column + j]);
				count++;
			}
		}
	}

	return count; /*if count is 0 = OK, test completed successfully */
}

int ftm4_production_test_ito(struct fts_ts_info *info)
{
	int res = OK;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_ERROR_EVENT, EVENT_TYPE_ITO}; /*look for ito event */

	fts_info(info, "ITO Production test is starting...");

	res = ftm4_system_reset(info);
	if (res < 0) {
		fts_info(info, "ftm4_production_test_ito: ERROR %02X", ERROR_PROD_TEST_ITO);
		return (res | ERROR_PROD_TEST_ITO);
	}

	cmd = FTS_CMD_ITO_CHECK;

	fts_info(info, "ITO Check command sent...");
	if (ftm4_writeFwCmd(info, &cmd, 1) < 0) {
		fts_info(info, "ftm4_production_test_ito: ERROR %02X", (ERROR_I2C_W | ERROR_PROD_TEST_ITO));
		return (ERROR_I2C_W | ERROR_PROD_TEST_ITO);
	}

	fts_info(info, "Looking for ITO Event...");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_ITO_TEST_RESULT);
	if (res < 0) {
		fts_info(info, "ftm4_production_test_ito: ITO Production test failed... ERROR %02X", ERROR_PROD_TEST_ITO);
		return (res | ERROR_PROD_TEST_ITO);
	}
	fts_info(info, "ITO Production testes finished!... %02X %02X %02X %02X %02X %02X %02X %02X", readData[0], readData[1], readData[2], readData[3], readData[4], readData[5], readData[6], readData[7]);

	if (readData[2] != 0x00 || readData[3] != 0x00) {
		fts_info(info, "ITO Production testes finished!...FAILED  ERROR %02X", (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_ITO));
		res = (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_ITO);
	} else {
		fts_info(info, "ITO Production test finished!...OK");
		res = OK;
	}

	res |= ftm4_system_reset(info);
	if (res < 0) {
		fts_info(info, "ftm4_production_test_ito: ERROR %02X", ERROR_PROD_TEST_ITO);
		res = (res | ERROR_PROD_TEST_ITO);
	}
	return res;
}

int ftm4_production_test_initialization(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_FULL_INITIALIZATION};

	fts_err(info, " INITIALIZATION Production test is starting...\n");

	res = ftm4_system_reset(info);
	if (res < 0) {
		fts_err(info, " ERROR %02X \n", ERROR_PROD_TEST_INITIALIZATION);
		return (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	fts_err(info, " INITIALIZATION command sent... \n");
	cmd = FTS_CMD_FULL_INITIALIZATION;
	if (ftm4_writeFwCmd(info, &cmd, 1) < 0) {
		fts_err(info, " ERROR %02X \n", (ERROR_I2C_W | ERROR_PROD_TEST_INITIALIZATION));
		return (ERROR_I2C_W | ERROR_PROD_TEST_INITIALIZATION);
	}


	fts_err(info, " Looking for INITIALIZATION Event... \n");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_err(info, " INITIALIZATION Production test failed... ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
		return (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	if (readData[2] != 0x00) {
		fts_err(info, " INITIALIZATION Production testes finished!.................FAILED  ERROR %02X\n", (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_INITIALIZATION));
		res = (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_INITIALIZATION);
	} else {
		fts_err(info, " INITIALIZATION Production test.................OK\n");
		res = OK;
	}


	fts_err(info, " Refresh Chip Info...\n");
	res |= fts_readChipInfo(info, 1);				/*need to update the chipInfo in order to refresh the tuning_versione */

	if (res < 0) {
			fts_err(info, " read chip info ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
			res = (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	return res;

}

int ftm4_ms_compensation_tuning(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_MS_TUNING_CMPL};


	fts_err(info, " MS INITIALIZATION command sent... \n");
	cmd = FTS_CMD_MS_COMP_TUNING;
	if (ftm4_writeFwCmd(info, &cmd, 1) < 0) {
		fts_err(info, "ERROR %02X \n", (ERROR_I2C_W | ERROR_MS_TUNING));
		return (ERROR_I2C_W | ERROR_MS_TUNING);
	}


	fts_err(info, " Looking for MS INITIALIZATION Event... \n");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_err(info, "MS INITIALIZATION Production test failed... ERROR %02X\n", ERROR_MS_TUNING);
		return (res | ERROR_MS_TUNING);
	}

	if (readData[2] != 0x00 || readData[3] != 0x00) {
		fts_err(info, " MS INITIALIZATION Production test finished!.................FAILED  ERROR %02X\n", ERROR_MS_TUNING);
		res = ERROR_MS_TUNING;
	} else {
		fts_err(info, " MS INITIALIZATION Production test finished!.................OK\n");
		res = OK;
	}

	return res;
}

int ftm4_ss_compensation_tuning(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_SS_TUNING_CMPL};

	fts_err(info, " SS INITIALIZATION command sent... \n");
	cmd = FTS_CMD_SS_COMP_TUNING;
	if (ftm4_writeFwCmd(info, &cmd, 1) < 0) {
		fts_err(info, "ERROR %02X \n", (ERROR_I2C_W | ERROR_SS_TUNING));
		return (ERROR_I2C_W | ERROR_SS_TUNING);
	}


	fts_err(info, " Looking for SS INITIALIZATION Event... \n");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_err(info, " SS INITIALIZATION Production test failed... ERROR %02X\n", ERROR_SS_TUNING);
		return (res | ERROR_SS_TUNING);
	}

	if (readData[2] != 0x00 || readData[3] != 0x00) {
		fts_err(info, " SS INITIALIZATION Production test finished!.................FAILED  ERROR %02X\n", ERROR_SS_TUNING);
		res = ERROR_SS_TUNING;
	} else {
		fts_err(info, " SS INITIALIZATION Production test finished!.................OK\n");
		res = OK;
	}

	return res;
}

int ftm4_lp_timer_calibration(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_LPTIMER_TUNING_CMPL};

	fts_err(info, " LP TIMER CALIBRATION command sent... \n");
	cmd = FTS_CMD_LP_TIMER_CALIB;
	if (ftm4_writeFwCmd(info, &cmd, 1) < 0) {
		fts_err(info, "ERROR %02X \n", (ERROR_I2C_W | ERROR_LP_TIMER_TUNING));
		return (ERROR_I2C_W | ERROR_LP_TIMER_TUNING);
	}


	fts_err(info, " Looking for LP TIMER CALIBRATION Event... \n");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_err(info, "LP TIMER CALIBRATION Production test failed... ERROR %02X\n", ERROR_LP_TIMER_TUNING);
		return (res | ERROR_LP_TIMER_TUNING);
	}

	if (readData[2] != 0x00 || readData[3] != 0x01) {
		fts_err(info, " LP TIMER CALIBRATION Production test finished!.................FAILED  ERROR %02X\n", ERROR_LP_TIMER_TUNING);
		res = ERROR_LP_TIMER_TUNING;
	} else {
		fts_err(info, " LP TIMER CALIBRATION Production test finished!.................OK\n");
		res = OK;
	}

	return res;
}

int ftm4_save_cx_tuning(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_COMP_DATA_SAVED};

	fts_info(info, "SAVE CX command sent...");
	cmd = FTS_CMD_SAVE_CX_TUNING;
	if (ftm4_writeCmd(info, &cmd, 1) < 0) {
		fts_info(info, "2: ERROR %02X", (ERROR_I2C_W | ERROR_SAVE_CX_TUNING));
		return (ERROR_I2C_W | ERROR_SAVE_CX_TUNING);
	}


	fts_info(info, "Looking for SAVE CX Event...");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_info(info, "SAVE CX failed... ERROR %02X", ERROR_SAVE_CX_TUNING);
		return (res | ERROR_SAVE_CX_TUNING);
	}


	if (readData[2] != 0x00 || readData[3] != 0x00) {
		fts_info(info, "SAVE CX finished!.................FAILED  ERROR %02X", ERROR_SAVE_CX_TUNING);
		res = ERROR_SAVE_CX_TUNING;
	} else {
		fts_info(info, "SAVE CX finished!.................OK");
		res = OK;
	}

	return res;
}

int __fts_save_mp_flag(struct fts_ts_info *info)
{
	int res;
	u8 cmd;
	u8 readData[FIFO_EVENT_SIZE];
	int eventToSearch[2] = {EVENTID_STATUS_UPDATE, EVENT_TYPE_COMP_DATA_SAVED};

	fts_info(info, "SAVE CX command sent...");
	cmd = FTS_CMD_SAVE_MP_FLAG;
	if (ftm4_writeCmd(info, &cmd, 1) < 0) {
		fts_info(info, "ERROR %02X \n", (ERROR_I2C_W | ERROR_SAVE_CX_TUNING));
		return (ERROR_I2C_W | ERROR_SAVE_CX_TUNING);
	}


	fts_info(info, "Looking for SAVE CX Event...");
	res = ftm4_pollForEvent(info, eventToSearch, 2, readData, TIMEOUT_INITIALIZATION_TEST_RESULT);
	if (res < 0) {
		fts_info(info, "SAVE CX failed... ERROR %02X", ERROR_SAVE_CX_TUNING);
		return (res | ERROR_SAVE_CX_TUNING);
	}


	if (readData[2] != 0x01 || readData[3] != 0x00) {
		fts_info(info, "SAVE CX finished!.................FAILED  ERROR %02X", ERROR_SAVE_CX_TUNING);
		res = ERROR_SAVE_CX_TUNING;
	} else {
		fts_info(info, "SAVE CX finished!.................OK");
		res = OK;
	}

	return res;
}


int ftm4_production_test_splitted_initialization(struct fts_ts_info *info, int saveToFlash)
{
	int res;

	fts_err(info, " Splitted Initialization test is starting...\n");
	res = ftm4_system_reset(info);
	if (res < 0) {
		fts_err(info, " ERROR %02X \n", ERROR_PROD_TEST_INITIALIZATION);
		return (res | ERROR_PROD_TEST_INITIALIZATION);
	}

	fts_err(info, " MS INITIALIZATION TEST: \n");
	res = ftm4_ms_compensation_tuning(info);
	if (res < 0) {
		fts_err(info, " MS INITIALIZATION TEST FAILED! ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
		return (res | ERROR_PROD_TEST_INITIALIZATION);
	} else {
		fts_err(info, " MS INITIALIZATION TEST OK!\n");

		fts_err(info, " \n");

		fts_err(info, " SS INITIALIZATION TEST: \n");
		res = ftm4_ss_compensation_tuning(info);
		if (res < 0) {
			fts_err(info, " SS INITIALIZATION TEST FAILED! ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
			return (res | ERROR_PROD_TEST_INITIALIZATION);
		} else {
			fts_err(info, " SS INITIALIZATION TEST OK!\n");

			fts_err(info, " \n");

			fts_err(info, " LP INITIALIZATION TEST: \n");
			res = ftm4_lp_timer_calibration(info);
			if (res < 0) {
				fts_err(info, " LP INITIALIZATION TEST FAILED! ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
				return (res | ERROR_PROD_TEST_INITIALIZATION);
			} else {
				fts_err(info, " LP INITIALIZATION TEST OK!\n");
				if (saveToFlash) {

					fts_err(info, " \n");

					fts_err(info, " SAVE CX TEST: \n");
					res = ftm4_save_cx_tuning(info);
					if (res < 0) {
						fts_err(info, " SAVE CX TEST FAILED! ERROR %02X\n", res);
						return (res | ERROR_PROD_TEST_INITIALIZATION);
					} else {
						fts_err(info, " SAVE CX TEST OK!\n");
					}
				}
		fts_err(info, " Refresh Chip Info...\n");
		res |= fts_readChipInfo(info, 1);
			if (res < 0) {
					fts_err(info, " read chip info ERROR %02X\n", ERROR_PROD_TEST_INITIALIZATION);
					res = (res | ERROR_PROD_TEST_INITIALIZATION);
			} else
					fts_err(info, " Splitted Initialization test finished!.................OK\n");
				return res;
			}
		}
	}

}
#define FRE_NUM_ADDR	0x0121
extern int ftm4_getMSFrame3(struct fts_ts_info *info, u16 type, MutualSenseFrame *frame, int keep_first_row);
static int strength_compare(struct fts_ts_info *info, int threshold_value)
{
	int i,j;
	int ret = 0;
	int index;
	int fre_num;
	//u8 cmd[3] ={0xB2,0x01,0x21};
	u8 skip_freq[3]={0xC0,0x01,0x00};
	int event_search[3]={0x16,0x0E,0x00};
	u8 readData[FIFO_EVENT_SIZE];
	MutualSenseFrame frame;
	u8 *data;

	if(threshold_value <= 0){
		fts_info(info, "the threshold value is not right:%d", threshold_value);
		return -1;
	}
 	data = (u8*)kmalloc(sizeof(u8), GFP_KERNEL);
	if (data == NULL) {
		fts_info(info, "kmalloc fail");
		return -1;
	}
	ftm4_disableInterrupt(info);
	ftm4_senseOn(info);
 	ret = ftm4_readB2(info, FRE_NUM_ADDR, data, 1);
	if(ret<OK){
		fts_info(info, "get the freq num failed");
		goto END;
	}
		
	fre_num =(int)((*data) & 0x03)+1;

	fts_info(info, "the freq_num is:%d threshold is:%d", fre_num, threshold_value);

	for(index=0; index < fre_num; index++){
		skip_freq[2] = index & 0xFF;
		fts_info(info, "set %d freq scan", index+1);
		ret = ftm4_writeCmd(info, skip_freq,3);
		event_search[2] = skip_freq[2];
		ret = ftm4_pollForEvent(info, event_search, 3, readData,GENERAL_TIMEOUT);
		if(ret < 0){
			fts_info(info, "can not found the right event %08X", ret);
			break;
		}
		for(i=0;i<1;i++){
			mdelay(WAIT_FOR_FRESH_FRAMES);
			ftm4_senseOff(info);
			ret = ftm4_getMSFrame3(info, ADDR_NORM_TOUCH, &frame, 0);
			if(ret<OK){
				fts_info(info, "could not get the frame! ERROR %08X", ret);
				break;
			}
			ftm4_print_frame_short(info, "MS Strength frame =", ftm4_array1dTo2d_short(frame.node_data, frame.node_data_size, frame.header.sense_node), frame.header.force_node, frame.header.sense_node);
			for(j=0;j<frame.node_data_size;j++){
				if(frame.node_data[j] > threshold_value || frame.node_data[j] < -threshold_value){
					fts_info(info, "the strength value is bigger than the threshold_value: %d", frame.node_data[j]);
					ret =-1;
					break;
				}		
			}
			if(ret < OK){
				fts_info(info, "strength compare have some error:ERROR %08X", ret);
				break;
			}
			ftm4_senseOn(info);
		}
		if(ret < OK){
			fts_info(info, "strength compare have some error stop it ERROR %08X", ret);
			break;
		}
	}

END:
	kfree(data);
	ftm4_cleanUp(info, 1);
	return ret;
}

int ftm4_production_test_main(struct fts_ts_info *info, char *pathThresholds, int stop_on_fail, int saveInit, TestToDo *todo,
	u32 signature, enum vts_sensor_test_result *result)
{
	int res, ret;
	int delta_test_result = 0;
	int i = 0;
	*result = VTS_SENSOR_TEST_SUCCESS;

	fts_info(info, "MAIN Production test is starting...");

	fts_info(info, "delta test:");
	ret = strength_compare(info, 150);
	if (ret < 0) {
		fts_info(info, "strength test fail");
		delta_test_result = -1;
		*result |= VTS_SENSOR_TEST_DELTA_FAILED;
	}

	fts_info(info, "ITO TEST");
	res = ftm4_production_test_ito(info);
	if (res < 0) {
		fts_info(info, "Error during ITO TEST! ERROR %08X", res);
		*result |= VTS_SENSOR_TEST_ITO_FAILED;
		goto END; /*in case of ITO TEST failure is no sense keep going */
	}

	if (saveInit == 1) {
		for (i = 0; i < 5; i++) {
		res = ftm4_production_test_initialization(info);
			if (res >= 0) {
				fts_info(info, "initialization pass");
				break;
			}
			fts_info(info, "initialization fail,retry %d times", i);
			mdelay(10);
		}
		if (i >= 5) {
			*result |= VTS_SENSOR_TEST_INIT_FAILED;
			if (stop_on_fail)
				goto END;
		}
	} /*else
	fts_info(info, "INITIALIZATION TEST :................. SKIPPED"); */

	if (saveInit == 1) {
		ret = ftm4_cleanUp(info, 0);
		if (ret < 0) {
			res |= ret;
			if (stop_on_fail)
				goto END;
		}
	}

  	fts_info(info, "PRODUCTION DATA TEST:");
	ret = ftm4_production_test_data(info, pathThresholds, stop_on_fail, todo);
	if (ret < 0) {
		*result |= VTS_SENSOR_TEST_DATA_FAILED;
		fts_info(info, "Error during PRODUCTION DATA TEST! ERROR %08X", ret);
	}

	res |= ret;

	if (ret == OK && saveInit == 1) {
		ret = fts_save_mp_flag(info, signature);
		res |= ret;
		ret = fts_readChipInfo(info, 1);	/* need to update the MP Flag */
		if (ret < OK) {
			fts_info(info, "ftm4_production_test_main: read chip info ERROR %08X", ret);
		}
		res |= ret;
	}

END:
	return OK;
}

int ftm4_production_test_ms_raw(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret, count_fail = 0;
	MutualSenseFrame msRawFrame;


	int *thresholds = NULL;
	int trows, tcolumns;

	/******************************** Mutual Sense Test *******************************/
	fts_info(info, "MS RAW DATA TEST is starting...");
	if (todo->MutualRaw == 1 || todo->MutualRawGap == 1) {
		ret = ftm4_getMSFrame2(info, MS_TOUCH_ACTIVE, &msRawFrame);
		if (ret < 0) {
			fts_err(info, "ERROR %02X \n", ERROR_PROD_TEST_DATA);
			return (ret | ERROR_PROD_TEST_DATA);
		}

		fts_err(info, " MS RAW MIN MAX TEST:  \n");
		if (todo->MutualRaw == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_RAW_MIN_MAX, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				fts_err(info, " MS_RAW_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}


			ret = ftm4_checkLimitsMinMax(info, msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds[0], thresholds[1]);
			if (ret != OK) {
				fts_err(info, "MS RAW failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " MS RAW MIN MAX TEST:.................FAIL \n\n");
		count_fail += 1;
				if (stop_on_fail == 1)
	goto ERROR;
			} else
		fts_err(info, " MS RAW MIN MAX TEST:.................OK  \n");
			kfree(thresholds);
		thresholds = NULL;
		} else
			fts_err(info, " MS RAW MIN MAX TEST:.................SKIPPED  \n");

		fts_err(info, " \n");
		fts_err(info, " MS RAW GAP TEST: \n");
		if (todo->MutualRawGap == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_RAW_GAP, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				fts_err(info, "MS_RAW_GAP failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsGap(info, msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds[0]);
			if (ret != OK) {
				fts_err(info, "MS RAW failed... ERROR = %02X \n", ret);
		count_fail += 1;
				if (stop_on_fail == 1)
					goto ERROR;

			} else
				fts_err(info, " MS RAW GAP TEST:.................OK \n\n");
			kfree(thresholds);
		thresholds = NULL;
		} else
			fts_err(info, " MS RAW GAP TEST:.................SKIPPED  \n");

	} else
		fts_err(info, " MS RAW FRAME TEST:.................SKIPPED  \n");

	fts_err(info, " \n");
	fts_err(info, " MS KEY RAW TEST:\n");
	if (todo->MutualKeyRaw == 1) {
		ret = ftm4_production_test_ms_key_raw(info, path_limits);
		if (ret < 0) {
			fts_err(info, " ERROR = %02X \n", ret);
			count_fail += 1;
		if (count_fail == 1) {
		fts_err(info, " MS RAW DATA TEST:.................FAIL fails_count = %d\n\n", count_fail);
		goto ERROR_LIMITS;
		}
		}
	} else
		fts_err(info, " MS KEY RAW TEST:.................SKIPPED  \n");


ERROR:
	fts_err(info, " \n");
	ftm4_print_frame_short(info, "MS Raw frame =", ftm4_array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node), msRawFrame.header.force_node, msRawFrame.header.sense_node);
	if (count_fail == 0) {
		kfree(msRawFrame.node_data);
		msRawFrame.node_data = NULL;
		fts_err(info, " MS RAW DATA TEST finished!.................OK\n");
		return OK;
	} else {

	//ftm4_print_frame_short(info, "MS Raw frame =", ftm4_array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node), msRawFrame.header.force_node, msRawFrame.header.sense_node);

	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	fts_err(info, " MS RAW DATA TEST:.................FAIL fails_count = %d\n\n", count_fail);
	return (ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL);
   }


ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;
}

int ftm4_production_test_ms_key_raw(struct fts_ts_info *info, char *path_limits)
{

	int ret;
	MutualSenseFrame msRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/******************************** Mutual Sense Test *******************************/
	fts_err(info, " MS KEY RAW DATA TEST is starting...\n");

	ret = ftm4_getMSFrame2(info, MS_KEY, &msRawFrame);
	if (ret < 0) {
		fts_err(info, "getMSKeyFrame failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
		return (ret | ERROR_PROD_TEST_DATA);
	}

	ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_RAW_MIN_MAX, &thresholds, &trows, &tcolumns);
	if (ret < 0 || (trows != 1 || tcolumns != 2)) {
		fts_err(info, "MS_KEY_RAW_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
		ret |= ERROR_PROD_TEST_DATA;
	goto ERROR_LIMITS;
	}

	ret = ftm4_checkLimitsMinMax(info, msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds[0], thresholds[1]);
	if (ret != OK) {
		fts_err(info, "MS KEY RAW failed... ERROR COUNT = %d \n", ret);
		goto ERROR;
	} else
		fts_err(info, " MS KEY RAW TEST:.................OK \n\n");

	kfree(thresholds);
	thresholds = NULL;

	kfree(msRawFrame.node_data);
	msRawFrame.node_data = NULL;
	return OK;

ERROR:
	ftm4_print_frame_short(info, "MS Key Raw frame =", ftm4_array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node), msRawFrame.header.force_node, msRawFrame.header.sense_node);
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	fts_err(info, " MS KEY RAW TEST:.................FAIL \n\n");
	return (ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL);

ERROR_LIMITS:
	if (msRawFrame.node_data != NULL)
		kfree(msRawFrame.node_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;

}

int ftm4_production_test_ms_cx(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{

	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;

	u8 *adjhor = NULL;

	u8 *adjvert = NULL;

	u16 container;
	u16 *total_cx = NULL;
	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;


	/*MS CX TEST */
	fts_err(info, " \n");
	fts_err(info, " MS CX Testes are starting... \n");

	ret = ftm4_readMutualSenseCompensationData(info, MS_TOUCH_LOW_POWER, &msCompData); /*read MS compensation data */
	if (ret < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_PROD_TEST_DATA);
		return (ret | ERROR_PROD_TEST_DATA);
	}

	fts_err(info, " MS CX1 TEST: \n");
	if (todo->MutualCx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_CX1_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, "ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		container = (u16) msCompData.cx1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " ERROR COUNT = %d \n", ret);
			fts_err(info, " MS CX1 TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS CX1 TEST:.................OK \n\n");
	} else
		fts_err(info, " MS CX1 TEST:.................SKIPPED \n\n");

	kfree(thresholds);
	thresholds = NULL;

	fts_err(info, " MS CX2 MIN MAX TEST: \n");
	if (todo->MutualCx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_CX2_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load min thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
			fts_err(info, " MS_CX2_MAP_MIN failed... ERROR %02X  Cx Row = %d Cx Columns =%d \n", ERROR_PROD_TEST_DATA, msCompData.header.force_node, msCompData.header.sense_node);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_CX2_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load max thresholds */
		if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
			fts_err(info, " MS_CX2_MAP_MAX failed... ERROR %02X Cx Row = %d Cx Columns =%d\n", ERROR_PROD_TEST_DATA, msCompData.header.force_node, msCompData.header.sense_node);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, thresholds_min, thresholds_max); /*check the limits */
		if (ret != OK) {
			fts_err(info, " MS CX2 MIN MAX failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS CX2 MIN MAX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS CX2 MIN MAX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " MS CX2 MIN MAX TEST:.................SKIPPED \n\n");

	fts_err(info, " MS CX2 ADJ TEST: \n");
	if (todo->MutualCx2Adj == 1) {
		/*MS CX2 ADJ HORIZ */
		fts_err(info, " MS CX2 ADJ HORIZ TEST: \n");

		ret = ftm4_computeAdjHoriz(info, msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			fts_err(info, "failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " MS CX2 ADJ HORIZ computed! \n");

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_CX2_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node - 1)) {
			fts_err(info, "MS_CX2_ADJH_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjhor, msCompData.header.force_node, msCompData.header.sense_node - 1, thresholds_max);
		if (ret != OK) {
			fts_err(info, "CX2 ADJH failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS CX2 ADJ HORIZ TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS CX2 ADJ HORIZ TEST:.................OK \n\n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjhor);
	adjhor = NULL;

		/*MS CX2 ADJ VERT */
		fts_err(info, " MS CX2 ADJ VERT TEST: \n");

		ret = ftm4_computeAdjVert(info, msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjvert);
		if (ret < 0) {
			fts_err(info, " ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " MS CX2 ADJ VERT computed! \n");

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_CX2_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);
		if (ret < 0 || (trows != msCompData.header.force_node - 1 || tcolumns != msCompData.header.sense_node)) {
			fts_err(info, "MS_CX2_ADJV_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjvert, msCompData.header.force_node - 1, msCompData.header.sense_node - 1, thresholds_max);
		if (ret != OK) {
			fts_err(info, " CX2 ADJV failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS CX2 ADJ HORIZ TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS CX2 ADJ VERT TEST:.................OK \n\n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjvert);
	adjvert = NULL;
	} else
		fts_err(info, " MS CX2 ADJ TEST:.................SKIPPED \n\n");

	/*START OF TOTAL CHECK */
	fts_err(info, " MS TOTAL CX TEST: \n");

	if (todo->MutualCxTotal == 1 || todo->MutualCxTotalAdj == 1) {
		ret = ftm4_computeTotal(info, msCompData.node_data, msCompData.cx1, msCompData.header.force_node, msCompData.header.sense_node, CX1_WEIGHT, CX2_WEIGHT, &total_cx);
		if (ret < 0) {
			fts_err(info, " ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " MS TOTAL CX MIN MAX TEST: \n");
		if (todo->MutualCxTotal == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_TOTAL_CX_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load min thresholds */
			fts_info(info, "trows:%d force_node:%d tcolumns:%d sense_node:%d", trows, msCompData.header.force_node, tcolumns, msCompData.header.sense_node);
			if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
				fts_err(info, " MS_TOTAL_CX_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_TOTAL_CX_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load max thresholds */
			if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
				fts_err(info, "MS_TOTAL_CX_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapTotal(info, total_cx, msCompData.header.force_node, msCompData.header.sense_node, thresholds_min, thresholds_max); /*check the limits */
			ftm4_print_frame_short(info, "MS TOTAL CX =", ftm4_array1dTo2d_short(total_cx, msCompData.header.force_node * msCompData.header.sense_node, msCompData.header.sense_node), msCompData.header.force_node, msCompData.header.sense_node);
			if (ret != OK) {
				fts_err(info, " MS TOTAL CX TEST failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " MS TOTAL CX MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
	goto ERROR;
			} else
				fts_err(info, " MS TOTAL CX MIN MAX TEST:.................OK \n\n");

			kfree(thresholds_min);
		thresholds_min = NULL;
			kfree(thresholds_max);
			thresholds_max = NULL;
		} else
			fts_err(info, " MS TOTAL CX MIN MAX TEST:.................SKIPPED \n\n");


		fts_err(info, " MS TOTAL CX ADJ TEST: \n");
		if (todo->MutualCxTotalAdj == 1) {
			/*MS TOTAL CX ADJ HORIZ */
			fts_err(info, " MS TOTAL CX ADJ HORIZ TEST: \n");

			/*thresholds_max = NULL; */
			ret = ftm4_computeAdjHorizTotal(info, total_cx, msCompData.header.force_node, msCompData.header.sense_node, &total_adjhor);
			if (ret < 0) {
				fts_err(info, "failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}
			fts_err(info, " MS TOTAL CX ADJ HORIZ computed! \n");

			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_TOTAL_CX_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);
			if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node - 1)) {
				fts_err(info, " MS_TOTAL_CX_ADJH_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjhor, msCompData.header.force_node, msCompData.header.sense_node - 1, thresholds_max);
			ftm4_print_frame_short(info, "MS TOTAL CX ADJ HORIZ =", ftm4_array1dTo2d_short(total_adjhor, msCompData.header.force_node * (msCompData.header.sense_node - 1), msCompData.header.sense_node - 1), msCompData.header.force_node, msCompData.header.sense_node - 1);
			if (ret != OK) {
				fts_err(info, " MS TOTAL CX ADJH failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " MS TOTAL CX ADJ HORIZ TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
	goto ERROR;
			} else
				fts_err(info, " MS TOTAL CX ADJ HORIZ TEST:.................OK \n\n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjhor);
		total_adjhor = NULL;

			/*MS TOTAL CX ADJ VERT */
			fts_err(info, " MS TOTAL CX ADJ VERT TEST: \n");

			ret = ftm4_computeAdjVertTotal(info, total_cx, msCompData.header.force_node, msCompData.header.sense_node, &total_adjvert);
			if (ret < 0) {
				fts_err(info, "ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}
			fts_err(info, " MS TOTAL CX ADJ VERT computed! \n");

			ret = ftm4_parseProductionTestLimits(info, path_limits, MS_TOTAL_CX_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);
			if (ret < 0 || (trows != msCompData.header.force_node - 1 || tcolumns != msCompData.header.sense_node)) {
				fts_err(info, "MS_TOTAL_CX_ADJV_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjvert, msCompData.header.force_node - 1, msCompData.header.sense_node - 1, thresholds_max);
			ftm4_print_frame_short(info, "MS TOTAL CX ADJ VERT =", ftm4_array1dTo2d_short(total_adjvert, (msCompData.header.force_node - 1) * (msCompData.header.sense_node - 1), msCompData.header.sense_node - 1), msCompData.header.force_node - 1, msCompData.header.sense_node - 1);
			if (ret != OK) {
				fts_err(info, " MS TOTAL CX ADJV failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " MS TOTAL CX ADJ VERT TEST:.................FAIL \n");
				count_fail += 1;
				if (stop_on_fail)
	goto ERROR;
			} else
				fts_err(info, " MS TOTAL CX ADJ VERT TEST:.................OK \n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjvert);
		total_adjvert = NULL;
		} else
			fts_err(info, " MS TOTAL CX ADJ TEST:.................SKIPPED \n");

		kfree(total_cx);
	total_cx = NULL;
	} else
		fts_err(info, " MS TOTAL CX TEST:.................SKIPPED \n");



	if ((todo->MutualKeyCx1 | todo->MutualKeyCx2 | todo->MutualKeyCxTotal) == 1) {
		ret = ftm4_production_test_ms_key_cx(info, path_limits, stop_on_fail, todo);
		if (ret < 0) {
			count_fail += 1;
			fts_err(info, " ERROR = %02X \n", ret);
			fts_err(info, " MS CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
			return ret;
		}
	} else
		fts_err(info, " MS KEY CX TEST:.................SKIPPED \n");

ERROR:
	fts_err(info, " \n");
	if (count_fail == 0) {
		fts_err(info, " MS CX testes finished!.................OK\n");
	kfree(msCompData.node_data);
	msCompData.node_data = NULL;
		return OK;
	} else {
		ftm4_print_frame_u8(info, "MS Init Data (Cx2) =", ftm4_array1dTo2d_u8(msCompData.node_data, msCompData.node_data_size, msCompData.header.sense_node), msCompData.header.force_node, msCompData.header.sense_node);
		fts_err(info, " MS CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (total_cx != NULL)
		kfree(total_cx);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	return (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA);
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (total_cx != NULL)
		kfree(total_cx);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	return ret;

}

int ftm4_production_test_ms_key_cx(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{

	int ret;
	int count_fail = 0;
	int num_keys = 0;

	int *thresholds = NULL;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;
	int trows, tcolumns;

	MutualSenseData msCompData;


	u16 container;
	u16 *total_cx = NULL;


	/*MS CX TEST */
	fts_err(info, " MS KEY CX Testes are starting... \n");

	ret = ftm4_readMutualSenseCompensationData(info, MS_KEY, &msCompData); /*read MS compensation data */
	if (ret < 0) {
		fts_err(info, "ERROR %02X \n", ERROR_PROD_TEST_DATA);
		return (ret | ERROR_PROD_TEST_DATA);
	}

	if (msCompData.header.force_node > msCompData.header.sense_node) /*the meaningful data are only in the first row, the other rows are only a copy of the first one */
		num_keys = msCompData.header.force_node;
	else
		num_keys = msCompData.header.sense_node;

	fts_err(info, " MS KEY CX1 TEST: \n");
	if (todo->MutualKeyCx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_CX1_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, "MS_KEY_CX1_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		container = (u16) msCompData.cx1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " MS CX1 failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS KEY CX1 TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS KEY CX1 TEST:.................OK \n\n");
	} else
		fts_err(info, " MS KEY CX1 TEST:.................SKIPPED \n\n");

	kfree(thresholds);
	thresholds = NULL;

	fts_err(info, " MS KEY CX2 TEST: \n");
	if (todo->MutualKeyCx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_CX2_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != num_keys)) {
			fts_err(info, " MS_KEY_CX2_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_CX2_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != num_keys)) {
			fts_err(info, " MS_KEY_CX2_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, msCompData.node_data, 1, num_keys, thresholds_min, thresholds_max); /*check the limits */
		if (ret != OK) {
			fts_err(info, " MS KEY CX2 failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS KEY CX2 TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS KEY CX2 TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " MS CX2 TEST:.................SKIPPED \n\n");

	/*START OF TOTAL CHECK */
	fts_err(info, " MS KEY TOTAL CX TEST: \n");

	if (todo->MutualKeyCxTotal == 1) {
		ret = ftm4_computeTotal(info, msCompData.node_data, msCompData.cx1, 1, num_keys, CX1_WEIGHT, CX2_WEIGHT, &total_cx);
		if (ret < 0) {
			fts_err(info, " failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_TOTAL_CX_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != num_keys)) {
			fts_err(info, " MS_KEY_TOTAL_CX_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, MS_KEY_TOTAL_CX_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != num_keys)) {
			fts_err(info, " MS_KEY_TOTAL_CX_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapTotal(info, total_cx, 1, num_keys, thresholds_min, thresholds_max); /*check the limits */
		if (ret != OK) {
			fts_err(info, " MS TOTAL KEY CX TEST failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " MS KEY TOTAL CX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " MS KEY TOTAL CX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;

		kfree(total_cx);
	total_cx = NULL;
	} else
		fts_err(info, " MS KEY TOTAL CX TEST:.................SKIPPED \n");


ERROR:
	fts_err(info, " \n");
	if (count_fail == 0) {
		fts_err(info, " MS KEY CX testes finished!.................OK\n");
		kfree(msCompData.node_data);
		msCompData.node_data = NULL;
		return OK;
	} else {
		ftm4_print_frame_u8(info, "MS Key Init Data (Cx2) =", ftm4_array1dTo2d_u8(msCompData.node_data, msCompData.node_data_size, msCompData.header.sense_node), 1, msCompData.header.sense_node);
		fts_err(info, " MS Key CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	if (total_cx != NULL)
		kfree(total_cx);
	return (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA);
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (msCompData.node_data != NULL)
		kfree(msCompData.node_data);
	if (total_cx != NULL)
		kfree(total_cx);
	return ret;

}

int ftm4_production_test_ss_raw(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int ret;
	int count_fail = 0;
	int rows, columns;

	/*short *ssRawFrame = NULL; */
	SelfSenseFrame ssRawFrame;

	int *thresholds = NULL;
	int trows, tcolumns;

	/*MS SS TEST */
	fts_err(info, " \n");
	fts_err(info, " SS RAW Testes are starting... \n");

	/******************************** Self Sense Test *******************************/

	fts_err(info, " Getting SS Frame... \n");
	ret = ftm4_getSSFrame2(info, SS_TOUCH, &ssRawFrame);
		if (ret < 0) {
			fts_err(info, " getSSFrame failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			return (ret | ERROR_PROD_TEST_DATA);
		}

	/*SS RAW (PROXIMITY) FORCE TEST */
	fts_err(info, " SS RAW (PROXIMITY) FORCE TEST:  \n");



	if (todo->SelfForceRaw == 1 || todo->SelfForceRawGap == 1) {

		columns = 1; /*there are no data for the sense channels due to the fact that the force frame is analized */
		rows = ssRawFrame.header.force_node;

		fts_err(info, " SS RAW (PROXIMITY) FORCE MIN MAX TEST:  \n");
		if (todo->SelfForceRaw == 1) {

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_RAW_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				fts_err(info, " SS_RAW_FORCE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				/*return (ret | ERROR_PROD_TEST_DATA); */
		ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMinMax(info, ssRawFrame.force_data, rows, columns, thresholds[0], thresholds[1]);
			if (ret != OK) {
				fts_err(info, " SS RAW (PROXIMITY) FORCE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				ftm4_print_frame_short(info, "SS Raw force frame =", ftm4_array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
			 		goto ERROR_LIMITS;
				}
			} else {
				fts_err(info, " SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................OK \n\n");
				ftm4_print_frame_short(info, "SS Raw force frame =", ftm4_array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
			}
			kfree(thresholds);
		thresholds = NULL;
		} else
			fts_err(info, " SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................SKIPPED \n\n");

		fts_err(info, " \n");
		fts_err(info, " SS RAW (PROXIMITY) FORCE GAP TEST:  \n");
		if (todo->SelfForceRawGap == 1) {

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_RAW_FORCE_GAP, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				fts_err(info, " SS_RAW_FORCE_GAP failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsGap(info, ssRawFrame.force_data, rows, columns, thresholds[0]);
			if (ret != OK) {
				fts_err(info, " SS RAW (PROXIMITY) FORCE GAP failed... ERROR = %02X \n", ret);
				fts_err(info, " SS RAW (PROXIMITY) FORCE GAP TEST:.................FAIL \n\n");
				count_fail += 1;
				ftm4_print_frame_short(info, "SS Raw force frame =", ftm4_array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
				if (stop_on_fail) {
					ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
					goto ERROR_LIMITS;
				}
			} else {
				fts_err(info, " SS RAW (PROXIMITY) FORCE GAP TEST:.................OK \n\n");
				ftm4_print_frame_short(info, "SS Raw force frame =", ftm4_array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
			}
			kfree(thresholds);
			thresholds = NULL;
		} else
			fts_err(info, " SS RAW (PROXIMITY) FORCE GAP TEST:.................SKIPPED \n\n");

		kfree(ssRawFrame.force_data);
	ssRawFrame.force_data = NULL;
	} else
		fts_err(info, " SS RAW (PROXIMITY) FORCE TEST:.................SKIPPED \n\n");

	fts_err(info, " \n");
	/*SS RAW (PROXIMITY) SENSE TEST */
	fts_err(info, " SS RAW (PROXIMITY) SENSE TEST:  \n");

	if (todo->SelfSenseRaw == 1 || todo->SelfSenseRawGap == 1) {
		columns = ssRawFrame.header.sense_node;
		rows = 1; /* there are no data for the force channels due to the fact that the sense frame is analized */

		fts_err(info, " SS RAW (PROXIMITY) SENSE MIN MAX TEST:  \n");
		if (todo->SelfSenseRaw == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_RAW_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 2)) {
				fts_err(info, " SS_RAW_SENSE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMinMax(info, ssRawFrame.sense_data, rows, columns, thresholds[0], thresholds[1]);
			if (ret != OK) {
				fts_err(info, " SS RAW (PROXIMITY) SENSE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS RAW (PROXIMITY) SENSE MIN MAX TEST:.................FAIL \n");
				count_fail += 1;
				ftm4_print_frame_short(info, "SS Raw sense frame =", ftm4_array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
				if (stop_on_fail) {
			ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
			goto ERROR_LIMITS;
		}
			} else {
				fts_err(info, " SS RAW (PROXIMITY) SENSE MIN MAX TEST:.................OK \n");
				ftm4_print_frame_short(info, "SS Raw sense frame =", ftm4_array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
				}
			kfree(thresholds);
		thresholds = NULL;
		} else
			fts_err(info, " SS RAW (PROXIMITY) SENSE MIN MAX TEST:.................SKIPPED \n");

		fts_err(info, " \n");
		fts_err(info, " SS RAW (PROXIMITY) SENSE GAP TEST:  \n");
		if (todo->SelfSenseRawGap == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_RAW_SENSE_GAP, &thresholds, &trows, &tcolumns);
			if (ret < 0 || (trows != 1 || tcolumns != 1)) {
				fts_err(info, " SS_RAW_SENSE_GAP failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsGap(info, ssRawFrame.sense_data, rows, columns, thresholds[0]);
			if (ret != OK) {
				fts_err(info, " SS RAW (PROXIMITY) SENSE GAP failed... ERROR = %02X \n", ret);
				fts_err(info, " SS RAW (PROXIMITY) SENSE GAP TEST:.................FAIL \n");
				count_fail += 1;
				ftm4_print_frame_short(info, "SS Raw sense frame =", ftm4_array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
				if (stop_on_fail) {
			ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
			goto ERROR_LIMITS;
		}
			} else {
				fts_err(info, " SS RAW (PROXIMITY) SENSE GAP TEST:.................OK \n");
				ftm4_print_frame_short(info, "SS Raw sense frame =", ftm4_array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
			}
			kfree(thresholds);
		thresholds = NULL;
		} else
			fts_err(info, " SS RAW (PROXIMITY) SENSE GAP TEST:.................SKIPPED \n");

		kfree(ssRawFrame.sense_data);
	ssRawFrame.sense_data = NULL;
	}

	fts_err(info, " \n");
	if (count_fail == 0) {
		fts_err(info, " SS RAW testes finished!.................OK\n\n");
		return OK;
	} else {
		fts_err(info, " SS RAW testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
		return (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA);
	}

ERROR_LIMITS:
	if (ssRawFrame.force_data != NULL)
		kfree(ssRawFrame.force_data);
	if (ssRawFrame.sense_data != NULL)
		kfree(ssRawFrame.sense_data);
	if (thresholds != NULL)
		kfree(thresholds);
	return ret;

}

int ftm4_production_test_ss_ix_cx(struct fts_ts_info *info, char *path_limits, int stop_on_fail, TestToDo *todo)
{

	int ret;
	int count_fail = 0;

	int *thresholds = NULL;
	int trows, tcolumns;
	int *thresholds_min = NULL;
	int *thresholds_max = NULL;

	SelfSenseData ssCompData;

	u8 *adjhor = NULL;
	u8 *adjvert = NULL;

	u16 container;
	int *ix1_w = NULL;
	int *ix2_w = NULL;
	u16 *total_ix = NULL;
	u16 *total_cx = NULL;

	u16 *total_adjhor = NULL;
	u16 *total_adjvert = NULL;

	fts_err(info, " \n");
	fts_err(info, " SS IX CX testes are starting...  \n");
	ret = ftm4_readSelfSenseCompensationData(info, SS_TOUCH, &ssCompData); /*read the SS compensation data */
	if (ret < 0) {
		fts_err(info, " failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
		return (ret | ERROR_PROD_TEST_DATA);
	}

	/********************************************************** SS FORCE IX ****************************************************************/
	/*SS IX1 FORCE TEST */
	fts_err(info, " SS IX1 FORCE TEST:  \n");
	if (todo->SelfForceIx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX1_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, " SS_IX1_FORCE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		container = (u16) ssCompData.f_ix1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " SS IX1 FORCE TEST failed... ERROR COUNT = %d \n", ret);
			count_fail += 1;
			if (stop_on_fail)
	goto ERROR;
		} else
			fts_err(info, " SS IX1 FORCE TEST:.................OK \n\n");
	} else
		fts_err(info, " SS IX1 FORCE TEST:.................SKIPPED \n\n");

	kfree(thresholds);
	thresholds = NULL;
	/*SS IX2 FORCE TEST */
	fts_err(info, " SS IX2 FORCE MIN MAX TEST:  \n");
	if (todo->SelfForceIx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
			fts_err(info, " SS_IX2_FORCE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
			fts_err(info, " SS_IX2_FORCE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, ssCompData.ix2_fm, ssCompData.header.force_node, 1, thresholds_min, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS IX2 FORCE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS IX2 FORCE MIN MAX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS IX2 FORCE MIN MAX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " SS IX2 FORCE MIN MAX TEST:.................SKIPPED \n\n");

	fts_err(info, " SS IX2 FORCE ADJ TEST:  \n");
	if (todo->SelfForceIx2Adj == 1) {
		/*SS IX2 FORCE ADJV TEST */
		fts_err(info, " SS IX2 FORCE ADJVERT TEST:  \n");
		ret = ftm4_computeAdjVert(info, ssCompData.ix2_fm, ssCompData.header.force_node, 1, &adjvert);
		if (ret < 0) {
			fts_err(info, " SS IX2 FORCE ADJV failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " SS IX2 FORCE ADJV computed! \n");

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
			fts_err(info, " SS_IX2_FORCE_ADJV_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjvert, ssCompData.header.force_node - 1, 1, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS IX2 FORCE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS IX2 FORCE ADJV TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS IX2 FORCE ADJV TEST:.................OK \n\n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjvert);
	adjvert = NULL;

	} else
		fts_err(info, " SS IX2 FORCE ADJ TEST:.................SKIPPED \n\n");

	/*SS TOTAL FORCE IX */
	fts_err(info, " SS TOTAL IX FORCE TEST:  \n");
	if (todo->SelfForceIxTotal == 1 || todo->SelfForceIxTotalAdj == 1) {
		fts_err(info, " Reading TOTAL IX FORCE Weights...  \n");
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX1_FORCE_W, &ix1_w, &trows, &tcolumns); /*load the IX1 weight */
		if (ret < 0 || (trows != 1 || tcolumns != 1)) {
			fts_err(info, " SS_IX1_FORCE_W failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			return (ret | ERROR_PROD_TEST_DATA);
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_FORCE_W, &ix2_w, &trows, &tcolumns); /*load the IX2 weight */
		if (ret < 0 || (trows != 1 || tcolumns != 1)) {
			fts_err(info, " SS_IX1_FORCE_W failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			return (ret | ERROR_PROD_TEST_DATA);
		}

		fts_err(info, " Weights: IX1_W = %d   IX2_W = %d  \n", *ix1_w, *ix2_w);

		ret = ftm4_computeTotal(info, ssCompData.ix2_fm, ssCompData.f_ix1, ssCompData.header.force_node, 1, *ix1_w, *ix2_w, &total_ix);
		if (ret < 0) {
			fts_err(info, " Ix Force failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

	kfree(ix1_w);
	ix1_w = NULL;
		kfree(ix2_w);
	ix2_w = NULL;

		fts_err(info, " SS TOTAL IX FORCE MIN MAX TEST:  \n");
		if (todo->SelfForceIxTotal == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_IX_FORCE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_IX_FORCE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapTotal(info, total_ix, ssCompData.header.force_node, 1, thresholds_min, thresholds_max); /*check the values with thresholds */
			ftm4_print_frame_short(info, "SS TOTAL IX FORCE =", ftm4_array1dTo2d_short(total_ix, ssCompData.header.force_node, 1), ssCompData.header.force_node, 1);
			if (ret != OK) {
				fts_err(info, " SS TOTAL IX FORCE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL IX FORCE MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL IX FORCE MIN MAX TEST:.................OK \n\n");

			kfree(thresholds_min);
		thresholds_min = NULL;
			kfree(thresholds_max);
		thresholds_max = NULL;
		} else
			fts_err(info, " SS TOTAL IX FORCE MIN MAX TEST:.................SKIPPED \n");

		fts_err(info, " SS TOTAL IX FORCE ADJ TEST:  \n");
		if (todo->SelfForceIxTotalAdj == 1) {
			/*SS TOTAL IX FORCE ADJV TEST */
			fts_err(info, " SS TOTAL IX FORCE ADJVERT TEST:  \n");
			ret = ftm4_computeAdjVertTotal(info, total_ix, ssCompData.header.force_node, 1, &total_adjvert);
			if (ret < 0) {
				fts_err(info, " SS TOTAL IX FORCE ADJV failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}
			fts_err(info, " SS TOTAL IX FORCE ADJV computed! \n");

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_IX_FORCE_ADJV_MAP_MAX... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjvert, ssCompData.header.force_node - 1, 1, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL IX FORCE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL IX FORCE ADJV TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL IX FORCE ADJV TEST:.................OK \n\n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjvert);
		total_adjvert = NULL;
		} else
			fts_err(info, " SS TOTAL IX FORCE ADJ TEST:.................SKIPPED  \n");

		kfree(total_ix);
	total_ix = NULL;
	} else
		fts_err(info, " SS TOTAL IX FORCE TEST:.................SKIPPED \n\n");


	/********************************************************** SS SENSE IX ****************************************************************/
	/*SS IX1 SENSE TEST */
	fts_err(info, " SS IX1 SENSE TEST:  \n");
	if (todo->SelfSenseIx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX1_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, " SS_IX1_SENSE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		container = (u16) ssCompData.s_ix1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " SS IX1 SENSE TEST failed... ERROR COUNT = %d \n", ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS IX1 SENSE TEST:.................OK \n\n");
	} else
		fts_err(info, " SS IX1 SENSE TEST:.................SKIPPED \n\n");

	kfree(thresholds);
	thresholds = NULL;
	/*SS IX2 SENSE TEST */
	fts_err(info, " SS IX2 SENSE MIN MAX TEST:  \n");
	if (todo->SelfSenseIx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
			fts_err(info, " SS_IX2_SENSE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
			fts_err(info, " SS_IX2_SENSE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, ssCompData.ix2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS IX2 SENSE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS IX2 SENSE MIN MAX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS IX2 SENSE MIN MAX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " SS IX2 SENSE MIN MAX TEST:.................SKIPPED \n\n");

	fts_err(info, " SS IX2 SENSE ADJ TEST:  \n");
	if (todo->SelfSenseIx2Adj == 1) {
		/*SS IX2 SENSE ADJH TEST */
		fts_err(info, " SS IX2 SENSE ADJHORIZ TEST:  \n");
		ret = ftm4_computeAdjHoriz(info, ssCompData.ix2_sn, 1, ssCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			fts_err(info, " SS IX2 SENSE ADJH failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " SS IX2 SENSE ADJ HORIZ computed! \n");


		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
			fts_err(info, " SS_IX2_SENSE_ADJH_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS IX2 SENSE ADJH failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS IX2 SENSE ADJH TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS IX2 SENSE ADJH TEST:.................OK \n\n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjhor);
	adjhor = NULL;
	} else
		fts_err(info, " SS IX2 SENSE ADJ TEST:.................SKIPPED  \n");

	/*SS TOTAL IX SENSE */
	fts_err(info, " SS TOTAL IX SENSE TEST:  \n");
	if (todo->SelfSenseIxTotal == 1 || todo->SelfSenseIxTotalAdj == 1) {
		fts_err(info, " Reading TOTAL IX SENSE Weights...  \n");
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX1_SENSE_W, &ix1_w, &trows, &tcolumns); /*load the IX1 weight */
		if (ret < 0 || (trows != 1 || tcolumns != 1)) {
			fts_err(info, " SS_IX1_SENSE_W failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_IX2_SENSE_W, &ix2_w, &trows, &tcolumns); /*load the IX2 weight */
		if (ret < 0 || (trows != 1 || tcolumns != 1)) {
			fts_err(info, " SS_IX1_SENSE_W failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		fts_err(info, " Weights: IX1_W = %d   IX2_W = %d  \n", *ix1_w, *ix2_w);

		ret = ftm4_computeTotal(info, ssCompData.ix2_sn, ssCompData.s_ix1, 1, ssCompData.header.sense_node, *ix1_w, *ix2_w, &total_ix);
		if (ret < 0) {
			fts_err(info, " Ix Sense failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

	kfree(ix1_w);
	ix1_w = NULL;
		kfree(ix2_w);
	ix2_w = NULL;

		fts_err(info, " SS TOTAL IX SENSE MIN MAX TEST:  \n");
		if (todo->SelfSenseIxTotal == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
				fts_err(info, " SS_TOTAL_IX_SENSE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
				fts_err(info, " SS_TOTAL_IX_SENSE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapTotal(info, total_ix, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max); /*check the values with thresholds */
			ftm4_print_frame_short(info, "SS TOTAL IX SENSE =", ftm4_array1dTo2d_short(total_ix, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
			if (ret != OK) {
				fts_err(info, " SS TOTAL IX SENSE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL IX SENSE MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL IX SENSE MIN MAX TEST:.................OK \n\n");

			kfree(thresholds_min);
		thresholds_min = NULL;
			kfree(thresholds_max);
		thresholds_max = NULL;
		} else
			fts_err(info, " SS TOTAL IX SENSE MIN MAX TEST:.................SKIPPED  \n");


		fts_err(info, " SS TOTAL IX SENSE ADJ TEST:  \n");
		if (todo->SelfSenseIxTotalAdj == 1) {
			/*SS TOTAL IX SENSE ADJH TEST */
			fts_err(info, " SS TOTAL IX SENSE ADJHORIZ TEST:  \n");
			ret = ftm4_computeAdjHorizTotal(info, total_ix, 1, ssCompData.header.sense_node, &total_adjhor);
			if (ret < 0) {
				fts_err(info, " SS TOTAL IX SENSE ADJH failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}
			fts_err(info, " SS TOTAL IX SENSE ADJ HORIZ computed! \n");


			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_IX_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
				fts_err(info, " SS_TOTAL_IX_SENSE_ADJH_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL IX SENSE ADJH failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL IX SENSE ADJH TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL IX SENSE ADJH TEST:.................OK \n\n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjhor);
		total_adjhor = NULL;
		} else
			fts_err(info, " SS TOTAL IX SENSE ADJ TEST:.................SKIPPED  \n");
		kfree(total_ix);
	total_ix = NULL;
	} else
		fts_err(info, " SS TOTAL IX SENSE TEST:.................SKIPPED  \n");

	/********************************************************** SS SENSE CX ****************************************************************/
	/*SS CX1 FORCE TEST */
	fts_err(info, " SS CX1 FORCE TEST:  \n");
	if (todo->SelfForceCx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX1_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, " SS_CX1_FORCE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		container = (u16) ssCompData.f_cx1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " SS CX1 FORCE TEST failed... ERROR COUNT = %d \n", ret);
			count_fail += 1;
			/*if (stop_on_fail) return (ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL); */
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX1 FORCE TEST:.................OK \n\n");
		kfree(thresholds);
	thresholds = NULL;
	} else
		fts_err(info, " SS CX1 FORCE TEST:.................SKIPPED \n\n");



	/*SS CX2 FORCE TEST */
	fts_err(info, " SS CX2 FORCE MIN MAX TEST:  \n");
	if (todo->SelfForceCx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
			fts_err(info, " SS_CX2_FORCE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
			fts_err(info, " SS_CX2_FORCE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, ssCompData.cx2_fm, ssCompData.header.force_node, 1, thresholds_min, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS CX2 FORCE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS CX2 FORCE MIN MAX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX2 FORCE MIN MAX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " SS CX2 FORCE MIN MAX TEST:.................SKIPPED  \n");

	fts_err(info, " SS CX2 FORCE ADJ TEST:  \n");
	if (todo->SelfForceCx2Adj == 1) {
		/*SS CX2 FORCE ADJV TEST */
		fts_err(info, " SS CX2 FORCE ADJVERT TEST:  \n");
		ret = ftm4_computeAdjVert(info, ssCompData.cx2_fm, ssCompData.header.force_node, 1, &adjvert); /*comepute the ADJV for CX2  FORCE */
		if (ret < 0) {
			fts_err(info, " SS CX2 FORCE ADJV failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " SS CX2 FORCE ADJV computed! \n");

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
			fts_err(info, " SS_CX2_FORCE_ADJV_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjvert, ssCompData.header.force_node - 1, 1, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS IX2 FORCE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS CX2 FORCE ADJV TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX2 FORCE ADJV TEST:.................OK \n\n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjvert);
	adjvert = NULL;
	} else
		fts_err(info, " SS CX2 FORCE ADJ TEST:.................SKIPPED \n\n");

	/*SS TOTAL CX FORCE */
	fts_err(info, " SS TOTAL CX FORCE TEST:  \n");
	if (todo->SelfForceCxTotal == 1 || todo->SelfForceCxTotalAdj == 1) {
		ret = ftm4_computeTotal(info, ssCompData.cx2_fm, ssCompData.f_cx1, ssCompData.header.force_node, 1, CX1_WEIGHT, CX2_WEIGHT, &total_cx);
		if (ret < 0) {
			fts_err(info, " Cx Force failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			return (ret | ERROR_PROD_TEST_DATA);
		}

		fts_err(info, " SS TOTAL CX FORCE MIN MAX TEST:  \n");
		if (todo->SelfForceCxTotal == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_CX_FORCE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_CX_FORCE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapTotal(info, total_cx, ssCompData.header.force_node, 1, thresholds_min, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL FORCE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL FORCE MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL FORCE MIN MAX TEST:.................OK \n\n");

			kfree(thresholds_min);
		thresholds_min = NULL;
			kfree(thresholds_max);
		thresholds_max = NULL;
		} else
			fts_err(info, " SS TOTAL CX FORCE MIN MAX TEST:.................SKIPPED  \n");

		/*SS TOTAL CX FORCE ADJV TEST */
		fts_err(info, " SS TOTAL CX FORCE ADJ TEST:  \n");
		if (todo->SelfForceCxTotalAdj == 1) {
			fts_err(info, " SS TOTAL CX FORCE ADJVERT TEST:  \n");
			ret = ftm4_computeAdjVertTotal(info, total_cx, ssCompData.header.force_node, 1, &total_adjvert); /*comepute the ADJV for CX2  FORCE */
			if (ret < 0) {
				fts_err(info, " SS TOTAL CX FORCE ADJV failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}
			fts_err(info, " SS TOTAL CX FORCE ADJV computed! \n");

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
				fts_err(info, " SS_TOTAL_CX_FORCE_ADJV_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
				goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjvert, ssCompData.header.force_node - 1, 1, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL CX FORCE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL CX FORCE ADJV TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL CX FORCE ADJV TEST:.................OK \n\n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjvert);
		total_adjvert = NULL;

		} else
			fts_err(info, " SS TOTAL CX FORCE ADJ TEST:.................SKIPPED  \n");
		kfree(total_cx);
	total_cx = NULL;
	} else
		fts_err(info, " SS TOTAL CX FORCE TEST:.................SKIPPED \n\n");



	/********************************************************** SS SENSE CX ****************************************************************/
	/*SS CX1 SENSE TEST */
	fts_err(info, " SS CX1 SENSE TEST:  \n");
	if (todo->SelfSenseCx1 == 1) {

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX1_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
		if (ret < 0 || (trows != 1 || tcolumns != 2)) {
			fts_err(info, " SS_CX1_SENSE_MIN_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		container = (u16) ssCompData.s_cx1;
		ret = ftm4_checkLimitsMinMax(info, &container, 1, 1, thresholds[0], thresholds[1]); /*check the limits */
		if (ret != OK) {
			fts_err(info, " SS CX1 SENSE TEST failed... ERROR COUNT = %d \n", ret);
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX1 SENSE TEST:.................OK \n\n");
		kfree(thresholds);
	thresholds = NULL;
	} else
		fts_err(info, " SS CX1 SENSE TEST:.................SKIPPED \n\n");


	/*SS CX2 SENSE TEST */
	fts_err(info, " SS CX2 SENSE MIN MAX TEST:  \n");
	if (todo->SelfSenseCx2 == 1) {
		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
			fts_err(info, " SS_CX2_SENSE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
			fts_err(info, " SS_CX2_SENSE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMap(info, ssCompData.cx2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS CX2 SENSE failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS CX2 SENSE MIN MAX TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX2 SENSE MIN MAX TEST:.................OK \n\n");

		kfree(thresholds_min);
	thresholds_min = NULL;
		kfree(thresholds_max);
	thresholds_max = NULL;
	} else
		fts_err(info, " SS CX2 SENSE MIN MAX TEST:.................SKIPPED  \n");

	fts_err(info, " SS CX2 SENSE ADJ TEST:  \n");
	if (todo->SelfSenseCx2Adj == 1) {
		/*SS CX2 SENSE ADJH TEST */
		fts_err(info, " SS CX2 SENSE ADJHORIZ TEST:  \n");
		ret = ftm4_computeAdjHoriz(info, ssCompData.ix2_sn, 1, ssCompData.header.sense_node, &adjhor);
		if (ret < 0) {
			fts_err(info, " SS CX2 SENSE ADJH failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}
		fts_err(info, " SS CX2 SENSE ADJH computed! \n");


		ret = ftm4_parseProductionTestLimits(info, path_limits, SS_CX2_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
		if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
			fts_err(info, " SS_IX2_SENSE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		ret = ftm4_checkLimitsMapAdj(info, adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /*check the values with thresholds */
		if (ret != OK) {
			fts_err(info, " SS CX2 SENSE ADJH failed... ERROR COUNT = %d \n", ret);
			fts_err(info, " SS CX2 SENSE ADJH TEST:.................FAIL \n\n");
			count_fail += 1;
			if (stop_on_fail)
				goto ERROR;
		} else
			fts_err(info, " SS CX2 SENSE ADJH TEST:.................OK \n");

		kfree(thresholds_max);
	thresholds_max = NULL;
		kfree(adjhor);
	adjhor = NULL;
	} else
		fts_err(info, " SS CX2 SENSE ADJ TEST:.................SKIPPED \n\n");

	/*SS TOTAL CX SENSE */
	fts_err(info, " SS TOTAL CX SENSE TEST:  \n");
	if (todo->SelfSenseCxTotal == 1 || todo->SelfSenseCxTotalAdj == 1) {
		ret = ftm4_computeTotal(info, ssCompData.cx2_sn, ssCompData.s_cx1, 1, ssCompData.header.sense_node, CX1_WEIGHT, CX2_WEIGHT, &total_cx);
		if (ret < 0) {
			fts_err(info, " Cx Sense failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
			ret |= ERROR_PROD_TEST_DATA;
		goto ERROR_LIMITS;
		}

		fts_err(info, " SS TOTAL CX SENSE MIN MAX TEST:  \n");
		if (todo->SelfSenseCxTotal == 1) {
			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /*load the min thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
				fts_err(info, " SS_TOTAL_CX_SENSE_MAP_MIN failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
				fts_err(info, " SS_TOTAL_CX_SENSE_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapTotal(info, total_cx, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL CX SENSE failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL CX SENSE MIN MAX TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL CX SENSE MIN MAX TEST:.................OK \n\n");

			kfree(thresholds_min);
		thresholds_min = NULL;
			kfree(thresholds_max);
		thresholds_max = NULL;
		} else
			fts_err(info, " SS TOTAL CX SENSE MIN MAX TEST:.................SKIPPED  \n");


		/*SS TOTAL IX SENSE ADJH TEST */
		fts_err(info, " SS TOTAL CX SENSE ADJ TEST:  \n");
		if (todo->SelfSenseCxTotalAdj == 1) {
			fts_err(info, " SS TOTAL CX SENSE ADJHORIZ TEST:  \n");
			ret = ftm4_computeAdjHorizTotal(info, total_cx, 1, ssCompData.header.sense_node, &total_adjhor);
			if (ret < 0) {
				fts_err(info, " SS TOTAL CX SENSE ADJH failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}
			fts_err(info, " SS TOTAL CX SENSE ADJ HORIZ computed! \n");


			ret = ftm4_parseProductionTestLimits(info, path_limits, SS_TOTAL_CX_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /*load the max thresholds */
			if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
				fts_err(info, " SS_TOTAL_CX_SENSE_ADJH_MAP_MAX failed... ERROR %02X \n", ERROR_PROD_TEST_DATA);
				ret |= ERROR_PROD_TEST_DATA;
			goto ERROR_LIMITS;
			}

			ret = ftm4_checkLimitsMapAdjTotal(info, total_adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /*check the values with thresholds */
			if (ret != OK) {
				fts_err(info, " SS TOTAL CX SENSE ADJH failed... ERROR COUNT = %d \n", ret);
				fts_err(info, " SS TOTAL CX SENSE ADJH TEST:.................FAIL \n\n");
				count_fail += 1;
				if (stop_on_fail)
					goto ERROR;
			} else
				fts_err(info, " SS TOTAL CX SENSE ADJH TEST:.................OK \n\n");

			kfree(thresholds_max);
		thresholds_max = NULL;
			kfree(total_adjhor);
		total_adjhor = NULL;
		} else
			fts_err(info, " SS TOTAL CX SENSE ADJ TEST:.................SKIPPED  \n");
		kfree(total_cx);
	total_cx = NULL;
	} else
		fts_err(info, " SS TOTAL CX SENSE TEST:.................SKIPPED  \n");



ERROR:
	fts_err(info, " \n");
	if (count_fail == 0) {
		kfree(ssCompData.ix2_fm);
		ssCompData.ix2_fm = NULL;
		kfree(ssCompData.ix2_sn);
		ssCompData.ix2_sn = NULL;
		kfree(ssCompData.cx2_fm);
		ssCompData.cx2_fm = NULL;
		kfree(ssCompData.cx2_sn);
		ssCompData.cx2_sn = NULL;
		fts_err(info, " SS IX CX testes finished!.................OK\n\n");
		return OK;
	} else {
		/*print all kind of data in just one row for readability reason */
		ftm4_print_frame_u8(info, "SS Init Data Ix2_fm = ", ftm4_array1dTo2d_u8(ssCompData.ix2_fm, ssCompData.header.force_node, ssCompData.header.force_node), 1, ssCompData.header.force_node);
		ftm4_print_frame_u8(info, "SS Init Data Cx2_fm = ", ftm4_array1dTo2d_u8(ssCompData.cx2_fm, ssCompData.header.force_node, ssCompData.header.force_node), 1, ssCompData.header.force_node);
		ftm4_print_frame_u8(info, "SS Init Data Ix2_sn = ", ftm4_array1dTo2d_u8(ssCompData.ix2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
		ftm4_print_frame_u8(info, "SS Init Data Cx2_sn = ", ftm4_array1dTo2d_u8(ssCompData.cx2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
		fts_err(info, " SS IX CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
		if (thresholds != NULL)
			kfree(thresholds);
		if (thresholds_min != NULL)
			kfree(thresholds_min);
		if (thresholds_max != NULL)
			kfree(thresholds_max);
		if (adjhor != NULL)
			kfree(adjhor);
		if (adjvert != NULL)
			kfree(adjvert);
		if (ix1_w != NULL)
			kfree(ix1_w);
		if (ix2_w != NULL)
			kfree(ix2_w);
		if (total_ix != NULL)
			kfree(total_ix);
		if (total_cx != NULL)
			kfree(total_cx);
		if (total_adjhor != NULL)
			kfree(total_adjhor);
		if (total_adjvert != NULL)
			kfree(total_adjvert);
		if (ssCompData.ix2_fm != NULL)
			kfree(ssCompData.ix2_fm);
		if (ssCompData.ix2_sn != NULL)
			kfree(ssCompData.ix2_sn);
		if (ssCompData.cx2_fm != NULL)
			kfree(ssCompData.cx2_fm);
		if (ssCompData.cx2_sn != NULL)
			kfree(ssCompData.cx2_sn);
		return (ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA);
	}

ERROR_LIMITS:
	if (thresholds != NULL)
		kfree(thresholds);
	if (thresholds_min != NULL)
		kfree(thresholds_min);
	if (thresholds_max != NULL)
		kfree(thresholds_max);
	if (adjhor != NULL)
		kfree(adjhor);
	if (adjvert != NULL)
		kfree(adjvert);
	if (ix1_w != NULL)
		kfree(ix1_w);
	if (ix2_w != NULL)
		kfree(ix2_w);
	if (total_ix != NULL)
		kfree(total_ix);
	if (total_cx != NULL)
		kfree(total_cx);
	if (total_adjhor != NULL)
		kfree(total_adjhor);
	if (total_adjvert != NULL)
		kfree(total_adjvert);
	if (ssCompData.ix2_fm != NULL)
		kfree(ssCompData.ix2_fm);
	if (ssCompData.ix2_sn != NULL)
		kfree(ssCompData.ix2_sn);
	if (ssCompData.cx2_fm != NULL)
		kfree(ssCompData.cx2_fm);
	if (ssCompData.cx2_sn != NULL)
		kfree(ssCompData.cx2_sn);
	return ret;
}

int ftm4_production_test_data(struct fts_ts_info *info,char *path_limits, int stop_on_fail, TestToDo *todo)
{
	int res = OK, ret;

	if (todo == NULL) {
		fts_err(info, " No TestToDo specified!! ERROR = %02X \n", (ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA));
		return (ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA);
	}


	fts_err(info, " DATA Production test is starting...\n");


	ret = ftm4_production_test_ms_raw(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		fts_err(info, " failed... ERROR = %02X \n", ret);
		if (stop_on_fail == 1)
			goto END;
	}



	ret = ftm4_production_test_ms_cx(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		fts_err(info, " failed... ERROR = %02X \n", ret);
		if (stop_on_fail == 1)
			goto END;
	}


	ret = ftm4_production_test_ss_raw(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		fts_err(info, " failed... ERROR = %02X \n", ret);
		if (stop_on_fail == 1)
			goto END;
	}

	ret = ftm4_production_test_ss_ix_cx(info, path_limits, stop_on_fail, todo);
	res |= ret;
	if (ret < 0) {
		fts_err(info, " failed... ERROR = %02X \n", ret);
		if (stop_on_fail == 1)
			goto END;
	}

END:
	if (res < OK)
		fts_err(info, " DATA Production test failed!\n");
	else
		fts_err(info, " DATA Production test finished!\n");
	return res;
}

int fts_save_mp_flag(struct fts_ts_info *info, u32 signature)
{
	int res = -1;
	int i;
	u8 cmd[6] = {FTS_CMD_WRITE_MP_FLAG, 0x00, 0x00, 0x00, 0x00, 0x00};

	fts_u32ToU8(signature, &cmd[2]);

	fts_info(info, "Starting Saving Flag with signature = %08X ... ", signature);

	for (i = 0; i < SAVE_FLAG_RETRY && res < OK; i++) {
		fts_info(info, "Attempt number %d to save mp flag !", i + 1);
		fts_info(info, "Command write flag sent...");
		res = ftm4_writeFwCmd(info, cmd, 6);
		if (res >= OK)
			res = __fts_save_mp_flag(info);	//ftm4_save_cx_tuning();

	}

	if (res < OK) {
		fts_info(info, "save_mp_flag: ERROR %08X ... ", signature);
		return res;
	} else {
		fts_info(info, "Saving Flag DONE!");
		return OK;
	}
}

int ftm4_parseProductionTestLimits(struct fts_ts_info *info, char *path, char *label, int **data, int *row, int *column)
{

	int find = 0;
	char *token = NULL;
	int i = 0;
	int j = 0;
	int z = 0;


	char *line2 = NULL;
	char line[800];
	int fd = -1;
	char *buf = NULL;
	int n, size, pointer = 0, ret = OK;
	char *data_file = NULL;
	fd = 0;

	if (fd == 0) {
		data_file = vts_fw_data_get(info->vtsdev, VTS_FW_TYPE_LIMIT, &size);
		if (data_file == NULL || size == 0) {
			fts_info(info, "vivoTsGet limit fail");
			return -1;
		}

		fts_info(info, "The size of the limits file is %d bytes...", size);

		while (find == 0) {
			/*start to look for the wanted label */
			if (ftm4_readLine(&data_file[pointer], line, size - pointer, &n) < 0) {
				find = -1;
				break;
			}
			pointer += n;
			if (line[0] == '*') {														/*each header row start with *  ex. *label, n_row, n_colum */
				line2 = kstrdup(line, GFP_KERNEL);
				if (line2 == NULL) {
					fts_info(info, "kstrdup ERROR %02X", ERROR_ALLOC);
					ret = ERROR_ALLOC;
					goto END;
				}
				buf = line2;
				line2 += 1;
				token = strsep(&line2, ",");
				if (strcmp(token, label) == 0) {										/*if the row is the wanted one i retrieve rows and columns info */
					find = 1;
					token = strsep(&line2, ",");
					if (token != NULL) {
						sscanf(token, "%d", row);
						fts_info(info, "Row = %d", *row);
					} else {
						fts_info(info, " 1: ERROR %02X",  ERROR_FILE_PARSE);
						/*release_firmware(fw); */
						/*return ERROR_FILE_PARSE; */
						ret = ERROR_FILE_PARSE;
						goto END;
					}
					token = strsep(&line2, ",");
					if (token != NULL) {
						sscanf(token, "%d", column);
						fts_info(info, "Column = %d", *column);
					} else {
						fts_info(info, " 2: ERROR %02X", ERROR_FILE_PARSE);
						/*release_firmware(fw); */
						/*return ERROR_FILE_PARSE; */
						ret = ERROR_FILE_PARSE;
						goto END;
					}

					kfree(buf);
					buf = NULL;
					*data = (int *)kmalloc(((*row) * (*column)) * sizeof(int), GFP_KERNEL);				/*allocate the memory for containing the data */
					j = 0;
					if (*data == NULL) {
						fts_info(info, ": ERROR %02X", ERROR_ALLOC);
						/*release_firmware(fw); */
						/*return ERROR_ALLOC; */
						ret = ERROR_ALLOC;
						goto END;
					}


					/*start to read the data */
					for (i = 0; i < *row; i++) {
						/*line =  buf; */
						if (ftm4_readLine(&data_file[pointer], line, size - pointer, &n) < 0) {
							fts_info(info, " : ERROR %02X", ERROR_FILE_READ);
							/*release_firmware(fw); */
							/*return ERROR_FILE_READ; */
							ret = ERROR_FILE_READ;
							goto END;
						}
						pointer += n;
						line2 = kstrdup(line, GFP_KERNEL);
						if (line2 == NULL) {
							fts_info(info, ": kstrdup ERROR %02X\n", ERROR_ALLOC);
							ret = ERROR_ALLOC;
							goto END;
						}
						buf = line2;
						token = strsep(&line2, ",");
						for (z = 0; (z < *column) && (token != NULL); z++) {
							sscanf(token, "%d", ((*data) + j));
							j++;
							token = strsep(&line2, ",");
						}
						kfree(buf);
						buf = NULL;
					}
					if (j == ((*row) * (*column))) {												/*check that all the data are read */
						fts_info(info, "READ DONE!");
						/*release_firmware(fw); */
						/*return OK; */
						ret = OK;
						goto END;
					}
					fts_info(info, " 3: ERROR %02X", ERROR_FILE_PARSE);
					/*release_firmware(fw); */
					/*return ERROR_FILE_PARSE; */
					ret = ERROR_FILE_PARSE;
					goto END;
				}
				kfree(buf);
				buf = NULL;
			}

		}
		fts_info(info, ": ERROR %02X", ERROR_LABEL_NOT_FOUND);
		ret = ERROR_LABEL_NOT_FOUND;
END:
		if (buf != NULL)
			kfree(buf);

		vts_fw_data_put(info->vtsdev, VTS_FW_TYPE_LIMIT);
		return ret;

	} else {
		fts_info(info, ": ERROR %02X", ERROR_FILE_NOT_FOUND);
		vts_fw_data_put(info->vtsdev, VTS_FW_TYPE_LIMIT);
		return ERROR_FILE_NOT_FOUND;
	}


}

/*int ftm4_readLine(char * data, char ** line, int size, int *n) {
	int i = 0;
	if (size < 1)
		return -1;

		while (data[i] != '\n' && i < size) {
			 *(*line + i) = data[i];
			i++;
		}
		 *n = i + 1;
		 *(*line + i) = '\0';

	return OK;

} */


int ftm4_readLine(char *data, char *line, int size, int *n)
{
	int i = 0;
	if (size < 1) {
		i = -1;
		return i;
	}

		while (data[i] != '\n' && i < size) {
			line[i] = data[i];
			i++;
		}
		 *n = i + 1;
		line[i] = '\0';

	return OK;

}


