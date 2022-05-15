/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *		FTS Core functions					 *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsCore.c
  * \brief Contains the implementation of the Core functions
  */

#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsIO.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"

/**
  * Initialize core variables of the library.
  * Must be called during the probe before any other lib function
  * @param info pointer to fts_ts_info which contains info about the device and
  *its hw setup
  * @return OK if success or an error code which specify the type of error
  */
int initCore(struct fts_ts_info *info)
{
	int ret = OK;

	info->reset_gpio = GPIO_NOT_DEFINED;
	atomic_set(&info->disable_irq_count, 1);
	logError_st80y(0, "%s %s: Initialization of the Core...\n", tag_st80y, __func__);
	ret |= resetErrorList(info);
	ret |= initTestToDo(info);
	st80y_setResetGpio(info, info->board.reset_gpio);
	if (ret < OK)
		logError_st80y(0, "%s %s: Initialization Core ERROR %08X!\n", tag_st80y,
			 __func__, ret);
	else
		logError_st80y(0, "%s %s: Initialization Finished!\n", tag_st80y, __func__);
	return ret;
}

/**
  * Set the reset_gpio variable with the actual gpio number of the board link to
  *the reset pin
  * @param gpio gpio number link to the reset pin of the IC
  */
void st80y_setResetGpio(struct fts_ts_info *info, int gpio)
{
	info->reset_gpio = gpio;
	logError_st80y(1, "%s st80y_setResetGpio: reset_gpio = %d\n", tag_st80y, info->reset_gpio);
}

/**
  * Perform a system reset of the IC.
  * If the reset pin is associated to a gpio, the function execute an hw reset
  * (toggling of reset pin) otherwise send an hw command to the IC
  * @return OK if success or an error code which specify the type of error
  */
int st80y_system_reset(struct fts_ts_info *info)
{
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search;
	int res = -1;
	int i;
	u8 data[1] = { SYSTEM_RESET_VALUE };
	u32 rst_dis = 0;
	event_to_search = (int)EVT_ID_CONTROLLER_READY;
	if (info->vtsdev && info->vtsdev->module)
		vts_property_get(info->vtsdev, VTS_PROPERTY_RESET_DISCONECT, &rst_dis);
	
	logError_st80y(0, "%s System resetting...\n", tag_st80y);
	for (i = 0; i < RETRY_SYSTEM_RESET && res < 0; i++) {
		resetErrorList(info);
		st80y_disableInterrupt(info);	/* disable interrupt before resetting to
					 * be able to get boot events */
		
		if (info->reset_gpio == GPIO_NOT_DEFINED) {
			res = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
					    ADDR_SYSTEM_RESET, data, ARRAY_SIZE(
						    data));
	
		}
		else {
			if (rst_dis) {
				VTI("soft reset");
				res = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
					ADDR_SYSTEM_RESET, data, ARRAY_SIZE(
					data));
			} else {
				gpio_set_value(info->reset_gpio, 0);
				msleep(10);
				gpio_set_value(info->reset_gpio, 1);
				res = OK;
			}
		}
		if (res < OK)
			logError_st80y(1, "%s st80y_system_reset: ERROR %08X\n", tag_st80y,
				 ERROR_BUS_W);
		else {
			res = st80y_pollForEvent(info, &event_to_search, 1, readData,
					   GENERAL_TIMEOUT);
			if (res < OK)
				logError_st80y(1, "%s st80y_system_reset: ERROR %08X\n",
					 tag_st80y, res);
		}
	}
	if (res < OK) {
		logError_st80y(1,
			 "%s st80y_system_reset...failed after 3 attempts: ERROR %08X\n",
			 tag_st80y, (res | ERROR_SYSTEM_RESET_FAIL));
		return res | ERROR_SYSTEM_RESET_FAIL;
	} else {
		logError_st80y(0, "%s System reset DONE!\n", tag_st80y);
		info->system_reseted_down = 1;
		info->system_reseted_up = 1;
		return OK;
	}
}

/**
  * Return the value of system_resetted_down.
  * @return the flag value: 0 if not set, 1 if set
  */
int st80y_isSystemResettedDown(struct fts_ts_info *info)
{
	return info->system_reseted_down;
}

/**
  * Return the value of system_resetted_up.
  * @return the flag value: 0 if not set, 1 if set
  */
int st80y_isSystemResettedUp(struct fts_ts_info *info)
{
	return info->system_reseted_up;
}


/**
  * Set the value of system_reseted_down flag
  * @param val value to write in the flag
  */
void setSystemResetedDown(struct fts_ts_info *info, int val)
{
	info->system_reseted_down = val;
}

/**
  * Set the value of system_reseted_up flag
  * @param val value to write in the flag
  */
void setSystemResetedUp(struct fts_ts_info *info, int val)
{
	info->system_reseted_up = val;
}


/** @addtogroup events_group
  * @{
  */

/**
  * Poll the FIFO looking for a specified event within a timeout. Support a
  *retry mechanism.
  * @param event_to_search pointer to an array of int where each element
  * correspond to a byte of the event to find.
  * If the element of the array has value -1, the byte of the event,
  * in the same position of the element is ignored.
  * @param event_bytes size of event_to_search
  * @param readData pointer to an array of byte which will contain the event
  *found
  * @param time_to_wait time to wait before going in timeout
  * @return OK if success or an error code which specify the type of error
  */
int st80y_pollForEvent(struct fts_ts_info *info, int *event_to_search, int event_bytes, u8 *readData, int
		 time_to_wait)
{
	int i, find, retry, count_err;
	int time_to_count;
	int err_handling = OK;
	StopWatch clock;

	u8 cmd[1] = { FIFO_CMD_READONE };
	char temp[128] = { 0 };

	find = 0;
	retry = 0;
	count_err = 0;
	time_to_count = time_to_wait / TIMEOUT_RESOLUTION;

	st80y_startStopWatch(&clock);
	while (find != 1 && retry < time_to_count &&
		st80y_writeReadU8UX(info, cmd[0],
				  0, 0,
				  readData,
				  FIFO_EVENT_SIZE,
				  DUMMY_FIFO) >= OK) {
		/* Log of errors */
		if (readData[0] == EVT_ID_ERROR) {
			logError_st80y(1, "%s %s\n", tag_st80y, st80y_printHex("ERROR EVENT = ",
							     readData,
							     FIFO_EVENT_SIZE,
							     temp));
			memset(temp, 0, 128);
			count_err++;
			err_handling = st80y_errorHandler(info, readData, FIFO_EVENT_SIZE);
			if ((err_handling & 0xF0FF0000) ==
			    ERROR_HANDLER_STOP_PROC) {
				logError_st80y(1,
					 "%s forced to be stopped! ERROR %08X\n",
					 tag_st80y, err_handling);
				return err_handling;
			}
		} else {
			if (readData[0] != EVT_ID_NOEVENT) {
				logError_st80y(1, "%s %s\n", tag_st80y, st80y_printHex(
						 "READ EVENT = ", readData,
						 FIFO_EVENT_SIZE,
						 temp));
				memset(temp, 0, 128);
			}
			if (readData[0] == EVT_ID_CONTROLLER_READY &&
			    event_to_search[0] != EVT_ID_CONTROLLER_READY) {
				logError_st80y(1,
					 "%s Unmanned Controller Ready Event! Setting reset flags...\n",
					 tag_st80y);
				setSystemResetedUp(info, 1);
				setSystemResetedDown(info, 1);
			}
		}

		find = 1;

		for (i = 0; i < event_bytes; i++) {
			if (event_to_search[i] != -1 && (int)readData[i] !=
			    event_to_search[i]) {
				find = 0;
				break;
			}
		}

		retry++;
		msleep(TIMEOUT_RESOLUTION);
	}
	st80y_stopStopWatch(&clock);
	if ((retry >= time_to_count) && find != 1) {
		logError_st80y(1, "%s ERROR %08X\n", tag_st80y,
			 ERROR_TIMEOUT);
		return ERROR_TIMEOUT;
	} else if (find == 1) {
		/* temp = st80y_printHex("FOUND EVENT = ", readData, FIFO_EVENT_SIZE);
		 * */
		/* if (temp != NULL) */
		logError_st80y(0, "%s %s\n", tag_st80y, st80y_printHex("FOUND EVENT = ", readData,
						     FIFO_EVENT_SIZE, temp));
		memset(temp, 0, 128);
		/* kfree(temp); */
		logError_st80y(0,
			 "%s Event found in %d ms (%d iterations)! Number of errors found = %d\n",
			 tag_st80y, st80y_elapsedMillisecond(&clock), retry, count_err);
		return count_err;
	} else {
		logError_st80y(1, "%s ERROR %08X\n", tag_st80y, ERROR_BUS_R);
		return ERROR_BUS_R;
	}
}

/** @}*/

/**
  * Check that the FW sent the echo even after a command was sent
  * @param cmd pointer to an array of byte which contain the command previously
  *sent
  * @param size size of cmd
  * @return OK if success or an error code which specify the type of error
  */
int st80y_checkEcho(struct fts_ts_info *info, u8 *cmd, int size)
{
	int ret, i;
	int event_to_search[FIFO_EVENT_SIZE];
	u8 readData[FIFO_EVENT_SIZE];


	if (size < 1) {
		logError_st80y(1,
			 "%s st80y_checkEcho: Invalid Size = %d not valid! ERROR %08X\n",
			 tag_st80y,
			 size, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	} else {
		if ((size + 4) > FIFO_EVENT_SIZE)
			size = FIFO_EVENT_SIZE - 4;
		/* Echo event 0x43 0x01 xx xx xx xx xx fifo_status
		  * therefore command with more than 4 bytes will be trunked */

		event_to_search[0] = EVT_ID_STATUS_UPDATE;
		event_to_search[1] = EVT_TYPE_STATUS_ECHO;
		for (i = 2; i < size + 2; i++)
			event_to_search[i] = cmd[i - 2];
		if ((cmd[0] == FTS_CMD_SYSTEM) && (cmd[1] == SYS_CMD_SPECIAL) &&
			((cmd[2] == SPECIAL_FULL_PANEL_INIT) ||
			(cmd[2] == SPECIAL_PANEL_INIT)))
			ret = st80y_pollForEvent(info, event_to_search, size + 2, readData,
				   TIMEOUT_ECHO_FPI);
		else if ((cmd[0] == FTS_CMD_SYSTEM) &&
			(cmd[1] == SYS_CMD_CX_TUNING))
			ret = st80y_pollForEvent(info, event_to_search, size + 2, readData,
				   TIMEOUT_ECHO_SINGLE_ENDED_SPECIAL_AUTOTUNE);
		else
			ret = st80y_pollForEvent(info, event_to_search, size + 2, readData,
				   TIEMOUT_ECHO);
		if (ret < OK) {
			logError_st80y(1,
				 "%s st80y_checkEcho: Echo Event not found! ERROR %08X\n",
				 tag_st80y,
				 ret);
			return ret | ERROR_CHECK_ECHO_FAIL;
		} else if (ret > OK) {
			logError_st80y(1,
				 "%s st80y_checkEcho: Echo Event found but with some error events before! num_error = %d\n",
				 tag_st80y, ret);
			return ERROR_CHECK_ECHO_FAIL;
		}

		logError_st80y(0, "%s ECHO OK!\n", tag_st80y);
		return ret;
	}
}


/** @addtogroup scan_mode
  * @{
  */
/**
  * Set a scan mode in the IC
  * @param mode scan mode to set; possible values @link scan_opt Scan Mode
  * Option @endlink
  * @param settings option for the selected scan mode
  * (for example @link active_bitmask Active Mode Bitmask @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int setScanMode(struct fts_ts_info *info, u8 mode, u8 settings)
{
	u8 cmd[3] = { FTS_CMD_SCAN_MODE, mode, settings };
	int ret, size = 3;

	logError_st80y(0, "%s %s: Setting scan mode: mode = %02X settings = %02X !\n",
		 tag_st80y, __func__, mode, settings);
	if (mode == SCAN_MODE_LOW_POWER)
		size = 2;
	ret = st80y_write(info, cmd, size);	/* use write instead of writeFw because
					 * can be called while the interrupt are
					 * enabled */
	if (ret < OK) {
		logError_st80y(1, "%s %s: write failed...ERROR %08X !\n", tag_st80y,
			 __func__, ret);
		return ret | ERROR_SET_SCAN_MODE_FAIL;
	}
	logError_st80y(0, "%s %s: Setting scan mode OK!\n", tag_st80y, __func__);
	return OK;
}
/** @}*/


/** @addtogroup feat_sel
  * @{
  */
/**
  * Set a feature and its option in the IC
  * @param feat feature to set; possible values @link feat_opt Feature Selection
  *Option @endlink
  * @param settings pointer to an array of byte which store the options for
  * the selected feature (for example the gesture mask to activate
  * @link gesture_opt Gesture IDs @endlink)
  * @param size in bytes of settings
  * @return OK if success or an error code which specify the type of error
  */
int setFeatures(struct fts_ts_info *info, u8 feat, u8 *settings, int size)
{
	u8 *cmd = NULL;
	int i = 0;
	int ret;
	cmd = (u8 *)kmalloc((2 + size) * sizeof(u8), GFP_KERNEL);
	
	if(cmd==NULL){
		logError_st80y(0, "%s %s: allock cmd error info...\n", tag_st80y, __func__);
		return -1;
	}
	logError_st80y(0, "%s %s: Setting feature: feat = %02X !\n", tag_st80y, __func__,
		 feat);
	cmd[0] = FTS_CMD_FEATURE;
	cmd[1] = feat;
	logError_st80y(0, "%s %s: Settings = ", tag_st80y, __func__);
	for (i = 0; i < size; i++) {
		cmd[2 + i] = settings[i];
		logError_st80y(0, "%02X ", settings[i]);
	}
	logError_st80y(0, "\n");
	ret = st80y_write(info, cmd, 2 + size);	/* use write instead of writeFw because
					 * can be called while the interrupts
					 * are enabled */
	if (ret < OK) {
		logError_st80y(1, "%s %s: write failed...ERROR %08X !\n", tag_st80y,
			 __func__, ret);
		kfree(cmd);
		cmd = NULL;
		return ret | ERROR_SET_FEATURE_FAIL;
	}
	logError_st80y(0, "%s %s: Setting feature OK!\n", tag_st80y, __func__);
	kfree(cmd);
	cmd = NULL;
	return OK;
}
/** @}*/

/** @addtogroup sys_cmd
  * @{
  */
/**
  * Write a system command to the IC
  * @param sys_cmd System Command to execute; possible values
  * @link sys_opt System Command Option @endlink
  * @param sett settings option for the selected system command
  * (@link sys_special_opt Special Command Option @endlink, @link ito_opt
  * ITO Test Option @endlink, @link load_opt Load Host Data Option @endlink)
  * @param size in bytes of settings
  * @return OK if success or an error code which specify the type of error
  */
int writeSysCmd(struct fts_ts_info *info, u8 sys_cmd, u8 *sett, int size)
{
	u8 *cmd = NULL;
	int ret;
	cmd = (u8 *)kmalloc((2 + size) * sizeof(u8), GFP_KERNEL);
	if(cmd == NULL){
		logError_st80y(0, "%s %s: allock cmd error info...\n", tag_st80y, __func__);
		return -1;
	}
	cmd[0] = FTS_CMD_SYSTEM;
	cmd[1] = sys_cmd;

	logError_st80y(0, "%s %s: Command = %02X %02X ", tag_st80y, __func__, cmd[0],
		 cmd[1]);
	for (ret = 0; ret < size; ret++) {
		cmd[2 + ret] = sett[ret];
		logError_st80y(0, "%02X ", cmd[2 + ret]);
	}
	logError_st80y(0, "\n");
	logError_st80y(0, "%s %s: Writing Sys command...\n", tag_st80y, __func__);
	if (sys_cmd != SYS_CMD_LOAD_DATA)
		ret = st80y_writeFwCmd(info, cmd, 2 + size);
	else {
		if (size >= 1)
			ret = requestSyncFrame(info, sett[0]);
		else {
			logError_st80y(1, "%s %s: No setting argument! ERROR %08X\n",
				 tag_st80y, __func__, ERROR_OP_NOT_ALLOW);
			kfree(cmd);
			cmd = NULL;
			return ERROR_OP_NOT_ALLOW;
		}
	}
	if (ret < OK)
		logError_st80y(1, "%s %s: ERROR %08X\n", tag_st80y, __func__, ret);

	else
		logError_st80y(0, "%s %s: FINISHED!\n", tag_st80y, __func__);
	kfree(cmd);
	cmd = NULL;
	return ret;
}
/** @}*/

/** @addtogroup system_info
  * @{
  */
/**
  * Initialize the System Info Struct with default values according to the error
  * found during the reading
  * @param i2cError 1 if there was an I2C error while reading the System Info
  * data from memory, other value if another error occurred
  * @return OK if success or an error code which specify the type of error
  */
int defaultSysInfo(struct fts_ts_info *info, int i2cError)
{
	int i;

	logError_st80y(0, "%s Setting default System Info...\n", tag_st80y);

	if (i2cError == 1) {
		info->systemInfo.u16_fwVer = 0xFFFF;
		info->systemInfo.u16_cfgProjectId = 0xFFFF;
		for (i = 0; i < RELEASE_INFO_SIZE; i++)
			info->systemInfo.u8_releaseInfo[i] = 0xFF;
		info->systemInfo.u16_cxVer = 0xFFFF;
	} else {
		info->systemInfo.u16_fwVer = 0x0000;
		info->systemInfo.u16_cfgProjectId = 0x0000;
		for (i = 0; i < RELEASE_INFO_SIZE; i++)
			info->systemInfo.u8_releaseInfo[i] = 0x00;
		info->systemInfo.u16_cxVer = 0x0000;
	}

	info->systemInfo.u8_scrRxLen = 0;
	info->systemInfo.u8_scrTxLen = 0;

	logError_st80y(0, "%s default System Info DONE!\n", tag_st80y);
	return OK;
}

/**
  * Read the System Info data from memory. System Info is loaded automatically
  * after every system reset.
  * @param request if 1, will be asked to the FW to reload the data, otherwise
  * attempt to read it directly from memory
  * @return OK if success or an error code which specify the type of error
  */
int readSysInfo(struct fts_ts_info *info, int request)
{
	int ret, i, index = 0;
	u8 sett = LOAD_SYS_INFO;
	u8 data[SYS_INFO_SIZE] = { 0 };
	char temp[256] = { 0 };

	if (request == 1) {
		logError_st80y(0, "%s %s: Requesting System Info...\n", tag_st80y,
			 __func__);

		ret = writeSysCmd(info, SYS_CMD_LOAD_DATA, &sett, 1);
		if (ret < OK) {
			logError_st80y(1,
				 "%s %s: error while writing the sys cmd ERROR %08X\n",
				 tag_st80y, __func__, ret);
			goto FAIL;
		}
	}

	logError_st80y(0, "%s %s: Reading System Info...\n", tag_st80y, __func__);
	ret = st80y_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R, BITS_16,
				ADDR_FRAMEBUFFER, data, SYS_INFO_SIZE,
				DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		logError_st80y(1,
			 "%s %s: error while reading the system data ERROR %08X\n",
			 tag_st80y,
			 __func__, ret);
		goto FAIL;
	}

	logError_st80y(0, "%s %s: Parsing System Info...\n", tag_st80y, __func__);

	if (data[0] != HEADER_SIGNATURE) {
		logError_st80y(1,
			 "%s %s: The Header Signature is wrong!  sign: %02X != %02X ERROR %08X\n",
			 tag_st80y, __func__, data[0], HEADER_SIGNATURE,
			 ERROR_WRONG_DATA_SIGN);
		ret = ERROR_WRONG_DATA_SIGN;
		goto FAIL;
	}


	if (data[1] != LOAD_SYS_INFO) {
		logError_st80y(1,
			 "%s %s: The Data ID is wrong!  ids: %02X != %02X ERROR %08X\n",
			 tag_st80y, __func__, data[3], LOAD_SYS_INFO,
			 ERROR_DIFF_DATA_TYPE);
		ret = ERROR_DIFF_DATA_TYPE;
		goto FAIL;
	}

	index += 4;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_apiVer_rev);
	index += 2;
	info->systemInfo.u8_apiVer_minor = data[index++];
	info->systemInfo.u8_apiVer_major = data[index++];
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_chip0Ver);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_chip0Id);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_chip1Ver);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_chip1Id);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_fwVer);
	index += 2;
	logError_st80y(1, "%s FW VER = %04X\n", tag_st80y, info->systemInfo.u16_fwVer);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_svnRev);
	index += 2;
	logError_st80y(1, "%s SVN REV = %04X\n", tag_st80y, info->systemInfo.u16_svnRev);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_cfgVer);
	index += 2;
	logError_st80y(1, "%s CONFIG VER = %04X\n", tag_st80y, info->systemInfo.u16_cfgVer);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_cfgProjectId);
	index += 2;
	logError_st80y(1, "%s CONFIG PROJECT ID = %04X\n", tag_st80y,
		 info->systemInfo.u16_cfgProjectId);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_cxVer);
	index += 2;
	logError_st80y(1, "%s CX VER = %04X\n", tag_st80y, info->systemInfo.u16_cxVer);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_cxProjectId);
	index += 2;
	logError_st80y(1, "%s CX PROJECT ID = %04X\n", tag_st80y,
		 info->systemInfo.u16_cxProjectId);
	info->systemInfo.u8_cfgAfeVer = data[index++];
	info->systemInfo.u8_cxAfeVer =  data[index++];
	info->systemInfo.u8_panelCfgAfeVer = data[index++];
	logError_st80y(1, "%s AFE VER: CFG = %02X - CX = %02X - PANEL = %02X\n", tag_st80y,
		 info->systemInfo.u8_cfgAfeVer, info->systemInfo.u8_cxAfeVer,
		 info->systemInfo.u8_panelCfgAfeVer);
	info->systemInfo.u8_protocol = data[index++];
	logError_st80y(1, "%s Protocol = %02X\n", tag_st80y, info->systemInfo.u8_protocol);
	/* index+= 1;
	 * reserved area */

	/* logError_st80y(1, "%s Die Info =  ", tag_st80y); */
	for (i = 0; i < DIE_INFO_SIZE; i++)
		info->systemInfo.u8_dieInfo[i] = data[index++];
	/* logError_st80y(1, "%02X ", info->systemInfo.u8_dieInfo[i]); */
	/* logError_st80y(1, "\n"); */
	logError_st80y(1, "%s %s\n", tag_st80y, st80y_printHex("Die Info =  ",
					      info->systemInfo.u8_dieInfo,
					      DIE_INFO_SIZE, temp));
	memset(temp, 0, 256);


	/* logError_st80y(1, "%s Release Info =  ", tag_st80y); */
	for (i = 0; i < RELEASE_INFO_SIZE; i++)
		info->systemInfo.u8_releaseInfo[i] = data[index++];
	/* logError_st80y(1, "%02X ", info->systemInfo.u8_releaseInfo[i]); */
	/* logError_st80y(1, "\n"); */

	logError_st80y(1, "%s %s\n", tag_st80y, st80y_printHex("Release Info =  ",
					      info->systemInfo.u8_releaseInfo,
					      RELEASE_INFO_SIZE, temp));
	memset(temp, 0, 256);

	st80y_u8ToU32(&data[index], &info->systemInfo.u32_fwCrc);
	index += 4;
	st80y_u8ToU32(&data[index], &info->systemInfo.u32_cfgCrc);
	index += 4;

	logError_st80y(1, "%s MP FLAG index = %02X\n", tag_st80y, index);
	info->systemInfo.u8_mpFlag = data[index++];
	logError_st80y(1, "%s MP FLAG = %02X\n", tag_st80y,
		 info->systemInfo.u8_mpFlag);

	index += 3 + 4; /* +3 remaining from mp flag address */
	index += 4; /* skip reserved area */

	info->systemInfo.u8_ssDetScanSet = data[index];
	logError_st80y(1, "%s SS Detect Scan Select = %d \n", tag_st80y,
		 info->systemInfo.u8_ssDetScanSet);
	index += 4;
	logError_st80y(1, "%s res index = %02X\n", tag_st80y, index);
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_scrResX);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_scrResY);
	index += 2;
	logError_st80y(1, "%s Screen Resolution = %d x %d\n", tag_st80y,
		 info->systemInfo.u16_scrResX, info->systemInfo.u16_scrResY);
	info->systemInfo.u8_scrTxLen = data[index++];
	logError_st80y(1, "%s TX Len = %d\n", tag_st80y, info->systemInfo.u8_scrTxLen);
	info->systemInfo.u8_scrRxLen = data[index++];
	logError_st80y(1, "%s RX Len = %d\n", tag_st80y, info->systemInfo.u8_scrRxLen);
	info->systemInfo.u8_keyLen = data[index++];
	logError_st80y(1, "%s Key Len = %d\n", tag_st80y, info->systemInfo.u8_keyLen);
	info->systemInfo.u8_forceLen = data[index++];
	logError_st80y(1, "%s Force Len = %d\n", tag_st80y, info->systemInfo.u8_forceLen);
	index += 8;	

	st80y_u8ToU32(&data[index], &info->systemInfo.u32_productionTimestamp);
	logError_st80y(1, "%s Production Timestamp = %08X\n",
	tag_st80y, info->systemInfo.u32_productionTimestamp);

	index += 32;	/* skip reserved area */

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_dbgInfoAddr);
	index += 2;

	index += 6;	/* skip reserved area */

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_msTchRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_msTchFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_msTchStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_msTchBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchTxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchTxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchTxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchTxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchRxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchRxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchRxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssTchRxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_keyRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_keyFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_keyStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_keyBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_frcRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_frcFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_frcStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_frcBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrTxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrTxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrTxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrTxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrRxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrRxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrRxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssHvrRxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxTxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxTxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxTxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxTxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxRxRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxRxFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxRxStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssPrxRxBaselineAddr);
	index += 2;

	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssDetRawAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssDetFilterAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssDetStrenAddr);
	index += 2;
	st80y_u8ToU16(&data[index], &info->systemInfo.u16_ssDetBaselineAddr);
	index += 2;

	logError_st80y(1, "%s Parsed %d bytes!\n", tag_st80y, index);


	if (index != SYS_INFO_SIZE) {
		logError_st80y(1, "%s %s: index = %d different from %d ERROR %08X\n",
			 tag_st80y, __func__, index, SYS_INFO_SIZE,
			 ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	logError_st80y(1, "%s System Info Read DONE!\n", tag_st80y);
	return OK;

FAIL:
	defaultSysInfo(info, st80y_isI2cError(ret));
	return ret;
}
/** @}*/


/**
  * Read data from the Config Memory
  * @param offset Starting address in the Config Memory of data to read
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param len number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int readConfig(struct fts_ts_info *info, u16 offset, u8 *outBuf, int len)
{
	int ret;
	u64 final_address = offset + ADDR_CONFIG_OFFSET;

	logError_st80y(0, "%s %s: Starting to read config memory at %08llX ...\n",
		 tag_st80y, __func__, final_address);
	ret = st80y_writeReadU8UX(info, FTS_CMD_CONFIG_R, BITS_16, final_address,
				outBuf, len, DUMMY_CONFIG);
	if (ret < OK) {
		logError_st80y(1,
			 "%s %s: Impossible to read Config Memory... ERROR %08X!\n",
			 tag_st80y,
			 __func__, ret);
		return ret;
	}

	logError_st80y(0, "%s %s: Read config memory FINISHED!\n", tag_st80y, __func__);
	return OK;
}

/**
  * Write data into the Config Memory
  * @param offset Starting address in the Config Memory where write the data
  * @param data pointer of a byte array which contain the data to write
  * @param len number of bytes to write
  * @return OK if success or an error code which specify the type of error
  */
int writeConfig(struct fts_ts_info *info, u16 offset, u8 *data, int len)
{
	int ret;
	u64 final_address = offset + ADDR_CONFIG_OFFSET;

	logError_st80y(0, "%s %s: Starting to write config memory at %08llX ...\n",
		 tag_st80y, __func__, final_address);
	ret = st80y_writeU8UX(info, FTS_CMD_CONFIG_W, BITS_16, final_address, data,
			    len);
	if (ret < OK) {
		logError_st80y(1,
			 "%s %s: Impossible to write Config Memory... ERROR %08X!\n",
			 tag_st80y, __func__, ret);
		return ret;
	}

	logError_st80y(0, "%s %s: Write config memory FINISHED!\n", tag_st80y, __func__);
	return OK;
}

/**
  * Disable the interrupt so the ISR of the driver can not be called
  * @return OK if success or an error code which specify the type of error
  */
int st80y_disableInterrupt(struct fts_ts_info *info)
{
	if (atomic_read(&info->disable_irq_count) == 0) {
		logError_st80y(0, "%s Executing Disable...\n", tag_st80y);
		disable_irq(info->client->irq);
		atomic_inc(&info->disable_irq_count);
	}
	logError_st80y(0, "%s Interrupt Disabled!\n", tag_st80y);
	return OK;
}

/**
  * Reset the disable_irq count
  * @return OK
  */
int fts_resetDisableIrqCount(struct fts_ts_info *info)
{
	atomic_set(&info->disable_irq_count, 0);
	return OK;
}

/**
  * Enable the interrupt so the ISR of the driver can be called
  * @return OK if success or an error code which specify the type of error
  */
int st80y_enableInterrupt(struct fts_ts_info *info)
{
	while (atomic_read(&info->disable_irq_count) > 0) {
		logError_st80y(0, "%s Executing Enable...\n", tag_st80y);
		enable_irq(info->client->irq);
		atomic_dec(&info->disable_irq_count);
	}

	return OK;
}

/**
  *	Check if there is a crc error in the IC which prevent the fw to run.
  *	@return  OK if no CRC error, or a number >OK according the CRC error
  * found
  */
int fts_crc_check(struct fts_ts_info *info)
{
	u8 val;
	u8 crc_status;
	int res;
	u8 error_to_search[6] = { EVT_TYPE_ERROR_CRC_CFG_HEAD,
				  EVT_TYPE_ERROR_CRC_CFG,
				  EVT_TYPE_ERROR_CRC_CX,
				  EVT_TYPE_ERROR_CRC_CX_HEAD,
				  EVT_TYPE_ERROR_CRC_CX_SUB,
				  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD };


	res = st80y_writeReadU8UX(info, FTS_CMD_HW_REG_R, ADDR_SIZE_HW_REG, ADDR_CRC,
				&val, 1, DUMMY_HW_REG);
	/* read 2 bytes because the first one is a dummy byte! */
	if (res < OK) {
		logError_st80y(1, "%s %s Cannot read crc status ERROR %08X\n", tag_st80y,
			 __func__, res);
		return res;
	}

	crc_status = val & CRC_MASK;
	if (crc_status != OK) {	/* CRC error if crc_status!=0 */
		logError_st80y(1, "%s %s CRC ERROR = %02X\n", tag_st80y, __func__,
			 crc_status);
		return CRC_CODE;
	}

	logError_st80y(1, "%s %s: Verifying if Config CRC Error...\n", tag_st80y, __func__);
	res = st80y_system_reset(info);
	if (res >= OK) {
		res = pollForErrorType(info, error_to_search, 2);
		if (res < OK) {
			logError_st80y(1, "%s %s: No Config CRC Error Found!\n", tag_st80y,
				 __func__);
			logError_st80y(1, "%s %s: Verifying if Cx CRC Error...\n",
				 tag_st80y, __func__);
			res = pollForErrorType(info, &error_to_search[2], 4);
			if (res < OK) {
				logError_st80y(1, "%s %s: No Cx CRC Error Found!\n",
					 tag_st80y, __func__);
				return OK;
			} else {
				logError_st80y(1,
					 "%s %s: Cx CRC Error found! CRC ERROR = %02X\n",
					 tag_st80y, __func__, res);
				return CRC_CX;
			}
		} else {
			logError_st80y(1,
				 "%s %s: Config CRC Error found! CRC ERROR = %02X\n",
				 tag_st80y, __func__, res);
			return CRC_CONFIG;
		}
	} else {
		logError_st80y(1,
			 "%s %s: Error while executing system reset! ERROR %08X\n",
			 tag_st80y,
			 __func__, res);
		return res;
	}

	return OK;
}

/**
  * Request a host data and use the sync method to understand when the FW load
  * it
  * @param type the type ID of host data to load (@link load_opt Load Host Data
  * Option  @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int requestSyncFrame(struct fts_ts_info *info, u8 type)
{
	u8 request[3] = { FTS_CMD_SYSTEM, SYS_CMD_LOAD_DATA, type };
	u8 readData[DATA_HEADER] = { 0 };
	int ret, retry = 0, retry2 = 0, time_to_count;
	int count, new_count;

	logError_st80y(0, "%s %s: Starting to get a sync frame...\n", tag_st80y, __func__);

	while (retry2 < RETRY_MAX_REQU_DATA) {
		logError_st80y(0, "%s %s: Reading count...\n", tag_st80y, __func__);

		ret = st80y_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R, BITS_16,
					ADDR_FRAMEBUFFER, readData, DATA_HEADER,
					DUMMY_FRAMEBUFFER);
		if (ret < OK) {
			logError_st80y(0,
				 "%s %s: Error while reading count! ERROR %08X\n",
				 tag_st80y,
				 __func__, ret | ERROR_REQU_DATA);
			ret |= ERROR_REQU_DATA;
			retry2++;
			continue;
		}

		if (readData[0] != HEADER_SIGNATURE)
			logError_st80y(1,
				 "%s %s: Invalid Signature while reading count! ERROR %08X\n",
				 tag_st80y, __func__, ret | ERROR_REQU_DATA);
		/*ret|=ERROR_REQU_DATA;
		  * retry2++;
		  * continue;*/

		count = (readData[3] << 8) | readData[2];
		new_count = count;
		logError_st80y(0, "%s %s: Base count = %d\n", tag_st80y, __func__, count);

		logError_st80y(0, "%s %s: Requesting frame %02X  attempt = %d\n",
			 tag_st80y, __func__,  type, retry2 + 1);
		ret = st80y_write(info, request, ARRAY_SIZE(request));
		if (ret >= OK) {
			logError_st80y(0, "%s %s: Polling for new count...\n", tag_st80y,
				 __func__);
			time_to_count = TIMEOUT_REQU_DATA / TIMEOUT_RESOLUTION;
			while (count == new_count && retry < time_to_count) {
				ret = st80y_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R,
							BITS_16,
							ADDR_FRAMEBUFFER,
							readData,
							DATA_HEADER,
							DUMMY_FRAMEBUFFER);
				if ((ret >= OK) && (readData[0] ==
						    HEADER_SIGNATURE) &&
				    (readData[1] == type))
					new_count = ((readData[3] << 8) |
						     readData[2]);
				else
					logError_st80y(0,
						 "%s %s: invalid Signature or can not read count... ERROR %08X\n",
						 tag_st80y, __func__, ret);
				retry++;
				msleep(TIMEOUT_RESOLUTION);
			}

			if (count == new_count) {
				logError_st80y(1,
					 "%s %s: New count not received! ERROR %08X\n",
					 tag_st80y, __func__, ERROR_TIMEOUT |
					 ERROR_REQU_DATA);
				ret = ERROR_TIMEOUT | ERROR_REQU_DATA;
			} else {
				logError_st80y(0,
					 "%s %s: New count found! count = %d! Frame ready!\n",
					 tag_st80y, __func__, new_count);
				return OK;
			}
		}
		retry2++;
	}
	logError_st80y(1, "%s %s: Request Data failed! ERROR %08X\n", tag_st80y, __func__,
		 ret);
	return ret;
}


/**
  * Save MP flag value into the flash
  * @param mpflag Value to write in the MP Flag field
  * @return OK if success or an error code which specify the type of error
  */
int saveMpFlag(struct fts_ts_info *info, u8 mpflag)
{
	int ret;
	u8 mpflag_raw = mpflag;
	logError_st80y(1, "%s %s: Saving MP Flag = %02X\n", tag_st80y, __func__, mpflag);
	ret = writeSysCmd(info, SYS_CMD_MP_FLAG, &mpflag, 1);
	if (ret < OK) {
		logError_st80y(1, "%s %s: Error while writing MP flag on ram... ERROR %08X\n",
			tag_st80y, __func__, ret);
		return ret;
	}

	mpflag =  SAVE_PANEL_CONF;
	ret = writeSysCmd(info, SYS_CMD_SAVE_FLASH, &mpflag, 1);
	if (ret < OK) {
		logError_st80y(1, "%s %s: Error while saving MP flag on flash... ERROR %08X\n",
			tag_st80y, __func__, ret);
		return ret;
	}

	ret = readSysInfo(info, 1);
	if (ret < OK) {
		logError_st80y(1, "%s %s: Error while refreshing SysInfo... ERROR %08X\n",
			tag_st80y, __func__, ret);
		return ret;
	}
	if (info->systemInfo.u8_mpFlag != mpflag_raw) {
		logError_st80y(1, "%s %s::MP flag: %02x read in SysInfo not match with write value:%02x",
			tag_st80y, __func__, info->systemInfo.u8_mpFlag, mpflag_raw);
		ret = -1;
		return ret;
	}
		
	logError_st80y(1, "%s %s: Saving MP Flag OK!\n", tag_st80y, __func__);
	return OK;
}
