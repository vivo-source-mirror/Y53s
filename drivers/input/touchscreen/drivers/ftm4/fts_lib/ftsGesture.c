/*

 **************************************************************************
 **                        STMicroelectronics 							 **
 **************************************************************************
 **                        marco.cali@st.com								 **
 **************************************************************************
 *                                                                        *
 *                     FTS Gesture Utilities								 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsSoftware.h"
#include "ftsError.h"
#include "ftsGesture.h"
#include "ftsIO.h"
#include "ftsTool.h"

void ftm4_InitGesture(struct fts_ts_info *info)
{
	mutex_init(&info->gesture.mutex);
}

int ftm4_updateGestureMask(struct fts_ts_info *info, u8 *mask, int size, int en)
{
	u8 temp;
	int i;

	if (mask != NULL) {
		if (size <= GESTURE_MASK_SIZE) {
			if (en == FEAT_ENABLE) {
				mutex_lock(&info->gesture.mutex);
				fts_err(info, "setting gesture mask to enable...\n");
				if (mask != NULL) {
					for (i = 0; i < size; i++) {
						info->gesture.gesture_mask[i] = info->gesture.gesture_mask[i] | mask[i];						/*back up of the gesture enabled */
					}
				}
				info->gesture.refreshGestureMask = 1;
				fts_err(info, "gesture mask to enable SET! \n");
				mutex_unlock(&info->gesture.mutex);
				return OK;
			}

			else if (en == FEAT_DISABLE) {
				mutex_lock(&info->gesture.mutex);
				fts_err(info, "setting gesture mask to disable...\n");
				for (i = 0; i < size; i++) {
					temp = info->gesture.gesture_mask[i] ^ mask[i];			/* enabled XOR disabled */
					info->gesture.gesture_mask[i] = temp & info->gesture.gesture_mask[i];	/* temp AND enabled				//disable the gestures that were enabled */
				}
				fts_err(info, "gesture mask to disable SET! \n");
				info->gesture.refreshGestureMask = 1;
				mutex_unlock(&info->gesture.mutex);
				return OK;
			} else {
				fts_err(info, "Enable parameter Invalid! %d != %d or ERROR %08X", FEAT_DISABLE, FEAT_ENABLE, ERROR_OP_NOT_ALLOW);
				return ERROR_OP_NOT_ALLOW;
			}
		} else {
			fts_err(info, "Size not valid! %d > %d\n", size, GESTURE_MASK_SIZE);
			return ERROR_OP_NOT_ALLOW;
		}
	} else {
		fts_err(info, "Mask NULL! ERROR %08X \n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}


}

int ftm4_enableGesture(struct fts_ts_info *info, u8 *mask, int size)
{
	u8 cmd[GESTURE_MASK_SIZE + 2];
	u8 readData[FIFO_EVENT_SIZE];
	int i, res;
	/*int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, 0x00, GESTURE_ENABLE };*/

	fts_err(info, "Trying to enable gesture... \n");
	cmd[0] = FTS_CMD_GESTURE_CMD;
	cmd[1] = GESTURE_ENABLE;

	if (size <= GESTURE_MASK_SIZE) {
		mutex_lock(&info->gesture.mutex);
		if (mask != NULL) {
			for (i = 0; i < size; i++) {
				cmd[i + 2] = mask[i];
				info->gesture.gesture_mask[i] = info->gesture.gesture_mask[i] | mask[i];						/*back up of the gesture enabled */
			}
			while (i < GESTURE_MASK_SIZE) {
				cmd[i + 2] = info->gesture.gesture_mask[i];
				i++;
			}
		} else {
			for (i = 0; i < GESTURE_MASK_SIZE; i++) {
				cmd[i + 2] = info->gesture.gesture_mask[i];
			}
		}

		res = ftm4_writeFwCmd(info, cmd, GESTURE_MASK_SIZE + 2);
		if (res < OK) {
			fts_err(info, "ERROR %08X \n", res);
			goto END;
		}
		/*
		res = ftm4_pollForEvent(event_to_search, 4, readData, GENERAL_TIMEOUT);
		if (res < OK) {
			fts_err(info, " ftm4_enableGesture: ftm4_pollForEvent ERROR %08X \n", res);
			goto END;
		}
		*/
		if (readData[4] != 0x00) {
			fts_err(info, "ERROR %08X \n", ERROR_GESTURE_ENABLE_FAIL);
			res = ERROR_GESTURE_ENABLE_FAIL;
			goto END;
		}

		fts_err(info, "DONE! \n");
		res = OK;

	END:
		mutex_unlock(&info->gesture.mutex);
		return res;
	} else {
		fts_err(info, "Size not valid! %d > %d\n", size, GESTURE_MASK_SIZE);
		return ERROR_OP_NOT_ALLOW;
	}

}


int ftm4_disableGesture(struct fts_ts_info *info, u8 *mask, int size)
{
	u8 cmd[2 + GESTURE_MASK_SIZE];
	u8 readData[FIFO_EVENT_SIZE];
	u8 temp;
	int i, res;
	int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, 0x00, GESTURE_DISABLE };

	fts_err(info, "Trying to disable gesture... \n");
	cmd[0] = FTS_CMD_GESTURE_CMD;
	cmd[1] = GESTURE_DISABLE;

	if (size <= GESTURE_MASK_SIZE) {
		mutex_lock(&info->gesture.mutex);
		if (mask != NULL) {
			for (i = 0; i < size; i++) {
				cmd[i + 2] = mask[i];
				temp = info->gesture.gesture_mask[i] ^ mask[i];			/* enabled XOR disabled */
				info->gesture.gesture_mask[i] = temp & info->gesture.gesture_mask[i];	/* temp AND enabled				//disable the gestures that were enabled */
			}
			while (i < GESTURE_MASK_SIZE) {
				/*cmd[i + 2] = info->gesture.gesture_mask[i];			//disable all the other gesture not specified */
				/*info->gesture.gesture_mask[i] = 0x00; */

				cmd[i + 2] = 0x00;				/*leave untouched the gestures not specified */

				i++;
			}
		} else {
			for (i = 0; i < GESTURE_MASK_SIZE; i++) {
				/*cmd[i + 2] = info->gesture.gesture_mask[i]; */
				cmd[i + 2] = 0xFF;			/*if NULL is passed disable all the possible gestures */
			}
		}

		res = ftm4_writeFwCmd(info, cmd, 2 + GESTURE_MASK_SIZE);
		if (res < OK) {
			fts_err(info, "ERROR %08X \n", res);
			goto END;
		}

		res = ftm4_pollForEvent(info, event_to_search, 4, readData, GENERAL_TIMEOUT);
		if (res < OK) {
			fts_err(info, "ERROR %08X \n", res);
			goto END;
		}

		if (readData[4] != 0x00) {
			fts_err(info, "ERROR %08X \n", ERROR_GESTURE_ENABLE_FAIL);
			res = ERROR_GESTURE_ENABLE_FAIL;
			goto END;
		}

		fts_err(info, "DONE! \n");

		res = OK;

	END:
		mutex_unlock(&info->gesture.mutex);
		return res;
	} else {
		fts_err(info, "Size not valid! %d > %d\n", size, GESTURE_MASK_SIZE);
		return ERROR_OP_NOT_ALLOW;
	}
}

int ftm4_startAddCustomGesture(struct fts_ts_info *info, u8 gestureID)
{
	u8 cmd[3] = { FTS_CMD_GESTURE_CMD, GESTURE_START_ADD,  gestureID };
	int res;
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, gestureID, GESTURE_START_ADD };

	res = ftm4_writeFwCmd(info, cmd, 3);
	if (res < OK) {
		fts_err(info, "Impossible to start adding custom gesture ID = %02X! ERROR %08X \n", gestureID, res);
		return res;
	}

	res = ftm4_pollForEvent(info, event_to_search, 4, readData, GENERAL_TIMEOUT);
	if (res < OK) {
		fts_err(info, "start add event not found! ERROR %08X \n", res);
		return res;
	}

	if (readData[2] != gestureID || readData[4] != 0x00) {			/*check of gestureID is redundant */
		fts_err(info, "start add event status not OK! ERROR %08X \n", readData[4]);
		return ERROR_GESTURE_START_ADD;
	}


	return OK;
}


int ftm4_finishAddCustomGesture(struct fts_ts_info *info, u8 gestureID)
{
	u8 cmd[3] = { FTS_CMD_GESTURE_CMD, GESTURE_FINISH_ADD,  gestureID };
	int res;
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, gestureID, GESTURE_FINISH_ADD };

	res = ftm4_writeFwCmd(info, cmd, 3);
	if (res < OK) {
		fts_err(info, "Impossible to finish adding custom gesture ID = %02X! ERROR %08X \n", gestureID, res);
		return res;
	}

	res = ftm4_pollForEvent(info, event_to_search, 4, readData, GENERAL_TIMEOUT);
	if (res < OK) {
		fts_err(info, "finish add event not found! ERROR %08X \n", res);
		return res;
	}

	if (readData[2] != gestureID || readData[4] != 0x00) {			/*check of gestureID is redundant */
		fts_err(info, "finish add event status not OK! ERROR %08X \n", readData[4]);
		return ERROR_GESTURE_FINISH_ADD;
	}

	return OK;

}

int ftm4_loadCustomGesture(struct fts_ts_info *info, u8 *template, u8 gestureID)
{
	int res, i;
	int remaining = GESTURE_CUSTOM_POINTS;
	int toWrite, offset = 0;
	u8 cmd[TEMPLATE_CHUNK + 5];
	int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, gestureID, GESTURE_DATA_ADD };
	u8 readData[FIFO_EVENT_SIZE];

	fts_err(info, "Starting adding custom gesture procedure... \n");

	res = ftm4_startAddCustomGesture(info, gestureID);
	if (res < OK) {
		fts_err(info, "unable to start adding procedure! ERROR %08X \n", res);
		return res;
	}

	cmd[0] = FTS_CMD_GESTURE_CMD;
	cmd[1] = GESTURE_DATA_ADD;
	cmd[2] = gestureID;
	while (remaining > 0) {
		if (remaining > TEMPLATE_CHUNK) {
			toWrite = TEMPLATE_CHUNK;
		} else {
			toWrite = remaining;
		}

		cmd[3] = toWrite;
		cmd[4] = offset;
		for (i = 0; i < toWrite; i++) {
			cmd[i + 5] = template[i];
		}

		res = ftm4_writeFwCmd(info, cmd, toWrite + 5);
		if (res < OK) {
			fts_err(info, "unable to start adding procedure! ERROR %08X \n", res);
			return res;
		}

		res = ftm4_pollForEvent(info, event_to_search, 4, readData, GENERAL_TIMEOUT);
		if (res < OK) {
			fts_err(info, "add event not found! ERROR %08X \n", res);
			return res;
		}

		if (readData[2] != gestureID || readData[4] != 0x00) {			/*check of gestureID is redundant */
			fts_err(info, "add event status not OK! ERROR %08X \n", readData[4]);
			return ERROR_GESTURE_DATA_ADD;
		}

		remaining -= toWrite;
		offset += toWrite / 2;
	}

	res = ftm4_finishAddCustomGesture(info, gestureID);
	if (res < OK) {
		fts_err(info, "unable to finish adding procedure! ERROR %08X \n", res);
		return res;
	}

	fts_err(info, "Adding custom gesture procedure DONE! \n");
	return OK;

}


int ftm4_reloadCustomGesture(struct fts_ts_info *info)
{
	int res, i;

	fts_err(info, "Starting reload Gesture Template... \n");

	for (i = 0; i < GESTURE_CUSTOM_NUMBER; i++) {
		if (info->gesture.custom_gesture_index[i] == 1) {
			res = ftm4_loadCustomGesture(info, info->gesture.custom_gestures[i], GESTURE_CUSTOM_OFFSET + i);
			if (res < OK) {
				fts_err(info, "Impossible to load custom gesture ID = %02X! ERROR %08X \n", GESTURE_CUSTOM_OFFSET + i, res);
				return res;
			}
		}
	}

	fts_err(info, "Reload Gesture Template DONE! \n");
	return OK;

}

int ftm4_enterGestureMode(struct fts_ts_info *info, int reload)
{
	u8 cmd = FTS_CMD_GESTURE_MODE;
	int res, ret;
	bool int_enabled = false;

	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled) {
		res = ftm4_disableInterrupt(info);
		if (res < OK) {
			fts_err(info, "ERROR %08X \n", res | ERROR_DISABLE_INTER);
			return res | ERROR_DISABLE_INTER;
		}
	}

	if (reload == 1 || info->gesture.refreshGestureMask == 1) {

		if (reload == 1) {
			res = ftm4_reloadCustomGesture(info);
			if (res < OK) {
				fts_err(info, "impossible reload custom gesture! ERROR %08X \n", res);
				goto END;
			}
		}

		/****** mandatory steps to set the correct gesture mask defined by the user ******/

		/*
		res = ftm4_disableGesture(NULL, 0);
		if (res < OK) {
			fts_err(info, " ftm4_enterGestureMode: ftm4_disableGesture ERROR %08X \n", res);
			goto END;
		}
		*/
		res = ftm4_enableGesture(info, NULL, 0);
		if (res < OK) {
			fts_err(info, "ERROR %08X \n", res);
			goto END;
		}
		
		info->gesture.refreshGestureMask = 0;
		/**********************************************************************************/
	}

	res = ftm4_writeFwCmd(info, &cmd, 1);
	if (res < OK) {
		fts_err(info, "enter gesture mode ERROR %08X \n", res);
		goto END;
	}

	res = OK;
END:
	if (int_enabled) {
		ret = ftm4_enableInterrupt(info);
		if (ret < OK) {
			fts_err(info, "ERROR %08X \n", res | ERROR_ENABLE_INTER);
			res |= ret | ERROR_ENABLE_INTER;
		}
	}

	return res;
}

int ftm4_addCustomGesture(struct fts_ts_info *info, u8 *data, int size, u8 gestureID)
{
	int index, res, i;

	index = gestureID - GESTURE_CUSTOM_OFFSET;

	fts_err(info, "Starting Custom Gesture Adding procedure...\n");
	if (size != GESTURE_CUSTOM_POINTS || (gestureID != GES_ID_CUST1 && gestureID != GES_ID_CUST2 && gestureID != GES_ID_CUST3 && gestureID != GES_ID_CUST4 && gestureID != GES_ID_CUST5)) {
		fts_err(info, "Invalid size (%d) or Custom GestureID (%02X)! ERROR %08X \n", size, gestureID, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	for (i = 0; i < GESTURE_CUSTOM_POINTS; i++) {
		info->gesture.custom_gestures[index][i] = data[i];
	}

	res = ftm4_loadCustomGesture(info, info->gesture.custom_gestures[index], gestureID);
	if (res < OK) {
		fts_err(info, "impossible to load the custom gesture! ERROR %08X \n", res);
		return res;
	}

	info->gesture.custom_gesture_index[index] = 1;
	fts_err(info, "Custom Gesture Adding procedure DONE!\n");
	return OK;
}

int ftm4_removeCustomGesture(struct fts_ts_info *info, u8 gestureID)
{
	int res, index;
	u8 cmd[3] = { FTS_CMD_GESTURE_CMD, GETURE_REMOVE_CUSTOM, gestureID };
	int event_to_search[4] = { EVENTID_GESTURE, EVENT_TYPE_ENB, gestureID, GETURE_REMOVE_CUSTOM };
	u8 readData[FIFO_EVENT_SIZE];

	index = gestureID - GESTURE_CUSTOM_OFFSET;

	fts_err(info, "Starting Custom Gesture Removing procedure...\n");
	if (gestureID != GES_ID_CUST1 && gestureID != GES_ID_CUST2 && gestureID != GES_ID_CUST3 && gestureID != GES_ID_CUST4 && gestureID != GES_ID_CUST5) {
		fts_err(info, "Invalid size or Custom GestureID (%02X)! ERROR %08X \n", gestureID, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	res = ftm4_writeFwCmd(info, cmd, 3);					/*when a gesture is removed, it is also disabled automatically */
	if (res < OK) {
		fts_err(info, "Impossible to remove custom gesture ID = %02X! ERROR %08X \n", gestureID, res);
		return res;
	}

	res = ftm4_pollForEvent(info, event_to_search, 4, readData, GENERAL_TIMEOUT);
	if (res < OK) {
		fts_err(info, "remove event not found! ERROR %08X \n", res);
		return res;
	}

	if (readData[2] != gestureID || readData[4] != 0x00) {			/*check of gestureID is redundant */
		fts_err(info, "remove event status not OK! ERROR %08X \n", readData[4]);
		return ERROR_GESTURE_REMOVE;
	}


	info->gesture.custom_gesture_index[index] = 0;
	fts_err(info, "Custom Gesture Remove procedure DONE!\n");
	return OK;

}

int ftm4_isAnyGestureActive(struct fts_ts_info *info)
{
	int res = 0;

	while (res < (GESTURE_MASK_SIZE - 1) && info->gesture.gesture_mask[res] == 0) {		/* -1 because in any case the last gesture mask byte will be evaluated with the following if */
		res++;
	}

	if (info->gesture.gesture_mask[res] != 0) {
		fts_err(info, "Active Gestures Found! info->gesture.gesture_mask[%d] = %02X !\n", res, info->gesture.gesture_mask[res]);
		return FEAT_ENABLE;
	} else {
		fts_err(info, "All Gestures Disabled!\n");
		return FEAT_DISABLE;
	}
}

int ftm4_gestureIDtoGestureMask(struct fts_ts_info *info, u8 id, u8 *mask)
{
	fts_err(info, "Index = %d Position = %d !\n", ((int)((id) / 8)), (id % 8));
	mask[((int)((id) / 8))] |= 0x01 << (id % 8);
	return OK;
}

