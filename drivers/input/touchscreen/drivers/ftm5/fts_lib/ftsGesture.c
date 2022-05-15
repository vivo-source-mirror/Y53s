/*
  *
  **************************************************************************
  **                        STMicroelectronics				 **
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     FTS Gesture Utilities				**
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsGesture.c
  * \brief Contains all the functions and variable to handle the Gesture
  *Detection features
  */

#include "ftsSoftware.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsGesture.h"
#include "ftsIO.h"
#include "ftsTime.h"
#include "ftsTool.h"

void gesture_init_ftm5(struct fts_ts_info *info)
{
	mutex_init(&info->gestureMask_mutex_ftm5);
}
/**
  * Update the gesture mask stored in the driver and have to be used in gesture
  * mode
  * @param mask pointer to a byte array which store the gesture mask update
  * that want to be performed.
  * @param size dimension in byte of mask. This size can be <=
  * GESTURE_MASK_SIZE.
  * If size < GESTURE_MASK_SIZE the bytes of mask are considering continuos
  * and starting from the less significant byte.
  * @param en 0 = enable the gestures set in mask, 1 = disable the gestures set
  *in mask
  * @return OK if success or an error code which specify the type of error
  */
int updateGestureMask_ftm5(struct fts_ts_info *info, u8 *mask, int size, int en)
{
	u8 temp;
	int i;

	if (mask != NULL) {
		if (size <= GESTURE_MASK_SIZE) {
			if (en == FEAT_ENABLE) {
				mutex_lock(&info->gestureMask_mutex_ftm5);
				logError_ftm5(0,
					 "%s updateGestureMask_ftm5: setting gesture mask to enable...\n",
					 tag_ftm5);
				if (mask != NULL)
					for (i = 0; i < size; i++)
						info->gesture_mask[i] =
							info->gesture_mask[i] |
							mask[i];
				/* back up of the gesture enabled */
				info->refreshGestureMask = 1;
				logError_ftm5(0,
					 "%s updateGestureMask_ftm5: gesture mask to enable SET!\n",
					 tag_ftm5);
				mutex_unlock(&info->gestureMask_mutex_ftm5);
				return OK;
			} else if (en == FEAT_DISABLE) {
				mutex_lock(&info->gestureMask_mutex_ftm5);
				logError_ftm5(0,
					 "%s updateGestureMask_ftm5: setting gesture mask to disable...\n",
					 tag_ftm5);
				for (i = 0; i < size; i++) {
					temp = info->gesture_mask[i] ^ mask[i];
					/* enabled XOR disabled */
					info->gesture_mask[i] = temp &
							  info->gesture_mask[i];
					/* temp AND enabled
					  * disable the gestures that were
					  * enabled */
				}
				logError_ftm5(0,
					 "%s updateGestureMask_ftm5: gesture mask to disable SET!\n",
					 tag_ftm5);
				info->refreshGestureMask = 1;
				mutex_unlock(&info->gestureMask_mutex_ftm5);
				return OK;
			} else {
				logError_ftm5(1,
					 "%s updateGestureMask_ftm5: Enable parameter Invalid! %d != %d or %d ERROR %08X\n",
					 tag_ftm5, en, FEAT_DISABLE, FEAT_ENABLE,
					 ERROR_OP_NOT_ALLOW);
				return ERROR_OP_NOT_ALLOW;
			}
		} else {
			logError_ftm5(1,
				 "%s updateGestureMask_ftm5: Size not valid! %d > %d ERROR %08X\n",
				 tag_ftm5, size, GESTURE_MASK_SIZE,
				 ERROR_OP_NOT_ALLOW);
			return ERROR_OP_NOT_ALLOW;
		}
	} else {
		logError_ftm5(1, "%s updateGestureMask_ftm5: Mask NULL! ERROR %08X\n",
			 tag_ftm5, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Enable in the FW the gesture mask to be used in gesture mode
  * @param mask pointer to a byte array which store the gesture mask update
  * that want to be sent to the FW, if NULL, will be used info->gesture_mask
  * set previously without any changes.
  * @param size dimension in byte of mask. This size can be <=
  * GESTURE_MASK_SIZE.
  * If size < GESTURE_MASK_SIZE the bytes of mask are considering continuos and
  * starting from the less significant byte.
  * @return OK if success or an error code which specify the type of error
  */
int enableGesture_ftm5(struct fts_ts_info *info, u8 *mask, int size)
{
	int i, res;

	logError_ftm5(0, "%s Trying to enable gesture...\n", tag_ftm5);

	if (size <= GESTURE_MASK_SIZE) {
		mutex_lock(&info->gestureMask_mutex_ftm5);
		if (mask != NULL)
			for (i = 0; i < size; i++)
				info->gesture_mask[i] = info->gesture_mask[i] | mask[i];
		/* back up of the gesture enabled */

		res = setFeatures(info, FEAT_SEL_GESTURE, info->gesture_mask,
				  GESTURE_MASK_SIZE);
		if (res < OK) {
			logError_ftm5(1, "%s enableGesture_ftm5: ERROR %08X\n", tag_ftm5,
				 res);
			goto END;
		}

		logError_ftm5(0, "%s enableGesture_ftm5 DONE!\n", tag_ftm5);
		res = OK;

END:
		mutex_unlock(&info->gestureMask_mutex_ftm5);
		return res;
	} else {
		logError_ftm5(1,
			 "%s enableGesture_ftm5: Size not valid! %d > %d ERROR %08X\n",
			 tag_ftm5,
			 size, GESTURE_MASK_SIZE, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Disable in the FW the gesture mask to be used in gesture mode
  * @param mask pointer to a byte array which store the gesture mask update that
  *  want to be sent to the FW, if NULL, all the gestures will be disabled.
  * @param size dimension in byte of mask. This size can be <=
  * GESTURE_MASK_SIZE.
  * If size < GESTURE_MASK_SIZE the bytes of mask are considering continuos and
  * starting from the less significant byte.
  * @return OK if success or an error code which specify the type of error
  */
int disableGesture_ftm5(struct fts_ts_info *info, u8 *mask, int size)
{
	u8 temp;
	int i, res;
	u8 *pointer;


	logError_ftm5(0, "%s Trying to disable gesture...\n", tag_ftm5);


	if (size <= GESTURE_MASK_SIZE) {
		mutex_lock(&info->gestureMask_mutex_ftm5);
		if (mask != NULL) {
			for (i = 0; i < size; i++) {
				temp = info->gesture_mask[i] ^ mask[i];
				/* enabled mask XOR disabled mask */
				info->gesture_mask[i] = temp & info->gesture_mask[i];
				/* temp AND enabled
				  * disable the gestures that are specified and
				  *  previously enabled */
			}

			pointer = info->gesture_mask;
		} else {
			i = 0;	/* if NULL is passed disable all the possible
				 * gestures */
			pointer = (u8 *)&i;
		}

		res = setFeatures(info, FEAT_SEL_GESTURE, pointer, GESTURE_MASK_SIZE);
		if (res < OK) {
			logError_ftm5(1, "%s disableGesture_ftm5: ERROR %08X\n", tag_ftm5,
				 res);
			goto END;
		}

		logError_ftm5(0, "%s disableGesture_ftm5 DONE!\n", tag_ftm5);

		res = OK;

END:
		mutex_unlock(&info->gestureMask_mutex_ftm5);
		return res;
	} else {
		logError_ftm5(1,
			 "%s disableGesture_ftm5: Size not valid! %d > %d ERROR %08X\n",
			 tag_ftm5,
			 size, GESTURE_MASK_SIZE, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Perform all the steps required to put the chip in gesture mode
  * @param reload if set to 1, before entering in gesture mode it will re-enable
  *  in the FW the last defined gesture mask
  * @return OK if success or an error code which specify the type of error
  */
int enterGestureMode_ftm5(struct fts_ts_info *info, int reload)
{
	int res, ret;

	res = fts_disableInterrupt_ftm5(info);
	if (res < OK) {
		logError_ftm5(1, "%s enterGestureMode_ftm5: ERROR %08X\n", tag_ftm5, res |
			 ERROR_DISABLE_INTER);
		return res | ERROR_DISABLE_INTER;
	}

	if (reload == 1 || info->refreshGestureMask == 1) {
		res = enableGesture_ftm5(info, NULL, 0);
		if (res < OK) {
			logError_ftm5(1,
				 "%s enterGestureMode_ftm5: enableGesture_ftm5 ERROR %08X\n",
				 tag_ftm5,
				 res);
			goto END;
		}

		info->refreshGestureMask = 0;
	}

	res = setScanMode(info, SCAN_MODE_LOW_POWER, 0);
	if (res < OK) {
		logError_ftm5(1,
			 "%s enterGestureMode_ftm5: enter gesture mode ERROR %08X\n",
			 tag_ftm5,
			 res);
		goto END;
	}

	res = OK;
END:
	ret = fts_enableInterrupt_ftm5(info);
	if (ret < OK) {
		logError_ftm5(1,
			 "%s enterGestureMode_ftm5: fts_enableInterrupt_ftm5 ERROR %08X\n",
			 tag_ftm5,
			 res | ERROR_ENABLE_INTER);
		res |= ret | ERROR_ENABLE_INTER;
	}


	return res;
}

/**
  * Check if one or more Gesture IDs are currently enabled in info->gesture_mask
  * @return FEAT_ENABLE if one or more gesture ids are enabled, FEAT_DISABLE if
  * all the gesture ids are currently disabled
  */
int isAnyGestureActive_ftm5(struct fts_ts_info *info)
{
	int res = 0;

	while (res < (GESTURE_MASK_SIZE - 1) && info->gesture_mask[res] == 0)
		/* -1 because in any case the last gesture mask byte will
		  * be evaluated with the following if */
		res++;

	if (info->gesture_mask[res] != 0) {
		logError_ftm5(0,
			 "%s %s: Active Gestures Found! info->gesture_mask[%d] = %02X !\n",
			 tag_ftm5, __func__, res, info->gesture_mask[res]);
		return FEAT_ENABLE;
	} else {
		logError_ftm5(0, "%s %s: All Gestures Disabled!\n", tag_ftm5, __func__);
		return FEAT_DISABLE;
	}
}


/**
  * Read from the frame buffer the gesture coordinates pairs of the points draw
  * by an user when a gesture is detected
  * @param event pointer to a byte array which contains the gesture event
  * reported
  *  by the fw when a gesture is detected
  * @return OK if success or an error code which specify the type of error
  */
int readGestureCoords_V2(struct fts_ts_info *info, u8 *event)
{
	int i = 0;
	u64 address = 0;
	int res;

	u8 val[GESTURE_MAX_COORDS_PAIRS_REPORT * 4];

	/* the max coordinates to read are GESTURE_COORDS_REPORT_MAX*4
	  * (because each coordinate is a short(*2) and we have x and y) */


	if (event[0] == EVT_ID_USER_REPORT && event[1] ==
	    EVT_TYPE_USER_GESTURE) {
		address = (event[4] << 8) | event[3]; /* Offset in framebuff */
		info->gesture_coords_reported = event[5];	/* number of pairs
							 * coords reported */
		if (info->gesture_coords_reported > GESTURE_MAX_COORDS_PAIRS_REPORT) {
			logError_ftm5(1,
				 "%s %s:  FW reported more than %d points for the gestures! Decreasing to %d\n",
				 tag_ftm5, __func__, info->gesture_coords_reported,
				 GESTURE_MAX_COORDS_PAIRS_REPORT);
			info->gesture_coords_reported =
				GESTURE_MAX_COORDS_PAIRS_REPORT;
		}

		logError_ftm5(1, "%s %s: Offset: %08llX , coords pairs = %d\n", tag_ftm5,
			 __func__, address, info->gesture_coords_reported);

		res = ftm5_writeReadU8UX(info, FTS_CMD_FRAMEBUFFER_R, BITS_16, address,
					val, (info->gesture_coords_reported * 2 * 2),
					DUMMY_FRAMEBUFFER);
		/* *2 because each coord is made by 2 bytes,
		  * *2 because there are x and y */
		if (res < OK) {
			logError_ftm5(1,
				 "%s %s: Cannot read the coordinates! ERROR %08X\n",
				 tag_ftm5, __func__, res);
			info->gesture_coords_reported = ERROR_OP_NOT_ALLOW;
			return res;
		}

		/* all the points of the gesture are stored in val */
		for (i = 0; i < info->gesture_coords_reported; i++) {
			info->gesture_coordinates_x[i] =  (((u16)val[i * 2 + 1]) &
						     0x0F) << 8 |
						   (((u16)val[i * 2]) & 0xFF);
			info->gesture_coordinates_y[i] =
				(((u16)val[info->gesture_coords_reported * 2 + i * 2 + 1]) &
				 0x0F) << 8 |
				(((u16)val[info->gesture_coords_reported * 2 +
					   i * 2]) & 0xFF);
		}


		logError_ftm5(1, "%s %s: Reading Gesture Coordinates DONE!\n", tag_ftm5,
			 __func__);
		return OK;
	} else {
		logError_ftm5(1,
			 "%s %s: The event passsed as argument is invalid! ERROR %08X\n",
			 tag_ftm5, __func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}

/**
  * Return the coordinates of the points stored during the last detected gesture
  * @param x output parameter which will store the address of the array
  * containing the x coordinates
  * @param y output parameter which will store the address of the array
  * containing the y coordinates
  * @return the number of points (x,y) stored and therefore the size of the x
  * and y array returned.
  */
int getGestureCoords_V2(struct fts_ts_info *info, u16 **x, u16 **y)
{
	*x = info->gesture_coordinates_x;
	*y = info->gesture_coordinates_y;
	logError_ftm5(1,
		 "%s %s: Number of gesture coordinates pairs returned = %d\n",
		 tag_ftm5,
		 __func__, info->gesture_coords_reported);
	return info->gesture_coords_reported;
}
