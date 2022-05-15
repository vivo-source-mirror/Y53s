/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                     FTS Gesture Utilities				   *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsGesture.h
  * \brief Contains all the macro and prototypes to handle the Gesture Detection
  * features
  */


#ifndef FTS_GESTURE_H_
#define FTS_GESTURE_H_

#include "ftsHardware.h"

int st80y_updateGestureMask(struct fts_ts_info *info, u8 *mask, int size, int en);
int st80y_disableGesture(struct fts_ts_info *info, u8 *mask, int size);
int st80y_enableGesture(struct fts_ts_info *info, u8 *mask, int size);
int st80y_enterGestureMode(struct fts_ts_info *info, int reload);
int st80y_isAnyGestureActive(struct fts_ts_info *info);
int readGestureCoords_V2(struct fts_ts_info *info, u8 *event);
int getGestureCoords_V2(struct fts_ts_info *info, u16 **x, u16 **y);

#endif	/* ! _GESTURE_H_ */
