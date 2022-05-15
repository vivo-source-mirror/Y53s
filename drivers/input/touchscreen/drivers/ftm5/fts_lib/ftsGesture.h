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

void gesture_init_ftm5(struct fts_ts_info *info);
int updateGestureMask_ftm5(struct fts_ts_info *info, u8 *mask, int size, int en);
int disableGesture_ftm5(struct fts_ts_info *info, u8 *mask, int size);
int enableGesture_ftm5(struct fts_ts_info *info, u8 *mask, int size);
int enterGestureMode_ftm5(struct fts_ts_info *info, int reload);
int isAnyGestureActive_ftm5(struct fts_ts_info *info);
int readGestureCoords_V2(struct fts_ts_info *info, u8 *event);
int getGestureCoords_V2(struct fts_ts_info *info, u16 **x, u16 **y);

#endif	/* ! _GESTURE_H_ */
