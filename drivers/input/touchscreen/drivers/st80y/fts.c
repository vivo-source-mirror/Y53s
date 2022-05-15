/*
  * fts.c
  *
  * FTS Capacitive touch screen controller (FingerTipS)
  *
  * Copyright (C) 2016, STMicroelectronics Limited.
  * Authors: AMG(Analog Mems Group)
  *
  *             marco.cali@st.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
  * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
  * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
  * THE
  * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
  */


/*!
  * \file fts.c
  * \brief It is the main file which contains all the most important functions
  * generally used by a device driver the driver
  */
#include <linux/device.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsGesture.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"



/**
  * Event handler installer helpers
  */
#define event_id(_e)		(EVT_ID_##_e >> 4)
#define handler_name(_h)	fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd) \
		(_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd))



#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

char tag_st80y[12] = "[ VIVO_TS ]\0";

static int fts_init_sensing(struct fts_ts_info *info);
static int fts_mode_handler(struct fts_ts_info *info, int force);
static int fts_chip_initialization(struct fts_ts_info *info, int init_type);


/***************************************** UTILITIES
  * (current fw_ver/conf_id, active mode, file fw_ver/conf_id)
  ***************************************************/
/**
  * File node to show on terminal external release version in Little Endian \n
  * (first the less significant byte) \n
  * cat appid	show the external release version of the FW running in the IC
  */
static ssize_t fts_appid_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int error;
	char temp[100];
	struct fts_ts_info *info = dev_get_drvdata(dev);

	error = snprintf(buf, PAGE_SIZE, "%s\n", st80y_printHex("EXT Release = ",
							  info->systemInfo.
							  u8_releaseInfo,
							  EXTERNAL_RELEASE_INFO_SIZE,
							  temp));

	return error;
}

/**
  * File node to show on terminal the mode that is active on the IC \n
  * cat mode_active		    to show the bitmask which indicate
  * the modes/features which are running on the IC in a specific instant of time
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1 = 1 byte in HEX format which represent the actual running scan mode
  * (@link scan_opt Scan Mode Options @endlink) \n
  * X2 = 1 byte in HEX format which represent the bitmask on which is running
  * the actual scan mode \n
  * X3X4 = 2 bytes in HEX format which represent a bitmask of the features that
  * are enabled at this moment (@link feat_opt Feature Selection Options
  * @endlink) \n
  * } = end byte
  * @see fts_mode_handler()
  */
static ssize_t fts_mode_active_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError_st80y(1, "%s Current mode active = %08X\n", tag_st80y, info->mode);
	return snprintf(buf, 14, "{ %08X }\n", info->mode);
}

/**
  * File node to show the fw_ver and config_id of the FW file
  * cat fw_file_test			show on the kernel log external release
  * of the FW stored in the fw file/header file
  */
static ssize_t fts_fw_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	Firmware fw;
	int ret;
	char temp[100] = { 0 };
	struct fts_ts_info *info = dev_get_drvdata(dev);

	fw.data = NULL;
	ret = st80y_readFwFile(info, PATH_FILE_FW, &fw, 0);

	if (ret < OK)
		logError_st80y(1, "%s Error during reading FW file! ERROR %08X\n",
			 tag_st80y, ret);
	else
		logError_st80y(1, "%s %s, size = %d bytes\n", tag_st80y, st80y_printHex(
				 "EXT Release = ", info->systemInfo.u8_releaseInfo,
				 EXTERNAL_RELEASE_INFO_SIZE, temp),
			 fw.data_size);

	kfree(fw.data);
	return 0;
}

/***************************************** FEATURES
 * ***************************************************/

/* TODO: edit this function according to the features policy to allow during
  * the screen on/off, following is shown an example but check always with ST
  * for more details */
/**
  * Check if there is any conflict in enable/disable a particular feature
  * considering the features already enabled and running
  * @param info pointer to fts_ts_info which contains info about the device
  * and its hw setup
  * @param feature code of the feature that want to be tested
  * @return OK if is possible to enable/disable feature, ERROR_OP_NOT_ALLOW
  * in case of any other conflict
  */
int st80y_check_feature_feasibility(struct fts_ts_info *info, unsigned int feature)
{
	int res = OK;

/* Example based on the status of the screen and on the feature
  * that is trying to enable */
	/*res=ERROR_OP_NOT_ALLOW;
	  * if(info->resume_bit ==0){
	  *      switch(feature){
	  #ifdef GESTURE_MODE
	  *              case FEAT_SEL_GESTURE:
	  *                      res = OK;
	  *              break;
	  #endif
	  *              default:
	  *                      logError_st80y(1,"%s %s: Feature not allowed in this
	  * operating mode! ERROR %08X\n",
	  *				tag_st80y,__func__,res);
	  *              break;
	  *
	  *      }
	  * }else{
	  *      switch(feature){
	  #ifdef GESTURE_MODE
	  *              case FEAT_SEL_GESTURE:
	  #endif
	  *              case FEAT__SEL_GLOVE: // glove mode can only activate
	  *during sense on
	  *                      res = OK;
	  *              break;
	  *
	  *              default:
	  *                      logError_st80y(1,"%s %s: Feature not allowed in this
	  *operating mode! ERROR %08X\n",tag_st80y,__func__,res);
	  *              break;
	  *
	  *      }
	  * }*/


/* Example based only on the feature that is going to be activated */
	switch (feature) {
	case FEAT_SEL_GESTURE:
		if (info->cover_enabled == 1) {
			res = ERROR_OP_NOT_ALLOW;
			logError_st80y(1,
				 "%s %s: Feature not allowed when in Cover mode! ERROR %08X\n",
				 tag_st80y, __func__, res);
			/* for example here can be placed a code for disabling
			 * the cover mode when gesture is activated */
		}
		break;

	case FEAT_SEL_GLOVE:
		if (info->gesture_enabled == 1) {
			res = ERROR_OP_NOT_ALLOW;
			logError_st80y(1,
				 "%s %s: Feature not allowed when Gestures enabled! ERROR %08X\n",
				 tag_st80y, __func__, res);
			/* for example here can be placed a code for disabling
			  * the gesture mode when cover is activated
			  * (that means that cover mode has
			  * an higher priority on gesture mode) */
		}
		break;

	default:
		logError_st80y(1, "%s %s: Feature Allowed!\n", tag_st80y, __func__);
	}

	return res;
}

#ifdef USE_ONE_FILE_NODE
/**
  * File node to enable some feature
  * echo XX 00/01 > feature_enable		to enable/disable XX
  * (possible values @link feat_opt Feature Selection Options @endlink) feature
  * cat feature_enable		to show the result of enabling/disabling process
  * echo XX 01/00 > feature_enable; cat feature_enable
  * to perform both actions stated before in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 =
  * no error) \n
  * } = end byte
  */
static ssize_t fts_feature_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	char *p = (char *)buf;
	unsigned int temp, temp2;
	int res = OK;

	if ((count - 2 + 1) / 3 != 1)
		logError_st80y(1,
			 "%s fts_feature_enable: Number of parameter wrong! %d > %d\n",
			 tag_st80y, (count - 2 + 1) / 3, 1);
	else {
		if (sscanf(p, "%02X %02X ", &temp, &temp2) == 2) {
			p += 3;
			res = st80y_check_feature_feasibility(info, temp);
			if (res >= OK) {
				switch (temp) {
		#ifdef GESTURE_MODE
				case FEAT_SEL_GESTURE:
					info->gesture_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Gesture Enabled = %d\n",
						 tag_st80y, info->gesture_enabled);
					break;
		#endif

		#ifdef GLOVE_MODE
				case FEAT_SEL_GLOVE:
					info->glove_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Glove Enabled = %d\n",
						 tag_st80y, info->glove_enabled);

					break;
		#endif

		#ifdef STYLUS_MODE
				case FEAT_SEL_STYLUS:
					info->stylus_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Stylus Enabled = %d\n",
						 tag_st80y, info->stylus_enabled);

					break;
		#endif

		#ifdef COVER_MODE
				case FEAT_SEL_COVER:
					info->cover_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Cover Enabled = %d\n",
						 tag_st80y, info->cover_enabled);

					break;
		#endif

		#ifdef CHARGER_MODE
				case FEAT_SEL_CHARGER:
					info->charger_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Charger Enabled = %d\n",
						 tag_st80y, info->charger_enabled);

					break;
		#endif

		#ifdef GRIP_MODE
				case FEAT_SEL_GRIP:
					info->grip_enabled = temp2;
					logError_st80y(1,
						 "%s fts_feature_enable: Grip Enabled = %d\n",
						 tag_st80y, info->grip_enabled);

					break;
		#endif

				default:
					logError_st80y(1,
						 "%s fts_feature_enable: Feature %08X not valid! ERROR %08X\n",
						 tag_st80y, temp, ERROR_OP_NOT_ALLOW);
					res = ERROR_OP_NOT_ALLOW;
				}
				info->feature_feasibility = res;
			}

			if (info->feature_feasibility >= OK)
				info->feature_feasibility = fts_mode_handler(info, 1);
			else
			logError_st80y(1,
				 "%s %s: Call echo XX 00/01 > feature_enable with a correct feature value (XX)! ERROR %08X\n",
				 tag_st80y, __func__, res);
		} else
			logError_st80y(1, "%s %s: Error when reading with sscanf!\n",
				tag_st80y, __func__);


	}
	return count;
}



static ssize_t fts_feature_enable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int size = (6 * 2) + 1, index = 0;
	u8 *all_strbuff = NULL;
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	if (info->feature_feasibility < OK)
		logError_st80y(1,
			 "%s %s: Call before echo XX 00/01 > feature_enable with a correct feature value (XX)! ERROR %08X\n",
			 tag_st80y, __func__, info->feature_feasibility);


	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);
	if (all_strbuff != NULL) {
		index += snprintf(&all_strbuff[index], 13, "{ %08X }",
				  info->feature_feasibility);
		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError_st80y(1,
			 "%s fts_feature_enable_show: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag_st80y, ERROR_ALLOC);

	info->feature_feasibility = ERROR_OP_NOT_ALLOW;
	return count;
}

#else


#ifdef GRIP_MODE
/**
  * File node to set the grip mode
  * echo 01/00 > grip_mode	to enable/disable glove mode \n
  * cat grip_mode		to show the status of the grip_enabled switch \n
  * echo 01/00 > grip_mode; cat grip_mode		to enable/disable grip
  *mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent the value
  * info->grip_enabled (1 = enabled; 0= disabled) \n
  * } = end byte
  */
static ssize_t fts_grip_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int size = (6 * 2) + 1, index = 0;
	u8 *all_strbuff = NULL;
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError_st80y(0, "%s %s: grip_enabled = %d\n", tag_st80y, __func__,
		 info->grip_enabled);

	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);
	if (all_strbuff != NULL) {
		index += snprintf(&all_strbuff[index], 13, "{ %08X }",
				  info->grip_enabled);

		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError_st80y(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag_st80y,
			 __func__, ERROR_ALLOC);

	return count;
}


static ssize_t fts_grip_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


	/* in case of a different elaboration of the input, just modify this
	 * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError_st80y(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag_st80y, __func__, (int)((count + 1) / 3), 1);
	else {
		if (sscanf(p, "%02X ", &temp) == 1) {
			p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
			res = st80y_check_feature_feasibility(info, FEAT_SEL_GRIP);
			if (res >= OK || temp == FEAT_DISABLE) {
				info->grip_enabled = temp;
				res = fts_mode_handler(info, 1);
				if (res < OK)
					logError_st80y(1,
						 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
						 tag_st80y, __func__, res);
			}
		} else
			logError_st80y(1, "%s %s: Error when reading with sscanf!\n",
				tag_st80y, __func__);
	}

	return count;
}
#endif

#ifdef GLOVE_MODE
/**
  * File node to set the glove mode
  * echo 01/00 > glove_mode	to enable/disable glove mode \n
  * cat glove_mode	to show the status of the glove_enabled switch \n
  * echo 01/00 > glove_mode; cat glove_mode	to enable/disable glove mode and
  *  see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent the of value
  * info->glove_enabled (1 = enabled; 0= disabled) \n
  * } = end byte
  */
static ssize_t fts_glove_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int size = (6 * 2) + 1, index = 0;
	u8 *all_strbuff = NULL;
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError_st80y(0, "%s %s: glove_enabled = %d\n", tag_st80y, __func__,
		 info->glove_enabled);

	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);
	if (all_strbuff != NULL) {
		index += snprintf(&all_strbuff[index], 13, "{ %08X }",
				  info->glove_enabled);

		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError_st80y(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag_st80y,
			 __func__, ERROR_ALLOC);

	return count;
}


static ssize_t fts_glove_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError_st80y(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag_st80y, __func__, (int)((count + 1) / 3), 1);
	else {
		if (sscanf(p, "%02X ", &temp) == 1) {
			p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
			res = st80y_check_feature_feasibility(info, FEAT_SEL_GLOVE);
			if (res >= OK || temp == FEAT_DISABLE) {
				info->glove_enabled = temp;
				res = fts_mode_handler(info, 1);
				if (res < OK)
					logError_st80y(1,
						 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
						 tag_st80y, __func__, res);
			}
		} else
			logError_st80y(1, "%s %s: Error when reading with sscanf!\n",
				tag_st80y, __func__);
	}

	return count;
}
#endif


#ifdef COVER_MODE
/* echo 01/00 > cover_mode     to enable/disable cover mode */
/* cat cover_mode	to show the status of the cover_enabled switch
 * (example output in the terminal = "AA00000001BB" if the switch is enabled) */
/* echo 01/00 > cover_mode; cat cover_mode	to enable/disable cover mode and
  * see the switch status in just one call */
/**
  * File node to set the cover mode
  * echo 01/00 > cover_mode	to enable/disable cover mode \n
  * cat cover_mode	to show the status of the cover_enabled switch \n
  * echo 01/00 > cover_mode; cat cover_mode	to enable/disable cover mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->cover_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte \n
  * NOTE: \n
  */
static ssize_t fts_cover_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int size = (6 * 2) + 1, index = 0;
	u8 *all_strbuff = NULL;
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError_st80y(0, "%s %s: cover_enabled = %d\n", tag_st80y, __func__,
		 info->cover_enabled);

	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);
	if (all_strbuff != NULL) {
		index += snprintf(&all_strbuff[index], 13, "{ %08X }",
				  info->cover_enabled);

		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError_st80y(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag_st80y,
			 __func__, ERROR_ALLOC);

	return count;
}


static ssize_t fts_cover_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError_st80y(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag_st80y, __func__, (int)((count + 1) / 3), 1);
	else {
		if (sscanf(p, "%02X ", &temp) == 1) {
			p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
			res = st80y_check_feature_feasibility(info, FEAT_SEL_COVER);
			if (res >= OK || temp == FEAT_DISABLE) {
				info->cover_enabled = temp;
				res = fts_mode_handler(info, 1);
				if (res < OK)
					logError_st80y(1,
						 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
						 tag_st80y, __func__, res);
			}
		} else
			logError_st80y(1, "%s %s: Error when reading with sscanf!\n",
				tag_st80y, __func__);
	}

	return count;
}
#endif

#ifdef STYLUS_MODE
/**
  * File node to enable the stylus report
  * echo 01/00 > stylus_mode		to enable/disable stylus mode \n
  * cat stylus_mode	to show the status of the stylus_enabled switch \n
  * echo 01/00 > stylus_mode; cat stylus_mode	to enable/disable stylus mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->stylus_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte
  */
static ssize_t fts_stylus_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int size = (6 * 2) + 1, index = 0;
	u8 *all_strbuff = NULL;
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError_st80y(0, "%s %s: stylus_enabled = %d\n", tag_st80y, __func__,
		 info->stylus_enabled);

	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);
	if (all_strbuff != NULL) {
		index += snprintf(&all_strbuff[index], 13, "{ %08X }",
				  info->stylus_enabled);

		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError_st80y(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag_st80y,
			 __func__, ERROR_ALLOC);

	return count;
}


static ssize_t fts_stylus_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError_st80y(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag_st80y, __func__, (int)((count + 1) / 3), 1);
	else {
		if (sscanf(p, "%02X ", &temp) == 1) {
			p += 3;
			info->stylus_enabled = temp;
		} else
			logError_st80y(1, "%s %s: Error when reading with sscanf!\n",
				tag_st80y, __func__);

	}

	return count;
}
#endif

#endif

/***************************************** PRODUCTION TEST
 * ***************************************************/

/**
  * File node to execute the Mass Production Test or to get data from the IC
  * (raw or ms/ss init data)
  * echo cmd > stm_fts_cmd	to execute a command \n
  * cat stm_fts_cmd	to show the result of the command \n
  * echo cmd > stm_fts_cmd; cat stm_fts_cmd	to execute and show the result
  * in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error_code (00000000 =
  * OK)\n
  * (optional) data = data coming from the command executed represented as HEX
  * string \n
  *                   Not all the command return additional data \n
  * } = end byte \n
  * \n
  * Possible commands (cmd): \n
  * - 00 = MP Test -> return error_code \n
  * - 01 = ITO Test -> return error_code \n
  * - 03 = MS Raw Test -> return error_code \n
  * - 04 = MS Init Data Test -> return error_code \n
  * - 05 = SS Raw Test -> return error_code \n
  * - 06 = SS Init Data Test -> return error_code \n
  * - 13 xx(optional) = Read 1 MS Raw Frame -> return additional data:
  * MS frame row after row. if xx = 1, will read LP frame \n
  * - 14 = Read MS Init Data -> return additional data: MS init data row after
  * row \n
  * - 15 xx(optional) = Read 1 SS Raw Frame -> return additional data: SS frame,
  * force channels followed by sense channels. If xx = 1, will read LP frame \n
  * - 16 = Read SS Init Data -> return additional data: SS Init data,
  * first IX for force and sense channels and then CX for force and sense
  * channels \n
  * - F0 = Perform a system reset -> return error_code \n
  * - F1 = Perform a system reset and reenable the sensing and the interrupt
  */
static ssize_t stm_fts_cmd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int n;
	char *p = (char *)buf;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	memset(info->typeOfComand, 0, CMD_STR_LEN * sizeof(u32));
	info->numberParameters = 0;
	logError_st80y(1, "%s\n", tag_st80y);
	for (n = 0; n < (count + 1) / 3; n++) {
		if (sscanf(p, "%02X ", &info->typeOfComand[n]) == 1) {
			p += 3;
			logError_st80y(1, "%s info->typeOfComand[%d] = %02X\n", tag_st80y, n,
			 info->typeOfComand[n]);
			info->numberParameters++;
		}
	}

	/* info->numberParameters = n; */
	logError_st80y(1, "%s Number of Parameters = %d\n", tag_st80y, info->numberParameters);
	return count;
}

static ssize_t stm_fts_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int res, j, doClean = 0, count = 0, index = 0;

	int size = (6 * 2) + 1;
	int init_type = SPECIAL_PANEL_INIT;
	u8 *all_strbuff = NULL;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	MutualSenseData compData;
	SelfSenseData comData;
	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;
	enum vts_sensor_test_result result;

	if (info->numberParameters >= 1) {
		res = st80y_disableInterrupt(info);
		if (res < 0) {
			logError_st80y(0, "%s st80y_disableInterrupt: ERROR %08X\n",
				 tag_st80y, res);
			res = (res | ERROR_DISABLE_INTER);
			goto END;
		}

		switch (info->typeOfComand[0]) {
		/*ITO TEST*/
		case 0x01:
			res = st80y_production_test_ito(info, LIMITS_FILE, &info->tests_st80y);
			break;
		/*PRODUCTION TEST*/
		case 0x00:
#ifndef COMPUTE_INIT_METHOD
			if (info->systemInfo.u8_cfgAfeVer != info->systemInfo.u8_cxAfeVer) {
				res = ERROR_OP_NOT_ALLOW;
				logError_st80y(0,
					 "%s Miss match in CX version! MP test not allowed with wrong CX memory! ERROR %08X\n",
					 tag_st80y, res);
				break;
			}
#else
			if (info->systemInfo.u8_mpFlag != MP_FLAG_FACTORY) {
				init_type = SPECIAL_FULL_PANEL_INIT;
				logError_st80y(0,
					"%s Select Full Panel Init!\n", tag_st80y);
			} else {
				init_type = NO_INIT;
				logError_st80y(0,
					"%s Skip Full Panel Init!\n", tag_st80y);
			}
#endif

			res = st80y_production_test_main(info, LIMITS_FILE, 1, init_type,
						   &info->tests_st80y, MP_FLAG_FACTORY, &result);
			break;
		/*read mutual raw*/
		case 0x13:
			logError_st80y(0, "%s Get 1 MS Frame\n", tag_st80y);
			if (info->numberParameters >= 2 && info->typeOfComand[1] == 1)
				setScanMode(info, SCAN_MODE_LOCKED, LOCKED_LP_ACTIVE);
			else
				setScanMode(info, SCAN_MODE_LOCKED, LOCKED_ACTIVE);
			msleep(WAIT_FOR_FRESH_FRAMES);
			setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
			msleep(WAIT_AFTER_SENSEOFF);
			 st80y_flushFIFO(info);	/* delete the events related to some
					 * touch (allow to call this function
					 * while touching the screen without
					 * having a flooding of the FIFO) */
			res = st80y_getMSFrame3(info, MS_RAW, &frameMS);
			if (res < 0)
				logError_st80y(0,
					 "%s Error while taking the MS frame... ERROR %08X\n",
					 tag_st80y, res);

			else {
				logError_st80y(0, "%s The frame size is %d words\n",
					 tag_st80y, res);
				size += (res * sizeof(short) + 2) * 2;
				/* set res to OK because if getMSFrame is */
				/* successful res = number of words read */
				res = OK;
					st80y_print_frame_short(
						"MS frame =",
						st80y_array1dTo2d_short(
							frameMS.node_data,
							frameMS.node_data_size,
							frameMS.header.
							sense_node),
						frameMS.header.force_node,
						frameMS.header.sense_node);
			}
			break;
		/*read self raw*/
		case 0x15:
			logError_st80y(0, "%s Get 1 SS Frame\n", tag_st80y);
			if (info->numberParameters >= 2 && info->typeOfComand[1] == 1)
				setScanMode(info, SCAN_MODE_LOCKED, LOCKED_LP_DETECT);
			else
				setScanMode(info, SCAN_MODE_LOCKED, LOCKED_ACTIVE);
			msleep(WAIT_FOR_FRESH_FRAMES);
			setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
			msleep(WAIT_AFTER_SENSEOFF);
			 st80y_flushFIFO(info);	/* delete the events related to some
					 * touch (allow to call this function
					 * while touching the screen without
					 * having a flooding of the FIFO) */
			if (info->numberParameters >= 2 && info->typeOfComand[1] == 1)
				res = st80y_getSSFrame3(info, SS_DETECT_RAW, &frameSS);
			else
				res = st80y_getSSFrame3(info, SS_RAW, &frameSS);

			if (res < OK)
				logError_st80y(0,
					 "%s Error while taking the SS frame... ERROR %08X\n",
					 tag_st80y, res);

			else {
				logError_st80y(0, "%s The frame size is %d words\n",
					 tag_st80y, res);
				size += (res * sizeof(short) + 2) * 2;
				/* set res to OK because if getMSFrame is */
				/* successful res = number of words read */
				res = OK;
				st80y_print_frame_short("SS force frame =",
						  st80y_array1dTo2d_short(
							  frameSS.force_data,
							  frameSS.
							  header.force_node, 1),
						  frameSS.header.force_node, 1);
				st80y_print_frame_short("SS sense frame =",
						  st80y_array1dTo2d_short(
							  frameSS.sense_data,
							  frameSS.
							  header.sense_node,
							  frameSS.
							  header.sense_node), 1,
						  frameSS.header.sense_node);
			}

			break;

		case 0x14:	/* read mutual comp data */
			logError_st80y(0, "%s Get MS Compensation Data\n", tag_st80y);
			res = st80y_readMutualSenseCompensationData(info, LOAD_CX_MS_TOUCH,
							      &compData);

			if (res < 0)
				logError_st80y(0,
					 "%s Error reading MS compensation data ERROR %08X\n",
					 tag_st80y, res);
			else {
				logError_st80y(0,
					 "%s MS Compensation Data Reading Finished!\n",
					 tag_st80y);
				size += ((compData.node_data_size + 3) *
					 sizeof(u8)) * 2;
				print_frame_i8("MS Data (Cx2) =",
					       array1dTo2d_i8(
						       compData.node_data,
						       compData.
						       node_data_size,
						       compData.header.
						       sense_node),
					       compData.header.force_node,
					       compData.header.sense_node);
			}
			break;

		case 0x16:	/* read self comp data */
			logError_st80y(0, "%s Get SS Compensation Data...\n", tag_st80y);
			res = st80y_readSelfSenseCompensationData(info, LOAD_CX_SS_TOUCH,
							    &comData);
			if (res < 0)
				logError_st80y(0,
					 "%s Error reading SS compensation data ERROR %08X\n",
					 tag_st80y, res);
			else {
				logError_st80y(0,
					 "%s SS Compensation Data Reading Finished!\n",
					 tag_st80y);
				size += ((comData.header.force_node +
					  comData.header.sense_node) * 2 + 8) *
					sizeof(u8) * 2;
				st80y_print_frame_u8("SS Data Ix2_fm = ",
					       st80y_array1dTo2d_u8(comData.ix2_fm,
							      comData.header.
							      force_node, 1),
					       comData.header.force_node, 1);
				print_frame_i8("SS Data Cx2_fm = ",
					       array1dTo2d_i8(comData.cx2_fm,
							      comData.header.
							      force_node, 1),
					       comData.header.force_node, 1);
				st80y_print_frame_u8("SS Data Ix2_sn = ",
					       st80y_array1dTo2d_u8(comData.ix2_sn,
							      comData.header.
							      sense_node,
							      comData.header.
							      sense_node), 1,
					       comData.header.sense_node);
				print_frame_i8("SS Data Cx2_sn = ",
					       array1dTo2d_i8(comData.cx2_sn,
							      comData.header.
							      sense_node,
							      comData.header.
							      sense_node), 1,
					       comData.header.sense_node);
			}
			break;

		case 0x03:	/* MS Raw DATA TEST */
			res = st80y_system_reset(info);
			if (res >= OK)
				res = st80y_production_test_ms_raw(info, LIMITS_FILE, 1,
							     &info->tests_st80y);
			break;

		case 0x04:	/* MS CX DATA TEST */
			res = st80y_system_reset(info);
			if (res >= OK)
				res = st80y_production_test_ms_cx(info, LIMITS_FILE, 1,
							    &info->tests_st80y);
			break;

		case 0x05:	/* SS RAW DATA TEST */
			res = st80y_system_reset(info);
			if (res >= OK)
				res = st80y_production_test_ss_raw(info, LIMITS_FILE, 1,
							     &info->tests_st80y);
			break;

		case 0x06:	/* SS IX CX DATA TEST */
			res = st80y_system_reset(info);
			if (res >= OK)
				res = st80y_production_test_ss_ix_cx(info, LIMITS_FILE, 1,
					&info->tests_st80y);
			break;


		case 0xF0:
		case 0xF1:	/* TOUCH ENABLE/DISABLE */
			doClean = (int)(info->typeOfComand[0] & 0x01);
			res = st80y_cleanUp(info, doClean);
			break;

		default:
			logError_st80y(1,
				 "%s COMMAND NOT VALID!! Insert a proper value ...\n",
				 tag_st80y);
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		doClean = fts_mode_handler(info, 1);
		if (info->typeOfComand[0] != 0xF0)
			doClean |=  st80y_enableInterrupt(info);
		if (doClean < 0)
			logError_st80y(0, "%s %s: ERROR %08X\n", tag_st80y, __func__,
				 (doClean | ERROR_ENABLE_INTER));
	} else {
		logError_st80y(1,
			 "%s NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n",
			 tag_st80y);
		res = ERROR_OP_NOT_ALLOW;
	}

END:
	/* here start the reporting phase, assembling the data
	  * to send in the file node */
	all_strbuff = (u8 *)kzalloc(size, GFP_KERNEL);

	snprintf(&all_strbuff[index], 11, "{ %08X", res);
	index += 10;

	if (res >= OK) {
		/*all the other cases are already fine printing only the res.*/
		switch (info->typeOfComand[0]) {
		case 0x13:
			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frameMS.header.force_node);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frameMS.header.sense_node);
			index += 2;

			for (j = 0; j < frameMS.node_data_size; j++) {
				snprintf(&all_strbuff[index], 5, "%02X%02X",
					 (frameMS.node_data[j] & 0xFF00) >> 8,
					 frameMS.node_data[j] & 0xFF);
				index += 4;
			}

			kfree(frameMS.node_data);
			break;

		case 0x15:
			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frameSS.header.force_node);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frameSS.header.sense_node);
			index += 2;

			/* Copying self raw data Force */
			for (j = 0; j < frameSS.header.force_node; j++) {
				snprintf(&all_strbuff[index], 5, "%02X%02X",
					 (frameSS.force_data[j] & 0xFF00) >> 8,
					 frameSS.force_data[j] & 0xFF);
				index += 4;
			}


			/* Copying self raw data Sense */
			for (j = 0; j < frameSS.header.sense_node; j++) {
				snprintf(&all_strbuff[index], 5, "%02X%02X",
					 (frameSS.sense_data[j] & 0xFF00) >> 8,
					 frameSS.sense_data[j] & 0xFF);
				index += 4;
			}

			kfree(frameSS.force_data);
			kfree(frameSS.sense_data);
			break;

		case 0x14:
			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)compData.header.force_node);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)compData.header.sense_node);
			index += 2;

			/* Cpying CX1 value */
			snprintf(&all_strbuff[index], 3, "%02X",
				 (compData.cx1) & 0xFF);
			index += 2;

			/* Copying CX2 values */
			for (j = 0; j < compData.node_data_size; j++) {
				snprintf(&all_strbuff[index], 3, "%02X",
					 (compData.node_data[j]) & 0xFF);
				index += 2;
			}

			kfree(compData.node_data);
			break;

		case 0x16:
			snprintf(&all_strbuff[index], 3, "%02X",
				 comData.header.force_node);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 comData.header.sense_node);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.f_ix1) & 0xFF);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.s_ix1) & 0xFF);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.f_cx1) & 0xFF);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.s_cx1) & 0xFF);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.f_ix0) & 0xFF);
			index += 2;

			snprintf(&all_strbuff[index], 3, "%02X",
				 (comData.s_ix0) & 0xFF);
			index += 2;

			/* Copying IX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(&all_strbuff[index], 3, "%02X",
					 comData.ix2_fm[j] & 0xFF);
				index += 2;
			}

			/* Copying IX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(&all_strbuff[index], 3, "%02X",
					 comData.ix2_sn[j] & 0xFF);
				index += 2;
			}

			/* Copying CX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(&all_strbuff[index], 3, "%02X",
					 comData.cx2_fm[j] & 0xFF);
				index += 2;
			}

			/* Copying CX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(&all_strbuff[index], 3, "%02X",
					 comData.cx2_sn[j] & 0xFF);
				index += 2;
			}

			kfree(comData.ix2_fm);
			kfree(comData.ix2_sn);
			kfree(comData.cx2_fm);
			kfree(comData.cx2_sn);
			break;

		default:
			break;
		}
	}

	snprintf(&all_strbuff[index], 3, " }");
	index += 2;


	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	info->numberParameters = 0;
	/* need to reset the number of parameters in order to wait the
	  * next command, comment if you want to repeat the last command sent
	  * just doing a cat */
	/* logError_st80y(0,"%s info->numberParameters = %d\n",tag_st80y, info->numberParameters); */
	kfree(all_strbuff);

	return count;
}

static DEVICE_ATTR(appid, (S_IRUGO), fts_appid_show, NULL);
static DEVICE_ATTR(mode_active, (S_IRUGO), fts_mode_active_show, NULL);
static DEVICE_ATTR(fw_file_test, (S_IRUGO), fts_fw_test_show, NULL);
static DEVICE_ATTR(stm_fts_cmd, (S_IRUGO | S_IWUSR | S_IWGRP), stm_fts_cmd_show,
		   stm_fts_cmd_store);
#ifdef USE_ONE_FILE_NODE
static DEVICE_ATTR(feature_enable, (S_IRUGO | S_IWUSR | S_IWGRP),
		   fts_feature_enable_show, fts_feature_enable_store);
#else


#ifdef GRIP_MODE
static DEVICE_ATTR(grip_mode, (S_IRUGO | S_IWUSR | S_IWGRP), fts_grip_mode_show,
		   fts_grip_mode_store);
#endif

#ifdef GLOVE_MODE
static DEVICE_ATTR(glove_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
		   fts_glove_mode_show, fts_glove_mode_store);
#endif

#ifdef COVER_MODE
static DEVICE_ATTR(cover_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
		   fts_cover_mode_show, fts_cover_mode_store);
#endif

#ifdef STYLUS_MODE
static DEVICE_ATTR(stylus_mode, (S_IRUGO | S_IWUSR | S_IWGRP),
		   fts_stylus_mode_show, fts_stylus_mode_store);
#endif

#endif

/*  /sys/devices/soc.0/f9928000.i2c/i2c-6/6-0049 */
static struct attribute *fts_attr_group[] = {
	&dev_attr_appid.attr,
	&dev_attr_mode_active.attr,
	&dev_attr_fw_file_test.attr,
	&dev_attr_stm_fts_cmd.attr,
#ifdef USE_ONE_FILE_NODE
	&dev_attr_feature_enable.attr,
#else

#ifdef GRIP_MODE
	&dev_attr_grip_mode.attr,
#endif

#ifdef GLOVE_MODE
	&dev_attr_glove_mode.attr,
#endif
#ifdef COVER_MODE
	&dev_attr_cover_mode.attr,
#endif
#ifdef STYLUS_MODE
	&dev_attr_stylus_mode.attr,
#endif

#endif
	NULL,
};

/**
  * Event Handler for no events (EVT_ID_NOEVENT)
  */
static void fts_nop_event_handler(struct fts_ts_info *info, unsigned
				  char *event, ktime_t kt)
{
	logError_st80y(1,
		 "%s %s Doing nothing for event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag_st80y, __func__, event[0], event[1], event[2], event[3],
		 event[4],
		 event[5], event[6], event[7]);
}

/**
  * Event handler for enter and motion events (EVT_ID_ENTER_POINT,
  * EVT_ID_MOTION_POINT )
  * report to the linux input system touches with their coordinated and
  * additional informations
  */
static void fts_enter_pointer_event_handler(struct fts_ts_info *info, unsigned
					    char *event, ktime_t kt)
{
	unsigned char touchId;
	unsigned int touch_condition = 1, tool = MT_TOOL_FINGER;
	int x, y, z, distance;
	u8 touchType;

	if (!info->resume_bit)
		goto no_report;

	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;

	x = (((int)event[3] & 0x0F) << 8) | (event[2]);
	y = ((int)event[4] << 4) | ((event[3] & 0xF0) >> 4);
	/* TODO: check with fw how they will report distance and pressure */
	z = PRESSURE_MAX;
	distance = 0;	/* if the tool is touching the display the distance
			 * should be 0 */

	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;

	switch (touchType) {
#ifdef STYLUS_MODE
	case TOUCH_TYPE_STYLUS:
		VTI("It is a stylus!");
		if (info->stylus_enabled == 1) {	/* if stylus_enabled is
							 * not ==1 it will be
							 * reported as normal
							 * touch */
			tool = MT_TOOL_PEN;
			touch_condition = 1;
			__set_bit(touchId, &info->stylus_id);
			break;
		}
#endif
	/* TODO: customer can implement a different strategy for each kind of
	 * touch */
	case TOUCH_TYPE_FINGER:
	/* logError_st80y(0, "%s  %s : It is a finger!\n",tag_st80y,__func__); */
	case TOUCH_TYPE_GLOVE:
	/* logError_st80y(0, "%s  %s : It is a glove!\n",tag_st80y,__func__); */
	case TOUCH_TYPE_PALM:
		/* logError_st80y(0, "%s  %s : It is a palm!\n",tag_st80y,__func__); */
		tool = MT_TOOL_FINGER;
		touch_condition = 1;
		__set_bit(touchId, &info->touch_id);
		break;


	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		touch_condition = 0;	/* need to hover */
		z = 0;	/* no pressure */
		__set_bit(touchId, &info->touch_id);
		distance = DISTANCE_MAX;	/* check with fw report the
						 * hovering distance */
		break;

	case TOUCH_TYPE_INVALID:
	default:
		VTI("Invalid touch type = %d ! No Report...", touchType);
		goto no_report;
	}

	vts_report_point_down(info->vtsdev, touchId, 0, x, y, z, z, false, NULL, 0, kt);

no_report:
	return;
}

/**
  * Event handler for leave event (EVT_ID_LEAVE_POINT )
  * Report to the linux input system that one touch left the display
  */
static void fts_leave_pointer_event_handler(struct fts_ts_info *info, unsigned
					    char *event, ktime_t kt)
{
	unsigned char touchId;
	unsigned int tool = MT_TOOL_FINGER;
	u8 touchType;

	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;

	switch (touchType) {
#ifdef STYLUS_MODE
	case TOUCH_TYPE_STYLUS:
		VTI("It is a stylus!");
		if (info->stylus_enabled == 1) {
			/* if stylus_enabled is not ==1 it will be reported as
			 * normal touch */
			tool = MT_TOOL_PEN;
			__clear_bit(touchId, &info->stylus_id);
			break;
		}
#endif

	case TOUCH_TYPE_FINGER:
	/* logError_st80y(0, "%s  %s : It is a finger!\n",tag_st80y,__func__); */
	case TOUCH_TYPE_GLOVE:
	/* logError_st80y(0, "%s  %s : It is a glove!\n",tag_st80y,__func__); */
	case TOUCH_TYPE_PALM:
	/* logError_st80y(0, "%s  %s : It is a palm!\n",tag_st80y,__func__); */
	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		__clear_bit(touchId, &info->touch_id);
		break;

	case TOUCH_TYPE_INVALID:
	default:
		VTI("Invalid touch type = %d ! No Report...", touchType);
		return;
	}

	vts_report_point_up(info->vtsdev, touchId, 0, 0 , 0, 0, 0, 0, kt);
}

/* EventId : EVT_ID_MOTION_POINT */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler
/* remap the motion event handler to the same function which handle the enter
 * event */

/**
  * Event handler for error events (EVT_ID_ERROR)
  * Handle unexpected error events implementing recovery strategy and
  * restoring the sensing status that the IC had before the error occured
  */
static void fts_error_event_handler(struct fts_ts_info *info, unsigned
				    char *event, ktime_t kt)
{
	int error = 0;

	logError_st80y(0,
		 "%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag_st80y,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);

	switch (event[1]) {
	case EVT_TYPE_ERROR_ESD:/* esd */
	{
		st80y_chip_powercycle(info);
		error = st80y_system_reset(info);
		error |= fts_mode_handler(info, 0);
		error |=  st80y_enableInterrupt(info);
		if (error < OK)
			logError_st80y(1,
				 "%s %s Cannot restore the device ERROR %08X\n",
				 tag_st80y, __func__, error);
	}
	break;

	case EVT_TYPE_ERROR_HARD_FAULT:	/* hard fault */
	case EVT_TYPE_ERROR_WATCHDOG:	/* watch dog timer */
	{
		st80y_dumpErrorInfo(info, NULL, 0);
		error = st80y_system_reset(info);
		error |= fts_mode_handler(info, 0);
		error |=  st80y_enableInterrupt(info);
		if (error < OK)
			logError_st80y(1,
				 "%s %s Cannot reset the device ERROR %08X\n",
				 tag_st80y, __func__, error);
	}
	break;
	}
}

/**
  * Event handler for controller ready event (EVT_ID_CONTROLLER_READY)
  * Handle controller events received after unexpected reset of the IC updating
  * the resets flag and restoring the proper sensing status
  */
static void fts_controller_ready_event_handler(struct fts_ts_info *info,
					       unsigned char *event, ktime_t kt)
{
	int error;

	logError_st80y(0,
		"%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag_st80y,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);
	setSystemResetedUp(info, 1);
	setSystemResetedDown(info, 1);
	error = fts_mode_handler(info, 0);
	if (error < OK)
		logError_st80y(1,
			 "%s %s Cannot restore the device status ERROR %08X\n",
			 tag_st80y,
			 __func__, error);
}

/**
  * Event handler for status events (EVT_ID_STATUS_UPDATE)
  * Handle status update events
  */
static void fts_status_event_handler(struct fts_ts_info *info, unsigned
				     char *event, ktime_t kt)
{
	switch (event[1]) {
	case EVT_TYPE_STATUS_ECHO:
		logError_st80y(1,
			 "%s %s Echo event of command = %02X %02X %02X %02X %02X %02X\n",
			 tag_st80y, __func__, event[2], event[3], event[4], event[5],
			 event[6],
			 event[7]);
		break;

	case EVT_TYPE_STATUS_FORCE_CAL:
		switch (event[2]) {
		case 0x00:
			logError_st80y(1,
				 "%s %s Continuous frame drop Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x01:
			logError_st80y(1,
				 "%s %s Mutual negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x02:
			logError_st80y(1,
				 "%s %s Mutual calib deviation Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x11:
			logError_st80y(1,
				 "%s %s SS negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x12:
			logError_st80y(1,
				 "%s %s SS negative detect Force cal in Low Power mode = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x13:
			logError_st80y(1,
				 "%s %s SS negative detect Force cal in Idle mode = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x20:
			logError_st80y(1,
				 "%s %s SS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x21:
			logError_st80y(1,
				 "%s %s SS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x22:
			logError_st80y(1,
				 "%s %s SS invalid Self Island soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x30:
			logError_st80y(1,
				 "%s %s MS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x31:
			logError_st80y(1,
				 "%s %s MS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		default:
			logError_st80y(1,
				 "%s %s Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_FRAME_DROP:
		switch (event[2]) {
		case 0x01:
			logError_st80y(1,
				 "%s %s Frame drop noisy frame = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x02:
			logError_st80y(1,
				 "%s %s Frame drop bad R = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x03:
			logError_st80y(1,
				 "%s %s Frame drop invalid processing state = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		default:
			logError_st80y(1,
				 "%s %s Frame drop = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_SS_RAW_SAT:
		if (event[2] == 1)
			logError_st80y(1,
				 "%s %s SS Raw Saturated = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		else
			logError_st80y(1,
				 "%s %s SS Raw No more Saturated = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		break;

	case EVT_TYPE_STATUS_WATER:
		if (event[2] == 1)
			logError_st80y(1,
				 "%s %s Enter Water mode = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		else
			logError_st80y(1,
				 "%s %s Exit Water mode = %02X %02X %02X %02X %02X %02X\n",
				 tag_st80y, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		break;

	default:
		logError_st80y(1,
			 "%s %s Received unhandled status event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			 tag_st80y, __func__, event[0], event[1], event[2], event[3],
			 event[4],
			 event[5], event[6], event[7]);
		break;
	}
}


/* key events reported in the user report */
#ifdef PHONE_KEY
/* TODO: the customer should handle the events coming from the keys according
 * his needs (this is just an sample code that report the click of a botton
 * after a press->release action) */
/**
  * Event handler for status events (EVT_TYPE_USER_KEY)
  * Handle keys update events, the third byte of the event is a bitmask where if
  *the bit set means that the corresponding key is pressed.
  */
static void fts_key_event_handler(struct fts_ts_info *info, unsigned
				  char *event, ktime_t kt)
{
	/* int value; */
	logError_st80y(0,
		 "%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag_st80y,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);

	if (event[0] == EVT_ID_USER_REPORT && event[1] == EVT_TYPE_USER_KEY) {
		/* event[2] contain the bitmask of the keys that are actually
		 * pressed */

		if ((event[2] & FTS_KEY_0) == 0 && (info->key_mask & FTS_KEY_0) > 0) {
			logError_st80y(0,
				 "%s %s: Button HOME pressed and released!\n",
				 tag_st80y,
				 __func__);
			vts_report_event_down(info->vtsdev, KEY_HOMEPAGE);
			vts_report_event_up(info->vtsdev, KEY_HOMEPAGE);
		}

		if ((event[2] & FTS_KEY_1) == 0 && (info->key_mask & FTS_KEY_1) > 0) {
			logError_st80y(0,
				 "%s %s: Button Back pressed and released!\n",
				 tag_st80y,
				 __func__);
			vts_report_event_down(info->vtsdev, KEY_BACK);
			vts_report_event_up(info->vtsdev, KEY_BACK);
		}

		if ((event[2] & FTS_KEY_2) == 0 && (info->key_mask & FTS_KEY_2) > 0) {
			logError_st80y(0, "%s %s: Button Menu pressed!\n", tag_st80y,
				 __func__);
			vts_report_event_down(info->vtsdev, KEY_MENU);
			vts_report_event_up(info->vtsdev, KEY_MENU);
		}

		info->key_mask = event[2];
	} else
		logError_st80y(1, "%s %s: Invalid event passed as argument!\n", tag_st80y,
			 __func__);
}
#endif

/* gesture event must be handled in the user event handler */
#ifdef GESTURE_MODE
/* TODO: Customer should implement their own actions in respond of a gesture
 * event. This is an example that simply print the gesture received and simulate
 * the click on a different button for each gesture. */
/**
  * Event handler for gesture events (EVT_TYPE_USER_GESTURE)
  * Handle gesture events and simulate the click on a different button for any
  *gesture detected (@link gesture_opt Gesture IDs @endlink)
  */
static void fts_gesture_event_handler(struct fts_ts_info *info, unsigned
				      char *event, ktime_t kt)
{
	int value;
	u16 *x;
	u16 *y;
	int needCoords = 0;
	int fod_detec = 0;
	int nr_points = 0;
	
	logError_st80y(0,
		 "%s  gesture event data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag_st80y, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);



	if (event[0] == EVT_ID_USER_REPORT && event[1] ==
	    EVT_TYPE_USER_GESTURE) {
		needCoords = 1;
		/* default read the coordinates for all gestures excluding
		 * double tap */

		switch (event[2]) {
		case GEST_ID_DBLTAP:
			value = VTS_EVENT_GESTURE_DOUBLE_CLICK;
			logError_st80y(0, "%s %s: double tap !\n", tag_st80y, __func__);
			needCoords = 0;
			break;

		case GEST_ID_AT:
			value = VTS_EVENT_GESTURE_PATTERN_A;
			logError_st80y(0, "%s %s: @ !\n", tag_st80y, __func__);
			break;

		case GEST_ID_C:
			value = VTS_EVENT_GESTURE_PATTERN_C;
			logError_st80y(0, "%s %s: C !\n", tag_st80y, __func__);
			break;

		case GEST_ID_E:
			value = VTS_EVENT_GESTURE_PATTERN_E;
			logError_st80y(0, "%s %s: e !\n", tag_st80y, __func__);
			break;

		case GEST_ID_F:
			value = VTS_EVENT_GESTURE_PATTERN_F;
			logError_st80y(0, "%s %s: F !\n", tag_st80y, __func__);
			break;


		case GEST_ID_M:
			value = VTS_EVENT_GESTURE_PATTERN_M;
			logError_st80y(0, "%s %s: M !\n", tag_st80y, __func__);
			break;

		case GEST_ID_O:
			value = VTS_EVENT_GESTURE_PATTERN_O;
			logError_st80y(0, "%s %s: O !\n", tag_st80y, __func__);
			break;

		case GEST_ID_V:
			value = VTS_EVENT_GESTURE_PATTERN_V;
			logError_st80y(0, "%s %s:  V !\n", tag_st80y, __func__);
			break;

		case GEST_ID_W:
			value = VTS_EVENT_GESTURE_PATTERN_W;
			logError_st80y(0, "%s %s:  W !\n", tag_st80y, __func__);
			break;


		case GEST_ID_RIGHT_1F:
			value = VTS_EVENT_GESTURE_PATTERN_RIGHT;
			logError_st80y(0, "%s %s:  -> !\n", tag_st80y, __func__);
			break;

		case GEST_ID_LEFT_1F:
			value = VTS_EVENT_GESTURE_PATTERN_LEFT;
			logError_st80y(0, "%s %s:  <- !\n", tag_st80y, __func__);
			break;

		case GEST_ID_UP_1F:
			value = VTS_EVENT_GESTURE_PATTERN_UP;
			logError_st80y(0, "%s %s:  UP !\n", tag_st80y, __func__);
			break;

		case GEST_ID_DOWN_1F:
			value = VTS_EVENT_GESTURE_PATTERN_DOWN;
			logError_st80y(0, "%s %s:  DOWN !\n", tag_st80y, __func__);
			break;
		case GEST_ID_FP_DOWN:
			fod_detec = 1; 
			value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
			logError_st80y(0, "%s %s:  FD DOWN !\n", tag_st80y, __func__);
			break;
		case GEST_ID_FP_UP:
			fod_detec = 2; 
			value = VTS_EVENT_GESTURE_FINGERPRINT_DETECT;
			logError_st80y(0, "%s %s:  FD UP !\n", tag_st80y, __func__);
			break;
			
		default:
			logError_st80y(0, "%s %s:  No valid GestureID!\n", tag_st80y,
				 __func__);
			goto gesture_done;
		}

		if (needCoords == 1) {
			if (readGestureCoords_V2(info, event) == OK) {
				
				nr_points = getGestureCoords_V2(info, &x, &y);
				if (nr_points >= OK)
					vts_report_coordinates_set(info->vtsdev, x, y, nr_points);
				else
					VTE("get gesture coords failed!");
			} else {
				VTE("read gesture coords failed!");
			}
		}
		if(fod_detec) {
			VTI("enter fd point get");
			if (readGestureCoords_V2(info, event) == OK) {
				nr_points = getGestureCoords_V2(info, &x, &y);
			}
			if(fod_detec == 1)
				vts_report_event_down(info->vtsdev, value);
			else
				vts_report_event_up(info->vtsdev, value);
			
		}else{
			vts_report_event_down(info->vtsdev, value);
			vts_report_event_up(info->vtsdev, value);
		}

gesture_done:
		return;
	} else
		VTE("nvalid event passed as argument!");
}
#endif


/**
  * Event handler for user report events (EVT_ID_USER_REPORT)
  * Handle user events reported by the FW due to some interaction triggered
  * by an external user (press keys, perform gestures, etc.)
  */
static void fts_user_report_event_handler(struct fts_ts_info *info, unsigned
					  char *event, ktime_t kt)
{
	switch (event[1]) {
#ifdef PHONE_KEY
	case EVT_TYPE_USER_KEY:
		fts_key_event_handler(info, event);
		break;
#endif

	case EVT_TYPE_USER_PROXIMITY:
		if (event[2] == 0)
			logError_st80y(1, "%s %s No proximity!\n", tag_st80y, __func__);
		else
			logError_st80y(1, "%s %s Proximity Detected!\n", tag_st80y,
				 __func__);
		break;

#ifdef GESTURE_MODE
	case EVT_TYPE_USER_GESTURE:
		fts_gesture_event_handler(info, event, kt);
		break;
#endif
	default:
		logError_st80y(1,
			 "%s %s Received unhandled user report event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			 tag_st80y, __func__, event[0], event[1], event[2], event[3],
			 event[4],
			 event[5], event[6], event[7]);
		break;
	}
}

/**
  * Bottom Half Interrupt Handler function
  * This handler is called each time there is at least one new event in the FIFO
  * and the interrupt pin of the IC goes low. It will read all the events from
  * the FIFO and dispatch them to the proper event handler according the event
  * ID
  */
static void fts_event_handler(struct fts_ts_info *info, ktime_t kt)
{
	int error = 0, count = 0;
	unsigned char regAdd;
	unsigned char data[FIFO_EVENT_SIZE] = { 0 };
	unsigned char eventId;

	event_dispatch_handler_t event_handler;

	/* read the FIFO and parsing events */


	regAdd = FIFO_CMD_READONE;

	for (count = 0; count < FIFO_DEPTH; count++) {
		error = st80y_writeReadU8UX(info, regAdd, 0, 0, data, FIFO_EVENT_SIZE,
					  DUMMY_FIFO);

		logError_st80y(0, "%s %s event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			tag_st80y, __func__, data[0],
			data[1], data[2], data[3], data[4], data[5], data[6],
			data[7]);
		if (error == OK && data[0] != EVT_ID_NOEVENT)
			eventId = data[0] >> 4;
		else
			break;
		/* if(data[7]&0x20) */
		/* logError_st80y(1, "%s %s overflow ID = %02X  Last = %02X\n", tag_st80y,
		 * __func__, data[0], data[7]); */




		if (eventId < NUM_EVT_ID) {	/* this check prevent array out
						 * of index in case of no sense
						 * event ID */
			event_handler = info->event_dispatch_table[eventId];
			event_handler(info, (data), kt);
		}
	}
	vts_report_point_sync(info->vtsdev);
}
/** @}*/

static int ftm_init_type(struct fts_ts_info *info)
{
	int init_type = NO_INIT;
	int ret;
	u8 error_to_search[4] = { EVT_TYPE_ERROR_CRC_CX_HEAD,
				  EVT_TYPE_ERROR_CRC_CX,
				  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD,
				  EVT_TYPE_ERROR_CRC_CX_SUB };

	logError_st80y(1, "%s %s: Verifying if CX CRC Error...\n", tag_st80y, __func__);
	ret = st80y_system_reset(info);
	if (ret >= OK) {
		ret = pollForErrorType(info, error_to_search, 4);
		if (ret < OK) {
			logError_st80y(1, "%s %s: No Cx CRC Error Found!\n", tag_st80y,
				 __func__);
			logError_st80y(1, "%s %s: Verifying if Panel CRC Error...\n",
				 tag_st80y, __func__);
			error_to_search[0] = EVT_TYPE_ERROR_CRC_PANEL_HEAD;
			error_to_search[1] =  EVT_TYPE_ERROR_CRC_PANEL;
			ret = pollForErrorType(info, error_to_search, 2);
			if (ret < OK) {
				logError_st80y(1,
					 "%s %s: No Panel CRC Error Found!\n",
					 tag_st80y,
					 __func__);
				init_type = NO_INIT;
			} else {
				logError_st80y(1,
					 "%s %s: Panel CRC Error FOUND! CRC ERROR = %02X\n",
					 tag_st80y, __func__, ret);
				init_type = SPECIAL_PANEL_INIT;
			}
		} else {
			logError_st80y(1,
				 "%s %s: Cx CRC Error FOUND! CRC ERROR = %02X\n",
				 tag_st80y,
				 __func__, ret);

			logError_st80y(1,
				 "%s %s: Select Full Panel Init...\n", tag_st80y,
				 __func__);
			init_type = SPECIAL_FULL_PANEL_INIT;
		}
	} else
		logError_st80y(1,
			 "%s %s: Error while executing system reset! ERROR %08X\n",
			 tag_st80y,
			 __func__, ret);	/* better skip initialization
						 * because the real state is
						 * unknown */


	if (init_type != SPECIAL_FULL_PANEL_INIT) {
		if ((info->systemInfo.u8_cfgAfeVer != info->systemInfo.u8_cxAfeVer)
			|| ((info->systemInfo.u8_mpFlag != MP_FLAG_BOOT) &&
				(info->systemInfo.u8_mpFlag != MP_FLAG_FACTORY))
			) {
			init_type = SPECIAL_FULL_PANEL_INIT;
			logError_st80y(0,
				 "%s %s: Different CX AFE Ver: %02X != %02X or invalid MpFlag = %02X... Execute FULL Panel Init!\n",
				 tag_st80y, __func__, info->systemInfo.u8_cfgAfeVer,
				 info->systemInfo.u8_cxAfeVer, info->systemInfo.u8_mpFlag);
		} else if (info->systemInfo.u8_cfgAfeVer != info->systemInfo.u8_panelCfgAfeVer) {
			init_type = SPECIAL_PANEL_INIT;
			logError_st80y(0,
				 "%s %s: Different Panel AFE Ver: %02X != %02X... Execute Panel Init!\n",
				 tag_st80y, __func__, info->systemInfo.u8_cfgAfeVer,
				 info->systemInfo.u8_panelCfgAfeVer);
		} else {
		
			init_type = NO_INIT;
		}
	}
	VTI("init type: %d --0:NO_INIT--2:SPECIAL_PANEL_INIT --3:SPECIAL_FULL_PANEL_INIT", init_type);
	return init_type;
}

/**
  *	Implement the fw update and initialization flow of the IC that should be
  *executed at every boot up.
  *	The function perform a fw update of the IC in case of crc error or a new
  *fw version and then understand if the IC need to be re-initialized again.
  *	@return  OK if success or an error code which specify the type of error
  *	encountered
  */
int fts_fw_update(struct fts_ts_info *info)
{
	int retval = 0;
	int retval1 = 0;
	int ret;
	int crc_status = 0;
	int error = 0;
	//int init_type = NO_INIT;

#if defined(PRE_SAVED_METHOD) || defined (COMPUTE_INIT_METHOD)
	int keep_cx = 1;
#else
	int keep_cx = 0;
#endif

	VTI("Fw Auto Update is starting...");

	/* check CRC status */
	ret = fts_crc_check(info);
	if (ret > OK) {
		VTE("CRC Error or NO FW!");
		crc_status = ret;
	} else {
		crc_status = 0;
		VTI("NO CRC Error or Impossible to read CRC register!");
	}

	if(0){
		retval = st80y_flashProcedure(info, "xxxx", crc_status, keep_cx);
		if ((retval & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
			VTE("firmware update failed and retry! ERROR %08X", retval);
			st80y_chip_powercycle(info);	/* power reset */
			retval1 = st80y_flashProcedure(info, PATH_FILE_FW, crc_status, keep_cx);
			if ((retval1 & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
				VTE("firmware update failed again!  ERROR %08X\n", retval1);
			}
		}
	}
/*
	if (init_type != NO_INIT) {
		error = fts_chip_initialization(info, init_type);
		if (error < OK)
			logError_st80y(1,
				"%s %s Cannot initialize the chip ERROR %08X\n",
				 tag_st80y,
				 __func__, error);
	}
*/
	error = fts_init_sensing(info);
	if (error < OK)
		VTE("Cannot initialize the hardware device ERROR %08X", error);

	VTI("Fw Update Finished! ret = %08X\n", error);
	return error;
}

/* TODO: define if need to do the full mp at the boot */
/**
  *	Execute the initialization of the IC (supporting a retry mechanism),
  * checking also the resulting data
  *	@see  st80y_production_test_main()
  */
static int fts_chip_initialization(struct fts_ts_info *info, int init_type)
{
	int ret2 = 0;
	int retry;
	int initretrycnt = 0;
	//enum vts_sensor_test_result result;

	/* initialization error, retry initialization */
	for (retry = 0; retry < RETRY_INIT_BOOT; retry++) {
#ifdef DO_AUTO_TUNE_ONLY // not need to do other test 
		ret2 = st80y_production_test_initialization(info, init_type);
#else// 
		ret2 = st80y_production_test_main(info, LIMITS_FILE, 1, init_type, &info->tests_st80y,
			MP_FLAG_BOOT, &result);
#endif
		if (ret2 == OK)
			break;
		initretrycnt++;
		logError_st80y(1,
			 "%s initialization cycle count = %04d - ERROR %08X\n",
			 tag_st80y,
			 initretrycnt, ret2);
		st80y_chip_powercycle(info);
	}
	if (ret2 < OK)	/* initialization error */

		logError_st80y(1, "%s fts initialization failed %d times\n", tag_st80y,
			RETRY_INIT_BOOT);


	return ret2;
}


/**
  * @addtogroup isr
  * @{
  */
/**
  * Top half Interrupt handler function
  * Respond to the interrupt and schedule the bottom half interrupt handler
  * in its work queue
  * @see fts_event_handler()
  */
static irqreturn_t fts_interrupt_handler(int irq, void *handle, ktime_t kt)
{
	struct fts_ts_info *info = handle;

	fts_event_handler(info, kt);
	return IRQ_HANDLED;
}


/**
  * Initialize the dispatch table with the event handlers for any possible event
  * ID
  * Set IRQ pin behavior (level triggered low)
  * Register top half interrupt handler function.
  * @see fts_interrupt_handler()
  */
static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;

	info->event_dispatch_table = kzalloc(sizeof(event_dispatch_handler_t) *
					     NUM_EVT_ID, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		logError_st80y(1, "%s OOM allocating event dispatch table\n", tag_st80y);
		return -ENOMEM;
	}

	for (i = 0; i < NUM_EVT_ID; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINT, enter_pointer);
	install_handler(info, LEAVE_POINT, leave_pointer);
	install_handler(info, MOTION_POINT, motion_pointer);
	install_handler(info, ERROR, error);
	install_handler(info, CONTROLLER_READY, controller_ready);
	install_handler(info, STATUS_UPDATE, status);
	install_handler(info, USER_REPORT, user_report);

	/* disable interrupts in any case */
	error = st80y_disableInterrupt(info);

	logError_st80y(1, "%s Interrupt Mode\n", tag_st80y);
	if (vts_interrupt_register(info->vtsdev, info->client->irq,
		fts_interrupt_handler,IRQF_ONESHOT | IRQF_TRIGGER_LOW, info)) {
		logError_st80y(1, "%s Request irq failed\n", tag_st80y);
		kfree(info->event_dispatch_table);
		error = -EBUSY;
	}

	return error;
}

/**
  *	Clean the dispatch table and the free the IRQ.
  *	This function is called when the driver need to be removed
  */
static void fts_interrupt_uninstall(struct fts_ts_info *info)
{
	st80y_disableInterrupt(info);

	kfree(info->event_dispatch_table);

	free_irq(info->client->irq, info);
}

/**@}*/

/**
  * This function try to attempt to communicate with the IC for the first time
  * during the boot up process in order to read the necessary info for the
  * following stages.
  * The function execute a system reset, read fundamental info (system info)
  * @return OK if success or an error code which specify the type of error
  */
static int fts_init(struct fts_ts_info *info)
{
	int error;
	u8 readData[3];

#if !defined(I2C_INTERFACE) && defined(SPI4_WIRE)
	/* configure manually SPI4 because when no fw is running the chip use
	 * SPI3 by default */
	u8 cmd[1] = {0x00};

	logError_st80y(0, "%s Setting SPI4 mode...\n", tag_st80y);
	cmd[0] = 0x10;
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			    ADDR_GPIO_DIRECTION, cmd, 1);
	if (error < OK) {
		logError_st80y(1, "%s can not set gpio dir ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}

	cmd[0] = 0x02;
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			    ADDR_GPIO_PULLUP, cmd, 1);
	if (error < OK) {
		logError_st80y(1, "%s can not set gpio pull-up ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}

#if defined(ALIX) || defined (SALIXP)
#if defined(ALIX)	
	cmd[0] = 0x70;
#else
	cmd[0] = 0x07;
#endif
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			    ADDR_GPIO_CONFIG_REG3, cmd, 1);
	if (error < OK) {
		logError_st80y(1, "%s can not set gpio config ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}

#else
	cmd[0] = 0x07;
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			    ADDR_GPIO_CONFIG_REG2, cmd, 1);
	if (error < OK) {
		logError_st80y(1, "%s can not set gpio config ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}
#endif

	cmd[0] = 0x30;
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG,
			    ADDR_GPIO_CONFIG_REG0, cmd, 1);
	if (error < OK) {
		logError_st80y(1, "%s can not set gpio config ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}

	cmd[0] = SPI4_MASK;
	error = st80y_writeU8UX(info, FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_ICR, cmd,
			    1);
	if (error < OK) {
		logError_st80y(1, "%s can not set spi4 mode ERROR %08X\n",
			 tag_st80y, error);
		return error;
	}
	msleep(1);	/* wait for the GPIO to stabilize */
#endif
	logError_st80y(1, "%s Reading chip id\n", tag_st80y);
	error = st80y_writeReadU8UX(info, FTS_CMD_HW_REG_R, ADDR_SIZE_HW_REG,
					    ADDR_DCHIP_ID, readData, 2, DUMMY_FIFO);
	logError_st80y(1, "%s chip id: %02X %02X!\n",tag_st80y, readData[0], readData[1]);
	
	if((readData[0] != DCHIP_ID_0) || (readData[1] != DCHIP_ID_1))
	{
		logError_st80y(1, "%s wrong chip id detected!\n", tag_st80y);
		return ERROR_OP_NOT_ALLOW;
	}
	else
		logError_st80y(1, "%s chip id read successful!\n", tag_st80y);


	error = st80y_system_reset(info);
	if (error < OK && st80y_isI2cError(error)) {
		logError_st80y(1, "%s Cannot reset the device! ERROR %08X\n", tag_st80y,
			 error);
		return error;
	} else {
		if (error == (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
			logError_st80y(1, "%s Setting default Sys INFO!\n", tag_st80y);
			error = defaultSysInfo(info, 0);
		} else {
			error = readSysInfo(info, 0);	/* system reset OK */
			if (error < OK) {
				if (!st80y_isI2cError(error))
					error = OK;
				logError_st80y(1,
					"%s Cannot read Sys Info! ERROR %08X\n",
					 tag_st80y,
					 error);
			}
		}
	}

	return error;
}

/**
  * Execute a power cycle in the IC, toggling the power lines (AVDD and DVDD)
  * @param info pointer to fts_ts_info struct which contain information of the
  * regulators
  * @return 0 if success or another value if fail
  */
int st80y_chip_powercycle(struct fts_ts_info *info)
{
	int error = 0;

	logError_st80y(1, "%s %s: Power Cycle Starting...\n", tag_st80y, __func__);
	logError_st80y(1, "%s %s: Disabling IRQ...\n", tag_st80y, __func__);
	/* if IRQ pin is short with DVDD a call to the ISR will triggered when
	  * the regulator is turned off if IRQ not disabled */
	st80y_disableInterrupt(info);

	if (info->vdd_reg) {
		error = regulator_disable(info->vdd_reg);
		if (error < 0)
			logError_st80y(1, "%s %s: Failed to disable DVDD regulator\n",
				 tag_st80y, __func__);
	}

	if (info->avdd_reg) {
		error = regulator_disable(info->avdd_reg);
		if (error < 0)
			logError_st80y(1, "%s %s: Failed to disable AVDD regulator\n",
				 tag_st80y, __func__);
	}

	if (info->board.reset_gpio != GPIO_NOT_DEFINED)
		gpio_set_value(info->board.reset_gpio, 0);
	else
		msleep(300);

	/* in FTI power up first the digital and then the analog */
	if (info->vdd_reg) {
		error = regulator_enable(info->vdd_reg);
		if (error < 0)
			logError_st80y(1, "%s %s: Failed to enable DVDD regulator\n",
				 tag_st80y, __func__);
	}

	msleep(1);

	if (info->avdd_reg) {
		error = regulator_enable(info->avdd_reg);
		if (error < 0)
			logError_st80y(1, "%s %s: Failed to enable AVDD regulator\n",
				 tag_st80y, __func__);
	}

	msleep(5);	/* time needed by the regulators for reaching the regime
			 * values */


	if (info->board.reset_gpio != GPIO_NOT_DEFINED) {
		msleep(10);	/* time to wait before bring up the reset gpio
				 * after the power up of the regulators */
		gpio_set_value(info->board.reset_gpio, 1);
	}

	logError_st80y(1, "%s %s: Power Cycle Finished! ERROR CODE = %08x\n", tag_st80y,
		 __func__, error);
	setSystemResetedUp(info, 1);
	setSystemResetedDown(info, 1);
	return error;
}


/**
  * Complete the boot up process, initializing the sensing of the IC according
  * to the current setting chosen by the host
  * @return OK if success or an error code which specify the type of error
  */
static int fts_init_sensing(struct fts_ts_info *info)
{
	int error = 0;

	error |= fts_interrupt_install(info);	/* register event handler */
	error |= fts_mode_handler(info, 0);	/* enable the features and
						 * sensing */
	/* error |=  st80y_enableInterrupt(info); */	/* enable the interrupt */
	error |= fts_resetDisableIrqCount(info);

	if (error < OK)
		logError_st80y(1, "%s %s Init after Probe error (ERROR = %08X)\n",
			 tag_st80y, __func__, error);


	return error;
}

/* TODO: change this function according with the needs of customer in terms of
 * feature to enable/disable */

/**
  * @ingroup mode_section
  * @{
  */
/**
  * The function handle the switching of the mode in the IC enabling/disabling
  * the sensing and the features set from the host
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param force if 1, the enabling/disabling command will be send even
  * if the feature was already enabled/disabled otherwise it will judge if
  * the feature changed status or the IC had a system reset
  * @return OK if success or an error code which specify the type of error
  *encountered
  */
static int fts_mode_handler(struct fts_ts_info *info, int force)
{
	int res = OK;
	int ret = OK;
	u8 settings[4] = { 0 };

	/* disable irq wake because resuming from gesture mode */
	if (IS_POWER_MODE(info->mode, SCAN_MODE_LOW_POWER) &&
	    (info->resume_bit == 1))
		disable_irq_wake(info->client->irq);

	info->mode = MODE_NOTHING;	/* initialize the mode to nothing in
					 * order to be updated depending on the
					 * features enabled */

	VTI("Mode Handler starting...");
	switch (info->resume_bit) {
	case 0:	/* screen off */
		VTI("Screen & Sense OFF...");
		/* do sense off in order to avoid the flooding of the fifo with
		 * touch events if someone is touching the panel during suspend,
		 * for speed reason (no need to check echo in this case and
		 * interrupt can be enabled)
		 */
		ret = setScanMode(info, SCAN_MODE_ACTIVE, 0x00);
		res |= ret;	/* to avoid warning unsused ret variable when
				 * all the features are disabled */

#ifdef GESTURE_MODE
		if (info->gesture_enabled == 1) {
			VTI("enter in gesture mode !");
			res = st80y_enterGestureMode(info, st80y_isSystemResettedDown(info));
			if (res >= OK) {
				enable_irq_wake(info->client->irq);
				fromIDtoMask(FEAT_SEL_GESTURE, (u8 *)&info->mode, sizeof(info->mode));
				MODE_LOW_POWER(info->mode, 0);
			} else {
				VTE("st80y_enterGestureMode failed! ERROR %08X recovery in st80y_senseOff...", res);
			}
		}
#endif
		setSystemResetedDown(info, 0);
		break;

	case 1:	/* screen on */
		VTI("Screen ON...");
#ifdef GLOVE_MODE
		if ((info->glove_enabled == FEAT_ENABLE && st80y_isSystemResettedUp(info)) || force == 1) {
			VTI("Glove Mode setting...");
			settings[0] = info->glove_enabled;
			/* required to satisfy also the disable case */
			ret = setFeatures(info, FEAT_SEL_GLOVE, settings, 1);
			if (ret < OK)
				VTE("error during setting GLOVE_MODE! ERROR %08X", ret);

			res |= ret;
			if (ret >= OK && info->glove_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_GLOVE, (u8 *)&info->mode, sizeof(info->mode));
				VTI("GLOVE_MODE Enabled!");
			} else {
				VTE("GLOVE_MODE Disabled!");
			}
		}
#endif

#ifdef COVER_MODE
		if ((info->cover_enabled == FEAT_ENABLE && st80y_isSystemResettedUp(info)) || force == 1) {
			VTI("Cover Mode setting...");
			settings[0] = info->cover_enabled;
			ret = setFeatures(info, FEAT_SEL_COVER, settings, 1);
			if (ret < OK)
				VTE("error during setting COVER_MODE! ERROR %08X", ret);

			res |= ret;
			if (ret >= OK && info->cover_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_COVER, (u8 *)&info->mode, sizeof(info->mode));
				VTI("COVER_MODE Enabled!");
			} else{
				VTE("COVER_MODE Disabled!");
			}
		}
#endif

#ifdef CHARGER_MODE
		if ((info->charger_enabled > 0 && st80y_isSystemResettedUp(info)) || force == 1) {
			VTI("Charger Mode setting...");
			settings[0] = info->charger_enabled;
			ret = setFeatures(info, FEAT_SEL_CHARGER, settings, 1);
			if (ret < OK)
				VTE("error during setting CHARGER_MODE! ERROR %08X", ret);

			res |= ret;
			if (ret >= OK && info->charger_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_CHARGER, (u8 *)&info->mode, sizeof(info->mode));
				VTI("CHARGER_MODE Enabled!");
			} else{
				VTE("CHARGER_MODE Disabled!");
			}
		}
#endif

#ifdef GRIP_MODE
		if ((info->grip_enabled == FEAT_ENABLE && st80y_isSystemResettedUp(info)) || force == 1) {
			VTI("Grip Mode setting...");
			settings[0] = info->grip_enabled;
			ret = setFeatures(info, FEAT_SEL_GRIP, settings, 1);
			if (ret < OK)
				VTE("error during setting GRIP_MODE! ERROR %08X", ret);

			res |= ret;
			if (ret >= OK && info->grip_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_GRIP, (u8 *)&info->mode, sizeof(info->mode));
				VTI("GRIP_MODE Enabled!");
			} else{
				VTE("GRIP_MODE Disabled!");
			}
		}
#endif
		/* if some selective scan want to be enabled can be done an or
		 * of the following options */
		/* settings[0] = ACTIVE_MULTI_TOUCH | ACTIVE_KEY | ACTIVE_HOVER
		 * | ACTIVE_PROXIMITY | ACTIVE_FORCE; */
		settings[0] = 0x01;	/* enable all the possible scans mode
					 * supported by the config */
		VTI("Sense ON!");
		res |= setScanMode(info, SCAN_MODE_ACTIVE, settings[0]);
		info->mode |= (SCAN_MODE_ACTIVE << 24);
		MODE_ACTIVE(info->mode, settings[0]);
		setSystemResetedUp(info, 0);
		break;

	default:
		VTE("nvalid resume_bit value = %d! ERROR %08X", info->resume_bit, ERROR_OP_NOT_ALLOW);
		res = ERROR_OP_NOT_ALLOW;
	}


	VTI("Mode Handler finished! res = %08X mode = %08X", res, info->mode);
	return res;
}

int fts_mode_handler_extern(struct fts_ts_info *info, int force)
{
	return fts_mode_handler(info, force);

}
/**
  * From the name of the power regulator get/put the actual regulator structs
  * (copying their references into fts_ts_info variable)
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param get if 1, the regulators are get otherwise they are put (released)
  * back to the system
  * @return OK if success or an error code which specify the type of error
  */
static int fts_get_reg(struct fts_ts_info *info, bool get)
{
	int retval;
	const struct fts_hw_platform_data *bdata = &info->board;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->vdd_reg_name != NULL) && (*bdata->vdd_reg_name != 0)) {
		info->vdd_reg = regulator_get(info->dev, bdata->vdd_reg_name);
		if (IS_ERR(info->vdd_reg)) {
			VTE("Failed to get power regulator");
			retval = PTR_ERR(info->vdd_reg);
			goto regulator_put;
		}
	}

	if ((bdata->avdd_reg_name != NULL) && (*bdata->avdd_reg_name != 0)) {
		info->avdd_reg = regulator_get(info->dev, bdata->avdd_reg_name);
		if (IS_ERR(info->avdd_reg)) {
			VTE("Failed to get bus regulator");
			retval = PTR_ERR(info->avdd_reg);
			goto regulator_put;
		}
	}

	return OK;

regulator_put:
	if (info->vdd_reg) {
		regulator_put(info->vdd_reg);
		info->vdd_reg = NULL;
	}

	if (info->avdd_reg) {
		regulator_put(info->avdd_reg);
		info->avdd_reg = NULL;
	}

	return retval;
}


/**
  * Enable or disable the power regulators
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param enable if 1, the power regulators are turned on otherwise they are
  * turned off
  * @return OK if success or an error code which specify the type of error
  */
static int fts_enable_reg(struct fts_ts_info *info, bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (info->vdd_reg) {
		retval = regulator_enable(info->vdd_reg);
		if (retval < 0) {
			VTE("Failed to enable bus regulator");
			goto exit;
		}
	}
	msleep(5);

	if (info->avdd_reg) {
		retval = regulator_enable(info->avdd_reg);
		if (retval < 0) {
			VTE("Failed to enable power regulator");
			goto disable_bus_reg;
		}
	}
	msleep(10);

	return OK;

disable_pwr_reg:
	if (info->avdd_reg)
		regulator_disable(info->avdd_reg);

disable_bus_reg:
	if (info->vdd_reg)
		regulator_disable(info->vdd_reg);
	
	msleep(10);
exit:
	return retval;
}

/**
  * Configure a GPIO according to the parameters
  * @param gpio gpio number
  * @param config if true, the gpio is set up otherwise it is free
  * @param dir direction of the gpio, 0 = in, 1 = out
  * @param state initial value (if the direction is in, this parameter is
  * ignored)
  * return error code
  */
static int st80y_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		snprintf(buf, 16, "fts_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			VTE("Failed to get gpio %d (code: %d)", gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);

		if (retval) {
			VTE("Failed to set gpio %d direction", gpio);
			return retval;
		}
	} else
		gpio_free(gpio);

	return retval;
}

/**
  * Setup the IRQ and RESET (if present) gpios.
  * If the Reset Gpio is present it will perform a cycle HIGH-LOW-HIGH in order
  *to assure that the IC has been reset properly
  */
static int st80y_set_gpio(struct fts_ts_info *info)
{
	int retval;
	struct fts_hw_platform_data *bdata = &info->board;

	retval = st80y_gpio_setup(bdata->irq_gpio, true, 0, 0);
	if (retval < 0) {
		VTE("Failed to configure irq GPIO");
		goto err_gpio_irq;
	}

	if (bdata->reset_gpio >= 0) {
		retval = st80y_gpio_setup(bdata->reset_gpio, true, 1, 0);
		if (retval < 0) {
			VTE("Failed to configure reset GPIO");
			goto err_gpio_reset;
		}
	}
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		msleep(10);
		gpio_set_value(bdata->reset_gpio, 1);
	}

	return OK;

err_gpio_reset:
	st80y_gpio_setup(bdata->irq_gpio, false, 0, 0);
	bdata->reset_gpio = GPIO_NOT_DEFINED;
err_gpio_irq:
	return retval;
}

/**
  * Retrieve and parse the hw information from the device tree node defined in
  * the system.
  * the most important information to obtain are: IRQ and RESET gpio numbers,
  * power regulator names
  * In the device file node is possible to define additional optional
  *information that can be parsed here.
  */
static int parse_dt(struct device *dev, struct fts_hw_platform_data *bdata)
{
	int retval;
	const char *name;
	struct device_node *np = dev->of_node;

	bdata->irq_gpio = of_get_named_gpio_flags(np, "st,irq-gpio", 0, NULL);
	VTI("irq_gpio = %d", bdata->irq_gpio);
	VTI("device name = %s", np->name);
	retval = of_property_read_string(np, "st,regulator_dvdd", &name);
	if (retval == -EINVAL)
		bdata->vdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->vdd_reg_name = name;
		VTI("pwr_reg_name = %s", name);
	}

	retval = of_property_read_string(np, "st,regulator_avdd", &name);
	if (retval == -EINVAL)
		bdata->avdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->avdd_reg_name = name;
		VTI("bus_reg_name = %s", name);
	}
	if (of_property_read_bool(np, "st,reset-gpio")) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
							    "st,reset-gpio", 0,
							    NULL);
		VTI("reset_gpio = %d", bdata->reset_gpio);
	} else
		bdata->reset_gpio = GPIO_NOT_DEFINED;

	return OK;
}

static int st80y_power_init(struct fts_ts_info *info)
{
	int retval;
	const struct fts_hw_platform_data *bdata = &info->board;

	if (bdata->vdd_reg_name) {
		info->vdd_reg = devm_regulator_get(info->dev, bdata->vdd_reg_name);
		if (IS_ERR_OR_NULL(info->vdd_reg)) {
			VTE("Failed to get power regulator");
			retval = PTR_ERR(info->vdd_reg);
			return retval;
		}
	} else if (gpio_is_valid(bdata->power_gpio)) {
		retval = gpio_request(bdata->power_gpio, FTS_TS_DRV_NAME);
		if (retval) {
			VTE("power_gpio request fail!");
			return retval;
		}
	}

	if (bdata->avdd_reg_name) {
		info->avdd_reg = devm_regulator_get(info->dev, bdata->avdd_reg_name);
		if (IS_ERR_OR_NULL(info->avdd_reg)) {
			VTE("Failed to get bus pullup regulator");
			retval = PTR_ERR(info->avdd_reg);
			return retval;
		}
	} else if (gpio_is_valid(bdata->vcc_gpio)) {
		retval = gpio_request(bdata->vcc_gpio, FTS_TS_DRV_NAME);
		if (retval) {
			VTE("vcc_gpio request fail!");
			return retval;
		}
	}
	VTI("power init suscess");

	return OK;
}

static int st80y_power_on(struct fts_ts_info *info, bool enable)
{
	int retval = 0;
	struct fts_hw_platform_data *bdata = &info->board;

	retval = fts_enable_reg(info, enable);
	VTI("st80y_power_on");

	if (bdata->power_gpio >= 0) {
		VTI("3.3V power on");
		retval = gpio_direction_output(bdata->power_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			VTE("Fail to set power gpio %s", (enable) ? "on" : "off");
		}
	}
	if (bdata->vcc_gpio >= 0) {
		VTI("1.8 power on");
		retval = gpio_direction_output(bdata->vcc_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			VTE("Fail to set vcc gpio %s", (enable) ? "on" : "off");
		}
	}
	if (bdata->reset_gpio >= 0) {
		msleep(10);
		gpio_set_value(bdata->reset_gpio, 1);
		VTI("reset end");
	}

	return retval;
}
#if SLEEP_MODE_POWER_OFF
static int st80y_power_off(struct fts_ts_info *info, bool enable)
{
	int retval = 0;
	struct fts_hw_platform_data *bdata = &info->board;
	
	st80y_disableInterrupt(info);
	VTI("st80y_power_off");
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		VTI("reset end");
	}

	retval = fts_enable_reg(info, enable);

	if (bdata->power_gpio >= 0) {
		VTI("3.3V power off");
		retval = gpio_direction_output(bdata->power_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			VTE("Fail to set power gpio %s", (enable) ? "on" : "off");
		}
	}
	if (bdata->vcc_gpio >= 0) {
		VTI("1.8 power off");
		retval = gpio_direction_output(bdata->vcc_gpio, (enable) ? 1 : 0);
		if (retval < 0) {
			VTE("Fail to set vcc gpio %s", (enable) ? "on" : "off");
		}
	}
	mdelay(10);
	

	return retval;
}
#endif
static int st80y_hw_init(struct vts_device *vtsdev)
{
	int error;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	u8 guesturemask[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	
	VTI("FW Update and Sensing Initialization:");
	error = fts_fw_update(info);
	if (error < OK) {
		VTE("Cannot execute fw upgrade the device ERROR %08X", error);
		return -ENODEV;
	}

	st80y_updateGestureMask(info, guesturemask, 4, FEAT_ENABLE);  /*enable by default all the gestures */
	return 0;
}

static int st80y_mode_change(struct vts_device *vtsdev, int which)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	
	if (which == VTS_ST_NORMAL) {
		#if SLEEP_MODE_POWER_OFF
		if(info->last_mode_state == VTS_ST_SLEEP){
		//power on
			st80y_power_on(info, true);
			mdelay(20);
		}
		#endif
		info->resume_bit = 1;
		st80y_system_reset(info);
		fts_mode_handler(info, 0);
		info->sensor_sleep = false;
		 st80y_enableInterrupt(info);
	} else if (which == VTS_ST_SLEEP) {
		#if SLEEP_MODE_POWER_OFF
			st80y_power_off(info, false);
		#else 
			info->resume_bit = 0;
			info->gesture_enabled = 0;
			fts_mode_handler(info, 0);
			info->sensor_sleep = true;
			st80y_enableInterrupt(info);
		#endif
	} else if (which == VTS_ST_GESTURE) {
		#if SLEEP_MODE_POWER_OFF
			if(info->last_mode_state == VTS_ST_SLEEP){
				st80y_power_on(info, true);
				mdelay(20);
				st80y_system_reset(info);
			}
		#endif
		info->resume_bit = 0;
		info->gesture_enabled = 1;
		fts_mode_handler(info, 0);
		info->sensor_sleep = false;
		st80y_enableInterrupt(info);
	}
	info->last_mode_state = which;
	return 0;
}

static int st80y_write_charger_flag(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	u8 seting;

	fts_info(info, "write usb charger flag,state=%d", state);
	info->charger_enabled = !!state;
	seting = !!state;
	ret = setFeatures(info, FEAT_SEL_CHARGER, &seting, 1);
	if (ret < 0) {
		fts_info(info, "write usb charger flag fail");
	}
	return ret;
}

static int st80y_update_firmware(struct vts_device *vtsdev, const struct firmware *firmware)
{
	Firmware fw;
	int res;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int init_type;
	VTI("FW SIZE IN VTS:%d", (int)firmware->size);

	res = st80y_parseBinFile(info, (u8 *)firmware->data, (int)firmware->size, &fw, 1);
	if (res < OK) {
		VTI("impossible parse ERROR %08X", ERROR_MEMH_READ);
		return res | ERROR_MEMH_READ;
	}

	VTI("Starting flashing procedure...");
	res = st80y_flash_burn(info, fw, 1, 1);
	if (res < OK && res != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
		VTI("flashProcedure: ERROR %08X\n", ERROR_FLASH_PROCEDURE);
		return res | ERROR_FLASH_PROCEDURE;
	}

	VTI("flashing procedure Finished!");

	
	if ((init_type = ftm_init_type(info)) != NO_INIT) {
		res = fts_chip_initialization(info, init_type);
		if (res < 0) {
			fts_info(info, "Fail.Calibration fail.");
			goto exit;	
		}
		fts_info(info, "pass.Calibration pass.");
		
		if (info->systemInfo.u8_mpFlag != MP_FLAG_BOOT) {
			res = saveMpFlag(info, MP_FLAG_BOOT);
			if (res < 0)
				fts_err(info, "Fail! Error while saving the MP flag!\n");
		} else {
			fts_info(info, "skip save MP flag");
		}
	}
	
	res = 0;
exit:
	fts_mode_handler(info, 0);
	vts_report_release(info->vtsdev);
	st80y_enableInterrupt(info);
	return res;
}

static int st80y_fw_version(struct vts_device *vtsdev, u64 *version)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);

	*version = (((u64)info->systemInfo.u16_fwVer) << 16) | ((u64)info->systemInfo.u8_releaseInfo[1] << 8)
		| ((u64)(info->systemInfo.u8_releaseInfo[0]));
	return 0;
}
#if 0
static int st80y_sensor_test(struct vts_device *vtsdev, enum vts_sensor_test_result *result)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	TestToDo tests = info->tests_st80y;
	vts_report_release(info->vtsdev);

	if (info->systemInfo.u8_mpFlag != MP_FLAG_BOOT) {
		fts_info(info, "MP Flag not set!");
		ret = st80y_production_test_main(info, LIMITS_FILE, 1, SPECIAL_PANEL_INIT, &tests, MP_FLAG_BOOT, result);
	} else {
		fts_info(info, "MP Flag set!");
		ret = st80y_production_test_main(info, LIMITS_FILE, 1, NO_INIT, &tests, MP_FLAG_BOOT, result);
	}

	fts_mode_handler(info, 0);
	vts_report_release(info->vtsdev);
	 st80y_enableInterrupt(info);

	return ret;
}
#endif
static int st80y_sensor_caliberate(struct vts_device *vtsdev, int code, enum vts_sensor_cali_result *result)
{
	int ret = 0;
	int retry = 3;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	*result = VTS_SENSOR_CALIBERATE_SUCCESS;

	vts_report_release(info->vtsdev);

	while (retry-- > 0) {
		
		ret = fts_chip_initialization(info, SPECIAL_FULL_PANEL_INIT);
		if (ret < 0) {
			*result |= VTS_SENSOR_CALIBERATE_FAILED;
			fts_info(info, "Fail.Calibration fail. retry");
		} else {
			fts_info(info, "Pass.Calibration Pass.");
			break;
		}	
	}
	if (ret < 0) {
		fts_info(info, "Fail.Calibration fail.");
		goto exit;
	}
	ret = saveMpFlag(info, MP_FLAG_FACTORY);
	fts_info(info, "save mp");
	if (ret < 0) {
		fts_info(info, "[%s]Fail! Error while saving the MP flag!\n", __func__);
		*result |= VTS_SENSOR_CALIBERATE_FAILED;
		goto exit;
	}
	readSysInfo(info, 1);
	fts_info(info, "at_sensor_test result:%d\n", *result);
exit:
		
	fts_mode_handler(info, 0);
	vts_report_release(info->vtsdev);
	st80y_enableInterrupt(info);
	return ret;
}

static int st80y_get_flash_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 19;
	return 0;
}

static int st80y_set_rotation(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	u8 horizontal_screen_cmd[3] = {0xA4, 0x0B,01};
	u8 vertical_screen_cmd[3] = {0xA4, 0x0B,00};

	fts_info(info, "get screen state:%d", state);
	ret = st80y_write(info, (state == 0 || state == 2) ? horizontal_screen_cmd : vertical_screen_cmd, 3);
	return ret;
}

static int st80y_idle_enable(struct vts_device *vtsdev, int enable)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret;

	if (enable)
		ret = setScanMode(info, SCAN_MODE_LOCKED, LOCKED_ACTIVE);
	else
		ret = setScanMode(info, SCAN_MODE_ACTIVE, 0xFF);
	return ret;
}
static int st80y_get_frame(struct vts_device *vtsdev, enum vts_frame_type type, short *data, int size)
{
	int res = 0;
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	MutualSenseFrame frameMS;

	switch (type){
		case VTS_FRAME_MUTUAL_RAW:
			res = st80y_getMSFrame3(info, MS_RAW, &frameMS);
			break;
		case VTS_FRAME_MUTUAL_DELTA:
			res = st80y_getMSFrame3(info, MS_STRENGTH, &frameMS);
			break;
		case VTS_FRAME_AMBIENT_BASELINE:
			res = st80y_getMSFrame3(info, MS_BASELINE, &frameMS);
			break;
		default:
			res = -1;
			break;
	}
	if(frameMS.node_data)
		memcpy(data,frameMS.node_data, frameMS.node_data_size);
	return 0;
		
}
static ssize_t st80y_flash_read(struct vts_device *vtsdev, u8*buf, size_t nbytes)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	
	ret = st80y_readLockDownInfo(info, buf, 0x70, nbytes);
	if (ret < 0)
		return ret;

	return nbytes;
}

static ssize_t st80y_flash_write(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	int ret = 0;
	ret = st80y_writeLockDownInfo(info, buf, nbytes, 0x70);
	if (ret < 0)
		return ret;

	return nbytes;
}
static int st80y_set_aoi_intpin_state(struct vts_device *vtsdev, int state)
{
	int ret = 0;
	u8 cmd[3] = { 0xC0, 0x11, 0x00 };
	struct fts_ts_info *info = vts_get_drvdata(vtsdev);
	cmd[2] = state & 0x00ff;
	ret = st80y_write(info, cmd, 3);
	if (ret < 0)
		VTE("set pin state %d fail", state);
	else 
		VTI("set pin state %d success", state);
	return ret;
	
}


static const struct vts_operations fts_vts_ops = {
	.init = st80y_hw_init,
	.change_mode = st80y_mode_change,
	.set_charging = st80y_write_charger_flag,
	.update_firmware = st80y_update_firmware,
	.get_fw_version = st80y_fw_version,
	//.sensor_test = st80y_sensor_test,
	.sensor_caliberate = st80y_sensor_caliberate,
	.set_rotation = st80y_set_rotation,
	.set_auto_idle = st80y_idle_enable,
	.rom_size = st80y_get_flash_size,
	.get_frame = st80y_get_frame,
	.rom_read = st80y_flash_read,
	.rom_write = st80y_flash_write,
	.set_aoi_intpin = st80y_set_aoi_intpin_state,
};

/**
  * Probe function, called when the driver it is matched with a device with the
  *same name compatible name
  * This function allocate, initialize and define all the most important
  *function and flow that are used by the driver to operate with the IC.
  * It allocates device variables, initialize queues and schedule works,
  *registers the IRQ handler, suspend/resume callbacks, registers the device to
  *the linux input subsystem etc.
  */
#ifdef I2C_INTERFACE
static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
#else
static int fts_probe(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = NULL;
	struct vts_device *vtsdev = NULL;
	struct device_node *np = NULL;
	int error = 0;
	int retval;
	u16 bus_type;

	VTI("driver probe begin!______debug +++ ");
#ifdef I2C_INTERFACE
	VTI("I2C interface...");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VTE(" Unsupported I2C functionality");
		error = -EIO;
		goto ProbeErrorExit_0;
	}

	bus_type = BUS_I2C;
#else
	VTI("SPI interface...");
	client->mode = SPI_MODE_0;
	client->max_speed_hz = SPI_CLOCK_FREQ;
	client->bits_per_word = 8;
	if (spi_setup(client) < 0) {
		VTE(" Unsupported SPI functionality");
		error = -EIO;
		goto ProbeErrorExit_0;
	}
	bus_type = BUS_SPI;
#endif

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		VTE("Out of memory... Impossible to allocate struct info!");
		error = -ENOMEM;
		goto ProbeErrorExit_0;
	}

	np = client->dev.of_node;
	parse_dt(&client->dev, &info->board);
	info->dev = &client->dev;
	info->client = client;
	

	retval = st80y_power_init(info);
	if (retval < 0) {
		VTE("ERROR: Failed to get regulators");
		goto ProbeErrorExit_1;
	}

	info->client->irq = gpio_to_irq(info->board.irq_gpio);
	mutex_init(&info->st80y_gestureMask_mutex);

	info->touch_id = 0;
#ifdef STYLUS_MODE
	info->stylus_id = 0;
#endif

	/* init feature switches (by default all the features are disable,
	  * if one feature want to be enabled from the start,
	  * set the corresponding value to 1)*/
	info->gesture_enabled = 0;
	info->glove_enabled = 0;
	info->charger_enabled = 0;
	info->cover_enabled = 0;
	info->grip_enabled = 0;
	info->resume_bit = 1;
	info->flash_write_times = 0;
	info->last_mode_state = VTS_ST_UNKNOWN;
	dev_set_drvdata(info->dev, info);
	initCore(info);
	/* sysfs stuff */
	info->attrs.attrs = fts_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		VTE("ERROR: Cannot create sysfs structure!");
		error = -ENODEV;
		goto ProbeErrorExit_2;
	}

	error = st80y_proc_init(info);
	if (error < OK) {
		VTE("Error: can not create /proc file!");
		error = -ENODEV;
		goto ProbeErrorExit_2;
	}
	error = st80y_set_gpio(info);
	if (error < 0) {
		VTE("Failed to set up GPIO's");
		goto ProbeErrorExit_2;
	}

	error = st80y_power_on(info, true);
	if (error < 0) {
		VTE("Failed to enable regulators");
		goto ProbeErrorExit_2;
	}

	error = fts_init(info);
	if (error < OK) {
		VTE("Cannot initialize the device ERROR %08X", error);
		goto ProbeErrorExit_2;
	}

	vtsdev = vts_device_alloc();
	if(!vtsdev) {
		VTE("vivoTsAlloc fail");
		error = -ENOMEM;
		goto ProbeErrorExit_3;
	}

	vtsdev->ops = &fts_vts_ops;
	vtsdev->busType = bus_type;
	info->vtsdev = vtsdev;
	VTI("device name 1  = %s", np->name);
	vts_parse_dt_property(vtsdev, np);
	vts_set_drvdata(vtsdev, info);
	error = vts_register_driver(vtsdev);
	if(error < 0) {
		VTE("vts_register_driver failed");
		goto ProbeErrorExit_4;
	}
	st80y_tools_dev_init(vtsdev);

	VTI("Probe Finished!");
	return OK;

ProbeErrorExit_4:
	vts_device_free(vtsdev);
ProbeErrorExit_3:
	st80y_proc_remove(info);
ProbeErrorExit_2:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
ProbeErrorExit_1:
	devm_kfree(&client->dev, info);
ProbeErrorExit_0:
	VTI("Probe Failed!");

	return error;
}


/**
  * Clear and free all the resources associated to the driver.
  * This function is called when the driver need to be removed.
  */
#ifdef I2C_INTERFACE
static int fts_remove(struct i2c_client *client)
{
#else
static int fts_remove(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = dev_get_drvdata(&(client->dev));

	st80y_proc_remove(info);

	/* sysfs stuff */
	sysfs_remove_group(&client->dev.kobj, &info->attrs);

	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);

	fts_enable_reg(info, false);
	fts_get_reg(info, false);

	/* free all */
	kfree(info);

	return OK;
}

/**
  * Struct which contains the compatible names that need to match with
  * the definition of the device in the device tree node
  */
static struct of_device_id fts_of_match_table[] = {
	{
		.compatible = "st,fts-V2",
	},
	{},
};

#ifdef I2C_INTERFACE
static const struct i2c_device_id fts_device_id[] = {
	{ FTS_TS_DRV_NAME, 0 },
	{}
};

static struct i2c_driver fts_i2c_driver = {
	.driver			= {
		.name		= FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
	},
	.probe			= fts_probe,
	.remove			= fts_remove,
	.id_table		= fts_device_id,
};
#else
static struct spi_driver fts_spi_driver = {
	.driver			= {
		.name		= FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
		.owner		= THIS_MODULE,
	},
	.probe			= fts_probe,
	.remove			= fts_remove,
};
#endif


static struct work_struct i2c_register_work;
static void i2c_register_handler(struct work_struct *data)
{
	int ret = 0;
	logError_st80y(0, "%s enter", tag_st80y);
	ret = i2c_add_driver(&fts_i2c_driver);
	if(ret) {
		logError_st80y(0, "%s i2c add driver fail.", tag_st80y);
	}
}

static int fts_driver_init(void)
{
	logError_st80y(0, "%s driver init", tag_st80y);
#ifdef I2C_INTERFACE
	INIT_WORK(&i2c_register_work, i2c_register_handler);
	schedule_work(&i2c_register_work);
	return 0;
//#ifdef I2C_INTERFACE
//	return i2c_add_driver(&fts_i2c_driver);
#else
	return spi_register_driver(&fts_spi_driver);
#endif
}

static void fts_driver_exit(void)
{
#ifdef I2C_INTERFACE
	i2c_del_driver(&fts_i2c_driver);
#else
	spi_unregister_driver(&fts_spi_driver);
#endif
}

static const int ic_number[1] = {VTS_IC_ST80Y};
module_vts_driver(st80y, ic_number, fts_driver_init(), fts_driver_exit());

