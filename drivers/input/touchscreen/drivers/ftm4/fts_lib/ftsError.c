/*

 **************************************************************************
 **                        STMicroelectronics 		                **
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                  FTS error / info kernel log reporting			 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

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
#include <linux/completion.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "../fts.h"
#include "ftsCrossCompile.h"
#include "ftsError.h"
#include "ftsIO.h"
#include "ftsTool.h"

int ftm4_isI2cError(int error)
{
	if (((error & 0x000000FF) >= (ERROR_I2C_R & 0x000000FF)) && ((error & 0x000000FF) <= (ERROR_I2C_O & 0x000000FF)))
		return 1;
	else
		return 0;
}

static int ftm4_error_work(struct fts_ts_info *info, u8 ev1, u8 ev2)
{
	int res = OK;

	switch(ev1) {
	case EVENT_TYPE_ESD_ERROR:	/*esd */
		vts_report_ic_exception(info->vtsdev, VTS_EXCEPTION_ESD_ERR, 0);
		res = ftm4_chip_powercycle(info);
		if (res < OK) {
			fts_err(info, "Error performing powercycle ERROR %08X\n", res);
		}

		res = ftm4_system_reset(info);
		if (res < OK) {
			fts_err(info, "Cannot reset the device ERROR %08X\n", res);
		}
		res = (ERROR_HANDLER_STOP_PROC | res);
		break;

	case EVENT_TYPE_WATCHDOG_ERROR:	/*watchdog */
		vts_report_ic_exception(info->vtsdev, VTS_EXCEPTION_WATCHDOG_ERR, 0);
		res = ftm4_system_reset(info);
			if (res < OK) {
				fts_err(info, "Cannot reset the device ERROR %08X\n", res);
			}
		res = (ERROR_HANDLER_STOP_PROC | res);
		break;

	case EVENT_TYPE_CHECKSUM_ERROR: /*CRC ERRORS */
		vts_report_ic_exception(info->vtsdev, VTS_EXCEPTION_CHECKSUM_ERR, ev2);
		switch (ev2) {
		case CRC_CONFIG_SIGNATURE:
			fts_err(info, "Config Signature ERROR !\n");
			break;

		case CRC_CONFIG:
			fts_err(info, "Config CRC ERROR !\n");
			break;

		case CRC_CX_MEMORY:
			fts_err(info, "CX CRC ERROR !\n");
			break;
		}
		break;

	default:
		fts_info(info, "No Action taken! \n");
		break;
	}

	return res;
}

void ftm4_error_init(struct fts_ts_info *info)
{
	info->nr_err_events = 0;
}

int ftm4_errorHandler(struct fts_ts_info *info, u8 *event, int size)
{
	int res = OK;

	if (info != NULL && event != NULL && size > 1 && event[0] == EVENTID_ERROR_EVENT) {
		fts_info(info, "Starting handling...\n");
		if (info->nr_err_events >= 3) {
			fts_err(info, "err events recurrence more than 3 times, do nothing!!");
			return ERROR_OP_NOT_ALLOW;
		}
		info->nr_err_events++;
		res = ftm4_error_work(info, event[1], size > 2 ?  event[2] : 0);
		info->nr_err_events--;
		return res;
	} else {
		fts_err(info, "event Null or not correct size! ERROR %08X \n", ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}
}
