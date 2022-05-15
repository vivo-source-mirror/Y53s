/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                     FTS Utility Functions				 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsCompensation.h"
#include "ftsCrossCompile.h"
#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../fts.h"				/*needed for the PHONE_KEY define*/

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
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
/*
static int system_resetted_up = 0;
static int system_resetted_down = 0; */

int ftm4_readB2(struct fts_ts_info *info, u16 address, u8 *outBuf, int len)
{
	int remaining = len;
	int toRead = 0;
	int retry = 0;
	int ret;
	int event_to_search[3];
	char *temp = NULL;
	u8 readEvent[FIFO_EVENT_SIZE] = {0};
	u8 cmd[4] = { FTS_CMD_REQU_FW_CONF, 0x00, 0x00, (u8)len };

		ftm4_u16ToU8_be(address, &cmd[1]);
		temp = ftm4_printHex("Command B2 = ", cmd, 4);
		if (temp != NULL)
			fts_err(info, "%s", ftm4_printHex("Command B2 = ", cmd, 4));
		kfree(temp);
		do {
			remaining = len;
			ret = ftm4_writeFwCmd(info, cmd, 4);
			if (ret < 0) {
				fts_err(info, "ERROR %02X\n", ERROR_I2C_W);
				return ret;
			} 												/*ask to the FW the data */
			fts_err(info, "Command to FW sent! \n");
			event_to_search[0] = (int)EVENTID_FW_CONFIGURATION;

			while (remaining > OK) {
				event_to_search[1] = (int)((address & 0xFF00) >> 8);
				event_to_search[2] = (int) (address & 0x00FF);
					if (remaining > B2_DATA_BYTES) {
						toRead = B2_DATA_BYTES;
						remaining -= B2_DATA_BYTES;
					} else {
						toRead = remaining;
						remaining = 0;
					}

				ret = ftm4_pollForEvent(info, event_to_search, 3, readEvent, GENERAL_TIMEOUT);
				if (ret >= OK) {			/*start the polling for reading the reply */
					memcpy(outBuf, &readEvent[3], toRead);
					retry = 0;
					outBuf += toRead;

				} else {
					retry += 1;
					break;
				}
				address += B2_DATA_BYTES;
			}

		} while (retry < B2_RETRY && retry != 0);

		if (retry == B2_RETRY) {
			fts_err(info, "ERROR %02X\n", ERROR_TIMEOUT);
			return ERROR_TIMEOUT;
		}
		fts_err(info, "B2 read %d bytes\n", len);


		return OK;
}


int ftm4_readB2U16(struct fts_ts_info *info, u16 address, u8 *outBuf, int byteToRead)
{

	int remaining = byteToRead;
	int toRead = 0;
	int ret;


	u8 *buff = (u8 *)kmalloc((B2_CHUNK + 1) * sizeof(u8), GFP_KERNEL);
	if (buff == NULL) {
		fts_err(info, "ERROR %02X \n", ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	while (remaining > 0) {
		if (remaining >= B2_CHUNK) {
			toRead = B2_CHUNK;
			remaining -= B2_CHUNK;
		} else {
			toRead = remaining;
			remaining = 0;
		}

		ret = ftm4_readB2(info, address, buff, toRead);
		if (ret < 0) {
			kfree(buff);
			return ret;
		}
		memcpy(outBuf, buff, toRead);


		address += toRead;

		outBuf += toRead;

	}

	kfree(buff);
	return OK;
}


int ftm4_releaseInformation(struct fts_ts_info *info)
{
	int ret;
	u8 cmd[1] = { FTS_CMD_RELEASE_INFO };
	int event_to_search[1];
	u8 readEvent[FIFO_EVENT_SIZE];

	event_to_search[0] = (int)EVENTID_RELEASE_INFO;

	fts_err(info, "started... Chip INFO:\n");

	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ret);
		return ret;
	}

	ret = ftm4_pollForEvent(info, event_to_search, 1, &readEvent[0], RELEASE_INFO_TIMEOUT); /*start the polling for reading the reply */
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ret);
		return ret;
	}

	fts_err(info, "Finished!\n");
	return OK;

}

int ftm4_lockDownInfo(struct fts_ts_info *info, u8 *data)
{
	int ret;
	int i = 0, num_event;
	u8 cmd[1] = { FTS_CMD_LOCKDOWN_CMD };
	int event_to_search[3] = {EVENTID_LOCKDOWN_INFO, EVENT_TYPE_LOCKDOWN, 0x00};
	u8 readEvent[FIFO_EVENT_SIZE];


	fts_err(info, " started...\n");

	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, "ERROR %02X\n", ret);
		return ret;
	}


	if (LOCKDOWN_CODE_SIZE <= 4)
		num_event = 1;
	else if (LOCKDOWN_CODE_SIZE%4 == 0)
			num_event = LOCKDOWN_CODE_SIZE / 4;
		else
			num_event = (LOCKDOWN_CODE_SIZE) / 4 + 1;

	fts_err(info, "num_event = %d \n", num_event);
	for (i = 0; i < num_event; i++) {
		ret = ftm4_pollForEvent(info, event_to_search, 3, &readEvent[0], GENERAL_TIMEOUT); /*start the polling for reading the reply */
		if (ret < OK) {
			fts_err(info, "ERROR %02X\n", ret);
			return ret;
		}
		data[i * 4] = readEvent[3];
		data[i * 4 + 1] = readEvent[4];
		data[i * 4 + 2] = readEvent[5];
		data[i * 4 + 3] = readEvent[6];
		event_to_search[2] += 4;
		/*ftm4_logError(0, "%02X %02X %02X %02X ", readEvent[3], readEvent[4], readEvent[5], readEvent[6]); */
	}

	fts_err(info, "Finished!\n");
	return OK;

}


char *ftm4_printHex(char *label, u8 *buff, int count)
{
	int i, offset;
	char *result = NULL;

	offset = strlen(label);
	result = (char *)kmalloc(((offset + 3 * count) + 1) * sizeof(char), GFP_KERNEL);
	if (result != NULL) {
		strlcpy(result, label, offset);

		for (i = 0; i < count; i++) {
			snprintf(&result[offset + i * 3], 4, "%02X ", buff[i]);
		}
		result[offset + i * 3] = '\n';
		/* strcat(result, "\n"); */
	}
	return result;
}

int ftm4_pollForEvent(struct fts_ts_info *info, int *event_to_search, int event_bytes, u8 *readData, int time_to_wait)
{
	int i, find, retry, count_err;
	int time_to_count;
	int err_handling = OK;
	StopWatch clock;

	u8 cmd[1] = { FIFO_CMD_READONE };

	find = 0;
	retry = 0;
	count_err = 0;
	time_to_count = time_to_wait / TIMEOUT_RESOLUTION;

	/*disable_irq(info->client->irq); */
	/* fts_err(info, " disable_irq\n"); */
	ftm4_startStopWatch(&clock);
	while (find != 1 && retry < time_to_count && ftm4_readCmd(info, cmd, 1, readData, FIFO_EVENT_SIZE) >= 0) {
		/*Log of errors */
		if (readData[0] == EVENTID_ERROR_EVENT) {
			fts_err(info, " ERROR EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",readData[0],
				readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
			count_err++;
			err_handling = ftm4_errorHandler(info, readData, FIFO_EVENT_SIZE);
			if ((err_handling&0xF0FF0000) == ERROR_HANDLER_STOP_PROC) {
				fts_err(info, " forced to be stopped! ERROR %08X\n", err_handling);
				return err_handling;
			}
		} else {
			if (readData[0] != EVENTID_NO_EVENT) {
				fts_err(info, "READ EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",readData[0],
					readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
			}
			if (readData[0] == EVENTID_CONTROL_READY && event_to_search[0] != EVENTID_CONTROL_READY) {
				fts_err(info, " Unmanned Controller Ready Event! Setting reset flags...\n");
				/*
				setSystemResettedUp(1);
					setSystemResettedDown(1); */
			}
		}

		find = 1;

		for (i = 0; i < event_bytes; i++) {

			if (event_to_search[i] != -1 && (int)readData[i] != event_to_search[i]) {
				find = 0;
				break;
			}
		}

		retry++;
		mdelay(TIMEOUT_RESOLUTION);
	}
/*	enable_irq(info->client->irq); */
/*       fts_err(info, " enable_irq\n"); */
	ftm4_stopStopWatch(&clock);
	if ((retry >= time_to_count) && find != 1) {
		fts_err(info, " ERROR %02X \n", ERROR_TIMEOUT);
		return ERROR_TIMEOUT;
	} else if (find == 1) {
		fts_err(info, "FOUND EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",readData[0],
				readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
		fts_err(info, " Event found in %d ms (%d iterations)! Number of errors found = %d \n", ftm4_elapsedMillisecond(&clock), retry, count_err);
		return count_err;
	} else {
		fts_err(info, "ERROR %02X \n", ERROR_I2C_R);
		return ERROR_I2C_R;

	}
}


int ftm4_flushFIFO(struct fts_ts_info *info)
{
	u8 cmd = FIFO_CMD_FLUSH;								/*flush the FIFO */
	if (ftm4_writeCmd(info, &cmd, 1) < 0) {
		fts_err(info, ": ERROR %02X \n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}

	fts_info(info, " FIFO flushed! \n");
	return OK;

}

int ftm4_isInterruptEnabled(struct fts_ts_info *info, bool *is_enabled)
{
	u8 cmd[3] = {0xB6, 0x00, 0x2C};
	u8 out[3] = {0, 0, 0};
	int ret;

	ret = ftm4_readCmd(info, cmd,3, out, 3);
	if (ret < OK) {
		fts_err(info, "ERROR:0x%x\n", ret);
		return ret;
	}

	fts_info(info, "0x%x, 0x%x, 0x%x\n", out[0], out[1], out[2]);
	*is_enabled = (out[1] == 0x41) ? true:false;
	return 0;
}

int ftm4_disableInterrupt(struct fts_ts_info *info)
{
	u8 cmd[4] = { FTS_CMD_HW_REG_W, 0x00, 0x00, IER_DISABLE };				/*disable interrupt */
	ftm4_u16ToU8_be(IER_ADDR, &cmd[1]);

	if (ftm4_writeCmd(info, cmd, 4) < OK) {
		fts_err(info, ": ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}
	fts_info(info, " Interrupt Disabled! \n");
	return OK;
}


int ftm4_enableInterrupt(struct fts_ts_info *info)
{
	u8 cmd[4] = { FTS_CMD_HW_REG_W, 0x00, 0x00, IER_ENABLE };				/*enable interrupt */
	ftm4_u16ToU8_be(IER_ADDR, &cmd[1]);
		if (ftm4_writeCmd(info, cmd, 4) < 0) {
		fts_err(info, ": ERROR %02X\n", ERROR_I2C_W);
		return ERROR_I2C_W;
	}
	fts_info(info, " Interrupt Enabled!\n");
	return OK;
}

int fts_u8ToU16n(u8 *src, int src_length, u16 *dst)
{
	int i, j;

	if (src_length % 2 != 0) {
		i = -1;
		return i;
	} else {
		j = 0;
		dst = (u16 *)kmalloc((src_length / 2) * sizeof(u16), GFP_KERNEL);
		for (i = 0; i < src_length; i += 2) {
			dst[j] = ((src[i + 1] & 0x00FF) << 8) + (src[i] & 0x00FF);
			j++;
		}
	}

	return (src_length / 2);
}

int fts_u8ToU16(u8 *src, u16 *dst)
{
	 *dst = (u16)(((src[1] & 0x00FF) << 8) + (src[0] & 0x00FF));
	return 0;
}

int fts_u8ToU16_le(u8 *src, u16 *dst)
{
	 *dst = (u16)(((src[0] & 0x00FF) << 8) + (src[1] & 0x00FF));
	return 0;
}

int ftm4_u16ToU8n(u16 *src, int src_length, u8 *dst)
{
	int i, j;
	dst = (u8 *)kmalloc((2 * src_length) * sizeof(u8), GFP_KERNEL);
	j = 0;
	for (i = 0; i < src_length; i++) {
		dst[j] = (u8) (src[i] & 0xFF00) >> 8;
		dst[j + 1] = (u8) (src[i] & 0x00FF);
		j += 2;
	}

	return src_length * 2;

}

int ftm4_u16ToU8(u16 src, u8 *dst)
{
	dst[0] = (u8)((src & 0xFF00) >> 8);
	dst[1] = (u8)(src & 0x00FF);
	return 0;
}

int ftm4_u16ToU8_be(u16 src, u8 *dst)
{
	dst[0] = (u8)((src & 0xFF00) >> 8);
	dst[1] = (u8)(src & 0x00FF);
	return 0;
}

int ftm4_u16ToU8_le(u16 src, u8 *dst)
{
	dst[1] = (u8)((src & 0xFF00) >> 8);
	dst[0] = (u8)(src & 0x00FF);
	return 0;
}

int ftm4_u8ToU32(const u8 *src, u32 *dst)
{
	 *dst = (u32)(((src[3] & 0xFF) << 24) + ((src[2] & 0xFF) << 16) + ((src[1] & 0xFF) << 8) + (src[0] & 0xFF));
	return 0;
}

int fts_u32ToU8(u32 src, u8 *dst)
{
	dst[3] = (u8)((src & 0xFF000000) >> 24);
	dst[2] = (u8)((src & 0x00FF0000) >> 16);
	dst[1] = (u8)((src & 0x0000FF00) >> 8);
	dst[0] = (u8)(src & 0x000000FF);
	return 0;
}

int ftm4_attempt_function(int(*code)(void), unsigned long wait_before_retry, int retry_count)
{
	int result;
	int count = 0;

	do {
		result = code();
		count++;
		mdelay(wait_before_retry);
	} while (count < retry_count && result < 0);


	if (count == retry_count)
		return (result | ERROR_TIMEOUT);
	else
		return result;

}

int ftm4_system_reset(struct fts_ts_info *info)
{
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search;
	int res = -1;
	int i = 0;
	u8 cmd[4] = { FTS_CMD_HW_REG_W, 0x00, 0x00, SYSTEM_RESET_VALUE };
	event_to_search = (int)EVENTID_CONTROL_READY;

	ftm4_u16ToU8_be(SYSTEM_RESET_ADDRESS, &cmd[1]);

	fts_info(info, "System resetting...reset_gpio %d", info->bdata.reset_gpio);
	for (i = 0; i < SYSTEM_RESET_RETRY && res < 0; i++) {

		if (info->bdata.reset_gpio == GPIO_NOT_DEFINED) {
			res = ftm4_writeCmd(info, cmd, 4);
		} else {
			gpio_set_value(info->bdata.reset_gpio, 0);
			mdelay(10);
			gpio_set_value(info->bdata.reset_gpio, 1);
			res = OK;
		}
		if (res < OK) {
			fts_err(info, " : ERROR %02X\n", ERROR_I2C_W);
		} else {
			res = ftm4_pollForEvent(info, &event_to_search, 1, readData, GENERAL_TIMEOUT);
			if (res < OK) {
				fts_err(info, ": ERROR %02X\n", res);
			}
		}
	}
	if (res < OK) {
		fts_err(info, "...failed after 3 attempts: ERROR %02X\n", (res | ERROR_SYSTEM_RESET_FAIL));
		return (res | ERROR_SYSTEM_RESET_FAIL);
	} else {
		fts_info(info, " System reset DONE!\n");
		/*system_resetted_down = 1;
		system_resetted_up = 1; */
		return OK;
	}

}
/*
int isSystemResettedDown() {
	return system_resetted_down;
}

int isSystemResettedUp() {
	return system_resetted_up;
}

void setSystemResettedDown(int val) {
	system_resetted_down = val;
}

void setSystemResettedUp(int val) {
	system_resetted_up = val;
}
 */
int ftm4_senseOn(struct fts_ts_info *info)
{
	int ret = 0;
	u8 cmd[1] = { FTS_CMD_MS_MT_SENSE_ON };

	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, ": ERROR %02X\n", ERROR_SENSE_ON_FAIL);
		return (ret | ERROR_SENSE_ON_FAIL);
	}

	fts_err(info, ": SENSE ON\n");
	return OK;
}

int ftm4_senseOff(struct fts_ts_info *info)
{
	int ret = 0;
	u8 cmd[1] = { FTS_CMD_MS_MT_SENSE_OFF };


	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, ": ERROR %02X\n", ERROR_SENSE_OFF_FAIL);
		return (ret | ERROR_SENSE_OFF_FAIL);
	}

	fts_err(info, ": SENSE OFF\n");
	return OK;

}

int ftm4_keyOn(struct fts_ts_info *info)
{
	int ret;
	u8 cmd[1] = { FTS_CMD_MS_KEY_ON };


	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, ": ERROR %02X\n", ERROR_SENSE_ON_FAIL);
		return (ret | ERROR_SENSE_ON_FAIL);
	}

	fts_err(info, ": KEY ON\n");
	return OK;

}

int ftm4_keyOff(struct fts_ts_info *info)
{
	int ret;
	u8 cmd[1] = { FTS_CMD_MS_KEY_OFF };

	ret = ftm4_writeFwCmd(info, cmd, 1);
	if (ret < OK) {
		fts_err(info, " keyOff: ERROR %02X\n", ERROR_SENSE_OFF_FAIL);
		return (ret | ERROR_SENSE_OFF_FAIL);
	}

	fts_err(info, " keyOff: KEY OFF\n");
	return OK;

}



int ftm4_cleanUp(struct fts_ts_info *info, int enableTouch)
{
	int res;

	fts_err(info, ": system reset...\n");
	res = ftm4_system_reset(info);
	if (res < OK)
		return res;
	if (enableTouch) {
		fts_err(info, ": enabling touches...\n");
		res = ftm4_senseOn(info);
		if (res < OK)
			return res;
#ifdef PHONE_KEY
		res = ftm4_keyOn(info);
		if (res < OK)
			return res;
#endif
		fts_err(info, ": enabling interrupts...\n");
		res = ftm4_enableInterrupt(info);
		if (res < OK)
			return res;
	}
	return OK;

}

int ftm4_checkEcho(struct fts_ts_info *info, u8 *cmd, int size)
{
	int ret, i;
	//int event_to_search[size + 1];
	int event_to_search[10];//
	u8 readData[FIFO_EVENT_SIZE];

	if ((info->chipinfo.u32_echoEn & 0x00000001) != ECHO_ENABLED) {
		fts_err(info, " ECHO Not Enabled! \n");
		return OK;
	}
	if (size < 1) {
		fts_err(info, ": Error Size = %d not valid! or ECHO not Enabled! ERROR %08X\n", size, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	} else {
		if ((size + 2) > FIFO_EVENT_SIZE)
			size = FIFO_EVENT_SIZE - 2;	/*Echo event EC xx xx xx xx xx xx fifo_status therefore for command with more than 6 bytes will echo only the first 6 */

		event_to_search[0] = EVENTID_ECHO;
		for (i = 1; i <= size; i++) {
			event_to_search[i] = cmd[i - 1];
		}
		ret = ftm4_pollForEvent(info, event_to_search, size + 1, readData, GENERAL_TIMEOUT);
		if (ret < OK) {
				fts_err(info, ": Echo Event not found! ERROR %02X\n", ret);
				return (ret | ERROR_CHECK_ECHO_FAIL);
		}

		fts_info(info, " ECHO OK!\n");
		return OK;
	}

}

int ftm4_featureEnableDisable(struct fts_ts_info *info, int on_off, u32 feature)
{
	int ret;
	u8 cmd[5];

	if (on_off == FEAT_ENABLE) {
		cmd[0] = FTS_CMD_FEATURE_ENABLE;
		fts_info(info, ": Enabling feature %08X ...\n", feature);
	} else {
		cmd[0] = FTS_CMD_FEATURE_DISABLE;
		fts_info(info, ": Disabling feature %08X ...\n", feature);
	}

	fts_u32ToU8(feature, &cmd[1]);
	ret = ftm4_writeCmd(info, cmd, 5);			/*not use writeFwCmd because this function can be called also during interrupt enable and should be fast */
	if (ret < OK) {
		fts_err(info, ": ERROR %02X\n", ret);
		return (ret | ERROR_FEATURE_ENABLE_DISABLE);
	}

	fts_info(info, ": DONE!\n");
	return OK;

}

int ftm4_writeNoiseParameters(struct fts_ts_info *info, u8 *noise)
{
	int ret, i;
	u8 cmd[2 + NOISE_PARAMETERS_SIZE];
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search[2] = {EVENTID_NOISE_WRITE, NOISE_PARAMETERS};
	bool int_enabled = false;

	fts_err(info, ": Writing noise parameters to the IC ...\n");

	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled) {
		ret = ftm4_disableInterrupt(info);
		if (ret < OK) {
			fts_err(info, ": ERROR %08X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}
	}
		cmd[0] = FTS_CMD_NOISE_WRITE;
		cmd[1] = NOISE_PARAMETERS;

		fts_err(info, ": Noise parameters = ");
		for (i = 0; i < NOISE_PARAMETERS_SIZE; i++) {
			cmd[2 + i] = noise[i];
			fts_err(info, "%02X ", cmd[2 + i]);
		}

		fts_err(info, " \n");
		ret = ftm4_writeCmd(info, cmd, NOISE_PARAMETERS_SIZE + 2);			/*not use writeFwCmd because this function should be fast */
		if (ret < OK) {
			fts_err(info, ": impossible write command... ERROR %02X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}


		ret = ftm4_pollForEvent(info, event_to_search, 2, readData, GENERAL_TIMEOUT);
		if (ret < OK) {
			fts_err(info, ": polling FIFO ERROR %02X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}

		if (readData[2] != 0x00) {
			fts_err(info, ": Event check FAIL! %02X != 0x00 ERROR %02X\n", readData[2], ERROR_NOISE_PARAMETERS);
			ret = ERROR_NOISE_PARAMETERS;
			goto ERROR;
		}

		fts_err(info, ": DONE!\n");
		ret = OK;
ERROR:
		if (int_enabled) {
			ret = ftm4_enableInterrupt(info);			/*ensure that the interrupt are always renabled when exit from funct */
			if (ret < OK) {
				fts_err(info, ": ERROR %02X\n", ret);
				return (ret | ERROR_NOISE_PARAMETERS);
			}
		}

		return ret;
}


int ftm4_readNoiseParameters(struct fts_ts_info *info, u8 *noise)
{
	int ret, i;
	u8 cmd[2];
	u8 readData[FIFO_EVENT_SIZE];
	int event_to_search[2] = {EVENTID_NOISE_READ, NOISE_PARAMETERS};
	bool int_enabled = false;

	fts_err(info, " ReadNoiseParameters: Reading noise parameters from the IC ...\n");

	ftm4_isInterruptEnabled(info, &int_enabled);

	if (int_enabled) {
		ret = ftm4_disableInterrupt(info);
		if (ret < OK) {
			fts_err(info, ": ERROR %02X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}
	}
		cmd[0] = FTS_CMD_NOISE_READ;
		cmd[1] = NOISE_PARAMETERS;


		ret = ftm4_writeCmd(info, cmd, 2);			/*not use writeFwCmd should be fast */
		if (ret < OK) {
			fts_err(info, ": impossible write command... ERROR %02X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}


		ret = ftm4_pollForEvent(info, event_to_search, 2, readData, GENERAL_TIMEOUT);
		if (ret < OK) {
			fts_err(info, ": polling FIFO ERROR %02X\n", ret);
			ret = (ret | ERROR_NOISE_PARAMETERS);
			goto ERROR;
		}

		fts_err(info, ": Noise parameters = ");
		for (i = 0; i < NOISE_PARAMETERS_SIZE; i++) {
			noise[i] = readData[2 + i];
			fts_err(info, "%02X ", noise[i]);
		}

		fts_err(info, "\n");

		fts_err(info, ": DONE!\n");
		ret = OK;
ERROR:
	if (int_enabled) {
		ret = ftm4_enableInterrupt(info);			/*ensure that the interrupt are always renabled when exit from funct */
		if (ret < OK) {
			fts_err(info, ": ERROR %02X\n", ret);
			return (ret | ERROR_NOISE_PARAMETERS);
		}
	}

	return ret;

}

short **ftm4_array1dTo2d_short(short *data, int size, int columns)
{

	int i;
	short **matrix = (short **)kmalloc(((int)(size / columns)) * sizeof(short *), GFP_KERNEL);
	if (matrix != NULL) {
		for (i = 0; i < (int)(size / columns); i++) {
			matrix[i] = (short *)kmalloc(columns * sizeof(short), GFP_KERNEL);
		}

		for (i = 0; i < size; i++) {
			matrix[i / columns][i % columns] = data[i];
		}
	}

	return matrix;
}

u8 **ftm4_array1dTo2d_u8(u8 *data, int size, int columns)
{

	int i;
	u8 **matrix = (u8 **)kmalloc(((int)(size / columns)) * sizeof(u8 *), GFP_KERNEL);
	if (matrix != NULL) {
		for (i = 0; i < (int)(size / columns); i++) {
			matrix[i] = (u8 *)kmalloc(columns * sizeof(u8), GFP_KERNEL);
		}

		for (i = 0; i < size; i++) {
			matrix[i / columns][i % columns] = data[i];
		}
	}

	return matrix;
}

void ftm4_print_frame_short(struct fts_ts_info *info, char *label, short **matrix, int row, int column)
{
	int i, j;
	char line_buf[300];
	int offset = 0;
	
	fts_info(info, "%s value read from chip, row:%d column:%d", label, row, column);
	for (i = 0; i < row; i++) {
		/*logError(1, "%s ",tag);*/
		memset(line_buf, 0, sizeof(line_buf));
		offset = 0;
		offset += snprintf(&line_buf[offset], 255, "[%02d]:", i);
		for (j = 0; j < column; j++) {
			/*printk("%d ", matrix[i][j]);*/
			offset += snprintf(&line_buf[offset], 255, "%5d ", matrix[i][j]);
		}
		/*logError(1,"\n");*/
		fts_info(info, "%s\n", line_buf);
		kfree(matrix[i]);
	}
	kfree(matrix);
}

void ftm4_print_frame_u8(struct fts_ts_info *info, char *label, u8 **matrix, int row, int column)
{
	int i, j;
	char line_buf[300];
	int offset = 0;
	
	fts_info(info, "%s value read from chip, row:%d column:%d", label, row, column);
	for (i = 0; i < row; i++) {
		/*logError(1, "%s ",tag);*/
		memset(line_buf, 0, sizeof(line_buf));
		offset = 0;
		offset += snprintf(&line_buf[offset], 255, "[%02d]:", i);
		for (j = 0; j < column; j++) {
			/*printk("%d ", matrix[i][j]);*/
			offset += snprintf(&line_buf[offset], 255, "%5d ", matrix[i][j]);
		}
		/*logError(1,"\n");*/
		fts_info(info, "%s\n", line_buf);
		kfree(matrix[i]);
	}
	kfree(matrix);
}

void ftm4_print_frame_u32(struct fts_ts_info *info, char *label, u32 **matrix, int row, int column)
{
	int i, j;
	char line_buf[300];
	int offset = 0;
	fts_info(info, "%s value read from chip, row:%d column:%d", label, row, column);
	for (i = 0; i < row; i++) {
		/*logError(1, "%s ",tag);*/
		memset(line_buf, 0, sizeof(line_buf));
		offset = 0;
		offset += snprintf(&line_buf[offset], 255, "[%02d]:", i);
		for (j = 0; j < column; j++) {
			/*printk("%d ", matrix[i][j]);*/
			offset += snprintf(&line_buf[offset], 255, "%5d ", matrix[i][j]);
		}
		/*logError(1,"\n");*/
		fts_info(info, "%s\n", line_buf);
		kfree(matrix[i]);
	}
	kfree(matrix);
}


void ftm4_print_frame_int(struct fts_ts_info *info, char *label, int **matrix, int row, int column)
{
	int i, j;
	char line_buf[300];
	int offset = 0;
	fts_info(info, "%s value read from chip, row:%d column:%d", label, row, column);
	for (i = 0; i < row; i++) {
		/*logError(1, "%s ",tag);*/
		memset(line_buf, 0, sizeof(line_buf));
		offset = 0;
		offset += snprintf(&line_buf[offset], 255, "[%02d]:", i);
		for (j = 0; j < column; j++) {
			/*printk("%d ", matrix[i][j]);*/
			offset += snprintf(&line_buf[offset], 255, "%5d ", matrix[i][j]);
		}
		/*logError(1,"\n");*/
		fts_info(info, "%s\n", line_buf);
		kfree(matrix[i]);
	}
	kfree(matrix);
}
