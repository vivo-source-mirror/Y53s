/*

 **************************************************************************
 **						STMicroelectronics 							 **
 **************************************************************************
 **						marco.cali@st.com								 **
 **************************************************************************
 *																		*
 *					 API used by Driver Test Apk						 *
 *																		*
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
#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"

#include "../vts_core.h"

#ifdef DRIVER_TEST

#define MAX_PARAMS 10


/*DEFINE COMMANDS TO TEST */
#define CMD_READ   0x00
#define CMD_WRITE   0x01
#define CMD_READU16   0x02
#define CMD_READB2   0x03
#define CMD_READB2U16   0x04
#define CMD_POLLFOREVENT  0x05
#define CMD_SYSTEMRESET   0x06
#define CMD_CLEANUP   0x07
#define CMD_GETFORCELEN   0x08
#define CMD_GETSENSELEN   0x09
#define CMD_GETMSFRAME   0x0A
/*#define CMD_GETMSKEYFRAME  0x0B */
#define CMD_GETSSFRAME   0x0C
#define CMD_REQCOMPDATA   0x0D
#define CMD_READCOMPDATAHEAD  0x0E
#define CMD_READMSCOMPDATA  0x0F
#define CMD_READSSCOMPDATA  0x10
#define CMD_READGNCOMPDATA  0x11
#define CMD_GETFWVER   0x12
#define CMD_FLASHSTATUS   0x13
#define CMD_FLASHUNLOCK   0x14
#define CMD_READFWFILE   0x15
#define CMD_FLASHPROCEDURE  0x16
#define CMD_ITOTEST   0x17
#define CMD_INITTEST   0x18
#define CMD_MSRAWTEST   0x19
#define CMD_MSINITDATATEST  0x1A
#define CMD_SSRAWTEST   0x1B
#define CMD_SSINITDATATEST  0x1C
#define CMD_MAINTEST   0x1D
#define CMD_POWERCYCLE   0x1E
#define CMD_FWWRITE   0x1F
#define CMD_READCHIPINFO  0x20
#define CMD_REQFRAME	0x21

#define SELF_RAW 	0
#define SELF_DIFF 	1
#define MUTUAL_RAW 	0
#define MUTUAL_DIFF	1

static ssize_t stm_driver_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int n;
	char *p = (char *) buf;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	info->functionToTest = (u32 *) kmalloc(MAX_PARAMS * sizeof (u32), GFP_KERNEL);
	if (info->functionToTest == NULL) {
		fts_err(info, " impossible to allocate info->functionToTest!\n");
		return count;
	}
	memset(info->functionToTest, 0, MAX_PARAMS * sizeof (u32));


	for (n = 0; n < (count + 1) / 3 && n < MAX_PARAMS; n++) {
		sscanf(p, "%02X ", &info->functionToTest[n]);
		p += 3;
		fts_err(info, " info->functionToTest[%d] = %02X \n", n, info->functionToTest[n]);

	}

	info->numberParam = n;
	fts_err(info, " Number of Parameters = %d \n", info->numberParam);
	return count;
}

static ssize_t stm_driver_test_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	char buff[CMD_STR_LEN] = {0};
	int res = -1, j, count;
	/*int res2; */
	int size = 6 * 2;
	int temp = 0, i = 0, byteToRead = 0;
	u8 *readData = NULL;
	u8 *all_strbuff = NULL;
	u8 *cmd;

	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;

	DataHeader dataHead;
	MutualSenseData compData;
	SelfSenseData comData;
	GeneralData gnData;

	u16 address;
	u16 fw_version;
	u16 config_id;

	Firmware fw;


	/*struct used for defining which test  perform during the  MP test */

	TestToDo todoDefault;

	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	if(info == NULL){
		fts_err(info, " client is NULL or i2c can not get clientdata!\n"); /* Add by wangkairjptb */
		res = ERROR_OP_NOT_ALLOW;
		return res;
	}

	fw.data = NULL;

	todoDefault.MutualRaw = 1;
	todoDefault.MutualRawGap = 0;
	todoDefault.MutualCx1 = 0;
	todoDefault.MutualCx2 = 0;
	todoDefault.MutualCx2Adj = 0;
	todoDefault.MutualCxTotal = 1;
	todoDefault.MutualCxTotalAdj = 1;

	todoDefault.MutualKeyRaw = 0;
	todoDefault.MutualKeyCx1 = 0;
	todoDefault.MutualKeyCx2 = 0;
	todoDefault.MutualKeyCxTotal = 0;

	todoDefault.SelfForceRaw = 1;
	todoDefault.SelfForceRawGap = 0;
	todoDefault.SelfForceIx1 = 0;
	todoDefault.SelfForceIx2 = 0;
	todoDefault.SelfForceIx2Adj = 0;
	todoDefault.SelfForceIxTotal = 1;
	todoDefault.SelfForceIxTotalAdj = 0;
	todoDefault.SelfForceCx1 = 0;
	todoDefault.SelfForceCx2 = 0;
	todoDefault.SelfForceCx2Adj = 0;
	todoDefault.SelfForceCxTotal = 0;
	todoDefault.SelfForceCxTotalAdj = 0;

	todoDefault.SelfSenseRaw = 1;
	todoDefault.SelfSenseRawGap = 0;
	todoDefault.SelfSenseIx1 = 0;
	todoDefault.SelfSenseIx2 = 0;
	todoDefault.SelfSenseIx2Adj = 0;
	todoDefault.SelfSenseIxTotal = 1;
	todoDefault.SelfSenseIxTotalAdj = 0;
	todoDefault.SelfSenseCx1 = 0;
	todoDefault.SelfSenseCx2 = 0;
	todoDefault.SelfSenseCx2Adj = 0;
	todoDefault.SelfSenseCxTotal = 0;
	todoDefault.SelfSenseCxTotalAdj = 0;


	if (info->numberParam >= 1 && info->functionToTest != NULL) {
		res = ftm4_disableInterrupt(info);
		if (res < 0) {
			fts_err(info, " stm_driver_test_show: ERROR %08X \n", res);
			res = (res | ERROR_DISABLE_INTER);
			goto END;
		}
		switch (info->functionToTest[0]) {
		case CMD_READ:
			if (info->numberParam >= 4) { /*need to pass: cmdLength cmd[0]  cmd[1] … cmd[cmdLength -1] byteToRead */
				temp = (int) info->functionToTest[1];
				if (info->numberParam == 4 + (temp - 1) && temp != 0) {
					cmd = (u8 *) kmalloc(temp * sizeof (u8), GFP_KERNEL);
					for (i = 0; i < temp; i++) {
						cmd[i] = info->functionToTest[i + 2];
					}
					byteToRead = info->functionToTest[i + 2];
					readData = (u8 *) kmalloc(byteToRead * sizeof (u8), GFP_KERNEL);
					res = ftm4_readCmd(info, cmd, temp, readData, byteToRead);
					size += (byteToRead * sizeof (u8)) * 2;
				} else {
					fts_err(info, " Wrong parameters! \n");
					res = ERROR_OP_NOT_ALLOW;
				}

			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITE:
			if (info->numberParam >= 3) { /*need to pass: cmdLength cmd[0]  cmd[1] … cmd[cmdLength -1] */
				temp = (int) info->functionToTest[1];
				if (info->numberParam == 3 + (temp - 1) && temp != 0) {
					cmd = (u8 *) kmalloc(temp * sizeof (u8), GFP_KERNEL);
					for (i = 0; i < temp; i++) {
						cmd[i] = info->functionToTest[i + 2];
					}
					res = ftm4_writeCmd(info, cmd, temp);
				} else {
					fts_err(info, " Wrong parameters! \n");
					res = ERROR_OP_NOT_ALLOW;
				}

			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FWWRITE:
			if (info->numberParam >= 3) { /*need to pass: cmdLength cmd[0]  cmd[1] … cmd[cmdLength -1] */
				temp = (int) info->functionToTest[1];
				if (info->numberParam == 3 + (temp - 1) && temp != 0) {
					cmd = (u8 *) kmalloc(temp * sizeof (u8), GFP_KERNEL);
					for (i = 0; i < temp; i++) {
						cmd[i] = info->functionToTest[i + 2];
					}
					res = ftm4_writeFwCmd(info, cmd, temp);
				} else {
					fts_err(info, " Wrong parameters! \n");
					res = ERROR_OP_NOT_ALLOW;
				}

			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READU16:
			if (info->numberParam == 6) { /*need to pass: cmd addr[0]  addr[1] byteToRead hasDummyByte */
				byteToRead = info->functionToTest[4];
				readData = (u8 *) kmalloc(byteToRead * sizeof (u8), GFP_KERNEL);
				res = ftm4_readCmdU16(info, (u8) info->functionToTest[1], (u16) ((((u8) info->functionToTest[2] & 0x00FF) << 8) + ((u8) info->functionToTest[3] & 0x00FF)), readData, byteToRead, info->functionToTest[5]);
				size += (byteToRead * sizeof (u8)) * 2;
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READB2:
			if (info->numberParam == 4) { /*need to pass: addr[0]  addr[1] byteToRead */
				byteToRead = info->functionToTest[3];
				readData = (u8 *) kmalloc(byteToRead * sizeof (u8), GFP_KERNEL);
				res = ftm4_readB2(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), readData, byteToRead);
				size += (byteToRead * sizeof (u8)) * 2;
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READB2U16:
			if (info->numberParam == 4) { /*need to pass: addr[0]  addr[1] byteToRead */
				byteToRead = info->functionToTest[3];
				readData = (u8 *) kmalloc(byteToRead * sizeof (u8), GFP_KERNEL);
				res = ftm4_readB2U16(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), readData, byteToRead);
				size += (byteToRead * sizeof (u8)) * 2;
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_POLLFOREVENT:
			if (info->numberParam >= 5) { /*need to pass: eventLength event[0] event[1] … event[eventLength -1] timeTowait */
				temp = (int) info->functionToTest[1];
				if (info->numberParam == 5 + (temp - 1) && temp != 0) {
					readData = (u8 *) kmalloc(FIFO_EVENT_SIZE * sizeof (u8), GFP_KERNEL);
					res = ftm4_pollForEvent(info, (int *) &info->functionToTest[2], temp, readData, ((info->functionToTest[temp + 2] & 0x00FF) << 8) + (info->functionToTest[temp + 3] & 0x00FF));
					if (res >= OK)
						res = OK;	/*ftm4_pollForEvent return the number of error found */
					size += (FIFO_EVENT_SIZE * sizeof (u8)) * 2;
					byteToRead = FIFO_EVENT_SIZE;
				} else {
					fts_err(info, " Wrong parameters! \n");
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SYSTEMRESET:
			res = ftm4_system_reset(info);

			break;

		case CMD_READCHIPINFO:
			if (info->numberParam == 2) { /*need to pass: doRequest */
				res = fts_readChipInfo(info, info->functionToTest[1]);
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_CLEANUP: /* TOUCH ENABLE / DISABLE */
			if (info->numberParam == 2) { /*need to pass: enableTouch */
				res = ftm4_cleanUp(info, info->functionToTest[1]);
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_GETFORCELEN: /*read number Tx channels */
			temp = ftm4_getForceLen(info);
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof (u8)) * 2;
				res = OK;
			}
			break;

		case CMD_GETSENSELEN: /*read number Rx channels */
			temp = ftm4_getSenseLen(info);
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof (u8)) * 2;
				res = OK;
			}
			break;

		case CMD_REQFRAME:		/*request a frame */
			if (info->numberParam == 3) {
				fts_err(info, " Requesting Frame \n");
				res = ftm4_requestFrame(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)));

				if (res < OK) {
					fts_err(info, " Error requesting frame ERROR %02X \n", res);
				} else {
					fts_err(info, " Requesting Frame Finished! \n");
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETMSFRAME:
			if (info->numberParam == 3) {
				fts_err(info, " Get 1 MS Frame \n");
		ftm4_flushFIFO(info);		/*delete the events related to some touch (allow to call this function while touching the sreen without having a flooding of the FIFO) */
				res = ftm4_getMSFrame2(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &frameMS);
				if (res < 0) {
					fts_err(info, " Error while taking the MS frame... ERROR %02X \n", res);

				} else {
					fts_err(info, " The frame size is %d words\n", res);
					size = (res * sizeof (short) + 8) * 2;
					/* set res to OK because if ftm4_getMSFrame is
					   successful res = number of words read
					 */
					res = OK;
		ftm4_print_frame_short(info, "MS frame =", ftm4_array1dTo2d_short(frameMS.node_data, frameMS.node_data_size, frameMS.header.sense_node), frameMS.header.force_node, frameMS.header.sense_node);
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


			/*read self raw */
		case CMD_GETSSFRAME:
			if (info->numberParam == 3) {
				fts_err(info, " Get 1 SS Frame \n");
		ftm4_flushFIFO(info);		/*delete the events related to some touch (allow to call this function while touching the sreen without having a flooding of the FIFO) */
				res = ftm4_getSSFrame2(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &frameSS);

				if (res < OK) {
					fts_err(info, " Error while taking the SS frame... ERROR %02X \n", res);

				} else {
					fts_err(info, " The frame size is %d words\n", res);
					size = (res * sizeof (short) + 8) * 2 + 1;
					/* set res to OK because if ftm4_getMSFrame is
					   successful res = number of words read
					 */
					res = OK;
		ftm4_print_frame_short(info, "SS force frame =", ftm4_array1dTo2d_short(frameSS.force_data, frameSS.header.force_node, 1), frameSS.header.force_node, 1);
		ftm4_print_frame_short(info, "SS sense frame =", ftm4_array1dTo2d_short(frameSS.sense_data, frameSS.header.sense_node, frameSS.header.sense_node), 1, frameSS.header.sense_node);
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_REQCOMPDATA: /*request comp data */
			if (info->numberParam == 3) {
				fts_err(info, " Requesting Compensation Data \n");
				res = ftm4_requestCompensationData(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)));

				if (res < OK) {
					fts_err(info, " Error requesting compensation data ERROR %02X \n", res);
				} else {
					fts_err(info, " Requesting Compensation Data Finished! \n");
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READCOMPDATAHEAD: /*read comp data header */
			if (info->numberParam == 3) {
				fts_err(info, " Requesting Compensation Data \n");
				res = ftm4_requestCompensationData(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)));
				if (res < OK) {
					fts_err(info, " Error requesting compensation data ERROR %02X \n", res);
				} else {
					fts_err(info, " Requesting Compensation Data Finished! \n");
					res = ftm4_readCompensationDataHeader(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &dataHead, &address);
					if (res < OK) {
						fts_err(info, " Read Compensation Data Header ERROR %02X\n", res);
					} else {
						fts_err(info, " Read Compensation Data Header OK!\n");
						size += (2 * sizeof (u8)) * 2;
					}
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_READMSCOMPDATA: /*read mutual comp data */
			if (info->numberParam == 3) {
				fts_err(info, " Get MS Compensation Data \n");
				res = ftm4_readMutualSenseCompensationData(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &compData);

				if (res < OK) {
					fts_err(info, " Error reading MS compensation data ERROR %02X \n", res);
				} else {
					fts_err(info, " MS Compensation Data Reading Finished! \n");
					size = ((compData.node_data_size + 9) * sizeof (u8)) * 2;
		 ftm4_print_frame_u8(info, "MS Data (Cx2) =", ftm4_array1dTo2d_u8(compData.node_data, compData.node_data_size, compData.header.sense_node), compData.header.force_node, compData.header.sense_node);
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READSSCOMPDATA:
			if (info->numberParam == 3) { /*read self comp data */
				fts_err(info, " Get SS Compensation Data... \n");
				res = ftm4_readSelfSenseCompensationData(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &comData);
				if (res < OK) {
					fts_err(info, " Error reading SS compensation data ERROR %02X\n", res);
				} else {
					fts_err(info, " SS Compensation Data Reading Finished! \n");
					size = ((comData.header.force_node + comData.header.sense_node) * 2 + 12) * sizeof (u8) * 2;
		ftm4_print_frame_u8(info, "SS Data Ix2_fm = ", ftm4_array1dTo2d_u8(comData.ix2_fm, comData.header.force_node, comData.header.force_node), 1, comData.header.force_node);
			ftm4_print_frame_u8(info, "SS Data Cx2_fm = ", ftm4_array1dTo2d_u8(comData.cx2_fm, comData.header.force_node, comData.header.force_node), 1, comData.header.force_node);
			ftm4_print_frame_u8(info, "SS Data Ix2_sn = ", ftm4_array1dTo2d_u8(comData.ix2_sn, comData.header.sense_node, comData.header.sense_node), 1, comData.header.sense_node);
			ftm4_print_frame_u8(info, "SS Data Cx2_sn = ", ftm4_array1dTo2d_u8(comData.cx2_sn, comData.header.sense_node, comData.header.sense_node), 1, comData.header.sense_node);
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READGNCOMPDATA:
			if (info->numberParam == 3) { /*read self comp data */
				fts_err(info, " Get General Compensation Data... \n");
				res = ftm4_readGeneralCompensationData(info, (u16) ((((u8) info->functionToTest[1] & 0x00FF) << 8) + ((u8) info->functionToTest[2] & 0x00FF)), &gnData);
				if (res < OK) {
					fts_err(info, " Error reading General compensation data ERROR %02X\n", res);
				} else {
					fts_err(info, " General Compensation Data Reading Finished! \n");
					size = (14) * sizeof (u8) * 2;
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_GETFWVER:
			res = ftm4_getFirmwareVersion(info, &fw_version, &config_id);
			if (res < OK) {
				fts_err(info, " Error reading firmware version and config id ERROR %02X\n", res);
			} else {
				fts_err(info, " ftm4_getFirmwareVersion Finished! \n");
				size += (4) * sizeof (u8) * 2;
			}
			break;

		case CMD_FLASHUNLOCK:
			res = ftm4_flash_unlock(info);
			if (res < OK) {
				fts_err(info, " Impossible Unlock Flash ERROR %02X\n", res);
			} else {
				fts_err(info, " Flash Unlock OK!\n");
			}
			break;

		case CMD_READFWFILE:
			if (info->numberParam == 2) { /*read fw file */
				fts_err(info, " Reading FW File... \n");
				res = ftm4_readFwFile(info, PATH_FILE_FW, &fw, info->functionToTest[1]);
				if (res < OK) {
					fts_err(info, " Error reading FW File ERROR %02X\n", res);
				} else {
					fts_err(info, " Read FW File Finished! \n");
				}
		kfree(fw.data);
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FLASHPROCEDURE:
			if (info->numberParam == 3) { /*flashing procedure */
				fts_err(info, " Starting Flashing Procedure... \n");
				res = ftm4_flashProcedure(info, PATH_FILE_FW, info->functionToTest[1], info->functionToTest[2]);
				if (res < OK) {
					fts_err(info, " Error during flash procedure ERROR %02X\n", res);
				} else {
					fts_err(info, " Flash Procedure Finished! \n");
				}
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

			/*ITO TEST */
		case CMD_ITOTEST:
			res = ftm4_production_test_ito(info);
			break;

			/*Initialization */
		case CMD_INITTEST:
			if (info->numberParam == 2) { /*need to specify if if save value on Flash */
				if (info->functionToTest[1] == 0x01)
					res = ftm4_production_test_initialization(info);
				else
					res = ftm4_production_test_splitted_initialization(info, false);
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_MSRAWTEST: /* MS Raw DATA TEST */
			if (info->numberParam == 2) /*need to specify if stopOnFail */
				res = ftm4_production_test_ms_raw(info, LIMITS_FILE, info->functionToTest[1], &todoDefault);
			else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_MSINITDATATEST: /* MS CX DATA TEST */
			if (info->numberParam == 2) /*need to specify if stopOnFail */
				res = ftm4_production_test_ms_cx(info, LIMITS_FILE, info->functionToTest[1], &todoDefault);
			else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSRAWTEST: /* SS RAW DATA TEST */
			if (info->numberParam == 2) /*need to specify if stopOnFail */
				res = ftm4_production_test_ss_raw(info, LIMITS_FILE, info->functionToTest[1], &todoDefault);
			else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSINITDATATEST: /* SS IX CX DATA TEST */
			if (info->numberParam == 2) /*need to specify if stopOnFail */
				res = ftm4_production_test_ss_ix_cx(info, LIMITS_FILE, info->functionToTest[1], &todoDefault);
			else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

			/*PRODUCTION TEST */
		case CMD_MAINTEST:
			if (info->numberParam == 3) {/*need to specify if stopOnFail and saveInit */
				enum vts_sensor_test_result result;
				res = ftm4_production_test_main(info, LIMITS_FILE, info->functionToTest[1], info->functionToTest[2], &todoDefault, INIT_FIELD, &result);
			} else {
				fts_err(info, " Wrong number of parameters! \n");
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_POWERCYCLE:
			res = ftm4_chip_powercycle(info);
			break;


		default:
			fts_err(info, " COMMAND ID NOT VALID!! Inset a value between 00 and 1E..\n");
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		/*res2 = ftm4_enableInterrupt();						//enabling the interrupt was disabled on purpose in this node because it can be used for testing procedure and between one step and another the interrupt wan to be kept disabled
		if (res2 < 0) {
			fts_err(info, " stm_driver_test_show: ERROR %08X \n", (res2 | ERROR_ENABLE_INTER));
		} */
	} else {
		fts_err(info, " NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n");
		res = ERROR_OP_NOT_ALLOW;
	}

END: /*here start the reporting phase, assembling the data to send in the file node */
	all_strbuff = (u8 *) kmalloc(size, GFP_KERNEL);
	memset(all_strbuff, 0, size);

	snprintf(buff, sizeof (buff), "%02X", 0xAA);
	strlcat(all_strbuff, buff, size);

	snprintf(buff, sizeof (buff), "%08X", res);
	strlcat(all_strbuff, buff, size);

	if (res >= OK) {
		/*all the other cases are already fine printing only the res. */
		switch (info->functionToTest[0]) {
		case CMD_READ:
		case CMD_READU16:
		case CMD_READB2:
		case CMD_READB2U16:
		case CMD_POLLFOREVENT:
			for (j = 0; j < byteToRead; j++) {
				snprintf(buff, sizeof (buff), "%02X", readData[j]);
				strlcat(all_strbuff, buff, size);
			}
			break;

		case CMD_GETFORCELEN:
		case CMD_GETSENSELEN:
		case CMD_FLASHSTATUS:
			snprintf(buff, sizeof (buff), "%02X", (u8) temp);
			strlcat(all_strbuff, buff, size);
			break;


		case CMD_GETMSFRAME:
			snprintf(buff, sizeof (buff), "%02X", (u8) frameMS.header.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", (u8) frameMS.header.sense_node);
			strlcat(all_strbuff, buff, size);

			for (j = 0; j < frameMS.node_data_size; j++) {
				snprintf(buff, sizeof (buff), "%04X", frameMS.node_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			kfree(frameMS.node_data);
			break;

		case CMD_GETSSFRAME:
			snprintf(buff, sizeof (buff), "%02X", (u8) frameSS.header.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", (u8) frameSS.header.sense_node);
			strlcat(all_strbuff, buff, size);

			/* Copying self raw data Force */
			for (j = 0; j < frameSS.header.force_node; j++) {
				snprintf(buff, sizeof (buff), "%04X", frameSS.force_data[j]);
				strlcat(all_strbuff, buff, size);
			}


			/* Copying self raw data Sense */
			for (j = 0; j < frameSS.header.sense_node; j++) {
				snprintf(buff, sizeof (buff), "%04X", frameSS.sense_data[j]);
				strlcat(all_strbuff, buff, size);
			}

			kfree(frameSS.force_data);
			kfree(frameSS.sense_data);
			break;

		case CMD_READMSCOMPDATA:
			snprintf(buff, sizeof (buff), "%02X", (u8) compData.header.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", (u8) compData.header.sense_node);
			strlcat(all_strbuff, buff, size);

			/*Cpying CX1 value */
			snprintf(buff, sizeof (buff), "%02X", compData.cx1);
			strlcat(all_strbuff, buff, size);

			/*Copying CX2 values */
			for (j = 0; j < compData.node_data_size; j++) {
				snprintf(buff, sizeof (buff), "%02X", *(compData.node_data + j));
				strlcat(all_strbuff, buff, size);
			}

			kfree(compData.node_data);
			break;

		case CMD_READSSCOMPDATA:
			snprintf(buff, sizeof (buff), "%02X", comData.header.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", comData.header.sense_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", comData.f_ix1);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", comData.s_ix1);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", comData.f_cx1);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", comData.s_cx1);
			strlcat(all_strbuff, buff, size);

			/*Copying IX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(buff, sizeof (buff), "%02X", comData.ix2_fm[j]);
				strlcat(all_strbuff, buff, size);
			}

			/*Copying IX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(buff, sizeof (buff), "%02X", comData.ix2_sn[j]);
				strlcat(all_strbuff, buff, size);
			}

			/*Copying CX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				snprintf(buff, sizeof (buff), "%02X", comData.cx2_fm[j]);
				strlcat(all_strbuff, buff, size);
			}

			/*Copying CX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				snprintf(buff, sizeof (buff), "%02X", comData.cx2_sn[j]);
				strlcat(all_strbuff, buff, size);
			}

			kfree(comData.ix2_fm);
			kfree(comData.ix2_sn);
			kfree(comData.cx2_fm);
			kfree(comData.cx2_sn);
			break;

		case CMD_READGNCOMPDATA:
			snprintf(buff, sizeof (buff), "%02X", gnData.header.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.header.sense_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsd_lp_timer_cal0);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsd_lp_timer_cal1);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsd_lp_timer_cal2);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsd_lp_timer_cal3);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsa_lp_timer_cal0);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", gnData.ftsa_lp_timer_cal1);
			strlcat(all_strbuff, buff, size);
			break;

		case CMD_GETFWVER:
			snprintf(buff, sizeof (buff), "%04X", fw_version);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%04X", config_id);
			strlcat(all_strbuff, buff, size);
			break;

		case CMD_READCOMPDATAHEAD:
			snprintf(buff, sizeof (buff), "%02X", dataHead.force_node);
			strlcat(all_strbuff, buff, size);

			snprintf(buff, sizeof (buff), "%02X", dataHead.sense_node);
			strlcat(all_strbuff, buff, size);
			break;


		default:
			break;
		}
	}

	snprintf(buff, sizeof (buff), "%02X", 0xBB);
	strlcat(all_strbuff, buff, size);

	count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
	info->numberParam = 0; /*need to reset the number of parameters in order to wait the next comand, comment if you want to repeat the last comand sent just doing a cat */
	/*fts_err(info, " numberParameters = %d \n", info->numberParam); */
	kfree(all_strbuff);

	if(readData!=NULL){ /* add by wangkairjptb */
		kfree(readData);
		readData=NULL;
	}

	if(info->functionToTest!=NULL){ /* add by wangkairjptb */
		kfree(info->functionToTest);
		info->functionToTest=NULL;
	}

	return count;
}

static DEVICE_ATTR(stm_driver_test, 0644, stm_driver_test_show, stm_driver_test_store);	/*(S_IRWXU | S_IRWXG | S_IRWXO) */

static struct attribute *test_cmd_attributes[] = {
	&dev_attr_stm_driver_test.attr,
	NULL,
};

static struct attribute_group test_cmd_attr_group = {
	.attrs = test_cmd_attributes,
};

int ftm4_driver_test_init(struct fts_ts_info *info)
{
	return sysfs_create_group(&info->client->dev.kobj, &test_cmd_attr_group);
}

void ftm4_driver_test_deinit(struct fts_ts_info *info)
{
	sysfs_remove_group(&info->client->dev.kobj, &test_cmd_attr_group);
}

#else
int ftm4_driver_test_init(struct fts_ts_info *info)
{
	return 0;
}

void ftm4_driver_test_deinit(struct fts_ts_info *info)
{
	return 0;
}

#endif
