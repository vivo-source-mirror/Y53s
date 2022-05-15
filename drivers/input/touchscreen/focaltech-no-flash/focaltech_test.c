
/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include "focaltech_test.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*---------------------------------------------------------- -
Error Code for Comm
---------------------------------------------------------- - */
#define ERROR_CODE_OK                           0x00
#define ERROR_CODE_CHECKSUM_ERROR               0x01
#define ERROR_CODE_INVALID_COMMAND              0x02
#define ERROR_CODE_INVALID_PARAM                0x03
#define ERROR_CODE_IIC_WRITE_ERROR              0x04
#define ERROR_CODE_IIC_READ_ERROR               0x05
#define ERROR_CODE_WRITE_USB_ERROR              0x06
#define ERROR_CODE_WAIT_RESPONSE_TIMEOUT        0x07
#define ERROR_CODE_PACKET_RE_ERROR              0x08
#define ERROR_CODE_NO_DEVICE                    0x09
#define ERROR_CODE_WAIT_WRITE_TIMEOUT           0x0a
#define ERROR_CODE_READ_USB_ERROR               0x0b
#define ERROR_CODE_COMM_ERROR                   0x0c
#define ERROR_CODE_ALLOCATE_BUFFER_ERROR        0x0d
#define ERROR_CODE_DEVICE_OPENED                0x0e
#define ERROR_CODE_DEVICE_CLOSED                0x0f

#define RETRY_TIME				3
#define BYTES_PER_TIME          128

#define DEVICE_MODE_ADDR		0x00
#define REG_LINE_NUM    		0x01
#define RAW_DIFF_STATE_ADDR		0x06
#define REG_RawBuf0         	0x6A

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int focaltech_enter_factory(void)
{
	u8 RunState = 0;
	int index = 0;
	int ret = ERROR_CODE_COMM_ERROR;

	ret = fts_read_reg_byte(DEVICE_MODE_ADDR, &RunState);
	VTI("ReCode=%d", ret);
	if (ret >= ERROR_CODE_OK) {
		if (((RunState >> 4) & 0x07) == 0x04) {  /*factory */
			ret = ERROR_CODE_OK;
		} else {
			ret = fts_write_reg_byte(DEVICE_MODE_ADDR, 0x40);
			if (ret >= ERROR_CODE_OK) {
				for (index = 0; index < 20; ++index) {
					ret = fts_read_reg_byte(DEVICE_MODE_ADDR, &RunState);
					if (ret >= ERROR_CODE_OK) {
						if (((RunState >> 4) & 0x07) == 0x04) {
							ret = ERROR_CODE_OK;
							break;
						} else {
							ret = ERROR_CODE_COMM_ERROR;
						}
					}
					msleep(50);
				}
				if (ret != ERROR_CODE_OK)
					VTE("EnterFactory read DEVIDE_MODE_ADDR error 3.");
			} else
				VTE("EnterFactory write DEVIDE_MODE_ADDR error 2.");
		}
	} else {
		VTE("EnterFactory read DEVIDE_MODE_ADDR error 1.");
	}

	return ret;
}

static int focaltech_exit_factory(void)
{
	int ret = 0;
	u8 val;
	
	ret = fts_write_reg_byte(0x00, 0);
	if (ret < 0) {
		VTI("exit factary mode failed");
	} else {
		VTI("read factary mode done");
		ret = fts_read_reg_byte(0x00, &val);
		if (ret < 0) {
			VTI("exit factary mode failed");
		} else {
			VTI("exit factary mode sucess");
		}
	}

	return ret;
}

static int focaltech_startscan_tp(void)
{
	u8 read_val;
	int ret;
	unsigned char times = 0;
	const unsigned char MaxTimes = 20;  /*The longest wait 160ms */
	
	ret = fts_read_reg_byte(DEVICE_MODE_ADDR, &read_val);
	if (ret >= 0) {
		read_val |= 0x80;
		ret = fts_write_reg_byte(DEVICE_MODE_ADDR, read_val);
		if (ret >= 0) {
			while (times++ < MaxTimes) {     /*Wait for the scan to complete */
				mdelay(8);    /*8ms */
				ret = fts_read_reg_byte(DEVICE_MODE_ADDR, &read_val);
				if (ret >= 0) {
					if ((read_val >> 7) == 0)
						break;
				} else {
					break;
				}
			}
			if (times < MaxTimes)
				ret = 0;
			else
				ret = 0x0c;
		}
		
	}

	return ret;
}

static unsigned char focaltech_read_data(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
	unsigned char ReCode = ERROR_CODE_COMM_ERROR;
	unsigned char I2C_wBuffer[3] = {0};
	unsigned char *pReadData;
	int i, iReadNum;
	unsigned short BytesNumInTestMode1 = 0;

	iReadNum = ByteNum / BYTES_PER_TIME;

	if (0 != (ByteNum % BYTES_PER_TIME))
		iReadNum++;

	if (ByteNum <= BYTES_PER_TIME) {
		BytesNumInTestMode1 = ByteNum;
	} else {
		BytesNumInTestMode1 = BYTES_PER_TIME;
	}

	ReCode = fts_write_reg_byte(REG_LINE_NUM, LineNum); /*Set row addr; */

	/************************************************************Read raw data in test mode1 */
	mdelay(10);
	I2C_wBuffer[0] = REG_RawBuf0;   /*set begin address */
	pReadData = (unsigned char *)kzalloc(ByteNum, GFP_KERNEL);
	if (pReadData == NULL) {
		VTE("failed to alloc memory");
		return ERROR_CODE_ALLOCATE_BUFFER_ERROR;
	}
	ReCode = fts_read(I2C_wBuffer, 1, pReadData, ByteNum);

	VTI("ReadRawData :%d", ByteNum);
	for (i = 0; i < ByteNum; i++) {
		printk("%x ", pReadData[i]);
	}

	if (ReCode >= ERROR_CODE_OK) {
		for (i = 0; i < (ByteNum >> 1); i++) {
			pRevBuffer[i] = (pReadData[i << 1] << 8) + pReadData[(i << 1) + 1];
		}
	}

	kfree(pReadData);

	return ReCode;
}


static int focaltech_getRawOrDiffdata(int mode, int *data)
{
	int ret = 0;
	int val;
	int tx_num = 0xff & vivoTsGetTxRxNum();
	int rx_num = 0xff & (vivoTsGetTxRxNum() >> 8);

	VTI("set read rawdata or diffdata, mode = %d", mode);
	ret = fts_write_reg_byte(RAW_DIFF_STATE_ADDR, (u8)mode);
	if (ret < 0) {
		VTE("set read rawdata or diffdata failed");
	} else {
		ret = fts_read_reg_byte(RAW_DIFF_STATE_ADDR, (u8 *)&val);
		if (ret < 0) {
			VTE("read rawdata or diffdata failed");
		} else {
			if (val == mode) {
				VTI("could read rawdata or diffdata");
			} else {
				VTI("could not read rawdata or diffdata");
			}
		}
	}
	if (ret < 0) {
		return ret;
	}

	VTI("Start Scan");
	ret = focaltech_startscan_tp();
	if (ret < 0) {
		VTE("scan error");
		return ret;
	}

	ret = focaltech_read_data(0, 0xAD, (tx_num * rx_num) * 2, data);

	return ret;
}

/*****************************************************************************
* Public function prototypes
*****************************************************************************/
/* which : 0 - raw  1 - diff */
int focaltech_get_rawordiff_data(int which, int *data)
{
	int ret;
	
	VTI("get data %d(0:rawdata 1:diffdata)", which);

	ret = focaltech_enter_factory();
	if (ret < 0) {
		VTE("failed to enter factory");
		goto out;
	}
	
	/*rawdata or diffdata */
	ret = focaltech_getRawOrDiffdata(which, data);
	if (ret < 0) {
		VTE("failed to get data");
		goto out;
	}

	ret = focaltech_exit_factory();
	if (ret < 0) {
		VTE("failed to exit factory");
		goto out;
	}

out:
	return 0;
}

