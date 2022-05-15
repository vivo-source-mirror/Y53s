/*
  *
  **************************************************************************
  **                        STMicroelectronics				 **
  **************************************************************************
  **                        marco.cali@st.com				*
  **************************************************************************
  *                                                                        *
  *                     Utilities published in /proc/fts		*
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file fts_proc.c
  * \brief contains the function and variables needed to publish a file node in
  * the file system which allow to communicate with the IC from userspace
  */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"
#include <linux/miscdevice.h>




#define DRIVER_TEST_FILE_NODE	"driver_test"	/* /< name of file node
						 * published */

#define DIAGNOSTIC_NUM_FRAME	10	/* /< number of frames reading
					 * iterations during the diagnostic test
					 */

struct fts_ts_info *fts_tools_info;


/** @defgroup proc_file_code	 Proc File Node
  * @ingroup file_nodes
  * The /proc/fts/driver_test file node provide expose the most important API
  * implemented into the driver to execute any possible operation into the IC \n
  * Thanks to a series of Operation Codes, each of them, with a different set of
  * parameter, it is possible to select a function to execute\n
  * The result of the function is usually returned into the shell as an ASCII
  * hex string where each byte is encoded in two chars.\n
  * @{
  */

/* Bus operations */
#define CMD_READ				0x00	/* /< I2C/SPI read: need
							 * to pass: byteToRead1
							 * byteToRead0
							 * (optional) dummyByte
							 * */
#define CMD_WRITE				0x01	/* /< I2C/SPI write:
							 * need to pass: cmd[0]
							 *  cmd[1] …
							 * cmd[cmdLength-1] */
#define CMD_WRITEREAD				0x02	/* /< I2C/SPI writeRead:
							 * need to pass: cmd[0]
							 *  cmd[1] …
							 * cmd[cmdLength-1]
							 * byteToRead1
							 * byteToRead0 dummyByte
							 * */
#define CMD_WRITETHENWRITEREAD			0x03	/* /< I2C/SPI write then
							 * writeRead: need to
							 * pass: cmdSize1
							 * cmdSize2 cmd1[0]
							 * cmd1[1] …
							 * cmd1[cmdSize1-1]
							 * cmd2[0] cmd2[1] …
							 * cmd2[cmdSize2-1]
							 *  byteToRead1
							 * byteToRead0 */
#define CMD_WRITEU8UX				0x04	/* /< I2C/SPI writeU8UX:
							 * need to pass: cmd
							 * addrSize addr[0] …
							 * addr[addrSize-1]
							 * data[0] data[1] … */
#define CMD_WRITEREADU8UX			0x05	/* /< I2C/SPI
							 * writeReadU8UX: need
							 * to pass: cmd addrSize
							 * addr[0] …
							 * addr[addrSize-1]
							 * byteToRead1
							 * byteToRead0
							 * hasDummyByte */
#define CMD_WRITEU8UXTHENWRITEU8UX		0x06	/* /< I2C/SPI writeU8UX
							 * then writeU8UX: need
							 * to pass: cmd1
							 * addrSize1 cmd2
							 * addrSize2 addr[0] …
							 * addr[addrSize1+addrSize2-1]
							 * data[0] data[1] … */
#define CMD_WRITEU8UXTHENWRITEREADU8UX		0x07	/* /< I2C/SPI writeU8UX
							 *  then writeReadU8UX:
							 * need to pass: cmd1
							 * addrSize1 cmd2
							 * addrSize2 addr[0] …
							 * addr[addrSize1+addrSize2-1]
							 *  byteToRead1
							 * byteToRead0
							 * hasDummybyte */
#define CMD_GETLIMITSFILE			0x08	/* /< Get the Production
							 * Limits File and print
							 * its content into the
							 * shell: need to pass:
							 * path(optional)
							 * otherwise select the
							 * approach chosen at
							 * compile time */
#define CMD_GETFWFILE				0x09	/* /< Get the FW file
							 * and print its content
							 * into the shell: need
							 * to pass: path
							 * (optional) otherwise
							 * select the approach
							 * chosen at compile
							 * time */
#define CMD_VERSION				0x0A	/* /< Get the driver
							 * version and other
							 * driver setting info
							 * */
#define CMD_READCONFIG				0x0B	/* /< Read The config
							 * memory, need to pass:
							 * addr[0] addr[1]
							 * byteToRead0
							 * byteToRead1 */


/* GUI utils byte ver */
#define CMD_READ_BYTE				0xF0	/* /< Byte output
							 * version of I2C/SPI
							 * read @see CMD_READ */
#define CMD_WRITE_BYTE				0xF1	/* /< Byte output
							 * version of I2C/SPI
							 * write @see CMD_WRITE
							 * */
#define CMD_WRITEREAD_BYTE			0xF2	/* /< Byte output
							 * version of I2C/SPI
							 * writeRead @see
							 * CMD_WRITEREAD */
#define CMD_WRITETHENWRITEREAD_BYTE		0xF3	/* /< Byte output
							 * version of I2C/SPI
							 * write then writeRead
							 * @see
							 * CMD_WRITETHENWRITEREAD
							 * */
#define CMD_WRITEU8UX_BYTE			0xF4	/* /< Byte output
							 * version of I2C/SPI
							 * writeU8UX @see
							 * CMD_WRITEU8UX */
#define CMD_WRITEREADU8UX_BYTE			0xF5	/* /< Byte output
							 * version of I2C/SPI
							 * writeReadU8UX @see
							 * CMD_WRITEREADU8UX */
#define CMD_WRITEU8UXTHENWRITEU8UX_BYTE		0xF6	/* /< Byte output
							 * version of I2C/SPI
							 * writeU8UX then
							 * writeU8UX @see
							 * CMD_WRITEU8UXTHENWRITEU8UX
							 * */
#define CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE	0xF7	/* /< Byte output
							 * version of I2C/SPI
							 * writeU8UX  then
							 * writeReadU8UX @see
							 * CMD_WRITEU8UXTHENWRITEREADU8UX
							 * */
#define CMD_GETLIMITSFILE_BYTE			0xF8	/* /< Byte output
							 * version of Production
							 * Limits File @see
							 * CMD_GETLIMITSFILE */
#define CMD_GETFWFILE_BYTE			0xF9	/* /< Byte output
							 * version of FW file
							 * need to pass: @see
							 * CMD_GETFWFILE */
#define CMD_VERSION_BYTE			0xFA	/* /< Byte output
							 * version of driver
							 * version and setting
							 * @see CMD_VERSION */
#define CMD_CHANGE_OUTPUT_MODE			0xFF	/* /< Select the output
							 * mode of the
							 * scriptless protocol,
							 * need to pass:
							 * bin_output = 1 data
							 * returned as binary,
							 * bin_output =0 data
							 * returned as hex
							 * string */

/* Core/Tools */
#define CMD_POLLFOREVENT			0x11	/* /< Poll the FIFO for
							 * an event: need to
							 * pass: eventLength
							 * event[0] event[1] …
							 * event[eventLength-1]
							 * timeToWait1
							 * timeToWait0 */
#define CMD_SYSTEMRESET				0x12	/* /< System Reset */
#define CMD_CLEANUP				0x13	/* /< Perform a system
							 * reset and optionally
							 * re-enable the
							 * scanning, need to
							 * pass: enableTouch */
#define CMD_POWERCYCLE				0x14	/* /< Execute a power
							 * cycle toggling the
							 * regulators */
#define CMD_READSYSINFO				0x15	/* /< Read the System
							 * Info information from
							 * the framebuffer, need
							 * to pass: doRequest */
#define CMD_FWWRITE				0x16	/* /< Write a FW
							 * command: need to
							 * pass: cmd[0]  cmd[1]
							 * … cmd[cmdLength-1] */
#define CMD_INTERRUPT				0x17	/* /< Allow to enable or
							 * disable the
							 * interrupts, need to
							 * pass: enable (if 1
							 * will enable the
							 * interrupt) */
#define CMD_SETSCANMODE				0x18	/* /< set Scan Mode
							 * need to pass:
							 * scanType option */
#define CMD_SAVEMPFLAG				0x19	/* /< save manually a
							 * value in the MP flag
							 * need to pass: mpflag
							 */

/* Frame */
#define CMD_GETFORCELEN				0x20	/* /< Get the number of
							 * Force channels */
#define CMD_GETSENSELEN				0x21	/* /< Get the number of
							 * Sense channels */
#define CMD_GETMSFRAME				0x23	/* /< Get a MS frame:
							 * need to pass:
							 * MSFrameType */
#define CMD_GETSSFRAME				0x24	/* /< Get a SS frame:
							 * need to pass:
							 * SSFrameType */
#define CMD_GETSYNCFRAME			0x25	/* /< Get a Sync Frame:
							 * need to pass:
							 * frameType */

/* Compensation */
#define CMD_REQCOMPDATA				0x30	/* /< Request Init data:
							 * need to pass: type */
#define CMD_READCOMPDATAHEAD			0x31	/* /< Read Init data
							 * header: need to pass:
							 * type */
#define CMD_READMSCOMPDATA			0x32	/* /< Read MS Init data:
							 * need to pass: type */
#define CMD_READSSCOMPDATA			0x33	/* /< Read SS Init data:
							 * need to pass: type */
#define CMD_READTOTMSCOMPDATA			0x35	/* /< Read Tot MS Init
							 * data: need to pass:
							 * type */
#define CMD_READTOTSSCOMPDATA			0x36	/* /< Read Tot SS Init
							 * data: need to pass:
							 * type */

/* FW Update */
#define CMD_GETFWVER				0x40	/* /< Get the FW version
							 * of the IC */
#define CMD_FLASHUNLOCK				0x42	/* /< Unlock the flash
							 * */
#define CMD_READFWFILE				0x43	/* /< Try to read the FW
							 * file, need to pass:
							 * keep_cx */
#define CMD_FLASHPROCEDURE			0x44	/* /< Perform a full
							 * flashing procedure:
							 * need to pass: force
							 * keep_cx */
#define CMD_FLASHERASEUNLOCK			0x45	/* /< Unlock the erase
							 * of the flash */
#define CMD_FLASHERASEPAGE			0x46	/* /< Erase page by page
							 * the flash, need to
							 * pass: keep_cx, if
							 * keep_cx>SKIP_PANEL_INIT
							 * Panel Init Page will
							 * be skipped, if
							 * >SKIP_PANEL_CX_INIT
							 * Cx and Panel Init
							 * Pages will be skipped
							 * otherwise if
							 * =ERASE_ALL all the
							 * pages will be deleted
							 * */


/* MP test */
#define CMD_ITOTEST				0x50	/* /< Perform an ITO
							 * test */
#define CMD_INITTEST				0x51	/* /< Perform an
							 * Initialization test:
							 * need to pass: type */
#define CMD_MSRAWTEST				0x52	/* /< Perform MS raw
							 * test: need to pass
							 * stop_on_fail */
#define CMD_MSINITDATATEST			0x53	/* /< Perform MS Init
							 * data test: need to
							 * pass stop_on_fail */
#define CMD_SSRAWTEST				0x54	/* /< Perform SS raw
							 * test: need to pass
							 * stop_on_fail */
#define CMD_SSINITDATATEST			0x55	/* /< Perform SS Init
							 * data test: need to
							 * pass stop_on_fail */
#define CMD_MAINTEST				0x56	/* /< Perform a full
							 * Mass production test:
							 * need to pass
							 * stop_on_fail saveInit
							 * mpflag
							 * */
#define CMD_FREELIMIT				0x57	/* /< Free (if any)
							 * limit file which was
							 * loaded during any
							 * test procedure */

/* Diagnostic */
#define CMD_DIAGNOSTIC				0x60	/* /< Perform a series
							 * of commands and
							 * collect severals data
							 * to detect any
							 * malfunction */


#define CMD_CHANGE_SAD				0x70	/* /< Allow to change
							 * the SAD address (for
							 * debugging) */
/** @}*/

/** @defgroup scriptless Scriptless Protocol
  * @ingroup proc_file_code
  * Scriptless Protocol allows ST Software (such as FingerTip Studio etc) to
  * communicate with the IC from an user space.
  * This mode gives access to common bus operations (write, read etc) and
  * support additional functionalities. \n
  * The protocol is based on exchange of binary messages included between a
  * start and an end byte
  * @{
  */

#define MESSAGE_START_BYTE	0x7B	/* /< start byte of each message
					 * transferred in Scriptless Mode */
#define MESSAGE_END_BYTE	0x7D	/* /< end byte of each message
					 * transferred in Scriptless Mode */
#define MESSAGE_MIN_HEADER_SIZE 8	/* /< minimun number of bytes of the
					 * structure of a messages exchanged
					 * with host (include start/end byte,
					 * counter, actions, msg_size) */

/************************ SEQUENTIAL FILE UTILITIES **************************/
/**
  * This function is called at the beginning of the stream to a sequential file
  * or every time into the sequential were already written PAGE_SIZE bytes and
  * the stream need to restart
  * @param s pointer to the sequential file on which print the data
  * @param pos pointer to the offset where write the data
  * @return NULL if there is no data to print or the pointer to the beginning of
  * the data that need to be info->printed
  */
static void *fts_seq_start(struct seq_file *s, loff_t *pos)
{
	struct fts_ts_info *info = (struct fts_ts_info *)s->private;
	logError_st80y(0,
		 "%s %s: Entering start(), pos = %ld info->limit = %d info->printed = %d\n",
		 tag_st80y,
		 __func__, *pos, info->limit, info->printed);

	if (info->driver_test_buff == NULL && *pos == 0) {
		logError_st80y(1, "%s %s: No data to print!\n", tag_st80y, __func__);
		info->driver_test_buff = (u8 *)kmalloc(13 * sizeof(u8), GFP_KERNEL);

		snprintf(info->driver_test_buff, 14, "{ %08X }\n", ERROR_OP_NOT_ALLOW);


		info->limit = strlen(info->driver_test_buff);
		/* logError_st80y(0, "%s %s: len = %d info->driver_test_buff = %s\n", tag_st80y,
		 * __func__, info->limit, info->driver_test_buff); */
	} else {
		if (*pos != 0)
			*pos += info->chunk - 1;

		if (*pos >= info->limit)
			/* logError_st80y(0, "%s %s: Apparently, we're done.\n", tag_st80y,
			 * __func__); */
			return NULL;
	}

	info->chunk = CHUNK_PROC;
	if (info->limit - *pos < CHUNK_PROC)
		info->chunk = info->limit - *pos;
	/* logError_st80y(0, "%s %s: In start(), updated pos = %ld info->limit = %d info->printed = %d info->chunk = %d\n",
	 *	tag_st80y, __func__, *pos, info->limit, info->printed, info->chunk); */
	memset(info->buf_chunk, 0, CHUNK_PROC);
	memcpy(info->buf_chunk, &info->driver_test_buff[(int)*pos], info->chunk);

	return info->buf_chunk;
}

/**
  * This function actually print a info->chunk amount of data in the sequential file
  * @param s pointer to the sequential file where to print the data
  * @param v pointer to the data to print
  * @return 0
  */
static int fts_seq_show(struct seq_file *s, void *v)
{
	struct fts_ts_info *info = (struct fts_ts_info *)s->private;

	seq_write(s, (u8 *)v, info->chunk);
	info->printed += info->chunk;
	return 0;
}

/**
  * This function update the pointer and the counters to the next data to be
  * info->printed
  * @param s pointer to the sequential file where to print the data
  * @param v pointer to the data to print
  * @param pos pointer to the offset where write the next data
  * @return NULL if there is no data to print or the pointer to the beginning of
  * the next data that need to be info->printed
  */
static void *fts_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	struct fts_ts_info *info = (struct fts_ts_info *)s->private;
	/* int* val_ptr; */
	/* logError_st80y(0, "%s %s: In next(), v = %X, pos = %ld.\n", tag_st80y, __func__,
	 * v, *pos); */
	(*pos) += info->chunk;/* increase my position counter */
	info->chunk = CHUNK_PROC;

	if (*pos >= info->limit)	/* are we done? */
		return NULL;
	else if (info->limit - *pos < CHUNK_PROC)
		info->chunk = info->limit - *pos;


	memset(info->buf_chunk, 0, CHUNK_PROC);
	memcpy(info->buf_chunk, &info->driver_test_buff[(int)*pos], info->chunk);
	return info->buf_chunk;
}


/**
  * This function is called when there are no more data to print  the stream
  *need to be terminated or when PAGE_SIZE data were already written into the
  *sequential file
  * @param s pointer to the sequential file where to print the data
  * @param v pointer returned by fts_seq_next
  */
static void fts_seq_stop(struct seq_file *s, void *v)
{
	struct fts_ts_info *info = (struct fts_ts_info *)s->private;
	/* logError_st80y(0, "%s %s: Entering stop().\n", tag_st80y, __func__); */

	if (v) {
		/* logError_st80y(0, "%s %s: v is %X.\n", tag_st80y, __func__, v); */
	} else {
		/* logError_st80y(0, "%s %s: v is null.\n", tag_st80y, __func__); */
		info->limit = 0;
		info->chunk = 0;
		info->printed = 0;
		if (info->driver_test_buff != NULL) {
			/* logError_st80y(0, "%s %s: Freeing and clearing info->driver_test_buff.\n",
			 *	tag_st80y, __func__); */
			kfree(info->driver_test_buff);
			info->driver_test_buff = NULL;
		} else {
			/* logError_st80y(0, "%s %s: info->driver_test_buff is already null.\n",
			 *	tag_st80y, __func__); */
		}
	}
}

/**
  * Struct where define and specify the functions which implements the flow for
  *writing on a sequential file
  */
static const struct seq_operations fts_seq_ops = {
	.start	= fts_seq_start,
	.next	= fts_seq_next,
	.stop	= fts_seq_stop,
	.show	= fts_seq_show
};

/**
  * This function open a sequential file
  * @param inode Inode in the file system that was called and triggered this
  * function
  * @param file file associated to the file node
  * @return error code, 0 if success
  */
static int fts_open(struct inode *inode, struct file *file)
{
	int ret;

	ret = seq_open(file, &fts_seq_ops);
	if (!ret)
		((struct seq_file *)file->private_data)->private = PDE_DATA(inode);
	return ret;
};


/*****************************************************************************/

/**************************** DRIVER TEST ************************************/

/** @addtogroup proc_file_code
  * @{
  */

/**
  * Receive the OP code and the inputs from shell when the file node is called,
  * parse it and then execute the corresponding function
  * echo cmd+parameters > /proc/fts/driver_test to execute the select command
  * cat /proc/fts/driver_test			to obtain the result into the
  * shell \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * the answer content and format strictly depend on the cmd executed. In
  * general can be: an HEX string or a byte array (e.g in case of 0xF- commands)
  * \n
  * } = end byte \n
  */
static ssize_t fts_driver_test_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
	int numberParam = 0;
	struct fts_ts_info *info = (struct fts_ts_info *)(((struct seq_file *)file->private_data)->private);
	char *p = NULL;
	char *pbuf = NULL;
	u8 *readData = NULL;
	u8 *cmd = NULL;	/* worst case needs count bytes */
	u32 *funcToTest = NULL;
	u16 byteToRead = 0;
	u32 fileSize = 0;
	u64 addr = 0;
	char path[100] = { 0 };
	int res = -1, j, index = 0;
	int size = 6;
	int temp, byte_call = 0;
	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;
	enum vts_sensor_test_result result;

	DataHeader dataHead;
	MutualSenseData compData;
	SelfSenseData comData;
	TotMutualSenseData totCompData;
	TotSelfSenseData totComData;

	u64 address;
	u64 dup_address;
	u64 mod;

	Firmware fw;
	LimitFile lim;

	info->mess.dummy = 0;
	info->mess.action = 0;
	info->mess.msg_size = 0;

	pbuf = (char *)kmalloc(count * sizeof(u8), GFP_KERNEL);
	if(pbuf == NULL){
		res = ERROR_ALLOC;
		return res;
	}

	cmd = (u8 *)kmalloc(count * sizeof(u8), GFP_KERNEL);
	if(cmd == NULL){
		kfree(pbuf);
		res = ERROR_ALLOC;
		return res;
	}

	funcToTest = (u32 *)kmalloc((count+1)/3 * sizeof(u32), GFP_KERNEL);
	if(funcToTest == NULL){
		kfree(cmd);
		kfree(pbuf);
		res = ERROR_ALLOC;
		return res;
	}

	/*for(temp = 0; temp<count; temp++){
	  *      logError_st80y(0,"%s p[%d] = %02X\n",tag_st80y, temp, p[temp]);
	  * }*/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	if (access_ok(buf, count) < OK || copy_from_user(pbuf, buf, count) != 0) {
		res = ERROR_ALLOC;
		goto END;
	}
#else
	if (access_ok(VERIFY_READ, buf, count) < OK ||
	    copy_from_user(pbuf, buf, count) != 0) {
		res = ERROR_ALLOC;
		goto END;
	}
#endif
	p = pbuf;
	if (count > MESSAGE_MIN_HEADER_SIZE - 1 && p[0] == MESSAGE_START_BYTE) {
		logError_st80y(0, "%s Enter in Byte Mode!\n", tag_st80y);
		byte_call = 1;
		info->mess.msg_size = (p[1] << 8) | p[2];
		info->mess.counter = (p[3] << 8) | p[4];
		info->mess.action = (p[5] << 8) | p[6];
		logError_st80y(0,
			 "%s Message received: size = %d, counter_id = %d, action = %04X\n",
			 tag_st80y, info->mess.msg_size, info->mess.counter, info->mess.action);
		size = MESSAGE_MIN_HEADER_SIZE + 2;	/* +2 error code */
		if (count < info->mess.msg_size || p[count - 2] != MESSAGE_END_BYTE) {
			logError_st80y(1,
				 "%s number of byte received or end byte wrong! msg_size = %d != %d, last_byte = %02X != %02X ... ERROR %08X\n",
				 tag_st80y, info->mess.msg_size, (int)count, p[count - 1],
				 MESSAGE_END_BYTE, ERROR_OP_NOT_ALLOW);
			res = ERROR_OP_NOT_ALLOW;
			goto END;
		} else {
			numberParam = info->mess.msg_size - MESSAGE_MIN_HEADER_SIZE +
				      1;	/* +1 because put the internal
						 * op code */
			size = MESSAGE_MIN_HEADER_SIZE + 2;	/* +2 send also
								 * the first 2
								 * lsb of the
								 * error code */
			switch (info->mess.action) {
			case ACTION_READ:
				/* numberParam =
				 * info->mess.msg_size-MESSAGE_MIN_HEADER_SIZE+1; */
				cmd[0] = funcToTest[0] = CMD_READ_BYTE;
				break;

			case ACTION_WRITE:
				cmd[0] = funcToTest[0] = CMD_WRITE_BYTE;
				break;

			case ACTION_WRITE_READ:
				cmd[0] = funcToTest[0] = CMD_WRITEREAD_BYTE;
				break;

			case ACTION_GET_VERSION:
				cmd[0] = funcToTest[0] = CMD_VERSION_BYTE;
				break;

			case ACTION_WRITETHENWRITEREAD:
				cmd[0] = funcToTest[0] =
						 CMD_WRITETHENWRITEREAD_BYTE;
				break;

			case ACTION_WRITEU8UX:
				cmd[0] = funcToTest[0] = CMD_WRITEU8UX_BYTE;
				break;

			case ACTION_WRITEREADU8UX:
				cmd[0] = funcToTest[0] = CMD_WRITEREADU8UX_BYTE;
				break;

			case ACTION_WRITEU8UXTHENWRITEU8UX:
				cmd[0] = funcToTest[0] =
						 CMD_WRITEU8UXTHENWRITEU8UX_BYTE;
				break;

			case ACTION_WRITEU8XTHENWRITEREADU8UX:
				cmd[0] = funcToTest[0] =
						 CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE;
				break;

			case ACTION_GET_FW:
				cmd[0] = funcToTest[0] = CMD_GETFWFILE_BYTE;
				break;

			case ACTION_GET_LIMIT:
				cmd[0] = funcToTest[0] = CMD_GETLIMITSFILE_BYTE;
				break;

			default:
				logError_st80y(1,
					 "%s Invalid Action = %d ... ERROR %08X\n",
					 tag_st80y,
					 info->mess.action, ERROR_OP_NOT_ALLOW);
				res = ERROR_OP_NOT_ALLOW;
				goto END;
			}

			if (numberParam - 1 != 0)
				memcpy(&cmd[1], &p[7], numberParam - 1);
			/* -1 because i need to exclude the cmd[0] */
		}
	} else {
		if (((count + 1) / 3) >= 1) {
			if (sscanf(p, "%02X ", &funcToTest[0]) == 1) {
				p += 3;
				cmd[0] = (u8)funcToTest[0];
				numberParam = 1;
			}
		} else {
			res = ERROR_OP_NOT_ALLOW;
			goto END;
		}

		logError_st80y(1, "%s functionToTest[0] = %02X cmd[0]= %02X\n", tag_st80y,
			 funcToTest[0], cmd[0]);
		switch (funcToTest[0]) {
		case CMD_GETFWFILE:
		case CMD_GETLIMITSFILE:
			if (count - 2 - 1 > 1) {
				if (sscanf(p, "%100s", path) == 1)
					numberParam = 2;
					/* the first byte is an hex
					 * string coded in three byte (2
					 * chars for hex and the space)
					 * and -1 for the space at the
					 * end */
			}
			break;

		default:
			for (; numberParam < (count + 1) / 3; numberParam++) {
				if (sscanf(p, "%02X ",
					&funcToTest[numberParam]) == 1) {
					p += 3;
					cmd[numberParam] =
						(u8)funcToTest[numberParam];
					logError_st80y(1,
						"%s functionToTest[%d] = %02X cmd[%d]= %02X\n",
						tag_st80y, numberParam,
						funcToTest[numberParam],
						numberParam, cmd[numberParam]);
				}

			}
		}
	}


	fw.data = NULL;
	lim.data = NULL;

	logError_st80y(1, "%s Number of Parameters = %d\n", tag_st80y, numberParam);

	/* elaborate input */
	if (numberParam >= 1) {
		/*res = st80y_disableInterrupt(info);
		  * if (res < 0)
		  * {
		  *      logError_st80y(0, "%s %s: ERROR %08X\n", tag_st80y, __func__,
		  *res);
		  *      res = (res | ERROR_DISABLE_INTER);
		  *      goto END;
		  * }*/
		switch (funcToTest[0]) {
		case CMD_VERSION_BYTE:
			logError_st80y(0, "%s %s: Get Version Byte\n", tag_st80y,
				 __func__);
			byteToRead = 2;
			info->mess.dummy = 0;
			readData = (u8 *)kmalloc(byteToRead * sizeof(u8),
						 GFP_KERNEL);
			size += byteToRead;
			if (readData != NULL) {
				readData[0] = (u8)(FTS_TS_DRV_VER >> 24);
				readData[1] = (u8)(FTS_TS_DRV_VER >> 16);
				res = OK;
				logError_st80y(0, "%s %s: Version = %02X%02X\n", tag_st80y,
					 __func__, readData[0], readData[1]);
			} else {
				res = ERROR_ALLOC;
				logError_st80y(1,
					 "%s %s: Impossible allocate memory... ERROR %08X\n",
					 tag_st80y, __func__, res);
			}
			break;


		case CMD_VERSION:
			byteToRead = 2 * sizeof(u32);
			info->mess.dummy = 0;
			readData = (u8 *)kmalloc(byteToRead * sizeof(u8),
						 GFP_KERNEL);
			u32ToU8_be(FTS_TS_DRV_VER, readData);
			fileSize = 0;
			/* first two bytes bitmask of features enabled in the
			 * IC, second two bytes bitmask of features enabled in
			 * the driver */

#ifdef LIMITS_H_FILE
			fileSize |= 0x00020000;
#endif

#ifdef USE_ONE_FILE_NODE
			fileSize |= 0x00040000;
#endif

#ifdef FW_UPDATE_ON_PROBE
			fileSize |= 0x00080000;
#endif

#ifdef PRE_SAVED_METHOD
			fileSize |= 0x00100000;
#endif

#ifdef COMPUTE_INIT_METHOD
			fileSize |= 0x00200000;
#endif

#ifdef USE_GESTURE_MASK
			fileSize |= 0x00100000;
#endif

#ifdef I2C_INTERFACE
			fileSize |= 0x00200000;
#endif

#ifdef SPI4_WIRE
			fileSize |= 0x00400000;
#endif

#ifdef PHONE_KEY	/* it is a feature enabled in the config of the chip */
			fileSize |= 0x00000100;
#endif

#ifdef GESTURE_MODE
			fromIDtoMask(FEAT_SEL_GESTURE, (u8 *)&fileSize, 4);
#endif


#ifdef GRIP_MODE
			fromIDtoMask(FEAT_SEL_GRIP, (u8 *)&fileSize, 4);
#endif

#ifdef CHARGER_MODE
			fromIDtoMask(FEAT_SEL_CHARGER, (u8 *)&fileSize, 4);
#endif

#ifdef GLOVE_MODE
			fromIDtoMask(FEAT_SEL_GLOVE, (u8 *)&fileSize, 4);
#endif


#ifdef COVER_MODE
			fromIDtoMask(FEAT_SEL_COVER, (u8 *)&fileSize, 4);
#endif

#ifdef STYLUS_MODE
			fromIDtoMask(FEAT_SEL_STYLUS, (u8 *)&fileSize, 4);
#endif

			u32ToU8_be(fileSize, &readData[4]);
			res = OK;
			size += (byteToRead * sizeof(u8));
			break;

		case CMD_WRITEREAD:
		case CMD_WRITEREAD_BYTE:
			if (numberParam >= 5) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1]
						 * byteToRead1 byteToRead0
						 * dummyByte */
				temp = numberParam - 4;
				if (cmd[numberParam - 1] == 0)
					info->mess.dummy = 0;
				else
					info->mess.dummy = 1;

				u8ToU16_be(&cmd[numberParam - 3], &byteToRead);
				logError_st80y(0, "%s bytesToRead = %d\n", tag_st80y,
					 byteToRead + info->mess.dummy);

				readData = (u8 *)kmalloc((byteToRead +
							  info->mess.dummy) *
							 sizeof(u8),
							 GFP_KERNEL);
				res = st80y_writeRead(info, &cmd[1], temp, readData,
						    byteToRead + info->mess.dummy);
				size += (byteToRead * sizeof(u8));
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITE:
		case CMD_WRITE_BYTE:
			if (numberParam >= 2) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1] */
				temp = numberParam - 1;

				res = st80y_write(info, &cmd[1], temp);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READ:
		case CMD_READ_BYTE:
			if (numberParam >= 3) {	/* need to pass: byteToRead1
						 * byteToRead0 (optional)
						 * dummyByte */
				if (numberParam == 3 || (numberParam == 4 &&
							 cmd[numberParam - 1] ==
							 0))
					info->mess.dummy = 0;
				else
					info->mess.dummy = 1;
				u8ToU16_be(&cmd[1], &byteToRead);
				readData = (u8 *)kmalloc((byteToRead +
							  info->mess.dummy) *
							 sizeof(u8),
							 GFP_KERNEL);
				res = st80y_read(info, readData, byteToRead +
					       info->mess.dummy);
				size += (byteToRead * sizeof(u8));
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITETHENWRITEREAD:
		case CMD_WRITETHENWRITEREAD_BYTE:
			/* need to pass: cmdSize1 cmdSize2 cmd1[0] cmd1[1] …
			 * cmd1[cmdSize1-1] cmd2[0] cmd2[1] … cmd2[cmdSize2-1]
			 *  byteToRead1 byteToRead0 */
			if (numberParam >= 6) {
				u8ToU16_be(&cmd[numberParam - 2], &byteToRead);
				readData = (u8 *)kmalloc(byteToRead *
							 sizeof(u8),
							 GFP_KERNEL);
				res = st80y_writeThenWriteRead(info, &cmd[3], cmd[1],
							     &cmd[3 +
								  (int)cmd[1]],
							     cmd[2], readData,
							     byteToRead);
				size += (byteToRead * sizeof(u8));
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UX:
		case CMD_WRITEU8UX_BYTE:
			/* need to pass: cmd addrSize addr[0] … addr[addrSize-1]
			 * data[0] data[1] … */
			if (numberParam >= 4) {
				if (cmd[2] <= sizeof(u64)) {
					u8ToU64_be(&cmd[3], &addr, cmd[2]);
					logError_st80y(0, "%s addr = %016llX %ld\n",
						 tag_st80y, addr, (long int)addr);
					res = st80y_writeU8UX(info, cmd[1], cmd[2],
							    addr, &cmd[3 +
								       cmd[2]],
							    (numberParam -
							     cmd
							     [
								     2] - 3));
				} else {
					logError_st80y(1, "%s Wrong address size!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_WRITEREADU8UX:
		case CMD_WRITEREADU8UX_BYTE:
			/* need to pass: cmd addrSize addr[0] … addr[addrSize-1]
			 * byteToRead1 byteToRead0 hasDummyByte */
			if (numberParam >= 6) {
				if (cmd[2] <= sizeof(u64)) {
					u8ToU64_be(&cmd[3], &addr, cmd[2]);
					u8ToU16_be(&cmd[numberParam - 3],
						   &byteToRead);
					readData = (u8 *)kmalloc(byteToRead *
								 sizeof(u8),
								 GFP_KERNEL);
					logError_st80y(0,
						 "%s addr = %016llX byteToRead = %d\n",
						 tag_st80y, addr, byteToRead);
					res = st80y_writeReadU8UX(info, cmd[1], cmd[2],
								addr, readData,
								byteToRead,
								cmd[numberParam
								    - 1]);
					size += (byteToRead * sizeof(u8));
				} else {
					logError_st80y(1, "%s Wrong address size!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UXTHENWRITEU8UX:
		case CMD_WRITEU8UXTHENWRITEU8UX_BYTE:
			/* need to pass: cmd1 addrSize1 cmd2 addrSize2 addr[0] …
			 * addr[addrSize1+addrSize2-1] data[0] data[1] … */
			if (numberParam >= 6) {
				if ((cmd[2] + cmd[4]) <= sizeof(u64)) {
					u8ToU64_be(&cmd[5], &addr, cmd[2] +
						   cmd[4]);

					logError_st80y(0, "%s addr = %016llX %lld\n",
						 tag_st80y, addr, addr);
					res = st80y_writeU8UXthenWriteU8UX(info,
									cmd[1],
									 cmd[2],
									 cmd[3],
									 cmd[4],
									 addr,
									&cmd[5 +
									cmd[2] +
									cmd[4]],
									(numberParam -
									cmd[2] -
									cmd[4] -
						5));
				} else {
					logError_st80y(1, "%s Wrong address size!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_WRITEU8UXTHENWRITEREADU8UX:
		case CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE:
			/* need to pass: cmd1 addrSize1 cmd2 addrSize2 addr[0] …
			 * addr[addrSize1+addrSize2-1]  byteToRead1 byteToRead0
			 * hasDummybyte */
			if (numberParam >= 8) {
				if ((cmd[2] + cmd[4]) <= sizeof(u64)) {
					u8ToU64_be(&cmd[5], &addr, cmd[2] +
						   cmd[4]);
					logError_st80y(1,
						 "%s %s: cmd[5] = %02X, addr =  %016llX\n",
						 tag_st80y, __func__, cmd[5], addr);
					u8ToU16_be(&cmd[numberParam - 3],
						   &byteToRead);
					readData = (u8 *)kmalloc(byteToRead *
								 sizeof(u8),
								 GFP_KERNEL);
					res = st80y_writeU8UXthenWriteReadU8UX(info,
						cmd[1], cmd[2], cmd[3], cmd[4],
						addr,
						readData, byteToRead,
						cmd[numberParam - 1]);
					size += (byteToRead * sizeof(u8));
				} else {
					logError_st80y(1,
						 "%s Wrong total address size!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_CHANGE_OUTPUT_MODE:
			/* need to pass: bin_output */
			if (numberParam >= 2) {
				info->bin_output = cmd[1];
				logError_st80y(0,
					 "%s Setting Scriptless output mode: %d\n",
					 tag_st80y,
					 info->bin_output);
				res = OK;
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FWWRITE:
			if (numberParam >= 3) {	/* need to pass: cmd[0]  cmd[1]
						 * … cmd[cmdLength-1] */
				if (numberParam >= 2) {
					temp = numberParam - 1;
					res = st80y_writeFwCmd(info, &cmd[1], temp);
				} else {
					logError_st80y(1, "%s Wrong parameters!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_INTERRUPT:
			/* need to pass: enable */
			if (numberParam >= 2) {
				if (cmd[1] == 1)
					res = st80y_enableInterrupt(info);
				else
					res = st80y_disableInterrupt(info);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SETSCANMODE:
			/* need to pass: scanMode option */
			if (numberParam >= 3)
				res = setScanMode(info, cmd[1], cmd[2]);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SAVEMPFLAG:
			/* need to pass: mpflag */
			if (numberParam == 2)
				res = saveMpFlag(info, cmd[1]);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_READCONFIG:
			if (numberParam == 5) {	/* need to pass: addr[0]
						 *  addr[1] byteToRead1
						 * byteToRead0 */
				byteToRead = ((funcToTest[3] << 8) |
					      funcToTest[4]);
				readData = (u8 *)kmalloc(byteToRead *
							 sizeof(u8),
							 GFP_KERNEL);
				res = readConfig(info, (u16)((((u8)funcToTest[1] &
							 0x00FF) << 8) +
						       ((u8)funcToTest[2] &
							0x00FF)),
						 readData, byteToRead);
				size += (byteToRead * sizeof(u8));
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_POLLFOREVENT:
			if (numberParam >= 5) {	/* need to pass: eventLength
						 * event[0] event[1] …
						 * event[eventLength-1]
						 * timeTowait1 timeTowait0 */
				temp = (int)funcToTest[1];
				if (numberParam == 5 + (temp - 1) && temp !=
				    0) {
					readData = (u8 *)kmalloc(
						FIFO_EVENT_SIZE * sizeof(u8),
						GFP_KERNEL);
					res = st80y_pollForEvent(info,
						(int *)&funcToTest[2], temp,
						readData,
						((funcToTest[temp + 2] &
						  0x00FF) << 8) +
						(funcToTest[temp + 3] &
						 0x00FF));
					if (res >= OK)
						res = OK;	/* st80y_pollForEvent
								 * return the
								 * number of
								 * error found
								 * */
					size += (FIFO_EVENT_SIZE * sizeof(u8));
					byteToRead = FIFO_EVENT_SIZE;
				} else {
					logError_st80y(1, "%s Wrong parameters!\n",
						 tag_st80y);
					res = ERROR_OP_NOT_ALLOW;
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SYSTEMRESET:
			res = st80y_system_reset(info);

			break;

		case CMD_READSYSINFO:
			if (numberParam == 2)	/* need to pass: doRequest */
				res = readSysInfo(info, funcToTest[1]);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_CLEANUP:/* TOUCH ENABLE/DISABLE */
			if (numberParam == 2)	/* need to pass: enableTouch */
				res = st80y_cleanUp(info, funcToTest[1]);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case CMD_GETFORCELEN:	/* read number Tx channels */
			temp = st80y_getForceLen(info);
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof(u8));
				res = OK;
			}
			break;

		case CMD_GETSENSELEN:	/* read number Rx channels */
			temp = st80y_getSenseLen(info);
			if (temp < OK)
				res = temp;
			else {
				size += (1 * sizeof(u8));
				res = OK;
			}
			break;


		case CMD_GETMSFRAME:
			if (numberParam == 2) {
				/*logError_st80y(0, "%s Get 1 MS Frame\n", tag_st80y);
				setScanMode(SCAN_MODE_ACTIVE, 0xFF);
				msleep(WAIT_FOR_FRESH_FRAMES);
				setScanMode(SCAN_MODE_ACTIVE, 0x00);
				msleep(WAIT_AFTER_SENSEOFF);
				  st80y_flushFIFO(info); */ /* delete the events related to
				 * some touch (allow to call this function while
				 * touching the screen without having a flooding
				 * of the FIFO) */
				res = st80y_getMSFrame3(info, (MSFrameType)cmd[1],
						  &frameMS);
				if (res < 0)
					logError_st80y(0,
						 "%s Error while taking the MS frame... ERROR %08X\n",
						 tag_st80y, res);

				else {
					logError_st80y(0,
						 "%s The frame size is %d words\n",
						 tag_st80y,
						 res);
					size += (res * sizeof(short) + 2);
					/* +2 to add force and sense channels */
					/* set res to OK because if getMSFrame
					 * is
					  * successful res = number of words
					  *read
					  */
					res = OK;
					st80y_print_frame_short("MS frame =",
							  st80y_array1dTo2d_short(
								  frameMS.
								  node_data,
								  frameMS
								  .
								  node_data_size,
								  frameMS
								  .header.
								  sense_node),
							  frameMS.header.
							  force_node,
							  frameMS.header.
							  sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		/*read self raw*/
		case CMD_GETSSFRAME:
			if (numberParam == 2) {
				logError_st80y(0, "%s Get 1 SS Frame\n", tag_st80y);
				/*  st80y_flushFIFO(info); //delete the events related to
				 * some touch (allow to call this function while
				 * touching the screen without having a flooding
				 * of the FIFO) */
				/* setScanMode(SCAN_MODE_ACTIVE, 0xFF);
				msleep(WAIT_FOR_FRESH_FRAMES);
				setScanMode(SCAN_MODE_ACTIVE, 0x00);
				msleep(WAIT_AFTER_SENSEOFF);  */
				res = st80y_getSSFrame3(info, (SSFrameType)cmd[1],
						  &frameSS);

				if (res < OK)
					logError_st80y(0,
						 "%s Error while taking the SS frame... ERROR %08X\n",
						 tag_st80y, res);

				else {
					logError_st80y(0,
						 "%s The frame size is %d words\n",
						 tag_st80y,
						 res);
					size += (res * sizeof(short) + 2);
					/* +2 to add force and sense channels */
					/* set res to OK because if getMSFrame
					 * is
					  * successful res = number of words
					  *read
					  */
					res = OK;
					st80y_print_frame_short("SS force frame =",
							  st80y_array1dTo2d_short(
								  frameSS.
								  force_data,
								  frameSS
								  .header.
								  force_node,
								  1),
							  frameSS.header.
							  force_node, 1);
					st80y_print_frame_short("SS sense frame =",
							  st80y_array1dTo2d_short(
								  frameSS.
								  sense_data,
								  frameSS
								  .header.
								  sense_node,
								  frameSS
								  .header.
								  sense_node),
							  1,
							  frameSS.header.
							  sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETSYNCFRAME:
			/* need to pass: frameType (this parameter can be
			 * one of LOAD_SYNC_FRAME_X) */
			if (numberParam == 2) {
				logError_st80y(1, "%s Reading Sync Frame...\n", tag_st80y);
				res = st80y_getSyncFrame(info, cmd[1], &frameMS, &frameSS);
				if (res < OK)
					logError_st80y(0,
						 "%s Error while taking the Sync Frame frame... ERROR %08X\n",
						 tag_st80y, res);

				else {
					logError_st80y(0,
						 "%s The total frames size is %d words\n",
						 tag_st80y, res);
					size += (res * sizeof(short) + 4);
					/* +4 to add force and sense channels
					 * for MS and SS */
					/* set res to OK because if st80y_getSyncFrame
					 * is
					  * successful res = number of words
					  *read
					  */
					res = OK;

					st80y_print_frame_short("MS frame =",
							  st80y_array1dTo2d_short(
								  frameMS.
								  node_data,
								  frameMS
								  .
								  node_data_size,
								  frameMS
								  .header.
								  sense_node),
							  frameMS.header.
							  force_node,
							  frameMS.header.
							  sense_node);
					st80y_print_frame_short("SS force frame =",
							  st80y_array1dTo2d_short(
								  frameSS.
								  force_data,
								  frameSS
								  .header.
								  force_node,
								  1),
							  frameSS.header.
							  force_node, 1);
					st80y_print_frame_short("SS sense frame =",
							  st80y_array1dTo2d_short(
								  frameSS.
								  sense_data,
								  frameSS
								  .header.
								  sense_node,
								  frameSS
								  .header.
								  sense_node),
							  1,
							  frameSS.header.
							  sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_REQCOMPDATA:	/* request comp data */
			if (numberParam == 2) {
				logError_st80y(0,
					 "%s Requesting Compensation Data\n",
					 tag_st80y);
				res = st80y_requestCompensationData(info, cmd[1]);

				if (res < OK)
					logError_st80y(0,
						 "%s Error requesting compensation data ERROR %08X\n",
						 tag_st80y, res);
				else
					logError_st80y(0,
						 "%s Requesting Compensation Data Finished!\n",
						 tag_st80y);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READCOMPDATAHEAD:	/* read comp data header */
			if (numberParam == 2) {
				logError_st80y(0,
					 "%s Requesting Compensation Data\n",
					 tag_st80y);
				res = st80y_requestCompensationData(info, cmd[1]);
				if (res < OK)
					logError_st80y(0,
						 "%s Error requesting compensation data ERROR %08X\n",
						 tag_st80y, res);
				else {
					logError_st80y(0,
						 "%s Requesting Compensation Data Finished!\n",
						 tag_st80y);
					res = st80y_readCompensationDataHeader(info,
						(u8)funcToTest[1], &dataHead,
						&address);
					if (res < OK)
						logError_st80y(0,
							 "%s Read Compensation Data Header ERROR %08X\n",
							 tag_st80y, res);
					else {
						logError_st80y(0,
							 "%s Read Compensation Data Header OK!\n",
							 tag_st80y);
						size += (1 * sizeof(u8));
					}
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_READMSCOMPDATA:/* read mutual comp data */
			if (numberParam == 2) {
				logError_st80y(0, "%s Get MS Compensation Data\n",
					 tag_st80y);
				res = st80y_readMutualSenseCompensationData(info, cmd[1],
								      &compData);

				if (res < OK)
					logError_st80y(0,
						 "%s Error reading MS compensation data ERROR %08X\n",
						 tag_st80y, res);
				else {
					logError_st80y(0,
						 "%s MS Compensation Data Reading Finished!\n",
						 tag_st80y);
					size = ((compData.node_data_size + 10) *
						sizeof(i8));
					print_frame_i8("MS Data (Cx2) =",
						       array1dTo2d_i8(
							       compData.
							       node_data,
							       compData.
							       node_data_size,
							       compData.
							       header.sense_node),
						       compData.header.
						       force_node,
						       compData.header.
						       sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READSSCOMPDATA:
			if (numberParam == 2) {	/* read self comp data */
				logError_st80y(0, "%s Get SS Compensation Data...\n",
					 tag_st80y);
				res = st80y_readSelfSenseCompensationData(info, cmd[1],
								    &comData);
				if (res < OK)
					logError_st80y(0,
						 "%s Error reading SS compensation data ERROR %08X\n",
						 tag_st80y, res);
				else {
					logError_st80y(0,
						 "%s SS Compensation Data Reading Finished!\n",
						 tag_st80y);
					size = ((comData.header.force_node +
						 comData.header.sense_node) *
						2 + 15) *
					       sizeof(i8);
					print_frame_i8("SS Data Ix2_fm = ",
						       array1dTo2d_i8(
							       comData.ix2_fm,
							       comData.
							       header.force_node,
							       comData.
							       header.force_node),
						       1,
						       comData.header.force_node);
					print_frame_i8("SS Data Cx2_fm = ",
						       array1dTo2d_i8(
							       comData.cx2_fm,
							       comData.
							       header.force_node,
							       comData.
							       header.force_node),
						       1,
						       comData.header.force_node);
					print_frame_i8("SS Data Ix2_sn = ",
						       array1dTo2d_i8(
							       comData.ix2_sn,
							       comData.
							       header.sense_node,
							       comData.
							       header.sense_node),
						       1,
						       comData.header.sense_node);
					print_frame_i8("SS Data Cx2_sn = ",
						       array1dTo2d_i8(
							       comData.cx2_sn,
							       comData.
							       header.sense_node,
							       comData.
							       header.sense_node),
						       1,
						       comData.header.sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READTOTMSCOMPDATA:	/* read mutual comp data */
			if (numberParam == 2) {
				logError_st80y(0,
					 "%s Get TOT MS Compensation Data\n",
					 tag_st80y);
				res = readTotMutualSenseCompensationData(info, cmd[1],
								&totCompData);

				if (res < OK)
					logError_st80y(0,
						 "%s Error reading TOT MS compensation data ERROR %08X\n",
						 tag_st80y, res);
				else {
					logError_st80y(0,
						 "%s TOT MS Compensation Data Reading Finished!\n",
						 tag_st80y);
					size = (totCompData.node_data_size *
						sizeof(short) + 9);
					st80y_print_frame_short("MS Data (TOT Cx) =",
							  st80y_array1dTo2d_short(
								  totCompData.
								  node_data,
								  totCompData.
								  node_data_size,
								  totCompData
								  .header.
								  sense_node),
							  totCompData.header.
							  force_node,
							  totCompData.header.
							  sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_READTOTSSCOMPDATA:
			if (numberParam == 2) {	/* read self comp data */
				logError_st80y(0,
					 "%s Get TOT SS Compensation Data...\n",
					 tag_st80y);
				res = readTotSelfSenseCompensationData(info, cmd[1], &totComData);
				if (res < OK)
					logError_st80y(0,
						 "%s Error reading TOT SS compensation data ERROR %08X\n",
						 tag_st80y, res);
				else {
					logError_st80y(0,
						 "%s TOT SS Compensation Data Reading Finished!\n",
						 tag_st80y);
					size = ((totComData.header.force_node +
						 totComData.header.sense_node) *
						2 *
						sizeof(short) + 9);
					print_frame_u16("SS Data TOT Ix_fm = ",
							array1dTo2d_u16(
								totComData.ix_fm,
								totComData
								.header.
								force_node,
								totComData
								.header.
								force_node), 1,
							totComData.header.
							force_node);
					st80y_print_frame_short(
						"SS Data TOT Cx_fm = ",
						st80y_array1dTo2d_short(
							totComData.cx_fm,
							totComData.
							header.force_node,
							totComData.
							header.force_node), 1,
						totComData.header.force_node);
					print_frame_u16("SS Data TOT Ix_sn = ",
							array1dTo2d_u16(
								totComData.ix_sn,
								totComData
								.header.
								sense_node,
								totComData
								.header.
								sense_node), 1,
							totComData.header.
							sense_node);
					st80y_print_frame_short(
						"SS Data TOT Cx_sn = ",
						st80y_array1dTo2d_short(
							totComData.cx_sn,
							totComData.
							header.sense_node,
							totComData.
							header.sense_node), 1,
						totComData.header.sense_node);
				}
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_GETFWVER:
			size += (EXTERNAL_RELEASE_INFO_SIZE)*sizeof(u8);
			break;

		case CMD_FLASHUNLOCK:
			res = st80y_flash_unlock(info);
			if (res < OK)
				logError_st80y(1,
					 "%s Impossible Unlock Flash ERROR %08X\n",
					 tag_st80y,
					 res);
			else
				logError_st80y(0, "%s Flash Unlock OK!\n", tag_st80y);
			break;

		case CMD_READFWFILE:
			if (numberParam == 2) {	/* read fw file */
				logError_st80y(0, "%s Reading FW File...\n", tag_st80y);
				res = st80y_readFwFile(info, PATH_FILE_FW, &fw,
						 funcToTest[1]);
				if (res < OK)
					logError_st80y(0,
						 "%s Error reading FW File ERROR %08X\n",
						 tag_st80y, res);
				else
					logError_st80y(0,
						 "%s Read FW File Finished!\n",
						 tag_st80y);
				kfree(fw.data);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FLASHPROCEDURE:
			if (numberParam == 3) {	/* flashing procedure */
				logError_st80y(0,
					 "%s Starting Flashing Procedure...\n",
					 tag_st80y);
				res = st80y_flashProcedure(info, PATH_FILE_FW, cmd[1],
						     cmd[2]);
				if (res < OK)
					logError_st80y(0,
						 "%s Error during flash procedure ERROR %08X\n",
						 tag_st80y, res);
				else
					logError_st80y(0,
						 "%s Flash Procedure Finished!\n",
						 tag_st80y);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FLASHERASEUNLOCK:
			res = st80y_flash_erase_unlock(info);
			if (res < OK)
				logError_st80y(0,
					 "%s Error during flash erase unlock... ERROR %08X\n",
					 tag_st80y, res);
			else
				logError_st80y(0, "%s Flash Erase Unlock Finished!\n",
					 tag_st80y);
			break;

		case CMD_FLASHERASEPAGE:
			if (numberParam == 2) {	/* need to pass: keep_cx */
				logError_st80y(0, "%s Reading FW File...\n", tag_st80y);
				res = st80y_readFwFile(info, PATH_FILE_FW, &fw,
						 funcToTest[1]);
				if (res < OK)
					logError_st80y(0,
						 "%s Error reading FW File ERROR %08X\n",
						 tag_st80y, res);
				else
					logError_st80y(0,
						 "%s Read FW File Finished!\n",
						 tag_st80y);				
				logError_st80y(0,
					 "%s Starting Flashing Page Erase...\n",
					 tag_st80y);
				res = st80y_flash_erase_page_by_page(info, cmd[1], &fw);
				if (res < OK)
					logError_st80y(0,
						 "%s Error during flash page erase... ERROR %08X\n",
						 tag_st80y, res);
				else
					logError_st80y(0,
						 "%s Flash Page Erase Finished!\n",
						 tag_st80y);
				kfree(fw.data);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/*ITO TEST*/
		case CMD_ITOTEST:
			res = st80y_production_test_ito(info, LIMITS_FILE, &info->tests_st80y);
			break;

		/*Initialization*/
		case CMD_INITTEST:
			if (numberParam == 2)
				res = st80y_production_test_initialization(info, cmd[1]);

			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;


		case CMD_MSRAWTEST:	/* MS Raw DATA TEST */
			if (numberParam == 2)	/* need to specify if stopOnFail
						 * */
				res = st80y_production_test_ms_raw(info, LIMITS_FILE,
							     cmd[1], &info->tests_st80y);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_MSINITDATATEST:/* MS CX DATA TEST */
			if (numberParam == 2)	/* need stopOnFail */
				res = st80y_production_test_ms_cx(info, LIMITS_FILE, cmd[1],
							    &info->tests_st80y);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSRAWTEST:	/* SS RAW DATA TEST */
			if (numberParam == 2) /* need stopOnFail */
				res = st80y_production_test_ss_raw(info, LIMITS_FILE,
							     cmd[1], &info->tests_st80y);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_SSINITDATATEST:/* SS IX CX DATA TEST */
			if (numberParam == 2)	/* need stopOnFail */
				res = st80y_production_test_ss_ix_cx(info, LIMITS_FILE,
							       cmd[1], &info->tests_st80y);
			else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/*PRODUCTION TEST*/
		case CMD_MAINTEST:
			if (numberParam >= 3) {	/* need to specify if stopOnFail
						 * saveInit and
						 * mpflag(optional)*/
				if (numberParam == 3)
					res = st80y_production_test_main(info, LIMITS_FILE,
							   cmd[1],
							   cmd[2], &info->tests_st80y,
							   MP_FLAG_OTHERS,
							   &result);
				else
					res = st80y_production_test_main(info, LIMITS_FILE,
							   cmd[1],
							   cmd[2], &info->tests_st80y,
							   cmd[3],
							   &result);
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_FREELIMIT:
			res = freeCurrentLimitsFile(info);
			break;

		case CMD_POWERCYCLE:
			res = st80y_chip_powercycle(info);
			break;

		case CMD_GETLIMITSFILE:
			/* need to pass: path(optional) return error code +
			 * number of byte read otherwise GUI could not now how
			 * many byte read */
			if (numberParam >= 1) {
				lim.data = NULL;
				lim.size = 0;
				if (numberParam == 1)
					res = getLimitsFile(info, LIMITS_FILE, &lim);
				else
					res = getLimitsFile(info, path, &lim);
				readData = lim.data;
				fileSize = lim.size;
				size += (fileSize * sizeof(u8));
				if (byte_call == 1)
					size += sizeof(u32);	/* transmit as
								 * first 4 bytes
								 * the size */
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETLIMITSFILE_BYTE:
			/* need to pass: byteToRead1 byteToRead0 */
			if (numberParam >= 3) {
				lim.data = NULL;
				lim.size = 0;

				u8ToU16_be(&cmd[1], &byteToRead);
				addr = ((u64)byteToRead) * 4;	/* number of
								 * words */

				res = getLimitsFile(info, LIMITS_FILE, &lim);

				readData = lim.data;
				fileSize = lim.size;

				if (fileSize > addr) {
					logError_st80y(1,
						 "%s Limits dimension expected by Host is less than actual size: expected = %d, real = %d\n",
						 tag_st80y, byteToRead, fileSize);
					res = ERROR_OP_NOT_ALLOW;
				}

				size += (addr * sizeof(u8));
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETFWFILE:
			/* need to pass: from (optional) otherwise select the
			 * approach chosen at compile time */
			if (numberParam >= 1) {
				if (numberParam == 1)
					res = st80y_getFWdata(info, PATH_FILE_FW, &readData,
							&fileSize);
				else
					res = st80y_getFWdata(info, path, &readData,
							&fileSize);

				size += (fileSize * sizeof(u8));
				if (byte_call == 1)
					size += sizeof(u32);	/* transmit as
								 * first 4 bytes
								 * the size */
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		case CMD_GETFWFILE_BYTE:
			/* need to pass: byteToRead1 byteToRead0 */
			if (numberParam == 3) {
				u8ToU16_be(&cmd[1], &byteToRead);
				addr = ((u64)byteToRead) * 4;	/* number of
								 * words */


				res = st80y_getFWdata(info, PATH_FILE_FW, &readData,
						&fileSize);
				if (fileSize > addr) {
					logError_st80y(1,
						 "%s FW dimension expected by Host is less than actual size: expected = %d, real = %d\n",
						 tag_st80y, byteToRead, fileSize);
					res = ERROR_OP_NOT_ALLOW;
				}

				size += (addr * sizeof(u8));	/* return always
								 * the amount
								 * requested by
								 * host, if real
								 * size is
								 * smaller, the
								 * data are
								 * padded to
								 * zero */
			} else {
				logError_st80y(1, "%s Wrong number of parameters!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;

		/* finish all the diagnostic command with a goto ERROR in order
		 * to skip the modification on info->driver_test_buff */
		/* remember to set properly the info->limit and info->printed variables in
		 * order to make the seq_file logic to work */
		case CMD_DIAGNOSTIC:
			index = 0;
			size = 0;
			fileSize = 256 * 1024 * sizeof(char);
			info->driver_test_buff = (u8 *)kzalloc(fileSize, GFP_KERNEL);
			readData = (u8 *)kmalloc((ERROR_DUMP_ROW_SIZE *
						  ERROR_DUMP_COL_SIZE) *
						 sizeof(u8), GFP_KERNEL);
			if (info->driver_test_buff == NULL || readData == NULL) {
				res = ERROR_ALLOC;
				logError_st80y(1,
					 "%s Impossible allocate memory for buffers! ERROR %08X!\n",
					 tag_st80y, res);
				goto END;
			}
			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "DIAGNOSTIC TEST:\n1) I2C Test: ");
			index += j;

			res = st80y_writeReadU8UX(info, FTS_CMD_HW_REG_R,
						ADDR_SIZE_HW_REG, ADDR_DCHIP_ID,
						(u8 *)&temp, 2,
						DUMMY_HW_REG);
			if (res < OK) {
				logError_st80y(1,
					 "%s Error during I2C test: ERROR %08X!\n",
					 tag_st80y,
					 res);
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index, "ERROR %08X\n",
					     res);
				index += j;
				res = ERROR_OP_NOT_ALLOW;
				goto END_DIAGNOSTIC;
			}

			temp &= 0xFFFF;
			logError_st80y(1, "%s Chip ID = %04X!\n", tag_st80y, temp);
			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "DATA = %04X, expected = %02X%02X\n",
				     temp, DCHIP_ID_1,
				     DCHIP_ID_0);
			index += j;
			if (temp != ((DCHIP_ID_1 << 8) | DCHIP_ID_0)) {
				logError_st80y(1,
					 "%s Wrong CHIP ID, Diagnostic failed!\n",
					 tag_st80y);
				res = ERROR_OP_NOT_ALLOW;
				goto END_DIAGNOSTIC;
			}

			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "Present Driver Mode: %08X\n",
				     info->mode);
			index += j;

			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "2) FW running: Sensing On...");
			index += j;
			logError_st80y(1, "%s Sensing On!\n", tag_st80y);
			readData[0] = FTS_CMD_SCAN_MODE;
			readData[1] = SCAN_MODE_ACTIVE;
			readData[2] = 0x1;
			st80y_write(info, readData, 3);
			res = st80y_checkEcho(info, readData, 3);
			if (res < OK) {
				logError_st80y(1,
					 "%s No Echo received.. ERROR %08X !\n",
					 tag_st80y,
					 res);
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index,
					     "No echo found... ERROR %08X!\n",
					     res);
				index += j;
				goto END_DIAGNOSTIC;
			} else {
				logError_st80y(1, "%s Echo FOUND... OK!\n", tag_st80y);
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index,
					     "Echo FOUND... OK!\n");
				index += j;
			}

			logError_st80y(1, "%s Reading Frames...!\n", tag_st80y);
			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "3) Read Frames:\n");
			index += j;
			for (temp = 0; temp < DIAGNOSTIC_NUM_FRAME; temp++) {
				logError_st80y(1, "%s Iteration n. %d...\n", tag_st80y,
					 temp + 1);
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index,
					     "Iteration n. %d...\n", temp +
					     1);
				index += j;
				for (addr = 0; addr < 3; addr++) {
					switch (addr) {
					case 0:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"MS RAW FRAME =");
						index += j;
						res |= st80y_getMSFrame3(info, MS_RAW,
								   &frameMS);
						break;
					case 2:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"MS STRENGTH FRAME =");
						index += j;
						res |= st80y_getMSFrame3(info, MS_STRENGTH,
								   &frameMS);
						break;
					case 1:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"MS BASELINE FRAME =");
						index += j;
						res |= st80y_getMSFrame3(info, MS_BASELINE,
								   &frameMS);
						break;
					}
					if (res < OK) {
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"No data! ERROR %08X\n",
							res);
						index += j;
					} else {
						for (address = 0; address <
						     frameMS.node_data_size;
						     address++) {
							dup_address = address;
							mod = do_div(dup_address, frameMS.header.sense_node);
							if (mod == 0) {
								j = snprintf(
									&
									info->driver_test_buff
									[index],
									fileSize
									-
									index,
									"\n");
								index += j;
							}
							j = snprintf(
								&
								info->driver_test_buff
								[index],
								fileSize -
								index,
								"%5d, ",
								frameMS.
								node_data[
									address]);
							index += j;
						}
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index, "\n");
						index += j;
					}
					if (frameMS.node_data != NULL)
						kfree(frameMS.node_data);
				}
				for (addr = 0; addr < 3; addr++) {
					switch (addr) {
					case 0:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"SS RAW FRAME =\n");
						index += j;
						res |= st80y_getSSFrame3(info, SS_RAW,
								   &frameSS);
						break;
					case 2:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"SS STRENGTH FRAME =\n");
						index += j;
						res |= st80y_getSSFrame3(info, SS_STRENGTH,
								   &frameSS);
						break;
					case 1:
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"SS BASELINE FRAME =\n");
						index += j;
						res |= st80y_getSSFrame3(info, SS_BASELINE,
								   &frameSS);
						break;
					}
					if (res < OK) {
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"No data! ERROR %08X\n",
							res);
						index += j;
					} else {
						for (address = 0; address <
						     frameSS.header.force_node;
						     address++) {
							j = snprintf(
								&
								info->driver_test_buff
								[index],
								fileSize -
								index,
								"%d\n",
								frameSS.
								force_data[
									address]);

							index += j;
						}
						for (address = 0; address <
						     frameSS.header.sense_node;
						     address++) {
							j = snprintf(
								&
								info->driver_test_buff
								[index],
								fileSize -
								index,
								"%d, ",
								frameSS.
								sense_data[
									address]);

							index += j;
						}
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index, "\n");
						index += j;
					}
					if (frameSS.force_data != NULL)
						kfree(frameSS.force_data);
					if (frameSS.sense_data != NULL)
						kfree(frameSS.sense_data);
				}
			}


			logError_st80y(1, "%s Reading error info...\n", tag_st80y);
			j = snprintf(&info->driver_test_buff[index], fileSize - index,
				     "4) FW INFO DUMP: ");
			index += j;
			temp = st80y_dumpErrorInfo(info, readData, ERROR_DUMP_ROW_SIZE *
					     ERROR_DUMP_COL_SIZE);
			/* OR to detect if there are failures also in the
			 * previous reading of frames and write the correct
			 * result */
			if (temp < OK) {
				logError_st80y(1,
					 "%s Error during dump: ERROR %08X!\n",
					 tag_st80y,
					 res);
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index, "ERROR %08X\n",
					     temp);
				index += j;
			} else {
				logError_st80y(1, "%s DUMP OK!\n", tag_st80y);
				for (temp = 0; temp < ERROR_DUMP_ROW_SIZE *
				     ERROR_DUMP_COL_SIZE; temp++) {
					if (temp % ERROR_DUMP_COL_SIZE == 0) {
						j = snprintf(
							&info->driver_test_buff[index],
							fileSize - index,
							"\n%2d - ",
							temp /
							ERROR_DUMP_COL_SIZE);
						index += j;
					}
					j = snprintf(&info->driver_test_buff[index],
						     fileSize - index, "%02X ",
						     readData[temp]);
					index += j;
				}
			}
			res |= temp;

END_DIAGNOSTIC:
			if (res < OK) {
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index,
					     "\nRESULT = FAIL\n");
				index += j;
			} else {
				j = snprintf(&info->driver_test_buff[index],
					     fileSize - index,
					     "\nRESULT = FINISHED\n");
				index += j;
			}
			/* the sting is already terminated with the null char by
			 * snprintf */
			info->limit = index;
			info->printed = 0;
			goto ERROR;
			break;

		default:
			logError_st80y(1, "%s COMMAND ID NOT VALID!!!\n", tag_st80y);
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		/*res2 =  st80y_enableInterrupt(info);
		 * the interrupt was disabled on purpose in this node because it
		 * can be used for testing procedure and between one step and
		 * another the interrupt wan to be kept disabled
		  * if (res2 < 0) {
		  *      logError_st80y(0, "%s stm_driver_test_show: ERROR %08X\n",
		  *tag_st80y, (res2 | ERROR_ENABLE_INTER));
		  * }*/
	} else {
		logError_st80y(1,
			 "%s NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n",
			 tag_st80y);
		res = ERROR_OP_NOT_ALLOW;
	}

END:	/* here start the reporting phase, assembling the data to send in the
	 * file node */
	if (info->driver_test_buff != NULL) {
		logError_st80y(1,
			 "%s Consecutive echo on the file node, free the buffer with the previous result\n",
			 tag_st80y);
		kfree(info->driver_test_buff);
	}

	if (byte_call == 0) {
		size *= 2;
		size += 2;	/* add \n and \0 (terminator char) */
	} else {
		if (info->bin_output != 1) {
			size *= 2; /* need to code each byte as HEX string */
			size -= 1;	/* start byte is just one, the extra
					 * byte of end byte taken by \n */
		} else
			size += 1;	/* add \n */
	}

	logError_st80y(0, "%s Size = %d\n", tag_st80y, size);
	info->driver_test_buff = (u8 *)kzalloc(size, GFP_KERNEL);
	logError_st80y(0, "%s Finish to allocate memory!\n", tag_st80y);
	if (info->driver_test_buff == NULL) {
		logError_st80y(0,
			 "%s Unable to allocate info->driver_test_buff! ERROR %08X\n",
			 tag_st80y,
			 ERROR_ALLOC);
		goto ERROR;
	}

	if (byte_call == 0) {
		index = 0;
		snprintf(&info->driver_test_buff[index], 3, "{ ");
		index += 2;
		snprintf(&info->driver_test_buff[index], 9, "%08X", res);

		index += 8;
		if (res >= OK) {
			/*all the other cases are already fine printing only the
			 * res.*/
			switch (funcToTest[0]) {
			case CMD_VERSION:
			case CMD_READ:
			case CMD_WRITEREAD:
			case CMD_WRITETHENWRITEREAD:
			case CMD_WRITEREADU8UX:
			case CMD_WRITEU8UXTHENWRITEREADU8UX:
			case CMD_READCONFIG:
			case CMD_POLLFOREVENT:
				/* logError_st80y(0, "%s Data = ", tag_st80y); */
				if (info->mess.dummy == 1)
					j = 1;
				else
					j = 0;
				for (; j < byteToRead + info->mess.dummy; j++) {
					/* logError_st80y(0, "%02X ", readData[j]); */
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", readData[j]);
					/* this approach is much more faster */
					index += 2;
				}
				/* logError_st80y(0, "\n"); */
				break;
			case CMD_GETFWFILE:
			case CMD_GETLIMITSFILE:
				logError_st80y(0, "%s Start To parse!\n", tag_st80y);
				for (j = 0; j < fileSize; j++) {
					/* logError_st80y(0, "%02X ", readData[j]); */
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", readData[j]);
					index += 2;
				}
				logError_st80y(0, "%s Finish to parse!\n", tag_st80y);
				break;

			case CMD_GETFORCELEN:
			case CMD_GETSENSELEN:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)temp);
				index += 2;

				break;


			case CMD_GETMSFRAME:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameMS.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameMS.header.sense_node);
				index += 2;

				for (j = 0; j < frameMS.node_data_size; j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameMS.node_data[j] &
						  0xFF00) >> 8,
						 frameMS.node_data[j] &
						 0xFF);
					index += 4;
				}

				kfree(frameMS.node_data);
				break;

			case CMD_GETSSFRAME:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameSS.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameSS.header.sense_node);
				index += 2;
				/* Copying self raw data Force */
				for (j = 0; j < frameSS.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameSS.force_data[j] &
						  0xFF00) >> 8,
						 frameSS.force_data[j] &
						 0xFF);
					index += 4;
				}


				/* Copying self raw data Sense */
				for (j = 0; j < frameSS.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameSS.sense_data[j] &
						  0xFF00) >> 8,
						 frameSS.sense_data[j] &
						 0xFF);
					index += 4;
				}

				kfree(frameSS.force_data);
				kfree(frameSS.sense_data);
				break;

			case CMD_GETSYNCFRAME:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameMS.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameMS.header.sense_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameSS.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)frameSS.header.sense_node);
				index += 2;

				/* Copying mutual data */
				for (j = 0; j < frameMS.node_data_size; j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameMS.node_data[j] &
						  0xFF00) >> 8,
						 frameMS.node_data[j] &
						 0xFF);
					index += 4;
				}

				/* Copying self data Force */
				for (j = 0; j < frameSS.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameSS.force_data[j] &
						  0xFF00) >> 8,
						 frameSS.force_data[j] &
						 0xFF);
					index += 4;
				}


				/* Copying self  data Sense */
				for (j = 0; j < frameSS.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (frameSS.sense_data[j] &
						  0xFF00) >> 8,
						 frameSS.sense_data[j] &
						 0xFF);
					index += 4;
				}

				kfree(frameMS.node_data);
				kfree(frameSS.force_data);
				kfree(frameSS.sense_data);
				break;

			case CMD_READMSCOMPDATA:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)compData.header.type);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)compData.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)compData.header.sense_node);
				index += 2;

				/* Cpying CX1 value */
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 compData.cx1 & 0xFF);
				index += 2;

				/* Copying CX2 values */
				for (j = 0; j < compData.node_data_size; j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", compData.node_data[j] &
						 0xFF);
					index += 2;
				}

				kfree(compData.node_data);
				break;

			case CMD_READSSCOMPDATA:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)comData.header.type);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.header.force_node);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.header.sense_node);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.f_ix1 & 0xFF);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.s_ix1 & 0xFF);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.f_cx1 & 0xFF);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.s_cx1 & 0xFF);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.f_ix0 & 0xFF);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 comData.s_ix0 & 0xFF);
				index += 2;


				/* Copying IX2 Force */
				for (j = 0; j < comData.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", comData.ix2_fm[j] &
						 0xFF);
					index += 2;
				}

				/* Copying IX2 Sense */
				for (j = 0; j < comData.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", comData.ix2_sn[j] &
						 0xFF);
					index += 2;
				}

				/* Copying CX2 Force */
				for (j = 0; j < comData.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", comData.cx2_fm[j] &
						 0xFF);

					index += 2;
				}

				/* Copying CX2 Sense */
				for (j = 0; j < comData.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X", comData.cx2_sn[j] &
						 0xFF);
					index += 2;
				}

				kfree(comData.ix2_fm);
				kfree(comData.ix2_sn);
				kfree(comData.cx2_fm);
				kfree(comData.cx2_sn);
				break;



			case CMD_READTOTMSCOMPDATA:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)totCompData.header.type);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)totCompData.header.force_node);
				index += 2;


				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)totCompData.header.sense_node);

				index += 2;

				/* Copying TOT CX values */
				for (j = 0; j < totCompData.node_data_size;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (totCompData.node_data[j] &
						  0xFF00) >> 8,
						 totCompData.node_data[j] &
						 0xFF);
					index += 4;
				}

				kfree(totCompData.node_data);
				break;

			case CMD_READTOTSSCOMPDATA:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 (u8)totComData.header.type);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 totComData.header.force_node);
				index += 2;

				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 totComData.header.sense_node);
				index += 2;

				/* Copying TOT IX Force */
				for (j = 0; j < totComData.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (totComData.ix_fm[j] &
						  0xFF00) >> 8,
						 totComData.ix_fm[j] &
						 0xFF);
					index += 4;
				}

				/* Copying TOT IX Sense */
				for (j = 0; j < totComData.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (totComData.ix_sn[j] &
						  0xFF00) >> 8,
						 totComData.ix_sn[j] &
						 0xFF);
					index += 4;
				}

				/* Copying TOT CX Force */
				for (j = 0; j < totComData.header.force_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (totComData.cx_fm[j] &
						  0xFF00) >> 8,
						 totComData.cx_fm[j] &
						 0xFF);


					index += 4;
				}

				/* Copying CX2 Sense */
				for (j = 0; j < totComData.header.sense_node;
				     j++) {
					snprintf(&info->driver_test_buff[index], 5,
						 "%02X%02X",
						 (totComData.cx_sn[j] &
						  0xFF00) >> 8,
						 totComData.cx_sn[j] &
						 0xFF);
					index += 4;
				}

				kfree(totComData.ix_fm);
				kfree(totComData.ix_sn);
				kfree(totComData.cx_fm);
				kfree(totComData.cx_sn);
				break;

			case CMD_GETFWVER:
				for (j = 0; j < EXTERNAL_RELEASE_INFO_SIZE;
				     j++) {
					snprintf(&info->driver_test_buff[index], 3,
						 "%02X",
						 info->systemInfo.u8_releaseInfo[j]);
					index += 2;
				}
				break;

			case CMD_READCOMPDATAHEAD:
				snprintf(&info->driver_test_buff[index], 3, "%02X",
					 dataHead.type);
				index += 2;
				break;


			default:
				break;
			}
		}

		snprintf(&info->driver_test_buff[index], 4, " }\n");
		info->limit = size - 1;/* avoid to print \0 in the shell */
		info->printed = 0;
	} else {
		/* start byte */
		info->driver_test_buff[index++] = MESSAGE_START_BYTE;
		if (info->bin_output == 1) {
			/* msg_size */
			info->driver_test_buff[index++] = (size & 0xFF00) >> 8;
			info->driver_test_buff[index++] = (size & 0x00FF);
			/* counter id */
			info->driver_test_buff[index++] = (info->mess.counter & 0xFF00) >>
						    8;
			info->driver_test_buff[index++] = (info->mess.counter & 0x00FF);
			/* action */
			info->driver_test_buff[index++] = (info->mess.action & 0xFF00) >> 8;
			info->driver_test_buff[index++] = (info->mess.action & 0x00FF);
			/* error */
			info->driver_test_buff[index++] = (res & 0xFF00) >> 8;
			info->driver_test_buff[index++] = (res & 0x00FF);
		} else {
			if (funcToTest[0] == CMD_GETLIMITSFILE_BYTE ||
			    funcToTest[0] == CMD_GETFWFILE_BYTE)
				snprintf(&info->driver_test_buff[index], 5,
					 "%02X%02X", (((fileSize + 3) / 4) &
						      0xFF00) >>
					 8, ((fileSize + 3) / 4) & 0x00FF);
			else
				snprintf(&info->driver_test_buff[index], 5,
					 "%02X%02X", (size & 0xFF00) >> 8,
					 size & 0xFF);
			index += 4;
			index += snprintf(&info->driver_test_buff[index], 5, "%04X",
					  (u16)info->mess.counter);
			index += snprintf(&info->driver_test_buff[index], 5, "%04X",
					  (u16)info->mess.action);
			index += snprintf(&info->driver_test_buff[index], 5,
					  "%02X%02X", (res & 0xFF00) >> 8, res &
					  0xFF);
		}

		switch (funcToTest[0]) {
		case CMD_VERSION_BYTE:
		case CMD_READ_BYTE:
		case CMD_WRITEREAD_BYTE:
		case CMD_WRITETHENWRITEREAD_BYTE:
		case CMD_WRITEREADU8UX_BYTE:
		case CMD_WRITEU8UXTHENWRITEREADU8UX_BYTE:
			if (info->bin_output == 1) {
				if (info->mess.dummy == 1)
					memcpy(&info->driver_test_buff[index],
					       &readData[1], byteToRead);
				else
					memcpy(&info->driver_test_buff[index],
					       readData, byteToRead);
				index += byteToRead;
			} else {
				j = info->mess.dummy;
				for (; j < byteToRead + info->mess.dummy; j++)
					index += snprintf(
						&info->driver_test_buff[index], 3,
						"%02X",
						(u8)readData[j]);
			}
			break;

		case CMD_GETLIMITSFILE_BYTE:
		case CMD_GETFWFILE_BYTE:
			if (info->bin_output == 1) {
				/* override the msg_size with dimension in words
				 * */
				info->driver_test_buff[1] = (((fileSize + 3) / 4) &
						       0xFF00) >> 8;
				info->driver_test_buff[2] = (((fileSize + 3) / 4) &
						       0x00FF);

				if (readData != NULL)
					memcpy(&info->driver_test_buff[index],
					       readData, fileSize);
				else
					logError_st80y(0,
						 "%s readData = NULL... returning junk data!",
						 tag_st80y);
				index += addr;	/* in this case the byte to read
						 * are stored in addr because it
						 * is a u64 end byte need to be
						 * inserted at the end of the
						 * padded memory */
			} else {
				/* snprintf(&info->driver_test_buff[1], 3, "%02X",
				 * (((fileSize + 3) / 4)&0xFF00) >> 8); */
				/* snprintf(&info->driver_test_buff[3], 3, "%02X",
				 * ((fileSize + 3) / 4)&0x00FF); */
				for (j = 0; j < fileSize; j++)
					index += snprintf(
						&info->driver_test_buff[index], 3,
						"%02X",
						(u8)readData[j]);
				for (; j < addr; j++)
					index += snprintf(
						&info->driver_test_buff[index], 3,
						"%02X", 0);	/* pad memory
								 * with 0x00 */
			}
			break;
		default:
			break;
		}

		info->driver_test_buff[index++] = MESSAGE_END_BYTE;
		info->driver_test_buff[index] = '\n';
		/*for(j=0; j<size; j++){
		  *      logError_st80y(0,"%c", info->driver_test_buff[j]);
		  * }*/
		info->limit = size;
		info->printed = 0;
	}
ERROR:
	numberParam = 0;/* need to reset the number of parameters in order to
			 * wait the next command, comment if you want to repeat
			 * the last comand sent just doing a cat */
	if (readData != NULL)
		kfree(readData);

	if(pbuf)
		kfree(pbuf);

	if(cmd)
		kfree(cmd);

	if(funcToTest)
		kfree(funcToTest);

	return count;
}

/** @}*/

/**
  * file_operations struct which define the functions for the canonical
  *operation on a device file node (open. read, write etc.)
  */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
static const struct file_operations fts_driver_test_ops = {
	.open		= fts_open,
	.read		= seq_read,
	.write		= fts_driver_test_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};
#else
static const struct proc_ops fts_driver_test_ops = {
	.proc_open		= fts_open,
	.proc_read		= seq_read,
	.proc_write		= fts_driver_test_write,
	.proc_lseek		= seq_lseek,
	.proc_release	= seq_release
};
#endif

/*****************************************************************************/

/**
  * This function is called in the probe to initialize and create the directory
  * proc/fts and the driver test file node DRIVER_TEST_FILE_NODE into the /proc
  * file system
  * @return OK if success or an error code which specify the type of error
  */
int st80y_proc_init(struct fts_ts_info *info)
{
	struct proc_dir_entry *entry;

	int retval = 0;
	fts_tools_info = info;

	info->fts_dir = proc_mkdir_data("st80y", 0777, NULL, NULL);
	if (info->fts_dir == NULL) {	/* directory creation failed */
		retval = -ENOMEM;
		goto out;
	}

	entry = proc_create_data(DRIVER_TEST_FILE_NODE, 0777, info->fts_dir,
			    &fts_driver_test_ops, info);
	if (entry)
		logError_st80y(1, "%s %s: proc entry CREATED!\n", tag_st80y, __func__);
	else {
		logError_st80y(1, "%s %s: error creating proc entry!\n", tag_st80y,
			 __func__);
		retval = -ENOMEM;
		goto badfile;
	}
	return OK;
badfile:
	remove_proc_entry("fts", NULL);
out:
	return retval;
}
static int st80y_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = fts_tools_info;
	VTI("open st80y_dev");
	return 0;
}
typedef struct writeCmd{
    u8 *cmd;
    int cmdLength;
} T_writeCmd;
typedef struct writeReadCmd{
    u8 *cmd;
    int cmdLength;
    u8 *outBuf;
    int byteToRead;
} T_writeReadCmd;

#define ST_MDEV_IOC_MAGIC   		                    'S'
#define IOCTL_CMD_WRITE_READ                          _IOWR(ST_MDEV_IOC_MAGIC, 1, T_writeReadCmd)
#define IOCTL_CMD_WRITE_FW                            _IOW(ST_MDEV_IOC_MAGIC, 2, T_writeCmd)
#define IOCTL_CMD_IRQ		                         _IOW(ST_MDEV_IOC_MAGIC, 3, u8)
#define IOCTL_CMD_HW_RESET                           _IOW(ST_MDEV_IOC_MAGIC, 4, u8)
#define IOCTL_CMD_SYSTEM_RESET                       _IOW(ST_MDEV_IOC_MAGIC, 5, u8)
#define IOCTRL_CMD_GET_FRAME                         _IOR(ST_MDEV_IOC_MAGIC, 6, u8)
#define IOCTRL_CMD_READ_CONFIG                       _IOR(ST_MDEV_IOC_MAGIC, 7, u8)
#define IOCTL_CMD_WRITE                              _IOW(ST_MDEV_IOC_MAGIC, 8, T_writeCmd)
#define IOCTRL_CMD_GET_SYSINFO                       _IOR(ST_MDEV_IOC_MAGIC, 9, SysInfo)
#define IOCTRL_CMD_MODE_HANDLER                      _IOW(ST_MDEV_IOC_MAGIC, 10, u8)




#define I2C_MSG_HEAD_LEN  4 
static long st80y_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct fts_ts_info *info = fts_tools_info;
	T_writeReadCmd wr_cmd;
	T_writeCmd w_cmd;
	u8 *cmd_data = NULL;
	u8 *out_buf = NULL;
	u8 data[2];
	VTI("st80y_dev_ioctl enter cmd= %d", cmd);
	switch(cmd){
		case IOCTL_CMD_SYSTEM_RESET:
			VTI("system reset!");
			ret = st80y_system_reset(info);
			break;
		case IOCTL_CMD_HW_RESET:
			VTI("HW reset!");
			
			break;
			#if 0
  		case IOCTL_CMD_READ:
			ret = copy_from_user(&msg_head, (void *)arg, I2C_MSG_HEAD_LEN);//get msg head info 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			addr = msg_head[0] + (msg_head[1] << 8)+ (msg_head[2] << 16) + (msg_head[3] << 24);
			lenth = msg_head[4] + (msg_head[5] << 8)+ (msg_head[6] << 16) + (msg_head[7] << 24);
			databuf = kzalloc(lenth, GFP_KERNEL);
			if (!databuf) {
					VTE("Alloc memory failed");
					return -ENOMEM;
			}
			ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, lenth);
			if (ret) {
				ret = -EFAULT;
				VTE("Copy data from user failed");
				goto err_out;
			}
			VTI("start to read addr:0x%04x, len:%d ",addr, lenth);
	  		break;
		case IOCTL_CMD_WRITE:
			ret = copy_from_user(&msg_head, (void *)arg, I2C_MSG_HEAD_LEN);//get msg head info 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			addr = msg_head[0] + (msg_head[1] << 8)+ (msg_head[2] << 16) + (msg_head[3] << 24);
			lenth = msg_head[4] + (msg_head[5] << 8)+ (msg_head[6] << 16) + (msg_head[7] << 24);
			databuf = kzalloc(lenth, GFP_KERNEL);
			if (!databuf) {
					VTE("Alloc memory failed");
					return -ENOMEM;
			}
			ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, lenth);
			if (ret) {
				ret = -EFAULT;
				VTE("Copy data from user failed");
				goto err_out;
			}
	  		VTI("start to write data to addr:0x%04x, len:%d ",addr, lenth);
			break;
			#endif
		case IOCTRL_CMD_READ_CONFIG:
			VTI("read config");
			ret = readConfig(info, ADDR_CONFIG_SENSE_LEN, data, 2);
			if (ret < OK) 
				logError_st80y(1, "%s st80y_getChannelsLength: ERROR %08X\n", tag_st80y, ret);
			if (copy_to_user((u8 *)arg, data, 2)) {
	  			ret = -EFAULT;
	  			VTE("Copy_to_user failed");
  			}
			break;
		case IOCTL_CMD_WRITE_FW:
			VTI("IOCTL_CMD_WRITE_FW");
			ret = copy_from_user(&w_cmd, (void *)arg, sizeof(T_writeCmd));//get struct 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("CMD SIZE:%d", w_cmd.cmdLength);
			cmd_data = kzalloc(w_cmd.cmdLength, GFP_KERNEL);
			if(cmd_data == NULL)
				return 0;
			ret = copy_from_user(cmd_data, w_cmd.cmd, w_cmd.cmdLength);//get cmd data 
			if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			st80y_writeFwCmd(info,cmd_data, w_cmd.cmdLength);
			break;
		case IOCTL_CMD_WRITE:
			VTI("IOCTL_CMD_WRITE");
			ret = copy_from_user(&w_cmd, (void *)arg, sizeof(T_writeCmd));//get struct 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("CMD SIZE:%d", w_cmd.cmdLength);
			cmd_data = kzalloc(w_cmd.cmdLength, GFP_KERNEL);
			if(cmd_data == NULL)
				return 0;
			ret = copy_from_user(cmd_data, w_cmd.cmd, w_cmd.cmdLength);//get cmd data 
			if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			st80y_write(info,cmd_data, w_cmd.cmdLength);
			break;
		case IOCTL_CMD_WRITE_READ:
			VTI("IOCTL_CMD_WRITE_READ");
			ret = copy_from_user(&wr_cmd, (void *)arg, sizeof(T_writeReadCmd));//get struct 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("CMD SIZE:%d, read size %d:", wr_cmd.cmdLength, wr_cmd.byteToRead);
			cmd_data = kzalloc(wr_cmd.cmdLength * sizeof(u8), GFP_KERNEL);
			out_buf = kzalloc(wr_cmd.byteToRead * sizeof(u8), GFP_KERNEL);
			if(cmd_data == NULL || out_buf == NULL)
				return 0;
			ret = copy_from_user(cmd_data, wr_cmd.cmd, wr_cmd.cmdLength);//get cmd data 
			if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("cmd data:0x%x--0x%x", cmd_data[0],cmd_data[1]);
			ret = st80y_writeRead(info,cmd_data, wr_cmd.cmdLength, out_buf, wr_cmd.byteToRead);
			if(ret != 0){
				VTE("st80y_writeRead fail ");
				return -EFAULT;
			}	
			VTI("rbuf data:0x%x--0x%x", out_buf[0],out_buf[1]);
			if(copy_to_user((u8 *)wr_cmd.outBuf, out_buf, wr_cmd.byteToRead)){
				ret = -EFAULT;
	  			VTE("Copy_to_user out_buf failed");
			}
			if(copy_to_user((void *)arg, &wr_cmd, sizeof(T_writeReadCmd))){
				ret = -EFAULT;
	  			VTE("Copy_to_user T_writeReadCmd failed");
			}
			break;
		case IOCTRL_CMD_GET_SYSINFO:
			VTI("IOCTRL_CMD_GET_SYSINFO ");
			if(copy_to_user((u8 *)arg, &info->systemInfo, sizeof(SysInfo))){
				ret = -EFAULT;
	  			VTE("Copy_to_user sysinfo failed");
			}
			break;
		case IOCTL_CMD_IRQ:
			VTI("IOCTL_CMD_IRQ");
			ret = copy_from_user(&data[0], (void *)arg, sizeof(u8));//get struct 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("cmd data:%d",data[0]);
			if(data[0])
				st80y_enableInterrupt(info);
			else
				st80y_disableInterrupt(info);
			break;
		case IOCTRL_CMD_MODE_HANDLER:
			VTI("IOCTRL_CMD_MODE_HANDLER");
			ret = copy_from_user(&data[0], (void *)arg, sizeof(u8));//get struct 
		  	if (ret) {
		  		VTE("Copy data from user failed");
		  		return -EFAULT;
		  	}
			VTI("cmd data:%d",data[0]);
			fts_mode_handler_extern(info, data[0]);
		default:
			break;
		
	}

	if(cmd_data)
		kfree(cmd_data);
	if(out_buf)
		kfree(out_buf);

	return ret;
	

}

static ssize_t st80y_dev_write(struct file *filep, const char __user *buf, size_t len, loff_t *pos)
{
	char temp[65];
	if(len > 64)
		len = 64;
	if(copy_from_user(temp, buf,len))
	{
		return -EFAULT;
	}
	VTI("write %s\n",temp);
	
	return len;
}

static int st80y_dev_release(struct inode *inode, struct file *filp)
{
	VTI("release st80y_dev");
	return 0;
}

struct file_operations st80y_tool_fops = {
	.owner 			= THIS_MODULE,
	.open 			= st80y_dev_open,
	.release 		= st80y_dev_release,
	.unlocked_ioctl = st80y_dev_ioctl,
	.write 			= st80y_dev_write,

};
#define ST_TOOLS_NAME		"st_tools"

struct miscdevice fts_tools_misc_device = {
	.name = ST_TOOLS_NAME,
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &st80y_tool_fops,
	
};

int st80y_tools_dev_init(struct vts_device *vtsdev)
{
	int32_t ret = 0;
	//char *dev_name = vtsdev->type == VTS_TYPE_MAIN ? "st80y_tools" : "st80y_tools_second";


	ret = misc_register(&fts_tools_misc_device);

	if (ret < 0) {
		VTE("%s : register failed %d\n", __func__, ret);
		return ret;
	}

	VTI("%s : register sucess\n", __func__);
	return ret;
}

/**
  * Delete and Clean from the file system, all the references to the driver test
  * file node
  * @return OK
  */
int st80y_proc_remove(struct fts_ts_info *info)
{
	remove_proc_entry(DRIVER_TEST_FILE_NODE, info->fts_dir);
	remove_proc_entry(dev_name(info->dev), NULL);
	return OK;
}
