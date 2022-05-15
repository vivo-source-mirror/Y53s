/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/************************************************************************
*
* File Name: focaltech_spi_ft3518u.c
*
*    Author: FocalTech Driver Team
*
*   Created: 2021-01-16
*
*  Abstract: spi communication with TP
*
*   Version: v1.0
*
* Revision History:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define SPI_RETRY_NUMBER            3
#define CS_HIGH_DELAY               150 /* unit: us */
#define SPI_PREX_LEN                8
#define SPI_SUFFIX_LEN              4
#define SPI_HEADER_LENGTH           (SPI_PREX_LEN + SPI_SUFFIX_LEN + 2)
#define SPI_BUF_LENGTH              256


/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
/* spi interface */
static int fts_spi_transfer(u8 *tx_buf, u8 *rx_buf, u32 len)
{
	int ret = 0;
	struct spi_device *spi = ft3518u_fts_data->spi;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len    = len,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);
	if (ret) {
		FTS_ERROR("spi_sync fail,ret:%d", ret);
		return ret;
	}

	return ret;
}

static int fts_rdata_process(u8 *rxbuf, u32 rxlen, u8 *data, u32 datalen)
{
	int i = 0;

	if (rxbuf[0] != 0xA0) {
		FTS_ERROR("read rxbuf[0]:0x%02X!=0xA0", rxbuf[0]);
		VTD_BUF("rxbuf",rxbuf, rxlen);
		return -EINVAL;
	}

	for (i = 0; i < rxlen; i++) {
		if ((rxbuf[i] == 0xA5) && (rxbuf[i + 1] == 0x5A)) {
			memcpy(&data[0], &rxbuf[i + 3], datalen);
			if (rxbuf[i + 2] != datalen) {
				FTS_ERROR("read rxbuf len[%d]!=datalen[%d]", rxbuf[i + 2], datalen);
				VTD_BUF("rxbuf", rxbuf, rxlen);
			}
			break;
		}
	}

	if (i >= rxlen) {
		FTS_ERROR("read rxbuf parse fail");
		VTD_BUF("rxbuf", rxbuf, rxlen);
		return -ENODATA;
	}

	return 0;
}

int fts_write_command(u8 *writebuf, u32 writelen, u32 readlen)
{
	int ret = 0;
	int i = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;
	u32 txlen = 0;
	u8 dummy[16] = { 0 };

	if ((writelen + SPI_HEADER_LENGTH >= SPI_BUF_LENGTH)) {
		FTS_ERROR("writelen(%d) is invalid", writelen);
		return -EINVAL;
	}

	txbuf = ts_data->bus_tx_buf;
	rxbuf = ts_data->bus_rx_buf;
	memset(txbuf, 0x0, SPI_BUF_LENGTH);
	memset(rxbuf, 0x0, SPI_BUF_LENGTH);

	for (i = 0; i < SPI_PREX_LEN - 1; i++)
		txbuf[txlen++] = 0xA5;

	txbuf[txlen++] = 0x5A;
	txbuf[txlen++] = writelen;
	txbuf[txlen++] = readlen;
	if (writebuf && writelen) {
		memcpy(&txbuf[txlen], &writebuf[0], writelen);
		txlen = txlen + writelen;
	}
	for (i = 0; i < SPI_SUFFIX_LEN; i++)
		txbuf[txlen++] = 0xDD;

	ret = fts_spi_transfer(dummy, rxbuf, 4);
	udelay(20);
	
	ret = fts_spi_transfer(txbuf, rxbuf, txlen);
	if (ret < 0) {
		FTS_ERROR("spi write fail");
	}
	udelay(CS_HIGH_DELAY);
	return ret;
}

int ft3518u_fts_write(u8 *writebuf, u32 writelen)
{
	struct fts_ts_data *ts_data = ft3518u_fts_data;
	int ret = 0;

	if (!writebuf || !writelen || (writelen + SPI_HEADER_LENGTH >= SPI_BUF_LENGTH)) {
		FTS_ERROR("writebuf/writelen(%d) is invalid", writelen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	ret = fts_write_command(writebuf, writelen, 0);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int ft3518u_fts_write_reg(u8 addr, u8 value)
{
	u8 writebuf[2] = { 0 };

	writebuf[0] = addr;
	writebuf[1] = value;
	return ft3518u_fts_write(writebuf, 2);
}

int ft3518u_fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	int ret = 0;
	int i = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;
	u32 txlen = 0;

	if (!data || !datalen || (cmdlen + SPI_HEADER_LENGTH >= SPI_BUF_LENGTH)
		|| (datalen + SPI_HEADER_LENGTH >= SPI_BUF_LENGTH)) {
		FTS_ERROR("cmd/cmdlen(%d)/data/datalen(%d) invalid", cmdlen, datalen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	for (i = 0; i < SPI_RETRY_NUMBER; i++) {
		ret = fts_write_command(cmd, cmdlen, datalen);
		if (ret < 0) {
			FTS_ERROR("spi read(cmd) send fail,retries:%d", i);
			udelay(CS_HIGH_DELAY);
			continue;
		}
		/*need more delay*/
		if (!(cmd && cmdlen && (cmd[0] < 0x04)))
			udelay(datalen * 4);

		txbuf = ts_data->bus_tx_buf;
		rxbuf = ts_data->bus_rx_buf;
		memset(txbuf, 0x0, SPI_BUF_LENGTH);
		memset(rxbuf, 0x0, SPI_BUF_LENGTH);
		txlen = SPI_HEADER_LENGTH + datalen;
		ret = fts_spi_transfer(txbuf, rxbuf, txlen);
		fts_rdata_process(rxbuf, txlen, data, datalen);
		if (ret < 0) {
			FTS_ERROR("spi read(data) fail,retries:%d", i);
			udelay(CS_HIGH_DELAY);
			continue;
		}
		break;
	}

	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

inline int ft3518u_fts_read_reg(u8 addr, u8 *value)
{
	return ft3518u_fts_read(&addr, 1, value, 1);
}


int ft3518u_write_then_read(u8 addr, u8 value)
{
	u8 temp_data = 0;
	int i = 0;
	for (i = 0; i < 3; ++i){
		ft3518u_fts_write_reg(addr, value);
		ft3518u_fts_read_reg(addr, &temp_data);
		if (temp_data == value) {
			VTI("Success to write 0x%x value: %d", addr, value);
			return 0;
		}
	}

	VTI("Fail to write 0x%x value: %d", addr, value);	
	return -1;
}


int ft3518u_fts_spi_transfer_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret = 0;
	struct fts_ts_data *ts_data = ft3518u_fts_data;
	u8 *txbuf = NULL;
	u8 *rxbuf = NULL;

	if ((writelen >= SPI_BUF_LENGTH) || (readlen >= SPI_BUF_LENGTH)) {
		FTS_ERROR("writelen(%d)/readlen(%d) is invalid", writelen, readlen);
		return -EINVAL;
	}

	mutex_lock(&ts_data->bus_lock);
	txbuf = ts_data->bus_tx_buf;
	rxbuf = ts_data->bus_rx_buf;
	memset(txbuf, 0x0, SPI_BUF_LENGTH);
	memset(rxbuf, 0x0, SPI_BUF_LENGTH);
	if (writebuf && writelen) {
		memcpy(txbuf, writebuf, writelen);
		ret = fts_spi_transfer(txbuf, rxbuf, writelen);
		if (ret < 0) {
			FTS_ERROR("spi transfer(write) fails, ret:%d", ret);
			goto err_spi_dir;
		}
	}

	if (readbuf && readlen) {
		ret = fts_spi_transfer(txbuf, rxbuf, readlen);
		memcpy(readbuf, rxbuf, readlen);
		if (ret < 0) {
			FTS_ERROR("spi transfer(read) fails, ret:%d", ret);
			goto err_spi_dir;
		}
	}

err_spi_dir:
	udelay(CS_HIGH_DELAY);
	mutex_unlock(&ts_data->bus_lock);
	return ret;
}

int ft3518u_fts_bus_init(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	ts_data->bus_tx_buf = kzalloc(SPI_BUF_LENGTH, GFP_KERNEL | GFP_DMA);
	if (NULL == ts_data->bus_tx_buf) {
		FTS_ERROR("failed to allocate memory for bus_tx_buf");
		return -ENOMEM;
	}

	ts_data->bus_rx_buf = kzalloc(SPI_BUF_LENGTH, GFP_KERNEL | GFP_DMA);
	if (NULL == ts_data->bus_rx_buf) {
		FTS_ERROR("failed to allocate memory for bus_rx_buf");
		kfree_safe(ts_data->bus_rx_buf);
		return -ENOMEM;
	}

	FTS_FUNC_EXIT();
	return 0;
}

int ft3518u_fts_bus_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	if (ts_data && ts_data->bus_tx_buf)
		kfree_safe(ts_data->bus_tx_buf);

	if (ts_data && ts_data->bus_rx_buf)
		kfree_safe(ts_data->bus_rx_buf);

	FTS_FUNC_EXIT();
	return 0;
}

