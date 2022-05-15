/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

struct touch_bus_info {
	struct spi_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_tddi_dev *idev_V2;
int (*core_spi_mode_read)(u8 *data, int len);
int (*core_spi_mode_write)(u8 *data, int len);

#define DMA_TRANSFER_MAX_CHUNK		64   /* number of chunks to be transferred. */
#define DMA_TRANSFER_MAX_LEN		1024 /* length of a chunk. */
struct spi_transfer	xfer_V2[DMA_TRANSFER_MAX_CHUNK + 1];

int ilitek_spi_write_then_read_split_V2(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int status = -1;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	u8 *dma_txbuf = NULL, *dma_rxbuf = NULL;
	u8 cmd, temp1[1] = {0}, temp2[1] = {0};
	struct spi_message	message;
	int index = 0;

	if (idev_V2->spi_mode == SPI_SLAVE_MODE && atomic_read(&idev_V2->ice_stat) == DISABLE)
		index = 2;

	dma_txbuf = kzalloc(n_tx, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(dma_txbuf)) {
		ipio_err("Failed to allocate dma_txbuf, %ld\n", PTR_ERR(dma_txbuf));
		goto out;
	}

	dma_rxbuf = kzalloc(n_rx + index, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(dma_rxbuf)) {
		ipio_err("Failed to allocate dma_rxbuf, %ld\n", PTR_ERR(dma_rxbuf));
		goto out;
	}

	mutex_trylock(&lock);

	spi_message_init(&message);
	memset(xfer_V2, 0, sizeof(xfer_V2));

	if ((n_tx == 1) && (n_rx == 1))
		cmd = SPI_READ;
	else
		cmd = *((u8 *)txbuf);

	switch (cmd) {
	case SPI_WRITE:
		if (n_tx % DMA_TRANSFER_MAX_LEN)
			xferloop = (n_tx / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = n_tx / DMA_TRANSFER_MAX_LEN;

		xferlen = n_tx;
		memcpy(dma_txbuf, (u8 *)txbuf, xferlen);

		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer_V2[xfercnt].len = xferlen;
			xfer_V2[xfercnt].tx_buf = dma_txbuf + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer_V2[xfercnt], &message);
			xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		/* for write cmd and head */
		xfer_V2[0].len = n_tx;
		xfer_V2[0].tx_buf = txbuf;
		xfer_V2[0].rx_buf = temp1;
		spi_message_add_tail(&xfer_V2[0], &message);

		/* for read data */
		if (n_rx % DMA_TRANSFER_MAX_LEN)
			xferloop = (n_rx / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = n_rx / DMA_TRANSFER_MAX_LEN;

		xferlen = n_rx;
		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer_V2[xfercnt+1].len = xferlen;
			xfer_V2[xfercnt+1].tx_buf = temp2;
			xfer_V2[xfercnt+1].rx_buf = dma_rxbuf + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer_V2[xfercnt+1], &message);
			xferlen = n_rx - (xfercnt+1) * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		if (status == 0)
			memcpy((u8 *)rxbuf, dma_rxbuf, n_rx);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	mutex_unlock(&lock);

out:
	ipio_kfree((void **)&dma_txbuf);
	ipio_kfree((void **)&dma_rxbuf);
	return status;
}

int ilitek_spi_write_then_read_direct_V2(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int status = -1;
	u8 cmd;//, temp1[1] = {0}, temp2[1] = {0};
	struct spi_message	message;
	struct spi_transfer	xfer_V2[2];
	int retry = 0;
	int index = 0;
	u8 *rxbuf_temp = NULL;
	if (idev_V2->spi_mode == SPI_SLAVE_MODE && atomic_read(&idev_V2->ice_stat) == DISABLE)
		index = 2;
	
	mutex_trylock(&lock);

	while(idev_V2->ili_suspended) {
		msleep(10);
		if (++retry > 50) {
			VTI("after 500ms delay, device is stil in suspend mode!\n");
			mutex_unlock(&lock);
			return 0;
		}
	}

	spi_message_init(&message);
	memset(xfer_V2, 0, sizeof(xfer_V2));

	if ((n_tx == 1) && (n_rx == 1))
		cmd = SPI_READ;
	else
		cmd = *((u8 *)txbuf);

	switch (cmd) {
	case SPI_WRITE:
		xfer_V2[0].len = n_tx;
		xfer_V2[0].tx_buf = txbuf;
		spi_message_add_tail(&xfer_V2[0], &message);
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		/* for write cmd and head */
		xfer_V2[0].len = n_tx;
		xfer_V2[0].tx_buf = txbuf;
		//xfer_V2[0].rx_buf = temp1;
		spi_message_add_tail(&xfer_V2[0], &message);

		xfer_V2[1].len = (n_rx + index);
		//xfer_V2[1].tx_buf = temp2;
		rxbuf_temp = idev_V2->rxbuf;
		xfer_V2[1].rx_buf = rxbuf_temp;
		spi_message_add_tail(&xfer_V2[1], &message);
		status = spi_sync(spi, &message);
		if (status == 0)
			memcpy((u8 *)rxbuf, (rxbuf_temp + index), n_rx);

		if (rxbuf_temp[0] != SPI_ACK && atomic_read(&idev_V2->ice_stat) == DISABLE)
			status = DO_SPI_RECOVER;
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	mutex_unlock(&lock);
	
	return status;
}

static int core_rx_lock_check(int *ret_size)
{
	int i, count = 1;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x94, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, lock = 0x5AA5;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x94,0x0,0x2) error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];
		*ret_size = (rxbuf[0] << 8) + rxbuf[1];

		ipio_debug("Rx lock = 0x%x, size = %d\n", status, *ret_size);

		if (status == lock)
			return 0;

		mdelay(1);
	}

out:
	ipio_err("Rx check lock error, lock = 0x%x, size = %d\n", status, *ret_size);
	return -EIO;
}

static int core_tx_unlock_check(void)
{
	int i, count = 100;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x0, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, unlock = 0x9881;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x0,0x0,0x2) error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];

		ipio_debug("Tx unlock = 0x%x\n", status);

		if (status == unlock)
			return 0;

		mdelay(1);
	}

out:
	ipio_err("Tx check unlock error, unlock = 0x%x\n", status);
	return -EIO;
}

static int core_spi_ice_mode_unlock_read(u8 *data, int size)
{
	int ret = 0;
	u8 txbuf[64] = { 0 };

	/* set read address */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x98;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_info("spi write (0x25,0x98,0x00,0x2) error\n");
		ret = -EIO;
		return ret;
	}

	/* read data */
	txbuf[0] = SPI_READ;
	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, data, size) < 0) {
		ret = -EIO;
		return ret;
	}

	/* write data unlock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x94;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x98;
	txbuf[8] = (char)0x81;
	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 9, txbuf, 0) < 0) {
		ipio_err("spi write unlock (0x9881) error, ret = %d\n", ret);
		ret = -EIO;
	}
	return ret;
}

static int core_spi_ice_mode_lock_write(u8 *data, int size)
{
	int ret = 0;
	int safe_size = size;
	u8 check_sum = 0, wsize = 0;
	u8 *txbuf = NULL;

	txbuf = kcalloc(size + 9, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Write data */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x4;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;

	/* Calcuate checsum and fill it in the last byte */
	check_sum = ilitek_calc_packet_checksum_V2(data, size);
	ipio_memcpy(txbuf + 5, data, size, safe_size + 9);
	txbuf[5 + size] = check_sum;
	size++;
	wsize = size;
	if (wsize % 4 != 0)
		wsize += 4 - (wsize % 4);

	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, wsize + 5, txbuf, 0) < 0) {
		ipio_info("spi write (0x25,0x4,0x00,0x2) error\n");
		ret = -EIO;
		goto out;
	}

	/* write data lock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x0;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x5A;
	txbuf[8] = (char)0xA5;
	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 9, txbuf, 0) < 0) {
		ipio_err("spi write lock (0x5AA5) error, ret = %d\n", ret);
		ret = -EIO;
	}

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}

static int core_spi_ice_mode_disable(void)
{
	u8 txbuf[5] = {0x82, 0x1B, 0x62, 0x10, 0x18};

	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_err("spi write ice mode disable failed\n");
		return -EIO;
	}
	return 0;
}

static int core_spi_ice_mode_enable(void)
{
	u8 cmd[9] = {0x82, 0x25, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	u8 txbuf[5] = {0x82, 0x1F, 0x62, 0x10, 0x18};
	u8 rxbuf[2] = {0};
	VTD("ice_mode 1");
	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, rxbuf, 1) < 0) {
		ipio_err("spi write 0x82 error\n");
		return -EIO;
	}
	VTD("ice_mode 2");

	/* check recover data */
	if (rxbuf[0] != SPI_ACK) {
		ipio_err("Check SPI_ACK failed (0x%x)\n", rxbuf[0]);
		return DO_SPI_RECOVER;
	}
	VTD("ice_mode 3");

	/* if system is suspended, wake up our spi pll clock before communication. */
	if (idev_V2->tp_suspend) {
		ipio_info("Write dummy cmd to wake up spi pll clock\n");
		if (idev_V2->spi_write_then_read(idev_V2->spi, cmd, sizeof(cmd), rxbuf, 0) < 0) {
			ipio_err("spi write wake up cmd failed\n");
			return -EIO;
		}
	}
	VTD("ice_mode 4");

	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 5, rxbuf, 0) < 0) {
		ipio_err("spi write ice mode enable failed\n");
		return -EIO;
	}
	VTD("ice_mode 5");
	return 0;
}

static int core_spi_ice_mode_write(u8 *data, int len)
{
	int ret = 0;
	ret = core_spi_ice_mode_enable();
	if (ret < 0)
		return ret;

	/* send data and change lock status to 0x5AA5. */
	ret = core_spi_ice_mode_lock_write(data, len);
	if (ret < 0)
		goto out;

	/*
	 * Check FW if they already received the data we sent.
	 * They change lock status from 0x5AA5 to 0x9881 if they did.
	 */
	ret = core_tx_unlock_check();
	if (ret < 0)
		goto out;

out:
	if (core_spi_ice_mode_disable() < 0)
		return -EIO;

	return ret;
}

static int core_spi_ice_mode_read(u8 *data, int len)
{
	int size = 0, ret = 0;
	
	ret = core_spi_ice_mode_enable();
	if (ret < 0)
		return ret;
	

	/*
	 * Check FW if they already send their data to rxbuf.
	 * They change lock status from 0x9881 to 0x5AA5 if they did.
	 */
	

	ret = core_rx_lock_check(&size);
	if (ret < 0)
		goto out;
	

	if (len < size && idev_V2->fw_uart_en == DISABLE) {
		ipio_info("WARRING! size(%d) > len(%d), use len to get data\n", size, len);
		size = len;
	}
	

	/* receive data from rxbuf and change lock status to 0x9881. */
	ret = core_spi_ice_mode_unlock_read(data, size);
	if (ret < 0)
		goto out;

out:
	
	if (core_spi_ice_mode_disable() < 0)
		ret = -EIO;

	return (ret >= 0) ? size : ret;
}

static int core_spi_write(u8 *data, int len)
{
	int ret = 0, count = 5;
	u8 *txbuf = NULL;
	int safe_size = len;
    u8 alloc_flag = 0;
	
	if (atomic_read(&idev_V2->ice_stat) == DISABLE) {
		if (idev_V2->spi_mode == SPI_ICE_MODE)
			core_spi_mode_write = core_spi_ice_mode_write;
		else
			core_spi_mode_write = core_spi_slave_mode_write;
		
		do {
			ret = core_spi_mode_write(data, len);
			if (ret >= 0)
				break;
		} while (--count > 0);
		goto out;
	}
	
    if(len > SPI_TX_BUF_LEN-1 || NULL == idev_V2->txbuf){
		txbuf = kcalloc(len + 1, sizeof(u8), GFP_KERNEL|GFP_DMA);
		if (ERR_ALLOC_MEM(txbuf)) {
			ipio_err("Failed to allocate txbuf\n");
			return -ENOMEM;
		}
		alloc_flag = 1;
    }else{

	  txbuf = idev_V2->txbuf;
    }
	txbuf[0] = SPI_WRITE;
	ipio_memcpy(txbuf+1, data, len, safe_size + 1);
	

	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, len+1, txbuf, 0) < 0) {
		ipio_err("spi write data error in ice mode\n");
		ret = -EIO;
		goto out;
	}
	

out:
	if(alloc_flag)
	ipio_kfree((void **)&txbuf);
	return ret;
}

static int core_spi_read(u8 *rxbuf, int len)
{
	int ret = 0, count = 5;
	u8 txbuf[1] = {0};

	txbuf[0] = SPI_READ;
	

	if (atomic_read(&idev_V2->ice_stat) == DISABLE) {
		if (idev_V2->spi_mode == SPI_ICE_MODE)
			core_spi_mode_read = core_spi_ice_mode_read;
		else
			core_spi_mode_read = core_spi_slave_mode_read;
		do {
			ret = core_spi_mode_read(rxbuf, len);
			if (ret >= 0)
				break;
		} while (--count > 0);
		goto out;
	}
	

	if (idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, rxbuf, len) < 0) {
		ipio_err("spi read data error in ice mode\n");
		ret = -EIO;
		goto out;
	}
	

out:
	
	return ret;
}

static int ilitek_spi_write(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi write len is invaild\n");
		return -EINVAL;
	}
	mutex_lock(&idev_V2->spi_mutex);
	ret = core_spi_write(buf, len);
	mutex_unlock(&idev_V2->spi_mutex);
	if (ret < 0) {
		if (atomic_read(&idev_V2->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi write error, ret = %d\n", ret);
	}

out:
	return ret;
}

/* If ilitek_spi_read success ,this function will return read length */
static int ilitek_spi_read(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi read len is invaild\n");
		return -EINVAL;
	}

	//mutex_lock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
    mutex_lock(&idev_V2->spi_mutex);
	ret = core_spi_read(buf, len);
	mutex_unlock(&idev_V2->spi_mutex);
	//mutex_unlock(&(vivoTsGetVtsData()->lcmResumeAndProximityMutex));
	if (ret < 0) {
		if (atomic_read(&idev_V2->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi read error, ret = %d\n", ret);
	}

out:
	return ret;
}
int core_spi_slave_mode_write(u8 *data, int size)
{
	int ret = 0, index = 0;
	int safe_size = size, wsize = 0;
	// u8 check_sum = 0
	u8 *txbuf = NULL;

	txbuf = kcalloc(size + 9, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Write data */
	txbuf[index++] = SPI_WRITE;
	// txbuf[index++] = 0;
	// txbuf[index++] = 0;
	// txbuf[index++] = 0x4;
	// txbuf[index++] = 0x0;
	// txbuf[index++] = 0x2;

	ipio_memcpy(txbuf + index, data, size, safe_size + 9);

	/* Calcuate checsum and fill it in the last byte */
	// check_sum = ilitek_calc_packet_checksum(data, size);
	// txbuf[index + size] = check_sum;
	// size++;
	wsize = size + index;
	if (wsize % 4 != 0)
		wsize += 4 - (wsize % 4);

	ret = idev_V2->spi_write_then_read(idev_V2->spi, txbuf, wsize, txbuf, 0);
	if (ret < 0)
		ipio_info("spi slave  write error\n");

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}

int core_spi_slave_mode_read(u8 *data, int len)
{
	int ret = 0;
	u8 txbuf[64] = { 0 };

	/* set read address */
	// txbuf[index++] = SPI_WRITE;
	// txbuf[index++] = 0x25;
	// txbuf[index++] = 0x98;
	// txbuf[index++] = 0x0;
	// txbuf[index++] = 0x2;
	// if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
	// 	ipio_info("spi write (0x25,0x98,0x00,0x2) error\n");
	// 	ret = -EIO;
	// 	return ret;
	// }

	/* read data */
	txbuf[0] = SPI_READ;
	ret = idev_V2->spi_write_then_read(idev_V2->spi, txbuf, 1, data, len);

	return ret;
}

int ilitek_thp_ice_mode_send_cmd(u8* cmd, u32 w_len, u8* data, u32 r_len)
{
	u8 pre_cmd = P5_X_READ_DATA_CTRL;
	int ret = 0;

	ipio_info("ice mode read cmd\n");

	ipio_info("cmd = 0x%x, len = %d\n", cmd[0], r_len);
	if (idev_V2->write(&pre_cmd, sizeof(pre_cmd)) < 0) {
		ipio_err("write pre cmd fail\n");
		ret = -1;
		goto out;
	}

	if (idev_V2->write(cmd, w_len) < 0) {
		ipio_err("write CMD fail\n");
		ilitek_dump_data_V2(cmd, 8, w_len, 0, "Write CMD Fail");
		ret = -1;
		goto out;
	}

	if (r_len > 0 && data != NULL) {
		ret = idev_V2->read(data, r_len);
		ilitek_dump_data_V2(data, 8, r_len, 0, "dump CMD");
	}

out:
	return ret;
}

int ilitek_thp_slave_mode_send_cmd(u8* cmd, u32 w_len, u8* data, u32 r_len)
{
	u8 pre_cmd = P5_X_READ_DATA_CTRL, checksum;
	int ret = 0;
	int thp_rlen = r_len;
	// int test = 0;
	u8 *buf = NULL;
	u8 temp[2];
	u8 dummy[11] = {0x82, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};

	ipio_info("slave mode read cmd\n");
/*
	if (r_len == 0 || data == NULL)
		thp_rlen = 4;
*/
	buf = kcalloc(thp_rlen, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("Failed to allocate buf mem\n");
		return 0;
	}
	if (idev_V2->actual_tp_mode == P5_X_FW_GESTURE_MODE ) {
    	ipio_debug("Write dummy cmd to wake up spi pll clock\n");
         if (idev_V2->spi_write_then_read(idev_V2->spi, dummy, sizeof(dummy), buf, 0) < 0) {
             ipio_err("spi write wake up cmd failed\n");
             goto out;
            }
    }
//1. 0xf6 wlen
	temp[0] = pre_cmd;
	temp[1] = w_len;

	ipio_info("cmd = 0x%x, len = %d\n", cmd[0], thp_rlen);
	if (idev_V2->write(temp, sizeof(temp)) < 0) {
		ipio_err("write pre cmd fail\n");
		ret = -1;
		goto out;
	}

	SPI_MODE_DELAY;
	atomic_set(&idev_V2->mp_int_check, ENABLE);
//2. write cmd
	if (idev_V2->write(cmd, w_len) < 0) {
		ipio_err("write CMD fail\n");
		ilitek_dump_data_V2(cmd, 8, w_len, 0, "Write CMD Fail");
		ret = -1;
		goto out;
	}
	if (r_len == 0 || data == NULL)
			goto out;

/* wait INT */
	if(false == idev_V2->mp_mode){
		  mdelay(3);
	}else{
		ret = ilitek_tddi_ic_check_int_stat_V2();
		if (ret < 0)
			goto out;
	}
//3. read data

	ret = idev_V2->read(buf, thp_rlen);
	if (ret == DO_SPI_RECOVER)
		goto out;

	if(thp_rlen > 30)
		ilitek_dump_data_V2(buf, 8, 30, 0, "dump CMD");
	else
		ilitek_dump_data_V2(buf, 8, thp_rlen, 0, "dump CMD");

	if (data == NULL)
		goto out;

	checksum = ilitek_calc_packet_checksum_V2(buf, thp_rlen - 1);

	if (checksum == buf[thp_rlen-1]) {
		ipio_memcpy(data, buf, r_len, r_len);
	} else {
	    ipio_memcpy(data, buf, r_len, r_len);
		ipio_err("THP get Wrong checksum or wrong header, checksum = %x, buf = %x\n", checksum, buf[thp_rlen-1]);
		ilitek_dump_data_V2(buf, 8, thp_rlen, 0, "dump CMD");
	}

out:
	ipio_kfree((void **)&buf);
	atomic_set(&idev_V2->mp_int_check, DISABLE);
	return ret;
}

int ilitek_thp_send_cmd(u8* cmd, u32 w_len, u8* data, u32 r_len)
{
	int ret = 0;
	if (idev_V2->spi_mode == SPI_ICE_MODE)
		ret = ilitek_thp_ice_mode_send_cmd(cmd, w_len, data, r_len);
	else
		ret = ilitek_thp_slave_mode_send_cmd(cmd, w_len, data, r_len);

	return ret;
}

static int core_spi_setup(u32 freq)
{
	ipio_info("spi clock = %d\n", freq);

	idev_V2->spi->mode = SPI_MODE_0;
	idev_V2->spi->bits_per_word = 8;

	if (spi_setup(idev_V2->spi) < 0) {
		ipio_err("Failed to setup spi device\n");
		return -ENODEV;
	}

	ipio_info("name = %s, bus_num = %d,cs = %d, mode = %d, speed = %d\n",
			idev_V2->spi->modalias,
			idev_V2->spi->master->bus_num,
			idev_V2->spi->chip_select,
			idev_V2->spi->mode,
			idev_V2->spi->max_speed_hz);
	return 0;
}
extern int ilitek_plat_probe(void);
static int ilitek_spi_probe(struct spi_device *client, struct device_node *np)
{
    int ret;
	ipio_info("ilitek spi probe\n");
	idev_V2 = devm_kzalloc(&client->dev, sizeof(struct ilitek_tddi_dev), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev_V2)) {
		ipio_err("Failed to allocate idev_V2 memory, %ld\n", PTR_ERR(idev_V2));
		ret = -ENOMEM;
		return ret;
	}

	idev_V2->fw_dma_buf = kzalloc(MAX_HEX_FILE_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev_V2->fw_dma_buf)) {
		ipio_err("fw kzalloc error\n");
		ret = -ENOMEM;
		goto errorcode1;
	}
	idev_V2->txbuf = kzalloc(SPI_TX_BUF_LEN, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev_V2->txbuf)) {
		ipio_err("txbuf kzalloc error\n");
		ret = -ENOMEM;
		goto errorcode2;
	}
	
	idev_V2->rxbuf = kzalloc(SPI_RX_BUF_LEN+1, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev_V2->rxbuf)) {
		ipio_err("rxbuf  kzalloc error\n");
		ret = -ENOMEM;
		goto errorcode3;
	}
	
	idev_V2->spi = client;
	idev_V2->dev = &client->dev;

	idev_V2->phys = "SPI";
	idev_V2->node = np;
	spi_set_drvdata(client, idev_V2);

	idev_V2->write = ilitek_spi_write;
	idev_V2->read = ilitek_spi_read;
#ifdef SPI_DMA_TRANSFER_SPLIT
	idev_V2->spi_write_then_read = ilitek_spi_write_then_read_split_V2;
#else
	idev_V2->spi_write_then_read = ilitek_spi_write_then_read_direct_V2;
#endif

	idev_V2->spi_speed = ilitek_tddi_ic_spi_speed_ctrl_V2;
	idev_V2->actual_tp_mode = P5_X_FW_DEMO_MODE;

	if (TDDI_RST_BIND)
		idev_V2->reset = TP_IC_WHOLE_RST;
	else
		idev_V2->reset = TP_HW_RST_ONLY;

	idev_V2->rst_edge_delay = 10;
	idev_V2->fw_open = REQUEST_FIRMWARE;
	idev_V2->fw_upgrade_mode = UPGRADE_IRAM;
	idev_V2->mp_move_code = ilitek_tddi_move_mp_code_iram_V2;
	idev_V2->gesture_move_code = ilitek_tddi_move_gesture_code_iram_V2;
	idev_V2->esd_recover = ilitek_tddi_wq_esd_spi_check_V2;
	idev_V2->ges_recover = ilitek_tddi_touch_esd_gesture_iram_V2;
	idev_V2->gesture_mode = P5_X_FW_GESTURE_INFO_MODE;
	idev_V2->wtd_ctrl = ON;
	idev_V2->report = ENABLE;
	idev_V2->netlink = DISABLE;
	idev_V2->debug_node_open = DISABLE;
	idev_V2->gesture_debug = DISABLE;
	idev_V2->mp_mode = false;

	if (ENABLE_GESTURE)
		idev_V2->gesture_V2 = ENABLE;
	/* get spi property */
	ret = of_property_read_u32(np, "spi-max-frequency",
			&idev_V2->spi->max_speed_hz);
	if (ret) {
		VTE("set default spi-max-frequency 8M");
		idev_V2->spi->max_speed_hz = 8*1000*1000;
	}
	ret = core_spi_setup(SPI_CLK);
    if(ret < 0)	
		goto errorcode4;
	
	//ilitek_parse_dt(np);
	return ilitek_plat_probe();
	errorcode4:
		kfree(idev_V2->rxbuf);
	errorcode3:
		kfree(idev_V2->txbuf);
	errorcode2:
		kfree(idev_V2->fw_dma_buf);
	errorcode1:
		devm_kfree(&client->dev,idev_V2);
		return ret;
		
}

static int ilitek_spi_remove(struct spi_device *client, struct device_node *np)
{
	ipio_info();
	ilitek_tddi_dev_remove_V2();
	return 0;
}


void ilitek_tddi_interface_dev_exit_V2(struct ilitek_hwif_info *hwif)
{
	struct touch_bus_info *info = (struct touch_bus_info *)hwif->info;

	ipio_info("remove spi dev\n");
	ipio_kfree((void **)&info);
}


static struct vts_spi_driver ili_spi_driver = {
	.probe		= ilitek_spi_probe,
	.remove		= ilitek_spi_remove,
	.compatible = "ilitek,ILI-ts-spi-v2",
	//.shutdown = nvts_shut_down,
};

static const int ic_numbers[] = {VTS_IC_ILI_9881H};
module_vts_driver(ili_tek, ic_numbers, vts_spi_drv_reigster(&ili_spi_driver), vts_spi_drv_unreigster(&ili_spi_driver));

