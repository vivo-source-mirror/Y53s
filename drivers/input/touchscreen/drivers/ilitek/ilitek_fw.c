#include "ilitek.h"
/* Firmware data with static array */

#define UPDATE_PASS		0
#define UPDATE_FAIL		-1
#define TIMEOUT_SECTOR		500
#define TIMEOUT_PAGE		3500
#define TIMEOUT_PROGRAM		10

struct touch_fw_data {
	u8 block_number;
	u32 start_addr;
	u32 end_addr;
	u32 new_fw_cb;
	int delay_after_upgrade;
	bool isCRC;
	bool isboot;
	int hex_tag;
} tfd_V2;

struct flash_block_info {
	char *name;
	u32 start;
	u32 end;
	u32 len;
	u32 mem_start;
	u32 fix_mem_start;
	u8 mode;
} fbi_V2[FW_BLOCK_INFO_NUM];

u8 gestrue_fw_V2[(10 * K)];


static u32 HexToDec(char *phex, s32 len)
{
	u32 ret = 0, temp = 0, i;
	s32 shift = (len - 1) * 4;

	for (i = 0; i < len; shift -= 4, i++) {
		if ((phex[i] >= '0') && (phex[i] <= '9'))
			temp = phex[i] - '0';
		else if ((phex[i] >= 'a') && (phex[i] <= 'f'))
			temp = (phex[i] - 'a') + 10;
		else if ((phex[i] >= 'A') && (phex[i] <= 'F'))
			temp = (phex[i] - 'A') + 10;
		else
			temp = 0;

		ret |= (temp << shift);
	}
	return ret;
}

static int CalculateCRC32(u32 start_addr, u32 len, u8 *pfw)
{
	int i = 0, j = 0;
	int crc_poly = 0x04C11DB7;
	int tmp_crc = 0xFFFFFFFF;

	for (i = start_addr; i < start_addr + len; i++) {
		tmp_crc ^= (pfw[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((tmp_crc & 0x80000000) != 0)
				tmp_crc = tmp_crc << 1 ^ crc_poly;
			else
				tmp_crc = tmp_crc << 1;
		}
	}
	return tmp_crc;
}

static int host_download_dma_check(u32 start_addr, u32 block_size)
{
	int count = 50;
	u32 busy = 0;

	/* dma1 src1 address */
	ilitek_ice_mode_write_V2(0x072104, start_addr, 4);
	/* dma1 src1 format */
	ilitek_ice_mode_write_V2(0x072108, 0x80000001, 4);
	/* dma1 dest address */
	ilitek_ice_mode_write_V2(0x072114, 0x00030000, 4);
	/* dma1 dest format */
	ilitek_ice_mode_write_V2(0x072118, 0x80000000, 4);
	/* Block size*/
	ilitek_ice_mode_write_V2(0x07211C, block_size, 4);

	idev_V2->chip->hd_dma_check_crc_off();
	/* Dma1 stop */
	ilitek_ice_mode_write_V2(0x072100, 0x02000000, 4);

	/* crc on */
	ilitek_ice_mode_write_V2(0x041016, 0x01, 1);

	/* clr int */
	ilitek_ice_mode_write_V2(0x048006, 0x1, 1);
	/* Dma1 start */
	ilitek_ice_mode_write_V2(0x072100, 0x01000000, 4);

	/* Polling BIT0 */
	while (count > 0) {
		mdelay(1);
		if (ilitek_ice_mode_read_V2(0x048006, &busy, sizeof(u8)) < 0)
			ipio_err("Read busy error\n");
		ipio_debug("busy = %x\n", busy);
		if ((busy & 0x01) == 1)
			break;
		count--;
	}

	if (count <= 0) {
		ipio_err("BIT0 is busy\n");
		return -1;
	}

	if (ilitek_ice_mode_read_V2(0x04101C, &busy, sizeof(u32)) < 0) {
		ipio_err("Read dma crc error\n");
		return -1;
	}
	return busy;
}

static int ilitek_tddi_fw_iram_read(u8 *buf, u32 start, int len)
{
	int limit = 4096;
	int addr = 0, loop = 0, tmp_len = len, cnt = 0;
	u8 cmd[4] = {0};

	if (!buf) {
		ipio_err("buf is null\n");
		return -ENOMEM;
	}

	if (len % limit)
		loop = (len / limit) + 1;
	else
		loop = len / limit;

	for (cnt = 0, addr = start; cnt < loop; cnt++, addr += limit) {
		if (tmp_len > limit)
			tmp_len = limit;

		cmd[0] = 0x25;
		cmd[3] = (char)((addr & 0x00FF0000) >> 16);
		cmd[2] = (char)((addr & 0x0000FF00) >> 8);
		cmd[1] = (char)((addr & 0x000000FF));

	if (idev_V2->write(cmd, 4) < 0) {
		ipio_err("Failed to write iram data\n");
		return -ENODEV;
	}

		if (idev_V2->read(buf + cnt * limit, tmp_len) < 0) {
			ipio_err("Failed to Read iram data\n");
			return -ENODEV;
		}

		tmp_len = len - cnt * limit;
		idev_V2->fw_update_stat = ((len - tmp_len) * 100) / len;
		ipio_info("Reading iram data .... %d%c", idev_V2->fw_update_stat, '%');
	}
	return 0;
}

int ilitek_tddi_fw_dump_iram_data_V2(u32 start, u32 end)
{
#if 0
	struct file *f = NULL;
	u8 *buf = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	mm_segment_t old_fs;
#endif
	loff_t pos = 0;
	int ret, wdt, i;
	int len;

	f = filp_open(DUMP_IRAM_PATH, O_WRONLY | O_CREAT | O_TRUNC, 644);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
		return -1;
	}

	ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	if (ret < 0) {
		filp_close(f, NULL);
		return ret;
	}

	wdt = ilitek_tddi_ic_watch_dog_ctrl_V2(ILI_READ, DISABLE);
	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl_V2(ILI_WRITE, DISABLE);

	len = end - start + 1;

	buf = kzalloc(len, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("Failed to allocate buf memory, %ld\n", PTR_ERR(buf));
		filp_close(f, NULL);
		ret = ENOMEM;
		goto out;
	}

	for (i = 0; i < len; i++)
		buf[i] = 0xFF;

	if (ilitek_tddi_fw_iram_read(buf, start, end) < 0){
		ipio_err("Read IRAM data failed\n");
		goto out;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	old_fs = get_fs();
	//set_fs(get_ds());
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, buf, len, &pos);
	set_fs(old_fs);
#else
	pos = 0;
	//kernel_write(f, buf, len, &pos);
#endif
out:
	if (wdt)
		ilitek_tddi_ic_watch_dog_ctrl_V2(ILI_WRITE, ENABLE);

	ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
	filp_close(f, NULL);
	ipio_vfree((void **)&buf);
	ipio_info("dump iram data success\n");
	
#endif
	return 0;
	
}

static int ilitek_tddi_fw_iram_program(u32 start, u8 *w_buf, u32 w_len, u32 split_len)
{
	int i = 0, j = 0, addr = 0;
	u32 end = start + w_len;
	bool fix_4_alignment = false;

	if (split_len % 4 > 0)
		ipio_err("Since split_len must be four-aligned, it must be a multiple of four");

	if (split_len != 0) {
		for (addr = start, i = 0; addr < end; addr += split_len, i += split_len) {
			if ((addr + split_len) > end) {
				//split_len = end % split_len;
                split_len = end - addr;
				if (split_len % 4 != 0)
					fix_4_alignment = true;
			}
			idev_V2->fw_dma_buf[0] = SPI_WRITE;
			idev_V2->fw_dma_buf[1] = 0x25;
			idev_V2->fw_dma_buf[2] = (char)((addr & 0x000000FF));
			idev_V2->fw_dma_buf[3] = (char)((addr & 0x0000FF00) >> 8);
			idev_V2->fw_dma_buf[4] = (char)((addr & 0x00FF0000) >> 16);

			for (j = 0; j < split_len; j++)
				idev_V2->fw_dma_buf[5 + j] = w_buf[i + j];

			if (fix_4_alignment) {
				ipio_info("org split_len = 0x%X\n", split_len);
				ipio_info("idev_V2->fw_dma_buf[5 + 0x%X] = 0x%X\n", split_len - 4, idev_V2->fw_dma_buf[5 + split_len - 4]);
				ipio_info("idev_V2->fw_dma_buf[5 + 0x%X] = 0x%X\n", split_len - 3, idev_V2->fw_dma_buf[5 + split_len - 3]);
				ipio_info("idev_V2->fw_dma_buf[5 + 0x%X] = 0x%X\n", split_len - 2, idev_V2->fw_dma_buf[5 + split_len - 2]);
				ipio_info("idev_V2->fw_dma_buf[5 + 0x%X] = 0x%X\n", split_len - 1, idev_V2->fw_dma_buf[5 + split_len - 1]);
				for (j = 0; j < (4 - (split_len % 4)); j++) {
					idev_V2->fw_dma_buf[5 + j + split_len] = 0xFF;
					ipio_info("idev_V2->fw_dma_buf[5 + 0x%X] = 0x%X\n",j + split_len, idev_V2->fw_dma_buf[5 + j + split_len]);
				}

				ipio_info("split_len %% 4 = %d\n", split_len % 4);
				split_len = split_len + (4 - (split_len % 4));
				ipio_info("fix split_len = 0x%X\n", split_len);
			}
			if (idev_V2->spi_write_then_read(idev_V2->spi, idev_V2->fw_dma_buf, split_len + 5, NULL, 0)) {
				ipio_err("Failed to write data via SPI in host download (%x)\n", split_len + 5);
				return -EIO;
			}
		}
	} else {
		for (i = 0; i < MAX_HEX_FILE_SIZE; i++)
			idev_V2->fw_dma_buf[i] = 0xFF;
		idev_V2->fw_dma_buf[0] = SPI_WRITE;
		idev_V2->fw_dma_buf[1] = 0x25;
		idev_V2->fw_dma_buf[2] = (char)((start & 0x000000FF));
		idev_V2->fw_dma_buf[3] = (char)((start & 0x0000FF00) >> 8);
		idev_V2->fw_dma_buf[4] = (char)((start & 0x00FF0000) >> 16);

		memcpy(&idev_V2->fw_dma_buf[5], w_buf, w_len);
		if (w_len % 4 != 0) {
			ipio_info("org w_len = %d\n", w_len);
			w_len = w_len + (4 - (w_len % 4));
			ipio_info("w_len = %d w_len %% 4 = %d\n", w_len, w_len % 4);
		}
		/* It must be supported by platforms that have the ability to transfer all data at once. */
		if (idev_V2->spi_write_then_read(idev_V2->spi, idev_V2->fw_dma_buf, w_len + 5, NULL, 0) < 0) {
			ipio_err("Failed to write data via SPI in host download (%x)\n", w_len + 5);
			return -EIO;
		}
	}
	return 0;
}

static int ilitek_tddi_fw_check_hex_hw_crc(u8 *pfw)
{
	u32 i = 0, len = 0;
	u32 hex_crc = 0, hw_crc;

	for (i = 0; i < ARRAY_SIZE(fbi_V2); i++) {
		if (fbi_V2[i].end == 0)
			continue;

		len = fbi_V2[i].end - fbi_V2[i].start + 1 - 4;

		hex_crc = CalculateCRC32(fbi_V2[i].start, len, pfw);
		hw_crc = ilitek_tddi_fw_read_hw_crc_V2(fbi_V2[i].start, len);
//zhoucheng 20200428
		if(!hw_crc) {
			ipio_err("Read HW CRC failed!!!\n");
			return UPDATE_FAIL;
		}

		ipio_info("Block = %d, Hex CRC = %x, HW CRC = %x\n", i, hex_crc, hw_crc);

		if (hex_crc != hw_crc) {
			ipio_err("Hex and HW CRC NO matched !!!\n");
			return UPDATE_FAIL;
		}
	}

	ipio_info("Hex and HW CRC match!\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_flash_poll_busy(int timer)
{
	int ret = UPDATE_PASS, retry = timer;
	u8 cmd = 0x5;
	u32 temp = 0x01;//zhoucheng 20200428

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, cmd, 1);

	do {
		ilitek_ice_mode_write_V2(FLASH2_ADDR, 0xFF, 1); /* Dummy */
		mdelay(1);
		if (ilitek_ice_mode_read_V2(FLASH4_ADDR, &temp, sizeof(u8)) < 0)
			ipio_err("Read flash busy error\n");

		if ((temp & 0x3) == 0)
			break;
	} while (--retry >= 0);

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Flash polling busy timeout ! tmp = %x\n", temp);
		ret = UPDATE_FAIL;
	}

	return ret;
}

void ilitek_tddi_flash_clear_dma_V2(void)
{
	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write_V2(FLASH0_ADDR, FLASH0_reg_preclk_sel, (2 << 16));
	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x01, 1);	/* CS high */

	ilitek_ice_mode_bit_mask_write_V2(FLASH4_ADDR, FLASH4_reg_flash_dma_trigger_en, (0 << 24));
	ilitek_ice_mode_bit_mask_write_V2(FLASH0_ADDR, FLASH0_reg_rx_dual, (0 << 24));

	ilitek_ice_mode_write_V2(FLASH3_reg_rcv_cnt, 0x00, 1);
	ilitek_ice_mode_write_V2(FLASH4_reg_rcv_data, 0xFF, 1);
}

int ilitek_tddi_flash_read_int_flag_V2(void)
{
	int retry = 10; //zhoucheng 20200428
	u32 data = 0;

	do {
		if (ilitek_ice_mode_read_V2(INTR1_ADDR & BIT(25), &data, sizeof(u32)) < 0)
			ipio_err("Read flash int flag error\n");

		ipio_debug("int flag = %x\n", data);
		if (data)
			break;
		mdelay(1); //zhoucheng 20200428
	} while (--retry >= 0);

	if (retry <= 0) {
		ipio_err("Read Flash INT flag timeout !, flag = 0x%x\n", data);
		return -1;
	}
	return 0;
}

void ilitek_tddi_flash_dma_write_V2(u32 start, u32 end, u32 len)
{
	ilitek_ice_mode_bit_mask_write_V2(FLASH0_ADDR, FLASH0_reg_preclk_sel, 1 << 16);

	ilitek_ice_mode_write_V2(FLASH0_reg_flash_csb, 0x00, 1);	/* CS low */
	ilitek_ice_mode_write_V2(FLASH1_reg_flash_key1, 0x66aa55, 3);	/* Key */

	ilitek_ice_mode_write_V2(FLASH2_reg_tx_data, 0x0b, 1);

	if (ilitek_tddi_flash_read_int_flag_V2() < 0) {
		ipio_err("Write 0xb timeout \n");
		return;
	}

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write_V2(FLASH2_reg_tx_data, (start & 0xFF0000) >> 16, 1);

	if (ilitek_tddi_flash_read_int_flag_V2() < 0) {
		ipio_err("Write addr1 timeout\n");
		return;
	}

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write_V2(FLASH2_reg_tx_data, (start & 0x00FF00) >> 8, 1);

	if (ilitek_tddi_flash_read_int_flag_V2() < 0) {
		ipio_err("Write addr2 timeout\n");
		return;
	}

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write_V2(FLASH2_reg_tx_data, (start & 0x0000FF), 1);

	if (ilitek_tddi_flash_read_int_flag_V2() < 0) {
		ipio_err("Write addr3 timeout\n");
		return;
	}

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write_V2(FLASH0_ADDR, FLASH0_reg_rx_dual, 0 << 24);

	ilitek_ice_mode_write_V2(FLASH2_reg_tx_data, 0x00, 1);	/* Dummy */

	if (ilitek_tddi_flash_read_int_flag_V2() < 0) {
		ipio_err("Write dummy timeout\n");
		return;
	}

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write_V2(FLASH3_reg_rcv_cnt, len, 4);	/* Write Length */
}

static void ilitek_tddi_flash_write_enable(void)
{
	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x6, 1);
	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}

u32 ilitek_tddi_fw_read_hw_crc_V2(u32 start, u32 end)
{
	int retry = 30; //zhoucheng 20200428
	u32 busy = 0;
	u32 write_len = end;
	u32 flash_crc = 0;

	if (write_len > idev_V2->chip->max_count) {
		ipio_err("The length (%x) written into firmware is greater than max count (%x)\n",
			write_len, idev_V2->chip->max_count);
		return -1;
	}

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x3b, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0x0000FF), 1);
	ilitek_ice_mode_write_V2(0x041003, 0x01, 1); /* Enable Dio_Rx_dual */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0xFF, 1); /* Dummy */
	ilitek_ice_mode_write_V2(0x04100C, write_len, 3); /* Set Receive count */
	ilitek_ice_mode_write_V2(0x048007, 0x02, 1);/* Clear Int Flag */
	ilitek_ice_mode_write_V2(0x041016, 0x00, 1);
	ilitek_ice_mode_write_V2(0x041016, 0x01, 1);	/* Checksum_En */

	ilitek_ice_mode_write_V2(FLASH4_ADDR, 0xFF, 1); /* Start to receive */

	do {
		if (ilitek_ice_mode_read_V2(0x048007, &busy, sizeof(u8)) < 0)
			ipio_err("Read busy error\n");

		ipio_debug("busy = %x\n", busy);
		if (((busy >> 1) & 0x01) == 0x01)
			break;
		mdelay(1);//zhoucheng 20200428
	} while (--retry >= 0);

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Read HW CRC timeout !, busy = 0x%x\n", busy);
		return -1;
	}

	ilitek_ice_mode_write_V2(0x041003, 0x0, 1); /* Disable dio_Rx_dual */

	if (ilitek_ice_mode_read_V2(0x04101C, &flash_crc, sizeof(u32)) < 0) {
		ipio_err("Read hw crc error\n");
		return -1;
	}

	return flash_crc;
}

int ilitek_tddi_fw_read_flash_data_V2(u32 start, u32 end, u8 *data, int len)
{
	u32 i, index = 0, precent;
	u32 tmp;

	if (end - start > len) {
		ipio_err("the length (%d) reading crc is over than len(%d)\n", end - start, len);
		return -1;
	}

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x03, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, (start & 0x0000FF), 1);

	for (i = start; i <= end; i++) {
		ilitek_ice_mode_write_V2(FLASH2_ADDR, 0xFF, 1); /* Dummy */

		if (ilitek_ice_mode_read_V2(FLASH4_ADDR, &tmp, sizeof(u8)) < 0)
			ipio_err("Read flash data error!\n");

		data[index] = tmp;
		index++;
		precent = (i * 100) / end;
		ipio_debug("Reading flash data .... %d%c", precent, '%');
	}

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
	return 0;
}

int ilitek_tddi_fw_dump_flash_data_V2(u32 start, u32 end, bool user)
{
#if 0
	struct file *f = NULL;
	u8 *buf = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	mm_segment_t old_fs;
#endif
	loff_t pos = 0;
	u32 start_addr, end_addr;
	int ret, length;

	f = filp_open(DUMP_FLASH_PATH, O_WRONLY | O_CREAT | O_TRUNC, 644);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
		return -1;
	}

	ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	if (ret < 0)
		return ret;

	if (user) {
		start_addr = 0x0;
		end_addr = 0x1FFFF;
	} else {
		start_addr = start;
		end_addr = end;
	}

	length = end_addr - start_addr + 1;
	ipio_info("len = %d\n", length);

	buf = vmalloc(length * sizeof(u8));
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("Failed to allocate buf memory, %ld\n", PTR_ERR(buf));
		filp_close(f, NULL);
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
		return -1;
	}

	ilitek_tddi_fw_read_flash_data_V2(start_addr, end_addr, buf, length);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	old_fs = get_fs();
	//set_fs(get_ds());
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, buf, length, &pos);
	set_fs(old_fs);
#else
	pos = 0;
	//kernel_write(f, buf, length, &pos);
#endif
	filp_close(f, NULL);
	ipio_vfree((void **)&buf);
	ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
	ipio_info("dump flash success\n");
#endif
	return 0;
}

static void ilitek_tddi_flash_protect(bool enable)
{
	ipio_info("%s flash protection\n", enable ? "Enable" : "Disable");

	ilitek_tddi_flash_write_enable();

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x1, 1);
	ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x0, 1);

	switch (idev_V2->flash_mid) {
	case 0xEF:
		if (idev_V2->flash_devid == 0x6012 || idev_V2->flash_devid == 0x6011) {
			if (enable)
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x7E, 1);
			else
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x0, 1);
		}
		break;
	case 0xC8:
		if (idev_V2->flash_devid == 0x6012 || idev_V2->flash_devid == 0x6013) {
			if (enable)
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x7A, 1);
			else
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x0, 1);
		}
		break;
	default:
		ipio_err("Can't find flash id(0x%x), ignore protection\n", idev_V2->flash_mid);
		break;
	}

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}

static int ilitek_tddi_fw_check_ver(u8 *pfw)
{
	int i, crc_byte_len = 4;
	u8 flash_crc[4] = {0};
	u32 start_addr = 0, end_addr = 0;
	u32 block_crc, flash_crc_cb;

	/* Get current firmware/protocol version */
	ilitek_tddi_ic_get_protocl_ver_V2();
	ilitek_tddi_ic_get_fw_ver_V2();

	/* Check FW version */
	ipio_info("New FW ver = 0x%x, Current FW ver = 0x%x\n", tfd_V2.new_fw_cb, idev_V2->chip->fw_ver);
	if (tfd_V2.new_fw_cb != idev_V2->chip->fw_ver) {
		ipio_info("FW version is different, do upgrade\n");
		return UPDATE_FAIL;
	}

	ipio_info("FW version is the same, check Flash and HW CRC if there's corruption.\n");

	/* Check Flash and HW CRC with last 4 bytes in each block */
	for (i = 0; i < ARRAY_SIZE(fbi_V2); i++) {
		start_addr = fbi_V2[i].start;
		end_addr = fbi_V2[i].end;

		/* Invaild end address */
		if (end_addr == 0)
			continue;

		if (ilitek_tddi_fw_read_flash_data_V2(end_addr - crc_byte_len + 1, end_addr,
					flash_crc, sizeof(flash_crc)) < 0) {
			ipio_err("Read Flash failed\n");
			return UPDATE_FAIL;
		}

		flash_crc_cb = flash_crc[0] << 24 | flash_crc[1] << 16 | flash_crc[2] << 8 | flash_crc[3];

		block_crc = ilitek_tddi_fw_read_hw_crc_V2(start_addr, end_addr - start_addr - crc_byte_len + 1);
		if(!block_crc) {
			ipio_err("Read HW CRC failed!!!\n");
			return UPDATE_FAIL;
		}

		ipio_info("Block = %d, HW CRC = 0x%06x, Flash CRC = 0x%06x\n", i, block_crc, flash_crc_cb);

		/* Compare Flash CRC with HW CRC */
		if (flash_crc_cb != block_crc) {
			ipio_info("Both are different, do update\n");
			return UPDATE_FAIL;
		}
		memset(flash_crc, 0, sizeof(flash_crc));
	}

	if (idev_V2->force_fw_update == ENABLE) {
		ipio_info("update by node, force update\n");
		return UPDATE_FAIL;
	}

	ipio_info("Both are the same, no need to update\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_iram_upgrade(u8 *pfw)
{
	int i, ret = UPDATE_PASS;
	u32 mode, crc, dma, iram_crc;
	u8 *fw_ptr = NULL, crc_temp[4], crc_len = 4;
	bool iram_crc_err = false;
	s64 ktime_start;	
	s64 ktime_start1;

	ktime_start = ktime_to_ms(ktime_get());
	if (idev_V2->actual_tp_mode != P5_X_FW_GESTURE_MODE)
		ilitek_tddi_reset_ctrl_V2(idev_V2->reset);

	ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	if (ret < 0)
		return ret;

	ret = ilitek_tddi_ic_watch_dog_ctrl_V2(ILI_WRITE, DISABLE);
	if (ret < 0)
		return ret;

	fw_ptr = pfw;
	if (idev_V2->actual_tp_mode == P5_X_FW_TEST_MODE) {
		mode = MP;
	} else if (idev_V2->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		mode = GESTURE;
		fw_ptr = gestrue_fw_V2;
		crc_len = 0;
	} else {
		mode = AP;
	}

	
	ktime_start1 = ktime_to_ms(ktime_get());
	/* Program data to iram acorrding to each block */
	for (i = 0; i < ARRAY_SIZE(fbi_V2); i++) {
		if (fbi_V2[i].mode == mode && fbi_V2[i].len != 0) {
			ipio_info("Download %s code from hex 0x%x to IRAM 0x%x, len = 0x%x\n",
					fbi_V2[i].name, fbi_V2[i].start, fbi_V2[i].mem_start, fbi_V2[i].len);

#ifdef SPI_DMA_TRANSFER_SPLIT
			ilitek_tddi_fw_iram_program(fbi_V2[i].mem_start, (fw_ptr + fbi_V2[i].start), fbi_V2[i].len, SPI_UPGRADE_LEN);
#else
			ilitek_tddi_fw_iram_program(fbi_V2[i].mem_start, (fw_ptr + fbi_V2[i].start), fbi_V2[i].len, 0);
#endif

			crc = CalculateCRC32(fbi_V2[i].start, fbi_V2[i].len - crc_len, fw_ptr);
			dma = host_download_dma_check(fbi_V2[i].mem_start, fbi_V2[i].len - crc_len);

			if (mode != GESTURE) {
				ilitek_tddi_fw_iram_read(crc_temp, (fbi_V2[i].mem_start + fbi_V2[i].len - crc_len), sizeof(crc_temp));
				iram_crc = crc_temp[0] << 24 | crc_temp[1] << 16 |crc_temp[2] << 8 | crc_temp[3];
				if (iram_crc != dma)
					iram_crc_err = true;
			}

			ipio_info("%s CRC is %s hex(%x) : dma(%x) : iram(%x), calculation len is 0x%x\n",
				fbi_V2[i].name,((crc != dma)||(iram_crc_err)) ? "Invalid !" : "Correct !", crc, dma, iram_crc, fbi_V2[i].len - crc_len);


			if ((crc != dma)|| iram_crc_err) {
				ipio_err("CRC Error! print iram data with first 16 bytes\n");
				//ilitek_tddi_fw_print_iram_data(fbi_V2[i].mem_start, fbi_V2[i].len);
				return UPDATE_FAIL;
			}
			idev_V2->fw_update_stat = 90;
		}
	}
	
	VTI("Update_firmware_program! take %lld ms", ktime_to_ms(ktime_get()) - ktime_start1);
	if (idev_V2->actual_tp_mode != P5_X_FW_GESTURE_MODE)
		ilitek_tddi_reset_ctrl_V2(TP_IC_CODE_RST);

	ilitek_ice_mode_ctrl_V2(DISABLE, OFF);	
	/* Waiting for fw ready */
	msleep(50);
	VTI("Update_firmware_success! take %lld ms", ktime_to_ms(ktime_get()) - ktime_start);
	return ret;
}

static int ilitek_tddi_fw_flash_program(u8 *pfw)
{
	u8 buf[512] = {0};
	u32 i = 0, addr = 0, k = 0, recv_addr = 0;
	bool skip = true;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi_V2[i].end == 0)
			continue;

		if (fbi_V2[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi_V2[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Programing from (0x%x) to (0x%x)\n", i, fbi_V2[i].start, fbi_V2[i].end);

		for (addr = fbi_V2[i].start; addr < fbi_V2[i].end; addr += idev_V2->program_page) {
			buf[0] = 0x25;
			buf[3] = 0x04;
			buf[2] = 0x10;
			buf[1] = 0x08;

			for (k = 0; k < idev_V2->program_page; k++) {
				if (addr + k <= tfd_V2.end_addr)
					buf[4 + k] = pfw[addr + k];
				else
					buf[4 + k] = 0xFF;

				if (buf[4 + k] != 0xFF)
					skip = false;
			}

			if (skip) {
				ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
			ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x2, 1);
			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write_V2(FLASH2_ADDR, recv_addr, 3);

			if (idev_V2->write(buf, idev_V2->program_page + 4) < 0) {
				ipio_err("Failed to program data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
				addr, k, addr + k);
				ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (idev_V2->flash_mid == 0xEF) {
				mdelay(1);
			} else {
				if (ilitek_tddi_flash_poll_busy(TIMEOUT_PROGRAM) < 0)
					return UPDATE_FAIL;
			}

			/* holding the status until finish this upgrade. */
			idev_V2->fw_update_stat = (addr * 101) / tfd_V2.end_addr;
			if (idev_V2->fw_update_stat > 90)
				idev_V2->fw_update_stat = 90;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_flash_erase(void)
{
	int ret = 0;
	u32 i = 0, addr = 0, recv_addr = 0;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi_V2[i].end == 0)
			continue;

		if (fbi_V2[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi_V2[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Erasing from (0x%x) to (0x%x) \n", i, fbi_V2[i].start, fbi_V2[i].end);

		for (addr = fbi_V2[i].start; addr <= fbi_V2[i].end; addr += idev_V2->flash_sector) {
			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */

			if (addr == fbi_V2[AP].start)
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0xD8, 1);
			else
				ilitek_ice_mode_write_V2(FLASH2_ADDR, 0x20, 1);

			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write_V2(FLASH2_ADDR, recv_addr, 3);

			ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			/* Waitint for flash setting ready */
			mdelay(1);

			if (addr == fbi_V2[AP].start)
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_PAGE);
			else
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_SECTOR);

			if (ret < 0)
				return UPDATE_FAIL;

			ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (fbi_V2[i].start == fbi_V2[AP].start)
				break;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_flash_upgrade(u8 *pfw)
{
	int ret = UPDATE_PASS;

	ilitek_tddi_reset_ctrl_V2(idev_V2->reset);

	ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	if (ret < 0)
		return UPDATE_FAIL;

	ret = ilitek_tddi_ic_watch_dog_ctrl_V2(ILI_WRITE, DISABLE);
	if (ret < 0)
		return ret;

	ret = ilitek_tddi_fw_check_ver(pfw);
	if (ret == UPDATE_PASS) {
			if (ilitek_ice_mode_ctrl_V2(DISABLE, OFF) < 0) {
			ipio_err("Disable ice mode failed, call reset instead\n");
			ilitek_tddi_reset_ctrl_V2(idev_V2->reset);
			return UPDATE_PASS;
		}
		return UPDATE_PASS;
	}

	ret = ilitek_tddi_fw_flash_erase();
	if (ret == UPDATE_FAIL)
		return UPDATE_FAIL;

	ret = ilitek_tddi_fw_flash_program(pfw);
	if (ret == UPDATE_FAIL)
		return UPDATE_FAIL;

	ret = ilitek_tddi_fw_check_hex_hw_crc(pfw);
	if (ret == UPDATE_FAIL)
		return UPDATE_FAIL;

	/* We do have to reset chip in order to move new code from flash to iram. */
	ilitek_tddi_reset_ctrl_V2(idev_V2->reset);
	return ret;
}

static void ilitek_tddi_fw_update_block_info(u8 *pfw, u8 type)
{
	u32 ges_area_section, ges_info_addr, ges_fw_start, ges_fw_end;
	u32 ap_end, ap_len;
	u32 fw_info_addr = 0, fw_mp_ver_addr = 0;
	ipio_info("Upgarde = %s, Tag = %x\n", type ? "IRAM" : "Flash", tfd_V2.hex_tag);

	if (type == UPGRADE_IRAM) {
		if (tfd_V2.hex_tag == BLOCK_TAG_AF) {
			fbi_V2[AP].mem_start = (fbi_V2[AP].fix_mem_start != INT_MAX) ? fbi_V2[AP].fix_mem_start : 0;
			fbi_V2[DATA].mem_start = (fbi_V2[DATA].fix_mem_start != INT_MAX) ? fbi_V2[DATA].fix_mem_start : DLM_START_ADDRESS;
			fbi_V2[TUNING].mem_start = (fbi_V2[TUNING].fix_mem_start != INT_MAX) ? fbi_V2[TUNING].fix_mem_start :  fbi_V2[DATA].mem_start + fbi_V2[DATA].len;
			fbi_V2[MP].mem_start = (fbi_V2[MP].fix_mem_start != INT_MAX) ? fbi_V2[MP].fix_mem_start :  0;
			fbi_V2[GESTURE].mem_start = (fbi_V2[GESTURE].fix_mem_start != INT_MAX) ? fbi_V2[GESTURE].fix_mem_start :	 0;

			/* Parsing gesture_V2 info form AP code */
			ges_info_addr = (fbi_V2[AP].end + 1 - 60);
			ges_area_section = (pfw[ges_info_addr + 3] << 24) + (pfw[ges_info_addr + 2] << 16) + (pfw[ges_info_addr + 1] << 8) + pfw[ges_info_addr];
			fbi_V2[GESTURE].mem_start = (pfw[ges_info_addr + 7] << 24) + (pfw[ges_info_addr + 6] << 16) + (pfw[ges_info_addr + 5] << 8) + pfw[ges_info_addr + 4];
			ap_end = (pfw[ges_info_addr + 11] << 24) + (pfw[ges_info_addr + 10] << 16) + (pfw[ges_info_addr + 9] << 8) + pfw[ges_info_addr + 8];
			ap_len = ap_end - fbi_V2[GESTURE].mem_start + 1;
			ges_fw_start = (pfw[ges_info_addr + 15] << 24) + (pfw[ges_info_addr + 14] << 16) + (pfw[ges_info_addr + 13] << 8) + pfw[ges_info_addr + 12];
			ges_fw_end = (pfw[ges_info_addr + 19] << 24) + (pfw[ges_info_addr + 18] << 16) + (pfw[ges_info_addr + 17] << 8) + pfw[ges_info_addr + 16];
		if (ges_fw_end != ges_fw_start)
			fbi_V2[GESTURE].len = ges_fw_end - ges_fw_start; //address was automatically added one when it was generated

		/* update gesture address */
		fbi_V2[GESTURE].start = 0;
		} else {
			memset(fbi_V2, 0x0, sizeof(fbi_V2));
			fbi_V2[AP].start = 0;
			fbi_V2[AP].mem_start = 0;
			fbi_V2[AP].len = MAX_AP_FIRMWARE_SIZE;

			fbi_V2[DATA].start = DLM_HEX_ADDRESS;
			fbi_V2[DATA].mem_start = DLM_START_ADDRESS;
			fbi_V2[DATA].len = MAX_DLM_FIRMWARE_SIZE;

			fbi_V2[MP].start = MP_HEX_ADDRESS;
			fbi_V2[MP].mem_start = 0;
			fbi_V2[MP].len = MAX_MP_FIRMWARE_SIZE;

			/* Parsing gesture_V2 info form AP code */
			ges_info_addr = (MAX_AP_FIRMWARE_SIZE - 60);
			ges_area_section = (pfw[ges_info_addr + 3] << 24) + (pfw[ges_info_addr + 2] << 16) + (pfw[ges_info_addr + 1] << 8) + pfw[ges_info_addr];
			fbi_V2[GESTURE].mem_start = (pfw[ges_info_addr + 7] << 24) + (pfw[ges_info_addr + 6] << 16) + (pfw[ges_info_addr + 5] << 8) + pfw[ges_info_addr + 4];
			ap_end = (pfw[ges_info_addr + 11] << 24) + (pfw[ges_info_addr + 10] << 16) + (pfw[ges_info_addr + 9] << 8) + pfw[ges_info_addr + 8];
			ap_len = fbi_V2[GESTURE].mem_start - ap_end + 1;
			ges_fw_start = (pfw[ges_info_addr + 15] << 24) + (pfw[ges_info_addr + 14] << 16) + (pfw[ges_info_addr + 13] << 8) + pfw[ges_info_addr + 12];
			ges_fw_end = (pfw[ges_info_addr + 19] << 24) + (pfw[ges_info_addr + 18] << 16) + (pfw[ges_info_addr + 17] << 8) + pfw[ges_info_addr + 16];
		if (ges_fw_end != ges_fw_start)
			fbi_V2[GESTURE].len = ges_fw_end - ges_fw_start; //address was automatically added one when it was generated

		/* update gesture address */
		fbi_V2[GESTURE].start = 0;
		}

		memset(gestrue_fw_V2, 0xff, sizeof(gestrue_fw_V2));

		/* Copy gesture_V2 data */
		if (fbi_V2[GESTURE].mem_start != 0xffffffff && ges_fw_start != 0xffffffff && fbi_V2[GESTURE].mem_start != 0 && ges_fw_start != 0)
			ipio_memcpy(gestrue_fw_V2, (pfw + ges_fw_start), fbi_V2[GESTURE].len, sizeof(gestrue_fw_V2));
		else
			ipio_err("There is no gesture_V2 data inside fw\n");

		ipio_info("==== Gesture loader info ====\n");
		ipio_info("ap_start = 0x%x, ap_end = 0x%x, ap_len = 0x%x\n", fbi_V2[GESTURE].mem_start, ap_end, ap_len);
		ipio_info("gesture_start = 0x%x, gesture_end = 0x%x, gesture_len = 0x%x\n", ges_fw_start, ges_fw_end, fbi_V2[GESTURE].len);
		ipio_info("=============================\n");

		fbi_V2[AP].name = "AP";
		fbi_V2[DATA].name = "DATA";
		fbi_V2[TUNING].name = "TUNING";
		fbi_V2[MP].name = "MP";
		fbi_V2[GESTURE].name = "GESTURE";

		/* upgrade mode define */
		fbi_V2[DATA].mode = fbi_V2[AP].mode = fbi_V2[TUNING].mode = AP;
		fbi_V2[MP].mode = MP;
		fbi_V2[GESTURE].mode = GESTURE;
	}

	/* Get hex fw vers */
	tfd_V2.new_fw_cb = (pfw[FW_VER_ADDR] << 24) | (pfw[FW_VER_ADDR + 1] << 16) |
			(pfw[FW_VER_ADDR + 2] << 8) | (pfw[FW_VER_ADDR + 3]);
	/* Copy fw info */
	fw_info_addr = fbi_V2[AP].end - INFO_HEX_ST_ADDR;
	ipio_info("Parsing hex info start addr = 0x%x\n", fw_info_addr);
	ipio_memcpy(idev_V2->fw_info, (pfw + fw_info_addr), sizeof(idev_V2->fw_info), sizeof(idev_V2->fw_info));

	/* copy fw mp ver */
	fw_mp_ver_addr = fbi_V2[MP].end - INFO_MP_HEX_ADDR;
	ipio_info("Parsing hex mp ver addr = 0x%x\n", fw_mp_ver_addr);
	ipio_memcpy(idev_V2->fw_mp_ver, pfw + fw_mp_ver_addr, sizeof(idev_V2->fw_mp_ver), sizeof(idev_V2->fw_mp_ver));

	/* Calculate update address */
	ipio_info("New FW ver = 0x%x\n", tfd_V2.new_fw_cb);
	ipio_info("star_addr = 0x%06X, end_addr = 0x%06X, Block Num = %d\n", tfd_V2.start_addr, tfd_V2.end_addr, tfd_V2.block_number);
}


static int ilitek_tddi_fw_hex_convert(u8 *phex, int size, u8 *pfw)
{
	int block = 0;
	u32 i = 0, j = 0, k = 0, num = 0;
	u32 len = 0, addr = 0, type = 0;
	u32 start_addr = 0x0, end_addr = 0x0, ex_addr = 0;
	u32 offset, hex_crc, data_crc;

	memset(fbi_V2, 0x0, sizeof(fbi_V2));

	/* Parsing HEX file */
	for (; i < size;) {
		len = HexToDec(&phex[i + 1], 2);
		addr = HexToDec(&phex[i + 3], 4);
		type = HexToDec(&phex[i + 7], 2);

		if (type == 0x04) {
			ex_addr = HexToDec(&phex[i + 9], 4);
		} else if (type == 0x02) {
			ex_addr = HexToDec(&phex[i + 9], 4);
			ex_addr = ex_addr >> 12;
		} else if (type == BLOCK_TAG_AE || type == BLOCK_TAG_AF) {
			/* insert block info extracted from hex */
			tfd_V2.hex_tag = type;
			if (tfd_V2.hex_tag == BLOCK_TAG_AF)
				num = HexToDec(&phex[i + 9 + 6 + 6], 2);
			else
				num = block;

			if (num > (FW_BLOCK_INFO_NUM - 1)) {
				ipio_err("ERROR! block num is larger than its define (%d, %d)\n",
						num, FW_BLOCK_INFO_NUM - 1);
				return -EINVAL;
			}

			fbi_V2[num].start = HexToDec(&phex[i + 9], 6);
			fbi_V2[num].end = HexToDec(&phex[i + 9 + 6], 6);
			fbi_V2[num].fix_mem_start = INT_MAX;
			fbi_V2[num].len = fbi_V2[num].end - fbi_V2[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x", num, fbi_V2[num].start, fbi_V2[num].end);

			block++;
		} else if (type == BLOCK_TAG_B0 && tfd_V2.hex_tag == BLOCK_TAG_AF) {
			num = HexToDec(&phex[i + 9 + 6], 2);

			if (num > (FW_BLOCK_INFO_NUM - 1)) {
				ipio_err("ERROR! block num is larger than its define (%d, %d)\n",
						num, FW_BLOCK_INFO_NUM - 1);
				return -EINVAL;
			}

			fbi_V2[num].fix_mem_start = HexToDec(&phex[i + 9], 6);
			ipio_info("Tag 0xB0: change Block[%d] to addr = 0x%x\n", num, fbi_V2[num].fix_mem_start);
		}

		addr = addr + (ex_addr << 16);

		if (phex[i + 1 + 2 + 4 + 2 + (len * 2) + 2] == 0x0D)
			offset = 2;
		else
			offset = 1;

		if (addr > MAX_HEX_FILE_SIZE) {
			ipio_err("Invalid hex format %d\n", addr);
			return -1;
		}

		if (type == 0x00) {
			end_addr = addr + len;
			if (addr < start_addr)
				start_addr = addr;
			/* fill data */
			for (j = 0, k = 0; j < (len * 2); j += 2, k++)
				pfw[addr + k] = HexToDec(&phex[i + 9 + j], 2);
		}
		i += 1 + 2 + 4 + 2 + (len * 2) + 2 + offset;
	}

	/* Check the content of hex file by comparsing parsed data to the crc at last 4 bytes */
	for (i = 0; i < ARRAY_SIZE(fbi_V2); i++) {
		if (fbi_V2[i].end == 0)
			continue;
		ex_addr = fbi_V2[i].end;
		data_crc = CalculateCRC32(fbi_V2[i].start, fbi_V2[i].len - 4, pfw);
		hex_crc = pfw[ex_addr - 3] << 24 | pfw[ex_addr - 2] << 16 | pfw[ex_addr - 1] << 8 | pfw[ex_addr];
		ipio_debug("data crc = %x, hex crc = %x\n", data_crc, hex_crc);
		if (data_crc != hex_crc) {
			ipio_err("Content of hex file is broken. (%d, %x, %x)\n",
				i, data_crc, hex_crc);
			return -1;
		}
	}

	ipio_info("Contect of hex file is correct\n");
	tfd_V2.start_addr = start_addr;
	tfd_V2.end_addr = end_addr;
	tfd_V2.block_number = block;
	return 0;
}

static int ilitek_tdd_fw_hex_open(u8 open_file_method, u8 *pfw)
{
	int fsize = 0;
	struct firmware fw_download;
	u8 *hex_buffer = NULL;

    
	switch (open_file_method) {
	case REQUEST_FIRMWARE:
		ipio_info("Request firmware path = %s\n", UPDATE_REQUEST_FW_PATH);
        fw_download.data = vts_fw_data_get(idev_V2->vtsdev, VTS_FW_TYPE_FW, &fsize);
		idev_V2->vivo_fw_size = fsize;
		if (fw_download.data == NULL) {
			VTE("firmware get fail! ");
			return -EINVAL;
		}
		memcpy(idev_V2->vivo_fw_data, fw_download.data, idev_V2->vivo_fw_size);

		ipio_info("fsize = %d\n", fsize);
		if (fsize <= 0) {
			ipio_err("The size of file is zero\n");
			return -ENOMEM;
		}

		hex_buffer = vmalloc(fsize * sizeof(u8));
		if (ERR_ALLOC_MEM(hex_buffer)) {
			ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
			return -ENOMEM;
		}

		ipio_memcpy(hex_buffer, idev_V2->vivo_fw_data, fsize * sizeof(u8), fsize);
		break;
	case FILP_OPEN:
		#if 0
		ipio_info("File open path = %s\n", UPDATE_FW_PATH);
		f = filp_open(UPDATE_FW_PATH, O_RDONLY, 0644);
		if (ERR_ALLOC_MEM(f)) {
			ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
			return -ENOMEM;
		}

		fsize = f->f_inode->i_size;
		ipio_info("fsize = %d\n", fsize);
		if (fsize <= 0) {
			ipio_err("The size of file is invaild\n");
			filp_close(f, NULL);
			return -ENOMEM;
		}

		hex_buffer = vmalloc(fsize * sizeof(u8));
		if (ERR_ALLOC_MEM(hex_buffer)) {
			ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
			filp_close(f, NULL);
			return -ENOMEM;
		}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		/* ready to map user's memory to obtain data by reading files */
		old_fs = get_fs();
	//	set_fs(get_ds());
		set_fs(KERNEL_DS);
		pos = 0;
		vfs_read(f, hex_buffer, fsize, &pos);
		set_fs(old_fs);
#else
		/* ready to map user's memory to obtain data by reading files */
		pos = 0;
		//kernel_read(f, hex_buffer, fsize, &pos);
#endif
		filp_close(f, NULL);
	#endif
		break;
	default:
		ipio_err("Unknown open file method, %d\n", open_file_method);
		break;
	}

	/* Convert hex and copy data from hex_buffer to pfw */
	if (ilitek_tddi_fw_hex_convert(hex_buffer, fsize, pfw) < 0) {
		ipio_err("Convert hex file failed\n");
		ipio_vfree((void **)&hex_buffer);
		return -1;
	}
	ipio_vfree((void **)&hex_buffer);
	return 0;
}

static void ilitek_tddi_fw_check_update(int ret)
{
	ipio_info("FW upgrade %s\n", (ret == UPDATE_PASS ? "PASS" : "FAIL"));

	if (ret == UPDATE_FAIL) {
		if (atomic_read(&idev_V2->mp_stat)) {
			ipio_info("No need to erase data during mp test\n");
			return;
		}
		ipio_info("Erase all fw data\n");
		if (idev_V2->fw_upgrade_mode == UPGRADE_IRAM) {
			ilitek_tddi_reset_ctrl_V2(idev_V2->reset);
		} else {
			ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
			ilitek_tddi_fw_flash_erase();
			ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
			ilitek_tddi_reset_ctrl_V2(idev_V2->reset);
		}
		return;
	}
}

int ilitek_tddi_fw_upgrade_V2(int upgrade_type, int file_type, int open_file_method)
{
	int ret = 0, retry = 3;
	u8 *pfw = NULL;
	int fwSize = 0;

	pfw = vmalloc(MAX_HEX_FILE_SIZE * sizeof(u8));
	if (ERR_ALLOC_MEM(pfw)) {
		ipio_err("Failed to allocate pfw memory, %ld\n", PTR_ERR(pfw));
		ret = -ENOMEM;
		goto out;
	}
	memset(pfw, 0xFF, MAX_HEX_FILE_SIZE * sizeof(u8));

	ipio_info("Convert FW file from %s\n", (file_type == ILI_FILE ? "ILI_FILE" : "HEX_FILE"));

	 // ret = request_firmware(fw_buffer, "PD1928--FW--LCMID--.bin", &idev_V2->spi->dev);
	 // ret = request_firmware(fw_buffer, UPDATE_REQUEST_FW_PATH, idev_V2->dev);
	 ipio_info("fwSize = %d\n", fwSize);
	  //CTPM_FW = vmalloc(fwSize/*, GFP_KERNEL*/)                                   
	//ilitek_dump_data_V2(CTPM_FW, 8, fwSize, 0, "vivotest");
	if (idev_V2->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		if (ilitek_tdd_fw_hex_open(open_file_method, pfw) < 0) {
			ipio_err("Open firmware file fail, return \n");
			ret = -1;
			goto out;
			
		}
		ilitek_tddi_fw_update_block_info(pfw, upgrade_type);
	}
	
	ilitek_plat_irq_disable_V2();

	do {
		if (upgrade_type == UPGRADE_FLASH)
			ret = ilitek_tddi_fw_flash_upgrade(pfw);
		else
			ret = ilitek_tddi_fw_iram_upgrade(pfw);

		if (ret == UPDATE_PASS)
			break;

		ipio_err("Upgrade failed, do retry!\n");
	} while (--retry > 0);

	if (idev_V2->actual_tp_mode != P5_X_FW_GESTURE_MODE) {

		idev_V2->chip->fw_ver	 = (idev_V2->fw_info[48] << 24) | (idev_V2->fw_info[49] << 16) |
					(idev_V2->fw_info[50] << 8) | (idev_V2->fw_info[51]);
		idev_V2->chip->fw_mp_ver = idev_V2->fw_mp_ver[0] << 24 | idev_V2->fw_mp_ver[1] << 16 | idev_V2->fw_mp_ver[2] << 8 | idev_V2->fw_mp_ver[3];
		ipio_info("Firmware version = %x\n", idev_V2->chip->fw_ver);
		ipio_info("MP version = %x\n", idev_V2->chip->fw_mp_ver);
		
		idev_V2->chip->core_ver = (idev_V2->fw_info[68] << 24) | (idev_V2->fw_info[69] << 16) |
					(idev_V2->fw_info[70] << 8) | (idev_V2->fw_info[71]);
		ipio_info("Core version = %x\n", idev_V2->chip->core_ver);
		
		idev_V2->protocol->ver  = (idev_V2->fw_info[72] << 16) | (idev_V2->fw_info[73] << 8) |
					idev_V2->fw_info[74];
		ipio_info("Protocol version = %x\n", idev_V2->protocol->ver);
		
		idev_V2->min_x = idev_V2->fw_info[5];
		idev_V2->min_y = idev_V2->fw_info[7];
		idev_V2->max_x = idev_V2->fw_info[9] << 8 | idev_V2->fw_info[8];
		idev_V2->max_y = idev_V2->fw_info[11] << 8 | idev_V2->fw_info[10];
		idev_V2->xch_num = idev_V2->fw_info[12];
		idev_V2->ych_num = idev_V2->fw_info[14];
		idev_V2->stx = idev_V2->xch_num;
		idev_V2->srx = idev_V2->ych_num;
		ipio_info("TP Info: min_x = %d, min_y = %d, max_x = %d, max_y = %d\n", idev_V2->min_x, idev_V2->min_y, idev_V2->max_x, idev_V2->max_y);
		ipio_info("TP Info: xch = %d, ych = %d, stx = %d, srx = %d\n", idev_V2->xch_num, idev_V2->ych_num, idev_V2->stx, idev_V2->srx);
		
		idev_V2->panel_wid = idev_V2->fw_info[17] << 8 | idev_V2->fw_info[16];
		idev_V2->panel_hei = idev_V2->fw_info[19] << 8 | idev_V2->fw_info[18];
		idev_V2->trans_xy = idev_V2->fw_info[0];
		ipio_info("Panel info: width = %d, height = %d\n", idev_V2->panel_wid, idev_V2->panel_hei);
		ipio_info("Transfer touch coordinate = %s\n", idev_V2->trans_xy ? "ON" : "OFF");

	}
	if (ret != UPDATE_PASS) {
		ipio_err("Upgrade firmware failed after retry 3 times\n");
		ret = UPDATE_FAIL;
	}

out:
	ilitek_tddi_fw_check_update(ret);	
	ilitek_plat_irq_enable_V2();
	ipio_vfree((void **)&pfw);
	return ret;
}

struct flash_table {
	u16 mid;
	u16 dev_id;
	int mem_size;
	int program_page;
	int sector;
} flashtab_V2[] = {
	[0] = {0x00, 0x0000, (256 * K), 256, (4 * K)}, /* Default */
	[1] = {0xEF, 0x6011, (128 * K), 256, (4 * K)}, /* W25Q10EW	*/
	[2] = {0xEF, 0x6012, (256 * K), 256, (4 * K)}, /* W25Q20EW	*/
	[3] = {0xC8, 0x6012, (256 * K), 256, (4 * K)}, /* GD25LQ20B */
	[4] = {0xC8, 0x6013, (512 * K), 256, (4 * K)}, /* GD25LQ40 */
	[5] = {0x85, 0x6013, (4 * M), 256, (4 * K)},
	[6] = {0xC2, 0x2812, (256 * K), 256, (4 * K)},
	[7] = {0x1C, 0x3812, (256 * K), 256, (4 * K)},
};

void ilitek_tddi_fw_read_flash_info_V2(bool mode)
{
	int i = 0;
	u8 buf[4] = {0};
	u8 cmd = 0x9F;
	u32 tmp = 0;
	u16 flash_id = 0, flash_mid = 0;
	bool ice = atomic_read(&idev_V2->ice_stat);

	if (mode == UPGRADE_IRAM)
		return;

	if (!ice)
		ilitek_ice_mode_ctrl_V2(ENABLE, OFF);

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write_V2(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write_V2(FLASH2_ADDR, cmd, 1);

	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		ilitek_ice_mode_write_V2(FLASH2_ADDR, 0xFF, 1);
		if (ilitek_ice_mode_read_V2(FLASH4_ADDR, &tmp, sizeof(u8)) < 0)
			ipio_err("Read flash info error\n");
		buf[i] = tmp;
	}

	ilitek_ice_mode_write_V2(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];

	for (i = 0; i < ARRAY_SIZE(flashtab_V2); i++) {
		if (flash_mid == flashtab_V2[i].mid && flash_id == flashtab_V2[i].dev_id) {
			idev_V2->flash_mid = flashtab_V2[i].mid;
			idev_V2->flash_devid = flashtab_V2[i].dev_id;
			idev_V2->program_page = flashtab_V2[i].program_page;
			idev_V2->flash_sector = flashtab_V2[i].sector;
			break;
		}
	}

	if (i >= ARRAY_SIZE(flashtab_V2)) {
		ipio_info("Not found flash id in tab, use default\n");
		idev_V2->flash_mid = flashtab_V2[0].mid;
		idev_V2->flash_devid = flashtab_V2[0].dev_id;
		idev_V2->program_page = flashtab_V2[0].program_page;
		idev_V2->flash_sector = flashtab_V2[0].sector;
	}

	ipio_info("Flash MID = %x, Flash DEV_ID = %x\n", idev_V2->flash_mid, idev_V2->flash_devid);
	ipio_info("Flash program page = %d\n", idev_V2->program_page);
	ipio_info("Flash sector = %d\n", idev_V2->flash_sector);

	ilitek_tddi_flash_protect(DISABLE);

	if (!ice)
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
}
