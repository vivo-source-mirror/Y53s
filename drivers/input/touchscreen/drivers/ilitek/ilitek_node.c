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
#include <linux/miscdevice.h>

#define USER_STR_BUFF		PAGE_SIZE
#define IOCTL_I2C_BUFF		PAGE_SIZE
#define ILITEK_IOCTL_MAGIC	100
#define ILITEK_IOCTL_MAXNR	27

#define ILITEK_IOCTL_I2C_WRITE_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 0, u8*)
#define ILITEK_IOCTL_I2C_SET_WRITE_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 2, u8*)
#define ILITEK_IOCTL_I2C_SET_READ_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 3, int)

#define ILITEK_IOCTL_TP_HW_RESET		_IOWR(ILITEK_IOCTL_MAGIC, 4, int)
#define ILITEK_IOCTL_TP_POWER_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 5, int)
#define ILITEK_IOCTL_TP_REPORT_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 6, int)
#define ILITEK_IOCTL_TP_IRQ_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 7, int)

#define ILITEK_IOCTL_TP_DEBUG_LEVEL		_IOWR(ILITEK_IOCTL_MAGIC, 8, int)
#define ILITEK_IOCTL_TP_FUNC_MODE		_IOWR(ILITEK_IOCTL_MAGIC, 9, int)

#define ILITEK_IOCTL_TP_FW_VER			_IOWR(ILITEK_IOCTL_MAGIC, 10, u8*)
#define ILITEK_IOCTL_TP_PL_VER			_IOWR(ILITEK_IOCTL_MAGIC, 11, u8*)
#define ILITEK_IOCTL_TP_CORE_VER		_IOWR(ILITEK_IOCTL_MAGIC, 12, u8*)
#define ILITEK_IOCTL_TP_DRV_VER			_IOWR(ILITEK_IOCTL_MAGIC, 13, u8*)
#define ILITEK_IOCTL_TP_CHIP_ID			_IOWR(ILITEK_IOCTL_MAGIC, 14, u32*)

#define ILITEK_IOCTL_TP_NETLINK_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 15, int*)
#define ILITEK_IOCTL_TP_NETLINK_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 16, int*)

#define ILITEK_IOCTL_TP_MODE_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 17, u8*)
#define ILITEK_IOCTL_TP_MODE_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 18, int*)
#define ILITEK_IOCTL_ICE_MODE_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 19, int)

#define ILITEK_IOCTL_TP_INTERFACE_TYPE		_IOWR(ILITEK_IOCTL_MAGIC, 20, u8*)
#define ILITEK_IOCTL_TP_DUMP_FLASH		_IOWR(ILITEK_IOCTL_MAGIC, 21, int)
#define ILITEK_IOCTL_TP_FW_UART_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 22, u8*)
#define ILITEK_IOCTL_TP_PANEL_INFO		_IOWR(ILITEK_IOCTL_MAGIC, 23, u32*)
#define ILITEK_IOCTL_TP_INFO			_IOWR(ILITEK_IOCTL_MAGIC, 24, u32*)
#define ILITEK_IOCTL_WRAPPER_RW 		_IOWR(ILITEK_IOCTL_MAGIC, 25, u8*)
#define ILITEK_IOCTL_DDI_WRITE	 		_IOWR(ILITEK_IOCTL_MAGIC, 26, u8*)
#define ILITEK_IOCTL_DDI_READ	 		_IOWR(ILITEK_IOCTL_MAGIC, 27, u8*)


#ifdef CONFIG_COMPAT
#define ILITEK_COMPAT_IOCTL_I2C_WRITE_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 0, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_I2C_SET_WRITE_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 1, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_I2C_READ_DATA		_IOWR(ILITEK_IOCTL_MAGIC, 2, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_I2C_SET_READ_LENGTH		_IOWR(ILITEK_IOCTL_MAGIC, 3, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_HW_RESET			_IOWR(ILITEK_IOCTL_MAGIC, 4, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_POWER_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 5, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_REPORT_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 6, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_IRQ_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 7, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_DEBUG_LEVEL		_IOWR(ILITEK_IOCTL_MAGIC, 8, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_FUNC_MODE		_IOWR(ILITEK_IOCTL_MAGIC, 9, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_FW_VER			_IOWR(ILITEK_IOCTL_MAGIC, 10, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_PL_VER			_IOWR(ILITEK_IOCTL_MAGIC, 11, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_CORE_VER			_IOWR(ILITEK_IOCTL_MAGIC, 12, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_DRV_VER			_IOWR(ILITEK_IOCTL_MAGIC, 13, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_CHIP_ID			_IOWR(ILITEK_IOCTL_MAGIC, 14, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_NETLINK_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 15, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_NETLINK_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 16, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_MODE_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 17, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_MODE_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 18, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_ICE_MODE_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 19, compat_uptr_t)

#define ILITEK_COMPAT_IOCTL_TP_INTERFACE_TYPE		_IOWR(ILITEK_IOCTL_MAGIC, 20, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_DUMP_FLASH		_IOWR(ILITEK_IOCTL_MAGIC, 21, compat_uptr_t)
#define ILITEK_COMPAT_IOCTL_TP_FW_UART_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 22, compat_uptr_t)
#endif

unsigned char g_user_buf_V2[USER_STR_BUFF] = {0};

int str2hex_V2(char *str)
{
	int strlen, result, intermed, intermedtop;
	char *s = str;

	while (*s != 0x0) {
		s++;
	}

	strlen = (int)(s - str);
	s = str;
	if (*s != 0x30) {
		return -1;
	}

	s++;

	if (*s != 0x78 && *s != 0x58) {
		return -1;
	}
	s++;

	strlen = strlen - 3;
	result = 0;
	while (*s != 0x0) {
		intermed = *s & 0x0f;
		intermedtop = *s & 0xf0;
		if (intermedtop == 0x60 || intermedtop == 0x40) {
			intermed += 0x09;
		}
		intermed = intermed << (strlen << 2);
		result = result | intermed;
		strlen -= 1;
		s++;
	}
	return result;
}

int katoi_V2(char *str)
{
	int result = 0;
	unsigned int digit;
	int sign;

	if (*str == '-') {
		sign = 1;
		str += 1;
	} else {
		sign = 0;
		if (*str == '+') {
			str += 1;
		}
	}

	for (;; str += 1) {
		digit = *str - '0';
		if (digit > 9)
			break;
		result = (10 * result) + digit;
	}

	if (sign) {
		return -result;
	}
	return result;
}
#if 0
struct file_buffer {
	char *ptr;
	char file_name[128];
	int32_t file_len;
	int32_t file_max_zise;
};

static int file_write(struct file_buffer *file, bool new_open)
{
	struct file *f = NULL;
	mm_segment_t fs;
	loff_t pos;

	if (file->ptr == NULL) {
		ipio_err("str is invaild\n");
		return -1;
	}

	if (file->file_name == NULL) {
		ipio_err("file name is invaild\n");
		return -1;
	}

	if (file->file_len >= file->file_max_zise) {
		ipio_err("The length saved to file is too long !\n");
		return -1;
	}

	if (new_open)
		f = filp_open(file->file_name, O_WRONLY | O_CREAT | O_TRUNC, 644);
	else
		f = filp_open(file->file_name, O_WRONLY | O_CREAT | O_APPEND, 644);

	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s file\n", file->file_name);
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(f, file->ptr, file->file_len, &pos);
	set_fs(fs);
	filp_close(f, NULL);
	return 0;
}

static int debug_mode_get_data(struct file_buffer *file, u8 type, u32 frame_count)
{
	int ret;
	int timeout = 50;
	u8 cmd[2] = { 0 }, row, col;
	s16 temp;
	unsigned char *ptr;
	int j;
	u16 write_index = 0;

	idev_V2->debug_node_open = false;
	idev_V2->debug_data_frame = 0;
	row = idev_V2->ych_num;
	col = idev_V2->xch_num;

	mutex_lock(&idev_V2->touch_mutex);
	cmd[0] = 0xFA;
	cmd[1] = type;
	ret = idev_V2->write(cmd, 2);
	idev_V2->debug_node_open = true;
	mutex_unlock(&idev_V2->touch_mutex);
	if (ret < 0)
		return ret;

	while ((write_index < frame_count) && (timeout > 0)) {
		ipio_info("frame = %d,index = %d,count = %d\n", write_index, write_index % 1024, idev_V2->debug_data_frame);
		if ((write_index % 1024) < idev_V2->debug_data_frame) {
			mutex_lock(&idev_V2->touch_mutex);
			file->file_len = 0;
			memset(file->ptr, 0, file->file_max_zise);
			file->file_len += sprintf(file->ptr + file->file_len, "\n\nFrame%d,", write_index);
			for (j = 0; j < col; j++)
				file->file_len += sprintf(file->ptr + file->file_len, "[X%d] ,", j);
			ptr = &idev_V2->debug_buf[write_index%1024][35];
			for (j = 0; j < row * col; j++, ptr += 2) {
				temp = (*ptr << 8) + *(ptr + 1);
				if (j % col == 0)
					file->file_len += sprintf(file->ptr + file->file_len, "\n[Y%d] ,", (j / col));
				file->file_len += sprintf(file->ptr + file->file_len, "%d, ", temp);
			}
			file->file_len += sprintf(file->ptr + file->file_len, "\n[X] ,");
			for (j = 0; j < row + col; j++, ptr += 2) {
				temp = (*ptr << 8) + *(ptr + 1);
				if (j == col)
					file->file_len += sprintf(file->ptr + file->file_len, "\n[Y] ,");
				file->file_len += sprintf(file->ptr + file->file_len, "%d, ", temp);
			}
			file_write(file, false);
			write_index++;
			mutex_unlock(&idev_V2->touch_mutex);
			timeout = 50;
		}

		if (write_index % 1024 == 0 && idev_V2->debug_data_frame == 1024)
			idev_V2->debug_data_frame = 0;

		mdelay(100);/*get one frame data taken around 130ms*/
		timeout--;
		if (timeout == 0)
			ipio_err("debug_V2 mode get data timeout!\n");
	}
	idev_V2->debug_node_open = false;
	return 0;
}
#endif
#if 0
static ssize_t ilitek_proc_get_delta_data_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
{
	s16 *delta = NULL;
	int row = 0, col = 0,  index = 0;
	int ret, i, x, y;
	int read_length = 0;
	u8 cmd[2] = {0};
	u8 *data = NULL;

	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);
	mutex_lock(&idev_V2->touch_mutex);

	row = idev_V2->ych_num;
	col = idev_V2->xch_num;
	read_length = 4 + 2 * row * col + 1 ;

	ipio_info("read length = %d\n", read_length);

	data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(data)) {
		ipio_err("Failed to allocate data mem\n");
		return 0;
	}

	delta = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
	if (ERR_ALLOC_MEM(delta)) {
		ipio_err("Failed to allocate delta mem\n");
		return 0;
	}

	cmd[0] = 0xB7;
	cmd[1] = 0x1; /*get delta*/
	ret = idev_V2->write(cmd, sizeof(cmd));
	if (ret < 0) {
		ipio_err("Failed to write 0xB7,0x1 command, %d\n", ret);
		goto out;
	}

	msleep(20);

	/* read debug_V2 packet header */
	ret = idev_V2->read(data, read_length);

	cmd[1] = 0x03; /*switch to normal mode*/
	ret = idev_V2->write(cmd, sizeof(cmd));
	if (ret < 0) {
		ipio_err("Failed to write 0xB7,0x3 command, %d\n", ret);
		goto out;
	}

	for (i = 4, index = 0; index < row * col * 2; i += 2, index++)
		delta[index] = (data[i] << 8) + data[i + 1];

	size = snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "======== Deltadata ========\n");
	ipio_info("======== Deltadata ========\n");

	size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size,
		"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
	ipio_info("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);

	/* print delta data*/
	for (y = 0; y < row; y++) {
		size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "[%2d] ", (y+1));
		ipio_info("[%2d] ", (y+1));

		for (x = 0; x < col; x++) {
			int shift = y * col + x;
			size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "%5d", delta[shift]);
			printk(KERN_CONT "%5d", delta[shift]);
		}
		size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "\n");
		printk(KERN_CONT "\n");
	}

	if (copy_to_user(buf, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space");

	*pos += size;

out:
	mutex_unlock(&idev_V2->touch_mutex);
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);
	ipio_kfree((void **)&data);
	ipio_kfree((void **)&delta);
	return size;
}

static ssize_t ilitek_proc_fw_get_raw_data_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
{
	s16 *rawdata = NULL;
	int row = 0, col = 0,  index = 0;
	int ret, i, x, y;
	int read_length = 0;
	u8 cmd[2] = {0};
	u8 *data = NULL;

	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);
	mutex_lock(&idev_V2->touch_mutex);

	row = idev_V2->ych_num;
	col = idev_V2->xch_num;
	read_length = 4 + 2 * row * col + 1 ;

	ipio_info("read length = %d\n", read_length);

	data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(data)) {
			ipio_err("Failed to allocate data mem\n");
			return 0;
	}

	rawdata = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
	if (ERR_ALLOC_MEM(rawdata)) {
			ipio_err("Failed to allocate rawdata mem\n");
			return 0;
	}

	cmd[0] = 0xB7;
	cmd[1] = 0x2; /*get rawdata*/
	ret = idev_V2->write(cmd, sizeof(cmd));
	if (ret < 0) {
		ipio_err("Failed to write 0xB7,0x2 command, %d\n", ret);
		goto out;
	}

	msleep(20);

	/* read debug_V2 packet header */
	ret = idev_V2->read(data, read_length);

	cmd[1] = 0x03; /*switch to normal mode*/
	ret = idev_V2->write(cmd, sizeof(cmd));
	if (ret < 0) {
		ipio_err("Failed to write 0xB7,0x3 command, %d\n", ret);
		goto out;
	}

	for (i = 4, index = 0; index < row * col * 2; i += 2, index++)
		rawdata[index] = (data[i] << 8) + data[i + 1];

	size = snprintf(g_user_buf_V2, PAGE_SIZE, "======== RawData ========\n");
	ipio_info("======== RawData ========\n");

	size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size,
			"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
	ipio_info("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);

	/* print raw data*/
	for (y = 0; y < row; y++) {
		size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "[%2d] ", (y+1));
		ipio_info("[%2d] ", (y+1));

		for (x = 0; x < col; x++) {
			int shift = y * col + x;
			size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "%5d", rawdata[shift]);
			printk(KERN_CONT "%5d", rawdata[shift]);
		}
		size += snprintf(g_user_buf_V2 + size, PAGE_SIZE - size, "\n");
		printk(KERN_CONT "\n");
	}

	if (copy_to_user(buf, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space");

	*pos += size;

out:
	mutex_unlock(&idev_V2->touch_mutex);
	ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);
	ipio_kfree((void **)&data);
	ipio_kfree((void **)&rawdata);
	return size;
}
#endif
static ssize_t ilitek_proc_fw_pc_counter_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
{
	u32 pc;

	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	pc = ilitek_tddi_ic_get_pc_counter_V2();
	size = snprintf(g_user_buf_V2, PAGE_SIZE, "pc counter = 0x%x\n", pc);
	if (copy_to_user(buf, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space");

	*pos += size;
	return size;
}

u32 rw_reg_V2[5] = {0};
static ssize_t ilitek_proc_rw_tp_reg_read(struct file *pFile, char __user *buf, size_t size, loff_t *pos)
{
	int ret = 0;
	bool mcu_on = 0, read = 0;
	u32 type, addr, read_data, write_data, write_len, stop_mcu;
	bool esd_en = idev_V2->wq_esd_ctrl, bat_en = idev_V2->wq_bat_ctrl;

	if (*pos != 0)
		return 0;

	stop_mcu = rw_reg_V2[0];
	type = rw_reg_V2[1];
	addr = rw_reg_V2[2];
	write_data = rw_reg_V2[3];
	write_len = rw_reg_V2[4];

	ipio_info("stop_mcu = %d\n", rw_reg_V2[0]);

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);

	mutex_lock(&idev_V2->touch_mutex);

	if (stop_mcu == mcu_on) {
		ret = ilitek_ice_mode_ctrl_V2(ENABLE, ON);
		if (ret < 0) {
			ipio_err("Failed to enter ICE mode, ret = %d\n", ret);
			mutex_unlock(&idev_V2->touch_mutex);
			return -1;
		}
	} else {
		ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
		if (ret < 0) {
			ipio_err("Failed to enter ICE mode, ret = %d\n", ret);
			mutex_unlock(&idev_V2->touch_mutex);
			return -1;
		}
	}

	if (type == read) {
		if (ilitek_ice_mode_read_V2(addr, &read_data, sizeof(u32)) < 0)
			ipio_err("Read data error\n");
		ipio_info("READ:addr = 0x%06x, read = 0x%08x\n", addr, read_data);
		size = snprintf(g_user_buf_V2, PAGE_SIZE, "READ:addr = 0x%06x, read = 0x%08x\n", addr, read_data);
	} else {
		ilitek_ice_mode_write_V2(addr, write_data, write_len);
		ipio_info("WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
		size = snprintf(g_user_buf_V2, PAGE_SIZE, "WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
	}

	if (stop_mcu == mcu_on)
		ilitek_ice_mode_ctrl_V2(DISABLE, ON);
	else
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);

	if (copy_to_user(buf, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space");

	*pos += size;
	mutex_unlock(&idev_V2->touch_mutex);

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);

	return size;
}

static ssize_t ilitek_proc_rw_tp_reg_write(struct file *filp, const char *buff, size_t size, loff_t *pos)
{
	char *token = NULL, *cur = NULL;
	char cmd[256] = { 0 };
	u32 count = 0;

	if ((size - 1) > sizeof(cmd) || size <= 0) {
		ipio_err("ERROR! input length is invalid\n");
		return -1;
	}
	if (buff) {
		if (copy_from_user(cmd, buff, size - 1)) {
			ipio_info("Failed to copy data from user space\n");
			return -1;
		}
	}
	token = cur = cmd;
	while ((token = strsep(&cur, ",")) != NULL) {
		rw_reg_V2[count] = str2hex_V2(token);
		ipio_info("rw_reg_V2[%d] = 0x%x\n", count, rw_reg_V2[count]);
		count++;
	}
	return size;
}

static ssize_t ilitek_proc_debug_switch_read(struct file *pFile, char __user *buff, size_t size, loff_t *pos)
{
	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	idev_V2->debug_node_open = !idev_V2->debug_node_open;

	ipio_info(" %s debug_flag message = %x\n", idev_V2->debug_node_open ? "Enabled" : "Disabled", idev_V2->debug_node_open);

	size = sprintf(g_user_buf_V2, "debug_node_open : %s\n", idev_V2->debug_node_open ? "Enabled" : "Disabled");

	*pos += size;

	if (copy_to_user(buff, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space");

	return size;
}

static ssize_t ilitek_proc_debug_message_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	unsigned long p = *pos;
	int i = 0;
	int send_data_len = 0;
	int ret = 0;
	int data_count = 0;
	int one_data_bytes = 0;
	int need_read_data_len = 0;
	int type = 0;
	unsigned char *tmpbuf = NULL;
	unsigned char tmpbufback[128] = {0};
	
	if(!idev_V2->debug_node_open){
		VTE("debug node not open");
		return 0;
	}

	mutex_lock(&idev_V2->debug_read_mutex);

	while (idev_V2->debug_data_frame <= 0) {
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		wait_event_interruptible(idev_V2->inq, idev_V2->debug_data_frame > 0);
	}

	mutex_lock(&idev_V2->debug_mutex);

	tmpbuf = vmalloc(4096);	/* buf size if even */
	if (ERR_ALLOC_MEM(tmpbuf)) {
		ipio_err("buffer vmalloc error\n");
		send_data_len += sprintf(tmpbufback + send_data_len, "buffer vmalloc error\n");
		ret = copy_to_user(buff, tmpbufback, send_data_len); /*idev_V2->debug_buf[0] */
		goto out;
	}

	if (idev_V2->debug_data_frame > 0) {
		if (idev_V2->debug_buf[0][0] == P5_X_DEMO_PACKET_ID) {
			need_read_data_len = 43;
		} else if (idev_V2->debug_buf[0][0] == P5_X_I2CUART_PACKET_ID) {
			type = idev_V2->debug_buf[0][3] & 0x0F;

			data_count = idev_V2->debug_buf[0][1] * idev_V2->debug_buf[0][2];

			if (type == 0 || type == 1 || type == 6) {
				one_data_bytes = 1;
			} else if (type == 2 || type == 3) {
				one_data_bytes = 2;
			} else if (type == 4 || type == 5) {
				one_data_bytes = 4;
			}
			need_read_data_len = data_count * one_data_bytes + 1 + 5;
		} else if (idev_V2->debug_buf[0][0] == P5_X_DEBUG_PACKET_ID) {
			send_data_len = 0;	/* idev_V2->debug_buf[0][1] - 2; */
			need_read_data_len = 2040;
		}

		for (i = 0; i < need_read_data_len; i++) {
			send_data_len += sprintf(tmpbuf + send_data_len, "%02X", idev_V2->debug_buf[0][i]);
			if (send_data_len >= 4096) {
				ipio_err("send_data_len = %d set 4096 i = %d\n", send_data_len, i);
				send_data_len = 4096;
				break;
			}
		}

		send_data_len += sprintf(tmpbuf + send_data_len, "\n\n");

		if (p == 5 || size == 4096 || size == 2048) {
			idev_V2->debug_data_frame--;

			if (idev_V2->debug_data_frame < 0)
				idev_V2->debug_data_frame = 0;

			for (i = 1; i <= idev_V2->debug_data_frame; i++)
				memcpy(idev_V2->debug_buf[i - 1], idev_V2->debug_buf[i], 2048);
		}

	} else {
		ipio_err("no data send\n");
		send_data_len += sprintf(tmpbuf + send_data_len, "no data send\n");
	}

	/* Preparing to send debug_V2 data to user */
	if (size == 4096)
		ret = copy_to_user(buff, tmpbuf, send_data_len);
	else
		ret = copy_to_user(buff, tmpbuf + p, send_data_len - p);

	/* ipio_err("send_data_len = %d\n", send_data_len); */
	if (send_data_len <= 0 || send_data_len > 4096) {
		ipio_err("send_data_len = %d set 4096\n", send_data_len);
		send_data_len = 4096;
	}

	if (ret) {
		ipio_err("copy_to_user err\n");
		ret = -EFAULT;
	} else {
		*pos += send_data_len;
		ret = send_data_len;
		ipio_debug("Read %d bytes(s) from %ld\n", send_data_len, p);
	}

out:
	mutex_unlock(&idev_V2->debug_mutex);
	mutex_unlock(&idev_V2->debug_read_mutex);
	ipio_vfree((void **)&tmpbuf);
	return send_data_len;
}

#if 0
static ssize_t ilitek_proc_get_debug_mode_data_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	int ret;
	u8 tp_mode;
	struct file_buffer csv;

	if (*pos != 0)
		return 0;

	/* initialize file */
	memset(csv.file_name, 0, sizeof(csv.file_name));
	sprintf(csv.file_name, "%s", DEBUG_DATA_FILE_PATH);
	csv.file_len = 0;
	csv.file_max_zise = DEBUG_DATA_FILE_SIZE;
	csv.ptr = vmalloc(csv.file_max_zise);

	if (ERR_ALLOC_MEM(csv.ptr)) {
		ipio_err("Failed to allocate CSV mem\n");
		goto out;
	}

	/* save data to csv */
	ipio_info("Get Raw data %d frame\n", idev_V2->raw_count);
	ipio_info("Get Delta data %d frame\n", idev_V2->delta_count);
	csv.file_len += sprintf(csv.ptr + csv.file_len, "Get Raw data %d frame\n", idev_V2->raw_count);
	csv.file_len += sprintf(csv.ptr + csv.file_len, "Get Delta data %d frame\n", idev_V2->delta_count);
	file_write(&csv, true);

	/* change to debug_V2 mode */
	tp_mode = P5_X_FW_DEBUG_MODE;
	ret = ilitek_tddi_switch_mode_V2(&tp_mode);
	if (ret < 0)
		goto out;

	/* get raw data */
	csv.file_len = 0;
	memset(csv.ptr, 0, csv.file_max_zise);
	csv.file_len += sprintf(csv.ptr + csv.file_len, "\n\n=======Raw data=======");
	file_write(&csv, false);
	ret = debug_mode_get_data(&csv, P5_X_FW_RAW_DATA_MODE, idev_V2->raw_count);
	if (ret < 0)
		goto out;

	/* get delta data */
	csv.file_len = 0;
	memset(csv.ptr, 0, csv.file_max_zise);
	csv.file_len += sprintf(csv.ptr + csv.file_len, "\n\n=======Delta data=======");
	file_write(&csv, false);
	ret = debug_mode_get_data(&csv, P5_X_FW_DELTA_DATA_MODE, idev_V2->delta_count);
	if (ret < 0)
		goto out;

	/* change to demo mode */
	tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_switch_mode_V2(&tp_mode);

out:
	ipio_vfree((void **)&csv.ptr);
	return 0;
}

static ssize_t ilitek_proc_get_debug_mode_data_write(struct file *filp, const char *buff, size_t size, loff_t *pos)
{
	char *token = NULL, *cur = NULL;
	char cmd[256] = {0};
	u8 temp[256] = {0}, count = 0;

	if ((size - 1) > sizeof(cmd) || size <= 0) {
		ipio_err("ERROR! input length is invalid\n");
		return -1;
	}

	if (buff) {
		if (copy_from_user(cmd, buff, size - 1)) {
			ipio_err("Failed to copy data from user space\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);
	token = cur = cmd;
	while ((token = strsep(&cur, ",")) != NULL) {
		temp[count] = str2hex_V2(token);
		ipio_info("temp[%d] = %d\n", count, temp[count]);
		count++;
	}

	idev_V2->raw_count = ((temp[0] << 8) | temp[1]);
	idev_V2->delta_count = ((temp[2] << 8) | temp[3]);
	idev_V2->bg_count = ((temp[4] << 8) | temp[5]);

	ipio_info("Raw_count = %d, Delta_count = %d, BG_count = %d\n", idev_V2->raw_count, idev_V2->delta_count, idev_V2->bg_count);
	return size;
}
#endif
static ssize_t ilitek_proc_fw_process_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	u32 len = 0;

	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	len = sprintf(g_user_buf_V2, "%02d\n", idev_V2->fw_update_stat);

	ipio_info("update status = %d\n", idev_V2->fw_update_stat);

	if (copy_to_user((char *) buff, &idev_V2->fw_update_stat, len))
		ipio_err("Failed to copy data to user space\n");

	*pos = len;
	return len;
}

static ssize_t ilitek_node_fw_upgrade_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	u32 len = 0;
	int ret = 0;
	bool esd_en = idev_V2->wq_esd_ctrl, bat_en = idev_V2->wq_bat_ctrl;

	ipio_info("Preparing to upgarde firmware\n");

	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);

	mutex_lock(&idev_V2->touch_mutex);

	idev_V2->force_fw_update = ENABLE;
	idev_V2->fw_open = 1;
	ret = ilitek_tddi_fw_upgrade_handler_V2(NULL);
	idev_V2->fw_open = 0;	
	len = sprintf(g_user_buf_V2, "upgrade firwmare %s\n", (ret != 0) ? "failed" : "succeed");
	idev_V2->force_fw_update = DISABLE;
	if (copy_to_user((u32 *) buff, g_user_buf_V2, len))
		ipio_err("Failed to copy data to user space\n");

	mutex_unlock(&idev_V2->touch_mutex);

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);

	return 0;
}

static ssize_t ilitek_proc_debug_level_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	if (*pos != 0)
		return 0;

	memset(g_user_buf_V2, 0, USER_STR_BUFF * sizeof(unsigned char));

	ipio_debug_level_V2 = !ipio_debug_level_V2;

	ipio_info(" %s debug_V2 level = %x\n", ipio_debug_level_V2 ? "Enable" : "Disable", ipio_debug_level_V2);

	size = sprintf(g_user_buf_V2, "debug_V2 level : %s\n", ipio_debug_level_V2 ? "Enable" : "Disable");

	*pos += size;

	if (copy_to_user((u32 *) buff, g_user_buf_V2, size))
		ipio_err("Failed to copy data to user space\n");

	return size;
}

/* Firmware data with static array */
int debug_V2[32 * 18] = {0};
u16 gesture_V2[12] = {0};
static ssize_t ilitek_proc_vivo_write(struct file *filp, const char *buff, size_t size, loff_t *pos)
{
	int ret = 0, count = 0;
	char cmd[512] = {0};
	char *token = NULL, *cur = NULL;
	u32 *data = NULL;
	const struct firmware *fw = NULL;
	unsigned char *str = NULL;
	struct vts_device * vtsdev = idev_V2->vtsdev;

	if ((size - 1) > sizeof(cmd) || size <= 0) {
		ipio_err("ERROR! input length is invalid\n");
		return -1;
	}

	if (buff) {
		if (copy_from_user(cmd, buff, size - 1)) {
			ipio_info("Failed to copy data from user space\n");
			return -1;
		}
	}

    if (!vtsdev) {
		ipio_info("vtsdev is null\n");
		return -1;
	}
	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	token = cur = cmd;

	data = kcalloc(512, sizeof(u32), GFP_KERNEL);
	if (ERR_ALLOC_MEM(data)) {
		ipio_err("Failed to allocate data mem\n");
		ipio_kfree((void **)&data); //zhoucheng 20200428
		return -1;
	}
	while ((token = strsep(&cur, ",")) != NULL) {
		data[count] = str2hex_V2(token);
		ipio_info("data[%d] = %x\n", count, data[count]);
		count++;
	}

	ipio_info("cmd = %s\n", cmd);

	if (strcmp(cmd, "rawordiff") == 0) {
		memset(debug_V2, 0x0, 32 * 18 * sizeof(int));
		ret = bbk_ili_get_rawordiff_data_V2(vtsdev,data[1], (short *)debug_V2,32 * 18 * sizeof(int));
		ipio_info("Get RAW/Diff data %s\n", (ret < 0) ? "FAIL" : "DONE");
		ret = ipio_debug_level_V2;
		ipio_debug_level_V2 = DEBUG_ALL;
		ilitek_dump_data_V2(debug_V2, 8, 32 * 18, 0, "vivo debug_V2");
		ipio_debug_level_V2 = ret;
	} else if (strcmp(cmd, "fwupdate") == 0) {
		if (request_firmware(&fw, UPDATE_REQUEST_FW_PATH, idev_V2->dev) < 0) {
			ipio_err("Rquest firmware failed\n");
			goto out;
		}
mutex_lock(&idev_V2->touch_mutex);
		ret = bbk_ili_fw_update_V2(fw);
mutex_unlock(&idev_V2->touch_mutex);
		ipio_info("fw upgrade %s\n", (ret < 0) ? "FAIL" : "DONE");
		release_firmware(fw);
	} else if (strcmp(cmd, "idlectrl") == 0) {
		ret = iliIdleEnableOrDisable_V2(vtsdev,data[1]);
		ipio_info("set idle %s\n", (ret < 0) ? "Fail" : "DONE");
	} else if (strcmp(cmd, "readudd") == 0) {
		bbk_ili_readUdd_V2(NULL);
	} else if (strcmp(cmd, "writeudd") == 0) {
		bbk_ili_writeUdd_V2(NULL);
	} else if (strcmp(cmd, "modechange") == 0) {
		ret = bbk_ili_mode_change_V2(vtsdev,data[1]);
		ipio_info("switch mode %s\n", (ret < 0) ? "Fail" : "DONE");
	} else if (strcmp(cmd, "getfwver") == 0) {
		ret = bbk_ili_get_fw_version_V2(data[1]);
		ipio_info("get fw version %s\n", (ret < 0) ? "Fail" : "DONE");
	} else if (strcmp(cmd, "getgesture") == 0) {
		ret = bbk_ili_gesture_point_get_V2(gesture_V2);
		ipio_info("Get gesture_V2 len (%d), %s\n", ret, (ret < 0) ? "FAIL" : "DONE");
		ret = ipio_debug_level_V2;
		ipio_debug_level_V2 = DEBUG_ALL;
		ilitek_dump_data_V2(gesture_V2, 8, sizeof(gesture_V2), 0, "vivo gesture_V2 after");
		ipio_debug_level_V2 = ret;
	} else if (strcmp(cmd, "setcharger") == 0) {
		ret = bbk_ili_set_charger_bit_V2(vtsdev,data[1]);
		ipio_info("set charger %s\n", (ret < 0) ? "Fail" : "DONE");
	} else if (strcmp(cmd, "readcharger") == 0) {
		bbk_ili_read_charger_bit_V2();
	} else if (strcmp(cmd, "setedge") == 0) {
		ret = iliSetEdgeRestainSwitch_V2(vtsdev,data[1]);
		ipio_info("set edge palm %s\n", (ret < 0) ? "Fail" : "DONE");
	} else if (strcmp(cmd, "getfwdebuginfo") == 0) {
		str = kzalloc(PAGE_SIZE, GFP_KERNEL);
		if(!str){
			ret = bbk_ili_get_fw_debug_info_V2(str);
			ipio_info("return written length = %d\n", ret);
			ipio_info("%s", str);
			ipio_kfree((void **)&str);
		}
	} else {
		ipio_err("Unknown command\n");
	}
out:
	ipio_kfree((void **)&data);
	
	return size;
}

static ssize_t ilitek_node_ioctl_write(struct file *filp, const char *buff, size_t size, loff_t *pos)
{
	int i, count = 0;
	char cmd[512] = {0};
	char *token = NULL, *cur = NULL;
	u8 temp[256] = {0};
	u8 r_buf[256] = {0};
	u32 *data = NULL;
	u8 tp_mode;

	if ((size - 1) > sizeof(cmd) || size <= 0) {
		ipio_err("ERROR! input length is invalid\n");
		return -1;
	}

	if (buff) {
		if (copy_from_user(cmd, buff, size - 1)) {
			ipio_info("Failed to copy data from user space\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	token = cur = cmd;

	 //zhoucheng 20200428
	data = kcalloc(512, sizeof(u32), GFP_KERNEL);
	if (ERR_ALLOC_MEM(data)) {
		ipio_err("Failed to allocate data mem\n");
		return -1;
	}

	while ((token = strsep(&cur, ",")) != NULL) {
		data[count] = str2hex_V2(token);
		ipio_info("data[%d] = %x\n", count, data[count]);
		count++;
	}

	ipio_info("cmd = %s\n", cmd);

	mutex_lock(&idev_V2->touch_mutex);

	if (strcmp(cmd, "hwreset") == 0) {
		ilitek_tddi_reset_ctrl_V2(TP_HW_RST_ONLY);
	} else if (strcmp(cmd, "icwholereset") == 0) {
		ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
		ilitek_tddi_reset_ctrl_V2(TP_IC_WHOLE_RST);
	} else if (strcmp(cmd, "iccodereset") == 0) {
		ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
		ilitek_tddi_reset_ctrl_V2(TP_IC_CODE_RST);
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
	} else if (strcmp(cmd, "getinfo") == 0) {
		ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
		ilitek_tddi_ic_get_info_V2();
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
		ilitek_tddi_ic_get_protocl_ver_V2();
		ilitek_tddi_ic_get_fw_ver_V2();
		ilitek_tddi_ic_get_core_ver_V2();
		ilitek_tddi_ic_get_tp_info_V2();
		ilitek_tddi_ic_get_panel_info_V2();
		ipio_info("Driver version = %s\n", DRIVER_VERSION);
	} else if (strcmp(cmd, "enableicemode") == 0) {
		if (data[1] == ON)
			ilitek_ice_mode_ctrl_V2(ENABLE, ON);
		else
			ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	} else if (strcmp(cmd, "disableicemode") == 0) {
		ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
	} else if (strcmp(cmd, "enablewqesd") == 0) {
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	} else if (strcmp(cmd, "enablewqbat") == 0) {
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);
	} else if (strcmp(cmd, "disablewqesd") == 0) {
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	} else if (strcmp(cmd, "disablewqbat") == 0) {
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);
	} else if (strcmp(cmd, "gesture_V2") == 0) {
		idev_V2->gesture_V2 = !idev_V2->gesture_V2;
		ipio_info("gesture_V2 = %d\n", idev_V2->gesture_V2);
	} else if (strcmp(cmd, "esdgesture") == 0) {
		ilitek_tddi_wq_ctrl_V2(WQ_GES_RECOVER, ENABLE);
	} else if (strcmp(cmd, "iceflag") == 0) {
		if (data[1] == ENABLE)
			atomic_set(&idev_V2->ice_stat, ENABLE);
		else
			atomic_set(&idev_V2->ice_stat, DISABLE);
		ipio_info("ice mode flag = %d\n", atomic_read(&idev_V2->ice_stat));
	} else if (strcmp(cmd, "gesturenormal") == 0) {
		idev_V2->gesture_mode = P5_X_FW_GESTURE_NORMAL_MODE;
		ipio_info("gesture_V2 mode = %d\n", idev_V2->gesture_mode);
	} else if (strcmp(cmd, "gestureinfo") == 0) {
		idev_V2->gesture_mode = P5_X_FW_GESTURE_INFO_MODE;
		ipio_info("gesture_V2 mode = %d\n", idev_V2->gesture_mode);
	} else if (strcmp(cmd, "netlink") == 0) {
		idev_V2->netlink = !idev_V2->netlink;
		ipio_info("netlink flag= %d\n", idev_V2->netlink);
	} else if (strcmp(cmd, "switchtestmode") == 0) {
		tp_mode = P5_X_FW_TEST_MODE;
		ilitek_tddi_switch_mode_V2(&tp_mode);
	} else if (strcmp(cmd, "switchdebugmode") == 0) {
		tp_mode = P5_X_FW_DEBUG_MODE;
		ilitek_tddi_switch_mode_V2(&tp_mode);
	} else if (strcmp(cmd, "switchdemomode") == 0) {
		tp_mode = P5_X_FW_DEMO_MODE;
		ilitek_tddi_switch_mode_V2(&tp_mode);
	} else if (strcmp(cmd, "dbgflag") == 0) {
		idev_V2->debug_node_open = !idev_V2->debug_node_open;
		ipio_info("debug_V2 flag message = %d\n", idev_V2->debug_node_open);
	} else if (strcmp(cmd, "iow") == 0) {
		int w_len = 0;
		w_len = data[1];
		ipio_info("w_len = %d\n", w_len);

		for (i = 0; i < w_len; i++) {
			temp[i] = data[2 + i];
			ipio_info("write[%d] = %x\n", i, temp[i]);
		}

		idev_V2->write(temp, w_len);
	} else if (strcmp(cmd, "ior") == 0) {
		int r_len = 0;
		r_len = data[1];
		ipio_info("r_len = %d\n", r_len);
		idev_V2->read(temp, r_len);
		for (i = 0; i < r_len; i++)
			ipio_info("read[%d] = %x\n", i, temp[i]);
	} else if (strcmp(cmd, "iowr") == 0) {
		int delay = 0;
		int w_len = 0, r_len = 0;
		w_len = data[1];
		r_len = data[2];
		delay = data[3];
		ipio_info("w_len = %d, r_len = %d, delay = %d\n", w_len, r_len, delay);

		for (i = 0; i < w_len; i++) {
			temp[i] = data[4 + i];
			ipio_info("write[%d] = %x\n", i, temp[i]);
		}
		idev_V2->write(temp, w_len);
		memset(temp, 0, sizeof(temp));
		mdelay(delay);
		idev_V2->read(temp, r_len);

		for (i = 0; i < r_len; i++)
			ipio_info("read[%d] = %x\n", i, temp[i]);
	} else if (strcmp(cmd, "getddiregdata") == 0) {
		ipio_info("Get ddi reg one page: page = %x, reg = %x\n", data[1], data[2]);
		ilitek_tddi_ic_get_ddi_reg_onepage_V2(data[1], data[2]);
	} else if (strcmp(cmd, "setddiregdata") == 0) {
		ipio_info("Set ddi reg one page: page = %x, reg = %x, data = %x\n", data[1], data[2], data[3]);
		ilitek_tddi_ic_set_ddi_reg_onepage_V2(data[1], data[2], data[3]);
	} else if (strcmp(cmd, "dumpflashdata") == 0) {
		ipio_info("Start = 0x%x, End = 0x%x, Dump Hex path = %s\n", data[1], data[2], DUMP_FLASH_PATH);
		ilitek_tddi_fw_dump_flash_data_V2(data[1], data[2], false);
	} else if (strcmp(cmd, "dumpiramdata") == 0) {
		ipio_info("Start = 0x%x, End = 0x%x, Dump IRAM path = %s\n", data[1], data[2], DUMP_IRAM_PATH);
		ilitek_tddi_fw_dump_iram_data_V2(data[1], data[2]);
	} else if (strcmp(cmd, "edge_palm_ctrl") == 0) {
		ilitek_tddi_ic_func_ctrl_V2("edge_palm", data[1]);
	} else if (strcmp(cmd, "uart_mode_ctrl") == 0) {
		ilitek_tddi_fw_uart_ctrl_V2(data[1]);
	} else if (strcmp(cmd, "flashesdgesture") == 0) {

	} else if (strcmp(cmd, "spislavew") == 0) {
		int w_len = 0;
		w_len = data[1];
		ipio_info("spi slave w len = %d\n", w_len);

		for (i = 0; i < w_len; i++) {
			temp[i] = data[2 + i];
			ipio_info("write[%d] = %x\n", i, temp[i]);
		}

		core_spi_slave_mode_write(temp, w_len);
	} else if (strcmp(cmd, "spislaver") == 0) {
		int r_len = 0;
		r_len = data[1];
		ipio_info("spi slave r len = %d\n", r_len);
		core_spi_slave_mode_read(temp, r_len);
		for (i = 0; i < r_len; i++)
			ipio_info("read[%d] = %x\n", i, temp[i]);
	} else if (strcmp(cmd, "spislaverw") == 0) {
		int r_len = 0, w_len = 0;
		r_len = (data[1] << 8) | (data[2]);
		w_len = (data[3] << 8) | (data[4]);
		ipio_info("spi slave r len = %d, w len = %d\n", r_len, w_len);
		for (i = 0; i < w_len; i++) {
			temp[i] = data[5 + i];
			ipio_info("write[%d] = %x\n", i, temp[i]);
		}
		if (ilitek_thp_send_cmd(temp, w_len, r_buf, r_len) < 0)
			ipio_err("spi slave r len = %d\n", r_len);

		for (i = 0; i < r_len; i++)
			ipio_info("read[%d] = %x\n", i, r_buf[i]);


	} else {
		ipio_err("Unknown command\n");
	}

	ipio_kfree((void **)&data);
	mutex_unlock(&idev_V2->touch_mutex);
	return size;
}

#ifdef CONFIG_COMPAT
static long ilitek_node_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl) {
		ipio_err("There's no unlocked_ioctl defined in file\n");
		return -ENOTTY;
	}

	ipio_info("cmd = %d\n", _IOC_NR(cmd));

	switch (cmd) {
	case ILITEK_COMPAT_IOCTL_I2C_WRITE_DATA:
		ipio_info("compat_ioctl: convert i2c/spi write\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_I2C_WRITE_DATA, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_I2C_READ_DATA:
		ipio_info("compat_ioctl: convert i2c/spi read\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_I2C_READ_DATA, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_I2C_SET_WRITE_LENGTH:
		ipio_info("compat_ioctl: convert set write length\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_I2C_SET_WRITE_LENGTH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_I2C_SET_READ_LENGTH:
		ipio_info("compat_ioctl: convert set read length\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_I2C_SET_READ_LENGTH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_HW_RESET:
		ipio_info("compat_ioctl: convert hw reset\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_HW_RESET, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_POWER_SWITCH:
		ipio_info("compat_ioctl: convert power switch\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_POWER_SWITCH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_REPORT_SWITCH:
		ipio_info("compat_ioctl: convert report switch\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_REPORT_SWITCH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_IRQ_SWITCH:
		ipio_info("compat_ioctl: convert irq switch\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_IRQ_SWITCH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_DEBUG_LEVEL:
		ipio_info("compat_ioctl: convert debug_V2 level\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_DEBUG_LEVEL, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_FUNC_MODE:
		ipio_info("compat_ioctl: convert function mode\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_FUNC_MODE, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_FW_VER:
		ipio_info("compat_ioctl: convert set read length\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_FW_VER, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_PL_VER:
		ipio_info("compat_ioctl: convert fw version\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_PL_VER, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_CORE_VER:
		ipio_info("compat_ioctl: convert core version\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_CORE_VER, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_DRV_VER:
		ipio_info("compat_ioctl: convert driver version\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_DRV_VER, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_CHIP_ID:
		ipio_info("compat_ioctl: convert chip id\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_CHIP_ID, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_NETLINK_CTRL:
		ipio_info("compat_ioctl: convert netlink ctrl\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_NETLINK_CTRL, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_NETLINK_STATUS:
		ipio_info("compat_ioctl: convert netlink status\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_NETLINK_STATUS, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_MODE_CTRL:
		ipio_info("compat_ioctl: convert tp mode ctrl\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_MODE_CTRL, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_MODE_STATUS:
		ipio_info("compat_ioctl: convert tp mode status\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_MODE_STATUS, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_ICE_MODE_SWITCH:
		ipio_info("compat_ioctl: convert tp mode switch\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_ICE_MODE_SWITCH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_INTERFACE_TYPE:
		ipio_info("compat_ioctl: convert interface type\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_INTERFACE_TYPE, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_DUMP_FLASH:
		ipio_info("compat_ioctl: convert dump flash\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_DUMP_FLASH, (unsigned long)compat_ptr(arg));
		return ret;
	case ILITEK_COMPAT_IOCTL_TP_FW_UART_CTRL:
		ipio_info("compat_ioctl: convert fw uart\n");
		ret = filp->f_op->unlocked_ioctl(filp, ILITEK_IOCTL_TP_FW_UART_CTRL, (unsigned long)compat_ptr(arg));
		return ret;
	default:
		ipio_err("no ioctl cmd, return ilitek_node_ioctl\n");
		return -ENOIOCTLCMD;
	}
}
#endif

static long ilitek_node_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0, length = 0;
	u8 *szBuf = NULL, if_to_user = 0;
	static u16 i2c_rw_length;
	u32 id_to_user[64] = {0};
	bool esd_en = idev_V2->wq_esd_ctrl, bat_en = idev_V2->wq_bat_ctrl;
	u16 wrap_wlen = 0, wrap_rlen = 0, wrap_int = 0;
	u8 *wrap_rbuf = NULL;

	if (_IOC_TYPE(cmd) != ILITEK_IOCTL_MAGIC) {
		ipio_err("The Magic number doesn't match\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > ILITEK_IOCTL_MAXNR) {
		ipio_err("The number of ioctl doesn't match\n");
		return -ENOTTY;
	}

	ipio_info("cmd = %d\n", _IOC_NR(cmd));

	szBuf = kcalloc(IOCTL_I2C_BUFF, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(szBuf)) {
		ipio_err("Failed to allocate mem\n");
		return -ENOMEM;
	}

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, DISABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, DISABLE);

	mutex_lock(&idev_V2->touch_mutex);

	switch (cmd) {
	case ILITEK_IOCTL_I2C_WRITE_DATA:
		ipio_info("ioctl: write len = %d\n", i2c_rw_length);

		if (i2c_rw_length > IOCTL_I2C_BUFF || i2c_rw_length <= 0) {
			ipio_err("ERROR! i2c_rw_length is invalid (i2c_rw_length = %d, IOCTL_I2C_BUFF = %ld)\n",
					i2c_rw_length, IOCTL_I2C_BUFF);
			ret = -ENOTTY;
			break;
		}

		if (copy_from_user(szBuf, (u8 *) arg, i2c_rw_length)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ret = idev_V2->write(&szBuf[0], i2c_rw_length);
		if (ret < 0)
			ipio_err("Failed to write data\n");
		break;
	case ILITEK_IOCTL_I2C_READ_DATA:
		ipio_info("ioctl: read len = %d\n", i2c_rw_length);

		if (i2c_rw_length > IOCTL_I2C_BUFF) {
			ipio_err("ERROR! read len is largn than ioctl buf (%d, %ld)\n",
					i2c_rw_length, IOCTL_I2C_BUFF);
			ret = -ENOTTY;
			break;
		}

		ret = idev_V2->read(szBuf, i2c_rw_length);
		if (ret < 0) {
			ipio_err("Failed to read data\n");
			break;
		}
		if (copy_to_user((u8 *) arg, szBuf, i2c_rw_length)) {
			ipio_err("Failed to copy data to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_I2C_SET_WRITE_LENGTH:
	case ILITEK_IOCTL_I2C_SET_READ_LENGTH:
		i2c_rw_length = arg;
		break;
	case ILITEK_IOCTL_TP_HW_RESET:
		ipio_info("ioctl: hw reset\n");
		ilitek_tddi_reset_ctrl_V2(idev_V2->reset);
		break;
	case ILITEK_IOCTL_TP_POWER_SWITCH:
		ipio_info("Not implemented yet\n");
		break;
	case ILITEK_IOCTL_TP_REPORT_SWITCH:
		if (copy_from_user(szBuf, (u8 *) arg, 1)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_info("ioctl: report switch = %d\n", szBuf[0]);
		if (szBuf[0]) {
			idev_V2->report = ENABLE;
			ipio_info("report is enabled\n");
		} else {
			idev_V2->report = DISABLE;
			ipio_info("report is disabled\n");
		}
		break;
	case ILITEK_IOCTL_TP_IRQ_SWITCH:
		if (copy_from_user(szBuf, (u8 *) arg, 1)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_info("ioctl: irq switch = %d\n", szBuf[0]);
		if (szBuf[0])
			ilitek_plat_irq_enable_V2();
		else
			ilitek_plat_irq_disable_V2();
		break;
	case ILITEK_IOCTL_TP_DEBUG_LEVEL:
		if (copy_from_user(szBuf, (u32 *) arg, sizeof(u32))) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_debug_level_V2 = !ipio_debug_level_V2;
		ipio_info("ipio_debug_level_V2 = %d", ipio_debug_level_V2);
		break;
	case ILITEK_IOCTL_TP_FUNC_MODE:
		if (copy_from_user(szBuf, (u8 *) arg, 3)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_info("ioctl: set func mode = %x,%x,%x\n", szBuf[0], szBuf[1], szBuf[2]);
		idev_V2->write(&szBuf[0], 3);
		break;
	case ILITEK_IOCTL_TP_FW_VER:
		ipio_debug("ioctl: get fw and fw mp version\n");
		szBuf[7] = idev_V2->chip->fw_mp_ver & 0xFF;
		szBuf[6] = (idev_V2->chip->fw_mp_ver >> 8) & 0xFF;
		szBuf[5] = (idev_V2->chip->fw_mp_ver >> 16) & 0xFF;
		szBuf[4] = idev_V2->chip->fw_mp_ver >> 24;
		szBuf[3] = idev_V2->chip->fw_ver & 0xFF;
		szBuf[3] = idev_V2->chip->fw_ver & 0xFF;
		szBuf[2] = (idev_V2->chip->fw_ver >> 8) & 0xFF;
		szBuf[1] = (idev_V2->chip->fw_ver >> 16) & 0xFF;
		szBuf[0] = idev_V2->chip->fw_ver >> 24;
		ipio_debug("Firmware version = %d.%d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2], szBuf[3]);
		ipio_debug("Firmware MP version = %d.%d.%d.%d\n", szBuf[4], szBuf[5], szBuf[6], szBuf[7]);

		if (copy_to_user((u8 *) arg, szBuf, 8)) {
			ipio_err("Failed to copy firmware version to user space\n");
			ret = -ENOTTY;
		}
		break;

	case ILITEK_IOCTL_TP_PL_VER:
		ipio_info("ioctl: get protocl version\n");
		//ret = ilitek_tddi_ic_get_protocl_ver_V2();
		//if (ret < 0) {
			//ipio_err("Failed to get protocol version\n");
			//break;
		//}
		szBuf[2] = idev_V2->protocol->ver & 0xFF;
		szBuf[1] = (idev_V2->protocol->ver >> 8) & 0xFF;
		szBuf[0] = idev_V2->protocol->ver >> 16;
		ipio_info("Protocol version = %d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2]);
		if (copy_to_user((u8 *) arg, szBuf, 3)) {
			ipio_err("Failed to copy protocol version to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_CORE_VER:
		ipio_info("ioctl: get core version\n");
		//ret = ilitek_tddi_ic_get_core_ver_V2();
		//if (ret < 0) {
		//	ipio_err("Failed to get core version\n");
		//	break;
		//}
		szBuf[3] = idev_V2->chip->core_ver & 0xFF;
		szBuf[2] = (idev_V2->chip->core_ver >> 8) & 0xFF;
		szBuf[1] = (idev_V2->chip->core_ver >> 16) & 0xFF;
		szBuf[0] = idev_V2->chip->core_ver >> 24;
		ipio_info("Core version = %d.%d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2], szBuf[3]);
		if (copy_to_user((u8 *) arg, szBuf, 4)) {
			ipio_err("Failed to copy core version to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_DRV_VER:
		ipio_info("ioctl: get driver version\n");
		length = sprintf(szBuf, "%s", DRIVER_VERSION);
		if (copy_to_user((u8 *) arg, szBuf, length)) {
			ipio_err("Failed to copy driver ver to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_CHIP_ID:
		ipio_info("ioctl: get chip id\n");
		#if 0
		ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
		ret = ilitek_tddi_ic_get_info_V2();
		if (ret < 0) {
			ipio_err("Failed to get chip id\n");
			break;
		}
		#endif
		id_to_user[0] = idev_V2->chip->pid;
		id_to_user[1] = idev_V2->chip->otp_id;
		id_to_user[2] = idev_V2->chip->ana_id;
		if (copy_to_user((u32 *) arg, id_to_user, sizeof(id_to_user))) {
			ipio_err("Failed to copy chip id to user space\n");
			ret = -ENOTTY;
		}
		//ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
		break;
	case ILITEK_IOCTL_TP_NETLINK_CTRL:
		if (copy_from_user(szBuf, (u8 *) arg, 1)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_info("ioctl: netlink ctrl = %d\n", szBuf[0]);
		if (szBuf[0]) {
			idev_V2->netlink = ENABLE;
			ipio_info("ioctl: Netlink is enabled\n");
		} else {
			idev_V2->netlink = DISABLE;
			ipio_info("ioctl: Netlink is disabled\n");
		}
		break;
	case ILITEK_IOCTL_TP_NETLINK_STATUS:
		ipio_info("ioctl: get netlink stat = %d\n", idev_V2->netlink);
		if (copy_to_user((int *)arg, &idev_V2->netlink, sizeof(int))) {
			ipio_err("Failed to copy chip id to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_MODE_CTRL:
		if (copy_from_user(szBuf, (u8 *) arg, 4)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_info("ioctl: switch fw mode = %d\n", szBuf[0]);
		ret = ilitek_tddi_switch_mode_V2(szBuf);
		if (ret < 0) {
			ipio_info("switch to fw mode (%d) failed\n", szBuf[0]);
		}

		if(szBuf[0] == P5_X_FW_TEST_MODE) {
               idev_V2->mp_mode= true;
        }else {
               idev_V2->mp_mode = false;
         }
		break;
	case ILITEK_IOCTL_TP_MODE_STATUS:
		ipio_info("ioctl: current firmware mode = %d", idev_V2->actual_tp_mode);
		if (copy_to_user((int *)arg, &idev_V2->actual_tp_mode, sizeof(int))) {
			ipio_err("Failed to copy chip id to user space\n");
			ret = -ENOTTY;
		}
		break;
	/* It works for host downloado only */
	case ILITEK_IOCTL_ICE_MODE_SWITCH:
		if (copy_from_user(szBuf, (u8 *) arg, 1)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}

		ipio_debug("ioctl: switch ice mode = %d, mcu on = %d\n", szBuf[0], szBuf[1]);

		if (ilitek_ice_mode_ctrl_V2(szBuf[0] ? ENABLE : DISABLE,
			szBuf[1] ? ENABLE : DISABLE) < 0)
			ipio_err("ioctl: set ice mode failed\n");

		break;
	case ILITEK_IOCTL_TP_INTERFACE_TYPE:
		if_to_user = idev_V2->hwif->bus_type;
		if (copy_to_user((u8 *) arg, &if_to_user, sizeof(if_to_user))) {
			ipio_err("Failed to copy interface type to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_DUMP_FLASH:
		ipio_info("ioctl: dump flash data\n");
		ret = ilitek_tddi_fw_dump_flash_data_V2(0, 0, true);
		if (ret < 0) {
			ipio_err("ioctl: Failed to dump flash data\n");
		}
		break;
	case ILITEK_IOCTL_TP_FW_UART_CTRL:
		if (copy_from_user(szBuf, (u8 *) arg, 1)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}
		ipio_debug("ioctl: fw UART  = %d\n", szBuf[0]);

		ilitek_tddi_fw_uart_ctrl_V2(szBuf[0]);

		if_to_user = idev_V2->fw_uart_en;

		if (copy_to_user((u8 *) arg, &if_to_user, sizeof(if_to_user))) {
			ipio_err("Failed to copy driver ver to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_PANEL_INFO:
		ipio_debug("ioctl: get panel resolution\n");

		id_to_user[0] = idev_V2->panel_wid;
		id_to_user[1] = idev_V2->panel_hei;

		if (copy_to_user((u32 *) arg, id_to_user, sizeof(u32) * 2)) {
			ipio_err("Failed to copy driver ver to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_TP_INFO:
		ipio_debug("ioctl: get tp info\n");
		id_to_user[0] = idev_V2->min_x;
		id_to_user[1] = idev_V2->min_y;
		id_to_user[2] = idev_V2->max_x;
		id_to_user[3] = idev_V2->max_y;
		id_to_user[4] = idev_V2->xch_num;
		id_to_user[5] = idev_V2->ych_num;
		id_to_user[6] = idev_V2->stx;
		id_to_user[7] = idev_V2->srx;

		if (copy_to_user((u32 *) arg, id_to_user, sizeof(u32) * 8)) {
			ipio_err("Failed to copy driver ver to user space\n");
			ret = -ENOTTY;
		}
		break;
	case ILITEK_IOCTL_WRAPPER_RW:
		ipio_debug("ioctl: slave wrapper rw\n");
		if (i2c_rw_length > IOCTL_I2C_BUFF || i2c_rw_length <= 0) {
			ipio_err("ERROR! i2c_rw_length is is invalid\n");
			ret = -ENOTTY;
			break;
		}

		if (copy_from_user(szBuf, (u8 *) arg, i2c_rw_length)) {
			ipio_err("Failed to copy data from user space\n");
			ret = -ENOTTY;
			break;
		}

		wrap_int = szBuf[0];
		wrap_rlen = (szBuf[1] << 8) | szBuf[2];
		wrap_wlen = (szBuf[3] << 8) | szBuf[4];

		ipio_debug("wrap_int = %d\n", wrap_int);
		ipio_debug("wrap_rlen = %d\n", wrap_rlen);
		ipio_debug("wrap_wlen = %d\n", wrap_wlen);

		if (wrap_wlen > IOCTL_I2C_BUFF || wrap_rlen > IOCTL_I2C_BUFF) {
			ipio_err("ERROR! R/W len is largn than ioctl buf\n");
			ret = -ENOTTY;
			break;
		}

		if (wrap_rlen > 0) {
			wrap_rbuf = kcalloc(IOCTL_I2C_BUFF, sizeof(u8), GFP_KERNEL);
			if (ERR_ALLOC_MEM(wrap_rbuf)) {
				ipio_err("Failed to allocate mem\n");
				ret = -ENOMEM;
				break;
			}
		}

		ilitek_thp_send_cmd(szBuf + 5, wrap_wlen, wrap_rbuf, wrap_rlen);

		ilitek_dump_data_V2(szBuf + 5, 8, wrap_wlen, 16, "wrap_wbuf:");
		ilitek_dump_data_V2(wrap_rbuf, 8, wrap_rlen, 16, "wrap_rbuf:");

		if (copy_to_user((u8 *) arg, wrap_rbuf, wrap_rlen)) {
			ipio_err("Failed to copy driver ver to user space\n");
			ret = -ENOTTY;
		}

		break;
	default:
		ret = -ENOTTY;
		break;
	}

	ipio_kfree((void **)&szBuf);
	mutex_unlock(&idev_V2->touch_mutex);

	if (esd_en)
		ilitek_tddi_wq_ctrl_V2(WQ_ESD, ENABLE);
	if (bat_en)
		ilitek_tddi_wq_ctrl_V2(WQ_BAT, ENABLE);

	return ret;
}

struct proc_dir_entry *proc_dir_ilitek_V2;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct file_operations *fops;
	bool isCreated;
} proc_node_t;

struct file_operations proc_debug_message_fops_V2 = {
	.read = ilitek_proc_debug_message_read,
};

struct file_operations proc_debug_message_switch_fops_V2 = {
	.read = ilitek_proc_debug_switch_read,
};

struct file_operations proc_ioctl_fops_V2 = {
	.unlocked_ioctl = ilitek_node_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ilitek_node_compat_ioctl,
#endif
	.write = ilitek_node_ioctl_write,
};

struct file_operations proc_fw_upgrade_fops_V2 = {
	.read = ilitek_node_fw_upgrade_read,
};

struct file_operations proc_fw_process_fops_V2 = {
	.read = ilitek_proc_fw_process_read,
};
#if 0
struct file_operations proc_get_delta_data_fops_V2 = {
	.read = ilitek_proc_get_delta_data_read,
};

struct file_operations proc_get_raw_data_fops_V2 = {
	.read = ilitek_proc_fw_get_raw_data_read,
};
#endif
struct file_operations proc_rw_tp_reg_fops_V2 = {
	.read = ilitek_proc_rw_tp_reg_read,
	.write = ilitek_proc_rw_tp_reg_write,
};

struct file_operations proc_fw_pc_counter_fops_V2 = {
	.read = ilitek_proc_fw_pc_counter_read,
};
#if 0
struct file_operations proc_get_debug_mode_data_fops_V2 = {
	.read = ilitek_proc_get_debug_mode_data_read,
	.write = ilitek_proc_get_debug_mode_data_write,
};
#endif
struct file_operations proc_debug_level_fops_V2 = {
	.read = ilitek_proc_debug_level_read,
};

struct file_operations proc_vivo_fops_V2 = {
	.write = ilitek_proc_vivo_write,
};
#else /*kernel-5.10*/ 
typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct proc_ops *fops;
	bool isCreated;
} proc_node_t;

struct proc_ops proc_debug_message_fops_V2 = {
	.proc_read = ilitek_proc_debug_message_read,
};

struct proc_ops proc_debug_message_switch_fops_V2 = {
	.proc_read = ilitek_proc_debug_switch_read,
};

struct proc_ops proc_ioctl_fops_V2 = {
		.proc_ioctl = ilitek_node_ioctl,
#ifdef CONFIG_COMPAT
		.proc_ioctl = ilitek_node_compat_ioctl,
#endif

	.proc_write = ilitek_node_ioctl_write,
};

struct proc_ops proc_fw_upgrade_fops_V2 = {
	.proc_read = ilitek_node_fw_upgrade_read,
};

struct proc_ops proc_fw_process_fops_V2 = {
	.proc_read = ilitek_proc_fw_process_read,
};
#if 0
struct file_operations proc_get_delta_data_fops_V2 = {
	.read = ilitek_proc_get_delta_data_read,
};

struct file_operations proc_get_raw_data_fops_V2 = {
	.read = ilitek_proc_fw_get_raw_data_read,
};
#endif
struct proc_ops proc_rw_tp_reg_fops_V2 = {
	.proc_read = ilitek_proc_rw_tp_reg_read,
	.proc_write = ilitek_proc_rw_tp_reg_write,
};

struct proc_ops proc_fw_pc_counter_fops_V2 = {
	.proc_read = ilitek_proc_fw_pc_counter_read,
};
#if 0
struct file_operations proc_get_debug_mode_data_fops_V2 = {
	.read = ilitek_proc_get_debug_mode_data_read,
	.write = ilitek_proc_get_debug_mode_data_write,
};
#endif
struct proc_ops proc_debug_level_fops_V2 = {
	.proc_read = ilitek_proc_debug_level_read,
};

struct proc_ops proc_vivo_fops_V2 = {
	.proc_write = ilitek_proc_vivo_write,
};
#endif

proc_node_t proc_table_V2[] = {
	{"ioctl", NULL, &proc_ioctl_fops_V2, false},
	{"fw_process", NULL, &proc_fw_process_fops_V2, false},
	{"fw_upgrade", NULL, &proc_fw_upgrade_fops_V2, false},
	{"debug_level", NULL, &proc_debug_level_fops_V2, false},
	{"debug_message", NULL, &proc_debug_message_fops_V2, false},
	{"debug_message_switch", NULL, &proc_debug_message_switch_fops_V2, false},
	{"fw_pc_counter", NULL, &proc_fw_pc_counter_fops_V2, false},
	//{"show_delta_data", NULL, &proc_get_delta_data_fops_V2, false},
	//{"show_raw_data", NULL, &proc_get_raw_data_fops_V2, false},
	//{"get_debug_mode_data", NULL, &proc_get_debug_mode_data_fops_V2, false},
	{"rw_tp_reg", NULL, &proc_rw_tp_reg_fops_V2, false},
	{"vivo_test", NULL, &proc_vivo_fops_V2, false},
    //{"mp_test", NULL, &proc_vivo_mp_test_fops_V2, false},
};

#define NETLINK_USER 21
struct sock *netlink_skb_V2;
struct nlmsghdr *netlink_head_V2;
struct sk_buff *skb_out_V2;
int netlink_pid_V2;

void netlink_reply_msg_V2(void *raw, int size)
{
	int ret;
	int msg_size = size;
	u8 *data = (u8 *) raw;

	ipio_info("The size of data being sent to user = %d\n", msg_size);
	ipio_info("pid = %d\n", netlink_pid_V2);
	ipio_info("Netlink is enable = %d\n", idev_V2->netlink);

	if (idev_V2->netlink) {
		skb_out_V2 = nlmsg_new(msg_size, 0);

		if (!skb_out_V2) {
			ipio_err("Failed to allocate new skb\n");
			return;
		}

		netlink_head_V2 = nlmsg_put(skb_out_V2, 0, 0, NLMSG_DONE, msg_size, 0);
		NETLINK_CB(skb_out_V2).dst_group = 0;	/* not in mcast group */

		/* strncpy(NLMSG_DATA(netlink_head_V2), data, msg_size); */
		ipio_memcpy(nlmsg_data(netlink_head_V2), data, msg_size, size);

		ret = nlmsg_unicast(netlink_skb_V2, skb_out_V2, netlink_pid_V2);
		if (ret < 0)
			ipio_err("Failed to send data back to user\n");
	}
}

static void netlink_recv_msg(struct sk_buff *skb)
{
	netlink_pid_V2 = 0;

	ipio_info("Netlink = %d\n", idev_V2->netlink);

	netlink_head_V2 = (struct nlmsghdr *)skb->data;

	ipio_info("Received a request from client: %s, %d\n",
		(char *)NLMSG_DATA(netlink_head_V2), (int)strlen((char *)NLMSG_DATA(netlink_head_V2)));

	/* pid of sending process */
	netlink_pid_V2 = netlink_head_V2->nlmsg_pid;

	ipio_info("the pid of sending process = %d\n", netlink_pid_V2);

	/* TODO: may do something if there's not receiving msg from user. */
	if (netlink_pid_V2 != 0) {
		ipio_err("The channel of Netlink has been established successfully !\n");
		idev_V2->netlink = ENABLE;
	} else {
		ipio_err("Failed to establish the channel between kernel and user space\n");
		idev_V2->netlink = DISABLE;
	}
}

static int netlink_init(void)
{
	int ret = 0;

#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	netlink_skb_V2 = netlink_kernel_create(&init_net, NETLINK_USER, netlink_recv_msg, NULL, THIS_MODULE);
#else
	struct netlink_kernel_cfg cfg = {
		.input = netlink_recv_msg,
	};

	netlink_skb_V2 = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
#endif

	ipio_info("Initialise Netlink and create its socket\n");

	if (!netlink_skb_V2) {
		ipio_err("Failed to create nelink socket\n");
		ret = -EFAULT;
	}
	return ret;
}

static ssize_t ilitek_ioctl_read(struct file *filp, char __user *buff, size_t size, loff_t *pos)
{
	ipio_info("ilitek_ioctl_read\n");
	return 0;
}

static ssize_t ilitek_ioctl_write(struct file *filp, const char *buff, size_t size, loff_t *pos)
{
	ipio_info("ilitek_ioctl_write\n");
	return size;
}

static int ilitek_node_open(struct inode *inode, struct file *file)
{
	ipio_info("ilitek_node_open\n");
	return 0;
}

static const struct file_operations IoctlFileOps = {
	.owner = THIS_MODULE,
	.open = ilitek_node_open,
	.read = ilitek_ioctl_read,
	.write = ilitek_ioctl_write,
	.release = NULL,
	.unlocked_ioctl = ilitek_node_ioctl,
};

static struct miscdevice IoctlMisc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ilitek_ioctl",
	.fops = &IoctlFileOps,
};

void ilitek_tddi_node_init_V2(void)
{
	int i = 0, ret = 0;
	int nRetVal = 0;

	nRetVal = misc_register(&IoctlMisc);
	if (nRetVal)
		ipio_err("Failed to create ioctl node\n");

	proc_dir_ilitek_V2 = proc_mkdir("ilitek", NULL);

	for (; i < ARRAY_SIZE(proc_table_V2); i++) {
		proc_table_V2[i].node = proc_create(proc_table_V2[i].name, 0644, proc_dir_ilitek_V2, proc_table_V2[i].fops);

		if (proc_table_V2[i].node == NULL) {
			proc_table_V2[i].isCreated = false;
			ipio_err("Failed to create %s under /proc\n", proc_table_V2[i].name);
			ret = -ENODEV;
		} else {
			proc_table_V2[i].isCreated = true;
			ipio_info("Succeed to create %s under /proc\n", proc_table_V2[i].name);
		}
	}
	netlink_init();
}
