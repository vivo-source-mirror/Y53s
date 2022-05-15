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
//#include <linux/vivo_ts_function.h>
struct ilitek_gesture_info ilitek_geature_data;

void ilitek_dump_data_V2(void *data, int type, int len, int row_len, const char *name)
{
	int i, row = 31;
	u8 *p8 = NULL;
	s32 *p32 = NULL;

	if (!ipio_debug_level_V2)
		return;

	if (row_len > 0)
		row = row_len;

	if (data == NULL) {
		ipio_err("The data going to dump is NULL\n");
		return;
	}

	pr_cont("\n\n");
	pr_cont("ILITEK: Dump %s data\n", name);
	pr_cont("ILITEK: ");

	if (type == 8)
		p8 = (u8 *) data;
	if (type == 32 || type == 10)
		p32 = (s32 *) data;

	for (i = 0; i < len; i++) {
		if (type == 8)
			pr_cont(" %4x ", p8[i]);
		else if (type == 32)
			pr_cont(" %4x ", p32[i]);
		else if (type == 10)
			pr_cont(" %4d ", p32[i]);
		if ((i % row) == row - 1) {
			pr_cont("\n");
			pr_cont("ILITEK: ");
		}
	}
	pr_cont("\n\n");
}

static void dma_clear_reg_setting(void)
{
	ipio_info("[Clear register setting]\n");

	ipio_info("interrupt t0/t1 enable flag\n");
	ilitek_ice_mode_bit_mask_write_V2(INTR32_ADDR, INTR32_reg_t0_int_en, (0 << 24));
	ilitek_ice_mode_bit_mask_write_V2(INTR32_ADDR, INTR32_reg_t1_int_en, (0 << 25));

	ipio_info("clear tdi_err_int_flag\n");
	ilitek_ice_mode_bit_mask_write_V2(INTR2_ADDR, INTR2_tdi_err_int_flag_clear, (1 << 18));

	ipio_info("clear dma channel 0 src1 info\n");
	ilitek_ice_mode_write_V2(DMA49_reg_dma_ch0_src1_addr, 0x00000000, 4);
	ilitek_ice_mode_write_V2(DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1);
	ilitek_ice_mode_bit_mask_write_V2(DMA50_ADDR, DMA50_reg_dma_ch0_src1_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write_V2(DMA50_ADDR, DMA50_reg_dma_ch0_src1_en, (1 << 31));

	ipio_info("clear dma channel 0 src2 info\n");
	ilitek_ice_mode_bit_mask_write_V2(DMA52_ADDR, DMA52_reg_dma_ch0_src2_en, (0 << 31));

	ipio_info("clear dma channel 0 trafer info\n");
	ilitek_ice_mode_write_V2(DMA55_reg_dma_ch0_trafer_counts, 0x00000000, 4);
	ilitek_ice_mode_bit_mask_write_V2(DMA55_ADDR, DMA55_reg_dma_ch0_trafer_mode, (0 << 24));

	ipio_info("clear dma channel 0 trigger select\n");
	ilitek_ice_mode_bit_mask_write_V2(DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (0 << 16));

	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ipio_info("clear dma flash setting\n");
	ilitek_tddi_flash_clear_dma_V2();
}

static void dma_trigger_reg_setting(u32 reg_dest_addr, u32 flash_start_addr, u32 copy_size)
{
	ipio_info("set dma channel 0 clear\n");
	ilitek_ice_mode_bit_mask_write_V2(DMA48_ADDR, DMA48_reg_dma_ch0_start_clear, (1 << 25));

	ipio_info("set dma channel 0 src1 info\n");
	ilitek_ice_mode_write_V2(DMA49_reg_dma_ch0_src1_addr, 0x00041010, 4);
	ilitek_ice_mode_write_V2(DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1);
	ilitek_ice_mode_bit_mask_write_V2(DMA50_ADDR, DMA50_reg_dma_ch0_src1_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write_V2(DMA50_ADDR, DMA50_reg_dma_ch0_src1_en, (1 << 31));

	ipio_info("set dma channel 0 src2 info\n");
	ilitek_ice_mode_bit_mask_write_V2(DMA52_ADDR, DMA52_reg_dma_ch0_src2_en, (0 << 31));

	ipio_info("set dma channel 0 dest info\n");
	ilitek_ice_mode_write_V2(DMA53_reg_dma_ch0_dest_addr, reg_dest_addr, 3);
	ilitek_ice_mode_write_V2(DMA54_reg_dma_ch0_dest_step_inc, 0x01, 1);
	ilitek_ice_mode_bit_mask_write_V2(DMA54_ADDR, DMA54_reg_dma_ch0_dest_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write_V2(DMA54_ADDR, DMA54_reg_dma_ch0_dest_en, (1 << 31));

	ipio_info("set dma channel 0 trafer info\n");
	ilitek_ice_mode_write_V2(DMA55_reg_dma_ch0_trafer_counts, copy_size, 4);
	ilitek_ice_mode_bit_mask_write_V2(DMA55_ADDR, DMA55_reg_dma_ch0_trafer_mode, (0 << 24));

	ipio_info("set dma channel 0 int info\n");
	ilitek_ice_mode_bit_mask_write_V2(INTR33_ADDR, INTR33_reg_dma_ch0_int_en, (1 << 17));

	ipio_info("set dma channel 0 trigger select\n");
	ilitek_ice_mode_bit_mask_write_V2(DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (1 << 16));

	ipio_info("set dma flash setting, FlashAddr = 0x%x\n", flash_start_addr);
	ilitek_tddi_flash_dma_write_V2(flash_start_addr, (flash_start_addr+copy_size), copy_size);

	ipio_info("clear flash and dma ch0 int flag\n");
	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));
	ilitek_ice_mode_bit_mask_write_V2(INTR1_ADDR, INTR1_reg_dma_ch0_int_flag, (1 << 17));
	ilitek_ice_mode_bit_mask_write_V2(0x041013, BIT(0), 1); /*patch*/

	/* DMA Trigger */
	ilitek_ice_mode_write_V2(FLASH4_reg_rcv_data, 0xFF, 1);
	/* waiting for fw reload code completed. */
	mdelay(30);

	/* CS High */
	ilitek_ice_mode_write_V2(FLASH0_reg_flash_csb, 0x1, 1);
	/* waiting for CS status done */
	mdelay(10);
}

int ilitek_tddi_move_mp_code_flash_V2(void)
{
	int ret = 0;
	u32 mp_text_size = 0, mp_andes_init_size = 0;
	u32 mp_flash_addr, mp_size, overlay_start_addr, overlay_end_addr;
	bool dma_trigger_enable = 0;
	u8 cmd[16] = {0};

	cmd[0] = P5_X_MODE_CONTROL;
	cmd[1] = P5_X_FW_TEST_MODE;
	ret = idev_V2->write(cmd, 2);
	if (ret < 0)
		goto out;

	cmd[0] = P5_X_MP_TEST_MODE_INFO;
	ret = idev_V2->write(cmd, 1);
	if (ret < 0)
		goto out;

	memset(cmd, 0, sizeof(cmd));

	ipio_info("read mp info length = %d\n", idev_V2->protocol->mp_info_len);
	ret = idev_V2->read(cmd, idev_V2->protocol->mp_info_len);
	if (ret < 0)
		goto out;

	ilitek_dump_data_V2(cmd, 8, idev_V2->protocol->mp_info_len, 0, "MP overlay info");

	dma_trigger_enable = 0;

	mp_flash_addr = cmd[3] + (cmd[2] << 8) + (cmd[1] << 16);
	mp_size = cmd[6] + (cmd[5] << 8) + (cmd[4] << 16);
	overlay_start_addr = cmd[9] + (cmd[8] << 8) + (cmd[7] << 16);
	overlay_end_addr = cmd[12] + (cmd[11] << 8) + (cmd[10] << 16);

	if (overlay_start_addr != 0x0 && overlay_end_addr != 0x0
		&& cmd[0] == P5_X_MP_TEST_MODE_INFO)
		dma_trigger_enable = 1;

	ipio_info("MP info Overlay: Enable = %d, addr = 0x%x ~ 0x%x, flash addr = 0x%x, mp size = 0x%x\n",
		dma_trigger_enable, overlay_start_addr,
		overlay_end_addr, mp_flash_addr, mp_size);

	/* Check if ic is ready switching test mode from demo mode */
	idev_V2->actual_tp_mode = P5_X_FW_DEMO_MODE;
	ret = ilitek_tddi_ic_check_busy_V2(50, 50); /* Set busy as 0x41 */
	if (ret < 0)
		goto out;

	ret = ilitek_ice_mode_ctrl_V2(ENABLE, OFF);
	if (ret < 0)
		goto out;

	if (dma_trigger_enable) {
		mp_andes_init_size = overlay_start_addr;
		mp_text_size = (mp_size - overlay_end_addr) + 1;
		ipio_info("MP andes init size = %d , MP text size = %d\n", mp_andes_init_size, mp_text_size);

		dma_clear_reg_setting();

		ipio_info("[Move ANDES.INIT to DRAM]\n");
		dma_trigger_reg_setting(0, mp_flash_addr, mp_andes_init_size);	 /* DMA ANDES.INIT */

		dma_clear_reg_setting();

		ipio_info("[Move MP.TEXT to DRAM]\n");
		dma_trigger_reg_setting(overlay_end_addr, (mp_flash_addr + overlay_start_addr), mp_text_size);

		dma_clear_reg_setting();
	} else {
		/* DMA Trigger */
		ilitek_ice_mode_write_V2(FLASH4_reg_rcv_data, 0xFF, 1);
		/* waiting for fw reload code completed. */
		mdelay(30);

		/* CS High */
		ilitek_ice_mode_write_V2(FLASH0_reg_flash_csb, 0x1, 1);
		/* waiting for CS status done */
		mdelay(10);
	}

	ilitek_tddi_reset_ctrl_V2(TP_IC_CODE_RST);

	ret = ilitek_ice_mode_ctrl_V2(DISABLE, OFF);
	if (ret < 0)
		goto out;

	/* Check if ic is already in test mode */
	idev_V2->actual_tp_mode = P5_X_FW_TEST_MODE; /* set busy as 0x51 */
	ret = ilitek_tddi_ic_check_busy_V2(300, 50);

out:
	return ret;
}

int ilitek_tddi_move_mp_code_iram_V2(void)
{
	ipio_info("Download MP code to iram\n");
	return ilitek_tddi_fw_upgrade_handler_V2(NULL);
}

int ilitek_tddi_proximity_near_V2(int mode)
{
	int ret = 0;

	switch (mode) {
	case DDI_POWER_ON:
		/*
		 * If the power of VSP and VSN keeps alive when proximity near event
		 * occures, TP can just go to sleep in.
		 */
		ret = ilitek_tddi_ic_func_ctrl_V2("sleep", SLEEP_IN);
		break;
	case DDI_POWER_OFF:
		ipio_info("DDI POWER OFF, do nothing\n");
		break;
	default:
		ipio_err("Unknown mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int ilitek_tddi_proximity_far_V2(int mode)
{
	int ret = 0;
	u8 tp_mode = 0;
	u8 cmd[2] = {0};

	switch (mode) {
	case WAKE_UP_GESTURE_RECOVERY:
		/*
		 * If the power of VSP and VSN has been shut down previsouly,
		 * TP should go through gesture_V2 recovery to get back.
		 */
		ilitek_tddi_wq_ctrl_V2(WQ_GES_RECOVER, ENABLE);
		break;
	case WAKE_UP_SWITCH_GESTURE_MODE:
		/*
		 * If the power of VSP and VSN keeps alive in the event of proximity near,
		 * TP can be just recovered by switching gesture_V2 mode to get back.
		 */
		cmd[0] = 0xF6;
		cmd[1] = 0x0A;

		ipio_info("write prepare gesture_V2 command 0xF6 0x0A\n");
		// ret = idev->write(cmd, 2);
		ret = ilitek_thp_send_cmd(cmd, 2, NULL, 0);
		if (ret < 0) {
			ipio_info("write prepare gesture_V2 command error\n");
			break;
		}
		tp_mode = P5_X_FW_GESTURE_MODE;
		ret = ilitek_tddi_switch_mode_V2(&tp_mode);
		break;
	default:
		ipio_err("Unknown mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}
	return ret;
}

int ilitek_tddi_move_gesture_code_flash_V2(int mode)
{
	u8 tp_mode = P5_X_FW_GESTURE_MODE;

	ipio_info();
	return ilitek_tddi_switch_mode_V2(&tp_mode);
}

int ilitek_tddi_move_gesture_code_iram_V2(int mode)
{
	int i;
	int timeout = 10;
	u8 tp_mode = P5_X_FW_GESTURE_MODE;
	u8 cmd[3] = {0};
	u8 buf[3] = {0};

	ipio_info();

	if (ilitek_tddi_ic_func_ctrl_V2("lpwg", 0x3) < 0)
		ipio_err("write gesture_V2 flag failed\n");

	ilitek_tddi_switch_mode_V2(&tp_mode);

	for (i = 0; i < timeout; i++) {
		/* Prepare Check Ready */
		//cmd[0] = P5_X_READ_DATA_CTRL;
		//cmd[1] = 0xA;
		//idev_V2->write(cmd, 2);

		/* Check ready for load code */
		cmd[0] = 0x1;
		cmd[1] = 0xA;
		cmd[2] = 0x5;
		if (ilitek_thp_send_cmd(cmd, 3, buf, 2) < 0)
			ipio_err("write 0x1,0xA,0x5 error");


		//if ((idev_V2->read(cmd, 1)) < 0)
		//	ipio_err("read gesture_V2 ready byte error\n");

		ipio_debug("gesture_V2 ready byte = 0x%x\n", buf[0]);
		if (buf[0] == 0x91) {
			ipio_info("Gesture check fw ready\n");
			break;
		}
	}

	if (i >= timeout) {
		ipio_err("Gesture is not ready (0x%x), try to run its recovery\n", cmd[0]);
//		ilitek_tddi_wq_ctrl_V2(WQ_GES_RECOVER, ENABLE);
		ilitek_tddi_gesture_recovery_V2();
		return 0;
	}
	ilitek_tddi_fw_upgrade_handler_V2(NULL);

	/* FW star run gestrue code cmd */
	cmd[0] = 0x1;
	cmd[1] = 0xA;
	cmd[2] = 0x6;
	// if ((idev->write(cmd, 3)) < 0)
	if (ilitek_thp_send_cmd(cmd, 3, NULL, 0) < 0)
		ipio_err("write 0x1,0xA,0x6 error");
	return 0;
}

u8 ilitek_calc_packet_checksum_V2(u8 *packet, int len)
{
	int i;
	s32 sum = 0;

	for (i = 0; i < len; i++)
		sum += packet[i];

	return (u8) ((-sum) & 0xFF);
}

void ilitek_tddi_touch_esd_gesture_iram_V2(void)
{
	int retry = 100;
	u32 answer = 0;
	u8 cmd[3] = {0};

	ilitek_ice_mode_ctrl_V2(ENABLE, OFF);

	ipio_info("ESD Gesture PWD Addr = 0x%x, Answer = 0x%x\n",
		SPI_ESD_GESTURE_PWD_ADDR, SPI_ESD_GESTURE_RUN);

	/* write a special password to inform FW go back into gesture_V2 mode */
	if (ilitek_ice_mode_write_V2(SPI_ESD_GESTURE_PWD_ADDR, ESD_GESTURE_PWD, 4) < 0)
		ipio_err("write password failed\n");

	/* Host download gives effect to FW receives password successed */
	idev_V2->actual_tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_fw_upgrade_handler_V2(NULL);

	ilitek_ice_mode_ctrl_V2(ENABLE, ON);

	/* polling another specific register to see if gesutre is enabled properly */
	do {
		if (ilitek_ice_mode_read_V2(SPI_ESD_GESTURE_PWD_ADDR, &answer, sizeof(u32)) < 0)
			ipio_err("Read gesture_V2 answer error\n");

		if (answer != SPI_ESD_GESTURE_RUN)
			ipio_info("answer = 0x%x != (0x%x)\n", answer, SPI_ESD_GESTURE_RUN);
		mdelay(1);
	} while (answer != SPI_ESD_GESTURE_RUN && --retry > 0);

	if (retry <= 0)
		ipio_err("Enter gesture_V2 failed\n");
	else
		ipio_info("Enter gesture_V2 successfully\n");

	ilitek_ice_mode_ctrl_V2(DISABLE, ON);

	idev_V2->actual_tp_mode = P5_X_FW_GESTURE_MODE;
	ilitek_tddi_fw_upgrade_handler_V2(NULL);

	/* FW star run gestrue code cmd */
	cmd[0] = 0x1;
	cmd[1] = 0xA;
	cmd[2] = 0x6;
	// if ((idev->write(cmd, sizeof(cmd))) < 0)
	if (ilitek_thp_send_cmd(cmd, sizeof(cmd), NULL, 0) < 0)
		ipio_err("write 0x1,0xA,0x6 error");

}

int ilitek_tddi_debug_report_alloc_V2(void)
{
	int i, ret = 0;
	int row_size = 1 * K + 1, col_size = 2 * K + 1;

	if (!idev_V2->debug_node_open && !idev_V2->debug_buf)
		return ret;

	if (!idev_V2->debug_node_open && idev_V2->debug_buf != NULL)
		goto out;

	if (idev_V2->debug_node_open && !idev_V2->debug_buf) {
		idev_V2->debug_buf = (unsigned char **)kzalloc(row_size * sizeof(unsigned char *), GFP_KERNEL);
		if (ERR_ALLOC_MEM(idev_V2->debug_buf)) {
			ipio_err("Failed to allocate debug_buf mem, %ld\n", PTR_ERR(idev_V2->debug_buf));
			ret = -ENOMEM;
			goto out;
		}

		for (i = 0; i < row_size; i++) {
			idev_V2->debug_buf[i] = (unsigned char *)kzalloc(col_size * sizeof(unsigned char), GFP_KERNEL);
			if (ERR_ALLOC_MEM(idev_V2->debug_buf[i])) {
				ipio_err("Failed to allocate debug_buf[%d] mem, %ld\n", i, PTR_ERR(idev_V2->debug_buf[i]));
				ret = -ENOMEM;
				goto out;
			}
		}
	}
	return ret;

out:
	/* Note that it might be freed by next touch event */
	if (idev_V2->debug_buf != NULL) {
		idev_V2->debug_data_frame = 0;
		for (i = 0; i < row_size; i++) {
			if (idev_V2->debug_buf[i] != NULL) {
				kfree(idev_V2->debug_buf[i]);
				idev_V2->debug_buf[i] = NULL;
			}
		}
		kfree(idev_V2->debug_buf);
		idev_V2->debug_buf = NULL;
	}
	return ret;
}



static void ilitek_tddi_touch_send_debug_data(u8 *buf, int len)
{

	mutex_lock(&idev_V2->debug_mutex);

	if (!idev_V2->debug_node_open)
		idev_V2->debug_data_frame = 0;

	if (ilitek_tddi_debug_report_alloc_V2() < 0)
		goto out;

	if (!idev_V2->netlink && !idev_V2->debug_node_open)
		goto out;

	/* Send data to netlink */
	if (idev_V2->netlink) {
		netlink_reply_msg_V2(buf, len);
		goto out;
	}

	/* Sending data to apk via the node of debug_message node */
	if (idev_V2->debug_node_open) {
		memset(idev_V2->debug_buf[idev_V2->debug_data_frame], 0x00, (u8)sizeof(u8) * 2048);
		ipio_memcpy(idev_V2->debug_buf[idev_V2->debug_data_frame], buf, len, 2048);
		idev_V2->debug_data_frame++;
		if (idev_V2->debug_data_frame > 1)
			ipio_debug("idev_V2->debug_data_frame = %d\n", idev_V2->debug_data_frame);
		if (idev_V2->debug_data_frame > 1023) {
			ipio_err("idev_V2->debug_data_frame = %d > 1024\n",
				idev_V2->debug_data_frame);
			idev_V2->debug_data_frame = 1023;
		}
		wake_up(&(idev_V2->inq));
		complete(&idev_V2->touch_debug_done);
		goto out;
	}

out:
	mutex_unlock(&idev_V2->debug_mutex);
}

/*
void ilitek_tddi_touch_release(u16 x, u16 y, u16 id)
{
	ipio_debug("Touch Release: id = %d, x = %d, y = %d\n", id, x, y);

	if (MT_B_TYPE) {
		input_mt_slot(idev_V2->input, id);
		input_mt_report_slot_state(idev_V2->input, MT_TOOL_FINGER, false);
	} else {
		input_report_key(idev_V2->input, BTN_TOUCH, 0);
		input_mt_sync(idev_V2->input);
	}
}
*/
/*
void ilitek_tddi_touch_release_all_point(void)
{
	int i;

	if (MT_B_TYPE) {
		for (i = 0 ; i < MAX_TOUCH_NUM; i++)
			ilitek_tddi_touch_release(0, 0, i);

		input_report_key(idev_V2->input, BTN_TOUCH, 0);
		input_report_key(idev_V2->input, BTN_TOOL_FINGER, 0);
	} else {
		ilitek_tddi_touch_release(0, 0, 0);
	}
	input_sync(idev_V2->input);
}
*/
static struct ilitek_touch_info touch_info[MAX_TOUCH_NUM];

void ilitek_tddi_report_ap_mode_V2(u8 *buf, int len, ktime_t kt)
{
	int i = 0;
	u32 xop = 0, yop = 0;
    struct vts_device * vtsdev = idev_V2->vtsdev;
	memset(touch_info, 0x0, sizeof(touch_info));

	idev_V2->finger = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((buf[(4 * i) + 1] == 0xFF) && (buf[(4 * i) + 2] == 0xFF)
			&& (buf[(4 * i) + 3] == 0xFF)) {
			if (MT_B_TYPE)
				idev_V2->curt_touch[i] = 0;
			continue;
		}

		xop = (((buf[(4 * i) + 1] & 0xF0) << 4) | (buf[(4 * i) + 2]));
		yop = (((buf[(4 * i) + 1] & 0x0F) << 8) | (buf[(4 * i) + 3]));

		if (idev_V2->trans_xy) {
			touch_info[idev_V2->finger].x = xop;
			touch_info[idev_V2->finger].y = yop;
		} else {
			touch_info[idev_V2->finger].x = xop * idev_V2->panel_wid / TPD_WIDTH;
			touch_info[idev_V2->finger].y = yop * idev_V2->panel_hei / TPD_HEIGHT;
		}

		touch_info[idev_V2->finger].id = i;

		if (MT_PRESSURE)
			touch_info[idev_V2->finger].pressure = buf[(4 * i) + 4];
		else
			touch_info[idev_V2->finger].pressure = 1;

		ipio_debug("original x = %d, y = %d\n", xop, yop);
		idev_V2->finger++;
		if (MT_B_TYPE)
			idev_V2->curt_touch[i] = 1;
	}
	
	

	ipio_debug("figner number = %d, LastTouch = %d\n", idev_V2->finger, idev_V2->last_touch);

	if (idev_V2->finger) {	
		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			if (idev_V2->curt_touch[i] == 0 && idev_V2->prev_touch[i] == 1)
				//vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
				//ilitek_tddi_touch_release(0, 0, i);
				vts_report_point_up(vtsdev,i,idev_V2->finger,touch_info[i].x,touch_info[i].y,
				   touch_info[i].pressure,touch_info[i].pressure,0,kt);
			idev_V2->prev_touch[i] = idev_V2->curt_touch[i];
		}
		for (i = 0; i < idev_V2->finger; i++) {
			//input_report_key(idev_V2->input, BTN_TOUCH, 1);
			//ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
			//input_report_key(idev_V2->input, BTN_TOOL_FINGER, 1);			
			//vivoTsInputReport(VTS_TOUCH_DOWN, touch_info[i].id, touch_info[i].x, touch_info[i].y, touch_info[i].pressure);
			vts_report_point_down(vtsdev,touch_info[i].id,idev_V2->finger,touch_info[i].x,touch_info[i].y,
			touch_info[i].pressure,touch_info[i].pressure,0,NULL,0,kt);

		}
		 
		//input_sync(idev_V2->input);
		idev_V2->last_touch = idev_V2->finger;
	} else {
		if (idev_V2->last_touch) {		
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				if (idev_V2->curt_touch[i] == 0 && idev_V2->prev_touch[i] == 1)
					//vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
					//ilitek_tddi_touch_release(0, 0, i);	
					//vts_report_point_up(vtsdev,i,0,int x,int y,int wx,int wy,bool large_press,ktime_t kt);
				   vts_report_point_up(vtsdev,i,idev_V2->finger,touch_info[i].x,touch_info[i].y,
				   touch_info[i].pressure,touch_info[i].pressure,0,kt);
				idev_V2->prev_touch[i] = idev_V2->curt_touch[i];
			}
			//input_report_key(idev_V2->input, BTN_TOUCH, 0);
			//input_report_key(idev_V2->input, BTN_TOOL_FINGER, 0);
			
			//input_sync(idev_V2->input);
			idev_V2->last_touch = 0;
		}
	}
	vts_report_point_sync(vtsdev);
	ilitek_tddi_touch_send_debug_data(buf, len);
}

void ilitek_tddi_report_debug_mode_V2(u8 *buf, int len, ktime_t kt)
{
	int i = 0;
	u32 xop = 0, yop = 0;
	struct vts_device * vtsdev = idev_V2->vtsdev;

	memset(touch_info, 0x0, sizeof(touch_info));

	idev_V2->finger = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((buf[(3 * i) + 5] == 0xFF) && (buf[(3 * i) + 6] == 0xFF)
			&& (buf[(3 * i) + 7] == 0xFF)) {
			if (MT_B_TYPE)
				idev_V2->curt_touch[i] = 0;
			continue;
		}

		xop = (((buf[(3 * i) + 5] & 0xF0) << 4) | (buf[(3 * i) + 6]));
		yop = (((buf[(3 * i) + 5] & 0x0F) << 8) | (buf[(3 * i) + 7]));

		if (idev_V2->trans_xy) {
			touch_info[idev_V2->finger].x = xop;
			touch_info[idev_V2->finger].y = yop;
		} else {
			touch_info[idev_V2->finger].x = xop * idev_V2->panel_wid / TPD_WIDTH;
			touch_info[idev_V2->finger].y = yop * idev_V2->panel_hei / TPD_HEIGHT;
		}

		touch_info[idev_V2->finger].id = i;

		if (MT_PRESSURE)
			touch_info[idev_V2->finger].pressure = buf[(4 * i) + 4];
		else
			touch_info[idev_V2->finger].pressure = 1;

		ipio_debug("original x = %d, y = %d\n", xop, yop);
		idev_V2->finger++;
		if (MT_B_TYPE)
			idev_V2->curt_touch[i] = 1;
	}

	ipio_debug("figner number = %d, LastTouch = %d\n", idev_V2->finger, idev_V2->last_touch);

	if (idev_V2->finger) {	
		for (i = 0; i < idev_V2->finger; i++) {
			//input_report_key(idev_V2->input, BTN_TOUCH, 1);
			//ilitek_tddi_touch_press(touch_info[i].x, touch_info[i].y, touch_info[i].pressure, touch_info[i].id);
			//input_report_key(idev_V2->input, BTN_TOOL_FINGER, 1);			
			//vivoTsInputReport(VTS_TOUCH_DOWN, touch_info[i].id, touch_info[i].x, touch_info[i].y, touch_info[i].pressure);
			vts_report_point_down(vtsdev,touch_info[i].id,idev_V2->finger,touch_info[i].x,touch_info[i].y,
						touch_info[i].pressure,touch_info[i].pressure,0,NULL,0,kt);

		}
		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			if (idev_V2->curt_touch[i] == 0 && idev_V2->prev_touch[i] == 1)
			//	vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
				//ilitek_tddi_touch_release(0, 0, i);		
			vts_report_point_up(vtsdev,i,idev_V2->finger,touch_info[i].x,touch_info[i].y,
				   touch_info[i].pressure,touch_info[i].pressure,0,kt);
			idev_V2->prev_touch[i] = idev_V2->curt_touch[i];
		}		 
		//input_sync(idev_V2->input);
		idev_V2->last_touch = idev_V2->finger;
	} else {
		if (idev_V2->last_touch) {		
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				if (idev_V2->curt_touch[i] == 0 && idev_V2->prev_touch[i] == 1)
				//	vivoTsInputReport(VTS_TOUCH_UP, i, 0, 0, 0);
					//ilitek_tddi_touch_release(0, 0, i);			
				vts_report_point_up(vtsdev,i,idev_V2->finger,touch_info[i].x,touch_info[i].y,
				   touch_info[i].pressure,touch_info[i].pressure,0,kt);
				idev_V2->prev_touch[i] = idev_V2->curt_touch[i];
			}
			//input_report_key(idev_V2->input, BTN_TOUCH, 0);
			//input_report_key(idev_V2->input, BTN_TOOL_FINGER, 0);
			
			//input_sync(idev_V2->input);
			idev_V2->last_touch = 0;
		}
	}
	vts_report_point_sync(vtsdev);
	ilitek_tddi_touch_send_debug_data(buf, len);
}

void ilitek_tddi_report_gesture_mode_V2(u8 *buf, int len)
{   
    u8 i;
	uint32_t gesture_V2 = 0;
	u16 gesturePoint[130];	/*gesture_V2 coordinate */
	int gesturePointNum = 0;
	memset(gesturePoint, 0xff, sizeof(gesturePoint));
	ipio_info("gesture_V2 code = 0x%x\n", buf[1]);

	switch (buf[1]) {
	case GESTURE_DOUBLECLICK:
		ipio_info("Double Click key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_DOUBLE_CLICK; 		
		break;	
	case GESTURE_UP:
		ipio_info("GESTURE_UP key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_UP;		
		break;	
	case GESTURE_DOWN:
		ipio_info("GESTURE_DOWN key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_DOWN;
		break;
	case GESTURE_LEFT:		
		ipio_info("GESTURE_LEFT key event\n");
        gesture_V2 = VTS_EVENT_GESTURE_PATTERN_LEFT;
        break;
    case GESTURE_RIGHT:		
		ipio_info("GESTURE_RIGHT key event\n");
        gesture_V2 = VTS_EVENT_GESTURE_PATTERN_RIGHT;
        break;
	case GESTURE_O:
		ipio_info("GESTURE_O key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_O;		
		break;
	case GESTURE_W:
		ipio_info("GESTURE_W key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_W;		
		break;
	case GESTURE_M:
		ipio_info("GESTURE_M key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_M;
		break;
	case GESTURE_E:
		ipio_info("GESTURE_E key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_E;		
		break;	
	case GESTURE_C:
		ipio_info("GESTURE_C key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_C;		
		break;
	case GESTURE_F:
		ipio_info("GESTURE_F key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_F;		
		break;
	case GESTURE_A:
		ipio_info("GESTURE_A key event\n");
		gesture_V2 = VTS_EVENT_GESTURE_PATTERN_A;		
		break;
	default:
		gesture_V2 = -1;
		break;
	}
	idev_V2->debug_node_open = ENABLE;
	idev_V2->debug_data_frame = 0;
	ilitek_tddi_touch_send_debug_data(buf, len);

	 if (gesture_V2 != -1) {
		/*
        VTD("Gesture Code=%d", gesture_V2);
        input_report_key(input_dev, gesture_V2, 1);
        input_sync(input_dev);
        input_report_key(input_dev, gesture_V2, 0);
        input_sync(input_dev);
        */
        VTI("Gesture num %d", gesture_V2);
		vts_report_event_down(idev_V2->vtsdev, gesture_V2);
		vts_report_event_up(idev_V2->vtsdev, gesture_V2);
       // vivoTsInputReport(VTS_GESTURE_EVENT, gesture_V2, -1, -1, -1);
       if(gesturePointNum > ILI_GESTURE_POINT_MAX)
	   	  gesturePointNum = ILI_GESTURE_POINT_MAX;
	   	/*get gesture_V2 data*/
	   gesturePointNum = bbk_ili_gesture_point_get_V2(gesturePoint);
	   memset(ilitek_geature_data.coordinate_x, 0, ILI_GESTURE_POINT_MAX * sizeof(u16));
   	   memset(ilitek_geature_data.coordinate_y, 0, ILI_GESTURE_POINT_MAX * sizeof(u16)); 
       for(i=0; i<gesturePointNum; i++){
       	ilitek_geature_data.coordinate_x[i] = gesturePoint[2*i];
        ilitek_geature_data.coordinate_y[i] = gesturePoint[2*i+1];
	   }
    }
	 vts_report_coordinates_set(idev_V2->vtsdev, ilitek_geature_data.coordinate_x, ilitek_geature_data.coordinate_y, gesturePointNum);
}

void ilitek_tddi_report_i2cuart_mode_V2(u8 *buf, int len)
{
	int type = buf[3] & 0x0F;
	int need_read_len = 0, one_data_bytes = 0;
	int actual_len = len - 5;
	int uart_len;
	u8 *uart_buf, *total_buf;

	ipio_debug("data[3] = %x, type = %x, actual_len = %d\n",
					buf[3], type, actual_len);

	need_read_len = buf[1] * buf[2];

	if (type == 0 || type == 1 || type == 6) {
		one_data_bytes = 1;
	} else if (type == 2 || type == 3) {
		one_data_bytes = 2;
	} else if (type == 4 || type == 5) {
		one_data_bytes = 4;
	}

	need_read_len =  need_read_len * one_data_bytes + 1;
	ipio_debug("need_read_len = %d  one_data_bytes = %d\n", need_read_len, one_data_bytes);

	if (need_read_len > actual_len) {
		uart_len = need_read_len - actual_len;
		ipio_debug("uart len = %d\n", uart_len);

		uart_buf = kcalloc(uart_len, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(uart_buf)) {
			ipio_err("Failed to allocate uart_buf memory %ld\n", PTR_ERR(uart_buf));
			return;
		}

		if (idev_V2->read(uart_buf, uart_len) < 0) {
			ipio_err("i2cuart read data failed\n");
			return;
		}

		total_buf = kcalloc(len + uart_len, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(total_buf)) {
			ipio_err("Failed to allocate total_buf memory %ld\n", PTR_ERR(total_buf));
			return;
		}
		memcpy(total_buf, buf, len);
		memcpy(total_buf + len, uart_buf, uart_len);
		ilitek_tddi_touch_send_debug_data(total_buf, len + uart_len);
		return;
	}
	ilitek_tddi_touch_send_debug_data(buf, len);
}
