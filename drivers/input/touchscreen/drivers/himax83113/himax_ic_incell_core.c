#include "himax_ic_core.h"
#include "himax_common.h"
#if defined(HX_ZERO_FLASH)
	extern char i_CTPM_firmware_name[256];
#endif
#ifdef HX_ZERO_FLASH
extern int g_f_0f_updat;
int g_cfg_crc = -1;
int g_cfg_sz = -1;
int sense_on_flag = 0;
#endif

extern unsigned long FW_VER_MAJ_FLASH_ADDR;
extern unsigned long FW_VER_MIN_FLASH_ADDR;
extern unsigned long CFG_VER_MAJ_FLASH_ADDR;
extern unsigned long CFG_VER_MIN_FLASH_ADDR;
extern unsigned long CID_VER_MAJ_FLASH_ADDR;
extern unsigned long CID_VER_MIN_FLASH_ADDR;

extern unsigned long FW_VER_MAJ_FLASH_LENG;
extern unsigned long FW_VER_MIN_FLASH_LENG;
extern unsigned long CFG_VER_MAJ_FLASH_LENG;
extern unsigned long CFG_VER_MIN_FLASH_LENG;
extern unsigned long CID_VER_MAJ_FLASH_LENG;
extern unsigned long CID_VER_MIN_FLASH_LENG;

extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern unsigned char IC_CHECKSUM;
extern s64 timeStart, timeEnd;;


#ifdef HX_ESD_RECOVERY
extern int g_zero_event_count;
#endif

#ifdef HX_RST_PIN_FUNC
	extern u8 HX_HW_RESET_ACTIVATE;
	extern void himax_rst_gpio_set(int pinnum, uint8_t value);
#endif

extern int himax_report_data_init(void);
extern int xfer_error_count;

struct himax_core_command_operation *g_core_cmd_op;
struct ic_operation *pic_op;
struct fw_operation *pfw_op;
struct flash_operation *pflash_op;
struct sram_operation *psram_op;
struct driver_operation *pdriver_op;
#ifdef HX_TP_PROC_GUEST_INFO
struct hx_guest_info *g_guest_info_data;
#endif
#ifdef HX_ZERO_FLASH
struct zf_operation *pzf_op;
#endif

static uint8_t *g_internal_buffer;

extern struct himax_core_fp g_core_fp;
/*VIVO start*/
#define CONFIG_VERSION 1
#define FW_VERSION_HIMAX 2

unsigned char WriteBuffer[4*1024] = {0};
/*==============================================VIVO======================================================*/

#if 0
int Calculate_4k_CRC_value(unsigned char *FW_content)
{
/* 	uint8_t tmp_data[4]; */
	int i, j;
	int fw_data;
	int fw_data_2;
	int CRC = 0xFFFFFFFF;
	int PolyNomial = 0x82F63B78;
	int length = 1024; /* 4k */

	for (i = 0; i < length; i++) {
		fw_data = FW_content[i * 4];
		for (j = 1; j < 4; j++) {
			fw_data_2 = FW_content[i * 4 + j];
			fw_data += (fw_data_2) << (8 * j);
		}

		CRC = fw_data ^ CRC;

		for (j = 0; j < 32; j++) {
			if ((CRC % 2) != 0) {
				CRC = ((CRC >> 1) & 0x7FFFFFFF) ^ PolyNomial;
			} else {
				CRC = (((CRC >> 1) & 0x7FFFFFFF)/*& 0x7FFFFFFF*/);
			}
		}
	}

	I("%s: CRC calculate from bin file is %x \n", __func__, CRC);

	return CRC;
}
#endif
int bbk_himax_set_charger_bit(struct vts_device *vtsdev,int state) /*#8*/
{
	int result = -1;

	if (private_ts->chip_name == 0) {
		I("%s, read ic id failed \n", __func__);
		return MINUS_ONE;
	}

	result = g_core_fp.fp_usb_detect_set(state);

	return result;
}
#if 0
int bbk_himax_read_udd(unsigned char *udd) /*#4-1*/
{
	uint8_t *dump_buf; /* read 128 bytes */
	int start_addr = 64 * 1024;

	dump_buf = kzalloc(128 * sizeof(uint8_t), GFP_KERNEL);
	if (!dump_buf)
		return -ENOMEM;
	g_core_fp.fp_flash_dump_func(0, 128, dump_buf, start_addr);
	memcpy(udd, dump_buf, 32); /* assume info is 32bytes */

	kfree(dump_buf);
	return 0;
}

int bbk_himax_write_udd(unsigned char *udd)  /*#4-2*/
{
	int WriteSize = 4 * 1024;
	uint8_t CRC_addr[4];
	int CRC_value = 0;
	int start_addr = 64 * 1024;
	int ret = 0;

	/* 1. Calculate CRC checksum(4bytes) for the customer info */
	memcpy(WriteBuffer, udd, 32); /* assume info is 32bytes */

	CRC_value = Calculate_4k_CRC_value(WriteBuffer);

	WriteBuffer[WriteSize - 4] = (uint8_t)(CRC_value >> 24);
	WriteBuffer[WriteSize - 3] = (uint8_t)(CRC_value >> 16);
	WriteBuffer[WriteSize - 2] = (uint8_t)(CRC_value >> 8);
	WriteBuffer[WriteSize - 1] = (uint8_t) CRC_value;

	/* 2.Sector erase for 4k */
	ret = g_core_fp.fp_sector_erase(start_addr);
	if (ret == false) {
		E("Sector erase fail!\n");
		return 0;
	}

	/* 3.Program for 4k */
	g_core_fp.fp_flash_programming(WriteBuffer, WriteSize);

	/* 4.HW CRC checksum */
	CRC_addr[3] = (uint8_t)(start_addr >> 24);
	CRC_addr[2] = (uint8_t)(start_addr >> 16);
	CRC_addr[1] = (uint8_t)(start_addr >> 8);
	CRC_addr[0] = (uint8_t) start_addr;

	ret = g_core_fp.fp_check_CRC(CRC_addr, WriteSize);
	if (ret != 0) {
		E("CRC checksum fail! CRC = %x\n", ret);
	}
	return ret;
}
#endif

/*==============================================VIVO END===================================================*/

#ifdef CORE_IC
/* IC side start*/
static void himax_mcu_burst_enable(uint8_t auto_add_4_byte)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	/*I("%s,Entering \n",__func__);*/
	tmp_data[0] = pic_op->data_conti[0];

	if (himax_bus_write(pic_op->addr_conti[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return;
	}

	tmp_data[0] = (pic_op->data_incr4[0] | auto_add_4_byte);

	if (himax_bus_write(pic_op->addr_incr4[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return;
	}
}

static int himax_mcu_register_read(uint8_t *read_addr, uint32_t read_length, uint8_t *read_data, uint8_t cfg_flag)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int i = 0;
	int address = 0;

	/*I("%s,Entering \n",__func__);*/

	if (cfg_flag == false) {
		/*if (read_length > FLASH_RW_MAX_LEN) {
			E("%s: read len over %d!\n", __func__, FLASH_RW_MAX_LEN);
			return LENGTH_FAIL;
		}*/

		if (read_length > DATA_LEN_4) {
			g_core_fp.fp_burst_enable(1);
		} else {
			g_core_fp.fp_burst_enable(0);
		}

		address = (read_addr[3] << 24) + (read_addr[2] << 16) + (read_addr[1] << 8) + read_addr[0];
		i = address;
		tmp_data[0] = (uint8_t)i;
		tmp_data[1] = (uint8_t)(i >> 8);
		tmp_data[2] = (uint8_t)(i >> 16);
		tmp_data[3] = (uint8_t)(i >> 24);

		if (himax_bus_write(pic_op->addr_ahb_addr_byte_0[0], tmp_data, DATA_LEN_4, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return XFER_FAIL;
		}

		tmp_data[0] = pic_op->data_ahb_access_direction_read[0];

		if (himax_bus_write(pic_op->addr_ahb_access_direction[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return XFER_FAIL;
		}

		if (himax_bus_read(pic_op->addr_ahb_rdata_byte_0[0], read_data, read_length, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return XFER_FAIL;
		}

		if (read_length > DATA_LEN_4) {
			g_core_fp.fp_burst_enable(0);
		}
	} else {
		if (himax_bus_read(read_addr[0], read_data, read_length, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return XFER_FAIL;
		}
	}
	return NO_ERR;
}

/*
static int himax_mcu_flash_write_burst(uint8_t *reg_byte, uint8_t *write_data)
{
	uint8_t data_byte[FLASH_WRITE_BURST_SZ];
	int i = 0, j = 0;
	int data_byte_sz = sizeof(data_byte);

	for (i = 0; i < ADDR_LEN_4; i++) {
		data_byte[i] = reg_byte[i];
	}

	for (j = ADDR_LEN_4; j < data_byte_sz; j++) {
		data_byte[j] = write_data[j - ADDR_LEN_4];
	}

	if (himax_bus_write(pic_op->addr_ahb_addr_byte_0[0], data_byte, data_byte_sz, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return XFER_FAIL;
	}
	return NO_ERR;
}*/

static int himax_mcu_flash_write_burst_lenth(uint8_t *reg_byte, uint8_t *write_data, uint32_t length)
{
	uint8_t *data_byte;
	/*int i = 0, j = 0;*/

	/* if (length + ADDR_LEN_4 > FLASH_RW_MAX_LEN) {
		E("%s: write len over %d!\n", __func__, FLASH_RW_MAX_LEN);
		return;
	} */

	if (!g_internal_buffer) {
		E("%s: internal buffer not initialized!\n", __func__);
		return MEM_ALLOC_FAIL;
	}
	data_byte = g_internal_buffer;

	/*for (i = 0; i < ADDR_LEN_4; i++) {
		data_byte[i] = reg_byte[i];
	}*/
	memcpy(data_byte, reg_byte, ADDR_LEN_4); /* assign addr 4bytes */

	/*for (j = ADDR_LEN_4; j < length + ADDR_LEN_4; j++) {
		data_byte[j] = write_data[j - ADDR_LEN_4];
	}*/
	memcpy(data_byte + ADDR_LEN_4, write_data, length); /* assign data n bytes */

	if (himax_bus_write(pic_op->addr_ahb_addr_byte_0[0], data_byte, length + ADDR_LEN_4, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return XFER_FAIL;
	}

	return NO_ERR;
}

static int himax_mcu_register_write(uint8_t *write_addr, uint32_t write_length, uint8_t *write_data, uint8_t cfg_flag)
{
	int address;
	uint8_t tmp_addr[4];
	uint8_t *tmp_data;

	int total_read_times = 0;
	uint32_t max_bus_size = 128;
	uint32_t total_size_temp = 0;
	int i =0;

	/*I("%s,Entering \n", __func__);*/
	if (cfg_flag == 0) {
		total_size_temp = write_length;
		if (write_length > (HX_SPI_MTK_MAX_WRITE_SZ - 4)) {
			max_bus_size = HX_SPI_MTK_MAX_WRITE_SZ - 4;
		} else {
			max_bus_size = write_length;
		}

		tmp_addr[3] = write_addr[3];
		tmp_addr[2] = write_addr[2];
		tmp_addr[1] = write_addr[1];
		tmp_addr[0] = write_addr[0];

		if (total_size_temp % max_bus_size == 0) {
			total_read_times = total_size_temp / max_bus_size;
		} else {
			total_read_times = total_size_temp / max_bus_size + 1;
		}
		address = (write_addr[3] << 24) + (write_addr[2] << 16) + (write_addr[1] << 8) + write_addr[0];

		if (write_length > DATA_LEN_4) {
			g_core_fp.fp_burst_enable(1);
		} else {
			g_core_fp.fp_burst_enable(0);
		}

		for (i = 0; i < (total_read_times); i++) {
			/*I("[log]write %d time start!\n", i);
			I("[log]addr[3]=0x%02X, addr[2]=0x%02X, addr[1]=0x%02X, addr[0]=0x%02X!\n", tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);*/
			//I("%s, write addr = 0x%02X%02X%02X%02X\n", __func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

			if (total_size_temp >= max_bus_size) {
				tmp_data = write_data+(i * max_bus_size);
				if (himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, max_bus_size) < 0) {
					I("%s: i2c access fail!\n", __func__);
					return XFER_FAIL;
				}
				total_size_temp = total_size_temp - max_bus_size;
			} else {
				tmp_data = write_data+(i * max_bus_size);
				// I("last total_size_temp=%d\n", total_size_temp % max_bus_size);
				if (himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, max_bus_size) < 0) {
					I("%s: i2c access fail!\n", __func__);
					return XFER_FAIL;
				}
			}

			/*I("[log]write %d time end!\n", i);*/
			address = ((i+1) * max_bus_size);
			tmp_addr[0] = write_addr[0] + (uint8_t) ((address) & 0x00FF);

			if (tmp_addr[0] <  write_addr[0]) {
				tmp_addr[1] = write_addr[1] + (uint8_t) ((address>>8) & 0x00FF) + 1;
			} else {
				tmp_addr[1] = write_addr[1] + (uint8_t) ((address>>8) & 0x00FF);
			}

			udelay (100);
		}
	} else if (cfg_flag == 1) {
		if (himax_bus_write(write_addr[0], write_data, write_length, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return XFER_FAIL;
		}
	} else {
		E("%s: cfg_flag = %d, value is wrong!\n", __func__, cfg_flag);
	}

	return NO_ERR;
}

static int himax_write_read_reg(uint8_t *tmp_addr, uint8_t *tmp_data, uint8_t hb, uint8_t lb)
{
	int cnt = 0;

	do {
		g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
		msleep(10);
		g_core_fp.fp_register_read(tmp_addr, 4, tmp_data, 0);
		/* I("%s:Now tmp_data[0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n",
		 __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);*/
	} while ((tmp_data[1] != hb && tmp_data[0] != lb) && cnt++ < 100);

	if (cnt == 99) {
		return HX_RW_REG_FAIL;
	}

	I("Now register 0x%08X : high byte=0x%02X,low byte=0x%02X\n", tmp_addr[3], tmp_data[1], tmp_data[0]);
	return NO_ERR;
}

static void himax_mcu_interface_on(void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t tmp_data2[DATA_LEN_4] = {0};
	int cnt = 0;

	/* Read a dummy register to wake up Xfer bus.*/
	if (himax_bus_read(pic_op->addr_ahb_rdata_byte_0[0], tmp_data, DATA_LEN_4, HIMAX_XFER_RETRY_TIMES) < 0) {/* to knock bus*/
		E("%s: xfer fail!\n", __func__);
		return;
	}

	do {
		tmp_data[0] = pic_op->data_conti[0];

		if (himax_bus_write(pic_op->addr_conti[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return;
		}

		tmp_data[0] = pic_op->data_incr4[0];

		if (himax_bus_write(pic_op->addr_incr4[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return;
		}

		/*Check cmd*/
		himax_bus_read(pic_op->addr_conti[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES);
		himax_bus_read(pic_op->addr_incr4[0], tmp_data2, 1, HIMAX_XFER_RETRY_TIMES);

		if (tmp_data[0] == pic_op->data_conti[0] && tmp_data2[0] == pic_op->data_incr4[0]) {
			break;
		}

		msleep(1);
	} while (++cnt < 10);

	if (cnt > 0) {
		I("%s:Polling burst mode: %d times\n", __func__, cnt);
	}
}

static bool himax_mcu_wait_wip(int Timing)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int retry_cnt = 0;

	g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_fmt, DATA_LEN_4, pflash_op->data_spi200_trans_fmt, 0);
	tmp_data[0] = 0x01;

	do {
		g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_1, 0);

		g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_1, 0);
		tmp_data[0] = tmp_data[1] = tmp_data[2] = tmp_data[3] = 0xFF;
		g_core_fp.fp_register_read(pflash_op->addr_spi200_data, 4, tmp_data, 0);

		if ((tmp_data[0] & 0x01) == 0x00) {
			return true;
		}

		retry_cnt++;

		if (tmp_data[0] != 0x00 || tmp_data[1] != 0x00 || tmp_data[2] != 0x00 || tmp_data[3] != 0x00)
			I("%s:Wait wip retry_cnt:%d, buffer[0]=%d, buffer[1]=%d, buffer[2]=%d, buffer[3]=%d \n",
			  __func__, retry_cnt, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

		if (retry_cnt > 100) {
			E("%s: Wait wip error!\n", __func__);
			return false;
		}

		msleep(Timing);
	} while ((tmp_data[0] & 0x01) == 0x01);

	return true;
}

static void himax_mcu_sense_on(uint8_t FlashMode)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int retry = 0;
	I("Enter %s \n", __func__);
	g_core_fp.fp_interface_on();

	if (!FlashMode) {
#ifdef HX_RST_PIN_FUNC
		/*g_core_fp.fp_ic_reset(false, false);*///12.03
		/*===========================================
		 password[7:0] set Enter safe mode : 0x31 ==> 0x27
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_lb[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("%s: xfer fail!\n", __func__);

		/*===========================================
		 password[15:8] set Enter safe mode :0x32 ==> 0x95
		===========================================*/
		tmp_data[0] = pic_op->data_sonoff_psw_ub[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("%s: xfer fail!\n", __func__);


		if (sense_on_flag) {
			/*===========================================
			 password[7:0] set Enter safe mode : 0x31 ==> 0x00
			===========================================*/
			tmp_data[0] = 0x00;

			if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
				E("%s: xfer fail!\n", __func__);

		}
		sense_on_flag = 1;

#else
		g_core_fp.fp_system_reset();
#endif
	} else {
		do {
			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_active), pfw_op->data_safe_mode_release_pw_active, 0);

			g_core_fp.fp_register_read(pfw_op->addr_flag_reset_event, DATA_LEN_4, tmp_data, 0);
			I("%s:Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
		} while ((tmp_data[1] != 0x01 || tmp_data[0] != 0x00) && retry++ < 5);

		if (retry >= 5) {
			E("%s: Fail:\n", __func__);
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#else
			g_core_fp.fp_system_reset();
#endif
		} else {
			I("%s:OK and Read status from IC = %X,%X\n", __func__, tmp_data[0], tmp_data[1]);
			/* reset code*/
			tmp_data[0] = 0x00;

			if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
			}

			if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
			}

			g_core_fp.fp_register_write(pfw_op->addr_safe_mode_release_pw,
				sizeof(pfw_op->data_safe_mode_release_pw_reset), pfw_op->data_safe_mode_release_pw_reset, 0);
		}
	}
}

static bool himax_mcu_sense_off(bool check_en)
{
	uint8_t cnt = 0;
	uint8_t tmp_data[DATA_LEN_4] = {0};

	do {
		tmp_data[0] = pic_op->data_sonoff_psw_lb[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		tmp_data[0] = pic_op->data_sonoff_psw_ub[0];

		if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return false;
		}

		g_core_fp.fp_register_read(pic_op->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);
		I("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);

		if (tmp_data[0] == 0x0C) {
			g_core_fp.fp_register_write(pic_op->addr_tcon_on_rst, DATA_LEN_4, pic_op->data_rst, 0);
			msleep(1);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(pic_op->addr_tcon_on_rst, DATA_LEN_4, tmp_data, 0);

			g_core_fp.fp_register_write(pic_op->addr_adc_on_rst, DATA_LEN_4, pic_op->data_rst, 0);
			msleep(1);
			tmp_data[3] = pic_op->data_rst[3];
			tmp_data[2] = pic_op->data_rst[2];
			tmp_data[1] = pic_op->data_rst[1];
			tmp_data[0] = pic_op->data_rst[0] | 0x01;
			g_core_fp.fp_register_write(pic_op->addr_adc_on_rst, DATA_LEN_4, tmp_data, 0);
			return true;
		} else {
			msleep(10);
#ifdef HX_RST_PIN_FUNC
			g_core_fp.fp_ic_reset(false, false);
#endif
		}
	} while (cnt++ < 15);

	return false;
}

static void himax_mcu_init_psl(void) /*power saving level*/
{
	g_core_fp.fp_register_write(pic_op->addr_psl, sizeof(pic_op->data_rst), pic_op->data_rst, 0);
	I("%s: power saving level reset OK!\n", __func__);
}

static void himax_mcu_resume_ic_action(void)
{
	/* Nothing to do */
}

static void himax_mcu_suspend_ic_action(void)
{
	/* Nothing to do */
}

static void himax_mcu_power_on_init(void)
{
	I("%s:\n", __func__);
	g_core_fp.fp_touch_information();
	/*RawOut select initial*/
	g_core_fp.fp_register_write(pfw_op->addr_raw_out_sel, sizeof(pfw_op->data_clear), pfw_op->data_clear, 0);
	/*DSRAM func initial*/
	g_core_fp.fp_assign_sorting_mode(pfw_op->data_clear);
	if (private_ts->chip_name == HX_83112A_SERIES_PWON) {
		g_core_fp.fp_sense_on(0x00);
	}
}

/* IC side end*/
#endif

#ifdef CORE_FW
/* FW side start*/
static void diag_mcu_parse_raw_data(struct himax_report_data *hx_touch_data, int mul_num, int self_num, uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data)
{
	int RawDataLen_word;
	int index = 0;
	int temp1, temp2, i;

	if (hx_touch_data->hx_rawdata_buf[0] == pfw_op->data_rawdata_ready_lb[0]
	    && hx_touch_data->hx_rawdata_buf[1] == pfw_op->data_rawdata_ready_hb[0]
	    && hx_touch_data->hx_rawdata_buf[2] > 0
	    && hx_touch_data->hx_rawdata_buf[3] == diag_cmd) {
		RawDataLen_word = hx_touch_data->rawdata_size / 2;
		index = (hx_touch_data->hx_rawdata_buf[2] - 1) * RawDataLen_word;

		/* I("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
		 I("RawDataLen=%d , RawDataLen_word=%d , hx_touch_info_size=%d\n", RawDataLen, RawDataLen_word, hx_touch_info_size);*/
		for (i = 0; i < RawDataLen_word; i++) {
			temp1 = index + i;

			if (temp1 < mul_num) { /*mutual*/
				mutual_data[index + i] = ((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) * 256 + hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			} else { /*self*/
				temp1 = i + index;
				temp2 = self_num + mul_num;

				if (temp1 >= temp2) {
					break;
				}

				self_data[i + index - mul_num] = (((int8_t)hx_touch_data->hx_rawdata_buf[i * 2 + 4 + 1]) << 8) +
				hx_touch_data->hx_rawdata_buf[i * 2 + 4];
			}
		}
	}
}

static void himax_mcu_system_reset(void)
{
	uint8_t tmp_data[DATA_LEN_4];
	int retry = 0;

	g_core_fp.fp_interface_on();
	//g_core_fp.fp_register_write(pfw_op->addr_ctrl_fw_isr, sizeof(pfw_op->data_clear), pfw_op->data_clear, 0);

	do {
		/* reset code */
		tmp_data[0] = pic_op->data_sonoff_psw_lb[0];
		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("spi access failed!\n");
		
		tmp_data[0] = pic_op->data_sonoff_psw_ub[0];
		if (himax_bus_write(pic_op->adr_sonoff_psw_ub[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("spi access failed!\n");
		
		tmp_data[0] = 0x00;
		if (himax_bus_write(pic_op->adr_sonoff_psw_lb[0], tmp_data, 1, HIMAX_XFER_RETRY_TIMES) < 0)
			E("spi access failed!\n");
		
		usleep_range(10000, 11000);
		
		g_core_fp.fp_register_read(pfw_op->addr_flag_reset_event, DATA_LEN_4, tmp_data, 0);
		I("%s: Read status from IC = %x, %x\n", __func__, tmp_data[0], tmp_data[1]);
	} while ((tmp_data[1] != 0x02 || tmp_data[0] != 0x00) && retry++ < 5);
	
	//g_core_fp.fp_register_write(pfw_op->addr_system_reset, sizeof(pfw_op->data_system_reset), pfw_op->data_system_reset, 0);
}

static int himax_mcu_Calculate_CRC_with_AP(unsigned char *FW_content, int CRC_from_FW, int len)
{
    int i, j, length = 0;
    int fw_data;
    int fw_data_2;
    int CRC = 0xFFFFFFFF;
    int PolyNomial = 0x82F63B78;

		length = len / 4;

    for (i = 0; i < length; i++) {
			fw_data = FW_content[i * 4];

			for (j = 1; j < 4; j++) {
				fw_data_2 = FW_content[i * 4 + j];
				fw_data += (fw_data_2) << (8 * j);
			}

			CRC = fw_data ^ CRC;

			for (j = 0; j < 32; j++) {
				if ((CRC % 2) != 0)
					CRC = ((CRC >> 1) & 0x7FFFFFFF) ^ PolyNomial;
				else
					CRC = (((CRC >> 1) & 0x7FFFFFFF)/*& 0x7FFFFFFF*/);
				}
			/*I("CRC = %x , i = %d \n", CRC, i);*/
    }

		return CRC;
}

static uint32_t himax_mcu_check_CRC(uint8_t *start_addr, int reload_length)
{
	uint32_t result = 0;
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int cnt = 0, ret = 0;
	int length = reload_length / DATA_LEN_4;

	ret = g_core_fp.fp_register_write(pfw_op->addr_reload_addr_from, DATA_LEN_4, start_addr, 0);
	if (ret < NO_ERR) {
		E("%s: xfer fail!\n", __func__);
		return HW_CRC_FAIL;
	}

	tmp_data[3] = 0x00; tmp_data[2] = 0x99; tmp_data[1] = (length >> 8); tmp_data[0] = length;
	ret = g_core_fp.fp_register_write(pfw_op->addr_reload_addr_cmd_beat, DATA_LEN_4, tmp_data, 0);
	if (ret < NO_ERR) {
		E("%s: xfer fail!\n", __func__);
		return HW_CRC_FAIL;
	}
	cnt = 0;

	do {
		ret = g_core_fp.fp_register_read(pfw_op->addr_reload_status, DATA_LEN_4, tmp_data, 0);
		if (ret < NO_ERR) {
			E("%s: xfer fail!\n", __func__);
			return HW_CRC_FAIL;
		}

		if ((tmp_data[0] & 0x01) != 0x01) {
			ret = g_core_fp.fp_register_read(pfw_op->addr_reload_crc32_result, DATA_LEN_4, tmp_data, 0);
			if (ret < NO_ERR) {
				E("%s: xfer fail!\n", __func__);
				return HW_CRC_FAIL;
			}
			/*I("%s: tmp_data[3]=%X, tmp_data[2]=%X, tmp_data[1]=%X, tmp_data[0]=%X  \n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);*/
			result = ((tmp_data[3] << 24) + (tmp_data[2] << 16) + (tmp_data[1] << 8) + tmp_data[0]);
			break;
		} else {
			I("Waiting for HW ready!\n");
			msleep(1);
		}

	} while (cnt++ < 100);

	return result;
}

static void himax_mcu_set_reload_cmd(uint8_t *write_data, int idx, uint32_t cmd_from, uint32_t cmd_to, uint32_t cmd_beat)
{
	int index = idx * 12;
	int i;

	for (i = 3; i >= 0; i--) {
		write_data[index + i] = (cmd_from >> (8 * i));
		write_data[index + 4 + i] = (cmd_to >> (8 * i));
		write_data[index + 8 + i] = (cmd_beat >> (8 * i));
	}
}

static bool himax_mcu_program_reload(void)
{
	return true;
}

static void himax_mcu_set_SMWP_enable(uint8_t SMWP_enable, bool suspended)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t back_data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	do {
		if (SMWP_enable) {
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_smwp_enable, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, back_data, 4);
		} else {
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_smwp_enable, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, back_data, 4);
		}

		g_core_fp.fp_register_read(pfw_op->addr_smwp_enable, DATA_LEN_4, tmp_data, 0);
		/*I("%s: tmp_data[0]=%d, SMWP_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0],SMWP_enable,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static void himax_mcu_set_HSEN_enable(uint8_t HSEN_enable, bool suspended)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t back_data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	do {
		if (HSEN_enable) {
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_hsen_enable, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, back_data, 4);
		} else {
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_hsen_enable, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, back_data, 4);
		}

		g_core_fp.fp_register_read(pfw_op->addr_hsen_enable, DATA_LEN_4, tmp_data, 0);
		/*I("%s: tmp_data[0]=%d, HSEN_enable=%d, retry_cnt=%d \n", __func__, tmp_data[0],HSEN_enable,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);
}

static int himax_mcu_usb_detect_set(int cable_config)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t back_data[DATA_LEN_4] = {0};
	uint8_t retry_cnt = 0;

	do {
		if (cable_config == 0x01) {
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_usb_detect, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_func_handshaking_pwd, back_data, 4);
			I("%s: USB detect status IN!\n", __func__);
		} else {
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, tmp_data, 4);
			g_core_fp.fp_register_write(pfw_op->addr_usb_detect, DATA_LEN_4, tmp_data, 0);
			himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, back_data, 4);
			I("%s: USB detect status OUT!\n", __func__);
		}

		g_core_fp.fp_register_read(pfw_op->addr_usb_detect, DATA_LEN_4, tmp_data, 0);
		/*I("%s: tmp_data[0]=%d, USB detect=%d, retry_cnt=%d \n", __func__, tmp_data[0],cable_config[1] ,retry_cnt);*/
		retry_cnt++;
	} while ((tmp_data[3] != back_data[3] || tmp_data[2] != back_data[2] || tmp_data[1] != back_data[1]  || tmp_data[0] != back_data[0]) && retry_cnt < HIMAX_REG_RETRY_TIMES);

	//return cable_config;
	return 0;
}

static void himax_mcu_diag_register_set(uint8_t diag_command, uint8_t storage_type)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t back_data[DATA_LEN_4] = {0};
	uint8_t cnt = 50;

	if (diag_command > 0 && storage_type % 8 > 0)
		tmp_data[0] = diag_command + 0x08;
	else
		tmp_data[0] = diag_command;
	I("diag_command = %d, tmp_data[0] = %X\n", diag_command, tmp_data[0]);
	g_core_fp.fp_interface_on();
	tmp_data[3] = 0x00; tmp_data[2] = 0x00; tmp_data[1] = 0x00;
	do {
		g_core_fp.fp_register_write(pfw_op->addr_raw_out_sel, DATA_LEN_4, tmp_data, 0);
		g_core_fp.fp_register_read(pfw_op->addr_raw_out_sel, DATA_LEN_4, back_data, 0);
		I("%s: back_data[3]=0x%02X,back_data[2]=0x%02X,back_data[1]=0x%02X,back_data[0]=0x%02X!\n",
		  __func__, back_data[3], back_data[2], back_data[1], back_data[0]);
		cnt--;
	} while (tmp_data[0] != back_data[0] && cnt > 0);
}

static int himax_check_remapping(void)
{
    uint8_t cmd[4];
    uint8_t data[64];
    uint8_t data2[64];
    int retry = 50;
    int reload_status = 1;

    while(reload_status == 1) {
        cmd[3] = 0x10;
        cmd[2] = 0x00;
        cmd[1] = 0x7f;
        cmd[0] = 0x00;
        g_core_fp.fp_register_read(cmd, 4, data, false);

        cmd[3] = 0x10;
        cmd[2] = 0x00;
        cmd[1] = 0x72;
        cmd[0] = 0xc0;
        g_core_fp.fp_register_read(cmd, 4, data2, false);
		
		I("%s reload after 0x10007f00: data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);
		I("%s reload after 0x100072c0: data2[0] = 0x%2.2X, data2[1] = 0x%2.2X, data2[2] = 0x%2.2X, data2[3] = 0x%2.2X\n", __func__, data2[0], data2[1], data2[2], data2[3]);

        if ((data[2] == 0x9A && data[3] == 0xA9) && (data2[1] == 0x72 && data2[0] == 0xc0)) {
            I("reload OK! \n");
            reload_status = 0;
            break;
        } else if (retry == 0) {
            I("reload 20 times! fail \n");
            break;
        } else {
            retry--;
	    usleep_range(3000,3001);
	    
            I("reload fail, delay 10ms retry=%d\n", retry);
        }
    }
	I("%s: 0x10007f00 data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, data[0], data[1], data[2], data[3]);
	I("%s : 0x100072c0 data2[0] = 0x%2.2X, data2[1] = 0x%2.2X, data2[2] = 0x%2.2X, data2[3] = 0x%2.2X\n", __func__, data2[0], data2[1], data2[2], data2[3]);
    I("reload_status=%d\n", reload_status);
    return reload_status;
}


static void himax_mcu_idle_mode(int disable)
{
	int retry = 20;
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t switch_cmd = 0x00;

	I("%s:entering\n", __func__);
	do {
		I("%s,now %d times!\n", __func__, retry);
		g_core_fp.fp_register_read(pfw_op->addr_fw_mode_status, DATA_LEN_4, tmp_data, 0);

		if (disable)
			switch_cmd = pfw_op->data_idle_dis_pwd[0];
		else
			switch_cmd = pfw_op->data_idle_en_pwd[0];

		tmp_data[0] = switch_cmd;
		g_core_fp.fp_register_write(pfw_op->addr_fw_mode_status, DATA_LEN_4, tmp_data, 0);
		g_core_fp.fp_register_read(pfw_op->addr_fw_mode_status, DATA_LEN_4, tmp_data, 0);
		I("%s:After turn ON/OFF IDLE Mode [0] = 0x%02X,[1] = 0x%02X,[2] = 0x%02X,[3] = 0x%02X\n",
		  __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
		retry--;
		msleep(10);
	} while ((tmp_data[0] != switch_cmd) && retry > 0);

	I("%s: setting OK!\n", __func__);
}

static void himax_reload_fw_clear_register(void){
    uint8_t tmp_data[4] = {0};
	uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
	  /* 100072c0 clear 00 */
	
	int retry = 0;

	do {
	  
		g_core_fp.fp_register_write(pdriver_op->addr_fw_define_2nd_flash_reload, DATA_LEN_4, data, 0);
		usleep_range(1000, 1001);
		g_core_fp.fp_register_read(pdriver_op->addr_fw_define_2nd_flash_reload, 4, tmp_data, false);
		
		//g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, rec_data, 0);

		I("%s: Now retry=%d,tmp_data[3]=0x%02X,tmp_data[2]=0x%02X,tmp_data[1]=0x%02X,tmp_data[0]=0x%02X\n",
				   __func__, retry, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
		
	} while((retry++ < 10) && (tmp_data[3] != 0x00 || tmp_data[2] != 0x00 || tmp_data[1] != 0x00 || tmp_data[0] != 0x00));

	I("%s: END !\n", __func__);

}


static void himax_mcu_reload_disable(int disable)
{
	I("%s:entering\n", __func__);

	if (disable) { /*reload disable*/
		g_core_fp.fp_register_write(pdriver_op->addr_fw_define_flash_reload, DATA_LEN_4, pdriver_op->data_fw_define_flash_reload_dis, 0);
	} else { /*reload enable*/
		g_core_fp.fp_register_write(pdriver_op->addr_fw_define_flash_reload, DATA_LEN_4, pdriver_op->data_fw_define_flash_reload_en, 0);
	}

	I("%s: setting OK!\n", __func__);
}

static bool himax_mcu_check_chip_version(void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t ret_data = false;
	int i = 0;

	for (i = 0; i < 5; i++) {
		g_core_fp.fp_register_read(pfw_op->addr_icid_addr, DATA_LEN_4, tmp_data, 0);
		I("%s:Read driver IC ID = %X,%X,%X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1]);

		if ((tmp_data[3] == 0x83) && (tmp_data[2] == 0x10) && (tmp_data[1] == 0x2a)) {
			//strlcpy(private_ts->chip_name, HX_83102A_SERIES_PWON, 30);
			ret_data = true;
			break;
		} else {
			ret_data = false;
			E("%s:Read driver ID register Fail:\n", __func__);
		}
	}

	return ret_data;
}

static int himax_mcu_read_ic_trigger_type(void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int trigger_type = false;
	g_core_fp.fp_register_read(pfw_op->addr_trigger_addr, DATA_LEN_4, tmp_data, 0);
	if ((tmp_data[1] & 0x01) == 1) {
		trigger_type = true;
	}

	return trigger_type;
}

static int himax_mcu_read_xfer_status(void)
{
	return xfer_error_count;
}

static void himax_mcu_read_FW_ver(void)
{
	uint8_t data[12];
	g_core_fp.fp_sense_on(0x00);
	if (g_core_fp.fp_check_remapping())
		return;
	/*
	 Read FW version
	 */
	g_core_fp.fp_sense_off(true);
	g_core_fp.fp_fw_sts_clear();
	g_core_fp.fp_register_read(pfw_op->addr_fw_ver_addr, DATA_LEN_4, data, 0);
	ic_data->vendor_panel_ver =  data[0];
	ic_data->vendor_fw_ver = data[1] << 8 | data[2];
	I("PANEL_VER : %X\n", ic_data->vendor_panel_ver);
	I("FW_VER : %X\n", ic_data->vendor_fw_ver);
	g_core_fp.fp_register_read(pfw_op->addr_fw_cfg_addr, DATA_LEN_4, data, 0);
	ic_data->vendor_config_ver = data[2] << 8 | data[3];
	/*I("CFG_VER : %X\n",ic_data->vendor_config_ver);*/
	ic_data->vendor_touch_cfg_ver = data[2];
	I("TOUCH_VER : %X\n", ic_data->vendor_touch_cfg_ver);
	ic_data->vendor_display_cfg_ver = data[3];
	I("DISPLAY_VER : %X\n", ic_data->vendor_display_cfg_ver);
	g_core_fp.fp_register_read(pfw_op->addr_fw_vendor_addr, DATA_LEN_4, data, 0);
	ic_data->vendor_cid_maj_ver = data[2] ;
	ic_data->vendor_cid_min_ver = data[3];
	I("CID_VER : %X\n", (ic_data->vendor_cid_maj_ver << 8 | ic_data->vendor_cid_min_ver));
	g_core_fp.fp_register_read(pfw_op->addr_cus_info, 12, data, 0);
	memcpy(ic_data->vendor_cus_info, data, 12);
	I("Cusomer ID = %s\n", ic_data->vendor_cus_info);
	g_core_fp.fp_register_read(pfw_op->addr_proj_info, 12, data, 0);
	memcpy(ic_data->vendor_proj_info, data, 12);
	I("Project ID = %s\n", ic_data->vendor_proj_info);
}

static bool himax_mcu_read_event_stack(uint8_t *buf, uint8_t length)
{
	uint8_t cmd[DATA_LEN_4];
	/*  AHB_I2C Burst Read Off */
	cmd[0] = pfw_op->data_ahb_dis[0];

	if (himax_bus_write(pfw_op->addr_ahb_addr[0], cmd, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return 0;
	}

	himax_bus_read(pfw_op->addr_event_addr[0], buf, length, HIMAX_XFER_RETRY_TIMES);
	/*  AHB_I2C Burst Read On */
	cmd[0] = pfw_op->data_ahb_en[0];

	if (himax_bus_write(pfw_op->addr_ahb_addr[0], cmd, 1, HIMAX_XFER_RETRY_TIMES) < 0) {
		E("%s: xfer fail!\n", __func__);
		return 0;
	}

	return 1;
}

static void himax_mcu_return_event_stack(void)
{
	int retry = 20, i;
	uint8_t tmp_data[DATA_LEN_4] = {0};

	I("%s:entering\n", __func__);
	do {
		I("now %d times!\n", retry);

		for (i = 0; i < DATA_LEN_4; i++) {
			tmp_data[i] = psram_op->addr_rawdata_end[i];
		}

		g_core_fp.fp_register_write(psram_op->addr_rawdata_addr, DATA_LEN_4, tmp_data, 0);
		g_core_fp.fp_register_read(psram_op->addr_rawdata_addr, DATA_LEN_4, tmp_data, 0);
		retry--;
		msleep(10);
	} while ((tmp_data[1] != psram_op->addr_rawdata_end[1] && tmp_data[0] != psram_op->addr_rawdata_end[0]) && retry > 0);

	I("%s: End of setting!\n", __func__);
}

static bool himax_mcu_calculateChecksum(bool change_iref)
{
	uint8_t CRC_result = 0, i;
	uint8_t tmp_data[DATA_LEN_4] = {0};

	for (i = 0; i < DATA_LEN_4; i++) {
		tmp_data[i] = psram_op->addr_rawdata_end[i];
	}

	CRC_result = g_core_fp.fp_check_CRC(tmp_data, FW_SIZE_64k);
	msleep(50);

	if (CRC_result != 0) {
		I("%s: CRC Fail=%d\n", __func__, CRC_result);
	}
	return (CRC_result == 0) ? true : false;
}

static int himax_mcu_read_FW_status(uint8_t *state_addr, uint8_t *tmp_addr)
{
	uint8_t i;
	uint8_t req_size = 0;
	uint8_t status_addr[DATA_LEN_4];
	uint8_t cmd_addr[DATA_LEN_4];

	if (state_addr[0] == 0x01) {
		state_addr[1] = 0x04;

		for (i = 0; i < DATA_LEN_4; i++) {
			state_addr[i + 2] = pfw_op->addr_fw_dbg_msg_addr[i];
			status_addr[i] = pfw_op->addr_fw_dbg_msg_addr[i];
		}

		req_size = 0x04;
		g_core_fp.fp_register_read(status_addr, req_size, tmp_addr, 0);
	} else if (state_addr[0] == 0x02) {
		state_addr[1] = 0x30;

		for (i = 0; i < DATA_LEN_4; i++) {
			state_addr[i + 2] = pfw_op->addr_fw_dbg_msg_addr[i];
			cmd_addr[i] = pfw_op->addr_fw_dbg_msg_addr[i];
		}

		req_size = 0x30;
		g_core_fp.fp_register_read(cmd_addr, req_size, tmp_addr, 0);
	}

	return NO_ERR;
}

static void himax_mcu_irq_switch(int switch_on)
{
	if (switch_on) {
		if (private_ts->use_irq)
			himax_int_enable(switch_on);
		else
			hrtimer_start(&private_ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	} else {
		if (private_ts->use_irq) {
			himax_int_enable(switch_on);
		} else {
			hrtimer_cancel(&private_ts->timer);
			cancel_work_sync(&private_ts->work);
		}
	}
}

static int himax_mcu_assign_sorting_mode(uint8_t *tmp_data)
{

	I("%s:Now tmp_data[3]=0x%02X,tmp_data[2]=0x%02X,tmp_data[1]=0x%02X,tmp_data[0]=0x%02X\n", __func__, tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);
	g_core_fp.fp_register_write(pfw_op->addr_sorting_mode_en, DATA_LEN_4, tmp_data, 0);

	return NO_ERR;
}

static int himax_mcu_check_sorting_mode(uint8_t *tmp_data)
{

	g_core_fp.fp_register_read(pfw_op->addr_sorting_mode_en, DATA_LEN_4, tmp_data, 0);
	I("%s: tmp_data[0]=%x,tmp_data[1]=%x\n", __func__, tmp_data[0], tmp_data[1]);

	return NO_ERR;
}

static int himax_mcu_switch_mode(int mode)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t mode_wirte_cmd;
	uint8_t mode_read_cmd;
	int result = -1;
	int retry = 200;

	I("%s: Entering\n", __func__);
	if (mode == 0) { /* normal mode */
		mode_wirte_cmd = pfw_op->data_normal_cmd[0];
		mode_read_cmd = pfw_op->data_normal_status[0];
	} else { /* sorting mode */
		mode_wirte_cmd = pfw_op->data_sorting_cmd[0];
		mode_read_cmd = pfw_op->data_sorting_status[0];
	}

	g_core_fp.fp_sense_off(true);
	/*g_core_fp.fp_interface_on();*/
	/* clean up FW status */
	g_core_fp.fp_register_write(psram_op->addr_rawdata_addr, DATA_LEN_4, psram_op->addr_rawdata_end, 0);
	tmp_data[3] = 0x00;
	tmp_data[2] = 0x00;
	tmp_data[1] = mode_wirte_cmd;
	tmp_data[0] = mode_wirte_cmd;
	g_core_fp.fp_assign_sorting_mode(tmp_data);
	g_core_fp.fp_idle_mode(1);
	g_core_fp.fp_reload_disable(1);

	/* To stable the sorting*/
	if (mode) {
		g_core_fp.fp_register_write(pdriver_op->addr_fw_define_rxnum_txnum_maxpt, DATA_LEN_4, pdriver_op->data_fw_define_rxnum_txnum_maxpt_sorting, 0);
	} else {
		g_core_fp.fp_register_write(pfw_op->addr_set_frame_addr, DATA_LEN_4, pfw_op->data_set_frame, 0);
		g_core_fp.fp_register_write(pdriver_op->addr_fw_define_rxnum_txnum_maxpt, DATA_LEN_4, pdriver_op->data_fw_define_rxnum_txnum_maxpt_normal, 0);
	}

	g_core_fp.fp_sense_on(0x01);

	while (retry != 0) {
		I("[%d] %s Read\n", retry, __func__);
		g_core_fp.fp_check_sorting_mode(tmp_data);
		msleep(100);
		I("mode_read_cmd(0)=0x%2.2X,mode_read_cmd(1)=0x%2.2X\n", tmp_data[0], tmp_data[1]);

		if (tmp_data[0] == mode_read_cmd && tmp_data[1] == mode_read_cmd) {
			I("Read OK!\n");
			result = 0;
			break;
		}

		g_core_fp.fp_register_read(pfw_op->addr_chk_fw_status, DATA_LEN_4, tmp_data, 0);

		if (tmp_data[0] == 0x00 && tmp_data[1] == 0x00 && tmp_data[2] == 0x00 && tmp_data[3] == 0x00) {
			E("%s,: FW Stop!\n", __func__);
			break;
		}

		retry--;
	}

	if (result == 0) {
		if (mode == 0) { /*normal mode*/
			return HX_NORMAL_MODE;
		} else { /*sorting mode*/
			return HX_SORTING_MODE;
		}
	} else { /*change mode fail*/
		return HX_CHANGE_MODE_FAIL;
	}
}

static uint8_t himax_mcu_read_DD_status(uint8_t *cmd_set, uint8_t *tmp_data)
{
	int cnt = 0;
	uint8_t req_size = cmd_set[0];

	cmd_set[3] = pfw_op->data_dd_request[0];
	g_core_fp.fp_register_write(pfw_op->addr_dd_handshak_addr, DATA_LEN_4, cmd_set, 0);
	I("%s: cmd_set[0] = 0x%02X,cmd_set[1] = 0x%02X,cmd_set[2] = 0x%02X,cmd_set[3] = 0x%02X\n",
	  __func__, cmd_set[0], cmd_set[1], cmd_set[2], cmd_set[3]);

	/* Doing hand shaking 0xAA -> 0xBB */
	for (cnt = 0; cnt < 100; cnt++) {
		g_core_fp.fp_register_read(pfw_op->addr_dd_handshak_addr, DATA_LEN_4, tmp_data, 0);
		msleep(10);

		if (tmp_data[3] == pfw_op->data_dd_ack[0]) {
			I("%s Data ready goto moving data\n", __func__);
			break;
		} else {
			if (cnt >= 99) {
				I("%s Data not ready in FW\n", __func__);
				return FW_NOT_READY;
			}
		}
	}

	g_core_fp.fp_register_read(pfw_op->addr_dd_data_addr, req_size, tmp_data, 0);
	return NO_ERR;
}

static void hx_fw_sts_clear(void)
{
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	g_core_fp.fp_register_read(pic_op->addr_cs_central_state, ADDR_LEN_4, tmp_data, 0);
	I("%s: Check enter_save_mode data[0]=%X \n", __func__, tmp_data[0]);
	
	if (tmp_data[0] == 0x0C) {
		I("%s: Enter safe mode, OK!\n", __func__);
	} else {
		E("%s: It doen't enter safe mode, please check it again\n", __func__);
		return;
	}

	himax_in_parse_assign_cmd(0x10007FCC, tmp_addr, sizeof(tmp_addr));
	g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, tmp_data, 0);
	I("Before Write %s:Now 10007FCC=0x%02X, 10007FCD=0x%02X, 10007FCE=0x%02X, 10007FCF=0x%02X\n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	usleep_range(10000, 10000);

	tmp_data[2] = 0x00;
	tmp_data[3] = 0x00;
	g_core_fp.fp_register_write(tmp_addr, DATA_LEN_4, tmp_data, 0);
	usleep_range(10000, 10000);

	g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, tmp_data, 0);
	I("After Write %s:Now 10007FCC=0x%02X, 10007FCD=0x%02X, 10007FCE=0x%02X, 10007FCF=0x%02X\n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);

}
/* FW side end*/
#endif

#ifdef CORE_FLASH
/* FLASH side start*/
static void himax_mcu_chip_erase(void)
{
	g_core_fp.fp_interface_on();

	/* Reset power saving level */
	if (g_core_fp.fp_init_psl != NULL) {
		g_core_fp.fp_init_psl();
	}

	g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_fmt, DATA_LEN_4, pflash_op->data_spi200_trans_fmt, 0);

	g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_2, 0);
	g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_2, 0);

	g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_3, 0);
	msleep(2000);

	if (!g_core_fp.fp_wait_wip(100))
		E("%s: Chip_Erase Fail\n", __func__);
}

static bool himax_mcu_block_erase(int start_addr, int length) /*complete not yet*/
{
	uint32_t page_prog_start = 0;
	uint32_t block_size = 0x10000;

	uint8_t tmp_data[DATA_LEN_4] = {0};

	g_core_fp.fp_interface_on();

	g_core_fp.fp_init_psl();

	g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_fmt, DATA_LEN_4, pflash_op->data_spi200_trans_fmt, 0);

	for (page_prog_start = start_addr; page_prog_start < start_addr + length; page_prog_start = page_prog_start + block_size) {
		g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_2, 0);
		g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_2, 0);

		tmp_data[3] = (page_prog_start >> 24)&0xFF;
		tmp_data[2] = (page_prog_start >> 16)&0xFF;
		tmp_data[1] = (page_prog_start >> 8)&0xFF;
		tmp_data[0] = page_prog_start&0xFF;
		g_core_fp.fp_register_write(pflash_op->addr_spi200_addr, DATA_LEN_4, tmp_data, 0);

		g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_3, 0);
		g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_4, 0);
		msleep(1000);

		if (!g_core_fp.fp_wait_wip(100)) {
			E("%s:Erase Fail\n", __func__);
			return false;
		}
	}

	I("%s:END\n", __func__);
	return true;
}
#if 0
static bool himax_mcu_sector_erase(int start_addr)
{
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[DATA_LEN_4] = {0};
	int page_prog_start = 0;

	g_core_fp.fp_burst_enable(0);

	/* =====================================
	 SPI Transfer Format : 0x8000_0010 ==> 0x0002_0780
	===================================== */
	tmp_addr[3] = 0x80;
	tmp_addr[2] = 0x00;
	tmp_addr[1] = 0x00;
	tmp_addr[0] = 0x10;

	tmp_data[3] = 0x00;
	tmp_data[2] = 0x02;
	tmp_data[1] = 0x07;
	tmp_data[0] = 0x80;
	himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);
	for (page_prog_start = start_addr; page_prog_start < start_addr
			+ 0x0F000; page_prog_start = page_prog_start + 0x1000) {
			/* =====================================
			 Chip Erase
			 Write Enable : 1. 0x8000_0020 ==> 0x4700_0000
							  2. 0x8000_0024 ==> 0x0000_0006
			===================================== */
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;

			tmp_data[3] = 0x47;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x24;

			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x06;
			himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			/* =====================================
			Sector Erase
			Erase Command : 0x8000_0028 ==> 0x0000_0000 SPI addr
			0x8000_0020 ==> 0x6700_0000 control
			0x8000_0024 ==> 0x0000_0020 SE
			===================================== */
			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x28;
			if (page_prog_start < 0x100) {
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = 0x00;
				tmp_data[0] = (uint8_t)page_prog_start;
			} else if (page_prog_start >= 0x100
					&& page_prog_start < 0x10000) {
				tmp_data[3] = 0x00;
				tmp_data[2] = 0x00;
				tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
			} else if (page_prog_start >= 0x10000
					&& page_prog_start < 0x1000000) {
				tmp_data[3] = 0x00;
				tmp_data[2] = (uint8_t)(page_prog_start >> 16);
				tmp_data[1] = (uint8_t)(page_prog_start >> 8);
				tmp_data[0] = (uint8_t)page_prog_start;
			}
			himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x20;

			tmp_data[3] = 0x67;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x00;
			himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			tmp_addr[3] = 0x80;
			tmp_addr[2] = 0x00;
			tmp_addr[1] = 0x00;
			tmp_addr[0] = 0x24;

			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = 0x20;
			himax_mcu_flash_write_burst_lenth(tmp_addr, tmp_data, 4);

			msleep(200);

			if (!g_core_fp.fp_wait_wip(100)) {
				E("%s:83112_Erase Fail\n", __func__);
				return false;
			}
		}
		return true;
}
#endif
static void himax_mcu_flash_programming(uint8_t *FW_content, int FW_Size)
{
	int page_prog_start = 0, i = 0, j = 0, k = 0;
	int program_length = PROGRAM_SZ;
	uint8_t tmp_data[DATA_LEN_4] = {0};
	uint8_t buring_data[FLASH_RW_MAX_LEN];	/* Read for flash data, 128K*/
	/* 4 bytes for padding*/
	g_core_fp.fp_interface_on();

	g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_fmt, DATA_LEN_4, pflash_op->data_spi200_trans_fmt, 0);

	for (page_prog_start = 0; page_prog_start < FW_Size; page_prog_start += FLASH_RW_MAX_LEN) {
		g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_2, 0);
		g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_2, 0);

		 /*Programmable size = 1 page = 256 bytes, word_number = 256 byte / 4 = 64*/
		g_core_fp.fp_register_write(pflash_op->addr_spi200_trans_ctrl, DATA_LEN_4, pflash_op->data_spi200_trans_ctrl_4, 0);

		/* Flash start address 1st : 0x0000_0000*/
		if (page_prog_start < 0x100) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = 0x00;
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x100 && page_prog_start < 0x10000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = 0x00;
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		} else if (page_prog_start >= 0x10000 && page_prog_start < 0x1000000) {
			tmp_data[3] = 0x00;
			tmp_data[2] = (uint8_t)(page_prog_start >> 16);
			tmp_data[1] = (uint8_t)(page_prog_start >> 8);
			tmp_data[0] = (uint8_t)page_prog_start;
		}
		g_core_fp.fp_register_write(pflash_op->addr_spi200_addr, DATA_LEN_4, tmp_data, 0);

		for (i = 0; i < ADDR_LEN_4; i++)
			buring_data[i] = pflash_op->addr_spi200_data[i];
		for (i = page_prog_start, j = 0; i < 16 + page_prog_start; i++, j++)
			buring_data[j + ADDR_LEN_4] = FW_content[i];
		if (himax_bus_write(pic_op->addr_ahb_addr_byte_0[0], buring_data, ADDR_LEN_4 + 16, HIMAX_XFER_RETRY_TIMES) < 0) {
			E("%s: xfer fail!\n", __func__);
			return;
		}

		g_core_fp.fp_register_write(pflash_op->addr_spi200_cmd, DATA_LEN_4, pflash_op->data_spi200_cmd_6, 0);

		for (j = 0; j < 5; j++) {
			for (i = (page_prog_start + 16 + (j * 48)), k = 0; i < (page_prog_start + 16 + (j * 48)) + program_length; i++, k++)
				buring_data[k + ADDR_LEN_4] = FW_content[i];
			if (himax_bus_write(pic_op->addr_ahb_addr_byte_0[0], buring_data, program_length + ADDR_LEN_4, HIMAX_XFER_RETRY_TIMES) < 0) {
				E("%s: xfer fail!\n", __func__);
				return;
			}
		}

		if (!g_core_fp.fp_wait_wip(1))
			E("%s:Flash_Programming Fail\n", __func__);
	}
}

static void himax_mcu_flash_page_write(uint8_t *write_addr, int length, uint8_t *write_data)
{
}

static int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_32k(unsigned char *fw, int len, bool change_iref)
{
	/* Not use */
	return 0;
}

static int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_60k(unsigned char *fw, int len, bool change_iref)
{
	/* Not use */
	return 0;
}

static int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_64k(unsigned char *fw, int len, bool change_iref)
{
	int burnFW_success = 0;

	if (len != FW_SIZE_64k) {
		E("%s: The file size is not 64K bytes\n", __func__);
		return false;
	}

#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, false);
#else
	g_core_fp.fp_system_reset();
#endif
	g_core_fp.fp_sense_off(true);
	g_core_fp.fp_block_erase(0x00, FW_SIZE_64k);
	g_core_fp.fp_flash_programming(fw, FW_SIZE_64k);

	if (g_core_fp.fp_check_CRC(pfw_op->addr_program_reload_from, FW_SIZE_64k) == 0)
		burnFW_success = 1;
	/*RawOut select initial*/
	g_core_fp.fp_register_write(pfw_op->addr_raw_out_sel, sizeof(pfw_op->data_clear), pfw_op->data_clear, 0);
	/*DSRAM func initial*/
	g_core_fp.fp_assign_sorting_mode(pfw_op->data_clear);

#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_ic_reset(false, false);
#else
	/*System reset*/
	g_core_fp.fp_system_reset();
#endif
	return burnFW_success;
}

static int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_124k(unsigned char *fw, int len, bool change_iref)
{
	/* Not use */
	return 0;
}

static int himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_128k(unsigned char *fw, int len, bool change_iref)
{
	/* Not use */
	return 0;
}

/* FLASH side end*/
#endif

#ifdef CORE_SRAM
/* SRAM side start*/
static void himax_mcu_sram_write(uint8_t *FW_content)
{
}

static bool himax_mcu_sram_verify(uint8_t *FW_File, int FW_Size)
{
	return true;
}

static bool himax_mcu_get_DSRAM_data(uint8_t *info_data, bool DSRAM_flag)
{
	int i = 0;
	unsigned char tmp_addr[ADDR_LEN_4];
	unsigned char tmp_data[DATA_LEN_4];
	uint8_t max_xfer_size = MAX_XFER_TRANS_SZ;
	uint8_t x_num = ic_data->HX_RX_NUM;
	uint8_t y_num = ic_data->HX_TX_NUM;
	/*int m_key_num = 0;*/
	int total_size = (x_num * y_num + x_num + y_num) * 2 + 4;
	int total_size_temp;
	int mutual_data_size = x_num * y_num * 2;
	int total_read_times = 0;
	int address = 0;
	uint8_t  *temp_info_data; /*max mkey size = 8*/
	uint16_t check_sum_cal = 0;
	int fw_run_flag = -1;

	temp_info_data = kzalloc(sizeof(uint8_t) * (total_size + 8), GFP_KERNEL);
	/*1. Read number of MKey R100070E8H to determin data size*/
	/*m_key_num = ic_data->HX_BT_NUM;
	I("%s,m_key_num=%d\n",__func__ ,m_key_num);
	total_size += m_key_num * 2;
	 2. Start DSRAM Rawdata and Wait Data Ready */
	tmp_data[3] = 0x00; tmp_data[2] = 0x00;
	tmp_data[1] = psram_op->passwrd_start[1];
	tmp_data[0] = psram_op->passwrd_start[0];
	fw_run_flag = himax_write_read_reg(psram_op->addr_rawdata_addr, tmp_data, psram_op->passwrd_end[1], psram_op->passwrd_end[0]);

	if (fw_run_flag < 0) {
		I("%s Data NOT ready => bypass\n", __func__);
		if (temp_info_data != NULL)
			kfree(temp_info_data);
		//return false;
		goto FAIL;
	}

	/* 3. Read RawData */
	total_size_temp = total_size;
	I("%s: tmp_data[0] = 0x%02X,tmp_data[1] = 0x%02X,tmp_data[2] = 0x%02X,tmp_data[3] = 0x%02X\n",
	  __func__, psram_op->addr_rawdata_addr[0], psram_op->addr_rawdata_addr[1], psram_op->addr_rawdata_addr[2], psram_op->addr_rawdata_addr[3]);
	tmp_addr[0] = psram_op->addr_rawdata_addr[0];
	tmp_addr[1] = psram_op->addr_rawdata_addr[1];
	tmp_addr[2] = psram_op->addr_rawdata_addr[2];
	tmp_addr[3] = psram_op->addr_rawdata_addr[3];

	if (total_size % max_xfer_size == 0)
		total_read_times = total_size / max_xfer_size;
	else
		total_read_times = total_size / max_xfer_size + 1;

	for (i = 0; i < total_read_times; i++) {
		address = (psram_op->addr_rawdata_addr[3] << 24) +
		(psram_op->addr_rawdata_addr[2] << 16) +
		(psram_op->addr_rawdata_addr[1] << 8) +
		psram_op->addr_rawdata_addr[0] + i * max_xfer_size;
		/*I("%s address = %08X\n", __func__, address);*/
		tmp_addr[3] = (uint8_t)((address >> 24) & 0x00FF);
		tmp_addr[2] = (uint8_t)((address >> 16) & 0x00FF);
		tmp_addr[1] = (uint8_t)((address >> 8) & 0x00FF);
		tmp_addr[0] = (uint8_t)((address) & 0x00FF);

		if (total_size_temp >= max_xfer_size) {
			g_core_fp.fp_register_read(tmp_addr, max_xfer_size, &temp_info_data[i * max_xfer_size], 0);
			total_size_temp = total_size_temp - max_xfer_size;
		} else {
			/*I("last total_size_temp=%d\n",total_size_temp);*/
			g_core_fp.fp_register_read(tmp_addr, total_size_temp % max_xfer_size, &temp_info_data[i * max_xfer_size], 0);
		}
	}

	/* 4. FW stop outputing */
	/*I("DSRAM_flag=%d\n",DSRAM_flag);*/
	if (DSRAM_flag == false || private_ts->diag_cmd == 0) {
		/*I("Return to Event Stack!\n");*/
		g_core_fp.fp_register_write(psram_op->addr_rawdata_addr, DATA_LEN_4, psram_op->data_fin, 0);
	} else {
		/*I("Continue to SRAM!\n");*/
		// g_core_fp.fp_register_write(psram_op->addr_rawdata_addr, DATA_LEN_4, psram_op->data_conti, 0);
	}

	/* 5. Data Checksum Check */
	for (i = 2; i < total_size; i += 2) { /* 2:PASSWORD NOT included */
		check_sum_cal += (temp_info_data[i + 1] * 256 + temp_info_data[i]);
	}
	if (check_sum_cal % 0x10000 != 0) {
		I("%s check_sum_cal fail=%2X\n", __func__, check_sum_cal);
		kfree(temp_info_data);
		temp_info_data = NULL;
		goto FAIL;
	} else {
		memcpy(info_data, &temp_info_data[4], mutual_data_size * sizeof(uint8_t));
		/*I("%s checksum PASS\n", __func__);*/
	}
	if (temp_info_data != NULL)
		kfree(temp_info_data);

	return true;
FAIL:
	g_core_fp.fp_register_read(pfw_op->addr_raw_out_sel, DATA_LEN_4, tmp_data, 0);
	I("%s: raw_out_sel(0x%08X): 0x%08X\n",
		__func__, ((uint32_t)pfw_op->addr_raw_out_sel[3]) << 24 | ((uint32_t)pfw_op->addr_raw_out_sel[2]) << 16 | ((uint32_t)pfw_op->addr_raw_out_sel[1]) << 8 | pfw_op->addr_raw_out_sel[0],
		((uint32_t)tmp_data[3]) << 24 | ((uint32_t)tmp_data[2]) << 16 | ((uint32_t)tmp_data[1]) << 8 | tmp_data[0]);
	himax_in_parse_assign_cmd(0x900000F8, tmp_addr, sizeof(tmp_addr));
	g_core_fp.fp_register_read(tmp_addr, DATA_LEN_4, tmp_data, 0);
	I("%s: 0x%08X: 0x%08X\n",
		__func__, ((uint32_t)tmp_addr[3]) << 24 | ((uint32_t)tmp_addr[2]) << 16 | ((uint32_t)tmp_addr[1]) << 8 | tmp_addr[0],
		((uint32_t)tmp_data[3]) << 24 | ((uint32_t)tmp_data[2]) << 16 | ((uint32_t)tmp_data[1]) << 8 | tmp_data[0]);
	memcpy(tmp_addr,  psram_op->addr_rawdata_addr, sizeof(tmp_addr));
	return false;
}
/* SRAM side end*/
#endif

#ifdef CORE_DRIVER

static void himax_mcu_init_ic(void)
{
	I("%s: use default incell init.\n", __func__);
}

#ifdef HX_RST_PIN_FUNC
static void himax_mcu_pin_reset(void)
{
	I("%s: Now reset the Touch chip.\n", __func__);
	himax_rst_gpio_set(private_ts->rst_gpio, 0);
	msleep(2);
	himax_rst_gpio_set(private_ts->rst_gpio, 1);

}

static void himax_mcu_ic_reset(uint8_t loadconfig, uint8_t int_off)
{
	struct himax_ts_data *ts = private_ts;
	if(!(private_ts->chip_name == HX_83112A_SERIES_PWON))
		HX_HW_RESET_ACTIVATE = 0;

	I("%s,status: loadconfig=%d,int_off=%d\n", __func__, loadconfig, int_off);
	if (ts->rst_gpio >= 0) {
		if (int_off)
			g_core_fp.fp_irq_switch(0);
		g_core_fp.fp_pin_reset();
		if (loadconfig)
			g_core_fp.fp_reload_config();
		if (int_off)
			g_core_fp.fp_irq_switch(1);
	}
}
#endif

extern struct himax_platform_data *hv_pdata;

static void himax_mcu_touch_information(void)
{
#ifndef HX_FIX_TOUCH_INFO
	char data[DATA_LEN_8] = {0};

	g_core_fp.fp_register_read(pdriver_op->addr_fw_define_rxnum_txnum_maxpt, DATA_LEN_8, data, 0);
	ic_data->HX_RX_NUM				= data[2];
	ic_data->HX_TX_NUM				= data[3];
	ic_data->HX_MAX_PT				= data[4];
	/*I("%s : HX_RX_NUM=%d,ic_data->HX_TX_NUM=%d,ic_data->HX_MAX_PT=%d\n",__func__,ic_data->HX_RX_NUM,ic_data->HX_TX_NUM,ic_data->HX_MAX_PT);*/
	g_core_fp.fp_register_read(pdriver_op->addr_fw_define_xy_res_enable, DATA_LEN_4, data, 0);

	/*I("%s : c_data->HX_XY_REVERSE=0x%2.2X\n",__func__,data[1]);*/
	if ((data[1] & 0x04) == 0x04)
		ic_data->HX_XY_REVERSE = true;
	else
		ic_data->HX_XY_REVERSE = false;

	g_core_fp.fp_register_read(pdriver_op->addr_fw_define_x_y_res, DATA_LEN_4, data, 0);
	ic_data->HX_Y_RES = data[0] * 256 + data[1];
	ic_data->HX_X_RES = data[2] * 256 + data[3];
	/*I("%s : ic_data->HX_Y_RES=%d,ic_data->HX_X_RES=%d\n",__func__,ic_data->HX_Y_RES,ic_data->HX_X_RES);*/

	g_core_fp.fp_register_read(pdriver_op->addr_fw_define_int_is_edge, DATA_LEN_4, data, 0);
	/*I("%s : data[0]=0x%2.2X,data[1]=0x%2.2X,data[2]=0x%2.2X,data[3]=0x%2.2X\n",__func__,data[0],data[1],data[2],data[3]);
	I("data[0] & 0x01 = %d\n",(data[0] & 0x01));*/
	VTI("data[1] & 0x01 = %d\n",(data[1] & 0x01));
	if ((data[1] & 0x01) == 1)
		ic_data->HX_INT_IS_EDGE = true;
	else
		ic_data->HX_INT_IS_EDGE = false;

	if (ic_data->HX_RX_NUM > 100)
		ic_data->HX_RX_NUM = 32;
	if (ic_data->HX_TX_NUM > 100)
		ic_data->HX_TX_NUM = 18;
	if (ic_data->HX_MAX_PT > 10)
		ic_data->HX_MAX_PT = 10;
	if (ic_data->HX_Y_RES > 4096)
		ic_data->HX_Y_RES = 1280;
	if (ic_data->HX_X_RES > 4096)
		ic_data->HX_X_RES = 720;
	/*1. Read number of MKey R100070E8H to determin data size*/
	g_core_fp.fp_register_read(psram_op->addr_mkey, DATA_LEN_4, data, 0);
	/* I("%s: tmp_data[0] = 0x%02X,tmp_data[1] = 0x%02X,tmp_data[2] = 0x%02X,tmp_data[3] = 0x%02X\n",
	 __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);*/
	ic_data->HX_BT_NUM = data[0] & 0x03;
#else
	ic_data->HX_RX_NUM				= hv_pdata->rx;
	ic_data->HX_TX_NUM				= hv_pdata->tx;
	ic_data->HX_BT_NUM				= FIX_HX_BT_NUM;
	ic_data->HX_X_RES				= hv_pdata->screenWidth;
	ic_data->HX_Y_RES				= hv_pdata->screenHeight;
	ic_data->HX_MAX_PT				= FIX_HX_MAX_PT;
	ic_data->HX_XY_REVERSE			= FIX_HX_XY_REVERSE;
	ic_data->HX_INT_IS_EDGE			= FIX_HX_INT_IS_EDGE;
#endif
	I("%s:HX_RX_NUM =%d,HX_TX_NUM =%d,HX_MAX_PT=%d\n", __func__, ic_data->HX_RX_NUM, ic_data->HX_TX_NUM, ic_data->HX_MAX_PT);
	I("%s:HX_XY_REVERSE =%d,HX_Y_RES =%d,HX_X_RES=%d\n", __func__, ic_data->HX_XY_REVERSE, ic_data->HX_Y_RES, ic_data->HX_X_RES);
	I("%s:HX_INT_IS_EDGE =%d\n", __func__, ic_data->HX_INT_IS_EDGE);
}

static void himax_mcu_reload_config(void)
{
	if (himax_report_data_init())
		E("%s: allocate data fail\n", __func__);
	g_core_fp.fp_sense_on(0x00);
}

static int himax_mcu_get_touch_data_size(void)
{
	return HIMAX_TOUCH_DATA_SIZE;
}

static int himax_mcu_hand_shaking(void)
{
	/* 0:Running, 1:Stop, 2:I2C Fail */
	int result = 0;
	return result;
}

static int himax_mcu_determin_diag_rawdata(int diag_command)
{
	return diag_command % 10;
}

static int himax_mcu_determin_diag_storage(int diag_command)
{
	return diag_command / 10;
}

static int himax_mcu_cal_data_len(int raw_cnt_rmd, int HX_MAX_PT, int raw_cnt_max)
{
	int RawDataLen;

	if (raw_cnt_rmd != 0x00)
		RawDataLen = MAX_XFER_TRANS_SZ - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;
	else
		RawDataLen = MAX_XFER_TRANS_SZ - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;

	return RawDataLen;
}

static bool himax_mcu_diag_check_sum(struct himax_report_data *hx_touch_data)
{
	uint16_t check_sum_cal = 0;
	int i;

	/* Check 128th byte CRC */
	for (i = 0, check_sum_cal = 0; i < (hx_touch_data->touch_all_size - hx_touch_data->touch_info_size); i += 2)
		check_sum_cal += (hx_touch_data->hx_rawdata_buf[i + 1] * FLASH_RW_MAX_LEN + hx_touch_data->hx_rawdata_buf[i]);
	if (check_sum_cal % HX64K != 0) {
		I("%s fail=%2X\n", __func__, check_sum_cal);
		return 0;
	}

	return 1;
}

static void himax_mcu_diag_parse_raw_data(struct himax_report_data *hx_touch_data, int mul_num, int self_num, uint8_t diag_cmd, int32_t *mutual_data, int32_t *self_data)
{
	diag_mcu_parse_raw_data(hx_touch_data, mul_num, self_num, diag_cmd, mutual_data, self_data);
}

#ifdef HX_ESD_RECOVERY
static int himax_mcu_ic_esd_recovery(int hx_esd_event, int hx_zero_event, int length)
{
	int ret_val = NO_ERR;

	if (g_zero_event_count > 5) {
		g_zero_event_count = 0;
		I("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
		ret_val = HX_ESD_EVENT;
		goto END_FUNCTION;
	}

	if (hx_esd_event == length) {
		g_zero_event_count = 0;
		ret_val = HX_ESD_EVENT;
		goto END_FUNCTION;
	} else if (hx_zero_event == length) {
		g_zero_event_count++;
		I("[HIMAX TP MSG]: ALL Zero event is %d times.\n", g_zero_event_count);
		ret_val = HX_ZERO_EVENT_COUNT;
		goto END_FUNCTION;
	}

END_FUNCTION:
	return ret_val;
}

static void himax_mcu_esd_ic_reset(void)
{
	HX_ESD_RESET_ACTIVATE = 0;
#ifdef HX_RST_PIN_FUNC
	himax_mcu_pin_reset();
#else
	g_core_fp.fp_system_reset();
#endif
	I("%s:\n", __func__);
}
#endif
#ifdef HX_TP_PROC_GUEST_INFO
char *g_checksum_str = "check sum fail";
char *g_guest_info_item[] = {
	"projectID",
	"CGColor",
	"BarCode",
	"Reserve1",
	"Reserve2",
	"Reserve3",
	"Reserve4",
	"Reserve5",
	"VCOM",
	"Vcom-3Gar",
	NULL
};

static int himax_guest_info_get_status(void)
{
	return g_guest_info_data->g_guest_info_ongoing;
}
static void himax_guest_info_set_status(int setting)
{
	g_guest_info_data->g_guest_info_ongoing = setting;
}

static int himax_guest_info_read(uint32_t start_addr, uint8_t *flash_tmp_buffer)
{
	uint32_t temp_addr = 0;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint32_t flash_page_len = 0x1000;
	/* uint32_t checksum = 0x00; */
	int result = 0;


	I("Reading guest info in start_addr = 0x%08X !\n", start_addr);

	tmp_addr[0] = start_addr % 0x100;
	tmp_addr[1] = (start_addr >> 8) % 0x100;
	tmp_addr[2] = (start_addr >> 16) % 0x100;
	tmp_addr[3] = start_addr / 0x1000000;
	I("Now start addr: tmp_addr[0]=0x%2X,tmp_addr[1]=0x%2X,tmp_addr[2]=0x%2X,tmp_addr[3]=0x%2X\n", tmp_addr[0], tmp_addr[1], tmp_addr[2], tmp_addr[3]);
	result = g_core_fp.fp_check_CRC(tmp_addr, flash_page_len);
	I("Checksum = 0x%8X\n", result);
	if (result != 0)
		goto END_FUNC;

	for (temp_addr = start_addr; temp_addr < (start_addr + flash_page_len); temp_addr = temp_addr + 128) {

		/* I("temp_addr=%d,tmp_addr[0]=0x%2X,tmp_addr[1]=0x%2X,tmp_addr[2]=0x%2X,tmp_addr[3]=0x%2X\n",temp_addr,tmp_addr[0],tmp_addr[1],tmp_addr[2],tmp_addr[3]); */
		tmp_addr[0] = temp_addr % 0x100;
		tmp_addr[1] = (temp_addr >> 8) % 0x100;
		tmp_addr[2] = (temp_addr >> 16) % 0x100;
		tmp_addr[3] = temp_addr / 0x1000000;
		g_core_fp.fp_register_read(tmp_addr, 128, &flash_tmp_buffer[temp_addr - start_addr], false);
		/* memcpy(&flash_tmp_buffer[temp_addr - start_addr],buffer,128);*/
	}

END_FUNC:
	return result;
}

static int hx_read_guest_info(void)
{
	/* uint8_t tmp_addr[4]; */
	uint32_t panel_color_addr = 0x10000;/*64k*/

	uint32_t info_len;
	uint32_t flash_page_len = 0x1000;/*4k*/
	uint8_t *flash_tmp_buffer 	= NULL;
	/* uint32_t temp_addr = 0; */
	uint8_t *temp_str;
	int i = 0;
	int custom_info_temp = 0;
	int checksum = 0;

	himax_guest_info_set_status(1);

	temp_str = kzalloc(128 * sizeof(uint8_t), GFP_KERNEL);
	if (!temp_str)
		return -ENOMEM;
	flash_tmp_buffer = kzalloc(HX_GUEST_INFO_SIZE * flash_page_len * sizeof(uint8_t), GFP_KERNEL);
	if (flash_tmp_buffer == NULL) {
		I("%s: Memory allocate fail!\n", __func__);
		kfree(temp_str);
		temp_str = NULL;
		return MEM_ALLOC_FAIL;
	}

	g_core_fp.fp_sense_off(true);
	/* g_core_fp.fp_burst_enable(1); */

	for (custom_info_temp = 0; custom_info_temp < HX_GUEST_INFO_SIZE; custom_info_temp++) {
		checksum = himax_guest_info_read(panel_color_addr+custom_info_temp * flash_page_len, &flash_tmp_buffer[custom_info_temp * flash_page_len]);
		if (checksum != 0) {
			E("%s:Checksum Fail! g_checksum_str len=%d\n", __func__, (int)strlen(g_checksum_str));
			memcpy(&g_guest_info_data->g_guest_str_in_format[custom_info_temp][0], g_checksum_str, (int)strlen(g_checksum_str));
			memcpy(&g_guest_info_data->g_guest_str[custom_info_temp][0], g_checksum_str, (int)strlen(g_checksum_str));
			continue;
		}

		info_len = flash_tmp_buffer[custom_info_temp * flash_page_len]
		+ (flash_tmp_buffer[custom_info_temp*flash_page_len + 1] << 8)
		+ (flash_tmp_buffer[custom_info_temp*flash_page_len + 2] << 16)
		+ (flash_tmp_buffer[custom_info_temp*flash_page_len + 3] << 24);

		I("Now custom_info_temp = %d\n", custom_info_temp);

		I("Now size_buff[0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n"
		, flash_tmp_buffer[custom_info_temp*flash_page_len]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 1]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 2]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 3]);

		I("Now total length=%d\n", info_len);

		g_guest_info_data->g_guest_data_len[custom_info_temp] = info_len;

		I("Now custom_info_id [0]=%d,[1]=%d,[2]=%d,[3]=%d\n"
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 4]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 5]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 6]
		, flash_tmp_buffer[custom_info_temp*flash_page_len + 7]);

		g_guest_info_data->g_guest_data_type[custom_info_temp] = flash_tmp_buffer[custom_info_temp * flash_page_len + 7];

		/* if(custom_info_temp < 3) { */
			if (info_len > 128) {
				I("%s: info_len=%d\n", __func__, info_len);
				info_len = 128;
			}
			for (i = 0; i < info_len; i++)
				temp_str[i] = flash_tmp_buffer[custom_info_temp * flash_page_len + HX_GUEST_INFO_LEN_SIZE + HX_GUEST_INFO_ID_SIZE + i];
			I("g_guest_info_data->g_guest_str_in_format[%d] size = %d\n", custom_info_temp, info_len);
			memcpy(&g_guest_info_data->g_guest_str_in_format[custom_info_temp][0], temp_str, info_len);
		/*}*/

		for (i = 0; i < 128; i++)
			temp_str[i] = flash_tmp_buffer[custom_info_temp * flash_page_len + i];

		I("g_guest_info_data->g_guest_str[%d] size = %d\n", custom_info_temp, 128);
		memcpy(&g_guest_info_data->g_guest_str[custom_info_temp][0], temp_str, 128);
		/*if(custom_info_temp == 0)
		{
			for( i = 0; i< 256 ; i++)
			{
				if(i % 16 == 0 && i > 0)
					I("\n");
				I("g_guest_info_data->g_guest_str[%d][%d] = 0x%02X",custom_info_temp,i,g_guest_info_data->g_guest_str[custom_info_temp][i]);
			}
		}*/
	}
	/* himax_burst_enable(private_ts->client, 0); */
	g_core_fp.fp_fw_sts_clear();
	g_core_fp.fp_sense_on(0x01);

	kfree(flash_tmp_buffer);
	kfree(temp_str);
	himax_guest_info_set_status(0);
	return NO_ERR;
}
#endif
#endif

#if defined(HX_SMART_WAKEUP)
static void himax_mcu_resend_cmd_func(bool suspended)
{
#if defined(HX_SMART_WAKEUP)
	struct himax_ts_data *ts = private_ts;
#endif
#ifdef HX_SMART_WAKEUP
	g_core_fp.fp_set_SMWP_enable(ts->SMWP_enable, suspended);
#endif
#ifdef HX_HIGH_SENSE
	g_core_fp.fp_set_HSEN_enable(ts->HSEN_enable, suspended);
#endif
#ifdef HX_USB_DETECT_GLOBAL
	himax_cable_detect_func(true);
#endif
}
#endif

#ifdef HX_ZERO_FLASH
int G_POWERONOF = 1;

void hx_dis_rload_0f(int disable)
{
	/*Diable Flash Reload*/
	uint8_t tmp_data[4] = {0};
	int retry = 0;
	
	I("%s\n", __func__);
	/*Diable Flash Reload 0x10007f00 write 0x00009AA9*/

	do {
		
		g_core_fp.fp_register_write(pzf_op->addr_dis_flash_reload, DATA_LEN_4, pzf_op->data_dis_flash_reload, 0);
		usleep_range(1000, 1001);
	
		g_core_fp.fp_register_read(pzf_op->addr_dis_flash_reload, DATA_LEN_4, tmp_data, 0);
		
		I("%s: Now retry=%d, data=0x%02X%02X%02X%02X\n", __func__, retry,
		tmp_data[3], tmp_data[2], tmp_data[1], tmp_data[0]);

	} while((retry++ < 10) && (!(tmp_data[3] == 0x00 && tmp_data[2] == 0x00 && tmp_data[1] == 0x9A && tmp_data[0] == 0xA9 )));

	I("%s data[0] = 0x%2.2X, data[1] = 0x%2.2X, data[2] = 0x%2.2X, data[3] = 0x%2.2X\n", __func__, tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3]);
	
}

void himax_mcu_clean_sram_0f(uint8_t *addr, int write_len, int type)
{
	int total_read_times = 0;
	int max_bus_size = MAX_XFER_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;

	uint8_t fix_data = 0x00;
	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t tmp_data[MAX_XFER_TRANS_SZ] = {0};

	I("%s, Entering\n", __func__);

	total_size = write_len;
	if (write_len > (HX_SPI_MTK_MAX_WRITE_SZ - 4)) {
		max_bus_size = (HX_SPI_MTK_MAX_WRITE_SZ - 4);
	} else {
		max_bus_size = write_len;
	}

	total_size_temp = write_len;

	g_core_fp.fp_burst_enable(1);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	I("%s, write addr tmp_addr[3]=0x%2.2X,  tmp_addr[2]=0x%2.2X,  tmp_addr[1]=0x%2.2X,  tmp_addr[0]=0x%2.2X\n", __func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	switch (type) {
	case 0:
		fix_data = 0x00;
		break;
	case 1:
		fix_data = 0xAA;
		break;
	case 2:
		fix_data = 0xBB;
		break;
	}

	for (i = 0; i < MAX_XFER_TRANS_SZ; i++)
		tmp_data[i] = fix_data;
	I("%s,  total size=%d\n", __func__, total_size);

	if (total_size_temp % max_bus_size == 0)
		total_read_times = total_size_temp / max_bus_size;
	else
		total_read_times = total_size_temp / max_bus_size + 1;

	for (i = 0; i < (total_read_times); i++) {
		I("[log]write %d time start!\n", i);
		if (total_size_temp >= max_bus_size) {
			g_core_fp.fp_register_write(tmp_addr,  max_bus_size, tmp_data, 0);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			I("last total_size_temp=%d\n", total_size_temp);
			g_core_fp.fp_register_write(tmp_addr,  total_size_temp % max_bus_size, tmp_data, 0);
		}
		address = ((i+1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address>>8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}

	I("%s, END\n", __func__);
}

void himax_mcu_write_sram_0f(const struct firmware *fw_entry, uint8_t *addr, int start_index, uint32_t write_len)
{
	//int max_bus_size = MAX_XFER_TRANS_SZ;

	//uint8_t tmp_addr[DATA_LEN_4] = {0};

	/*I("%s, Entering - total write size=%d\n", __func__, total_size_temp);*/

	//memcpy(tmp_addr, addr, 4); /* assign addr 4bytes */

	//g_core_fp.fp_register_write(tmp_addr,  write_len, (uint8_t *)&fw_entry->data[start_index], 0);
	g_core_fp.fp_register_write(addr,  write_len, (uint8_t *)&fw_entry->data[start_index], 0);
}

int himax_sram_write_crc_check(const struct firmware *fw_entry, uint8_t *addr, int strt_idx, uint32_t len)
{
	int retry = 0;
	int crc = -1;

	do {
		g_core_fp.fp_write_sram_0f(fw_entry, addr, strt_idx, len);
		crc = g_core_fp.fp_check_CRC(addr,  len);
		retry++;
		/*I("%s, HW CRC %s in %d time\n", __func__, (crc == 0)?"OK":"Fail", retry);*/
	} while (crc != 0 && retry < 10);

	return crc;
}

extern int himax_zf_part_info(const struct firmware *fw_entry);
void himax_mcu_firmware_update_0f(const struct firmware *fw_entry)
{
	bool ret = false;

	I("%s,Entering - total FW size=%d\n", __func__, (int)fw_entry->size);

	if ((int)fw_entry->size == 0)
		return;

	g_core_fp.fp_register_write(pzf_op->addr_system_reset, 4, pzf_op->data_system_reset, 0);

	g_core_fp.fp_sense_off(false);
	
	timeStart = ktime_to_us(ktime_get());
	if ((int)fw_entry->size > HX64K) {
		ret = himax_zf_part_info(fw_entry);
	}
	timeEnd = ktime_to_us(ktime_get());
	I("update firmware time = %lldus\n", timeEnd - timeStart);

	I("%s, End\n", __func__);
}
#endif

int hx_parse_bin_cfg_data(const struct firmware *fw_entry)
{
	int part_num = 0;
	int i = 0;
	uint8_t buf[16];
	int i_max = 0;
	int i_min = 0;
	uint32_t dsram_base = 0xFFFFFFFF;
	uint32_t dsram_max = 0;
	int ret = 0;
	int hx_cfg_sz = 0;
	/*int different = 0;*/
	/*int j = 0;*/
	bool flag_1k_header = false;
	int cfg_crc_sw = 0;
	int cfg_crc_hw = 0;
	int retry = 3;

	uint8_t hx_sram_min[4];
	struct zf_info *zf_info_arr;
	unsigned char *hx_nf_FW_buf;

		/*0. check 1k header*/
	if (fw_entry->data[0x00] == 0x00
		&& fw_entry->data[0x01] == 0x00
		&& fw_entry->data[0x02] == 0x00
		&& fw_entry->data[0x03] == 0x00
		&& fw_entry->data[0x04] == 0x00
		&& fw_entry->data[0x05] == 0x00
		&& fw_entry->data[0x06] == 0x00
		&& fw_entry->data[0x07] == 0x00
		&& fw_entry->data[0x0E] == 0x87)
		flag_1k_header = true;
	else
		flag_1k_header = false;

	/*1. get number of partition*/
	if(flag_1k_header == true)
		part_num = fw_entry->data[HX64K + HX1K + 12];
	else
		part_num = fw_entry->data[HX64K + 12];

	I("%s, Number of partition is %d\n", __func__, part_num);
	if (part_num <= 1) {
		E("%s, size of cfg part failed! part_num = %d\n", __func__, part_num);
		return LENGTH_FAIL;
	}

	/*2. initial struct of array*/
	zf_info_arr = kcalloc(part_num, sizeof(struct zf_info), GFP_KERNEL);
	if (zf_info_arr == NULL) {
		E("%s, Allocate ZF info array failed!\n", __func__);
		ret =  MEM_ALLOC_FAIL;
		goto HX_ZF_MEM_ALLOC_FAIL;
	}


	/* i = 0 is fw entity, it need to update by itself*/
	for (i = 0; i < part_num; i++) {
		
		/*3. get all partition*/
		if(flag_1k_header == true)
			memcpy(buf, &fw_entry->data[i * 0x10 + HX64K + HX1K], 16);
		else
			memcpy(buf, &fw_entry->data[i * 0x10 + HX64K], 16);
		
		memcpy(zf_info_arr[i].sram_addr, buf, 4);
		
		zf_info_arr[i].write_size = buf[7] << 24 | buf[6] << 16 | buf[5] << 8 | buf[4];
		zf_info_arr[i].fw_addr = buf[11] << 24 | buf[10] << 16 | buf[9] << 8 | buf[8];
		zf_info_arr[i].cfg_addr = zf_info_arr[i].sram_addr[0];
		zf_info_arr[i].cfg_addr += zf_info_arr[i].sram_addr[1] << 8;
		zf_info_arr[i].cfg_addr += zf_info_arr[i].sram_addr[2] << 16;
		zf_info_arr[i].cfg_addr += zf_info_arr[i].sram_addr[3] << 24;

		if (i > 0) {
			if (dsram_base > zf_info_arr[i].cfg_addr) {
				dsram_base = zf_info_arr[i].cfg_addr;
				i_min = i;
			}
			if (dsram_max < zf_info_arr[i].cfg_addr) {
				dsram_max = zf_info_arr[i].cfg_addr;
				i_max = i;
			}
		}

	}
	for (i = 0; i < ADDR_LEN_4; i++)
		hx_sram_min[i] = zf_info_arr[i_min].sram_addr[i];

	hx_cfg_sz = (dsram_max - dsram_base) + zf_info_arr[i_max].write_size;
	if (hx_cfg_sz % 16 != 0)
		hx_cfg_sz = hx_cfg_sz + 16 - (hx_cfg_sz % 16);

	I("%s, cfg_sz = %d!, dsram_base = %X, dsram_max = %X\n", __func__, hx_cfg_sz, dsram_base, dsram_max);

	hx_nf_FW_buf = kcalloc(FW_BIN_16K_SZ, sizeof(uint8_t), GFP_KERNEL);
	if (hx_nf_FW_buf == NULL) {
		E("%s, Allocate FW_buf array failed!\n", __func__);
		ret =  MEM_ALLOC_FAIL;
		goto HX_FW_MEM_ALLOC_FAIL;
	}

	for (i = 1; i < part_num; i++) {
		if (zf_info_arr[i].cfg_addr % 4 != 0)
			zf_info_arr[i].cfg_addr = zf_info_arr[i].cfg_addr - (zf_info_arr[i].cfg_addr % 4);

		I("%s,[%d] SRAM addr = %08X\n", __func__, i, zf_info_arr[i].cfg_addr);
		I("%s,[%d] fw_addr = %08X\n", __func__, i, zf_info_arr[i].fw_addr);
		I("%s,[%d] write_size = %d\n", __func__, i, zf_info_arr[i].write_size);

		memcpy(&hx_nf_FW_buf[zf_info_arr[i].cfg_addr - dsram_base], &fw_entry->data[zf_info_arr[i].fw_addr], zf_info_arr[i].write_size);
	}

	cfg_crc_sw = g_core_fp.fp_Calculate_CRC_with_AP(hx_nf_FW_buf, 0, hx_cfg_sz);
	I("Now cfg_crc_sw=%X\n", cfg_crc_sw);
		

	I("Now zf_info_arr[0].sram_addr=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n", zf_info_arr[0].sram_addr[0], zf_info_arr[0].sram_addr[1], zf_info_arr[0].sram_addr[2], zf_info_arr[0].sram_addr[3]);
	if (himax_sram_write_crc_check(fw_entry, zf_info_arr[0].sram_addr, zf_info_arr[0].fw_addr, zf_info_arr[0].write_size) != 0)
		E("%s, HW CRC FAIL\n", __func__);
	else
		I("%s, HW CRC PASS\n", __func__);


	I("Now hx_sram_min[0]=0x%02X,[1]=0x%02X,[2]=0x%02X,[3]=0x%02X\n", hx_sram_min[0], hx_sram_min[1], hx_sram_min[2], hx_sram_min[3]);
	/* 4. write to sram*/

	do {
		g_core_fp.fp_register_write(hx_sram_min, hx_cfg_sz, hx_nf_FW_buf, 0);
		cfg_crc_hw = g_core_fp.fp_check_CRC(hx_sram_min, hx_cfg_sz);
		if (cfg_crc_hw != cfg_crc_sw)
			E("Config CRC FAIL, HW CRC = %X,SW CRC = %X, retry = %d\n", cfg_crc_hw, cfg_crc_sw, retry);
		else
			I("Config CRC Pass\n");

	} while (cfg_crc_hw != cfg_crc_sw && retry-- > 0);

	kfree(hx_nf_FW_buf);

	return hx_cfg_sz;


HX_FW_MEM_ALLOC_FAIL:
kfree(zf_info_arr);

HX_ZF_MEM_ALLOC_FAIL:

return ret;

}


int himax_zf_part_info(const struct firmware *fw_entry)
{
	int ret = 0;
	int cfg_sz = 0;

	/*3. Find the data in the bin file */
	/*CFG*/
	cfg_sz = hx_parse_bin_cfg_data(fw_entry);
	I("Now cfg_sz=%d\n", cfg_sz);
	if (cfg_sz < 0) {
		ret = LENGTH_FAIL;
		return ret;
	}

	return ret;
}

int hx_0f_op_file_dirly(char *file_name)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;

	mutex_lock(&private_ts->w_fw_lock);
	#ifdef HX_VIVO_FW_BIN_FOR_H
		static struct firmware array_info;
		int fw_size = 0;
	#endif

	I("%s, Entering\n", __func__);
	I("file name = %s\n", file_name);
	err = request_firmware(&fw_entry, file_name, private_ts->dev);
	if (err < 0) {
		E("%s, fail in line%d error code=%d,file maybe fail\n", __func__, __LINE__, err);
		return err;
	}

	himax_int_enable(0);

	if (g_f_0f_updat == 1) {
		I("%s:[Warning]Other thread is updating now!\n", __func__);
		err = -1;
		release_firmware(fw_entry);
		mutex_unlock(&private_ts->w_fw_lock);
		return err;
	} else {
		I("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}

	g_core_fp.fp_firmware_update_0f(fw_entry);
	release_firmware(fw_entry);

	g_f_0f_updat = 0;
	I("%s, END\n", __func__);
	mutex_unlock(&private_ts->w_fw_lock);
	return err;
}

int himax_mcu_0f_operation_dirly(void)
{
	int ret = NO_ERR;
	const struct firmware *fw_entry = NULL;	
	static struct firmware array_info;
	int fw_size = 0;
	int flag_boot_h = 0;

	I("%s, Entering\n", __func__);
	VTI("file name = %s\n", i_CTPM_firmware_name);

	mutex_lock(&private_ts->w_fw_lock);
	ret = request_firmware(&fw_entry, i_CTPM_firmware_name, private_ts->dev);
	if (ret < 0) {
		if (!strcmp(i_CTPM_firmware_name, HX_MP_FW)) {
			VTI("VTS_FW_TYPE_FW= %d", VTS_FW_TYPE_MP);
			array_info.data = vts_fw_data_get(private_ts->vtsdev, VTS_FW_TYPE_MP, &fw_size);
			array_info.size = fw_size;
			release_firmware(fw_entry);
			fw_entry = &array_info;
			I("%s, in line%d code=%d,bin file not load\n", __func__, __LINE__, ret);
			flag_boot_h = 1;
			ret = 0;
		} else if (!strcmp(i_CTPM_firmware_name, HX_NORMAL_FW)) {
			VTI("VTS_FW_TYPE_FW= %d", VTS_FW_TYPE_FW);
			array_info.data = vts_fw_data_get(private_ts->vtsdev, VTS_FW_TYPE_FW, &fw_size);
			array_info.size = fw_size;
			release_firmware(fw_entry);
			fw_entry = &array_info;
			I("%s, in line%d code=%d,bin file not load\n", __func__, __LINE__, ret);
			flag_boot_h = 1;
			ret = 0;
		}
	}


	himax_int_enable(0);

	if (g_f_0f_updat == 1) {
		I("%s:[Warning]Other thread is updating now!\n", __func__);
		release_firmware(fw_entry);
		ret = -1;
		return ret;
	} else {
		I("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}

	g_core_fp.fp_firmware_update_0f(fw_entry);
	if (0 == flag_boot_h) {
		release_firmware(fw_entry);
	}

	g_f_0f_updat = 0;
	I("%s, END\n", __func__);
	mutex_unlock(&private_ts->w_fw_lock);
	return ret;
}
void himax_mcu_0f_operation(struct work_struct *work)
{
	int ret;
	const struct firmware *fw_entry = NULL;
	static struct firmware array_info;
	int fw_size = 0;
	int flag_boot_h = 0;
	
	mutex_lock(&private_ts->w_fw_lock);
	I("%s, Entering\n", __func__);
	I("file name = %s\n", i_CTPM_firmware_name);
	ret = request_firmware(&fw_entry, i_CTPM_firmware_name, private_ts->dev);
	if (ret < 0) {
		VTI("VTS_FW_TYPE_FW= %d", VTS_FW_TYPE_FW);
		array_info.data = vts_fw_data_get(private_ts->vtsdev, VTS_FW_TYPE_FW, &fw_size);
		array_info.size = fw_size;
		release_firmware(fw_entry);
		fw_entry = &array_info;
		I("%s, in line %d code = %d,bin file not load\n", __func__, __LINE__, ret);
		flag_boot_h = 1;
	}

	if (g_f_0f_updat == 1) {
		I("%s:[Warning]Other thread is updating now!\n", __func__);
		release_firmware(fw_entry);
		mutex_unlock(&private_ts->w_fw_lock);
		return;
	} else {
		I("%s:Entering Update Flow!\n", __func__);
		g_f_0f_updat = 1;
	}


	himax_int_enable(0);

	g_core_fp.fp_firmware_update_0f(fw_entry);
	if (0 == flag_boot_h) {
		release_firmware(fw_entry);
	}
	g_core_fp.fp_fw_sts_clear();
	g_core_fp.fp_reload_disable(0);
	g_core_fp.fp_reload_fw_clear_register();
	msleep(10);
	g_core_fp.fp_read_FW_ver();
	g_core_fp.fp_touch_information();
	msleep(10);

	g_core_fp.fp_sense_on(0x00);
	msleep(10);
	I("%s:End\n", __func__);
	himax_int_enable(1);

	g_f_0f_updat = 0;
	I("%s, END\n", __func__);
	mutex_unlock(&private_ts->w_fw_lock);
	return;
}

static int himax_mcu_0f_esd_check(void)
{
	return NO_ERR;
}

#ifdef HX_0F_DEBUG
void himax_mcu_read_sram_0f(const struct firmware *fw_entry, uint8_t *addr, int start_index, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_XFER_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0, j = 0;
	int not_same = 0;

	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t *temp_info_data;
	int *not_same_buff;

	I("%s, Entering\n", __func__);

	g_core_fp.fp_burst_enable(1);

	total_size = read_len;

	total_size_temp = read_len;

#if defined(HX_SPI_OPERATION)
	if (read_len > 2048)
		max_bus_size = 2048;
	else
		max_bus_size = read_len;

#else
	if (read_len > 240)
		max_bus_size = 240;
	else
		max_bus_size = read_len;

#endif

	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);
	not_same_buff = kzalloc(sizeof(int) * total_size, GFP_KERNEL);

	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	I("%s,  read addr tmp_addr[3]=0x%2.2X,  tmp_addr[2]=0x%2.2X,  tmp_addr[1]=0x%2.2X,  tmp_addr[0]=0x%2.2X\n", __func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	I("%s,  total size=%d\n", __func__, total_size);

	g_core_fp.fp_burst_enable(1);

	if (total_size % max_bus_size == 0)
		total_read_times = total_size / max_bus_size;
	else
		total_read_times = total_size / max_bus_size + 1;


	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			g_core_fp.fp_register_read(tmp_addr, max_bus_size, &temp_info_data[i*max_bus_size], false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			g_core_fp.fp_register_read(tmp_addr, total_size_temp % max_bus_size, &temp_info_data[i*max_bus_size], false);
		}

		address = ((i+1) * max_bus_size);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);
		if (tmp_addr[0] < addr[0])
			tmp_addr[1] = addr[1] + (uint8_t) ((address>>8) & 0x00FF) + 1;
		else
			tmp_addr[1] = addr[1] + (uint8_t) ((address>>8) & 0x00FF);

		msleep(10);
	}
	I("%s, READ Start\n", __func__);
	I("%s, start_index = %d\n", __func__, start_index);
	j = start_index;
	for (i = 0; i < read_len; i++, j++) {
		if (fw_entry->data[j] != temp_info_data[i]) {
			not_same++;
			not_same_buff[i] = 1;
		}

		I("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i%16 == 15)
			printk("\n");
	}
	I("%s, READ END\n", __func__);
	I("%s, Not Same count=%d\n", __func__, not_same);
	if (not_same != 0) {
		j = start_index;
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1)
				I("bin = [%d] 0x%2.2X\n", i, fw_entry->data[j]);
		}
		for (i = 0; i < read_len; i++, j++) {
			if (not_same_buff[i] == 1)
				I("sram = [%d] 0x%2.2X\n", i, temp_info_data[i]);
		}
	}
	I("%s, READ END\n", __func__);
	I("%s, Not Same count=%d\n", __func__, not_same);
	I("%s, END\n", __func__);

	kfree(not_same_buff);
	kfree(temp_info_data);
}

void himax_mcu_read_all_sram(uint8_t *addr, int read_len)
{
	int total_read_times = 0;
	int max_bus_size = MAX_XFER_TRANS_SZ;
	int total_size_temp = 0;
	int total_size = 0;
	int address = 0;
	int i = 0;
	/*
	struct file *fn;
	struct filename *vts_name;
	*/

	uint8_t tmp_addr[DATA_LEN_4] = {0};
	uint8_t *temp_info_data;

	I("%s, Entering\n", __func__);

	g_core_fp.fp_burst_enable(1);

	total_size = read_len;

	total_size_temp = read_len;

	temp_info_data = kzalloc(sizeof(uint8_t) * total_size, GFP_KERNEL);


	tmp_addr[3] = addr[3];
	tmp_addr[2] = addr[2];
	tmp_addr[1] = addr[1];
	tmp_addr[0] = addr[0];
	I("%s,  read addr tmp_addr[3]=0x%2.2X,  tmp_addr[2]=0x%2.2X,  tmp_addr[1]=0x%2.2X,  tmp_addr[0]=0x%2.2X\n", __func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);

	I("%s,  total size=%d\n", __func__, total_size);

	if (total_size % max_bus_size == 0)
		total_read_times = total_size / max_bus_size;
	else
		total_read_times = total_size / max_bus_size + 1;

	for (i = 0; i < (total_read_times); i++) {
		if (total_size_temp >= max_bus_size) {
			g_core_fp.fp_register_read(tmp_addr,  max_bus_size,  &temp_info_data[i*max_bus_size],  false);
			total_size_temp = total_size_temp - max_bus_size;
		} else {
			g_core_fp.fp_register_read(tmp_addr,  total_size_temp % max_bus_size,  &temp_info_data[i*max_bus_size],  false);
		}

		address = ((i+1) * max_bus_size);
		tmp_addr[1] = addr[1] + (uint8_t) ((address>>8) & 0x00FF);
		tmp_addr[0] = addr[0] + (uint8_t) ((address) & 0x00FF);

		msleep(10);
	}
	I("%s,  NOW addr tmp_addr[3]=0x%2.2X,  tmp_addr[2]=0x%2.2X,  tmp_addr[1]=0x%2.2X,  tmp_addr[0]=0x%2.2X\n", __func__, tmp_addr[3], tmp_addr[2], tmp_addr[1], tmp_addr[0]);
	/*for(i = 0;i<read_len;i++)
	{
		I("0x%2.2X, ", temp_info_data[i]);

		if (i > 0 && i%16 == 15)
			printk("\n");
	}*/

	/* need modify
	I("Now Write File start!\n");
	vts_name = getname_kernel("/sdcard/dump_dsram.txt");
	fn = file_open_name(vts_name, O_CREAT | O_WRONLY, 0);
	if (!IS_ERR (fn)) {
		I("%s create file and ready to write\n", __func__);
		fn->f_op->write (fn, temp_info_data, read_len*sizeof (uint8_t), &fn->f_pos);
		filp_close (fn, NULL);
	}
	I("Now Write File End!\n");
	*/

	I("%s, END\n", __func__);

	kfree(temp_info_data);
}

void himax_mcu_firmware_read_0f(const struct firmware *fw_entry, int type)
{
	uint8_t tmp_addr[DATA_LEN_4] = {0};

	I("%s, Entering\n", __func__);
	if (type == 0) { /* first 48K */
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_sram_start_addr, 0, HX_48K_SZ);
		g_core_fp.fp_read_all_sram(tmp_addr, 0xC000);
	} else { /*last 16k*/
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_cfg_info, 0xC000, 132);

		/*FW config*/
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg_1, 0xC0FE, 484);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg_2, 0xC9DE, 36);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_fw_cfg_3, 0xCA00, 72);

		/*ADC config*/

		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_1, 0xD630, 1188);
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_adc_cfg_2, 0xD318, 792);


		/*mapping table*/
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_map_table, 0xE000, 1536);

		/* set n frame=0*/
		g_core_fp.fp_read_sram_0f(fw_entry, pzf_op->data_mode_switch, 0xC30C, 4);
	}

	I("%s, END\n", __func__);
}

void himax_mcu_0f_operation_check(int type)
{
	int err = NO_ERR;
	const struct firmware *fw_entry = NULL;
	/* char *firmware_name = "himax.bin"; */


	I("%s, Entering\n", __func__);
	memset(i_CTPM_firmware_name, 0x00, sizeof(i_CTPM_firmware_name));
	memcpy(i_CTPM_firmware_name, HX_NORMAL_FW, sizeof(char)*strlen(HX_NORMAL_FW));
	I("file name = %s\n", i_CTPM_firmware_name);


	err = request_firmware(&fw_entry,  i_CTPM_firmware_name, private_ts->dev);
	if (err < 0) {
		E("%s, fail in line%d error code=%d\n", __func__, __LINE__, err);
		return;
	}

	I("first 4 bytes 0x%2X, 0x%2X, 0x%2X, 0x%2X !\n", fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3]);
	I("next 4 bytes 0x%2X, 0x%2X, 0x%2X, 0x%2X !\n", fw_entry->data[4], fw_entry->data[5], fw_entry->data[6], fw_entry->data[7]);
	I("and next 4 bytes 0x%2X, 0x%2X, 0x%2X, 0x%2X !\n", fw_entry->data[8], fw_entry->data[9], fw_entry->data[10], fw_entry->data[11]);

	g_core_fp.fp_firmware_read_0f(fw_entry, type);

	release_firmware(fw_entry);
	I("%s, END\n", __func__);
	return;
}
#endif

#ifdef CORE_INIT
/* init start */
static void himax_mcu_fp_init(void)
{
#ifdef CORE_IC
	g_core_fp.fp_burst_enable = himax_mcu_burst_enable;
	g_core_fp.fp_register_read = himax_mcu_register_read;
	/*g_core_fp.fp_flash_write_burst = himax_mcu_flash_write_burst;*/
	g_core_fp.fp_flash_write_burst_lenth = himax_mcu_flash_write_burst_lenth;
	g_core_fp.fp_register_write = himax_mcu_register_write;
	g_core_fp.fp_interface_on = himax_mcu_interface_on;
	g_core_fp.fp_sense_on = himax_mcu_sense_on;
	g_core_fp.fp_sense_off = himax_mcu_sense_off;
	g_core_fp.fp_wait_wip = himax_mcu_wait_wip;
	g_core_fp.fp_init_psl = himax_mcu_init_psl;
	g_core_fp.fp_resume_ic_action = himax_mcu_resume_ic_action;
	g_core_fp.fp_suspend_ic_action = himax_mcu_suspend_ic_action;
	g_core_fp.fp_power_on_init = himax_mcu_power_on_init;
#endif
#ifdef CORE_FW
	g_core_fp.fp_system_reset = himax_mcu_system_reset;
	g_core_fp.fp_Calculate_CRC_with_AP = himax_mcu_Calculate_CRC_with_AP;
	g_core_fp.fp_check_CRC = himax_mcu_check_CRC;
	g_core_fp.fp_set_reload_cmd = himax_mcu_set_reload_cmd;
	g_core_fp.fp_program_reload = himax_mcu_program_reload;
	g_core_fp.fp_set_SMWP_enable = himax_mcu_set_SMWP_enable;
	g_core_fp.fp_set_HSEN_enable = himax_mcu_set_HSEN_enable;
	g_core_fp.fp_usb_detect_set = himax_mcu_usb_detect_set;
	g_core_fp.fp_diag_register_set = himax_mcu_diag_register_set;
	g_core_fp.fp_fw_sts_clear = hx_fw_sts_clear;
#ifndef HX_VIVO_DEBUG_NODE
	g_core_fp.fp_chip_self_test = himax_mcu_chip_self_test;
#endif

	g_core_fp.fp_reload_fw_clear_register = himax_reload_fw_clear_register;
	g_core_fp.fp_check_remapping = himax_check_remapping;
	g_core_fp.fp_idle_mode = himax_mcu_idle_mode;
	g_core_fp.fp_reload_disable = himax_mcu_reload_disable;
	g_core_fp.fp_check_chip_version = himax_mcu_check_chip_version;
	g_core_fp.fp_read_ic_trigger_type = himax_mcu_read_ic_trigger_type;
	g_core_fp.fp_read_xfer_status = himax_mcu_read_xfer_status;
	g_core_fp.fp_read_FW_ver = himax_mcu_read_FW_ver;
	g_core_fp.fp_read_event_stack = himax_mcu_read_event_stack;
	g_core_fp.fp_return_event_stack = himax_mcu_return_event_stack;
	g_core_fp.fp_calculateChecksum = himax_mcu_calculateChecksum;
	g_core_fp.fp_read_FW_status = himax_mcu_read_FW_status;
	g_core_fp.fp_irq_switch = himax_mcu_irq_switch;
	g_core_fp.fp_assign_sorting_mode = himax_mcu_assign_sorting_mode;
	g_core_fp.fp_check_sorting_mode = himax_mcu_check_sorting_mode;
	g_core_fp.fp_switch_mode = himax_mcu_switch_mode;
	g_core_fp.fp_read_DD_status = himax_mcu_read_DD_status;
#endif
#ifdef CORE_FLASH
	g_core_fp.fp_chip_erase = himax_mcu_chip_erase;
	g_core_fp.fp_block_erase = himax_mcu_block_erase;
	g_core_fp.fp_flash_programming = himax_mcu_flash_programming;
	g_core_fp.fp_flash_page_write = himax_mcu_flash_page_write;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_32k = himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_32k;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_60k = himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_60k;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_64k = himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_64k;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_124k = himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_124k;
	g_core_fp.fp_fts_ctpm_fw_upgrade_with_sys_fs_128k = himax_mcu_fts_ctpm_fw_upgrade_with_sys_fs_128k;
#endif
#ifdef CORE_SRAM
	g_core_fp.fp_sram_write = himax_mcu_sram_write;
	g_core_fp.fp_sram_verify = himax_mcu_sram_verify;
	g_core_fp.fp_get_DSRAM_data = himax_mcu_get_DSRAM_data;
#endif
#ifdef CORE_DRIVER
	g_core_fp.fp_chip_init = himax_mcu_init_ic;
#ifdef HX_RST_PIN_FUNC
	g_core_fp.fp_pin_reset = himax_mcu_pin_reset;
	g_core_fp.fp_ic_reset = himax_mcu_ic_reset;
#endif
	g_core_fp.fp_touch_information = himax_mcu_touch_information;
	g_core_fp.fp_reload_config = himax_mcu_reload_config;
	g_core_fp.fp_get_touch_data_size = himax_mcu_get_touch_data_size;
	g_core_fp.fp_hand_shaking = himax_mcu_hand_shaking;
	g_core_fp.fp_determin_diag_rawdata = himax_mcu_determin_diag_rawdata;
	g_core_fp.fp_determin_diag_storage = himax_mcu_determin_diag_storage;
	g_core_fp.fp_cal_data_len = himax_mcu_cal_data_len;
	g_core_fp.fp_diag_check_sum = himax_mcu_diag_check_sum;
	g_core_fp.fp_diag_parse_raw_data = himax_mcu_diag_parse_raw_data;
#ifdef HX_ESD_RECOVERY
	g_core_fp.fp_ic_esd_recovery = himax_mcu_ic_esd_recovery;
	g_core_fp.fp_esd_ic_reset = himax_mcu_esd_ic_reset;
#endif
#if defined(HX_SMART_WAKEUP)
	g_core_fp.fp_resend_cmd_func = himax_mcu_resend_cmd_func;
#endif
#if defined(HX_TP_PROC_GUEST_INFO)
	g_core_fp.guest_info_get_status = himax_guest_info_get_status;
	g_core_fp.read_guest_info = hx_read_guest_info;
#endif
#endif
#ifdef HX_ZERO_FLASH
	g_core_fp.fp_reload_disable = hx_dis_rload_0f;
	g_core_fp.fp_clean_sram_0f = himax_mcu_clean_sram_0f;
	g_core_fp.fp_write_sram_0f = himax_mcu_write_sram_0f;
	g_core_fp.fp_firmware_update_0f = himax_mcu_firmware_update_0f;
	g_core_fp.fp_0f_operation = himax_mcu_0f_operation;
	g_core_fp.fp_0f_operation_dirly = himax_mcu_0f_operation_dirly;
	g_core_fp.fp_0f_op_file_dirly = hx_0f_op_file_dirly;
	g_core_fp.fp_0f_esd_check = himax_mcu_0f_esd_check;
#ifdef HX_0F_DEBUG
	g_core_fp.fp_read_sram_0f = himax_mcu_read_sram_0f;
	g_core_fp.fp_read_all_sram = himax_mcu_read_all_sram;
	g_core_fp.fp_firmware_read_0f = himax_mcu_firmware_read_0f;
	g_core_fp.fp_0f_operation_check = himax_mcu_0f_operation_check;
#endif
#endif
}

int himax_mcu_in_cmd_struct_init(void)
{
	int err = 0;

	I("%s: Entering!\n", __func__);
	g_core_cmd_op = kzalloc(sizeof(struct himax_core_command_operation), GFP_KERNEL);
	if (g_core_cmd_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_fail;
	}
	g_core_cmd_op->ic_op = kzalloc(sizeof(struct ic_operation), GFP_KERNEL);
	if (g_core_cmd_op->ic_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_ic_op_fail;
	}
	g_core_cmd_op->fw_op = kzalloc(sizeof(struct fw_operation), GFP_KERNEL);
	if (g_core_cmd_op->fw_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_fw_op_fail;
	}
	g_core_cmd_op->flash_op = kzalloc(sizeof(struct flash_operation), GFP_KERNEL);
	if (g_core_cmd_op->flash_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_flash_op_fail;
	}
	g_core_cmd_op->sram_op = kzalloc(sizeof(struct sram_operation), GFP_KERNEL);
	if (g_core_cmd_op->sram_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_sram_op_fail;
	}
	g_core_cmd_op->driver_op = kzalloc(sizeof(struct driver_operation), GFP_KERNEL);
	if (g_core_cmd_op->driver_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_driver_op_fail;
	}

	pic_op = g_core_cmd_op->ic_op;
	pfw_op = g_core_cmd_op->fw_op;
	pflash_op = g_core_cmd_op->flash_op;
	psram_op = g_core_cmd_op->sram_op;
	pdriver_op = g_core_cmd_op->driver_op;
#ifdef HX_ZERO_FLASH
	g_core_cmd_op->zf_op = kzalloc(sizeof(struct zf_operation), GFP_KERNEL);
	if (g_core_cmd_op->zf_op == NULL) {
		err = -ENOMEM;
		goto err_g_core_cmd_op_zf_op_fail;
	}

	pzf_op = g_core_cmd_op->zf_op;
#endif

	g_internal_buffer = kzalloc(sizeof(uint8_t)*(HX_SPI_MTK_MAX_WRITE_SZ), GFP_KERNEL);
	himax_mcu_fp_init();

	return 0;

#ifdef HX_ZERO_FLASH
err_g_core_cmd_op_zf_op_fail:
#endif
	kfree(g_core_cmd_op->driver_op);
err_g_core_cmd_op_driver_op_fail:
	kfree(g_core_cmd_op->sram_op);
err_g_core_cmd_op_sram_op_fail:
	kfree(g_core_cmd_op->flash_op);
err_g_core_cmd_op_flash_op_fail:
	kfree(g_core_cmd_op->fw_op);
err_g_core_cmd_op_fw_op_fail:
	kfree(g_core_cmd_op->ic_op);
err_g_core_cmd_op_ic_op_fail:
	kfree(g_core_cmd_op);
err_g_core_cmd_op_fail:

	return err;
}

/*
static void himax_mcu_in_cmd_struct_free(void)
{
	pic_op = NULL;
	pfw_op = NULL;
	pflash_op = NULL;
	psram_op = NULL;
	pdriver_op = NULL;
	kfree(g_core_cmd_op);
	kfree(g_core_cmd_op->ic_op);
	kfree(g_core_cmd_op->flash_op);
	kfree(g_core_cmd_op->sram_op);
	kfree(g_core_cmd_op->driver_op);
}
*/

void himax_in_parse_assign_cmd(uint32_t addr, uint8_t *cmd, int len)
{
	/*I("%s: Entering!\n", __func__);*/
	switch (len) {
	case 1:
		cmd[0] = addr;
		/*I("%s: cmd[0] = 0x%02X\n", __func__, cmd[0]);*/
		break;

	case 2:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		/*I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X\n", __func__, cmd[0], cmd[1]);*/
		break;

	case 4:
		cmd[0] = addr % 0x100;
		cmd[1] = (addr >> 8) % 0x100;
		cmd[2] = (addr >> 16) % 0x100;
		cmd[3] = addr / 0x1000000;
		/*  I("%s: cmd[0] = 0x%02X,cmd[1] = 0x%02X,cmd[2] = 0x%02X,cmd[3] = 0x%02X\n",
			__func__, cmd[0], cmd[1], cmd[2], cmd[3]);*/
		break;

	default:
		E("%s: input length fault,len = %d!\n", __func__, len);
	}
}

void himax_mcu_in_cmd_init(void)
{
	I("%s: Entering!\n", __func__);
#ifdef CORE_IC
	himax_in_parse_assign_cmd(ic_adr_ahb_addr_byte_0, pic_op->addr_ahb_addr_byte_0, sizeof(pic_op->addr_ahb_addr_byte_0));
	himax_in_parse_assign_cmd(ic_adr_ahb_rdata_byte_0, pic_op->addr_ahb_rdata_byte_0, sizeof(pic_op->addr_ahb_rdata_byte_0));
	himax_in_parse_assign_cmd(ic_adr_ahb_access_direction, pic_op->addr_ahb_access_direction, sizeof(pic_op->addr_ahb_access_direction));
	himax_in_parse_assign_cmd(ic_adr_conti, pic_op->addr_conti, sizeof(pic_op->addr_conti));
	himax_in_parse_assign_cmd(ic_adr_incr4, pic_op->addr_incr4, sizeof(pic_op->addr_incr4));
	himax_in_parse_assign_cmd(ic_adr_sonoff_psw_lb, pic_op->adr_sonoff_psw_lb, sizeof(pic_op->adr_sonoff_psw_lb));
	himax_in_parse_assign_cmd(ic_adr_sonoff_psw_ub, pic_op->adr_sonoff_psw_ub, sizeof(pic_op->adr_sonoff_psw_ub));
	himax_in_parse_assign_cmd(ic_cmd_ahb_access_direction_read, pic_op->data_ahb_access_direction_read, sizeof(pic_op->data_ahb_access_direction_read));
	himax_in_parse_assign_cmd(ic_cmd_conti, pic_op->data_conti, sizeof(pic_op->data_conti));
	himax_in_parse_assign_cmd(ic_cmd_incr4, pic_op->data_incr4, sizeof(pic_op->data_incr4));
	himax_in_parse_assign_cmd(ic_cmd_sonoff_psw_lb, pic_op->data_sonoff_psw_lb, sizeof(pic_op->data_sonoff_psw_lb));
	himax_in_parse_assign_cmd(ic_cmd_sonoff_psw_ub, pic_op->data_sonoff_psw_ub, sizeof(pic_op->data_sonoff_psw_ub));
	himax_in_parse_assign_cmd(ic_adr_tcon_on_rst, pic_op->addr_tcon_on_rst, sizeof(pic_op->addr_tcon_on_rst));
	himax_in_parse_assign_cmd(ic_addr_adc_on_rst, pic_op->addr_adc_on_rst, sizeof(pic_op->addr_adc_on_rst));
	himax_in_parse_assign_cmd(ic_adr_psl, pic_op->addr_psl, sizeof(pic_op->addr_psl));
	himax_in_parse_assign_cmd(ic_adr_cs_central_state, pic_op->addr_cs_central_state, sizeof(pic_op->addr_cs_central_state));
	himax_in_parse_assign_cmd(ic_cmd_rst, pic_op->data_rst, sizeof(pic_op->data_rst));
#endif
#ifdef CORE_FW
	himax_in_parse_assign_cmd(fw_addr_system_reset, pfw_op->addr_system_reset, sizeof(pfw_op->addr_system_reset));
	himax_in_parse_assign_cmd(fw_addr_safe_mode_release_pw, pfw_op->addr_safe_mode_release_pw, sizeof(pfw_op->addr_safe_mode_release_pw));
	himax_in_parse_assign_cmd(fw_addr_ctrl_fw, pfw_op->addr_ctrl_fw_isr, sizeof(pfw_op->addr_ctrl_fw_isr));
	himax_in_parse_assign_cmd(fw_addr_flag_reset_event, pfw_op->addr_flag_reset_event, sizeof(pfw_op->addr_flag_reset_event));
	himax_in_parse_assign_cmd(fw_addr_hsen_enable, pfw_op->addr_hsen_enable, sizeof(pfw_op->addr_hsen_enable));
	himax_in_parse_assign_cmd(fw_addr_smwp_enable, pfw_op->addr_smwp_enable, sizeof(pfw_op->addr_smwp_enable));
	himax_in_parse_assign_cmd(fw_addr_program_reload_from, pfw_op->addr_program_reload_from, sizeof(pfw_op->addr_program_reload_from));
	himax_in_parse_assign_cmd(fw_addr_program_reload_to, pfw_op->addr_program_reload_to, sizeof(pfw_op->addr_program_reload_to));
	himax_in_parse_assign_cmd(fw_addr_program_reload_page_write, pfw_op->addr_program_reload_page_write, sizeof(pfw_op->addr_program_reload_page_write));
	himax_in_parse_assign_cmd(fw_addr_raw_out_sel, pfw_op->addr_raw_out_sel, sizeof(pfw_op->addr_raw_out_sel));
	himax_in_parse_assign_cmd(fw_addr_reload_status, pfw_op->addr_reload_status, sizeof(pfw_op->addr_reload_status));
	himax_in_parse_assign_cmd(fw_addr_reload_crc32_result, pfw_op->addr_reload_crc32_result, sizeof(pfw_op->addr_reload_crc32_result));
	himax_in_parse_assign_cmd(fw_addr_reload_addr_from, pfw_op->addr_reload_addr_from, sizeof(pfw_op->addr_reload_addr_from));
	himax_in_parse_assign_cmd(fw_addr_reload_addr_cmd_beat, pfw_op->addr_reload_addr_cmd_beat, sizeof(pfw_op->addr_reload_addr_cmd_beat));
	himax_in_parse_assign_cmd(fw_addr_selftest_addr_en, pfw_op->addr_selftest_addr_en, sizeof(pfw_op->addr_selftest_addr_en));
	himax_in_parse_assign_cmd(fw_addr_criteria_addr, pfw_op->addr_criteria_addr, sizeof(pfw_op->addr_criteria_addr));
	himax_in_parse_assign_cmd(fw_addr_set_frame_addr, pfw_op->addr_set_frame_addr, sizeof(pfw_op->addr_set_frame_addr));
	himax_in_parse_assign_cmd(fw_addr_selftest_result_addr, pfw_op->addr_selftest_result_addr, sizeof(pfw_op->addr_selftest_result_addr));
	himax_in_parse_assign_cmd(fw_addr_sorting_mode_en, pfw_op->addr_sorting_mode_en, sizeof(pfw_op->addr_sorting_mode_en));
	himax_in_parse_assign_cmd(fw_addr_fw_mode_status, pfw_op->addr_fw_mode_status, sizeof(pfw_op->addr_fw_mode_status));
	himax_in_parse_assign_cmd(fw_addr_icid_addr, pfw_op->addr_icid_addr, sizeof(pfw_op->addr_icid_addr));
	himax_in_parse_assign_cmd(fw_addr_trigger_addr, pfw_op->addr_trigger_addr, sizeof(pfw_op->addr_trigger_addr));
	himax_in_parse_assign_cmd(fw_addr_fw_ver_addr, pfw_op->addr_fw_ver_addr, sizeof(pfw_op->addr_fw_ver_addr));
	himax_in_parse_assign_cmd(fw_addr_fw_cfg_addr, pfw_op->addr_fw_cfg_addr, sizeof(pfw_op->addr_fw_cfg_addr));
	himax_in_parse_assign_cmd(fw_addr_fw_vendor_addr, pfw_op->addr_fw_vendor_addr, sizeof(pfw_op->addr_fw_vendor_addr));
	himax_in_parse_assign_cmd(fw_addr_cus_info, pfw_op->addr_cus_info, sizeof(pfw_op->addr_cus_info));
	himax_in_parse_assign_cmd(fw_addr_proj_info, pfw_op->addr_proj_info, sizeof(pfw_op->addr_proj_info));
	himax_in_parse_assign_cmd(fw_addr_fw_state_addr, pfw_op->addr_fw_state_addr, sizeof(pfw_op->addr_fw_state_addr));
	himax_in_parse_assign_cmd(fw_addr_fw_dbg_msg_addr, pfw_op->addr_fw_dbg_msg_addr, sizeof(pfw_op->addr_fw_dbg_msg_addr));
	himax_in_parse_assign_cmd(fw_addr_chk_fw_status, pfw_op->addr_chk_fw_status, sizeof(pfw_op->addr_chk_fw_status));
	himax_in_parse_assign_cmd(fw_addr_dd_handshak_addr, pfw_op->addr_dd_handshak_addr, sizeof(pfw_op->addr_dd_handshak_addr));
	himax_in_parse_assign_cmd(fw_addr_dd_data_addr, pfw_op->addr_dd_data_addr, sizeof(pfw_op->addr_dd_data_addr));
	himax_in_parse_assign_cmd(fw_data_system_reset, pfw_op->data_system_reset, sizeof(pfw_op->data_system_reset));
	himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_active, pfw_op->data_safe_mode_release_pw_active, sizeof(pfw_op->data_safe_mode_release_pw_active));
	himax_in_parse_assign_cmd(fw_data_clear, pfw_op->data_clear, sizeof(pfw_op->data_clear));
	himax_in_parse_assign_cmd(fw_data_clear, pfw_op->data_clear, sizeof(pfw_op->data_clear));
	himax_in_parse_assign_cmd(fw_data_fw_stop, pfw_op->data_fw_stop, sizeof(pfw_op->data_fw_stop));
	himax_in_parse_assign_cmd(fw_data_safe_mode_release_pw_reset, pfw_op->data_safe_mode_release_pw_reset, sizeof(pfw_op->data_safe_mode_release_pw_reset));
	himax_in_parse_assign_cmd(fw_data_program_reload_start, pfw_op->data_program_reload_start, sizeof(pfw_op->data_program_reload_start));
	himax_in_parse_assign_cmd(fw_data_program_reload_compare, pfw_op->data_program_reload_compare, sizeof(pfw_op->data_program_reload_compare));
	himax_in_parse_assign_cmd(fw_data_program_reload_break, pfw_op->data_program_reload_break, sizeof(pfw_op->data_program_reload_break));
	himax_in_parse_assign_cmd(fw_data_selftest_request, pfw_op->data_selftest_request, sizeof(pfw_op->data_selftest_request));
	himax_in_parse_assign_cmd(fw_data_criteria_aa_top, pfw_op->data_criteria_aa_top, sizeof(pfw_op->data_criteria_aa_top));
	himax_in_parse_assign_cmd(fw_data_criteria_aa_bot, pfw_op->data_criteria_aa_bot, sizeof(pfw_op->data_criteria_aa_bot));
	himax_in_parse_assign_cmd(fw_data_criteria_key_top, pfw_op->data_criteria_key_top, sizeof(pfw_op->data_criteria_key_top));
	himax_in_parse_assign_cmd(fw_data_criteria_key_bot, pfw_op->data_criteria_key_bot, sizeof(pfw_op->data_criteria_key_bot));
	himax_in_parse_assign_cmd(fw_data_criteria_avg_top, pfw_op->data_criteria_avg_top, sizeof(pfw_op->data_criteria_avg_top));
	himax_in_parse_assign_cmd(fw_data_criteria_avg_bot, pfw_op->data_criteria_avg_bot, sizeof(pfw_op->data_criteria_avg_bot));
	himax_in_parse_assign_cmd(fw_data_set_frame, pfw_op->data_set_frame, sizeof(pfw_op->data_set_frame));
	himax_in_parse_assign_cmd(fw_data_selftest_ack_hb, pfw_op->data_selftest_ack_hb, sizeof(pfw_op->data_selftest_ack_hb));
	himax_in_parse_assign_cmd(fw_data_selftest_ack_lb, pfw_op->data_selftest_ack_lb, sizeof(pfw_op->data_selftest_ack_lb));
	himax_in_parse_assign_cmd(fw_data_selftest_pass, pfw_op->data_selftest_pass, sizeof(pfw_op->data_selftest_pass));
	himax_in_parse_assign_cmd(fw_data_normal_cmd, pfw_op->data_normal_cmd, sizeof(pfw_op->data_normal_cmd));
	himax_in_parse_assign_cmd(fw_data_normal_status, pfw_op->data_normal_status, sizeof(pfw_op->data_normal_status));
	himax_in_parse_assign_cmd(fw_data_sorting_cmd, pfw_op->data_sorting_cmd, sizeof(pfw_op->data_sorting_cmd));
	himax_in_parse_assign_cmd(fw_data_sorting_status, pfw_op->data_sorting_status, sizeof(pfw_op->data_sorting_status));
	himax_in_parse_assign_cmd(fw_data_dd_request, pfw_op->data_dd_request, sizeof(pfw_op->data_dd_request));
	himax_in_parse_assign_cmd(fw_data_dd_ack, pfw_op->data_dd_ack, sizeof(pfw_op->data_dd_ack));
	himax_in_parse_assign_cmd(fw_data_idle_dis_pwd, pfw_op->data_idle_dis_pwd, sizeof(pfw_op->data_idle_dis_pwd));
	himax_in_parse_assign_cmd(fw_data_idle_en_pwd, pfw_op->data_idle_en_pwd, sizeof(pfw_op->data_idle_en_pwd));
	himax_in_parse_assign_cmd(fw_data_rawdata_ready_hb, pfw_op->data_rawdata_ready_hb, sizeof(pfw_op->data_rawdata_ready_hb));
	himax_in_parse_assign_cmd(fw_data_rawdata_ready_lb, pfw_op->data_rawdata_ready_lb, sizeof(pfw_op->data_rawdata_ready_lb));
	himax_in_parse_assign_cmd(fw_addr_ahb_addr, pfw_op->addr_ahb_addr, sizeof(pfw_op->addr_ahb_addr));
	himax_in_parse_assign_cmd(fw_data_ahb_dis, pfw_op->data_ahb_dis, sizeof(pfw_op->data_ahb_dis));
	himax_in_parse_assign_cmd(fw_data_ahb_en, pfw_op->data_ahb_en, sizeof(pfw_op->data_ahb_en));
	himax_in_parse_assign_cmd(fw_addr_event_addr, pfw_op->addr_event_addr, sizeof(pfw_op->addr_event_addr));
	himax_in_parse_assign_cmd(fw_usb_detect_addr, pfw_op->addr_usb_detect, sizeof(pfw_op->addr_usb_detect));
#endif
#ifdef CORE_FLASH
	himax_in_parse_assign_cmd(flash_addr_spi200_trans_fmt, pflash_op->addr_spi200_trans_fmt, sizeof(pflash_op->addr_spi200_trans_fmt));
	himax_in_parse_assign_cmd(flash_addr_spi200_trans_ctrl, pflash_op->addr_spi200_trans_ctrl, sizeof(pflash_op->addr_spi200_trans_ctrl));
	himax_in_parse_assign_cmd(flash_addr_spi200_cmd, pflash_op->addr_spi200_cmd, sizeof(pflash_op->addr_spi200_cmd));
	himax_in_parse_assign_cmd(flash_addr_spi200_addr, pflash_op->addr_spi200_addr, sizeof(pflash_op->addr_spi200_addr));
	himax_in_parse_assign_cmd(flash_addr_spi200_data, pflash_op->addr_spi200_data, sizeof(pflash_op->addr_spi200_data));
	himax_in_parse_assign_cmd(flash_addr_spi200_bt_num, pflash_op->addr_spi200_bt_num, sizeof(pflash_op->addr_spi200_bt_num));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_fmt, pflash_op->data_spi200_trans_fmt, sizeof(pflash_op->data_spi200_trans_fmt));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_ctrl_1, pflash_op->data_spi200_trans_ctrl_1, sizeof(pflash_op->data_spi200_trans_ctrl_1));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_ctrl_2, pflash_op->data_spi200_trans_ctrl_2, sizeof(pflash_op->data_spi200_trans_ctrl_2));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_ctrl_3, pflash_op->data_spi200_trans_ctrl_3, sizeof(pflash_op->data_spi200_trans_ctrl_3));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_ctrl_4, pflash_op->data_spi200_trans_ctrl_4, sizeof(pflash_op->data_spi200_trans_ctrl_4));
	himax_in_parse_assign_cmd(flash_data_spi200_trans_ctrl_5, pflash_op->data_spi200_trans_ctrl_5, sizeof(pflash_op->data_spi200_trans_ctrl_5));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_1, pflash_op->data_spi200_cmd_1, sizeof(pflash_op->data_spi200_cmd_1));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_2, pflash_op->data_spi200_cmd_2, sizeof(pflash_op->data_spi200_cmd_2));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_3, pflash_op->data_spi200_cmd_3, sizeof(pflash_op->data_spi200_cmd_3));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_4, pflash_op->data_spi200_cmd_4, sizeof(pflash_op->data_spi200_cmd_4));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_5, pflash_op->data_spi200_cmd_5, sizeof(pflash_op->data_spi200_cmd_5));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_6, pflash_op->data_spi200_cmd_6, sizeof(pflash_op->data_spi200_cmd_6));
	himax_in_parse_assign_cmd(flash_data_spi200_cmd_7, pflash_op->data_spi200_cmd_7, sizeof(pflash_op->data_spi200_cmd_7));
	himax_in_parse_assign_cmd(flash_data_spi200_addr, pflash_op->data_spi200_addr, sizeof(pflash_op->data_spi200_addr));
#endif
#ifdef CORE_SRAM
	/* sram start*/
	himax_in_parse_assign_cmd(sram_adr_mkey, psram_op->addr_mkey, sizeof(psram_op->addr_mkey));
	himax_in_parse_assign_cmd(sram_adr_rawdata_addr, psram_op->addr_rawdata_addr, sizeof(psram_op->addr_rawdata_addr));
	himax_in_parse_assign_cmd(sram_adr_rawdata_end, psram_op->addr_rawdata_end, sizeof(psram_op->addr_rawdata_end));
	himax_in_parse_assign_cmd(sram_cmd_conti, psram_op->data_conti, sizeof(psram_op->data_conti));
	himax_in_parse_assign_cmd(sram_cmd_fin, psram_op->data_fin, sizeof(psram_op->data_fin));
	himax_in_parse_assign_cmd(sram_passwrd_start, psram_op->passwrd_start, sizeof(psram_op->passwrd_start));
	himax_in_parse_assign_cmd(sram_passwrd_end, psram_op->passwrd_end, sizeof(psram_op->passwrd_end));
	/* sram end*/
#endif
#ifdef CORE_DRIVER
	himax_in_parse_assign_cmd(driver_addr_fw_define_flash_reload, pdriver_op->addr_fw_define_flash_reload, sizeof(pdriver_op->addr_fw_define_flash_reload));
	himax_in_parse_assign_cmd(driver_addr_fw_define_2nd_flash_reload, pdriver_op->addr_fw_define_2nd_flash_reload, sizeof(pdriver_op->addr_fw_define_2nd_flash_reload));
	himax_in_parse_assign_cmd(driver_addr_fw_define_int_is_edge, pdriver_op->addr_fw_define_int_is_edge, sizeof(pdriver_op->addr_fw_define_int_is_edge));
	himax_in_parse_assign_cmd(driver_addr_fw_define_rxnum_txnum_maxpt, pdriver_op->addr_fw_define_rxnum_txnum_maxpt, sizeof(pdriver_op->addr_fw_define_rxnum_txnum_maxpt));
	himax_in_parse_assign_cmd(driver_addr_fw_define_xy_res_enable, pdriver_op->addr_fw_define_xy_res_enable, sizeof(pdriver_op->addr_fw_define_xy_res_enable));
	himax_in_parse_assign_cmd(driver_addr_fw_define_x_y_res, pdriver_op->addr_fw_define_x_y_res, sizeof(pdriver_op->addr_fw_define_x_y_res));
	himax_in_parse_assign_cmd(driver_data_fw_define_flash_reload_dis, pdriver_op->data_fw_define_flash_reload_dis, sizeof(pdriver_op->data_fw_define_flash_reload_dis));
	himax_in_parse_assign_cmd(driver_data_fw_define_flash_reload_en, pdriver_op->data_fw_define_flash_reload_en, sizeof(pdriver_op->data_fw_define_flash_reload_en));
	himax_in_parse_assign_cmd(driver_data_fw_define_rxnum_txnum_maxpt_sorting, pdriver_op->data_fw_define_rxnum_txnum_maxpt_sorting, sizeof(pdriver_op->data_fw_define_rxnum_txnum_maxpt_sorting));
	himax_in_parse_assign_cmd(driver_data_fw_define_rxnum_txnum_maxpt_normal, pdriver_op->data_fw_define_rxnum_txnum_maxpt_normal, sizeof(pdriver_op->data_fw_define_rxnum_txnum_maxpt_normal));
#endif
#ifdef HX_ZERO_FLASH
	himax_in_parse_assign_cmd(zf_addr_dis_flash_reload, pzf_op->addr_dis_flash_reload, sizeof(pzf_op->addr_dis_flash_reload));
	himax_in_parse_assign_cmd(zf_data_dis_flash_reload, pzf_op->data_dis_flash_reload, sizeof(pzf_op->data_dis_flash_reload));
	himax_in_parse_assign_cmd(zf_addr_system_reset, pzf_op->addr_system_reset, sizeof(pzf_op->addr_system_reset));
	himax_in_parse_assign_cmd(zf_data_system_reset, pzf_op->data_system_reset, sizeof(pzf_op->data_system_reset));
	himax_in_parse_assign_cmd(zf_data_sram_start_addr, pzf_op->data_sram_start_addr, sizeof(pzf_op->data_sram_start_addr));
	himax_in_parse_assign_cmd(zf_data_sram_clean, pzf_op->data_sram_clean, sizeof(pzf_op->data_sram_clean));
	himax_in_parse_assign_cmd(zf_data_cfg_info, pzf_op->data_cfg_info, sizeof(pzf_op->data_cfg_info));
	himax_in_parse_assign_cmd(zf_data_fw_cfg_1, pzf_op->data_fw_cfg_1, sizeof(pzf_op->data_fw_cfg_1));
	himax_in_parse_assign_cmd(zf_data_fw_cfg_2, pzf_op->data_fw_cfg_2, sizeof(pzf_op->data_fw_cfg_2));
	himax_in_parse_assign_cmd(zf_data_fw_cfg_2, pzf_op->data_fw_cfg_3, sizeof(pzf_op->data_fw_cfg_3));
	himax_in_parse_assign_cmd(zf_data_adc_cfg_1, pzf_op->data_adc_cfg_1, sizeof(pzf_op->data_adc_cfg_1));
	himax_in_parse_assign_cmd(zf_data_adc_cfg_2, pzf_op->data_adc_cfg_2, sizeof(pzf_op->data_adc_cfg_2));
	himax_in_parse_assign_cmd(zf_data_adc_cfg_3, pzf_op->data_adc_cfg_3, sizeof(pzf_op->data_adc_cfg_3));
	himax_in_parse_assign_cmd(zf_data_map_table, pzf_op->data_map_table, sizeof(pzf_op->data_map_table));
	himax_in_parse_assign_cmd(zf_data_mode_switch, pzf_op->data_mode_switch, sizeof(pzf_op->data_mode_switch));
	himax_in_parse_assign_cmd(zf_addr_sts_chk, pzf_op->addr_sts_chk, sizeof(pzf_op->addr_sts_chk));
	himax_in_parse_assign_cmd(zf_data_activ_sts, pzf_op->data_activ_sts, sizeof(pzf_op->data_activ_sts));
	himax_in_parse_assign_cmd(zf_addr_activ_relod, pzf_op->addr_activ_relod, sizeof(pzf_op->addr_activ_relod));
	himax_in_parse_assign_cmd(zf_data_activ_in, pzf_op->data_activ_in, sizeof(pzf_op->data_activ_in));
#endif
}

/* init end*/
#endif
