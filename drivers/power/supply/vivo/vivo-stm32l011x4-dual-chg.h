#ifndef __VIVO_STM32L011X4_DUAL_CHG_H
#define __VIVO_STM32L011X4_DUAL_CHG_H


#define I2C_BURN_REG			0x01
#define PROTOCOL_BOOT_START		0x55
#define PROTOCOL_APP_START		0xAA
#define TRANSFER_BATCH_SIZE		250//256
#define TRANSFER_RETRY_TIME		3
#define BATCH_ALIGN			4
#define FLASH_APP_ADDR_SHIFT		0x08000C00//0x08001000//0x08002000
#define FLASH_PAGE_SIZE			128
#define INSIDE_BATCH			128
#define OUTSIDE_BATCH			64
#define MAX_FW_SIZE			15360U
#define FW_VERSION_DIVIDE		0x80
#define FW_VERSION_MAX			0xFF
#define FW_SIZE_MAX_KB			20//13
#define RT9759_DEVICE_INFO		0x08

#define OLD_CAM_NOTIFY			0

#define CMD_NO_PARAM			0
#define CMD_ONE_PARAM			1
#define CMD_ERASE_PARAM			6
#define CMD_CRC_PARAM			12
typedef enum {
	SERIAL_BURN			= 0x0,
	I2C_BURN			= 0x1,
} BURN_TYPE;

typedef enum {
	REPLY_STATUS_OK			= 0x0,
	REPLY_STATUS_ERR		= 0x1,
} REPLY_STATUS;

typedef enum {
	NO_ERROR			= 0x0,
	WRITE_ERROR			= 0x1,
	READ_ERROR			= 0x2,
	REPLY_ERROR			= 0x3,
} ERROR_TYPE;

typedef enum {
	BURN_CMD_UNKNOW			= 0x00,
	BURN_CMD_RESERVED		= 0x01,
	BURN_CMD_READ			= 0x02,
	BURN_CMD_WRITE			= 0x03,
	BURN_CMD_ERASE			= 0x04,
	BURN_CMD_RESET			= 0x05,
	BURN_CMD_JUMP			= 0x06,
	BURN_CMD_RETRY			= 0x07,
	BURN_CMD_REPLY			= 0x08,
	BURN_CMD_CRC			= 0x09,
	BURN_CMD_DONE			= 0x0A,
	BURN_CMD_CHECK			= 0x0B,
	BURN_CMD_BOOT_VERSION		= 0x0C,
	BURN_CMD_UPDATE			= 0x5A,
	BURN_CMD_APP_UPDATE		= 0xA5,
	BURN_CMD_APP_VERSION		= 0xA6,
	BURN_CMD_TEST_I2C		= 0xB0,
	BURN_CMD_TEST_SERIAL		= 0xB1,
} BURN_CMD;


typedef enum {
	TEST_DIRECT_CHARGE_SWITCH	= 0xC0,
	TEST_SET_VOLTAGE		= 0xC1,
	TEST_SET_CURRENT		= 0xC2,
	TEST_SET_IMPEDANCE		= 0xC3,
	TEST_GET_INSIDE_VERSION		= 0xC4,
	TEST_GET_OUTSIDE_VERSION	= 0xC5,
} TEST_OPT;


#define I2C_BURN_HEAD_SIZE		3
struct i2c_wrap_head {
	u8 reg;
	u8 start;
	u8 length;
};

#define COMMAND_WIDTH			1
#define FLASH_ADDR_WIDTH		4
#define FLASH_BATCH_WIDTH		1
#define FLASH_READ_BATCH_WIDTH		1
#define CHKSUM_WIDTH			1
struct burn_wrap {
	struct i2c_wrap_head head;
	u8 command;
	u32 shift;
	u16 batch;
	u8 *data;
	u8 chksum;
};

#define REPLY_DEF_LEN			3
struct reply_wrap {
	struct i2c_wrap_head head;
	u8 reply;
	u8 command;
	u8 status;
	u8 *data;
	u8 chksum;
};

struct mcu_fw {
	int id;
	u8 *data;
	int size;
	u32 crc;
	u32 version;
	const char *type;
};

/* voter define */
#define CALLING_EVENT			"CALLING_EVENT"
#define DCHG_RESTART_EVENT		"DCHG_RESTART_EVENT"
#define DCHG_CAM_EVENT			"DCHG_CAM_EVENT"


#define TOTAL_DCHG_FCC_LIMIT_MAX	"TOTAL_DCHG_FCC_LIMIT_MAX"
#define USER_FCC_VOTER			"USER_FCC_VOTER"
#define MASTER_FCC_VOTER		"MASTER_FCC_VOTER"
#define STANDALONE_FCC_VOTER		"STANDALONE_FCC_VOTER"
#define FACTORY_FCC_VOTER		"FACTORY_FCC_VOTER"
#define CABLE_R_FCC_VOTER		"CABLE_R_FCC_VOTER"
#define CABLE_ID_FCC_VOTER		"CABLE_ID_FCC_VOTER"
#define CHG_EXCETION_FCC_VOTER		"CHG_EXCETION_FCC_VOTER"



/////////////////////////////////////////////////////////////////////
/* Configuration registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define FAST_CHG_CURRENT_MASK		stm32l011_MASK(7, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK	stm32l011_MASK(3, 0)

/* IRQ status registers r*/
#define MCU_IRQ_A_REG			0x00
#define MCU_IRA_INIT_DONE		BIT(7)
#define MCU_IRA_HANDSHAKE_SUCCESS	BIT(6)
#define EVENT_HANDSHAKE_FAIL		BIT(5)
#define MCU_IRA_PMI_SUSPEND		BIT(4)
#define MCU_IRA_VBAT_GOOD		BIT(3)
#define MCU_IRA_POWER_MATCHED		BIT(1)
#define MCU_IRA_HIGH_VOL_REQUEST_SUCCESS	BIT(0)

#define MCU_IRQ_B_REG			0x01
#define MCU_IRB_DCHG_EXIT		BIT(0)
#define EXIT_HIGH_VBAT			BIT(2)

#define MCU_IRQ_C_REG			0x02
#define MCU_IRQ_D_REG			0x03

/*r/w*/
#define MCU_REG_04			0x04
#define MCU_PMI_SUSPENDED_Pos		(0U)
#define MCU_PMI_SUSPENDED_Msk		(0x1U << MCU_PMI_SUSPENDED_Pos)
#define MCU_PMI_SUSPENDED		MCU_PMI_SUSPENDED_Msk

#define MCU_SET_VOLTAGE_POS		(1U)
#define MCU_SET_VOLTAGE			(0x1U << MCU_SET_VOLTAGE_POS)

#define MCU_SET_CURRENT_POS		(2U)
#define MCU_SET_CURRENT			(0x1U << MCU_SET_CURRENT_POS)

#define MCU_SET_IMPEDANCE_POS		(3U)
#define MCU_SET_IMPEDANCE		(0x1U << MCU_SET_IMPEDANCE_POS)

#define MCU_DCHG_SWITCH_POS		(4U)
#define MCU_DCHG_SWITCH			(0x1U << MCU_DCHG_SWITCH_POS)

#define MCU_SET_ADAPTER_RESET_POS	(5U)
#define MCU_SET_ADAPTER_RESET		(0x1U << MCU_SET_ADAPTER_RESET_POS)

#define MCU_SET_AT_BOOT_MODE_POS	(6U)
#define MCU_SET_AT_BOOT_MODE		(0x1U << MCU_SET_AT_BOOT_MODE_POS)

#define MCU_SET_ENABLE_SLAVE_CHG_POS	(7U)
#define MCU_SET_ENABLE_SLAVE_CHG	(0x1U << MCU_SET_ENABLE_SLAVE_CHG_POS)
/* record exit chg reason */
#define MCU_REG_05			0x05

#define MCU_REG_06			0x06
#define EVENT_SLAVE_CHG_ENABLE_SUCCESS_Pos	(0U)
#define EVENT_SLAVE_CHG_ENABLE_SUCCESS_Msk	(0x1U << EVENT_SLAVE_CHG_ENABLE_SUCCESS_Pos)
#define EVENT_SLAVE_CHG_ENABLE_SUCCESS		EVENT_SLAVE_CHG_ENABLE_SUCCESS_Msk

#define EVENT_SLAVE_CHG_ENABLE_FAIL_Pos	(1U)
#define EVENT_SLAVE_CHG_ENABLE_FAIL_Msk	(0x1U << EVENT_SLAVE_CHG_ENABLE_FAIL_Pos)
#define EVENT_SLAVE_CHG_ENABLE_FAIL	EVENT_SLAVE_CHG_ENABLE_FAIL_Msk
/*watchdog reg,kick by Ap R/W*/
#define MCU_REG_07			0x07

#define MCU_REG_08			0x08
#define MCU_SET_FACTORY_MODE_POS	(0U)
#define MCU_SET_FACTORY_MODE		(0x1U << MCU_SET_FACTORY_MODE_POS)
#define MCU_SET_CV_MODE_POS		(1U)
#define MCU_SET_CV_MODE			(0x1U << MCU_SET_CV_MODE_POS)
#define MCU_FFC_BAT_HIGH_POS		(2U)
#define MCU_FFC_BAT_HIGH		(0x1U << MCU_FFC_BAT_HIGH_POS)
#define MCU_SET_SHUTDOWN_POWER_POS      (5U)
#define MCU_SET_SHUTDOWN_POWER          (0x1U << MCU_SET_SHUTDOWN_POWER_POS)


#define MCU_REGESTER_START		0x00
#define MCU_REGESTER_END		0x0C
#define MCU_REGESTER_TATOL		(MCU_REGESTER_END - MCU_REGESTER_START + 1)

#define MCU_VERSION_START		0x0D
#define MCU_VERSION_END			0x0F
#define MCU_VERSION_TATOL		(MCU_VERSION_END - MCU_VERSION_START + 1)

#define MCU_ADC_START			0x10
#define MCU_ADC_END			0x17
#define MCU_ADC_TATOL			(MCU_ADC_END - MCU_ADC_START + 1)
#define MCU_LOG1_START			0x18
#define MCU_LOG1_END			0x1F
#define MCU_LOG1_TATOL			(MCU_LOG1_END - MCU_LOG1_START + 1)
#define MCU_BQ25970_REGS_START		0x20
#define MCU_BQ25970_REGS_END		0x52
#define MCU_BQ25970_REGS_TATOL		(MCU_BQ25970_REGS_END - MCU_BQ25970_REGS_START + 1)

#define MCU_BQ25970_REGS2_START		0x68
#define MCU_BQ25970_REGS2_END		0x96
#define MCU_BQ25970_REGS2_TATOL		(MCU_BQ25970_REGS2_END - MCU_BQ25970_REGS2_START + 1)

#define MCU_BQ25970_REGS3_START		0x97
#define MCU_BQ25970_REGS3_END		0xC5
#define MCU_BQ25970_REGS3_TATOL		(MCU_BQ25970_REGS3_END - MCU_BQ25970_REGS3_START + 1)

#define MCU_EXCEPTON_LOG_START		0x53
#define MCU_EXCEPTON_LOG_EN		 0x65
#define MCU_EXCEPTON_LOG_TATOL		(MCU_EXCEPTON_LOG_END - MCU_EXCEPTON_LOG_START + 1)
#define MCU_BUFFERSIZE			(MCU_BQ25970_REGS3_END + 1)

#define MCU_BTB_CV_VOLTAGE		(MCU_BQ25970_REGS_START + 0x00)	//base=4300mV+*
#define MCU_ADAPTER_CURRENT		(MCU_BQ25970_REGS_START + 0x01)
#define MCU_ADAPTER_IMPEDANCE		(MCU_BQ25970_REGS_START + 0x02)
#define MCU_ADAPTER_VERSION		(MCU_BQ25970_REGS_START + 0x03)
#define MCU_FG_VBAT_CV_VOLTAGE		(MCU_BQ25970_REGS_START + 0x25)	//base=4300mV+*


#define MCU_ADAPTER_POWERDERATE		0xDD

#define MCU_ADAPTER_POWER		(MCU_BQ25970_REGS_START + 0x26)
#define MCU_ADAPTER_VENDOR		(MCU_BQ25970_REGS_START + 0x28)
#define MCU_FG_VBAT_HIGH		(MCU_BQ25970_REGS_START + 0x29)
#define MCU_FG_VBAT_LOW			(MCU_BQ25970_REGS_START + 0x2A)

/*25970*/
#define BQ25970_REG_06			(MCU_BQ25970_REGS_START + 0x06)
#define BQ25970_CHGEN_MASK		0x10
#define BQ25970_CHGEN_SHIFT		4
#define BQ25970_REG_13			(MCU_BQ25970_REGS_START + 0x13)
#define BQ25970_REG_14			(MCU_BQ25970_REGS_START + 0x14)
#define BQ25970_VBUS_ADC_MASK		0x7F
#define BQ25970_VBUS_ADC_SHIFT		0
#define BQ25970_REG_15			(MCU_BQ25970_REGS_START + 0x15)
#define BQ25970_REG_16			(MCU_BQ25970_REGS_START + 0x16)
#define BQ25970_IBUS_ADC_MASK		0x7F
#define BQ25970_IBUS_ADC_SHIFT		0
#define BQ25970_REG_1B			(MCU_BQ25970_REGS_START + 0x1B)
#define BQ25970_REG_1C			(MCU_BQ25970_REGS_START + 0x1C)
#define BQ25970_VBAT_ADC_MASK		0x7F

#define BQ25970_REG_1F			(MCU_BQ25970_REGS_START + 0x1F)
#define BQ25970_REG_20			(MCU_BQ25970_REGS_START + 0x20)
#define BQ25970_TBUS_ADC_MASK		0x7F

#define BQ25970_REG_21			(MCU_BQ25970_REGS_START + 0x21)
#define BQ25970_REG_22			(MCU_BQ25970_REGS_START + 0x22)
#define BQ25970_TBAT_ADC_MASK		0x7F

#define BQ25970_REG_23			(MCU_BQ25970_REGS_START + 0x21)
#define BQ25970_REG_24			(MCU_BQ25970_REGS_START + 0x22)
#define BQ25970_REG_2B			(MCU_BQ25970_REGS_START + 0x2B)
#define BQ25970_REG_2C			(MCU_BQ25970_REGS_START + 0x2C)
#define BQ25970_REG_2D			(MCU_BQ25970_REGS_START + 0x2D)

/*mcu globe paramters*/
#define MCU_ADAPTER_CONN_TEMP		(MCU_LOG1_START + 0)
#define MCU_ADAPTER_TEMP		(MCU_LOG1_START + 2)
#define MCU_CABLEROM			(MCU_LOG1_START + 4)
#define MCU_CABLE_MATCHED		(MCU_LOG1_START + 6)
/*mcu adc*/
//#define MCU_PCB_CONN_TMEP		(MCU_ADC_START + 0)
#define MCU_BAT_CONN_TMEP		(MCU_ADC_START + 0)
#define MCU_DP_MV			(MCU_ADC_START + 2)
#define MCU_DM_MV			(MCU_ADC_START + 4)
/*mcu communication exception count*/
#define ANALOG_I2C_READ_COUNT		(MCU_EXCEPTON_LOG_START + 0)
#define ANALOG_I2C_WRITE_COUNT		(MCU_EXCEPTON_LOG_START + 0x02)
#define UART_TX_COUNT			(MCU_EXCEPTON_LOG_START + 0x04)
#define UART_RX_COUNT			(MCU_EXCEPTON_LOG_START + 0x06)
#define ADAPTER_I2C_READ_COUNT		(MCU_EXCEPTON_LOG_START + 0x08)
#define ADAPTER_I2C_WRITE_COUNT		(MCU_EXCEPTON_LOG_START + 0x0A)
#define ADAPTER_UART_TX_COUNT		(MCU_EXCEPTON_LOG_START + 0x0C)
#define ADAPTER_UART_RX_COUNT		(MCU_EXCEPTON_LOG_START + 0x0E)
#define ADAPTER_STATUS			(MCU_EXCEPTON_LOG_START + 0x10)

#define MCU_MAIN_CHG_STATE_MACHINE	0xC6
#define AP_CONTROL_MCU_UART_STATUS_REG	0xC7// return the uart_CMD value to indicate the UART RX OK. otherwise [0xF0: unknow_error;   0xF1:UART_TX_error;  0xF2: UART_RX_error;  0xF3: DP_DM_short_error ]
#define AP_CONTROL_MCU_UART_TX_DATA_REG	0xC8// length = 20, {cmd, tx_data_lenght, tx_data1(value1), ...}
#define AP_CONTROL_MCU_UART_RX_DATA_REG	0xDC

/*charger FSM*/
#define CHARGER_INITED			0x00
#define CHARGER_PRE_HANDSHAKING		0x01
#define CHARGER_HANDSHAKING		0x02
#define CHARGER_GET_ADAPTER_INFO	0x03
#define CHARGER_ADAPTER_POWER_MATCH	0x04
#define	CHARGER_IDENTIFICATION		0x05
#define CHARGER_HIGH_VOL_MODE_REQUEST	0x06
#define CHARGER_DIRECT_CHARGING		0x07
#define CHARGER_POST_DIRECT_CHARGING	0x08
#define CHARGER_EXIT_DIRECT_CHARGING	0x09
#define CHARGER_HANDSHAKING_FAIL	0x0A
#define CHARGER_SUSPENDED		0x0B
#define CHARGER_IDLE			0x0C
#define	CHARGER_POWER_OFF		0x0D

#define EXIT_ADAPTER_IDENTIFICATION_ERROR	BIT(2)


/*enum statement*/
typedef enum {
	VENDOR_VIVO,
	VENDOR_DIALOG,
	VENDOR_NXP,
	VENDOR_RICHTEK,
} DChg_adapter_Vendor;

enum {
	unknow_Bootloader = 0,
	lierda_Bootloader = 1,
	vivo_Bootloader = 2,
} BOOTLOADER_VENDOR;

enum {
	lierda_MCU_Bootloader_Version = 0x21,
	vivo_MCU_Bootloader_Version = 0x31,
} BOOTLOADER_VERSION;

enum {
	MCU_FW_INSIDE_BURN_RESULT__NONE = 0,
	MCU_FW_INSIDE_BURN_RESULT__SUCCESS,
	MCU_FW_INSIDE_BURN_RESULT__I2C_ERROR,
	MCU_FW_INSIDE_BURN_RESULT__FW_CRC_ERROR,
	MCU_FW_INSIDE_BURN_RESULT__UNKNOW_ERROR,
};

typedef enum {
	HCHG_WAIT_MASTER_CHG,
	HCHG_MASTER_CHG,
	HCHG_DUAL_MODE_REQUEST,
	HCHG_DUAL_MODE,
	HCHG_STANDALONE_MODE,
	HCHG_EXIT,
} HChg_STEP;


/*static var statement*/
static const char * const adapter_vendor[] = {
	"VIVO",
	"DIALOG",
	"NXP",
	"RICHTEK"
};

static const char * const mcu_fw_niside_burn_result_str[] = {
	"NONE",
	"1",/*Success*/
	"I2C_error",
	"FW_CrC_error",
	"unknow_error",
};

#define TEST_CYCLE_CHARGE_DISCHARGE	1
#define TEST_BY_FXI_FOR_BURN
#ifdef TEST_BY_FXI_FOR_BURN
static u32 const crc32_table[256] = {
	0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b,
	0x1a864db2, 0x1e475005, 0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
	0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd, 0x4c11db70, 0x48d0c6c7,
	0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
	0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3,
	0x709f7b7a, 0x745e66cd, 0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
	0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5, 0xbe2b5b58, 0xbaea46ef,
	0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
	0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb,
	0xceb42022, 0xca753d95, 0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
	0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d, 0x34867077, 0x30476dc0,
	0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
	0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4,
	0x0808d07d, 0x0cc9cdca, 0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
	0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02, 0x5e9f46bf, 0x5a5e5b08,
	0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
	0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc,
	0xb6238b25, 0xb2e29692, 0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
	0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a, 0xe0b41de7, 0xe4750050,
	0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
	0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34,
	0xdc3abded, 0xd8fba05a, 0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
	0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb, 0x4f040d56, 0x4bc510e1,
	0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
	0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5,
	0x3f9b762c, 0x3b5a6b9b, 0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
	0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623, 0xf12f560e, 0xf5ee4bb9,
	0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
	0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd,
	0xcda1f604, 0xc960ebb3, 0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
	0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b, 0x9b3660c6, 0x9ff77d71,
	0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
	0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2,
	0x470cdd2b, 0x43cdc09c, 0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
	0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24, 0x119b4be9, 0x155a565e,
	0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
	0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a,
	0x2d15ebe3, 0x29d4f654, 0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
	0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c, 0xe3a1cbc1, 0xe760d676,
	0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
	0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662,
	0x933eb0bb, 0x97ffad0c, 0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
	0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};
#endif

enum {
	USER 				= BIT(0),
	THERMAL				= BIT(1),
	CURRENT				= BIT(2),
	SOC				= BIT(3),
};

/* PD1824A: Dual BQ25970 FFC/No-FFC project: [git: PrivateGroups/mcu_sw |  branch: PD1824_DUAL_CHG_FFC | ]
PD1824BA: Single BQ25970 project: [git: PrivateGroups/mcu_sw |  branch: pd1824_single_charge | ]
*/

enum {
	BQ25970_SINGLE_DCHG = 1,/* slave addr: 0x66 (7bits I2C), Default ac ovp is 6.5v */
	NXP_PCA9486_DCHG,/*pcb9468 iic 7-bit Address pin low:0x57,hz:0x5E*/
	BQ25970_DUAL_DCHG,
	LN8000_DCHG,
};
static const char * const DCHG_IC_VENDOR[] = {
	"unknow",
	"TI:BQ25970",
	"NXP:PCA9486",
	"TI:DUAL BQ25970",
	"LION:LN8000",
};

struct sbu_r_adc {
	int vmin;
	int vmax;
	int id;
};

static const char * const sbu_cable_id_type_strings[] = {
	"unknow_cable",
	"default_cable (3A)",
	"4A cable(120k)",
	"5A cable(68k)",
	"6A cable(47k)",
	"8A cable(30k)",
	"cableID_error"
};

/*68K-4A, 47K-6A, 30K-8A.*/
enum SBU_CABLE_ID_TYPE {
	SBU_CABLE_ID_UNKNOW = 0,
	SBU_CABLE_ID_DEFAULT,//SBU float cable(default typec Cable 3A)
	SBU_CABLE_ID_4A,//sbu_pull_down=120k
	SBU_CABLE_ID_5A,//sbu_pull_down=68k
	SBU_CABLE_ID_6A,//sbu_pull_down=47k
	SBU_CABLE_ID_8A,//sbu_pull_down=30k
	SBU_CABLE_ID_ERROR,
};

/* 3.5mm no usb switch */
static const struct sbu_r_adc sub_normal_radc_no_switch[] = {
	{
		.vmin = 1450,
		.vmax = 1890,
		.id = SBU_CABLE_ID_DEFAULT,
	},
	{
		.vmin = 984,
		.vmax = 1099,
		.id = SBU_CABLE_ID_4A,
	},
	{
		.vmin = 784,
		.vmax = 877,
		.id = SBU_CABLE_ID_5A,
	},
	{
		.vmin = 668,
		.vmax = 747,
		.id = SBU_CABLE_ID_6A,
	},
	{
		.vmin = 614,
		.vmax = 550,
		.id = SBU_CABLE_ID_8A,
	},
};

static const struct sbu_r_adc sub_error_radc_no_switch[] = {
	{
		.vmin = 1401,
		.vmax = 1890,
		.id = SBU_CABLE_ID_ERROR,
	},
	{
		.vmin = 1100,
		.vmax = 1400,
		.id = SBU_CABLE_ID_DEFAULT,
	},
	{
		.vmin = 878,
		.vmax = 1099,
		.id = SBU_CABLE_ID_4A,
	},
	{
		.vmin = 748,
		.vmax = 877,
		.id = SBU_CABLE_ID_5A,
	},
	{
		.vmin = 551,
		.vmax = 747,
		.id = SBU_CABLE_ID_6A,
	},
	{
		.vmin = 0,
		.vmax = 550,
		.id = SBU_CABLE_ID_ERROR,
	},
};

/* no 3.3mm and usb switch */
static const struct sbu_r_adc sub_normal_radc_rely_switch[] = {
	{
		.vmin = 1450,
		.vmax = 1890,
		.id = SBU_CABLE_ID_DEFAULT,
	},
	{
		.vmin = 874,
		.vmax = 1094,
		.id = SBU_CABLE_ID_4A,
	},
	{
		.vmin = 650,
		.vmax = 816,
		.id = SBU_CABLE_ID_5A,
	},
	{
		.vmin = 516,
		.vmax = 648,
		.id = SBU_CABLE_ID_6A,
	},
	{
		.vmin = 377,
		.vmax = 471,
		.id = SBU_CABLE_ID_8A,
	},
};

static const struct sbu_r_adc sub_error_radc_rely_switch[] = {
	{
		.vmin = 1401,
		.vmax = 1890,
		.id = SBU_CABLE_ID_ERROR,
	},
	{
		.vmin = 1095,
		.vmax = 1400,
		.id = SBU_CABLE_ID_DEFAULT,
	},
	{
		.vmin = 817,
		.vmax = 1094,
		.id = SBU_CABLE_ID_4A,
	},
	{
		.vmin = 649,
		.vmax = 816,
		.id = SBU_CABLE_ID_5A,
	},
	{
		.vmin = 378,
		.vmax = 648,
		.id = SBU_CABLE_ID_6A,
	},
	{
		.vmin = 0,
		.vmax = 377,
		.id = SBU_CABLE_ID_ERROR,
	},
};

static const char * const sbu_switcher_test_case_str[] = {
	"SBU_OFF       ",
	"SBU_DIRECT_ON ",
	"SBU_FLIP_ON   ",
	"SBU_EX_DET    ",
	"SBU_TEST_CASE_MAX",
};

/* SBU switch IC ON/OFF */
enum sbu_switcher_test_case {
	SBU_OFF = 0,	//switch IC OFF
	SBU_DIRECT_ON,	//switch IC direct on
	SBU_FLIP_ON,	//switch IC flip on
	SBU_EX_DET,		//use external IC to detect SBU adc

	SBU_TEST_CASE_MAX,
};

struct sbu_adc_test {
	int	sbu1;
	int	sbu2;
};

enum {
	STRUCT_INT_ARRAY = 0,
	STRUCT_CABLE_R_LIMIT_TABLE,
	STRUCT_FFC_CC_TO_CV_IBAT_THR_TABLE,
	STRUCT_FFC_DCHG_CV_MV_TABLE,
};

struct cable_r_limit_table {
	int rmin;
	int rmax;
	int dchg_ibus_max;
	int cable_id_type;
	int limit_count;
};

static int default_cable_r_limit_table_rc[] = {7, 5};

static struct cable_r_limit_table default_cable_r_limit_table[] = {
	{
		.rmin = 0,
		.rmax = 220,
		.dchg_ibus_max = 4000,
		.cable_id_type = 0,
		.limit_count = 0,
	},
	{
		.rmin = 221,
		.rmax = 278,
		.dchg_ibus_max = 3000,
		.cable_id_type = 0,
		.limit_count = 0,
	},
	{
		.rmin = 279,
		.rmax = 382,
		.dchg_ibus_max	= 2250,
		.cable_id_type = 0,
		.limit_count = 0,
	},
	{
		.rmin = 383,
		.rmax = 515,
		.dchg_ibus_max = 1800,
		.cable_id_type = 0,
		.limit_count = 0,
	},
	{
		.rmin = 516,
		.rmax = INT_MAX,
		.dchg_ibus_max = 0,
		.cable_id_type = 0,
		.limit_count = 0,
	},

	/* below for OTHER cable only: */
	{
		.rmin = 0,
		.rmax = 278,
		.dchg_ibus_max = 2700,
		.cable_id_type = 1,/*signed flag*/
		.limit_count = 0,
	},
	{
		.rmin = 279,
		.rmax = 382,
		.dchg_ibus_max = 2250,
		.cable_id_type = 1,
		.limit_count = 0,
	},
};

struct ffc_cc_to_cv_ibat_thr_table {
	int tmin;
	int tmax;
	int ibat;
};

static int default_ffc_cc_to_cv_ibat_thr_rc[] = {1, 3};

static struct ffc_cc_to_cv_ibat_thr_table default_ffc_cc_to_cv_ibat_thr_table[] = {
	{
		.tmin = 150,
		.tmax = 450,
		.ibat = 4000,
	},
};

struct ffc_dchg_cv_mv_table {
	int btb_cv_mv;
	int fg_cv_mv;
};

static int default_ffc_dchg_cv_mv_rc[] = {3, 2};

static struct ffc_dchg_cv_mv_table default_ffc_dchg_cv_mv_table[] = {
	{
		.btb_cv_mv = 4450,
		.fg_cv_mv = 4430,
	},
	{
		.btb_cv_mv = 4450,
		.fg_cv_mv = 4430,
	},
	{
		.btb_cv_mv = 4450,
		.fg_cv_mv = 4430,
	},
};

enum ffc_dchg_step_cv {
	FFC_DCHG_STEP_CV_DEFAULT = 0,
	FFC_DCHG_STEP_CV_SINGLE,
	FFC_DCHG_STEP_CV_DUAL,
	FFC_DCHG_STEP_CV_MAX,
};

struct stm32_iio {
	struct iio_channel *sbu1_adc_chan;
	struct iio_channel *sbu2_adc_chan;
	struct iio_channel *bat_board_temp_chan;
	struct iio_channel *usb_conn_temp_chan;
	struct iio_channel *bat_conn_temp_chan;
	struct iio_channel *master_bat_conn_temp_chan;
};

#ifdef TEST_BY_FXI_FOR_BURN
struct test_data {
	struct stm32l011_mcu *chip;
	bool dir_chg_switch;
	bool burn_from_user;
	u8 type;
	u32 voltage;
	u32 icurrent;
	u32 impedance;
	u8 dchg_switch;
	u32 crc32;
	u32 size;
	int opt_ret;
	u8 buffer[MAX_FW_SIZE];
};
#endif

struct stm32l011_mcu {
	struct i2c_client *client;
	struct device *dev;
	struct stm32_iio iio;
	char *name;

	const char *bms_psy_name;
	bool resume_completed;
	bool irq_waiting;
	struct delayed_work chg_monitor_work;
	struct delayed_work chg_deinit_work;
	struct delayed_work restore_pmi_work;
	struct mutex pm_lock;
	int wake_reasons;

	struct wakeup_source stm32l011_wake_lock;

	/* status tracking */
	int workaround_flags;

	bool apsd_rerun;
	bool usbin_ov;

	/* psy */
	struct power_supply_lite *mcu_psyl;
	bool exist;

	/*direct charger*/
	int chg_version_gpio;
	int usbsel_gpio;
	int usbsel_state;
	int power_gpio;
	int int_gpio;
	int shift_en_gpio;
	int sbu_pwr_gpio;
	int irq;
	struct regulator *power_ldo;

	struct mutex irq_complete;
	struct mutex stm32l011_i2c_lock;
	struct mutex status_lock;
	struct mutex parameters_lock;
	struct mutex dchg_enable_lock;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debug_root;
#endif
	/* stm32 debug class */
	struct class stm32_debug;
	struct test_data *test_data;

	/* adc_tm parameters */
	struct qpnp_vadc_chip *vadc_dev;

	/*parameters for engineer mode*/
#if TEST_CYCLE_CHARGE_DISCHARGE
	bool chg_switch;
	bool burn_success;
	u8 exit_event;
#endif
	int ibus_ma;
	int vbus_mv;
	int vbat_mv;
	int cable_mohm;
	int bat_temp;
	int bat_temp_backup;
	int bat_temp_mv;
	int bat_conn_temp;
	int bat_conn_temp_backup;
	int pcb_conn_temp;
	int usb_conn_temp;
	int usb_conn_temp_backup;
	int adapter_temp;
	int adapter_conn_temp;
	int bq_die_temp;
	int dp_mv;
	int dm_mv;
	bool dp_dm_ovp;
	int req_ibus;
	int ldo_count;
	int ptmValue;
	bool cable_matched;
	bool chg_enable;
	char dchg_state[300];
	bool adjust_stop;
	bool is_ldo;
	u8 handshakeid[4];

	/*monitor changing status*/
	bool init_done;
	bool exit;
	bool pmi_suspend;
	bool handshake_success;
	bool vbat_good;
	bool handshake_failed;
	bool cable_matched_event;
	bool high_vol_mode_event;

	bool usb_present;
	bool check_pwr_reset_after_chg_remove;
	bool pmi_suspended;
	u8 dchg_exit_status;
	int total_dchg_limit_max_ma;

	/* jeita parameters */
	int MCU_hot_decidegc;

	unsigned int MCU_warm_ma;

	struct delayed_work inside_burn_work;
	struct mutex burn_lock;
	struct mutex cmd_lock;
	u8 erase_cmd[6];
	u8 crc_cmd[12];

	struct mcu_fw fw;
	int fw_id_num;
	bool update_done;
	bool burn_handling;
	int inside_burn_result;

	u32 analog_i2c_read_exception;
	u32 analog_i2c_write_exception;
	u32 uart_tx_exception;
	u32 uart_rx_exception;
	u32 ap_i2c_read_exception;
	u32 ap_i2c_write_exception;
	u32 adapter_i2c_read_exception;
	u32 adapter_i2c_write_exception;
	u32 adapter_uart_tx_exception;
	u32 adapter_uart_rx_exception;
	u32 adapter_status;

	/*big data*/
	bool report_big_data;
	u8 irqA;
	u8 irqB;
	u8 irqC;
	u8 irqD;
	u8 reg04;
	u8 reg05;
	int usb_type;
	int bigdata_collect_time_limit;
	unsigned long cable_connect_start;

	u8 adapter_mcu_version;
	u8 adapter_power;
	u8 adapter_vendor;
	u8 dchg_version;
	u8 mcu_current_fw;
	u8 mcu_bl_vendor;
	u8 mcu_bl_version;
	int req_vbus;
	int user_dchg_current;

	/*big data fuel summary*/
	bool chg_update;
	struct power_supply *fuelsummary_psy;

	/*bq25970 int status flag*/
	u8 int_stat_08;
	u8 int_stat_0A;
	u8 int_stat_0B;
	u8 int_stat_0E;
	u8 int_stat_11;
	u8 int_stat_2D;
	u8 dchg_status;
	u8 dchg_enabled;
	u8 dchg_pon_reset;
	struct votable *dchg_disable_votable;
	struct votable *dchg_fcc_votable;

	/*vbus monitor for QC2.0*/
	int vbus_exception;

	unsigned long time_count;

	/*check dchg restart*/
	int low_temp_retries;
	int medial_temp_retries;
	int normal_temp_retries;
	int high_temp_retries;

	bool setting_curr_too_low_for_dchg;

	/*check pca9468 restart*/
	bool reg_exception;
	int reg_exception_count;
	int reset_retries;
	u16 adapter_vbus;
	u16 pca_vbus;

	/*mcu debug fw update*/
	struct delayed_work mcu_debug_fw_update;
	char mcu_debug_fw_name[100];
	int mcu_debug_fw_update_result;
	unsigned char *mcu_debug_fw;
	bool debug_fw_updating;

	bool burn_ctl_work_state;
	struct delayed_work burn_ctl_work;

	u16 low_current_exit_threshold;
	/* add for slave charger */
	bool enable_slave_charger;
	int slave_ibus_ma;
	bool slave_chg_enable;
	u8 slave_int_stat_08;
	u8 slave_int_stat_0A;
	u8 slave_int_stat_0E;
	u8 slave_int_stat_11;
	u8 slave_int_stat_2D;
	u8 dual_chg_step;
	int dual_chg_cur_threshold;
	bool slave_ic_enable_fail;
	bool dual_chg_to_standalone_req;
	int dchg_master_max_current;
	bool in_standalone_mode;
	bool slave_charger_should_disable;
	int both_charger_fault_counter;
	int slave_charger_fault_counter;
	int master_slave_charger_current_miss_match;
	bool set_cur_too_low_for_dual_chg;
	int master_bat_conn_temp;
	int master_bat_conn_temp_backup;
	u8 at_mode_slave_chg_enable_req;
	int factory_mode_state;
	int factory_10W_charge_test_enabe;
	int real_vbat_mv;

	/*cable R limit table function*/
	struct cable_r_limit_table *cable_r_limit_table;
	int *cable_r_limit_table_rc;
	int cable_r_current_limit_max;
	int cable_r_disable_dchg;

	int batt_capacity;
	struct mutex ffc_lock;

	/*SBUx Cablel ID adc*/
	bool sbu_cable_id_detect_enable;
	bool sbu_rely_switch;
	bool sbu_detect_busy;
	int sbu1_adc;
	int sbu2_adc;
	int sbu_cable_id;
	int sbu1_range_index;
	int sbu2_range_index;
	int cable_id_detect_done;//0---unfinished 1---normal range 2---err range
	struct sbu_adc_test sbu_adc[SBU_TEST_CASE_MAX];

	/*add for MCU AP main SM*/
	u8 tx_buff[40];
	u8 rx_buff[60];
	u8 mcu_main_chg_sm;
	u8 main_chg_sm_error_in_ap_side;	//same with hdchg.Instance->R05

	int vivo_flash_charge_status;

	unsigned long dchg_start_time;
	unsigned long dchg_continue_time;	//s

	unsigned long last_cable_disconnect_start_time;
	bool ffc_support;

	/*Vbat > 4.45V, Battery_board will trigger Cout_interrupt to AP*/
	int battery_cout_counter;
	int battery_cout_value;
	struct mutex cout_irq_complete;
	int cout_gpio;
	int cout_irq;

	/*dual IC + ffc chg  threshold*/
	struct ffc_cc_to_cv_ibat_thr_table *ffc_cc_to_cv_ibat_thr;
	struct ffc_dchg_cv_mv_table *ffc_dchg_cv_mv;
	int *ffc_cc_to_cv_ibat_thr_rc;
	int *ffc_dchg_cv_mv_rc;
	int ffc_temperature_range[2];
	int ffc_param_tuning_enable;

	/*NTC  enable state*/
	int bat_board_temp_enable;
	int usb_conn_temp_enable;
	int bat_conn_temp_enable;
	int master_bat_conn_temp_enable;

	/*bat_baord, usb_conn, pcb_conn(master_bat_conn), bat_conn*/
	int *dchg_exit_ntc_threshold;

	/*enable bat board temp debug layer node*/
	bool bat_temp_exist;

	/*dchg supported type*/
	int dchg_supported_type;

	/* power gpio pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *power_gpio_default;
	struct pinctrl_state *power_gpio_high;
	struct pinctrl_state *power_gpio_low;

	/*adapter power derate function*/
	int adapter_power_derate_enable;
	bool adapter_powerderate_ready;
	u32 adapter_powerderate;

	bool ex_fg_ffc_support;
	unsigned int ex_fg_i2c_error_counter;
	bool suspend;
	bool user_vote_lose;
	bool disable_vote_miss;
	bool chgic_rt9759;
#if OLD_CAM_NOTIFY
	bool cam_running;
	bool cam_exit_restore;
	int cam_exit_second;
	struct delayed_work cam_exit_work;
#endif
	int delta_ibus;

	bool ln8000_supported;
	u8 int_stat_ln8000_01;
	u8 int_stat_ln8000_03;
	u8 int_stat_ln8000_04;
	u8 int_stat_ln8000_05;
	u8 int_stat_ln8000_06;
	u8 int_stat_ln8000_08;
};

struct stm_irq_info {
	const char *name;
	int (*smb_irq)(struct stm32l011_mcu *chip, u8 rt_stat);
	int high;
	int low;
};

struct irq_handler_info {
	u8 stat_reg;
	u8 val;
	u8 prev_val;
	struct stm_irq_info irq_info[8];
};

struct mcu_status {
	bool mcu_hot;
	bool mcu_warm;
	bool mcu_cool;
	bool mcu_cold;
	bool mcu_present;
};

enum UPDATE_BQFS_UPDATE_FLAG {
	UPDATE_BQFS_NO_NEED_UPDATE = BIT(0),
	UPDATE_BQFS_UPDATE_ON_GOING = BIT(1),
	UPDATE_BQFS_UPDATE_DONE = BIT(2),
};

static const char * const mcu_debug_fw_update_result_str[] = {
	"waiting",
	"error",
	"success",
	"no such file",
};

enum {
	mcu_debug_fw_update_result_waiting = 0,
	mcu_debug_fw_update_result_error,
	mcu_debug_fw_update_result_success,
	mcu_debug_fw_update_result_no_such_file,
};

enum {
	MCU_HOT = 0,
	MCU_WARM,
	MCU_NORMAL,
	MCU_COOL,
	MCU_COLD,
	MCU_MISSING,
	MCU_STATUS_MAX,
};

enum {
	MCU_TEMP_ADAPTER		= 0x00U,
	MCU_TEMP_ADAPTER_CONN		= 0x01U,
	MCU_TEMP_USB_CONN		= 0x02U,
	MCU_TEMP_PCB_BTB		= 0x03U,
	MCU_TEMP_BAT_BTB		= 0x04U,
	MCU_TEMP_BAT			= 0x05U,
};

enum wake_reason {
	PM_BURN				= BIT(0),
	PM_DEINIT			= BIT(1),
};

enum DCHG_SUPPORTED_TYPE {
	/*BQ25970_SINGLE_DCHG*/
	DCHG_SINGLE_TYPE_22P5W = 0,
	DCHG_SINGLE_TYPE_33W,
	DCHG_SINGLE_TYPE_44W,
	/*BQ25970_DUAL_DCHG*/
	DCHG_DUAL_TYPE_44W,
	DCHG_DUAL_TYPE_MAX,
};

enum print_reason {
	PR_INFO				= BIT(0),
	PR_ERROR			= BIT(1),
	PR_WARN 			= BIT(2),
	PR_DBG				= BIT(3),
};

enum pinctrl_config {
	PINCFG_LOW = 0,
	PINCFG_HIGH,
	PINCFG_DEFAULT,
};

/*bat_baord, usb_conn, pcb_conn(master_bat_conn), bat_conn*/
enum dchg_exit_ntc_threshold {
	DENT_BAT_BOARD_TEMP = 0,
	DENT_USB_CONN_TEMP,
	DENT_MASTER_BAT_CONN_TEMP,
	DENT_BAT_CONN_TEMP,
	DENT_BQ_DIE_TEMP,
};
static int default_dchg_exit_ntc_threshold[] = {650, 650, 700, 700, 1200};

enum ntc_enable_state {
	NTC_NONSUPPORT = 0,
	NTC_FROM_BQ25970,
	NTC_FROM_PMI,
	NTC_FROM_EX_FG,
};

/*ntc adc table*/
struct adc_map_temp {
	int temp;
	unsigned long voltage;
};

/*BAT_BTB_ADC/PCB_BTB_ADC*/
static const struct adc_map_temp adc_map_temp_table1[] = {
	{-300, 9999999},
	{-290, 3224672},
	{-280, 3219720},
	{-270, 3214498},
	{-260, 3208925},
	{-250, 3203123},
	{-240, 3196944},
	{-230, 3190466},
	{-220, 3183646},
	{-210, 3176414},
	{-200, 3168782},
	{-190, 3160897},
	{-180, 3152426},
	{-170, 3143649},
	{-160, 3134401},
	{-150, 3124706},
	{-140, 3114540},
	{-130, 3103894},
	{-120, 3092730},
	{-110, 3081087},
	{-100, 3068887},
	{-90, 3056132},
	{-80, 3042871},
	{-70, 3028988},
	{-60, 3014522},
	{-50, 2999477},
	{-40, 2983792},
	{-30, 2967524},
	{-20, 2950597},
	{-10, 2932986},
	{00, 2914754},
	{10, 2895883},
	{20, 2876345},
	{30, 2856096},
	{40, 2835211},
	{50, 2813488},
	{60, 2791142},
	{70, 2768289},
	{80, 2744484},
	{90, 2720187},
	{100, 2695086},
	{110, 2669256},
	{120, 2642797},
	{130, 2615836},
	{140, 2588206},
	{150, 2559666},
	{160, 2530655},
	{170, 2501339},
	{180, 2471032},
	{190, 2440244},
	{200, 2408621},
	{210, 2376786},
	{220, 2344951},
	{230, 2312102},
	{240, 2278261},
	{250, 2244898},
	{260, 2210815},
	{270, 2176413},
	{280, 2141673},
	{290, 2106648},
	{300, 2071291},
	{310, 2035838},
	{320, 2000134},
	{330, 1964313},
	{340, 1928283},
	{350, 1892304},
	{360, 1856135},
	{370, 1820179},
	{380, 1784019},
	{390, 1748069},
	{400, 1712162},
	{410, 1676426},
	{420, 1640999},
	{430, 1605474},
	{440, 1570325},
	{450, 1535495},
	{460, 1500905},
	{470, 1466667},
	{480, 1432675},
	{490, 1399265},
	{500, 1365844},
	{510, 1333223},
	{520, 1300773},
	{530, 1268832},
	{540, 1237500},
	{550, 1206600},
	{560, 1176215},
	{570, 1146132},
	{580, 1116723},
	{590, 1087762},
	{600, 1059318},
	{610, 1031461},
	{620, 1004263},
	{630, 977448},
	{640, 951423},
	{650, 925536},
	{660, 900557},
	{670, 876184},
	{680, 852083},
	{690, 828681},
	{700, 805629},
	{710, 783368},
	{720, 761538},
	{730, 740594},
	{740, 719730},
	{750, 699396},
	{760, 679625},
	{770, 660449},
	{780, 641902},
	{790, 624017},
	{800, 606356},
	{810, 588936},
	{820, 572200},
	{830, 555936},
	{840, 540115},
	{850, 524757},
	{860, 509779},
	{870, 495248},
	{880, 481128},
	{890, 467434},
	{900, 454128},
	{910, 441225},
	{920, 428629},
	{930, 416457},
	{940, 404615},
	{950, 393165},
	{960, 382009},
	{970, 371209},
	{980, 360719},
	{990, 350546},
	{1000, 340641},
	{1010, 331068},
	{1020, 321775},
	{1030, 312769},
	{1040, 303999},
	{1050, 295525},
	{1060, 287297},
	{1070, 279317},
	{1080, 271590},
	{1090, 264062},
	{1100, 256783},
	{1110, 249721},
	{1120, 242873},
	{1130, 236230},
	{1140, 229783},
	{1150, 223535},
	{1160, 217475},
	{1170, 211595},
	{1180, 205896},
	{1190, 200368},
	{1200, 195001},
	{1210, 189802},
	{1220, 184748},
	{1230, 179853},
	{1240, 175101},
	{1250, 170492},
	{1260, 1},
	{-333, 0},
};

/*BQ25970: USB_CON_ADC/BAT_Therm_ADC*/
static const struct adc_map_temp adc_map_temp_table2[] = {
	{-410, 9999999},
	{-400, 49420},
	{-390, 49380},
	{-380, 49330},
	{-370, 49290},
	{-360, 49240},
	{-350, 49180},
	{-340, 49120},
	{-330, 49070},
	{-320, 49000},
	{-310, 48930},
	{-300, 48860},
	{-290, 48790},
	{-280, 48710},
	{-270, 48620},
	{-260, 48530},
	{-250, 48440},
	{-240, 48340},
	{-230, 48240},
	{-220, 48130},
	{-210, 48010},
	{-200, 47890},
	{-190, 47760},
	{-180, 47630},
	{-170, 47490},
	{-160, 47340},
	{-150, 47180},
	{-140, 47020},
	{-130, 46850},
	{-120, 46670},
	{-110, 46490},
	{-100, 46290},
	{-90, 46090},
	{-80, 45880},
	{-70, 45650},
	{-60, 45420},
	{-50, 45180},
	{-40, 44930},
	{-30, 44680},
	{-20, 44410},
	{-10, 44130},
	{0, 43840},
	{10, 43540},
	{20, 43230},
	{30, 42910},
	{40, 42570},
	{50, 42230},
	{60, 41880},
	{70, 41520},
	{80, 41140},
	{90, 40760},
	{100, 40360},
	{110, 39960},
	{120, 39540},
	{130, 39120},
	{140, 38680},
	{150, 38240},
	{160, 37780},
	{170, 37320},
	{180, 36850},
	{190, 36370},
	{200, 35880},
	{210, 35380},
	{220, 34890},
	{230, 34380},
	{240, 33850},
	{250, 33330},
	{260, 32810},
	{270, 32270},
	{280, 31740},
	{290, 31200},
	{300, 30650},
	{310, 30110},
	{320, 29560},
	{330, 29010},
	{340, 28460},
	{350, 27910},
	{360, 27360},
	{370, 26810},
	{380, 26260},
	{390, 25710},
	{400, 25170},
	{410, 24630},
	{420, 24090},
	{430, 23550},
	{440, 23020},
	{450, 22500},
	{460, 21980},
	{470, 21460},
	{480, 20950},
	{490, 20450},
	{500, 19950},
	{510, 19460},
	{520, 18970},
	{530, 18500},
	{540, 18030},
	{550, 17570},
	{560, 17120},
	{570, 16670},
	{580, 16230},
	{590, 15800},
	{600, 15380},
	{610, 14970},
	{620, 14570},
	{630, 14170},
	{640, 13790},
	{650, 13410},
	{660, 13040},
	{670, 12680},
	{680, 12330},
	{690, 11980},
	{700, 11640},
	{710, 11320},
	{720, 11000},
	{730, 10690},
	{740, 10390},
	{750, 10090},
	{760, 9800},
	{770, 9520},
	{780, 9250},
	{790, 8990},
	{800, 8730},
	{810, 8480},
	{820, 8240},
	{830, 8000},
	{840, 7770},
	{850, 7550},
	{860, 7330},
	{870, 7120},
	{880, 6910},
	{890, 6710},
	{900, 6520},
	{910, 1},
	{-333, 0},
};

/*PCA9486: USB_CON_ADC/BAT_Therm_ADC*/
static const struct adc_map_temp adc_map_temp_table2_2[] = {
	{-400, 1630819},
	{-390, 1629437},
	{-380, 1627971},
	{-370, 1626409},
	{-360, 1624756},
	{-350, 1622996},
	{-340, 1621124},
	{-330, 1619148},
	{-320, 1617053},
	{-310, 1614834},
	{-300, 1612483},
	{-290, 1609991},
	{-280, 1607365},
	{-270, 1604596},
	{-260, 1601642},
	{-250, 1598567},
	{-240, 1595292},
	{-230, 1591861},
	{-220, 1588249},
	{-210, 1584420},
	{-200, 1580380},
	{-190, 1576208},
	{-180, 1571727},
	{-170, 1567086},
	{-160, 1562197},
	{-150, 1557074},
	{-140, 1551704},
	{-130, 1546083},
	{-120, 1540191},
	{-110, 1534048},
	{-100, 1527615},
	{-90, 1520893},
	{-80, 1513907},
	{-70, 1506597},
	{-60, 1498985},
	{-50, 1491072},
	{-40, 1482827},
	{-30, 1474282},
	{-20, 1465395},
	{-10, 1456157},
	{0, 1446598},
	{10, 1436712},
	{20, 1426484},
	{30, 1415892},
	{40, 1404975},
	{50, 1393630},
	{60, 1381969},
	{70, 1370055},
	{80, 1357655},
	{90, 1345010},
	{100, 1331959},
	{110, 1318542},
	{120, 1304812},
	{130, 1290836},
	{140, 1276528},
	{150, 1261765},
	{160, 1246775},
	{170, 1231644},
	{180, 1216018},
	{190, 1200164},
	{200, 1183899},
	{210, 1167544},
	{220, 1151210},
	{230, 1134375},
	{240, 1117055},
	{250, 1100000},
	{260, 1082600},
	{270, 1065060},
	{280, 1047371},
	{290, 1029560},
	{300, 1011604},
	{310, 993624},
	{320, 975540},
	{330, 957422},
	{340, 939223},
	{350, 921073},
	{360, 902853},
	{370, 884765},
	{380, 866599},
	{390, 848563},
	{400, 830573},
	{410, 812692},
	{420, 794990},
	{430, 777262},
	{440, 759745},
	{450, 742410},
	{460, 725216},
	{470, 708220},
	{480, 691367},
	{490, 674823},
	{500, 658295},
	{510, 642182},
	{520, 626173},
	{530, 610434},
	{540, 595013},
	{550, 579823},
	{560, 564903},
	{570, 550147},
	{580, 535738},
	{590, 521564},
	{600, 507658},
	{610, 494053},
	{620, 480783},
	{630, 467713},
	{640, 455041},
	{650, 442448},
	{660, 430308},
	{670, 418473},
	{680, 406782},
	{690, 395438},
	{700, 384275},
	{710, 373504},
	{720, 362949},
	{730, 352831},
	{740, 342759},
	{750, 332951},
	{760, 323421},
	{770, 314184},
	{780, 305257},
	{790, 296654},
	{800, 288165},
	{810, 279796},
	{820, 271762},
	{830, 263958},
	{840, 256372},
	{850, 249012},
	{860, 241838},
	{870, 234882},
	{880, 228126},
	{890, 221577},
	{900, 215218},
	{910,	1},
	{-333,	0},
};

/*LCM /BQ adc*/
static const struct adc_map_temp adc_map_temp_table3[] = {
	{-300, 9999999},
	{-29, 1430433},
	{-28, 1428647},
	{-27, 1426761},
	{-26, 1424748},
	{-25, 1422650},
	{-24, 1420414},
	{-23, 1418069},
	{-22, 1415598},
	{-21, 1412976},
	{-20, 1410206},
	{-19, 1407341},
	{-18, 1404261},
	{-17, 1401067},
	{-16, 1397697},
	{-15, 1394161},
	{-14, 1390448},
	{-13, 1386556},
	{-12, 1382469},
	{-11, 1378201},
	{-10, 1373722},
	{-9, 1369033},
	{-8, 1364151},
	{-7, 1359031},
	{-6, 1353688},
	{-5, 1348121},
	{-4, 1342307},
	{-3, 1336265},
	{-2, 1329966},
	{-1, 1323400},
	{0, 1316587},
	{1, 1309521},
	{2, 1302188},
	{3, 1294570},
	{4, 1286694},
	{5, 1278481},
	{6, 1270010},
	{7, 1261324},
	{8, 1252251},
	{9, 1242964},
	{10, 1233341},
	{11, 1223408},
	{12, 1213202},
	{13, 1202769},
	{14, 1192041},
	{15, 1180924},
	{16, 1169583},
	{17, 1158082},
	{18, 1146150},
	{19, 1133984},
	{20, 1121440},
	{21, 1108764},
	{22, 1096038},
	{23, 1082855},
	{24, 1069218},
	{25, 1055718},
	{26, 1041870},
	{27, 1027833},
	{28, 1013598},
	{29, 999184},
	{30, 984570},
	{31, 969852},
	{32, 954963},
	{33, 939959},
	{34, 924800},
	{35, 909593},
	{36, 894237},
	{37, 878903},
	{38, 863410},
	{39, 847938},
	{40, 832413},
	{41, 816892},
	{42, 801435},
	{43, 785866},
	{44, 770391},
	{45, 754988},
	{46, 739623},
	{47, 724346},
	{48, 709113},
	{49, 694075},
	{50, 678966},
	{51, 664157},
	{52, 649362},
	{53, 634738},
	{54, 620333},
	{55, 606069},
	{56, 591987},
	{57, 577989},
	{58, 564250},
	{59, 550669},
	{60, 537280},
	{61, 524117},
	{62, 511220},
	{63, 498458},
	{64, 486028},
	{65, 473622},
	{66, 461610},
	{67, 449850},
	{68, 438184},
	{69, 426820},
	{70, 415591},
	{71, 404715},
	{72, 394017},
	{73, 383723},
	{74, 373440},
	{75, 363390},
	{76, 353592},
	{77, 344063},
	{78, 334823},
	{79, 325891},
	{80, 317050},
	{81, 308307},
	{82, 299889},
	{83, 291690},
	{84, 283697},
	{85, 275920},
	{86, 268321},
	{87, 260933},
	{88, 253740},
	{89, 246750},
	{90, 239947},
	{91, 233336},
	{92, 226872},
	{93, 220616},
	{94, 214518},
	{95, 208613},
	{96, 202850},
	{97, 197263},
	{98, 191828},
	{99, 186550},
	{100, 181404},
	{101, 176423},
	{102, 171582},
	{103, 166884},
	{104, 162304},
	{105, 157873},
	{106, 153565},
	{107, 149383},
	{108, 145329},
	{109, 141375},
	{110, 137548},
	{111, 133831},
	{112, 130224},
	{113, 126721},
	{114, 123318},
	{115, 120018},
	{116, 116814},
	{117, 113702},
	{118, 110684},
	{119, 107754},
	{120, 104907},
	{-333, 0},
};

/*adapter/adapter_conn adc*/
static const struct adc_map_temp adc_map_temp_table4[] = {
	{-333, 0},
	{-300, 44138},
	{-290, 47013},
	{-280, 50053},
	{-270, 53269},
	{-260, 56669},
	{-250, 60262},
	{-240, 64058},
	{-230, 68065},
	{-220, 72294},
	{-210, 76753},
	{-200, 81452},
	{-190, 86400},
	{-180, 91607},
	{-170, 97083},
	{-160, 102836},
	{-150, 108877},
	{-140, 115215},
	{-130, 121861},
	{-120, 128823},
	{-110, 136113},
	{-100, 143741},
	{-90, 151716},
	{-80, 160050},
	{-70, 168754},
	{-60, 177838},
	{-50, 187313},
	{-40, 197191},
	{-30, 207483},
	{-20, 218201},
	{-10, 229355},
	{0, 240957},
	{10, 253019},
	{20, 265552},
	{30, 278566},
	{40, 292073},
	{50, 306082},
	{60, 320604},
	{70, 335648},
	{80, 351223},
	{90, 367337},
	{100, 383998},
	{110, 401212},
	{120, 418985},
	{130, 437323},
	{140, 456228},
	{150, 475704},
	{160, 495752},
	{170, 516372},
	{180, 537565},
	{190, 559327},
	{200, 581655},
	{210, 604544},
	{220, 627989},
	{230, 651982},
	{240, 676514},
	{250, 701575},
	{260, 727153},
	{270, 753236},
	{280, 779810},
	{290, 806859},
	{300, 834368},
	{310, 862318},
	{320, 890692},
	{330, 919470},
	{340, 948632},
	{350, 978156},
	{360, 1008021},
	{370, 1038203},
	{380, 1068681},
	{390, 1099430},
	{400, 1130425},
	{410, 1161643},
	{420, 1193059},
	{430, 1224648},
	{440, 1256384},
	{450, 1288242},
	{460, 1320198},
	{470, 1352226},
	{480, 1384302},
	{490, 1416400},
	{500, 1448496},
	{510, 1480566},
	{520, 1512587},
	{530, 1544535},
	{540, 1576387},
	{550, 1608121},
	{560, 1639715},
	{570, 1671148},
	{580, 1702399},
	{590, 1733450},
	{600, 1764280},
	{610, 1794872},
	{620, 1825208},
	{630, 1855270},
	{640, 1885043},
	{650, 1914512},
	{660, 1943663},
	{670, 1972481},
	{680, 2000954},
	{690, 2029070},
	{700, 2056818},
	{710, 2084189},
	{720, 2111172},
	{730, 2137759},
	{740, 2163943},
	{750, 2189716},
	{760, 2215073},
	{770, 2240009},
	{780, 2264518},
	{790, 2288598},
	{800, 2312245},
	{810, 2335458},
	{820, 2358234},
	{830, 2380573},
	{840, 2402474},
	{850, 2423939},
	{860, 2444967},
	{870, 2465561},
	{880, 2485722},
	{890, 2505454},
	{900, 2524759},
	{910, 2543641},
	{920, 2562103},
	{930, 2580150},
	{940, 2597787},
	{950, 2615019},
	{960, 2631850},
	{970, 2648287},
	{980, 2664335},
	{990, 2680001},
	{1000, 2695290},
	{1010, 2710209},
	{1020, 2724764},
	{1030, 2738963},
	{1040, 2752811},
	{1050, 2766315},
	{1060, 2779483},
	{1070, 2792322},
	{1080, 2804838},
	{1090, 2817038},
	{1100, 2828929},
	{1110, 2840518},
	{1120, 2851813},
	{1130, 2862819},
	{1140, 2873544},
	{1150, 2883994},
	{1160, 2894176},
	{1170, 2904096},
	{1180, 2913762},
	{1190, 2923178},
	{1200, 2932353},
	{1210, 2941290},
	{1220, 2949998},
	{1230, 2958481},
	{1240, 2966745},
	{1250, 2974796},
	{1260, 9999999},
};
/* samsung platform ntc table :start*/

/* td1902_exynos9820 platform ntc table :start*/
/*BATT_THERM_ADC/AP_BOARD_THERM_ADC*/
static const struct adc_map_temp adc_map_temp_table5[] = {
	{-410, 9999},
	{-400, 1759},
	{-390, 1756},
	{-380, 1753},
	{-370, 1749},
	{-360, 1746},
	{-350, 1742},
	{-340, 1738},
	{-330, 1734},
	{-320, 1730},
	{-310, 1725},
	{-300, 1720},
	{-290, 1715},
	{-280, 1709},
	{-270, 1704},
	{-260, 1697},
	{-250, 1691},
	{-240, 1684},
	{-230, 1677},
	{-220, 1670},
	{-210, 1662},
	{-200, 1654},
	{-190, 1646},
	{-180, 1637},
	{-170, 1628},
	{-160, 1618},
	{-150, 1608},
	{-140, 1598},
	{-130, 1587},
	{-120, 1575},
	{-110, 1564},
	{-100, 1551},
	{-90, 1539},
	{-80, 1526},
	{-70, 1512},
	{-60, 1498},
	{-50, 1484},
	{-40, 1469},
	{-30, 1454},
	{-20, 1438},
	{-10, 1422},
	{0, 1405},
	{10, 1388},
	{20, 1371},
	{30, 1353},
	{40, 1335},
	{50, 1316},
	{60, 1297},
	{70, 1278},
	{80, 1258},
	{90, 1238},
	{100, 1218},
	{110, 1198},
	{120, 1177},
	{130, 1156},
	{140, 1136},
	{150, 1114},
	{160, 1093},
	{170, 1072},
	{180, 1050},
	{190, 1029},
	{200, 1007},
	{210, 986},
	{220, 964},
	{230, 943},
	{240, 921},
	{250, 900},
	{260, 879},
	{270, 858},
	{280, 837},
	{290, 816},
	{300, 796},
	{310, 775},
	{320, 755},
	{330, 736},
	{340, 716},
	{350, 697},
	{360, 678},
	{370, 659},
	{380, 641},
	{390, 623},
	{400, 605},
	{410, 588},
	{420, 571},
	{430, 555},
	{440, 538},
	{450, 522},
	{460, 507},
	{470, 492},
	{480, 477},
	{490, 463},
	{500, 449},
	{510, 435},
	{520, 422},
	{530, 409},
	{540, 396},
	{550, 384},
	{560, 372},
	{570, 360},
	{580, 349},
	{590, 338},
	{600, 327},
	{610, 317},
	{620, 307},
	{630, 297},
	{640, 288},
	{650, 279},
	{660, 270},
	{670, 261},
	{680, 253},
	{690, 245},
	{700, 237},
	{710, 230},
	{720, 222},
	{730, 215},
	{740, 209},
	{750, 202},
	{760, 196},
	{770, 189},
	{780, 183},
	{790, 178},
	{800, 172},
	{810, 167},
	{820, 162},
	{830, 156},
	{840, 152},
	{850, 147},
	{860, 142},
	{870, 138},
	{880, 134},
	{890, 130},
	{900, 126},
	{910, 122},
	{920, 118},
	{930, 114},
	{940, 111},
	{950, 108},
	{960, 104},
	{970, 101},
	{980, 98},
	{990, 95},
	{1000, 92},
	{1010, 90},
	{1020, 87},
	{1030, 84},
	{1040, 82},
	{1050, 80},
	{1060, 77},
	{1070, 75},
	{1080, 73},
	{1090, 71},
	{1100, 69},
	{1110, 67},
	{1120, 65},
	{1130, 63},
	{1140, 61},
	{1150, 59},
	{1160, 58},
	{1170, 56},
	{1180, 55},
	{1190, 53},
	{1200, 52},
	{1210, 50},
	{1220, 49},
	{1230, 47},
	{1240, 46},
	{1250, 45},
	{-333, 0},
};


#endif/* __VIVO_STM32L011X4_H */
