/*DUMMY BYTES DATA */
#define DUMMY_HW_REG				1
#define DUMMY_FRAMEBUFFER			1
#define DUMMY_MEMORY				1

/*DIGITAL CHIP INFO */
#define DCHIP_ID_0					0x36
#define DCHIP_ID_1					0x70

#define DCHIP_ID_ADDR				0x0004
#define DCHIP_FW_VER_ADDR			0x000B

#define DCHIP_FW_VER_BYTE			2

/*CHUNKS */
#define READ_CHUNK					(2 * 1024)
#define WRITE_CHUNK					(2 * 1024)
#define MEMORY_CHUNK				(2 * 1024)

/*PROTOCOL INFO */
#define I2C_SAD						0x48

#define I2C_INTERFACE				/*comment if the chip use SPI */
#define ICR_ADDR					0x0024
#define ICR_SPI_VALUE				0x02

/*SYSTEM RESET INFO */
#define SYSTEM_RESET_ADDRESS		0x0028
#define SYSTEM_RESET_VALUE			0x80

/*INTERRUPT INFO */
#define IER_ADDR					0x002C

#define IER_ENABLE					0x41
#define IER_DISABLE					0x00

/*FLASH COMMAND */
#define FLASH_CMD_UNLOCK			0xF7

#define FLASH_CMD_WRITE_64K                     0xF8
#define FLASH_CMD_READ_REGISTER                 0xF9
#define FLASH_CMD_WRITE_REGISTER		0xFA

#define GOLDEN_PANEL_INIT_PAGE       60
#define FLASH_ADDR_CX				0x0000F400
/*FLASH UNLOCK PARAMETER */
#define FLASH_UNLOCK_CODE0			0x74
#define FLASH_UNLOCK_CODE1			0x45

/*FLASH ERASE and DMA PARAMETER */
#define FLASH_ERASE_UNLOCK_CODE0			0x72
#define FLASH_ERASE_UNLOCK_CODE1			0x03
#define FLASH_ERASE_UNLOCK_CODE2			0x02
#define FLASH_ERASE_CODE0                           0x02
#define FLASH_ERASE_CODE1                           0xC0
#define FLASH_DMA_CODE0                           0x05
#define FLASH_DMA_CODE1                           0xC0
#define FLASH_DMA_CONFIG                        0x06
#define FLASH_ERASE_START			0x80
#define FLASH_NUM_PAGE				64//number of pages
#define FLASH_CX_PAGE_START			61
#define FLASH_CX_PAGE_END			62

/*FLASH ADDRESS */
#define ADDR_WARM_BOOT                          0x001E
#define WARM_BOOT_VALUE                          0x38
#define FLASH_ADDR_CODE				0x00000000
#define FLASH_ADDR_CONFIG			0x0000FC00

/*CRC ADDR */
#define ADDR_CRC_BYTE0				0x00
#define ADDR_CRC_BYTE1				0x74
#define CRC_MASK				0x03

/*SIZES FW, CODE, CONFIG, MEMH */
#define FW_HEADER_SIZE					64
#define FW_HEADER_SIGNATURE				0xAA55AA55
#define FW_FTB_VER					0x00000001
#define FW_BYTES_ALLIGN					4
#define FW_BIN_VER_OFFSET					16
#define FW_BIN_CONFIG_VER_OFFSET				20

/*FIFO */
#define FIFO_EVENT_SIZE				8
#define FIFO_DEPTH					64

#define FIFO_CMD_READONE			0x85
#define FIFO_CMD_READALL			0x86
#define FIFO_CMD_LAST				0x87
#define FIFO_CMD_FLUSH				0xA1

/*CONSTANT TOTAL CX */
#define CX1_WEIGHT					4
#define CX2_WEIGHT					1

/*OP CODES FOR MEMORY (based on protocol) */
#define FTS_CMD_HW_REG_R			0xB6
#define FTS_CMD_HW_REG_W			0xB6
#define FTS_CMD_FRAMEBUFFER_R		0xD0
#define FTS_CMD_FRAMEBUFFER_W		0xD0
