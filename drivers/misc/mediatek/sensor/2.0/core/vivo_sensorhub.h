#ifndef _VIVO_SENSOR_HUB_H_
#define _VIVO_SENSOR_HUB_H_

#ifndef CONFIG_VIVO_SENSOR_CONTEXTHUB
#include <linux/ioctl.h>
#endif

#define EVT_VIVO_SENSOR_EVENT			0x00000408

#define VIVO_CMD_DATA_SIZE				10
#define VIVO_USER_SPACE_CMD_DATA_SIZE	(VIVO_CMD_DATA_SIZE+2)
#define VIVO_CMD_TIMEOUT				1000

#define VIVO_CMD_TYPE_REQUEST			0
#define VIVO_CMD_TYPE_RESPOSE			1


typedef enum{
	/* ACC */
	SENSOR_COMMAND_ACC_SELF_TEST                    = 0x100,
	SENSOR_COMMAND_ACC_ENABLE                       = 0x101,
	SENSOR_COMMAND_ACC_DISABLE                      = 0x102,
	SENSOR_COMMAND_ACC_RAW_DATA                     = 0x103,
	SENSOR_COMMAND_ACC_READ_REG                     = 0x104,
	SENSOR_COMMAND_ACC_WRITE_REG                    = 0x105,
	SENSOR_COMMAND_ACC_SET_CALI_DATA                = 0x106,
	SENSOR_COMMAND_ACC_GET_INFO                     = 0x107,
	SENSOR_COMMAND_ACC_CLEAR_INT                    = 0x108,
	SENSOR_COMMAND_ACC_READ_INT                     = 0x109,

	/* GRYO */
	SENSOR_COMMAND_GYRO_SELF_TEST                   = 0x200,
	SENSOR_COMMAND_GYRO_ENABLE                      = 0x201,
	SENSOR_COMMAND_GYRO_DISABLE                     = 0x202,
	SENSOR_COMMAND_GYRO_RAW_DATA                    = 0x203,
	SENSOR_COMMAND_GYRO_SET_CALI_DATA               = 0x204,
	SENSOR_COMMAND_GYRO_GET_INFO                    = 0x205,
	SENSOR_COMMAND_GYRO_CLEAR_INT                   = 0x206,
	SENSOR_COMMAND_GYRO_READ_INT                    = 0x207,

	/* MAG */
	SENSOR_COMMAND_MAG_SELF_TEST                    = 0x400,
	SENSOR_COMMAND_MAG_ENABLE                       = 0x401,
	SENSOR_COMMAND_MAG_DISABLE                      = 0x402,
	SENSOR_COMMAND_MAG_RAW_DATA                     = 0x403,
	/* LIGHT */
	SENSOR_COMMAND_LIGHT_ENABLE                     = 0x500,
	SENSOR_COMMAND_LIGHT_DISABLE                    = 0x501,
	SENSOR_COMMAND_LIGHT_RAW_DATA                   = 0x502,
	SENSOR_COMMAND_LIGHT_INDEX                      = 0x503,

	/* PROXIMITY */
	SENSOR_COMMAND_PROXIMITY_ENABLE                 = 0x800,
	SENSOR_COMMAND_PROXIMITY_DISABLE                = 0x801,
	SENSOR_COMMAND_PROXIMITY_RAWDATA                = 0x802,
	SENSOR_COMMAND_PROXIMITY_INDEX                  = 0x803,
	SENSOR_COMMAND_PROXIMITY_READ_REG_DATA          = 0x804,
	SENSOR_COMMAND_PROXIMITY_WRITE_REG_DATA         = 0x805,
	SENSOR_COMMAND_PROXIMITY_CALI_DATA              = 0x806,
	SENSOR_COMMAND_PROXIMITY_ENG_CALI_DATA          = 0x807,
	SENSOR_COMMAND_PROXIMITY_ENG_CALI_DATA_LOW      = 0x808,
	SENSOR_COMMAND_PROXIMITY_NOTIFY_THRES_LEVEL     = 0x809,
	SENSOR_COMMAND_PROXIMITY_GET_CALI_OFFSET_DATA   = 0x80A,
	SENSOR_COMMAND_PROXIMITY_SET_CALI_OFFSET_DATA   = 0x80B,
	SENSOR_COMMAND_PROXIMITY_POWER_LEVEL            = 0x80C,
	SENSOR_COMMAND_PROXIMITY_SET_PRE_CALI           = 0x80D,
	SENSOR_COMMAND_PROXIMITY_GET_INFO               = 0x80E,

	/*CCT*/
	SENSOR_COMMAND_CCT_ENABLE                       = 0x900,
	SENSOR_COMMAND_CCT_DISABLE                      = 0x901,
	SENSOR_COMMAND_CCT_RAWDATA                      = 0x902,
	SENSOR_COMMAND_CCT_SET_CALI_FACTOR              = 0x903,
} vivo_sensor_cmd;

/*
** ioctrl**
*/
#define VIVO_SENSOR_HUB			0x80
#define VIVO_SENSOR_HUB_SENSOR_CMD 	_IOW(VIVO_SENSOR_HUB, 0x01, int)


typedef struct SCP_SENSOR_HUB_VIVO_CMD_REQ {
	uint8_t sensorType;
	uint8_t action;
	uint8_t reserve[2];
	uint32_t cmd;
	int32_t data[VIVO_CMD_DATA_SIZE];
} SCP_SENSOR_HUB_VIVO_CMD_REQ;

typedef struct SCP_SENSOR_HUB_VIVO_CMD_RSP {
	uint8_t sensorType;
	uint8_t action;
	uint8_t errCode;
	uint8_t reserve[1];
	uint32_t cmd;
	int32_t data[VIVO_CMD_DATA_SIZE];
} SCP_SENSOR_HUB_VIVO_CMD_RSP;

#endif
