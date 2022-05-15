/*
* Copyright (C) 2009 Motorola, Inc.
*/

#ifndef __LEDS_lm3697_H__
#define __LEDS_lm3697_H__

#include <linux/ioctl.h>

#define LM3697_NAME "lm3697_i2c"
#define lm3697_LED_NAME "lm3697-backlight"
#define lm3697_I2C_ADDR 0x36

/*****************************************************************************
* LM3697 registers
*****************************************************************************/
#define LM3697_OUTPUT_CFG_REG 0x10
#define LM3697_CONTROL_A_START_UP_RAMP_REG 0x11
#define LM3697_CONTROL_B_START_UP_RAMP_REG 0x12
#define LM3697_RUN_TIME_RAMP_TIME_REG 0x13
#define LM3697_RUN_TIME_RAMP_CFG_REG 0x14
#define LM3697_CTRL_BR_CFG_REG 0x16
#define LM3697_CTRL_A_FS_CURR_REG 0x17
#define LM3697_CTRL_B_FS_CURR_REG 0x18
#define LM3697_FEEDBACK_ENABLE_REG 0x19
#define LM3697_BOOST_CONTROL_REG 0x1A
#define LM3697_AUTO_FRE_THRESHOLD_REG 0x1B
#define LM3697_PWM_CFG_REG 0x1C

/*
 *  LED current ramping does not start until the MSB is written, 
 *  LSB must always be written before MSB
*/
#define LM3697_CONTROL_A_BRIGHT_LSB_REG 0x20
#define LM3697_CONTROL_A_BRIGHT_MSB_REG 0x21

#define LM3697_CONTROL_B_BRIGHT_LSB_REG 0x22
#define LM3697_CONTROL_B_BRIGHT_MSB_REG 0x23
#define LM3697_CONTROL_BANK_ENABLE_REG 0x24

#define LM3697_HVLED_OPEN_FAULTS_REG 0xB0
#define LM3697_HVLED_SHORT_FAULTS_REG 0xB2
#define LM3697_LED_FAULT_ENABLE_REG 0xB4

/* Brightness config values */
#define LM3697_LINEAR_MAPPING 0x01
#define LM3697_EXP_MAPPING 0x00

#define LM3697_5mA_FS_CURRENT 0x00
#define LM3697_5p8mA_FS_CURRENT 0x01
#define LM3697_6p6mA_FS_CURRENT 0x02
#define LM3697_7p4mA_FS_CURRENT 0x03
#define LM3697_8p2mA_FS_CURRENT 0x04
#define LM3697_9mA_FS_CURRENT 0x05
#define LM3697_9p8mA_FS_CURRENT 0x06
#define LM3697_10p6mA_FS_CURRENT 0x07
#define LM3697_11p4mA_FS_CURRENT 0x08
#define LM3697_12p2mA_FS_CURRENT 0x09
#define LM3697_13mA_FS_CURRENT 0x0A
#define LM3697_13p8mA_FS_CURRENT 0x0B
#define LM3697_14p6mA_FS_CURRENT 0x0C
#define LM3697_15p4mA_FS_CURRENT 0x0D
#define LM3697_16p2mA_FS_CURRENT 0x0E
#define LM3697_17mA_FS_CURRENT 0x0F
#define LM3697_17p8mA_FS_CURRENT 0x10
#define LM3697_18p6mA_FS_CURRENT 0x11
#define LM3697_19p4mA_FS_CURRENT 0x12
#define LM3697_20p2mA_FS_CURRENT 0x13
#define LM3697_21mA_FS_CURRENT 0x14
#define LM3697_21p8mA_FS_CURRENT 0x15
#define LM3697_22p6mA_FS_CURRENT 0x16
#define LM3697_23p4mA_FS_CURRENT 0x17
#define LM3697_24p2mA_FS_CURRENT 0x18
#define LM3697_25mA_FS_CURRENT 0x19
#define LM3697_25p8mA_FS_CURRENT 0x1A
#define LM3697_26p6mA_FS_CURRENT 0x1B
#define LM3697_27p4mA_FS_CURRENT 0x1C
#define LM3697_28p2mA_FS_CURRENT 0x1D
#define LM3697_29mA_FS_CURRENT 0x1E
#define LM3697_29p8mA_FS_CURRENT 0x1F

/*****************************************************************************
* KTD3136 registers
*****************************************************************************/
#define KTD3161_DEVICE_ID_REG		0x00
#define KTD3161_SW_RESET_REG		0x01
#define KTD3161_MODE_REG			0x02
#define KTD3161_CONTROL_REG			0x03
#define KTD3161_LED_CUR_LSB_REG		0x04
#define KTD3161_LED_CUR_MSB_REG		0x05
#define KTD3161_PWM_REG				0x06
#define KTD3161_TURNIMG_RAMP_REG	0x07
#define KTD3161_TRANSITION_REG		0x08
#define KTD3161_STATUS_REG			0x0A

struct lm3697_platform_data {
    int (*init)(void);
void (*exit)(void);
int (*power_on)(void);
int (*power_off)(void);
int (*bkg_enable)(unsigned en);

unsigned flags;
unsigned ramp_time; /* Ramp time if ramping in the driver */
unsigned ctrl_a_fs_current; /* control a full scale current */
unsigned ctrl_b_fs_current; /* control b full scale current */
unsigned ctrl_c_fs_current; /* control c full scale current */
unsigned ctrl_a_mapping_mode; /* Control A Mapping mode (linear/exp) */
unsigned ctrl_b_mapping_mode; /* Control B Mapping mode (linear/exp) */
unsigned ctrl_c_mapping_mode; /* Control C Mapping mode (linear/exp) */
unsigned ctrl_a_pwm;
unsigned ctrl_b_pwm;
};

#endif /* __LEDS_lm3697_H__ */
