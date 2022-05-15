/*==============================================================================

		 S E N S O R S    M A G N E T O M E T E R    D R I V E R

DESCRIPTION

  Defines the interface for the magnetometer driver


-- Copyright Notice --

  Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
  All Rights Reserved.

-- End Asahi Kasei Microdevices Copyright Notice --
==============================================================================*/

/*==============================================================================

					  EDIT HISTORY FOR FILE

  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.

$Id: //components/rel/ssc.slpi/2.0/dd/qcom/src/sns_dd_mag_akm099xx_priv.h#1 $


when         who     what, where, why
----------   ---     -----------------------------------------------------------
01/31/13     AKM    AK09912
10/05/11     RY     Updated AKM_MAG8963_LO_PWR from 0 to 1 (uA).
09/15/11     RY     Modified for AK8963.
01/18/11     OF     Initial revision (based on sns_dd_mag_priv.h)
02/27/11     OF     Add Device ID and Device Info, read data in one 8 byte read
03/24/11     OF     Adjust sensitivity values to the measurement data read out
==============================================================================*/

#ifndef _SNSD_MAG_AKM099XX_PRIV_H
#define _SNSD_MAG_AKM099XX_PRIV_H

/* AKM sensor defines */
#define AKM099XX_MAG_I2C_ADDR1                 0x0C
#define AKM099XX_MAG_I2C_ADDR2                 0x0E
#define AKM099XX_MAG_I2C_ADDR3                 0x0D
#define AKM099XX_MAG_I2C_ADDR4                 0x0F

/* AKM Register Addresses */
#define AKM_MAG_REG_ADDR_WIA1_099XX            0x00
#define AKM_MAG_REG_ADDR_WIA2_099XX            0x01
#define AKM_MAG_REG_ADDR_INFO1_099XX           0x02
#define AKM_MAG_REG_ADDR_INFO2_099XX           0x03

#define AKM_MAG_REG_ADDR_STATUS1_099XX         0x10
#define AKM_MAG_REG_ADDR_HXL_099XX             0x11
#define AKM_MAG_REG_ADDR_HXH_099XX             0x12
#define AKM_MAG_REG_ADDR_HYL_099XX             0x13
#define AKM_MAG_REG_ADDR_HYH_099XX             0x14
#define AKM_MAG_REG_ADDR_HZL_099XX             0x15
#define AKM_MAG_REG_ADDR_HZH_099XX             0x16
#define AKM_MAG_REG_ADDR_TEMPERATURE_099XX     0x17
#define AKM_MAG_REG_ADDR_STATUS2_099XX         0x18

#define AKM_MAG_REG_ADDR_CNTL1_099XX           0x30
#define AKM_MAG_REG_ADDR_CNTL2_099XX           0x31
#define AKM_MAG_REG_ADDR_CNTL3_099XX           0x32
#define AKM_MAG_REG_ADDR_I2C_DISABLE_099XX     0x36

#define AKM_MAG_REG_ADDR_X_SEN_ADJUST_099XX    0x60
#define AKM_MAG_REG_ADDR_Y_SEN_ADJUST_099XX    0x61
#define AKM_MAG_REG_ADDR_Z_SEN_ADJUST_099XX    0x62

#define AKM_MAG_COMPANY_ID                     0x48
/*add by vivo sensor team default enable soft iron function start*/
/* Enable PDC function */
#define AKM_MAG_ENABLE_PDC
#define CONFIG_VIVO_SKIP_MAG_OVEARFLOW
/*add by vivo sensor team default enable soft iron function end*/

/* AKM INT bit mask */
#define AKM_MAG_DRDY_BIT_MASK                  0x1

/* AKM Magnetic sensor overflow bit mask */
#define AKM_MAG_HOFL_BIT_MASK                  0x8

/* Soft reset */
#define AKM_MAG_SOFT_RESET                     0x1

/* AKM099XX number of data types*/
#define AKM_MAG099XX_NUM_SUPPORTED_SENSORS     1
#define AKM_MAG099XX_NUM_DATATYPES             3
#define AKM_MAG099XX_NUM_SENSITIVITY           3
#define AKM_MAG099XX_NUM_DATA_BYTES            9

/* The following parameters are different from each devices. */

/* DEVICE_ID */
#define AKM_MAG09912_DEVICE_ID                 0x04
#define AKM_MAG09911_DEVICE_ID                 0x05
/*add by vivo sensor add for akm09918 start*/
#define AKM_MAG09918_DEVICE_ID                 0x0C
 /* Power consumption limits */
#define AKM_MAG09918_LO_PWR                    (1)        /* unit of uA */
#define AKM_MAG09918_HI_PWR                    (1100)     /* unit of uA @ 100Hz */
#define AKM_MAG09918_MIN_RANGE                 (-3219128) /* Minimum -49.12G */
#define AKM_MAG09918_MAX_RANGE                 (3219128)  /* Maximum  49.12G */
#define AKM_MAG09918_MAX_VALUE                 (32752)    /* Maximum value of every axis */
 /* Resulution */
#define AKM_MAG09918_RESOLUTION_ADC            (16)       /* 16 bit */
/* Sampling frequency */
#define AKM_MAG09918_SUPPORTED_ODR             AKM_MAG099XX_SUPPORTED_ODR_100HZ

#define AKM_MAG09918_SENSITIVITY               (0.15f)  /* Gauss/LSB */
/*add by vivo sensor add for akm09918 end*/
/* AKM09912 option settings */
#define AKM_MAG09912_TEMPERATURE_MASK          0x80
#define AKM_MAG09912_NSF_BIT_MASK              0x40

/* AKM09912 power consumption limits */
#define AKM_MAG09912_LO_PWR     3       /* unit of uA */
#define AKM_MAG09912_HI_PWR     100     /* unit of uA @ 10Hz */
/* AKM09911 power consumption limits */
#define AKM_MAG09911_LO_PWR     3       /* unit of uA */
#define AKM_MAG09911_HI_PWR     240     /* unit of uA @ 10Hz */

/* AKM09912 range */
#define AKM_MAG09912_MIN_RANGE -3219128 /* Minimum range in Q16 Gauss units -49.12G */
#define AKM_MAG09912_MAX_RANGE  3219128 /* Maximum range in Q16 Gauss units  49.12G */
/* AKM09911 range */
#define AKM_MAG09911_MIN_RANGE -3219128 /* Minimum range in Q16 Gauss units -49.12G */
#define AKM_MAG09911_MAX_RANGE  3219128 /* Maximum range in Q16 Gauss units  49.12G */

/* AKM09912 resulution */
#define AKM_MAG09912_RESOLUTION_ADC 16       /* 16 bit (0.15 uT / LSB) */
/* AKM09911 resulution */
#define AKM_MAG09911_RESOLUTION_ADC 14       /* 14 bit (0.6  uT / LSB) */

/*
 * From the table 5.3.3 of the magnetometer datasheet we see
 * that the sensitivity of the magnetometer is:
 * Expressed in Gauss units (1 Tesla == 10000 Gauss, 1 microTesla == 0.01Gauss) */
/*  AKM09912 sensitivity: 0.15 microTesla/LSB */
#define AKM_MAG09912_SENSITIVITY    (0.15f)   /* uT/LSB */
/*  AKM09911 sensitivity: 0.6  microTesla/LSB */
#define AKM_MAG09911_SENSITIVITY    (0.6f)    /* uT/LSB */

/* Measurement time, us */
#define AKM_MAG09918_MEASUREMENT_TIME_TYP  7200
#define AKM_MAG09918_MEASUREMENT_TIME_MAX  8300

#endif /* End include guard  _SNSD_MAG_AKM099XX_PRIV_H */

