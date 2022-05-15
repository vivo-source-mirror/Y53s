#ifndef CTS_CONFIG_H
#define CTS_CONFIG_H

/** Driver version */
#define CFG_CTS_DRIVER_MAJOR_VERSION        1
#define CFG_CTS_DRIVER_MINOR_VERSION        3
#define CFG_CTS_DRIVER_PATCH_VERSION        16

#define CFG_CTS_DRIVER_VERSION              "v1.3.16"

/** Whether reset pin is used */
#define CFG_CTS_HAS_RESET_PIN

//#define CONFIG_CTS_I2C_HOST
#ifndef CONFIG_CTS_I2C_HOST

#ifndef CFG_CTS_HAS_RESET_PIN
#define CFG_CTS_HAS_RESET_PIN
#endif

#define CFG_CTS_SPI_SPEED_KHZ               4000

#endif

/* Handle IRQ whether in workqueue or kthread. */
//#define CFG_CTS_HANDLE_IRQ_USE_WORKQUEUE

/* With SCHED_RR option, kthread will get better performance by
 * dramatic decrease delay between irq and work.
 */
//#define CFG_CTS_HANDLE_IRQ_USE_KTHREAD

/* UP event missing by some reason will cause touch remaining when
 * all fingers are lifted or flying line when touch in the next time.
 * Enable following option will prevent this.
 * FIXME:
 *   Currently, one finger UP event missing in multi-touch, it will
 *   report UP to system when all fingers are lifted.
 */
//#define CFG_CTS_MAKEUP_EVENT_UP

//#define CFG_CTS_FW_LOG_REDIRECT

/** Whether force download firmware to chip */
//#define CFG_CTS_FIRMWARE_FORCE_UPDATE

/** Use build in firmware or firmware file in fs*/
#define CFG_CTS_DRIVER_BUILTIN_FIRMWARE
#define CFG_CTS_FIRMWARE_IN_FS
#ifdef CFG_CTS_FIRMWARE_IN_FS
    #define CFG_CTS_FIRMWARE_FILENAME       "ICNL9911.bin"
    #define CFG_CTS_FIRMWARE_FILEPATH       "/etc/firmware/ICNL9911.bin"
#endif /* CFG_CTS_FIRMWARE_IN_FS */

#ifdef CONFIG_PROC_FS
    /* Proc FS for backward compatibility for APK tool com.ICN85xx */
    #define CONFIG_CTS_LEGACY_TOOL
#endif /* CONFIG_PROC_FS */

#ifdef CONFIG_SYSFS
    /* Sys FS for gesture report, debug feature etc. */
    #define CONFIG_CTS_SYSFS
#endif /* CONFIG_SYSFS */

#define CFG_CTS_MAX_TOUCH_NUM               (10)

/* Virtual key support */
//#define CONFIG_CTS_VIRTUALKEY
#ifdef CONFIG_CTS_VIRTUALKEY
    #define CFG_CTS_MAX_VKEY_NUM            (4)
    #define CFG_CTS_NUM_VKEY                (3)
    #define CFG_CTS_VKEY_KEYCODES           {KEY_BACK, KEY_HOME, KEY_MENU}
#endif /* CONFIG_CTS_VIRTUALKEY */

/* Gesture wakeup */
#define CFG_CTS_GESTURE
#ifdef CFG_CTS_GESTURE
//gesture add start
#define _BIT(val)                   (1 << val)
#define GSTR_EN_MASK_D_TAP           _BIT(0)
#define GSTR_EN_MASK_UP              _BIT(1)
#define GSTR_EN_MASK_DOWN            _BIT(2)
#define GSTR_EN_MASK_LEFT            _BIT(3)
#define GSTR_EN_MASK_RIGHT           _BIT(4)
#define GSTR_EN_MASK_O               _BIT(5)
#define GSTR_EN_MASK_M               _BIT(6)
#define GSTR_EN_MASK_W               _BIT(7)
#define GSTR_EN_MASK_V               _BIT(8)
#define GSTR_EN_MASK_CARET           _BIT(9)
#define GSTR_EN_MASK_MORE            _BIT(10)
#define GSTR_EN_MASK_LESS            _BIT(11)
#define GSTR_EN_MASK_VLINES          _BIT(12)
#define GSTR_EN_MASK_C               _BIT(13)
#define GSTR_EN_MASK_E               _BIT(14)
#define GSTR_EN_MASK_S               _BIT(15)
#define GSTR_EN_MASK_B               _BIT(16)
#define GSTR_EN_MASK_T               _BIT(17)
#define GSTR_EN_MASK_H               _BIT(18)
#define GSTR_EN_MASK_F               _BIT(19)
#define GSTR_EN_MASK_X               _BIT(20)
#define GSTR_EN_MASK_Z               _BIT(21)   
#define GSTR_EN_MASK_A               _BIT(22)  
//gesture add end


#define GESTURE_UP                          0x11
#define GESTURE_C                           0x12
#define GESTURE_O                           0x13
#define GESTURE_M                           0x14
#define GESTURE_W                           0x15
#define GESTURE_E                           0x16
#define GESTURE_S                           0x17
#define GESTURE_B                           0x18
#define GESTURE_T                           0x19
#define GESTURE_H                           0x1a
#define GESTURE_F                           0x1b
#define GESTURE_X                           0x1c
#define GESTURE_Z                           0x1d
#define GESTURE_V                           0x1e
#define GESTURE_CARET                       0x1f
#define GESTURE_LESS                        0x20
#define GESTURE_MORE                        0x21
#define GESTURE_DOWN                        0x22
#define GESTURE_LEFT                        0x23
#define GESTURE_RIGHT                       0x24
#define GESTURE_DOUBLE_VLINES               0x25
#define GESTURE_A                           0x26        // @
#define GESTURE_D_TAP                       0x50
#define GESTURE_DOWN                        0x22
#define GESTURE_LEFT                        0x23
#define GESTURE_RIGHT                       0x24

#define CFG_CTS_NUM_GESTURE             (14u)
#define CFG_CTS_GESTURE_REPORT_KEY
#define CFG_CTS_GESTURE_KEYMAP      \
    {{GESTURE_D_TAP, VTS_EVENT_GESTURE_DOUBLE_CLICK,},   \
     {GESTURE_UP, VTS_EVENT_GESTURE_PATTERN_UP,},         \
     {GESTURE_DOWN, VTS_EVENT_GESTURE_PATTERN_DOWN,},     \
     {GESTURE_LEFT, VTS_EVENT_GESTURE_PATTERN_LEFT,},     \
     {GESTURE_RIGHT, VTS_EVENT_GESTURE_PATTERN_RIGHT,},   \
     {GESTURE_C, VTS_EVENT_GESTURE_PATTERN_C,},           \
     {GESTURE_W, VTS_EVENT_GESTURE_PATTERN_W,},           \
     {GESTURE_V, VTS_EVENT_GESTURE_PATTERN_V,},           \
     {GESTURE_Z, KEY_Z,},           \
     {GESTURE_M, VTS_EVENT_GESTURE_PATTERN_M,},           \
     {GESTURE_O, VTS_EVENT_GESTURE_PATTERN_O,},           \
     {GESTURE_E, VTS_EVENT_GESTURE_PATTERN_E,},           \
     {GESTURE_S, KEY_S,},           \
     {GESTURE_F, VTS_EVENT_GESTURE_PATTERN_F,},           \
    }
#define CFG_CTS_GESTURE_REPORT_TRACE    1
#endif /* CFG_CTS_GESTURE */
//#define CONFIG_CTS_GLOVE

//#define CONFIG_CTS_CHARGER_DETECT

//#define CONFIG_CTS_EARJACK_DETECT


/* ESD protection */
//#define CONFIG_CTS_ESD_PROTECTION
#ifdef CONFIG_CTS_ESD_PROTECTION
    #define CFG_CTS_ESD_PROTECTION_CHECK_PERIOD         (2 * HZ)
    #define CFG_CTS_ESD_FAILED_CONFIRM_CNT              3
#endif /* CONFIG_CTS_ESD_PROTECTION */

/* Use slot protocol (protocol B), comment it if use protocol A. */
#define CONFIG_CTS_SLOTPROTOCOL

#ifdef CONFIG_CTS_LEGACY_TOOL
    #define CFG_CTS_TOOL_PROC_FILENAME      "icn85xx_tool"
#endif /* CONFIG_CTS_LEGACY_TOOL */

/****************************************************************************
 * Platform configurations
 ****************************************************************************/

#include "cts_plat_qcom_config.h"

#endif /* CTS_CONFIG_H */

