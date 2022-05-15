/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MAIN_OIS_H

#define _MAIN_OIS_H

#include <linux/ioctl.h>
/*chenhan add*/
#include "ois_core.h"
/*add end*/
/*chehnan add*/
#define OISDRV_LC89129 "LC89129"
/*add end*/
/*chehnan add*/
#define OISDRV_DW9781C "DW9781C"
/*add end*/
/*chenhan add for ois*/
#define AFIOC_X_OIS_START           _IOWR('A', 50, int)
#define AFIOC_X_OIS_SETMODE         _IOWR('A', 51, int)
#define AFIOC_X_OIS_SETACC          _IOWR('A', 52, struct ois_acc_param)
#define AFIOC_X_OIS_STATUSCHECK     _IOWR('A', 53, struct ois_flash_info)
#define AFIOC_X_OIS_SETGYROGAIN     _IOWR('A', 54, int[3])
#define AFIOC_X_OIS_SETFIXMODE      _IOWR('A', 55, struct ois_fixmode_parameter)
#define AFIOC_X_OIS_SETSINEMODE     _IOWR('A', 56, struct ois_sinemode_parameter)
#define AFIOC_X_OIS_SETCIRCLEMODE   _IOWR('A', 57, struct ois_circlemode_parameter)
#define AFIOC_X_OIS_SETSTROKELIMIT  _IOWR('A', 58, int)
#define AFIOC_X_OIS_SETPANTILT      _IOWR('A', 59, struct ois_pantilt_param)
#define AFIOC_X_OIS_GETMODE         _IOWR('A', 60, int)
#define AFIOC_X_OIS_GETFWVERSION    _IOWR('A', 61, int)
#define AFIOC_X_OIS_GETGYROOFFSET   _IOWR('A', 62, int[2])
#define AFIOC_X_OIS_GETLENSINFO     _IOWR('A', 63, struct ois_lens_info_buf)
#define AFIOC_X_OIS_GETINITINFO     _IOWR('A', 64, struct ois_flash_info)
#define AFIOC_X_OIS_GETOTPINFO      _IOWR('A', 65, struct ois_otp_info)
#define AFIOC_X_OIS_OFFSETCAL       _IOWR('A', 66, int)
#define AFIOC_X_OIS_FWUPDATE        _IOWR('A', 67, int)
#define AFIOC_X_OIS_FLASHSAVE       _IOWR('A', 68, int)
#define AFIOC_X_OIS_INIT            _IOWR('A', 69, int)
#define AFIOC_X_OIS_DEINIT          _IOWR('A', 70, int)
#define AFIOC_X_OIS_STREAMON        _IOWR('A', 71, int)
#define AFIOC_X_OIS_STREAMOFF       _IOWR('A', 72, int)
#define AFIOC_X_OIS_GETGYROGAIN     _IOWR('A', 73, int[2])
#define AFIOC_X_OIS_SETTRIPOD       _IOWR('A', 74, int)
#define AFIOC_X_OIS_SETSMOOTH       _IOWR('A', 75, struct ois_smooth_info)
#define AFIOC_X_OIS_INIT_SLAVE      _IOWR('A', 76, int)
#define AFICO_X_OIS_VSYNC           _IOWR('A', 77, int)
#define AFIOC_X_OIS_READY_CHECK     _IOWR('A', 78, int)
#define AFICO_X_OIS_LOG_LEVEL       _IOWR('A', 79, int)
#define AFICO_X_OIS_GETLOOPGAIN     _IOWR('A', 80, int[2])
#define AFIOC_X_OIS_END             _IOWR('A', 81, int)

/*add end*/

#endif

