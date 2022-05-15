/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_config.h
*
* Author: Software Development Team, AE
*
* Created: 2016-08-01
*
* Abstract: global function for test
*
************************************************************************/
#ifndef __LINUX_TEST_CONFIG__
#define __LINUX_TEST_CONFIG__

#include "../focaltech_core.h"


/*-----------------------------------------------------------
IC Type Test
-----------------------------------------------------------*/
#define FT5X46_TEST     0x001
#define FT6X36_TEST     0x002
#define FT5822_TEST     0x004
#define FT8606_TEST     0x008
#define FT8716_TEST     0x010
#define FT3C47_TEST     0x020
#define FT8607_TEST     0x040
#define FT8736_TEST     0x080
#define FT3D47_TEST     0x100
#define FTE716_TEST     0x200
#define FTE736_TEST     0x400
#define FT8006_TEST     0x800


//#ifdef  FTS_CHIP_TYPE
//
//#if (FTS_CHIP_TYPE == _FT8716)
//#define FTS_CHIP_TEST_TYPE      FT8716_TEST
//#elif(FTS_CHIP_TYPE == _FT8736)
//#define FTS_CHIP_TEST_TYPE      FT8736_TEST
//#elif(FTS_CHIP_TYPE == _FT8006)
//#define FTS_CHIP_TEST_TYPE      FT8006_TEST
//#elif(FTS_CHIP_TYPE == _FT8606)
//#define FTS_CHIP_TEST_TYPE      FT8606_TEST
//#elif(FTS_CHIP_TYPE == _FT8607)
//#define FTS_CHIP_TEST_TYPE      FT8607_TEST
//#elif(FTS_CHIP_TYPE == _FTE716)
//#define FTS_CHIP_TEST_TYPE      FTE716_TEST
//#elif(FTS_CHIP_TYPE == _FT3D47)
//#define FTS_CHIP_TEST_TYPE      FT3D47_TEST
//#elif(IC_SERIALS == 0x01)
//#define FTS_CHIP_TEST_TYPE      FT5822_TEST
//#elif(FTS_CHIP_TYPE == _FT3C47U)
//#define FTS_CHIP_TEST_TYPE      FT3C47_TEST
//#elif(IC_SERIALS == 0x02)
//#define FTS_CHIP_TEST_TYPE      FT5X46_TEST
//#elif((IC_SERIALS == 0x03) || (IC_SERIALS == 0x04))
//#define FTS_CHIP_TEST_TYPE      FT6X36_TEST
//#endif
//
//#else
//#define FTS_CHIP_TEST_TYPE          FT8716_TEST
//
//#endif
//
//#ifdef FTS_CHIP_TEST_TYPE
//#undef FTS_CHIP_TEST_TYPE
#define FTS_CHIP_TEST_TYPE (FT8736_TEST | FT8006_TEST)
//#endif

#endif /* __LINUX_TEST_CONFIG__ */
