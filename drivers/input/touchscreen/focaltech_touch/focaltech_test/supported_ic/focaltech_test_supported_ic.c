/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_supported_ic.c
*
* Author: Software Development
*
* Created: 2016-08-01
*
* Abstract: test item for FT8716
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/


#include "../include/focaltech_test_supported_ic.h"
#include "../focaltech_test_config.h"



void OnInit_FT8607_TestItem(char*  strIniFile);
void OnInit_FT8607_BasicThreshold(char* strIniFile);
void SetTestItem_FT8607(void);
boolean FT8607_StartTest(void);
int FT8607_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

boolean FT8716_StartTest(void);
int FT8716_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT8716_TestItem(char *strIniFile);
void OnInit_FT8716_BasicThreshold(char *strIniFile);
void SetTestItem_FT8716(void);

boolean FT3D47_StartTest(void);
int FT3D47_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT3D47_TestItem(char *strIniFile);
void OnInit_FT3D47_BasicThreshold(char *strIniFile);
void SetTestItem_FT3D47(void);

boolean FT5X46_StartTest(void);
int FT5X46_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT5X46_TestItem(char *strIniFile);
void OnInit_FT5X46_BasicThreshold(char *strIniFile);
void SetTestItem_FT5X46(void);

boolean FT6X36_StartTest(void);
int FT6X36_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT6X36_TestItem(char *strIniFile);
void OnInit_FT6X36_BasicThreshold(char *strIniFile);
void SetTestItem_FT6X36(void);

boolean FT5822_StartTest(void);
int FT5822_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT5822_TestItem(char *strIniFile);
void OnInit_FT5822_BasicThreshold(char *strIniFile);
void SetTestItem_FT5822(void);

boolean FTE716_StartTest(void);
int FTE716_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FTE716_TestItem(char *strIniFile);
void OnInit_FTE716_BasicThreshold(char *strIniFile);
void SetTestItem_FTE716(void);


boolean FT8736_StartTest(void);
int FT8736_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT8736_TestItem(char *strIniFile);
void OnInit_FT8736_BasicThreshold(char *strIniFile);
void SetTestItem_FT8736(void);

boolean FT8006_StartTest(void);
int FT8006_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT8006_TestItem(char *strIniFile);
void OnInit_FT8006_BasicThreshold(char *strIniFile);
void SetTestItem_FT8006(void);

boolean FT8606_StartTest(void);
int FT8606_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT8606_TestItem(char *strIniFile);
void OnInit_FT8606_BasicThreshold(char *strIniFile);
void SetTestItem_FT8606(void);

boolean FT3C47_StartTest(void);
int FT3C47_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80
void OnInit_FT3C47_TestItem(char *strIniFile);
void OnInit_FT3C47_BasicThreshold(char *strIniFile);
void SetTestItem_FT3C47(void);


struct StTestFuncs g_allTestFuncs[] = {


#if (FTS_CHIP_TEST_TYPE & FT8006_TEST)
	{
		.type = _FT8006,
		.OnInit_TestItem = OnInit_FT8006_TestItem,
		.OnInit_BasicThreshold = OnInit_FT8006_BasicThreshold,
		.SetTestItem  = SetTestItem_FT8006,
		.Start_Test = FT8006_StartTest,
		.Get_test_data = FT8006_get_test_data,
	},
#endif

#if (FTS_CHIP_TEST_TYPE & FT8736_TEST)
	{
		.type = _FT8736,
		.OnInit_TestItem = OnInit_FT8736_TestItem,
		.OnInit_BasicThreshold = OnInit_FT8736_BasicThreshold,
		.SetTestItem  = SetTestItem_FT8736,
		.Start_Test = FT8736_StartTest,
		.Get_test_data = FT8736_get_test_data,
	},
#endif

};

void fts_test_funcs(void)
{
	int size = sizeof(g_allTestFuncs) / sizeof(struct StTestFuncs);
	int i;

	FTS_TEST_FUNC_ENTER();
	memcpy(&g_stTestFuncs, &g_allTestFuncs[0], sizeof(struct StTestFuncs));


	for (i = 0; i < size; i++) {
		if (g_global.ic_type == g_allTestFuncs[i].type) {
			FTS_TEST_DBG("%lx selected", g_allTestFuncs[i].type);
			memcpy(&g_stTestFuncs, &g_allTestFuncs[i], sizeof(struct StTestFuncs));
			break;
		}
	}


	FTS_TEST_FUNC_EXIT();
}
