#ifndef __VIVO_TOUCHSCREEN_CONFIG_H__
#define __VIVO_TOUCHSCREEN_CONFIG_H__
enum {
	FS_FUNCTION_BEG = 0,
	FS_GLOVES_MODE = FS_FUNCTION_BEG,
	FS_LARGE_OBJ_SUPPRESSION,
	FS_DCLICK_WAKE,
	FS_GESTURE_MODE,                      /*1=sold in native,2=sold in abroad,3...etc=for custom extensive*/
	FS_CUSTOM_GESTURE_MODE,
	FS_TS_MODULE_ID_METHODS,              /*0=by IC id,1=gpio,2=LCD ic,3=...etc=for custom extensive*/
	FS_ANTI_ESD,
	FS_CHR_CONN_JUDGE,
	FS_MODULE_SMT_MODE,                   /*0=COB,1=COF*/
	FS_TS_FW_UPGRADE_LCD_ID_REF,
	FS_TS_SENSITIVIITY_ADJUST,
	/*-***NOTE***: DON'T forget to add the "function_set_name",when you add some enums here! *****/
	FS_FUNCTION_END
};

/*FS_TS_MODULE_ID_METHODS*/
enum {
	TMID_BY_ICID = 0,
	TMID_BY_GPIO,
	TMID_BY_LCDID,
};

/*FS_GESTURE_MODE*/
enum {
	GM_CLOSE = 0,
	GM_SOLD_NATIVE,
	GM_SOLD_ABROAD,
};

/*FS_MODULE_SMT_MODE*/
enum {
	TMSM_BY_COB = 0,
	TMSM_BY_COF,
};

int vivo_touchscreen_id_pinctrl(int active);
int vivo_touchscreen_test_ic_in_use(const char *ic_name);
int vivo_touchscreen_is_support(int function);
int vivo_touchscreen_get_module_id_gpio(int idx);
void vivo_touchscreen_get_product_name(const char **pp_Product_name);
void vivo_touchscreen_get_vdd_which_mode(const char **vdd_which_mode);
#endif
