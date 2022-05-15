#include "vts_core.h"


#if (defined(CONFIG_ARCH_QCOM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)))
#define call_symbol(symbol, fmt, arg1...) \
		do { \
			return symbol(arg1); \
	 } while (0)
#else
#define call_symbol(symbol, fmt, arg1...) \
	static bool searched = false; \
	static typeof(&symbol) func = NULL; \
	do { \
		if (func) { \
			return func(arg1); \
		} \
		\
		if (searched) { \
			VTE("can not find function "#symbol); \
			break; \
		} \
		\
		func = symbol_get(symbol); \
		searched = true; \
		if (!func) { \
			VTE("can not find function "#symbol); \
			break; \
		} \
		\
		return func(arg1); \
 } while (0)
#endif


int vts_channel_broken_collect(int type, u8 Txnum, u8 Rxnum, u16 Txbit, long long Rxbit)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_channel_broken,"%d, %d, %d, %d, %llx", type, Txnum, Rxnum, Txbit, Rxbit);
	#endif
	return 0;
}

int vts_firmwware_version_collect(int type, int lcmid, u64 fwversion)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_firmware_version, "%d, %d, %lld", type, lcmid, fwversion);
	#endif
	return 0;
}

int vts_virtual_prox_enable_collect(int type, unsigned int enable)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_virtual_prox_enable, "%d, %d", type, enable);
	#endif
	return 0;
}

int vts_communication_abnormal_collect(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_communication_abnormal, "%d", type);
	#endif
	return 0;
}

int vts_click_frequency_collect(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_click_frequency, "%d", type);
	#endif
	return 0;
}

int vts_ng_panel_collect(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_ng_panel, "%d", type);
	#endif
	return 0;
}

int vts_hsync_state_collect(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_hsync_state, "%d", type);
	#endif
	return 0;
}

int vts_abnormal_reset_collect(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_abnormal_reset, "%d", type);
	#endif
	return 0;
}

int vts_ic_mode_collect(int type, int mode, int mode_in)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_ic_mode, "%d, %d, %d", type, mode, mode_in);
	#endif
	return 0;
}

int vts_int_cost_time(int type, int t_to_k, int int_bottom)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_int_cost_time, "%d %d %d", type, t_to_k, int_bottom);
	#endif
	return 0;
}

int vts_large_press_event(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_large_press_event, "%d", type);
	#endif
	return 0;
}
int vts_multi_touch(int type, int finger_type, int finger_cost_time)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_multi_touch, "%d %d %d", type, finger_type, finger_cost_time);
	#endif
	return 0;
}
int vts_press_area(int type, int area_type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_press_area, "%d %d", type, area_type);
	#endif
	return 0;
}
int vts_sensor_test_collect(int type, int pass, int fail)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_sensor_test, "%d %d %d", type, pass, fail);
	#endif
	return 0;
}
int vts_edge_reject_fail(int type)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_edge_reject_fail, "%d", type);
	#endif
	return 0;
}
int vts_resume_to_touch(int type, int min_time)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_resume_to_touch, "%d %d", type, min_time);
	#endif
	return 0;
}
int vts_handler_cost(int type, enum touch_vcode_lock_type lock_type, int lock_time)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_handler_cost, "%d %d %d", type, lock_type, lock_time);
	#endif
	return 0;
}
int vts_five_more_time(int type, int nr_touchs)
{
	#ifdef TOUCH_DATA_COLLECT
	call_symbol(report_five_more_time, "%d %d", type, nr_touchs);
	#endif
	return 0;
}