#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include "vts_core.h"

extern unsigned int is_atboot;
extern unsigned int power_off_charging_mode;
extern unsigned int os_boot_loadfail;

#define VTS_LCMID_DEFAULT 255

extern int get_boot_mode(void);

#if defined(CONFIG_MEDIATEK_SOLUTION)
enum vts_boot_mode vts_get_boot_mode(void)
{
	int boot_mode = get_boot_mode();
	VTI("boot_mode:%d\n", boot_mode);

	if (is_atboot == 1)
		return VTS_BOOT_MODE_AT;

	if (boot_mode == 8 /*KERNEL_POWER_OFF_CHARGING_BOOT*/  ||
		boot_mode == 9 /*LOW_POWER_OFF_CHARGING_BOOT*/ ||
		boot_mode == 1 /*META_BOOT*/ ||
		boot_mode == 4 /*FACTORY_BOOT*/ ||
		boot_mode == 5 /*ADVMETA_BOOT*/) {
		VTI("boot in at mode or power off charging mode");
		return VTS_BOOT_MODE_POWER_OFF_CHARGING;
	}

	if (os_boot_loadfail == 1) {
		VTI("boot in os_boot_puresys mode\n");
		return VTS_BOOT_MODE_OS_BOOT_LOADFAIL;
	}

	return VTS_BOOT_MODE_NORMAL;
}
#else
enum vts_boot_mode vts_get_boot_mode(void)
{
	if (is_atboot == 1) {
		VTI("boot in at mode\n");
		return VTS_BOOT_MODE_AT;
	}

	if (power_off_charging_mode == 1) {
		VTI("power off charging mode\n");
		return VTS_BOOT_MODE_POWER_OFF_CHARGING;
	}
	if (os_boot_loadfail == 1) {
		VTI("boot in os_boot_puresys mode\n");
		return VTS_BOOT_MODE_OS_BOOT_LOADFAIL;
	}

	return VTS_BOOT_MODE_NORMAL;
}
#endif

#if (defined(CONFIG_ARCH_QCOM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)))
#define call_symbol(symbol, fmt, arg1...) \
		do { \
			VTI("call "#symbol" with "fmt"\n", ##arg1); \
			return symbol(arg1); \
	 } while (0)
#else
#define call_symbol(symbol, fmt, arg1...) \
	static bool searched = false; \
	static typeof(&symbol) func = NULL; \
	do { \
		if (func) { \
			VTI("call "#symbol" with "fmt"\n", ##arg1); \
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
		VTI("call "#symbol" with "fmt"\n",##arg1); \
		return func(arg1); \
 } while (0)
#endif

extern int dsi_panel_set_tsp_sync(bool enable);
int tsp_sync_enable(bool enable)
{
	call_symbol(dsi_panel_set_tsp_sync, "%d", enable);
	VTI("set tsp sync with value %d", enable);
	return 0;
}
EXPORT_SYMBOL(tsp_sync_enable);

extern int mdss_dsi_panel_reset_and_powerctl(int enable);
int vts_dsi_panel_reset_power_ctrl(int enable)
{
	call_symbol(mdss_dsi_panel_reset_and_powerctl, "%d", enable);
	return -EFAULT;
}

extern unsigned int mdss_report_lcm_id(void);
extern unsigned int mdss_report_lcm1_id(void);
extern unsigned int mdss_report_lcm2_id(void);
u32 vts_get_msdd_report_lcm_id(enum vts_type type)
{
	u32 lcm_id;

	if (type == VTS_TYPE_MAIN)
		lcm_id = vts_msdd_report_lcm_id(VTS_LCMID_DEFAULT);
	else if (type == VTS_TYPE_SECOND)
		lcm_id = vts_msdd_report_lcm1_id(VTS_LCMID_DEFAULT);
	else {
		VTE("module panel type error!");
		return 0xff;
	}
	lcm_id = lcm_id & 0xffff;
	return lcm_id;

}

u32 vts_msdd_report_lcm_id(u32 err_default)
{
	call_symbol(mdss_report_lcm_id, " ");
	return err_default;
}

u32 vts_msdd_report_lcm1_id(u32 err_default)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	call_symbol(mdss_report_lcm1_id, " ");
#endif
#ifdef VTS_SECOND_LCMID
	call_symbol(mdss_report_lcm2_id, " ");
#endif
	return err_default;
}
