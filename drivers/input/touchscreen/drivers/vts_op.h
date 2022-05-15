#ifndef __VTS_OP_H
#define __VTS_OP_H

enum vts_boot_mode {
	VTS_BOOT_MODE_NORMAL,
	VTS_BOOT_MODE_AT,
	VTS_BOOT_MODE_POWER_OFF_CHARGING,
	VTS_BOOT_MODE_OS_BOOT_LOADFAIL,
};

enum vts_boot_mode vts_get_boot_mode(void);
int vts_dsi_panel_reset_power_ctrl(int enable);
unsigned int vts_msdd_report_lcm_id(u32 err_default);
unsigned int vts_msdd_report_lcm1_id(u32 err_default);

#endif
