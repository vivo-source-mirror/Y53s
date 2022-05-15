#ifndef _VPERF_MONITOR_UMS_H_
#define _VPERF_MONITOR_UMS_H_

#include "usb.h"

void vpm_ums_update_command_start(const struct us_data *us, struct scsi_cmnd *srb);
void vpm_ums_update_command_finish(const struct us_data *us,
				   struct scsi_cmnd *srb, int result);
void vpm_ums_update_data_start(const struct us_data *us, bool is_read);
void vpm_ums_update_data_finish(const struct us_data *us, bool is_read,
				unsigned int act_len, int result);
void vpm_ums_update_csw_start(const struct us_data *us);
void vpm_ums_update_csw_finish(const struct us_data *us, int result);

int vpm_ums_register(const struct us_data *us);
void vpm_ums_unregister(const struct us_data *us);

#endif
