
#ifndef __VIVO_TS_PROX_H

#include <linux/i2c.h>

int vivo_ts_prox_init(struct i2c_client *client);
void vivo_ts_prox_report_values(int *xyz);
int vivo_ts_prox_ear_detect_get(void);


#endif
