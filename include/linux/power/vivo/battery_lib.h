#ifndef __BATTERY_INTERFACE_H
#define __BATTERY_INTERFACE_H


#include <linux/iio/consumer.h>


extern int battery_lib_get_vbat(void);
extern int battery_lib_get_tbat(void);
extern int battery_lib_get_tboard(struct iio_channel *chan);
extern int battery_lib_get_parallel_tboard(struct iio_channel *chan);
extern int battery_lib_get_plug_status(void);
extern int battery_lib_get_pwrap_irq_cnt(void);


#endif/* #ifndef __BATTERY_INTERFACE_H */
