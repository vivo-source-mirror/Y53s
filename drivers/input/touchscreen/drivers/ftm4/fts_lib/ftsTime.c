/*

 **************************************************************************
 **                        STMicroelectronics 						**
 **************************************************************************
 **                        marco.cali@st.com				 **
 **************************************************************************
 *                                                                        *
 *                  FTS Utility for mesuring / handling the time		 *
 *                                                                        *
 **************************************************************************
 **************************************************************************

 */

#include "ftsCrossCompile.h"
#include "ftsTime.h"

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
/*#include <linux/sec_sysfs.h>*/


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
void ftm4_startStopWatch(StopWatch *w)
{
	ktime_get_coarse_ts64(&w->start);
}

void ftm4_stopStopWatch(StopWatch *w)
{
	ktime_get_coarse_ts64(&w->end);
}
#else
void ftm4_startStopWatch(StopWatch *w)
{
	w->start = current_kernel_time();
}

void ftm4_stopStopWatch(StopWatch *w)
{
	w->end = current_kernel_time();
}
#endif

int ftm4_elapsedMillisecond(StopWatch *w)
{
	int result;

	result = ((w->end.tv_sec - w->start.tv_sec) * 1000) + (w->end.tv_nsec - w->start.tv_nsec) / 1000000;
	return result;
}

int ftm4_elapsedNanosecond(StopWatch *w)
{
	int result;

	result = ((w->end.tv_sec - w->start.tv_sec) * 1000000000) + (w->end.tv_nsec - w->start.tv_nsec);
	return result;
}

char *ftm4_timestamp(void)
{
	char *result = NULL;
	result = (char *)kmalloc((1) * sizeof(char), GFP_KERNEL);
	if (result == NULL)
		return NULL;
	result[0] = ' ';
	return result;
}

void ftm4_stdelay(unsigned long ms)
{
	 mdelay(ms);
}
