/*

 **************************************************************************
 **                        STMicroelectronics 		                **
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

#include <linux/time.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
typedef struct {
	struct timespec64 start, end;
} StopWatch;
#else
typedef struct {
	struct timespec start, end;
} StopWatch;
#endif

void ftm4_startStopWatch(StopWatch *w);
void ftm4_stopStopWatch(StopWatch *w);
int ftm4_elapsedMillisecond(StopWatch *w);
int ftm4_elapsedNanosecond(StopWatch *w);
char *ftm4_timestamp(void);
void ftm4_stdelay(unsigned long ms);
