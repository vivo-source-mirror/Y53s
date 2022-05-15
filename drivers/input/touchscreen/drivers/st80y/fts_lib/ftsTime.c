/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *                  FTS Utility for mesuring/handling the time		  *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsTime.c
  * \brief Contains all functions to handle and measure the time in the driver
  */

#include "ftsTime.h"


#include <linux/errno.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
/**
  * Take the starting time and save it in a StopWatch variable
  * @param w pointer of a StopWatch struct
  */
void st80y_startStopWatch(StopWatch *w)
{
	ktime_get_coarse_ts64(&w->start);
}

/**
  * Take the stop time and save it in a StopWatch variable
  * @param w pointer of a StopWatch struct
  */
void st80y_stopStopWatch(StopWatch *w)
{
	ktime_get_coarse_ts64(&w->end);
}
#else
/**
  * Take the starting time and save it in a StopWatch variable
  * @param w pointer of a StopWatch struct
  */
void st80y_startStopWatch(StopWatch *w)
{
	w->start = current_kernel_time();
}

/**
  * Take the stop time and save it in a StopWatch variable
  * @param w pointer of a StopWatch struct
  */
void st80y_stopStopWatch(StopWatch *w)
{
	w->end = current_kernel_time();
}
#endif

/**
  * Compute the amount of time spent from when the st80y_startStopWatch and then
  * the st80y_stopStopWatch were called on the StopWatch variable
  * @param w pointer of a StopWatch struct
  * @return amount of time in ms (the return value is meaningless
  * if the st80y_startStopWatch and st80y_stopStopWatch were not called before)
  */
int st80y_elapsedMillisecond(StopWatch *w)
{
	int result;

	result = ((w->end.tv_sec - w->start.tv_sec) * 1000) + (w->end.tv_nsec -
							       w->start.tv_nsec)
		 / 1000000;
	return result;
}

/**
  * Compute the amount of time spent from when the st80y_startStopWatch and
  * then the st80y_stopStopWatch were called on the StopWatch variable
  * @param w pointer of a StopWatch struct
  * @return amount of time in ns (the return value is meaningless
  * if the st80y_startStopWatch and st80y_stopStopWatch were not called before)
  */
int st80y_elapsedNanosecond(StopWatch *w)
{
	int result;

	result = ((w->end.tv_sec - w->start.tv_sec) * 1000000000) +
		 (w->end.tv_nsec - w->start.tv_nsec);
	return result;
}
