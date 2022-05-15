/*
  *
  **************************************************************************
  **                        STMicroelectronics				 **
  **************************************************************************
  **                        marco.cali@st.com				 **
  **************************************************************************
  *                                                                        *
  *                  FTS Utility for measuring/handling the time	   *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsTime.h
  * \brief Contains all the definitions and structs to handle and measure the
  * time in the driver
  */

#ifndef FTS_TIME_H
#define FTS_TIME_H

#include <linux/version.h>
#include <linux/time.h>

/* TIMEOUT */
/** @defgroup timeouts	 Timeouts
  * Definitions of all the Timeout used in several operations
  * @{
  */
#define TIMEOUT_RESOLUTION			50
/* /< timeout resolution in ms (all timeout should be multiples of this unit) */
#define GENERAL_TIMEOUT				(15 * TIMEOUT_RESOLUTION)
/* /< general timeout in ms */
#define RELEASE_INFO_TIMEOUT			(2 * TIMEOUT_RESOLUTION)
/* /< timeout to request release info in ms */


#define TIMEOUT_REQU_COMP_DATA			(4 * TIMEOUT_RESOLUTION)
/* /< timeout to request compensation data in ms */
#define TIMEOUT_REQU_DATA			(8 * TIMEOUT_RESOLUTION)
/* /< timeout to request data in ms */
#define TIMEOUT_ITO_TEST_RESULT			(4 * TIMEOUT_RESOLUTION)
/* /< timeout to perform ito test in ms */
#define TIMEOUT_INITIALIZATION_TEST_RESULT	(5000 * TIMEOUT_RESOLUTION)
/* /< timeout to perform initialization test in ms */
#define TIEMOUT_ECHO 				(50 * TIMEOUT_RESOLUTION)
/* /< timeout of the echo command,*/
#define TIMEOUT_ECHO_FPI			(200  * TIMEOUT_RESOLUTION)
/* /< timeout of the Full panel Init echo command */
#define TIMEOUT_ECHO_SINGLE_ENDED_SPECIAL_AUTOTUNE \
	(100  * TIMEOUT_RESOLUTION)
/* /< timeout of the Single ended special Autotune echo command */
/** @}*/


/**
  * Struct used to measure the time elapsed between a starting and ending point.
  */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
typedef struct {
	struct timespec64 start;	/* /< store the starting time */
	struct timespec64 end;		/* /< store the finishing time */
} StopWatch;
#else
typedef struct {
	struct timespec start;	/* /< store the starting time */
	struct timespec end;	/* /< store the finishing time */
} StopWatch;
#endif

void st80y_startStopWatch(StopWatch *w);
void st80y_stopStopWatch(StopWatch *w);
int st80y_elapsedMillisecond(StopWatch *w);
int st80y_elapsedNanosecond(StopWatch *w);

#endif
