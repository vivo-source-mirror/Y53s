#ifndef _RSC_ZRAM_MODULE_H_
#define _RSC_ZRAM_MODULE_H_

#include <linux/mutex.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/swap.h>

#ifndef RSC_ZRAM_FAST_LOAD_CHECK
enum {
	IDX_CPU_USER = 0,
	IDX_CPU_SYSTEM,
	IDX_CPU_IDLE,

	IDX_CPU_MAX
};
#endif
struct rsc_zram_module {
	/* protect configs from other threads,notifications. */
	struct rw_semaphore	lock;
	/* wakeup/wait for working thread. */
	struct semaphore	wait;
	/* sysfs node */
	struct kobject		*kobj;
	/* kthread handler, this also is enable status flag. */
	struct task_struct	*task;
	/* only action on received event, remove periodically action. */
	int                 period_mode;
	/* display is off. */
	bool				display_off;
	/* flag to indicate that we will do a full clean. */
	atomic_t			full_clean_flag;
#ifndef RSC_ZRAM_FAST_LOAD_CHECK
	/* old cpu stat, for calculate cpu idle. */
	clock_t				last_cpu_stat[IDX_CPU_MAX];
	/* cpu load in percent. */
	int 				cpu_load[3];
	/* cpu load threshld in percent for compress action.*/
	int					idle_threshold;
#endif
	/*  last time to get cpu stat. */
	unsigned int		last_cpu_msec;
	/* max usage of swap area, in percent. */
	int					swap_percent_low;
	/* min free pages to keep in our system. */
	int					free_pages_min;
	/* max file pages to be cleaned in full clean process. */
	int					full_clean_file_pages;
	/* min anon pages kept uncompress. */
	int					anon_pages_min;
	/* max anon pages kept uncompress. */
	int					anon_pages_max;
	/* max anon pages to be cleaned in full clean process. */
	int					full_clean_anon_pages;
	/* counter for recording wakeup times. */
	unsigned int		wakeup_count;
	/* counter for total full clean pages. */
	unsigned int		nr_full_clean_pages;
	/* counter for total normal clean pages. */
	unsigned int		nr_normal_clean_pages;
	/* jiffies counter for all swap time. */
	unsigned int		total_spent_times;
	unsigned int		fullclean_count;
	/*ms, neg value is infinity wait!*/
	int					complete_time;
	u8 					notfirst_run;
	u8					force_period;
	u8					task_exit;
	u8					task_out;
};

/* common fail, no special reason. */
#define EFAIL					1

/* page to KB,MB */
#define K(x) ((x) << (PAGE_SHIFT - 10))
#define M(x) ((x) >> (20 - PAGE_SHIFT))

#define RSC_ZRAM_MAX_SWAP_SIZE			((1024*1024*1024)>>(PAGE_SHIFT))

/*msm8953, 8937 use 2GB memory, kernel version is 3.18.71*/
#if LINUX_VERSION_CODE == KERNEL_VERSION(3, 18, 71)
/*
threshhold time for idle stat judgement. in percent.
4cores is 20, 8cores is 11
*/
#define RSC_ZRAM_IDLE_THRESHOLD		20

/* swap full percent for stop compress thread. in percent.
4GB ram 80, 2GB RAM 85
*/
#define RSC_ZRAM_SWAP_PERCENT_LOW		85
/* stop free process until +RSC_ZRAM_SWAP_PERCENT_LOW_EX */
#define RSC_ZRAM_SWAP_PERCENT_LOW_EX	10

/* min free memory size to be kept in out system. in page count. */
/*#define RSC_ZRAM_FREE_PAGE_MIN		((320*1024*1024)>>(PAGE_SHIFT))*/
/* stop free process until RSC_ZRAM_FREE_PAGE_MIN+RSC_ZRAM_FREE_PAGE_MIN_EX */
#define RSC_ZRAM_FREE_PAGE_MIN_EX		((48*1024*1024)>>(PAGE_SHIFT))

/* keep uncompressed anon pages at least RSC_ZRAM_ANON_PAGE_MIN.
4GB ram 160MB, 2GB RAM 120MB
*/
#define RSC_ZRAM_ANON_PAGE_MIN		((120*1024*1024)>>(PAGE_SHIFT))
/* when anon pages larger than RSC_ZRAM_ANON_PAGE_MAX, start compress.
4GB ram 320MB, 2GB RAM 260MB
*/
#define RSC_ZRAM_ANON_PAGE_MAX		((260*1024*1024)>>(PAGE_SHIFT))

/* full clean file pages. */
#define RSC_ZRAM_FULL_CLEAN_FILE_PAGE	((16*1024*1024)>>(PAGE_SHIFT))

/* swap page size once */
#define RSC_ZRAM_NR_SWAP_UNIT_SIZE		((1*1024*1024)>>(PAGE_SHIFT))

/* max reclaim page size on boot complete
4G RAM is 320MB, 2GB RAM is 380
*/
#define RSC_ZRAM_MAX_RECLAIM_ON_BOOT	((380*1024*1024)>>(PAGE_SHIFT))
/*
for 8937. do echo 1 > /proc/sys/vm/rsc_zram_enable in init.qcom.post_boot.sh is about 27s.
*/
#define RSC_ZRAM_WAIT_COMPLETE		(60*1000) /*60s*/
#else
/*
threshhold time for idle stat judgement. in percent.
4cores is 20, 8cores is 11 except sdm845 that is 5
*/
#ifdef CONFIG_ARCH_SDM845
#define RSC_ZRAM_IDLE_THRESHOLD		5
#else
#define RSC_ZRAM_IDLE_THRESHOLD		11
#endif

/* swap full percent for stop compress thread. in percent.
4GB ram 80, 2GB RAM 85
*/
#define RSC_ZRAM_SWAP_PERCENT_LOW		80
/* stop free process until +RSC_ZRAM_SWAP_PERCENT_LOW_EX */
#define RSC_ZRAM_SWAP_PERCENT_LOW_EX	10

/* min free memory size to be kept in out system. in page count. */
/*#define RSC_ZRAM_FREE_PAGE_MIN		((320*1024*1024)>>(PAGE_SHIFT))*/
/* stop free process until RSC_ZRAM_FREE_PAGE_MIN+RSC_ZRAM_FREE_PAGE_MIN_EX */
#define RSC_ZRAM_FREE_PAGE_MIN_EX		((48*1024*1024)>>(PAGE_SHIFT))

/* keep uncompressed anon pages at least RSC_ZRAM_ANON_PAGE_MIN.
4GB ram 160MB, 2GB RAM 120MB
*/
#define RSC_ZRAM_ANON_PAGE_MIN		((160*1024*1024)>>(PAGE_SHIFT))
/* when anon pages larger than RSC_ZRAM_ANON_PAGE_MAX, start compress.
4GB ram 320MB, 2GB RAM 260MB
*/
#define RSC_ZRAM_ANON_PAGE_MAX		((320*1024*1024)>>(PAGE_SHIFT))

/* full clean file pages. */
#define RSC_ZRAM_FULL_CLEAN_FILE_PAGE	((16*1024*1024)>>(PAGE_SHIFT))

/* swap page size once */
#define RSC_ZRAM_NR_SWAP_UNIT_SIZE		((1*1024*1024)>>(PAGE_SHIFT))

/* max reclaim page size on boot complete
4G RAM is 320MB, 2GB RAM is 380
*/
#define RSC_ZRAM_MAX_RECLAIM_ON_BOOT	((320*1024*1024)>>(PAGE_SHIFT))

#define RSC_ZRAM_WAIT_COMPLETE		(90*1000) /*90s*/
#endif

#define RSC_ZRAM_IDLE_FAST			20

#define WS_NEED_WAKEUP			0
#define WF_NOT_ENABLED			0x01
#define WF_CPU_BUSY				0x02
#define WF_MEM_FREE_ENOUGH		0x04
#define WF_SWAP_FULL			0x08
#define WF_NO_ANON_PAGE			0x10

#define RSC_ZRAM_WAIT_INFINITE		-1

#endif
