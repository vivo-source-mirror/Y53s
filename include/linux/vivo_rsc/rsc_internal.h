/*
 * kernel/rsc/rsc_internal.h
 *
 * VIVO Resource Control.
 *
 */

#ifndef __RSC_INTERNAL_H__
#define __RSC_INTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/list.h>
#include <linux/uidgid.h>
#include <linux/fs.h>
#include <linux/vivo_rsc_ioctl.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#include <linux/sched/cputime.h>
#endif
#include <linux/cpuset.h>

/* operation */
#define MAX(a, b)		((a) >= (b) ? (a) : (b))
#define MIN(a, b)		((a) >= (b) ? (b) : (a))

/* LOCK */
#define rsc_lock(lock)		mutex_lock(lock)
#define rsc_unlock(lock)	mutex_unlock(lock)


struct rsc_global_attr {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *a, struct attribute *b,
					const char *c, size_t count);
};

#ifndef define_rsc_one_global_ro
#define define_rsc_one_global_ro(_name)		\
static struct rsc_global_attr _name =		\
__ATTR(_name, 0440, show_##_name, NULL)
#endif

#ifndef define_rsc_one_global_rw
#define define_rsc_one_global_rw(_name)		\
static struct rsc_global_attr _name =		\
__ATTR(_name, 0640, show_##_name, store_##_name)
#endif


/* LOG */
#undef TAG
#define TAG     "^RSC "

#define rsc_err(fmt, args...)		\
	pr_err(TAG"[ERROR]"fmt, ##args)
#define rsc_warn(fmt, args...)		\
	pr_warn(TAG"[WARNING]"fmt, ##args)
#define rsc_info(fmt, args...)		\
	pr_warn(TAG""fmt, ##args)
#define rsc_dbg(type, fmt, args...)				\
	do {											\
		if (rsc_debug & type)	\
			rsc_info(fmt, ##args);					\
	} while (0)
#define rsc_ver(fmt, args...)			\
	do {					\
		if (rsc_debug == ALL)		\
			rsc_info(fmt, ##args);	\
	} while (0)
#define rsc_cont(fmt, args...)		\
	pr_cont(fmt, ##args)

#define rsc_main(fmt, args...)			\
	do {					\
		if (rsc_debug & MAIN)		\
			rsc_info(fmt, ##args);	\
	} while (0)

/* default value is print*/
#define rsc_cpucap(fmt, args...)			\
			do {					\
				if (!(rsc_debug & CPU_CAPINF_OFF))		\
					rsc_info(fmt, ##args);	\
			} while (0)

#define FUNC_LV_MODULE		BIT(0)	/* module, platform driver interface */
#define FUNC_LV_API			BIT(1)	/* mt_rsc driver global function */
#define FUNC_LV_MAIN		BIT(2)	/* mt_rsc driver main function */
#define FUNC_LV_HICA		BIT(3)	/* mt_rsc driver HICA related function */
#define FUNC_LV_POLICY		BIT(4)	/* mt_rsc driver other policy function */

#define FUNC_ENTER(lv)	\
	do { if ((lv) & rsc_func_lv_mask) rsc_info(">> %s()\n", __func__); } while (0)
#define FUNC_EXIT(lv)	\
	do { if ((lv) & rsc_func_lv_mask) rsc_info("<< %s():%d\n", __func__, __LINE__); } while (0)

/*==============================================================*/
/* Enum								*/
/*==============================================================*/
enum {
	NO_LOG	= 0,
	ALL		= 1 << 0,/*1*/
	MAIN	= 1 << 1,/*2*/
	PROFILE = 1 << 2,/*4*/
	CPU_CAP = 1 << 3,/*8*/
	CPU_CAPINF_OFF = 1 << 4,/*0x10, 16 , 1 means off*/
	CPU_TASK_LATENCY = 1 << 5,/*0x20, 32*/
	CPU_TASK_USAGE = 1 << 6,/*0x40, 64*/
	UID_IO = 1 << 7,/*0x80, 128*/
	UID_LMK = 1 << 8,/*0x100, 256*/
	CPU_TOP = 1 << 9,/*0x200, 512*/
	IOLIMIT = 1 << 10,/*0x400, 1024*/
	RSC_ZRAM = 1 << 11,/*0x800, 2048*/
	RSC_BINDER = 1 << 12,/*0x1000, 4096*/
	RSC_BOOST_TASKKILL = 1 << 13,/*0x2000, 8192*/
#ifdef CONFIG_RSC_PFREFETCH_PAGE_FAULT
	/*only show uid >= 10000,
	* otherwise, enable show all,
	* echo 1 > /sys/module/rsc_mem_mon/parameters/show_all_pre_pagefault
	*/
	RSC_PREFETCH_PAGEFAULT = 1 << 14,/*0x4000, 16384*/
#endif
#ifdef CONFIG_RSC_APP_LAUNCH_BOOST
	RSC_APP_LAUNCH_BOOST = 1 << 15,/*0x8000, 32768*/
#endif
#ifdef CONFIG_RSC_NEW_FIX_SPF_DEADLOCK
	RSC_FIX_SPF = 1 << 16,/*0x10000, 65536*/
#endif
#ifdef CONFIG_RSC_VAUDIT
	RSC_VAUDIT = 1 << 17,/*0x20000, 131072*/
#endif
	RSC_SCHED = 1 << 18,/* 0x40000*/
#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
	RSC_FIX_DEADLOCK = 1 << 19,/* 0x80000*/
#endif
#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
	RSC_SVP_TASK_SCHEDULE = 1 << 21,/* 0x200000*/
#endif
#ifdef CONFIG_RSC_ION_OPT
	RSC_ION_OPT = 1 << 22,/* 0x400000*/
#endif
#ifdef CONFIG_RSC_KSWAPD_IMPROVE
	RSC_VSWAPD = 1 << 23,/* 0x800000*/
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
	RSC_LOCK_BOOST_FUTEX = 1 << 24,/* 0x1000000*/
#endif
	RSC_LOCK_BOOST_FUTEX_TRACK = 1 << 25,/* 0x2000000*/
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
	RSC_LOCK_BOOST_RECORD_BLOCK_ADDR = 1 << 26,/* 0x4000000*/
#endif
};

enum rsc_type {
	RSC_TYPE_CPU_LEVEL = 0,
	RSC_TYPE_CPU_INTERNAL,
	RSC_TYPE_GPU_LEVEL,
	RSC_TYPE_MEM_LEVEL,
	RSC_TYPE_BUS_LEVEL,
	RSC_TYPE_IO_LEVEL,
	RSC_TYPE_SCHED_LEVEL,
	RSC_TYPE_IOSCHED_LEVEL,
	RSC_TYPE_VM_OPTS_LEVEL,
	RSC_TYPE_LOW_MEM_KILLER_LEVEL,
	RSC_TYPE_MAX,
};

enum rsc_mode {
	RSC_MODE_LOW_POWER = 0,
	RSC_MODE_NORMAL,
	RSC_MODE_PERFORMANCE,
};

enum rsc_plat_mode {
	RSC_PLAT_MTK = 0,
	RSC_PLAT_SAMSUNG,
	RSC_PLAT_QUALCOMM,
};

enum power_state_search_policy {
	PERFORMANCE = 0,
	LOW_POWER,
};

#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
/*
 * Debugging: various feature bits
 */

#define RSC_FEAT(name, enabled)														\
/*	(																				\
		do {	(*/ 																\
	RSC_FEAT_##name,																\
/*		) } while (0)																\
	) */

enum {
#include <linux/vivo_rsc/rsc_features.h>
	RSC_FEAT_NR,
	RSC_SWITCH_CLEAR = 31,
};

#define rsc_feat(x) (sysctl_rsc_switch & (1UL << RSC_FEAT_##x))
#define rsc_feats(x, y) (sysctl_rsc_switch & ((1UL << RSC_FEAT_##x) | (1UL << RSC_FEAT_##y)))
#define rsc_val_feat(val, x) (val & (1UL << RSC_FEAT_##x))
#define rsc_val_feats(val, x, y) (val & ((1UL << RSC_FEAT_##x) | (1UL << RSC_FEAT_##y)))
#endif

#define CPU_PERD_FREQ_ID 		89889966557766
#define CPU_POWER_FREQ_ID 		89888568958888

#define CPU_INTERNAL_FREQ_ID 	89155075917309

#define CPU_LEVEL_FREQ_ID 		89168689977309

#define RSC_END_PID 0xffffffff

#define RSC_UID_IO_EXIT 	(1 << 0)
#define RSC_CPU_TOP_EXIT 	(1 << 1)

/*==============================================================*/
/* Data Structures*/
/*==============================================================*/

#define MAX_VAL_NUM 4

struct rsc_qos_udate {
	int val;
	int timeout;/*ms*/
	u64 settime;
};

struct rsc_qos_val {
	struct rsc_qos_udate data;
	bool is_new_add;
};

struct rsc_upper_req {
	struct list_head node;

	enum rsc_type types;

	/* hash of package name */
	u64 id;

	/* hash of package name */
	char *name;

	/* virtual level */
	int minlevel;
	int maxlevel;

	/* -1 means no time limit */
	long timeout;

	int v_num;
	int req_num;
	unsigned int req_bitmap;
	struct rsc_qos_udate *pval;

};

struct rsc_pm_qos_req {
	/* hash of package name */
	u64 id;

	/* hash of package name */
	char *name;
	/* virtual level */
	int minlevel;
	int maxlevel;
	/* -1 means no time limit */
	long timeout;
	/* request to pm qos module */
	struct pm_qos_request rsc_level_min;
	struct pm_qos_request rsc_level_max;
	int v_num;
	int req_num;
	unsigned int req_bitmap;
	struct rsc_qos_val *pval;
	struct pm_qos_request *rsc_qos_req;
	bool is_new_add;
	/* lock */
	struct mutex lock;
	/* for profile */
	unsigned long last_atime;
	unsigned long counts;

	/* link to rsc_policy_data.pm_qos_req_head */
	struct list_head link;
};

struct rsc_policy_data {
	/* settings */
	const char *name;
	enum rsc_type types;

	/* status */
	bool is_enabled;
	/* lock */
	struct mutex lock;
	/* link to rsc_data.sub_module_list*/
	struct list_head link;
	/* pm qos list header */
	struct list_head pm_qos_req_head;

	/* callbacks */
	void (*update_cb)(struct rsc_pm_qos_req *req);
	void (*mode_change_cb)(enum rsc_mode mode);
};

struct rsc_data {
	/* status */
	enum rsc_mode cur_mode;
	enum rsc_plat_mode cur_plat;	/* mtk/samsung/qualcomm */

	bool is_enabled;
	bool is_in_suspend;

	/* platform dev/driver */
	const struct dev_pm_ops rsc_pm_ops;
	struct platform_device rsc_pdev;
	struct platform_driver rsc_pdrv;

	/* rsc core data */
	struct mutex lock;
	struct list_head sub_module_list;
};

#define RSC_MAX_ID_SIZE	16
#define RSC_ID_PREFIX "RSC_ID_"
struct rc_id_map_t {
	u32 id;
	const char *name;
};

static const struct rc_id_map_t const rsc_ids[RSC_ID_END - RSC_ID_START] = {
	{RSC_ID_PERFD, RSC_ID(RSC_ID_PERFD)},
	{RSC_ID_BBKLOG, RSC_ID(RSC_ID_BBKLOG)},
	{RSC_ID_POWER, RSC_ID(RSC_ID_POWER)},
	{RSC_ID_VIVOD, RSC_ID(RSC_ID_VIVOD)},
	{RSC_ID_BIGDATA, RSC_ID(RSC_ID_BIGDATA)},
	{RSC_ID_SHELL, RSC_ID(RSC_ID_SHELL)},
	{RSC_ID_OTHER, RSC_ID(RSC_ID_OTHER)}
};

/*==============================================================*/
/* Global variables						*/
/*==============================================================*/
extern struct rsc_data rsc_main_info;
extern struct kobject *rsc_root_dir;
extern struct proc_dir_entry *vivo_rsc;

extern unsigned int rsc_func_lv_mask;
extern unsigned int __read_mostly rsc_debug;

/*==============================================================*/
/* APIs								*/
/*==============================================================*/
/* procfs */
extern int rsc_sys_api_init(void);

/* main */
extern int rsc_main_register_sub_module(struct rsc_policy_data *policy);
extern void rsc_main_unregister_sub_module(struct rsc_policy_data *policy);
extern int rsc_main_update(struct rsc_upper_req *up, struct rsc_policy_data *policy);

/* profiling */
extern int rsc_profile_init(void);
extern void rsc_profile_exit(void);
extern int rsc_cpu_initialized;
extern int update_rsc_cpu_freq(u64 id, struct rsc_qos_udate *cluster_freq, int input_num, unsigned int req_bitmap);
extern int rsc_chown_to_system(struct kobject *kobj, const struct attribute *attr);
#if defined(CONFIG_RSC_UID_IO) && defined(CONFIG_RSC_CPU_TOP)
int rsc_top_process_notifier(struct notifier_block *self,
			unsigned long cmd, void *v);
#endif

#define SYSTEM_ID KUIDT_INIT(1000)
#define SYSTEM_GROUP_ID KGIDT_INIT(1000)

#ifdef CONFIG_RSC_ZRAM
/*use timer tick to check load, performance is very good!*/
#define RSC_ZRAM_FAST_LOAD_CHECK
/*
not need to enable/disable rsc_zram thread,
Fix me: need to consider race conditon, if open this macro
*/
#define RSC_ZRAM_THREAD_SWITCH

#define RSC_ZRAM_MODE_ANON   1
#define RSC_ZRAM_MODE_FILE   2

/* threshhold time for idle stat judgement. in ms.*/
#define RSC_ZRAM_IDLE_SLOW			2000
/*low ram project need adjust RSC_ZRAM_FREE_PAGE_MIN*/
#if 1
/*2GB ram is 130MB, 4GB ram is 320MB*/
extern unsigned long RSC_ZRAM_FREE_PAGE_MIN;
#else
#define RSC_ZRAM_FREE_PAGE_MIN		((160*1024*1024)>>(PAGE_SHIFT))
#endif

#define RSC_MAX_SWAP_PAGE_ERR 20

int rsc_try_to_free_pages(int nr_pages, u16 mode);
int rsc_zram_enable_handler(struct ctl_table *table, int write,
	void __user *buffer, size_t *length, loff_t *ppos);

#ifdef RSC_ZRAM_FAST_LOAD_CHECK
#define RSC_ZRAM_IDLE_CHECK_INTERNAL 200
#define RSC_ZRAM_LOAD_NUM	2
#if defined(CONFIG_MTK_PLATFORM)
#define RSC_ZRAM_TICK_CNT_CONST (RSC_ZRAM_IDLE_CHECK_INTERNAL*CONFIG_HZ/1000)
extern u32 __read_mostly RSC_ZRAM_TICK_CNT[RSC_ZRAM_LOAD_NUM];
#else
#define RSC_ZRAM_TICK_CNT (RSC_ZRAM_IDLE_CHECK_INTERNAL*CONFIG_HZ/1000)
#endif

#define RSC_ZRAM_LOAD_MASK	(RSC_ZRAM_LOAD_NUM-1)
#define RSC_ZRAM_WAIT_CON_PERIOD 0x02
#define RSC_ZRAM_WAIT_CON_ACTION 0x01
#define RSC_ZRAM_TICK_PERIOD_CNT (RSC_ZRAM_IDLE_SLOW/RSC_ZRAM_IDLE_CHECK_INTERNAL)

extern int __read_mostly rsc_zram_enable;
/*ms*/
extern atomic_t rsc_zram_load;
extern atomic_t rsc_zram_load_val[RSC_ZRAM_LOAD_NUM];
extern atomic_t rsc_zram_period_cnt;
extern int rsc_zram_idle_threshold;
extern int rsc_zram_idle_load;
extern wait_queue_head_t rsc_zram_wait;
extern atomic_t rsc_zram_wait_condition;
extern long rsc_zram_swap_min_free;
extern int rsc_zram_anon_pages_max;

void inc_zone_page_state(struct page *page, enum zone_stat_item item);
void dec_zone_page_state(struct page *page, enum zone_stat_item item);
#endif
#if defined(CONFIG_MTK_PLATFORM)
extern seqcount_t rsc_zram_load_seq_lock;
#endif
#endif

#ifdef CONFIG_RSC_MEM_DEFRAG
#ifndef SZ_1M
#define SZ_1M				0x00100000
#endif
/*#define CONFIG_RSC_ISOLATE_SORD_SIZE_MB 256*/
#define RSC_ISOLATE_SORD_SIZE_BLOCKS \
	(256*SZ_1M/PAGE_SIZE/pageblock_nr_pages)

/*#define CONFIG_RSC_ISOLATE_BORD_SIZE_MB 128*/
#define RSC_ISOLATE_BORD_SIZE_BLOCKS \
	(128*SZ_1M/PAGE_SIZE/pageblock_nr_pages)

#define RSC_UNMOVABLE_ISOLATE_BORD_STEP_BLOCKS \
	(20*SZ_1M/PAGE_SIZE/pageblock_nr_pages)


#define UNMOVABLE_ISOLATE_SORD_MIN_ORDER 0
#define UNMOVABLE_ISOLATE_SORD_MAX_ORDER 0

#define UNMOVABLE_ISOLATE_BORD_MIN_ORDER 2
#define UNMOVABLE_ISOLATE_BORD_MAX_ORDER 3
#if 0
#define use_unmovable_isolate_ord(zoneidx, type, order)		\
	((zoneidx == ZONE_NORMAL) &&							\
	(type == MIGRATE_UNMOVABLE) &&							\
	(order >= UNMOVABLE_ISOLATE_BORD_MIN_ORDER &&			\
	order <= UNMOVABLE_ISOLATE_BORD_MAX_ORDER))
#else
#define use_unmovable_isolate_ord(zoneidx, type, order)		\
	(type == MIGRATE_UNMOVABLE)
#endif

extern int rsc_mem_antifrag_disabled;
extern u32 mem_smallord_MB, mem_bigord_MB;
/*
 * check if the zone is DMA.
 */
static inline int is_dma_zone(struct zone *zone)
{
#ifdef CONFIG_ZONE_DMA
	if (zone != NULL && zone_idx(zone) == ZONE_DMA)
#else
	if (zone != NULL && zone_idx(zone) == ZONE_NORMAL)
#endif
		return 1;
	else
		return 0;
}

/*
 * check if the order is valid for unmovable_isolate area.
 */
static inline int rsc_valid_isolate_order(int order, int migratetype)
{
	if (is_unmovable_isolate_sord(migratetype) &&
		(order >= UNMOVABLE_ISOLATE_SORD_MIN_ORDER &&
		order <= UNMOVABLE_ISOLATE_SORD_MAX_ORDER))
		return 1;
	else if (is_unmovable_isolate_bord(migratetype) &&
		(order >= UNMOVABLE_ISOLATE_BORD_MIN_ORDER &&
		order <= UNMOVABLE_ISOLATE_BORD_MAX_ORDER))
		return 1;
	return 0;
}

/*
 * check if the unmovable_isolate_enabled is enabled.
 * We only enable ui function in DMA zone and rsc_mem_antifrag_disabled is 0.
 */
static inline int unmovable_isolate_enabled (struct zone *zone)
{
	if (!rsc_mem_antifrag_disabled && is_dma_zone(zone))
		return 1;
	else
		return 0;
}

extern int unmovable_isolate_pageblock(struct zone *zone, struct page *page);
extern int rsc_setup_zone_isolate_page(struct zone *zone,
     int unmovable_isolate_type, int disable);

#define RSC_SKIP_MEM_DEFRAG_IN_RECOVER_MODE
#ifdef RSC_SKIP_MEM_DEFRAG_IN_RECOVER_MODE
extern int rsc_recovery_survival_mode;
#endif
#endif

#define task_rsc_alloc(task)		(task_cred_xxx((task), rsc_alloc))

#ifdef CONFIG_RSC_BINDER_OPTIMIZATION
#define RSC_SET_CPUS_MAX 0x0f
#define RSC_SET_CPUS_HIGH_MASK (~RSC_SET_CPUS_MAX)

#define RSC_SET_CPUS_PRINT 0x10
enum {
	RSC_SET_CPUS_NOCHANGE  				= 0,
	RSC_SET_CPUS_SYS_CHANGE  			= 1,
	RSC_SET_CPUS_EQUAL 					= 2,
	RSC_SET_CPUS_RACE_CHANGE			= 3,
	RSC_SET_CPUS_MASK_ERR     			= 4,
	RSC_SET_CPUS_BINDER_LIMIT_CHANGE  	= 5,
	RSC_SET_CPUS_OTHER_CHANGE  			= 6,
	RSC_SET_CPUS_RESTORE  				= 7,
	RSC_SET_CPUS_SUCCESS				= RSC_SET_CPUS_MAX,
 };

extern struct rsc_alloc_stat_t rsc_system_server_time;
extern struct cpumask rsc_cpu_littlecore;
extern struct cpumask rsc_cpu_all;
extern unsigned int __read_mostly sysctl_rsc_switch;
#ifndef RSC_USE_FAST_CPUSET_LOCK
extern int rsc_set_cpus_allowed_ptr(struct task_struct *p,
				  struct cpumask *set_mask, int migration);
#endif
#endif

#ifdef CONFIG_RSC_BOOST_TASKKILL
extern void boost_free_user_mem(void);
#endif

#ifdef CONFIG_RSC_PFREFETCH_PAGE_FAULT
extern int rsc_show_all_pre_pagefault;
#endif

#if defined(CONFIG_RSC_FIX_CPU_HOTPLUG_FAIL) && defined(CONFIG_RSC_V2_CPU_CAP_NOTIFY)
struct rsc_cpu_hotplug_info {
	__kernel_time_t down_time;
	__kernel_time_t up_time;
	char down_comm[TASK_COMM_LEN];
	pid_t down_pid;
	char down_pcomm[TASK_COMM_LEN];
	pid_t down_ppid;
};
#endif

#if defined(CONFIG_RSC_THREAD_BOOST)
enum {
	ENMU_BOOST_THREAD_BOOST = 0,
	ENMU_BOOST_PROCESS_BOOST,
	ENMU_BOOST_LPM_DISABLE,
	ENMU_COLOCATION_SYNC,
	ENMU_RT_BOOST_BIG,
	ENMU_RT_BOOST_BIG_FORCE,
	ENMU_RT_SCHED_BOOST_BIG_FORCE,
	ENMU_RT_SCHED_BOOST_BIG,
	ENMU_BOOST_MAX_VALUE,
};
/* task_struct rsc_boost status */
#define RSC_BOOST_NONE					0
#define RSC_BOOST_THREAD_BOOST			(1 << ENMU_BOOST_THREAD_BOOST) /*0x1*/
#define RSC_BOOST_PROCESS_BOOST			(1 << ENMU_BOOST_PROCESS_BOOST) /*0x2*/
#define RSC_BOOST_LPM_DISABLE			(1 << ENMU_BOOST_LPM_DISABLE) /*0x4*/
#define RSC_COLOCATION_SYNC				(1 << ENMU_COLOCATION_SYNC) /*0x8*/
#define RSC_RT_BOOST_BIG				(1 << ENMU_RT_BOOST_BIG) /*0x10*/
#define RSC_RT_BOOST_BIG_FORCE			(1 << ENMU_RT_BOOST_BIG_FORCE) /*0x20*/
#define RSC_RT_SCHED_BOOST_BIG_FORCE	(1 << ENMU_RT_SCHED_BOOST_BIG_FORCE) /*0x40*/
#define RSC_RT_SCHED_BOOST_BIG			(1 << ENMU_RT_SCHED_BOOST_BIG) /*0x80*/
#define RSC_BOOST_MAX_VALUE 			((1 << ENMU_BOOST_MAX_VALUE) - 1) /*2^n -1, n = 8*/

enum {
	ENMU_TOP_APP_SCHED_BOOST = 0,
	ENMU_RT_SCHED_BOOST,
	ENMU_SCHED_CTRL_MAX_VALUE,
};

/*0x1, when CONSERVATIVE_BOOST enable, only boost top-app to big cores, except foreground tasks.*/
#define RSC_TOP_APP_SCHED_BOOST			(1 << ENMU_TOP_APP_SCHED_BOOST)
/*0x2, boost rt task to big cores*/
#define RSC_RT_SCHED_BOOST				(1 << ENMU_RT_SCHED_BOOST)
#define RSC_SCHED_CTRL_MAX_VALUE 		((1 << ENMU_SCHED_CTRL_MAX_VALUE) - 1) /*2^n -1, n = 2*/

extern int rsc_sched_boost_ctrl;
static inline int get_rsc_sched_boost_ctrl(void)
{
	return rsc_sched_boost_ctrl;
}
#endif

#if defined(CONFIG_RSC_SHOW_GPUCLK) && defined(CONFIG_RSC_SHOW_GPUCLK_PROJECT)
extern int gpuclk_enable;
static inline int is_gpuclk_enable(void)
{
	return gpuclk_enable;
}
#endif

#if defined(CONFIG_RSC_LPM_MANAGER)
extern atomic_t is_rsc_lpm_boost;
static inline int is_lpm_boost(void)
{
	return atomic_read(&is_rsc_lpm_boost);
}
#endif
#ifdef CONFIG_RSC_COLOCATION_SYNC
extern int sync_cgroup_colocation(struct task_struct *p, bool insert);
extern void set_rsc_boost_without_entry(struct task_struct *p, int value);
extern void clear_rsc_boost_without_entry(struct task_struct *p, int value);
extern int rsc_colo_sync_enable;
static inline bool is_rsc_colo_sync_enable(void)
{
	return rsc_colo_sync_enable;
}
#endif

#if defined(CONFIG_RSC_MEM_MON) && defined(CONFIG_RSC_NEW_FIX_SPF_DEADLOCK)
	extern int rsc_disable_old_fix_spf_deadlock;
#endif

#ifdef CONFIG_RSC_VAUDIT

/*
 * Debugging: various feature bits
 */

#define VAUDIT_FEAT(name, enabled)														\
/*	(																				\
		do {	(*/ 																\
	VAUDIT_FEAT_##name,																\
/*		) } while (0)																\
	) */

enum {
#include <linux/vivo_rsc/vaudit_features.h>
	VAUDIT_FEAT_NR,
	VAUDIT_SET_COSTTIME = 30,
	VAUDIT_SWITCH_CLEAR = 31,
};

#define vaudit_feat(x) (rsc_vaudit_switch & (1UL << VAUDIT_FEAT_##x))

typedef struct rsc_sysent {
	const char *sys_name;
	char nargs;
	u16 count;
} struct_rsc_sysent;

extern unsigned int rsc_vaudit_switch;
extern u32 __read_mostly rsc_syscall_threshold_time;
extern struct_rsc_sysent sysent_32[];
extern struct_rsc_sysent sysent_64_1[];
extern struct_rsc_sysent sysent_64_2[];
#define RSC_SYSCALL64_2_TABLE_BASE 1024
extern u32 rsc_vaudit_num;
extern u32 rsc_vaudit_failnum;
int rsc_syscall_check(char *buf, int ret);
extern unsigned long rsc_vaudit_backtrace_show_count;
extern unsigned long rsc_vaudit_iowait_over_count;
#endif

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE) || defined(CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK)
#define RSC_MAX_PACKAGE_NAME			48
#endif

#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE
extern int svp_min_sched_delay_granularity;
extern int svp_min_migration_delay;
extern atomic_t rsc_svp_task_cnt;
extern int rsc_svp_task_uid;
extern int rsc_bigcore_num;
extern u32 rsc_debug_enqueue_overcnt;
extern u32 rsc_debug_enqueue_thread_overcnt;
extern u32 rsc_debug_enqueue_bind_bigcore_cnt;
extern u64 svp_enqueue_overtime;
extern u64 heavytask_limit_maxtime;

#define RSC_LIMIT_PROCESS_MODE_TO_LITCORE 			1
#define RSC_LIMIT_PROCESS_MODE_SCHED_AFFINITY 		2
#define RSC_LIMIT_PROCESS_MODE_TO_LITCORE_SWITCH 	3
#define RSC_LIMIT_PROCESS_MODE_SCHED_AFFINITY_SWITCH 4

#define RSC_HEAVYTASK_LIMIT_MAX_PROCESS 8
#define RSC_SET_AFFINITY_LIMIT_MAX_PROCESS 8

extern int HEAVYTASK_LIMIT_SVP_MIN_CNT;
#define HEAVYTASK_LIMIT_TO_LITCORE_NONE					0
#define HEAVYTASK_LIMIT_TO_LITCORE_SLIM					0x1
#define HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_ALLTASK		0x2
#define HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_THREADTASK	0x4
#define HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_ALLTASK		0x8
#define HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_THREADTASK	0x10/*16*/
#define HEAVYTASK_LIMIT_TO_LITCORE_NONE_DO_MIGRATE_ONLY	0x20/*32*/

#define HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP			0xff

#define  HEAVYTASK_LIMIT_TO_LITCORE_SVP (HEAVYTASK_LIMIT_TO_LITCORE_SLIM|HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_ALLTASK|			\
			HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_THREADTASK|HEAVYTASK_LIMIT_TO_LITCORE_NONE_DO_MIGRATE_ONLY)
#define HEAVYTASK_LIMIT_TO_LITCORE_HOK (HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_ALLTASK|HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_THREADTASK)

#define HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT (HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_ALLTASK|HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_ALLTASK)
#define HEAVYTASK_LIMIT_TO_LITCORE_THREADTASK_LIMIT (HEAVYTASK_LIMIT_TO_LITCORE_SVP_SET_THREADTASK|HEAVYTASK_LIMIT_TO_LITCORE_HOK_SET_THREADTASK)

#define HEAVYTASK_LIMIT_TO_LITCORE_NOTDO HEAVYTASK_LIMIT_TO_LITCORE_NONE_DO_MIGRATE_ONLY

extern u32 heavytask_limit_to_litcore;
extern int rsc_svp_set_count;
extern struct cpumask rsc_cpu_all;
#define RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX 32
extern u32 rsc_debug_enqueue_otherthread_cnt[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
extern char rsc_debug_enqueue_otherthread[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX][TASK_COMM_LEN];
extern char rsc_debug_enqueue_otherthread_gl[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX][TASK_COMM_LEN];
extern u32 rsc_debug_enqueue_heavytask_uid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
extern u16 rsc_debug_enqueue_heavytask_pid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
extern u16 rsc_debug_enqueue_heavytask_glpid[RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX];
extern u32 rsc_debug_enqueue_otherthread_totalcnt;
extern u32 rsc_debug_enqueue_otherthread_actcnt;
extern struct cpumask rsc_cpu_bigcore;
extern int svp_limit_sched_setaffinity;
extern atomic_t svp_limit_sched_setaffinity_cnt;
extern atomic_t svp_notlimit_sched_setaffinity_cnt;
extern atomic_t rsc_total_svp_pid;
extern u64 heavytask_limit_intervaltime;

extern char *do_svp_limit_sched_setaffinity_list[RSC_SET_AFFINITY_LIMIT_MAX_PROCESS];
extern int do_svp_limit_sched_setaffinity_num;
extern u32 __read_mostly svp_max_cpu_usage_diff_cnt;
extern u32  svp_max_cpu_usage_overload_cnt;
extern u32 svp_max_cpu_usage_overload_jiffies_ignore_cnt;
extern u32  svp_max_cpu_usage_overload_delay_ms;
extern u32 svp_max_cpu_usage_overload_preempt_ignore_cnt;

/*
* For good performance,
* we use cpus_allowed to
* check is or not task is foreground task,
* instead of using cgroup.
*/
static inline bool svp_is_fg_task(struct task_struct *p)
{
	if (cpumask_intersects(&rsc_cpu_bigcore,
		&p->cpus_allowed))
		return true;

	return false;
}
static inline int rsc_should_do_action(char *app_name, char *list[], int cnt)
{
	int i, rslt = 0;

	for (i = 0; i < cnt; i++) {
		if (!strncmp(app_name, list[i], RSC_MAX_PACKAGE_NAME)) {
			rslt = 1;
			break;
		}
	}
	return rslt;
}

struct rq;
/*
extern void enqueue_svp_task(struct rq *rq, struct task_struct *p);
extern void dequeue_svp_task(struct rq *rq, struct task_struct *p);
extern void pick_svp_task(struct rq *rq, struct task_struct **p, struct sched_entity **se);
*/
extern void dynamic_svp_dequeue(struct task_struct *task, int type);
extern const struct sched_class fair_sched_class;
extern int dynamic_svp_enqueue_internal(struct task_struct *task, struct task_struct *from,
	int type, int depth, const char *func);
extern void trigger_svp_balance(struct rq *rq);
extern void svp_init_rq_data(struct rq *rq);
extern int set_cpus_allowed_ptr_force(struct task_struct *p, const struct cpumask *new_mask);

#define RSC_SVP_MANUAL 0x1
extern u32 RSC_SVP_MANUAL_VAL;
/*move to sched.h*/
/*#define RSC_SVP_MAINTHREAD 0x2*/
#define RSC_SVP_MANUAL_GROUP_LEADER 0x4
#define RSC_SVP_MANUAL_ABNORMAL_TASK 0x8
#define RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE 0x10
#define RSC_SVP_FORBIT_DO_CPUSET 0x20

//#define RSC_SVP_BINDER_PROMOTE_ONLY 0x40
//#ifdef CONFIG_RSC_LOCK_BOOST
//CONFIG_RSC_LOCK_BOOST_IMPROVE
#define RSC_SVP_PREEMPT 0x40
//#endif

/*set to zero to disable boost renderthread*/
#define RSC_SVP_RENDERTHREAD 0x0
#define RSC_SVP_MANUAL_HAS_SET 0x80

static inline void set_rsc_svp(struct task_struct *p, u8 val)
{
	u8 old_svp, svp;

	do {
		old_svp = svp = READ_ONCE(p->rsc_svp);
		svp |= val;
	} while (cmpxchg(&p->rsc_svp, old_svp, svp) != old_svp);
}

static inline void clear_rsc_svp(struct task_struct *p, u8 val)
{
	u8 old_svp, svp;

	do {
		old_svp = svp = READ_ONCE(p->rsc_svp);
		svp &= ~val;
	} while (cmpxchg(&p->rsc_svp, old_svp, svp) != old_svp);
}

extern u32 __read_mostly dynamic_svp_enqueue_enable;
extern struct cpumask rsc_svp_run_cpus;

#ifdef CONFIG_RSC_LOCK_BOOST
extern const char *rsc_dynamic_svp_name[DYNAMIC_SVP_MAX];
extern int svp_max_dynamic_granularity;
extern unsigned long dynamic_svp_count[DYNAMIC_SVP_MAX];
extern u32 dynamic_svp_timeout_count[DYNAMIC_SVP_MAX];
extern u32 dynamic_svp_timeout_deq_count[DYNAMIC_SVP_MAX];
extern u32 dynamic_svp_tick_hit;
extern u32 dynamic_svp_timeout_total_count;
extern int dynamic_svp_max_depth;
extern  u32 dynamic_svp_depth_count[SVP_DEPTH_MAX+1];
#ifdef CONFIG_RSC_LOCK_BOOST_STAT
extern u64 dynamic_svp_cost_time[DYNAMIC_SVP_MAX];
#endif
extern u64 dynamic_svp_slim_cost_time[DYNAMIC_SVP_MAX];
extern u32 dynamic_svp_tick_preempt_tick_count;
//extern u32 rsc_svp_tick_preempt_tick_count;
extern u32 rsc_svp_preempt_pickup_count;
extern unsigned long rsc_svp_select_cpu_hit_count;

/*debug control switch*/
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
#define RSC_FUTEX_DEBUG
#endif
#ifdef RSC_FUTEX_DEBUG
#define RSC_FUTEX_STAT_ARRAY_CNT 3
#else
#define RSC_FUTEX_STAT_ARRAY_CNT 2
#endif

#define RSC_ALLOW_SET_DYNAMIC_SVP_REENTRY

extern u32 rsc_pi_lock_count[RSC_FUTEX_STAT_ARRAY_CNT];
extern unsigned long rsc_futex_not_boost_count[RSC_FUTEX_STAT_ARRAY_CNT];
extern unsigned long rsc_futex_lock_boost_hit;
extern u32 rsc_fuetx_boost_pid_zero[RSC_FUTEX_STAT_ARRAY_CNT];

extern u32 rsc_lock_boost_trace[DYNAMIC_SVP_MAX][SVP_DEPTH_MAX];
extern u32 rsc_block_chain_boost_count[DYNAMIC_SVP_MAX];
extern u32 rsc_futex_uaddr2_value_wrong_count;
extern u32 rsc_futex_pid_change_count;
extern u32 rsc_futex_pid_change_pid_invalid_count;
extern u32 rsc_futex_block_chain_dead_loop_count[DYNAMIC_SVP_MAX][2];
extern u32 rsc_futex_timespec_invalid_count[2];

#define RSC_PTHREAD_MUTEX_BOOST			    0x10000
#define RSC_SEM_BOOST_PID				    0x20000
#define RSC_CLASS_LOCK_BOOST_PID		    0x40000
#define RSC_ART_JAVA_LOCK_BOOST_PID		    0x80000
#define RSC_ART_JAVA_LOCK_NOT_BOOST_PID	   0x100000
#define RSC_COND_WAIT_LOCK_BOOST_PID	   0x200000
#define RSC_SEM_MULTI_COUNT_BOOST_PID	   0x400000
#define RSC_RW_LOCK_BOOST_PID			   0x800000
#define RSC_QPATH_MAX_BOOST_PID			   0x800000
#define RSC_MAX_BOOST_PID_ARRAY_SIZE		9

extern u32 rsc_futex_boost_count[RSC_MAX_BOOST_PID_ARRAY_SIZE][RSC_FUTEX_STAT_ARRAY_CNT];
extern u32 rsc_futex_boost_zero_count[RSC_MAX_BOOST_PID_ARRAY_SIZE][RSC_FUTEX_STAT_ARRAY_CNT];
extern const char *rsc_futex_boost_pid_name[RSC_MAX_BOOST_PID_ARRAY_SIZE];

extern u32 rsc_mutex_wakeup_count;
extern u32 rsc_mutex_seq_boost_count;
extern u32 rsc_mutex_seq_boost_fail_count;
extern u32 rsc_mutex_seq_boost_loop_over_count;

DECLARE_PER_CPU(unsigned int, rsc_rwsem_enq_boost_count);
DECLARE_PER_CPU(unsigned int, rsc_rwsem_deq_boost_write_count);
extern u32 dynamic_svp_page_alloc_count;
extern u16 rsc_surfaceflinger_pid;
extern struct task_struct *rsc_surfaceflinger;

extern struct task_struct *rsc_launcher_group_leader;
extern struct task_struct *rsc_blur_server_group_leader;
#endif

#define RSC_SVP_SET_RENDER_THREAD 4

//#define RSC_LOCK_BOOST_STATTIME_USE_JIFFIES
static inline u64 rsc_get_clock(void)
{
#ifdef RSC_LOCK_BOOST_STATTIME_USE_JIFFIES
	u64 now =  jiffies_to_nsecs(jiffies);
#else
	u64 now =  sched_clock();
#endif

	return now;
}
#endif

#if 0
static inline void init_task_svp_info(struct task_struct *p)
{
	p->rsc_svp = 0;
	/*atomic64_set(&(p->dynamic_svp), 0);*/
	INIT_LIST_HEAD(&p->svp_entry);
	p->svp_depth = 0;
	p->enqueue_time = 0;
	/*p->dynamic_svp_start = 0;*/
}
#endif

#define task_cred_xxx_nolock(task, xxx)			\
({							\
	__typeof__(((struct cred *)NULL)->xxx) ___val;	\
	___val = __task_cred((task))->xxx;		\
	___val;						\
})

#define task_rsc_alloc_nolock(task)		(task_cred_xxx_nolock((task), rsc_alloc))

#define RSC_RENDER_THREAD_DEFAULT_NICE_VAL (-10)
#endif

#ifdef CONFIG_RSC_FIX_TASK_SUSPEND_DEADLOCK
#define RSC_SUSPEND_CHECK_MAX_PROCESS 8
#define RSC_SUSPEND_CHECK_SPEC_SIGNUM 4

#define RSC_NO_GCTHREAD_IN_SUSPEND -3
#define RSC_GCTHREAD_NOT_IN_SUSPEND -2
#define RSC_MAINTHREAD_NOT_IN_SUSPEND -1
#define RSC_IS_NOT_IN_SUSPEND 0
#define RSC_IS_IN_SUSPEND 1
#define RSC_FORCE_IN_SUSPEND 0x80
#define RSC_IN_SUSPEND_MASK 0x7f
extern bool rsc_need_check_suspend(struct task_struct *p);
extern int rsc_check_signal_enable;
extern int rsc_check_suspend_uid;
extern int rsc_suspend_check_start_signum;
extern int rsc_suspend_check_end_signum;
extern char rsc_suspend_check_task[RSC_SUSPEND_CHECK_MAX_PROCESS][TASK_COMM_LEN];
extern int rsc_suspend_check_task_num;
extern int rsc_suspend_check_sig[RSC_SUSPEND_CHECK_SPEC_SIGNUM];
extern int rsc_sig24_ignore;
extern unsigned long rsc_sig_cost_time_thr;
extern unsigned long rsc_sigsuspend_cnt;
extern int rsc_sigsuspend_toolong_cnt;
extern int rsc_sig_fatal_pending_cnt;
extern int rsc_sig_other_pending_cnt;
extern atomic_t rsc_fix_suspend_deadlock_time;
extern atomic_t rsc_fix_suspend_deadlock_fail_time;
extern atomic_t rsc_forcefix_suspend_deadlock_time;
extern atomic_t rsc_fix_suspend_try_time;
extern int rsc_force_ignore_sig24;
extern unsigned long  rsc_force_delay_for_suspend_deadlock_time_ns;
extern int rsc_resend_signum;
extern atomic_t rsc_hok_not_in_suspend_cnt[-(RSC_NO_GCTHREAD_IN_SUSPEND)+1];
extern u32 rsc_max_sigsuspend_costtime_ms;
extern char rsc_hok_gc_thread[TASK_COMM_LEN];
extern int rsc_force_delay_for_suspend_deadlock;
extern bool rsc_need_check_suspend_current(void);
#ifndef RSC_NS_TO_JIFF
#define RSC_NS_TO_JIFF(ns)		((ns) * HZ/(1000UL * 1000UL * 1000UL))
#endif
extern atomic_t rsc_total_svp_pid;
extern u32 rsc_get_cmdline_cnt;
extern u32 rsc_hok_changename_cnt;
extern u32 rsc_sig_has_val_cnt;
#endif

#ifdef CONFIG_RSC_APP_LAUNCH_BOOST
#define RSC_DUAL_APP_START_	99910000 /*same as RSC_DUAL_APP_START*/
#define RSC_APP_START_	10000 /*sane as RSC_APP_START*/
#define RSC_APP_MAX_	600 /*5000*/

extern unsigned long __read_mostly rsc_app_boost_stime;
extern unsigned long rsc_appboost_late_no_framework;
extern u32 __read_mostly rsc_app_boost_uid;
extern u16 __read_mostly rsc_app_boost_mainpid;

#define RSC_MIN_LITTLE_CORE 0
#define RSC_MAX_LITTLE_CORE 3
#define RSC_MIN_BIG_CORE 4
#define RSC_SUPER_BIG_CORE 7

/*task->mainthread type*/
#define RSC_UI_MAINTHREAD 0x1
#ifdef CONFIG_RSC_VAUDIT
#define RSC_VAUDIT_MAINTHREAD 0x2
#define RSC_VAUDIT_SVP_MAINTHREAD 0x4
#define RSC_NEED_VAUDIT_MAINTHREAD (RSC_UI_MAINTHREAD|RSC_VAUDIT_MAINTHREAD)
#endif

#define RSC_BINDER_MAINTHREAD 0x8

/*6*/
#define RSC_RWSEM_MAINTHREAD 0x10
/*7*/
#define RSC_MUTEX_MAINTHREAD 0x20

/*
* the status will be reset in find_energy_efficient_cpu after rsc_app_boost_stime+3s
*/
#define RSC_INVERSION_MAINTHREAD (RSC_RWSEM_MAINTHREAD|RSC_MUTEX_MAINTHREAD)

#define RSC_PROMOTE_MAINTHREAD (RSC_BINDER_MAINTHREAD|RSC_RWSEM_MAINTHREAD|RSC_MUTEX_MAINTHREAD)

#define RSC_UNKOWN1_MAINTHREAD 16
#define RSC_UNKOWN2_MAINTHREAD 17

#if 0
#define rsc_is_mainthread(p) (p->mainthread &  RSC_UI_MAINTHREAD)
#else
//#define rsc_is_mainthread(p) (p->pid == rsc_app_boost_uid)
#define rsc_is_mainthread(p) (likely(rsc_feat(APP_LAUNCH_BY_FRAMEWORK))?(p->pid == rsc_app_boost_mainpid):(p->mainthread & 	RSC_UI_MAINTHREAD))
#endif
static inline bool test_task_mainthread(struct task_struct *p)
{
#ifdef CONFIG_RSC_LOCK_BOOST
	return (p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD));
#else
	return rsc_is_mainthread(p);
#endif
}

static inline bool test_task_top_mainthread(struct task_struct *p)
{
#ifdef CONFIG_RSC_LOCK_BOOST
	return ((p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && rsc_top_task(p));
#else
	return rsc_is_mainthread(p);
#endif
}
#endif

#ifdef CONFIG_RSC_OPT_KGSL_POOL
#define RSC_BOOST_KGSL_POOL_MAX_PROCESS 8

#define RSC_BOOST_KGSL_MAX_POOLS 4

extern char boost_kgsl_pool_process[RSC_BOOST_KGSL_POOL_MAX_PROCESS][TASK_COMM_LEN];
extern int boost_kgsl_pool_process_num;
extern u32 boost_kgsl_count;
extern u32 boost_kgsl_hit;
extern u32 boost_kgsl_install_count;
extern u32 boost_kgsl_install_hit;

static inline int is_opt_process(char *app_name, int name_len, char *list, int cnt)
{
	int i, rslt = 0;

	for (i = 0; i < cnt; i++) {
		if (!strncmp(app_name, list + (i * name_len), name_len)) {
			rslt = 1;
			break;
		}
	}
	return rslt;
}
#endif

#if (defined(CONFIG_RSC_DROP_MMAP_SEM_DEBUG) && (CONFIG_RSC_DROP_MMAP_SEM_DEBUG > 0)) || defined(CONFIG_RSC_DROP_MMAP_SEM_IN_MKWRITE)
enum {
	MMAP_SEM_DBG_FORCEOFF	= 0,
	MMAP_SEM_DBG_OFF		= 1,
	MMAP_SEM_DBG_BRIEF 		= 2,/*2*/
	MMAP_SEM_DBG_DETAIL		= 4,/*4*/
};
extern u32 rsc_mmap_sem_debug;
extern u32 rsc_io_schedule_print_ms;
extern int rsc_io_schedule_print_group;

extern bool rsc_disable_drop_sem_in_filemap_fault;
extern bool rsc_disable_drop_sem_in_mkwrite;
extern bool rsc_disable_drop_sem_in_madvise;
extern bool rsc_disable_drop_sem_in_showmap;
extern bool rsc_disable_drop_sem_in_gpu_driver;
#endif

#ifndef RSC_JIFF_TO_MS
#define RSC_JIFF_TO_MS(jif)		((jif) * 1000/HZ)
#endif
#ifndef RSC_MS_TO_JIFF
#define RSC_MS_TO_JIFF(ms)		((ms) * HZ/1000)
#endif
#ifdef CONFIG_RSC_KSWAPD_IMPROVE
#define RSC_VSWAPD_STOP_DEFAULT_PAGES (150UL << (20 - PAGE_SHIFT))
#define RSC_VSWAPD_START_DEFAULT_PAGES (120UL << (20 - PAGE_SHIFT))

extern wait_queue_head_t __read_mostly vswapd_wait;
extern int __read_mostly kswapd_is_asleep;
extern u32 __read_mostly vswapd_start_pages;
extern u32 vswapd_stop_pages;
extern unsigned long vswapd_total_pages;
extern unsigned long vswapd_total_time_jiffies;
extern u64 vswapd_total_exe_time_us;
extern u32 vswapd_total_wakeup_cnt;
extern u32 vswapd_total_timeout_cnt;
extern u32 vswapd_total_kswap_running_cnt;
extern struct task_struct *vswapd_task;
extern u32 vswapd_max_running_time_jiffies;
extern u32 vswapd_min_filecache_stop_pages;
extern u32 vswapd_total_min_filecache_cnt;
extern int vswapd_emergency_stop;
extern u32 vswapd_reclaim_zero_cnt;
extern u32 vswapd_emergency_stop_wakeup_miss_cnt;
extern u32 vswapd_emergency_stop_timeout_cnt;
extern u32 vswapd_short_sleep_wakeup_cnt;

int rsc_vswapd(void *p);
void setup_per_zone_wmarks(void);
#if 0 //disable fo MTK
int rsc_vswap_fn(void *p);

extern u32 vswap_fn_wakeup_cnt;
extern u32 vswap_fn_wakeup_no_swap_cnt;
extern struct task_struct *vswap_fn_task;
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /*__RSC_INTERNAL_H__*/
