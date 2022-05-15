#ifndef _VIVO_RSC_IOCTL_H
#define _VIVO_RSC_IOCTL_H

#include <linux/ioctl.h>

#define VIVO_RSC_IOCTL_NAME "vivo_rsc_dev"

#define RSC_IOCTL_WRITE 1U
#define RSC_IOCTL_READ 	2U

enum {
	RSC_ID_START = 0x49435352,
	RSC_ID_PERFD = RSC_ID_START,
	RSC_ID_BBKLOG,	/*0x49435353*/
	RSC_ID_POWER,	/*0x49435354*/
	RSC_ID_VIVOD,	/*0x49435355	*/
	RSC_ID_BIGDATA, /*0x49435356*/
	RSC_ID_SHELL,	/*0x49435357*/
	RSC_ID_OTHER,	/*0x49435358*/
	RSC_ID_END
};

#define RSC_ID(name) __stringify(name)

enum {
	RSC_IOCTL_MODE,
	RSC_FILE_MODE,
};

#ifndef NODE_MAX
#define NODE_MAX                150
#endif

struct file_mode {
	char file_path[NODE_MAX];
	char str[NODE_MAX];
	int len;
};

struct rsc_core_data {
	/* RSC_IOCTL_MODE or RSC_FILE_MODE */
	unsigned char access_mode;
	/* RSC_IOCTL_WRITE or RSC_IOCTL_READ */
	unsigned char dir;
	uint32_t id;
	union {
		int val_int;
		unsigned int val_uint;
		unsigned int freq;
	};
	/*-1: no limit, 0: remove qos, > 0 : timeout ms*/
	int timeout_ms;
	int fd;
};

struct vivo_rsc_ioctl {
	uint16_t size;
	pid_t pid;
	pid_t tid;
	struct rsc_core_data core;
	/* members of RSC_FILE_MODE */
	struct file_mode f_mode;
};

#define RSC_QOS_WRITE_RESERVE_CNT (90 - 10)
#define RSC_NO_QOS_WRITE_RESERVE_CNT (50 - 1)

enum {
	RSC_FIRST_IOCTL,
	RSC_FREQ_SET_BEGIN 				 	= RSC_FIRST_IOCTL,
	RSC_MIN_CPU_FREQ 					= RSC_FREQ_SET_BEGIN,
	RSC_MIN_FREQ_CLUSTER_BIG_CORE 		= RSC_MIN_CPU_FREQ,
	RSC_MIN_FREQ_CLUSTER_LITTLE_CORE,
	RSC_MAX_CPU_FREQ,
	RSC_MAX_FREQ_CLUSTER_BIG_CORE 		= RSC_MAX_CPU_FREQ,
	RSC_MAX_FREQ_CLUSTER_LITTLE_CORE,
	RSC_FREQ_SET_END 					= RSC_MAX_FREQ_CLUSTER_LITTLE_CORE,

	RSC_SCHED_PREFER_IDLE,
	RSC_SCHED_MIGRATE_COST,
	RSC_SCHED_SMALL_TASK,
	RSC_SCHED_MOSTLY_IDLE_LOAD,
	RSC_SCHED_MOSTLY_IDLE_NR_RUN,
	RSC_SCHED_INIT_TASK_LOAD,
	RSC_SCHED_UPMIGRATE,
	RSC_SCHED_DOWNMIGRATE,
	RSC_SCHED_MOSTLY_IDLE_FREQ,
	/*RSC_SCHED_GROUP,sched_group_id*/
	RSC_SCHED_SPILL_NR_RUN,/*sched_spill_nr_run*/
	RSC_SCHED_STATIC_CPU_PWR_COST,
	RSC_SCHED_RESTRICT_CLUSTER_SPILL,
	/*RSC_SCHED_FREQ_AGGR_GROUP,/proc/%s/sched_group_id*/

	RSC_SCHED_SET_FREQ_AGGR,/*"/proc/sys/kernel/sched_freq_aggregate"*/
	RSC_SCHED_ENABLE_THREAD_GROUPING,/*uint 32 "/proc/sys/kernel/sched_enable_thread_grouping"*/
	RSC_SCHED_GROUP_UPMIGRATE,
	RSC_SCHED_GROUP_DOWNMIGRATE,
	RSC_SCHED_FREQ_AGGR_THRESHOLD,
	RSC_MIN_ONLINE_CPU_CLUSTER_BIG,
	RSC_MAX_ONLINE_CPU_CLUSTER_BIG,

	RSC_BOOST_INTERACTIVE_CLUSTER_BIG,
	RSC_BOOSTPULSE_INTERACTIVE_CLUSTER_BIG,
	RSC_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_BIG,
	RSC_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_BIG,
	RSC_HISPEED_FREQ_INTERACTIVE_CLUSTER_BIG,
	RSC_IO_IS_BUSY_INTERACTIVE_CLUSTER_BIG,
	RSC_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_BIG,
	RSC_TARGET_LOADS_INTERACTIVE_CLUSTER_BIG,
	RSC_TIMER_RATE_INTERACTIVE_CLUSTER_BIG,
	RSC_TIMER_SLACK_INTERACTIVE_CLUSTER_BIG,
	RSC_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_BIG,
	RSC_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_BIG,
	RSC_USE_MIGRATION_NOTIF_CLUSTER_BIG,
	RSC_IGNORE_HISPEED_NOTIF_CLUSTER_BIG,

	RSC_MIN_ONLINE_CPU_CLUSTER_LITTLE,
	RSC_MAX_ONLINE_CPU_CLUSTER_LITTLE,
	RSC_BOOST_INTERACTIVE_CLUSTER_LITTLE,
	RSC_BOOSTPULSE_INTERACTIVE_CLUSTER_LITTLE,
	RSC_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_LITTLE,
	RSC_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_LITTLE,
	RSC_HISPEED_FREQ_INTERACTIVE_CLUSTER_LITTLE,
	RSC_IO_IS_BUSY_INTERACTIVE_CLUSTER_LITTLE,
	RSC_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_LITTLE,
	RSC_TARGET_LOADS_INTERACTIVE_CLUSTER_LITTLE,
	RSC_TIMER_RATE_INTERACTIVE_CLUSTER_LITTLE,
	RSC_TIMER_SLACK_INTERACTIVE_CLUSTER_LITTLE,
	RSC_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_LITTLE,
	RSC_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_LITTLE,
	RSC_USE_MIGRATION_NOTIF_CLUSTER_LITTLE,
	RSC_IGNORE_HISPEED_NOTIF_CLUSTER_LITTLE,

	RSC_CPUBW_HWMON_MIN_FREQ,
	RSC_CPUBW_HWMON_DECAY_RATE,
	RSC_CPUBW_HWMON_IO_PERCENT,

	RSC_CPUBW_HWMON_HYST_MEMORY,
	RSC_CPUBW_HWMON_HYST_LEN,
	RSC_CPUBW_HWMON_HYST_TRIG_CNT,
	RSC_CPUBW_HWMON_GUARD_BAND_MBPS,
	RSC_CPUBW_HWMON_UP_SCALE,

	RSC_CPUBW_HWMON_LOW_POWER_CEIL_MBPS,
	RSC_CPUBW_HWMON_LOW_POWER_IO_PERCENT,
	RSC_CPUBW_HWMON_MAX_FREQ,
	RSC_CPUBW_HWMON_POLLING_INTERVAL,
	RSC_CPUBW_HWMON_SAMPLE_MS,
	RSC_CPUBW_HWMON_IDLE_MBPS,

	/*RSC_VIDEO_ENCODE_PB_HINT,
	RSC_VIDEO_DECODE_PB_HINT,
	RSC_VIDEO_DISPLAY_PB_HINT,

	RSC_KSM_RUN_STATUS,
	RSC_KSM_PARAMS,*/

	RSC_SAMPLING_RATE_ONDEMAND,
	RSC_IO_IS_BUSY_ONDEMAND,
	RSC_SAMPLING_DOWN_FACTOR_ONDEMAND,
	RSC_SYNC_FREQ_ONDEMAND,
	RSC_OPTIMAL_FREQ_ONDEMAND,
	RSC_ENABLE_STEP_UP_ONDEMAND,
	RSC_MAX_INTERMEDIATE_STEPS_ONDEMAND,
	RSC_NOTIFY_ON_MIGRATE,

	RSC_GPU_POWER_LEVEL,/*"/sys/class/kgsl/kgsl-3d0/default_pwrlevel"*/
	RSC_GPU_MIN_POWER_LEVEL,/*"/sys/class/kgsl/kgsl-3d0/min_pwrlevel"*/
	RSC_GPU_MAX_POWER_LEVEL,/*"/sys/class/kgsl/kgsl-3d0/max_pwrlevel"*/
	RSC_GPU_MIN_FREQ,
	RSC_GPU_MAX_FREQ,
	RSC_GPU_BUS_MIN_FREQ,
	/*msm845 add*/
	RSC_GPU_BUS_MAX_FREQ,
	RSC_GPU_FORCE_RAIL_ON,
	RSC_GPU_FORCE_CLK_ON,
	RSC_GPU_IDLE_TIMER,
	RSC_GPU_FORCE_NO_NAP,
	RSC_GPU_LLCCBW_MIN_FREQ,/*/sys/class/devfreq/soc:qcom,llccbw/min_freq*/
	RSC_GPU_LLCCBW_MAX_FREQ,/*/sys/class/devfreq/soc:qcom,llccbw/max_freq*/

	/*RSC_IRQ_BALANCER,
	RSC_UNSUPPORTED,
	RSC_INPUT_BOOST_RESET,*/
	RSC_SWAP_RATIO,
	/*RSC_KEEP_ALIVE,*/

	RSC_BIG_CORE_CTL,/* /sys/devices/system/cpu/cpu4/core_ctl/big_core_ctl*/
	/*msm845 new add*/
	RSC_LPM_BIAS_HYST,
	RSC_LPM_LEVELS_REF_STDDEV,
	RSC_LPM_LEVELS_TMR_ADD,
	RSC_HWMON_MIN_FREQ,
	RSC_L3_MIN_FREQ,
	RSC_L3_CPU0_MIN_FREQ,/* /sys/class/devfreq/soc:qcom,l3-cpu0/min_freq*/
	RSC_L3_CPU4_MIN_FREQ,/* /sys/class/devfreq/soc:qcom,l3-cpu4/min_freq*/
	RSC_MEMLAT_RATIO_CEIL_0,
	RSC_MEMLAT_RATIO_CEIL_1,/*89*/
	RSC_CPU_USAGE_PERCENT,
	/* add new node here!!*/
	RSC_QOS_WRITE_NODE_NEXT,
	RSC_QOS_WRITE_NODE_END = RSC_QOS_WRITE_NODE_NEXT - 1,

	/*no qos, write node*/
	RSC_NO_QOS_WRITE_NODE_START = 90 + RSC_QOS_WRITE_RESERVE_CNT,/*180 RSC_QOS_WRITE_NODE_NEXT + RSC_QOS_WRITE_RESERVE_CNT,*/
	RSC_SCHED_BOOST = RSC_NO_QOS_WRITE_NODE_START,
	RSC_SCHED_CPUSET_TOP_APP,/*cgroup #define SCHED_TOP_APP_CPUSET "/dev/cpuset/top-app/cpus"*/
	RSC_SCHED_CPUSET_FOREGROUND,/*cgroup #define SCHED_FOREGROUND_CPUSET "/dev/cpuset/foreground/cpus"*/

	/*#define SCHED_FOREGROUND_BOOST "/dev/cpuset/foreground/boost/cpus"*/

	RSC_SCHED_CPUSET_SYSTEM_BACKGROUND,/*cgroup #define SCHED_SYSTEM_BACKGROUND_CPUSET "/dev/cpuset/system-background/cpus"*/
	RSC_SCHED_CPUSET_BACKGROUND,/*cgroup #define SCHED_BACKGROUND_CPUSET "/dev/cpuset/background/cpus"*/
	RSC_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_BIG,
	RSC_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_LITTLE,

	RSC_MSM_MANAGED_CPUS,
	RSC_KPM_NUM_CLUSTERS,
	RSC_MMC0_CLK_SCANLING,
	RSC_LPM_SLEEP_DISABLED,
	RSC_CPU0_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU1_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU2_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU3_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU4_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU5_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU6_SCHED_STATIC_CPU_PWR_COST,
	RSC_CPU7_SCHED_STATIC_CPU_PWR_COST,
	RSC_STORAGE_CLK_SCALING_DISABLE,
	/*msm845 new add*/
	RSC_TOGGLE_L2_PC_PERF,/*200*/
	/*new add by msm8953 x9 (pd1616), pd1724*/
	RSC_KPM_MAX_CPUS,

	/* add new node here!!*/
	RSC_NO_QOS_WRITE_NEXT_NODE,
	RSC_NO_QOS_WRITE_NODE_END = RSC_NO_QOS_WRITE_NEXT_NODE - 1,

	/*no any record*/
	RSC_WRITE_DIRECTLY = 200 + RSC_NO_QOS_WRITE_RESERVE_CNT,
	/* next read only and not support qos ,see PM_QOS_RSC_END*/
	RSC_NO_QOS_READ_NODE_START,
	RSC_CPU_ONLINE = RSC_NO_QOS_READ_NODE_START,/* /sys/devices/system/cpu/online*/
	RSC_GPU_AVAILABLE_FREQ,
	RSC_GPUBW_AVAILABLE_FREQ,
	RSC_NO_QOS_OTHER_READ_NODE,
	RSC_NO_QOS_READ_NODE_NEXT,
	RSC_NO_QOS_READ_NODE_END = RSC_NO_QOS_READ_NODE_NEXT - 1,

	RSC_LAST_NR_IOCTL,/*max value is 255*/
	RSC_MAX_NR_IOCTL = 256,
};

#define VIVO_RSC_MAGIC_NUM 0xAC /*Unique magic number*/

/*
	the definition is so complex for fixing the check patch error :
	Macros with complex values should be	enclosed in parentheses
	scripts/checkpatch.pl xxx.patch
*/
#define define_rsc_one_ioctl_rw(name)											\
/*	(																			\
		do {	(*/																	\
				enum {															\
					VIVO_##name =												\
						_IOW(VIVO_RSC_MAGIC_NUM, (name), struct vivo_rsc_ioctl)	\
				}																\
/*		) } while (0)																\
	) */

#define RSC_IOCTL(name) VIVO_##name

define_rsc_one_ioctl_rw(RSC_FIRST_IOCTL);

define_rsc_one_ioctl_rw(RSC_MIN_FREQ_CLUSTER_BIG_CORE);
define_rsc_one_ioctl_rw(RSC_MIN_FREQ_CLUSTER_LITTLE_CORE);
define_rsc_one_ioctl_rw(RSC_MAX_FREQ_CLUSTER_BIG_CORE);
define_rsc_one_ioctl_rw(RSC_MAX_FREQ_CLUSTER_LITTLE_CORE);

define_rsc_one_ioctl_rw(RSC_SCHED_PREFER_IDLE);
define_rsc_one_ioctl_rw(RSC_SCHED_MIGRATE_COST);
define_rsc_one_ioctl_rw(RSC_SCHED_SMALL_TASK);
define_rsc_one_ioctl_rw(RSC_SCHED_MOSTLY_IDLE_LOAD);
define_rsc_one_ioctl_rw(RSC_SCHED_MOSTLY_IDLE_NR_RUN);
define_rsc_one_ioctl_rw(RSC_SCHED_INIT_TASK_LOAD);
define_rsc_one_ioctl_rw(RSC_SCHED_UPMIGRATE);
define_rsc_one_ioctl_rw(RSC_SCHED_DOWNMIGRATE);
define_rsc_one_ioctl_rw(RSC_SCHED_MOSTLY_IDLE_FREQ);
/*define_rsc_one_ioctl_rw(RSC_SCHED_GROUP);*/
define_rsc_one_ioctl_rw(RSC_SCHED_SPILL_NR_RUN);
define_rsc_one_ioctl_rw(RSC_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_SCHED_RESTRICT_CLUSTER_SPILL);
/*define_rsc_one_ioctl_rw(RSC_SCHED_FREQ_AGGR_GROUP);*/

define_rsc_one_ioctl_rw(RSC_SCHED_SET_FREQ_AGGR);
define_rsc_one_ioctl_rw(RSC_SCHED_ENABLE_THREAD_GROUPING);
define_rsc_one_ioctl_rw(RSC_SCHED_GROUP_UPMIGRATE);
define_rsc_one_ioctl_rw(RSC_SCHED_GROUP_DOWNMIGRATE);
define_rsc_one_ioctl_rw(RSC_SCHED_FREQ_AGGR_THRESHOLD);

define_rsc_one_ioctl_rw(RSC_MIN_ONLINE_CPU_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_MAX_ONLINE_CPU_CLUSTER_BIG);

define_rsc_one_ioctl_rw(RSC_BOOST_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_BOOSTPULSE_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_HISPEED_FREQ_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_IO_IS_BUSY_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_TARGET_LOADS_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_TIMER_RATE_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_TIMER_SLACK_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_USE_MIGRATION_NOTIF_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_IGNORE_HISPEED_NOTIF_CLUSTER_BIG);

define_rsc_one_ioctl_rw(RSC_MIN_ONLINE_CPU_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_MAX_ONLINE_CPU_CLUSTER_LITTLE);

define_rsc_one_ioctl_rw(RSC_BOOST_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_BOOSTPULSE_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_BOOSTPULSE_DURATION_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_GO_HISPEED_LOAD_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_HISPEED_FREQ_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_IO_IS_BUSY_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_MIN_SAMPLE_TIME_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_TARGET_LOADS_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_TIMER_RATE_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_TIMER_SLACK_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_MAX_FREQ_HYSTERESIS_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_USE_SCHED_LOAD_INTERACTIVE_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_USE_MIGRATION_NOTIF_CLUSTER_LITTLE);
define_rsc_one_ioctl_rw(RSC_IGNORE_HISPEED_NOTIF_CLUSTER_LITTLE);

define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_MIN_FREQ);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_DECAY_RATE);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_IO_PERCENT);
/*define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_HYST_OPT);*/

define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_HYST_MEMORY);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_HYST_LEN);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_HYST_TRIG_CNT);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_GUARD_BAND_MBPS);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_UP_SCALE);

define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_LOW_POWER_CEIL_MBPS);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_LOW_POWER_IO_PERCENT);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_MAX_FREQ);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_POLLING_INTERVAL);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_SAMPLE_MS);
define_rsc_one_ioctl_rw(RSC_CPUBW_HWMON_IDLE_MBPS);

/*define_rsc_one_ioctl_rw(RSC_VIDEO_ENCODE_PB_HINT);
define_rsc_one_ioctl_rw(RSC_VIDEO_DECODE_PB_HINT);
define_rsc_one_ioctl_rw(RSC_VIDEO_DISPLAY_PB_HINT);

define_rsc_one_ioctl_rw(RSC_KSM_RUN_STATUS);
define_rsc_one_ioctl_rw(RSC_KSM_PARAMS);
*/
define_rsc_one_ioctl_rw(RSC_SAMPLING_RATE_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_IO_IS_BUSY_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_SAMPLING_DOWN_FACTOR_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_SYNC_FREQ_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_OPTIMAL_FREQ_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_ENABLE_STEP_UP_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_MAX_INTERMEDIATE_STEPS_ONDEMAND);
define_rsc_one_ioctl_rw(RSC_NOTIFY_ON_MIGRATE);

define_rsc_one_ioctl_rw(RSC_GPU_POWER_LEVEL);
define_rsc_one_ioctl_rw(RSC_GPU_MIN_POWER_LEVEL);
define_rsc_one_ioctl_rw(RSC_GPU_MAX_POWER_LEVEL);
define_rsc_one_ioctl_rw(RSC_GPU_MIN_FREQ);
define_rsc_one_ioctl_rw(RSC_GPU_MAX_FREQ);
define_rsc_one_ioctl_rw(RSC_GPU_BUS_MIN_FREQ);

define_rsc_one_ioctl_rw(RSC_GPU_BUS_MAX_FREQ);
define_rsc_one_ioctl_rw(RSC_GPU_FORCE_RAIL_ON);
define_rsc_one_ioctl_rw(RSC_GPU_FORCE_CLK_ON);
define_rsc_one_ioctl_rw(RSC_GPU_IDLE_TIMER);
define_rsc_one_ioctl_rw(RSC_GPU_FORCE_NO_NAP);
define_rsc_one_ioctl_rw(RSC_GPU_LLCCBW_MIN_FREQ);
define_rsc_one_ioctl_rw(RSC_GPU_LLCCBW_MAX_FREQ);

/*define_rsc_one_ioctl_rw(RSC_IRQ_BALANCER);
define_rsc_one_ioctl_rw(RSC_UNSUPPORTED);
define_rsc_one_ioctl_rw(RSC_INPUT_BOOST_RESET);
*/
define_rsc_one_ioctl_rw(RSC_SWAP_RATIO);
/*define_rsc_one_ioctl_rw(RSC_KEEP_ALIVE);*/

define_rsc_one_ioctl_rw(RSC_BIG_CORE_CTL);
define_rsc_one_ioctl_rw(RSC_LPM_BIAS_HYST);
define_rsc_one_ioctl_rw(RSC_LPM_LEVELS_REF_STDDEV);
define_rsc_one_ioctl_rw(RSC_LPM_LEVELS_TMR_ADD);
define_rsc_one_ioctl_rw(RSC_HWMON_MIN_FREQ);
define_rsc_one_ioctl_rw(RSC_L3_MIN_FREQ);

define_rsc_one_ioctl_rw(RSC_L3_CPU0_MIN_FREQ);
define_rsc_one_ioctl_rw(RSC_L3_CPU4_MIN_FREQ);

define_rsc_one_ioctl_rw(RSC_MEMLAT_RATIO_CEIL_0);
define_rsc_one_ioctl_rw(RSC_MEMLAT_RATIO_CEIL_1);
define_rsc_one_ioctl_rw(RSC_CPU_USAGE_PERCENT);


define_rsc_one_ioctl_rw(RSC_SCHED_BOOST);
define_rsc_one_ioctl_rw(RSC_SCHED_CPUSET_TOP_APP);
define_rsc_one_ioctl_rw(RSC_SCHED_CPUSET_FOREGROUND);
define_rsc_one_ioctl_rw(RSC_SCHED_CPUSET_SYSTEM_BACKGROUND);
define_rsc_one_ioctl_rw(RSC_SCHED_CPUSET_BACKGROUND);

define_rsc_one_ioctl_rw(RSC_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_BIG);
define_rsc_one_ioctl_rw(RSC_ABOVE_HISPEED_DELAY_INTERACTIVE_CLUSTER_LITTLE);

define_rsc_one_ioctl_rw(RSC_MSM_MANAGED_CPUS);
define_rsc_one_ioctl_rw(RSC_KPM_NUM_CLUSTERS);
define_rsc_one_ioctl_rw(RSC_MMC0_CLK_SCANLING);
define_rsc_one_ioctl_rw(RSC_LPM_SLEEP_DISABLED);

define_rsc_one_ioctl_rw(RSC_CPU0_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU1_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU2_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU3_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU4_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU5_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU6_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_CPU7_SCHED_STATIC_CPU_PWR_COST);
define_rsc_one_ioctl_rw(RSC_STORAGE_CLK_SCALING_DISABLE);
define_rsc_one_ioctl_rw(RSC_TOGGLE_L2_PC_PERF);

define_rsc_one_ioctl_rw(RSC_KPM_MAX_CPUS);

define_rsc_one_ioctl_rw(RSC_WRITE_DIRECTLY);

define_rsc_one_ioctl_rw(RSC_CPU_ONLINE);
define_rsc_one_ioctl_rw(RSC_GPU_AVAILABLE_FREQ);
define_rsc_one_ioctl_rw(RSC_GPUBW_AVAILABLE_FREQ);
define_rsc_one_ioctl_rw(RSC_NO_QOS_OTHER_READ_NODE);

#endif
