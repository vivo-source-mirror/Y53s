#ifndef _RSC_CPU_INT_H
#define _RSC_CPU_INT_H

#include <linux/pm_qos.h>
#include <linux/list.h>
#include <linux/cpufreq.h>
#include <linux/stacktrace.h>

struct pm_qos_cpu_internal_req {
	struct pm_qos_request cpu_num_min_qos;
	struct pm_qos_request l_cpufreq_min_qos;
	struct pm_qos_request b_cpufreq_min_qos;

	struct pm_qos_request cpu_num_max_qos;
	struct pm_qos_request l_cpufreq_max_qos;
	struct pm_qos_request b_cpufreq_max_qos;
};

struct cpu_level_table {
	int cluster0_num;
	int cluster0_cpufreq;
	int cluster1_num;
	int cluster1_cpufreq;
};

#define INIT(c0n, c0f, c1n, c1f) {\
	.cluster0_num = c0n,\
	.cluster0_cpufreq = c0f,\
	.cluster1_num = c1n,\
	.cluster1_cpufreq = c1f,\
}

#define CPU_LEVEL_NUM (PM_QOS_CPU_LEVEL_MAX_DEFAULT_VALUE + 1)
#define CPU_MIN_LEVEL PM_QOS_CPU_LEVEL_MIN_DEFAULT_VALUE
#define CPU_MAX_LEVEL PM_QOS_CPU_LEVEL_MAX_DEFAULT_VALUE

#define CPU_DIRECT_FREQ_REQ
#define CPU_INTERNAL_FREQ_REQ
/*#define CONFIG_VIVO_CPUFREQLIM_CHANGE_NOTIFY*/

/*#define CPU_CORECTL_REQ*/



/*#define LITTLE_CLUSTER_CPU 	0*/
/*#define BIG_CLUSTER_CPU 	4*/
typedef enum {
	MIN_CPUFREQ = 1 << 0,
	MAX_CPUFREQ = 1 << 1,
} cpufreq_mode;

typedef enum {
	CL_ZERO = 0,
	CL_ONE,	
	CL_TWO,
	CL_END,
} cluster_type;

typedef enum {
	CLUSER0_MINFREQ = 0,
	CLUSER0_MAXFREQ,
	CLUSER1_MINFREQ,
	CLUSER1_MAXFREQ,
	CLUSER_ENDFREQ,
	CLUSER_ALL = 0xffff,
} cluster_freq_type;

typedef enum {
	CPUFREQ_SPEC_FREQ_START = -1,
	CPUFREQ_RESET_FREQ 		= CPUFREQ_SPEC_FREQ_START,
	CPUFREQ_NOTSET_FREQ 	= -2,

	/*The last one spec value*/
	CPUFREQ_SPEC_FREQ_END	= CPUFREQ_NOTSET_FREQ,
} cpufreq_spec;

#define MAX_CLUSTER_NUM 2
#define rsc_cpu_to_cluster(cpu) ((cpu < cluster_pol_cpu[CL_ONE]) ? CL_ZERO : CL_ONE)
#define RSC_US_TO_NS(usec)		((usec) * 1000)
#define RSC_MS_TO_US(msec)		((msec) * 1000)
#define RSC_MS_TO_NS(msec)		((msec) * 1000 * 1000)
#define RSC_NS_TO_MS(nsec)		((nsec) / (1000 * 1000))
#define RSC_MS_TO_JIFF(ms)		((ms) * HZ/1000)
#define RSC_JIFF_TO_MS(jif)		((jif) * 1000/HZ)
#define RSC_S_TO_JIFF(s)		((s) * HZ)
#define RSC_JIFF_TO_S(s)		((s)/HZ)

#define RSC_MAX_STACK_ENRTYS 30

int rsc_cpu_internal_init(void);
#ifdef CPU_DIRECT_FREQ_REQ
struct rsc_cpu_freq_st {
	int freq;
	long timeout;/*ms*/
	u64 ts_nsec;/* time of setting*/
};

struct cpu_direct_req_st {
	struct rsc_cpu_freq_st cluster_freq[CLUSER_ENDFREQ];
	int hasset;
	struct mutex lock;
};

extern struct cpu_direct_req_st glb_cpu_direct_req;
extern const int cluster_minfreq_qos_class[CL_END];
extern const int cluster_maxfreq_qos_class[CL_END];

extern int rsc_cpu_direct_update(struct rsc_cpu_freq_st *cluster_freq);

#endif

extern const int default_freq[CLUSER_ENDFREQ];
extern int cluster_pol_cpu[CL_END];
extern int rsc_cpu_initialized;
#ifdef CONFIG_VIVO_CPUFREQLIM_CHANGE_NOTIFY
struct cpufreq_notify_st {
	int min;
	int max;
	int last_min;
	int last_max;
	int cluster;
	u16 change_minmax_bit;
	char min_change_reason[CHANGE_REASON_BUFSIZE];
	char max_change_reason[CHANGE_REASON_BUFSIZE];
	char comm[TASK_COMM_LEN];
};
#define NOTIFY_QUERY_SIZE 16
extern struct cpufreq_notify_st cpufreq_change[NOTIFY_QUERY_SIZE][CL_END];
extern struct mutex cpufreq_change_lock;
extern int cpufreq_change_pos;
#endif

#ifdef CONFIG_RSC_V2_CPU_CAP_NOTIFY
struct cpufreq_adj_cell {
	int is_valid;
	rsc_cpucap_adj_type mode;

	unsigned int cur_freq, min_freq, max_freq, max_mitigated_freq;
	unsigned int last_max_freq, last_max_mitifreq;

	int cpu;
	struct cpumask cpumask;
	/*cpu offline*/
	struct cpumask offline;

	int percent, last_percent;

	struct stack_trace callstack;
	unsigned long stack_entries[RSC_MAX_STACK_ENRTYS];
	struct timeval tv;
	pid_t pid;
	char comm[TASK_COMM_LEN];
	unsigned int seq;
};
#endif

DECLARE_PER_CPU(int, cpu_to_cluster_map);

extern unsigned long rsc_total_cpu_efficiency;
extern unsigned long rsc_min_max_freq;
extern unsigned long rsc_min_possible_efficiency;
extern uint32_t cpus_offlined;
#ifdef CONFIG_RSC_V2_CPU_CAP_NOTIFY
extern struct cpufreq_adj_cell rsc_cpu_cluster[CL_END];
#endif
extern struct cpufreq_freqs rsc_cpu_freqs[CL_END];
extern u32 rsc_max_freq[CL_END];
extern u32 rsc_max_mitigated_freq[CL_END];

DECLARE_PER_CPU(unsigned long, rsc_scale_cpu_efficiency);
DECLARE_PER_CPU(unsigned long, rsc_scale_cpu_freq);
DECLARE_PER_CPU(unsigned long, rsc_cpu_maxfreq);

int __init rsc_cpu_internal_init(void);

#ifdef CONFIG_RSC_TASK_CPU_USAGE
	#define CPU_USAGE_DEF_PERIOD 10000 /*mini second*/
	#if 1
		/*default disable in boot*/
		#define CPU_USAGE_DEFAULT_ENABLE	0

	#else
		/*default enable in boot*/
		#define CPU_USAGE_DEFAULT_ENABLE	1
	#endif
	#if CPU_USAGE_DEFAULT_ENABLE == 0
		#define CPU_USAGE_INIT_PERIOD 		-1
	#else
		#define CPU_USAGE_INIT_PERIOD	CPU_USAGE_DEF_PERIOD
	#endif
	extern struct completion cpucap_notify_complete;
	extern long rsc_cpu_usage_notify(long comret);
#endif

#endif /*_RSC_CPU_H*/

