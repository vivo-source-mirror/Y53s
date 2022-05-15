#ifndef _RSC_VAUDIT_H
#define _RSC_VAUDIT_H
#include <linux/types.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#else
#include <linux/signal.h>
#endif
#define VAUDIT_BUFFER_SIZE			(256)

#define VAUDIT_ALL_MASK				(0x3fffffff)

typedef enum {
	CHANNEL_ID_NONE         = 0x0,
	CHANNEL_ID_LOCAL_SERVER = 0x1 << 0,
	CHANNEL_ID_LOCAL_CLIENT = 0x1 << 1,
	CHANNEL_ID_NETLINK      = 0x1 << 2,
	CHANNEL_ID_KCOLLECT     = 0x1 << 3,
	CHANNEL_ID_END
} CHANNEL_ID;

typedef enum {
	PACKET_TAG_NONE = 0,
	PACKET_TAG_VALIDATE_CHANNEL,
	PACKET_TAG_MONITOR_CMD,
	PACKET_TAG_MONITOR_INFO,
	PACKET_TAG_KCOLLECT,
	PACKET_TAG_KPG,
	PACKET_TAG_END
} PACKET_TAG;

/*data format between user and kernel*/
struct vaudit_kmsg {
	PACKET_TAG tag;
	CHANNEL_ID src;
	CHANNEL_ID dst;
	unsigned int length;
	char buffer[0];
};

typedef enum {
	VAUDIT_SYSCALL             = 1U <<  0,/*0x1*/
	VAUDIT_SYSPROCFS           = 1U <<  1,/*0x2*/
	VAUDIT_BINDER              = 1U <<  2,/*0x4*/
	VAUDIT_IO                  = 1U <<  3,/*0x8*/
	VAUDIT_CPU                 = 1U <<  4,/*0x10*/
	VAUDIT_MEMORY              = 1U <<  5,/*0x20*/
	VAUDIT_NET                 = 1U <<  6,/*0x40*/
	VAUDIT_FS                  = 1U <<  7,/*0x80*/
	VAUDIT_TOUCHPANEL          = 1U <<  8,/*0x100*/
	VAUDIT_INTERRUPT           = 1U <<  9,/*0x200*/
	VAUDIT_DRIVER              = 1U << 10,/*0x400*/
	VAUDIT_OTHERS              = 1U << 11,/*0x800*/
	VAUDIT_SCHEDWAIT           = 1U << 12,/*0x1000*/

	VAUDIT_SET_COSTTIME_IDX        = 0x40000000,
	VAUDIT_DISABLE_SWITCH_IDX      = 0x80000000
} vaudit_type;

typedef enum {
	VAUDIT_SCHEDWAIT_IO,
	VAUDIT_SCHEDWAIT_NORMAL_SCHED,
	VAUDIT_SCHEDWAIT_SVP_SCHED,
} vaudit_schedwait_subtype;

struct vaudit_info {
	u32 type;
	u32 subtype;
	u32 tv_sec;	/* seconds */	
	u32 overval;	/* if time microseconds or other*/
	/*u32 tv_usec;*//* microseconds */
	u32 idx;
	u16 len;
	char buffer[VAUDIT_BUFFER_SIZE];
};

#if 0
static struct trace_print_flags rsc_vauditflag_names[] = {
	{VAUDIT_SYSCALL,		"VAUDIT_SYSCALL"},
	{VAUDIT_IO,	"VAUDIT_IO"},
	{VAUDIT_CPU,	"VAUDIT_CPU"},
	{VAUDIT_MEMORY,		"VAUDIT_MEMORY"},
	{VAUDIT_NET,		"VAUDIT_NET"},
	{VAUDIT_FS,	"VAUDIT_FS"},
	{VAUDIT_TOUCHPANEL,		"VAUDIT_TOUCHPANEL"},
	{VAUDIT_INTERRUPT,		"VAUDIT_INTERRUPT"},
	{VAUDIT_DRIVER,		"VAUDIT_DRIVER"},
	{VAUDIT_OTHERS,		"VAUDIT_OTHERS"}
};

#define show_vaudit_flags(flags)						\
	(flags) ? __print_flags(flags, "|",				\
	rsc_vauditflag_names						\
	) : "none"
#endif

/*skill shell task!*/
static inline void rsc_get_ptask(struct task_struct *p, char **pcomm, int *pid)
{
	struct task_struct *task;

	if (!thread_group_leader(p) && likely(!(p->exit_state & (EXIT_TRACE | TASK_DEAD)))) {
		*pid = p->group_leader?p->group_leader->pid:99999;
		*pcomm = p->group_leader?p->group_leader->comm:"NULL";
	} else {
		task = p->real_parent;
		if (task) {
			if (!strncmp("sh", task->comm, TASK_COMM_LEN) && (task->real_parent))
				task = task->real_parent;
			*pcomm = task->comm;
			*pid = task->pid;
		} else {
			*pcomm = "NULL";
			*pid = 99999;
		}
	}
}

/*
  * Function: vaudit
  * Description: collect the data and send to user space
  * It can use in atomic context env, except hardware interrupt.
  * It could not used in hardware interrupt context.

  * type is the main type see vaudit_type
  * subtype is user define type
  * overval is the value, maybe millisecond or times
  * fmt -- string
  * Return: -1--failed, 0--success
*/
int rsc_vaudit(vaudit_type type, u32 subtype, u32 overval, const char *fmt, ...);
extern u32 __read_mostly rsc_syscall_threshold_time;
extern u32 __read_mostly rsc_sysproc_threshold_bufsize;
extern u32 __read_mostly rsc_sysproc_threshold_filesize;
extern bool rsc_sysproc_checkonetime;
extern u32 rsc_sysproc_checkmode;
extern u32 rsc_sysproc_speedup_buf_size;
extern u32 rsc_sysproc_speedup_vmalloc_buf_size;

extern void rsc_seq_buf_pre_alloc(struct file *file, size_t size, u32 checkmode);

/*ns*/
extern unsigned long rsc_vaudit_iowait_threashold;
/*ns*/
extern unsigned long rsc_vaudit_iowait_period;
extern u32 rsc_vaudit_iowait_threashold_percent;

extern unsigned long __read_mostly rsc_vaudit_schedwait_threashold[2];
extern unsigned long __read_mostly rsc_vaudit_schedwait_period[2];
extern u32 rsc_vaudit_schedwait_threashold_percent[2];
extern unsigned long __read_mostly rsc_vaudit_schedwait_interruptsleep_threashold[2];

extern unsigned long __read_mostly rsc_vaudit_app_launch_stime;

#define RSC_VAUDIT_SVP_THREAD_ALL 0x1
#define RSC_VAUDIT_SVP_THREAD_60FPS_ONLY 0x2
#define RSC_VAUDIT_SVP_THREAD_40FPS_ONLY 0x4
#define RSC_VAUDIT_SVP_THREAD_30FPS_ONLY 0x8
#define RSC_VAUDIT_SVP_THREAD_25FPS_ONLY 0x10
extern u32 rsc_vaudit_svp_thread;
extern unsigned long rsc_vaudit_backtrace_threshold ;
#endif
