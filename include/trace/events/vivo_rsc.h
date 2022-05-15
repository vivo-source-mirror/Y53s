#undef TRACE_SYSTEM
#define TRACE_SYSTEM vivo_rsc

#if !defined(_TRACE_VIVO_RSC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VIVO_RSC_H
#include <linux/tracepoint.h>

#ifdef RSC_I_AM_IN_CPU_USAGE
#define RSC_MAX_CPU	8
TRACE_EVENT(rsc_cpu_usage,

	TP_PROTO(u32 *notidle, int num, u32 period),

	TP_ARGS(notidle, num, period),

	TP_STRUCT__entry(
		__field(int, num);
		__field(u32, period);
		__array(u32, notidle, RSC_MAX_CPU);
	),

	TP_fast_assign(
		__entry->num = num;
		__entry->period = period;
		if (RSC_MAX_CPU > num)
			memset(__entry->notidle + num,
				0, sizeof(u32) * (RSC_MAX_CPU - num));
		else
			num = min(RSC_MAX_CPU, num);

		memcpy(__entry->notidle, notidle,
			num * sizeof(u32));
	),

	TP_printk("%u\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%u\t%u",
		__entry->period,
		__entry->num,
		__entry->notidle[0],
		__entry->notidle[1],
		__entry->notidle[2],
		__entry->notidle[3],
		__entry->notidle[4],
		__entry->notidle[5],
		__entry->notidle[6],
		__entry->notidle[7],
		rsc_cpu_freqs[0].new,
		rsc_cpu_freqs[1].new
		)
);

TRACE_EVENT(rsc_cpu_usaged,

	TP_PROTO(u32 *notidle, u32 *user, u32 *hirq,
			u32 *sirq, int num, u32 period),

	TP_ARGS(notidle, user, hirq, sirq, num, period),
	TP_STRUCT__entry(
		__field(int, num);
		__field(u32, period);
		__array(u32, notidle, RSC_MAX_CPU);
		__array(u32, user, RSC_MAX_CPU);
		__array(u32, hirq, RSC_MAX_CPU);
		__array(u32, sirq, RSC_MAX_CPU);
	),

	TP_fast_assign(
		__entry->num = num;
		__entry->period = period;
		if (RSC_MAX_CPU > num) {
			memset(__entry->notidle + num,
				0, sizeof(u32) * (RSC_MAX_CPU - num));
			memset(__entry->user + num,
				0, sizeof(u32) * (RSC_MAX_CPU - num));
			memset(__entry->hirq + num,
				0, sizeof(u32) * (RSC_MAX_CPU - num));
			memset(__entry->sirq + num,
				0, sizeof(u32) * (RSC_MAX_CPU - num));
		} else
			num = min(RSC_MAX_CPU, num);

		memcpy(__entry->notidle, notidle,
			num * sizeof(u32));
		memcpy(__entry->user, user,
			num * sizeof(u32));
		memcpy(__entry->hirq, hirq,
			num * sizeof(u32));
		memcpy(__entry->sirq, sirq,
			num * sizeof(u32));
	),

	TP_printk("%u\t%d\t0\t%d\t%d\t%d\t%d\t1\t%d\t%d\t%d\t%d\t2\t%d\t%d\t%d\t%d"
					"\t3\t%d\t%d\t%d\t%d\t4\t%d\t%d\t%d\t%d\t5\t%d\t%d\t%d\t%d"
					"\t6\t%d\t%d\t%d\t%d\t7\t%d\t%d\t%d\t%d\t%u\t%u",
		__entry->period,
		__entry->num,
		__entry->notidle[0], __entry->user[0], __entry->hirq[0], __entry->sirq[0],
		__entry->notidle[1], __entry->user[1], __entry->hirq[1], __entry->sirq[1],
		__entry->notidle[2], __entry->user[2], __entry->hirq[2], __entry->sirq[2],
		__entry->notidle[3], __entry->user[3], __entry->hirq[3], __entry->sirq[3],
		__entry->notidle[4], __entry->user[4], __entry->hirq[4], __entry->sirq[4],
		__entry->notidle[5], __entry->user[5], __entry->hirq[5], __entry->sirq[5],
		__entry->notidle[6], __entry->user[6], __entry->hirq[6], __entry->sirq[6],
		__entry->notidle[7], __entry->user[7], __entry->hirq[7], __entry->sirq[7],
		rsc_cpu_freqs[0].new,
		rsc_cpu_freqs[1].new
		)
);

#else

TRACE_EVENT(ktop_mon_start,

	TP_PROTO(int cpu),

	TP_ARGS(cpu),

	TP_STRUCT__entry(
		__field(int, cpu);
	),

	TP_fast_assign(
		__entry->cpu = cpu;
	),

	TP_printk("\tcpu%d",
		__entry->cpu
		)
);


TRACE_EVENT(ktop_mon_end,

	TP_PROTO(int cpu, int isidle, u32 ticks),

	TP_ARGS(cpu, isidle, ticks),

	TP_STRUCT__entry(
		__field(int, cpu);
		__field(int, isidle);
		__field(u32, ticks);
		__field(int, pid);
		__array(char, comm, TASK_COMM_LEN);
	),

	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->isidle = isidle;
		__entry->ticks = ticks;
		__entry->pid = current->pid;
		memcpy(__entry->comm, current->comm, TASK_COMM_LEN);
	),

	TP_printk("\tcpu%d\tidl%d\ttick\t%d\t%16s\t%5d\n",
		__entry->cpu,
		__entry->isidle,
		__entry->ticks,
		__entry->comm,
		__entry->pid
		)
);
#endif
#endif /* _TRACE_VIVO_RSC_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
