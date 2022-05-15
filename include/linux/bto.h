/*
 * 2019, Yang Yang
 */

#ifndef _LINUX_BTO_H
#define _LINUX_BTO_H

#ifdef CONFIG_BLK_ENHANCEMENT

#include <linux/timer.h>
#include <linux/sched.h>


#ifdef CONFIG_BLK_WBT
#define BLK_ENHANCEMENT_WBT		1
#endif
//#define BLK_ENHANCEMENT_SCMD		1
//#define BLK_ENHANCEMENT_MQ_ATOMIC	1
//#define BLK_ENHANCEMENT_USE_TASK	1

struct bto {
#ifdef BLK_ENHANCEMENT_USE_TASK
	struct task_struct		*task;
#else
	char		comm[TASK_COMM_LEN];
	char		leader[TASK_COMM_LEN];
	pid_t		pid;
	pid_t		gid;
#endif

	unsigned long long		queue_ns;
#ifdef BLK_ENHANCEMENT_WBT
	unsigned long long		wbtin_ns;
#endif
	unsigned long long		wbtout_ns;
	unsigned long long		getrq_ns;
	unsigned long long		insert_ns;
	unsigned long long		dequeue_ns;
	unsigned long long		issue_ns;
#ifdef BLK_ENHANCEMENT_SCMD
	unsigned long long		scmd_ns;
#endif
	unsigned long long		endio_ns;
	unsigned long long		tagin_ns;
	unsigned long long		tagout_ns;
	unsigned long long		flush_start_ns;
	unsigned long long		flush_end_ns;
	unsigned long long		softin_ns;
	unsigned long long		softout_ns;
	unsigned long long		qcmd_ns;
	unsigned long long		compl_ns;

	unsigned int		t_flag;
	unsigned long		nivcsw[3];

	int		cgroup;
	int		priority[2];
	int		cpu[2];
	int		inflight[2];
	int		nr_rqs[2];

	sector_t	n_sector;
};

struct rq_info {
	unsigned int		nr_rqs[2];
	unsigned int		inflight[2];

	unsigned int		requeue_count;

	unsigned long long		rq_get_time;
	unsigned long long		rq_insert_time;
	unsigned long long		rq_dequeue_time;
	unsigned long long		rq_requeue_time;
	unsigned long long		rq_issue_time;
	unsigned long long		rq_softin_time;
	unsigned long long		rq_softout_time;
	unsigned long long		rq_qcmd_time;
	unsigned long long		rq_compl_time;
	unsigned long long		rq_abort_time;
};
#endif

#endif /*  _LINUX_BTO_H */
