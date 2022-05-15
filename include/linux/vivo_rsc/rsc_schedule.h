#ifndef __RSC_SCHEDULE_H__
#define __RSC_SCHEDULE_H__

/*
#include "../../../kernel/sched/sched.h"
struct sched_entity;
*/
#include <linux/cpuset.h>
#ifdef CONFIG_RSC_LOCK_BOOST
#include <linux/vivo_rsc/rsc_test_svp.h>
#endif

/* Future-safe accessor for struct task_struct's cpus_allowed. */
#ifndef tsk_cpus_allowed
#define tsk_cpus_allowed(tsk) (&(tsk)->cpus_allowed)
#endif

#define S2US_TIME 1000000
extern int svp_min_sched_delay_granularity;/*svp thread delay upper bound(ms)*/
/*extern int svp_max_dynamic_granularity ;*//*svp dynamic max exist time(ms)*/
extern int svp_min_migration_delay;/*svp min migration delay time(ms)*/

extern struct cpumask rsc_cpu_bigcore;

static inline int entity_before_int(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

extern int rsc_do_printk;

static inline void do_heavytask_enqueue_check(struct rq *rq, struct task_struct *p, int rsc_svp, int quick_restore)
{
	/* state is TASK_RUNNING, it means that task is preempt or migrated */
	/*TASK_ON_RQ_QUEUED or TASK_ON_RQ_MIGRATING means that p is in doing migration*/
	/*
	p->on_rq == TASK_ON_RQ_MIGRATING, task_on_rq_migrating
	enqueue_task_fair
	activate_task
	load_balance
	pick_next_task_fair
	__schedule
	schedule
	worker_thread
	kthread
	cpu_switch_to

	p->on_rq == TASK_ON_RQ_QUEUED, task_on_rq_queued
	enqueue_task_fair
	set_user_nice
	binder_do_set_priority
	binder_transaction
	binder_thread_write
	binder_ioctl_write_read.isra.46.constprop.59
	binder_ioctl
	do_vfs_ioctl
	SyS_ioctl
	cpu_switch_to

	in_task1 on_rq2 state0
	CPU: 0 PID: 13 Comm: migration/0
	enqueue_task_fair
	move_queued_task
	__migrate_task
	migration_cpu_stop
	cpu_stopper_thread
	smpboot_thread_fn
	kthread
	cpu_switch_to

	in_task1 on_rq1 state0
	CPU: 0 PID: 9627 Comm: adbd
	enqueue_task_fair
	sched_move_task
	autogroup_move_group
	sched_autogroup_create_attach
	sys_setsid
	cpu_switch_to

	in_task1 on_rq1 state0
	CPU: 3 PID: 388 Comm: kworker/u16:6
	enqueue_task_fair
	rt_mutex_setprio
	rt_mutex_adjust_prio
	rt_mutex_unlock
	update_request_adhoc
	msm_bus_scale_client_update_request
	devbw_target
	update_devfreq
	devfreq_monitor
	process_one_work
	worker_thread
	kthread
	cpu_switch_to

	in_irq0 in_softirq1 in_task0 on_rq2 state0
	CPU: 6 PID: 0 Comm: swapper/6
	uid 	0	   rcu_preempt	   7
	enqueue_task_fair
	activate_task
	load_balance
	rebalance_domains
	run_rebalance_domains
	__do_softirq
	irq_exit
	scheduler_ipi
	handle_IPI
	gic_handle_irq

	in_task1 on_rq1 state0
	CPU: 0 PID: 9616 Comm: shell svc 9615
	dump_backtrace
	show_stack
	dump_stack
	enqueue_task_fair
	sched_exit
	do_exit
	do_group_exit
	cpu_switch_to

	in_task1 on_rq2 state0
	CPU: 2 PID: 2056 Comm: ndroid.systemui
	enqueue_task_fair
	activate_task
	load_balance
	pick_next_task_fair
	__schedule
	schedule
	futex_wait_queue_me
	futex_wait
	do_futex
	SyS_futex
	cpu_switch_to
	*/

	/*
	p->on_rq is zero, it means that task is from sleeping(or uninterrpt sleeping) to runnable.
	We can't use p->state for judgment, because that when new task fork,
	task state is TASK_RUNNING, but task is not on queue, see the example:
	in_task1 on_rq0 p->on_rq == 0, state0 p->state == TASK_RUNNING
	CPU: 2 PID: 9627 Comm: dumpsys
	enqueue_task_fair
	activate_task
	wake_up_new_task
	_do_fork
	SyS_clone
	cpu_switch_to
	*/

	if (quick_restore == 1)
		goto doclear;
	else if (quick_restore == 0) {
		if (!p->on_rq/*(p->state != TASK_RUNNING)*/ || !p->enqueue_time_lock) {
			/*p->enqueue_time_lock = rq->clock/S2US_TIME;*/
			/*p->enqueue_time_lock = rq->clock>>20;*/
			p->enqueue_time_lock = p->se.sum_exec_runtime>>20;
			/*goto doprint;*/
		}
	}
#if 0
	if (task_uid(p).val == 0 && (!strncmp("cat", p->comm, TASK_COMM_LEN)))
		printk_deferred("RSC vmigrate preempt RUNNING enqueu uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) svpcnt: %d "
			"cpuset: %*pbl in_irq%d in_softirq%d in_task%d on_rq%d state%lx ccaller: %ps pcaller: %ps enqueue_time_lock:%u ms exe:%llu ms diff: %llu ms\n",
			task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
			task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
			atomic_read(&rsc_svp_task_cnt),
			cpumask_pr_args(tsk_cpus_allowed(p)), !!in_irq(), !!in_softirq(), !!in_task(), p->on_rq, p->state,
			(void *)get_wchan(current), (void *)get_wchan(p), p->enqueue_time_lock, p->se.sum_exec_runtime>>20, ((p->se.sum_exec_runtime>>20) - p->enqueue_time_lock));
#endif
#if 0
	if (p->state == TASK_RUNNING) {
doprint:
		if (!svp_limit_sched_setaffinity) {
			rsc_do_printk = 0;
			/* could not to use printk or deadlock. */
			printk_deferred("RSC vmigrate preempt RUNNING enqueu uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) svpcnt: %d "
				"cpuset: %*pbl in_irq%d in_softirq%d in_task%d on_rq%d state%lx ccaller: %ps pcaller: %ps\n",
				task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
				task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
				atomic_read(&rsc_svp_task_cnt),
				cpumask_pr_args(tsk_cpus_allowed(p)), !!in_irq(), !!in_softirq(), !!in_task(), p->on_rq, p->state, (void *)get_wchan(current), (void *)get_wchan(p));
			/* could not to use printk or deadlock. Use rsc_do_printk to limit printk in dump_stack*/
			dump_stack();
			rsc_do_printk = 1;
		}
	}
#endif

	if (unlikely(rsc_svp & RSC_SVP_MANUAL_ABNORMAL_TASK)) {
#if 0
		bool quick_restore = false;

		if (cpumask_test_cpu(rq->cpu, &rsc_cpu_bigcore) && (p->on_rq == TASK_ON_RQ_MIGRATING))
			printk_deferred("RSC vmigrate bigcore RUNNING enqueu uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) svpcnt: %d "
				"cpuset: %*pbl in_irq%d in_softirq%d in_task%d on_rq%d state%lx ccaller: %ps pcaller: %ps cpu: %d\n",
				task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
				task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
				atomic_read(&rsc_svp_task_cnt),
				cpumask_pr_args(tsk_cpus_allowed(p)), !!in_irq(), !!in_softirq(), !!in_task(), p->on_rq, p->state,
				(void *)get_wchan(current), (void *)get_wchan(p), rq->cpu);
#endif
#if 0
		if (unlikely(((p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER) ||
			(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT)) && p->heavytask_limit_time &&
			(p->rsc_heavytask_trycnt != HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP) && (rq->clock > (((u64)(p->heavytask_limit_time + heavytask_limit_intervaltime*(p->rsc_heavytask_trycnt&0xf)) << 20/** S2US_TIME*/))))) {
			if (!atomic_read(&rsc_svp_task_cnt) && (!(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT))) {
				/* 5 x 64ms*/
				if ((p->rsc_heavytask_trycnt&0xf0) > 0x30) {
					quick_restore = true;
					goto doclear;
				}
				if (((p->rsc_heavytask_trycnt & 0xf) == 0xf) || ((p->rsc_heavytask_trycnt & 0xf0) == 0xf0))
					p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
				else
					p->rsc_heavytask_trycnt += 0x11;
				printk_deferred("RSC vmigrate abntask recheck cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
						"cpuset: %*pbl cost %llu ms cpu%d trycnt: %x warning\n",
						task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
						atomic_read(&rsc_svp_task_cnt),
						cpumask_pr_args(tsk_cpus_allowed(p)),
						(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, rq->cpu, p->rsc_heavytask_trycnt);
			} else {
				/* 3 x 64ms*/
				if ((p->rsc_heavytask_trycnt&0xf) > 0x2) {
					struct cpumask allowed_mask;

					if (!(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_NOTDO)) {
						cpumask_and(&allowed_mask, cpu_active_mask, &rsc_cpu_littlecore);
						if (!cpumask_empty(&allowed_mask) && !cpumask_equal(&allowed_mask, &p->cpus_allowed))
							set_cpus_allowed_common(p, &allowed_mask);
					}
					p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
				} else {
					if ((p->rsc_heavytask_trycnt & 0xf) == 0xf)
						p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
					else
						p->rsc_heavytask_trycnt += 0x1;
				}
				printk_deferred("RSC vmigrate abntask recheck cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
						"cpuset: %*pbl cost %llu ms cpu%d trycnt: %x okok\n",
						task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
						atomic_read(&rsc_svp_task_cnt),
						cpumask_pr_args(tsk_cpus_allowed(p)),
						(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, rq->cpu, p->rsc_heavytask_trycnt);
			}
		}
#endif
/*
		if (heavytask_limit_to_litcore == 3)
			printk_deferred("RSC vmigrate abntask restore test cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
				"cpuset: %*pbl all %*pbl cost %llu ms trycnt: %x qrs%d cpu%d limit: %u ms rq->clock: %llu ms g%d\n",
				task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
				atomic_read(&rsc_svp_task_cnt),
				cpumask_pr_args(tsk_cpus_allowed(p)),  cpumask_pr_args(&rsc_cpu_all),
				(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, p->rsc_heavytask_trycnt, quick_restore,
				rq->cpu, p->heavytask_limit_time, rq->clock>>20, rsc_get_task_group(p->group_leader));
*/
		if (unlikely(p->heavytask_limit_time &&
			(rq->clock > (((u64)(p->heavytask_limit_time + heavytask_limit_maxtime)) << 20/** S2US_TIME*/)))) {
doclear:
			if (RSC_GROUP_FG == rsc_get_task_group(p->group_leader) || (RSC_GROUP_DEFAULT == rsc_get_task_group(p->group_leader))) {
				struct cpumask allowed_mask;

				cpumask_copy(&allowed_mask, cpu_active_mask);
				printk_deferred("RSC vmigrate abntask restore cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
					"cpuset: %*pbl ->  %*pbl all %*pbl cost %llu ms trycnt: %x qrs%d cpu%d\n",
					task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
					atomic_read(&rsc_svp_task_cnt),
					cpumask_pr_args(tsk_cpus_allowed(p)), cpumask_pr_args(&allowed_mask),  cpumask_pr_args(&rsc_cpu_all),
					(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, p->rsc_heavytask_trycnt, quick_restore, rq->cpu);

				/*
				* reset it before set_cpus_allowed_ptr_force,
				* in case of reentry. scheduler_tick ->
				* set_cpus_allowed_ptr_force -> rsc_limit_active_heavytask ->
				* fifo task [migration/6] or [migration/7] -> ... -> enqueue_svp_task ->
				* rsc_do_heavytask_check
				*/
				/*recheck 1s later*/
				if (!cpumask_equal(&rsc_cpu_all, &allowed_mask))
					p->heavytask_limit_time = ((rq->clock+1000000000UL)>>20)-heavytask_limit_maxtime;
				else {
					clear_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK|RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE);
					p->heavytask_limit_time = 0;/*rq->clock>>20;rq->clock/S2US_TIME;*/
				}
				if (!cpumask_equal(&allowed_mask, &p->cpus_allowed) && !cpumask_empty(&allowed_mask)) {
					if (quick_restore)
						set_cpus_allowed_ptr_force(p, &allowed_mask);
					else
						set_cpus_allowed_common(p, &allowed_mask);
				}
			}
		}
	}
}

static inline void rsc_do_heavytask_check(struct rq *rq, struct task_struct *p)
{
	if (unlikely(p->rsc_svp & RSC_SVP_MANUAL_ABNORMAL_TASK)) {
		bool quick_restore = false;
#if 0
		if (cpumask_test_cpu(rq->cpu, &rsc_cpu_bigcore) && (p->on_rq == TASK_ON_RQ_MIGRATING))
			printk_deferred("RSC vmigrate bigcore RUNNING enqueu uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) svpcnt: %d "
				"cpuset: %*pbl in_irq%d in_softirq%d in_task%d on_rq%d state%lx ccaller: %ps pcaller: %ps cpu: %d\n",
				task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
				task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
				atomic_read(&rsc_svp_task_cnt),
				cpumask_pr_args(tsk_cpus_allowed(p)), !!in_irq(), !!in_softirq(), !!in_task(), p->on_rq, p->state,
				(void *)get_wchan(current), (void *)get_wchan(p), rq->cpu);
#endif
		if (unlikely(((p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER) ||
			(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT)) && p->heavytask_limit_time &&
			(p->rsc_heavytask_trycnt != HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP) &&
			(rq->clock > (((u64)(p->heavytask_limit_time + heavytask_limit_intervaltime*(p->rsc_heavytask_trycnt&0xf)) << 20/** S2US_TIME*/))))) {
			if (!atomic_read(&rsc_svp_task_cnt) && (!(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT))) {
				/* 5 x 64ms*/
				if ((p->rsc_heavytask_trycnt&0xf0) > 0x30) {
					quick_restore = true;
					//goto doclear;
					do_heavytask_enqueue_check(rq, p, p->rsc_svp, 1);
					return;
				}
				if (((p->rsc_heavytask_trycnt & 0xf) == 0xf) || ((p->rsc_heavytask_trycnt & 0xf0) == 0xf0))
					p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
				else
					p->rsc_heavytask_trycnt += 0x11;
				printk_deferred("RSC vmigrate abntask recheck cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
						"cpuset: %*pbl cost %llu ms cpu%d trycnt: %x warning\n",
						task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
						atomic_read(&rsc_svp_task_cnt),
						cpumask_pr_args(tsk_cpus_allowed(p)),
						(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, rq->cpu, p->rsc_heavytask_trycnt);
			} else {
				/* 3 x 64ms*/
				if ((p->rsc_heavytask_trycnt&0xf) > 0x2) {
					struct cpumask allowed_mask;

					if (!(heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_NOTDO)) {
						cpumask_and(&allowed_mask, cpu_active_mask, &rsc_cpu_littlecore);
						if (!cpumask_empty(&allowed_mask) && !cpumask_equal(&allowed_mask, &p->cpus_allowed)) {
							/*set_cpus_allowed_common(p, &allowed_mask);*/
							set_cpus_allowed_ptr_force(p, &allowed_mask);
						}
					}
					p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
				} else {
					if ((p->rsc_heavytask_trycnt & 0xf) == 0xf)
						p->rsc_heavytask_trycnt = HEAVYTASK_LIMIT_TO_LITCORE_RECHECK_STOP;
					else
						p->rsc_heavytask_trycnt += 0x1;
				}
				printk_deferred("RSC vmigrate abntask recheck cpuset uid %5d %16s %5d(%16s %5d) svpcnt: %d "
						"cpuset: %*pbl cost %llu ms cpu%d trycnt: %x okok\n",
						task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
						atomic_read(&rsc_svp_task_cnt),
						cpumask_pr_args(tsk_cpus_allowed(p)),
						(rq->clock - ((u64)p->heavytask_limit_time<<20))/S2US_TIME, rq->cpu, p->rsc_heavytask_trycnt);
			}
		} else {
			/*restore heavytask cpuset*/
			do_heavytask_enqueue_check(rq, p, p->rsc_svp, 2);
		}
	}
}

#define RSC_COLLECT_MAINTHREAD
static inline void enqueue_svp_task(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
	bool exist = false;
	int rsc_svp;

	if (!rq || !p || !list_empty(&p->svp_entry)) {
		return;
	}

	p->enqueue_time = rq->clock;

	rsc_svp = p->rsc_svp;
/*
	if (rsc_svp & RSC_SVP_MANUAL) {
		if (RSC_SVP_MANUAL_VAL != RSC_SVP_MANUAL) {
			set_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
			atomic_inc(&rsc_svp_task_cnt);
		} else
			goto enq_now;
	} else
*/

	//CONFIG_RSC_LOCK_BOOST_IMPROVE
	if ((rsc_svp & RSC_SVP_PREEMPT)
#ifdef CONFIG_RSC_LOCK_BOOST
		|| atomic64_read(&p->dynamic_svp)
#endif
	) {
//enq_now:
		if (1/*svp_is_fg_task(p)*/) {
			list_for_each_safe(pos, n, &rq->svp_task_list) {
				if (pos == &p->svp_entry) {
					exist = true;
					break;
				}
			}
			if (!exist) {
				/*
				if (unlikely(rq->svp_manual_cnt >= ((typeof(rq->svp_manual_cnt))~0U)))
					printk_deferred("svp_cnt too more!! uid %5d %16s %5d(%16s %5d)\n",
						task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid);
				*/
				if (likely(dynamic_svp_enqueue_enable)) {
					//CONFIG_RSC_LOCK_BOOST_IMPROVE
					cpumask_set_cpu(rq->cpu, &rsc_svp_run_cpus);
#ifdef RSC_COLLECT_MAINTHREAD
					atomic_inc(&rsc_svp_task_cnt);
					rq->svp_manual_cnt++;
#else
					if (likely(rsc_svp & RSC_SVP_MANUAL)) {
						atomic_inc(&rsc_svp_task_cnt);
						set_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
						rq->svp_manual_cnt++;
					}
#endif
					list_add_tail(&p->svp_entry, &rq->svp_task_list);
					get_task_struct(p);
				}
				trace_sched_svp_queue_op(p, "svp_enqueue");
			}
		}
	} else if (rsc_svp & RSC_SVP_MANUAL) {
		//CONFIG_RSC_LOCK_BOOST_IMPROVE
		set_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
		atomic_inc(&rsc_svp_task_cnt);
	} else {
		do_heavytask_enqueue_check(rq, p, rsc_svp, 0);
	}
}

typedef enum {
	RSC_NOT_DO_MIG_AND_SET_NOT_FORCE = 0,
	RSC_NOT_DO_MIG_AND_NOT_SET_NOT_FORCE,
	RSC_FORCE_DO_MIG_AND_NOT_SET_NOT_FORCE,
} rsc_migration_type;

static inline bool iam_rt_policy(int policy)
{
	return policy == SCHED_FIFO || policy == SCHED_RR;
}

static inline bool iam_fair_policy(int policy)
{
	return policy == SCHED_NORMAL || policy == SCHED_BATCH;
}

static inline int rsc_do_heavytask_limit(struct rq *rq, struct task_struct *p, rsc_migration_type do_migrate)
{
	int do_limit = 0;
#if 0
	if (task_uid(p).val == 0 && (!strncmp("cat", p->comm, TASK_COMM_LEN)))
		printk_deferred("RSC vmigrate preempt RUNNING dequeu uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) svpcnt: %d "
			"cpuset: %*pbl in_irq%d in_softirq%d in_task%d on_rq%d state%lx ccaller: %ps pcaller: %ps enqueue_time_lock:%u ms exe:%llu ms diff: %llu ms\n",
			task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
			task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
			atomic_read(&rsc_svp_task_cnt),
			cpumask_pr_args(tsk_cpus_allowed(p)), !!in_irq(), !!in_softirq(), !!in_task(), p->on_rq, p->state,
			(void *)get_wchan(current), (void *)get_wchan(p), p->enqueue_time_lock, p->se.sum_exec_runtime>>20, ((p->se.sum_exec_runtime>>20) - p->enqueue_time_lock));

#endif
	if ((do_migrate == RSC_FORCE_DO_MIG_AND_NOT_SET_NOT_FORCE) &&
		(unlikely((p->rsc_svp & (RSC_SVP_MANUAL_ABNORMAL_TASK|RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE)) ==
		(RSC_SVP_MANUAL_ABNORMAL_TASK|RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE)))) {
		clear_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE);
		set_cpus_allowed_ptr_force(p, &rsc_cpu_littlecore);
		rcu_read_lock();
		printk_deferred("RSC vmigrate abntask running  toolong uid %5d %16s %5d(%16s %5d) father: %16s %5d(%16s %5d) svpcnt: %d "
			"workaround cpuset: %*pbl ->  %*pbl cpu%d do_migrate%d scheduler_tick\n",
			task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
			p->real_parent->comm, p->real_parent->pid,	p->parent->comm, p->parent->pid,
			atomic_read(&rsc_svp_task_cnt),
			cpumask_pr_args(tsk_cpus_allowed(p)), cpumask_pr_args(&rsc_cpu_littlecore), rq->cpu, do_migrate);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return 1;
	}

	/*limit the heavy task of HOK*/
	if (svp_is_fg_task(p) && (rsc_svp_set_count >= HEAVYTASK_LIMIT_SVP_MIN_CNT) &&
		((heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_SVP) ||
		((heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_HOK) &&
		unlikely(p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER))) &&
		unlikely(/*rq->clock*/p->se.sum_exec_runtime > (((u64)(p->enqueue_time_lock + svp_enqueue_overtime)) << 20/* S2US_TIME*/)) &&
		p->enqueue_time_lock && !(p->flags & PF_NO_SETAFFINITY)
		) {
		struct cpumask allowed_mask = CPU_MASK_NONE;
		bool is_thread = false;
		bool docpuset;
		int uid = task_uid(p).val;

		/*only support RT task and fair task*/
		if (do_migrate == RSC_FORCE_DO_MIG_AND_NOT_SET_NOT_FORCE) {
			if (!iam_rt_policy(p->policy) && !iam_fair_policy(p->policy))
				return 1;
		}

		if (!(p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MANUAL_GROUP_LEADER | RSC_SVP_MAINTHREAD))) {
			/*
				we have get lock here.
				raw_spin_lock_irqsave(&p->pi_lock, rf->flags);
				raw_spin_lock(&rq->lock);
			*/
			/* workaround: limit heavy task to littlecore cpus*/

			rsc_debug_enqueue_overcnt++;
			p->heavytask_limit_time = rq->clock>>20;/*rq->clock/S2US_TIME;*/
			p->rsc_heavytask_trycnt = 1;
			if (do_migrate == RSC_NOT_DO_MIG_AND_SET_NOT_FORCE)
				set_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK|RSC_SVP_MANUAL_ABNORMAL_TASK_NOT_FORCE);
			else
				set_rsc_svp(p, RSC_SVP_MANUAL_ABNORMAL_TASK);

			/* heavy task is limit by sched_setaffinity*/
			if (!cpumask_equal(cpu_active_mask, &p->cpus_allowed)) {
				docpuset = true;
				rsc_debug_enqueue_bind_bigcore_cnt++;
				printk_deferred("RSC vmigrate abntask detect bind cpucore uid %5d %16s %5d(%16s %5d) father: %16s %5d(%16s %5d) svpcnt: %d "
					"cost %llu ms workaround cpuset: %*pbl xxx %*pbl\n",
					uid, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
					p->real_parent->comm, p->real_parent->pid,	p->parent->comm, p->parent->pid,
					atomic_read(&rsc_svp_task_cnt), (p->se.sum_exec_runtime/*rq->clock*/ - ((u64)p->enqueue_time_lock<<20))/S2US_TIME,
					cpumask_pr_args(tsk_cpus_allowed(p)), cpumask_pr_args(cpu_active_mask));
			} else
				docpuset = false;

			if ((p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER) && !strncmp("Thread-", p->comm, 7)) {
				is_thread = true;
				rsc_debug_enqueue_thread_overcnt++;
			} else {
				int i;
				int match = 0;
				int cnt = min(rsc_debug_enqueue_otherthread_actcnt, (u32)RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX);

				for (i = 0; i < cnt; i++)
					if ((rsc_debug_enqueue_heavytask_uid[i] == uid) &&
						!strncmp(rsc_debug_enqueue_otherthread[i], p->comm, TASK_COMM_LEN)) {
						match = 1;
						break;
					}

				if (!match) {
					if (cnt < RSC_DEBUG_ENQUEUE_OTHERTHREAD_MAX) {
						strlcpy(rsc_debug_enqueue_otherthread[cnt], p->comm, TASK_COMM_LEN);
						if (thread_group_leader(p)) {
							struct task_struct *rp;

							rp = p->real_parent;
							rcu_read_lock();
							if (!strncmp("sh", rp->comm, TASK_COMM_LEN) && (rp->real_parent)) {
								rp = rp->real_parent;
								if (!strncmp("Binder:", rp->comm, 7) && (rp->group_leader))
									rp = rp->group_leader;
							} else if (!strncmp("Binder:", rp->comm, 7) && (rp->group_leader))/*(Binder:583_2)*/
								rp = rp->group_leader;
							strlcpy(rsc_debug_enqueue_otherthread_gl[cnt], rp->comm, TASK_COMM_LEN);
							rsc_debug_enqueue_heavytask_glpid[cnt] = rp->pid;
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
							rcu_read_unlock_inline();
#else
							rcu_read_unlock();
#endif
						} else {
							strlcpy(rsc_debug_enqueue_otherthread_gl[cnt], p->group_leader->comm, TASK_COMM_LEN);
							rsc_debug_enqueue_heavytask_glpid[cnt] = p->group_leader->pid;
						}
						rsc_debug_enqueue_heavytask_uid[cnt] = uid;
						rsc_debug_enqueue_heavytask_pid[cnt] = p->pid;
						rsc_debug_enqueue_otherthread_actcnt++;
						rsc_debug_enqueue_otherthread_cnt[cnt]++;
					}
				} else {
					rsc_debug_enqueue_otherthread_cnt[i]++;
					if (thread_group_leader(p)) {
						struct task_struct *rp;

						rp = p->real_parent;
						rcu_read_lock();
						if (!strncmp("sh", rp->comm, TASK_COMM_LEN) && (rp->real_parent)) {
							rp = rp->real_parent;
							if (!strncmp("Binder:", rp->comm, 7) && (rp->group_leader))
								rp = rp->group_leader;
						} else if (!strncmp("Binder:", rp->comm, 7) && (rp->group_leader))/*(Binder:583_2)*/
							rp = rp->group_leader;
						strlcpy(rsc_debug_enqueue_otherthread_gl[i], rp->comm, TASK_COMM_LEN);
						rsc_debug_enqueue_heavytask_glpid[i] = rp->pid;
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
						rcu_read_unlock_inline();
#else
						rcu_read_unlock();
#endif
					} else {
						strlcpy(rsc_debug_enqueue_otherthread_gl[i], p->group_leader->comm, TASK_COMM_LEN);
						rsc_debug_enqueue_heavytask_glpid[i] = p->group_leader->pid;
					}
					rsc_debug_enqueue_heavytask_pid[i] = p->pid;
				}

				rsc_debug_enqueue_otherthread_totalcnt++;
			}
			/* limit except HOK and system_server*/
			if ((heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_ALLTASK_LIMIT) ||
				((heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_THREADTASK_LIMIT) && is_thread) ||
				((heavytask_limit_to_litcore & HEAVYTASK_LIMIT_TO_LITCORE_SLIM) &&
				((!(p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER) && !(task_rsc_alloc(p->group_leader) == &rsc_system_server_time)) ||
				((task_rsc_alloc(p->group_leader) == &rsc_system_server_time) && (!strncmp("android.bg", p->comm, TASK_COMM_LEN))) ||
				((p->group_leader->rsc_svp & RSC_SVP_MANUAL_GROUP_LEADER) && atomic_read(&rsc_svp_task_cnt))))
				) {
				cpumask_and(&allowed_mask, cpu_active_mask, &rsc_cpu_littlecore);
				docpuset = true;
			} else {
				cpumask_copy(&allowed_mask, cpu_active_mask);
			}

			if (1/*unlikely(rsc_debug & RSC_SVP_TASK_SCHEDULE)*/) {

				rcu_read_lock();
				printk_deferred("RSC vmigrate abntask running  toolong uid %5d %16s %5d(%16s %5d) father: %16s %5d(%16s %5d) svpcnt: %d "
					"cost %llu ms workaround cpuset: %*pbl ->  %*pbl cpu%d mig%d\n",
					uid, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
					p->real_parent->comm, p->real_parent->pid,	p->parent->comm, p->parent->pid,
					atomic_read(&rsc_svp_task_cnt), (p->se.sum_exec_runtime/*rq->clock*/ - ((u64)p->enqueue_time_lock<<20))/S2US_TIME,
					cpumask_pr_args(tsk_cpus_allowed(p)), cpumask_pr_args(&allowed_mask), rq->cpu, do_migrate);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
				rcu_read_unlock_inline();
#else
				rcu_read_unlock();
#endif
			}
			/*
			* reset it before set_cpus_allowed_ptr_force,
			* in case of reentry. scheduler_tick ->
			* set_cpus_allowed_ptr_force -> rsc_limit_active_heavytask ->
			* fifo task [migration/6] or [migration/7] -> ... -> dequeue_svp_task ->
			* rsc_do_heavytask_limit
			*/
			p->enqueue_time_lock = 0;
			if (docpuset && !cpumask_empty(&allowed_mask)) {
				if (do_migrate == RSC_FORCE_DO_MIG_AND_NOT_SET_NOT_FORCE) {
					/*
					* Could not use set_cpus_allowed_ptr(p, &allowed_mask),
					* because call from scheduler_tick(interrupt)
					*/
					set_cpus_allowed_ptr_force(p, &allowed_mask);
				} else
					set_cpus_allowed_common(p, &allowed_mask);
				do_limit = 1;
			}
		} else
			p->enqueue_time_lock = 0;
	}

	return do_limit;
}

static inline void dequeue_svp_task(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
#ifdef CONFIG_RSC_LOCK_BOOST
	u64 now = rsc_get_clock();
#endif

	if (!rq || !p) {
		return;
	}

	p->enqueue_time = 0;
	if (!list_empty(&p->svp_entry)) {
#ifdef CONFIG_RSC_LOCK_BOOST
		u64 dynamic_svp = atomic64_read(&p->dynamic_svp);

		if (dynamic_svp &&
			((now - p->dynamic_svp_start) > (u64)svp_max_dynamic_granularity * S2US_TIME)) {
			int type;

			for (type = 0; type < DYNAMIC_SVP_MAX; type++) {
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
				if (type < DYNAMIC_SVP_KERNEL_LOCK_BEGIN)
					p->dynamic_svp_ulock_pid[type] = 0;
				else if (type < DYNAMIC_SVP_LOCK_MAX)
					p->dynamic_svp_klock_chain_pid_addr[type - DYNAMIC_SVP_KERNEL_LOCK_BEGIN] = NULL;
#endif
				if (dynamic_svp & dynamic_svp_mask_of(type))
					dynamic_svp_timeout_deq_count[type]++;
			}
			atomic64_set(&p->dynamic_svp, 0);
			p->svp_depth = 0;
		}
#endif
		list_for_each_safe(pos, n, &rq->svp_task_list) {
			if (pos == &p->svp_entry) {
#ifdef RSC_COLLECT_MAINTHREAD
				atomic_dec(&rsc_svp_task_cnt);
				rq->svp_manual_cnt--;
				/*some task has dynamic svp and static svp*/
				if (p->rsc_svp & RSC_SVP_MANUAL_HAS_SET)
					clear_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
#else
				if (p->rsc_svp & RSC_SVP_MANUAL_HAS_SET) {
					atomic_dec(&rsc_svp_task_cnt);
					clear_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
					rq->svp_manual_cnt--;
				}
#endif
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
				if (unlikely(dynamic_svp & dynamic_svp_mask_of(DYNAMIC_SVP_CFS_FORCE))) {
					atomic64_set(&p->dynamic_svp, dynamic_svp & (~(dynamic_svp_mask_of(DYNAMIC_SVP_CFS_FORCE))));
					p->dynamic_svp_start =  now;
					p->svp_depth = 0;
				}
#endif
				list_del_init(&p->svp_entry);
				put_task_struct(p);
				//CONFIG_RSC_LOCK_BOOST_IMPROVE
				if (list_empty(&rq->svp_task_list))
					cpumask_clear_cpu(rq->cpu, &rsc_svp_run_cpus);
				trace_sched_svp_queue_op(p, "svp_dequeue");
				return;
			}
		}
		printk_deferred("RSC [WARNING] vmigrate svp not found in dequeue_svp_task. uid %5d %16s %5d(%16s %5d) "
			"father: %16s %5d svpcnt: %d - %u cpuset: %*pbl cpu%d\n",
			task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
			p->real_parent->comm, p->real_parent->pid,
			atomic_read(&rsc_svp_task_cnt), (u32)rq->svp_manual_cnt,
			cpumask_pr_args(tsk_cpus_allowed(p)), rq->cpu);
	} else if (p->rsc_svp & RSC_SVP_MANUAL_HAS_SET) {
			atomic_dec(&rsc_svp_task_cnt);
			//atomic_dec_if_positive(&rsc_svp_task_cnt);
			clear_rsc_svp(p, RSC_SVP_MANUAL_HAS_SET);
	} else {
		rsc_do_heavytask_limit(rq, p, RSC_NOT_DO_MIG_AND_SET_NOT_FORCE);
	}
}

static inline struct task_struct *pick_first_svp_task(struct rq *rq)
{
	struct list_head *svp_task_list = &rq->svp_task_list;
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct task_struct *temp = NULL;
	struct task_struct *leftmost_task = NULL;
	list_for_each_safe(pos, n, svp_task_list) {
		temp = list_entry(pos, struct task_struct, svp_entry);
		/*ensure svp task in current rq cpu otherwise delete it*/
		if (unlikely(task_cpu(temp) != rq->cpu)) {
			printk_deferred("RSC [WARNING] vmigrate task(%s,%d,%d) does not belong to cpu%d warning!\n",
				temp->comm, task_cpu(temp), temp->policy, rq->cpu);
			list_del_init(&temp->svp_entry);
			continue;
		}
		if (leftmost_task == NULL) {
			leftmost_task = temp;
		} else if (entity_before_int(&temp->se, &leftmost_task->se)) {
			leftmost_task = temp;
		}
	}

	return leftmost_task;
}

static inline void pick_svp_task(struct rq *rq, struct task_struct **p, struct sched_entity **se)
{
	struct task_struct *ori_p;
	struct task_struct *key_task;
	struct sched_entity *key_se;

#if 0
	if (!rq || !p || !se) {
		return;
	}
#endif
#if 0
/*CONFIG_RSC_LOCK_BOOST*/
	if (unlikely(time_before(jiffies, rq->svp_overload_jiffies))) {
		svp_max_cpu_usage_overload_jiffies_ignore_cnt++;
		goto out;
	}

	/* prevent svp task always occupy cpu, give 1/6 change to normal task*/
	if (unlikely(rq->svp_pick_cnt - rq->nor_pick_cnt >= svp_max_cpu_usage_diff_cnt)) {
		//if (unlikely(rsc_debug & RSC_SVP_TASK_SCHEDULE))
			printk_deferred("RSC vmigrate pick_svp_task overload svp_pick_cnt: %u diff: %d cpu%d svp_cnt:%d nr_running: %u cfs: %u jiffies ignore: %u %u\n",
				rq->svp_pick_cnt, rq->svp_pick_cnt - rq->nor_pick_cnt, rq->cpu, rq->svp_manual_cnt, rq->nr_running, rq->cfs.nr_running,
				svp_max_cpu_usage_overload_jiffies_ignore_cnt, svp_max_cpu_usage_overload_preempt_ignore_cnt);
		rq->svp_overload_jiffies = jiffies + msecs_to_jiffies(svp_max_cpu_usage_overload_delay_ms);
		svp_max_cpu_usage_overload_cnt++;
		goto out;
	}
#endif

	ori_p = *p;
	if (ori_p &&
			//CONFIG_RSC_LOCK_BOOST_IMPROVE
			!((ori_p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && rsc_top_task(ori_p))
			//!ori_p->rsc_svp
#ifdef CONFIG_RSC_LOCK_BOOST
			&& !atomic64_read(&ori_p->dynamic_svp)
#endif
		) {
		if (!list_empty(&rq->svp_task_list)) {
			key_task = pick_first_svp_task(rq);
			if (key_task) {
				key_se = &key_task->se;
				if (key_se && (rq->clock >= key_task->enqueue_time) &&
					rq->clock - key_task->enqueue_time >= ((u64)svp_min_sched_delay_granularity * S2US_TIME)) {
					trace_sched_svp_sched(key_task, "pick_svp");
					*p = key_task;
					*se = key_se;
					//CONFIG_RSC_LOCK_BOOST_IMPROVE
#ifdef CONFIG_RSC_LOCK_BOOST
					if (unlikely(key_task->rsc_svp & RSC_SVP_PREEMPT)) {
						rsc_svp_preempt_pickup_count++;
						clear_rsc_svp(key_task, RSC_SVP_PREEMPT);
						list_del_init(&key_task->svp_entry);
						atomic_dec(&rsc_svp_task_cnt);
						rq->svp_manual_cnt--;
						put_task_struct(key_task);
						if (list_empty(&rq->svp_task_list))
							cpumask_clear_cpu(rq->cpu, &rsc_svp_run_cpus);
						trace_sched_svp_queue_op(key_task, "svp_pick_dequeue");
					}
#endif
					/*
					if (rq->cfs.nr_running > rq->svp_manual_cnt)
						rq->svp_pick_cnt++;
					*/
					return;
				}
			}
		}
	}
#if 0
/*CONFIG_RSC_LOCK_BOOST*/
out:
	rq->nor_pick_cnt = rq->svp_pick_cnt;
#endif
}
#endif
