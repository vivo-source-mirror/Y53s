#ifdef CONFIG_RSC_LOCK_BOOST
#ifndef __RSC_MUTEX__
#define __RSC_MUTEX__
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_test_svp.h>

static inline void mutex_list_add_svp(struct list_head *entry, struct list_head *head)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct mutex_waiter *waiter = NULL;
	list_for_each_safe(pos, n, head) {
		waiter = list_entry(pos, struct mutex_waiter, list);
		if (waiter->task && !test_task_svp_render_nocheck(waiter->task)) {
			list_add(entry, waiter->list.prev);
			return;
		}
	}
	if (pos == head) {
		list_add_tail(entry, head);
	}
}

static inline void mutex_dynamic_svp_enqueue(struct mutex *lock, struct task_struct *task)
{
	bool is_svp;
	struct task_struct *owner ;

	is_svp = test_task_svp_checkdepth(task);
	if (is_svp && !lock->svp_dep_task) {
		rcu_read_lock();
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		owner = __mutex_owner(lock);
#else
		owner = lock->owner;
#endif
		if (owner && !test_task_set_dynamic_svp_reentry_disable(owner)) {
			dynamic_svp_enqueue(owner, current, DYNAMIC_SVP_MUTEX, task->svp_depth, __func__);
			lock->svp_dep_task = owner;
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
			if (unlikely(owner->state & (TASK_INTERRUPTIBLE | TASK_UNINTERRUPTIBLE))) {
				struct task_struct *p[SVP_DEPTH_MAX];
				int deadloop = 0;
				u32 rwsem_race = 0;
				int rwsem_block_dep = 0;
				int rwsem_block_cnt = 0;
				u32 rwsemboost = 0;
				int stoptype;
				struct rsc_boost_lock_inf inf = {
					.begintype = DYNAMIC_SVP_KERNEL_LOCK_BEGIN,
					.boosttype = DYNAMIC_SVP_MUTEX,
					.deadloop = &deadloop,
					.rwsem_race = &rwsem_race,
					.rwsem_block_dep = &rwsem_block_dep,
					.rwsem_block_cnt = &rwsem_block_cnt,
					.stoptype = &stoptype,
					.typestring = "mutex_wait",
					.rwsemboost = &rwsemboost,
					.print = 1,
				};
				p[0] = owner;

				rsc_boost_lock_blockchain(p, &inf);
			}
#endif
		}
		rcu_read_unlock();
	}
}

static inline void mutex_dynamic_svp_dequeue(struct mutex *lock, struct task_struct *task)
{
	if (lock->svp_dep_task == task) {
		dynamic_svp_dequeue(task, DYNAMIC_SVP_MUTEX);
		lock->svp_dep_task = NULL;
	}
}

#endif
#endif
