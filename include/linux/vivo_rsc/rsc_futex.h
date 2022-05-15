#ifndef __RSC_FUTEX_H__
#define __RSC_FUTEX_H__

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_test_svp.h>

#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
/* enqueue mainthread or  dynamic thread*/
static inline int futex_dynamic_svp_enqueue(struct task_struct *owner, int svp_depth)
{
	if (!test_task_set_dynamic_svp_reentry_disable(owner)) {
		return dynamic_svp_enqueue(owner, current, DYNAMIC_SVP_FUTEX, svp_depth, __func__);
	}

	return -ESRCH;
}

static inline void futex_dynamic_svp_dequeue(struct task_struct *tsk, int dequeue, u32 __user *uaddr, bool share)
{
	bool is_svp = test_dynamic_svp(tsk, DYNAMIC_SVP_FUTEX);

	if (dequeue && is_svp) {
		if (rsc_debug & RSC_LOCK_BOOST_FUTEX_TRACK) {
			rsc_info("futex_wake uid %5d %16s %5d(%16s %5d) vp%02x dy%08llx %s depth %d %sdeq%d %s sh%d uaddr: %p\n",
				current_uid().val, tsk->comm, tsk->pid,
				tsk->group_leader->comm, tsk->group_leader->pid,
				tsk->rsc_svp, atomic64_read(&tsk->dynamic_svp),
				is_compat_task()?"32bit":"64bit",
				tsk->svp_depth, dequeue ? "do" : "no",
				dequeue, (dequeue && !is_svp) ? "nohit" : "amhit", share, uaddr);
		}
		dynamic_svp_dequeue(tsk, DYNAMIC_SVP_FUTEX);
	}
}
#endif
#endif
