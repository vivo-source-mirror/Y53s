/*
 * include/linux/vivo_rsc/rsc_iolimit.h
 *
 * VIVO Resource Control.
 *
 * for iolimit.
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */

#include <linux/vivo_rsc/rsc_internal.h>

#ifndef _RSC_IOLIMIT_H
#define _RSC_IOLIMIT_H

#include <linux/cgroup.h>
#include <linux/atomic.h>
#include <linux/timer.h>

#ifdef CONFIG_RSC_IOLIMIT
struct iolimit_cgroup {
	struct cgroup_subsys_state      css;
	atomic_long_t					switching;

	/*0 means not limit*/
	atomic_long_t					write_limit;
	s64                             write_part_nbyte;
	s64                             write_already_used;
	struct timer_list               write_timer;
	spinlock_t                      write_lock;
	wait_queue_head_t               write_wait;

	/*0 means not limit*/
	atomic_long_t					read_limit;
	s64                             read_part_nbyte;
	s64                             read_already_used;
	struct timer_list               read_timer;
	spinlock_t                      read_lock;
	wait_queue_head_t               read_wait;
};

static inline struct iolimit_cgroup *css_iolimit(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct iolimit_cgroup, css) : NULL;
}

static inline struct iolimit_cgroup *task_iolimitcg(struct task_struct *tsk)
{
	return css_iolimit(task_css(tsk, iolimit_cgrp_id));
}

void do_io_write_bandwidth_control(size_t count);

void do_io_read_bandwidth_control(size_t count);

static inline void io_read_bandwidth_control(size_t count)
{
	rcu_read_lock();
	if (likely(task_css_is_root(current, iolimit_cgrp_id))) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return;
	} else {
		rcu_read_unlock();
		do_io_read_bandwidth_control(count);
	}
}

static inline void io_write_bandwidth_control(size_t count)
{
	rcu_read_lock();
	if (likely(task_css_is_root(current, iolimit_cgrp_id))) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return;
	} else {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		do_io_write_bandwidth_control(count);
	}
}

static inline void io_generic_read_bandwidth_control(size_t count)
{
	rcu_read_lock();
	if (likely(task_css_is_root(current, iolimit_cgrp_id))) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return;
	} else {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		task_set_in_pagefault(current);
		do_io_read_bandwidth_control(count);
		task_clear_in_pagefault(current);
	}
}
#else  /* !CONFIG_RSC_IOLIMIT */

static inline void io_write_bandwidth_control(size_t count)
{
}

static inline void io_read_bandwidth_control(size_t count)
{
}

static inline void io_generic_read_bandwidth_control(size_t count)
{
}
#endif /* !CONFIG_RSC_IOLIMIT */

#endif /* _RSC_IOLIMIT_H */

