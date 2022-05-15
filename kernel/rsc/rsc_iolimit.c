/*
 * kernel/rsc/rsc_iolimit.c
 *
 * VIVO Resource Control.
 *
 *for limit io
 *
 * Copyright (C) 2017 VIVO Technology Co., Ltd
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vivo_rsc/rsc_iolimit.h>
#include <linux/jiffies.h>
#include <linux/blk-cgroup.h>

#if 0
#define RSC_IO_TYPE
#endif

#define RSC_FIX_HIGH_CPUUSAGE
/*HZ/8 = 125ms*/
#define RSC_TIMER_CNT_PER_HZ	8
/*ms*/
#define RSC_WAIT_TIMEOUT	(1000/RSC_TIMER_CNT_PER_HZ)
#define RSC_IN_READ		0
#define RSC_IN_WRITE	1

static int is_need_iolimit(struct iolimit_cgroup *iolimitcg, int rw)
{
	int ret = 0;
#ifdef RSC_IO_TYPE
	struct blkcg *blkcg = task_blkcg(current);

	if (blkcg->type != BLK_THROTL_KBG && blkcg->type != BLK_THROTL_BG)
		return 0;
#endif

	ret = signal_pending_state(TASK_INTERRUPTIBLE, current);
	if (ret == 1)
		return 0;

	if (!atomic_long_read(&iolimitcg->switching))
		return 0;
	else {
		if (rw == RSC_IN_WRITE)
			return atomic_long_read(&iolimitcg->write_limit);
		else
			return atomic_long_read(&iolimitcg->read_limit);
		return 0;
	}
}

static bool is_write_need_wakeup(struct iolimit_cgroup *iolimitcg, size_t count)
{
	int ret = false;
#ifdef RSC_IO_TYPE
	struct blkcg *blkcg;
#endif

	if (atomic_long_read(&iolimitcg->switching) == 0)
		ret = true;

#ifdef RSC_FIX_HIGH_CPUUSAGE
	if (!iolimitcg->write_part_nbyte ||
		(iolimitcg->write_part_nbyte >= (iolimitcg->write_already_used + count)))
#else
	if (iolimitcg->write_part_nbyte > iolimitcg->write_already_used)
#endif
		ret = true;

	rcu_read_lock();
	if (iolimitcg != task_iolimitcg(current))
		ret = true;

#ifdef RSC_IO_TYPE
	blkcg = task_blkcg(current);
	if (blkcg->type != BLK_THROTL_KBG && blkcg->type != BLK_THROTL_BG)
		ret = true;
#endif
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	return ret;
}

static bool is_read_need_wakeup(struct iolimit_cgroup *iolimitcg, size_t count)
{
	int ret = false;
#ifdef RSC_IO_TYPE
	struct blkcg *blkcg;
#endif

	if (atomic_long_read(&iolimitcg->switching) == 0)
		ret = true;

#ifdef RSC_FIX_HIGH_CPUUSAGE
	if (!iolimitcg->read_part_nbyte ||
		(iolimitcg->read_part_nbyte >= (iolimitcg->read_already_used + count)))
#else
	if (iolimitcg->read_part_nbyte > iolimitcg->read_already_used)
#endif
		ret = true;

	rcu_read_lock();
	if (iolimitcg != task_iolimitcg(current))
		ret = true;

#ifdef RSC_IO_TYPE
	blkcg = task_blkcg(current);
	if (blkcg->type != BLK_THROTL_KBG && blkcg->type != BLK_THROTL_BG)
		ret = true;
#endif
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
	return ret;
}

void do_io_write_bandwidth_control(size_t count)
{
	size_t may_io_cnt;
	struct iolimit_cgroup *iolimitcg;
	u64 t0 = 0, t1;
	int ret;

repeat:
	rcu_read_lock();
	iolimitcg = task_iolimitcg(current);
	if (!is_need_iolimit(iolimitcg, RSC_IN_WRITE)) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return;
	}

	spin_lock_bh(&iolimitcg->write_lock);
	may_io_cnt = iolimitcg->write_part_nbyte - iolimitcg->write_already_used;
	if (may_io_cnt < count) {
		spin_unlock_bh(&iolimitcg->write_lock);
		if (css_tryget_online(&iolimitcg->css)) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
			rcu_read_unlock_inline();
#else
			rcu_read_unlock();
#endif
			if (rsc_debug & IOLIMIT)
				t0 = local_clock();
			ret = wait_event_interruptible_timeout(iolimitcg->write_wait,
				is_write_need_wakeup(iolimitcg, count), msecs_to_jiffies(RSC_WAIT_TIMEOUT));
			if (rsc_debug & IOLIMIT) {
				t1 = local_clock();
				if ((t1 - t0) < (1 * 1000 * 1000))
					rsc_dbg(IOLIMIT, "%s: write delay %llu us comm: %s(%d) ret: %d may_io_cnt: %lu count: %lu cond: %d sw: %ld "
					"w: %lld aw: %lld iolimitcg: %p %p\n", __func__,
					(t1 - t0)/1000, current->comm, current->pid, ret, may_io_cnt, count,
					is_write_need_wakeup(iolimitcg, count),
					atomic_long_read(&iolimitcg->switching),
					iolimitcg->write_part_nbyte,
					iolimitcg->write_already_used,
					(void *)iolimitcg, (void *)task_iolimitcg(current)
					);
			}
			css_put(&iolimitcg->css);
		} else {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
			rcu_read_unlock_inline();
#else
			rcu_read_unlock();
#endif
		}
		goto repeat;
	} else if (may_io_cnt >= count) {
		may_io_cnt = count;
		iolimitcg->write_already_used += may_io_cnt;
	}

	spin_unlock_bh(&iolimitcg->write_lock);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
}

void do_io_read_bandwidth_control(size_t count)
{
	size_t may_io_cnt;
	struct iolimit_cgroup *iolimitcg;
	u64 t0 = 0, t1;
	int ret;

	if (unlikely(task_in_pagefault(current)))
		return;

repeat:
	rcu_read_lock();
	iolimitcg = task_iolimitcg(current);
	if (!is_need_iolimit(iolimitcg, RSC_IN_READ)) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
		rcu_read_unlock_inline();
#else
		rcu_read_unlock();
#endif
		return;
	}

	spin_lock_bh(&iolimitcg->read_lock);
	may_io_cnt = iolimitcg->read_part_nbyte - iolimitcg->read_already_used;
	if (may_io_cnt < count) {
		spin_unlock_bh(&iolimitcg->read_lock);
		if (css_tryget_online(&iolimitcg->css)) {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
			rcu_read_unlock_inline();
#else
			rcu_read_unlock();
#endif
			if (rsc_debug & IOLIMIT)
				t0 = local_clock();
			ret = wait_event_interruptible_timeout(iolimitcg->read_wait,
				is_read_need_wakeup(iolimitcg, count), msecs_to_jiffies(RSC_WAIT_TIMEOUT));
			if (rsc_debug & IOLIMIT) {
				t1 = local_clock();
				if ((t1 - t0) < (1 * 1000 * 1000))
					rsc_dbg(IOLIMIT, "%s: read delay %llu us comm: %s(%d) ret: %d may_io_cnt: %lu count: %lu cond: %d sw: %ld "
					"r: %lld ar: %lld iolimitcg: %p %p\n", __func__,
					(t1 - t0)/1000, current->comm, current->pid, ret, may_io_cnt, count,
					is_read_need_wakeup(iolimitcg, count),
					atomic_long_read(&iolimitcg->switching),
					iolimitcg->read_part_nbyte,
					iolimitcg->read_already_used,
					(void *)iolimitcg, (void *)task_iolimitcg(current)
					);
			}
			css_put(&iolimitcg->css);
		} else {
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
			rcu_read_unlock_inline();
#else
			rcu_read_unlock();
#endif
		}

		if (task_in_pagefault(current)) {
			return;
		} else {
			goto repeat;
		}
	} else if (may_io_cnt >= count) {
		may_io_cnt = count;
		iolimitcg->read_already_used += may_io_cnt;
	}

	spin_unlock_bh(&iolimitcg->read_lock);
#if defined(CONFIG_RSC_RCU_LOCK_INLINE)
	rcu_read_unlock_inline();
#else
	rcu_read_unlock();
#endif
}

static void write_timer_handler(unsigned long data)
{
	struct iolimit_cgroup *iolimitcg = (struct iolimit_cgroup *)data;
	spin_lock_bh(&iolimitcg->write_lock);
	iolimitcg->write_already_used = 0;
	spin_unlock_bh(&iolimitcg->write_lock);
	wake_up_all(&iolimitcg->write_wait);
	mod_timer(&iolimitcg->write_timer, jiffies + HZ / RSC_TIMER_CNT_PER_HZ);
}

static void read_timer_handler(unsigned long data)
{
	struct iolimit_cgroup *iolimitcg = (struct iolimit_cgroup *)data;
	spin_lock_bh(&iolimitcg->read_lock);
	iolimitcg->read_already_used = 0;
	spin_unlock_bh(&iolimitcg->read_lock);
	wake_up_all(&iolimitcg->read_wait);
	mod_timer(&iolimitcg->read_timer, jiffies + HZ / RSC_TIMER_CNT_PER_HZ);
}

static struct cgroup_subsys_state *
iolimit_css_alloc(struct cgroup_subsys_state *parent)
{
	struct iolimit_cgroup *iolimitcg;

	iolimitcg = kzalloc(sizeof(struct iolimit_cgroup), GFP_KERNEL);
	if (!iolimitcg) {
		return ERR_PTR(-ENOMEM);
	}

		atomic_long_set(&iolimitcg->switching, 0);
		iolimitcg->write_part_nbyte = 0;
		iolimitcg->write_already_used = 0;
		init_timer(&iolimitcg->write_timer);
		iolimitcg->write_timer.data = (unsigned long)iolimitcg;
		iolimitcg->write_timer.function = write_timer_handler;
		spin_lock_init(&iolimitcg->write_lock);
		init_waitqueue_head(&iolimitcg->write_wait);

		atomic_long_set(&iolimitcg->read_limit, 0);
		iolimitcg->read_part_nbyte = 0;
		iolimitcg->read_already_used = 0;
		init_timer(&iolimitcg->read_timer);
		iolimitcg->read_timer.data = (unsigned long)iolimitcg;
		iolimitcg->read_timer.function = read_timer_handler;
		spin_lock_init(&iolimitcg->read_lock);
		init_waitqueue_head(&iolimitcg->read_wait);

		return &iolimitcg->css;
}

static void iolimit_css_free(struct cgroup_subsys_state *css)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	del_timer_sync(&iolimitcg->write_timer);
	del_timer_sync(&iolimitcg->read_timer);
	kfree(css_iolimit(css));
}

static s64 iolimit_switching_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic_long_read(&iolimitcg->switching);
}

static int iolimit_switching_write(struct cgroup_subsys_state *css, struct cftype *cft,
										s64 switching)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (switching != 0 && switching != 1) {
		err = -EINVAL;
		goto out;
	}
	atomic_long_set(&iolimitcg->switching, switching);
	if (switching == 0) {
#ifdef RSC_FIX_HIGH_CPUUSAGE
		iolimitcg->write_already_used = 0;
#endif
		wake_up_all(&iolimitcg->write_wait);
		del_timer_sync(&iolimitcg->write_timer);

#ifdef RSC_FIX_HIGH_CPUUSAGE
		iolimitcg->read_already_used = 0;
#endif
		wake_up_all(&iolimitcg->read_wait);
		del_timer_sync(&iolimitcg->read_timer);
	} else if (switching == 1) {
		mod_timer(&iolimitcg->write_timer, jiffies + HZ / RSC_TIMER_CNT_PER_HZ);
		iolimitcg->write_already_used = iolimitcg->write_part_nbyte;

		mod_timer(&iolimitcg->read_timer, jiffies + HZ / RSC_TIMER_CNT_PER_HZ);
		iolimitcg->read_already_used = iolimitcg->read_part_nbyte;
	}
out:
	return err;
}

static s64 writeiolimit_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic_long_read(&iolimitcg->write_limit);
}

static int writeiolimit_write(struct cgroup_subsys_state *css, struct cftype *cft,
				s64 limit)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (limit < 0) {
		rsc_err("limit(%lld) must bigger than %ld comm:%s(%d)\n",
			limit, (PAGE_SIZE * RSC_TIMER_CNT_PER_HZ) - 1, current->comm, current->pid);
		err = -EINVAL;
		goto out;
	}

#ifdef RSC_FIX_HIGH_CPUUSAGE
	if (limit > 0 && limit < (PAGE_SIZE * RSC_TIMER_CNT_PER_HZ)) {
		/*at lease 32KB*/
		rsc_err("limit(%lld) must bigger than %ld comm:%s(%d)\n",
			limit, (PAGE_SIZE * RSC_TIMER_CNT_PER_HZ) - 1, current->comm, current->pid);
		err = -EINVAL;
		goto out;
	}
#endif

	atomic_long_set(&iolimitcg->write_limit, limit);
	spin_lock_bh(&iolimitcg->write_lock);
	iolimitcg->write_part_nbyte = limit / RSC_TIMER_CNT_PER_HZ;
	spin_unlock_bh(&iolimitcg->write_lock);
#ifdef RSC_FIX_HIGH_CPUUSAGE
	if (!limit)
		wake_up_all(&iolimitcg->write_wait);
#endif
out:
	return err;
}

static s64 readiolimit_read(struct cgroup_subsys_state *css, struct cftype *cft)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);

	return atomic_long_read(&iolimitcg->read_limit);
}

static int readiolimit_write(struct cgroup_subsys_state *css, struct cftype *cft,
				s64 limit)
{
	struct iolimit_cgroup *iolimitcg = css_iolimit(css);
	int err = 0;

	if (limit < 0) {
		err = -EINVAL;
		goto out;
	}

#ifdef RSC_FIX_HIGH_CPUUSAGE
	/*at lease 32KB*/
	if (limit > 0 && limit < (PAGE_SIZE * RSC_TIMER_CNT_PER_HZ)) {
		err = -EINVAL;
		goto out;
	}
#endif

	atomic_long_set(&iolimitcg->read_limit, limit);
	spin_lock_bh(&iolimitcg->read_lock);
	iolimitcg->read_part_nbyte = limit / RSC_TIMER_CNT_PER_HZ;
	spin_unlock_bh(&iolimitcg->read_lock);
#ifdef RSC_FIX_HIGH_CPUUSAGE
	if (!limit)
		wake_up_all(&iolimitcg->read_wait);
#endif
out:
	return err;
}

static void iolimit_attach(struct cgroup_taskset *tset)
{
	struct task_struct *task;
	struct cgroup_subsys_state *dst_css;

	cgroup_taskset_for_each(task, dst_css, tset)
		wake_up_process(task);
}

static struct cftype iolimit_files[] = {
	{
		.name = "switching",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = iolimit_switching_read,
		.write_s64 = iolimit_switching_write,
	},
	{
		.name = "write_limit",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = writeiolimit_read,
		.write_s64 = writeiolimit_write,
	},
	{
		.name = "read_limit",
		.flags = CFTYPE_NOT_ON_ROOT,
		.read_s64 = readiolimit_read,
		.write_s64 = readiolimit_write,
	},
	{}
};

struct cgroup_subsys iolimit_cgrp_subsys = {
	.css_alloc      = iolimit_css_alloc,
	.css_free       = iolimit_css_free,
	.attach         = iolimit_attach,
	.legacy_cftypes = iolimit_files,
};
