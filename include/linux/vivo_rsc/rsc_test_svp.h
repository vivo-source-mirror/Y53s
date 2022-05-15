#ifndef __RSC_TEST_SVP_H__
#define __RSC_TEST_SVP_H__

#ifdef CONFIG_RSC_LOCK_BOOST
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/cpuset.h>
#include <linux/kasan.h>

static inline bool test_task_svp_depth(int vip_depth)
{
	return vip_depth < SVP_DEPTH_MAX;
}

static inline bool test_task_svp_nocheck(struct task_struct *task)
{
	return (((task->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && !(task->rsc_svp & RSC_SVP_RENDERTHREAD) && /*svp_is_fg_task(task)*/ rsc_top_task(task)) ||
		atomic64_read(&task->dynamic_svp));
}

static inline bool test_task_svp_checkdepth(struct task_struct *task)
{
	return (((task->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && !(task->rsc_svp & RSC_SVP_RENDERTHREAD) && /*svp_is_fg_task(task)*/ rsc_top_task(task)) ||
		(atomic64_read(&task->dynamic_svp) && test_task_svp_depth(task->svp_depth)));
}

static inline bool test_task_svp_nocheck_render(struct task_struct *task)
{
	return (((task->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && /*svp_is_fg_task(task)*/ rsc_top_task(task)) ||
		atomic64_read(&task->dynamic_svp));
}

static inline bool test_task_svp(struct task_struct *task)
{
	return task && test_task_svp_nocheck(task);
}

static inline bool test_task_set_dynamic_svp_reentry_disable(struct task_struct *task)
{
#ifdef RSC_ALLOW_SET_DYNAMIC_SVP_REENTRY
	return false;
#else
	return test_task_svp_nocheck_render(task);
#endif
}

static inline bool test_task_svp_render_nocheck(struct task_struct *task)
{
	return test_task_svp_nocheck_render(task);
}

static inline bool test_task_dynamic_svp_checkdepth(struct task_struct *task)
{
	return atomic64_read(&task->dynamic_svp) && test_task_svp_depth(task->svp_depth);
}

static inline bool test_task_svp_nolock(struct task_struct *task)
{
	return (((task->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && rsc_top_task_nolock(task)) ||
		atomic64_read(&task->dynamic_svp));
}

static inline bool test_task_svp_fast(struct task_struct *p)
{
	return (((p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD))/* && rsc_top_task(p)*/)
			|| atomic64_read(&p->dynamic_svp));
}

static inline bool test_task_static_svp(struct task_struct *p)
{
	return ((p->rsc_svp & (RSC_SVP_MANUAL | RSC_SVP_MAINTHREAD)) && rsc_top_task(p));

}

static inline bool test_task_launcher_ui_thread(struct task_struct *p)
{
	return (p == rsc_launcher_group_leader);
}

static inline bool test_task_launcher_render_thread(struct task_struct *p)
{
	return ((p->rsc_svp & RSC_SVP_MANUAL) && (p->group_leader == rsc_launcher_group_leader));
}

#define DYNAMIC_SVP_SEC_WIDTH   8
#define DYNAMIC_SVP_MASK_BASE   0x00000000ff

#define dynamic_svp_offset_of(type) (type * DYNAMIC_SVP_SEC_WIDTH)
#define dynamic_svp_mask_of(type) ((u64)(DYNAMIC_SVP_MASK_BASE) << (dynamic_svp_offset_of(type)))
#define dynamic_svp_get_bits(value, type) ((value & dynamic_svp_mask_of(type)) >> dynamic_svp_offset_of(type))
#define dynamic_svp_one(type) ((u64)1 << dynamic_svp_offset_of(type))

static inline void dynamic_svp_inc_extern(struct task_struct *task, int type)
{
	atomic64_add(dynamic_svp_one(type), &task->dynamic_svp);
}

static inline bool test_dynamic_svp(struct task_struct *task, int type)
{
	u64 dynamic_svp;
	if (!task) {
		return false;
	}
	dynamic_svp = atomic64_read(&task->dynamic_svp);
	return dynamic_svp_get_bits(dynamic_svp, type) > 0;
}

static inline bool test_dynamic_svp_nocheck(struct task_struct *task, int type)
{
	u64 dynamic_svp;

	dynamic_svp = atomic64_read(&task->dynamic_svp);
	return dynamic_svp_get_bits(dynamic_svp, type) > 0;
}

static inline int dynamic_svp_enqueue(struct task_struct *task, struct task_struct *from, int type, int depth, const char *func)
{
	if (task->sched_class != &fair_sched_class) {
		return -ERANGE;
	}

	return dynamic_svp_enqueue_internal(task, from, type, depth, func);
}

#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
#include <linux/uaccess.h>
#include <asm/thread_info.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>

#ifndef TASK_STATE_TO_CHAR_STR
#define TASK_STATE_TO_CHAR_STR "RSDTtXZPxKWNn"
#endif
static const char stat_nam[] = TASK_STATE_TO_CHAR_STR;

static inline s16 rsc_get_owner_pid(s16 *dest, s16 *from)
{
	s16 ret;

	ret = __get_user(*dest, from);

	return ret ? -EFAULT : 0;
}

struct rsc_boost_lock_inf {
	enum DYNAMIC_SVP_TYPE begintype;
	enum DYNAMIC_SVP_TYPE boosttype;
	int *deadloop;
	u32 *rwsem_race;
	int *rwsem_block_dep;
	int *rwsem_block_cnt;
	int *stoptype;
	const char *typestring;
	u32 *rwsemboost;
	int print;
};

/*must hold rcu_read_lock, before call it!*/
static inline int rsc_boost_lock_blockchain(struct task_struct *p[], struct rsc_boost_lock_inf *inf)
{
	struct task_struct *b;
	int i, t;
	s16 pid;
	mm_segment_t fs;
	s16 *chain_pid;
	int loopcnt = 1;
	static const char *rwsem_hint[] = {
		"noblock ",
		"failget ",
		"failrace",
		"read_own",
		"write_own",
		"mutex_own"
	};

	b = p[0];
	/*
	 * We need to switch to kernel mode so that we can use __get_user
	 * to safely read from kernel space.
	 */
	fs = get_fs();
	/*same as probe_kernel_read*/
	set_fs(KERNEL_DS);
	pagefault_disable();
	t = inf->begintype;

loop:
	for (; t < DYNAMIC_SVP_LOCK_MAX; t++) {
		if (t >= DYNAMIC_SVP_KERNEL_LOCK_BEGIN) {
			/* maybe the lock memory has release, we just skip it in KASAN  check*/
			kasan_disable_current();
			chain_pid = READ_ONCE(b->dynamic_svp_klock_chain_pid_addr[t - DYNAMIC_SVP_KERNEL_LOCK_BEGIN]);
			if (chain_pid) {
				*inf->rwsem_block_dep = loopcnt;
				(*inf->rwsem_block_cnt)++;
				if (rsc_get_owner_pid(&pid, chain_pid) < 0) {
					*inf->rwsem_race = 1;
					kasan_enable_current();
					break;
				}
				if (chain_pid != READ_ONCE(b->dynamic_svp_klock_chain_pid_addr[t - DYNAMIC_SVP_KERNEL_LOCK_BEGIN])) {
					*inf->rwsem_race = 2;
					kasan_enable_current();
					break;
				}
				if (t == DYNAMIC_SVP_RWSEM) {
					#if 1
					/*reader owner not support now.*/
					*inf->rwsem_race = 4;
					#else
					if (pid & RSC_RWSEM_READER_OWNED)
						*inf->rwsem_race = 3;
					else
						*inf->rwsem_race = 4;
					pid = pid & ~RSC_RWSEM_READER_OWNED;
					#endif
				} else {
					*inf->rwsem_race = 5;
				}
				kasan_enable_current();
				if (pid <= 0)
					break;
			} else {
				kasan_enable_current();
				continue;
			}
		} else {
			pid = b->dynamic_svp_ulock_pid[t];
		}
		if (pid) {
			if (pid < 0)
				break;
			/*avoid block chain dead loop!*/
			if (pid == current->pid) {
				rsc_futex_block_chain_dead_loop_count[inf->boosttype][0]++;
				*inf->deadloop = 1;
				p[loopcnt] = current;
				loopcnt++;
				goto out_loop;
			}
			for (i = 1; i <= loopcnt; i++) {
				if (pid == p[i-1]->pid) {
					rsc_futex_block_chain_dead_loop_count[inf->boosttype][1]++;
					*inf->deadloop = 2;
					p[loopcnt] = p[i-1];
					loopcnt++;
					goto out_loop;
				}
			}

			b = find_task_by_vpid(pid);
			if (b) {
				int ret;

				(*inf->rwsemboost)++;
				rsc_block_chain_boost_count[t]++;
				ret = dynamic_svp_enqueue(b, current, t, current->svp_depth, __func__);
				if ((t == DYNAMIC_SVP_RWSEM) && (ret >= 0)/* && likely(!(p[loopcnt-1]->exit_state & (EXIT_TRACE | TASK_DEAD)))*/) {
					#if 1
					*inf->rwsemboost = *inf->rwsemboost + (1<<8) + (1<<16);
//					atomic_inc(&(p[loopcnt-1]->group_leader->dynamic_mmap_sem_boost_count));
					atomic_inc(&(b->dynamic_svp_rwsem_boost));
					this_cpu_inc(rsc_rwsem_enq_boost_count);
					#else
					struct mm_struct *mm;
					struct rw_semaphore *mmap_sem, *rwsem;
					//mm = get_task_mm(p[loopcnt-1]);
					mm = READ_ONCE(p[loopcnt-1]->mm);
					if (mm) {
						mmap_sem = &mm->mmap_sem;
						rwsem = container_of(chain_pid, struct rw_semaphore, owner_pid);
						*inf->rwsemboost = *inf->rwsemboost + (1<<8);
						/*access group_leader is safe?*/
						if (mmap_sem == rwsem) {
							*inf->rwsemboost = *inf->rwsemboost + (1<<16);
							this_cpu_inc(rsc_rwsem_enq_boost_count);
//							atomic_inc(&(p[loopcnt-1]->group_leader->dynamic_mmap_sem_boost_count));
							atomic_inc(&(p[loopcnt-1]->dynamic_svp_rwsem_boost));
						}
						//mmput(mm);
					}
					#endif
				}

				p[loopcnt] = b;
				loopcnt++;

				/*stop search when block task is not sleeping.*/
				if (!(b->state & (TASK_INTERRUPTIBLE | TASK_UNINTERRUPTIBLE))) {
					/*binder is a special case, update*/
					if (t != DYNAMIC_SVP_BINDER)
						break;
					else {
						//rsc_block_chain_binder_not_sleeping_count++;
					}
				}

				if (loopcnt >= SVP_DEPTH_MAX)
					break;
				if (t >= DYNAMIC_SVP_KERNEL_LOCK_BEGIN)
					t = DYNAMIC_SVP_KERNEL_LOCK_BEGIN;
				else
					t = DYNAMIC_SVP_BINDER;/*DYNAMIC_SVP_USER_LOCK_BEGIN;*//*futex block chain has not implemented, it is hard work!!!*/
				goto loop;
			}
			break;
		}
	}

out_loop:
	pagefault_enable();
	set_fs(fs);
	rsc_lock_boost_trace[inf->boosttype][loopcnt-1]++;
	*inf->stoptype = t;

	if (inf->print) {
		if (unlikely(rsc_debug & RSC_LOCK_BOOST_FUTEX_TRACK)) {
			unsigned state;

			state = p[0]->state ? ffs(p[0]->state) : 0;
			rsc_info("%s svp block owneruid %5d %16s %5d s=%c run%d %03lx b%02d "
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
				"block %16ps "
#endif
				"iowait%d (%16s %5d) vp%02x dy%08llx <- "
				"uid %5d %16s %5d(%16s %5d) vp%02x dy%08llx %s depth %d loop %d t %d "\
				"rwsrace %s dep%d cnt%d "
				"deadloop%d%d rwsemboost%d %06x %s",
				inf->typestring,
				task_uid(p[0]).val, p[0]->comm, p[0]->pid,
				state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?',
				p[0]->on_cpu,
				p[0]->state,
				ffs(p[0]->state),
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
				p[0]->dynamic_svp_block_addr,
#endif
				(int)(p[0]->in_iowait),
				p[0]->group_leader->comm, p[0]->group_leader->pid,
				p[0]->rsc_svp, atomic64_read(&p[0]->dynamic_svp),
				current_uid().val, current->comm, current->pid,
				current->group_leader->comm, current->group_leader->pid,
				current->rsc_svp, atomic64_read(&current->dynamic_svp),
				is_compat_task()?"32bit":"64bit",
				current->svp_depth+1,
				loopcnt, *inf->stoptype,
				*inf->rwsem_race < ARRAY_SIZE(rwsem_hint) ? rwsem_hint[*inf->rwsem_race] : "???????",
				*inf->rwsem_block_dep, *inf->rwsem_block_cnt,
				*inf->deadloop ? 1 : 0, *inf->deadloop,
				(*inf->rwsemboost >= 0x10000) ? 1 : 0, *inf->rwsemboost, 
				loopcnt > 1 ? "futex_trace" : "\n");
			for (i = 1; i < loopcnt; i++) {
				state = p[i]->state ? ffs(p[i]->state) : 0;
				printk(" %d <- uid %5d %16s %5d s=%c run%d %03lx b%02d "
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
					"block %16ps "
#endif
					"iowait%d "
					"(%16s %5d)%s",
					i, task_uid(p[i]).val, p[i]->comm, p[i]->pid,
					state < sizeof(stat_nam) - 1 ? stat_nam[state] : '?',
					p[i]->on_cpu,
					p[i]->state,
					ffs(p[i]->state),
#ifdef CONFIG_RSC_LOCK_BOOST_TRACK_DEBUG
					p[i]->dynamic_svp_block_addr,
#endif
					(int)(p[i]->in_iowait),
					p[i]->group_leader->comm, p[i]->group_leader->pid,
					(i+1 == loopcnt) ? "\n" : ""
				);
			}
		}
	}

	return loopcnt;
}
#endif
#endif
#endif
