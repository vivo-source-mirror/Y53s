#ifdef CONFIG_RSC_SVP_TASK_SCHEDULE

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <trace/events/sched.h>
#include <../sched/sched.h>

#if defined(CONFIG_RSC_SVP_TASK_SCHEDULE)
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_schedule.h>
#endif

int svp_min_sched_delay_granularity;/*svp thread delay upper bound(ms)*/
#ifdef CONFIG_RSC_LOCK_BOOST
int svp_max_dynamic_granularity = 32;/*svp dynamic max exist time(ms)*/
#endif
int svp_min_migration_delay = 10;/*svp min migration delay time(ms)*/

atomic_t rsc_svp_task_cnt = ATOMIC_INIT(0);
int rsc_svp_task_uid = -1;
u32 RSC_SVP_MANUAL_VAL;

/*int rsc_do_printk = 1;*/
#if 0
/*move to linux/vivo_rsc/rsc_schedule.h*/
static int entity_before(struct sched_entity *a,
				struct sched_entity *b)
{
	return (s64)(a->vruntime - b->vruntime) < 0;
}

void enqueue_svp_task(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
	bool exist = false;

	if (!rq || !p || !list_empty(&p->svp_entry)) {
		return;
	}
	p->enqueue_time = rq->clock;
	if (p->rsc_svp/* || atomic64_read(&p->dynamic_svp)*/) {
		list_for_each_safe(pos, n, &rq->svp_task_list) {
			if (pos == &p->svp_entry) {
				exist = true;
				break;
			}
		}
		if (!exist) {
			list_add_tail(&p->svp_entry, &rq->svp_task_list);
			get_task_struct(p);
			trace_sched_svp_queue_op(p, "svp_enqueue_succ");
		}
	}
}

void dequeue_svp_task(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
/*	u64 now =  jiffies_to_nsecs(jiffies);*/

	if (!rq || !p) {
		return;
	}
	p->enqueue_time = 0;
	if (!list_empty(&p->svp_entry)) {
		/*
		list_for_each_safe(pos, n, &rq->svp_task_list) {
			if (atomic64_read(&p->dynamic_svp) && (now - p->dynamic_svp_start) > (u64)svp_max_dynamic_granularity * S2US_TIME) {
				atomic64_set(&p->dynamic_svp, 0);
			}
		}
		*/
		list_for_each_safe(pos, n, &rq->svp_task_list) {
			if (pos == &p->svp_entry) {
				list_del_init(&p->svp_entry);
				put_task_struct(p);
				trace_sched_svp_queue_op(p, "svp_dequeue_succ");
				return;
			}
		}
	}
}

static struct task_struct *pick_first_svp_task(struct rq *rq)
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
			rsc_warn("task(%s,%d,%d) does not belong to cpu%d\n", temp->comm, task_cpu(temp), temp->policy, rq->cpu);
			list_del_init(&temp->svp_entry);
			continue;
		}
		if (leftmost_task == NULL) {
			leftmost_task = temp;
		} else if (entity_before(&temp->se, &leftmost_task->se)) {
			leftmost_task = temp;
		}
	}

	return leftmost_task;
}

void pick_svp_task(struct rq *rq, struct task_struct **p, struct sched_entity **se)
{
	struct task_struct *ori_p;
	struct task_struct *key_task;
	struct sched_entity *key_se;
	if (!rq || !p || !se) {
		return;
	}
	ori_p = *p;
	if (ori_p && !ori_p->rsc_svp/* && !atomic64_read(&ori_p->dynamic_svp)*/) {
		if (!list_empty(&rq->svp_task_list)) {
			key_task = pick_first_svp_task(rq);
			if (key_task) {
				key_se = &key_task->se;
				if (key_se && (rq->clock >= key_task->enqueue_time) &&
				rq->clock - key_task->enqueue_time >= ((u64)svp_min_sched_delay_granularity * S2US_TIME)) {
					trace_sched_svp_sched(key_task, "pick_svp");
					*p = key_task;
					*se = key_se;
				}
			}
		}
	}
}
#endif

static bool test_task_exist(struct task_struct *task, struct list_head *head)
{
	struct list_head *pos, *n;
	list_for_each_safe(pos, n, head) {
		if (pos == &task->svp_entry) {
			return true;
		}
	}
	return false;
}

u32 __read_mostly dynamic_svp_enqueue_enable = 1;
struct cpumask rsc_svp_run_cpus;
//CONFIG_RSC_LOCK_BOOST_IMPROVE
u32 dynamic_svp_tick_preempt_tick_count;

#ifdef CONFIG_RSC_LOCK_BOOST
/*
#define DYNAMIC_SVP_SEC_WIDTH   8
#define DYNAMIC_SVP_MASK_BASE   0x00000000ff

#define dynamic_svp_offset_of(type) (type * DYNAMIC_SVP_SEC_WIDTH)
#define dynamic_svp_mask_of(type) ((u64)(DYNAMIC_SVP_MASK_BASE) << (dynamic_svp_offset_of(type)))
#define dynamic_svp_get_bits(value, type) ((value & dynamic_svp_mask_of(type)) >> dynamic_svp_offset_of(type))
#define dynamic_svp_one(type) ((u64)1 << dynamic_svp_offset_of(type))

bool test_dynamic_svp(struct task_struct *task, int type)
{
	u64 dynamic_svp;
	if (!task) {
		return false;
	}
	dynamic_svp = atomic64_read(&task->dynamic_svp);
	return dynamic_svp_get_bits(dynamic_svp, type) > 0;
}
*/
#include <linux/vivo_rsc/rsc_test_svp.h>

unsigned long dynamic_svp_count[DYNAMIC_SVP_MAX];
u32 dynamic_svp_timeout_count[DYNAMIC_SVP_MAX];
u32 dynamic_svp_timeout_deq_count[DYNAMIC_SVP_MAX];

u32 dynamic_svp_tick_hit;
//u32 rsc_svp_tick_preempt_tick_count;
u32 rsc_svp_tick_preempt_tick_count;
u32 rsc_svp_preempt_pickup_count;
unsigned long rsc_svp_select_cpu_hit_count;

u32 dynamic_svp_timeout_total_count;
int dynamic_svp_max_depth;
u32 dynamic_svp_depth_count[SVP_DEPTH_MAX+1];
u32 dynamic_svp_page_alloc_count;
#ifdef CONFIG_RSC_LOCK_BOOST_STAT
u64 dynamic_svp_cost_time[DYNAMIC_SVP_MAX];
#endif
u64 dynamic_svp_slim_cost_time[DYNAMIC_SVP_MAX];

const char *rsc_dynamic_svp_name[DYNAMIC_SVP_MAX] = {
	"FUTEX",
	"BINDER",
	"RWSEM_WRITE",
	"MUTEX",
#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
	"RWSEM_READER",
#endif
	"PAGE_ALLOC",
	"CFS_FORCE"
};

static inline void dynamic_svp_dec(struct task_struct *task, int type)
{
	atomic64_sub(dynamic_svp_one(type), &task->dynamic_svp);
}

static inline void dynamic_svp_inc(struct task_struct *task, int type)
{
	atomic64_add(dynamic_svp_one(type), &task->dynamic_svp);
}

static void __dynamic_svp_dequeue(struct task_struct *task, int type)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	unsigned long flags;
#else
	struct rq_flags flags;
#endif
	bool exist = false;
	struct rq *rq = NULL;
	u64 dynamic_svp = 0;

	rq = task_rq_lock(task, &flags);
	dynamic_svp = atomic64_read(&task->dynamic_svp);
	if (dynamic_svp <= 0) {
		task->svp_depth = 0;
		task_rq_unlock(rq, task, &flags);
		return;
	}
	/*
	* maybe dequeue and enqueue is not pair!!! call times of dequeue is more than enqueue.
	*/
	if (test_dynamic_svp(task, type)) {
		dynamic_svp_dec(task, type);
		if (likely(!test_dynamic_svp(task, type))) {
			/*for stat time!*/
			u64 now;

#ifdef CONFIG_RSC_LOCK_BOOST_STAT
			now = sched_clock();
			dynamic_svp_cost_time[type] += now - task->dynamic_svp_time[type];
#endif

#if defined(RSC_LOCK_BOOST_STATTIME_USE_JIFFIES) || !defined(CONFIG_RSC_LOCK_BOOST_STAT)
			now = rsc_get_clock();
#endif
			dynamic_svp_slim_cost_time[type] += now - task->dynamic_svp_start;
			task->dynamic_svp_start = now;
		}
	}
	dynamic_svp = atomic64_read(&task->dynamic_svp);
	if (dynamic_svp > 0) {
		task_rq_unlock(rq, task, &flags);
		return;
	}
	task->svp_depth = 0;

	exist = test_task_exist(task, &rq->svp_task_list);
	if (exist) {
		list_del_init(&task->svp_entry);
		atomic_dec(&rsc_svp_task_cnt);
		rq->svp_manual_cnt--;
		put_task_struct(task);

		//CONFIG_RSC_LOCK_BOOST_IMPROVE
		if (list_empty(&rq->svp_task_list))
			cpumask_clear_cpu(rq->cpu, &rsc_svp_run_cpus);
		trace_sched_svp_queue_op(task, "dynamic_svp_dequeue_succ");
	}
	task_rq_unlock(rq, task, &flags);
}

void dynamic_svp_dequeue(struct task_struct *task, int type)
{
	/*
	if (!task || type >= DYNAMIC_SVP_MAX) {
		return;
	}
	*/
	__dynamic_svp_dequeue(task, type);
}

int dynamic_svp_enqueue_internal(struct task_struct *task, struct task_struct *from, int type, int depth, const char *func)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	unsigned long flags;
#else
	struct rq_flags flags;
#endif
	bool exist = false;
	struct rq *rq = NULL;

	if (type >= DYNAMIC_SVP_MAX)
		return -EINVAL;

	rq = task_rq_lock(task, &flags);
	if (task->sched_class != &fair_sched_class) {
		task_rq_unlock(rq, task, &flags);
		return -ERANGE;
	}
/* move to the bellow.
	if (unlikely(!list_empty(&task->svp_entry))) {
		printk(KERN_WARNING "task(%s,%d,%d) is already in another list", task->comm, task->pid, task->policy);
		task_rq_unlock(rq, task, &flags);
		return 1;
	}
*/
	dynamic_svp_count[type]++;
	task->dynamic_svp_start = rsc_get_clock();;
#ifdef CONFIG_RSC_LOCK_BOOST_STAT
	if (!test_dynamic_svp(task, type))
		task->dynamic_svp_time[type] = sched_clock();
#endif
	dynamic_svp_inc(task, type);

	dynamic_svp_max_depth = max(depth + 1, dynamic_svp_max_depth);
	task->svp_depth = task->svp_depth > depth + 1 ? task->svp_depth : depth + 1;
	if (unlikely(depth + 1 < SVP_DEPTH_MAX))
		dynamic_svp_depth_count[depth + 1]++;
	else
		dynamic_svp_depth_count[SVP_DEPTH_MAX]++;

	if (unlikely(!list_empty(&task->svp_entry))) {
		//printk(KERN_WARNING "RSC task(%s,%d,%d) is already in another list", task->comm, task->pid, task->policy);
		task_rq_unlock(rq, task, &flags);
		return 1;
	}

#if 0
	/*CONFIG_RSC_LOCK_BOOST_IMPROVE*/
	if (!task->on_rq) {
		if (likely(rsc_feat(RSC_LOCK_BOOST_TO_BIGCORE)))
			set_rsc_svp(task, RSC_SVP_MAINTHREAD);
	}
#endif


	if (task->state == TASK_RUNNING) {
		exist = test_task_exist(task, &rq->svp_task_list);
		if (!exist) {
			if (likely(dynamic_svp_enqueue_enable)) {
				//CONFIG_RSC_LOCK_BOOST_IMPROVE
				cpumask_set_cpu(rq->cpu, &rsc_svp_run_cpus);
				get_task_struct(task);
				list_add_tail(&task->svp_entry, &rq->svp_task_list);
				atomic_inc(&rsc_svp_task_cnt);
				rq->svp_manual_cnt++;
			}
			rsc_dbg(RSC_SVP_TASK_SCHEDULE, "svp enqueue type%d d%d %d "
#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
				"idx%d "
#endif
				"uid %5d %16s %5d(%16s %5d) u%d ou%d or%d vp%02x dy%08llx uid %5d %16s %5d(%16s %5d) call: %s\n",
					type, depth, task->svp_depth,
#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
					task->reader_owned_idx,
#endif
					task_uid(task).val, task->comm, task->pid, task->group_leader?task->group_leader->comm:"NULL"
					, task->group_leader?task->group_leader->pid:99999, task_cpu(task), task->on_cpu, task->on_rq,
					from->rsc_svp, atomic64_read(&from->dynamic_svp),
					task_uid(from).val, from->comm, from->pid, from->group_leader?from->group_leader->comm:"NULL"
					, from->group_leader?from->group_leader->pid:99999, func);

			trace_sched_svp_queue_op(task, "dynamic_svp_enqueue_succ");
		}
	} else {
		rsc_dbg(RSC_SVP_TASK_SCHEDULE, "svp enqueue type%d d%d %d "
#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
			"idx%d "
#endif
			"uid %5d %16s %5d(%16s %5d) u%d ou%d or%d vp%02x dy%08llx uid %5d %16s %5d(%16s %5d) call: %s NOT_RUNNING\n",
				type, depth, task->svp_depth,
#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
				task->reader_owned_idx,
#endif
				task_uid(task).val, task->comm, task->pid, task->group_leader?task->group_leader->comm:"NULL"
				, task->group_leader?task->group_leader->pid:99999, task_cpu(task), task->on_cpu, task->on_rq,
				from->rsc_svp, atomic64_read(&from->dynamic_svp),
				task_uid(from).val, from->comm, from->pid, from->group_leader?from->group_leader->comm:"NULL"
				, from->group_leader?from->group_leader->pid:99999, func);
	}
	task_rq_unlock(rq, task, &flags);

	return 0;
}
#endif

static struct task_struct *check_svp_delayed(struct rq *rq)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct task_struct *tsk = NULL;
	struct list_head *svp_task_list = NULL;
	svp_task_list = &rq->svp_task_list;

	list_for_each_safe(pos, n, svp_task_list) {
		tsk = list_entry(pos, struct task_struct, svp_entry);
		if (tsk && (tsk->rsc_svp & RSC_SVP_MANUAL) && (rq->clock - tsk->enqueue_time) >= (u64)svp_min_migration_delay * S2US_TIME)
			return tsk;
	}
	return NULL;
}

static int svp_task_hot(struct task_struct *p, struct rq *src_rq, struct rq *dst_rq)
{
	s64 delta;

	lockdep_assert_held(&src_rq->lock);

	if (p->sched_class != &fair_sched_class)
		return 0;

	if (unlikely(p->policy == SCHED_IDLE))
		return 0;

	if (sched_feat(CACHE_HOT_BUDDY) && dst_rq->nr_running &&
	    (&p->se == src_rq->cfs.next || &p->se == src_rq->cfs.last))
		return 1;

	if (sysctl_sched_migration_cost == (unsigned int)-1)
		return 1;
	if (sysctl_sched_migration_cost == 0)
		return 0;

	delta = src_rq->clock_task - p->se.exec_start;
	return delta < (s64)sysctl_sched_migration_cost;
}

static void detach_task(struct task_struct *p, struct rq *src_rq, struct rq *dst_rq)
{
	trace_sched_svp_queue_op(p, "detach_task");
	lockdep_assert_held(&src_rq->lock);
	deactivate_task(src_rq, p, 0);
	p->on_rq = TASK_ON_RQ_MIGRATING;
	double_lock_balance(src_rq, dst_rq);
	set_task_cpu(p, dst_rq->cpu);
	double_unlock_balance(src_rq, dst_rq);
}

static void attach_task(struct rq *dst_rq, struct task_struct *p)
{
	trace_sched_svp_queue_op(p, "attach_task");
	raw_spin_lock(&dst_rq->lock);
	BUG_ON(task_rq(p) != dst_rq);
	p->on_rq = TASK_ON_RQ_QUEUED;
	activate_task(dst_rq, p, 0);
	check_preempt_curr(dst_rq, p, 0);
	raw_spin_unlock(&dst_rq->lock);
}

static int svp_can_migrate(struct task_struct *p, struct rq *src_rq, struct rq *dst_rq)
{
	if (task_running(src_rq, p))
		return 0;
	if (svp_task_hot(p, src_rq, dst_rq))
		return 0;
	if (task_rq(p) != src_rq)
		return 0;
	if (!test_task_exist(p, &src_rq->svp_task_list))
		return 0;
	return 1;
}

#define CONFIG_RSC_FAST_SCHED
#ifdef CONFIG_RSC_FAST_SCHED
extern struct cpumask rsc_cpu_bigcore;
extern struct cpumask rsc_cpu_littlecore;
#endif

static int __do_svp_balance(void *data)
{
	struct rq *src_rq = data;
	struct rq *dst_rq = NULL;
	int src_cpu = cpu_of(src_rq);
	int i;
	struct task_struct *p = NULL;
#ifdef CONFIG_RSC_FAST_SCHED
	struct cpumask cpus_allowed = CPU_MASK_NONE;
#endif
	bool is_mig = false;

	/*find a delayed svp task*/
	raw_spin_lock_irq(&src_rq->lock);
	if (unlikely(src_cpu != smp_processor_id() || !src_rq->active_svp_balance) || src_rq->svp_manual_cnt <= 1) {
		src_rq->active_svp_balance = 0;
		raw_spin_unlock_irq(&src_rq->lock);
		return 0;
	}
	p = check_svp_delayed(src_rq);
	if (!p) {
		src_rq->active_svp_balance = 0;
		raw_spin_unlock_irq(&src_rq->lock);
		return 0;
	}

	/*find a free-cpu*/
#ifdef CONFIG_RSC_FAST_SCHED
	if (cpumask_test_cpu(src_cpu, &rsc_cpu_bigcore))
		cpumask_and(&cpus_allowed, &rsc_cpu_bigcore, &(p->cpus_allowed));
	else
#ifdef CONFIG_RSC_LOCK_BOOST
		cpumask_copy(&cpus_allowed, &(p->cpus_allowed));
#else
		cpumask_and(&cpus_allowed, &rsc_cpu_littlecore, &(p->cpus_allowed));
#endif

	raw_spin_unlock(&src_rq->lock);
	for (i = 0; i < nr_cpu_ids; i++) {
		if (i == src_cpu)
			continue;
		if (cpumask_test_cpu(i, &cpus_allowed)) {
			if (!cpu_online(i)
#if !defined(CONFIG_MTK_PLATFORM) && !defined(CONFIG_SOC_SAMSUNG)
				|| cpu_isolated(i)
#endif
				)
				continue;
			dst_rq = cpu_rq(i);
			raw_spin_lock(&dst_rq->lock);
			if (!dst_rq->rt.rt_nr_running && !dst_rq->svp_manual_cnt/*list_empty(&dst_rq->svp_task_list)*/) {
				raw_spin_unlock(&dst_rq->lock);
				break;
			} else {
				raw_spin_unlock(&dst_rq->lock);
			}
		}
	}
#else
	raw_spin_unlock(&src_rq->lock);
	for_each_cpu_and(i, tsk_cpus_allowed(p), cpu_coregroup_mask(src_cpu)) {
		if (i == src_cpu || !cpu_online(i)
#if !defined(CONFIG_MTK_PLATFORM) && !defined(CONFIG_SOC_SAMSUNG)
			|| cpu_isolated(i)
#endif
			)
			continue;
		dst_rq = cpu_rq(i);
		raw_spin_lock(&dst_rq->lock);
		if (!dst_rq->rt.rt_nr_running && !dst_rq->svp_manual_cnt/*list_empty(&dst_rq->svp_task_list)*/) {
			raw_spin_unlock(&dst_rq->lock);
			break;
		} else {
			raw_spin_unlock(&dst_rq->lock);
		}
	}
#endif
	/*move p from src to dst cpu*/
	raw_spin_lock(&src_rq->lock);
	if (i != nr_cpu_ids && p != NULL && dst_rq != NULL) {
		if (svp_can_migrate(p, src_rq, dst_rq)) {
			detach_task(p, src_rq, dst_rq);
			is_mig = true;
		}
	}
	src_rq->active_svp_balance = 0;
	raw_spin_unlock(&src_rq->lock);
	if (is_mig) {
		printk_deferred("RSC vmigrate do_svp_balance uid %5d %16s %5d(%16s %5d) cur %5d %16s %5d(%16s %5d) "
			"cpuset: %*pbl on_rq%d state%lx pcaller: %ps cpu src%d dst%d svp_cnt:%d\n",
			task_uid(p).val, p->comm, p->pid, p->group_leader->comm, p->group_leader->pid,
			task_uid(current).val, current->comm, current->pid, current->group_leader->comm, current->group_leader->pid,
			cpumask_pr_args(tsk_cpus_allowed(p)), p->on_rq, p->state, (void *)get_wchan(p),
			src_rq->cpu, dst_rq->cpu, src_rq->svp_manual_cnt);

		attach_task(dst_rq, p);
	}
	local_irq_enable();
	return 0;
}

void trigger_svp_balance(struct rq *rq)
{
	struct task_struct *task = NULL;
	int active_svp_balance = 0;
	if (!rq) {
		return;
	}
	raw_spin_lock(&rq->lock);
	task = check_svp_delayed(rq);
	/*
	 * active_svp_balance synchronized accesss to svp_balance_work
	 * 1 means svp_balance_work is ongoing, and reset to 0 after
	 * svp_balance_work is done
	 */
	if (!rq->active_svp_balance && task) {
		active_svp_balance = 1;
		rq->active_svp_balance = 1;
	}
	raw_spin_unlock(&rq->lock);
	if (active_svp_balance) {
		stop_one_cpu_nowait(cpu_of(rq), __do_svp_balance, rq, &rq->svp_balance_work);
	}
}

void svp_init_rq_data(struct rq *rq)
{
	if (!rq) {
		return;
	}
	rq->active_svp_balance = 0;
	rq->active_limit_balance = 0;
	rq->svp_pick_cnt = 0;
	rq->nor_pick_cnt = 0;
	rq->svp_manual_cnt = 0;
	rq->svp_overload_jiffies = 0;
	INIT_LIST_HEAD(&rq->svp_task_list);
}
#endif
