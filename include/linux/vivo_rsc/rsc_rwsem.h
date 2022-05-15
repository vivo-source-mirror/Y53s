#ifdef CONFIG_RSC_LOCK_BOOST
#ifndef __RSC_RWSEM_H__
#define __RSC_RWSEM_H__

#include <linux/sched.h>
#include <linux/list.h>
#include <linux/rwsem.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_test_svp.h>

#ifndef RWSEM_READER_OWNED
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
#define RWSEM_READER_OWNED	(1UL << 0)
#define RWSEM_RD_NONSPINNABLE	(1UL << 1)
#define RWSEM_WR_NONSPINNABLE	(1UL << 2)
#define RWSEM_NONSPINNABLE	(RWSEM_RD_NONSPINNABLE | RWSEM_WR_NONSPINNABLE)
#define RWSEM_OWNER_FLAGS_MASK	(RWSEM_READER_OWNED | RWSEM_NONSPINNABLE)
#else
#define RWSEM_ANONYMOUSLY_OWNED	(1UL << 0)
#define RWSEM_READER_OWNED	((struct task_struct *)RWSEM_ANONYMOUSLY_OWNED)
#endif
#endif

#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
enum rwsem_waiter_type {
	RWSEM_WAITING_FOR_WRITE,
	RWSEM_WAITING_FOR_READ
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	enum rwsem_waiter_type type;
	unsigned long timeout;
	unsigned long last_rowner;
};
#else
struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	enum rwsem_waiter_type type;
};
#endif

#if 0
#define rwsem_first_waiter(sem) \
		list_first_entry(&sem->wait_list, struct rwsem_waiter, list)

//unsigned long get_current_wchan_debug(struct task_struct *p);
static inline void rwsem_boost_blockchain(struct rwsem_waiter *waiter,
	struct rw_semaphore *sem, const char *hintstring, bool is_first_waiter)
{
#if 0//use to develop
	s16 owner_pid = sem->owner_pid & ~RSC_RWSEM_READER_OWNED;
	bool owner_reader = !!(sem->owner_pid & RSC_RWSEM_READER_OWNED);

	if (sem->owner_pid & RSC_RWSEM_READER_OWNED) {
		//write_failed
		//maybe
		//do boost owner_pid
		//should count != RWSEM_WAITING_BIAS
		//regular expression: rwsemfailwrite.* owner_pid[\s]{1,}[1-9]{1,} read1.*canboost1
		//[7:om.vivo.browser:17706] rwsemfailwrite firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in 	   vm_mmap_pgoff no writeowner uid 10025  om.vivo.browser 17706 vp42 dy00000000 ( om.vivo.browser 17706) owner_pid 17958 read1 owner 0000000000000001 block1 count ffffffff00000001 WAITING ffffffff00000000 writer: 1 writer uid 10025  om.vivo.browser 17706 vp42 dy00000000 -> 2 writer uid 10025  TaskSchedulerSi 18213 vp00 dy00000000 -> 3 reader uid 10025  Jit thread pool 17713 vp00 dy00000000 -> canboost1 owninlist0 tc   3 wc	2 rc  1 writerinlist

		//read_failed
		//seldom
		//do boost owner_pid
		//should count != RWSEM_WAITING_BIAS
		//regular expression: rwsemfailread.* owner_pid[\s]{1,}[1-9]{1,} read1.*canboost1
		//<4>[ 3474.281573][10-20 22:00:24]  [5:pool-22-thread-:20002] rwsemfailread firstwaiter0 waiter->task1 ifwaiter0 ib1 isfg1 istop1 current block in        do_page_fault no writeowner uid 10303  pool-22-thread- 20002 vp00 dy00010000 ( ngdong.app.mall 19566) owner_pid 19686 read1 owner 0000000000000001 block1 count ffffffff00000001 WAITING ffffffff00000000 writer: 1 writer uid 10303   Binder:19566_2 19578 vp00 dy00000001 -> 2 reader uid 10303  pool-22-thread- 20002 vp00 dy00010000 -> 3 writer uid 10303  pool-4-thread-1 19687 vp00 dy00000000 -> canboost1 owninlist0 tc   3 wc  2 rc  1 writerinlist
		if (owner_pid) {

		}
	}
#endif
#if 0//use to develop
	else if (sem->owner_pid) {
		//write_failed
		//maybe
		//donot need boost owner_pid
		//should count == RWSEM_WAITING_BIAS
		//regular expression: rwsemfailwrite.* owner_pid[\s]{1,}[1-9]{1,} read0.*canboost1
		//<4>[ 3423.639064][10-20 21:59:33]  [1:sankuai.meituan:16938] rwsemfailwrite firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in        vm_mmap_pgoff no writeowner uid 10304  sankuai.meituan 16938 vp02 dy00000000 ( sankuai.meituan 16938) owner_pid 16961 read0 owner 0000000000000001 block0 count ffffffff00000000 WAITING ffffffff00000000 writer: 1 writer uid 10304  sankuai.meituan 16938 vp02 dy00000000 -> 2 writer uid 10304   HeapTaskDaemon 16961 vp00 dy00000000 -> 3 reader uid 10304      MbcThread#1 17286 vp00 dy00000000 -> 4 reader uid 10304  Jit thread pool 16955 vp00 dy00000000 -> canboost1 owninlist1 tc   4 wc  2 rc  2 writerinlist

		//read_failed
		//seldom
		//donot need boost owner_pid
		//should count == RWSEM_WAITING_BIAS
		//regular expression: rwsemfailread.* owner_pid[\s]{1,}[1-9]{1,} read0.*canboost1
		//rwsemfailread firstwaiter0 waiter->task1 ifwaiter0 ib1 isfg1 istop1 current block in          SyS_madvise no writeowner uid 10304  Jit thread pool 17975 vp00 dy00010000 ( sankuai.meituan 17964) owner_pid 18455 read0 owner           (null) block0 count ffffffff00000000 WAITING ffffffff00000000 writer: 1 writer uid 10304      MbcThread#1 18455 vp00 dy00000001 -> 2 reader uid 10304  Jit thread pool 17975 vp00 dy00010000 -> 3 reader uid 10304  Sniffer Runnabl 18169 vp00 dy00000000 -> canboost1
	} else {//sem->owner_pid is zero
		//write_failed
		//maybe
		//donot need boost owner_pid: because reader is doing up_read and in rwsem_wake function!
		//regular expression: rwsemfailwrite.* owner_pid     0 read0.*canboost1
		//[6:sankuai.meituan:16938] rwsemfailwrite firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in 	   vm_mmap_pgoff no writeowner uid 10304  sankuai.meituan 16938 vp02 dy00000000 ( sankuai.meituan 16938) owner_pid	   0 read0 owner 0000000000000001 block1 count ffffffff00000001 WAITING ffffffff00000000 writer: 1 writer uid 10304  sankuai.meituan 16938 vp02 dy00000000 -> 2 writer uid 10304  pool-24-thread- 17376 vp00 dy00000000 -> 3 reader uid 10304  pool-15-thread- 17155 vp00 dy00000000 -> 4 reader uid 10304	Jit thread pool 16955 vp00 dy00000000 -> canboost1 owninlist0 tc   4 wc  2 rc  2 writerinlist

		//write_failed
		//seldom
		//donot need boost owner_pid: need to check!
		//regular expression: rwsemfailread.* owner_pid     0 read0.*canboost1
		//[4: HeapTaskDaemon:16961] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10304   HeapTaskDaemon 16961 vp00 dy00000001 ( sankuai.meituan 16938) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10304	 HeapTaskDaemon 16961 vp00 dy00000001 -> 2 writer uid 10304  pool-1-thread-1 17346 vp00 dy00000000 -> 3 reader uid 10304  Jit thread pool 16955 vp00 dy00000000 -> canboost1 owninlist0 tc	 3 wc  1 rc  2 writerinlist
		//[7:om.vivo.browser:17706] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025  om.vivo.browser 17706 vp42 dy00000000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	om.vivo.browser 17706 vp42 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[1:om.vivo.browser:17706] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025  om.vivo.browser 17706 vp42 dy00000000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	om.vivo.browser 17706 vp42 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[4: vivo-data-self:17912] rwsemfailread firstwaiter0 waiter->task1 ifwaiter0 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025   vivo-data-self 17912 vp00 dy00010000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 0000000000000001 block1 count ffffffff00000004 WAITING ffffffff00000000 writer: 1 writer uid 10025	Jit thread pool 17713 vp00 dy00000001 -> 2 reader uid 10025   vivo-data-self 17912 vp00 dy00010000 -> 3 reader uid 10025  pool-4-thread-4 18299 vp00 dy00000000 -> 4 writer uid 10025  pool-ad_common1 18040 vp00 dy00000000 -> 5 reader uid 10025	ProxyRuntimeThr 18230 vp00 dy00000000 -> 6 reader uid 10025  vivo.com.cn/... 18256 vp00 dy00000000 -> 7 reader uid 10025  vivo.com.cn/... 17951 vp00 dy00000000 -> canboost1 owninlist0 tc	 7 wc  2 rc  5 writerinlist
		//[5:om.vivo.browser:17706] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025  om.vivo.browser 17706 vp42 dy00000000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	om.vivo.browser 17706 vp42 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[6:om.vivo.browser:17706] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025  om.vivo.browser 17706 vp02 dy00000000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	om.vivo.browser 17706 vp02 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[1:vivo-data-singl:17932] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10025  vivo-data-singl 17932 vp00 dy00010000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	vivo-data-singl 17932 vp00 dy00010000 -> 2 reader uid 10025  browser_async_e 17887 vp00 dy00000000 -> canboost1 owninlist0 tc	2 wc  0 rc	2 writernolist
		//[4:om.vivo.browser:17706] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in			   move_vma no writeowner uid 10025  om.vivo.browser 17706 vp02 dy00000000 ( om.vivo.browser 17706) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10025	om.vivo.browser 17706 vp02 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[5: Binder:5257_1D: 7985] rwsemfailread firstwaiter0 waiter->task1 ifwaiter0 ib1 isfg1 istop0 current block in				m_start no writeowner uid  1000   Binder:5257_1D  7985 vp00 dy00000001 (   system_server  5257) owner_pid	  0 read0 owner 0000000000000001 block1 count ffffffff00000001 WAITING ffffffff00000000 writer: 1 writer uid 10305	m.taobao.taobao 18846 vp02 dy00000000 -> 2 reader uid  1000   Binder:5257_1D  7985 vp00 dy00000001 -> 3 reader uid	1000   Binder:5257_1C  7921 vp00 dy00000000 -> canboost1 owninlist0 tc	 3 wc  1 rc  2 writerinlist
		//[5:m.taobao.taobao:18846] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10305  m.taobao.taobao 18846 vp42 dy00000000 ( m.taobao.taobao 18846) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10305	m.taobao.taobao 18846 vp42 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
		//[7:m.taobao.taobao:18846] rwsemfailread firstwaiter1 waiter->task1 ifwaiter1 ib1 isfg1 istop1 current block in		  do_page_fault no writeowner uid 10305  m.taobao.taobao 18846 vp42 dy00000000 ( m.taobao.taobao 18846) owner_pid	  0 read0 owner 		  (null) block1 count fffffffe00000001 WAITING ffffffff00000000 writer: 1 reader uid 10305	m.taobao.taobao 18846 vp42 dy00000000 -> canboost1 owninlist0 tc   1 wc  0 rc  1 writernolist
	}
#endif

#if 0
	if (rsc_debug & RSC_LOCK_BOOST_FUTEX_TRACK) {
		struct rwsem_waiter *waiter_inlist, *tmp;
		int writer_count = 0, total_count = 0;
		char print_str[1024];
		int printlen = 0, cur;
		bool firstwaiter = (rwsem_first_waiter(sem) == waiter);
		bool inbootime = (time_before(jiffies, rsc_app_boost_stime));
		bool isfg = svp_is_fg_task(current);
		bool istop = rsc_top_task(current);
		bool owner_pid_in_list = false;

		cur = snprintf(print_str + printlen, sizeof(print_str) - printlen,
					"rwsemfailwrite firstwaiter%d waiter->task%d ifwaiter%d ib%d isfg%d istop%d current block in %20ps no writeowner uid %5d %16s %5d vp%02x dy%08llx (%16s %5d) "
					"owner_pid %5hd read%d owner %p block%d count %08lx WAITING %08lx writer: ",
					firstwaiter, !!waiter->task, is_first_waiter, inbootime, isfg, istop,
					get_current_wchan_debug(current), current_uid().val, current->comm, current->pid,
					current->rsc_svp, atomic64_read(&current->dynamic_svp),
					current->group_leader->comm, current->group_leader->pid,
					owner_pid, owner_reader, READ_ONCE(sem->owner), count != RWSEM_WAITING_BIAS, count, RWSEM_WAITING_BIAS);
		printlen += cur;

		list_for_each_entry_safe(waiter_inlist, tmp, &sem->wait_list, list) {
			struct task_struct *tsk;

			total_count++;
			if (waiter_inlist->type == RWSEM_WAITING_FOR_WRITE)
				writer_count++;
			//if (waiter_inlist->type == RWSEM_WAITING_FOR_WRITE) {
				tsk = waiter_inlist->task;
				if (!owner_pid_in_list && (tsk->pid == owner_pid))
					owner_pid_in_list = true;
				cur = snprintf(print_str + printlen, sizeof(print_str) - printlen - 50, "%d %s uid %5d %16s %5d vp%02x dy%08llx -> ",
					total_count, (waiter_inlist->type == RWSEM_WAITING_FOR_WRITE) ? "writer" : "reader", task_uid(tsk).val, tsk->comm, tsk->pid,
					tsk->rsc_svp, atomic64_read(&tsk->dynamic_svp));
				printlen += cur;
			//}
		}

		cur = snprintf(print_str + printlen, sizeof(print_str) - printlen, "canboost%d owninlist%d tc %3d wc %2d rc %2d %s\n",
			(/*(total_count > 1) && */waiter->task) ? 1 : 0, owner_pid_in_list, total_count, writer_count, total_count - writer_count,	writer_count ? "writerinlist" : "writernolist");
		printlen += cur;
		printk("%s", print_str);
	}
#endif
}
#endif
#endif

static inline void rwsem_list_add_svp(struct list_head *entry, struct list_head *head)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct rwsem_waiter *waiter = NULL;
	list_for_each_safe(pos, n, head) {
		waiter = list_entry(pos, struct rwsem_waiter, list);
		if (!test_task_svp_render_nocheck(waiter->task)) {
			list_add(entry, waiter->list.prev);
			return;
		}
	}
	if (pos == head) {
		list_add_tail(entry, head);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
static inline struct task_struct * rsc_rwsem_owner(struct rw_semaphore *sem, bool *read)
{
	long owner_org = atomic_long_read(&sem->owner);//READ_ONCE(sem->owner);

#if 1
	/*only boost write owner.*/
	*read = false;
	return !(owner_org & RWSEM_READER_OWNED) ? (struct task_struct *)(owner_org & ~RWSEM_OWNER_FLAGS_MASK) : NULL;
#else
	/*boost read owner*/
	*read = !!(owner_org & RWSEM_READER_OWNED);
	return (struct task_struct *)(owner_org & ~RWSEM_OWNER_FLAGS_MASK);
#endif
}
#else
static inline struct task_struct * rsc_rwsem_owner(struct rw_semaphore *sem, bool *read)
{
	struct task_struct *owner;

	owner = READ_ONCE(sem->owner);
	*read = false;
	return (owner && owner != RWSEM_READER_OWNED) ? owner : NULL;
}
#endif

/* enqueue mainthread or  dynamic thread*/
static inline void rwsem_dynamic_svp_writer_enqueue(struct task_struct *tsk,
	struct rwsem_waiter *waiter, struct rw_semaphore *sem, bool isreadpath, bool is_first_waiter)
{
	bool is_svp = test_task_svp_checkdepth(tsk);

	if (is_svp && waiter->task) {
		struct task_struct *owner;
		bool read;

		rcu_read_lock();
		owner = rsc_rwsem_owner(sem, &read);
		if (owner) {
			if (!test_task_set_dynamic_svp_reentry_disable(owner) && !sem->svp_dep_task) {
				char hintstr[24];

				snprintf(hintstr, sizeof(hintstr), "rwsem_enq_read%d_rp%d", read, isreadpath);
				dynamic_svp_enqueue(owner, current, DYNAMIC_SVP_RWSEM, tsk->svp_depth, (const char *)hintstr/*__func__*/);
				sem->svp_dep_task = owner;
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
						.boosttype = DYNAMIC_SVP_RWSEM,
						.deadloop = &deadloop,
						.rwsem_race = &rwsem_race,
						.rwsem_block_dep = &rwsem_block_dep,
						.rwsem_block_cnt = &rwsem_block_cnt,
						.stoptype = &stoptype,
						.typestring = isreadpath ? "rwsem_read" : "rwsem_write",
						.rwsemboost = &rwsemboost,
						.print = 1,
					};

					p[0] = owner;
					rsc_boost_lock_blockchain(p, &inf);
				}
#endif
			}
		}
#ifdef CONFIG_RSC_LOCK_BOOST_FUTEX
		else {
			//rwsem_boost_blockchain(waiter, sem, hitstring, is_first_waiter);
		}
#endif
		rcu_read_unlock();
	}
}

static inline void rwsem_dynamic_svp_dequeue(struct rw_semaphore *sem, struct task_struct *tsk)
{
	if (sem->svp_dep_task == tsk) {
		dynamic_svp_dequeue(tsk, DYNAMIC_SVP_RWSEM);
		sem->svp_dep_task = NULL;
	}
}

#ifdef CONFIG_RSC_LOCK_BOOST_RWSEM_READER_OWNED
static inline void rwsem_dynamic_svp_reader_enqueue(struct task_struct *tsk, struct task_struct *waiter_task, struct task_struct *owner, struct rw_semaphore *sem, int idx)
{
	if (!test_task_svp_render_nocheck(owner)/* && !sem->reader_dep_task[idx]*/) {
		dynamic_svp_enqueue(owner, current, DYNAMIC_SVP_RWSEM_READER, tsk->svp_depth, __func__);
		/*sem->reader_dep_task[idx] = owner;*/
	}
}

static inline void rwsem_dynamic_svp_reader_dequeue(struct rw_semaphore *sem, struct task_struct *tsk/*, int idx*/)
{
	dynamic_svp_dequeue(tsk, DYNAMIC_SVP_RWSEM_READER);
	/*sem->reader_dep_task[idx] = NULL;*/
}
#endif
#endif
#endif
