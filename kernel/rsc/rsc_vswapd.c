#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/rwsem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/hugetlb.h>
#include <linux/huge_mm.h>
#include <asm/tlbflush.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include "rsc_zram.h"

#define RSC_VSWAPD_FREE_UNIT_SIZE		(2UL << (20 - PAGE_SHIFT))
#define RSC_VSWAPD_5MB_PAGE				(5UL << (20 - PAGE_SHIFT))

/*max sleep 10s*/
#define RSC_VSWAPD_EMERGENCY_STOP_MAX_TIME_JIFFIES (10*HZ)
extern unsigned long totalram_pages;
extern struct cpumask rsc_cpu_littlecore;
int vswapd_try_to_free_pages(int nr_pages, u16 mode, int priority);
#ifndef RSC_MEM_SEVEN_GB_PAGES
#define RSC_MEM_SEVEN_GB_PAGES ((7*1024UL) << (20 - PAGE_SHIFT))
#endif
extern unsigned long totalram_pages;

wait_queue_head_t __read_mostly vswapd_wait = {
		.lock = __SPIN_LOCK_UNLOCKED(vswapd_wait.spinlock),
		.head = {&vswapd_wait.head, &vswapd_wait.head},
};
int __read_mostly kswapd_is_asleep = 2;
int __read_mostly vswapd_enable = 1;
u32 __read_mostly vswapd_start_pages = RSC_VSWAPD_START_DEFAULT_PAGES;
u32 vswapd_stop_pages = RSC_VSWAPD_STOP_DEFAULT_PAGES;

unsigned long vswapd_total_pages;
unsigned long vswapd_total_time_jiffies;
u64 vswapd_total_exe_time_us;
u32 vswapd_total_wakeup_cnt;
u32 vswapd_total_timeout_cnt;
u32 vswapd_total_kswap_running_cnt;
struct task_struct *vswapd_task;
u32 vswapd_max_running_time_jiffies = RSC_MS_TO_JIFF(20*1000);
u32 vswapd_min_filecache_stop_pages = 1024 << (20 - PAGE_SHIFT);
u32 vswapd_total_min_filecache_cnt;
/*
* defult value of vswapd_emergency_stop should be zero,
* or vswapd_task is NULL and then system crash in function kswapd_try_to_sleep.
*/
int vswapd_emergency_stop;
u32 vswapd_reclaim_zero_cnt;
u32 vswapd_emergency_stop_wakeup_miss_cnt;
u32 vswapd_emergency_stop_timeout_cnt;
u32 vswapd_short_sleep_wakeup_cnt;

static int rsc_free_pages(int nr_pages, int scan_mode)
{
	int unit_pages, total, real = 0, reclaim;

	for (total = 0; total < nr_pages; total += 32) {
		unit_pages =
		    ((total + 32) > nr_pages) ? (nr_pages - total) : 32;
		reclaim = vswapd_try_to_free_pages(unit_pages, scan_mode, DEF_PRIORITY);
		real += reclaim;
		if (unlikely(!reclaim))
			vswapd_reclaim_zero_cnt++;
		//cond_resched();
	}

	/* return real reclaimed page count */
	return real;
}

static inline unsigned long elapsed_jiffies(unsigned long start)
{
	unsigned long end = jiffies;

	if (end >= start)
		return (unsigned long)(end - start);

	return (unsigned long)(end + (MAX_JIFFY_OFFSET - start) + 1);
}

int rsc_vswapd(void *p)
{
	struct task_struct *tsk = current;
	struct reclaim_state reclaim_state = {
		.reclaimed_slab = 0,
	};
	const struct cpumask *cpumask = &rsc_cpu_littlecore;
	u64 exetime;
	unsigned long free_pages, reclaim_pages;
	unsigned long time_jiffies, cur_jiffies;
	bool timeout;
	bool mincache;
	DEFINE_WAIT(wait);
	long remaining;
	bool shortwakeup;

	vswapd_task = current;
	if (!cpumask_empty(cpumask))
		set_cpus_allowed_ptr(tsk, cpumask);
	tsk->reclaim_state = &reclaim_state;

	tsk->flags |= PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD | PF_FREEZER_SKIP;
	free_pages = global_zone_page_state(NR_FREE_PAGES);
	if (totalram_pages <= RSC_MEM_SEVEN_GB_PAGES) {
		rsc_info("rsc_vswapd disable because DRAM size %lu MB less than 7GB, "
			"you can enable it by setting vspwad_enable\n",
			totalram_pages >> (20 - PAGE_SHIFT));
		vswapd_start_pages = 0;
		vswapd_stop_pages = 0;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	for ( ; ; ) {
		cur_jiffies = jiffies;
		time_jiffies = 0;
		reclaim_pages = 0;
		timeout = false;
		mincache = false;
		shortwakeup = false;
		exetime = tsk->se.sum_exec_runtime;
		/*free more 5MB(RSC_VSWAPD_FREE_UNIT_SIZE/2)*/
		while (free_pages < (vswapd_stop_pages + RSC_VSWAPD_5MB_PAGE)) {
			if (!kswapd_is_asleep) {
				vswapd_total_kswap_running_cnt++;
				break;
			}
			if (unlikely(global_node_page_state(NR_FILE_PAGES) <= vswapd_min_filecache_stop_pages)) {
				mincache = true;
				vswapd_total_min_filecache_cnt++;
				break;
			}
			reclaim_pages += rsc_free_pages(RSC_VSWAPD_FREE_UNIT_SIZE, RSC_ZRAM_MODE_FILE|RSC_ZRAM_MODE_ANON);
			free_pages = global_zone_page_state(NR_FREE_PAGES);
			time_jiffies = elapsed_jiffies(cur_jiffies);
			if (time_jiffies >= vswapd_max_running_time_jiffies) {
				timeout = true;
				vswapd_total_timeout_cnt++;
				break;
			}
		}
		vswapd_total_pages += reclaim_pages;
		vswapd_total_time_jiffies += time_jiffies;
		vswapd_total_wakeup_cnt++;
		vswapd_total_exe_time_us += (tsk->se.sum_exec_runtime-exetime)/1000;

		/*we always donot stop vswapd*/
#if 0
		if (kthread_should_stop())
			break;
#endif

		if (!(vswapd_total_wakeup_cnt % 10) || (time_jiffies > RSC_MS_TO_JIFF(2*1000)) || (rsc_debug & RSC_VSWAPD))
			rsc_info("rsc_vswapd: goto sleep free %3lu fc %4lu MB reclaim %3lu MB total %7lu MB "
				"cost %4lu exe %4llu tt %8lu texe %8llu ms wakecnt %5u timeout%d mincache%d kslp%d sp %u longrun%d\n",
				free_pages >> (20 - PAGE_SHIFT),
				global_node_page_state(NR_FILE_PAGES) >> (20 - PAGE_SHIFT),
				reclaim_pages >> (20 - PAGE_SHIFT), vswapd_total_pages >> (20 - PAGE_SHIFT),
				RSC_JIFF_TO_MS(time_jiffies), (tsk->se.sum_exec_runtime-exetime)/(1000*1000),
				RSC_JIFF_TO_MS(vswapd_total_time_jiffies), vswapd_total_exe_time_us/1000,
				vswapd_total_wakeup_cnt, timeout, mincache, kswapd_is_asleep, vswapd_start_pages,
				!!(time_jiffies > RSC_MS_TO_JIFF(2*1000)));

		/*
		* Park vswapd by emergency case, such as mincache or kswapd running,
		* then waitqueue_active(&vswapd_wait) will be return false.
		* and only do one judgment in __alloc_pages_nodemask.
		* It will be good performance.
		*/
		if (unlikely(mincache || !kswapd_is_asleep || !vswapd_start_pages)) {

sleep_again:
			set_current_state(TASK_INTERRUPTIBLE);
			if (likely(vswapd_start_pages)) {
				vswapd_emergency_stop = 1;
				remaining = schedule_timeout(RSC_VSWAPD_EMERGENCY_STOP_MAX_TIME_JIFFIES);
				if (!remaining) {
					vswapd_emergency_stop_timeout_cnt++;
					if (!kswapd_is_asleep ||
						(unlikely(global_node_page_state(NR_FILE_PAGES) <= vswapd_min_filecache_stop_pages))) {
						goto sleep_again;
					} else
						vswapd_emergency_stop_wakeup_miss_cnt++;
				}
			} else
				schedule();
			__set_current_state(TASK_RUNNING);
			vswapd_emergency_stop = 0;
			free_pages = global_zone_page_state(NR_FREE_PAGES);
		} else {
			/*
			* Park vswapd by normal case,
			* when buddy free page is less than vswapd_start_pages,
			* vswapd will wake up.
			*/
			prepare_to_wait(&vswapd_wait, &wait, TASK_INTERRUPTIBLE);
			remaining = schedule_timeout(HZ/10);
			finish_wait(&vswapd_wait, &wait);
			free_pages = global_zone_page_state(NR_FREE_PAGES);
			/*
			* prevent vswapd wakeup frequently.
			*/
			if (!remaining && (free_pages >= vswapd_stop_pages)) {
				vswapd_short_sleep_wakeup_cnt++;
				prepare_to_wait(&vswapd_wait, &wait, TASK_INTERRUPTIBLE);
				schedule();
				finish_wait(&vswapd_wait, &wait);
				free_pages = global_zone_page_state(NR_FREE_PAGES);
			} else
				shortwakeup = true;
		}
		if (!(vswapd_total_wakeup_cnt % 10) || (time_jiffies > RSC_MS_TO_JIFF(2*1000)) || (rsc_debug & RSC_VSWAPD))
			rsc_info("rsc_vswapd: wakeup: free_pages: %llu MB ssw%d\n", free_pages >> (20 - PAGE_SHIFT), shortwakeup);
	}

	vswapd_task = NULL;

	tsk->flags &= ~(PF_MEMALLOC | PF_SWAPWRITE | PF_KSWAPD);
	tsk->reclaim_state = NULL;

	return 0;
}
