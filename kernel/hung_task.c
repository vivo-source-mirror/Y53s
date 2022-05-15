/*
 * Detect Hung Task
 *
 * kernel/hung_task.c - kernel thread for detecting tasks stuck in D state
 *
 */

#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/nmi.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/lockdep.h>
#include <linux/export.h>
#include <linux/sysctl.h>
#include <linux/suspend.h>
#include <linux/utsname.h>
#include <linux/sched/signal.h>
#include <linux/sched/debug.h>
#include <linux/module.h>

#include <trace/events/sched.h>
#define VIVO_HUNG_TASK_FEATURE
#ifdef VIVO_HUNG_TASK_FEATURE

extern int download_mode;
struct task_struct *g_hung_task;
static bool hung_task_call_panic_critical;
#define VIVO_CONFIG_DEFAULT_HUNG_TASK_TIMEOUT (150)
#define VIVO_CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE (1)
#define VIVO_HUNG_TASK_CRITICAL_MAGIC (0x0F)
#define VIVO_HUNG_TASK_WHITE_LIST_MAGIC (0xF0)	
typedef struct {
	char *task_comm;
	char flag;
} task_t;
static task_t task_white_list[] = {
	{"mmc-cmdqd", 0xF0},
	{"cpuhp", 0xF0},
	{"MtpServer", 0xF0},
	{"ipacm", 0xF0},
	{"swapon", 0xF0},
	{"pem_policy_task", 0xF0},
	{"kcompactd0", 0xF0},
	{"hdcp_2x", 0xF0},
	{"dp_hdcp2p2", 0xF0},
	{"ion-pool-uncach", 0xF0},
	{"ion-pool-cached", 0xF0},
	{"pwrap_thread", 0xF0},
	{"mdrt_thread", 0xF0},
	{"chre_kthread", 0xF0},
	{"scp_power_reset", 0xF0},
	{"ccci_fsm1", 0xF0},
	{"gauge_coulomb_t", 0xF0},
	{"gauge_timer_thr", 0xF0},
	{"mtk_drm_disp_id", 0xF0},
	{"qos_ipi_recv", 0xF0},
	{"ipi_cpu_dvfs_rt", 0xF0},
	{"amms_task", 0xF0},
	{"pcs_thd", 0xF0},
	{"sync", 0xF0},
	{"psl_thd", 0xF0},
	{"disp_idlemgr", 0xF0},
	{"ddp_irq_log_kth", 0xF0},
	{"decouple_trigge", 0xF0},
	{"disp_check", 0xF0},
	{NULL, 0},
};
static task_t task_critical_list[] = {
	{"init", 0x0F},
	{NULL, 0},
};		
void vivo_set_hung_task_critical_mark(bool enable)
{
	if (enable)
		current->hang_detection_enabled |= VIVO_HUNG_TASK_CRITICAL_MAGIC;
	else
		current->hang_detection_enabled &= ~VIVO_HUNG_TASK_CRITICAL_MAGIC;
}
void vivo_hung_task_dump_debug_info(void)
{
	show_state_filter(TASK_UNINTERRUPTIBLE);
	trigger_all_cpu_backtrace();
}
bool task_in_vivo_critical_list(struct task_struct *tsk)
{
	task_t *info = NULL;
	
	if (tsk->hang_detection_enabled & VIVO_HUNG_TASK_CRITICAL_MAGIC) {
		pr_info("hung check, critical task blocked: (%d)%s\n", tsk->pid, tsk->comm);
		return true;
	}
	for (info = task_critical_list; info->task_comm; info++) {
		if (strnstr(tsk->comm, info->task_comm, strlen(info->task_comm))) {
			pr_info("hung check, critical task blocked: (%d)%s\n", tsk->pid, tsk->comm);
			return true;
		}
	}
	return false;
}
static bool task_in_vivo_white_list(struct task_struct *tsk)
{
	task_t *info = NULL;
	if (tsk->hang_detection_enabled & VIVO_HUNG_TASK_WHITE_LIST_MAGIC)
		return true;
	for (info = task_white_list; info->task_comm; info++) {
		if (strnstr(tsk->comm, info->task_comm, strlen(info->task_comm))) {
			tsk->hang_detection_enabled |= VIVO_HUNG_TASK_WHITE_LIST_MAGIC;
			return true;
		}
	}
	return false;
}
#endif
/*
 * The number of tasks checked:
 */
int __read_mostly sysctl_hung_task_check_count = PID_MAX_LIMIT;

/*
 * Limit number of tasks checked in a batch.
 *
 * This value controls the preemptibility of khungtaskd since preemption
 * is disabled during the critical section. It also controls the size of
 * the RCU grace period. So it needs to be upper-bound.
 */
#define HUNG_TASK_LOCK_BREAK (HZ / 10)

/*
 * Zero means infinite timeout - no checking done:
 */
#ifdef VIVO_HUNG_TASK_FEATURE
unsigned long __read_mostly sysctl_hung_task_timeout_secs = VIVO_CONFIG_DEFAULT_HUNG_TASK_TIMEOUT;
#else
unsigned long __read_mostly sysctl_hung_task_timeout_secs = CONFIG_DEFAULT_HUNG_TASK_TIMEOUT;
#endif

#ifdef VIVO_HUNG_TASK_FEATURE
int __read_mostly sysctl_hung_task_warnings = INT_MAX;
#else
int __read_mostly sysctl_hung_task_warnings = 10;
#endif

static int __read_mostly did_panic;
static bool hung_task_show_lock;
static bool hung_task_call_panic;

static struct task_struct *watchdog_task;

/*
 * Should we panic (and reboot, if panic_timeout= is set) when a
 * hung task is detected:
 */
unsigned int __read_mostly sysctl_hung_task_panic =
#ifdef VIVO_HUNG_TASK_FEATURE
				VIVO_CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE;
#else
				CONFIG_BOOTPARAM_HUNG_TASK_PANIC_VALUE;
#endif

static int __init hung_task_panic_setup(char *str)
{
	int rc = kstrtouint(str, 0, &sysctl_hung_task_panic);

	if (rc)
		return rc;
	return 1;
}
__setup("hung_task_panic=", hung_task_panic_setup);

static int
hung_task_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	did_panic = 1;

	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = hung_task_panic,
};

static void check_hung_task(struct task_struct *t, unsigned long timeout)
{
	unsigned long switch_count = t->nvcsw + t->nivcsw;

	/*
	 * Ensure the task is not frozen.
	 * Also, skip vfork and any other user process that freezer should skip.
	 */
	if (unlikely(t->flags & (PF_FROZEN | PF_FREEZER_SKIP)))
	    return;

	/*
	 * When a freshly created task is scheduled once, changes its state to
	 * TASK_UNINTERRUPTIBLE without having ever been switched out once, it
	 * musn't be checked.
	 */
	if (unlikely(!switch_count))
		return;

	if (switch_count != t->last_switch_count) {
		t->last_switch_count = switch_count;
		return;
	}

	trace_sched_process_hang(t);

	if (sysctl_hung_task_panic) {
		console_verbose();
		hung_task_show_lock = true;
		hung_task_call_panic = true;
	}

#ifdef VIVO_HUNG_TASK_FEATURE
	if (task_in_vivo_white_list(t)) {
		pr_info("hung check, skip task:%s\n", t->comm);
		hung_task_show_lock = false;
		hung_task_call_panic = false;
		return;
	}
#endif
	/*
	 * Ok, the task did not get scheduled for more than 2 minutes,
	 * complain:
	 */
	if (sysctl_hung_task_warnings) {
		if (sysctl_hung_task_warnings > 0)
			sysctl_hung_task_warnings--;
#ifdef VIVO_HUNG_TASK_FEATURE
		g_hung_task = t;
#endif
		pr_err("INFO: task %s:%d blocked for more than %ld seconds.\n",
			t->comm, t->pid, timeout);
		pr_err("      %s %s %.*s\n",
			print_tainted(), init_utsname()->release,
			(int)strcspn(init_utsname()->version, " "),
			init_utsname()->version);
		pr_err("\"echo 0 > /proc/sys/kernel/hung_task_timeout_secs\""
			" disables this message.\n");
		sched_show_task(t);
		hung_task_show_lock = true;
	}

#ifndef VIVO_HUNG_TASK_FEATURE
	touch_nmi_watchdog();
#else
	if (task_in_vivo_critical_list(t)) {
		hung_task_call_panic_critical = true;
	}
#endif
}

/*
 * To avoid extending the RCU grace period for an unbounded amount of time,
 * periodically exit the critical section and enter a new one.
 *
 * For preemptible RCU it is sufficient to call rcu_read_unlock in order
 * to exit the grace period. For classic RCU, a reschedule is required.
 */
static bool rcu_lock_break(struct task_struct *g, struct task_struct *t)
{
	bool can_cont;

	get_task_struct(g);
	get_task_struct(t);
	rcu_read_unlock();
	cond_resched();
	rcu_read_lock();
	can_cont = pid_alive(g) && pid_alive(t);
	put_task_struct(t);
	put_task_struct(g);

	return can_cont;
}

/*
 * Check whether a TASK_UNINTERRUPTIBLE does not get woken up for
 * a really long time (120 seconds). If that happens, print out
 * a warning.
 */
static void check_hung_uninterruptible_tasks(unsigned long timeout)
{
	int max_count = sysctl_hung_task_check_count;
	unsigned long last_break = jiffies;
	struct task_struct *g, *t;

	/*
	 * If the system crashed already then all bets are off,
	 * do not report extra hung tasks:
	 */
	if (test_taint(TAINT_DIE) || did_panic)
		return;

	hung_task_show_lock = false;
	rcu_read_lock();
	for_each_process_thread(g, t) {
		if (!max_count--)
			goto unlock;
		if (time_after(jiffies, last_break + HUNG_TASK_LOCK_BREAK)) {
			if (!rcu_lock_break(g, t))
				goto unlock;
			last_break = jiffies;
		}

		/* use "==" to skip the TASK_KILLABLE tasks waiting on NFS */
#ifdef VIVO_HUNG_TASK_FEATURE
		if (t->state == TASK_UNINTERRUPTIBLE) {
			if (t->hang_detection_enabled == VIVO_HUNG_TASK_WHITE_LIST_MAGIC) {
				pr_debug("hung check, skip task:%s\n", t->comm);
				continue;
			} else {
				pr_info("hung check, checking task:%s\n", t->comm);
				check_hung_task(t, timeout);
			}
		}
#else
		if (t->state == TASK_UNINTERRUPTIBLE)
			check_hung_task(t, timeout);
#endif
	}
 unlock:
	rcu_read_unlock();
	if (hung_task_show_lock)
		debug_show_all_locks();
#ifdef VIVO_HUNG_TASK_FEATURE
	if (hung_task_call_panic && download_mode) {
		trigger_all_cpu_backtrace();
		if (hung_task_show_lock)
			panic("hung_task: blocked tasks");
	} else if (hung_task_call_panic_critical) {
		vivo_hung_task_dump_debug_info();
		panic("hung_task: blocked tasks");	
	}
#else
	if (hung_task_call_panic) {
		trigger_all_cpu_backtrace();
		panic("hung_task: blocked tasks");
	}
#endif
}

static long hung_timeout_jiffies(unsigned long last_checked,
				 unsigned long timeout)
{
	/* timeout of 0 will disable the watchdog */
	return timeout ? last_checked - jiffies + timeout * HZ :
		MAX_SCHEDULE_TIMEOUT;
}

/*
 * Process updating of timeout sysctl
 */
int proc_dohung_task_timeout_secs(struct ctl_table *table, int write,
				  void __user *buffer,
				  size_t *lenp, loff_t *ppos)
{
	int ret = 0;

#ifdef VIVO_HUNG_TASK_FEATURE
	if (!write)
		ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		goto out;

#else
	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);

	if (ret || !write)
		goto out;

	wake_up_process(watchdog_task);
#endif
 out:
	return ret;
}

static atomic_t reset_hung_task = ATOMIC_INIT(0);

void reset_hung_task_detector(void)
{
	atomic_set(&reset_hung_task, 1);
}
EXPORT_SYMBOL_GPL(reset_hung_task_detector);

static bool hung_detector_suspended;

static int hungtask_pm_notify(struct notifier_block *self,
			      unsigned long action, void *hcpu)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
		hung_detector_suspended = true;
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		hung_detector_suspended = false;
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

/*
 * kthread which checks for tasks stuck in D state
 */
static int watchdog(void *dummy)
{
	unsigned long hung_last_checked = jiffies;

	set_user_nice(current, 0);

	pr_info("khungtaskd start...\n");
	for ( ; ; ) {
		unsigned long timeout = sysctl_hung_task_timeout_secs;
		long t = hung_timeout_jiffies(hung_last_checked, timeout);
		if (t <= 0) {
			if (!atomic_xchg(&reset_hung_task, 0) &&
			    !hung_detector_suspended)
				check_hung_uninterruptible_tasks(timeout);
			hung_last_checked = jiffies;
			continue;
		}
		schedule_timeout_interruptible(t);
	}

	return 0;
}

static int __init hung_task_init(void)
{
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	/* Disable hung task detector on suspend */
	pm_notifier(hungtask_pm_notify, 0);

	watchdog_task = kthread_run(watchdog, NULL, "khungtaskd");

	return 0;
}
subsys_initcall(hung_task_init);
#ifdef VIVO_HUNG_TASK_FEATURE
#include <linux/module.h>
static int hung_task_test(const char *val, const struct kernel_param *kp)
{
	unsigned long sec = 0;
	if (kstrtoul(val, 10, &sec))
		return -EINVAL;
	pr_info("Hung task Dsleep %ld s, start...\n", sec);
	msleep(sec*1000);
	pr_info("Hung task Dsleep %ld s, end\n", sec);
	return 0;
}
static const struct kernel_param_ops hung_task_test_ops = {
	.set = hung_task_test,
};
module_param_cb(test, &hung_task_test_ops, NULL, 0600);
#endif
