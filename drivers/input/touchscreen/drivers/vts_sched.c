#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <../kernel/sched/sched.h>
#include "vts_core.h"

#if IS_ENABLED(CONFIG_INPUT_BOOST)
#if IS_ENABLED(CONFIG_RSC_LPM_MANAGER)
#include <linux/vivo_rsc/rsc_cpu_internal.h>
#endif

#if IS_ENABLED(CONFIG_DDR_BW_REQUEST)
#include <linux/devfreq.h>
#endif
#endif

struct vts_max_cpu {
	struct list_head entry;
	int max_cpu_id;
};

enum vts_idle_type {
	VTS_IDLE_0,
	VTS_IDLE_1,
	VTS_IDLE_MAX
};

enum vts_sched_flags_e {
	FLAGS_CPU_FREQ_BOOST = BIT(0),
	FLAGS_DDR_FREQ_BOOST = BIT(1),
	FLAGS_BIND_MAX_CPU_BOOST = BIT(2),
};

#define LPM_TIMEOUT_DEFAULT 5
#define DDR_TIMEOUT_DEFAULT 20

static LIST_HEAD(vts_sched_maxcpu);
static DEFINE_MUTEX(maxcpu_mutex);

int vts_sched_set_flags(struct vts_device *vtsdev, unsigned long flags)
{
#if !IS_ENABLED(CONFIG_INPUT_BOOST)
	if(flags & (FLAGS_CPU_FREQ_BOOST | FLAGS_DDR_FREQ_BOOST | FLAGS_BIND_MAX_CPU_BOOST)) {
		vts_dev_info(vtsdev, "not support boost policy\n");
		return -EPERM;
	}
#endif
	vtsdev->sched.flags = flags;
	return 0;
}

unsigned long vts_sched_flags(struct vts_device *vtsdev)
{
	return vtsdev->sched.flags;
}

int vts_sched_set_lpm_timeout(struct vts_device *vtsdev, unsigned long timeout)
{
	vtsdev->sched.lpm_timeout = timeout;
	return 0;
}

unsigned long vts_sched_lpm_timeout(struct vts_device *vtsdev)
{
	return vtsdev->sched.lpm_timeout;
}

int vts_sched_set_ddr_timeout(struct vts_device *vtsdev, unsigned long timeout)
{
	vtsdev->sched.ddr_timeout = timeout;
	return 0;
}

unsigned long vts_sched_ddr_timeout(struct vts_device *vtsdev)
{
	return vtsdev->sched.ddr_timeout;
}

#if IS_ENABLED(CONFIG_INPUT_BOOST)
extern struct atomic_notifier_head input_boost_list;

enum vts_boost_state vts_get_boost_state(struct vts_device *vtsdev)
{
	return atomic_read(&vtsdev->boost_state);
}

void vts_set_boost_state(struct vts_device *vtsdev, enum vts_boost_state state)
{
	atomic_set(&vtsdev->boost_state, state);
}

void vts_boost_enable(struct vts_device *vtsdev, int nr_touches)
{
	if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL && vts_state_get(vtsdev, VTS_STA_GAME_MODE))
		if (nr_touches == 0)
			vts_set_boost_state(vtsdev, VTS_BOOST_ENABLE);
}

static enum hrtimer_restart vts_lpm_enable(struct hrtimer *timer)
{
#if IS_ENABLED(CONFIG_RSC_LPM_MANAGER)
	rsc_set_lpm_disabled_e(false);
#endif
	return HRTIMER_NORESTART;
}

static int vts_lpm_disable(struct vts_sched *sched)
{
	ktime_t ktime;
	u64 ktime_quot;
	u32 ktime_mod;

	hrtimer_cancel(&sched->timer);
#if IS_ENABLED(CONFIG_RSC_LPM_MANAGER)
	rsc_set_lpm_disabled_e(true);
#endif
	ktime_quot = sched->lpm_timeout;
	ktime_mod = do_div(ktime_quot, 1000);
	ktime = ktime_set((s64)ktime_quot, ktime_mod * 1000000);
	hrtimer_start(&sched->timer, ktime, HRTIMER_MODE_REL);
	return 0;
}

static void vts_ddr_boost_work_rem(struct work_struct *work)
{
#if IS_ENABLED(CONFIG_DDR_BW_REQUEST)
	ddr_bw_disable();
#endif
}

static void vts_ddr_boost_work(struct work_struct *work)
{
	struct vts_sched *sched = container_of(work, struct vts_sched, ddr_work);

	cancel_delayed_work_sync(&sched->ddr_work_rem);
#if IS_ENABLED(CONFIG_DDR_BW_REQUEST)
	ddr_bw_enable();
#endif
	queue_delayed_work(sched->wq, &sched->ddr_work_rem, msecs_to_jiffies(sched->ddr_timeout));
}

static int vts_ddr_boost(struct vts_sched *sched)
{
	if (work_pending(&sched->ddr_work))
		return 0;

	queue_work(sched->wq, &sched->ddr_work);

	return 0;
}

static void vts_lpm_disable_boost(struct vts_sched *sched)
{
	if (sched->flags & FLAGS_CPU_FREQ_BOOST)
		vts_lpm_disable(sched);
}

static void vts_cpu_freq_boost(struct vts_sched *sched)
{
	if (sched->flags & FLAGS_CPU_FREQ_BOOST)
		atomic_notifier_call_chain(&input_boost_list, 0, NULL);
}

static void vts_ddr_freq_boost(struct vts_sched *sched)
{
	if (sched->flags & FLAGS_DDR_FREQ_BOOST)
		vts_ddr_boost(sched);
}

static void vts_bind_max_cpu_boost(struct vts_sched *sched)
{
	if (sched->flags & FLAGS_BIND_MAX_CPU_BOOST) {
		if (likely(sched->max_cpu_num > 2)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
			vts_up(&sched->sem0);
			vts_up(&sched->sem1);
#else
			up(&sched->sem0);
			up(&sched->sem1);
#endif
		}
	}
}

void vts_boost(struct vts_device *vtsdev)
{
	struct vts_sched *sched = &vtsdev->sched;

	if (vts_get_run_mode(vtsdev) == VTS_ST_NORMAL && vts_state_get(vtsdev, VTS_STA_GAME_MODE)) {
		if (vts_get_boost_state(vtsdev) == VTS_BOOST_ENABLE) {
			vts_lpm_disable_boost(sched);
			vts_bind_max_cpu_boost(sched);
			vts_cpu_freq_boost(sched);
			vts_ddr_freq_boost(sched);
			vts_set_boost_state(vtsdev, VTS_BOOST_DISABLE);
		}
	}
}

static int vts_idle_thread0(void *dev)
{
	struct vts_device *vtsdev = (struct vts_device *)dev;
	struct vts_sched *sched = &vtsdev->sched;

	static struct sched_param para = {
		.sched_priority = 60,
	};

	sched_setscheduler(current, SCHED_FIFO, &para);
	while (likely(!kthread_should_stop())) {

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		vts_down_interruptible(&sched->sem0);
#else
		down_interruptible(&sched->sem0);
#endif
		__pm_stay_awake(sched->idle_wakelock0);
		vts_dev_dbg(vtsdev, "vts_idle_thread0 running\n");
		__pm_relax(sched->idle_wakelock0);
	}
	vts_wakelock_unregister(sched->idle_wakelock0);
	sched->idle_wakelock0 = NULL;

	return 0;
}

static int vts_idle_thread1(void *dev)
{
	struct vts_device *vtsdev = (struct vts_device *)dev;
	struct vts_sched *sched = &vtsdev->sched;

	static struct sched_param para = {
		.sched_priority = 60,
	};

	sched_setscheduler(current, SCHED_FIFO, &para);
	while (likely(!kthread_should_stop())) {

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		vts_down_interruptible(&sched->sem1);
#else
		down_interruptible(&sched->sem1);
#endif
		__pm_stay_awake(sched->idle_wakelock1);
		vts_dev_dbg(vtsdev, "vts_idle_thread1 running\n");
		__pm_relax(sched->idle_wakelock1);
	}
	vts_wakelock_unregister(sched->idle_wakelock1);
	sched->idle_wakelock1 = NULL;

	return 0;
}

typedef int (*vts_idle_handler)(void *dev);
static const vts_idle_handler vts_idle_handlers[VTS_IDLE_MAX] = {
	[VTS_IDLE_0] = vts_idle_thread0,
	[VTS_IDLE_1] = vts_idle_thread1
};

static vts_sched_wakelock_init(struct vts_device *vtsdev)
{
	struct vts_sched *sched = &vtsdev->sched;

	snprintf(sched->idle_wakelock_name0, sizeof(sched->idle_wakelock_name0), "vtsidle_%d0", vtsdev->type);
	sched->idle_wakelock0 = vts_wakelock_register(vtsdev, sched->idle_wakelock_name0);
	if (!sched->idle_wakelock0) {
		vts_dev_err(vtsdev, "idle_wakelock0 init fail\n");
		return -ENOMEM;
	}

	snprintf(sched->idle_wakelock_name1, sizeof(sched->idle_wakelock_name1), "vtsidle_%d1", vtsdev->type);
	sched->idle_wakelock1 = vts_wakelock_register(vtsdev, sched->idle_wakelock_name1);
	if (!sched->idle_wakelock1) {
		vts_dev_err(vtsdev, "idle_wakelock1 init fail!\n");
		return -ENOMEM;
	}

	return 0;
}

void vts_sched_wakelock_deinit(struct vts_device *vtsdev)
{
	struct vts_sched *sched = &vtsdev->sched;

	vts_wakelock_unregister(sched->idle_wakelock0);
	vts_wakelock_unregister(sched->idle_wakelock1);
	sched->idle_wakelock0 = NULL;
	sched->idle_wakelock1 = NULL;
}

static int vts_find_maxcpu(struct vts_device *vtsdev)
{
	int cpu;
	struct vts_max_cpu *max_cpu;
	struct vts_sched *sched = &vtsdev->sched;

	mutex_lock(&maxcpu_mutex);
	for_each_possible_cpu(cpu) {
		if(!is_min_capacity_cpu(cpu)) {
			max_cpu = kzalloc(sizeof(*max_cpu), GFP_KERNEL);
			if (!max_cpu) {
				vts_dev_err(vtsdev, "no memory!\n");
				continue;
			}
			sched->max_cpu_num ++;
			max_cpu->max_cpu_id = cpu;
			INIT_LIST_HEAD(&max_cpu->entry);
			list_add_tail(&max_cpu->entry, &vts_sched_maxcpu);
		}
	}

	if (list_empty(&vts_sched_maxcpu)) {
		vts_dev_err(vtsdev, "error, no maxcpu!");
		mutex_unlock(&maxcpu_mutex);
		return -EINVAL;
	}

	if (unlikely(sched->max_cpu_num < 2)) {
		vts_dev_err(vtsdev, "maxcpu less!");
		mutex_unlock(&maxcpu_mutex);
		return -EINVAL;
	}

	vts_dev_info(vtsdev, "max_cpu list:\n");
	list_for_each_entry(max_cpu, &vts_sched_maxcpu, entry) {
		vts_dev_info(vtsdev, "cpu:%d\n", max_cpu->max_cpu_id);
	}
	mutex_unlock(&maxcpu_mutex);

	return 0;
}

static int vts_bind_maxcpu(struct vts_device *vtsdev, struct vts_max_cpu *max_cpu, struct task_struct *idle)
{
	cpumask_var_t new_mask;

	if (!idle)
		return -EINVAL;

	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL))
		return -ENOMEM;

	cpumask_clear(new_mask);
	cpumask_set_cpu(max_cpu->max_cpu_id, new_mask);
	sched_setaffinity(idle->pid, new_mask);
	free_cpumask_var(new_mask);
	vts_dev_info(vtsdev, "bind max_cpu:%*pbl\n", cpumask_pr_args(new_mask));

	return 0;
}

static int vts_idle_thread_init(struct vts_device *vtsdev)
{
	int i;
	int ret = 0;
	struct vts_max_cpu *max_cpu;
	struct vts_sched *sched = &vtsdev->sched;

	sema_init(&sched->sem0, 0);
	sema_init(&sched->sem1, 0);

	mutex_lock(&maxcpu_mutex);
	for (i = 0; i < VTS_IDLE_MAX; i ++) {
		sched->idle[i] = kthread_run(vts_idle_handlers[i], vtsdev, "idlethread_%d%d", vtsdev->type, i);
		if (IS_ERR(sched->idle[i])) {
			vts_dev_err(vtsdev, "create idlethread_%d%d error!", vtsdev->type, i);
			ret = PTR_ERR(sched->idle[i]);
			mutex_unlock(&maxcpu_mutex);
			return ret;
		}
		sched->idle_num ++;

		max_cpu = list_first_entry(&vts_sched_maxcpu, struct vts_max_cpu, entry);
		ret = vts_bind_maxcpu(vtsdev, max_cpu, sched->idle[i]);
		if (ret) {
			vts_dev_err(vtsdev, "bind idlethread_%d%d to max_cpu:%d fail!", vtsdev->type, i, max_cpu->max_cpu_id);
			mutex_unlock(&maxcpu_mutex);
			return ret;
		}
		list_del(&max_cpu->entry);
		kfree(max_cpu);
	}
	mutex_unlock(&maxcpu_mutex);

	return ret;
}

static int vts_ddr_boost_init(struct vts_device *vtsdev)
{
	struct vts_sched *sched = &vtsdev->sched;

	if (!(sched->flags & FLAGS_DDR_FREQ_BOOST)) {
		vts_dev_info(vtsdev, "not ddr boost");
		return 0;
	}

	sched->wq = alloc_workqueue("vtsboost_wq", WQ_HIGHPRI, 0);
	if (!sched->wq) {
		vts_dev_info(vtsdev, "create vtsboost_wq fail");
		return -EFAULT;
	}

	INIT_WORK(&sched->ddr_work, vts_ddr_boost_work);
	INIT_DELAYED_WORK(&sched->ddr_work_rem, vts_ddr_boost_work_rem);
	sched->ddr_timeout = DDR_TIMEOUT_DEFAULT;

	return 0;
}

static int vts_cpu_boost_init(struct vts_device *vtsdev)
{
	struct vts_sched *sched = &vtsdev->sched;

	if (!(sched->flags & FLAGS_CPU_FREQ_BOOST)) {
		vts_dev_info(vtsdev, "not cpu boost");
		return 0;
	}

	hrtimer_init(&sched->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sched->timer.function = vts_lpm_enable;
	sched->lpm_timeout = LPM_TIMEOUT_DEFAULT;

	return 0;
}

static int vts_bind_max_cpu_boost_init(struct vts_device *vtsdev)
{
	int ret = 0;
	struct vts_sched *sched = &vtsdev->sched;

	if (!(sched->flags & FLAGS_BIND_MAX_CPU_BOOST)) {
		vts_dev_info(vtsdev, "not bind max cpu boost");
		return 0;
	}

	ret = vts_sched_wakelock_init(vtsdev);
	if (ret)
		return ret;

	ret = vts_find_maxcpu(vtsdev);
	if (ret)
		return ret;

	ret = vts_idle_thread_init(vtsdev);
	if (ret)
		return ret;

	return 0;
}

int vts_boost_init(struct vts_device *vtsdev)
{
	int ret;
	int boost;
	struct vts_sched *sched = &vtsdev->sched;

	vts_property_get(vtsdev, VTS_PROPERTY_BOOST, &boost);
	sched->flags = boost;
	if (!sched->flags) {
		vts_dev_info(vtsdev, "not boost");
		return 0;
	}

	ret = vts_cpu_boost_init(vtsdev);
	if (ret)
		return ret;

	ret = vts_ddr_boost_init(vtsdev);
	if (ret)
		return ret;

	ret = vts_bind_max_cpu_boost_init(vtsdev);
	if (ret)
		return ret;

	vts_set_boost_state(vtsdev, VTS_BOOST_ENABLE);

	return 0;
}

void vts_boost_deinit(struct vts_device *vtsdev)
{
	struct vts_max_cpu *max_cpu;
	struct vts_max_cpu *next;
	struct vts_sched *sched = &vtsdev->sched;

	if (list_empty(&vts_sched_maxcpu))
		return;

	list_for_each_entry_safe(max_cpu, next, &vts_sched_maxcpu, entry) {
		list_del(&max_cpu->entry);
		kfree(max_cpu);
	}

	if (sched->idle_num > 0)
		vts_kthread_stop(&sched->sem0, sched->idle[0]);
	if (sched->idle_num > 1)
		vts_kthread_stop(&sched->sem1, sched->idle[1]);

	sched->max_cpu_num = 0;
	destroy_workqueue(sched->wq);
	vts_sched_wakelock_deinit(vtsdev);
}
#endif
