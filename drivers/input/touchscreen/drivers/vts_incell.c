#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/version.h>

#include "vts_core.h"

#define callback(incell, func, display_id) do { \
		int ret = 0; \
		struct vts_device *vtsdev = incell_to_vts(incell); \
		vts_dev_info(vtsdev, "%s\n", #func); \
		if (incell && incell->ops && incell->ops->func) \
			ret = incell->ops->func(incell, display_id); \
		if (ret) { \
			vts_dev_err(vtsdev, "incell callback %s error, display_id = %d, ret = %d", #func, display_id, ret); \
		} \
	} while (0)

enum vts_lcm_event {
	LCM_RESUME = 1,
	LCM_EARLY_RESUME,
	LCM_SUSPEND,
	LCM_EARLY_SUSPEND
};

static int vts_lcm_state_process(struct vts_incell *incell, unsigned long event, int blank, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);
	static int last_lcm_state = 0;
	int set_blank = 0;

	vts_property_get(vtsdev, VTS_PROPERTY_SET_BLANK, &set_blank);

	vts_dev_info(vtsdev, "event:%ld, blank:%d, display_id:%d\n", event, blank, display_id);
	if (display_id != 0) {
		vts_dev_info(vtsdev, "display id is non-zero, not phone display, do nothing\n");
		return 0;
	}

	if (event == VTS_EVENT_BLANK && (set_blank ? blank == VTS_SPECIAL_BLANK_UNBLANK : blank == VTS_BLANK_UNBLANK)) {
		if (last_lcm_state == LCM_RESUME) {
			VTI("same lcm state：resume");
			return 0;
		}
		last_lcm_state = LCM_RESUME;
		callback(incell, resume, display_id);
		vts_device_unlock(vtsdev);
		vtsdev->policy->incell_proc(vtsdev, event, blank, display_id);
		__pm_stay_awake(incell->lcd_wakelock);
	} else if (event == VTS_EARLY_EVENT_BLANK && blank == VTS_BLANK_UNBLANK) {
		if (last_lcm_state == LCM_EARLY_RESUME) {
			VTI("same lcm state：early_resume");
			return 0;
		}
		last_lcm_state = LCM_EARLY_RESUME;
		vtsdev->policy->incell_proc(vtsdev, event, blank, display_id);
		vts_device_lock(vtsdev);
		callback(incell, early_resume, display_id);
	} else if (event == VTS_EVENT_BLANK && blank == VTS_BLANK_POWERDOWN) {
		if (last_lcm_state == LCM_SUSPEND) {
			VTI("same lcm state：suspend");
			return 0;
		}
		last_lcm_state = LCM_SUSPEND;
		callback(incell, suspend, display_id);
		vts_device_unlock(vtsdev);
		vtsdev->policy->incell_proc(vtsdev, event, blank, display_id);
	} else if (event == VTS_EARLY_EVENT_BLANK && blank == VTS_BLANK_POWERDOWN){
		if (last_lcm_state == LCM_EARLY_SUSPEND) {
			VTI("same lcm state：early_suspend");
			return 0;
		}
		last_lcm_state = LCM_EARLY_SUSPEND;
		__pm_relax(incell->lcd_wakelock);
		vtsdev->policy->incell_proc(vtsdev, event, blank, display_id);
		vts_device_lock(vtsdev);
		callback(incell, early_suspend, display_id);
	}
	return 0;
}

static int vts_lcm_state_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct vts_incell *incell = container_of(self, struct vts_incell, nb);
	int blank;
	#ifndef MTK_DRM_NOTIFY
	display_notifier *evdata = data;
	if(evdata == NULL){
		VTE("evdata is NULL");
		return 0;
	}
	if(evdata->data == NULL){
		VTE("evdata->data is NULL");
		return 0;	
	}
	if(IS_ERR(evdata) || IS_ERR(evdata->data)){
		VTE("evdata or evdata->data is ERR point");
		return 0;	
	}
	blank = *(int *)evdata->data;
	return vts_lcm_state_process(incell, event, blank, vts_get_display_id(evdata));
	#else 
	if(data == NULL){
		VTE("data is ERR point");
		return 0;	
	}
	blank = *(int *)data;
	return vts_lcm_state_process(incell, event, blank, 0);
	#endif
}

static int incell_early_suspend(struct vts_incell *incell, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);
	int ret = 0;

	ret = vts_call_ic_ops(vtsdev, early_suspend);
	if (ret) {
		vts_dev_err(vtsdev, "early_suspend run error!");
		return ret;
	}

	return 0;
}

static void incell_suspend_work(struct work_struct *work)
{
	struct vts_incell *incell = container_of(work, struct vts_incell, suspend_work);
	struct vts_device *vtsdev = incell_to_vts(incell);

	vts_dev_dbg(vtsdev, "step3:suspend work execute start\n");
	vts_state_set_sync(vtsdev, VTS_STA_TDDI_LCD, 0);
	up(&incell->sem);
	vts_dev_dbg(vtsdev, "step4:suspend work execute finished\n");
}

static int incell_suspend(struct vts_incell *incell, int display_id)
{
	struct vts_device *vtsdev = incell_to_vts(incell);
	int ret;

	vts_dev_dbg(vtsdev, "step1:enqueue suspend work\n");
	queue_work(incell->wq, &incell->suspend_work);
	vts_dev_dbg(vtsdev, "step2:wait suspend work start\n");
	ret = down_timeout(&incell->sem, 2*HZ);
	vts_dev_dbg(vtsdev, "step5:wait suspend work finished, ret = %d\n", ret);
	return ret;
}

static int incell_early_resume(struct vts_incell *incell, int display_id)
{
	 struct vts_device *vtsdev = incell_to_vts(incell);
	 int ret = 0;

	 ret = vts_call_ic_ops(vtsdev, early_resume);
	 if (ret) {
		vts_dev_err(vtsdev, "early_resume run error!");
		return ret;
	 }

	 return 0;
}

static int incell_resume(struct vts_incell *incell, int display_id)
{
	 struct vts_device *vtsdev = incell_to_vts(incell);
	 return vts_state_set(vtsdev, VTS_STA_TDDI_LCD, 1);
}

static const struct vts_incell_ops incell_ops = {
	.early_suspend = incell_early_suspend,
	.suspend = incell_suspend,
	.early_resume = incell_early_resume,
	.resume = incell_resume,
};

int vts_incell_init(struct vts_device *vtsdev)
{
	struct vts_incell *incell = &vtsdev->incell;
	u32 tddi;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);
	if (!tddi){
		VTI("not incell!!!!!!!!!!!");
		return 0;
		}
	incell->wq = create_singlethread_workqueue(vts_name(vtsdev));
	if (!incell->wq) {
		vts_dev_err(vtsdev, "create workqueue error!\n");
		return -ENOMEM;
	}

	incell->lcd_wakelock = vts_wakelock_register(vtsdev, vts_name(vtsdev));
	if (!incell->lcd_wakelock) {
		vts_dev_err(vtsdev, "lcd wakelock init fail!\n");
		return -ENOMEM;
	}

	INIT_WORK(&incell->suspend_work, incell_suspend_work);
	sema_init(&incell->sem, 0);
	incell->ops = &incell_ops;
	incell->nb.notifier_call = vts_lcm_state_callback;

#if defined(MTK_DRM_NOTIFY)
	return vts_lcm_register_client("vts_incell", &incell->nb);
#elif (defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	if (vtsdev->module->vts_lcm_module_active) {
		if(vtsdev->module->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			return vts_lcm_register_client(vtsdev->module->active_panel_v2, &incell->nb);
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	} else {
		if(vtsdev->active_panel_v2){
			VTI("active_panel_v2 ok ,callback_register success");
			return vts_lcm_register_client(vtsdev->active_panel_v2, &incell->nb);
		}
		else{
			VTI("active_pane_v2 == null ,callback register fail");
			return 0;
		}
	}
#else
	return vts_lcm_register_client(&incell->nb);
#endif
}

int vts_incell_deinit(struct vts_device *vtsdev)
{
	struct vts_incell *incell = &vtsdev->incell;
	int ret = 0;
	u32 tddi;

	vts_property_get(vtsdev, VTS_PROPERTY_TDDI, &tddi);

	if (!tddi)
		return 0;

	vts_wakelock_unregister(incell->lcd_wakelock);
	incell->lcd_wakelock = NULL;	

#if (!defined(MTK_DRM_NOTIFY) && defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	VTI("active_panel_v2  deinit ");
	if(vtsdev->module->active_panel_v2)
	 ret = vts_lcm_unregister_client(vtsdev->module->active_panel_v2, &incell->nb);
#else
	 ret = vts_lcm_unregister_client(&incell->nb);
#endif
	incell->nb.notifier_call = NULL;
	incell->ops = NULL;
	destroy_workqueue(incell->wq);
	return ret;
}
