#ifndef __VTS_INCELL_H
#define __VTS_INCELL_H

#include <linux/version.h>

struct vts_incell {
	struct notifier_block nb;
	const struct vts_incell_ops *ops;
	struct wakeup_source *lcd_wakelock;
	struct workqueue_struct *wq;
	struct work_struct suspend_work;
	struct semaphore sem;
};

#define VTS_SPECIAL_BLANK_UNBLANK 16

extern int mtk_disp_notifier_register(const char *source, struct notifier_block *nb);
extern int mtk_disp_notifier_unregister(struct notifier_block *nb);


#if defined(MTK_DRM_NOTIFY)
#include <mtk_disp_notify.h>
#include <drm/drm_panel.h>
#define VTS_BLANK_UNBLANK		MTK_DISP_BLANK_UNBLANK
#define VTS_BLANK_POWERDOWN		MTK_DISP_BLANK_POWERDOWN
#define VTS_EVENT_BLANK			MTK_DISP_EVENT_BLANK
#define VTS_EARLY_EVENT_BLANK	MTK_DISP_EARLY_EVENT_BLANK
#define vts_lcm_register_client mtk_disp_notifier_register
#define vts_lcm_unregister_client mtk_disp_notifier_unregister

#elif defined(CONFIG_ARCH_QCOM) && !defined(CONFIG_ARCH_SDM439) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)  && LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
#include <linux/msm_drm_notify.h>
#define VTS_BLANK_UNBLANK		MSM_DRM_BLANK_UNBLANK
#define VTS_BLANK_POWERDOWN		MSM_DRM_BLANK_POWERDOWN
#define VTS_EVENT_BLANK			MSM_DRM_EVENT_BLANK
#define VTS_EARLY_EVENT_BLANK	MSM_DRM_EARLY_EVENT_BLANK
#define vts_lcm_register_client msm_drm_register_client
#define vts_lcm_unregister_client msm_drm_unregister_client
typedef struct msm_drm_notifier display_notifier;
static inline int vts_get_display_id(display_notifier *notifier)
{
	return notifier->id;
}

#elif defined(CONFIG_ARCH_QCOM) && LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)  //QCOM kernel version > =4.19 use DRM_PANEL
#include <drm/drm_panel.h>
#define VTS_BLANK_UNBLANK		DRM_PANEL_BLANK_UNBLANK
#define VTS_BLANK_POWERDOWN		DRM_PANEL_BLANK_POWERDOWN //DRM_PANEL_EVENT_BLANK
#define VTS_EVENT_BLANK			DRM_PANEL_EVENT_BLANK
#define VTS_EARLY_EVENT_BLANK	DRM_PANEL_EARLY_EVENT_BLANK
#define vts_lcm_register_client drm_panel_notifier_register 
#define vts_lcm_unregister_client drm_panel_notifier_unregister
typedef struct drm_panel_notifier display_notifier;
static inline int vts_get_display_id(display_notifier *notifier)
{
	return 0;
}

#else
#include <linux/fb.h>
#define VTS_BLANK_UNBLANK              FB_BLANK_UNBLANK
#define VTS_BLANK_POWERDOWN            FB_BLANK_POWERDOWN
#define VTS_EVENT_BLANK                        FB_EVENT_BLANK
#define VTS_EARLY_EVENT_BLANK  FB_EARLY_EVENT_BLANK
#define vts_lcm_register_client fb_register_client
#define vts_lcm_unregister_client fb_unregister_client
typedef struct fb_event display_notifier;
static inline int vts_get_display_id(display_notifier *notifier)
{
       return 0;
}
#endif

#endif
