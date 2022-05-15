#define LOG_TAG         "I2CDrv"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_sysfs.h"
#include "cts_charger_detect.h"
#include "cts_earjack_detect.h"
#include "cts_strerror.h"
#include <linux/pm_runtime.h>

#include "../vts_core.h"
extern int cts_suspend(struct chipone_ts_data *cts_data);
extern int cts_resume(struct chipone_ts_data *cts_data);
extern int cts_plat_process_touch_msg(struct cts_platform_data *pdata, struct cts_device_touch_msg *msgs, int num, ktime_t kt);

extern int bbk_chipone_mode_change(struct vts_device *vtsdev, int which);
extern int bbk_chipone_set_charger_bit(struct vts_device *vtsdev, int state);
extern int bbk_chipone_fw_update(struct vts_device *vtsdev, const struct firmware *firmware);
extern int bbk_chipone_get_fw_version(struct vts_device *vtsdev, u64 *fw_ver);
extern int chipone_setEdgeRestainSwitch(struct vts_device *vtsdev, int on);
extern int  chipone_get_frame(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size);
extern int bbk_chipone_get_ic_work_mode(struct vts_device *vtsdev);
extern int cts_set_gesture(struct vts_device *vtsdev, int enable);
extern int cts_idle_write(struct vts_device *vtsdev, int state);


/**********************************************/
struct chipone_ts_data *cts_data = NULL;
/*********************************************/

bool cts_show_debug_log = false;
module_param_named(debug_log, cts_show_debug_log, bool, 0660);
MODULE_PARM_DESC(debug_log, "Show debug log control");

int cts_suspend(struct chipone_ts_data *cts_data)
{
    int ret;

    cts_info("Suspend");

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_suspend_device(&cts_data->cts_dev);
    cts_unlock_device(&cts_data->cts_dev);

    if (ret) {
        cts_err("Suspend device failed %d", ret);
        // TODO:
        //return ret;
    }

    ret = cts_stop_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Stop device failed %d", ret);
        return ret;
    }

#ifdef CFG_CTS_GESTURE
    /* Enable IRQ wake if gesture wakeup enabled */
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_enable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_err("Enable IRQ wake failed %d", ret);
            return ret;
        }
        ret = cts_plat_enable_irq(cts_data->pdata);
        if (ret){
            cts_err("Enable IRQ failed %d",ret);
            return ret;
        }
    }
#endif /* CFG_CTS_GESTURE */

    /** - To avoid waking up while not sleeping,
            delay 20ms to ensure reliability */
    msleep(20);

    return 0;
}

int cts_resume(struct chipone_ts_data *cts_data)
{
    int ret;

    cts_info("Resume");

#ifdef CFG_CTS_GESTURE
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_disable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_warn("Disable IRQ wake failed %d", ret);
            //return ret;
        }
        if ((ret = cts_plat_disable_irq(cts_data->pdata)) < 0) {
            cts_err("Disable IRQ failed %d", ret);
            //return ret;
        }
    }
#endif /* CFG_CTS_GESTURE */

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_resume_device(&cts_data->cts_dev);
    cts_unlock_device(&cts_data->cts_dev);
    if(ret) {
        cts_warn("Resume device failed %d", ret);
        return ret;
    }

    ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        return ret;
    }
    return 0;
}

/*
#ifdef CONFIG_CTS_PM_FB_NOTIFIER
#ifdef CFG_CTS_DRM_NOTIFIER
static int fb_notifier_callback(struct notifier_block *nb,
        unsigned long action, void *data)
{
    volatile int blank;
    const struct cts_platform_data *pdata =
        container_of(nb, struct cts_platform_data, fb_notifier);
    struct chipone_ts_data *cts_data =
        container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
    struct fb_event *evdata = data;

    cts_info("FB notifier callback");

    if (evdata && evdata->data) {
        if (action == MSM_DRM_EVENT_BLANK) {
            blank = *(int *)evdata->data;
            if (blank == MSM_DRM_BLANK_UNBLANK) {
                cts_resume(cts_data);
                return NOTIFY_OK;
            }
        } else if (action == MSM_DRM_EARLY_EVENT_BLANK) {
            blank = *(int *)evdata->data;
            if (blank == MSM_DRM_BLANK_POWERDOWN) {
                cts_suspend(cts_data);
                return NOTIFY_OK;
            }
        }
    }

    return NOTIFY_DONE;
}
#else
static int fb_notifier_callback(struct notifier_block *nb,
        unsigned long action, void *data)
{
    volatile int blank;
    const struct cts_platform_data *pdata =
        container_of(nb, struct cts_platform_data, fb_notifier);
    struct chipone_ts_data *cts_data =
        container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
    struct fb_event *evdata = data;

    cts_info("FB notifier callback");

    if (evdata && evdata->data) {
        if (action == FB_EVENT_BLANK) {
            blank = *(int *)evdata->data;
            if (blank == FB_BLANK_UNBLANK) {
                cts_resume(cts_data);
                return NOTIFY_OK;
            }
        } else if (action == FB_EARLY_EVENT_BLANK) {
            blank = *(int *)evdata->data;
            if (blank == FB_BLANK_POWERDOWN) {
                cts_suspend(cts_data);
                return NOTIFY_OK;
            }
        }
    }

    return NOTIFY_DONE;
}
#endif
*/


/*
static int cts_init_pm_fb_notifier(struct chipone_ts_data * cts_data)
{
    cts_info("Init FB notifier");

    cts_data->pdata->fb_notifier.notifier_call = fb_notifier_callback;

#ifdef CFG_CTS_DRM_NOTIFIER
    return msm_drm_register_client(&cts_data->pdata->fb_notifier);
#else
    return fb_register_client(&cts_data->pdata->fb_notifier);
#endif
}

static int cts_deinit_pm_fb_notifier(struct chipone_ts_data * cts_data)
{
    cts_info("Deinit FB notifier");
#ifdef CFG_CTS_DRM_NOTIFIER
    return msm_drm_unregister_client(&cts_data->pdata->fb_notifier)
#else
    return fb_unregister_client(&cts_data->pdata->fb_notifier);
#endif
}
#endif // CONFIG_CTS_PM_FB_NOTIFIER 
*/

extern int cts_vendor_init(struct chipone_ts_data *cts_data);
extern int cts_vendor_deinit(struct chipone_ts_data *cts_data);

/*vivo panshan add */
int cts_plat_process_gesture_info(struct cts_platform_data *pdata, struct cts_device_gesture_info *gesture_info)
{
    int i;
    u16 gest_posx[CTS_CHIP_MAX_GESTURE_TRACE_POINT];
    u16 gest_posy[CTS_CHIP_MAX_GESTURE_TRACE_POINT];

    cts_info("Process gesture, id=0x%02x", gesture_info->gesture_id);

    for (i = 0; i < gesture_info->num_points; i++) {
        gest_posx[i] = le16_to_cpu(gesture_info->points[i].x);
        gest_posy[i] = le16_to_cpu(gesture_info->points[i].y);  
    }  
    
    vts_report_coordinates_set(cts_data->vtsdev, gest_posx, gest_posy, gesture_info->num_points);

#if defined(CFG_CTS_GESTURE_REPORT_KEY)
    for (i = 0; i < CFG_CTS_NUM_GESTURE; i++) {
        if (gesture_info->gesture_id == pdata->gesture_keymap[i][0]) {
            cts_info("Report key[%u]", pdata->gesture_keymap[i][1]);
            vts_report_event_down(cts_data->vtsdev, pdata->gesture_keymap[i][1]);

            vts_report_event_up(cts_data->vtsdev, pdata->gesture_keymap[i][1]);
            return 0;
        }
    }
#endif // CFG_CTS_GESTURE_REPORT_KEY 

    cts_warn("Process unrecognized gesture id=%u",
        gesture_info->gesture_id);

    return -EINVAL;
}

int cts_plat_process_touch_msg(struct cts_platform_data *pdata, struct cts_device_touch_msg *msgs, int num, ktime_t kt)
{
    int i;
    int contact = 0;

#ifdef CONFIG_CTS_SLOTPROTOCOL
    static unsigned char finger_last[CFG_CTS_MAX_TOUCH_NUM] = {0};
    unsigned char finger_current[CFG_CTS_MAX_TOUCH_NUM] = {0};
#endif
    //*custom_data = 1;
    //cts_dbg("Process touch %d msgs", num);
    if (num ==0 || num > CFG_CTS_MAX_TOUCH_NUM) {
        return 0;
    }
    for (i = 0; i < num; i++) {
        u16 x, y;

        x = le16_to_cpu(msgs[i].x);
        y = le16_to_cpu(msgs[i].y);

#ifdef CFG_CTS_SWAP_XY
        swap(x,y);
#endif // CFG_CTS_SWAP_XY 
#ifdef CFG_CTS_WRAP_X
        x = wrap(pdata->res_x,x);
#endif // CFG_CTS_WRAP_X 
#ifdef CFG_CTS_WRAP_Y
        y = wrap(pdata->res_y,y);
#endif // CFG_CTS_WRAP_Y 
        if (msgs[i].event == CTS_DEVICE_TOUCH_EVENT_DOWN || msgs[i].event == CTS_DEVICE_TOUCH_EVENT_MOVE|| msgs[i].event == CTS_DEVICE_TOUCH_EVENT_STAY) {
            if (msgs[i].id < CFG_CTS_MAX_TOUCH_NUM) {
                finger_current[msgs[i].id] = 1;
            }
        }
        switch (msgs[i].event) {
            case CTS_DEVICE_TOUCH_EVENT_DOWN:
            case CTS_DEVICE_TOUCH_EVENT_MOVE:
            case CTS_DEVICE_TOUCH_EVENT_STAY:
                contact++;
                vts_report_point_down(cts_data->vtsdev, msgs[i].id, num, x, y, msgs[i].pressure, msgs[i].pressure, false, cts_data->custom_data, sizeof(cts_data->custom_data), kt);
                break;
            case CTS_DEVICE_TOUCH_EVENT_UP:
                vts_report_point_up(cts_data->vtsdev, msgs[i].id, num, x, y, msgs[i].pressure, msgs[i].pressure, false, kt);
                finger_last[msgs[i].id] = 0;
                break;

            default:
                cts_warn("Process touch msg with unknown event %u id %u",
                    msgs[i].event, msgs[i].id);
                break;
        }

    }

    for (i = 0; i < CFG_CTS_MAX_TOUCH_NUM; i++) {
        if (finger_last[i] != 0 && finger_current[i] == 0) {
            vts_report_point_up(cts_data->vtsdev, i, num, le16_to_cpu(msgs[i].x), le16_to_cpu(msgs[i].y), msgs[i].pressure, msgs[i].pressure, false, kt);
        }
        finger_last[i] = finger_current[i];
    }

    vts_report_point_sync(cts_data->vtsdev);

    return 0;
}

static int cts_get_touchinfo(const struct cts_device *cts_dev, struct cts_device_touch_info *touch_info)
{
    cts_dbg("Get touch info");

    if (cts_dev->rtdata.program_mode) {
        cts_warn("Get touch info in program mode");
        return -ENODEV;
    }

    if (cts_dev->rtdata.suspended) {
        cts_warn("Get touch info while is suspended");
        return -ENODEV;
    }

    return cts_fw_reg_readsb(cts_dev, CTS_DEVICE_FW_REG_TOUCH_INFO, touch_info, sizeof(*touch_info));
}


static irqreturn_t cts_work_func(int irq, void *handle,  ktime_t kt)
{
    int ret;
    u8 freq_index;
    u8 water_reject_mode;
    u8 noise_level;
    u16 fwid;
    struct cts_device_touch_info *touch_info;
    u8 palm_detec = 0;
    static u8 last_palm_detec = 0;
    if (cts_data->pdata->cts_dev->rtdata.program_mode) {
        cts_err("IRQ triggered in program mode");
    
        return -EINVAL;
    }

    cts_lock_device(cts_data->pdata->cts_dev);

    if (unlikely(cts_data->pdata->cts_dev->rtdata.suspended)) {
#ifdef CFG_CTS_GESTURE
        if (cts_data->pdata->cts_dev->rtdata.gesture_wakeup_enabled) {
            struct cts_device_gesture_info gesture_info;
            //int i;

            cts_info("Get gesture info");
            ret = cts_get_gesture_info(cts_data->pdata->cts_dev,
                    &gesture_info, CFG_CTS_GESTURE_REPORT_TRACE);
            if (ret) {
                cts_warn("Get gesture info failed %d", ret);
                //return ret;
            }
            //
            cts_dbg("Gesture id:%02x, Gesture point num:%d", gesture_info.gesture_id, gesture_info.num_points);
            if (gesture_info.num_points > CTS_CHIP_MAX_GESTURE_TRACE_POINT) {
                cts_err("Too many trace point");
                gesture_info.num_points = CTS_CHIP_MAX_GESTURE_TRACE_POINT;
            }
            /* gesture point debug
            for (i = 0; i < gesture_info.num_points; i++)
                cts_dbg("Gesture Point:[%2d, %4d, %4d, %2d]", i, le16_to_cpu(gesture_info.points[i].x),\
                   le16_to_cpu(gesture_info.points[i].y), gesture_info.points[i].event);
            */
            //panshan add gesture mode esd recover start
            fwid = 0;
            ret = cts_get_fwid(cts_data->pdata->cts_dev, &fwid);
            cts_info("firmware id = %x",fwid);
            if(fwid!=CTS_DEV_FWID_ICNL9911C || ret)
            {
                mdelay(100);
                ret = cts_get_fwid(cts_data->pdata->cts_dev, &fwid);
                if(fwid!=CTS_DEV_FWID_ICNL9911C || ret)
                {
                    cts_info(" cts_func_work lcm reset");
                    vts_dsi_panel_reset_power_ctrl(0);
                    mdelay(15);
                    vts_dsi_panel_reset_power_ctrl(1);
                }
            }
            //panshan add gesture mode esd recover end
           
            //
            /** - Issure another suspend with gesture wakeup command to device
             * after get gesture info.
             */
            cts_info("Set device enter gesture mode");
            //IC return gesture mode --panshan
            cts_send_command(cts_data->pdata->cts_dev, CTS_CMD_SUSPEND_WITH_GESTURE_REPEAT);

            ret = cts_plat_process_gesture_info(cts_data->pdata->cts_dev->pdata, &gesture_info);
            if (ret) {
                cts_err("Process gesture info failed %d", ret);
                cts_unlock_device(cts_data->pdata->cts_dev);
                return IRQ_NONE;
            }
        } else {
            cts_warn("IRQ triggered while device suspended "
                    "without gesture wakeup enable");
        }
#endif /* CFG_CTS_GESTURE */
    } else {
        touch_info = &cts_data->pdata->cts_dev->pdata->touch_info;
        
#ifdef CFG_CTS_FW_LOG_REDIRECT
        ret = cts_fw_reg_readsb(cts_data->pdata->cts_dev, CTS_DEVICE_FW_REG_TOUCH_INFO, touch_info, 1);
        if (ret) {
            cts_err("Get vkey_state failed %d", ret);
            cts_unlock_device(cts_data->pdata->cts_dev);
            return IRQ_NONE;
        }

        if (touch_info->vkey_state == CTS_FW_LOG_REDIRECT_SIGN) {
            if (cts_is_fw_log_redirect(cts_data->pdata->cts_dev)) {
                cts_show_fw_log(cts_data->pdata->cts_dev);
            }
            cts_unlock_device(cts_data->pdata->cts_dev);
            return IRQ_HANDLED;
        }
#endif /* CFG_CTS_FW_LOG_REDIRECT */
        ret = cts_get_touchinfo(cts_data->pdata->cts_dev, touch_info);

        if (ret) {
            cts_err("Get touch info failed %d", ret);
            cts_unlock_device(cts_data->pdata->cts_dev);
            return IRQ_NONE;
        }

//noise add start
        cts_data->custom_data[0]=freq_index = touch_info->add_info & 0x07;
        cts_data->custom_data[1]=water_reject_mode = (touch_info->add_info >> 3) & 0x01;
        cts_data->custom_data[2]=noise_level      = (touch_info->add_info >> 4) & 0x0f;
//noise add end
        if(touch_info->vkey_state == CTS_FW_PALM_DETECT){//palm on 
            palm_detec = 1;
            VTD("palm dtect ");
            if(0 == last_palm_detec){
                VTI("palm ON ");
                vts_report_release(cts_data->vtsdev);
                vts_report_event_down(cts_data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);     
            }
            last_palm_detec = palm_detec;
            cts_unlock_device(cts_data->pdata->cts_dev);
            return IRQ_HANDLED;
        }else {//palm OFF
            palm_detec = 0;
            if(last_palm_detec){
                VTI("palm OFF");
                vts_report_release(cts_data->vtsdev);
                vts_report_event_up(cts_data->vtsdev, VTS_EVENT_GESTURE_LARGE_AREA_PRESS);
                last_palm_detec = palm_detec;
                cts_unlock_device(cts_data->pdata->cts_dev);
                return IRQ_HANDLED;
            }
        }
        last_palm_detec = palm_detec;
        ret = cts_plat_process_touch_msg(cts_data->pdata->cts_dev->pdata, touch_info->msgs, touch_info->num_msg, kt);
        if (ret) {
            cts_err("Process touch msg failed %d", ret);
            cts_unlock_device(cts_data->pdata->cts_dev);
            return IRQ_NONE;
        }
    }
    cts_unlock_device(cts_data->pdata->cts_dev);
    return IRQ_HANDLED;
}

int cts_plat_request_irq(struct chipone_ts_data *cts_data)
{
    unsigned long irq_flags;
    int ret;

    irq_flags = IRQF_ONESHOT;
    if (cts_data->pdata->cts_dev->fwdata.int_mode) {
        irq_flags |= IRQF_TRIGGER_RISING;
    } else {
        irq_flags |= IRQF_TRIGGER_FALLING;
     }

    cts_info("Request IRQ flags: 0x%lx", irq_flags);

    ret = vts_interrupt_register(cts_data->vtsdev, cts_data->pdata->irq, cts_work_func, irq_flags, cts_data);
    if (ret != 0) {
        cts_err("Request IRQ failed %d", ret);
        return ret;
    }
    return 0;
}
/*vivo panshan add*/

static const struct vts_operations chipone_vts_ops = {
	//.init = bbk_xxx_hw_init,
	.change_mode = bbk_chipone_mode_change,
	.get_frame = chipone_get_frame,
	.get_fw_version = bbk_chipone_get_fw_version,
	.get_ic_mode = bbk_chipone_get_ic_work_mode,
	.set_charging = bbk_chipone_set_charger_bit,
	.set_rotation = chipone_setEdgeRestainSwitch,
	.set_auto_idle = cts_idle_write,
    //.update_gesture_bit = cts_set_gesture,
	.set_gesture = cts_set_gesture,
	.update_firmware = bbk_chipone_fw_update,
	.rom_read = bbk_chipone_readUdd,
	.rom_size = bbk_chipone_getUdd_size,
	//.reset = bbk_xxx_reset,
};



static int cts_driver_probe(struct spi_device *client, struct device_node *np)
{
    int ret = 0;
    struct vts_device *vtsdev = NULL;

    VTI("%s: enter cts probe", __func__);


    cts_data = (struct chipone_ts_data *)kzalloc(sizeof(*cts_data), GFP_KERNEL);
    if (cts_data == NULL) {
        cts_err("Allocate chipone_ts_data failed");
        return -ENOMEM;
    }

    cts_data->pdata = (struct cts_platform_data *)kzalloc(sizeof(struct cts_platform_data), GFP_KERNEL);
    if (cts_data->pdata == NULL) {
        cts_err("Allocate cts_platform_data failed");
        ret = -ENOMEM;
        goto err_free_cts_data;
    }


    spi_set_drvdata(client, cts_data);
    cts_data->spi_client = client;
    cts_data->device = &client->dev;

    cts_init_platform_data(cts_data->pdata, client, np);

    cts_data->cts_dev.pdata = cts_data->pdata;
    cts_data->pdata->cts_dev = &cts_data->cts_dev;

    //panshan add for platform spi delay
    //pm_runtime_forbid(client->master->dev.parent);

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_data->esd_workqueue = create_singlethread_workqueue(CFG_CTS_DEVICE_NAME "-esd_workqueue");
    if (cts_data->esd_workqueue == NULL) {
        cts_err("Create esd workqueue failed");
        ret = -ENOMEM;
        //goto err_destroy_workqueue;
    }
#endif

    ret = cts_plat_request_resource(cts_data->pdata);
    if (ret < 0) {
        cts_err("Request resource failed %d", ret);
        goto err_destroy_esd_workqueue;
    }

    ret = cts_plat_reset_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Reset device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_probe_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Probe device failed %d", ret);
        goto err_free_resource;
    }
   
    cts_init_esd_protection(cts_data);

    ret = cts_tool_init(cts_data);
    if (ret < 0) {
        cts_warn("Init tool node failed %d", ret);
    }

    ret = cts_sysfs_add_device(&client->dev);
    if (ret < 0) {
        cts_warn("Add sysfs entry for device failed %d", ret);
    }

    cts_vendor_init(cts_data);

    vtsdev = vts_device_alloc();
	if (!vtsdev) {
		ret = -ENOMEM;
		goto err_free_resource;
	}

    vtsdev->ops = &chipone_vts_ops;
    vtsdev->busType = BUS_SPI;
	cts_data->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, np);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto err_vts_device_free;
	}
	vts_set_drvdata(vtsdev, cts_data);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {		
		VTE("vts_register_driver failed");
		goto err_vts_device_free;
	}

    ret = cts_plat_request_irq(cts_data);
    if (ret < 0) {
    cts_err("Request IRQ failed %d", ret);
    goto err_register_fb;
    } 

	ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        goto err_deinit_earjack_detect;
    }

    return 0;


err_vts_device_free:
    vts_device_free(vtsdev);
err_deinit_earjack_detect:
    cts_plat_free_irq(cts_data->pdata);
err_register_fb:
    cts_sysfs_remove_device(&client->dev);
#ifdef CONFIG_CTS_LEGACY_TOOL
    cts_tool_deinit(cts_data);
#endif /* CONFIG_CTS_LEGACY_TOOL */
#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_deinit_esd_protection(cts_data);
#endif /* CONFIG_CTS_ESD_PROTECTION */
err_free_resource:
    cts_plat_free_resource(cts_data->pdata);
err_destroy_esd_workqueue:
#ifdef CONFIG_CTS_ESD_PROTECTION
    destroy_workqueue(cts_data->esd_workqueue);
#endif
    kfree(cts_data->pdata);
err_free_cts_data:
    kfree(cts_data);

    cts_err("Probe failed %d", ret);

    return ret;
}


static int cts_driver_remove(struct spi_device *client, struct device_node *np)
{
    struct chipone_ts_data *cts_data;
    int ret = 0;
    struct vts_device *vtsdev;

    cts_info("Remove");

    cts_data = (struct chipone_ts_data *)spi_get_drvdata(client);
	vtsdev = cts_data->vtsdev;

	vts_unregister_driver(vtsdev);
	vts_device_free(vtsdev);
    
    if (cts_data) {
        cts_vendor_deinit(cts_data);

        ret = cts_stop_device(&cts_data->cts_dev);
        if (ret) {
            cts_warn("Stop device failed %d", ret);
        }

        cts_plat_free_irq(cts_data->pdata);

        cts_tool_deinit(cts_data);

        cts_sysfs_remove_device(&client->dev);

        cts_deinit_esd_protection(cts_data);

        cts_plat_free_resource(cts_data->pdata);

#ifdef CONFIG_CTS_ESD_PROTECTION
        if (cts_data->esd_workqueue) {
            destroy_workqueue(cts_data->esd_workqueue);
        }
#endif
        if (cts_data->pdata) {
            kfree(cts_data->pdata);
        }
        kfree(cts_data);
    }else {
        cts_warn("Chipone i2c driver remove while NULL chipone_ts_data");
        return -EINVAL;
    }

    return ret;
}

#ifdef CONFIG_CTS_PM_LEGACY
static int cts_i2c_driver_suspend(struct device *dev, pm_message_t state)
{
    cts_info("Suspend by legacy power management");
    return cts_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_resume(struct device *dev)
{
    cts_info("Resume by legacy power management");
    return cts_resume(dev_get_drvdata(dev));
}
#endif /* CONFIG_CTS_PM_LEGACY */

#ifdef CONFIG_CTS_PM_GENERIC
static int cts_i2c_driver_pm_suspend(struct device *dev)
{
    cts_info("Suspend by bus power management");
    return cts_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_pm_resume(struct device *dev)
{
    cts_info("Resume by bus power management");
    return cts_resume(dev_get_drvdata(dev));
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops cts_i2c_driver_pm_ops = {
    .suspend = cts_i2c_driver_pm_suspend,
    .resume = cts_i2c_driver_pm_resume,
};
#endif /* CONFIG_CTS_PM_GENERIC */

#ifdef CONFIG_CTS_SYSFS
static ssize_t reset_pin_show(struct device_driver *driver, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_HAS_RESET_PIN
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(reset_pin, S_IRUGO, reset_pin_show, NULL);
#else
static DRIVER_ATTR_RO(reset_pin);
#endif

static ssize_t swap_xy_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_SWAP_XY: %c\n",
#ifdef CFG_CTS_SWAP_XY
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(swap_xy, S_IRUGO, swap_xy_show, NULL);
#else
static DRIVER_ATTR_RO(swap_xy);
#endif

static ssize_t wrap_x_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_WRAP_X: %c\n",
#ifdef CFG_CTS_WRAP_X
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(wrap_x, S_IRUGO, wrap_x_show, NULL);
#else
static DRIVER_ATTR_RO(wrap_x);
#endif

static ssize_t wrap_y_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_WRAP_Y: %c\n",
#ifdef CFG_CTS_WRAP_Y
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(wrap_y, S_IRUGO, wrap_y_show, NULL);
#else
static DRIVER_ATTR_RO(wrap_y);
#endif

static ssize_t force_update_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_FIRMWARE_FORCE_UPDATE
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(force_update, S_IRUGO, force_update_show, NULL);
#else
static DRIVER_ATTR_RO(force_update);
#endif

static ssize_t max_touch_num_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_MAX_TOUCH_NUM: %d\n",
        CFG_CTS_MAX_TOUCH_NUM);
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(max_touch_num, S_IRUGO, max_touch_num_show, NULL);
#else
static DRIVER_ATTR_RO(max_touch_num);
#endif

static ssize_t vkey_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CONFIG_CTS_VIRTUALKEY: %c\n",
#ifdef CONFIG_CTS_VIRTUALKEY
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(vkey, S_IRUGO, vkey_show, NULL);
#else
static DRIVER_ATTR_RO(vkey);
#endif

static ssize_t gesture_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_GESTURE: %c\n",
#ifdef CFG_CTS_GESTURE
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(gesture, S_IRUGO, gesture_show, NULL);
#else
static DRIVER_ATTR_RO(gesture);
#endif

static ssize_t esd_protection_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CONFIG_CTS_ESD_PROTECTION: %c\n",
#ifdef CONFIG_CTS_ESD_PROTECTION
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(esd_protection, S_IRUGO, esd_protection_show, NULL);
#else
static DRIVER_ATTR_RO(esd_protection);
#endif

static ssize_t slot_protocol_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "CONFIG_CTS_SLOTPROTOCOL: %c\n",
#ifdef CONFIG_CTS_SLOTPROTOCOL
        'Y'
#else
        'N'
#endif
        );
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(slot_protocol, S_IRUGO, slot_protocol_show, NULL);
#else
static DRIVER_ATTR_RO(slot_protocol);
#endif

static ssize_t max_xfer_size_show(struct device_driver *dev, char *buf)
{
#ifdef CONFIG_CTS_I2C_HOST
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_MAX_I2C_XFER_SIZE: %d\n",
        CFG_CTS_MAX_I2C_XFER_SIZE);
#else
    return scnprintf(buf, PAGE_SIZE, "CFG_CTS_MAX_SPI_XFER_SIZE: %d\n",
        CFG_CTS_MAX_SPI_XFER_SIZE);
#endif
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(max_xfer_size, S_IRUGO, max_xfer_size_show, NULL);
#else
static DRIVER_ATTR_RO(max_xfer_size);
#endif

static ssize_t driver_info_show(struct device_driver *dev, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "Driver version: %s\n", CFG_CTS_DRIVER_VERSION);
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static DRIVER_ATTR(driver_info, S_IRUGO, driver_info_show, NULL);
#else
static DRIVER_ATTR_RO(driver_info);
#endif

static struct attribute *cts_i2c_driver_config_attrs[] = {
    &driver_attr_reset_pin.attr,
    &driver_attr_swap_xy.attr,
    &driver_attr_wrap_x.attr,
    &driver_attr_wrap_y.attr,
    &driver_attr_force_update.attr,
    &driver_attr_max_touch_num.attr,
    &driver_attr_vkey.attr,
    &driver_attr_gesture.attr,
    &driver_attr_esd_protection.attr,
    &driver_attr_slot_protocol.attr,
    &driver_attr_max_xfer_size.attr,
    &driver_attr_driver_info.attr,
    NULL
};

static const struct attribute_group cts_i2c_driver_config_group = {
    .name = "config",
    .attrs = cts_i2c_driver_config_attrs,
};
/*
static const struct attribute_group *cts_i2c_driver_config_groups[] = {
    &cts_i2c_driver_config_group,
    NULL,
};
*/
#endif /* CONFIG_CTS_SYSFS */

void cts_shut_down(struct spi_device *spi)
{
	VTI("cts shut down !!!");

	vts_dsi_panel_reset_power_ctrl(8);

}


static struct vts_spi_driver cts_spi_driver = {
	.probe		= cts_driver_probe,
	.remove		= cts_driver_remove,
	.compatible = "chipone,cts_nl9911c_spi",
	.shutdown = cts_shut_down,
};

static const int ic_numbers[] = {VTS_IC_CHIPONE_NL9911C};

module_vts_driver(chipone_nl9911c, ic_numbers, vts_spi_drv_reigster(&cts_spi_driver), vts_spi_drv_unreigster(&cts_spi_driver));

