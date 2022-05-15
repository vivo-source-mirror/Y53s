#define LOG_TAG         "Vendor"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_firmware.h"
#include "cts_strerror.h"
#include <linux/firmware.h>

#define CFG_CTS_VIVO_TOUCH_INTF_TEST
#define CTS_CHIP_ID_LEN 14

extern int cts_suspend(struct chipone_ts_data *cts_data);
extern int cts_resume(struct chipone_ts_data *cts_data);
extern int bbk_chipone_mode_change(struct vts_device *vtsdev, int which);
extern int bbk_chipone_set_charger_bit(struct vts_device *vtsdev, int state);
extern int bbk_chipone_fw_update(struct vts_device *vtsdev, const struct firmware *firmware);
extern int bbk_chipone_get_fw_version(struct vts_device *vtsdev, u64 *fw_ver);
extern int chipone_setEdgeRestainSwitch(struct vts_device *vtsdev, int on);
extern int chipone_get_frame(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size);
extern int bbk_chipone_get_ic_work_mode(struct vts_device *vtsdev);
extern int cts_set_gesture(struct vts_device *vtsdev,  int enable);
extern int cts_idle_write(struct vts_device *vtsdev, int state);

extern int cts_idle_com_exit(struct cts_device *cts_dev);
extern int cts_idle_com_active(struct cts_device *cts_dev);

//noise add 
static int  argc;
static char *argv[5];
static char cmdline_param[128];
//noise add end

static int last_ts_state = VTS_ST_NORMAL;

//

struct cts_vendor_data {
#if defined( CONFIG_CTS_SYSFS) && defined(CFG_CTS_VIVO_TOUCH_INTF_TEST)
    bool sysfs_attr_group_created;
#endif

    struct chipone_ts_data *cts_data;
};

static struct cts_vendor_data *cts_vendor_data = NULL;




/*
gesture set
*/

struct gesture_en_map {
    u32 gesture_en_ui;
    u32 gesture_en_fw;
    char *gesture_disc;
};

static const struct gesture_en_map gesture_en_map_tab[] ={
    {VTS_GESTURE_C,         GSTR_EN_MASK_C,         "C"},
    {VTS_GESTURE_E,         GSTR_EN_MASK_E,      "E"},
    {VTS_GESTURE_F,         GSTR_EN_MASK_F,         "F"},
    {VTS_GESTURE_M,         GSTR_EN_MASK_M,         "M"},
    {VTS_GESTURE_O,         GSTR_EN_MASK_O,         "O"},
    {VTS_GESTURE_DCLICK,    GSTR_EN_MASK_D_TAP,     "DTAP"},
    {VTS_GESTURE_W,         GSTR_EN_MASK_W,         "W"},
    {VTS_GESTURE_A,         GSTR_EN_MASK_A,         "@"},
    {VTS_GESTURE_UP,        GSTR_EN_MASK_UP,        "UP"},
    {VTS_GESTURE_DOWN,      GSTR_EN_MASK_DOWN,      "DOWN"},
    {VTS_GESTURE_LR,      GSTR_EN_MASK_LEFT | GSTR_EN_MASK_RIGHT, "LEFT & RIGHT"}
   //{VTS_GESTURE_RIGHT,     GSTR_EN_MASK_RIGHT,     "RIGHT"},
};

int cts_set_gesture(struct vts_device *vtsdev,  int enable)
{
    int i;
    int ret;
    u32 fw_en = 0;
    struct cts_device *cts_dev;
    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    cts_info("cts set gesture enable map");
    cts_info("enable = %d", enable);
    for (i = 0; i < sizeof(gesture_en_map_tab) / sizeof(gesture_en_map_tab[0]); i++) {
        if (enable & gesture_en_map_tab[i].gesture_en_ui) {
            fw_en |= gesture_en_map_tab[i].gesture_en_fw;
            cts_info("Enable gesture \'%s\'", gesture_en_map_tab[i].gesture_disc);
        }    
    }
    ret = cts_fw_reg_writel_retry(cts_dev, CTS_DEVICE_FW_REG_GESTURE_EN_MAP, fw_en, 3, 0);
    cts_info("write CTS_DEVICE_FW_REG_GESTURE_EN_MAP 0x%04x", fw_en);
    if (ret) {
        cts_err("write CTS_DEVICE_FW_REG_GESTURE_EN_MAP 0x%04x failed", fw_en);
    }
    return ret;
}



/*
 * Get touch data from device.
 *
 * @param which:
 *  0 - rawdata
 *  1 - diff/delta
 *  2 - baseline
 * @param data:
 *
 * Anticlockwise 90degree，left bottom is original point.
 *
 * @return 0 if success, -EINVAL if failed.
 */
int bbk_chipone_get_rawordiff_data(int which, short *data)
{
#define DEV_TOUCH_DATA_BUFFER_SIZE(cts_dev) \
        (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

    struct cts_device *cts_dev;
    u16 *touch_data = NULL;
    int   r, c, ret;

    if (data == NULL) {
        cts_err("%s: data = NULL", __func__);
        return -EINVAL;
    }

    if (which < 0 || which > 2) {
        cts_err("%s: which = %d is invalid", __func__, which);
        return -EINVAL;
    }

    cts_info("%s: which = %d, data = %p", __func__, which, data);

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    touch_data = (u16 *)kmalloc(DEV_TOUCH_DATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (touch_data == NULL) {
        cts_err("%s: Alloc mem for touch data failed", __func__);
        return -EINVAL;
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("%s: Enable read touch data failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto free_mem;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("%s: Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto free_mem;
    }
    msleep(50);

    switch (which) {
        case 0:  ret = cts_get_rawdata(cts_dev, touch_data);  break;
        case 1:  ret = cts_get_diffdata(cts_dev, touch_data); break;
        case 2:  
            ret = cts_get_baseline(cts_dev, touch_data, CTS_WORK_MODE_NORMAL_ACTIVE,
                CTS_GET_TOUCH_DATA_FLAG_ENABLE_GET_TOUCH_DATA_BEFORE |
                CTS_GET_TOUCH_DATA_FLAG_CLEAR_DATA_READY |
                CTS_GET_TOUCH_DATA_FLAG_REMOVE_TOUCH_DATA_BORDER |
                CTS_GET_TOUCH_DATA_FLAG_FLIP_TOUCH_DATA |
                CTS_GET_TOUCH_DATA_FLAG_DISABLE_GET_TOUCH_DATA_AFTER,
                0, 0);
            break;
        default: ret = -EINVAL; goto disable_get_touch_data;
    }
    if(ret) {
        cts_err("%s: Get touch data failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto disable_get_touch_data;
    }

    /* Re-order touch data */
    cts_info("Re-order touch data");
    for (r = 0; r < cts_dev->fwdata.cols; r++) {
        for (c = 0; c < cts_dev->fwdata.rows; c++) {
            data[r * cts_dev->fwdata.rows + c] = (short)(touch_data[c * cts_dev->fwdata.cols + cts_dev->fwdata.cols - 1 - r]);
        }
    }

disable_get_touch_data: {
        r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("%s: Disable read touch data failed %d(%s)", __func__, r, cts_strerror(r));
        }
    }
free_mem:
    cts_unlock_device(cts_dev);
    if (touch_data) {
        kfree(touch_data);
    }

    return ret ? -EINVAL : 0;

#undef DEV_TOUCH_DATA_BUFFER_SIZE
}

//vivo panshan add
int chipone_get_frame(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size)
{
    int chipone_which;
    int ret;
    if (which != VTS_FRAME_MUTUAL_RAW && which != VTS_FRAME_MUTUAL_DELTA) {
		return -EINVAL;
	}
    if (which == VTS_FRAME_MUTUAL_RAW) 
        chipone_which = 0;
    else if(which == VTS_FRAME_MUTUAL_DELTA)
        chipone_which = 1;
    else if(which == VTS_FRAME_AMBIENT_BASELINE)
        chipone_which = 2;
    else
    {
        return -EINVAL;
    }
    ret = bbk_chipone_get_rawordiff_data(chipone_which, data);

    return ret;
}
 
//


/*
 * Update firmware to device.
 *
 * @param firmware: firmware.
 *
 * @return 0 if success, -1 if failed.
 */
int bbk_chipone_fw_update(struct vts_device *vtsdev, const struct firmware *firmware)
{
    struct cts_firmware fw;
    struct cts_device *cts_dev;
    int ret;
    int r;

    if (firmware == NULL) {
        cts_err("%s: firmware = NULL", __func__);
        return -1;
    }

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_info("%s: data = %p, size = %zu",
        __func__, firmware->data, firmware->size);

    memset(&fw, 0, sizeof(fw));
    fw.data = (u8 *)firmware->data;
    fw.size = firmware->size;

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("%s: Stop device failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        return -1;
    }

    cts_lock_device(cts_dev);
    ret = cts_update_firmware(cts_dev, &fw, true);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("%s: Update firmware failed %d(%s)",
            __func__, ret, cts_strerror(ret));
    }

    {
        r = cts_start_device(cts_dev);
        if (r) {
            cts_err("%s: Start device failed %d(%s)",
                __func__, r, cts_strerror(r));
        }
    }

    return ret ? -1 : 0;
}

/*
 * Enable/disable idle mode.
 *
 * @param state:
 *  0 - disable
 *  1 - enable
 *
 * @return 0 if success, -1 if failed.
 */
int chipone_idleEnableOrDisable(int state)
{
    struct cts_device *cts_dev;
    int ret;

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_info("%s: state = %d", __func__, state);

    cts_lock_device(cts_dev);
    ret = cts_send_command(cts_dev,
        state ? CTS_CMD_ENABLE_IDLE_MODE : CTS_CMD_DISABLE_IDLE_MODE);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("%s: %s idle mode failed %d(%s)", __func__,
            state ? "ENABLE" : "DISABLE", ret, cts_strerror(ret));
    }

    return ret ? -1 : 0;
}

int cts_idle_write(struct vts_device *vtsdev, int state)
{
u8 idle_state;
u32 idle_time = 0;
int ret = 0;
struct cts_device *cts_dev;

idle_state = (u8)(state?1:0);
vts_property_get(vtsdev, VTS_PROPERTY_GAME_IDLE_TIME, &idle_time);
VTI("write idle state: %d, idle time: %d ", idle_state, idle_time);

cts_dev = &cts_vendor_data->cts_data->cts_dev;
 
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);
    if(idle_state)
        ret = cts_fw_reg_writeb(cts_dev, 0x0700, 2);
    else
        {
        ret = cts_fw_reg_writeb(cts_dev, 0x0700, 1);
        ret = cts_fw_reg_writew(cts_dev, 0x0701, (u16)idle_time);
        //VTI("write idle state: %d, idle time: %d ", idle_state, idle_time);
        }
    cts_unlock_device(cts_dev);

    if (ret) 
        {
        cts_err("%s:  idle mode failed ", __func__);
        }
    else
        VTI("write fw_reg 0x0700 successful");

return 0;
}

/*
 * Read user data from flash.
 *
 * @param udd: user data buffer to store user data read from flash.
 *
 * @return 0 if success, -1 if failed.
 */
//int bbk_chipone_readUdd(unsigned char *udd)
//{
    /* 0-flash */
//}

/*
 * Write user data to flash.
 *
 * @param udd: user data will be written to flash.
 *
 * @return 0 if success, -1 if failed.
 */
//int bbk_chipone_writeUdd(unsigned char *udd)
//{
    /* 0-flash */
//}

/*
 * Change work mode.
 *
 * @param which:
 *  0 - active
 *  1 - sleep
 *  2 - gesture
 *
 * @return 0 if success, -1 if failed.
 */
int bbk_chipone_mode_change(struct vts_device *vtsdev, int which)
{
    struct cts_device *cts_dev;
   // u8  cmd;
    int ret;

    if (which < 0 || which > 2) {
        cts_err("%s: which = %d is invalid", __func__, which);
        return -1;
    }

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_info("%s: which = %d", __func__, which);

    switch (which) {
        case 0:
            ret = cts_resume(cts_vendor_data->cts_data); 
            last_ts_state = VTS_ST_NORMAL;
            break;
#ifdef CFG_CTS_GESTURE			
        case 2:
            if(last_ts_state == VTS_ST_NORMAL)
            {
                cts_enable_gesture_wakeup(cts_dev);
                ret = cts_suspend(cts_vendor_data->cts_data); 
            }
            else if(last_ts_state == VTS_ST_SLEEP)
            {
                vts_dsi_panel_reset_power_ctrl(1);
                mdelay(5);
                
                cts_lock_device(cts_dev);
                cts_idle_com_active(cts_dev);
                cts_enable_gesture_wakeup(cts_dev);
                cts_unlock_device(cts_dev);
                ret = cts_suspend(cts_vendor_data->cts_data); 
                //cts_idle_com_exit(cts_dev);
            }
            else
                return -1;
            last_ts_state = VTS_ST_GESTURE;
            break;
#endif			
        case 1:
            if(last_ts_state == VTS_ST_NORMAL)
            {
                cts_disable_gesture_wakeup(cts_dev);
                ret = cts_suspend(cts_vendor_data->cts_data);
            }
            else if(last_ts_state == VTS_ST_GESTURE)
            {
                cts_lock_device(cts_dev);
                cts_idle_com_active(cts_dev);  
                cts_disable_gesture_wakeup(cts_dev);
                cts_unlock_device(cts_dev);
                ret = cts_suspend(cts_vendor_data->cts_data);
                //cts_idle_com_exit(cts_dev);

                ret = cts_plat_disable_irq_wake(cts_vendor_data->cts_data->pdata);
                if (ret) 
                {
                    cts_warn("Disable IRQ wake failed %d", ret);
                    return ret;
                }
               
            }
            else
                return -1;
            last_ts_state = VTS_ST_SLEEP;
            
        
            vts_dsi_panel_reset_power_ctrl(0);
		    VTI("touchscreen is shutdown!!!");
            break;
        default: return -1;
    }

    if (ret) {
        cts_err("%s: Set work mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
    }

    return ret ? -1 : 0;
}

/*
 * Get firmware/config version.
 *
 * @param which:
 *  0 - firmware
 *  1 - config
 *
 * If do not has config version, return 0. Valid version != 0.
 *
 * @return firmware version if success, -1 if failed.
 */
 int bbk_chipone_get_fw_version(struct vts_device *vtsdev, u64 *version)
{
    struct cts_device *cts_dev;
    u16 fw_ver;
    int ret;
/*    int which = 0;

    if (which < 0 || which > 1) {
        cts_err("%s: which = %d is invalid", __func__, which);
        return -1;
    }

    cts_info("%s: which = %d", __func__, which);

    if (which == 1) {
        return 0;
    }
*/
    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);

    ret = cts_get_firmware_version(cts_dev, &fw_ver);
    cts_unlock_device(cts_dev);
    

    if (ret) {
        cts_err("%s: Get fw version failed %d(%s)",
            __func__, ret, cts_strerror(ret));
    }
    *version = (u64)(fw_ver & 0x00FF);
    return 0;
}

#ifdef CFG_CTS_GESTURE
int bbk_chipone_gesture_point_get(u16 *data)
{
    struct cts_device *cts_dev;
    struct cts_device_gesture_info dev_gesture_info;
    int i, ret;

    if (data == NULL) {
        cts_err("%s: data = NULL", __func__);
        return -1;
    }

    cts_info("%s", __func__);

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);
    ret = cts_get_gesture_info(cts_dev, &dev_gesture_info, true);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("%s: Get gesture info failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        return -1;
    }

    if (dev_gesture_info.num_points > CTS_CHIP_MAX_GESTURE_TRACE_POINT) {
        cts_err("%s: Get gesture %u with max point %d too large",
            __func__, dev_gesture_info.gesture_id,
            dev_gesture_info.num_points);
        return -1;
    }

    for (i = 0; i < dev_gesture_info.num_points; i++) {
        *data++ = dev_gesture_info.points[i].x;
        *data++ = dev_gesture_info.points[i].y;
    }

    return 0;

#undef MAX_POINTS
}
#endif /* CFG_CTS_GESTURE */

int bbk_chipone_set_charger_bit(struct vts_device *vtsdev, int state)
{
    struct cts_device *cts_dev;
    int ret;

    if (state < 0 || state > 1) {
        cts_err("%s: state = %d is invalid", __func__, state);
        return -1;
    }

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_info("%s: state = %d", __func__, state);

    cts_lock_device(cts_dev);
    ret = cts_send_command(cts_dev,
        state ? CTS_CMD_CHARGER_ATTACHED : CTS_CMD_CHARGER_DETACHED);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("%s: Set charger state failed %d(%s)",
            __func__, ret, cts_strerror(ret));
    }

    return ret ? -1 : 0;
}

/*
 * Get charger state from device.
 *
 * @param none
 *
 *
 * @return 0 if charger detached, 1 if charger attached, -1 if failed.
 */
int bbk_chipone_read_charger_bit(void)
{
    struct cts_device *cts_dev;
    u8  val;
    int ret;

    cts_info("%s", __func__);

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(cts_dev, 0x0007, &val);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("%s: Get charger state failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        return -1;
    }

    return val ? 1 : 0;
}

/*
 * Get firmware/config version.
 *
 * @param on:
 *  0 -
 *  1 -
 *  2 -
 *
 *
 *
 * @return 0 if success, -1 if failed.
 */
int chipone_setEdgeRestainSwitch(struct vts_device *vtsdev, int on)
{
    struct cts_device *cts_dev;
    int ret;

    if (on < 0 || on > 2) {
        cts_err("%s: on = %d is invalid", __func__, on);
        return -1;
    }

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_info("%s: on = %d", __func__, on);

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_writeb(cts_dev, 0x0801, (u8)on);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("%s: Set edge restain failed %d(%s)",
            __func__, ret, cts_strerror(ret));
    }

    return ret ? -1 : 0;
}

int bbk_chipone_get_header_file_version(int which, unsigned char *fw)
{
    if (which < 0 || which > 1) {
        cts_err("%s: which = %d is invalid", __func__, which);
        return -1;
    }

    cts_info("%s: which = %d", __func__, which);

    switch (which) {
        case 0:
            return (int)(get_unaligned_le16(fw + 0x100) & 0x00FF);
        case 1: return 0;
        default: return -1;
    }
}

int bbk_chipone_get_fw_debug_info(unsigned char *buf)
{
    struct cts_device *cts_dev;
    u8 active_mode;
    u8 charger_mode;
    u8 set_work_mode;
    u8 get_work_mode;
    u8 rawdata_index;
    u8 noise_diff[2];
    u8 scan_freq;
    u8 sw_busy;
    u8 ddi_state;
    u8 scan_err_reg[8];
    int ret;
    int count;

    if (buf == NULL) {
        cts_err("%s: buf = NULL", __func__);
        return -1;
    }

    cts_info("%s", __func__);

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(cts_dev, 0x0005, &active_mode);
    if (ret) {
        cts_err("%s: Get active mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x0007, &charger_mode);
    if (ret) {
        cts_err("%s: Get charger mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x0001, &set_work_mode);
    if (ret) {
        cts_err("%s: Get set work mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x003F, &get_work_mode);
    if (ret) {
        cts_err("%s: Get get work mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x0028, &rawdata_index);
    if (ret) {
        cts_err("%s: Get get rawdata index failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readsb(cts_dev, 0x0058, noise_diff, sizeof(noise_diff));
    if (ret) {
        cts_err("%s: Get get noise diff failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x0018, &scan_freq);
    if (ret) {
        cts_err("%s: Get get scan freq failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_hw_reg_readb(cts_dev, 0x35012, &sw_busy);
    if (ret) {
        cts_err("%s: Get get rHwSwBusy failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_hw_reg_readb(cts_dev, 0x30060, &ddi_state);
    if (ret) {
        cts_err("%s: Get get rHwDdiState failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_hw_reg_readsb(cts_dev, 0x351F0,
        scan_err_reg, sizeof(scan_err_reg));
    if (ret) {
        cts_err("%s: Get get rHwScanErroe failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto unlock_device;
    }

    count = sprintf(buf,
        "Active Mode   : 0x%02x\n"
        "Charge Mode   : 0x%02x\n"
        "WorkMode      : 0x%02x\n"
        "Curr Work Mode: 0x%02x\n"
        "Raw Index     : 0x%02x\n"
        "Noise Diff    : 0x%02x 0x%02x\n"
        "Scan Freq     : 0x%02x\n"
        "Swbusy        : 0x%02x\n"
        "Ddi Mode      : 0x%02x\n"
        "Scan Error    : 0x%02x 0x%02x 0x%02x 0x%02x"
                       " 0x%02x 0x%02x 0x%02x 0x%02x\n",
        active_mode, charger_mode, set_work_mode, get_work_mode,
        rawdata_index, noise_diff[0], noise_diff[1], scan_freq,
        sw_busy, ddi_state,
        scan_err_reg[0], scan_err_reg[1],
        scan_err_reg[2], scan_err_reg[3],
        scan_err_reg[4], scan_err_reg[5],
        scan_err_reg[6], scan_err_reg[7]);

unlock_device:
    cts_unlock_device(cts_dev);

    return ret ? -1 : count;
}

int bbk_chipone_get_ic_work_mode(struct vts_device *vtsdev)
{
    struct cts_device *cts_dev;
    u8  dev_mode;
    int ret;

    cts_info("%s", __func__);

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    if (cts_dev == NULL) {
        cts_err("%s: cts_dev = NULL", __func__);
        return -1;
    }

    cts_lock_device(cts_dev);
    ret = cts_idle_com_active(cts_dev);
        if (ret) {
            cts_err("Idle com enter error");
        }
    ret = cts_fw_reg_readb(cts_dev, 0x0005, &dev_mode);
     if (ret) {
        cts_err("%s: Get work mode failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        cts_unlock_device(cts_dev);
        return -1;
    }
    
    ret = cts_idle_com_exit(cts_dev);
        if (ret) {
            cts_err("Idle com exit error!");
        }
    cts_unlock_device(cts_dev);

   

    switch (dev_mode) {
        case 0: return 0;
        case 1: return 0;
        case 2: return 1;
        case 3: return 2;
        default: return -1;
    }
}

#if defined( CONFIG_CTS_SYSFS) && defined(CFG_CTS_VIVO_TOUCH_INTF_TEST)

#define VENDOR_VIVO_SYSFS_GROUP_NAME    "vivo"

static int dump_touch_data_to_buffer(char *buf, int buf_size,
    const void *touch_data, int row, int col, char separate)
{
    int r, c;
    int count = 0;
    short *data = (short *)touch_data;

    for (r = 0; r < row && count < buf_size; r++) {
        for (c = 0; c < col && count < buf_size; c++) {
            count += scnprintf(buf + count, buf_size - count,
                "%4d%c", data[r * col + c], separate);
        }

        if (count < buf_size) {
            buf[count++] = '\n';
        }
    }

    return count;
}

#define VENDOR_VIVO_SYSFS_GROUP_NAME     "vivo"

static ssize_t rawdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * sizeof(short))

    struct chipone_ts_data *cts_data;
    struct cts_device *cts_dev;
    short *touch_data = NULL;
    int ret = 0;
    int count = 0;

    cts_data = dev_get_drvdata(dev);
    cts_dev = &cts_data->cts_dev;
    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'", attr->attr.name);

    touch_data = (short *)kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (touch_data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for touch_data failed\n");
    }

    ret = bbk_chipone_get_rawordiff_data(0, touch_data);
    if (ret) {
        cts_err("Get rawdata failed");
        goto free_mem;
    }

    count = dump_touch_data_to_buffer(buf, PAGE_SIZE,
        touch_data, cts_dev->fwdata.cols, cts_dev->fwdata.rows, ',');

free_mem:
    kfree(touch_data);

    return ret ? -EIO : count;

#undef RAWDATA_BUFFER_SIZE
}
static DEVICE_ATTR(rawdata, S_IRUGO, rawdata_show, NULL);

static ssize_t diffdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#define DIFFDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * sizeof(short))

    struct chipone_ts_data *cts_data;
    struct cts_device *cts_dev;
    short *touch_data = NULL;
    int ret = 0;
    int count = 0;

    cts_data = dev_get_drvdata(dev);
    cts_dev = &cts_data->cts_dev;
    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'", attr->attr.name);

    touch_data = (short *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (touch_data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for touch_data failed\n");
    }

    ret = bbk_chipone_get_rawordiff_data(1, touch_data);
    if (ret) {
        cts_err("Get diffdata failed");
        goto free_mem;
    }

    count = dump_touch_data_to_buffer(buf, PAGE_SIZE,
        touch_data, cts_dev->fwdata.cols, cts_dev->fwdata.rows, ',');

free_mem:
    kfree(touch_data);

    return ret ? -EIO : count;

#undef DIFFDATA_BUFFER_SIZE
}
static DEVICE_ATTR(diffdata, S_IRUGO, diffdata_show, NULL);

static ssize_t baseline_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define BASELINE_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * sizeof(short))

    struct chipone_ts_data *cts_data;
    struct cts_device *cts_dev;
    short *touch_data = NULL;
    int ret = 0;
    int count = 0;

    cts_data = dev_get_drvdata(dev);
    cts_dev = &cts_data->cts_dev;
    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);

    touch_data = (short *)kmalloc(BASELINE_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (touch_data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for touch_data failed\n");
    }

    ret = bbk_chipone_get_rawordiff_data(2, touch_data);
    if (ret) {
        cts_err("Get baseline failed");
        goto free_mem;
    }

    count = dump_touch_data_to_buffer(buf, PAGE_SIZE,
        touch_data, cts_dev->fwdata.cols, cts_dev->fwdata.rows, ',');

free_mem:
    kfree(touch_data);

    return ret ? -EIO : count;

#undef BASELINE_BUFFER_SIZE
}
static DEVICE_ATTR(baseline, S_IRUGO, baseline_show, NULL);

static ssize_t fw_update_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data;
    const struct firmware *firmware = NULL;
    char *filename = NULL;
    int ret;

    cts_data = dev_get_drvdata(dev);
    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);

    filename = kstrndup(buf, count, GFP_KERNEL);
    if (filename == NULL) {
        cts_err("kstrdup filename failed");
        return -ENOMEM;
    }

    ret = request_firmware(&firmware, filename, cts_data->device);
    if (ret) {
        cts_err("Request firmware failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    ret = bbk_chipone_fw_update(cts_vendor_data->cts_data->vtsdev, firmware);
    if (ret) {
        cts_err("Update firmware failed %d", ret);
        goto release_firmware;
    }

release_firmware:
    release_firmware(firmware);
free_mem:
    kfree(filename);

    return ret < 0 ? ret : count;
}
static DEVICE_ATTR(fw_update, S_IWUSR, NULL, fw_update_store);

static ssize_t idle_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    bool enable;
    int ret;

    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);

    ret = kstrtobool(buf, &enable);
    if (ret) {
        cts_err("Invalid arg for enable");
        return ret;
    }
    ret = chipone_idleEnableOrDisable(enable ? 1 : 0);
    if (ret) {
        cts_err("Set idle mode failed");
        return ret;
    }

    return count;
}
static DEVICE_ATTR(idle_enable,   S_IWUSR, NULL, idle_enable_store);

static ssize_t work_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;

    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'", attr->attr.name);

    ret = bbk_chipone_get_ic_work_mode(cts_vendor_data->cts_data->vtsdev);
    switch (ret) {
        case 0:  return scnprintf(buf, PAGE_SIZE, "Active\n");
        case 1:  return scnprintf(buf, PAGE_SIZE, "Sleep\n");
        case 2:  return scnprintf(buf, PAGE_SIZE, "Gesture\n");
        default: return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }
}

static ssize_t work_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int mode;
    int ret;

    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);

    switch (buf[0]) {
        case '0': mode = 0; break;
        case '1': mode = 1; break;
        case '2': mode = 2; break;
        default:
            cts_err("Invalid arg for mode");
            return -EINVAL;
    }

    ret = bbk_chipone_mode_change(cts_vendor_data->cts_data->vtsdev,mode);
    if (ret) {
        cts_err("Change mode failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(work_mode, S_IWUSR | S_IRUGO,
        work_mode_show, work_mode_store);

static ssize_t fw_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    u64 fw_ver = 0;
    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);

    ret = bbk_chipone_get_fw_version(cts_vendor_data->cts_data->vtsdev, &fw_ver);
    if (ret < 0) {
        return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }
    ret = fw_ver;
    return scnprintf(buf, PAGE_SIZE, "Fw version: 0x%02x\n", ret);
}
static DEVICE_ATTR(fw_version, S_IRUGO, fw_version_show, NULL);

static ssize_t charger_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;

    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);

    ret = bbk_chipone_read_charger_bit();
    switch (ret) {
        case 0:  return scnprintf(buf, PAGE_SIZE, "Detached\n");
        case 1:  return scnprintf(buf, PAGE_SIZE, "Attached\n");
        default: return scnprintf(buf, PAGE_SIZE, "Read error\n");
    }
}

static ssize_t charger_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int state;
    int ret;

    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);

    switch (buf[0]) {
        case '0': state = 0; break;
        case '1': state = 1; break;
        default:
            cts_err("Invalid arg for state");
            return -EINVAL;
    }

    ret = bbk_chipone_set_charger_bit(cts_vendor_data->cts_data->vtsdev, state);
    if (ret) {
        cts_err("Set charger bit failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(charger_state, S_IRUGO | S_IWUSR,
        charger_state_show, charger_state_store);

static ssize_t edge_restain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int mode;
    int ret;

    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);

    switch (buf[0]) {
        case '0': mode = 0; break;
        case '1': mode = 1; break;
        case '2': mode = 2; break;
        default:
            cts_err("Invalid arg for mode");
            return -EINVAL;
    }

    ret = chipone_setEdgeRestainSwitch(cts_vendor_data->cts_data->vtsdev, mode);
    if (ret) {
        cts_err("Set edge restain failed");
        return -EIO;
    }

    return count;
}
static DEVICE_ATTR(edge_restain, S_IWUSR, NULL, edge_restain_store);

static ssize_t fw_debug_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);

    return bbk_chipone_get_fw_debug_info(buf);
}
static DEVICE_ATTR(fw_debug_info, S_IRUGO, fw_debug_info_show, NULL);
//noise add start
int bbk_chipone_get_jitter_data(u16 count, int mode ,void *touch_data)
{
    int ret;
    struct cts_device *cts_dev;
    int data_len;

    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    data_len = cts_dev->fwdata.jitter_data_len;
 
    cts_lock_device(cts_dev);    
    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("%s: Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            __func__, ret, cts_strerror(ret));
        goto bbk_get_jitter_data_unlock;
    }
    msleep(50);    
    cts_set_jitter_frame(cts_dev, count);
    cts_set_jitter_mode(cts_dev, mode);
    ret = cts_start_jitter(cts_dev);
    if (ret) {
        cts_err("start jitter");  
        goto bbk_get_jitter_data_unlock;      
    }
    /*
    ret = cts_stop_jitter(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE, "stop jitter err");
        goto exit_jitter_show;
    }
    */
    ret = cts_get_jitter_data(cts_dev, touch_data);
    if (ret) {
        cts_err("get jitter err");
        //goto bbk_get_jitter_data_exit;
    }
bbk_get_jitter_data_unlock:    
    cts_unlock_device(cts_dev);  
    return ret;  
}

static ssize_t jitter_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    struct cts_device *cts_dev;
    void *touch_data;
    int count = 0;
    int i, j;
    int data_len;
    u8 *u8Ptr;
    u16 *u16Ptr;
    u16 frame;
    u8 mode;

    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);
    if (argc != 2) {
        return scnprintf(buf, PAGE_SIZE, "Invalid num args\n"
                   "USAGE:\n"
                   "  1. echo frame_num mode > jitter\n"
                   "  2. cat jitter\n");
    }

    ret = kstrtou16(argv[0], 0, &frame);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid frame num: %s\n", argv[0]);
    }   

    ret = kstrtou8(argv[1], 0, &mode);
    if (ret || (mode != 0 && mode != 1)) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid jitter mode: %s\n", argv[1]);
    }   
    cts_info("frame %d, mode %d", frame, mode);
    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    data_len = cts_dev->fwdata.jitter_data_len;
    touch_data = (void *)kmalloc(cts_dev->fwdata.rows *\
        cts_dev->fwdata.cols * data_len, GFP_KERNEL);
    if (touch_data == NULL) {
        return scnprintf(buf, PAGE_SIZE, "malloc buf err");
    }    
    ret = bbk_chipone_get_jitter_data(frame, mode, touch_data);
    if (ret) {
        scnprintf(buf, PAGE_SIZE, "Get jitter data error");
        goto exit_jitter_show;
    }

    u8Ptr = (u8 *)touch_data;
    u16Ptr = (u16 *)touch_data;

    for (i = 0; i < cts_dev->fwdata.cols; i++) {
        for (j = 0; j < cts_dev->fwdata. rows; j++) {
            if (data_len == 1) {
                count += scnprintf(buf + count, PAGE_SIZE, "%04d,", \
                    u8Ptr[i * cts_dev->fwdata.rows + j]);
            }
            else {
                count += scnprintf(buf + count, PAGE_SIZE, "%04d,", \
                    u16Ptr[i * cts_dev->fwdata.rows + j]);
            }
        }
        count += scnprintf(buf + count, PAGE_SIZE, "\n");
    }

exit_jitter_show:
    kfree(touch_data);
   // cts_unlock_device(cts_dev);
    return count;
}


static int parse_arg(const char *buf, size_t count)
{
    char *p;

    memcpy(cmdline_param, buf, min((size_t)128, count));
    cmdline_param[count] = '\0';

    argc = 0;
    p = strim(cmdline_param);
    if (p == NULL || p[0] == '\0') {
        return 0;
    }

    while (p && p[0] != '\0' && argc < 5) {
        argv[argc++] = strsep(&p, " ,");
    }

    return argc;
}


static ssize_t jitter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //int mode;
    //int ret;

    cts_info("Write sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s' size %zu",
        attr->attr.name, count);
    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(jitter, S_IWUSR | S_IRUGO, jitter_show, jitter_store);
//noise add end

static ssize_t idle_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret;
    struct cts_device *cts_dev;
    u8 mode;
    u16 idle_time;
    int count = 0;


    cts_info("Read sysfs '"VENDOR_VIVO_SYSFS_GROUP_NAME"/%s'",
        attr->attr.name);
    cts_dev = &cts_vendor_data->cts_data->cts_dev;
    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb_retry(cts_dev, 0x0700, &mode, 3, 1);
    if (ret) {
        cts_err("Readb from 0x0700 error");
    }
    ret = cts_fw_reg_readw_retry(cts_dev, 0x0701, &idle_time, 3, 1);
    if (ret) {
        cts_err("Readw from 0x0701 error");
    }
    cts_info("Read idle ctrl, reg[0700]=%02x, reg[0701]=%04x", mode, idle_time);
    count = scnprintf(buf, PAGE_SIZE, "%s mode, idle time：%x\n", (mode==1?"game":"normal"), idle_time);
    cts_unlock_device(cts_dev);
    return count;
}

static int cts_get_chipId(struct cts_device *cts_dev, unsigned char *id, int len)
{
    int i, j;
    int ret;
    u32 crc;
    u16 crc_rd;
    unsigned char print_buf[100];
    unsigned char id_buf[CTS_CHIP_ID_LEN + 2] = {0};
    int count = 0;

    for (i = 0; i < 5; i++) {
        ret = cts_fw_reg_readsb_delay_idle(cts_dev, CTS_DEVICE_FW_REG_CHIP_ID, 
            id_buf, CTS_CHIP_ID_LEN + 2, 30);
        if (ret) {
            cts_err("Read CTS_DEVICE_FW_REG_CHIP_ID error!");
            continue;              
        }
        count = scnprintf(print_buf, PAGE_SIZE, "Read ID REG:");
        for (j = 0; j < CTS_CHIP_ID_LEN + 2; j++)
            scnprintf(print_buf + strlen(print_buf), PAGE_SIZE, "%02X ", id_buf[j]);
        cts_info("%s", print_buf);

        crc = cts_crc32(id_buf, CTS_CHIP_ID_LEN);
        crc_rd = (((u16)id_buf[CTS_CHIP_ID_LEN+1]) << 8) | id_buf[CTS_CHIP_ID_LEN];
        cts_info("Read CRC:%04X, Calc CRC:%08X", crc_rd, crc);
        if (((crc & 0xffff) == crc_rd) && (crc_rd != 0))
            break;
    }
    if (i >= 5) {
        cts_err("Get chip id error!");
        return -1;
    }
    memset(id, 0, len);
    if (len > CTS_CHIP_ID_LEN) 
        len = CTS_CHIP_ID_LEN;
    memcpy(id, id_buf, len);

    return 0;
}

ssize_t bbk_chipone_readUdd(struct vts_device *vtsdev, u8 *buf, size_t nbytes)
{
	int ret = -1;
	struct cts_device *cts_dev = &cts_vendor_data->cts_data->cts_dev;
	if (nbytes > 0) 
		ret = cts_get_chipId(cts_dev, buf, nbytes);
	if (ret) 
		cts_err("bbk_chipone_readUdd error!");
	
	return nbytes;
}

int bbk_chipone_getUdd_size(struct vts_device *vtsdev, u32 *size)
{
	*size = 15;
	return 0;
}

static DEVICE_ATTR(idle_mode, S_IRUGO, idle_mode_show, NULL);

static struct attribute *vendor_vivo_attrs[] = {
    &dev_attr_rawdata.attr,
    &dev_attr_diffdata.attr,
    &dev_attr_baseline.attr,
    &dev_attr_fw_update.attr,
    &dev_attr_idle_enable.attr,
    &dev_attr_work_mode.attr,
    &dev_attr_fw_version.attr,
    &dev_attr_charger_state.attr,
    &dev_attr_edge_restain.attr,
    &dev_attr_fw_debug_info.attr,
    &dev_attr_jitter.attr,
    &dev_attr_idle_mode.attr,
    NULL
};

static const struct attribute_group vendor_vivo_attr_group = {
    .name  = VENDOR_VIVO_SYSFS_GROUP_NAME,
    .attrs = vendor_vivo_attrs,
};
#endif

int cts_vendor_init(struct chipone_ts_data *cts_data)
{
    struct cts_vendor_data *vdata;
    int ret;

    if (cts_data == NULL) {
        cts_err("Init with cts_data = NULL");
        return -EFAULT;
    }

    vdata = kzalloc(sizeof(*vdata), GFP_KERNEL);
    if (vdata == NULL) {
        cts_err("Alloc vendor data failed");
        return -ENOMEM;
    }

    cts_info("Init for vivo");

#if defined( CONFIG_CTS_SYSFS) && defined(CFG_CTS_VIVO_TOUCH_INTF_TEST)
    cts_info("Create sysfs attr group '%s'", VENDOR_VIVO_SYSFS_GROUP_NAME);
    ret = sysfs_create_group(&cts_data->device->kobj,
        &vendor_vivo_attr_group);
    if (ret) {
        cts_warn("Create sysfs attr group failed %d", ret);
    } else {
        vdata->sysfs_attr_group_created = true;
    }
#endif

    cts_data->vendor_data = vdata;
    vdata->cts_data = cts_data;
    cts_vendor_data = vdata;

    return 0;
}

int cts_vendor_deinit(struct chipone_ts_data *cts_data)
{
    struct cts_vendor_data *vdata;

    if (cts_data == NULL) {
        cts_err("Deinit with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_warn("Deinit with vendor_data = NULL");
        return 0;
    }

    cts_info("Deinit for vivo");

#if defined( CONFIG_CTS_SYSFS) && defined(CFG_CTS_VIVO_TOUCH_INTF_TEST)
    if (vdata->sysfs_attr_group_created) {
        cts_info("Remove sysfs attr group '%s'",
            VENDOR_VIVO_SYSFS_GROUP_NAME);
        sysfs_remove_group(&cts_data->device->kobj,
            &vendor_vivo_attr_group);
        vdata->sysfs_attr_group_created = false;
    }
#endif

    kfree(vdata);
    cts_data->vendor_data = NULL;

    return 0;
}

