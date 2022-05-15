#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/list.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include "vts_core.h"
#include <linux/sensors.h>

#define VTS_PROXIMITY_NAME	"algo-prox"
#define VTS_PROXIMITY_TRIGGER_TIME 5
#define VTS_PROXIMITY_TRIGGER_MS 500

static int vts_prox_enable_set(struct sensors_classdev *scdev,
	unsigned int enable)
{
	struct vts_virtual_proxminity *virtual_prox = container_of(scdev, struct vts_virtual_proxminity, cdev);
	struct vts_device *vtsdev = container_of(virtual_prox, struct vts_device, virtual_prox);
	int ret;

	if (scdev->enabled == enable) {
		vts_dev_info(vtsdev, "same state, doesn't set again!, enable = %d", enable);
		return 0;
	}

	ret = vts_state_set(vtsdev, VTS_STA_VIRTUAL_PROX_STATE, enable);
	if (ret) {
		vts_dev_err(vtsdev, "set virtual proxminity state failed!");
		return ret;
	}

	vts_virtual_prox_enable_collect(TOUCH_VCODE_PROX_EVENT, enable);

	if (!enable)
		cancel_delayed_work(&virtual_prox->dwork);

	vts_dev_info(vtsdev, "virtual prixminity set %d", enable);
	scdev->enabled = enable;
	return 0;
}

static int vts_prox_poll_delay_set(struct sensors_classdev *scdev,
	unsigned int delay_msec)
{
	return 0;
}

static struct sensors_classdev vts_proxminity_sensor = {
	.name = VTS_PROXIMITY_NAME,
	.vendor = "vivo",
	.version = 1,
	.handle = SENSOR_TYPE_SMARTPROX_TP_HANDLE,
	.type = SENSOR_TYPE_SMARTPROX_TP,
	.max_range = "78.4",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 0,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = vts_prox_enable_set,
	.sensors_poll_delay = vts_prox_poll_delay_set,
	.flags = 2,
};

static int vts_prox_open(struct input_dev *input)
{
	return 0;
}

static void vts_prox_close(struct input_dev *dev)
{

}

static void vts_proxminity_set_value(struct vts_virtual_proxminity *virtual_prox, int x, int y, int z, int nr_report_times)
{
	spin_lock(&virtual_prox->lock);
	if(0 == x)//for factory AT test, can not change 
	virtual_prox->cdev.delay_msec = x; 
	virtual_prox->values[0] = x;
	virtual_prox->values[1] = y;
	virtual_prox->values[2] = z;
	virtual_prox->nr_remain_report_times = nr_report_times;
	spin_unlock(&virtual_prox->lock);
}

static void __vts_proxminity_report(struct vts_virtual_proxminity *virtual_prox)
{
	struct vts_device *vtsdev = container_of(virtual_prox, struct vts_device, virtual_prox);
	spin_lock(&virtual_prox->lock);
	input_report_rel(virtual_prox->idev, REL_Z, (virtual_prox->values[0] + 1));
	vts_dev_dbg(vtsdev, "virtual prox values:%d ", virtual_prox->values[0]);
	input_sync(virtual_prox->idev);
	if (--virtual_prox->nr_remain_report_times > 0)
		schedule_delayed_work(&virtual_prox->dwork, msecs_to_jiffies(VTS_PROXIMITY_TRIGGER_MS));
	spin_unlock(&virtual_prox->lock);
}

static void vts_prox_delay_work(struct work_struct *work)
{
	__vts_proxminity_report(container_of(work, struct vts_virtual_proxminity, dwork.work));
}

static int vts_prox_input_init(struct vts_device *vtsdev, struct vts_virtual_proxminity *virtual_prox)
{
	struct input_dev *idev;
	int ret;

	idev = input_allocate_device();
	if (!idev) {
		vts_dev_err(vtsdev, "no memory to alloc virtual proxminity input device");
		return -ENOMEM;
	}

	idev->open = vts_prox_open;
	idev->close = vts_prox_close;
	idev->name = VTS_PROXIMITY_NAME;
	set_bit(EV_REL, idev->evbit);
	set_bit(EV_SYN, idev->evbit);
	input_set_capability(idev, EV_REL, REL_Z);
	input_set_abs_params(idev, EV_REL, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(idev, virtual_prox);

	ret = input_register_device(idev);
	if (ret) {
		vts_dev_err(vtsdev, "register input device for virtual proxminity error!");
		input_free_device(idev);
		return ret;
	}

	virtual_prox->idev = idev;
	vts_proxminity_set_value(virtual_prox, 1, 0, 0, 1);
	__vts_proxminity_report(virtual_prox);
	return 0;
}

static void vts_prox_input_deinit(struct vts_virtual_proxminity *virtual_prox)
{
	input_unregister_device(virtual_prox->idev);
}

int vts_proxminity_init(struct vts_device *vtsdev)
{
	struct vts_virtual_proxminity *virtual_prox = &vtsdev->virtual_prox;
	int ret;
	u32 virtual_proximinity;

	vts_property_get(vtsdev, VTS_PROPERTY_VIRTUAL_PROXIMINITY, &virtual_proximinity);
	if (!virtual_proximinity) {
		vts_dev_info(vtsdev, "don't support virtual proxminity feature, don't need to init");
		return 0;
	}

	spin_lock_init(&virtual_prox->lock);
	virtual_prox->cdev = vts_proxminity_sensor;
	virtual_prox->cdev.delay_msec = 0;
	ret = sensors_classdev_register(NULL, &virtual_prox->cdev);
	if (ret) {
		vts_dev_err(vtsdev, "register sensor device failed!\n");
		return ret;
	}

	ret = vts_prox_input_init(vtsdev, virtual_prox);
	if (ret) {
		vts_dev_err(vtsdev, "proxminity inupt init failed!");
		sensors_classdev_unregister(&virtual_prox->cdev);
		return ret;
	}

	INIT_DELAYED_WORK(&virtual_prox->dwork, vts_prox_delay_work);
	virtual_prox->inited = true;
	return 0;
}

void vts_proxminity_deinit(struct vts_device *vtsdev)
{
	struct vts_virtual_proxminity *virtual_prox = &vtsdev->virtual_prox;

	if (!virtual_prox->inited)
		return ;

	virtual_prox->inited = false;
	vts_prox_input_deinit(virtual_prox);
	sensors_classdev_unregister(&virtual_prox->cdev);
}

void vts_proxminity_report(struct vts_device *vtsdev, int x, int y, int z)
{
	u32 virtual_proximinity;
	vts_property_get(vtsdev, VTS_PROPERTY_VIRTUAL_PROXIMINITY, &virtual_proximinity);
	if (!virtual_proximinity) {
		vts_dev_info(vtsdev, "don't support virtual proxminity feature, don't need to report");
		return;
	}
	vts_dev_info(vtsdev, "%d %d %d", x, y, z);
	vts_proxminity_set_value(&vtsdev->virtual_prox, x, y, z, VTS_PROXIMITY_TRIGGER_TIME);
	__vts_proxminity_report(&vtsdev->virtual_prox);
}