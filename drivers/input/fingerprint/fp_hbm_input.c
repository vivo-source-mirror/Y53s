/***************************************************************************************
  gongyulong created @ 2019/12/26
***************************************************************************************/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/input.h>

#define VTS_FP_NAME "vivo_ts_fp"
struct kobject udfp_kobj;
static bool hbm_notify_enable;
static bool hbm_notify_debug;
static int hbm_work_queue_count;
static int hbm_work_run_count;

static struct workqueue_struct *udfp_hbm_wq;
static struct work_struct udfp_hbm_work;

static ssize_t hbm_notify_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", hbm_notify_enable);
}

static ssize_t hbm_notify_enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int temp, ret;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		pr_err("Invalid input");
		return count;
	}

	hbm_notify_enable = temp;
	pr_info("fp_hbm_notify: udfp cmd send oled hbm state = %d\n", hbm_notify_enable);
	return count;
}

static ssize_t hbm_notify_debug_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", hbm_notify_debug);
}

static ssize_t hbm_notify_debug_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int temp, ret;
	ret = kstrtoint(buf, 10, &temp);
	if (ret) {
		pr_err("Invalid input");
		return count;
	}

	hbm_notify_debug = temp;
	pr_info("fp_hbm_notify: hbm_notify_debug = %d\n", hbm_notify_debug);
	return count;
}

static struct kobj_attribute udfp_hbm_notify_enable_attribute =
	__ATTR(hbm_notify_enable, 0644, hbm_notify_enable_show, hbm_notify_enable_store);

static struct kobj_attribute udfp_hbm_notify_debug_attribute =
	__ATTR(hbm_notify_debug, 0644, hbm_notify_debug_show, hbm_notify_debug_store);

static struct attribute *udfp_oled_hbm_own_sys_attrs[] = {
	&udfp_hbm_notify_enable_attribute.attr,
	&udfp_hbm_notify_debug_attribute.attr,
	NULL,
};

static ssize_t udfp_oled_hbm_object_show(struct kobject *k, struct attribute *attr, char *buf)
{

	struct kobj_attribute *kobj_attr;
	int ret = -EIO;
	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;

}

static ssize_t udfp_oled_hbm_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;

}

static void udfp_oled_hbm_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops udfp_oled_hbm_bject_sysfs_ops = {
	.show = udfp_oled_hbm_object_show,
	.store = udfp_oled_hbm_object_store,
};
static struct kobj_type udfp_oled_hbm_object_type = {
	.sysfs_ops	= &udfp_oled_hbm_bject_sysfs_ops,
	.release	= udfp_oled_hbm_object_release,
	.default_attrs = udfp_oled_hbm_own_sys_attrs,
};

extern int dsi_panel_update_hbm_aod_ud_fingerprint_for_tp(void) __attribute__((weak));

static void udfp_input_work(struct work_struct *work)
{
	pr_info("fp_hbm_notify: %s: %d: hbm_work_run_count = %d, send hbm cmd to lcm", __func__, __LINE__,  ++hbm_work_run_count);
	if (dsi_panel_update_hbm_aod_ud_fingerprint_for_tp != NULL) {
		dsi_panel_update_hbm_aod_ud_fingerprint_for_tp();
		pr_info("fp_hbm_notify: %s call dsi_panel_update_hbm_aod_ud_fingerprint_for_tp succeed", __func__);
	}
	pr_info("fp_hbm_notify: %s exit", __func__); 
}

static void fp_input_filter_process(unsigned int type, unsigned int code, int value)
{
	bool queue_work_result;

	if (true == hbm_notify_debug) {
		pr_info("fp_hbm_notify: %s: %d: type=%x, code=%x, value=%x", __func__, __LINE__, type, code, value);
	}

	if (type == EV_KEY && code == KEY_FINGERPRINT_WAKE && value == 1 && true == hbm_notify_enable) {
		queue_work_result = queue_work(udfp_hbm_wq, &udfp_hbm_work);
		if (true == queue_work_result) {
			hbm_work_queue_count++;
			if (true == hbm_notify_debug) {
				pr_info("fp_hbm_notify: %s: %d: queue hbm_notify_work success, hbm_work_queue_count = %d", __func__, __LINE__, hbm_work_queue_count);
			}
		} else {
			pr_err("fp_hbm_notify: %s: %d: queue hbm_notify_work failed", __func__, __LINE__);
		}
	}
}

static bool fp_input_filter(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	fp_input_filter_process(type, code, value);
	return 0;
}

static int fp_input_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret = -1;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	if (!strncmp(dev->name, VTS_FP_NAME, strlen(VTS_FP_NAME))) {
		handle->dev = dev;
		handle->handler = handler;
		handle->name = "fp_input_handle";

		ret = input_register_handle(handle);
		if (ret) {
			pr_err("fingerprint input handle register error");
			goto err2;
		}

		ret = input_open_device(handle);
		if (ret) {
			pr_err("input device open error");
			goto err1;
		}
		pr_info("fp_hbm_notify: fp_input_connect success");
	} else {
		goto err2;
	}

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return ret;
}

static void fp_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id fp_input_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler fp_input_handler = {
	.filter		= fp_input_filter,
	.connect	= fp_input_connect,
	.disconnect	= fp_input_disconnect,
	.name		= "fp_input",
	.id_table	= fp_input_ids,
};


static int __init fp_hbm_input_init(void)
{
	int status;

	status = input_register_handler(&fp_input_handler);
	if (status) {
		pr_err("fingerprint input register handler error");
	}

	status = kobject_init_and_add(&udfp_kobj, &udfp_oled_hbm_object_type, NULL, "fingerprint");
	if (status) {
		pr_err("%s: Create sys/fingerprint error!\n", __func__);
		return -EFAULT;
	}

	udfp_hbm_wq = alloc_workqueue("udfp_hbm_wq", WQ_HIGHPRI|WQ_UNBOUND, 0);
	if (!udfp_hbm_wq) {
		pr_err("%s: alloc_workqueue error!\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&udfp_hbm_work, udfp_input_work);

	pr_info("fp_hbm_notify: udfp_hbm_input_init success");

	return 0;
}

static void __exit fp_hbm_input_exit(void)
{
	/*nothing to do */
}

arch_initcall(fp_hbm_input_init);
module_exit(fp_hbm_input_exit);