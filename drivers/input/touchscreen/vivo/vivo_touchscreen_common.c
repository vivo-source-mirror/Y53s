#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/slab.h>
//#include <linux/sched.h>

#include <linux/sched/rt.h>
#include <linux/sched/prio.h>

#include <uapi/linux/sched/types.h>
#include <linux/vivo_touchscreen_common.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>

static vivo_touchscreen_common_data *vivotouchscreen_common_data;


void vivo_get_file_path_str(char **pp_file, char *file_path)
{
#if 0
	char *p = NULL;
	char *p1 = NULL;

	p = strrchr(file_path, '/');
	if (p != NULL) {
		strlcpy(pp_file, p + 1, 255);
		p1 = strrchr(pp_file, '.');
		if (p1 != NULL) {
			*p1 = 0;
		}
	}
#else
	char *p = NULL;
	p = strrchr(file_path, '/');
	if (p != NULL)
		*pp_file = p+1;
	else
		*pp_file = file_path;

#endif
}

int usb_charger_flag;
int vivo_get_before_ts_load_usb_charger_flag(void)
{
	return usb_charger_flag;
}
void vivo_clear_before_ts_load_usb_charger_flag(void)
{
	usb_charger_flag = 0;
}
void charger_connect_judge(char on_or_off)
{
#if 1
	if (vivotouchscreen_common_data != NULL && vivotouchscreen_common_data->charge_connect_judge != NULL) {
		vivotouchscreen_common_data->charge_connect_judge(on_or_off);
	} else {
		if (on_or_off) {
			usb_charger_flag = 1;
			printk(KERN_ERR"%s is NULL, usb_charger_flag = %d\n", __func__, usb_charger_flag);
		} else {
			usb_charger_flag = 0;
		printk(KERN_ERR"%s is NULL, usb_charger_flag = %d\n", __func__, usb_charger_flag);
		}
	}
#endif
}


int get_ts_log_switch(void)
{
#if 1
	if (vivotouchscreen_common_data != NULL && vivotouchscreen_common_data->get_ts_log_switch != NULL) {
		return vivotouchscreen_common_data->get_ts_log_switch();
	} else {
		/*printk(KERN_ERR"%s is NULL", __func__);*/
	/*  dump_stack();*/
	}
#endif

	return 0;
}

EXPORT_SYMBOL(charger_connect_judge);
EXPORT_SYMBOL(get_ts_log_switch);

static char factory_mode[8] = "0";
int bbk_factory_mode(void)
{
	if (factory_mode[0] == '1')
		return 1;
	else if (factory_mode[0] == '0')
		return 0;
		
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_factory_mode);

static __init int get_factory_mode_value(char *str)
{
	strlcpy(factory_mode, str, sizeof(factory_mode));
	pr_err("factory_mode is %s\n", factory_mode);
	return 0;
}
early_param("factory_mode", get_factory_mode_value);

int register_touchscreen_common_interface(vivo_touchscreen_common_data *common_data)
{
	int retval = 0;
	if (vivotouchscreen_common_data == NULL)
		vivotouchscreen_common_data = common_data;
	else {
		printk(KERN_ERR"%s:Can not register %s, it has been registered by %s!!!\n", __func__, common_data->driver_name, vivotouchscreen_common_data->driver_name);
		retval = -1;
	}

	return retval;
}

void unregister_touchscreen_common_interface(vivo_touchscreen_common_data *common_data)
{
	if (vivotouchscreen_common_data == common_data) {
		vivotouchscreen_common_data  = NULL;
	}
}

#define	FUNC_NAME_LEN	60
struct vivo_touchscreen_func {
	struct list_head	func_lists;
	void (*init_func)(void);
	char func_name[FUNC_NAME_LEN];
};

static struct touchscreen_boot_ctl {
	struct mutex boot_mutex;
	struct list_head boot_lists;
	wait_queue_head_t wait;
} touchscreen_boot_ctl;

static struct task_struct *vivo_touchscreen_module_init_kthread;

struct mutex NotifyTsThreadMutex;
static int vivo_ts_module_init_kthread(void *ignored)
{
	struct vivo_touchscreen_func *boot_func = NULL;
	while (1) {
		wait_event_interruptible(
			touchscreen_boot_ctl.wait,
			(!list_empty(&touchscreen_boot_ctl.boot_lists)
			 || kthread_should_stop()));

		mutex_lock(&touchscreen_boot_ctl.boot_mutex);
		while (!list_empty(&touchscreen_boot_ctl.boot_lists)) {
			boot_func = list_first_entry(&touchscreen_boot_ctl.boot_lists, struct vivo_touchscreen_func, func_lists);
			list_del(&boot_func->func_lists);
			mutex_unlock(&touchscreen_boot_ctl.boot_mutex);
			if ((NULL !=  boot_func) && (NULL != boot_func->init_func)) {
				printk("VIVO_TS^%s: %s start.\n", __func__, boot_func->func_name);
				boot_func->init_func();
			} else {
				printk("VIVO_TS^%s:boot func is NULL or init_func is NULL.\n", __func__);
			}
			if (NULL != boot_func) {
				kfree(boot_func);
				boot_func = NULL;
			}
			mutex_lock(&touchscreen_boot_ctl.boot_mutex);
		}
		mutex_unlock(&touchscreen_boot_ctl.boot_mutex);
	}
	return 0;
}

int vivo_touchscreen_new_module_init(void (* module_init_func)(void), const char *func_name)
{
	struct vivo_touchscreen_func *boot_func = NULL;
	if (NULL == module_init_func) {
		printk("VIVO_TS^%s:Invalid init func.\n", __func__);
		return -EINVAL;
	}
	boot_func = (struct vivo_touchscreen_func *)kzalloc(sizeof(struct vivo_touchscreen_func), GFP_KERNEL);
	if (!boot_func) {
		printk("VIVO_TS^%s:Failed to alloc init func.\n", __func__);
		return -ENOMEM;
	}
	boot_func->init_func = module_init_func;
	strlcpy(boot_func->func_name, func_name, FUNC_NAME_LEN);
	mutex_lock(&touchscreen_boot_ctl.boot_mutex);
	list_add_tail(&boot_func->func_lists, &touchscreen_boot_ctl.boot_lists);
	mutex_unlock(&touchscreen_boot_ctl.boot_mutex);
	wake_up(&touchscreen_boot_ctl.wait);
	return 0;
}


struct touchscreen_req {
	int req_id;
	int req_state;
	struct completion done;
	struct list_head req_list;
};


#define TOUCHSCREEN_KTHREAD_ZOMBIE 0x00000001



static struct touchscreen_kthread_ctl {
	u32 flags;
	struct mutex mux;
	RESP_FUNC resp_func[TOUCHSCREEN_REQ_ID_MAX];
	struct list_head req_lists;
	wait_queue_head_t wait;
	void *data;
	struct wakeup_source wake_lock;
} touchscreen_kthread_ctl;

static struct task_struct *touchscreen_kthread;

static int touchscreen_thread(void *ignored)
{
	static const struct sched_param param = {
		.sched_priority = DEFAULT_PRIO, 		/*170531 change MAX_USER_RT_PRIO/2 to DEFAULT_PRIO, solve finger wakeup slow question*/
	};

		sched_setscheduler(current, SCHED_FIFO, &param);

	/*set_freezable();*/

	while (1)  {
		struct touchscreen_req  *req;
		wait_event_interruptible(
			touchscreen_kthread_ctl.wait,
			(!list_empty(&touchscreen_kthread_ctl.req_lists)
			 || kthread_should_stop()));
		mutex_lock(&touchscreen_kthread_ctl.mux);
		if (touchscreen_kthread_ctl.flags & TOUCHSCREEN_KTHREAD_ZOMBIE) {
			mutex_unlock(&touchscreen_kthread_ctl.mux);
			goto out;
		}

		while (!list_empty(&touchscreen_kthread_ctl.req_lists)) {
			req = list_first_entry(&touchscreen_kthread_ctl.req_lists, struct touchscreen_req, req_list);
			list_del(&req->req_list);
			mutex_unlock(&touchscreen_kthread_ctl.mux);

					if (req->req_id >= TOUCHSCREEN_REQ_ID_RESUME_SUSPEND &&
						req->req_id < TOUCHSCREEN_REQ_ID_MAX) {
				if (touchscreen_kthread_ctl.resp_func[req->req_id] != NULL) {
					mutex_lock(&NotifyTsThreadMutex);
					printk("VIVO_TS_INFO:NotifyTsThreadMutex lock");
					printk(KERN_ERR "%s: req_id is valid; req_id= [%d], req->req_state:%d\n", __func__, req->req_id, req->req_state);
					__pm_stay_awake(&touchscreen_kthread_ctl.wake_lock);
					touchscreen_kthread_ctl.resp_func[req->req_id](touchscreen_kthread_ctl.data, req->req_state);
					__pm_relax(&touchscreen_kthread_ctl.wake_lock);
					printk("VIVO_TS_INFO:NotifyTsThreadMutex unlock");
					mutex_unlock(&NotifyTsThreadMutex);
				}
					} else {
						printk(KERN_ERR "%s: req_id is invalid; req_id= [%d]" "\n", __func__, req->req_id);
					}
			/*  avoid free same memory twice */
			if (req != NULL) {
				if (req->req_id == TOUCHSCREEN_REQ_ID_PROX_STATE) {
					complete(&req->done);
				} else {
					kfree(req);
					req = NULL;
				}
			}

			mutex_lock(&touchscreen_kthread_ctl.mux);
		}

		mutex_unlock(&touchscreen_kthread_ctl.mux);

	}
out:
	return 0;
}

int __init touchscreen_init_kthread(void)
{
	int rc = 0;
	struct touchscreen_req *req, *tmp;

	mutex_init(&NotifyTsThreadMutex);
	memset(&touchscreen_kthread_ctl, 0, sizeof(touchscreen_kthread_ctl));
	mutex_init(&touchscreen_kthread_ctl.mux);
	wakeup_source_init(&touchscreen_kthread_ctl.wake_lock, "vivo-ts-kthread-wakelock");
	init_waitqueue_head(&touchscreen_kthread_ctl.wait);
	INIT_LIST_HEAD(&touchscreen_kthread_ctl.req_lists);
	touchscreen_kthread = kthread_run(&touchscreen_thread, NULL, "vivo-ts-kthread");
	if (IS_ERR(touchscreen_kthread)) {
		rc = PTR_ERR(touchscreen_kthread);
		printk(KERN_ERR "%s: Failed to create kernel thread; rc = [%d]"
		       "\n", __func__, rc);
		return rc;
	}

	memset(&touchscreen_boot_ctl, 0, sizeof(touchscreen_boot_ctl));
	mutex_init(&touchscreen_boot_ctl.boot_mutex);
	init_waitqueue_head(&touchscreen_boot_ctl.wait);
	INIT_LIST_HEAD(&touchscreen_boot_ctl.boot_lists);
	vivo_touchscreen_module_init_kthread = kthread_run(&vivo_ts_module_init_kthread, NULL, "vivo-ts-module-init");
	if (IS_ERR(vivo_touchscreen_module_init_kthread)) {
		rc = PTR_ERR(vivo_touchscreen_module_init_kthread);
		printk(KERN_ERR "%s: Failed to create kernel thread; rc = [%d]"
		       "\n", __func__, rc);
		goto create_module_init_err;
	}
	return rc;

create_module_init_err:
	mutex_lock(&touchscreen_kthread_ctl.mux);
	touchscreen_kthread_ctl.flags |= TOUCHSCREEN_KTHREAD_ZOMBIE;
	list_for_each_entry_safe(req, tmp, &touchscreen_kthread_ctl.req_lists, req_list) {
		list_del(&req->req_list);
		kfree(req);
	}
	mutex_unlock(&touchscreen_kthread_ctl.mux);
	kthread_stop(touchscreen_kthread);
	wake_up(&touchscreen_kthread_ctl.wait);
	return rc;
}

void __exit touchscreen_destroy_kthread(void)
{
	struct touchscreen_req *req, *tmp;
	struct vivo_touchscreen_func *boot_func = NULL, *temp_func = NULL;


	mutex_lock(&touchscreen_boot_ctl.boot_mutex);
	list_for_each_entry_safe(boot_func, temp_func, &touchscreen_boot_ctl.boot_lists, func_lists) {
		list_del(&boot_func->func_lists);
		kfree(boot_func);
	}
	mutex_unlock(&touchscreen_boot_ctl.boot_mutex);
	kthread_stop(vivo_touchscreen_module_init_kthread);
	wake_up(&touchscreen_boot_ctl.wait);

	mutex_lock(&touchscreen_kthread_ctl.mux);
	touchscreen_kthread_ctl.flags |= TOUCHSCREEN_KTHREAD_ZOMBIE;
	list_for_each_entry_safe(req, tmp, &touchscreen_kthread_ctl.req_lists, req_list) {
		list_del(&req->req_list);
		kfree(req);
	}
	mutex_unlock(&touchscreen_kthread_ctl.mux);
	kthread_stop(touchscreen_kthread);
	wake_up(&touchscreen_kthread_ctl.wait);
}

int touchscreen_register_resp_func(int id, RESP_FUNC  pfun)
{
	int ret_val = 0;

	/*mutex_lock(&touchscreen_kthread_ctl.mux);*/
	if (id >= TOUCHSCREEN_REQ_ID_RESUME_SUSPEND &&
		id < TOUCHSCREEN_REQ_ID_MAX) {
		touchscreen_kthread_ctl.resp_func[id] = pfun;
	} else {
		printk(KERN_ERR "%s: id is invalid; id= [%d]" "\n", __func__, id);
		ret_val = -1;
	}
	/*mutex_unlock(&touchscreen_kthread_ctl.mux);*/

	return ret_val;
}

void touchscreen_set_priv_data(void *data)
{
	touchscreen_kthread_ctl.data = data;
}



int touchscreen_request_send(int id, int state)
{
	 int ret_val = 0;
	 struct touchscreen_req *req;

	 if (id >= TOUCHSCREEN_REQ_ID_RESUME_SUSPEND &&
		id < TOUCHSCREEN_REQ_ID_MAX) {
		req = kzalloc(sizeof(struct touchscreen_req), GFP_KERNEL);
		if (req != NULL) {
			req->req_id = id;
			req->req_state = state;
			init_completion(&req->done);
			printk(KERN_ERR "%s: succeed to alloc req; id= [%d]" "\n", __func__, id);
			mutex_lock(&touchscreen_kthread_ctl.mux);
			list_add_tail(&req->req_list, &touchscreen_kthread_ctl.req_lists);
			       mutex_unlock(&touchscreen_kthread_ctl.mux);
			wake_up(&touchscreen_kthread_ctl.wait);
			/*printk("VIVO_TS^%s:after wake_up.req_id=%d id=%d\n", __func__, req->req_id, id);*/
			/* next judge must use id, do not use req->id.else cause phone reboot */
			if (id == TOUCHSCREEN_REQ_ID_PROX_STATE) {
				/* avoid free same memory twice	*/
				if (req != NULL) {
					if (&req->done != NULL) {
						wait_for_completion(&req->done);
						if (req != NULL) {
							kfree(req);
							req = NULL;
							printk("VIVO_TS^%s:free req.\n", __func__);
						}
					}
				}
			}
		} else {
			printk(KERN_ERR "%s: failed to alloc req; id= [%d]" "\n", __func__, id);
			ret_val = -1;
		}

	} else {
		printk(KERN_ERR "%s: id is invalid; id= [%d]" "\n", __func__, id);
		ret_val = -1;
	}

	return ret_val;
}


arch_initcall(touchscreen_init_kthread);
module_exit(touchscreen_destroy_kthread);

MODULE_DESCRIPTION("VIVO touchscreen common");
MODULE_LICENSE("GPL v2");

