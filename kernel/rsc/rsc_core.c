/*
 * kernel/rsc/rsc_core.c
 *
 * VIVO Resource Control.
 *
 * Base on pm_qos.
 *
 */
#include <linux/slab.h>
#include <linux/pm_qos.h>

#include <linux/vivo_rsc/rsc_internal.h>

/*==============================================================*/
/* Local function declarition					*/
/*==============================================================*/
static int rsc_main_suspend(struct device *dev);
static int rsc_main_resume(struct device *dev);
static int rsc_main_pdrv_probe(struct platform_device *pdev);
static int rsc_main_pdrv_remove(struct platform_device *pdev);

/*==============================================================*/
/* Global variables						*/
/*==============================================================*/
struct rsc_data rsc_main_info = {
	.is_enabled = true,
	.is_in_suspend = false,

	.rsc_pm_ops = {
		.suspend	= rsc_main_suspend,
		.resume	= rsc_main_resume,
		.freeze	= rsc_main_suspend,
		.thaw	= rsc_main_resume,
		.restore	= rsc_main_resume,
	},
	.rsc_pdev = {
		.name	= "vivo-rsc",
		.id	= -1,
	},
	.rsc_pdrv = {
		.probe	= rsc_main_pdrv_probe,
		.remove	= rsc_main_pdrv_remove,
		.driver	= {
			.name	= "vivo-rsc",
			.pm	= &rsc_main_info.rsc_pm_ops,
			.owner	= THIS_MODULE,
		},
	},

	.lock = __MUTEX_INITIALIZER(rsc_main_info.lock),
	.sub_module_list = LIST_HEAD_INIT(rsc_main_info.sub_module_list),
};



int rsc_main_register_sub_module(struct rsc_policy_data *policy)
{
	struct rsc_policy_data *pd;
	int ret = 0;

	rsc_lock(&rsc_main_info.lock);

	list_for_each_entry(pd, &rsc_main_info.sub_module_list, link) {
		if (pd->types == policy->types)
			goto out;
	}

	policy->is_enabled = true;
	list_add(&policy->link, &rsc_main_info.sub_module_list);

out:
	rsc_unlock(&rsc_main_info.lock);

	return ret;
}

void rsc_main_unregister_sub_module(struct rsc_policy_data *policy)
{
	rsc_lock(&rsc_main_info.lock);

	policy->is_enabled = false;

	rsc_unlock(&rsc_main_info.lock);
}

int rsc_main_update(struct rsc_upper_req *up, struct rsc_policy_data *policy)
{
	int ret = 0;
	struct rsc_pm_qos_req *pr = NULL;
	struct rsc_pm_qos_req *qpos;
	int i;
	int allocsize;
	int pm_size, qos_val_size, qos_req_size;
	rsc_lock(&rsc_main_info.lock);

	/* get pm qos data */
	list_for_each_entry(qpos, &policy->pm_qos_req_head, link) {
		if (qpos->id == up->id) {
			pr = qpos;
			goto find;
		}
	}
	pm_size 		= 	sizeof(struct rsc_pm_qos_req);
	qos_val_size 	= 	sizeof(struct rsc_qos_val) * up->v_num;
	qos_req_size 	= 	sizeof(struct pm_qos_request) * up->req_num;

	allocsize = pm_size + qos_val_size + qos_req_size;

	/* if we don't find app id add a request node */
	pr = kzalloc(allocsize, GFP_KERNEL);
	if (!pr) {
		ret = -ENOMEM;
		goto fail;
	}

	if (qos_val_size)
		pr->pval = (struct rsc_qos_val *)((unsigned long)pr + pm_size);
	else
		pr->pval = NULL;

	if (qos_req_size)
		pr->rsc_qos_req = (struct pm_qos_request *)((unsigned long)pr + pm_size + qos_val_size);
	else
		pr->rsc_qos_req = NULL;

	pr->id = up->id;
	pr->v_num = up->v_num;
	pr->req_num = up->req_num;
	list_add(&pr->link, &policy->pm_qos_req_head);

find:
	for (i = 0; i < pr->v_num; i++) {
		if (up->req_bitmap & (1 << i))
			memcpy(&pr->pval[i].data, &up->pval[i], sizeof(struct rsc_qos_udate));
	}

	pr->req_bitmap = up->req_bitmap;
	pr->minlevel = up->minlevel;
	pr->maxlevel = up->maxlevel;
	pr->timeout = up->timeout;

	policy->update_cb(pr);

fail:
	rsc_unlock(&rsc_main_info.lock);

	return ret;
}

static void rsc_main_send_request_for_suspend(void)
{

}

static int rsc_main_suspend(struct device *dev)
{
	/*rsc_info("%s: suspend callback in\n", __func__);*/

	rsc_lock(&rsc_main_info.lock);
	rsc_main_send_request_for_suspend();
	rsc_main_info.is_in_suspend = true;
	rsc_unlock(&rsc_main_info.lock);

	return 0;
}

static int rsc_main_resume(struct device *dev)
{
	/*rsc_info("%s: resume callback in\n", __func__);*/

	rsc_lock(&rsc_main_info.lock);
	rsc_main_info.is_in_suspend = false;
	rsc_unlock(&rsc_main_info.lock);

	return 0;
}

int __init resource_control_init(void)
{
#ifdef CONFIG_RSC_V2_CPU
	int error;
#endif
	rsc_info("vivo Resource Control Init\n");

#ifdef CONFIG_RSC_V2_CPU
	error = rsc_cpu_internal_init();
#endif

#ifdef CONFIG_RSC_V2_GPU
	error = rsc_gpu_init();
#endif

#ifdef CONFIG_RSC_V2_BUS
	error = rsc_bus_init();
#endif

#ifdef CONFIG_RSC_V2_MEM
	error = rsc_mem_init();
#endif

#ifdef CONFIG_RSC_V2_IO
	error = rsc_io_init();
#endif

	return 0;
}

static int rsc_main_data_init(void)
{
	int ret = 0;

	/*resource_control_init();*/

	rsc_info("@%s: done!\n", __func__);

	return ret;
}

static void rsc_main_data_deinit(void)
{
	rsc_lock(&rsc_main_info.lock);
	rsc_main_info.is_enabled = false;
	rsc_unlock(&rsc_main_info.lock);
}

static int rsc_main_pdrv_probe(struct platform_device *pdev)
{

	rsc_info("@%s: rsc probe done!\n", __func__);

	return 0;
}

static int rsc_main_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

static int __init rsc_main_init(void)
{
	int ret = 0;

	/* rsc data init */
	ret = rsc_main_data_init();
	if (ret) {
		rsc_err("fail to init rsc data @ %s()\n", __func__);
		goto fail;
	}

	/* init sysfs */
	ret = rsc_sys_api_init();
	if (ret) {
		rsc_err("fail to create rsc procfs @ %s()\n", __func__);
		goto fail;
	}

	/* register platform device/driver */
	ret = platform_device_register(&rsc_main_info.rsc_pdev);
	if (ret) {
		rsc_err("fail to register rsc device @ %s()\n", __func__);
		goto fail;
	}

	ret = platform_driver_register(&rsc_main_info.rsc_pdrv);
	if (ret) {
		rsc_err("fail to register rsc driver @ %s()\n", __func__);
		goto reg_platform_driver_fail;
	}

	rsc_info("rsc driver init done!\n");

	return ret;

reg_platform_driver_fail:
	platform_device_unregister(&rsc_main_info.rsc_pdev);

fail:
	rsc_main_info.is_enabled = false;
	rsc_err("rsc driver init fail!\n");

	return ret;
}

static void __exit rsc_main_exit(void)
{

	platform_driver_unregister(&rsc_main_info.rsc_pdrv);
	platform_device_unregister(&rsc_main_info.rsc_pdev);

	rsc_main_data_deinit();
}

/*late_initcall(rsc_main_init);*/
arch_initcall(rsc_main_init);
/*module_init(rsc_main_init);*/
module_exit(rsc_main_exit);

MODULE_DESCRIPTION("VIVO RSC Driver v0.2");
MODULE_LICENSE("GPL");
