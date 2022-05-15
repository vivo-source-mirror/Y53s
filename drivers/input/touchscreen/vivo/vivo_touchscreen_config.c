#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/vivo_touchscreen_config.h>
#include <linux/vivo_touchscreen_common.h>


#define DRIVER_NAME "ts-configure"


typedef struct {
	const char	*product_name;
	const char	**use_ic_name_list;
	const char	*vdd_mode;
	int	  use_ic_num;
	int	  function_set[FS_FUNCTION_END];
	int	  module_id_gpio[2];

	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_active;
	struct pinctrl_state *pin_disactive;


} touchscreen_config;

static touchscreen_config ts_config;

static const char *function_set_name[] = {
"vivo,ts-gloves-mode",
"vivo,ts-large-obj-suppression",
"vivo,ts-dclick-wake",
"vivo,ts-gesture-mode",
"vivo,ts-custom-gesture",
"vivo,ts-module-id-methods",
"vivo,ts-anti-esd",
"vivo,ts-charger-conn-judge",
"vivo,ts-module-smt-mode",
"vivo,ts-fw-upgrade-lcd-referrence",
"vivo,ts-sensitivity-adjust",

};

/*================================================================================================*/
/*Interfaces code for the touchscreen driver													  */
/*================================================================================================*/
int vivo_touchscreen_test_ic_in_use(const char *ic_name)
{
	int i;

	for (i = 0; i < ts_config.use_ic_num; i++) {
		if (!strcmp(ts_config.use_ic_name_list[i], ic_name)) {
			return	1;
		}
	}

	return 0;
}

/*
function_set is from FS_GLOVES_MODE to FS_FUNCTION_END
*/
int vivo_touchscreen_is_support(int function)
{
	int *pfunc_start = ts_config.function_set;
	if (function >= FS_FUNCTION_BEG && function < FS_FUNCTION_END) {
		return pfunc_start[function];
	}

	return 0;
}

int vivo_touchscreen_get_module_id_gpio(int idx)
{
	int ret = -1;

	if (idx > 0 && idx < 3)
		ret = ts_config.module_id_gpio[idx - 1];

	return ret;
}

void vivo_touchscreen_get_product_name(const char **pp_Product_name)
{
	if (pp_Product_name != NULL) {
		*pp_Product_name = ts_config.product_name;
	} else {
		*pp_Product_name = "ERROR!!!";
	}
}

/*zhj add for judge gpio mode or regulator mode*/
void vivo_touchscreen_get_vdd_which_mode(const char **vdd_which_mode)
{
	if (vdd_which_mode != NULL) {
		*vdd_which_mode = ts_config.vdd_mode;
	} else {
		*vdd_which_mode = "ERROR";
	}

}
/*zhj add end*/

static int vivo_touchscreen_id_pinctrl_init(struct platform_device *pdev)
{
	ts_config.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(ts_config.pinctrl)) {
		VIVO_TS_LOG_ERR("Failed to get pinctrl!!\n");
		/*printk(DRV_NAME" %s:Failed to get pinctrl\n", __func__);*/
		return PTR_ERR(ts_config.pinctrl);
	}

	ts_config.pin_active =
		pinctrl_lookup_state(ts_config.pinctrl, "ts_id_active");
	if (IS_ERR_OR_NULL(ts_config.pin_active)) {
			/*printk(DRV_NAME" %s:Failed to look up active state\n", __func__);*/
			VIVO_TS_LOG_ERR("Failed to look up active state\n");
		return PTR_ERR(ts_config.pin_active);
	}

	ts_config.pin_disactive =
		pinctrl_lookup_state(ts_config.pinctrl, "ts_id_disactive");
	if (IS_ERR_OR_NULL(ts_config.pin_disactive)) {
			/*printk(DRV_NAME" %s:Failed to look up disactive state\n", __func__);*/
		VIVO_TS_LOG_ERR("Failed to look up disactive state\n");
		return PTR_ERR(ts_config.pin_disactive);
	}

	return 0;
}

int vivo_touchscreen_id_pinctrl(int active)
{
	int err = -1;
	struct pinctrl_state *pin_state = (active != 0) ? ts_config.pin_active : ts_config.pin_disactive;

	if (IS_ERR_OR_NULL(ts_config.pinctrl)) {
		/*printk(DRV_NAME" %s:pinctrl is invalid!!\n", __func__);*/
		VIVO_TS_LOG_ERR("pinctrl is invalid!!\n");
		return PTR_ERR(ts_config.pinctrl);
	}

	err = pinctrl_select_state(ts_config.pinctrl, pin_state);
	if (err) {
		/*printk(DRV_NAME" %s:Failed to set state to %d\n", __func__,active);*/
		VIVO_TS_LOG_ERR("Failed to set state to %d\n", active);
	}

	return err;
}

EXPORT_SYMBOL(vivo_touchscreen_id_pinctrl);
EXPORT_SYMBOL(vivo_touchscreen_get_product_name);
EXPORT_SYMBOL(vivo_touchscreen_get_module_id_gpio);
EXPORT_SYMBOL(vivo_touchscreen_test_ic_in_use);
EXPORT_SYMBOL(vivo_touchscreen_is_support);
EXPORT_SYMBOL(vivo_touchscreen_get_vdd_which_mode);
/*================================================================================================*/
/*debug sys fs interfaces										   */
/*================================================================================================*/
static ssize_t vivo_touchscreen_config_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, cnt;

	cnt = snprintf(buf, PAGE_SIZE, "use_ic_num=%d\n", ts_config.use_ic_num);

	/*printk(DRV_NAME" %s:product-name=%s\n", __func__,ts_config.product_name);*/
	VIVO_TS_LOG_INF("product-name=%s\n", ts_config.product_name);

	for (i = 0; i < ts_config.use_ic_num; i++) {
		/*printk(DRV_NAME" %s:use_ic_name_list[%d]=%s\n", __func__,i,ts_config.use_ic_name_list[i]);*/
		VIVO_TS_LOG_INF("use_ic_name_list[%d]=%s\n", i, ts_config.use_ic_name_list[i]);
	}

	for (i = 0; i < FS_FUNCTION_END; i++) {
		 /*printk(DRV_NAME" %s:[%s]=<%d>\n", __func__,function_set_name[i],ts_config.function_set[i]);*/
		 VIVO_TS_LOG_INF("[%s]=<%d>\n", function_set_name[i], ts_config.function_set[i]);
	}

	for (i = 0; i < 2; i++) {
		 /*printk(DRV_NAME" %s:gpio[%d]=<%d>\n", __func__,i,ts_config.module_id_gpio[i]);*/
		 VIVO_TS_LOG_INF("gpio[%d]=<%d>\n", i, ts_config.module_id_gpio[i]);
	}

	return cnt;
}

static ssize_t vivo_touchscreen_config_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

static struct device_attribute vivo_touchscreen_config_debug_attrs[] = {
	 __ATTR(vtcdebug, 0644,
			vivo_touchscreen_config_debug_show,
			vivo_touchscreen_config_debug_store)
};

static int	vivo_touchscreen_config_debug_create(struct platform_device *pdev)
{
	int i;
	int rc;
	int cnt = sizeof(vivo_touchscreen_config_debug_attrs) / sizeof(vivo_touchscreen_config_debug_attrs[0]);

	for (i = 0; i < cnt; i++) {
		rc = sysfs_create_file(&pdev->dev.kobj, &vivo_touchscreen_config_debug_attrs[i].attr);
		if (rc < 0) {
			/*printk(DRV_NAME" %s: Failed to create sysfs attributes\n", __func__);*/
			VIVO_TS_LOG_ERR("Failed to create sysfs attributes\n");
			goto err_sysfs;
		}
	}

err_sysfs:
	return rc;
}

/*================================================================================================*/
/*load device tree driver code for touchscreen configure										  */
/*================================================================================================*/

static int vivo_touchscreen_config_load_product_name(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int rc = -1;
	const char *strval = NULL;
	const char *propname = "vivo,ts-product-name";

	rc = of_property_read_string(pdev->dev.of_node, propname, &strval);
	if (rc) {
		/*printk(DRV_NAME" Property '%s' could not be read: %d\n",propname, rc);*/
		VIVO_TS_LOG_ERR("Property '%s' could not be read: %d\n", propname, rc);
		return -EINVAL;
	}

	ts_cfg->product_name = strval;

	return rc;
}

/*zhj add for judge gpio mode or regulator mode*/
static int vivo_touchscreen_config_load_vdd_which_mode(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int rc = -1;
	const char *strval = NULL;
	const char *propname = "vivo,ts_vdd_mode";

	rc = of_property_read_string(pdev->dev.of_node, propname, &strval);
	if (rc) {
		/*printk(DRV_NAME" Property '%s' could not be read: %d\n",propname, rc);*/
		VIVO_TS_LOG_INF("Property '%s' could not be read: %d\n", propname, rc);
		ts_cfg->vdd_mode = "regulator_mode";
		return -EINVAL;
	}

	ts_cfg->vdd_mode = strval;

	return rc;
}
/*zhj add end*/

static int vivo_touchscreen_config_load_use_ic(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int rc = -1;
	int num_use_ic, i;
	const char *propname = "vivo,use-ic-list";
	const char	**use_ic_name_list;

	num_use_ic = of_property_count_strings(pdev->dev.of_node, propname);
	if (num_use_ic <= 0) {
		/*printk(DRV_NAME" Property '%s' does not exist\n", propname);*/
		VIVO_TS_LOG_ERR("Property '%s' does not exist\n", propname);
		return -EINVAL;
	}


	use_ic_name_list = devm_kzalloc(&pdev->dev, sizeof(const char	**) * num_use_ic, GFP_KERNEL);
	if (use_ic_name_list == NULL) {
		/*printk(DRV_NAME" ts-cfg: use_ic_name_list alloc failed.\n");*/
		VIVO_TS_LOG_ERR(" ts-cfg: use_ic_name_list alloc failed.\n");
		return -EINVAL;
	}

	for (i = 0; i < num_use_ic; i++) {
		rc = of_property_read_string_index(pdev->dev.of_node, propname,
			i, &use_ic_name_list[i]);
		if (rc) {
			VIVO_TS_LOG_ERR(" Property '%s' index %d could not be read: %d\n", propname, i, rc);
			  /*printk(DRV_NAME" Property '%s' index %d could not be read: %d\n", propname, i, rc);*/
			return -EINVAL;
		}
	}

	ts_cfg->use_ic_name_list = use_ic_name_list;
	ts_cfg->use_ic_num = num_use_ic;

	return rc;
}


static void vivo_touchscreen_config_load_function_set(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int rc = -1;
	int int_val;

	int i;

	for (i = 0; i < FS_FUNCTION_END; i++) {
		rc = of_property_read_u32(pdev->dev.of_node, function_set_name[i], &int_val);
		if (rc) {
			VIVO_TS_LOG_INF(" %s: missing '%s' in dt node\n", __func__, function_set_name[i]);
			/*printk(DRV_NAME" %s: missing '%s' in dt node\n", __func__,function_set_name[i]);*/
			/*dev_err(&pdev->dev,"%s: missing '%s' in dt node\n", __func__,function_set_name[i]);*/
			continue;
		}

		ts_config.function_set[i] = int_val;
	}
}

static void vivo_touchscreen_config_load_module_id_gpio(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int int_val;

	int_val = of_get_named_gpio_flags(pdev->dev.of_node, "vivo,ts-module-id1-gpio", 0, NULL);
	if (IS_ERR_VALUE(int_val)) {
		VIVO_TS_LOG_ERR(" missing '%s' in dt node\n", "vivo,ts-module-id1-gpio");
		int_val	= -1;
	}
	ts_config.module_id_gpio[0] = int_val;

	int_val = of_get_named_gpio_flags(pdev->dev.of_node, "vivo,ts-module-id2-gpio", 0, NULL);
	if (IS_ERR_VALUE(int_val)) {
		VIVO_TS_LOG_ERR(" missing '%s' in dt node\n", "vivo,ts-module-id2-gpio");
		int_val = -1;
	}

	ts_config.module_id_gpio[1] = int_val;
}
static int vivo_touchscreen_config_load_device_tree(struct platform_device *pdev, touchscreen_config *ts_cfg)
{
	int rc = -1;


	rc = vivo_touchscreen_config_load_product_name(pdev, ts_cfg);
	if (rc < 0)	{
		VIVO_TS_LOG_INF(" load prj name error. \n");
		   /*printk(DRV_NAME" %s load prj name error. \n", __func__);*/
		 goto out;
	}

	rc = vivo_touchscreen_config_load_vdd_which_mode(pdev, ts_cfg);
	if (rc < 0)	{
		VIVO_TS_LOG_INF(" load touchscreen's vdd mode is error. \n");
		/* goto out;*/
	}

	rc = vivo_touchscreen_config_load_use_ic(pdev, ts_cfg);
	if (rc < 0)	{
		VIVO_TS_LOG_INF(" load use_ic error. \n");
		 /*printk(DRV_NAME" %s load use_ic error. \n", __func__);*/
		 goto out;
	}

	vivo_touchscreen_config_load_function_set(pdev, ts_cfg);

out:
	return rc;
}

static int vivo_touchscreen_config_probe(struct platform_device *pdev)
{

	int rc = -1;

	/*printk(DRV_NAME" %s start.... \n", __func__);*/
	VIVO_TS_LOG_INF("vivo_touchscreen_config_probe start.... \n");
	memset((void *)&ts_config, 0, sizeof(ts_config));

	rc = vivo_touchscreen_config_load_device_tree(pdev, &ts_config);

	if (vivo_touchscreen_is_support(FS_TS_MODULE_ID_METHODS) == TMID_BY_GPIO) {

		vivo_touchscreen_config_load_module_id_gpio(pdev, &ts_config);

		rc	= vivo_touchscreen_id_pinctrl_init(pdev);
		if (!rc) {
			vivo_touchscreen_id_pinctrl(0);
		} else {
			/*printk(DRV_NAME" %s id pinctrl init failed!!\n", __func__);*/
			VIVO_TS_LOG_ERR(" id pinctrl init failed!!\n");
		}
	}

	vivo_touchscreen_config_debug_create(pdev);

	VIVO_TS_LOG_INF("vivo_touchscreen_config_probe end with rc=%d\n", rc);
	/*printk(DRV_NAME" %s end with rc=%d\n", __func__,rc);*/

	return 0;
}

static int vivo_touchscreen_config_remove(struct platform_device *pdev)
{

	return 0;
}

int i2c_bus_state;
EXPORT_SYMBOL_GPL(i2c_bus_state);

static int vts_i2c_suspend(struct device *dev)
{
	i2c_bus_state = 1;
	return 0;
}

static int vts_i2c_resume(struct device *dev)
{
	i2c_bus_state = 0;
	return 0;
}

struct dev_pm_ops vts_core_pm = {
	.suspend = vts_i2c_suspend,
	.resume = vts_i2c_resume,
};

static const struct of_device_id vivo_touchscreen_config_of_match[]	 = {
	{ .compatible = "vivo,touchscreen-config", },
	{},
};

static struct platform_driver vivo_touchscreen_config_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = vivo_touchscreen_config_of_match,
		.pm = &vts_core_pm,
	},
	.probe = vivo_touchscreen_config_probe,
	.remove = vivo_touchscreen_config_remove,
};


static int __init vivo_touchscreen_config_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&vivo_touchscreen_config_driver);
	if (rc < 0)
		/*pr_err("platform register failed (%s)\n", __func__);*/
		VIVO_TS_LOG_ERR(" platform register failed (%s)\n", __func__);
	return 0;
}

void __exit vivo_touchscreen_config_exit(void)
{
	platform_driver_unregister(&vivo_touchscreen_config_driver);
	return;
}
arch_initcall(vivo_touchscreen_config_init);
module_exit(vivo_touchscreen_config_exit);

MODULE_DESCRIPTION("VIVO touchscreen configure");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, vivo_touchscreen_config_of_match);

