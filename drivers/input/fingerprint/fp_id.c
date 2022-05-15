#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#include "fp_id.h"

#define MAX_TIMES		7

struct kobject kobj;

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};
static const struct vreg_config const vreg_conf[] = {
	{ "vcc_silead", 3300000UL, 3300000UL, 10, },
};

static const char * const pctl_names[] = {
	"fp_gpio_pull_up",
	"fp_gpio_pull_down",
};

int get_fp_id(void);
static int fp_id;
static int fp_up;
static int fp_down;
const char *fp_project_name;
struct regulator *fp_vreg[ARRAY_SIZE(vreg_conf)];
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

static int vreg_setup(struct device *dev, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;

	printk(KERN_INFO "vreg_setup start\n");
	for (i = 0; i < ARRAY_SIZE(fp_vreg); i++) {
		const char *n = vreg_conf[i].name;

		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fp_vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get  %s\n", name);
				return -ENODEV;
			}
		}
		printk(KERN_INFO "vreg_setup vfp\n");
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fp_vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fp_vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static int select_pin_ctl(struct device *dev, const char *name) /*down */
{
	size_t i;
	int rc;
	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fingerprint_pinctrl, pinctrl_state[i]);
			if (rc)
				dev_err(dev, "bio_fp_error cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "bio_fp_error %s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};

static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

int get_fp_id(void)
{
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id = "default";
	if (fp_id == FPC_FPC1229) {
		fp_frame_id = "fpc_1229";
	} else if (fp_id == GOODIX_GF5126M) {
		fp_frame_id = "goodix_5126m";
	} else if (fp_id == GOODIX_GF5216C) {
		fp_frame_id = "goodix_5216c";
	} else if (fp_id == GOODIX_GF5269) {
		fp_frame_id = "goodix_5269";
	} else if (fp_id == GOODIX_GF3208) {
		fp_frame_id = "goodix_3208";
	} else if (fp_id == GOODIX_GF318M) {
		fp_frame_id = "goodix_318m";
	} else if (fp_id == GOODIX_GF5288) {
		fp_frame_id = "goodix_5288";
	} else if (fp_id == GOODIX_GF3658) {
		fp_frame_id = "goodix_3658";
	} else if (fp_id == FPC_FPC1511) {
		fp_frame_id = "fpc_1511";
	} else if (fp_id == GOODIX_GF3626) {
		fp_frame_id = "sidefp_goodix_3626";
	} else if (fp_id == FPC_FPC1540) {
		fp_frame_id = "sidefp_fpc_1540";
	} else if (fp_id == GOODIX_GF9518) {
		fp_frame_id = "udfp_goodix_gf9518";
	} else if (fp_id == GOODIX_GF9518N) {
		fp_frame_id = "udfp_goodix2_gf9518";
	} else if (fp_id == EGIS_ET713) {
		fp_frame_id = "udfp_egis_et713";
	} else if (fp_id == SILEAD_GSL7001) {
		fp_frame_id = "udfp_silead_gsl7001";
	} else if (fp_id == GOODIX_GF9578) {
		fp_frame_id = "udfp_goodix_gf9578";
	}
	printk("fp_id_int get_fp_id=%d, fp_frame_id=%s, fp_down=%d, fp_up=%d.\n", get_fp_id(), fp_frame_id, fp_down, fp_up);
	return snprintf(buf, strlen(fp_frame_id) + 2, "%s", fp_frame_id);

}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
};

static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

static int fp_id_probe(struct platform_device *pdev)
{
	int ret;
	int fp_gpio = -1;
	int i;
	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		printk("%s: Create fp_id error!\n", __func__);
		return -1;
	}
	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		printk(KERN_ERR "bio_fp_error %s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
	}
	printk("%s:vivo,project-name = %s\n", __func__, fp_project_name);
	if ((!strncmp(fp_project_name, "PD2080", 6)) || (!strncmp(fp_project_name, "PD2083", 6))) {
		fp_id = GOODIX_GF9578;
		printk("direct return GOODIX_GF9578 \n");
		return 0;
	}
	if ((!strncmp(fp_project_name, "PD2079", 6)) || (!strncmp(fp_project_name, "PD2062", 6))) {
		fp_id = GOODIX_GF9578;
		printk("direct return GOODIX_GF9578 \n");
		return 0;
	}
	if (!strncmp(fp_project_name, "PD2078F_EX", 10)) {
		fp_id = GOODIX_GF9578;
		printk("direct return GOODIX_GF9578 \n");
		return 0;
	}
	if (!strncmp(fp_project_name, "TD1909", 6)) {
		fp_id = GOODIX_GF9518;
		printk("TD1909 direct return GOODIX_GF9518 \n");
		return 0;
	}
	if (!strncmp(fp_project_name, "TD2001", 6)) {
		fp_id = GOODIX_GF3658;
		printk("TD2001 direct return GOODIX_GF3658 \n");
		return 0;
	}
	if (!strncmp(fp_project_name, "PD1913F_EX", 10)) {
		fp_id = GOODIX_GF9518;
		printk("PD1913F_EX direct return GOODIX_GF9518 \n");
		return 0;
	}
	if ((!strncmp(fp_project_name, "PD2031", 6)) || (!strncmp(fp_project_name, "PD2048", 6))) {
		ret = vreg_setup(&pdev->dev, "vcc_silead", true);
		if (ret) {
			printk(KERN_INFO "%s:can not set vreg\n", __func__);
			return -EINVAL;
		}
		mdelay(5);
	}
	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		printk("%s: get fp_id gpio failed!\n", __func__);
		return -1;
	}
	printk("%s:fp gpio: %d \n", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id,gpios");
	if (ret)  {
		printk("%s: request fp_id gpio failed!\n", __func__);
		return -EINVAL;
	}

	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		if (PTR_ERR(fingerprint_pinctrl) == -EPROBE_DEFER) {
			printk(KERN_ERR "bio_fp_error %s: pinctrl not ready!\n", __func__);
			return -EINVAL;
		}
		printk(KERN_ERR "%s: Target does not use pinctrl\n", __func__);
		fingerprint_pinctrl = NULL;
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			printk(KERN_ERR "bio_fp_error %s: cannot find '%s'\n", __func__, n);
			return -EINVAL;
		}
		printk(KERN_INFO "%s: found pin control %s\n", __func__, n);
		pinctrl_state[i] = state;
	}

	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
	mdelay(5);
	if (ret)
		return -EINVAL;
	fp_up = gpio_get_value(fp_gpio);
	/*printk(KERN_INFO "%s: set fp-id pull up,get gpio value = %d\n", __func__, fp_up); */
	ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
	mdelay(5);
	if (ret)
		return -EINVAL;
	fp_down = gpio_get_value(fp_gpio);

	if ((!strncmp(fp_project_name, "PD2031", 6))) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = EGIS_ET713;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("PD2031 return EGIS_ET713: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		} else if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF9518N;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("PD2031 return GOODIX_GF9518N: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		} else if ((fp_up == 1) && (fp_down == 1)) {
			fp_id = SILEAD_GSL7001;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
			printk("PD2031 return SILEAD_GSL7001: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull up error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		}
	} else if ((!strncmp(fp_project_name, "PD1934", 6)) || (!strncmp(fp_project_name, "PD1934F_EX", 10))) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("PD1934 return GOODIX_GF3658: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		} else {
			fp_id = FPC_FPC1511;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("PD1934 return FPC_FPC1511: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		}
	}  else if ((!strncmp(fp_project_name, "PD2057", 6)) || (!strncmp(fp_project_name, "PD2066F_EX", 10)) || (!strncmp(fp_project_name, "PD1986", 6)) || (!strncmp(fp_project_name, "PD2066", 6)) \
			|| (!strncmp(fp_project_name, "PD2103F_EX", 10))) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3626;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk(" sidefp return GOODIX_GF3626: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		} else {
			fp_id = FPC_FPC1540;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("sidefp return FPC_FPC1540: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		}
	} else if (!strncmp(fp_project_name, "PD2048", 6)) {
		if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF9578;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
			printk("PD2031 return GOODIX_GF9578: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		} else if ((fp_up == 1) && (fp_down == 1)) {
			fp_id = SILEAD_GSL7001;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_up");
			printk("PD2031 return SILEAD_GSL7001: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull up error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -EINVAL;
			}
		}
	} else if (!strncmp(fp_project_name, "PD1831", 6)) {
		if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF5288;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		} else {
			fp_id = GOODIX_GF3658;
			ret = select_pin_ctl(&pdev->dev, "fp_gpio_pull_down");
		}
	}
	gpio_free(fp_gpio);
	if ((!strncmp(fp_project_name, "PD2031", 6)) || (!strncmp(fp_project_name, "PD2048", 6))) {
		ret = vreg_setup(&pdev->dev, "vcc_silead", false);
		if (ret) {
			printk(KERN_INFO "%s:can not set vreg\n", __func__);
			return -EINVAL;
		}
		mdelay(10);
	}
	return 0;
}

static int
fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};
#endif

static struct platform_driver fp_id_driver = {
	.probe      = fp_id_probe,
	.remove     = fp_id_remove,
	.driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
	},
};

static int __init fp_id_init(void)
{
	return platform_driver_register(&fp_id_driver);
}
late_initcall(fp_id_init);

static void __exit fp_id_exit(void)
{
	platform_driver_unregister(&fp_id_driver);
}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
