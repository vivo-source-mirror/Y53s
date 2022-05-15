#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <uapi/asm-generic/unistd.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include "../inc/timed_output.h"

#define VIB8846_DEV_NAME "vivo,vib_8846"

#define GPIO_PINCTRL_INTERFACE
#define MTK_PWM_INTERFACE

/* #define PWM_NEW_FRAMEWORK_API */
/* #define PWM_NEW_FRAMEWORK_API_DEBUG */

#ifdef MTK_PWM_INTERFACE
#include <mt-plat/mtk_pwm.h>

#define MTK_PWM_SOURCE_NO  1
#endif

#ifndef MTK_PWM_INTERFACE
#if defined(PWM_NEW_FRAMEWORK_API)
struct vib_pwm_setting {
	u64	period_ns;
	u64	duty_ns;
};
#endif
#endif

struct qpnp_hap {
	struct platform_device	*pdev;
	struct timed_output_dev	timed_dev;
	struct mutex			lock;
	struct mutex			gpio_lock;
	char *cali_data;
	int		adec;
	int		dir;
	int		nenable;
	int		nsleep;
	int		dcen;
	int		timeout;
	bool	pwm_enabled;
	int		stage;
	struct	delayed_work delayed_work;
#ifdef MTK_PWM_INTERFACE
	int    duration;
#else
	struct pwm_device	*pwm_dev;
#if defined(PWM_NEW_FRAMEWORK_API)
	struct vib_pwm_setting pwm_setting;
#endif
#endif
#ifdef GPIO_PINCTRL_INTERFACE
	struct pinctrl *pinctrl;
	struct pinctrl_state *nenable_gpio_high;
	struct pinctrl_state *nenable_gpio_low;
	struct pinctrl_state *nsleep_gpio_high;
	struct pinctrl_state *nsleep_gpio_low;
	struct pinctrl_state *dir_gpio_high;
	struct pinctrl_state *dir_gpio_low;
	struct pinctrl_state *dcen_gpio_high;
	struct pinctrl_state *dcen_gpio_low;
	struct pinctrl_state *timeout_gpio_high;
	struct pinctrl_state *timeout_gpio_low;
#endif
};

static struct workqueue_struct *motor_work_queue;
static int vivo_step[4] = {0, 0, 0, 0};
static struct qpnp_hap *p_hap;
static struct qpnp_hap *obj_ccm_vibrator;

/*motor period */

static int motor_period;
static int motor_runtime;
static int current_dir;
static int total_time;
static int true_runtime;

static int ccm_vibrator_parse_dt (struct qpnp_hap *hap)
{
#ifndef MTK_PWM_INTERFACE
	struct platform_device *pdev = hap->pdev;
#endif
	int rc = 0;

#ifndef MTK_PWM_INTERFACE
	/*get pwm device*/
	hap->pwm_dev = of_pwm_get(pdev->dev.of_node, NULL);
	if (IS_ERR(hap->pwm_dev)) {
		rc = PTR_ERR(hap->pwm_dev);
		printk(KERN_ERR "ccm_vibrator: Cannot get PWM device rc:(%d)\n", rc);
		hap->pwm_dev = NULL;
		return rc;
	}
	/* end */
#endif
#ifdef GPIO_PINCTRL_INTERFACE
	printk(KERN_ERR "ccm_vibrator: use pinctrl instead of gpio's fuction\n");
#else
	/*adec input-PD*/
	hap->adec = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-adec", 0);
	if (hap->adec < 0) {
		/* there's no adec pin */
		printk(KERN_ERR "ccm_vibrator: there's no adec pin, the ret is %d\n", hap->adec);
	} else {
		/* use adec pin */
		rc = gpio_request(hap->adec, "vivo,motor-adec");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request vivo,motot-adec error: %d\n", rc);
		rc = gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
		gpio_set_value(hap->adec, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	/*dir input-PD*/
	hap->dir = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-dir", 0);
	if (hap->dir < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dir gpio error, the ret is %d\n", hap->dir);
	}
	printk(KERN_ERR "ccm_vibrator: motor-dir %d\n", hap->dir);
	rc = gpio_request(hap->dir, "vivo,motor-dir");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request dir gpio error: %d\n", rc);
	rc = gpio_direction_input(hap->dir);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}
	gpio_set_value(hap->dir, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}

	/* nenable PD*/
	hap->nenable = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-nenable", 0);
	if (hap->nenable < 0) {
		printk(KERN_ERR "ccm_vibrator: parse motor-nenable error, the ret is %d\n", hap->nenable);
		return hap->nenable;
	}
	rc = gpio_request(hap->nenable, "vivo,motor-nenable");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request motor-nenable gpio error: %d\n", rc);
	printk(KERN_ERR "ccm_vibrator: motor-dir %d\n", hap->nenable);
	rc = gpio_direction_input(hap->nenable);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	gpio_set_value(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}

	/* nsleep PD*/
	hap->nsleep = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-nsleep", 0);
	if (hap->nsleep < 0) {
		printk(KERN_ERR "ccm_vibrator: parse nsleep error, the ret is %d\n", hap->nsleep);
		return hap->nsleep;
	}
	rc = gpio_request(hap->nsleep, "vivo,motor-nsleep");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request nsleep gpio error: %d\n", rc);
	printk(KERN_ERR "ccm_vibrator: motor-dir %d\n", hap->nsleep);
	rc = gpio_direction_input(hap->nsleep);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	gpio_set_value(hap->nsleep, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}

	/* dcen output-low*/
	hap->dcen = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-dcen", 0);
	if (hap->dcen < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dcen error, the ret is %d\n", hap->dcen);
		return hap->dcen;
	}
	rc = gpio_request(hap->dcen, "vivo,motor-dcen");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request dcen gpio error: %d\n", rc);
	printk(KERN_ERR "ccm_vibrator: motor-dir %d\n", hap->dcen);
	rc = gpio_direction_output(hap->dcen, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* timeout output-low*/
	hap->timeout = of_get_named_gpio(pdev->dev.of_node, "vivo,motor-timeout", 0);
	if (hap->timeout < 0) {
		printk(KERN_ERR "ccm_vibrator: parse timeout error, the ret is %d\n", hap->timeout);
		return hap->timeout;
	}
	rc = gpio_request(hap->timeout, "vivo,motor-timeout");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request timeout gpio error: %d\n", rc);
	printk(KERN_ERR "ccm_vibrator: motor-dir %d\n", hap->timeout);
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}
#endif
	return rc;
}

static int ccm_vivo_enable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

	printk(KERN_ERR"ccm_vibrator: Enter the %s\n", __func__);

#ifdef GPIO_PINCTRL_INTERFACE
	if (hap->pinctrl != NULL && hap->timeout_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->timeout_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set timeout_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}

	if (hap->pinctrl != NULL && hap->dcen_gpio_high != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->dcen_gpio_high);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set dcen_gpio_high rc is %d \n", __func__, rc);
			return rc;
		}
	}

	/* 2.pull nENBL high hold 1.5ms*/
	if (hap->pinctrl != NULL && hap->nenable_gpio_high != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nenable_gpio_high);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nenable_gpio_high rc is %d \n", __func__, rc);
			return rc;
		}
	}
	usleep_range(1500, 1600);

	/* 2.pull ADEC high, hold 1us */
	/* No need ADEC */

	usleep_range(1, 3);

	/* 3.pull nsleep high, hold 1ms */
	if (hap->pinctrl != NULL && hap->nsleep_gpio_high != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nsleep_gpio_high);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nsleep_gpio_high rc is %d \n", __func__, rc);
			return rc;
		}
	}
	usleep_range(1000, 1100);
	
	if (hap->pinctrl != NULL && hap->nenable_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nenable_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nenable_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}
	usleep_range(1, 3);
#else
	/* 1.pull dcen high */
	rc = gpio_direction_output(hap->dcen, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* 2.pull nENBL high hold 1.5ms*/
	rc = gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	usleep_range(1500, 1600);

	/* 2.pull ADEC high, hold 1us */
	if (hap->adec > 0) {
		rc = gpio_direction_output(hap->adec, 1);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	usleep_range(1, 3);

	/* 3.pull nsleep high, hold 1ms */
	rc = gpio_direction_output(hap->nsleep, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	usleep_range(1000, 1100);
#endif
	return rc;
}

static int ccm_vivo_disable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

#ifdef GPIO_PINCTRL_INTERFACE
	/* pull nenable high */
	if (hap->pinctrl != NULL && hap->nenable_gpio_high != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nenable_gpio_high);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nenable_gpio_high rc is %d \n", __func__, rc);
			return rc;
		}
	}

	/* set nsleep input-PD */
	if (hap->pinctrl != NULL && hap->nsleep_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nsleep_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nsleep_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}

	/* pull timeout high, delay 0.5ms */
	if (hap->pinctrl != NULL && hap->timeout_gpio_high != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->timeout_gpio_high);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set timeout_gpio_high rc is %d \n", __func__, rc);
			return rc;
		}
	}
	usleep_range(1, 3);
	/* set dir input-PD*/
	if (hap->pinctrl != NULL && hap->dir_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->dir_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set dir_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}

	/* pull dcen low */
	if (hap->pinctrl != NULL && hap->dcen_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->dcen_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set dcen_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}

	/* set nenable input-PD*/
	if (hap->pinctrl != NULL && hap->nenable_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->nenable_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set nenable_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}
	usleep_range(20000, 21000);

	/*after 20ms, pull timeout low */
	if (hap->pinctrl != NULL && hap->timeout_gpio_low != NULL) {
		mutex_lock(&hap->gpio_lock);
		rc = pinctrl_select_state(hap->pinctrl, hap->timeout_gpio_low);
		mutex_unlock(&hap->gpio_lock);
		if (rc != 0) {
			printk(KERN_ERR "%s Unable to set timeout_gpio_low rc is %d \n", __func__, rc);
			return rc;
		}
	}
#else
	/* pull nenable high */
	rc = gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}

	/* set nsleep input-PD */
	rc = gpio_direction_input(hap->nsleep);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	gpio_set_value(hap->nsleep, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}

	/* pull timeout high, delay 0.5ms */
	rc = gpio_direction_output(hap->timeout, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}
	usleep_range(1, 3);
	/* set dir input-PD*/
	rc = gpio_direction_input(hap->dir);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}
	gpio_set_value(hap->dir, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}

	/* set adec input-PD*/
	if (hap->adec > 0) {
		rc = gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
		gpio_set_value(hap->adec, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	/* pull dcen low */
	rc = gpio_direction_output(hap->dcen, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* set nenable input-PD*/
	rc = gpio_direction_input(hap->nenable);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	gpio_set_value(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	usleep_range(20000, 21000);

	/*after 20ms, pull timeout low */
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}
#endif
	return 0;
}


static void stop_motor_work(struct qpnp_hap *hap)
{
	int rc;

#ifdef MTK_PWM_INTERFACE
	struct pwm_spec_config conf;

	conf.pwm_no = MTK_PWM_SOURCE_NO;
	conf.mode = PWM_MODE_FIFO;
	conf.clk_div = CLK_DIV1;
	conf.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	conf.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE;
	conf.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE;
	conf.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	conf.PWM_MODE_FIFO_REGS.HDURATION = hap->duration;
	conf.PWM_MODE_FIFO_REGS.LDURATION = hap->duration;
	conf.PWM_MODE_FIFO_REGS.GDURATION = 0;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xAAAAAAAA;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xAAAAAAAA;
	conf.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	/*mt_pwm_26M_clk_enable_hal(0);
	pwm_set_spec_config(&conf);*/
	mt_pwm_disable(conf.pwm_no, 0);
	printk(KERN_ERR"ccm_vibrator: stop_motor_work %d\n", hap->duration);
#else
	#if defined(PWM_NEW_FRAMEWORK_API)
		struct pwm_state pstate;
	#endif
		/*disable the pwm*/
		if (hap->pwm_dev == NULL) {
			printk(KERN_ERR"ccm_vibrator: disable pwm dev NULL!\n");
			return;
		}
	#if defined(PWM_NEW_FRAMEWORK_API)
		pwm_get_state(hap->pwm_dev, &pstate);
		pstate.enabled = false;
		pstate.period = hap->pwm_setting.period_ns;
		pstate.duty_cycle = hap->pwm_setting.duty_ns;
		pstate.output_type = PWM_OUTPUT_FIXED;
		pstate.output_pattern = NULL;
		rc = pwm_apply_state(hap->pwm_dev, &pstate);
		printk(KERN_ERR"ccm_vibrator: disable pwm_apply_state rc(%d)!\n", rc);
	#if defined(PWM_NEW_FRAMEWORK_API_DEBUG)
		printk(KERN_ERR"ccm_vibrator: pwm enable status (%d)!\n", pwm_is_enabled(hap->pwm_dev));
	#endif
	#else
		pwm_disable(hap->pwm_dev);
	#endif
#endif
	hap->pwm_enabled = 0;
	/*reset to stage 0*/
	hap->stage = 0;
	total_time = 0;
	true_runtime = 0;
	/*disable motor gpio*/
	rc = ccm_vivo_disable_vib(hap);
	if (rc < 0) {
		printk(KERN_ERR"ccm_vibrator: disable vib fail\n");
	}
	pr_err("ccm_vibrator: stop_motor_work done\n");
}



static void vivo_vib_stage_enable(struct qpnp_hap *hap, int time_ms, bool call_by_service)
{
	int rc;
	/* direction of motor's movement, 0:pop up, 1:withdraw*/
	int dir = -1;
	int runtime = 0;
	int temp_period;
	bool is_jammed = false;
	char *caller = NULL;
	/*34722;   //1800*/
	int first_p  = 41667;   /*1500*/
	int second_p = 26041;   /*2400*/
	int third_p  = 78125;   /*800*/
#ifdef MTK_PWM_INTERFACE
	struct pwm_spec_config conf;
#else
	#if defined(PWM_NEW_FRAMEWORK_API)
		struct pwm_state pstate;
	#endif
#endif

	motor_runtime = time_ms;
	caller = call_by_service ? "service" : "driver";
	dir = time_ms >> 16;
	runtime = time_ms & 0xFFFF;

#ifndef MTK_PWM_INTERFACE
	if (hap->pwm_dev == NULL) {
		printk(KERN_ERR "ccm_vibrator: enable but pwm dev NULL\n");
		return;
	}
#if defined(PWM_NEW_FRAMEWORK_API)
	pwm_get_state(hap->pwm_dev, &pstate);
#endif
#endif

	printk(KERN_ERR "ccm_vibrator: caller:%s-vivo_vib_stage_enable stage:%d  The enable value is %d, dir is %d, current_dir:%d\n", caller, hap->stage, time_ms, dir, current_dir);
	printk(KERN_ERR "ccm_vibrator: total:%d, true:%d, run:%d\n", total_time, true_runtime, runtime);

	if (runtime == 0xEEEE) {
		printk(KERN_ERR "ccm_vibrator: vibrator jammed too many times, switch to 800pps");
		is_jammed = true;
	}
  if (call_by_service) {
		if (!is_jammed) {
			rc = cancel_delayed_work_sync(&hap->delayed_work);
			if (rc) {
				printk(KERN_ERR "ccm_vibrator: call_by_service cancel delayed work successful\n");
			} else {
				printk(KERN_ERR "ccm_vibrator: call_by_service no delayed work or cancel failed\n");
			}
			if (!time_ms) {
				stop_motor_work(hap);
				/*sleep 10ms*/
				usleep_range(10000, 12000);
				pr_err("ccm_vibrator: times 0 stop motor\n");
				return;
			}
		    hap->stage = 0;
		    current_dir = dir;
		    total_time = runtime;
		    true_runtime = 0;
		} else {
			/* Enter stage 2 directly and use 800pps when jammed*/
			rc = cancel_delayed_work_sync(&hap->delayed_work);
			if (rc) {
				printk(KERN_ERR "ccm_vibrator: is_jammed delayed work successful");
				stop_motor_work(hap);
				/*sleep 10ms*/
				usleep_range(10000, 12000);
			} else {
				printk(KERN_ERR "ccm_vibrator: is_jammed cancel failed");
			}
			hap->stage = 2;
			current_dir = dir;
			total_time = 1650;
			runtime = 1650;
			true_runtime = 0;
		}
	}
	printk(KERN_ERR "ccm_vibrator: caller:%s stage %d start", caller, hap->stage);
	if (total_time <= 0) {
		stop_motor_work(hap);
		usleep_range(10000, 12000);
		return;
	}

	/* 1.set the motor mode */
	if (call_by_service || is_jammed) {
	    pr_err("ccm_vibrator: only run in call_by_service enable vib dir :%d\n", dir);
	    rc = ccm_vivo_enable_vib(hap);
	}

	if (rc < 0) {
		printk(KERN_ERR "ccm_vibrator: ccm_vivo_enable_vib occur error\n");
		return;
	}
	printk(KERN_ERR "ccm_vibrator: 1\n");

	if (hap->stage == 0) {
		/* first stage */
		temp_period = first_p;
		if (runtime > 72) {
		    runtime = 72;
		}
		printk(KERN_ERR "BBBBB ccm_vibrator: stage0  runtime:%d, period:%d\n", runtime, temp_period);
	} else if (hap->stage == 1) {
		/* second stage */
		if (runtime > 382) {
		    runtime = 382;
		}
		temp_period = second_p;
		printk(KERN_ERR "CCCCCC ccm_vibrator: stage1 runtime:%d, period:%d\n", runtime, temp_period);
	} else if (hap->stage == 2) {
		temp_period = third_p;
		printk(KERN_ERR "DDDDDD ccm_vibrator: stage2 runtime:%d, period:%d\n", runtime, temp_period);
	}
	hap->duration = (((126 * temp_period)/255)/(1000/26))-1;
	pr_err("ccm_vibrator: 1.5   period:%d, duaration:%d", temp_period, hap->duration);
#ifdef MTK_PWM_INTERFACE
	conf.pwm_no = MTK_PWM_SOURCE_NO;
	conf.mode = PWM_MODE_FIFO;
	conf.clk_div = CLK_DIV1;
	conf.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	conf.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE;
	conf.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE;
	conf.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	conf.PWM_MODE_FIFO_REGS.HDURATION = hap->duration;
	conf.PWM_MODE_FIFO_REGS.LDURATION = hap->duration;
	conf.PWM_MODE_FIFO_REGS.GDURATION = 0;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xAAAAAAAA;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xAAAAAAAA;
	conf.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
#else
	#if defined(PWM_NEW_FRAMEWORK_API)
		hap->pwm_setting.period_ns = motor_period;
		hap->pwm_setting.duty_ns = (126 * motor_period)/255;
		pstate.enabled = true;
		pstate.period = hap->pwm_setting.period_ns;
		pstate.duty_cycle = hap->pwm_setting.duty_ns;
		pstate.output_type = PWM_OUTPUT_FIXED;
		pstate.output_pattern = NULL;
	#else
		/* 2.set pwm duty cyle:100, period: 415us(2.4KHz) */
		rc = pwm_config(hap->pwm_dev, (126 * motor_period)/255, motor_period);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator: pwm config failed\n");
			return;
		}
	#endif
#endif
	/* pwm config end */

	printk(KERN_ERR "ccm_vibrator: 2\n");

#ifdef GPIO_PINCTRL_INTERFACE
	/* 3.set direction */
	if (call_by_service) {
		if (dir == 1) {
			if (hap->pinctrl != NULL && hap->dir_gpio_high != NULL) {
				mutex_lock(&hap->gpio_lock);
				rc = pinctrl_select_state(hap->pinctrl, hap->dir_gpio_high);
				mutex_unlock(&hap->gpio_lock);
				if (rc != 0) {
					printk(KERN_ERR "%s Unable to set dir_gpio_high rc is %d \n", __func__, rc);
					return;
				}
			}
		} else if (dir == 0) {
			if (hap->pinctrl != NULL && hap->dir_gpio_low != NULL) {
				mutex_lock(&hap->gpio_lock);
				rc = pinctrl_select_state(hap->pinctrl, hap->dir_gpio_low);
				mutex_unlock(&hap->gpio_lock);
				if (rc != 0) {
					printk(KERN_ERR "%s Unable to set dir_gpio_low rc is %d \n", __func__, rc);
					return;
				}
			}
		}
	}
#else
	/* 3.set direction */
	if (dir == 1) {
		rc = gpio_direction_output(hap->dir, 1);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator: Unable to set gpio: %d to 1, the rc is %d \n", hap->dir, rc);
		}
	} else if (dir == 0) {
		rc = gpio_direction_output(hap->dir, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio: %d to 0, the rc is %d \n", __func__, hap->dir, rc);
		}
	}

	/* 4.pull nenable low, hold 200ns*/
	rc = gpio_direction_output(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
	}

	usleep_range(1, 3);
#endif
	printk(KERN_ERR "ccm_vibrator: 3\n");

     mt_pwm_disable(conf.pwm_no, 0);
	/* 5.enable pwm */
	if (1) {
	#ifdef MTK_PWM_INTERFACE
		/*mt_pwm_26M_clk_enable_hal(1);*/
		rc = pwm_set_spec_config(&conf);
		printk(KERN_ERR"ccm_vibrator: pwm_set_spec_config %d\n", rc);
	#else
		#if defined(PWM_NEW_FRAMEWORK_API)
			rc = pwm_apply_state(hap->pwm_dev, &pstate);
			printk(KERN_ERR"ccm_vibrator: enable pwm_apply_state rc(%d)!\n", rc);
		#else
			printk(KERN_ERR"ccm_vibrator: before pwm_enable\n");
			rc = pwm_enable(hap->pwm_dev);
			if (rc < 0)
				printk(KERN_ERR"ccm_vibrator: pwm enable fail!\n");
		#endif
	#endif
		hap->pwm_enabled = 1;
	}

	printk(KERN_ERR "ccm_vibrator: 4 : runtime :%d\n", runtime);
	true_runtime = runtime;
	rc = queue_delayed_work(motor_work_queue, &hap->delayed_work, msecs_to_jiffies(runtime));
	if (!rc) {
		printk(KERN_ERR "ccm_vibrator: The queue_work return is %d\n", rc);
		return;
	}
}

static void ccm_vivo_vib_enable(struct timed_output_dev *dev, int time_ms)
{
	struct qpnp_hap *hap = container_of(dev, struct qpnp_hap,
				 timed_dev);
	printk(KERN_ERR "ccm_vibrator: enter vivo_vib_enable\n");
	mutex_lock(&hap->lock);
	vivo_vib_stage_enable(hap, time_ms, true);
	mutex_unlock(&hap->lock);
}

/* work func */
static void motor_work_func(struct work_struct *work)
{
	struct qpnp_hap *hap = container_of(work, struct qpnp_hap, delayed_work.work);
	int time_ms;
	printk(KERN_ERR "ccm_vibrator: motor_work_func\n");
	printk(KERN_ERR "ccm_vibrator %d stage end.total:%d, true:%d\n", hap->stage, total_time, true_runtime);
	if (hap->stage == 0 || hap->stage == 1) {
		hap->stage++;
		total_time = total_time - true_runtime;
		time_ms = ((current_dir << 16) | (total_time & 0xFFFF));
		vivo_vib_stage_enable(hap, time_ms, false);
	} else if (hap->stage == 2) {
		stop_motor_work(hap);
	}
}
/* end */

/*sysfs attributes*/
static ssize_t motor_period_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long period = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &period);
	if (rc)
		return rc;

	motor_period = (int)period*1000;
#ifdef MTK_PWM_INTERFACE
	obj_ccm_vibrator->duration = (((126 * motor_period)/255)/(1000/26))-1;
	printk(KERN_ERR"ccm_vibrator: motor duration is %d(ns)\n", obj_ccm_vibrator->duration);
#endif
	printk(KERN_ERR"ccm_vibrator: motor period is %d(ns)\n", motor_period);

	return count;
}

static ssize_t motor_period_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", (int)(motor_period/1000));
}

static ssize_t motor_runtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long time = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return rc;

	motor_runtime = (int)time;
	printk(KERN_ERR"ccm_vibrator: motor_runtime is %d\n", motor_runtime);

	return count;
}

static ssize_t motor_step_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long step = 0;
	int rc = 0,  i = 0;

	printk(KERN_ERR"ccm_vibrator: motor step is %s\n", buf);
	rc = kstrtoul(buf, 10, &step);
	if (rc)
		return rc;
	printk(KERN_ERR "ccm_vibrator: The step number is %lu\n", step);
	for (i = 3; i >= 0; i--) {
		vivo_step[i] = (int)step%10;
		step = step/10;
	}
	printk(KERN_ERR "ccm_vibrator: step is %d, %d, %d, %d\n", vivo_step[0], vivo_step[1], vivo_step[2], vivo_step[3]);

	return count;
}

static ssize_t cali_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_ERR"ccm_vibrator: cali_data is %s, count:%zd\n", buf, count);
	memset(p_hap->cali_data, 0, 30);
	memcpy(p_hap->cali_data, buf, count < 30 ? count : 30);

	return count;
}
static ssize_t cali_data_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 30, "%s\n", p_hap->cali_data);
}

static struct device_attribute qpnp_hap_attrs[] = {
	__ATTR(period, 0664, motor_period_show, motor_period_store),
	__ATTR(runtime, 0664, NULL, motor_runtime_store),
	__ATTR(step, 0664, NULL, motor_step_store),
	__ATTR(cali_data, 0664, cali_data_show, cali_data_store),
};
/*sysfs attributes end*/

/* get time api to know the remaining time */
static int qpnp_hap_get_time(struct timed_output_dev *dev)
{
	return 0;
}


static int qpnp_haptic_probe(struct platform_device *pdev)
{
	struct qpnp_hap *hap;
	int rc, i = 0;
#ifdef GPIO_PINCTRL_INTERFACE
	int err;
#endif

	printk("ccm_vibrator: Enter the probe, platform_device name is %s\n", pdev->name);
	hap = devm_kzalloc(&pdev->dev, sizeof(*hap), GFP_KERNEL);
	if (!hap)
		return -ENOMEM;

	hap->pdev = pdev;
	obj_ccm_vibrator = hap;

	dev_set_drvdata(&pdev->dev, hap);
#ifndef MTK_PWM_INTERFACE
	hap->pwm_dev = NULL;
#endif
	printk("ccm_vibrator: before qpnp_hap_parse_dt\n");
	rc = ccm_vibrator_parse_dt(hap);
	if (rc < 0) {
		pr_err("ccm_vibrator: DT parsing failed\n");
		return rc;
	}

#ifdef GPIO_PINCTRL_INTERFACE
	/*added for set pinctrl state*/
	hap->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(hap->pinctrl)) {
		err = PTR_ERR(hap->pinctrl);
		pr_err("ccm_vibrator: pinctrl %d\n", err);
		return err;
	}
	hap->nenable_gpio_high = pinctrl_lookup_state(hap->pinctrl, "motor_nenable_pin_gpio_high");
	if (IS_ERR(hap->nenable_gpio_high)) {
		err = PTR_ERR(hap->nenable_gpio_high);
		pr_err("ccm_vibrator: nenable_gpio_high %d\n", err);
		return err;
	}
	hap->nenable_gpio_low = pinctrl_lookup_state(hap->pinctrl, "motor_nenable_pin_gpio_low");
	if (IS_ERR(hap->nenable_gpio_low)) {
		err = PTR_ERR(hap->nenable_gpio_low);
		pr_err("ccm_vibrator: nenable_gpio_low %d\n", err);
		return err;
	}
	hap->nsleep_gpio_high = pinctrl_lookup_state(hap->pinctrl, "motor_nsleep_pin_gpio_high");
	if (IS_ERR(hap->nsleep_gpio_high)) {
		err = PTR_ERR(hap->nsleep_gpio_high);
		pr_err("ccm_vibrator: nsleep_gpio_high %d\n", err);
		return err;
	}
	hap->nsleep_gpio_low = pinctrl_lookup_state(hap->pinctrl, "motor_nsleep_pin_gpio_low");
	if (IS_ERR(hap->nsleep_gpio_low)) {
		err = PTR_ERR(hap->nsleep_gpio_low);
		pr_err("ccm_vibrator: nsleep_gpio_low %d\n", err);
		return err;
	}
	hap->dir_gpio_high = pinctrl_lookup_state(hap->pinctrl, "motor_dir_pin_gpio_high");
	if (IS_ERR(hap->dir_gpio_high)) {
		err = PTR_ERR(hap->dir_gpio_high);
		pr_err("ccm_vibrator: dir_gpio_high %d\n", err);
		return err;
	}
	hap->dir_gpio_low = pinctrl_lookup_state(hap->pinctrl, "motor_dir_pin_gpio_low");
	if (IS_ERR(hap->dir_gpio_low)) {
		err = PTR_ERR(hap->dir_gpio_low);
		pr_err("ccm_vibrator: dir_gpio_low %d\n", err);
		return err;
	}
	hap->dcen_gpio_high = pinctrl_lookup_state(hap->pinctrl, "motor_dcen_pin_gpio_high");
	if (IS_ERR(hap->dcen_gpio_high)) {
		err = PTR_ERR(hap->dcen_gpio_high);
		pr_err("ccm_vibrator: dcen_gpio_high %d\n", err);
		return err;
	}
	hap->dcen_gpio_low = pinctrl_lookup_state(hap->pinctrl, "motor_dcen_pin_gpio_low");
	if (IS_ERR(hap->dcen_gpio_low)) {
		err = PTR_ERR(hap->dcen_gpio_low);
		pr_err("ccm_vibrator: dcen_gpio_low %d\n", err);
		return err;
	}
	hap->timeout_gpio_high = pinctrl_lookup_state(hap->pinctrl, "motor_timeout_pin_gpio_high");
	if (IS_ERR(hap->timeout_gpio_high)) {
		err = PTR_ERR(hap->timeout_gpio_high);
		pr_err("ccm_vibrator: timeout_gpio_high %d\n", err);
		return err;
	}
	hap->timeout_gpio_low = pinctrl_lookup_state(hap->pinctrl, "motor_timeout_pin_gpio_low");
	if (IS_ERR(hap->timeout_gpio_low)) {
		err = PTR_ERR(hap->timeout_gpio_low);
		pr_err("ccm_vibrator: timeout_gpio_low %d\n", err);
		return err;
	}
#endif
	hap->stage = 0;

	hap->pwm_enabled = 0;
#ifdef MTK_PWM_INTERFACE
	hap->duration = (((126 * motor_period)/255)/(1000/26))-1;
#endif

	mutex_init(&hap->lock);
	mutex_init(&hap->gpio_lock);

	/* init delay work */
	motor_work_queue = create_singlethread_workqueue("motor_queue");
	if (!motor_work_queue) {
		pr_err("Fail to create gpio_work_queue\n");
		goto timed_output_fail;
	}
	INIT_DELAYED_WORK(&hap->delayed_work, motor_work_func);

	/*function*/
	hap->timed_dev.name = "motor";
	hap->timed_dev.get_time = qpnp_hap_get_time;
	hap->timed_dev.enable = ccm_vivo_vib_enable;

	/*register*/
	printk("ccm_vibrator: before timed_output_dev_register\n");
	rc = timed_output_dev_register(&hap->timed_dev);
	if (rc < 0) {
		pr_err("timed_output registration failed\n");
		goto timed_output_fail;
	}

	printk("ccm_vibrator: before create attributes\n");
	for (i = 0; i < ARRAY_SIZE(qpnp_hap_attrs); i++) {
		rc = sysfs_create_file(&hap->timed_dev.dev->kobj,
				&qpnp_hap_attrs[i].attr);
		if (rc < 0) {
			pr_err("sysfs creation failed\n");
			goto sysfs_fail;
		}
	}
	hap->cali_data = devm_kzalloc(&pdev->dev, 30, GFP_KERNEL);
	p_hap = hap;
	printk("ccm_vibrator: Probe ok!\n");
	return 0;

timed_output_fail:
	cancel_delayed_work_sync(&hap->delayed_work);
	mutex_destroy(&hap->lock);
	mutex_destroy(&hap->gpio_lock);
sysfs_fail:
	for (; i > 0; i--)
		sysfs_remove_file(&hap->timed_dev.dev->kobj,
			&qpnp_hap_attrs[i].attr);
	timed_output_dev_unregister(&hap->timed_dev);

	return rc;
}

static int qpnp_haptic_remove(struct platform_device *pdev)
{
	struct qpnp_hap *hap = dev_get_drvdata(&pdev->dev);
	timed_output_dev_unregister(&hap->timed_dev);
	mutex_destroy(&hap->lock);
	mutex_destroy(&hap->gpio_lock);

	return 0;
}

static const struct of_device_id vib_8846_match_table[] = {
	{ .compatible = VIB8846_DEV_NAME, },
	{ },
};

static struct platform_driver qpnp_haptic_driver = {
	.driver	 = {
		.name	= VIB8846_DEV_NAME,
		.of_match_table = vib_8846_match_table,
	},
	.probe	  = qpnp_haptic_probe,
	.remove	 = qpnp_haptic_remove,
};

static int __init qpnp_haptic_init(void)
{
	printk("qpnp_haptic_init!\n");
	return platform_driver_register(&qpnp_haptic_driver);
}
module_init(qpnp_haptic_init);

static void __exit qpnp_haptic_exit(void)
{
	printk("qpnp_haptic_exit!\n");
	return platform_driver_unregister(&qpnp_haptic_driver);
}
module_exit(qpnp_haptic_exit);

MODULE_DESCRIPTION("Vivo vib_cam haptic driver");
MODULE_LICENSE("GPL v2");
