/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/clk.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

/* vivo zhangjiebin add begin, for nfc ic check */
//#define NFC_HW_DETECT
/* vivo zhangjiebin add end */
struct nqx_platform_data {
	unsigned int irq_gpio;
	unsigned int en_gpio;
	#ifdef USE_CLK_CONTROL
	unsigned int clkreq_gpio;
	#endif
	unsigned int firm_gpio;
	unsigned int ese_gpio;
	const char *clk_src_name;
	/* NFC_CLK pin voting state */
    #ifdef USE_CLK_CONTROL
	bool clk_pin_voting;
	#endif
};

static const struct of_device_id msm_match_table[] = {
	{.compatible = "mediatek,nq-pn553"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

#define MAX_BUFFER_SIZE			(320)
#define WAKEUP_SRC_TIMEOUT		(2000)
#define MAX_RETRY_COUNT			3

#define NFC_SET_PWR			_IOW(0xE9, 0x01, unsigned int)
#define ESE_SET_PWR			_IOW(0xE9, 0x02, unsigned int)
#define ESE_GET_PWR			_IOR(0xE9, 0x03, unsigned int)
#define SET_RX_BLOCK			_IOW(0xE9, 0x04, unsigned int)
#define SET_EMULATOR_TEST_POINT		_IOW(0xE9, 0x05, unsigned int)
#define NFCC_INITIAL_CORE_RESET_NTF	_IOW(0xE9, 0x10, unsigned int)

#define NFC_RX_BUFFER_CNT_START		(0x0)
#define PAYLOAD_HEADER_LENGTH		(0x3)
#define PAYLOAD_LENGTH_MAX		(256)
#define BYTE				(0x8)
#define NCI_IDENTIFIER			(0x10)


#define NFCC_MAGIC 0xE9
#define NFCC_GET_INFO _IOW(NFCC_MAGIC, 0x09, unsigned int)

struct nqx_devinfo {
		unsigned char chip_type;
		unsigned char rom_version;
		unsigned char fw_major;
		unsigned char fw_minor;
};

union nqx_uinfo {
		unsigned int i;
		struct nqx_devinfo info;
};



enum nfcc_initial_core_reset_ntf {
	TIMEDOUT_INITIAL_CORE_RESET_NTF = 0, /* 0*/
	ARRIVED_INITIAL_CORE_RESET_NTF, /* 1 */
	DEFAULT_INITIAL_CORE_RESET_NTF, /*2*/
};

enum nfcc_chip_variant {
	NFCC_NQ_210			= 0x48,	/**< NFCC NQ210 */
	NFCC_NQ_220			= 0x58,	/**< NFCC NQ220 */
	NFCC_NQ_310			= 0x40,	/**< NFCC NQ310 */
	NFCC_NQ_330			= 0x51,	/**< NFCC NQ330 */
	NFCC_PN66T			= 0x18,	/**< NFCC PN66T */
	NFCC_NOT_SUPPORTED	        = 0xFF	/**< NFCC is not supported */
};



struct nqx_dev {
	wait_queue_head_t	read_wq;
	struct	mutex		read_mutex;
	struct	mutex		write_mutex;
	struct  mutex           ioctl_mutex;
	struct	i2c_client	*client;
	struct	miscdevice	nqx_device;
	union  nqx_uinfo	nqx_info;
	/* NFC GPIO variables */
	unsigned int		irq_gpio;
	unsigned int		en_gpio;
	unsigned int		firm_gpio;
	#ifdef USE_CLK_CONTROL
	unsigned int		clkreq_gpio;
	#endif
	unsigned int		ese_gpio;
	/* NFC VEN pin state powered by Nfc */
	bool			nfc_ven_enabled;
	/* NFC_IRQ state */
	bool			irq_enabled;
	/* NFC_IRQ wake-up state */
	bool			irq_wake_up;
	spinlock_t		irq_enabled_lock;
	unsigned int		count_irq;
	/* Initial CORE RESET notification */
	unsigned int		core_reset_ntf;
	/* CLK control */
	 #ifdef USE_CLK_CONTROL
	bool			clk_run;
	struct	clk		*s_clk;
	#endif
	/* read buffer*/
	size_t kbuflen;
	u8 *kbuf;
	struct nqx_platform_data *pdata;

};

static int nfcc_reboot(struct notifier_block *notifier, unsigned long val,
			void *v);
/*clock enable function*/
#ifdef USE_CLK_CONTROL
static int nqx_clock_select(struct nqx_dev *nqx_dev);
/*clock disable function*/
static int nqx_clock_deselect(struct nqx_dev *nqx_dev);
#endif
static struct notifier_block nfcc_notifier = {
	.notifier_call	= nfcc_reboot,
	.next			= NULL,
	.priority		= 0
};

unsigned int	disable_ctrl;

unsigned int   nfc_result;
struct kobject *nfc_kobject;
unsigned int nfc_support;
unsigned int boardversion_mask;
unsigned int boardversion_shift;
unsigned int boardversion_num;

const char *boardversions[20];

static void nqx_init_stat(struct nqx_dev *nqx_dev)
{
	nqx_dev->count_irq = 0;
}

static void nqx_disable_irq(struct nqx_dev *nqx_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	if (nqx_dev->irq_enabled) {
		disable_irq_nosync(nqx_dev->client->irq);
		nqx_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
}

/**
 * nqx_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: void
 */
static void nqx_enable_irq(struct nqx_dev *nqx_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	if (!nqx_dev->irq_enabled) {
		nqx_dev->irq_enabled = true;
		enable_irq(nqx_dev->client->irq);
	}
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
}

static irqreturn_t nqx_dev_irq_handler(int irq, void *dev_id)
{
	struct nqx_dev *nqx_dev = dev_id;
	unsigned long flags;

	if (device_may_wakeup(&nqx_dev->client->dev))
		pm_wakeup_event(&nqx_dev->client->dev, WAKEUP_SRC_TIMEOUT);

	nqx_disable_irq(nqx_dev);
	spin_lock_irqsave(&nqx_dev->irq_enabled_lock, flags);
	nqx_dev->count_irq++;
	spin_unlock_irqrestore(&nqx_dev->irq_enabled_lock, flags);
	wake_up(&nqx_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t nfc_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offset)
{
	struct nqx_dev *nqx_dev = filp->private_data;
	unsigned char *tmp = NULL;
	int ret;
	int irq_gpio_val = 0;

	if (!nqx_dev) {
		ret = -ENODEV;
		goto out;
	}

	if (count > nqx_dev->kbuflen)
		count = nqx_dev->kbuflen;

	dev_dbg(&nqx_dev->client->dev, "%s : reading %zu bytes.\n",
			__func__, count);

	mutex_lock(&nqx_dev->read_mutex);

	irq_gpio_val = gpio_get_value(nqx_dev->irq_gpio);
	if (irq_gpio_val == 0) {
		if (filp->f_flags & O_NONBLOCK) {
			dev_err(&nqx_dev->client->dev,
			":f_falg has O_NONBLOCK. EAGAIN\n");
			ret = -EAGAIN;
			goto err;
		}
		while (1) {
			ret = 0;
			if (!nqx_dev->irq_enabled) {
				nqx_dev->irq_enabled = true;
				enable_irq(nqx_dev->client->irq);
			}
			if (!gpio_get_value(nqx_dev->irq_gpio)) {
				ret = wait_event_interruptible(nqx_dev->read_wq,
					!nqx_dev->irq_enabled);
			}
			if (ret)
				goto err;
			nqx_disable_irq(nqx_dev);

			if (gpio_get_value(nqx_dev->irq_gpio))
				break;
			dev_err_ratelimited(&nqx_dev->client->dev,
			"gpio is low, no need to read data\n");
		}
	}

	tmp = nqx_dev->kbuf;
	if (!tmp) {
		dev_err(&nqx_dev->client->dev,
			"%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}
	memset(tmp, 0x00, count);

	/* Read data */
	ret = i2c_master_recv(nqx_dev->client, tmp, count);
	if (ret < 0) {
		dev_err(&nqx_dev->client->dev,
			"%s: i2c_master_recv returned %d\n", __func__, ret);
		goto err;
	}
	if (ret > count) {
		dev_err(&nqx_dev->client->dev,
			"%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		ret = -EIO;
		goto err;
	}
#ifdef NFC_KERNEL_BU
		dev_dbg(&nqx_dev->client->dev, "%s : NfcNciRx %x %x %x\n",
			__func__, tmp[0], tmp[1], tmp[2]);
#endif
	if (copy_to_user(buf, tmp, ret)) {
		dev_warn(&nqx_dev->client->dev,
			"%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto err;
	}
	mutex_unlock(&nqx_dev->read_mutex);
	return ret;

err:
	mutex_unlock(&nqx_dev->read_mutex);
out:
	return ret;
}

static ssize_t nfc_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct nqx_dev *nqx_dev = filp->private_data;
	char *tmp = NULL;
	int ret = 0;

	if (!nqx_dev) {
		ret = -ENODEV;
		goto out;
	}
	if (count > nqx_dev->kbuflen) {
		dev_err(&nqx_dev->client->dev, "%s: out of memory\n",
			__func__);
		ret = -ENOMEM;
		goto out;
	}

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) {
		dev_err(&nqx_dev->client->dev, "%s: memdup_user failed\n",
			__func__);
		ret = PTR_ERR(tmp);
		goto out;
	}
	mutex_lock(&nqx_dev->write_mutex);
	ret = i2c_master_send(nqx_dev->client, tmp, count);
	if (ret != count) {
		dev_err(&nqx_dev->client->dev,
		"%s: failed to write %d\n", __func__, ret);
		ret = -EIO;
		goto out_free;
	}
#ifdef NFC_KERNEL_BU
	dev_dbg(&nqx_dev->client->dev,
			"%s : i2c-%d: NfcNciTx %x %x %x\n",
			__func__, iminor(file_inode(filp)),
			tmp[0], tmp[1], tmp[2]);
#endif
	usleep_range(1000, 1100);
out_free:
	kfree(tmp);
	mutex_unlock(&nqx_dev->write_mutex);
out:
	return ret;
}

/**
 * nqx_standby_write()
 * @buf:       pointer to data buffer
 * @len:       # of bytes need to transfer
 *
 * write data buffer over I2C and retry
 * if NFCC is in stand by mode
 *
 * Return: # of bytes written or -ve value in case of error
 */
static int nqx_standby_write(struct nqx_dev *nqx_dev,
				const unsigned char *buf, size_t len)
{
	int ret = -EINVAL;
	int retry_cnt;

	for (retry_cnt = 1; retry_cnt <= MAX_RETRY_COUNT; retry_cnt++) {
		ret = i2c_master_send(nqx_dev->client, buf, len);
		if (ret < 0) {
			dev_err(&nqx_dev->client->dev,
				"%s: write failed, Maybe in Standby Mode - Retry(%d)\n",
				 __func__, retry_cnt);
			usleep_range(1000, 1100);
		} else if (ret == len)
			break;
	}
	return ret;
}

/*
 * Power management of the eSE
 * NFC & eSE ON : NFC_EN high and eSE_pwr_req high.
 * NFC OFF & eSE ON : NFC_EN high and eSE_pwr_req high.
 * NFC OFF & eSE OFF : NFC_EN low and eSE_pwr_req low.
 */
static int nqx_ese_pwr(struct nqx_dev *nqx_dev, unsigned long int arg)
{
	int r = -1;
	const unsigned char svdd_off_cmd_warn[] =  {0x2F, 0x31, 0x01, 0x01};
	const unsigned char svdd_off_cmd_done[] =  {0x2F, 0x31, 0x01, 0x00};

	if (!gpio_is_valid(nqx_dev->ese_gpio)) {
		dev_err(&nqx_dev->client->dev,
			"%s: ese_gpio is not valid\n", __func__);
		return -EINVAL;
	}

	if (arg == 0) {
		/*
		 * We want to power on the eSE and to do so we need the
		 * eSE_pwr_req pin and the NFC_EN pin to be high
		 */
		if (gpio_get_value(nqx_dev->ese_gpio)) {
			dev_dbg(&nqx_dev->client->dev, "ese_gpio is already high\n");
			r = 0;
		} else {
			/**
			 * Let's store the NFC_EN pin state
			 * only if the eSE is not yet on
			 */
			nqx_dev->nfc_ven_enabled =
					gpio_get_value(nqx_dev->en_gpio);
			if (!nqx_dev->nfc_ven_enabled) {
				gpio_set_value(nqx_dev->en_gpio, 1);
				/* hardware dependent delay */
				usleep_range(1000, 1100);
			}
			gpio_set_value(nqx_dev->ese_gpio, 1);
			usleep_range(1000, 1100);
			if (gpio_get_value(nqx_dev->ese_gpio)) {
				dev_dbg(&nqx_dev->client->dev, "ese_gpio is enabled\n");
				r = 0;
			}
		}
	} else if (arg == 1) {
		if (nqx_dev->nfc_ven_enabled &&
			((nqx_dev->nqx_info.info.chip_type == NFCC_NQ_220) ||
			(nqx_dev->nqx_info.info.chip_type == NFCC_PN66T))) {
			/**
			 * Let's inform the CLF we're
			 * powering off the eSE
			 */
			r = nqx_standby_write(nqx_dev, svdd_off_cmd_warn,
						sizeof(svdd_off_cmd_warn));
			if (r < 0) {
				dev_err(&nqx_dev->client->dev,
					"%s: write failed after max retry\n",
					 __func__);
				return -ENXIO;
			}
			dev_dbg(&nqx_dev->client->dev,
				"%s: svdd_off_cmd_warn sent\n", __func__);

			/* let's power down the eSE */
			gpio_set_value(nqx_dev->ese_gpio, 0);
			dev_dbg(&nqx_dev->client->dev,
				"%s: nqx_dev->ese_gpio set to 0\n", __func__);

			/**
			 * Time needed for the SVDD capacitor
			 * to get discharged
			 */
			usleep_range(8000, 8100);

			/* Let's inform the CLF the eSE is now off */
			r = nqx_standby_write(nqx_dev, svdd_off_cmd_done,
						sizeof(svdd_off_cmd_done));
			if (r < 0) {
				dev_err(&nqx_dev->client->dev,
					"%s: write failed after max retry\n",
					 __func__);
				return -ENXIO;
			}
			dev_dbg(&nqx_dev->client->dev,
				"%s: svdd_off_cmd_done sent\n", __func__);
		} else {
			/**
			 * In case the NFC is off,
			 * there's no need to send the i2c commands
			 */
			gpio_set_value(nqx_dev->ese_gpio, 0);
			usleep_range(1000, 1100);
		}

		if (!gpio_get_value(nqx_dev->ese_gpio)) {
			dev_dbg(&nqx_dev->client->dev, "ese_gpio is disabled\n");
			r = 0;
		}

		if (!nqx_dev->nfc_ven_enabled) {
			/* hardware dependent delay */
			usleep_range(1000, 1100);
			dev_dbg(&nqx_dev->client->dev, "disabling en_gpio\n");
			gpio_set_value(nqx_dev->en_gpio, 0);
		}
	} else if (arg == 3) {
		r = gpio_get_value(nqx_dev->ese_gpio);
	}
	return r;
}

static int nfc_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct nqx_dev *nqx_dev = container_of(filp->private_data,
				struct nqx_dev, nqx_device);

	filp->private_data = nqx_dev;
	nqx_init_stat(nqx_dev);

	dev_dbg(&nqx_dev->client->dev,
			"%s: %d,%d\n", __func__, imajor(inode), iminor(inode));
	return ret;
}
static ssize_t nfc_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	if (nfc_result == 1)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t nfc_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	return nfc_result;
}

static DEVICE_ATTR(nfc_enable, 0664, nfc_enable_show, nfc_enable_store);

static struct attribute *nfc_attributes[] = {
	&dev_attr_nfc_enable.attr,
	NULL
};

static const struct attribute_group nfc_attr_group = {
	.attrs = nfc_attributes,
};

/*
 * nfc_ioctl_power_states() - power control
 * @filp:	pointer to the file descriptor
 * @arg:	mode that we want to move to
 *
 * Device power control. Depending on the arg value, device moves to
 * different states
 * (arg = 0): NFC_ENABLE	GPIO = 0, FW_DL GPIO = 0
 * (arg = 1): NFC_ENABLE	GPIO = 1, FW_DL GPIO = 0
 * (arg = 2): FW_DL GPIO = 1
 *
 * Return: -ENOIOCTLCMD if arg is not supported, 0 in any other case
 */
int nfc_ioctl_power_states(struct file *filp, unsigned long arg)
{
	int r = 0;
	struct nqx_dev *nqx_dev = filp->private_data;

	mutex_lock(&nqx_dev->ioctl_mutex);
	if (arg == 0) {
		/*
		 * We are attempting a hardware reset so let us disable
		 * interrupts to avoid spurious notifications to upper
		 * layers.
		 */
		nqx_disable_irq(nqx_dev);
		dev_dbg(&nqx_dev->client->dev,
			"gpio_set_value disable: %s: info: %p\n",
			__func__, nqx_dev);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 0);
			usleep_range(10000, 10100);
		}

		if (gpio_is_valid(nqx_dev->ese_gpio)) {
			if (!gpio_get_value(nqx_dev->ese_gpio)) {
				dev_dbg(&nqx_dev->client->dev, "disabling en_gpio\n");
				gpio_set_value(nqx_dev->en_gpio, 0);
				usleep_range(10000, 10100);
			} else {
				dev_dbg(&nqx_dev->client->dev, "keeping en_gpio high\n");
			}
		} else {
			dev_dbg(&nqx_dev->client->dev, "ese_gpio invalid, set en_gpio to low\n");
			gpio_set_value(nqx_dev->en_gpio, 0);
			usleep_range(10000, 10100);
		}
		#ifdef USE_CLK_CONTROL
		if (nqx_dev->pdata->clk_pin_voting) {
			r = nqx_clock_deselect(nqx_dev);
			if (r < 0)
				dev_err(&nqx_dev->client->dev, "unable to disable clock\n");
		}
		#endif
		nqx_dev->nfc_ven_enabled = false;
	} else if (arg == 1) {
		nqx_enable_irq(nqx_dev);
		dev_dbg(&nqx_dev->client->dev,
			"gpio_set_value enable: %s: info: %p\n",
			__func__, nqx_dev);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 0);
			usleep_range(10000, 10100);
		}

		if (gpio_get_value(nqx_dev->en_gpio)) {
			dev_dbg(&nqx_dev->client->dev, "VEN gpio already high\n");
		} else {
			gpio_set_value(nqx_dev->en_gpio, 1);
			usleep_range(10000, 10100);
		}

		#ifdef USE_CLK_CONTROL
		if (nqx_dev->pdata->clk_pin_voting) {
			r = nqx_clock_select(nqx_dev);
			if (r < 0)
				dev_err(&nqx_dev->client->dev, "unable to enable clock\n");
		}
		#endif
		nqx_dev->nfc_ven_enabled = true;
	} else if (arg == 2) {
		/*
		 * We are switching to Dowload Mode, toggle the enable pin
		 * in order to set the NFCC in the new mode
		 */
		if (gpio_is_valid(nqx_dev->ese_gpio)) {
			if (gpio_get_value(nqx_dev->ese_gpio)) {
				dev_err(&nqx_dev->client->dev,
				"FW download forbidden while ese is on\n");
				/*add by lixuefeng for NFC test start*/
				mutex_unlock(&nqx_dev->ioctl_mutex);
				/*add by lixuefeng for NFC test end*/
				return -EBUSY; /* Device or resource busy */
			}
		}
		gpio_set_value(nqx_dev->en_gpio, 1);
		usleep_range(10000, 10100);
		if (gpio_is_valid(nqx_dev->firm_gpio)) {
			gpio_set_value(nqx_dev->firm_gpio, 1);
			usleep_range(10000, 10100);
		}
		gpio_set_value(nqx_dev->en_gpio, 0);
		usleep_range(10000, 10100);
		gpio_set_value(nqx_dev->en_gpio, 1);
		usleep_range(10000, 10100);
	} else {
		r = -ENOIOCTLCMD;
	}
	mutex_unlock(&nqx_dev->ioctl_mutex);
	return r;
}

#ifdef CONFIG_COMPAT
static long nfc_compat_ioctl(struct file *pfile, unsigned int cmd,
				unsigned long arg)
{
	long r = 0;

	arg = (compat_u64)arg;
	switch (cmd) {
	case NFC_SET_PWR:
		nfc_ioctl_power_states(pfile, arg);
		break;
	case ESE_SET_PWR:
		nqx_ese_pwr(pfile->private_data, arg);
		break;
	case ESE_GET_PWR:
		nqx_ese_pwr(pfile->private_data, 3);
		break;
	case SET_RX_BLOCK:
		break;
	case SET_EMULATOR_TEST_POINT:
		break;
	default:
		r = -ENOTTY;
	}
	return r;
}
#endif

/*
 * nfc_ioctl_core_reset_ntf()
 * @filp:       pointer to the file descriptor
 *
 * Allows callers to determine if a CORE_RESET_NTF has arrived
 *
 * Return: the value of variable core_reset_ntf
 */
int nfc_ioctl_core_reset_ntf(struct file *filp)
{
	struct nqx_dev *nqx_dev = filp->private_data;

	dev_dbg(&nqx_dev->client->dev, "%s: returning = %d\n", __func__,
		nqx_dev->core_reset_ntf);
	return nqx_dev->core_reset_ntf;
}

/*
 * Inside nfc_ioctl_nfcc_info
 *
 * @brief   nfc_ioctl_nfcc_info
 *
 * Check the NQ Chipset and firmware version details
 */
unsigned int nfc_ioctl_nfcc_info(struct file *filp, unsigned long arg)
{
	unsigned int r = 0;
	struct nqx_dev *nqx_dev = filp->private_data;

	r = nqx_dev->nqx_info.i;
	dev_dbg(&nqx_dev->client->dev,
		"nqx nfc : nfc_ioctl_nfcc_info r = %d\n", r);

	return r;
}

static long nfc_ioctl(struct file *pfile, unsigned int cmd,
			unsigned long arg)
{
	int r = 0;

	switch (cmd) {
	case NFC_SET_PWR:
		r = nfc_ioctl_power_states(pfile, arg);
		break;
	case ESE_SET_PWR:
		r = nqx_ese_pwr(pfile->private_data, arg);
		break;
	case ESE_GET_PWR:
		r = nqx_ese_pwr(pfile->private_data, 3);
		break;
	case SET_RX_BLOCK:
		break;
	case SET_EMULATOR_TEST_POINT:
		break;
	case NFCC_INITIAL_CORE_RESET_NTF:
		r = nfc_ioctl_core_reset_ntf(pfile);
		break;
	case NFCC_GET_INFO:
		r = nfc_ioctl_nfcc_info(pfile, arg);
		break;
	default:
		r = -ENOIOCTLCMD;
	}
	return r;
}

static const struct file_operations nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read  = nfc_read,
	.write = nfc_write,
	.open = nfc_open,
	.unlocked_ioctl = nfc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nfc_compat_ioctl
#endif
};

/* vivo zhangjiebin add begin, for nfc ic check */
#ifdef NFC_HW_DETECT
static int nfcc_hw_detect(struct nqx_dev *nqx_dev)
{
	int ret = 0;
	char val[1] = {0};
	int count = 0;
	unsigned int enable_gpio = nqx_dev->en_gpio;
	unsigned int firm_gpio = nqx_dev->firm_gpio;

	gpio_set_value(firm_gpio, 1);
	msleep(10);
	do {
		gpio_set_value(enable_gpio, 0);
		/* hardware dependent delay */
		msleep(10);
		gpio_set_value(enable_gpio, 1);
		msleep(10);
		ret = i2c_master_send(nqx_dev->client, val, 0);
		if (ret < 0)
			dev_err(&nqx_dev->client->dev, "%s:returned %d;count %d\n", __func__, ret, count);
		else
			dev_err(&nqx_dev->client->dev, "%s:returned %d;count %d\n", __func__, ret, count);
	} while ((count++ < 3) && (ret < 0));

	gpio_set_value(firm_gpio, 0);
	gpio_set_value(enable_gpio, 0);
    msleep(20);
	return ret;
}
#endif
/* vivo zhangjiebin add end */

/* Check for availability of NQ_ NFC controller hardware */
#ifdef VIVO_NFCC_HW_CHECK
static int nfcc_hw_check(struct i2c_client *client, struct nqx_dev *nqx_dev)
{
	int ret = 0;
	unsigned int enable_gpio = nqx_dev->en_gpio;
	unsigned int firm_gpio = nqx_dev->firm_gpio;

	unsigned char raw_fw_get_version_cmd[] =  {0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF};
	unsigned char fw_get_version_rsp[14];


	gpio_set_value(firm_gpio, 1);
	msleep(10);
	/* making sure that the NFCC starts in a clean state. */
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	/* hardware dependent delay */
	msleep(10);
	gpio_set_value(enable_gpio, 1);/* HPD : Enable*/
	/* hardware dependent delay */
	msleep(10);
	/* send get FW Version CMD */
	ret = i2c_master_send(client, raw_fw_get_version_cmd,
						sizeof(raw_fw_get_version_cmd));
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_send Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	pr_err("%s: raw_fw_get_version success\n", __func__);
	/* hardware dependent delay */
	msleep(30);
	/* Read Response of get fw version */
	ret = i2c_master_recv(client, fw_get_version_rsp,
						sizeof(fw_get_version_rsp));
	dev_err(&client->dev,
		"%s: - nq - firm cmd answer : NfcNciRx %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n",
			__func__, fw_get_version_rsp[0], fw_get_version_rsp[1],
			fw_get_version_rsp[2], fw_get_version_rsp[3], fw_get_version_rsp[4], fw_get_version_rsp[5], fw_get_version_rsp[6],
			fw_get_version_rsp[7], fw_get_version_rsp[8], fw_get_version_rsp[9], fw_get_version_rsp[10], fw_get_version_rsp[11],
			fw_get_version_rsp[12], fw_get_version_rsp[13]);
	if (ret < 0) {
		dev_err(&client->dev,
		"%s: - i2c_master_recv Error\n", __func__);
		goto err_nfcc_hw_check;
	}
	gpio_set_value(firm_gpio, 0);
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	ret = 0;
	goto done;

err_nfcc_hw_check:
	ret = -ENXIO;
	gpio_set_value(firm_gpio, 0);
	gpio_set_value(enable_gpio, 0);/* ULPM: Disable */
	dev_err(&client->dev,
		"%s: - NFCC HW not available\n", __func__);
done:
	return ret;
}
#endif

/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
 #ifdef USE_CLK_CONTROL
static int nqx_clock_select(struct nqx_dev *nqx_dev)
{
	int r = 0;

	nqx_dev->s_clk = clk_get(&nqx_dev->client->dev, "ref_clk");

	if (nqx_dev->s_clk == NULL)
		goto err_clk;

	if (nqx_dev->clk_run == false)
		r = clk_prepare_enable(nqx_dev->s_clk);

	if (r)
		goto err_clk;

	nqx_dev->clk_run = true;

	return r;

err_clk:
	r = -1;
	return r;
}

/*
 * Routine to disable clocks
 */
static int nqx_clock_deselect(struct nqx_dev *nqx_dev)
{
	int r = -1;

	if (nqx_dev->s_clk != NULL) {
		if (nqx_dev->clk_run == true) {
			clk_disable_unprepare(nqx_dev->s_clk);
			nqx_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}
#endif
static int nfc_parse_dt(struct device *dev, struct nqx_platform_data *pdata)
{
	int r = 0;
	int i = 0;
	int ret = 0;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "vivo,nfc_support", &nfc_support);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,nfc_support, ret = %d\n", ret);
		nfc_support = 0;
	}
	// printk("nfc nfc_support:%d\n", nfc_support);
	ret = of_property_read_u32(np, "vivo,boardversion_shift", &boardversion_shift);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_shift, ret = %d\n", ret);
		boardversion_shift = 0;
	}
	// printk("nfc boardversion_shift:%d\n", boardversion_shift);
	ret = of_property_read_u32(np, "vivo,boardversion_mask", &boardversion_mask);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_mask, ret = %d\n", ret);
		boardversion_mask = 0;
	}
	// printk("nfc boardversion_mask:%d\n", boardversion_mask);
	ret = of_property_read_u32(np, "vivo,boardversion_num", &boardversion_num);
	if (ret < 0) {
		dev_err(dev, "Failure reading vivo,boardversion_num, ret = %d\n", ret);
		boardversion_num = 0;
	}
	// printk("nfc boardversion_num:%d\n", boardversion_num);
	if (boardversion_num > 0) {
		ret = of_property_read_string_array(np, "vivo,boardversions", boardversions, boardversion_num);
		if (ret < 0) {
			dev_err(dev, "Failure reading vivo,boardversions, ret = %d\n", ret);
		}
	for (i = 0; i < boardversion_num; i++)
		printk("nfc_log nfc :%s\n", boardversions[i]);
	}
	pdata->en_gpio = of_get_named_gpio(np, "mediatek,nq-ven", 0);
	dev_err(dev, "*****pdata->en_gpio is %d***********\n", pdata->en_gpio);
	if ((!gpio_is_valid(pdata->en_gpio)))
		return -EINVAL;
	disable_ctrl = pdata->en_gpio;

	pdata->irq_gpio = of_get_named_gpio(np, "mediatek,nq-irq", 0);
	dev_err(dev, "*****pdata->irq_gpio is %d***********\n", pdata->irq_gpio);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "mediatek,nq-firm", 0);
	dev_err(dev, "*****pdata->firm_gpio is %d***********\n", pdata->firm_gpio);
	if (!gpio_is_valid(pdata->firm_gpio)) {
		dev_warn(dev,
			"FIRM GPIO <OPTIONAL> error getting from OF node\n");
		pdata->firm_gpio = -EINVAL;
	}

	pdata->ese_gpio = of_get_named_gpio(np, "mediatek,nq-esepwr", 0);
	dev_err(dev, "*****pdata->ese_gpio is %d***********\n", pdata->ese_gpio);
	if (!gpio_is_valid(pdata->ese_gpio)) {
		dev_warn(dev,
			"ese GPIO <OPTIONAL> error getting from OF node\n");
		pdata->ese_gpio = -EINVAL;
	}
#ifdef USE_CLK_CONTROL
	if (of_property_read_string(np, "mediatek,clk-src", &pdata->clk_src_name))
		pdata->clk_pin_voting = false;
	else
		pdata->clk_pin_voting = true;


	pdata->clkreq_gpio = of_get_named_gpio(np, "mediatek,nq-clkreq", 0);
	if (!gpio_is_valid(pdata->clkreq_gpio)) {
		dev_warn(dev,
			"clkreq GPIO <OPTIONAL> error getting from OF node\n");
		pdata->clkreq_gpio = -EINVAL;
	}
#endif
	return r;
}

static inline int gpio_input_init(const struct device * const dev,
			const int gpio, const char * const gpio_name)
{
	int r = gpio_request(gpio, gpio_name);

	if (r) {
		dev_err(dev, "unable to request gpio [%d]\n", gpio);
		return r;
	}

	r = gpio_direction_input(gpio);
	if (r)
		dev_err(dev, "unable to set direction for gpio [%d]\n", gpio);

	return r;
}
extern char *get_board_version(void);
static int nqx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	char *board_version = NULL;
	int r = 0;
	int i = 0;
	int irqn = 0;
	struct nqx_platform_data *platform_data;
	struct nqx_dev *nqx_dev;
	dev_dbg(&client->dev, "%s: enter\n", __func__);
	nfc_kobject = kobject_create_and_add("nfc", NULL);
	if (nfc_kobject == NULL) {
		pr_err("%s: nfc node create error\n", __func__);
		return -ENOMEM;
	}

	if (sysfs_create_group(nfc_kobject, &nfc_attr_group)) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		kobject_put(nfc_kobject);
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct nqx_platform_data), GFP_KERNEL);
		if (!platform_data) {
			r = -ENOMEM;
			goto err_platform_data;
		}
		r = nfc_parse_dt(&client->dev, platform_data);
		if (r)
			goto err_free_data;
	} else{
		platform_data = client->dev.platform_data;
	}

	dev_dbg(&client->dev,
		"%s, inside nfc-nci flags = %x\n",
		__func__, client->flags);

	if (nfc_support == 1) {
		if (boardversion_num == 0) {
				dev_err(&client->dev, "The machine has NFC HW\n");
				nfc_result = 1;
		} else {
			board_version = get_board_version();
			if (board_version == NULL) {
				nfc_result = 0;
				dev_err(&client->dev, "The machine hasn't NFC HW\n");
			} else{
				dev_err(&client->dev, "nfc_log board_version is %s:***********\n", board_version);
				for (i = 0; i < boardversion_num; i++) {
					dev_err(&client->dev, "nfc_log nfc:%s,%s", boardversions[i], &(board_version[boardversion_shift]));
					if (strncmp(boardversions[i], &(board_version[boardversion_shift]), boardversion_mask) == 0) {
						dev_err(&client->dev, "The machine has NFC HW\n");
						nfc_result = 1;
						break;
					}
				}
				if (i == boardversion_num) {
					dev_err(&client->dev, "The machine hasn't NFC HW\n");
					nfc_result = 0;
				}
			}
		}
	} else {
		dev_err(&client->dev, "The machine hasn't NFC HW\n");
		nfc_result = 0;
	}

	if (platform_data == NULL) {
		dev_err(&client->dev, "%s: failed\n", __func__);
		r = -ENODEV;
		goto err_platform_data;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		r = -ENODEV;
		goto err_free_data;
	}
	nqx_dev = kzalloc(sizeof(*nqx_dev), GFP_KERNEL);
	if (nqx_dev == NULL) {
		r = -ENOMEM;
		goto err_free_data;
	}
	nqx_dev->client = client;
	nqx_dev->kbuflen = MAX_BUFFER_SIZE;
	nqx_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!nqx_dev->kbuf) {
		dev_err(&client->dev,
			"failed to allocate memory for nqx_dev->kbuf\n");
		r = -ENOMEM;
		goto err_free_dev;
	}

	if (gpio_is_valid(platform_data->en_gpio)) {
		r = gpio_request(platform_data->en_gpio, "nfc_reset_gpio");
		if (r) {
			dev_err(&client->dev,
			"%s: unable to request nfc reset gpio [%d]\n",
				__func__,
				platform_data->en_gpio);
			goto err_mem;
		}
		r = gpio_direction_output(platform_data->en_gpio, 0);
		if (r) {
			dev_err(&client->dev,
				"%s: unable to set direction for nfc reset gpio [%d]\n",
					__func__,
					platform_data->en_gpio);
			goto err_en_gpio;
		}
	} else {
		dev_err(&client->dev,
		"%s: nfc reset gpio not provided\n", __func__);
		goto err_mem;
	}

	if (gpio_is_valid(platform_data->irq_gpio)) {
		r = gpio_request(platform_data->irq_gpio, "nfc_irq_gpio");
		if (r) {
			dev_err(&client->dev, "%s: unable to request nfc irq gpio [%d]\n",
				__func__, platform_data->irq_gpio);
			goto err_en_gpio;
		}
		r = gpio_direction_input(platform_data->irq_gpio);
		if (r) {
			dev_err(&client->dev,
			"%s: unable to set direction for nfc irq gpio [%d]\n",
				__func__,
				platform_data->irq_gpio);
			goto err_irq_gpio;
		}
		irqn = gpio_to_irq(platform_data->irq_gpio);
		if (irqn < 0) {
			r = irqn;
			goto err_irq_gpio;
		}
		client->irq = irqn;
	} else {
		dev_err(&client->dev, "%s: irq gpio not provided\n", __func__);
		goto err_en_gpio;
	}
	if (gpio_is_valid(platform_data->firm_gpio)) {
		r = gpio_request(platform_data->firm_gpio,
			"nfc_firm_gpio");
		if (r) {
			dev_err(&client->dev,
				"%s: unable to request nfc firmware gpio [%d]\n",
				__func__, platform_data->firm_gpio);
			goto err_irq_gpio;
		}
		r = gpio_direction_output(platform_data->firm_gpio, 0);
		if (r) {
			dev_err(&client->dev,
			"%s: cannot set direction for nfc firmware gpio [%d]\n",
			__func__, platform_data->firm_gpio);
			goto err_firm_gpio;
		}
	} else {
		dev_err(&client->dev,
			"%s: firm gpio not provided\n", __func__);
		goto err_irq_gpio;
	}
	if (gpio_is_valid(platform_data->ese_gpio)) {
		r = gpio_request(platform_data->ese_gpio,
				"nfc-ese_pwr");
		if (r) {
			nqx_dev->ese_gpio = -EINVAL;
			dev_err(&client->dev,
				"%s: unable to request nfc ese gpio [%d]\n",
					__func__, platform_data->ese_gpio);
		} else {
			nqx_dev->ese_gpio = platform_data->ese_gpio;
			r = gpio_direction_output(platform_data->ese_gpio, 0);
			if (r) {
				gpio_free(platform_data->ese_gpio);
				nqx_dev->ese_gpio = -EINVAL;
				dev_err(&client->dev,
					"%s: cannot set direction for nfc ese gpio [%d]\n",
					__func__, platform_data->ese_gpio);
			}
		}
	} else {
		nqx_dev->ese_gpio = -EINVAL;
		dev_err(&client->dev,
			"%s: ese gpio not provided\n", __func__);
	}
	#ifdef USE_CLK_CONTROL
	if (gpio_is_valid(platform_data->clkreq_gpio)) {
		r = gpio_request(platform_data->clkreq_gpio,
			"nfc_clkreq_gpio");
		if (r) {
			nqx_dev->clkreq_gpio = -EINVAL;
			dev_err(&client->dev,
				"%s: unable to request nfc clkreq gpio [%d]\n",
				__func__, platform_data->clkreq_gpio);
		} else {
			nqx_dev->clkreq_gpio = platform_data->clkreq_gpio;
			r = gpio_direction_input(platform_data->clkreq_gpio);
			if (r) {
				gpio_free(platform_data->clkreq_gpio);
				nqx_dev->clkreq_gpio = -EINVAL;
				dev_err(&client->dev,
				"%s: cannot set direction for nfc clkreq gpio [%d]\n",
				__func__, platform_data->clkreq_gpio);
			}
		}
	} else {
		nqx_dev->clkreq_gpio = -EINVAL;
		dev_err(&client->dev,
			"%s: clkreq gpio not provided\n", __func__);
	}
	#endif

	nqx_dev->en_gpio = platform_data->en_gpio;
	nqx_dev->irq_gpio = platform_data->irq_gpio;
	nqx_dev->firm_gpio  = platform_data->firm_gpio;
	nqx_dev->pdata = platform_data;

	/* init mutex and queues */
	init_waitqueue_head(&nqx_dev->read_wq);
	mutex_init(&nqx_dev->read_mutex);
	mutex_init(&nqx_dev->write_mutex);
	mutex_init(&nqx_dev->ioctl_mutex);
	spin_lock_init(&nqx_dev->irq_enabled_lock);

	nqx_dev->nqx_device.minor = MISC_DYNAMIC_MINOR;
	nqx_dev->nqx_device.name = "nq-nci";
	nqx_dev->nqx_device.fops = &nfc_dev_fops;




	r = misc_register(&nqx_dev->nqx_device);
	if (r) {
		dev_err(&client->dev, "%s: misc_register failed\n", __func__);
		goto err_misc_register;
	}
	/* NFC_INT IRQ */
	nqx_dev->irq_enabled = true;
	r = request_irq(client->irq, nqx_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, nqx_dev);
	if (r) {
		dev_err(&client->dev, "%s: request_irq failed\n", __func__);
		goto err_request_irq_failed;
	}
	nqx_disable_irq(nqx_dev);

	/*
	 * To be efficient we need to test whether nfcc hardware is physically
	 * present before attempting further hardware initialisation.
	 *
	 */

#ifdef VIVO_NFCC_HW_CHECK
	r = nfcc_hw_check(client, nqx_dev);
	if (r) {
		/* make sure NFCC is not enabled */
		gpio_set_value(platform_data->en_gpio, 0);
		/* We don't think there is hardware switch NFC OFF */
		goto err_request_hw_check_failed;
	}
#endif
	/* vivo zhangjiebin add begin, for nfc ic check */
#ifdef NFC_HW_DETECT
	if (nfc_result == 1) {
		r = nfcc_hw_detect(nqx_dev);
		if (r < 0) {
			dev_err(&client->dev, "%s: nfc ic is broken!\n", __func__);
			nfc_result = 0;
		} else {
			dev_err(&client->dev, "%s: nfc ic is ready!\n", __func__);
		}
	}
#endif
	/* vivo zhangjiebin add end */

	/* Register reboot notifier here */
	r = register_reboot_notifier(&nfcc_notifier);
	if (r) {
		dev_err(&client->dev,
			"%s: cannot register reboot notifier(err = %d)\n",
			__func__, r);
		/*
		 * nfcc_hw_check function not doing memory
		 * allocation so using same goto target here
		 */
		goto err_request_hw_check_failed;
	}

#ifdef NFC_KERNEL_BU
#ifdef USE_CLK_CONTROL
	r = nqx_clock_select(nqx_dev);
	if (r < 0) {
		dev_err(&client->dev,
			"%s: nqx_clock_select failed\n", __func__);
		goto err_clock_en_failed;
	}
#endif
	gpio_set_value(platform_data->en_gpio, 1);
#endif
	device_init_wakeup(&client->dev, true);
	device_set_wakeup_capable(&client->dev, true);
	i2c_set_clientdata(client, nqx_dev);
	nqx_dev->irq_wake_up = false;

	dev_err(&client->dev,
	"%s: probing NFCC NQxxx exited successfully\n",
		 __func__);
	return 0;

#ifdef NFC_KERNEL_BU
err_clock_en_failed:
	unregister_reboot_notifier(&nfcc_notifier);
#endif
err_request_hw_check_failed:
	free_irq(client->irq, nqx_dev);
err_request_irq_failed:
	misc_deregister(&nqx_dev->nqx_device);

err_misc_register:
	mutex_destroy(&nqx_dev->read_mutex);
	mutex_destroy(&nqx_dev->write_mutex);
	mutex_destroy(&nqx_dev->ioctl_mutex);
	#ifdef USE_CLK_CONTROL
	if (nqx_dev->clkreq_gpio > 0)
		gpio_free(nqx_dev->clkreq_gpio);
	#endif
	/* optional gpio, not sure was configured in probe */
	if (nqx_dev->ese_gpio > 0)
		gpio_free(nqx_dev->ese_gpio);
err_firm_gpio:
	gpio_free(platform_data->firm_gpio);
err_irq_gpio:
	gpio_free(platform_data->irq_gpio);
err_en_gpio:
	gpio_free(platform_data->en_gpio);
err_mem:
	kfree(nqx_dev->kbuf);
err_free_dev:
	kfree(nqx_dev);
err_free_data:
	if (client->dev.of_node)
		devm_kfree(&client->dev, platform_data);
err_platform_data:
	dev_err(&client->dev,
	"%s: probing nqxx failed, check hardware\n",
		 __func__);
	return r;
}

static int nqx_remove(struct i2c_client *client)
{
	int ret = 0;
	struct nqx_dev *nqx_dev;

	nqx_dev = i2c_get_clientdata(client);
	if (!nqx_dev) {
		dev_err(&client->dev,
		"%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	unregister_reboot_notifier(&nfcc_notifier);
	free_irq(client->irq, nqx_dev);
	misc_deregister(&nqx_dev->nqx_device);
	mutex_destroy(&nqx_dev->read_mutex);
	mutex_destroy(&nqx_dev->write_mutex);
	mutex_destroy(&nqx_dev->ioctl_mutex);
	#ifdef USE_CLK_CONTROL
	if (nqx_dev->clkreq_gpio > 0)
	gpio_free(nqx_dev->clkreq_gpio);
	#endif
	/* optional gpio, not sure was configured in probe */
	if (nqx_dev->ese_gpio > 0)
		gpio_free(nqx_dev->ese_gpio);
	gpio_free(nqx_dev->firm_gpio);
	gpio_free(nqx_dev->irq_gpio);
	gpio_free(nqx_dev->en_gpio);
	sysfs_remove_group(nfc_kobject, &nfc_attr_group);
	kobject_put(nfc_kobject);
	kfree(nqx_dev->kbuf);
	if (client->dev.of_node)
		devm_kfree(&client->dev, nqx_dev->pdata);

	kfree(nqx_dev);
err:
	return ret;
}

static int nqx_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nqx_dev *nqx_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && nqx_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			nqx_dev->irq_wake_up = true;
	}
	return 0;
}

static int nqx_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nqx_dev *nqx_dev = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev) && nqx_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			nqx_dev->irq_wake_up = false;
	}
	return 0;
}

static const struct i2c_device_id nqx_id[] = {
	{"nq-pn553", 0},
	{}
};

static const struct dev_pm_ops nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nqx_suspend, nqx_resume)
};

static struct i2c_driver nqx = {
	.id_table = nqx_id,
	.probe = nqx_probe,
	.remove = nqx_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "nq-nci",
		.of_match_table = msm_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &nfc_pm_ops,
	},
};

static int nfcc_reboot(struct notifier_block *notifier, unsigned long val,
			  void *v)
{
	gpio_set_value(disable_ctrl, 1);
	return NOTIFY_OK;
}

/*
 * module load/unload record keeping
 */
static int __init nqx_dev_init(void)
{
	return i2c_add_driver(&nqx);
}
module_init(nqx_dev_init);

static void __exit nqx_dev_exit(void)
{
	unregister_reboot_notifier(&nfcc_notifier);
	i2c_del_driver(&nqx);
}
module_exit(nqx_dev_exit);

MODULE_DESCRIPTION("NFC nqx");
MODULE_LICENSE("GPL v2");
