/*
  * ist8801.c - Linux kernel modules for hall switch
 *
  * Copyright (C) 2013 Seunghwan Park <seunghwan.park@magnachip.com>
  * Copyright (C) 2014 MagnaChip Semiconductor.
 *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
 *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
*/

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "../inc/ist8801.h"
#include "../inc/akm09918.h"

/***********************************************************/
/*customer config*/
/***********************************************************/
#define IST8801_DBG_ENABLE					/* for debugging */
#define IST8801_DETECTION_MODE				IST8801_DETECTION_MODE_POLLING/*IST8801_DETECTION_MODE_POLLING /James*/
#define IST8801_INTERRUPT_TYPE				IST8801_VAL_INTSRS_INTTYPE_BESIDE
#define IST8801_SENSITIVITY_TYPE			  IST8801_VAL_INTSRS_SRS_10BIT_0_068mT
#define IST8801_PERSISTENCE_COUNT			 IST8801_VAL_PERSINT_COUNT(4)
#define IST8801_OPERATION_FREQUENCY		   IST8801_VAL_OPF_FREQ_80HZ
#define IST8801_OPERATION_RESOLUTION		  IST8801_VAL_OPF_BIT_10
#define IST8801_DETECT_RANGE_HIGH			 (60)/*Need change via test.*/
#define IST8801_DETECT_RANGE_LOW			  (50)/*Need change via test.*/
#define IST8801_RESULT_STATUS_A			   (0x01)  /* result status A ----> ==180Degree.*/
#define IST8801_RESULT_STATUS_B			   (0x02)  /* result status B ----> != 180Degree.*/
#define IST8801_EVENT_TYPE					EV_ABS  /* EV_KEY */
#define IST8801_EVENT_CODE					ABS_X   /* KEY_F1 */
#define IST8801_EVENT_DATA_CAPABILITY_MIN	 (-32768)
#define IST8801_EVENT_DATA_CAPABILITY_MAX	 (32767)

/*MagnaChip Hall Sensor power supply VDD 2.7V~3.6V, VIO 1.65~VDD*/
#define IST8801_VDD_MIN_UV	   2700000
#define IST8801_VDD_MAX_UV	   3600000
#define IST8801_VIO_MIN_UV	   1650000
#define IST8801_VIO_MAX_UV	   3600000

/***********************************************************/
/*debug macro*/
/***********************************************************/
#ifdef IST8801_DBG_ENABLE
#define dbg(fmt, args...)  printk("[IST8801-DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)
#define dbgn(fmt, args...)
#endif /* IST8801_DBG_ENABLE */
#define dbg_func_in()	   dbg("[IST8801-DBG-F.IN] %s", __func__)
#define dbg_func_out()	  dbg("[IST8801-DBG-F.OUT] %s", __func__)
#define dbg_line()		  dbg("[LINE] %d(%s)", __LINE__, __func__)
/***********************************************************/


/***********************************************************/
/*error display macro*/
/***********************************************************/
#define mxerr(pdev, fmt, args...)		   \
	dev_err(pdev, "[IST8801-ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define mxinfo(pdev, fmt, args...)		  \
	dev_info(pdev, "[IST8801-INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
/***********************************************************/

/***********************************************************/
/*static variable*/
/***********************************************************/
static ist8801_data_t *p_ist8801_data;
static int div;
/***********************************************************/


/***********************************************************/
/*function protyps*/
/***********************************************************/
/*i2c interface*/
static int  ist8801_i2c_read(struct i2c_client *client, u8 reg, u8 *rdata, u8 len);
static int  ist8801_i2c_get_reg(struct i2c_client *client, u8 reg, u8 *rdata);
static int  ist8801_i2c_write(struct i2c_client *client, u8 reg, u8 *wdata, u8 len);
static int  ist8801_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata);
/*vdd / vid power control*/
static int ist8801_set_power(struct device *dev, bool on);
/*scheduled work*/
static void ist8801_work_func(struct work_struct *work);
static void ak09918_work_func(struct work_struct *work);
/*interrupt handler*/
/* static irqreturn_t ist8801_irq_handler(int irq, void *dev_id); */
/*configuring or getting configured status*/
static void ist8801_get_reg(struct device *dev, int *regdata);
static void ist8801_set_reg(struct device *dev, int *regdata);
static int  ist8801_get_enable(struct device *dev);
static void ist8801_set_enable(struct device *dev, int enable);
static int  ist8801_get_delay(struct device *dev);
static void ist8801_set_delay(struct device *dev, int delay);
static int  ist8801_get_debug(struct device *dev);
static void ist8801_set_debug(struct device *dev, int debug);
static int  ist8801_clear_interrupt(struct device *dev);
static int  ist8801_update_interrupt_threshold(struct device *dev, short raw);
static int  ist8801_set_operation_mode(struct device *dev, int mode);
static int  ist8801_set_detection_mode(struct device *dev, u8 mode);
static int  ist8801_init_device(struct device *dev);
static int  ist8801_reset_device(struct device *dev);
static int  ist8801_set_calibration(struct device *dev);
static int  ist8801_get_calibrated_data(struct device *dev, int *data);
static int  ist8801_measure(ist8801_data_t *p_data, short *raw);
static int  ist8801_get_result_status(ist8801_data_t *p_data, int raw);
static int  ist8801_power_ctl(ist8801_data_t *data, bool on);
static int  mag_get_rawdata(struct device *dev, int16_t *data);
/***********************************************************/


/***********************************************************/
/*functions for i2c interface*/
/***********************************************************/
#define IST8801_I2C_BUF_SIZE				  (17)
static int ist8801_i2c_read(struct i2c_client *client, u8 reg, u8 *rdata, u8 len)
{
	int rc;

	/* Add By James for i2c_smbus_read_i2c_block_data */
	rc = i2c_smbus_read_i2c_block_data(client, reg, len, rdata);
	if (rc < 0) {
		mxerr(&client->dev, "i2c_transfer was failed(%d)", rc);
		return rc;
	}
	return 0;
}

static int  ist8801_i2c_get_reg(struct i2c_client *client, u8 reg, u8 *rdata)
{
	return ist8801_i2c_read(client, reg, rdata, 1);
}

static int ist8801_i2c_write(struct i2c_client *client, u8 reg, u8 *wdata, u8 len)
{
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	u8  buf[IST8801_I2C_BUF_SIZE];
	int rc;
	int i;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len+1,
			.buf = buf,
		},
	};

	if (client == NULL) {
		printk("[ERROR] %s : i2c client is NULL.\n", __func__);
		return -ENODEV;
	}

	buf[0] = reg;
	if (len > IST8801_I2C_BUF_SIZE) {
		mxerr(&client->dev, "i2c buffer size must be less than %d", IST8801_I2C_BUF_SIZE);
		return -EIO;
	}
	for (i = 0 ; i < len; i++)
		buf[i+1] = wdata[i];

	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc < 0) {
		mxerr(&client->dev, "i2c_transfer was failed (%d)", rc);
		return rc;
	}

	if (len == 1) {
		switch (reg) {
		case IST8801_REG_PERSINT:
			p_data->reg.map.persint = wdata[0];
			break;
		case IST8801_REG_INTSRS:
			p_data->reg.map.intsrs = wdata[0];
			break;
		case IST8801_REG_LTHL:
			p_data->reg.map.lthl = wdata[0];
			break;
		case IST8801_REG_LTHH:
			p_data->reg.map.lthh = wdata[0];
			break;
		case IST8801_REG_HTHL:
			p_data->reg.map.hthl = wdata[0];
			break;
		case IST8801_REG_HTHH:
			p_data->reg.map.hthh = wdata[0];
			break;
		case IST8801_REG_I2CDIS:
			p_data->reg.map.i2cdis = wdata[0];
			break;
		case IST8801_REG_SRST:
			p_data->reg.map.srst = wdata[0];
			msleep(1);
			break;
		case IST8801_REG_OPF:
			p_data->reg.map.opf = wdata[0];
			break;
		}
	}

	for (i = 0; i < len; i++)
		dbg("reg=0x%02X data=0x%02X", buf[0]+(u8)i, buf[i+1]);

	return 0;
}

static int ist8801_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata)
{
	return ist8801_i2c_write(client, reg, &wdata, sizeof(wdata));
}

/***********************************************************/



/***********************************************************/
/*vdd / vid power control*/
/***********************************************************/
static int ist8801_set_power(struct device *dev, bool on)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	if (on) {
		/*  to do for vdd power up */
		mxinfo(&client->dev, "vdd power up");

		msleep(5); /* wait 5ms */
		dbg("waiting 5ms after vdd power up");

		/*  to do vid power up */
		mxinfo(&client->dev, "vid power up");

		msleep(10); /* wait 10ms*/
		dbg("waiting 10ms after vid power up");
	} else {
		/* to do for vid power down */
		mxinfo(&client->dev, "vid power down");

		/* to do for vdd power down */
		mxinfo(&client->dev, "vdd power down");
	}
#else
	ist8801_power_ctl(p_ist8801_data, on);
#endif
	return 0;
}
/***********************************************************/


/***********************************************************/
/*functions for scheduling*/
/***********************************************************/
static void ist8801_work_func(struct work_struct *work)
{
	ist8801_data_t *p_data = container_of((struct delayed_work *)work, ist8801_data_t, work);
	unsigned long delay = msecs_to_jiffies(ist8801_get_delay(&p_data->client->dev));
	short raw = 0;
	int err = 0;

	dbg_func_in();
	err = ist8801_measure(p_data, &raw);

	printk(KERN_ERR"ist8801_work_func up : rawData--== %6d\n", raw);

	if (!err) {
		if (p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) {
			p_data->last_data = ist8801_get_result_status(p_data, raw);
		} else {
		/* ///James add here: polling mode: */
		/* ///to call ist8801_get_result_status(p_data, raw) get the status of Camera here.*/
			p_data->last_data = (int)raw;
		}

#if (IST8801_EVENT_TYPE == EV_ABS)
		input_report_abs(p_data->input_dev, IST8801_EVENT_CODE, p_data->last_data);
#elif (IST8801_EVENT_TYPE == EV_KEY)
		input_report_key(p_data->input_dev, IST8801_EVENT_CODE, p_data->last_data);
#else
#error ("[ERR] IST8801_EVENT_TYPE is not defined.")
#endif

		input_sync(p_data->input_dev);
	}

	if (p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) {
		dbg("run update_interrupt_threshold");
		ist8801_update_interrupt_threshold(&p_data->client->dev, raw);
	} else {
		schedule_delayed_work(&p_data->work, delay);
		dbg("run schedule_delayed_work");
	}
}
static void ak09918_work_func(struct work_struct *work)
{
	int ret = 0;
	ist8801_data_t *p_data = container_of((struct delayed_work *)work, ist8801_data_t, work);
	unsigned long delay = msecs_to_jiffies(1);
	ret = mag_get_rawdata(&p_ist8801_data->client->dev, &p_data->z_value);
	printk(KERN_ERR "ecompass z_value:%d", p_data->z_value);
	schedule_delayed_work(&p_data->work, delay);
}
/***********************************************************/


/***********************************************************/
/*functions for interrupt handler*/
/***********************************************************/
static irqreturn_t ist8801_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();
	if (p_ist8801_data != NULL) {
		dbg("run schedule_delayed_work");
		schedule_delayed_work(&p_ist8801_data->work, 0);
	}
	/*James add: the INTB has interrupt single now.*/
	printk("ist8801_irq_handler : the INTB has interrupt single now\n");
	return IRQ_HANDLED;
}
/***********************************************************/


/***********************************************************/
/*functions for configuring or getting configured status*/
/***********************************************************/

static void ist8801_get_reg(struct device *dev, int *regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = 0;
	err = ist8801_i2c_get_reg(client, rega, &regd);

	*regdata = 0;
	*regdata |= (err == 0) ? 0x0000 : 0xFF00;
	*regdata |= regd;
}

static void ist8801_set_reg(struct device *dev, int *regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = *regdata&0xFF;
	err = ist8801_i2c_set_reg(client, rega, regd);

	*regdata = 0;
	*regdata |= (err == 0) ? 0x0000 : 0xFF00;
}


static int ist8801_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.enable);
}


static void ist8801_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	mutex_lock(&p_data->mtx.enable);

	if (enable) {				   /*enable if state will be changed*/
		if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
			if (p_data->use_e_compass) {
				printk(KERN_ERR "ist8801_set_enable:%d", enable);
				schedule_delayed_work(&p_data->work, msecs_to_jiffies(0));
			} else {
				ist8801_set_detection_mode(dev, p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT);
				ist8801_set_operation_mode(&p_ist8801_data->client->dev, OPERATION_MODE_MEASUREMENT);
			}
		}
	} else {						/*disable if state will be changed*/
		if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
			cancel_delayed_work_sync(&p_data->work);
			if (!p_data->use_e_compass) {
				ist8801_set_operation_mode(&p_ist8801_data->client->dev, OPERATION_MODE_POWERDOWN);
			}
		}
	}
	atomic_set(&p_data->atm.enable, enable);

	mutex_unlock(&p_data->mtx.enable);
}

static int ist8801_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	int delay = 0;

	delay = atomic_read(&p_data->atm.delay);

	return delay;
}

static void ist8801_set_delay(struct device *dev, int delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	if (delay < IST8801_DELAY_MIN)
		delay = IST8801_DELAY_MIN;
	atomic_set(&p_data->atm.delay, delay);

	mutex_lock(&p_data->mtx.enable);

	if (ist8801_get_enable(dev)) {
		if (!(p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT)) {
			cancel_delayed_work_sync(&p_data->work);
			schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
		}
	}

	mutex_unlock(&p_data->mtx.enable);
}

static int ist8801_get_debug(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.debug);
}

static void ist8801_set_debug(struct device *dev, int debug)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	atomic_set(&p_data->atm.debug, debug);
}

static int ist8801_clear_interrupt(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	int ret = 0;

	ret = ist8801_i2c_set_reg(p_data->client, IST8801_REG_PERSINT, p_data->reg.map.persint | 0x01);

	return ret;
}

static void ist8801_convdata_short_to_2byte(u8 opf, short x, unsigned char *hbyte, unsigned char *lbyte)
{
	if ((opf & IST8801_VAL_OPF_BIT_8) == IST8801_VAL_OPF_BIT_8) {
		/*8 bit resolution*/
		if (x < -128)
			x = -128;
		else if (x > 127)
			x = 127;

		if (x >= 0) {
			*lbyte = x & 0x7F;
		} else {
			*lbyte = ((0x80 - (x*(-1))) & 0x7F) | 0x80;
		}
		*hbyte = 0x00;
	} else {
		/*10 bit resolution*/
		if (x < -512)
			x = -512;
		else if (x > 511)
			x = 511;

		if (x >= 0) {
			*lbyte = x & 0xFF;
			*hbyte = (((x&0x100)>>8)&0x01) << 6;
		} else {
			*lbyte = (0x0200 - (x*(-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x*(-1))) & 0x100)>>8)<<6) | 0x80;
		}
	}
}

static short ist8801_convdata_2byte_to_short(u8 opf, unsigned char hbyte, unsigned char lbyte)
{
	short x = 0;
	x = (short) ((hbyte<<8) | lbyte);
    if (div == 0)
		div = 8;

	return -1 * x / div;
}

static int ist8801_update_interrupt_threshold(struct device *dev, short raw)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	u8 lthh = 0, lthl = 0, hthh = 0, hthl = 0;
	int err = -1;

	if (p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) {

		dbg("reg.map.intsrs = 0x%02X", p_data->reg.map.intsrs);
		if (p_data->reg.map.intsrs & IST8801_VAL_INTSRS_INTTYPE_WITHIN) {
			/* to do another condition */
		} else {
			dbg("BESIDE raw = %d", raw);
			if ((raw >= -512) && (raw < p_data->thrhigh)) {
				ist8801_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrhigh, &hthh, &hthl);
				ist8801_convdata_short_to_2byte(p_data->reg.map.opf, -512, &lthh, &lthl);
			} else if ((raw >= p_data->thrlow) && (raw <= 511)) {
				ist8801_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
				ist8801_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrlow, &lthh, &lthl);
			}
		}

		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_HTHH, hthh);
		if (err)
			return err;
		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_HTHL, hthl);
		if (err)
			return err;
		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_LTHH, lthh);
		if (err)
			return err;
		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_LTHL, lthl);
		if (err)
			return err;

		dbg("threshold : (0x%02X%02X, 0x%02X%02X)\n", hthh, hthl, lthh, lthl);

		err = ist8801_clear_interrupt(dev);
		if (err)
			return err;
	}

	return err;
}

static int ist8801_set_operation_mode(struct device *dev, int mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	u8 opf = p_data->reg.map.opf;
	u8 state_machine_reset;
	int err = -1;

	switch (mode) {
	case OPERATION_MODE_POWERDOWN:
		if (p_data->irq_enabled) {

			/*disable irq*/
			disable_irq(p_data->irq);
			free_irq(p_data->irq, NULL);
			p_data->irq_enabled = 0;
		}
		/*stand by mode*/
		opf &= 0x00;
		err = ist8801_i2c_set_reg(client, IST8801_REG_OPF, opf);

		/*state machine reset*/
		state_machine_reset = 0x04;
		err = ist8801_i2c_set_reg(client, IST8801_REG_MACHINE_RESET, state_machine_reset);

		/*go to suspend mode*/
		err = ist8801_i2c_set_reg(client, IST8801_REG_ACTR, 0x02);
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_POWERDOWN, suspend mode");
		break;
	case OPERATION_MODE_MEASUREMENT:
		/*get away suspend mode*/
		err = ist8801_i2c_set_reg(client, IST8801_REG_ACTR, 0x00);
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_POWERDOWN, suspend mode");

		usleep_range(5000, 5200);

		/*go to stand by mode*/
		opf &= 0x00;
		err = ist8801_i2c_set_reg(client, IST8801_REG_OPF, opf);

		/*state machine reset*/
		state_machine_reset = 0x04;
		err = ist8801_i2c_set_reg(client, IST8801_REG_MACHINE_RESET, state_machine_reset);

		/*measure mode ODR 80HZ*/
		opf &= 0x00;
		opf |= 0x40;
		err = ist8801_i2c_set_reg(client, IST8801_REG_OPF, opf);

		if (p_data->reg.map.intsrs & IST8801_DETECTION_MODE_INTERRUPT) {
			if (!p_data->irq_enabled) {
				/*enable irq*/
				err = request_irq(p_data->irq, &ist8801_irq_handler, IRQF_TRIGGER_FALLING, IST8801_IRQ_NAME, 0);
				if (err) {
					mxerr(dev, "request_irq was failed");
					return err;
				}
				mxinfo(dev, "request_irq was success");
				enable_irq(p_data->irq);
				p_data->irq_enabled = 1;
			}
		}
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_MEASUREMENT");
		break;
	}

	return err;
}

static int ist8801_set_detection_mode(struct device *dev, u8 mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	u8 data;
	int err = 0;

	if (mode & IST8801_DETECTION_MODE_INTERRUPT) {

		/*config threshold*/
		ist8801_update_interrupt_threshold(dev, p_data->last_data);

		/*write intsrs*/
		data = p_data->reg.map.intsrs | IST8801_DETECTION_MODE_INTERRUPT;
		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_INTSRS, data);
		if (err)
			return err;

	} else {

		/*write intsrs*/
		data = p_data->reg.map.intsrs & (0xFF - IST8801_DETECTION_MODE_INTERRUPT);
		err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_INTSRS, data);
		if (err)
			return err;
	}

	return err;
}


static int ist8801_init_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	int err = -1;
	div = 8;
	/*(1) vdd and vid power up*/
	err = ist8801_set_power(dev, 1);
	if (err) {
		mxerr(&client->dev, "ist8801 power-on was failed (%d)", err);
		return err;
	}

	/*(2) init variables*/
	atomic_set(&p_data->atm.enable, 0);
	atomic_set(&p_data->atm.delay, IST8801_DELAY_MIN);
#ifdef IST8801_DBG_ENABLE
	atomic_set(&p_data->atm.debug, 1);
#else
	atomic_set(&p_data->atm.debug, 0);
#endif
	p_data->calibrated_data = 0;
	p_data->last_data = 0;
	p_data->irq_enabled = 0;
	p_data->irq_first = 1;
	p_data->thrhigh = IST8801_DETECT_RANGE_HIGH;
	p_data->thrlow = IST8801_DETECT_RANGE_LOW;
	ist8801_set_delay(&client->dev, IST8801_DELAY_MAX);
	ist8801_set_debug(&client->dev, 0);

	/*(3) reset registers*/
	err = ist8801_reset_device(dev);
	if (err) {
		mxerr(&client->dev, "ist8801_reset_device was failed (%d)", err);
		return err;
	}

	mxinfo(&client->dev, "initializing device was success");

	return 0;
}

static int ist8801_reset_device(struct device *dev)
{
	int err = 0;
	u8  read_buffer[2];
	u8  id = 0xFF, data = 0x00;

	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	if ((p_data == NULL) || (p_data->client == NULL))
		return -ENODEV;

	/*(0) check if use compass*/
	err = ist8801_i2c_read(p_data->client, AKM_MAG_REG_ADDR_WIA1_099XX, read_buffer, sizeof(read_buffer));
	if (err < 0)
		return err;
	printk(KERN_ERR"ecompass:akm099xx company ID:0x%x, device ID:0x%x\n", read_buffer[0], read_buffer[1]);

	/* Verify Company ID */
	if (read_buffer[0] != AKM_MAG_COMPANY_ID)
		printk(KERN_ERR"ecompass: not akm099xx product, company ID must be 0x48, current:0x%x\n", read_buffer[0]);

	/* Check Device ID */
	if ((read_buffer[1] != AKM_MAG09912_DEVICE_ID) &&
	(read_buffer[1] != AKM_MAG09911_DEVICE_ID) && (read_buffer[1] != AKM_MAG09918_DEVICE_ID)) {
	/* Unknown device. Return with nothing detected */
		printk(KERN_ERR"ecompass: not akm099xx product, device ID must be 0x05(akm09911) or 0x04(akm09912), current:0x%x", read_buffer[1]);
	} else {
		printk(KERN_ERR"ecompass:use compass(low cost plan), device ID:0x%x\n", read_buffer[1]);
		p_ist8801_data->use_e_compass = true;
		goto finish;
	}

	/*get away suspend mode*/
	err = ist8801_i2c_set_reg(client, IST8801_REG_ACTR, 0x00);

	/*(1) sw reset*/
	err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_SRST, IST8801_VAL_SRST_RESET);
	if (err) {
		mxerr(&client->dev, "sw-reset was failed(%d)", err);
		return err;
	}
	msleep(20);
	dbg("wait 20ms after vdd power up");

	/*(2) check id*/
	err = ist8801_i2c_get_reg(p_data->client, IST8801_REG_DID, &id);
	if (err < 0)
		return err;
	if (id != 0x81) {
		mxerr(&client->dev, "current device id(0x%02X) is not IST8801 device id(0x%02X)", id, 0x81);
		return -ENXIO;
	}

	/*(3) init variables*/
	/*(3-1) persint*/
	data = IST8801_PERSISTENCE_COUNT;
	err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_PERSINT, data);
	/*(3-2) intsrs*/
	data = IST8801_DETECTION_MODE | IST8801_SENSITIVITY_TYPE;
	if (data & IST8801_DETECTION_MODE_INTERRUPT) {
		data |= IST8801_INTERRUPT_TYPE;
	}
	err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_INTSRS, data);
	/*(3-3) opf*/
	data = 0x40;
	err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_OPF, data);

	/*set tst mode*/
	data = IST8801_TST_CONNECT_GND;
	err = ist8801_i2c_set_reg(p_data->client, IST8801_REG_TSTCNTL, data);

	/*(4) write variable to register*/
	err = ist8801_set_detection_mode(dev, IST8801_DETECTION_MODE);
	if (err) {
		mxerr(&client->dev, "ist8801_set_detection_mode was failed(%d)", err);
		return err;
	}

	/*(5) set  range&resolution*/
	data = 0x26;
	err = ist8801_i2c_set_reg(p_data->client, 0x0D, data);

	/*(6) set power-down mode*/
	err = ist8801_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);
	if (err) {
		mxerr(&client->dev, "ist8801_set_detection_mode was failed(%d)", err);
		return err;
	}

finish:
	return err;
}

static int ist8801_set_calibration(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	int retrycnt = 10, cnt = 0;

	short raw = 0;
	int err;

	ist8801_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);
	for (cnt = 0; cnt < retrycnt; cnt++) {
		msleep(IST8801_DELAY_FOR_READY);
		err = ist8801_measure(p_data, &raw);
		if (!err) {
			p_data->calibrated_data = raw;
			break;
		}
	}
	if (!ist8801_get_enable(dev))
		ist8801_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);

	return err;
}

static int ist8801_get_calibrated_data(struct device *dev, int *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);

	int err = 0;

	if (p_data == NULL)
		err = -ENODEV;
	else
		*data = p_data->calibrated_data;

	return err;
}

static int ist8801_measure(ist8801_data_t *p_data, short *raw)
{
	struct i2c_client *client = p_data->client;
	int err;
	u8 buf[3];
	int st1_is_ok = 0;

	/*  (1) read data */
	err = ist8801_i2c_read(client, IST8801_REG_ST1, buf, sizeof(buf));
	if (err)
		return err;

	/* (2) check st1 at polling mode */
	if ((buf[0] & 0x01) | (buf[0] & 0x4))
		st1_is_ok = 1;

	if (st1_is_ok) {
		*raw = ist8801_convdata_2byte_to_short(p_data->reg.map.opf, buf[2], buf[1]);
		p_data->last_data = *raw;
	} else {
		/* if data not ready, use last measure result */
		*raw = p_data->last_data;
		mxerr(&client->dev, "st1(0x%02X) is not DRDY", buf[0]);
		err = -1;
	}

	if (ist8801_get_debug(&client->dev)) {
		printk("raw data (%d)\n", *raw);
	}

	return err;
}


static int ist8801_get_result_status(ist8801_data_t *p_data, int raw)
{
	int status;

	if (p_data->thrhigh <= raw) {
		status = IST8801_RESULT_STATUS_B;
	} else if (p_data->thrlow >= raw) {
		status = IST8801_RESULT_STATUS_A;
	} else {
		status = p_data->last_data;
	}

	switch (status) {
	case IST8801_RESULT_STATUS_A:
		dbg("Result is status [A]\n");
		break;
	case IST8801_RESULT_STATUS_B:
		dbg("Result is status [B]\n");
		break;
	}

	return status;
}

/**************************************************
   input device interface
   **************************************************/
static int ist8801_input_dev_init(ist8801_data_t *p_data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev) {
		return -ENOMEM;
	}
	dev->name = IST8801_DRIVER_NAME;
	dev->id.bustype = BUS_I2C;

#if (IST8801_EVENT_TYPE == EV_ABS)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, IST8801_EVENT_TYPE, ABS_MISC);
	input_set_abs_params(dev, IST8801_EVENT_CODE, IST8801_EVENT_DATA_CAPABILITY_MIN, IST8801_EVENT_DATA_CAPABILITY_MAX, 0, 0);
#elif (IST8801_EVENT_TYPE == EV_KEY)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, IST8801_EVENT_TYPE, IST8801_EVENT_CODE);
#else
#error ("[ERR] IST8801_EVENT_TYPE is not defined.")
#endif

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		printk(KERN_ERR "ist8801_input_dev_init register error(%d)\n", err);
		return err;
	}

	p_data->input_dev = dev;

	return 0;
}

static void ist8801_input_dev_terminate(ist8801_data_t *p_data)
{
	struct input_dev *dev = p_data->input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}





/**************************************************
   misc device interface
   **************************************************/

static int ist8801_misc_dev_open(struct inode*, struct file*);
static int ist8801_misc_dev_release(struct inode*, struct file*);
static long ist8801_misc_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t ist8801_misc_dev_read(struct file *filp, char *buf, size_t count, loff_t *ofs);
static ssize_t ist8801_misc_dev_write(struct file *filp, const char *buf, size_t count, loff_t *ofs);
static unsigned int ist8801_misc_dev_poll(struct file *filp, struct poll_table_struct *pwait);

static struct file_operations ist8801_misc_dev_fops = {
	.owner = THIS_MODULE,
	.open = ist8801_misc_dev_open,
	.unlocked_ioctl = ist8801_misc_dev_ioctl,
	.release = ist8801_misc_dev_release,
	.read = ist8801_misc_dev_read,
	.write = ist8801_misc_dev_write,
	.poll = ist8801_misc_dev_poll,
};

static struct miscdevice ist8801_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = IST8801_DRIVER_NAME,
	.fops = &ist8801_misc_dev_fops,
};
/*ist8801 misc device file operation*/
static int ist8801_misc_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ist8801_misc_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}



static long ist8801_misc_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	void __user *argp = (void __user *)arg;
	int kbuf = 0;
	int caldata = 0;

	switch (cmd) {
	case IST8801_IOCTL_SET_ENABLE:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("IST8801_IOCTL_SET_ENABLE(%d)\n", kbuf);
		ist8801_set_enable(&p_ist8801_data->client->dev, kbuf);
		break;
	case IST8801_IOCTL_GET_ENABLE:
		kbuf = ist8801_get_enable(&p_ist8801_data->client->dev);
		dbg("IST8801_IOCTL_GET_ENABLE(%d)\n", kbuf);
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case IST8801_IOCTL_SET_DELAY:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("IST8801_IOCTL_SET_DELAY(%d)\n", kbuf);
		ist8801_set_delay(&p_ist8801_data->client->dev, kbuf);
		break;
	case IST8801_IOCTL_GET_DELAY:
		kbuf = ist8801_get_delay(&p_ist8801_data->client->dev);
		dbg("IST8801_IOCTL_GET_DELAY(%d)\n", kbuf);
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case IST8801_IOCTL_SET_CALIBRATION:
		dbg("IST8801_IOCTL_SET_CALIBRATION\n");
		kbuf = ist8801_set_calibration(&p_ist8801_data->client->dev);
		ret = copy_to_user(argp, &kbuf, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case IST8801_IOCTL_GET_CALIBRATED_DATA:
		dbg("IST8801_IOCTL_GET_CALIBRATED_DATA\n");
		kbuf = ist8801_get_calibrated_data(&p_ist8801_data->client->dev, &caldata);
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		dbg("calibrated data (%d)\n", caldata);
		break;
	case IST8801_IOCTL_SET_REG:
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		dbg("IST8801_IOCTL_SET_REG([0x%02X] %02X", (u8)((kbuf>>8)&0xFF), (u8)(kbuf&0xFF));
		ist8801_set_reg(&p_ist8801_data->client->dev, &kbuf);
		dbgn(" (%s))\n", (kbuf&0xFF00)?"Not Ok":"Ok");
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		break;
	case IST8801_IOCTL_GET_REG:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("IST8801_IOCTL_GET_REG([0x%02X]", (u8)((kbuf>>8)&0xFF));
		ist8801_get_reg(&p_ist8801_data->client->dev, &kbuf);
		dbgn(" 0x%02X (%s))\n", (u8)(kbuf&0xFF), (kbuf&0xFF00)?"Not Ok":"Ok");
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case IST8801_IOCTL_SET_INTERRUPT:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("IST8801_IOCTL_SET_INTERRUPT(%d)\n", kbuf);
		if (kbuf) {
			ist8801_set_detection_mode(&p_ist8801_data->client->dev, IST8801_DETECTION_MODE_INTERRUPT);
		} else {
			ist8801_set_detection_mode(&p_ist8801_data->client->dev, IST8801_DETECTION_MODE_POLLING);
		}
		break;
	case IST8801_IOCTL_GET_INTERRUPT:
		break;
	case IST8801_IOCTL_SET_THRESHOLD_HIGH:
		break;
	case IST8801_IOCTL_GET_THRESHOLD_HIGH:
		break;
	case IST8801_IOCTL_SET_THRESHOLD_LOW:
		break;
	case IST8801_IOCTL_GET_THRESHOLD_LOW:
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static ssize_t ist8801_misc_dev_read(struct file *filp, char *buf, size_t count, loff_t *ofs)
{
	return 0;
}

static ssize_t ist8801_misc_dev_write(struct file *filp, const char *buf, size_t count, loff_t *ofs)
{
	return 0;
}

static unsigned int ist8801_misc_dev_poll(struct file *filp, struct poll_table_struct *pwait)
{
	return 0;
}

/**************************************************
   sysfs attributes
   **************************************************/
static ssize_t ist8801_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", ist8801_get_enable(&p_ist8801_data->client->dev));
}

static ssize_t ist8801_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1)) {
		ist8801_set_enable(&p_ist8801_data->client->dev, enable);
	}

	return count;
}

static ssize_t ist8801_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", ist8801_get_delay(&p_ist8801_data->client->dev));
}

static ssize_t ist8801_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > IST8801_DELAY_MAX) {
		delay = IST8801_DELAY_MAX;
	}

	ist8801_set_delay(&p_ist8801_data->client->dev, delay);

	return count;
}

static ssize_t ist8801_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", ist8801_get_debug(&p_ist8801_data->client->dev));
}

static ssize_t ist8801_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long debug = simple_strtoul(buf, NULL, 10);

	ist8801_set_debug(&p_ist8801_data->client->dev, debug);

	return count;
}

static ssize_t ist8801_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return 0;
}

/* chenkai add for getting rawdata by electronic compass*/
static int mag_get_rawdata(struct device *dev, int16_t *data)
{
	int err = 0;
	u8 akm_write_buffer = 0;
	u8 akm_read_buffer = 0;
	int16_t sae_d[3] = {0, 0, 0};
	u8 rw_buffer[AKM_MAG099XX_NUM_DATA_BYTES] = {0};
	struct i2c_client *client = to_i2c_client(dev);

	/* get cntl */
	err = ist8801_i2c_read(client, AKM_MAG_REG_ADDR_CNTL2_099XX, &akm_read_buffer, sizeof(akm_read_buffer));
	if (err < 0) {
		printk(KERN_ERR "ecompass: rawdata flag read failed 1");
		return err;
	}
	usleep_range(50, 100);

	/*1:switch to single measurement mode*/
	akm_write_buffer = 0x01;
	err = ist8801_i2c_set_reg(client, AKM_MAG_REG_ADDR_CNTL2_099XX, akm_write_buffer);
	printk(KERN_ERR "ecompass err2:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 2");
		return err;
	}
	usleep_range(AKM_MAG09918_MEASUREMENT_TIME_TYP, AKM_MAG09918_MEASUREMENT_TIME_MAX);

	/*2:check DRDY bit*/
	err = ist8801_i2c_get_reg(client, AKM_MAG_REG_ADDR_STATUS1_099XX, &rw_buffer[0]);
	printk(KERN_ERR "ecompass err3:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 3");
		return err;
	}
	usleep_range(50, 100);
	if ((rw_buffer[0] & AKM_MAG_DRDY_BIT_MASK) == 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 4");
	}

	/*3: read data*/
	err = ist8801_i2c_read(client, AKM_MAG_REG_ADDR_STATUS1_099XX, rw_buffer, sizeof(rw_buffer));
	printk(KERN_ERR "ecompass err4:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 5");
		return err;
	}

	/*4: read st2 register and checkout hoft bit*/
	if (rw_buffer[AKM_MAG099XX_NUM_DATA_BYTES - 1] & AKM_MAG_HOFL_BIT_MASK) {
		printk(KERN_ERR "ecompass rawdata overflow!");
		*data = AKM_MAG09918_MAX_VALUE;
		return true;
	}
	printk(KERN_ERR " ecompass AK099xx get_rawdata rw_buffer[1] %d rw_buffer[2] %d rw_buffer[3] %d",
						 rw_buffer[1],
						 rw_buffer[2],
						 rw_buffer[3]);
	printk(KERN_ERR " ecompass AK099xx get_rawdata rw_buffer[4] %d rw_buffer[5] %d rw_buffer[6] %d",
						 rw_buffer[4],
						 rw_buffer[5],
						 rw_buffer[6]);
	sae_d[0] = (int16_t) (((uint16_t)rw_buffer[1]) | ((uint16_t)rw_buffer[2] << 8));
	sae_d[1] = (int16_t) (((uint16_t)rw_buffer[3]) | ((uint16_t)rw_buffer[4] << 8));
	sae_d[2] = (int16_t) (((uint16_t)rw_buffer[5]) | ((uint16_t)rw_buffer[6] << 8));
	/*data[0] = sae_d[0] * 1;
	data[1] = sae_d[1] * 1;
	data[2] = sae_d[2] * 1;*/
	*data = sae_d[2] * 1;

	err = ist8801_i2c_set_reg(client, AKM_MAG_REG_ADDR_CNTL2_099XX, akm_read_buffer);
	printk(KERN_ERR "ecompass err5:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 7");
		return err;
	}
	usleep_range(50, 100);
	return true;
}

/*getting measure data*/
static ssize_t ist8801_data_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	short value = 0;
	if (p_ist8801_data->use_e_compass) {
		value = 0 - p_ist8801_data->z_value/(5*40);
	} else {
		ist8801_measure(p_ist8801_data, &value);
	}
	if (ist8801_get_debug(&p_ist8801_data->client->dev))
		pr_err("ist8801_data_show down data:%d\n", value);
	return snprintf(buf, 10, "%d\n", value);
}

static ssize_t ist8801_dump_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	   int reg = 0;
	   int reg_l = IST8801_REG_HSL;
	   int reg_h = IST8801_REG_HSH;
	   int i = 0;

	   for (i = 0; i < 33; i++) {
		   reg = i<<8;
		   ist8801_get_reg(&p_ist8801_data->client->dev, &reg);
		   printk(KERN_ERR"ist8801_dump_show:-- the reg 0x%02X value: 0x%02X\n", i, reg);
	   }

	   ist8801_get_reg(&p_ist8801_data->client->dev, &reg_l);
	   printk(KERN_ERR"ist8801_dump_show: the reg_l is 0x%02X\n", (u8)(reg_l&0xFF));

	   ist8801_get_reg(&p_ist8801_data->client->dev, &reg_h);
	   printk(KERN_ERR"ist8801_dump_show: the reg_h is 0x%02X", (u8)(reg_h&0xFF));

	   reg = ((reg_h&0xC0) << 2)|reg_l;
	   printk(KERN_ERR"ist8801_dump_show: the up hall reg measure is 0x%02X\n", reg);

	   return snprintf(buf, 10, "%d\n", reg);
	   /* return 0; */
}


/* vivo sensor team chenkai add for lowcost plan start*/
static ssize_t ist8801_plan_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", p_ist8801_data->use_e_compass);
}
/* vivo sensor team chenkai add for lowcost plan end*/


static ssize_t ist8801_regset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret, adj;
	uint16_t reg = 0, val = 0;
	char next[20], *nextp;
	nextp = next;
	reg = simple_strtoul(buf, &nextp, 16);
	pr_err("ist8801_regset_store ==next:%s\n", nextp);

	val = simple_strtoul(nextp + 1, NULL, 16);

	pr_err("ist8801_regset_store buf:%s;\n", buf);
	pr_err("ist8801_regset_store nexp:%s;\n", nextp);
	pr_err("ist8801_regset_store reg:0x%x;  val:0x%x\n", reg, val);
	if (reg == 0x0D) {
		adj = (val >> 1) & 0x0F;
		switch (adj) {
		case 0:
				div = 64;
		break;
		case 1:
				div = 32;
		break;
		case 2:
				div = 16;
		break;
		case 3:
				div = 8;
		break;
		case 4:
				div = 4;
		break;
		case 5:
				div = 2;
		break;
		case 6:
				div = 1;
		break;
		}
		pr_err("ist8801_regset_store set div:%d\n", div);
    }
	ret = ist8801_i2c_set_reg(p_ist8801_data->client, reg, val);
	return count;
}

static DEVICE_ATTR(enable,  S_IRUGO|S_IWUSR|S_IWGRP, ist8801_enable_show, ist8801_enable_store);
static DEVICE_ATTR(delay,   S_IRUGO|S_IWUSR|S_IWGRP, ist8801_delay_show,  ist8801_delay_store);
static DEVICE_ATTR(debug,   S_IRUGO|S_IWUSR|S_IWGRP, ist8801_debug_show,  ist8801_debug_store);
static DEVICE_ATTR(wake,	S_IWUSR|S_IWGRP,		 NULL,			  ist8801_wake_store);
static DEVICE_ATTR(data,	S_IRUGO|S_IWUSR|S_IWGRP, ist8801_data_show,   NULL);
static DEVICE_ATTR(dump,	S_IRUGO|S_IWUSR|S_IWGRP, ist8801_dump_show,   NULL);
static DEVICE_ATTR(plan,    S_IRUGO|S_IWUSR|S_IWGRP, ist8801_plan_show,   NULL);
static DEVICE_ATTR(regset,  S_IRUGO|S_IWUSR|S_IWGRP, NULL,   ist8801_regset_store);

static struct attribute *ist8801_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_debug.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_dump.attr,
	&dev_attr_plan.attr,
	&dev_attr_regset.attr,
	NULL
};

static struct attribute_group ist8801_attribute_group = {
	.attrs = ist8801_attributes
};

static int ist8801_power_ctl(ist8801_data_t *data, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
			  msleep(8);/* >=5ms OK. */
		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		msleep(10); /* wait 10ms */
		data->power_enabled = on;
	} else {
		dev_info(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int ist8801_power_init(ist8801_data_t *data)
{
	int ret;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				IST8801_VDD_MIN_UV,
				IST8801_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->client->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->client->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				IST8801_VIO_MIN_UV,
				IST8801_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->client->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, IST8801_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}


static int ist8801_parse_dt(struct device *dev,
			ist8801_data_t *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	ist8801_data_t *p_data = i2c_get_clientdata(client);


	rc = of_property_read_u32(np, "magnachip,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		if (temp_val < IST8801_DELAY_MIN)
			temp_val = IST8801_DELAY_MIN;
		atomic_set(&p_data->atm.delay, temp_val);
	}

	p_data->int_en = of_property_read_bool(np, "magnachip,use-interrupt");

	p_data->igpio = of_get_named_gpio_flags(dev->of_node,
				"magnachip,gpio-int", 0, NULL);

	p_data->use_hrtimer = of_property_read_bool(np, "magnachip,use-hrtimer");

	return 0;
}

/**************************************************
   i2c client
   **************************************************/

static int ist8801_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ist8801_platform_data_t   *p_platform;
	ist8801_data_t			*p_data;
	int					 err = 0;
	struct vivo_hall_dev *hall_dev;

	dbg_func_in();

	/*(1) allocation memory for p_ist8801_data*/
	p_data = kzalloc(sizeof(ist8801_data_t), GFP_KERNEL);
	if (!p_data) {
		mxerr(&client->dev, "kernel memory alocation was failed");
		err = -ENOMEM;
		goto error_0;
	}

	/*(2) init mutex variable*/
	mutex_init(&p_data->mtx.enable);
	mutex_init(&p_data->mtx.data);
	p_data->power_enabled = false;

	/*(3) config i2c client*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		mxerr(&client->dev, "i2c_check_functionality was failed");
		err = -ENODEV;
		goto error_1;
	}
	i2c_set_clientdata(client, p_data);
	p_data->client = client;
	p_ist8801_data = p_data;

	if (client->dev.of_node) {
		dev_err(&client->dev, "Use client->dev.of_node\n");
		err = ist8801_parse_dt(&client->dev, p_data);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto error_1;
		}
	} else {
		p_platform = client->dev.platform_data;
		dev_err(&client->dev, "Use platform data\n");
	}
#if 0
	/* replaced by using ist8801_parse_dt() dts tree... */
	/*(4) get platform data*/
	p_platform = client->dev.platform_data;
	if (p_platform) {
		p_data->power_vi2c	  = p_platform->power_vi2c;
		p_data->power_vdd	   = p_platform->power_vdd;
		p_data->igpio		   = p_platform->interrupt_gpio;
		p_data->irq			 = p_platform->interrupt_irq;
	} else {
		p_data->power_vi2c = -1;
		p_data->power_vdd = -1;
		p_data->igpio = -1;
	}
#endif
	/*(5) setup interrupt gpio*/
	/*if (p_data->igpio != -1) {
		err = gpio_request(p_data->igpio, "ist8801_irq");
		if (err) {
			mxerr(&client->dev, "gpio_request was failed(%d)", err);
			goto error_1;
		}
		mxinfo(&client->dev, "gpio_request was success");
		err = gpio_direction_input(p_data->igpio);
		if (err < 0) {
			mxerr(&client->dev, "gpio_direction_input was failed(%d)", err);
			goto error_2;
		}
		mxinfo(&client->dev, "gpio_direction_input was success");
	}*/

	err = ist8801_power_init(p_data);
	if (err) {
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto error_1;
	}
	err = ist8801_power_ctl(p_data, true);
	if (err) {
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto error_1;
	}


	/*(6) reset and init device*/
	err = ist8801_init_device(&p_data->client->dev);
	if (err) {
		mxerr(&client->dev, "ist8801_init_device was failed(%d)", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was found", id->name);

	/*(7) config work function*/
	if (p_data->use_e_compass) {
		INIT_DELAYED_WORK(&p_data->work, ak09918_work_func);
	} else {
		INIT_DELAYED_WORK(&p_data->work, ist8801_work_func);
	}

	/*(8) init input device*/
	err = ist8801_input_dev_init(p_data);
	if (err) {
		mxerr(&client->dev, "ist8801_input_dev_init was failed(%d)", err);
		/* goto error_1; */
	} else {
		mxerr(&client->dev, "ist8801_input_dev_init (%d)", err);
	}
	/* TODO DONOT CARE INPUT DEVICE*/
	hall_dev = &(p_data->hall_dev);
	hall_dev->name = UP_HOLL_NAME;
	err = vivo_hall_dev_register(hall_dev);
	if (err < 0) {
		pr_err("vivo hall registration failed (%d)\n", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was initialized", IST8801_DRIVER_NAME);

	/*(9) create sysfs group*/
	err = sysfs_create_group(&hall_dev->dev->kobj, &ist8801_attribute_group);
	if (err) {
		mxerr(&client->dev, "sysfs_create_group was failed(%d)", err);
		goto error_3;
	}

	/*(10) register misc device*/
	err = misc_register(&ist8801_misc_dev);
	if (err) {
		mxerr(&client->dev, "misc_register was failed(%d)", err);
		goto error_4;
	}

	/*(11) imigrate p_data to p_ist8801_data*/
	dbg("%s : %s was probed.\n", __func__, IST8801_DRIVER_NAME);

	return 0;

error_4:
	sysfs_remove_group(&hall_dev->dev->kobj, &ist8801_attribute_group);

error_3:
	ist8801_input_dev_terminate(p_data);

error_1:
	kfree(p_data);

error_0:
	p_ist8801_data = NULL;
	return err;
}

static int ist8801_i2c_drv_remove(struct i2c_client *client)
{
	ist8801_data_t *p_data = i2c_get_clientdata(client);
	struct vivo_hall_dev *hall_dev = &(p_data->hall_dev);

	ist8801_set_enable(&client->dev, 0);
	misc_deregister(&ist8801_misc_dev);
	sysfs_remove_group(&hall_dev->dev->kobj, &ist8801_attribute_group);
	ist8801_input_dev_terminate(p_data);
	if (p_data->igpio != -1) {
		gpio_free(p_data->igpio);
	}
	kfree(p_data);

	return 0;
}

static const struct i2c_device_id ist8801_i2c_drv_id_table[] = {
	{IST8801_DRIVER_NAME, 0 },
	{ }
};


static const struct of_device_id ist8801_of_match[] = {
	{ .compatible = "isentek,ist8801,up", },
	{ },
};

static struct i2c_driver ist8801_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = IST8801_DRIVER_NAME,
		.of_match_table = ist8801_of_match,
	},
	.probe	  = ist8801_i2c_drv_probe,
	.remove	 = ist8801_i2c_drv_remove,
	.id_table   = ist8801_i2c_drv_id_table,
	/* .suspend  = ist8801_i2c_drv_suspend, */
	/* .resume	   = ist8801_i2c_drv_resume, */
};

static int __init ist8801_driver_init_up(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&ist8801_driver);
}
module_init(ist8801_driver_init_up);

static void __exit ist8801_driver_exit_up(void)
{
	printk(KERN_INFO "%s\n", __func__);
	i2c_del_driver(&ist8801_driver);
}
module_exit(ist8801_driver_exit_up);

MODULE_AUTHOR("vivo sensor <goodpeople@programer.com>");
MODULE_VERSION(IST8801_DRIVER_VERSION);
MODULE_DESCRIPTION("IST8801 hallswitch driver");
MODULE_LICENSE("GPL");

