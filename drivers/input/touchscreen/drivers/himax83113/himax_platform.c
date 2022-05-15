/*  Himax Android Driver Sample Code for QCT platform

    Copyright (C) 2018 Himax Corporation.

    This software is licensed under the terms of the GNU General Public
    License version 2, as published by the Free Software Foundation, and
    may be copied, distributed, and modified under those terms.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#include "himax_platform.h"
#include "himax_common.h"
//#include "firmware/PD1813B_himax_fw.h"
//#include "firmware/PD1813B_himax_mp_fw.h"


#define PACKET_MAX_SZ 254
int xfer_error_count;
//int irq_enable_count;
//int tpd_flag;
//DECLARE_WAIT_QUEUE_HEAD(waiter);
struct spi_device *spi;
static uint8_t *gBuffer;

extern bool g_DSRAM_Flag;//27
extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern struct himax_platform_data *hv_pdata;
extern uint8_t *event_buf;


extern void himax_ts_work(struct himax_ts_data *ts, ktime_t kt);
extern enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);

extern int himax_chip_common_init(void);
extern void himax_chip_common_deinit(void);

int himax_dev_set(struct himax_ts_data *ts)
{
	int ret = 0;
	ts->input_dev = input_allocate_device();

	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		E("%s: Failed to allocate input device\n", __func__);
		return ret;
	}

	ts->input_dev->name = "himax-touchscreen";
	return ret;
}
int himax_input_register_device(struct input_dev *input_dev)
{
	return input_register_device(input_dev);
}

int himax_parse_dt(struct device_node *np,struct himax_platform_data *pdata)
{
	int rc, coords_size = 0;
	uint32_t coords[4] = {0};
	struct property *prop;
	u32 data = 0;

	prop = of_find_property(np, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);

		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d\n", __func__, coords_size);
	}

	if (of_property_read_u32_array(np, "himax,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = (coords[1] - 1);
		pdata->abs_y_min = coords[2], pdata->abs_y_max = (coords[3] - 1);
		I(" DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
		  pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}

	prop = of_find_property(np, "himax,display-coords", NULL);

	if (prop) {
		coords_size = prop->length / sizeof(u32);

		if (coords_size != 4)
			D(" %s:Invalid display coords size %d\n", __func__, coords_size);
	}

	rc = of_property_read_u32_array(np, "himax,display-coords", coords, coords_size);

	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}

	pdata->screenWidth  = coords[1];
	pdata->screenHeight = coords[3];
	I(" DT-%s:display-coords = (%d, %d)\n", __func__, pdata->screenWidth,
	  pdata->screenHeight);

	if(of_property_read_u32(np, "himax,tx-sensors", &data) == 0) {
		pdata->tx = data;
		I(" DT:TX = %d\n", pdata->tx);
	}

	if(of_property_read_u32(np, "himax,rx-sensors", &data) == 0) {
		pdata->rx = data;
		I(" DT:TX = %d\n", pdata->rx);
	}
	
	pdata->gpio_irq = of_get_named_gpio(np, "himax,irq-gpio", 0);

	if (!gpio_is_valid(pdata->gpio_irq)) {
		I(" DT:gpio_irq value is not valid\n");
	}

	pdata->gpio_reset = of_get_named_gpio(np, "himax,rst-gpio", 0);

	if (!gpio_is_valid(pdata->gpio_reset)) {
		I(" DT:gpio_rst value is not valid\n");
	}

	pdata->gpio_cs = of_get_named_gpio(np, "himax,cs-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_cs)) {
		I(" DT:gpio_cs value is not valid\n");
	}

	I(" DT:gpio_irq=%d, gpio_rst=%d\n", pdata->gpio_irq, pdata->gpio_reset);

	if (of_property_read_u32(np, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		I(" DT:protocol_type=%d\n", pdata->protocol_type);
	}
	rc = of_property_read_u32(np, "spi-max-frequency", &pdata->spi_frequency);
	if (rc || pdata->spi_frequency <= 0 || pdata->spi_frequency > 9600000) {
		pdata->spi_frequency = 6000000;
		VTE("error reading spi frequency, use default frequency");	
	} 
	VTI("spi frequency:%d\n", pdata->spi_frequency);
	return 0;
}

static ssize_t himax_spi_sync(struct himax_ts_data *ts, struct spi_message *message)
{
	int status;

	status = spi_sync(ts->spi, message);

	if (status == 0) {
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static int himax_spi_read(uint8_t *command, uint8_t command_len, uint8_t *data, uint32_t length, uint8_t toRetry)
{
	struct spi_message message;
	struct spi_transfer xfer[2];
	int retry = 0;
	int error = -1;

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));

	xfer[0].tx_buf = command;
	xfer[0].len = command_len;
	spi_message_add_tail(&xfer[0], &message);

	xfer[1].tx_buf = data;
	xfer[1].rx_buf = data;
	xfer[1].len = length;
	spi_message_add_tail(&xfer[1], &message);

	for (retry = 0; retry < toRetry; retry++) {
		error = spi_sync(private_ts->spi, &message);
		if (unlikely(error))
			E("SPI read error: %d\n", error);
		else
			break;

	}

	if (retry == toRetry) {
		E("%s: SPI read error retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}

	return 0;
}

static int himax_spi_write(uint8_t *buf, uint32_t length)
{

	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= length,
	};
	struct spi_message	m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return himax_spi_sync(private_ts, &m);

}

int himax_bus_read(uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry)
{
	int result = 0;
	uint8_t spi_format_buf[3];

	mutex_lock(&(private_ts->spi_lock));
	spi_format_buf[0] = 0xF3;
	spi_format_buf[1] = command;
	spi_format_buf[2] = 0x00;

	result = himax_spi_read(&spi_format_buf[0], 3, data, length, toRetry);
	mutex_unlock(&(private_ts->spi_lock));

	return result;
}

int himax_bus_write(uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry)
{
	int i = 0;
	int result = 0;

	uint8_t *spi_format_buf = gBuffer;


	mutex_lock(&(private_ts->spi_lock));
	spi_format_buf[0] = 0xF2;
	spi_format_buf[1] = command;

	for (i = 0; i < length; i++)
		spi_format_buf[i + 2] = data[i];

	result = himax_spi_write(spi_format_buf, length + 2);
	mutex_unlock(&(private_ts->spi_lock));

	return result;
}

int himax_bus_write_command(uint8_t command, uint8_t toRetry)
{
	return himax_bus_write(command, NULL, 0, toRetry);
}

int himax_bus_master_write(uint8_t *data, uint32_t length, uint8_t toRetry)
{
	uint8_t *buf = NULL;

	struct spi_transfer	t = {
		.tx_buf	= buf,
		.len	= length,
	};
	struct spi_message	m;
	int result = 0;

	buf = kzalloc(sizeof(uint8_t) * length, GFP_KERNEL);
	if (!buf)
		return 0;
	mutex_lock(&(private_ts->spi_lock));
	memcpy(buf, data, length);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	result = himax_spi_sync(private_ts, &m);
	mutex_unlock(&(private_ts->spi_lock));

	kfree(buf);
	return result;
}

void himax_int_enable(int enable)
{
	int irqnum = 0;
	irqnum = private_ts->hx_irq;

	if (enable == 1 && private_ts->irq_enabled == 0) {
		I("%s:enable irq\n", __func__);
		enable_irq(irqnum);
		//irq_enable_count = 1;
		private_ts->irq_enabled = 1;
	} else if (enable == 0 && private_ts->irq_enabled == 1) {
		I("%s:disable irq\n", __func__);
		disable_irq_nosync(irqnum);
		//irq_enable_count = 0;
		private_ts->irq_enabled = 0;
	}

	VTD("irq_enable_count = %d\n", private_ts->irq_enabled);
}

#ifdef HX_RST_PIN_FUNC
void himax_rst_gpio_set(int pinnum, uint8_t value)
{
	gpio_direction_output(pinnum, value);
}
#endif

uint8_t himax_int_gpio_read(int pinnum)
{
	return gpio_get_value(pinnum);
}

int himax_gpio_power_config(struct himax_platform_data *pdata)
{
	int error = 0;

#ifdef HX_RST_PIN_FUNC

	if (pdata->gpio_reset >= 0) {
		error = gpio_request(pdata->gpio_reset, "himax-reset");

		if (error < 0) {
			E("%s: request reset pin failed\n", __func__);
			return error;
		}

		error = gpio_direction_output(pdata->gpio_reset, 0);

		if (error) {
			E("unable to set direction for gpio [%d]\n",
			  pdata->gpio_reset);
			return error;
		}
	}

#endif	

#if defined(CONFIG_MEDIATEK_SOLUTION)
	if (private_ts->cs_bootup) {
		if (!IS_ERR_OR_NULL(private_ts->spi_cs_active)) {
			pinctrl_select_state(private_ts->pinctrl, private_ts->spi_cs_active);
			I("spi_cs_active select");
		}
	}
#endif

	if (gpio_is_valid(pdata->gpio_irq)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_irq, "himax,irq-gpio");

		if (error) {
			E("unable to request gpio [%d]\n", pdata->gpio_irq);
			return error;
		}

		error = gpio_direction_input(pdata->gpio_irq);

		if (error) {
			E("unable to set direction for gpio [%d]\n", pdata->gpio_irq);
			return error;
		}

		private_ts->hx_irq = gpio_to_irq(pdata->gpio_irq);
		I("set irq gpio success");
	} else {
		E("irq gpio not provided\n");
		return error;
	}

	msleep(20);
#ifdef HX_RST_PIN_FUNC
	if (pdata->gpio_reset >= 0) {
		error = gpio_direction_output(pdata->gpio_reset, 1);

		if (error) {
			E("unable to set direction for gpio [%d]\n",
			  pdata->gpio_reset);
			return error;
		}
	}

#endif
	return error;
}
static  irqreturn_t  himax_ts_isr_func(int irq, void *handle, ktime_t kt)
{
	if (g_DSRAM_Flag == true) {
		I("%s:g_DSRAM_Flag = %d\n", __func__, g_DSRAM_Flag);//28
		return IRQ_HANDLED;
	}
	//himax_int_enable(0);
	himax_ts_work(private_ts, kt);
	//himax_int_enable(1);
	return IRQ_HANDLED;
}
#if 0
irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	/* SRAM raw data report no need to parse touch data*/
	if (g_DSRAM_Flag == true) {
		I("%s:g_DSRAM_Flag = %d\n", __func__, g_DSRAM_Flag);//28
		return IRQ_HANDLED;
	}
	//tpd_flag = 1;
	VTD("disable_irq_nosync");
	himax_int_enable(0);
	//wake_up_interruptible(&waiter);
	vtsIrqBtmThreadWake();

	return IRQ_HANDLED;
}
#endif
static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	himax_ts_work(ts, ktime_set(0, 0));
}

/*int hx_ts_work_handler(void *unused)
{
	do {
	wait_event_interruptible(waiter,tpd_flag!=0);
	tpd_flag = 0;

	himax_ts_work(private_ts);
	himax_int_enable(1);
	} while (!kthread_should_stop());

	return 0;
}*/

int himax_int_register_trigger(void)
{
	int ret = 0;
	struct himax_ts_data *ts = private_ts;

	if (ic_data->HX_INT_IS_EDGE) {
		VTI("%s edge triiger falling\n ", __func__);
		ret = vts_interrupt_register(ts->vtsdev, ts->hx_irq , himax_ts_isr_func, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ts);
		//sret = request_threaded_irq(ts->hx_irq, NULL, himax_ts_thread, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, HIMAX_COMMON_NAME, ts);
	} else {
		VTI("%s level trigger low\n ", __func__);
		ret = vts_interrupt_register(ts->vtsdev, ts->hx_irq , himax_ts_isr_func, IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts);
		//ret = request_threaded_irq(ts->hx_irq, NULL, himax_ts_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT, HIMAX_COMMON_NAME, ts);
	}
	if(ret==0)
	VTI("%s   success\n",__func__);
	return ret;
}

int himax_int_en_set(void)
{
	int ret = NO_ERR;
	ret = himax_int_register_trigger();
	return ret;
}

int himax_ts_register_interrupt(void)
{
	struct himax_ts_data *ts = private_ts;
	int ret = 0;
	ts->irq_enabled = 0;
	/* Work functon */
	if (private_ts->hx_irq) {/*INT mode*/
		ts->use_irq = 1;
		ret = himax_int_register_trigger();

		if (ret == 0) {
			ts->irq_enabled = 1;
			//irq_enable_count = 1;
			I("%s: irq enabled at qpio: %d\n", __func__, private_ts->hx_irq);
#ifdef HX_SMART_WAKEUP
			irq_set_irq_wake(private_ts->hx_irq, 1);
#endif
		} else {
			ts->use_irq = 0;
			E("%s: request_irq failed\n", __func__);
		}
	} else {
		I("%s: private_ts->hx_irq is empty, use polling mode.\n", __func__);
	}

	if (!ts->use_irq) {/*if use polling mode need to disable HX_ESD_RECOVERY function*/
		ts->himax_wq = create_singlethread_workqueue("himax_touch");
		INIT_WORK(&ts->work, himax_ts_work_func);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}

	return ret;
}
/*
static int himax_common_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	I("%s: enter\n", __func__);
	himax_chip_common_suspend(ts);
	return 0;
}
*/
/*
static int himax_common_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	I("%s: enter\n", __func__);
	himax_chip_common_resume(ts);
	return 0;
}
*/
extern unsigned int mdss_report_lcm_id(void);
/*
static int bbk_xxx_get_lcm_id(void)
{
	int id = 0;
	id = mdss_report_lcm_id();
	if (id == 0x18 || id == 0xff) {
		id = 0x18;
	}
	return id;
}

static int bbk_xxx_module_id(void)
{
	return VTS_MVC_BOE;
}
*/
/*
#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
							unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts =
	    container_of(self, struct himax_ts_data, fb_notif);
	I(" %s\n", __func__);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts != NULL &&
	    ts->dev != NULL) {
		blank = evdata->data;

		switch (*blank) {
		case FB_BLANK_UNBLANK:
			himax_common_resume(ts->dev);
			break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			himax_common_suspend(ts->dev);
			break;
		}
	}

	return 0;
}
#endif
*/
static int bbk_himax_common_init(struct vts_device *vtsdev)
{
	int ret;
	ret = himax_chip_common_init();
	if (ret < 0) {
		VTI("Failed to himax_chip_common_init");
		return -ENOMEM;
	}
	return 0;
}
static int bbk_himax_common_exit(struct vts_device *vtsdev)
{
	himax_chip_common_deinit();
	return 0;
}
extern int bbk_himax_get_fw_version(struct vts_device *vtsdev, u64 *version);
extern int bbk_himax_mode_change(struct vts_device *vtsdev, int which);
extern int bbk_himax_get_rawordiff_data(struct vts_device *vtsdev, enum vts_frame_type which, short *data, int size);
extern int bbk_himax_gesture_point_get(uint16_t *data);
extern int bbk_himax_set_charger_bit(struct vts_device *vtsdev,int state);
extern int idle_enable_or_disable(struct vts_device *vtsdev,int state);
extern int set_edge_restain_switch(struct vts_device *vtsdev,int on);
extern int himax_common_proc_init(void);
extern int himax_touch_proc_init(void);
extern void himax_common_proc_deinit(void);
extern void himax_touch_proc_deinit(void);
extern int bbk_himax_set_gesture(struct vts_device *vtsdev, int enable);
extern int bbk_himax_set_idle(struct vts_device * vtsdev, int enable);
extern int bbk_himax_get_touch_ic_mdoe(struct vts_device * vtsdev);

static const struct vts_operations hvt_vts_ops = {
	.init = bbk_himax_common_init,
	.exit = bbk_himax_common_exit,
	.change_mode = bbk_himax_mode_change,
	.get_frame = bbk_himax_get_rawordiff_data,
	.get_fw_version = bbk_himax_get_fw_version,
	.set_charging = bbk_himax_set_charger_bit,
	.set_rotation = set_edge_restain_switch,
	//.set_auto_idle = idle_enable_or_disable,
	.update_firmware = bbk_himax_fw_update,
	.set_gesture = bbk_himax_set_gesture,
	.set_auto_idle = bbk_himax_set_idle,
	.get_ic_mode = bbk_himax_get_touch_ic_mdoe,
};


int himax_chip_common_probe(struct spi_device *spi, struct device_node *np)
{
	struct himax_ts_data *ts;

	int ret = 0;
	struct vts_device *vtsdev = NULL;

	I("Enter %s\n", __func__);
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
        I("Full duplex not supported by master\n");
        ret = -EIO;
        return ret;
    }
	VTI("%s start\n",__func__);

/*
if (1) {
	himax_spi_pinctrl = devm_pinctrl_get(&(spi->dev));
	if (IS_ERR(himax_spi_pinctrl)) {
		ret = PTR_ERR(himax_spi_pinctrl);
		E("Can not found touch spi pinctrl\n");
	}
	//spi_all = pinctrl_lookup_state(himax_spi_pinctrl, "state_spi_all");
	spi_all = pinctrl_lookup_state(himax_spi_pinctrl, "default");
	if (IS_ERR(spi_all)) {
		ret = PTR_ERR(spi_all);
		E("Can not found state_spi_all %d\n", ret);
	} else {
		ret = pinctrl_select_state(himax_spi_pinctrl, spi_all);
		I("%s: state_spi_all select ret = %d\n", __func__, ret);
	}
}
*/
	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	gBuffer = kzalloc(sizeof(uint8_t) * (HX_SPI_MTK_MAX_WRITE_SZ + 2), GFP_KERNEL);
	if (gBuffer == NULL) {
		E("%s: allocate gBuffer failed\n", __func__);
		ret = -ENOMEM;
		goto errcode1;
	}
	hv_pdata = kzalloc(sizeof(*hv_pdata), GFP_KERNEL);
	if (hv_pdata == NULL) { 
		ret = -ENOMEM;
		goto errcode2;
	}
	
	if (himax_parse_dt(np, hv_pdata) < 0) {
		VTI(" pdata is NULL for DT");
		goto errcode3;
	}

	ts->pdata=hv_pdata;

	spi->bits_per_word = 8;
	//spi->mode = SPI_MODE_3;
	spi->chip_select = 0;
	spi->max_speed_hz = ts->pdata->spi_frequency;
	ts->node=np;
	ts->spi = spi;
	mutex_init(&(ts->spi_lock));
	mutex_init(&(ts->rw_lock));
	mutex_init(&(ts->w_fw_lock));
	ts->dev = &spi->dev;
	dev_set_drvdata(&spi->dev, ts);
	//spi_set_drvdata(spi, ts);

	ret = spi_setup(ts->spi);
	if (ret < 0) {
		VTI("Failed to perform SPI setup");
		goto errcode4;
	}
	private_ts = ts;
	private_ts->dev->of_node = np;
	private_ts->pinctrl = devm_pinctrl_get(private_ts->dev);
	if (IS_ERR_OR_NULL(private_ts->pinctrl)) {
		E("Can not found touch spi pinctrl\n");
	} else {
		I("find pinctrl");
		private_ts->spi_cs_active = pinctrl_lookup_state(private_ts->pinctrl, "spi_cs_set");
		if (IS_ERR_OR_NULL(private_ts->spi_cs_active)) {
			E("Can not found spi_cs_active");
		}
		private_ts->spi_cs_sleep_pulllow = pinctrl_lookup_state(private_ts->pinctrl, "spi_cs_pulllow");
		if (IS_ERR_OR_NULL(private_ts->spi_cs_sleep_pulllow)) {
			E("Can not found spi_cs_pulllow");
		}
	}

#if defined(CONFIG_MEDIATEK_SOLUTION)
	private_ts->vddi_poweroff = of_property_read_bool(np, "mtk,vts-vddi-poweroff");
	if (private_ts->vddi_poweroff) {
		I("vddi is poweroff in sleep mode");
	} else {
		I("vddi no poweroff in sleep mode");
	}
	private_ts->cs_bootup = of_property_read_bool(np, "mtk,vts-cs-bootup");
	if (private_ts->cs_bootup) {
		I("cs is pull-down in boot loader");
	} else {
		I("cs no pull-down in boot loader");
	}
#endif
	ret = himax_common_proc_init();
	if (ret<0) {
		E(" %s: himax_common proc_init failed!\n", __func__);
		goto errcode4;
	}
	ret = himax_touch_proc_init();
	if (ret<0){
		E(" %s: debug initial failed!\n", __func__);
		goto errcode5;
	}
	vtsdev = vts_device_alloc();
	if(!vtsdev) {
		VTI("vivoTsAlloc fail");
		ret = -ENOMEM;
		goto errcode6;
	}
	
	vtsdev->ops = &hvt_vts_ops;
	vtsdev->busType = BUS_SPI;
	ts->vtsdev = vtsdev;
	ret = vts_parse_dt_property(vtsdev, np);
	if (ret == -EPROBE_DEFER) {
		VTE("parse_dt_property vts-incell-panel error");
		goto errcode7;
	}
	vts_set_drvdata(vtsdev, ts);
	ret = vts_register_driver(vtsdev);
	if(ret < 0) {		
		VTE("vts_register_driver failed");
		goto errcode7;
	}
	ret = 0;
	goto normalcode;
	
errcode7:
	vts_device_free(vtsdev);
errcode6:
	himax_touch_proc_deinit();
errcode5:
	himax_common_proc_deinit();
errcode4:
	mutex_destroy(&(ts->spi_lock));
	mutex_destroy(&(ts->w_fw_lock));
	spi_set_drvdata(spi, NULL);
errcode3:
	kfree(hv_pdata);
	hv_pdata = NULL;
errcode2:
	kfree(gBuffer);
	gBuffer = NULL;
errcode1:
	kfree(ts);
normalcode:
	return ret;

}

int himax_chip_common_remove(struct spi_device *spi,struct device_node *np)
{
	struct himax_ts_data *ts = spi_get_drvdata(spi);
	struct vts_device *vtsdev = ts->vtsdev;

	vts_unregister_driver(vtsdev);
	vts_device_free(vtsdev);
	himax_touch_proc_deinit();
	himax_common_proc_deinit();
	ts->spi = NULL;
	/* spin_unlock_irq(&ts->spi_lock); */
	spi_set_drvdata(spi, NULL);
	kfree(hv_pdata);
	hv_pdata = NULL;
	mutex_destroy(&(ts->spi_lock));
	mutex_destroy(&(ts->w_fw_lock));
	kfree(gBuffer);
	gBuffer = NULL;
	kfree(event_buf);
	event_buf = NULL;
	kfree(ts);

	return 0;
}

static void himax_chip_shutdown(struct spi_device *spi)
{
	struct himax_platform_data *pdata=hv_pdata;
	VTI("himax chip shutdown!");
	if (pdata) {
		if (gpio_is_valid(pdata->gpio_cs)) {
			gpio_set_value(pdata->gpio_cs, 0);
		} else {
			VTE("cs gpio has been freed !!!");
		}
		if (gpio_is_valid(pdata->gpio_reset)) {
			gpio_set_value(pdata->gpio_reset, 0);
		} else {
			VTE("tp reset gpio has been freed !!!");
		}
	} else {
		VTE("hv_pdata has been freed !!!");
	}

	vts_dsi_panel_reset_power_ctrl(8);
}


static struct vts_spi_driver hvt_spi_driver = {
	.probe		= himax_chip_common_probe,
	.remove		= himax_chip_common_remove,
	.compatible = "himax,hx83112-spi-v2",
	.shutdown = himax_chip_shutdown,
};

static unsigned long hvt_spi_flags;
enum hvt_flags {
	HVT_SPI_INIT = BIT(0),
	HVT83112_INIT = BIT(1)
};

int hvt_spi_init(void)
{
	if (himax_hx83112_init())
		return -1;

	hvt_spi_flags |= HVT83112_INIT;
	if (vts_spi_drv_reigster(&hvt_spi_driver))
		return -1;

	hvt_spi_flags |= HVT_SPI_INIT;

	return 0;
}

void hvt_spi_exit(void)
{
	if (hvt_spi_flags & HVT83112_INIT)
		vts_spi_drv_unreigster(&hvt_spi_driver);

	if (hvt_spi_flags & HVT_SPI_INIT)
		himax_hx83112_exit();
}

static const int ic_number[1] = {VTS_IC_HIMAX83112};
module_vts_driver(hvt_no_flash, ic_number, hvt_spi_init(), hvt_spi_exit());

