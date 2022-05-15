#include <linux/vivo_ts_function.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/kthread.h>
#include <linux/delay.h>

struct algo_prox_ts_info {
	/*daixiang add for algo-prox start*/
	struct i2c_client *client;
	struct sensors_classdev cdev;
	struct input_dev *input_dev;
	struct delayed_work input_work;
	int nr_remain_report_times;
	spinlock_t lock;
	int values;
	/*daixiang add for algo-prox end*/
	int earDetecting;  /*earDetect algorithm running flag*/
	int touchEnable;  /*if lcd shutoff, touchEnable should be 0 */
};


static u8 baseline_flag;
static struct algo_prox_ts_info prox_ts_info;

#define ALGO_PROX_INPUT_DEV_NAME	"algo-prox"
#define ALGO_FREQ 100
#define VTS_PROXIMITY_TRIGGER_TIME 5
#define VTS_PROXIMITY_TRIGGER_MS 500


static struct sensors_classdev algo_prox_cdev = {
	.name = "algo-prox",
	.vendor = "vivo",
	.version = 1,
	.handle = SENSOR_TYPE_SMARTPROX_TP_HANDLE,
	.type = SENSOR_TYPE_SMARTPROX_TP,
	.max_range = "78.4",
	.resolution = "0.01",
	.sensor_power = "0.01",
	.min_delay = 5000,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


/*daixiang add for algo-prox start*/
/*algo-prox run with 10hz*/
static void algo_prox_input_work_func(struct work_struct *work)
{
	work = NULL;
	return ;

}

static int algo_prox_input_open(struct input_dev *input)
{
	/*struct fts_ts_info *ts_info = input_get_drvdata(input);*/

	return 0;
}

static void algo_prox_input_close(struct input_dev *dev)
{
	/*struct fts_ts_info *ts_info = input_get_drvdata(dev);

	algo_prox_disable(ts_info);*/
	/*do nothing*/
}

static int algo_prox_input_init(struct algo_prox_ts_info *ts_info)
{
	int err;

	INIT_DELAYED_WORK(&ts_info->input_work, algo_prox_input_work_func);
	ts_info->input_dev = input_allocate_device();
	if (!ts_info->input_dev) {
		err = -ENOMEM;
		dev_err(&ts_info->client->dev, "input device allocation failed\n");
		goto err0;
	}

	ts_info->input_dev->open = algo_prox_input_open;
	ts_info->input_dev->close = algo_prox_input_close;
	ts_info->input_dev->name = ALGO_PROX_INPUT_DEV_NAME;

	ts_info->input_dev->id.bustype = BUS_I2C;
	ts_info->input_dev->dev.parent = &ts_info->client->dev;

	input_set_drvdata(ts_info->input_dev, ts_info);

#if 0
	set_bit(EV_ABS, ts_info->input_dev->evbit);
	//set_bit(EV_REP, ts_info->input_dev->evbit);/*daixiang add for report repeat data*/
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, ts_info->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, ts_info->input_dev->absbit);

	input_set_abs_params(ts_info->input_dev, ABS_X, -10, 10, 0, 0);
	input_set_abs_params(ts_info->input_dev, ABS_Y, -10, 10, 0, 0);
	input_set_abs_params(ts_info->input_dev, ABS_Z, -10, 10, 0, 0);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(ts_info->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(ts_info->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);
#endif

	 set_bit(EV_REL, ts_info->input_dev->evbit);
	 set_bit(EV_SYN, ts_info->input_dev->evbit);
	 input_set_capability(ts_info->input_dev, EV_REL, REL_Z);
	 input_set_abs_params(ts_info->input_dev, EV_REL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(ts_info->input_dev);
	if (err) {
		dev_err(&ts_info->client->dev,
				"unable to register input device %s\n",
				ts_info->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(ts_info->input_dev);
err0:
	return err;
}

/*static void algo_prox_input_cleanup(struct algo_prox_ts_info *ts_info)
{
	input_unregister_device(algo_prox_ts_info->input_dev);
	input_free_device(algo_prox_ts_info->input_dev);
}*/
static int algo_prox_enable_set(struct sensors_classdev *sensors_cdev,
	unsigned int enable)
{
	struct algo_prox_ts_info *ts_info;
	ts_info = container_of((struct sensors_classdev *)sensors_cdev, struct algo_prox_ts_info, cdev);
	/*ts_info = global_info;*/
	VTI("algo-prox enable_set %d", enable);
	if (!enable) {
		cancel_delayed_work(&ts_info->input_work);
	}
	if (atomic_read(&(vivoTsGetVtsData()->tsState)) != TOUCHSCREEN_NORMAL || vivoTsGetVtsData()->lcdRtState != 1) {
		ts_info->earDetecting = enable;
		return ts_info->earDetecting;
	}
	ts_info->earDetecting = enable;
	if (enable) {
		if (0) {
			schedule_delayed_work(&ts_info->input_work,
				msecs_to_jiffies(ALGO_FREQ));
			baseline_flag = 1;
		}
		/*do something*/
		/*ftm4_earDetect_start_end(1);*/
		vivoFaceDetectStartEnd(1);
	} else {
		//complete(&unsign_comp);
		if (0) {
			msleep(50);
			cancel_delayed_work_sync(&ts_info->input_work);
			baseline_flag = 0;
		}
		/*do something*/
		/*ftm4_earDetect_start_end(0);*/
		vivoFaceDetectStartEnd(0);
	}
	sensors_cdev->enabled = ts_info->earDetecting;
	return ts_info->earDetecting;
}

static int algo_prox_poll_delay_set(struct sensors_classdev *sensors_cdev,
	unsigned int delay_msec)
{
	struct algo_prox_ts_info *ts_info;
	ts_info = container_of((struct sensors_classdev *)sensors_cdev, struct algo_prox_ts_info, cdev);
	/*ts_info = global_info;*/
	VTI("algo-prox algo_prox_poll_delay_set %d", delay_msec);
	/*eanble delay==10ms in whitelist*/
	if (delay_msec == 10) {
		if (0) {
			schedule_delayed_work(&ts_info->input_work,
				msecs_to_jiffies(ALGO_FREQ));
			baseline_flag = 1;
		}
		/*do something*/
		if (baseline_flag != 1) {
			baseline_flag = 1;
			/*ftm4_earDetect_start_end(1);*/
			vivoFaceDetectStartEnd(1);
			VTI("algo-prox real do enable_set %d", delay_msec);
		}
	} else {
		//complete(&unsign_comp);
		if (0) {
			msleep(50);
			cancel_delayed_work_sync(&ts_info->input_work);
			baseline_flag = 0;
		}
		/*do something*/
		if (baseline_flag == 1) {
			baseline_flag = 0;
			ts_info->earDetecting = 0;
			/*ftm4_earDetect_start_end(0);*/
			vivoFaceDetectStartEnd(0);
			VTI("algo-prox real do disable_set %d", delay_msec);
		}
	}
	sensors_cdev->enabled = ts_info->earDetecting;
	return ts_info->earDetecting;
}
/*daixiang add for algo-prox end*/
static void __fd_reinput_work(struct algo_prox_ts_info *info)
{
	spin_lock(&info->lock);
	input_report_rel(info->input_dev, REL_Z, (info->values + 1));
	input_sync(info->input_dev);
	VTI("input fd %d times", VTS_PROXIMITY_TRIGGER_TIME + 1 - info->nr_remain_report_times);
	if (--info->nr_remain_report_times > 0)
		schedule_delayed_work(&info->input_work, msecs_to_jiffies(VTS_PROXIMITY_TRIGGER_MS));
	spin_unlock(&info->lock);
}


static void fd_reinput_work(struct work_struct *work)
{
	__fd_reinput_work(container_of(work, struct algo_prox_ts_info, input_work.work));
}


int vivo_ts_prox_init(struct i2c_client *client)
{
	int retval = 0;
	struct algo_prox_ts_info *info = &prox_ts_info;
	
	/*daixiang add for algo-prox sensor start*/
	spin_lock_init(&info->lock);
	info->client = client;
	retval = algo_prox_input_init(info);
	if (retval < 0) {
		dev_err(&info->client->dev, "input init failed\n");
		goto vivo_ts_prox_err1;
	}

	info->cdev = algo_prox_cdev;
	info->cdev.sensors_enable = algo_prox_enable_set;
	info->cdev.sensors_poll_delay = algo_prox_poll_delay_set;
	retval = sensors_classdev_register(&info->client->dev, &info->cdev);
	if (retval < 0) {
		VTE("algo-prox sensors_classdev_register err");
		goto vivo_ts_prox_err0;
	}
	info->cdev.delay_msec = 0;
	INIT_DELAYED_WORK(&info->input_work, fd_reinput_work);

#if 0
	/*frist report defualt value prox far*/
	input_report_abs(info->input_dev, ABS_X, 1);
	input_report_abs(info->input_dev, ABS_Y, 0);
	input_report_abs(info->input_dev, ABS_Z, 0);
	//input_mt_sync(info->input_dev);
	input_sync(info->input_dev);
	/*daixiang add for algo-prox sensor end*/
#endif
	


	return retval;

vivo_ts_prox_err0:

	sensors_classdev_unregister(&info->cdev);

vivo_ts_prox_err1:

	return retval;

}

void vivo_ts_prox_report_values(int *xyz)
{
	struct algo_prox_ts_info *info = &prox_ts_info;
	if (info != NULL) {
		spin_lock(&info->lock);
		info->cdev.delay_msec = xyz[0]; //just for temp at factory
		info->values = xyz[0];
		input_report_rel(info->input_dev, REL_Z, (info->values + 1));
		input_sync(info->input_dev);
		VTI("input fd 1 times");
		info->nr_remain_report_times = VTS_PROXIMITY_TRIGGER_TIME - 1;
		spin_unlock(&info->lock);
		schedule_delayed_work(&info->input_work, msecs_to_jiffies(VTS_PROXIMITY_TRIGGER_MS));

	} else {
		VTE("prox_ts_info is NULL, fail to report values");
	}
}

int vivo_ts_prox_ear_detect_get(void)
{
	return prox_ts_info.earDetecting;
}

