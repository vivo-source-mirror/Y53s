#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/unistd.h>
#include "vts_core.h"


#define VTS_SPI_DRV_NAME "vivo,ts-spi"
static atomic_t driver_registered = ATOMIC_INIT(0);
static struct spi_device *vts_spi_dev = NULL;

static int vts_spi_probe(struct spi_device *spi)
{
	vts_spi_dev = spi;
	VTI("probed\n");
	return 0;
}

static int vts_spi_remove(struct spi_device *spi)
{
	return 0;
}

static void vts_spi_shutdown(struct spi_device *spi)
{
	VTI("vts spi device shut down !");
}

static const struct spi_device_id vts_ts_id[] = {
    {VTS_SPI_DRV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(spi, vts_ts_id);

static struct of_device_id vts_match_table[] = {
    { .compatible = VTS_SPI_DRV_NAME, },
    { },
};


static struct spi_driver vts_spi_drv = {
    .probe = vts_spi_probe,
    .remove = vts_spi_remove,
    .driver = {
        .name = VTS_SPI_DRV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = vts_match_table,
    },
    .shutdown = vts_spi_shutdown,
    .id_table = vts_ts_id,
};

int vts_spi_drv_reigster(struct vts_spi_driver *drv)
{
	struct device_node *child = NULL;
	int ret;

	if (atomic_inc_return(&driver_registered) == 1) {
		ret = spi_register_driver(&vts_spi_drv);
		if (ret) {
			VTE("probe "VTS_SPI_DRV_NAME" failed, ret = %d\n", ret);
			return ret;
		}
	}

	if (!vts_spi_dev) {
		VTE("no "VTS_SPI_DRV_NAME" device in dts\n");
		return -ENODEV;
	}

	while ((child = of_get_next_child(vts_spi_dev->dev.of_node, child)) != NULL) {
		if(of_device_is_compatible(child, drv->compatible)) {
			VTI("found %s node in "VTS_SPI_DRV_NAME" node\n", drv->compatible);
			if(vts_dev_match_module_ic_number(child)){
				if(drv->shutdown){
				 vts_spi_drv.shutdown = drv->shutdown;
				}
				return drv->probe(vts_spi_dev, child);
			}
		}
	}

	VTE("no child named %s in "VTS_SPI_DRV_NAME" node\n", drv->compatible);
	return -ENODEV;
}


int vts_spi_drv_unreigster(struct vts_spi_driver *drv)
{
	return 0;
}

