#ifndef __VTS_SPI_DRV_H
#define __VTS_SPI_DRV_H

#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/list.h>

struct vts_spi_driver {
	struct list_head list;
	const char *compatible;
	int (*probe)(struct spi_device *spi, struct device_node *np);
	int (*remove)(struct spi_device *spi, struct device_node *np);
	void (*shutdown)(struct spi_device *spi);
	
};

int vts_spi_drv_reigster(struct vts_spi_driver *drv);
int vts_spi_drv_unreigster(struct vts_spi_driver *drv);

#endif
