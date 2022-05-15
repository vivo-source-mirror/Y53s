#ifndef _LINUX_VIVO_HALL_H_
#define _LINUX_VIVO_HALL_H_

struct vivo_hall_dev {
	const char	*name;

	struct device	*dev;
	int		index;
	int		state;
};

int vivo_hall_dev_register(struct vivo_hall_dev *dev);
void vivo_hall_dev_unregister(struct vivo_hall_dev *dev);

#endif