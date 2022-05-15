/*
 *	sarpower class driver
 *
 * Copyright (C) 2020 vivo Technologies, Inc.
 * Author: Kangkai Deng<dengkangkai@vivo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
*/

#ifndef __LINUX_sarpower_H__
#define __LINUX_sarpower_H__

struct sarpower_dev {
	const char	*name;
	struct device	*dev;
	int		index;
	int		state;

	ssize_t	(*print_name)(struct sarpower_dev *sdev, char *buf);
	ssize_t	(*print_state)(struct sarpower_dev *sdev, char *buf);
};

extern int sarpower_dev_register(struct sarpower_dev *sdev);
extern void sarpower_dev_unregister(struct sarpower_dev *sdev);

static inline int sarpower_get_state(struct sarpower_dev *sdev)
{
	return sdev->state;
}

extern void sarpower_set_state(struct sarpower_dev *sdev, int state);

#endif /* __LINUX_sarpower_H__ */
