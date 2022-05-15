/*
 *  linux/drivers/../../tusb422_notify.c
 *
 *  Copyright (C) 2006 Antonino Daplas <adaplas@pol.net>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/notifier.h>

extern int tusb_register_client(struct notifier_block *nb);
extern int tusb_unregister_client(struct notifier_block *nb);
extern int tusb_notifier_call_chain(unsigned long val, void *v);


