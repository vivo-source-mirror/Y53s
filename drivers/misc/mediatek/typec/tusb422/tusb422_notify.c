/*
 *  linux/drivers/video/fb_notify.c
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
#include <linux/power/vivo/tusb_notify.h>
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(tusb_notifier_list);

/**
 *	fb_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int tusb_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&tusb_notifier_list, nb);
}
EXPORT_SYMBOL(tusb_register_client);

/**
 *	fb_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int tusb_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&tusb_notifier_list, nb);
}
EXPORT_SYMBOL(tusb_unregister_client);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *
 */
int tusb_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&tusb_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(tusb_notifier_call_chain);
