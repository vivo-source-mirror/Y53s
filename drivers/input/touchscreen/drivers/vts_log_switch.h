#ifndef VTS_LOG_SWITCH_H
#define VTS_LOG_SWITCH_H
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/gpio.h>

struct vts_callback_handler {
	void (*callback)(bool is_switch_on);
	void (*screenshot_callback)(struct vts_callback_handler *entry, unsigned int count);
	struct list_head list;
};

extern int vts_log_switch_register(struct vts_callback_handler *handler);
extern void vts_log_switch_unregister(struct vts_callback_handler *handler);

#endif