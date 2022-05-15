/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
/* vivo android project */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/of_address.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/sched/clock.h>
#include <linux/circ_buf.h>


static int handshake_open(struct inode *ip, struct file *fp)
{
	return 0;
}
static ssize_t handshake_write(struct file *fp, const char __user *buf,
	size_t count, loff_t *pos)
{
	return count;
}
static ssize_t handshake_read(struct file *fp, char __user *buf,
	size_t count, loff_t *pos)
{
   return count;
}


static long handshake_ioctl(struct file *fp, unsigned code, unsigned long value)
{
	return 0;
}
static int handshake_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static const struct file_operations handshake_fops = {
	.owner = THIS_MODULE,
	.read = handshake_read,
	.write = handshake_write,
	.unlocked_ioctl = handshake_ioctl,
	.open = handshake_open,
	.release = handshake_release,
};

static struct miscdevice handshake_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "handshake_accessory",
  .fops = &handshake_fops,
 };

#define HS_TIMEOUT_LOG_ARRAYCOUNT (256)

static int handshake_init_flag;

unsigned long long last_timestamp;

struct tcp_handshake_timeout_log {
	//unsigned long long timestamp;
	char handshakeinfo[256];
};

struct tcp_handshake_timeout_log handshake_timeout_array[HS_TIMEOUT_LOG_ARRAYCOUNT];

struct workqueue_struct *workqueue_handshake;

struct work_struct work_handshake;

struct handshake_circ_buf {
	struct tcp_handshake_timeout_log *buf;
	int head;   //next position to write
	int tail;   //next position to read 
};

static struct handshake_circ_buf hs_circ_buf = {
	.buf = handshake_timeout_array,
	.head = 0,
	.tail = 0,
};
spinlock_t hs_put_work_spinlock;
spinlock_t hs_get_work_spinlock;

void work_handshake_func(struct work_struct *work)
{
	int read_idx;
	//get work item , and send out.
	char data_buffer[256] = {0};
	char *envp[2] = { data_buffer, NULL };
	int result = 0;
	int got_item = 1;
	//while loop 
	while (got_item > 0) {
		int head;
		int tail;
		spin_lock(&hs_get_work_spinlock);
		/* Read index before reading contents at that index. */
		head = smp_load_acquire(&hs_circ_buf.head);
		tail = hs_circ_buf.tail;
		got_item = 0;
		if (CIRC_CNT(head, tail, HS_TIMEOUT_LOG_ARRAYCOUNT) >= 1) {
			read_idx = tail;
			if ((read_idx < HS_TIMEOUT_LOG_ARRAYCOUNT) && (read_idx >= 0)) {
				snprintf(data_buffer, sizeof(data_buffer), "ACCESSORY=%s,%d", 
					  handshake_timeout_array[read_idx].handshakeinfo, 
					  read_idx);

				got_item = 1;
			}
			/* Finish reading descriptor before incrementing tail. */
			smp_store_release(&hs_circ_buf.tail, 
				  (tail + 1) & (HS_TIMEOUT_LOG_ARRAYCOUNT - 1));

		}
		spin_unlock(&hs_get_work_spinlock);
		//then send to up layer. it may sleep?
		if (got_item > 0) {
			result = kobject_uevent_env(&handshake_device.this_device->kobj, KOBJ_CHANGE, envp);
		}
	}
}

static int __init handshake_init(void)
{
	printk(KERN_DEBUG "handshake_init");
	spin_lock_init(&hs_put_work_spinlock);
	spin_lock_init(&hs_get_work_spinlock);
	if (misc_register(&handshake_device) != 0) {
		printk(KERN_ERR "handshake_device register failed");
	}
	workqueue_handshake = alloc_workqueue("workqueue_handshake", 0, 0);
	INIT_WORK(&work_handshake, work_handshake_func);
    last_timestamp = 0;
	handshake_init_flag = 1;
	return 0;
} 
static void __exit handshake_exit(void)
{
	printk(KERN_DEBUG "handshake_exit");
	handshake_init_flag = 0;
	misc_deregister(&handshake_device);
	destroy_workqueue(workqueue_handshake);
}

//don't call this function too frequently
void  handshake_work(char handshakeinfo[])
{
	int write_idx = 0;
	int putted = 0;
	if (handshake_init_flag > 0) {
		//when to write,  just record, don't take care read slow issue.
		//schedule issue.
		int head;
		int tail;
		unsigned long long now;

		spin_lock(&hs_put_work_spinlock);
		//check time 1s 
		now = cpu_clock(raw_smp_processor_id());
		if ((now - READ_ONCE(last_timestamp)) < (1ULL*1000ULL*1000ULL*1000ULL)) {
			goto OUT;
		}
		WRITE_ONCE(last_timestamp, now);
		head = hs_circ_buf.head;
		tail = READ_ONCE(hs_circ_buf.tail);
		if (CIRC_SPACE(head, tail, HS_TIMEOUT_LOG_ARRAYCOUNT) >= 1) {
			//increase write position
			write_idx = head;
			//fill in the collected data.
			//handshake_timeout_array[write_idx].timestamp = local_clock();
			if ((write_idx < HS_TIMEOUT_LOG_ARRAYCOUNT) && (write_idx >= 0)) {
				strlcpy(handshake_timeout_array[write_idx].handshakeinfo, handshakeinfo, sizeof(handshake_timeout_array[write_idx].handshakeinfo)-1);
				putted = 1;
			}

			//update head
			smp_store_release(&hs_circ_buf.head,
				  (head + 1) & (HS_TIMEOUT_LOG_ARRAYCOUNT - 1));
		} else {
			printk(KERN_WARNING "handshake_work no space=%d", CIRC_SPACE(head, tail, HS_TIMEOUT_LOG_ARRAYCOUNT)); 
			//for this case, may add another handler.
			//head++. tail++
		}

OUT:
		spin_unlock(&hs_put_work_spinlock);

		if (putted > 0) {
			queue_work(workqueue_handshake, &work_handshake);
		}
	}
}

MODULE_LICENSE("GPL V2");
MODULE_AUTHOR("test@test.com");
MODULE_DESCRIPTION("Control for handshake");

MODULE_VERSION("1.5.0");

module_init(handshake_init);
module_exit(handshake_exit);

EXPORT_SYMBOL(handshake_work);//exported for other module used
