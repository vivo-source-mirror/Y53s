#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <asm/unistd.h>
#include <asm/delay.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include "vivo_audio_ktv.h"

#define KTV_DUMP_ENABLE

#define VIVO_AUDIO_KTV_DEVICE_NAME "vivo_audio_ktv_dev"

static int vivo_audio_ktv_dev_Open;
static struct vivo_audio_ktv_prv g_vivo_audio_ktv;

#define VIVO_AUDIO_KTV_PERIOD_FRAME 240

#define VIVO_AUDIO_KTV_PERIOD_SIZE  1920 /*  5 *  48 * 2 *  4 */
										 /* 5ms 48K 2ch 32bits */
#define VIVO_AUDIO_KTV_BUFFER_NUM  8
										 /* 4 buffer 20ms */

uint8_t save_processed_buf[VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM];

uint8_t *vivo_audio_ktv_mmap_buf;


DECLARE_WAIT_QUEUE_HEAD(vivo_audio_ktv_wait);

static uint32_t vivo_audio_ktv_data_is_full;

static int vivo_audio_ktv_dev_open(struct inode *inode, struct file *file);
static int vivo_audio_ktv_dev_release(struct inode *inode, struct file *file);

static ssize_t vivo_audio_ktv_dev_read(
		struct file *file,
		char *buffer,
		size_t length,
		loff_t *offset);
static ssize_t vivo_audio_ktv_dev_write(
		struct file *file,
		const char __user *buffer,
		size_t length,
		loff_t *offset);

static long vivo_audio_ktv_dev_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param);

static long vivo_audio_ktv_dev_compat_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param);

static unsigned vivo_audio_ktv_dev_poll(struct file *file, poll_table *wait);

static int vivo_audio_ktv_dev_mmap(struct file *file, struct vm_area_struct *vma);

#ifdef KTV_DUMP_ENABLE

#define DUMP_BUF_SIZE (8 * 4096) //32KB
static uint8_t ktv_rx_final_dump[DUMP_BUF_SIZE];

struct ktv_dump_info {
	const char *name;
	bool started;
	size_t pointer;
	size_t file_pointer;
	uint8_t *buf_area;
	size_t buf_size;
	struct proc_dir_entry *file;
	wait_queue_head_t file_waitqueue;
};

static struct ktv_dump_info ktv_rx_final_info;

static ssize_t ktv_dump_file_read(struct file *file,
		char __user *data, size_t count, loff_t *ppos)
{
	struct ktv_dump_info *info = file->private_data;
	size_t end, pointer;
	ssize_t size;
	int ret;

	if (!info->buf_size) {
		pr_err("%s buf size invalid, failed\n", __func__);
		return size;
	}

	do {
		pointer = READ_ONCE(info->pointer);
		end = (info->file_pointer <= pointer) ? pointer :
				info->buf_size;
		size = min(end - info->file_pointer, count);
		pr_debug("pointer %#zx file_pointer %#zx size %#zx\n",
				pointer, info->file_pointer, size);
		if (!size) {
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ret = wait_event_interruptible(info->file_waitqueue,
					pointer != READ_ONCE(info->pointer));
			if (ret < 0)
				return ret;
		}
	} while (!size);

	if (copy_to_user(data, info->buf_area + info->file_pointer, size))
		return -EFAULT;

	info->file_pointer += size;
	info->file_pointer %= info->buf_size;

	return size;
}

static int ktv_dump_file_open(struct inode *i, struct file *f)
{
	struct ktv_dump_info *info = PDE_DATA(file_inode(f));

	pr_info("%s\n", __func__);

	f->private_data = info;
	info->started = true;
	info->pointer = info->file_pointer = 0;

	return 0;
}

static int ktv_dump_file_release(struct inode *i, struct file *f)
{
	struct ktv_dump_info *info = f->private_data;

	pr_info("%s\n", __func__);

	info->started = false;
	return 0;
}

static unsigned int ktv_dump_file_poll(struct file *file, poll_table *wait)
{
	struct ktv_dump_info *info = file->private_data;

	pr_info("%s\n", __func__);

	poll_wait(file, &info->file_waitqueue, wait);
	return POLLIN | POLLRDNORM;
}

static struct file_operations ktv_dump_fops = {
	.llseek = generic_file_llseek,
	.read = ktv_dump_file_read,
	.poll = ktv_dump_file_poll,
	.open = ktv_dump_file_open,
	.release = ktv_dump_file_release,
	.owner = THIS_MODULE,
};

static void ktv_dump_data_save(struct ktv_dump_info *info, const uint8_t *buf, size_t bytes)
{
	size_t size, pointer;

	if (bytes > info->buf_size || !info->buf_size) {
		pr_err("%s buf size invalid, failed\n", __func__);
		return;
	}

	pr_debug("%s %zx, %zx\n", __func__, bytes, info->pointer);

	size = min(bytes, info->buf_size - info->pointer);
	memcpy(info->buf_area + info->pointer, buf, size);
	if (bytes - size > 0)
		memcpy(info->buf_area, buf + size, bytes - size);

	pointer = (info->pointer + bytes) % info->buf_size;

	info->pointer = pointer;
	wake_up_interruptible(&info->file_waitqueue);
	return;
}

static void ktv_dump_register_file(struct ktv_dump_info *info)
{
	static struct proc_dir_entry *dir_dump;

	if (!dir_dump)
		dir_dump = proc_mkdir("vivo-ktv", NULL);

	info->name = "rx_final_dump";
	info->buf_area = ktv_rx_final_dump;
	info->buf_size = DUMP_BUF_SIZE;
	info->file = proc_create_data(info->name, 0664, dir_dump, &ktv_dump_fops, (void *)info);
	init_waitqueue_head(&info->file_waitqueue);

	return;
}
#endif

struct file_operations vivo_audio_ktv_dev_fops = {
	.owner 			= THIS_MODULE,
	.open 			= vivo_audio_ktv_dev_open,
	.release 		= vivo_audio_ktv_dev_release,
	.read 			= vivo_audio_ktv_dev_read,
	.write 			= vivo_audio_ktv_dev_write,
	.unlocked_ioctl = vivo_audio_ktv_dev_ioctl,
	.compat_ioctl   = vivo_audio_ktv_dev_compat_ioctl,
	.poll 			= vivo_audio_ktv_dev_poll,
	.mmap			= vivo_audio_ktv_dev_mmap,
};


struct miscdevice vivo_audio_ktv_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = VIVO_AUDIO_KTV_DEVICE_NAME,
	.fops = &vivo_audio_ktv_dev_fops,
	.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH,
};



static void vivo_audio_ktv_prv_init(void)
{
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 0;
	g_vivo_audio_ktv.mixer_flag = 0;
	g_vivo_audio_ktv.ears_back = 0;
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_write_total = 0;
	g_vivo_audio_ktv.buffer_size = VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM;
	g_vivo_audio_ktv.buffer_in_samples = (g_vivo_audio_ktv.buffer_size >> 2);
	*g_vivo_audio_ktv.share_buf_idx_addr = 0;


	vivo_audio_ktv_tx_init();
	vivo_audio_ktv_rx_init();

    /* set IRQ to 5ms */
	/* vivo_audio_ktv_set_irq_cnt(); */

}

void vivo_audio_ktv_set_irq_mcu_counter(void)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		/* set IRQ to 5ms */
		/* vivo_audio_ktv_set_irq_cnt(); */
	}
}

void vivo_audio_ktv_set_channels(int channels)
{
	g_vivo_audio_ktv.tx_channels = channels;
}

int vivo_audio_ktv_get_status(void)
{
	return g_vivo_audio_ktv.vivo_audio_ktv_flag;
}
static void vivo_audio_ktv_open(void)
{
	printk("%s\n", __func__);
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 1;
	g_vivo_audio_ktv.tx_channels = 1;

}

static void vivo_audio_ktv_close(void)
{
	printk("%s\n", __func__);
	g_vivo_audio_ktv.vivo_audio_ktv_flag = 0;
	vivo_audio_ktv_data_is_full = 1;
	wake_up(&vivo_audio_ktv_wait);

}

static void vivo_audio_ktv_set_mixer_flag(unsigned long ioctl_param)
{
	printk("%s: flag = %ld\n", __func__, ioctl_param);
	g_vivo_audio_ktv.mixer_flag = (int32_t)ioctl_param;

}

static void vivo_audio_ktv_set_ears_back(unsigned long ioctl_param)
{
	printk("%s: flag = %ld\n", __func__, ioctl_param);
	g_vivo_audio_ktv.ears_back = !!(int32_t)ioctl_param;

}

static int vivo_audio_ktv_dev_open(struct inode *inode, struct file *file)
{

	printk("%s\n", __func__);

	if (vivo_audio_ktv_dev_Open)
		return -EBUSY;

	vivo_audio_ktv_dev_Open++;
	try_module_get(THIS_MODULE);

	vivo_audio_ktv_data_is_full = 0;

	vivo_audio_ktv_mmap_buf = (uint8_t *)kmalloc(4*1024*4, GFP_KERNEL);
	if (vivo_audio_ktv_mmap_buf == NULL) {
		printk(KERN_ERR "%s malloc buf faild\n", __func__);
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(vivo_audio_ktv_mmap_buf));

	g_vivo_audio_ktv.mmap_share_buffer_addr = vivo_audio_ktv_mmap_buf;
	g_vivo_audio_ktv.share_buf_idx_addr = (uint64_t *)(vivo_audio_ktv_mmap_buf
		+ VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM);
	vivo_audio_ktv_prv_init();

	memset(save_processed_buf, 0, sizeof(uint8_t) * VIVO_AUDIO_KTV_PERIOD_SIZE * VIVO_AUDIO_KTV_BUFFER_NUM);

	return 0;
}

static int vivo_audio_ktv_dev_release(struct inode *inode, struct file *file)
{
	printk("%s (%p,%p)\n", __func__, inode, file);
	vivo_audio_ktv_dev_Open--;
	vivo_audio_ktv_data_is_full = 0;

	ClearPageReserved(virt_to_page(vivo_audio_ktv_mmap_buf));
	kfree(vivo_audio_ktv_mmap_buf);
	vivo_audio_ktv_mmap_buf = NULL;
	module_put(THIS_MODULE);
	return 0;

}

static ssize_t vivo_audio_ktv_dev_read(
		struct file *file,
		char *buffer,
		size_t length,
		loff_t *offset)
{

	return 0;
}

static ssize_t vivo_audio_ktv_dev_write(
		struct file *file,
		const char __user *buffer,
		size_t length,
		loff_t *offset)
{

	return 0;

}

static long vivo_audio_ktv_dev_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param)
{
	printk("%s ioctl = %u\n", __func__, cmd);

	switch (cmd) {
	case IOCTL_VIVO_AUDIO_KTV_OPEN:
		vivo_audio_ktv_open();
		break;
	case IOCTL_VIVO_AUDIO_KTV_CLOSE:
		vivo_audio_ktv_close();
		break;
	case IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG:
		vivo_audio_ktv_set_mixer_flag(ioctl_param);
		break;
	case IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK:
		vivo_audio_ktv_set_ears_back(ioctl_param);
		break;
	default:
		printk(KERN_ERR "%s wrong ioctrl 0x%x\n", __func__, cmd);
		return -ENOTTY;
	}
	return 0;
}

static long vivo_audio_ktv_dev_compat_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long ioctl_param)
{
	printk("%s ioctl = %u\n", __func__, cmd);

	switch (cmd)	{
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_OPEN:
		cmd = IOCTL_VIVO_AUDIO_KTV_OPEN;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_CLOSE:
		cmd = IOCTL_VIVO_AUDIO_KTV_CLOSE;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG:
		cmd = IOCTL_VIVO_AUDIO_KTV_SET_MIXER_FLAG;
		break;
	case COMPAT_IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK:
		cmd = IOCTL_VIVO_AUDIO_KTV_SET_EARS_BACK;
		break;
	default:
		printk(KERN_ERR "%s wrong ioctrl 0x%x\n", __func__, cmd);
		return -ENOTTY;

	}

	return vivo_audio_ktv_dev_ioctl(file, cmd, ioctl_param);

}

static unsigned vivo_audio_ktv_dev_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &vivo_audio_ktv_wait, wait);

	if (vivo_audio_ktv_data_is_full)	{
		mask |= (POLLIN | POLLRDNORM);
	}
	vivo_audio_ktv_data_is_full = 0;
	return mask;

}

static int vivo_audio_ktv_dev_mmap(struct file *file, struct vm_area_struct *vma)
{

	unsigned long page = (virt_to_phys(vivo_audio_ktv_mmap_buf) >> PAGE_SHIFT);
	unsigned long vmsize = vma->vm_end - vma->vm_start;
	unsigned long size = sizeof(vivo_audio_ktv_mmap_buf);

	printk("%s size = %lu, vmsize = %lu\n", __func__, size, vmsize);

	/* PAGE_SIZE */
	if (remap_pfn_range(vma, vma->vm_start, page, vmsize, PAGE_SHARED)) {
		printk(KERN_ERR "%s faild\n", __func__);
		return -EAGAIN;
	}
	return 0;

}

void vivo_audio_ktv_tx_init(void)
{
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_write_total = 0;


	g_vivo_audio_ktv.tx_first_cnt = 0; /* for the first time check if data is 5ms period */

	g_vivo_audio_ktv.tx_write_remained = 0;

	g_vivo_audio_ktv.write_share_buffer_idx = 0;
	g_vivo_audio_ktv.read_share_buffer_idx = 0;

	g_vivo_audio_ktv.tx_saved_read_idx = 0;
	g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;
	g_vivo_audio_ktv.rx_mixer_flag = 0;

}

void vivo_audio_ktv_rx_init(void)
{
	g_vivo_audio_ktv.rx_write_total = 0;
	g_vivo_audio_ktv.tx_read_total = 0;
	g_vivo_audio_ktv.rx_mixer_flag = 0;

	g_vivo_audio_ktv.tx_first_cnt = 0;

	g_vivo_audio_ktv.read_share_buffer_idx = g_vivo_audio_ktv.write_share_buffer_idx;

	g_vivo_audio_ktv.tx_saved_read_idx = 0;
	g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;


}

/********************************************************************
*  Function: get tx data for ktv algorithm processing, the data will be read in user
*   space, and processed by ktv algorithm, if data is gether more than one periods
*   VIVO_AUDIO_KTV_PERIOD_SIZE, it will poll to inform the user to takeaway the
*   data.
*  addr_start: the virtual address of kernel buffer for tx(usually DMA buffer)
*  readIdx: the cursor of tx data start to read
*  size: the size of tx data want to read, after that the cursor will move to readIdx+size
*  buffer_size: the DMA buffer size, decide by user process.
*
*******************************************************************/

int32_t vivo_audio_ktv_save_tx_data(uint8_t *addr_start, int32_t readIdx, int32_t size, int32_t buffer_size)
{

	uint32_t *user_buffer_addr;
	uint32_t *kernel_buffer_addr;
	uint32_t *saved_buffer_addr;

	uint32_t frame_count;
	uint32_t actual_size;
	int32_t valueLeft, valueRight;

	int32_t i;
	uint32_t tx_kernel_idx;
	uint32_t user_idx;
	uint32_t save_idx;
	int32_t buffer_in_samples = buffer_size >> 2; /* 32bit for one samples */

	PRINTK_AUD_KTV(KERN_DEBUG "vivo_audio_ktv_save_tx_data readIdx = 0x%x size = 0x%x.\n", readIdx, size);
#if 0
   /* add for the interrupt time is not 5ms at the start of the catputre, ignore the first 20ms data. */
	if (g_vivo_audio_ktv.tx_first_cnt == 0 && size >= VIVO_AUDIO_KTV_PERIOD_SIZE * 4) {
		return 0;
	 }

	g_vivo_audio_ktv.tx_first_cnt += size;

	 /* ignore the first two periods (10ms) datas,but still need to move tx_kernel_idx */
	 if (g_vivo_audio_ktv.tx_first_cnt <= VIVO_AUDIO_KTV_PERIOD_SIZE * 2) {
		PRINTK_AUD_KTV("vivo_audio_ktv_save_tx_data tx_first_cnt = %lld\n", g_vivo_audio_ktv.tx_first_cnt);
		tx_kernel_idx = readIdx + size;

		if (tx_kernel_idx >= buffer_size) {

			tx_kernel_idx -= buffer_size;

		}

		return 0;
	 }

	if (tx_kernel_idx != readIdx) {
		if (readIdx + size > tx_kernel_idx) {
			actual_size = readIdx + size - tx_kernel_idx;
		} else {
			actual_size = 0;
		}

		if (actual_size >= buffer_size)
			actual_size -= buffer_size;

		if (actual_size >= (buffer_size >> 1)) {
			actual_size = 0;
		}

		if (actual_size == 0) {
			return 0;
		}

	} else {
		actual_size = size;
	}
#else
	g_vivo_audio_ktv.tx_first_cnt += size;

	 /* ignore the first two periods (10ms) datas to avoid pop noise,but still need to move tx_kernel_idx */
	if (g_vivo_audio_ktv.tx_first_cnt <= VIVO_AUDIO_KTV_PERIOD_SIZE * 2) {
		PRINTK_AUD_KTV("vivo_audio_ktv_save_tx_data tx_first_cnt = %lld\n", g_vivo_audio_ktv.tx_first_cnt);
		tx_kernel_idx = readIdx + size;

		if (tx_kernel_idx >= buffer_size) {

			tx_kernel_idx -= buffer_size;

		}

		return 0;
	}
	if (size > buffer_size)
		size -= buffer_size;
	actual_size = size;
	tx_kernel_idx = readIdx;
#endif
	if (actual_size > VIVO_AUDIO_KTV_PERIOD_SIZE) {
		PRINTK_AUD_KTV(KERN_DEBUG "%s overflow size = 0x%x, actual_size=%d. \n", __func__, size, actual_size);
	}

	kernel_buffer_addr = (int32_t *)(addr_start);
	tx_kernel_idx = (tx_kernel_idx >> 2); /* bitwidth=32bit, 4byte */

	user_buffer_addr = (int32_t *)(g_vivo_audio_ktv.mmap_share_buffer_addr);
	user_idx = (g_vivo_audio_ktv.write_share_buffer_idx >> 2); /* bitwidth=32bit, 4byte */

	saved_buffer_addr = (int32_t *)(save_processed_buf);
	save_idx = (g_vivo_audio_ktv.tx_saved_read_idx >> 2); /* bitwidth=32bit, 4byte */
	PRINTK_AUD_KTV(KERN_DEBUG "vivo_audio_ktv_save_tx_data tx_kernel_idx = 0x%x user_idx = 0x%x save_idx = 0x%x actual_size = 0x%x, mixer_flag=%d, rx_mixer_flag=%d, tx_read_total=%d.\n",
		(tx_kernel_idx << 2), (user_idx << 2), (save_idx << 2), actual_size, g_vivo_audio_ktv.mixer_flag, g_vivo_audio_ktv.rx_mixer_flag, g_vivo_audio_ktv.tx_read_total);

	if (g_vivo_audio_ktv.tx_channels >= 2)
		frame_count = actual_size >> 3; /* byte to frame(32bit * 2ch) */
	else
		frame_count = actual_size >> 2;

	for (i = 0; i < frame_count; i++) {
		if (IS_ERR_OR_NULL(kernel_buffer_addr)) {
			PRINTK_AUD_KTV(KERN_DEBUG "%s: kernel_buffer_addr is eror.\n", __func__);
			return -1;
		}
		user_buffer_addr[user_idx] = kernel_buffer_addr[tx_kernel_idx];

		if (g_vivo_audio_ktv.mixer_flag) {
			valueLeft = saved_buffer_addr[save_idx];
			//valueLeft <<= 1;
			if (valueLeft > 0x7fffff)
				valueLeft = 0x7fffff;

			if (valueLeft < -0x800000)
				valueLeft = -0x800000;

			if (g_vivo_audio_ktv.rx_mixer_flag)
				kernel_buffer_addr[tx_kernel_idx] = valueLeft;
			else
				kernel_buffer_addr[tx_kernel_idx] = 0;

		} else {
			valueLeft = kernel_buffer_addr[tx_kernel_idx];
			valueLeft <<= 1;
			if (valueLeft > 0x7fffff)
				valueLeft = 0x7fffff;

			if (valueLeft < -0x800000)
				valueLeft = -0x800000;

			kernel_buffer_addr[tx_kernel_idx] = valueLeft;
		}
		user_idx++;
		save_idx++;

		if (g_vivo_audio_ktv.tx_channels >= 2) /* only for channel is bigger than 2, get the same data if channel is 1*/
			tx_kernel_idx++;
		else
			user_buffer_addr[user_idx] = user_buffer_addr[user_idx-1]; /*tx is mono, so to ktv algo right = left*/

		user_buffer_addr[user_idx] = kernel_buffer_addr[tx_kernel_idx];

		if (g_vivo_audio_ktv.mixer_flag) {
			valueRight = saved_buffer_addr[save_idx];
			//valueRight <<= 1;
			if (valueRight > 0x7fffff)
				valueRight = 0x7fffff;

			if (valueRight < -0x800000)
				valueRight = -0x800000;
			if (g_vivo_audio_ktv.rx_mixer_flag)
				kernel_buffer_addr[tx_kernel_idx] = valueRight;
			else
				kernel_buffer_addr[tx_kernel_idx] = 0;

		} else {
			valueRight = kernel_buffer_addr[tx_kernel_idx];
			valueRight <<= 1;
			if (valueRight > 0x7fffff)
				valueRight = 0x7fffff;

			if (valueRight < -0x800000)
				valueRight = -0x800000;

			kernel_buffer_addr[tx_kernel_idx] = valueRight;
		}

		user_idx++;
		tx_kernel_idx++;
		save_idx++;

		if (tx_kernel_idx >= buffer_in_samples) {
			tx_kernel_idx = 0;
		}

		if (user_idx >= g_vivo_audio_ktv.buffer_in_samples) {
			user_idx = 0;
		}

		if (save_idx >= g_vivo_audio_ktv.buffer_in_samples) {
			save_idx = 0;
		}

	}

	if (g_vivo_audio_ktv.rx_mixer_flag)
		g_vivo_audio_ktv.tx_saved_read_idx = (save_idx << 2);

	g_vivo_audio_ktv.write_share_buffer_idx = (user_idx << 2);

	if (g_vivo_audio_ktv.tx_channels >= 2) {
		g_vivo_audio_ktv.tx_read_total += actual_size;
		g_vivo_audio_ktv.tx_write_remained += actual_size;
	} else {
		g_vivo_audio_ktv.tx_read_total += actual_size*2;
		g_vivo_audio_ktv.tx_write_remained += actual_size*2;
	}

	/* if data is full, inform the user process to take away the data */
	if (g_vivo_audio_ktv.tx_write_remained >= VIVO_AUDIO_KTV_PERIOD_SIZE) {
		g_vivo_audio_ktv.tx_write_remained -= VIVO_AUDIO_KTV_PERIOD_SIZE;
		*g_vivo_audio_ktv.share_buf_idx_addr = (g_vivo_audio_ktv.write_share_buffer_idx / VIVO_AUDIO_KTV_PERIOD_SIZE);
		vivo_audio_ktv_data_is_full = 1;
		wake_up(&vivo_audio_ktv_wait);

	}
	return 0;

}

/****************************************************************************************
*  Function: this will mixer tx data to rx, the tx data is the one has been process by ktv algorithm, rx is the
*   original playback data. the mixer is tx_left + rx_left = left_final, tx_right + rx_right = right_final
*  addr_start: the virtual address of rx kernel buffer for playback, the mixerdata will put back the kernel buffer
*  writeIdx: the cursor of data in kernel to start mixer
*  size: the size in byte to mixer
*  buffer_size: the DMA buffer size, decide by user process
*
*********************************************************************************************/
int32_t vivo_audio_ktv_mixer_tx_to_rx(uint8_t *addr_start, int32_t writeIdx, int32_t size, int32_t buffer_size)
{

	ssize_t frame_count;
	int32_t i;
	int32_t txDataLeft, txDataRight, rxDataLeft, rxDataRight;
	int32_t valueLeft, valueRight;

	uint32_t *user_buffer_addr;
	uint32_t *kernel_buffer_addr;
	uint32_t *saved_buffer_addr;

	int32_t kernel_idx;
	int32_t user_idx;
	int32_t save_idx;
	int32_t buffer_in_samples = buffer_size >> 2; /* 32bit for one sample */

#ifdef KTV_DUMP_ENABLE
	uint8_t *addr_start_save = addr_start;
	int32_t writeIdx_save = writeIdx;
	int32_t size_save = size;
#endif

	PRINTK_AUD_KTV(KERN_DEBUG "%s : writeIdx = 0x%x size = 0x%x, mixer_flag=%d, rx_mixer_flag=%d, tx_read_total=%d, rx_write_total=%d, tx_channel=%d.\n",
		__func__, writeIdx, size, g_vivo_audio_ktv.mixer_flag, g_vivo_audio_ktv.rx_mixer_flag, g_vivo_audio_ktv.tx_read_total, g_vivo_audio_ktv.rx_write_total, g_vivo_audio_ktv.tx_channels);

	if (g_vivo_audio_ktv.rx_mixer_flag == 0) { /* ignore first 4 periods data(4*5ms) to avoid pop noise when open*/
		if ((g_vivo_audio_ktv.rx_write_total + 4 * VIVO_AUDIO_KTV_PERIOD_SIZE) > g_vivo_audio_ktv.tx_read_total) {
			return 0;
		} else {
			g_vivo_audio_ktv.rx_mixer_flag = 1;
		}
	} else {
		if (size > VIVO_AUDIO_KTV_PERIOD_SIZE) {
			PRINTK_AUD_KTV("%s overflow size = %d\n", __func__, size);
		}

		if ((g_vivo_audio_ktv.rx_write_total  + size) > g_vivo_audio_ktv.tx_read_total) {
			g_vivo_audio_ktv.rx_mixer_flag = 0;
			return 0;
		}
	}

	if (g_vivo_audio_ktv.tx_read_total == 0 || (g_vivo_audio_ktv.tx_read_total < g_vivo_audio_ktv.rx_write_total))
		return 0;

	if ((g_vivo_audio_ktv.tx_read_total >= g_vivo_audio_ktv.rx_write_total) && (g_vivo_audio_ktv.tx_read_total - g_vivo_audio_ktv.rx_write_total >= VIVO_AUDIO_KTV_PERIOD_SIZE * 7)) {
		PRINTK_AUD_KTV(KERN_DEBUG "%s: overflow need reset\n", __func__);
		g_vivo_audio_ktv.rx_write_total = 0;
		g_vivo_audio_ktv.tx_read_total = 0;
		g_vivo_audio_ktv.rx_mixer_flag = 0;

		g_vivo_audio_ktv.read_share_buffer_idx = g_vivo_audio_ktv.write_share_buffer_idx;

		g_vivo_audio_ktv.tx_saved_read_idx = 0;
		g_vivo_audio_ktv.rx_saved_write_idx = 2 * VIVO_AUDIO_KTV_PERIOD_SIZE;
		*g_vivo_audio_ktv.share_buf_idx_addr = 0;
		return 0;
	}

	PRINTK_AUD_KTV(KERN_DEBUG "%s : rx_write_total = %lld tx_read_total = %lld diff = %lld\n", __func__,
		g_vivo_audio_ktv.rx_write_total, g_vivo_audio_ktv.tx_read_total, g_vivo_audio_ktv.tx_read_total - g_vivo_audio_ktv.rx_write_total);

	g_vivo_audio_ktv.rx_write_total += size;

	kernel_buffer_addr = (int32_t *)(addr_start);
	kernel_idx = writeIdx >> 2; /* bit_width_sample=32 */
	if (kernel_idx >= buffer_in_samples) {
		kernel_idx -= buffer_in_samples;
	}


	user_buffer_addr = (int32_t *)(g_vivo_audio_ktv.mmap_share_buffer_addr);
	user_idx = (g_vivo_audio_ktv.read_share_buffer_idx >> 2);

	saved_buffer_addr = (int32_t *)(save_processed_buf);
	save_idx = (g_vivo_audio_ktv.rx_saved_write_idx >> 2);
	PRINTK_AUD_KTV(KERN_DEBUG "%s : kernel_idx = 0x%x user_idx = 0x%x save_idx = 0x%x\n", __func__,
		(kernel_idx << 2), (user_idx << 2), (save_idx << 2));

	frame_count = (size >> 3);

	for (i = 0; i < frame_count; i++) {
		if (IS_ERR_OR_NULL(kernel_buffer_addr)) {
			PRINTK_AUD_KTV(KERN_DEBUG "%s: kernel_buffer_addr is eror.\n", __func__);
			return -1;
		}
		txDataLeft = user_buffer_addr[user_idx]; /* the data has been processed by ktv algorithm */
		user_buffer_addr[user_idx] = 0;
		rxDataLeft = kernel_buffer_addr[kernel_idx]; /* the original playback data */

		valueLeft = rxDataLeft  + txDataLeft; /*mix the tx to rx playback data */


		if (valueLeft > 0x7fffff)
			valueLeft = 0x7fffff;

		if (valueLeft < -0x800000)
			valueLeft = -0x800000;

		/* inverse the left channel value to counteract the right siginal in headset to fix crosstalk noise */
		if (valueLeft <= -0x800000) // -0x800000 is 0xff80 0000, inverse it will be 0x0080 0000 (0x800000), for 24bit, the 0x800000 is -8388608,value doesn't inverse
			valueLeft = 0x7fffff; // so here set to 0x7fffff
		else
			valueLeft = -valueLeft;

		if (g_vivo_audio_ktv.ears_back)
			kernel_buffer_addr[kernel_idx] = valueLeft; /* put the mix data to playback buffer again, to play them together. */
#if 0
		valueLeft <<= 2; //here to add 16dB, easy to cause saturation distorsion
		if (valueLeft > 0x7fffff)
			valueLeft = 0x7fffff;

		if (valueLeft < -0x800000)
			valueLeft = -0x800000;
#endif
		if (g_vivo_audio_ktv.mixer_flag == 2)
			saved_buffer_addr[save_idx] = txDataLeft;
		else
			saved_buffer_addr[save_idx] = valueLeft;

		user_idx++;
		kernel_idx++;
		save_idx++;

		txDataRight = user_buffer_addr[user_idx];
		user_buffer_addr[user_idx] = 0;
		rxDataRight = kernel_buffer_addr[kernel_idx];


		valueRight = rxDataRight + txDataRight;


		if (valueRight > 0x7fffff)
			valueRight = 0x7fffff;

		if (valueRight < -0x800000)
			valueRight = -0x800000;

		if (g_vivo_audio_ktv.ears_back)
			kernel_buffer_addr[kernel_idx] = valueRight;

#if 0
		valueRight <<= 2; //here to add 16dB, easy to cause saturation distorsion
		if (valueRight > 0x7fffff)
			valueRight = 0x7fffff;

		if (valueRight < -0x800000)
			valueRight = -0x800000;
#endif
		if (g_vivo_audio_ktv.mixer_flag == 2)
			saved_buffer_addr[save_idx] = txDataRight;
		else
			saved_buffer_addr[save_idx] = valueRight;

		user_idx++;
		kernel_idx++;
		save_idx++;

		if (kernel_idx >= buffer_in_samples) {
			kernel_idx = 0;
		}

		if (user_idx >= g_vivo_audio_ktv.buffer_in_samples) {
			user_idx = 0;
		}

		if (save_idx >= g_vivo_audio_ktv.buffer_in_samples) {
			save_idx = 0;
		}

	}

	g_vivo_audio_ktv.rx_saved_write_idx = (save_idx << 2);
	g_vivo_audio_ktv.read_share_buffer_idx = (user_idx << 2);

#ifdef KTV_DUMP_ENABLE
	if (ktv_rx_final_info.started)
		ktv_dump_data_save(&ktv_rx_final_info, addr_start_save + writeIdx_save, size_save);
#endif

	return 0;
}


void vivo_audio_ktv_tx_process(uint8_t *addr_start, int32_t readIdx, int32_t size, int32_t buffer_size)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		vivo_audio_ktv_save_tx_data(addr_start, readIdx, size, buffer_size);
	}

}

void vivo_audio_ktv_rx_process(uint8_t *addr_start, int32_t writeIdx, int32_t size, int32_t buffer_size)
{
	if (g_vivo_audio_ktv.vivo_audio_ktv_flag)	{
		vivo_audio_ktv_mixer_tx_to_rx(addr_start, writeIdx, size, buffer_size);
	}

}


static int vivo_audio_ktv_platform_probe(struct platform_device *pdev)
{
	int32_t ret = 0;

	ret = misc_register(&vivo_audio_ktv_dev_device);

	if (ret < 0) {
		printk("%s : register failed %d\n", __func__, ret);
		return ret;
	}

#ifdef KTV_DUMP_ENABLE
	memset(&ktv_rx_final_info, 0, sizeof(ktv_rx_final_info));
	ktv_dump_register_file(&ktv_rx_final_info);
#endif

	printk("%s : register sucess\n", __func__);
	return ret;
}

static int vivo_audio_ktv_platform_remove(struct platform_device *pdev)
{
	misc_deregister(&vivo_audio_ktv_dev_device);
	return 0;
}

static const struct of_device_id vivo_ktv_of_match[] = {
		{ .compatible = "vivo,vivo-audio-ktv",},
		{},
};

static struct platform_driver vivo_ktv_platform_driver = {
	.probe = vivo_audio_ktv_platform_probe,
	.remove = vivo_audio_ktv_platform_remove,
	.driver = {
		.name = "vivo_audio_ktv",
		.owner = THIS_MODULE,
		.of_match_table = vivo_ktv_of_match,
	},
};

static int __init vivo_audio_ktv_init(void)
{
	return platform_driver_register(&vivo_ktv_platform_driver);
}

static void __exit vivo_audio_ktv_exit(void)
{
	platform_driver_unregister(&vivo_ktv_platform_driver);
}

module_init(vivo_audio_ktv_init);
module_exit(vivo_audio_ktv_exit);
MODULE_DESCRIPTION("Vivo ktv module");
MODULE_LICENSE("GPL v2");

