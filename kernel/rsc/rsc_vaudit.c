#include <linux/time.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <net/sock.h>
#include <linux/rtc.h>
#include <linux/proc_fs.h>
#include <linux/vivo_rsc/rsc_internal.h>
#include <linux/vivo_rsc/rsc_vaudit.h>
#include <linux/seq_file.h>
/*#ifdef CONFIG_IRQ_DETECT*/
#include <linux/kernel_stat.h>
/*#endif*/

int getnstimeofday64_nolock(struct timespec64 *ts);

/* kfifo buffer size. item is 4096/sizeof(struct vaudit_kmsg*) = 512*/
#define VAUDIT_FIFO_SIZE	4096
#define VAUDIT_THREAD_NAME	"rsc_vaudit"

static struct task_struct *vaudit_thread;
static struct kfifo vaudit_fifo; /*fifo is full of pointer to struct vaudit_kmsg*/
#define VAUDIT_RECORD_LAST_SIZE (16*sizeof(struct vaudit_kmsg *))
static struct kfifo vaudit_fifo_last; /*last 10 items*/

static spinlock_t vaudit_kfifo_lock;

static bool vaudit_kfifo_init;

#define RSC_SYSCALL_WARNING_TIME (50*1000*1000)
/*ns*/
u32 __read_mostly rsc_syscall_threshold_time = RSC_SYSCALL_WARNING_TIME;
u32 __read_mostly rsc_sysproc_threshold_bufsize = 32*1024;
u32 __read_mostly rsc_sysproc_threshold_filesize = 1*1024*1024;
bool rsc_sysproc_checkonetime = true;
u32 rsc_sysproc_checkmode = RSC_SYSPROCFS_CHECK_SKIP | RSC_SYSPROCFS_CHECK_SPEEPUP | RSC_SYSPROCFS_CHECK_BUFSIZE_SKIP;
#define RSC_SYSPROC_SPEEUP_BUF_SIZE (4*1024)
#define RSC_SYSPROC_SPEEUP_VMALLOC_BUF_SIZE (16*1024)
#define RSC_SYSPROC_SPEEUP_BUF_MAX_SIZE (10*1024*1024)
u32 rsc_sysproc_speedup_buf_size = RSC_SYSPROC_SPEEUP_BUF_SIZE;
u32 rsc_sysproc_speedup_vmalloc_buf_size = RSC_SYSPROC_SPEEUP_VMALLOC_BUF_SIZE;
/*disable epm collect*/
#define RSC_DISABLE_COLLECT_EPM_ONLY 1
#define RSC_DISABLE_COLLECT_ALL 2
int rsc_disable_epm_collect;

#define RSC_VAUDIT_IOWAIT_THRESHOLD (64*1000*1000)//(32*1000*1000)
/*300ms*/
#define RSC_VAUDIT_IOWAIT_BACKTRACE_THRESHOLD (300UL*1000UL*1000UL)
#define RSC_VAUDIT_IOWAIT_PERIOD (10*16*1000*1000)
#define RSC_VAUDIT_IOWAIT_THRESHOLD_PERCENT (50)

#define RSC_VAUDIT_SCHEDWAIT_THRESHOLD (64*1000*1000)//(32*1000*1000)
#define RSC_VAUDIT_SCHEDWAIT_PERIOD (10*16*1000*1000)
#define RSC_VAUDIT_SCHEDWAIT_THRESHOLD_PERCENT (50)

#define RSC_VAUDIT_SCHEDWAIT_INTERRUPT_SLEEP_THRESHOLD (100*1000*1000)

/*
* 20 ms, svp task runnable or TASK_UNINTERRUPTIBLE state more than 20ms
* 50ms is well for Game For Peace
* 20ms is well  For Honour of Kings
*/
#define RSC_VAUDIT_SVP_SCHEDWAIT_THRESHOLD (50*1000*1000)//(32*1000*1000)
/*1000ms*/
#define RSC_VAUDIT_SVP_SCHEDWAIT_PERIOD (500*1000*1000)
/*
* drop 1000ms x 10% = 100ms, about 10fps => 60 x 10% = 6fps ,
* except TASK_INTERRUPTIBLE state, cent.tmgp.sgame maybe sleep 5- 10ms in per frame
* svp task runnable or TASK_UNINTERRUPTIBLE state more than 100ms in 1s period
*/
#define RSC_VAUDIT_SVP_SCHEDWAIT_THRESHOLD_PERCENT (15)

#define RSC_VAUDIT_SVP_SCHEDWAIT_INTERRUPT_SLEEP_THRESHOLD (16*1000*1000)

/*ns*/
unsigned long rsc_vaudit_iowait_threashold = RSC_VAUDIT_IOWAIT_THRESHOLD;

unsigned long rsc_vaudit_backtrace_show_count;
unsigned long rsc_vaudit_iowait_over_count;

/*ns*/
unsigned long rsc_vaudit_backtrace_threshold = RSC_VAUDIT_IOWAIT_BACKTRACE_THRESHOLD;

/*ns*/
unsigned long rsc_vaudit_iowait_period = RSC_VAUDIT_IOWAIT_PERIOD;
u32 rsc_vaudit_iowait_threashold_percent = RSC_VAUDIT_IOWAIT_THRESHOLD_PERCENT;

/*idex 0 normal, idex 1 svp task*/
unsigned long __read_mostly rsc_vaudit_schedwait_threashold[2] = {
	RSC_VAUDIT_SCHEDWAIT_THRESHOLD, RSC_VAUDIT_SVP_SCHEDWAIT_THRESHOLD
};

unsigned long __read_mostly rsc_vaudit_schedwait_period[2] = {
	RSC_VAUDIT_SCHEDWAIT_PERIOD, RSC_VAUDIT_SVP_SCHEDWAIT_PERIOD
};

u32 rsc_vaudit_schedwait_threashold_percent[2] = {
	RSC_VAUDIT_SCHEDWAIT_THRESHOLD_PERCENT, RSC_VAUDIT_SVP_SCHEDWAIT_THRESHOLD_PERCENT
};

unsigned long __read_mostly rsc_vaudit_schedwait_interruptsleep_threashold[2] = {
	RSC_VAUDIT_SCHEDWAIT_INTERRUPT_SLEEP_THRESHOLD, RSC_VAUDIT_SVP_SCHEDWAIT_INTERRUPT_SLEEP_THRESHOLD
};

unsigned long __read_mostly rsc_vaudit_app_launch_stime;

u32 rsc_vaudit_svp_thread = RSC_VAUDIT_SVP_THREAD_ALL;

u32 rsc_vaudit_num;
u32 rsc_vaudit_failnum;

#define RSC_VAUDIT_NAME vaudit

atomic_t rsc_vaudit_gb_idx;
/*
 * Function: is_channel_valid
 * Description: check channel valid or not
 * Input:	@channel - channel to check
 * Output:
 * Return:	false -- invalid
 *			true -- valid
*/
static inline bool is_channel_valid(int channel)
{
	return (channel > CHANNEL_ID_NONE && channel < CHANNEL_ID_END);
}

/*
 * Function: is_tag_valid
 * Description: check packet tag valid or not
 * Input:	@tag - tag to check
 * Output:
 * Return:	false -- invalid
 *			true -- valid
*/
static inline bool is_tag_valid(int tag)
{
	return (tag > PACKET_TAG_NONE && tag < PACKET_TAG_END);
}


struct completion rsc_vaudit_complete;

/*
 * Function: fifo_out
 * Description: receive data from module and process it here
 * Input:	@data - source data
 * Output:
 * Return:	void
*/
static int fifo_out(void *data)
{
	while (!kthread_should_stop()) {
		while (wait_for_completion_interruptible(&rsc_vaudit_complete) != 0)
			;

		reinit_completion(&rsc_vaudit_complete);
		if (!kfifo_is_empty(&vaudit_fifo))
			sysfs_notify(rsc_root_dir, NULL, __stringify(RSC_VAUDIT_NAME));
	}

	return 0;
}

static int fifo_in(struct vaudit_kmsg *msg)
{
	int ret = -1;
	unsigned long flags;

	if (IS_ERR_OR_NULL(msg)) {
		rsc_err("vaudit %s: the msg point is err!\n", __func__);
		goto err;
	}
	if (unlikely(!vaudit_kfifo_init)) {
		rsc_err("vaudit %s: vaudit_kfifo_init not init\n", __func__);
		kfree(msg);
		return ret;
	}
		
	/*put data in kfifo*/
	spin_lock_irqsave(&vaudit_kfifo_lock, flags);
	if (0 == kfifo_in(&vaudit_fifo, &msg, sizeof(msg))) {
		rsc_vaudit_failnum++;
		complete(&rsc_vaudit_complete);
		/*put data failed*/
		if (kfifo_is_full(&vaudit_fifo)) {
			printk_ratelimited("vaudit %s: send msg to user failed!\n", __func__);
			kfree(msg);
		} else {
			printk_ratelimited("vaudit %s: kfifo in error!\n", __func__);
			kfree(msg);
		}
	} else {
		/*put data successfully*/
		rsc_vaudit_num++;
		ret = 0;
		complete(&rsc_vaudit_complete);
	}
	spin_unlock_irqrestore(&vaudit_kfifo_lock, flags);

err:
	return ret;
}

static DEFINE_MUTEX(rsc_fifo_show);

/*
 * Function: fifo_out
 * Description: receive data from module and process it here
 * Input:	@data - source data
 * Output:
 * Return:	void
*/
static int vaudit_show(struct seq_file *s, void *data)
{
	struct vaudit_kmsg *msg = NULL, *lastmsg;
	int i;
	struct kfifo *fifo;
	struct rtc_time l_tm;
	char *buf;
	unsigned long flags;

	if (unlikely(rsc_disable_epm_collect)) {
		if ((RSC_DISABLE_COLLECT_EPM_ONLY == rsc_disable_epm_collect)  && (current_uid().val != 0))
			return 0;

		if (RSC_DISABLE_COLLECT_ALL == rsc_disable_epm_collect)
			return 0;
	}

	mutex_lock(&rsc_fifo_show);
	fifo = &vaudit_fifo;

	if (kfifo_len(fifo)) {
		seq_printf(s, "msgnum:\t%u\ttotalnum\t%u\t%u\n",
			(u32)(kfifo_len(fifo)/sizeof(struct vaudit_kmsg *)), rsc_vaudit_num, rsc_vaudit_failnum);

		seq_printf(s, "IDX \tTYPE\tSUBT\t"
					/*"overval\t"*/
					"TimeOccurrence\t      msg\n");

		i = 0;
		while (0 != kfifo_out(fifo, &msg, sizeof(struct vaudit_kmsg *))) {
			/*fifo has data*/
			struct vaudit_info *info;
			info = (struct vaudit_info *)msg->buffer;

			rtc_time_to_tm((unsigned long)info->tv_sec-sys_tz.tz_minuteswest * 60, &l_tm);
			buf = s->buf + s->count;
			seq_printf(s, "%4d\t%4x\t%4d\t"
				/*"%7u\t"*/
				"%04d%02d%02d%02d%02d%02d\t%s\n",
				info->idx, info->type, info->subtype,/* info->overval,*/
				l_tm.tm_year+1900, l_tm.tm_mon+1, l_tm.tm_mday, l_tm.tm_hour, l_tm.tm_min, l_tm.tm_sec,
				info->buffer);
			if (seq_has_overflowed(s)) {
				int resent = 1;
				/*putback data in kfifo*/
				spin_lock_irqsave(&vaudit_kfifo_lock, flags);
				if (0 == kfifo_in(&vaudit_fifo, &msg, sizeof(msg))) {
					resent = 0;
					if (kfifo_is_full(&vaudit_fifo)) {
						rsc_err("vaudit %s: fifo putback error - fifofull!\n", __func__);
						kfree(msg);
					} else {
						rsc_err("vaudit %s: fifo putback error!\n", __func__);
						kfree(msg);
					}
				} else {
					/*
						BUG: scheduling while atomic: sysfs_notify/9220/0x00000002
						Modules linked in: wlan(O) vivo_tfa9894_dlkm(O) vivo_tas2560_dlkm(O) vivo_aw87319_dlkm(O) vivo_ak4377a_dlkm(O) vivo_cs43130_dlkm(O) machine_dlkm(O) wcd934x_dlkm(O) mbhc_dlkm(O) vivo_ia6xx_dlkm(O) vivo_codec_common_dlkm(O) wcd9360_dlkm(O) swr_ctrl_dlkm(O) wcd9xxx_dlkm(O) wsa881x_dlkm(O) wcd_core_dlkm(O) stub_dlkm(O) wcd_spi_dlkm(O) hdmi_dlkm(O) swr_dlkm(O) pinctrl_wcd_dlkm(O) usf_dlkm(O) native_dlkm(O) platform_dlkm(O) q6_dlkm(O) adsp_loader_dlkm(O) apr_dlkm(O) q6_notifier_dlkm(O) q6_pdr_dlkm(O) wglink_dlkm(O) msm_11ad_proxy
						CPU: 4 PID: 9220 Comm: sysfs_notify Tainted: G S   U  W  O	  4.14.83-perf+ #489
						Hardware name: Qualcomm Technologies, Inc. SM8150 V2 PM8150 MTP PD1824 (DT)
						Call trace:
						dump_backtrace+0x0/0x138
						show_stack+0x18/0x20
						dump_stack+0xc4/0x100
						__schedule_bug+0x50/0x70
						__schedule+0xa68/0x1458
						schedule_preempt_disabled+0x7c/0xa8
						__mutex_lock+0x7a8/0xa58
						__mutex_lock_slowpath+0x10/0x18
						mutex_lock+0x30/0x38
						kernfs_find_and_get_ns+0x28/0x88
						sysfs_notify+0x44/0x88
						show_rsc_vaudit+0x260/0x318
						kobj_attr_show+0x14/0x28
						sysfs_kf_seq_show+0x8c/0x108
						kernfs_seq_show+0x28/0x30
						seq_read+0x1b4/0x680
						kernfs_fop_read+0x5c/0x1a8
						__vfs_read+0x44/0x130
						vfs_read+0xa0/0x138
						SyS_read+0x54/0xb8
						el0_irq_naked+0x1f0/0x159c
						could not call sysfs_notify in here.
						sysfs_notify(rsc_root_dir, NULL, __stringify(RSC_VAUDIT_NAME));
					*/
					rsc_info("vaudit %s: fifo putback ok!\n", __func__);
				}
				spin_unlock_irqrestore(&vaudit_kfifo_lock, flags);
				if (resent)
					sysfs_notify(rsc_root_dir, NULL, __stringify(RSC_VAUDIT_NAME));
				break;
			}
			rsc_dbg(RSC_VAUDIT, "vaudit proc %s\n", buf);
			i++;

			if (kfifo_is_full(&vaudit_fifo_last)) {
				if (0 != kfifo_out(&vaudit_fifo_last, &lastmsg, sizeof(struct vaudit_kmsg *)))
					kfree(lastmsg);
				else
					rsc_err("vaudit %s: fifo out last error!\n", __func__);
			}
			if (0 == kfifo_in(&vaudit_fifo_last, &msg, sizeof(msg))) {
				rsc_err("vaudit %s: fifo in last error!\n", __func__);
				kfree(msg);
			}
		}
	}
	mutex_unlock(&rsc_fifo_show);

	return 0;
}

static ssize_t vaudit_write(struct file *file, const char *buffer, size_t count, loff_t *off)
{
	int ret, flush;
	char kbuf[64];
	struct vaudit_kmsg *msg = NULL;

	if (count >= sizeof(kbuf))
		count = sizeof(kbuf) - 1;

	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	ret = sscanf(kbuf, "%d", &flush);
	if ((ret != 1) && (flush != 1))
		return -EINVAL;

	rsc_info("%s vaudit flush fifo\n", __func__);

	while (0 != kfifo_out(&vaudit_fifo, &msg, sizeof(struct vaudit_kmsg *)))
		kfree(msg);

	return count;
}

static int vaudit_open(struct inode *inode, struct file *file)
{
	size_t size;
	int ret;

	size = kfifo_len(&vaudit_fifo)/sizeof(struct vaudit_kmsg *) * 300;
	if (size < 8 * 1024)
		size = 8 * 1024;

	ret = single_open_size(file, vaudit_show, PDE_DATA(inode), size);
#ifdef CONFIG_RSC_VAUDIT
	if (!ret)
		((struct seq_file *)file->private_data)->rsc_check = RSC_SYSPROCFS_CHECK_SKIP;
#endif
	return ret;
}

static const struct file_operations vaudit_operations = {
	.owner = THIS_MODULE,
	.open = vaudit_open,	
	.write = vaudit_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t show_rsc_vaudit(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct vaudit_kmsg *msg, *lastmsg;
	int ret = 0;
	int i, cur;
	struct rtc_time l_tm;
	unsigned long flags;

	if (unlikely(rsc_disable_epm_collect)) {
		if ((RSC_DISABLE_COLLECT_EPM_ONLY == rsc_disable_epm_collect)  && (current_uid().val != 0))
			return 0;

		if (RSC_DISABLE_COLLECT_ALL == rsc_disable_epm_collect)
			return 0;
	}

	i = 0;
	mutex_lock(&rsc_fifo_show);

	if (kfifo_len(&vaudit_fifo)) {
		cur = snprintf(buf + ret, PAGE_SIZE - ret,  "msgnum:\t%u\ttotalnum\t%u\t%u\n",
			(u32)(kfifo_len(&vaudit_fifo)/sizeof(struct vaudit_kmsg *)), rsc_vaudit_num, rsc_vaudit_failnum);
		ret += cur;

		cur = snprintf(buf + ret, PAGE_SIZE - ret, "IDX \tTYPE\tSUBT\t"
													/*"overval\t"*/
													"TimeOccurrence\t      msg\n");
		ret += cur;
		/*
			PD1824:/ # cat /sys/rsc/vaudit
			rsc_syscall_threshold_time: 50000000 ns fifo_debug: 1 msgnum: 9
		   1    TYPE       1    cost      56972 us      syscall: 172 running too long   56972 us uid  1046  android.hardwar   901 <-  android.hardwar   901
		   2    TYPE       1    cost     112443 us      syscall: 167 running too long  112443 us uid  1040   mediaextractor   891 <-   mediaextractor   891
		   3    TYPE       1    cost     151840 us      syscall:  63 running too long  151840 us uid  1000    QseeLogThread  6306 <-  .android.bbklog  6109
		   4    TYPE       1    cost      50971 us      syscall:  63 running too long   50971 us uid  1000    QseeLogThread  6306 <-  .android.bbklog  6109
		   5    TYPE       1    cost    4150899 us      syscall:  63 running too long 4150899 us uid  1000      TzLogThread  6307 <-  .android.bbklog  6109
		   6    TYPE       1    cost    5453546 us      syscall:  63 running too long 5453546 us uid  1000      TzLogThread  6307 <-  .android.bbklog  6109
		   7    TYPE       1    cost     206111 us      syscall:  63 running too long  206111 us uid  1000    QseeLogThread  6306 <-  .android.bbklog  6109
		   8    TYPE       1    cost    1475276 us      syscall:  63 running too long 1475276 us uid  1000      TzLogThread  6307 <-  .android.bbklog  6109
		   9    TYPE       1    cost      51099 us      syscall:  63 running too long   51099 us uid  1000    system_server 11019 <-    system_server 11019
		*/
		while (0 != kfifo_out(&vaudit_fifo, &msg, sizeof(struct vaudit_kmsg *))) {
			/*fifo has data*/
			struct vaudit_info *info;
			info = (struct vaudit_info *)msg->buffer;

			rtc_time_to_tm((unsigned long)info->tv_sec-sys_tz.tz_minuteswest * 60, &l_tm);
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%4d\t%4x\t%4d\t"
														/*"%7u\t"*/
														"%04d%02d%02d%02d%02d%02d\t%s\n",
				info->idx, info->type, info->subtype,/* info->overval,*/
				l_tm.tm_year+1900, l_tm.tm_mon+1, l_tm.tm_mday, l_tm.tm_hour, l_tm.tm_min, l_tm.tm_sec,
				info->buffer);
			if (((cur+1) >= (PAGE_SIZE-ret)) || (cur <= 0)) {
				int resent = 1;
				/*putback data in kfifo*/
				spin_lock_irqsave(&vaudit_kfifo_lock, flags);
				if (0 == kfifo_in(&vaudit_fifo, &msg, sizeof(msg))) {
					resent = 0;
					if (kfifo_is_full(&vaudit_fifo)) {
						rsc_err("vaudit %s: fifo putback error - fifofull!\n", __func__);
						kfree(msg);
					} else {
						rsc_err("vaudit %s: fifo putback error!\n", __func__);
						kfree(msg);
					}
				} else {
/*
					BUG: scheduling while atomic: sysfs_notify/9220/0x00000002
					Modules linked in: wlan(O) vivo_tfa9894_dlkm(O) vivo_tas2560_dlkm(O) vivo_aw87319_dlkm(O) vivo_ak4377a_dlkm(O) vivo_cs43130_dlkm(O) machine_dlkm(O) wcd934x_dlkm(O) mbhc_dlkm(O) vivo_ia6xx_dlkm(O) vivo_codec_common_dlkm(O) wcd9360_dlkm(O) swr_ctrl_dlkm(O) wcd9xxx_dlkm(O) wsa881x_dlkm(O) wcd_core_dlkm(O) stub_dlkm(O) wcd_spi_dlkm(O) hdmi_dlkm(O) swr_dlkm(O) pinctrl_wcd_dlkm(O) usf_dlkm(O) native_dlkm(O) platform_dlkm(O) q6_dlkm(O) adsp_loader_dlkm(O) apr_dlkm(O) q6_notifier_dlkm(O) q6_pdr_dlkm(O) wglink_dlkm(O) msm_11ad_proxy
					CPU: 4 PID: 9220 Comm: sysfs_notify Tainted: G S   U  W  O	  4.14.83-perf+ #489
					Hardware name: Qualcomm Technologies, Inc. SM8150 V2 PM8150 MTP PD1824 (DT)
					Call trace:
					dump_backtrace+0x0/0x138
					show_stack+0x18/0x20
					dump_stack+0xc4/0x100
					__schedule_bug+0x50/0x70
					__schedule+0xa68/0x1458
					schedule_preempt_disabled+0x7c/0xa8
					__mutex_lock+0x7a8/0xa58
					__mutex_lock_slowpath+0x10/0x18
					mutex_lock+0x30/0x38
					kernfs_find_and_get_ns+0x28/0x88
					sysfs_notify+0x44/0x88
					show_rsc_vaudit+0x260/0x318
					kobj_attr_show+0x14/0x28
					sysfs_kf_seq_show+0x8c/0x108
					kernfs_seq_show+0x28/0x30
					seq_read+0x1b4/0x680
					kernfs_fop_read+0x5c/0x1a8
					__vfs_read+0x44/0x130
					vfs_read+0xa0/0x138
					SyS_read+0x54/0xb8
					el0_irq_naked+0x1f0/0x159c
					could not call sysfs_notify in here.
					sysfs_notify(rsc_root_dir, NULL, __stringify(RSC_VAUDIT_NAME));
*/
					rsc_info("vaudit %s: fifo putback ok! cur: %d leftbuf: %d ret: %d\n", __func__, cur, (int)(PAGE_SIZE-ret), ret);
				}
				spin_unlock_irqrestore(&vaudit_kfifo_lock, flags);
				if (resent)
					sysfs_notify(rsc_root_dir, NULL, __stringify(RSC_VAUDIT_NAME));
				break;
			}

			rsc_dbg(RSC_VAUDIT, "vaudit sys %s\n", buf + ret);
			ret += cur;
			i++;
			/*
			* put msg backto last fifo for debug,
			* you can cat /sys/rsc/vaudit_switch to get the last 16 fifo items.
			*/
			if (kfifo_is_full(&vaudit_fifo_last)) {
				if (0 != kfifo_out(&vaudit_fifo_last, &lastmsg, sizeof(struct vaudit_kmsg *))) {
					/*printk("vaudit free lastmsg\n");*/
					kfree(lastmsg);
				} else
					rsc_err("vaudit %s: fifo out last error!\n", __func__);
			}
			if (0 == kfifo_in(&vaudit_fifo_last, &msg, sizeof(msg))) {
				rsc_err("vaudit %s: fifo in last error!\n", __func__);
				kfree(msg);
			}
		}
	}
#if 0
	if (!ret) {
		cur = snprintf(buf, PAGE_SIZE, "There is not any vaudit data.\n");
			ret += cur;
	}
#endif
	mutex_unlock(&rsc_fifo_show);

	return ret;
}

static ssize_t store_rsc_vaudit(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, flush, collect;
	struct vaudit_kmsg *msg = NULL;
	
	ret = sscanf(buf, "%d %d", &collect, &flush);
	if ((ret < 1) || (ret > 2))
		return -EINVAL;

	rsc_info("%s vaudit rsc_disable_epm_collect %d -> %d flush%d fifo\n", __func__,
		rsc_disable_epm_collect, collect, ret > 1?flush:0);

	rsc_disable_epm_collect = collect;
	if ((ret > 1) && flush) {
		while (0 != kfifo_out(&vaudit_fifo, &msg, sizeof(struct vaudit_kmsg *)))
			kfree(msg);
	}

	return count;
}
static struct kobj_attribute rsc_vaudit_attr =
__ATTR(RSC_VAUDIT_NAME, 0660, show_rsc_vaudit, store_rsc_vaudit);

#define RSC_VAUDIT_DEFAULT_RATELIMIT_INTERVAL	(5 * HZ)
#define RSC_VAUDIT_DEFAULT_RATELIMIT_BURST		150
static DEFINE_RATELIMIT_STATE(rsc_vaudit_rs, RSC_VAUDIT_DEFAULT_RATELIMIT_INTERVAL,
				  RSC_VAUDIT_DEFAULT_RATELIMIT_BURST);

atomic_t  rsc_ratelimit_cnt;

static ssize_t show_rsc_syscheck(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	int cur;

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "tnum\t%u\t%u\tsyscallth\t%u\tus\t"
		"sysproc\t%u\t%u\tonetime\t%d\tchkmode\t%x\t"
		"speedbuf\t%u\t%u\tvaudit_svp: %x\trs: %u %u ov %u\n"
		"disable audit svp: echo 1000 2 0 > /sys/rsc/vaudit_syscheck\n",
		rsc_vaudit_num, rsc_vaudit_failnum, rsc_syscall_threshold_time/1000,
		rsc_sysproc_threshold_bufsize, rsc_sysproc_threshold_filesize,
		rsc_sysproc_checkonetime, rsc_sysproc_checkmode,
		rsc_sysproc_speedup_buf_size, rsc_sysproc_speedup_vmalloc_buf_size,
		rsc_vaudit_svp_thread,
		rsc_vaudit_rs.burst, (u32)jiffies_to_msecs(rsc_vaudit_rs.interval), atomic_read(&rsc_ratelimit_cnt)
		);
	ret += cur;

	ret = rsc_syscall_check(buf, ret);

	return ret;
}

static ssize_t store_rsc_syscheck(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int type;
	/* if val1 == -1 ,then not set*/
	int val1 = 999, val2 = 999, val3 = 999, val4 = 999, val5 = 999, val6 = 999;;

	ret = sscanf(buf, "%x %d %d %d %d %d %d", &type, &val1, &val2, &val3, &val4, &val5, &val6);
	if (ret < 2)
		return -EINVAL;

	if (type == VAUDIT_SYSCALL) {
		if (ret != 2)
			return -EINVAL;
		rsc_info("set syscall time %u -> %u us buf: %s\n",
			rsc_syscall_threshold_time/1000, val1, buf);
		rsc_syscall_threshold_time = val1 * 1000;
	} else if (type == VAUDIT_SYSPROCFS) {
		if (ret > 7)
			return -EINVAL;
		rsc_info("set sysproc ret %d bufsize %u -> %u "
				"filesize %u -> %u "
				"onetime %u -> %u chekmode %x -> %x "
				"speedbuf %u -> %u %u -> %u buf: %s\n",
			ret, rsc_sysproc_threshold_bufsize, val1,
			rsc_sysproc_threshold_filesize, val2,
			rsc_sysproc_checkonetime, val3,
			rsc_sysproc_checkmode,
			(val4 & (RSC_SYSPROCFS_CHECK_SKIP | RSC_SYSPROCFS_CHECK_SPEEPUP | RSC_SYSPROCFS_CHECK_BUFSIZE_SKIP)),
			rsc_sysproc_speedup_buf_size, val5,
			rsc_sysproc_speedup_vmalloc_buf_size, val6,
			buf);
		/*size at least PAGE_SIZE+1, avoid deadlock, threadName='FileObserver',threadId=7068,cmdline='com.vivo.epm',pid=7027,*/
		if (ret >= 2 && (val1 != -1))
			rsc_sysproc_threshold_bufsize = max(val1, (int)PAGE_SIZE+1);
		if (ret >= 3 && (val2 != -1))
			rsc_sysproc_threshold_filesize = max(val2, (int)PAGE_SIZE+1);
		if (ret >= 4)
			rsc_sysproc_checkonetime = val3;
		if (ret >= 5)
			rsc_sysproc_checkmode =
				(val4 & (RSC_SYSPROCFS_CHECK_SKIP | RSC_SYSPROCFS_CHECK_SPEEPUP | RSC_SYSPROCFS_CHECK_BUFSIZE_SKIP));
		if (ret >= 6 && (val5 != -1) && val5 <= RSC_SYSPROC_SPEEUP_BUF_MAX_SIZE)
			rsc_sysproc_speedup_buf_size = val5;
		if (ret >= 7 && (val6 != -1) && val6 <= RSC_SYSPROC_SPEEUP_BUF_MAX_SIZE)
			rsc_sysproc_speedup_vmalloc_buf_size = val6;
	} else if (type == VAUDIT_SCHEDWAIT) {
		if (ret < 3)
			return -EINVAL;
		/* echo 0x1000 2 1 > /sys/rsc/vaudit_syscheck*/
		if (val1 == VAUDIT_SCHEDWAIT_SVP_SCHED)
			rsc_vaudit_svp_thread = val2;
	} else if (type == VAUDIT_SET_COSTTIME_IDX) {
		/*
		* example: set rsc_vaudit_rs max 600 times in 10000ms.
		* echo 0x40000000 600 10000 > /sys/rsc/vaudit_syscheck
		*/
		if (ret < 3)
			return -EINVAL;
		rsc_vaudit_rs.burst = val1;
		/*ms to jiffies*/
		rsc_vaudit_rs.interval = msecs_to_jiffies(val2);
	} else
		return -EINVAL;

	return count;
}

static struct kobj_attribute rsc_syscall_attr =
__ATTR(vaudit_syscheck, 0660, show_rsc_syscheck, store_rsc_syscheck);


static DEFINE_MUTEX(vaudit_switch_lock);

#undef VAUDIT_FEAT
#define VAUDIT_FEAT(name, enabled)														\
/*	(																				\
	do {	(*/ 																	\
	(1UL << VAUDIT_FEAT_##name) * enabled |											\
/*		) } while (0)																\
	) */

#if 1
unsigned int rsc_vaudit_switch = 
#include <linux/vivo_rsc/vaudit_features.h>
	0;
#else
unsigned int rsc_vaudit_switch;
#endif

#undef VAUDIT_FEAT
#define VAUDIT_FEAT(name, enabled)	\
/*	(																				\
		do {	(*/ 																\
	#name,																			\
/*		) } while (0)																\
	) */

static const char * const vaudit_feat_names[] = {
#include <linux/vivo_rsc/vaudit_features.h>
};
#undef VAUDIT_FEAT

static ssize_t show_vaudit_switch(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int cur;
	int i;
	int num;
	struct vaudit_kmsg *msg;
	struct rtc_time l_tm;
	struct vaudit_info *info;

	for (i = 0; i < VAUDIT_FEAT_NR; i++) {
		if (!(rsc_vaudit_switch & (1UL << i))) {
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "NO_");
			ret += cur;
		}
		cur = snprintf(buf + ret, PAGE_SIZE - ret, "%s %lx\t", vaudit_feat_names[i], (1UL << i));
		ret += cur;
	}

	cur = snprintf(buf + ret, PAGE_SIZE - ret, "\nexample, set or clear bit1:\n"
								"setswt: echo        0x1       > /sys/rsc/vaudit_switch\n"
								" clear: echo 0x80000001       > /sys/rsc/vaudit_switch\n"
								"setval: echo 0x40000001 0 5000 > /sys/rsc/vaudit_switch\n"
								"        echo 0x40001000 2 20000 1000000 10 > /sys/rsc/vaudit_switch\n"
								"        echo 0x40001000 0 500 -1 -1 1000 > /sys/rsc/vaudit_switch\n"
								);
	ret += cur;

	mutex_lock(&rsc_fifo_show);
	cur = snprintf(buf + ret, PAGE_SIZE - ret, "0x%x\ttotalnum\t%u\t%u\tepm_disable%d\tsyscallth\t%u us\t"
		"iowait\t%lu\t%lu\t%u\t"
		"btthr\t%lu\tioover\t%lu\theavyio\t%lu\t"
		"schedwait\t%lu\t%lu\t%u\t%lu\tsvp\t%lu\t%lu\t%u\t%lu\t"
#ifdef CONFIG_IRQ_DETECT
		"irqth\t%u\t"
#endif
		"fifolen\t%d\tlastlen\t%d\n",
		rsc_vaudit_switch, rsc_vaudit_num,
		rsc_vaudit_failnum, rsc_disable_epm_collect, rsc_syscall_threshold_time/1000,
		rsc_vaudit_iowait_threashold/1000, rsc_vaudit_iowait_period/1000, rsc_vaudit_iowait_threashold_percent,
		rsc_vaudit_backtrace_threshold/1000, rsc_vaudit_iowait_over_count, rsc_vaudit_backtrace_show_count,
		rsc_vaudit_schedwait_threashold[0]/1000, rsc_vaudit_schedwait_period[0]/1000, rsc_vaudit_schedwait_threashold_percent[0],
		rsc_vaudit_schedwait_interruptsleep_threashold[0],
		rsc_vaudit_schedwait_threashold[1]/1000, rsc_vaudit_schedwait_period[1]/1000, rsc_vaudit_schedwait_threashold_percent[1],
		rsc_vaudit_schedwait_interruptsleep_threashold[1],
	#ifdef CONFIG_IRQ_DETECT
		get_irq_threshold(),
	#endif
		(int)(kfifo_len(&vaudit_fifo)/sizeof(struct vaudit_kmsg *)),
		(int)(kfifo_len(&vaudit_fifo_last)/sizeof(struct vaudit_kmsg *)));
	ret += cur;

	num = kfifo_len(&vaudit_fifo_last)/sizeof(struct vaudit_kmsg *);
	for (i = 0; i < num ; i++) {
		if (0 != kfifo_out(&vaudit_fifo_last, &msg, sizeof(struct vaudit_kmsg *))) {
			info = (struct vaudit_info *)msg->buffer;
			rtc_time_to_tm((unsigned long)info->tv_sec-sys_tz.tz_minuteswest * 60, &l_tm);
			cur = snprintf(buf + ret, PAGE_SIZE - ret, "%4d\t%4x\t%4d\t%7u\t%04d%02d%02d%02d%02d%02d\t%s\n",
					info->idx, info->type, info->subtype, info->overval,
					l_tm.tm_year+1900, l_tm.tm_mon+1, l_tm.tm_mday, l_tm.tm_hour, l_tm.tm_min, l_tm.tm_sec,
					info->buffer);
			if (0 == kfifo_in(&vaudit_fifo_last, &msg, sizeof(msg))) {
				rsc_err("vaudit %s: fifo in last error!\n", __func__);
				kfree(msg);
			}
			if (cur <= 0)
				break;
			ret += cur;
		}
	}
	mutex_unlock(&rsc_fifo_show);

	return ret;
}

static ssize_t store_vaudit_switch(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int value, tmp, subtype;
	/* if time == -1 ,then not set*/
	long time = 0, period = 0, percent = 0, intsleeptime = 0;
	unsigned int oldval;
	bool setval = false;

	ret = sscanf(buf, "%x %d %ld %ld %ld %ld",
		&value, &subtype, &time, &period, &percent, &intsleeptime);
	if (ret < 1)
		return -EINVAL;

	tmp = value & (~((1 << VAUDIT_SWITCH_CLEAR) | (1 << VAUDIT_SET_COSTTIME)));
	if (value & (1 << VAUDIT_SET_COSTTIME)) {
		if ((tmp == VAUDIT_SYSCALL) && ret != 3)
			return -EINVAL;
		if ((tmp == VAUDIT_SCHEDWAIT) && ret < 3)
			return -EINVAL;
		if ((tmp == VAUDIT_INTERRUPT) && ret != 3)
			return -EINVAL;
		setval = true;
	}
	mutex_lock(&vaudit_switch_lock);
	oldval = rsc_vaudit_switch;
	if (value & (1 << VAUDIT_SWITCH_CLEAR))
		rsc_vaudit_switch &= ~value;
	else if (!setval) {
		rsc_vaudit_switch |= value;
	}

	if (setval) {
		/*tmp = value & (~((1 << VAUDIT_SWITCH_CLEAR) | (1 << VAUDIT_SET_COSTTIME)));*/
		if (hweight32(tmp) > 1)
			rsc_err("%s vaudit_switch setval error! %x -> %x val: %x tmp: %x time: %ld buf: %s\n",
			(value & (1 << VAUDIT_SWITCH_CLEAR))?"clear":"set",
			oldval, rsc_vaudit_switch, value, tmp, setval?time:0UL, buf);
		else if ((tmp & (1UL << VAUDIT_FEAT_SYSCALL)) && (time != -1))
			rsc_syscall_threshold_time = time * 1000;
		else if (tmp == VAUDIT_SCHEDWAIT) {
			if (subtype == VAUDIT_SCHEDWAIT_IO) {
				if (ret >= 3 && time != -1)
					rsc_vaudit_iowait_threashold = time*1000;
				if (ret >= 4 && period != -1)
					rsc_vaudit_iowait_period = period*1000;
				if (ret >= 5 && percent != -1)
					rsc_vaudit_iowait_threashold_percent = percent;
				if (ret >= 6 && intsleeptime != -1)
					rsc_vaudit_backtrace_threshold = intsleeptime*1000;
			} else if (subtype == VAUDIT_SCHEDWAIT_NORMAL_SCHED) {
				if (ret >= 3 && time != -1)
					rsc_vaudit_schedwait_threashold[0] = time*1000;
				if (ret >= 4 && period != -1)
					rsc_vaudit_schedwait_period[0] = period*1000;
				if (ret >= 5 && percent != -1)
					rsc_vaudit_schedwait_threashold_percent[0] = percent;
				if (ret >= 6 && intsleeptime != -1)
					rsc_vaudit_schedwait_interruptsleep_threashold[0] = intsleeptime;
			} else if (subtype == VAUDIT_SCHEDWAIT_SVP_SCHED) {
				if (ret >= 3 && time != -1)
					rsc_vaudit_schedwait_threashold[1] = time*1000;
				if (ret >= 4 && period != -1)
					rsc_vaudit_schedwait_period[1] = period*1000;
				if (ret >= 5 && percent != -1)
					rsc_vaudit_schedwait_threashold_percent[1] = percent;
				if (ret >= 6 && intsleeptime != -1)
					rsc_vaudit_schedwait_interruptsleep_threashold[1] = intsleeptime;
			}
#ifdef CONFIG_IRQ_DETECT
		} else if (tmp == VAUDIT_INTERRUPT) {
			set_irq_threshold(time);
#endif
		}
	}

	rsc_info("%s vaudit_switch %x -> %x val: %x setval: %ld %ld %ld us buf: %s\n",
		(value & (1 << VAUDIT_SWITCH_CLEAR))?"clear":"set",
		oldval, rsc_vaudit_switch, value, setval?time:0UL, period, percent, buf);

	mutex_unlock(&vaudit_switch_lock);

	return count;
}

static struct kobj_attribute vaudit_switch_attr =
__ATTR(vaudit_switch, 0660, show_vaudit_switch, store_vaudit_switch);

/*
 * Function: vaudit
 * Description: kernel information entry function
 * Input:	@dst - user to send
 *			@tag - packet tag
 *			@data - source data
 *			@len - data len
 * Output:
 * Return:	-1 -- failed
 *			0 -- successed
*/
static int vaudit(CHANNEL_ID dst, PACKET_TAG tag, const char *data, size_t len)
{
	struct vaudit_kmsg *msg = NULL;
	int msg_len = 0;

	if (!is_channel_valid(dst)) {
		rsc_err("vaudit %s: channel is invalid!\n", __func__);
		return -EINVAL;
	}

	if (!is_tag_valid(tag)) {
		rsc_err("vaudit %s: tag is invalid!\n", __func__);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(data)) {
		rsc_err("vaudit %s: data is NULL!\n", __func__);
		return -EINVAL;
	}

	/*msg will be freed in fifo_in() or send_to_user()*/
	msg_len = sizeof(struct vaudit_kmsg) + len;
	msg = (struct vaudit_kmsg *)kmalloc(msg_len, GFP_ATOMIC);
	if (IS_ERR_OR_NULL(msg)) {
		rsc_err("vaudit %s: msg is NULL!\n", __func__);
		return -ENOMEM;
	}
	/* not need to do memset */
	/*memset(msg, 0, msg_len);*/
	msg->tag = tag;
	msg->src = CHANNEL_ID_NETLINK;
	msg->dst = dst;
	msg->length = len;
	memcpy(msg->buffer, data, len);

	if (fifo_in(msg) < 0) {
		/*rsc_err("vaudit %s: save msg from module failed!\n", __func__);*/
		return -ENOSPC;
	}
	return 0;
}

/* remove spinlock for speed up*/
static inline int rsc_vaudit_ratelimit(struct ratelimit_state *rs, const char *func)
{
/*
	unsigned long flags;
*/
	int ret;

/*
	if (!rs->interval)
		return 1;
*/

	/*
	 * If we contend on this state's lock then almost
	 * by definition we are too busy to print a message,
	 * in addition to the one that will be printed by
	 * the entity that is holding the lock already:
	 */
/*
	if (!raw_spin_trylock_irqsave(&rs->lock, flags))
		return 0;

	if (unlikely(!rs->begin))
		rs->begin = jiffies;
*/

	if (time_is_before_eq_jiffies(rs->begin + rs->interval)) {
		if (rs->missed) {
			/*if (!(rs->flags & RATELIMIT_MSG_ON_RELEASE)) {*/
				printk_deferred(KERN_WARNING
						"vaudit %s: %d callbacks suppressed\n",
						func, rs->missed);
				rs->missed = 0;
			/*}*/
		}
		rs->begin = jiffies;
		rs->printed = 0;
	}
	if (rs->burst && rs->burst > rs->printed) {
		rs->printed++;
		ret = 1;
	} else {
		if (unlikely(!rs->interval))
			return 1;
		rs->missed++;
		ret = 0;
	}
/*
	raw_spin_unlock_irqrestore(&rs->lock, flags);
*/
	return ret;
}

/*
  * Function: vsend
  * Description: Packet and Send data to userspace by vaudit
  * Input: type -- message type
  *        fmt -- string
  * Return: -1--failed, 0--success
**/
static int vsend(u32 type, u32 subtype, u32 overval, va_list args, const char *fmt)
{
	int ret  = -1;
	struct vaudit_info info;
	int length = 0;
	size_t info_len = 0;
	struct timespec64 dtime;

	memset(&info, 0, sizeof(info));
	length = vscnprintf(info.buffer, VAUDIT_BUFFER_SIZE - 1, fmt, args);
	if (length > 0) {
		info.type = type;
		info.subtype = subtype;
		info.len = length + 1;
		getnstimeofday64_nolock(&dtime);
		info.tv_sec = (u32)dtime.tv_sec;
		/*info.tv_usec = 0;*/
		info.overval = overval;
		info.idx = atomic_inc_return(&rsc_vaudit_gb_idx);
		info_len = sizeof(info) - VAUDIT_BUFFER_SIZE + length + 1;
		if (unlikely(!rsc_vaudit_ratelimit(&rsc_vaudit_rs, __func__))) {
			atomic_inc(&rsc_ratelimit_cnt);
			printk_ratelimited("RSC vaudit over ratelimit ignorecnt: %u idx: %u buf: %s\n",
				atomic_read(&rsc_ratelimit_cnt), info.idx, info.buffer);
			return ret;
		}
		ret = vaudit(CHANNEL_ID_KCOLLECT, PACKET_TAG_KCOLLECT, (char *)&info, info_len);
		if (ret < 0) {
			/*pr_err("vaudit %s: vaudit error\n", __func__);*/
			ret = -1;
		}
	}
	pr_debug("vaudit %s: length=%d type=%d\n", __func__, length, type);
	return ret;
}

/*
  * Function: vaudit
  * Description: collect the data and send to user space\
  * It can use in atomic context env, except hardware interrupt.
  * It could not used in hardware interrupt context.

  * type is the main type see vaudit_type
  * subtype is user define type
  * overval is the value, maybe millisecond or times
  * fmt -- string
  * Return: -1--failed, 0--success
**/
int rsc_vaudit(vaudit_type type, u32 subtype, u32 overval, const char *fmt, ...)
{
	va_list args;
	int ret = -1;

	if (type & rsc_vaudit_switch) {
		va_start(args, fmt);
		ret = vsend(type, subtype, overval, args, fmt);
		va_end(args);
	}
	return ret;
}

static int __init vaudit_init(void)
{
	int ret = -1;
	struct proc_dir_entry *dir;

	spin_lock_init(&vaudit_kfifo_lock);

	/*init fifo*/
	ret = kfifo_alloc(&vaudit_fifo, VAUDIT_FIFO_SIZE, GFP_ATOMIC);
	if (0 != ret) {
		rsc_err("vaudit %s: error kfifo_alloc!\n", __func__);
		goto err_alloc_kfifo;
	}

	/*init fifo*/
	ret = kfifo_alloc(&vaudit_fifo_last, VAUDIT_RECORD_LAST_SIZE, GFP_ATOMIC);
	if (0 != ret) {
		rsc_err("vaudit %s: error kfifo_alloc last!\n", __func__);
		goto err_alloc_kfifo;
	}

	init_completion(&rsc_vaudit_complete);
	dir = proc_create_data("vaudit", S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, vivo_rsc, &vaudit_operations,
					NULL);
	if (dir)
		proc_set_user(dir, SYSTEM_ID, SYSTEM_GROUP_ID);
	else
		rsc_err("Create vaudit error!\n");

	/*run thread*/
	vaudit_thread = kthread_run(fifo_out, NULL, VAUDIT_THREAD_NAME);
	if (IS_ERR_OR_NULL(vaudit_thread)) {
		rsc_err("vaudit %s: error create thread!\n", __func__);
		goto err_create_thread;
	}

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_vaudit_attr.attr);
	if (!ret)
		rsc_chown_to_system(rsc_root_dir, &rsc_vaudit_attr.attr);

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &vaudit_switch_attr.attr);
	if (!ret)
		rsc_chown_to_system(rsc_root_dir, &vaudit_switch_attr.attr);

	/* create sysfs */
	ret = sysfs_create_file(rsc_root_dir, &rsc_syscall_attr.attr);
	if (!ret)
		rsc_chown_to_system(rsc_root_dir, &rsc_syscall_attr.attr);

	smp_wmb();
	vaudit_kfifo_init = true;
	rsc_info("vaudit %s: inited! vaudit_fifo size: %d lastsize: %d\n", __func__,
		vaudit_fifo.kfifo.mask+1, vaudit_fifo_last.kfifo.mask+1
		);
	return 0;

err_create_thread:
	kfifo_free(&vaudit_fifo);
err_alloc_kfifo:
	return ret;
}

static void __exit vaudit_exit(void)
{
	if (!IS_ERR_OR_NULL(vaudit_thread)) {
		kthread_stop(vaudit_thread);
	}
	kfifo_free(&vaudit_fifo);
	kfifo_free(&vaudit_fifo_last);
}

module_init(vaudit_init);
module_exit(vaudit_exit);
