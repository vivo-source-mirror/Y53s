#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/kernel.h>

extern u32 get_devinfo_with_index(u32 index);
extern u32 get_hrid(unsigned char *rid, unsigned char *rid_sz);
extern u32 get_hrid_size(void);



static u8 cpuid_buffer[128];
static int cpuid_proc_show(struct seq_file *m, void *v)
{
	unsigned char cpuid_size = 0;
	int i;

	cpuid_size = 4 * get_hrid_size();
	memset(cpuid_buffer, 0, 128);
	get_hrid(cpuid_buffer, &cpuid_size);
	//printk("cpuid_size:0x%d\n", cpuid_size);
	for (i = 0; i < cpuid_size; i++) {
		//printk("cpuid_value:0x%x", cpuid_buffer[i]);
		seq_printf(m, "%x", cpuid_buffer[i]);
	}
	//printk("\n");
	seq_printf(m, "\n");

	return 0;
}

static int cpuid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpuid_proc_show, NULL);
}

static const struct file_operations cpuid_proc_fops = {
	.open		= cpuid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int sec_proc_show(struct seq_file *m, void *v)
{
	u32 sec_en = (get_devinfo_with_index(27) & 0x00000002) ? 1 : 0;
	seq_printf(m, "%d\n", sec_en);
	return 0;
}

static int sec_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_proc_show, NULL);
}

static const struct file_operations sec_proc_fops = {
	.open		= sec_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_sec_init(void)
{
	proc_create("sec_en", 0x0, NULL, &sec_proc_fops);
	proc_create("cpu_id", 0x0, NULL, &cpuid_proc_fops);

	return 0;
}

static void __exit proc_sec_exit(void)
{
	remove_proc_entry("sec_en", NULL);
	remove_proc_entry("cpu_id", NULL);

}

module_init(proc_sec_init);
module_exit(proc_sec_exit);

