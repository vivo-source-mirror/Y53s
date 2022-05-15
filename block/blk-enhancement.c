/*
 * 2019, Yang Yang
 */

#include <linux/blk-enhancement.h>
#include <linux/module.h>

DEFINE_PER_CPU(u64, io_nr_read);
DEFINE_PER_CPU(u64, io_nr_read_sched);

unsigned int blk_enhancement_trace_en = 1;

unsigned int blk_host_busy = 30;
EXPORT_SYMBOL_GPL(blk_host_busy);
unsigned int blk_sched_busy = 15;
EXPORT_SYMBOL_GPL(blk_sched_busy);
unsigned int blk_queuecmd = 50;
EXPORT_SYMBOL_GPL(blk_queuecmd);

unsigned int timeout_threshold_mt[] = {
	[OP_DISCARD]		= 800, /* DISCARD */
	[OP_FLUSH]		= 200, /* FLUSH */
	[OP_SNYC_WR_128]	= 200, /* Sync Write */
	[OP_SNYC_WR_256]	= 200,
	[OP_SNYC_WR_1024]	= 200,
	[OP_READ_128]		= 100, /* Read */
	[OP_READ_256]		= 100,
	[OP_READ_1024]		= 100,
	[OP_SNYC]		= 200, /* Sync */
	[OP_ASNYC]		= 2000, /* A-Sync */
};

unsigned int timeout_threshold_fg[] = {
	[OP_DISCARD]		= 800, /* DISCARD */
	[OP_FLUSH]		= 300, /* FLUSH */
	[OP_SNYC_WR_128]	= 800, /* Sync Write */
	[OP_SNYC_WR_256]	= 800,
	[OP_SNYC_WR_1024]	= 800,
	[OP_READ_128]		= 200, /* Read */
	[OP_READ_256]		= 200,
	[OP_READ_1024]		= 200,
	[OP_SNYC]		= 200, /* Sync */
	[OP_ASNYC]		= 2000, /* A-Sync */
};

unsigned int timeout_threshold[] = {
	[OP_DISCARD]		= 2000, /* DISCARD */
	[OP_FLUSH]		= 2000, /* FLUSH */
	[OP_SNYC_WR_128]	= 2000, /* Sync Write */
	[OP_SNYC_WR_256]	= 2000,
	[OP_SNYC_WR_1024]	= 2000,
	[OP_READ_128]		= 500, /* Read */
	[OP_READ_256]		= 500,
	[OP_READ_1024]		= 500,
	[OP_SNYC]		= 2000, /* Sync */
	[OP_ASNYC]		= 2000, /* A-Sync */
};

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX	"customize."

static int get_io_nr_read_sched(char *buf, const struct kernel_param *kp)
{
	int cpu;
	u64 count = 0;

	for_each_possible_cpu(cpu) {
		count += per_cpu(io_nr_read_sched, cpu);
	}

	return sprintf(buf, "%llu\n", count);
}

static int get_io_nr_read(char *buf, const struct kernel_param *kp)
{
	int cpu;
	u64 count = 0;

	for_each_possible_cpu(cpu) {
		count += per_cpu(io_nr_read, cpu);
	}

	return sprintf(buf, "%llu\n", count);
}

module_param_call(io_nr_read, NULL, get_io_nr_read, NULL, 0644);
module_param_call(io_nr_read_sched, NULL, get_io_nr_read_sched, NULL, 0644);

module_param_named(host_busy, blk_host_busy, uint, 0644);
module_param_named(sched_busy, blk_sched_busy, uint, 0644);
module_param_named(queuecmd, blk_queuecmd, uint, 0644);

module_param_array_named(bto4mt, timeout_threshold_mt, uint, NULL,
			 S_IRUGO | S_IWUSR);

module_param_array_named(bto4fg, timeout_threshold_fg, uint, NULL,
			 S_IRUGO | S_IWUSR);

module_param_array_named(bto, timeout_threshold, uint, NULL,
			 S_IRUGO | S_IWUSR);

module_param_named(trace, blk_enhancement_trace_en, uint, 0644);
