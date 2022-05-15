// SPDX-License-Identifier: GPL-2.0
/*
 * Functions related to softirq rq completions
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/sched/topology.h>

#include "blk.h"

#define BLK_SOFTIRQ_TUNING		1
#define BLK_SOFTIRQ_PENDING		2

static DEFINE_PER_CPU(struct list_head, blk_cpu_done);

/*
 * Softirq action handler - move entries to local list and loop over them
 * while passing them to the queue registered handler.
 */
static __latent_entropy void blk_done_softirq(struct softirq_action *h)
{
	struct list_head *cpu_list, local_list;
#ifdef CONFIG_BLK_ENHANCEMENT
	int cpu;
#endif

	local_irq_disable();
#ifdef CONFIG_BLK_ENHANCEMENT
	cpu = smp_processor_id();
#endif
	cpu_list = this_cpu_ptr(&blk_cpu_done);
	list_replace_init(cpu_list, &local_list);
	local_irq_enable();

	while (!list_empty(&local_list)) {
		struct request *rq;

		rq = list_entry(local_list.next, struct request, ipi_list);
		list_del_init(&rq->ipi_list);
#ifdef CONFIG_BLK_ENHANCEMENT
		if (rq->q->mq_ops)
			trigger_rq_softirq_out(rq, rq->mq_ctx->cpu, cpu);
		else
			trigger_rq_softirq_out(rq, rq->cpu, cpu);
#endif
		rq->q->softirq_done_fn(rq);
	}
}

#ifdef CONFIG_SMP
static void trigger_softirq(void *data)
{
	struct request *rq = data;
	unsigned long flags;
	struct list_head *list;

	local_irq_save(flags);
	list = this_cpu_ptr(&blk_cpu_done);
	list_add_tail(&rq->ipi_list, list);

	if (list->next == &rq->ipi_list)
		raise_softirq_irqoff(BLOCK_SOFTIRQ);

	local_irq_restore(flags);
}

/*
 * Setup and invoke a run of 'trigger_softirq' on the given cpu.
 */
static int raise_blk_irq(int cpu, struct request *rq)
{
	if (cpu_online(cpu)) {
		call_single_data_t *data = &rq->csd;

		data->func = trigger_softirq;
		data->info = rq;
		data->flags = 0;

		smp_call_function_single_async(cpu, data);
		return 0;
	}

	return 1;
}
#else /* CONFIG_SMP */
static int raise_blk_irq(int cpu, struct request *rq)
{
	return 1;
}
#endif

static int blk_softirq_cpu_dead(unsigned int cpu)
{
	/*
	 * If a CPU goes away, splice its entries to the current CPU
	 * and trigger a run of the softirq
	 */
	local_irq_disable();
	list_splice_init(&per_cpu(blk_cpu_done, cpu),
			 this_cpu_ptr(&blk_cpu_done));
	raise_softirq_irqoff(BLOCK_SOFTIRQ);
	local_irq_enable();

	return 0;
}

void __blk_complete_request(struct request *req)
{
	int ccpu, cpu;
	struct request_queue *q = req->q;
	unsigned long flags;
	bool shared = false;

	BUG_ON(!q->softirq_done_fn);

#ifdef CONFIG_BLK_ENHANCEMENT
	trigger_rq_softirq_in(req);
#endif

	local_irq_save(flags);
	cpu = smp_processor_id();

	/*
	 * Select completion CPU
	 */
	if (req->cpu != -1) {
		ccpu = req->cpu;
		if (!test_bit(QUEUE_FLAG_SAME_FORCE, &q->queue_flags))
			shared = cpus_share_cache(cpu, ccpu);
	} else
		ccpu = cpu;

	/*
	 * If current CPU and requested CPU share a cache, run the softirq on
	 * the current CPU. One might concern this is just like
	 * QUEUE_FLAG_SAME_FORCE, but actually not. blk_complete_request() is
	 * running in interrupt handler, and currently I/O controller doesn't
	 * support multiple interrupts, so current CPU is unique actually. This
	 * avoids IPI sending from current CPU to the first CPU of a group.
	 */
	if (ccpu == cpu || shared) {
		struct list_head *list;
#if BLK_SOFTIRQ_TUNING
		struct list_head *pos;
		int count = 0;
#endif
do_local:
		list = this_cpu_ptr(&blk_cpu_done);

#if BLK_SOFTIRQ_TUNING
		/* modify by yangyang */
		if (ccpu != cpu) {
			list_for_each(pos, list) {
				count++;
				if (count >= BLK_SOFTIRQ_PENDING) {
					if (raise_blk_irq(ccpu, req)) {
						break;
					} else {
						local_irq_restore(flags);
						return;
					}
				}
			}
		}
#endif

		list_add_tail(&req->ipi_list, list);

		/*
		 * if the list only contains our just added request,
		 * signal a raise of the softirq. If there are already
		 * entries there, someone already raised the irq but it
		 * hasn't run yet.
		 */
		if (list->next == &req->ipi_list)
			raise_softirq_irqoff(BLOCK_SOFTIRQ);
	} else if (raise_blk_irq(ccpu, req))
		goto do_local;

	local_irq_restore(flags);
}

/**
 * blk_complete_request - end I/O on a request
 * @req:      the request being processed
 *
 * Description:
 *     Ends all I/O on a request. It does not handle partial completions,
 *     unless the driver actually implements this in its completion callback
 *     through requeueing. The actual completion happens out-of-order,
 *     through a softirq handler. The user must have registered a completion
 *     callback through blk_queue_softirq_done().
 **/
void blk_complete_request(struct request *req)
{
	if (unlikely(blk_should_fake_timeout(req->q)))
		return;
	if (!blk_mark_rq_complete(req))
		__blk_complete_request(req);
}
EXPORT_SYMBOL(blk_complete_request);

static __init int blk_softirq_init(void)
{
	int i;

	for_each_possible_cpu(i)
		INIT_LIST_HEAD(&per_cpu(blk_cpu_done, i));

	open_softirq(BLOCK_SOFTIRQ, blk_done_softirq);
	cpuhp_setup_state_nocalls(CPUHP_BLOCK_SOFTIRQ_DEAD,
				  "block/softirq:dead", NULL,
				  blk_softirq_cpu_dead);
	return 0;
}
subsys_initcall(blk_softirq_init);
