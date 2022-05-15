/*
 * 2019, Yang Yang
 */

#ifndef _LINUX_BLK_ENHANCEMENT_H
#define _LINUX_BLK_ENHANCEMENT_H

#ifdef CONFIG_BLK_ENHANCEMENT

#include <linux/bto.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/blk-mq.h>
#include <linux/blk-cgroup.h>

#ifdef CONFIG_RSC_LOCK_BOOST
#include <linux/vivo_rsc/rsc_test_svp.h>
#endif


#define IS_BIO_DUMP		2
#define IS_BIO_REMAP	3
#define IS_BIO_LOGED	4
#define IS_BIO_END		5

DECLARE_PER_CPU(u64, io_nr_read);
DECLARE_PER_CPU(u64, io_nr_read_sched);

extern unsigned int blk_enhancement_trace_en;
extern unsigned int blk_host_busy;
extern unsigned int blk_sched_busy;
extern unsigned int blk_queuecmd;

static inline bool bio_trace_enable(struct bio *bio)
{
	struct request_queue *q;

	if (!blk_enhancement_trace_en)
		return false;

	q = bio->bi_disk->queue;

	if (!q->io_trace)
		return false;

	return true;
}

static inline bool bio_get_t_flag(struct bio *bio, unsigned int bit)
{
	return (bio->bto.t_flag & (1U << bit)) != 0;
}

static inline void bio_set_t_flag(struct bio *bio, unsigned int bit)
{
	bio->bto.t_flag |= (1U << bit);
}

static inline void bio_clear_t_flag(struct bio *bio, unsigned int bit)
{
	bio->bto.t_flag &= ~(1U << bit);
}

static inline sector_t ll_blk_rq_sector(struct request *rq)
{
	/*
	 * Tracing should ignore starting sector for passthrough requests and
	 * requests where starting sector didn't get set.
	 */
	if (blk_rq_is_passthrough(rq) || blk_rq_pos(rq) == (sector_t)-1)
		return 0;
	return blk_rq_pos(rq);
}

static inline unsigned int ll_blk_rq_nr_sectors(struct request *rq)
{
	return blk_rq_is_passthrough(rq) ? 0 : blk_rq_sectors(rq);
}

static inline void trigger_mq_get_tag_in(struct request *rq)
{
	struct bio *bio;
	unsigned long long now;

	if (!rq->info.rq_get_time)
		return;

	now = ktime_get_ns();

	__rq_for_each_bio(bio, rq) {
		if (bio->bto.tagin_ns == 0) {
			bio->bto.tagin_ns = now;
		}
	}
}

static inline void trigger_mq_get_tag_out(struct request *rq)
{
	struct bio *bio;
	unsigned long long now;

	if (!rq->info.rq_get_time)
		return;

	now = ktime_get_ns();

	__rq_for_each_bio(bio, rq) {
		if (bio->bto.tagout_ns == 0)
			bio->bto.tagout_ns = now;
	}
}

/****************************************************************/

enum threshold_index {
	OP_DISCARD,
	OP_FLUSH,
	OP_SNYC_WR_128,
	OP_SNYC_WR_256,
	OP_SNYC_WR_1024,
	OP_READ_128,
	OP_READ_256,
	OP_READ_1024,
	OP_SNYC,
	OP_ASNYC,
};

#define DONE(bio)	(bio->bi_iter.bi_sector - bio->bto.n_sector)

extern unsigned int timeout_threshold_mt[];
extern unsigned int timeout_threshold_fg[];
extern unsigned int timeout_threshold[];

static inline unsigned int get_bio_timeout_fn(struct bio *bio,
			unsigned int threshold[])
{
	if (bio_op(bio) == REQ_OP_READ) {
		if (DONE(bio) <= 256)
			return threshold[OP_READ_256];
		else
			return threshold[OP_READ_1024];
	}

	if (op_is_flush(bio->bi_opf))
		return threshold[OP_FLUSH];

	if (bio_op(bio) == REQ_OP_DISCARD)
		return threshold[OP_DISCARD];

	if (op_is_sync(bio->bi_opf)) {
		if ((bio_op(bio) == REQ_OP_WRITE)) {
			if (DONE(bio) <= 256)
				return threshold[OP_SNYC_WR_256];
			else
				return threshold[OP_SNYC_WR_1024];
		} else {
			return threshold[OP_SNYC];
		}
	}

	return threshold[OP_ASNYC];
}

static inline unsigned int get_bio_timeout(struct bio *bio)
{
	unsigned int *threshold = NULL;

	if (bio->bto.cgroup == 1 || bio->bto.priority[0] == 1)
		threshold = timeout_threshold_mt;
	else if (bio->bto.priority[0] > 1 && bio->bto.priority[0] < 5)
		threshold = timeout_threshold_fg;
	else
		threshold = timeout_threshold;

	return get_bio_timeout_fn(bio, threshold);
}

static inline void ll_fill_rwbs(char *rwbs, unsigned int op)
{
	int i = 0;

	if (op & REQ_PREFLUSH)
		rwbs[i++] = 'F';

	switch (op & REQ_OP_MASK) {
	case REQ_OP_WRITE:
	case REQ_OP_WRITE_SAME:
		rwbs[i++] = 'W';
		break;
	case REQ_OP_DISCARD:
		rwbs[i++] = 'D';
		break;
	case REQ_OP_SECURE_ERASE:
		rwbs[i++] = 'D';
		rwbs[i++] = 'E';
		break;
	case REQ_OP_FLUSH:
		rwbs[i++] = 'F';
		break;
	case REQ_OP_READ:
		rwbs[i++] = 'R';
		break;
	default:
		rwbs[i++] = 'N';
	}

	if (op & REQ_FUA)
		rwbs[i++] = 'F';
	if (op & REQ_RAHEAD)
		rwbs[i++] = 'A';
	if (op & REQ_SYNC)
		rwbs[i++] = 'S';
	if (op & REQ_META)
		rwbs[i++] = 'M';

	rwbs[i] = '\0';
}

#define MS(x) ((x) / 1000000)
#define US(x) ((x) / 1000)

#define MAX_BTO		512

static inline void dump_suspect_bio(struct bio *bio)
{
	struct request_queue *q = bio->bi_disk->queue;
	unsigned long long q2d = 0, d2e = 0, q2e = 0, elv = 0;
#ifdef BLK_ENHANCEMENT_WBT
	unsigned long long qos = 0;
#endif
	unsigned long long getrq = 0, qcmd = 0, busy = 0;
#ifdef BLK_ENHANCEMENT_SCMD
	unsigned long long scmd = 0;
#endif

	char b[BDEVNAME_SIZE];
	char rwbs[8];
	char *buf = NULL;
	int len = 0;

	buf = kzalloc(MAX_BTO, GFP_ATOMIC);
	if (!buf)
		return;

	ll_fill_rwbs(rwbs, bio->bi_opf);

	len += snprintf(buf + len, MAX_BTO - len,
			"BTO: [%d %d] (%s)(%s) <%d><%d %d> %s %s %Lu + %Lu -> ",
#ifndef BLK_ENHANCEMENT_USE_TASK
			bio->bto.pid, bio->bto.gid, bio->bto.comm, bio->bto.leader,
#else
			bio->bto.task->pid, bio->bto.task->group_leader->pid,
			bio->bto.task->comm, bio->bto.task->group_leader->comm,
#endif
			bio->bto.cgroup, bio->bto.priority[0], bio->bto.priority[1],
			bio_devname(bio, b), rwbs,
			(unsigned long long)(bio->bi_iter.bi_sector - DONE(bio)),
			(unsigned long long)DONE(bio));

	q2e = bio->bto.endio_ns - bio->bto.queue_ns;

#if 0
	if (bio_get_t_flag(bio, IS_BIO_REMAP)) {
		unsigned long long remap = bio->bto.remap_ns - bio->bto.queue_ns;

		len += snprintf(buf + len, MAX_BTO - len,
				"q2e=%llu remap=%llu [%lu]",
				MS(q2e), MS(remap),
				bio->bto.nivcsw[1]-bio->bto.nivcsw[0]);
		pr_info("%s %ps\n", buf, bio->bi_end_io);
		kfree(buf);
		return;
	}
#endif

	if (bio->bto.issue_ns) {
		q2d = bio->bto.issue_ns - bio->bto.queue_ns;
		d2e = bio->bto.endio_ns - bio->bto.issue_ns;
		busy = bio->bto.issue_ns - bio->bto.dequeue_ns;
		elv = bio->bto.dequeue_ns - bio->bto.insert_ns;

		if (bio->bto.qcmd_ns)
			qcmd = bio->bto.qcmd_ns - bio->bto.issue_ns;

#ifdef BLK_ENHANCEMENT_SCMD
		if (bio->bto.scmd_ns)
			scmd = bio->bto.scmd_ns - bio->bto.issue_ns;
#endif
	}

	if (bio->bto.wbtout_ns) {
#ifdef BLK_ENHANCEMENT_WBT
		if (bio->bto.wbtin_ns)
			qos = bio->bto.wbtout_ns - bio->bto.wbtin_ns;
#endif
		getrq = bio->bto.getrq_ns - bio->bto.wbtout_ns;
	}

	len += snprintf(buf + len, MAX_BTO - len,
			"q2d=%llu d2e=%llu q2e=%llu "
			"getrq=%llu "
			"plug=%llu "
			"elv=%llu "
			"flush=%llu "
			"qcmd=%llu "
			"lat=%llu "
			"sirq=%llu ",
			MS(q2d), MS(d2e), MS(q2e),
			MS(getrq),
			MS(bio->bto.insert_ns - bio->bto.getrq_ns),
			MS(elv),
			MS(bio->bto.flush_end_ns - bio->bto.flush_start_ns),
			MS(qcmd),
			MS(bio->bto.compl_ns - bio->bto.qcmd_ns),
			MS(bio->bto.softout_ns - bio->bto.softin_ns));

	if (q->mq_ops) {
		unsigned long long tag = bio->bto.tagout_ns - bio->bto.tagin_ns;
		len += snprintf(buf + len, MAX_BTO - len,
				"tag=%llu ", MS(tag));
	}

	len += snprintf(buf + len, MAX_BTO - len,
			"busy=%llu "
#ifdef BLK_ENHANCEMENT_SCMD
			"scmd=%llu "
#endif
#ifdef BLK_ENHANCEMENT_WBT
			"qos=%llu "
#endif
#ifdef BLK_ENHANCEMENT_MQ_ATOMIC
			"inflight=[%d %d] "
			"nr_rqs=[%d %d] "
#endif
			"{%d %d} "
			"[%lu %lu]",
			MS(busy),
#ifdef BLK_ENHANCEMENT_SCMD
			MS(scmd),
#endif
#ifdef BLK_ENHANCEMENT_WBT
			MS(qos),
#endif
#ifdef BLK_ENHANCEMENT_MQ_ATOMIC
			bio->bto.inflight[0], bio->bto.inflight[1],
			bio->bto.nr_rqs[0], bio->bto.nr_rqs[1],
#endif
			bio->bto.cpu[0], bio->bto.cpu[1],
			bio->bto.nivcsw[1]-bio->bto.nivcsw[0],
			bio->bto.nivcsw[2]-bio->bto.nivcsw[1]);

	if (len >= MAX_BTO)
		BUG();

	pr_info("%s %ps\n", buf, bio->bi_end_io);

#if 0
	len = 0;
	memset(buf, 0, sizeof(buf));

	len += snprintf(buf + len, MAX_BTO - len,
			"BTO: queue=%llu getrq=%llu insert=%llu "
			"dequeue=%llu issue=%llu "
			"endio=%llu wbtin=%llu wbtout=%llu ",
			US(bio->bto.queue_ns), US(bio->bto.getrq_ns), US(bio->bto.insert_ns),
			US(bio->bto.dequeue_ns), US(bio->bto.issue_ns),
			US(bio->bto.endio_ns),
			US(bio->bto.wbtin_ns), US(bio->bto.wbtout_ns));

	if (q->mq_ops) {
		len += snprintf(buf + len, MAX_BTO - len,
				"tagin=%llu tagout=%llu ",
				US(bio->bto.tagin_ns), US(bio->bto.tagout_ns));
	}

	pr_info("%s\n", buf);
#endif

	kfree(buf);
}

static inline void trigger_bio_timer(struct bio *bio)
{
#ifndef BLK_ENHANCEMENT_USE_TASK
	bio->bto.pid = current->pid;
	bio->bto.gid = current->group_leader->pid;

	strlcpy(bio->bto.comm, current->comm, TASK_COMM_LEN);
	strlcpy(bio->bto.leader, current->group_leader->comm,
			TASK_COMM_LEN);
#else
	bio->bto.task = current;
#endif
}

static inline void trigger_bio_queue(struct bio *bio, int prio)
{
	if (!bio_trace_enable(bio))
		return;

	bio->bto.queue_ns = ktime_get_ns();
	bio->bto.nivcsw[0] = current->nivcsw;
	bio->bto.n_sector = bio->bi_iter.bi_sector;
	bio->bto.priority[0] = prio;

#ifdef CONFIG_RSC_LOCK_BOOST
	bio->bto.cgroup = test_task_svp_nocheck_render(current);
#else
	bio->bto.cgroup = 0;
#endif

	trigger_bio_timer(bio);
}

static inline void trigger_bio_split(struct bio *bio, struct bio *split)
{
	if (!bio->bto.queue_ns)
		return;

	split->bto.queue_ns = bio->bto.queue_ns;
	split->bto.nivcsw[0] = bio->bto.nivcsw[0];
	split->bto.cgroup = bio->bto.cgroup;
	split->bto.priority[0] = bio->bto.priority[0];

	bio->bto.n_sector = bio->bi_iter.bi_sector;
	split->bto.n_sector = split->bi_iter.bi_sector;

	trigger_bio_timer(split);
}

static inline void trigger_bio_remap(struct bio *bio)
{
#if 0
	if (!bio->clone->bto.queue_ns)
		return;

	bio->clone->bto.remap_ns = ktime_get_ns();
	bio->clone->bto.nivcsw[1] = current->nivcsw;

	bio_set_t_flag(bio->clone, IS_BIO_REMAP);
#endif
}

static inline void trigger_bio_wbtin(struct bio *bio)
{
#ifdef BLK_ENHANCEMENT_WBT
	if (!bio->bto.queue_ns)
		return;

	switch (bio_op(bio)) {
		case REQ_OP_WRITE:
			if ((bio->bi_opf & (REQ_SYNC | REQ_IDLE)) ==
				(REQ_SYNC | REQ_IDLE))
				return;
			/* fallthrough */
		case REQ_OP_DISCARD:
			break;
		default:
			return;
	}

	bio->bto.wbtin_ns = ktime_get_ns();
#endif
}

static inline void trigger_bio_wbtout(struct bio *bio, struct request_queue *q)
{
	if (!bio->bto.queue_ns)
		return;

	bio->bto.wbtout_ns = ktime_get_ns();
}

static inline void trigger_rq_flush_start(struct request *rq)
{
#if 0
	struct bio *bio;
	unsigned long long now;

	if (!rq->info.rq_get_time)
		return;

	now = ktime_get_ns();

	__rq_for_each_bio(bio, rq) {
		bio->bto.flush_start_ns = now;
	}
#endif
}

static inline void trigger_rq_flush_end(struct request *rq)
{
#if 0
	struct bio *bio;
	unsigned long long now;

	if (!rq->info.rq_get_time)
		return;

	now = ktime_get_ns();

	__rq_for_each_bio(bio, rq) {
		bio->bto.flush_end_ns = now;
	}
#endif
}

static inline void trigger_getrq(struct request *rq, struct bio *sio)
{
	struct bio *bio;
	struct request_queue *q = rq->q;

	if (!sio->bto.queue_ns)
		return;

	rq->info.rq_get_time = ktime_get_ns();

	if (q->mq_ops) {
#ifdef BLK_ENHANCEMENT_MQ_ATOMIC
		rq->info.nr_rqs[0] = atomic_read(&q->tag_set->sched_inflight[0]);
		rq->info.nr_rqs[1] = atomic_read(&q->tag_set->sched_inflight[1]);
#else
		rq->info.nr_rqs[0] = -1;
		rq->info.nr_rqs[1] = -1;
#endif
	}

	rq->info.inflight[1] = rq->info.inflight[0] = 0;
	rq->info.rq_insert_time = 0;

	__rq_for_each_bio(bio, rq) {
		bio->bto.getrq_ns = rq->info.rq_get_time;
		bio->bto.nivcsw[1] = current->nivcsw;

		bio->bto.nr_rqs[0] = rq->info.nr_rqs[0];
		bio->bto.nr_rqs[1] = rq->info.nr_rqs[1];
	}
}

static inline void trigger_bio_merge(struct bio *bio, struct request *rq)
{
	struct request_queue *q = rq->q;

	if (!bio->bto.queue_ns)
		return;

	if (q->mq_ops) {
		bio->bto.nr_rqs[0] = rq->info.nr_rqs[0];
		bio->bto.nr_rqs[1] = rq->info.nr_rqs[1];
	}

	bio->bto.getrq_ns = ktime_get_ns();
	bio->bto.nivcsw[1] = current->nivcsw;

	/* bio elv merged */
	if (rq->info.rq_insert_time) {
		bio->bto.insert_ns = bio->bto.getrq_ns;
		bio->bto.nivcsw[2] = bio->bto.nivcsw[1];

		/* if plug merge, inflight will be set in trigger_rq_insert */
		if (q->mq_ops) {
			bio->bto.inflight[0] = rq->info.inflight[0];
			bio->bto.inflight[1] = rq->info.inflight[1];
		}
	}
}

static inline void trigger_rq_merge(struct request *rq, struct request *next,
			struct request_queue *q)
{
	struct bio *bio;
	unsigned long long now;
	struct request *orig;

	if (!rq->info.rq_get_time || !next->info.rq_get_time)
		return;

	if (rq->info.rq_insert_time && next->info.rq_insert_time)
		return;

	now = ktime_get_ns();

	if (rq->info.rq_insert_time) {
		orig = rq;
	} else if (next->info.rq_insert_time) {
		orig = next;
		rq->info.rq_insert_time = now;
	} else {
		BUG();
		return;
	}

	__rq_for_each_bio(bio, rq) {
		if (bio->bto.insert_ns == 0) {
			bio->bto.insert_ns = now;

			bio->bto.inflight[0] = orig->info.inflight[0];
			bio->bto.inflight[1] = orig->info.inflight[1];

			bio->bto.nivcsw[2] = current->nivcsw;
		}
	}
}

static inline void trigger_rq_insert(struct request *rq)
{
	struct bio *bio;
	struct request_queue *q = rq->q;

	if (rq->info.rq_insert_time)
		return;

	rq->info.rq_insert_time = ktime_get_ns();

	if (!rq->info.rq_get_time)
		return;

	if (q->mq_ops) {
#ifdef BLK_ENHANCEMENT_MQ_ATOMIC
		rq->info.inflight[0] = atomic_read(&q->tag_set->inflight[0]);
		rq->info.inflight[1] = atomic_read(&q->tag_set->inflight[1]);
#else
		rq->info.inflight[0] = -1;
		rq->info.inflight[1] = -1;
#endif
	}

	__rq_for_each_bio(bio, rq) {
		struct blkcg *blkcg;
		int prio = 0;

		rcu_read_lock();
		blkcg = bio_blkcg(bio);
		prio = blkcg ? blkcg->priority : 0;
		rcu_read_unlock();

		bio->bto.insert_ns = rq->info.rq_insert_time;

		bio->bto.inflight[0] = rq->info.inflight[0];
		bio->bto.inflight[1] = rq->info.inflight[1];

		bio->bto.nivcsw[2] = current->nivcsw;

		bio->bto.priority[1] = prio;
	}
}

static inline void trigger_rq_issue(struct request *rq)
{
	struct bio *bio;

	rq->info.rq_issue_time = ktime_get_ns();

	if (!rq->info.rq_dequeue_time)
		rq->info.rq_dequeue_time = rq->info.rq_issue_time;

	if (!rq->info.rq_get_time)
		return;

	__rq_for_each_bio(bio, rq) {
		bio->bto.issue_ns = rq->info.rq_issue_time;
		bio->bto.dequeue_ns = rq->info.rq_dequeue_time;
	}
}

static inline void trigger_rq_scmd(struct request *rq)
{
#ifdef BLK_ENHANCEMENT_SCMD
	struct bio *bio;
	unsigned long long now;

	if (!rq->info.rq_get_time)
		return;

	now = ktime_get_ns();

	__rq_for_each_bio(bio, rq) {
		bio->bto.scmd_ns = now;
	}
#endif
}

static inline void trigger_bio_endio(struct bio *bio)
{
	if (!bio->bto.queue_ns)
		return;

	if (bio_get_t_flag(bio, IS_BIO_END))
		return;

	bio->bto.endio_ns = ktime_get_ns();

	if (MS(bio->bto.endio_ns - bio->bto.queue_ns) >= get_bio_timeout(bio)) {
		dump_suspect_bio(bio);

		if (bio_op(bio) == REQ_OP_READ) {
			if (bio->bto.cgroup == 1 || bio->bto.priority[0] == 1) {
				if (bio->bto.nivcsw[2] == bio->bto.nivcsw[0])
					this_cpu_inc(io_nr_read);
				else
					this_cpu_inc(io_nr_read_sched);
			}
		}
	}

	bio_set_t_flag(bio, IS_BIO_END);
}

static inline void trigger_rq_softirq_in(struct request *rq)
{
	rq->info.rq_softin_time = ktime_get_ns();
}

static inline void trigger_rq_softirq_out(struct request *rq, int cpu0, int cpu1)
{
	struct bio *bio;

	rq->info.rq_softout_time = ktime_get_ns();

	if (!rq->info.rq_get_time)
		return;

	__rq_for_each_bio(bio, rq) {
		bio->bto.softin_ns = rq->info.rq_softin_time;
		bio->bto.softout_ns = rq->info.rq_softout_time;
		bio->bto.qcmd_ns = rq->info.rq_qcmd_time;
		bio->bto.compl_ns = rq->info.rq_compl_time;
		bio->bto.cpu[0] = cpu0;
		bio->bto.cpu[1] = cpu1;
	}
}

static inline void trigger_cmd_issue(struct request *rq, ktime_t now)
{
	rq->info.rq_qcmd_time = ktime_to_ns(now);

	if (rq->info.rq_issue_time) {
		u64 busy;

		busy = MS(rq->info.rq_qcmd_time - rq->info.rq_issue_time);

		if (busy >= blk_queuecmd) {
			dev_t dev;
			sector_t sector;
			unsigned int nr_sector;
			char rwbs[8];

			dev = rq->rq_disk ? disk_devt(rq->rq_disk) : 0;
			sector = ll_blk_rq_sector(rq);
			nr_sector = ll_blk_rq_nr_sectors(rq);

			ll_fill_rwbs(rwbs, rq->cmd_flags);

			pr_info("BTO: QueueCmd %lluMS %d,%d %s %llu + %u\n",
					busy,
					MAJOR(dev), MINOR(dev), rwbs,
					(unsigned long long)sector, nr_sector);
		}

		busy = MS(rq->info.rq_issue_time - rq->info.rq_dequeue_time);

		if (busy >= blk_host_busy) {
			dev_t dev;
			sector_t sector;
			unsigned int nr_sector;
			char rwbs[8];

			dev = rq->rq_disk ? disk_devt(rq->rq_disk) : 0;
			sector = ll_blk_rq_sector(rq);
			nr_sector = ll_blk_rq_nr_sectors(rq);

			ll_fill_rwbs(rwbs, rq->cmd_flags);

			pr_info("BTO: Host Busy %lluMS <%u> %d,%d %s %llu + %u\n",
					busy,
					rq->info.requeue_count,
					MAJOR(dev), MINOR(dev), rwbs,
					(unsigned long long)sector, nr_sector);
		}
	}
}

static inline void trigger_cmd_abort(struct request *rq, ktime_t now)
{
	rq->info.rq_abort_time = ktime_to_ns(now);
}

static inline void trigger_cmd_compl(struct request *rq, ktime_t now)
{
	rq->info.rq_compl_time = ktime_to_ns(now);
}

static inline void trigger_rq_requeue(struct request *rq)
{
	u64 now, busy;

	now = ktime_get_ns();

	if (rq->info.rq_requeue_time) {
		busy = MS(now - rq->info.rq_requeue_time);

		if (busy >= blk_sched_busy) {
			dev_t dev;
			sector_t sector;
			unsigned int nr_sector;
			char rwbs[8];

			dev = rq->rq_disk ? disk_devt(rq->rq_disk) : 0;
			sector = ll_blk_rq_sector(rq);
			nr_sector = ll_blk_rq_nr_sectors(rq);

			ll_fill_rwbs(rwbs, rq->cmd_flags);

			pr_info("BTO: Schedule Busy %lluMS <%u> %d,%d %s %llu + %u <%d>\n",
					busy,
					rq->info.requeue_count,
					MAJOR(dev), MINOR(dev), rwbs,
					(unsigned long long)sector, nr_sector,
					rq->rq_flags & RQF_DONTPREP);
		}
	}

	rq->info.rq_requeue_time = now;
	rq->info.requeue_count++;
}

#else

static inline void trigger_bio_queue(struct bio *bio, int prio) {}
static inline void trigger_bio_split(struct bio *bio, struct bio *split) {}
static inline void trigger_bio_wbtin(struct bio *bio) {}
static inline void trigger_bio_wbtout(struct bio *bio,
						struct request_queue *q) {}
static inline void trigger_rq_insert(struct request *rq) {}
static inline void trigger_rq_issue(struct request *rq) {}
static inline void trigger_rq_scmd(struct request *rq) {}
static inline void trigger_bio_endio(struct bio *bio) {}
static inline void trigger_rq_flush_start(struct request *rq) {}
static inline void trigger_rq_flush_end(struct request *rq) {}
static inline void trigger_getrq(struct request *rq, struct bio *sio) {}
static inline void trigger_bio_merge(struct bio *bio, struct request *rq) {}
static inline void trigger_rq_merge(struct request *rq, struct request *next,
						struct request_queue *q) {}
static inline void trigger_bio_remap(struct bio *bio) {}

static inline void trigger_mq_get_tag_in(struct request *rq) {}
static inline void trigger_mq_get_tag_out(struct request *rq) {}

static inline void trigger_rq_softirq_in(struct request *rq) {}
static inline void trigger_rq_softirq_out(struct request *rq, int cpu0, int cpu1) {}

static inline void trigger_cmd_issue(struct request *rq, ktime_t now) {}
static inline void trigger_cmd_abort(struct request *rq, ktime_t now) {}
static inline void trigger_cmd_compl(struct request *rq, ktime_t now) {}

static inline void trigger_rq_requeue(struct request *rq) {}
#endif

#endif
