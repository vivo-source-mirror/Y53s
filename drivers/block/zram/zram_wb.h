/*
 * Write back zram to backing device
 *
 * Copyright (C) 2020 vivo Co., Ltd. All rights reserved.
 * Author: Jiewen Wang
 */

#ifndef _ZRAM_WB_H_
#define _ZRAM_WB_H_

#define WMARK_COUNT 3
#define WMARK_HIGH 0
#define WMARK_A 1
#define WMARK_B 2

// statistics stay time in backing device
#define STAY_COUNT 4
#define STAY1 0
#define STAY2 1
#define STAY3 2
#define STAY_DEFAULT 3

struct zram;

struct zram_wb {
	struct task_struct *thread;
	wait_queue_head_t wait_queue;
	/*** sysfs begin ***/
	struct kobject *kobj;
	unsigned int wmark[WMARK_COUNT]; // percentage of zram disksize
	unsigned long huge_interval;
	unsigned long sleep_millisecs;
	unsigned long pages_per_time;
	unsigned long max_bd_nr_pages; // backing devices max store pages
	unsigned long bd_reclaim_min; // if free bd count less than it, stop reclaim
	bool enable;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
	bool dswappiness_enable;
	unsigned long dswappiness_high;
	unsigned long dswappiness_low;
	unsigned long dswappiness_location;
	unsigned long dswappiness_multiple;
#endif
	// stats
	atomic64_t do_work_times;
	atomic64_t stay_read[STAY_COUNT];
	atomic64_t stay_free[STAY_COUNT];
	atomic64_t bd_reads;
	atomic64_t bd_writes;
	atomic64_t bd_max_used;
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DEBUG
	atomic64_t reclaim_time;
	atomic64_t reclaim_huge_time;
	atomic64_t reclaim_idle_time;
#endif
	/*** sysfs end ***/
	unsigned long ram_max; // pages stored in zsmalloc max. used of wmark max
	struct page *t_page; // temp page for reuse
};

void zram_wb_register(struct zram *zram);
void zram_wb_unregister(struct zram *zram);
void wake_up_zram_wbd(struct zram *zram);
#ifdef CONFIG_ZRAM_WRITEBACK_VIVO_DYNAMIC_SWAPPINESS
void zram_wb_update_swappiness(struct zram *zram);
#endif

#endif
