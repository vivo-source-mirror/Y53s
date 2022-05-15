#include <linux/mmzone.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/init.h>
#include <linux/page-isolation.h>
#include <linux/vivo_rsc/rsc_internal.h>

int __read_mostly rsc_mem_antifrag_disabled;

/*
 * check if the pageblock is UNMOVABLE_ISOLATE.
 */
int unmovable_isolate_pageblock(struct zone *zone, struct page *page)
{
	int migratetype;

	migratetype = get_pageblock_migratetype(page);
	if (unmovable_isolate_enabled(zone) &&
	    (is_unmovable_isolate_sord(migratetype) ||
	    is_unmovable_isolate_bord(migratetype)))
		return 1;
	else
		return 0;
}

#define RSC_MEM_DEFRAG_ONE_GB_PAGES 	((1*1024*1024UL)  >> (PAGE_SHIFT - 10))
#define RSC_MEM_DEFRAG_TWO_GB_PAGES 	((2*1024*1024UL)  >> (PAGE_SHIFT - 10))
#define RSC_MEM_DEFRAG_THREE_GB_PAGES 	((3*1024*1024UL)  >> (PAGE_SHIFT - 10))
#define RSC_MEM_DEFRAG_FIVE_GB_PAGES 	((5*1024*1024UL)  >> (PAGE_SHIFT - 10))
#define RSC_MEM_DEFRAG_SEVEN_GB_PAGES 	((7*1024*1024UL)  >> (PAGE_SHIFT - 10))
#define RSC_MEM_DEFRAG_NINE_GB_PAGES 	((9*1024*1024UL)  >> (PAGE_SHIFT - 10))

/*
 * get the related unmovable-isolate size
 */
static inline u64 get_unmovable_isolate_blocks (int unmovable_isolate_type)
{
	unsigned long s_block;
	unsigned long b_block;
	unsigned long ui_blocks;

	/*sord size 256MB*/
	s_block = RSC_ISOLATE_SORD_SIZE_BLOCKS;
	if (totalram_pages >= RSC_MEM_DEFRAG_SEVEN_GB_PAGES) {
		/*ram >= 7GB, bord size is 108MB*/
		/*RSC_ISOLATE_BORD_SIZE_BLOCKS;*/
		b_block = (108*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
	} else if (totalram_pages >= RSC_MEM_DEFRAG_FIVE_GB_PAGES) {
		/*ram >= 5GB, bord size is 68MB*/
		b_block = (108*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
	} else if (totalram_pages >= RSC_MEM_DEFRAG_THREE_GB_PAGES) {
		/*ram >= 4GB, bord size is 68MB*/
#if defined(CONFIG_MTK_PLATFORM)
		/*FixBug: B190328-1469【影响自动化测试】执行iceberg脚本过程中，手机掉端口*/
		/*MTK platform 3+1 GB*/
		s_block = (192*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		b_block = (44*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#else
		b_block = (108*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif
	} else if (totalram_pages >= RSC_MEM_DEFRAG_TWO_GB_PAGES) {
		/*ram >= 2GB, bord size is 128MB, only one zone(DMA zone), so need more bord page*/
#if defined(CONFIG_MTK_PLATFORM)
		/*
		#ifdef CONFIG_RSC_RESERVER_HIATOMIC
		mtk setting slub_max_order=0, see /proc/cmdline, so slab size always  allcate order-0 page
		Fixbug: [B190406-193] 1901EF/1901BF 3GB, reboot test fail
		com.iqoo.secur: page allocation failure: order:0, mode:0x2280020(GFP_ATOMIC|__GFP_NOTRACK)
		warn_alloc+0x114/0x148
		__alloc_pages_nodemask+0x1de0/0x380c
		new_slab+0x29c/0x2c4
		___slab_alloc+0x458/0x590
		__kmalloc+0x2e4/0x2fc
		ebitmap_cpy+0x3c/0xc4
		mls_compute_sid+0x1b4/0x2f0
		security_compute_sid+0x3e0/0x550
		security_transition_sid+0x2c/0x34
		inode_doinit_with_dentry+0x2d8/0x548
		selinux_d_instantiate+0x1c/0x24
		security_d_instantiate+0x4c/0x68
		d_instantiate+0x30/0x68
		__shmem_file_setup+0xf0/0x18c
		shmem_file_setup+0x10/0x18
		ashmem_mmap+0x74/0x10c
		mmap_region+0x4b4/0x758
		do_mmap+0x300/0x3f0
		vm_mmap_pgoff+0x9c/0xf4
		SyS_mmap_pgoff+0x74/0xe8
		sys_mmap+0x20/0x28
		cpu_switch_to+0x39c/0x1848
		#endifs
		*/
		b_block = (44*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		s_block = (192*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#else
		b_block = (108*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		s_block = (272*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif
	} else {
#if defined(CONFIG_MTK_PLATFORM)
		/* mtk setting slub_max_order=0, so slab size always  allcate order-0 page*/
		b_block = (64*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		s_block = (192*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#else
		/*  ram < 2GB, bord size  is 128MB, only one zone(DMA zone), so need more bord page*/
		b_block = (108*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		s_block = (272*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif
	}

#ifdef CONFIG_VMAP_STACK
	/*
	* kernel stack use vmalloc buffer in qcom linux kernel 4.14 for sdm855 and sdm6150 platform
	* so, we dont need such more big pool
	*/
	if (b_block > (68*SZ_1M/PAGE_SIZE/pageblock_nr_pages))
		b_block = (68*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif

#if defined(CONFIG_RSC_ISOLATE_SORD_SIZE_MB) && CONFIG_RSC_ISOLATE_SORD_SIZE_MB > 0
	s_block = (CONFIG_RSC_ISOLATE_SORD_SIZE_MB*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif

#if defined(CONFIG_RSC_ISOLATE_BORD_SIZE_MB) && CONFIG_RSC_ISOLATE_BORD_SIZE_MB > 0
	b_block = (CONFIG_RSC_ISOLATE_BORD_SIZE_MB*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
#endif

	if (mem_smallord_MB) {
		s_block = (mem_smallord_MB*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		rsc_info("mem_defrag mem_smallord_MB:%d! s_block:%lu\n", mem_smallord_MB, s_block);
	}

	if (mem_bigord_MB) {
		b_block = (mem_bigord_MB*SZ_1M/PAGE_SIZE/pageblock_nr_pages);
		rsc_info("mem_defrag mem_bigord_MB:%d! b_block:%lu\n", mem_bigord_MB, b_block);
	}

	if (is_unmovable_isolate_sord(unmovable_isolate_type))
		ui_blocks = s_block;
	else if (is_unmovable_isolate_bord(unmovable_isolate_type))
		ui_blocks = b_block;
	else
		ui_blocks = 0;

	return ui_blocks;
}

/*
 * Check if a pageblock contains reserved pages
 */
static int pageblock_is_reserved(unsigned long start_pfn, unsigned long end_pfn)
{
	unsigned long pfn;

	for (pfn = start_pfn; pfn < end_pfn; pfn++) {
		if (!pfn_valid_within(pfn) || PageReserved(pfn_to_page(pfn)))
			return 1;
	}
	return 0;
}

/*
 * Check if should setup unmovable_isolate area
 */
static int should_setup_unmovable_isolate(struct zone *zone, unsigned long ui_block,
			int disable)
{
	/* only set UNMOVABLE_ISOLATE in DMA zone */
	if (!is_dma_zone(zone) || min_wmark_pages(zone) == 0)
		return 0;
	/* just setup the unmovable-isolate once when enable */
	else if (ui_block != 0 && !disable)
		return 0;
	else
		return 1;
}

#ifdef CONFIG_SMP
/*
 * flush the item stat, this function is
 * __mod_zone_page_state remove stat_threshold.
 */
static void update_zone_page_state(struct zone *zone, enum zone_stat_item item)
{
	struct per_cpu_pageset __percpu *pcp = zone->pageset;
	s8 __percpu *p = pcp->vm_stat_diff + item;
	long x;

	x = __this_cpu_read(*p);

	zone_page_state_add(x, zone, item);
	x = 0;
	__this_cpu_write(*p, x);
}
#else
/*
 * We do not maintain differentials in a single processor configuration.
 * The functions will do nothing.
 */
static void update_zone_page_state(struct zone *zone, enum zone_stat_item item)
{
	return;
}
#endif

/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
int rsc_highatomic_reserve_pages;
/*#endif*/

/*
 * Mark a number of pageblocks as MIGRATE_UNMOVABLE_ISOLATE.
 */
int rsc_setup_zone_isolate_page(struct zone *zone,
     int unmovable_isolate_type, int disable)
{
	unsigned long start_pfn, pfn, end_pfn, block_end_pfn;
	struct page *page;
	int block_migratetype;
	unsigned long *zone_ui_block;
	long long unmovable_isolate_count, old_size;
	enum zone_stat_item ui_stat_item;
	int pages_moved = 0;
/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
	int highatomic_reserve_block = 1;
/*#endif*/

	if (is_unmovable_isolate_sord(unmovable_isolate_type)) {
		zone_ui_block = &(zone->nr_migrate_unmovable_isolate_sord_block);
		ui_stat_item = NR_FREE_UNMOVABLE_ISOLATE_SORD;
	} else if (is_unmovable_isolate_bord(unmovable_isolate_type)) {
		zone_ui_block = &(zone->nr_migrate_unmovable_isolate_bord_block);
		ui_stat_item = NR_FREE_UNMOVABLE_ISOLATE_BORD;
	} else {
		rsc_err("unknown unmovable isolate type!\n");
		return -EINVAL;
	}

	/* just setup the unmovable-isolate once when enable */
	if (!should_setup_unmovable_isolate(zone, *zone_ui_block, disable))
		return -EPERM;

	if (!disable)
		unmovable_isolate_count = get_unmovable_isolate_blocks(unmovable_isolate_type);
	else
		unmovable_isolate_count = 0;

	old_size = *zone_ui_block;
	if (unmovable_isolate_count == old_size)
		return 0;

	*zone_ui_block = unmovable_isolate_count;

	/*
	 * Get the start pfn, end pfn and the number of blocks to unmovable_isolate
	 * We have to be careful to be aligned to pageblock_nr_pages to
	 * make sure that we always check pfn_valid for the first page in
	 * the block.
	 */
	start_pfn = zone->zone_start_pfn;
	end_pfn = zone_end_pfn(zone);
	start_pfn = roundup(start_pfn, pageblock_nr_pages);

	for (pfn = start_pfn; pfn < end_pfn; pfn += pageblock_nr_pages) {
		if (!pfn_valid(pfn))
			continue;
		page = pfn_to_page(pfn);

		/* Watch out for overlapping nodes */
		if (page_to_nid(page) != zone_to_nid(zone))
			continue;

		block_migratetype = get_pageblock_migratetype(page);

		/* Only test what is necessary when the reserves are not met */
		if (unmovable_isolate_count > 0) {
			/*
			 * Blocks with reserved pages will never free, skip
			 * them.
			 */
			block_end_pfn = min(pfn + pageblock_nr_pages, end_pfn);
			if (pageblock_is_reserved(pfn, block_end_pfn))
				continue;

			/* If this block is unmovable_isolate, account for it */
			if (block_migratetype == unmovable_isolate_type) {
				unmovable_isolate_count--;
				continue;
			}

			/* Suitable for UNMOVABLE_ISOLATE if this block is movable */
			if (block_migratetype == MIGRATE_MOVABLE) {
				set_pageblock_migratetype(page,
							unmovable_isolate_type);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
				pages_moved = move_freepages_block(zone, page,
							unmovable_isolate_type, NULL);
#else
				pages_moved = move_freepages_block(zone, page,
							unmovable_isolate_type);
#endif
				__mod_zone_page_state(zone, ui_stat_item, pages_moved);
				unmovable_isolate_count--;
				continue;
			}
		} else if (!old_size) {

/*#ifdef CONFIG_RSC_RESERVER_HIATOMIC*/
#if 1
			/*
			mtk setting slub_max_order=0, see /proc/cmdline, so slab size always  allcate order-0 page,
			wer reserve 8MB HighAtomic for it.
			Fixbug: [B190406-193] 1901EF/1901BF 3GB, reboot test fail
			*/
			if ((MIGRATE_UNMOVABLE_ISOLATE_SORD == unmovable_isolate_type) && !disable) {
				/* same as reserve_highatomic_pageblock*/
				/*
				 * Limit the number reserved to 1 pageblock or roughly 1% of a zone.
				 * Check is race-prone but harmless.
				 */
				if (zone->nr_reserved_highatomic >= ((zone->managed_pages / 100) + pageblock_nr_pages))
					break;

				block_end_pfn = min(pfn + pageblock_nr_pages, end_pfn);
				if (pageblock_is_reserved(pfn, block_end_pfn))
					continue;

				if (block_migratetype == MIGRATE_MOVABLE) {
					zone->nr_reserved_highatomic += pageblock_nr_pages;
					set_pageblock_migratetype(page, MIGRATE_HIGHATOMIC);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
					pages_moved = move_freepages_block(zone, page,
								MIGRATE_HIGHATOMIC, NULL);

#else
					pages_moved = move_freepages_block(zone, page, MIGRATE_HIGHATOMIC);
#endif
					rsc_info("idx: %d reserve %d pages to MIGRATE_HIGHATOMIC\n", highatomic_reserve_block, pages_moved);
					highatomic_reserve_block--;
					rsc_highatomic_reserve_pages += pages_moved;
					if (highatomic_reserve_block)
						continue;
				} else
					continue;
#endif
			}
			/*
			 * At boot time we don't need to scan the whole zone
			 * for turning off MIGRATE_UNMOVABLE_ISOLATE.
			 */
			break;
		}

		/*
		 * If the unmovable_isolate is met and this is a previous reserved block,
		 * take it back
		 */
		if (block_migratetype == unmovable_isolate_type) {
			set_pageblock_migratetype(page, MIGRATE_MOVABLE);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
			pages_moved = move_freepages_block(zone, page, MIGRATE_MOVABLE, NULL);
#else
			pages_moved = move_freepages_block(zone, page, MIGRATE_MOVABLE);
#endif
			__mod_zone_page_state(zone, ui_stat_item, -pages_moved);
		}
	}
	update_zone_page_state(zone, ui_stat_item);

	return unmovable_isolate_count;
}

#ifdef RSC_SKIP_MEM_DEFRAG_IN_RECOVER_MODE
int rsc_recovery_survival_mode;
static int __init isrecoverymode_detection(char *str)
{
	if (*str == '1') {
		rsc_recovery_survival_mode = 1;
		rsc_info("enter recovery mode %d!!!!\n", rsc_recovery_survival_mode);
	}

	return 0;
}
__setup("recoverymode=", isrecoverymode_detection);

static int __init issurvivalmode_detection(char *str)
{
	if (*str == '1') {
		rsc_recovery_survival_mode = 2;
		rsc_info("enter surviva mode %d!!!!\n", rsc_recovery_survival_mode);
	}

	return 0;
}
__setup("survivalmode=", issurvivalmode_detection);
#endif
