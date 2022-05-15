// SPDX-License-Identifier: GPL-2.0
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif
#include <asm/page.h>
#include <asm/pgtable.h>
#include "internal.h"

void __attribute__((weak)) arch_report_meminfo(struct seq_file *m)
{
}

static void show_val_kb(struct seq_file *m, const char *s, unsigned long num)
{
	char v[32];
	static const char blanks[7] = {' ', ' ', ' ', ' ',' ', ' ', ' '};
	int len;

	len = num_to_str(v, sizeof(v), num << (PAGE_SHIFT - 10));

	seq_write(m, s, 16);

	if (len > 0) {
		if (len < 8)
			seq_write(m, blanks, 8 - len);

		seq_write(m, v, len);
	}
	seq_write(m, " kB\n", 4);
}

static int meminfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	int lru;

	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();
	sreclaimable = global_node_page_state(NR_SLAB_RECLAIMABLE);
	sunreclaim = global_node_page_state(NR_SLAB_UNRECLAIMABLE);

#ifdef CONFIG_ZRAM_WRITEBACK_VIVO
#define TOTAL_SWAP_SHOW_PAGE 1048576  // 4G
	// if not root, then a fake value.
	if ((current_uid().val != 0) && (current_uid().val != 1000)) {
		if (i.totalswap > TOTAL_SWAP_SHOW_PAGE) {
			i.totalswap = TOTAL_SWAP_SHOW_PAGE;
			if (i.freeswap > i.totalswap)
				i.freeswap = i.totalswap;
		}
	}
#endif

	show_val_kb(m, "MemTotal:       ", i.totalram);
	show_val_kb(m, "MemFree:        ", i.freeram);
	show_val_kb(m, "MemAvailable:   ", available);
	show_val_kb(m, "Buffers:        ", i.bufferram);
	show_val_kb(m, "Cached:         ", cached);
	show_val_kb(m, "SwapCached:     ", total_swapcache_pages());
	show_val_kb(m, "Active:         ", pages[LRU_ACTIVE_ANON] +
					   pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive:       ", pages[LRU_INACTIVE_ANON] +
					   pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Active(anon):   ", pages[LRU_ACTIVE_ANON]);
	show_val_kb(m, "Inactive(anon): ", pages[LRU_INACTIVE_ANON]);
	show_val_kb(m, "Active(file):   ", pages[LRU_ACTIVE_FILE]);
	show_val_kb(m, "Inactive(file): ", pages[LRU_INACTIVE_FILE]);
	show_val_kb(m, "Unevictable:    ", pages[LRU_UNEVICTABLE]);
	show_val_kb(m, "Mlocked:        ", global_zone_page_state(NR_MLOCK));

#ifdef CONFIG_HIGHMEM
	show_val_kb(m, "HighTotal:      ", i.totalhigh);
	show_val_kb(m, "HighFree:       ", i.freehigh);
	show_val_kb(m, "LowTotal:       ", i.totalram - i.totalhigh);
	show_val_kb(m, "LowFree:        ", i.freeram - i.freehigh);
#endif

#ifndef CONFIG_MMU
	show_val_kb(m, "MmapCopy:       ",
		    (unsigned long)atomic_long_read(&mmap_pages_allocated));
#endif

	show_val_kb(m, "SwapTotal:      ", i.totalswap);
	show_val_kb(m, "SwapFree:       ", i.freeswap);
	show_val_kb(m, "Dirty:          ",
		    global_node_page_state(NR_FILE_DIRTY));
	show_val_kb(m, "Writeback:      ",
		    global_node_page_state(NR_WRITEBACK));
	show_val_kb(m, "AnonPages:      ",
		    global_node_page_state(NR_ANON_MAPPED));
	show_val_kb(m, "Mapped:         ",
		    global_node_page_state(NR_FILE_MAPPED));
	show_val_kb(m, "Shmem:          ", i.sharedram);
	show_val_kb(m, "KReclaimable:   ", sreclaimable +
		    global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE));
	show_val_kb(m, "Slab:           ", sreclaimable + sunreclaim);
	show_val_kb(m, "SReclaimable:   ", sreclaimable);
	show_val_kb(m, "SUnreclaim:     ", sunreclaim);
	seq_printf(m, "KernelStack:    %8lu kB\n",
		   global_zone_page_state(NR_KERNEL_STACK_KB));
#ifdef CONFIG_SHADOW_CALL_STACK
	seq_printf(m, "ShadowCallStack:%8lu kB\n",
		   global_zone_page_state(NR_KERNEL_SCS_BYTES) / 1024);
#endif
	show_val_kb(m, "PageTables:     ",
		    global_zone_page_state(NR_PAGETABLE));
#ifdef CONFIG_QUICKLIST
	show_val_kb(m, "Quicklists:     ", quicklist_total_size());
#endif

	show_val_kb(m, "NFS_Unstable:   ",
		    global_node_page_state(NR_UNSTABLE_NFS));
	show_val_kb(m, "Bounce:         ",
		    global_zone_page_state(NR_BOUNCE));
	show_val_kb(m, "WritebackTmp:   ",
		    global_node_page_state(NR_WRITEBACK_TEMP));
	show_val_kb(m, "CommitLimit:    ", vm_commit_limit());
	show_val_kb(m, "Committed_AS:   ", committed);
	seq_printf(m, "VmallocTotal:   %8lu kB\n",
		   (unsigned long)VMALLOC_TOTAL >> 10);
	show_val_kb(m, "VmallocUsed:    ", vmalloc_nr_pages());
	show_val_kb(m, "VmallocChunk:   ", 0ul);

#ifdef CONFIG_MEMORY_FAILURE
	seq_printf(m, "HardwareCorrupted: %5lu kB\n",
		   atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10));
#endif

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	show_val_kb(m, "AnonHugePages:  ",
		    global_node_page_state(NR_ANON_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemHugePages: ",
		    global_node_page_state(NR_SHMEM_THPS) * HPAGE_PMD_NR);
	show_val_kb(m, "ShmemPmdMapped: ",
		    global_node_page_state(NR_SHMEM_PMDMAPPED) * HPAGE_PMD_NR);
#endif

#ifdef CONFIG_CMA
	show_val_kb(m, "CmaTotal:       ", totalcma_pages);
	show_val_kb(m, "CmaFree:        ",
		    global_zone_page_state(NR_FREE_CMA_PAGES));
#endif

	hugetlb_report_meminfo(m);

	arch_report_meminfo(m);

	return 0;
}

void meminfo_printk_show(void)
{
	struct sysinfo i;
	unsigned long committed;
	long cached;
	long available;
	unsigned long pages[NR_LRU_LISTS];
	unsigned long sreclaimable, sunreclaim;
	int lru;
#ifdef CONFIG_RSC_MEM_STAT
	long total_use;
	long pcppages;
#endif
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);

	cached = global_node_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);

	available = si_mem_available();
	sreclaimable = global_node_page_state(NR_SLAB_RECLAIMABLE);
	sunreclaim = global_node_page_state(NR_SLAB_UNRECLAIMABLE);

#ifdef CONFIG_RSC_MEM_STAT
	pcppages = rsc_get_all_pcppages();
	total_use = pcppages + i.freeram  +  global_node_page_state(NR_ANON_MAPPED) +
	global_node_page_state(NR_FILE_PAGES) +
	global_node_page_state(NR_SLAB_RECLAIMABLE) +
	global_node_page_state(NR_SLAB_UNRECLAIMABLE) +
	(global_zone_page_state(NR_KERNEL_STACK_KB) >> (PAGE_SHIFT-10)) +
	global_zone_page_state(NR_PAGETABLE);

	total_use += global_zone_page_state(NR_KMALLOC) +
		global_zone_page_state(NR_VMALLOC) +
		global_zone_page_state(NR_DMA_NORMAL) +
		global_zone_page_state(NR_DMA_CMA) +
		global_zone_page_state(NR_ION) +
		global_zone_page_state(NR_GPU)
#ifdef CONFIG_ZSMALLOC
			+ rsc_zram_pages()
#endif
#ifdef CONFIG_ZCACHE
			+ (long)zcache_pages()
#endif
			;
#endif
	pr_info("MemTotal:       %lu kB\n", i.totalram << (PAGE_SHIFT - 10));
	pr_info("MemFree:        %lu kB\n", i.freeram << (PAGE_SHIFT - 10));
	pr_info("MemAvailable:   %lu kB\n", available << (PAGE_SHIFT - 10));
	pr_info("Buffers:        %lu kB\n", i.bufferram << (PAGE_SHIFT - 10));
	pr_info("Cached:         %lu kB\n", cached << (PAGE_SHIFT - 10));
	pr_info("SwapCached:     %lu kB\n", total_swapcache_pages() << (PAGE_SHIFT - 10));
	pr_info("Active:         %lu kB\n", (pages[LRU_ACTIVE_ANON] + pages[LRU_ACTIVE_FILE]) << (PAGE_SHIFT - 10));
	pr_info("Inactive:       %lu kB\n", (pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]) << (PAGE_SHIFT - 10));
	pr_info("Active(anon):   %lu kB\n", pages[LRU_ACTIVE_ANON] << (PAGE_SHIFT - 10));
	pr_info("Inactive(anon): %lu kB\n", pages[LRU_INACTIVE_ANON] << (PAGE_SHIFT - 10));
	pr_info("Active(file):   %lu kB\n", pages[LRU_ACTIVE_FILE] << (PAGE_SHIFT - 10));
	pr_info("Inactive(file): %lu kB\n", pages[LRU_INACTIVE_FILE] << (PAGE_SHIFT - 10));
	pr_info("Unevictable:    %lu kB\n", pages[LRU_UNEVICTABLE] << (PAGE_SHIFT - 10));
	pr_info("Mlocked:        %lu kB\n", global_zone_page_state(NR_MLOCK) << (PAGE_SHIFT - 10));

#ifdef CONFIG_HIGHMEM
	pr_info("HighTotal:      %lu kB\n", i.totalhigh << (PAGE_SHIFT - 10));
	pr_info("HighFree:       %lu kB\n", i.freehigh << (PAGE_SHIFT - 10));
	pr_info("LowTotal:       %lu kB\n", (i.totalram - i.totalhigh) << (PAGE_SHIFT - 10));
	pr_info("LowFree:        %lu kB\n", (i.freeram - i.freehigh) << (PAGE_SHIFT - 10));
#endif

#ifndef CONFIG_MMU
	pr_info("MmapCopy:       %lu kB\n",
		    (unsigned long)atomic_long_read(&mmap_pages_allocated) << (PAGE_SHIFT - 10));
#endif

	pr_info("SwapTotal:      %lu kB\n", i.totalswap << (PAGE_SHIFT - 10));
	pr_info("SwapFree:       %lu kB\n", i.freeswap << (PAGE_SHIFT - 10));
	pr_info("Dirty:          %lu kB\n",
		    global_node_page_state(NR_FILE_DIRTY) << (PAGE_SHIFT - 10));
	pr_info("Writeback:      %lu kB\n",
		    global_node_page_state(NR_WRITEBACK) << (PAGE_SHIFT - 10));
	pr_info("AnonPages:      %lu kB\n",
		    global_node_page_state(NR_ANON_MAPPED) << (PAGE_SHIFT - 10));
	pr_info("Mapped:         %lu kB\n",
		    global_node_page_state(NR_FILE_MAPPED) << (PAGE_SHIFT - 10));
	pr_info("Shmem:          %lu kB\n", i.sharedram << (PAGE_SHIFT - 10));
	pr_info("KReclaimable:   %lu kB\n", (sreclaimable +
		    global_node_page_state(NR_KERNEL_MISC_RECLAIMABLE)) << (PAGE_SHIFT - 10));
	pr_info("Slab:           %lu kB\n", (sreclaimable + sunreclaim) << (PAGE_SHIFT - 10));
	pr_info("SReclaimable:   %lu kB\n", sreclaimable << (PAGE_SHIFT - 10));
	pr_info("SUnreclaim:     %lu kB\n", sunreclaim << (PAGE_SHIFT - 10));
	pr_info("KernelStack:    %lu kB\n",
		   global_zone_page_state(NR_KERNEL_STACK_KB));
#ifdef CONFIG_SHADOW_CALL_STACK
	pr_info("ShadowCallStack:%lu kB\n",
		   global_zone_page_state(NR_KERNEL_SCS_BYTES) / 1024);
#endif
	pr_info("PageTables:     %lu kB\n",
		    global_zone_page_state(NR_PAGETABLE) << (PAGE_SHIFT - 10));
#ifdef CONFIG_QUICKLIST
	pr_info("Quicklists:     %lu kB\n", quicklist_total_size() << (PAGE_SHIFT - 10));
#endif

	pr_info("NFS_Unstable:   %lu kB\n",
		    global_node_page_state(NR_UNSTABLE_NFS) << (PAGE_SHIFT - 10));
	pr_info("Bounce:         %lu kB\n",
		    global_zone_page_state(NR_BOUNCE) << (PAGE_SHIFT - 10));
	pr_info("WritebackTmp:   %lu kB\n",
		    global_node_page_state(NR_WRITEBACK_TEMP) << (PAGE_SHIFT - 10));
	pr_info("CommitLimit:    %lu kB\n", vm_commit_limit() << (PAGE_SHIFT - 10));
	pr_info("Committed_AS:   %lu kB\n", committed << (PAGE_SHIFT - 10));
	pr_info("VmallocTotal:   %8lu kB\n",
		   (unsigned long)VMALLOC_TOTAL >> 10);
	pr_info("VmallocUsed:    %lu kB\n", vmalloc_nr_pages() << (PAGE_SHIFT - 10));
	pr_info("VmallocChunk:   %lu kB\n", 0ul);

#ifdef CONFIG_MEMORY_FAILURE
	pr_info("HardwareCorrupted: %5lu kB\n",
		   atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10));
#endif

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	pr_info("AnonHugePages:  %lu kB\n",
		   (global_node_page_state(NR_ANON_THPS) * HPAGE_PMD_NR) << (PAGE_SHIFT - 10));
	pr_info("ShmemHugePages: %lu kB\n",
		    (global_node_page_state(NR_SHMEM_THPS) * HPAGE_PMD_NR) << (PAGE_SHIFT - 10));
	pr_info("ShmemPmdMapped: %lu kB\n",
		    (global_node_page_state(NR_SHMEM_PMDMAPPED) * HPAGE_PMD_NR) << (PAGE_SHIFT - 10));
#endif

#ifdef CONFIG_CMA
	pr_info("CmaTotal:       %lu kB\n", totalcma_pages << (PAGE_SHIFT - 10));
	pr_info("CmaFree:        %lu kB\n",
		    global_zone_page_state(NR_FREE_CMA_PAGES) << (PAGE_SHIFT - 10));
#endif
	if ((current_uid().val == 0) || (current_uid().val == 1000)) {
#ifdef CONFIG_RSC_MEM_DEFRAG
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		pr_info("RSCISF:         %lu kB\n", global_zone_page_state(NR_FREE_UNMOVABLE_ISOLATE_SORD) << (PAGE_SHIFT - 10));
		pr_info("RSCIBF:         %lu kB\n", global_zone_page_state(NR_FREE_UNMOVABLE_ISOLATE_BORD) << (PAGE_SHIFT - 10));
#else
		pr_info("RSCISF:         %lu kB\n", global_page_state(NR_FREE_UNMOVABLE_ISOLATE_SORD) << (PAGE_SHIFT - 10));
		pr_info("RSCIBF:         %lu kB\n", global_page_state(NR_FREE_UNMOVABLE_ISOLATE_BORD) << (PAGE_SHIFT - 10));
#endif
#endif

#ifdef CONFIG_RSC_MEM_STAT
		pr_info("NR_KMALLOC:     %lu kB\n", global_zone_page_state(NR_KMALLOC) << (PAGE_SHIFT - 10));

#ifdef CONFIG_RSC_MEM_STAT_DEBUG
		pr_info("kmallocleak: 	%lu kB\n", atomic_long_read(&rsc_kmalloc_leak) << (PAGE_SHIFT - 10));
#endif
		pr_info("NR_VMALLOC:     %lu kB\n", global_zone_page_state(NR_VMALLOC) << (PAGE_SHIFT - 10));
		pr_info("NR_DMA_NOR:     %lu kB\n", global_zone_page_state(NR_DMA_NORMAL) << (PAGE_SHIFT - 10));
		pr_info("NR_DMA_CMA:     %lu kB\n", global_zone_page_state(NR_DMA_CMA) << (PAGE_SHIFT - 10));
		pr_info("NR_ION:         %lu kB\n", global_zone_page_state(NR_ION) << (PAGE_SHIFT - 10));
		// static func ion_page_pool_nr_pages retrun 0 in this file. so change ion_page_pool_nr_pages to 0
		pr_info("free_ion:       %lu kB\n", (atomic_long_read(&rsc_ion_head_free_list_size) >> PAGE_SHIFT) << (PAGE_SHIFT - 10));
		pr_info("free_ion_pool:  %lu kB\n", 0);
		pr_info("free_ion_heap:  %lu kB\n", (atomic_long_read(&rsc_ion_head_free_list_size) >> PAGE_SHIFT));
		pr_info("NR_GPU:         %lu kB\n", global_zone_page_state(NR_GPU) << (PAGE_SHIFT - 10));
		pr_info("free_gpu:       %lu kB\n", (unsigned long)rsc_kgsl_pool_size_total() << (PAGE_SHIFT - 10));
#ifdef CONFIG_ZSMALLOC
		pr_info("zram_size:      %lu kB\n", rsc_zram_pages() << (PAGE_SHIFT - 10));
#endif
#ifdef CONFIG_ZCACHE
		pr_info("zcache_size:    %lu kB\n", (unsigned long)zcache_pages() << (PAGE_SHIFT - 10));
#endif
		pr_info("pcppages:       %lu kB\n", pcppages << (PAGE_SHIFT - 10));
		pr_info("ALL_MEM:        %lu kB\n", (unsigned long)total_use << (PAGE_SHIFT - 10));
#endif
	}
}

static int meminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_proc_show, NULL);
}

static const struct file_operations meminfo_proc_fops = {
	.open		= meminfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_meminfo_init(void)
{
	proc_create("meminfo", 0, NULL, &meminfo_proc_fops);
	return 0;
}
fs_initcall(proc_meminfo_init);
