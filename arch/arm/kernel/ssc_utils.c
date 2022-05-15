#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/syscalls.h>

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/sort.h>

#define SEC_SYSCALL_INCLUDE_HEADER_FILES
#include "ssc.hpp"
#undef SEC_SYSCALL_INCLUDE_HEADER_FILES


/*#define CONFIG_RS_BOOT_DEBUG*/ /*调试相关异常请打开这个*/

#if !defined(CONFIG_RS_BOOT_DEBUG)
	#define DEFAULT_RS_LOG_ENABLE (0) /*默认是否开调试log*/
#else
	#define DEFAULT_RS_LOG_ENABLE (1)
#endif

#if !defined(CONFIG_RS_BOOT_DEBUG)
	#define RS_DEBUG (0)  /*是否打开调试log功能*/
#else
	#define RS_DEBUG (1)
#endif

#define RS_TEMP_LOG (1)  /*是否打开临时log功能, DLOG("xxx")*/

#if (!RS_DEBUG)
	#undef CONFIG_INIT_DELAY_RS_LOG
	#undef CONFIG_CONFIG_RS_LOG
#endif

/*//////////////////////////////////////////////*/

#if defined(CONFIG_64BIT) /*(__LONG_MAX__ > 2147483647L) || (BITS_PER_LONG > 32)*/
	#define RS_64BIT_SUPPORT /*64位系统支持*/
#endif

#if defined(CONFIG_ARCH_EXYNOS)
	#define RS_SAMSANG_PLATFORM

	#if !defined(CONFIG_DEBUG_SNAPSHOT_USER_MODE)
		#define RS_IS_ENG_BUILD
	#endif

#elif defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM)
	#define RS_MTK_PLATFORM

/*比较按不同TARGET_BUILD_VARIANT编译出来的 .config 文件*/
#if defined(CONFIG_MT_ENG_BUILD) \
	|| (defined(CONFIG_OPROFILE) && defined(CONFIG_KPROBES) && defined(CONFIG_KGDB)) /*eng, 老版本无CONFIG_MT_ENG_BUILD*/
	/*|| (defined(CONFIG_MTK_SCHED_TRACERS) && defined(CONFIG_RCU_TRACE))*/ /*userdebug*/
	#define RS_IS_ENG_BUILD
#endif

#elif defined(CONFIG_ARCH_MSM) || defined(CONFIG_ARCH_QCOM)
	#define RS_QUALCOMM_PLATFORM

/*比较arch/arm/configs/下xxxx-perf_defconfig(user版)与xxxx_defconfig(debug版)的区别,userdebug与eng同*/
#if (!defined(CONFIG_ARM64) && defined(CONFIG_CORESIGHT) && defined(CONFIG_DEBUG_KMEMLEAK)) \
		|| (defined(CONFIG_DEBUG_KMEMLEAK) && defined(CONFIG_DEBUG_LIST) && defined(CONFIG_FAULT_INJECTION))
	#define RS_IS_ENG_BUILD
#endif

	#if defined(CONFIG_LOCALVERSION)
		/*something like CONFIG_LOCALVERSION="-perf"*/
		#undef RS_IS_ENG_BUILD
	#endif

#endif

#if defined(CONFIG_KALLSYMS) && !defined(RS_IS_ENG_BUILD)
	#define CONFIG_RS_OBFUSCATED_NAME /*变量、函数名字混淆化，被别的文件引用的不能混淆，暂时动态修改*/
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
	#if defined(CONFIG_ARCH_HAS_STRICT_KERNEL_RWX) && defined(CONFIG_STRICT_KERNEL_RWX)
		#define CONFIG_DEBUG_RODATA
	#endif
#endif

#if defined(CONFIG_DEBUG_RODATA)
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)) || defined(RS_DEBUG_RODATA_BACKPORT) /*&& defined(RS_QUALCOMM_PLATFORM)*/ /*MTK 6750 6.0 也开了 CONFIG_DEBUG_RODATA*/
		#define RS_SET_PAGE_BY_FIXMAP

		#if !defined(RS_64BIT_SUPPORT) && defined(CONFIG_ARM_KERNMEM_PERMS)
			#undef RS_SET_PAGE_BY_FIXMAP
			/*#define RS_CAN_NOT_MODIFY_RODATA*/ /*can't modify ro data after mark_rodata_ro()/mark_readonly() called, even using fixmap or set_kernel_text_rw*/
			/*#define RS_FIXMAP_DIRECT_COPY_TO_RODATA*/ /*can't modify ro data through fixmapped addr*/
		#endif
	#endif
#endif

#define RS_IS_ANDROID_10_ABOVE

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
#include <linux/uidgid.h>
#endif

/*#include <linux/xattr.h>*/
/*#include <linux/mtd/mtd.h>*/

#include <linux/random.h>

#if defined(CONFIG_ARM)
#include <linux/highmem.h>
#endif

#include <linux/vmalloc.h>
#include <linux/module.h>
/*#include <linux/moduleloader.h>*/

/*#include <linux/crc32.h>*/

/*#include <linux/utsname.h>*/
/*#include <linux/time.h>*/
/*#include <linux/timex.h>*/
/*#include <linux/rtc.h>*/

#include <asm/sections.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#if defined(RS_SET_PAGE_BY_FIXMAP)
#include <asm/fixmap.h>
#endif

/*/////////////////////////////////////////////*/
#if RS_DEBUG

#define RS_LOG_LEVEL KERN_ALERT /*KERN_DEBUG*/

#define RS_DBG_EVT_NONE        (0)       /* No event */

#define RS_DBG_EVT_EMERG       (1 << 0)  /* system is unusable                   */
#define RS_DBG_EVT_ALERT       (1 << 1)  /* action must be taken immediately     */
#define RS_DBG_EVT_CRIT        (1 << 2)  /* critical conditions                  */
#define RS_DBG_EVT_ERR         (1 << 3)  /* error conditions                     */
#define RS_DBG_EVT_WARN        (1 << 4)  /* warning conditions                   */
#define RS_DBG_EVT_NOTICE      (1 << 5)  /* normal but significant condition     */
#define RS_DBG_EVT_INFO        (1 << 6)  /* informational                        */
#define RS_DBG_EVT_DBG         (1 << 7)  /* debug-level messages                 */

#define RS_DBG_EVT_ALL         (0xffffffff)

#define RS_DBG_EVT_MASK        (RS_DBG_EVT_ALL)


#if defined(CONFIG_CONFIG_RS_LOG) || defined(CONFIG_INIT_DELAY_RS_LOG)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_enable_log RS_HIDE(_26)
#endif
noused static unsigned long g_rs_enable_log
#if !defined(CONFIG_INIT_DELAY_RS_LOG)
	= DEFAULT_RS_LOG_ENABLE
#endif
	;

	#define RS_LOG(fmt, args...) \
	do { \
		if (g_rs_enable_log) \
		printk(RS_LOG_LEVEL "rs: " fmt, ## args); \
	} while (0)



	#define RS_XLOG(evt, fmt, args...) \
	do { \
		if ((g_rs_enable_log) && ((RS_DBG_EVT_##evt) & RS_DBG_EVT_MASK)) \
		printk(RS_LOG_LEVEL "rs: "fmt, ## args); \
	} while (0)

#else
	#define RS_LOG(fmt, args...)    printk(RS_LOG_LEVEL "rs: " fmt, ## args)

	#define RS_XLOG(evt, fmt, args...) \
	do { \
		if ((RS_DBG_EVT_##evt) & RS_DBG_EVT_MASK) \
		printk(RS_LOG_LEVEL "rs: " fmt, ## args); \
	} while (0)
#endif

	/*#define  RS_LOG(...)    printk(__VA_ARGS__)*/
	/*#define  RS_LOG(...)    xlog_printk(ANDROID_LOG_DEBUG, "RS_TAG", __VA_ARGS__);*/
#else
	#define RS_LOG(...)    do {} while (0)
	#define RS_XLOG(...)    do {} while (0)
#endif

#undef DLOG

#if RS_TEMP_LOG
	#define DLOG(fmt, args...)    printk("rsd:" fmt, ## args)
#else
	#define DLOG(...)    do {} while (0)
#endif
/*///////////////////////////////////////////////*/

#if defined(RS_QUALCOMM_PLATFORM) && !defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
#include <asm/mmu_writeable.h> /*_MMU_WRITEABLE_H*/
#endif
#endif

/*#include <asm/rodata.h>*/ /*_ASMARM_RODATA_H got defined, and rodata.h included in cacheflush.h*/
/*#include <asm/pgtable.h>*/

/*#include "./proc/internal.h"*/

/*#include <linux/futex.h>*/
#include <linux/random.h>
#include <linux/mm.h>


/*#include <net/sock.h>*/
/*#include <linux/socket.h>*/
#include <linux/un.h>
#include <linux/poll.h>
#include <linux/file.h>
#include <linux/mman.h>

#include <linux/bitmap.h>
#include <linux/bitops.h>

#include <linux/io.h>
#if defined(CONFIG_OF)
#include <linux/of.h>
#endif
#if defined(CONFIG_OF_ADDRESS)
#include <linux/of_address.h>
#endif

#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
#include <linux/kthread.h>
#endif

#if defined(CONFIG_KASAN)
#include <linux/kasan.h>
#endif

#include <linux/namei.h> /* for kern_path() */
#include <linux/fs.h> /* for struct file_path */
#include <linux/stat.h> /* for S_ISDIR */


#undef RS_HAS_QCOM_SECURE_BUFFER

#if defined(RS_QUALCOMM_PLATFORM) && defined(CONFIG_MSM_SECURE_BUFFER) && defined(CONFIG_MSM_KERNEL_PROTECT)
#include <soc/qcom/secure_buffer.h>

	#define RS_HAS_QCOM_SECURE_BUFFER
#endif

#ifndef intptr_t
typedef long intptr_t;
#endif

#if !defined(__rs_weak)
	#if defined(__weak)
		#define	__rs_weak	__weak
	#else
		#define	__rs_weak	__attribute__((weak))
	#endif
#endif

#if 1 /*defined(CONFIG_RS_OBFUSCATED_NAME)*/
/*有可能把user版判为eng版？*/
#define RS_HIDE(name) name##_v_r_s_h_s
#else
#define RS_HIDE(name) name
#endif

noused notrace static int __is_kernel_rodata(unsigned long addr)
{
	extern char __start_rodata[];
	extern char __end_rodata[];
	extern char __start___ex_table[];
	extern char __stop___ex_table[];
	extern char __start_notes[];
	extern char __stop_notes[];

	return ((addr >= PAGE_ALIGN((unsigned long)__start_rodata) &&
		addr < PAGE_ALIGN((unsigned long)__end_rodata))
		|| (addr >= PAGE_ALIGN((unsigned long)__start___ex_table) &&
		addr < PAGE_ALIGN((unsigned long)__stop___ex_table))
		|| (addr >= PAGE_ALIGN((unsigned long)__start_notes) &&
		addr < PAGE_ALIGN((unsigned long)__stop_notes))
		);
}

#if defined(GFP_TEMPORARY)
	#define RS_KMALLOC_FLAG	GFP_TEMPORARY
#else
	#define RS_KMALLOC_FLAG	GFP_KERNEL
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kmalloc RS_HIDE(_1)
#endif
noused notrace static void *rs_kmalloc(size_t size)
{
	void *ret = kmalloc(size, RS_KMALLOC_FLAG);

	if (!ret) {
		/*!wait and use emergency pool*/
		ret = kmalloc(size, GFP_ATOMIC);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kfree RS_HIDE(_2)
#endif
noused notrace static void rs_kfree(const void *buf)
{
	kfree(buf);
}


#if defined(CONFIG_MODULES)
extern void *module_alloc(unsigned long size);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)
extern void module_memfree(void *module_region);

#if 0
void __weak module_free(noused struct module *mod, void *module_region)
{
	vfree(module_region);
}
#endif

#define RS_MODULE_FREE(module_region) module_memfree((module_region))

#else /* LINUX_VERSION_CODE >= 3.18.24 */
extern void module_free(struct module *mod, void *module_region);

#define RS_MODULE_FREE(module_region) module_free(NULL, (module_region))

#endif /* LINUX_VERSION_CODE < 3.18.24 */

#else /* CONFIG_MODULES */

#if !defined(CONFIG_ARM64)
#ifdef CONFIG_XIP_KERNEL
/*
 * The XIP kernel text is mapped in the module area for modules and
 * some other stuff to work without any indirect relocations.
 * MODULES_VADDR is redefined here and not in asm/memory.h to avoid
 * recompiling the whole kernel when CONFIG_XIP_KERNEL is turned on/off.
 */
#undef MODULES_VADDR
#define MODULES_VADDR   (((unsigned long)_etext + ~PMD_MASK) & PMD_MASK)
#endif
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define module_alloc RS_HIDE(_32_)
#endif
/*noused void *__weak module_alloc(unsigned long size)*/ /*__rs_alias(__module_alloc)*/
noused notrace static void *module_alloc(unsigned long size)
{
#if defined(CONFIG_MODULES) && defined(MODULES_VADDR)
	void *p;

	p = __vmalloc_node_range(size,
	#if defined(MODULE_ALIGN)
		MODULE_ALIGN,
	#else
		1,
	#endif
		MODULES_VADDR, MODULES_END,
		GFP_KERNEL, PAGE_KERNEL_EXEC, NUMA_NO_NODE,
		__builtin_return_address(0));

#if defined(CONFIG_KASAN)
	if (p && (kasan_module_alloc(p, size) < 0)) {
		vfree(p);
		return NULL;
	}
#endif

	return p;
#else
	return vmalloc_exec(size);
#endif
}
/*
#if defined(CONFIG_RS_OBFUSCATED_NAME)
__weak_alias(module_alloc, RS_HIDE(_32_));
#else
__weak_alias(module_alloc, _module_alloc);
#endif
*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 24)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define module_memfree RS_HIDE(_33_)
#endif
/*noused void __weak module_memfree(void *module_region)*/
noused notrace static void module_memfree(void *module_region)
{
	vfree(module_region);
}

#define RS_MODULE_FREE(module_region) module_memfree((module_region))

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define module_free RS_HIDE(_33_)
#endif
/*noused void __weak module_memfree(void *module_region)*/
noused notrace static void module_memfree(void *module_region)
{
	vfree(module_region);
}

#define RS_MODULE_FREE(module_region) module_memfree((module_region))

#endif

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_vmalloc_addr RS_HIDE(_33_0)
#endif
/* copy from vmalloc_to_page() */
noused notrace static bool rs_is_vmalloc_addr(const void *vmalloc_addr)
{
	unsigned long addr = (unsigned long)vmalloc_addr;
	bool ret = false;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
	pgd_t *pgd = pgd_offset_k(addr);
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep, pte;

	/*VIRTUAL_BUG_ON(!is_vmalloc_or_module_addr(vmalloc_addr));*/

	if (pgd_none(*pgd))
		goto out;
	p4d = p4d_offset(pgd, addr);
	if (p4d_none(*p4d))
		goto out;
	pud = pud_offset(p4d, addr);

	if (pud_none(*pud) || pud_bad(*pud))
		goto out;
	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		goto out;

	ptep = pte_offset_map(pmd, addr);
	pte = *ptep;
	if (pte_present(pte))
		/*if (pte_page(pte))*/
			ret = true;
		/*ret = (pte_page(pte) != NULL);*/
	pte_unmap(ptep);
out:
	return ret;
#else
	pgd_t *pgd = pgd_offset_k(addr);

	/*VIRTUAL_BUG_ON(!is_vmalloc_or_module_addr(vmalloc_addr));*/

	if (!pgd_none(*pgd)) {
		pud_t *pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd_t *pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte_t *ptep, pte;

				ptep = pte_offset_map(pmd, addr);
				pte = *ptep;
				if (pte_present(pte))
					/*if (pte_page(pte))*/
						ret = true;
					/*ret = (pte_page(pte) != NULL);*/
				pte_unmap(ptep);
			}
		}
	}

	return ret;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_module_address RS_HIDE(_33_1)
#endif
noused notrace static bool rs_is_module_address(unsigned long addr)
{
	/* only applicable for real module related code sections */
	/*return is_module_address(addr);*/
	/* only check for [MODULES_VADDR,MODULES_END) or [VMALLOC_START,VMALLOC_END) */
	/*return is_vmalloc_or_module_addr((const void *)addr);*/
	/*return (vmalloc_to_page((void *)addr) != NULL);*/
	return rs_is_vmalloc_addr((const void *)addr);
}


/*//////////////////////////////////////////*/
#if defined(RS_HAS_QCOM_SECURE_BUFFER)

/*
refer to msm_protect_kernel() in drivers/soc/qcom/kernel_protect.c, called as early_initcall
in msm 3.18 kernel, msm_protect_kernel() depends on alloc_secure_shared_memory() which called as pure_initcall,
pure_initcall is called after early_initcall, so msm_protect_kernel() always fail acctually
as qcom_secure_mem is not allocated
*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_msm_kernel_protect_test_data RS_HIDE(_f0_1_1)
#endif
noused static int g_rs_msm_kernel_protect_test_data __attribute__((__section__(".text")));

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_msm_kernel_protected RS_HIDE(_f0_1_2)
#endif
noused static int g_rs_msm_kernel_protected;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_msm_kernel_protected RS_HIDE(_f0_1_3)
#endif
noused notrace static int __init rs_check_msm_kernel_protected(void)
{
	/*without addrlimit checking*/
	if (!IS_ENABLED(CONFIG_MSM_KERNEL_PROTECT_MPU)) {
		/* MPU protected kernel code is HLOS writable
		 * refer to msm_protect_kernel()/msm_protect_kernel_test()/KERNEL_PROTECT_MPU
		 */
		if (__put_user(1, &g_rs_msm_kernel_protect_test_data)) {
			/*error, can't write to text region*/
			g_rs_msm_kernel_protected = 1;
		}
	}

	return 0;
}
/*pure_initcall called after early_initcall*/
pure_initcall(rs_check_msm_kernel_protected);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_kernel_text_pages_ro RS_HIDE(_f0_1)
#endif
noused notrace static int set_kernel_text_pages_ro(unsigned long start_addr, unsigned long end_addr)
{
	unsigned long virt_start_rounded;
	unsigned long virt_end_rounded;

	/*extern struct pid *cad_pid;*/ /*declared in include/linux/sched.h*/
	if (!cad_pid) {
		/*cad_pid got assigned before do_pre_smp_initcalls() called, so msm_protect_kernel() not called yet*/
		return 0;
	} else if (!g_rs_msm_kernel_protected) {
		return 0;
	} else if (IS_ENABLED(CONFIG_MSM_KERNEL_PROTECT_MPU)) {
		return 0;
	}

	virt_start_rounded = round_down((unsigned long)_stext, PAGE_SIZE);
	virt_end_rounded = round_up((unsigned long)_etext, PAGE_SIZE);

	if ((start_addr >= virt_start_rounded) && (start_addr < virt_end_rounded)) {
		phys_addr_t phys_addr = __pa(start_addr);

		u32 vmid_hlos = VMID_HLOS;
		int dest_perms = (PERM_READ | PERM_EXEC);
		int ret;

		if (end_addr > virt_end_rounded)
			end_addr = virt_end_rounded;

		/*hyp_assign_phys() calls XXalloc(..., GFP_KERNEL), thus should be called in preemptable context*/
		ret = hyp_assign_phys(phys_addr, /*addr*/
				(end_addr - start_addr), /*PAGE_SIZE,*/ /*size*/
				&vmid_hlos, /*source_vmlist*/
				1, /*source_nelems*/
				&vmid_hlos, /*dest_vmids*/
				&dest_perms, /*dest_perms*/
				1 /*dest_nelems*/
				);

		if (ret == -EIO) {
			/*scm_call failed: func id 0x42000c16, ret: -1, syscall returns: 0x0, 0x0, 0x0*/
			/*hyp_assign_table: Failed to assign memory protection, ret = -5*/
			ret = 0;
		}

		return ret;
	} else {
		return 0; /*-EFAULT;*/
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_kernel_text_pages_rw RS_HIDE(_f0_2)
#endif
noused notrace static int set_kernel_text_pages_rw(unsigned long start_addr, unsigned long end_addr)
{
	unsigned long virt_start_rounded;
	unsigned long virt_end_rounded;

	/*extern struct pid *cad_pid;*/ /*declared in include/linux/sched.h*/
	if (!cad_pid) {
		/*cad_pid got assigned before do_pre_smp_initcalls() called, so msm_protect_kernel() not called yet*/
		return 0;
	} else if (!g_rs_msm_kernel_protected) {
		return 0;
	} else if (IS_ENABLED(CONFIG_MSM_KERNEL_PROTECT_MPU)) {
		return 0;
	}

	virt_start_rounded = round_down((unsigned long)_stext, PAGE_SIZE);
	virt_end_rounded = round_up((unsigned long)_etext, PAGE_SIZE);

	if ((start_addr >= virt_start_rounded) && (start_addr < virt_end_rounded)) {
		phys_addr_t phys_addr = __pa(start_addr);

		u32 vmid_hlos = VMID_HLOS;
		int dest_perms = (PERM_READ | PERM_WRITE | PERM_EXEC);
		int ret;

		if (end_addr > virt_end_rounded)
			end_addr = virt_end_rounded;

		/*hyp_assign_phys() calls XXalloc(..., GFP_KERNEL), thus should be called in preemptable context*/
		ret = hyp_assign_phys(phys_addr, /*addr*/
				(end_addr - start_addr), /*PAGE_SIZE,*/ /*size*/
				&vmid_hlos, /*source_vmlist*/
				1, /*source_nelems*/
				&vmid_hlos, /*dest_vmids*/
				&dest_perms, /*dest_perms*/
				1 /*dest_nelems*/
				);

		if (ret == -EIO) {
			/*scm_call failed: func id 0x42000c16, ret: -1, syscall returns: 0x0, 0x0, 0x0*/
			/*hyp_assign_table: Failed to assign memory protection, ret = -5*/
			ret = 0;
		}

		return ret;
	} else {
		return 0; /*-EFAULT;*/
	}
}
#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_kernel_text_pages_ro RS_HIDE(_f0_1)
#endif
noused notrace static int set_kernel_text_pages_ro(unsigned long start_addr, unsigned long end_addr)
{
#if defined(CONFIG_ARM_KERNMEM_PERMS) && defined(CONFIG_DEBUG_RODATA)
	/*extern void set_kernel_text_ro(void);*/

	/*set_kernel_text_ro();*/ /*will trigger KE when access mm->page_table_lock*/
	/*flush_tlb_kernel_page(_addr & PAGE_MASK);*/
	return 0;
#else
	return 0; /*-ENOSYS;*/
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_kernel_text_pages_rw RS_HIDE(_f0_2)
#endif
noused notrace static int set_kernel_text_pages_rw(unsigned long start_addr, unsigned long end_addr)
{
#if defined(CONFIG_ARM_KERNMEM_PERMS) && defined(CONFIG_DEBUG_RODATA)
	/*extern void set_kernel_text_rw(void);*/

	/*set_kernel_text_rw();*/ /*will trigger KE when access mm->page_table_lock*/
	/*flush_tlb_kernel_page(_addr & PAGE_MASK);*/
	return 0;
#else
	return 0; /*-ENOSYS;*/
#endif
}

#endif

/*//////////////////////////////////////////*/

#if defined(CONFIG_ARM64)
	/*arch/arm64/include/asm/pgtable-hwdef.h*/
	#ifndef PTE_RDONLY
		#define PTE_RDONLY			(_AT(pteval_t, 1) << 7)		/* AP[2] */
	#endif

	#define RS_PTE_RDONLY PTE_RDONLY
#else
	/*arch/arm/include/asm/pgtable-2level.h*/
	#ifndef L_PTE_RDONLY
		#define L_PTE_RDONLY			(_AT(pteval_t, 1) << 7)
	#endif

	#define RS_PTE_RDONLY L_PTE_RDONLY
#endif

#if defined(PTE_WRITE)
	/*qcom 8939 android 5.0 arm64 已定义了PTE_WRITE, 但 arch/arm64/mm/pageattr.c set_memory_xx() 只用了 PTE_RDONLY*/
	/*qcom 8996 android 6.0 arm64 pageattr.c set_memory_xx() 用到了 PTE_RDONLY*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
	#define RS_DO_NOT_USE_PTE_WRITE
#endif

	#if !defined(RS_DO_NOT_USE_PTE_WRITE)
		#define RS_PTE_WRITE PTE_WRITE
	#endif
#endif

/*from mmu.c*/
typedef struct tag_pte_callback_data {
	pgprot_t set_prot;
#if defined(RS_PTE_WRITE)
	pgprot_t clr_prot;
#endif
	/*int result;*/
} pte_callback_data;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_check_page_cb RS_HIDE(_fa)
#endif
noused notrace static int rs_check_page_cb(pte_t *ptep, pgtable_t token, unsigned long addr,
			void *data)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	pte_t val = pte_val(*ptep);
	pte_callback_data *cb_data = (pte_callback_data *)data;

	if ((val & pgprot_val(cb_data->set_prot))
	#if defined(RS_PTE_WRITE)
		&& (!(val & pgprot_val(cb_data->clr_prot)))
	#endif
		) {
		/*cb_data->result = 1;*/
		return 1;
	}
#else
	pte_t val = *ptep;

	pte_callback_data *cb_data = (pte_callback_data *)data;

	if ((pte_val(val) & pgprot_val(cb_data->set_prot))
	#if defined(RS_PTE_WRITE)
		&& (!(pte_val(val) & pgprot_val(cb_data->clr_prot)))
	#endif
		) {
		/*cb_data->result = 1;*/
		return 1;
	}
#endif

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_page_readonly_std RS_HIDE(_fb)
#endif
noused notrace static bool rs_is_page_readonly_std(unsigned long addr)
{
	int ret;
	pte_callback_data data;
	data.set_prot = __pgprot(RS_PTE_RDONLY);
#if defined(RS_PTE_WRITE)
	data.clr_prot = __pgprot(RS_PTE_WRITE);
#endif
	/*data.result = 0;*/

	ret = apply_to_page_range(&init_mm, addr, PAGE_SIZE,
					rs_check_page_cb, &data);

	if ((ret > 0)/* || (data.result)*/) {
		return true;
	}

	return false;
}

noused notrace static pte_t rs_clear_pte_bit(pte_t pte, pgprot_t prot)
{
	pte_val(pte) &= ~pgprot_val(prot);
	return pte;
}

noused notrace static pte_t rs_set_pte_bit(pte_t pte, pgprot_t prot)
{
	pte_val(pte) |= pgprot_val(prot);
	return pte;
}

noused notrace static pte_t rs_pud_pte(pud_t pud)
{
	return __pte(pud_val(pud));
}

noused notrace static pmd_t rs_pud_pmd(pud_t pud)
{
	return __pmd(pud_val(pud));
}

noused notrace static pte_t rs_pmd_pte(pmd_t pmd)
{
	return __pte(pmd_val(pmd));
}

noused notrace static pmd_t rs_pte_pmd(pte_t pte)
{
	return __pmd(pte_val(pte));
}

#ifndef PTE_TABLE_BIT
	#define PTE_TABLE_BIT		(_AT(pteval_t, 1) << 1)
#endif

noused notrace static pgprot_t rs_mk_sect_prot(pgprot_t prot)
{
	return __pgprot(pgprot_val(prot) & ~PTE_TABLE_BIT);
}

#if 0
noused notrace static pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	const pteval_t mask = PTE_USER | PTE_PXN | PTE_UXN | PTE_RDONLY |
				PTE_PROT_NONE | PTE_WRITE | PTE_TYPE_MASK;

	pte_val(pte) = (pte_val(pte) & ~mask) | (pgprot_val(newprot) & mask);
	return pte;
}

noused notrace static pmd_t rs_pmd_modify(pmd_t pmd, pgprot_t newprot)
{
	return rs_pte_pmd(pte_modify(rs_pmd_pte(pmd), newprot));
}
#endif

#if !defined(CONFIG_ARM64)

#ifndef PTE_BIT_FUNC /*pte_mkwrite*/

/*pte_wrprotect already defined in arch/arm/include/asm/pgtable.h*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#define RS_PTE_BIT_FUNC(fn, op) \
noused static pte_t pte_##fn(pte_t pte) { pte_val(pte) op; return pte; }

RS_PTE_BIT_FUNC(wrprotect, |= RS_PTE_RDONLY/*L_PTE_RDONLY*/);
RS_PTE_BIT_FUNC(mkwrite,   &= ~RS_PTE_RDONLY/*L_PTE_RDONLY*/);
#endif

#endif
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pte_wrprotect RS_HIDE(_fd)
#endif
noused notrace static pte_t rs_pte_wrprotect(pte_t pte)
{
#if defined(RS_PTE_WRITE)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	pte = rs_clear_pte_bit(pte, RS_PTE_WRITE);
	return rs_set_pte_bit(pte, RS_PTE_RDONLY);
#else
	pte = rs_clear_pte_bit(pte, __pgprot(RS_PTE_WRITE));
	return rs_set_pte_bit(pte, __pgprot(RS_PTE_RDONLY));
#endif
#else
#if defined(RS_DO_NOT_USE_PTE_WRITE)
	pte_val(pte) |= RS_PTE_RDONLY;
	return pte;
#else
	return pte_wrprotect(pte);
#endif
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pte_mkwrite RS_HIDE(_fe)
#endif
noused notrace static pte_t rs_pte_mkwrite(pte_t pte)
{
#if defined(RS_PTE_WRITE)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	pte = rs_clear_pte_bit(pte, RS_PTE_RDONLY);
	return rs_set_pte_bit(pte, RS_PTE_WRITE);
#else
	pte = rs_clear_pte_bit(pte, __pgprot(RS_PTE_RDONLY));
	return rs_set_pte_bit(pte, __pgprot(RS_PTE_WRITE));
#endif
#else
#if defined(RS_DO_NOT_USE_PTE_WRITE)
	pte_val(pte) &= ~RS_PTE_RDONLY;
	return pte;
#else
	return pte_mkwrite(pte);
#endif
#endif
}

#ifndef PMD_SECT_RDONLY
	#define PMD_SECT_RDONLY		(_AT(pmdval_t, 1) << 7)		/* AP[2] */ /*same as PTE_RDONLY*/
#endif

/*#define RS_HANDLE_PMD_BY_RODATA*/

#if defined(RS_HANDLE_PMD_BY_RODATA)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_write_prot RS_HIDE(_ff)
#endif
noused notrace static pgprot_t rs_get_write_prot(void)
{
#if defined(CONFIG_ARM64)
	return __pgprot(PMD_SECT_RDONLY);
#else
#ifdef CONFIG_ARM_LPAE
#if defined(L_PMD_SECT_RDONLY)
	return __pgprot(L_PMD_SECT_RDONLY);
#else
	return __pgprot(PMD_SECT_AP2);
#endif
#else
	return __pgprot(PMD_SECT_APX);
#endif
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pmd_wrprotect RS_HIDE(_fg)
#endif
noused notrace static pmd_t rs_pmd_wrprotect(pmd_t pmd)
{
	/*return pte_pmd(pte_wrprotect(pmd_pte(pmd)));*/
	return __pmd(pmd_val(pmd) | pgprot_val(rs_get_write_prot()));
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pmd_mkwrite RS_HIDE(_fh)
#endif
noused notrace static pmd_t rs_pmd_mkwrite(pmd_t pmd)
{
	/*return pte_pmd(pte_mkwrite(pmd_pte(pmd)));*/
	return __pmd(pmd_val(pmd) & ~pgprot_val(rs_get_write_prot()));
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pte_set_ro RS_HIDE(_fi)
#endif
noused notrace static int rs_pte_set_ro(pte_t *ptep, pgtable_t token, unsigned long addr, void *data)
{
#if defined(CONFIG_ARM64)
	set_pte(ptep, rs_pte_wrprotect(*ptep));
#else
	set_pte_ext(ptep, rs_pte_wrprotect(*ptep), 0);
#endif
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_pte_set_rw RS_HIDE(_fj)
#endif
noused notrace static int rs_pte_set_rw(pte_t *ptep, pgtable_t token, unsigned long addr, void *data)
{
#if defined(CONFIG_ARM64)
	set_pte(ptep, rs_pte_mkwrite(*ptep));
#else
	set_pte_ext(ptep, rs_pte_mkwrite(*ptep), 0);
#endif
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_ro_std RS_HIDE(_fk)
#endif
noused notrace static int rs_set_memory_ro_std(unsigned long addr, int numpages)
{
	int ret;
	unsigned long start = addr;
	unsigned long size = numpages << PAGE_SHIFT;
	unsigned end = start + size;

	ret = apply_to_page_range(&init_mm, start, size, rs_pte_set_ro, NULL);
	if (!ret) {
		flush_tlb_kernel_range(start, end);
		isb();
	}
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_rw_std RS_HIDE(_fl)
#endif
noused notrace static int rs_set_memory_rw_std(unsigned long addr, int numpages)
{
	int ret;
	unsigned long start = addr;
	unsigned long size = numpages << PAGE_SHIFT;
	unsigned end = start + size;

	ret = apply_to_page_range(&init_mm, start, size, rs_pte_set_rw, NULL);
	if (!ret) {
		flush_tlb_kernel_range(start, end);
		isb();
	}
	return ret;
}

/*from rodata.c*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define pmd_off_k RS_HIDE(_fm)
#endif
noused notrace static pmd_t *pmd_off_k(unsigned long virt)
{
	return pmd_offset(pud_offset(pgd_offset_k(virt), virt), virt);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_page_readonly_rodata RS_HIDE(_fn)
#endif
noused notrace static bool rs_is_page_readonly_rodata(unsigned long virt)
{
	bool ret = false;
	pmd_t *pmd;
	pte_t *pte;
	pmd_t pmd_value;
	pte_t pte_value;
	/*noused unsigned long start = virt;*/
	unsigned long end = virt + (1/*numpages*/ << PAGE_SHIFT);
	unsigned long pmd_end;

	while (virt < end) {
		pmd = pmd_off_k(virt);
		pmd_end = min(ALIGN(virt + 1, PMD_SIZE), end);

	#if defined(RS_HANDLE_PMD_BY_RODATA) && !defined(CONFIG_ARM64)
		#if !defined(CONFIG_ARM_LPAE)
		if (virt & SECTION_SIZE)
			pmd++;
		#endif
	#endif

		pmd_value = *pmd;

		if ((pmd_val(pmd_value) & PMD_TYPE_MASK) != PMD_TYPE_TABLE) {
		#if defined(RS_HANDLE_PMD_BY_RODATA)
			if ((pmd_val(pmd_value) & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
				if (pmd_val(pmd_value) & pgprot_val(rs_get_write_prot())) {
					ret = true;
				}
			}
		#endif

			/*pr_err("%s: pmd %p=%08lx for %08lx not page table\n",
				__func__, pmd,
				(unsigned long)pmd_val(*pmd), virt);
			*/
			virt = pmd_end;
			continue;
		}

		while (virt < pmd_end) {
			pte = pte_offset_kernel(pmd, virt);
			pte_value = *pte;

			if ((pte_val(pte_value) & pgprot_val(__pgprot(RS_PTE_RDONLY)))
			#if defined(RS_PTE_WRITE)
				&& (!(pte_val(pte_value) & pgprot_val(__pgprot(RS_PTE_WRITE))))
			#endif
				) {
				ret = true;
				break;
			}
			virt += PAGE_SIZE;
		}

		if (ret) {
			break;
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_page_readonly RS_HIDE(_fn1)
#endif
noused notrace static bool rs_is_page_readonly(unsigned long virt)
{
#if defined(CONFIG_FORCE_PAGES)
	return rs_is_page_readonly_std(virt);
#else
	return rs_is_page_readonly_rodata(virt);
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_attributes_rodata RS_HIDE(_fo)
#endif
noused notrace static int set_page_attributes_rodata(unsigned long virt, int numpages,
#if defined(RS_HANDLE_PMD_BY_RODATA)
	pmd_t (*f_pmd)(pmd_t),
#endif
	pte_t (*f_pte)(pte_t))
{
	pmd_t *pmd;
	pte_t *pte;
	pmd_t pmd_value;
	pte_t pte_value;
#if defined(RS_HANDLE_PMD_BY_RODATA) && !defined(CONFIG_ARM64)
	noused pmd_t *pmd_to_flush;
#endif
	unsigned long start = virt;
	unsigned long end = virt + (numpages << PAGE_SHIFT);
	unsigned long pmd_end;
	bool done = false;

	while (virt < end) {
		pmd = pmd_off_k(virt);
		pmd_end = min(ALIGN(virt + 1, PMD_SIZE), end);
	#if defined(RS_HANDLE_PMD_BY_RODATA) && !defined(CONFIG_ARM64)
		pmd_to_flush = pmd;
		#if !defined(CONFIG_ARM_LPAE)
		if (virt & SECTION_SIZE)
			pmd++;
		#endif
	#endif

		pmd_value = *pmd;
		if ((pmd_val(pmd_value) & PMD_TYPE_MASK) != PMD_TYPE_TABLE) {
		#if defined(RS_HANDLE_PMD_BY_RODATA)
			if ((pmd_val(pmd_value) & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
				set_pmd(pmd, f_pmd(pmd_value));
			#if !defined(CONFIG_ARM64)
				flush_pmd_entry(pmd_to_flush);
			#endif
				done = true;
			}
		#endif
			/*pr_err("%s: pmd %p=%08lx for %08lx not page table\n",
				__func__, pmd,
				(unsigned long)pmd_val(*pmd), virt);
			*/
			virt = pmd_end;
			continue;
		}

		if (virt < pmd_end) {
			done = true;

			do {
				pte = pte_offset_kernel(pmd, virt);
				pte_value = *pte;
			#if defined(CONFIG_ARM64)
				set_pte(pte, f_pte(pte_value));
			#else
				set_pte_ext(pte, f_pte(pte_value), 0);
			#endif
				virt += PAGE_SIZE;
			} while (virt < pmd_end);
		}
	}

	if (done) {
		flush_tlb_kernel_range(start, end);
		isb();

		return 0;
	} else
		return -EFAULT;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_ro_rodata RS_HIDE(_fp)
#endif
noused notrace static int rs_set_memory_ro_rodata(unsigned long virt, int numpages)
{
	return set_page_attributes_rodata(virt, numpages,
	#if defined(RS_HANDLE_PMD_BY_RODATA)
		rs_pmd_wrprotect,
	#endif
		rs_pte_wrprotect);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_rw_rodata RS_HIDE(_fq)
#endif
noused notrace static int rs_set_memory_rw_rodata(unsigned long virt, int numpages)
{
	return set_page_attributes_rodata(virt, numpages,
	#if defined(RS_HANDLE_PMD_BY_RODATA)
		rs_pmd_mkwrite,
	#endif
		rs_pte_mkwrite);
}

/*#if (defined(CONFIG_ARM64) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)))
	|| (defined(CONFIG_ARM64) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)))*/
extern int set_memory_ro(unsigned long virt, int numpages);
extern int set_memory_rw(unsigned long virt, int numpages);
/*#endif*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define set_memory_ro RS_HIDE(_fr)*/
#endif
noused notrace int __weak set_memory_ro(unsigned long virt, int numpages)
/*noused notrace static int set_memory_ro(unsigned long virt, int numpages)*/
{
	return rs_set_memory_ro_rodata(virt, numpages);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define set_memory_rw RS_HIDE(_fs)*/
#endif
noused notrace int __weak set_memory_rw(unsigned long virt, int numpages)
/*noused notrace static int set_memory_rw(unsigned long virt, int numpages)*/
{
	return rs_set_memory_rw_rodata(virt, numpages);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_ro RS_HIDE(_ft)
#endif
noused notrace static int rs_set_memory_ro(unsigned long virt, int numpages)
{
#if defined(RS_SET_PAGE_BY_FIXMAP) /*defined(CONFIG_DEBUG_RODATA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))*/
	return rs_set_memory_ro_rodata(virt, numpages); /*rs_set_memory_ro_std(virt, numpages);*/
#else
	return set_memory_ro(virt, numpages);
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_rw RS_HIDE(_fu)
#endif
noused notrace static int rs_set_memory_rw(unsigned long virt, int numpages)
{
#if defined(RS_SET_PAGE_BY_FIXMAP) /*defined(CONFIG_DEBUG_RODATA) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))*/
	return rs_set_memory_rw_rodata(virt, numpages); /*rs_set_memory_rw_std(virt, numpages);*/
#else
	return set_memory_rw(virt, numpages);
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_rw_generic RS_HIDE(_fu0)
#endif
noused notrace static int rs_set_memory_rw_generic(unsigned long addr, int numpages)
{
#if defined(CONFIG_FORCE_PAGES)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	return rs_set_memory_rw_rodata(addr, numpages); /*rs_set_memory_rw_std(addr, numpages);*/
#elif defined(RS_64BIT_SUPPORT) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	/* arm64 set_memory_XX will handle _addr which belongs to kernel vmalloc area only */
	int ret;

	ret = set_memory_rw(addr, numpages);
	if (ret == -EINVAL) {
		ret = rs_set_memory_rw_rodata(addr, numpages); /*set_memory_rw(addr, numpages);*/
	}
	return ret;
#else
	return set_memory_rw(addr, numpages);
#endif
#else
	return rs_set_memory_rw_rodata(addr, numpages); /*set_memory_rw(addr, numpages);*/
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_memory_ro_generic RS_HIDE(_fu1)
#endif
noused notrace static int rs_set_memory_ro_generic(unsigned long addr, int numpages)
{
#if defined(CONFIG_FORCE_PAGES)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	return rs_set_memory_ro_rodata(addr, numpages); /*rs_set_memory_ro_std(addr, numpages);*/
#elif defined(RS_64BIT_SUPPORT) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	/* arm64 set_memory_XX will handle _addr which belongs to kernel vmalloc area only */
	int ret;

	ret = set_memory_ro(addr, numpages);
	if (ret == -EINVAL) {
		ret = rs_set_memory_ro_rodata(addr, numpages); /*set_memory_ro(addr, numpages);*/
	}
	return ret;
#else
	return set_memory_ro(addr, numpages);
#endif
#else
	return rs_set_memory_ro_rodata(addr, numpages); /*set_memory_ro(addr, numpages);*/
#endif
}


#if defined(RS_SET_PAGE_BY_FIXMAP)

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_patch_lock RS_HIDE(_fv_)
#endif

noused static DEFINE_RAW_SPINLOCK(rs_patch_lock);
#endif

#if defined(CONFIG_ARM) && !defined(CONFIG_JUMP_LABEL) \
	&& !defined(CONFIG_KPROBES) && !defined(CONFIG_KGDB)
/* patch.c is not built */
DEFINE_RAW_SPINLOCK(patch_lock);
#else
/* insn.c or patch.c is built.
  insn.c always built on arm64.
 */
extern raw_spinlock_t patch_lock;

noused __rs_weak DEFINE_RAW_SPINLOCK(patch_lock);
#endif

#define	rs_patch_lock	patch_lock

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_patch_map RS_HIDE(_fw_)
#endif

noused notrace static void *rs_patch_map(void *addr, int fixmap, unsigned long *flags, int *patched)
	__acquires(&rs_patch_lock)
{
	uintptr_t uintaddr = (uintptr_t)addr;
	bool module = !(core_kernel_text(uintaddr) || __is_kernel_rodata(uintaddr));
	struct page *page;

	if (patched) {
		*patched = 0;
	}

	if (module && IS_ENABLED(CONFIG_DEBUG_SET_MODULE_RONX)) {
		page = vmalloc_to_page(addr);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
	} else if (!module) {
		page = phys_to_page(__pa_symbol(addr));
#else
	} else if (!module && (IS_ENABLED(CONFIG_DEBUG_RODATA) ||
		IS_ENABLED(CONFIG_KERNEL_TEXT_RDONLY))) {
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
		page = pfn_to_page(PHYS_PFN(__pa(addr)));
	#else
		page = virt_to_page(addr);
	#endif
#endif
	} else {
		return addr;
	}

	if (!page) {
		return ERR_PTR(-EFAULT);
	}

	if (flags)
		raw_spin_lock_irqsave(&rs_patch_lock, *flags);
	else
		__acquire(&rs_patch_lock);

	if (patched) {
		*patched = 1;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)) && !defined(CONFIG_ARM) /*KERNEL_VERSION(3, 14, 0)*/
	return (void *)set_fixmap_offset(fixmap, page_to_phys(page) + (uintaddr & ~PAGE_MASK));
#else
	set_fixmap(fixmap, page_to_phys(page));

	return (void *)(__fix_to_virt(fixmap) + (uintaddr & ~PAGE_MASK));
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_patch_unmap RS_HIDE(_fx_)
#endif
noused notrace static void rs_patch_unmap(int fixmap, unsigned long *flags)
	__releases(&rs_patch_lock)
{
	clear_fixmap(fixmap);

	if (flags)
		raw_spin_unlock_irqrestore(&rs_patch_lock, *flags);
	else
		__release(&rs_patch_lock);
}
#endif

/* ARM64 CONFIG_RANDOMIZE_BASE (KASLR)
  https://lwn.net/Articles/673598/
  Randomize the module region independently from the core kernel. It is chosen
  such that it covers the [_stext, _etext] interval of the core kernel to avoid
  using PLT entries unless we really have to.
  
  please refer to module_alloc_base related code in kaslr_early_init() of kaslr.c
  
  extern unsigned long kaslr_offset(void); //equal to (kimage_vaddr - KIMAGE_VADDR) for ARM64
 */

#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
	#define RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING
#endif

#define TEXT_AREA_START           _stext
#define TEXT_AREA_END             _etext
#define DATA_AREA_START           _sdata
#define DATA_AREA_END             _edata
#define BSS_AREA_START            __bss_start
#define BSS_AREA_END              __bss_stop

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)

/*#define RS_USE_PROBE_WRITE*/

#if defined(RS_USE_PROBE_WRITE)

#undef probe_kernel_write
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define probe_kernel_write RS_HIDE(_g0)
#endif
noused notrace static long probe_kernel_write(void *dst, const void *src, size_t size)
{
#if 0
	long ret;
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	pagefault_disable();
#endif
	memcpy(dst, src, size); /*ret = __copy_to_user_inatomic((__force void __user *)dst, src, size);*/
#if 0
	pagefault_enable();
	set_fs(old_fs);

	return ret;
#endif
	return 0;
}
#endif

#else

/*#define RS_USE_PROBE_WRITE*/

#if defined(RS_USE_PROBE_WRITE)
extern long notrace probe_kernel_write(void *dst, const void *src, size_t size);
#endif

#endif

typedef struct tag_set_mem_protect_struct {
	unsigned long prev_page_addr;
	size_t max_data_unit;
	unsigned long flags;
} set_mem_protect_struct;

typedef struct tag_set_page_protect_struct {
	set_mem_protect_struct header;
} set_page_protect_struct;



#if defined(CONFIG_ARM64) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	#define RS_FLUSH_ICACHE_RANGE(start, end)	__flush_icache_range((start), (end))
#else
	#define RS_FLUSH_ICACHE_RANGE(start, end)	flush_icache_range((start), (end))
#endif

#if defined(CONFIG_ARM64)
	#define RS_FLUSH_DCACHE_RANGE(start, end)	\
		do {	\
			if ((size_t)(end) >= (size_t)(start)) {	\
				__flush_dcache_area((void *)(start), ((size_t)(end) - (size_t)(start)));	\
			} else {	\
				__flush_dcache_area((void *)(end), ((size_t)(start) - (size_t)(end)));	\
			}	\
		} while (0)
#elif defined(CONFIG_ARM)
	/* void __cpuc_flush_dcache_area(void *p, size_t size) */
	/* void __sync_cache_range_w(volatile void *p, size_t size) */
	/* void clean_dcache_area(void *p, size_t size) */
	#define RS_FLUSH_DCACHE_RANGE(start, end)	\
		do {	\
			if ((size_t)(end) >= (size_t)(start)) {	\
				__cpuc_flush_dcache_area((void *)(start), (int)((size_t)(end) - (size_t)(start)));	\
			} else {	\
				__cpuc_flush_dcache_area((void *)(end), (int)((size_t)(start) - (size_t)(end)));	\
			}	\
		} while (0)
#else
	#define RS_FLUSH_DCACHE_RANGE(start, end)	do { } while (0)
#endif

#if defined(RS_SET_PAGE_BY_FIXMAP)

typedef struct tag_fixmap_protect_struct {
	set_mem_protect_struct header;
	unsigned long page_addr;
	void *first_addr;
	size_t first_size;
	int first_patched;
#if !defined(CONFIG_ARM64)
	void *second_addr;
	size_t second_size;
	int second_patched;
#endif
} fixmap_protect_struct;

#define page_protect_struct tag_fixmap_protect_struct

#define RX_AREA_START		_stext
#define RX_AREA_END		__end_rodata /*高通原始是__start_rodata，需要放宽范围*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_h4)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			RS_LOG("spw,1\n");
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_DEBUG_RODATA)
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING)
	#endif
		{
			int ret;
			void *page_addr;
			void *first_addr;
			int patched = 0;
			size_t data_size = data->header.max_data_unit; /*PAGE_SIZE;*/

		#if defined(CONFIG_ARM64)
			if (!data_size) {
				RS_LOG("spw,2\n");
				ret = -EINVAL;
				goto out;
			}

			data->first_addr = 0;
			data->first_size = 0;
			data->first_patched = 0;

			if (((((uintptr_t)_addr) & ~PAGE_MASK) + data_size) > PAGE_SIZE) {
				RS_LOG("spw,3\n");
				ret = -EINVAL;
				goto out;
			}

			page_addr = (void *)(_addr & PAGE_MASK);

			first_addr = rs_patch_map(page_addr, FIX_TEXT_POKE0, &data->header.flags, &patched);

			if (unlikely(!first_addr)) {
				RS_LOG("spw,4\n");
				ret = -EFAULT;
				goto out;
			} else if (IS_ERR(first_addr)) {
				RS_LOG("spw,5\n");
				ret = PTR_ERR(first_addr);
				goto out;
			}

			data->first_addr = first_addr;
			data->first_size = PAGE_SIZE; /*data_size;*/
			data->first_patched = patched;
			data->page_addr = (unsigned long)page_addr;

			RS_LOG("spw,6\n");
			ret = 0;
		#else
			int first_size;
			int second_size;
			void *second_addr;
			int second_patched = 0;

			if (!data_size) {
				RS_LOG("spw,2\n");
				ret = -EINVAL;
				goto out;
			}

			data->first_addr = 0;
			data->first_size = 0;
			data->first_patched = 0;
			data->second_addr = 0;
			data->second_size = 0;
			data->second_patched = 0;

			page_addr = (void *)(_addr & PAGE_MASK);

			first_addr = rs_patch_map(page_addr, FIX_TEXT_POKE0, &data->header.flags, &patched);
			if (unlikely(!first_addr)) {
				ret = -EFAULT;
				goto out;
			} else if (IS_ERR(first_addr)) {
				ret = PTR_ERR(first_addr);
				goto out;
			}

			first_size = ((((uintptr_t)_addr) & ~PAGE_MASK) + data_size);
			if (first_size > PAGE_SIZE) {
				second_size = first_size - PAGE_SIZE;
			} else {
				second_size = 0;
			}

			first_size = PAGE_SIZE; /*data_size - second_size;*/

			if (second_size) {
				second_addr = rs_patch_map((void *)((uintptr_t)page_addr + PAGE_SIZE),
					FIX_TEXT_POKE1, NULL, &second_patched);
				if (unlikely(!second_addr)) {
					ret = -EFAULT;
					if (patched) {
						rs_patch_unmap(FIX_TEXT_POKE0, &data->header.flags);
					}
					goto out;
				} else if (IS_ERR(second_addr)) {
					ret = PTR_ERR(second_addr);
					if (patched) {
						rs_patch_unmap(FIX_TEXT_POKE0, &data->header.flags);
					}

					goto out;
				}

				data->second_addr = second_addr;
				data->second_size = second_size;
				data->second_patched = second_patched;
			}

			data->first_addr = first_addr;
			data->first_size = first_size;
			data->first_patched = patched;
			data->page_addr = (unsigned long)page_addr;

			ret = 0;
		#endif

		out:
			return ret;
		}
	#if defined(CONFIG_DEBUG_RODATA)
		else {
			RS_LOG("spw,a\n");
			return 0;
		}
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_h5)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			RS_LOG("spr,1\n");
			rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((data) && ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END))) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_DEBUG_RODATA)
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING)
	#endif
		{
			int ret;

		#if defined(CONFIG_ARM64)
			if ((data->first_addr) && (data->first_size)) {
				RS_LOG("spr,2\n");
				/*flush_kernel_vmap_range(data->first_addr, data->first_size);*/
				if (data->first_patched) {
					RS_LOG("spr,3\n");
					rs_patch_unmap(FIX_TEXT_POKE0, &data->header.flags);
				}

				ret = 0;
			} else {
				RS_LOG("spr,4\n");
				ret = -EINVAL;
			}
		#else
			if ((data->second_addr) && (data->second_size)) {
				flush_kernel_vmap_range(data->second_addr, data->second_size);
				if (data->second_patched) {
					rs_patch_unmap(FIX_TEXT_POKE1, NULL);
				}
			}

			if ((data->first_addr) && (data->first_size)) {
				flush_kernel_vmap_range(data->first_addr, data->first_size);
				if (data->first_patched) {
					rs_patch_unmap(FIX_TEXT_POKE0, &data->header.flags);
				}

				ret = 0;
			} else {
				ret = -EINVAL;
			}
		#endif

			if (!ret) {
				_addr = data->page_addr;
				/*RS_FLUSH_ICACHE_RANGE(_addr, _addr + data->first_size);*/
				/*RS_FLUSH_DCACHE_RANGE(_addr, _addr + data->first_size);*/

			#if !defined(CONFIG_ARM64)
				if (data->second_patched) {
					_addr += data->first_size;
					/*RS_FLUSH_ICACHE_RANGE(_addr, _addr + data->second_size);*/
					/*RS_FLUSH_DCACHE_RANGE(_addr, _addr + data->second_size);*/
				}
			#endif
			}

			return ret;
		}
	#if defined(CONFIG_DEBUG_RODATA)
		else {
			return 0;
		}
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src, size_t src_size,
	struct page_protect_struct *data)
{
	int ret = -EINVAL;
	unsigned long _addr = (unsigned long)dest;

	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			RS_LOG("c2rp,1\n");
		#if RS_DEBUG
			return (int)probe_kernel_write(dest, src, src_size);
		#else
			memmove(dest, src, src_size);
			return 0;
		#endif
		}
	}

	if ((data) && ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END))) {
		/*extern int scheduler_running;*/
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		/*if (scheduler_running)*/ /*if (remap)*/
	#if defined(CONFIG_DEBUG_RODATA)
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING)
	#endif
		{
		#if defined(CONFIG_ARM64)
			if (data->first_addr) {
				size_t offs_in_page = _addr - data->page_addr;

				if (data->first_size >= (offs_in_page + src_size)) {
					RS_LOG("c2rp,2\n");
				#if RS_DEBUG
					ret = (int)probe_kernel_write(data->first_addr + offs_in_page, src, src_size);
				#else
					memmove(data->first_addr + offs_in_page, src, src_size);

					ret = 0;
				#endif
				} else {
					RS_LOG("c2rp,3\n");
				}
			} else {
				RS_LOG("c2rp,4\n");
			}
		#else
			if (data->first_addr) {
			#if defined(RS_FIXMAP_DIRECT_COPY_TO_RODATA)
				memmove(dest, src, src_size);
			#else
				size_t offs_in_page = _addr - data->page_addr;

				if (data->first_size >= (offs_in_page + src_size)) {
					RS_LOG("c2rp,5\n");
				#if RS_DEBUG
					ret = (int)probe_kernel_write(data->first_addr + offs_in_page, src, src_size);
				#else
					memmove(data->first_addr + offs_in_page, src, src_size);
					ret = 0;
				#endif
				} else if (data->second_addr) {
					size_t first_part_size = (offs_in_page + src_size) - data->first_size;
					RS_LOG("c2rp,6\n");
				#if RS_DEBUG
					ret = (int)probe_kernel_write(data->first_addr + offs_in_page, src, first_part_size);
					if (!ret)
						ret = (int)probe_kernel_write(data->second_addr, ((char *)src + first_part_size),
														(src_size - first_part_size));
				#else
					memmove(data->first_addr + offs_in_page, src, first_part_size);
					memmove(data->second_addr, ((char *)src + first_part_size),
							(src_size - first_part_size));
					ret = 0;
				#endif
				} else {
					RS_LOG("c2rp,7\n");
				}
			#endif
			} else {
				RS_LOG("c2rp,8\n");
			}
		#endif
		}
	#if defined(CONFIG_DEBUG_RODATA)
		else {
			RS_LOG("c2rp,9\n");
		#if RS_DEBUG
			return (int)probe_kernel_write(dest, src, src_size);
		#else
			memmove(dest, src, src_size);
			ret = 0;
		#endif
		}
	#endif
	}

	return ret;
}

#elif defined(CONFIG_STRICT_MEMORY_RWX) /*qualcomm only*/

#if defined(CONFIG_ARM64)

/*arch/arm64/mm/mmu.c*/

extern void mem_text_writeable_spinlock(unsigned long *flags);
extern void mem_text_writeable_spinunlock(unsigned long *flags);

#define RX_AREA_START		_stext
#define RX_AREA_END		__end_rodata /*高通原始是__start_rodata，需要放宽范围*/

typedef struct tag_mem_unprotect_struct {
	u64 addr;
	pmd_t *pmd;
	pte_t *pte;
	pmd_t saved_pmd;
	pte_t saved_pte;
	bool made_writeable;
} mem_unprotect_struct;

typedef struct tag_set_mem_unprotect_struct {
	set_mem_protect_struct header;
	mem_unprotect_struct mem_unprotect;
} set_mem_unprotect_struct;

#define page_protect_struct tag_set_mem_unprotect_struct

/*refer to get_pmd_prot_sect_kernel()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_prot_sect_kernel RS_HIDE(_h1)
#endif
noused notrace static pmdval_t get_prot_sect_kernel(unsigned long addr)
{
	extern pmdval_t get_pmd_prot_sect_kernel(unsigned long addr);

	pmdval_t val = get_pmd_prot_sect_kernel(addr);

	if (addr >= (unsigned long)__init_data_begin)
		return val & ~PMD_SECT_PXN; /*prot_sect_kernel | PMD_SECT_PXN;*/
	if (addr >= (unsigned long)__init_begin)
		return val & ~PMD_SECT_RDONLY; /*prot_sect_kernel | PMD_SECT_RDONLY;*/
	if (addr >= (unsigned long)__start_rodata)
		return val & ~(PMD_SECT_RDONLY | PMD_SECT_PXN); /*prot_sect_kernel | PMD_SECT_RDONLY | PMD_SECT_PXN;*/
	if (addr >= (unsigned long)_stext)
		return val & ~PMD_SECT_RDONLY; /*prot_sect_kernel | PMD_SECT_RDONLY;*/
	return val & ~PMD_SECT_PXN; /*prot_sect_kernel | PMD_SECT_PXN;*/
}

/* this function must be called with mem_text_writeable_lock held */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_mem_text_address_writeable RS_HIDE(_h2)
#endif
noused notrace static int rs_mem_text_address_writeable(u64 addr,
	mem_unprotect_struct *mem_unprotect)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	/*u64 addr_aligned;*/
	pmdval_t prot_sect_kernel;

	mem_unprotect->made_writeable = 0;

	/*
	if ((addr < (unsigned long)RX_AREA_START) ||
		(addr >= (unsigned long)RX_AREA_END)) {
		return -EINVAL;
	}
	*/

	pgd = pgd_offset_k(addr);
	pud = pud_offset(pgd, addr);
	pmd = pmd_offset(pud, addr);

	/*mem_unprotect->pmd = pmd_offset(pud, addr);*/
	/*addr_aligned = addr & PAGE_MASK;*/
	mem_unprotect->addr = addr;
	mem_unprotect->pmd = pmd;

	mem_unprotect->saved_pmd = *pmd;
	if ((mem_unprotect->saved_pmd & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
		prot_sect_kernel = get_prot_sect_kernel(addr);

		set_pmd(pmd, __pmd(__pa((addr & PAGE_MASK)) | prot_sect_kernel));
	} else {
		pte_t *pte = pte_offset_kernel(pmd, addr);
		mem_unprotect->pte = pte;
		mem_unprotect->saved_pte = *pte;
		set_pte(pte, pfn_pte(__pa(addr) >> PAGE_SHIFT, PAGE_KERNEL_EXEC));
	}

	flush_tlb_kernel_range(addr, addr + PAGE_SIZE);
	isb();

	mem_unprotect->made_writeable = 1;

	return 0;
}

/* this function must be called with mem_text_writeable_lock held */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_mem_text_address_restore RS_HIDE(_h3)
#endif
noused notrace static int rs_mem_text_address_restore(mem_unprotect_struct * mem_unprotect)
{
	if (mem_unprotect->made_writeable) {
		if ((mem_unprotect->saved_pmd & PMD_TYPE_MASK) == PMD_TYPE_SECT)
			*(mem_unprotect->pmd) = mem_unprotect->saved_pmd;
		else
			*(mem_unprotect->pte) = mem_unprotect->saved_pte;

		flush_tlb_kernel_range(mem_unprotect->addr, mem_unprotect->addr + PAGE_SIZE);
		isb();

		mem_unprotect->made_writeable = 0;

		return 0;
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_h4)
#endif
noused notrace static int set_page_rw(u64 _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		{
			int ret;

			mem_text_writeable_spinlock(&data->header.flags);
			ret = rs_mem_text_address_writeable(_addr, &data->mem_unprotect);
			if (ret) {
				mem_text_writeable_spinunlock(&data->header.flags);
			} else {
			#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
				rs_set_memory_rw_generic(_addr, 1);
			#endif
			}

			return ret;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_h5)
#endif
noused notrace static int set_page_ro(u64 _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		{
			int ret;

		#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
			rs_set_memory_ro_generic(_addr, 1);
		#endif

			ret = rs_mem_text_address_restore(&data->mem_unprotect);
			if (!ret) {
				mem_text_writeable_spinunlock(&data->header.flags);
			}

			return ret;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#else


/*change_page_attr, set_page_attributes, set_memory_ro/set_memory_rw, set_kernel_text_ro/set_kernel_text_rw
set_all_modules_text_rw/set_all_modules_text_ro
 kernel/arch/arm/mm/rodata.c/
CONFIG_DEBUG_RODATA 新版 linux 控制 text 和 rodata 为 ro
参考 kernel/kernel/kprobes.c 中 arch_arm_kprobe()，需要临时改写指令
*/

/*copy from kernel/arch/arm/mm/mmu.c*/

#define RX_AREA_START		_stext
#define RX_AREA_END		__end_rodata /*高通原始是__start_rodata，需要放宽范围*/

extern void mem_text_writeable_spinlock(unsigned long *flags);
extern void mem_text_writeable_spinunlock(unsigned long *flags);

typedef struct tag_mem_unprotect_struct {
	struct mm_struct *active_mm;
	pmd_t *pmd_to_flush;
	pmd_t *pmd;
	unsigned long addr;
	pmd_t saved_pmd;
	pte_t *pte;
	pte_t saved_pte;
	bool made_writeable;
} mem_unprotect_struct;

typedef struct tag_set_mem_unprotect_struct {
	set_mem_protect_struct header;
	mem_unprotect_struct mem_unprotect;
} set_mem_unprotect_struct;

#define page_protect_struct tag_set_mem_unprotect_struct

/*refer to mem_text_address_writeable()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_mem_text_address_writeable RS_HIDE(_i1)
#endif
noused notrace static int rs_mem_text_address_writeable(unsigned long addr,
	mem_unprotect_struct *mem_unprotect)
{
	struct task_struct *tsk;
	struct mm_struct *mm;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long align_addr;

	mem_unprotect->made_writeable = 0;

	/*
	if ((addr < (unsigned long)RX_AREA_START) ||
		(addr >= (unsigned long)RX_AREA_END)) {
		return -EINVAL;
	}
	*/

	tsk = current;

	task_lock(tsk);
	mm = tsk->active_mm;
	if (mm) {
		atomic_inc(&mm->mm_count);
	}
	task_unlock(tsk);

	if (!mm) {
		mm = &init_mm; /*return -ENOENT;*/
	}

	mem_unprotect->active_mm = mm;

	pgd = pgd_offset(mm, addr);
	pud = pud_offset(pgd, addr);
	pmd = pmd_offset(pud, addr);

	/*mem_unprotect->pmd = pmd_offset(pud, addr);*/
	mem_unprotect->pmd_to_flush = pmd; /*= mem_unprotect->pmd;*/
	/*mem_unprotect->addr = addr & PAGE_MASK;*/

#ifndef CONFIG_ARM_LPAE
	if (addr & SECTION_SIZE)
		pmd++; /*mem_unprotect->pmd++;*/
#endif

	align_addr = addr & PAGE_MASK;
	mem_unprotect->addr = align_addr;
	mem_unprotect->pmd = pmd;

	mem_unprotect->saved_pmd = *pmd; /**(mem_unprotect->pmd);*/
	if ((mem_unprotect->saved_pmd & PMD_TYPE_MASK) != PMD_TYPE_SECT) {
		/*
		if (mm != &init_mm) {
			mmdrop(mm);
		}
		*/

		/*return -EFAULT;*/
		pte_t *pte = pte_offset_kernel(pmd, addr);
		mem_unprotect->pte = pte;
		mem_unprotect->saved_pte = *pte;
		set_pte_ext(pte,  pfn_pte(__pa(addr) >> PAGE_SHIFT, PAGE_KERNEL_EXEC), 0);
	} else {
	#ifdef CONFIG_ARM_LPAE
		#if defined(L_PMD_SECT_RDONLY)
		*pmd &= ~L_PMD_SECT_RDONLY;
		#else
		*pmd &= ~PMD_SECT_AP2;
		#endif
	#else
		*pmd &= ~PMD_SECT_APX;
	#endif

		flush_pmd_entry(mem_unprotect->pmd_to_flush);
	}
	flush_tlb_kernel_page(align_addr/*mem_unprotect->addr*/);
	isb();

	mem_unprotect->made_writeable = 1;

	return 0;
}

/*refer to mem_text_address_restore()*/
/* this function must be called with mem_text_writeable_lock held */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_mem_text_address_restore RS_HIDE(_i2)
#endif
noused notrace static int rs_mem_text_address_restore(mem_unprotect_struct * mem_unprotect)
{
	if (mem_unprotect->made_writeable) {
		struct mm_struct *mm;
		if ((mem_unprotect->saved_pmd & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
			*(mem_unprotect->pmd) = mem_unprotect->saved_pmd;
			flush_pmd_entry(mem_unprotect->pmd_to_flush);
		} else {
			*(mem_unprotect->pte) = mem_unprotect->saved_pte;
		}
		flush_tlb_kernel_page(mem_unprotect->addr);
		isb();

		mm = mem_unprotect->active_mm;
		if (mm != &init_mm) {
			mmdrop(mm);
		}
		mem_unprotect->made_writeable = 0;
		return 0;
	}

	return -EINVAL;
}

/*set_page_rw 与 set_page_ro 必须配对使用，因为用到了spinlock*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_i3)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		{
			int ret;

			mem_text_writeable_spinlock(&data->header.flags);
			ret = rs_mem_text_address_writeable(_addr, &data->mem_unprotect);
			if (ret) {
				mem_text_writeable_spinunlock(&data->header.flags);
			} else {
			#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
				rs_set_memory_rw_generic(_addr, 1);
			#endif
			}

			return ret;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_i4)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		{
			int ret;

		#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
			rs_set_memory_ro_generic(_addr, 1);
		#endif

			ret = rs_mem_text_address_restore(&data->mem_unprotect);
			if (!ret) {
				mem_text_writeable_spinunlock(&data->header.flags);
			}
			return ret;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#if 0
noused __weak void __sync_cache_range_w(volatile void *p, size_t size)
{
	char *_p = (char *)p;

	__cpuc_flush_dcache_area(_p, size); /*__cpuc_clean_dcache_area(_p, size);*/
	outer_clean_range(__pa(_p), __pa(_p + size));
}
#endif

#endif

#elif defined(_ASMARM_RODATA_H) && defined(CONFIG_DEBUG_RODATA)
/*有 rodata.h/rodata.c, 实际开了 CONFIG_DEBUG_RODATA 才有效
int set_memory_rw(unsigned long virt, int numpages);
int set_memory_ro(unsigned long virt, int numpages);
这个set_memory_rw/set_memory_ro没有 MODULES_VADDR ~ MODULES_END 地址范围的限制
*/

#define page_protect_struct tag_set_page_protect_struct

/*addr range refer to set_kernel_text_rw()/set_kernel_text_ro()*/
#define RX_AREA_START		PAGE_ALIGN((unsigned long)_text)
#define RX_AREA_END		PAGE_ALIGN((unsigned long)__end_rodata)

/*怎么区分是 arch/arm/mm 下 rodata.c 还是 mmu.c 里的 set_memory_rw/set_memory_ro?
目前看来一个内核版本只有一处有定义
*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_j0)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING) {
			return set_memory_rw(_addr, 1);
		} else {
			return 0;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_j1)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING) {
			return set_memory_ro(_addr, 1);
		} else {
			return 0;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#elif defined(CONFIG_ARM64) && defined(RS_QUALCOMM_PLATFORM)
/*arm64暂不支持text段只读
set_memory_rw/set_memory_ro有 MODULES_VADDR ~ MODULES_END 地址范围的限制
*/
#define page_protect_struct tag_set_page_protect_struct

#define RX_AREA_START		PAGE_ALIGN((unsigned long)_stext)
#define RX_AREA_END		PAGE_ALIGN((unsigned long)__end_rodata)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_k0)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
		return rs_set_memory_rw_generic(_addr, 1);
	#else
		return 0; /*return set_memory_rw(_addr, 1);*/
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_k1)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
		return rs_set_memory_ro_generic(_addr, 1);
	#else
		return 0; /*return set_memory_ro(_addr, 1);*/
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#elif !defined(RS_64BIT_SUPPORT) && defined(CONFIG_ARM_KERNMEM_PERMS) && defined(CONFIG_DEBUG_RODATA)
/*new branch*/

/*refer to set_kernel_text_rw/set_kernel_text_ro in arch/arm/mm/init.c*/

#include <asm/cp15.h>
#include <asm/system_info.h>

/*refer to arch_has_strict_perms()*/
/* Make sure extended page tables are in use. */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_arch_has_strict_perms RS_HIDE(_h6_1)
#endif
noused notrace static int rs_arch_has_strict_perms(void)
{
	static int g_rs_arch_has_strict_perms;

	if (g_rs_arch_has_strict_perms) {
		return ((g_rs_arch_has_strict_perms > 0) ? 1 : 0);
	}

	if (cpu_architecture() < CPU_ARCH_ARMv6) {
		goto out;
	}

	if (get_cr() & CR_XP) {
		g_rs_arch_has_strict_perms = 1;
		return 1;
	}

out:
	g_rs_arch_has_strict_perms = -1;
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_rodata_writeable_lock RS_HIDE(_h6_2)
#endif
noused static DEFINE_SPINLOCK(rs_rodata_writeable_lock);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_rodata_writeable_spinlock RS_HIDE(_h6_3)
#endif
noused notrace static void rs_rodata_writeable_spinlock(unsigned long *flags)
{
	spin_lock_irqsave(&rs_rodata_writeable_lock, *flags);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_rodata_writeable_spinunlock RS_HIDE(_h6_4)
#endif
noused notrace static void rs_rodata_writeable_spinunlock(unsigned long *flags)
{
	spin_unlock_irqrestore(&rs_rodata_writeable_lock, *flags);
}

#define RX_AREA_START		_stext
#define RX_AREA_END		__init_begin /*refer to ro_perms[] in arch/arm/mm/init.c*/

typedef struct tag_mem_unprotect_struct {
	struct mm_struct *active_mm;
	pmd_t *pmd_to_flush;
	pmd_t *pmd;
	unsigned long addr;
	pmd_t saved_pmd;
	pte_t *ptep;
	pte_t saved_pte;
	bool made_writeable;
} mem_unprotect_struct;

typedef struct tag_set_mem_unprotect_struct {
	set_mem_protect_struct header;
	mem_unprotect_struct mem_unprotect;
} set_mem_unprotect_struct;

#define page_protect_struct tag_set_mem_unprotect_struct


struct pte_data {
	pteval_t mask;
	pteval_t val;
	mem_unprotect_struct *mem_unprotect;
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __pte_update RS_HIDE(_h6_6)
#endif
noused notrace static int __pte_update(pte_t *ptep, pgtable_t token, unsigned long addr,
			void *d)
{
	struct pte_data *data = d;
	pte_t pte = *ptep;

	data->mem_unprotect->ptep = ptep;
	data->mem_unprotect->saved_pte = pte;

	pte = __pte((pte_val(pte) & data->mask) | data->val);
	set_pte_ext(ptep, pte, 0);

	return 0;
}

/*refer to mem_text_address_writeable()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_rodata_address_writeable RS_HIDE(_i1)
#endif
noused notrace static int rs_rodata_address_writeable(unsigned long addr,
	mem_unprotect_struct *mem_unprotect)
{
	struct task_struct *tsk;
	struct mm_struct *mm;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long align_addr;

	if (!rs_arch_has_strict_perms()) {
		return 0;
	}

	mem_unprotect->made_writeable = 0;

	/*
	if ((addr < (unsigned long)RX_AREA_START) ||
		(addr >= (unsigned long)RX_AREA_END)) {
		return -EINVAL;
	}
	*/

	tsk = current;

	task_lock(tsk);
	mm = tsk->active_mm;
	if (mm) {
		atomic_inc(&mm->mm_count);
	}
	task_unlock(tsk);

	if (!mm) {
		mm = &init_mm; /*return -ENOENT;*/
	}

	mem_unprotect->active_mm = mm;

	pgd = pgd_offset(mm, addr);
	pud = pud_offset(pgd, addr);
	pmd = pmd_offset(pud, addr);

	align_addr = addr & PAGE_MASK;
	mem_unprotect->addr = align_addr;

	if (pmd_bad(*pmd)) {
		/*section_update();*/
		/*mem_unprotect->pte = NULL;*/ /*should cleared in init_page_protect_data()*/

		mem_unprotect->pmd_to_flush = pmd;

	#ifdef CONFIG_ARM_LPAE
		/*pmd[0] = __pmd((pmd_val(pmd[0]) & mask) | prot);*/
	#else
		if (addr & SECTION_SIZE)
			pmd++; /*pmd[1] = __pmd((pmd_val(pmd[1]) & mask) | prot);*/
		/*else*/
			/*pmd[0] = __pmd((pmd_val(pmd[0]) & mask) | prot);*/
	#endif

		mem_unprotect->pmd = pmd;

		mem_unprotect->saved_pmd = __pmd(pmd_val(pmd[0])); /* *pmd*/; /**(mem_unprotect->pmd);*/

	#ifdef CONFIG_ARM_LPAE
		pmd[0] = __pmd((pmd_val(pmd[0]) & (~PMD_SECT_RDONLY)) | 0);
	#else
		pmd[0] = __pmd((pmd_val(pmd[0]) & (~(PMD_SECT_APX | PMD_SECT_AP_WRITE))) | PMD_SECT_AP_WRITE);
	#endif

		flush_pmd_entry(mem_unprotect->pmd_to_flush);
		flush_tlb_kernel_page(align_addr);
		/*local_flush_tlb_kernel_page(align_addr);*/
		/*local_flush_tlb_kernel_range(align_addr, align_addr + PAGE_SIZE);*/
	} else {
	#if 1
		pte_t *ptep = pte_offset_map(pmd, addr); /*pte_offset_kernel(pmd, addr);*/
		pte_t pte = *ptep;
		mem_unprotect->ptep = ptep;
		mem_unprotect->saved_pte = pte;
		set_pte_ext(ptep, __pte((pte_val(pte) & (~L_PTE_RDONLY)) | 0), 0);
	#else
		/*pte_update()*/
		/*will access to NULL pointer when call into apply_to_pte_range() -> pte_alloc_map_lock() -> pte_offset_map_lock()
		-> pte_lockptr(mm, pmd), while the page->ptl is not initialized yet
		*/
		struct pte_data data;

		/*mem_unprotect->pmd = NULL;*/ /*should cleared in init_page_protect_data()*/

		data.mem_unprotect = mem_unprotect;
		data.mask = (~L_PTE_RDONLY);
		data.val = 0; /*(L_PTE_RDONLY);*/

		apply_to_page_range(mm, addr, PAGE_SIZE, __pte_update, &data);
	#endif
		flush_tlb_kernel_page(align_addr);
		/*flush_tlb_kernel_range(align_addr, align_addr + PAGE_SIZE);*/
	}
	isb();

	mem_unprotect->made_writeable = 1;

	return 0;
}

/*refer to mem_text_address_restore()*/
/* this function must be called with rs_rodata_writeable_lock held */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_rodata_address_restore RS_HIDE(_i2)
#endif
noused notrace static int rs_rodata_address_restore(mem_unprotect_struct * mem_unprotect)
{
	if (!rs_arch_has_strict_perms()) {
		return 0;
	}

	if (mem_unprotect->made_writeable) {
		struct mm_struct *mm;

		if (mem_unprotect->pmd) {
			*(mem_unprotect->pmd) = __pmd(pmd_val(mem_unprotect->saved_pmd)); /*mem_unprotect->saved_pmd*/;
			flush_pmd_entry(mem_unprotect->pmd_to_flush);
			flush_tlb_kernel_page(mem_unprotect->addr);
			/*local_flush_tlb_kernel_page(mem_unprotect->addr);*/
		} else {
			/**(mem_unprotect->pte) = mem_unprotect->saved_pte;*/
			set_pte_ext(mem_unprotect->ptep, __pte(pte_val(mem_unprotect->saved_pte)), 0);
			pte_unmap(mem_unprotect->pte);

			flush_tlb_kernel_page(mem_unprotect->addr);
			/*flush_tlb_kernel_range(mem_unprotect->addr, mem_unprotect->addr + PAGE_SIZE);*/
		}
		isb();

		mm = mem_unprotect->active_mm;
		if (mm != &init_mm) {
			mmdrop(mm);
		}
		mem_unprotect->made_writeable = 0;
		return 0;
	}

	return -EINVAL;
}

/*set_page_rw 与 set_page_ro 必须配对使用，因为用到了spinlock*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_i3)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING) {
			int ret;

			rs_rodata_writeable_spinlock(&data->header.flags);
			ret = rs_rodata_address_writeable(_addr, &data->mem_unprotect);
			if (ret) {
				rs_rodata_writeable_spinunlock(&data->header.flags);
			} else {
			#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
				rs_set_memory_rw_generic(_addr, 1);
			#endif
			}

			return ret;
		} else {
			return 0;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_i4)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
		/*mark_rodata_ro()/mark_readonly() called just before system_state set to SYSTEM_RUNNING*/
		if (system_state > SYSTEM_BOOTING) {
			int ret;

		#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
			rs_set_memory_ro_generic(_addr, 1);
		#endif

			ret = rs_rodata_address_restore(&data->mem_unprotect);
			if (!ret) {
				rs_rodata_writeable_spinunlock(&data->header.flags);
			}

			return ret;
		} else {
			return 0;
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#else
/*///////*/

#define RX_AREA_START		PAGE_ALIGN((unsigned long)_stext)
#define RX_AREA_END		PAGE_ALIGN((unsigned long)__end_rodata)

#define page_protect_struct tag_set_page_protect_struct

extern int set_memory_ro(unsigned long virt, int numpages);
extern int set_memory_rw(unsigned long virt, int numpages);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_rw RS_HIDE(_l0)
#endif
noused notrace static int set_page_rw(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_rw_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
		return rs_set_memory_rw_generic(_addr, 1);
	#else
		return 0; /*return set_memory_rw(_addr, 1);*/
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_page_ro RS_HIDE(_l1)
#endif
noused notrace static int set_page_ro(unsigned long _addr, struct page_protect_struct *data)
{
#if 1
	if (
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		(_addr >= module_alloc_base) && (_addr < (module_alloc_base + MODULES_VSIZE))
	#elif defined(CONFIG_MODULES)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#elif defined(CONFIG_MMU)
		(_addr >= VMALLOC_START) && (_addr < VMALLOC_END)
	#elif defined(MODULES_VADDR)
		(_addr >= MODULES_VADDR) && (_addr < MODULES_END)
	#else
		0
	#endif
		) {
	#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
		if ((_addr >= (unsigned long)RX_AREA_START)
			&& (_addr < (unsigned long)RX_AREA_END)) {
			goto handle_core_rx_area;
		} else 
	#endif
		{
			return rs_set_memory_ro_generic(_addr, 1);
		}
	}
#endif

	if ((_addr >= (unsigned long)RX_AREA_START)
		&& (_addr < (unsigned long)RX_AREA_END)) {
#if defined(RS_MODULE_AND_RX_AREA_REGIONS_OVERLAPPING)
	handle_core_rx_area:
#endif
	#if defined(CONFIG_FORCE_PAGES) || defined(CONFIG_KERNEL_TEXT_RDONLY)
		return rs_set_memory_ro_generic(_addr, 1);
	#else
		return 0; /*return set_memory_ro(_addr, 1);*/
	#endif
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define copy_to_ro_page RS_HIDE(_h6)
#endif
noused notrace static int copy_to_ro_page(void *dest, const void *src,
	size_t src_size, noused struct page_protect_struct *data)
{
#if RS_DEBUG
	return (int)probe_kernel_write(dest, src, src_size);
#else
	memmove(dest, src, src_size);
	return 0;
#endif
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_page_protect_data RS_HIDE(_h9)
#endif
noused notrace static void init_page_protect_data(struct page_protect_struct *data, size_t max_data_unit)
{
	if (data) {
		memset(data, 0, sizeof(*data));
		data->header.max_data_unit = max_data_unit;
	}
}

#define RS_FLUSH_MODE_NONE			(0)
#define RS_FLUSH_MODE_RIGHT_AWAY		(1)
#define RS_FLUSH_MODE_GROUP			(2)
#define RS_FLUSH_MODE_ONCE_4_ALL		(3)

/* cache flush mode for rs_set_text_vars(), rs_fill_text_vars() */
#define RS_PATCH_FLUSH_MODE				(RS_FLUSH_MODE_ONCE_4_ALL)

#if defined(RS_SET_PAGE_BY_FIXMAP) && (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
	#error RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY when using fixmap
#endif

/* cache flush mode for __sec_syscall_init_entries() */
#define RS_PATCH_ENTRIES_FLUSH_MODE		(RS_FLUSH_MODE_ONCE_4_ALL)
#define RS_PATCH_CALL_TABLE_FLUSH_MODE		(RS_FLUSH_MODE_GROUP)

#if defined(RS_SET_PAGE_BY_FIXMAP) && (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
	#error RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY when using fixmap
#endif
#if defined(RS_SET_PAGE_BY_FIXMAP) && (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
	#error RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY when using fixmap
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars RS_HIDE(_l2)
#endif
noused notrace static int rs_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count)
{
	noused int ret = 0;
#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
	bool changed = false;
#endif
	size_t i;
	unsigned long dest_addr;
	unsigned long page_addr, prev_page_addr = 0;
#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
	unsigned long flush_start_addr = 0, flush_end_addr = 0;
#endif
	struct page_protect_struct protect_data;

	if ((!target) || (!source) || (!unit_size) || (unit_size > PAGE_SIZE) || (!unit_count)) {
		return -EINVAL;
	}

	init_page_protect_data(&protect_data, unit_size);

	for (i = 0; i < unit_count; i++) {
		prev_page_addr = protect_data.header.prev_page_addr;

		dest_addr = (unsigned long)((char *)target + (unit_size * i));
		page_addr = (dest_addr & PAGE_MASK);
		if (!page_addr) {
			break;
		}

		if (page_addr != prev_page_addr) {
			if (prev_page_addr) {
				ret = set_page_ro(prev_page_addr, &protect_data);
			#if (RS_PATCH_FLUSH_MODE != RS_FLUSH_MODE_GROUP)
				prev_page_addr = 0;
				if (ret) {
					/*fail to set as ro*/
					RS_LOG("stv,0,%d\n", ret);
					break;
				}
			#endif
			}

		#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
			if ((flush_start_addr) && (flush_start_addr < flush_end_addr)) {
				RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
				RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);

				flush_start_addr = 0;
				flush_end_addr = 0;
			}

			if (prev_page_addr) {
				prev_page_addr = 0;
				if (ret) {
					/*fail to set as ro*/
					RS_LOG("stv,0,%d\n", ret);
					break;
				}
			}
		#endif

			/*RS_LOG("stv,0.1\n");*/
			ret = set_page_rw(page_addr, &protect_data);
			if (ret) {
				/*fail to set as rw*/
				RS_LOG("stv,1,%d\n", ret);
				break;
			}
			/*RS_LOG("stv,0.2\n");*/
			prev_page_addr = page_addr;
		}

	#if defined(RS_USE_PROBE_WRITE)
		ret = (int)probe_kernel_write((char *)dest_addr, ((char *)source + (unit_size * i)), unit_size);
		if (ret)
			break;
	#else
		/*target[i] = source[i];*/
		ret = copy_to_ro_page((char *)dest_addr, ((char *)source + (unit_size * i)), unit_size, &protect_data);
		if (ret)
			break;
	#endif

		/*RS_FLUSH_ICACHE_RANGE((unsigned long)dest_addr, (unsigned long)(dest_addr + unit_size));*/
		/*__sync_cache_range_w((unsigned long)dest_addr, unit_size);*/
		/*flush_dcache_range((unsigned long)dest_addr, (unsigned long)(dest_addr + unit_size));*/

	#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
		RS_FLUSH_ICACHE_RANGE(dest_addr, dest_addr + unit_size);
		RS_FLUSH_DCACHE_RANGE(dest_addr, dest_addr + unit_size);
	#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		if (!flush_start_addr) {
			flush_start_addr = dest_addr;
		}
		flush_end_addr = dest_addr + unit_size;
	#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		changed = true;
	#endif

		protect_data.header.prev_page_addr = prev_page_addr;
	}

	if (prev_page_addr) {
		ret = set_page_ro(prev_page_addr, &protect_data);
		/*target[0] = source[0];*/
		RS_LOG("stv,2,%d\n", ret);
	}

#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
	if ((flush_start_addr) && (flush_start_addr < flush_end_addr)) {
		RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
		RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);
	}
#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
	if (changed) {
		RS_FLUSH_ICACHE_RANGE((unsigned long)target, (unsigned long)((char *)target + (unit_size * unit_count)));
		RS_FLUSH_DCACHE_RANGE((unsigned long)target, (unsigned long)((char *)target + (unit_size * unit_count)));
	}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_fill_text_vars RS_HIDE(_l2_)
#endif
noused notrace static int rs_fill_text_vars(void *target, void *value_ptr, size_t value_size, size_t value_count)
{
	noused int ret = 0;
#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
	bool changed = false;
#endif
	size_t i;
	unsigned long dest_addr;
	unsigned long page_addr, prev_page_addr = 0;
#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
	unsigned long flush_start_addr = 0, flush_end_addr = 0;
#endif
	struct page_protect_struct protect_data;

	if ((!target) || (!value_ptr) || (!value_size) || (value_size > PAGE_SIZE) || (!value_count)) {
		return -EINVAL;
	}

	init_page_protect_data(&protect_data, value_size);

	for (i = 0; i < value_count; i++) {
		prev_page_addr = protect_data.header.prev_page_addr;

		dest_addr = (unsigned long)((char *)target + (value_size * i));
		page_addr = (dest_addr & PAGE_MASK);
		if (!page_addr) {
			break;
		}

		if (page_addr != prev_page_addr) {
			if (prev_page_addr) {
				ret = set_page_ro(prev_page_addr, &protect_data);
			#if (RS_PATCH_FLUSH_MODE != RS_FLUSH_MODE_GROUP)
				prev_page_addr = 0;
				if (ret) {
					/*fail to set as ro*/
					break;
				}
			#endif
			}

		#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
			if ((flush_start_addr) && (flush_start_addr < flush_end_addr)) {
				RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
				RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);

				flush_start_addr = 0;
				flush_end_addr = 0;
			}

			if (prev_page_addr) {
				prev_page_addr = 0;
				if (ret) {
					/*fail to set as ro*/
					break;
				}
			}
		#endif

			ret = set_page_rw(page_addr, &protect_data);
			if (ret) {
				/*fail to set as rw*/
				break;
			}

			prev_page_addr = page_addr;
		}

	#if defined(RS_USE_PROBE_WRITE)
		ret = (int)probe_kernel_write((char *)dest_addr, value_ptr, value_size);
		if (ret)
			break;
	#else
		/*target[i] = value;*/
		ret = copy_to_ro_page((char *)dest_addr, value_ptr, value_size, &protect_data);
		if (ret)
			break;
	#endif

		/*RS_FLUSH_ICACHE_RANGE(dest_addr, dest_addr + value_size);*/
		/*__sync_cache_range_w(dest_addr, value_size);*/
		/*flush_dcache_range(dest_addr, dest_addr + value_size));*/

	#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
		RS_FLUSH_ICACHE_RANGE(dest_addr, dest_addr + value_size);
		RS_FLUSH_DCACHE_RANGE(dest_addr, dest_addr + value_size);
	#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		if (!flush_start_addr) {
			flush_start_addr = dest_addr;
		}
		flush_end_addr = dest_addr + value_size;
	#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		changed = true;
	#endif

		protect_data.header.prev_page_addr = prev_page_addr;
	}

	if (prev_page_addr) {
		ret = set_page_ro(prev_page_addr, &protect_data);
		/*target[0] = value;*/
	}

#if (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
	if ((flush_start_addr) && (flush_end_addr)) {
		RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
		RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);
	}
#elif (RS_PATCH_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
	if (changed) {
		RS_FLUSH_ICACHE_RANGE((unsigned long)target, (unsigned long)((char *)target + (value_size * value_count)));
		RS_FLUSH_DCACHE_RANGE((unsigned long)target, (unsigned long)((char *)target + (value_size * value_count)));
	}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars_rw RS_HIDE(_l2_1)
#endif
noused notrace static int rs_set_text_vars_rw(void *target, size_t unit_size, size_t unit_count)
{
	noused int ret = 0;
#if defined(RS_HAS_QCOM_SECURE_BUFFER)
	size_t i;
	unsigned long page_addr, prev_page_addr = 0;

	if ((!target) || (!unit_size) || (unit_size > PAGE_SIZE) || (!unit_count)) {
		return -EINVAL;
	}

	for (i = 0; i < unit_count; i++) {
		page_addr = ((unsigned long)((char *)target + (unit_size * i)) & PAGE_MASK);
		if (!page_addr) {
			break;
		}

		if (page_addr != prev_page_addr) {
			ret = set_kernel_text_pages_rw(page_addr, page_addr + PAGE_SIZE);
			if (ret) {
				/*fail to set as rw*/
				RS_LOG("stvw,1,%d\n", ret);
				break;
			}

			prev_page_addr = page_addr;
		}
	}
#elif defined(CONFIG_ARM_KERNMEM_PERMS) && defined(CONFIG_DEBUG_RODATA)
	/*set_kernel_text_rw();*/
	/*set_kernel_text_page_rw((unsigned long)target);*/
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars_ro RS_HIDE(_l2_2)
#endif
noused notrace static int rs_set_text_vars_ro(void *target, size_t unit_size, size_t unit_count)
{
	noused int ret = 0;
#if defined(RS_HAS_QCOM_SECURE_BUFFER)
	size_t i;
	unsigned long page_addr, prev_page_addr = 0;

	if ((!target) || (!unit_size) || (unit_size > PAGE_SIZE) || (!unit_count)) {
		return -EINVAL;
	}

	for (i = 0; i < unit_count; i++) {
		page_addr = ((unsigned long)((char *)target + (unit_size * i)) & PAGE_MASK);
		if (!page_addr) {
			break;
		}

		if (page_addr != prev_page_addr) {
			ret = set_kernel_text_pages_ro(page_addr, page_addr + PAGE_SIZE);
			if (ret) {
				/*fail to set as rw*/
				RS_LOG("stvo,1,%d\n", ret);
				break;
			}

			prev_page_addr = page_addr;
		}
	}
#elif defined(CONFIG_ARM_KERNMEM_PERMS) && defined(CONFIG_DEBUG_RODATA)
	/*set_kernel_text_ro();*/
	/*set_kernel_text_page_ro((unsigned long)target);*/
#endif

	return ret;
}

noused notrace int __init __sec_syscall_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count)
{
	int ret;

	if ((!target) || (!source) || (!unit_size) || (!unit_count)) {
		return -EINVAL;
	}

	ret = rs_set_text_vars_rw(target, unit_size, unit_count);
	if (!ret) {
		ret = rs_set_text_vars(target, source, unit_size, unit_count);

		rs_set_text_vars_ro(target, unit_size, unit_count);
	}

	return ret;
}
/*/////////////////////////////////////////////////////*/


typedef struct {
	uint16_t index_4_sort;
	uint16_t index_2_entry;
} __attribute__((packed)) index_unit_data;

typedef union {
	index_unit_data data;
	uint32_t value;
} __attribute__((packed)) index_unit;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define swap_ex RS_HIDE(_l4_0)
#endif
noused notrace static void swap_ex(void *a, void *b, int size)
{
	index_unit *x = (index_unit *)a;
	index_unit *y = (index_unit *)b;

	uint32_t tmp = x->value;
	x->value = y->value;
	y->value = tmp;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define cmp_ex RS_HIDE(_l4_1)
#endif
noused notrace static int cmp_ex(const void *a, const void *b)
{
	const index_unit *x = (const index_unit *)a;
	const index_unit *y = (const index_unit *)b;

	int left = (int)x->data.index_4_sort;
	int right = (int)y->data.index_4_sort;
	return (left - right);
}

notrace int __init __sec_syscall_init_entries(const uint16_t *indexes, size_t index_count,
	void_func *call_table, size_t call_table_size, void_func *old_entries,
	const void_func *new_entries)
{
	int ret;
	size_t i;
	extern long sys_ni_syscall(void);
	int dest_is_rw;

	if ((!indexes) || (!index_count) || (!call_table)
		|| (!call_table_size) || (!old_entries) || (!new_entries)) {
		return -EINVAL;
	}

	if ((((unsigned long)old_entries >= (unsigned long)DATA_AREA_START)
		&& ((unsigned long)old_entries < (unsigned long)DATA_AREA_END))
		|| (((unsigned long)old_entries >= (unsigned long)BSS_AREA_START)
		&& ((unsigned long)old_entries < (unsigned long)BSS_AREA_END))
		) {
		dest_is_rw = 1;
	} else {
		dest_is_rw = 0;
	}

	/*first: fill old_entries content*/
	if (!dest_is_rw) {
		unsigned long start_page_addr;
		unsigned long end_page_addr;
		unsigned long page_addr, prev_page_addr = 0;
	#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		unsigned long flush_start_addr = 0, flush_end_addr = 0;
	#endif
	#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		bool changed = false;
	#endif
		struct page_protect_struct protect_data;

		start_page_addr = round_down((unsigned long)&old_entries[0], PAGE_SIZE);
		end_page_addr = round_down((unsigned long)&old_entries[index_count - 1], PAGE_SIZE);

		ret = set_kernel_text_pages_rw(start_page_addr, end_page_addr);
		if (ret) {
			/*fail to set as rw*/
			RS_LOG("ssie,0,%d\n", ret);
			goto out;
		}

		init_page_protect_data(&protect_data, sizeof(old_entries[0]));

		for (i = 0; i < index_count; i++) {
			size_t svc_no = indexes[i];
			if (svc_no < call_table_size) {
				void_func old_entry = call_table[svc_no];
				void_func new_entry = new_entries[i];

				if ((old_entry != (void_func)sys_ni_syscall)
					&& (old_entry != new_entry)
					&& (((unsigned long)old_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)old_entry < (unsigned long)TEXT_AREA_END))
					&& (((unsigned long)new_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)new_entry < (unsigned long)TEXT_AREA_END))
					) {
					unsigned long dest_addr;

					prev_page_addr = protect_data.header.prev_page_addr;

					dest_addr = (unsigned long)(&old_entries[i]);
					page_addr = (dest_addr & PAGE_MASK);
					if (!page_addr) {
						break;
					}

					if (page_addr != prev_page_addr) {
						if (prev_page_addr) {
							ret = set_page_ro(prev_page_addr, &protect_data);
						#if (RS_PATCH_ENTRIES_FLUSH_MODE != RS_FLUSH_MODE_GROUP)
							prev_page_addr = 0;
							if (ret) {
								/*fail to set as ro*/
								RS_LOG("ssie,1,%d\n", ret);
								break;
							}
						#endif
						}

					#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
						if ((flush_start_addr) && (flush_end_addr)) {
							RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
							RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);

							flush_start_addr = 0;
							flush_end_addr = 0;
						}

						if (prev_page_addr) {
							prev_page_addr = 0;
							if (ret) {
								/*fail to set as ro*/
								RS_LOG("ssie,1,%d\n", ret);
								break;
							}
						}
					#endif

						/*RS_LOG("ssie,1.1\n");*/
						ret = set_page_rw(page_addr, &protect_data);
						if (ret) {
							/*fail to set as rw*/
							RS_LOG("ssie,2,%d\n", ret);
							break;
						}

						/*RS_LOG("ssid,2.1\n");*/
						prev_page_addr = page_addr;
					}

					ret = copy_to_ro_page((void *)dest_addr, &old_entry, sizeof(old_entry), &protect_data);
					if (ret) {
						RS_LOG("ssie,2.2,%d\n", ret);
						break;
					}

				#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
					RS_FLUSH_ICACHE_RANGE(dest_addr, dest_addr + sizeof(old_entry));
					RS_FLUSH_DCACHE_RANGE(dest_addr, dest_addr + sizeof(old_entry));
				#elif (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
					if (!flush_start_addr) {
						flush_start_addr = dest_addr;
					}
					flush_end_addr = dest_addr + sizeof(old_entry);
				#elif (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
					changed = true;
				#endif

					protect_data.header.prev_page_addr = prev_page_addr;

					/*RS_LOG("ssie,std1,%p,%p,%p\n", old_entries[i], old_entry, call_table[svc_no]);*/
				}
			}
		}

		if (prev_page_addr) {
			ret = set_page_ro(prev_page_addr, &protect_data);
			RS_LOG("ssie,3,%d\n", ret);
		}

	#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		if ((flush_start_addr) && (flush_end_addr)) {
			RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
			RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);
		}
	#elif (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		if (changed) {
			RS_FLUSH_ICACHE_RANGE((unsigned long)(&old_entries[0]), (unsigned long)((char *)(&old_entries[0]) + (sizeof(old_entries[0]) * index_count)));
			RS_FLUSH_DCACHE_RANGE((unsigned long)(&old_entries[0]), (unsigned long)((char *)(&old_entries[0]) + (sizeof(old_entries[0]) * index_count)));
		}
	#endif

		ret = set_kernel_text_pages_ro(start_page_addr, end_page_addr);
		if (ret) {
			/*fail to set as rw*/
			RS_LOG("ssie,4,%d\n", ret);
			goto out;
		}
	} else {
	#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		bool changed = false;
	#endif
		for (i = 0; i < index_count; i++) {
			size_t svc_no = indexes[i];
			if (svc_no < call_table_size) {
				void_func old_entry = call_table[svc_no];
				void_func new_entry = new_entries[i];
				if ((old_entry != (void_func)sys_ni_syscall)
					&& (old_entry != new_entry)
					&& (((unsigned long)old_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)old_entry < (unsigned long)TEXT_AREA_END))
					&& (((unsigned long)new_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)new_entry < (unsigned long)TEXT_AREA_END))
					) {
				#if RS_DEBUG
					ret = (int)probe_kernel_write(&old_entries[i], &old_entry, sizeof(old_entry));
					if (ret) {
						RS_LOG("ssie,5,%d\n", ret);
						goto out;
					}
				#else
					old_entries[i] = old_entry;
				#endif

				#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
					changed = true;
				#else
					RS_FLUSH_ICACHE_RANGE((unsigned long)(&old_entries[i]), (unsigned long)((char *)(&old_entries[i]) + sizeof(old_entry)));
					RS_FLUSH_DCACHE_RANGE((unsigned long)(&old_entries[i]), (unsigned long)((char *)(&old_entries[i]) + sizeof(old_entry)));
				#endif

					/*RS_LOG("ssie,std2,%p,%p,%p\n", old_entries[i], old_entry, call_table[svc_no]);*/
				}
			}
		}

	#if (RS_PATCH_ENTRIES_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		if (changed) {
			RS_FLUSH_ICACHE_RANGE((unsigned long)(&old_entries[0]), (unsigned long)((char *)(&old_entries[0]) + (sizeof(old_entries[0]) * index_count)));
			RS_FLUSH_DCACHE_RANGE((unsigned long)(&old_entries[0]), (unsigned long)((char *)(&old_entries[0]) + (sizeof(old_entries[0]) * index_count)));
		}
	#endif
	}

	/*second step: fill call_table content*/
	if ((((unsigned long)call_table >= (unsigned long)DATA_AREA_START)
		&& ((unsigned long)call_table < (unsigned long)DATA_AREA_END))
		|| (((unsigned long)call_table >= (unsigned long)BSS_AREA_START)
		&& ((unsigned long)call_table < (unsigned long)BSS_AREA_END))
		) {
		dest_is_rw = 1;
	} else {
		dest_is_rw = 0;
	}

	if (!dest_is_rw) {
		unsigned long start_page_addr;
		unsigned long end_page_addr;
		index_unit *index_data = NULL;
		unsigned long page_addr, prev_page_addr = 0;
	#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		unsigned long flush_start_addr = ULONG_MAX, flush_end_addr = 0;
	#endif
	#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		size_t start_svc_no = call_table_size, end_svc_no = 0;
	#endif
		struct page_protect_struct protect_data;

		index_data = (index_unit *)rs_kmalloc((sizeof(index_unit) * index_count));
		if (!index_data) {
			RS_LOG("ssie,5,%d\n", ret);
			ret = -ENOMEM;
			goto can_rw_out;
		}

		for (i = 0; i < index_count; i++) {
			uint16_t index = indexes[i];
			index_data[i].data.index_4_sort = index;
			index_data[i].data.index_2_entry = i;
		}

		if (index_count > 1) {
			sort(index_data, index_count, sizeof(index_unit), cmp_ex, swap_ex);
		}

		start_page_addr = round_down((unsigned long)&call_table[0], PAGE_SIZE);
		end_page_addr = round_down((unsigned long)&call_table[call_table_size - 1], PAGE_SIZE);

		ret = set_kernel_text_pages_rw(start_page_addr, end_page_addr);
		if (ret) {
			/*fail to set as rw*/
			RS_LOG("ssie,6,%d\n", ret);
			goto can_rw_out;
		}

		init_page_protect_data(&protect_data, sizeof(call_table[0]));

		for (i = 0; i < index_count; i++) {
			size_t svc_no = index_data[i].data.index_4_sort;
			if (svc_no < call_table_size) {
				size_t index_to_entry = index_data[i].data.index_2_entry;

				void_func old_entry = call_table[svc_no];
				void_func new_entry = new_entries[index_to_entry];

				if ((old_entries[index_to_entry])
					&& (old_entry != (void_func)sys_ni_syscall)
					&& (old_entry != new_entry)
					&& (((unsigned long)old_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)old_entry < (unsigned long)TEXT_AREA_END))
					&& (((unsigned long)new_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)new_entry < (unsigned long)TEXT_AREA_END))
					) {
					unsigned long dest_addr;
					/*old_entries[index_to_entry] = old_entry;*/

					prev_page_addr = protect_data.header.prev_page_addr;

					dest_addr = (unsigned long)(&call_table[svc_no]);
					page_addr = (dest_addr & PAGE_MASK);
					if (!page_addr) {
						break;
					}

					if (page_addr != prev_page_addr) {
						if (prev_page_addr) {
							ret = set_page_ro(page_addr, &protect_data);
						#if (RS_PATCH_CALL_TABLE_FLUSH_MODE != RS_FLUSH_MODE_GROUP)
							prev_page_addr = 0;
							if (ret) {
								/*fail to set as ro*/
								RS_LOG("ssie,7,%d\n", ret);
								break;
							}
						#endif
						}

					#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
						if ((flush_start_addr < ULONG_MAX) && (flush_end_addr)) {
							RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
							RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);

							flush_start_addr = ULONG_MAX;
							flush_end_addr = 0;
						}

						if (prev_page_addr) {
							prev_page_addr = 0;
							if (ret) {
								/*fail to set as ro*/
								RS_LOG("ssie,7,%d\n", ret);
								break;
							}
						}
					#endif

						/*RS_LOG("ssie,7.1\n");*/
						ret = set_page_rw(page_addr, &protect_data);
						if (ret) {
							/*fail to set as rw*/
							RS_LOG("ssie,8,%d\n", ret);
							break;
						}

						/*RS_LOG("ssie,8.1\n");*/
						prev_page_addr = page_addr;
					}

					ret = copy_to_ro_page((void *)dest_addr, &new_entry, sizeof(new_entry), &protect_data);
					if (ret) {
						RS_LOG("ssie,8.2,%d\n", ret);
						break;
					}

				#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_RIGHT_AWAY)
					RS_FLUSH_ICACHE_RANGE(dest_addr, dest_addr + sizeof(new_entry));
					RS_FLUSH_DCACHE_RANGE(dest_addr, dest_addr + sizeof(new_entry));
				#elif (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
					if (flush_start_addr > dest_addr) {
						flush_start_addr = dest_addr;
					}

					if (flush_end_addr < (dest_addr + sizeof(new_entry))) {
						flush_end_addr = dest_addr + sizeof(new_entry);
					}
				#elif (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
					if (start_svc_no > svc_no) {
						start_svc_no = svc_no;
					}

					if (end_svc_no < svc_no) {
						end_svc_no = svc_no;
					}
				#endif

					protect_data.header.prev_page_addr = prev_page_addr;

					/*RS_LOG("ssie,std3,%p,%p,%p\n", old_entries[index_to_entry], old_entry, call_table[svc_no]);*/
				}
			}
		}

		if (prev_page_addr) {
			ret = set_page_ro(prev_page_addr, &protect_data);
			RS_LOG("ssie,9,%d\n", ret);
		}

	#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_GROUP)
		if ((flush_start_addr < ULONG_MAX) && (flush_start_addr < flush_end_addr)) {
			RS_FLUSH_ICACHE_RANGE(flush_start_addr, flush_end_addr);
			RS_FLUSH_DCACHE_RANGE(flush_start_addr, flush_end_addr);
		}
	#elif (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		if ((start_svc_no < call_table_size) && (start_svc_no <= end_svc_no)) {
			RS_FLUSH_ICACHE_RANGE((unsigned long)(&call_table[start_svc_no]), (unsigned long)((char *)(&call_table[end_svc_no]) + sizeof(call_table[0])));
			RS_FLUSH_DCACHE_RANGE((unsigned long)(&call_table[start_svc_no]), (unsigned long)((char *)(&call_table[end_svc_no]) + sizeof(call_table[0])));
		}
	#endif

		ret = set_kernel_text_pages_ro(start_page_addr, end_page_addr);
		if (ret) {
			/*fail to set as rw*/
			RS_LOG("ssie,a,%d\n", ret);
			goto can_rw_out;
		}

	can_rw_out:
		if (index_data) {
			rs_kfree(index_data);
		}
	} else {
	#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		size_t start_svc_no = call_table_size, end_svc_no = 0;
	#endif

		for (i = 0; i < index_count; i++) {
			size_t svc_no = indexes[i];
			if (svc_no < call_table_size) {
				void_func old_entry = call_table[svc_no];
				void_func new_entry = new_entries[i];
				if ((old_entries[i])
					&& (old_entry != (void_func)sys_ni_syscall)
					&& (old_entry != new_entry)
					&& (((unsigned long)old_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)old_entry < (unsigned long)TEXT_AREA_END))
					&& (((unsigned long)new_entry >= (unsigned long)TEXT_AREA_START)
					&& ((unsigned long)new_entry < (unsigned long)TEXT_AREA_END))
					) {
					/*old_entries[i] = old_entry;*/
				#if RS_DEBUG
					ret = (int)probe_kernel_write(&call_table[svc_no], &new_entry, sizeof(new_entry));
					if (ret) {
						RS_LOG("ssie,b,%d\n", ret);
						goto out;
					}
				#else
					call_table[svc_no] = new_entry;
				#endif

				#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
					if (start_svc_no > svc_no) {
						start_svc_no = svc_no;
					}

					if (end_svc_no < svc_no) {
						end_svc_no = svc_no;
					}
				#else
					RS_FLUSH_ICACHE_RANGE((unsigned long)(&call_table[svc_no]), (unsigned long)((char *)(&call_table[svc_no]) + sizeof(new_entry)));
					RS_FLUSH_DCACHE_RANGE((unsigned long)(&call_table[svc_no]), (unsigned long)((char *)(&call_table[svc_no]) + sizeof(new_entry)));
				#endif

					/*RS_LOG("ssie,std4,%p,%p,%p\n", old_entries[i], old_entry, call_table[svc_no]);*/
				}
			}
		}

	#if (RS_PATCH_CALL_TABLE_FLUSH_MODE == RS_FLUSH_MODE_ONCE_4_ALL)
		if ((start_svc_no < call_table_size) && (start_svc_no <= end_svc_no)) {
			RS_FLUSH_ICACHE_RANGE((unsigned long)(&call_table[start_svc_no]), (unsigned long)((char *)(&call_table[end_svc_no]) + sizeof(call_table[0])));
			RS_FLUSH_DCACHE_RANGE((unsigned long)(&call_table[start_svc_no]), (unsigned long)((char *)(&call_table[end_svc_no]) + sizeof(call_table[0])));
		}
	#endif
	}

	ret = 0;

out:
	RS_LOG("ssie,e,%d\n", ret);
	return ret;
}

#if defined(RS_MTK_PLATFORM)
/*path of header file for RECOVERY_BOOT etc. is a total mess, just define them directly*/
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 18, 0)
	/*kernel/drivers/misc/mediatek/include/mt-plat/mt_boot_common.h*/
	/*#include "../../../drivers/misc/mediatek/include/mt-plat/mt_boot_common.h"*/
#else
	/*#include "../../mediatek/platform/mt6752/kernel/core/include/mach/mt_boot.h"*/
	/*#include <mach/mt_boot_common.h>*/ /*kernel/include/mach/mt_boot_common.h need add SURVIVAL_BOOT*/
#endif
#endif

#if 1
#if defined(__ro_after_init)
__ro_after_init static int g_ssc_boot_mode = -1;
#else
static const int g_ssc_boot_mode = -1;
#endif
#else
static int g_ssc_boot_mode = -1;
#endif

#if defined(RS_MTK_PLATFORM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	#define SSC_DEFAULT_BOOT_MODE (SSC_NORMAL_BOOT)
#else
	#define SSC_DEFAULT_BOOT_MODE (SSC_UNKNOWN_BOOT)
#endif

noused notrace enum ssc_boot_mode_t ssc_get_boot_mode(void)
{
	if (g_ssc_boot_mode >= 0) {
		return (enum ssc_boot_mode_t)g_ssc_boot_mode;
	} else {
		return SSC_DEFAULT_BOOT_MODE;
	}
}

#if !defined(RS_MTK_PLATFORM)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_file_exist RS_HIDE(_93)
#endif
noused notrace static bool __init is_file_exist(const char *filename)
{
	bool ret = false;

	if ((!filename) || (!filename[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(filename, 0, &file_path);
		if (!error) {
			struct inode *inode = file_path.dentry->d_inode;

			if ((inode) && (!S_ISDIR(inode->i_mode))) {
				ret = true;
			}

			path_put(&file_path);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_dir_exist RS_HIDE(_98)
#endif
noused notrace static bool __init is_dir_exist(const char *dirname)
{
	bool ret = false;

	if ((!dirname) || (!dirname[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(dirname, 0, &file_path);
		if (!error) {
			if (S_ISDIR(file_path.dentry->d_inode->i_mode)) {
				ret = true;
			}

			path_put(&file_path);
		}
	}

	return ret;
}
#endif /* !RS_MTK_PLATFORM */

#if defined(RS_MTK_PLATFORM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))

#if defined(CONFIG_OF)

#include <linux/of_fdt.h>
#include <linux/of.h>

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_boot_mode RS_HIDE(_98_0)
#endif
noused __initdata static u32 g_boot_mode = U32_MAX;

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define dt_get_boot_common RS_HIDE(_98_1)
#endif
noused notrace static int __init dt_get_boot_common(unsigned long node, const char *uname,
	int depth, void *data)
{
	struct tag_bootmode *tag;

	if (unlikely((depth != 1) || ((strcmp(uname, "chosen") != 0)
		&& (strcmp(uname, "chosen@0") != 0))))
		return 0;

	tag = (struct tag_bootmode *)of_get_flat_dt_prop(node,
		"atag,boot", NULL);

	if (likely(tag)) {
		g_boot_mode = tag->bootmode;
	} else {
		RS_LOG("'atag,boot' is not found\n");
	}

	/* break now */
	return 1;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_boot_mode RS_HIDE(_98_2)
#endif
noused notrace static int __init get_boot_mode(void)
{
#if defined(CONFIG_OF)
#if 1
	/* refer to meta_dt_get_mboot_params() */
	struct device_node *np_chosen;
	struct tag_bootmode *tag;

	np_chosen = of_find_node_by_path("/chosen");
	if (!np_chosen)
		np_chosen = of_find_node_by_path("/chosen@0");

	tag = (struct tag_bootmode *)of_get_property(np_chosen, "atag,boot",
							NULL);
	if (likely(tag)) {
		g_boot_mode = tag->bootmode;
	} else {
		RS_LOG("'atag,boot' is not found\n");
	}
#else
	/* refer to init_boot_common() */
	of_scan_flat_dt(dt_get_boot_common, NULL);
#endif

	RS_LOG("gbm,%u\n", g_boot_mode);

	if (g_boot_mode == U32_MAX)
		return SSC_DEFAULT_BOOT_MODE;
	else
		return (int)g_boot_mode;
#else
	return SSC_DEFAULT_BOOT_MODE;
#endif
}
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sec_syscall_init_boot_mode RS_HIDE(_100)
#endif
noused notrace static int __init __sec_syscall_init_boot_mode(void)
{
#if defined(RS_MTK_PLATFORM)
	/*
	 refer to drivers/misc/mediatek/boot/mtk_boot_common.c,
	 pure_initcall(boot_common_core) -> init_boot_common()
	*/

	#ifndef RECOVERY_BOOT
		#define RECOVERY_BOOT (2)
	#endif

	#ifndef FACTORY_BOOT
		#define FACTORY_BOOT (4)
	#endif

	#ifndef SURVIVAL_BOOT
		#define SURVIVAL_BOOT (103)
	#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
	extern int get_boot_mode(void);
#endif

	int boot_mode = get_boot_mode();

	if (RECOVERY_BOOT == boot_mode)
		g_ssc_boot_mode = SSC_RECOVERY_BOOT;
	else if (SURVIVAL_BOOT == boot_mode)
		g_ssc_boot_mode = SSC_SURVIVAL_BOOT;
	else if (FACTORY_BOOT == boot_mode)
		g_ssc_boot_mode = SSC_FACTORY_BOOT;
	else if (boot_mode >= 0)
		g_ssc_boot_mode = SSC_NORMAL_BOOT;

#else

	g_ssc_boot_mode = SSC_NORMAL_BOOT;

#if defined(RS_SAMSANG_PLATFORM)
	{
	#if 0
		/* writel(0, EXYNOSxxxx_POWER_SYSIP_DAT0) called in lk for RECOVERY/FACTORY mode */
		/* refer to drivers/soc/samsung/exynos-reboot.c */
		#define EXYNOS_PMU_SYSIP_DAT0			(0x0810)

		#define REBOOT_MODE_NORMAL		0x00
		#define REBOOT_MODE_CHARGE		0x0A
		/* Reboot into fastboot mode */
		#define REBOOT_MODE_FASTBOOT		0xFC
		/* Reboot into recovery */
		#define REBOOT_MODE_RECOVERY		0xFF
		/* Reboot into recovery */
		#define REBOOT_MODE_FACTORY		0xFD
		#define REBOOT_MODE_SURVIVAL		0xFF

		struct device_node *np;
		void __iomem *exynos_pmu_base;

		RS_LOG("ssibm,1\n");
		np = of_find_compatible_node(NULL, NULL, "exynos,reboot");

		if (np) {
			u32 id;
			RS_LOG("ssibm,2\n");

			if (!of_property_read_u32(np, "pmu_base", &id)) {
				RS_LOG("ssibm,3\n");

				exynos_pmu_base = ioremap(id, SZ_16K);
				if (exynos_pmu_base) {
					void __iomem *addr = exynos_pmu_base + EXYNOS_PMU_SYSIP_DAT0;
					u32 val = __raw_readl(addr);
					RS_LOG("ssibm,4,%x\n", val);

					if (REBOOT_MODE_RECOVERY == val)
						g_ssc_boot_mode = SSC_RECOVERY_BOOT;
				#if (REBOOT_MODE_SURVIVAL != REBOOT_MODE_RECOVERY)
					else if (REBOOT_MODE_RECOVERY == val)
						g_ssc_boot_mode = SSC_SURVIVAL_BOOT;
				#endif
					else if (REBOOT_MODE_FACTORY == val)
						g_ssc_boot_mode = SSC_FACTORY_BOOT;

					iounmap(exynos_pmu_base);
				}
			}

			of_node_put(np);
		}
	#endif
		/* refer to bootargs_process() in lk code */
		#define STR_BUF_LEN (32)

		extern noused char *saved_command_line;
		char str_buf[STR_BUF_LEN];

		if ((sizeof(str_buf) >= sizeof("androidboot.mode=sfactory"))
			&& (sizeof(str_buf) >= sizeof("root=/dev/ram0"))) {
			size_t search_len = strlen(saved_command_line);
			char *p;

			RS_LOG("ssibm,1\n");

			/* androidboot.mode=sfactory */

			str_buf[0] = 'a';
			str_buf[1] = 'n';
			str_buf[2] = 'd';
			str_buf[3] = 'r';
			str_buf[4] = 'o';
			str_buf[5] = 'i';
			str_buf[6] = 'd';
			str_buf[7] = 'b';
			str_buf[8] = 'o';
			str_buf[9] = 'o';
			str_buf[10] = 't';
			str_buf[11] = '.';
			str_buf[12] = 'm';
			str_buf[13] = 'o';
			str_buf[14] = 'd';
			str_buf[15] = 'e';
			str_buf[16] = '=';
			str_buf[17] = 's';
			str_buf[18] = 'f';
			str_buf[19] = 'a';
			str_buf[20] = 'c';
			str_buf[21] = 't';
			str_buf[22] = 'o';
			str_buf[23] = 'r';
			str_buf[24] = 'y';
			str_buf[25] = '\0';

			p = strnstr(saved_command_line, str_buf, search_len);
			if (p) {
				p += (sizeof("androidboot.mode=sfactory") - 1);
				if (*p <= ' ') {
					RS_LOG("ssibm,2\n");
					g_ssc_boot_mode = SSC_FACTORY_BOOT;
				}
			}

			/*
			 removal of 'skip_initramfs', 'ro', 'init=/init' from cmdline may fail,
			 but success in exynos9630 actually, ard 10.0 has 'init=/init' in normal boot
			*/
			/* root=/dev/ram0 */

			str_buf[0] = 'r';
			str_buf[1] = 'o';
			str_buf[2] = 'o';
			str_buf[3] = 't';
			str_buf[4] = '=';
			str_buf[5] = '/';
			str_buf[6] = 'd';
			str_buf[7] = 'e';
			str_buf[8] = 'v';
			str_buf[9] = '/';
			str_buf[10] = 'r';
			str_buf[11] = 'a';
			str_buf[12] = 'm';
			str_buf[13] = '0';
			str_buf[14] = '\0';

			p = strnstr(saved_command_line, str_buf, search_len);
			if (p) {
				p += (sizeof("root=/dev/ram0") - 1);
				if (*p <= ' ') {
					RS_LOG("ssibm,3\n");

					str_buf[0] = 'i';
					str_buf[1] = 'n';
					str_buf[2] = 'i';
					str_buf[3] = 't';
					str_buf[4] = '=';
					str_buf[5] = '/';
					str_buf[6] = 'i';
					str_buf[7] = 'n';
					str_buf[8] = 'i';
					str_buf[9] = 't';
					str_buf[10] = '\0';

					if (!strnstr(saved_command_line, str_buf, search_len)) {
						RS_LOG("ssibm,4\n");
						g_ssc_boot_mode = SSC_RECOVERY_BOOT;
					}
				}
			}
		}
	}
#endif /* RS_SAMSANG_PLATFORM */

	{
		extern noused char *saved_command_line;
		#define STR_BUF_LEN (32)

		char str_buf[STR_BUF_LEN];

		if ((sizeof(str_buf) >= sizeof("recoverymode=1"))
			&& (sizeof(str_buf) >= sizeof("survivalmode=1"))
			&& (sizeof(str_buf) >= sizeof("androidboot.boot_devices="))) {
			size_t search_len = strlen(saved_command_line);

			str_buf[0] = 'r';
			str_buf[1] = 'e';
			str_buf[2] = 'c';
			str_buf[3] = 'o';
			str_buf[4] = 'v';
			str_buf[5] = 'e';
			str_buf[6] = 'r';
			str_buf[7] = 'y';
			str_buf[8] = 'm';
			str_buf[9] = 'o';
			str_buf[10] = 'd';
			str_buf[11] = 'e';
			str_buf[12] = '=';
			str_buf[13] = '1';
			str_buf[14] = '\0';

			if (strnstr(saved_command_line, str_buf, search_len)) {
				RS_LOG("ssibm,5\n");
				g_ssc_boot_mode = SSC_RECOVERY_BOOT;
			} else {
				str_buf[0] = 's';
				str_buf[1] = 'u';
				str_buf[2] = 'r';
				str_buf[3] = 'v';
				str_buf[4] = 'i';
				str_buf[5] = 'v';
				str_buf[6] = 'a';
				str_buf[7] = 'l';
			#if 0
				/*the remain part is same as the first pattern*/
				str_buf[8] = 'm';
				str_buf[9] = 'o';
				str_buf[10] = 'd';
				str_buf[11] = 'e';
				str_buf[12] = '=';
				str_buf[13] = '1';
				str_buf[14] = '\0';
			#endif

				if (strnstr(saved_command_line, str_buf, search_len)) {
					RS_LOG("ssibm,6\n");
					g_ssc_boot_mode = SSC_SURVIVAL_BOOT;
				} else {
					/*check if dynamic partitions supported, 'androidboot.boot_devices=soc/XX' */
					str_buf[0] = 'a';
					str_buf[1] = 'n';
					str_buf[2] = 'd';
					str_buf[3] = 'r';
					str_buf[4] = 'o';
					str_buf[5] = 'i';
					str_buf[6] = 'd';
					str_buf[7] = 'b';
					str_buf[8] = 'o';
					str_buf[9] = 'o';
					str_buf[10] = 't';
					str_buf[11] = '.';
					str_buf[12] = 'b';
					str_buf[13] = 'o';
					str_buf[14] = 'o';
					str_buf[15] = 't';
					str_buf[16] = '_';
					str_buf[17] = 'd';
					str_buf[18] = 'e';
					str_buf[19] = 'v';
					str_buf[20] = 'i';
					str_buf[21] = 'c';
					str_buf[22] = 'e';
					str_buf[23] = 's';
					str_buf[24] = '=';
					str_buf[25] = '\0';

					if (strnstr(saved_command_line, str_buf, search_len)) {
						/* dynamic partitions supported */
						str_buf[0] = '/';
						str_buf[1] = 't';
						str_buf[2] = 'm';
						str_buf[3] = 'p';
						str_buf[4] = '\0';

						if (is_dir_exist(str_buf)) {
							/* check is_file_exist("/plat_file_contexts") further? */
							RS_LOG("ssibm,7\n");
							g_ssc_boot_mode = SSC_RECOVERY_BOOT;
						}
					} else {
					#if !(defined(RS_SAMSANG_PLATFORM) && defined(RS_IS_ANDROID_10_ABOVE))
						/* no 'skip_initramfs' if dynamic partition being used */
						/*normal mode with 'skip_initramfs', because root in system image */
						/*samsung 10.0 no skip_initramfs in normal mode */
						str_buf[0] = 's';
						str_buf[1] = 'k';
						str_buf[2] = 'i';
						str_buf[3] = 'p';
						str_buf[4] = '_';
						str_buf[5] = 'i';
						str_buf[6] = 'n';
						str_buf[7] = 'i';
						str_buf[8] = 't';
						str_buf[9] = 'r';
						str_buf[10] = 'a';
						str_buf[11] = 'm';
						str_buf[12] = 'f';
						str_buf[13] = 's';
						str_buf[14] = '\0';

						if (!strnstr(saved_command_line, str_buf, search_len)) {
							RS_LOG("ssibm,8\n");
							g_ssc_boot_mode = SSC_RECOVERY_BOOT;
						}
					#endif
					}
				}
			}
		}
	}
#endif

	printk("ssc: bm,%d\n", g_ssc_boot_mode);
	return 0;
}

fs_initcall(__sec_syscall_init_boot_mode); /* must before __sec_syscall_init_mode() */
