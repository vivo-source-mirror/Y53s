
#include <linux/compiler.h>
#include <linux/syscalls.h>
#include <linux/export.h>
#include <linux/capability.h>
#include <linux/mnt_namespace.h>
#include <linux/user_namespace.h>
#include <linux/namei.h>
#include <linux/security.h>
#include <linux/cred.h>
#include <linux/init.h>		/* init_rootfs */
#include <linux/fs_struct.h>	/* get_fs_root et.al. */
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/mount.h>
#include <linux/proc_ns.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 11, 0)
#include <linux/sched/task.h>
#endif


/*lizonglin transplant for is_root function start*/

#if defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM)

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 18, 0)
	/*kernel/drivers/misc/mediatek/include/mt-plat/mt_boot_common.h*/
	#include "../drivers/misc/mediatek/include/mt-plat/mtk_boot_common.h"
#else
	#include <mach/mt_boot_common.h> /* kernel/include/mach/mt_boot_common.h need add SURVIVAL_BOOT */
#endif
#endif

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
/*lizonglin transplant end*/
#include <linux/blkdev.h>
#include <linux/genhd.h>

/*lizonglin transplant for is_root function start*/
static int isMountFlagEnabled = 1;
#if !(defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM))
static int isMountRecord = 1;
#endif

#include <linux/fdtable.h>

#include <linux/binfmts.h>

static int mountFlag; /*LiZhao*/
static struct proc_dir_entry *is_root_dir;
static struct proc_dir_entry *is_root_ent;
/*static struct isroot_value *rootvalue;*/

/*lizonglin transplant end*/

#include "vrr_internal.hpp"


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
#include <linux/uidgid.h>
#endif

#include <linux/xattr.h>
#include <linux/mtd/mtd.h>

#include <linux/random.h>

#include <linux/vmalloc.h>
#include <linux/module.h>
/*#include <linux/moduleloader.h>*/

#include <linux/crc32.h>

#include <linux/utsname.h>
#include <linux/time.h>
/*#include <linux/timex.h>*/
#include <linux/rtc.h>

#include <asm/sections.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#if defined(RS_SET_PAGE_BY_FIXMAP)
#include <asm/fixmap.h>
#endif

#if defined(RS_QUALCOMM_PLATFORM) && !defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 18, 0)
#include <asm/mmu_writeable.h> /*_MMU_WRITEABLE_H*/
#endif
#endif
/*#include <asm/rodata.h>*/ /*_ASMARM_RODATA_H got defined, and rodata.h included in cacheflush.h*/
/*#include <asm/pgtable.h>*/

#include "./proc/internal.h"
#include "./internal.h"

#include <linux/futex.h>
#include <linux/random.h>
#include <linux/mm.h>


#include <net/sock.h>
#include <linux/socket.h>
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

/*#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)*/
#include <linux/kthread.h>
/*#endif*/

#if defined(CONFIG_KASAN)
#include <linux/kasan.h>
#endif

#include "mount.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
#include <asm/ptrace.h>
#include <asm/syscall.h>
#else
#if defined(CONFIG_ARM) && !defined(CONFIG_ARM64)
static inline int syscall_get_nr(struct task_struct *task,
				struct pt_regs *regs)
{
	return task_thread_info(task)->syscall;
}

static inline unsigned long user_stack_pointer(struct pt_regs *regs)
{
	return regs->ARM_sp;
}

#if !defined(instruction_pointer)
	#define instruction_pointer(regs)		((regs)->ARM_pc)
#endif

#endif
#endif

#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)

#if defined(SYSCALL_DEFINE_MAXARGS) && (SYSCALL_DEFINE_MAXARGS > 0)
	#define RS_SYSCALL_DEFINE_MAXARGS (SYSCALL_DEFINE_MAXARGS)
#else
	#define RS_SYSCALL_DEFINE_MAXARGS (6)
#endif

struct rs_user_pt_regs {
	__u64		regs[RS_SYSCALL_DEFINE_MAXARGS];
};

struct rs_pt_regs {
	union {
		struct rs_user_pt_regs user_regs;
		struct {
			u64 regs[RS_SYSCALL_DEFINE_MAXARGS];
		};
	};
};

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __is_kernel_rodata RS_HIDE(_0)
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
	/*if (buf)*/
	{
		kfree(buf);
	}
}

struct align_header {
	uintptr_t offset;
	size_t magic;
};

#if defined(CONFIG_ARM64)
	#define MEM_ALIGN_HDR_MAGIC 0x414136344D41484D /*"4141AA64MAHM"*/
#else
	#define MEM_ALIGN_HDR_MAGIC 0x4D41484D /*"MAHM"*/
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_memalign RS_HIDE(_2_1)
#endif
noused notrace static void *rs_memalign(size_t alignment, size_t size)
{
	struct align_header *h;
	size_t extra_size;
	char *ptr, *ret = NULL;
	/*assert(!(alignment & (alignment - 1)));*/

	if ((alignment & (alignment - 1))) {
		return ERR_PTR(-EINVAL);
	}

	extra_size = (size + alignment - 1);
	extra_size += sizeof(*h);

	ptr = (char *)kmalloc(size + extra_size, GFP_KERNEL);
	if (!IS_ERR(ptr)) {
		ret = ptr + sizeof(*h);
		ret = (char *)round_up((uintptr_t)ret, alignment);

		h = (struct align_header *)(ret - sizeof(*h));
		h->offset = ret - ptr;
		h->magic = MEM_ALIGN_HDR_MAGIC;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_memalign_free RS_HIDE(_2_2)
#endif
noused notrace static void rs_memalign_free(void *ptr)
{
	if (ptr) {
		struct align_header *h = (struct align_header *)((char *)ptr - sizeof(*h));

		if (h->magic == MEM_ALIGN_HDR_MAGIC) {
			kfree((char *)ptr - h->offset);
		}
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_free_pages RS_HIDE(_3)
#endif
noused notrace static unsigned long rs_get_free_pages(unsigned int order)
{
	unsigned long ret = __get_free_pages(RS_KMALLOC_FLAG, order);

	if (!ret) {
		/*!wait and use emergency pool*/
		ret = __get_free_pages(GFP_ATOMIC, order);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_free_pages RS_HIDE(_4)
#endif
noused notrace static void rs_free_pages(unsigned long addr, unsigned int order)
{
	/*if (addr)*/
	{
		free_pages(addr, order);
	}
}

#define rs_get_free_page() rs_get_free_pages(0)
#define rs_free_page(addr) rs_free_pages((unsigned long)(addr), 0)

#if (PAGE_SIZE < PATH_MAX)
	#define RS_BUFFER_SIZE			PATH_MAX
	#define RS_SYS_BUFFER_SIZE		PAGE_SIZE
	#define RS_GET_BUFFER()			rs_kmalloc(PATH_MAX) /*kmalloc(PATH_MAX, RS_KMALLOC_FLAG)*/
	#define RS_FREE_BUFFER(buf)		rs_kfree(buf) /*kfree(buf)*/

	#define RS_GET_TWO_BUFFER()		rs_kmalloc(PATH_MAX << 1) /*kmalloc(PATH_MAX << 1, RS_KMALLOC_FLAG)*/
	#define RS_FREE_TWO_BUFFER(buf)		rs_kfree(buf) /*kfree(buf)*/
#else
	#define RS_BUFFER_SIZE			PAGE_SIZE
	#define RS_SYS_BUFFER_SIZE		PATH_MAX
	#define RS_GET_BUFFER()			rs_get_free_page() /*__get_free_page(RS_KMALLOC_FLAG)*/
	#define RS_FREE_BUFFER(buf)		rs_free_page(buf) /*free_page((unsigned long)(buf))*/

	#define RS_GET_TWO_BUFFER()		rs_get_free_pages(1) /*__get_free_pages(RS_KMALLOC_FLAG, 1)*/
	#define RS_FREE_TWO_BUFFER(buf)		rs_free_pages((unsigned long)(buf), 1) /*free_pages((unsigned long)(buf), 1)*/
#endif


#define RCU_READ_LOCK rcu_read_lock()
#define RCU_READ_UNLOCK rcu_read_unlock()

#define TASKLIST_READ_LOCK read_lock(&tasklist_lock)
#define TASKLIST_READ_UNLOCK read_unlock(&tasklist_lock)

#define RS_READ_TASK_LOCK()   RCU_READ_LOCK
#define RS_READ_TASK_UNLOCK() RCU_READ_UNLOCK


/*android 5.0 0x0000001fffffffff, 之前是 0xffffffffffffffff*/
#define ZYGOTE_CAP_MASK (0x0000001fffffffff)

/*frameworks/base/core/java/com/android/internal/os/ZygoteInit.java startSystemServer()"--capabilities=130104352,130104352",*/
#define SYSTEM_SERVER_CAP_VALUE (130104352)

/*#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
8974 4.4 内核是 3.4.0，未定义CAP_BLOCK_SUSPEND*/

#undef VALUE_SHIFT_SEED

#if !defined(CAP_BLOCK_SUSPEND)
	#define CAP_BLOCK_SUSPEND (36)
	#define VALUE_SHIFT_SEED (0ull)
#else
	#define VALUE_SHIFT_SEED (1ull)
#endif

#define CAP_BLOCK_SUSPEND_VAL (VALUE_SHIFT_SEED << CAP_BLOCK_SUSPEND)

#if defined(CAP_BLOCK_SUSPEND)
/*0x1007813C20 36bits showed via "cat proc/pid/status"
8974 android 4.4, ART模式下0x07813C20,非ART模式下0x1007813C20*/
#define SYSTEM_SERVER_CAP_VALUE2 \
	( \
	(1ull << CAP_KILL) | \
	(1ull << CAP_NET_ADMIN) | \
	(1ull << CAP_NET_BIND_SERVICE) | \
	(1ull << CAP_NET_BROADCAST) | \
	(1ull << CAP_NET_RAW) | \
	(1ull << CAP_SYS_MODULE) | \
	(1ull << CAP_SYS_NICE) | \
	(1ull << CAP_SYS_RESOURCE) | \
	(1ull << CAP_SYS_TIME) | \
	(1ull << CAP_SYS_TTY_CONFIG) | \
	CAP_BLOCK_SUSPEND_VAL \
	)
#endif

typedef union cast_kernel_cap_struct {
	kernel_cap_t cap;
#if (_KERNEL_CAPABILITY_U32S == 1)
	__u32 val;
#elif (_KERNEL_CAPABILITY_U32S == 2)
	__u64 val;
#elif (_KERNEL_CAPABILITY_U32S == 4)
	__u128 val;
#else
	#error "_KERNEL_CAPABILITY_U32S value is odd to me!"
#endif
} cast_kernel_cap_t;

#define RS_CAP_GET cap_capget /*security_capget*/


#define RS_FILP_CLOSE_ID (current->files) /*NULL*/

#define RS_READ_DIR_OPEN_FLAGS (O_RDONLY | O_DIRECTORY | O_NOATIME | O_CLOEXEC)
#define RS_READ_DIR_OPEN_MODE (0)
#define RS_READ_DIR_KERN_PATH_FLAGS (LOOKUP_FOLLOW) /*|LOOKUP_DIRECTORY|LOOKUP_AUTOMOUNT*/

#define RS_READ_FILE_OPEN_FLAGS (O_RDONLY | O_NOATIME | O_CLOEXEC)
#define RS_READ_FILE_OPEN_MODE (0) /*0666*/
#define RS_WRITE_FILE_OPEN_FLAGS (O_RDWR | O_NOATIME | O_CLOEXEC)
#define RS_WRITE_FILE_OPEN_MODE (0644) /*0755*/

#ifdef O_LARGEFILE
	#define RS_IOCTL_OPEN_FLAGS (O_RDONLY | O_NONBLOCK | O_LARGEFILE | O_NOATIME | O_CLOEXEC)
#else
	#define RS_IOCTL_OPEN_FLAGS (O_RDONLY | O_NONBLOCK | O_NOATIME | O_CLOEXEC)
#endif

#define RS_IOCTL_OPEN_MODE (0)

#define RS_FS_IMMUTABLE_FLAGS (FS_IMMUTABLE_FL | FS_APPEND_FL)

#define RS_FILE_EXIST_KERN_PATH_FLAGS (0)
#define RS_REAL_FILE_KERN_PATH_FLAGS (LOOKUP_FOLLOW)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
	#define RS_GET_RANDOM_BYTES get_random_bytes
#else
	#define RS_GET_RANDOM_BYTES get_random_bytes_arch
#endif

/*sys.c sys_getppid()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_task_ppid RS_HIDE(_10)
#endif
noused notrace static int rs_get_task_ppid(struct task_struct *task)
{
	int pid;
	rcu_read_lock();
	/*pid = task_tgid_vnr(rcu_dereference(task->real_parent));*/ /*id seen from the ns specified*/
	pid = pid_nr(task_tgid(task->real_parent));
	rcu_read_unlock();

	return pid;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_task_pid RS_HIDE(_11)
#endif
noused notrace static int rs_get_task_pid(struct task_struct *task)
{
	int pid;
	rcu_read_lock();
	/*pid = task_tgid_vnr(task);*/ /*id seen from the ns specified*/
	pid = pid_nr(task_tgid(task)); /*id seen from the init namespace*/
	rcu_read_unlock();

	return pid;
}

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_task_init RS_HIDE(_12)
#endif
noused notrace static int is_task_init(struct task_struct *tsk)
{
	int ret = 0;
	struct pid *pid;

	rcu_read_lock();
	pid = task_pid(tsk); /*task_tgid(tsk);*/
	/*if (pid != NULL && pid->numbers[pid->level].nr == 1)*/
	/*refer to pid_nr(), global id, i.e. the id seen from the init namespace*/
	if (pid_nr(pid) == 1)
		ret = 1;
	rcu_read_unlock();

	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_init_task RS_HIDE(_13)
#endif
noused notrace static int is_init_task(const struct task_struct *task)
{
	/*对coolroot单纯判断pid==1或tgid==1不行，加上 real_parent->pid==0 ?
	 *init_task是idle{swapper/0}的task_struct,init的parent总是swapper/0
	 */
	/*#define is_init_task(task) ((task)->real_parent == &init_task)*/ /*is_global_init(task)*/ /*((task)->pid == 1)*/ /*((task)->parent == &init_task)*/

	return (task->real_parent == &init_task) ? 1 : 0;
	/*按namespace来说，应该用global pid判断*/
	/*return ((task->pid == 1) && (rs_get_task_pid(task) == 1)) ? 1 : 0;*/
	/*return (rs_get_task_ppid(task) == 0);*/
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define should_retry_syscall RS_HIDE(_14)
#endif
noused notrace static int should_retry_syscall(int err_no)
{
	/* not safe to loop in kernel space, may result in process hanging */
	/*android 上 EWOULDBLOCK == EAGAIN*/
	if (unlikely(((err_no == -EINTR) || (err_no == -EAGAIN)
		|| (err_no == -ERESTARTSYS) || (err_no == -ERESTARTNOINTR)
		|| (err_no == -ERESTARTNOHAND) || (err_no == -ERESTART_RESTARTBLOCK)
		) && (!/*signal_pending*/fatal_signal_pending(current)))) {
		return 1;
	} else {
		return 0;
	}
}


enum rs_boot_mode_t {
	RS_NORMAL_BOOT = 0,
	RS_RECOVERY_BOOT = 1,
	RS_FACTORY_BOOT = 2,
	RS_SURVIVAL_BOOT = 3,
	RS_UNKNOWN_BOOT,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_boot_mode RS_HIDE(_15)
#endif

#if 1
#if defined(__ro_after_init)
__ro_after_init static int g_rs_boot_mode = -1;
#else
static const int g_rs_boot_mode = -1;
#endif
#else
static int g_rs_boot_mode = -1;
#endif

#if defined(RS_MTK_PLATFORM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	#define RS_DEFAULT_BOOT_MODE (RS_NORMAL_BOOT)
#else
	#define RS_DEFAULT_BOOT_MODE (RS_UNKNOWN_BOOT)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_boot_mode RS_HIDE(_16)
#endif
noused notrace enum rs_boot_mode_t rs_get_boot_mode(void)
{
	if (g_rs_boot_mode >= 0) {
		return (enum rs_boot_mode_t)g_rs_boot_mode;
	} else {
		return RS_DEFAULT_BOOT_MODE;
	}
}

enum {
	ovMount,
	ovInsmod,
	ovSetuid,
	ovExec,
	ovPtrace,
	ovChRoot,
	ovPivotRoot,
	ovAccessDev,
	ovDisableSELinux,
	ovFilesCheck,
	ovSuExec,
	ovDynTransToSu,
	ovCheckBoot,
	ovFuncCheck,
	ovSockCheck,
	ovLoadSELinuxPolicy,

	ovInvalid
};

enum {
	catMount,
	catInsmod,
	catSuDaemon,
	catBusybox,

	catInvalid,
};

enum {
	rmtSystem,
	rmtRoot,
	rmtApps,
	rmtTmp,
	rmtOem,
	rmtVendor,

	rmtInvalid,
};

#if 0
#if defined(CONFIG_CONFIG_RS_PERMIT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_permit RS_HIDE(_20)
#endif
noused static unsigned long g_rs_permit;
#endif
#if defined(CONFIG_CONFIG_RS_BYPASS)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_bypass RS_HIDE(_21)
#endif
noused static unsigned long g_rs_bypass;
#endif
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_vars_lock RS_HIDE(_21_)
#endif

#if defined(RS_VARS_LOCK_USE_RWSEM)
noused static DECLARE_RWSEM(g_rs_vars_lock);

#define RS_VAR_LOCK_INIT do {} while (0)

#define RS_VAR_READ_LOCK down_read(&g_rs_vars_lock)
#define RS_VAR_READ_UNLOCK up_read(&g_rs_vars_lock)

#define RS_VAR_WRITE_LOCK down_write(&g_rs_vars_lock)
#define RS_VAR_WRITE_UNLOCK up_write(&g_rs_vars_lock)

#else
noused static DEFINE_RWLOCK(g_rs_vars_lock);

#define RS_VAR_LOCK_INIT do {} while (0)

#define RS_VAR_READ_LOCK read_lock(&g_rs_vars_lock)
#define RS_VAR_READ_UNLOCK read_unlock(&g_rs_vars_lock)

#define RS_VAR_WRITE_LOCK write_lock(&g_rs_vars_lock)
#define RS_VAR_WRITE_UNLOCK write_unlock(&g_rs_vars_lock)
#endif

#ifndef U8_MAX
	#define U8_MAX		((u8)~0U)
#endif

enum {
	opvChecksum,
	opvPermit,
	opvBypass,

	opvInvalid,
};

/*#define RS_OP_VAR_CHECKSUM_FLAG (1)*/
/*#define RS_OP_VAR_PERMIT_FLAG (2)*/
/*#define RS_OP_VAR_BYPASS_FLAG (4)*/

#define RS_OP_VAR_FLAG_MASK ((1 << opvInvalid) - 1)

/*#define RS_OP_CHECKSUM_VAR_PAGE_OFFSET (0)*/
/*#define RS_OP_PERMIT_VAR_PAGE_OFFSET (512)*/
/*#define RS_OP_BYPASS_VAR_PAGE_OFFSET (1024)*/

#define RS_OP_VAR_PAGE_DATA_SIZE (1024 * (opvInvalid + 1))

#define RS_OP_VAR_FLAG_TYPE u32

#define RS_HIDE_OP_VAR_FLAG /*隐藏g_rs_op_var_flag信息*/

/*高16位保存下面xx_vars数组相关信息*/
#if !defined(RS_HIDE_OP_VAR_FLAG)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_op_var_flag RS_HIDE(_1a)
#endif
noused static RS_OP_VAR_FLAG_TYPE g_rs_op_var_flag;
#else
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_op_var_flags RS_HIDE(_1a)
#endif
noused static RS_OP_VAR_FLAG_TYPE g_rs_op_var_flags[sizeof(RS_OP_VAR_FLAG_TYPE)];
#endif

#define RS_OP_VAR_TUPLE_ELEMENTS_USED_COUNT (3) /*!不要修改!*/

#define RS_OP_VAR_TUPLE_ELEMENTS_COUNT (4) /*不能超过16(4bits能保存的最大值)*/

#if (RS_OP_VAR_TUPLE_ELEMENTS_COUNT < RS_OP_VAR_TUPLE_ELEMENTS_USED_COUNT)
	#error "error: RS_OP_VAR_TUPLE_ELEMENTS_COUNT < RS_OP_VAR_TUPLE_ELEMENTS_USED_COUNT"
#endif

#if defined(RS_MTK_PLATFORM)
	#define RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT (4) /*不能超过16*/
#else
	#define RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT (8) /*不能超过16*/
#endif

#define RS_OP_VARS_STORE_COUNT (RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT + RS_OP_VAR_TUPLE_ELEMENTS_COUNT - 1)

#define RS_OP_VARS_OFFS_RANDOM_SHIFT ((sizeof(uintptr_t) >> 3) + 1)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_op_vars RS_HIDE(_1b)
#endif
/*noused static uintptr_t g_rs_op_vars[(RS_OP_VAR_COUNT * 2) - 1] __rs_text = {0};*/
noused static uintptr_t g_rs_op_vars[RS_OP_VARS_STORE_COUNT] __rs_text = {0};

#define RS_OP_VARS_STORE_SIZE sizeof(g_rs_op_vars)
#define RS_OP_VAR_TUPLE_ELEMENTS_SIZE (sizeof(g_rs_op_vars[0]) * RS_OP_VAR_TUPLE_ELEMENTS_COUNT)

/*#define RS_OP_VAR_TUPLE_ELEMENTS_UNUSED_SIZE (sizeof(g_rs_op_vars[0]) * (RS_OP_VAR_TUPLE_ELEMENTS_COUNT - RS_OP_VAR_TUPLE_ELEMENTS_USED_COUNT))*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_forbid_kern_mount RS_HIDE(_1c)
#endif
noused static uintptr_t g_rs_forbid_kern_mount __rs_text;

#if defined(CONFIG_DO_RS_JOURNAL)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_software_version_str RS_HIDE(_22)
#endif
noused static char *g_software_version_str;
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_software_version_str_len RS_HIDE(_23)
#endif
noused static int g_software_version_str_len;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_enable_journal RS_HIDE(_24)
#endif
noused static unsigned int g_rs_enable_journal
#if  (DEFAULT_RS_JOURNAL_ENABLE)
	= DEFAULT_RS_JOURNAL_ENABLE
#endif
	;
#endif

#define RS_BLK_DEV_MIN_TOTAL_SIZE ((1024 * 1024) * 10) /*从72平台到目前backup分区至少10MB*/
#define RS_BLK_DEV_TOTAL_SIZE_MASK ((1024 * 1024) - 1)

#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	#define RS_BLK_DEV_DATA_SIZE ((1024) * 64)
#else
	#define RS_BLK_DEV_DATA_SIZE ((1024 * 1024) * 1)
#endif
#define RS_BLK_DEV_DATA_SIGNATURE_OFFSET (1024)
#define RS_BLK_DEV_DATA_CONFIG_DATA_OFFSET (4096)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_blk_dev_size RS_HIDE(_25)
#endif
noused static loff_t g_blk_dev_size;

#if RS_DEBUG

#define RS_LOG_LEVEL KERN_ALERT /*KERN_DEBUG*/

#define RS_DBG_EVT_NONE		(0)	/* No event */

#define RS_DBG_EVT_EMERG		(1 << 0)	/* system is unusable				   */
#define RS_DBG_EVT_ALERT		(1 << 1)	/* action must be taken immediately	 */
#define RS_DBG_EVT_CRIT		(1 << 2)	/* critical conditions				  */
#define RS_DBG_EVT_ERR			(1 << 3)	/* error conditions					 */
#define RS_DBG_EVT_WARN		(1 << 4)	/* warning conditions				   */
#define RS_DBG_EVT_NOTICE		(1 << 5)	/* normal but significant condition	 */
#define RS_DBG_EVT_INFO		(1 << 6)	/* informational						*/
#define RS_DBG_EVT_DBG			(1 << 7)	/* debug-level messages				 */

#define RS_DBG_EVT_ALL		(0xffffffff)

#define RS_DBG_EVT_MASK	(RS_DBG_EVT_ALL)


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
	#define RS_LOG(fmt, args...)	printk(RS_LOG_LEVEL "rs: " fmt, ## args)

	#define RS_XLOG(evt, fmt, args...) \
		do { \
			if ((RS_DBG_EVT_##evt) & RS_DBG_EVT_MASK) \
				printk(RS_LOG_LEVEL "rs: " fmt, ## args); \
		} while (0)
#endif

	/*#define  RS_LOG(...)	printk(__VA_ARGS__)*/
	/*#define  RS_LOG(...)	xlog_printk(ANDROID_LOG_DEBUG, "RS_TAG", __VA_ARGS__);*/
#else
	#define RS_LOG(...)	do {} while (0)
	#define RS_XLOG(...)	do {} while (0)
#endif

#undef DLOG

#if RS_TEMP_LOG
	#define DLOG(fmt, args...)	printk("rsd:" fmt, ## args)
#else
	#define DLOG(...)	do {} while (0)
#endif

#ifndef strict_strtoul
	#define strict_strtoul  kstrtoul
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
	#define __kuid_val(uid) (uid)
	#define __guid_val(gid) (gid)
#endif

#ifdef __LEADING_UNDERSCORE
#define	_C_LABEL(x)	__CONCAT(_, x)
#define _C_LABEL_STRING(x)	"_"x
#else
#define	_C_LABEL(x)	x
#define _C_LABEL_STRING(x)	x
#endif

#define	__strong_alias(alias, sym)						\
	__asm__(".global " _C_LABEL_STRING(#alias) "\n"			\
		_C_LABEL_STRING(#alias) " = " _C_LABEL_STRING(#sym));

#define	__weak_alias(alias, sym)						\
	__asm__(".weak " _C_LABEL_STRING(#alias) "\n"			\
		_C_LABEL_STRING(#alias) " = " _C_LABEL_STRING(#sym));

#define	__weak_extern(sym)						\
	__asm__(".weak " _C_LABEL_STRING(#sym));

#define CHECK_ROOT_UID(x)	(__kuid_val((x)->cred->uid) == 0 || __kgid_val((x)->cred->gid) == 0 || \
						__kuid_val((x)->cred->euid) == 0 || __kgid_val((x)->cred->egid) == 0 || \
						__kuid_val((x)->cred->suid) == 0 || __kgid_val((x)->cred->sgid) == 0)


#define CHECK_ROOT_UID_BY_CRED(x)	(__kuid_val((x)->uid) == 0 || __kgid_val((x)->gid) == 0 || \
						__kuid_val((x)->euid) == 0 || __kgid_val((x)->egid) == 0 || \
						__kuid_val((x)->suid) == 0 || __kgid_val((x)->sgid) == 0)

#define CHECK_UID(x, id)		(__kuid_val((x)->cred->uid) == (id) || __kgid_val((x)->cred->gid) == (id) || \
						__kuid_val((x)->cred->euid) == (id) || __kgid_val((x)->cred->egid) == (id) || \
						__kuid_val((x)->cred->suid) == (id) || __kgid_val((x)->cred->sgid) == (id))

#define CHECK_UID_BY_CRED(x, id)	(__kuid_val((x)->uid) == (id) || __kgid_val((x)->gid) == (id) || \
						__kuid_val((x)->euid) == (id) || __kgid_val((x)->egid) == (id) || \
						__kuid_val((x)->suid) == (id) || __kgid_val((x)->sgid) == (id))

#ifndef AID_SYSTEM
	#define AID_SYSTEM 1000
#endif

#ifndef AID_SHELL
	#define AID_SHELL 2000
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kern_access_ok RS_HIDE(_27)
#endif
noused notrace static int rs_kern_access_ok(int type, const void *addr, unsigned long size)
{
	int ret;
	mm_segment_t old_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
	ret = access_ok(type, (const void __user *)addr, size);
	set_fs(old_fs);
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_set_value RS_HIDE(_28)
#endif
noused notrace static noinline int rs_get_set_value(void)
{
	int ret;

	ret = (int)get_cycles();
#if 1
	if (!ret) {
		ret = (int)jiffies;

		if (!ret) {
			RS_GET_RANDOM_BYTES(&ret, sizeof(ret));

			if (!ret) {
				ret = 1;
			}
		}
	}
#else
	if (!ret) {
		RS_GET_RANDOM_BYTES(&ret, sizeof(ret));

		if (!ret) {
			ret = 1;
		}
	}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_positive_set_value RS_HIDE(_28_1)
#endif
static noinline int noused rs_get_positive_set_value(void)
{
	int ret = rs_get_set_value();
	if (ret < 0)
		ret = -ret;
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_negative_set_value RS_HIDE(_28_2)
#endif
static noinline int noused rs_get_negative_set_value(void)
{
	int ret = rs_get_set_value();
	if (ret > 0)
		ret = -ret;
	return ret;
}

enum {
	fcCheckFunc,

	fcIsOpOK,
	fcIsOpPermitted,
	fcIsOpBypassed,
	fcIsOpPermittedNoFC,
	fcIsOpBypassedNoFC,
	fcIsOpPermittedByFile,
	fcDoOpVerify,

#if defined(CONFIG_MOUNT_RESTRICT)
	fcIsMountDirNeedCheck,
	fcIsMountDevNeedCheck,
	fcIsMountNeedCheck,
	fcDoMountCheck,
	fcDoMount,
#endif

#if defined(CONFIG_INSMOD_RESTRICT)
	fcInsmodCheckPath,
	fcDoInsmodCheck,
#endif
#if defined(CONFIG_CHROOT_RESTRICT)
	fcDoChrootCheck,
#endif
#if defined(CONFIG_ACCESS_DEV_RESTRICT)
	fcDoAccessDevCheck,
#endif
#if defined(CONFIG_SECURITY_SELINUX) && defined(CONFIG_DISABLE_SELINUX_RESTRICT)
	fcDoDisableSelinuxCheck,
#endif
#if defined(CONFIG_PTRACE_RESTRICT)
	fcDoPtraceCheck,
#endif
#if defined(CONFIG_SETUID_RESTRICT)
	/*fcDoUidCheck,*/
#endif

#if defined(CONFIG_SECURITY_SELINUX) && defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
	fcDoLoadSelinuxPolicyCheck,
#endif

	fcInvalid,
};


#if defined(CONFIG_RS_CHECK_FUNC)

#define RS_FC_MAGIC (0x72536643)

#define RS_FC_STACK_BUDGET (512)

#ifndef task_stack_end_corrupted
#define task_stack_end_corrupted(task) \
				(*(end_of_stack(task)) != STACK_END_MAGIC)
#endif

#define RS_FC_BITMAP_MAX_BITS (fcInvalid + 1)
#define RS_FC_BITMAP_MAX_REFCOUNT (256)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_fc_bitmap RS_HIDE(_29)
#endif
noused notrace static int init_fc_bitmap(unsigned long **bitmap_buffer_ptr)
{
	unsigned long *end_ptr = end_of_stack(current);
	unsigned long *ptr = end_ptr;

	if (bitmap_buffer_ptr) {
		*bitmap_buffer_ptr = NULL;
	}

#if defined(CONFIG_STACK_GROWSUP)
	ptr -= 3;
#else
	ptr++;
#endif

	if ((!ptr[0]) && (!ptr[1]) && (!ptr[2])) {
	init_bitmap:
		if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			unsigned long *buffer = (unsigned long *)kzalloc(((RS_FC_BITMAP_MAX_BITS / BITS_PER_LONG) + 1) * sizeof(unsigned long), GFP_KERNEL);
			if (!buffer) {
				return -ENOMEM;
			}

			if (bitmap_buffer_ptr) {
				*bitmap_buffer_ptr = buffer;
			}

		#if defined(CONFIG_STACK_GROWSUP)
			ptr[1] = (uintptr_t)buffer;
		#else
			ptr[1] = (uintptr_t)buffer;
		#endif
		}

	#if defined(CONFIG_STACK_GROWSUP)
		ptr[0] = RS_FC_MAGIC;
		ptr[2]++;
	#else
		ptr[2] = RS_FC_MAGIC;
		ptr[0]++;
	#endif

		return 0;
	}
#if defined(CONFIG_STACK_GROWSUP)
	else if ((ptr[0] == RS_FC_MAGIC) && (ptr[2] < RS_FC_BITMAP_MAX_REFCOUNT))
#else
	else if ((ptr[2] == RS_FC_MAGIC) && (ptr[0] < RS_FC_BITMAP_MAX_REFCOUNT))
#endif
	{
	#if defined(CONFIG_STACK_GROWSUP)
		ptr[2]++;
	#else
		ptr[0]++;
	#endif

		if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			if (bitmap_buffer_ptr) {
				*bitmap_buffer_ptr = (unsigned long *)ptr[1];
			}
		}

		return 1;
	} else {
		if (*end_ptr == STACK_END_MAGIC) {
			register unsigned long current_stack_pointer asm ("sp");

		#if defined(CONFIG_STACK_GROWSUP)
			if ((current_stack_pointer + (RS_FC_STACK_BUDGET + 3)) < (unsigned long)end_ptr)
		#else
			if ((current_stack_pointer - (RS_FC_STACK_BUDGET + 3)) > (unsigned long)end_ptr)
		#endif
			{
				ptr[0] = 0;
				ptr[1] = 0;
				ptr[2] = 0;

				goto init_bitmap;
			}
		}

		return -EPERM;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define done_fc_bitmap RS_HIDE(_2a)
#endif
noused notrace static int done_fc_bitmap(unsigned long *bitmap_buffer)
{
	unsigned long *ptr = end_of_stack(current);

#if defined(CONFIG_STACK_GROWSUP)
	ptr -= 3;
#else
	ptr++;
#endif

#if defined(CONFIG_STACK_GROWSUP)
	if ((ptr[0] == RS_FC_MAGIC) && (ptr[2] < RS_FC_BITMAP_MAX_REFCOUNT))
#else
	if ((ptr[2] == RS_FC_MAGIC) && (ptr[0] < RS_FC_BITMAP_MAX_REFCOUNT))
#endif
	{
	#if defined(CONFIG_STACK_GROWSUP)
		if (!ptr[2])
	#else
		if (!ptr[0])
	#endif
		{
		#if defined(CONFIG_STACK_GROWSUP)
			ptr[0] = 0;
		#else
			ptr[2] = 0;
		#endif
			ptr[1] = 0;

			return 1;
		}

	#if defined(CONFIG_STACK_GROWSUP)
		if ((--ptr[2]) == 0)
	#else
		if ((--ptr[0]) == 0)
	#endif
		{
			if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			#if defined(CONFIG_STACK_GROWSUP)
				unsigned long *buffer = (unsigned long *)ptr[1];
			#else
				unsigned long *buffer = (unsigned long *)ptr[1];
			#endif
				if ((bitmap_buffer) && (bitmap_buffer != buffer)) {
					kfree(bitmap_buffer);
				}

				if (buffer) {
					kfree(buffer);
				}
			}

		#if defined(CONFIG_STACK_GROWSUP)
			ptr[0] = 0;
		#else
			ptr[2] = 0;
		#endif
			ptr[1] = 0;
		}

		return 0;
	} else {
		if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			if (bitmap_buffer) {
				kfree(bitmap_buffer);
			}
		}

		return -EPERM;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define test_bitmap RS_HIDE(_2b)
#endif
noused notrace static int test_bitmap(unsigned long *map, int start, int nr)
{
	unsigned long *p = map + BIT_WORD(start);
	const int size = start + nr;
	int bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
	unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);
	int set_cnt = 0;

	while (nr - bits_to_set >= 0) {
		set_cnt += ((*p & mask_to_set) ? 1 : 0);
		nr -= bits_to_set;
		bits_to_set = BITS_PER_LONG;
		mask_to_set = ~0UL;
		p++;
	}
	if (nr) {
		mask_to_set &= BITMAP_LAST_WORD_MASK(size);
		set_cnt += ((*p & mask_to_set) ? 1 : 0);
	}

	return set_cnt;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define test_and_set_bitmap RS_HIDE(_2c)
#endif
noused notrace static int test_and_set_bitmap(unsigned long *map, int start, int nr)
{
	unsigned long *p = map + BIT_WORD(start);
	const int size = start + nr;
	int bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
	unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);
	int set_cnt = 0;

	while (nr - bits_to_set >= 0) {
		if (*p & mask_to_set)
			set_cnt++;
		else
			*p |= mask_to_set;
		nr -= bits_to_set;
		bits_to_set = BITS_PER_LONG;
		mask_to_set = ~0UL;
		p++;
	}
	if (nr) {
		mask_to_set &= BITMAP_LAST_WORD_MASK(size);
		if (*p & mask_to_set)
			set_cnt++;
		else
			*p |= mask_to_set;
	}

	return set_cnt;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define test_fc_bitmap RS_HIDE(_2d)
#endif
noused notrace static int test_fc_bitmap(int func_enum)
{
	if (func_enum < RS_FC_BITMAP_MAX_BITS) {
		unsigned long *ptr = end_of_stack(current);

	#if defined(CONFIG_STACK_GROWSUP)
		ptr -= 3;
	#else
		ptr++;
	#endif

	#if defined(CONFIG_STACK_GROWSUP)
		if ((ptr[0] == RS_FC_MAGIC) && (ptr[2] < RS_FC_BITMAP_MAX_REFCOUNT))
	#else
		if ((ptr[2] == RS_FC_MAGIC) && (ptr[0] < RS_FC_BITMAP_MAX_REFCOUNT))
	#endif
		{
			if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			#if defined(CONFIG_STACK_GROWSUP)
				unsigned long *buffer = (unsigned long *)ptr[1];
			#else
				unsigned long *buffer = (unsigned long *)ptr[1];
			#endif
				if (buffer) {
					if (test_bitmap(buffer, func_enum, 1)) {
						return 1;
					} else {
						return 0;
					}
				}
			} else {
			#if defined(CONFIG_STACK_GROWSUP)
				if (test_bit(func_enum, &ptr[1])) /*(test_and_set_bitmap(&ptr[1], func_enum, 1))*/
			#else
				if (test_bit(func_enum, &ptr[1])) /*(test_and_set_bitmap(&ptr[1], func_enum, 1))*/
			#endif
				{
					return 1;
				} else {
					return 0;
				}
			}
		}
	}

	return -EINVAL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_fc_bitmap RS_HIDE(_2e)
#endif
noused notrace static int try_set_fc_bitmap(int func_enum)
{
	if (func_enum < RS_FC_BITMAP_MAX_BITS) {
		unsigned long *ptr = end_of_stack(current);

	#if defined(CONFIG_STACK_GROWSUP)
		ptr -= 3;
	#else
		ptr++;
	#endif

	#if defined(CONFIG_STACK_GROWSUP)
		if ((ptr[0] == RS_FC_MAGIC) && (ptr[2] < RS_FC_BITMAP_MAX_REFCOUNT))
	#else
		if ((ptr[2] == RS_FC_MAGIC) && (ptr[0] < RS_FC_BITMAP_MAX_REFCOUNT))
	#endif
		{
			if (RS_FC_BITMAP_MAX_BITS > BITS_PER_LONG) {
			#if defined(CONFIG_STACK_GROWSUP)
				unsigned long *buffer = (unsigned long *)ptr[1];
			#else
				unsigned long *buffer = (unsigned long *)ptr[1];
			#endif
				if (buffer) {
					if (test_and_set_bitmap(buffer, func_enum, 1)) {
						return 1;
					} else {
						return 0;
					}
				}
			} else {
			#if defined(CONFIG_STACK_GROWSUP)
				if (test_and_set_bit(func_enum, &ptr[1])) /*(test_and_set_bitmap(&ptr[1], func_enum, 1))*/
			#else
				if (test_and_set_bit(func_enum, &ptr[1])) /*(test_and_set_bitmap(&ptr[1], func_enum, 1))*/
			#endif
				{
					return 1;
				} else {
					return 0;
				}
			}
		}
	}

	return -EINVAL;
}
#else
#define init_fc_bitmap(...) do {} while (0)
#define done_fc_bitmap(...) do {} while (0)
#define test_fc_bitmap(...) (0)
#define try_set_fc_bitmap(...) do {} while (0)
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_in_call_from_user RS_HIDE(_9e_1)
#endif
static noinline int noused task_in_call_from_user(struct task_struct *task)
{
	int ret;
	noused unsigned long sp, pc, task_size;
	noused long callno;
	noused struct pt_regs *regs;

	if (unlikely(!task->active_mm)) {
		RS_LOG("icfu,1\n");
		ret = 1;
		goto out;
	}

	regs = task_pt_regs(task);
	if (unlikely(!regs)) { /*will not happen on arm/arm64*/
		RS_LOG("icfu,2\n");
		ret = 0;
		goto out;
	}

	callno = syscall_get_nr(task, regs);
	if ((callno != -1L)
		&& (callno != /*__NR_restart_syscall*/0)
		) {
		RS_LOG("icfu,3,%ld\n", callno);
		ret = 1;
		goto out;
	}

	sp = user_stack_pointer(regs);
	pc = instruction_pointer(regs);

	task_size = TASK_SIZE;

	if ((sp) && (sp < task_size)) {
		RS_LOG("icfu,4\n");
		ret = 1;
		goto out;
	}

	if ((pc) && (pc < task_size)) {
		RS_LOG("icfu,5\n");
		ret = 1;
		goto out;
	}

	if (user_mode(regs)) {
		RS_LOG("icfu,6\n");
		ret = 1;
		goto out;
	}

	ret = 0;

out:
	return ret;
}

enum {
	gfBootupDone,
	gfIsEngByProp,
	gfADBRootOK,
	gfIsBadBoot,
	gfFuncChecksumsOK,
#if defined(CONFIG_EXEC_SU_RESTRICT)
	gfAllowSuExec,
#endif
#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
	gfSelinuxPolicyLoaded,
#if defined(RS_VIVOROOT_SUPPORT)
	gfVivoRootTriggered,
#endif
#endif

	gfInvalid,
};


#define NBITS2(n) (((n) & 2) ? 1 : 0)
#define NBITS4(n) (((n) & (0xC)) ? (2 + NBITS2((n) >> 2)) : (NBITS2(n)))
#define NBITS8(n) (((n) & 0xF0) ? (4 + NBITS4((n) >> 4)):(NBITS4(n)))
#define NBITS16(n) (((n) & 0xFF00) ? (8 + NBITS8((n) >> 8)):(NBITS8(n)))
#define NBITS32(n) (((n) & 0xFFFF0000) ? (16 + NBITS16((n) >> 16)) : (NBITS16(n)))
#define NBITS(n) (n == 0 ? 0 : NBITS32((n)) + 1)

#if !defined(RS_64BIT_SUPPORT)
#define LOG2(n)   \
	((n) < (1UL << 1) ? 0 : \
	(n) < (1UL << 2) ? 1 : \
	(n) < (1UL << 3) ? 2 : \
	(n) < (1UL << 4) ? 3 : \
	(n) < (1UL << 5) ? 4 : \
	(n) < (1UL << 6) ? 5 : \
	(n) < (1UL << 7) ? 6 : \
	(n) < (1UL << 8) ? 7 : \
	(n) < (1UL << 9) ? 8 : \
	(n) < (1UL << 10) ? 9 : \
	(n) < (1UL << 11) ? 10 : \
	(n) < (1UL << 12) ? 11 : \
	(n) < (1UL << 13) ? 12 : \
	(n) < (1UL << 14) ? 13 : \
	(n) < (1UL << 15) ? 14 : \
	(n) < (1UL << 16) ? 15 : \
	(n) < (1UL << 17) ? 16 : \
	(n) < (1UL << 18) ? 17 : \
	(n) < (1UL << 19) ? 18 : \
	(n) < (1UL << 20) ? 19 : \
	(n) < (1UL << 21) ? 20 : \
	(n) < (1UL << 22) ? 21 : \
	(n) < (1UL << 23) ? 22 : \
	(n) < (1UL << 24) ? 23 : \
	(n) < (1UL << 25) ? 24 : \
	(n) < (1UL << 26) ? 25 : \
	(n) < (1UL << 27) ? 26 : \
	(n) < (1UL << 28) ? 27 : \
	(n) < (1UL << 29) ? 28 : \
	(n) < (1UL << 30) ? 29 : \
	(n) < (1UL << 31) ? 30 : \
	31)
#else
#define LOG2(n)   \
	((n) < (1UL << 1) ? 0 : \
	(n) < (1UL << 2) ? 1 : \
	(n) < (1UL << 3) ? 2 : \
	(n) < (1UL << 4) ? 3 : \
	(n) < (1UL << 5) ? 4 : \
	(n) < (1UL << 6) ? 5 : \
	(n) < (1UL << 7) ? 6 : \
	(n) < (1UL << 8) ? 7 : \
	(n) < (1UL << 9) ? 8 : \
	(n) < (1UL << 10) ? 9 : \
	(n) < (1UL << 11) ? 10 : \
	(n) < (1UL << 12) ? 11 : \
	(n) < (1UL << 13) ? 12 : \
	(n) < (1UL << 14) ? 13 : \
	(n) < (1UL << 15) ? 14 : \
	(n) < (1UL << 16) ? 15 : \
	(n) < (1UL << 17) ? 16 : \
	(n) < (1UL << 18) ? 17 : \
	(n) < (1UL << 19) ? 18 : \
	(n) < (1UL << 20) ? 19 : \
	(n) < (1UL << 21) ? 20 : \
	(n) < (1UL << 22) ? 21 : \
	(n) < (1UL << 23) ? 22 : \
	(n) < (1UL << 24) ? 23 : \
	(n) < (1UL << 25) ? 24 : \
	(n) < (1UL << 26) ? 25 : \
	(n) < (1UL << 27) ? 26 : \
	(n) < (1UL << 28) ? 27 : \
	(n) < (1UL << 29) ? 28 : \
	(n) < (1UL << 30) ? 29 : \
	(n) < (1UL << 31) ? 30 : \
	(n) < (1UL << 32) ? 31 : \
	(n) < (1UL << 33) ? 32 : \
	(n) < (1UL << 34) ? 33 : \
	(n) < (1UL << 35) ? 34 : \
	(n) < (1UL << 36) ? 35 : \
	(n) < (1UL << 37) ? 36 : \
	(n) < (1UL << 38) ? 37 : \
	(n) < (1UL << 39) ? 38 : \
	(n) < (1UL << 40) ? 39 : \
	(n) < (1UL << 41) ? 40 : \
	(n) < (1UL << 42) ? 41 : \
	(n) < (1UL << 43) ? 42 : \
	(n) < (1UL << 44) ? 43 : \
	(n) < (1UL << 45) ? 44 : \
	(n) < (1UL << 46) ? 45 : \
	(n) < (1UL << 47) ? 46 : \
	(n) < (1UL << 48) ? 47 : \
	(n) < (1UL << 49) ? 48 : \
	(n) < (1UL << 50) ? 49 : \
	(n) < (1UL << 51) ? 50 : \
	(n) < (1UL << 52) ? 51 : \
	(n) < (1UL << 53) ? 52 : \
	(n) < (1UL << 54) ? 53 : \
	(n) < (1UL << 55) ? 54 : \
	(n) < (1UL << 56) ? 55 : \
	(n) < (1UL << 57) ? 56 : \
	(n) < (1UL << 58) ? 57 : \
	(n) < (1UL << 59) ? 58 : \
	(n) < (1UL << 60) ? 59 : \
	(n) < (1UL << 61) ? 60 : \
	(n) < (1UL << 62) ? 61 : \
	(n) < (1UL << 63) ? 62 : \
	63)
#endif


#if defined(CONFIG_MOUNT_FILES_CHECK)
	#define RS_HAS_FGF_SUPPRESS_DEL_FILES (1)
#else
	#define RS_HAS_FGF_SUPPRESS_DEL_FILES (0)
#endif
#if defined(CONFIG_SOCK_RESTRICT) && defined(SOCK_CHECK_FASTER_BY_GLOBAL_VAR)
	#define RS_HAS_FGF_ALLOW_SOCK_BIND (1)
	#define RS_HAS_FGF_ALLOW_SOCK_CONNECT (1)
#else
	#define RS_HAS_FGF_ALLOW_SOCK_BIND (0)
	#define RS_HAS_FGF_ALLOW_SOCK_CONNECT (0)
#endif
#if defined(CONFIG_VIVO_ALLOW_REBOOT_REMOUNT) && defined(RS_ALLOW_REMOUNT_IN_FAST_FLAG)
	#define RS_HAS_FGF_ALLOW_REMOUNT (1)
#else
	#define RS_HAS_FGF_ALLOW_REMOUNT (0)
#endif
/*add new RS_HAS_FGF_XXX item here*/

#define RS_HAS_FGF_DUMMY (1)


/*add new "+ RS_HAS_FGF_XXX item \" below and before "+ RS_HAS_FGF_DUMMY \"*/
#define RS_FAST_FLAG_USED_COUNT ( \
	RS_HAS_FGF_SUPPRESS_DEL_FILES \
	+ RS_HAS_FGF_ALLOW_SOCK_BIND + RS_HAS_FGF_ALLOW_SOCK_CONNECT \
	+ RS_HAS_FGF_ALLOW_REMOUNT \
	+ RS_HAS_FGF_DUMMY \
	)

#if (RS_FAST_FLAG_USED_COUNT < 2)
	/*only one dummy item*/
	#undef RS_FAST_FLAG_SUPPORT
#endif

#define RS_FAST_FLAG_SHIFT (LOG2(RS_FAST_FLAG_USED_COUNT))
#define RS_FAST_FLAG_COUNT (1 << RS_FAST_FLAG_SHIFT)
#define RS_FAST_FLAG_MASK ((1 << RS_FAST_FLAG_SHIFT) - 1)

#if defined(RS_FAST_FLAG_SUPPORT)

enum {
#if defined(CONFIG_MOUNT_FILES_CHECK)
	fgfSuppressDelFiles,
#endif
#if defined(CONFIG_SOCK_RESTRICT) && defined(SOCK_CHECK_FASTER_BY_GLOBAL_VAR)
	fgfAllowSockBind,
	fgfAllowSockConnect,
#endif
#if defined(CONFIG_VIVO_ALLOW_REBOOT_REMOUNT) && defined(RS_ALLOW_REMOUNT_IN_FAST_FLAG)
	fgfAllowRemount,
#endif
	/*add new flag before fgfDummy*/

	fgfDummy,
	fgfInvalid = (1 << RS_FAST_FLAG_SHIFT), /*为了取模快速，应该为尽量小的 2的n次幂*/
};


#ifndef __intptr_t_align
	#define __intptr_t_align	__attribute__((__aligned__(sizeof(intptr_t))))
#endif

typedef union cast_fast_flag_crc_struct {
	uintptr_t val;
	u32 crcs[sizeof(uintptr_t) / sizeof(u32)];
} cast_fast_flag_crc_t;

typedef struct tag_rs_fast_flags {
	intptr_t dummy0[((LINUX_VERSION_CODE + __COUNTER__) & 3) + 1];
	intptr_t flags_1[RS_FAST_FLAG_COUNT];
	intptr_t dummy1[(((LINUX_VERSION_CODE >> 16) + __COUNTER__) & 3) + 1];
	intptr_t indexs[RS_FAST_FLAG_COUNT + 3];
	intptr_t dummy2[((((LINUX_VERSION_CODE & 0xFF00) >> 8) + __COUNTER__) & 3) + 1];
	intptr_t flags_2[RS_FAST_FLAG_COUNT];
#if defined(RS_FAST_FLAG_CHECK_CRC)
	intptr_t dummy3[(((LINUX_VERSION_CODE & 0xF) + __COUNTER__) & 3) + 1];
	u32 crcs[RS_FAST_FLAG_COUNT];
#endif
	intptr_t dummy4 __intptr_t_align;
} rs_fast_flags;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_fast_flags RS_HIDE(_2f_0)
#endif
noused static rs_fast_flags g_rs_fast_flags __intptr_t_align
#if defined(RS_FAST_FLAG_WRITE_PROTECT)
	__rs_text
#endif
	;


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_fast_vars_lock RS_HIDE(_2f_1)
#endif
noused static DEFINE_RWLOCK(g_rs_fast_vars_lock);

#define RS_FAST_FLAGS_LOCK_INIT do {} while (0)

#define RS_FAST_FLAGS_READ_LOCK read_lock(&g_rs_fast_vars_lock)
#define RS_FAST_FLAGS_READ_UNLOCK read_unlock(&g_rs_fast_vars_lock)

#define RS_FAST_FLAGS_WRITE_LOCK write_lock(&g_rs_fast_vars_lock)
#define RS_FAST_FLAGS_WRITE_UNLOCK write_unlock(&g_rs_fast_vars_lock)


#if defined(RS_FAST_FLAG_WRITE_PROTECT)
/*foward declaration*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars RS_HIDE(_l2)
#endif
noused notrace static int rs_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars_rw RS_HIDE(_l2_1)
#endif
noused notrace static int rs_set_text_vars_rw(void *target, size_t unit_size, size_t unit_count);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_text_vars_ro RS_HIDE(_l2_2)
#endif
noused notrace static int rs_set_text_vars_ro(void *target, size_t unit_size, size_t unit_count);

#endif /*RS_FAST_FLAG_WRITE_PROTECT*/


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_fast_flags_inited RS_HIDE(_2f_2)
#endif
noused static __initdata int g_rs_fast_flags_inited;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_fast_flags RS_HIDE(_2f_10)
#endif
noused notrace static int __init rs_init_fast_flags(void)
{
	rs_fast_flags *fast_flags_ptr;
	int ret = 0;

	/*RS_FAST_FLAGS_LOCK_INIT;*/

#if defined(RS_FAST_FLAG_WRITE_PROTECT)
	int inited;

	RS_FAST_FLAGS_READ_LOCK;
	inited = g_rs_fast_flags_inited;
	RS_FAST_FLAGS_READ_UNLOCK;

	if (inited) {
		RS_LOG("riff,0\n");
		return 0;
	}

	fast_flags_ptr = (rs_fast_flags *)kmalloc(sizeof(*fast_flags_ptr), GFP_KERNEL);
	if (!fast_flags_ptr) {
		RS_LOG("riff,1\n");
		return 0;
	}
#else /*RS_FAST_FLAG_WRITE_PROTECT*/
	fast_flags_ptr = &g_rs_fast_flags;

	RS_FAST_FLAGS_WRITE_LOCK;

	if (!g_rs_fast_flags_inited)
#endif /*!RS_FAST_FLAG_WRITE_PROTECT*/
	{
		noused size_t index;
		noused intptr_t save_idx;
		noused intptr_t base_idx;
		noused intptr_t mask;
		noused intptr_t val;

		RS_GET_RANDOM_BYTES(fast_flags_ptr, sizeof(*fast_flags_ptr));

		save_idx = (fast_flags_ptr->indexs[0] ^ fast_flags_ptr->indexs[fgfInvalid + 1]);
		mask = (fast_flags_ptr->indexs[0] ^ fast_flags_ptr->indexs[fgfInvalid + 2]);
		for (index = 0; index < fgfDummy; index++) {
			base_idx = save_idx + index;

			base_idx &= RS_FAST_FLAG_MASK;

			val = (0 ^ fast_flags_ptr->indexs[base_idx + 1]);

			val ^= save_idx;
			fast_flags_ptr->flags_1[base_idx] = (fast_flags_ptr->flags_1[base_idx] & mask)
				| (val & (~mask));
			fast_flags_ptr->flags_2[base_idx] = (fast_flags_ptr->flags_2[base_idx] & (~mask))
				| (val & mask);
		#if defined(RS_FAST_FLAG_CHECK_CRC)
			{
				u32 first_crc = crc32_le(0, (unsigned char const *)&save_idx, sizeof(save_idx));
				fast_flags_ptr->crcs[base_idx] = crc32_le(first_crc, (unsigned char const *)&val, sizeof(val));
			}
		#endif /*RS_FAST_FLAG_CHECK_CRC*/
		}

	#if !defined(RS_FAST_FLAG_WRITE_PROTECT)
		g_rs_fast_flags_inited = 1;
	#endif
	}

#if defined(RS_FAST_FLAG_WRITE_PROTECT)
	RS_FAST_FLAGS_WRITE_LOCK;

	if (!g_rs_fast_flags_inited) {
		if (!rs_set_text_vars_rw(&g_rs_fast_flags, sizeof(uintptr_t), (sizeof(g_rs_fast_flags) / sizeof(uintptr_t)))) {
			RS_LOG("riff,3\n");
			ret = rs_set_text_vars(&g_rs_fast_flags, fast_flags_ptr, sizeof(uintptr_t),
				(sizeof(g_rs_fast_flags) / sizeof(uintptr_t)));
			rs_set_text_vars_ro(&g_rs_fast_flags, sizeof(uintptr_t), (sizeof(g_rs_fast_flags) / sizeof(uintptr_t)));

			if (!ret) {
				g_rs_fast_flags_inited = 1;
			} else {
				g_rs_fast_flags_inited = -1;
			}
		} else {
			RS_LOG("riff,4\n");
			g_rs_fast_flags_inited = -1;
		}
	}

	RS_FAST_FLAGS_WRITE_UNLOCK;

	kfree(fast_flags_ptr);
#else /*RS_FAST_FLAG_WRITE_PROTECT*/

	RS_FAST_FLAGS_WRITE_UNLOCK;
#endif /*!RS_FAST_FLAG_WRITE_PROTECT*/

	return ret;
}

device_initcall(rs_init_fast_flags);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_fast_flag RS_HIDE(_2f_11)
#endif
noused notrace static int rs_get_fast_flag(int index, intptr_t *flag_ptr)
{
	int ret;
	noused intptr_t val;
	noused intptr_t save_idx;
	noused intptr_t base_idx;
	noused intptr_t mask;

	if (!flag_ptr) {
		/*RS_LOG("rgff,1\n");*/
		ret = -EINVAL;
		goto err_ret_no_ptr;
	}

	if ((index < 0) || (index >= fgfDummy)) {
		/*RS_LOG("rgff,2\n");*/
		ret = -EINVAL;
		goto err_ret;
	}

	RS_FAST_FLAGS_READ_LOCK;

	mask = (g_rs_fast_flags.indexs[0] ^ g_rs_fast_flags.indexs[fgfInvalid + 2]);
	save_idx = (g_rs_fast_flags.indexs[0] ^ g_rs_fast_flags.indexs[fgfInvalid + 1]);
	base_idx = save_idx + index;

	base_idx &= RS_FAST_FLAG_MASK;

	val = (g_rs_fast_flags.flags_1[base_idx] & (~mask))
		| (g_rs_fast_flags.flags_2[base_idx] & mask);
#if defined(RS_FAST_FLAG_CHECK_CRC)
	{
		u32 first_crc = crc32_le(0, (unsigned char const *)&save_idx, sizeof(save_idx));
		if (g_rs_fast_flags.crcs[base_idx] != crc32_le(first_crc, (unsigned char const *)&val, sizeof(val))) {
			RS_FAST_FLAGS_READ_UNLOCK;
			/*RS_LOG("rgff,3\n");*/
			ret = -ENXIO;
			goto err_ret;
		}
	}
#endif /*RS_FAST_FLAG_CHECK_CRC*/

	val ^= save_idx;

	val ^= g_rs_fast_flags.indexs[base_idx + 1];

	RS_FAST_FLAGS_READ_UNLOCK;

	*flag_ptr = val;

	/*RS_LOG("rgff,e\n");*/
	ret = 0;
	return ret;
err_ret:
	*flag_ptr = 0;
err_ret_no_ptr:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_fast_flag RS_HIDE(_2f_12)
#endif
noused notrace static int rs_set_fast_flag(int index, intptr_t flag)
{
	noused intptr_t val;
	noused intptr_t save_idx;
	noused intptr_t base_idx;
	noused intptr_t mask;
	int ret;

	if ((index < 0) || (index >= fgfDummy)) {
		/*RS_LOG("rsff,1\n");*/
		ret = -EINVAL;
		goto out;
	}

	RS_FAST_FLAGS_WRITE_LOCK;

	mask = (g_rs_fast_flags.indexs[0] ^ g_rs_fast_flags.indexs[fgfInvalid + 2]);
	save_idx = (g_rs_fast_flags.indexs[0] ^ g_rs_fast_flags.indexs[fgfInvalid + 1]);
	base_idx = save_idx + index;

	base_idx &= RS_FAST_FLAG_MASK;

	val = (flag ^ g_rs_fast_flags.indexs[base_idx + 1]);

	val ^= save_idx;

#if defined(RS_FAST_FLAG_WRITE_PROTECT)
	if (rs_set_text_vars_rw(&g_rs_fast_flags, sizeof(uintptr_t), (sizeof(g_rs_fast_flags) / sizeof(uintptr_t)))) {
		ret = -EACCES;
		/*RS_LOG("rsff,2\n");*/
		goto out_with_lock;
	} else {
		intptr_t val1, val2;
		val1 = (g_rs_fast_flags.flags_1[base_idx] & mask) | (val & (~mask));
		val2 = (g_rs_fast_flags.flags_2[base_idx] & (~mask)) | (val & mask);

		ret = rs_set_text_vars(&g_rs_fast_flags.flags_1[base_idx], &val1, sizeof(uintptr_t), (sizeof(val1) / sizeof(uintptr_t)));
		if (ret)
			goto out_with_lock;
		ret = rs_set_text_vars(&g_rs_fast_flags.flags_2[base_idx], &val2, sizeof(uintptr_t), ((sizeof(val2) / sizeof(uintptr_t)));
		if (ret)
			goto out_with_lock;
	}
#else /*RS_FAST_FLAG_WRITE_PROTECT*/
	ret = 0;
	g_rs_fast_flags.flags_1[base_idx] = (g_rs_fast_flags.flags_1[base_idx] & mask)
		| (val & (~mask));
	g_rs_fast_flags.flags_2[base_idx] = (g_rs_fast_flags.flags_2[base_idx] & (~mask))
		| (val & mask);
#endif /*!RS_FAST_FLAG_WRITE_PROTECT*/

#if defined(RS_FAST_FLAG_CHECK_CRC)
	{
		u32 first_crc = crc32_le(0, (unsigned char const *)&save_idx, sizeof(save_idx));
		u32 crc = crc32_le(first_crc, (unsigned char const *)&val, sizeof(val));
	#if defined(RS_FAST_FLAG_WRITE_PROTECT)
		{
		#if defined(RS_64BIT_SUPPORT)
			cast_fast_flag_crc_t cast_crc;
			char *ptr = (char *)&g_rs_fast_flags.crcs[base_idx];
			char *align_ptr = PTR_ALIGN(ptr, sizeof(uintptr_t));
			cast_crc.val = *((uintptr_t *)align_ptr);
			cast_crc.crcs[(ptr - align_ptr) / sizeof(u32)] = crc;

			ret = rs_set_text_vars(align_ptr, &cast_crc.val, sizeof(uintptr_t),
				(sizeof(cast_crc.val) / sizeof(uintptr_t)));
		#else /*RS_64BIT_SUPPORT*/
			ret = rs_set_text_vars((&g_rs_fast_flags.crcs[base_idx], &crc,
				sizeof(uintptr_t), (sizeof(crc) / sizeof(uintptr_t)));
		#endif /*!RS_64BIT_SUPPORT*/
			if (ret)
				goto out_with_lock;
		}
	#else /*RS_FAST_FLAG_WRITE_PROTECT*/
		g_rs_fast_flags.crcs[base_idx] = crc;
	#endif /*!RS_FAST_FLAG_WRITE_PROTECT*/
	}
#endif /*RS_FAST_FLAG_CHECK_CRC*/

#if defined(RS_FAST_FLAG_WRITE_PROTECT)
	rs_set_text_vars_ro(&g_rs_fast_flags, sizeof(uintptr_t), (sizeof(g_rs_fast_flags) / sizeof(uintptr_t)));
#endif /*RS_FAST_FLAG_WRITE_PROTECT*/

out_with_lock:
	RS_FAST_FLAGS_WRITE_UNLOCK;
out:
	/*RS_LOG("rsff,e\n");*/
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_fast_flag_early RS_HIDE(_2f_13)
#endif
noused notrace static int __init rs_get_fast_flag_early(int index, intptr_t *flag_ptr)
{
	rs_init_fast_flags();

	return rs_get_fast_flag(index, flag_ptr);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_fast_flag_early RS_HIDE(_2f_14)
#endif
noused notrace static int __init rs_set_fast_flag_early(int index, intptr_t flag)
{
	rs_init_fast_flags();

	return rs_set_fast_flag(index, flag);
}

#endif /*RS_FAST_FLAG_SUPPORT*/

/*/////////////////////////////////////////////////////////////*/

enum {
	fcbdfNotInited,
	fcbdfPermitted,
	fcbdfByPassed,
	fcbdfInited,
};

#if defined(CONFIG_RS_CHECK_FUNC)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_verify_func RS_HIDE(_42_)
#endif

noused notrace static int rs_verify_func(RS_FC_SYM_VAL_TYPE * func_end, int func_enum);
#define rs_verify_func_null(...) (0)
#else
	#define rs_verify_func(...) (0)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_valid_vars_page RS_HIDE(_l5)
#endif
noused notrace static bool rs_is_valid_vars_page(void *page, bool first_alloc);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_ok RS_HIDE(_lc)
#endif
noused notrace static int is_op_ok(unsigned var_flag_idx, int op_type);
RS_FORWARD_FC_BEGIN(is_op_ok);
RS_FORWARD_FC_END(is_op_ok);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_permitted_by_file RS_HIDE(_la_d)
#endif
noused notrace static int is_op_permitted_by_file(int op_type);
RS_FORWARD_FC_BEGIN(is_op_permitted_by_file);
RS_FORWARD_FC_END(is_op_permitted_by_file);

#if defined(CONFIG_CONFIG_RS_PERMIT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_permitted RS_HIDE(_30)
#endif

RS_DEFINE_FC_BEGIN(is_op_permitted);

noused notrace RS_DEFINE_FC_BODY(is_op_permitted) static noinline int is_op_permitted(int op_type)
{
	int ret;

	if ((op_type >= 0) && (op_type < ovInvalid)
		&& (is_op_ok(opvPermit, op_type))
		&& (!rs_verify_func(RS_FC_PARAMS(is_op_ok), fcIsOpOK))
		) {
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

RS_DEFINE_FC_END(is_op_permitted);

#else
	#define is_op_permitted(op_type)	(0)
#endif


#if defined(CONFIG_CONFIG_RS_BYPASS)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_bypassed RS_HIDE(_31)
#endif

RS_DEFINE_FC_BEGIN(is_op_bypassed);

noused notrace RS_DEFINE_FC_BODY(is_op_bypassed) static noinline int is_op_bypassed(int op_type)
{
	int ret;

	if ((op_type >= 0) && (op_type < ovInvalid)
		&& (((is_op_ok(opvBypass, op_type))
		&& (!rs_verify_func(RS_FC_PARAMS(is_op_ok), fcIsOpOK)))
		|| ((is_op_permitted_by_file(op_type))
		&& (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_by_file), fcIsOpPermittedByFile))))
		) {
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

RS_DEFINE_FC_END(is_op_bypassed);

#else
	#define is_op_bypassed(op_type)	(0)
#endif

#if defined(CONFIG_RS_CHECK_FUNC)

#if defined(CONFIG_CONFIG_RS_PERMIT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_permitted_no_fc RS_HIDE(_30_)
#endif

RS_DEFINE_FC_BEGIN(is_op_permitted_no_fc);

noused notrace RS_DEFINE_FC_BODY(is_op_permitted_no_fc) static int is_op_permitted_no_fc(int op_type)
{
	int ret;

	if ((op_type >= 0) && (op_type < ovInvalid)
		&& (is_op_ok(opvPermit, op_type))
		) {
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

RS_DEFINE_FC_END(is_op_permitted_no_fc);

#else
	#define is_op_permitted_no_fc(op_type)	(0)
#endif


#if defined(CONFIG_CONFIG_RS_BYPASS)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_bypassed_no_fc RS_HIDE(_31_)
#endif

RS_DEFINE_FC_BEGIN(is_op_bypassed_no_fc);

noused notrace RS_DEFINE_FC_BODY(is_op_bypassed_no_fc) static int is_op_bypassed_no_fc(int op_type)
{
	int ret;

	if ((op_type >= 0) && (op_type < ovInvalid)
		&& (is_op_ok(opvBypass, op_type)
		|| is_op_permitted_by_file(op_type))
		) {
		ret = 1;
	} else {
		ret = 0;
	}

	return ret;
}

RS_DEFINE_FC_END(is_op_bypassed_no_fc);

#else
	#define is_op_bypassed_no_fc(op_type)	(0)
#endif

#else
	#define is_op_permitted_no_fc	is_op_permitted
	#define is_op_bypassed_no_fc	is_op_bypassed
#endif

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
#define rs_is_module_address RS_HIDE(_33_0)
#endif
noused notrace static bool rs_is_module_address(unsigned long addr)
{
	/* only applicable for real module related code sections */
	/*return is_module_address(addr);*/
	/* only check for [MODULES_VADDR,MODULES_END) or [VMALLOC_START,VMALLOC_END) */
	/*return is_vmalloc_or_module_addr((const void *)addr);*/
	struct page *page = vmalloc_to_page((void *)addr);
	return (page != NULL);
}


#define RS_CHECKSUM_TYPE uintptr_t

#if defined(CONFIG_RS_CHECK_FUNC)

typedef RS_CHECKSUM_TYPE (*rs_crc_8slice_table_type)[8][256];
typedef RS_CHECKSUM_TYPE (*rs_crc_16slice_table_type)[16][256];

#if defined(CONFIG_ARM64)
	#define RS_CRC_TABLE_SIZE (8 * 256 * sizeof(RS_CHECKSUM_TYPE))
#else
	#define RS_CRC_TABLE_SIZE (16 * 256 * sizeof(RS_CHECKSUM_TYPE))
#endif

#else
#define RS_CRC_TABLE_SIZE (0)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_valid_vars_page_slim RS_HIDE(_34_)
#endif
noused notrace static bool rs_is_valid_vars_page_slim(void *page)
{
	if ((!((uintptr_t)page & ((1 << PAGE_SHIFT) - 1)))
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		&& (((uintptr_t)page >= module_alloc_base) && ((uintptr_t)page <= ((module_alloc_base + MODULES_VSIZE - (PAGE_SIZE + RS_CRC_TABLE_SIZE)))))
	#elif defined(CONFIG_MODULES)
		&& (((uintptr_t)page >= MODULES_VADDR) && ((uintptr_t)page <= (MODULES_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#elif defined(CONFIG_MMU)
		&& (((uintptr_t)page >= VMALLOC_START) && ((uintptr_t)page <= (VMALLOC_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#elif defined(MODULES_VADDR)
		&& (((uintptr_t)page >= MODULES_VADDR) && ((uintptr_t)page <= (MODULES_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#endif
		) {
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
		struct vm_struct *area = find_vm_area(page);
		if ((area) && ((area->flags & VM_USERMAP) == 0))
	#endif
		{
			return true;
		}
	}

	return false;
}

#if defined(RS_HIDE_OP_VAR_FLAG)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_store_op_var_flag RS_HIDE(_35_)
#endif
noused notrace static void rs_store_op_var_flag(RS_OP_VAR_FLAG_TYPE flag)
{
	/*若第一个元素的0xff值为0，则认为未初始化*/

	unsigned int flags_idx, nibble_idx, val_nibble_idx;
	unsigned int idx_val, idx_mask, idx1, idx2, val1, val2, val1_idx, val2_idx, shift1, shift2;
	RS_OP_VAR_FLAG_TYPE *var_flags = g_rs_op_var_flags;

	RS_GET_RANDOM_BYTES(var_flags, sizeof(g_rs_op_var_flags));

	idx_mask = 0x55;
	flags_idx = 0;
	nibble_idx = 0;
	val_nibble_idx = 0;

	while (val_nibble_idx < 8) {
		idx_val = (var_flags[flags_idx] >> (nibble_idx * 4)) & 0xff;

		/*idx1 = (var_flags[flags_idx] >> (nibble_idx * 4)) & 0xf;*/
		/*idx2 = (var_flags[flags_idx] >> ((nibble_idx + 1) * 4)) & 0xf;*/
		idx1 = idx_val & 0xf;
		idx2 = (idx_val >> 4);

		idx_val ^= idx_mask;
		idx_val <<= (nibble_idx * 4);
		var_flags[flags_idx] &= ~(0xff << (nibble_idx * 4));
		var_flags[flags_idx] |= idx_val;

		idx_mask = (idx_mask & 0xf0) | (~idx_mask & 0xf);
		if (flags_idx/*(val_nibble_idx >> 1)*/ & 1)
			idx_mask = (~idx_mask & 0xf0) | (idx_mask & 0xf);

		/*剩余24bit*/
		val1 = (flag >> (val_nibble_idx * 4)) & 0xf;
		val2 = (flag >> ((val_nibble_idx + 1) * 4)) & 0xf;
		val_nibble_idx += 2;

		val1_idx = idx1 % 6;
		val2_idx = idx2 % 6;
		if (val1_idx == val2_idx)
			val2_idx = (val1_idx + 1) % 6;

		shift1 = (((val1_idx + (nibble_idx + 2)) % 8) * 4);
		shift2 = (((val2_idx + (nibble_idx + 2)) % 8) * 4);

		var_flags[flags_idx] &= ~((0xf << shift1) | (0xf << shift2));
		var_flags[flags_idx] |= (((val1 ^ val2_idx) << shift1) | ((val2 ^ val1_idx) << shift2));

		/*下一个元素*/
		flags_idx++;
		nibble_idx = (idx1 + idx2) % 7;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_fetch_op_var_flag RS_HIDE(_36_)
#endif
noused notrace static void rs_fetch_op_var_flag(RS_OP_VAR_FLAG_TYPE *flag_ptr)
{
	RS_OP_VAR_FLAG_TYPE flag = 0;
	unsigned int flags_idx, nibble_idx, val_nibble_idx;
	unsigned int idx_val, idx_mask, idx1, idx2, val1, val2, val1_idx, val2_idx, shift1, shift2;
	RS_OP_VAR_FLAG_TYPE *var_flags = g_rs_op_var_flags;

	idx_mask = 0x55;
	flags_idx = 0;
	nibble_idx = 0;
	val_nibble_idx = 0;

	while (val_nibble_idx < 8) {
		idx_val = ((var_flags[flags_idx] >> (nibble_idx * 4)) & 0xff) ^ idx_mask;

		idx_mask = (idx_mask & 0xf0) | (~idx_mask & 0xf);
		if (flags_idx/*(val_nibble_idx >> 1)*/ & 1)
			idx_mask = (~idx_mask & 0xf0) | (idx_mask & 0xf);

		/*idx1 = (var_flags[flags_idx] >> (nibble_idx * 4)) & 0xf;*/
		/*idx2 = (var_flags[flags_idx] >> ((nibble_idx + 1) * 4)) & 0xf;*/
		idx1 = idx_val & 0xf;
		idx2 = (idx_val >> 4);

		val1_idx = idx1 % 6;
		val2_idx = idx2 % 6;
		if (val1_idx == val2_idx)
			val2_idx = (val1_idx + 1) % 6;

		shift1 = (((val1_idx + (nibble_idx + 2)) % 8) * 4);
		shift2 = (((val2_idx + (nibble_idx + 2)) % 8) * 4);

		val1 = ((var_flags[flags_idx] >> shift1) & 0xf) ^ val2_idx;
		val2 = ((var_flags[flags_idx] >> shift2) & 0xf) ^ val1_idx;

		flag |= ((val1 << (val_nibble_idx * 4)) | (val2 << ((val_nibble_idx + 1) * 4)));
		val_nibble_idx += 2;

		/*下一个元素*/
		flags_idx++;
		nibble_idx = (idx1 + idx2) % 7;
	}

	*flag_ptr = flag;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_vars_global_flag RS_HIDE(_lg)
#endif
noused notrace static int get_vars_global_flag(size_t flag_enum, intptr_t *flag_ptr);

#if defined(CONFIG_RS_CHECK_FUNC)

#define RS_CRC_PREFETCH_SIZE (0) /*(256)*/

#define RS_CRC_PREFETCH

#define RS_CRC_PREFETCH_IN_LOOP

#define RS_CRC_BAT_SIZE (1024)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_crc_table RS_HIDE(_37_)
#endif
noused notrace static RS_CHECKSUM_TYPE *rs_get_crc_table(void)
{
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_saved_crc_table RS_HIDE(_37_1)
#endif
	static uintptr_t *g_saved_crc_table;

	uintptr_t *page;
	uintptr_t *vars;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift;

	if (g_saved_crc_table) {
		page = g_saved_crc_table;
		goto out;
	}

	vars = g_rs_op_vars;

	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

#if !defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	if (page) {
		if (rs_is_valid_vars_page_slim(page)) {
			page += (RS_OP_VAR_PAGE_DATA_SIZE / sizeof(*page));
			g_saved_crc_table = page;
		} else {
			page = NULL;
		}
	}

#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

out:
	return (RS_CHECKSUM_TYPE *)page;
}


#ifdef __GNUC__
	#define PREFETCH(location) __builtin_prefetch(location)
#else
	#define PREFETCH(location)
#endif

#if (defined(__KERNEL__) && defined(__BIG_ENDIAN)) || (defined(__BYTE_ORDER) && (__BYTE_ORDER == __BIG_ENDIAN))
	#define RS_BIG_ENDIAN
#endif


#if defined(CONFIG_ARM64)

static inline RS_CHECKSUM_TYPE noused rs_swap64(RS_CHECKSUM_TYPE a)
{
#if defined(__GNUC__) || defined(__clang__)
	return __builtin_bswap64(a);
#else
	u64 m;

	m = (u64)(0xff00ff00ff00ff);
	a = ((a >> 8) & m) | (a & m) << 8;
	m = (u64)(0xffff0000ffff);
	a = ((a >> 16) & m) | (a & m) << 16;
	return a >> 32 | a << 32;
#endif
}

/*noused const u64 Polynomial_val1 = U64_C(0xD65F03C0D2800000);*/ /*MOV X0, #0;RET*/
/*noused const u64 Polynomial_val2 = U64_C(0x1F33545505070F42);*/
/*noused const u64 Polynomial = U64_C(0xc96c5795d7870f42);*/ /*0xd800000000000000*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_crc_table RS_HIDE(_38_)
#endif
noused notrace static int rs_init_crc_table(void)
{
	unsigned n, k;
	RS_CHECKSUM_TYPE crc;
	const u64 Polynomial_val1 = U64_C(0xD65F03C0D2800000); /*MOV X0, #0;RET*/
	const u64 Polynomial_val2 = U64_C(0x1F33545505070F42);
	RS_CHECKSUM_TYPE Polynomial;
	rs_crc_8slice_table_type table = (rs_crc_8slice_table_type)rs_get_crc_table(); /*g_rs_crc_table;*/

	if (!table)
		return -EINVAL;

	Polynomial = Polynomial_val1 ^ Polynomial_val2;

	/* generate CRC-64's for all single byte sequences */
	for (n = 0; n < 256; n++) {
		crc = n;
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (-((s64)(crc & 1)) & Polynomial);
			/*crc = crc & 1 ? Polynomial ^ (crc >> 1) : crc >> 1;*/
		(*table)[0][n] = crc;

		/* generate CRC-64's for those followed by 1 to 7 zeros */
		for (k = 1; k < 8; k++) {
			crc = (*table)[0][crc & 0xff] ^ (crc >> 8);
			(*table)[k][n] = crc;
		}
	}

#if defined(RS_BIG_ENDIAN)
	for (k = 0; k < 8; k++)
		for (n = 0; n < 256; n++)
			(*table)[k][n] = rs_swap64((*table)[k][n]);
#endif

	return 0;
}

/*crcspeed64little*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_crc RS_HIDE(_39_)
#endif
noused notrace static int rs_crc(const void *data, size_t length, RS_CHECKSUM_TYPE crc, RS_CHECKSUM_TYPE *out_crc)
{
#if 0
	*out_crc = 0;
	return 0;
#else
	rs_crc_8slice_table_type table;
	if (!out_crc) {
		return -EINVAL;
	}

	table = (rs_crc_8slice_table_type)rs_get_crc_table(); /*g_rs_crc_table;*/

	if ((!table) || (!(*table)[1])) {
	#if defined(RS_CRC_PREFETCH)
		PREFETCH(((const char *)data) + length);
	#endif
		*out_crc = (RS_CHECKSUM_TYPE)crc32_le((u32)crc, (unsigned char const *)data, length);
	} else {
		const RS_CHECKSUM_TYPE *curr = (const RS_CHECKSUM_TYPE *)data;

		const size_t Unroll = 4;
		const size_t BytesAtOnce = sizeof(RS_CHECKSUM_TYPE) * Unroll;
		const u8 *currentChar;
		size_t prefix_len = (uintptr_t)data & (sizeof(RS_CHECKSUM_TYPE) - 1);
	#if (RS_CRC_BAT_SIZE)
		size_t count = 0;
	#endif

	#if defined(RS_BIG_ENDIAN)
		crc = rs_swap64(crc);
	#endif

	#if defined(RS_CRC_PREFETCH)
		PREFETCH(((const char *)(*table)));

		PREFETCH(((const char *)curr) + length);
	#endif

		if (prefix_len) {
			length -= prefix_len;

			currentChar = (const u8 *)curr;
			do {
			#if defined(RS_BIG_ENDIAN)
				crc = (*table)[0][(crc >> 56) ^ *currentChar++] ^ (crc << 8);
			#else
				crc = (*table)[0][(crc ^ *currentChar++) & 0xff] ^ (crc >> 8);
			#endif
			} while (prefix_len-- != 0);

			curr = (const RS_CHECKSUM_TYPE *)currentChar;
		}

		/* fast middle processing, 8 bytes (aligned!) per loop */
		while (length >= (BytesAtOnce + RS_CRC_PREFETCH_SIZE)) {
			size_t unrolling;

		#if RS_CRC_PREFETCH_SIZE
			PREFETCH(((const char *)curr) + RS_CRC_PREFETCH_SIZE);
		#elif defined(RS_CRC_PREFETCH_IN_LOOP)
			PREFETCH(((const char *)curr) + BytesAtOnce);
		#endif

			for (unrolling = 0; unrolling < Unroll; unrolling++) {
				crc ^= *curr++;
			#if defined(RS_BIG_ENDIAN)
				crc = (*table)[0][crc & 0xff] ^
						(*table)[1][(crc >> 8) & 0xff] ^
						(*table)[2][(crc >> 16) & 0xff] ^
						(*table)[3][(crc >> 24) & 0xff] ^
						(*table)[4][(crc >> 32) & 0xff] ^
						(*table)[5][(crc >> 40) & 0xff] ^
						(*table)[6][(crc >> 48) & 0xff] ^
						(*table)[7][crc >> 56];
			#else
				crc = (*table)[7][crc & 0xff] ^
						(*table)[6][(crc >> 8) & 0xff] ^
						(*table)[5][(crc >> 16) & 0xff] ^
						(*table)[4][(crc >> 24) & 0xff] ^
						(*table)[3][(crc >> 32) & 0xff] ^
						(*table)[2][(crc >> 40) & 0xff] ^
						(*table)[1][(crc >> 48) & 0xff] ^
						(*table)[0][crc >> 56];
			#endif
		}

		#if (RS_CRC_BAT_SIZE)
			count += BytesAtOnce;
			if (unlikely((count & (RS_CRC_BAT_SIZE - 1)) == 0)) {
				cond_resched();
			}
		#endif

			length -= BytesAtOnce;
		}

		/* process remaining bytes (can't be larger than 8) */
		currentChar = (const u8 *)curr;
		while (length) {
		#if defined(RS_BIG_ENDIAN)
			crc = (*table)[0][(crc >> 56) ^ *currentChar++] ^ (crc << 8);
		#else
			crc = (*table)[0][(crc ^ *currentChar++) & 0xff] ^ (crc >> 8);
		#endif
			length--;
		}

	#if defined(RS_BIG_ENDIAN)
		*out_crc = rs_swap64(crc);
	#else
		*out_crc = crc;
	#endif
	}

	return 0;
#endif
}

#else

/*zlib's CRC32 polynomial*/
/*const RS_CHECKSUM_TYPE Polynomial_val1 = U32_C(0xE1A0C00D);*/ /*MOV R12, SP*/
/*const RS_CHECKSUM_TYPE Polynomial_val2 = U32_C(0xFF7CAF4C);*/
/*const u32 Polynomial = U32_C(0x1EDC6F41);*/ /*0xEDB88320;*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_crc_table RS_HIDE(_38_)
#endif
noused notrace static int rs_init_crc_table(void)
{
/* same algorithm as crc32_bitwise */
	unsigned i, j;
	RS_CHECKSUM_TYPE crc;
	const RS_CHECKSUM_TYPE Polynomial_val1 = U32_C(0xE1A0C00D); /*MOV R12, SP*/
	const RS_CHECKSUM_TYPE Polynomial_val2 = U32_C(0xFF7CAF4C);
	RS_CHECKSUM_TYPE Polynomial;
	rs_crc_16slice_table_type table = (rs_crc_16slice_table_type)rs_get_crc_table(); /*g_rs_crc_table;*/

	if (!table)
		return -EINVAL;

	Polynomial = Polynomial_val1 ^ Polynomial_val2;

	for (i = 0; i < 256; i++) {
		crc = i;
		for (j = 0; j < 16/*8*/; j++)
			crc = (crc >> 1) ^ (-((s32)(crc & 1)) & Polynomial);
			/*crc = crc & 1 ? Polynomial ^ (crc >> 1) : crc >> 1;*/

		(*table)[0][i] = crc;

		for (j = 1; j < 16/*8*/; j++) {
			crc = (*table)[j - 1][i];
			(*table)[j][i] = (crc >> 8) ^ (*table)[0][crc & 0xFF];
		}
	}
	/* ... and the following slicing-by-8 algorithm (from Intel):
	 * http://www.intel.com/technology/comms/perfnet/download/CRC_generators.pdf
	 * http://sourceforge.net/projects/slicing-by-8/
	 */
	/*
	for (slice = 1; slice < 8; slice++)
		(*table)[slice][i] = ((*table)[slice - 1][i] >> 8) ^ (*table)[0][(*table)[slice - 1][i] & 0xFF];
	*/

	return 0;
}

static inline RS_CHECKSUM_TYPE noused rs_swap32(RS_CHECKSUM_TYPE x)
{
#if defined(__GNUC__) || defined(__clang__)
	return __builtin_bswap32(x);
#else
	return (x >> 24) |
				((x >>  8) & 0x0000FF00) |
				((x <<  8) & 0x00FF0000) |
				(x << 24);
#endif
}

/*compute CRC32 (Slicing-by-16 algorithm), unroll inner loop 4 times*/ /*crc32_16bytes_prefetch()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_crc RS_HIDE(_39_)
#endif
noused int rs_crc(const void *data, size_t length, RS_CHECKSUM_TYPE crc, RS_CHECKSUM_TYPE *out_crc)
{
	rs_crc_16slice_table_type table;

	if (!out_crc) {
		return -EINVAL;
	}

	table = (rs_crc_16slice_table_type)rs_get_crc_table(); /*g_rs_crc_table;*/

	if ((!table) || (!(*table)[1])) {
	#if defined(RS_CRC_PREFETCH)
		PREFETCH(((const char *)data) + length);
	#endif
		*out_crc = (RS_CHECKSUM_TYPE)crc32_le((u32)crc, (unsigned char const *)data, length);
	} else {
		const RS_CHECKSUM_TYPE *curr = (const RS_CHECKSUM_TYPE *)data;

		/*enabling optimization (at least -O2) automatically unrolls the inner for-loop*/
		const size_t Unroll = 4;
		const size_t BytesAtOnce = (sizeof(RS_CHECKSUM_TYPE) * 4) /*8*/ * Unroll;
		const u8 *currentChar;
		size_t prefix_len = (uintptr_t)data & (sizeof(RS_CHECKSUM_TYPE) - 1);
	#if (RS_CRC_BAT_SIZE)
		size_t count = 0;
	#endif

		crc = ~crc; /*same as previousCrc32 ^ 0xFFFFFFFF*/

	#if defined(RS_CRC_PREFETCH)
		PREFETCH(((const char *)(*table)));

		PREFETCH(((const char *)curr) + length);
	#endif

		if (prefix_len) {
			length -= prefix_len;
			currentChar = (const u8 *)curr;
			do {
				crc = (crc >> 8) ^ (*table)[0][(crc & 0xFF) ^ *currentChar++];
			} while (prefix_len-- != 0);

			curr = (const RS_CHECKSUM_TYPE *)currentChar;
		}

		/*process 4x 16 bytes at once (Slicing-by-16)*/
		while (length >= (BytesAtOnce + RS_CRC_PREFETCH_SIZE)) {
			size_t unrolling;

		#if RS_CRC_PREFETCH_SIZE
			PREFETCH(((const char *)curr) + RS_CRC_PREFETCH_SIZE);
		#elif defined(RS_CRC_PREFETCH_IN_LOOP)
			PREFETCH(((const char *)curr) + BytesAtOnce);
		#endif

			for (unrolling = 0; unrolling < Unroll; unrolling++) {
			#if defined(RS_BIG_ENDIAN)
				RS_CHECKSUM_TYPE one = *curr++ ^ rs_swap32(crc);
				RS_CHECKSUM_TYPE two = *curr++;
				RS_CHECKSUM_TYPE three = *curr++;
				RS_CHECKSUM_TYPE four = *curr++;
				crc = (*table)[0][four       & 0xFF] ^
							(*table)[1][(four >> 8)  & 0xFF] ^
							(*table)[2][(four >> 16)  & 0xFF] ^
							(*table)[3][(four >> 24)  & 0xFF] ^
							(*table)[4][three      & 0xFF] ^
							(*table)[5][(three >> 8) & 0xFF] ^
							(*table)[6][(three >> 16) & 0xFF] ^
							(*table)[7][(three >> 24) & 0xFF] ^
							(*table)[8][two        & 0xFF] ^
							(*table)[9][(two >> 8)  & 0xFF] ^
							(*table)[10][(two >> 16)   & 0xFF] ^
							(*table)[11][(two >> 24)   & 0xFF] ^
							(*table)[12][one        & 0xFF] ^
							(*table)[13][(one >> 8)   & 0xFF] ^
							(*table)[14][(one >> 16)   & 0xFF] ^
							(*table)[15][(one >> 24)   & 0xFF];
			#else
				RS_CHECKSUM_TYPE one = *curr++ ^ crc;
				RS_CHECKSUM_TYPE two = *curr++;
				RS_CHECKSUM_TYPE three = *curr++;
				RS_CHECKSUM_TYPE four = *curr++;
				crc = (*table)[0][(four >> 24)  & 0xFF] ^
							(*table)[1][(four >> 16)  & 0xFF] ^
							(*table)[2][(four >> 8)  & 0xFF] ^
							(*table)[3][four       & 0xFF] ^
							(*table)[4][(three >> 24) & 0xFF] ^
							(*table)[5][(three >> 16) & 0xFF] ^
							(*table)[6][(three >> 8) & 0xFF] ^
							(*table)[7][three      & 0xFF] ^
							(*table)[8][(two >> 24)   & 0xFF] ^
							(*table)[9][(two >> 16)   & 0xFF] ^
							(*table)[10][(two >> 8)    & 0xFF] ^
							(*table)[11][two        & 0xFF] ^
							(*table)[12][(one >> 24)  & 0xFF] ^
							(*table)[13][(one >> 16)  & 0xFF] ^
							(*table)[14][(one >>  8)  & 0xFF] ^
							(*table)[15][one        & 0xFF];
			#endif
			}

		#if (RS_CRC_BAT_SIZE)
			count += BytesAtOnce;
			if ((count & (RS_CRC_BAT_SIZE - 1)) == 0) {
				cond_resched();
			}
		#endif

			length -= BytesAtOnce;
		}

		currentChar = (const u8 *)curr;
		/*remaining 1 to 63 bytes (standard algorithm)*/
		while (length-- != 0)
			crc = (crc >> 8) ^ (*table)[0][(crc & 0xFF) ^ *currentChar++];

		*out_crc = ~crc; /*same as crc ^ 0xFFFFFFFF*/
	}

	return 0;
}
#endif
#endif /*CONFIG_RS_CHECK_FUNC*/


#if defined(CONFIG_RS_CHECK_FUNC)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_calc_func_checksum RS_HIDE(_40_)
#endif

static inline int noused rs_calc_func_checksum(void *func_start, void *func_end, RS_CHECKSUM_TYPE *checksum_ptr)
{
	return rs_crc(func_start, (size_t)func_end - (size_t)func_start, 0, checksum_ptr);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_calc_func_checksum_by_end RS_HIDE(_41_)
#endif
noused notrace static int rs_calc_func_checksum_by_end(RS_FC_SYM_VAL_TYPE * func_end, RS_CHECKSUM_TYPE *checksum_ptr)
{
	int ret;
	void *func_start;
	if (!checksum_ptr)
		return -EINVAL;

	/*below causing relocation overflows in calling func
	func_start = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_start) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	func_end = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_end) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	*/
	func_start = (RS_FC_SYM_PTR_TYPE)(*func_end);
	func_start = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_start) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	func_start = dereference_function_descriptor(func_start);

	/*RS_LOG("cfc,0,%p,%p\n", func_start, func_end);*/
	/*func_end = dereference_function_descriptor(func_end);*/ /*func_end is a variable*/

	if ((((uintptr_t)func_start + max(sizeof(RS_FC_MARK_TYPE), sizeof(u32))) >= (uintptr_t)func_end)
		|| (((uintptr_t)func_end - (uintptr_t)func_start) >= RS_FC_MAX_SIZE)) {
		/*RS_LOG("cfc,1\n");*/
		*checksum_ptr = 0;
		ret = 0;
	} else {
		/*RS_LOG("cfc,2\n");*/
		ret = rs_calc_func_checksum(func_start, func_end, checksum_ptr);
	}

	/*RS_LOG("cfc,e,%d\n", ret);*/
	return ret;
}

RS_DEFINE_FC_BEGIN(rs_verify_func);
RS_FORWARD_FC_END(rs_verify_func);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_vars_checksum RS_HIDE(_le)
#endif
noused notrace static int get_vars_checksum(RS_CHECKSUM_TYPE *checksum_ptr, size_t data_offs);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_verify_func RS_HIDE(_42_)
#endif
noused notrace RS_DEFINE_FC_BODY(rs_verify_func) static noinline int rs_verify_func(RS_FC_SYM_VAL_TYPE * func_end, int func_enum)
{
#if 0
	return 0;
#else
	int ret;
	void *func_start;
	struct task_struct *curr;
	intptr_t flag;

	/*below causing relocation overflows in calling func
	func_start = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_start) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	func_end = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_end) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	*/
	func_start = (RS_FC_SYM_PTR_TYPE)(*func_end);
	func_start = (RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(func_start) + (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE));
	func_start = dereference_function_descriptor(func_start);
	/*func_end = dereference_function_descriptor(func_end);*/ /*func_end is a variable*/

	/*RS_LOG("cf,0,%p,%p,%d\n", func_start, func_end, func_enum);*/

	if ((((uintptr_t)func_start + max(sizeof(RS_FC_MARK_TYPE), sizeof(u32))) >= (uintptr_t)func_end)
		|| (((uintptr_t)func_end - (uintptr_t)func_start) >= RS_FC_MAX_SIZE)) {
		/*RS_LOG("cf,1\n");*/

		ret = 0;
	} else {
		curr = current;
		if ((curr) && ((is_init_task(curr)) || (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm)
			&& (!task_in_call_from_user(curr))))) {
			ret = 0;
		} else {
			intptr_t bootup_flag;

			RS_CHECKSUM_TYPE checksum;

			if (test_fc_bitmap(0) > 0) {
				return 0;
			}

			bootup_flag = fcbdfNotInited;

			if ((!get_vars_global_flag(gfBootupDone, &bootup_flag))
				&& (bootup_flag <= fcbdfPermitted)) {
				/*RS_LOG("cf,1.1\n");*/

				try_set_fc_bitmap(0);
				return 0;
			} else {
				RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
				/*RS_LOG("cf,2,%d\n", (int)flag);*/
				RS_VAR_READ_LOCK;
			#if defined(RS_HIDE_OP_VAR_FLAG)
				rs_fetch_op_var_flag(&rs_op_var_flag);
			#else
				rs_op_var_flag = g_rs_op_var_flag;
			#endif
				RS_VAR_READ_UNLOCK;

				if (!(rs_op_var_flag & (1 << opvChecksum))) {
					/*RS_LOG("cf,3\n");*/
					try_set_fc_bitmap(0);
					ret = 0;
				}
			}

			ret = -EINVAL;
			/*goto OK;*/
		#if 1
			if (test_fc_bitmap(fcCheckFunc + 1) > 0) {
				/*RS_LOG("cf,4\n");*/
				goto OK;
			}

			if (get_vars_checksum(&checksum, fcCheckFunc + 1) == 0) {
				noused RS_CHECKSUM_TYPE tmp_checksum;

				/*RS_LOG("cf,5\n");*/
				if ((!rs_calc_func_checksum_by_end(RS_FC_PARAMS(rs_verify_func), &tmp_checksum))
					&& (tmp_checksum == checksum)) {
					/*RS_LOG("cf,6\n");*/
					try_set_fc_bitmap(fcCheckFunc + 1);

					goto OK;
				}
			}
		#else
			goto OK;
		#endif

		check_if_init_failed:
			{
			#if 1
				/*RS_LOG("cf,7\n");*/
				if ((!get_vars_global_flag(gfFuncChecksumsOK, &flag)) && (!flag)) {
					/*RS_LOG("cf,8\n");*/
					try_set_fc_bitmap(0);
					ret = 0;
				} else {
					RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
					/*RS_LOG("cf,9,%d\n", (int)flag);*/
					RS_VAR_READ_LOCK;
				#if defined(RS_HIDE_OP_VAR_FLAG)
					rs_fetch_op_var_flag(&rs_op_var_flag);
				#else
					rs_op_var_flag = g_rs_op_var_flag;
				#endif
					RS_VAR_READ_UNLOCK;

					if (!(rs_op_var_flag & (1 << opvChecksum))) {
						/*RS_LOG("cf,a\n");*/
						try_set_fc_bitmap(0);
						ret = 0;
					}
				}
			#else
				ret = 0;
			#endif
			}

			if ((ret) && (bootup_flag == fcbdfByPassed)) {
				/*bypass check func*/
				ret = 0;
			}

			goto out;

		OK:
			if (func_enum == fcCheckFunc) {
				ret = 0;
			} else if (test_fc_bitmap(func_enum + 1) > 0) {
				ret = 0;
			} else {
			#if 1
				/*RS_LOG("cf,b\n");*/
				if (get_vars_checksum(&checksum, func_enum + 1) == 0) {
					/*ret = 0;*/
					/*goto out;*/
				#if 1
					RS_CHECKSUM_TYPE tmp_checksum;
					if ((!rs_calc_func_checksum(func_start, func_end, &tmp_checksum))
						&& (tmp_checksum == checksum)) {
						/*RS_LOG("cf,c,%d\n", func_enum);*/
						try_set_fc_bitmap(func_enum + 1);

						ret = 0;
						goto out;
					}
					/*RS_LOG("cf,d,%p,%p\n", (void *)checksum, (void *)tmp_checksum);*/
				#endif
				}

				goto check_if_init_failed;
			#endif
			}
		}
	}

out:

	/*
	if (ret) {
		RS_LOG("cf,e,%d,%p,%p,%d\n", ret, func_start, func_end, func_enum);
		RS_LOG("cf,e,%d\n", func_enum);
		ret = 0;
	}
	*/

	return ret;
#endif
}

RS_DEFINE_FC_END(rs_verify_func);

#else
	/*#define update_func_checked_flag(...) do{}while(0)*/
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_permit_vars RS_HIDE(_l9)
#endif
noused notrace static int set_permit_vars(void *data);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_bypass_vars RS_HIDE(_la)
#endif
noused notrace static int set_bypass_vars(void *data);

typedef struct {
	const char *str;
	int str_len;
} check_string;

#define NULL_CS_ITEM "", 0

enum {
	csSlash,
	csSystem,
	csSystemSlash,
	csApps,
	csAppsSlash,
	csTmp,
	csTmpSlash,
	csMksh,
	csAdbd,
	csRecovery,
	csInit,
	csDataSlash,
	csDevSlash,
	csMntSlash,
	csProcSlash,
	csSysSlash,
	csUpdateBinary,
	csSdcard0sh,
	csSdcard1sh,
	csVold,

	csLogWrapper,
	csBBKSu_NotUsed,
	csIQooSecurePart,
	csBBKThemePart,
	csSettingsPart,
	csBBKLauncherPart,
	csAppProcess,
	csIQooSecureApk,
	csBBKThemeApk,
	csSettingsApk,
	csBBKLauncherApk,
	csSbinSlash,
	csFrameworkVivo,
	csJar,
	csApk,
	csSystemAppSlash,

	csZygote,
	csSystemServer,
	csSystemFrameworkSlash,
	csFrameworkResPart,
	csVivoResPart,
	csSystemLibPrefix,
	csAndroidServersPart,
	csSensorServicePart,
	csSurfaceFlingerPart,
	csAdbdPart,
	csRootFS,
	csAndroid,

	csEmmc,
	csDevBlockSlash,
	csVendorSlash,
	csInstallPart,
	csInitDotPart,

	csAtcid,
	csVivoEmSvr,

	csDalvikvm,
	csDvz,
	csBusybox,

	csMmcblk0p,
	csMtkMsdc0,
	csMsmSdcc1,
	csEmmcAndroid,
	csEmmcApps,
	csMsmSystem,
	csMsmApps,

	csMount,
	csInsmod,
	csModprobe,

	csUniqueTMS,

	csDevBackup,
	csBlkPlatformSlash,
	csSlashByName,
	csSlashBackup,
	csMMCBlk0CID,
	csBootDevicePrefix,
	csSlashMMCBlk0,
	csIQooSecure,
	csDataUsageSlash,
	csAppBin,
	csRO,
	csRW,
	csRemount,
	csLoopEqu,
	csLoop,

	csRSId,
	csRSLog,
	csRSPermit,
	csRSByPass,
	csRSCfgUpd,

	csOptAll,
	csOptChRoot,
	csOptDisableSELinux,
	csOptExec,
	csOptFilesCheck,
	csOptInsmod,
	csOptMount,
	csOptPtrace,
	csOptPivotRoot,
	csOptSetuid,
	csOptAccessDev,

	csUnbindable,
	csPrivate,
	csSlave,
	csShared,

	csRecUnbindable,
	csRecPrivate,
	csRecSlave,
	csRecShared,

	csDalvikCacheSlash,
	csClassesDex,
	csBootOat,

	csUserDot,
	csRoot,
	csSuper,
	csSystemDownloader,

	csUObjectUnlabeled,

	csSystemLib64Prefix,
	csDevKmem,

	csBBKDebugFile,
	csProductVersion,

	csRSJournal,

	csVivoDaemonService,

	csOem,
	csSh,

	csOptSuExec,

	csOptCheckBoot,
	csOptFuncCheck,

	csDevNull,
	csDevZero,
	csDevMem,
	csProcKallsyms,

	csBBKTelBox,
	csBBKMobileBox,

	csBoot,
	csLK,
	csBootImg,
	csUBoot,

	csPermit,
	csKThread,
	csPass,

	csBlockFileSizeFmt,

	csPersistPropDir,
	csPersistPropTempFmt,
	csPersistPropPathFmt,

	csPropRODot,
	csPropNetDot,
	csPropNetChange,
	csPropPersistDot,
	csPropImport,

	csPropFileName,
	csPropServiceSocket,

	csTmpFS,
	csUser,
	csMnt,
	csSu,
	csUs,

	csABoot,
	csBmtPool,

	csDiskNameWDigitFmt,
	csDiskNameFmt,

	csVivoBox,
	csSCM,

	csSocSlash,
	csSlashSda,
	csAndroidBootSerialNo,

	csBlkBootDevice,
	csSurvival,

	csUFSDeviceID,

	csLKSecApp,
	csKeyMaster,
	csDevInfoMagic,
	csSlashAboot,
	csSlashDevInfo,
	csDevLK,
	csDevSecCfg,

	csProcSecEnable,
	csOptSockCheck,

	csAndroidBootEmmc,
	csTrue,
	csOptLoadSELinuxPolicy,

	csOemPartName, /*for /oem mount dev_name, "/oem" for qcom, "/cust" for mtk(MTK的OEM分区不能用oem命名)*/
	csOemSlash, /*for /oem mount dir_name*/

	csVivoDaemon,

	csRecoverySystemSlash,

	csDevRoot,
	csSplashRoot,

	csSlashConfigPart,

	csVendor,
	csVBMeta,

	csXbinSu,
	csBootDeviceIDPrefix,
	csBionicSlash,
	csApexSlash,

	csSAMSUNGBootModePrefix,
	csSAMSUNGFactoryMode,
	csSAMSUNGRootDevRAM0,

	csVadbd,
	csVadbdPart,
	csVusbd,
	csVusbdPart,

	/*and new items before csInvalid*/
	csInvalid
};

#if defined(RS_WITHOUT_MKSH)
	#define SH_STR_IDX (csSh)
#else
	#define SH_STR_IDX (csMksh)
#endif

enum {
	sidcsBBKSu,
	sidcsNetCfg,
	sidcsRunAs,
	sidcsCpLog,

	/*and new items before sidcsInvalid*/
	sidcsInvalid
};

enum {
	suaAutoDaemon,
	suaDaemon,
	suaMountMaster,
	suaReload,
	suaAutoDaemonShort,
	suaDaemonShort,
	suaMountMasterShort,
	suaReloadShort,

	/*and new items before suaInvalid*/
	suaInvalid
};

#if defined(RS_ENCRYPT_STR)
/*编码后字符串内容不应包含\x00，否则比较函数的结果会不正确，除非不使用比较、搜索相关函数*/

#define INIT_ENCRYPT_CHAR (0xBE)
#define INIT_ENCRYPT_SEED_MASK (47)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define simple_str_encrypt RS_HIDE(_32)
#endif
noused notrace static void simple_str_encrypt(char *str, int len, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	char *str_end = str + len;
	while (str < str_end) {
		*(unsigned char *)str ^= ch;
		++str;
		ch--;
	}
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define simple_str_encrypt_no_len RS_HIDE(_33)
#endif
noused notrace static void simple_str_encrypt_no_len(char *str, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	while (*str) {
		*(unsigned char *)str ^= ch;
		++str;
		ch--;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define simple_str_encrypt_ex RS_HIDE(_34)
#endif
noused notrace static void simple_str_encrypt_ex(char *str, int offset, int len, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - offset;
	char *str_end = str + len;
	while (str < str_end) {
		*(unsigned char *)str ^= ch;
		++str;
		ch--;
	}
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define simple_str_encrypt_ex_no_len RS_HIDE(_35)
#endif
noused notrace static void simple_str_encrypt_ex_no_len(char *str, int offset, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - offset;
	while (*str) {
		*(unsigned char *)str ^= ch;
		++str;
		ch--;
	}
}

/*/////////////////////////////////////////////*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_strings RS_HIDE(_40)
#endif
noused
static const check_string check_strings[csInvalid] = {
	{
		"\x91",
		sizeof("/") - 1
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA",
		sizeof("/system") - 1
	},
	{
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A",
		sizeof("/system/") - 1
	},
	{
		"\x94\xDB\xC9\xC8\xC4",
		sizeof("/apps") - 1
	},
	{
		"\x95\xD8\xC8\xC7\xC5\x9A",
		sizeof("/apps/") - 1
	},
	{
		"\x96\xCC\xDA\xC6",
		sizeof("/tmp") - 1
	},
	{
		"\x97\xC3\xDB\xC5\x9B",
		sizeof("/tmp/") - 1
	},
	{
		"\x98\xC5\xCC\xC7\xC7\xD7\xDC\x9F\xCD\xC7\xC3\x83\xC6\xC1\xDA\xC0",
		sizeof("/system/bin/mksh") - 1
	},
	{
	#if defined(RS_IS_ANDROID_8_1_ABOVE)
		"\x99\xC6\xCD\xC0\xC6\xD4\xDD\x80\xCC\xC4\xC2\x84\xCB\xCD\xCA\xC3",
		sizeof("/system/bin/adbd") - 1
	#else
		"\x99\xC6\xD6\xDA\xDC\x9E\xD1\xCB\xCC\xC9",
		sizeof("/sbin/adbd") - 1
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"\x9A\xC7\xCA\xC1\xC5\xD5\xC2\x81\xCF\xC5\xC5\x85\xDB\xCD\xC4\xC9\xD3\xC1\xD1\xDB",
		sizeof("/system/bin/recovery") - 1
	#else
		"\x9A\xC7\xD1\xDB\xDF\x9F\xDD\xCB\xCE\xC3\xDD\xCF\xDB\xD1",
		sizeof("/sbin/recovery") - 1
	#endif
	},
	{
		"\x9B\xDA\xDC\xD8\xC4",
		sizeof("/init") - 1
	},
	{
		"\x9C\xD6\xD0\xC4\xCE\x81",
		sizeof("/data/") - 1
	},
	{
		"\x9D\xD5\xD5\xD9\x81",
		sizeof("/dev/") - 1
	},
	{
		"\x9E\xDD\xC1\xDA\x82",
		sizeof("/mnt/") - 1
	},
	{
		"\x9F\xDF\xDC\xC2\xCF\x84",
		sizeof("/proc/") - 1
	},
	{
		"\x80\xDD\xD4\xDF\x84",
		sizeof("/sys/") - 1
	},
	{
	#if defined(RS_IS_ANDROID_8_1_ABOVE)
		"\x91\xC9\xD1\xCB\x95\xCC\xC8\xD3\xD7\xC1\xD1\x9E\xD0\xD8\xDE\xCE\xDC\xD4",
		(sizeof("/tmp/update-binary") - 1)
	#else
		"\x91\xC9\xD1\xCB\x95\xCC\xC8\xD3\xD7\xC1\xD1\xEC\xD0\xD8\xDE\xCE\xDC\xD4",
		(sizeof("/tmp/update_binary") - 1)
	#endif
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xD0\xC0\xD0\x9D\xC2\xD4\xCC\xCF\xDF\xC8\x9B\x84\xDA\xC0",
		(sizeof("/system/etc/sdcard0.sh") - 1)
	},
	{
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xD1\xC7\xD1\x9E\xC3\xCB\xCD\xCC\xDE\xCF\x9B\x87\xDB\xCF",
		(sizeof("/system/etc/sdcard1.sh") - 1)
	},
	{
		"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xD1\xDB\xDF\x9F\xD9\xC1\xC1\xC8",
		(sizeof("/system/bin/vold") - 1)
	},

	{
		"\x95\xCA\xC1\xC4\xC2\xD0\xD9\x9C\xD0\xD8\xDE\x80\xC2\xC2\xCB\xDC\xD8\xC8\xD8\xD7\xC3\xD7",
		sizeof("/system/bin/logwrapper") - 1
	},
	{
		"", /*"\x96\xCB\xCE\xC5\xC1\xD1\xDE\x9D\xC9\xD2\xC6\xC0\x82\xCE\xC9\xC1\xDA\xDD",*/
		0, /*sizeof("/system/xbin/bbksu") - 1*/
	},
	{
		"\xD1\xC6\xD9\xDA\x9A\xC0\xD7\xD2\xC5\xDD\xCB",
		sizeof("iqoo.secure") - 1
	},
	{
		"\xD5\xD4\xDE\x9A\xC7\xDA\xD4\xDD\xCA",
		sizeof("bbk.theme") - 1
	},
	{
		"\xD7\xDB\xD0\xC1\xDD\xD8\xD4\x81\xDD\xC8\xD8\xDF\xC3\xC7\xCF\xD4",
		sizeof("android.settings") - 1
	},
	{
		"\xD7\xD6\xD8\x9C\xDD\xD1\xDA\xC0\xCE\xC4\xCE\xD8\x9B",
		sizeof("bbk.launcher2") - 1
	},
	{
		"\x9B\xC0\xCB\xC2\xC4\xCA\xC3\x82\xCE\xC2\xC4\x86\xC9\xD7\xD6\xFA\xD4\xD1\xCD\xC2\xC5\xEC\xED",
		sizeof("/system/bin/app_process") - 1
	},
	{
		"\xFA\xC3\xDE\xDF\xFC\xCB\xCE\xD9\xD9\xCF",
		sizeof("IqooSecure") - 1
	},
	{
		"\xF0\xF3\xFB\xFB\xC6\xC8\xC1\xCE",
		sizeof("BBKTheme") - 1
	},
	{
		"\xE2\xD5\xDB\xDA\xC4\xC2\xCC\xD9",
		sizeof("Settings") - 1
	},
	{
		"\xF2\xED\xE5\xE1\xCD\xDE\xC4\xCA\xC0\xC2\xD4\x97",
		sizeof("BBKLauncher2") - 1
	},
	{
		"\x80\xDD\xCF\xC5\xC5\x85",
		sizeof("/sbin/") - 1
	},
	{
		"\xB1\xEE\xE5\xE8\xEE\xFC\xF5\xB8\xF0\xE7\xF5\xFE\xF7\xE6\xFF\xFD\xE5\xA2\xFA\xE2\xFC\xE6\xA5",
		sizeof("/system/framework/vivo-") - 1
	},
	{
		"\xF7\xFD\xE9",
		sizeof("jar") - 1
	},
	{
		"\xFD\xEB\xF1",
		sizeof("apk") - 1
	},
	{
		"\xB4\xE9\xE0\xEB\xE3\xF3\xF8\xBB\xF2\xE2\xE1\xBF",
		sizeof("/system/app/") - 1
	},

	{
		"\xE0\xE0\xFF\xF8\xE2\xF0",
		sizeof("zygote") - 1
	},
	{
		"\xEA\xE1\xE4\xE2\xF0\xF9\xCC\xE1\xF4\xE2\xF9\xEB\xFF",
		sizeof("system_server") - 1
	},
	{
		"\xB7\xE4\xEF\xE6\xE0\xF6\xFF\xBE\xF6\xFD\xEF\xE0\xE9\xFC\xE5\xFB\xE3\xA8",
		sizeof("/system/framework/") - 1
	},
	{
		"\xF1\xE4\xF4\xF9\xF6\xE5\xFE\xE2\xE4\xA3\xFF\xE9\xF8",
		sizeof("framework-res") - 1
	},
	{
		"\xE0\xFC\xE2\xFC\xBF\xE3\xF5\xFC",
		sizeof("vivo-res") - 1
	},
	{
		"\xBA\xE7\xEA\xE1\xE5\xF5\xE2\xA1\xE1\xE5\xE9\xA5\xE5\xE1\xE5",
		sizeof("/system/lib/lib") - 1
	},
	{
		"\xF5\xFD\xF6\xE3\xFF\xE6\xEA\xD2\xFF\xEE\xF8\xFF\xED\xF5\xF5",
		sizeof("android_servers") - 1
	},
	{
		"\xE0\xF7\xFF\xE3\xE0\xFC\xFE\xE9\xF9\xFC\xE0\xEB\xE2",
		sizeof("sensorservice") - 1
	},
	{
		"\xE1\xE4\xE2\xE9\xEF\xEE\xE9\xED\xE6\xE0\xE6\xE0\xE3\xF7",
		sizeof("surfaceflinger") - 1
	},
	{
		"\xF0\xF4\xED\xEA",
		sizeof("adbd") - 1
	},
	{
		"\xE2\xE0\xE1\xF9\xEA\xF8",
		sizeof("rootfs") - 1
	},
	{
		"\xEE\xE0\xE9\xFE\xE4\xE3\xED",
		sizeof("android") - 1
	},
	{
		"\xB1\xF8\xF1\xF6\xF9",
		sizeof("/emmc") - 1
	},
	{
		"\xB2\xF8\xFE\xEC\xB6\xFA\xFB\xF9\xF6\xFF\xBC",
		sizeof("/dev/block/") - 1
	},
	{
		"\xB3\xED\xFF\xF7\xFC\xF8\xE4\xBA",
		sizeof("/vendor/") - 1
	},
	{
		"\xF2\xF4\xEA\xEC\xF6\xFA\xF9",
		sizeof("install") - 1
	},
	{
		"\xF3\xF7\xF1\xE3\xB8",
		sizeof("init.") - 1
	},
	{
		"", /*"\xB6\xEB\xEE\xE5\xE1\xF1\xFE\xBD\xF3\xF9\xE1\xA1\xEC\xF8\xE8\xE3\xED",*/
		0, /*sizeof("/system/bin/atcid") - 1*/
	},
	{
		"", /*"\xB7\xE4\xEF\xE6\xE0\xF6\xFF\xBE\xF2\xE6\xE0\xA2\xFA\xE2\xFC\xE6\xD7\xE2\xEB\xDA\xF7\xF5\xF0",*/
		0, /*sizeof("/system/bin/vivo_em_svr") - 1*/
	},
	{
		"\xF3\xF7\xF9\xE2\xFA\xF9\xE7\xFD",
		sizeof("dalvikvm") - 1
	},
	{
		"\xF2\xE3\xEE",
		sizeof("dvz") - 1
	},
	{
		"\xF7\xE1\xE0\xEB\xF3\xFF\xF7",
		sizeof("busybox") - 1
	},

	{
		"\xBB\xF7\xF7\xE7\xBF\xED\xE2\xE2\xEF\xE0\xA5\xE4\xE5\xE4\xE4\xE9\xEF\xB3\xF2",
		sizeof("/dev/block/mmcblk0p") - 1
	},
	{
		"", /*"\xBC\xF6\xF4\xE6\xA0\xEC\xE1\xE3\xE8\xE1\xA6\xF8\xEB\xE7\xF1\xE2\xEC\xF0\xEC\xAF\x12\x0A\x16\x51\x16\x09\x1D\x1B\x59\x46\x5A",*/
		0, /*sizeof("/dev/block/platform/mtk-msdc.0/") - 1*/
	},
	{
		"", /*"\xBD\xF5\xF5\xF9\xA1\xEF\xE0\xE4\xE9\xE2\xA7\xF7\xEA\xE4\xF0\xE5\xED\xF3\xED\x50\x13\x0E\x11\x24\x09\x1D\x1B\x14\x58\x44\x5B",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/") - 1*/
	},
	{
		"\xBE\xF5\xE2\xE3\xEE\xCC\xEA\xE4\xED\xFA\xE8\xEF\xE1",
		sizeof("/emmc@android") - 1
	},
	{
		"\xBF\xEA\xE3\xE0\xEF\xCB\xEB\xF9\xF8\xF4",
		sizeof("/emmc@apps") - 1
	},
	{
		"", /*"\xA0\xEA\xE8\xFA\xA4\xE8\xE5\xE7\xE4\xED\xAA\xF4\xEF\xE3\xF5\xE6\x10\x0C\x10\x53\x16\x09\x14\x27\x04\x12\x16\x17\x5D\x43\x5E\x12\x16\x43\x03\x0D\x06\x0F\x46\x1B\x1E\x15\x11\x01\x0E",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/by-name/system") - 1*/
	},
	{
		"", /*"\x91\xD9\xD9\xCD\x95\xDB\xD4\xD8\xD5\xDE\x9B\xC3\xDE\xD0\xC4\xC9\xC1\xDF\xC1\x84\xC7\xDA\xC5\xF8\xD5\xC1\xC7\xC0\x8C\x90\x8F\xFD\xE7\xB0\xF2\xFA\xF7\xFC\xB7\xF6\xE6\xE5\xE7",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/by-name/apps") - 1*/
	},
	{
		"\xD0\xD3\xCE\xD4\xCD",
		sizeof("mount") - 1
	},
	{
		"\xD5\xD5\xC9\xD4\xD7\xD3",
		sizeof("insmod") - 1
	},
	{
		"\xD6\xD5\xDD\xC8\xC5\xD9\xD7\xD1",
		sizeof("modprobe") - 1
	},
	{
	#if !defined(RS_WITHOUT_BBKSU)
		"\x95\xCA\xC1\xC4\xC2\xD0\xD9\x9C\xCA\xD3\xD9\xC1\x81\xD8\xC2\xC2\xDB\xDC\xCD\xD3\xCB\xD6",
		sizeof("/system/xbin/uniquetms") - 1
	#else
		NULL_CS_ITEM
	#endif
	},

	{
		"\x96\xDC\xD2\xC0\x9A\xD6\xD2\xD1\xDA\xC5\xDF",
		sizeof("/dev/backup") - 1
	},
	{
		"\x97\xD3\xD3\xC3\x9B\xD1\xDE\xDE\xD3\xC4\x81\xDD\xC0\xCA\xDE\xCF\xC7\xD5\xCB\x8A",
		sizeof("/dev/block/platform/") - 1
	},
	{
		"\x98\xD4\xCC\x99\xDD\xD3\xDC\xD5",
		sizeof("/by-name") - 1
	},
	{
		"\x99\xD7\xD5\xD0\xD9\xC4\xC0",
		sizeof("/backup") - 1
	},
	{
		"\x9A\xC7\xCA\xC1\x9E\xD2\xC3\xC1\xCE\xC7\x84\xC7\xC4\xCB\xC5\xCA\xCE\x94\x8C\xC6\xC4\xD6\xF6\xFD\xF8\xB3\xF8\xF3\xFD",
		sizeof("/sys/block/mmcblk0/device/cid") - 1
	},
	{
		"\xD5\xDD\xD6\xC3\xDF\xC6\xCA\xCF\xC3\xC4\xDE\x87\xCA\xC8\xC9\xD1\xC0\xC6\xD4\xC8\xC3\xFA\xA3",
		sizeof("androidboot.bootdevice=") - 1
	},
	{
		"\x9C\xDF\xDC\xD3\xCD\xC2\xC6\x9C",
		sizeof("/mmcblk0") - 1
	},
	{
		"\x9D\xD2\xDF\xC2\x80\xC4\xDD\xC4\xC5\x87\xDB\xC2\xC5\xD0\xD6\xC6",
		sizeof("/com.iqoo.secure") - 1
	},
	{
		"\xD5\xD1\xDB\xCF\xD8\xDF\xCA\xCD\xCC\x87",
		sizeof("datausage/") - 1
	},
	{
		"\x9F\xCE\xDE\xDD\xF3\xC9\xC3\xC7\x87",
		sizeof("/app_bin/") - 1
	},
	{
		"\xDD\xC1",
		sizeof("ro") - 1
	},
	{
		"\xCC\xCA",
		sizeof("rw") - 1
	},
	{
		"\xCF\xD9\xD6\xD5\xCC\xD6\xC3",
		sizeof("remount") - 1
	},
	{
		"\xD0\xD4\xD5\xC9\x85",
		sizeof("loop=") - 1
	},
	{
		"\xD7\xD5\xD6\xC8",
		sizeof("loop") - 1
	},
	{
		"\xC8\xCA\x96\xDE\xD2\x88",
		sizeof("rs.id=") - 1
	},
	{
		"\xCB\xCB\x99\xDA\xDA\xD3\x8E",
		sizeof("rs.log=") - 1
	},
	{
		"\xCA\xC4\x98\xC5\xD1\xC1\xDF\xD8\xC4\x92",
		sizeof("rs.permit=") - 1
	},
	{
		"\xC5\xC5\x9B\xD6\xCA\xC2\xD0\xC3\xDC\x93",
		sizeof("rs.bypass=") - 1
	},
	{
		"\xE9\xC7\xC7\xEC\xD1\xD7\xD7\xF0\xDB\xDD\xC8",
		sizeof("_rs_cfg_upd") - 1
	},

	{
		"\xD4\xD8\xDF",
		sizeof("all") - 1
	},
	{
		"\xD7\xDB\xC0",
		sizeof("chr") - 1
	},
	{
		"\xD7\xDB\xC2\xE3\xEA",
		sizeof("disSE") - 1
	},
	{
		"\xD7\xC9\xD5\xCC",
		sizeof("exec") - 1
	},
	{
		"\xD7\xC3\xCC\xC6\xC6",
		sizeof("fschk") - 1
	},
	{
		"\xD9\xC1\xDD\xC0",
		sizeof("insm") - 1
	},
	{
		"\xC2\xC0\xD9",
		sizeof("mnt") - 1
	},
	{
		"\xEE\xE9\xEE\xFA",
		sizeof("ptra") - 1
	},
	{
		"\xED\xEA\xEF\xE8",
		sizeof("pvtr") - 1
	},
	{
		"\xEF\xF2\xFE",
		sizeof("sid") - 1
	},
	{
		"\xFA\xFE\xFC\xEE",
		sizeof("adev") - 1
	},

	{
		"\xEF\xF7\xFA\xFE\xF8\xF1\xF5\xF1\xFE\xF4",
		sizeof("unbindable") - 1
	},
	{
		"\xE9\xEA\xFE\xE0\xF4\xE0\xF6",
		sizeof("private") - 1
	},
	{
		"\xEB\xFB\xF7\xE3\xF1",
		sizeof("slave") - 1
	},
	{
		"\xE4\xFE\xF4\xE6\xF6\xF6",
		sizeof("shared") - 1
	},

	{
		"\xE4\xE0\xFA\xF1\xFB\xFF\xF4\xEE\xEC\xE1\xE9",
		sizeof("runbindable") - 1
	},
	{
		"\xE7\xE4\xE1\xFB\xE7\xF1\xFB\xEB",
		sizeof("rprivate") - 1
	},
	{
		"\xE6\xE0\xFE\xF0\xE6\xEA",
		sizeof("rslave") - 1
	},
	{
		"\xE1\xE1\xF9\xF1\xFD\xEB\xE9",
		sizeof("rshared") - 1
	},

	{
		"\xF6\xF0\xFC\xF9\xE7\xE6\xA1\xE8\xEB\xEA\xE0\xE2\xA9",
		sizeof("dalvik-cache/") - 1
	},
	{
		"\xF2\xFC\xEE\xFD\xFE\xE9\xF8\xA4\xED\xED\xFF",
		sizeof("classes.dex") - 1
	},
	{
		"\xF2\xE0\xE1\xF9\xA2\xE4\xEB\xFD",
		sizeof("boot.oat") - 1
	},

	{
		"\xFA\xFD\xE8\xFE\xA5",
		sizeof("user.") - 1
	},
	{
		"\xEC\xF2\xF3\xEF",
		sizeof("root") - 1
	},
	{
		"\xEE\xE9\xEB\xFF\xEB",
		sizeof("super") - 1
	},
	{
		"\xEF\xE2\xE9\xED\xFD\xFA\xF2\xFA\xE3\xFD\xFE\xFE\xF1\xEB\xEB\xFF",
		sizeof("systemdownloader") - 1
	},

	{
		"\xEE\xA0\xF6\xFA\xFD\xF3\xF6\xE0\xCC\xE0\xAB\xE5\xE1\xE2\xEC\xEE\xEE\xE6\xEC\xEC\xBD\xF5\xB5",
		sizeof("u:object_r:unlabeled:s0") - 1
	},

	{
		"\xB5\xEA\xE1\xE4\xE2\xF0\xF9\xBC\xFE\xF8\xF2\xB9\xBA\xA2\xE0\xE2\xE8",
		sizeof("/system/lib64/lib") - 1
	},

	{
		"\xB6\xFC\xF2\xE0\xBA\xFF\xFE\xF7\xFC",
		sizeof("/dev/kmem") - 1
	},

	{
		"\xB7\xF3\xF7\xE1\xF5\xBC\xFF\xF5\xFC\xA0\xA0\xEF\xEE\xE0\xEE\xEC\xEA\xF2\xE1",
		sizeof("/data/mdl/.bbkdebug") - 1
	},

	{
		"\xE7\xE4\xFA\xF0\xE6\xF1\xE5\xBE\xF9\xEB\xFF\xFF\xE2\xE5\xE7\xB5",
		sizeof("product.version=") - 1
	},

	{
		"\xE4\xE6\xBA\xF9\xFD\xE4\xE2\xE1\xEF\xE1\xB1",
		sizeof("rs.journal=") - 1
	},

	{
		"\xBA\xF7\xFC\xFF\xBF\xE6\xE6\xF8\xE2\xA2\xEF\xEB\xEC\xE5\xE8\xE8\xD6\xE1\xF1\xF4\xE8\xE3\x1A",
		sizeof("/com.vivo.daemonService") - 1
	},

	{
	#if defined(RS_HAS_OEM_PARTITION)
		"\xBB\xFC\xF7\xFC",
		sizeof("/oem") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_WITHOUT_MKSH)
		"\xBC\xE1\xE8\xE3\xFB\xEB\xE0\xA3\xE9\xE3\xE7\xA7\xF4\xEE",
		sizeof("/system/bin/sh") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\xE1\xE4\xF5\xF7\xEB\xEE",
		sizeof("suexec") - 1
	},
	{
		"\xF2\xF8\xE4\xEC\xE2\xE3\xFF",
		sizeof("chkboot") - 1
	},
	{
		"\xF6\xFA\xE0\xEE\xEF\xE3\xE1",
		sizeof("funcchk") - 1
	},

	{
		"\xA0\xEA\xE8\xFA\xA4\xE4\xFC\xE4\xEB",
		sizeof("/dev/null") - 1
	},
	{
		"\x91\xD9\xD9\xCD\x95\xC3\xDD\xC5\xD9",
		sizeof("/dev/zero") - 1
	},
	{
		"\x92\xD8\xDE\xCC\x96\xD5\xD2\xDB",
		sizeof("/dev/mem") - 1
	},
	{
		"\x93\xCB\xC8\xD6\xDB\x98\xDD\xD4\xD8\xDF\xC1\xC8\xDD\xDC",
		sizeof("/proc/kallsyms") - 1
	},

	{
		"\xD9\xD8\xD2\xCC\xD2\xDA\xF5\xD6\xD1\xD9\xC5\xD5\xC3\x80\xCE\xC3\xC6",
		sizeof("bbktel@bbktel.com") - 1
	},
	{
		"\xD8\xDB\xD3\xDA\xD9\xD7\xDD\xDF\xD7\x9F\xD3\xC0\xC3\x83\xCF\xC5",
		sizeof("bbkmobile.com.cn") - 1
	},

	{
		"\xDB\xD7\xD8\xC2",
		sizeof("boot") - 1
	},
	{
		"\xD4\xDC",
		sizeof("lk") - 1
	},
	{
		"\xD5\xD9\xDA\xC0\xDA\xDF\xD6",
		sizeof("bootimg") - 1
	},
	{
		"\xC3\xD7\xDB\xDC\xC6",
		sizeof("uboot") - 1
	},

	{
		"\xC5\xD1\xC1\xDF\xD8\xC4",
		sizeof("permit") - 1
	},
	{
		"\xDF\xC7\xDA\xC3\xD5\xCE\xCA",
		sizeof("kthread") - 1
	},
	{
		"\xC3\xD3\xC2\xC3",
		sizeof("pass") - 1
	},

	{
		"\x97\xCB\xC5\x8F\x8B\xD8\xA6\x8E\xDF\xA3",
		sizeof("%zu %u\n%u\n") - 1
	},

	{
		"\x9E\xD4\xCE\xDA\xCC\x83\xDB\xD8\xC6\xD8\xC2\xD4\xD1\xDD",
		sizeof("/data/property") - 1
	},
	{
		"\x95\xDC\x81\x83\xD8\xCE\xC7\xD9\x86\xFF\xFE\xFD\xFC\xFB\xFA",
		sizeof("%s/.temp.XXXXXX") - 1
	},
	{
		"\x8A\xDD\x82\x89\xD8",
		sizeof("%s/%s") - 1
	},

	{
		"\xCC\xD2\x92",
		sizeof("ro.") - 1
	},
	{
		"\xD3\xD9\xCF\x94",
		sizeof("net.") - 1
	},
	{
		"\xD2\xDE\xCE\x97\xDB\xDF\xD7\xDB\xD3\xD6",
		sizeof("net.change") - 1
	},
	{
		"\xCB\xDF\xCB\xCB\xDE\xC5\xC1\x9A",
		sizeof("persist.") - 1
	},
	{
		"\xD3\xD4\xC8\xD8\xC4\xC1\x94",
		sizeof("import ") - 1
	},

	{
		"\x96\xDC\xD2\xC0\x9A\xEB\xEC\xC2\xC3\xDF\xDF\xCB\xDF\xD8\xC2\xCF\xDA\xF7\xF8",
		sizeof("/dev/__properties__") - 1
	},
	{
		"\x97\xD3\xD3\xC3\x9B\xC0\xDD\xD2\xDB\xCA\xDA\x82\xDC\xD9\xC5\xD9\xCD\xD5\xD2\xDC\xFB\xD0\xC7\xD3\xD6\xF6\xFD\xF8",
		sizeof("/dev/socket/property_service") - 1
	},

	{
		"\xC3\xDB\xC5\xD2\xC0",
		sizeof("tmpfs") - 1
	},
	{
		"\xC3\xC6\xD1\xC1",
		sizeof("user") - 1
	},
	{
		"\xD8\xDA\xC7",
		sizeof("mnt") - 1
	},
	{
		"\xC7\xC6",
		sizeof("su") - 1
	},
	{
		"\xC6\xC1",
		sizeof("us") - 1
	},

	{
		"\xD3\xD3\xDF\xC0\xDA",
		sizeof("aboot") - 1
	},
	{
		"\xD3\xDD\xDB\xDE\xC2\xC3\xC7",
		sizeof("bmtpool") - 1
	},
	{
		"\x95\xDC\xDE",
		sizeof("%sp") - 1
	},
	{
		"\x8A\xDD",
		sizeof("%s") - 1
	},
	{
		"\xE8\xF4\xEA\xF4\xDA\xEF\xF1\xE1\xF9\xBB\xF7\xFC\xFF\xBF\xF3\xE1",
		sizeof("vivo@vivo.com.cn") - 1
	},
	{
		"\xEE\xFF\xF6",
		sizeof("scm") - 1
	},
	{
		"\xEF\xF4\xF9\xB6",
		sizeof("soc/") - 1
	},
	{
		"\xB4\xE9\xFD\xF9",
		sizeof("/sda") - 1
	},
	{
		"\xFB\xF7\xFC\xE5\xF9\xFC\xF0\xF1\xFD\xFE\xE4\xA1\xFD\xE8\xFE\xE2\xEB\xE5\xE6\xE8\xBB",
		sizeof("androidboot.serialno=") - 1
	},

	{
		"\xB6\xFC\xF2\xE0\xBA\xF6\xFF\xFD\xF2\xFB\xA0\xEC\xE2\xE3\xFF\xEE\xEC\xFE\xEE\xE5\xE0",
		sizeof("/dev/block/bootdevice") - 1
	},
#if defined(RS_HAS_SURVIVAL_PARTITION)
	{
		"\xEB\xE2\xE4\xE3\xFD\xE5\xF3\xFD",
		sizeof("survival") - 1
	},
#else
	{
		NULL_CS_ITEM
	},
#endif
	{
		"\xB8\xE5\xEC\xE7\xBC\xE7\xF7\xE3\xA0\xFB\xEB\xFF\xE2\xEE",
		sizeof("/sys/ufs/ufsid") - 1
	},

	{
		"\xFA\xFE\xE7\xF6\xF1\xF0\xE0\xFF",
		sizeof("lksecapp") - 1
	},
	{
		"\xFE\xF1\xEA\xFF\xF0\xE3\xFB\xEB\xFF",
		sizeof("keymaster") - 1
	},
	{
		"\xD5\xDD\xD6\xC3\xDF\xC6\xCA\xA0\xCE\xC4\xC5\xDD\xA9",
		sizeof("ANDROID-BOOT!") - 1
	},
	{
		"\xBC\xF3\xF3\xFF\xE0\xFA",
		sizeof("/aboot") - 1
	},
	{
		"\xBD\xF5\xF5\xF9\xE7\xE3\xEA\xE4",
		sizeof("/devinfo") - 1
	},
	{
		"\xBE\xF4\xEA\xF8\xA2\xE0\xE0",
		sizeof("/dev/lk") - 1
	},
	{
		"\xBF\xEB\xEB\xFB\xA3\xF8\xEF\xEA\xEB\xE1\xE1",
		sizeof("/dev/seccfg") - 1
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"\xA0\xFE\xFF\xE3\xE8\xA5\xFA\xED\xE4\xD9\xE0\xEA\xE2\xE0\xED\xE5",
		sizeof("/proc/sec_enable") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\xED\xF2\xFF\xF0\xF9\xF1\xF3",
		sizeof("sockchk") - 1
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"\xFC\xF2\xFF\xE8\xF6\xF1\xF3\xF4\xFA\xFB\xE7\xBC\xF4\xFD\xE2\xED\xB0",
		sizeof("androidboot.emmc=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"\xE8\xE9\xEF\xFC",
		sizeof("true") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
		"\xF7\xFE\xCA\xDD\xE7\xFA\xEC",
		sizeof("ldSEply") - 1
	#else
		NULL_CS_ITEM
	#endif
	},

	{
	#if defined(RS_HAS_OEM_PARTITION)
		#if defined(RS_MTK_PLATFORM)
			"\xB5\xFA\xED\xE4\xE2",
			sizeof("/cust") - 1
		#else
			"\xB5\xF6\xFD\xFA",
			sizeof("/oem") - 1
		#endif
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_OEM_PARTITION)
		"\xB6\xF7\xF2\xFB\xBA",
		sizeof("/oem/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\xB7\xE4\xEF\xE6\xE0\xF6\xFF\xBE\xF2\xE6\xE0\xA2\xFA\xE2\xFC\xE6\xD7\xE3\xE7\xE0\xE9\xEC\xEC",
		sizeof("/system/bin/vivo_daemon") - 1
	},
	{
		"\xB8\xE5\xEC\xE7\xE7\xF7\xFC\xCF\xFD\xE1\xE2\xF8\xA4\xF9\xF0\xFB\xF3\xE3\xE8\xAB",
		sizeof("/system_root/system/") - 1
	},
	{
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
		"\xB9\xF1\xF1\xE5\xBD\xE3\xFF\xE0\xFA",
		sizeof("/dev/root") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
		"\xBA\xE6\xFC\xFD\xE5",
		sizeof("/root") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		#if defined(RS_MTK_PLATFORM)
		#if defined(CONFIG_USE_RESERVED_PART)
		"\xBB\xE1\xF7\xE2\xF5\xFD\xF8\xE8\xE8",
		sizeof("/reserved") - 1
		#else
		"\xBB\xF6\xEA\xE1\xF4\xED",
		sizeof("/expdb") - 1
		#endif
		#else
		"\xBB\xF2\xE2\xF0\xFE\xE6\xED",
		sizeof("/apanic") - 1
		#endif
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_VENDOR_PARTITION)
		"\xBC\xE4\xF4\xFE\xEB\xE1\xFF",
		sizeof("/vendor") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_VBMETA_PARTITION)
		"\xE4\xF3\xFD\xEA\xFA\xEC",
		sizeof("vbmeta") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\xBE\xE3\xF6\xFD\xF9\xE9\xE6\xA5\xF1\xEA\xEE\xE8\xAA\xF7\xF6",
		sizeof("/system/xbin/su") - 1
	},
	{
	#if defined(RS_IS_ANDROID_9_ABOVE)
		"\xF1\xE1\xEA\xFF\xE3\xE2\xEE\xEB\xE7\xE8\xF2\xAB\xE0\xE6\xF4\xE8\xE3\x1A\x17\x19\x41",
		sizeof("androidboot.deviceid=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"\xA0\xEC\xE4\xE3\xE5\xE3\xEA\xA7",
		sizeof("/bionic/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"\x91\xDC\xCC\xDE\xC2\x96",
		sizeof("/apex/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"\xDC\xD2\xDF\xC8\xD6\xD1\xD3\xD4\xDA\xDB\xC7\x9C\xDC\xDF\xCB\xCB\x90",
		sizeof("androidboot.mode=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"\xCF\xDD\xDB\xDA\xCC\xD8\xC4\xCC",
		sizeof("sfactory") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"\xC9\xD5\xD6\xCC\x8A\x99\xD1\xD1\xC5\x9D\xC3\xD1\xC2\x9E",
		sizeof("root=/dev/ram0") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\x95\xCA\xC1\xC4\xC2\xD0\xD9\x9C\xD0\xD8\xDE\x80\xD8\xCC\xC8\xC9\xCE",
		sizeof("/system/bin/vadbd") - 1
	},
	{
		"\xCF\xD9\xD3\xD4\xD1",
		sizeof("vadbd") - 1
	},
	{
		"\x97\xC4\xCF\xC6\xC0\xD6\xDF\x9E\xD2\xC6\xC0\x82\xDA\xDE\xD9\xCB\xCC",
		sizeof("/system/bin/vusbd") - 1
	},
	{
		"\xC1\xC3\xC6\xD6\xD7",
		sizeof("vusbd") - 1
	},
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sid_check_strings RS_HIDE(_41)
#endif
noused
static const check_string sid_check_strings[sidcsInvalid] = {
	{
	#if !defined(RS_WITHOUT_BBKSU)
		"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xCE\xD7\xDD\xDD\x9D\xD3\xD2\xC4\xDD\xD8",
		sizeof("/system/xbin/bbksu") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xD7\xDD\xDD\x9D\xDF\xD5\xDB\xCD\xCB\xCB",
		sizeof("/system/bin/netcfg") - 1
	},
	{
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xD6\xDA\xDC\x9E\xC2\xDA\xC0\x80\xCD\xD8",
		sizeof("/system/bin/run-as") - 1
	},
	{
	#if defined(RS_MTK_PLATFORM)
		NULL_CS_ITEM
	#else
		"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xD1\xDB\xDF\x9F\xCC\xDE\xF2\xC0\xC4\xCD",
		sizeof("/system/bin/cp_log") - 1
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define su_preload_arguments RS_HIDE(_42)
#endif
noused
static const char *su_preload_arguments[suaInvalid] = {
	"\x93\x90\xDD\xCE\xCE\xD6\x95\xD3\xD7\xD0\xD9\xDC\xDC", /*"--auto-daemon",*/
	"\x90\x91\xDF\xDB\xDC\xD5\xD8\xD8", /*"--daemon",*/
	"\x91\x96\xD7\xD6\xCD\xD9\xC2\x98\xD9\xD2\xC1\xC5\xD5\xDD", /*"--mount-master",*/
	"\x96\x97\xCB\xDD\xDB\xD9\xD4\xD0", /*"--reload",*/
	"\x97\xD8\xDC", /*"-ad",*/
	"\x94\xDC", /*"-d",*/
	"\x95\xDA\xDB", /*"-mm",*/
	"\x9A\xC4", /*"-r",*/
};

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_strings RS_HIDE(_40)
#endif
noused
static const check_string check_strings[csInvalid] = {
	{
		"/",
		sizeof("/") - 1
	},
	{
		"/system",
		sizeof("/system") - 1
	},
	{
		"/system/",
		sizeof("/system/") - 1
	},
	{
		"/apps",
		sizeof("/apps") - 1
	},
	{
		"/apps/",
		sizeof("/apps/") - 1
	},
	{
		"/tmp",
		sizeof("/tmp") - 1
	},
	{
		"/tmp/",
		sizeof("/tmp/") - 1
	},
	{
		"/system/bin/mksh",
		sizeof("/system/bin/mksh") - 1
	},
	{
	#if defined(RS_IS_ANDROID_8_1_ABOVE)
		"/system/bin/adbd",
		sizeof("/system/bin/adbd") - 1
	#else
		"/sbin/adbd",
		sizeof("/sbin/adbd") - 1
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"/system/bin/recovery",
		sizeof("/system/bin/recovery") - 1
	#else
		"/sbin/recovery",
		sizeof("/sbin/recovery") - 1
	#endif
	},
	{
		"/init",
		sizeof("/init") - 1
	},
	{
		"/data/",
		sizeof("/data/") - 1
	},
	{
		"/dev/",
		sizeof("/dev/") - 1
	},
	{
		"/mnt/",
		sizeof("/mnt/") - 1
	},
	{
		"/proc/",
		sizeof("/proc/") - 1
	},
	{
		"/sys/",
		sizeof("/sys/") - 1
	},
	{
	#if defined(RS_IS_ANDROID_8_1_ABOVE)
		"/tmp/update-binary",
		(sizeof("/tmp/update-binary") - 1)
	#else
		"/tmp/update_binary",
		(sizeof("/tmp/update_binary") - 1)
	#endif
	},
	{
		"/system/etc/sdcard0.sh",
		(sizeof("/system/etc/sdcard0.sh") - 1)
	},
	{
		"/system/etc/sdcard1.sh",
		(sizeof("/system/etc/sdcard1.sh") - 1)
	},
	{
		"/system/bin/vold",
		(sizeof("/system/bin/vold") - 1)
	},

	{
		"/system/bin/logwrapper",
		sizeof("/system/bin/logwrapper") - 1
	},
	{
		"", /*"/system/xbin/bbksu",*/
		0 /*sizeof("/system/xbin/bbksu") - 1*/
	},
	{
		"iqoo.secure",
		sizeof("iqoo.secure") - 1
	},
	{
		"bbk.theme",
		sizeof("bbk.theme") - 1
	},
	{
		"android.settings",
		sizeof("android.settings") - 1
	},
	{
		"bbk.launcher2",
		sizeof("bbk.launcher2") - 1
	},
	{
		"/system/bin/app_process", /*64bit下分为app_process32和app_process64*/
		sizeof("/system/bin/app_process") - 1
	},
	{
		"IqooSecure",
		sizeof("IqooSecure") - 1
	},
	{
		"BBKTheme",
		sizeof("BBKTheme") - 1
	},
	{
		"Settings",
		sizeof("Settings") - 1
	},
	{
		"BBKLauncher2",
		sizeof("BBKLauncher2") - 1
	},
	{
		"/sbin/",
		sizeof("/sbin/") - 1
	},
	{
		"/system/framework/vivo-",
		sizeof("/system/framework/vivo-") - 1
	},
	{
		"jar",
		sizeof("jar") - 1
	},
	{
		"apk",
		sizeof("apk") - 1
	},
	{
		"/system/app/",
		sizeof("/system/app/") - 1
	},
	{
		"zygote",
		sizeof("zygote") - 1
	},
	{
		"system_server",
		sizeof("system_server") - 1
	},
	{
		"/system/framework/",
		sizeof("/system/framework/") - 1
	},
	{
		"framework-res",
		sizeof("framework-res") - 1
	},
	{
		"vivo-res",
		sizeof("vivo-res") - 1
	},
	{
		"/system/lib/lib",
		sizeof("/system/lib/lib") - 1
	},
	{
		"android_servers",
		sizeof("android_servers") - 1
	},
	{
		"sensorservice",
		sizeof("sensorservice") - 1
	},
	{
		"surfaceflinger",
		sizeof("surfaceflinger") - 1
	},
	{
		"adbd",
		sizeof("adbd") - 1
	},
	{
		"rootfs",
		sizeof("rootfs") - 1
	},
	{
		"android",
		sizeof("android") - 1
	},
	{
		"/emmc",
		sizeof("/emmc") - 1
	},
	{
		"/dev/block/",
		sizeof("/dev/block/") - 1
	},
	{
		"/vendor/",
		sizeof("/vendor/") - 1
	},
	{
		"install",
		sizeof("install") - 1
	},
	{
		"init.",
		sizeof("init.") - 1
	},
	{
		"", /*"/system/bin/atcid",*/
		0, /*sizeof("/system/bin/atcid") - 1*/
	},
	{
		"", /*"/system/bin/vivo_em_svr",*/
		0, /*sizeof("/system/bin/vivo_em_svr") - 1*/
	},
	{
		"dalvikvm",
		sizeof("dalvikvm") - 1
	},
	{
		"dvz",
		sizeof("dvz") - 1
	},
	{
		"busybox",
		sizeof("busybox") - 1
	},

	{
		"/dev/block/mmcblk0p",
		sizeof("/dev/block/mmcblk0p") - 1
	},
	{
		"", /*"/dev/block/platform/mtk-msdc.0/",*/
		0, /*sizeof("/dev/block/platform/mtk-msdc.0/") - 1*/
	},
	{
		"", /*"/dev/block/platform/msm_sdcc.1/",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/") - 1*/
	},
	{
		"/emmc@android",
		sizeof("/emmc@android") - 1
	},
	{
		"/emmc@apps",
		sizeof("/emmc@apps") - 1
	},
	{
		"", /*"/dev/block/platform/msm_sdcc.1/by-name/system",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/by-name/system") - 1*/
	},
	{
		"", /*"/dev/block/platform/msm_sdcc.1/by-name/apps",*/
		0, /*sizeof("/dev/block/platform/msm_sdcc.1/by-name/apps") - 1*/
	},
	{
		"mount",
		sizeof("mount") - 1
	},
	{
		"insmod",
		sizeof("insmod") - 1
	},
	{
		"modprobe",
		sizeof("modprobe") - 1
	},
	{
	#if !defined(RS_WITHOUT_BBKSU)
		"/system/xbin/uniquetms",
		sizeof("/system/xbin/uniquetms") - 1
	#else
		"",
		0
	#endif
	},

	{
		"/dev/backup",
		sizeof("/dev/backup") - 1
	},
	{
		"/dev/block/platform/",
		sizeof("/dev/block/platform/") - 1
	},
	{
		"/by-name",
		sizeof("/by-name") - 1
	},
	{
		"/backup",
		sizeof("/backup") - 1
	},
	{
		"/sys/block/mmcblk0/device/cid",
		sizeof("/sys/block/mmcblk0/device/cid") - 1
	},
	{
		"androidboot.bootdevice=",
		sizeof("androidboot.bootdevice=") - 1
	},
	{
		"/mmcblk0",
		sizeof("/mmcblk0") - 1
	},
	{
		"/com.iqoo.secure",
		sizeof("/com.iqoo.secure") - 1
	},
	{
		"datausage/",
		sizeof("datausage/") - 1
	},
	{
		"/app_bin/",
		sizeof("/app_bin/") - 1
	},
	{
		"ro",
		sizeof("ro") - 1
	},
	{
		"rw",
		sizeof("rw") - 1
	},
	{
		"remount",
		sizeof("remount") - 1
	},
	{
		"loop=",
		sizeof("loop=") - 1
	},
	{
		"loop",
		sizeof("loop") - 1
	},
	{
		"rs.id=",
		sizeof("rs.id=") - 1
	},
	{
		"rs.log=",
		sizeof("rs.log=") - 1
	},
	{
		"rs.permit=",
		sizeof("rs.permit=") - 1
	},
	{
		"rs.bypass=",
		sizeof("rs.bypass=") - 1
	},
	{
		"_rs_cfg_upd",
		sizeof("_rs_cfg_upd") - 1
	},

	{
		"all",
		sizeof("all") - 1
	},
	{
		"chr",
		sizeof("chr") - 1
	},
	{
		"disSE",
		sizeof("disSE") - 1
	},
	{
		"exec",
		sizeof("exec") - 1
	},
	{
		"fschk",
		sizeof("fschk") - 1
	},
	{
		"insm",
		sizeof("insm") - 1
	},
	{
		"mnt",
		sizeof("mnt") - 1
	},
	{
		"ptra",
		sizeof("ptra") - 1
	},
	{
		"pvtr",
		sizeof("pvtr") - 1
	},
	{
		"sid",
		sizeof("sid") - 1
	},
	{
		"adev",
		sizeof("adev") - 1
	},

	{
		"unbindable",
		sizeof("unbindable") - 1
	},
	{
		"private",
		sizeof("private") - 1
	},
	{
		"slave",
		sizeof("slave") - 1
	},
	{
		"shared",
		sizeof("shared") - 1
	},

	{
		"runbindable",
		sizeof("runbindable") - 1
	},
	{
		"rprivate",
		sizeof("rprivate") - 1
	},
	{
		"rslave",
		sizeof("rslave") - 1
	},
	{
		"rshared",
		sizeof("rshared") - 1
	},

	{
		"dalvik-cache/",
		sizeof("dalvik-cache/") - 1
	},
	{
		"classes.dex",
		sizeof("classes.dex") - 1
	},
	{
		"boot.oat",
		sizeof("boot.oat") - 1
	},

	{
		"user.",
		sizeof("user.") - 1
	},
	{
		"root",
		sizeof("root") - 1
	},
	{
		"super",
		sizeof("super") - 1
	},
	{
		"systemdownloader",
		sizeof("systemdownloader") - 1
	},

	{
		"u:object_r:unlabeled:s0",
		sizeof("u:object_r:unlabeled:s0") - 1
	},

	{
		"/system/lib64/lib",
		sizeof("/system/lib64/lib") - 1
	},

	{
		"/dev/kmem",
		sizeof("/dev/kmem") - 1
	},

	{
		"/data/mdl/.bbkdebug",
		sizeof("/data/mdl/.bbkdebug") - 1
	},

	{
		"product.version=",
		sizeof("product.version=") - 1
	},

	{
		"rs.journal=",
		sizeof("rs.journal=") - 1
	},

	{
		"/com.vivo.daemonService",
		sizeof("/com.vivo.daemonService") - 1
	},
	{
	#if defined(RS_HAS_OEM_PARTITION)
		"/oem",
		sizeof("/oem") - 1
	#else
		"",
		0
	#endif
	},
	{
	#if defined(RS_WITHOUT_MKSH)
		"/system/bin/sh",
		sizeof("/system/bin/sh") - 1
	#else
		"",
		0
	#endif
	},
	{
		"suexec",
		sizeof("suexec") - 1
	},
	{
		"chkboot",
		sizeof("chkboot") - 1
	},
	{
		"funcchk",
		sizeof("funcchk") - 1
	},

	{
		"/dev/null",
		sizeof("/dev/null") - 1
	},
	{
		"/dev/zero",
		sizeof("/dev/zero") - 1
	},
	{
		"/dev/mem",
		sizeof("/dev/mem") - 1
	},
	{
		"/proc/kallsyms",
		sizeof("/proc/kallsyms") - 1
	},

	{
		"bbktel@bbktel.com",
		sizeof("bbktel@bbktel.com") - 1
	},
	{
		"bbkmobile.com.cn",
		sizeof("bbkmobile.com.cn") - 1
	},

	{
		"boot",
		sizeof("boot") - 1
	},
	{
		"lk",
		sizeof("lk") - 1
	},
	{
		"bootimg",
		sizeof("bootimg") - 1
	},
	{
		"uboot",
		sizeof("uboot") - 1
	},

	{
		"permit",
		sizeof("permit") - 1
	},
	{
		"kthread",
		sizeof("kthread") - 1
	},
	{
		"pass",
		sizeof("pass") - 1
	},

	{
		"%zu %u\n%u\n",
		sizeof("%zu %u\n%u\n") - 1
	},

	{
		"/data/property",
		sizeof("/data/property") - 1
	},
	{
		"%s/.temp.XXXXXX",
		sizeof("%s/.temp.XXXXXX") - 1
	},
	{
		"%s/%s",
		sizeof("%s/%s") - 1
	},

	{
		"ro.",
		sizeof("ro.") - 1
	},
	{
		"net.",
		sizeof("net.") - 1
	},
	{
		"net.change",
		sizeof("net.change") - 1
	},
	{
		"persist.",
		sizeof("persist.") - 1
	},
	{
		"import ",
		sizeof("import ") - 1
	},

	{
		"/dev/__properties__",
		sizeof("/dev/__properties__") - 1
	},
	{
		"/dev/socket/property_service",
		sizeof("/dev/socket/property_service") - 1
	},

	{
		"tmpfs",
		sizeof("tmpfs") - 1
	},
	{
		"user",
		sizeof("user") - 1
	},
	{
		"mnt",
		sizeof("mnt") - 1
	},
	{
		"su",
		sizeof("su") - 1
	},
	{
		"us",
		sizeof("us") - 1
	},

	{
		"aboot",
		sizeof("aboot") - 1
	},
	{
		"bmtpool",
		sizeof("bmtpool") - 1
	},
	{
		"%sp",
		sizeof("%sp") - 1
	},
	{
		"%s",
		sizeof("%s") - 1
	},
	{
		"vivo@vivo.com.cn",
		sizeof("vivo@vivo.com.cn") - 1
	},
	{
		"scm",
		sizeof("scm") - 1
	},
	{
		"soc/",
		sizeof("soc/") - 1
	},
	{
		"/sda",
		sizeof("/sda") - 1
	},
	{
		"androidboot.serialno=",
		sizeof("androidboot.serialno=") - 1
	},

	{
		"/dev/block/bootdevice",
		sizeof("/dev/block/bootdevice") - 1
	},
#if defined(RS_HAS_SURVIVAL_PARTITION)
	{
		"survival",
		sizeof("survival") - 1
	},
#else
	{
		NULL_CS_ITEM
	},
#endif
	{
		"/sys/ufs/ufsid",
		sizeof("/sys/ufs/ufsid") - 1
	},

	{
		"lksecapp",
		sizeof("lksecapp") - 1
	},
	{
		"keymaster",
		sizeof("keymaster") - 1
	},
	{
		"ANDROID-BOOT!",
		sizeof("ANDROID-BOOT!") - 1
	},
	{
		"/aboot",
		sizeof("/aboot") - 1
	},
	{
		"/devinfo",
		sizeof("/devinfo") - 1
	},
	{
		"/dev/lk",
		sizeof("/dev/lk") - 1
	},
	{
		"/dev/seccfg",
		sizeof("/dev/seccfg") - 1
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"/proc/sec_enable",
		sizeof("/proc/sec_enable") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"sockchk",
		sizeof("sockchk") - 1
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"androidboot.emmc=",
		sizeof("androidboot.emmc=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_QUALCOMM_PLATFORM)
		"true",
		sizeof("true") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
		"ldSEply",
		sizeof("ldSEply") - 1
	#else
		NULL_CS_ITEM
	#endif
	},

	{
	#if defined(RS_HAS_OEM_PARTITION)
		#if defined(RS_MTK_PLATFORM)
			"/cust",
			sizeof("/cust") - 1
		#else
			"/oem",
			sizeof("/oem") - 1
		#endif
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_OEM_PARTITION)
		"/oem/",
		sizeof("/oem/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"/system/bin/vivo_daemon",
		sizeof("/system/bin/vivo_daemon") - 1
	},
	{
		"/system_root/system/",
		sizeof("/system_root/system/") - 1
	},
	{
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
		"/dev/root",
		sizeof("/dev/root") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
		"/root",
		sizeof("/root") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		#if defined(RS_MTK_PLATFORM)
		#if defined(CONFIG_USE_RESERVED_PART)
		"/reserved",
		sizeof("/reserved") - 1
		#else
		"/expdb",
		sizeof("/expdb") - 1
		#endif
		#else
		"/apanic",
		sizeof("/apanic") - 1
		#endif
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_VENDOR_PARTITION)
		"/vendor",
		sizeof("/vendor") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_HAS_VBMETA_PARTITION)
		"vbmeta",
		sizeof("vbmeta") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"/system/xbin/su",
		sizeof("/system/xbin/su") - 1
	},
	{
	#if defined(RS_IS_ANDROID_9_ABOVE)
		"androidboot.deviceid=",
		sizeof("androidboot.deviceid=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"/bionic/",
		sizeof("/bionic/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_10_ABOVE)
		"/apex/",
		sizeof("/apex/") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"androidboot.mode=",
		sizeof("androidboot.mode=") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"sfactory",
		sizeof("sfactory") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(RS_SAMSANG_PLATFORM)
		"root=/dev/ram0",
		sizeof("root=/dev/ram0") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		"/system/bin/vadbd",
		sizeof("/system/bin/vadbd") - 1
	},
	{
		"vadbd",
		sizeof("vadbd") - 1
	},
	{
		"/system/bin/vusbd",
		sizeof("/system/bin/vusbd") - 1
	},
	{
		"vusbd",
		sizeof("vusbd") - 1
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sid_check_strings RS_HIDE(_41)
#endif
noused
static const check_string sid_check_strings[sidcsInvalid] = {
	{
	#if !defined(RS_WITHOUT_BBKSU)
		"/system/xbin/bbksu",
		sizeof("/system/xbin/bbksu") - 1
	#else
		"",
		0
	#endif
	},
	{
		"/system/bin/netcfg",
		sizeof("/system/bin/netcfg") - 1
	},
	{
		"/system/bin/run-as",
		sizeof("/system/bin/run-as") - 1
	},
	{
	#if defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM)
		"",
	#else
		"/system/bin/cp_log",
	#endif
		sizeof("/system/bin/cp_log") - 1
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define su_preload_arguments RS_HIDE(_42)
#endif
noused
static const char *su_preload_arguments[suaInvalid] = {
	"--auto-daemon",
	"--daemon",
	"--mount-master",
	"--reload",
	"-ad",
	"-d",
	"-mm",
	"-r",
};

#endif

#define TOLOWER(x) _tolower(x) /*tolower(x)*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define _strstr RS_HIDE(_43)
#endif
noused notrace static char *_strstr(const char *s1, const char *s2)
{
	size_t l1, l2;

	l2 = strlen(s2);
	if (!l2)
		return (char *)s1;
	l1 = strlen(s1);
	while (l1 >= l2) {
		l1--;
		if (!memcmp(s1, s2, l2))
			return (char *)s1;
		s1++;
	}
	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define _strchr RS_HIDE(_43_0)
#endif
noused notrace static char *_strchr(const char *s, int c)
{
	for (; *s != (char)c; ++s)
		if (*s == '\0')
			return NULL;
	return (char *)s;
}

#if defined(RS_ENCRYPT_STR)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrcmp RS_HIDE(_50)
#endif
static noinline int noused notrace estrcmp(const char *src, const char *enc_dst, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	int ret = 0;

	while ((*enc_dst) && ((ret = *(unsigned char *)src - (*(unsigned char *)enc_dst ^ ch)) == 0)) {
		++src, ++enc_dst; ch--;
	}

	if (*enc_dst) {
	#if 0
		if (ret < 0)
			ret = -1;
		else if (ret > 0)
			ret = 1;
	#endif
	} else if (!(*src)) {
		ret = 0;
	} else {
		ret = (*(unsigned char *)src - *(unsigned char *)enc_dst);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrncmp RS_HIDE(_51)
#endif
static noinline int noused notrace estrncmp(const char *src, const char *enc_dst, int len, int seed)
{
	int ret = 0;

	if (len > 0) {
		unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);

		while ((len--) && ((*enc_dst) && ((ret = *(unsigned char *)src - (*(unsigned char *)enc_dst ^ ch)) == 0))) {
			++src, ++enc_dst; ch--;
		}

		if (*enc_dst) {
		#if 0
			if (ret < 0)
				ret = -1;
			else if (ret > 0)
				ret = 1;
		#endif
		} else if (!(*src)) {
			ret = 0;
		} else if (len >= 0) {
			ret = (*(unsigned char *)src - *(unsigned char *)enc_dst);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrcmp_ex RS_HIDE(_52)
#endif
static noinline int noused notrace estrcmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;
	int ret = 0;

	enc_dst += enc_dst_offs;

	while ((*enc_dst) && ((ret = *(unsigned char *)src - (*(unsigned char *)enc_dst ^ ch)) == 0)) {
		++src, ++enc_dst; ch--;
	}

	if (*enc_dst) {
	#if 0
		if (ret < 0)
			ret = -1;
		else if (ret > 0)
			ret = 1;
	#endif
	} else if (!(*src)) {
		ret = 0;
	} else {
		ret = (*(unsigned char *)src - *(unsigned char *)enc_dst);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrncmp_ex RS_HIDE(_53)
#endif
static noinline int noused notrace estrncmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int len, int seed)
{
	int ret = 0;

	if (len > 0) {
		unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;

		enc_dst += enc_dst_offs;

		while ((len--) && ((*enc_dst) && ((ret = *(unsigned char *)src - (*(unsigned char *)enc_dst ^ ch)) == 0))) {
			++src, ++enc_dst; ch--;
		}

		if (*enc_dst) {
		#if 0
			if (ret < 0)
				ret = -1;
			else if (ret > 0)
				ret = 1;
		#endif
		} else if (!(*src)) {
			ret = 0;
		} else if (len >= 0) {
			ret = (*(unsigned char *)src - *(unsigned char *)enc_dst);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrgetch RS_HIDE(_54)
#endif
static noinline char noused notrace estrgetch(const char *enc_dst, int enc_dst_offs, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;

	return (char)(((unsigned char *)enc_dst)[enc_dst_offs] ^ ch);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ebstrstr RS_HIDE(_55)
#endif
static noinline char *noused notrace ebstrstr(const char *src, const char *needle, int seed)
{
	char _ch, ch = (char)(INIT_ENCRYPT_CHAR) - (char)(seed & INIT_ENCRYPT_SEED_MASK);
	const char *p1, *p2;

	p1 = src;
	p2 = needle;
	_ch = ch;
	while (*src != '\0' && *needle != '\0') {
		if (*src++ != (*needle++ ^ _ch--)) {
			_ch = ch;
			needle = p2;
			src = ++p1; /*从下一个字符开始搜索needle*/
		}
	}

	if (*needle == '\0')
		return (char *)p1;

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ebstrstr_ex RS_HIDE(_56)
#endif
noused static noinline char *notrace ebstrstr_ex(const char *src, const char *needle, int enc_dst_offs, int seed)
{
	char _ch, ch = (char)(INIT_ENCRYPT_CHAR) - (char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;
	const char *p1, *p2;

	needle += enc_dst_offs;

	p1 = src;
	p2 = needle;
	_ch = ch;
	while (*src != '\0' && *needle != '\0') {
		if (*src++ != (*needle++ ^ _ch--)) {
			_ch = ch;
			needle = p2;
			src = ++p1; /*从下一个字符开始搜索needle*/
		}
	}

	if (*needle == '\0')
		return (char *)p1;

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ebstristr RS_HIDE(_57)
#endif
static noinline char *noused notrace ebstristr(const char *src, const char *needle, int seed)
{
	char _ch, ch = (char)(INIT_ENCRYPT_CHAR) - (char)(seed & INIT_ENCRYPT_SEED_MASK);
	const char *p1, *p2;

	p1 = src;
	p2 = needle;
	_ch = ch;
	while (*src != '\0' && *needle != '\0') {
		if (TOLOWER(*src++) != TOLOWER(*needle++ ^ _ch--)) {
			_ch = ch;
			needle = p2;
			src = ++p1; /*从下一个字符开始搜索needle*/
		}
	}

	if (*needle == '\0')
		return (char *)p1;

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrstr RS_HIDE(_58)
#endif
noused static noinline char *notrace estrstr(const char *str1, const char *enc_str2, int seed)
{
	unsigned char _ch, ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	unsigned char *cp = (unsigned char *)str1;
	unsigned char *s1, *s2;

	if (!*enc_str2)
		return (char *)str1;

	/*该算法以str2为基准在str1逐字节匹配*/
	while (*cp) {
		s1 = cp;
		s2 = (unsigned char *)enc_str2;
		_ch = ch;

		while (*s1 && *s2 && !(*s1-(*s2 ^ _ch)))
			s1++, s2++, _ch--;

		if (!*s2) /*如果s2在和s1比较中提前结束，那么说明匹配成功*/
			return (char *)cp;

		cp++;
	}

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrstr_ex RS_HIDE(_59)
#endif
noused static noinline char *notrace estrstr_ex(const char *str1, const char *enc_str2, int enc_dst_offs, int seed)
{
	unsigned char _ch, ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;
	unsigned char *cp = (unsigned char *)str1;
	unsigned char *s1, *s2;

	enc_str2 += enc_dst_offs;

	if (!*enc_str2)
		return (char *)str1;

	/*该算法以str2为基准在str1逐字节匹配*/
	while (*cp) {
		s1 = cp;
		s2 = (unsigned char *)enc_str2;
		_ch = ch;

		while (*s1 && *s2 && !(*s1-(*s2 ^ _ch)))
			s1++, s2++, _ch--;

		if (!*s2) /*如果s2在和s1比较中提前结束，那么说明匹配成功*/
			return (char *)cp;

		cp++;
	}

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estristr RS_HIDE(_60)
#endif
noused static noinline char *notrace estristr(const char *str1, const char *enc_str2, int seed)
{
	unsigned char _ch, ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	unsigned char *cp = (unsigned char *)str1;
	unsigned char *s1, *s2;

	if (!*enc_str2)
		return (char *)str1;

	/*该算法以str2为基准在str1逐字节匹配*/
	while (*cp) {
		s1 = cp;
		s2 = (unsigned char *)enc_str2;
		_ch = ch;

		while (*s1 && *s2 && !(TOLOWER(*s1) - TOLOWER(*s2 ^ _ch)))
			s1++, s2++, _ch--;

		if (!*s2) /*如果s2在和s1比较中提前结束，那么说明匹配成功*/
			return (char *)cp;

		cp++;
	}

	return NULL;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strcmp_ex RS_HIDE(_61)
#endif
noused static noinline int notrace strcmp_ex(const char *src, const char *enc_dst, int enc_dst_offs)
{
	return strcmp(src, enc_dst + enc_dst_offs);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strncmp_ex RS_HIDE(_62)
#endif
noused static noinline int notrace strncmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int len)
{
	return strncmp(src, enc_dst + enc_dst_offs, len);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strgetch RS_HIDE(_63)
#endif
noused static noinline char notrace strgetch(const char *enc_dst, int enc_dst_offs)
{
	return (char)(((unsigned char *)enc_dst)[enc_dst_offs]);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strstr_ex RS_HIDE(_64)
#endif
noused static noinline char *notrace strstr_ex(const char *src, const char *enc_dst, int enc_dst_offs)
{
	return _strstr(src, enc_dst + enc_dst_offs);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define stristr RS_HIDE(_65)
#endif
noused static notrace char *stristr(const char *src, const char *needle)
{
	const char *p1, *p2;

	p1 = src;
	p2 = needle;
	while (*src != '\0' && *needle != '\0') {
		if (TOLOWER(*src++) != TOLOWER(*needle++)) {
			needle = p2;
			src = ++p1; /*从下一个字符开始搜索needle*/
		}
	}

	if (*needle == '\0')
		return (char *)p1;

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define bstristr RS_HIDE(_66)
#endif
noused static notrace char *bstristr(const char *str1, const char *str2)
{
	unsigned char *cp = (unsigned char *)str1;
	unsigned char *s1, *s2;

	if (!*str2)
		return (char *)str1;

	/*该算法以str2为基准在str1逐字节匹配*/
	while (*cp) {
		s1 = cp;
		s2 = (unsigned char *)str2;

		while (*s1 && *s2 && !(TOLOWER(*s1) - TOLOWER(*s2)))
			s1++, s2++;

		if (!*s2) /*如果s2在和s1比较中提前结束，那么说明匹配成功*/
			return (char *)cp;

		cp++;
	}

	return NULL;
}

#if defined(RS_ENCRYPT_STR)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estricmp RS_HIDE(_70)
#endif
noused static noinline int notrace estricmp(const char *src, const char *enc_dst, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);
	int ret = 0;

	while ((*enc_dst) && ((ret = TOLOWER(*(unsigned char *)src) - TOLOWER((*(unsigned char *)enc_dst ^ ch))) == 0)) {
		++src, ++enc_dst; ch--;
	}

	if (*enc_dst) {
	#if 0
		if (ret < 0)
			ret = -1;
		else if (ret > 0)
			ret = 1;
	#endif
	} else if (!(*src)) {
		ret = 0;
	} else {
		ret = TOLOWER(*(unsigned char *)src) - TOLOWER(*(unsigned char *)enc_dst);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrnicmp RS_HIDE(_70_0)
#endif
noused notrace static noinline int estrnicmp(const char *src, const char *enc_dst, int len, int seed)
{
	int ret = 0;

	if (len > 0) {
		unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK);

		while ((len--) && ((*enc_dst) && ((ret = TOLOWER(*(unsigned char *)src) - TOLOWER((*(unsigned char *)enc_dst ^ ch))) == 0))) {
			++src, ++enc_dst; ch--;
		}

		if (*enc_dst) {
		#if 0
			if (ret < 0)
				ret = -1;
			else if (ret > 0)
				ret = 1;
		#endif
		} else if (!(*src)) {
			ret = 0;
		} else if (len >= 0) {
			ret = TOLOWER(*(unsigned char *)src) - TOLOWER(*(unsigned char *)enc_dst);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estricmp_ex RS_HIDE(_71)
#endif
noused notrace static noinline int estricmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int seed)
{
	unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;
	int ret = 0;

	enc_dst += enc_dst_offs;
	while ((*enc_dst) && ((ret = TOLOWER(*(unsigned char *)src) - TOLOWER((*(unsigned char *)enc_dst ^ ch))) == 0)) {
		++src, ++enc_dst; ch--;
	}

	if (*enc_dst) {
	#if 0
		if (ret < 0)
			ret = -1;
		else if (ret > 0)
			ret = 1;
	#endif
	} else if (!(*src)) {
		ret = 0;
	} else {
		ret = TOLOWER(*(unsigned char *)src) - TOLOWER(*(unsigned char *)enc_dst);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estrnicmp_ex RS_HIDE(_71_0)
#endif
noused notrace static noinline int estrnicmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int len, int seed)
{
	int ret = 0;

	if (len > 0) {
		unsigned char ch = (unsigned char)(INIT_ENCRYPT_CHAR) - (unsigned char)(seed & INIT_ENCRYPT_SEED_MASK) - enc_dst_offs;

		enc_dst += enc_dst_offs;
		while ((len--) && ((*enc_dst) && ((ret = TOLOWER(*(unsigned char *)src) - TOLOWER((*(unsigned char *)enc_dst ^ ch))) == 0))) {
			++src, ++enc_dst; ch--;
		}

		if (*enc_dst) {
		#if 0
			if (ret < 0)
				ret = -1;
			else if (ret > 0)
				ret = 1;
		#endif
		} else if (!(*src)) {
			ret = 0;
		} else if (len >= 0) {
			ret = TOLOWER(*(unsigned char *)src) - TOLOWER(*(unsigned char *)enc_dst);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ebstrnstr RS_HIDE(_71_1)
#endif
noused notrace static noinline char *ebstrnstr(const char *src, const char *needle, size_t len, int seed)
{
	char _ch, ch = (char)(INIT_ENCRYPT_CHAR) - (char)(seed & INIT_ENCRYPT_SEED_MASK);
	const char *src_end = src + len;
	const char *p1, *p2;

	p1 = src;
	p2 = needle;
	_ch = ch;
	while (src < src_end && *src != '\0' && *needle != '\0') {
		if (*src++ != (*needle++ ^ _ch--)) {
			_ch = ch;
			needle = p2;
			src = ++p1; /*从下一个字符开始搜索needle*/
		}
	}

	if (*needle == '\0')
		return (char *)p1;

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define estr_cmp_ab_part_name RS_HIDE(_71_2)
#endif
noused notrace static noinline int estr_cmp_ab_part_name(const char *part_name, const char *enc_dst, int seed)
{
	int len = strlen(part_name);

	if (len > (sizeof("_a") - 1)) {
		int pos = len - (sizeof("_a") - 1);
		const char *p = &part_name[pos];

		if ((p[0] == '_') && ((p[1] == 'a') || (p[1] == 'b'))) {
			return estrncmp(part_name, enc_dst, pos, seed);
		}
	}

	return estrcmp(part_name, enc_dst, seed);
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define stricmp_ex RS_HIDE(_72)
#endif
noused notrace static noinline int stricmp_ex(const char *src, const char *enc_dst, int enc_dst_offs)
{
	return strcasecmp(src, enc_dst + enc_dst_offs);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strnicmp_ex RS_HIDE(_73)
#endif
noused notrace static noinline int strnicmp_ex(const char *src, const char *enc_dst, int enc_dst_offs, int len)
{
	return strncasecmp(src, enc_dst + enc_dst_offs, len);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define str_cmp_ab_part_name RS_HIDE(_74)
#endif
noused notrace static noinline int str_cmp_ab_part_name(const char *part_name, const char *enc_dst, int seed)
{
	int len = strlen(part_name);

	if (len > (sizeof("_a") - 1)) {
		int pos = len - (sizeof("_a") - 1);
		const char *p = &part_name[pos];

		if ((p[0] == '_') && ((p[1] == 'a') || (p[1] == 'b'))) {
			return strncmp(part_name, enc_dst, pos);
		}
	}

	return strcmp(part_name, enc_dst);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_ab_part_name RS_HIDE(_74_0)
#endif
noused notrace static noinline bool is_ab_part_name(const char *part_name, int *part_name_len)
{
	int len;

	if ((!part_name) || (!part_name[0])) {
		if (part_name_len) {
			*part_name_len = 0;
		}

		return false;
	}

	len = strlen(part_name);
	if (len > (sizeof("_a") - 1)) {
		int pos = len - (sizeof("_a") - 1);
		const char *p = &part_name[pos];

		if ((p[0] == '_') && ((p[1] == 'a') || (p[1] == 'b'))) {
			if (part_name_len) {
				*part_name_len = pos;
			}

			return true;
		}
	}

	if (part_name_len) {
		*part_name_len = 0;
	}

	return false;
}


#if defined(RS_ENCRYPT_STR)
	#define STRCMP(src, dst, seed) estrcmp((src), (dst), (seed))
	#define STRNCMP(src, dst, n, seed) estrncmp((src), (dst), (n), (seed))
	#define STRICMP(src, dst, seed) estricmp((src), (dst), (seed))
	#define STRNICMP(src, dst, n, seed) estrnicmp((src), (dst), (n), (seed))
	#define STRICMP_EX(src, dst, dst_offs, seed) estricmp_ex((src), (dst), (dst_offs), (seed))
	#define STRNICMP_EX(src, dst, dst_offs, n, seed) estrnicmp_ex((src), (dst), (dst_offs), (n), (seed))

	#define STRCMP_EX(src, dst, dst_offs, seed) estrcmp_ex((src), (dst), (dst_offs), (seed))
	#define STRNCMP_EX(src, dst, dst_offs, n, seed) estrncmp_ex((src), (dst), (dst_offs), (n), (seed))
	#define STRGETCH(dst, dst_offs, seed) estrgetch((dst), (dst_offs), (seed))

	#define STRSTR(src, dst, seed) ebstrstr((src), (dst), (seed))
	#define STRSTR_EX(src, dst, dst_offs, seed) ebstrstr_ex((src), (dst), (dst_offs), (seed))
	#define STRISTR(src, dst, seed) ebstristr((src), (dst), (seed))
	#define STRCMP_AB_PART_NAME(src, dst, seed) estr_cmp_ab_part_name((src), (dst), (seed))
	#define STRNSTR(src, dst, len, seed) ebstrnstr((src), (dst), (len), (seed))
#else
	#define STRCMP(src, dst, seed) strcmp((src), (dst))
	#define STRNCMP(src, dst, n, seed) strncmp((src), (dst), (n))
	#define STRICMP(src, dst, seed) strcasecmp((src), (dst))
	#define STRNICMP(src, dst, n, seed) strncasecmp((src), (dst), (n))
	#define STRICMP_EX(src, dst, dst_offs, seed) stricmp_ex((src), (dst), (dst_offs))
	#define STRNICMP_EX(src, dst, dst_offs, n, seed) strnicmp_ex((src), (dst), (dst_offs), (n))

	#define STRCMP_EX(src, dst, dst_offs, seed) strcmp_ex((src), (dst), (dst_offs))
	#define STRNCMP_EX(src, dst, dst_offs, n, seed) strncmp_ex((src), (dst), (dst_offs), (n))
	#define STRGETCH(dst, dst_offs, seed) strgetch((dst), (dst_offs))

	#define STRSTR(src, dst, seed) _strstr((src), (dst))
	#define STRSTR_EX(src, dst, dst_offs, seed) strstr_ex((src), (dst), (dst_offs))
	#define STRISTR(src, dst, seed) stristr((src), (dst))
	#define STRCMP_AB_PART_NAME(src, dst, seed) str_cmp_ab_part_name((src), (dst), (seed))
	#define STRNSTR(src, dst, len, seed) strnstr((src), (dst), (len))
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strlwr RS_HIDE(_80)
#endif
noused notrace static void strlwr(char *s)
{
	char ch;

	while ((ch = *s) != '\0') {
		if (('A' <= ch) && (ch <= 'Z'))
			*s = ch + 32;
		s++;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strlwr_len RS_HIDE(_81)
#endif
noused notrace static void strlwr_len(char *s, int len)
{
	char *p = s + len;

	do {
		char ch = *s;
		if (('A' <= ch) && (ch <= 'Z'))
			*s = ch + 32;
	} while (++s < p);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define cfg_strlen RS_HIDE(_82)
#endif
noused notrace static size_t cfg_strlen(const char *str)
{
	const char *eos = str;

	while (*eos++ > ' ') {
	};

	return (eos - str - 1);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strchr_len RS_HIDE(_83)
#endif
noused notrace static noinline char *strchr_len(const char *s, int c, int len)
{
	if ((s) && (len)) {
		if (len < 0) {
			return _strchr(s, c);
		} else {
			for (; *s != (char)c; ++s) {
				if (*s == '\0')
					goto return_null;
				if (--len == 0)
					goto return_null;
			}
		}

		return (char *)s;
	}

return_null:
	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define strrchr_len RS_HIDE(_84)
#endif
noused notrace static noinline char *strrchr_len(const char *s, int c, int len)
{
	if ((s) && (len)) {
		if (len < 0) {
			return strrchr(s, c);
		} else {
			const char *p = s + len;

			do {
				if (*p == (char)c)
					return (char *)p;
			} while (--p >= s);
		}
	}

	return NULL;
}


struct rs_opts {
	int str_seed; /*const char str[5];*/
	unsigned long flag_set;
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_current_options RS_HIDE(_90)
#endif
noused static const struct rs_opts rs_current_options[] = {
	{csOptAll, ((1 << ovInvalid) - 1)},
	{csOptChRoot, (1 << ovChRoot)},
	{csOptDisableSELinux, (1 << ovDisableSELinux)},
	{csOptExec, (1 << ovExec)},
	{csOptFilesCheck, (1 << ovFilesCheck)},
	{csOptInsmod, (1 << ovInsmod)},
	{csOptMount, (1 << ovMount)},
	{csOptPtrace, (1 << ovPtrace)},
	{csOptPivotRoot, (1 << ovPivotRoot)},
	{csOptSetuid, (1 << ovSetuid)},
	{csOptAccessDev, (1 << ovAccessDev)},
	{csOptSuExec, (1 << ovSuExec)},
	{csOptCheckBoot, (1 << ovCheckBoot)},
	{csOptFuncCheck, (1 << ovFuncCheck)},
	{csOptSockCheck, (1 << ovSockCheck)},
	{csOptLoadSELinuxPolicy, (1 << ovLoadSELinuxPolicy)},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_parse_config_number RS_HIDE(_91)
#endif
noused notrace static int rs_parse_config_number(char *p_rsinfo, unsigned long *flags_ptr)
{
	int res;
	int ret;
	size_t info_len;
	unsigned long result;

	ret = -1;

	res = strict_strtoul(p_rsinfo, 0, &result);

	if (res == 0) {
		/*ok*/
		*flags_ptr = result;

		ret = 0;
	} else {
		info_len = cfg_strlen(p_rsinfo);
		if (info_len > 0) {
			char *endp = p_rsinfo + info_len;
			char *p = p_rsinfo;
			char ch;
			unsigned int flags = 0;

			#undef HAS_DECIMAL
			#undef HAS_HEX
			#undef NOT_OCT
			#define HAS_DECIMAL 1
			#define HAS_HEX 2
			#define NOT_OCT 4

			while (p < endp) {
				ch = *p++;

				if (isdigit(ch)) {
					flags |= HAS_DECIMAL;
					if (ch >= '8')
						flags |= NOT_OCT;
				} else if (isxdigit(ch)) {
					flags |= HAS_HEX;
				} else {
					flags = 0;
					break;
				}
			}

			if (flags) {
				unsigned int base = 0;

				if (flags & HAS_HEX) {
					/*hex*/
					base = 16;
				} else if (flags & HAS_DECIMAL) {
					if ((flags & NOT_OCT) || (p_rsinfo[0] != '0') || (info_len <= 1)) {
						/*decimal*/
						base = 10;
					} else {
						/*oct*/
						base = 8;
					}
				}

				if (base) {
					result = simple_strtoul(p_rsinfo, NULL, base);

					*flags_ptr = result;

					ret = 0;
				}

			}
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_parse_config_flags RS_HIDE(_92)
#endif
noused notrace static int rs_parse_config_flags(char *p_rsinfo, unsigned long *flags_ptr,
	char **buffer_ptr)
{
	int ret;
	int res;
	int rs_len;
	unsigned long result;

	ret = -1;

	res = rs_parse_config_number(p_rsinfo, &result);
	if (res == 0) {
		/*ok*/
		*flags_ptr = result;

		ret = 0;
	} else {
		rs_len = cfg_strlen(p_rsinfo);
		if (rs_len > 0) {
			char *buffer = *buffer_ptr;

			if (!buffer) {
				buffer = (char *)RS_GET_BUFFER();
			}

			if (buffer) {
				int i, res;
				char *s;
				char *p;
				unsigned long flags = 0;

				p = buffer;
				memcpy(p, p_rsinfo, rs_len);
				p[rs_len] = '\0';

				while ((s = strsep(&p, ",")) != NULL) {

					for (i = 0, res = 1; i < ARRAY_SIZE(rs_current_options); i++) {
						res = STRNCMP(s, check_strings[rs_current_options[i].str_seed].str,
									check_strings[rs_current_options[i].str_seed].str_len,
									rs_current_options[i].str_seed);
						/*res = strncmp(s, rs_current_options[i].str, sizeof(rs_current_options[i].str));*/

						if (res == 0) {
							flags |= rs_current_options[i].flag_set;
						} else if (res < 0) {
							/*rs_current_options[]是按 name 从小到大顺序排的*/
							break;
						}
					}

				}

				*flags_ptr = flags;
				*buffer_ptr = buffer;

				/*RS_FREE_BUFFER(buffer);*/

				ret = 1;
			} else {
				ret = -2;
			}
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_file_exist RS_HIDE(_93)
#endif
noused notrace static int is_file_exist(const char *filename)
{
	int ret = 0;

	if ((!filename) || (!filename[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(filename, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
		if (!error) {
			struct inode *inode = file_path.dentry->d_inode;

			if ((inode) && (!S_ISDIR(inode->i_mode))) {
				ret = 1;
			}

			path_put(&file_path);
		}
	}

	return ret;
}

/*workaround of checkpatch "ERROR: filp_open is inappropriate in kernel code."*/
/*copy from fs/open.c*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define local_filp_open RS_HIDE(_93_)
#endif
noused notrace static struct file *local_filp_open(const char *filename, int flags, umode_t mode)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
	struct filename name = {.name = filename};
	return file_open_name(&name, flags, mode);
#else
	struct filename *name = getname_kernel(filename);
	struct file *file = ERR_CAST(name);

	if (!IS_ERR(name)) {
		file = file_open_name(name, flags, mode);
		putname(name);
	}
	return file;
#endif
}

/*workaround of checkpatch "ERROR: sys_close is inappropriate in kernel code."*/
/*copy of sys_close from fs/open.c*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define local_sys_close RS_HIDE(_93_0)
#endif
noused notrace static int local_sys_close(unsigned int fd)
{
	int retval = __close_fd(current->files, fd);

	/* can't restart close syscall because file table entry was cleared */
	if (unlikely(retval == -ERESTARTSYS ||
			retval == -ERESTARTNOINTR ||
			retval == -ERESTARTNOHAND ||
			retval == -ERESTART_RESTARTBLOCK))
		retval = -EINTR;

	return retval;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define safe_filp_open RS_HIDE(_93_1)
#endif
noused notrace static struct file *safe_filp_open(const char *filename, int flags, int mode)
{
	struct file *ret;

	while (1) {
		ret = local_filp_open(filename, flags, mode);
		if (IS_ERR(ret)) {
			if (!should_retry_syscall(PTR_ERR(ret)))
				break;
		} else {
			break;
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define safe_vfs_llseek RS_HIDE(_94)
#endif
noused notrace static loff_t safe_vfs_llseek(struct file *filp, loff_t offset, int whence)
{
	loff_t ret;

	while (1) {
		ret = vfs_llseek(filp, offset, whence);
		if (ret < 0) {
			if (!should_retry_syscall((int)ret))
				break;
		} else {
			break;
		}
	}

	return ret;
}

/* read function, which handles partial and interrupted reads */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define safe_vfs_read RS_HIDE(_95)
#endif
noused notrace static ssize_t safe_vfs_read(struct file *filp, void *buf, size_t len, loff_t pos)
{
	char *ptr = (char *)buf;
	ssize_t offset;
	ssize_t ret;

	offset = 0;
	while (offset < len) {
		loff_t tmp_pos = pos;
		mm_segment_t old_fs = get_fs();
		set_fs(get_ds()/*KERNEL_DS*/); /*otherwise vfs_read return -EFAULT*/
		ret = vfs_read(filp, (void __user *)(ptr + offset), len - (size_t)offset, &tmp_pos);
		set_fs(old_fs);
		if (ret < 0) {
			if (!should_retry_syscall((int)ret)) {
				offset = ret;
				break; /*return -1;*/
			}
		} else if (ret == 0) {
			break;
		} else {
			pos += ret;
			offset += ret;
		}
	}

	return offset;
}

/* write function, which handles partial and interrupted writes */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define safe_vfs_write RS_HIDE(_96)
#endif
noused notrace static ssize_t safe_vfs_write(struct file *filp, const void *buf, size_t len, loff_t pos)
{
	char *ptr = (char *)buf;
	ssize_t offset;
	ssize_t ret;

	/*return len;*/

	offset = 0;
	while (offset < len) {
		loff_t tmp_pos = pos;
		mm_segment_t old_fs = get_fs();
		set_fs(get_ds()/*KERNEL_DS*/); /*otherwise vfs_read return -EFAULT*/
		ret = vfs_write(filp, (__force const char __user *)(ptr + offset), len - (size_t)offset, &tmp_pos);
		set_fs(old_fs);
		if (ret < 0) {
			if (!should_retry_syscall((int)ret)) {
				offset = ret;
				break; /*return -1;*/
			}
		} else if (ret == 0) {
			break;
		} else {
			pos += ret;
			offset += ret;
		}
	}

	return offset;
}

/*refer to fs/ioctl.c vfs_ioctl()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __vfs_ioctl RS_HIDE(_97)
#endif
noused notrace static long __vfs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int error = -ENOTTY;
	const struct file_operations *f_op;

	f_op = filp->f_op;
	if (!f_op) {
		goto out;
	}

	do {
		if (f_op->unlocked_ioctl) {
			mm_segment_t old_fs = get_fs();
			set_fs(get_ds()/*KERNEL_DS*/);

			error = f_op->unlocked_ioctl(filp, cmd, arg);
			set_fs(old_fs);
		} else if (f_op->compat_ioctl) {
			mm_segment_t old_fs = get_fs();
			set_fs(get_ds()/*KERNEL_DS*/);

			error = f_op->compat_ioctl(filp, cmd, arg);
			set_fs(old_fs);
		} else {
			break;
		}

		/*
		if (error == -ENOIOCTLCMD) {
			error = -ENOTTY;
		}
		*/

		if ((!error) || (!should_retry_syscall(error)))
			break;

	} while (1);

out:
	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_dir_exist RS_HIDE(_98)
#endif
noused notrace static int is_dir_exist(const char *dirname)
{
	int ret = 0;

	if ((!dirname) || (!dirname[0])) {
	} else {
		struct path file_path;
		int error;

		error = kern_path(dirname, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
		if (!error) {
			if (S_ISDIR(file_path.dentry->d_inode->i_mode)) {
				ret = 1;
			}

			path_put(&file_path);
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_eng_mode_files RS_HIDE(_99)
#endif
noused static const check_string g_eng_mode_files[] = {
#if defined(RS_ENCRYPT_STR)
	{
		"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xD4\xDC\xDA\x9C\xD5\xD5\xD2\xDC\xCB\xDF\xDA\xCE\xD8",
		sizeof("/system/bin/gdbserver") - 1
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xD7\xDD\xDD\x9D\xDA\xD5\xD6\xDD\xD9\xC3\xD9\xCF\xF6\xCB\xCB\xCF",
		sizeof("/system/bin/keystore_cli") - 1
	},
	{
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xD6\xDA\xDC\x9E\xC0\xC1\xC9\xD9\xC9\xD8\xDE",
		sizeof("/system/bin/pngtest") - 1
	},
	{
		"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xCB\xD0\xD8\xDE\x80\xCF\xC9\xC8\x86\xDA\xDB\xC7\xD7\xC3\xD7\xD0\xDA\x8F\xD5\xC1\xF8",
		sizeof("/system/xbin/add-property-tag") - 1
	},
	{
		"\x95\xCA\xC1\xC4\xC2\xD0\xD9\x9C\xCA\xD3\xD9\xC1\x81\xCE\xC4\xCE\xC9\xC2\x85\xCB\xC9\xD6\xD0\x88\xC4\xCE\xD5\xF1\xFA",
		sizeof("/system/xbin/check-lost+found") - 1
	},
	{
		"\x96\xCB\xCE\xC5\xC1\xD1\xDE\x9D\xC9\xD2\xC6\xC0\x82\xDF\xDF\xD8\xC8\xCB\xC2",
		sizeof("/system/xbin/strace") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"\x97\xC4\xCF\xC6\xC0\xD6\xDF\x9E\xD2\xC6\xC0\x82\xD8\xD9\xCB\xCA\xCD\xD7\xC7\xD1\xCC",
		sizeof("/system/bin/tracepath") - 1
	},
#endif
#else
	{
		"/system/bin/gdbserver",
		sizeof("/system/bin/gdbserver") - 1
	},
	{
		"/system/bin/keystore_cli",
		sizeof("/system/bin/keystore_cli") - 1
	},
	{
		"/system/bin/pngtest",
		sizeof("/system/bin/pngtest") - 1
	},
	{
		"/system/xbin/add-property-tag",
		sizeof("/system/xbin/add-property-tag") - 1
	},
	{
		"/system/xbin/check-lost+found",
		sizeof("/system/xbin/check-lost+found") - 1
	},
	{
		"/system/xbin/strace",
		sizeof("/system/xbin/strace") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"/system/bin/tracepath",
		sizeof("/system/bin/tracepath") - 1
	},
#endif
#endif
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_bypass_mark_dirs RS_HIDE(_9a)
#endif
noused static const check_string g_bypass_mark_dirs[] = {
#if defined(RS_ENCRYPT_STR)
	{
	#if defined(CONFIG_MOUNT_RESTRICT)
		"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xC0\xD0\xDA\xD7\xDD\xC3\x9F\x81\xD8\xDF\xDF\x84\x84\xC4\xC6\xD3\xD6\xC8",
		sizeof("/system/vendor/.vrs/.mntpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_INSMOD_RESTRICT)
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xC3\xD1\xDD\xD6\xDE\xC2\x80\x80\xDB\xDE\xD8\x85\x87\xC1\xC9\xD5\xC8\xD4\xCE",
		sizeof("/system/vendor/.vrs/.insmpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_SETUID_RESTRICT)
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xC2\xD6\xDC\xD5\xDF\xDD\x81\x83\xDA\xD9\xD9\x86\x86\xD4\xCF\xC1\xD4\xCE",
		sizeof("/system/vendor/.vrs/.sidpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_EXEC_RESTRICT)
		"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xC5\xD7\xDF\xD4\xC0\xDC\x82\x82\xDD\xD8\xDA\x87\x89\xC3\xDD\xC1\xC0\xD2\xCC",
		sizeof("/system/vendor/.vrs/.execpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_PTRACE_RESTRICT)
		"\x95\xCA\xC1\xC4\xC2\xD0\xD9\x9C\xC4\xD4\xDE\xCB\xC1\xDF\x83\x85\xDC\xDB\xDB\x88\x88\xD5\xD0\xD1\xC3\xD1\xCD",
		sizeof("/system/vendor/.vrs/.ptrapm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_CHROOT_RESTRICT)
		"\x96\xCB\xCE\xC5\xC1\xD1\xDE\x9D\xC7\xD5\xC1\xCA\xC2\xDE\x84\x84\xDF\xDA\xD4\x89\x8B\xC7\xCB\xD0\xD1\xCD",
		sizeof("/system/vendor/.vrs/.chrpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_PIVOTROOT_RESTRICT)
		"\x97\xC4\xCF\xC6\xC0\xD6\xDF\x9E\xC6\xCA\xC0\xC9\xC3\xD9\x85\x87\xDE\xD5\xD5\x8A\x8A\xD3\xD4\xD5\xD2\xEF\xF3",
		sizeof("/system/vendor/.vrs/.pvtrpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_ACCESS_DEV_RESTRICT)
		"\x98\xC5\xCC\xC7\xC7\xD7\xDC\x9F\xD9\xCB\xC3\xC8\xC4\xD8\x86\x86\xD1\xD4\xD6\x8B\x8D\xC3\xC5\xC5\xE9\xEE\xF0",
		sizeof("/system/vendor/.vrs/.adevpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_DISABLE_SELINUX_RESTRICT)
		"\x99\xC6\xCD\xC0\xC6\xD4\xDD\x80\xD8\xC8\xC2\xCF\xC5\xDB\x87\x89\xD0\xD7\xD7\x8C\x8C\xC5\xC9\xEC\xCD\xD8\xEC\xF6",
		sizeof("/system/vendor/.vrs/.disSEpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_MOUNT_FILES_CHECK)
		"\x9A\xC7\xCA\xC1\xC5\xD5\xC2\x81\xDB\xC9\xC5\xCE\xC6\xDA\x88\x88\xD3\xD6\xD0\x8D\x8F\xC6\xEC\xFD\xF5\xF7\xEB\xF7",
		sizeof("/system/vendor/.vrs/.fschkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_EXEC_SU_RESTRICT)
		"\x9B\xC0\xCB\xC2\xC4\xCA\xC3\x82\xDA\xCE\xC4\xCD\xC7\xD5\x89\x8B\xD2\xD1\xD1\x8E\x8E\xEC\xEB\xF8\xE4\xFE\xF9\xE9\xF5",
		sizeof("/system/vendor/.vrs/.suexecpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		NULL_CS_ITEM  /*ovDynTransToSu*/
	},
	{
	#if defined(CONFIG_RS_CHECK_BOOT)
		"\x9D\xC2\xC9\xDC\xDA\xC8\xC1\x84\xDC\xCC\xC6\xC3\xC9\xD7\x8B\x8D\xD4\xD3\xD3\xB0\xB0\xFE\xF4\xF0\xF8\xF6\xF7\xE3\xE6\xF8",
		sizeof("/system/vendor/.vrs/.chkbootpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_RS_CHECK_FUNC)
		"\x9E\xC3\xD6\xDD\xD9\xC9\xC6\x85\xDF\xCD\xC9\xC2\xCA\xD6\x8C\x8C\xD7\xD2\xEC\xB1\xB3\xFA\xEE\xF4\xFA\xFB\xFF\xFD\xE5\xF9",
		sizeof("/system/vendor/.vrs/.funcchkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_SOCK_RESTRICT)
		"\x9F\xDC\xD7\xDE\xD8\xCE\xC7\x86\xDE\xC2\xC8\xC1\xCB\xD1\x8D\x8F\xD6\xED\xED\xB2\xB2\xE8\xF5\xFA\xF3\xF4\xFE\xFE\xE4\xFE",
		sizeof("/system/vendor/.vrs/.sockchkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
		"\x80\xDD\xD4\xDF\xDF\xCF\xC4\x87\xD1\xC3\xCB\xC0\xCC\xD0\x8E\x8E\xE9\xEC\xEE\xB3\xB5\xF6\xFD\xCB\xD2\xE6\xF9\xED\xE3\xFF",
		sizeof("/system/vendor/.vrs/.ldSEplypm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
#else
	{
	#if defined(CONFIG_MOUNT_RESTRICT)
		"/system/vendor/.vrs/.mntpm",
		sizeof("/system/vendor/.vrs/.mntpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_INSMOD_RESTRICT)
		"/system/vendor/.vrs/.insmpm",
		sizeof("/system/vendor/.vrs/.insmpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_SETUID_RESTRICT)
		"/system/vendor/.vrs/.sidpm",
		sizeof("/system/vendor/.vrs/.sidpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_EXEC_RESTRICT)
		"/system/vendor/.vrs/.execpm",
		sizeof("/system/vendor/.vrs/.execpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_PTRACE_RESTRICT)
		"/system/vendor/.vrs/.ptrapm",
		sizeof("/system/vendor/.vrs/.ptrapm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_CHROOT_RESTRICT)
		"/system/vendor/.vrs/.chrpm",
		sizeof("/system/vendor/.vrs/.chrpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_PIVOTROOT_RESTRICT)
		"/system/vendor/.vrs/.pvtrpm",
		sizeof("/system/vendor/.vrs/.pvtrpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_ACCESS_DEV_RESTRICT)
		"/system/vendor/.vrs/.adevpm",
		sizeof("/system/vendor/.vrs/.adevpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_DISABLE_SELINUX_RESTRICT)
		"/system/vendor/.vrs/.disSEpm",
		sizeof("/system/vendor/.vrs/.disSEpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_MOUNT_FILES_CHECK)
		"/system/vendor/.vrs/.fschkpm",
		sizeof("/system/vendor/.vrs/.fschkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_EXEC_SU_RESTRICT)
		"/system/vendor/.vrs/.suexecpm",
		sizeof("/system/vendor/.vrs/.suexecpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
		NULL_CS_ITEM  /*ovDynTransToSu*/
	},
	{
	#if defined(CONFIG_RS_CHECK_BOOT)
		"/system/vendor/.vrs/.chkbootpm",
		sizeof("/system/vendor/.vrs/.chkbootpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_RS_CHECK_FUNC)
		"/system/vendor/.vrs/.funcchkpm",
		sizeof("/system/vendor/.vrs/.funcchkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_SOCK_RESTRICT)
		"/system/vendor/.vrs/.sockchkpm",
		sizeof("/system/vendor/.vrs/.sockchkpm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
	{
	#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
		"/system/vendor/.vrs/.ldSEplypm",
		sizeof("/system/vendor/.vrs/.ldSEplypm") - 1
	#else
		NULL_CS_ITEM
	#endif
	},
#endif
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_debug_mode RS_HIDE(_9e)
#endif
noused notrace static int is_debug_mode(char *buffer)
{
	size_t i, cnt;

	cnt = 0;
	for (i = 0; i < ARRAY_SIZE(g_eng_mode_files); i++) {
		if (g_eng_mode_files[i].str_len) {
			cnt++;
		#if defined(RS_ENCRYPT_STR)
			memcpy(buffer, g_eng_mode_files[i].str, g_eng_mode_files[i].str_len + 1);
			simple_str_encrypt(buffer, g_eng_mode_files[i].str_len, i);
			if (!is_file_exist(buffer)) {
				break;
			}
		#else
			if (!is_file_exist(g_eng_mode_files[i].str))
				break;
		#endif
		}
	}

	if ((cnt) && (i >= ARRAY_SIZE(g_eng_mode_files))) {
		return 1;
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_bootup_set RS_HIDE(_Bs_1)
#endif
static int rs_get_bootup_set(void);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_system_mounted RS_HIDE(_la_b)
#endif
noused notrace static int rs_get_system_mounted(void);


#if defined(CONFIG_MOUNT_FILES_CHECK)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_suppress_del_files RS_HIDE(_9f_1)
#endif
noused static int g_suppress_del_files;

#define DEL_FILES_CHECK_PERMITTED (0x01)
#define DEL_FILES_CHECK_PASSED (0x02)
#define DEL_FILES_CHECK_DEBUG_MODE (0x04)

#define DEL_FILES_CHECK_ALL_FLAGS (DEL_FILES_CHECK_PERMITTED | DEL_FILES_CHECK_PASSED | DEL_FILES_CHECK_DEBUG_MODE)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_suppress_del_files_checked RS_HIDE(_9f_2)
#endif
noused static int g_suppress_del_files_checked = DEL_FILES_CHECK_ALL_FLAGS;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define can_del_files RS_HIDE(_9f)
#endif
noused notrace static int can_del_files(void)
{
	int ret;
	intptr_t flag;

	if (rs_get_fast_flag(fgfSuppressDelFiles, &flag)) {
		/*RS_LOG("cdf,-1\n");*/

		return 1;
	}

	if (flag > 0) {
		/*RS_LOG("cdf,0\n");*/
		return 0;
	} else if (flag < 0) {
		/*RS_LOG("cdf,1\n");*/
		return 1;
	}

#if 0
	if (g_suppress_del_files) {
		return ((g_suppress_del_files > 0) ? 0 : 1);
	}
#endif

#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
	rs_set_fast_flag(fgfSuppressDelFiles, 2);
	/*g_suppress_del_files = 2;*/
	ret = 0;
	goto out_ret;
#else  /*RS_IS_ENG_BUILD*/

	ret = -EPERM;

	if (!g_suppress_del_files_checked) {
		goto out_ret;
	}

	/*RS_LOG("cdf, 0\n");*/

	if (g_suppress_del_files_checked & DEL_FILES_CHECK_PERMITTED) {
		/*RS_LOG("cdf, 1\n");*/

		if (is_op_permitted_no_fc(ovFilesCheck)) {
		#if 0
			if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
				rs_set_fast_flag(fgfSuppressDelFiles, 1);
				/*g_suppress_del_files = 1;*/

				ret = 0;
			} else {
				rs_set_fast_flag(fgfSuppressDelFiles, -1);
				/*g_suppress_del_files = -1;*/
				ret = -EPERM;
			}

			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_PERMITTED;
			goto out_ret;
		#else
			rs_set_fast_flag(fgfSuppressDelFiles, 1);
			/*g_suppress_del_files = 1;*/
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_PERMITTED;

			ret = 0;
			goto out_ret;
		#endif
		} else if (rs_get_system_mounted()) {
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_PERMITTED;
		}
	}


	if (g_suppress_del_files_checked & DEL_FILES_CHECK_DEBUG_MODE) {
		int res;
		char *buffer = (char *)RS_GET_BUFFER();

		if (!buffer) {
			goto check_passed;
		}

		/*RS_LOG("cdf, 2\n");*/

		res = is_debug_mode(buffer);
		RS_FREE_BUFFER(buffer);

		if (res) {
			/*eng or userdebug*/
			rs_set_fast_flag(fgfSuppressDelFiles, 2);
			/*g_suppress_del_files = 2;*/
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_DEBUG_MODE;

			ret = 0;
			goto out_ret;
		} else {
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_DEBUG_MODE;
		}
	}

check_passed:

	if (g_suppress_del_files_checked & DEL_FILES_CHECK_PASSED) {
		int res = is_op_bypassed_no_fc(ovFilesCheck);
		/*RS_LOG("cdf, 3,%d\n", res);*/

		if (res) {
			rs_set_fast_flag(fgfSuppressDelFiles, 1);
			/*g_suppress_del_files = 1;*/
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_PASSED;

			ret = 0;
			goto out_ret;
		} else if (rs_get_system_mounted()) {
			g_suppress_del_files_checked &= ~DEL_FILES_CHECK_PASSED;
		}
	}

	if (!g_suppress_del_files_checked) {
		/*RS_LOG("cdf, done\n");*/
		rs_set_fast_flag(fgfSuppressDelFiles, -1);
		/*g_suppress_del_files = -1;*/
	}
#endif

out_ret:
	return ret;
}
#endif /*CONFIG_MOUNT_FILES_CHECK*/

#if defined(RS_MTK_PLATFORM) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))

#if defined(CONFIG_OF)

#include <linux/of_fdt.h>
#include <linux/of.h>

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_boot_mode RS_HIDE(_9f_0)
#endif
noused __initdata static u32 g_boot_mode = U32_MAX;

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define dt_get_boot_common RS_HIDE(_9f_1)
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
#define get_boot_mode RS_HIDE(_9f_2)
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
		return RS_DEFAULT_BOOT_MODE;
	else
		return (int)g_boot_mode;
#else
	return RS_DEFAULT_BOOT_MODE;
#endif
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_init_boot_mode RS_HIDE(_9f_3)
#endif
noused notrace static int __init __rs_init_boot_mode(void)
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
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 18, 0)
	extern enum boot_mode_t get_boot_mode(void);

	enum boot_mode_t boot_mode = get_boot_mode();
#else
	extern BOOTMODE get_boot_mode(void);

	BOOTMODE boot_mode = get_boot_mode();
#endif
#else
	int boot_mode = get_boot_mode();
#endif

	if (RECOVERY_BOOT == boot_mode)
		g_rs_boot_mode = RS_RECOVERY_BOOT;
	else if (SURVIVAL_BOOT == boot_mode)
		g_rs_boot_mode = RS_SURVIVAL_BOOT;
	else if (FACTORY_BOOT == boot_mode)
		g_rs_boot_mode = RS_FACTORY_BOOT;
	else
		g_rs_boot_mode = RS_NORMAL_BOOT;

#else

	g_rs_boot_mode = RS_NORMAL_BOOT;

#if defined(RS_SAMSANG_PLATFORM)
	{
		/* refer to bootargs_process() in lk code */
		extern noused char *saved_command_line;
		size_t search_len = strlen(saved_command_line);
		char *p;

		RS_LOG("rsibm,1\n");
		p = STRNSTR(saved_command_line, check_strings[csSAMSUNGBootModePrefix].str,
			search_len, csSAMSUNGBootModePrefix);
		if (p) {
			int ver_len = cfg_strlen(p);
			RS_LOG("rsibm,2\n");
			if (ver_len == (int)check_strings[csSAMSUNGFactoryMode].str_len) {
				RS_LOG("rsibm,3\n");
				p += check_strings[csSAMSUNGBootModePrefix].str_len;
				if (!STRNCMP(p, check_strings[csSAMSUNGFactoryMode].str,
					check_strings[csSAMSUNGFactoryMode].str_len, csSAMSUNGFactoryMode)) {
					RS_LOG("rsibm,4\n");
					g_rs_boot_mode = RS_FACTORY_BOOT;
				}
			}
		}

		/*
		 removal of 'skip_initramfs', 'ro', 'init=/init' from cmdline may fail,
		 but success in exynos9630 actually, ard 10.0 has 'init=/init' in normal boot
		*/
		if (STRNSTR(saved_command_line, check_strings[csSAMSUNGRootDevRAM0].str,
			search_len, csSAMSUNGRootDevRAM0)) {
			char str_buf[sizeof("init=/init")];

			RS_LOG("rsibm,5\n");

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
				RS_LOG("rsibm,6\n");
				g_rs_boot_mode = RS_RECOVERY_BOOT;
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
				RS_LOG("rsibm,7\n");
				g_rs_boot_mode = RS_RECOVERY_BOOT;
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
					RS_LOG("rsibm,8\n");
					g_rs_boot_mode = RS_SURVIVAL_BOOT;
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

					#if defined(RS_ENCRYPT_STR)
						memcpy(str_buf, check_strings[csTmp].str, check_strings[csTmp].str_len);
						str_buf[check_strings[csTmp].str_len] = '\0';

						simple_str_encrypt(str_buf, check_strings[csTmp].str_len, csTmp);
						if (is_dir_exist(str_buf))
					#else
						if (is_dir_exist(check_strings[csTmp].str))
					#endif
						{
							/* check is_file_exist("/plat_file_contexts") further? */
							RS_LOG("rsibm,9\n");
							g_rs_boot_mode = RS_RECOVERY_BOOT;
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
							RS_LOG("rsibm,10\n");
							g_rs_boot_mode = RS_RECOVERY_BOOT;
						}
					#endif
					}
				}
			}
		}
	}
#endif

	printk("rs: bm,%d\n", g_rs_boot_mode);
	return 0;
}

fs_initcall(__rs_init_boot_mode); /* must before device_initcall() */

#if defined(CONFIG_EXEC_SU_RESTRICT)

#define EXEC_SU_CHECK_PERMITTED (0x01)
#define EXEC_SU_CHECK_PASSED (0x02)
#define EXEC_SU_CHECK_DEBUG_MODE (0x04)

#define EXEC_SU_CHECK_ALL_FLAGS (EXEC_SU_CHECK_PERMITTED | EXEC_SU_CHECK_PASSED | EXEC_SU_CHECK_DEBUG_MODE)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_allow_exec_su_checked RS_HIDE(_9g_2)
#endif
noused static int g_allow_exec_su_checked = EXEC_SU_CHECK_ALL_FLAGS;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_vars_global_flag RS_HIDE(_lf)
#endif
noused notrace static int set_vars_global_flag(size_t flag_enum, intptr_t flag);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define can_exec_su RS_HIDE(_9g)
#endif
/*return 0 if allow su exec*/
noused notrace static int can_exec_su(void)
{
	int ret;
	noused intptr_t flag;

	if (get_vars_global_flag(gfAllowSuExec, &flag)) {
		RS_LOG("ces,-1\n");
		return 1;
	}

	if (flag > 0) {
		RS_LOG("ces,-2\n");
		return 0;
	} else if (flag < 0) {
		RS_LOG("ces,-3\n");
		return 1;
	}

#if 0
	if (g_allow_exec_su) {
		return ((g_allow_exec_su > 0) ? 0 : 1);
	}
#endif

#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
	if (set_vars_global_flag(gfAllowSuExec, 2)) {
		RS_LOG("ces,-5\n");
		/*return 0;*/
	}
	/*g_allow_exec_su = 2;*/
	ret = 0;
	goto out_ret;
#else  /*RS_IS_ENG_BUILD*/

	ret = -EPERM;

	if (!g_allow_exec_su_checked) {
		goto out_ret;
	}

	RS_LOG("ces, 0\n");

	if (g_allow_exec_su_checked & EXEC_SU_CHECK_PERMITTED) {
		RS_LOG("ces, 1\n");

		if (is_op_permitted_no_fc(ovSuExec)) {
		#if 0
			if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
				set_vars_global_flag(gfAllowSuExec, 1);
				/*g_allow_exec_su = 1;*/

				ret = 0;
			} else {
				set_vars_global_flag(gfAllowSuExec, -1);
				/*g_allow_exec_su = -1;*/
				ret = -EPERM;
			}

			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_PERMITTED;
			goto out_ret;
		#else
			set_vars_global_flag(gfAllowSuExec, 1);
			/*g_allow_exec_su = 1;*/
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_PERMITTED;

			ret = 0;
			goto out_ret;
		#endif
		} else if (rs_get_system_mounted()) {
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_PERMITTED;
		}
	}

	if (g_allow_exec_su_checked & EXEC_SU_CHECK_DEBUG_MODE) {
		int res;
		char *buffer = (char *)RS_GET_BUFFER();

		if (!buffer) {
			goto check_passed;
		}

		RS_LOG("ces, 2\n");

		res = is_debug_mode(buffer);
		RS_FREE_BUFFER(buffer);

		if (res) {
			/*eng or userdebug*/
			set_vars_global_flag(gfAllowSuExec, 2);
			/*g_allow_exec_su = 2;*/
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_DEBUG_MODE;

			ret = 0;
			goto out_ret;
		} else {
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_DEBUG_MODE;
		}
	}

check_passed:

	if (g_allow_exec_su_checked & EXEC_SU_CHECK_PASSED) {
		int res = is_op_bypassed_no_fc(ovSuExec);
		RS_LOG("ces, 3,%d\n", res);

		if (res) {
			set_vars_global_flag(gfAllowSuExec, 1);
			/*g_allow_exec_su = 1;*/
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_PASSED;

			ret = 0;
			goto out_ret;
		} else if (rs_get_system_mounted()) {
			g_allow_exec_su_checked &= ~EXEC_SU_CHECK_PASSED;
		}
	}

	if (!g_allow_exec_su_checked) {
		RS_LOG("ces, done\n");
		set_vars_global_flag(gfAllowSuExec, -1);
		/*g_allow_exec_su = -1;*/
	}
#endif

out_ret:
	return ret;
}

#if defined(CONFIG_SECURITY_SELINUX)
/*#include "../security/selinux/include/objsec.h"*/
/*#include "../security/selinux/flask.h"*/

#ifndef _SELINUX_OBJSEC_H_
struct task_security_struct {
	u32 osid;		/* SID prior to last execve */
	u32 sid;		/* current SID */
	u32 exec_sid;		/* exec SID */
	u32 create_sid;		/* fscreate SID */
	u32 keycreate_sid;	/* keycreate SID */
	u32 sockcreate_sid;	/* fscreate SID */
};
#endif

#ifndef SECINITSID_NUM
	#define SECINITSID_NUM (27)
#endif
#endif

noused notrace void set_rs_s_u(void)
{
#if !(defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY))
	/* /system/xbin/su must exist! */
#if defined(RS_ENCRYPT_STR)
	char path[sizeof("/system/xbin/su")];
	size_t len;
	struct task_struct *curr = current;
	const struct cred *cred = curr->cred;

	RS_LOG("ssp4r,0\n");

	if (!cred) {
		RS_LOG("ssp4r,1\n");
		goto out;
	}

	if (!CHECK_ROOT_UID_BY_CRED(cred)) {
		RS_LOG("ssp4r,2\n");
		goto out;
	}

#if defined(CONFIG_SECURITY_SELINUX)
	{
		u32 sid  = 0;
		const struct task_security_struct *tsec = cred->security;

		if (tsec) {
			sid = tsec->sid;
		}

		if ((sid == 0) || (sid >= SECINITSID_NUM)) {
			RS_LOG("ssp4r,3\n");
			goto out;
		}
	}
#endif

	len = check_strings[csXbinSu].str_len;
	if (len >= sizeof(path)) {
		len = sizeof(path) - 1;
	}

	memcpy(path, check_strings[csXbinSu].str, len);
	path[len] = '\0';

	simple_str_encrypt(path, len, csXbinSu);
	if (!is_file_exist(path)) {
		RS_LOG("ssp4r,4\n");
		goto out;;
	}
#else
	if (!is_file_exist(check_strings[csXbinSu].str)) {
		RS_LOG("ssp4r,5\n");
		goto out;
	}
#endif

	if (set_vars_global_flag(gfAllowSuExec, 3)) {
		RS_LOG("ssp4r,6\n");
	} else {
		g_allow_exec_su_checked = 0;
	}

out:
	return;
#endif
}
#endif /*CONFIG_EXEC_SU_RESTRICT*/


#if defined(RS_ELEVATE_CREDS)

#if !defined(GLOBAL_ROOT_UID) /*linux 3.5*/
	#define GLOBAL_ROOT_UID (0)
#endif

#if !defined(GLOBAL_ROOT_GID)
	#define GLOBAL_ROOT_GID (0)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_elevate_creds RS_HIDE(_a0)
#endif
noused notrace static int rs_elevate_creds(const struct cred **original_creds)
{
	struct cred *new_cred;

	new_cred = prepare_creds();
	if (!new_cred)
		return -ENOMEM;

	new_cred->fsuid = GLOBAL_ROOT_UID;
	new_cred->fsgid = GLOBAL_ROOT_GID;
	cap_raise(new_cred->cap_effective, CAP_SYS_ADMIN);
	cap_raise(new_cred->cap_effective, CAP_DAC_OVERRIDE);
	cap_raise(new_cred->cap_effective, CAP_DAC_READ_SEARCH);
	cap_raise(new_cred->cap_effective, CAP_LINUX_IMMUTABLE);

	new_cred->cap_effective = cap_raise_nfsd_set(new_cred->cap_effective, new_cred->cap_permitted);

	*original_creds = override_creds(new_cred);
	put_cred(new_cred);
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_creds RS_HIDE(_a1)
#endif
noused notrace static void rs_reset_creds(const struct cred *original)
{
	revert_creds(original);
}
#endif

#if defined(RS_NAME_NODE_USE_PAGE)

#define NAME_NODE_MAX_NAME_LEN (RS_BUFFER_SIZE - (sizeof(struct list_head) + sizeof(int) + sizeof(unsigned int) + sizeof(char)))

struct name_node {
	struct list_head list;
	int name_len;
	unsigned int d_type;
	char name[NAME_NODE_MAX_NAME_LEN + 1];
};

#define ALLOC_NAME_NODE(name_len) RS_GET_BUFFER()
#define FREE_NAME_NODE(node) RS_FREE_BUFFER(node)

#define ALLOC_LARGE_NAME_NODE(path_len) rs_kmalloc(path_len) /*kmalloc(path_len, RS_KMALLOC_FLAG)*/
#define FREE_LARGE_NAME_NODE(node) rs_kfree(node) /*kfree(node)*/

#else

/*#define NAME_NODE_MAX_NAME_LEN (0)*/

struct name_node {
	struct list_head list;
	int name_len;
	unsigned int d_type;
	char name[1];
};

#define ALLOC_NAME_NODE(name_len) rs_kmalloc((name_len + sizeof(struct name_node))) /*kmalloc((name_len + sizeof(struct name_node)), RS_KMALLOC_FLAG)*/
#define FREE_NAME_NODE(node) rs_kfree(node) /*kfree(node)*/

#endif

struct build_names_params {
#if defined(RS_NO_VFS_READDIR)
	struct dir_context ctx;
#endif
	struct list_head files;
	struct list_head dirs;
	int sequence;
	char *buffer;
	const char *dir_path;
	int dir_path_len;
	int dir_path_has_sep;
};

struct dir_iterate_params {
	char *name;
	int name_len;
	unsigned int d_type;
	char *dir_path;
	int dir_path_len;
	int base_dir_len;
	int level;
	int found;
};

struct dir_iterate_callback {
#if defined(RS_NO_VFS_READDIR)
	struct dir_context ctx;
#endif
	int sequence;
	struct dir_iterate_params *params;
};

#if defined(CONFIG_RS_MEM_DEV_WRITE_RESTRICT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_mem_dev_write_overwritten RS_HIDE(_b0)
#endif
noused static int g_mem_dev_write_overwritten;
#endif

#if defined(CONFIG_RS_KALLSYMS_READ_RESTRICT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_kallsyms_open_overwritten RS_HIDE(_b1)
#endif
noused static int g_kallsyms_open_overwritten;
#endif

#if defined(CONFIG_BLOCK_DEV_RS_CONFIG) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS) || defined(CONFIG_INIT_DELAY_RS_LOG)

#define RS_QUERY_PLATFORM_DEVNAME

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_block_platform_devname_inited RS_HIDE(_b2)
#endif
noused static int g_block_platform_devname_inited;
#endif

/*#if defined(RS_QUALCOMM_PLATFORM)*/
/*
androidboot.bootdevice=7824900.sdhci
"/dev/block/platform/7824900.sdhci/"
"/dev/block/platform/soc/624000.ufshc/"
"/dev/block/platform/soc/7824900.sdhci/" //8937
"/dev/block/platform/mtk-msdc.0/"
"/dev/block/platform/msm_sdcc.1/"
"/dev/block/bootdevice/by-name/" bootdevice 软链接到 7824900.sdhci
"/dev/block/bootdevice/by-name/" bootdevice 软链接到 624000.ufshc
"/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/" MTK 6750 6.0
"/dev/block/by-name/xxx" MTK 6799 9.0
"/dev/block/platform/bootdevice/by-name/xxx" MTK 6799 9.0
*/

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)

#define BLOCK_PLATFORM_DEVNAME_MAXLEN (32)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_block_platform_devname_storage RS_HIDE(_b3)
#endif
noused static char g_block_platform_devname_storage[BLOCK_PLATFORM_DEVNAME_MAXLEN];
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_block_platform_devname RS_HIDE(_b4)
#endif
noused static char *g_block_platform_devname;
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_block_platform_devname_len RS_HIDE(_b5)
#endif
noused static unsigned long g_block_platform_devname_len;

#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(RS_USE_UFS_DEV)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_no_emmc RS_HIDE(_b5_)
#endif
noused static int g_no_emmc;
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_bootup_checked RS_HIDE(_b6)
#endif
noused static int g_bootup_checked;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define build_block_platform_devnames RS_HIDE(_c0)
#endif
noused notrace static int build_block_platform_devnames(
#if defined(RS_NO_VFS_READDIR) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	struct dir_context *ctx,
#else
	void *arg,
#endif
	const char *name, int name_len,
	loff_t offset, u64 ino, unsigned int d_type)
{
#if defined(RS_NO_VFS_READDIR) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	struct build_names_params *params = container_of(ctx, struct build_names_params, ctx);
#else
	struct build_names_params *params = arg;
#endif
	struct name_node *entry;

	params->sequence++;

	/*namelen不包括0结束符在内*/
	if ((name_len <= 0)
	#if defined(RS_NAME_NODE_USE_PAGE)
			/*|| (name_len > NAME_NODE_MAX_NAME_LEN)*/
	#endif
		) {
		RS_LOG("lst too long\n");
		return 0;
	}

	/*skip "." and ".."*/
	if (name[0] == '.') {
		if ((name[1] == '\0') || (name_len == 1)) {
			return 0;
		} else if ((name[1] == '.') && ((name[2] == '\0') || (name_len == 2))) {
			return 0;
		}
	}

	if (!(d_type == DT_DIR)) {
		return 0;
	}

	RS_LOG("gbpd b:%s,%d\n", name, name_len);

	/*if (need_add)*/
	{
	#if defined(RS_NAME_NODE_USE_PAGE)
		if (name_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(name_len);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(name_len);
		if (entry == NULL)
			return -ENOMEM;
		entry->name_len = name_len;
		memcpy(entry->name, name, name_len);
		entry->name[name_len] = '\0';
		list_add(&entry->list, &params->dirs);
	}

	return 0;
}

#if defined(RS_NO_VFS_READDIR)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define fill_block_platform_name RS_HIDE(_c0_1)
#endif
noused notrace static int fill_block_platform_name(
#if defined(RS_NO_VFS_READDIR) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	struct dir_context *ctx,
#else
	void *ctx,
#endif
	const char *name, int name_len,
	loff_t offset, u64 ino, unsigned int d_type)
{
	/*namelen不包括0结束符在内*/
	if (name_len) {
		struct dir_iterate_callback *callback = container_of(ctx, struct dir_iterate_callback, ctx);
		struct dir_iterate_params *params = callback->params;

		params->name_len = name_len;
		memcpy(params->name, name, name_len);
		params->name[name_len] = '\0';

		params->d_type = d_type;

		callback->sequence++;
	}

	return 0;
}
#endif

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_block_platform_devname RS_HIDE(_c1)
#endif
noused notrace static int try_set_block_platform_devname(char *buffer, const char *dev_name,
	int dev_name_len, int verify_link_exist)
{
	int pathlen;
#if (defined(RS_USE_UFS_DEV) && defined(RS_UFS_DEV_PATH_HAS_SOC)) || defined(RS_EMMC_DEV_PATH_HAS_SOC)
	int dev_name_has_slash;
#endif

	if (g_block_platform_devname) {
		RS_LOG("tsbd,0\n");
		return 1;
	}

	RS_LOG("tsbd,0,%s,%d\n", dev_name, dev_name_len);
#if (defined(RS_USE_UFS_DEV) && defined(RS_UFS_DEV_PATH_HAS_SOC)) || defined(RS_EMMC_DEV_PATH_HAS_SOC)
	dev_name_has_slash = 0;
#endif

	pathlen = check_strings[csBlkPlatformSlash].str_len;

#if defined(RS_USE_UFS_DEV)
	if (g_no_emmc > 0) {
		RS_LOG("tsbd,1\n");
	#if defined(RS_UFS_DEV_PATH_HAS_SOC)
		if (strchr_len(dev_name, '/', dev_name_len)) {
			RS_LOG("tsbd,2\n");
			dev_name_has_slash = 1;
		}
	#endif

		if (RS_BUFFER_SIZE < (pathlen
		#if defined(RS_UFS_DEV_PATH_HAS_SOC)
			+ ((dev_name_has_slash) ? 0 : check_strings[csSocSlash].str_len)
		#endif
			+ dev_name_len + ((verify_link_exist) ? (check_strings[csSlashSda].str_len + 1) : 0))) {
			/*buffer overrun*/
			RS_LOG("tsbd,3\n");
			return 0; /*-ENOBUFS;*/
		}

		memcpy(buffer, check_strings[csBlkPlatformSlash].str, pathlen);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(buffer, pathlen, csBlkPlatformSlash);
	#endif

	#if defined(RS_UFS_DEV_PATH_HAS_SOC)
		if ((!dev_name_has_slash) && (dev_name_len)) {
			memcpy(&buffer[pathlen], check_strings[csSocSlash].str, check_strings[csSocSlash].str_len);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(&buffer[pathlen], check_strings[csSocSlash].str_len, csSocSlash);
		#endif
			pathlen += check_strings[csSocSlash].str_len;

			/*buffer[pathlen] = '\0';*/
			/*RS_LOG("tsbd,4,%s\n", buffer);*/
		}
	#endif

		if (dev_name_len) {
			memcpy(&buffer[pathlen], dev_name, dev_name_len);
			pathlen += dev_name_len;
			if (verify_link_exist) {
				memcpy(&buffer[pathlen], check_strings[csSlashSda].str, check_strings[csSlashSda].str_len);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(&buffer[pathlen], check_strings[csSlashSda].str_len, csSlashSda);
			#endif
				/*pathlen += check_strings[csSlashSda].str_len;*/
			}
		} else if (verify_link_exist) {
			memcpy(&buffer[pathlen], check_strings[csSlashSda].str + 1, check_strings[csSlashSda].str_len - 1);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt_ex(&buffer[pathlen], 1, check_strings[csSlashSda].str_len - 1, csSlashSda);
		#endif
			/*pathlen += (check_strings[csSlashSda].str_len - 1);*/
		}

		/*buffer[pathlen] = '\0';*/
		/*RS_LOG("tsbd,5,%s\n", buffer);*/

	#if defined(RS_UFS_DEV_PATH_HAS_SOC)
		if ((!dev_name_has_slash) && (dev_name_len)) {
			dev_name_len += check_strings[csSocSlash].str_len;
		}
	#endif
	} else
#endif
	{
	#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
		if (strchr_len(dev_name, '/', dev_name_len)) {
			RS_LOG("tsbd,6\n");
			dev_name_has_slash = 1;
		}

		if (RS_BUFFER_SIZE < (pathlen + ((dev_name_has_slash) ? 0 : check_strings[csSocSlash].str_len)
			+ dev_name_len + ((verify_link_exist) ? (check_strings[csSlashMMCBlk0].str_len + 1) : 0))) {
			/*buffer overrun*/
			RS_LOG("tsbd,6.1\n");
			return 0; /*-ENOBUFS;*/
		}
	#else
		if (RS_BUFFER_SIZE < (pathlen + dev_name_len + ((verify_link_exist) ? (check_strings[csSlashMMCBlk0].str_len + 1) : 0))) {
			/*buffer overrun*/
			RS_LOG("tsbd,6\n");
			return 0; /*-ENOBUFS;*/
		}
	#endif

		memcpy(buffer, check_strings[csBlkPlatformSlash].str, pathlen);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(buffer, pathlen, csBlkPlatformSlash);
	#endif

	#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
		if ((!dev_name_has_slash) && (dev_name_len)) {
			memcpy(&buffer[pathlen], check_strings[csSocSlash].str, check_strings[csSocSlash].str_len);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(&buffer[pathlen], check_strings[csSocSlash].str_len, csSocSlash);
		#endif
			pathlen += check_strings[csSocSlash].str_len;

			/*buffer[pathlen] = '\0';*/
			/*RS_LOG("tsbd,6.2,%s\n", buffer);*/
		}
	#endif

		if (dev_name_len) {
			memcpy(&buffer[pathlen], dev_name, dev_name_len);
			pathlen += dev_name_len;
			if (verify_link_exist) {
				memcpy(&buffer[pathlen], check_strings[csSlashMMCBlk0].str, check_strings[csSlashMMCBlk0].str_len + 1);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(&buffer[pathlen], check_strings[csSlashMMCBlk0].str_len, csSlashMMCBlk0);
			#endif

				/*pathlen += check_strings[csSlashMMCBlk0].str_len;*/
			}
		} else if (verify_link_exist) {
			memcpy(&buffer[pathlen], check_strings[csSlashMMCBlk0].str + 1, check_strings[csSlashMMCBlk0].str_len);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt_ex(&buffer[pathlen], 1, check_strings[csSlashMMCBlk0].str_len - 1, csSlashMMCBlk0);
		#endif

			/*pathlen += (check_strings[csSlashMMCBlk0].str_len - 1);*/
		}

		/*buffer[pathlen] = '\0';*/
		/*RS_LOG("tsbd,7,%s\n", buffer);*/

	#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
		if ((!dev_name_has_slash) && (dev_name_len)) {
			dev_name_len += check_strings[csSocSlash].str_len;
		}
	#endif
	}

	if (((verify_link_exist) && (is_file_exist(buffer)))
		|| ((!verify_link_exist) && (is_dir_exist(buffer)))) {
		RS_LOG("tsbd,8,%d\n", (int)dev_name_len);
		if (dev_name_len < BLOCK_PLATFORM_DEVNAME_MAXLEN) {
			RS_LOG("tsbd,9\n");
			g_block_platform_devname = g_block_platform_devname_storage;
		} else {
			g_block_platform_devname = kmalloc(dev_name_len + 1, GFP_KERNEL);
		}

		if (g_block_platform_devname) {
			RS_LOG("tsbd,a\n");
			g_block_platform_devname_len = dev_name_len;

		#if defined(RS_USE_UFS_DEV)
			if ((g_no_emmc > 0) && (
			#if defined(RS_UFS_DEV_PATH_HAS_SOC)
				(!dev_name_has_slash) && 
			#endif
				(dev_name_len))) {
				RS_LOG("tsbd,b\n");
			#if defined(RS_UFS_DEV_PATH_HAS_SOC)
				pathlen = check_strings[csSocSlash].str_len;
				memcpy(g_block_platform_devname, check_strings[csSocSlash].str, pathlen);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(g_block_platform_devname, pathlen, csSocSlash);
			#endif
				memcpy(&g_block_platform_devname[pathlen], dev_name, (dev_name_len - pathlen));
			#else
				memcpy(g_block_platform_devname, dev_name, dev_name_len);
			#endif
			} else
		#endif
			{
				RS_LOG("tsbd,c\n");
			#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
				if (
				#if defined(RS_USE_UFS_DEV)
					(g_no_emmc <= 0) &&
				#endif
					(!dev_name_has_slash) && (dev_name_len)) {
					RS_LOG("tsbd,d\n");
					pathlen = check_strings[csSocSlash].str_len;
					memcpy(g_block_platform_devname, check_strings[csSocSlash].str, pathlen);
				#if defined(RS_ENCRYPT_STR)
					simple_str_encrypt(g_block_platform_devname, pathlen, csSocSlash);
				#endif
					memcpy(&g_block_platform_devname[pathlen], dev_name, (dev_name_len - pathlen));
				} else
			#endif
				{
					memcpy(g_block_platform_devname, dev_name, dev_name_len);
				}
			}

			g_block_platform_devname[dev_name_len] = '\0';
		}

		return 1;
	} else {
		return 0;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_block_platform_devname_by_enumeration RS_HIDE(_c2)
#endif
noused notrace static int get_block_platform_devname_by_enumeration(char *buffer)
{
	int error;
	char *dir_name;
#if !defined(RS_READ_DIR_USE_FILP)
	struct path dir_path;
	struct inode *dir;
#endif
	struct file *file;
	struct build_names_params params; /* = { .names = LIST_HEAD_INIT(.names), .buffer = NULL};*/
	extern void fput(struct file *);

	if (!buffer) {
		return -EINVAL; /*-ENOBUFS;*/
	}

#if defined(RS_USE_UFS_DEV)
	if (g_no_emmc > 0) {
		if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		#if defined(RS_UFS_DEV_PATH_HAS_SOC)
			+ check_strings[csSocSlash].str_len
		#endif
		)) {
			/*buffer overrun*/
			RS_LOG("gbpd -1\n");
			return -EINVAL; /*-ENOBUFS;*/
		}
	} else
#endif
	{
	#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
		if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len + check_strings[csSocSlash].str_len)) {
			/*buffer overrun*/
			RS_LOG("gbpd -2\n");
			return -EINVAL; /*-ENOBUFS;*/
		}
	#else
		if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len)) {
			/*buffer overrun*/
			RS_LOG("gbpd -2\n");
			return -EINVAL; /*-ENOBUFS;*/
		}
	#endif
	}

#if 0
	dir_name = (char *)RS_GET_BUFFER();
	if (!dir_name) {
		RS_LOG("gbpd -3\n");
		error = -ENOMEM;
		goto out_free_dir;
	}
#endif

	dir_name = buffer;

#if defined(RS_USE_UFS_DEV)
	if (g_no_emmc > 0) {
		memcpy(dir_name, check_strings[csBlkPlatformSlash].str, check_strings[csBlkPlatformSlash].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(dir_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
	#endif
	#if defined(RS_UFS_DEV_PATH_HAS_SOC)
		memcpy(&dir_name[check_strings[csBlkPlatformSlash].str_len], check_strings[csSocSlash].str, check_strings[csSocSlash].str_len - 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dir_name[check_strings[csBlkPlatformSlash].str_len], check_strings[csSocSlash].str_len - 1, csSocSlash);
	#endif
		dir_name[check_strings[csBlkPlatformSlash].str_len + check_strings[csSocSlash].str_len - 1] = '\0';
	#else
		dir_name[check_strings[csBlkPlatformSlash].str_len - 1] = '\0';
	#endif
	} else
#endif
	{
		memcpy(dir_name, check_strings[csBlkPlatformSlash].str, check_strings[csBlkPlatformSlash].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(dir_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
	#endif
	#if defined(RS_EMMC_DEV_PATH_HAS_SOC)
		memcpy(&dir_name[check_strings[csBlkPlatformSlash].str_len], check_strings[csSocSlash].str, check_strings[csSocSlash].str_len - 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dir_name[check_strings[csBlkPlatformSlash].str_len], check_strings[csSocSlash].str_len - 1, csSocSlash);
	#endif
		dir_name[check_strings[csBlkPlatformSlash].str_len + check_strings[csSocSlash].str_len - 1] = '\0';
	#else /*RS_EMMC_DEV_PATH_HAS_SOC*/
		dir_name[check_strings[csBlkPlatformSlash].str_len - 1] = '\0';
	#endif /*!RS_EMMC_DEV_PATH_HAS_SOC*/
	}

	RS_LOG("gbpd -4,%s\n", dir_name);
#if defined(RS_READ_DIR_USE_FILP)

	file = local_filp_open(dir_name, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

#else

	error = kern_path(dir_name/*"/dev/block/platform"*/, RS_READ_DIR_KERN_PATH_FLAGS, &dir_path);
	if (error) {
		RS_LOG("gbpd 0,%d\n", error);
		goto out_free_dir;
	}

	dir = dir_path.dentry->d_inode;

	if (!dir || !S_ISDIR(dir->i_mode)) {
		RS_LOG("gbpd 1\n");
		error = -ENOTDIR;
		goto out;
	}

	if (!dir->i_fop) {
		RS_LOG("gbpd 2\n");
		error = -EINVAL;
		goto out;
	}
	/*
	 * Open the directory ...
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	/*struct file *dentry_open(const struct path *path, int flags, const struct cred *)*/
	file = dentry_open(&dir_path, RS_READ_DIR_OPEN_FLAGS, current_cred());
#else
	/*struct file *dentry_open(struct dentry *dentry, struct vfsmount *mnt, int flags, const struct cred *)*/
	file = dentry_open(dget(dir_path.dentry), mntget(dir_path.mnt), RS_READ_DIR_OPEN_FLAGS, current_cred());
#endif

#endif

	if (IS_ERR(file)) {
		RS_LOG("gbpd 3\n");
		error = PTR_ERR(file);
		file = NULL;
		goto out;
	}

#if defined(RS_NO_VFS_READDIR)
	if (!file->f_op->iterate)
#else
	if (!file->f_op->readdir)
#endif
	{
		RS_LOG("gbpd 4\n");
		/*error = -EINVAL;*/
		/*goto out_close;*/
	}

	params.buffer = (char *)RS_GET_BUFFER();

	if (!params.buffer) {
		RS_LOG("gbpd 5\n");
		error = -ENOMEM;
		goto out_close;
	}

#if defined(RS_NO_VFS_READDIR)
	memset(&params.ctx, 0, sizeof(params.ctx));
	*((filldir_t *)&params.ctx.actor) = build_block_platform_devnames;
#endif
	INIT_LIST_HEAD(&params.files);
	INIT_LIST_HEAD(&params.dirs);
	params.dir_path = NULL;
	params.dir_path_len = 0;
	params.dir_path_has_sep = 0;

	params.sequence = 0;

	while (1) {
		int old_seq = params.sequence;

	#if defined(RS_NO_VFS_READDIR)
		error = iterate_dir(file, &params.ctx);
	#else
		error = vfs_readdir(file, build_block_platform_devnames, &params);
	#endif

		if ((error < 0) && (!should_retry_syscall(error)))
			break;

		/*error = -ENOENT;*/
		if (old_seq == params.sequence)
			break;
	}

	if (params.sequence) {
		int done = 0;
		RS_LOG("gbpd 6\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_lock_nested(file->f_dentry->d_inode, I_MUTEX_PARENT);
	#else
		inode_lock_nested(dir_path.dentry->d_inode, I_MUTEX_PARENT);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_lock_nested(&file->f_dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#else
		mutex_lock_nested(&dir_path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#endif
#endif

		while (!list_empty(&params.dirs)) {
			struct name_node *entry;
			/*noused struct dentry *dentry;*/
			entry = list_entry(params.dirs.next, struct name_node, list);
			if (!done) {
				if (try_set_block_platform_devname(params.buffer, entry->name, entry->name_len, 0)) {
					RS_LOG("gbpd done\n");
					done = 1;
				}
			}

			RS_LOG("gbpd bin:%s\n", entry->name);

			/*cond_resched();*/

			list_del(&entry->list);
		#if defined(RS_NAME_NODE_USE_PAGE)
			if (entry->name_len > NAME_NODE_MAX_NAME_LEN)
				FREE_LARGE_NAME_NODE(entry);
			else
		#endif
			FREE_NAME_NODE(entry);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_unlock(file->f_dentry->d_inode);
	#else
		inode_unlock(dir_path.dentry->d_inode);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_unlock(&file->f_dentry->d_inode->i_mutex);
	#else
		mutex_unlock(&dir_path.dentry->d_inode->i_mutex);
	#endif
#endif
	}

	RS_FREE_BUFFER(params.buffer);
	RS_LOG("gbpd bin e\n");

out_close:
#if defined(RS_READ_DIR_USE_FILP)
	if (file) {
		filp_close(file, RS_FILP_CLOSE_ID);
	}
#else
	fput(file);
#endif

out:
#if !defined(RS_READ_DIR_USE_FILP)
	path_put(&dir_path);
#endif

out_free_dir:
#if 0
	if (dir_name) {
		RS_FREE_BUFFER(dir_name);
	}
#endif

	RS_LOG("gbpd e,%d\n", error);
	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define dir_iterate_block_platform RS_HIDE(_c2_1)
#endif
noused notrace static int dir_iterate_block_platform(struct dir_iterate_params *params)
{
	int error;

	struct dir_iterate_callback callback;
	struct file *file;

	if (!params) {
		RS_LOG("dibp,1\n");
		error = -EINVAL;
		goto out;
	}

	if (params->level > RS_ITERATE_BLK_PLATFORM_DIR_MAX_DEPTH) {
		RS_LOG("dibp,1.1\n");
		error = 0;
		goto out;
	}

	file = local_filp_open(params->dir_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

	if (!file) {
		RS_LOG("dibp,1.2\n");
		error = -EINVAL;
		goto out;
	}

	if (IS_ERR(file)) {
		RS_LOG("dibp,1.3\n");
		error = PTR_ERR(file);
		file = NULL;
		goto out;
	}

#if defined(RS_NO_VFS_READDIR)
	if (!file->f_op->iterate)
#else
	if (!file->f_op->readdir)
#endif
	{
		RS_LOG("dibp,2\n");
		/*error = -EINVAL;*/
		/*goto out_close;*/
	}

#if defined(RS_NO_VFS_READDIR)
	memset(&callback.ctx, 0, sizeof(callback.ctx));
	*((filldir_t *)&callback.ctx.actor) = fill_block_platform_name;
#endif

	callback.params = params;
	callback.sequence = 0;

	params->level++;

	while (1) {
		int old_seq = callback.sequence;

	#if defined(RS_NO_VFS_READDIR)
		error = iterate_dir(file, &callback.ctx);
	#else
		error = vfs_readdir(file, fill_block_platform_name, &callback);
	#endif

		if ((error < 0) && (!should_retry_syscall(error))) {
			RS_LOG("dibp,3,%d\n", error);
			break;
		}

		/*error = -ENOENT;*/
		if (old_seq == callback.sequence) {
			RS_LOG("dibp,4\n");
			break;
		}

		/*skip "." and ".."*/
		if (params->name[0] == '.') {
			if ((params->name[1] == '\0') || (params->name_len == 1)) {
				continue;
			} else if ((params->name[1] == '.') && ((params->name[2] == '\0') || (params->name_len == 2))) {
				continue;
			}
		}

		if (params->d_type != DT_DIR) {
			if (params->d_type == DT_LNK) {
				RS_LOG("dibp,5,%s,%d\n", params->name, params->name_len);
				if (
				#if defined(RS_USE_UFS_DEV)
					((params->name_len == (check_strings[csSlashSda].str_len - (sizeof("/") - 1)))
					&& (STRCMP_EX(params->name, check_strings[csSlashSda].str, 1, csSlashSda) == 0)) ||
				#endif
					((params->name_len == (check_strings[csSlashMMCBlk0].str_len - (sizeof("/") - 1)))
					&& (STRCMP_EX(params->name, check_strings[csSlashMMCBlk0].str, 1, csSlashMMCBlk0) == 0))
					) {
					int dir_path_len = params->dir_path_len;
					int path_len = dir_path_len + params->name_len + (sizeof("/") - 1); /*with slash*/

					params->dir_path[dir_path_len++] = '/';

					memcpy(&params->dir_path[dir_path_len], params->name, params->name_len);
					params->dir_path[path_len] = '\0';

					RS_LOG("dibp,6,%s,%ld\n", &params->dir_path[params->base_dir_len], (long)(path_len - params->base_dir_len));
					try_set_block_platform_devname(params->name, &params->dir_path[params->base_dir_len],
						(params->dir_path_len - params->base_dir_len), 0);

					params->found = 1;
					break;
				}
			}

			continue;
		}
		else
		{
			RS_LOG("dibp,6.1,%s,%d\n", params->name, params->name_len);
			if ((params->name_len == (check_strings[csSlashByName].str_len - (sizeof("/") - 1)))
				&& (STRCMP_EX(params->name, check_strings[csSlashByName].str, 1, csSlashByName) == 0)
				) {
				int dir_path_len = params->dir_path_len;
				int path_len = dir_path_len + params->name_len + (sizeof("/") - 1); /*with slash*/
				params->dir_path[dir_path_len++] = '/';

				memcpy(&params->dir_path[dir_path_len], params->name, params->name_len);
				params->dir_path[path_len] = '\0';

				RS_LOG("dibp,6.2,%s,%ld\n", &params->dir_path[params->base_dir_len], (long)(path_len - params->base_dir_len));
				try_set_block_platform_devname(params->name, &params->dir_path[params->base_dir_len],
					(params->dir_path_len - params->base_dir_len), 0);

				params->found = 1;
				break;
			}
		}

		{
			int dir_path_len = params->dir_path_len;
			int path_len = dir_path_len + params->name_len + (sizeof("/") - 1); /*with slash*/

			params->dir_path[dir_path_len++] = '/';

			memcpy(&params->dir_path[dir_path_len], params->name, params->name_len);
			params->dir_path[path_len] = '\0';

			RS_LOG("dibp,7,%s\n", params->dir_path);

	#if defined(RS_USE_UFS_DEV)
			if (g_no_emmc > 0) {
				memcpy(&params->dir_path[path_len], check_strings[csSlashSda].str, check_strings[csSlashSda].str_len + 1);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(&params->dir_path[path_len], check_strings[csSlashSda].str_len, csSlashSda);
			#endif
			}
			else
	#endif
			{
				memcpy(&params->dir_path[path_len], check_strings[csSlashMMCBlk0].str, check_strings[csSlashMMCBlk0].str_len + 1);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(&params->dir_path[path_len], check_strings[csSlashMMCBlk0].str_len, csSlashMMCBlk0);
			#endif
			}

			RS_LOG("dibp,8,%s\n", params->dir_path);

			if (is_file_exist(params->dir_path)) {
				RS_LOG("dibp,9,%s,%ld\n", &params->dir_path[params->base_dir_len], (long)(path_len - params->base_dir_len));
				try_set_block_platform_devname(params->name, &params->dir_path[params->base_dir_len], (path_len - params->base_dir_len), 0);

				params->found = 1;
				break;
			}

			if (params->level > RS_ITERATE_BLK_PLATFORM_DIR_MAX_DEPTH) {
				RS_LOG("dibp,a\n");
				break;
			}

			dir_path_len = params->dir_path_len;
			params->dir_path_len = path_len;
			params->dir_path[path_len] = '\0';

			RS_LOG("dibp,b,%s,%d\n", params->dir_path, path_len);

			error = dir_iterate_block_platform(params);
			params->dir_path_len = dir_path_len;

			if (error) {
				break;
			}
		}

		if (params->found) {
			break;
		}
	}
	params->level--;

	if (file) {
		filp_close(file, RS_FILP_CLOSE_ID);
	}

out:

	RS_LOG("dibp,e,%d\n", error);
	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_block_platform_devname_by_enumeration_recursively RS_HIDE(_c2_2)
#endif
noused notrace static int get_block_platform_devname_by_enumeration_recursively(char *buffer)
{
	int error;
	struct dir_iterate_params params;

	RS_LOG("gbpe,0\n");

	if (!buffer) {
		RS_LOG("gbpe,1\n");
		return -EINVAL; /*-ENOBUFS;*/
	}

	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len + check_strings[csSlashByName].str_len
		+ check_strings[csSystem].str_len + BLOCK_PLATFORM_DEVNAME_MAXLEN)) {
		/*buffer overrun*/
		RS_LOG("gbpe,2\n");
		return -EINVAL; /*-ENOBUFS;*/
	}

	params.dir_path = (char *)RS_GET_BUFFER();
	if (!params.dir_path) {
		RS_LOG("gbpe,3\n");
		error = -ENOMEM;
		goto out;
	}

	params.name = buffer;
	params.name_len = 0;
	params.d_type = 0;
	params.level = 0;
	params.found = 0;

	{
		int len = check_strings[csBlkPlatformSlash].str_len;

		/*including slash*/
		params.base_dir_len = len;
		len -= (sizeof("/") - 1);
		params.dir_path_len = len;

		memcpy(params.dir_path, check_strings[csBlkPlatformSlash].str, len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(params.dir_path, len, csBlkPlatformSlash);
	#endif
		params.dir_path[len] = '\0';
	}

	RS_LOG("gbpe,4,%s,%d\n", params.dir_path, params.base_dir_len);
	error = dir_iterate_block_platform(&params);

	RS_FREE_BUFFER(params.dir_path);

out:

	RS_LOG("gbpe,e,%d\n", error);
	return error;
}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/
/*#endif*/

#if defined(CONFIG_DO_RS_JOURNAL)

/*#include <generated/compile.h>*/
/*UTS_VERSION,*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_mount_check_env_info RS_HIDE(_CtD)
#endif
noused notrace static int rs_init_mount_check_env_info(void);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_version_str RS_HIDE(_c3)
#endif
noused notrace static int __init rs_init_version_str(void)
{
	noused extern char *saved_command_line;
	noused char *p_version;

	if (g_software_version_str) {
		return 0;
	}

	p_version = STRSTR(saved_command_line, check_strings[csProductVersion].str, csProductVersion);
	if (p_version) {
		int ver_len;
		char *tmp_str;

		p_version += check_strings[csProductVersion].str_len;
	#if 0
		tmp_str = _strchr(p_version, '_');
		if (!tmp_str)
			tmp_str = p_version;
		else
			tmp_str++;
	#else
		tmp_str = p_version;
	#endif

		ver_len = cfg_strlen(tmp_str);
		if (ver_len) {
			g_software_version_str_len = ver_len;
			g_software_version_str = tmp_str;
		}
	}

	if (!g_software_version_str) {
		/*"#21 SMP PREEMPT Mon Apr 20 17:52:56 CST 2015"*/
		char *ver_str = init_utsname()->version;
		int ver_len = strlen(ver_str);
		/*if (ver_len > (23 + 4))*/
		{
			/*ver_str += (ver_len - 28);*/

			int space_count = 0;
			int non_space_count = 0;
			int got_non_space = 0;
			char *ver_end = ver_str + ver_len - 1;
			while (ver_end >= ver_str) {
				if (*ver_end <= ' ') {
					got_non_space = 0;

					space_count++;
					if (space_count == 5) {
						ver_end++;
						break;
					}
				} else {
					if (!got_non_space) {
						got_non_space = 1;

						non_space_count++;
					}
				}

				ver_end--;
			}

			if ((ver_end > ver_str) && (non_space_count == 5)) {
				ver_len = strlen(ver_end);
				if (ver_len) {
					g_software_version_str_len = ver_len;
					g_software_version_str = ver_end;
				}
			}
		}
	}

	return 0;
}

late_initcall(rs_init_version_str); /*module_init(rs_init_version_str);*/
#endif

#if defined(CONFIG_CMDLINE_RS_CONFIG)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init RS_HIDE(_c4)
#endif
noused notrace static int __init rs_init(void)
{
	noused extern char *saved_command_line;
	noused char *p_rsinfo;
	char *buffer;

	buffer = NULL;

#if RS_DEBUG
#if defined(CONFIG_CONFIG_RS_LOG)
	p_rsinfo = STRSTR(saved_command_line, check_strings[csRSLog].str, csRSLog);
	if (p_rsinfo) {
		int res;
		unsigned long result = 0;

		p_rsinfo += check_strings[csRSLog].str_len;

		res = rs_parse_config_number(p_rsinfo, &result);
		if (res == 0) {
			/*ok*/
			g_rs_enable_log = result;
		}
	}
#endif
#endif

#if defined(CONFIG_CONFIG_RS_JOURNAL)
	p_rsinfo = STRSTR(saved_command_line, check_strings[csRSJournal].str, csRSJournal);
	if (p_rsinfo) {
		int res;
		unsigned long result = 0;

		p_rsinfo += check_strings[csRSJournal].str_len;

		res = rs_parse_config_number(p_rsinfo, &result);
		if (res == 0) {
			/*ok*/
			g_rs_enable_journal = result;
		}
	}
#endif

#if defined(CONFIG_CONFIG_RS_PERMIT)
	p_rsinfo = STRSTR(saved_command_line, check_strings[csRSPermit].str, csRSPermit);
	if (p_rsinfo) {
		unsigned long local_rs_permit = 0;

		p_rsinfo += check_strings[csRSPermit].str_len;

		if (rs_parse_config_flags(p_rsinfo, &local_rs_permit, &buffer) >= 0) {
			set_permit_vars(&local_rs_permit);
		}
	}
#endif

#if defined(CONFIG_CONFIG_RS_BYPASS)
	p_rsinfo = STRSTR(saved_command_line, check_strings[csRSByPass].str, csRSByPass);
	if (p_rsinfo) {
		unsigned long local_rs_bypass = 0;

		p_rsinfo += check_strings[csRSByPass].str_len;

		if (rs_parse_config_flags(p_rsinfo, &local_rs_bypass, &buffer) >= 0) {
			set_bypass_vars(&local_rs_bypass);
		}
	}
#endif

	if (buffer) {
		RS_FREE_BUFFER(buffer);
	}

	{
	#if 0
		noused char *p_version;

		p_version = strnstr(saved_command_line, "product.version=", strlen(saved_command_line));
		if (p_version) {
			int ver_len;
			p_version += (sizeof("product.version=") - 1);
			ver_len = cfg_strlen(p_version);
			if ((rs_len == ver_len) && (rs_len)) {
				if (strncmp(p_rsinfo, p_version, rs_len) == 0) {
					g_rs_bypass = 1;
				}
			}
		} else if ((rs_len == 3) && (strncmp(p_rsinfo, "yes", rs_len) == 0)) {
			g_rs_bypass = 1;
		}
	#endif
	}

	return 0;
}

late_initcall_sync(rs_init); /*module_init(rs_init);*/

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_readlink RS_HIDE(_c41)
#endif
noused notrace static int __sys_readlink(const char *path, char *buf, int bufsiz)
{
	ssize_t ret;
	mm_segment_t old_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_readlink(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)path;
		arg_regs.regs[1] = (u64)buf;
		arg_regs.regs[2] = (u64)bufsiz;

		ret = __arm64_sys_readlink((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_readlink((char __user *)path, (char __user *)buf, bufsiz);
#endif
	set_fs(old_fs);
	return ret;
}

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_block_platform_devname_by_bootdevice_link RS_HIDE(_c42)
#endif
noused notrace static int get_block_platform_devname_by_bootdevice_link(char *buffer)
{
#if defined(RS_QUALCOMM_PLATFORM)
	/* /dev/block/bootdevice -> /dev/block/platform/soc/624000.ufshc or /dev/block/platform/soc/7824900.sdhci*/
	int ret;
	/*char *buffer;*/
	char *path, *link;
	int plat_len, link_len;

	if ((!buffer) || (RS_BUFFER_SIZE <= check_strings[csBlkPlatformSlash].str_len)
		|| (RS_BUFFER_SIZE <= check_strings[csBlkBootDevice].str_len)) {
		/*RS_LOG("gbdb,0\n");*/
		ret = -EINVAL;
		goto out_no_buffer;
	}

#if 0
	buffer = (char *)RS_GET_TWO_BUFFER();
	if (!buffer) {
		/*RS_LOG("gbdb,1\n");*/
		ret = -ENOMEM;
		goto out;
	}
#endif

	path = buffer;
	/*link = path + RS_BUFFER_SIZE;*/
	link = (char *)RS_GET_BUFFER();
	if (!link) {
		/*RS_LOG("gbdb,0.1\n");*/
		ret = -ENOMEM;
		goto out;
	}

	memcpy(path, check_strings[csBlkBootDevice].str, check_strings[csBlkBootDevice].str_len + 1);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(buffer, check_strings[csBlkBootDevice].str_len, csBlkBootDevice);
#endif

	ret = __sys_readlink(path, link, (int)RS_BUFFER_SIZE);
	if (ret <= 0) {
		/*RS_LOG("gbdb,2\n");*/
		ret = -EINVAL;
		goto out;
	}

	/*RS_LOG("gbdb,3\n");*/
	link_len = ret;
	link[link_len] = '\0';
	plat_len = check_strings[csBlkPlatformSlash].str_len;
	if (plat_len < link_len) {
		/*RS_LOG("gbdb,4\n");*/
		if (STRNCMP(link, check_strings[csBlkPlatformSlash].str, plat_len, csBlkPlatformSlash) == 0) {
			/*RS_LOG("gbdb,5\n");*/
			path = (link + plat_len);

			link_len -= plat_len;

			try_set_block_platform_devname(buffer, path, link_len, 0);

			ret = 0;
		}
	}

out:
#if 0
	if (buffer) {
		RS_FREE_TWO_BUFFER(buffer);
	}
#endif

	if (link) {
		RS_FREE_BUFFER(link);
	}
out_no_buffer:

	/*RS_LOG("gbdb,e\n");*/
	return ret;
#else
	/* /dev/bootimg -> /dev/block/platform/mtk-msdc.0/by-name/boot*/
	int ret;
	/*char *buffer;*/
	char *path, *link;
	int plat_len, link_len;

	if ((!buffer) || (RS_BUFFER_SIZE <= (check_strings[csDevSlash].str_len
		+ check_strings[csBootImg].str_len))
		) {
		RS_LOG("gbdb,0\n");
		ret = -EINVAL;
		goto out_no_buffer;
	}

#if 0
	buffer = (char *)RS_GET_TWO_BUFFER();
	if (!buffer) {
		RS_LOG("gbdb,1\n");
		ret = -ENOMEM;
		goto out;
	}
#endif

	path = buffer;
	/*link = path + RS_BUFFER_SIZE;*/
	link = (char *)RS_GET_BUFFER();
	if (!link) {
		/*RS_LOG("gbdb,0.1\n");*/
		ret = -ENOMEM;
		goto out;
	}

	/* /dev/bootimg*/
	memcpy(path, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(buffer, check_strings[csDevSlash].str_len, csDevSlash);
#endif
	memcpy(&path[check_strings[csDevSlash].str_len], check_strings[csBootImg].str, check_strings[csBootImg].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&path[check_strings[csDevSlash].str_len], check_strings[csBootImg].str_len, csBootImg);
#endif

	RS_LOG("gbdb,1.1,%s\n", path);

	ret = __sys_readlink(path, link, (int)RS_BUFFER_SIZE);
	if (ret <= 0) {
		RS_LOG("gbdb,2\n");
		ret = -EINVAL;
		goto out;
	}

	RS_LOG("gbdb,3\n");
	link_len = ret;
	link[link_len] = '\0';
	plat_len = check_strings[csBlkPlatformSlash].str_len;
	if (plat_len < link_len) {
		RS_LOG("gbdb,4\n");
		if (STRNCMP(link, check_strings[csBlkPlatformSlash].str, plat_len, csBlkPlatformSlash) == 0) {
			RS_LOG("gbdb,5\n");
			path = (link + plat_len);

			link_len -= plat_len;

			{
				/* /by-name/boot*/
				char *by_name_part = STRSTR(path, check_strings[csSlashByName].str, csSlashByName);
				if (by_name_part) {
					int by_name_part_len = strlen(by_name_part);
					RS_LOG("gbdb,6\n");

					link_len -= by_name_part_len;
				}
			}

			try_set_block_platform_devname(buffer, path, link_len, 0);

			ret = 0;
		}
	}

out:
#if 0
	if (buffer) {
		RS_FREE_TWO_BUFFER(buffer);
	}
#endif

	if (link) {
		RS_FREE_BUFFER(link);
	}
out_no_buffer:

	RS_LOG("gbdb,e,%d\n", ret);
	return ret;
#endif
}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(RS_USE_UFS_DEV)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_if_no_emmc RS_HIDE(_c43)
#endif
noused notrace static int check_if_no_emmc(char *buffer)
{
	int len;
	char *local_buffer;
	if (g_no_emmc) {
		return g_no_emmc;
	}

	RS_LOG("cine,0\n");

#if defined(RS_QUALCOMM_PLATFORM)
	{
		noused extern char *saved_command_line;
		char *p_boot_emmc;

		p_boot_emmc = STRSTR(saved_command_line, check_strings[csAndroidBootEmmc].str, csAndroidBootEmmc);
		if (p_boot_emmc) {
			int value_len;
			p_boot_emmc += check_strings[csAndroidBootEmmc].str_len;
			value_len = cfg_strlen(p_boot_emmc);
			if (value_len == check_strings[csTrue].str_len) {
				if (STRNCMP(p_boot_emmc, check_strings[csTrue].str, value_len, csTrue) == 0) {
					RS_LOG("cine,1\n");
					g_no_emmc = -1;
					goto out;
				}
			}
		}
	}
#endif

	if (!buffer) {
		local_buffer = (char *)RS_GET_BUFFER();
		if (!local_buffer) {
			RS_LOG("cine,2\n");
			return 0;
		}
	} else {
		local_buffer = buffer;
	}

	/* check if /dev/block exist*/
	len = sizeof("/dev/block") - 1;
	memcpy(local_buffer, check_strings[csMmcblk0p].str, len);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(local_buffer, len, csMmcblk0p);
#endif
	local_buffer[len] = '\0';

	if (!is_dir_exist(local_buffer)) {
		/*RS_LOG("cine,3,%s\n", local_buffer);*/

		if (!buffer) {
			RS_FREE_BUFFER(local_buffer);
		} else {
			buffer[0] = '\0';
		}

		{
			noused int partno;
			noused struct gendisk *disk;
		#if defined(RS_DISK_OWNER_MODULE_PUT)
			const struct block_device_operations *fops;
			struct module *owner;
		#endif
			dev_t ufs_dev;

			ufs_dev = MKDEV(SCSI_DISK0_MAJOR, 0); /*get disk_type, which is static var in genhd.c*/

			/*partno = 0;*/
			disk = get_gendisk(ufs_dev, &partno);

			if (!disk) {
				/*RS_LOG("cine,4\n");*/
				g_no_emmc = -1;
			} else {
			#if defined(RS_DISK_OWNER_MODULE_PUT)
				fops = disk->fops;
				if (fops) {
					owner = (struct module *)(fops->owner);

					if (owner)
						module_put(owner);
				}
			#endif

				put_disk(disk);

				/*RS_LOG("cine,5\n");*/
				g_no_emmc = 1;
			}
		}

		return g_no_emmc;
	}

	/* check if /dev/block/mmcblk0 exist*/
	len = check_strings[csMmcblk0p].str_len - 1;
	memcpy(local_buffer, check_strings[csMmcblk0p].str, len);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(local_buffer, len, csMmcblk0p);
#endif
	local_buffer[len] = '\0';

	if (!is_file_exist(local_buffer)) {
		RS_LOG("cine,6\n");
		g_no_emmc = 1;
	} else {
		RS_LOG("cine,7\n");
		g_no_emmc = -1;
	}

	if (!buffer) {
		RS_FREE_BUFFER(local_buffer);
	} else {
		buffer[0] = '\0';
	}

#if defined(RS_QUALCOMM_PLATFORM)
out:
#endif
	return g_no_emmc;
}
#endif

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_block_platform_devname RS_HIDE(_c5)
#endif
noused notrace static void init_block_platform_devname(const char *dev_name)
{
	char *buffer;

#if defined(RS_QUALCOMM_PLATFORM)
	noused extern char *saved_command_line;
	noused char *p_rsinfo;
#endif

	/*
	if (g_block_platform_devname) {
		return;
	}
	*/

	buffer = (char *)RS_GET_BUFFER();
	if (!buffer) {
		RS_LOG("ibdb,0\n");
		return;
	}

#if defined(RS_USE_UFS_DEV)
	RS_LOG("ibdb,0.1\n");
	check_if_no_emmc(buffer);
#endif

#if defined(RS_QUALCOMM_PLATFORM)
	p_rsinfo = STRSTR(saved_command_line, check_strings[csBootDevicePrefix].str,
		csBootDevicePrefix);
	if (p_rsinfo) {
		size_t len;
		RS_LOG("ibdb,2\n");
		p_rsinfo += check_strings[csBootDevicePrefix].str_len;

		len = cfg_strlen(p_rsinfo);
		if (len > 0) {
			RS_LOG("ibdb,3\n");
			try_set_block_platform_devname(buffer, p_rsinfo, len, 0);
		}
	}
#endif

	if (!g_block_platform_devname) {
		RS_LOG("ibdb,4\n");

	#if 1
		if (STRNCMP(dev_name, check_strings[csBlkPlatformSlash].str,
			check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash) == 0) {
			int skip_len = check_strings[csBlkPlatformSlash].str_len;
			const char *s = dev_name + skip_len;
			char *e = STRSTR(s, check_strings[csSlashByName].str, csSlashByName);

			RS_LOG("ibdb,5,%s\n", s);

			if (e) {
				RS_LOG("ibdb,6,%s,%ld\n", e, (long)(e - s));
				try_set_block_platform_devname(buffer, s, (e - s), 0);
			}
		}
	#endif

		if (!g_block_platform_devname) {
			RS_LOG("ibdb,8\n");
			get_block_platform_devname_by_bootdevice_link(buffer);

			if (!g_block_platform_devname) {
				RS_LOG("ibdb,9\n");
			#if defined(RS_ITERATE_BLK_PLATFORM_DIR_RECURSIVELY)
				get_block_platform_devname_by_enumeration_recursively(buffer);
			#else
				get_block_platform_devname_by_enumeration(buffer);
			#endif
			}
		}
	}

	RS_LOG("ibdb,e,%s\n", g_block_platform_devname);

	RS_FREE_BUFFER(buffer);
}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(CONFIG_BLOCK_DEV_RS_CONFIG)

#define RS_DATA_BLOCK_MAGIC (0x52734462)

#define RS_DEV_ID_SIZE (32) /*16字节，作为字符串则32个字符*/

#define RS_EMMC_DEV_ID_SIZE (32)

typedef union rs_emmc_dev_id_struct {
	__u16 u16_vals[RS_EMMC_DEV_ID_SIZE / sizeof(__u16)];
	char u8_vals[RS_EMMC_DEV_ID_SIZE];
} rs_emmc_dev_id_t;

#define RS_UFS_DEV_ID_SIZE (64)

typedef union rs_ufs_dev_id_struct {
	__u16 u16_vals[RS_UFS_DEV_ID_SIZE / sizeof(__u16)];
	char u8_vals[RS_UFS_DEV_ID_SIZE];
} rs_ufs_dev_id_t;

typedef union rs_dev_id_struct {
	rs_emmc_dev_id_t emmc_id;
	rs_ufs_dev_id_t ufs_id;
} rs_dev_id_t;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_dev_id RS_HIDE(_d0)
#endif
noused static char g_rs_dev_id[RS_DEV_ID_SIZE];

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define char2nibble RS_HIDE(_d01)
#endif
noused notrace static char char2nibble(char ch)
{
	ch = TOLOWER(ch);

	if (isdigit(ch)) {
		ch = (ch - '0');
	} else if ((ch >= 'a') && (ch <= 'f')) {
		ch = ((ch - 'a') + 10);
	} else {
		ch = (ch & 0x0f);
	}

	return ch;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define isHexChar RS_HIDE(_d02)
#endif
noused notrace static bool isHexChar(char ch)
{
	bool ret = false;
	ch = TOLOWER(ch);

	if (isdigit(ch)) {
		ret = true;
	} else if ((ch >= 'a') && (ch <= 'f')) {
		ret = true;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_rs_dev_id RS_HIDE(_d1)
#endif
noused notrace static int init_rs_dev_id(void)
{
	char *dev_name = NULL;
	struct file *filp = NULL;
#if defined(RS_IS_ANDROID_9_ABOVE)
	const char *cmdline_device_id = NULL;
	size_t cmdline_device_id_len = 0;
#endif
	int se_flag = 0;
	rs_dev_id_t *p_dev_id_1, *p_dev_id_2;
	int ret;

	RS_LOG("irdi,0\n");

	p_dev_id_1 = (rs_dev_id_t *)rs_kmalloc(sizeof(rs_dev_id_t) * 2);
	if (!p_dev_id_1) {
		RS_LOG("irdi,1\n");
		ret = -ENOMEM;
		goto out_no_file;
	}

	p_dev_id_2 = p_dev_id_1 + 1;

#if defined(RS_IS_ANDROID_9_ABOVE)
	cmdline_device_id = STRSTR(saved_command_line, check_strings[csBootDeviceIDPrefix].str,
		csBootDeviceIDPrefix);
	if (cmdline_device_id) {
		bool cmd_dev_id_valid = true;
		
		RS_LOG("irdi,1.1\n");
		cmdline_device_id += check_strings[csBootDeviceIDPrefix].str_len;

		cmdline_device_id_len = cfg_strlen(cmdline_device_id);
		if ((cmdline_device_id_len >= RS_EMMC_DEV_ID_SIZE) && (isHexChar(cmdline_device_id[0]))
			&& (isHexChar(cmdline_device_id[cmdline_device_id_len - 1]))) {
		#if defined(RS_USE_UFS_DEV)
			if (g_no_emmc) {
				if (cmdline_device_id_len <= RS_EMMC_DEV_ID_SIZE) {
					cmd_dev_id_valid = false;
				}
			} else
		#endif
			{
				if (cmdline_device_id_len > RS_EMMC_DEV_ID_SIZE) {
					cmd_dev_id_valid = false;
				}
			}
		} else {
			cmd_dev_id_valid = false;
		}
		
		if (!cmd_dev_id_valid) {
			RS_LOG("irdi,1.2\n");
			cmdline_device_id = NULL;
			cmdline_device_id_len = 0;
		}
	}
#endif

#if defined(RS_IS_ANDROID_9_ABOVE)
	if (!cmdline_device_id) {
#endif
		if ((RS_BUFFER_SIZE < (check_strings[csMMCBlk0CID].str_len + 1))
		#if defined(RS_USE_UFS_DEV)
			|| (RS_BUFFER_SIZE < (check_strings[csUFSDeviceID].str_len + 1))
		#endif
			) {
			/*buffer overrun*/
			RS_LOG("irdi,2\n");
			ret = -EINVAL; /*-ENOBUFS;*/
			goto out_no_file;
		}

		dev_name = (char *)RS_GET_BUFFER(); /*"/sys/block/mmcblk0/device/cid";*/
		if (!dev_name) {
			RS_LOG("irdi,3\n");
			ret = -ENOMEM;
			goto out_no_file;
		}

	#if defined(RS_USE_UFS_DEV)
		if (g_no_emmc > 0) {
			memcpy(dev_name, check_strings[csUFSDeviceID].str, check_strings[csUFSDeviceID].str_len + 1);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(dev_name, check_strings[csUFSDeviceID].str_len, csUFSDeviceID);
		#endif
		} else
	#endif
		{
			memcpy(dev_name, check_strings[csMMCBlk0CID].str, check_strings[csMMCBlk0CID].str_len + 1);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(dev_name, check_strings[csMMCBlk0CID].str_len, csMMCBlk0CID);
		#endif
		}

		se_flag = test_and_set_thread_flag((BITS_PER_LONG - 1));

		filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
		if (IS_ERR(filp)) {
			RS_LOG("irdi,4,%s\n", dev_name);
			ret = PTR_ERR(filp);
			filp = NULL;
			goto out;
		}
#if defined(RS_IS_ANDROID_9_ABOVE)
	}
#endif

#if defined(RS_USE_UFS_DEV)
	if (g_no_emmc > 0) {
		int i;
		ssize_t ssize_res;
		RS_LOG("irdi,5\n");

	#if defined(RS_IS_ANDROID_9_ABOVE)
		if (!cmdline_device_id) {
	#endif
			ssize_res = safe_vfs_read(filp, p_dev_id_1->ufs_id.u8_vals, sizeof(p_dev_id_1->ufs_id.u8_vals), 0);

			if (ssize_res <= 0) {
				RS_LOG("irdi,6\n");
				ret = ssize_res;
				goto out;
			}
	#if defined(RS_IS_ANDROID_9_ABOVE)
		} else {
			if (cmdline_device_id_len > sizeof(p_dev_id_1->ufs_id.u8_vals)) {
				ssize_res = (ssize_t)sizeof(p_dev_id_1->ufs_id.u8_vals);
			} else {
				ssize_res = (ssize_t)cmdline_device_id_len;
			}
			memcpy(p_dev_id_1->ufs_id.u8_vals, cmdline_device_id, ssize_res);
		}
	#endif

		if (ssize_res < sizeof(p_dev_id_1->ufs_id.u8_vals)) {
			/*为了root脚本方便,不让出现值为'\0'的字符*/
			memset(&p_dev_id_1->ufs_id.u8_vals[ssize_res], 'f', (sizeof(p_dev_id_1->ufs_id.u8_vals) - ssize_res));
		}

	#if 0
	#if defined(RS_IS_ANDROID_9_ABOVE)
		if (dev_name) {
	#endif
			memcpy(dev_name, p_dev_id_1->ufs_id.u8_vals, sizeof(p_dev_id_1->ufs_id.u8_vals));
			dev_name[sizeof(p_dev_id_1->ufs_id.u8_vals)] = '\0';
			RS_LOG("irdi,6.2,%s\n", dev_name);
	#if defined(RS_IS_ANDROID_9_ABOVE)
		}
	#endif
	#endif

		/*64字节压缩到32字节,改为按emmc_id(32字节)来使用*/
		for (i = 0; i < ARRAY_SIZE(p_dev_id_2->ufs_id.u16_vals); i++) {
			char ch = (char)(char2nibble(p_dev_id_1->ufs_id.u8_vals[i * 2])
				| (char2nibble(p_dev_id_1->ufs_id.u8_vals[(i * 2) + 1]) << 4));

			/*必须为可打印的字符*/
			if (ch <= ' ') {
				ch = ' ' + ch + i + 1;
			}

			p_dev_id_2->ufs_id.u8_vals[i] = ch;
		}

	#if 0
	#if defined(RS_IS_ANDROID_9_ABOVE)
		if (dev_name)
	#endif
		{
			int len = 0;
			for (i = 0; i < RS_EMMC_DEV_ID_SIZE; i++) {
				len += scnprintf(&dev_name[len], (RS_BUFFER_SIZE - len), "%02x",  p_dev_id_2->emmc_id.u8_vals[i]);
			}

			RS_LOG("irdi,6.3,%s\n", dev_name);
		}
	#endif

		/*每两个字节互换*/
		for (i = 0; i < ARRAY_SIZE(p_dev_id_1->emmc_id.u16_vals); i++) {
			p_dev_id_1->emmc_id.u16_vals[i] =
				(__u16)(((p_dev_id_2->emmc_id.u16_vals[i] >> 8) & 0xffu) | ((p_dev_id_2->emmc_id.u16_vals[i] & 0xffu) << 8));
		}

	#if 0
	#if defined(RS_IS_ANDROID_9_ABOVE)
		if (dev_name)
	#endif
		{
			int len = 0;
			for (i = 0; i < RS_EMMC_DEV_ID_SIZE; i++) {
				len += scnprintf(&dev_name[len], (RS_BUFFER_SIZE - len), "%02x",  p_dev_id_1->emmc_id.u8_vals[i]);
			}

			RS_LOG("irdi,6.4,%s\n", dev_name);
		}
	#endif
	} else
#endif
	{
		int i;
		char ch;
		ssize_t ssize_res;
		RS_LOG("irdi,7\n");

	#if defined(RS_IS_ANDROID_9_ABOVE)
		if (!cmdline_device_id) {
	#endif
			ssize_res = safe_vfs_read(filp, p_dev_id_1->emmc_id.u8_vals, sizeof(p_dev_id_1->emmc_id.u8_vals), 0);

			if (ssize_res <= 0) {
				RS_LOG("irdi,8\n");
				ret = ssize_res;
				goto out;
			}
	#if defined(RS_IS_ANDROID_9_ABOVE)
		} else {
			if (cmdline_device_id_len > sizeof(p_dev_id_1->emmc_id.u8_vals)) {
				ssize_res = (ssize_t)sizeof(p_dev_id_1->emmc_id.u8_vals);
			} else {
				ssize_res = (ssize_t)cmdline_device_id_len;
			}
			memcpy(p_dev_id_1->emmc_id.u8_vals, cmdline_device_id, ssize_res);
		}
	#endif

		if (ssize_res < sizeof(p_dev_id_1->emmc_id.u8_vals)) {
			memset(&p_dev_id_1->emmc_id.u8_vals[ssize_res], 'f', (sizeof(p_dev_id_1->emmc_id.u8_vals) - ssize_res));
		}

		for (i = 0; i < ARRAY_SIZE(p_dev_id_1->emmc_id.u8_vals); i++) {
			ch = p_dev_id_1->emmc_id.u8_vals[i];
			if (ch <= ' ') {
				ch = ' ' + ch + i + 1;
				p_dev_id_1->emmc_id.u8_vals[i] = ch;
			}
		}

		/*每两个字节互换*/
		for (i = 0; i < ARRAY_SIZE(p_dev_id_1->emmc_id.u16_vals); i++) {
			p_dev_id_1->emmc_id.u16_vals[i] =
				(__u16)(((p_dev_id_1->emmc_id.u16_vals[i] >> 8) & 0xffu) | ((p_dev_id_1->emmc_id.u16_vals[i] & 0xffu) << 8));
		}

	}

	/* 15 13 11 9 | 6 4 2 0 | 14 12 10 8 | 7 5 3 1 */
	memcpy(p_dev_id_2->emmc_id.u8_vals, p_dev_id_1->emmc_id.u8_vals, sizeof(p_dev_id_1->emmc_id.u8_vals));

	p_dev_id_1->emmc_id.u16_vals[0] = p_dev_id_2->emmc_id.u16_vals[15];
	p_dev_id_1->emmc_id.u16_vals[1] = p_dev_id_2->emmc_id.u16_vals[13];
	p_dev_id_1->emmc_id.u16_vals[2] = p_dev_id_2->emmc_id.u16_vals[11];
	p_dev_id_1->emmc_id.u16_vals[3] = p_dev_id_2->emmc_id.u16_vals[9];
	p_dev_id_1->emmc_id.u16_vals[4] = p_dev_id_2->emmc_id.u16_vals[6];
	p_dev_id_1->emmc_id.u16_vals[5] = p_dev_id_2->emmc_id.u16_vals[4];
	p_dev_id_1->emmc_id.u16_vals[6] = p_dev_id_2->emmc_id.u16_vals[2];
	p_dev_id_1->emmc_id.u16_vals[7] = p_dev_id_2->emmc_id.u16_vals[0];
	p_dev_id_1->emmc_id.u16_vals[8] = p_dev_id_2->emmc_id.u16_vals[14];
	p_dev_id_1->emmc_id.u16_vals[9] = p_dev_id_2->emmc_id.u16_vals[12];
	p_dev_id_1->emmc_id.u16_vals[10] = p_dev_id_2->emmc_id.u16_vals[10];
	p_dev_id_1->emmc_id.u16_vals[11] = p_dev_id_2->emmc_id.u16_vals[8];
	p_dev_id_1->emmc_id.u16_vals[12] = p_dev_id_2->emmc_id.u16_vals[7];
	p_dev_id_1->emmc_id.u16_vals[13] = p_dev_id_2->emmc_id.u16_vals[5];
	p_dev_id_1->emmc_id.u16_vals[14] = p_dev_id_2->emmc_id.u16_vals[3];
	p_dev_id_1->emmc_id.u16_vals[15] = p_dev_id_2->emmc_id.u16_vals[1];

#if 0
	#if defined(RS_IS_ANDROID_9_ABOVE)
	if (dev_name)
	#endif
	{
		int i, len = 0;
		for (i = 0; i < RS_EMMC_DEV_ID_SIZE; i++) {
			len += scnprintf(&dev_name[len], (RS_BUFFER_SIZE - len), "%02x", p_dev_id_1->emmc_id.u8_vals[i]);
		}
		RS_LOG("irdi,9,%s\n", dev_name);
	}
#endif

	memcpy(g_rs_dev_id, p_dev_id_1->emmc_id.u8_vals, sizeof(g_rs_dev_id));

	ret = 0;

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	if (dev_name) {
		RS_FREE_BUFFER(dev_name);
	}
	
#if defined(RS_IS_ANDROID_9_ABOVE)
	if (!cmdline_device_id) {
#endif
		if (!se_flag) {
			clear_thread_flag((BITS_PER_LONG - 1));
		}
#if defined(RS_IS_ANDROID_9_ABOVE)
	}
#endif

out_no_file:
	if (p_dev_id_1) {
		rs_kfree(p_dev_id_1);
	}

	RS_LOG("irdi,e,%d\n", ret);
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_rs_dev_id_from_serial RS_HIDE(_d1_)
#endif
noused notrace static int init_rs_dev_id_from_serial(void)
{
	noused extern char *saved_command_line;
	char *p_serial, *p_xor_serial;
	rs_dev_id_t *p_dev_id_1, *p_dev_id_2;
	int ret;

	p_dev_id_1 = (rs_dev_id_t *)rs_kmalloc(sizeof(rs_dev_id_t) * 2);
	if (!p_dev_id_1) {
		ret = -ENOMEM;
		goto out;
	}

	p_dev_id_2 = p_dev_id_1 + 1;
	p_xor_serial = (char *)p_dev_id_2;

	p_serial = STRSTR(saved_command_line, check_strings[csAndroidBootSerialNo].str, csAndroidBootSerialNo);
	if (p_serial) {
		size_t i, j, real_serial_len, serial_len, len;
		char new_ch, ch = 0x55;
		u32 xor_value = 0;
		u8 *p_xor_value = (u8 *)&xor_value;

		for (i = 0; i < sizeof(u32); i++) {
			if ((i & 1) == 0) {
				ch = (char)(~ch);
				xor_value = (xor_value << 8) | ch;
			} else {
				ch = (char)((~ch & 0xf0) | (ch & 0xf));
				xor_value = (xor_value << 8) | ch;
			}
		}

		p_serial += check_strings[csAndroidBootSerialNo].str_len;

		real_serial_len = cfg_strlen(p_serial);
		serial_len = ALIGN(real_serial_len, 4);
		if (serial_len > sizeof(g_rs_dev_id)) {
			serial_len = sizeof(g_rs_dev_id);
		}

		i = 0;
		len = 0;
		while ((len + serial_len) <= sizeof(g_rs_dev_id)) {
			i = 0;
			while (i < serial_len) {
				if (i >= real_serial_len) {
					ch = p_serial[i - real_serial_len];
				} else {
					ch = p_serial[i];
				}

				new_ch = ch ^ p_xor_value[i & (sizeof(xor_value) - 1)];
				/*必须为可打印的字符*/
				if (new_ch <= ' ') {
					if (ch <= ' ') {
						new_ch = ' ' + ch + len + i + 1;
					} else {
						new_ch = ch;
					}
				}

				p_xor_serial[len + i] = new_ch;

				i++;

				if ((i & (sizeof(xor_value) - 1)) == 0) {
					xor_value = ((xor_value & 0xff) << 24) | (xor_value >> 8);
				}
			}

			len += serial_len;
		}

		i = 0;
		while (len < sizeof(g_rs_dev_id)) {
			if (i >= serial_len) {
				j = i - serial_len;
				i = 0;
			} else {
				j = i;
			}

			if (j >= real_serial_len) {
				ch = p_serial[j - real_serial_len];
			} else {
				ch = p_serial[j];
			}

			new_ch = ch ^ p_xor_value[i & (sizeof(xor_value) - 1)];
			/*必须为可打印的字符*/
			if (new_ch <= ' ') {
				if (ch <= ' ') {
					new_ch = ' ' + ch + len + 1;
				} else {
					new_ch = ch;
				}
			}

			p_xor_serial[len] = new_ch;

			i++;

			if ((i & (sizeof(xor_value) - 1)) == 0) {
				xor_value = ((xor_value & 0xff) << 24) | (xor_value >> 8);
			}

			len++;
		}

		memcpy(p_dev_id_1->emmc_id.u8_vals, p_xor_serial, sizeof(p_dev_id_1->emmc_id));

		/*每两个字节互换*/
		for (i = 0; i < ARRAY_SIZE(p_dev_id_1->emmc_id.u16_vals); i++) {
			p_dev_id_1->emmc_id.u16_vals[i] =
				(__u16)(((p_dev_id_1->emmc_id.u16_vals[i] >> 8) & 0xffu) | ((p_dev_id_1->emmc_id.u16_vals[i] & 0xffu) << 8));
		}

		/* 15 13 11 9 | 6 4 2 0 | 14 12 10 8 | 7 5 3 1 */
		memcpy(p_dev_id_2->emmc_id.u8_vals, p_dev_id_1->emmc_id.u8_vals, sizeof(p_dev_id_1->emmc_id.u8_vals));

		p_dev_id_1->emmc_id.u16_vals[0] = p_dev_id_2->emmc_id.u16_vals[15];
		p_dev_id_1->emmc_id.u16_vals[1] = p_dev_id_2->emmc_id.u16_vals[13];
		p_dev_id_1->emmc_id.u16_vals[2] = p_dev_id_2->emmc_id.u16_vals[11];
		p_dev_id_1->emmc_id.u16_vals[3] = p_dev_id_2->emmc_id.u16_vals[9];
		p_dev_id_1->emmc_id.u16_vals[4] = p_dev_id_2->emmc_id.u16_vals[6];
		p_dev_id_1->emmc_id.u16_vals[5] = p_dev_id_2->emmc_id.u16_vals[4];
		p_dev_id_1->emmc_id.u16_vals[6] = p_dev_id_2->emmc_id.u16_vals[2];
		p_dev_id_1->emmc_id.u16_vals[7] = p_dev_id_2->emmc_id.u16_vals[0];
		p_dev_id_1->emmc_id.u16_vals[8] = p_dev_id_2->emmc_id.u16_vals[14];
		p_dev_id_1->emmc_id.u16_vals[9] = p_dev_id_2->emmc_id.u16_vals[12];
		p_dev_id_1->emmc_id.u16_vals[10] = p_dev_id_2->emmc_id.u16_vals[10];
		p_dev_id_1->emmc_id.u16_vals[11] = p_dev_id_2->emmc_id.u16_vals[8];
		p_dev_id_1->emmc_id.u16_vals[12] = p_dev_id_2->emmc_id.u16_vals[7];
		p_dev_id_1->emmc_id.u16_vals[13] = p_dev_id_2->emmc_id.u16_vals[5];
		p_dev_id_1->emmc_id.u16_vals[14] = p_dev_id_2->emmc_id.u16_vals[3];
		p_dev_id_1->emmc_id.u16_vals[15] = p_dev_id_2->emmc_id.u16_vals[1];

		memcpy(g_rs_dev_id, p_dev_id_1->emmc_id.u8_vals, sizeof(g_rs_dev_id));
		/*chk_dev_id_res = rs_parse_dev_id(p_rsinfo);*/

		ret = 0;
	} else {
		ret = -EINVAL;
	}

out:

	if (p_dev_id_1) {
		rs_kfree(p_dev_id_1);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_parse_dev_id RS_HIDE(_d2)
#endif
noused notrace static int rs_parse_dev_id(char *p_rsinfo)
{
	int ret;
	int rs_len;

	ret = -1;
	RS_LOG("pdi,0\n");
	rs_len = cfg_strlen(p_rsinfo);
	if (rs_len >= sizeof(g_rs_dev_id)) {
	#if 0
		{
			int i, len = 0;
			char buffer[72];
			if (rs_len > sizeof(g_rs_dev_id))
				rs_len = sizeof(g_rs_dev_id);

			for (i = 0; i < rs_len; i++) {
				len += scnprintf(&buffer[len], (sizeof(buffer) - len), "%02x", p_rsinfo[i]);
			}
			RS_LOG("pdi,1,%s\n", buffer);
		}
	#else
		/*RS_LOG("pdi,1\n");*/
	#endif

		if (strncmp(p_rsinfo, g_rs_dev_id, sizeof(g_rs_dev_id)) == 0) {
			RS_LOG("pdi,2\n");
			ret = 0;
		} else {
			ret = -2;
		}
	}
	RS_LOG("pdi,e,%d\n", rs_len);
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_blkdev_path_by_part_name RS_HIDE(_la_25)
#endif
noused notrace static int rs_get_blkdev_path_by_part_name(const char *part_name,
	char *buffer, size_t buffer_size);


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_init_rs_config_from_blkdev_dev_ok RS_HIDE(_d2_1)
#endif
noused static int g_init_rs_config_from_blkdev_dev_ok = 0;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_rs_config_from_block_dev RS_HIDE(_d3)
#endif
noused notrace static int init_rs_config_from_block_dev(void)
{
#if defined(RS_HAS_DEV_SHORTCUTS) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	noused int pathlen;
	char *dev_name = NULL;
	struct file *filp;
	loff_t loff_res, start_address;
	ssize_t ssize_res;
	char *config_buffer, *buffer, *p_rsinfo;
	int is_readonly, se_flag = 0;
	int ret, chk_dev_id_res, loop_count;

	#define BLOCK_DEV_CONFIG_OFFSET (1024 * 128)
	#define BLOCK_DEV_CONFIG_SIZE (512)

	#if BLOCK_DEV_CONFIG_SIZE > RS_BUFFER_SIZE
		#error "BLOCK_DEV_CONFIG_SIZE larger than RS_BUFFER_SIZE!"
	#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
	if (RS_BUFFER_SIZE < (check_strings[csDevBackup].str_len + 1)) {
		/*buffer overrun*/
		RS_LOG("icfb,1\n");
		return -EINVAL; /*-ENOBUFS;*/
	}

	/*dev_name = "/dev/backup";*/
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		RS_LOG("icfb,2\n");
		return -ENOMEM;
	}

	memcpy(dev_name, check_strings[csDevBackup].str, check_strings[csDevBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBackup].str_len, csDevBackup);
#endif

	if (is_file_exist(dev_name)) {
		RS_LOG("icfb,3\n");
		goto open_dev;
	}
#endif

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (!g_block_platform_devname) {
		RS_LOG("icfb,4\n");
		ret = 1;
		goto out_with_dev_name;
	}
#endif

#if !defined(RS_HAS_DEV_SHORTCUTS)
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		RS_LOG("icfb,5\n");
		return -ENOMEM;
	}
#endif

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (RS_BUFFER_SIZE < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		+ check_strings[csSlashConfigPart].str_len
	#else
		+ check_strings[csSlashBackup].str_len
	#endif
		+ 1))
#else
	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		+ check_strings[csSlashConfigPart].str_len
	#else
		+ check_strings[csSlashBackup].str_len
	#endif
		+ 1))
#endif
	{
		/*buffer overrun*/
		RS_LOG("icfb,6\n");
		ret = -ENOBUFS;
		goto out_with_dev_name;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	memcpy(&dev_name[pathlen], check_strings[csSlashConfigPart].str, check_strings[csSlashConfigPart].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashConfigPart].str_len, csSlashConfigPart);
#endif
#else
	memcpy(&dev_name[pathlen], check_strings[csSlashBackup].str, check_strings[csSlashBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashBackup].str_len, csSlashBackup);
#endif
#endif

#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
open_dev:
#endif

	ret = 0; /*-Wmaybe-uninitialized*/

	buffer = NULL;
	is_readonly = 0;

	se_flag = test_and_set_thread_flag((BITS_PER_LONG - 1));

#if 0
	if (!is_file_exist(dev_name)) {
		char *part_name;

		part_name = strrchr(dev_name, '/');
		if (part_name)
			part_name++;
		else
			part_name = dev_name;

		RS_LOG("icfb,6.1,%d,%s\n", ret, part_name);

		ret = rs_get_blkdev_path_by_part_name(part_name, dev_name, RS_BUFFER_SIZE);
		if (ret) {
			RS_LOG("icfb,6.2,%d\n", ret);
			goto out_with_dev_name;
		}
	}
#endif

	filp = local_filp_open(dev_name, RS_WRITE_FILE_OPEN_FLAGS, RS_WRITE_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		RS_LOG("icfb,7,%s\n", dev_name);
		ret = PTR_ERR(filp);
		if (ret == -EACCES) {
			filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
			if (IS_ERR(filp)) {
				RS_LOG("icfb,7.1\n");
				ret = PTR_ERR(filp);
			} else {
				is_readonly = 1;
				goto dev_opened;
			}
		}
		filp = NULL;
		goto out_no_buffer;
	}

dev_opened:
	g_init_rs_config_from_blkdev_dev_ok = 1;

	config_buffer = (char *)RS_GET_BUFFER();
	if (!config_buffer) {
		RS_LOG("icfb,8\n");
		ret = -ENOMEM;
		goto out_no_buffer;
	}
	config_buffer[0] = '\0';

	loop_count = 0;

	start_address = BLOCK_DEV_CONFIG_OFFSET;
	chk_dev_id_res = 1;

loop:

#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	if (loop_count) {
#endif
		loff_res = safe_vfs_llseek(filp, start_address, SEEK_SET);

		if (loff_res < 0) {
			RS_LOG("icfb,9\n");
			ret = loff_res;
			goto out;
		}

		ssize_res = safe_vfs_read(filp, config_buffer, BLOCK_DEV_CONFIG_SIZE, start_address);

		if (ssize_res <= 0) {
			RS_LOG("icfb,a\n");
			ret = ssize_res;
			goto out;
		}

		RS_LOG("icfb,b\n");
		if (ssize_res >= BLOCK_DEV_CONFIG_SIZE) {
			ssize_res = BLOCK_DEV_CONFIG_SIZE - 1;
		}
		config_buffer[ssize_res] = '\0';

		p_rsinfo = STRSTR(config_buffer, check_strings[csRSId].str, csRSId);
		if (p_rsinfo) {
			RS_LOG("icfb,c\n");
			p_rsinfo += check_strings[csRSId].str_len;

			chk_dev_id_res = rs_parse_dev_id(p_rsinfo);
		}
#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	}
#endif

	if (!loop_count) {
		RS_LOG("icfb,d\n");
		if (!g_blk_dev_size) {
			RS_LOG("icfb,e\n");
			g_blk_dev_size = safe_vfs_llseek(filp, 0, SEEK_END);
			/*RS_LOG("blk_size:%lld\n", g_blk_dev_size);*/

		#if defined(RS_HAS_DEV_SHORTCUTS)
			if (g_blk_dev_size <= 0) {
				/*noused struct inode *inode;*/
				/*if ((inode = filp->f_dentry->d_inode) && (S_ISCHR(inode->i_mode)))*/
				{
					struct mtd_info_user dev_info;

					dev_info.size = 0;
					ret = __vfs_ioctl(filp, MEMGETINFO, (unsigned long)&dev_info);
					if (ret < 0) {
						/*RS_LOG("xjy, 13.1, %d\n", ret);*/
					} else {
						g_blk_dev_size = dev_info.size;
					}
				}
			}
		#endif
		}

		if ((g_blk_dev_size < RS_BLK_DEV_MIN_TOTAL_SIZE/*RS_BLK_DEV_DATA_SIZE*/)
			|| (g_blk_dev_size & RS_BLK_DEV_TOTAL_SIZE_MASK)) {
			RS_LOG("icfb,f\n");
			ret = 2;
			chk_dev_id_res = 2;
			goto out;
		}

		loop_count++;

		start_address = g_blk_dev_size - RS_BLK_DEV_DATA_SIZE + RS_BLK_DEV_DATA_CONFIG_DATA_OFFSET;

		/*test and write signature, data[512] == signature*/
		{
			loff_t marker_address = g_blk_dev_size - RS_BLK_DEV_DATA_SIZE + RS_BLK_DEV_DATA_SIGNATURE_OFFSET;

			RS_LOG("icfb,g\n");
			loff_res = safe_vfs_llseek(filp, marker_address, SEEK_SET);

			if (loff_res < 0) {
				RS_LOG("icfb,h\n");
				ret = loff_res;
				goto out;
			}

			ssize_res = safe_vfs_read(filp, config_buffer, sizeof(u32), marker_address);
			if (ssize_res <= 0) {
				RS_LOG("icfb,i\n");
				ret = ssize_res;
				goto out;
			}

			if ((!is_readonly) && (*((u32 *)config_buffer) != RS_DATA_BLOCK_MAGIC)) {
				RS_LOG("icfb,j\n");
				loff_res = safe_vfs_llseek(filp, marker_address, SEEK_SET);

				if (loff_res < 0) {
					RS_LOG("icfb,k\n");
					ret = loff_res;
					goto out;
				}

				*((u32 *)config_buffer) = RS_DATA_BLOCK_MAGIC;
				ssize_res = safe_vfs_write(filp, config_buffer, sizeof(u32), marker_address);
				if (ssize_res <= 0) {
					RS_LOG("icfb,l\n");
					ret = ssize_res;
					goto out;
				}
			}
		}

		RS_LOG("icfb,m\n");
		goto loop;
	} else {
		RS_LOG("icfb,n\n");
		/*chk_dev_id_res = 1;*/
	}

	if (chk_dev_id_res != 0) {
		RS_LOG("icfb,o\n");
		ret = 3;
		goto out;
	}

#if RS_DEBUG
#if defined(CONFIG_CONFIG_RS_LOG)
	p_rsinfo = STRSTR(config_buffer, check_strings[csRSLog].str, csRSLog);
	if (p_rsinfo) {
		int res;
		unsigned long result = 0;

		p_rsinfo += check_strings[csRSLog].str_len;

		res = rs_parse_config_number(p_rsinfo, &result);
		if (res == 0) {
			/*ok*/
			g_rs_enable_log = result;
		}
	}
#endif
#endif

#if defined(CONFIG_CONFIG_RS_JOURNAL)
	p_rsinfo = STRSTR(config_buffer, check_strings[csRSJournal].str, csRSJournal);
	if (p_rsinfo) {
		int res;
		unsigned long result = 0;

		p_rsinfo += check_strings[csRSJournal].str_len;

		res = rs_parse_config_number(p_rsinfo, &result);
		if (res == 0) {
			/*ok*/
			g_rs_enable_journal = result;
		}
	}
#endif

#if defined(CONFIG_CONFIG_RS_PERMIT)
	p_rsinfo = STRSTR(config_buffer, check_strings[csRSPermit].str, csRSPermit);
	if (p_rsinfo) {
		unsigned long local_rs_permit = 0;

		p_rsinfo += check_strings[csRSPermit].str_len;

		if (rs_parse_config_flags(p_rsinfo, &local_rs_permit, &buffer) >= 0) {
			set_permit_vars(&local_rs_permit);
		}
	}
#endif

#if defined(CONFIG_CONFIG_RS_BYPASS)
	p_rsinfo = STRSTR(config_buffer, check_strings[csRSByPass].str, csRSByPass);
	if (p_rsinfo) {
		unsigned long local_rs_bypass = 0;

		p_rsinfo += check_strings[csRSByPass].str_len;

		if (rs_parse_config_flags(p_rsinfo, &local_rs_bypass, &buffer) >= 0) {
			set_bypass_vars(&local_rs_bypass);
		}
	}
#endif

	if (buffer) {
		RS_FREE_BUFFER(buffer);
	}

	ret = 0;

out:
	RS_FREE_BUFFER(config_buffer);

out_no_buffer:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
out_with_dev_name:
#endif
/*#if defined(RS_QUALCOMM_PLATFORM)*/
	RS_FREE_BUFFER(dev_name);
/*#endif*/

	if (!se_flag) {
		clear_thread_flag((BITS_PER_LONG - 1));
	}

	RS_LOG("icfb,z,%d\n", ret);
	return ret;
#else
	return 0;
#endif
}
#endif

/*///////////////////*/



#define RS_JOURNAL_LOG_BLOCK_COUNT (2)

#define g_rs_dev_log_start_block_index (1)

/*refer to mediatek's dumchar.c dumchar_ioctl() handling MEMGETINFO*/
#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	#define g_rs_dev_log_dev_block_size (16 * 1024)
#else
	#define g_rs_dev_log_dev_block_size (128 * 1024)
#endif

#define g_rs_dev_log_dev_write_size (512)
#define g_rs_dev_log_dev_write_pages_count (g_rs_dev_log_dev_block_size / g_rs_dev_log_dev_write_size)


#if RS_BLK_DEV_DATA_SIZE < ((g_rs_dev_log_start_block_index + RS_JOURNAL_LOG_BLOCK_COUNT) * g_rs_dev_log_dev_block_size)
	#error "RS_BLK_DEV_DATA_SIZE smaller than RS config and journal log data size!"
#endif

#define RS_JOURNAL_MAGIC (0x52734A6C)

#define RS_JOURNAL_RECORD_VERSION (1)

#define RS_JOURNAL_RECORD_SIZE (512)

#define RS_JOURNAL_VERSION_LENGTH (24)
#define RS_JOURNAL_TAG_LENGTH (12)

#define RS_JOURNAL_LOG_LENGTH (RS_JOURNAL_RECORD_SIZE - 4 * sizeof(u32) - sizeof(time_t) \
		- RS_JOURNAL_VERSION_LENGTH - RS_JOURNAL_TAG_LENGTH)

typedef struct tag_rs_journal_record {
	u32 magic;
	u32 record_version;
	u32 sequence;
	u32 flags;
	time_t utc_time;
	char software_version[RS_JOURNAL_VERSION_LENGTH];
	char tag[RS_JOURNAL_TAG_LENGTH];
	char log[RS_JOURNAL_LOG_LENGTH];
} __attribute__((packed)) rs_journal_record;


#define RS_JOURNAL_LOG_ENCRYPT_KEY_LEN (4)
#define RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT (1)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_journal_encrypt RS_HIDE(_e0)
#endif
noused notrace static void rs_journal_encrypt(char *str)
{
	if (str) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((ch = *str)) {
			if (++i >= (RS_JOURNAL_LOG_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			ch = ((tmp_ch << RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT) | (tmp_ch >> (8 - RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT)));

			*str++ = ch;
		}
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_journal_decrypt RS_HIDE(_e1)
#endif
noused notrace static void rs_journal_decrypt(char *str)
{
	if (str) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((ch = *str)) {
			ch = ((ch >> RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT) | (ch << (8 - RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT)));
			if (++i >= (RS_JOURNAL_LOG_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			*str++ = tmp_ch;
		}
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_journal_encrypt_len RS_HIDE(_e2)
#endif
noused notrace static void rs_journal_encrypt_len(char *str, int len)
{
	if ((str) && (len)) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((len--) && (ch = *str)) {
			if (++i >= (RS_JOURNAL_LOG_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			ch = ((tmp_ch << RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT) | (tmp_ch >> (8 - RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT)));

			*str++ = ch;
		}
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_journal_decrypt_len RS_HIDE(_e3)
#endif
noused notrace static void rs_journal_decrypt_len(char *str, int len)
{
	if ((str) && (len)) {
		unsigned int i = 0;
		unsigned char ch, tmp_ch;

		while ((len--) && (ch = *str)) {
			ch = ((ch >> RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT) | (ch << (8 - RS_JOURNAL_LOG_ENCRYPT_SHIFT_BIT)));

			if (++i >= (RS_JOURNAL_LOG_ENCRYPT_KEY_LEN + 1))
				i = 1;

			tmp_ch = ch ^ i;
			if (!tmp_ch)
				tmp_ch = ch;

			*str++ = tmp_ch;
		}
	}
}

#if defined(CONFIG_DO_RS_JOURNAL)

/*from CFG_IQOO_NV_Item.h*/
typedef struct {
	/* the result of GSensor calibration */
	u32 gsensor_calibration_flag;
	/* the flag to indicae that this is a mater clear reboot */
	u32 master_clear_flag;
	/* the result of master clear */
	u32 master_clear_result;
	/* the flag to open ADB port after reboot */
	u32 adb_flag;
	/* IQOO software version */
	u8 iqoo_sv[50];
	/* proximity sensor calibration data */
	s32 ps_cali_data;
	/* ambient light sensor calibration data */
	s32 als_cali_data;
	/* gravity sensor calibration data */
	s32 gs_cali_dataX;
	s32 gs_cali_dataY;
	s32 gs_cali_dataZ;
	/* Wallpaper id */
	s32 wallpaper_id;
	/* the result of als_flag calibration */
	s32 als_cali_flag;
	/* the result of ps_flag calibration */
	s32 ps_cali_flag;
	/* the result of acm_flag */
	s32 acm_flag;
	/* the result of copy_flag */
	s32 copy_flag;
	/*is  root or not */
	s32 is_root;
} IQOO_NV_Item_Header;

#if defined(CONFIG_RS_BOOT_DEBUG)
	#define RJ_LOG(...)  RS_LOG(__VA_ARGS__)
#else
	#define RJ_LOG(...)
#endif

#if defined(CONFIG_RS_JOURNAL_CHECK_NV) && defined(CONFIG_DONOT_USE_BACKUP_PART)
/*read nvram is_root flag*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_nv_isroot RS_HIDE(_e6)
#endif
noused notrace static int read_nv_isroot(char *dev_name, size_t dev_name_size, unsigned int *flags_ptr)
{
	noused int pathlen;
	int se_flag = 0;
	struct file *filp = NULL;
	ssize_t ssize_res;
	int ret = 0;

	if ((!dev_name) || (dev_name_size < RS_BUFFER_SIZE) || (!flags_ptr)) {
		return -EINVAL;
	}

#if defined(RS_HAS_DEV_SHORTCUTS)
	memcpy(dev_name, check_strings[csDevBackup].str, check_strings[csDevBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBackup].str_len, csDevBackup);
#endif

	if (is_file_exist(dev_name)) {
		goto open_dev;
	}
#endif

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (dev_name_size < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
		+ check_strings[csSlashBackup].str_len
		+ 1))
#else
	if (!g_block_platform_devname) {
		ret = 1;
		goto out;
	}

	if (dev_name_size < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
		+ check_strings[csSlashBackup].str_len
		+ 1))
#endif
	{
		/*buffer overrun*/
		ret = -ENOBUFS;
		goto out;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

	memcpy(&dev_name[pathlen], check_strings[csSlashBackup].str, check_strings[csSlashBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashBackup].str_len, csSlashBackup);
#endif

#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
open_dev:
#endif

	se_flag = test_and_set_thread_flag((BITS_PER_LONG - 1));

	filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		RJ_LOG("rnr,1:%d\n", ret);
		goto out;
	}

	{
	#if defined(CONFIG_RS_OBFUSCATED_NAME)
	#define g_nv_root_checked RS_HIDE(_f1_1)
	#endif
		static int g_nv_root_checked;

		if (g_nv_root_checked >= 0) {
			IQOO_NV_Item_Header *nv_rec;
			char *nv_sv;

		#if 1
			if (sizeof(IQOO_NV_Item_Header) > dev_name_size) {
				ret = -EMSGSIZE;
				RJ_LOG("rnr,2:%d\n", ret);
				g_nv_root_checked = -1;
				goto out;
			}
		#endif

			/*dev_name内容已不用，作为缓冲区*/
			ssize_res = safe_vfs_read(filp, dev_name, sizeof(IQOO_NV_Item_Header), 0);
			if (ssize_res <= 0) {
				ret = ssize_res;
				RJ_LOG("rnr,3:%d\n", ret);
				goto out;
			}

			nv_rec = (IQOO_NV_Item_Header *)dev_name;
			/*版本号*/
			nv_sv = nv_rec->iqoo_sv;
			nv_sv[sizeof(nv_rec->iqoo_sv) - 1] = '\0';

			nv_sv = _strchr(nv_sv, 'D');
			if (!nv_sv) {
				goto not_checked;
			}
			nv_sv = _strchr(nv_sv, '_');
			if (!nv_sv) {
				goto not_checked;
			}
			nv_sv = _strchr(nv_sv, '.');
			if (!nv_sv) {
				goto not_checked;
			}
			if (nv_sv) {
				if (nv_rec->is_root) {
					*flags_ptr |= (1 << ((sizeof(u32) << 3) - 1)); /*RS_MOUNT_CHECK_NV_IS_ROOT_FLAG;*/
				} else {
					*flags_ptr &= ~(1 << ((sizeof(u32) << 3) - 1));
				}

				g_nv_root_checked = rs_get_set_value(); /*1;*/
			} else {
			not_checked:
				g_nv_root_checked = -1;
			}
		}
	}

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	if (!se_flag) {
		clear_thread_flag((BITS_PER_LONG - 1));
	}

	return ret;
}
#endif

#if defined(RS_HAS_DEV_SHORTCUTS) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_journal_mutex RS_HIDE(_f0)
#endif
noused static DEFINE_MUTEX(g_rs_journal_mutex);
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_journal RS_HIDE(_f1)
#endif
noused notrace static int rs_journal(const char *tag, const char *log, unsigned int flags, int log_len)
{
#if defined(RS_HAS_DEV_SHORTCUTS) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	noused int pathlen;
	char *dev_name = NULL;
	int se_flag = 0;
	struct file *filp;
	ssize_t ssize_res;
	int ret = 0;

	loff_t start_address, write_start_address;
	loff_t loff_res;
	loff_t offset;
	u32 sequence;
	rs_journal_record *journal;

	int slots_per_block;
	int total_slots;
	int seq_no;

	/*#define BLOCK_DEV_JOURNAL_OFFSET (1024 * 128)*/
	#define BLOCK_DEV_JOURNAL_RECORD_SIZE (512)

	#if BLOCK_DEV_JOURNAL_RECORD_SIZE > RS_BUFFER_SIZE
		#error "BLOCK_DEV_JOURNAL_RECORD_SIZE larger than RS_BUFFER_SIZE!"
	#endif

	if ((!log_len) || (!g_rs_enable_journal)) {
		return 0;
	}

	/*RJ_LOG("rec size:%d\n", sizeof(*journal));*/
	/*RJ_LOG("ver:%d, %-10.10s\n", g_software_version_str_len, g_software_version_str);*/

#if defined(RS_HAS_DEV_SHORTCUTS)
	/*dev_name = "/dev/backup";*/
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		return -ENOMEM;
	}

	memcpy(dev_name, check_strings[csDevBackup].str, check_strings[csDevBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBackup].str_len, csDevBackup);
#endif

	if (is_file_exist(dev_name)) {
		goto open_dev;
	}
#endif

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (!g_block_platform_devname) {
		ret = 1;
		goto out_with_dev_name;
	}
#endif

#if !defined(RS_HAS_DEV_SHORTCUTS)
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		return -ENOMEM;
	}
#endif

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (RS_BUFFER_SIZE < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
		+ check_strings[csSlashBackup].str_len
		+ 1))
#else
	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		+ check_strings[csSlashConfigPart].str_len
	#else
		+ check_strings[csSlashBackup].str_len
	#endif
		+ 1))
#endif
	{
		/*buffer overrun*/
		ret = -ENOBUFS;
		goto out_with_dev_name;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(CONFIG_DONOT_USE_BACKUP_PART)
	memcpy(&dev_name[pathlen], check_strings[csSlashConfigPart].str, check_strings[csSlashConfigPart].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashConfigPart].str_len, csSlashConfigPart);
#endif
#else
	memcpy(&dev_name[pathlen], check_strings[csSlashBackup].str, check_strings[csSlashBackup].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashBackup].str_len, csSlashBackup);
#endif
#endif

#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
open_dev:
#endif

	/*buffer = NULL;*/

	se_flag = test_and_set_thread_flag((BITS_PER_LONG - 1));

	filp = local_filp_open(dev_name, RS_WRITE_FILE_OPEN_FLAGS, RS_WRITE_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		RJ_LOG("l2d,1:%s,%d\n", dev_name, ret);
		goto out_no_buffer;
	}

	if (!g_blk_dev_size) {
		g_blk_dev_size = safe_vfs_llseek(filp, 0, SEEK_END);
		RS_LOG("blk_size: %lld\n", g_blk_dev_size);

	#if defined(RS_HAS_DEV_SHORTCUTS)
		if (g_blk_dev_size <= 0) {
			/*noused struct inode *inode;*/
			/*if ((inode = filp->f_dentry->d_inode) && (S_ISCHR(inode->i_mode)))*/
			{
				struct mtd_info_user dev_info;
				dev_info.size = 0;
				ret = __vfs_ioctl(filp, MEMGETINFO, (unsigned long)&dev_info);
				if (ret < 0) {
					RS_LOG("l2d,1_ %d\n", ret);
				} else {
					g_blk_dev_size = dev_info.size;
				}
			}
		}
	#endif
	}

	if ((g_blk_dev_size < RS_BLK_DEV_MIN_TOTAL_SIZE/*RS_BLK_DEV_DATA_SIZE*/)
		|| (g_blk_dev_size & RS_BLK_DEV_TOTAL_SIZE_MASK)) {
		RJ_LOG("l2d,1.0\n");
		goto out_no_buffer;
	}

#if defined(CONFIG_RS_JOURNAL_CHECK_NV)
	/*read nvram is_root flag*/
	{
#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		/*dev_name内容已不用，作为缓冲区*/
		read_nv_isroot(dev_name, RS_BUFFER_SIZE, &flags);
#else
	#if defined(CONFIG_RS_OBFUSCATED_NAME)
	#define g_nv_root_checked RS_HIDE(_f1_1)
	#endif
		static int g_nv_root_checked;

		if (g_nv_root_checked >= 0) {
			IQOO_NV_Item_Header *nv_rec;
			char *nv_sv;

		#if 0
			if (sizeof(IQOO_NV_Item_Header) > RS_BUFFER_SIZE) {
				BUG();
			}
		#endif

			/*dev_name内容已不用，作为缓冲区*/
			ssize_res = safe_vfs_read(filp, dev_name, sizeof(IQOO_NV_Item_Header), 0);
			if (ssize_res <= 0) {
				ret = ssize_res;
				RJ_LOG("l2d,1.1:%d\n", ret);
				goto out_no_buffer;
			}

			nv_rec = (IQOO_NV_Item_Header *)dev_name;
			/*版本号*/
			nv_sv = nv_rec->iqoo_sv;
			nv_sv[sizeof(nv_rec->iqoo_sv) - 1] = '\0';

			nv_sv = _strchr(nv_sv, 'D');
			if (!nv_sv)
				goto not_checked;
			nv_sv = _strchr(nv_sv, '_');
			if (!nv_sv)
				goto not_checked;
			nv_sv = _strchr(nv_sv, '.');
			if (!nv_sv)
				goto not_checked;
			if (nv_sv) {
				if (nv_rec->is_root) {
					flags |= (1 << ((sizeof(u32) << 3) - 1)); /*RS_MOUNT_CHECK_NV_IS_ROOT_FLAG;*/
				} else {
					flags &= ~(1 << ((sizeof(u32) << 3) - 1));
				}

				g_nv_root_checked = rs_get_set_value(); /*1;*/
			} else {
			not_checked:
				g_nv_root_checked = -1;
			}
		}
#endif
	}
#endif

	/*用2个block,第一项是用来保存当前 sequence*/
	start_address = (g_blk_dev_size - RS_BLK_DEV_DATA_SIZE) + (g_rs_dev_log_start_block_index * g_rs_dev_log_dev_block_size);

	loff_res = safe_vfs_llseek(filp, start_address, SEEK_SET);
	if (loff_res < 0) {
		ret = loff_res;
		RJ_LOG("l2d,2:%d\n", ret);
		goto out_no_buffer;
	}

	journal = (rs_journal_record *)RS_GET_BUFFER();
	if (!journal) {
		ret = -ENOMEM;
		goto out_no_buffer;
	}

	mutex_lock(&g_rs_journal_mutex);

	ssize_res = safe_vfs_read(filp, journal, sizeof(*journal)/*BLOCK_DEV_JOURNAL_RECORD_SIZE*/, start_address);

	if (ssize_res <= 0) {
		ret = ssize_res;
		RJ_LOG("l2d,3:%d\n", ret);
		goto out_with_lock;
	}

	if (ssize_res >= sizeof(*journal)/*BLOCK_DEV_JOURNAL_RECORD_SIZE*/) {
		ssize_res = sizeof(*journal) - 1;
	}
	((char *)journal)[ssize_res] = '\0';

	if ((journal->magic == RS_JOURNAL_MAGIC) && (journal->flags == 0xFFFFFFFF)
		&& ((journal->utc_time & 0xFFFFFFFF) == 0xFFFFFFFF)) {
		sequence = journal->sequence;
	} else {
		sequence = 0;
	}

	/*Inc sequence, then write back*/
	memset(journal, 0, sizeof(*journal));

	journal->magic = RS_JOURNAL_MAGIC;
	journal->record_version = RS_JOURNAL_RECORD_VERSION;
	journal->flags = 0xFFFFFFFF;
	journal->utc_time = 0xFFFFFFFF;
	journal->sequence = (sequence + 1); /*溢出回到0*/

	loff_res = safe_vfs_llseek(filp, start_address, SEEK_SET);
	if (loff_res  < 0) {
		ret = loff_res;
		RJ_LOG("l2d,4:%d\n", ret);
		goto out_with_lock;
	}

	ssize_res = safe_vfs_write(filp, journal, sizeof(*journal), start_address);

	if (ssize_res <= 0) {
		ret = ssize_res;
		RJ_LOG("l2d,5:%d\n", ret);
		goto out_with_lock;
	}

	mutex_unlock(&g_rs_journal_mutex);

	slots_per_block = g_rs_dev_log_dev_write_pages_count;
	total_slots = (slots_per_block * RS_JOURNAL_LOG_BLOCK_COUNT) - 1;

	write_start_address = start_address;


	seq_no = sequence % total_slots;
	/*RJ_LOG("seq_no:%d, slot_per: %d, total_slots:%d, write_size:%d\n", seq_no, slots_per_block,
		total_slots, g_history_log_dev_write_size);*/

	if (seq_no < (slots_per_block - 1)) {
		/*属于 first block*/
		/*write_start_address += 0;*/
		offset = (seq_no + 1) * g_rs_dev_log_dev_write_size;
		/*RJ_LOG("1 start:%lld, offset:%lld\n", write_start_address, offset);*/
	} else {
		/*属于后面的 block*/
		seq_no -= (slots_per_block - 1);
		write_start_address += (seq_no / slots_per_block) * g_rs_dev_log_dev_block_size;
		offset = (seq_no % slots_per_block) * g_rs_dev_log_dev_write_size;
		/*RJ_LOG("2 start:%ld, offset:%ld\n", write_start_address, offset);*/
	}

	/*write_start_address += (seq_no / slots_per_block) * g_rs_dev_log_dev_block_size;*/
	/*offset = (seq_no % slots_per_block) * g_rs_dev_log_dev_write_size;*/

	memset(journal, 0, sizeof(*journal));

	journal->magic = RS_JOURNAL_MAGIC;
	journal->record_version = RS_JOURNAL_RECORD_VERSION;
	journal->sequence = sequence;
	journal->flags = flags;
	{
		struct timeval tmp_time;
		/* 获取当前的UTC时间 */
		do_gettimeofday(&tmp_time);

		/* 把UTC时间调整为本地时间 */
		/*tmp_time.tv_sec -= sys_tz.tz_minuteswest * 60;*/

		journal->utc_time = tmp_time.tv_sec;

		/*journal->utc_time = time(NULL);*/
	}

	if ((g_software_version_str) && (g_software_version_str[0])) {
		int ver_len = g_software_version_str_len;

		if (ver_len > sizeof(journal->software_version)) {
			ver_len = sizeof(journal->software_version);
		}
		/*g_software_version_str has no null terminator*/
		/*strncpy(journal->software_version, g_software_version_str, len);*/

		memcpy(journal->software_version, g_software_version_str, ver_len);
		rs_journal_encrypt_len(journal->software_version, ver_len);
	} else {
		char *p = journal->software_version;
		p[0] = 'u';
		p[1] = 'n';
		p[2] = 'k';
		/*p[3] = '\0';*/
		rs_journal_encrypt_len(p, (sizeof("unk") - 1));
	}


	if ((uintptr_t)tag < (uintptr_t)csInvalid) {
		int str_index = (intptr_t)tag;
		int tag_len = check_strings[str_index].str_len;

		if (tag_len > sizeof(journal->tag)) {
			tag_len = sizeof(journal->tag);
		}

		memcpy(journal->tag, check_strings[str_index].str, tag_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(journal->tag, tag_len, str_index);
	#endif

		rs_journal_encrypt_len(journal->tag, tag_len);
	} else if (tag && (tag[0])) {
		int tag_len = strnlen(tag, sizeof(journal->tag));

		memcpy(journal->tag, tag, tag_len);
		rs_journal_encrypt_len(journal->tag, tag_len);
	}

	if ((uintptr_t)log < (uintptr_t)csInvalid) {
		int str_index = (intptr_t)log;

		if ((log_len < 0) || (log_len > check_strings[str_index].str_len)) {
			log_len = check_strings[str_index].str_len;
		}

		if (log_len > sizeof(journal->log)) {
			log_len = sizeof(journal->log);
		}

		memcpy(journal->log, check_strings[str_index].str, log_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(journal->log, log_len, str_index);
	#endif

		rs_journal_encrypt_len(journal->log, log_len);
	} else if (log && (log[0])) {
		int len = strnlen(log, sizeof(journal->log));

		if ((log_len < 0) || (log_len > len)) {
			log_len = len;
		}

		memcpy(journal->log, log, log_len);
		rs_journal_encrypt_len(journal->log, log_len);
	}

	/*RJ_LOG("seek:%lld\n", write_start_address + offset);*/
	loff_res = safe_vfs_llseek(filp, write_start_address + offset, SEEK_SET);
	if (loff_res < 0) {
		ret = loff_res;
		RJ_LOG("l2d,6:%d\n", ret);
		goto out;
	}

	ssize_res = safe_vfs_write(filp, journal, sizeof(*journal), write_start_address + offset);

	if (ssize_res <= 0) {
		ret = ssize_res;
		RJ_LOG("l2d,7:%d\n", ret);
		goto out;
	}

	ret = 0;

out:

	RS_FREE_BUFFER(journal);

out_no_buffer:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
out_with_dev_name:
#endif
/*#if defined(RS_QUALCOMM_PLATFORM)*/
	RS_FREE_BUFFER(dev_name);
/*#endif*/

	if (!se_flag) {
		clear_thread_flag((BITS_PER_LONG - 1));
	}

	return ret;

out_with_lock:
	/*if (mutex_is_locked(&g_rs_journal_mutex))*/
	{
		mutex_unlock(&g_rs_journal_mutex);
	}

	goto out;

#else
	return 0;
#endif
}

#else
#define rs_journal(...) (void)(0)
#endif

#if 0
/*atomic_t g_adb_remount_recorded;*/ /*atomic_cmpxchg*/

noused notrace static int get_time_str(char *output)
{
	struct timeval tv;
	struct rtc_time tm;

	/* 获取当前的UTC时间 */
	do_gettimeofday(&tv);

	/* 把UTC时间调整为本地时间 */
	tv.tv_sec -= sys_tz.tz_minuteswest * 60;

	/* 算出时间中的年月日等数值到tm中 */
	rtc_time_to_tm(tv.tv_sec, &tm);

	return snprintf(output, 64, "%04d-%02d-%02d %02d:%02d:%02d"
		, (tm.tm_year + 1900)
		, (tm.tm_mon + 1)
		, tm.tm_mday
		, tm.tm_hour
		, tm.tm_min
		, tm.tm_sec);
	/*u16 year;u8 month; u8 day; u8 hour; u8 minute; u8 second*/
}
#endif

/*///////////////////*/

/*kernel 4.9 arm
set_kernel_text_rw()/set_kernel_text_ro()
set_all_modules_text_rw()/set_all_modules_text_ro() ==> flush_tlb_all()
*/

#if defined(RS_HAS_QCOM_SECURE_BUFFER)

/*refer to msm_protect_kernel() in drivers/soc/qcom/kernel_protect.c, called as early_initcall
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
#define rs_check_msm_kernel_protected RS_HIDE(_f0_1_3)
#endif
noused notrace static int __init rs_check_msm_kernel_protected(void)
{
	/*without addrlimit checking*/
	if (!IS_ENABLED(CONFIG_MSM_KERNEL_PROTECT_MPU)) {
		/*MPU protected kernel code is HLOS writable*/
		/*refer to msm_protect_kernel()/msm_protect_kernel_test()/KERNEL_PROTECT_MPU*/
		if (__put_user(1, &g_rs_msm_kernel_protect_test_data)) {
			/*error, can't write to text region*/
			g_rs_msm_kernel_protected = 1;
		}
	}

	return 0;
}
/*pure_initcall called after early_initcall(msm_protect_kernel)*/
pure_initcall(rs_check_msm_kernel_protected);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_kernel_text_page_ro RS_HIDE(_f0_1)
#endif
noused int static set_kernel_text_page_ro(unsigned long _addr)
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

	if ((_addr >= virt_start_rounded) && (_addr < virt_end_rounded)) {
		phys_addr_t phys_addr = __pa(_addr);

		u32 vmid_hlos = VMID_HLOS;
		int dest_perms = (PERM_READ | PERM_EXEC);

		/*hyp_assign_phys() calls XXalloc(..., GFP_KERNEL), thus should be called in preemptable context*/
		int ret = hyp_assign_phys(phys_addr, /*addr*/
				PAGE_SIZE, /*size*/
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
#define set_kernel_text_page_rw RS_HIDE(_f0_2)
#endif
noused int static set_kernel_text_page_rw(unsigned long _addr)
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

	if ((_addr >= virt_start_rounded) && (_addr < virt_end_rounded)) {
		phys_addr_t phys_addr = __pa(_addr);

		u32 vmid_hlos = VMID_HLOS;
		int dest_perms = (PERM_READ | PERM_WRITE | PERM_EXEC);

		/*hyp_assign_phys() calls XXalloc(..., GFP_KERNEL), thus should be called in preemptable context*/
		int ret = hyp_assign_phys(phys_addr, /*addr*/
				PAGE_SIZE, /*size*/
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
#define set_kernel_text_page_ro RS_HIDE(_f0_1)
#endif
noused int static set_kernel_text_page_ro(unsigned long _addr)
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
#define set_kernel_text_page_rw RS_HIDE(_f0_2)
#endif
noused int static set_kernel_text_page_rw(unsigned long _addr)
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
noused int __weak set_memory_ro(unsigned long virt, int numpages)
/*noused notrace static int set_memory_ro(unsigned long virt, int numpages)*/
{
	return rs_set_memory_ro_rodata(virt, numpages);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define set_memory_rw RS_HIDE(_fs)*/
#endif
noused int __weak set_memory_rw(unsigned long virt, int numpages)
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

extern raw_spinlock_t patch_lock;

noused __rs_weak DEFINE_RAW_SPINLOCK(patch_lock);

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

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_patch_text_real RS_HIDE(_fy_)
#endif
noused notrace static int __rs_patch_text_real(void *addr, void *data, size_t data_size, bool remap)
{
	int ret = 0;

	if (remap) {
		unsigned long flags;
		void *first_addr;
		int patched = 0;

	#if defined(CONFIG_ARM64)
		first_addr = rs_patch_map(addr, FIX_TEXT_POKE0, &flags, &patched);
		if (unlikely(!first_addr)) {
			ret = -EFAULT;
			goto out;
		} else if (IS_ERR(first_addr)) {
			ret = PTR_ERR(first_addr);
			goto out;
		}

		memmove(first_addr, data, data_size);
		flush_kernel_vmap_range(first_addr, data_size);
		if (patched) {
			rs_patch_unmap(FIX_TEXT_POKE0, &flags);
		}
	#else
		int first_size;
		int second_size;
		void *second_addr;
		int second_patched = 0;

		first_addr = rs_patch_map(addr, FIX_TEXT_POKE0, &flags, &patched);
		if (unlikely(!first_addr)) {
			ret = -EFAULT;
			goto out;
		} else if (IS_ERR(first_addr)) {
			ret = PTR_ERR(first_addr);
			goto out;
		}

		first_size = ((((uintptr_t)addr) & ~PAGE_MASK) + data_size);
		if (first_size > PAGE_SIZE) {
			second_size = first_size - PAGE_SIZE;
		} else {
			second_size = 0;
		}

		first_size = data_size - second_size;

		if (second_size) {
			second_addr = rs_patch_map((void *)((uintptr_t)addr + first_size), FIX_TEXT_POKE1, NULL, &second_patched);
			if (unlikely(!second_addr)) {
				ret = -EFAULT;
			} else if (IS_ERR(second_addr)) {
				ret = PTR_ERR(second_addr);
			} else {
				memmove(second_addr, (void *)((uintptr_t)data + first_size), second_size);
				flush_kernel_vmap_range(second_addr, second_size);
				if (second_patched) {
					rs_patch_unmap(FIX_TEXT_POKE1, NULL);
				}
			}
		}

		if (!ret) {
			memmove(first_addr, data, first_size);

			flush_kernel_vmap_range(first_addr, first_size);
		}

		if (patched) {
			rs_patch_unmap(FIX_TEXT_POKE0, &flags);
		}
	#endif
	} else {
		__acquire(&rs_patch_lock);
		memmove(addr, data, data_size);
		__release(&rs_patch_lock);
	}

	if (!ret) {
		flush_icache_range((uintptr_t)(addr),
			(uintptr_t)(addr) + data_size);
	}

out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_patch_text RS_HIDE(_f11_)
#endif
noused notrace static int __rs_patch_text(void *addr, void *data, size_t data_size)
{
	return __rs_patch_text_real(addr, data, data_size, true);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_patch_text_early RS_HIDE(_f12_)
#endif
noused notrace static int __rs_patch_text_early(void *addr, void *data, size_t data_size)
{
	return __rs_patch_text_real(addr, data, data_size, false);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_patch_text_common RS_HIDE(_f13_)
#endif
noused notrace static int __rs_patch_text_common(void *addr, void *data, size_t data_size)
{
	extern int scheduler_running;
	/*scheduler_running initialized int sched_init(), which called after mm_init()*/
	if (scheduler_running)
		return __rs_patch_text(addr, data, data_size);
	else
		return __rs_patch_text_real(addr, data, data_size, false);
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
noused notrace static int rs_mem_text_address_restore(mem_unprotect_struct *mem_unprotect)
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
noused notrace static int rs_mem_text_address_restore(mem_unprotect_struct *mem_unprotect)
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
noused notrace static int rs_rodata_address_restore(mem_unprotect_struct *mem_unprotect)
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

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_malloc_vars_page RS_HIDE(_l3)
#endif
noused notrace static void *rs_malloc_vars_page(void)
{
	void *ret;

#if defined(CONFIG_MODULES) && defined(MODULES_VADDR)
	/*为避免分配得到的缓冲区总是固定在MODULES_VADDR地址, 前面浪费一段最多4KB~128KB*/
	#if defined(CONFIG_RS_OBFUSCATED_NAME)
		#define g_vars_page_allocated RS_HIDE(_l3_)
	#endif
	static int g_vars_page_allocated;

	#define VARS_PAGE_RAMDOM_OFFSET_DIVISOR (16) /*(8)*/

	void *dummy_offset_buffer = NULL;
	char random_num;

	if (!g_vars_page_allocated) {
		g_vars_page_allocated = 1;

		RS_GET_RANDOM_BYTES(&random_num, sizeof(random_num));

		if (random_num < VARS_PAGE_RAMDOM_OFFSET_DIVISOR) {
			random_num += VARS_PAGE_RAMDOM_OFFSET_DIVISOR;
		}

		dummy_offset_buffer = module_alloc((random_num * PAGE_SIZE) / VARS_PAGE_RAMDOM_OFFSET_DIVISOR);
	}
#endif

	ret = module_alloc(RS_OP_VAR_PAGE_DATA_SIZE + RS_CRC_TABLE_SIZE);

	if (ret) {
		memset(ret, 0, RS_OP_VAR_PAGE_DATA_SIZE + RS_CRC_TABLE_SIZE);
	}

#if defined(CONFIG_MODULES) && defined(MODULES_VADDR)
	if (dummy_offset_buffer) {
		RS_MODULE_FREE(dummy_offset_buffer);
	}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_free_vars_page RS_HIDE(_l4)
#endif
noused notrace static void rs_free_vars_page(void *page)
{
	if (page) {
		RS_MODULE_FREE(page);
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_valid_vars_page RS_HIDE(_l5)
#endif
noused notrace static bool rs_is_valid_vars_page(void *page, bool first_alloc)
{
	if ((!((uintptr_t)page & ((1 << PAGE_SHIFT) - 1)))
	#if defined(CONFIG_MODULES) && defined(CONFIG_ARM64) && defined(CONFIG_RANDOMIZE_BASE) && defined(MODULES_VSIZE)
		&& (((uintptr_t)page >= module_alloc_base) && ((uintptr_t)page <= ((module_alloc_base + MODULES_VSIZE - (PAGE_SIZE + RS_CRC_TABLE_SIZE)))))
	#elif defined(CONFIG_MODULES)
		&& (((uintptr_t)page >= MODULES_VADDR) && ((uintptr_t)page <= (MODULES_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#elif defined(CONFIG_MMU)
		&& (((uintptr_t)page >= VMALLOC_START) && ((uintptr_t)page <= (VMALLOC_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#elif defined(MODULES_VADDR)
		&& (((uintptr_t)page >= MODULES_VADDR) && ((uintptr_t)page <= (MODULES_END - (PAGE_SIZE + RS_CRC_TABLE_SIZE))))
	#endif
		) {
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
		struct vm_struct *area = find_vm_area(page);
		if ((area) && ((area->flags & VM_USERMAP) == 0))
	#endif
		{
			if (first_alloc)
				return true;
			else if (/*access_ok(VERIFY_READ, (unsigned long)page, PAGE_SIZE)
				&&*/ rs_is_page_readonly((unsigned long)page))
				return true;
		}
	}

	return false;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_vars_internal RS_HIDE(_l6)
#endif
noused notrace static int set_vars_internal(unsigned var_flag_idx, size_t data_offs, void *data,
	size_t data_count, void *allocated_page)
{
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *data_ptr = (uintptr_t *)data;
	uintptr_t values[RS_OP_VAR_TUPLE_ELEMENTS_COUNT];
	uintptr_t *page;
	int ret = 0;
	uintptr_t tick, random_num;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;
	int is_text_rw_set = 0;
	bool flush_all = false;

#if !defined(RS_VARS_LOCK_USE_RWSEM)
do_loop:
#endif

	RS_VAR_WRITE_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);
	/*page = (uintptr_t *)(vars[var_index] ^ vars[var_index + 1] ^ vars[var_index + 2]);*/

	RS_LOG("svi,0,%u,%zu,%p\n", var_flag_idx, data_count, page);

	if ((page) && (!rs_is_valid_vars_page(page, false))) {
		RS_LOG("svi,0.1\n");
		page = NULL;
	}

	if (!page) {
	#if !defined(RS_VARS_LOCK_USE_RWSEM)
		RS_OP_VAR_FLAG_TYPE tmp_ov_flag;

		RS_VAR_WRITE_UNLOCK;
	#endif
		RS_LOG("svi,1\n");

		flush_all = true;

		if (allocated_page) {
			page = allocated_page;
		} else {
			RS_LOG("svi,2\n");
			page = rs_malloc_vars_page();

			if (!page) {
				RS_LOG("svi,3\n");
				ret = -ENOMEM;
				goto out;
			}

			if (!rs_is_valid_vars_page(page, true)) {
				RS_LOG("svi,4\n");
				ret = -EFAULT;
				if (page != allocated_page) {
					rs_free_vars_page(page);
				}
				page = NULL;
				goto out;
			}
		}

		RS_GET_RANDOM_BYTES(&random_num, sizeof(random_num));

		offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;
		if (((offs + 1 + data_offs + data_count) * sizeof(*page)) >= 1024) {
			RS_LOG("svi,5,%zu\n", offs);

			ret = -EINVAL;
			if (page != allocated_page) {
				rs_free_vars_page(page);
			}
			page = NULL;
			goto out;
		}

		RS_GET_RANDOM_BYTES(page, RS_OP_VARS_STORE_SIZE);

		if (!is_text_rw_set) {
			if (!rs_set_text_vars_rw(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
				is_text_rw_set = 1;
			} else {
				RS_LOG("svi,5.1\n");

				ret = -EFAULT;
				if (page != allocated_page) {
					rs_free_vars_page(page);
				}
				page = NULL;
				goto out;
			}
		}

	#if !defined(RS_VARS_LOCK_USE_RWSEM)
		RS_VAR_WRITE_LOCK;

	#if defined(RS_HIDE_OP_VAR_FLAG)
		rs_fetch_op_var_flag(&tmp_ov_flag);
	#else
		tmp_ov_flag = g_rs_op_var_flag;
	#endif
		if (tmp_ov_flag == rs_op_var_flag) {
			ret = rs_set_text_vars(vars, page, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT);
			if (ret) {
				goto out_with_lock;
			}
		} else {
			RS_LOG("svi,6\n");
			if (page != allocated_page) {
				rs_free_vars_page(page);
			}

			if (tmp_ov_flag) {
				RS_VAR_WRITE_UNLOCK;

				if (is_text_rw_set) {
					if (!rs_set_text_vars_ro(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
						is_text_rw_set = 0;
					}
				}

				RS_LOG("svi,7\n");
				goto do_loop;
			} else {
				ret = -EINTR;
				goto out_with_lock;
			}
		}
	#endif

		memset(page, 0, RS_OP_VARS_STORE_SIZE);

		{
			struct timeval tmp_time;

			do_gettimeofday(&tmp_time);

			tick = jiffies;

			tick ^= tmp_time.tv_sec;
		}

	#if (RS_OP_VAR_TUPLE_ELEMENTS_COUNT > RS_OP_VAR_TUPLE_ELEMENTS_USED_COUNT)
		RS_GET_RANDOM_BYTES(values, RS_OP_VAR_TUPLE_ELEMENTS_SIZE);
	#endif

		var_index = ((random_num & 0xff) / RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
		value_shift = (random_num & 0xff) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;
		values[value_shift] = tick;
		values[(value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT] = random_num;
		values[(value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT] = (tick ^ random_num ^ (uintptr_t)page);

		ret = rs_set_text_vars(&vars[var_index], values, sizeof(values[0]), ARRAY_SIZE(values));

		if (ret) {
			uintptr_t zero = 0;
			/*rs_op_var_flag = (rs_op_var_flag << 16) >> 16;*/

			/*memset(values, 0, sizeof(values));*/
			RS_LOG("svi,8\n");
			ret = rs_fill_text_vars(&vars[var_index], &zero, sizeof(vars[0]), RS_OP_VAR_TUPLE_ELEMENTS_COUNT);

			if (page != allocated_page) {
				rs_free_vars_page(page);
			}

			if (!ret) {
				rs_op_var_flag = 0;

			#if defined(RS_HIDE_OP_VAR_FLAG)
				rs_store_op_var_flag(rs_op_var_flag);
			#else
				g_rs_op_var_flag = rs_op_var_flag;
			#endif
			} else {
				/*
				if (page != allocated_page)
				{
					rs_free_vars_page(page);
				}
				*/
			}

			RS_LOG("svi,9\n");
			ret = -EINVAL;
		#if !defined(RS_VARS_LOCK_USE_RWSEM)
			goto out_with_lock;
		#else
			goto out;
		#endif
		} else {
			RS_LOG("svi,a\n");

			{
				size_t tmp_offs = value_shift ^ var_index;
				if (tmp_offs < gfInvalid) {
					/*留位置给bad_boot等全局标志*/
					tmp_offs = gfInvalid;
				}

				/*RS_LOG("ioo,2,%d\n", (int)tmp_offs);*/

				tmp_offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

				page[tmp_offs] = (uintptr_t)crc32_le(0, (unsigned char const *)&values[0], RS_OP_VAR_TUPLE_ELEMENTS_SIZE);
			}

			rs_op_var_flag |= (1 << var_flag_idx);

			rs_op_var_flag &= ~(0xf << ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4)));
			rs_op_var_flag &= ~(0xf << ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/)));

			rs_op_var_flag |= ((var_index & 0xf) << ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4)));
			rs_op_var_flag |= ((value_shift & 0xf) << ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/)));

			/*g_rs_op_var_flag = rs_op_var_flag;*/
		}
	} else {
		RS_LOG("svi,b\n");
		random_num = vars[var_index + ((value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT)];

		offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;
		if (((offs + 1 + data_offs + data_count) * sizeof(*page)) >= 1024) {
			RS_LOG("svi,c,%zu\n", offs);

			ret = -EINVAL;
		#if !defined(RS_VARS_LOCK_USE_RWSEM)
			goto out_with_lock;
		#else
			goto out; /*BUG_ON(0);*/
		#endif
		}

		rs_op_var_flag |= (1 << var_flag_idx);

		rs_set_memory_rw((unsigned long)page, 1);
	}

	/*
	offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;
	if (((offs + 1 + data_offs + data_count) * sizeof(*page)) >= 1024) {
		BUG_ON(0);
	}
	*/

	offs = ((var_flag_idx * 1024) / sizeof(*page)) + offs;

	page[offs] = random_num ^ (uintptr_t)page ^ offs;

	memcpy(&page[offs + 1 + data_offs], data_ptr, sizeof(*page) * data_count); /*page[offs + 1 + data_offs] = *data_ptr;*/

	if (flush_all) {
		RS_FLUSH_DCACHE_RANGE((unsigned long)page, (unsigned long)page + PAGE_SIZE);
	} else {
		unsigned long flush_addr = (unsigned long)(&page[offs + 1 + data_offs]);
		RS_FLUSH_DCACHE_RANGE(flush_addr, flush_addr + (sizeof(*page) * data_count));
	}

	rs_set_memory_ro((unsigned long)page, 1);

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_store_op_var_flag(rs_op_var_flag);
#else
	g_rs_op_var_flag = rs_op_var_flag;
#endif

#if !defined(RS_VARS_LOCK_USE_RWSEM)
out_with_lock:
	RS_LOG("svi,d\n");
	RS_VAR_WRITE_UNLOCK;
#endif
out:
	if (is_text_rw_set) {
		if (!rs_set_text_vars_ro(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
			is_text_rw_set = 0;
		}
	}
#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_WRITE_UNLOCK;
#endif

	RS_LOG("svi,e\n");
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define reset_vars_internal RS_HIDE(_l7)
#endif
noused notrace static int reset_vars_internal(unsigned var_flag_idx)
{
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	int ret;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift;
	int is_text_rw_set = 0;

	ret = -EINVAL;

	if (!is_text_rw_set) {
		if (!rs_set_text_vars_rw(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
			is_text_rw_set = 1;
		}
	}

	RS_VAR_WRITE_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	if (!(rs_op_var_flag & (1 << var_flag_idx))) {
		goto out;
	}

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

	if ((page) && rs_is_valid_vars_page(page, false)) {
		rs_op_var_flag &= ~(1 << var_flag_idx);

		rs_op_var_flag &= ~(0xf << ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/)));

		if (((rs_op_var_flag << 16) >> 16) == 0) {
			uintptr_t zero = 0;
			/*uintptr_t values[RS_OP_VAR_TUPLE_ELEMENTS_COUNT] = {0};*/

			rs_fill_text_vars(&vars[var_index], &zero, sizeof(vars[0]), RS_OP_VAR_TUPLE_ELEMENTS_COUNT);

			rs_set_memory_rw((unsigned long)page, 1);

			rs_free_vars_page(page);

			rs_op_var_flag = 0;
		}

	#if defined(RS_HIDE_OP_VAR_FLAG)
		rs_store_op_var_flag(rs_op_var_flag);
	#else
		g_rs_op_var_flag = rs_op_var_flag;
	#endif

		ret = 0;
	}

out:
	RS_VAR_WRITE_UNLOCK;

	if (is_text_rw_set) {
		if (!rs_set_text_vars_ro(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
			is_text_rw_set = 0;
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_op_var_exit RS_HIDE(_l8)
#endif
noused notrace static void __exit rs_op_var_exit(void)
{
	reset_vars_internal(opvBypass);
	reset_vars_internal(opvPermit);
	reset_vars_internal(opvChecksum);
}
module_exit(rs_op_var_exit);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_vars_internal RS_HIDE(_l8_)
#endif
noused notrace static int get_vars_internal(unsigned var_flag_idx, size_t data_offs, void *data,
	size_t data_count)
{
#if 1
	int ret = -EINVAL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	/*RS_LOG("gvi,-1,%x,%u,%d\n", rs_op_var_flag, var_flag_idx);*/

	if (!(rs_op_var_flag & (1 << var_flag_idx))) {
		/*RS_LOG("gvi,-2\n");*/
		ret = -ESPIPE; /*var not set before*/
		goto out;
	}

	if ((!data) || (!data_count)) {
		goto out;
	}

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	/*RS_LOG("gvi,0,%d,%d,%d\n", (int)var_flag_idx, (int)var_index, (int)value_shift);*/
	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

	if ((page) && rs_is_valid_vars_page(page, false)) {
		uintptr_t checksum = 0;
		/*RS_LOG("gvi,1\n");*/
		checksum = (uintptr_t)crc32_le(0, (unsigned char const *)&vars[var_index], RS_OP_VAR_TUPLE_ELEMENTS_SIZE);
		{
			size_t tmp_offs = value_shift ^ var_index;
			if (tmp_offs < gfInvalid) {
				/*留位置给bad_boot等全局标志*/
				tmp_offs = gfInvalid;
			}

			/*RS_LOG("gvi,2,%d\n", (int)tmp_offs);*/

			offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

			if (checksum == page[offs]) {
				uintptr_t random_num = vars[var_index + ((value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT)];

				offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;

				offs = ((var_flag_idx * 1024) / sizeof(*page)) + offs;
				/*RS_LOG("gvi,3,%d\n", (int)offs);*/

				if (page[offs] == ((uintptr_t)page ^ offs ^ random_num)) {
					memcpy(data, &page[offs + 1 + data_offs], sizeof(*page) * data_count);
					ret = 0;
				}
			} else {
				ret = -ESPIPE; /* Illegal seek */
			}
		}
	}

out:
	RS_VAR_READ_UNLOCK;

	return ret;
#endif
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_checksum_vars RS_HIDE(_lb)
#endif
noused notrace static int set_checksum_vars(size_t data_offs, void *data, void *allocated_page)
{
	/*目前opvChecksum类型的空间未使用？*/
	return set_vars_internal(opvChecksum, data_offs, data, 1, allocated_page);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_ok RS_HIDE(_lc)
#endif
RS_DEFINE_FC_BEGIN(is_op_ok);

noused RS_DEFINE_FC_BODY(is_op_ok) static int is_op_ok(unsigned var_flag_idx, int op_type)
{
	/*return 0;*/
#if 1
	int ret = 0;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	/*RS_LOG("ioo,-1,%x,%u,%d\n", rs_op_var_flag, var_flag_idx, op_type);*/

	if (!(rs_op_var_flag & (1 << var_flag_idx))) {
		/*RS_LOG("ioo,-2\n");*/
	#if !defined(RS_VARS_LOCK_USE_RWSEM)
		RS_VAR_READ_UNLOCK;
	#endif
		goto out;
	}

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	/*RS_LOG("ioo,0,%d,%d,%d\n", (int)var_flag_idx, (int)var_index, (int)value_shift);*/
	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

#if !defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	if ((page) && rs_is_valid_vars_page(page, false)) {
		uintptr_t checksum = 0;
		/*RS_LOG("ioo,1\n");*/
		checksum = (uintptr_t)crc32_le(0, (unsigned char const *)&vars[var_index], RS_OP_VAR_TUPLE_ELEMENTS_SIZE);
		{
			size_t tmp_offs = value_shift ^ var_index;
			if (tmp_offs < gfInvalid) {
				/*留位置给bad_boot等全局标志*/
				tmp_offs = gfInvalid;
			}

			/*RS_LOG("ioo,2,%d\n", (int)tmp_offs);*/

			offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

			if (checksum == page[offs]) {
				uintptr_t random_num = vars[var_index + ((value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT)];

				offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;

				offs = ((var_flag_idx * 1024) / sizeof(*page)) + offs;
				/*RS_LOG("ioo,3,%d\n", (int)offs);*/

				if ((page[offs] == ((uintptr_t)page ^ offs ^ random_num))
					&& ((1 << op_type) & page[offs + 1])) {
					/*RS_LOG("ioo,4\n");*/
					ret = 1;
				}
			}
		}
	}

out:
#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	return ret;
#endif
}

RS_DEFINE_FC_END(is_op_ok);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_vars_checksum RS_HIDE(_ld)
#endif
noused notrace static int set_vars_checksum(RS_CHECKSUM_TYPE *checksum, size_t checksum_count, size_t data_offs)
{
	int ret = -EINVAL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	RS_VAR_WRITE_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

	if ((page) && rs_is_valid_vars_page(page, false)) {
		size_t tmp_offs = value_shift ^ var_index;

		if (tmp_offs < gfInvalid) {
			/*留位置给bad_boot等全局标志*/
			tmp_offs = gfInvalid;
		}

		if (((tmp_offs + data_offs + checksum_count) * sizeof(*page)) >= 1024) {
			ret = -EINVAL;
			goto out; /*BUG_ON(0);*/
		}

		offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

		rs_set_memory_rw((unsigned long)page, 1);
		memcpy(&page[offs + data_offs], checksum, (sizeof(*page) * checksum_count));

		{
			unsigned long flush_addr = (unsigned long)(&page[offs + data_offs]);
			RS_FLUSH_DCACHE_RANGE(flush_addr, flush_addr + (sizeof(*page) * checksum_count));
		}

		rs_set_memory_ro((unsigned long)page, 1);

		ret = 0;
	}

out:
	RS_VAR_WRITE_UNLOCK;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_vars_checksum RS_HIDE(_le)
#endif
noused notrace static int get_vars_checksum(RS_CHECKSUM_TYPE *checksum_ptr, size_t data_offs)
{
	int ret = -EINVAL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	if (!checksum_ptr) {
		goto out_no_lock;
	}

	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

#if !defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	if ((page) && rs_is_valid_vars_page(page, false)) {
		size_t tmp_offs = value_shift ^ var_index;

		if (tmp_offs < gfInvalid) {
			/*留位置给bad_boot等全局标志*/
			tmp_offs = gfInvalid;
		}

		if (((tmp_offs + data_offs) * sizeof(*page)) >= 1024) {
			goto out; /*BUG_ON(0);*/
		}

		offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

		*checksum_ptr = page[offs + data_offs];

		ret = 0;
	}

out:
#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

out_no_lock:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_vars_global_flag RS_HIDE(_lf)
#endif
noused notrace static int set_vars_global_flag(size_t flag_enum, intptr_t flag)
{
	int ret = -EINVAL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	if (gfInvalid <= flag_enum) {
		goto out_no_lock;
	}

	/*RS_LOG("svgf,%zu\n", flag_enum);*/
	RS_VAR_WRITE_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

	/*RS_LOG("svgf,0,%zu,%p\n", flag_enum, page);*/
	if ((page) && rs_is_valid_vars_page(page, false)) {
		size_t tmp_offs = value_shift ^ var_index;

		if (tmp_offs < gfInvalid) {
			/*留位置给bad_boot等全局标志*/
			tmp_offs = 0;
		} else {
			tmp_offs -= (size_t)gfInvalid;
		}
		/*RS_LOG("svgf,1\n");*/
		offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

		rs_set_memory_rw((unsigned long)page, 1);
		page[offs + flag_enum] = (uintptr_t)flag;

		{
			unsigned long flush_addr = (unsigned long)(&page[offs + flag_enum]);
			RS_FLUSH_DCACHE_RANGE(flush_addr, flush_addr + sizeof(page[0]));
		}

		rs_set_memory_ro((unsigned long)page, 1);

		ret = 0;
	}

	RS_VAR_WRITE_UNLOCK;

out_no_lock:
	/*RS_LOG("svgf,e\n");*/
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_vars_global_flag RS_HIDE(_lg)
#endif
noused notrace static int get_vars_global_flag(size_t flag_enum, intptr_t *flag_ptr)
{
	int ret = -EINVAL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	if ((gfInvalid <= flag_enum) || (!flag_ptr)) {
		goto out_no_lock;
	}
	/*RS_LOG("gvgf,0\n");*/
	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif
	/*RS_LOG("gvgf,1,%x\n", rs_op_var_flag);*/
	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

#if !defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	/*RS_LOG("gvgf,1.1,%p\n", page);*/
	if ((page) && rs_is_valid_vars_page(page, false)) {
		size_t tmp_offs = value_shift ^ var_index;

		if (tmp_offs < gfInvalid) {
			/*留位置给bad_boot等全局标志*/
			tmp_offs = 0;
		} else {
			tmp_offs -= (size_t)gfInvalid;
		}
		/*RS_LOG("gvgf,2\n");*/
		offs = ((1024 * opvInvalid) / sizeof(*page)) + tmp_offs;

		*flag_ptr = (intptr_t)page[offs + flag_enum];

		ret = 0;
	}

#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

out_no_lock:
	/*RS_LOG("gvgf,e\n");*/
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_permit_vars RS_HIDE(_l9)
#endif
noused notrace static int set_permit_vars(void *data)
{
#if defined(CONFIG_RS_CHECK_FUNC)
	uintptr_t flag = *((uintptr_t *)data);
	intptr_t value;

	if (flag & (1 << ovFuncCheck)) {
		value = fcbdfPermitted;
	} else {
		value = ((rs_get_bootup_set()) ? fcbdfInited : fcbdfNotInited);
	}

	set_vars_global_flag(gfBootupDone, value);
#endif

	return set_vars_internal(opvPermit, 0, data, 1, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_bypass_vars RS_HIDE(_la)
#endif
noused notrace static int set_bypass_vars(void *data)
{
#if defined(CONFIG_RS_CHECK_FUNC)
	uintptr_t flag = *((uintptr_t *)data);
	intptr_t value;

	if (flag & (1 << ovFuncCheck)) {
		value = fcbdfByPassed;
	} else {
		value = ((rs_get_bootup_set()) ? fcbdfInited : fcbdfNotInited);
	}

	set_vars_global_flag(gfBootupDone, value);
#endif

	return set_vars_internal(opvBypass, 0, data, 1, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_system_mounted RS_HIDE(_la_a)
#endif
static int g_rs_system_mounted;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_system_mounted RS_HIDE(_la_b)
#endif
noused notrace static int rs_get_system_mounted(void)
{
	return g_rs_system_mounted;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_set_system_mounted RS_HIDE(_la_c)
#endif
noused notrace static void rs_set_system_mounted(void)
{
	if (!g_rs_system_mounted) {
		g_rs_system_mounted = rs_get_set_value();
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_op_permitted_by_file RS_HIDE(_la_d)
#endif

RS_DEFINE_FC_BEGIN(is_op_permitted_by_file);

noused notrace RS_DEFINE_FC_BODY(is_op_permitted_by_file) static noinline int is_op_permitted_by_file(int op_type)
{
	int ret;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_permitted_by_file_checked RS_HIDE(_la_d_1)
#endif
	static int g_permitted_by_file_checked;

	ret = 0;

	if ((op_type >= 0) && (op_type < ovInvalid)) {
		int op_flag = (1 << op_type);

		/*RS_LOG("obf,0,%d\n", op_type);*/

		if (op_flag & g_permitted_by_file_checked) {
			/*RS_LOG("obf,1\n");*/
		} else if ((g_bypass_mark_dirs[op_type].str_len) && (rs_get_system_mounted())) {
		#if defined(RS_ENCRYPT_STR)
			char *buffer;

			buffer = (char *)RS_GET_BUFFER();
			if (!buffer) {
				/*RS_LOG("obf,1.1\n");*/
				goto out;
			}

			memcpy(buffer, g_bypass_mark_dirs[op_type].str, g_bypass_mark_dirs[op_type].str_len + 1);
			simple_str_encrypt(buffer, g_bypass_mark_dirs[op_type].str_len, op_type);

			/*RS_LOG("obf,2,%s\n", buffer);*/

			if (is_dir_exist(buffer))
		#else
			if (is_dir_exist(g_bypass_mark_dirs[op_type].str))
		#endif
			{
				int res;
				unsigned long local_rs_permit = 0;
				/*RS_LOG("obf,3\n");*/

				res = get_vars_internal(opvPermit, 0, &local_rs_permit, 1);
				if ((!res) || (res == -ESPIPE)) {
					/*RS_LOG("obf,4\n");*/
					if (!(local_rs_permit & op_flag)) {
						/*RS_LOG("obf,5\n");*/
						local_rs_permit |= (unsigned long)op_flag;

						set_permit_vars(&local_rs_permit);
					}
				}


				ret = 1;
			} else {
				/*RS_LOG("obf,6\n");*/
			}

		#if defined(RS_ENCRYPT_STR)
			RS_FREE_BUFFER(buffer);
		#endif

			g_permitted_by_file_checked |= op_flag;
		}
	}

#if defined(RS_ENCRYPT_STR)
out:
#endif
	return ret;
}

RS_DEFINE_FC_END(is_op_permitted_by_file);

noused static int set_checksum_vars(size_t data_offs, void *data, void *allocated_page);
noused static int set_vars_checksum(RS_CHECKSUM_TYPE *checksum, size_t checksum_count, size_t data_offs);

#if defined(CONFIG_RS_CHECK_FUNC)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_func_checksums RS_HIDE(_CA)
#endif
noused notrace static int rs_init_func_checksums(void);
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_malloc_vars_page RS_HIDE(_l3)
#endif
noused notrace static void *rs_malloc_vars_page(void);
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_free_vars_page RS_HIDE(_l4)
#endif
noused notrace static void rs_free_vars_page(void *page);

#if defined(CONFIG_ROOT_RESTRICT)


#if defined(RS_HAS_EMMC_SHORTCUTS)

/*below from partition_define.h*/

#define PART_MAX_COUNT 40

typedef enum  {
	EMMC = 1,
	NAND = 2,
} dev_type;

typedef enum {
	USER = 0,
	BOOT_1,
	BOOT_2,
	RPMB,
	GP_1,
	GP_2,
	GP_3,
	GP_4,
} Region;


struct excel_info {
	char *name;
	unsigned long long size;
	unsigned long long start_address;
	dev_type type;
	unsigned int partition_idx;
	Region region;
};

extern struct excel_info *PartInfo;

#endif

/*system/recovery/boot/uboot(lk)/apps/oem/survival*/

enum {
	ptRecovery,
	ptSystem,
	ptBoot,
	ptLK,
	ptApps,
#if defined(RS_HAS_OEM_PARTITION)
	ptOem,
#endif
#if defined(RS_HAS_SURVIVAL_PARTITION)
	ptSurvival,
#endif
#if defined(RS_HAS_VENDOR_PARTITION)
	ptVendor,
#endif
#if defined(RS_HAS_VBMETA_PARTITION)
	ptVBMeta,
#endif
#if defined(RS_HAS_SUPER_PARTITION)
	ptSuper,
#endif
	ptInvalid,
};

#define RS_PART_NO_STORE_TYPE u32

typedef struct tag_emmc_part_names_info {
	size_t disk_name_len;
	char disk_name[BDEVNAME_SIZE + 16];
	char part_names[ptInvalid][sizeof(RS_PART_NO_STORE_TYPE)];
} emmc_part_names_info;

#define RS_UFS_PART_NAME_MAX_LEN (8)

typedef struct tag_ufs_part_names_info {
	char part_names[ptInvalid][RS_UFS_PART_NAME_MAX_LEN]; /* "sdxNNN"*/
} ufs_part_names_info;

typedef struct tag_part_names_info {
#if defined(RS_USE_UFS_DEV)
	int is_ufs;
#endif

	union {
		emmc_part_names_info emmc;
		ufs_part_names_info ufs;
	} info;
} part_names_info;

typedef struct tag_part_str_idx {
	int idx;
	int offset;
} part_str_idx;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define part_str_indexes RS_HIDE(_la_1)
#endif

noused static const part_str_idx part_str_indexes[ptInvalid] = {
	{csRecovery, sizeof("/sbin/") - 1},
#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	{csSystem, 1},
	{csBoot, 0},
#if defined(RS_MTK_PLATFORM)
	{csLK, 0},
#else
	{csABoot, 0},
#endif
#else
	{csAndroid, 0},
	{csBootImg, 0},
	{csUBoot, 0},
#endif
	{csApps, 1},
#if defined(RS_HAS_OEM_PARTITION)
	{csOem, 1},
#endif
#if defined(RS_HAS_SURVIVAL_PARTITION)
	{csSurvival, 0},
#endif
#if defined(RS_HAS_VENDOR_PARTITION)
	{csVendor, 1},
#endif
#if defined(RS_HAS_VBMETA_PARTITION)
	{csVBMeta, 0},
#endif
#if defined(RS_HAS_SUPER_PARTITION)
	{csSuper, 0},
#endif
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_save_part_names_info RS_HIDE(_la_2)
#endif
noused notrace static int rs_save_part_names_info(const part_names_info * info)
{
	size_t data_count = (sizeof(*info) + sizeof(uintptr_t) - 1) / sizeof(uintptr_t);
	RS_LOG("spni\n");
	return set_vars_internal(opvChecksum, 0, (void *)info, data_count, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_part_names_emmc RS_HIDE(_la_21)
#endif
noused notrace static int rs_init_part_names_emmc(void)
{
/*#define RS_DISK_OWNER_MODULE_PUT*/

	noused int ret = 0;
	noused struct disk_part_iter piter;
	noused struct hd_struct *part;
	noused struct gendisk *disk;
#if defined(RS_DISK_OWNER_MODULE_PUT)
	const struct block_device_operations *fops;
	struct module *owner;
#endif
	noused int partno, i, count;
#if defined(RS_HAS_EMMC_SHORTCUTS)
	noused int j, part_count;
#endif
	dev_t dev;
	part_names_info *save_info;
	extern void module_put(struct module *module);

	/*RS_LOG("ipne,0\n");*/

#if defined(RS_HAS_EMMC_SHORTCUTS)
	if (!PartInfo) {
		/*RS_LOG("ipne,1\n");*/
		ret = -ENODEV;
		goto out_no_save_info;
	}
#endif

	if (RS_BUFFER_SIZE < (sizeof(*save_info) + sizeof(RS_PART_NO_STORE_TYPE))) {
		/*RS_LOG("ipne,2\n");*/
		ret = -E2BIG;
		goto out_no_save_info;
	}

	save_info = (part_names_info *)RS_GET_BUFFER();
	if (!save_info) {
		/*RS_LOG("ipne,3\n");*/
		ret = -ENOMEM;
		goto out;
	}

	memset(save_info, 0, sizeof(*save_info) + sizeof(RS_PART_NO_STORE_TYPE));

#if defined(RS_USE_UFS_DEV)
	save_info->is_ufs = g_no_emmc;
#endif

	dev = MKDEV(MMC_BLOCK_MAJOR, 0);

	/*partno = 0;*/
	disk = get_gendisk(dev, &partno);

	if (!disk) {
		/*RS_LOG("ipne,4\n");*/
		ret = -ENXIO;
		goto out;
	}

	/*RS_LOG("ipne,5,%s\n", disk->disk_name);*/

#if defined(RS_HAS_EMMC_SHORTCUTS)
	for (i = 0; i < PART_MAX_COUNT ; i++) {
		if (!PartInfo[i].size) {
			/*fat? part before bmtpool*/
			i += 2;
			break;
		}

		if ((PartInfo[i].name < (char *)PAGE_OFFSET) || ((int)(PartInfo[i].type) > NAND)
			|| ((int)(PartInfo[i].type) < 0)
			|| ((int)(PartInfo[i].region) > GP_4) || ((int)(PartInfo[i].region) < 0)
			|| (PartInfo[i].partition_idx >= (unsigned int)PART_MAX_COUNT)) {
			break;
		}

		if (STRICMP(PartInfo[i].name, check_strings[csBmtPool].str, csBmtPool) == 0) {
			i++;
			break;
		}
	}

	part_count = i;

	if (!part_count) {
		/*RS_LOG("ipne,6\n");*/
		ret = -EINVAL;
		goto out;
	}
#endif

	count = 0;

	disk_part_iter_init(&piter, disk, 0);
	while ((part = disk_part_iter_next(&piter))) {
	#if defined(RS_HAS_EMMC_SHORTCUTS)
		for (i = 0; i < ptInvalid; i++) {
			for (j = 0; j < part_count; j++) {
				if ((PartInfo[j].partition_idx != 0) && (PartInfo[j].partition_idx == part->partno)) {
					bool ok; /* = false;*/
					int idx, offset;
					const char *part_name = PartInfo[j].name;

					/*RS_LOG("ipne,7,%s,%d\n", part_name, (int)part->partno);*/

					idx = part_str_indexes[i].idx;
					offset = part_str_indexes[i].offset;

				#if defined(RS_AB_UPDATE_SUPPORT)
					{
						int part_name_len;
						bool is_ab_update = is_ab_part_name(part_name, &part_name_len);
						if (is_ab_update) {
							ok = (STRNICMP_EX(part_name, check_strings[idx].str, offset, part_name_len, idx) == 0);
						} else {
							ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
						}
					}
				#else
					ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
				#endif

					if (ok) {
						/*RS_LOG("ipne,8,%s\n", part_name);*/

						if (!count) {
							/*refer to disk_name()*/
							char diskname_fmt[sizeof("%sp") + 1];
							size_t len = strlen(disk->disk_name);

							if (isdigit(disk->disk_name[len - 1])) {
								memcpy(diskname_fmt, check_strings[csDiskNameWDigitFmt].str, check_strings[csDiskNameWDigitFmt].str_len + 1);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt(diskname_fmt, check_strings[csDiskNameWDigitFmt].str_len, csDiskNameWDigitFmt);
							#endif
								save_info->info.emmc.disk_name_len = scnprintf(save_info->info.emmc.disk_name,
									sizeof(save_info->info.emmc.disk_name), diskname_fmt, disk->disk_name);
							} else {
								memcpy(diskname_fmt, check_strings[csDiskNameFmt].str, check_strings[csDiskNameFmt].str_len + 1);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt(diskname_fmt, check_strings[csDiskNameFmt].str_len, csDiskNameFmt);
							#endif
								save_info->info.emmc.disk_name_len = scnprintf(save_info->info.emmc.disk_name,
									sizeof(save_info->info.emmc.disk_name), diskname_fmt, disk->disk_name);
							}
						}

						if (!save_info->info.emmc.part_names[i][0]) {
							char tmp_buf[sizeof(RS_PART_NO_STORE_TYPE) + 1];
							size_t len = scnprintf(tmp_buf, sizeof(tmp_buf), "%d", part->partno);
							if (len < sizeof(RS_PART_NO_STORE_TYPE))
								len++;

							memcpy(save_info->info.emmc.part_names[i], tmp_buf, len);

							count++;
							if (count >= ptInvalid) {
								goto out_of_loop;
							}
						}

						break;
					}

					i = ptInvalid; /*to goto next partition loop*/
					break;
				}
			}
		}
	#else
		struct partition_meta_info *info = part->info;

		if ((info) && (info->volname[0])) {
			const char *part_name = info->volname;

			/*RS_LOG("ipne,9,%s\n", part_name);*/

			for (i = 0; i < ptInvalid; i++) {
				bool ok; /* = false;*/
				int idx, offset;

				idx = part_str_indexes[i].idx;
				offset = part_str_indexes[i].offset;

			#if defined(RS_AB_UPDATE_SUPPORT)
				{
					int part_name_len;
					bool is_ab_update = is_ab_part_name(part_name, &part_name_len);

					if (is_ab_update) {
						ok = (STRNICMP_EX(part_name, check_strings[idx].str, offset, part_name_len, idx) == 0);
					} else {
						ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
					}
				}
			#else
				ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
			#endif

				if (ok) {
					/*RS_LOG("ipne,a,%s\n", part_name);*/

					if (!count) {
						/*refer to disk_name()*/
						char diskname_fmt[sizeof("%sp") + 1];
						size_t len = strlen(disk->disk_name);

						if (isdigit(disk->disk_name[len - 1])) {
							memcpy(diskname_fmt, check_strings[csDiskNameWDigitFmt].str, check_strings[csDiskNameWDigitFmt].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(diskname_fmt, check_strings[csDiskNameWDigitFmt].str_len, csDiskNameWDigitFmt);
						#endif
							save_info->info.emmc.disk_name_len = scnprintf(save_info->info.emmc.disk_name,
								sizeof(save_info->info.emmc.disk_name), diskname_fmt, disk->disk_name);
						} else {
							memcpy(diskname_fmt, check_strings[csDiskNameFmt].str, check_strings[csDiskNameFmt].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(diskname_fmt, check_strings[csDiskNameFmt].str_len, csDiskNameFmt);
						#endif
							save_info->info.emmc.disk_name_len = scnprintf(save_info->info.emmc.disk_name,
								sizeof(save_info->info.emmc.disk_name), diskname_fmt, disk->disk_name);
						}
					}

					if (!save_info->info.emmc.part_names[i][0]) {
						char tmp_buf[sizeof(RS_PART_NO_STORE_TYPE) + 1];
						size_t len = scnprintf(tmp_buf, sizeof(tmp_buf), "%d", part->partno);

						if (len < sizeof(RS_PART_NO_STORE_TYPE))
							len++;

						memcpy(save_info->info.emmc.part_names[i], tmp_buf, len);

						count++;
						if (count >= ptInvalid) {
							goto out_of_loop;
						}
					}

					break;
				}
			}
		}
	#endif
	}

out_of_loop:
	disk_part_iter_exit(&piter);

#if defined(RS_DISK_OWNER_MODULE_PUT)
	fops = disk->fops;
	if (fops) {
		owner = (struct module *)(fops->owner);

		if (owner)
			module_put(owner);
	}
#endif

	put_disk(disk);

	if (count) {
		ret = rs_save_part_names_info(save_info);
	}

out:
	if (save_info) {
		RS_FREE_BUFFER(save_info);
	}

out_no_save_info:
	/*RS_LOG("ipne,e,%d\n", ret);*/

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_blkdev_path_by_part_name_emmc RS_HIDE(_la_22)
#endif
noused notrace static int rs_get_blkdev_path_by_part_name_emmc(const char *part_name,
	char *buffer, size_t buffer_size)
{
/*#define RS_DISK_OWNER_MODULE_PUT*/

	noused int ret = 0;
	noused struct disk_part_iter piter;
	noused struct hd_struct *part;
	noused struct gendisk *disk;
#if defined(RS_DISK_OWNER_MODULE_PUT)
	const struct block_device_operations *fops;
	struct module *owner;
#endif
	noused int partno, i;
#if defined(RS_HAS_EMMC_SHORTCUTS)
	noused int j, part_count;
#endif
	dev_t dev;
	extern void module_put(struct module *module);

	RS_LOG("gbdpe,0\n");

#if defined(RS_HAS_EMMC_SHORTCUTS)
	if (!PartInfo) {
		RS_LOG("gbdpe,1\n");
		ret = -ENODEV;
		goto out;
	}
#endif

	if ((!part_name) || (!part_name[0]) || (!buffer)
		|| (buffer_size < sizeof("/dev/block/mmcblk0p1"))) {
		RS_LOG("gbdpe,1\n");
		ret = -EINVAL;
		goto out;
	}

	dev = MKDEV(MMC_BLOCK_MAJOR, 0);

	/*partno = 0;*/
	disk = get_gendisk(dev, &partno);

	if (!disk) {
		RS_LOG("gbdpe,4\n");
		ret = -ENXIO;
		goto out;
	}

	RS_LOG("gbdpe,5,%s\n", disk->disk_name);

#if defined(RS_HAS_EMMC_SHORTCUTS)
	for (i = 0; i < PART_MAX_COUNT ; i++) {
		if (!PartInfo[i].size) {
			/*fat? part before bmtpool*/
			i += 2;
			break;
		}

		if ((PartInfo[i].name < (char *)PAGE_OFFSET) || ((int)(PartInfo[i].type) > NAND)
			|| ((int)(PartInfo[i].type) < 0)
			|| ((int)(PartInfo[i].region) > GP_4) || ((int)(PartInfo[i].region) < 0)
			|| (PartInfo[i].partition_idx >= (unsigned int)PART_MAX_COUNT)) {
			break;
		}

		if (STRICMP(PartInfo[i].name, check_strings[csBmtPool].str, csBmtPool) == 0) {
			i++;
			break;
		}
	}

	part_count = i;

	if (!part_count) {
		RS_LOG("gbdpe,6\n");
		ret = -EINVAL;
		goto out;
	}
#endif

	ret = -ENOENT;

	disk_part_iter_init(&piter, disk, 0);
	while ((part = disk_part_iter_next(&piter))) {
	#if defined(RS_HAS_EMMC_SHORTCUTS)
		for (i = 0; i < ptInvalid; i++) {
			for (j = 0; j < part_count; j++) {
				if ((PartInfo[j].partition_idx != 0) && (PartInfo[j].partition_idx == part->partno)) {
					bool ok; /* = false;*/
					const char *local_part_name = PartInfo[j].name;

					RS_LOG("gbdpe,7,%s,%d\n", local_part_name, (int)part->partno);

				#if defined(RS_AB_UPDATE_SUPPORT)
					{
						int part_name_len;
						bool is_ab_update = is_ab_part_name(local_part_name, &part_name_len);
						if (is_ab_update) {
							ok = (strncasecmp(local_part_name, part_name, part_name_len) == 0);
						} else {
							ok = (strcasecmp(local_part_name, part_name) == 0);
						}
					}
				#else
					ok = (strcasecmp(local_part_name, part_name) == 0);
				#endif

					if (ok) {
						/*refer to disk_name()*/
						char diskname_fmt[sizeof("%sp") + 1];
						size_t path_len, len;

						path_len = sizeof("/dev/block/") - 1;
						memcpy(buffer, check_strings[csMmcblk0p].str, path_len);
					#if defined(RS_ENCRYPT_STR)
						simple_str_encrypt(buffer, path_len, csMmcblk0p);
					#endif

						len = strlen(disk->disk_name);
						if (isdigit(disk->disk_name[len - 1])) {
							memcpy(diskname_fmt, check_strings[csDiskNameWDigitFmt].str, check_strings[csDiskNameWDigitFmt].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(diskname_fmt, check_strings[csDiskNameWDigitFmt].str_len, csDiskNameWDigitFmt);
						#endif
						} else {
							memcpy(diskname_fmt, check_strings[csDiskNameFmt].str, check_strings[csDiskNameFmt].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(diskname_fmt, check_strings[csDiskNameFmt].str_len, csDiskNameFmt);
						#endif
						}

						path_len += scnprintf(&buffer[path_len], (buffer_size - path_len), diskname_fmt, disk->disk_name);
						scnprintf(&buffer[path_len], (buffer_size - path_len), "%d", part->partno);

						RS_LOG("gbdpe,8,%s\n", buffer);

						ret = 0;
						goto out_of_loop;
					}

					i = ptInvalid; /*to goto next partition loop*/
					break;
				}
			}
		}
	#else
		struct partition_meta_info *info = part->info;

		if ((info) && (info->volname[0])) {
			const char *local_part_name = info->volname;

			RS_LOG("gbdpe,9,%s\n", local_part_name);

			for (i = 0; i < ptInvalid; i++) {
				bool ok; /* = false;*/

			#if defined(RS_AB_UPDATE_SUPPORT)
				{
					int part_name_len;
					bool is_ab_update = is_ab_part_name(local_part_name, &part_name_len);

					if (is_ab_update) {
						ok = (strncasecmp(local_part_name, part_name, part_name_len) == 0);
					} else {
						ok = (strcasecmp(local_part_name, part_name) == 0);
					}
				}
			#else
				ok = (strcasecmp(local_part_name, part_name) == 0);
			#endif

				if (ok) {
					/*refer to disk_name()*/
					char diskname_fmt[sizeof("%sp") + 1];
					size_t path_len, len;

					path_len = sizeof("/dev/block/") - 1;
					memcpy(buffer, check_strings[csMmcblk0p].str, path_len);
				#if defined(RS_ENCRYPT_STR)
					simple_str_encrypt(buffer, path_len, csMmcblk0p);
				#endif

					len = strlen(disk->disk_name);
					if (isdigit(disk->disk_name[len - 1])) {
						memcpy(diskname_fmt, check_strings[csDiskNameWDigitFmt].str, check_strings[csDiskNameWDigitFmt].str_len + 1);
					#if defined(RS_ENCRYPT_STR)
						simple_str_encrypt(diskname_fmt, check_strings[csDiskNameWDigitFmt].str_len, csDiskNameWDigitFmt);
					#endif
					} else {
						memcpy(diskname_fmt, check_strings[csDiskNameFmt].str, check_strings[csDiskNameFmt].str_len + 1);
					#if defined(RS_ENCRYPT_STR)
						simple_str_encrypt(diskname_fmt, check_strings[csDiskNameFmt].str_len, csDiskNameFmt);
					#endif
					}

					path_len += scnprintf(&buffer[path_len], (buffer_size - path_len), diskname_fmt, disk->disk_name);
					scnprintf(&buffer[path_len], (buffer_size - path_len), "%d", part->partno);

					RS_LOG("gbdpe,a,%s\n", buffer);

					ret = 0;
					goto out_of_loop;
				}
			}
		}
	#endif
	}

out_of_loop:
	disk_part_iter_exit(&piter);

#if defined(RS_DISK_OWNER_MODULE_PUT)
	fops = disk->fops;
	if (fops) {
		owner = (struct module *)(fops->owner);

		if (owner)
			module_put(owner);
	}
#endif

	put_disk(disk);

out:
	RS_LOG("gbdpe,e,%d\n", ret);
	return ret;
}

#if defined(RS_USE_UFS_DEV)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_part_names_ufs RS_HIDE(_la_23)
#endif
noused notrace static int rs_init_part_names_ufs(void)
{
/*#define RS_DISK_OWNER_MODULE_PUT*/

	/*refer to printk_all_partitions()*/
	noused int ret = 0;
	const struct device_type *disk_type_ptr;
	struct class_dev_iter iter;
	struct device *dev;
	noused struct disk_part_iter piter;
	noused struct hd_struct *part;
	noused struct gendisk *disk;
	dev_t ufs_dev;
	noused int partno, i, count;
	part_names_info *save_info;
#if defined(RS_DISK_OWNER_MODULE_PUT)
	const struct block_device_operations *fops;
	struct module *owner;
#endif
	char name_buf[BDEVNAME_SIZE];
	extern void module_put(struct module *module);

	/*RS_LOG("ipnu,0\n");*/

	if (RS_BUFFER_SIZE < (sizeof(*save_info) + sizeof(RS_PART_NO_STORE_TYPE))) {
		/*RS_LOG("ipnu,1\n");*/
		ret = -E2BIG;
		goto out_no_save_info;
	}

	save_info = (part_names_info *)RS_GET_BUFFER();
	if (!save_info) {
		/*RS_LOG("ipnu,2\n");*/
		ret = -ENOMEM;
		goto out;
	}

	memset(save_info, 0, sizeof(*save_info) + sizeof(RS_PART_NO_STORE_TYPE));

	save_info->is_ufs = g_no_emmc;

	ufs_dev = MKDEV(SCSI_DISK0_MAJOR, 0); /*get disk_type, which is static var in genhd.c*/

	/*partno = 0;*/
	disk = get_gendisk(ufs_dev, &partno);

	if (!disk) {
		/*RS_LOG("ipnu,3\n");*/
		ret = -ENXIO;
		goto out;
	}

	disk_type_ptr = disk_to_dev(disk)->type;

#if defined(RS_DISK_OWNER_MODULE_PUT)
	fops = disk->fops;
	if (fops) {
		owner = (struct module *)(fops->owner);

		if (owner)
			module_put(owner);
	}
#endif

	put_disk(disk);

	count = 0;

	class_dev_iter_init(&iter, &block_class, NULL, disk_type_ptr/*&disk_type*/);
	while ((dev = class_dev_iter_next(&iter))) {
		disk = dev_to_disk(dev);

		/*
		 * Don't show empty devices or things that have been
		 * suppressed
		 */
		if (get_capacity(disk) == 0
			|| (disk->flags & GENHD_FL_SUPPRESS_PARTITION_INFO)) {
			continue;
		}

		/*
		 * Note, unlike /proc/partitions, I am showing the
		 * numbers in hex - the same format as the root=
		 * option takes.
		 */
		disk_part_iter_init(&piter, disk, 0/*DISK_PITER_INCL_PART0*/);
		while ((part = disk_part_iter_next(&piter))) {
			/*bool is_part0 = (part == &disk->part0);*/

			/*if (!is_part0)*/
			{
				struct partition_meta_info *info;
				int part_major = MAJOR(part_devt(part));

				if ((part_major != SCSI_DISK0_MAJOR) && (part_major != BLOCK_EXT_MAJOR)) {
					continue;
				}

				info = part->info;

				if ((info) && (info->volname[0])) {
					const char *part_name = info->volname;

					/*RS_LOG("ipnu,4,%s\n", part_name);*/

					for (i = 0; i < ptInvalid; i++) {
						bool ok; /* = false;*/
						int idx, offset;

						idx = part_str_indexes[i].idx;
						offset = part_str_indexes[i].offset;

					#if defined(RS_AB_UPDATE_SUPPORT)
						{
							int part_name_len;
							bool is_ab_update = is_ab_part_name(part_name, &part_name_len);

							if (is_ab_update) {
								ok = (STRNICMP_EX(part_name, check_strings[idx].str, offset, part_name_len, idx) == 0);
							} else {
								ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
							}
						}
					#else
						ok = (STRICMP_EX(part_name, check_strings[idx].str, offset, idx) == 0);
					#endif

						if (ok) {

							if (!save_info->info.ufs.part_names[i][0]) {
								/*refer to disk_name()*/
								disk_name(disk, part->partno, name_buf);
								strlcpy(save_info->info.ufs.part_names[i], name_buf, sizeof(save_info->info.ufs.part_names[i]));
								save_info->info.ufs.part_names[i][sizeof(save_info->info.ufs.part_names[i]) - 1] = '\0';

								/*RS_LOG("ipnu,5,%s\n", save_info->info.ufs.part_names[i]);*/

								count++;
								if (count >= ptInvalid) {
									goto out_of_loop;
								}
							}

							break;
						}
					}
				}
			}
		}
		disk_part_iter_exit(&piter);
	}

out_of_loop:
	class_dev_iter_exit(&iter);

	if (count) {
		/*RS_LOG("ipnu,6,%d\n", ret);*/
		ret = rs_save_part_names_info(save_info);
	}

out:
	if (save_info) {
		RS_FREE_BUFFER(save_info);
	}

out_no_save_info:
	/*RS_LOG("ipnu,e,%d\n", ret);*/

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_blkdev_path_by_part_name_ufs RS_HIDE(_la_24)
#endif
noused notrace static int rs_get_blkdev_path_by_part_name_ufs(const char *part_name,
	char *buffer, size_t buffer_size)
{
/*#define RS_DISK_OWNER_MODULE_PUT*/

	/*refer to printk_all_partitions()*/
	noused int ret = 0;
	const struct device_type *disk_type_ptr;
	struct class_dev_iter iter;
	struct device *dev;
	noused struct disk_part_iter piter;
	noused struct hd_struct *part;
	noused struct gendisk *disk;
	dev_t ufs_dev;
	noused int partno;
#if defined(RS_DISK_OWNER_MODULE_PUT)
	const struct block_device_operations *fops;
	struct module *owner;
#endif
	char name_buf[BDEVNAME_SIZE];
	extern void module_put(struct module *module);

	RS_LOG("gbdpu,0\n");

	if ((!part_name) || (!part_name[0]) || (!buffer)
		|| (buffer_size < sizeof("/dev/block/mmcblk0p1"))) {
		RS_LOG("gbdpu,1\n");
		ret = -EINVAL;
		goto out;
	}

	ufs_dev = MKDEV(SCSI_DISK0_MAJOR, 0); /*get disk_type, which is static var in genhd.c*/

	/*partno = 0;*/
	disk = get_gendisk(ufs_dev, &partno);

	if (!disk) {
		RS_LOG("gbdpu,3\n");
		ret = -ENXIO;
		goto out;
	}

	disk_type_ptr = disk_to_dev(disk)->type;

#if defined(RS_DISK_OWNER_MODULE_PUT)
	fops = disk->fops;
	if (fops) {
		owner = (struct module *)(fops->owner);

		if (owner)
			module_put(owner);
	}
#endif

	put_disk(disk);

	ret = -ENOENT;

	class_dev_iter_init(&iter, &block_class, NULL, disk_type_ptr/*&disk_type*/);
	while ((dev = class_dev_iter_next(&iter))) {
		disk = dev_to_disk(dev);

		/*
		 * Don't show empty devices or things that have been
		 * suppressed
		 */
		if (get_capacity(disk) == 0
			|| (disk->flags & GENHD_FL_SUPPRESS_PARTITION_INFO)) {
			continue;
		}

		/*
		 * Note, unlike /proc/partitions, I am showing the
		 * numbers in hex - the same format as the root=
		 * option takes.
		 */
		disk_part_iter_init(&piter, disk, 0/*DISK_PITER_INCL_PART0*/);
		while ((part = disk_part_iter_next(&piter))) {
			/*bool is_part0 = (part == &disk->part0);*/

			/*if (!is_part0)*/
			{
				struct partition_meta_info *info;
				int part_major = MAJOR(part_devt(part));

				if ((part_major != SCSI_DISK0_MAJOR) && (part_major != BLOCK_EXT_MAJOR)) {
					continue;
				}

				info = part->info;

				if ((info) && (info->volname[0])) {
					const char *local_part_name = info->volname;

					RS_LOG("gbdpu,4,%s\n", local_part_name);

					{
						bool ok;

					#if defined(RS_AB_UPDATE_SUPPORT)
						{
							int part_name_len;
							bool is_ab_update = is_ab_part_name(local_part_name, &part_name_len);

							if (is_ab_update) {
								ok = (strncasecmp(local_part_name, part_name, part_name_len) == 0);
							} else {
								ok = (strcasecmp(local_part_name, part_name) == 0);
							}
						}
					#else
						ok = (strcasecmp(local_part_name, part_name) == 0);
					#endif

						if (ok) {
							size_t path_len;

							path_len = sizeof("/dev/block/") - 1;
							memcpy(buffer, check_strings[csMmcblk0p].str, path_len);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(buffer, path_len, csMmcblk0p);
						#endif

							/*refer to disk_name()*/
							disk_name(disk, part->partno, name_buf);
							strlcpy(&buffer[path_len], name_buf, (buffer_size - path_len));

							RS_LOG("gbdpu,5,%s\n", buffer);

							ret = 0;
							goto out_of_loop;
						}
					}
				}
			}
		}
		disk_part_iter_exit(&piter);
	}

out_of_loop:
	class_dev_iter_exit(&iter);

out:
	RS_LOG("gbdpu,e,%d\n", ret);
	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_blkdev_path_by_part_name RS_HIDE(_la_25)
#endif
noused notrace static int rs_get_blkdev_path_by_part_name(const char *part_name,
	char *buffer, size_t buffer_size)
{
#if defined(RS_USE_UFS_DEV)
	check_if_no_emmc(buffer);

	if (g_no_emmc > 0)
		return rs_get_blkdev_path_by_part_name_ufs(part_name, buffer, buffer_size);
	else
#endif
		return rs_get_blkdev_path_by_part_name_emmc(part_name, buffer, buffer_size);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_part_names RS_HIDE(_la_3)
#endif
noused notrace static int __init rs_init_part_names(void)
{
#if defined(RS_USE_UFS_DEV)
	check_if_no_emmc(NULL);

	if (g_no_emmc > 0)
		return rs_init_part_names_ufs();
	else
#endif
		return rs_init_part_names_emmc();
}

#if defined(CONFIG_MOUNT_RESTRICT)
late_initcall_sync(rs_init_part_names);
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_part_names_info RS_HIDE(_la_4)
#endif
noused notrace static const part_names_info *rs_get_part_names_info(void)
{
	/*return 0;*/
#if 1
	const part_names_info *ret = NULL;
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	unsigned var_flag_idx = opvChecksum;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift, offs;

	RS_VAR_READ_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	/*RS_LOG("gpn,-1,%x,%u,%d\n", rs_op_var_flag, var_flag_idx, op_type);*/

	if (!(rs_op_var_flag & (1 << var_flag_idx))) {
		/*RS_LOG("gpn,-2\n");*/
	#if !defined(RS_VARS_LOCK_USE_RWSEM)
		RS_VAR_READ_UNLOCK;
	#endif
		goto out;
	}

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	/*RS_LOG("gpn,0,%d,%d,%d\n", (int)var_flag_idx, (int)var_index, (int)value_shift);*/
	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

#if !defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	if ((page) && rs_is_valid_vars_page(page, false)) {
		uintptr_t checksum = 0;
		/*RS_LOG("gpn,1\n");*/
		checksum = (uintptr_t)crc32_le(0, (unsigned char const *)&vars[var_index], RS_OP_VAR_TUPLE_ELEMENTS_SIZE);
		{
			size_t tmp_offs = value_shift ^ var_index;
			if (tmp_offs < gfInvalid) {
				/*留位置给bad_boot等全局标志*/
				tmp_offs = gfInvalid;
			}

			/*RS_LOG("gpn,2,%d\n", (int)tmp_offs);*/

			offs = (1024 * opvInvalid) / sizeof(*page) + tmp_offs;

			if (checksum == page[offs]) {
				uintptr_t random_num = vars[var_index + ((value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT)];

				offs = ((random_num >> ((1 + var_flag_idx) * 8)) & 0xff) >> RS_OP_VARS_OFFS_RANDOM_SHIFT;

				offs = ((var_flag_idx * 1024) / sizeof(*page)) + offs;
				/*RS_LOG("gpn,3,%d\n", (int)offs);*/

				if (page[offs] == ((uintptr_t)page ^ offs ^ random_num)) {
					/*RS_LOG("gpn,4\n");*/
					ret = (const part_names_info *)(&page[offs + 1]);
				}
			}
		}
	}

out:
#if defined(RS_VARS_LOCK_USE_RWSEM)
	RS_VAR_READ_UNLOCK;
#endif

	return ret;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_do_mount_check_parts RS_HIDE(_la_5)
#endif
noused static const int rs_do_mount_check_parts[] = {
	ptSystem,
	ptApps,
#if defined(RS_HAS_OEM_PARTITION)
	ptOem,
#endif
#if defined(RS_HAS_VENDOR_PARTITION)
	ptVendor,
#endif
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_need_check_part_internal RS_HIDE(_la_6)
#endif
noused notrace static int rs_is_need_check_part_internal(const char *part_name, const int *parts,
	size_t parts_count, int *part_type_ptr)
{
	int ret = 0;
	const part_names_info *parts_info;
	const char *name_part;
	int i;

	if ((!part_name) || (!part_name[0]))
		goto out;

	parts_info = rs_get_part_names_info();
	if (parts_info) {
	#if defined(RS_USE_UFS_DEV)
		if (parts_info->is_ufs > 0) {
			/*if (parts_info)*/
			{
				int part_name_len;

				/*RS_LOG("ncp,0\n");*/

				if (part_name[0] == '/') {
					name_part = strrchr(part_name, '/');
					if (name_part)
						name_part++;
					else
						name_part = part_name;
				} else {
					name_part = part_name;
				}

				part_name_len = strlen(name_part);

				if ((parts) && (parts_count)) {
					/*RS_LOG("ncp,3\n");*/
					for (i = 0; i < parts_count; i++) {
						int pt_type = parts[i];

						if (pt_type < ptInvalid) {
							if ((parts_info->info.ufs.part_names[pt_type][0])
								&& (strncmp(name_part, parts_info->info.ufs.part_names[pt_type], part_name_len) == 0)) {
								/*RS_LOG("ncp,4,%d,%s\n", i, name_part);*/
								if (part_type_ptr) {
									*part_type_ptr = pt_type;
								}

								ret = 1;
								break;
							}
						}
					}
				} else {
					/*RS_LOG("ncp,5\n");*/
					for (i = 0; i < ptInvalid; i++) {
						if ((parts_info->info.ufs.part_names[i][0])
							&& (strncmp(name_part, parts_info->info.ufs.part_names[i], part_name_len) == 0)) {
							/*RS_LOG("ncp,6,%d,%s\n", i, name_part);*/
							if (part_type_ptr) {
								*part_type_ptr = i;
							}

							ret = 1;
							break;
						}
					}
				}
			}
		} else
	#endif
		{
			int disk_name_len, part_no_len;

			disk_name_len = parts_info->info.emmc.disk_name_len;
			if (/*(parts_info) &&*/ (disk_name_len)) {
				/*RS_LOG("ncp,8\n");*/

				if (part_name[0] == '/') {
					name_part = strrchr(part_name, '/');
					if (name_part)
						name_part++;
					else
						name_part = part_name;
				} else {
					name_part = part_name;
				}

				if (strncmp(name_part, parts_info->info.emmc.disk_name, disk_name_len) == 0) {
					/*RS_LOG("ncp,9\n");*/

					name_part += disk_name_len;
					part_no_len = strlen(name_part);
					if ((part_no_len) && (part_no_len <= sizeof(RS_PART_NO_STORE_TYPE))) {
						/*RS_LOG("ncp,a\n");*/
						if ((parts) && (parts_count)) {
							/*RS_LOG("ncp,b\n");*/
							for (i = 0; i < parts_count; i++) {
								int pt_type = parts[i];

								if (pt_type < ptInvalid) {
									if ((parts_info->info.emmc.part_names[pt_type][0])
										&& (strncmp(name_part, parts_info->info.emmc.part_names[pt_type],
										sizeof(RS_PART_NO_STORE_TYPE)) == 0)) {
										/*RS_LOG("ncp,c,%d,%s\n", i, name_part);*/
										if (part_type_ptr) {
											*part_type_ptr = pt_type;
										}

										ret = 1;
										break;
									}
								}
							}
						} else {
							/*RS_LOG("ncp,d\n");*/
							for (i = 0; i < ptInvalid; i++) {
								if ((parts_info->info.emmc.part_names[i][0])
									&& (strncmp(name_part, parts_info->info.emmc.part_names[i],
									sizeof(RS_PART_NO_STORE_TYPE)) == 0)) {
									/*RS_LOG("ncp,e,%d,%s\n", i, name_part);*/
									if (part_type_ptr) {
										*part_type_ptr = i;
									}

									ret = 1;
									break;
								}
							}
						}
					}
				}
			}
		}
	}
out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_need_check_part RS_HIDE(_la_7)
#endif
noused notrace static int rs_is_need_check_part(const char *part_name)
{
	return rs_is_need_check_part_internal(part_name, NULL, 0, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_is_mount_need_check_part RS_HIDE(_la_8)
#endif
noused notrace static int rs_is_mount_need_check_part(const char *part_name, int *part_type_ptr)
{
	if (STRNCMP(part_name, check_strings[csDevBlockSlash].str,
		check_strings[csDevBlockSlash].str_len, csDevBlockSlash) == 0) {
		const char *name_part = strrchr(part_name, '/');

		if (name_part) {
			name_part++;
		} else {
			name_part = part_name;
		}

		return rs_is_need_check_part_internal(name_part, rs_do_mount_check_parts,
			ARRAY_SIZE(rs_do_mount_check_parts), part_type_ptr);
	} else {
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
		if (STRCMP(part_name, check_strings[csDevRoot].str, csDevRoot) == 0) {
			if (part_type_ptr) {
				*part_type_ptr = ptSystem;
			}

			return 1;
		}
	#endif

		return 0;
	}
}

enum {
	rivsNotInitialized = 0,
	rivsLockedInitialized,
	rivsAllocPage,
	rivsAllocPageOK,
	rivsValidatePageOK,
	rivsSetChecksumOK,
	rivsSetChecksumFail,
	rivsValidatePageFail,
	rivsAllocPageFail,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_init_vars_stage RS_HIDE(_lh1)
#endif
noused static int g_rs_init_vars_stage = rivsNotInitialized;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_vars RS_HIDE(_lh)
#endif
noused notrace static int __init rs_init_vars(void)
{
	noused RS_CHECKSUM_TYPE v;
	uintptr_t *page;

	RS_LOG("riv\n");

	if (g_rs_init_vars_stage == rivsNotInitialized) {
		g_rs_init_vars_stage = rivsLockedInitialized;

		RS_VAR_LOCK_INIT;
	}

	page = (uintptr_t *)rs_malloc_vars_page();

	g_rs_init_vars_stage = rivsAllocPage;

	if (page) {
		RS_LOG("riv,1,%p\n", page);
		g_rs_init_vars_stage = rivsAllocPageOK;

		if (rs_is_valid_vars_page(page, true)) {
			RS_LOG("riv,2\n");
			g_rs_init_vars_stage = rivsValidatePageOK;

			if (!set_checksum_vars(0, &v, page)) {
				RS_LOG("riv,3\n");
				g_rs_init_vars_stage = rivsSetChecksumOK;

			#if defined(CONFIG_RS_CHECK_FUNC)
				rs_init_crc_table();

				rs_init_func_checksums();
			#endif
			} else {
				g_rs_init_vars_stage = rivsSetChecksumFail;

				rs_set_memory_rw((unsigned long)page, 1);
				rs_free_vars_page(page);
			}
		} else {
			g_rs_init_vars_stage = rivsValidatePageFail;

			rs_free_vars_page(page);
		}
	} else {
		g_rs_init_vars_stage = rivsAllocPageFail;
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ensure_rs_init_vars RS_HIDE(_lh2)
#endif
noused notrace static int ensure_rs_init_vars(void)
{
	if ((g_rs_init_vars_stage == rivsNotInitialized)
		|| (g_rs_init_vars_stage == rivsAllocPageFail)
		|| (g_rs_init_vars_stage == rivsValidatePageFail)
		) {
		RS_LOG("eriv,1\n");

		rs_init_vars();
		return 1;
	}

	return 0;
}

#if defined(CONFIG_ROOT_RESTRICT)
device_initcall_sync(rs_init_vars); /*module_init(rs_init_vars);*/
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_exit_vars RS_HIDE(_li)
#endif
noused notrace static void __exit rs_exit_vars(void)
{
	uintptr_t *vars = g_rs_op_vars;
	uintptr_t *page;
	RS_OP_VAR_FLAG_TYPE rs_op_var_flag;
	size_t var_index, value_shift;
	int is_text_rw_set = 0;

	if (!is_text_rw_set) {
		if (!rs_set_text_vars_rw(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
			is_text_rw_set = 1;
		}
	}

	RS_VAR_WRITE_LOCK;

#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_fetch_op_var_flag(&rs_op_var_flag);
#else
	rs_op_var_flag = g_rs_op_var_flag;
#endif

	if (!rs_op_var_flag) {
		goto out;
	}

	var_index = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4))) & 0xf) % RS_OP_VAR_TUPLE_STORE_SLOTS_COUNT;
	value_shift = ((rs_op_var_flag >> ((sizeof(RS_OP_VAR_FLAG_TYPE) * 8 - 4) - 4 * (1/* + var_flag_idx*/))) & 0xf) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT;

	page = (uintptr_t *)(vars[var_index + value_shift] ^ vars[var_index + (value_shift + 1) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]
		^ vars[var_index + (value_shift + 2) % RS_OP_VAR_TUPLE_ELEMENTS_COUNT]);

	if ((page) && rs_is_valid_vars_page(page, false)) {
		uintptr_t zero = 0;
		/*uintptr_t values[RS_OP_VAR_TUPLE_ELEMENTS_COUNT] = {0};*/

		rs_fill_text_vars(&vars[var_index], &zero, sizeof(vars[0]), RS_OP_VAR_TUPLE_ELEMENTS_COUNT);

		rs_set_memory_rw((unsigned long)page, 1);

		rs_free_vars_page(page);

	}

	rs_op_var_flag = 0;
#if defined(RS_HIDE_OP_VAR_FLAG)
	rs_store_op_var_flag(rs_op_var_flag);
#else
	g_rs_op_var_flag = rs_op_var_flag;
#endif

out:
	RS_VAR_WRITE_UNLOCK;

	if (is_text_rw_set) {
		if (!rs_set_text_vars_ro(vars, sizeof(vars[0]), RS_OP_VARS_STORE_COUNT)) {
			is_text_rw_set = 0;
		}
	}
}

#if defined(CONFIG_ROOT_RESTRICT)
module_exit(rs_exit_vars);
#endif

#endif

/*///////////////////*/


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define kern_abs_path RS_HIDE(_n0)
#endif
noused notrace static noinline int kern_abs_path(const char *name, unsigned int flags, struct path *path)
{
	/*struct path root = {};*/
	return vfs_path_lookup(/*root.dentry, root.mnt*/NULL, NULL, name, flags | LOOKUP_ROOT, path);
}

/*
dentry_path(), unlike d_path(), is relative to the root of file system containing dentry in question.
There are 3 of those suckers:
	d_path(): vfsmount/dentry => path to current process' root
	d_absolute_path(): ditto, but ignores chroot jails (goes to the absolute root of namespace)
	dentry_path(): dentry => path from root of fs containing that dentry.
	IOW, if you have something mounted on /jail/mnt/foo and are chrooted into/jail,
	passing vfsmount/dentry of /jail/mnt/foo/bar/baz to d_path() will yield "/mnt/foo/bar/baz",
	to d_absolute_path() - "/jail/mnt/foo/bar/baz" and passing the same dentry to dentry_path() - "/bar/baz".
*/

#if RS_HANDLE_PATH_NULL_RETURN

#define MAX_RES_OFFSET_VALUE (100)

#define RES_CHECK_FILL_PATTERN (0xdeadbeab)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_res_offset RS_HIDE(_n1)
#endif
noused static int g_res_offset;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_stack_ptr_offset RS_HIDE(_n2)
#endif
noused static int g_stack_ptr_offset;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_stack_dir RS_HIDE(_n3)
#endif
noused static int g_stack_dir;


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define find_stack_direction RS_HIDE(_na)
#endif
noused notrace static noinline void find_stack_direction(void)
{
	static char *addr;	/* Address of first `dummy', once known.  */
	auto char dummy;			/* To get stack address.  */

	if (g_stack_dir != 0) {
		return;
	}

	if (addr == NULL) {
		/* Initial entry.  */
		addr = &dummy;

		find_stack_direction();  /* Recurse once.  */
	} else {
		/* Second entry.  */
		if (&dummy > addr)
			g_stack_dir = 1;		/* Stack grew upward.  */
		else
			g_stack_dir = -1;		/* Stack grew downward.  */
	}
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define d_path_without_deleted RS_HIDE(_nb)
#endif
noused notrace char *noinline d_path_without_deleted(const struct path *path, char *buf, int buflen)
{
	struct dentry *dentry = path->dentry;
	const struct dentry_operations *d_op = dentry->d_op;

	#define STACK_PTR_ANCHOR (&d_op) /*__builtin_frame_address(0)*/

	if ((d_op) && (d_op->d_dname) && (!IS_ROOT(dentry) || dentry != path->mnt->mnt_root)) {
		return d_op->d_dname(dentry, buf, buflen);
	} else {
		/*extern char *dentry_path_raw(struct dentry *dentry, char *buf, int buflen);*/ /*只到本文件系统根部，不能越过挂载点*/
		extern char *__d_path(const struct path *path, const struct path *root, char *buf, int buflen);
		/*extern char *__d_path_(const struct path *path, const struct path *root, char *buf, int buflen);*/
		extern char *d_absolute_path(const struct path *path, char *buf, int buflen);
		char *res;
	#if RS_HANDLE_PATH_NULL_RETURN
		noused char *bufend;
		noused char **stack_ptr, **stack_start;
		noused int check_count, i, offs;
	#endif

	#if RS_USE_D_ABS_PATH
		#define D_X_PATH(path, root, buf, buflen) d_absolute_path(path, buf, buflen)
	#else
		noused struct path root;

		#define D_X_PATH(path, root, buf, buflen) __d_path(path, &root, buf, buflen)
	#endif


	#if (!RS_HANDLE_PATH_NULL_RETURN) || ((RS_HANDLE_PATH_NULL_RETURN) && (RS_HANDLE_PATH_NULL_RETURN_AFTERWARD))
	#if RS_USE_D_ABS_PATH
		res = D_X_PATH(path, root, buf, buflen);
	#else

		get_fs_root(current->fs, &root);

		res = D_X_PATH(path, root, buf, buflen);
	#endif
	#endif

	#if ((RS_HANDLE_PATH_NULL_RETURN) && (RS_HANDLE_PATH_NULL_RETURN_AFTERWARD))
		if (!res) {
			if (g_res_offset == 0) {
				stack_ptr = (char **)({ char *__csf; &__csf; });

				stack_start = (char **)current_thread_info();
				g_stack_dir = ((stack_ptr > stack_start) ? -1 : 1);
				/*(((char *)&d_op < (char *)__builtin_frame_address(0)) ? -1 : 1);*/ /*会多保存一个FP寄存器*/
				/*find_stack_direction();*/ /*会破坏前一个调用的栈上信息*/

				if (g_stack_dir < 0) {
					stack_start = (char **)(ALIGN((size_t)((char *)stack_start + sizeof(struct thread_info)), sizeof(char **))) + 1;

					if ((stack_ptr - stack_start) > MAX_RES_OFFSET_VALUE) {
						check_count = MAX_RES_OFFSET_VALUE;
					} else {
						check_count = stack_ptr - stack_start;
					}

					bufend = buf + buflen - 1;
					offs = 0;
					for (i = 0; i < check_count; i++) {
						res = stack_ptr[-i];
						if ((res >= buf) && (res <= bufend) && (*res == '/')) {
							while ((++res < bufend) && (*res)) {
							};
							if (res >= bufend) {
								/*buf++;*/
								/*buf_len--;*/
								offs = -i;
								break;
							}
						}
					}

					if (!offs) {
						g_res_offset = 1;
					} else {
						g_stack_ptr_offset = stack_ptr - (char **)STACK_PTR_ANCHOR;
						g_res_offset = offs;
						res = stack_ptr[offs];
						/*RS_LOG("null1: %d,%s\n", offs, res);*/
					}
				} else {
					stack_start--;

					if ((stack_start - stack_ptr) > MAX_RES_OFFSET_VALUE) {
						check_count = MAX_RES_OFFSET_VALUE;
					} else {
						check_count = stack_start - stack_ptr;
					}

					bufend = buf + buflen - 1;
					offs = 0;
					for (i = 0; i < check_count; i++) {
						res = stack_ptr[i];
						if ((res >= buf) && (res <= bufend) && (*res == '/')) {
							while ((++res < bufend) && (*res)) {
							};
							if (res >= bufend) {
								/*buf++;*/
								/*buf_len--;*/
								offs = i;
								break;
							}
						}
					}

					if (!offs) {
						g_res_offset = -1;
					} else {
						g_stack_ptr_offset = stack_ptr - (char **)STACK_PTR_ANCHOR;
						g_res_offset = offs;
						res = stack_ptr[offs];
						/*RS_LOG("null2: %d,%s\n", offs, res);*/
					}
				}
			} else if ((g_stack_dir < 0) && (g_res_offset < 0))
				|| (((g_stack_dir > 0) && (g_res_offset > 0))) {
				stack_ptr = (char **)STACK_PTR_ANCHOR + g_stack_ptr_offset;
				res = stack_ptr[g_res_offset];

				/*RS_LOG("null3: %s\n", res);*/
			}
		}
	#elif (RS_HANDLE_PATH_NULL_RETURN)

	#if !RS_USE_D_ABS_PATH
		get_fs_root(current->fs, &root);
	#endif

		if (g_res_offset) {
			res = D_X_PATH(path, root, buf, buflen);

			if (!res) {
				if (((g_stack_dir < 0) && (g_res_offset < 0))
					|| ((g_stack_dir > 0) && (g_res_offset > 0))) {
					stack_ptr = (char **)STACK_PTR_ANCHOR + g_stack_ptr_offset;
					res = stack_ptr[g_res_offset];

					/*RS_LOG("null1: %s\n", res);*/
				} else {
					/*RS_LOG("null0: %d\n", g_res_offset);*/
				}
			}
		} else {
			stack_ptr = (char **)({ char *__csf; &__csf; });

			stack_start = (char **)current_thread_info();
			g_stack_dir = ((stack_ptr > stack_start) ? -1 : 1);

			if (g_stack_dir < 0) {
				stack_start = (char **)(ALIGN((size_t)((char *)stack_start + sizeof(struct thread_info)), sizeof(char **))) + 1;

				if ((stack_ptr - stack_start) > MAX_RES_OFFSET_VALUE) {
					check_count = MAX_RES_OFFSET_VALUE;
				} else {
					check_count = stack_ptr - stack_start;
				}

				for (i = 0; i < check_count; i++) {
					stack_ptr[-i] = (char *)RES_CHECK_FILL_PATTERN;
				}

				memset(buf, 0, buflen);

				res = D_X_PATH(path, root, buf, buflen);

				if (!IS_ERR(res)) {
					if (!res) {
						res = buf + buflen - 1;
						while ((res >= buf) && (*--res)) {
						};

						res++;
					}

					for (i = check_count - 1; i >= 0; i--) {
						if (stack_ptr[-i] != (char *)RES_CHECK_FILL_PATTERN)
							break;
					}

					if (i >= 0) {
						check_count = i + 1;
						offs = 0;
						for (i = 0; i < check_count; i++) {
							if (res == stack_ptr[-i]) {
								offs = -i;
								break;
							}
						}

						if (!offs) {
							g_res_offset = 1;
						} else {
							g_stack_ptr_offset = stack_ptr - (char **)STACK_PTR_ANCHOR;
							g_res_offset = offs;
							/*RS_LOG("null1.3: %d,%s\n", offs, res);*/
						}
					}
				}
			} else {
				stack_start--;

				if ((stack_start - stack_ptr) > MAX_RES_OFFSET_VALUE) {
					check_count = MAX_RES_OFFSET_VALUE;
				} else {
					check_count = stack_start - stack_ptr;
				}

				for (i = 0; i < check_count; i++) {
					stack_ptr[i] = (char *)RES_CHECK_FILL_PATTERN;
				}

				memset(buf, 0, buflen);

				res = D_X_PATH(path, root, buf, buflen);

				if (!IS_ERR(res)) {
					if (!res) {
						res = buf + buflen - 1;
						while ((res >= buf) && (*--res)) {
						};

						res++;
					}

					for (i = check_count - 1; i >= 0; i--) {
						if (stack_ptr[i] != (char *)RES_CHECK_FILL_PATTERN)
							break;
					}

					if (i >= 0) {
						check_count = i + 1;
						offs = 0;
						for (i = 0; i < check_count; i++) {
							if (res == stack_ptr[i]) {
								offs = i;
								break;
							}
						}

						if (!offs) {
							g_res_offset = -1;
						} else {
							g_stack_ptr_offset = stack_ptr - (char **)STACK_PTR_ANCHOR;
							g_res_offset = offs;
							/*RS_LOG("null2: %d,%s\n", offs, res);*/
						}
					}
				}
			}

		}
	#endif

	#if !RS_USE_D_ABS_PATH
		path_put(&root);
	#endif

		return res;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define d_path_without_deleted_ex RS_HIDE(_nc)
#endif
noused notrace static char *d_path_without_deleted_ex(const struct path *path, char *buf, int buflen)
{
	char *ret_ptr = d_path_without_deleted(path, buf, buflen);

	if (IS_ERR(ret_ptr)) {
		ret_ptr = NULL;
	}

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define d_path_ex RS_HIDE(_nd)
#endif
noused notrace static char *d_path_ex(const struct path *path, char *buf, int buflen)
{
	char *ret_ptr = d_path(path, buf, buflen);

	if (IS_ERR(ret_ptr)) {
		ret_ptr = NULL;
	}

	return ret_ptr;
}

#undef D_PATH

#if RS_USE_ABSOLUTE_PATH
	#define D_PATH d_path_without_deleted
	#define D_PATH_EX d_path_without_deleted_ex
#else
	#define D_PATH d_path
	#define D_PATH_EX d_path_ex
#endif


#undef GET_TASK_MM
#undef PUT_TASK_MM

#if RS_SAFE_GET_TASK_MM
	#define GET_TASK_MM(tsk)	get_task_mm(tsk)
	#define PUT_TASK_MM(mm)	do { \
								if (mm) \
									mmput(mm); \
							} while (0)
#else
	#define GET_TASK_MM(tsk)	(tsk->mm)
	#define PUT_TASK_MM(mm)	(mm)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_absolute_path RS_HIDE(_ne)
#endif
noused notrace static noinline char *get_absolute_path(struct task_struct *task, char **buffer_ptr)
{
	char *buffer;
	char *ret_ptr = NULL;
	/*struct vm_area_struct *vma = NULL;*/
	/*struct path base_path;*/
	noused struct mm_struct *mm;
	struct file *exe_file;
	/*char *path;*/
	extern void fput(struct file *);

	if (buffer_ptr == NULL) {
		return NULL;
	}

	if (NULL == task) {
		*buffer_ptr = NULL;
		return NULL;
	}

	buffer = (char *)RS_GET_BUFFER();
	if (!buffer) {
		*buffer_ptr = NULL;
		return NULL;
	}

	get_task_struct(task); /*task_lock(task);*/

	exe_file = (mm = GET_TASK_MM(task)) ? get_mm_exe_file(mm) : NULL;
	if (exe_file) {
		/*buffer[RS_BUFFER_SIZE - 1] = '\0';*/

		ret_ptr = D_PATH_EX(&exe_file->f_path, buffer, RS_BUFFER_SIZE);

		fput(exe_file);
	}

	PUT_TASK_MM(mm);

	put_task_struct(task); /*task_unlock(task);*/

	if (!ret_ptr) {
		RS_FREE_BUFFER(buffer);
		buffer = NULL;
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_absolute_path_ex RS_HIDE(_nf)
#endif
noused notrace static noinline char *get_absolute_path_ex(struct task_struct *task, char **buffer_ptr,
	umode_t *mode_ptr, unsigned int *flags_ptr)
{
	char *buffer;
	char *ret_ptr = NULL;
	/*struct vm_area_struct *vma = NULL;*/
	/*struct path base_path;*/
	noused struct mm_struct *mm;
	struct file *exe_file;
	umode_t mode;
	unsigned int flags;
	/*char *path;*/
	extern void fput(struct file *);

	if (buffer_ptr == NULL) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	if (NULL == task) {
		*buffer_ptr = NULL;
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;

		return NULL;
	}

	buffer = (char *)RS_GET_BUFFER();
	if (!buffer) {
		*buffer_ptr = NULL;
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	mode = 0;
	flags = 0;
	get_task_struct(task); /*task_lock(task);*/

	exe_file = (mm = GET_TASK_MM(task)) ? get_mm_exe_file(mm) : NULL;
	if (exe_file) {
		struct dentry *dentry = exe_file->f_dentry;
		/*buffer[RS_BUFFER_SIZE - 1] = '\0';*/

		if (dentry) {
			ret_ptr = D_PATH_EX(&exe_file->f_path, buffer, RS_BUFFER_SIZE);

			if (ret_ptr) {
				struct inode *exe_inode;

				exe_inode = dentry->d_inode;
				if (exe_inode) {
					mode = exe_inode->i_mode;
					flags = exe_inode->i_flags;
				}
			}
		}

		fput(exe_file);
	}

	PUT_TASK_MM(mm);

	put_task_struct(task); /*task_unlock(task);*/

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	if (!ret_ptr) {
		RS_FREE_BUFFER(buffer);
		buffer = NULL;
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define free_path RS_HIDE(_ng)
#endif
noused notrace static noinline void free_path(const char *buffer)
{
	if (buffer) {
		RS_FREE_BUFFER(buffer);
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_absolute_path_with_buffer RS_HIDE(_nh)
#endif
noused notrace static noinline char *get_absolute_path_with_buffer(struct task_struct *task, char *buffer, int buffer_size)
{
	char *ret_ptr = NULL;
	/*struct vm_area_struct *vma = NULL;*/
	/*struct path base_path;*/
	noused struct mm_struct *mm;
	struct file *exe_file;
	/*char *path;*/
	extern void fput(struct file *);

	if ((NULL == task) || (buffer == NULL) || (buffer_size <= 0)) {
		return NULL;
	}

	get_task_struct(task); /*task_lock(task);*/

	exe_file = (mm = GET_TASK_MM(task)) ? get_mm_exe_file(mm) : NULL;
	if (exe_file) {
		/*buffer[buffer_size - 1] = '\0';*/

		ret_ptr = D_PATH_EX(&exe_file->f_path, buffer, buffer_size);

		fput(exe_file);
	}

	PUT_TASK_MM(mm);

	put_task_struct(task); /*task_unlock(task);*/

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_absolute_path_ex_with_buffer RS_HIDE(_ni)
#endif
noused notrace static noinline char *get_absolute_path_ex_with_buffer(struct task_struct *task, char *buffer, int buffer_size,
	umode_t *mode_ptr, unsigned int *flags_ptr)
{
	char *ret_ptr = NULL;
	/*struct vm_area_struct *vma = NULL;*/
	/*struct path base_path;*/
	noused struct mm_struct *mm;
	struct file *exe_file;
	/*char *path;*/
	umode_t mode;
	unsigned int flags;
	extern void fput(struct file *);

	if ((NULL == task) || (buffer == NULL) || (buffer_size <= 0)) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	mode = 0;
	flags = 0;
	get_task_struct(task); /*task_lock(task);*/

	exe_file = (mm = GET_TASK_MM(task)) ? get_mm_exe_file(mm) : NULL;
	if (exe_file) {
		struct dentry *dentry = exe_file->f_dentry;
		/*buffer[buffer_size - 1] = '\0';*/

		if (dentry) {
			ret_ptr = D_PATH_EX(&exe_file->f_path, buffer, buffer_size);

			if (ret_ptr) {
				struct inode *exe_inode;

				exe_inode = dentry->d_inode;
				if (exe_inode) {
					mode = exe_inode->i_mode;
					flags = exe_inode->i_flags;
				}
			}
		}

		fput(exe_file);
	}

	PUT_TASK_MM(mm);

	put_task_struct(task); /*task_unlock(task);*/

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_real_filename RS_HIDE(_nj)
#endif
noused notrace static char *get_real_filename(const char *filename, char **buffer_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		buffer = (char *)RS_GET_BUFFER();
		if (buffer) {
			ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

			if (!ret_ptr) {
				RS_FREE_BUFFER(buffer);
				buffer = NULL;
			}
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_real_filename_with_path RS_HIDE(_nk)
#endif
noused notrace static char *get_real_filename_with_path(const char *filename, struct path *file_path,
	char **buffer_ptr)
{
	int error;
	char *buffer;
	char *ret_ptr;

	if ((!filename) || (!buffer_ptr) || (!filename[0]) || (!file_path)) {
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, file_path);

	if (!error) {
		buffer = (char *)RS_GET_BUFFER();
		if (buffer) {
			ret_ptr = D_PATH_EX(file_path, buffer, RS_BUFFER_SIZE);

			if (!ret_ptr) {
				RS_FREE_BUFFER(buffer);
				buffer = NULL;
			}
		}

		path_put(file_path);
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_filename_with_path RS_HIDE(_nl)
#endif
noused notrace static char *get_filename_with_path(struct path *file_path, char **buffer_ptr)
{
	char *buffer;
	char *ret_ptr;

	if ((!buffer_ptr) || (!file_path)) {
		return NULL;
	}

	ret_ptr = NULL;

	buffer = (char *)RS_GET_BUFFER();
	if (buffer) {
		ret_ptr = D_PATH_EX(file_path, buffer, RS_BUFFER_SIZE);

		if (!ret_ptr) {
			RS_FREE_BUFFER(buffer);
			buffer = NULL;
		}
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_real_filename_with_buffer RS_HIDE(_nm)
#endif
noused notrace static char *get_real_filename_with_buffer(const char *filename, char *buffer, int buffer_size)
{
	struct path file_path;
	int error;
	char *ret_ptr;

	if ((!filename) || (!filename[0]) || (!buffer) || (buffer_size <= 0)) {
		return NULL;
	}

	ret_ptr = NULL;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		ret_ptr = D_PATH_EX(&file_path, buffer, buffer_size);

		path_put(&file_path);
	}

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_real_filename_ex RS_HIDE(_nn)
#endif
noused notrace static char *get_real_filename_ex(const char *filename, char **buffer_ptr,
	umode_t *mode_ptr, unsigned int *flags_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;
	umode_t mode;
	unsigned int flags;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;
	mode = 0;
	flags = 0;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		buffer = (char *)RS_GET_BUFFER();
		if (buffer) {
			ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

			if (ret_ptr) {
				struct inode *file_inode;

				file_inode = file_path.dentry->d_inode;
				if (file_inode) {
					mode = file_inode->i_mode;
					flags = file_inode->i_flags;
				}
			}

			if (!ret_ptr) {
				RS_FREE_BUFFER(buffer);
				buffer = NULL;
			}
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_real_filename_ex_with_buffer RS_HIDE(_no)
#endif
noused notrace static char *get_real_filename_ex_with_buffer(const char *filename, char *buffer,
	int buffer_size, umode_t *mode_ptr, unsigned int *flags_ptr)
{
	struct path file_path;
	int error;
	char *ret_ptr;
	umode_t mode;
	unsigned int flags;

	if ((!filename) || (!filename[0]) || (!buffer) || (buffer_size <= 0)) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	ret_ptr = NULL;
	mode = 0;
	flags = 0;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		ret_ptr = D_PATH_EX(&file_path, buffer, buffer_size);

		if (ret_ptr) {
			struct inode *file_inode;

			file_inode = file_path.dentry->d_inode;
			if (file_inode) {
				mode = file_inode->i_mode;
				flags = file_inode->i_flags;
			}
		}

		path_put(&file_path);
	}

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_filename_info RS_HIDE(_np)
#endif
noused notrace static int get_filename_info(const char *filename, umode_t *mode_ptr, uid_t *owner_ptr,
	gid_t *group_ptr, unsigned int *flags_ptr)
{
	struct path file_path;
	int error;
	umode_t mode;
	uid_t owner;
	gid_t group;
	unsigned int flags;

	if ((!filename) || (!filename[0])) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (owner_ptr)
			*owner_ptr = 0;
		if (group_ptr)
			*group_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return 1;
	}

	mode = 0;
	owner = 0;
	group = 0;
	flags = 0;

	error = kern_path(filename, 0, &file_path);

	if (!error) {
		struct inode *file_inode;

		file_inode = file_path.dentry->d_inode;
		if (file_inode) {
			mode = file_inode->i_mode;
			owner = __kuid_val(file_inode->i_uid);
			group = __kgid_val(file_inode->i_gid);
			flags = file_inode->i_flags;
		}

		path_put(&file_path);
	}

	if (mode_ptr)
		*mode_ptr = mode;
	if (owner_ptr)
		*owner_ptr = owner;
	if (group_ptr)
		*group_ptr = group;
	if (flags_ptr)
		*flags_ptr = flags;

	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_get_real_filename RS_HIDE(_nq)
#endif
noused notrace static char *try_get_real_filename(const char *filename, char **buffer_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		if ((filename[0] != '/') ||
		#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
			(current->total_link_count)
		#else
			(d_is_symlink(file_path.dentry))
		#endif
			) {
			/*relative path or has symbol link*/
			/*refer to follow_link/follow_automount*/
			buffer = (char *)RS_GET_BUFFER();
			if (buffer) {
				ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

				if (!ret_ptr) {
					RS_FREE_BUFFER(buffer);
					buffer = NULL;
				}
			}
		} else {
			ret_ptr = (char *)filename;
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_get_real_filename_with_buffer RS_HIDE(_nr)
#endif
noused notrace static char *try_get_real_filename_with_buffer(const char *filename, char *buffer,
	int buffer_size)
{
	struct path file_path;
	int error;
	char *ret_ptr;

	if ((!filename) || (!filename[0]) || (!buffer) || (buffer_size <= 0)) {
		return NULL;
	}

	ret_ptr = NULL;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		if ((filename[0] != '/') ||
		#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
			(current->total_link_count)
		#else
			(d_is_symlink(file_path.dentry))
		#endif
			) {
			/*relative path or has symbol link*/
			/*refer to follow_link/follow_automount*/
			ret_ptr = D_PATH_EX(&file_path, buffer, buffer_size);
		} else {
			ret_ptr = (char *)filename;
		}

		path_put(&file_path);
	}

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_get_real_filename_ex RS_HIDE(_ns)
#endif
noused notrace static char *try_get_real_filename_ex(const char *filename, char **buffer_ptr,
	umode_t *mode_ptr, unsigned int *flags_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;
	umode_t mode;
	unsigned int flags;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;
	mode = 0;
	flags = 0;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		struct inode *file_inode = file_path.dentry->d_inode;

		if (file_inode) {
			mode = file_inode->i_mode;
			flags = file_inode->i_flags;
		}

		if ((filename[0] != '/') ||
		#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
			(current->total_link_count)
		#else
			(d_is_symlink(file_path.dentry))
		#endif
			) {
			/*relative path or has symbol link*/
			/*refer to follow_link/follow_automount*/
			buffer = (char *)RS_GET_BUFFER();
			if (buffer) {
				ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

				if (!ret_ptr) {
					RS_FREE_BUFFER(buffer);
					buffer = NULL;
				}
			}
		} else {
			ret_ptr = (char *)filename;
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_get_real_filename_no_follow_link RS_HIDE(_nt)
#endif
noused notrace static char *try_get_real_filename_no_follow_link(const char *filename, char **buffer_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;

	error = kern_path(filename, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		if ((filename[0] != '/')/* || (current->total_link_count)*/) {
			/*relative path or has symbol link*/
			/*refer to follow_link/follow_automount*/
			buffer = (char *)RS_GET_BUFFER();
			if (buffer) {
				ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

				if (!ret_ptr) {
					RS_FREE_BUFFER(buffer);
					buffer = NULL;
				}
			}
		} else {
			ret_ptr = (char *)filename;
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_real_dir_name RS_HIDE(_nu)
#endif
noused notrace static char *rs_get_real_dir_name(struct path *path, char **buffer_ptr)
{
	char *buffer = (char *)RS_GET_BUFFER();
	if (buffer) {
		char *ret_ptr = D_PATH_EX(path, buffer, RS_BUFFER_SIZE);
		if (!ret_ptr) {
			RS_FREE_BUFFER(buffer);
		} else {
			*buffer_ptr = buffer;
		}

		return ret_ptr;
	} else {
		return NULL;
	}
}

#define CHECK_TASK_NODE_LOCAL_TASK_COUNT (5)

struct check_task_node {
	struct check_task_node *next;
	int max_task_count;
	int task_count;
	struct task_struct *tasks[CHECK_TASK_NODE_LOCAL_TASK_COUNT];
};

#define PROCEED_TO_PARENT(node, task_index) \
	({ \
		int ret = 1; \
		task_index++; \
		if (task_index >= node->task_count) { \
			node = node->next; \
			if (node)  { \
				task_index = 0; \
			} else { \
				ret = 0; \
			} \
		} \
		ret; \
	})

#define GOTO_TO_TASK(head, node, task_index) \
	({ \
		struct task_struct *task = NULL; \
		int index = 0; \
		node = head; \
		while (node) { \
			int tmp_index = index + node->task_count; \
			if (tmp_index > task_index) { \
				task = node->tasks[task_index - index]; \
				break; \
			} \
			index = tmp_index; \
			node = node->next; \
		} \
		task; \
	})

#define IS_LAST_PARENT(node, task_index) \
	({ \
		int ret = ((!node->next) && (task_index == (node->task_count - 1)) ? 1 : 0); \
		ret; \
	})

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define free_upper_tasks RS_HIDE(_o0)
#endif
noused notrace static noinline void free_upper_tasks(struct check_task_node *head_ptr)
{
	struct check_task_node *node_ptr;
	int i;

	while (head_ptr) {
		node_ptr = head_ptr->next;

		/*RS_LOG("free upper:%d\n", head_ptr->task_count);*/
		for (i = 0; i < head_ptr->task_count; i++) {
			/*RS_LOG("free:%p,%s\n", head_ptr->tasks[i], head_ptr->tasks[i]->comm);*/

			put_task_struct(head_ptr->tasks[i]);
		}
		RS_FREE_BUFFER(head_ptr);
		head_ptr = node_ptr;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_upper_tasks RS_HIDE(_o1)
#endif
noused notrace static noinline struct check_task_node *get_upper_tasks(struct task_struct *task)
{
	struct check_task_node *node_ptr;
	struct check_task_node *head_ptr = NULL;
	int bad;

	if (is_init_task(task) || !(task->mm)) {
		/*RS_LOG("upper,0\n");*/
		return head_ptr;
	}

	head_ptr = (struct check_task_node *)RS_GET_BUFFER();
	if (!head_ptr) {
		/*RS_LOG("upper,1\n");*/
		return head_ptr;
	}

	node_ptr = head_ptr;

	bad = 0;
	get_task_struct(task);
	head_ptr->next = NULL;
	head_ptr->task_count = 1;
	head_ptr->tasks[0] = task;

	/*RS_LOG("get:%p,%s\n", task, task->comm);*/

	/*rcu_read_lock();*/
	/*read_lock(&tasklist_lock);*/

	{
		int got_init = 0;
		struct task_struct *parent;

		do {
			RS_READ_TASK_LOCK();

			parent = task->parent;
			if ((parent) && (parent == task->real_parent)) {
				if (is_init_task(parent)) {
					parent = NULL;

					got_init = 1;
				} else if (parent->mm) {
					get_task_struct(parent);
				} else {
					parent = NULL;
				}
			} else {
				parent = NULL;
			}

			RS_READ_TASK_UNLOCK();

			if (got_init) {
				break;
			} else if (!parent) {
				bad = 1;
				break;
			} else {
				int count = node_ptr->task_count;

				/*get_task_struct(parent);*/
				/*RS_LOG("get:%p,%s\n", parent, parent->comm);*/

				if (count == (node_ptr->max_task_count - 1)) {
					struct check_task_node *next_node_ptr = (struct check_task_node *)RS_GET_BUFFER();

					if (!next_node_ptr) {
						/*free all nodes*/
						RS_LOG("bad 1\n");

						/*put_task_struct(parent);*/
						bad = 2;
						break;
					}

					next_node_ptr->next = NULL;
					next_node_ptr->max_task_count = ((RS_BUFFER_SIZE - sizeof(*next_node_ptr)) / sizeof(*next_node_ptr))
														+ ARRAY_SIZE(node_ptr->tasks);
					node_ptr->next = next_node_ptr;
					node_ptr = next_node_ptr;

					count = 0;
				}

				node_ptr->tasks[count] = parent;
				node_ptr->task_count = count + 1;

				task = parent;
			}
			/*
			else {
				bad = 4;

				break;
			}
			*/
		} while (1);

		if (parent) {
			put_task_struct(parent);
		}
	}

	/*rcu_read_unlock();*/

	if (bad) {
		RS_LOG("upper,bad %d\n", bad);

		free_upper_tasks(head_ptr);
		head_ptr = NULL;
	}

	return head_ptr;
}

typedef struct tag_op_check_params {
	char *path;
	struct task_struct *current_task;
	char *buffer;
	int buffer_size;
	int op_type;
	ssize_t/*int*/ op_data;
	int is_query;
	struct check_task_node *head_ptr;
	struct check_task_node node;
} op_check_params;


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define free_op_check_params_upper_tasks RS_HIDE(_o2)
#endif
noused notrace
static noinline void free_op_check_params_upper_tasks(struct tag_op_check_params *params)
{
	struct check_task_node *head_ptr, *local_node_ptr;
	struct check_task_node *node_ptr;
	int i;

	if (!params) {
		return;
	}

	local_node_ptr = &params->node;
	head_ptr = params->head_ptr;

	while (head_ptr) {
		node_ptr = head_ptr->next;

		/*RS_LOG("free upper:%d\n", head_ptr->task_count);*/
		for (i = 0; i < head_ptr->task_count; i++) {
			/*RS_LOG("free:%p,%s\n", head_ptr->tasks[i], head_ptr->tasks[i]->comm);*/

			put_task_struct(head_ptr->tasks[i]);
		}

		if (head_ptr != local_node_ptr) {
			RS_FREE_BUFFER(head_ptr);
		} else {
			head_ptr->task_count = 0;
		}

		head_ptr = node_ptr;
	}

	params->head_ptr = NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_op_check_params_upper_tasks RS_HIDE(_o3)
#endif
noused notrace
static noinline int get_op_check_params_upper_tasks(struct tag_op_check_params *params,
	struct task_struct *task)
{
	struct check_task_node *node_ptr;
	/*struct check_task_node *head_ptr = NULL;*/
	int ret;

	if (!params) {
		return -EINVAL;
	}

	if (params->head_ptr) {
		free_op_check_params_upper_tasks(params);
	}

	if (is_init_task(task) || !(task->mm)) {
		/*RS_LOG("upper,0\n");*/
		return 0;
	}

	/*
	head_ptr = (struct check_task_node *)RS_GET_BUFFER();
	if (!head_ptr) {
		RS_LOG("upper,1\n");
		return head_ptr;
	}
	*/

	/*head_ptr =*/
	node_ptr = &params->node;
	params->head_ptr = node_ptr;

	ret = 0;
	get_task_struct(task);
	node_ptr->next = NULL;
	node_ptr->max_task_count = ARRAY_SIZE(node_ptr->tasks);
	node_ptr->task_count = 1;
	node_ptr->tasks[0] = task;

	/*RS_LOG("get:%p,%s\n", task, task->comm);*/

	/*rcu_read_lock();*/
	/*read_lock(&tasklist_lock);*/

	{
		int got_init = 0;
		struct task_struct *parent;

		do {
			RS_READ_TASK_LOCK();

			parent = task->parent;
			if ((parent) && (parent == task->real_parent)) {
				if (is_init_task(parent)) {
					parent = NULL;

					got_init = 1;
				} else if (parent->mm) {
					get_task_struct(parent);
				} else {
					parent = NULL;
				}
			} else {
				parent = NULL;
			}

			RS_READ_TASK_UNLOCK();

			if (got_init) {
				break;
			} else if (!parent) {
				ret = 1;
				break;
			} else {
				int count = node_ptr->task_count;

				/*get_task_struct(parent);*/
				/*RS_LOG("get:%p,%s\n", parent, parent->comm);*/

				if (count == (node_ptr->max_task_count - 1)) {
					struct check_task_node *next_node_ptr = (struct check_task_node *)RS_GET_BUFFER();

					if (!next_node_ptr) {
						/*free all nodes*/
						RS_LOG("bad 1\n");

						/*put_task_struct(parent);*/
						ret = 2;
						break;
					}

					next_node_ptr->next = NULL;
					next_node_ptr->max_task_count = ((RS_BUFFER_SIZE - sizeof(*next_node_ptr)) / sizeof(*next_node_ptr))
														+ ARRAY_SIZE(node_ptr->tasks);
					node_ptr->next = next_node_ptr;
					node_ptr = next_node_ptr;

					count = 0;
				}

				node_ptr->tasks[count] = parent;
				node_ptr->task_count = count + 1;

				task = parent;
			}
			/*
			else {
				bad = 4;

				break;
			}
			*/
		} while (1);

		if (parent) {
			put_task_struct(parent);
		}
	}

	/*rcu_read_unlock();*/

	if (ret) {
		RS_LOG("upper,bad %d\n", ret);

		free_op_check_params_upper_tasks(params);
	}

	return ret;
}

typedef struct tag_mount_verify_data {
	int mount_type;
	unsigned int mount_flags;
	char *task_infos;
	int task_infos_len;
} mount_verify_data;

typedef enum {
	fptKnown,
	fptUnknown
} file_path_type;

#define RS_MOUNT_FLAG_IGNORE_RECORD (0x01)

#define RS_MOUNT_VERIFY_TASK_INFOS_MAX_LEN (RS_JOURNAL_LOG_LENGTH) /*(512)*/

#define RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN (32)
#define RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN (3)
#define RS_MOUNT_VERIFY_TASK_INFOS_MAX_PATH_LEN_DIVIDER (4)

#if defined(CONFIG_DO_RS_JOURNAL)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define fill_mount_verify_task_info RS_HIDE(_o9)
#endif
noused notrace static void fill_mount_verify_task_info(struct tag_mount_verify_data *data,
	struct task_struct *task, char *path, char *path_buf,
	char *name_part, file_path_type path_type)
{
	int pos = data->task_infos_len;
	int remain = RS_MOUNT_VERIFY_TASK_INFOS_MAX_LEN - pos;
	char *buf;
	int len;

	if (remain < (RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN + 1)) {
		return;
	}

	/*path0:comm0|path1:comm1*/

	len = RS_BUFFER_SIZE - (path - path_buf) - 1;
	if (len <= 0) {
		return;
	}

	buf = data->task_infos + pos;

	if (pos) {
		*buf++ = '|';
		remain--;
	}

	if (path_type == fptUnknown) {
		if (!name_part) {
			name_part = path + len - 1;
			while ((name_part >= path) && (*name_part != '/')) {
				name_part--;
			}

			if (name_part < path)
				name_part = path;
			else
				name_part++;
		}

		if ((name_part <= path)
			|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
			|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
			|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
			|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
		#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
			|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
		#endif
			) {
			path_type = fptKnown;
		}
	}

	if (path_type == fptKnown) {
		if ((len <= RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN) && (len <= remain)) {
			memcpy(buf, path, len);

			buf += len;
			remain -= len;
		} else {
			if (!name_part) {
				name_part = path + len - 1;
				while ((name_part >= path) && (*name_part != '/')) {
					name_part--;
				}

				if (name_part < path)
					name_part = path;
				else
					name_part++;

				len = RS_BUFFER_SIZE - (name_part - path_buf) - 1;
			} else if (name_part != path) {
				len = RS_BUFFER_SIZE - (name_part - path_buf) - 1;
			}

			if (len <= remain) {
				memcpy(buf, name_part, len);

				buf += len;
				remain -= len;
			} else if (remain > RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN) {
				memcpy(buf, (name_part + len - remain), remain);

				/*buf += remain;*/
				remain = 0;
			} else {
				return;
			}
		}
	} else {
		char *p;
		int max_len = remain / RS_MOUNT_VERIFY_TASK_INFOS_MAX_PATH_LEN_DIVIDER;

		if (max_len < RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN) {
			max_len = RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN;
		}

	#if 0
		if (!name_part) {
			name_part = path + len - 1;
			while ((name_part >= path) && (*name_part != '/')) {
				name_part--;
			}

			if (name_part < path)
				name_part = path;
			else
				name_part++;

			/*len = RS_BUFFER_SIZE - (name_part - path_buf) - 1;*/
		}
	#endif

		p = path;
		while ((len > max_len) && (p < name_part)) {
			if (p == path) {
				p++;
				len--;
			} else {
				while ((p < name_part) && (*p != '/')) {
					p++;
					len--;
				}

				if (p >= name_part)
					break;
				else {
					p++;
					len--;
				}
			}
		}

		if (len <= max_len) {
			memcpy(buf, p, len);

			buf += len;
			remain -= len;
		} else
	#if (RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN <= RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN)
		if (max_len > RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN)
	#endif
		{
			memcpy(buf, (p + len - max_len), max_len);

			buf += max_len;
			remain -= max_len;
		}
	#if (RS_MOUNT_VERIFY_TASK_INFOS_MAX_FULL_PATH_LEN <= RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN)
		else if ((len >= remain) && (remain > RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN)) {
			memcpy(buf, (p + len - remain), remain);

			/*buf += remain;*/
			remain = 0;
		} else {
			return;
		}
	#endif
	}

	if (remain > (RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN + 1)) {
		char *name;

		name = task->comm;
		len = strnlen(name, sizeof(task->comm));

		if (len) {
			*buf++ = ':';
			remain--;

			if (remain >= len) {
				memcpy(buf, name, len);
				remain -= len;
			} else if (remain > RS_MOUNT_VERIFY_TASK_INFOS_MIN_TOKEN_LEN) {
				memcpy(buf, (name + len - remain), remain);

				/*buf += remain;*/
				remain = 0;
			}
		}
	}

	data->task_infos_len = (RS_MOUNT_VERIFY_TASK_INFOS_MAX_LEN - remain);
}
#endif

#define OP_CHECK_FULL_COMPARE (0x01)
#define OP_CHECK_LEADING_PART_COMPARE (0x02)
#define OP_CHECK_NAME_PART_COMPARE (0x04)

typedef int (*op_check_func)(op_check_params *params);

typedef struct {
	const char *pattern;
	int pattern_len;
	unsigned int compare_flags; /*full_compare;*/ /*is_file;*/
	int partial_match_continue;
	int further_check;
	op_check_func check_func;
} op_check_pattern;


/*return 1 if task's privilege is not root*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_is_not_root_privilege RS_HIDE(_p0)
#endif
noused notrace static noinline int task_is_not_root_privilege(struct task_struct *task)
{
	int ret = 0;
	const struct cred *cred;

	cred = get_task_cred(task);
	if (cred) {
		if (!CHECK_ROOT_UID_BY_CRED(cred))
			ret = 1;

		put_cred(cred);
	}

	return ret;
}

#if 1
	#define task_is_ptraced(task) (0)
#else
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_is_ptraced RS_HIDE(_p1)
#endif
noused notrace static noinline int task_is_ptraced(struct task_struct *task)
{
	int ret = 0;
	struct task_struct *t = task;

	/*read_lock(&tasklist_lock);*/

	do {
		if ((t->ptrace != 0) || (!list_empty(&t->ptrace_entry)) /*t is traced by other task*/
			|| (!list_empty(&t->ptraced)) /*t is tracing other task*/
			|| task_is_traced(t)
			|| (test_ti_thread_flag(task_thread_info(t), TIF_SYSCALL_TRACE | TIF_SYSCALL_AUDIT
			/*| TIF_SYSCALL_TRACEPOINT | TIF_SINGLESTEP*/
			))) {
			ret = 1;
			break;
		}
	} while_each_thread(task, t);

	/*read_unlock(&tasklist_lock);*/

	return ret;
}
#endif

#if defined(RS_ART_MODE_SUPPORT)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define has_art_mode RS_HIDE(_p2)
#endif
noused notrace static noinline int has_art_mode(void)
{
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define has_art_mode_checked RS_HIDE(_p2_1)
#define really_has_art_mode RS_HIDE(_p2_2)
#endif
	noused static int has_art_mode_checked;
	noused static int really_has_art_mode;

	if (!has_art_mode_checked) {
		struct path file_path;
		int error;

		error = kern_path("/system/lib/libart.so",
						RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);

	#if defined(RS_64BIT_SUPPORT)
		if (error == -ENOENT) {
			error = kern_path("/system/lib64/libart.so",
							RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
		}
	#endif

		if (!error) {
			path_put(&file_path);

			really_has_art_mode = rs_get_set_value(); /*1;*/
		} else {
			error = kern_path("/data/property/persist.sys.dalvik.vm.lib",
							RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);

			if (!error) {
				path_put(&file_path);

				really_has_art_mode = rs_get_set_value(); /*1;*/
			}
		}

		has_art_mode_checked = rs_get_set_value(); /*1;*/
	}

	return really_has_art_mode;
}

/*
/data/dalvik-cache/system@framework@com.android.location.provider.jar@classes.dex
/data/dalvik-cache/system@app@com.qualcomm.location.apk@classes.dex
/data/dalvik-cache/system@framework@boot.oat
*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_art_system_lib RS_HIDE(_p3)
#endif
noused notrace static noinline int is_art_system_lib(char *path)
{
	int ret = 0;

	if ((STRNCMP(path, check_strings[csDataSlash].str, check_strings[csDataSlash].str_len, csDataSlash) == 0)
		&& (STRNCMP(&path[check_strings[csDataSlash].str_len], check_strings[csDalvikCacheSlash].str,
		check_strings[csDalvikCacheSlash].str_len, csDalvikCacheSlash) == 0)
		) {
		char *p;
		path += (check_strings[csDataSlash].str_len + check_strings[csDalvikCacheSlash].str_len); /*(sizeof("/data/dalvik-cache/") - 1);*/

		p = path--;

		while (*p) {
			if (*p == '@') {
				*p = '/';
			}

			p++;
		}

		if (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
			p = strrchr(path, '/');

			if (p > path) {
				p++;

				if ((STRCMP(p, check_strings[csClassesDex].str, csClassesDex) == 0)
					|| (STRCMP(p, check_strings[csBootOat].str, csBootOat) == 0)
					) {
					(*--p) = '\0';

					{
						struct path file_path;
						int error;

						error = kern_path(path, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
						if (!error) {
							path_put(&file_path);

							ret = 1;
						}
					}
				}
			}
		}
	}

	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_has_non_system_lib RS_HIDE(_p4)
#endif
noused notrace static noinline int task_has_non_system_lib(struct task_struct *task)
{
	int ret = 0;
	struct vm_area_struct *vma = NULL;
	struct mm_struct *mm;
	/*struct path file_path;*/
	char *path_buffer;
	char *path_ptr;
	struct inode *prev_inode, *curr_inode;
	struct inode **inode_ptrs;
	int inode_ptrs_count, i;
	/*struct dentry *system_mount_root;*/
	/*struct vfsmount *system_mount_root;*/
	struct super_block *system_mount_root;
	struct super_block *vendor_mount_root;
#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
	struct super_block *oem_mount_root;
#endif

	extern void fput(struct file *);

	/*
	if (NULL == task) {
		goto out;
	}
	*/

	path_buffer = (char *)RS_GET_TWO_BUFFER();

	if (path_buffer == NULL) {
		goto out;
	}

	inode_ptrs = (struct inode **)(path_buffer + RS_BUFFER_SIZE);

	inode_ptrs_count = 0;

	/*get_task_struct(task);*/ /*task_lock(task);*/

	mm = get_task_mm(task);
	if (!mm) {
		goto out_without_mm;
		return ret;
	}

	vma = mm->mmap;
	if (vma != NULL) {
		struct file *exe_file;
		struct inode *exe_inode;
		int is_other_boot;
		is_other_boot = (rs_get_boot_mode() != RS_NORMAL_BOOT);
		prev_inode = NULL;
		system_mount_root = NULL;
		vendor_mount_root = NULL;
	#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
		oem_mount_root = NULL;
	#endif

		down_read(&mm->mmap_sem);

		exe_file = mm->exe_file;
		exe_inode = ((exe_file) && (exe_file->f_dentry)) ? exe_file->f_dentry->d_inode : NULL;

		while (vma) {
			struct file *file_ptr;

			if ((vma->vm_flags & (VM_EXEC /* | VM_MAYEXEC | VM_EXECUTABLE*/)) == 0)
				goto next_loop;
			file_ptr = vma->vm_file;
			if ((!file_ptr) || (!file_ptr->f_dentry))
				goto next_loop;
			curr_inode = file_ptr->f_dentry->d_inode;
			if (curr_inode == prev_inode)
				goto next_loop;
			if (curr_inode == exe_inode)
				goto next_loop;
			if (d_unlinked(file_ptr->f_dentry))
				goto next_loop;

			{
				prev_inode = curr_inode;

				for (i = 0; (i < inode_ptrs_count) && (inode_ptrs[i] != curr_inode); i++) {
				}

				if (i >= inode_ptrs_count) {
					if (inode_ptrs_count < (RS_BUFFER_SIZE / sizeof(inode_ptrs[0]))) {
						inode_ptrs[inode_ptrs_count] = curr_inode;
						inode_ptrs_count++;
					}

					if (system_mount_root) {
						/*if (file_ptr->f_path.mnt == system_mount_root)*/
						/*if (file_ptr->f_path.mnt->mnt_root == system_mount_root)*/
						if (file_ptr->f_path.mnt->mnt_sb == system_mount_root) {
							continue;
						} else {
							/*RS_LOG("bad lib1:%s\n", file_ptr->f_dentry->d_name.name);*/

							/*ret = 1;*/
							/*break;*/
						}
					}

					if (vendor_mount_root) {
						/*if (file_ptr->f_path.mnt == vendor_mount_root)*/
						/*if (file_ptr->f_path.mnt->mnt_root == vendor_mount_root)*/
						if (file_ptr->f_path.mnt->mnt_sb == vendor_mount_root) {
							continue;
						} else {
							/*RS_LOG("bad lib1.1:%s\n", file_ptr->f_dentry->d_name.name);*/

							/*ret = 1;*/
							/*break;*/
						}
					}

				#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
					if (oem_mount_root) {
						/*if (file_ptr->f_path.mnt == oem_mount_root)*/
						/*if (file_ptr->f_path.mnt->mnt_root == oem_mount_root)*/
						if (file_ptr->f_path.mnt->mnt_sb == oem_mount_root) {
							continue;
						} else {
							/*RS_LOG("bad lib1.2:%s\n", file_ptr->f_dentry->d_name.name);*/

							/*ret = 1;*/
							/*break;*/
						}
					}
				#endif

					/*get_file(file_ptr);*/

					/*path_buffer[RS_BUFFER_SIZE - 1] = '\0';*/
					path_ptr = D_PATH_EX(&file_ptr->f_path, path_buffer, RS_BUFFER_SIZE);
					/*fput(file_ptr);*/

					if (path_ptr) {
						if ((STRNCMP(path_ptr, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) != 0)
							&& (STRNCMP(path_ptr, check_strings[csMntSlash].str, check_strings[csMntSlash].str_len, csMntSlash) != 0)
							&& (STRNCMP(path_ptr, check_strings[csProcSlash].str, check_strings[csProcSlash].str_len, csProcSlash) != 0)
							&& (STRNCMP(path_ptr, check_strings[csSysSlash].str, check_strings[csSysSlash].str_len, csSysSlash) != 0)
							) {
							/*RS_LOG("lib:%s\n", path_ptr);*/
							/*
							struct vfsmount *vfsmnt = file_ptr->f_path.mnt;
							struct mount *mnt = real_mount(vfsmnt);
							struct dentry *mountpoint = mnt->mnt_mountpoint;
							if (mountpoint)
								RS_LOG("lib:%s\n", mountpoint->d_name.name); //"system"
							*/

							if (STRNCMP(path_ptr, check_strings[csSystemSlash].str,
								check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
								/*system_mount_root = file_ptr->f_path.mnt;*/
								/*system_mount_root = file_ptr->f_path.mnt->mnt_root;*/
								if (!system_mount_root) {
									system_mount_root = file_ptr->f_path.mnt->mnt_sb;
								}
								continue;
							}

							if (STRNCMP(path_ptr, check_strings[csVendorSlash].str,
								check_strings[csVendorSlash].str_len, csVendorSlash) == 0) {
								/*vendor_mount_root = file_ptr->f_path.mnt;*/
								/*vendor_mount_root = file_ptr->f_path.mnt->mnt_root;*/
								if (!vendor_mount_root) {
									vendor_mount_root = file_ptr->f_path.mnt->mnt_sb;
								}
								continue;
							}

						#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
							if (STRNCMP(path_ptr, check_strings[csOemSlash].str,
								check_strings[csOemSlash].str_len, csOemSlash) == 0) {
								/*oem_mount_root = file_ptr->f_path.mnt;*/
								/*oem_mount_root = file_ptr->f_path.mnt->mnt_root;*/
								if (!oem_mount_root) {
									oem_mount_root = file_ptr->f_path.mnt->mnt_sb;
								}
								continue;
							}
						#endif

						#if defined(RS_IS_ANDROID_10_ABOVE)
							if ((STRNCMP(path_ptr, check_strings[csApexSlash].str,
								check_strings[csApexSlash].str_len, csApexSlash) == 0)
								|| (STRNCMP(path_ptr, check_strings[csBionicSlash].str,
								check_strings[csBionicSlash].str_len, csBionicSlash) == 0)) {
								continue;
							}
						#endif

							/*survival 模式自带的 .so 文件都放在/sbin或/sbin/lib下*/
							if ((is_other_boot)
								&& ((!STRNCMP(path_ptr, check_strings[csSbinSlash].str,
								check_strings[csSbinSlash].str_len, csSbinSlash))
								/*android 7.x 也会在 /system_root/system/lib 下*/
								|| (!STRNCMP(path_ptr, check_strings[csRecoverySystemSlash].str,
								check_strings[csRecoverySystemSlash].str_len, csRecoverySystemSlash))
								)) {
								continue;
							}

						#if defined(RS_ART_MODE_SUPPORT)
							if (!is_art_system_lib(path_ptr))
						#endif
							{
								/*RS_LOG("bad lib2:%d,%s\n", is_other_boot, path_ptr);*/
								ret = 1;

								break;
							}

							/*break;*/
						}
					} else {
						/*break;*/
					}
				}
			}

		next_loop:
			vma = vma->vm_next;
		}

		up_read(&mm->mmap_sem);
	}

	mmput(mm);

out_without_mm:
	/*put_task_struct(task);*/ /*task_unlock(task);*/

	RS_FREE_TWO_BUFFER(path_buffer);
out:
	return ret;
}

noused notrace unsigned int get_task_reap_init_flag(void)
{
#if defined(PF_ALIGNWARN)
	#if !defined(PF_WQ_WORKER)
		#define REAP_INIT_FLAG 0x00000020
	#else
		#if !defined(PF_NPROC_EXCEEDED)
			#define REAP_INIT_FLAG 0x00001000
		#else
			#error "can't not define REAP_INIT_FLAG"
		#endif
	#endif
#else
	#define REAP_INIT_FLAG 0x00000001
#endif

	return REAP_INIT_FLAG;
}

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_dbg_task_on RS_HIDE(_pA)
#endif
noused static unsigned g_dbg_task_on;

#undef DBG_TASK_ON_MAGIC
#define DBG_TASK_ON_MAGIC 0x4462674F /*"DbgO"*/

#undef TASK_DBG_FLAG

#if defined(PF_STARTING)
	#if !defined(PF_OOM_ORIGIN)
		#define TASK_DBG_FLAG 0x00080000
	#else
		#if defined(PF_ALIGNWARN)
			/*#error "can't not define TASK_DBG_FLAG, bit used by REAP_INIT_FLAG"*/
		#else
			#if !defined(PF_NPROC_EXCEEDED)
				#define TASK_DBG_FLAG 0x00001000
			#else
				/*#error "can't not define TASK_DBG_FLAG"*/
			#endif
		#endif
	#endif
#else
	#define TASK_DBG_FLAG 0x00000002
#endif

#if !defined(TASK_DBG_FLAG)
/*depending on field definition (should be bit field like "in_execve:1", refer to sched.h) after task_struct.personality
and fork.c : dup_task_struct() -> arch_dup_task_struct() implementation
*/

/*64位int还是32位，只是long为64位*/
typedef struct tag_task_flags_struct {
	unsigned dummy: 31;
	unsigned dbg_flag: 1;
} task_flags_struct;

typedef union tag_task_flags_union {
	task_flags_struct flags;
	unsigned value;
} task_flags_union;

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_task_dbg_flag RS_HIDE(_p5)
#endif
noused notrace static unsigned int get_task_dbg_flag(struct task_struct *task)
{
	unsigned int ret;
	struct task_struct *group_leader;

	if ((DBG_TASK_ON_MAGIC != g_dbg_task_on) || (!task)) {
		return 0;
	}

	RS_READ_TASK_LOCK();

	group_leader = task->group_leader;

	if (group_leader) {
		task = group_leader;
	}

#if !defined(TASK_DBG_FLAG)
	task_flags_union *task_flags = (task_flags_union *)((char *)(&(task->personality)) + sizeof(task->personality));
	ret = task_flags->flags.dbg_flag;
#else
	ret = ((task->flags & TASK_DBG_FLAG) ? 1 : 0);
#endif

	RS_READ_TASK_UNLOCK();

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_task_dbg_flag RS_HIDE(_p6)
#endif
noused notrace static void set_task_dbg_flag(struct task_struct *task)
{
	struct task_struct *group_leader;

	if (!task) {
		return;
	}

	RS_READ_TASK_LOCK();

	group_leader = task->group_leader;

	if (group_leader) {
		task = group_leader;
	}

#if !defined(TASK_DBG_FLAG)
	task_flags_union *task_flags = (task_flags_union *)((char *)(&(task->personality)) + sizeof(task->personality));
	task_flags->flags.dbg_flag = 1;
#else
	task->flags |= TASK_DBG_FLAG;
#endif

	RS_READ_TASK_UNLOCK();

	if (!g_dbg_task_on) {
		g_dbg_task_on = DBG_TASK_ON_MAGIC;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define clear_task_dbg_flag RS_HIDE(_p7)
#endif
noused notrace static void clear_task_dbg_flag(struct task_struct *task)
{
	struct task_struct *group_leader;

	if (!task) {
		return;
	}

	RS_READ_TASK_LOCK();

	group_leader = task->group_leader;

	if (group_leader) {
		task = group_leader;
	}

#if !defined(TASK_DBG_FLAG)
	task_flags_union *task_flags = (task_flags_union *)((char *)(&(task->personality)) + sizeof(task->personality));
	task_flags->flags.dbg_flag = 0;
#else
	task->flags &= ~TASK_DBG_FLAG;
#endif

	RS_READ_TASK_UNLOCK();
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_maybe_injected RS_HIDE(_pa)
#endif
noused notrace static noinline int task_maybe_injected(struct task_struct *task)
{
	int ret = 0;

	if (task == NULL) {
		return ret;
	}

	get_task_struct(task);

	if (task_is_ptraced(task)) {
		RS_LOG("ptraced\n");

		ret = -EPERM;
	} else {
		struct task_struct *group_leader = task->group_leader;

		if ((group_leader) && (group_leader->flags & get_task_reap_init_flag())) {
			RS_LOG("orphaned %s, %d\n", group_leader->comm, task->pid);

			if ((group_leader->comm[0] == 's') && (group_leader->comm[1] == 'h')
				&& (group_leader->comm[2] == '\0')) {
				char *pathbuf, *path;

				path = get_absolute_path(group_leader, &pathbuf);

				if (path) {
					if (STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) != 0) {
						ret = -EPERM;
					}

					free_path(pathbuf);
				} else {
					ret = -EPERM;
				}
			} else {
				ret = -EPERM;
			}
#if 0
			if (group_leader->comm[0] == 's' && group_leader->comm[1] == 'h'
				&& group_leader->comm[2] == '\0') {
			} else {
				ret = -EPERM;
			}
#endif
			if (ret) {
				goto out;
			}
		}
	#if 1
		if (task_has_non_system_lib(task)) {
			RS_LOG("bad lib\n");

			ret = -EPERM;
		}
	#endif
	}

out:
	put_task_struct(task);

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_has_vivo_apk RS_HIDE(_pb)
#endif
noused notrace static noinline int task_has_vivo_apk(struct task_struct *task, const char *apk_name, int apk_name_seed)
{
	int ret = 0;
	struct vm_area_struct *vma = NULL;
	struct mm_struct *mm;
	/*struct path file_path;*/
	char *path_buffer;
	char *path_ptr;
	struct inode *prev_inode, *curr_inode;
	struct inode **inode_ptrs;
	int inode_ptrs_count, i;
#if defined(CONFIG_CHECK_VIVO_FILE)
	int got_vivo_framework;
#endif
	int got_apk;

	extern void fput(struct file *);

	if (NULL == task) {
		goto out;
	}

	path_buffer = (char *)RS_GET_TWO_BUFFER();

	if (path_buffer == NULL) {
		goto out;
	}

	inode_ptrs = (struct inode **)(path_buffer + RS_BUFFER_SIZE);

	inode_ptrs_count = 0;

	get_task_struct(task); /*task_lock(task);*/

	mm = get_task_mm(task);
	if (!mm) {
		goto out_without_mm;
	}

	vma = mm->mmap;
	if (vma != NULL) {
		int apk_name_len = strlen(apk_name);
	#if defined(CONFIG_CHECK_VIVO_FILE)
		got_vivo_framework = 0;
	#endif
		got_apk = 0;
		prev_inode = NULL;

		down_read(&mm->mmap_sem);

		while (vma) {
			struct file *file_ptr;

			if ((vma->vm_flags & (VM_READ | VM_MAYSHARE)) != (VM_READ | VM_MAYSHARE)) /*r--s*/
				goto next_loop;
			file_ptr = vma->vm_file;
			if ((!file_ptr) || (!file_ptr->f_dentry))
				goto next_loop;
			curr_inode = file_ptr->f_dentry->d_inode;
			if (curr_inode == prev_inode)
				goto next_loop;
			if (d_unlinked(file_ptr->f_dentry))
				goto next_loop;

			{
				prev_inode = curr_inode;

				for (i = 0; (i < inode_ptrs_count) && (inode_ptrs[i] != curr_inode); i++) {
				}

				if (i >= inode_ptrs_count) {
					char *ext;

					if (inode_ptrs_count < (RS_BUFFER_SIZE / sizeof(inode_ptrs[0]))) {
						inode_ptrs[inode_ptrs_count] = curr_inode;
						inode_ptrs_count++;
					}

					/*get_file(file_ptr);*/

					ext = strrchr(file_ptr->f_dentry->d_name.name, '.');
					if (ext) {
						ext++;

						if (/*(STRCMP(ext, check_strings[csJar].str, csJar) == 0)
							||*/
							((ext[0] == 'a') && (ext[1] == 'p') && (ext[2] == 'k') && (ext[3] == '\0')) /*(STRCMP(ext, check_strings[csApk].str, csApk) == 0)*/
							) {
							/*path_buffer[RS_BUFFER_SIZE - 1] = '\0';*/
							path_ptr = D_PATH_EX(&file_ptr->f_path, path_buffer, RS_BUFFER_SIZE);
							/*fput(file_ptr);*/

							if (path_ptr) {
								/*
								if ((STRNCMP(path_ptr, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) != 0)
									&& (STRNCMP(path_ptr, check_strings[csMntSlash].str, check_strings[csMntSlash].str_len, csMntSlash) != 0)
									&& (STRNCMP(path_ptr, check_strings[csProcSlash].str, check_strings[csProcSlash].str_len, csProcSlash) != 0)
									&& (STRNCMP(path_ptr, check_strings[csSysSlash].str, check_strings[csSysSlash].str_len, csSysSlash) != 0)
									)
								*/
								{
								#if defined(CONFIG_CHECK_VIVO_FILE)
									/*老版本(4.0.4)只有 bbk-common.jar，无 vivo-xxx.apk*/
									if (!got_vivo_framework) {
										if (STRNCMP(path_ptr, check_strings[csFrameworkVivo].str,
											check_strings[csFrameworkVivo].str_len, csFrameworkVivo) == 0) {
											got_vivo_framework = 1;
											if (got_apk)
												break;
										}
									}
								#endif

									if (!got_apk) {
										if (STRNCMP(path_ptr, check_strings[csSystemAppSlash].str,
											check_strings[csSystemAppSlash].str_len, csSystemAppSlash) == 0) {
											#undef START_IDX
											#define START_IDX (sizeof("/system/app/") - 1)

											if (STRNCMP(&path_ptr[START_IDX], apk_name,
												apk_name_len, apk_name_seed) == 0) {
												got_apk = 1;

											#if defined(CONFIG_CHECK_VIVO_FILE)
												if (got_vivo_framework)
											#endif
													break;
											}
										}
									}
								}
							} else {
								/*break;*/
							}
						}
					}
				}
			}

		next_loop:
			vma = vma->vm_next;
		}

		up_read(&mm->mmap_sem);

		if (
		#if defined(CONFIG_CHECK_VIVO_FILE)
			got_vivo_framework &&
		#endif
			got_apk) {
			ret = 1;
		}
	}

	mmput(mm);

out_without_mm:
	put_task_struct(task); /*task_unlock(task);*/

	RS_FREE_TWO_BUFFER(path_buffer);
out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_is_zygote RS_HIDE(_pc)
#endif
noused notrace static noinline int task_is_zygote(struct task_struct *task)
{
	int ret = 0;
	struct vm_area_struct *vma = NULL;
	struct mm_struct *mm;
	/*struct path file_path;*/
	char *path_buffer;
	char *path_ptr;
	struct inode *prev_inode, *curr_inode;
	struct inode **inode_ptrs;
	int inode_ptrs_count, i;
	int good_apks, bad_apks;

	extern void fput(struct file *);

	if (NULL == task) {
		goto out;
	}

	path_buffer = (char *)RS_GET_TWO_BUFFER();

	if (path_buffer == NULL) {
		goto out;
	}

	inode_ptrs = (struct inode **)(path_buffer + RS_BUFFER_SIZE);

	inode_ptrs_count = 0;

	get_task_struct(task); /*task_lock(task);*/

	mm = get_task_mm(task);
	if (!mm) {
		goto out_without_mm;
	}

	vma = mm->mmap;
	if (vma != NULL) {
		#undef GOOD_APKS_FLAGS
	#if defined(CONFIG_CHECK_VIVO_FILE)
		#define GOOD_APKS_FLAGS (1 | 2)
	#else
		#define GOOD_APKS_FLAGS (1)
	#endif
		good_apks = 0;
		bad_apks = 0;
		prev_inode = NULL;

		down_read(&mm->mmap_sem);

		while (vma) {
			struct file *file_ptr;

			if ((vma->vm_flags & (VM_READ | VM_MAYSHARE)) != (VM_READ | VM_MAYSHARE)) /*r--s*/
				goto next_loop;
			file_ptr = vma->vm_file;
			if ((!file_ptr) || (!file_ptr->f_dentry))
				goto next_loop;
			curr_inode = file_ptr->f_dentry->d_inode;
			if (curr_inode == prev_inode)
				goto next_loop;
			if (d_unlinked(file_ptr->f_dentry))
				goto next_loop;

			/*if ((!curr_inode) || (special_file(curr_inode->i_mode)))
				goto next_loop;
			*/

		{
				prev_inode = curr_inode;

				for (i = 0; (i < inode_ptrs_count) && (inode_ptrs[i] != curr_inode); i++) {
				}

				if (i >= inode_ptrs_count) {
					char *ext;

					if (inode_ptrs_count < (RS_BUFFER_SIZE / sizeof(inode_ptrs[0]))) {
						inode_ptrs[inode_ptrs_count] = curr_inode;
						inode_ptrs_count++;
					}

					/*get_file(file_ptr);*/
					/*RS_LOG("iszg 0,%s\n", file_ptr->f_dentry->d_name.name);*/

					ext = strrchr(file_ptr->f_dentry->d_name.name, '.');
					if (ext) {
						ext++;

						/*if ((STRCMP(ext, check_strings[csApk].str, csApk) == 0))*/
						if ((ext[0] == 'a') && (ext[1] == 'p') && (ext[2] == 'k') && (ext[3] == '\0')) {
							RS_LOG("iszg 1\n");

							path_ptr = D_PATH_EX(&file_ptr->f_path, path_buffer, RS_BUFFER_SIZE);
							/*fput(file_ptr);*/

							if (path_ptr) {
								/*最多打开/system/framework/framework-res.apk、/system/framework/vivo-res.apk，但未打开其他apk*/
								RS_LOG("iszg 2,%s\n", path_ptr);

								if (STRNCMP(path_ptr, check_strings[csSystemFrameworkSlash].str,
									check_strings[csSystemFrameworkSlash].str_len, csSystemFrameworkSlash) == 0) {
									#undef START_IDX
									#define START_IDX (sizeof("/system/framework/") - 1)

									RS_LOG("iszg 3\n");

									if (!(good_apks & 1) && (STRNCMP(&path_ptr[START_IDX], check_strings[csFrameworkResPart].str,
										check_strings[csFrameworkResPart].str_len, csFrameworkResPart) == 0)) {
										RS_LOG("iszg 4\n");

										good_apks |= 1;

										if (good_apks == GOOD_APKS_FLAGS) {
											break;
										}
									}
								#if defined(CONFIG_CHECK_VIVO_FILE)
									else if (!(good_apks & 2) && (STRNCMP(&path_ptr[START_IDX], check_strings[csVivoResPart].str,
										check_strings[csVivoResPart].str_len, csVivoResPart) == 0)) {
										RS_LOG("iszg 5\n");

										good_apks |= 2;

										if (good_apks == GOOD_APKS_FLAGS) {
											break;
										}
									}
								#endif
									else {
										RS_LOG("iszg 6\n");

										bad_apks++;
										break;
									}
								} else {
									RS_LOG("iszg 7\n");

									bad_apks++;
									break;
								}
							} else {
								/*bad_apks++;*/
								/*break;*/
							}
						}
					}
				}
			}

		next_loop:
			vma = vma->vm_next;
		}

		up_read(&mm->mmap_sem);

		if (!bad_apks) {
			if (good_apks <= GOOD_APKS_FLAGS) {
				RS_LOG("iszg OK\n");
				ret = 1;
			}
		}
	}

	mmput(mm);

out_without_mm:
	put_task_struct(task); /*task_unlock(task);*/

	RS_FREE_TWO_BUFFER(path_buffer);
out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define task_is_system_server RS_HIDE(_pd)
#endif
noused notrace static noinline int task_is_system_server(struct task_struct *task)
{
	int ret = 0;
	struct vm_area_struct *vma = NULL;
	struct mm_struct *mm;
	/*struct path file_path;*/
	char *path_buffer;
	char *path_ptr;
	struct inode *prev_inode, *curr_inode;
	struct inode **inode_ptrs;
	int inode_ptrs_count, i;
	int good_libs;
#if defined(RS_64BIT_SUPPORT)
	int is_64bit;
#endif

	extern void fput(struct file *);

	if (NULL == task) {
		goto out;
	}

	path_buffer = (char *)RS_GET_TWO_BUFFER();

	if (path_buffer == NULL) {
		goto out;
	}

	inode_ptrs = (struct inode **)(path_buffer + RS_BUFFER_SIZE);

	inode_ptrs_count = 0;

	get_task_struct(task);

	mm = get_task_mm(task);
	if (!mm) {
		goto out_without_mm;
	}

	vma = mm->mmap;
	if (vma != NULL) {
	#if defined(RS_64BIT_SUPPORT)
		/*mm->task_size/mm->highest_vm_end*/
		/*start_code < start_data < start_brk < arg_start < env_start*/
		is_64bit = ((mm->start_code > 0xffffffff) ? 1 : 0);
	#endif
		good_libs = 0;
		prev_inode = NULL;

		down_read(&mm->mmap_sem);

		while (vma) {
			struct file *file_ptr;

			if (!(vma->vm_flags & (VM_EXEC)))
				goto next_loop;
			file_ptr = vma->vm_file;
			if ((!file_ptr) || (!file_ptr->f_dentry))
				goto next_loop;
			curr_inode = file_ptr->f_dentry->d_inode;
			if (curr_inode == prev_inode)
				goto next_loop;
			if (d_unlinked(file_ptr->f_dentry))
				goto next_loop;

			{
				prev_inode = curr_inode;

				for (i = 0; (i < inode_ptrs_count) && (inode_ptrs[i] != curr_inode); i++) {
				}

				if (i >= inode_ptrs_count) {
					char *ext;

					if (inode_ptrs_count < (RS_BUFFER_SIZE / sizeof(inode_ptrs[0]))) {
						inode_ptrs[inode_ptrs_count] = curr_inode;
						inode_ptrs_count++;
					}

					/*get_file(file_ptr);*/

					ext = strrchr(file_ptr->f_dentry->d_name.name, '.');
					if (ext) {
						ext++;

						/*if ((STRCMP(ext, check_strings[csApk].str, csApk) == 0))*/
						if ((ext[0] == 's') && (ext[1] == 'o') && (ext[2] == '\0')) {
							/*path_buffer[RS_BUFFER_SIZE - 1] = '\0';*/
							path_ptr = D_PATH_EX(&file_ptr->f_path, path_buffer, RS_BUFFER_SIZE);
							/*fput(file_ptr);*/

							if (path_ptr) {
							#if defined(RS_64BIT_SUPPORT)
								/*system/lib64/libandroid_servers.so、/system/lib64/libsystem_server.so*/
								if (is_64bit) {
									if (STRNCMP(path_ptr, check_strings[csSystemLib64Prefix].str,
										check_strings[csSystemLib64Prefix].str_len, csSystemLib64Prefix) == 0) {
										#undef START_IDX_64
										#define START_IDX_64 (sizeof("/system/lib64/lib") - 1)

										/*RS_LOG("isss 0\n");*/

										if (!(good_libs & 1) && (STRNCMP(&path_ptr[START_IDX_64],
											check_strings[csAndroidServersPart].str,
											check_strings[csAndroidServersPart].str_len,
											csAndroidServersPart) == 0)) {
											RS_LOG("isss 1.\n");

											good_libs |= 1;
										}

										if (good_libs == (1 /*| 2 | 4 | 8*/)) {
											ret = 1;

											RS_LOG("isss OK.\n");
											break;
										}
									}
								} else {
							#endif

								/*system/lib/libandroid_servers.so、/system/lib/libsystem_server.so*/
								if (STRNCMP(path_ptr, check_strings[csSystemLibPrefix].str,
									check_strings[csSystemLibPrefix].str_len, csSystemLibPrefix) == 0) {
									#undef START_IDX
									#define START_IDX (sizeof("/system/lib/lib") - 1)

									/*RS_LOG("isss 0\n");*/

									if (!(good_libs & 1) && (STRNCMP(&path_ptr[START_IDX],
										check_strings[csAndroidServersPart].str,
										check_strings[csAndroidServersPart].str_len,
										csAndroidServersPart) == 0)) {
										RS_LOG("isss 1\n");

										good_libs |= 1;
									}
								#if 0
									else if (!(good_libs & 2) && (STRNCMP(&path_ptr[START_IDX],
										check_strings[csSystemServer].str,
										check_strings[csSystemServer].str_len, csSystemServer) == 0)) {
										/*android 4.4 及之后版本无 libsystem_server.so*/
										RS_LOG("isss 2\n");

										good_libs |= 2;
									} else if (!(good_libs & 4) && (STRNCMP(&path_ptr[START_IDX],
										check_strings[csSensorServicePart].str,
										check_strings[csSensorServicePart].str_len, csSensorServicePart) == 0)) {
										RS_LOG("isss 3\n");

										good_libs |= 4;
									} else if (!(good_libs & 8) && (STRNCMP(&path_ptr[START_IDX],
										check_strings[csSurfaceFlingerPart].str,
										check_strings[csSurfaceFlingerPart].str_len, csSurfaceFlingerPart) == 0)) {
										/*android 4.4 无此 lib*/
										RS_LOG("isss 4\n");

										good_libs |= 8;
									}
								#endif

									if (good_libs == (1 /*| 2 | 4 | 8*/)) {
										ret = 1;

										RS_LOG("isss OK\n");
										break;
									}
								}
							#if defined(RS_64BIT_SUPPORT)
								}
							#endif
							} else {
								/*break;*/
							}
						}
					}
				}
			}

		next_loop:
			vma = vma->vm_next;
		}

		up_read(&mm->mmap_sem);

		/*if (good_libs == (1 | 2 | 4 | 8)) {
			ret = 1;
		}
		*/
	}

	mmput(mm);

out_without_mm:
	put_task_struct(task); /*task_unlock(task);*/

	RS_FREE_TWO_BUFFER(path_buffer);
out:
	return ret;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define internal_mksh_check_func RS_HIDE(_q0)
#endif
noused notrace
static noinline int internal_mksh_check_func(op_check_params *params, int task_index)
{
	/*disallowed: return 1; allowed: return 0; error: return -1*/
	int ret = 1;
	char *path;
	struct check_task_node *node;
	struct task_struct *task; /*, *grand_parent, *real_parent;*/
	struct files_struct *files;
	struct fdtable *fdt;
	/*unsigned int *fds;*/
	int sh_files_checked = 0;
	int ok_sh_files = 0;
	int bad_sh_files = 0;
	int real_bad_sh_files = 0;
	int not_sh_files = 0;

	if (!params) {
		return ret;
	}

	task = GOTO_TO_TASK(params->head_ptr, node, task_index);
	if (!task) {
		return ret;
	}

	/*
	if (!PROCEED_TO_PARENT(node, task_index)) {
		return ret;
	}

	parent = node->tasks[task_index];

	get_task_struct(parent); //task_lock(parent);
	*/

	/*
	grand_parent = parent->parent;
	real_parent = parent->real_parent;
	*/

	files = task->files;
	if ((files == NULL)
		|| (task_maybe_injected(task) < 0)) {
		return ret;
	}

	spin_lock(&files->file_lock);

	fdt = files_fdtable(files);
	if (fdt) {
		struct file *file;
		int fd;

		RS_LOG("2,%d\n", fdt->max_fds);

		for (fd = 0; (fd < fdt->max_fds); fd++) {
			/*file = fcheck_files(files, fd);*/
			file = rcu_dereference_check_fdtable(files, fdt->fd[fd]);

			if ((file) && (fd_is_open(fd, fdt)) && (file->f_dentry)
				&& (!d_unlinked(file->f_dentry))
				&& (file->f_dentry->d_inode)
				) {
				umode_t mode = file->f_dentry->d_inode->i_mode;

				if (!special_file(mode)) {
					path = D_PATH_EX(&file->f_path, params->buffer, params->buffer_size);

					if (!path) {
						not_sh_files++;

						break;
					} else if (/*(STRNCMP(path, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) != 0)
						&& (STRNCMP(path, check_strings[csMntSlash].str, check_strings[csMntSlash].str_len, csMntSlash) != 0)
						&&*/ (STRNCMP(path, check_strings[csProcSlash].str, check_strings[csProcSlash].str_len, csProcSlash) != 0)
						&& (STRNCMP(path, check_strings[csSysSlash].str, check_strings[csSysSlash].str_len, csSysSlash) != 0)
						) {
						char *ext;

						RS_LOG("open:%s\n", path);

						ext = strrchr(path, '.');

						if ((ext) && (ext[1] == 's') && (ext[2] == 'h')
							&& (ext[3] == '\0')) {
							if (
								(strrchr(path, '/') <= path)
								|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
								|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
							#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
								|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
							#endif
								) {
								/*sdcard0.sh/sdcard1.sh*/
								/*if (
									(STRNCMP(path, check_strings[csSdcard0sh].str, check_strings[csSdcard0sh].str_len, csSdcard0sh) == 0)
									|| (STRNCMP(path, check_strings[csSdcard1sh].str, check_strings[csSdcard1sh].str_len, csSdcard1sh) == 0)
									)
								*/
								{
									ok_sh_files++;
								}
								/*
								else {
									bad_sh_files++;
								}
								*/
							} else {
								if (
									(STRNCMP(path, check_strings[csDataSlash].str, check_strings[csDataSlash].str_len, csDataSlash) == 0)
									) {
									/*xxx.sh under /data/...*/
									real_bad_sh_files++;

									break;
								} else {
									bad_sh_files++;
								}
							}
						} else {
							if ((STRNCMP(path, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) != 0)
								&& (STRNCMP(path, check_strings[csMntSlash].str, check_strings[csMntSlash].str_len, csMntSlash) != 0)) {
								not_sh_files++;

								break;
							}
						}
					}
				}
			}
		}


		sh_files_checked = 1;
	}

	spin_unlock(&files->file_lock);


	if (sh_files_checked) {
		RS_LOG("3,%d,%d,%d,%d\n", not_sh_files, ok_sh_files, real_bad_sh_files, bad_sh_files);

		/*if (not_sh_files == 0)*/
		{
			if ((not_sh_files == 0) && (ok_sh_files)) {
				ret = 0;
			} else if (real_bad_sh_files) {
			} else {
				/*with/without bad_sh_files*/
			#if defined(CONFIG_DO_RS_JOURNAL)
				mount_verify_data * verify_data;

				if (params->op_type == ovMount) {
					verify_data = (mount_verify_data *)params->op_data;
					if ((verify_data) && (verify_data->mount_type == rmtSystem))
						verify_data->mount_flags |= RS_MOUNT_FLAG_IGNORE_RECORD;
					else
						verify_data = NULL;
				} else {
					verify_data = NULL;
				}
			#endif

				/*init -> adbd -> sh -> ... -> sh -> mount*/
				/*init -> void -> sh -> ... -> mount*/
				/*/////////////////////*/
				do {
					/*struct task_struct *temp_parent;*/
					/*int proceed_result;*/

					if (!PROCEED_TO_PARENT(node, task_index)) {
						break;
					}

					task = node->tasks[task_index];

					{

						if (task_is_not_root_privilege(task)) {

							RS_LOG("grandp not root\n");
							break;
						}

						if (task_maybe_injected(task)) {
							RS_LOG("grandp injected\n");
							break;
						}

						path = get_absolute_path_with_buffer(task, params->buffer, params->buffer_size);
						if (!path) {
							break;
						}

						RS_LOG("grandp:%s\n", path);
						/*
						proceed_result = PROCEED_TO_PARENT(node, task_index);

						temp_parent = grand_parent->parent;
						real_parent = grand_parent->real_parent;
						put_task_struct(grand_parent);

						if ((temp_parent) && (temp_parent == real_parent) && (task_maybe_injected(temp_parent) == 0))
						*/
						{
							if (
								((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
							#if defined(RS_IS_ANDROID_11_ABOVE)
								|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
								|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
								|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
							#endif
								/*|| (STRCMP(path, check_strings[csInit].str, csInit) == 0)
								|| ((params->op_type == ovMount) && (STRCMP(path, check_strings[csVold].str, csVold) == 0))
								|| ((params->op_type == ovInsmod) && ((STRCMP(path, check_strings[csAtcid].str, csAtcid) == 0)
								|| (STRCMP(path, check_strings[csVivoEmSvr].str, csVivoEmSvr) == 0))
								)*/
								)
								) {
							#if defined(CONFIG_DO_RS_JOURNAL)
								if (verify_data) {
									verify_data->mount_flags &= ~RS_MOUNT_FLAG_IGNORE_RECORD;

									if (verify_data->task_infos) {
										fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
									}
									/*now verify_data can set to NULL safely*/
									verify_data = NULL;
								}
							#endif

								if (IS_LAST_PARENT(node, task_index)) {
									/*grand parent is init*/
									{
										ret = 0;
									}
								}

								break;
							} else if (
								((STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0))
								|| ((STRCMP(path, check_strings[csLogWrapper].str, csLogWrapper) == 0))
								) {
							#if defined(CONFIG_DO_RS_JOURNAL)
								if (verify_data) {
									if (verify_data->task_infos) {
										fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
									}
								}
							#endif

								if (IS_LAST_PARENT(node, task_index)) {
									/*grand parent is init*/
									break;
								}

								continue;
							} else {
								char *name_part;

								name_part = strrchr(path, '/');
								if ((name_part <= path)
									/*|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)*/
									|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
									|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
									|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
								#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
									|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
								#endif
									) {
								#if defined(CONFIG_DO_RS_JOURNAL)
									if ((verify_data) && (verify_data->task_infos)) {
										fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
									}
								#endif

									if (IS_LAST_PARENT(node, task_index)) {
										/*grand parent is init*/
										ret = 0;
										break;
									}

									continue;
								} else if (STRNCMP(path, check_strings[csSystemSlash].str,
									check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
									RS_LOG("grandp in sys\n");

									#undef START_IDX0
									#define START_IDX0 (sizeof("/system/") - 1)

									#undef START_IDX
									#define START_IDX (sizeof("/system/bin/") - 1)

									if ((STRNCMP_EX(&path[START_IDX0], check_strings[csAppProcess].str,
										START_IDX0, (sizeof("bin/") - 1), csAppProcess) == 0)
										&& (
									#if defined(RS_64BIT_SUPPORT)
										(STRNCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX,
										(check_strings[csAppProcess].str_len - START_IDX), csAppProcess) == 0)
										|| (STRNCMP_EX(&path[START_IDX], check_strings[csDalvikvm].str, 0,
										check_strings[csDalvikvm].str_len, csDalvikvm) == 0)
									#else
										(STRCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX, csAppProcess) == 0)
										|| (STRCMP(&path[START_IDX], check_strings[csDalvikvm].str, csDalvikvm) == 0)
										|| (STRCMP(&path[START_IDX], check_strings[csDvz].str, csDvz) == 0)
									#endif
										)
										) {
										/*not allowed*/
										RS_LOG("grandp not allowed\n");
									} else {
									#if defined(RS_SU_MORE_RESTRICT)
										/*disallow: su/us/daemonsu/ksud/.su/.us/sugote/kugote/ku.sud,*/
										/*allow: bbksu/superumount/wpa_supplicant/surfaceflinger/subsystem_ramdump/dsdnsutil*/
										int is_bad = 0;
										char *buffer;
										int name_len;
										int buffer_len;
										char sig_char;
										char *pSu;

										name_part++;

										if (name_part[0] == '.') {
											is_bad = 1;

											goto check_su_end;
										}

										name_len = params->buffer + params->buffer_size - name_part; /*include null char*/
										buffer_len = path - params->buffer;
										if (name_len <= buffer_len) {
											buffer = params->buffer;
										} else {
											buffer = (char *)RS_GET_BUFFER();

											if (!buffer) {
												goto check_su_end;
											}
										}

										memcpy(buffer, name_part, name_len);
										strlwr_len(buffer, name_len - 1);
										sig_char = 's';

									su_check_loop:

										pSu = strrchr(buffer, sig_char);

										/*XXXsu 或 XXXus*/
										if (pSu) {
											if (pSu[1] == 'u') {
												if (pSu[2] == '\0') {
													/*su结尾*/
												#if !defined(RS_WITHOUT_BBKSU)
													if ((pSu[0] == 's') && (STRCMP_EX(buffer, sid_check_strings[sidcsBBKSu].str,
														(sizeof("/system/xbin/") - 1), sidcsBBKSu) == 0)) {
														/*bbksu*/
														goto check_su_end;
													} else
												#endif
													{
														is_bad = 1;
														goto check_su_end;
													}
												} else if ((pSu[2] == 'd') || (pSu[2] == 'g')) {
													/*带"sud"/"sug"*/
													is_bad = 1;
													goto check_su_end;
												}
											} else if ((pSu[1] == 'y') && (sig_char == 's') && (pSu[2] == 'b') && (pSu[3] == 'o')
												&& (pSu[4] == 'x') && (pSu > (buffer - 1)) && (pSu[-1] == 'u')
												&& (pSu[-2] == 'b')) {
												/*busybox, only allowed in recovery mode*/
												if (rs_get_boot_mode() != RS_NORMAL_BOOT) {
													is_bad = 1;
													goto check_su_end;
												}
											}

											if ((pSu > buffer) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
												|| ((pSu[-1] == '.') && (/*((pSu[1] == 'o') && (pSu[2] == '\0')) //.so结尾 //360 mobilesafe, libsu.so，liba.so.4x.so
												||*/ (pSu[1] == 'u'))) /*.suXXX*/
												)) {
												is_bad = 1;
											}

										}

										if (!is_bad) {
											if (sig_char != 'k') {
												/*continue check for ku* */
												sig_char = 'k';

												goto su_check_loop;
											}
										}

									check_su_end:
										if ((buffer) && (buffer != params->buffer)) {
											RS_FREE_BUFFER(buffer);
										}

										if (is_bad) {
											/*not allowed*/
											RS_LOG("grandp not allowed su\n");

											break;
										}
									#endif

										/*RS_LOG("grandp next\n");*/
									#if defined(CONFIG_DO_RS_JOURNAL)
										if ((verify_data) && (verify_data->task_infos)) {
											fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
										}
									#endif

										if (IS_LAST_PARENT(node, task_index)) {
											/*grand parent is init*/
											ret = 0;
											break;
										}

										continue;
									}
								}
							}

						}
					}

					break;

				} while (1);

			#if defined(CONFIG_DO_RS_JOURNAL)
				if ((verify_data) && (ret)) {
					verify_data->mount_flags &= ~RS_MOUNT_FLAG_IGNORE_RECORD;
				}
			#endif
			}
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define toolbox_mount_check_func RS_HIDE(_q1)
#endif
noused notrace static noinline int toolbox_mount_check_func(op_check_params *params)
{
	/*disallowed: return 1; allowed: return 0; error: return -errno*/
	int ret = 1;
	struct check_task_node *node;
	int task_index;
	struct task_struct *task;
	/*
	struct task_struct *curr;
	struct task_struct *parent;
	struct task_struct *real_parent;
	unsigned long _flags = ((flags) ? *flags : 0);
	*/
#if defined(CONFIG_DO_RS_JOURNAL)
	mount_verify_data *verify_data;
#endif

	if (!params) {
		return -EINVAL;
	}

	if (params->is_query) {
		/*need upper tasks*/
		return 1;
	}

	if (!params->head_ptr) {
		return -EFAULT;
	}

#if defined(CONFIG_DO_RS_JOURNAL)
	if ((params) && (params->op_type == ovMount)) {
		verify_data = (mount_verify_data *)params->op_data;
		if ((verify_data) && (verify_data->mount_type == rmtSystem))
			verify_data->mount_flags |= RS_MOUNT_FLAG_IGNORE_RECORD;
		else
			verify_data = NULL;
	} else {
		verify_data = NULL;
	}
#endif

	/*如果 parent == real_parent*/

	node = params->head_ptr;
	/*curr = node->tasks[0];*/
	task_index = 0;

loop:
	if (!PROCEED_TO_PARENT(node, task_index)) {
		ret = 0;
		goto out;
	}
#if 0
	task_index++;
	if (task_index >= node->task_count) {
		node = node->next;
		if (!node) {
			goto out;
		}

		task_index = 0;
	}
#endif

	task = node->tasks[task_index];

	{
		char *path;
		/*如果 parent 路径不是 /system/bin/mksh，则不允许
		  如果 parent 打开了非 (/system/ 或 /) 下的 .sh 文件，则如果 parent->parent 不是 adbd/vadbd/vusbd 或 /init 则不允许
		  如果
		*/

		path = get_absolute_path_with_buffer(task, params->buffer, params->buffer_size);
		if (!path) {
			goto out;
		}


		RS_LOG("parent:%s\n", path);

		if (task_is_not_root_privilege(task)) {

			RS_LOG("1\n");

			return ret;
		}

		if ((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
		#if defined(RS_IS_ANDROID_11_ABOVE)
			|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
			|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
			|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
		#endif
			)
		{
			/*struct task_struct *grand_parent;*/
			int tmp;

		#if defined(CONFIG_DO_RS_JOURNAL)
			if (verify_data) {
				verify_data->mount_flags &= ~RS_MOUNT_FLAG_IGNORE_RECORD;

				if (verify_data->task_infos) {
					fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
				}
			}
		#endif

			tmp = PROCEED_TO_PARENT(node, task_index);
			if (!tmp) {
				/*grand parent is init*/
				{
					ret = 0;
				}
			}

			return ret;
		}

		if (STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0) {
			return internal_mksh_check_func(params, task_index);
		}

		if ((STRCMP(path, check_strings[csVivoDaemon].str, csVivoDaemon) == 0)
			|| (STRCMP(path, check_strings[csLogWrapper].str, csLogWrapper) == 0)) {
			/*curr = parent;*/
		#if defined(CONFIG_DO_RS_JOURNAL)
			if ((verify_data) && (verify_data->task_infos)) {
				fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
			}
		#endif

			goto loop;
		}
/*/////*/
#if 0
		if ((strrchr(path, '/') <= path)
			/*|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)*/
			|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
			|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
			|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
		#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
			|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
		#endif
			) {
		#if defined(CONFIG_DO_RS_JOURNAL)
			if ((verify_data) && (verify_data->task_infos)) {
				fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
			}
		#endif

		#if 0
			if (IS_LAST_PARENT(node, task_index)) {
				/*grand parent is init*/
				ret = 0;
				goto out; /*break;*/
			}
		#endif

			goto loop; /*continue;*/
		} else if (STRNCMP(path, check_strings[csSystemSlash].str,
			check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
			/*RS_LOG("grandp in sys\n");*/

			#undef START_IDX0
			#define START_IDX0 (sizeof("/system/") - 1)

			#undef START_IDX
			#define START_IDX (sizeof("/system/bin/") - 1)

			if ((STRNCMP_EX(&path[START_IDX0], check_strings[csAppProcess].str,
				START_IDX0, (sizeof("bin/") - 1), csAppProcess) == 0)
				&& (
			#if defined(RS_64BIT_SUPPORT)
				(STRNCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX,
				(check_strings[csAppProcess].str_len - START_IDX), csAppProcess) == 0)
				|| (STRNCMP_EX(&path[START_IDX], check_strings[csDalvikvm].str, 0,
				check_strings[csDalvikvm].str_len, csDalvikvm) == 0)
			#else
				(STRCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX, csAppProcess) == 0)
				|| (STRCMP(&path[START_IDX], check_strings[csDalvikvm].str, csDalvikvm) == 0)
				|| (STRCMP(&path[START_IDX], check_strings[csDvz].str, csDvz) == 0)
			#endif
				)
				) {
				/*not allowed*/
				/*RS_LOG("grandp not allowed\n");*/
			} else {
				/*RS_LOG("grandp next\n");*/
			#if defined(CONFIG_DO_RS_JOURNAL)
				if ((verify_data) && (verify_data->task_infos)) {
					fill_mount_verify_task_info(verify_data, task, path, params->buffer, NULL, fptKnown);
				}
			#endif

			#if 0
				if (IS_LAST_PARENT(node, task_index)) {
					/*grand parent is init*/
					ret = 0;
					goto out; /*break;*/
				}
			#endif

				goto loop; /*continue;*/
			}
		}
#endif
/*////*/
	}

out:
#if defined(CONFIG_DO_RS_JOURNAL)
	if ((verify_data) && (ret)) {
		verify_data->mount_flags &= ~RS_MOUNT_FLAG_IGNORE_RECORD;
	}
#endif

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_recovery_mode_deeper RS_HIDE(_q2)
#endif
noused notrace static noinline int check_recovery_mode_deeper(void)
{
#if defined(CONFIG_RS_OBFUSCATED_NAME)
	#define recoverymode_checked RS_HIDE(_p)
	#define really_recoverymode RS_HIDE(_q)
#endif
	noused static int recoverymode_checked;
	noused static int really_recoverymode;

	if (!recoverymode_checked) {
		struct path file_path;
		int error;

	#if defined(RS_ENCRYPT_STR)
		char *path = (char *)RS_GET_BUFFER();

		if (!path) {
			return 0;
		}

		memcpy(path, check_strings[csRecovery].str, check_strings[csRecovery].str_len + 1);
		simple_str_encrypt(path, check_strings[csRecovery].str_len, csRecovery);

		error = kern_path(path, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);

		free_path(path);
	#else
		error = kern_path(check_strings[csRecovery].str, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
	#endif

		if (!error) {
			path_put(&file_path);

			really_recoverymode = rs_get_set_value(); /*1;*/
		}

		recoverymode_checked = rs_get_set_value(); /*1;*/
	}

	return really_recoverymode;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define tmp_mount_check_func RS_HIDE(_q3)
#endif
noused notrace static noinline int tmp_mount_check_func(op_check_params *params)
{
	/*disallowed: return 0; allowed: return 1; error: return -errno*/
	int ret = 0;
	struct check_task_node *node;
	int task_index;
	/*noused extern char *saved_command_line;*/

	/*RS_LOG("tmp,%s\n", saved_command_line);*/

	/*RS_LOG("tmp,0,%d\n", isrecoverymode);*/

	if (rs_get_boot_mode() != RS_NORMAL_BOOT) {
		struct task_struct *task;

		if (!params) {
			return -EINVAL;
		}

		if (params->is_query) {
			/*need upper tasks*/
			return 1;
		}

		if (!params->head_ptr) {
			return -EFAULT;
		}

		/*init -> recovery -> update_binary or update-binary -> xxx -> ... -> sys_mount

		如果 parent == real_parent，且 parent 路径是 /sbin/recovery or /system/bin/recovery
		*/
		/*struct task_struct *curr;*/ /* = current;*/
		/*struct task_struct *parent, *real_parent;*/

		/*RS_LOG("tmp,1\n");*/
		node = params->head_ptr;
		task_index = 0;

		do {
			int proceed_result;
			char *path;

			/*get_task_struct(curr);*/
			task = node->tasks[task_index];

			path = get_absolute_path_with_buffer(task, params->buffer, params->buffer_size);

			if (!path) {
				break;
			}

			if (task_is_not_root_privilege(task)) {
				/*RS_LOG("tmp,2\n");*/
				break;
			}

			/*parent = curr->parent;
			real_parent = curr->real_parent;
			put_task_struct(curr);
			*/
			/*RS_LOG("tmp,2.1,%s\n", path);*/

			if (STRCMP(path, check_strings[csUpdateBinary].str, csUpdateBinary) == 0) {
				/*RS_LOG("tmp,3\n");*/

				if (PROCEED_TO_PARENT(node, task_index)) {
					/*RS_LOG("tmp,4\n");*/

					task = node->tasks[task_index];

					if ((!task_is_not_root_privilege(task))
						&& (task_maybe_injected(task) == 0)) {
						path = get_absolute_path_with_buffer(task, params->buffer, params->buffer_size);

						/*RS_LOG("tmp,5,%s\n", path);*/

						if ((path) && (STRCMP(path, check_strings[csRecovery].str, csRecovery) == 0)) {
							/*RS_LOG("tmp,6\n");*/
							proceed_result = PROCEED_TO_PARENT(node, task_index);

							if (!proceed_result) {
								/*init -> recovery -> update_binary or update-binary -> ... -> sys_mount
								grand parent is init
								*/
								/*RS_LOG("tmp,7\n");*/
								ret = 1;
							} else {
								/*RS_LOG("tmp,8\n");*/
								task = node->tasks[task_index];

								if ((!task_is_not_root_privilege(task)) && (task_maybe_injected(task) == 0)) {
									path = get_absolute_path_with_buffer(task, params->buffer, params->buffer_size);
									/*RS_LOG("tmp,9,%s\n", path);*/
									if ((path) && (STRCMP(path, check_strings[csInit].str, csInit) == 0)) {
										/*RS_LOG("tmp,10\n");*/
										proceed_result = PROCEED_TO_PARENT(node, task_index);
										if (!proceed_result) {
											/*init -> init -> recovery -> update_binary or update-binary -> ... -> sys_mount
											grand grand parent is init
											*/
											/*RS_LOG("tmp,11\n");*/
											ret = 1;
										}
									}
								}
							}
						}
					}
				}
			}
		#if defined(RS_CHECK_INIT_BY_NAME)
			else if (STRCMP(path, check_strings[csInit].str, csInit) == 0)
		#else
			else if (is_init_task(task))
		#endif
			{
				/*init*/
				/*RS_LOG("tmp,12\n");*/
				ret = 1;
			} else {
				/*RS_LOG("tmp,13\n");*/
				if (PROCEED_TO_PARENT(node, task_index)) {
					/*RS_LOG("tmp,14\n");*/
					continue;
				}
			}

			break;
		} while (1);
	}

	/*RS_LOG("tmp,e,%d\n", ret);*/
	return ret;
}

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define updater_mount_check_func RS_HIDE(_q4)
#endif
noused notrace static noinline int updater_mount_check_func(char *path, char *buffer, int buffer_size, unsigned long *flags)
{
	/*allowed: return 1; disallowed: return 0*/
	int ret = 0;

	if (rs_get_boot_mode() != RS_NORMAL_BOOT) {
		/*init -> recovery -> update_binary or update-binary -> xxx -> ... -> sys_mount*/

		/*如果 parent == real_parent，且 parent 路径是 /sbin/recovery or /system/bin/recovery */
		struct task_struct *curr = current;
		struct task_struct *parent, *real_parent;

		get_task_struct(curr);
		parent = curr->parent;
		real_parent = curr->real_parent;
		put_task_struct(curr);

		/*如果 parent == real_parent*/
		if ((parent) && (parent == real_parent)) {
			char *path;

			get_task_struct(parent);

			path = get_absolute_path_with_buffer(parent, buffer, buffer_size);
			if (!path) {
				put_task_struct(parent);

				return ret;
			}

			RS_LOG("parent:%s\n", path);
#if 1
			if (task_is_not_root_privilege(parent)) {
				put_task_struct(parent);

				RS_LOG("parent is not root\n");
				return ret;
			}
#endif
			if (STRCMP(path, check_strings[csRecovery].str, csRecovery) == 0) {
				struct task_struct *grand_parent;
				/*get_task_struct(parent);*/
				grand_parent = parent->parent;
				real_parent = parent->real_parent;
				put_task_struct(parent);

				if ((grand_parent) && (grand_parent == real_parent)
					&& (!task_is_not_root_privilege(grand_parent))) {
					/*init -> recovery -> update_binary or update-binary -> sys_mount*/
					get_task_struct(grand_parent);

				#if defined(RS_CHECK_INIT_BY_NAME)
					path = get_absolute_path_with_buffer(grand_parent, buffer, buffer_size);

					if (path) {
						RS_LOG("grandp:%s\n", path);

						if (STRCMP(path, check_strings[csInit].str, csInit) == 0) {
							ret = 1;
						}
					}
				#else
					if (is_init_task(grand_parent)) {
						ret = 1;
					}
				#endif

					put_task_struct(grand_parent);
				}

			} else {
				put_task_struct(parent);
			}
		}
	}

	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define app_process_check_func RS_HIDE(_q5)
#endif
noused notrace static noinline int app_process_check_func(op_check_params *params)
{
	/*allowed: return 0; disallowed: return 1;error: return -1*/
	int ret = 1;
	/*only allow zygote and system_server*/
	struct task_struct *curr;
	struct task_struct *curr_task;
	/*struct task_struct *parent, *real_parent;*/

	if (!params) {
		return -EINVAL;
	}

	if (params->is_query) {
		/*do not need upper tasks*/
		return 0;
	}

	curr = current;

	/*RS_LOG("app,0,%s\n", curr->comm);*/

	if (curr->pid != curr->tgid) {
		curr_task = curr->group_leader;
	} else {
		curr_task = curr;
	}

	if (!curr_task) {
		RS_LOG("app,nil\n");
		return ret;
	}

	get_task_struct(curr_task);

	RS_LOG("app,1,%s\n", curr_task->comm);

#if defined(RS_64BIT_SUPPORT)
	/*zygote64或zygote*/
	if (STRNCMP(curr_task->comm, check_strings[csZygote].str,
		check_strings[csZygote].str_len, csZygote) == 0)
#else
	if (STRCMP(curr_task->comm, check_strings[csZygote].str, csZygote) == 0)
#endif
	{
		/*zygote 有 4 个 thread*/
		RS_LOG("zg 0\n");

		if (!task_is_not_root_privilege(curr_task)) {
			cast_kernel_cap_t pEffective, pInheritable, pPermitted;
			int res;

			res = RS_CAP_GET(curr_task, &pEffective.cap, &pInheritable.cap, &pPermitted.cap);

			if (!res) {
				/*RS_LOG("zg,1,%u,%u\n", pEffective.cap[0], pEffective.cap[1]);*/
				if (((pEffective.val & ZYGOTE_CAP_MASK) == ZYGOTE_CAP_MASK)
					&& (pPermitted.val == pEffective.val)) {
					if (task_is_zygote(curr_task)) {
						RS_LOG("zg OK\n");

						ret = 0;
					}

				}
			}
		}
	} else if (STRCMP(curr_task->comm, check_strings[csSystemServer].str,
		csSystemServer) == 0) {
		/*uid是UID_SYSTEM(1000)*/
		const struct cred *cred;

		RS_LOG("ss 0\n");

		cred = curr_task->cred;

		if ((cred) && (CHECK_UID_BY_CRED(cred, AID_SYSTEM))) {
			noused cast_kernel_cap_t pEffective, pInheritable, pPermitted;
			noused int res;

			RS_LOG("ss 1\n");

#if 1
			/*read_lock(&tasklist_lock);*/
			res = RS_CAP_GET(curr_task, &pEffective.cap, &pInheritable.cap, &pPermitted.cap);
			/*read_unlock(&tasklist_lock);*/

			/*RS_LOG("ss 1.1 %d\n", res);*/

			/*RS_LOG("ss 2,%u,%u,%u\n", pEffective.cap.cap[0], pEffective.cap.cap[1], (unsigned int)(SYSTEM_SERVER_CAP_VALUE2));*/

			if ((!res) && ((SYSTEM_SERVER_CAP_VALUE == pEffective.val)
			#if defined(CAP_BLOCK_SUSPEND) /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)*/
				|| ((SYSTEM_SERVER_CAP_VALUE2 & pEffective.val) == SYSTEM_SERVER_CAP_VALUE2)
			#endif
				)
				&& (pEffective.val == pPermitted.val))
#endif
			{
				/*RS_LOG("ss 3,%u,%u\n", pPermitted.cap.cap[0], pPermitted.cap.cap[1]);*/

				if (task_is_system_server(curr_task)) {
					RS_LOG("ss OK\n");

					ret = 0;
				}
			}

		}
	}

	put_task_struct(curr_task);

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_vivo_root RS_HIDE(_q6)
#endif
noused notrace static noinline int check_vivo_root(void)
{
#if defined(CONFIG_RS_OBFUSCATED_NAME)
	#define vivo_root_checked RS_HIDE(_pr)
	#define really_vivo_root RS_HIDE(_qr)
#endif
	noused static int vivo_root_checked;
	noused static int really_vivo_root;

	if (!vivo_root_checked) {
		struct path file_path;
		int error;

	#if defined(RS_ENCRYPT_STR)
		char *path = (char *)RS_GET_BUFFER();

		if (!path) {
			return 0;
		}

		memcpy(path, check_strings[csBBKDebugFile].str, check_strings[csBBKDebugFile].str_len + 1);
		simple_str_encrypt(path, check_strings[csBBKDebugFile].str_len, csBBKDebugFile);

		error = kern_path(path, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);

		free_path(path);
	#else
		error = kern_path(check_strings[csBBKDebugFile].str, RS_FILE_EXIST_KERN_PATH_FLAGS, &file_path);
	#endif

		if (!error) {
			path_put(&file_path);

			really_vivo_root = rs_get_set_value(); /*1;*/
		}

		vivo_root_checked = rs_get_set_value(); /*1;*/
	}

	return really_vivo_root;
}

/*64bit下分为app_process32和app_process64*/
#if defined(RS_64BIT_SUPPORT)
	#define APP_PROCESS_COMPARE_FLAGS (OP_CHECK_LEADING_PART_COMPARE)
	#define DALVIKVM_COMPARE_FLAGS (OP_CHECK_LEADING_PART_COMPARE)
	#define DVZ_COMPARE_FLAGS (0)
#else
	#if defined(RS_VIVOROOT_SUPPORT) /*android 5.0*/
		#define APP_PROCESS_COMPARE_FLAGS (OP_CHECK_LEADING_PART_COMPARE)
	#else
		#define APP_PROCESS_COMPARE_FLAGS (OP_CHECK_FULL_COMPARE)
	#endif
	#define DALVIKVM_COMPARE_FLAGS (OP_CHECK_LEADING_PART_COMPARE)
	#if defined(RS_VIVOROOT_SUPPORT) /*android 5.0*/
		#define DVZ_COMPARE_FLAGS (0)
	#else
		#define DVZ_COMPARE_FLAGS (OP_CHECK_FULL_COMPARE)
	#endif
#endif

#if defined(RS_ENCRYPT_STR)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mnt_allowed_patterns RS_HIDE(_r0)
#endif
noused static const op_check_pattern mnt_allowed_patterns[] = {
	{
		"\x91\xCE\xDE\xD2\xD4\x96", (sizeof("/sbin/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, NULL
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99", (sizeof("/system/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 1, NULL
	},
#if 1
	{
		"\x93\xCF\xD7\xC9\x97", (sizeof("/tmp/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, tmp_mount_check_func
	},
#else
	{
		"\x93\xCF\xD7\xC9\x97\xC2\xC6\xD1\xD5\xC7\xD7\xEE\xD2\xC6\xC0\xCC\xDE\xD2", (sizeof("/tmp/update_binary") - 1), OP_CHECK_FULL_COMPARE, 0, 0, updater_mount_check_func
	},
#endif
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"\x94\xCC\xDC\xD6\xD3\xD9\xC7\x9B", (sizeof("/vendor/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 1, NULL
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"\x95\xD8\xC8\xD2\xCE\x9A", (sizeof("/apex/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, NULL
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mnt_disallowed_patterns RS_HIDE(_r1)
#endif
noused static const op_check_pattern mnt_disallowed_patterns[] = {
	{
		"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xD4\xDC\xDA\x9C\xC6\xDE\xDF\xC3\xCC\xC2\xD4",
		(sizeof("/system/bin/toolbox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	},
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xD7\xDD\xDD\x9D\xD0\xC0\xDF\xF1\xDD\xDE\xC4\xC9\xCC\xDB\xD4",
		(sizeof("/system/bin/app_process") - 1), APP_PROCESS_COMPARE_FLAGS, 0, 0, app_process_check_func
	},
	{
		"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xD6\xDA\xDC\x9E\xD4\xCE\xC2\xDB\xC5\xC0\xDC\xC4",
		(sizeof("/system/bin/dalvikvm") - 1), DALVIKVM_COMPARE_FLAGS, 0, 0, NULL
	},
	{
		"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xD1\xDB\xDF\x9F\xCB\xD8\xD7",
		(sizeof("/system/bin/dvz") - 1), DVZ_COMPARE_FLAGS, 0, 0, NULL
	},
	{
		"\xD8\xCC\xCB\xCE\xD4\xDA\xCC",
		(sizeof("busybox") - 1), OP_CHECK_NAME_PART_COMPARE, 0, 0, toolbox_mount_check_func
	},
#if defined(RS_HAS_TOYBOX)
	{
		"\x96\xCB\xCE\xC5\xC1\xD1\xDE\x9D\xD3\xD9\xC1\x81\xD9\xC3\xD2\xC8\xC6\xD0",
		(sizeof("/system/bin/toybox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	},
#else
	{
		NULL, 0, 0, 0, 0, NULL
	},
#endif
#if defined(RS_SU_MORE_RESTRICT)
	{
		"\xCB\xC2",
		(sizeof("su") - 1), OP_CHECK_NAME_PART_COMPARE, 0, 0, NULL
	},
#else
	{
		NULL, 0, 0, 0, 0, NULL
	},
#endif
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"\x98\xC0\xD0\xDA\xD7\xDD\xC3\x9F\xCD\xC7\xC3\x83\xDF\xC5\xC6\xC4\xC5\xC9\xDD",
		(sizeof("/vendor/bin/toolbox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
};

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mnt_allowed_patterns RS_HIDE(_r0)
#endif
noused static const op_check_pattern mnt_allowed_patterns[] = {
	{
		"/sbin/", (sizeof("/sbin/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, NULL
	},
	{
		"/system/", (sizeof("/system/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 1, NULL
	},
#if 1
	{
		"/tmp/", (sizeof("/tmp/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, tmp_mount_check_func
	},
#else
	{
		"/tmp/update_binary", (sizeof("/tmp/update_binary") - 1), OP_CHECK_FULL_COMPARE, 0, 0, updater_mount_check_func
	},
#endif
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"/vendor/", (sizeof("/vendor/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 1, NULL
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"/apex/", (sizeof("/apex/") - 1), OP_CHECK_LEADING_PART_COMPARE, 0, 0, NULL
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mnt_disallowed_patterns RS_HIDE(_r1)
#endif
noused static const op_check_pattern mnt_disallowed_patterns[] = {
	{
		"/system/bin/toolbox", (sizeof("/system/bin/toolbox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	},
	{
		"/system/bin/app_process", (sizeof("/system/bin/app_process") - 1), APP_PROCESS_COMPARE_FLAGS, 0, 0, app_process_check_func /*NULL*/
	},
	{
		"/system/bin/dalvikvm", (sizeof("/system/bin/dalvikvm") - 1), DALVIKVM_COMPARE_FLAGS, 0, 0, NULL
	},
	{
		"/system/bin/dvz", (sizeof("/system/bin/dvz") - 1), DVZ_COMPARE_FLAGS, 0, 0, NULL
	},
	{
		"busybox", (sizeof("busybox") - 1), OP_CHECK_NAME_PART_COMPARE, 0, 0, toolbox_mount_check_func
	},
#if defined(RS_HAS_TOYBOX)
	{
		"/system/bin/toybox", (sizeof("/system/bin/toybox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	},
#else
	{
		NULL, 0, 0, 0, 0, NULL
	},
#endif
#if defined(RS_SU_MORE_RESTRICT)
	{
		"su",
		(sizeof("su") - 1), OP_CHECK_NAME_PART_COMPARE, 0, 0, NULL
	},
#else
	{
		NULL, 0, 0, 0, 0, NULL
	},
#endif
	{
	#if defined(RS_IS_ANDROID_11_ABOVE)
		"/vendor/bin/toolbox", (sizeof("/vendor/bin/toolbox") - 1), OP_CHECK_FULL_COMPARE, 0, 0, toolbox_mount_check_func
	#else
		NULL, 0, 0, 0, 0, NULL
	#endif
	},
};
#endif

#ifndef ARRAY_SIZE
	#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define dump_opened_fd RS_HIDE(_s0)
#endif
noused notrace static void dump_opened_fd(struct task_struct *task, char *buffer, int buffer_size)
{
	struct files_struct *files;
	struct fdtable *fdt;
	/*unsigned int *fds;*/
	/*int i;*/

	if (!task) {
		return;
	}

	task_lock(task);

	files = task->files;
	if (files) {
		RS_LOG("1\n");

		spin_lock(&files->file_lock);

		fdt = files_fdtable(files);
		if (fdt) {
			struct file *file;
			int fd;

			RS_LOG("2,%d\n", fdt->max_fds);

			for (fd = 0; fd < fdt->max_fds; fd++) {
				/*file = fcheck_files(files, fd);*/
				file = rcu_dereference_check_fdtable(files, fdt->fd[fd]);

				if ((file) && (fd_is_open(fd, fdt)) && (file->f_dentry)
					&& (!d_unlinked(file->f_dentry))
					&& (file->f_dentry->d_inode)
					) {
					umode_t mode = file->f_dentry->d_inode->i_mode;

					if (!special_file(mode)) {
						char *ret_ptr = D_PATH_EX(&file->f_path, buffer, buffer_size);

						if (ret_ptr) {
							RS_LOG("open:%s\n", ret_ptr);
						}
					}
				}
			}
		}

		spin_unlock(&files->file_lock);
	}

	task_unlock(task);
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_op_verify RS_HIDE(_s1)
#endif

RS_DEFINE_FC_BEGIN(do_op_verify);

noused notrace RS_DEFINE_FC_BODY(do_op_verify)
static noinline int do_op_verify(int op_type, ssize_t/*int*/ op_data)
{
	/*pass: 0; fail: < 0*/
	int retval = 0;

	{
		/*struct task_struct *curr;
		struct mm_struct *mm;
		struct file *exe_file;
		struct path exe_path;
		char *abs_path;
		struct inode *exe_inode = NULL;
		unsigned long flags = (unsigned long)op_type; //0
		*/
		extern void fput(struct file *);
		char *pathbuf;

		/*if ((curr = current) != NULL)
			curr = current;
		*/

		pathbuf = (char *)RS_GET_TWO_BUFFER();
		if (pathbuf) {
			op_check_params params;

			RS_LOG("ov:%d\n", op_type);

			/*params.head_ptr = NULL;*/
			params.current_task = current;

			params.path = get_absolute_path_with_buffer(params.current_task, pathbuf, RS_BUFFER_SIZE);
			if (params.path) {
				int allowed;
				char *ptr;

				ptr = strrchr(params.path, '/');

			#if defined(CONFIG_MOUNT_RESTRICT) && defined(CONFIG_DO_RS_JOURNAL)
				if (op_type == ovMount) {
					if (rs_get_boot_mode() == RS_NORMAL_BOOT) {
						mount_verify_data *verify_data = (mount_verify_data *)op_data;

						if ((verify_data) && (verify_data->mount_type == rmtSystem)) {
							if (verify_data->task_infos) {
								fill_mount_verify_task_info(verify_data, params.current_task,
									params.path, pathbuf, ((ptr > params.path) ? (ptr + 1) : params.path),
									fptUnknown);
							}
						}
					}
				}
			#endif

				if (ptr <= params.path) {
					/*"/' OK*/
					allowed = 1;
				} else {
					noused int need_further_check;
					noused int need_upper_tasks;
					noused int i;

					ptr++;

					params.buffer = pathbuf + RS_BUFFER_SIZE;
					params.buffer_size = RS_BUFFER_SIZE;
					params.op_type = op_type;
					params.op_data = op_data;
					params.is_query = 1;
					params.head_ptr = NULL;
					/*allowed = 1;*/

					RS_LOG("path,%s\n", params.path);

					/*第一轮，查询是否需要获取upper_tasks*/
					need_further_check = 0;
					need_upper_tasks = 0;
#if 1

					for (i = 0; i < ARRAY_SIZE(mnt_allowed_patterns); i++) {
						if ((!mnt_allowed_patterns[i].pattern) || (!mnt_allowed_patterns[i].pattern[0])) {
							continue;
						}

						if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_FULL_COMPARE) {
							/*pattern is file*/
							if (STRCMP(params.path, mnt_allowed_patterns[i].pattern, i) == 0) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) > 0)) {
									need_upper_tasks = 1;
								}
								break;
							}
						} else if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_LEADING_PART_COMPARE) {
							if (STRNCMP(params.path, mnt_allowed_patterns[i].pattern,
								mnt_allowed_patterns[i].pattern_len, i) == 0) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) > 0)) {
									need_upper_tasks = 1;
									break;
								}

								if (!mnt_allowed_patterns[i].partial_match_continue)
									break;
							}
						} else if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_NAME_PART_COMPARE) {
							/*pattern is file name part*/
							if (STRISTR(ptr, mnt_allowed_patterns[i].pattern, i)) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) > 0)) {
									need_upper_tasks = 1;
								}

								if (!mnt_allowed_patterns[i].partial_match_continue)
									break;
							}
						}
					}

					if ((!need_upper_tasks) && (need_further_check)) {
						for (i = 0; i < ARRAY_SIZE(mnt_disallowed_patterns); i++) {
							if ((!mnt_disallowed_patterns[i].pattern) || (!mnt_disallowed_patterns[i].pattern[0])) {
								continue;
							}

							if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_FULL_COMPARE) {
								/*pattern is file*/
								if (STRCMP(params.path, mnt_disallowed_patterns[i].pattern, i) == 0) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) > 0)) {
										need_upper_tasks = 1;
									}
									break;
								}
							} else if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_LEADING_PART_COMPARE) {
								if (STRNCMP(params.path, mnt_disallowed_patterns[i].pattern,
									mnt_disallowed_patterns[i].pattern_len, i) == 0) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) > 0)) {
										need_upper_tasks = 1;
										break;
									}

									if (!mnt_disallowed_patterns[i].partial_match_continue)
										break;
								}
							} else if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_NAME_PART_COMPARE) {
								/*pattern is file name part*/
								if (STRISTR(ptr, mnt_disallowed_patterns[i].pattern, i)) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) > 0)) {
										need_upper_tasks = 1;
									}

									if (!mnt_disallowed_patterns[i].partial_match_continue)
										break;
								}
							}
						}
					}
#endif

#if 1
					RS_LOG("need upper:%d\n", need_upper_tasks);

					if (need_upper_tasks) {
						RS_LOG("upper\n");

						if (get_op_check_params_upper_tasks(&params, params.current_task)) {
							RS_LOG("upper err\n");

							retval = -EPERM;
							goto done;
						}
					}
/*///////////////////////////////////////////////////*/

#endif
#if 1
					params.is_query = 0;
					allowed = 0;
					need_further_check = 0;

					for (i = 0; i < ARRAY_SIZE(mnt_allowed_patterns); i++) {
						if ((!mnt_allowed_patterns[i].pattern) || (!mnt_allowed_patterns[i].pattern[0])) {
							continue;
						}

						if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_FULL_COMPARE) {
							/*pattern is file*/
							if (STRCMP(params.path, mnt_allowed_patterns[i].pattern, i) == 0) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) == 0)) {
									/*allowed = 0;*/
								} else {
									allowed = 1;
								}
								break;
							}
						} else if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_LEADING_PART_COMPARE) {
							if (STRNCMP(params.path, mnt_allowed_patterns[i].pattern,
								mnt_allowed_patterns[i].pattern_len, i) == 0) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) == 0)) {
									allowed = 0;
								} else {
									allowed = 1;
								}

								if (!mnt_allowed_patterns[i].partial_match_continue)
									break;
								/*break;*/
							}
						} else if (mnt_allowed_patterns[i].compare_flags & OP_CHECK_NAME_PART_COMPARE) {
							/*pattern is file name part*/
							if (STRISTR(ptr, mnt_allowed_patterns[i].pattern, i)) {
								need_further_check |= mnt_allowed_patterns[i].further_check;

								if ((mnt_allowed_patterns[i].check_func)
									&& (mnt_allowed_patterns[i].check_func(&params) == 0)) {
									allowed = 0;
								} else {
									allowed = 1;
								}

								if (!mnt_allowed_patterns[i].partial_match_continue)
									break;
							}
						}

					}

					if ((allowed) && (need_further_check)) {
						for (i = 0; i < ARRAY_SIZE(mnt_disallowed_patterns); i++) {
							if ((!mnt_disallowed_patterns[i].pattern) || (!mnt_disallowed_patterns[i].pattern[0])) {
								continue;
							}

							if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_FULL_COMPARE) {
								/*pattern is file*/
								if (STRCMP(params.path, mnt_disallowed_patterns[i].pattern, i) == 0) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) == 0)) {
										/*allowed = 1;*/
									} else {
										allowed = 0;
									}
									break;
								}
							} else if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_LEADING_PART_COMPARE) {
								if (STRNCMP(params.path, mnt_disallowed_patterns[i].pattern, mnt_disallowed_patterns[i].pattern_len, i) == 0) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) == 0)) {
										allowed = 1;
									} else {
										allowed = 0;
									}

									if (!mnt_disallowed_patterns[i].partial_match_continue)
										break;

									/*break;*/
								}
							} else if (mnt_disallowed_patterns[i].compare_flags & OP_CHECK_NAME_PART_COMPARE) {
								/*pattern is file name part*/
								if (STRISTR(ptr, mnt_disallowed_patterns[i].pattern, i)) {
									if ((mnt_disallowed_patterns[i].check_func)
										&& (mnt_disallowed_patterns[i].check_func(&params) == 0)) {
										allowed = 1;
									} else {
										allowed = 0;
									}

									if (!mnt_disallowed_patterns[i].partial_match_continue)
										break;

									/*break;*/
								}
							}

						}
					}

					/*allowed = 1;*/

					RS_LOG("allow:%d, %d(%s)\n", allowed, params.current_task->pid, params.current_task->comm);

					if (!allowed) {
						retval = -EPERM; /*-EAGAIN;*/ /*EOPNOTSUPP;*/
					}

#endif
					free_op_check_params_upper_tasks(&params);
				}
			} else {
				/*RS_LOG("bad: no exe path\n");*/
				retval = -EPERM;
			}

		done:

			RS_FREE_TWO_BUFFER(pathbuf);

			if (retval == 0
			#if defined(CONFIG_SOCK_RESTRICT) && defined(SOCK_CHECK_ALLOW_NON_BUILT_IN_PROCESS)
				&& (op_type != ovSockCheck)
			#endif
			#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
				&& (op_type != ovLoadSELinuxPolicy)
			#endif
				) {
				if (task_maybe_injected(params.current_task)) {
					retval = -EPERM;
				}
			}
		}

	}

	return retval;
}

RS_DEFINE_FC_END(do_op_verify);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_fd_absolute_path RS_HIDE(_s5)
#endif
noused notrace static noinline char *get_fd_absolute_path(int fd, char **buffer_ptr)
{
	char *buffer;
	char *ret_ptr = NULL;
	struct file *file;
	extern struct file *fget(unsigned int fd);
	extern void fput(struct file *);

	if ((fd < 0) || (buffer_ptr == NULL)) {
		return NULL;
	}

	buffer = (char *)RS_GET_BUFFER();
	if (!buffer) {
		*buffer_ptr = NULL;
		return NULL;
	}

	file = fget((unsigned int)fd);

	if (file) {
		/*buffer[RS_BUFFER_SIZE - 1] = '\0';*/

		ret_ptr = D_PATH_EX(&file->f_path, buffer, RS_BUFFER_SIZE);

		fput(file);
	}

	if (!ret_ptr) {
		RS_FREE_BUFFER(buffer);
		buffer = NULL;
	}

	*buffer_ptr = buffer;

	return ret_ptr;
}

#if defined(CONFIG_INSMOD_RESTRICT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define insmod_check_path RS_HIDE(_s9)
#endif
RS_DEFINE_FC_BEGIN(insmod_check_path);
RS_FORWARD_FC_END(insmod_check_path);

noused notrace static noinline int insmod_check_path(const char *file_path)
{
	int ret;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if ((STRNCMP(file_path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
		/*|| (STRNCMP(file_path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)*/
		|| (STRNCMP(file_path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
		|| (STRNCMP(file_path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
	#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
		|| (STRNCMP(file_path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
	#endif
		) {
		/*OK*/
		ret = 0;
	} else {
		RS_LOG("bad module\n");

	#if defined(PASS_INSMOD_RESTRICT)
		ret = 0;
	#else
		if (is_op_bypassed(ovInsmod)) {
			if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed)) {
				ret = 0;
				goto out;
			} else {
				ret = -EPERM;
				goto out_checked;
			}
		} else {
			ret = -EPERM;
		}
	#endif
	}

out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(insmod_check_path), fcInsmodCheckPath))) {
		ret = -EPERM;
	}
out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
}
#endif

#if defined(CONFIG_INSMOD_RESTRICT)
RS_DEFINE_FC_BEGIN(do_insmod_check);
RS_FORWARD_FC_END(do_insmod_check);
#endif

noused notrace
noinline int do_insmod_check(int fd)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_INSMOD_RESTRICT)
	int ret;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if (is_op_permitted(ovInsmod)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

#if defined(__NR_finit_module)
	if (fd >= 0) {
		char *path, *pathbuf;

		path = get_fd_absolute_path(fd, &pathbuf);

		if (path) {
			ret = insmod_check_path(path);
			if (pathbuf) {
				free_path(pathbuf);
			}

			if (ret) {
				goto out;
			} else if (rs_verify_func(RS_FC_PARAMS(insmod_check_path), fcInsmodCheckPath)) {
				ret = -EPERM;
				goto out_checked;
			}
		}
	}
#endif

	/*sys_init_moudle not called in kernel space*/
	ret = do_op_verify(ovInsmod, 0);
	if (ret < 0) {
		RS_LOG("Restricted load module. PID = %d(%s)\n", current->pid, current->comm);

	#if defined(PASS_INSMOD_RESTRICT)
		ret = 0;
	#else
		if ((is_op_bypassed(ovInsmod))
			&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))) {
			ret = 0;
		}
	#endif
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	}

out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_insmod_check), fcDoInsmodCheck))) {
		ret = -EPERM;
	}
out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_INSMOD_RESTRICT)
RS_DEFINE_FC_END(do_insmod_check);
#endif

#if defined(CONFIG_CHROOT_RESTRICT)
RS_DEFINE_FC_BEGIN(do_chroot_check);
RS_FORWARD_FC_END(do_chroot_check);
#endif

noused notrace
noinline int do_chroot_check(void)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_CHROOT_RESTRICT)
	int ret;
	struct task_struct *curr;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if (is_op_permitted(ovChRoot)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	/*sys_chroot called in kernel space*/
	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		ret = 0;
		goto out;
	}

	ret = do_op_verify(ovChRoot, 0);
	if (ret < 0) {
		RS_LOG("Restricted chroot. PID = %d(%s)\n", curr->pid, curr->comm);

	#if defined(PASS_CHROOT_RESTRICT)
		ret = 0;
	#else
		if ((is_op_bypassed(ovChRoot))
			&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))) {
			ret = 0;
		}
	#endif
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	}

out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_chroot_check), fcDoChrootCheck))) {
		ret = -EPERM;
	}

out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_CHROOT_RESTRICT)
RS_DEFINE_FC_END(do_chroot_check);
#endif

noused notrace
noinline int do_pivotroot_check(void)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_PIVOTROOT_RESTRICT)
	int ret;

	/* sys_pivot_root not called in kernel space */
	if (!(is_op_permitted(ovPivotRoot)))
		error = do_op_verify(ovPivotRoot, 0);
	else if ((rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted))
		|| (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)))
			error = -EPERM;

	if (error) {
		RS_LOG("Restricted pivot. PID = %d(%s)/n", current->pid, current->comm);

	#if defined(PASS_PIVOTROOT_RESTRICT)
		error = 0;
	#else
		if ((is_op_bypassed(ovPivotRoot))
			&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
			)
			error = 0;
		else
			goto out;
	#endif
	}

out:
	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_MOUNT_RESTRICT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_check RS_HIDE(_yb)
#endif

RS_FORWARD_FC_BEGIN(do_mount_check);

struct tag_mount_check_info;

static int do_mount_check(const char *dev_name, const char *dir_name, unsigned long flags, int *mnt_flags,
	struct tag_mount_check_info *check_info);

RS_FORWARD_FC_END(do_mount_check);

RS_FORWARD_FC_BEGIN_PUB(do_mount);
RS_FORWARD_FC_END_PUB(do_mount);
#endif

enum {
	adoDumcharOpenWrite,
	adoDumcharMemErase,
	adoBlockDevOpenWrite,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_access_dev_check RS_HIDE(_yd)
#endif

#if defined(CONFIG_ACCESS_DEV_RESTRICT)
RS_DEFINE_FC_BEGIN(do_access_dev_check);
RS_FORWARD_FC_END(do_access_dev_check);
#endif

RS_DEFINE_FC_BODY(do_access_dev_check) \
noused notrace
static noinline int do_access_dev_check(const char *part_name, int op, int is_write)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_ACCESS_DEV_RESTRICT)
	int ret;
	struct task_struct *curr;
	noused unsigned long *bitmap_buffer;
#if defined(RS_AB_UPDATE_SUPPORT)
	int part_name_len;
	bool is_ab_part;
#endif
	init_fc_bitmap(&bitmap_buffer);

#if 0
	if (is_op_permitted(ovAccessDev)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}
#endif

	curr = current;

	/*called in kernel space*/
	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		ret = 0;
		goto out;
	}

#if defined(RS_AB_UPDATE_SUPPORT)
	is_ab_part = is_ab_part_name(part_name, &part_name_len);
#endif

	if (
	#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	#if defined(RS_AB_UPDATE_SUPPORT)
		(
		((is_ab_part) &&
		((!STRNICMP_EX(part_name, check_strings[csRecovery].str, (sizeof("/sbin/") - 1), part_name_len, csRecovery))
		|| (!STRNICMP_EX(part_name, check_strings[csSystem].str, 1, part_name_len, csSystem))
		|| (!STRNICMP(part_name, check_strings[csBoot].str, part_name_len, csBoot))
		|| (!STRNICMP(part_name, check_strings[csLK].str, part_name_len, csLK))
	#if defined(RS_HAS_OEM_PARTITION)
		|| (!STRNICMP_EX(part_name, check_strings[csOemPartName].str, 1, part_name_len, csOemPartName))
	#endif
		)
		)
		|| ((!is_ab_part) &&
		((!STRICMP_EX(part_name, check_strings[csRecovery].str, (sizeof("/sbin/") - 1), csRecovery))
		|| (!STRICMP_EX(part_name, check_strings[csSystem].str, 1, csSystem))
		|| (!STRICMP(part_name, check_strings[csBoot].str, csBoot))
		|| (!STRICMP(part_name, check_strings[csLK].str, csLK))
	#if defined(RS_HAS_OEM_PARTITION)
		|| (!STRICMP_EX(part_name, check_strings[csOemPartName].str, 1, csOemPartName))
	#endif
		)
		)
		)
	#else /*RS_AB_UPDATE_SUPPORT*/
		(!STRICMP_EX(part_name, check_strings[csRecovery].str, (sizeof("/sbin/") - 1), csRecovery))
		|| (!STRICMP_EX(part_name, check_strings[csSystem].str, 1, csSystem))
		|| (!STRICMP(part_name, check_strings[csBoot].str, csBoot))
		|| (!STRICMP(part_name, check_strings[csLK].str, csLK))
	#if defined(RS_HAS_OEM_PARTITION)
		|| (!STRICMP_EX(part_name, check_strings[csOemPartName].str, 1, csOemPartName))
	#endif
	#endif /*!RS_AB_UPDATE_SUPPORT*/
	#elif defined(RS_HAS_DEV_SHORTCUTS)
		((op < adoBlockDevOpenWrite)
		&& ((!STRICMP_EX(part_name, check_strings[csRecovery].str, (sizeof("/sbin/") - 1), csRecovery))
		|| (!STRICMP(part_name, check_strings[csAndroid].str, csAndroid))
		|| (!STRICMP(part_name, check_strings[csBootImg].str, csBootImg))
		|| (!STRICMP(part_name, check_strings[csUBoot].str, csUBoot))))
	#else
		(0)
	#endif
	/*#if defined(RS_HAS_EMMC_SHORTCUTS)*/
		|| (
	#if defined(RS_HAS_DEV_SHORTCUTS)
		(op == adoBlockDevOpenWrite) &&
	#endif
		(rs_is_need_check_part(part_name)))
	/*#endif*/
		) {
		/*RS_LOG("w rec,%s,%d\n", part_name, op);*/

		if (is_op_permitted(ovAccessDev)) {
			if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
				ret = 0;
				goto out;
			} else {
				ret = -EPERM;
				goto out_checked;
			}
		}

		ret = do_op_verify(ovAccessDev, 0);
		if (ret < 0) {
			RS_LOG("Restricted access device. PID = %d(%s)\n", curr->pid, curr->comm);

		#if defined(PASS_ACCESS_DEV_RESTRICT)
			ret = 0;
		#else
			if ((is_op_bypassed(ovAccessDev))
				&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))) {
				ret = 0;
			}
		#endif
		} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
			ret = -EPERM;
		}

		if (!ret) {
		#if 1
			if ((rs_verify_func(RS_FC_PARAMS(do_access_dev_check), fcDoAccessDevCheck))
			#if defined(CONFIG_MOUNT_RESTRICT)
				|| (rs_verify_func(RS_FC_PARAMS(do_mount_check), fcDoMountCheck))
				|| (rs_verify_func(RS_FC_PARAMS_PUB(do_mount), fcDoMount))
			#endif
				) {
				ret = -EPERM;
			}
		#endif
		}
	} else {
		ret = 0;
	}

out:
out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_ACCESS_DEV_RESTRICT)
RS_DEFINE_FC_END(do_access_dev_check);
#endif

#if defined(CONFIG_SECURITY_SELINUX)

#if defined(CONFIG_DISABLE_SELINUX_RESTRICT)
RS_DEFINE_FC_BEGIN(do_disable_selinux_check);
RS_FORWARD_FC_END(do_disable_selinux_check);
#endif

noused notrace
noinline int do_disable_selinux_check(void)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_DISABLE_SELINUX_RESTRICT) && defined(CONFIG_SECURITY_SELINUX_DEVELOP)
	int ret;
	struct task_struct *curr;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if (is_op_permitted(ovDisableSELinux)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	/*called in kernel space*/
	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		ret = 0;
		goto out;
	}

	RS_LOG("off selinux\n");

	ret = do_op_verify(ovDisableSELinux, 0);
	if (ret < 0) {
		RS_LOG("Restricted disable selinux. PID = %d(%s)\n", curr->pid, curr->comm);

	#if defined(PASS_DISABLE_SELINUX_RESTRICT)
		ret = 0;
	#else
		if ((is_op_bypassed(ovDisableSELinux))
			&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))) {
			ret = 0;
		}
	#endif
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	}

out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_disable_selinux_check), fcDoDisableSelinuxCheck))) {
		ret = -EPERM;
	}

out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_DISABLE_SELINUX_RESTRICT)
RS_DEFINE_FC_END(do_disable_selinux_check);
#endif

#else /*CONFIG_SECURITY_SELINUX*/
noused notrace
noinline int do_disable_selinux_check(void)
{
	return 0;
}
#endif /*!CONFIG_SECURITY_SELINUX*/


#if defined(CONFIG_SECURITY_SELINUX)

#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
RS_DEFINE_FC_BEGIN(do_load_selinux_policy_check);
RS_FORWARD_FC_END(do_load_selinux_policy_check);
#endif

RS_DEFINE_FC_BODY(do_load_selinux_policy_check) \
noused notrace
noinline int do_load_selinux_policy_check(void)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
	int ret;
	struct task_struct *curr;
	noused intptr_t policy_loaded;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if (is_op_permitted(ovLoadSELinuxPolicy)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	/*called in kernel space*/
	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		ret = 0;
		goto out;
	}

	RS_LOG("selinux ld ply\n");

	/*only allow load selinux policy once, "setprop selinux.reload_policy 1" won't work too*/
	if (get_vars_global_flag(gfSelinuxPolicyLoaded, &policy_loaded)) {
		RS_LOG("lspc,1\n");
		ret = -EPERM;
	} else if (policy_loaded) {
	#if defined(CONFIG_EXEC_SU_RESTRICT) || defined(RS_VIVOROOT_SUPPORT)
		noused intptr_t flag;
	#endif

		RS_LOG("lspc,2\n");
	#if defined(CONFIG_EXEC_SU_RESTRICT)
		if ((!get_vars_global_flag(gfAllowSuExec, &flag)) && (flag))
			ret = 0;
		else
	#endif
	#if defined(RS_VIVOROOT_SUPPORT)
		if ((!get_vars_global_flag(gfVivoRootTriggered, &flag)) && (flag))
			ret = 0;
		else
	#endif
			ret = -EPERM;
	} else {
		RS_LOG("lspc,3\n");

		set_vars_global_flag(gfSelinuxPolicyLoaded, 1);
		ret = 0;
	}

	if (!ret)
		ret = do_op_verify(ovLoadSELinuxPolicy, 0);

	if (ret < 0) {
		RS_LOG("Restricted selinux load policy. PID = %d(%s)\n", curr->pid, curr->comm);

	#if defined(PASS_LOAD_SELINUX_POLICY_RESTRICT)
		ret = 0;
	#else
		if ((is_op_bypassed(ovLoadSELinuxPolicy))
			&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))) {
			ret = 0;
		}
	#endif
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	}

out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_load_selinux_policy_check), fcDoLoadSelinuxPolicyCheck))) {
		ret = -EPERM;
	}

out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else /*CONFIG_LOAD_SELINUX_POLICY_RESTRICT*/
	return 0;
#endif /*!CONFIG_LOAD_SELINUX_POLICY_RESTRICT*/
}

#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
RS_DEFINE_FC_END(do_load_selinux_policy_check);
#endif

#else /*CONFIG_SECURITY_SELINUX*/
noused notrace
noinline int do_load_selinux_policy_check(void)
{
	return 0;
}
#endif /*!CONFIG_SECURITY_SELINUX*/

#if defined(CONFIG_PTRACE_RESTRICT)
RS_DEFINE_FC_BEGIN(do_ptrace_check);
RS_FORWARD_FC_END(do_ptrace_check);
#endif

noused notrace
noinline int do_ptrace_check(struct task_struct *task)
{
	/*pass: 0; fail: < 0*/
#if defined(CONFIG_PTRACE_RESTRICT)
	int ret = 0;
	noused unsigned long *bitmap_buffer;
	init_fc_bitmap(&bitmap_buffer);

	if (is_op_permitted(ovPtrace)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	if (task) {
		/*whether task is kthread is already checked in caller, eg. ptrace_attach()*/
		/*RS_LOG("pt,0,%s\n", task->comm);*/

		if (task->pid != task->tgid) {
			task = task->group_leader;

			if (!task) {
				RS_LOG("pt,nil\n");
				goto out; /*return ret;*/
			}
		}

		get_task_struct(task);

		RS_LOG("pt,1,%s\n", task->comm);

	#if defined(RS_64BIT_SUPPORT)
		if (STRNCMP(task->comm, check_strings[csZygote].str, check_strings[csZygote].str_len, csZygote) == 0)
	#else
		if (STRCMP(task->comm, check_strings[csZygote].str, csZygote) == 0)
	#endif
		{
			RS_LOG("pt zg 0\n");

			if (!task_is_not_root_privilege(task)) {
				cast_kernel_cap_t pEffective, pInheritable, pPermitted;
				int res;

				res = RS_CAP_GET(task, &pEffective.cap, &pInheritable.cap, &pPermitted.cap);
				if (!res) {
					/*RS_LOG("pt,zg,1,%u,%u\n", pEffective.cap.cap[0], pEffective.cap.cap[1]);*/

					if (((pEffective.val & ZYGOTE_CAP_MASK) == ZYGOTE_CAP_MASK)
						&& (pPermitted.val == pEffective.val)) {
						char *pathbuf, *path;
						path = get_absolute_path(task, &pathbuf);

						if (path) {
						#if defined(RS_64BIT_SUPPORT)
							if (STRNCMP(path, check_strings[csAppProcess].str, check_strings[csAppProcess].str_len, csAppProcess) == 0)
						#else
							if (STRCMP(path, check_strings[csAppProcess].str, csAppProcess) == 0)
						#endif
							{
								RS_LOG("pt,zg 2\n");

								ret = -EPERM;
							}

							free_path(pathbuf);
						} else {
							RS_LOG("pt,zg 3\n");
							ret = -EPERM;
						}
					}
				}
			}
		} else if (STRCMP(task->comm, check_strings[csSystemServer].str, csSystemServer) == 0) {
			const struct cred *cred;

			RS_LOG("pt ss 0\n");

			cred = get_task_cred(task);

			if (cred) {
				if (CHECK_UID_BY_CRED(cred, AID_SYSTEM)) {
					noused cast_kernel_cap_t pEffective, pInheritable, pPermitted;
					noused int res;

#if 1
					/*read_lock(&tasklist_lock);*/
					res = RS_CAP_GET(task, &pEffective.cap, &pInheritable.cap, &pPermitted.cap);
					/*read_unlock(&tasklist_lock);*/

					if ((!res) && ((SYSTEM_SERVER_CAP_VALUE == pEffective.val)
					#if defined(CAP_BLOCK_SUSPEND) /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)*/
						|| ((SYSTEM_SERVER_CAP_VALUE2 & pEffective.val) == SYSTEM_SERVER_CAP_VALUE2)
					#endif
						)
						&& (pEffective.val == pPermitted.val))
#endif
					{
						char *pathbuf, *path;

						/*RS_LOG("pt ss 1,%u,%u\n", pEffective.cap[0], pEffective.cap[1]);*/

						path = get_absolute_path(task, &pathbuf);

						if (path) {
						#if defined(RS_64BIT_SUPPORT)
							if (STRNCMP(path, check_strings[csAppProcess].str, check_strings[csAppProcess].str_len, csAppProcess) == 0)
						#else
							if (STRCMP(path, check_strings[csAppProcess].str, csAppProcess) == 0)
						#endif
							{
								RS_LOG("pt ss 1\n");

								ret = -EPERM;
							}

							free_path(pathbuf);
						} else {
							RS_LOG("pt ss 2\n");
							ret = -EPERM;
						}
					}
				}

				put_cred(cred);
			}
		} else if ((STRCMP(task->comm, check_strings[csAdbdPart].str, csAdbdPart) == 0)
		#if defined(RS_IS_ANDROID_11_ABOVE)
			|| (STRCMP(task->comm, check_strings[csVadbdPart].str, csVadbdPart) == 0)
			|| (STRCMP(task->comm, check_strings[csVusbdPart].str, csVusbdPart) == 0)
		#endif
			)
		{
			char *pathbuf, *path;

			RS_LOG("pt adbd 0\n");

			path = get_absolute_path(task, &pathbuf);

			if (path) {
				if ((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
				#if defined(RS_IS_ANDROID_11_ABOVE)
					|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
					|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
					|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
				#endif
					)
				{
					RS_LOG("pt adbd 1\n");

					ret = -EPERM;
				}

				free_path(pathbuf);
			} else {
				RS_LOG("pt adbd 2\n");
				ret = -EPERM;
			}
		} else {
			const struct cred *cred;

			RS_LOG("pt sys 0\n");

			cred = get_task_cred(task);

			if (cred) {
				int need_check = 0;

				if (CHECK_UID_BY_CRED(cred, AID_SYSTEM)) {
					need_check = 1;
				} else {
					struct group_info *group_info = cred->group_info;

					if (group_info) {
						get_group_info(group_info);
						if ((group_info->ngroups) && (groups_search(group_info, AID_SYSTEM))) {
							need_check = 1;
						}
						put_group_info(group_info);
					}
				}

				if (need_check) {
					noused char *pathbuf, *path;

					RS_LOG("pt sys 1\n");

				#if 1
					ret = -EPERM;
				#else
					path = get_absolute_path(task, &pathbuf);

					if (path) {
					#if defined(RS_64BIT_SUPPORT)
						if (STRNCMP(path, check_strings[csAppProcess].str, check_strings[csAppProcess].str_len, csAppProcess) == 0)
					#else
						if (STRCMP(path, check_strings[csAppProcess].str, csAppProcess) == 0)
					#endif
						{
							RS_LOG("pt sys 2\n");

							ret = -EPERM;
						}

						free_path(pathbuf);
					} else {
						RS_LOG("pt sys 3\n");
						ret = -EPERM;
					}
				#endif
				}

				put_cred(cred);
			}
		}

		put_task_struct(task);

		if (ret < 0) {
			char *pathbuf, *path;

			path = get_absolute_path(current, &pathbuf);

			if (path) {
				RS_LOG("pt path %s\n", path);

				if (
					(strrchr(path, '/') <= path)
					|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
					|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
					|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
					|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
				#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
					|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
				#endif
					) {
					/*allow be ptraced*/
					RS_LOG("pt OK\n");

					ret = 0;
				}

				free_path(pathbuf);
			}
		}

		if (ret < 0) {
			RS_LOG("Restricted ptrace attach. PID = %d(%s)\n", task->pid, task->comm);

		#if defined(PASS_PTRACE_RESTRICT)
			ret = 0;
		#else
			if (is_op_bypassed(ovPtrace)) {
				if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed)) {
					ret = 0;
					goto out;
				} else {
					ret = -EPERM;
					goto out_checked;
				}
			}
		#endif
		}
	}


out:
	if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_ptrace_check), fcDoPtraceCheck))) {
		ret = -EPERM;
	}
out_checked:
	done_fc_bitmap(bitmap_buffer);

	return ret;
#else
	return 0;
#endif
}
#if defined(CONFIG_PTRACE_RESTRICT)
RS_DEFINE_FC_END(do_ptrace_check);
#endif

typedef struct tag_mount_dir_info {
	int seed;
	int cmp_len;
} mount_dir_info;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_mount_dir_infos RS_HIDE(_yb_)
#endif

noused static const mount_dir_info g_mount_dir_infos[rmtInvalid] = {
	{csSystem, sizeof("system") - 1},
	{csSlash, 0}, /*rmtRoot,*/
	{csApps, sizeof("apps") - 1},
	{csTmp, sizeof("tmp") - 1},
	{csOem, sizeof("oem") - 1},
	{csVendorSlash, sizeof("vendor") - 1},
};


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_mount_dir_need_check RS_HIDE(_t0)
#endif

RS_DEFINE_FC_BEGIN(is_mount_dir_need_check);

noused notrace RS_DEFINE_FC_BODY(is_mount_dir_need_check) \
static noinline int is_mount_dir_need_check(const char *dir_name, int *mount_type_ptr)
{
	int need_check = 0;
	int mount_type = rmtInvalid;
	char ch;

	if ((!dir_name) || (!dir_name[0])) {
		goto got_mount_type;
	}

	/*RS_LOG("dm,0,%s\n", dir_name);*/

	ch = *dir_name;

	if (ch == '/') {
		ch = *(++dir_name);
	}

	if (!ch) {
		mount_type = rmtRoot;
		need_check = 1;
	} else {
		switch (ch) {
		case 'a': {
			mount_type = rmtApps;
		}
		break;
	#if defined(RS_HAS_OEM_PARTITION)
		case 'o': {
			mount_type = rmtOem;
		}
		break;
	#endif
		case 's': {
			mount_type = rmtSystem;
		}
		break;
		case 't': {
			mount_type = rmtTmp;
		}
		break;
		case 'v': {
			mount_type = rmtVendor;
		}
		break;
		default: {
			mount_type = rmtInvalid;
			break;
		}
		}

		if (mount_type < rmtInvalid) {
			const mount_dir_info *info = &g_mount_dir_infos[mount_type];
			int dst_seed = info->seed;
			int dst_cmp_len = info->cmp_len;

			if ((STRNCMP_EX(dir_name + 1, check_strings[dst_seed].str, 2, dst_cmp_len - 1, dst_seed) == 0)
					&& (dir_name[dst_cmp_len] == '\0' || (dir_name[dst_cmp_len] == '/'
				#if defined(RS_MOUNT_DIR_CHECK_TOP_LEVEL_ONLY)
					&& dir_name[dst_cmp_len + 1] == '\0'
				#endif
				))) {
				RS_LOG("dm,5\n");
				need_check = 1;
			} else {
				mount_type = rmtInvalid;
			}
		}
	}

got_mount_type:
	if (mount_type_ptr) {
		*mount_type_ptr = mount_type;
	}

	return need_check;
}

RS_DEFINE_FC_END(is_mount_dir_need_check);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_mount_dev_need_check RS_HIDE(_t1)
#endif

RS_DEFINE_FC_BEGIN(is_mount_dev_need_check);

noused notrace RS_DEFINE_FC_BODY(is_mount_dev_need_check) \
static noinline int is_mount_dev_need_check(const char *dev_name, int *mount_type_ptr)
{
	int need_check = 0;
	int mount_type = rmtInvalid;

	if ((!dev_name) || (!dev_name[0])) {
		goto got_mount_type;
	}

	/*RS_LOG("dm,0.3,%s\n", dev_name);*/

	if (STRCMP(check_strings[csRootFS].str, dev_name, csRootFS) == 0) {
		need_check = 1;
		mount_type = rmtRoot;
	}
#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
	else if (STRCMP(check_strings[csSplashRoot].str, dev_name, csSplashRoot) == 0) {
		need_check = 1;
		mount_type = rmtSystem;
	}
#endif
	else {
		char *main_dev;
		bool flag;
	#if defined(RS_AB_UPDATE_SUPPORT)
		bool is_ab_part;
		int cmp_len;
	#endif

	#if defined(RS_HAS_EMMC_SHORTCUTS)
		/* "/emmc@android" / "emmc@android"*/
		/* "/dev/apps"、"/dev/android"*/
		/* "/dev/block/platform/mtk-msdc.0/xxx"*/
		RS_LOG("dm,2,%s\n", dev_name);
		main_dev = strrchr(dev_name, '@');
		if (main_dev) {
		/*	RS_LOG("dm,2.1,%d,%d\n", (STRNCMP(dev_name, check_strings[csEmmc].str, check_strings[csEmmc].str_len, csEmmc) == 0), (main_dev - (sizeof("/emmc") - 1) == dev_name));*/
		}

		if ((main_dev) && (((main_dev - (sizeof("/emmc") - 1) == dev_name)
			&& (STRNCMP(dev_name, check_strings[csEmmc].str, check_strings[csEmmc].str_len, csEmmc) == 0))
			|| ((main_dev - (sizeof("emmc") - 1) == dev_name)
			&& (STRNCMP_EX(dev_name, check_strings[csEmmc].str, 1, check_strings[csEmmc].str_len - 1, csEmmc) == 0)))
			) {
			main_dev++;
			RS_LOG("dm,2.2,%s\n", main_dev);
			if ((STRCMP(main_dev, check_strings[csAndroid].str, csAndroid) == 0)
				|| (STRCMP_EX(main_dev, check_strings[csApps].str, 1, csApps) == 0)
			#if defined(RS_HAS_OEM_PARTITION)
				|| (STRCMP_EX(main_dev, check_strings[csOemPartName].str, 1, csOemPartName) == 0)
			#endif
			#if defined(RS_HAS_VENDOR_PARTITION)
				|| (STRCMP_EX(main_dev, check_strings[csVendor].str, 1, csVendor) == 0)
			#endif
				) {
				RS_LOG("dm,2.3\n");
				need_check = 1;

				if (main_dev[1] == 'n')
					mount_type = rmtSystem;
				else if (main_dev[1] == 'p')
					mount_type = rmtApps;
			#if defined(RS_HAS_VENDOR_PARTITION)
				else if (main_dev[0] == 'v')
					mount_type = rmtVendor;
			#endif
			#if defined(RS_HAS_OEM_PARTITION)
				else /*if (main_dev[1] == 'e')*/
					mount_type = rmtOem;
			#endif

				goto got_mount_type;
			}
		}
	#endif
		main_dev = strrchr(dev_name, '/');

	#if defined(RS_HAS_DEV_SHORTCUTS)
		if ((main_dev) && (((main_dev == (dev_name + sizeof("/dev") - 1))
			&& (STRNCMP(dev_name, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) == 0))
			|| ((main_dev == (dev_name + sizeof("dev") - 1))
			&& (STRNCMP_EX(dev_name, check_strings[csDevSlash].str, 1, check_strings[csDevSlash].str_len - 1, csDevSlash) == 0)))
			) {
			main_dev++;
			RS_LOG("dm,2.4,%s\n", main_dev);
			if ((STRCMP(main_dev, check_strings[csAndroid].str, csAndroid) == 0)
				|| (STRCMP_EX(main_dev, check_strings[csApps].str, 1, csApps) == 0)
			#if defined(RS_HAS_OEM_PARTITION)
				|| (STRCMP_EX(main_dev, check_strings[csOemPartName].str, 1, csOemPartName) == 0)
			#endif
			#if defined(RS_HAS_VENDOR_PARTITION)
				|| (STRCMP_EX(main_dev, check_strings[csVendor].str, 1, csVendor) == 0)
			#endif
				) {
				RS_LOG("dm,2.5\n");
				need_check = 1;

				if (main_dev[1] == 'n')
					mount_type = rmtSystem;
				else if (main_dev[1] == 'p')
					mount_type = rmtApps;
			#if defined(RS_HAS_VENDOR_PARTITION)
				else if (main_dev[0] == 'v')
					mount_type = rmtVendor;
			#endif
			#if defined(RS_HAS_OEM_PARTITION)
				else /*if (main_dev[1] == 'e')*/
					mount_type = rmtOem;
			#endif

				goto got_mount_type;
			}
		}
	#endif

		/* "/dev/block/platform/msm_sdcc.1/by-name/system"*/
		flag = false;
		if ((main_dev) && ((STRNCMP(dev_name, check_strings[csDevBlockSlash].str, check_strings[csDevBlockSlash].str_len, csDevBlockSlash) == 0)
			|| (STRNCMP_EX(dev_name, check_strings[csDevBlockSlash].str, 1, check_strings[csDevBlockSlash].str_len - 1, csDevBlockSlash) == 0))) {
		#if defined(RS_AB_UPDATE_SUPPORT)
			is_ab_part = is_ab_part_name(main_dev, &cmp_len);
			if (is_ab_part) {
				if ((STRNCMP(main_dev, check_strings[csSystem].str, cmp_len, csSystem) == 0)
					|| (STRNCMP(main_dev, check_strings[csApps].str, cmp_len, csApps) == 0)
				#if defined(RS_HAS_OEM_PARTITION)
					|| (STRNCMP(main_dev, check_strings[csOemPartName].str, cmp_len, csOemPartName) == 0)
				#endif
				#if defined(RS_HAS_VENDOR_PARTITION)
					|| (STRNCMP(main_dev, check_strings[csVendor].str, cmp_len, csVendor) == 0)
				#endif
					)
					flag = true;
			} else {
				if ((STRCMP(main_dev, check_strings[csSystem].str, csSystem) == 0)
					|| (STRCMP(main_dev, check_strings[csApps].str, csApps) == 0)
				#if defined(RS_HAS_OEM_PARTITION)
					|| (STRCMP(main_dev, check_strings[csOemPartName].str, csOemPartName) == 0)
				#endif
				#if defined(RS_HAS_VENDOR_PARTITION)
					|| (STRCMP(main_dev, check_strings[csVendor].str, csVendor) == 0)
				#endif
					)
					flag = true;
			}
		#else
			if ((STRCMP(main_dev, check_strings[csSystem].str, csSystem) == 0)
				|| (STRCMP(main_dev, check_strings[csApps].str, csApps) == 0)
			#if defined(RS_HAS_OEM_PARTITION)
				|| (STRCMP(main_dev, check_strings[csOemPartName].str, csOemPartName) == 0)
			#endif
			#if defined(RS_HAS_VENDOR_PARTITION)
				|| (STRCMP(main_dev, check_strings[csVendor].str, csVendor) == 0)
			#endif
				)
				flag = true;
		#endif
		}

		if (flag) {
			/*if ((STRCMP(main_dev, check_strings[csSystem].str, csSystem) == 0)
				|| (STRCMP(main_dev, check_strings[csApps].str, csApps) == 0)
				)
			*/
			{
				RS_LOG("dm,3\n");
				need_check = 1;
				main_dev++;

				if (main_dev[0] == 's')
					mount_type = rmtSystem;
				else if (main_dev[0] == 'a')
					mount_type = rmtApps;
			#if defined(RS_HAS_VENDOR_PARTITION)
				else if (main_dev[0] == 'v')
					mount_type = rmtVendor;
			#endif
			#if defined(RS_HAS_OEM_PARTITION)
				else /*if (main_dev[0] == 'o')*/
					mount_type = rmtOem;
			#endif

				goto got_mount_type;
			}
		} else {
			/*dev_name 为 '/dev/block/mmcblk0pX' 的格式？也有可能root工具自己建了个指到真实分区设备的软链接*/
			/*
			if ((STRNCMP(dev_name, check_strings[csMmcblk0p].str, check_strings[csMmcblk0p].str_len, csMmcblk0p) == 0)
			#if defined(RS_MTK_PLATFORM)
				|| (STRNCMP(dev_name, check_strings[csMtkMsdc0].str, check_strings[csMtkMsdc0].str_len, csMtkMsdc0) == 0)
			#else
				|| (STRNCMP(dev_name, check_strings[csMsmSdcc1].str, check_strings[csMsmSdcc1].str_len, csMsmSdcc1) == 0)
			#endif
				)
			*/

			/*
			if ((STRNCMP(dev_name, check_strings[csDevBlockSlash].str, check_strings[csDevBlockSlash].str_len, csDevBlockSlash) == 0)
				|| (STRNCMP_EX(dev_name, check_strings[csDevBlockSlash].str, 1, check_strings[csDevBlockSlash].str_len - 1, csDevBlockSlash) == 0))
			*/
			{
				char *pathbuf;
				noused int pathlen;

				/*RS_LOG("dm,4\n");*/
				pathbuf = (char *)RS_GET_TWO_BUFFER();

				if (pathbuf) {
					const char *real_dev_name;
					noused int partition_seed;
					noused char *partition_dev = NULL;

				#if 0
					/*if (dev_name[sizeof("/dev/block/") - 1] == 'p')*/
					if ((STRNCMP(dev_name, check_strings[csMmcblk0p].str, check_strings[csMmcblk0p].str_len, csMmcblk0p) != 0)
						|| (STRNCMP_EX(dev_name, check_strings[csMmcblk0p].str, 1, check_strings[csMmcblk0p].str_len - 1, csMmcblk0p) != 0)) {
						/* "/dev/block/platform/..."*/
						RS_LOG("dm,5\n");
						real_dev_name = get_real_filename_with_buffer(dev_name, pathbuf + RS_BUFFER_SIZE, RS_BUFFER_SIZE);
						if (!real_dev_name) {
							real_dev_name = dev_name;
						}
					} else {
						real_dev_name = dev_name;
					}
				#else
					real_dev_name = try_get_real_filename_with_buffer(dev_name, pathbuf + RS_BUFFER_SIZE, RS_BUFFER_SIZE);
					if (!real_dev_name) {
						real_dev_name = dev_name;
					}
				#endif

			#if 1
				{
				#if defined(CONFIG_ROOT_RESTRICT)
					int part_type;

					if (rs_is_mount_need_check_part(real_dev_name, &part_type)) {
						if (part_type == ptSystem) {
							need_check = 1;
							mount_type = rmtSystem;
						} else if (part_type == ptApps) {
							need_check = 1;
							mount_type = rmtApps;
						}
					#if defined(RS_HAS_OEM_PARTITION)
						else if (part_type == ptOem) {
							need_check = 1;
							mount_type = rmtOem;
						}
					#endif
					#if defined(RS_HAS_VENDOR_PARTITION)
						else if (part_type == ptVendor) {
							need_check = 1;
							mount_type = rmtVendor;
						}
					#endif
					}
				#endif
				}
			#else

				#if defined(RS_HAS_EMMC_SHORTCUTS)
					partition_seed = csEmmcAndroid;

					#if defined(RS_ENCRYPT_STR)
						memcpy(pathbuf, check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
						simple_str_encrypt(pathbuf, check_strings[partition_seed].str_len, partition_seed);
						/*RS_LOG("dm,6.1,%s\n", pathbuf);*/
						partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
					#else
						partition_dev = get_real_filename_with_buffer(check_strings[partition_seed].str, pathbuf, RS_BUFFER_SIZE);
					#endif

				#endif

				#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
					if ((!partition_dev) && (g_block_platform_devname)) {
						partition_seed = csSystem; /*csMsmSystem;*/

						if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
							+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
							+ check_strings[partition_seed].str_len + 1)) {
							/*buffer overrun*/
							goto out_with_error; /*return -ENOBUFS;*/ /*返回非0值可能有问题，相当于挂载需要root相关检查*/
						}

						pathlen = check_strings[csBlkPlatformSlash].str_len;
						memcpy(pathbuf, check_strings[csBlkPlatformSlash].str, pathlen);
					#if defined(RS_ENCRYPT_STR)
						simple_str_encrypt(pathbuf, pathlen, csBlkPlatformSlash);
					#endif

						if (g_block_platform_devname_len) {
							memcpy(&pathbuf[pathlen], g_block_platform_devname, g_block_platform_devname_len);
							pathlen += g_block_platform_devname_len;
							memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(&pathbuf[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
						#endif
							pathlen += check_strings[csSlashByName].str_len;
						} else {
							memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt_ex(&pathbuf[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
						#endif
							pathlen += (check_strings[csSlashByName].str_len - 1);
						}
						memcpy(&pathbuf[pathlen], check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
					#if defined(RS_ENCRYPT_STR)
						simple_str_encrypt(&pathbuf[pathlen], check_strings[partition_seed].str_len, partition_seed);
					#endif
						/*pathlen += check_strings[partition_seed].str_len;*/
						/*pathbuf[pathlen] = '\0';*/

						/*RS_LOG("dm,6.2,%s\n", pathbuf);*/

						partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
					} else {
						partition_dev = NULL;
					}


					if (partition_dev) {
						/*RS_LOG("dm,7,%s\n", partition_dev);*/

						if (strcmp(real_dev_name, partition_dev) == 0) {
							RS_LOG("dm,8\n");
							need_check = 1;

							mount_type = rmtSystem;
						}
					}
				#endif

					if (!need_check) {
					#if defined(RS_HAS_EMMC_SHORTCUTS)
						partition_seed = csEmmcApps;

						#if defined(RS_ENCRYPT_STR)
							memcpy(pathbuf, check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
							simple_str_encrypt(pathbuf, check_strings[partition_seed].str_len, partition_seed);
							/*RS_LOG("dm,9.1,%s\n", pathbuf);*/
							partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
						#else
							partition_dev = get_real_filename_with_buffer(check_strings[partition_seed].str, pathbuf, RS_BUFFER_SIZE);
						#endif
					#endif

					#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
						if ((!partition_dev) && (g_block_platform_devname)) {
							partition_seed = csApps; /*csMsmApps;*/

							if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
								+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
								+ check_strings[partition_seed].str_len + 1)) {
								/*buffer overrun*/
								goto out_with_error; /*return -ENOBUFS;*/ /*返回非0值可能有问题，相当于挂载需要root相关检查*/
							}

							pathlen = check_strings[csBlkPlatformSlash].str_len;
							memcpy(pathbuf, check_strings[csBlkPlatformSlash].str, pathlen);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(pathbuf, pathlen, csBlkPlatformSlash);
						#endif
							if (g_block_platform_devname_len) {
								memcpy(&pathbuf[pathlen], g_block_platform_devname, g_block_platform_devname_len);
								pathlen += g_block_platform_devname_len;
								memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt(&pathbuf[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
							#endif
								pathlen += check_strings[csSlashByName].str_len;
							} else {
								memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt_ex(&pathbuf[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
							#endif
								pathlen += (check_strings[csSlashByName].str_len - 1);
							}
							memcpy(&pathbuf[pathlen], check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(&pathbuf[pathlen], check_strings[partition_seed].str_len, partition_seed);
						#endif
							/*pathlen += check_strings[partition_seed].str_len;*/
							/*pathbuf[pathlen] = '\0';*/

							RS_LOG("dm,9.2,%s\n", pathbuf);

							partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
						} else {
							partition_dev = NULL;
						}

						if (partition_dev) {
							RS_LOG("dm,10,%s\n", partition_dev);

							if (strcmp(real_dev_name, partition_dev) == 0) {
								RS_LOG("dm,11\n");
								need_check = 1;

								mount_type = rmtApps;
							}
						}
					#endif
					}

				#if defined(RS_HAS_OEM_PARTITION)
					if (!need_check) {
					#if 0 /*defined(RS_HAS_EMMC_SHORTCUTS)*/
						partition_seed = csEmmcOem;

						#if defined(RS_ENCRYPT_STR)
							memcpy(pathbuf, check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
							simple_str_encrypt(pathbuf, check_strings[partition_seed].str_len, partition_seed);
							/*RS_LOG("dm,11.1,%s\n", pathbuf);*/
							partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
						#else
							partition_dev = get_real_filename_with_buffer(check_strings[partition_seed].str, pathbuf, RS_BUFFER_SIZE);
						#endif
					#endif

					#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
						if ((!partition_dev) && (g_block_platform_devname)) {
							partition_seed = csOemPartName;

							if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
								+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
								+ check_strings[partition_seed].str_len + 1)) {
								/*buffer overrun*/
								goto out_with_error; /*return -ENOBUFS;*/ /*返回非0值可能有问题，相当于挂载需要root相关检查*/
							}

							pathlen = check_strings[csBlkPlatformSlash].str_len;
							memcpy(pathbuf, check_strings[csBlkPlatformSlash].str, pathlen);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(pathbuf, pathlen, csBlkPlatformSlash);
						#endif
							if (g_block_platform_devname_len) {
								memcpy(&pathbuf[pathlen], g_block_platform_devname, g_block_platform_devname_len);
								pathlen += g_block_platform_devname_len;
								memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt(&pathbuf[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
							#endif
								pathlen += check_strings[csSlashByName].str_len;
							} else {
								memcpy(&pathbuf[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
							#if defined(RS_ENCRYPT_STR)
								simple_str_encrypt_ex(&pathbuf[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
							#endif
								pathlen += (check_strings[csSlashByName].str_len - 1);
							}
							memcpy(&pathbuf[pathlen], check_strings[partition_seed].str, check_strings[partition_seed].str_len + 1);
						#if defined(RS_ENCRYPT_STR)
							simple_str_encrypt(&pathbuf[pathlen], check_strings[partition_seed].str_len, partition_seed);
						#endif
							/*pathlen += check_strings[partition_seed].str_len;*/
							/*pathbuf[pathlen] = '\0';*/

							RS_LOG("dm,11.2,%s\n", pathbuf);

							partition_dev = get_real_filename_with_buffer(pathbuf, pathbuf, RS_BUFFER_SIZE);
						} else {
							partition_dev = NULL;
						}


						if (partition_dev) {
							RS_LOG("dm,12,%s\n", partition_dev);

							if (strcmp(real_dev_name, partition_dev) == 0) {
								RS_LOG("dm,13\n");
								need_check = 1;

								mount_type = rmtOem;
							}
						}
					#endif
					}
				#endif

			#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
				out_with_error:
			#endif

			#endif
					RS_FREE_TWO_BUFFER(pathbuf);
				}
			}
		}
	}

got_mount_type:
	if (mount_type_ptr) {
		*mount_type_ptr = mount_type;
	}

	return need_check;
}

RS_DEFINE_FC_END(is_mount_dev_need_check);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_mount_need_check RS_HIDE(_t2)
#endif

RS_DEFINE_FC_BEGIN(is_mount_need_check);

noused notrace RS_DEFINE_FC_BODY(is_mount_need_check) \
static noinline int is_mount_need_check(const char *dev_name, const char *dir_name, int *mount_type_ptr)
{
#if 0
	return ((is_mount_dir_need_check(dir_name, mount_type_ptr)
		&& (!rs_verify_func(RS_FC_PARAMS(is_mount_dir_need_check), fcIsMountDirNeedCheck)))
		|| (is_mount_dev_need_check(dev_name, mount_type_ptr)
		&& (!rs_verify_func(RS_FC_PARAMS(is_mount_dev_need_check), fcIsMountDevNeedCheck)))
		);
#else
	#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
	int mount_type;

	if (is_mount_dir_need_check(dir_name, &mount_type)) {
		/*RS_LOG("imnc,1\n");*/
		if (mount_type == rmtRoot) {
			/*may be rmtSystem*/
			/*RS_LOG("imnc,2\n");*/
			is_mount_dev_need_check(dev_name, &mount_type);
		}

		if (mount_type_ptr) {
			*mount_type_ptr = mount_type;
		}

		return 1;
	} else {
		/*RS_LOG("imnc,4\n");*/
		return ((is_mount_dev_need_check(dev_name, mount_type_ptr))
			|| (rs_verify_func(RS_FC_PARAMS(is_mount_dir_need_check), fcIsMountDirNeedCheck))
			|| (rs_verify_func(RS_FC_PARAMS(is_mount_dev_need_check), fcIsMountDevNeedCheck))
			);
	}
	#else
	return ((is_mount_dir_need_check(dir_name, mount_type_ptr))
		|| (is_mount_dev_need_check(dev_name, mount_type_ptr))
		|| (rs_verify_func(RS_FC_PARAMS(is_mount_dir_need_check), fcIsMountDirNeedCheck))
		|| (rs_verify_func(RS_FC_PARAMS(is_mount_dev_need_check), fcIsMountDevNeedCheck))
		);
	#endif
#endif
}

RS_DEFINE_FC_END(is_mount_need_check);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_mount_bind_need_check RS_HIDE(_t2_)
#endif

noused notrace static noinline int is_mount_bind_need_check(void)
{
	/*android 5+, zygote ForkAndSpecializeCommon()中先调用DropCapabilitiesBoundingSet()再调用MountEmulatedStorage*/
	/*mount 时使用了 MS_BIND*/
	const struct cred *cred;
	cast_kernel_cap_t bset;
	/*rcu_read_lock();*/
	cred = current_cred();
	bset.cap = cred->cap_bset;
	/*rcu_read_unlock();*/
	return ((bset.val) ? 1 : 0);
}

#if defined(CONFIG_MOUNT_FILES_CHECK)

#if defined(RS_DELETE_BY_SYSCALL)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_rmdir RS_HIDE(_t3)
#endif
noused notrace static int __sys_rmdir(const char *pathname)
{
	int ret;
	mm_segment_t curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_rmdir(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)pathname;

		ret = __arm64_sys_rmdir((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_rmdir((const char __user *)pathname);
#endif
	set_fs(curr_fs);

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_unlink RS_HIDE(_t4)
#endif
noused notrace static int __sys_unlink(const char *pathname)
{
	int ret;
	mm_segment_t curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_unlink(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)pathname;

		ret = __arm64_sys_unlink((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_unlink((const char __user *)pathname);
#endif
	set_fs(curr_fs);

	return ret;
}

#else

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
/*kern_path_parent() removed*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_rmdir RS_HIDE(_t3)
#endif
noused notrace static int __sys_rmdir(const char *pathname)
{
	struct path parent;
	struct dentry *dentry;
	int error;

	dentry = kern_path_locked(pathname, &parent);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (dentry->d_inode) {
		error = vfs_rmdir(parent.dentry->d_inode, dentry);
	} else {
		error = -ENOENT;
	}
	dput(dentry);
	mutex_unlock(&parent.dentry->d_inode->i_mutex);
	path_put(&parent);
	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_unlink RS_HIDE(_t4)
#endif
noused notrace static int __sys_unlink(const char *pathname)
{
	struct path parent;
	struct dentry *dentry;
	int error;

	dentry = kern_path_locked(pathname, &parent);
	if (IS_ERR(dentry))
		return PTR_ERR(dentry);

	if (dentry->d_inode) {
		error = vfs_unlink(parent.dentry->d_inode, dentry);
	} else {
		error = -ENOENT;
	}
	dput(dentry);
	mutex_unlock(&parent.dentry->d_inode->i_mutex);
	path_put(&parent);
	return error;
}

#else
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_rmdir RS_HIDE(_t3)
#endif
noused notrace static int __sys_rmdir(const char *pathname)
{
	int error = 0;
	/*char *name;*/
	struct dentry *dentry;
	struct nameidata nd;

	error = kern_path_parent(pathname, &nd);
	if (error)
		return error;

	switch (nd.last_type) {
	case LAST_DOTDOT:
		error = -ENOTEMPTY;
		goto exit1;
	case LAST_DOT:
		error = -EINVAL;
		goto exit1;
	case LAST_ROOT:
		error = -EBUSY;
		goto exit1;
	}

	nd.flags &= ~LOOKUP_PARENT;
	error = mnt_want_write(nd.path.mnt);
	if (error)
		goto exit1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	inode_lock_nested(nd.path.dentry->d_inode, I_MUTEX_PARENT);
#else
	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
#endif
	/*dentry = lookup_hash(&nd);*/
	dentry = lookup_one_len(nd.last.name, nd.path.dentry, nd.last.len);
	error = PTR_ERR(dentry);
	if (IS_ERR(dentry))
		goto exit2;
	if (!dentry->d_inode) {
		error = -ENOENT;
		goto exit3;
	}
	error = security_path_rmdir(&nd.path, dentry);
	if (error)
		goto exit3;
	error = vfs_rmdir(nd.path.dentry->d_inode, dentry);
exit3:
	dput(dentry);
exit2:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	inode_unlock(nd.path.dentry->d_inode);
#else
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
#endif
	mnt_drop_write(nd.path.mnt);
exit1:
	path_put(&nd.path);
	return error;
}

/*refer to handle_remove in drivers/base/devtmpfs.c*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sys_unlink RS_HIDE(_t4)
#endif
noused notrace static int __sys_unlink(const char *pathname)
{
	struct nameidata nd;
	struct dentry *dentry;
	int error;
	struct inode *inode = NULL;

	error = kern_path_parent(pathname, &nd);
	if (error)
		return error;

	error = -EISDIR;
	if (nd.last_type != LAST_NORM)
		goto exit1;

	nd.flags &= ~LOOKUP_PARENT;
	error = mnt_want_write(nd.path.mnt);
	if (error)
		goto exit1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	inode_lock_nested(nd.path.dentry->d_inode, I_MUTEX_PARENT);
#else
	mutex_lock_nested(&nd.path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
#endif
	dentry = lookup_one_len(nd.last.name, nd.path.dentry, nd.last.len);
	error = PTR_ERR(dentry);
	if (!IS_ERR(dentry)) {
		/* Why not before? Because we want correct error value */
		if (nd.last.name[nd.last.len])
			goto slashes;
		inode = dentry->d_inode;
		if (!inode)
			goto slashes;
		ihold(inode);
		/*if (S_ISDIR(inode->i_mode))
			error = security_path_rmdir(&nd.path, dentry);
		else
		*/
			error = security_path_unlink(&nd.path, dentry);
		if (error)
			goto exit2;
		/*if (S_ISDIR(inode->i_mode))
			error = vfs_rmdir(nd.path.dentry->d_inode, dentry);
		else
		*/
			error = vfs_unlink(nd.path.dentry->d_inode, dentry);
exit2:
		dput(dentry);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	inode_unlock(nd.path.dentry->d_inode);
#else
	mutex_unlock(&nd.path.dentry->d_inode->i_mutex);
#endif
	if (inode)
		iput(inode);	/* truncate the inode here */
	mnt_drop_write(nd.path.mnt);
exit1:
	path_put(&nd.path);
	return error;

slashes:
	error = !dentry->d_inode ? -ENOENT :
		S_ISDIR(dentry->d_inode->i_mode) ? -EISDIR : -ENOTDIR;
	goto exit2;
}
#endif

#endif



/*
#include "ext2/ext2.h"
#define EXT2_IOC_GETFLAGS FS_IOC_GETFLAGS
#define EXT2_IOC_SETFLAGS FS_IOC_SETFLAGS
#define EXT2_IMMUTABLE_FL FS_IMMUTABLE_FL
#define EXT2_APPEND_FL FS_APPEND_FL
*/

/*from fs/sync.c syncfs*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_syncfs RS_HIDE(_u1)
#endif
noused notrace static int rs_syncfs(const char *filepath)
{
	int ret;
	struct file *filp;
	struct super_block *sb;
	/*mm_segment_t curr_fs;*/

	filp = local_filp_open(filepath, RS_IOCTL_OPEN_FLAGS, RS_IOCTL_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

#if 0
	{
		struct inode *node;
		umode_t mode;
		mode = ((node = filp->f_dentry->d_inode) ? node->i_mode : 0);

		if (!S_ISREG(mode) && !S_ISDIR(mode)) {
			ret = -EINVAL;
			goto out;
		}
	}
#endif

	sb = filp->f_dentry->d_sb;

	down_read(&sb->s_umount);
	ret = sync_filesystem(sb);
	up_read(&sb->s_umount);

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define set_file_flags RS_HIDE(_u2)
#endif
noused notrace static int set_file_flags(const char *filepath, unsigned long flags)
{
	int ret;
	struct file *filp;
	mm_segment_t curr_fs;

	filp = local_filp_open(filepath, RS_IOCTL_OPEN_FLAGS, RS_IOCTL_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

	{
		struct inode *node;
		umode_t mode;
		mode = ((node = filp->f_dentry->d_inode) ? node->i_mode : 0);

		if (!S_ISREG(mode) && !S_ISDIR(mode)) {
			ret = -EINVAL;
			goto out;
		}
	}

	curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
	ret = __vfs_ioctl(filp, FS_IOC_SETFLAGS, (unsigned long)&flags);
	set_fs(curr_fs);

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_file_flags RS_HIDE(_u3)
#endif
noused notrace static int get_file_flags(const char *filepath, unsigned long *flags)
{
	int ret;
	unsigned long local_flags = 0;
	struct file *filp;
	mm_segment_t curr_fs;

	filp = local_filp_open(filepath, RS_IOCTL_OPEN_FLAGS, RS_IOCTL_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

	{
		struct inode *node;
		umode_t mode;
		mode = ((node = filp->f_dentry->d_inode) ? node->i_mode : 0);

		if (!S_ISREG(mode) && !S_ISDIR(mode)) {
			ret = -EINVAL;
			goto out;
		}
	}

	curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
	ret = __vfs_ioctl(filp, FS_IOC_GETFLAGS, (unsigned long)&local_flags);
	set_fs(curr_fs);

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	if (flags)
		*flags = local_flags;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_file_flags_internal RS_HIDE(_u4)
#endif
noused notrace static int try_set_file_flags_internal(struct file *filp, unsigned long flags, int is_reset)
{
	int ret;
	unsigned long old_flags = 0, new_flags;
	mm_segment_t curr_fs;

	curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
	ret = __vfs_ioctl(filp, FS_IOC_GETFLAGS, (unsigned long)&old_flags);
	if (!ret) {
		if (is_reset) {
			new_flags = old_flags & ~flags;
		} else {
			new_flags = old_flags | flags;
		}

		if (new_flags != old_flags) {
			ret = __vfs_ioctl(filp, FS_IOC_SETFLAGS, (unsigned long)&new_flags);
		}
	}

	set_fs(curr_fs);

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_file_flags RS_HIDE(_u5)
#endif
noused notrace static int try_set_file_flags(const char *filepath, unsigned long flags, int is_reset)
{
	int ret;
	struct file *filp;

	filp = local_filp_open(filepath, RS_IOCTL_OPEN_FLAGS, RS_IOCTL_OPEN_MODE);
	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

	{
		struct inode *node;
		umode_t mode;
		mode = ((node = filp->f_dentry->d_inode) ? node->i_mode : 0);

		if (!S_ISREG(mode) && !S_ISDIR(mode)) {
			ret = -EINVAL;
			goto out;
		}
	}

	ret = try_set_file_flags_internal(filp, flags, is_reset);

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_file_immutable RS_HIDE(_u6)
#endif
noused notrace static int is_file_immutable(const char *filepath)
{
	unsigned long flags;

	if (!get_file_flags(filepath, &flags)) {
		return ((flags & RS_FS_IMMUTABLE_FLAGS) ? 1 : 0);
	} else {
		return 0;
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define reset_file_immutable_by_path RS_HIDE(_u7)
#endif
noused notrace static int reset_file_immutable_by_path(const char *filepath)
{
	return try_set_file_flags(filepath, RS_FS_IMMUTABLE_FLAGS, 1);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define reset_file_immutable_by_file RS_HIDE(_u8)
#endif
noused notrace static int reset_file_immutable_by_file(struct file *file)
{
	return try_set_file_flags_internal(file, RS_FS_IMMUTABLE_FLAGS, 1);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_child_file_flags RS_HIDE(_u9)
#endif
noused notrace static int try_set_child_file_flags(struct path *parent_path, struct dentry *dentry,
	unsigned long flags, int is_reset)
{
	int ret;
	struct file filp;

	/*faking one file filp*/
	memset(&filp, 0, sizeof(filp));

	filp.f_path.dentry = dentry;
	filp.f_path.mnt = parent_path->mnt;
	/*path_get(&filp.f_path);*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
	filp.f_inode = dentry->d_inode;
#endif

	{
		struct inode *node;
		umode_t mode;
		mode = ((node = dentry->d_inode) ? node->i_mode : 0);

		if (!S_ISREG(mode) && !S_ISDIR(mode)) {
			ret = -EINVAL;
			goto out;
		}
	}

	ret = try_set_file_flags_internal(&filp, flags, is_reset);

out:
	/*path_put(&filp.f_path);*/

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define reset_child_file_immutable RS_HIDE(_ua)
#endif
noused notrace static int reset_child_file_immutable(struct path *parent_path, struct dentry *dentry)
{
	return try_set_child_file_flags(parent_path, dentry, RS_FS_IMMUTABLE_FLAGS, 1);
}

#if defined(CONFIG_SECURITY_SELINUX)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_sel_file_xattr_internal RS_HIDE(_ub)
#endif
noused notrace static int try_set_sel_file_xattr_internal(struct dentry *dentry, const char *value, size_t size)
{
	return vfs_setxattr(dentry, XATTR_NAME_SELINUX, value, size, 0/*XATTR_REPLACE*/);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_set_sel_file_xattr RS_HIDE(_uc)
#endif
noused notrace static int try_set_sel_file_xattr(const char *filepath, int follow_link, const char *value, size_t size)
{
	int error;
	struct path path;

	error = kern_path(filepath, (follow_link ? LOOKUP_FOLLOW : 0), &path);

	if (!error) {
		error = try_set_sel_file_xattr_internal(path.dentry, value, size);

		path_put(&path);
	}

	return error;
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_reset_sel_file_xattr_by_dentry RS_HIDE(_ud)
#endif
noused notrace static int try_reset_sel_file_xattr_by_dentry(struct dentry *dentry)
{
#if defined(CONFIG_SECURITY_SELINUX)

#if defined(RS_ENCRYPT_STR)
	int ret;
	char *buffer = (char *)rs_kmalloc((check_strings[csUObjectUnlabeled].str_len + 1));
	if (buffer) {
		memcpy(buffer, check_strings[csUObjectUnlabeled].str, (check_strings[csUObjectUnlabeled].str_len + 1));
		simple_str_encrypt(buffer, check_strings[csUObjectUnlabeled].str_len, csUObjectUnlabeled);

		ret = try_set_sel_file_xattr_internal(dentry, buffer, (check_strings[csUObjectUnlabeled].str_len + 1));

		rs_kfree(buffer);
	} else {
		ret = -ENOMEM;
	}

	return ret;

#else
	return try_set_sel_file_xattr_internal(dentry, check_strings[csUObjectUnlabeled].str, (check_strings[csUObjectUnlabeled].str_len + 1));
#endif

#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define try_reset_sel_file_xattr_by_path RS_HIDE(_ue)
#endif
noused notrace static int try_reset_sel_file_xattr_by_path(const char *filepath)
{
#if defined(CONFIG_SECURITY_SELINUX)

#if defined(RS_ENCRYPT_STR)
	int ret;
	char *buffer = (char *)rs_kmalloc((check_strings[csUObjectUnlabeled].str_len + 1));
	if (buffer) {
		memcpy(buffer, check_strings[csUObjectUnlabeled].str, (check_strings[csUObjectUnlabeled].str_len + 1));
		simple_str_encrypt(buffer, check_strings[csUObjectUnlabeled].str_len, csUObjectUnlabeled);

		ret = try_set_sel_file_xattr(filepath, 0, buffer, (check_strings[csUObjectUnlabeled].str_len + 1));

		rs_kfree(buffer);
	} else {
		ret = -ENOMEM;
	}

	return ret;

#else
	return try_set_sel_file_xattr(filepath, 0, check_strings[csUObjectUnlabeled].str, (check_strings[csUObjectUnlabeled].str_len + 1));
#endif

#else
	return 0;
#endif
}

/*#if defined(CONFIG_MOUNT_CHECK_FILES_BY_BLACKLIST)*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define delete_files RS_HIDE(_v0)
#endif
noused notrace static int delete_files(const char **dir_list, int dir_list_size,
	const char **file_list, int file_list_size, int file_start_index)
{
	int dir_index, file_index;
	char *path_buf;
	const char *path;
	int dir_len, deleted_count;

	if ((!dir_list) || (!dir_list_size) || (!file_list) || (!file_list_size)
		|| (file_start_index >= file_list_size)) {
		/*RS_LOG("del 0\n");*/
		return -EPERM;
	}

	path_buf = (char *)RS_GET_BUFFER();
	if (!path_buf) {
		/*RS_LOG("del 1\n");*/
		return -ENOMEM;
	}

	deleted_count = 0;
	for (dir_index = 0; dir_index < dir_list_size; dir_index++) {
		path = dir_list[dir_index];

		if ((!path) || (*path == '\0')) {
			/*RS_LOG("del skip 0\n");*/
			continue;
		}

		dir_len = strlen(path);
		/*path + '/' + file_name + '\0'*/
		if (RS_BUFFER_SIZE < (dir_len + 1 + 1 + 1)) {
			/*buffer overrun*/
			RS_LOG("del overrun 0,%d\n", dir_index);
			continue;
		}

		memcpy(path_buf, path, dir_len);

	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(path_buf, dir_len, dir_index);
	#endif

		/*path_buf[dir_len] = '\0';*/
		/*RS_LOG("del dir 0,%s\n", path_buf);*/

		if (!is_dir_exist(path_buf)) {
			/*RS_LOG("del skip 1\n");*/
			continue;
		}

		if (path_buf[dir_len - 1] != '/') {
			path_buf[dir_len] = '/';
			dir_len++;
		}

		for (file_index = file_start_index; file_index < file_list_size; file_index++) {
			noused int file_name_len;
			const char *file_name = file_list[file_index];
			if ((!file_name) || (*file_name == '\0')) {
				/*RS_LOG("del skip file, %d\n", file_index);*/
				continue;
			}

			file_name_len = strlen(file_name);

			if (RS_BUFFER_SIZE < (dir_len + file_name_len + 1)) {
				/*buffer overrun*/
				/*RS_LOG("del overrun 1,%d\n", dir_index);*/
				continue;
			}

			memcpy(&path_buf[dir_len], file_name, file_name_len + 1); /*strcpy(&path_buf[dir_len], file_name);*/
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(&path_buf[dir_len], file_name_len, file_index);
			/*path_buf[dir_len + file_name_len] = '\0';*/
		#endif
			/*RS_LOG("del file,%s\n", path_buf);*/

		#if !defined(PASS_MOUNT_FILES_CHECK)
			reset_file_immutable_by_path(path_buf);

			try_reset_sel_file_xattr_by_path(path_buf);

			if (!__sys_unlink(path_buf))
		#else
			if (is_file_exist(path_buf))
		#endif
			{
				deleted_count++;

				/*RS_LOG("del %s\n", path_buf);*/
			}
		}
	}

	if (path_buf) {
		free_path(path_buf);
	}

	return deleted_count;
}

#if defined(RS_ENCRYPT_STR)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_dirs RS_HIDE(_va)
#endif
noused static const char *g_system_app_dirs[] = {
	"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xD7\xC5\xC4", /*"/system/app",*/
	"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xC5\xC6\xDA\xC4\x9C\xD1\xDF\xDE", /*"/system/priv-app",*/
	"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xC2\xDA\xC4\xDE\x9D\xCE\xDE\xDD\xDF", /*"/system/vivo-apps",*/
	"\x94\xC9\xC0\xCB\xC3\xD3\xD8\x9B\xC3\xC0\xD4\x9D\xC6\xC0\xDE\xD8\xCA\xC6\xC5", /*"/system/pre-install",*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_short_dirs RS_HIDE(_vb)
#endif
noused static const char *g_system_app_short_dirs[] = {
	"\xDF\xCD\xCC", /*"app",*/
	"\xCD\xCE\xD2\xCC\x94\xD9\xC7\xC6", /*"priv-app",*/
	"\xCA\xD2\xCC\xD6\x95\xD6\xC6\xC5\xC7", /*"vivo-apps",*/
	"\xCB\xC8\xDC\x95\xDE\xD8\xC6\xC0\xD2\xDE\xDD", /*"pre-install",*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_blacklist_files RS_HIDE(_vc)
#endif
noused static const char *g_system_app_blacklist_files[] = {
	"\xED\xC8\xCC\xDE\xC8\xCC\xCB\xD2\xC4\x9B\xD5\xC3\xD9", /*"Superuser.apk",*/
	"\xFF\xDD\xD2\xDE\xCC\xEA\xD8\xD9\xC1\x9A\xD2\xC2\xDA", /*"BaiduRoot.apk",*/
	"\xF7\xD2\xD4\xDE\xCD\xC4\xD3\xC7\x9A\xD2\xC2\xDA", /*"Kinguser.apk",*/
	"\xE8\xC3\xCA\xCC\xD2\xDB\xF1\xDB\xC4\xDC\xDD\xDF\xCE\xCA\xC8\xDE\x85\xCB\xD9\xC3", /*"SystemDownloader.apk",*/
	"\xE9\xCC\xC8\xD2\xC4\xE6\xE1\x9D\xD3\xC1\xDB", /*"SuperSU.apk"*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_dirs RS_HIDE(_vd)
#endif
noused static const char *g_system_bin_dirs[] = {
	"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xD4\xDC\xDA", /*"/system/bin",*/
	"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xCD\xD6\xDA\xDC", /*"/system/xbin",*/
	"\x93\xC8\xC3\xCA\xCC\xD2\xDB\x9A\xC7\xD1\xDB\xDF", /*"/system/sbin",*/
	"\x94\xCC\xDC\xD6\xD3\xD9\xC7\x9B\xD1\xDB\xDF", /*"/vendor/bin",*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_short_dirs RS_HIDE(_ve)
#endif
noused static const char *g_system_bin_short_dirs[] = {
	"\xDC\xD4\xD2", /*"bin",*/
	"\xC5\xDE\xD2\xD4", /*"xbin",*/
	"\xCF\xD9\xD3\xD7", /*"sbin",*/
	"\xCD\xDF\xD7\xDC\xD8\xC4\x9A\xD6\xDA\xDC", /*"vendor/bin",*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_blacklist_files RS_HIDE(_vf)
#endif
noused static const char *g_system_bin_blacklist_files[] = {
	"\xCD\xC8", /*"su",*/
	"\xD9\xDD\xDE\xD7\xD6\xD6\xC4\xC3", /*"daemonsu",*/
#if !defined(CONFIG_MOUNT_CHECK_SU_ONLY)
	"\xCF\xCE\xDD\xD6\xCC\xD2", /*"sugote",*/
	"\xC8\xCF\xDE\xD7\xC3\xD3\x98\xD9\xD8\xC1\xD9", /*"sugote-mksh",*/
	"\xC8\xCD\x96\xC4\xDE", /*"rt.sh",*/
	"\xCC\xCB", /*"us",*/
	"\x96\xC4\xC3", /*".su",*/
	"\x99\xC3\xC6", /*".us",*/
	"\x9B\xD1\xCB\xC6\x9E\x9E\xDC\xDB", /*"supolicy"*/
	"\x9B\xD1\xCB\xC6\x9E\x9E\xDC\xDB", /*".ext/.su"*/
	"\xDF\xC6\x9C\xC2\xC5\xCB",  /*"ku.sud",*/ /*kingroot*/
#endif
};

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_dirs RS_HIDE(_va)
#endif
noused static const char *g_system_app_dirs[] = {
	"/system/app",
	"/system/priv-app",
	"/system/vivo-apps",
	"/system/pre-install",
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_short_dirs RS_HIDE(_vb)
#endif
noused static const char *g_system_app_short_dirs[] = {
	"app",
	"priv-app",
	"vivo-apps",
	"pre-install",
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_app_blacklist_files RS_HIDE(_vc)
#endif
noused static const char *g_system_app_blacklist_files[] = {
	"Superuser.apk",
	"BaiduRoot.apk",
	"Kinguser.apk",
	"SystemDownloader.apk",
	"SuperSU.apk",
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_dirs RS_HIDE(_vd)
#endif
noused static const char *g_system_bin_dirs[] = {
	"/system/bin",
	"/system/xbin",
	"/system/sbin",
	"/vendor/bin",
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_short_dirs RS_HIDE(_ve)
#endif
noused static const char *g_system_bin_short_dirs[] = {
	"bin",
	"xbin",
	"sbin",
	"vendor/bin",
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_system_bin_blacklist_files RS_HIDE(_vf)
#endif
noused static const char *g_system_bin_blacklist_files[] = {
	"su", /*supersu*/
	"daemonsu", /*supersu*/
#if !defined(CONFIG_MOUNT_CHECK_SU_ONLY)
	"sugote", /*supersu*/
	"sugote-mksh", /*supersu*/
	"rt.sh",
	"us",
	".su", /*supersu*/
	".us",
	"supolicy",
	".ext/.su",
	"ku.sud", /*kingroot*/
#endif
};

#endif

#define RS_BIN_BLACKLIST_FILES_MISC_START_INDEX (2)
#define RS_BIN_BLACKLIST_FILES_SU_COUNT RS_BIN_BLACKLIST_FILES_MISC_START_INDEX

/*#endif*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define unlink_child RS_HIDE(_w0)
#endif
noused notrace static int unlink_child(struct path *parent_path, struct dentry *dentry)
{
	int error;
	struct inode *inode;

	error = mnt_want_write(parent_path->mnt);
	if (error)
		goto exit1;

	inode = dentry->d_inode;
	if (!inode)
			goto slashes;
	ihold(inode);
	error = security_path_unlink(parent_path, dentry);
	if (error)
		goto exit3;

	reset_child_file_immutable(parent_path, dentry);

	try_reset_sel_file_xattr_by_dentry(dentry);

	error = vfs_unlink(parent_path->dentry->d_inode, dentry);
exit3:
exit2:
	if (inode)
		iput(inode);	/* truncate the inode here */
	mnt_drop_write(parent_path->mnt);
exit1:

	return error;

slashes:
	error = !dentry->d_inode ? -ENOENT :
		S_ISDIR(dentry->d_inode->i_mode) ? -EISDIR : -ENOTDIR;
	goto exit2;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rmdir_child RS_HIDE(_w1)
#endif
noused notrace static int rmdir_child(struct path *parent_path, struct dentry *dentry)
{
	int error;
	struct inode *inode;

	error = mnt_want_write(parent_path->mnt);
	if (error)
		goto exit1;

	inode = dentry->d_inode;
	if (!inode) {
			error = -ENOENT;
			goto exit2;
	}
	error = security_path_rmdir(parent_path, dentry);
	if (error)
		goto exit3;

	reset_child_file_immutable(parent_path, dentry);

	try_reset_sel_file_xattr_by_dentry(dentry);

	error = vfs_rmdir(parent_path->dentry->d_inode, dentry);
exit3:
exit2:
	mnt_drop_write(parent_path->mnt);
exit1:

	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_valid_app_name RS_HIDE(_w4)
#endif
noused notrace static int is_valid_app_name(const char *s, int len)
{
	const char *p = s + len;

	#undef INVALID_FLAG
	#undef ALPHABET_FLAG
	#undef DIGIT_FLAG
	#undef HEX_DIGIT_FLAG
	#define INVALID_FLAG 1
	#define ALPHABET_FLAG 2
	#define DIGIT_FLAG 4
	#define HEX_DIGIT_FLAG 8

	int flags = 0;

	do {
		char ch = *s;
		if ((ch == '.') || (ch == '-') || (ch == '_')) {

		} else if ((('A' <= ch) && (ch <= 'F'))
			|| (('a' <= ch) && (ch <= 'f'))) {
			flags |= HEX_DIGIT_FLAG;
		} else if ((('G' <= ch) && (ch <= 'Z'))
			|| (('g' <= ch) && (ch <= 'z'))) {
			flags |= ALPHABET_FLAG;
		} else if (('0' <= ch) && (ch <= '9')) {
			flags |= DIGIT_FLAG;
		} else {
			flags |= INVALID_FLAG;
			break;
		}

	} while (++s < p);

	if (INVALID_FLAG & flags) {
		return 0;
	} else {
		if ((flags == DIGIT_FLAG) /*only digit*/
			|| ((flags & HEX_DIGIT_FLAG) && (flags & DIGIT_FLAG) && (!(flags & ALPHABET_FLAG))) /*only hex*/
			) {
			return 0;
		} else {
			return 1;
		}
	}
}

#undef RS_DIR_CHECKED_DT_FLAG
#define RS_DIR_CHECKED_DT_FLAG ((unsigned int)INT_MAX + 1) /*(DT_DIR)*/ /*(32)*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define build_dir_child_names RS_HIDE(_w5)
#endif
noused notrace static int build_dir_child_names(
#if defined(RS_NO_VFS_READDIR) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	struct dir_context *ctx,
#else
	void *arg,
#endif
	const char *name, int name_len,
		loff_t offset, u64 ino, unsigned int d_type)
{
#if defined(RS_NO_VFS_READDIR) && (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0))
	struct build_names_params *params = container_of(ctx, struct build_names_params, ctx);
#else
	struct build_names_params *params = arg;
#endif
	struct name_node *entry;

	params->sequence++;

	/*namelen不包括0结束符在内*/

	if ((name_len <= 0)
	#if defined(RS_NAME_NODE_USE_PAGE)
			/*|| (name_len > NAME_NODE_MAX_NAME_LEN)*/
	#endif
		) {
		/*RS_LOG("lst too long\n");*/
		return 0;
	}

	/*skip "." and ".."*/
	if (name[0] == '.') {
		if ((name[1] == '\0') || (name_len == 1)) {
			return 0;
		} else if ((name[1] == '.') && ((name[2] == '\0') || (name_len == 2))) {
			return 0;
		}
	}

	/*if (d_type == DT_DIR)*/
	{
		/*dir use full path*/
		int dir_path_len = params->dir_path_len;
		int path_len = dir_path_len + name_len; /*without null char*/
		if (!params->dir_path_has_sep) /*without path separator*/
			path_len++;

	#if defined(RS_NAME_NODE_USE_PAGE)
		if (path_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(path_len + 1);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(path_len + 1);

		if (entry == NULL)
			return -ENOMEM;

		entry->name_len = path_len;
		memcpy(entry->name, params->dir_path, dir_path_len);
		if (!params->dir_path_has_sep) {
			entry->name[dir_path_len] = '/';
			dir_path_len++;
		}
		memcpy(&entry->name[dir_path_len], name, name_len);
		entry->name[path_len] = '\0';

		if (d_type == DT_DIR) {
			entry->d_type = d_type & ~RS_DIR_CHECKED_DT_FLAG;
			list_add(&entry->list, &params->dirs);
		} else {
			entry->d_type = d_type;
			list_add(&entry->list, &params->files);
		}
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_and_del_dir_children RS_HIDE(_w6)
#endif
noused notrace static int check_and_del_dir_children(const char *dir_str_path, struct build_names_params *params)
{
	int error;
#if !defined(RS_READ_DIR_USE_FILP)
	struct path dir_path;
	struct inode *dir;
#endif
	struct file *file;
	extern void fput(struct file *);

#if defined(RS_READ_DIR_USE_FILP)

	file = local_filp_open(dir_str_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

#else

	error = kern_path(dir_str_path, RS_READ_DIR_KERN_PATH_FLAGS, &dir_path);
	if (error) {
		RS_LOG("cdc 0,%d\n", error);
		return error;
	}

	dir = dir_path.dentry->d_inode;

	if (!dir || !S_ISDIR(dir->i_mode)) {
		RS_LOG("cdc 1\n");
		error = -ENOTDIR;
		goto out;
	}

	if (!dir->i_fop) {
		RS_LOG("cdc 2\n");
		error = -EINVAL;
		goto out;
	}
	/*
	 * Open the directory ...
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	file = dentry_open(&dir_path, RS_READ_DIR_OPEN_FLAGS, current_cred());
#else
	file = dentry_open(dget(dir_path.dentry), mntget(dir_path.mnt), RS_READ_DIR_OPEN_FLAGS, current_cred());
#endif

#endif

	if (IS_ERR(file)) {
		error = PTR_ERR(file);
		file = NULL;
		RS_LOG("cdc 3\n");
		goto out;
	}

#if defined(RS_NO_VFS_READDIR)
	if (!file->f_op->iterate)
#else
	if (!file->f_op->readdir)
#endif
	{
		RS_LOG("cdc 4\n");
		/*error = -EINVAL;*/
		/*goto out_close;*/
	}

	/*reset dir's immutable flag*/
	reset_file_immutable_by_file(file);

	try_reset_sel_file_xattr_by_dentry(file->f_dentry);

#if 0

#if defined(RS_NO_VFS_READDIR)
	memset(&params->ctx, 0, sizeof(params->ctx));
	*((filldir_t *)&params->ctx.actor) = build_dir_child_names;
#endif
	INIT_LIST_HEAD(&params->files);
	INIT_LIST_HEAD(&params->dirs);
	params->buffer = (char *)RS_GET_BUFFER();

	if (!params->buffer) {
		RS_LOG("cdc 5\n");
		error = -ENOMEM;
		goto out_close;
	}

	params->dir_path = system_app_path;
	params->dir_path_len = strlen(params->dir_path);
	params->dir_path_has_sep = ((params->dir_path_len && (params->dir_path[params->dir_path_len - 1] == '/')) ? 1 : 0);
#endif

#if defined(RS_NO_VFS_READDIR)
	*((filldir_t *)&params->ctx.actor) = build_dir_child_names;
#endif
	params->sequence = 0;

	while (1) {
		int old_seq = params->sequence;

	#if defined(RS_NO_VFS_READDIR)
		error = iterate_dir(file, &params->ctx);
	#else
		error = vfs_readdir(file, build_dir_child_names, params);
	#endif

		if ((error < 0) && (!should_retry_syscall(error)))
			break;

		/*error = -ENOENT;*/
		if (old_seq == params->sequence)
			break;
	}

	if (params->sequence) {
		/*RS_LOG("cdc 6\n");*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_lock_nested(file->f_dentry->d_inode, I_MUTEX_PARENT);
	#else
		inode_lock_nested(dir_path.dentry->d_inode, I_MUTEX_PARENT);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_lock_nested(&file->f_dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#else
		mutex_lock_nested(&dir_path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#endif
#endif

		while (!list_empty(&params->files)) {
			struct name_node *entry;
			struct dentry *dentry;
			entry = list_entry(params->files.next, struct name_node, list);
		#if defined(RS_READ_DIR_USE_FILP)
			dentry = lookup_one_len(entry->name, file->f_dentry, entry->name_len);
		#else
			dentry = lookup_one_len(entry->name, dir_path.dentry, entry->name_len);
		#endif

			if (IS_ERR(dentry)) {
				error = PTR_ERR(dentry);
				/*RS_LOG("cdc 7,%d\n", error);*/
				/*break;*/
			} else {
			#if !defined(PASS_MOUNT_FILES_CHECK)
				if ((!is_op_bypassed(ovFilesCheck))
					&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
					) {
				#if defined(RS_READ_DIR_USE_FILP)
					error = unlink_child(&file->f_path, dentry);
				#else
					error = unlink_child(&dir_path, dentry);
				#endif
				}
			#endif

				RS_LOG("cdc del file:%s, %d\n", entry->name, error);

				dput(dentry);
			}

			/*RS_LOG("cdc file:%s\n", entry->name);*/

			/*cond_resched();*/

			list_del(&entry->list);
		#if defined(RS_NAME_NODE_USE_PAGE)
			if (entry->name_len > NAME_NODE_MAX_NAME_LEN)
				FREE_LARGE_NAME_NODE(entry);
			else
		#endif
			FREE_NAME_NODE(entry);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_unlock(file->f_dentry->d_inode);
	#else
		inode_unlock(dir_path.dentry->d_inode);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_unlock(&file->f_dentry->d_inode->i_mutex);
	#else
		mutex_unlock(&dir_path.dentry->d_inode->i_mutex);
	#endif
#endif
	}

	/*RS_FREE_BUFFER(params->buffer);*/

	/*RS_LOG("cdc e\n");*/

out_close:
#if defined(RS_READ_DIR_USE_FILP)
	if (file) {
		filp_close(file, RS_FILP_CLOSE_ID);
	}
#else
	fput(file);
#endif

out:
#if !defined(RS_READ_DIR_USE_FILP)
	path_put(&dir_path);
#endif

	return error;
}

#define is_valid_bin_name is_valid_app_name

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define build_bad_app_names RS_HIDE(_w7)
#endif
noused notrace static int build_bad_app_names(void *arg, const char *name, int name_len,
		loff_t offset, u64 ino, unsigned int d_type)
{
	struct build_names_params *params = arg;
	char *buffer = params->buffer;
	struct name_node *entry;
	char *p;
	int need_add = 0;

	params->sequence++;

	/*namelen不包括0结束符在内*/

	if ((name_len <= 0)
	#if defined(RS_NAME_NODE_USE_PAGE)
			/*|| (name_len > NAME_NODE_MAX_NAME_LEN)*/
	#endif
		) {
		/*RS_LOG("ban too long\n");*/
		return 0;
	}

	/*skip "." and ".."*/
#if 1
	if (name[0] == '.') {
		if ((name[1] == '\0') || (name_len == 1)) {
			return 0;
		} else if ((name[1] == '.') && ((name[2] == '\0') || (name_len == 2))) {
			return 0;
		}
	}
#endif

	if (d_type == DT_DIR) {
		/*dir use full path*/
		int dir_path_len = params->dir_path_len;
		int path_len = dir_path_len + name_len; /*without null char*/
		if (!params->dir_path_has_sep) /*without path separator*/
			path_len++;

	#if defined(RS_NAME_NODE_USE_PAGE)
		if (path_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(path_len + 1);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(path_len + 1);

		if (entry == NULL)
			return -ENOMEM;

		entry->d_type = d_type & ~RS_DIR_CHECKED_DT_FLAG;
		entry->name_len = path_len;
		memcpy(entry->name, params->dir_path, dir_path_len);
		if (!params->dir_path_has_sep) {
			entry->name[dir_path_len] = '/';
			dir_path_len++;
		}
		memcpy(&entry->name[dir_path_len], name, name_len);
		entry->name[path_len] = '\0';
		list_add(&entry->list, &params->dirs);

		return 0;
	}

	if (!((d_type == DT_REG) || (d_type == DT_LNK)) && (d_type != DT_UNKNOWN)) {
		return 0;
	}

	/*RS_LOG("ban b:%s,%d\n", name, name_len);*/

	memcpy(buffer, name, name_len);
	buffer[name_len] = '\0';
	strlwr_len(buffer, name_len);

	/*superuser/kinguser/AngelXuser*/
	if (STRSTR(buffer, check_strings[csUserDot].str, csUserDot)) {
		/*UserDictionaryProvider.apk*/
		need_add = 1;
	} else if (STRSTR(buffer, check_strings[csRoot].str, csRoot)) {
		/*baiduroot*/
		need_add = 1;
	} else {
		p = STRSTR(buffer, check_strings[csSuper].str, csSuper);
		if (p != NULL) {
			/*SuperPowerSave.apk
			superuser.apk supersu.apk
			预置的apk特征：开头+30字节读取的内容是"AndroidManifest.xml",末尾-39字节读取的内容是"META-INF/CERT.RSAPK"
			如果有对应.odex文件，则为预置的apk
			*/
			/*filp_open/filp_close/vfs_llseek/vfs_read*/
			p += check_strings[csSuper].str_len;
			if ((p[0] == 's') && (p[1] == 'u'))
				need_add = 1;
		} else if (STRSTR(buffer, check_strings[csSystemDownloader].str, csSystemDownloader) != NULL) {
			/*SystemDownloader.apk root精灵的木马*/
			need_add = 1;
		} else if (!is_valid_app_name(name, name_len)) {
			need_add = 1;
		}
	}

	if (need_add) {
	#if defined(RS_NAME_NODE_USE_PAGE)
		if (name_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(name_len);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(name_len);
		if (entry == NULL)
			return -ENOMEM;
		entry->name_len = name_len;
		memcpy(entry->name, name, name_len);
		entry->name[name_len] = '\0';
		list_add(&entry->list, &params->files);
	}

	return 0;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define build_bad_bin_names RS_HIDE(_w8)
#endif
noused notrace static int build_bad_bin_names(void *arg, const char *name, int name_len,
		loff_t offset, u64 ino, unsigned int d_type)
{
	struct build_names_params *params = arg;
	char *buffer = params->buffer;
	noused char *pName = buffer;
	noused struct name_node *entry;
	noused int need_add = 0;

	params->sequence++;

	/*namelen不包括0结束符在内*/

	if ((name_len <= 0)
	#if defined(RS_NAME_NODE_USE_PAGE)
			/*|| (name_len > NAME_NODE_MAX_NAME_LEN)*/
	#endif
		) {
		/*RS_LOG("bbn too long\n");*/
		return 0;
	}

	/*skip "." and ".."*/
	if (name[0] == '.') {
		if ((name[1] == '\0') || (name_len == 1)) {
			return 0;
		} else if ((name[1] == '.') && ((name[2] == '\0') || (name_len == 2))) {
			return 0;
		} else {
			need_add = 1;
		}
	}

	if (d_type == DT_DIR) {
		/*dir use full path*/
		int dir_path_len = params->dir_path_len;
		int path_len = dir_path_len + name_len; /*without null char*/
		if (!params->dir_path_has_sep) /*without path separator*/
			path_len++;

	#if defined(RS_NAME_NODE_USE_PAGE)
		if (path_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(path_len + 1);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(path_len + 1);

		if (entry == NULL) {
			return -ENOMEM;
		}

		entry->d_type = d_type & ~RS_DIR_CHECKED_DT_FLAG;
		entry->name_len = path_len;
		memcpy(entry->name, params->dir_path, dir_path_len);
		if (!params->dir_path_has_sep) {
			entry->name[dir_path_len] = '/';
			dir_path_len++;
		}
		memcpy(&entry->name[dir_path_len], name, name_len);
		entry->name[path_len] = '\0';
		list_add(&entry->list, &params->dirs);

#if 0
	#if defined(RS_NAME_NODE_USE_PAGE)
		if (name_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(name_len);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(name_len);
		if (entry == NULL)
			return -ENOMEM;
		entry->d_type = d_type & ~RS_DIR_CHECKED_DT_FLAG;
		entry->name_len = name_len;
		memcpy(entry->name, name, name_len);
		entry->name[name_len] = '\0';
		list_add(&entry->list, &params->dirs);
#endif
		return 0;
	}

	if (!((d_type == DT_REG) || (d_type == DT_LNK)) && (d_type != DT_UNKNOWN)) {
		return 0;
	}

	/*RS_LOG("bbn b:%s,%d\n", name, name_len);*/

	if (!need_add) {
		int is_su;
		char *pSu;

		memcpy(buffer, name, name_len);
		buffer[name_len] = '\0';
		strlwr_len(buffer, name_len);

		pSu = strrchr(pName, 's');

		/*XXXsu 或 XXXus*/
		if (pSu)
			is_su = ((pSu[1] == 'u') && ((pSu[2] == '\0') || (pSu[2] == 'd')));

		if ((pSu) && ((is_su) /*su结尾或带"sud"*/
			|| ((pSu > pName) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
			|| ((pSu[-1] == '.') && (((pSu[1] == 'o') && (pSu[2] == '\0')) /*.so结尾*/ /*360 mobilesafe, libsu.so，liba.so.4x.so*/
			|| (pSu[1] == 'u'))) /*.suXXX*/
			)
			)
			)
			) {
		#if !defined(RS_WITHOUT_BBKSU)
			if ((is_su) && (STRCMP_EX(pName, sid_check_strings[sidcsBBKSu].str,
				(sizeof("/system/xbin/") - 1), sidcsBBKSu) == 0)) {
				/*bbksu*/
			} else
		#endif
			{
				need_add = 1;
			}
		} else {
		#if 1
			char *pDot = strrchr(pName, '.');
			if (pDot) {
				/*带'.'的有sensors.qcom/mount.exfat/mkfs.exfat/fsck.exfat/exfat.label等*/
				if ((pDot[1] == 's') && (pDot[2] == 'h') && (pDot[3] == '\0')) {
					/* rt.sh，目前系统未有放.sh在system的bin和xbin下*/
					need_add = 1;
				} else {
					/*need_add = 1;*/
				}
			}
		#endif

			if (!need_add) {
				if ((pSu) && (pSu == pName) && (pSu[1] == 'u') && (name_len < 10)) {
					/*subsystem_ramdump，surfaceflinger*/
					/*"sugote"*/ /*zygote 的 类比？*/
					need_add = 1;
				}
			}
		}

	}
	/*还有带"root"/"inject"/"exploit"*/

	if ((!need_add) && (!is_valid_bin_name(name, name_len))) {
		need_add = 1;
	}

	if (need_add) {
	#if defined(RS_NAME_NODE_USE_PAGE)
		if (name_len > NAME_NODE_MAX_NAME_LEN)
			entry = (struct name_node *)ALLOC_LARGE_NAME_NODE(name_len);
		else
	#endif
		entry = (struct name_node *)ALLOC_NAME_NODE(name_len);
		if (entry == NULL) {
			return -ENOMEM;
		}
		entry->name_len = name_len;
		memcpy(entry->name, name, name_len);
		entry->name[name_len] = '\0';
		list_add(&entry->list, &params->files);
	}

	return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 7, 0)
	#define SYSTEM_PATH_APPEND_MARGIN (0)
#else
	#define SYSTEM_PATH_APPEND_MARGIN (sizeof(struct filename))
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_system_dir RS_HIDE(_x0)
#endif
noused notrace static int check_system_dir(const char *system_dir_path, const filldir_t filldir_callback)
{
	int error;
#if !defined(RS_READ_DIR_USE_FILP)
	struct path dir_path;
	struct inode *dir;
#endif
	struct file *file;
	struct build_names_params params; /* = { .names = LIST_HEAD_INIT(.names), .buffer = NULL};*/
	noused extern void fput(struct file *);
#if defined(RS_ELEVATE_CREDS)
	noused const struct cred *original_cred;
#endif
	noused int need_check_dirs = 0;

	/*RS_LOG("csd %s\n", system_dir_path);*/

#if defined(RS_READ_DIR_USE_FILP)

	file = local_filp_open(system_dir_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

#else

	error = kern_path(system_dir_path, RS_READ_DIR_KERN_PATH_FLAGS, &dir_path);
	if (error) {
		RS_LOG("csd 0,%d\n", error);
		return error;
	}

	dir = dir_path.dentry->d_inode;

	if (!dir || !S_ISDIR(dir->i_mode)) {
		RS_LOG("csd 1\n");
		error = -ENOTDIR;
		goto out;
	}

	if (!dir->i_fop) {
		RS_LOG("csd 2\n");
		error = -EINVAL;
		goto out;
	}
	/*
	 * Open the directory ...
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	file = dentry_open(&dir_path, RS_READ_DIR_OPEN_FLAGS, current_cred());
#else
	file = dentry_open(dget(dir_path.dentry), mntget(dir_path.mnt), RS_READ_DIR_OPEN_FLAGS, current_cred());
#endif

#endif

	if (IS_ERR(file)) {
		error = PTR_ERR(file);
		file = NULL;
		RS_LOG("csd 3\n");
		goto out;
	}

#if defined(RS_NO_VFS_READDIR)
	if (!file->f_op->iterate)
#else
	if (!file->f_op->readdir)
#endif
	{
		RS_LOG("csd 4\n");
		/*error = -EINVAL;*/
		/*goto out_close;*/
	}

	params.buffer = (char *)RS_GET_BUFFER();
	if (!params.buffer) {
		RS_LOG("csd 5\n");
		error = -ENOMEM;
		goto out_close;
	}

#if defined(RS_NO_VFS_READDIR)
	memset(&params.ctx, 0, sizeof(params.ctx));
	*((filldir_t *)&params.ctx.actor) = filldir_callback;
#endif
	INIT_LIST_HEAD(&params.files);
	INIT_LIST_HEAD(&params.dirs);

	params.dir_path = system_dir_path;
	params.dir_path_len = strlen(params.dir_path);
	params.dir_path_has_sep = ((params.dir_path_len && (params.dir_path[params.dir_path_len - 1] == '/')) ? 1 : 0);

#if defined(RS_ELEVATE_CREDS)
	error = rs_elevate_creds(&original_cred);
	if (error) {
		RS_LOG("csd 5.1\n");
		goto out_with_buff;
	}
#endif

	/*reset dir's immutable flag*/
	reset_file_immutable_by_file(file);

	/*需要删掉的子目录才需要设置SELinux属性,本目录是系统目录,不需要设置*/
	/*try_reset_sel_file_xattr_by_dentry(file->f_dentry);*/

	/*safe_vfs_llseek(file, 0, SEEK_SET);*/

	params.sequence = 0;

	while (1) {
		int old_seq = params.sequence;

	#if defined(RS_NO_VFS_READDIR)
		error = iterate_dir(file, &params.ctx);
	#else
		error = vfs_readdir(file, filldir_callback, &params);
	#endif

		if ((error < 0) && (!should_retry_syscall(error)))
			break;

		/*error = -ENOENT;*/
		if (old_seq == params.sequence)
			break;
	}

	if (params.sequence) {
		/*RS_LOG("csd 6\n");*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_lock_nested(file->f_dentry->d_inode, I_MUTEX_PARENT);
	#else
		inode_lock_nested(dir_path.dentry->d_inode, I_MUTEX_PARENT);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_lock_nested(&file->f_dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#else
		mutex_lock_nested(&dir_path.dentry->d_inode->i_mutex, I_MUTEX_PARENT);
	#endif
#endif

		while (!list_empty(&params.files)) {
			struct name_node *entry;
			struct dentry *dentry;
			entry = list_entry(params.files.next, struct name_node, list);
		#if defined(RS_READ_DIR_USE_FILP)
			dentry = lookup_one_len(entry->name, file->f_dentry, entry->name_len);
		#else
			dentry = lookup_one_len(entry->name, dir_path.dentry, entry->name_len);
		#endif

			if (IS_ERR(dentry)) {
				error = PTR_ERR(dentry);
				/*RS_LOG("csd 7,%d\n", error);*/
				/*break;*/
			} else {
			#if !defined(PASS_MOUNT_FILES_CHECK)
				if ((!is_op_bypassed(ovFilesCheck))
					&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
					) {
				#if defined(RS_READ_DIR_USE_FILP)
					error = unlink_child(&file->f_path, dentry);
				#else
					error = unlink_child(&dir_path, dentry);
				#endif
				}
			#endif

				/*RS_LOG("csd del:%s, %d\n", entry->name, error);*/

				dput(dentry);
			}

			/*RS_LOG("csd entry:%s\n", entry->name);*/

			/*cond_resched();*/

			list_del(&entry->list);
		#if defined(RS_NAME_NODE_USE_PAGE)
			if (entry->name_len > NAME_NODE_MAX_NAME_LEN)
				FREE_LARGE_NAME_NODE(entry);
			else
		#endif
			FREE_NAME_NODE(entry);
		}

	#if 0
		while (!list_empty(&params.dirs)) {
			struct name_node *entry;
			struct dentry *dentry;
			entry = list_entry(params.dirs.next, struct name_node, list);
		#if defined(RS_READ_DIR_USE_FILP)
			dentry = lookup_one_len(entry->name, file->f_dentry, entry->name_len);
		#else
			dentry = lookup_one_len(entry->name, dir_path.dentry, entry->name_len);
		#endif

			if (IS_ERR(dentry)) {
				error = PTR_ERR(dentry);
				/*RS_LOG("csd 7,%d\n", error);*/
				/*break;*/
			} else {
			#if !defined(PASS_MOUNT_FILES_CHECK)
				if ((!is_op_bypassed(ovFilesCheck))
					&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
					) {
				#if defined(RS_READ_DIR_USE_FILP)
					error = rmdir_child(&file->f_path, dentry);
				#else
					error = rmdir_child(&dir_path, dentry);
				#endif
				}
			#endif

				/*RS_LOG("csd del dir:%s, %d\n", entry->name, error);*/

				dput(dentry);
			}

			/*RS_LOG("csd dir:%s\n", entry->name);*/

			/*cond_resched();*/

			list_del(&entry->list);
		#if defined(RS_NAME_NODE_USE_PAGE)
			if (entry->name_len > NAME_NODE_MAX_NAME_LEN)
				FREE_LARGE_NAME_NODE(entry);
			else
		#endif
			FREE_NAME_NODE(entry);
		}
	#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	#if defined(RS_READ_DIR_USE_FILP)
		inode_unlock(file->f_dentry->d_inode);
	#else
		inode_unlock(dir_path.dentry->d_inode);
	#endif
#else
	#if defined(RS_READ_DIR_USE_FILP)
		mutex_unlock(&file->f_dentry->d_inode->i_mutex);
	#else
		mutex_unlock(&dir_path.dentry->d_inode->i_mutex);
	#endif
#endif

		need_check_dirs = 1;
	}

#if defined(RS_ELEVATE_CREDS)
	rs_reset_creds(original_cred);

out_with_buff:
#endif
	RS_FREE_BUFFER(params.buffer);
	params.buffer = NULL;

out_close:
#if defined(RS_READ_DIR_USE_FILP)
	if (file) {
		filp_close(file, RS_FILP_CLOSE_ID);
	}
#else
	fput(file);
#endif
out:
#if !defined(RS_READ_DIR_USE_FILP)
	path_put(&dir_path);
#endif

	if (need_check_dirs) {
		/*RS_LOG("csd dir chk\n");*/

		if (!list_empty(&params.dirs)) {
			struct name_node *new_first_entry;
			struct name_node *old_first_entry;

			/*RS_LOG("csd dir 0\n");*/

			new_first_entry = list_first_entry(&params.dirs, struct name_node, list);

			while (1) {
				int ret;
				/*int count = 0;*/
				struct name_node *entry;

				old_first_entry = new_first_entry;

			#if !defined(list_next_entry) /*linux 3.13*/
				#define list_next_entry(pos, member) \
					list_entry((pos)->member.next, typeof(*(pos)), member)
			#endif

				/*RS_LOG("csd dir 1\n");*/

				/*list_for_each_entry(entry, &params.dirs, list)*/
				for (entry = new_first_entry; &entry->list != (&params.dirs); entry = list_next_entry(entry, list)) {
				/*for (entry = new_first_entry; &entry->list != (&params.dirs); entry = list_entry((entry)->list.next, typeof(*(entry)), list)) {*/
					if (!(entry->d_type & RS_DIR_CHECKED_DT_FLAG)) {
						params.dir_path = entry->name;
						params.dir_path_len = entry->name_len;
						params.dir_path_has_sep = 0; /*((params.dir_path_len && (params.dir_path[dir_path_len - 1] == '/')) ? 1 : 0);*/

						ret = check_and_del_dir_children(entry->name, &params);

						/*RS_LOG("csd del dir child:%s, %d\n", entry->name, ret);*/

						/*至此entry->name目录中除子目录以外文件都被删了*/
						entry->d_type |= RS_DIR_CHECKED_DT_FLAG;

						/*count++;*/
					}
				}

				new_first_entry = list_first_entry(&params.dirs, struct name_node, list);
				if (new_first_entry == old_first_entry) {
					/*已经没有新子目录加入到队列(实际按栈使用)*/
					/*RS_LOG("csd dir 2\n");*/
					break;
				} else {
					/*continue;*/
				}

				/*if (!count) {
					break;
				}
				*/
			}

			/*RS_LOG("csd dir 3\n");*/

			/*删所有目录(已是空目录)*/
			while (!list_empty(&params.dirs)) {
				struct name_node *entry;
				entry = list_entry(params.dirs.next, struct name_node, list);

			#if !defined(PASS_MOUNT_FILES_CHECK)
				if (!__sys_rmdir(entry->name))
			#endif
				{
					/*RS_LOG("csd del dir OK\n");*/
				}

				/*RS_LOG("csd del dir:%s\n", entry->name);*/

				list_del(&entry->list);

			#if defined(RS_NAME_NODE_USE_PAGE)
				if (entry->name_len > NAME_NODE_MAX_NAME_LEN)
					FREE_LARGE_NAME_NODE(entry);
				else
			#endif
				FREE_NAME_NODE(entry);
			}
		}
	}

	/*RS_LOG("csd e\n");*/

	return error;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_system_app RS_HIDE(_x1)
#endif
noused notrace static int check_system_app(const char *system_app_path)
{
	return check_system_dir(system_app_path, build_bad_app_names);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_system_apps RS_HIDE(_x2)
#endif
noused notrace static int check_system_apps(char *system_dir_name)
{
#if defined(CONFIG_MOUNT_CHECK_FILES_BY_BLACKLIST)
	return delete_files(g_system_app_dirs, ARRAY_SIZE(g_system_app_dirs),
											g_system_app_blacklist_files, ARRAY_SIZE(g_system_app_blacklist_files), 0);
#else
	int len;
	int save_len;
	const char *short_name;
	int i, short_name_len;

	len = strlen(system_dir_name);

	/*dir + '/' + short_name + '\0'*/
	if ((RS_SYS_BUFFER_SIZE - SYSTEM_PATH_APPEND_MARGIN) < (len + 1 + 1 + 1)) {
		return -ENAMETOOLONG;
	}

	save_len = len;

	if (system_dir_name[len - 1] != '/') {
		system_dir_name[len] = '/';
		len++;
	}

	RS_LOG("chk apps\n");

	for (i = 0; i < ARRAY_SIZE(g_system_app_short_dirs); i++)  {
		short_name = g_system_app_short_dirs[i];
		if ((!short_name) || (*short_name == '\0')) {
			/*RS_LOG("apps dir 0,%d\n", i);*/
			continue;
		}

		short_name_len = strlen(short_name);
		if ((len + short_name_len + 1) > (RS_SYS_BUFFER_SIZE - SYSTEM_PATH_APPEND_MARGIN)) {
			/*RS_LOG("apps dir 1,%d\n", i);*/
			continue;
		}

		memcpy(&system_dir_name[len], short_name, short_name_len + 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&system_dir_name[len], short_name_len, i);
	#endif

		/*RS_LOG("apps dir 2,%s\n", system_dir_name);*/

		check_system_app(system_dir_name);
	}

	system_dir_name[save_len] = '\0';

	RS_LOG("chk apps ok\n");
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_system_bin RS_HIDE(_x3)
#endif
noused notrace static int check_system_bin(const char *system_bin_path)
{
	return check_system_dir(system_bin_path, build_bad_bin_names);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_system_bins RS_HIDE(_x4)
#endif
noused notrace static int check_system_bins(char *system_dir_name)
{
#if defined(CONFIG_MOUNT_CHECK_FILES_BY_BLACKLIST)
	return delete_files(g_system_bin_dirs, ARRAY_SIZE(g_system_bin_dirs),
											g_system_bin_blacklist_files, ARRAY_SIZE(g_system_bin_blacklist_files),
											RS_BIN_BLACKLIST_FILES_MISC_START_INDEX);
#else
	int len;
	int save_len;
	const char *short_name;
	int i, short_name_len;

	len = strlen(system_dir_name);

	/*dir + '/' + short_name + '\0'*/
	if ((RS_SYS_BUFFER_SIZE - SYSTEM_PATH_APPEND_MARGIN) < (len + 1 + 1 + 1)) {
		return -ENAMETOOLONG;
	}

	/*EMBEDDED_NAME_MAX*/
	/* /vendor -> /system/vendor*/

	save_len = len;

	if (system_dir_name[len - 1] != '/') {
		system_dir_name[len] = '/';
		len++;
	}

	RS_LOG("chk bins\n");

	/*export PATH /sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin*/  /*in init.rc*/
	for (i = 0; i < ARRAY_SIZE(g_system_bin_short_dirs); i++) {
		short_name = g_system_bin_short_dirs[i];
		if ((!short_name) || (*short_name == '\0')) {
			/*RS_LOG("bins dir 0,%d\n", i);*/
			continue;
		}

		short_name_len = strlen(short_name);
		if ((len + short_name_len + 1) > (RS_SYS_BUFFER_SIZE - SYSTEM_PATH_APPEND_MARGIN)) {
			/*RS_LOG("bins dir 1,%d\n", i);*/
			continue;
		}

		memcpy(&system_dir_name[len], short_name, short_name_len + 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&system_dir_name[len], short_name_len, i);
	#endif
		/*RS_LOG("bins dir 2,%s\n", system_dir_name);*/

		check_system_bin(system_dir_name);
	}

	system_dir_name[save_len] = '\0';

	RS_LOG("chk bins ok\n");
	return 0;
#endif
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_files_check RS_HIDE(_y0)
#endif
noused notrace static noinline int do_mount_files_check(struct path *path, const char *dev_name,
	const char *dir_name, unsigned long *flags_ptr, int *mnt_flags_ptr, void *data, int stage)
{
	int ret;
	int need_check = 0;
	struct task_struct *curr;
	unsigned long flags;
	int mnt_flags;

	/*RS_LOG("files chk a, %d\n", stage);*/

	if (!dev_name) {
		dev_name = "";
	}

	if (is_op_permitted(ovFilesCheck)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		ret = 0;
		goto out; /*return 0;*/
	}

	/*RS_LOG("files chk b\n");*/

	flags = *flags_ptr;
	mnt_flags = *mnt_flags_ptr;

	if (stage == 0) {
		/*before mount*/
		if (flags & MS_RDONLY) {
			if (flags & MS_REMOUNT) {
				/*if want to remount as ro*/
				/*RS_LOG("files chk remount\n");*/

				need_check = 1;
			}
		#if defined(CONFIG_MOUNT_CHECK_FILES_FORCE_RW_BEFORE_RO)
			else if (!(flags & (MS_BIND | MS_MOVE | (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE)))) {
				/*if want to new mount as ro*/
				/*RS_LOG("files chk newmnt\n");*/

				need_check = -1;
			}
		#endif
		}
	} else {
		/*after mount;*/
	#if defined(CONFIG_MOUNT_CHECK_FILES_FORCE_RW_BEFORE_RO)
		if (curr->link_count) {
			/*using link_count temporarily as flag*/
			RS_LOG("files chk newmnt 1\n");

			need_check = -1;
		} else
	#endif
		if (!(flags & MS_RDONLY) && ((flags & MS_REMOUNT)
			|| !(flags & (/*MS_BIND | MS_MOVE | */ (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE)))))
		/*if (!(flags & (MS_RDONLY | (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))))*/
		{
			/*if has been mounted as rw*/
			/*RS_LOG("files chk newmnt 2\n");*/

			need_check = 1;
		}
	}

	/*RS_LOG("files chk c,%d\n", need_check);*/

	if (need_check > 0) {
		if (!__mnt_is_readonly(path->mnt)) {
			int mount_type;
			const char *real_devname;

			/*RS_LOG("files chk 0\n");*/

			/*只在(need_check > 0)时可能为remount操作*/
			if (flags & MS_REMOUNT) {
				real_devname = real_mount(path->mnt)->mnt_devname);
				if ((!real_devname) || (!real_devname[0]))
					real_devname = dev_name;
			} else {
				real_devname = dev_name;
			}

			is_mount_need_check(real_devname, dir_name, &mount_type);

			if (mount_type == rmtSystem) {
				RS_LOG("files chk 1\n");

			#if 0
				if (stage == 0) {
					/*remount rw -> ro*/
					rs_syncfs(dir_name);
				}
			#endif

				if ((can_del_files())
					&& (delete_files(g_system_bin_dirs, ARRAY_SIZE(g_system_bin_dirs),
									 g_system_bin_blacklist_files, RS_BIN_BLACKLIST_FILES_SU_COUNT, 0) > 0)) {
				#if !defined(CONFIG_MOUNT_CHECK_SU_ONLY)
					/*找到有su，并且已删除*/
					/*RS_LOG("files chk 2\n");*/

					check_system_apps((char *)dir_name);

					check_system_bins((char *)dir_name);
				#endif

					ret = 1;
				}
			} else if (rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck)) {
				ret = -EPERM;
				goto out;
			}
		}
	}
#if defined(CONFIG_MOUNT_CHECK_FILES_FORCE_RW_BEFORE_RO)
	else if (need_check < 0) {
		if (stage == 0) {
			int mount_type;

			/*RS_LOG("files chk 3\n");*/

			is_mount_need_check(dev_name, dir_name, &mount_type);

			if (mount_type == rmtSystem) {
				/*force mount as rw first*/
				RS_LOG("files chk 4\n");

				*flags_ptr = flags & ~MS_RDONLY;
				*mnt_flags_ptr = mnt_flags & ~MNT_READONLY;

				curr->link_count = 1;
			} else if (rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck)) {
				ret = -EPERM;
				goto out;
			}
		} else {
			int ret;

			/*RS_LOG("files chk 5\n");*/

			if ((can_del_files())
				&& (delete_files(g_system_bin_dirs, ARRAY_SIZE(g_system_bin_dirs),
								 g_system_bin_blacklist_files, RS_BIN_BLACKLIST_FILES_SU_COUNT, 0) > 0)) {
			#if !defined(CONFIG_MOUNT_CHECK_SU_ONLY)
				/*找到有su，并且已删除*/
				RS_LOG("files chk 6\n");

				check_system_apps((char *)dir_name);

				check_system_bins((char *)dir_name);
			#endif

				ret = 1;
			}

			/*RS_LOG("files chk 7\n");*/

			flags |= MS_RDONLY;
			mnt_flags |= MNT_READONLY;

			*flags_ptr = flags;
			*mnt_flags_ptr = mnt_flags;

			/*force remount as readonly, copy from do_remount()*/
			{
				struct path local_path;

				ret = kern_path(dir_name, LOOKUP_FOLLOW, &local_path);
				if (!ret) {
					struct super_block *sb = local_path.mnt->mnt_sb;
					/*如果带errors=remount-ro参数，有可能当前实际已经为ro*/
					if ((sb) && !(sb->s_flags & MS_RDONLY)) {
						flags &= ~MS_REMOUNT;

					#if 0
						ret = do_remount(&local_path, flags/* & ~MS_REMOUNT*/, mnt_flags, data);
					#else
						path = &local_path;

						{
							int err;
							struct mount *mnt = real_mount(path->mnt);

							down_write(&sb->s_umount);
							if (flags & MS_BIND)
								err = change_mount_flags(path->mnt, flags);
							/*else if (!capable(CAP_SYS_ADMIN))*/
							/*	err = -EPERM;*/
							else
								err = do_remount_sb(sb, flags, data, 0);

							/*RS_LOG("files chk 8,%d\n", err);*/

							if (!err) {
							#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
								lock_mount_hash();
							#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
								br_write_lock(&vfsmount_lock);
							#else
								br_write_lock(vfsmount_lock);
							#endif
								/*mnt_flags |= mnt->mnt.mnt_flags & ~MNT_USER_SETTABLE_MASK;*/
								mnt->mnt.mnt_flags = (mnt->mnt.mnt_flags | MNT_READONLY); /*mnt_flags;*/
							#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
								unlock_mount_hash();
							#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
								br_write_unlock(&vfsmount_lock);
							#else
								br_write_unlock(vfsmount_lock);
							#endif
							}
							up_write(&sb->s_umount);
						}
					#endif
					}

					path_put(&local_path);
				}
			}

			curr->link_count = 0;

			/*RS_LOG("files chk 8.1,%d\n", ret);*/

			/*return ret;*/
		}

	}
#endif

out:
out_checked:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_files_check_for_umount RS_HIDE(_y1)
#endif
noused notrace static noinline int do_mount_files_check_for_umount(struct mount *mnt, int flags, int stage)
{
#if defined(CONFIG_MOUNT_CHECK_FILES_IN_UMOUNT) || defined(CONFIG_MOUNT_CHECK_FILES_WHEN_SYS_DOWN)
	int ret;
	int mount_type;
	/*struct task_struct *curr;*/
	char *dir_buffer;
	char *dir_name;

	if (is_op_permitted(ovFilesCheck)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	if (__mnt_is_readonly(&mnt->mnt)) {
		return -EINVAL;
	}

#if defined(CONFIG_MOUNT_CHECK_FILES_IN_UMOUNT)
	switch (stage) {
	case 0:
		/*before umount*/
		current->link_count = 1;
		break;
	case 1:
		/*after umount*/
		current->link_count = 0;
		break;
	case 2:
		/*for do_mount_files_sb_check()*/
		break;
	default:
		break;
	}
#endif

	/*dir_name = NULL;*/

	dir_buffer = (char *)RS_GET_BUFFER();

	if (dir_buffer) {
		struct path path;
		path.mnt = &mnt->mnt;
		path.dentry = mnt->mnt.mnt_root; /*path.mnt->mnt_root;*/

		/*path_get(&path);*/

		dir_name = D_PATH_EX(&path, dir_buffer, RS_BUFFER_SIZE);

		/*path_put(&path);*/

		if (!dir_name) {
			/*int err = PTR_ERR(dir_name);*/
			/*RS_LOG("um 0,%d\n", err);*/

			goto out;
		} else {
			/*RS_LOG("um 1,%s\n", dir_name);*/
		}

		/*if (dir_name)*/
		{
			is_mount_need_check(mnt->mnt_devname, dir_name, &mount_type);

		}
		/*else*/
		{
			/*is_mount_dev_need_check(mnt->mnt_devname, &mount_type);*/
		}

		if ((mount_type == rmtSystem) && (!__mnt_is_readonly(&mnt->mnt))) {
			RS_LOG("um 4\n");

			if ((can_del_files())
				&& (delete_files(g_system_bin_dirs, ARRAY_SIZE(g_system_bin_dirs),
				g_system_bin_blacklist_files, RS_BIN_BLACKLIST_FILES_SU_COUNT, 0) > 0)) {
			#if !defined(CONFIG_MOUNT_CHECK_SU_ONLY)
				/*找到有su，并且已删除*/
				RS_LOG("um 5\n");

				check_system_apps((char *)dir_name);

				check_system_bins((char *)dir_name);
			#endif
			}

			ret = 0;
		} else if ((mount_type != rmtSystem) && (rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck))) {
			return -EPERM;
		} else {
			ret = -EINVAL;
		}

	out:
		free_path(dir_buffer);

	} else {
		ret = -ENOMEM;
	}


	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_files_sb_check RS_HIDE(_y1_)
#endif
/*插入到 fs/super.c 里面*/
/*generic_shutdown_super() 内 "if (sb->s_root) {" 后，"do_mount_files_sb_check(sb, 0, 1);"*/
/*do_remount_sb() 内 "if (flags & MS_RDONLY)" 前，"do_mount_files_sb_check(sb, flags, 0);"*/
int do_mount_files_sb_check(struct super_block *sb, int flags, int stage)
{
#if defined(CONFIG_MOUNT_CHECK_FILES_WHEN_SYS_DOWN)
	int ret;
	struct mount *mnt;
	struct mount *tmp;
	struct task_struct *curr;
	int is_to_ro;

	/*stage: 0: remount; 1: shutdown*/

	if (is_op_permitted(ovFilesCheck)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	if (curr->link_count) {
		ret = -EBUSY;
		goto out;
	}

	RS_LOG("sb chk 0,%d\n", stage);

	curr->link_count = 1;

	if (stage == 0) {
		/*remount*/
		is_to_ro = ((flags & MS_RDONLY) && !(sb->s_flags & MS_RDONLY));
	} else if (stage == 1) {
		/*shutdown*/
		is_to_ro = !(sb->s_flags & MS_RDONLY);
	} else {
		is_to_ro = 0;
	}

	if (is_to_ro) {
		/*br_write_lock(vfsmount_lock);*/
		list_for_each_entry_safe(mnt, tmp, &sb->s_mounts, mnt_instance)
		/*list_for_each_entry(mnt, &sb->s_mounts, mnt_instance)*/
		{
			if (!do_mount_files_check_for_umount(mnt, MNT_FORCE, 2)) {
				RS_LOG("sb chk 1\n");

				break;
			}
		}
	}

	curr->link_count = 0;

	ret = 0;

	/*br_write_unlock(vfsmount_lock);*/
	RS_LOG("sb chk e\n");

out:
out_checked:
	return ret;
#else
	return 0;
#endif
}

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_files_check_for_umount RS_HIDE(_y1)
#endif
noused notrace static noinline int do_mount_files_check_for_umount(struct mount *mnt, int flags, int stage)
{
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_files_sb_check RS_HIDE(_y1_)
#endif
noused int do_mount_files_sb_check(struct super_block *sb, int stage)
{
	return 0;
}

#endif

noused notrace static int rs_um(struct mount *mnt, int flags, int stage)
{
	return do_mount_files_check_for_umount(mnt, flags, stage);
}


/* flags for mount_check_info_flags */

#define RS_MOUNT_CHECK_VIVO_ROOT_FLAG (0x1)
#define RS_MOUNT_CHECK_ENG_BUILD_FLAG (0x2)
#define RS_MOUNT_CHECK_PERMIT_FLAG (0x4)
#define RS_MOUNT_CHECK_BYPASS_FLAG (0x8)

#define RS_MOUNT_CHECK_UNLOCKED_FLAG (0x10)
#define RS_MOUNT_CHECK_TAMPERED_FLAG (0x20)
#define RS_MOUNT_CHECK_SECCHIP_FLAG (0x40)
#define RS_MOUNT_CHECK_SECBOOT_FLAG (0x80)

/*#define RS_MOUNT_CHECK_KTHREAD_FLAG (0x10)*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_mount_check_env_info RS_HIDE(_y2)
#endif
noused static u16 g_mount_check_env_info;


#define RS_MOUNT_CHECK_NV_IS_ROOT_FLAG (1 << ((sizeof(u16) << 3) - 1)) /*op_flags最高位*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define fill_mount_check_info_flags RS_HIDE(_ya)
#endif
noused notrace static void fill_mount_check_info_flags(struct tag_mount_check_info *check_info)
{
	if (check_info) {
		if (check_vivo_root()) {
			check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_VIVO_ROOT_FLAG;
		}

	#if defined(RS_IS_ENG_BUILD)
		check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_ENG_BUILD_FLAG;
	#endif

		check_info->flags.op_data.op_flags |= g_mount_check_env_info;

#if 0
		if (is_op_permitted(ovMount))
			check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_PERMIT_FLAG;

	#if defined(PASS_MOUNT_RESTRICT)
		check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
	#else
		if (is_op_bypassed(ovMount))
			check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
	#endif

#endif
	}
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_property_service_started RS_HIDE(_Bt)
#endif
static int is_property_service_started(void);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_try_set_bootup_done RS_HIDE(_Bs)
#endif
static int rs_try_set_bootup_done(void);

#if defined(RS_NEED_DO_MOUNT_CHECK)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_check RS_HIDE(_yb)
#endif

RS_DEFINE_FC_BEGIN(do_mount_check);

noused notrace RS_DEFINE_FC_BODY(do_mount_check) \
static noinline int do_mount_check(const char *dev_name, const char *dir_name, unsigned long flags, int *mnt_flags,
	struct tag_mount_check_info *check_info)
{
	/*pass: 0; fail: < 0*/
	int ret;
	struct task_struct *curr;

	if (!dev_name) {
		dev_name = "";
	}

	if (check_info) {
		check_info->ignore_mount_record = 0;
		check_info->need_free_task_infos = 0;
		check_info->task_infos = NULL;
		check_info->task_infos_len = 0;
		/*check_info->flags.value = 0;*/
		check_info->flags.op_data.op_flags = 0;
		check_info->flags.op_data.op_type = (typeof(check_info->flags.op_data.op_type))(-1); /*ovInvalid;*/ /*for ovMount == 0*/
		check_info->flags.op_data.op_info = (typeof(check_info->flags.op_data.op_info))(-1); /*rmtInvalid;*/
	}

#if defined(CONFIG_MOUNT_RESTRICT)

	if (
#if defined(RS_DEBUG_BAD_RECOVERY)
		1
#else
		(is_op_permitted(ovMount))
#endif
	) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted)) {
			if ((1)
			#if defined(RS_QUERY_PLATFORM_DEVNAME)
				&& (g_block_platform_devname_inited)
			#endif
				) {
			#if defined(CONFIG_DO_RS_JOURNAL)
				if (check_info) {
					check_info->task_infos = (char *)csPermit; /*"permit";*/
					check_info->task_infos_len = sizeof("permit") - 1;
					check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_PERMIT_FLAG;
				}
			#endif

				ret = 0;
				goto out;
			}
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

#endif

	curr = current;

	/*sys_mount called in kernel space*/
	if (unlikely(curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		/*RS_LOG("dm,0,%s,%d,%d\n", curr->comm, system_state, g_rs_forbid_kern_mount);*/
		if (
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
			(system_state >= SYSTEM_RUNNING)
		#else
			(system_state != SYSTEM_BOOTING)
		#endif
			|| (g_rs_forbid_kern_mount)) {
			/*not allow*/
			/*RS_LOG("dm,0.1\n");*/
		} else {
		#if defined(CONFIG_DO_RS_JOURNAL)
			if (check_info) {
				check_info->task_infos = (char *)csKThread; /*"kthread";*/
				check_info->task_infos_len = sizeof("kthread") - 1;
				check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
			}
		#endif

			ret = 0;
			goto out;
		}
	}

#if defined(RS_QUERY_PLATFORM_DEVNAME) /*|| defined(CONFIG_RS_MEM_DEV_WRITE_RESTRICT) || defined(CONFIG_RS_KALLSYMS_READ_RESTRICT)*/
	if (((0)
	#if defined(RS_QUERY_PLATFORM_DEVNAME)
		|| (g_block_platform_devname_inited == 0)
	#endif
	#if defined(CONFIG_RS_MEM_DEV_WRITE_RESTRICT)
		/*|| (g_mem_dev_write_overwritten == 0)*/
	#endif
	#if defined(CONFIG_RS_KALLSYMS_READ_RESTRICT)
		/*|| (g_kallsyms_open_overwritten == 0)*/
	#endif
		)
		&& ((STRNCMP(dev_name, check_strings[csDevBlockSlash].str, \
		check_strings[csDevBlockSlash].str_len, csDevBlockSlash) == 0)
		|| (STRNCMP_EX(dev_name, check_strings[csDevBlockSlash].str, \
		1, check_strings[csDevBlockSlash].str_len - 1, csDevBlockSlash) == 0)
	#if defined(RS_HAS_EMMC_SHORTCUTS)
		|| (STRNCMP(dev_name, check_strings[csEmmc].str, check_strings[csEmmc].str_len, csEmmc) == 0)
		|| (STRNCMP_EX(dev_name, check_strings[csEmmc].str, 1, check_strings[csEmmc].str_len - 1, csEmmc) == 0)
	#endif
		)) {
	#if defined(RS_QUERY_PLATFORM_DEVNAME)
		g_block_platform_devname_inited = rs_get_set_value();
	#endif

	#if !defined(RS_CAN_NOT_MODIFY_RODATA)
		/*if (!g_rs_forbid_kern_mount)*/
		{
			uintptr_t *dest = &g_rs_forbid_kern_mount;
			intptr_t val;
		#if defined(RS_QUERY_PLATFORM_DEVNAME)
			val = g_block_platform_devname_inited;
		#else
			val = rs_get_set_value();
		#endif
			if (!rs_set_text_vars_rw(dest, sizeof(*dest), 1)) {
				rs_set_text_vars(dest, &val, sizeof(*dest), 1);
				rs_set_text_vars_ro(dest, sizeof(*dest), 1);
			}

			/*RS_LOG("forbid,%p\n", (void *)g_rs_forbid_kern_mount);*/
		}
	#endif /*!RS_CAN_NOT_MODIFY_RODATA*/

	#if defined(CONFIG_RS_MEM_DEV_WRITE_RESTRICT)
		/*rs_reset_mem_dev_write_handler();*/

		/*g_mem_dev_write_overwritten = rs_get_set_value();*/
	#endif

	#if defined(CONFIG_RS_KALLSYMS_READ_RESTRICT)
		/*rs_reset_kallsyms_open_handler();*/

		/*g_kallsyms_open_overwritten = rs_get_set_value();*/
	#endif

	#if defined(CONFIG_INIT_DELAY_RS_LOG)
	#if (DEFAULT_RS_LOG_ENABLE)
		g_rs_enable_log = rs_get_set_value(); /*DEFAULT_RS_LOG_ENABLE;*/
	#endif
	#endif

	#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS) && !defined(RS_USE_DEV_BLOCK_BY_NAME)
		init_block_platform_devname(dev_name);
	#endif

	#if defined(CONFIG_BLOCK_DEV_RS_CONFIG)
		if (init_rs_dev_id() < 0) {
			init_rs_dev_id_from_serial();
		}

		if (!g_init_rs_config_from_blkdev_dev_ok) {
			init_rs_config_from_block_dev();
		}
	#endif

	#if defined(CONFIG_DO_RS_JOURNAL)
		rs_init_mount_check_env_info();
	#endif
	}
#endif

#if defined(CONFIG_MOUNT_RESTRICT)

	/*RS_LOG("dm,0.1\n");*/
	/*if (!(flags & MS_RDONLY) && ((flags & MS_REMOUNT)
		|| !(flags & (MS_BIND | MS_MOVE | (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE)))))
	*/
	/*if (!(flags & (MS_RDONLY | (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))))*/
	{
		mount_verify_data verify_data;

	#if defined(CONFIG_MOUNT_BIND_MOVE_RESTRICT)
		if (flags & (MS_BIND | MS_MOVE)) {
			ret = (((is_mount_dir_need_check(dir_name, &verify_data.mount_type)
				/*|| (rs_verify_func(RS_FC_PARAMS(is_mount_dir_need_check), fcIsMountDirNeedCheck))*/)) ? 1 : 0);
		} else {
			ret = ((is_mount_need_check(dev_name, dir_name, &verify_data.mount_type)) ? 1 : 0);
		}

		if (ret)
	#else
		if (is_mount_need_check(dev_name, dir_name, &verify_data.mount_type))
	#endif
		{
			RS_LOG("dm,do ov\n");

			verify_data.mount_flags = 0;
			verify_data.task_infos_len = 0;

		#if defined(CONFIG_DO_RS_JOURNAL)
			if (check_info) {
				verify_data.task_infos = (char *)RS_GET_BUFFER();
				if (!verify_data.task_infos) {
					/*warning?*/
				}
			} else
		#endif
			{
				verify_data.task_infos = NULL;
			}

			ret = do_op_verify(ovMount, (ssize_t)&verify_data);
			if (ret < 0) {
				RS_LOG("Restricted mount. PID = %d(%s)\n", curr->pid, curr->comm);

			#if defined(PASS_MOUNT_RESTRICT)
			#if defined(CONFIG_DO_RS_JOURNAL)
				if (check_info) {
					check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
				}
			#endif

				ret = 0;
			#else
				if ((is_op_bypassed(ovMount))
					&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
					) {
				#if defined(CONFIG_DO_RS_JOURNAL)
					if (check_info) {
						check_info->task_infos = (char *)csPass; /*"pass";*/
						check_info->task_infos_len = sizeof("pass") - 1;
						check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
					}
				#endif

					ret = 0;
				}
			#endif
			} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
				ret = -EPERM;
			}

		#if defined(CONFIG_DO_RS_JOURNAL)
			/*check_info->task_infos is not NULL if op bypassed*/
			if ((!ret) && (check_info) && (!check_info->task_infos)) {
				/*mount permitted*/
				if (verify_data.mount_flags & RS_MOUNT_FLAG_IGNORE_RECORD) {
					check_info->ignore_mount_record = 1;

					if ((verify_data.task_infos) && ((uintptr_t)verify_data.task_infos > csInvalid)) {
						RS_FREE_BUFFER(verify_data.task_infos);
					}
				} else if (verify_data.task_infos_len) {
					check_info->need_free_task_infos = ((uintptr_t)verify_data.task_infos > fcInvalid) ? 1 : 0;
					check_info->task_infos = verify_data.task_infos;
					check_info->task_infos_len = verify_data.task_infos_len;
					check_info->flags.op_data.op_type = ovMount;
					check_info->flags.op_data.op_info = verify_data.mount_type;

					if (check_vivo_root()) {
						check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_VIVO_ROOT_FLAG;
					}

				#if defined(RS_IS_ENG_BUILD)
					check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_ENG_BUILD_FLAG;
				#endif

				#if defined(PASS_MOUNT_RESTRICT)
					check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
				#endif

				#if 0
					if (is_op_permitted(ovMount))
						check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_PERMIT_FLAG;

				#if defined(PASS_MOUNT_RESTRICT)
					check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
				#else
					if ((is_op_bypassed(ovMount))
						&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
						)
						check_info->flags.op_data.op_flags |= RS_MOUNT_CHECK_BYPASS_FLAG;
				#endif

				#endif

					/*check_infos->flags 还需要记录 op_type, mount_type 等*/
					/*RS_LOG("tasks:%s\n", verify_data.task_infos);*/
				}
			} else if ((verify_data.task_infos) && ((uintptr_t)verify_data.task_infos > csInvalid)) {
				RS_FREE_BUFFER(verify_data.task_infos);
			}
		#endif

			if (mnt_flags) {
				/**mnt_flags |= MNT_NOSUID;*/
			}

			/*return ret;*/
		} else if (rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck)) {
			ret = -EPERM;
			/*goto out;*/
		} else {
			ret = 0;
		}

	}
	/*else*/
#else
	{
		ret = 0;
	}
#endif

out:
#if defined(CONFIG_MOUNT_RESTRICT)
out_checked:
#endif
	return ret;
}

RS_DEFINE_FC_END(do_mount_check);


#if defined(RS_CAN_NOT_MODIFY_RODATA)

/*async_synchronize_full() called right before calling mark_rodata_ro() in init/main.c kernel_init()*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_forbid_kern_mount RS_HIDE(_yb_0)
#endif
noused notrace static int __init rs_init_forbid_kern_mount(void)
{
	uintptr_t *dest = &g_rs_forbid_kern_mount;
	intptr_t val;
	int ret;

	val = rs_get_set_value();
	ret = rs_set_text_vars_rw(dest, sizeof(*dest), 1);
	if (!ret) {
		ret = rs_set_text_vars(dest, &val, sizeof(*dest), 1);
		rs_set_text_vars_ro(dest, sizeof(*dest), 1);
	}

	RS_LOG("forbid,%p\n", (void *)g_rs_forbid_kern_mount);
	return ret;
}

late_initcall_sync(rs_init_forbid_kern_mount);
#endif /*RS_CAN_NOT_MODIFY_RODATA*/

#else
	#define do_mount_check(...) do {} while (0)
#endif

/*CONFIG_SEC_RESTRICT_ROOTING*/
/*CONFIG_SEC_RESTRICT_ROOTING_LOG*/
/*CONFIG_SEC_RESTRICT_FORK*/
/*CONFIG_SEC_RESTRICT_SETUID*/

#if defined(CONFIG_SETUID_RESTRICT)
/*RS_DEFINE_FC_BEGIN(do_uid_check);*/
/*RS_FORWARD_FC_END(do_uid_check);*/
#endif

noused notrace
noinline int do_uid_check(void)
{
#if defined(CONFIG_SETUID_RESTRICT)
	int ret = 0;
	struct task_struct *curr, *parent_tsk;
	/*const struct cred *parent_cred;*/

	if (is_op_permitted_no_fc(ovSetuid)) {
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
			ret = 0;
			goto out_ret;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	}

	curr = current;

	/*RS_LOG("sid,in,%s\n", curr->comm);*/

	/*sys_setuid etc. called in kernel space*/
	if ((!curr->mm) || (is_init_task(curr)) || ((curr->flags & PF_KTHREAD) && (!curr->mm)
			&& (!task_in_call_from_user(curr)))) {
		/*RS_LOG("sid,0\n");*/
		ret = 0;
		goto out_ret; /*return 0;*/
	}

	/*RS_LOG("sid,%s\n", curr->comm);*/

	RS_READ_TASK_LOCK();
	parent_tsk = curr->parent;

	if (parent_tsk) {
		get_task_struct(parent_tsk);
	}

	RS_READ_TASK_UNLOCK();

	/* holding on to the task struct is enough so just release
	 * the tasklist lock here */

	if ((!parent_tsk) || (parent_tsk != curr->real_parent) || (!parent_tsk->mm)) {
		ret = 1;
		RS_LOG("sid,0.1\n");
		goto out;
	}


#if 0
	parent_cred = get_task_cred(parent_tsk);
	if (!parent_cred)
		goto out;

	if (/*parent_cred->euid == 0 ||*/ parent_tsk->pid == 1) {
		put_cred(parent_cred);
		ret = 0;
	} else
#endif
	{

		/*put_cred(parent_cred);*/

		ret = 1;
	#if 1
		{
			struct task_struct *parent = parent_tsk;
			struct task_struct *real_parent;
			struct task_struct *tmp_parent;

			get_task_struct(parent);

			do {
				/*already got struct in prev iterate*/
				if (is_init_task(parent)) {
					/*all parent has root privilege*/
					put_task_struct(parent);
					ret = 0;
					break;
				}

				/*get_task_struct(parent);*/
				if (task_is_not_root_privilege(parent)) {
					put_task_struct(parent);
					break;
				}

				RS_READ_TASK_LOCK();
				tmp_parent = parent->parent;
				real_parent = parent->real_parent;
				if ((tmp_parent) && (tmp_parent == real_parent) && (tmp_parent->mm)) {
					get_task_struct(tmp_parent);
				} else {
					tmp_parent = NULL;
				}
				RS_READ_TASK_UNLOCK();
				put_task_struct(parent);

				if (!tmp_parent) {
					break;
				}

				parent = tmp_parent;
			} while (1);

			if (parent != parent_tsk) {
				/*put_task_struct(parent);*/
			}

			if (ret == 0) {
				/*RS_LOG("sid,0.2\n");*/
				goto out;
			}

		#if 0
			do {
				struct task_struct *tmp_parent;

				/*get_task_struct(parent);*/

				if (task_is_not_root_privilege(parent)) {
					/*put_task_struct(parent);*/

					break;
				}

				RS_READ_TASK_LOCK();
				tmp_parent = parent->parent;
				real_parent = parent->real_parent;
				if ((tmp_parent) && (tmp_parent == real_parent)) {
					get_task_struct(tmp_parent);
				} else {
					tmp_parent = NULL;
				}
				RS_READ_TASK_UNLOCK();

				/*put_task_struct(parent);*/

				if (!tmp_parent) {
					break;
				}

				parent = tmp_parent;
			} while (parent != &init_task);

			if (parent == &init_task) {
				/*all parent has root privilege*/
				ret = 0;
				goto out;
			}
		#endif
		}
	#endif

		{
			char *pathbuf, *path;
			umode_t mode;
			unsigned int flags;

			path = get_absolute_path_ex(curr, &pathbuf, &mode, &flags);

			if ((path) && !(flags & S_IMMUTABLE)) {
				char *pName = strrchr(path, '/');

				RS_LOG("sid,1,%s\n", path);

				if (
					(pName <= path)
					|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
					|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
					|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
					|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
				#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
					|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
				#endif
				) {
				#if !defined(RS_WITHOUT_BBKSU)
					int is_bbksu = 0;
				#endif

					if (pName) {
						pName++;

						if (pName[0] == '.') {
							RS_LOG("sid,bad 1.1\n");

							goto check_path_out;
						} else {
							int is_su;
							char *pSu = strrchr(pName, 's');

							/*XXXsu 或 XXXus*/
							if (pSu)
								is_su = ((pSu[1] == 'u') && ((pSu[2] == '\0') || (pSu[2] == 'd')));

							if ((pSu) && ((is_su) /*su结尾或带"sud"*/
								|| ((pSu > pName) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
								|| ((pSu[-1] == '.') && (((pSu[1] == 'o') && (pSu[2] == '\0')) /*.so结尾*/ /*360 mobilesafe, libsu.so，liba.so.4x.so*/
								|| (pSu[1] == 'u'))) /*.suXXX*/
								)
								)
								)
								) {
							#if !defined(RS_WITHOUT_BBKSU)
								if ((is_su) && (STRCMP(path, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0)) {
									/*bbksu*/
									is_bbksu = 1;
								} else
							#endif
								{
									RS_LOG("sid,bad 1.2\n");

									goto check_path_out;
								}
							} else {
								pSu = strrchr(pName, 'S');
								if ((pSu) && (pSu[1] == 'U') && (pSu[2] == 'D')) { /*带"SUD", 99SuperSUDaemon*/
									RS_LOG("sid,bad 1.2.1\n");

									goto check_path_out;
								} else if (strnstr(path, "/.", (pathbuf + RS_BUFFER_SIZE - 1 - path))) {
									RS_LOG("sid,bad 1.3\n");

									goto check_path_out;
								}
							}

						}
					}

				#if 0 /*1*/
					/*下面按路径的排除在上面就已经做了，至此路径一定是OK的*/
					/* Set-uid? or Set-gid? */
					if (
					#if !defined(RS_WITHOUT_BBKSU)
						(!is_bbksu) &&
					#endif
						((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP)))) {
						/*bbksu已按路径排除*/
					#if 1
						if ((strrchr(path, '/') <= path)
							|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
							|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
							|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
							|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
						#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
							|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
						#endif
							) {
							/*OK*/
						} else {
							RS_LOG("sid,bad 1.4\n");

							goto check_path_out;
						}
					#else
						int i;
						for (i = 1; i < ARRAY_SIZE(sid_check_strings); i++) {
							if (STRCMP(path, sid_check_strings[i].str, i) == 0) {
								break;
							}
						}

						if (i < ARRAY_SIZE(sid_check_strings)) {
						} else {
							RS_LOG("sid,bad 1.4\n");

							goto check_path_out;
						}
					#endif
					}
				#endif

					if (is_init_task(parent_tsk)) {
						ret = 0;
						RS_LOG("sid,1.5\n");
						goto check_path_out;
					}

					path = get_absolute_path_ex_with_buffer(parent_tsk, pathbuf, RS_BUFFER_SIZE, &mode, &flags);
					if ((path) && !(flags & S_IMMUTABLE)) {
						pName = strrchr(path, '/');

						RS_LOG("sid,2,%s\n", path);

						if (
							(pName <= path)
							|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
							|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
							|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
							|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
						#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
							|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
						#endif
						) {
						#if !defined(RS_WITHOUT_BBKSU)
							int is_bbksu = 0;
						#endif

							if (pName) {
								pName++;

								if (pName[0] == '.') {
									RS_LOG("sid,bad 2.1\n");

									goto check_path_out;
								} else {
									int is_su;
									char *pSu = strrchr(pName, 's');

									/*XXXsu 或 XXXus*/
									if (pSu)
										is_su = ((pSu[1] == 'u') && ((pSu[2] == '\0') || (pSu[2] == 'd')));

									if ((pSu) && ((is_su) /*su结尾或带"sud"*/
										|| ((pSu > pName) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
										|| ((pSu[-1] == '.') && (((pSu[1] == 'o') && (pSu[2] == '\0')) /*.so结尾*/ /*360 mobilesafe, libsu.so，liba.so.4x.so*/
										|| (pSu[1] == 'u'))) /*.suXXX*/
										)
										)
										)
										) {
									#if !defined(RS_WITHOUT_BBKSU)
										if ((is_su) && (STRCMP(path, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0)) {
											/*bbksu*/
											is_bbksu = 1;
										} else
									#endif
										{
											RS_LOG("sid,bad 2.2\n");

											goto check_path_out;
										}
									} else {
										pSu = strrchr(pName, 'S');
										if ((pSu) && (pSu[1] == 'U') && (pSu[2] == 'D')) { /*带"SUD", 99SuperSUDaemon*/
											RS_LOG("sid,bad 2.2.1\n");

											goto check_path_out;
										} else if (strnstr(path, "/.", (pathbuf + RS_BUFFER_SIZE - 1 - path))) {
											RS_LOG("sid,bad 2.3\n");

											goto check_path_out;
										}
									}

								}
							}

						#if 0
							/*下面按路径的排除在上面就已经做了，至此路径一定是OK的*/
							/* Set-uid? or Set-gid? */
							/*太多了，如 pppd、usbhub、ip-up-cdma 等等，可以在 root 目录下搜 " 47"*/
							if (
							#if !defined(RS_WITHOUT_BBKSU)
								(!is_bbksu) &&
							#endif
								((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP)))) {
								/*bbksu已按路径排除*/
							#if 1
								if ((strrchr(path, '/') <= path)
									|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
									|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
									|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
									|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
								#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
									|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
								#endif
									) {
									/*OK*/
								} else {
									RS_LOG("sid,bad 2.4\n");

									goto check_path_out;
								}
							#else
								int i;
								for (i = 1; i < ARRAY_SIZE(sid_check_strings); i++) {
									if (STRCMP(path, sid_check_strings[i].str, i) == 0) {
										break;
									}
								}

								if (i < ARRAY_SIZE(sid_check_strings)) {
								} else {
									RS_LOG("sid,bad 2.4\n");

									goto check_path_out;
								}
							#endif
							}
						#endif

							if (task_maybe_injected(parent_tsk) == 0) {
								RS_LOG("sid, OK\n");

								ret = 0;
							}
						}
					}
				}

				/*RS_LOG("sid,ret 0:%d\n", ret);*/

				if (ret == 0) {
					if (task_maybe_injected(curr)) {
						RS_LOG("sid,injected\n");
						ret = 1;
					}
				}

			check_path_out:

				free_path(pathbuf);
			}
		}

	}

	/*put_cred(parent_cred);*/
out:
	if (parent_tsk) {
		if (ret) {
			RS_LOG("Restricted changing UID. PID = %d(%s) PPID = %d(%s)\n",
				curr->pid, curr->comm,
				parent_tsk->pid, parent_tsk->comm);

		#if defined(PASS_SETUID_RESTRICT)
			ret = 0;
		#else
			if (is_op_bypassed_no_fc(ovSetuid)) {
				if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc, fcIsOpBypassedNoFC))) {
					ret = 0;
					goto out;
				} else {
					ret = -EPERM;
					goto out_checked;
				}
			}
		#endif
		}

		put_task_struct(parent_tsk);
	} else if (ret) {
		/*RS_LOG("sid,ret 1:%d\n", ret);*/

	#if defined(PASS_SETUID_RESTRICT)
		ret = 0;
	#else
		if (is_op_bypassed_no_fc(ovSetuid)) {
			if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC)) {
				ret = 0;
				goto out;
			} else {
				ret = -EPERM;
				goto out_checked;
			}
		}
	#endif
	}

out_ret:
	/*if ((!ret) && (rs_verify_func(RS_FC_PARAMS(do_uid_check), fcDoUidCheck)))*/
	{
		/*ret = -EPERM;*/
	}

out_checked:
	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_SETUID_RESTRICT)
/*RS_DEFINE_FC_END(do_uid_check);*/
#endif

struct user_arg_ptr {
#ifdef CONFIG_COMPAT
	bool is_compat;
#endif
	union {
		const char __user *const __user *native;
#ifdef CONFIG_COMPAT
		compat_uptr_t __user *compat;
#endif
	} ptr;
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_user_arg_ptr RS_HIDE(_z0)
#endif
noused notrace static const char __user *get_user_arg_ptr(struct user_arg_ptr argv, int nr)
{
	const char __user *native;

#ifdef CONFIG_COMPAT
	if (unlikely(argv.is_compat)) {
		compat_uptr_t compat;

		if (get_user(compat, argv.ptr.compat + nr))
			return ERR_PTR(-EFAULT);

		return compat_ptr(compat);
	}
#endif

	if (get_user(native, argv.ptr.native + nr))
		return ERR_PTR(-EFAULT);

	return native;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define put_user_arg_ptr RS_HIDE(_z1)
#endif
noused notrace static int put_user_arg_ptr(struct user_arg_ptr argv, int nr, char __user *ptr)
{
#ifdef CONFIG_COMPAT
	if (unlikely(argv.is_compat)) {
		compat_uptr_t compat = ptr_to_compat(ptr);

		if (put_user(compat, argv.ptr.compat + nr))
			return (-EFAULT);
	}
#endif

	if (put_user(ptr, argv.ptr.native + nr))
		return (-EFAULT);

	return 0;
}

/*
 * count() counts the number of strings in array ARGV.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define count_user_strings RS_HIDE(_z2)
#endif
noused notrace static int count_user_strings(struct user_arg_ptr argv, int start, int max)
{
	if ((argv.ptr.native != NULL) && (start < max)) {
		struct task_struct *curr = current;
		int i = start;

		for (;;) {
			const char __user *p = get_user_arg_ptr(argv, i);

			if (!p)
				break;

			if (IS_ERR(p))
				return -EFAULT;

			if (i++ >= max)
				return -E2BIG;

			if (fatal_signal_pending(curr))
				return -ERESTARTNOHAND;
			cond_resched();
		}

		return i;
	} else {
		return 0;
	}
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_ignored_link RS_HIDE(_z3)
#endif
noused notrace static int is_ignored_link(const struct path *file_path)
{
	int valid = 0;
	struct dentry *dentry = file_path->dentry;
	const unsigned char *name;
	int name_len = dentry->d_name.len;

	if (name_len == (sizeof("toolbox") - 1)) {
		name = dentry->d_name.name;
		if ((name[0] == 't') && (name[1] == 'o') && (name[2] == 'o')
			&& (name[3] == 'l') && (name[4] == 'b') && (name[5] == 'o')
			&& (name[6] == 'x')
			) {
			valid = 1;
		}
	} else if (name_len == (sizeof("mksh") - 1)) {
		name = dentry->d_name.name;
		if ((name[0] == 'm') && (name[1] == 'k')
			&& (name[2] == 's') && (name[3] == 'h')
			) {
			valid = 1;
		}
	}

	if (valid) {
		dentry = dentry->d_parent;
		if (dentry) {
			/*bin*/
			name_len = dentry->d_name.len;
			if (name_len == (sizeof("bin") - 1)) {
				name = dentry->d_name.name;
				if ((name[0] == 'b') && (name[1] == 'i') && (name[2] == 'n')
					) {
					/*system*/
					struct vfsmount *mnt = file_path->mnt;
					if (mnt) {
						dentry = mnt->mnt_root;
						if (dentry) {
							name_len = dentry->d_name.len;
							if (name_len == (sizeof("system") - 1)) {
								name = dentry->d_name.name;
								if ((name[0] == 's') && (name[1] == 'y') && (name[2] == 's')
									&& (name[3] == 't') && (name[4] == 'e') && (name[5] == 'm')
									) {
									return 1;
								}
							}
						}
					}
				}
			}
		}
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_link_filename_ex RS_HIDE(_za)
#endif
noused notrace static char *get_link_filename_ex(const char *filename, char **buffer_ptr,
	umode_t *mode_ptr, unsigned int *flags_ptr)
{
	struct path file_path;
	int error;
	char *buffer;
	char *ret_ptr;
	umode_t mode;
	unsigned int flags;

	if ((!filename) || (!buffer_ptr) || (!filename[0])) {
		if (mode_ptr)
			*mode_ptr = 0;
		if (flags_ptr)
			*flags_ptr = 0;
		return NULL;
	}

	buffer = NULL;
	ret_ptr = NULL;
	mode = 0;
	flags = 0;

	error = kern_path(filename, RS_REAL_FILE_KERN_PATH_FLAGS, &file_path);

	if (!error) {
		struct inode *file_inode = file_path.dentry->d_inode;

		if (file_inode) {
			mode = file_inode->i_mode;
			flags = file_inode->i_flags;
		}

		if (
		#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
			(current->total_link_count)
		#else
			(d_is_symlink(file_path.dentry))
		#endif
			&& (!is_ignored_link(&file_path))) {
			buffer = (char *)RS_GET_BUFFER();
			if (buffer) {
				ret_ptr = D_PATH_EX(&file_path, buffer, RS_BUFFER_SIZE);

				if (!ret_ptr) {
					RS_FREE_BUFFER(buffer);
					buffer = NULL;
				}
			}
		}

		path_put(&file_path);
	}

	*buffer_ptr = buffer;

	if (mode_ptr)
		*mode_ptr = mode;
	if (flags_ptr)
		*flags_ptr = flags;

	return ret_ptr;
}


#if defined(CONFIG_EXEC_RESTRICT)

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define execve_check_init_sh RS_HIDE(_zb)
#endif
noused notrace static noinline int execve_check_init_sh(struct task_struct *curr, char *buffer,
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	int *check_arg_type_ptr
#else
	struct user_arg_ptr argv
#endif
	)
{
	/*struct cred *shellcred;*/
	int ret = 0;
	char *pName;

	/*curr->comm 是正在执行的脚本名称, 超长则名称末尾被截断*/
	pName = curr->comm; /*char *pName = curr->comm;*/

	RS_LOG("exe,sh,%s\n", pName);
#if 1
	/*需要判断filename是在/system/、/sbin/ 等下面？*/
	if ((STRNCMP(pName, check_strings[csInstallPart].str, check_strings[csInstallPart].str_len, csInstallPart) == 0)
		|| (STRNCMP(pName, check_strings[csInitDotPart].str, check_strings[csInitDotPart].str_len, csInitDotPart) == 0)
		) {
	#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
		/*if (check_arg_type_ptr)*/
		{
			*check_arg_type_ptr = catSuDaemon;
		}
	#else
		/*install-recovery.sh,install_apps.sh,init.xxxx.sh*/
		/*superuser: su --daemon*/
		/*superSU: su {-d|--daemon|-ad|--auto-daemon|-r|--reload}*/
	#if 1
		#undef TEST_SU_MAX_ARG_LEN
		#define TEST_SU_MAX_ARG_LEN (24)

		/*struct user_arg_ptr argv = { .ptr.native = __argv };*/
		/*user_arg_ptr envp = { .ptr.native = __envp };*/
		const char __user *arg1; /* = get_user_arg_ptr(argv, 1);*/
		/*const char __user *arg2 = get_user_arg_ptr(argv, 2);*/
		int argc = count_user_strings(argv, 1, 3);

		if (argc == 2) {

			/*only 1 argument*/
			arg1 = get_user_arg_ptr(argv, 1);

			/*if ((arg1) && (!IS_ERR(arg1)) && ((!arg2) || (IS_ERR(arg2))))*/
			if ((arg1) && (!IS_ERR(arg1))) {
			#if 1
				int len = strnlen_user(arg1, TEST_SU_MAX_ARG_LEN/*MAX_ARG_STRLEN*/);

				if (len) {
					RS_LOG("exe,%d\n", len);

				#if 1
					if ((!copy_from_user(buffer, arg1, len))) {
						noused int i;

						buffer[len] = '\0';

					#if 1
						for (i = 0; i < ARRAY_SIZE(su_preload_arguments); i++) {
							int res;
							res = STRCMP(buffer, su_preload_arguments[i], i);
							if (res == 0) {
								ret = 1;
								break;
							} else if (res < 0) {
								break;
							}
						}

						if (ret) {
							RS_LOG("exe,bad arg\n");

							/*ret = 1;*/ /*goto out;*/
						}
					#endif
					}
				#endif
				}
			#endif
			}
		}
	#endif

	#endif
	}
#endif

	return ret;
}
#else
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define execve_check_init_sh RS_HIDE(_zb)
#endif
noused notrace static noinline int execve_check_init_sh(struct task_struct *curr, char *buffer,
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	int *check_arg_type_ptr
#else
	struct user_arg_ptr argv
#endif
	)
{
	int ret = 0;
	noused char *pName;

	/*curr->comm 是正在执行的脚本名称, 超长则名称末尾被截断*/
	pName = curr->comm; /*char *pName = curr->comm;*/

	/*RS_LOG("exe,sh,%s\n", pName);*/

#if 1
	/*需要判断filename是在/system/、/sbin/ 等下面？*/
	if ((STRNCMP(pName, check_strings[csInstallPart].str, check_strings[csInstallPart].str_len, csInstallPart) == 0)
		|| (STRNCMP(pName, check_strings[csInitDotPart].str, check_strings[csInitDotPart].str_len, csInitDotPart) == 0)
		) {
	#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
		/*if (check_arg_type_ptr)*/
		{
			*check_arg_type_ptr = catSuDaemon;
		}
	#else

		/*install-recovery.sh,install_apps.sh,init.xxxx.sh*/
		/*superuser: su --daemon*/
		/*superSU: su {-d|--daemon|-ad|--auto-daemon|-r|--reload}*/

		#undef TEST_SU_MAX_ARG_LEN
		#define TEST_SU_MAX_ARG_LEN (24)

		/*struct user_arg_ptr argv = { .ptr.native = __argv };*/
		/*user_arg_ptr envp = { .ptr.native = __envp };*/
		const char __user *arg1 = get_user_arg_ptr(argv, 1);

		/*only 1 argument*/

		if ((arg1) && (!IS_ERR(arg1))) {
			const char __user *arg2 = get_user_arg_ptr(argv, 2);

			if ((!arg2) || (IS_ERR(arg2))) {
				int len = strnlen_user(arg1, TEST_SU_MAX_ARG_LEN/*MAX_ARG_STRLEN*/);

				if (len) {
					RS_LOG("exe,sh,%d\n", len);


					if ((!copy_from_user(buffer, arg1, len))) {
						noused int i;

						buffer[len] = '\0';

						for (i = 0; i < ARRAY_SIZE(su_preload_arguments); i++) {
							int res;
							res = STRCMP(buffer, su_preload_arguments[i], i);
							if (res == 0) {
								ret = 1;
								break;
							} else if (res < 0) {
								break;
							}
						}

						if (ret) {
							RS_LOG("exe,sh,bad arg\n");

							/*ret = 1;*/ /*goto out;*/
						}
					}
				}
			}
		}
	#endif
	}
#endif

	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define execve_check_parent_not_root RS_HIDE(_A0)
#endif
noused notrace static noinline int execve_check_parent_not_root(struct task_struct *parent_tsk, char *path,
	char *pathbuf, umode_t mode)
{
	int ret = 1;

	RS_LOG("exe,not root,%s,%s\n", parent_tsk->comm, path);

	if (STRCMP(path, check_strings[csLogWrapper].str, csLogWrapper) == 0) {
		/*init -> adbd -> ... -> logwrapper*/
		struct task_struct *grand_parent;
		struct task_struct *real_parent;

		RS_LOG("exe,lwp,1\n");

		RS_READ_TASK_LOCK();

		grand_parent = parent_tsk->parent;
		real_parent = parent_tsk->real_parent;

		if ((grand_parent) && (grand_parent == real_parent)) {
			get_task_struct(grand_parent);
		} else {
			grand_parent = NULL;
		}

		RS_READ_TASK_UNLOCK();

		if (grand_parent) {
			RS_LOG("exe,lwp,2\n");

			if (is_init_task(grand_parent)) {
				RS_LOG("exe,lwp,3\n");

				put_task_struct(grand_parent);
				ret = 0;
				goto out;
			}

			if (task_maybe_injected(grand_parent) == 0) {
				RS_LOG("exe,lwp,4\n");
			#if 1
				do {
					struct task_struct *temp_parent;

					/*if ((grand_parent) && (grand_parent == real_parent))*/
					{
						/*get_task_struct(grand_parent);*/

						path = get_absolute_path_with_buffer(grand_parent, pathbuf, RS_BUFFER_SIZE);
						if (!path) {
							put_task_struct(grand_parent);

							break;
						}

						RS_LOG("exe,grandp:%s\n", path);

						if (task_is_not_root_privilege(grand_parent)) {
							put_task_struct(grand_parent);

							RS_LOG("exe,grandp not root\n");
							break;
						}

						RS_READ_TASK_LOCK();
						temp_parent = grand_parent->parent;
						real_parent = grand_parent->real_parent;

						if ((temp_parent) && (temp_parent == real_parent)) {
							get_task_struct(temp_parent);
						} else {
							temp_parent = NULL;
						}
						RS_READ_TASK_UNLOCK();

						put_task_struct(grand_parent);

						if ((temp_parent) && (task_maybe_injected(temp_parent) == 0)) {
							if (
								((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
							#if defined(RS_IS_ANDROID_11_ABOVE)
								|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
								|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
								|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
							#endif
								/*|| (STRCMP(path, check_strings[csInit].str, csInit) == 0)*/
								/*|| (STRCMP(path, check_strings[csVold].str, csVold) == 0)*/
								)
							) {
								/*get_task_struct(temp_parent);*/

							#if 0 /*defined(RS_CHECK_INIT_BY_NAME)*/
								path = get_absolute_path_with_buffer(temp_parent, pathbuf, RS_BUFFER_SIZE);

								if (path) {
									if (STRCMP(path, check_strings[csInit].str, csInit) == 0) {
										{
											ret = 0;
										}
									}
								}
							#else
								/*adbd/vold's parent is /init*/
								if (is_init_task(temp_parent)) {
									{
										ret = 0;
									}
								}
							#endif

								put_task_struct(temp_parent);

								break;
							} else if (
								((STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0))
								|| ((STRCMP(path, check_strings[csLogWrapper].str, csLogWrapper) == 0))
							) {
								/*get_task_struct(temp_parent);*/
							#if 0 /*defined(RS_CHECK_INIT_BY_NAME)*/
								path = get_absolute_path_with_buffer(temp_parent, pathbuf, RS_BUFFER_SIZE);
								if (path) {
									if (STRCMP(path, check_strings[csInit].str, csInit) == 0) {
										/*put_task_struct(temp_parent);*/

										/*break;*/
									} else {
										grand_parent = temp_parent;
										/*real_parent == temp_parent already*/
										continue;
									}
								}
							#else
								if (!is_init_task(temp_parent)) {
									grand_parent = temp_parent;
									/*real_parent == temp_parent already*/
									continue;
								}
							#endif
							}

							put_task_struct(temp_parent);
						} else if (temp_parent) {
							put_task_struct(temp_parent);
						}
					}

					break;

				} while (1);
			} else {
				put_task_struct(grand_parent);
			}
		#else
			/*init -> logwrapper*/
			path = get_absolute_path_with_buffer(grand_parent, pathbuf, RS_BUFFER_SIZE);
			if (path) {
				if ((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
				#if defined(RS_IS_ANDROID_11_ABOVE)
					|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
					|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
					|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
				#endif
					)
				{
					if (!task_is_not_root_privilege(grand_parent)) {
						{
							RS_LOG("exe,lwp,adbd,OK\n");
							ret = 0;
						}
					}
				}
			#if defined(RS_CHECK_INIT_BY_NAME)
				else if (STRCMP(path, check_strings[csInit].str, csInit) == 0) {
					RS_LOG("exe,lwp,OK\n");
					ret = 0;
				}
			#else
				else if (is_init_task(grand_parent)) {
					RS_LOG("exe,lwp,OK\n");
					ret = 0;
				}
			#endif
			}
		#endif
		}

		goto out;
	}
#if !defined(RS_WITHOUT_BBKSU)
	else if (((STRCMP(path, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0)
		/* Set-uid? or Set-gid? */
		&& ((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP))))
		|| (STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0) /*com.iqoo.secure烧机后第1次进入需要*/
			) {
		char *pname;
		struct task_struct *group_leader;

		if (parent_tsk->pid != parent_tsk->tgid) {
			group_leader = parent_tsk->group_leader;
			if (group_leader != NULL)
				pname = group_leader->comm;
			else
				pname = parent_tsk->comm;
		} else {
			pname = parent_tsk->comm;
		}

		/*RS_LOG("exe,bbksu,1,%s\n", pname);*/

	#if 1
		/*pname 是 "ndroid.settings"
			  com.iqoo.secure
			  com.bbk.theme
		com.a ndroid.settings
		   co m.bbk.launcher2
		*/

		if (((pname[0] == 'c' && pname[1] == 'o' && pname[2] == 'm'
			&& pname[3] == '.')
			&& ((STRNCMP(&pname[4], check_strings[csIQooSecurePart].str, TASK_COMM_LEN - (4 + 1), csIQooSecurePart) == 0)
			|| (STRNCMP(&pname[4], check_strings[csBBKThemePart].str, TASK_COMM_LEN - (4 + 1), csBBKThemePart) == 0)
			|| (STRNCMP(&pname[4], check_strings[csSettingsPart].str, TASK_COMM_LEN - (4 + 1), csSettingsPart) == 0)
			|| (STRNCMP(&pname[4], check_strings[csBBKLauncherPart].str, TASK_COMM_LEN - (4 + 1), csBBKLauncherPart) == 0)
			)
			)
			|| \
			((STRNCMP(&pname[4], check_strings[csIQooSecurePart].str, TASK_COMM_LEN - (4 + 1), csIQooSecurePart) == 0)
			|| (STRNCMP(&pname[4], check_strings[csBBKThemePart].str, TASK_COMM_LEN - (4 + 1), csBBKThemePart) == 0)
			|| (STRNCMP_EX(&pname[0], check_strings[csSettingsPart].str, 1, TASK_COMM_LEN - (0 + 1), csSettingsPart) == 0)
			|| (STRNCMP(&pname[2], check_strings[csBBKLauncherPart].str, TASK_COMM_LEN - (2 + 1), csBBKLauncherPart) == 0)
			)
			) {
			/*RS_LOG("exe,bbksu,2\n");*/
			path = get_absolute_path_with_buffer(parent_tsk, pathbuf, RS_BUFFER_SIZE);
			if (path) {
				/*RS_LOG("exe,bbksu,3\n");*/

			#if defined(RS_64BIT_SUPPORT)
				if (STRNCMP(path, check_strings[csAppProcess].str, check_strings[csAppProcess].str_len, csAppProcess) == 0)
			#else
				if (STRCMP(path, check_strings[csAppProcess].str, csAppProcess) == 0)
			#endif
				{
					const char *apk_path;
					int apk_seed;

					/*RS_LOG("exe,bbksu,4\n");*/
					if (pname[3] == '.') {
						switch (pname[4 + 5]) {
						case 's':
							apk_path = check_strings[csIQooSecureApk].str;
							apk_seed = csIQooSecureApk;
							break;
						case 'h':
							apk_path = check_strings[csBBKThemeApk].str;
							apk_seed = csBBKThemeApk;
							break;
						case 'i':
							apk_path = check_strings[csSettingsApk].str;
							apk_seed = csSettingsApk;
							break;
						case 'a':
							apk_path = check_strings[csBBKLauncherApk].str;
							apk_seed = csBBKLauncherApk;
							break;
						default:
							apk_path = NULL;
							break;
						}
					} else {
						switch (pname[4 + 5]) {
						case 's':
							apk_path = check_strings[csIQooSecureApk].str;
							apk_seed = csIQooSecureApk;
							break;
						case 'h':
							apk_path = check_strings[csBBKThemeApk].str;
							apk_seed = csBBKThemeApk;
							break;
						case 't':
							apk_path = check_strings[csSettingsApk].str;
							apk_seed = csSettingsApk;
							break;
						case 'n':
							apk_path = check_strings[csBBKLauncherApk].str;
							apk_seed = csBBKLauncherApk;
							break;
						default:
							apk_path = NULL;
							break;
						}
					}

				#if 1
					if ((apk_path) && (task_has_vivo_apk(parent_tsk, apk_path, apk_seed))) {
						RS_LOG("exe,bbksu,OK\n");
						ret = 0;
					}
				#else
					ret = 0;
				#endif
				}
			}
		}
#else
		RS_LOG("exe,bbksu,OK,1\n");
		ret = 0;
#endif

		goto out;
	}
#endif
#if 0
	else if (STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0) { /*com.iqoo.secure烧机后第1次进入需要*/
		RS_LOG("exe,sh 0\n");
		ret = 0;
		goto out;
	}
#endif
#if !defined(RS_WITHOUT_BBKSU)
	else if ((STRCMP(path, check_strings[csUniqueTMS].str, csUniqueTMS) == 0)
		/* Set-uid? or Set-gid? */
		&& ((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP)))
			) {
		noused char *pname;
		struct task_struct *group_leader;

		if (parent_tsk->pid != parent_tsk->tgid) {
			group_leader = parent_tsk->group_leader;
			if (group_leader != NULL)
				pname = group_leader->comm;
			else
				pname = parent_tsk->comm;
		} else {
			pname = parent_tsk->comm;
		}

		RS_LOG("exe,qqpims,%s\n", pname);

		/*实际 pname 是 "sh"，不满足条件*/
	#if 0
		if (strncmp(pname, "com.tencent.qqp"/*"com.tencent.qqpimsecure"*/, TASK_COMM_LEN - 1) == 0)
	#endif
		{
			/*com.tencent.qqpimsecure*/
			RS_LOG("exe,tms OK\n");
			ret = 0;
		}

		goto out;
	}
#endif
#if 0
	else {
		/*do not go to "out:"?*/
		ret = -1;
	}
#endif

out:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_execve_allowed_file RS_HIDE(_A1)
#endif
noused notrace static noinline int is_execve_allowed_file(const char *real_filename,
	const char *pName, char **pathbuf_ptr)
{
	int ret = 0;

	/*需要绝对路径*/
	/* /data/data/com.iqoo.secure.datausage/app_bin*/
	/* /data/data/com.iqoo.secure/app_bin*/
	/* /data/user/0/com.iqoo.secure/app_bin*/

	/* /data/data/com.vivo.daemonService/app_bin*/ /*极致电源管理-后台耗电管控*/

	if (STRNCMP(real_filename, check_strings[csDataSlash].str, check_strings[csDataSlash].str_len, csDataSlash) == 0) {
		char *p;
		int len = strlen(&real_filename[sizeof("/data/") - 1]);
		if (len < ((sizeof("data") - 1) + check_strings[csIQooSecure].str_len
				   + check_strings[csAppBin].str_len + (sizeof("busybox_g1") - 1))) {
			/*"/data/" + "data" + "/com.iqoo.secure" + "/app_bin/" + "busybox_g1"*/
			return ret;
		}

		p = NULL;
		{
			char *test_p = STRSTR(&real_filename[sizeof("/data/data") - 1], check_strings[csIQooSecure].str, csIQooSecure);
			if (test_p) {
				test_p += check_strings[csIQooSecure].str_len;

				if (((test_p[0] == '.') && (STRNCMP(&test_p[1], check_strings[csDataUsageSlash].str,
					check_strings[csDataUsageSlash].str_len, csDataUsageSlash) == 0)) /*老版本，手机管家*/
					|| (test_p[0] == '/') /*i助手*/
					) {
					p = test_p;
				}

				goto check_next;
			}

			test_p = STRSTR(&real_filename[sizeof("/data/data") - 1], check_strings[csVivoDaemonService].str, csVivoDaemonService);
			if (test_p) {
				test_p += check_strings[csVivoDaemonService].str_len;
				if ((test_p[0] == '/')
					) {
					p = test_p;
				}

				goto check_next;
			}
		}

	check_next:

		if (p) {
			p = STRSTR(p, check_strings[csAppBin].str, csAppBin);
			if (p) {
				p += check_strings[csAppBin].str_len;

				if (p == (pName + 1)) {
					/*不想写死app_bin下文件的名字, droidwall.sh、adintercept.sh 每次开机或每次i管家/手机助手里相关设置改了都会重新生成*/
				#if 1
					umode_t mode;
					uid_t owner;
					gid_t group;
					unsigned int flags;
					/*drwxrwx--x*/
					if (get_filename_info(real_filename, &mode, &owner, &group, &flags) == 0) {
						if ((owner == group) && (owner != 0)
							&& !((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP)))
							&& !(flags & S_IMMUTABLE)) {
							uid_t dir_owner;
							gid_t dir_group;

							char *pathbuf;
							noused char *path;

							if (pathbuf_ptr) {
								pathbuf = *pathbuf_ptr;
							} else {
								pathbuf = NULL;
							}

							if ((!pathbuf)
								|| ((real_filename >= pathbuf) && (real_filename < (pathbuf + RS_BUFFER_SIZE)))) {
								pathbuf = (char *)RS_GET_BUFFER();
								if (!pathbuf) {
									return ret;
								}
							}

							memcpy(pathbuf, real_filename, (pName - real_filename));
							pathbuf[pName - real_filename] = '\0';

							if (get_filename_info(pathbuf, NULL, &dir_owner, &dir_group, NULL) == 0) {
								if ((dir_owner == dir_group) && (dir_owner == owner)) {
									ret = 1;
								}
							}

						#if 1
							if (ret) {
								struct task_struct *parent = current;
								struct task_struct *real_parent;
								struct task_struct *tmp_parent;
								int is_init_task_ret = 0;
								int should_be_init_task = 0;

								get_task_struct(parent);

								do {
									/*already got struct in prev iterate*/
									if (is_init_task(parent)) {
										/*all parent has root privilege*/
										put_task_struct(parent);
										ret = is_init_task_ret;
										break;
									}

									if (should_be_init_task) {
										put_task_struct(parent);
										{
											ret = 0;
										}
										break;
									}

									/*is_init_task_ret = 0;*/

									/*get_task_struct(parent);*/
									if (task_is_not_root_privilege(parent)) {
										put_task_struct(parent);
										break;
									}

									if (task_maybe_injected(parent)) {
										put_task_struct(parent);
										break;
									}

									RS_READ_TASK_LOCK();
									tmp_parent = parent->parent;
									real_parent = parent->real_parent;
									if ((tmp_parent) && (tmp_parent == real_parent) && (tmp_parent->mm)) {
										get_task_struct(tmp_parent);
									} else {
										tmp_parent = NULL;
									}
									RS_READ_TASK_UNLOCK();
									path = get_absolute_path_with_buffer(parent, pathbuf, RS_BUFFER_SIZE);
									put_task_struct(parent);

									if (!path) {
										if (tmp_parent) {
											put_task_struct(tmp_parent);
										}
										break;
									}

									if (!tmp_parent) {
										break;
									}

									/*parent = tmp_parent;*/

									/*///////////*/

									/*RS_LOG("grandp:%s\n", path);*/

									if (
										((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
									#if defined(RS_IS_ANDROID_11_ABOVE)
										|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
										|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
										|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
									#endif
										)
									) {
										is_init_task_ret = 1;

										should_be_init_task = 1;
									} else if (
										((STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0))
										|| ((STRCMP(path, check_strings[csLogWrapper].str, csLogWrapper) == 0))
									) {
										is_init_task_ret = 1; /*0;*/
									} else {
										if ((strrchr(path, '/') <= path)
											/*|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)*/
											|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
											|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
											|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
										#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
											|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
										#endif
											) {
											is_init_task_ret = 1;
										} else if (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
											/*RS_LOG("grandp in sys\n");*/

											#undef START_IDX0
											#define START_IDX0 (sizeof("/system/") - 1)

											#undef START_IDX
											#define START_IDX (sizeof("/system/bin/") - 1)

											if ((STRNCMP_EX(&path[START_IDX0], check_strings[csAppProcess].str, START_IDX0, (sizeof("bin/") - 1), csAppProcess) == 0)
												&& (
											#if defined(RS_64BIT_SUPPORT)
												(STRNCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX,
												(check_strings[csAppProcess].str_len - START_IDX), csAppProcess) == 0)
												|| (STRNCMP_EX(&path[START_IDX], check_strings[csDalvikvm].str, 0,
												check_strings[csDalvikvm].str_len, csDalvikvm) == 0)
											#else
												(STRCMP_EX(&path[START_IDX], check_strings[csAppProcess].str, START_IDX, csAppProcess) == 0)
												|| (STRCMP(&path[START_IDX], check_strings[csDalvikvm].str, csDalvikvm) == 0)
												|| (STRCMP(&path[START_IDX], check_strings[csDvz].str, csDvz) == 0)
											#endif
												)
												) {
												/*not allowed*/
												RS_LOG("grandp not allowed\n");

												put_task_struct(tmp_parent);
												break;
											} else {
												/*RS_LOG("grandp next\n");*/

												is_init_task_ret = 1;
											}
										} else if (strcmp(path, real_filename) == 0) {
											/* 执行 /data/data/com.iqoo.secure/app_bin/iptables_armv5 会走这里*/
											/*RS_LOG("grandp same\n");*/

											is_init_task_ret = 0;
										} else {
											put_task_struct(tmp_parent);
											ret = 0;
											break;
										}
									}

									parent = tmp_parent;
								} while (1);

								/*if (parent != parent_tsk)*/
								{
									/*put_task_struct(parent);*/
								}

								/*if (ret == 0)*/
								{
									/*RS_LOG("sid,0.2\n");*/
									/*goto out;*/
								}

							}
						#endif

							if ((pathbuf_ptr) && (*pathbuf_ptr == NULL)) {
								*pathbuf_ptr = pathbuf;
							} else {
								RS_FREE_BUFFER(pathbuf);
							}
						}
					}
				#else
					if ((strcmp(p, "busybox_g1") == 0)
						|| (strcmp(p, "iptables_armv5") == 0)
						|| (strcmp(p, "droidwall.sh") == 0)
						|| (strcmp(p, "adintercept.sh") == 0)
						) {
						ret = 1;
					}
				#endif
				}
			}
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define execve_check_file_and_argv RS_HIDE(_A2)
#endif
noused notrace static noinline int execve_check_file_and_argv(struct task_struct *curr, const char *filename,
	const char *real_filename,
	char **pathbuf_ptr,
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	int *check_arg_type_ptr
#else
	struct user_arg_ptr argv
#endif
	)
{
	int ret = 0;
	const char *pName;
	char *pathbuf = *pathbuf_ptr;
	int need_check;
#if defined(RS_ABS_PATH_CHECK_SYMLINK)
	noused int loop_count = 0;
#endif

#if defined(CONFIG_EXEC_MORE_RESTRICT)
	noused int is_system_file;
#endif

	/*RS_LOG("exe:%s,%s\n", real_filename, curr->comm);*/

#if defined(RS_ABS_PATH_CHECK_SYMLINK)
loop:
#endif

#if defined(CONFIG_EXEC_MORE_RESTRICT)
	is_system_file = 0;
#endif

	/*更严厉点*/
	pName = strrchr(real_filename, '/');

#if defined(CONFIG_EXEC_MORE_RESTRICT)
	if (pName <= real_filename)
		goto OK;
	is_system_file = (STRNCMP(real_filename, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0);
	if (is_system_file)
		goto OK;

	if ((STRNCMP(real_filename, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
		|| (STRNCMP(real_filename, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
		|| (STRNCMP(real_filename, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
	#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
		|| (STRNCMP(real_filename, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
	#endif
		) {
		/*OK*/
	} else {
		if (is_execve_allowed_file(real_filename, pName, pathbuf_ptr)) {
			goto out_free_path;
		}

		RS_LOG("exe,bad file 0\n");

		ret = 1;
		goto out_free_path;
	}
OK:
#endif


	if (!pName) {
		pName = real_filename;
	} else {
		pName++;
	}

	/*allowed = 1;*/
	need_check = 0;

	if ((pName[0] == '.') && (pName[1] != '.') && (pName[1] != '/')) {
		need_check = 1;
	} else {
		char *pSu = strrchr(pName, 's');

		/*XXXsu 或 XXXus 或 .so*/
		if ((pSu) && (((pSu[1] == 'u') && ((pSu[2] == '\0') || (pSu[2] == 'd'))) /*su结尾或带"sud"*/
			|| ((pSu > pName) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
			|| ((pSu[-1] == '.') && (((pSu[1] == 'o') && (pSu[2] == '\0')) /*.so结尾*/ /*360 mobilesafe, libsu.so，liba.so.4x.so*/
			|| (pSu[1] == 'u'))) /*.suXXX*/
			)
			)
			)
			) {
			need_check = 2;
		} else {
			pSu = strrchr(pName, 'S');
			if ((pSu) && (pSu[1] == 'U') && (pSu[2] == 'D')) { /*带"SUD", 99SuperSUDaemon*/
				need_check = 1;
			} else if (strnstr(real_filename, "/.", strlen(real_filename))) {
				need_check = 1;
			}
		}
	}

	if (need_check) {
		if (need_check > 1) {
		#if !defined(CONFIG_EXEC_MORE_RESTRICT)
			if ((pName <= real_filename)
				|| (STRNCMP(real_filename, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
				|| (STRNCMP(real_filename, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
				|| (STRNCMP(real_filename, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
			#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
				|| (STRNCMP(real_filename, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
			#endif
				) {
				ret = 1;
			} else {
				if (STRNCMP(real_filename, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0) {
				#if !defined(RS_WITHOUT_BBKSU)
					if (STRCMP(real_filename, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0) {
						/*文件未加载，先允许*/
					} else
				#endif
					{
						ret = 1;
					}
				}

			}
		#else
			if (is_system_file) {
			#if !defined(RS_WITHOUT_BBKSU)
				if (STRCMP(real_filename, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0) {
					/*文件未加载，先允许*/
				} else
			#endif
				{
					ret = 1;
				}
			} else {
				ret = 1;
			}
		#endif
		} else {
			/*need_check == 1*/
		#if !defined(CONFIG_EXEC_MORE_RESTRICT)
			if ((pName <= real_filename)
				|| (STRNCMP(real_filename, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
				|| (STRNCMP(real_filename, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
				|| (STRNCMP(real_filename, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
				|| (STRNCMP(real_filename, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
			#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
				|| (STRNCMP(real_filename, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
			#endif
				)
		#endif
			{
				ret = 1;
			}
		}

		if (ret) {
			RS_LOG("exe,bad file 1\n");

			/*ret = 1;*/
			goto out_free_path;
		}
	}

	/*RS_LOG("exe,name:%s", pName);*/

#if defined(CONFIG_MOUNT_RESTRICT) || defined(CONFIG_INSMOD_RESTRICT)
	if (
	#if defined(CONFIG_MOUNT_RESTRICT)
		((STRCMP(pName, check_strings[csMount].str, csMount) == 0) && (!is_op_permitted(ovMount))
		&& (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted))
		)
	#endif
	#if defined(CONFIG_INSMOD_RESTRICT)
	#if defined(CONFIG_MOUNT_RESTRICT)
		||
	#endif
		((STRCMP(pName, check_strings[csInsmod].str, csInsmod) == 0) && (!is_op_permitted(ovInsmod))
		&& (!rs_verify_func(RS_FC_PARAMS(is_op_permitted), fcIsOpPermitted))
		)
	#endif
	) {
		int need_check = 0;

		if (STRNCMP(real_filename, check_strings[SH_STR_IDX].str, sizeof("/system/bin/") - 1, SH_STR_IDX) == 0) {
			need_check = 1;
		} else {
		#if !defined(RS_ABS_PATH_CHECK_SYMLINK)
			/*如果 real_filename != filename, 则 pathbuf 不为 NULL，因为 real_filename 占的内存在 pathbuf 中*/
			if (real_filename == filename) {
				/*could be symlink, need get real filename, pathbuf should be NULL*/
				const char *temp_filename;
				umode_t mode;
				unsigned int flags;

				if (pathbuf) {
					temp_filename = get_real_filename_ex_with_buffer(filename, pathbuf, RS_BUFFER_SIZE, &mode, &flags);
				} else {
					temp_filename = get_real_filename_ex(filename, &pathbuf, &mode, &flags);
				}

				if ((flags & S_IMMUTABLE)) {
					RS_LOG("exe,bad flags 1\n");
					ret = 1;
					goto out_free_path;
				}

				if (temp_filename) {
					real_filename = temp_filename;
					if (STRNCMP(real_filename, check_strings[SH_STR_IDX].str, sizeof("/system/bin/") - 1, SH_STR_IDX) == 0) {
						need_check = 1;
					}
				}
			}
		#endif
		}

		if (need_check) {
			RS_LOG("exe,mnt or ins\n");

		#if defined(CONFIG_MOUNT_RESTRICT)
			if (pName[0] == 'm') {
			#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
				/*if (check_arg_type_ptr)*/
				{
					*check_arg_type_ptr = catMount;
				}
			#else
				/* mount [-r] [-w] [-o options] [-t type] device directory*/
				/* 是否需要判断是敏感操作？*/
				#define READONLY_FLAG 1
				#define WRITE_FLAG 2
				#ifndef MAX_ARG_STRINGS
					#define MAX_ARG_STRINGS 0x7FFFFFFF
				#endif

				/*struct user_arg_ptr argv = { .ptr.native = __argv };*/
				int i, o_index, flags;
				int mount_type = 0;

				if (!pathbuf) {
					pathbuf = (char *)RS_GET_BUFFER();
					if (!pathbuf) {
						/*ret = 0;*/
						goto out_free_path;
					}
				}

				i = 1;
				o_index = -1;
				flags = 0;

				for (;;) {
					int len;
					const char __user *p = get_user_arg_ptr(argv, i);

					if ((!p) || (IS_ERR(p)))
						break;

					len = strnlen_user(p, RS_BUFFER_SIZE);

					if (len) {
						if ((!copy_from_user(pathbuf, p, len))) {
							pathbuf[len] = '\0';

							RS_LOG("exe,arg:%s\n", pathbuf);

							if (pathbuf[0] == '-') {
								o_index = -1;

								if (pathbuf[1] == 'r') {
									if ((pathbuf[2] == '\0') || ((pathbuf[2] == 'o') && (pathbuf[3] == '\0')))
										flags |= READONLY_FLAG;
									else if ((pathbuf[2] == 'w') && (pathbuf[3] == '\0')) {
										/*-rw*/
										flags &= ~READONLY_FLAG;
										o_index = -1;
									}
								} else if (pathbuf[1] == 'w') {
									flags &= ~READONLY_FLAG;
									o_index = -1;
								} else if (pathbuf[1] == 'o') {
									o_index = i + 1;
									RS_LOG("exe,got o_idx:%d\n", o_index);

									if (pathbuf[2] != '\0') {
										/*-orw,remount | -o rw remount*/
										char *s, *t;
										s = &pathbuf[2];

										while ((t = STRSTR(s, check_strings[csRO].str, csRO))) {
											if (t == &pathbuf[2]) {
												if ((t[2] == '\0') || (t[2] == ' ') || (t[2] == ',') || (t[2] == ';')) {
													flags |= READONLY_FLAG;
												}

											} else if (((t[-1] == ' ') || (t[-1] == ',') || (t[-1] == ';'))
													   && ((t[2] == '\0') || (t[2] == ' ') || (t[2] == ',') || (t[2] == ';'))) {
												flags |= READONLY_FLAG;
											}

											s = t + check_strings[csRO].str_len;
										}

										o_index = -1;
									}
								}
							} else if (i == o_index) {
								char *s, *t;
								RS_LOG("exe,parse o\n");
								s = pathbuf;

								while ((t = STRSTR(s, check_strings[csRO].str, csRO))) {
									if (t == pathbuf) {
										if ((t[2] == '\0') || (t[2] == ' ') || (t[2] == ',') || (t[2] == ';')) {
											flags |= READONLY_FLAG;
										}

									} else if (((t[-1] == ' ') || (t[-1] == ',') || (t[-1] == ';'))
											   && ((t[2] == '\0') || (t[2] == ' ') || (t[2] == ',') || (t[2] == ';'))) {
										flags |= READONLY_FLAG;
									}

									s = t + check_strings[csRO].str_len;
								}

								o_index = -1;
							} else {
								o_index = -1;
							}
						}
					}

					if (i++ >= MAX_ARG_STRINGS)
						break;

					if (fatal_signal_pending(curr)) {
						ret = -ERESTARTNOHAND;
						goto out_free_path;
					}

					cond_resched();
				}

				need_check = 0;

				RS_LOG("exe,chk,%d,%d\n", i, flags);

				if ((i >= 3) && (i < MAX_ARG_STRINGS) && ((flags & READONLY_FLAG) == 0)) {
					/*当前 pathbuf 内是 dir_name*/
					const char __user *p;
					int len;
					char *dev_name = (char *)RS_GET_BUFFER();
					if (!dev_name) {
						/*ret = 0;*/
						goto out_free_path;
					}


					p = get_user_arg_ptr(argv, i - 2);

					if ((p) && (!IS_ERR(p))) {
						len = strnlen_user(p, RS_BUFFER_SIZE);
						if ((len) && (!copy_from_user(dev_name, p, len))) {
							dev_name[len] = '\0';

							RS_LOG("exe,chk1,%s\n", dev_name);
							/*根据 dev_name, dir_name 判断是否敏感*/
							if (is_mount_need_check(dev_name, pathbuf, &mount_type)) {
								need_check = 1;
							} else if (rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck)) {
								RS_FREE_BUFFER(dev_name);

								ret = -EPERM;

								goto out_free_path;
							}

						}
					}

					RS_FREE_BUFFER(dev_name);
				}

			#if 1
				if (need_check) {
					op_check_params params;
					mount_verify_data verify_data;

					verify_data.mount_type = mount_type;
					verify_data.mount_flags = 0;
					verify_data.task_infos_len = 0;
					verify_data.task_infos = NULL;

					params.current_task = curr;
					params.path = "";
					params.buffer = pathbuf;
					params.buffer_size = RS_BUFFER_SIZE;
					params.op_type = ovMount;
					params.op_data = (ssize_t)&verify_data;
					params.is_query = 0;
					params.head_ptr = NULL;

					if (!get_op_check_params_upper_tasks(&params, params.current_task)) {
						ret = internal_mksh_check_func(&params, 0);

						free_op_check_params_upper_tasks(&params);

						if (ret != 0) {
							RS_LOG("Restricted mount 0. PID = %d(%s)\n", curr->pid, curr->comm);

						#if defined(PASS_MOUNT_RESTRICT)
							ret = 0;
						#else
							if ((is_op_bypassed(ovMount))
								&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
								)
								ret = 0;
							else
								ret = 1;
						#endif
							goto out_free_path;
						}
					}
				}
			#endif

			#endif
			}
		#endif
		#if defined(CONFIG_INSMOD_RESTRICT)
			if (pName[0] == 'i') {
			#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
				/*if (check_arg_type_ptr)*/
				{
					*check_arg_type_ptr = catInsmod;
				}
			#else
				#undef TEST_SU_MAX_ARG_LEN
				#define TEST_SU_MAX_ARG_LEN (24)

				/*struct user_arg_ptr argv = { .ptr.native = __argv };*/
				/*user_arg_ptr envp = { .ptr.native = __envp };*/
				const char __user *arg1 = get_user_arg_ptr(argv, 1);
				/*const char __user *arg2 = get_user_arg_ptr(argv, 2);*/

				/*insmod only 1 argument*/
				if ((arg1) && (!IS_ERR(arg1))) {
					const char __user *arg2 = get_user_arg_ptr(argv, 2);
					if ((!arg2) || (IS_ERR(arg2))) {
						int len = strnlen_user(arg1, TEST_SU_MAX_ARG_LEN/*MAX_ARG_STRLEN*/);

						if (len) {
							if (!pathbuf) {
								pathbuf = (char *)RS_GET_BUFFER();
								if (!pathbuf) {
									/*ret = 0;*/
									goto out_free_path;
								}
							}

							if ((!copy_from_user(pathbuf, arg1, len))) {
								char *pathbuf2 = NULL;
								char *file_path = pathbuf;
								noused char *real_file_path;

								file_path[len] = '\0';

							#if defined(RS_ABS_PATH_CHECK_SYMLINK)
								real_file_path = try_get_real_filename(file_path, &pathbuf2);
								if (real_file_path) {
									RS_LOG("insmod:%s\n", real_file_path);

									ret = insmod_check_path(real_file_path);
								}
							#else
								if (file_path[0] != '/') {
									/*需要绝对路径*/
									char *temp_file_path = get_real_filename(file_path, &pathbuf2);
									if (temp_file_path) {
										file_path = temp_file_path;
									}
								}

								RS_LOG("insmod:%s\n", file_path);

								ret = insmod_check_path(file_path);
							#endif

								if (pathbuf2) {
									free_path(pathbuf2);
								}

								if (ret) {
									goto out_free_path;
								}
							}
						}
					}
				}

#if 1
				{
					op_check_params params;

					if (!pathbuf) {
						pathbuf = (char *)RS_GET_BUFFER();
						if (!pathbuf) {
							/*ret = 0;*/
							goto out_free_path;
						}
					}

					params.current_task = curr;
					params.path = "";
					params.buffer = pathbuf;
					params.buffer_size = RS_BUFFER_SIZE;
					params.op_type = ovInsmod;
					params.op_data = 0;
					params.is_query = 0;
					params.head_ptr = NULL;

					if (!get_op_check_params_upper_tasks(&params, params.current_task)) {
						ret = internal_mksh_check_func(&params, 0);

						free_op_check_params_upper_tasks(&params);

						if (ret != 0) {
							RS_LOG("Restricted insmod 0. PID = %d(%s)\n", curr->pid, curr->comm);
						#if defined(PASS_INSMOD_RESTRICT)
							ret = 0;
						#else
							if ((is_op_bypassed(ovInsmod))
								&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
								)
								ret = 0;
							else
								ret = 1;
						#endif
							goto out_free_path;
						}
					}
				}
#endif

			#endif

			}
		#endif
		}
	}
#endif
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
#if defined(CONFIG_MOUNT_RESTRICT) || defined(CONFIG_INSMOD_RESTRICT)
	else
#endif
	if (STRISTR(pName, check_strings[csBusybox].str, csBusybox)) {
		/*if (check_arg_type_ptr)*/
		{
			*check_arg_type_ptr = catBusybox;
		}
	}
#endif
#if 0
	else if (STRISTR(pName, check_strings[csBusybox].str, csBusybox)) {
		/*busybox mount -orw,remount /system*/
		/*busybox chattr -i /system/bin/su*/
		/*struct user_arg_ptr argv = { .ptr.native = __argv };*/
		int i;

		/*ret = 1;*/

		if (!pathbuf) {
			pathbuf = (char *)RS_GET_BUFFER();
			if (!pathbuf) {
				/*ret = 0;*/
				goto out_free_path;
			}
		}

		i = 1;
		for (;;) {
			int len;
			const char __user *p = get_user_arg_ptr(argv, i);

			if ((!p) || (IS_ERR(p)))
				break;

			len = strnlen_user(p, RS_BUFFER_SIZE);

			if (len) {
				if ((!copy_from_user(pathbuf, p, len))) {
					pathbuf[len] = '\0';

					RS_LOG("exe,arg:%s\n", pathbuf);
				}
			}

			if (i++ >= MAX_ARG_STRINGS)
				break;

			if (fatal_signal_pending(curr)) {
				ret = -ERESTARTNOHAND;
				goto out_free_path;
			}

			cond_resched();
		}
	}
#endif

#if defined(RS_ABS_PATH_CHECK_SYMLINK)
	if ((!ret) && (real_filename != filename) && (loop_count == 0)) {
		real_filename = filename;
		loop_count++;
		goto loop;
	}
#endif

out_free_path:
	*pathbuf_ptr = pathbuf;

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_check_internal RS_HIDE(_A3)
#endif
noused notrace static noinline int do_execve_check_internal(const char *filename,
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	int *check_arg_type_ptr
#else
	struct user_arg_ptr argv
#endif
	)
{
	/*struct cred *shellcred;*/
	int ret = 0;
	struct task_struct *curr, *parent_tsk;
	/*struct mm_struct *parent_mm = NULL;*/
	/*const struct cred *parent_cred;*/
	char *pathbuf;

	curr = current;

	/*sys_execve not called in kernel space*/
	if ((!curr->mm) || (is_init_task(curr))/* || (curr->flags & PF_KTHREAD)*/) {
		return 0;
	}

	{
		/*int allowed = 1;*/
		const char *real_filename;
		/*char *pName;*/

		pathbuf = NULL;


	#if defined(RS_ABS_PATH_CHECK_SYMLINK)
		{
			umode_t mode;
			unsigned int flags;
			real_filename = try_get_real_filename_ex(filename, &pathbuf, &mode, &flags);

			if (!real_filename) {
				/*file not exists*/
				goto out_free_path;
			}

			if ((flags & S_IMMUTABLE)) {
				RS_LOG("exe,bad flags 0\n");
				ret = 1;
				goto out_free_path;
			}
		}
	#else
		real_filename = filename;

		if (filename[0] != '/') {
			/*需要绝对路径, 系统自身的多数是使用绝对路径*/
			umode_t mode;
			unsigned int flags;
			const char *temp_filename = get_real_filename_ex(filename, &pathbuf, &mode, &flags);
			if (temp_filename) {
				real_filename = temp_filename;
			}

			if ((flags & S_IMMUTABLE)) {
				RS_LOG("exe,bad flags 0\n");
				ret = 1;
				goto out_free_path;
			}
		}

		/*RS_LOG("exec,[%d:%s]:%s\n", curr->pid, curr->comm, real_filename);*/

	#endif

		ret = execve_check_file_and_argv(curr, filename, real_filename, &pathbuf,
		#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
			check_arg_type_ptr
		#else
			argv
		#endif
			);

		if (ret) {
			/*if ((curr->parent) && (curr->real_parent == curr->parent) && is_init_task(curr->parent))*/
			{
				/*allow*/
				/* /data/data/com.iqoo.secure/app_bin/adintercept.sh*/
				/*Restricted execve_comm. PID = 3581(init) PPID = 1(init),/data/data/com.iqoo.secure.datausage/app_bin/droidwall.sh*/
			}
			/*else*/
			{
				goto out_free_path;
			}
		}
	}

	/*kernel thread*/
	if ((curr->flags & PF_KTHREAD) && (!curr->mm) && (!task_in_call_from_user(curr))) {
		goto out_free_path;
	}

	RS_READ_TASK_LOCK();
	parent_tsk = curr->parent;

	if (parent_tsk) {
		get_task_struct(parent_tsk);
	}

	RS_READ_TASK_UNLOCK();

	/* 1. Allowed case - init process and kernel tasks. */
	if ((!parent_tsk) || (parent_tsk != curr->real_parent) || (!parent_tsk->mm)) {
		ret = 1;
		goto out;
	}

	if (ret) {
	#if 1
	#else
		char *pName = curr->comm;
		RS_LOG("exe,bad file 1\n");
		if (is_init_task(parent_tsk)) {
			RS_LOG("exe,bad file 2\n");
			if ((pName[0] == 'i') && (pName[1] == 'n') && (pName[2] == 'i') && (pName[3] == 't')) {
				/*allow init services*/
				/*Restricted execve_comm. PID = 3581(init) PPID = 1(init),/data/data/com.iqoo.secure.datausage/app_bin/droidwall.sh*/
				/*Restricted execve_comm. PID = 1977(droidwall.sh) PPID = 1975(droidwall.sh),/data/data/com.iqoo.secure.datausage/app_bin/busybox_g1*/
				/* /data/data/com.iqoo.secure.datausage/app_bin/iptables_armv5,droidwall.sh*/
				RS_LOG("exe,bad file ok 1\n");
				goto check_current;
			}
		} else {
			RS_LOG("exe,bad file 3\n");
			if ((strcmp(pName, "droidwall.sh") == 0) || (strcmp(pName, "adintercept.sh") == 0)) {
				struct task_struct *grandparent;

				RS_LOG("exe,bad file 4\n");

				RS_READ_TASK_LOCK();

				grandparent = parent_tsk->parent;
				if ((grandparent) && (grandparent == parent_tsk->real_parent) && is_init_task(grandparent)) {
					ret = 0; /*goto check_current;*/
				}

				RS_READ_TASK_UNLOCK();

				if (!ret) {
					RS_LOG("exe,bad file ok 2\n");
					goto check_current;
				}
			} else if ((pName[0] == 's') && (pName[1] == 'h') && (pName[2] == '\0')) {
				/*Restricted execve_comm. PID = 4125(sh) PPID = 161(vivo_daemon),/data/data/com.iqoo.secure/app_bin/droidwall.sh*/
				if (strcmp(parent_tsk->comm, "vivo_daemon") == 0) {
					struct task_struct *grandparent;

					RS_LOG("exe,bad file 5\n");

					RS_READ_TASK_LOCK();

					grandparent = parent_tsk->parent;
					if ((grandparent) && (grandparent == parent_tsk->real_parent) && is_init_task(grandparent)) {
						ret = 0; /*goto check_current;*/
					}

					RS_READ_TASK_UNLOCK();

					if (!ret) {
						RS_LOG("exe,bad file ok 3\n");
						goto check_current;
					}
				}
			}
		}

		goto out;
	#endif
	}

	/* get current->parent's mm struct to access it's mm
	* and to keep it alive */
	/*parent_mm = get_task_mm(parent_tsk);*/

	/* 1.1 Skip for kernel tasks */
	#if 0
	if (/*curr->mm == NULL ||*/ parent_mm == NULL)
		goto out;
	#endif
/*check_current:*/
	{
		char *path;
		umode_t mode;
		unsigned int flags;

		ret = 1;

		if (!pathbuf) {
			path = get_absolute_path_ex(curr, &pathbuf, &mode, &flags);
		} else {
			path = get_absolute_path_ex_with_buffer(curr, pathbuf, RS_BUFFER_SIZE, &mode, &flags);
		}

		if ((path) && !(flags & S_IMMUTABLE)) { /*(pathbuf)*/

		#if 0
			/* 2. Restrict case - parent process is adbd. */
		#if defined(RS_IS_ANDROID_8_1_ABOVE)
			if ((sec_check_execpath(parent_mm, "/system/bin/adbd"))
			#if defined(RS_IS_ANDROID_11_ABOVE)
				|| (sec_check_execpath(parent_mm, "/system/bin/vadbd"))
				|| (sec_check_execpath(parent_mm, "/system/bin/vusbd"))
			#endif
				)
		#else
			if (sec_check_execpath(parent_mm, "/sbin/adbd"))
		#endif
			{
				/*will prevent adbd -> shell as root*/
				shellcred = prepare_creds();
				if (!shellcred) {
					ret = 1;
					goto out;
				}

				shellcred->uid = 2000;
				shellcred->gid = 2000;
				shellcred->euid = 2000;
				shellcred->egid = 2000;

				commit_creds(shellcred);
				RS_LOG("exe,adbd spawn\n");
				ret = 0;
				goto out;
			}
		#endif

		#if 0
			/* 3. Restrict case - execute file in /data directory.
			*/

			if (STRNCMP(path, check_strings[csDataSlash].str, check_strings[csDataSlash].str_len, csDataSlash) == 0) {
				goto out;
			}
		#endif

			/* 4. Restrict case - parent's privilege is not root. */
			if (task_is_not_root_privilege(parent_tsk) && (task_maybe_injected(parent_tsk) == 0)) {
				ret = execve_check_parent_not_root(parent_tsk, path, pathbuf, mode);

			#if 1
				goto out;
			#else
				if (ret >= 0)
					goto out;
				else
					/*ret == -1*/
					ret = 1;
			#endif
			}
		#if 1
			else {
				/*判断当前是否 su 在调 exec ？*/
				int need_check_path = 0;
				noused int need_check_suid = 0;
			#if defined(CONFIG_EXEC_MORE_RESTRICT)
				/*int is_system_file = 0;*/
			#endif

				char *pName;

				pName = strrchr(path, '/');

			#if defined(CONFIG_EXEC_MORE_RESTRICT)
				if ((pName <= path)
					|| (/*is_system_file = */(STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0))
					|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
					|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
					|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
				#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
					|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
				#endif
					) {
					/*OK*/
				} else {
					/*RS_LOG("exe,path:%s\n", path);*/

					if (is_execve_allowed_file(path, pName, &pathbuf)) {
						/*OK*/
						/*RS_LOG("exe,path ok 0\n");*/
					} else {
						RS_LOG("exe,path err 0.1\n");
						/*ret = 1;*/
						goto out;
					}
				}
			#endif

				if (pName) {
					if (pName[0] == '.') {
						need_check_path = 1;
					} else {
						char *pSu = strrchr(pName, 's');

						/*XXXsu 或 XXXus*/
						if ((pSu) && (((pSu[1] == 'u') && ((pSu[2] == '\0') || (pSu[2] == 'd'))) /*su结尾或带"sud"*/
							|| ((pSu > pName) && (((pSu[-1] == 'u') && (pSu[1] == '\0')) /*us结尾*/
							|| ((pSu[-1] == '.') && (((pSu[1] == 'o') && (pSu[2] == '\0')) /*.so结尾*/ /*360 mobilesafe, libsu.so，liba.so.4x.so*/
							|| (pSu[1] == 'u'))) /*.suXXX*/
							)
							)
							)
							) {
							need_check_path = 1;
						} else {
							pSu = strrchr(pName, 'S');
							if ((pSu) && (pSu[1] == 'U') && (pSu[2] == 'D')) { /*带"SUD", 99SuperSUDaemon*/
								need_check_path = 1;
							} else if (strnstr(path, "/.", (pathbuf + RS_BUFFER_SIZE - 1 - path))) {
								need_check_path = 1;
							}
						}
					}
				}

			#if !defined(CONFIG_EXEC_MORE_RESTRICT)
				/*若定义了CONFIG_EXEC_MORE_RESTRICT，不满足路径条件的上面已先排除了*/
				/* Set-uid? or Set-gid? */
				if ((mode & S_ISUID) || ((mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP))) {
					need_check_suid = 1;
				}
			#endif

				if (need_check_path) {
					noused int allowed; /* = 1;*/

				#if defined(CONFIG_EXEC_MORE_RESTRICT)
					allowed = 0;

					#if 0
					if (is_system_file) {
					#if !defined(RS_WITHOUT_BBKSU)
						if (STRCMP(path, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0) {
							/*如果是 bbksu，且非静态(maps中带libc.so)，或是静态，但大于 128KB，则不许*/
							allowed = 0;
						} else
					#endif
						{
							allowed = 0;
						}
					} else {
						allowed = 0;
					}
					#endif
				#else
					allowed = 1;

					if ((strrchr(path, '/') <= path)
						|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
						|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
						|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
						|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
					#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
						|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
					#endif
						) {
						allowed = 0;
					} else {
						/*if (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)*/
						{
							allowed = 0;

						#if 0
						#if !defined(RS_WITHOUT_BBKSU)
							if (STRCMP(path, sid_check_strings[sidcsBBKSu].str, sidcsBBKSu) == 0) {
								/*如果是 bbksu，且非静态(maps中带libc.so)，或是静态，但大于 128KB，则不许*/
								allowed = 0;
							} else
						#endif
							{
								allowed = 0;
							}
						#endif
						}

					}
				#endif

					if (!allowed) {
						RS_LOG("exe,path err 1\n");
						/*ret = 1;*/
						goto out;
					}
				}

			#if !defined(CONFIG_EXEC_MORE_RESTRICT)
				if (need_check_suid) {
				#if 1
					if ((strrchr(path, '/') <= path)
						|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
						|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
						|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
						|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
					#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
						|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
					#endif
						) {
						/*OK*/
					} else {
						RS_LOG("exe,bad suid\n");
						/*ret = 1;*/
						goto out;
					}
				#else
					/*bbksu已按路径排除*/
					int i;
					for (i = 1; i < ARRAY_SIZE(sid_check_strings); i++) {
						if (STRCMP(path, sid_check_strings[i].str, i) == 0) {
							break;
						}
					}

					if (i < ARRAY_SIZE(sid_check_strings)) {
					} else {
						RS_LOG("exe,check sid fail\n");
						/*ret = 1;*/
						goto out;
					}
				#endif
				}
			#endif

			#if 1
				/*是否在执行脚本*/
				if ((is_init_task(parent_tsk)) && (STRCMP(path, check_strings[SH_STR_IDX].str, SH_STR_IDX) == 0)) {
				#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
					ret = execve_check_init_sh(curr, pathbuf, check_arg_type_ptr);
				#else
					ret = execve_check_init_sh(curr, pathbuf, argv);
				#endif

					if (ret) {
						goto out;
					}
				}
			#endif
				/*RS_LOG("exe,parent OK,%s\n", path);*/
				ret = 0;
			}
		#else

			{
				RS_LOG("exe,1,%s\n", path);

				if (
					(strrchr(path, '/') <= path)
					|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
					|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
					|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
					|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
				#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
					|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
				#endif
				) {
					path = get_absolute_path_with_buffer(parent_tsk, pathbuf, RS_BUFFER_SIZE);
					if (path) {
						RS_LOG("exe,2,%s\n", path);
						if (
							(strrchr(path, '/') <= path)
							|| (STRNCMP(path, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
							|| (STRNCMP(path, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
							|| (STRNCMP(path, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
							|| (STRNCMP(path, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
						#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
							|| (STRNCMP(path, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
						#endif
						) {
							RS_LOG("exe OK\n");

							ret = 0;
						}
					}
				}
			}
		#endif

		}
	}

	if (ret == 0) {
		if (task_maybe_injected(curr)) {
			ret = 1;
		}
	}

out:
	/*if (parent_mm)*/
	/*	mmput(parent_mm);*/
	if (parent_tsk) {
		put_task_struct(parent_tsk);
	}

out_free_path:
	free_path(pathbuf);
	/*RS_LOG("exe,end,%d\n", ret);*/

	return ret;
}
#endif


#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_common_check_without_argv RS_HIDE(_A4)
#endif
noused notrace static int do_execve_common_check_without_argv(const char *filename,
	int *check_arg_type_ptr
	)
{
#if defined(CONFIG_EXEC_RESTRICT)
	int ret;
	struct task_struct *curr;

	if (is_op_permitted_no_fc(ovExec)) {
	#if 0
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	#else
		ret = 0;
		goto out;
	#endif
	}

	curr = current;

	if (check_arg_type_ptr) {
		*check_arg_type_ptr = catInvalid;
	}

	if (CHECK_ROOT_UID(curr)) {
		if (do_execve_check_internal(filename, check_arg_type_ptr)) {
			RS_LOG("Restricted execve_comm. PID = %d(%s) "
				"PPID = %d(%s),%s\n",
				curr->pid, curr->comm,
				curr->parent->pid, curr->parent->comm, filename);

		#if defined(PASS_EXEC_RESTRICT)
			ret = 0;
		#else
			if (is_op_bypassed_no_fc(ovExec)) {
			#if 0
				if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC)) {
					ret = 0;
					goto out;
				} else {
					ret = -EPERM;
					goto out_checked;
				}
			#else
				ret = 0;
				goto out;
			#endif
			} else {
				ret = -EPERM;
				goto out_checked;
			}
		#endif
		}
	}
out:
out_checked:
	return ret;
#else
	return 0;
#endif

}
#endif

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_check RS_HIDE(_A5)
#endif
noused notrace
noinline int do_execve_check(const char *filename, const char __user *const __user *__argv)
{
#if defined(CONFIG_EXEC_RESTRICT)
	int ret;
	struct task_struct *curr;

	if (is_op_permitted_no_fc(ovExec)) {
	#if 0
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	#else
		ret = 0;
		goto out;
	#endif
	}

	curr = current;

	if (CHECK_ROOT_UID(curr)) {
		struct user_arg_ptr argv = { .ptr.native = __argv };

		if (do_execve_check_internal(filename, argv)) {
			RS_LOG("Restricted sys_execve. PID = %d(%s) "
				"PPID = %d(%s),%s\n",
				curr->pid, curr->comm,
				curr->parent->pid, curr->parent->comm, filename);

		#if defined(PASS_EXEC_RESTRICT)
			ret = 0;
		#else
			if (is_op_bypassed_no_fc(ovExec)) {
#if 0
				if (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC)) {
					ret = 0;
					goto out;
				} else {
					ret = -EPERM;
					goto out_checked;
				}
			#else
				ret = 0;
				goto out;
			#endif
			} else {
				ret = -EPERM;
				goto out_checked;
			}
		#endif
		}
	}
#endif

out:
out_checked:
	return ret;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __getcwd RS_HIDE(_A7)
#endif
noused notrace static noinline int __getcwd(char *buff, int size)
{
	mm_segment_t old_fs;
	int ret;

	old_fs = get_fs();
	set_fs(get_ds());

#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_getcwd(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)buff;
		arg_regs.regs[1] = (u64)size;

		ret = __arm64_sys_getcwd((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_getcwd((char __user *)buff, size);
#endif

	set_fs(old_fs);

	return ret;
}

#if defined(CONFIG_EXEC_SU_RESTRICT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_su_check_internal RS_HIDE(_A8)
#endif
noused notrace static noinline int do_execve_su_check_internal(const char *filename
#if defined(CONFIG_EXEC_SU_CHECK_PATH_BY_ENV)
	, struct user_arg_ptr envp
#endif
	)
{
#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
	return 0;
#else
	int ret;
	struct task_struct *curr;
	const char *pName;
	const char *name_part;

	curr = current;

	/*sys_execve not called in kernel space*/
#if 0
	if (/*(!filename) ||*/ /* (!curr->mm) || (is_init_task(curr)) ||*/ ((curr->flags & PF_KTHREAD) && (!curr->mm)
			&& (!task_in_call_from_user(curr)))) {
		return 0;
	}
#endif

	ret = 0;

	pName = strrchr(filename, '/');
	name_part = pName;
	if (pName < filename) {
		pName = filename;
	} else {
		pName++;
	}

	if ((pName[0] == 's') && (pName[1] == 'u') && (pName[2] == '\0')) {
	#if defined(CONFIG_EXEC_SU_CHECK_PATH)
		const char *real_filename = filename;
		char *pathbuf = NULL;
		int need_check = 0;

		if (filename[0] != '/') {
			/*需要绝对路径, 系统自身的多数是使用绝对路径*/
			const char *temp_filename = try_get_real_filename_no_follow_link(filename, &pathbuf);
			if ((temp_filename) && (temp_filename != filename)) {
				real_filename = temp_filename;
				name_part = strrchr(temp_filename, '/');

			#if defined(CONFIG_EXEC_SU_CHECK_PATH_BY_ENV)
				if (name_part < temp_filename) {
					pName = temp_filename;
				} else {
					pName = name_part + 1;
				}
			#endif
			}
		}

		if ((name_part <= real_filename)
			|| (STRNCMP(real_filename, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
			|| (STRNCMP(real_filename, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
			|| (STRNCMP(real_filename, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
			|| (STRNCMP(real_filename, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0)
		#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
			|| (STRNCMP(real_filename, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
		#endif
			) {
			need_check = 1;
		}
	#if defined(CONFIG_EXEC_SU_CHECK_PATH_BY_ENV)
		else if (pName > real_filename) {
			int real_filename_len = pName - real_filename;
			int i = 0;
			char *buffer = NULL;
			char *cwd_buffer = NULL;

			for (;;) {
				int len;
				const char __user *p = get_user_arg_ptr(envp, i);

				if ((!p) || (IS_ERR(p)))
					break;

				len = strnlen_user(p, MAX_ARG_STRLEN);

				if (len) {
					int copy_len;

					if (!buffer) {
						buffer = (char *)RS_GET_TWO_BUFFER();
						if (!buffer) {
							ret = -ENOMEM;
							goto out_of_env_chk;
						}
					}

					if (len >= (RS_BUFFER_SIZE * 2))
						copy_len = (RS_BUFFER_SIZE * 2) - 1;
					else
						copy_len = len;

					len -= copy_len;

					if ((!copy_from_user(buffer, p, copy_len))) {
						p += copy_len;
						buffer[copy_len] = '\0';

						/*RS_LOG("exe su,env:%s\n", buffer);*/
						if ((buffer[0] == 'P') && (buffer[1] == 'A')
							&& (buffer[2] == 'T') && (buffer[3] == 'H')
							&& (buffer[4] == '=')/* && (buffer[5])*/) {
							int cmp_len, cwd_len;
							char *ptr = &buffer[5];
							char *start_ptr = ptr;
							char ch;

						path_loop:

							do {
								while ((ch = *ptr++) && (ch != ':')) {
								}

								if ((ch == ':') || ((!ch) && (!len))) {
									cmp_len = ptr - start_ptr - 1;

									if (cmp_len == 0) {
										/*current working directory*/
										if (!cwd_buffer) {
											cwd_buffer = (char *)RS_GET_BUFFER();
											if (!cwd_buffer) {
												ret = -ENOMEM;
												goto out_of_env_chk;
											}

											ret = __getcwd(cwd_buffer, RS_BUFFER_SIZE);
											if (ret < 0) {
												goto out_of_env_chk;
											}

											cwd_len = strlen(cwd_buffer);
										}

										if ((cwd_len <= real_filename_len)
											&& (strncmp(cwd_buffer, real_filename, cwd_len) == 0)
											&& (real_filename[cwd_len] == '/')) {
											need_check = 1;
											break;
										}
									} else if ((cmp_len <= real_filename_len)
										&& (strncmp(start_ptr, real_filename, cmp_len) == 0)
										&& (real_filename[cmp_len] == '/')) {
										need_check = 1;
										break;
									}

									start_ptr = ptr;
								}
							} while (ch);

							if ((!need_check) && (len)) {
								/*some content still not copied*/
								cmp_len = ptr - start_ptr - 1;

								memmove(buffer, start_ptr, cmp_len);

								if (len >= ((RS_BUFFER_SIZE * 2) - cmp_len))
									copy_len = (RS_BUFFER_SIZE * 2) - 1 - cmp_len;
								else
									copy_len = len;

								if ((copy_from_user(&buffer[cmp_len], p, copy_len))) {
									ret = -EFAULT;
									goto out_of_env_chk;
								}

								buffer[cmp_len + copy_len] = '\0';
								p += copy_len;
								len -= copy_len;
								start_ptr = buffer;
								ptr = &buffer[cmp_len];

								goto path_loop;
							}

							break;
						}

					}
				}

				if (i++ >= MAX_ARG_STRINGS)
					break;

				if (fatal_signal_pending(curr)) {
					ret = -ERESTARTNOHAND;
					goto out_of_env_chk;
				}

				cond_resched();
			}

		out_of_env_chk:

			if (cwd_buffer) {
				RS_FREE_BUFFER(cwd_buffer);
			}

			if (buffer) {
				RS_FREE_TWO_BUFFER(buffer);
			}
		}
	#endif  /*CONFIG_EXEC_SU_CHECK_PATH_BY_ENV*/

		if (pathbuf) {
			RS_FREE_BUFFER(pathbuf);
		}

		if ((need_check) && (can_exec_su())) {
			ret = -EPERM;
		}
	#else  /*CONFIG_EXEC_SU_CHECK_PATH*/
		if (can_exec_su()) {
			ret = -EPERM;
		}
	#endif
	}

	return ret;
#endif
}

#endif /*CONFIG_EXEC_SU_RESTRICT*/

enum {
	selsInit,
	selsInitShell,
	selsSu,
	selsCurrent,
	selsShell,

	/*and new items before selsInvalid*/
	selsInvalid,
};

#if defined(RS_ENCRYPT_STR)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sel_strings RS_HIDE(_B0)
#endif
noused static const check_string sel_strings[selsInvalid] = {
	{
		"\xCB\x87\xCE\x81\xD3\xD7\xD1\xC3\x8C\xC6\x84",
		sizeof("u:r:init:s0") - 1
	},
	{
		"\xC8\x86\xC9\x80\xD0\xD6\xDE\xC2\xEA\xC7\xDB\xD7\xDD\xDC\x95\xDD\x9D",
		sizeof("u:r:init_shell:s0") - 1
	},
	{
		"\xC9\x81\xC8\x83\xCB\xC2\x8C\xC6\x84",
		sizeof("u:r:su:s0") - 1
	},
	{
		"\xD8\xCF\xCB\xCA\xD2\xD8\xC1",
		sizeof("current") - 1
	},
	{
		"\xCF\x83\xCA\x8D\xC5\xDD\xD1\xDF\xDE\x8B\xC3\x9F",
		sizeof("u:r:shell:s0") - 1
	},
};

#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define sel_strings RS_HIDE(_B0)
#endif
noused static const check_string sel_strings[selsInvalid] = {
	{
		"u:r:init:s0",
		sizeof("u:r:init:s0") - 1
	},
	{
		"u:r:init_shell:s0",
		sizeof("u:r:init_shell:s0") - 1
	},
	{
		"u:r:su:s0",
		sizeof("u:r:su:s0") - 1
	},
	{
		"current",
		sizeof("current") - 1
	},
	{
		"u:r:shell:s0",
		sizeof("u:r:shell:s0") - 1
	},
};

#endif

#if 1

#undef CHECK_SHELL_UID
#define CHECK_SHELL_UID(x) ((__kuid_val((x)->cred->uid) == AID_SHELL) && (__kgid_val((x)->cred->gid) == AID_SHELL))

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define is_dbg_task RS_HIDE(_BA)*/
#endif
noused notrace int is_dbg_task(struct task_struct *task, int check_path)
{
#if defined(CONFIG_SECURITY_SELINUX) && defined(RS_VIVOROOT_SUPPORT)
	int ret;

#if 1
	if (CHECK_ROOT_UID(task)) {
		ret = do_op_verify(ovDynTransToSu, 0);
		if (ret < 0) {
		} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
			ret = -EPERM;
		} else {
		#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
			set_vars_global_flag(gfVivoRootTriggered, 1);
		#endif

			ret = 1;
		}
	} else {
		ret = -EPERM;
	}

	return ret;
#else
	if (!task) {
		return 0;
	}

	ret = 0;

	get_task_struct(task);

	if (CHECK_ROOT_UID(task) || CHECK_SHELL_UID(task)) {
		struct task_struct *parent;
		struct task_struct *group_leader = NULL;

		RS_READ_TASK_LOCK();

		parent = task->parent;
		if ((parent) && (parent == task->real_parent)) {
			if (is_init_task(parent)) {
				group_leader = task->group_leader;
				if (!group_leader) {
					group_leader = task;
				} else {
					get_task_struct(group_leader);
				}
			}
		}

		RS_READ_TASK_UNLOCK();

		if (group_leader) {
			if (!(group_leader->flags & get_task_reap_init_flag())) {
				char *p_comm = group_leader->comm;

				if (((p_comm[0] == 'a') && (p_comm[1] == 'd') && (p_comm[2] == 'b')
					&& (p_comm[3] == 'd') && (p_comm[4] == '\0'))
				#if defined(RS_IS_ANDROID_11_ABOVE)
					|| ((p_comm[0] == 'v') && (p_comm[1] == 'a') && (p_comm[2] == 'd')
					&& (p_comm[3] == 'b') && (p_comm[4] == 'd') && (p_comm[5] == '\0'))
					|| ((p_comm[0] == 'v') && (p_comm[1] == 'u') && (p_comm[2] == 's')
					&& (p_comm[3] == 'b') && (p_comm[4] == 'd') && (p_comm[5] == '\0'))
				#endif
					)
				{
					if (task_is_ptraced(group_leader)) {
						/*RS_LOG("is_dbg,ptraced\n");*/

						/*ret = -EPERM;*/
					} else if (!check_path) {
						ret = 1;
					} else {
						char *pathbuf, *path;

						path = get_absolute_path(group_leader, &pathbuf);

						if (path) {
							if ((STRCMP(path, check_strings[csAdbd].str, csAdbd) == 0)
							#if defined(RS_IS_ANDROID_11_ABOVE)
								|| (STRCMP(path, check_strings[csVadbd].str, csVadbd) == 0)
								|| (STRCMP(path, check_strings[csVusbd].str, csVusbd) == 0)
								|| (STRNCMP(path, check_strings[csApexSlash].str, check_strings[csApexSlash].str_len, csApexSlash) == 0)
							#endif
								)
							{
							#if 0
								if (task_has_non_system_lib(group_leader)) {
									RS_LOG("is_dbg,bad lib\n");
								} else
							#endif
								{
									/*RS_LOG("is_dbg,OK\n");*/
									ret = 1;
								}
							}

							free_path(pathbuf);
						}
					}
				}
			}

			if (group_leader != task) {
				put_task_struct(group_leader);
			}
		}
	}

/*out:*/
	put_task_struct(task);

	return ret;
#endif
#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define rs_can_exec RS_HIDE(_BA1)*/
#endif
noused notrace int rs_can_exec(/*struct task_struct *task,*/ const struct linux_binprm *bprm)
{
#if defined(CONFIG_SECURITY_SELINUX) && defined(RS_VIVOROOT_SUPPORT)
	int ret = -EPERM;

	/*if ((task) && CHECK_ROOT_UID(task) && (bprm))*/
	{
		/*RS_LOG("cde,0\n");*/

	#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
		ret = 0;
		goto out_ret;
	#else  /*RS_IS_ENG_BUILD*/

	#if 0
		/*already checked in can_exec_su()*/
		if (is_op_permitted_no_fc(ovSuExec)) {
#if 0
			if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
				ret = 0;
				goto out_ret;
			} else {
				ret = -EPERM;
				goto out_ret;
			}
		#else
			ret = 0;
			goto out_ret;
		#endif
		}
	#endif

	#endif

#if 0
		ret = do_op_verify(ovSuExec, 0);
		if (ret < 0) {
			noused struct task_struct *task = current;
			RS_LOG("Restricted daemon. PID = %d(%s)/n", task->pid, task->comm);

		#if defined(PASS_EXEC_SU_RESTRICT)
			ret = 0;
			goto out_ret;
		#else
			if ((is_op_bypassed_no_fc(ovSuExec))
				) {
				ret = 0;
				goto out_ret;
			}
		#endif
		}
#endif

	}

#if defined(CONFIG_EXEC_SU_RESTRICT)
	/*if CONFIG_ROOT_RESTRICT is disabled, the system
	 can't boot up normally after SuperSU installed
	*/
	if (!can_exec_su()) /*(ret)*/
#endif
	{
		char *buffer = NULL;
		int need_real_filename = 0;
		const char *filename = bprm->filename;
		struct file *filp;
		int flag = 0;

		/*RS_LOG("cde,1\n");*/

		if (filename) {
			if (filename[0] != '/') {
				need_real_filename = 1;
			} else if (STRNCMP(filename, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) == 0) {
				/*is "/dev/fd/xx" ?*/
				const char *ptr = filename + sizeof("/dev/") - 1;
				if ((ptr[0] == 'f') && (ptr[1] == 'd') && (ptr[2] == '/')) {
					need_real_filename = 1;
				}
			}
		} else {
			need_real_filename = 1;
		}

		if (need_real_filename) {
			filp = bprm->file;
			if (filp)
				flag = 1;
		}

		if (flag) {
			buffer = (char *)RS_GET_BUFFER();

			if (buffer) {
				char *ret_ptr = D_PATH_EX(&(filp->f_path), buffer, RS_BUFFER_SIZE);

				if (ret_ptr) {
					filename = ret_ptr;
				} else {
					/*RS_LOG("cde,1.1\n");*/
					RS_FREE_BUFFER(buffer);
					buffer = NULL;

					ret = -ENOMEM;
					goto out_ret;
				}
			} else {
				/*RS_LOG("cde,1.2\n");*/
				ret = -ENOMEM;
				goto out_ret;
			}
		}

		if ((filename) && ((STRNCMP(filename, check_strings[csSystemSlash].str, check_strings[csSystemSlash].str_len, csSystemSlash) == 0)
			|| (STRNCMP(filename, check_strings[csSbinSlash].str, check_strings[csSbinSlash].str_len, csSbinSlash) == 0)
			|| (STRNCMP(filename, check_strings[csTmpSlash].str, check_strings[csTmpSlash].str_len, csTmpSlash) == 0)
			|| ((!need_real_filename) && (STRNCMP(filename, check_strings[csVendorSlash].str, check_strings[csVendorSlash].str_len, csVendorSlash) == 0))
		#if defined(RS_OEM_PARTITION_HAS_EXE_FILE)
			|| (STRNCMP(filename, check_strings[csOemSlash].str, check_strings[csOemSlash].str_len, csOemSlash) == 0)
		#endif
			)) {
			ret = 0;
		}

		if (buffer) {
			RS_FREE_BUFFER(buffer);
		}

		if (ret) {
			noused struct task_struct *task = current;
			RS_LOG("Restricted daemon. PID = %d(%s)/n", task->pid, task->comm);

		#if 0
			/*already checked in can_exec_su()*/

		#if defined(PASS_EXEC_SU_RESTRICT)
			ret = 0;
		#else
			if ((is_op_bypassed_no_fc(ovSuExec))
				)
				ret = 0;
			else
				goto out_ret;
		#endif

		#endif
		}
	}

out_ret:
	return ret;
#else
	return -EPERM;
#endif
}

#if 0

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define check_dbg_task RS_HIDE(_BB)*/
#endif
noused notrace
static noinline int check_dbg_task(struct task_struct *task,
	char *name, char *value, size_t *size_p)
{
#if defined(CONFIG_SECURITY_SELINUX) && defined(RS_VIVOROOT_SUPPORT)
	size_t size;

	if (!size_p) {
		return 0;
	}

	/*refer to "setcon((char *)root_seclabel)" in adb.c: adb_main()*/
	size = *size_p;

	if ((size >= sel_strings[selsSu].str_len)
		&& (size <= sel_strings[selsInit].str_len)
		&& (name) && (STRCMP(name, sel_strings[selsCurrent].str, selsCurrent) == 0)
		&& (value) && (STRCMP(value, sel_strings[selsSu].str, selsSu) == 0)) {
		if (/*(task) &&*/ (is_dbg_task(task, 1))) {
			memcpy(value, sel_strings[selsInit].str, sel_strings[selsInit].str_len + 1);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(value, sel_strings[selsInit].str_len, selsInit);
		#endif

			*size_p = sel_strings[selsInit].str_len;

			set_task_dbg_flag(task);

			return 1;
		}
	}
#endif

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
/*#define get_shell_sctx RS_HIDE(_BC)*/
#endif
noused notrace
static noinline int get_shell_sctx(char **value, size_t *size_p)
{
#if defined(CONFIG_SECURITY_SELINUX) && defined(RS_VIVOROOT_SUPPORT)
	char *buf;

	if ((!value) || (!size_p))
		return -EINVAL;

	buf = (char *)kmalloc(sel_strings[selsShell].str_len + 1, GFP_KERNEL);
	if (!buf) {
		return -ENOMEM;
	}

	memcpy(buf, sel_strings[selsShell].str, sel_strings[selsShell].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(buf, sel_strings[selsShell].str_len, selsShell);
#endif

	*value = buf;
	*size_p = sel_strings[selsShell].str_len;

	return 0;
#else
	return -ENOENT; /*-EINVAL;*/
#endif
}

#endif

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __rs_might_fault RS_HIDE(_BC1)
#endif
#if defined(CONFIG_PROVE_LOCKING) || defined(CONFIG_DEBUG_ATOMIC_SLEEP)
noused notrace static void __rs_might_fault(void)
{
	/*
	 * Some code (nfs/sunrpc) uses socket ops on kernel memory while
	 * holding the mmap_sem, this is safe because kernel memory doesn't
	 * get paged out, therefore we'll never actually fault, and the
	 * below annotations will generate false positives.
	 */
	/*if (segment_eq(get_fs(), KERNEL_DS))
		return;
	*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
	/*
	 * it would be nicer only to annotate paths which are not under
	 * pagefault_disable, however that requires a larger audit and
	 * providing helpers like get_user_atomic.
	 */
	if (in_atomic())
		return;
#else
	if (pagefault_disabled())
		return;
#endif

	__might_sleep("", 0, 0); /*__FILE__, __LINE__, 0);*/

#if defined(CONFIG_DEBUG_ATOMIC_SLEEP)
	{
		struct mm_struct *mm;
		mm = current->mm;
		if (mm)
			might_lock_read(&mm->mmap_sem);
	}
#endif
}
#else
noused notrace static void __rs_might_fault(void)
{
}
#endif

#ifdef __GNUC__
#define	__GNUC_PREREQ(x, y)						\
	((__GNUC__ == (x) && __GNUC_MINOR__ >= (y)) ||			\
	(__GNUC__ > (x)))
#else
#define	__GNUC_PREREQ(x, y)	0
#endif

#if __GNUC_PREREQ(2, 96)
#define	__predict_true(exp)	__builtin_expect((exp) != 0, 1)
#define	__predict_false(exp)	__builtin_expect((exp) != 0, 0)
#else
#define	__predict_true(exp)	(exp)
#define	__predict_false(exp)	(exp)
#endif


#if defined(RS_SET_PERSIST_PROPERTY)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mktemp_internal RS_HIDE(_Bj)
#endif
noused notrace static ssize_t mktemp_internal(char *path, int slen, int mode)
{
#define MKTEMP_NAME	0
#define MKTEMP_FILE	1
#define MKTEMP_DIR	2

#define TEMPCHARS	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
#define NUM_CHARS	(sizeof(TEMPCHARS) - 1)
#define MIN_X		6

#ifndef nitems
#define nitems(_a)	(sizeof((_a)) / sizeof((_a)[0]))
#endif

	char *start, *cp, *ep;
	const char *tempchars = TEMPCHARS;
	unsigned int tries;
	struct kstat sb;
	size_t len;
	/*int fd;*/
	ssize_t error;

	len = strlen(path);
	if (len < MIN_X || slen < 0 || (size_t)slen > len - MIN_X) {
		return(-EINVAL);
	}
	ep = path + len - slen;

	for (start = ep; start > path && start[-1] == 'X'; start--)
		;
	if (ep - start < MIN_X) {
		return(-EINVAL);
	}

	tries = INT_MAX;
	do {
		cp = start;
		do {
			unsigned short rbuf[16];
			unsigned int i;

			/*
			 * Avoid lots of arc4random() calls by using
			 * a buffer sized for up to 16 Xs at a time.
			 */
			RS_GET_RANDOM_BYTES(rbuf, sizeof(rbuf)); /*arc4random_buf(rbuf, sizeof(rbuf));*/
			for (i = 0; i < nitems(rbuf) && cp != ep; i++)
				*cp++ = tempchars[rbuf[i] % NUM_CHARS];
		} while (cp != ep);

		switch (mode) {
		case MKTEMP_NAME: {
			mm_segment_t old_fs = get_fs();
			set_fs(get_ds()/*KERNEL_DS*/);
			error = vfs_lstat(path, &sb);
			set_fs(old_fs);
			if (error < 0)
				return ((error == -ENOENT) ? 0 : error);
		}
		break;
		case MKTEMP_FILE: {
			struct file *file = NULL;
			extern void fput(struct file *);

			mm_segment_t old_fs = get_fs();
			set_fs(get_ds()/*KERNEL_DS*/);
			file = local_filp_open(path, O_CREAT|O_EXCL|O_RDWR|O_CLOEXEC|O_NOATIME, S_IRUSR|S_IWUSR);
			set_fs(old_fs);
			if (IS_ERR(file)) {
				error = PTR_ERR(file);
				file = NULL;
				if (error != -EEXIST)
					return (ssize_t)(ERR_PTR(error));
			}
			return (ssize_t)file; /*fput(file);*/
		}
		break;
		case MKTEMP_DIR: {
			mm_segment_t old_fs = get_fs();
			set_fs(get_ds()/*KERNEL_DS*/);
		#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
			{
				extern long __arm64_sys_mkdir(const struct pt_regs *regs);
				struct rs_pt_regs arg_regs;
				arg_regs.regs[0] = (u64)path;
				arg_regs.regs[1] = (u64)(S_IRUSR|S_IWUSR|S_IXUSR);

				error = __arm64_sys_mkdir((const struct pt_regs *)&arg_regs);
			}
		#else
			error = sys_mkdir(path, S_IRUSR|S_IWUSR|S_IXUSR);
		#endif
			set_fs(old_fs);
			if (!error)
				return 0;
			if (error != -EEXIST)
				return error;
		}
		break;
		}
	} while (--tries);

	return -EEXIST;
}

/*static char *_mktemp(char *);*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define _mktemp RS_HIDE(_Bk)
#endif
noused notrace static char *_mktemp(char *path)
{
	if (mktemp_internal(path, 0, MKTEMP_NAME) == -1)
		return NULL;
	return path;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mktemp RS_HIDE(_Bl)
#endif
noused notrace static char *mktemp(char *path)
{
	return _mktemp(path);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mkstemp RS_HIDE(_Bm)
#endif
noused notrace static struct file *mkstemp(char *path)
{
	return (struct file *)(mktemp_internal(path, 0, MKTEMP_FILE));
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mkstemps RS_HIDE(_Bn)
#endif
noused notrace static struct file *mkstemps(char *path, int slen)
{
	return (struct file *)(mktemp_internal(path, slen, MKTEMP_FILE));
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mkdtemp RS_HIDE(_Bo)
#endif
noused notrace static char *mkdtemp(char *path)
{
	int error;

	error = mktemp_internal(path, 0, MKTEMP_DIR);
	return (error) ? NULL : path;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ___sys_rename RS_HIDE(_Bp)
#endif
noused notrace static int ___sys_rename(const char *oldname, const char *newname)
{
	int ret;
	mm_segment_t curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_rename(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)oldname;
		arg_regs.regs[1] = (u64)newname;

		ret = __arm64_sys_rename((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_rename((const char __user *)oldname, (const char __user *)newname);
#endif
	set_fs(curr_fs);

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define ___sys_unlink RS_HIDE(_Bq)
#endif
noused notrace static int ___sys_unlink(const char *pathname)
{
	int ret;
	mm_segment_t curr_fs = get_fs();
	set_fs(get_ds()/*KERNEL_DS*/);
#if defined(RS_ARCH_USE_SYSCALL_WRAPPER)
	{
		extern long __arm64_sys_unlink(const struct pt_regs *regs);
		struct rs_pt_regs arg_regs;
		arg_regs.regs[0] = (u64)pathname;

		ret = __arm64_sys_unlink((const struct pt_regs *)&arg_regs);
	}
#else
	ret = sys_unlink((const char __user *)pathname);
#endif
	set_fs(curr_fs);

	return ret;
}


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define write_persistent_property RS_HIDE(_Br)
#endif
noused notrace static void write_persistent_property(const char *name, const char *value)
{
	#define PERSISTENT_PROPERTY_DIR  "/data/property"
	/*char tempPath[PATH_MAX];*/
	/*char path[PATH_MAX];*/
	char *tempPath, *path;
	char *buffer;
	char *fmt_buffer, *dir_buffer;
	/*int fd;*/
	struct file *filp;
	extern void fput(struct file *);

	if (((check_strings[csPersistPropTempFmt].str_len + 1) + (check_strings[csPersistPropDir].str_len + 1) + 2) >= RS_BUFFER_SIZE) {
		goto out; /*#error "persistent property dir buffer too small!"*/
	}

	buffer = (char *)RS_GET_TWO_BUFFER();
	if (!buffer) {
		goto out;
	}

	fmt_buffer = (char *)RS_GET_BUFFER();
	if (!fmt_buffer) {
		goto out_no_fmt_buffer;
	}

	dir_buffer = fmt_buffer + sizeof("%s/.temp.XXXXXX") + 2;

	tempPath = buffer;
	path = buffer + RS_BUFFER_SIZE;

	memcpy(fmt_buffer, check_strings[csPersistPropTempFmt].str, check_strings[csPersistPropTempFmt].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(fmt_buffer, check_strings[csPersistPropTempFmt].str_len, csPersistPropTempFmt);
#endif

	memcpy(dir_buffer, check_strings[csPersistPropDir].str, check_strings[csPersistPropDir].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dir_buffer, check_strings[csPersistPropDir].str_len, csPersistPropDir);
#endif

	snprintf(tempPath, RS_BUFFER_SIZE, fmt_buffer/*"%s/.temp.XXXXXX"*/, dir_buffer);
	filp = mkstemp(tempPath);
	if (IS_ERR(filp)) {
		/*ERROR("Unable to write persistent property to temp file %s errno: %d\n", tempPath, errno);*/
		goto out_with_buffer;
	}

	safe_vfs_write(filp, value, strlen(value), 0); /*write(fd, value, strlen(value));*/
	vfs_fsync(filp, 0); /*fsync(fd);*/
	filp_close(filp, RS_FILP_CLOSE_ID); /*fput(filp);*/ /*close(fd);*/

	memcpy(fmt_buffer, check_strings[csPersistPropPathFmt].str, check_strings[csPersistPropPathFmt].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(fmt_buffer, check_strings[csPersistPropPathFmt].str_len, csPersistPropPathFmt);
#endif

	snprintf(path, RS_BUFFER_SIZE, fmt_buffer/*"%s/%s"*/, dir_buffer, name);
	if (___sys_rename(tempPath, path)) {
		___sys_unlink(tempPath);
		/*ERROR("Unable to rename persistent property file %s to %s\n", tempPath, path);*/
	}

out_with_buffer:
	RS_FREE_BUFFER(fmt_buffer);
out_no_fmt_buffer:
	RS_FREE_TWO_BUFFER(buffer);
out:
	return;
}
#endif

/*#define property_service_socket "/dev/socket/"PROP_SERVICE_NAME*/

#if defined(CONFIG_ROOT_RESTRICT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_rs_bootup_set RS_HIDE(_Bs_)
#endif
static int g_rs_bootup_set;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_bootup_set RS_HIDE(_Bs_1)
#endif
noused notrace static int rs_get_bootup_set(void)
{
	return g_rs_bootup_set;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_try_set_bootup_done RS_HIDE(_Bs)
#endif
noused notrace static int rs_try_set_bootup_done(void)
{

	if (!g_rs_bootup_set) {
		noused intptr_t flag;

		g_rs_bootup_set = rs_get_set_value();

	#if 1
		/*RS_LOG("sbd,0\n");*/
	#if defined(CONFIG_RS_CHECK_FUNC)
		if ((!get_vars_global_flag(gfFuncChecksumsOK, &flag)) && (flag))
	#endif
		{
			/*RS_LOG("sbd,1\n");*/
			flag = fcbdfNotInited;

			if ((!get_vars_global_flag(gfBootupDone, &flag)) && (flag == fcbdfNotInited)) {
				/*RS_LOG("sbd,2\n");*/
				set_vars_global_flag(gfBootupDone, fcbdfInited);
			}
		}
	#endif
	}

	return 0;
}
#else
noused notrace static int rs_get_bootup_set(void)
{
	return 1;
}

noused notrace static int rs_try_set_bootup_done(void)
{
	return 0;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_property_service_started RS_HIDE(_Bt)
#endif
noused notrace
static noinline int is_property_service_started(void)
{
	int ret;
	char *socket_path;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_property_service_started RS_HIDE(_Bt_)
#endif
	static int g_property_service_started;

	if (g_property_service_started)
		return g_property_service_started;


	socket_path = (char *)RS_GET_BUFFER();
	if (!socket_path) {
		RS_LOG("ipss,0\n");
		ret = 0;
		goto out;
	}

	memcpy(socket_path, check_strings[csPropServiceSocket].str, check_strings[csPropServiceSocket].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(socket_path, check_strings[csPropServiceSocket].str_len, csPropServiceSocket);
#endif
	if (is_file_exist(socket_path/*property_service_socket*/)) {
		g_property_service_started = rs_get_set_value();

		ret = 1;
	} else {
		ret = 0;
	}

	RS_FREE_BUFFER(socket_path);

out:
	return ret;
}


/* Used to retry syscalls that can return EINTR. */
#define TEMP_FAILURE_RETRY(exp) ({		\
	__typeof__(exp) _rc;				\
	do {								\
		_rc = (exp);					\
	} while ((_rc < 0) && (should_retry_syscall((int)_rc))); \
	_rc; })


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define timespec_from_ms RS_HIDE(_By)
#endif
noused notrace static void timespec_from_ms(struct timespec *ts, const int ms)
{
	ts->tv_sec = ms / 1000;
	ts->tv_nsec = (ms % 1000) * 1000000;
}

#if 0
/*应该放在 security/selinux/hooks.c 里*/
/*change to "u:r:init:s0"*/
/* from net/socket.c */
static struct socket *sockfd_lookup_light(int fd, int *err, int *fput_needed)
{
	struct file *file;
	struct socket *sock;

	*err = -EBADF;
	file = fget_light(fd, fput_needed);
	if (file) {
		sock = sock_from_file(file, err);
		if (sock)
			return sock;
		fput_light(file, *fput_needed);
	}
	return NULL;
}

/*#include "../security/selinux/include/objsec.h"*/
/*#include "flask.h"*/
/*#include "security/selinux/flask.h"*/

int set_socket_peer_selinux_sid(int fd, u32 new_peer_sid)
{
	int err, fput_needed;
	struct socket *sock;

	sock = sockfd_lookup_light(fd, &err, &fput_needed);
	if (sock != NULL) {
		struct sk_security_struct *sksec = sock->sk->sk_security;

		if (sksec->sclass == SECCLASS_UNIX_STREAM_SOCKET
			|| sksec->sclass == SECCLASS_TCP_SOCKET) {
			sksec->peer_sid = new_peer_sid;

			err = 0;
		}

/*out_put:*/
		fput_light(sock->file, fput_needed);
	}
	return err;
}

#endif


#if defined(CONFIG_RS_CHECK_BOOT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_file RS_HIDE(_C5)
#endif
noused notrace static noinline void *read_file(const char *fn, loff_t *_sz, int is_prop_file)
{
	char *data;
	int error;
	loff_t sz;
	struct kstat sb;
	struct file *filp;
	/*struct stat sb;*/
	extern void fput(struct file *);

	data = NULL;
	filp = local_filp_open(fn, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		/*ret = PTR_ERR(filp);*/
		filp = NULL;
		goto out;
	}

	/* for security reasons, disallow world-writable
	 * or group-writable files
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
	error = vfs_getattr(filp->f_path.mnt, filp->f_path.dentry, &sb);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
	error = vfs_getattr(&filp->f_path, &sb,
				STATX_TYPE | STATX_MODE | STATX_SIZE, AT_STATX_SYNC_AS_STAT);
#else
	error = vfs_getattr(&filp->f_path, &sb);
#endif

	if (error) {

		goto out;
	}

#if 0
	if ((sb.mode & (S_IWGRP | S_IWOTH)) != 0) {
		goto out;
	}
#endif

#if 0
	sz = safe_vfs_llseek(filp, 0, SEEK_END);
	if (sz < 0) {
		goto out;
	}

	if (safe_vfs_llseek(filp, 0, SEEK_SET) != 0) {
		goto out;
	}
#else
	sz = sb.size;
#endif

	data = (char *)vmalloc(((is_prop_file) ? (sz + 2) : (sz + 1)));
	if (!data) {
		goto out;
	}

	if (safe_vfs_read(filp, data, sz, 0) != sz) {
		goto out;
	}

	filp_close(filp, RS_FILP_CLOSE_ID); /*fput(filp);*/
	filp = NULL;

	if (is_prop_file) {
		data[sz] = '\n';
		data[sz + 1] = '\0';
	} else {
		data[sz] = '\0';
	}

	if (_sz) {
		*_sz = sz;
	}

	return data;

out:
	if (filp)
		filp_close(filp, RS_FILP_CLOSE_ID); /*fput(filp);*/

	if (data)
		vfree(data);

	return NULL;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define free_read_file_data RS_HIDE(_C6)
#endif
noused notrace static void free_read_file_data(char *data)
{
	if (data)
		vfree(data);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_prop_from_file RS_HIDE(_C7)
#endif
noused notrace static noinline int get_prop_from_file(const char *fn,
	const char *filter,
	const char *in_key, char *out_value, const char *default_value);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_prop_from_buffer RS_HIDE(_C8)
#endif
noused notrace static int get_prop_from_buffer(char *data,
	const char *filter,
	const char *in_key, char *out_value)
{
	char *key, *value, *eol, *sol, *tmp, *fn;
	int len = 0;
#if 1
	size_t flen = 0;

	if (filter) {
		flen = strlen(filter);
	}
#endif

	sol = data;
	while ((eol = _strchr(sol, '\n'))) {
		key = sol;
		*eol++ = 0;
		sol = eol;

		while (isspace(*key))
			key++;
		if (*key == '#')
			continue;

		tmp = eol - 2;
		while ((tmp > key) && isspace(*tmp))
			*tmp-- = 0;

		if (!STRNCMP(key, check_strings[csPropImport].str, check_strings[csPropImport].str_len, csPropImport) && flen == 0) {
			fn = key + (sizeof("import ") - 1);
			while (isspace(*fn))
				fn++;

			key = _strchr(fn, ' ');
			if (key) {
				*key++ = 0;
				while (isspace(*key))
					key++;
			}

			len = get_prop_from_file(fn, key, in_key, out_value, NULL);
			if (len) {
				break;
			}
		} else {
			value = _strchr(key, '=');
			if (!value)
				continue;
			*value++ = 0;

			tmp = value - 2;
			while ((tmp > key) && isspace(*tmp))
				*tmp-- = 0;

			while (isspace(*value))
				value++;

#if 1
			if (flen > 0) {
				if (filter[flen - 1] == '*') {
					if (strncmp(key, filter, flen - 1))
						continue;
				} else {
					if (strcmp(key, filter))
						continue;
				}
			}
#endif

			if (!strcmp(key, in_key)) {
				len = strlen(value);
				if (len >= PROP_VALUE_MAX) {
					len = PROP_VALUE_MAX - 1;
					/*return -1;*/
				}

				memcpy(out_value, value, len);
				out_value[len] = '\0';

				break;
			}

			/*property_set(key, value);*/
		}
	}

	return len;
}

noused notrace static noinline int get_prop_from_file(const char *fn,
	const char *filter,
	const char *in_key, char *out_value, const char *default_value)
{
	char *data;
	/*loff_t sz;*/
	int len;

	data = read_file(fn, NULL/*&sz*/, 1);

	if (data) {
		len = get_prop_from_buffer(data, filter, in_key, out_value);

		if (len <= 0) {
			if (default_value) {
				len = strlen(default_value);
				if (len >= PROP_VALUE_MAX) {
					len = PROP_VALUE_MAX - 1;
				}
				memcpy(out_value, default_value, len);
				out_value[len] = '\0';
			}
		}

		free_read_file_data(data);
	} else {
		len = -1;
	}

	return len;
}
#endif

/*noused static int g_rs_is_eng_by_prop = 0;*/
noused notrace static int set_vars_global_flag(size_t flag_enum, intptr_t flag);
noused notrace static int get_vars_global_flag(size_t flag_enum, intptr_t *flag_ptr);

/*/////////////////////////////////////////////*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_dummy_open RS_HIDE(_lo)
#endif
noused notrace static int rs_dummy_open(struct inode *inode, struct file *file)
{
	return -EACCES; /*return seq_open(file, &rs_kallsyms_op);*/
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_dummy_read RS_HIDE(_lp)
#endif
noused notrace static ssize_t rs_dummy_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_dummy_write RS_HIDE(_lq)
#endif
noused notrace static ssize_t rs_dummy_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	return count;
}

typedef int (*fops_open)(struct inode *, struct file *);

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_absolute_path_with_buffer RS_HIDE(_nh)
#endif
static char *get_absolute_path_with_buffer(struct task_struct *task, char *buffer, int buffer_size);

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_op_verify RS_HIDE(_s1)
#endif
static int do_op_verify(int op_type, ssize_t/*int*/ op_data);
#endif

#if defined(CONFIG_RS_MEM_DEV_WRITE_RESTRICT)

#define RS_MEM_FUNC_BASE (RS_SYM_VAL_TYPE)(PAGE_OFFSET + LINUX_VERSION_CODE + GCC_VERSION_CODE + 0x4D65)

#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)

#if defined(CONFIG_DEVMEM)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_mem_real_open RS_HIDE(_m0_1)
#endif
noused static void *g_mem_real_open;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_mem_open RS_HIDE(_m0_2)
#endif
noused notrace static int rs_mem_open(struct inode *inode, struct file *file)
{
	int ret;
	fops_open real_open;
	struct task_struct *curr = current;

	if (!CHECK_ROOT_UID(curr)) {
		ret = -EPERM;
		goto out;
	}

	if (!g_mem_real_open) {
		ret = -EFAULT;
		goto out;
	}

	ret = do_op_verify(ovAccessDev, 0);
	if (ret < 0) {
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	} else {
		real_open = (fops_open)((RS_SYM_PTR_TYPE)g_mem_real_open + RS_MEM_FUNC_BASE);
		ret = (*real_open)(inode, file);
	}

out:
	return ret;
}
#endif

#if defined(CONFIG_DEVKMEM)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_kmem_real_open RS_HIDE(_m0_3)
#endif
noused static void *g_kmem_real_open;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kmem_open RS_HIDE(_m0_4)
#endif
noused notrace static int rs_kmem_open(struct inode *inode, struct file *file)
{
	int ret;
	fops_open real_open;
	struct task_struct *curr = current;

	if (!CHECK_ROOT_UID(curr)) {
		ret = -EPERM;
		goto out;
	}

	if (!g_kmem_real_open) {
		ret = -EFAULT;
		goto out;
	}

	ret = do_op_verify(ovAccessDev, 0);
	if (ret < 0) {
	} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
		ret = -EPERM;
	} else {
		real_open = (fops_open)((RS_SYM_PTR_TYPE)g_kmem_real_open + RS_MEM_FUNC_BASE);
		ret = (*real_open)(inode, file);
	}

out:
	return ret;
}
#endif

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_mem_dev_write_handler_internal RS_HIDE(_m0)
#endif
noused notrace static int __init rs_reset_mem_dev_write_handler_internal(const char *dev_name, size_t dev_name_len,
#if defined(RS_ENCRYPT_STR)
	int dev_name_str_seed,
#endif
	int *fops_direction_ptr, int fops_step
#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)
	, void **fops_open_store, fops_open new_open
#endif
	)
{
	/*重置/dev/kmem对应kmem_fops(drivers/char/mem.c)中的write函数指针*/
	int ret;
	struct file *filp;
	struct file_operations *fops;
	char *file_path;
	int fops_dir;
	char local_buffer[16];

#if 0
	if (sizeof(file_path) < (check_strings[csDevKmem].str_len + 1)) {
		buffer overrun
		ret = -EINVAL; /*-ENOBUFS;*/
		goto out_ret;
	}
#endif

	if (dev_name_len >= sizeof(local_buffer)) {
		file_path = (char *)RS_GET_BUFFER();
		if (!file_path) {
			ret = -ENOMEM;
			goto out_ret;
		}
	} else {
		file_path = local_buffer;
	}

	memcpy(file_path, dev_name, dev_name_len + 1);
#if defined(RS_ENCRYPT_STR)
	/*if (dev_name_str_seed >= 0)*/
	simple_str_encrypt(file_path, dev_name_len, dev_name_str_seed);
#endif

	filp = local_filp_open(file_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

	if (IS_ERR(filp)) {
		/*5.0 EACCES*/
		ret = PTR_ERR(filp);
		filp = NULL;

		if ((ret == -EACCES) || (ret == -EPERM)) {
			/*kmem_fops null_fops [port_fops] zero_fops, search kmem_fops by null_fops*/
			struct file *null_filp, *zero_filp;
			struct file_operations *null_fops, *zero_fops;

			if (fops_direction_ptr)
				fops_dir = *fops_direction_ptr;
			else
				fops_dir = 0;

			memcpy(file_path, check_strings[csDevNull].str, check_strings[csDevNull].str_len + 1);
		#if defined(RS_ENCRYPT_STR)
			simple_str_encrypt(file_path, check_strings[csDevNull].str_len, csDevNull);
		#endif
			/*strcpy(file_path, "/dev/null");*/
			null_filp = local_filp_open(file_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

			if (IS_ERR(null_filp)) {
				ret = PTR_ERR(null_filp);
				goto out;
			}
			null_fops = (struct file_operations *)null_filp->f_op;
			filp_close(null_filp, RS_FILP_CLOSE_ID);

			if (!null_fops) {
				ret = -ENXIO;
				goto out;
			}

			if (!fops_dir) {
				memcpy(file_path, check_strings[csDevZero].str, check_strings[csDevZero].str_len + 1);
			#if defined(RS_ENCRYPT_STR)
				simple_str_encrypt(file_path, check_strings[csDevZero].str_len, csDevZero);
			#endif
				/*strcpy(file_path, "/dev/zero");*/
				zero_filp = local_filp_open(file_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

				if (IS_ERR(zero_filp)) {
					ret = PTR_ERR(zero_filp);
					goto out;
				}
				zero_fops = (struct file_operations *)zero_filp->f_op;
				filp_close(zero_filp, RS_FILP_CLOSE_ID);

				if (!zero_fops) {
					ret = -ENXIO;
					goto out;
				}

				if ((uintptr_t)zero_fops > (uintptr_t)null_fops) {
					/*increment*/
					/*fops = --null_fops;*/
					fops_dir = -1;
				} else {
					/*fops = ++null_fops;*/
					fops_dir = 1;
				}

				if (fops_direction_ptr)
					*fops_direction_ptr = fops_dir;
			}

			fops = null_fops + (fops_dir * fops_step);

			if ((fops->open) && (fops->mmap)) {
				goto do_patch;
			} else {
				ret = -EINVAL;
				goto out;
			}
		} else {
			goto out;
		}
	}

	fops = (struct file_operations *)filp->f_op;
	if (fops/* && (fops->write)*/) {
		char **addr;

		filp_close(filp, RS_FILP_CLOSE_ID);

	do_patch:
		addr = (char **)(&fops->open);

	#if defined(RS_USE_PROBE_WRITE)
		#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)
		*fops_open_store = (void *)((RS_SYM_PTR_TYPE)(fops->open) - RS_MEM_FUNC_BASE);
		ret = probe_kernel_write((void *)&(fops->open), (char *)new_open, sizeof(fops->open));
		#else
		ret = probe_kernel_write((void *)&(fops->open), (char *)rs_dummy_open, sizeof(fops->open));
		#endif
		if (!ret)
			RS_FLUSH_DCACHE_RANGE((unsigned long)(&fops->open), (unsigned long)(&fops->open) + sizeof(fops->open));
	#else
		{
			struct page_protect_struct protect_data;
			unsigned long align_addr = ((unsigned long)addr & PAGE_MASK);

			init_page_protect_data(&protect_data, sizeof(void *));

			ret = set_kernel_text_page_rw((unsigned long)align_addr);
			if (ret) {
				/*fail to set as rw*/
				goto out;
			}

			ret = set_page_rw((unsigned long)align_addr, &protect_data);
			if (ret) {
				/*fail to set as rw*/
				goto out;
			}

		#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)
			*fops_open_store = (void *)((RS_SYM_PTR_TYPE)(fops->open) - RS_MEM_FUNC_BASE);
			/*fops->open = new_open;*/
			{
				char *tmp = (char *)new_open;
				ret = copy_to_ro_page(&fops->open, &tmp, sizeof(tmp), &protect_data);
				if (ret)
					RS_LOG("rmdwhi,2,%d\n", ret);
				else
					RS_FLUSH_DCACHE_RANGE((unsigned long)(&fops->open), (unsigned long)(&fops->open) + sizeof(tmp));
			}
		#else
			fops->open = rs_dummy_open;
			if (align_addr == ((unsigned long)&fops->read & PAGE_MASK)) {
				/*fops->read = rs_dummy_read;*/
				char *tmp = (char *)rs_dummy_read;
				ret = copy_to_ro_page(&fops->read, &tmp, sizeof(tmp), &protect_data);
				if (ret)
					RS_LOG("rmdwhi,3,%d\n", ret);
				else
					RS_FLUSH_DCACHE_RANGE((unsigned long)(&fops->read), (unsigned long)(&fops->read) + sizeof(tmp));
			}
			if (align_addr == ((unsigned long)&fops->write & PAGE_MASK)) {
				/*fops->write = rs_dummy_write;*/
				char *tmp = (char *)rs_dummy_write;
				ret = copy_to_ro_page(&fops->write, &tmp, sizeof(tmp), &protect_data);
				if (ret)
					RS_LOG("rmdwhi,4,%d\n", ret);
				else
					RS_FLUSH_DCACHE_RANGE((unsigned long)(&fops->write), (unsigned long)(&fops->write) + sizeof(tmp));
			}
		#endif

			ret = set_page_ro((unsigned long)align_addr, &protect_data);

			if (!ret) {
				ret = set_kernel_text_page_ro((unsigned long)align_addr);
			}
		}
	#endif
	} else {
		filp_close(filp, RS_FILP_CLOSE_ID);
		ret = 0;
	}

out:
	if (file_path != local_buffer) {
		RS_FREE_BUFFER(file_path);
	}

out_ret:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_mem_dev_write_handler RS_HIDE(_m1)
#endif
noused notrace static int __init rs_reset_mem_dev_write_handler(void)
{
	/*重置/dev/kmem对应kmem_fops(drivers/char/mem.c)中的write等函数指针*/
	int fops_dir = 0;

#ifdef CONFIG_DEVKMEM
	rs_reset_mem_dev_write_handler_internal(check_strings[csDevKmem].str, check_strings[csDevKmem].str_len,
	#if defined(RS_ENCRYPT_STR)
		csDevKmem,
	#endif
		&fops_dir, 1
	#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)
		, &g_kmem_real_open, rs_kmem_open
	#endif
		);
#endif

#ifdef CONFIG_DEVMEM
	rs_reset_mem_dev_write_handler_internal(check_strings[csDevMem].str, check_strings[csDevMem].str_len,
	#if defined(RS_ENCRYPT_STR)
		csDevMem,
	#endif
		&fops_dir, 2
	#if defined(CONFIG_RS_MEM_DEV_ALLOW_SPECIAL)
		, &g_mem_real_open, rs_mem_open
	#endif
		);
#endif

	return 0;
}

late_initcall_sync(rs_reset_mem_dev_write_handler);

#endif

#if defined(CONFIG_RS_KALLSYMS_READ_RESTRICT)

#if defined(CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL)

#define RS_KS_FUNC_BASE (RS_SYM_VAL_TYPE)(PAGE_OFFSET + LINUX_VERSION_CODE + GCC_VERSION_CODE + 0x4B73)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define kallsyms_allowed_strings RS_HIDE(_m1_1)
#endif

#if !defined(RS_ENCRYPT_STR)
noused const static check_string kallsyms_allowed_strings[] = {
#if defined(RS_VIVOROOT_SUPPORT) /*android 5.0*/
	{
		"/system/bin/debuggerd",
		sizeof("/system/bin/debuggerd") - 1
	},
#else
	{
		NULL,
		0
	},
#endif
#if !defined(RS_VIVOROOT_SUPPORT) /*android 4.x*/
	{
		"/system/bin/aee_aed",
		sizeof("/system/bin/aee_aed") - 1
	},
#endif
};

#else

noused const static check_string kallsyms_allowed_strings[] = {
#if defined(RS_VIVOROOT_SUPPORT) /*android 5.0*/
	{
		"\x91\xCE\xC5\xC8\xCE\xDC\xD5\x98\xD4\xDC\xDA\x9C\xD6\xD4\xD2\xDA\xC9\xCA\xC9\xD9\xCE",
		sizeof("/system/bin/debuggerd") - 1
	},
#else
	{
		NULL,
		0
	},
#endif
#if !defined(RS_VIVOROOT_SUPPORT) /*android 4.x*/
	{
		"\x92\xCF\xC2\xC9\xCD\xDD\xDA\x99\xD7\xDD\xDD\x9D\xD0\xD5\xCA\xF1\xCC\xC9\xCF",
		sizeof("/system/bin/aee_aed") - 1
	},
#endif
};

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_kallsyms_real_open RS_HIDE(_m1_2)
#endif
noused static void *g_kallsyms_real_open;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kallsyms_open RS_HIDE(_m1_3)
#endif
noused notrace static int rs_kallsyms_open(struct inode *inode, struct file *file)
{
	int ret;
	char *buffer, *path;
	size_t i;
	fops_open real_open;
	struct task_struct *curr;

	if (!g_kallsyms_real_open) {
		ret = -EFAULT;
		goto out;
	}

	buffer = (char *)RS_GET_BUFFER();
	if (!buffer) {
		ret = -ENOMEM;
		goto out;
	}

	curr = current;

	path = get_absolute_path_with_buffer(curr, buffer, RS_BUFFER_SIZE);
	if (!path) {
		ret = -EINVAL;
		goto out_with_buffer;
	}

	ret = -EACCES;

	real_open = (fops_open)((RS_SYM_PTR_TYPE)g_kallsyms_real_open + RS_KS_FUNC_BASE);

	for (i = 0; i < ARRAY_SIZE(kallsyms_allowed_strings); i++) {
		if ((kallsyms_allowed_strings[i].str)
			&& (!STRCMP(path, kallsyms_allowed_strings[i].str, i))) {
			ret = 0; /*(*real_open)(inode, file);*/
			break;
		}
	}

#if 1
	RS_FREE_BUFFER(buffer);
	buffer = NULL;

	if (ret) {
		if (!CHECK_ROOT_UID(curr)) {
			ret = -EPERM;
			goto out;
		}

		ret = do_op_verify(ovAccessDev, 0);
		if (ret < 0) {
		} else if (rs_verify_func(RS_FC_PARAMS(do_op_verify), fcDoOpVerify)) {
			ret = -EPERM;
		} else {
			ret = 0;
		}
	}

	if (!ret) {
		real_open = (fops_open)((RS_SYM_PTR_TYPE)g_kallsyms_real_open + RS_KS_FUNC_BASE);

		ret = (*real_open)(inode, file);
	}

	/*goto out;*/
#endif

out_with_buffer:
	if (buffer) {
		RS_FREE_BUFFER(buffer);
	}

out:
	return ret; /*return seq_open(file, &rs_kallsyms_op);*/
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_kallsyms_open_handler RS_HIDE(_m2)
#endif
noused notrace static int __init rs_reset_kallsyms_open_handler(void)
{
	/*重置/proc/kallsyms对应kallsyms_operations(kernel/kallsyms.c)中的open函数指针*/
	int ret;
	struct file *filp;
	struct file_operations *fops;
	char *file_path;
	char local_buffer[16];

#if 0
	if (sizeof(file_path) < sizeof("/proc/kallsyms")/*(check_strings[csDevKmem].str_len + 1)*/) {
		/*buffer overrun*/
		ret = -EINVAL; /*-ENOBUFS;*/
		goto out_ret;
	}
#endif

	if ((sizeof("/proc/kallsyms") - 1) >= sizeof(local_buffer)) {
		file_path = (char *)RS_GET_BUFFER();
		if (!file_path) {
			ret = -ENOMEM;
			goto out_ret;
		}
	} else {
		file_path = local_buffer;
	}

	/*memcpy(file_path, "/proc/kallsyms", sizeof("/proc/kallsyms"));*/
	memcpy(file_path, check_strings[csProcKallsyms].str, check_strings[csProcKallsyms].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(file_path, check_strings[csProcKallsyms].str_len, csProcKallsyms);
#endif

	filp = local_filp_open(file_path, RS_READ_DIR_OPEN_FLAGS, RS_READ_DIR_OPEN_MODE);

	if (IS_ERR(filp)) {
		ret = PTR_ERR(filp);
		/*filp = NULL;*/
		goto out;
	}

	fops = (struct file_operations *)filp->f_op;
	if ((fops) && (fops->open)) {
	#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
		struct proc_dir_entry *pde = PDE(filp->f_path.dentry->d_inode); /*refer to proc_reg_read()*/
	#else
		struct proc_dir_entry *pde = PDE(filp->f_inode); /*refer to proc_reg_open()*/
	#endif

		if (pde) {
			fops = (struct file_operations *)pde->proc_fops;
		} else {
			fops = NULL;
		}

		filp_close(filp, RS_FILP_CLOSE_ID);

		if (fops) {
			char **addr = (char **)(&fops->open);

		#if defined(RS_USE_PROBE_WRITE)
			#if defined(CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL)
			g_kallsyms_real_open = (void *)((RS_SYM_PTR_TYPE)(fops->open) - RS_KS_FUNC_BASE);
			ret = probe_kernel_write((void *)addr, rs_kallsyms_open, sizeof(fops->open));
			#else
			ret = probe_kernel_write((void *)addr, rs_dummy_open, sizeof(fops->open));
			#endif
			if (!ret)
				RS_FLUSH_DCACHE_RANGE((unsigned long)addr, (unsigned long)addr + sizeof(fops->open));
		#else
			{
				struct page_protect_struct protect_data;
				unsigned long align_addr = ((unsigned long)addr & PAGE_MASK);

				init_page_protect_data(&protect_data, sizeof(void *));

				ret = set_kernel_text_page_rw((unsigned long)align_addr);
				if (ret) {
					/*fail to set as rw*/
					goto out;
				}

				ret = set_page_rw((unsigned long)align_addr, &protect_data);
				if (ret) {
					/*fail to set as rw*/
					goto out;
				}

			#if defined(CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL)
				/*g_kallsyms_real_open = fops->open;*/
				g_kallsyms_real_open = (void *)((RS_SYM_PTR_TYPE)(fops->open) - RS_KS_FUNC_BASE);
				/**addr = (char *)rs_kallsyms_open;*/
				{
					char *tmp = (char *)rs_kallsyms_open;
					ret = copy_to_ro_page(addr, &tmp, sizeof(tmp), &protect_data);
					if (ret)
						RS_LOG("rkohi,2,%d\n", ret);
					else
						RS_FLUSH_DCACHE_RANGE((unsigned long)addr, (unsigned long)addr + sizeof(tmp));
				}
			#else
				/**addr = (char *)rs_dummy_open;*/
				{
					char *tmp = (char *)rs_dummy_open;
					ret = copy_to_ro_page(addr, &tmp, sizeof(tmp), &protect_data);
					if (ret)
						RS_LOG("rkohi,3,%d\n", ret);
					else
						RS_FLUSH_DCACHE_RANGE((unsigned long)addr, (unsigned long)addr + sizeof(tmp));
				}
			#endif

				ret = set_page_ro((unsigned long)align_addr, &protect_data);

				if (!ret) {
					ret = set_kernel_text_page_ro((unsigned long)align_addr);
				}
			}
		#endif
		} else {
			ret = -EINVAL;
		}
	} else {
		filp_close(filp, RS_FILP_CLOSE_ID);
		ret = 0;
	}

out:
	if (file_path != local_buffer) {
		RS_FREE_BUFFER(file_path);
	}

out_ret:
	return ret;
}

late_initcall_sync(rs_reset_kallsyms_open_handler);

#endif

/*///////////////////////////////////////////////////////*/
#if defined(SOCK_CHECK_FASTER_BY_GLOBAL_VAR)

#define SOCK_SET_ALLOW_BIND() { rs_set_fast_flag(fgfAllowSockBind, 1); }
#define SOCK_SET_DISALLOW_BIND() { rs_set_fast_flag(fgfAllowSockBind, -1); }

#define SOCK_SET_ALLOW_CONNECT() { rs_set_fast_flag(fgfAllowSockConnect, 1); }
#define SOCK_SET_DISALLOW_CONNECT() { rs_set_fast_flag(fgfAllowSockConnect, -1); }

#else

#define SOCK_SET_ALLOW_BIND() do {} while (0)
#define SOCK_SET_DISALLOW_BIND() do {} while (0)

#define SOCK_SET_ALLOW_CONNECT() do {} while (0)
#define SOCK_SET_DISALLOW_CONNECT() do {} while (0)

#endif

#if defined(RS_MTK_PLATFORM)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_bypass_sock_addrs RS_HIDE(_n1_3)
#endif
noused static const check_string g_bypass_sock_addrs[] = {
#if defined(RS_ENCRYPT_STR)
	{
		"\xDD\xD2\xD1\x95\xD7\xCD\xD3\x99\xD7\xD0\xD1\x9D\xD3\xD4\xD4",
		sizeof("com.mtk.aee.aed") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"\xDE\xD3\xD6\x94\xD4\xCC\xDC\x98\xD4\xD1\xD6\x9C\xD0\xD5\xCB\xF1\x9B\x98",
		sizeof("com.mtk.aee.aed_64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif
#else
	{
		"com.mtk.aee.aed",
		sizeof("com.mtk.aee.aed") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"com.mtk.aee.aed_64",
		sizeof("com.mtk.aee.aed_64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif
#endif
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_bypass_sock_addr_procs RS_HIDE(_n1_4)
#endif
noused static const check_string g_bypass_sock_addr_procs[] = {
#if defined(RS_ENCRYPT_STR)
#if defined(RS_IS_ANDROID_8_1_ABOVE)
	{
		"\xDF\xD8\xD9\xE4\xDB\xDC\xDC",
		sizeof("aee_aed") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"\xDC\xD9\xDE\xE5\xD8\xDD\xD3\x80\x81",
		sizeof("aee_aed64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif
	{
		"\xDD\xDE\xDF\xE6\xD9\xD2\xD2\xC3",
		sizeof("aee_aedv") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"\xDA\xDF\xDC\xE7\xD6\xD3\xD1\xC2\x85\x86",
		sizeof("aee_aedv64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif

#else /* RS_IS_ANDROID_8_1_ABOVE */
	{
		"\xDA\xD8\xDE\xCE\xDD\xDE\xDD\xC5\xD2",
		sizeof("debuggerd") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"\xD9\xD9\xD9\xCF\xDE\xDF\xD2\xC4\xD1\x82\x87",
		sizeof("debuggerd64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif
#endif /* RS_IS_ANDROID_8_1_ABOVE */

#else /* RS_ENCRYPT_STR */

#if defined(RS_IS_ANDROID_8_1_ABOVE)
	{
		"aee_aed",
		sizeof("aee_aed") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"aee_aed64",
		sizeof("aee_aed64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif
	{
		"aee_aedv",
		sizeof("aee_aedv") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"aee_aed6v4",
		sizeof("aee_aedv64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif

#else /* RS_IS_ANDROID_8_1_ABOVE */
	{
		"debuggerd",
		sizeof("debuggerd") - 1
	},
#if defined(RS_64BIT_SUPPORT)
	{
		"debuggerd64",
		sizeof("debuggerd64") - 1
	},
#else
	{ NULL_CS_ITEM },
#endif /* RS_64BIT_SUPPORT */
#endif /* !RS_IS_ANDROID_8_1_ABOVE */
#endif /* !RS_ENCRYPT_STR */
};

noused notrace static int is_sock_addr_bypass(const char *addr, int is_bind)
{
	int ret = -1;
	const char *curr_proc;
	int i;
	if (is_bind) {
		curr_proc = current->comm;
	}

	for (i = 0; i < ARRAY_SIZE(g_bypass_sock_addrs); i++) {
		int res;
		const char *str = g_bypass_sock_addrs[i].str;
		int len = g_bypass_sock_addrs[i].str_len;

		if ((!str) || (!len))
			continue;

		res = STRNCMP(addr, str, len, i);
		/*res = strncmp(addr, str, len);*/

		if (res == 0) {
			if (addr[len] == '\0') {
				if (is_bind) {
					const char *proc_str = g_bypass_sock_addr_procs[i].str;
					len = g_bypass_sock_addr_procs[i].str_len;
					if ((proc_str) && (len) && (STRCMP(curr_proc, proc_str, i) == 0)) {
						ret = i;
					}
				} else {
					ret = i;
				}

				break;
			}
		} else if (res < 0) { /*g_bypass_sock_addrs[]是按 name 从小到大顺序排的*/
			break;
		}
	}

	return ret;
}
#else
	#define is_sock_addr_bypass(addr, is_bind) (-1)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_sock_bind_check RS_HIDE(_n1)
#endif
noused int do_sock_bind_check(struct socket *sock, struct sockaddr_storage *address, int addrlen)
{
#if defined(CONFIG_SOCK_RESTRICT)
	int ret;
	noused struct task_struct *curr;

#if defined(SOCK_CHECK_FASTER_BY_GLOBAL_VAR)
	noused intptr_t flag;

	if (rs_get_fast_flag(fgfAllowSockBind, &flag)) {
		/*RS_LOG("bsc,-1\n");*/
		return 1;
	}

	if (flag > 0) {
		/*RS_LOG("bsc,0\n");*/
		return 0;
	} else if (flag < 0) {
		/*RS_LOG("bsc,1\n");*/
		return 1;
	}

#if 0
	if (g_allow_sock_bind) {
		/*RS_LOG("bsc,0,%d\n", g_allow_sock_bind);*/
		return ((g_allow_sock_bind > 0) ? 0 : 1);
	}
#endif
#endif

#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
	SOCK_SET_ALLOW_BIND();
	ret = 0;
	goto out;
#else  /*RS_IS_ENG_BUILD*/

	ret = -EPERM;

	curr = current;

#if 1
	if (/*(!file) ||*/ /* (!curr->mm) ||*/ (is_init_task(curr)) || ((curr->flags & PF_KTHREAD) && (!curr->mm)
										  && (!task_in_call_from_user(curr)))) {
		RS_LOG("bsc,1\n");
		ret = 0;
		goto out;
	}
#endif

	if (CHECK_ROOT_UID(curr)) {
		int flag = 0;
		struct sock *sk;
		const struct proto_ops *ops;

		/*RS_LOG("bsc,3\n");*/
		if ((addrlen > (int)sizeof(address->ss_family))
			&& (sock->type == SOCK_STREAM)) {
			if (address->ss_family == PF_UNIX/*AF_LOCAL*/)
				flag = 1;
			else {
				sk = sock->sk;
				if ((sk) && (sk->sk_family == PF_UNIX))
					flag = 1;
				else {
					ops = sock->ops;
					if ((ops) && (ops->family == PF_UNIX))
						flag = 1;
				}
			}
		}

		if (flag) {
			char *p;

			/*RS_LOG("bsc,4\n");*/
			if (addrlen < sizeof(*address)) {
				p = (char *)address;
				p[addrlen] = '\0';
			} else {
				address->__data[sizeof(address->__data) - 1] = '\0';
			}

			p = address->__data;

			if (!p[0]) {
				/*abstract socket*/
				p++;
				if (!p[0]) {
					ret = 0;
					goto out;
				}
			}

			/*if (*p)*/
			{
				/*RS_LOG("bind2:%s->%s\n", current->comm, p);*/

				if (((_strchr(p, '.'))
					|| (STRISTR(p, check_strings[csSu].str, csSu))
					|| (STRISTR(p, check_strings[csRoot].str, csRoot))
					) && (is_sock_addr_bypass(p, 1) < 0)) {
					/*ret = -EPERM;*/

					if (is_op_permitted_no_fc(ovSockCheck)) {
					#if 0
						if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
							SOCK_SET_ALLOW_BIND();
							ret = 0;
							goto out;
						} else {
							SOCK_SET_DISALLOW_BIND();
							ret = -EPERM;
							goto out_checked;
						}
					#else
						RS_LOG("bsc,5\n");
						SOCK_SET_ALLOW_BIND();
						ret = 0;
						goto out;
					#endif
					}

					/*RS_LOG("bsc,6,%s\n", curr->comm);*/

				#if defined(SOCK_CHECK_ALLOW_NON_BUILT_IN_PROCESS)
					if (do_op_verify(ovSockCheck, 0) < 0) {
						/*allow non built-in process do binding*/
						RS_LOG("bsc,7\n");
						ret = 0;
					} else
				#endif
					{
						RS_LOG("Restricted sock bind check. PID = %d(%s) "
							"PPID = %d(%s) %s\n",
							curr->pid, curr->comm,
							curr->parent->pid, curr->parent->comm, p);

					#if defined(PASS_SOCK_RESTRICT)
						SOCK_SET_ALLOW_BIND();
						ret = 0;
					#else
						#if 1
						if ((is_op_bypassed_no_fc(ovSockCheck))
							/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
							) {
							SOCK_SET_ALLOW_BIND();
							ret = 0;
						} else {
							SOCK_SET_DISALLOW_BIND();
							/*ret = -EACCES;*/
						}
						#endif
					#endif
					}
				} else {
					ret = 0;
				}
			}
		} else {
			ret = 0;
		}
	} else {
		ret = 0;
	}
#endif /*!RS_IS_ENG_BUILD*/

out:
/*out_checked:*/

	return ret;
#else /*CONFIG_SOCK_RESTRICT*/
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_sock_connect_check RS_HIDE(_n2)
#endif
noused int do_sock_connect_check(struct socket *sock, struct sockaddr_storage *address, int addrlen)
{
#if defined(CONFIG_SOCK_RESTRICT)
	int ret;
	noused struct task_struct *curr;
	noused const struct cred *cred;

#if defined(SOCK_CHECK_FASTER_BY_GLOBAL_VAR)
	noused intptr_t flag;

	if (rs_get_fast_flag(fgfAllowSockConnect, &flag)) {
		/*RS_LOG("scc,-1\n");*/
		return 1;
	}

	if (flag > 0) {
		/*RS_LOG("scc,0\n");*/
		return 0;
	} else if (flag < 0) {
		/*RS_LOG("scc,1\n");*/
		return 1;
	}

#if 0
	if (g_allow_sock_connect) {
		/*RS_LOG("bsc,0,%d\n", g_allow_sock_connect);*/
		return ((g_allow_sock_connect > 0) ? 0 : 1);
	}
#endif
#endif

#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
	SOCK_SET_ALLOW_CONNECT();
	ret = 0;
	goto out;
#else  /*RS_IS_ENG_BUILD*/

	ret = -EPERM;

	curr = current;

#if 1
	if (/*(!file) ||*/ /* (!curr->mm) || (is_init_task(curr)) ||*/ ((curr->flags & PF_KTHREAD) && (!curr->mm)
			&& (!task_in_call_from_user(curr)))) {
		RS_LOG("scc,1\n");
		ret = 0;
		goto out;
	}
#endif

	cred = curr->cred;
	if ((cred) && (__kuid_val((cred)->uid) >= 10000/*AID_APP*/)) {
		/*RS_LOG("scc,3\n");*/
		if ((addrlen > (int)sizeof(address->ss_family))
			&& (sock->type == SOCK_STREAM)
			&& ((address->ss_family == PF_UNIX/*AF_LOCAL*/) || ((sock->ops) && (sock->ops->family == PF_UNIX)))) {
			char *p;

			/*RS_LOG("scc,4\n");*/
			if (addrlen < sizeof(*address)) {
				p = (char *)address;
				p[addrlen] = '\0';
			} else {
				address->__data[sizeof(address->__data) - 1] = '\0';
			}

			p = address->__data;

			if (!p[0]) {
				/*abstract socket*/
				p++;
				if (!p[0]) {
					ret = 0;
					goto out;
				}
			}

			/*if (*p)*/
			{
				/*RS_LOG("connt2:%s->%s\n", current->comm, p);*/

				if (((_strchr(p, '.'))
					|| (STRISTR(p, check_strings[csSu].str, csSu))
					|| (STRISTR(p, check_strings[csRoot].str, csRoot))
					) && (is_sock_addr_bypass(p, 0) < 0)) {
					/*ret = -EPERM;*/

					if (is_op_permitted_no_fc(ovSockCheck)) {
					#if 0
						if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
							SOCK_SET_ALLOW_CONNECT();
							ret = 0;
							goto out;
						} else {
							SOCK_SET_DISALLOW_CONNECT();
							ret = -EPERM;
							goto out_checked;
						}
					#else
						RS_LOG("scc,5\n");
						SOCK_SET_ALLOW_CONNECT();
						ret = 0;
						goto out;
					#endif
					}

					/*RS_LOG("scc,6,%s\n", curr->comm);*/

				#if defined(SOCK_CHECK_ALLOW_NON_BUILT_IN_PROCESS)
					if (do_op_verify(ovSockCheck, 0) < 0) {
						/*allow non built-in process do binding*/
						RS_LOG("scc,7\n");
						ret = 0;
					} else
				#endif
					{
						RS_LOG("Restricted sock connect check. PID = %d(%s) "
							"PPID = %d(%s) %s\n",
							curr->pid, curr->comm,
							curr->parent->pid, curr->parent->comm, p);

					#if defined(PASS_SOCK_RESTRICT)
						SOCK_SET_ALLOW_CONNECT();
						ret = 0;
					#else
						#if 1
						if ((is_op_bypassed_no_fc(ovSockCheck))
							/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
							) {
							SOCK_SET_ALLOW_CONNECT();
							ret = 0;
						} else {
							SOCK_SET_DISALLOW_CONNECT();
							/*ret = -EACCES;*/
						}
						#endif
					#endif
					}
				} else {
					ret = 0;
				}
			}
		} else {
			ret = 0;
		}
	} else {
		ret = 0;
	}
#endif /*!RS_IS_ENG_BUILD*/

out:
/*out_checked:*/

	return ret;
#else /*CONFIG_SOCK_RESTRICT*/
	return 0;
#endif
}

/*///////////////////////////////////////////////////////*/

enum {
	prnsMountInfo,
	prnsIsMountFlagEnable,
	prnsMountOperationHappened,

	/*add new items here*/
	prnsInvalid,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define printk_strings RS_HIDE(_Cg)
#endif

#if !defined(RS_ENCRYPT_STR)

noused const static check_string printk_strings[prnsInvalid] = {
	{
		"mnt:%s,%s,%s,%lx",
		sizeof("mnt:%s,%s,%s,%lx") - 1
	},
	{
		"mnt:%d",
		sizeof("mnt:%d") - 1
	},
	{
		"mnt:%d:%s",
		sizeof("mnt:%d:%s") - 1
	},
};

#else

noused const static check_string printk_strings[prnsInvalid] = {
	{
		"\xD3\xD3\xC8\x81\x9F\xCA\x94\x92\xC5\x99\x91\xC0\x9E\x94\xDC\xD7",
		sizeof("mnt:%s,%s,%s,%lx") - 1
	},
	{
		"\xD0\xD2\xCF\x80\x9C\xDC",
		sizeof("mnt:%d") - 1
	},
	{
		"\xD1\xD5\xCE\x83\x9D\xD3\x9A\x90\xC7",
		sizeof("mnt:%d,%s") - 1
	},
};

#endif

#if (RS_DEBUG)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_printk_idx RS_HIDE(_Cs)
#endif
noused notrace
static noinline int rs_printk_idx(const char *level, size_t fmt_str_idx, ...)
{
	int ret;

	if (fmt_str_idx < prnsInvalid) {
		int level_len;
		char *fmt;
		va_list ap;

		level_len = ((level) ? strlen(level) : 0);

		fmt = (char *)rs_kmalloc(level_len + printk_strings[fmt_str_idx].str_len + 4);
		if (!fmt) {
			ret = -ENOMEM;
			goto out;
		}

		if (level_len) {
			memcpy(fmt, level, level_len);
		}

		memcpy(&fmt[level_len], printk_strings[fmt_str_idx].str, printk_strings[fmt_str_idx].str_len + 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&fmt[level_len], printk_strings[fmt_str_idx].str_len, fmt_str_idx);
	#endif

		level_len += printk_strings[fmt_str_idx].str_len;
		if (fmt[level_len - 1] != '\n') {
			fmt[level_len++] = '\n';
			fmt[level_len] = '\0';
		}

		va_start(ap, fmt_str_idx);
		ret = vprintk(fmt, ap);
		va_end(ap);

		rs_kfree(fmt);
	} else {
		ret = -EINVAL;
	}

out:
	return ret;
}

#define rs_printk	printk

#else
	#define rs_printk_idx(...)	do {} while (0)
	#define rs_printk(...)		do {} while (0)
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_do_mount_post_process RS_HIDE(_Cs1)
#endif

noused notrace
static noinline void rs_do_mount_post_process(const char *dev_name,
	const char *dir_name, unsigned long flags, struct tag_mount_check_info *check_info,
	bool dir_name_from_user)
{
	if ((flags) && (dev_name) && (dir_name)) {
	#define RDMPP_FLAG_IS_SYSTEM_CHECKED (0x1)
	#define RDMPP_FLAG_SYSTEM_CHECK_NOT_NEEDED (0x2)
	#define RDMPP_FLAG_NEED_RECORD_MOUNT (0x4)
		char local_dir_name[sizeof("/system/") + 1];
		bool dir_name_copied = false;
		int rdmpp_flags = 0;

		if (!(flags & MS_RDONLY)) {
		#if !defined(RS_MTK_PLATFORM)
			if ((isMountRecord) && (isMountFlagEnabled))
		#else
			if (isMountFlagEnabled)
		#endif
			{
				if (rs_get_boot_mode() == RS_NORMAL_BOOT) {
				#if defined(RS_NEED_DO_MOUNT_CHECK)
					if ((check_info) && (!check_info->ignore_mount_record))
				#endif
					{
						/*RS_LOG("rdmpp,1,%d,%d\n", check_info->flags.op_data.op_type, check_info->flags.op_data.op_info);*/
						rdmpp_flags |= RDMPP_FLAG_IS_SYSTEM_CHECKED;

						if (dir_name_from_user) {
							unsigned long remain;

							remain = copy_from_user(local_dir_name, dir_name, (sizeof(local_dir_name) - 1));
							if (remain >= (sizeof(local_dir_name) - 1))
								return;

							local_dir_name[sizeof(local_dir_name) - 1 - remain] = '\0';
							/*local_dir_name[sizeof(local_dir_name) - 1] = '\0';*/

							dir_name = (const char *)local_dir_name;
							dir_name_copied = true;
						}

					#if defined(RS_NEED_DO_MOUNT_CHECK)
						if ((check_info->flags.op_data.op_type == ovMount)
							&& (check_info->flags.op_data.op_info == rmtSystem)) {
							rdmpp_flags |= RDMPP_FLAG_NEED_RECORD_MOUNT;
							/*RS_LOG("rdmpp,2\n");*/
						} else
					#endif
					#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
						if ((dir_name[0] == '/') && (dir_name[1] == '\0')
							/* dev: /dev/root or /dev/block/xxx... */
							&& (STRNCMP(dev_name, check_strings[csDevSlash].str, check_strings[csDevSlash].str_len, csDevSlash) == 0)
							) {
							/*RS_LOG("rdmpp,3\n");*/
							rdmpp_flags |= RDMPP_FLAG_NEED_RECORD_MOUNT;
						} else
					#endif
						if ((STRNCMP(dir_name, check_strings[csSystem].str, check_strings[csSystem].str_len, csSystem) == 0)
							&& (dir_name[sizeof("/system") - 1] == '\0' || (dir_name[sizeof("/system") - 1] == '/'
						#if defined(RS_MOUNT_DIR_CHECK_TOP_LEVEL_ONLY)
							&& dir_name[sizeof("/system")] == '\0'
						#endif
							))) {
							/*RS_LOG("rdmpp,4\n");*/
							rdmpp_flags |= RDMPP_FLAG_NEED_RECORD_MOUNT;
						}
					}
				} else {
					/*RS_LOG("rdmpp,5\n");*/
					rdmpp_flags |= RDMPP_FLAG_SYSTEM_CHECK_NOT_NEEDED;
				}
			}

			if ((rdmpp_flags & RDMPP_FLAG_NEED_RECORD_MOUNT)
			#if defined(RS_HAS_SUPER_PARTITION)
				/*
				 init will mount "/dev,/system/dev,(null),(null),MS_MOVE",
				 "/proc,/system/proc,(null),(null),MS_MOVE",... etc.
				 */
				&& (!((flags & MS_MOVE) && is_init_task(current)))
			#endif
				) {
				noused struct task_struct *curr = current;
				/*printk(KERN_ERR "mount operation happened !!");*/
				rs_printk_idx(KERN_ERR, prnsMountOperationHappened, curr->pid, curr->comm);
				/*printk(KERN_INFO "The process is \"%s\" (pid %i) (tgid %i)\n", curr->comm, curr->pid, curr->tgid);*/
				/*printk(KERN_ERR "mnt:%s,%s,%lx,%s\n", dev_name, dir_name, flags, curr->comm);*/
			#if defined(RS_NEED_DO_MOUNT_CHECK)
				if (check_info) {
					rs_journal((char *)csMnt/*"mnt"*/, check_info->task_infos, check_info->flags.value, check_info->task_infos_len);
				}
			#endif

				mountFlag = 1; /*LiZhao*/
			}
		}

		if (!rs_get_system_mounted()) {
			/*RS_LOG("rdmpp,6\n");*/
			if (!(rdmpp_flags & RDMPP_FLAG_SYSTEM_CHECK_NOT_NEEDED)) {
				/*RS_LOG("rdmpp,7\n");*/
				if (!(rdmpp_flags & RDMPP_FLAG_IS_SYSTEM_CHECKED)) {
					/*RS_LOG("rdmpp,8\n");*/
					if ((dir_name_from_user) && (!dir_name_copied)) {
						unsigned long remain;

						remain = copy_from_user(local_dir_name, dir_name, (sizeof(local_dir_name) - 1));
						if (remain >= (sizeof(local_dir_name) - 1))
							return;

						local_dir_name[sizeof(local_dir_name) - 1 - remain] = '\0';

						dir_name = (const char *)local_dir_name;
					}

				#if defined(RS_SYSTEM_AS_ROOT_SUPPORT)
					/* dev: /dev/root, dir: /root */
					if ((STRCMP(dir_name, check_strings[csSplashRoot].str, csSplashRoot) == 0)
						&& (STRCMP(dev_name, check_strings[csDevRoot].str, csDevRoot) == 0)
						) {
						/*RS_LOG("rdmpp,9\n");*/
						rdmpp_flags |= RDMPP_FLAG_NEED_RECORD_MOUNT;
					} else
				#endif
					if ((STRNCMP(dir_name, check_strings[csSystem].str, check_strings[csSystem].str_len, csSystem) == 0)
						&& (dir_name[sizeof("/system") - 1] == '\0' || (dir_name[sizeof("/system") - 1] == '/'
					#if defined(RS_MOUNT_DIR_CHECK_TOP_LEVEL_ONLY)
						&& dir_name[sizeof("/system")] == '\0'
					#endif
						))) {
						/*RS_LOG("rdmpp,10\n");*/
						rdmpp_flags |= RDMPP_FLAG_NEED_RECORD_MOUNT;
					}
				}

				if (rdmpp_flags & RDMPP_FLAG_NEED_RECORD_MOUNT) {
					/*RS_LOG("rdmpp,11\n");*/
					rs_set_system_mounted();
				}
			}
		}

	#if defined(CONFIG_BLOCK_DEV_RS_CONFIG)
		if (!g_init_rs_config_from_blkdev_dev_ok) {
			if ((STRNCMP(dev_name, check_strings[csDevBlockSlash].str, \
				check_strings[csDevBlockSlash].str_len, csDevBlockSlash) == 0)
				|| (STRNCMP_EX(dev_name, check_strings[csDevBlockSlash].str, \
				1, check_strings[csDevBlockSlash].str_len - 1, csDevBlockSlash) == 0)
			#if defined(RS_HAS_EMMC_SHORTCUTS)
				|| (STRNCMP(dev_name, check_strings[csEmmc].str, check_strings[csEmmc].str_len, csEmmc) == 0)
				|| (STRNCMP_EX(dev_name, check_strings[csEmmc].str, 1, check_strings[csEmmc].str_len - 1, csEmmc) == 0)
			#endif
				)
			{
				if ((dir_name_from_user) && (!dir_name_copied)) {
					unsigned long remain;

					remain = copy_from_user(local_dir_name, dir_name, (sizeof(local_dir_name) - 1));
					if (remain >= (sizeof(local_dir_name) - 1))
						return;

					local_dir_name[sizeof(local_dir_name) - 1 - remain] = '\0';

					dir_name = (const char *)local_dir_name;
				}

				/*RS_LOG("rdmpp,12\n");*/

				if ((dir_name[0] == '/')
					&& (!STRNCMP_EX(&dir_name[1], check_strings[csDalvikCacheSlash].str, \
					(sizeof("dalvik-") - 1), (sizeof("cache") - 1), csDalvikCacheSlash))
					&& (dir_name[sizeof("/cache") - 1] == '\0'))
				{
					g_init_rs_config_from_blkdev_dev_ok = 1;

					/*RS_LOG("rdmpp,13\n");*/
					init_rs_config_from_block_dev();
				}
			}
		}
	#endif
	}
}

#define RS_DO_MOUNT_CHECK_BOOT (0x1)
#define RS_DO_MOUNT_CHECK_MOUNT (0x2)


/*///////////////////////////////////////*/
/*lizonglin transplant for is_root function start*/

noused static int strcllf(char *str)
{
	int len = strlen(str);
	int i = 0;
	int num = 0;
	for (i = 0; i < len; i++) {
		if (0x0a == *(str+i)) {
			*(str+i) = 0x0;
			num++;
		}
	}
	return num;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mount_write_proc RS_HIDE(_Cs1_1)
#endif
noused static int mount_write_proc(struct file *filp, const char *buffer, unsigned long count, void *data)
{
	#define PROCFS_MAX_SIZE 30

	char proc_buffer[PROCFS_MAX_SIZE] = {0};
	char *enable_mount_flag_code/*[PROCFS_MAX_SIZE]*/ = "_enable_mount_flag";
	char *disable_mount_flag_code/*[PROCFS_MAX_SIZE]*/ = "_disable_mount_flag";
#if defined(RS_QUALCOMM_PLATFORM)
	char *enable_mount_record/*[PROCFS_MAX_SIZE]*/ = "_enable_mount_record";
	char *disable_mount_record/*[PROCFS_MAX_SIZE]*/ = "_disable_mount_record";
#endif
	int proc_buffer_size = count;

	if (PROCFS_MAX_SIZE < proc_buffer_size) {
		proc_buffer_size = PROCFS_MAX_SIZE;
	}

	if (copy_from_user(proc_buffer, buffer, proc_buffer_size)) {
		rs_printk("mount_write_proc, copy_from_user error!.\n");
		return -EFAULT;
	}
	if (proc_buffer_size)
		proc_buffer[proc_buffer_size - 1] = '\0';
	else
		proc_buffer[0] = '\0';
	rs_printk("mount_write_proc, proc_buffer:%s\n", proc_buffer);

	/**buffer may ends with 0x0A (ex.use 'echo'), replace it with 0x0*/
	strcllf(proc_buffer);

	if (0 == strcmp(proc_buffer, disable_mount_flag_code)) {
		isMountFlagEnabled = 0;
		rs_printk("set isMountFlagEnabled=%d\n", isMountFlagEnabled);
	} else if (0 == strcmp(proc_buffer, enable_mount_flag_code)) {
		isMountFlagEnabled = 1;
		rs_printk("set isMountFlagEnabled=%d\n", isMountFlagEnabled);
	}
#if defined(RS_QUALCOMM_PLATFORM)
	else if (0 == strcmp(proc_buffer, enable_mount_record)) {
		isMountRecord = 1;
		rs_printk("set isMountRecord=%d\n", isMountRecord);
	} else if (0 == strcmp(proc_buffer, disable_mount_record)) {
		isMountRecord = 0;
		rs_printk("set isMountRecord=%d\n", isMountRecord);
	}
#endif
#if defined(CONFIG_BLOCK_DEV_RS_CONFIG)
	else if (0 == STRCMP(proc_buffer, check_strings[csRSCfgUpd].str, csRSCfgUpd)) {
		/*adb shell "echo _rs_cfg_upd > /proc/isroot/isroot"*/
		init_rs_config_from_block_dev();
	}
#endif

	return proc_buffer_size;

}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_root_proc_show RS_HIDE(_Cs1_2)
#endif
noused static int is_root_proc_show(struct seq_file *m, void *v)
{
#if defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_RS_CHECK_FUNC)
	if ((!g_bootup_checked) && (CHECK_ROOT_UID(current))) {
	#if defined(CONFIG_RS_CHECK_FUNC)
		if (!rs_get_bootup_set()) {
			if (is_property_service_started()) {
				g_bootup_checked = rs_get_set_value();
				RS_LOG("sb4\n");
				rs_try_set_bootup_done();
			}
		}
	#endif
	}
#endif

	seq_printf(m, "mount state:%u\n", mountFlag);

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_root_proc_open RS_HIDE(_Cs1_3)
#endif
noused static int is_root_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, is_root_proc_show, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_root_proc_write RS_HIDE(_Cs1_4)
#endif
noused static ssize_t is_root_proc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *off)
{
	return mount_write_proc(file, buf, count, NULL);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_root_proc_fops RS_HIDE(_Cs1_5)
#endif
noused static const struct file_operations is_root_proc_fops = {
	.open		= is_root_proc_open,
	.read		= seq_read,
	.write		= is_root_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_is_root_nodes RS_HIDE(_Cs1_6)
#endif
noused static int __init init_is_root_nodes(void)
{
/*lizonglin transplant for is_root function start*/
	/* added by dengqiang for root operation check 2012-06-01 */
#if defined(CONFIG_PROC_FS)
	is_root_dir = proc_mkdir("isroot", NULL);
	if (is_root_dir == NULL) {
		printk(KERN_ERR "can't create dir-isroot !!!");
		return 0;
	}
	/* Creating read/write "isroot" entry */
	/*begin modified by xiejiyuan*/
	is_root_ent = proc_create("isroot", 0644, is_root_dir, &is_root_proc_fops);
	/*is_root_ent = create_proc_entry("isroot", 0644, is_root_dir);*/
	if (is_root_ent == NULL) {
		printk(KERN_ERR "can't create entry-isroot !!!");
		return 0;
	}
	/*is_root_ent->read_proc = mount_read_proc;*/
	/*is_root_ent->write_proc = mount_write_proc;*/
	/*end modified by xiejiyuan*/
#endif
#if defined(CONFIG_DEBUG_FS)
	/* ard 9.x and above,system_server has no permission
	 to access /proc/isroot/isroot */
	debugfs_create_file("isroot", 0644, NULL, NULL,
			&is_root_proc_fops);
#endif
	/* add end */
/*lizonglin transplant for is_root function end*/
	return 0;
}
late_initcall(init_is_root_nodes);

/*lizonglin transplant for is_root function end*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define g_is_unlocked RS_HIDE(_Ct1)
#define g_is_tampered RS_HIDE(_Ct2)
#define g_is_sec_schip RS_HIDE(_Ct3)
#define g_is_sec_boot RS_HIDE(_Ct4)
#endif
noused static char g_is_unlocked;
noused static char g_is_tampered;
noused static char g_is_sec_schip;
noused static char g_is_sec_boot;


typedef int lk_bool;

#ifndef __PACKED
	#if __GNUC__
		#define __PACKED __attribute__((packed))
	#else
		#define __PACKED
	#endif
#endif



#if defined(RS_QUALCOMM_PLATFORM)

#if defined(CONFIG_MSM_SCM)

#include <soc/qcom/scm.h>

#define IS_SECURE_BOOT_ENABLED 0x4

#endif

/*noused static int secure_boot_enabled = 0;*/
/*noused static int wdog_debug_fuse_disabled = 0;*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define scm_check_boot_fuses RS_HIDE(_Ct8)
#endif
noused notrace static int scm_check_boot_fuses(void)
{
#if defined(CONFIG_MSM_SCM)
	int ret = 0;
	uint32_t resp;
#if defined(SCM_SIP_FNID)
	struct scm_desc desc = {0};

	desc.args[0] = 0;
	desc.arginfo = 0;
#endif

	RS_LOG("cbf,0\n");

#if defined(SCM_SIP_FNID)
	if (!is_scm_armv8())
#endif
	{
		RS_LOG("cbf,1\n");
		ret = scm_call(SCM_SVC_INFO/*TZBSP_SVC_INFO*/, IS_SECURE_BOOT_ENABLED, NULL, 0, &resp, sizeof(resp));
	}
#if defined(SCM_SIP_FNID)
	else {
		RS_LOG("cbf,2\n");
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_INFO, IS_SECURE_BOOT_ENABLED), &desc);
		resp = desc.ret[0];
	}
#endif

	/* Parse Bit 0 and Bit 2 of the response */
	if (!ret) {
		RS_LOG("cbf,3\n");
		/* Bit 0 - SECBOOT_ENABLE_CHECK */
		if (resp & 0x1) {
			RS_LOG("cbf,4\n");
			g_is_sec_boot = 1; /*false*/
		} else {
			g_is_sec_boot = 2; /*true*/
		}
		/* Bit 2 - DEBUG_DISABLE_CHECK */
		/*if(resp & 0x4)
			wdog_debug_fuse_disabled = false;
		*/
	} else {
		g_is_sec_boot = -1;
	}

	return ret;
#else
	g_is_sec_boot = 1;

	return 0;
#endif
}

#if defined(RS_CHEKC_VIVO_SECURE_BOOT_BY_GLOBAL_VAR)
__weak u32 sec_enable;
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_vivo_secure_boot_enable RS_HIDE(_Ct8_)
#endif
noused notrace static int check_vivo_secure_boot_enable(void)
{
	#define SEC_ENABLE_FLAG (0x00303030)

#if defined(RS_CHEKC_VIVO_SECURE_BOOT_BY_GLOBAL_VAR)
	extern u32 sec_enable; /* defined in fs/proc/sec.c*/

	if (sec_enable == SEC_ENABLE_FLAG) {
		RS_LOG("ivsb,1\n");
		g_is_sec_boot = 2; /*true*/
	} else {
		g_is_sec_boot = 1; /*false*/
	}

	return 0;
#else

	#define SEC_ENABLE_READ_BUFFER_SIZE (32)

	/* /proc/sec_enable */
	char *dev_name;
	struct file *filp;
	char *buffer;
	int ret;

	RS_LOG("ivsb,0\n");

	buffer = (char *)rs_kmalloc(SEC_ENABLE_READ_BUFFER_SIZE);
	if (!buffer) {
		RS_LOG("ivsb,1\n");
		ret = -ENOMEM;
		goto out_no_file;
	}

	if (RS_BUFFER_SIZE < (check_strings[csProcSecEnable].str_len + 1)) {
		/*buffer overrun*/
		RS_LOG("ivsb,2\n");
		ret = -EINVAL; /*-ENOBUFS;*/
		goto out_no_file;
	}

	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		RS_LOG("ivsb,3\n");
		ret = -ENOMEM;
		goto out_no_file;
	}

	memcpy(dev_name, check_strings[csProcSecEnable].str, check_strings[csProcSecEnable].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csProcSecEnable].str_len, csProcSecEnable);
#endif

	filp = safe_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		RS_LOG("ivsb,4\n");
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

	{
		ssize_t ssize_res;
		u32 sec_enable;
		char *endp;

		RS_LOG("ivsb,5\n");

		ssize_res = safe_vfs_read(filp, buffer, (SEC_ENABLE_READ_BUFFER_SIZE - 1), 0);

		if (ssize_res <= 0) {
			RS_LOG("ivsb,6\n");
			ret = ssize_res;
			goto out;
		}

		if (ssize_res >= SEC_ENABLE_READ_BUFFER_SIZE) {
			RS_LOG("ivsb,7\n");
			ret = -EINVAL;
			goto out;
		}

		memset(&buffer[ssize_res], 0, (SEC_ENABLE_READ_BUFFER_SIZE - ssize_res));

		sec_enable = simple_strtol(buffer, &endp, 16);

		if (sec_enable == SEC_ENABLE_FLAG) {
			RS_LOG("ivsb,8\n");
			g_is_sec_boot = 2; /*true*/
		} else {
			g_is_sec_boot = 1; /*false*/
		}

		ret = 0;
	}

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	RS_FREE_BUFFER(dev_name);

out_no_file:
	if (buffer) {
		rs_kfree(buffer);
	}

	return ret;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define is_secure_boot_enable RS_HIDE(_Ct9)
#endif
noused notrace static bool is_secure_boot_enable(void)
{
	if (!g_is_sec_boot) {
		scm_check_boot_fuses();

		check_vivo_secure_boot_enable();
	}

	if (g_is_sec_boot > 1)
		return true;
	else
		return false;
}

#if defined(CONFIG_QSEECOM)

#include "../../drivers/misc/qseecom_kernel.h"


struct send_cmd_req {
	uint32_t cmd_id;
	uint32_t data;
	uint32_t len;
} __PACKED;

struct send_cmd_rsp {
	uint32_t cmd_id;
	uint32_t data;
	int32_t status;
} __PACKED;

struct qseecom_send_cmd_req {
	void *cmd_req_buf; /* in */
	unsigned int cmd_req_len; /* in */
	void *resp_buf; /* in/out */
	unsigned int resp_len; /* in/out */
};

/*from drivers/misc/qseecom.c*/

enum qseecom_client_handle_type {
	QSEECOM_CLIENT_APP = 1,
	QSEECOM_LISTENER_SERVICE,
	QSEECOM_SECURE_SERVICE,
	QSEECOM_GENERIC,
	QSEECOM_UNAVAILABLE_CLIENT_APP,
};

#if 0
/*android 4.4 32bit*/
struct qseecom_client_handle {
	u32  app_id;
	u8 *sb_virt;
	s32 sb_phys;
	uint32_t user_virt_sb_base;
	size_t sb_length;
	struct ion_handle *ihandle;		/* Retrieve phy addr */
};
#endif

struct qseecom_client_handle {
	u32  app_id;
	u8 *sb_virt;
	phys_addr_t sb_phys;
	unsigned long user_virt_sb_base;
	size_t sb_length;
#if 0
	struct ion_handle *ihandle;		/* Retrieve phy addr */
	char app_name[MAX_APP_NAME_SIZE];
	u32  app_arch;
	struct qseecom_sec_buf_fd_info sec_buf_fd[MAX_ION_FD];
#endif
};

struct qseecom_listener_handle {
	u32			   id;
};

struct qseecom_dev_handle {
	enum qseecom_client_handle_type type;
	union {
		struct qseecom_client_handle client;
		struct qseecom_listener_handle listener;
	};
};


/*8996, from lk*/

#define KEYMASTER_CMD_ID  0x100UL
#define KEYMASTER_UTILS_CMD_ID  0x200UL

typedef enum {
	/*
	 * List the commands supportedin by the hardware.
	 */
	KEYMASTER_GET_SUPPORTED_ALGORITHMS		= (KEYMASTER_CMD_ID + 1UL),
	KEYMASTER_GET_SUPPORTED_BLOCK_MODES		= (KEYMASTER_CMD_ID + 2UL),
	KEYMASTER_GET_SUPPORTED_PADDING_MODES	= (KEYMASTER_CMD_ID + 3UL),
	KEYMASTER_GET_SUPPORTED_DIGESTS			= (KEYMASTER_CMD_ID + 4UL),
	KEYMASTER_GET_SUPPORTED_IMPORT_FORMATS	= (KEYMASTER_CMD_ID + 5UL),
	KEYMASTER_GET_SUPPORTED_EXPORT_FORMATS	= (KEYMASTER_CMD_ID + 6UL),
	KEYMASTER_ADD_RNG_ENTROPY				= (KEYMASTER_CMD_ID + 7UL),
	KEYMASTER_GENERATE_KEY					= (KEYMASTER_CMD_ID + 8UL),
	KEYMASTER_GET_KEY_CHARACTERISTICS		= (KEYMASTER_CMD_ID + 9UL),
	KEYMASTER_RESCOPE						= (KEYMASTER_CMD_ID + 10UL),
	KEYMASTER_IMPORT_KEY					= (KEYMASTER_CMD_ID + 11UL),
	KEYMASTER_EXPORT_KEY					= (KEYMASTER_CMD_ID + 12UL),
	KEYMASTER_DELETE_KEY					= (KEYMASTER_CMD_ID + 13UL),
	KEYMASTER_DELETE_ALL_KEYS				= (KEYMASTER_CMD_ID + 14UL),
	KEYMASTER_BEGIN							= (KEYMASTER_CMD_ID + 15UL),
	KEYMASTER_GET_OUTPUT_SIZE				= (KEYMASTER_CMD_ID + 16UL),
	KEYMASTER_UPDATE						= (KEYMASTER_CMD_ID + 17UL),
	KEYMASTER_FINISH						= (KEYMASTER_CMD_ID + 18UL),
	KEYMASTER_ABORT							= (KEYMASTER_CMD_ID + 19UL),

	KEYMASTER_SET_ROT						= (KEYMASTER_UTILS_CMD_ID + 1UL),
	KEYMASTER_READ_LK_DEVICE_STATE			= (KEYMASTER_UTILS_CMD_ID + 2UL),
	KEYMASTER_WRITE_LK_DEVICE_STATE			= (KEYMASTER_UTILS_CMD_ID + 3UL),
	KEYMASTER_MILESTONE_CALL				= (KEYMASTER_UTILS_CMD_ID + 4UL),
	KEYMASTER_SECURE_WRITE_PROTECT			= (KEYMASTER_UTILS_CMD_ID + 6UL),

	KEYMASTER_LAST_CMD_ENTRY				= (int)0xFFFFFFFFULL
} keymaster_cmd_t;

/*8976, from lk*/
enum app_commands {
	CLIENT_CMD_READ_LK_DEVICE_STATE = 0,
	CLIENT_CMD_LK_END_MILESTONE,
	CLIENT_CMD_GET_VERSION,
	CLIENT_CMD_WRITE_LK_DEVICE_STATE,
};


/*from drivers/misc/qseecom.c*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __qseecom_uvirt_to_kphys RS_HIDE(_Cta)
#endif
noused notrace static phys_addr_t __qseecom_uvirt_to_kphys(struct qseecom_handle *handle, uintptr_t virt)
{
	struct qseecom_dev_handle *data = (struct qseecom_dev_handle *)(handle->dev);

	return data->client.sb_phys + (virt - data->client.user_virt_sb_base);
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __qseecom_uvirt_to_kvirt RS_HIDE(_Ctb)
#endif
noused notrace static uintptr_t __qseecom_uvirt_to_kvirt(struct qseecom_handle *handle, uintptr_t virt)
{
	struct qseecom_dev_handle *data = (struct qseecom_dev_handle *)(handle->dev);

	return (uintptr_t)data->client.sb_virt + (virt - data->client.user_virt_sb_base);
}

#define QSEECOM_SBUFF_SIZE	4096

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define get_cmd_rsp_buffers RS_HIDE(_Ctc)
#endif
noused notrace static int get_cmd_rsp_buffers(struct qseecom_handle *handle,
	void **cmd,
	int *cmd_len,
	void **rsp,
	int *rsp_len)
{
	struct qseecom_dev_handle *data = (struct qseecom_dev_handle *)(handle->dev);

	if (*cmd_len & QSEECOM_ALIGN_MASK)
		*cmd_len = QSEECOM_ALIGN(*cmd_len);

	if (*rsp_len & QSEECOM_ALIGN_MASK)
		*rsp_len = QSEECOM_ALIGN(*rsp_len);

	if ((*rsp_len + *cmd_len) > data->client.sb_length/*QSEECOM_SBUFF_SIZE*/) {
		return -ENOMEM;
	}

	*cmd = handle->sbuf;
	*rsp = handle->sbuf + *cmd_len;

	return 0;
}

/*8996还需要调 qseecom_register_listener() 注册回调处理rpmb的读写才行，参考lk rpmb_listener_start/rpmb_listener_stop*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_device_info_rpmb RS_HIDE(_Ctd)
#endif
noused notrace static int read_device_info_rpmb(void *info, uint32_t size)
{
	#define QSEECOM_MAX_APP_NAME_LEN (12)

	int ret = 0;
	struct qseecom_handle *app_handle = NULL;
	struct send_cmd_req *read_req;
	struct send_cmd_rsp *read_rsp;
	uint32_t req_len;
	uint32_t rsp_len;
	char app_name[QSEECOM_MAX_APP_NAME_LEN];

	RS_LOG("rdir,0\n");

#if !defined(CONFIG_ARCH_MSM8996)
	if (QSEECOM_MAX_APP_NAME_LEN < (check_strings[csLKSecApp].str_len + 1)) {
		RS_LOG("rdir,1\n");
		ret = -EINVAL;
		goto out_no_handle;
	}

	memcpy(app_name, check_strings[csLKSecApp].str, check_strings[csLKSecApp].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(app_name, check_strings[csLKSecApp].str_len, csLKSecApp);
#endif

#else
	/* start keymaster app */
	if (QSEECOM_MAX_APP_NAME_LEN < (check_strings[csKeyMaster].str_len + 1)) {
		RS_LOG("rdir,2\n");
		ret = -EINVAL;
		goto out_no_handle;
	}
	memcpy(app_name, check_strings[csKeyMaster].str, check_strings[csKeyMaster].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(app_name, check_strings[csKeyMaster].str_len, csKeyMaster);
#endif
#endif

	/*strcpy(app_name, "keymaste");*/

	ret = qseecom_start_app(&app_handle, app_name, QSEECOM_SBUFF_SIZE);

	if (ret) {
		/*RS_LOG("qseecom_start_app failed %d\n", ret);*/
		RS_LOG("rdir,3\n");
		ret = -ENOSYS;
		goto out_no_handle;
	}

	req_len = sizeof(*read_req);
	rsp_len = sizeof(*read_rsp);

	ret = get_cmd_rsp_buffers(app_handle,
			(void **)&read_req,
			&req_len,
			(void **)&read_rsp,
			&rsp_len);

	if (ret != 0) {
		/*RS_LOG("get_cmd_rsp_buffers failed %d\n", ret);*/
		RS_LOG("rdir,4\n");
		goto out_with_handle;
	}

#if !defined(CONFIG_ARCH_MSM8996)
	read_req->cmd_id = CLIENT_CMD_READ_LK_DEVICE_STATE;
	read_req->data = (uint32_t)__qseecom_uvirt_to_kphys(app_handle, (uintptr_t)info);
	read_req->len = size;

	read_rsp->cmd_id = CLIENT_CMD_READ_LK_DEVICE_STATE;
#else
	read_req->cmd_id = KEYMASTER_READ_LK_DEVICE_STATE;
	read_req->data = (uint32_t)__qseecom_uvirt_to_kphys(app_handle, (uintptr_t)info);
	read_req->len = size;

	read_rsp->cmd_id = KEYMASTER_READ_LK_DEVICE_STATE;
#endif
	read_rsp->data = 0;
	read_rsp->status = 0;

	/* Read the device info */
	/*RS_LOG("req_len=%u,rsp_len=%u\n", req_len, rsp_len);*/

	/*arch_clean_invalidate_cache_range((addr_t) info, sz);*/
	ret = qseecom_send_command(app_handle, (void *)read_req, req_len, (void *)read_rsp, rsp_len);
	/*arch_invalidate_cache_range((addr_t) info, sz);*/

	if (ret < 0 || read_rsp->status < 0) {
		ret = ((ret < 0) ? ret : read_rsp->status);
		/*RS_LOG("Reading device info failed: Error: %d\n", ret);*/
		RS_LOG("rdir,5\n");
		/*ret = -1;*/
	}

out_with_handle:
	/*qseecom_shutdown_app(&app_handle);*/

out_no_handle:
	RS_LOG("rdir,e,%d\n", ret);
	return ret;
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define target_is_emmc_boot RS_HIDE(_Cte)
#endif
noused notrace static int target_is_emmc_boot(void)
{
	return 1;
}


#define DEV_PAGE_SIZE 4096 /*refer to "PAGE_SIZE" defination in lk*/

#define ROUNDUP(a, b) (((a) + ((b)-1)) & ~((b)-1))
#define ROUNDDOWN(a, b) ((a) & ~((b)-1))

#define DEVICE_MAGIC "ANDROID-BOOT!"
#define DEVICE_MAGIC_SIZE 13
#define MAX_PANEL_ID_LEN 64
#define MAX_VERSION_LEN  64

#if 0
struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	bool is_unlocked;
	bool is_tampered;
	bool is_unlock_critical;
	bool charger_screen_enabled;
	char display_panel[MAX_PANEL_ID_LEN];
	char bootloader_version[MAX_VERSION_LEN];
	char radio_version[MAX_VERSION_LEN];
	bool verity_mode; /* 1 = enforcing, 0 = logging*/
};
#else

/*kernel: sizeof(bool) == sizeof(char)*/
/*lk: sizeof(bool) == sizeof(int)*/

struct device_info {
	unsigned char magic[DEVICE_MAGIC_SIZE];
	lk_bool is_unlocked;
	lk_bool is_tampered;
	lk_bool is_unlock_critical;
	lk_bool charger_screen_enabled;
	char display_panel[MAX_PANEL_ID_LEN];
	char bootloader_version[MAX_VERSION_LEN];
	char radio_version[MAX_VERSION_LEN];
	lk_bool verity_mode; /* 1 = enforcing, 0 = logging*/
};
#endif

typedef struct device_info device_info;

/*#define BOOT_MAGIC "ANDROID!"*/
/*#define BOOT_MAGIC_SIZE 8*/
/*#define BOOT_NAME_SIZE  16*/
/*#define BOOT_ARGS_SIZE  512*/
#define BOOT_IMG_MAX_PAGE_SIZE 4096 /*defined in lk*/

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mmc_get_device_blocksize RS_HIDE(_Ctf)
#endif
noused notrace static uint32_t mmc_get_device_blocksize(void)
{
#if defined(RS_USE_UFS_DEV)
	/*search "block_size = " in lk*/
	if (g_no_emmc > 0) {
		/*refer to ufs_init() in lk*/
		return 4096;
	} else
#endif
	{
		/*refer to mmc_boot_init_card() in lk*/
		return 512; /*lk 中 BLOCK_SIZE 的定义*/
	}
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_device_info_mmc RS_HIDE(_Ctg)
#endif
noused notrace static int read_device_info_mmc(struct device_info *info)
{
#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	noused int pathlen;
	char *dev_name = NULL;
#if defined(RS_USE_DEVINFO_PART)
	bool devinfo_present;
#endif
	uint32_t blocksize;
	struct file *filp;
	char *info_buf;
	ssize_t ssize_res;
	int ret;

	loff_t dev_size;
	loff_t loff_res;
	loff_t offset;

	RS_LOG("rdim,0\n");

	if (!info) {
		RS_LOG("rdim,1\n");
		ret = -EINVAL;
		goto out_ret;
	}

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (!g_block_platform_devname) {
		RS_LOG("rdim,2\n");
		ret = 1;
		goto out_ret;
	}
#endif

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (RS_BUFFER_SIZE < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
	#if defined(RS_USE_DEVINFO_PART)
		+ check_strings[csSlashDevInfo].str_len
	#else
		+ check_strings[csSlashAboot].str_len
	#endif
		+ 1))
#else
	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
	#if defined(RS_USE_DEVINFO_PART)
		+ check_strings[csSlashDevInfo].str_len
	#else
		+ check_strings[csSlashAboot].str_len
	#endif
		+ 1))
#endif
	{
		/*buffer overrun*/
		RS_LOG("rdim,3\n");
		ret = -ENOBUFS;
		goto out_ret;
	}

	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		ret = -ENOMEM;
		goto out_ret;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

#if defined(RS_USE_DEVINFO_PART)
	memcpy(&dev_name[pathlen], check_strings[csSlashDevInfo].str, check_strings[csSlashDevInfo].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashDevInfo].str_len, csSlashDevInfo);
#endif

	if (is_file_exist(dev_name)) {
		RS_LOG("rdim,4\n");

		devinfo_present = true;
	} else
#endif
	{
	#if defined(RS_USE_DEVINFO_PART)
		devinfo_present = false;
	#endif

		memcpy(&dev_name[pathlen], check_strings[csSlashAboot].str, check_strings[csSlashAboot].str_len + 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashAboot].str_len, csSlashAboot);
	#endif
		RS_LOG("rdim,5,%s\n", dev_name);
	}

	info_buf = (char *)RS_GET_BUFFER();
	if (!info_buf) {
		RS_LOG("rdim,6\n");
		ret = -ENOMEM;
		goto out_with_dev_name;
	}

	filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		RS_LOG("rdim,7\n");
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out_no_buffer;
	}

	blocksize = mmc_get_device_blocksize();

#if defined(RS_USE_DEVINFO_PART)
	if (devinfo_present) {
		ssize_res = safe_vfs_read(filp, info_buf, blocksize, 0);
	} else
#endif
	{
		dev_size = safe_vfs_llseek(filp, 0, SEEK_END);
		if (dev_size <= 0) {
			RS_LOG("rdim,8\n");
			ret = -EINVAL;
			goto out;
		}

		offset = dev_size - blocksize;

		loff_res = safe_vfs_llseek(filp, offset, SEEK_SET);

		if (loff_res < 0) {
			RS_LOG("rdim,9\n");
			ret = loff_res;
			goto out;
		}

		ssize_res = safe_vfs_read(filp, info_buf, blocksize, offset);
	}

	if (ssize_res <= 0) {
		RS_LOG("rdim,10\n");
		ret = ssize_res;
		goto out;
	}

	if (ssize_res < sizeof(*info)/*blocksize*/) {
		memset(&info_buf[ssize_res], 0, ((ssize_t)sizeof(*info) - ssize_res));
	}

	memcpy(info, info_buf, sizeof(*info));

	ret = 0;

out:
	RS_FREE_BUFFER(info_buf);

out_no_buffer:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

out_with_dev_name:
	RS_FREE_BUFFER(dev_name);

out_ret:
	RS_LOG("rdim,e,%d\n", ret);
	return ret;
#else
	return -ENODEV;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_device_info_flash RS_HIDE(_Cth)
#endif
noused notrace static int read_device_info_flash(device_info * dev_info)
{
	memset(dev_info, 0, sizeof(*dev_info));
	return -ENODEV;

#if 0
	int ret;
	struct device_info *info = rs_memalign(DEV_PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, DEV_PAGE_SIZE));
	struct ptentry *ptn;
	struct ptable *ptable;
	if (info == NULL) {
		/*dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");*/
		/*ASSERT(0);*/
		ret = -ENOMEM;
		goto out;
	}
	info_buf = info;
	ptable = flash_get_ptable();
	if (ptable == NULL) {
		/*dprintf(CRITICAL, "ERROR: Partition table not found\n");*/
		ret = -ENXIO;
		goto out;
	}

	ptn = ptable_find(ptable, "devinfo");
	if (ptn == NULL) {
		/*dprintf(CRITICAL, "ERROR: No devinfo partition found\n");*/
		ret = -ENODEV;
		goto out;
	}

	ret = flash_read(ptn, 0, (void *)info_buf, page_size);
	if (ret) {
		/*dprintf(CRITICAL, "ERROR: Cannot write device info\n");*/
		goto out;
	}

	if (memcmp(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE)) {
		memcpy(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE);
		info->is_unlocked = 0;
		info->is_tampered = 0;
		write_device_info_flash(info);
	}
	memcpy(dev, info, sizeof(device_info));
	rs_memalign_free(info);
out:
	return ret;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_device_info_shared_mem RS_HIDE(_Cth_)
#endif
noused notrace static int read_device_info_shared_mem(struct device_info *info)
{
	/*refer to MSM_SHARED_IMEM_BASE in lk\platform\xxx\include\platform\iomap.h*/
	#define MSM_SHARED_IMEM_BASE (0)

	#define LK_SHARED_DEVICE_INFO_SIZE (256)
	#define EDL_MODE_ADDR_OFFSET (0xFE0)

	int ret = -EINVAL;

#if defined(CONFIG_OF) && defined(CONFIG_OF_ADDRESS)
	/*refer to msm_restart_probe() in drivers/power/reset/msm_poweroff.c*/
	#define EDL_MODE_PROP "qcom,msm-imem-emergency_download_mode"
	#define DL_MODE_PROP "qcom,msm-imem-download_mode"
	#define MSM_IMEM_PROP "qcom,msm-imem"

	struct device_node *np;
#endif

	uintptr_t source = 0;

	RS_LOG("rdsm,0\n");

#if defined(CONFIG_OF) && defined(CONFIG_OF_ADDRESS)
	np = of_find_compatible_node(NULL, NULL, EDL_MODE_PROP);
	if (!np) {
		RS_LOG("rdsm,1\n");
	} else {
		struct resource res;
		if (of_address_to_resource(np, 0, &res))
			RS_LOG("rdsm,2\n");
		else
			source = res.start - LK_SHARED_DEVICE_INFO_SIZE;
	}

	if (!np) {
		RS_LOG("rdsm,3\n");

		np = of_find_compatible_node(NULL, NULL, DL_MODE_PROP);
		if (!np) {
			RS_LOG("rdsm,4\n");

			np = of_find_compatible_node(NULL, NULL, MSM_IMEM_PROP);
			if (!np) {
				RS_LOG("rdsm,5\n");
			}
		}

		if (np) {
			struct resource res;

			RS_LOG("rdsm,6\n");

			if (of_address_to_resource(np, 0, &res))
				RS_LOG("rdsm,7\n");
			else
				source = res.start + EDL_MODE_ADDR_OFFSET - LK_SHARED_DEVICE_INFO_SIZE;
		}
	}

#else
	/*refer to MSM_SHARED_IMEM_BASE definition in lk*/
	#if defined(MSM_SHARED_IMEM_BASE) && ((MSM_SHARED_IMEM_BASE + EDL_MODE_ADDR_OFFSET) > LK_SHARED_DEVICE_INFO_SIZE)
	source = MSM_SHARED_IMEM_BASE + EDL_MODE_ADDR_OFFSET - LK_SHARED_DEVICE_INFO_SIZE;
	#endif
#endif

	if (source) {
		struct device_info *shared_info = (struct device_info *)ioremap(source, LK_SHARED_DEVICE_INFO_SIZE);

		if (shared_info) {
			char *dest = (char *)info;
			size_t len = sizeof(*shared_info);

			RS_LOG("rdsm,8\n");

			if (len > LK_SHARED_DEVICE_INFO_SIZE) {
				len = LK_SHARED_DEVICE_INFO_SIZE;
			}
			memcpy_fromio(dest, shared_info, len);
			iounmap(shared_info);

			if (len < sizeof(*shared_info)) {
				memset(&dest[len], 0, (sizeof(*shared_info) - len));
			}

			ret = 0;
		}
	}

	return ret;
}

#if defined(CONFIG_QSEECOM) && (defined(CONFIG_ARCH_MSM8996) || defined(CONFIG_ARCH_SDM845))
	/*#define USE_RPMB_FOR_DEVINFO*/ /*从 2016.02.02 起 8996 lk 中已关闭 USE_RPMB_FOR_DEVINFO*/
	#define USE_LK_SHARED_DEVINFO /*需要 lk 中 aboot.c read_device_info/write_device_info 保存device_info到共享内存区域(IMEM)*/
#endif


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define verify_device_info RS_HIDE(_Cth0)
#endif
noused notrace static int verify_device_info(device_info * info)
{
	int ret;

	if (!info) {
		RS_LOG("vdi,1\n");
		return -EINVAL;
	}

	/*if (memcmp(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE))*/
	if (STRNCMP(info->magic, check_strings[csDevInfoMagic].str, check_strings[csDevInfoMagic].str_len, csDevInfoMagic)) {
		RS_LOG("vdi,2\n");
	#if 0
		/*memcpy(info->magic, DEVICE_MAGIC, DEVICE_MAGIC_SIZE);*/
		memcpy(info->magic, check_strings[csDevInfoMagic].str, check_strings[csDevInfoMagic].str_len + 1);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(info->magic, check_strings[csDevInfoMagic].str_len, csDevInfoMagic);
	#endif
		if (is_secure_boot_enable()) {
			info->is_unlocked = 0;
			info->is_unlock_critical = 0;
		} else {
			info->is_unlocked = 1;
			info->is_unlock_critical = 1;
		}
		info->is_tampered = 0;
		info->charger_screen_enabled = 0;
		info->verity_mode = 1; /*enforcing by default*/
		/*write_device_info(info);*/
	#endif
		ret = -ENXIO;
	} else {
		ret = 0;
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_device_info RS_HIDE(_Cti)
#endif
noused notrace static int read_device_info(device_info * dev_info)
{
	int ret;

	RS_LOG("rdi,0\n");

#if defined(USE_LK_SHARED_DEVINFO)
	ret = read_device_info_shared_mem(dev_info);

	RS_LOG("rdi,0.1,%d\n", ret);

	if (!ret) {
		ret = verify_device_info(dev_info);
		if (!ret) {
			RS_LOG("rdi,0.2\n");

			goto out;
		}
	}
#endif

	if (target_is_emmc_boot()) {
		struct device_info *info = rs_memalign(DEV_PAGE_SIZE, ROUNDUP(BOOT_IMG_MAX_PAGE_SIZE, DEV_PAGE_SIZE));
		RS_LOG("rdi,1\n");
		if (info == NULL) {
			/*dprintf(CRITICAL, "Failed to allocate memory for device info struct\n");*/
			/*ASSERT(0);*/
			RS_LOG("rdi,2\n");
			ret = -ENOMEM;
			goto out;
		} else if (IS_ERR(info)) {
			RS_LOG("rdi,2.1\n");
			ret = PTR_ERR(info);
			goto out;
		}

		memset(info, 0, sizeof(*info));

	#if defined(USE_RPMB_FOR_DEVINFO)
		if (is_secure_boot_enable()) {
			RS_LOG("rdi,3\n");
			ret = read_device_info_rpmb((void *)info, DEV_PAGE_SIZE);
			if (ret < 0) {
				/*ASSERT(0);*/
				RS_LOG("rdi,4\n");
				goto out_with_info;
			}
		} else {
			RS_LOG("rdi,5\n");
			ret = read_device_info_mmc(info);
			if (ret < 0) {
				goto out_with_info;
			}
		}
	#else
		RS_LOG("rdi,6\n");
		ret = read_device_info_mmc(info);
		if (ret < 0) {
			goto out_with_info;
		}
	#endif

		ret = verify_device_info(info);
		if (!ret) {
			RS_LOG("rdi,8\n");

			memcpy(dev_info, info, sizeof(device_info));

			ret = 0;
		}

	out_with_info:
		rs_memalign_free(info);
	} else {
		ret = read_device_info_flash(dev_info);
	}
out:
	RS_LOG("rdi,e,%d\n", ret);
	return ret;
}
#endif

#if defined(RS_MTK_PLATFORM)

/*MTK 4.4~5.0*/
typedef struct tag_vivo_mtk_device_info {
	lk_bool is_tamperal;
	lk_bool is_unlock;
} vivo_mtk_device_info;


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define read_vivo_mtk_device_info RS_HIDE(_Cto)
#endif
noused notrace static int read_vivo_mtk_device_info(struct tag_vivo_mtk_device_info *dev_info)
{
#if defined(RS_HAS_DEV_SHORTCUTS) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	noused int pathlen;
	char *dev_name = NULL;
	struct file *filp;
	loff_t loff_res;
	ssize_t ssize_res;
	int ret, se_flag = 0;
	loff_t dev_size;
	loff_t offset;

	RS_LOG("rvmdi,0\n");

#if defined(RS_HAS_DEV_SHORTCUTS)
	if (RS_BUFFER_SIZE < (check_strings[csDevLK].str_len + 1)) {
		/*buffer overrun*/
		RS_LOG("rvmdi,1\n");
		ret = -EINVAL; /*-ENOBUFS;*/
		goto out_ret;
	}

	/*dev_name = "/dev/lk";*/
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		RS_LOG("rvmdi,2\n");
		ret = -ENOMEM;
		goto out_ret;
	}

	memcpy(dev_name, check_strings[csDevLK].str, check_strings[csDevLK].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevLK].str_len, csDevLK);
#endif

	if (is_file_exist(dev_name)) {
		RS_LOG("rvmdi,3\n");
		goto open_dev;
	}
#endif

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)

#if !defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (!g_block_platform_devname) {
		RS_LOG("rvmdi,4\n");
		ret = 1;
		goto out_with_dev_name;
	}
#endif

#if !defined(RS_HAS_DEV_SHORTCUTS)
	dev_name = (char *)RS_GET_BUFFER();
	if (!dev_name) {
		RS_LOG("rvmdi,5\n");
		ret = -ENOMEM;
		goto out_ret;
	}
#endif

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (RS_BUFFER_SIZE < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
		+ (sizeof("/lk") - 1) + 1))
#else
	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
		+ (sizeof("/lk") - 1) + 1))
#endif
	{
		/*buffer overrun*/
		RS_LOG("rvmdi,6\n");
		ret = -ENOBUFS;
		goto out_with_dev_name;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

	#undef SLASH_DEV_LEN
	#define SLASH_DEV_LEN (sizeof("/dev") - 1)

	memcpy(&dev_name[pathlen], &check_strings[csDevLK].str[SLASH_DEV_LEN], (check_strings[csDevLK].str_len - SLASH_DEV_LEN) + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], SLASH_DEV_LEN, (check_strings[csDevLK].str_len - SLASH_DEV_LEN), csDevLK);
#endif

#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
open_dev:
#endif

	ret = 0; /*-Wmaybe-uninitialized*/

	se_flag = test_and_set_thread_flag((BITS_PER_LONG - 1));

	filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(filp)) {
		RS_LOG("rvmdi,7\n");
		ret = PTR_ERR(filp);
		filp = NULL;
		goto out;
	}

	dev_size = safe_vfs_llseek(filp, 0, SEEK_END);
	if (dev_size <= 0) {
		RS_LOG("rvmdi,8\n");
		ret = -EINVAL;
		goto out;
	}

	/*struct vivo_mtk_device_info *pdevice = container_of(lk_end_addr - 4, vivo_mtk_device_info, is_unlock);*/

	offset = dev_size - sizeof(*dev_info);

	loff_res = safe_vfs_llseek(filp, offset, SEEK_SET);

	if (loff_res < 0) {
		RS_LOG("rvmdi,9\n");
		ret = loff_res;
		goto out;
	}

	ssize_res = safe_vfs_read(filp, dev_info, sizeof(*dev_info), offset);

	if (ssize_res <= 0) {
		ret = ssize_res;
		goto out;
	}

	if (ssize_res < sizeof(*dev_info)) {
		char *fill_buf = (char *)dev_info;
		memset(&fill_buf[ssize_res], 0, ((ssize_t)sizeof(*dev_info) - ssize_res));
	}

	/*memcpy(info, info_buf, sizeof(*info));*/

	ret = 0;

out:
	if (filp) {
		filp_close(filp, RS_FILP_CLOSE_ID);
	}

	if (!se_flag) {
		clear_thread_flag((BITS_PER_LONG - 1));
	}

out_with_dev_name:
	RS_FREE_BUFFER(dev_name);

out_ret:
	RS_LOG("rvmdi,e,%d\n", ret);
	return ret;
#else
	memset(dev_info, 0, sizeof(*dev_info));
	return -ENODEV;
#endif
}

#endif

#if 0 /*defined(CONFIG_MTK_SECURITY_SW_SUPPORT)*/
/*no MTK_SECURITY_MODULE_LITE defined*/

#undef HASH_SIZE /*otherwise, warning: "HASH_SIZE" redefined*/

/*kernel\drivers\misc\mediatek\masp\asf\asf_inc\sec_boot_lib.h*/

#if !defined(CONFIG_ARCH_MT6755)
#include "../../drivers/misc/mediatek/masp/asf/asf_inc/sec_boot_lib.h"
#include "../../drivers/misc/mediatek/masp/asf/asf_inc/sec_ioctl.h"
#include <mach/sec_osal.h>
#include <mach/mt_sec_hal.h>

extern SECCFG_U seccfg;
extern bool bSECROInit;

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_check_sec_boot_unlocked RS_HIDE(_Ctp)
#endif
noused notrace static int rs_check_sec_boot_unlocked(const SECCFG_U * cfg)
{
	int ret = -EINVAL;
	int is_unlocked = -1;

	if ((!cfg) || (g_is_unlocked)) {
		goto out_ret;
	}

	if (SEC_CFG_END_PATTERN == cfg->v1.end_pattern) {
		if ((SECCFG_V1 != get_seccfg_ver()) && (SECCFG_V1_2 != get_seccfg_ver())) {
			/*SMSG(true,"[%s] mismatch seccfg version v%d\n",MOD,get_seccfg_ver());*/
			/*SEC_ASSERT(0);*/
			goto out;
		}

		/*cipher_len = get_seccfg_cipher_len();*/
		/*sec_update_lks(seccfg.v1.sw_sec_lock_try, seccfg.v1.sw_sec_lock_done, seccfg.v1.attr == ATTR_DISABLE_IMG_CHECK);*/
		/*masp_hal_sp_hacc_enc((unsigned char*)&seccfg.v1.image_info,cipher_len,rom_info.m_SEC_CTRL.m_seccfg_ac_en,HACC_USER1,FALSE);*/
		if (cfg->v1.attr == ATTR_DISABLE_IMG_CHECK) {
			is_unlocked = 2;
		} else {
			is_unlocked = 1;
		}

		ret = 0;
	} else if (SEC_CFG_END_PATTERN == seccfg.v3.end_pattern) {
		if (SECCFG_V3 != get_seccfg_ver()) {
			/*SMSG(true,"[%s] mismatch seccfg version v%d\n",MOD,get_seccfg_ver());*/
			/*SEC_ASSERT(0);*/
			goto out;
		}

		/*cipher_len = get_seccfg_cipher_len();*/
		/*sec_update_lks(seccfg.v3.sw_sec_lock_try, seccfg.v3.sw_sec_lock_done, seccfg.v3.seccfg_attr == ATTR_DISABLE_IMG_CHECK);*/
		/*masp_hal_sp_hacc_enc((unsigned char*)&seccfg.v3.image_info,cipher_len,rom_info.m_SEC_CTRL.m_seccfg_ac_en,HACC_USER1,FALSE);*/
		if (cfg->v3.seccfg_attr == ATTR_DISABLE_IMG_CHECK) {
			is_unlocked = 2;
		} else {
			is_unlocked = 1;
		}

		ret = 0;
	}
out:
	g_is_unlocked = is_unlocked;

out_ret:
	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define load_seccfg_and_check RS_HIDE(_Ctq)
#endif
noused notrace static int load_seccfg_and_check(struct file *sec_filp, char *dev_name)
{
#if defined(RS_HAS_DEV_SHORTCUTS) || defined(RS_HAS_DEV_BYNAME_SHORTCUTS)
	noused int pathlen;
	struct file *seccfg_filp = NULL;
	ssize_t ssize_res;
	char *buffer = NULL;
	int try_lock_cnt;
	int ret;
	loff_t part_size;
	loff_t loff_res;

	RS_LOG("lscc,0\n");

	if ((IS_ERR_OR_NULL(sec_filp)) || (!dev_name)) {
		RS_LOG("lscc,1\n");
		ret = -EINVAL;
		goto out_ret;
	}

#if defined(RS_HAS_DEV_SHORTCUTS)
	if (RS_BUFFER_SIZE < (check_strings[csDevSecCfg].str_len + 1)) {
		/*buffer overrun*/
		RS_LOG("lscc,2\n");
		ret = -EINVAL; /*-ENOBUFS;*/
		goto out_ret;
	}

	memcpy(dev_name, check_strings[csDevSecCfg].str, check_strings[csDevSecCfg].str_len + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevSecCfg].str_len, csDevSecCfg);
#endif

	if (is_file_exist(dev_name)) {
		RS_LOG("lscc,3\n");
		goto open_dev;
	}
#endif

#if defined(RS_HAS_DEV_BYNAME_SHORTCUTS)

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	if (RS_BUFFER_SIZE < (check_strings[csDevBlockSlash].str_len
		+ (check_strings[csSlashByName].str_len - 1)
		+ (sizeof("/seccfg") - 1) + 1))
#else
	if (!g_block_platform_devname) {
		RS_LOG("lscc,4\n");
		ret = 1;
		goto out_ret;
	}

	if (RS_BUFFER_SIZE < (check_strings[csBlkPlatformSlash].str_len
		+ g_block_platform_devname_len + check_strings[csSlashByName].str_len
		+ (sizeof("/seccfg") - 1) + 1))
#endif
	{
		/*buffer overrun*/
		RS_LOG("lscc,5\n");
		ret = -ENOBUFS;
		goto out_ret;
	}

#if defined(RS_USE_DEV_BLOCK_BY_NAME)
	pathlen = check_strings[csDevBlockSlash].str_len;
	memcpy(dev_name, check_strings[csDevBlockSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csDevBlockSlash].str_len, csDevBlockSlash);
#endif
	memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);

#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len, csSlashByName);
#endif
	pathlen += (check_strings[csSlashByName].str_len - 1);
#else /*RS_USE_DEV_BLOCK_BY_NAME*/
	pathlen = check_strings[csBlkPlatformSlash].str_len;
	memcpy(dev_name, check_strings[csBlkPlatformSlash].str, pathlen);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt(dev_name, check_strings[csBlkPlatformSlash].str_len, csBlkPlatformSlash);
#endif
	if (g_block_platform_devname_len) {
		memcpy(&dev_name[pathlen], g_block_platform_devname, g_block_platform_devname_len);
		pathlen += g_block_platform_devname_len;
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(&dev_name[pathlen], check_strings[csSlashByName].str_len, csSlashByName);
	#endif
		pathlen += check_strings[csSlashByName].str_len;
	} else {
		memcpy(&dev_name[pathlen], check_strings[csSlashByName].str + 1, check_strings[csSlashByName].str_len);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt_ex(&dev_name[pathlen], 1, check_strings[csSlashByName].str_len - 1, csSlashByName);
	#endif
		pathlen += (check_strings[csSlashByName].str_len - 1);
	}
#endif /*!RS_USE_DEV_BLOCK_BY_NAME*/

	#undef SLASH_DEV_LEN
	#define SLASH_DEV_LEN (sizeof("/dev") - 1)

	/* "/seccfg" */
	memcpy(&dev_name[pathlen], &check_strings[csDevSecCfg].str[SLASH_DEV_LEN], (check_strings[csDevSecCfg].str_len - SLASH_DEV_LEN) + 1);
#if defined(RS_ENCRYPT_STR)
	simple_str_encrypt_ex(&dev_name[pathlen], SLASH_DEV_LEN, (check_strings[csDevSecCfg].str_len - SLASH_DEV_LEN), csDevSecCfg);
#endif

#endif

#if defined(RS_HAS_DEV_SHORTCUTS)
open_dev:
#endif

	seccfg_filp = local_filp_open(dev_name, RS_READ_FILE_OPEN_FLAGS, RS_READ_FILE_OPEN_MODE);
	if (IS_ERR(seccfg_filp)) {
		RS_LOG("lscc,6\n");
		ret = PTR_ERR(seccfg_filp);
		seccfg_filp = NULL;
		goto out_ret;
	}

	part_size = safe_vfs_llseek(seccfg_filp, 0, SEEK_END);
	if (part_size < 0) {
		RS_LOG("lscc,7\n");
		ret = part_size;
		goto out_no_buffer;
	}

	loff_res = safe_vfs_llseek(seccfg_filp, 0, SEEK_SET);

	if (loff_res < 0) {
		RS_LOG("lscc,8\n");
		ret = loff_res;
		goto out_no_buffer;
	}

	if (part_size > sizeof(seccfg)) {
		part_size = sizeof(seccfg);
	}

	buffer = (char *)vmalloc(part_size);
	if (!buffer) {
		RS_LOG("lscc,9\n");
		ret = -ENOMEM;
		goto out_no_buffer;
	}

	ssize_res = safe_vfs_read(seccfg_filp, buffer, part_size, 0);
	if (ssize_res <= 0) {
		RS_LOG("lscc,10\n");
		ret = ssize_res;
		goto out;
	}

	if (ssize_res < part_size) {
		memset(&buffer[ssize_res], 0, ((ssize_t)part_size - ssize_res));
	}

	try_lock_cnt = 0;

try_lock:
	ret = __vfs_ioctl(sec_filp, SEC_HACC_LOCK, 0);
	if (ret < 0) {
		if (should_retry_syscall(ret)) {
			try_lock_cnt++;
			if (try_lock_cnt < 10) {
				RS_LOG("lscc,11\n");
				msleep(100);
				goto try_lock;
			}
		}
		RS_LOG("lscc,12\n");
		goto out;
	}

	{
		int call_ret;
		ret = __vfs_ioctl(sec_filp, SEC_HACC_CONFIG, (unsigned long)&call_ret);
		if (ret < 0) {
			RS_LOG("lscc,13\n");
			goto out_locked;
		}
	}

	ret = __vfs_ioctl(sec_filp, SEC_SECCFG_DECRYPT, (unsigned long)buffer);
	if (ret < 0) {
		RS_LOG("lscc,14\n");
		goto out_locked;
	}

	rs_check_sec_boot_unlocked((const SECCFG_U *)buffer);

out_locked:
	__vfs_ioctl(sec_filp, SEC_HACC_UNLOCK, 0);

out:
	if (buffer) {
		vfree(buffer);
	}

out_no_buffer:
	if (seccfg_filp) {
		filp_close(seccfg_filp, RS_FILP_CLOSE_ID);
	}

out_ret:
	RS_LOG("lscc,e,%d\n", ret);
	return ret;
#else
	return -EINVAL;
#endif
}

#else
/*defined(CONFIG_ARCH_MT6755)*/

/*refer to bootable\bootloader\lk\platform\mt6755\include\platform\lock_state.h*/


/* LKS means "LocK State" */
typedef enum {
	LKS_DEFAULT				= 0x01,
	LKS_MP_DEFAULT,
	LKS_UNLOCK,
	LKS_LOCK,
	LKS_VERIFIED,
	LKS_CUSTOM,
} LKS;

/* LKCS means "LocK Crtitical State" */
typedef enum {
	LKCS_UNLOCK				 = 0x01,
	LKCS_LOCK,
} LKCS;

/*refer to bootable\bootloader\lk\platform\mt6755\include\platform\sec_logo_auth.h*/

typedef enum {
	SEC_LOCK = 0,
	SEC_UNLOCK = 1,
	SEC_VERIFIED = 2,
	SEC_CUSTOM = 3,
	SEC_UNDEFINED = 0xff,
} LOCK_STATE;
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_sec_lock RS_HIDE(_Ctr)
#endif
noused notrace static int check_sec_lock(void)
{
	int ret = 0;

#if defined(CONFIG_ARCH_MT6755)
	extern unsigned int g_lock_state;
#endif

	RS_LOG("csl,0\n");

#if defined(CONFIG_ARCH_MT6755)
	if (g_lock_state == LKS_UNLOCK) {
		g_is_unlocked = 2;
	} else {
		g_is_unlocked = 1;
	}
#else
	if ((!bSECROInit)
		|| ((SEC_CFG_END_PATTERN != seccfg.v1.end_pattern)
		&& (SEC_CFG_END_PATTERN != seccfg.v3.end_pattern))
		) {
		char *dev_name;
		struct file *sec_filp;

		#undef DEV_SEC_LEN
		#define DEV_SEC_LEN (sizeof("/dev/sec") - 1)

	#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
		if (!(current->flags & PF_KTHREAD)) {
			ret = -EBUSY;
			goto out_ret;
		}
	#endif

		if (RS_BUFFER_SIZE < (DEV_SEC_LEN + 1)) {
			/*buffer overrun*/
			RS_LOG("csl,1\n");
			ret = -EINVAL; /*-ENOBUFS;*/
			goto out_ret;
		}

		dev_name = (char *)RS_GET_BUFFER();
		if (!dev_name) {
			RS_LOG("csl,2\n");
			ret = -ENOMEM;
			goto out_ret;
		}


		/* "/dev/sec" */
		memcpy(dev_name, check_strings[csDevSecCfg].str, DEV_SEC_LEN);
	#if defined(RS_ENCRYPT_STR)
		simple_str_encrypt(dev_name, DEV_SEC_LEN, csDevSecCfg);
	#endif
		dev_name[DEV_SEC_LEN] = '\0';

		if (!is_file_exist(dev_name)) {
			RS_LOG("csl,3\n");
			ret = -ENODEV;
			goto out_with_dev_name;
		}

		sec_filp = local_filp_open(dev_name, RS_IOCTL_OPEN_FLAGS, RS_IOCTL_OPEN_MODE);
		if (IS_ERR(sec_filp)) {
			RS_LOG("csl,4\n");
			ret = PTR_ERR(sec_filp);
			sec_filp = NULL;
			goto out;
		}

		if (!bSECROInit) {
			int call_ret = 0;
			RS_LOG("csl,5\n");
		#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
			/* bSECROInit 在 masp_boot_init() 中最后被设为真，
			 * sec_core_ioctl(SEC_BOOT_INIT) -> masp_boot_init -> sec_dev_read_secroimg -> masp_hal_sp_hacc_dec
			 * -> hacc_secure_request -> open_sdriver_connection -> mc_open_device
			 * -> connection_read_datablock -> connection_read_data 通过 netlink 与 daemon 通讯
			 *需要 /system/bin/mcDriverDaemon 运行起来后才能成功通过
			 *ccci_mdinit、rpmb_open都会调到 mc_open_device, sbchk会调到 sec_core_ioctl
			 */
		#endif

			ret = __vfs_ioctl(sec_filp, SEC_BOOT_INIT, (unsigned long)&call_ret);

			if (ret < 0) {
				RS_LOG("csl,6\n");
				goto out;
			}
		}

		if (bSECROInit) {
			RS_LOG("csl,7\n");
			if (sec_boot_enabled()) {
				RS_LOG("csl,8\n");
				load_seccfg_and_check(sec_filp, dev_name);
			}
		}

	out:
		if (sec_filp) {
			filp_close(sec_filp, RS_FILP_CLOSE_ID);
		}

	out_with_dev_name:

		if (dev_name) {
			RS_FREE_BUFFER(dev_name);
		}
	} else {
		RS_LOG("csl,9\n");
		ret = rs_check_sec_boot_unlocked((const SECCFG_U *)&seccfg);
	}

out_ret:
	RS_LOG("csl,e,%d\n", ret);
#endif
	return ret;
}

#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_check_sec RS_HIDE(_CtA)
#endif
noused notrace static void rs_check_sec(void)
{
#if defined(RS_MTK_PLATFORM)

#if defined(CONFIG_MTK_SECURITY_SW_SUPPORT)
	extern int sec_boot_enabled(void);
	extern int sec_schip_enabled(void);

	RS_LOG("rcs,0\n");

	if (!g_is_sec_schip) {
		/*data from MTK devinfo (inited from device tree, device: /dev/devmap, ioctl: READ_DEV_DATA*/
		int is_sec_schip = sec_schip_enabled();
		RS_LOG("rcs,1\n");
		if (is_sec_schip) {
			RS_LOG("rcs,2\n");
			g_is_sec_schip = 2;
		} else {
			g_is_sec_schip = 1;
		}
	}

	if (!g_is_unlocked) {
		RS_LOG("rcs,3\n");
		/*check_sec_lock();*/
	}

	if (!g_is_sec_boot) {
		RS_LOG("rcs,4\n");

	#if !defined(CONFIG_ARCH_MT6755)
		/*if (bSECROInit)*/
	#endif
		{
			int is_sec_boot = sec_boot_enabled();
			RS_LOG("rcs,5\n");
			if (is_sec_boot) {
				RS_LOG("rcs,6\n");
				g_is_sec_boot = 2;
			} else {
				g_is_sec_boot = 1;
			}
		}
	}
#endif

	if (!g_is_unlocked) {
		int ret;
		vivo_mtk_device_info dev_info;
		RS_LOG("rcs,7\n");
		ret = read_vivo_mtk_device_info(&dev_info);
		if (!ret) {
			RS_LOG("rcs,8\n");
			g_is_unlocked = (dev_info.is_unlock ? 2 : 1);
			g_is_tampered = (dev_info.is_tamperal ? 2 : 1);
		} else {
			g_is_unlocked = -1;
			g_is_tampered = -1;
		}
	}

#elif defined(RS_QUALCOMM_PLATFORM)

	if (!g_is_sec_schip) {
		RS_LOG("rcs,9\n");
		g_is_sec_schip = (IS_ENABLED(CONFIG_QSEECOM) ? 2 : 1);
		is_secure_boot_enable(); /*will modify g_is_sec_boot*/

		{
			int ret;
			device_info dev_info;
			dev_info.is_unlocked = 0;
			dev_info.is_tampered = 0;
			RS_LOG("rcs,10\n");
			ret = read_device_info(&dev_info);
			if (!ret) {
				RS_LOG("rcs,11\n");
				g_is_unlocked = dev_info.is_unlocked;
				g_is_tampered = dev_info.is_tampered;
			} else {
				g_is_unlocked = -1;
				g_is_tampered = -1;
			}
		}
	}
#endif
	RS_LOG("rcs,e\n");
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_do_init_mount_check_env_info RS_HIDE(_CtB)
#endif
noused notrace static void rs_do_init_mount_check_env_info(void)
{
	RS_LOG("imcei,0\n");
	if (!g_is_unlocked) {
		RS_LOG("imcei,1\n");
		rs_check_sec();

		if (g_is_unlocked > 1) {
			g_mount_check_env_info |= RS_MOUNT_CHECK_UNLOCKED_FLAG;
		}

		if (g_is_tampered > 1) {
			g_mount_check_env_info |= RS_MOUNT_CHECK_TAMPERED_FLAG;
		}

		if (g_is_sec_schip > 1) {
			g_mount_check_env_info |= RS_MOUNT_CHECK_SECCHIP_FLAG;
		}

		if (g_is_sec_boot > 1) {
			g_mount_check_env_info |= RS_MOUNT_CHECK_SECBOOT_FLAG;
		}
	}

	RS_LOG("imcei,e\n");
}

#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_do_init_mount_check_env_info_thread RS_HIDE(_CtC)
#endif
noused notrace static int rs_do_init_mount_check_env_info_thread(void *context)
{
	/*wait for mounting of /system*/
	msleep(2000);

	RS_LOG("ceit start\n");
	rs_do_init_mount_check_env_info();
	RS_LOG("ceit end\n");

	return 0;
}
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_mount_check_env_info RS_HIDE(_CtD)
#endif
noused notrace static int rs_init_mount_check_env_info(void)
{
	if (rs_get_boot_mode() == RS_NORMAL_BOOT) {
	#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT)
		/*refer to emmc_rpmb_thread() in drivers/misc/mediatek/mmc-host/emmc_rpmb.c*/
		char comm[TASK_COMM_LEN];
		struct task_struct *father, *p, *n;
		struct task_struct *cei_thread;
		char *cei_thread_name = NULL;
		int ret;

		father = &init_task;

		read_lock(&tasklist_lock);
		list_for_each_entry_safe(p, n, &father->children, sibling) {
			struct task_struct *t = p;
			if (t->real_parent == father) {
				strlcpy(comm, t->comm, sizeof(comm));
				comm[sizeof(comm) - 1] = '\0';
				cei_thread_name = comm;
				break;
			}
		}
		read_unlock(&tasklist_lock);

		if (!cei_thread_name) {
			cei_thread_name = init_task.comm;
		}

		cei_thread = kthread_run(rs_do_init_mount_check_env_info_thread, NULL, cei_thread_name/*"ceit"*/);

		if (!cei_thread) {
			ret = -ENOEXEC;
		} else if (IS_ERR(cei_thread)) {
			ret = PTR_ERR(cei_thread);
		} else {
			ret = 0;
		}

		/*kthread_stop(cei_thread);*/

		if (ret) {
			RS_LOG("ceit run failed!\n");
		}

		return ret;
	#else
		rs_do_init_mount_check_env_info();
	#endif
	}

	return 0;
}

#if defined(CONFIG_RS_CHECK_FUNC)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_init_func_checksums RS_HIDE(_CA)
#endif

noused notrace static int rs_init_func_checksums(void)
{
	int ret;
	char *buffer;
	RS_CHECKSUM_TYPE *checksums;

	/*RS_LOG("ifc,0\n");*/
	buffer = rs_kmalloc((fcInvalid + 1) * sizeof(RS_CHECKSUM_TYPE) - 1);
	if (!buffer) {
		RS_LOG("ifc,1\n");
		return -ENOMEM;
	}

	checksums = (RS_CHECKSUM_TYPE *)round_up((uintptr_t)buffer, sizeof(RS_CHECKSUM_TYPE));

	memset(checksums, 0, (fcInvalid * sizeof(RS_CHECKSUM_TYPE)));

#if 1
	if ((!rs_calc_func_checksum_by_end(RS_FC_PARAMS(rs_verify_func), &checksums[fcCheckFunc]))
	#if defined(CONFIG_CONFIG_RS_PERMIT) || defined(CONFIG_CONFIG_RS_BYPASS)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_ok), &checksums[fcIsOpOK]))
	#if defined(CONFIG_CONFIG_RS_PERMIT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_permitted), &checksums[fcIsOpPermitted]))
	#endif
	#if defined(CONFIG_CONFIG_RS_BYPASS)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_bypassed), &checksums[fcIsOpBypassed]))
	#endif
	#if defined(CONFIG_CONFIG_RS_PERMIT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_permitted_no_fc), &checksums[fcIsOpPermittedNoFC]))
	#endif
	#if defined(CONFIG_CONFIG_RS_BYPASS)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_bypassed_no_fc), &checksums[fcIsOpBypassedNoFC]))
	#endif
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_op_permitted_by_file), &checksums[fcIsOpPermittedByFile]))
	#endif
	/*#if 0*/
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_op_verify), &checksums[fcDoOpVerify]))
	#if defined(CONFIG_MOUNT_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_mount_dir_need_check), &checksums[fcIsMountDirNeedCheck]))
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_mount_dev_need_check), &checksums[fcIsMountDevNeedCheck]))
	/*#if 0*/
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(is_mount_need_check), &checksums[fcIsMountNeedCheck]))
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_mount_check), &checksums[fcDoMountCheck]))
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS_PUB(do_mount), &checksums[fcDoMount]))
	#endif
	#if defined(CONFIG_INSMOD_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(insmod_check_path), &checksums[fcInsmodCheckPath]))
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_insmod_check), &checksums[fcDoInsmodCheck]))
	#endif
	#if defined(CONFIG_CHROOT_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_chroot_check), &checksums[fcDoChrootCheck]))
	#endif
	#if defined(CONFIG_ACCESS_DEV_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_access_dev_check), &checksums[fcDoAccessDevCheck]))
	#endif
	#if defined(CONFIG_SECURITY_SELINUX) && defined(CONFIG_DISABLE_SELINUX_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_disable_selinux_check), &checksums[fcDoDisableSelinuxCheck]))
	#endif
	#if defined(CONFIG_PTRACE_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_ptrace_check), &checksums[fcDoPtraceCheck]))
	#endif
	#if defined(CONFIG_SETUID_RESTRICT)
		/*&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_uid_check), &checksums[fcDoUidCheck]))*/
	#endif
	#if defined(CONFIG_SECURITY_SELINUX) && defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
		&& (!rs_calc_func_checksum_by_end(RS_FC_PARAMS(do_load_selinux_policy_check), &checksums[fcDoLoadSelinuxPolicyCheck]))
	#endif

	/*#endif*/
		) {
		/*RS_LOG("ifc,2\n");*/
		ret = set_vars_checksum(checksums, fcInvalid, 1);
		if (!ret) {
			/*RS_LOG("ifc,3\n");*/
			set_vars_global_flag(gfFuncChecksumsOK, 1);
		} else
			RS_LOG("ifc,4,%d\n", ret);
	} else {
		RS_LOG("ifc,5\n");
		ret = -EINVAL;
	}
#else
	ret = 0;
#endif

	rs_kfree(buffer);
	return ret;
}

#endif

#define no_argument		0
#define required_argument  1
#define optional_argument  2

struct option {
	/* name of long option */
	const char *name;
	/*
	 * one of no_argument, required_argument, and optional_argument:
	 * whether option takes an argument
	 */
	int has_arg;
	/* if not NULL, set *flag to val when option found */
	int *flag;
	/* if flag not NULL, value to set *flag to; else return value */
	int val;
};

#if 1

#define GNU_COMPATIBLE		/* Be more compatible, configure's use us! */

#if 1				/* we prefer to keep our getopt(3) */
#define	REPLACE_GETOPT		/* use this getopt as the system getopt(3) */
#endif

typedef struct {
	int	opterr;		/* if error message should be printed */
	int	optind;		/* index into parent argv vector */
	int	optopt;		/* character checked for validity */
	char	*optarg;		/* argument associated with option */
	int	optreset;		/* reset getopt */

	char *place; /* option letter processing */

	/* XXX: set optreset to 1 rather than these two */
	int nonopt_start; /* first non option argument (for permute) */
	int nonopt_end;   /* first option after non options (for permute) */
#ifdef GNU_COMPATIBLE
	int dash_prefix;
#endif
} parse_opt_params;

#ifdef REPLACE_GETOPT
#if 0
int	opterr = 1;		/* if error message should be printed */
int	optind = 1;		/* index into parent argv vector */
int	optopt = '?';		/* character checked for validity */
int	optreset;		/* reset getopt */
char	*optarg;		/* argument associated with option */
#endif
#endif

#define PRINT_ERROR	((params->opterr) && (*options != ':'))

#define FLAG_PERMUTE	0x01	/* permute non-options to the end of argv */
#define FLAG_ALLARGS	0x02	/* treat non-options as args to option "-1" */
#define FLAG_LONGONLY	0x04	/* operate as getopt_long_only */

/* return values */
#define	BADCH		((int)'?')
#define	BADARG		((*options == ':') ? (int)':' : (int)'?')
#define	INORDER		((int)1)

#define	EMSG		""

#ifdef GNU_COMPATIBLE
#define NO_PREFIX	(-1)
#define D_PREFIX	0
#define DD_PREFIX	1
#define W_PREFIX	2
#endif

#if 0
static int getopt_internal(int, char *const *, const char *,
				const struct option *, int *, int);
static int parse_long_options(char *const *, const char *,
				const struct option *, int *, int, int);
static int gcd(int, int);
static void permute_args(int, int, int, char *const *);
#endif

/*static char *place = EMSG;*/ /* option letter processing */

/* XXX: set optreset to 1 rather than these two */
/*static int nonopt_start = -1;*/ /* first non option argument (for permute) */
/*static int nonopt_end = -1;*/   /* first option after non options (for permute) */

/* Error messages */
#if 0
static const char recargchar[] = "option requires an argument -- %c";
static const char illoptchar[] = "illegal option -- %c"; /* From P1003.2 */
#ifdef GNU_COMPATIBLE
static int dash_prefix = NO_PREFIX;
static const char gnuoptchar[] = "invalid option -- %c";

static const char recargstring[] = "option `%s%s' requires an argument";
static const char ambig[] = "option `%s%.*s' is ambiguous";
static const char noarg[] = "option `%s%.*s' doesn't allow an argument";
static const char illoptstring[] = "unrecognized option `%s%s'";
#else
static const char recargstring[] = "option requires an argument -- %s";
static const char ambig[] = "ambiguous option -- %.*s";
static const char noarg[] = "option doesn't take an argument -- %.*s";
static const char illoptstring[] = "unknown option -- %s";
#endif

#else

#define recargchar "option requires an argument -- %c"
#define illoptchar "illegal option -- %c" /* From P1003.2 */
#ifdef GNU_COMPATIBLE
/*static int dash_prefix = NO_PREFIX;*/
#define gnuoptchar "invalid option -- %c"

#define recargstring "option `%s%s' requires an argument"
#define ambig "option `%s%.*s' is ambiguous"
#define noarg "option `%s%.*s' doesn't allow an argument"
#define illoptstring "unrecognized option `%s%s'"
#else
#define recargstring "option requires an argument -- %s"
#define ambig "ambiguous option -- %.*s"
#define noarg "option doesn't take an argument -- %.*s"
#define illoptstring "unknown option -- %s"
#endif

#endif

#if 0
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define _vwarnx RS_HIDE(_200)
#endif
void _vwarnx(fmt, ap) const char *fmt; _BSD_VA_LIST_ ap;
{
	(void)fprintf(stderr, "%s: ", getprogname());
	if (fmt != NULL)
		(void)vfprintf(stderr, fmt, ap);
	(void)fprintf(stderr, "\n");
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define warnx RS_HIDE(_201)
#endif
void warnx(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	_vwarnx(fmt, ap);
	va_end(ap);
}
#endif

#define warnx(fmt, ...) printk(fmt, __VA_ARGS__)

/*
 * Compute the greatest common divisor of a and b.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define gcd RS_HIDE(_202)
#endif
noused notrace static int
gcd(int a, int b)
{
	int c;

	c = a % b;
	while (c != 0) {
		a = b;
		b = c;
		c = a % b;
	}

	return b;
}

/*
 * Exchange the block from nonopt_start to nonopt_end with the block
 * from nonopt_end to opt_end (keeping the same order of arguments
 * in each block).
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define permute_args RS_HIDE(_203)
#endif
noused notrace static void
permute_args(int panonopt_start, int panonopt_end, int opt_end,
	char *const *nargv)
{
	int cstart, cyclelen, i, j, ncycle, nnonopts, nopts, pos;
	char *swap;

	/*
	 * compute lengths of blocks and number and size of cycles
	 */
	nnonopts = panonopt_end - panonopt_start;
	nopts = opt_end - panonopt_end;
	ncycle = gcd(nnonopts, nopts);
	cyclelen = (opt_end - panonopt_start) / ncycle;

	for (i = 0; i < ncycle; i++) {
		cstart = panonopt_end+i;
		pos = cstart;
		for (j = 0; j < cyclelen; j++) {
			if (pos >= panonopt_end)
				pos -= nnonopts;
			else
				pos += nopts;
			swap = nargv[pos];
			/* LINTED const cast */
			((char **) nargv)[pos] = nargv[cstart];
			/* LINTED const cast */
			((char **)nargv)[cstart] = swap;
		}
	}
}

/*
 * parse_long_options --
 *	Parse long options in argc/argv argument vector.
 * Returns -1 if short_too is set and the option does not match long_options.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define parse_long_options RS_HIDE(_204)
#endif
noused notrace static int
parse_long_options(parse_opt_params * params, char *const *nargv, const char *options,
	const struct option *long_options, int *idx, int short_too, int flags)
{
	char *current_argv, *has_equal;
#ifdef GNU_COMPATIBLE
	char *current_dash;
#endif
	size_t current_argv_len;
	int i, match, exact_match, second_partial_match;

	current_argv = params->place;
#ifdef GNU_COMPATIBLE
	switch (params->dash_prefix) {
	case D_PREFIX:
		current_dash = "-";
		break;
	case DD_PREFIX:
		current_dash = "--";
		break;
	case W_PREFIX:
		current_dash = "-W ";
		break;
	default:
		current_dash = "";
		break;
	}
#endif
	match = -1;
	exact_match = 0;
	second_partial_match = 0;

	params->optind++;

	has_equal = _strchr(current_argv, '=');
	if (has_equal != NULL) {
		/* argument found (--option=arg) */
		current_argv_len = has_equal - current_argv;
		has_equal++;
	} else
		current_argv_len = strlen(current_argv);

	for (i = 0; long_options[i].name; i++) {
		/* find matching long option */
		if (strncmp(current_argv, long_options[i].name,
			current_argv_len))
			continue;

		if (strlen(long_options[i].name) == current_argv_len) {
			/* exact match */
			match = i;
			exact_match = 1;
			break;
		}
		/*
		 * If this is a known short option, don't allow
		 * a partial match of a single character.
		 */
		if (short_too && current_argv_len == 1)
			continue;

		if (match == -1)		/* first partial match */
			match = i;
		else if ((flags & FLAG_LONGONLY) ||
			long_options[i].has_arg !=
				long_options[match].has_arg ||
			long_options[i].flag != long_options[match].flag ||
			long_options[i].val != long_options[match].val)
			second_partial_match = 1;
	}
	if (!exact_match && second_partial_match) {
		/* ambiguous abbreviation */
		if (PRINT_ERROR)
			warnx(ambig,
#ifdef GNU_COMPATIBLE
				current_dash,
#endif
				(int)current_argv_len,
				current_argv);
		params->optopt = 0;
		return BADCH;
	}
	if (match != -1) {		/* option found */
		if (long_options[match].has_arg == no_argument
			&& has_equal) {
			if (PRINT_ERROR)
				warnx(noarg,
#ifdef GNU_COMPATIBLE
					current_dash,
#endif
					(int)current_argv_len,
					current_argv);
			/*
			 * XXX: GNU sets optopt to val regardless of flag
			 */
			if (long_options[match].flag == NULL)
				params->optopt = long_options[match].val;
			else
				params->optopt = 0;
#ifdef GNU_COMPATIBLE
			return BADCH;
#else
			return BADARG;
#endif
		}
		if (long_options[match].has_arg == required_argument ||
			long_options[match].has_arg == optional_argument) {
			if (has_equal)
				params->optarg = has_equal;
			else if (long_options[match].has_arg ==
				required_argument) {
				/*
				 * optional argument doesn't use next nargv
				 */
				params->optarg = nargv[params->optind++];
			}
		}
		if ((long_options[match].has_arg == required_argument)
			&& (params->optarg == NULL)) {
			/*
			 * Missing argument; leading ':' indicates no error
			 * should be generated.
			 */
			if (PRINT_ERROR)
				warnx(recargstring,
#ifdef GNU_COMPATIBLE
					current_dash,
#endif
					current_argv);
			/*
			 * XXX: GNU sets optopt to val regardless of flag
			 */
			if (long_options[match].flag == NULL)
				params->optopt = long_options[match].val;
			else
				params->optopt = 0;
			--params->optind;
			return BADARG;
		}
	} else {			/* unknown option */
		if (short_too) {
			--params->optind;
			return (-1);
		}
		if (PRINT_ERROR)
			warnx(illoptstring,
#ifdef GNU_COMPATIBLE
				current_dash,
#endif
				current_argv);
		params->optopt = 0;
		return BADCH;
	}
	if (idx)
		*idx = match;
	if (long_options[match].flag) {
		*long_options[match].flag = long_options[match].val;
		return 0;
	} else
		return long_options[match].val;
}

/*
 * getopt_internal --
 *	Parse argc/argv argument vector.  Called by user level routines.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_internal RS_HIDE(_205)
#endif
noused notrace static int
getopt_internal(parse_opt_params * params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx, int flags)
{
	char *oli;				/* option letter list index */
	int optchar, short_too;
	int posixly_correct;	/* no static, can be changed on the fly */
	int flag;

	if (options == NULL)
		return (-1);

	/*
	 * Disable GNU extensions if POSIXLY_CORRECT is set or options
	 * string begins with a '+'.
	 */
	posixly_correct = 0; /*(getenv("POSIXLY_CORRECT") != NULL);*/
#ifdef GNU_COMPATIBLE
	if (*options == '-')
		flags |= FLAG_ALLARGS;
	else if (posixly_correct || *options == '+')
		flags &= ~FLAG_PERMUTE;
#else
	if (posixly_correct || *options == '+')
		flags &= ~FLAG_PERMUTE;
	else if (*options == '-')
		flags |= FLAG_ALLARGS;
#endif
	if (*options == '+' || *options == '-')
		options++;

	/*
	 * XXX Some GNU programs (like cvs) set optind to 0 instead of
	 * XXX using optreset.  Work around this braindamage.
	 */
	if (params->optind == 0)
		params->optind = params->optreset = 1;

	params->optarg = NULL;
	if (params->optreset)
		params->nonopt_start = params->nonopt_end = -1;
start:
	if (params->optreset || !*params->place) {		/* update scanning pointer */
		params->optreset = 0;
		if (params->optind >= nargc) {		/* end of argument vector */
			params->place = EMSG;
			if (params->nonopt_end != -1) {
				/* do permutation, if we have to */
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->optind -= params->nonopt_end - params->nonopt_start;
			} else if (params->nonopt_start != -1) {
				/*
				 * If we skipped non-options, set optind
				 * to the first of them.
				 */
				params->optind = params->nonopt_start;
			}
			params->nonopt_start = params->nonopt_end = -1;
			return (-1);
		}
		params->place = nargv[params->optind];
		if (*params->place != '-' ||
#ifdef GNU_COMPATIBLE
			params->place[1] == '\0') {
#else
			(params->place[1] == '\0' && _strchr(options, '-') == NULL)) {
#endif
			params->place = EMSG;		/* found non-option */
			if (flags & FLAG_ALLARGS) {
				/*
				 * GNU extension:
				 * return non-option as argument to option 1
				 */
				params->optarg = nargv[params->optind++];
				return INORDER;
			}
			if (!(flags & FLAG_PERMUTE)) {
				/*
				 * If no permutation wanted, stop parsing
				 * at first non-option.
				 */
				return (-1);
			}
			/* do permutation */
			if (params->nonopt_start == -1)
				params->nonopt_start = params->optind;
			else if (params->nonopt_end != -1) {
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->nonopt_start = params->optind -
					(params->nonopt_end - params->nonopt_start);
				params->nonopt_end = -1;
			}
			params->optind++;
			/* process next argument */
			goto start;
		}
		if (params->nonopt_start != -1 && params->nonopt_end == -1)
			params->nonopt_end = params->optind;

		/*
		 * If we have "-" do nothing, if "--" we are done.
		 */
		if (params->place[1] != '\0' && *++params->place == '-' && params->place[1] == '\0') {
			params->optind++;
			params->place = EMSG;
			/*
			 * We found an option (--), so if we skipped
			 * non-options, we have to permute.
			 */
			if (params->nonopt_end != -1) {
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->optind -= params->nonopt_end - params->nonopt_start;
			}
			params->nonopt_start = params->nonopt_end = -1;
			return (-1);
		}
	}

	/*
	 * Check long options if:
	 *  1) we were passed some
	 *  2) the arg is not just "-"
	 *  3) either the arg starts with -- we are getopt_long_only()
	 */
	if (long_options != NULL && params->place != nargv[params->optind] &&
		(*params->place == '-' || (flags & FLAG_LONGONLY))) {
		short_too = 0;
#ifdef GNU_COMPATIBLE
		params->dash_prefix = D_PREFIX;
#endif
		if (*params->place == '-') {
			params->place++;		/* --foo long option */
#ifdef GNU_COMPATIBLE
			params->dash_prefix = DD_PREFIX;
#endif
		} else if (*params->place != ':' && _strchr(options, *params->place) != NULL)
			short_too = 1;		/* could be short option too */

		optchar = parse_long_options(params, nargv, options, long_options,
			idx, short_too, flags);
		if (optchar != -1) {
			params->place = EMSG;
			return optchar;
		}
	}

	flag = 0;
	optchar = (int)*params->place++;
	if (optchar == (int)':')
		flag = 1;
	else if (optchar == (int)'-' && *params->place != '\0')
		flag = 1;
	else {
		oli = _strchr(options, optchar);
		if (oli == NULL)
			flag = 1;
	}

	if (flag) {
		/*
		 * If the user specified "-" and  '-' isn't listed in
		 * options, return -1 (non-option) as per POSIX.
		 * Otherwise, it is an unknown option character (or ':').
		 */
		if (optchar == (int)'-' && *params->place == '\0')
			return (-1);
		if (!*params->place)
			++params->optind;
#ifdef GNU_COMPATIBLE
		if (PRINT_ERROR)
			warnx(posixly_correct ? illoptchar : gnuoptchar,
				optchar);
#else
		if (PRINT_ERROR)
			warnx(illoptchar, optchar);
#endif
		params->optopt = optchar;
		return BADCH;
	}
	if (long_options != NULL && optchar == 'W' && oli[1] == ';') {
		/* -W long-option */
		if (*params->place)			/* no space */
			/* NOTHING */;
		else if (++params->optind >= nargc) {	/* no arg */
			params->place = EMSG;
			if (PRINT_ERROR)
				warnx(recargchar, optchar);
			params->optopt = optchar;
			return BADARG;
		} else				/* white space */
			params->place = nargv[params->optind];
#ifdef GNU_COMPATIBLE
		params->dash_prefix = W_PREFIX;
#endif
		optchar = parse_long_options(params, nargv, options, long_options,
			idx, 0, flags);
		params->place = EMSG;
		return optchar;
	}
	if (*++oli != ':') {			/* doesn't take argument */
		if (!*params->place)
			++params->optind;
	} else {				/* takes (optional) argument */
		params->optarg = NULL;
		if (*params->place)			/* no white space */
			params->optarg = params->place;
		else if (oli[1] != ':') {	/* arg not optional */
			if (++params->optind >= nargc) {	/* no arg */
				params->place = EMSG;
				if (PRINT_ERROR)
					warnx(recargchar, optchar);
				params->optopt = optchar;
				return BADARG;
			} else
				params->optarg = nargv[params->optind];
		}
		params->place = EMSG;
		++params->optind;
	}
	/* dump back option letter */
	return optchar;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_parse_opt_params RS_HIDE(_206)
#endif
void init_parse_opt_params(parse_opt_params *params)
{
	if (!params)
		return;

	params->opterr = 1;
	params->optind = 1;
	params->optopt = '?';
	params->optarg = NULL;
	params->optreset = 0;

	params->place = EMSG;
	params->nonopt_start = -1;
	params->nonopt_end = -1;
#ifdef GNU_COMPATIBLE
	params->dash_prefix = NO_PREFIX;
#endif
}

#ifdef REPLACE_GETOPT
/*
 * getopt --
 *	Parse argc/argv argument vector.
 *
 * [eventually this will replace the BSD getopt]
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt RS_HIDE(_207)
#endif
int
getopt(parse_opt_params *params, int nargc, char *const *nargv, const char *options)
{

	/*
	 * We don't pass FLAG_PERMUTE to getopt_internal() since
	 * the BSD getopt(3) (unlike GNU) has never done this.
	 *
	 * Furthermore, since many privileged programs call getopt()
	 * before dropping privileges it makes sense to keep things
	 * as simple (and bug-free) as possible.
	 */
	return getopt_internal(params, nargc, nargv, options, NULL, NULL, 0);
}
#endif /* REPLACE_GETOPT */

/*
 * getopt_long --
 *	Parse argc/argv argument vector.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_long RS_HIDE(_208)
#endif
int
getopt_long(parse_opt_params *params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx)
{

	return (getopt_internal(params, nargc, nargv, options, long_options, idx,
		FLAG_PERMUTE));
}

/*
 * getopt_long_only --
 *	Parse argc/argv argument vector.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_long_only RS_HIDE(_209)
#endif
int
getopt_long_only(parse_opt_params *params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx)
{

	return (getopt_internal(params, nargc, nargv, options, long_options, idx,
		FLAG_PERMUTE|FLAG_LONGONLY));
}
#else
typedef struct {
	int	opterr;		/* if error message should be printed */
	int	optind;		/* index into parent argv vector */
	int	optopt;		/* character checked for validity */
	char	*optarg;		/* argument associated with option */
	int	optreset;		/* reset getopt */

	char *place; /* option letter processing */

	/* XXX: set optreset to 1 rather than these two */
	int nonopt_start; /* first non option argument (for permute) */
	int nonopt_end;   /* first option after non options (for permute) */
} parse_opt_params;

#define	REPLACE_GETOPT		/* use this getopt as the system getopt(3) */

#ifdef REPLACE_GETOPT
#if 0
int	opterr = 1;		/* if error message should be printed */
int	optind = 1;		/* index into parent argv vector */
int	optopt = '?';		/* character checked for validity */
char	*optarg;		/* argument associated with option */
#endif
#endif
#if 0
int	optreset;		/* reset getopt */
#endif

#define PRINT_ERROR	((params->opterr) && (*options != ':'))

#define FLAG_PERMUTE	0x01	/* permute non-options to the end of argv */
#define FLAG_ALLARGS	0x02	/* treat non-options as args to option "-1" */
#define FLAG_LONGONLY	0x04	/* operate as getopt_long_only */

/* return values */
#define	BADCH		((int)'?')
#define	BADARG		((*options == ':') ? (int)':' : (int)'?')
#define	INORDER		((int)1)

#define	EMSG		""

#if 0
noused notrace static int getopt_internal(int, char *const *, const char *,
			const struct option *, int *, int);
noused notrace static int parse_long_options(char *const *, const char *,
				const struct option *, int *, int);
noused notrace static int gcd(int, int);
noused notrace static void permute_args(int, int, int, char *const *);
#endif

#if 0
noused static char *place = EMSG; /* option letter processing */

/* XXX: set optreset to 1 rather than these two */
noused static int nonopt_start = -1; /* first non option argument (for permute) */
noused static int nonopt_end = -1;   /* first option after non options (for permute) */
#endif

/* Error messages */
#if 0
noused static const char recargchar[] = "option requires an argument -- %c";
noused static const char recargstring[] = "option requires an argument -- %s";
noused static const char ambig[] = "ambiguous option -- %.*s";
noused static const char noarg[] = "option doesn't take an argument -- %.*s";
noused static const char illoptchar[] = "unknown option -- %c";
noused static const char illoptstring[] = "unknown option -- %s";
#else
#define recargchar "option requires an argument -- %c"
#define recargstring "option requires an argument -- %s"
#define ambig "ambiguous option -- %.*s"
#define noarg "option doesn't take an argument -- %.*s"
#define illoptchar "unknown option -- %c"
#define illoptstring "unknown option -- %s"
#endif

/*
 * Compute the greatest common divisor of a and b.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define gcd RS_HIDE(_220)
#endif
noused notrace static int
gcd(int a, int b)
{
	int c;

	c = a % b;
	while (c != 0) {
		a = b;
		b = c;
		c = a % b;
	}

	return b;
}

/*
 * Exchange the block from nonopt_start to nonopt_end with the block
 * from nonopt_end to opt_end (keeping the same order of arguments
 * in each block).
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define permute_args RS_HIDE(_221)
#endif
noused notrace static void
permute_args(int panonopt_start, int panonopt_end, int opt_end,
	char *const *nargv)
{
	int cstart, cyclelen, i, j, ncycle, nnonopts, nopts, pos;
	char *swap;

	/*
	 * compute lengths of blocks and number and size of cycles
	 */
	nnonopts = panonopt_end - panonopt_start;
	nopts = opt_end - panonopt_end;
	ncycle = gcd(nnonopts, nopts);
	cyclelen = (opt_end - panonopt_start) / ncycle;

	for (i = 0; i < ncycle; i++) {
		cstart = panonopt_end+i;
		pos = cstart;
		for (j = 0; j < cyclelen; j++) {
			if (pos >= panonopt_end)
				pos -= nnonopts;
			else
				pos += nopts;
			swap = nargv[pos];
			/* LINTED const cast */
			((char **) nargv)[pos] = nargv[cstart];
			/* LINTED const cast */
			((char **)nargv)[cstart] = swap;
		}
	}
}

/*
 * parse_long_options --
 *	Parse long options in argc/argv argument vector.
 * Returns -1 if short_too is set and the option does not match long_options.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define parse_long_options RS_HIDE(_222)
#endif
noused notrace static int
parse_long_options(parse_opt_params *params, char *const *nargv, const char *options,
	const struct option *long_options, int *idx, int short_too)
{
	char *current_argv, *has_equal;
	size_t current_argv_len;
	int i, match;

	current_argv = params->place;
	match = -1;

	params->optind++;

	has_equal = _strchr(current_argv, '=');
	if (has_equal != NULL) {
		/* argument found (--option=arg) */
		current_argv_len = has_equal - current_argv;
		has_equal++;
	} else
		current_argv_len = strlen(current_argv);

	for (i = 0; long_options[i].name; i++) {
		/* find matching long option */
		if (strncmp(current_argv, long_options[i].name,
			current_argv_len))
			continue;

		if (strlen(long_options[i].name) == current_argv_len) {
			/* exact match */
			match = i;
			break;
		}
		/*
		 * If this is a known short option, don't allow
		 * a partial match of a single character.
		 */
		if (short_too && current_argv_len == 1)
			continue;

		if (match == -1)	/* partial match */
			match = i;
		else {
			/* ambiguous abbreviation */
			if (PRINT_ERROR)
				printk(KERN_ERR ambig, (int)current_argv_len,
						current_argv);
			params->optopt = 0;
			return BADCH;
		}
	}
	if (match != -1) {		/* option found */
		if (long_options[match].has_arg == no_argument
			&& has_equal) {
			if (PRINT_ERROR)
				printk(KERN_ERR noarg, (int)current_argv_len,
						current_argv);
			/*
			 * XXX: GNU sets optopt to val regardless of flag
			 */
			if (long_options[match].flag == NULL)
				params->optopt = long_options[match].val;
			else
				params->optopt = 0;
			return BADARG;
		}
		if (long_options[match].has_arg == required_argument ||
			long_options[match].has_arg == optional_argument) {
			if (has_equal)
				params->optarg = has_equal;
			else if (long_options[match].has_arg ==
				required_argument) {
				/*
				 * optional argument doesn't use next nargv
				 */
				params->optarg = nargv[params->optind++];
			}
		}
		if ((long_options[match].has_arg == required_argument)
			&& (params->optarg == NULL)) {
			/*
			 * Missing argument; leading ':' indicates no error
			 * should be generated.
			 */
			if (PRINT_ERROR)
				printk(KERN_ERR recargstring,
						current_argv);
			/*
			 * XXX: GNU sets optopt to val regardless of flag
			 */
			if (long_options[match].flag == NULL)
				params->optopt = long_options[match].val;
			else
				params->optopt = 0;
			--params->optind;
			return BADARG;
		}
	} else {			/* unknown option */
		if (short_too) {
			--params->optind;
			return (-1);
		}
		if (PRINT_ERROR)
			printk(KERN_ERR illoptstring, current_argv);
		params->optopt = 0;
		return BADCH;
	}
	if (idx)
		*idx = match;
	if (long_options[match].flag) {
		*long_options[match].flag = long_options[match].val;
		return 0;
	} else
		return long_options[match].val;
}

/*
 * getopt_internal --
 *	Parse argc/argv argument vector.  Called by user level routines.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_internal RS_HIDE(_223)
#endif
noused notrace static int
getopt_internal(parse_opt_params *params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx, int flags)
{
	char *oli;				/* option letter list index */
	int optchar, short_too;
	static int posixly_correct = -1;
	int flag;

	if (options == NULL)
		return (-1);

	/*
	 * Disable GNU extensions if POSIXLY_CORRECT is set or options
	 * string begins with a '+'.
	 */
	if (posixly_correct == -1)
		posixly_correct = 0; /*(getenv("POSIXLY_CORRECT") != NULL);*/
	if (posixly_correct || *options == '+')
		flags &= ~FLAG_PERMUTE;
	else if (*options == '-')
		flags |= FLAG_ALLARGS;
	if (*options == '+' || *options == '-')
		options++;

	/*
	 * XXX Some GNU programs (like cvs) set optind to 0 instead of
	 * XXX using optreset.  Work around this braindamage.
	 */
	if (params->optind == 0)
		params->optind = params->optreset = 1;

	params->optarg = NULL;
	if (params->optreset)
		params->nonopt_start = params->nonopt_end = -1;
start:
	if (params->optreset || !*params->place) {		/* update scanning pointer */
		params->optreset = 0;
		if (params->optind >= nargc) {		/* end of argument vector */
			params->place = EMSG;
			if (params->nonopt_end != -1) {
				/* do permutation, if we have to */
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->optind -= params->nonopt_end - params->nonopt_start;
			} else if (params->nonopt_start != -1) {
				/*
				 * If we skipped non-options, set optind
				 * to the first of them.
				 */
				params->optind = params->nonopt_start;
			}
			params->nonopt_start = params->nonopt_end = -1;
			return (-1);
		}
		flag = 0;
		params->place = nargv[params->optind];
		if (*params->place != '-' ||
			(params->place[1] == '\0' && _strchr(options, '-') == NULL)) {
			params->place = EMSG;		/* found non-option */
			if (flags & FLAG_ALLARGS) {
				/*
				 * GNU extension:
				 * return non-option as argument to option 1
				 */
				params->optarg = nargv[params->optind++];
				return INORDER;
			}
			if (!(flags & FLAG_PERMUTE)) {
				/*
				 * If no permutation wanted, stop parsing
				 * at first non-option.
				 */
				return (-1);
			}
			/* do permutation */
			if (params->nonopt_start == -1)
				params->nonopt_start = params->optind;
			else if (params->nonopt_end != -1) {
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->nonopt_start = params->optind -
					(params->nonopt_end - params->nonopt_start);
				params->nonopt_end = -1;
			}
			params->optind++;
			/* process next argument */
			goto start;
		}
		if (params->nonopt_start != -1 && params->nonopt_end == -1)
			params->nonopt_end = params->optind;

		/*
		 * If we have "-" do nothing, if "--" we are done.
		 */
		if (params->place[1] != '\0' && *++params->place == '-' && params->place[1] == '\0') {
			params->optind++;
			params->place = EMSG;
			/*
			 * We found an option (--), so if we skipped
			 * non-options, we have to permute.
			 */
			if (params->nonopt_end != -1) {
				permute_args(params->nonopt_start, params->nonopt_end,
					params->optind, nargv);
				params->optind -= params->nonopt_end - params->nonopt_start;
			}
			params->nonopt_start = params->nonopt_end = -1;
			return (-1);
		}
	}

	/*
	 * Check long options if:
	 *  1) we were passed some
	 *  2) the arg is not just "-"
	 *  3) either the arg starts with -- we are getopt_long_only()
	 */
	if (long_options != NULL && params->place != nargv[params->optind] &&
		(*params->place == '-' || (flags & FLAG_LONGONLY))) {
		short_too = 0;
		if (*params->place == '-')
			params->place++;		/* --foo long option */
		else if (*params->place != ':' && _strchr(options, *params->place) != NULL)
			short_too = 1;		/* could be short option too */

		optchar = parse_long_options(params, nargv, options, long_options,
			idx, short_too);
		if (optchar != -1) {
			params->place = EMSG;
			return optchar;
		}
	}

	flag = 0;
	optchar = (int)*params->place++;
	if (optchar == (int)':')
		flag = 1;
	else if (optchar == (int)'-' && *params->place != '\0')
		flag = 1;
	else {
		oli = _strchr(options, optchar);
		if (oli == NULL)
			flag = 1;
	}

	if (flag) {
		/*
		 * If the user specified "-" and  '-' isn't listed in
		 * options, return -1 (non-option) as per POSIX.
		 * Otherwise, it is an unknown option character (or ':').
		 */
		if (optchar == (int)'-' && *params->place == '\0')
			return (-1);
		if (!*params->place)
			++params->optind;
		if (PRINT_ERROR)
			printk(KERN_ERR illoptchar, optchar);
		params->optopt = optchar;
		return BADCH;
	}
	if (long_options != NULL && optchar == 'W' && oli[1] == ';') {
		/* -W long-option */
		if (*params->place)			/* no space */
			/* NOTHING */;
		else if (++params->optind >= nargc) {	/* no arg */
			params->place = EMSG;
			if (PRINT_ERROR)
				printk(KERN_ERR recargchar, optchar);
			params->optopt = optchar;
			return BADARG;
		} else				/* white space */
			params->place = nargv[params->optind];
		optchar = parse_long_options(params, nargv, options, long_options,
			idx, 0);
		params->place = EMSG;
		return optchar;
	}
	if (*++oli != ':') {			/* doesn't take argument */
		if (!*params->place)
			++params->optind;
	} else {				/* takes (optional) argument */
		params->optarg = NULL;
		if (*params->place)			/* no white space */
			params->optarg = params->place;
		/* XXX: disable test for :: if PC? (GNU doesn't) */
		else if (oli[1] != ':') {	/* arg not optional */
			if (++params->optind >= nargc) {	/* no arg */
				params->place = EMSG;
				if (PRINT_ERROR)
					printk(KERN_ERR recargchar, optchar);
				params->optopt = optchar;
				return BADARG;
			} else
				params->optarg = nargv[params->optind];
		} else if (!(flags & FLAG_PERMUTE)) {
			/*
			 * If permutation is disabled, we can accept an
			 * optional arg separated by whitespace so long
			 * as it does not start with a dash (-).
			 */
			if (params->optind + 1 < nargc && *nargv[params->optind + 1] != '-')
				params->optarg = nargv[++params->optind];
		}
		params->place = EMSG;
		++params->optind;
	}
	/* dump back option letter */
	return optchar;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define init_parse_opt_params RS_HIDE(_224)
#endif
void init_parse_opt_params(parse_opt_params *params)
{
	if (!params)
		return;

	params->opterr = 1;
	params->optind = 1;
	params->optopt = '?';
	params->optarg = NULL;
	params->optreset = 0;

	params->place = EMSG;
	params->nonopt_start = -1;
	params->nonopt_end = -1;
}

#ifdef REPLACE_GETOPT
/*
 * getopt --
 *	Parse argc/argv argument vector.
 *
 * [eventually this will replace the BSD getopt]
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt RS_HIDE(_225)
#endif
int
getopt(parse_opt_params *params, int nargc, char *const *nargv, const char *options)
{

	/*
	 * We don't pass FLAG_PERMUTE to getopt_internal() since
	 * the BSD getopt(3) (unlike GNU) has never done this.
	 *
	 * Furthermore, since many privileged programs call getopt()
	 * before dropping privileges it makes sense to keep things
	 * as simple (and bug-free) as possible.
	 */
	return getopt_internal(params, nargc, nargv, options, NULL, NULL, 0);
}
#endif /* REPLACE_GETOPT */

/*
 * getopt_long --
 *	Parse argc/argv argument vector.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_long RS_HIDE(_226)
#endif
int
getopt_long(parse_opt_params *params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx)
{

	return (getopt_internal(params, nargc, nargv, options, long_options, idx,
		FLAG_PERMUTE));
}

/*
 * getopt_long_only --
 *	Parse argc/argv argument vector.
 */
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define getopt_long_only RS_HIDE(_227)
#endif
int
getopt_long_only(parse_opt_params *params, int nargc, char *const *nargv, const char *options,
	const struct option *long_options, int *idx)
{

	return (getopt_internal(params, nargc, nargv, options, long_options, idx,
		FLAG_PERMUTE|FLAG_LONGONLY));
}
#endif


#if 1

struct mount_opts {
	int str_seed; /*const char str[8];*/
	unsigned long rwmask;
	unsigned long rwset;
	unsigned long rwnoset;
};


/*
 * These options define the function of "mount(2)".
 */
#define MS_TYPE	(MS_REMOUNT|MS_BIND|MS_MOVE)


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define mount_options RS_HIDE(_B)
#endif
noused static const struct mount_opts mount_options[] = {
	/* name		mask		set		noset		*/
/*	{ "async",	MS_SYNCHRONOUS,	0,		MS_SYNCHRONOUS	},*/
/*	{ "atime",	MS_NOATIME,	0,		MS_NOATIME	},*/
/*	{ "bind",	MS_TYPE,	MS_BIND,	0,		},*/
/*	{ "dev",	MS_NODEV,	0,		MS_NODEV	},*/
/*	{ "diratime",	MS_NODIRATIME,	0,		MS_NODIRATIME	},*/
/*	{ "dirsync",	MS_DIRSYNC,	MS_DIRSYNC,	0		},*/
/*	{ "exec",	MS_NOEXEC,	0,		MS_NOEXEC	},*/
/*	{ "move",	MS_TYPE,	MS_MOVE,	0		},*/
	{ csPrivate,	MS_PRIVATE,	MS_PRIVATE,	0		}, /*"private"*/
/*	{ "rec",	MS_REC,		MS_REC,		0		},*/
/*	{ "recurse",	MS_REC,		MS_REC,		0		},*/
	{ csRemount,	MS_TYPE, MS_REMOUNT, 0}, /*"remount"*/
	{ csRO, MS_RDONLY, MS_RDONLY, 0 }, /*"ro"*/
	{ csRecPrivate,	MS_REC | MS_PRIVATE,	MS_REC | MS_PRIVATE,	0		}, /*"rprivate"*/
	{ csRecShared,	MS_REC | MS_SHARED,	MS_REC | MS_SHARED,	0		}, /*"rshared"*/
	{ csRecSlave,	MS_REC | MS_SLAVE,	MS_REC | MS_SLAVE,	0		}, /*"rslave"*/
	{ csRecUnbindable,	MS_REC | MS_UNBINDABLE,	MS_REC | MS_UNBINDABLE,	0		}, /*"runbindable"*/
	{ csRW, MS_RDONLY, 0,  MS_RDONLY }, /*"rw"*/
	{ csShared,	MS_SHARED,	MS_SHARED,	0		}, /*"shared"*/
	{ csSlave,	MS_SLAVE,	MS_SLAVE,	0		}, /*"slave"*/
/*	{ "suid",	MS_NOSUID,	0,		MS_NOSUID	},*/
/*	{ "sync",	MS_SYNCHRONOUS,	MS_SYNCHRONOUS,	0		},*/
	{ csUnbindable,	MS_UNBINDABLE,	MS_UNBINDABLE,	0		}, /*"unbindable"*/
/*	{ "verbose",	MS_VERBOSE,	MS_VERBOSE,	0		},*/
};

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define parse_mount_options RS_HIDE(_230)
#endif
noused notrace static unsigned long parse_mount_options(char *arg, unsigned long rwflag)
{
	char *s;

	while ((s = strsep(&arg, ",")) != NULL) {
		/*char *opt = s;*/
		unsigned int i;
		int res, no = s[0] == 'n' && s[1] == 'o';

		if (no)
			s += 2;

		if (STRNCMP(s, check_strings[csLoopEqu].str, check_strings[csLoopEqu].str_len, csLoopEqu) == 0) {
			continue;
		}

		if (STRCMP(s, check_strings[csLoop].str, csLoop) == 0) {
			continue;
		}

		for (i = 0, res = 1; i < ARRAY_SIZE(mount_options); i++) {
			res = STRNCMP(s, check_strings[mount_options[i].str_seed].str,
						check_strings[mount_options[i].str_seed].str_len, mount_options[i].str_seed);
			/*res = strncmp(s, mount_options[i].str, sizeof(mount_options[i].str));*/

			if (res == 0) {
				rwflag &= ~mount_options[i].rwmask;

				if (no)
					rwflag |= mount_options[i].rwnoset;
				else
					rwflag |= mount_options[i].rwset;
			} else if (res < 0) { /*注意mount_options[]是按 name 从小到大顺序排的*/
				break;
			}
		}

		/*if (res != 0 && s[0])
			add_extra_option(extra, opt);
		*/
	}

	return rwflag;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_mount_execve_arg_check RS_HIDE(_231)
#endif
noused notrace int do_mount_execve_arg_check(int argc, char *const *argv)
{
	/*pass: return 0, otherwise: not 0*/
#if defined(CONFIG_MOUNT_RESTRICT)
	int ret = 0;
	int local_optind;

	unsigned long rwflag; /* = MS_VERBOSE;*/

	if (argc < 2) {
		return ret;
	}

	rwflag = 0;

	{
		int c;

		parse_opt_params params;

		init_parse_opt_params(&params);

		do {
			c = getopt(&params, argc, argv, "o:rw");

			if (c == -1/*EOF*/) {
				break;
			}

			switch (c) {
			case 'o':
				rwflag = parse_mount_options(params.optarg, rwflag);
				break;
			case 'r':
				rwflag |= MS_RDONLY;
				break;
			/*case 't':
				type = optarg;
				break;
			*/
			case 'w':
				rwflag &= (unsigned long)(~MS_RDONLY);
				break;
			}
		} while (1);

		local_optind = params.optind;
	}

	if (!(rwflag & MS_RDONLY) && ((rwflag & MS_REMOUNT)
		|| !(rwflag & (/*MS_BIND | MS_MOVE | */ (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE)))))
	/*if (!(rwflag & (MS_RDONLY | (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))))*/
	{
		/*rw*/
		char *pathbuf = NULL;
		char *dev_name = NULL;
		char *dir_name = NULL;
		int mount_type = 0;
		int need_check = 0;

		if ((local_optind + 2) == argc) {
			noused char *real_dir_name;

			dev_name = argv[local_optind];
			dir_name = argv[local_optind + 1];

			real_dir_name = try_get_real_filename(dir_name, &pathbuf);
			if (real_dir_name) {
				dir_name = real_dir_name;
			}
			need_check = (is_mount_need_check(dev_name, dir_name, &mount_type)
						&& (!rs_verify_func(RS_FC_PARAMS(is_mount_need_check), fcIsMountNeedCheck)));
		} else if ((local_optind + 1 == argc) && (rwflag & MS_REMOUNT)) {
			char *arg = argv[local_optind];

			char *real_arg = try_get_real_filename(arg, &pathbuf);
			if (real_arg) {
				arg = real_arg;
			}
			need_check = (is_mount_dir_need_check(arg, &mount_type)
						&& (!rs_verify_func(RS_FC_PARAMS(is_mount_dir_need_check), fcIsMountDirNeedCheck)))
						|| (is_mount_dev_need_check(arg, &mount_type)
						&& (!rs_verify_func(RS_FC_PARAMS(is_mount_dev_need_check), fcIsMountDevNeedCheck))
						);
		}

		if (need_check) {
			if (!pathbuf) {
				pathbuf = (char *)RS_GET_BUFFER();
			}

			if (pathbuf) {
				op_check_params params;
				mount_verify_data verify_data;
				struct task_struct *curr = current;

				verify_data.mount_type = mount_type;
				verify_data.mount_flags = 0;
				verify_data.task_infos_len = 0;
				verify_data.task_infos = NULL;

				params.current_task = curr;
				params.path = "";
				params.buffer = pathbuf;
				params.buffer_size = RS_BUFFER_SIZE;
				params.op_type = ovMount;
				params.op_data = (ssize_t)&verify_data;
				params.is_query = 0;
				params.head_ptr = NULL;

				if (!get_op_check_params_upper_tasks(&params, params.current_task)) {
					ret = internal_mksh_check_func(&params, 0);

					free_op_check_params_upper_tasks(&params);

					if (ret != 0) {
						RS_LOG("Restricted mount 2. PID = %d(%s)\n", curr->pid, curr->comm);

					#if defined(PASS_MOUNT_RESTRICT)
						ret = 0;
					#else
						if ((is_op_bypassed(ovMount))
							&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
							)
							ret = 0;
						else
							ret = 1;
					#endif
					}
				}

				free_path(pathbuf);
			} else {
			#if defined(PASS_MOUNT_RESTRICT)
				ret = 0;
			#else
				if ((is_op_bypassed(ovMount))
					&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed), fcIsOpBypassed))
					)
					ret = 0;
				else
					ret = 1;
			#endif
			}
		} else {
			free_path(pathbuf);
		}
	}

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_insmod_execve_arg_check RS_HIDE(_232)
#endif
noused notrace int do_insmod_execve_arg_check(int argc, char *const *argv)
{
	/*pass: return 0, otherwise: not 0*/
#if defined(CONFIG_INSMOD_RESTRICT)
	int ret = 0;
	/*int local_optind;*/
	char *file_path;
	noused char *real_file_path;
	char *pathbuf;

	if (argc < 2) {
		return ret;
	}

	pathbuf = NULL;
	file_path = argv[1];

#if defined(RS_ABS_PATH_CHECK_SYMLINK)
	real_file_path = try_get_real_filename(file_path, &pathbuf);

	if (real_file_path) {
		RS_LOG("insmod:%s\n", real_file_path);

		ret = insmod_check_path(real_file_path);
	}
#else
	if (file_path[0] != '/') {
		/*需要绝对路径*/
		char *temp_file_path = get_real_filename(file_path, &pathbuf);
		if (temp_file_path) {
			file_path = temp_file_path;
		}
	}

	RS_LOG("insmod:%s\n", file_path);

	ret = insmod_check_path(file_path);
#endif

	if (pathbuf) {
		free_path(pathbuf);
	}

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_SuDaemon_execve_arg_check RS_HIDE(_233)
#endif
noused notrace int do_SuDaemon_execve_arg_check(int argc, char *const *argv)
{
	/*pass: return 0, otherwise: not 0*/
	int ret = 0;
	int i, j;

	/*superuser: su --daemon*/
	/*superSU: su {-d|--daemon|-ad|--auto-daemon|-r|--reload}*/

	if (argc < 2) {
		return ret;
	}

	for (i = 1; i < argc; i++) {
		char *arg = argv[i];

		for (j = 0; j < ARRAY_SIZE(su_preload_arguments); j++) {
			int res;

			res = STRCMP(arg, su_preload_arguments[j], j);

			if (res == 0) {
				ret = 1;
				break;
			} else if (res < 0) {
				break;
			}
		}

		if (ret) {
			RS_LOG("exe,bad arg\n");

			break;
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_busybox_execve_arg_check RS_HIDE(_234)
#endif
noused notrace int do_busybox_execve_arg_check(int argc, char *const *argv)
{
	/*pass: return 0, otherwise: not 0*/
	int ret = 0;

	if (argc < 3) {
		return ret;
	}

	argc--;
	argv++;

	if (STRCMP(argv[0], check_strings[csMount].str, csMount) == 0) {
		return do_mount_execve_arg_check(argc, argv);
	} else if (STRCMP(argv[0], check_strings[csInsmod].str, csInsmod) == 0) {
		return do_insmod_execve_arg_check(argc, argv);
	}

	return ret;
}

/*程序 argv, envp 构造在 binfmt_elf.c 的 create_elf_tables() 中*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_arg_check_internal RS_HIDE(_235)
#endif
noused notrace static int do_execve_arg_check_internal(int check_arg_type,
	int argc,
	struct user_arg_ptr argv
#if defined(CONFIG_EXEC_CHECK_ENV)
	, int envc
	, struct user_arg_ptr envp
#endif
	)
{
	/*pass: return 0, otherwise: < 0*/
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	int ret = 0;
	int mem_flag;
	noused int i;
	noused int count, buf_size;
	noused char *argv_envp_buf;
	noused size_t *len_ptr;
	noused char *p_data;
	noused char **_argv;
#if defined(CONFIG_EXEC_CHECK_ENV)
	noused char **_envp;
#endif

	count = (argc + 1)
	#if defined(CONFIG_EXEC_CHECK_ENV)
		+ (envc + 1)
	#endif
		;

	mem_flag = 0;

	buf_size = (count * sizeof(size_t) * 2);

	if (buf_size <= PAGE_SIZE) {
		mem_flag |= 1;
		argv_envp_buf = (char *)rs_get_free_page(); /*__get_free_page(RS_KMALLOC_FLAG);*/
	} else {
		argv_envp_buf = rs_kmalloc(buf_size); /*kmalloc(buf_size, RS_KMALLOC_FLAG);*/
	}

	if (!argv_envp_buf) {
		return -ENOMEM;
	}

	len_ptr = (size_t *)argv_envp_buf;
	p_data = NULL;
	buf_size = 0;

#if defined(CONFIG_EXEC_SKIP_ARGV0)
	len_ptr++;
	len_ptr++;
	buf_size++;

	for (i = 1; i < argc; i++)
#else
	/*可以跳过 argv[0] ?*/
	for (i = 0; i < argc; i++)
#endif
	{
		size_t len;
		const char __user *p = get_user_arg_ptr(argv, i);
		if (!p) {
			goto out;
		}

		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			goto out;
		}

		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len) {
			ret = -EFAULT;
			goto out;
		}
		*len_ptr++ = (size_t)p;
		*len_ptr++ = len;

		buf_size += len;
	}

	*len_ptr++ = 0;
	buf_size++;

#if defined(CONFIG_EXEC_CHECK_ENV)
	for (i = 0; i < envp; i++) {
		size_t len;
		const char __user *p = get_user_arg_ptr(envp, i);
		if (!p) {
			goto out;
		}

		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			goto out;
		}

		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len) {
			ret = -EFAULT;
			goto out;
		}

		*len_ptr++ = (size_t)p;
		*len_ptr++ = len;

		buf_size += len;
	}

	*len_ptr++ = 0;
	buf_size++;
#endif

	if (buf_size <= PAGE_SIZE) {
		mem_flag |= 2;
		p_data = (char *)rs_get_free_page(); /*__get_free_page(RS_KMALLOC_FLAG);*/
	} else {
		p_data = rs_kmalloc(buf_size); /*kmalloc(buf_size, RS_KMALLOC_FLAG);*/
	}

	if (!p_data) {
		ret = -ENOMEM;
		goto out;
	}

	_argv = (char **)argv_envp_buf;
	len_ptr = (size_t *)argv_envp_buf;

#if defined(CONFIG_EXEC_SKIP_ARGV0)
	len_ptr++;
	len_ptr++;

	*_argv++ = p_data;
	*p_data++ = '\0';

	for (i = 1; i < argc; i++)
#else
	/*可以跳过 argv[0] ?*/
	for (i = 0; i < argc; i++)
#endif
	{
		size_t ptr = *len_ptr++;
		size_t len = *len_ptr++;
		const char __user *p = (const char __user *)ptr;

		if (copy_from_user(p_data, p, len)) {
			ret = -EFAULT;
			goto out;
		}

		/*RS_LOG("arg[%d]:%s\n", i, p_data);*/

		*_argv++ = p_data;
		p_data += len;
	}

	len_ptr++;
	*_argv++ = NULL;
	*p_data++ = 0;

#if defined(CONFIG_EXEC_CHECK_ENV)
	_envp = _argv;

	for (i = 0; i < envc; i++) {
		size_t ptr = *len_ptr++;
		size_t len = *len_ptr++;
		const char __user *p = (const char __user *)ptr;

		if (copy_from_user(p_data, p, len)) {
			ret = -EFAULT;
			goto out;
		}

		/*RS_LOG("env[%d]:%s\n", i, p_data);*/

		*_envp++ = p_data;
		p_data += len;
	}

	/*len_ptr++;*/
	*_envp++ = NULL;
	*p_data++ = 0;
#endif

	_argv = (char **)argv_envp_buf;
#if defined(CONFIG_EXEC_CHECK_ENV)
	_envp = _argv + argc + 1;
#endif

#if 1
	if (check_arg_type == catMount) {
		ret = do_mount_execve_arg_check(argc, _argv);
	} else if (check_arg_type == catInsmod) {
		ret = do_insmod_execve_arg_check(argc, _argv);
	} else if (check_arg_type == catSuDaemon) {
		ret = do_SuDaemon_execve_arg_check(argc, _argv);
	} else if (check_arg_type == catBusybox) {
		ret = do_busybox_execve_arg_check(argc, _argv);
	}
#endif

out:

	if (mem_flag & 1) {
		rs_free_page((unsigned long)argv_envp_buf); /*free_page((unsigned long)argv_envp_buf);*/
	} else {
		rs_kfree(argv_envp_buf); /*kfree(argv_envp_buf);*/
	}

	if (p_data) {
		if (mem_flag & 2) {
			rs_free_page((unsigned long)p_data); /*free_page((unsigned long)p_data);*/
		} else {
			rs_kfree(p_data); /*kfree(p_data);*/
		}
	}

	if (ret) {
		RS_LOG("Restricted arg. PID = %d(%s)\n", current->pid, current->comm);

	#if defined(PASS_EXEC_RESTRICT)
		ret = 0;
	#else
		if ((is_op_bypassed_no_fc(ovExec))
			/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
			)
			ret = 0;
	#endif
	}

	return ret;
#else
	return 0;
#endif
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define check_arg_type_and_count RS_HIDE(_236)
#endif
noused notrace static int check_arg_type_and_count(int check_arg_type, int argc)
{
	int ret = 0;

	if (check_arg_type == catMount) {
		if (argc < 3) {
			ret = 1;
		}
	} else if (check_arg_type == catInsmod) {
		if (argc < 2) {
			ret = 1;
		}
	} else if (check_arg_type == catSuDaemon) {
		if (argc < 2) {
			ret = 1;
		}
	} else if (check_arg_type == catBusybox) {
		if (argc < 3) {
			ret = 1;
		}
	}

	return ret;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_arg_check RS_HIDE(_237)
#endif
noused notrace static int do_execve_arg_check(int check_arg_type,
	int argc,
	struct user_arg_ptr argv
#if defined(CONFIG_EXEC_CHECK_ENV)
	, int envc
	, struct user_arg_ptr envp
#endif
	)
{
	/*pass: return 0, otherwise: < 0*/
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)

	if ((check_arg_type < 0) || (check_arg_type >= catInvalid)) {
		return 0;
	}

	if (check_arg_type_and_count(check_arg_type, argc)) {
		return 0;
	}

	return do_execve_arg_check_internal(check_arg_type, argc, argv
	#if defined(CONFIG_EXEC_CHECK_ENV)
		, envc, envp
	#endif
		);
#else
	return 0;
#endif

}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define do_execve_arg_check_no_count RS_HIDE(_238)
#endif
noused notrace int do_execve_arg_check_no_count(int check_arg_type,
	struct user_arg_ptr argv
#if defined(CONFIG_EXEC_CHECK_ENV)
	, struct user_arg_ptr envp
#endif
	)
{
	/*pass: return 0, otherwise: < 0*/
#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
	noused int argc;
#if defined(CONFIG_EXEC_CHECK_ENV)
	noused int envc;
#endif


	if ((check_arg_type < 0) || (check_arg_type >= catInvalid)) {
		return 0;
	}

	/*构造一个 argv*/
	argc = count_user_strings(argv, 1, MAX_ARG_STRINGS);

	if (argc <= 0) {
		return argc;
	}

	if (check_arg_type_and_count(check_arg_type, argc)) {
		return 0;
	}

#if defined(CONFIG_EXEC_CHECK_ENV)
	envc = count_user_strings(envp, 0, MAX_ARG_STRINGS);

	if (envc < 0) {
		return envc;
	}
#endif

	return do_execve_arg_check(check_arg_type, argc, argv
	#if defined(CONFIG_EXEC_CHECK_ENV)
		, envc, envp
	#endif
		);
#else
	return 0;
#endif
}

#endif

/*call_usermodehelper之类的也要检查*/
noused notrace int do_execve_common_check(const char *filename,
	struct user_arg_ptr argv
#if defined(CONFIG_EXEC_CHECK_ENV) || defined(CONFIG_EXEC_SU_CHECK_PATH_BY_ENV)
	, struct user_arg_ptr envp
#endif
	)
{
#if defined(CONFIG_EXEC_RESTRICT) || defined(CONFIG_EXEC_SU_RESTRICT)
	int ret;
	struct task_struct *curr;

	if ((1)
	#if defined(CONFIG_EXEC_RESTRICT)
		&& (is_op_permitted_no_fc(ovExec))
	#endif
	#if defined(CONFIG_EXEC_SU_RESTRICT)
		&& (is_op_permitted_no_fc(ovSuExec))
	#endif
		) {
	#if 0 /*defined(CONFIG_EXEC_RESTRICT) || defined(CONFIG_EXEC_SU_RESTRICT)*/
		if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
			set_vars_global_flag(gfAllowSuExec, 1);
			/*g_allow_exec_su = 1;*/

			ret = 0;
			goto out;
		} else {
			ret = -EPERM;
			goto out_checked;
		}
	#else
		set_vars_global_flag(gfAllowSuExec, 1);
		/*g_allow_exec_su = 1;*/

		ret = 0;
		goto out;
	#endif
	}


	curr = current;

	if (CHECK_ROOT_UID(curr)) {
#if defined(CONFIG_EXEC_RESTRICT)

	#if defined(CONFIG_EXEC_CHECK_ARG_LATER)
		int check_arg_type = catInvalid;

		if (do_execve_check_internal(filename, &check_arg_type) || do_execve_arg_check_no_count(check_arg_type, argv
		#if defined(CONFIG_EXEC_CHECK_ENV)
			, envp
		#endif
			)
			)
	#else
		if (do_execve_check_internal(filename, argv))
	#endif
		{
			RS_LOG("Restricted execve_comm. PID = %d(%s) "
				"PPID = %d(%s),%s\n",
				curr->pid, curr->comm,
				curr->parent->pid, curr->parent->comm, filename);

		#if defined(PASS_EXEC_RESTRICT)
			ret = 0;
		#else
			if ((is_op_bypassed_no_fc(ovExec))
				/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
				)
				ret = 0;
			else
				ret = -EACCES;
		#endif
		} else {
			ret = 0;
		}
#else
		ret = 0;
#endif
	}
#if defined(CONFIG_EXEC_SU_RESTRICT)
	else {
		if (do_execve_su_check_internal(filename
		#if defined(CONFIG_EXEC_SU_CHECK_PATH_BY_ENV)
			, envp
		#endif
			)
			) {
			RS_LOG("Restricted execve su. PID = %d(%s) "
				"PPID = %d(%s),%s\n",
				curr->pid, curr->comm,
				curr->parent->pid, curr->parent->comm, filename);

		#if defined(PASS_EXEC_SU_RESTRICT)
			ret = 0;
		#else
			if ((is_op_bypassed_no_fc(ovSuExec))
				/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
				)
				ret = 0;
			else
				ret = -EACCES;
		#endif
		} else {
			ret = 0;
		}
	}

out:
/*out_checked:*/
	return ret;
#endif
#else
#if 0
	struct task_struct *curr;

	if (is_op_permitted_no_fc(ovExec)) {
		return 0;
	}

	curr = current;

	if (CHECK_ROOT_UID(curr)) {
		RS_LOG("[%d:%s] exec %s\n", curr->pid, curr->comm, filename);
	}
#endif

	return 0;
#endif
}

/*call_usermodehelper之类的也要检查*/
noused notrace int do_execve_common_file_check(const struct file *file)
{
#if defined(CONFIG_EXEC_SU_RESTRICT)
	int ret = -EPERM;
	struct task_struct *curr;

	curr = current;

	/*sys_execve not called in kernel space*/
#if 0
	if (/*(!file) ||*/ /* (!curr->mm) || (is_init_task(curr)) ||*/ ((curr->flags & PF_KTHREAD) && (!curr->mm)
			&& (!task_in_call_from_user(curr)))) {
		ret = 0;
		goto out;
	}
#endif

	if (!CHECK_ROOT_UID(curr)) {
		const struct qstr *name = &(file->f_path.dentry->d_name);
		const unsigned char *pName = name->name;
		int len = name->len;

		if ((len == 2) && ((pName[0] == 's') && (pName[1] == 'u'))) {
		#if defined(RS_IS_ENG_BUILD) || defined(BBK_FOR_NET_ENTRY)
			ret = 0;
		#else  /*RS_IS_ENG_BUILD*/
		#if 0
			/*already checked in can_exec_su()*/
			if (is_op_permitted_no_fc(ovSuExec)) {
			#if 0
				if (!rs_verify_func(RS_FC_PARAMS(is_op_permitted_no_fc), fcIsOpPermittedNoFC)) {
					ret = 0;
					goto out;
				} else {
					ret = -EPERM;
					goto out_checked;
				}
			#else
				ret = 0;
				goto out;
			#endif
			}
		#endif

			if (can_exec_su()) {
				RS_LOG("Restricted execve su file. PID = %d(%s) "
					"PPID = %d(%s)\n",
					curr->pid, curr->comm,
					curr->parent->pid, curr->parent->comm);

			#if defined(PASS_EXEC_SU_RESTRICT)
				ret = 0;
			#else
			#if 0
				/*already checked in can_exec_su()*/
				if ((is_op_bypassed_no_fc(ovSuExec))
					/*&& (!rs_verify_func(RS_FC_PARAMS(is_op_bypassed_no_fc), fcIsOpBypassedNoFC))*/
					)
					ret = 0;
				else
					ret = -EACCES;
			#endif
			#endif
			} else {
				ret = 0;
			}
		#endif
		} else {
			ret = 0;
		}
	} else {
		ret = 0;
	}
/*out:*/
/*out_checked:*/
	return ret;
#else
	return 0;
#endif
}

/*////////////////////////////*/

#if 0 /*defined(CONFIG_KALLSYMS) && defined(CONFIG_RS_OBFUSCATED_NAME)*/ /*&& defined(CONFIG_KALLSYMS_ALL)*/
/*试了 CONFIG_KALLSYMS_ALL 不开/proc/kallsyms也能看到 do_uid_check 等*/
/*改为修改在 scripts/kallsyms.c 中屏蔽相关符号*/

/*copy from kernel/kernel/kallsyms.c*/

extern const unsigned long kallsyms_addresses[] __attribute__((weak));
extern const u8 kallsyms_names[] __attribute__((weak));

extern const unsigned long kallsyms_num_syms
__attribute__((weak, section(".rodata")));

extern const u8 kallsyms_token_table[] __attribute__((weak));
extern const u16 kallsyms_token_index[] __attribute__((weak));

extern const unsigned long kallsyms_markers[] __attribute__((weak));

/*from get_symbol_pos()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_symbol_pos RS_HIDE(_6a)
#endif
noused notrace static unsigned long rs_get_symbol_pos(unsigned long addr)
{
	/*unsigned long symbol_start = 0, symbol_end = 0;*/
	noused unsigned long i, low, high, mid;

	/* This kernel should never had been booted. */
	if (!kallsyms_addresses) {
		return 0;
	}

	/* Do a binary search on the sorted kallsyms_addresses array. */
	low = 0;
	high = kallsyms_num_syms;

	while (high - low > 1) {
		mid = low + (high - low) / 2;
		if (kallsyms_addresses[mid] <= addr)
			low = mid;
		else
			high = mid;
	}

	/*
	 * Search for the first aliased symbol. Aliased
	 * symbols are symbols with the same address.
	 */
	while (low && kallsyms_addresses[low-1] == kallsyms_addresses[low])
		--low;

	return low;
}

/*from get_symbol_offset()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_symbol_offset RS_HIDE(_6b)
#endif
noused notrace static unsigned int rs_get_symbol_offset(unsigned long pos)
{
	const u8 *name;
	int i;

	/*
	 * Use the closest marker we have. We have markers every 256 positions,
	 * so that should be close enough.
	 */
	name = &kallsyms_names[kallsyms_markers[pos >> 8]];

	/*
	 * Sequentially scan all the symbols up to the point we're searching
	 * for. Every symbol is stored in a [<len>][<len> bytes of data] format,
	 * so we just need to add the len to the current pointer for every
	 * symbol we wish to skip.
	 */
	for (i = 0; i < (pos & 0xFF); i++)
		name = name + (*name) + 1;

	return name - kallsyms_names;
}

/*copy from kallsyms_expand_symbol()*/
noused notrace static unsigned int rs_kallsyms_expand_symbol(unsigned int off,
					char *result, size_t maxlen)
{
	int len, skipped_first = 0;
	const u8 *tptr, *data;

	/* Get the compressed symbol length from the first symbol byte. */
	data = &kallsyms_names[off];
	len = *data;
	data++;

	/*
	 * Update the offset to return the offset for the next symbol on
	 * the compressed stream.
	 */
	off += len + 1;

	/*
	 * For every byte on the compressed symbol data, copy the table
	 * entry for that byte.
	 */
	while (len) {
		tptr = &kallsyms_token_table[kallsyms_token_index[*data]];
		data++;
		len--;

		while (*tptr) {
			if (skipped_first) {
				if (maxlen <= 1)
					goto tail;
				*result = *tptr;
				result++;
				maxlen--;
			} else
				skipped_first = 1;
			tptr++;
		}
	}

tail:
	if (maxlen)
		*result = '\0';

	/* Return to offset to the next symbol. */
	return off;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_get_index_of_tonken_index RS_HIDE(_6q)
#endif
noused notrace static int rs_get_index_of_tonken_index(u8 start_ch, u8 end_ch)
{

	/*可研究下 .tmp_kallsyms1.S, .tmp_kallsyms2.S, '0'~'9', 'A'~'Z', 'a'~'z' 每块是连续的
	kallsyms_token_index表紧跟在kallsyms_token_table表后面,两个数组都是最多256个元素
	可参考 kernel/scripts/kallsyms.c中write_src()
	*/

	#define KALLSYMS_TOKEN_INDEX_ARRAY_SIZE (256)

	int ret, index_of_token_index;
	u16 token_index;
	const u8 *tptr;
	u8 ch;
	ret = -1;

	for (index_of_token_index = 0; index_of_token_index < KALLSYMS_TOKEN_INDEX_ARRAY_SIZE; index_of_token_index++) {
		token_index = kallsyms_token_index[index_of_token_index];

		tptr = &kallsyms_token_table[token_index];

		ch = *tptr++;
		/*长度为1的token串*/
		if ((*tptr == '\0') && ((ch >= start_ch) && (ch <= end_ch))) {
			ret = index_of_token_index;
			break;
		}
	}

	return ret;
}

/*refer to kallsyms_expand_symbol()*/
#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_kallsyms_reset_symbol RS_HIDE(_6r)
#endif
static int rs_kallsyms_reset_symbol(unsigned int off, int index, struct page_protect_struct *protect_data)
{
	noused int len, skipped_first = 0;
	noused u8 *tptr, *data;
	noused unsigned long prev_page_addr;
	noused unsigned long page_addr;
	int ret = 0;

	/* Get the compressed symbol length from the first symbol byte. */
	data = (u8 *)&kallsyms_names[off];
	len = *data;
	data++;

	/*
	 * For every byte on the compressed symbol data, copy the table
	 * entry for that byte.
	 */

	if (len) {
		prev_page_addr = protect_data->header.prev_page_addr;

		do {
			/*tptr = (u8 *)&kallsyms_token_table[kallsyms_token_index[*data]];*/
			/*第一个是代表符号类型的字符，需要跳过?*/
			/*if (skipped_first)*/
			{
				page_addr = ((unsigned long)data & (~(PAGE_SIZE - 1)));
				if (!page_addr) {
					break;
				}

				if (page_addr != prev_page_addr) {
					if (prev_page_addr) {
						ret = set_page_ro(page_addr, protect_data);
						if (ret) {
							/*fail to set as ro*/
							prev_page_addr = 0;
							break;
						}

						ret = set_kernel_text_page_ro(page_addr);
						if (ret) {
							/*fail to set as ro*/
							prev_page_addr = 0;
							goto out;
						}

					}

					ret = set_kernel_text_page_rw(page_addr);
					if (ret) {
						/*fail to set as rw*/
						prev_page_addr = 0;
						goto out;
					}

					ret = set_page_rw(page_addr, protect_data);
					if (ret) {
						/*fail to set as rw*/
						prev_page_addr = 0;
						break;
					}

					prev_page_addr = page_addr;
				}

				if (skipped_first) {
				#if defined(RS_USE_PROBE_WRITE)
					ret = probe_kernel_write((void *)data, &index, sizeof(index));
					if (ret)
						break;
					else
						RS_FLUSH_DCACHE_RANGE((unsigned long)data, (unsigned long)data + sizeof(index));
				#else
					/**data = index;*/
					{
						u8 tmp = index;
						ret = copy_to_ro_page(data, &tmp, sizeof(tmp), protect_data);
						if (ret)
							RS_LOG("rkrs,2,%d\n", ret);
						else
							RS_FLUSH_DCACHE_RANGE((unsigned long)data, (unsigned long)data + sizeof(tmp));
					}
				#endif
				} else {
					/*把符号类型从 'T' 改成 'R' (只读数据段)*/
				#if defined(RS_USE_PROBE_WRITE)
					ret = probe_kernel_write((void *)data, "R", (sizeof("R") - 1));
					if (ret)
						break;
					else
						RS_FLUSH_DCACHE_RANGE((unsigned long)data, (unsigned long)data + (sizeof("R") - 1));
				#else
					/**data = 'R';*/
					{
						u8 tmp = 'R';
						ret = copy_to_ro_page(data, &tmp, sizeof(tmp), protect_data);
						if (ret)
							RS_LOG("rkrs,3,%d\n", ret);
						else
							RS_FLUSH_DCACHE_RANGE((unsigned long)data, (unsigned long)data + sizeof(tmp));
					}
				#endif

					skipped_first = 1;
				}
			}
			/*else {
				把符号类型从 'T' 改成 'R' (只读数据段)
				skipped_first = 1;
			}
			*/

			data++;
			len--;
		} while (len);

		protect_data->header.prev_page_addr = prev_page_addr;
	}

#if 1
	if (prev_page_addr) {
		ret = set_page_ro(prev_page_addr, protect_data);

		if (!ret) {
			ret = set_kernel_text_page_ro(prev_page_addr));
		}

		protect_data->header.prev_page_addr = 0;
	}
#endif

	return ret;
}

noused notrace static inline int rs_is_kernel_inittext(unsigned long addr)
{
	if (addr >= (unsigned long)_sinittext
		&& addr <= (unsigned long)_einittext)
		return 1;
	return 0;
}

noused notrace static unsigned long rs_kallsyms_lookup_name(const char *name, unsigned long *symbol_size)
{
	char namebuf[KSYM_NAME_LEN];
	unsigned long i;
	unsigned int off;
	unsigned long symbol_start = 0, symbol_end = 0;

	for (i = 0, off = 0; i < kallsyms_num_syms; i++) {
		off = rs_kallsyms_expand_symbol(off, namebuf, ARRAY_SIZE(namebuf));

		if (strcmp(namebuf, name) == 0) {
			/*return kallsyms_addresses[i];*/
			symbol_start = kallsyms_addresses[i];

			if (symbol_size) {
				unsigned long j;

				/* Search for next non-aliased symbol. */
				for (j = i + 1; j < kallsyms_num_syms; j++) {
					if (kallsyms_addresses[j] > symbol_start) {
						symbol_end = kallsyms_addresses[j];
						break;
					}
				}


				if (!symbol_end) {
					if (rs_is_kernel_inittext(symbol_start))
						symbol_end = (unsigned long)_einittext;
					else if (IS_ENABLED(CONFIG_KALLSYMS_ALL))
						symbol_end = (unsigned long)_end;
					else
						symbol_end = (unsigned long)_etext;
				}
			}
		}
	}

	if (symbol_size) {
		*symbol_size = symbol_end - symbol_start;
	}

	return symbol_start; /*module_kallsyms_lookup_name(name);*/
}

#ifndef GCC_VERSION_CODE
#define GCC_VERSION_CODE (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

typedef char *RS_SYM_PTR_TYPE;
typedef char **RS_SYM_PTR_TYPE_PTR;
typedef ssize_t RS_SYM_VAL_TYPE; /*sizeof(RS_SYM_VAL_TYPE)应该和sizeof(RS_SYM_PTR_TYPE)一致*/ /*long*/

#define RS_FUNC_BASE (RS_SYM_VAL_TYPE)(PAGE_OFFSET + LINUX_VERSION_CODE + GCC_VERSION_CODE)

#define RS_PUBLIC_SYM(sym_name) \
	(RS_SYM_PTR_TYPE)((RS_SYM_PTR_TYPE)(sym_name) - (RS_SYM_VAL_TYPE)(RS_FUNC_BASE))


#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_public_syms RS_HIDE(_N) /*不能用 _C, _D*/
#endif
noused const static RS_SYM_PTR_TYPE rs_public_syms[] = {
	RS_PUBLIC_SYM(get_task_reap_init_flag),
#if defined(RS_VIVOROOT_SUPPORT)
	/*RS_PUBLIC_SYM(get_task_dbg_flag),*/
	/*RS_PUBLIC_SYM(set_task_dbg_flag),*/
	/*RS_PUBLIC_SYM(clear_task_dbg_flag),*/
#endif
	RS_PUBLIC_SYM(do_insmod_check),
	RS_PUBLIC_SYM(do_chroot_check),
	RS_PUBLIC_SYM(do_access_dev_check),
	RS_PUBLIC_SYM(do_disable_selinux_check),
	RS_PUBLIC_SYM(do_ptrace_check),
	RS_PUBLIC_SYM(do_mount_files_sb_check),
	RS_PUBLIC_SYM(do_uid_check),
	RS_PUBLIC_SYM(do_execve_common_check),
	RS_PUBLIC_SYM(do_execve_common_file_check),
#if defined(CONFIG_KALLSYMS_ALL)
	RS_PUBLIC_SYM(rs_um),
#endif
#if defined(RS_VIVOROOT_SUPPORT)
	RS_PUBLIC_SYM(is_dbg_task),
	/*RS_PUBLIC_SYM(check_dbg_task),*/
	/*RS_PUBLIC_SYM(get_shell_sctx),*/
#endif
};

#ifndef KSYM_NAME_LEN
	#define KSYM_NAME_LEN 128
#endif

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_kallsyms_list RS_HIDE(_6s)
#endif
noused notrace static int rs_reset_kallsyms_list(const RS_SYM_PTR_TYPE_PTR sym_addrs, int sym_addrs_count,
	RS_SYM_VAL_TYPE sym_base, int is_funcs)
{
	unsigned long addr;
	noused unsigned long sym_pos;
	noused unsigned int sym_offs;
	noused int i, ret, index_of_token_index;
	noused char symname[KSYM_NAME_LEN];
	struct page_protect_struct protect_data;

	index_of_token_index = rs_get_index_of_tonken_index('A', 'z');

	if (index_of_token_index < 0) {
		return index_of_token_index;
	}


	/*protect_data.header.prev_page_addr = 0;*/
	/*protect_data.header.flags = 0;*/
	memset(&protect_data, 0, sizeof(protect_data));

	for (i = 0; i < sym_addrs_count; i++) {
		addr = (unsigned long)(sym_addrs[i] + sym_base);
		if (!addr) {
			continue;
		}

		if (is_funcs) {
			addr = (unsigned long)(dereference_function_descriptor((void *)addr));
			if (!addr) {
				continue;
			}
		}

		sym_pos = rs_get_symbol_pos(addr);

		if (sym_pos) {
			sym_offs = rs_get_symbol_offset(sym_pos);

#if 0
			symname[0] = '\0';
			symname[KSYM_NAME_LEN - 1] = '\0';
			rs_kallsyms_expand_symbol(sym_offs,
						symname, KSYM_NAME_LEN);
			printk("xjy,relist 2,%s\n", symname);
#endif

			rs_kallsyms_reset_symbol(sym_offs, (index_of_token_index + i), &protect_data);
		}
	}

	if (protect_data.header.prev_page_addr) {
		set_page_ro(protect_data.header.prev_page_addr, &protect_data);
	}

	return 0;
}

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_kallsyms_info RS_HIDE(_6t)
#endif
noused notrace static int __init rs_reset_kallsyms_info(void)
{
	/*重置几个不能混淆名称的函数在/proc/kallsyms中的显示*/
	return rs_reset_kallsyms_list(rs_public_syms, ARRAY_SIZE(rs_public_syms), RS_FUNC_BASE, 0);
}

late_initcall_sync(rs_reset_kallsyms_info); /*module_init(rs_reset_kallsyms_info);*/
#else

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define rs_reset_kallsyms_info RS_HIDE(_6t)
#endif
noused notrace static int rs_reset_kallsyms_info(void)
{
	return 0;
}
#endif

notrace void vrr_do_mount_init_check_param(struct tag_do_mount_check_param *param,
	struct path *path_ptr, unsigned int *mnt_flags_ptr,
	const char *dev_name, const char __user *dir_name,
	const char *type_page, unsigned long flags, void *data_page)
{
	param->path_ptr = path_ptr;
	param->mnt_flags_ptr = mnt_flags_ptr;
	param->dev_name = dev_name;
	param->dir_name = untagged_addr(dir_name);
	param->type_page = type_page;
	param->flags = flags;
	param->data_page = data_page;

#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK)
#if defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
	param->kernel_dir = NULL;
#endif
	param->bitmap_buffer = NULL;
#endif
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
	param->check_flags = 0;
#endif
#if defined(RS_NEED_DO_MOUNT_CHECK)
	param->dir_name_buffer = NULL;

	param->check_info.task_infos = NULL;
	param->check_info.need_free_task_infos = 0;
#endif
}

notrace int vrr_do_mount_prepare_check_param(struct tag_do_mount_check_param *param)
{
#if !defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
	if (!param->dir_name || !*param->dir_name || !memchr(param->dir_name, 0, PAGE_SIZE))
		return -EINVAL;
#endif

#if defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
#if (defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK))
	param->dir_name_from_user = false;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	param->kernel_dir = copy_mount_string(param->dir_name);
	if (IS_ERR(param->kernel_dir)) {
		/* for "run vts -m VtsKernelLtp -t VtsKernelLtp#syscalls.mount02" fail.
		 * refer to copy_mount_string() -> strndup_user(). the latter return -EINVAL,
		 * if length of string (including tailing NUL) exceed PAGE_SIZE
		 */
		if (PTR_ERR(param->kernel_dir) == -EINVAL) {
			param->kernel_dir = ERR_PTR(-ENAMETOOLONG);
		}
		return PTR_ERR(param->kernel_dir);
	}
	param->dir_name = (param->kernel_dir) ? (const char *)(param->kernel_dir) : "";
#else
	param->kernel_dir = getname(param->dir_name);
	if (IS_ERR(param->kernel_dir)) {
		return PTR_ERR(param->kernel_dir);
	}
	param->dir_name = (const char *)(param->kernel_dir->name);
#endif
#else
	param->dir_name_from_user = true;
#endif
#else /*RS_DO_MOUNT_USE_USER_DIR_NAME*/
	param->dir_name_from_user = false;
#endif

	/*printk(KERN_ERR "mnt:%s,%s,%s,%s,%lx,%s\n", param->dev_name, param->dir_name, (char *)param->type_page, (char *)param->data_page, param->flags, current->comm);*/
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
	if (param->flags & MS_REMOUNT) {
		if (!(param->flags & MS_RDONLY)) {
		#if defined(RS_NEED_DO_MOUNT_CHECK)
			param->check_flags |= RS_DO_MOUNT_CHECK_MOUNT;
		#endif
		}
	#if defined(CONFIG_RS_CHECK_FUNC)
		else if ((!g_bootup_checked) && (param->dir_name[0] == '/') && (param->dir_name[1] == '\0')
				 && (!is_init_task(current))) {
			param->check_flags |= RS_DO_MOUNT_CHECK_BOOT;  /*条件3*/
		}
	#endif
	} else if (param->flags & MS_BIND) {
	#if defined(CONFIG_RS_CHECK_FUNC)
		if ((!g_bootup_checked) && (param->flags == MS_BIND)) {
			param->check_flags |= RS_DO_MOUNT_CHECK_BOOT;  /*条件6*/
		}
	#endif
	#if defined(RS_NEED_DO_MOUNT_CHECK) && defined(CONFIG_MOUNT_BIND_MOVE_RESTRICT)
		if (is_mount_bind_need_check()) {
			param->check_flags |= RS_DO_MOUNT_CHECK_MOUNT;
		}
	#endif
	} else if (param->flags & (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE)) {
	#if defined(CONFIG_RS_CHECK_FUNC)
		if ((!g_bootup_checked) && (param->flags == (MS_SLAVE | MS_REC))) {
			param->check_flags |= RS_DO_MOUNT_CHECK_BOOT;  /*条件2*/
		}
	#endif
	} else if (param->flags & MS_MOVE) {
	#if defined(RS_NEED_DO_MOUNT_CHECK) && defined(CONFIG_MOUNT_BIND_MOVE_RESTRICT)
		param->check_flags |= RS_DO_MOUNT_CHECK_MOUNT;
	#endif
	} else {
	#if defined(CONFIG_RS_CHECK_FUNC)
		if (!g_bootup_checked) {
			param->check_flags |= RS_DO_MOUNT_CHECK_BOOT; /*条件1 和 条件5*/
		}
	#endif

		if (!(param->flags & MS_RDONLY)) {
		#if defined(RS_NEED_DO_MOUNT_CHECK)
			param->check_flags |= RS_DO_MOUNT_CHECK_MOUNT;
		#endif
		}
	}

	if (param->check_flags) {
		/*lizonglin transplant for is_root function start*/
		rs_printk_idx(KERN_ERR, prnsMountInfo, param->dev_name, param->dir_name, param->type_page, param->flags);
		/*printk(KERN_ERR "mnt:%s,%s,%s,%s,%lx,%d,%s\n", param->dev_name, param->dir_name, param->type_page, (char *)param->data_page, param->flags, param->check_flags, current->comm);*/
		/*lizonglin transplant end*/
	}
#endif

	return 0;
}

notrace bool vrr_do_mount_get_mountpoint(struct tag_do_mount_check_param *param, int *retval_ptr)
{
	noused int retval;

#if defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
#if (defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK))
	retval = kern_path(param->dir_name, LOOKUP_FOLLOW, param->path_ptr);
	if (retval_ptr) {
		*retval_ptr = retval;
	}

	if (retval) {
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
		kfree(param->kernel_dir);
	#else
		putname(param->kernel_dir);
	#endif
		param->kernel_dir = NULL;
		param->dir_name = NULL;

		return false;
	} else {
		return true;
	}
#else
	/* do nothing */
	return false;
#endif
#else
	retval = kern_path(param->dir_name, LOOKUP_FOLLOW, param->path_ptr);
	if (retval_ptr) {
		*retval_ptr = retval;
	}
	if (retval) {
		return false;
	} else {
		return true;
	}
#endif

	return 0;
}

notrace bool vrr_do_mount_check_before_mount(struct tag_do_mount_check_param *param, int *retval_ptr)
{
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
	if (param->check_flags) {
		const char *real_devname;

		if (param->flags & MS_REMOUNT) {
			/* may make real_devname as /dev/root if dev_name is /dev/block/bootdevice/by-name/system_X
			 * when RS_SYSTEM_AS_ROOT_SUPPORT
			 */
			real_devname = real_mount(param->path_ptr->mnt)->mnt_devname;
			if ((!real_devname) || (!real_devname[0]))
				real_devname = param->dev_name;
		} else {
			real_devname = param->dev_name;
		}

		init_fc_bitmap(&param->bitmap_buffer);

		if (param->check_flags & RS_DO_MOUNT_CHECK_BOOT) {
		#if defined(CONFIG_RS_CHECK_FUNC)
			/*if (!g_bootup_checked)*/
			{
				if (!rs_get_bootup_set()) {
					if (is_property_service_started()) {
						g_bootup_checked = rs_get_set_value();
						RS_LOG("sb3\n");
						rs_try_set_bootup_done();
					}
				}
			}
		#endif
		}

		if (param->check_flags & RS_DO_MOUNT_CHECK_MOUNT) {
			int retval;

			if (((param->dir_name[0] != '/') && (param->dir_name[0] != '\0')) ||
			#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
				(current->total_link_count)
			#else
				(d_is_symlink(param->path_ptr->dentry))
			#endif
				) {
				/* relative path or has symbol link
				 * refer to follow_link/follow_automount
				 */
				char *ret_ptr = rs_get_real_dir_name(param->path_ptr, &param->dir_name_buffer);
				if (ret_ptr) {
					param->dir_name = ret_ptr;
					param->dir_name_from_user = false;
				}
			}

			retval = do_mount_check(real_devname, param->dir_name, param->flags, param->mnt_flags_ptr, &param->check_info);
			if (retval < 0) {
				if (retval_ptr) {
					*retval_ptr = retval;
				}

				return false;
			}

		#if defined(CONFIG_RS_CHECK_FUNC)
			if ((param->check_info.flags.op_data.op_type == ovMount)
				&& (param->check_info.flags.op_data.op_info < rmtInvalid)
				) {
				if ((rs_verify_func(RS_FC_PARAMS(do_mount_check), fcDoMountCheck))
					|| (rs_verify_func(RS_FC_PARAMS_PUB(do_mount), fcDoMount))
				#if defined(CONFIG_ACCESS_DEV_RESTRICT)
					|| (rs_verify_func(RS_FC_PARAMS(do_access_dev_check), fcDoAccessDevCheck))
				#endif
					) {
					retval = -EPERM;

					if (retval_ptr) {
						*retval_ptr = retval;
					}
					return false;
				}
			}
		#endif
		}
	}
#endif

#if defined(CONFIG_MOUNT_FILES_CHECK)
#if !defined(RS_NEED_DO_MOUNT_CHECK)
	init_fc_bitmap(&param->bitmap_buffer);
#endif
	do_mount_files_check(param->path_ptr, param->dev_name, param->dir_name, param->flags_ptr, param->mnt_flags_ptr, param->data_page, 0);
#endif

	return true;
}

notrace void vrr_do_mount_check_after_mount(struct tag_do_mount_check_param *param, int retval)
{
#if defined(CONFIG_MOUNT_FILES_CHECK)
	if (retval == 0) {
		do_mount_files_check(param->path_ptr, param->dev_name, param->dir_name, &param->flags, param->mnt_flags_ptr, param->data_page, 1);
	}
#endif

	/*lizonglin transplant for is_root function start*/
	/* added by dengqiang for check root operation 2012-06-01 */
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
	if (param->check_flags) {
		rs_printk_idx(NULL, prnsIsMountFlagEnable, isMountFlagEnabled);
	}
#endif

	if ((!retval)
	#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
		&& (param->check_flags/* & RS_DO_MOUNT_CHECK_MOUNT*/)
	#endif
		) {
		rs_do_mount_post_process(param->dev_name, param->dir_name, param->flags,
		#if defined(RS_NEED_DO_MOUNT_CHECK)
			((param->check_flags & RS_DO_MOUNT_CHECK_MOUNT) ? &param->check_info : NULL)
		#else
			NULL
		#endif
			, param->dir_name_from_user
			);
	}
	/* add end */
	/*lizonglin transplant end*/
}

notrace void vrr_do_mount_done_check_param(struct tag_do_mount_check_param *param)
{
#if defined(RS_NEED_DO_MOUNT_CHECK)
	if (param->dir_name_buffer) {
		RS_FREE_BUFFER(param->dir_name_buffer);
	}

	if ((param->check_info.task_infos) && ((uintptr_t)param->check_info.task_infos > csInvalid)
		&& (param->check_info.need_free_task_infos)) {
		RS_FREE_BUFFER(param->check_info.task_infos);
	}
#endif
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK)
	done_fc_bitmap(param->bitmap_buffer);
#endif
#if defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
#if (defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK))
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	kfree(param->kernel_dir);
#else
	putname(param->kernel_dir);
#endif
#endif
#endif
}

noused notrace int do_access_blkdev_check(struct gendisk *disk, struct hd_struct *part, fmode_t mode)
{
	if ((disk) && (part) && ((MMC_BLOCK_MAJOR == disk->major)
	#if defined(CONFIG_SCSI_UFSHCD)
		|| (SCSI_DISK0_MAJOR == disk->major)
		|| (BLOCK_EXT_MAJOR == disk->major)
	#endif
		)) {
		int partno = part->partno;
		if ((partno) && (mode & (FMODE_WRITE | FMODE_PWRITE | FMODE_WRITE_IOCTL))) {
			char part_name[BDEVNAME_SIZE];
			int pname_ok = 0;
			char *pname;
			struct partition_meta_info *info = part->info;

			if (info) {
				pname = info->volname;
				if (pname[0])
					pname_ok = 1;
			}

			if (!pname_ok) {
				part_name[0] = '\0';

				disk_name(disk, partno, part_name);

				if (part_name[0])
					pname = part_name;
				else
					pname = NULL;
			}

			if (pname) {
				int result = do_access_dev_check(pname, 2, 1);

				if (result < 0) {
					return -EPERM;
				}
			}
		}
	}

	return 0;
}

#define SSC_SET_TEXT_VARS_INLINE /* __sec_syscall_set_text_vars() inline or not */

#undef SSC_SET_TEXT_VARS_CHECK_CALLER

#if  !(defined(CONFIG_LTO_CLANG) && defined(CONFIG_CFI_CLANG))
/* __sec_syscall_init_mode() called in init stage, will not call this routine */
#if defined(CONFIG_SECURITY_SELINUX)

extern int __sec_syscall_sel_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count);
#define SSC_SET_TEXT_VARS_CHECK_CALLER
#endif

noused static const unsigned long __sec_syscall_set_text_vars_valid_callers[] = {
#if defined(CONFIG_SECURITY_SELINUX)
	(unsigned long)&__sec_syscall_sel_set_text_vars,
#endif
};

#define RS_SET_TEXT_VARS_MAX_CALLER_FUNC_SIZE (PAGE_SIZE)
#endif

/* __sec_syscall_set_text_vars() does not need to check caller if it is inlined */
#if !defined(SSC_SET_TEXT_VARS_INLINE)
	#define SSC_INLINE_PREFIX noinine
#else /* !SSC_SET_TEXT_VARS_INLINE */
	#undef SSC_SET_TEXT_VARS_CHECK_CALLER
	#define SSC_INLINE_PREFIX inline
#endif /* SSC_SET_TEXT_VARS_INLINE */

SSC_INLINE_PREFIX	\
noused notrace static int __sec_syscall_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count)
{
	int ret;

#if defined(SSC_SET_TEXT_VARS_CHECK_CALLER)
	{
		unsigned long caller_addr = (unsigned long)__builtin_return_address(0);
		size_t i;

		for (i = 0; i < ARRAY_SIZE(__sec_syscall_set_text_vars_valid_callers); i++) {
			unsigned long start_addr = __sec_syscall_set_text_vars_valid_callers[i];
			unsigned long end_addr;

			if (!start_addr) {
				continue;
			}

			end_addr = start_addr + RS_SET_TEXT_VARS_MAX_CALLER_FUNC_SIZE;

			if (likely((caller_addr >= start_addr)
				&& (caller_addr < end_addr))) {
				break;
			}
		}

		if (i >= ARRAY_SIZE(__sec_syscall_set_text_vars_valid_callers)) {
			return -EPERM;
		}
	}
#endif

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

#if defined(CONFIG_MARK_SELINUX_STATE_RDONLY)

#if !(defined(CONFIG_LTO_CLANG) && defined(CONFIG_CFI_CLANG))

typedef unsigned long (*get_addr_redirect_func_type)(void);

extern unsigned long __sec_syscall_get_sel_write_checkreqprot_addr(void);
extern unsigned long __sec_syscall_get_security_load_policycaps_addr(void);
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
extern unsigned long __sec_syscall_get_sel_write_enforce_addr(void);
extern unsigned long __sec_syscall_get_sel_write_em_enforce_addr(void);
#endif /* CONFIG_SECURITY_SELINUX_DEVELOP */

enum ssc_sel_set_text_var_caller_enum {
	ssstvceSelWriteCheckreqprot,
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
	ssstvceSelWriteEnforce,
	ssstvceSelWriteEmEnforce,
#endif /* CONFIG_SECURITY_SELINUX_DEVELOP */
	ssstvceDummy_1,

	/* above entries need redirection */
	ssstvceSecurityLoadPolicy,
#ifdef CONFIG_SECURITY_SELINUX_DISABLE
	ssstvceSELinuxDisable,
#endif /* CONFIG_SECURITY_SELINUX_DISABLE */

	ssstvceNeedRedirectFirst = ssstvceSelWriteCheckreqprot,
	ssstvceNeedRedirectLast = ssstvceDummy_1,

	ssstvceInvalid,
};

extern int security_load_policy(void *data, size_t len);
#ifdef CONFIG_SECURITY_SELINUX_DISABLE
extern int selinux_disable(void);
#endif

noused static const unsigned long __sec_syscall_sel_set_text_vars_valid_callers[] __initconst = {
	(unsigned long)&__sec_syscall_get_sel_write_checkreqprot_addr,
#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
	(unsigned long)&__sec_syscall_get_sel_write_enforce_addr,
	(unsigned long)&__sec_syscall_get_sel_write_em_enforce_addr,
#endif /* CONFIG_SECURITY_SELINUX_DEVELOP */
	(unsigned long)NULL,

	/* above entries need redirection */
	(unsigned long)&security_load_policy,
#ifdef CONFIG_SECURITY_SELINUX_DISABLE
	(unsigned long)&selinux_disable,
#endif /* CONFIG_SECURITY_SELINUX_DISABLE */
};

noused static unsigned long __sec_syscall_sel_set_text_vars_valid_caller_start __rs_rodata;
noused static unsigned long __sec_syscall_sel_set_text_vars_valid_caller_end __rs_rodata;

#define RS_SEL_SET_TEXT_VARS_CALLER_FUNC_SIZE (PAGE_SIZE * 2)

#if defined(CONFIG_RS_OBFUSCATED_NAME)
#define __sec_syscall_sel_set_text_vars_init_caller_range RS_HIDE(_6u)
#endif
noused notrace static int __init __sec_syscall_sel_set_text_vars_init_caller_range(void)
{
	unsigned long _start = (unsigned long)(-1);
	unsigned long _end = 0;
	get_addr_redirect_func_type get_addr_func;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(__sec_syscall_sel_set_text_vars_valid_callers); i++) {
		unsigned long start_addr = __sec_syscall_sel_set_text_vars_valid_callers[i];
		unsigned long end_addr;

		if (!start_addr) {
			continue;
		}

		if ((i >= ssstvceNeedRedirectFirst) && (i <= ssstvceNeedRedirectLast)) {
			get_addr_func = (get_addr_redirect_func_type)start_addr;
			start_addr = (*get_addr_func)();

			if (!start_addr) {
				continue;
			}
		}

		end_addr = start_addr + RS_SEL_SET_TEXT_VARS_CALLER_FUNC_SIZE;

		if (_start > start_addr) {
			_start = start_addr;
		}

		if (_end < end_addr) {
			_end = end_addr;
		}
	}

	if (likely(_start < _end)) {
		*((unsigned long *)&__sec_syscall_sel_set_text_vars_valid_caller_start) = _start;
		*((unsigned long *)&__sec_syscall_sel_set_text_vars_valid_caller_end) = _end;
	} else {
		*((unsigned long *)&__sec_syscall_sel_set_text_vars_valid_caller_start) = _end;
		*((unsigned long *)&__sec_syscall_sel_set_text_vars_valid_caller_end) = _start;
	}

	return 0;
}

core_initcall(__sec_syscall_sel_set_text_vars_init_caller_range); /*pure_initcall*/
#endif

int notrace __sec_syscall_sel_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count)
{
#if !(defined(CONFIG_LTO_CLANG) && defined(CONFIG_CFI_CLANG))
	unsigned long caller_addr = (unsigned long)__builtin_return_address(0);

	if (unlikely((caller_addr < __sec_syscall_sel_set_text_vars_valid_caller_start)
		|| (caller_addr > __sec_syscall_sel_set_text_vars_valid_caller_end))) {
		return -EPERM;
	}
#endif

	return __sec_syscall_set_text_vars(target, source, unit_size, unit_count);
}
#endif /* CONFIG_MARK_SELINUX_STATE_RDONLY */

/*////////////////////////////*/
#if defined(RS_ENCRYPT_STR)
/*
#define OUTPUT(...) do {} while (0)
//#define OUTPUT(...) printf(__VA_ARGS__)

noused notrace static void gen_check_strings(const check_string *array, int array_size, char *fmt_buffer, char *buffer)
{
	int i, j, len;
	for (i = 0; i < array_size; i++) {
		strcpy(fmt_buffer, array[i].str);
		simple_str_encrypt_no_len(fmt_buffer, i);

		if ((estrcmp(fmt_buffer, array[i].str, i) != 0)
			|| (estrncmp(fmt_buffer, array[i].str, array[i].str_len, i) != 0)) {
			OUTPUT("bug!!\n");
		}

		strcpy(buffer, array[i].str);
		simple_str_encrypt_no_len(buffer, i);

		len = 0;
		for (j = 0; j < array[i].str_len; j++) {
			if (j == 0)
				len += sprintf(&fmt_buffer[len], "\"\\x%02X", (unsigned char)buffer[j]);
			else
				len += sprintf(&fmt_buffer[len], "\\x%02X", (unsigned char)buffer[j]);
		}
		strcat(fmt_buffer, "\"\n");

		OUTPUT(fmt_buffer);
	}

}

noused notrace static void gen_op_check_patterns(const op_check_pattern *array, int array_size, char *fmt_buffer, char *buffer)
{
	int i, j, len;
	for (i = 0; i < array_size; i++) {
		strcpy(fmt_buffer, array[i].pattern);
		simple_str_encrypt_no_len(fmt_buffer, i);

		if ((estrcmp(fmt_buffer, array[i].pattern, i) != 0)
			|| (estrncmp(fmt_buffer, array[i].pattern, array[i].pattern_len, i) != 0)) {
			OUTPUT("bug!!\n");
		}

		strcpy(buffer, array[i].pattern);
		simple_str_encrypt_no_len(buffer, i);
		len = 0;
		for (j = 0; j < array[i].pattern_len; j++) {
			if (j == 0)
				len += sprintf(&fmt_buffer[len], "\"\\x%02X", (unsigned char)buffer[j]);
			else
				len += sprintf(&fmt_buffer[len], "\\x%02X", (unsigned char)buffer[j]);
		}
		strcat(fmt_buffer, "\"\n");

		OUTPUT(fmt_buffer);
	}
}

noused notrace static void gen_normal_strings(const char **array, int array_size, char *fmt_buffer, char *buffer)
{
	int i, j, len;
	for (i = 0; i < array_size; i++) {
		int ilen;
		strcpy(fmt_buffer, array[i]);
		simple_str_encrypt_no_len(fmt_buffer, i);

		if ((estrcmp(fmt_buffer, array[i], i) != 0)) {
			OUTPUT("bug!!\n");
		}

		strcpy(buffer, array[i]);
		simple_str_encrypt_no_len(buffer, i);
		len = 0;
		ilen = strlen(array[i]);
		for (j = 0; j < ilen; j++) {
			if (j == 0)
				len += sprintf(&fmt_buffer[len], "\"\\x%02X", (unsigned char)buffer[j]);
			else
				len += sprintf(&fmt_buffer[len], "\\x%02X", (unsigned char)buffer[j]);
		}
		strcat(fmt_buffer, "\"\n");

		OUTPUT(fmt_buffer);
	}
}

noused notrace static void gen_enc_strings(void)
{
	char fmt_buffer[512];
	char buffer[512];

#if 1
	gen_check_strings(check_strings, ARRAY_SIZE(check_strings), fmt_buffer, buffer);
#endif

#if 1
	gen_check_strings(sid_check_strings, ARRAY_SIZE(sid_check_strings), fmt_buffer, buffer);
#endif

#if 1
	gen_op_check_patterns(mnt_allowed_patterns, ARRAY_SIZE(mnt_allowed_patterns), fmt_buffer, buffer);
#endif

#if 1
	gen_op_check_patterns(mnt_disallowed_patterns, ARRAY_SIZE(mnt_disallowed_patterns), fmt_buffer, buffer);
#endif

	gen_normal_strings(su_preload_arguments, ARRAY_SIZE(su_preload_arguments), fmt_buffer, buffer);

#if defined(CONFIG_MOUNT_FILES_CHECK)
	gen_normal_strings(g_system_app_dirs, ARRAY_SIZE(g_system_app_dirs), fmt_buffer, buffer);
	gen_normal_strings(g_system_app_short_dirs, ARRAY_SIZE(g_system_app_short_dirs), fmt_buffer, buffer);
	gen_normal_strings(g_system_app_blacklist_files, ARRAY_SIZE(g_system_app_blacklist_files), fmt_buffer, buffer);
	gen_normal_strings(g_system_bin_dirs, ARRAY_SIZE(g_system_bin_dirs), fmt_buffer, buffer);
	gen_normal_strings(g_system_bin_short_dirs, ARRAY_SIZE(g_system_bin_short_dirs), fmt_buffer, buffer);
	gen_normal_strings(g_system_bin_blacklist_files, ARRAY_SIZE(g_system_bin_blacklist_files), fmt_buffer, buffer);
#endif
}

#if 0
//external/libselinux/src
#define XATTR_NAME_SELINUX "security.selinux"

int setfilecon(const char *path, const security_context_t context)
{
	return setxattr(path, XATTR_NAME_SELINUX, context, strlen(context) + 1, 0);
}


int lsetfilecon(const char *path, const security_context_t context)
{
	return lsetxattr(path, XATTR_NAME_SELINUX, context, strlen(context) + 1, 0);
}

int fsetfilecon(int fd, const security_context_t context)
{
	return fsetxattr(fd, XATTR_NAME_SELINUX, context, strlen(context) + 1, 0);
}

//chcon调用setfilecon
//setxattr,lsetxattr,fsetxattr都调用setxattr(fs/xattr.c)，最终调 vfs_setxattr，需要文件系统可写
#pragma GCC optimize("-O0")
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wunused-variable"
...
#pragma GCC diagnostic pop
#pragma GCC reset_options

#endif
*/

#endif
