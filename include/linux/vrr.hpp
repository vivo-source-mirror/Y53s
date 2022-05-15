#ifndef _VIVO_LINUX_VRR_HPP
#define _VIVO_LINUX_VRR_HPP

#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/version.h>
#include <linux/cache.h>

#ifndef VIVO_DISABLE_ROOTCHECK
#define CONFIG_ROOT_RESTRICT
#endif

#define CONFIG_MOUNT_RESTRICT
	/*#define PASS_MOUNT_RESTRICT*/
	#define CONFIG_MOUNT_BIND_MOVE_RESTRICT /*限制 mount -o bind/move*/

/*#define CONFIG_INSMOD_RESTRICT*/
	/*#define PASS_INSMOD_RESTRICT*/
/*#define CONFIG_SETUID_RESTRICT*/
	/*#define PASS_SETUID_RESTRICT*/

/*#define CONFIG_EXEC_RESTRICT*/
	/*#define PASS_EXEC_RESTRICT*/
	/*only /、/system、/sbin、/vendor、/tmp*/
	#define CONFIG_EXEC_MORE_RESTRICT
	#define CONFIG_EXEC_CHECK_ARG_LATER
	#define CONFIG_EXEC_SKIP_ARGV0
	/*#define CONFIG_EXEC_CHECK_ENV*/

/*#define CONFIG_PTRACE_RESTRICT*/
	/*#define PASS_PTRACE_RESTRICT*/

/*#define CONFIG_CHROOT_RESTRICT*/
	/*#define PASS_CHROOT_RESTRICT*/

/*#define CONFIG_PIVOTROOT_RESTRICT*/
	/*#define PASS_PIVOTROOT_RESTRICT*/

#define CONFIG_ACCESS_DEV_RESTRICT
	/*#define PASS_ACCESS_DEV_RESTRICT*/

/*#define CONFIG_DISABLE_SELINUX_RESTRICT*/
	/*#define PASS_DISABLE_SELINUX_RESTRICT*/

/*#define CONFIG_MOUNT_FILES_CHECK*/
	/*#define PASS_MOUNT_FILES_CHECK*/
	#define CONFIG_MOUNT_CHECK_SU_ONLY /*只删除su文件*/
	/*#define CONFIG_MOUNT_CHECK_FILES_BY_BLACKLIST*/ /*按黑名单删除root产生的特定文件*/
	#define CONFIG_MOUNT_CHECK_FILES_FORCE_RW_BEFORE_RO /*system分区new mount为ro前临时mount成rw并删root文件*/
	/*#define CONFIG_MOUNT_CHECK_FILES_IN_UMOUNT*/ /*umount卸掉挂载前尝试删root文件，system分区一般在使用中卸不掉*/
	/*#define CONFIG_MOUNT_CHECK_FILES_WHEN_SYS_DOWN*/ /*关机前卸掉挂载前尝试删root文件，system分区一般关机前不卸掉*/

#define CONFIG_EXEC_SU_RESTRICT /*不允许执行su程序*/
	/*#define PASS_EXEC_SU_RESTRICT*/
	/*#define CONFIG_EXEC_SU_CHECK_PATH*/ /*检查默认路径，在这些路径中的su不许执行*/
	/*#define CONFIG_EXEC_SU_CHECK_PATH_BY_ENV*/ /*检查PATH环境变量中的路径，在列出路径中的su不许执行*/

/*#define CONFIG_SOCK_RESTRICT*/ /*android 4.3以上，不允许su的daemon调用sock bind(connect?)*/
	/*#define PASS_SOCK_RESTRICT*/
	/*#define SOCK_CHECK_FASTER_BY_GLOBAL_VAR*/ /*在设置了permit或bypass的情况下，检查函数能跑得快些*/
	/*#define SOCK_CHECK_ALLOW_NON_BUILT_IN_PROCESS*/ /*允许非内置程序调用sock bind(su的daemon一般放到system下面)*/

#define CONFIG_LOAD_SELINUX_POLICY_RESTRICT /*限制加载selinux policy*/
	/*#define PASS_LOAD_SELINUX_POLICY_RESTRICT*/

#define CONFIG_RS_MEM_DEV_WRITE_RESTRICT /*不让写入 /dev/mem /dev/kmem*/
	#define CONFIG_RS_MEM_DEV_ALLOW_SPECIAL /*允许系统目录下程序读取*/
	#if !defined(CONFIG_DEVKMEM) && !defined(CONFIG_DEVMEM)
		#undef CONFIG_RS_MEM_DEV_WRITE_RESTRICT
		#undef CONFIG_RS_MEM_DEV_ALLOW_SPECIAL
	#endif

#define CONFIG_RS_KALLSYMS_READ_RESTRICT /*不让读取 /proc/kallsyms*/
	#define CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL /*允许 aee_aed/debuggerd 读取*/
	#if !defined(CONFIG_KALLSYMS)
		#undef CONFIG_RS_KALLSYMS_READ_RESTRICT
		#undef CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL
	#endif

#define CONFIG_MARK_SELINUX_STATE_RDONLY /*将 selinux 关键变量保存到只读区域*/
	#if !defined(CONFIG_SECURITY_SELINUX)
		#undef CONFIG_MARK_SELINUX_STATE_RDONLY
	#endif

/*#define CONFIG_RS_CHECK_BOOT*/ /*检查是否修改过的boot(包括recovery模式)*/

#define CONFIG_RS_CHECK_FUNC /*检查函数代码是否被恶意修改*/

/*#define CONFIG_CHECK_VIVO_FILE*/

#define CONFIG_DO_RS_JOURNAL
	#define CONFIG_RS_JOURNAL_CHECK_NV /*检查nvram is_root标记*/

/*#define CONFIG_RS_BOOT_DEBUG*/ /*调试防root相关异常请打开这个*/

#if !defined(CONFIG_RS_BOOT_DEBUG)
	#define CONFIG_INIT_DELAY_RS_LOG /*mount了"/dev/block"设备之后才开调试log*/
#endif

/*#define CONFIG_CMDLINE_RS_CONFIG*/
#define CONFIG_BLOCK_DEV_RS_CONFIG

#define CONFIG_CONFIG_RS_LOG
#define CONFIG_CONFIG_RS_PERMIT
#define CONFIG_CONFIG_RS_BYPASS
#define CONFIG_CONFIG_RS_JOURNAL

/*backup partition will be mounted with filesystem later*/
#define CONFIG_DONOT_USE_BACKUP_PART

	#if defined(CONFIG_DONOT_USE_BACKUP_PART)
		/*nv items no longer stored as record in backup partition*/
		/* SDM660 ard 8.1 still using nvram record in backup partition */
		/*#undef CONFIG_RS_JOURNAL_CHECK_NV*/
	#endif

#if defined(BBK_FOR_NET_ENTRY)
	/*入网测试安全一级测试需要root权限*/
	#undef CONFIG_ROOT_RESTRICT
#else
	#define RS_NET_ENTRY_MAY_NOT_DEFINE /*android 5.x 平台，有可能入网软件内核里没定义 "BBK_FOR_NET_ENTRY"*/
#endif

#if defined(CONFIG_64BIT) /*(__LONG_MAX__ > 2147483647L) || (BITS_PER_LONG > 32)*/
	#define RS_64BIT_SUPPORT /*64位系统支持*/

	#define RS_WITHOUT_MKSH /*use sh instead of mksh*/
	#define RS_WITHOUT_BBKSU /*bbksu not used any more*/

	/*#undef CONFIG_ROOT_RESTRICT*/ /*64位系统(Android 5.0)未适配，暂时先关闭防root功能*/
#endif


#ifndef CONFIG_ROOT_RESTRICT
	#undef CONFIG_MOUNT_RESTRICT
	#undef CONFIG_INSMOD_RESTRICT
	#undef CONFIG_SETUID_RESTRICT
	#undef CONFIG_EXEC_RESTRICT
		#undef CONFIG_EXEC_MORE_RESTRICT
		#undef CONFIG_EXEC_CHECK_ARG_LATER

	#undef CONFIG_PTRACE_RESTRICT
	#undef CONFIG_CHROOT_RESTRICT
	#undef CONFIG_PIVOTROOT_RESTRICT

	#undef CONFIG_ACCESS_DEV_RESTRICT
	#undef CONFIG_DISABLE_SELINUX_RESTRICT
	#undef CONFIG_MOUNT_FILES_CHECK
	#undef CONFIG_EXEC_SU_RESTRICT

	#undef CONFIG_CHECK_VIVO_FILE
	#undef CONFIG_DO_RS_JOURNAL

	#undef CONFIG_INIT_DELAY_RS_LOG

	#undef CONFIG_CMDLINE_RS_CONFIG
	#undef CONFIG_BLOCK_DEV_RS_CONFIG

	#undef CONFIG_CONFIG_RS_LOG
	#undef CONFIG_CONFIG_RS_PERMIT
	#undef CONFIG_CONFIG_RS_BYPASS
	#undef CONFIG_CONFIG_RS_JOURNAL

	#undef CONFIG_RS_MEM_DEV_WRITE_RESTRICT
	#undef CONFIG_RS_KALLSYMS_READ_RESTRICT
	#undef CONFIG_RS_CHECK_BOOT
	#undef CONFIG_RS_CHECK_FUNC
	#undef CONFIG_SOCK_RESTRICT
	#undef CONFIG_LOAD_SELINUX_POLICY_RESTRICT
	#undef CONFIG_MARK_SELINUX_STATE_RDONLY
#endif

#if !defined(CONFIG_SECURITY_SELINUX)
	/*android 4.3开始带selinux*/
	#undef CONFIG_SOCK_RESTRICT
	#undef CONFIG_LOAD_SELINUX_POLICY_RESTRICT
#endif

#if defined(BBK_FOR_NET_ENTRY)
	/*入网测试安全一级测试需要root权限，允许remount，但保留journal功能*/
	#define CONFIG_ROOT_RESTRICT

	#define CONFIG_MOUNT_RESTRICT
		#define PASS_MOUNT_RESTRICT

	#define CONFIG_DO_RS_JOURNAL
#endif

#if !defined(CONFIG_DO_RS_JOURNAL)
	#undef CONFIG_CONFIG_RS_JOURNAL
	#undef CONFIG_RS_JOURNAL_CHECK_NV
#endif

#define RS_ENCRYPT_STR /*使用字符串加密*/
#define RS_ART_MODE_SUPPORT /*支持ART模式*/

/*#define RS_CHECK_INIT_BY_NAME*/ /*按名称判断init进程*/

#if !defined(CONFIG_RS_BOOT_DEBUG)
	#define DEFAULT_RS_LOG_ENABLE (0) /*默认是否开调试log*/
#else
	#define DEFAULT_RS_LOG_ENABLE (1)
#endif

#define DEFAULT_RS_JOURNAL_ENABLE (1) /*默认是否保存root日志到设备*/


#if !defined(CONFIG_RS_BOOT_DEBUG)
	#define RS_DEBUG (0)  /*是否打开调试log功能*/
#else
	#define RS_DEBUG (1)
#endif

#define RS_TEMP_LOG (1)  /*是否打开临时log功能, DLOG("xxx")*/
#define RS_USE_ABSOLUTE_PATH (1) /*使用绝对路径*/
#define RS_SAFE_GET_TASK_MM (1) /*使用get_task_mm*/

#define RS_USE_D_ABS_PATH (1) /*是否使用d_absolute_path*/
#define RS_HANDLE_PATH_NULL_RETURN (0) /*是否处理d_xx_path返回NULL的情况*/
	#define RS_HANDLE_PATH_NULL_RETURN_AFTERWARD (1) /*在d_xx_path调用后再检查是否NULL*/

#if (!RS_DEBUG)
	#undef CONFIG_INIT_DELAY_RS_LOG
	#undef CONFIG_CONFIG_RS_LOG
#endif

#define RS_DELETE_BY_SYSCALL /*使用sys_unlink/sys_rmdir调用删除文件/目录*/

#if !defined(CONFIG_MOUNT_FILES_CHECK)
	#undef CONFIG_MOUNT_CHECK_FILES_BY_BLACKLIST
#endif

#define RS_VIVOROOT_SUPPORT /*允许 android 5.0 vivoroot*/

#define RS_HAS_OEM_PARTITION /*有oem分区*/
	/*#define RS_OEM_PARTITION_HAS_EXE_FILE*/ /*有放置可执行文件(.so,.ko,.sh ...)在oem分区*/

#define RS_HAS_SURVIVAL_PARTITION /*有survival分区*/

#define RS_HAS_VENDOR_PARTITION /*有vendor分区*/
#define RS_HAS_VBMETA_PARTITION /*有vbmeta分区*/
#define RS_HAS_SUPER_PARTITION /*有super分区, dynamic partitions*/

#define RS_IS_ANDROID_8_1_ABOVE /*ard 8.1 recovery 模式 /tmp/update_binary 改成了 /tmp/update-binary*/
#define RS_IS_ANDROID_9_ABOVE /*ard 9.x 使用了 cmdline 中 androidboot.deviceid=xxx */
#define RS_IS_ANDROID_10_ABOVE /*ard 10.x ES1 在 /bionic/、ES2 在 /apex/ 下有 bin、lib 和 lib64, /sbin/recovery 变 /system/bin/recovery */
#define RS_IS_ANDROID_11_ABOVE /*ard 11.x 除了 adbd，vivo 还另外加了 vadbd */

/*#define RS_SU_MORE_RESTRICT*/ /*加强对su的限制*/

#define RS_USE_UFS_DEV /*可能使用UFS而不是EMMC*/
#if defined(RS_IS_ANDROID_10_ABOVE)
	#define RS_USE_DEV_BLOCK_BY_NAME /*使用/dev/block/by-name/xxx, ard 10.0 及以后可用 */
#endif
#define RS_HAS_TOYBOX /*文件/system/bin/toybox存在*/

#define RS_USE_DEVINFO_PART /*高通dev_info可能保存在devinfo分区, 5.0虽有分区但未存, 5.1+存了*/

/*#define RS_NAME_NODE_USE_PAGE*/ /*遍历目录时 name 结点使用 page 分配*/
/*#define RS_READ_DIR_USE_FILP*/ /*vfs_readdir用的file使用filp_open打开,否则使用dentry_open打开*/
/*#define RS_ELEVATE_CREDS*/ /*临时提升权限*/
/*#define RS_ABS_PATH_CHECK_SYMLINK*/ /*对使用绝对路径的文件也额外文件所链接到的文件，会多调用一次kern_path()*/

/*#define RS_SET_PERSIST_PROPERTY*/ /*允许设置persist属性*/
/*#define RS_DEBUG_BAD_RECOVERY*/ /*调试is_bad_recovery()相关功能*/

#define RS_DO_MOUNT_USE_USER_DIR_NAME /*android 6.0 do_mount() 中 dir_name 为 user 数据*/

/*#define RS_VARS_LOCK_USE_RWSEM*/ /*使用 read/write semaphore 保护互斥访问 g_rs_op_var_xxx 数据,上锁后允许被抢占,效率稍低*/

#define RS_CHEKC_VIVO_SECURE_BOOT_BY_GLOBAL_VAR /*直接访问 sec_enable 变量判断 secure boot 是否开启*/

#define RS_EMMC_DEV_PATH_HAS_SOC /* /dev/block/platform/soc/7829400.sdhci*/

#define RS_UFS_DEV_PATH_HAS_SOC /* /dev/block/platform/soc/624000.ufshc/ */

#define RS_ITERATE_BLK_PLATFORM_DIR_RECURSIVELY /*递归搜索/dev/block/platform目录,定位xxx/mmcblk0子目录*/

	#define RS_ITERATE_BLK_PLATFORM_DIR_MAX_DEPTH (3)

#if defined(RS_ITERATE_BLK_PLATFORM_DIR_RECURSIVELY)
	#undef RS_EMMC_DEV_PATH_HAS_SOC
#endif

/*#define RS_DEBUG_RODATA_BACKPORT*/ /*android 6.x CONFIG_DEBUG_RODATA 移植到 5.x*/

#define RS_FAST_FLAG_SUPPORT /*store global flag as fast flag*/
	#define RS_FAST_FLAG_WRITE_PROTECT /*store fast flag values in read-only area*/
	#define RS_FAST_FLAG_CHECK_CRC /*check crc for fast flag value*/
	#if defined(CONFIG_VIVO_ALLOW_REBOOT_REMOUNT)
		/*vivo_em_svr会一直轮询/proc/isroot节点,所以开了会比不开耗电*/
		/*#define RS_ALLOW_REMOUNT_IN_FAST_FLAG*/ /*store allow_remount flag as fast flag*/
	#endif

/*#define RS_AB_UPDATE_SUPPORT*/ /*support A/B system update*/

#define RS_SYSTEM_AS_ROOT_SUPPORT /*support BOARD_BUILD_SYSTEM_ROOT_IMAGE*/

#include <linux/ctype.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
#include <linux/sched/mm.h>
#endif

#if defined(CONFIG_ARM64)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	/*entry.S call el0_svc_handler()*/
	#define RS_ARCH_USE_SYSCALL_WRAPPER
#endif
#endif

#if defined(CONFIG_ARCH_EXYNOS)
	#define RS_SAMSANG_PLATFORM

	#if !defined(CONFIG_DEBUG_SNAPSHOT_USER_MODE)
		#define RS_IS_ENG_BUILD
	#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 72)
	#define RS_NO_VFS_READDIR
#endif

	#undef RS_CHEKC_VIVO_SECURE_BOOT_BY_GLOBAL_VAR

	#if defined(CONFIG_DONOT_USE_BACKUP_PART) && defined(RS_IS_ANDROID_9_ABOVE)
		#define CONFIG_USE_RESERVED_PART
	#endif

	#define RS_HAS_DEV_BYNAME_SHORTCUTS /* /dev/block/platform/xxxx/by-name/xxx */

	#undef RS_EMMC_DEV_PATH_HAS_SOC
	#undef RS_UFS_DEV_PATH_HAS_SOC /* /dev/block/platform/13600000.ufs/by-name/xxx */

#elif defined(CONFIG_MTK_RTC) || defined(CONFIG_MTK_PLATFORM)
	#define RS_MTK_PLATFORM

/*比较按不同TARGET_BUILD_VARIANT编译出来的 .config 文件*/
#if defined(CONFIG_MT_ENG_BUILD) \
	|| (defined(CONFIG_OPROFILE) && defined(CONFIG_KPROBES) && defined(CONFIG_KGDB)) /*eng, 老版本无CONFIG_MT_ENG_BUILD*/
	/*|| (defined(CONFIG_MTK_SCHED_TRACERS) && defined(CONFIG_RCU_TRACE))*/ /*userdebug*/
	#define RS_IS_ENG_BUILD
#endif

/*mt6752 4.4.4, linux 3.10.48, 没有了 /dev/backup 和 /emmc@android, /dev/nvram等变成软链接. 之前有的是 3.4.xx*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	#define RS_HAS_DEV_SHORTCUTS
	#define RS_HAS_EMMC_SHORTCUTS
#else
	#define RS_HAS_DEV_BYNAME_SHORTCUTS /* /dev/block/platform/xxxx/by-name/xxx*/
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 72)
	#define RS_NO_VFS_READDIR
#endif

	#undef RS_CHEKC_VIVO_SECURE_BOOT_BY_GLOBAL_VAR

	#if defined(CONFIG_DONOT_USE_BACKUP_PART) && defined(RS_IS_ANDROID_9_ABOVE)
		#define CONFIG_USE_RESERVED_PART /* MTK 9.0 already using tailing portion of expdb partition */
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

	#define RS_HAS_DEV_BYNAME_SHORTCUTS

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 84)
	#define RS_NO_VFS_READDIR
#endif

	#undef CONFIG_RS_KALLSYMS_READ_ALLOW_SPECIAL
#endif

#if defined(RS_IS_ENG_BUILD)
	/*调试版本软件不删system下su*/
	#undef CONFIG_MOUNT_FILES_CHECK
	#undef CONFIG_RS_MEM_DEV_WRITE_RESTRICT
	#undef CONFIG_RS_KALLSYMS_READ_RESTRICT
#endif

#if !defined(RS_HAS_OEM_PARTITION)
	#undef RS_OEM_PARTITION_HAS_EXE_FILE
#endif

#if defined(CONFIG_KALLSYMS) && !defined(CONFIG_RS_OBFUSCATED_NAME) /*&& !defined(RS_IS_ENG_BUILD)*/
	#define CONFIG_RS_OBFUSCATED_NAME /*变量、函数名字混淆化，被别的文件引用的不能混淆，暂时动态修改*/
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
	#define RS_DO_MMAP_PGOFF_NO_POPULATE
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

#undef RS_HAS_QCOM_SECURE_BUFFER

#if defined(RS_QUALCOMM_PLATFORM) && defined(CONFIG_MSM_SECURE_BUFFER) && defined(CONFIG_MSM_KERNEL_PROTECT)
#include <soc/qcom/secure_buffer.h>

	#define RS_HAS_QCOM_SECURE_BUFFER
#endif

#if defined(CONFIG_MOUNT_RESTRICT) || (defined(CONFIG_BLOCK_DEV_RS_CONFIG) /*|| defined(RS_QUALCOMM_PLATFORM)*/ || defined(CONFIG_INIT_DELAY_RS_LOG))
	#define RS_NEED_DO_MOUNT_CHECK
#endif

#if defined(CONFIG_RS_CHECK_FUNC)
	#define RS_NEED_DO_BOOT_CHECK
#endif

#define RS_MARK_VAR_READONLY

#if !defined(RS_HIDE)
	#if 1 /*defined(CONFIG_RS_OBFUSCATED_NAME)*/
		/*有可能把user版判为eng版？*/
		#define RS_HIDE(name) name##_v_r_s_h_s
	#else
		#define RS_HIDE(name) name
	#endif
#endif

#if !defined(noused)
	#define noused __attribute__((unused))
#endif

#if !defined(__rodata)
#if defined(__section)
	#define	__rodata	__section(.rodata)
#else
	#define	__rodata	__attribute__((section(".rodata")))
#endif
#endif

#if defined(RS_BOOT_DEBUG) || !defined(CONFIG_ROOT_RESTRICT)
	#define RS_EXPORT static
#else
	#define RS_EXPORT __weak
#endif

#if !defined(__rs_weak)
	#if defined(__weak)
		#define	__rs_weak	__weak
	#else
		#define	__rs_weak	__attribute__((weak))
	#endif
#endif

/*/////////////////////////////////////////////////////////////////*/

/* fs/block_dev.c */

#define do_access_blkdev_check RS_HIDE(_vrpb_1)

#if defined(CONFIG_ACCESS_DEV_RESTRICT)
	#define	VIVO_DECLARE_DO_ACCESS_BLKDEV_CHECK_FUNC	\
		extern int do_access_blkdev_check(struct gendisk *disk, struct hd_struct *part, fmode_t mode)

	#define	VIVO_DO_ACCESS_BLKDEV_CHECK(disk, part, mode)	\
		do_access_blkdev_check((disk), (part), (mode))

#else
	#define	VIVO_DECLARE_DO_ACCESS_BLKDEV_CHECK_FUNC
	#define	VIVO_DO_ACCESS_BLKDEV_CHECK(disk, part, mode)	(0)
#endif

/* fs/exec.c */

#define do_execve_common_check RS_HIDE(_vrpb_2)
#define do_execve_common_file_check RS_HIDE(_vrpb_3)

#if defined(CONFIG_EXEC_RESTRICT) || defined(CONFIG_EXEC_SU_RESTRICT)
	#define	VIVO_DECLARE_DO_EXECVE_COMMON_CHECK_FUNC	\
		extern int do_execve_common_check(const char *filename, struct user_arg_ptr argv);

	#define	VIVO_DO_EXECVE_COMMON_CHECK(filename, argv)	\
		do_execve_common_check((filename), (argv))

#else
	#define	VIVO_DECLARE_DO_EXECVE_COMMON_CHECK_FUNC
	#define	VIVO_DO_EXECVE_COMMON_CHECK(filename, argv)	(0)
#endif

#if defined(CONFIG_EXEC_SU_RESTRICT)
	#define	VIVO_DECLARE_DO_EXECVE_COMMON_FILE_CHECK_FUNC	\
		extern int do_execve_common_file_check(const struct file *file);

	#define	VIVO_DO_EXECVE_COMMON_FILE_CHECK(file)	\
		do_execve_common_file_check((file))

#else
	#define	VIVO_DECLARE_DO_EXECVE_COMMON_FILE_CHECK_FUNC
	#define	VIVO_DO_EXECVE_COMMON_FILE_CHECK(file)	(0)
#endif

/* fs/open.c */

#define do_chroot_check RS_HIDE(_vrpb_4)

#if defined(CONFIG_CHROOT_RESTRICT)
	#define	VIVO_DECLARE_DO_CHROOT_CHECK_FUNC	\
		extern int do_chroot_check(void);

	#define	VIVO_DO_CHROOT_CHECK()	\
		do_chroot_check()
#else
	#define	VIVO_DECLARE_DO_CHROOT_CHECK_FUNC
	#define	VIVO_DO_CHROOT_CHECK()	(0)
#endif

/* fs/namespace.c */

#define do_pivotroot_check RS_HIDE(_vrpb_5)

#if defined(CONFIG_PIVOTROOT_RESTRICT)
	#define	VIVO_DECLARE_DO_PIVOTROOT_CHECK_FUNC	\
		extern int do_pivotroot_check(void);

	#define	VIVO_DO_PIVOTROOT_CHECK()	\
		do_pivotroot_check()
#else
	#define	VIVO_DECLARE_DO_PIVOTROOT_CHECK_FUNC
	#define	VIVO_DO_PIVOTROOT_CHECK()	(0)
#endif

/* kernel/exit.c */

#define get_task_reap_init_flag RS_HIDE(_vrpb_6)

#if defined(CONFIG_ROOT_RESTRICT)

	#define	VIVO_DEFINE_VARS_FOR_REPARENT	\
		extern unsigned int get_task_reap_init_flag(void);	\
		unsigned int reap_flag;	\
		unsigned int flags;	\
		int reaper_to_init;

	#define	VIVO_INIT_VARS_FOR_REPARENT(father)	\
		reap_flag = get_task_reap_init_flag();	\
		flags = father->flags;	\
		if (flags & reap_flag) {	\
			reaper_to_init = 1;	\
		} else {	\
			reaper_to_init = ((flags & (PF_FORKNOEXEC | PF_SIGNALED | PF_KTHREAD | PF_SUPERPRIV/* | PF_RANDOMIZE*/)) == PF_FORKNOEXEC) ? -1 : 0;	\
		}

static inline void vivo_update_vars_for_reparent(int reaper_to_init, struct task_struct *father,
	struct task_struct *reaper, unsigned int flags)
{
	if (reaper_to_init < 0) {
		reaper_to_init = ((father->real_parent != reaper) && is_global_init((reaper))) ? 1 : 0; /* && ((father)->exit_code != (127 << 8));*/
	}

	/*
	if (reaper_to_init) {
		printk("rs: reap, [%d:%s], %x\n", father->pid, father->comm, flags);
	}
	*/
}

	#define	VIVO_UPDATE_VARS_FOR_REPARENT(father, reaper)	\
		vivo_update_vars_for_reparent(reaper_to_init, (father), (reaper), flags)

	#define	VIVO_UPDATE_THREAD_FLAGS(t)	\
		if (reaper_to_init) {	\
			(t)->flags |= reap_flag;	\
		}

#else
	#define	VIVO_DEFINE_VARS_FOR_REPARENT
	#define	VIVO_INIT_VARS_FOR_REPARENT
	#define	VIVO_UPDATE_VARS_FOR_REPARENT(father, reaper)	do {} while(0)
	#define	VIVO_UPDATE_THREAD_FLAGS(t)	do {} while(0)
#endif


/* kernel/moudle.c */

#define do_insmod_check RS_HIDE(_vrpb_8)

#if defined(CONFIG_INSMOD_RESTRICT)
	#define	VIVO_DECLARE_DO_INSMOD_CHECK_FUNC	\
		extern int do_insmod_check(int fd);

	#define	VIVO_DO_INSMOD_CHECK(fd)	\
		do_insmod_check((fd))

#else
	#define	VIVO_DECLARE_DO_INSMOD_CHECK_FUNC
	#define	VIVO_DO_INSMOD_CHECK(fd)	(0)
#endif

/* kernel/ptrace.c */

#define do_ptrace_check RS_HIDE(_vrpb_9)

#if defined(CONFIG_PTRACE_RESTRICT)
	#define	VIVO_DECLARE_DO_PTRACE_CHECK_FUNC	\
		extern int do_ptrace_check(struct task_struct *task)

	#define	VIVO_DO_PTRACE_CHECK(task)	\
		do_ptrace_check((task))
#else
	#define	VIVO_DECLARE_DO_PTRACE_CHECK_FUNC
	#define	VIVO_DO_PTRACE_CHECK(task)	(0)
#endif

/* kernel/sys.c */

#define do_uid_check RS_HIDE(_vrpb_10)

#if defined(CONFIG_SETUID_RESTRICT)
	#define	VIVO_DECLARE_DO_UID_CHECK_FUNC	\
		extern int do_uid_check(void)

	#define	VIVO_DO_UID_CHECK()	\
		do_uid_check()
#else
	#define	VIVO_DECLARE_DO_UID_CHECK_FUNC
	#define	VIVO_DO_UID_CHECK()	(0)
#endif

/* security/selinux/selinuxfs.c */

#define do_disable_selinux_check RS_HIDE(_vrpb_20)

#if defined(CONFIG_DISABLE_SELINUX_RESTRICT) && defined(CONFIG_SECURITY_SELINUX_DEVELOP)
	#define	VIVO_DECLARE_DO_DISABLE_SELINUX_CHECK_FUNC	\
		extern int do_disable_selinux_check(void)

	#define	VIVO_DO_DISABLE_SELINUX_CHECK()	\
		do_disable_selinux_check()
#else
	#define	VIVO_DECLARE_DO_DISABLE_SELINUX_CHECK_FUNC
	#define	VIVO_DO_DISABLE_SELINUX_CHECK()	(0)
#endif

/* security/selinux/ss/services.c */

#define do_load_selinux_policy_check RS_HIDE(_vrpb_21)

#if defined(CONFIG_LOAD_SELINUX_POLICY_RESTRICT)
	#define	VIVO_DECLARE_DO_LOAD_SELINUX_POLICY_CHECK_FUNC	\
		extern  int do_load_selinux_policy_check(void)

	#define	VIVO_DO_LOAD_SELINUX_POLICY_CHECK()	\
		do_load_selinux_policy_check()
#else
	#define	VIVO_DECLARE_DO_LOAD_SELINUX_POLICY_CHECK_FUNC
	#define	VIVO_DO_LOAD_SELINUX_POLICY_CHECK()	(0)
#endif

#if defined(CONFIG_MARK_SELINUX_STATE_RDONLY)

#if defined(__ro_after_init)
	#define	__rs_rodata	__ro_after_init
#else /* __ro_after_init */
	#define	__rs_rodata	__rodata
#endif /* !__ro_after_init */

#define	__sec_syscall_sel_set_text_vars RS_HIDE(_vrpb_30)
extern int __sec_syscall_sel_set_text_vars(void *target, void *source, size_t unit_size, size_t unit_count);

#define	VIVO_SEL_SET_RO_VARS(target, source, unit_size, unit_count)	__sec_syscall_sel_set_text_vars((target), (source), (unit_size), (unit_count))

#endif

/*////////////////////////////////////////*/
#if defined(CONFIG_SECURITY_SELINUX)
/* for public functions name obfuscation,
 should sync with security/selinux/include/vrt.hpp
 */

/* fs/vrr.c */
#if !defined(rs_can_exec)
	#define rs_can_exec RS_HIDE(_vrpb_51)
#endif
#if !defined(is_dbg_task)
	#define is_dbg_task RS_HIDE(_vrpb_52)
#endif
#if !defined(set_rs_s_u)
	#define set_rs_s_u RS_HIDE(_vrpb_53)
#endif

/* security/selinux/XXX */
#if !defined(vr_is_exec)
	#define vr_is_exec RS_HIDE(_vrpb_60)
#endif
#if !defined(vr_try_update_policy)
	#define vr_try_update_policy RS_HIDE(_vrpb_61)
#endif
#if !defined(rs_sec_bounded_trans)
	#define rs_sec_bounded_trans RS_HIDE(_vrpb_62)
#endif

#endif /* CONFIG_SECURITY_SELINUX */
/*////////////////////////////////////////*/

#endif /* _VIVO_LINUX_VRR_HPP */

