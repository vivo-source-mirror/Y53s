
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/version.h>

#include <linux/unistd.h>
/*#include <asm/unistd.h>*/

#include <linux/securebits.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
#include <linux/uidgid.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)
#include <linux/user_namespace.h>
#endif

#if !defined(CONFIG_ARM64)
#include <asm/unified.h>
#endif

#define SEC_SYSCALL_INCLUDE_HEADER_FILES
#include "ssc.hpp"
#undef SEC_SYSCALL_INCLUDE_HEADER_FILES


#if defined(SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL)
#include <generated/vivo_uts_timestamp.h>
#endif

#if defined(SEC_SYSCALL_CHECK_UID_CHANGE)
	#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
		#define SSC_CUC_FLAG_TYPE unsigned int /*typedef unsigned int SSC_CUC_FLAG_TYPE;*/

		#define SSC_CUC_FLAG_BITS (sizeof(SSC_CUC_FLAG_TYPE) << 3)
		#define SSC_CUC_FLAG_BIT_MASK(nr) (1 << ((nr) % SSC_CUC_FLAG_BITS))
		#define SSC_CUC_FLAG_BIT_WORD(nr) ((nr) / SSC_CUC_FLAG_BITS)

		/*(SSC_CUC_FLAG_BITS - 1) already used by namespace.c get_task_dbg_flag()*/
		#define SEC_SYSCALL_UID_MAY_CHANGE_FLAG (SSC_CUC_FLAG_BITS - 2)
		#define SEC_SYSCALL_SID_MAY_CHANGE_FLAG (SSC_CUC_FLAG_BITS - 3)
	#else
		#define SEC_SYSCALL_UID_MAY_CHANGE_FLAG (BITS_PER_LONG - 2)
		#define SEC_SYSCALL_SID_MAY_CHANGE_FLAG (BITS_PER_LONG - 3)
	#endif
#endif

#if !defined(thread_pt_regs)
	#define thread_pt_regs(thread_info) \
		((struct pt_regs *)(THREAD_START_SP + (unsigned long)(thread_info)) - 1)
#endif

#if defined(SEC_SYSCALL_CHECK_SELINUX_SID)
#if !defined(SEC_SYSCALL_DEFINE_SELINUX_SECURITY_TYPES)
	/*#if defined(CONFIG_SECURITY) && defined(CONFIG_SECURITY_SELINUX)*/
		#include "../../../security/selinux/include/objsec.h"
	/*#endif*/
#else /*!SEC_SYSCALL_DEFINE_SELINUX_SECURITY_TYPES*/
struct task_security_struct {
	u32 osid;		/* SID prior to last execve */
	u32 sid;		/* current SID */
	u32 exec_sid;		/* exec SID */
	u32 create_sid;		/* fscreate SID */
	u32 keycreate_sid;	/* keycreate SID */
	u32 sockcreate_sid;	/* fscreate SID */
};
#endif /*SEC_SYSCALL_DEFINE_SELINUX_SECURITY_TYPES*/
#endif /*SEC_SYSCALL_CHECK_SELINUX_SID*/

#if !defined(__NR_syscalls) && defined(CONFIG_ARM)
	/*#define __NR_syscalls (381)*/ /*before kernel 3.5*/
	extern size_t __sec_syscall_NR_syscalls;
	#define __NR_syscalls __sec_syscall_NR_syscalls
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
	#define kuid_t uid_t
	#define kgid_t gid_t
	#define __kuid_val(uid) (uid)
	#define __guid_val(gid) (gid)
	#define uid_eq(left, right) ((left) == (right))
	#define gid_eq(left, right) ((left) == (right))
	#define uid_ne(left, right) ((left) != (right))
	#define gid_ne(left, right) ((left) != (right))
	#define from_kuid_munged(user_ns, uid) (uid)
	#define from_kgid_munged(user_ns, gid) (gid)

	#if !defined(GLOBAL_ROOT_UID)
		#define GLOBAL_ROOT_UID (0)
	#endif

#else
static inline bool uid_ne(kuid_t left, kuid_t right)
{
	return __kuid_val(left) != __kuid_val(right);
}

static inline bool gid_ne(kgid_t left, kgid_t right)
{
	return __kgid_val(left) != __kgid_val(right);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
	#define RS_GET_RANDOM_BYTES get_random_bytes
#else
	#define RS_GET_RANDOM_BYTES get_random_bytes_arch
#endif

#if defined(SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL)

#if !defined(VIVO_UTS_TIMESTAMP)
	#define VIVO_UTS_TIMESTAMP (0)
#endif

#define IN_SECOND_STAGE_FLAGS_COUNT (((LINUX_VERSION_CODE + VIVO_UTS_TIMESTAMP) % 4) + 3)
#define IN_SECOND_STAGE_INDEX (((LINUX_VERSION_CODE + VIVO_UTS_TIMESTAMP) + 3) % IN_SECOND_STAGE_FLAGS_COUNT)

noused static bool __sec_syscall_init_is_in_second_stage_flags[IN_SECOND_STAGE_FLAGS_COUNT] = { false };
#endif /*SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL*/

noused static bool __sec_syscall_pre_check_dump_cred_flag;
noused static bool __sec_syscall_post_check_dump_cred_flag;
noused static int __sec_syscall_pre_check_fail_reason;
noused static int __sec_syscall_post_check_fail_reason;

#if defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
	#define PRE_CHECK_FAIL(err) (__sec_syscall_pre_check_fail_reason = (err))
	#define POST_CHECK_FAIL(err) (__sec_syscall_post_check_fail_reason = (err))
#else /*SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/
	#define PRE_CHECK_FAIL(err)
	#define POST_CHECK_FAIL(err)
#endif /*!SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

#define SEC_SYSCALL_DUMP_CRED_LABEL "ssc"
#define SEC_SYSCALL_PRE_CHECK_DUMP_CRED_LABEL "pre_ssc"
#define SEC_SYSCALL_POST_CHECK_DUMP_CRED_LABEL "post_ssc"

#include <linux/init_task.h>
#include <linux/capability.h>

#if defined(SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP)
#include <linux/cgroup.h>
#endif /*SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP*/

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

#define RS_MARK_VAR_READONLY

#if !defined(RS_MARK_VAR_READONLY)
#define __rs_text
#else
#define __rs_text __attribute__((__section__(".text")))
#endif

#include "../../../fs/mount.h" /*for real_mount(), mnt_has_parent()*/
#include <linux/namei.h>


noused notrace static int __sec_syscall_prepend(char **buffer, int *buflen, const char *str, int namelen)
{
	*buflen -= namelen;
	if (*buflen < 0)
		return -ENAMETOOLONG;
	*buffer -= namelen;
	memcpy(*buffer, str, namelen);
	return 0;
}

noused notrace static int __sec_syscall_prepend_name(char **buffer, int *buflen, struct qstr *name)
{
	return __sec_syscall_prepend(buffer, buflen, name->name, name->len);
}

noused notrace static int __sec_syscall_prepend_first_level_path_no_lock(const struct path *path,
			const struct path *root,
			char **buffer, int *buflen)
{
	struct dentry *dentry = path->dentry;
	struct vfsmount *vfsmnt = path->mnt;
	struct dentry *self_dentry = dentry;
	struct mount *mnt = real_mount(vfsmnt);
	int error = 0;
	struct dentry *child_dentry = NULL;

	while (dentry != root->dentry || vfsmnt != root->mnt) {

		if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) {
			/* Global root? */
			if (!mnt_has_parent(mnt)) {
				break;
			}
			dentry = mnt->mnt_mountpoint;
			mnt = mnt->mnt_parent;
			vfsmnt = &mnt->mnt;
			continue;
		}

		child_dentry = dentry;

		dentry = dentry->d_parent;
	}

	if ((child_dentry) && (child_dentry != self_dentry)) {
		error = __sec_syscall_prepend_name(buffer, buflen, &child_dentry->d_name);
	}

	if (!error) {
		error = __sec_syscall_prepend(buffer, buflen, "/", 1);
	}

	return error;
}

noused notrace static struct dentry *__sec_syscall_get_first_level_dentry_no_lock(const struct path *path,
			const struct path *root)
{
	struct dentry *dentry = path->dentry;
	struct vfsmount *vfsmnt = path->mnt;
	struct dentry *self_dentry = dentry;
	struct mount *mnt = real_mount(vfsmnt);
	struct dentry *child_dentry = NULL;

	while (dentry != root->dentry || vfsmnt != root->mnt) {

		if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) {
			/* Global root? */
			if (!mnt_has_parent(mnt)) {
				break;
			}
			dentry = mnt->mnt_mountpoint;
			mnt = mnt->mnt_parent;
			vfsmnt = &mnt->mnt;
			continue;
		}

		child_dentry = dentry;

		dentry = dentry->d_parent;
	}

	if ((child_dentry) && (child_dentry != self_dentry)) {
		return child_dentry;
	}

	return NULL;
}

noused notrace static char *__sec_syscall_d_abs_first_level_path_no_lock(const struct path *path, char *buf, int buflen)
{
	struct path root = {};
	char *res = buf + buflen;
	int error;

	/*
	if (path->dentry->d_op && path->dentry->d_op->d_dname &&
		(!IS_ROOT(path->dentry) || path->dentry != path->mnt->mnt_root))
		return path->dentry->d_op->d_dname(path->dentry, buf, buflen);
	*/

	__sec_syscall_prepend(&res, &buflen, "\0", 1);
	error = __sec_syscall_prepend_first_level_path_no_lock(path, &root, &res, &buflen);
	/*
	if (error > 1)
		error = -EINVAL;
	*/
	if (error < 0)
		return NULL; /*return ERR_PTR(error);*/
	return res;
}

noused notrace static struct dentry *__sec_syscall_d_abs_first_level_dentry_no_lock(const struct path *path)
{
	struct path root = {};

	return __sec_syscall_get_first_level_dentry_no_lock(path, &root);
}

#if defined(SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT)

/* other mode */
#define SSC_OTM_PATH_LEN_XOR_VALUE (0x55)
#define SSC_OTM_PATH_CHAR_XOR_VALUE (0xAA)

#define SSC_OTM_PATH_LEN(len) ((unsigned char)(len) ^ SSC_OTM_PATH_LEN_XOR_VALUE)
#define SSC_OTM_PATH_CHAR(ch, i) ((unsigned char)(ch) ^ (SSC_OTM_PATH_CHAR_XOR_VALUE + (i)))

#define SSC_OTM_MIN_PATH_LEN (sizeof("tmp") - 1)

noused static const unsigned char g_other_mode_valid_path_bases[] = {
	/* sbin */
	SSC_OTM_PATH_LEN(sizeof("sbin") - 1),
	SSC_OTM_PATH_CHAR('s', 0), SSC_OTM_PATH_CHAR('b', 1), SSC_OTM_PATH_CHAR('i', 2), SSC_OTM_PATH_CHAR('n', 3),
	/* tmp */
	SSC_OTM_PATH_LEN(sizeof("tmp") - 1),
	SSC_OTM_PATH_CHAR('t', 0), SSC_OTM_PATH_CHAR('m', 1), SSC_OTM_PATH_CHAR('p', 2),
	/* system */
	SSC_OTM_PATH_LEN(sizeof("system") - 1),
	SSC_OTM_PATH_CHAR('s', 0), SSC_OTM_PATH_CHAR('y', 1), SSC_OTM_PATH_CHAR('s', 2), SSC_OTM_PATH_CHAR('t', 3),
	SSC_OTM_PATH_CHAR('e', 4), SSC_OTM_PATH_CHAR('m', 5),

	/* null terminator, not needed */
	/*SSC_OTM_PATH_LEN(0), */
};


noused notrace static bool __sec_syscall_other_mode_is_allowed_executable_by_path(const struct path *_path)
{
	char path_buffer[(16 + 1)/*(NAME_MAX + 2)*/]; /*plus leading '/' and tailing '\0' */
	char *path;

	path = __sec_syscall_d_abs_first_level_path_no_lock(_path, path_buffer, sizeof(path_buffer));
	if (path) {
		if (path[0] == '/') {
			if ((*(++path)) == '\0') {
				/*is under fs root*/
				return true;
			}
		}

		{
			unsigned int name_len = path_buffer + sizeof(path_buffer) - path - 1;
			unsigned char const *p_path = g_other_mode_valid_path_bases;
			unsigned char const *p_path_end = p_path + ARRAY_SIZE(g_other_mode_valid_path_bases);

			while (p_path < p_path_end) {
				unsigned int len;

				len = SSC_OTM_PATH_LEN(*p_path++);
				if (!len) {
					break;
				}

				if (name_len == len) {
					unsigned int i;

					for (i = 0; i < len; i++) {
						if (path[i] != SSC_OTM_PATH_CHAR(p_path[i], i)) {
							break;
						}
					}

					if (i >= len) {
						return true;
					}
				}

				p_path += len;
			}
		}
	}

	return false;
}

noused notrace static bool __sec_syscall_other_mode_is_allowed_executable_by_dentry(const struct path *_path)
{
	bool ret;
	struct dentry *dentry = __sec_syscall_d_abs_first_level_dentry_no_lock(_path);

	if (!dentry) {
		ret = true;
	} else {
		unsigned int name_len = dentry->d_name.len;
		const unsigned char *pName = dentry->d_name.name;

		ret = false;

		if (likely((pName) && (name_len >= SSC_OTM_MIN_PATH_LEN))) {
			unsigned char const *p_path = g_other_mode_valid_path_bases;
			unsigned char const *p_path_end = p_path + ARRAY_SIZE(g_other_mode_valid_path_bases);

			while (p_path < p_path_end) {
				unsigned int len;

				len = SSC_OTM_PATH_LEN(*p_path++);
				if (!len) {
					break;
				}

				if (name_len == len) {
					unsigned int i;

					for (i = 0; i < len; i++) {
						if (pName[i] != SSC_OTM_PATH_CHAR(p_path[i], i)) {
							break;
						}
					}

					if (i >= len) {
						ret = true;
						break;
					}
				}

				p_path += len;
			}
		}
	}

	return ret;
}

#endif

/* normal mode */
#define SSC_NOM_PATH_LEN_XOR_VALUE (0x5A)
#define SSC_NOM_PATH_CHAR_XOR_VALUE (0xA5)

#define SSC_NOM_PATH_LEN(len) ((unsigned char)(len) ^ SSC_NOM_PATH_LEN_XOR_VALUE)
#define SSC_NOM_PATH_CHAR(ch, i) ((unsigned char)(ch) ^ (SSC_NOM_PATH_CHAR_XOR_VALUE + (i)))

#define SSC_NOM_MIN_PATH_LEN (sizeof("system") - 1) /*(sizeof("sbin") - 1) */

noused static const unsigned char g_normal_mode_valid_path_bases[] = {
	/* system */
	SSC_NOM_PATH_LEN(sizeof("system") - 1),
	SSC_NOM_PATH_CHAR('s', 0), SSC_NOM_PATH_CHAR('y', 1), SSC_NOM_PATH_CHAR('s', 2), SSC_NOM_PATH_CHAR('t', 3),
	SSC_NOM_PATH_CHAR('e', 4), SSC_NOM_PATH_CHAR('m', 5),

#if 0
	/* sbin */
	SSC_NOM_PATH_LEN(sizeof("sbin") - 1),
	SSC_NOM_PATH_CHAR('s', 0), SSC_NOM_PATH_CHAR('b', 1), SSC_NOM_PATH_CHAR('i', 2), SSC_NOM_PATH_CHAR('n', 3),

	/* vendor */
	SSC_NOM_PATH_LEN(sizeof("vendor") - 1),
	SSC_NOM_PATH_CHAR('v', 0), SSC_NOM_PATH_CHAR('e', 1), SSC_NOM_PATH_CHAR('n', 2), SSC_NOM_PATH_CHAR('d', 3),
	SSC_NOM_PATH_CHAR('o', 4), SSC_NOM_PATH_CHAR('r', 5),
#endif

	/* null terminator, not needed */
	/* SSC_NOM_PATH_LEN(0), */
};

noused notrace static bool __sec_syscall_normal_mode_is_allowed_executable_by_dentry(const struct path *_path)
{
	bool ret;
	struct dentry *dentry = __sec_syscall_d_abs_first_level_dentry_no_lock(_path);
	if (!dentry) {
		ret = true;
	} else {
		unsigned int name_len = dentry->d_name.len;
		const unsigned char *pName = dentry->d_name.name;

		ret = false;

		if (likely((pName) && (name_len >= SSC_NOM_MIN_PATH_LEN))) {
			unsigned char const *p_path = g_normal_mode_valid_path_bases;
			unsigned char const *p_path_end = p_path + ARRAY_SIZE(g_normal_mode_valid_path_bases);

			while (p_path < p_path_end) {
				unsigned int len;

				len = SSC_NOM_PATH_LEN(*p_path++);
				if (!len) {
					break;
				}

				if (name_len == len) {
					unsigned int i;

					for (i = 0; i < len; i++) {
						if (pName[i] != SSC_NOM_PATH_CHAR(p_path[i], i))
							break;
					}

					if (i >= len) {
						ret = true;
						break;
					}
				}

				p_path += len;
			}
		}
	}

	return ret;
}

noused notrace static bool __init __sec_syscall_is_file_exist(const char *filename)
{
	bool ret = false;

	if ((filename) && (filename[0])) {
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

#if defined(SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT)

#define IS_OTHER_MODE_FLAGS_COUNT (((LINUX_VERSION_CODE + VIVO_UTS_TIMESTAMP) % 3) + 2)
#define IS_OTHER_MODE_INDEX (((LINUX_VERSION_CODE + VIVO_UTS_TIMESTAMP) + 2) % IS_OTHER_MODE_FLAGS_COUNT)

noused static uintptr_t __sec_syscall_is_other_mode_flags[IS_OTHER_MODE_FLAGS_COUNT] __rs_text = { 0 };

noused notrace static int __init __sec_syscall_init_mode(void)
{
	noused long value;

#if defined(RS_MARK_VAR_READONLY)
	extern int __sec_syscall_set_text_vars(char *target, char *source, size_t unit_size, size_t unit_count);
	uintptr_t flags[IS_OTHER_MODE_FLAGS_COUNT];

	RS_GET_RANDOM_BYTES(&flags[0], sizeof(flags));
#else /*RS_MARK_VAR_READONLY*/
	RS_GET_RANDOM_BYTES(&__sec_syscall_is_other_mode_flags[0], sizeof(__sec_syscall_is_other_mode_flags));
#endif /*!RS_MARK_VAR_READONLY*/

	value = (typeof(value))get_cycles();

	if (!value) {
		value = (typeof(value))jiffies;
		if (!value) {
			RS_GET_RANDOM_BYTES(&value, sizeof(value));
		}

		if (!value) {
			value = 1;
		}
	}

#if 0
	if (__sec_syscall_is_file_exist("/etc/recovery.fstab")) {
		if (value < 0) {
			value = -value;
		}
	} else {
		if (value > 0) {
			value = -value;
		}
	}

#else /*0*/
	int boot_mode = ssc_get_boot_mode();

	if ((SSC_RECOVERY_BOOT == boot_mode) || (SSC_SURVIVAL_BOOT == boot_mode)) {
		if (value < 0) {
			value = -value;
		}
	} else {
		if (value > 0) {
			value = -value;
		}
	}
#endif /*!0*/

#if defined(RS_MARK_VAR_READONLY)
	flags[IS_OTHER_MODE_INDEX] = (uintptr_t)value;

	if (system_state > SYSTEM_BOOTING)
		__sec_syscall_set_text_vars(&__sec_syscall_is_other_mode_flags[0], &flags[0], sizeof(flags[0]), IS_OTHER_MODE_FLAGS_COUNT);
	else
		memcpy(&__sec_syscall_is_other_mode_flags[0], &flags[0], sizeof(__sec_syscall_is_other_mode_flags[0]));
#else /*RS_MARK_VAR_READONLY*/
	__sec_syscall_is_other_mode_flags[IS_OTHER_MODE_INDEX] = (uintptr_t)value;
#endif /*!RS_MARK_VAR_READONLY*/

	return 0;
}
late_initcall(__sec_syscall_init_mode);

#endif /*SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT*/


#if defined(SEC_SYSCALL_CHECK_UID_CHANGE) && defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)

static inline void noused notrace __sec_syscall_set_task_flag(struct task_struct *task, int nr)
{
	/*cannot take address of bit-field 'in_execve'*/
	/*unsigned long addr = ((unsigned long)(&task->in_execve) & ~(sizeof(CUC_FLAG_TYPE) - 1));*/
	unsigned long addr = ((unsigned long)(&task->personality) + sizeof(task->personality));
	SSC_CUC_FLAG_TYPE mask = SSC_CUC_FLAG_BIT_MASK(nr);
	SSC_CUC_FLAG_TYPE *p = ((SSC_CUC_FLAG_TYPE *)addr) + SSC_CUC_FLAG_BIT_WORD(nr);

	*p |= mask;
}

static inline void noused notrace __sec_syscall_clear_task_flag(struct task_struct *task, int nr)
{
	unsigned long addr = ((unsigned long)(&task->personality) + sizeof(task->personality));
	SSC_CUC_FLAG_TYPE mask = SSC_CUC_FLAG_BIT_MASK(nr);
	SSC_CUC_FLAG_TYPE *p = ((SSC_CUC_FLAG_TYPE *)addr) + SSC_CUC_FLAG_BIT_WORD(nr);

	*p &= ~mask;
}

static inline int noused notrace __sec_syscall_test_and_clear_task_flag(struct task_struct *task, int nr)
{
	unsigned long addr = ((unsigned long)(&task->personality) + sizeof(task->personality));
	SSC_CUC_FLAG_TYPE mask = SSC_CUC_FLAG_BIT_MASK(nr);
	SSC_CUC_FLAG_TYPE *p = ((SSC_CUC_FLAG_TYPE *)addr) + SSC_CUC_FLAG_BIT_WORD(nr);
	SSC_CUC_FLAG_TYPE old = *p;

	*p = old & ~mask;
	return (old & mask) != 0;
}
#endif

noused notrace static void __sec_syscall_dump_cap(const char *label, const struct kernel_cap_struct *cap)
{
	size_t i;
	printk(KERN_ALERT "%s:", label);
	for (i = 0; i < _KERNEL_CAPABILITY_U32S; i++) {
		printk(KERN_ALERT " [%d]=0x%x", (int)i, cap->cap[i]);
	}
	printk(KERN_ALERT "\n");
}

static inline int _read_cred_subscribers(const struct cred *cred)
{
#ifdef CONFIG_DEBUG_CREDENTIALS
	return atomic_read(&cred->subscribers);
#else
	return 0;
#endif
}
noused notrace static void __sec_syscall_dump_cred(const struct cred *cred, const char *label,
				const struct task_struct *tsk)
{
	if (!cred) {
		cred = tsk->cred;
	}

	printk(KERN_ALERT "CRED: %s %s %d %d %d credentials: %p %s%s%s\n",
		label, tsk->comm, __sec_syscall_pre_check_fail_reason,
		__sec_syscall_post_check_fail_reason, tsk->tgid, cred,
		cred == &init_cred ? "[init]" : "",
		cred == tsk->real_cred ? "[real]" : "",
		cred == tsk->cred ? "[eff]" : "");
#ifdef CONFIG_DEBUG_CREDENTIALS
	printk(KERN_ALERT "CRED: ->magic=%x, put_addr=%p\n",
		cred->magic, cred->put_addr);
	printk(KERN_ALERT "CRED: ->usage=%d, subscr=%d\n",
		atomic_read(&cred->usage),
		_read_cred_subscribers(cred));
#endif /*CONFIG_DEBUG_CREDENTIALS*/
	printk(KERN_ALERT "CRED: ->*uid = { %d,%d,%d,%d }\n",
		from_kuid_munged(&init_user_ns, cred->uid),
		from_kuid_munged(&init_user_ns, cred->euid),
		from_kuid_munged(&init_user_ns, cred->suid),
		from_kuid_munged(&init_user_ns, cred->fsuid));
	printk(KERN_ALERT "CRED: ->*gid = { %d,%d,%d,%d }\n",
		from_kgid_munged(&init_user_ns, cred->gid),
		from_kgid_munged(&init_user_ns, cred->egid),
		from_kgid_munged(&init_user_ns, cred->sgid),
		from_kgid_munged(&init_user_ns, cred->fsgid));
	printk(KERN_ALERT "CRED: ->securebits= { %x }\n",
		cred->securebits);

	__sec_syscall_dump_cap("cap_inheritable", &cred->cap_inheritable);
	__sec_syscall_dump_cap("cap_permitted", &cred->cap_permitted);
	__sec_syscall_dump_cap("cap_effective", &cred->cap_effective);
	__sec_syscall_dump_cap("cap_bset", &cred->cap_bset);

	printk(KERN_ALERT "CRED: ->user= { %p }\n", cred->user);
	printk(KERN_ALERT "CRED: ->user_ns= { %p }\n", cred->user_ns);
	printk(KERN_ALERT "CRED: ->group_info= { %p }\n", cred->group_info);

#ifdef CONFIG_SECURITY
	printk(KERN_ALERT "CRED: ->security is %p\n", cred->security);
	if ((unsigned long) cred->security >= PAGE_SIZE &&
		(((unsigned long) cred->security & 0xffffff00) !=
		(POISON_FREE << 24 | POISON_FREE << 16 | POISON_FREE << 8))) {
	#if defined(CONFIG_SECURITY_SELINUX) && defined(SEC_SYSCALL_CHECK_SELINUX_SID)
		struct task_security_struct *security = cred->security;
		printk(KERN_ALERT "CRED: ->security {%x,%x,%x,%x,%x,%x}\n",
			security->osid,
			security->sid,
			security->exec_sid,
			security->create_sid,
			security->keycreate_sid,
			security->sockcreate_sid
			);
	#else /*CONFIG_SECURITY_SELINUX && SEC_SYSCALL_CHECK_SELINUX_SID*/
		printk(KERN_ALERT "CRED: ->security {%x, %x}\n",
			((u32 *)cred->security)[0],
			((u32 *)cred->security)[1]);
	#endif /*!(CONFIG_SECURITY_SELINUX && SEC_SYSCALL_CHECK_SELINUX_SID)*/
	}
#endif /*CONFIG_SECURITY*/
}

#if (_KERNEL_CAPABILITY_U32S == 1)
typedef	__u32 kernel_cap_val_t;
#elif (_KERNEL_CAPABILITY_U32S == 2)
typedef	__u64 kernel_cap_val_t;
#elif (_KERNEL_CAPABILITY_U32S == 4)
typedef	__u128 kernel_cap_val_t;
#else
	#error "_KERNEL_CAPABILITY_U32S value is odd to me!"
#endif

typedef union cast_kernel_cap_struct {
	kernel_cap_t cap;
	kernel_cap_val_t val;
} cast_kernel_cap_t;

#if !defined(FIELD_OFFSET)
	#define FIELD_OFFSET(type, field)    ((long)(char *)&(((type *)0)->field))
#endif


#if defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)

asmlinkage notrace void __sec_syscall_force_stop(struct thread_info *thread)
{
	static bool __sec_syscall_dump_cred_flag;
#if defined(SEC_SYSCALL_FORCE_STOP_WITH_LOG)
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	struct task_struct *task = (struct task_struct *)thread;
#else
	struct task_struct *task = thread->task;
#endif

	printk(KERN_ALERT "ssc: stop [%d:%s]\n", task->pid, task->comm);
#endif /*SEC_SYSCALL_FORCE_STOP_WITH_LOG*/

	if (!__sec_syscall_dump_cred_flag) {
		__sec_syscall_dump_cred_flag = true;
		__sec_syscall_dump_cred(NULL, SEC_SYSCALL_DUMP_CRED_LABEL,
		#if defined(SEC_SYSCALL_FORCE_STOP_WITH_LOG)
			task
		#else /*SEC_SYSCALL_FORCE_STOP_WITH_LOG*/
		#if defined(CONFIG_THREAD_INFO_IN_TASK)
			(struct task_struct *)thread
		#else
			thread->task
		#endif
		#endif /*!SEC_SYSCALL_FORCE_STOP_WITH_LOG*/
		);
	}
}

#else /*SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

#if defined(SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP)

static int __sec_syscall_is_task_killable(struct task_struct *task)
{
	if (task->exit_state) {
		return 0;
	}

	if (test_tsk_thread_flag(task, TIF_MEMDIE)) {
		return 0;
	}

	if (!task->mm) {
		return 0;
	}

	if (task->flags & PF_EXITING) {
		if (!(task->group_leader->ptrace & PT_TRACE_EXIT)) {
			return 0;
		}
	}

	return 1;
}

asmlinkage notrace void __sec_syscall_force_stop(struct thread_info *thread)
{
	struct cgroup *cgrp;
	struct cgroup_iter it;
	struct task_struct *curr_tsk, *tsk;

	if (!thread)
		goto do_exit;

#if defined(CONFIG_THREAD_INFO_IN_TASK)
	tsk = (struct task_struct *)thread;
#else
	tsk = thread->task;
#endif

	if (!tsk)
		goto do_exit;

	curr_tsk = tsk->group_leader;

	if (curr_tsk) {
		task_lock(curr_tsk);
		cgrp = task_cgroup(curr_tsk, cpuacct_subsys_id); /*depend on CONFIG_CGROUP_CPUACCT*/
		/*cgrp = task_cgroup_from_root(curr_tsk, &cgrp_dfl_root);*/
		task_unlock(curr_tsk);

		if (cgrp) {
		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
			/*cgroup_get(cgrp);*/
		#else
			/*atomic_inc(&cgrp->refcnt);*/
		#endif

			if (cgrp != &(cgrp->root->top_cgroup)) {
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
				css_task_iter_start(&cgrp->dummy_css, &it);
				while ((tsk = css_task_iter_next(&it))) {
			#else
				cgroup_iter_start(cgrp, &it); /*will aquire css_set_lock*/
				while ((tsk = cgroup_iter_next(cgrp, &it))) {
			#endif
					struct task_struct *p;

					if (tsk == curr_tsk)
						continue;

					task_lock(tsk);
					p = ((__sec_syscall_is_task_killable(tsk)) ? tsk : NULL);
					task_unlock(tsk);

					if (p) {
						do_send_sig_info(SIGKILL, SEND_SIG_FORCED, p, true);
					}

				#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
					put_task_struct(tsk);
				#endif
				}
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
				css_task_iter_end(&it);
			#else
				cgroup_iter_end(cgrp, &it);
			#endif

			}

		#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 15, 0)
			/*cgroup_put(cgrp);*/ /*static function*/
		#else
			/*atomic_dec(&cgrp->refcnt);*/
		#endif
		}
	}

do_exit:
	do_group_exit(SEC_SYSCALL_GROUP_EXIT_CODE);
}

#else /*SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP*/

#if defined(SEC_SYSCALL_FORCE_STOP_WITH_LOG)
asmlinkage notrace void __sec_syscall_force_stop(struct thread_info *thread)
{
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	struct task_struct *task = (struct task_struct *)thread;
#else
	struct task_struct *task = thread->task;
#endif

	printk(KERN_ALERT "ssc: stop [%d:%s]\n", task->pid, task->comm);

#if defined(SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT)
	do_group_exit(SEC_SYSCALL_GROUP_EXIT_CODE);
#else /*SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
	force_sig(SIGKILL, task);
#endif /*!SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
}

#else /*SEC_SYSCALL_FORCE_STOP_WITH_LOG*/

asmlinkage notrace void
#if !defined(CONFIG_ARM64)
__naked
#endif
__sec_syscall_force_stop(struct thread_info *thread)
{
#if defined(CONFIG_ARM64)
#if defined(SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT)
	asm volatile ("mov	x0, %0	;"
			"b	do_group_exit	;" /*will not return*/
			:
			: "i" (SEC_SYSCALL_GROUP_EXIT_CODE)
			: "x0", "x1"
			);
#else /*SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	asm volatile ("mov	x1, x0	;"
			"mov	x0, %1	;"
			"b	force_sig	;"
			:
			: "i" (SIGKILL)
			: "x0", "x1"
			);
#else
	asm volatile ("ldr	x1, [x0, %0]	;"
			"mov	x0, %1	;"
			"b	force_sig	;"
			:
			: "i" (FIELD_OFFSET(struct thread_info, task)), "i" (SIGKILL)
			: "x0", "x1"
			);
#endif /*!CONFIG_THREAD_INFO_IN_TASK*/
#endif /*!SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
#else /*CONFIG_ARM64*/
#if defined(SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT)
	asm volatile ("mov	r0, %0	;"
			"b	do_group_exit	;" /*will not return*/
			:
			: "i" (SEC_SYSCALL_GROUP_EXIT_CODE)
			: "r0", "r1"
			);
#else /*SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	asm volatile ("mov	x1, x0	;"
			"mov	x0, %1	;"
			"b	force_sig	;"
			:
			: "i" (SIGKILL)
			: "x0", "x1"
			);
#else
	asm volatile ("ldr	r1, [r0, %0]	;"
			"mov	r0, %1	;"
			"b	force_sig	;"
			:
			: "i" (FIELD_OFFSET(struct thread_info, task)), "i" (SIGKILL)
			: "r0", "r1"
			);
#endif /*!CONFIG_THREAD_INFO_IN_TASK*/
#endif /*!SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT*/
#endif /*!CONFIG_ARM64*/
}

#endif /*!SEC_SYSCALL_FORCE_STOP_WITH_LOG*/
#endif /*!SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP*/
#endif /*!SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

asmlinkage notrace long __sec_syscall_pre_check(struct thread_info *thread
#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
	, unsigned long syscall_no
#endif /*SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER*/
)
{
#if 1
#if !defined(SEC_SYSCALL_CHECK_UID_CHANGE)
	noused struct task_struct *task;

#if 0
	if (!thread) {
		thread = current_thread_info();
	}
#endif

#if defined(SEC_SYSCALL_FORCE_STOP) || defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	task = (struct task_struct *)thread;
#else
	task = thread->task;
#endif
#endif /*SEC_SYSCALL_FORCE_STOP || SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

#if 0
	if (!task) {
		goto out;
	}
#endif

#if 0
	if (task->real_parent == &init_task) {
		goto out;
	}
#endif

#if !defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && !defined(SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT)
	/*addr_limit already checked in post check*/
	if (unlikely(thread->addr_limit != USER_DS)) {
		PRE_CHECK_FAIL(1);
		goto force_stop;
	} else
#endif /*!SEC_SYSCALL_POST_CHECK_ALL_CALL && !SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT*/
	{
		noused const struct cred *cred;
		kuid_t uid;

	#if defined(SEC_SYSCALL_PRE_CHECK_VERIFY_SELINUX_PERMISSIVE)
		if (unlikely(test_ti_thread_flag(thread, (BITS_PER_LONG - 1)))) {
			PRE_CHECK_FAIL(2);
			goto force_stop;
		}
	#endif /*SEC_SYSCALL_PRE_CHECK_VERIFY_SELINUX_PERMISSIVE*/

	#if defined(CONFIG_THREAD_INFO_IN_TASK)
		task = (struct task_struct *)thread;
	#else
		task = thread->task;
	#endif

		cred = task->cred;

		if (unlikely(!cred)) {
			PRE_CHECK_FAIL(3);
			goto force_stop;
			/*goto out;*/
		}

		/*need check euid? only when SUID enabled?*/
		/*system partition has been mounted with no_suid since ard 4.3*/
		uid = cred->uid;
		if (likely(uid_ne(uid, GLOBAL_ROOT_UID))) {
		#if !defined(SEC_SYSCALL_CHECK_SELINUX_SID)
			if (likely(uid_ne(cred->euid, GLOBAL_ROOT_UID)))
		#endif /*!SEC_SYSCALL_CHECK_SELINUX_SID*/
			{
				/*not root user*/
				goto out;
			}
		}

		{
			noused const struct user_struct *user = cred->user;

			if (unlikely(!user)) {
				PRE_CHECK_FAIL(4);
				goto force_stop;
				/*goto out;*/
			}

	#if 0
			if (user == INIT_USER) {
				if (task->real_parent != &init_task) {
					PRE_CHECK_FAIL(5);
					goto force_stop;
				}
			}
	#endif

			if (unlikely(uid_ne(user->uid, uid))) {
				PRE_CHECK_FAIL(6);
				goto force_stop;
			}
		}

#if 0
		if (!__sec_syscall_pre_check_dump_cred_flag) {
			if (task->pid == 1) {
				__sec_syscall_pre_check_dump_cred_flag = true;
				__sec_syscall_dump_cred(cred, SEC_SYSCALL_PRE_CHECK_DUMP_CRED_LABEL, task);
			}
		}
#endif

	#if 0 /*not sure about this field*/
		{
			unsigned int securebits = cred->securebits;

			if (unlikely(securebits)) {
				if (unlikely(securebits > (/*SECURE_ALL_BITS |*/ SECURE_ALL_LOCKS))) {
					PRE_CHECK_FAIL(8);
					goto force_stop;
				} else {
					unsigned int locks = (securebits & SECURE_ALL_LOCKS);
					unsigned int bits = (securebits & SECURE_ALL_BITS);

					if (unlikely((locks)
						&& (bits))) {
						int lock_bit;
						int max_lock_bit = (sizeof(securebits) << 3) - __builtin_clz(SECURE_ALL_LOCKS | 0x2) - 1;

						for (lock_bit = 1; lock_bit <= max_lock_bit; lock_bit += 2) {
							if (locks & (1 << lock_bit)) {
								/*bit got locked*/
								if (bits & (1 << (lock_bit - 1))) {
									/*bit got set*/
									PRE_CHECK_FAIL(9);
									goto force_stop;
								}
							}
						}
					}
				}
			}
		}
	#endif

		/*init task's cap_inheritable is all zero*/
		/*root tool will set all cap_xxx to -1 in general*/
	#if defined(CAP_LAST_U32_VALID_MASK) && defined(CAP_LAST_CAP) \
		&& (CAP_LAST_CAP < ((_KERNEL_CAPABILITY_U32S * 32) - 1))
		/*this works after LINUX_VERSION_CODE >= 3.17.0*/
		{
			register cast_kernel_cap_t cast_cap_full = { .cap = CAP_FULL_SET, };

			if (unlikely((((cast_kernel_cap_t *)&(cred->cap_inheritable))->val > cast_cap_full.val)
				|| (((cast_kernel_cap_t *)&(cred->cap_permitted))->val > cast_cap_full.val)
				|| (((cast_kernel_cap_t *)&(cred->cap_effective))->val > cast_cap_full.val)
				|| (((cast_kernel_cap_t *)&(cred->cap_bset))->val > cast_cap_full.val))) {
				PRE_CHECK_FAIL(10);
				goto force_stop;
			}
		}
	#elif defined(CAP_LAST_CAP) && (CAP_LAST_CAP < ((_KERNEL_CAPABILITY_U32S * 32) - 1))
		/* if 3.8.0 <= LINUX_VERSION_CODE < 3.17.0, task_cap() will use NORM_CAPS() macro to normalize
		 * cap_xxx values before them were showed in /proc/pid/status, but the actual value is still -1
		 */
		{
			/*mobile_log_d cap_inheritable is -1, others' cap_inheritable are 0 in general*/
			kernel_cap_val_t val = ((cast_kernel_cap_t *)&(cred->cap_inheritable))->val;
			kernel_cap_val_t val_upper_bound = ((kernel_cap_val_t)1 << (CAP_LAST_CAP + 1));

			if (unlikely((val >= val_upper_bound)
				&& (((cast_kernel_cap_t *)&(cred->cap_permitted))->val == val)
				&& (((cast_kernel_cap_t *)&(cred->cap_effective))->val == val)
				&& (((cast_kernel_cap_t *)&(cred->cap_bset))->val == val))) {
				PRE_CHECK_FAIL(11);
				goto force_stop;
			}
		}
	#endif

	#if 0 /*LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0)*/ /*not applicable, only work for task with new user_ns*/
		{
			noused const struct user_namespace *user_ns = cred->user_ns;

			if (unlikely(!user_ns)) {
				PRE_CHECK_FAIL(12);
				goto force_stop;
				/*goto out;*/
			}

			if (unlikely(uid_ne(user_ns->owner, cred->euid) || gid_ne(user_ns->group, cred->egid))) {
				PRE_CHECK_FAIL(20);
				goto force_stop;
			}
		}
	#endif

	#if defined(CONFIG_KEYS)
		{
			noused const struct key *keyring = cred->thread_keyring;

			if (likely(keyring)) {
				if (unlikely(IS_ERR(keyring))) {
					PRE_CHECK_FAIL(21);
					goto force_stop;
				}

				if (unlikely(uid_ne(keyring->uid, cred->fsuid) || gid_ne(keyring->gid, cred->fsgid))) {
					PRE_CHECK_FAIL(22);
					goto force_stop;
				}
			}
		}
	#endif /*CONFIG_KEYS*/

		{
	#if defined(SEC_SYSCALL_CHECK_SELINUX_SID)
			int pid_checked = 0;

		/*#if defined(CONFIG_SECURITY) && defined(CONFIG_SECURITY_SELINUX)*/
			{
				/*
				init_task: osid == sid == SECINITSID_KERNEL == 1, exec_sid == create_sid == keycreate_sid == sockcreate_sid == 0
				osid can be 1, exec_sid can be 0, but sid and exec_sid should not be 1, even for init task
				*/
				u32 sid;
				struct task_security_struct *security = cred->security;

				if (unlikely(!security)) {
					goto check_pid_further;
				}

			#if 0
				if (unlikely(((unsigned long)security < PAGE_SIZE)
					|| (((unsigned long)security & 0xffffff00)
					== (POISON_FREE << 24 | POISON_FREE << 16 | POISON_FREE << 8))))
			#else
				if (unlikely((unsigned long)security < USER_DS))
			#endif
				{
					goto check_pid_further;
				}

				sid = security->sid;

				if (unlikely(!sid)) {
					PRE_CHECK_FAIL(30);
					goto force_stop;
				}

				/*no need for checking of su sid, it is set to not permissive by default*/
				if (unlikely(sid == 1)) {
					/* global init task, pid == 1, it's parent is init_task(idle task)
					 * global kthreadd task, pid == 2, it's parent is init_task(idle task) too
					 * normal root process's realparent is init or kthreadd
					 */
					if (unlikely(task->pid != 1)) {
					#if defined(SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT)
						if (unlikely((long)__sec_syscall_is_other_mode_flags[IS_OTHER_MODE_INDEX] > 0)) {
						#if 0
							goto misc_check;
						#else
							/* android 4.4 recovery mode, native service got killed, refer to prepend_first_level_path_no_lock() in oom_kill.c
							 * task is child of init/recovery
							 */
							preempt_disable_notrace();
							{
								struct mm_struct *mm;
								struct file *exe_file;

								/*task's executable is under fs root*/
								exe_file = (mm = task->mm) ? (mm->exe_file) : NULL;
								if (likely(exe_file)) {
								#if 0
									if (__sec_syscall_other_mode_is_allowed_executable_by_path(&(exe_file->f_path))) {
										/*OK*/
										preempt_enable_no_resched_notrace();
										goto misc_check;
									}
								#else
									if (__sec_syscall_other_mode_is_allowed_executable_by_dentry(&(exe_file->f_path))) {
										/*OK*/
										preempt_enable_no_resched_notrace();
										goto misc_check;
									}
								#endif
								}
							}
							preempt_enable_no_resched_notrace();
						#endif /*!0*/
						}
					#endif /*SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT*/

					#if !defined(SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS)
						PRE_CHECK_FAIL(31);
						goto force_stop;
					#endif
					}

					pid_checked = 1;

					if (unlikely(task->tgid != 1)) {
					#if defined(SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS)
						/* android 8.0 normal mode, init may fork /system/bin/secilc
						 * to compile split selinux policy
						 */
						preempt_disable_notrace();
						{
							struct mm_struct *mm;
							struct file *exe_file;

							/* task's executable is under fs root */
							exe_file = (mm = task->mm) ? (mm->exe_file) : NULL;
							if (likely(exe_file)) {
								if (__sec_syscall_normal_mode_is_allowed_executable_by_dentry(&(exe_file->f_path))) {
									/* OK */
									preempt_enable_no_resched_notrace();
									goto misc_check;
								}
							}
						}
						preempt_enable_no_resched_notrace();
					#endif
						PRE_CHECK_FAIL(32);
						goto force_stop;
					}

					/* android 6.x, init start with two stage, first stage is in kernel domain,
					 * and shoud not be forced stop. init task call execve() to start second stage
					 */
				#if defined(SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL)

					if (!__sec_syscall_init_is_in_second_stage_flags[IN_SECOND_STAGE_INDEX]) {
					#if !defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
					#if 1
						#if 0
						/*reg val will be stored to stack*/
						#if defined(CONFIG_ARM64)
						register unsigned long syscall_no asm("x26");
						#else
						register unsigned long syscall_no asm("r7");
						#endif
						#else /*0*/

						register unsigned long syscall_no;
						#if defined(CONFIG_ARM64)
						asm volatile ("mov %0, x26	;"
							: "=r" (syscall_no)
							:
							: "cc"
							);
						#else /*CONFIG_ARM64*/
						asm volatile ("mov %0, r7	;"
							: "=r" (syscall_no)
							:
							: "cc"
							);
						#endif /*!CONFIG_ARM64*/

						if (unlikely(syscall_no == __NR_execve)) {
							__sec_syscall_init_is_in_second_stage_flags[IN_SECOND_STAGE_INDEX] = true;
						}
						#endif /*!0*/
					#else /*0*/

					#if defined(CONFIG_ARM64)
						asm volatile ("cmp	x26, %0	;"
								"b.ne	not_execve	;"
								:
								: "i" (__NR_execve)
								: "cc"
								);
					#else /*CONFIG_ARM64*/
						asm volatile ("cmp	r7, %0	;"
								"bne	not_execve	;"
								:
								: "i" (__NR_execve)
								: "cc"
								);
					#endif /*!CONFIG_ARM64*/

						__sec_syscall_init_is_in_second_stage_flags[IN_SECOND_STAGE_INDEX] = true;

						asm volatile ("not_execve:");
					#endif /*!0*/
					#endif /*!SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER*/
					} else {
						/*kernel domain not allowed any more*/
						PRE_CHECK_FAIL(33);
						goto force_stop;
					}
				#endif /*SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL*/

					/*checking in first stage*/
					{
						/*6.x case, effective=permitted=bset: [0]=0xffffffff [1]=0x3f*/
						cast_kernel_cap_t *cast_cap = (cast_kernel_cap_t *)&(cred->cap_bset);
						register cast_kernel_cap_t cast_cap_full = { .cap = CAP_FULL_SET, };

						if (likely(cast_cap->val == cast_cap_full.val)) {
						#if 1
							if (unlikely(task->real_parent != &init_task)) {
								PRE_CHECK_FAIL(34);
								goto force_stop;
							} else {
								if (unlikely((cred->user != INIT_USER)
									|| (cred->user_ns != &init_user_ns)
								#if !defined(SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS)
									|| (cred->group_info != &init_groups)
								#endif
									)) {
									PRE_CHECK_FAIL(35);
									goto force_stop;
								}
							}
						#else
							PRE_CHECK_FAIL(36);
							goto force_stop;
						#endif
						} else {
							PRE_CHECK_FAIL(37);
							goto force_stop;
						}
					}

				#if 0
					if (unlikely(sid == security->exec_sid)) {
						PRE_CHECK_FAIL(38);
						goto force_stop;
					}
				#endif
				#if 0
					if (unlikely(sid == security->osid)) {
						if (task->real_parent != &init_task) {
							PRE_CHECK_FAIL(39);
							goto force_stop;
						}
					}
				#endif
				}

			#if 1
				if (unlikely(security->exec_sid == 1)) {
					PRE_CHECK_FAIL(40);
					goto force_stop;
				}
			#endif

			#if 0
				/*except for init task, service tasks directly forked by init obey this rule too, so can't be forced stop*/
				if (likely(security->osid == 1)) {
					if (unlikely((task->real_parent != &init_task) || (task->real_parent->real_parent != &init_task))) {
						PRE_CHECK_FAIL(41);
						goto force_stop;
					}
				}
			#endif

			#if 0
				if (!__sec_syscall_pre_check_dump_cred_flag) {
					if (security->osid != 1) {
						__sec_syscall_pre_check_dump_cred_flag = true;
						__sec_syscall_dump_cred(cred, SEC_SYSCALL_PRE_CHECK_DUMP_CRED_LABEL, task);
					}
				}
			#endif
			}
		/*#endif*/

		check_pid_further:
			if (likely(!pid_checked))
	#endif /*SEC_SYSCALL_CHECK_SELINUX_SID*/
			{
				/*global init task*/
				if (unlikely(task->pid == 1)) {
					if (unlikely(task->tgid != 1)) {
						PRE_CHECK_FAIL(50);
						goto force_stop;
					}

					if (unlikely(task->real_parent != &init_task)) {
						PRE_CHECK_FAIL(51);
						goto force_stop;
					} else {
						if (unlikely((cred->user != INIT_USER)
							|| (cred->user_ns != &init_user_ns)
						#if !defined(SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS)
							|| (cred->group_info != &init_groups)
						#endif
							)) {
							PRE_CHECK_FAIL(52);
							goto force_stop;
						}
					}
				}
			}
		}

#if defined(SEC_SYSCALL_CHECK_SELINUX_SID) \
	&& (defined(SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT) || defined(SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS))
misc_check:
#endif /*SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT*/
	#if 0
		/* it is very likely that user space process will causing kernel panic
		 * if it set task->mm to NULL forcely, so this check is not needed?
		 */
		if (unlikely(!task->mm)) {
			PRE_CHECK_FAIL(60);
			goto force_stop;
		}
	#endif

	/*#if !(defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD))*/
		/*maybe got modified but not by syscall, thus cannot detect by post check*/
		if (unlikely(task->flags & PF_KTHREAD)) {
			PRE_CHECK_FAIL(61);
			goto force_stop;
		}
	/*#endif*/ /*!(SEC_SYSCALL_POST_CHECK_ALL_CALL && SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD)*/
	}

#else /*!SEC_SYSCALL_CHECK_UID_CHANGE*/

	noused struct task_struct *task;
	noused const struct cred *cred;
#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	struct pt_regs *regs;
#endif

#if defined(CONFIG_THREAD_INFO_IN_TASK)
	task = (struct task_struct *)thread;
#else
	task = thread->task;
#endif
	/*
	if (unlikely(!task)) {
		PRE_CHECK_FAIL(1);
		goto force_stop;
	}
	*/

/*#if !(defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD))*/
	/*maybe got modified but not by syscall, thus cannot detect by post check*/
	if (unlikely(task->flags & PF_KTHREAD)) {
		PRE_CHECK_FAIL(2);
		goto force_stop;
	}
/*#endif*/ /*!(SEC_SYSCALL_POST_CHECK_ALL_CALL && SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD)*/

	cred = task->cred;

	/*
	if (unlikely(!cred)) {
		PRE_CHECK_FAIL(3);
		goto force_stop;
	}
	*/

#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	regs = thread_pt_regs(thread);
#endif

	if (likely(uid_ne(cred->uid, GLOBAL_ROOT_UID)
		&& uid_ne(cred->euid, GLOBAL_ROOT_UID))) {
	#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
		set_bit(SEC_SYSCALL_UID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused);
	#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
		__sec_syscall_set_task_flag(task, SEC_SYSCALL_UID_MAY_CHANGE_FLAG);
	#else
		set_ti_thread_flag(thread, SEC_SYSCALL_UID_MAY_CHANGE_FLAG);
	#endif
	} else {
	#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
		clear_bit(SEC_SYSCALL_UID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused);
	#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
		__sec_syscall_clear_task_flag(task, SEC_SYSCALL_UID_MAY_CHANGE_FLAG);
	#else
		clear_ti_thread_flag(thread, SEC_SYSCALL_UID_MAY_CHANGE_FLAG);
	#endif
	}

#if defined(SEC_SYSCALL_CHECK_SELINUX_SID)
	{
		u32 sid;
		struct task_security_struct *security = cred->security;

		/*
		if (unlikely(!security)) {
			PRE_CHECK_FAIL(4);
			goto force_stop;
		}
		*/

		/*
		if (unlikely((unsigned long)security < USER_DS)) {
			PRE_CHECK_FAIL(5);
			goto force_stop;
		}
		*/

		sid = security->sid;

		if (unlikely(!sid)) {
			PRE_CHECK_FAIL(6);
			goto force_stop;
		}

		if (likely(sid != 1)) {
		#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
			set_bit(SEC_SYSCALL_SID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused);
		#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
			__sec_syscall_set_task_flag(task, SEC_SYSCALL_SID_MAY_CHANGE_FLAG);
		#else
			set_ti_thread_flag(thread, SEC_SYSCALL_SID_MAY_CHANGE_FLAG);
		#endif
		} else {
		#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
			clear_bit(SEC_SYSCALL_SID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused);
		#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
			__sec_syscall_clear_task_flag(task, SEC_SYSCALL_SID_MAY_CHANGE_FLAG);
		#else
			clear_ti_thread_flag(thread, SEC_SYSCALL_SID_MAY_CHANGE_FLAG);
		#endif
		}
	}
#endif

#endif

#if !defined(SEC_SYSCALL_CHECK_UID_CHANGE)
out:
#endif
	return 0;
force_stop:
#if defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
	if (!__sec_syscall_pre_check_dump_cred_flag) {
		__sec_syscall_pre_check_dump_cred_flag = true;
		__sec_syscall_dump_cred(NULL, SEC_SYSCALL_PRE_CHECK_DUMP_CRED_LABEL, task);
	}

	return 0;
#else /*SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/
#if defined(SEC_SYSCALL_FORCE_STOP)
	{
	#if defined(SEC_SYSCALL_CALL_FORCE_STOP_IN_ASM)
	#if defined(CONFIG_ARM64)
		asm ("	stp	x29, x30, [sp, #-0x10]!	;"
				/*"		mov	x29, sp	;"*/
				"		mov	x0, %0	;" /*thread*/
				"		bl	__sec_syscall_force_stop	;"
				"		ldp	x29, x30, [sp], #0x10	;"
				/*"		mvn	x0, xzr	;"*/
				:
				: "r" (thread)
				: "x0", "x1"
				);
	#else /*CONFIG_ARM64*/
		asm ("	stmdb	sp!, {lr}	;" /*"	stmdb	sp!, {r4-r12,lr}	;"*/
				"		mov	r0, %0	;" /*thread*/
				"		bl	__sec_syscall_force_stop	;" /*force_sig will save regs itself*/
				"		ldmia	sp!,{lr}	;" /*"		ldmia	sp!,{r4-r12,lr}	;"*/
				:
				: "r" (thread)
				: "r0", "r1"
				);
	#endif /*!CONFIG_ARM64*/
	#else
		__sec_syscall_force_stop(thread); /*should define SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS if calling force_sig?*/
	#endif
	}
#endif /*SEC_SYSCALL_FORCE_STOP*/
	return -EPERM;
#endif /*!SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

#else /*1*/
	return 0;
#endif
}

asmlinkage notrace long __sec_syscall_post_check(struct thread_info *thread)
{
#if 1
#if !defined(SEC_SYSCALL_CHECK_UID_CHANGE)
	noused struct task_struct *task;

#if 0
	if (!thread) {
		thread = current_thread_info();
	}
#endif

#if 1 /*defined(SEC_SYSCALL_FORCE_STOP)*/
#if defined(CONFIG_THREAD_INFO_IN_TASK)
	task = (struct task_struct *)thread;
#else
	task = thread->task;
#endif
#endif

#if 0
	if (!task) {
		goto out;
	}
#endif

#if 0
	if (task->real_parent == &init_task) {
		goto out;
	}
#endif

#if !defined(SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT)
	if (unlikely(thread->addr_limit != USER_DS)) {
		POST_CHECK_FAIL(1);
		goto force_stop;
	} else
#endif
	{
	#if defined(SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE)
		if (unlikely(test_ti_thread_flag(thread, (BITS_PER_LONG - 1)))) {
			POST_CHECK_FAIL(2);
			goto force_stop;
		}
	#endif /*SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE*/

	#if defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD)
	#if defined(CONFIG_THREAD_INFO_IN_TASK)
		task = (struct task_struct *)thread;
	#else
		task = thread->task;
	#endif

		if (unlikely(task->flags & PF_KTHREAD)) {
			POST_CHECK_FAIL(3);
			goto force_stop;
		}
	#endif /*SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD*/
	}

#else /*!SEC_SYSCALL_CHECK_UID_CHANGE*/

	noused struct task_struct *task;
	noused const struct cred *cred;
#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	struct pt_regs *regs;
#endif

#if !defined(SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT)
	if (unlikely(thread->addr_limit != USER_DS)) {
		PRE_CHECK_FAIL(1);
		goto force_stop;
	}
#endif /*!SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT*/

#if defined(SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE)
	if (unlikely(test_ti_thread_flag(thread, (BITS_PER_LONG - 1)))) {
		PRE_CHECK_FAIL(2);
		goto force_stop;
	}
#endif /*SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE*/

#if defined(CONFIG_THREAD_INFO_IN_TASK)
	task = (struct task_struct *)thread;
#else
	task = thread->task;
#endif
	/*
	if (unlikely(!task)) {
		POST_CHECK_FAIL(3);
		goto force_stop;
	}
	*/

#if defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD)
	if (unlikely(task->flags & PF_KTHREAD)) {
		POST_CHECK_FAIL(4);
		goto force_stop;
	}
#endif /*SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD*/

	cred = task->cred;

	/*
	if (unlikely(!cred)) {
		POST_CHECK_FAIL(5);
		goto force_stop;
	}
	*/

#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	regs = thread_pt_regs(thread);
#endif

#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	if (likely(test_and_clear_bit(SEC_SYSCALL_UID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused)))
#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
	if (likely(__sec_syscall_test_and_clear_task_flag(task, SEC_SYSCALL_UID_MAY_CHANGE_FLAG)))
#else
	if (likely(test_and_clear_ti_thread_flag(thread, SEC_SYSCALL_UID_MAY_CHANGE_FLAG)))
#endif
	{
		if (unlikely(uid_eq(cred->uid, GLOBAL_ROOT_UID)
			|| uid_eq(cred->euid, GLOBAL_ROOT_UID))) {
			POST_CHECK_FAIL(6);
			goto force_stop;
		}
	}

#if defined(SEC_SYSCALL_CHECK_SELINUX_SID)
#if defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS)
	if (likely(test_and_clear_bit(SEC_SYSCALL_SID_MAY_CHANGE_FLAG, (unsigned long *)&regs->unused)))
#elif defined(SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT)
	if (likely(__sec_syscall_test_and_clear_task_flag(task, SEC_SYSCALL_SID_MAY_CHANGE_FLAG)))
#else
	if (likely(test_and_clear_ti_thread_flag(thread, SEC_SYSCALL_SID_MAY_CHANGE_FLAG)))
#endif
	{
		u32 sid;
		struct task_security_struct *security;

		security = cred->security;

		/*
		if (unlikely(!security)) {
			POST_CHECK_FAIL(7);
			goto force_stop;
		}
		*/

		/*
		if (unlikely((unsigned long)security < USER_DS)) {
			POST_CHECK_FAIL(8);
			goto force_stop;
		}
		*/

		sid = security->sid;

		if (unlikely(!sid)) {
			PRE_CHECK_FAIL(9);
			goto force_stop;
		}

		if (unlikely(sid == 1)) {
			PRE_CHECK_FAIL(10);
			goto force_stop;
		}
	}
#endif

#endif

/*out:*/
	return 0;
#if !defined(SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT) \
	|| defined(SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE) \
	|| defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD) \
	|| defined(SEC_SYSCALL_CHECK_UID_CHANGE)
force_stop:
#endif
#if defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
	if (!__sec_syscall_post_check_dump_cred_flag) {
		__sec_syscall_post_check_dump_cred_flag = true;
		__sec_syscall_dump_cred(NULL, SEC_SYSCALL_POST_CHECK_DUMP_CRED_LABEL, task);
	}

	return 0;
#else /*SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/
#if defined(SEC_SYSCALL_FORCE_STOP)
	{
	#if defined(SEC_SYSCALL_CALL_FORCE_STOP_IN_ASM)
	#if defined(CONFIG_ARM64)
		asm ("	stp	x29, x30, [sp, #-0x10]!	;"
				/*"		mov	x29, sp	;"*/
				"		mov	x0, %0	;" /*thread*/
				"		bl	__sec_syscall_force_stop	;"
				"		ldp	x29, x30, [sp], #0x10	;"
				/*"		mvn	x0, xzr	;"*/
				:
				: "r" (thread)
				: "x0", "x1"
				);
	#else /*CONFIG_ARM64*/
		asm ("	stmdb	sp!, {lr}	;" /*"	stmdb	sp!, {r4-r12,lr}	;"*/
				"		mov	r0, %0	;" /*thread*/
				"		bl	__sec_syscall_force_stop	;" /*force_sig will save regs itself*/
				"		ldmia	sp!,{lr}	;" /*"		ldmia	sp!,{r4-r12,lr}	;"*/
				:
				: "r" (thread)
				: "r0", "r1"
				);
	#endif /*!CONFIG_ARM64*/
	#else
		__sec_syscall_force_stop(thread); /*should define SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS if calling force_sig?*/
	#endif
	}
#endif /*SEC_SYSCALL_FORCE_STOP*/
	return -EPERM;
#endif /*!SEC_SYSCALL_CALL_FORCE_STOP_DEBUG*/

#else
	return 0;
#endif
}

#if defined(CONFIG_COMPAT) && defined(__NR_compat_syscalls)
const unsigned int __initconst __sec_syscall_NR_compat_syscalls = __NR_compat_syscalls;
#endif

#define SEC_SYSCALL_DEFINE_ENUMERATION
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_ENUMERATION

#define SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO

noused static const uint16_t __sec_syscall_indexes[nr_invalid] __initconst = {
#define SEC_SYSCALL_DEFINE_INDEX_ARRAY
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_INDEX_ARRAY
};


#if !defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)

#define SEC_SYSCALL_DEFINE_STUBS_REFERENCE
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_REFERENCE

#define SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO

noused static const void_func __sec_syscall_new_entries[nr_invalid] __initconst = {
#define SEC_SYSCALL_DEFINE_STUBS_ARRAY
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_ARRAY
};
#endif /*!SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/

noused notrace static void __init __sec_syscall_fill_buffer(uint8_t *buf, size_t buflen, unsigned long pattern)
{
	size_t i, align_prefix_len, align_postfix_len;
	unsigned long *pblock;

	if ((!buf) || (!buflen)) {
		return;
	}

	align_prefix_len = ((size_t)buf & (sizeof(unsigned long) - 1));
	if (align_prefix_len) {
		uint8_t *p;
		uint8_t *pdata;
		align_prefix_len = sizeof(unsigned long) - align_prefix_len;
		p = buf;
		pdata = (uint8_t *)(&pattern);

		for (i = 0; (i < align_prefix_len) && (i < buflen); i++)
			*p++ = *pdata++;

		buflen -= i;
		if (!buflen)
			return;

		pattern = (pattern >> (align_prefix_len << 3))
			| (pattern << ((sizeof(unsigned long) - align_prefix_len) << 3));

		pblock = (unsigned long *)p;
	} else {
		pblock = (unsigned long *)buf;
	}

	align_postfix_len = (((size_t)pblock + buflen) & (sizeof(unsigned long) - 1));
	buflen -= align_postfix_len;

	for (i = 0; i < buflen; i += sizeof(unsigned long), pblock++)
		*pblock = pattern;

	if (align_postfix_len) {
		uint8_t *p, *pdata;
		p = (uint8_t *)pblock;
		pdata = (uint8_t *)(&pattern);

		do {
			align_postfix_len--;
			p[align_postfix_len] = pdata[align_postfix_len];
		} while (align_postfix_len);
	}
}

noused notrace static int __init __sec_syscall_init_vars(void)
{
	noused unsigned long value;

	value = (typeof(value))get_cycles();

	if (!value) {
		value = (typeof(value))jiffies;
		if (!value) {
			RS_GET_RANDOM_BYTES(&value, sizeof(value));

			if (!value)
				value = 1;
		}
	}

#if defined(SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL)
	__sec_syscall_fill_buffer((uint8_t *)&__sec_syscall_init_is_in_second_stage_flags[0],
		sizeof(__sec_syscall_init_is_in_second_stage_flags), value);

	__sec_syscall_init_is_in_second_stage_flags[IN_SECOND_STAGE_INDEX] = false;
#endif /*SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL*/

	return 0;
}

console_initcall(__sec_syscall_init_vars);

#if (!defined(SEC_SYSCALL_PRE_CHECK_ALL_CALL) \
	&& !(defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && !defined(SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE))) \
	&& (defined(SEC_SYSCALL_DO_PRE_CHECK) || defined(SEC_SYSCALL_DO_POST_CHECK))

noused notrace static int __init __sec_syscall_init(void)
{
	noused size_t i;
	noused void_func *call_table;
	noused void_func *old_entries;
	noused void_func *new_entries;

	extern noused long sys_ni_syscall(void);
#if defined(SYSCALL_MAX_ARGS)
	/* since kernel version 3.5,
	 * sys_call_table[] has been declared in asm/include/syscall.h, SYSCALL_MAX_ARGS is also defined there
	 */
#else /*SYSCALL_MAX_ARGS*/
	extern noused void_func sys_call_table; /*actutally not a pointer type*/
#endif /*!SYSCALL_MAX_ARGS*/
#if defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
	extern noused void_func *__sec_syscall_old_entries_ptr;
#else /*SEC_SYSCALL_OLD_ENTRIES_BY_PTR*/
	extern noused void_func __sec_syscall_old_entries;
#endif /*!SEC_SYSCALL_OLD_ENTRIES_BY_PTR*/
#if defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
#if defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
	extern noused void_func *__sec_syscall_new_entries_ptr;
#else /*SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
	extern noused void_func __sec_syscall_new_entries;
#endif /*!SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
#endif /*SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/
	extern noused int __sec_syscall_init_entries(const uint16_t *indexes, size_t index_count,
		void_func *call_table, size_t call_table_size, void_func *old_entries,
		const void_func *new_entries);

	call_table = (void_func *)&sys_call_table;
#if defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
	old_entries = (void_func *)__sec_syscall_old_entries_ptr;
#else /*SEC_SYSCALL_OLD_ENTRIES_BY_PTR*/
	old_entries = (void_func *)&__sec_syscall_old_entries;
#endif /*!SEC_SYSCALL_OLD_ENTRIES_BY_PTR*/

#if defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
#if defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
	new_entries = (void_func *)__sec_syscall_new_entries_ptr;
#else /*SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
	new_entries = (void_func *)&__sec_syscall_new_entries;
#endif /*!SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
#else /*SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/
	new_entries = (void_func *)&__sec_syscall_new_entries[0];
#endif /*!SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/

#if 1
	__sec_syscall_init_entries(__sec_syscall_indexes, nr_invalid,
		call_table, __NR_syscalls, old_entries, new_entries);
#else
	for (i = 0; i < nr_invalid; i++) {
		unsigned int svc_no = __sec_syscall_indexes[i];

		if (svc_no < __NR_syscalls) {
			void_func svc_entry = call_table[svc_no];
			if (svc_entry != (void_func)sys_ni_syscall) {
				old_entries[i] = svc_entry;
				call_table[svc_no] = new_entries[i];
				/*printk("xjy,std,%p,%p,%p\n", old_entries[i], svc_entry, call_table[svc_no]);*/
			}
		}
	}
#endif

	return 0;
}

SEC_SYSCALL_INITCALL(__sec_syscall_init);
#endif /*(!SEC_SYSCALL_PRE_CHECK_ALL_CALL && !SEC_SYSCALL_POST_CHECK_ALL_CALL) && (SEC_SYSCALL_DO_PRE_CHECK || SEC_SYSCALL_DO_POST_CHECK)*/
