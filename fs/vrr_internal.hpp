#ifndef _VIVO_LINUX_VRR_INTERNAL_HPP
#define _VIVO_LINUX_VRR_INTERNAL_HPP

#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/version.h>

#include <linux/vrr.hpp>

/*workaround for checkpatch ERROR: Macros with complex values should be enclosed in parentheses*/

#if !defined(f_dentry)
	#define f_dentry		f_path.dentry
#endif

#ifndef intptr_t
typedef long intptr_t;
#endif

#if !defined(noused)
	#define noused __attribute__((unused))
#endif


#define __rs_alias(func) __attribute__((weak, alias(#func)))

/*__attribute__((optimize("omit-frame-pointer")))*/

#define RS_OPTIMIZE(level) __attribute__((optimize(level)))


#if !defined(RS_MARK_VAR_READONLY)
	#define __rs_text
#else
#if defined(CONFIG_EXYNOS_KERNEL_PROTECTION)
	#define __rs_text __attribute__((__section__(".rodata")))
#else
	#define __rs_text __attribute__((__section__(".text")))
#endif
#endif

#ifndef GCC_VERSION_CODE
#define GCC_VERSION_CODE (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#if !defined(PROP_NAME_MAX)
	#define PROP_NAME_MAX	32
#endif
#if !defined(PROP_VALUE_MAX)
	#define PROP_VALUE_MAX	92
#endif

typedef char *RS_SYM_PTR_TYPE;
typedef intptr_t RS_SYM_VAL_TYPE; /*sizeof(RS_SYM_VAL_TYPE)应该和sizeof(RS_SYM_PTR_TYPE)一致*/

#if defined(CONFIG_RS_CHECK_FUNC)

/*#define _FC_NAME(x) #x*/
/*#define FC_NAME(x) _FC_NAME(x)*/

#define FC_NAME(x) #x

/*#define __rs_fc(func) __attribute__((section(".text$rs$fc$"#func)))*/

#define __rs_hide __attribute__((visibility("hidden")))

#define __rs_fc(func) __attribute__((section(".rs.fc."#func)))

/*
	need modify arch/arm[64]/kernel/vmlinux.lds.S
			...
			TEXT_TEXT
			*(SORT(.rs.*))
			...
*/
#if 1
/*causing relocation overflows in calling func*/

typedef RS_SYM_PTR_TYPE RS_FC_SYM_PTR_TYPE;
typedef RS_SYM_VAL_TYPE RS_FC_SYM_VAL_TYPE; /*sizeof(RS_FC_SYM_VAL_TYPE)应该和sizeof(RS_FC_SYM_PTR_TYPE)一致*/

#define RS_FC_FUNC_BASE (RS_FC_SYM_VAL_TYPE)(PAGE_OFFSET + LINUX_VERSION_CODE + GCC_VERSION_CODE + 0x4643)

#if 0

#define RS_FC_SYM_BEGIN(sym_name) \
	(RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(&(sym_name)) - (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE))

#define RS_FC_SYM_END(sym_name) \
	(RS_FC_SYM_PTR_TYPE)((RS_FC_SYM_PTR_TYPE)(&(sym_name)) - (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE))

#endif

#endif

typedef u16 RS_FC_MARK_TYPE;

#define RS_FC_MAX_SIZE (1024 * 16) /*一次检查大小最多16KB*/

#define FC_NAME_BEGIN_SECTION(func) __rs_fc(func##$b)
#define FC_NAME_BODY_SECTION(func) __rs_fc(func##$c)
#define FC_NAME_END_SECTION(func) __rs_fc(func##$e)

#define FC_NAME_BEGIN(func) func##_b
#define FC_NAME_END(func) func##_e

#define FC_NAME_BEGIN_PUB(func) RS_HIDE(func##_b)
#define FC_NAME_END_PUB(func) RS_HIDE(func##_e)

#endif

//////////////////////////////////////////////////

#if defined(CONFIG_RS_CHECK_FUNC)

#define RS_DEFINE_FC_BEGIN(func) \
	static noused FC_NAME_BEGIN_SECTION(func) RS_FC_MARK_TYPE FC_NAME_BEGIN(func) = 0

#define RS_DEFINE_FC_BEGIN_PUB(func) \
	noused FC_NAME_BEGIN_SECTION(func) RS_FC_MARK_TYPE FC_NAME_BEGIN_PUB(func) = 0

#define RS_DEFINE_FC_BODY(func) \
	noused FC_NAME_BODY_SECTION(func)

//	noused __rs_fc(FC_NAME(func)##$c)

#define RS_DEFINE_FC_END(func) \
	static noused FC_NAME_END_SECTION(func) RS_FC_SYM_VAL_TYPE FC_NAME_END(func) \
		= (RS_FC_SYM_VAL_TYPE)((RS_FC_SYM_PTR_TYPE)(&(func)) - (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE))

#define RS_DEFINE_FC_END_PUB(func) \
	noused FC_NAME_END_SECTION(func) RS_FC_SYM_VAL_TYPE FC_NAME_END_PUB(func) \
		= (RS_FC_SYM_VAL_TYPE)((RS_FC_SYM_PTR_TYPE)(&(func)) - (RS_FC_SYM_VAL_TYPE)(RS_FC_FUNC_BASE))

	//static noused FC_NAME_END_SECTION(func) RS_FC_MARK_TYPE FC_NAME_END(func) = 0

//	static noused __rs_fc(FC_NAME(func)##$e) RS_FC_MARK_TYPE FC_NAME(func)##_e = 0

#define RS_FORWARD_FC_BEGIN(func) \
	static noused RS_FC_MARK_TYPE FC_NAME_BEGIN(func)

#define RS_FORWARD_FC_BEGIN_PUB(func) \
	extern noused RS_FC_MARK_TYPE FC_NAME_BEGIN_PUB(func)

//	static noused RS_FC_MARK_TYPE FC_NAME(func)##_b

#define RS_FORWARD_FC_END(func) \
	static noused RS_FC_SYM_VAL_TYPE FC_NAME_END(func)

#define RS_FORWARD_FC_END_PUB(func) \
	extern noused RS_FC_SYM_VAL_TYPE FC_NAME_END_PUB(func)

//	static noused RS_FC_MARK_TYPE FC_NAME_END(func)

//	static noused RS_FC_MARK_TYPE FC_NAME(func)##_e

//#define RS_FC_BEGIN(func) FC_NAME_BEGIN(func)
//#define RS_FC_END(func) FC_NAME_END(func)

#define RS_FC_PARAMS(func) &FC_NAME_END(func)

#define RS_FC_PARAMS_PUB(func) &FC_NAME_END_PUB(func)

//#define RS_FC_PARAMS(func) &FC_NAME(func), &FC_NAME_END(func)

//below causing relocation overflows in calling func
//#define RS_FC_PARAMS(func) RS_FC_SYM_BEGIN(FC_NAME(func)), RS_FC_SYM_END(FC_NAME_END(func))

#else

#define RS_DEFINE_FC_BEGIN(func)
#define RS_DEFINE_FC_BEGIN_PUB(func)
#define RS_DEFINE_FC_BODY(func)
#define RS_DEFINE_FC_END(func)
#define RS_DEFINE_FC_PUB(func)
#define RS_FORWARD_FC_BEGIN(func)
#define RS_FORWARD_FC_BEGIN_PUB(func)
#define RS_FORWARD_FC_END(func)
#define RS_FORWARD_FC_END_PUB(func)

#define RS_FC_PARAMS(func) (0)
#define RS_FC_PARAMS_PUB(func) (0)

#endif

/////////////////////////////////////////////////////////////

typedef union tag_mount_check_info_flags {
	u32 value;

	struct {
		u8 op_type;
		u8 op_info;
		u16 op_flags;
	} __attribute__((packed)) op_data;
} mount_check_info_flags;

typedef struct tag_mount_check_info {
	int ignore_mount_record;
	int need_free_task_infos;
	char *task_infos;
	int task_infos_len;
	mount_check_info_flags flags;
} mount_check_info;

typedef struct tag_do_mount_check_param {
	struct path *path_ptr;
	unsigned int *mnt_flags_ptr;
	const char *dev_name;
	const char *dir_name;
	const char *type_page;
	unsigned long flags;
	void *data_page;
	bool dir_name_from_user;
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK)
#if defined(RS_DO_MOUNT_USE_USER_DIR_NAME)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	char *kernel_dir;
#else
	struct filename *kernel_dir;
#endif
#endif
	noused unsigned long *bitmap_buffer;
#endif
#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK)
	int check_flags;
#endif
#if defined(RS_NEED_DO_MOUNT_CHECK)
	char *dir_name_buffer;
	mount_check_info check_info;
#endif
} do_mount_check_param;

#if defined(RS_NEED_DO_MOUNT_CHECK) || defined(RS_NEED_DO_BOOT_CHECK) || defined(CONFIG_MOUNT_FILES_CHECK)

#define vrr_do_mount_init_check_param RS_HIDE(_x0)
extern void vrr_do_mount_init_check_param(struct tag_do_mount_check_param *param,
	struct path *path_ptr, unsigned int *mnt_flags_ptr,
	const char *dev_name, const char __user *dir_name,
	const char *type_page, unsigned long flags, void *data_page);

#define vrr_do_mount_prepare_check_param RS_HIDE(_x1)
extern int vrr_do_mount_prepare_check_param(struct tag_do_mount_check_param *param);
#define vrr_do_mount_get_mountpoint RS_HIDE(_x2)
extern bool vrr_do_mount_get_mountpoint(struct tag_do_mount_check_param *param, int *retval_ptr);
#define vrr_do_mount_check_before_mount RS_HIDE(_x3)
extern bool vrr_do_mount_check_before_mount(struct tag_do_mount_check_param *param, int *retval_ptr);
#define vrr_do_mount_check_after_mount RS_HIDE(_x4)
extern void vrr_do_mount_check_after_mount(struct tag_do_mount_check_param *param, int retval);
#define vrr_do_mount_done_check_param RS_HIDE(_x5)
extern void vrr_do_mount_done_check_param(struct tag_do_mount_check_param *param);

#define	VIVO_DO_MOUNT_DEFINE_CHECK_PARAM	struct tag_do_mount_check_param check_param;

#define	VIVO_DO_MOUNT_INIT_CHECK_PARAM(path_ptr, mnt_flags_ptr, dev_name, dir_name, type_page, flags, data_page)	\
	vrr_do_mount_init_check_param(&check_param, (path_ptr), (mnt_flags_ptr),	\
		(dev_name), (dir_name), (type_page), (flags), (data_page));

#define	VIVO_DO_MOUNT_PREPARE_CHECK_PARAM	\
	vrr_do_mount_prepare_check_param(&check_param)

#define	VIVO_DO_MOUNT_GET_MOUNTPOINT(retval_ptr)	\
	vrr_do_mount_get_mountpoint(&check_param, (retval_ptr))

#define	VIVO_DO_MOUNT_CHECK_BEFORE_MOUNT(retval_ptr)	\
	vrr_do_mount_check_before_mount(&check_param, (retval_ptr))

#define	VIVO_DO_MOUNT_CHECK_AFTER_MOUNT(retval)	\
	vrr_do_mount_check_after_mount(&check_param, (retval))

#define	VIVO_DO_MOUNT_DONE_CHECK_PARAM	\
	vrr_do_mount_done_check_param(&check_param)

#else
	#define	VIVO_DO_MOUNT_DEFINE_CHECK_PARAM

	#define VIVO_DO_MOUNT_INIT_CHECK_PARAM(path_ptr, mnt_flags_ptr, dev_name, dir_name, type_page, flags, data_page)	\
		do {} while (0)

	#define	VIVO_DO_MOUNT_PREPARE_CHECK_PARAM	(0)
	#define	VIVO_DO_MOUNT_GET_MOUNTPOINT(retval_ptr)	(false)
	#define	VIVO_DO_MOUNT_CHECK_BEFORE_MOUNT(retval_ptr)	(true)
	#define	VIVO_DO_MOUNT_CHECK_AFTER_MOUNT(retval)	do {} while(0)
	#define	VIVO_DO_MOUNT_DONE_CHECK_PARAM	do {} while(0)

#endif


#if defined(CONFIG_RS_CHECK_FUNC) && defined(CONFIG_MOUNT_RESTRICT)
	#define	VIVO_DEFINE_FC_DO_MOUNT_BEGIN_STUB	\
		RS_DEFINE_FC_BEGIN_PUB(do_mount);			\
		RS_FORWARD_FC_END_PUB(do_mount);

	#define	VIVO_DEFINE_FC_DO_MOUNT_BODY	RS_DEFINE_FC_BODY(do_mount)

	#define	VIVO_DEFINE_FC_DO_MOUNT_END_STUB	\
		RS_DEFINE_FC_END_PUB(do_mount);
#else
	#define	VIVO_DEFINE_FC_DO_MOUNT_BEGIN_STUB
	#define	VIVO_DEFINE_FC_DO_MOUNT_BODY
	#define	VIVO_DEFINE_FC_DO_MOUNT_END_STUB
#endif


#endif /* _VIVO_LINUX_VRR_INTERNAL_HPP */
