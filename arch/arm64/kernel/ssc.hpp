
#if defined(SEC_SYSCALL_INCLUDE_HEADER_FILES)

#if !defined(CONFIG_COMPAT)
	#undef SEC_SYSCALL_COMPAT
#endif //!CONFIG_COMPAT

#if !defined(CONFIG_OABI_COMPAT)
	#undef SEC_SYSCALL_OABI_COMPAT
#endif

#if defined(SEC_SYSCALL_INCLUDE_UNISTD_H)

#if !defined(SEC_SYSCALL_COMPAT)
	#include <linux/unistd.h>
	//#include <uapi/asm-generic/unistd.h>
#else
	#include <asm/unistd32.h>
#endif //SEC_SYSCALL_COMPAT

#endif //SEC_SYSCALL_INCLUDE_UNISTD_H

#include <linux/version.h>

#if !defined(CONFIG_ARM64)
#if !defined(S_OFF)
	//#include "entry-header.S"

	#define S_OFF		(8)
#endif //!S_OFF
#endif //!CONFIG_ARM64

///////////////////////////////////////////

//whether svc exit will restore task's original addr_limit
#if !defined(CONFIG_ARM64)
	//#define SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT
#else
	//only restore original addr_limit for exception handler (el != 0)
	//while el0_svc/el0_svc_compat run in el0 mode
	//#define SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	//entry.S call el0_svc_handler()
	#define SSC_ARCH_USE_SYSCALL_WRAPPER
#endif
		#define SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX

	#if !defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
		#undef SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX
	#endif

	#if !defined(REG_SYSCALL_IDX)
	#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
		#if defined(SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX)
			#if defined(CONFIG_KASAN) || defined(CONFIG_KCOV)
				//incase el0_svc_handler/el0_svc_common/invoke_syscall do not x7
				#define REG_SYSCALL_IDX x7 //x4~x7 is not used currently

				#define SSC_NO_PRE_CHECK_WITH_CALL_NUMBER
			#else
				//incase el0_svc_handler/el0_svc_common/invoke_syscall do not use x27(stbl)
				#define REG_SYSCALL_IDX x27
			#endif
		#else
			#define REG_SYSCALL_IDX x8
		#endif
	#else
		#define REG_SYSCALL_IDX x20
	#endif
	#endif

#endif

//#define SEC_SYSCALL_CHECK_UID_CHANGE //check uid/euid/sid illegal change during syscalls
	#define SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS //store uid .etc checking flag in pt_regs's unused field
	#define SEC_SYSCALL_STORE_CUC_FLAGS_IN_TASK_STRUCT //store uid .etc checking flag in task struct's unused bit-field

#if defined(SEC_SYSCALL_CHECK_UID_CHANGE)
	#define SEC_SYSCALL_PRE_CHECK_ALL_CALL //pre check all syscalls
		//#define SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER
#endif

#if defined(SSC_NO_PRE_CHECK_WITH_CALL_NUMBER)
	#undef SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER
#endif

#define SEC_SYSCALL_POST_CHECK_ALL_CALL //post check all syscalls, always call force_sig()
	#define SEC_SYSCALL_POST_CHECK_IN_RET_FAST_SYSCALL //do post check in ret_fast_syscall(), check once per syscall
	#define SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE //no branch to check function

#if defined(SEC_SYSCALL_CHECK_UID_CHANGE)
	#undef SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE
#endif

#define SEC_SYSCALL_DO_PRE_CHECK //pre syscall check
	//#define SEC_SYSCALL_PRE_CHECK_VERIFY_SELINUX_PERMISSIVE //check task permissive flag
	//#define SEC_SYSCALL_ANDROID_4_4_OTHER_MODE_SUPPORT //android 4.4 recovery mode, native services without seclabel got killed, can't bootup


//#define SEC_SYSCALL_DO_POST_CHECK //post syscall check
	//#define SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE //check task permissive flag
	//#define SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD //check task kthread flag

#define SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL //check init task stage by execve() call

#define SEC_SYSCALL_DO_MINIMAL_CHECK //only check a few important syscalls


//#define SEC_SYSCALL_CHECK_SELINUX_SID //check selinux sid, exec_sid, will kill init when >= ard 10.0
	#define SEC_SYSCALL_DEFINE_SELINUX_SECURITY_TYPES //do not include selinux objsec.h


#define SEC_SYSCALL_FORCE_STOP //call force_sig()
	//#define SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP //kill processes in same cgroup
	#define SEC_SYSCALL_FORCE_STOP_BY_GROUP_EXIT //call do_group_exit()
	//#define SEC_SYSCALL_FORCE_STOP_WITH_LOG //print stopped process info
	#if !defined(SEC_SYSCALL_GROUP_EXIT_CODE)
		#define SEC_SYSCALL_GROUP_EXIT_CODE (0) //(SIGKILL)
	#endif

	#define SEC_SYSCALL_CALL_FORCE_STOP_IN_ASM

//#define SEC_SYSCALL_CALL_FORCE_STOP_DEBUG //for debug init task got killed issue

#if defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
	#undef SEC_SYSCALL_FORCE_STOP
#endif

#if defined(SEC_SYSCALL_FORCE_STOP) || defined(SEC_SYSCALL_CALL_FORCE_STOP_DEBUG)
	#define SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS
#endif

#define SEC_SYSCALL_OLD_ENTRIES_BY_PTR //init by __sec_syscall_old_entries_ptr

#define SEC_SYSCALL_STUBS_ARRAY_IN_ASM //define stub array in .S
	#define SEC_SYSCALL_NEW_ENTRIES_BY_PTR //init by __sec_syscall_new_entries_ptr

#define SEC_SYSCALL_INIT_TASK_HAS_SUPPLEMENTARY_GROUPS

#define SEC_SYSCALL_PREFER_PERFORMANCE //syscall hooks performance prefered

#if !defined(SEC_SYSCALL_FORCE_STOP) && !defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) \
	&& !defined(SEC_SYSCALL_DO_POST_CHECK)
	#undef SEC_SYSCALL_CALL_FORCE_STOP_IN_ASM
#endif

#if !defined(CONFIG_SEC_SYSCALL)
	#undef SEC_SYSCALL_PRE_CHECK_ALL_CALL
	#undef SEC_SYSCALL_POST_CHECK_ALL_CALL
#endif

#if !(defined(CONFIG_CGROUPS) && defined(CONFIG_CGROUP_CPUACCT))
	#undef SEC_SYSCALL_FORCE_STOP_BY_PROCESS_GROUP
#endif

#if !defined(CONFIG_SECURITY) || !defined(CONFIG_SECURITY_SELINUX)
	#undef SEC_SYSCALL_CHECK_SELINUX_SID
	#undef SEC_SYSCALL_PRE_CHECK_VERIFY_SELINUX_PERMISSIVE
	#undef SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE
#else //CONFIG_SECURITY && CONFIG_SECURITY_SELINUX
	#if defined(SEC_SYSCALL_CHECK_SELINUX_SID) && !defined(CONFIG_ARM64)
		#undef SEC_SYSCALL_CALL_FORCE_STOP_IN_ASM
	#endif
#endif //!CONFIG_SECURITY || !CONFIG_SECURITY_SELINUX

#if !defined(CONFIG_ARM64)
	//#define SEC_SYSCALL_ARM_THUMB_SUPPORT_RET_BY_REG //msm8937 6.x support "ret lr"
	#undef SEC_SYSCALL_STORE_CUC_FLAGS_IN_PT_REGS //only arm64 pt_regs has unused field
#endif


#if defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && defined(SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE)
	#if !defined(SEC_SYSCALL_POST_CHECK_VERIFY_SELINUX_PERMISSIVE) && !defined(SEC_SYSCALL_POST_CHECK_VERIFY_KTHREAD)
		#if defined(SEC_SYSCALL_SVC_RESTORE_ADDR_LIMIT)
			//svc exit will restore task's original addr_limit, no need to check addr_limit anymore
			//#undef SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE
			//#undef SEC_SYSCALL_POST_CHECK_ALL_CALL
		#else
			#if defined(__ASSEMBLY__)
				#include <asm/asm-offsets.h>

				#if (defined(SVC_ADDR_LIMIT) && !defined(CONFIG_64BIT)) \
					|| (defined(S_ORIG_ADDR_LIMIT) && defined(CONFIG_64BIT))
					//svc exit will restore task's original addr_limit, no need to check addr_limit anymore
					//#undef SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE
					//#undef SEC_SYSCALL_POST_CHECK_ALL_CALL
				#endif
			#endif
		#endif
	#endif
#endif


#if defined(__ASSEMBLY__)

#if defined(CONFIG_ARM64)
	#define SEC_SYSCALL_DEFAULT_ALIGN .balign	8
#else
	#define SEC_SYSCALL_DEFAULT_ALIGN .balign	4
#endif

#if defined(CONFIG_ARM64)
	#define SEC_SYSCALL_DEFINE_NEW_ENTRY(value) .quad	value
#else
	#define SEC_SYSCALL_DEFINE_NEW_ENTRY(value) .long	value
#endif

#if defined(CONFIG_ARM64)
	#define SEC_SYSCALL_DEFINE_OLD_ENTRY(value) .quad	value
#else
	#define SEC_SYSCALL_DEFINE_OLD_ENTRY(value) .long	value
#endif

#if defined(CONFIG_ARM64)
	#define SEC_SYSCALL_DEFINE_PTR(value) .quad	value
#else
	#define SEC_SYSCALL_DEFINE_PTR(value) .long	value
#endif

#if defined(CONFIG_EXYNOS_KERNEL_PROTECTION) && !defined(SEC_SYSCALL_PREFER_PERFORMANCE)
	#define SEC_SYSCALL_OLD_ENTREIS_SECTION ".rodata" //".data" //".bss"
#else
	#define SEC_SYSCALL_OLD_ENTREIS_SECTION ".text" //".rodata" //".data" //".bss"
#endif

#define SEC_SYSCALL_NEW_ENTREIS_SECTION ".init.rodata", "a"

#else //__ASSEMBLY__

#if !defined(noused)
	#define noused __attribute__((unused))
#endif

#if !defined(__naked)
	#define __naked __attribute__((naked)) noinline __no_clone notrace
#endif

#if defined(CONFIG_EXYNOS_KERNEL_PROTECTION) && defined(SEC_SYSCALL_PREFER_PERFORMANCE)
	/* refer to core_initcall(exynos_protect_kernel_text) in drivers/soc/samsung/exynos-el3_mon.c */
	#define SEC_SYSCALL_INITCALL(fn) pure_initcall(fn)
#else
#if defined(CONFIG_RANDOMIZE_BASE)
	/* console_initcall()/security_initcall() doesn't work, kernel fail to boot without UART log output */
	#define SEC_SYSCALL_INITCALL(fn) late_initcall(fn) //core_initcall(fn) //pure_initcall(fn)
#else
	#define SEC_SYSCALL_INITCALL(fn) device_initcall(fn) //security_initcall(fn) //console_initcall(fn)
#endif
#endif

typedef void (*void_func)(void);

enum ssc_boot_mode_t {
	SSC_NORMAL_BOOT = 0,
	SSC_RECOVERY_BOOT = 1,
	SSC_FACTORY_BOOT = 2,
	SSC_SURVIVAL_BOOT = 3,
	SSC_UNKNOWN_BOOT,
};

extern enum ssc_boot_mode_t ssc_get_boot_mode(void);

#endif //!__ASSEMBLY__

#endif //SEC_SYSCALL_INCLUDE_HEADER_FILES


/////////////////////////////////////////////////

#if defined(SEC_SYSCALL_DEFINE_ENUMERATION)

//refer to http://embdev.net/topic/153828

#if defined(__ASSEMBLY__)

  .set last_enum_value, 0
  .macro enum_val name
  .equiv nr_\name, last_enum_value
  .set last_enum_value, last_enum_value + 1
  .endm

#define ENUM_BEGIN  .set last_enum_value, 0

#define ENUM_VAL(name) enum_val name
#define ENUM_VALASSIGN(name, value)            \
  .set last_enum_value, value                 ;\
  enum_val name
#define ENUM_END(enum_name)

#else //__ASSEMBLY__

#define ENUM_BEGIN typedef enum {
#define ENUM_VAL(name) nr_##name,
#define ENUM_VALASSIGN(name, value) nr_##name = value,
#define ENUM_END(enum_name) } enum_name;

#endif //!__ASSEMBLY__

#define SEC_SYSCALL_SVC_OP(svc_name) ENUM_VAL(svc_name)


ENUM_BEGIN
#include "ssc_in.h"
SEC_SYSCALL_SVC_OP(invalid) //for nr_invalid
ENUM_END(sec_syscall_enum)

#undef SEC_SYSCALL_SVC_OP

#endif //SEC_SYSCALL_INCLUDE_ENUMERATION

///////////////////////////////////////////
#if !defined(__ASSEMBLY__) && defined(SEC_SYSCALL_DEFINE_COMMON_STUB)

#if defined(CONFIG_ARM64)

//no CONFIG_OABI_COMPAT

#if defined(SEC_SYSCALL_OABI_COMPAT)

noused static /*__naked*/ void __sec_syscall_oabi_compat_common_stub(void)
{
/*
	 * x21 - aborted SP
	 * x22 - aborted PC
	 * x23 - aborted PSTATE
sc_nr	.req	x25		// number of system calls
scno	.req	x26		// syscall number
stbl	.req	x27		// syscall table pointer
tsk	.req	x28		// current thread_info
sc_entry .req x16 //current syscall entry

x20: entry index for __sec_syscall_oabi_compat_old_entries[]
*/
	//register void_func *saved_entries asm("x19") = __sec_syscall_oabi_compat_old_entries;

	asm volatile ("	mov	x19, %0	;" //__sec_syscall_oabi_compat_old_entries -> x19
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	x0, x28	;" //thread_info -> x0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	x1, x26	;" //scno -> x1 for __sec_syscall_pre_check()
		#endif
			"		mov	x21, x30	;" //LR -> x21
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	x30, x21	;" //x21 -> LR
			"		cbnz	x0, 1f	;" //direct return if result is not zero
		#endif
			"		ldr	x16, [x19, x20, lsl #3]	;" //__sec_syscall_oabi_compat_old_entries[x20] -> x16
			"		ldp	x0, x1, [sp]	;" //restore args
			"		ldp	x2, x3, [sp, #16]	;"
		#if defined(SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS)
			"		ldp	x4, x5, [sp, #32]	;"
			"		ldp	x6, x7, [sp, #48]	;"
		#endif
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	x21, x30	;" //LR -> x21
			#endif
			"		adr	x30, ret_from_entry	;" //LR already saved in x21
		#endif
			"		br	x16	;" //jump to saved entry
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov	x20, x0	;" //save result
			"		mov	x0, x28	;" //thread_info -> x0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	x0, x20	;" //restore result
			"		mov	x30, x21	;" //x21 -> LR
		#endif
			"1:	;"
			:
			: "r" (__sec_syscall_oabi_compat_old_entries)
			: "x0", "x1"
			);
}
#elif !defined(SEC_SYSCALL_COMPAT)

noused static /*__naked*/ void __sec_syscall_common_stub(void)
{
/*
	 * x21 - aborted SP
	 * x22 - aborted PC
	 * x23 - aborted PSTATE
sc_nr	.req	x25		// number of system calls
scno	.req	x26		// syscall number
stbl	.req	x27		// syscall table pointer
tsk	.req	x28		// current thread_info
sc_entry .req x16 //current syscall entry

x20: entry index for __sec_syscall_old_entries[]
*/
	//register void_func *saved_entries asm("x19") = __sec_syscall_old_entries;

	asm volatile ("	mov	x19, %0	;" //__sec_syscall_old_entries -> x19
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	x0, x28	;" //thread_info -> x0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	x1, x26	;" //scno -> x1 for __sec_syscall_pre_check()
		#endif
			"		mov	x21, x30	;" //LR -> x21
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	x30, x21	;" //x21 -> LR
			"		cbnz	x0, 1f	;" //direct return if result is not zero
		#endif
			"		ldr	x16, [x19, x20, lsl #3]	;" //__sec_syscall_old_entries[x20] -> x16
			"		ldp	x0, x1, [sp]	;" //restore args
			"		ldp	x2, x3, [sp, #16]	;"
		#if defined(SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS)
			"		ldp	x4, x5, [sp, #32]	;"
			"		ldp	x6, x7, [sp, #48]	;"
		#endif
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	x21, x30	;" //LR -> x21
			#endif
			"		adr	x30, ret_from_entry	;" //LR already saved in x21
		#endif
			"		br	x16	;" //jump to saved entry
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov	x20, x0	;" //save result
			"		mov	x0, x28	;" //thread_info -> x0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	x0, x20	;" //restore result
			"		mov	x30, x21	;" //x21 -> LR
		#endif
			"1:	;"
			:
			: "r" (__sec_syscall_old_entries)
			: "x0", "x1"
			);
}
#else //!SEC_SYSCALL_COMPAT

noused static /*__naked*/ void __sec_syscall_compat_common_stub(void)
{
/*
	 * x21 - aborted SP
	 * x22 - aborted PC
	 * x23 - aborted PSTATE
sc_nr	.req	x25		// number of system calls
scno	.req	x26		// syscall number
stbl	.req	x27		// syscall table pointer
tsk	.req	x28		// current thread_info
sc_entry .req x16 //current syscall entry

x20: entry index for __sec_syscall_old_entries[]
*/
	//register void_func *saved_entries asm("x19") = __sec_syscall_compat_old_entries;

	asm volatile ("	mov	x19, %0	;" //__sec_syscall_compat_old_entries -> x19
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	x0, x28	;" //thread_info -> x0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	x1, x26	;" //scno -> x1 for __sec_syscall_pre_check()
		#endif
			"		mov	x21, x30	;" //LR -> x21
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	x30, x21	;" //x21 -> LR
			"		cbnz	x0, 1f	;" //direct return if result is not zero
		#endif
			"		ldr	x16, [x19, x20, lsl #3]	;" //__sec_syscall_compat_old_entries[x20] -> x16
			"		ldp	x0, x1, [sp]	;" //restore args
			"		ldp	x2, x3, [sp, #16]	;"
		#if defined(SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS)
			"		ldp	x4, x5, [sp, #32]	;"
			"		ldp	x6, x7, [sp, #48]	;"
		#endif
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov x21, x30	;" //LR -> x21
			#endif
			"		adr	x30, ret_from_entry	;" //LR already saved in x21
		#endif
			"		br	x16	;" //jump to saved entry
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov x20, x0	;" //save result
			"		mov	x0, x28	;" //thread_info -> x0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	x0, x20	;" //restore result
			"		mov	x30, x21	;" //x21 -> LR
		#endif
			"1:	;"
			:
			: "r" (__sec_syscall_compat_old_entries)
			: "x0", "x1"
			);
}
#endif //SEC_SYSCALL_COMPAT

#else //CONFIG_ARM64

#if defined(SEC_SYSCALL_OABI_COMPAT)
noused static __naked void __sec_syscall_oabi_compat_common_stub(void)
{
/*
r4: fifth arg (already pushed in stack)
r5: sixth arg (already pushed in stack)
scno	.req	r7		@ syscall number
tbl	.req	r8		@ syscall table pointer
why	.req	r8		@ Linux syscall (!= 0)
tsk	.req	r9		@ current thread_info

r10: entry index for __sec_syscall_oabi_compat_old_entries[]
r11: __sec_syscall_oabi_compat_old_entries
r6: save lr, r6 will be reload in "work_pending" //r12 may be trashed(used as temp var)
*/
	//register void_func *saved_entries asm("r11") = __sec_syscall_oabi_compat_old_entries;

	asm volatile ("	mov	r11, %0	;" //__sec_syscall_oabi_compat_old_entries -> r11
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r0, r9	;" //thread_info -> r0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	r1, r7	;" //scno -> r1 for __sec_syscall_pre_check()
		#endif
			"		mov	r6, lr	;" //LR -> r6
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	lr, r6	;" //r6 -> LR
			"		tst	r0, #0	;"
			"		bne	1f	;" //direct return if result is not zero
		#endif
			"		add	r1, sp, %1	;" //restore args
			"		ldmia	r1, {r0 - r3}	;" //ldmia	r1, {r0 - r5}	;"
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r6, lr	;" //r6 -> LR
			#endif
		#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
			"		badr	lr, ret_from_entry	;" //LR already saved in r6
		#else
		#if defined(BSYM)
			"		adr	lr, BSYM(ret_from_entry)	;" //LR already saved in r6
		#else
			#if defined(CONFIG_THUMB2_KERNEL) //refer to the BSYM() macro
			"		adr	lr, ret_from_entry + 1	;" //LR already saved in r6
			#else
			"		adr	lr, ret_from_entry	;" //LR already saved in r6
			#endif
		#endif
		#endif
		#endif
			"		ldr	pc, [r11, r10, lsl #2]	;" //__sec_syscall_oabi_compat_old_entries[r10] -> pc
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov	r10, r0	;" //save result
			"		mov	r0, r9	;" //thread_info -> x0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	r0, r10	;" //restore result
			"		mov	lr, r6	;" //r6 -> LR
		#endif
		#if defined(SEC_SYSCALL_ARM_THUMB_SUPPORT_RET_BY_REG)
			"1:	ret	lr	;"
		#else
			"1:	mov	pc, lr	;"
		#endif
			:
			: "r" (__sec_syscall_oabi_compat_old_entries), "i" (S_OFF)
			: "r0", "r1"
			);
}
#elif !defined(SEC_SYSCALL_COMPAT)

noused static __naked void __sec_syscall_common_stub(void)
{
/*
scno	.req	r7		@ syscall number
tbl	.req	r8		@ syscall table pointer
why	.req	r8		@ Linux syscall (!= 0)
tsk	.req	r9		@ current thread_info

r10: entry index for __sec_syscall_old_entries[]
r11: __sec_syscall_old_entries
r6: save lr
*/
	//register void_func *saved_entries asm("r11") = __sec_syscall_old_entries;

	asm volatile ("	mov	r11, %0	;" //__sec_syscall_old_entries -> r11
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r0, r9	;" //thread_info -> r0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	r1, r7	;" //scno -> r1 for __sec_syscall_pre_check()
		#endif
			"		mov	r6, lr	;" //LR -> r6
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	lr, r6	;" //r6 -> LR
			"		tst	r0, #0	;"
			"		bne	1f	;" //direct return if result is not zero
		#endif
			"		add	r1, sp, %1	;" //restore args
			"		ldmia	r1, {r0 - r3}	;" //ldmia	r1, {r0 - r5}	;"
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r6, lr	;" //LR -> r6
			#endif
		#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
			"		badr	lr, ret_from_entry	;" //LR already saved in r6
		#else
		#if defined(BSYM)
			"		adr	lr, BSYM(ret_from_entry)	;" //LR already saved in r6
		#else
			#if defined(CONFIG_THUMB2_KERNEL) //refer to the BSYM() macro
			"		adr	lr, ret_from_entry + 1	;" //LR already saved in r6
			#else
			"		adr	lr, ret_from_entry	;" //LR already saved in r6
			#endif
		#endif
		#endif
		#endif
			"		ldr	pc, [r11, r10, lsl #2]	;" //__sec_syscall_old_entries[r10] -> pc
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov	r10, r0	;" //save result
			"		mov	r0, r9	;" //thread_info -> x0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	r0, r10	;" //restore result
			"		mov	lr, r6	;" //r6 -> LR
		#endif
		#if defined(SEC_SYSCALL_ARM_THUMB_SUPPORT_RET_BY_REG)
			"1:	ret	lr	;"
		#else
			"1:	mov	pc, lr	;"
		#endif
			:
			: "r" (__sec_syscall_old_entries), "i" (S_OFF)
			: "r0", "r1"
			);
}
#else
noused static __naked void __sec_syscall_compat_common_stub(void)
{
/*
scno	.req	r7		@ syscall number
tbl	.req	r8		@ syscall table pointer
why	.req	r8		@ Linux syscall (!= 0)
tsk	.req	r9		@ current thread_info

r10: entry index for __sec_syscall_comp_old_entries[]
r11: __sec_syscall_compat_old_entries
r6: save lr
*/
	//register void_func *saved_entries asm("r11") = __sec_syscall_compat_old_entries;

	asm volatile ("	mov	r11, %0	;" //__sec_syscall_compat_old_entries -> r11
		#if defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r0, r9	;" //thread_info -> r0 for __sec_syscall_pre_check()
		#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
			"		mov	r1, r7	;" //scno -> r1 for __sec_syscall_pre_check()
		#endif
			"		mov	r6, lr	;" //LR -> r6
			"		bl	__sec_syscall_pre_check	;" // call __sec_syscall_pre_check()
			"		mov	lr, r6	;" //r6 -> LR
			"		tst	r0, #0	;"
			"		bne	1f	;" //direct return if result is not zero
		#endif
			"		add	r1, sp, %1	;" //restore args
			"		ldmia	r1, {r0 - r3}	;" //ldmia	r1, {r0 - r5}	;"
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
			"		mov	r6, lr	;" //LR -> r6
			#endif
		#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
			"		badr	lr, ret_from_entry	;" //LR already saved in r6
		#else
		#if defined(BSYM)
			"		adr	lr, BSYM(ret_from_entry)	;" //LR already saved in r6
		#else
			#if defined(CONFIG_THUMB2_KERNEL) //refer to the BSYM() macro
			"		adr	lr, ret_from_entry + 1	;" //LR already saved in r6
			#else
			"		adr	lr, ret_from_entry	;" //LR already saved in r6
			#endif
		#endif
		#endif
		#endif
			"		ldr	pc, [r11, r10, lsl #2]	;" //__sec_syscall_compat_old_entries[r10] -> pc
		#if defined(SEC_SYSCALL_DO_POST_CHECK)
			"ret_from_entry:	mov	r10, r0	;" //save result
			"		mov	r0, r9	;" //thread_info -> r0
			"		bl	__sec_syscall_post_check	;" //may call force_sig()
			"		mov	r0, r10	;" //restore result
			"		mov	lr, r6	;" //r6 -> LR
		#endif
		#if defined(SEC_SYSCALL_ARM_THUMB_SUPPORT_RET_BY_REG)
			"1:	ret	lr	;"
		#else
			"1:	mov	pc, lr	;"
		#endif
			:
			: "r" (__sec_syscall_comp_old_entries), "i" (S_OFF)
			: "r0", "r1"
			);
}
#endif //SEC_SYSCALL_COMPAT

#endif //!CONFIG_ARM64

#endif //!__ASSEMBLY__ && SEC_SYSCALL_DEFINE_COMMON_STUB

///////////////////////////////////////////
#if defined(SEC_SYSCALL_DEFINE_STUBS)

#if defined(CONFIG_ARM64)

//no CONFIG_OABI_COMPAT

#if defined(__ASSEMBLY__)

#if defined(SEC_SYSCALL_OABI_COMPAT)

.macro __DEFINE_SEC_SYSCALL svc_name:req
#if !defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
	.global __sec_syscall_oabi_compat_\svc_name\()_stub
#endif
	.type	__sec_syscall_oabi_compat_\svc_name\()_stub, #function
__sec_syscall_oabi_compat_\svc_name\()_stub:
	mov REG_SYSCALL_IDX, #nr_\svc_name
	b __sec_syscall_oabi_compat_common_stub
.endm

#elif !defined(SEC_SYSCALL_COMPAT)

.macro __DEFINE_SEC_SYSCALL svc_name:req
#if !defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
	.global __sec_syscall_\svc_name\()_stub
#endif
	.type	__sec_syscall_\svc_name\()_stub, #function
__sec_syscall_\svc_name\()_stub:
	mov REG_SYSCALL_IDX, #nr_\svc_name
	b __sec_syscall_common_stub
.endm

#else //!SEC_SYSCALL_COMPAT

.macro __DEFINE_SEC_SYSCALL svc_name:req
#if !defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
	.global __sec_syscall_compat_\svc_name\()_stub
#endif
	.type	__sec_syscall_compat_\svc_name\()_stub, #function
__sec_syscall_compat_\svc_name\()_stub:
	mov REG_SYSCALL_IDX, #nr_\svc_name
	b __sec_syscall_compat_common_stub
.endm

#endif //SEC_SYSCALL_COMPAT

#define DEFINE_SEC_SYSCALL(svc_name) __DEFINE_SEC_SYSCALL svc_name

#else //__ASSEMBLY__

#if !defined(SEC_SYSCALL_COMPAT)

#define DEFINE_SEC_SYSCALL(svc_name)							\
noused static /*__naked*/ void __sec_syscall_##svc_name##_stub(void)	\
{																						\
	asm volatile ("	mov	REG_SYSCALL_IDX, %0	;"										\
			"		br %1	;"													\
			:																			\
			: "i" (nr_##op), "r" (__sec_syscall_common_stub)		\
			: "cc"																\
			);																		\
}

#else //!SEC_SYSCALL_COMPAT

#define DEFINE_SEC_SYSCALL(svc_name)							\
noused static /*__naked*/ void __sec_syscall_compat_##svc_name##_stub(void)	\
{																						\
	asm volatile ("	mov	REG_SYSCALL_IDX, %0	;"										\
			"		br %1	;"													\
			:																			\
			: "i" (nr_##op), "r" (__sec_syscall_compat_common_stub)		\
			: "cc"																\
			);																		\
}

#endif //SEC_SYSCALL_COMPAT

#endif //!__ASSEMBLY__

#else //CONFIG_ARM64

/*
scno r7
tbl r8
tsk r9

r4 (fifth arg) <- sp
r5 (sixth arg)
r0
r1
r2
r3
r4
r5
r6
*/

#if defined(__ASSEMBLY__)

#if defined(SEC_SYSCALL_OABI_COMPAT)

.macro __DEFINE_SEC_SYSCALL svc_name:req
	.global __sec_syscall_oabi_compat_\svc_name\()_stub
	.type	__sec_syscall_oabi_compat_\svc_name\()_stub, #function
__sec_syscall_oabi_compat_\svc_name\()_stub:
	mov r10, #nr_\svc_name
	b __sec_syscall_oabi_compat_common_stub
.endm

#elif !defined(SEC_SYSCALL_COMPAT)

.macro __DEFINE_SEC_SYSCALL svc_name:req
	.global __sec_syscall_\svc_name\()_stub
	.type	__sec_syscall_\svc_name\()_stub, #function
__sec_syscall_\svc_name\()_stub:
	mov r10, #nr_\svc_name
	b __sec_syscall_common_stub
.endm

#else //!SEC_SYSCALL_COMPAT

.macro __DEFINE_SEC_SYSCALL svc_name:req
	.global __sec_syscall_compat_\svc_name\()_stub
	.type	__sec_syscall_compat_\svc_name\()_stub, #function
__sec_syscall_compat_\svc_name\()_stub:
	mov r10, #nr_\svc_name
	b __sec_syscall_compat_common_stub
.endm

#endif //SEC_SYSCALL_COMPAT

#define DEFINE_SEC_SYSCALL(svc_name) __DEFINE_SEC_SYSCALL svc_name

#else //__ASSEMBLY__

#if defined(SEC_SYSCALL_OABI_COMPAT)

#define DEFINE_SEC_SYSCALL(svc_name)							\
noused static __naked void __sec_syscall_oabi_compat_##svc_name##_stub(void)	\
{																						\
	asm volatile ("	mov	r10, %0	;"										\
			"		b __sec_syscall_oabi_compat_common_stub	;"													\
			:																			\
			: "i" (nr_##op)												\
			: "cc"																\
			);																		\
}

#elif !defined(SEC_SYSCALL_COMPAT)

#define DEFINE_SEC_SYSCALL(svc_name)							\
noused static __naked void __sec_syscall_##svc_name##_stub(void)	\
{																						\
	asm volatile ("	mov	r10, %0	;"										\
			"		b __sec_syscall_common_stub	;"													\
			:																			\
			: "i" (nr_##op)												\
			: "cc"																\
			);																		\
}

#else //!SEC_SYSCALL_COMPAT

#define DEFINE_SEC_SYSCALL(svc_name)							\
noused static __naked void __sec_syscall_compat_##svc_name##_stub(void)	\
{																						\
	asm volatile ("	mov	r10, %0	;"										\
			"		b __sec_syscall_compat_common_stub	;"													\
			:																			\
			: "i" (nr_##op)												\
			: "cc"																\
			);																		\
}

#endif //SEC_SYSCALL_COMPAT

#endif //!__ASSEMBLY__

#endif //!CONFIG_ARM64

#define SEC_SYSCALL_SVC_OP(svc_name) DEFINE_SEC_SYSCALL(svc_name)
#include "ssc_in.h"
#undef SEC_SYSCALL_SVC_OP

#endif //SEC_SYSCALL_DEFINE_STUBS

//////////////////////////////////////////

#if !defined(__ASSEMBLY__)

//#if defined(SEC_SYSCALL_DEFINE_STUBS_REFERENCE_MACRO)
//	#define DEFINE_SEC_SYSCALL_STUB_REF(svc_name) extern void_func __sec_syscall_##svc_name##_stub(void);
//#endif //SEC_SYSCALL_DEFINE_STUBS_REFERENCE_MACRO

#if defined(SEC_SYSCALL_DEFINE_STUBS_REFERENCE)

#if defined(SEC_SYSCALL_OABI_COMPAT)

	#define DEFINE_SEC_SYSCALL_STUB_REF(svc_name) extern void __sec_syscall_oabi_compat_##svc_name##_stub(void);

#elif !defined(SEC_SYSCALL_COMPAT)

	#define DEFINE_SEC_SYSCALL_STUB_REF(svc_name) extern void __sec_syscall_##svc_name##_stub(void);

#else //!SEC_SYSCALL_COMPAT

	#define DEFINE_SEC_SYSCALL_STUB_REF(svc_name) extern void __sec_syscall_compat_##svc_name##_stub(void);

#endif //SEC_SYSCALL_COMPAT


#define SEC_SYSCALL_SVC_OP(svc_name) DEFINE_SEC_SYSCALL_STUB_REF(svc_name)
#include "ssc_in.h"
#undef SEC_SYSCALL_SVC_OP

#endif //SEC_SYSCALL_DEFINE_STUBS_REFERENCE

#endif //!__ASSEMBLY__

////////////////////////////////////////////////

#if defined(SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO)

#if defined(__ASSEMBLY__)

#if defined(SEC_SYSCALL_OABI_COMPAT)

.macro __DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT svc_name:req
#if defined(SEC_SYSCALL_DEFINE_NEW_ENTRY)
	SEC_SYSCALL_DEFINE_NEW_ENTRY(__sec_syscall_oabi_compat_\svc_name\()_stub)
#else
	#if defined(CONFIG_ARM64)
		.quad __sec_syscall_oabi_compat_\svc_name\()_stub
	#else
		.long __sec_syscall_oabi_compat_\svc_name\()_stub
	#endif
#endif
.endm

#elif !defined(SEC_SYSCALL_COMPAT)

.macro __DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT svc_name:req
#if defined(SEC_SYSCALL_DEFINE_NEW_ENTRY)
	SEC_SYSCALL_DEFINE_NEW_ENTRY(__sec_syscall_\svc_name\()_stub)
#else
	#if defined(CONFIG_ARM64)
		.quad __sec_syscall_\svc_name\()_stub
	#else
		.long __sec_syscall_\svc_name\()_stub
	#endif
#endif
.endm

#else //!SEC_SYSCALL_COMPAT

.macro __DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT svc_name:req
#if defined(SEC_SYSCALL_DEFINE_NEW_ENTRY)
	SEC_SYSCALL_DEFINE_NEW_ENTRY(__sec_syscall_compat_\svc_name\()_stub)
#else
	#if defined(CONFIG_ARM64)
		.quad __sec_syscall_compat_\svc_name\()_stub
	#else
		.long __sec_syscall_compat_\svc_name\()_stub
	#endif
#endif
.endm

#endif //SEC_SYSCALL_COMPAT

#define DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT(svc_name) __DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT svc_name

#else

#if defined(SEC_SYSCALL_OABI_COMPAT)

	#define DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT(svc_name) (void_func)__sec_syscall_oabi_compat_##svc_name##_stub,

#elif !defined(SEC_SYSCALL_COMPAT)

	#define DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT(svc_name) (void_func)__sec_syscall_##svc_name##_stub,

#else //!SEC_SYSCALL_COMPAT

	#define DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT(svc_name) (void_func)__sec_syscall_compat_##svc_name##_stub,

#endif //SEC_SYSCALL_COMPAT

#endif //!__ASSEMBLY__

#endif //SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO

#if defined(SEC_SYSCALL_DEFINE_STUBS_ARRAY)

#define SEC_SYSCALL_SVC_OP(svc_name) DEFINE_SEC_SYSCALL_STUB_ARRAY_ELEMENT(svc_name)
#include "ssc_in.h"
#undef SEC_SYSCALL_SVC_OP

#endif //SEC_SYSCALL_DEFINE_STUBS_ARRAY

///////////////////////////
#if defined(SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO)

#if !defined(__NR_Base_Number)
	#define __NR_Base_Number (0)
#endif

#if defined(__ASSEMBLY__)

.macro __DEFINE_SEC_SYSCALL_INDEX_ARRAY_ELEMENT svc_name:req
	.word (__NR_\svc_name - (__NR_Base_Number))
.endm

#define DEFINE_SEC_SYSCALL_INDEX_ARRAY_ELEMENT(svc_name) __DEFINE_SEC_SYSCALL_INDEX_ARRAY_ELEMENT svc_name

#else

#define DEFINE_SEC_SYSCALL_INDEX_ARRAY_ELEMENT(svc_name) (__NR_##svc_name - (__NR_Base_Number)),

#endif

#endif //SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO

#if defined(SEC_SYSCALL_DEFINE_INDEX_ARRAY)

#define SEC_SYSCALL_SVC_OP(svc_name) DEFINE_SEC_SYSCALL_INDEX_ARRAY_ELEMENT(svc_name)
#include "ssc_in.h"
#undef SEC_SYSCALL_SVC_OP

#endif //SEC_SYSCALL_DEFINE_INDEX_ARRAY

