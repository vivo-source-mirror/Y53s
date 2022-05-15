/*
 * Low-level implementation code for security compatible syscalls hooking
 *
 */

#if defined(CONFIG_COMPAT)

#include <linux/init.h>
#include <linux/linkage.h>

#if defined(CONFIG_ARM64)
//#include <asm/alternative-asm.h>
#else
#include <asm/unified.h>
#include "entry-header.S"
#endif

#include <asm/assembler.h>
#include <asm/asm-offsets.h>

//#include <uapi/asm/unistd.h>
//#include <linux/unistd.h>

#if defined(CONFIG_ARM64)
#include <asm/unistd32.h>
#else
#include <asm/unistd.h>
#endif

#include <linux/version.h>

#define SEC_SYSCALL_COMPAT

#define SEC_SYSCALL_INCLUDE_HEADER_FILES
#define SEC_SYSCALL_DEFINE_ENUMERATION

#include "ssc.hpp"

#undef SEC_SYSCALL_INCLUDE_HEADER_FILES
#undef SEC_SYSCALL_DEFINE_ENUMERATION


.pushsection	SEC_SYSCALL_OLD_ENTREIS_SECTION
#if !defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
.global __sec_syscall_compat_old_entries
#endif
SEC_SYSCALL_DEFAULT_ALIGN
.type	__sec_syscall_compat_old_entries, #object
__sec_syscall_compat_old_entries:
.rept nr_invalid
SEC_SYSCALL_DEFINE_OLD_ENTRY(0)
.endr
.popsection


#if defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
.global __sec_syscall_compat_old_entries_ptr
#endif
SEC_SYSCALL_DEFAULT_ALIGN
.type	__sec_syscall_compat_old_entries_ptr, #object
__sec_syscall_compat_old_entries_ptr:
SEC_SYSCALL_DEFINE_PTR(__sec_syscall_compat_old_entries)

__sec_syscall_compat_common_stub:
#if defined(CONFIG_ARM64)
#if defined(SEC_SYSCALL_DO_PRE_CHECK)
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
#if defined(SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX)
	stp	x0, x30, [sp, #-0x10]!
#if defined(CONFIG_KASAN) || defined(CONFIG_KCOV)
	stp	REG_SYSCALL_IDX, x8, [sp, #-0x10]!
#endif
#else
	stp	x29, x30, [sp, #-0x10]!
	stp	x0, x8, [sp, #-0x10]!
#endif
#if defined(CONFIG_KASAN) || defined(CONFIG_KCOV)
	get_thread_info	x0
#else
	//x28 should not changed by el0_svc_handler/el0_svc_common/invoke_syscall
	mov	x0, x28	//thread_info -> x0 for __sec_syscall_pre_check()
#endif
#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
	mov	x1, x26	//scno -> x1 for __sec_syscall_pre_check()
#endif
	bl	__sec_syscall_pre_check	// call __sec_syscall_pre_check()
	cbnz	x0, 2f //direct return if result is not zero
#if defined(SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX)
#if defined(CONFIG_KASAN) || defined(CONFIG_KCOV)
	ldp	REG_SYSCALL_IDX, x8, [sp], #0x10
#endif
	ldp	x0, x30, [sp], #0x10
#else
	ldp	x0, x8, [sp], #0x10
	ldp	x29, x30, [sp], #0x10
#endif
#else //SSC_ARCH_USE_SYSCALL_WRAPPER
	mov	x0, x28	//thread_info -> x0 for __sec_syscall_pre_check()
#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
	mov	x1, x26	//scno -> x1 for __sec_syscall_pre_check()
#endif
	mov	x19, x30	//LR -> x19
	bl	__sec_syscall_pre_check	// call __sec_syscall_pre_check()
	mov	x30, x19	//x19 -> LR
	cbnz	x0, 1f //direct return if result is not zero
#endif //!SSC_ARCH_USE_SYSCALL_WRAPPER
#endif
	//adr	x2, __sec_syscall_compat_old_entries	//__sec_syscall_compat_old_entries -> x2
	ldr	x2, __sec_syscall_compat_old_entries_ptr
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
	ldr	x3, [x2, REG_SYSCALL_IDX, lsl #3]	//__sec_syscall_compat_old_entries[x20] -> x3
#else //SSC_ARCH_USE_SYSCALL_WRAPPER
	ldr	x16, [x2, REG_SYSCALL_IDX, lsl #3]	//__sec_syscall_compat_old_entries[x20] -> x16
	ldp	x0, x1, [sp]	//restore args
	ldp	x2, x3, [sp, #S_X2]
#if defined(SEC_SYSCALL_DO_CHECK_USE_4_MORE_REGISTERS)
	ldp	x4, x5, [sp, #S_X4]
	ldp	x6, x7, [sp, #S_X6]
#endif
#endif //!SSC_ARCH_USE_SYSCALL_WRAPPER

#if defined(SEC_SYSCALL_DO_POST_CHECK)
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
	stp	x0, x30, [sp, #-0x10]!
	adr	x30, ret_from_entry	//LR already saved in x19
#else
#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
	mov	x19, x30	//LR -> x19
#endif
	adr	x30, ret_from_entry	//LR already saved in x19
#endif
#endif
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
	br	x3	//jump to saved entry
#else
	br	x16	//jump to saved entry
#endif
#if defined(SEC_SYSCALL_DO_POST_CHECK)
ret_from_entry:
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
	str	x0, [sp, #0] //save result
	mov	x0, x28	//thread_info -> x0
	bl	__sec_syscall_post_check	//may call force_sig()
	ldp	x0, x30, [sp], #0x10
#else
	mov	x20, x0	//save result
	mov	x0, x28	//thread_info -> x0
	bl	__sec_syscall_post_check	//may call force_sig()
	mov	x0, x20	//restore result
	mov	x30, x19	//x19 -> LR
#endif
#endif
#if defined(SEC_SYSCALL_DO_POST_CHECK) || !defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
1:
	ret
#endif
#if defined(SSC_ARCH_USE_SYSCALL_WRAPPER)
2:
#if defined(SSC_SYSCALL_WRAPPER_STBL_AS_REG_IDX)
#if defined(CONFIG_KASAN) || defined(CONFIG_KCOV)
	ldp	REG_SYSCALL_IDX, x8, [sp], #0x10
#endif
	ldp	x1, x30, [sp], #0x10 //keep x0
#else
	ldp	x1, x8, [sp], #0x10 //keep x0
	ldp	x29, x30, [sp], #0x10
#endif
	ret
#endif
#else //CONFIG_ARM64
#if defined(SEC_SYSCALL_DO_PRE_CHECK)
	mov	r0, tsk	//r9	//thread_info -> r0 for __sec_syscall_pre_check()
#if defined(SEC_SYSCALL_PRE_CHECK_WITH_CALL_NUMBER)
	mov	r1, scno	//r7	//scno -> r1 for __sec_syscall_pre_check()
#endif
	mov	r6, lr	//LR -> r6
	bl	__sec_syscall_pre_check	// call __sec_syscall_pre_check()
	mov	lr, r6	//r6 -> LR
	tst	r0, #0
	bne	1f	//direct return if result is not zero
#endif
	//adr	r11, __sec_syscall_compat_old_entries	//__sec_syscall_compat_old_entries -> r11
	ldr	r11, __sec_syscall_compat_old_entries_ptr
	add	r1, sp, #S_OFF	//restore args
	ldmia	r1, {r0 - r3}	//ldmia	r1, {r0 - r5}
#if defined(SEC_SYSCALL_DO_POST_CHECK)
#if !defined(SEC_SYSCALL_DO_PRE_CHECK)
	mov	r6, lr	//LR -> r6
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 1, 0)
	badr	lr, ret_from_entry	//LR already saved in r6
#else
#if defined(BSYM)
	adr	lr, BSYM(ret_from_entry)	//LR already saved in r6
#else
#if defined(CONFIG_THUMB2_KERNEL) //refer to the BSYM() macro
	adr	lr, ret_from_entry + 1	//LR already saved in r6
#else
	adr	lr, ret_from_entry	//LR already saved in r6
#endif
#endif
#endif
#endif
	ldr	pc, [r11, r10, lsl #2]	//__sec_syscall_compat_old_entries[r10] -> pc
#if defined(SEC_SYSCALL_DO_POST_CHECK)
ret_from_entry:
	mov	r10, r0	//save result
	mov	r0, tsk	//r9	//thread_info -> x0
	bl	__sec_syscall_post_check	//may call force_sig()
	mov	r0, r10	//restore result
	mov	lr, r6	//r6 -> LR
#endif
1:
#if defined(SEC_SYSCALL_ARM_THUMB_SUPPORT_RET_BY_REG)
	ret	lr
#else
	mov	pc, lr	//bx lr
#endif
#endif //!CONFIG_ARM64
ENDPROC(__sec_syscall_compat_common_stub)


#define SEC_SYSCALL_DEFINE_STUBS
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS

#if defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)

#define SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_ARRAY_MACRO


.pushsection	SEC_SYSCALL_NEW_ENTREIS_SECTION
#if !defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
.global __sec_syscall_compat_new_entries
#endif //!SEC_SYSCALL_NEW_ENTRIES_BY_PTR
SEC_SYSCALL_DEFAULT_ALIGN
.type	__sec_syscall_compat_new_entries, #object
__sec_syscall_compat_new_entries:

#define SEC_SYSCALL_DEFINE_STUBS_ARRAY
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_ARRAY

#if defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
.global __sec_syscall_compat_new_entries_ptr
SEC_SYSCALL_DEFAULT_ALIGN
.type	__sec_syscall_compat_new_entries_ptr, #object
__sec_syscall_compat_new_entries_ptr:
SEC_SYSCALL_DEFINE_PTR(__sec_syscall_compat_new_entries)
#endif //SEC_SYSCALL_NEW_ENTRIES_BY_PTR

.popsection

#endif //SEC_SYSCALL_STUBS_ARRAY_IN_ASM

#endif //CONFIG_COMPAT

