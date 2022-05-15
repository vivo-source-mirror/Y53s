
#if defined(CONFIG_OABI_COMPAT)

#include <linux/init.h>


#if !defined(CONFIG_ARM64)
#include <asm/unified.h>
#endif

#if defined(CONFIG_ARM64)
#include <asm/unistd32.h>
#else
#include <asm/unistd.h>
#endif

#define SEC_SYSCALL_INCLUDE_HEADER_FILES
#include "ssc.hpp"
#undef SEC_SYSCALL_INCLUDE_HEADER_FILES


#if !defined(__NR_syscalls) && defined(CONFIG_ARM)
	/*#define __NR_syscalls (381)*/ /*before kernel 3.5*/
	extern size_t __sec_syscall_NR_syscalls;
	#define __NR_syscalls __sec_syscall_NR_syscalls
#endif


#define SEC_SYSCALL_OABI_COMPAT

#define SEC_SYSCALL_DEFINE_ENUMERATION
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_ENUMERATION

#if defined(__NR_SYSCALL_BASE)
	#define __NR_Base_Number (__NR_SYSCALL_BASE)
#endif

#define SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_INDEX_ARRAY_MACRO

noused static const uint16_t __sec_syscall_oabi_compat_indexes[nr_invalid] __initconst = {
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

noused static const void_func __sec_syscall_oabi_compat_new_entries[nr_invalid] __initconst = {
#define SEC_SYSCALL_DEFINE_STUBS_ARRAY
#include "ssc.hpp"
#undef SEC_SYSCALL_DEFINE_STUBS_ARRAY
};
#endif /*!SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/

#if (!defined(SEC_SYSCALL_PRE_CHECK_ALL_CALL) \
	&& !(defined(SEC_SYSCALL_POST_CHECK_ALL_CALL) && !defined(SEC_SYSCALL_POST_CHECK_ALL_CALL_INLINE))) \
	&& (defined(SEC_SYSCALL_DO_PRE_CHECK) || defined(SEC_SYSCALL_DO_POST_CHECK))

noused notrace static int __init __sec_syscall_oabi_compat_init(void)
{
	noused size_t i;
	noused void_func *call_table;
	noused void_func *old_entries;
	noused void_func *new_entries;

	/*extern noused const unsigned __sec_syscall_NR_compat_syscalls;*/
	extern noused long sys_ni_syscall(void);
	/*sys_oabi_call_table[] has not been declared in any header file*/
	extern noused void_func sys_oabi_call_table; /*actutally not a pointer type*/
#if defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
	extern noused void_func *__sec_syscall_oabi_compat_old_entries_ptr;
#else
	extern noused void_func __sec_syscall_oabi_compat_old_entries;
#endif
#if defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
#if defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
	extern noused void_func *__sec_syscall_oabi_compat_new_entries_ptr;
#else
	extern noused void_func __sec_syscall_oabi_compat_new_entries;
#endif /*!SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
#endif /*SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/
	extern noused int __sec_syscall_init_entries(const uint16_t *indexes, size_t index_count,
		void_func *call_table, size_t call_table_size, void_func *old_entries,
		const void_func *new_entries);
	extern noused int printk(const char *fmt, ...);

	call_table = (void_func *)&sys_oabi_call_table;
#if defined(SEC_SYSCALL_OLD_ENTRIES_BY_PTR)
	old_entries = (void_func *)__sec_syscall_oabi_compat_old_entries_ptr;
#else
	old_entries = (void_func *)&__sec_syscall_oabi_compat_old_entries;
#endif
#if defined(SEC_SYSCALL_STUBS_ARRAY_IN_ASM)
#if defined(SEC_SYSCALL_NEW_ENTRIES_BY_PTR)
	new_entries = (void_func *)__sec_syscall_oabi_compat_new_entries_ptr;
#else
	new_entries = (void_func *)&__sec_syscall_oabi_compat_new_entries;
#endif /*!SEC_SYSCALL_NEW_ENTRIES_BY_PTR*/
#else
	new_entries = (void_func *)&__sec_syscall_oabi_compat_new_entries[0];
#endif /*!SEC_SYSCALL_STUBS_ARRAY_IN_ASM*/

#if 1
	__sec_syscall_init_entries(__sec_syscall_oabi_compat_indexes, nr_invalid,
		call_table, __NR_syscalls, old_entries, new_entries);
#else
	for (i = 0; i < nr_invalid; i++) {
		unsigned svc_no = __sec_syscall_oabi_compat_indexes[i];

		if (svc_no < __NR_syscalls) {
			void_func svc_entry = call_table[svc_no];
			if (svc_entry != (void_func)sys_ni_syscall) {
				old_entries[i] = svc_entry;
				call_table[svc_no] = new_entries[i];
				/*printk("xjy,cpt,%p,%p,%p\n", old_entries[i], svc_entry, call_table[svc_no]);*/
			}
		}
	}
#endif

	return 0;
}

SEC_SYSCALL_INITCALL(__sec_syscall_oabi_compat_init);
#endif /*(!SEC_SYSCALL_PRE_CHECK_ALL_CALL && !SEC_SYSCALL_POST_CHECK_ALL_CALL) && (SEC_SYSCALL_DO_PRE_CHECK || SEC_SYSCALL_DO_POST_CHECK)*/

#endif /*CONFIG_OABI_COMPAT*/
