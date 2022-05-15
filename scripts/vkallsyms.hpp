#ifndef __VIVO_VKALLSYMS_HPP
#define __VIVO_VKALLSYMS_HPP

/* should be included in scripts\kallsyms.c */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>


#define CONFIG_ROOT_RESTRICT

#if defined(CONFIG_ROOT_RESTRICT)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

typedef struct tag_rs_symbol {
	const char *name;
	size_t len;
} rs_symbol;

static int compare_rs_symbol(const void *A, const void *B)
{
	return strcmp(((rs_symbol *)A)->name, ((rs_symbol *)B)->name);
}

static char *strnstr(const char *s1, const char *s2, size_t len)
{
	size_t l2;

	l2 = strlen(s2);
	if (!l2)
		return (char *)s1;
	while (len >= l2) {
		len--;
		if (!memcmp(s1, s2, l2))
			return (char *)s1;
		s1++;
	}
	return NULL;
}

static rs_symbol root_restrict_special_symbols[] = {
	{"____call_usermodehelper", (sizeof("____call_usermodehelper") - 1)},
	{"__aarch64_insn_hotpatch_safe", (sizeof("__aarch64_insn_hotpatch_safe") - 1)},
	{"__aarch64_insn_write", (sizeof("__aarch64_insn_write") - 1)},
	{"__apply_alternatives", (sizeof("__apply_alternatives") - 1)},
	{"__arch_jump_label_transform", (sizeof("__arch_jump_label_transform") - 1)},
	{"__blkdev_check", (sizeof("__blkdev_check") - 1)},
	{"__blkdev_get", (sizeof("__blkdev_get") - 1)},
	{"__call_usermodehelper", (sizeof("__call_usermodehelper") - 1)},
	{"__change_memory", (sizeof("__change_memory") - 1)},
	{"__ftrace_modify_caller", (sizeof("__ftrace_modify_caller") - 1)},
	{"__ftrace_modify_code", (sizeof("__ftrace_modify_code") - 1)},
	{"__patch_text", (sizeof("__patch_text") - 1)},
	{"__patch_text_real", (sizeof("__patch_text_real") - 1)},

	{"__set_fixmap", (sizeof("__set_fixmap") - 1)},

	{"SyS_chroot", (sizeof("SyS_chroot") - 1)},
	{"SyS_close", (sizeof("SyS_close") - 1)},
	{"SyS_execve", (sizeof("SyS_execve") - 1)},
	{"SyS_execveat", (sizeof("SyS_execveat") - 1)},
	{"SyS_fcntl", (sizeof("SyS_fcntl") - 1)},
	{"SyS_finit_module", (sizeof("SyS_finit_module") - 1)},
	{"SyS_getegid", (sizeof("SyS_getegid") - 1)},
	{"SyS_geteuid", (sizeof("SyS_geteuid") - 1)},
	{"SyS_getgid", (sizeof("SyS_getgid") - 1)},
	{"SyS_getresgid", (sizeof("SyS_getreguid") - 1)},
	{"SyS_getresuid", (sizeof("SyS_getresuid") - 1)},
	{"SyS_getuid", (sizeof("SyS_getuid") - 1)},

	{"SyS_finit_module", (sizeof("SyS_finit_module") - 1)},
	{"SyS_init_module", (sizeof("SyS_init_module") - 1)},
	{"SyS_mount", (sizeof("SyS_mount") - 1)},
	{"SyS_msync", (sizeof("SyS_msync") - 1)},
	{"SyS_ptrace", (sizeof("SyS_ptrace") - 1)},
	{"SyS_setgid", (sizeof("SyS_setgid") - 1)},
	{"SyS_setregid", (sizeof("SyS_setregid") - 1)},
	{"SyS_setresgid", (sizeof("SyS_setresgid") - 1)},
	{"SyS_setresuid", (sizeof("SyS_setresuid") - 1)},
	{"SyS_setreuid", (sizeof("SyS_setreuid") - 1)},
	{"SyS_setuid", (sizeof("SyS_setuid") - 1)},

	{"aarch64_insn_hotpatch_safe", (sizeof("aarch64_insn_hotpatch_safe") - 1)},
	{"aarch64_insn_patch_text", (sizeof("aarch64_insn_patch_text") - 1)},
	{"aarch64_insn_patch_text_cb", (sizeof("aarch64_insn_patch_text_cb") - 1)},
	{"aarch64_insn_patch_text_nosync", (sizeof("aarch64_insn_patch_text_nosync") - 1)},
	{"aarch64_insn_patch_text_sync", (sizeof("aarch64_insn_patch_text_sync") - 1)},
	{"aarch64_insn_read", (sizeof("aarch64_insn_read") - 1)},
	{"aarch64_insn_write", (sizeof("aarch64_insn_write") - 1)},
	{"aee_rr_reboot_reason_get", (sizeof("aee_rr_reboot_reason_get") - 1)},
	{"aee_rr_reboot_reason_set", (sizeof("aee_rr_reboot_reason_set") - 1)},

	{"apply_alternatives", (sizeof("apply_alternatives") - 1)},
	{"apply_to_page_range", (sizeof("apply_to_page_range") - 1)},
	{"arch_jump_label_transform", (sizeof("arch_jump_label_transform") - 1)},
	{"arch_jump_label_transform_static", (sizeof("arch_jump_label_transform_static") - 1)},
	{"avc_denied", (sizeof("avc_denied") - 1)},

	{"blkdev_get", (sizeof("blkdev_get") - 1)},
	{"blkdev_get_by_dev", (sizeof("blkdev_get_by_dev") - 1)},
	{"blkdev_get_by_path", (sizeof("blkdev_get_by_path") - 1)},
	{"blkdev_open", (sizeof("blkdev_open") - 1)},

	{"cad_pid", (sizeof("cad_pid") - 1)},
	{"call_helper", (sizeof("call_helper") - 1)},
	{"call_usermodehelper", (sizeof("call_usermodehelper") - 1)},
	{"call_usermodehelper_exec", (sizeof("call_usermodehelper_exec") - 1)},
	{"call_usermodehelper_setup", (sizeof("call_usermodehelper_setup") - 1)},

	{"change_memory_clear_bit", (sizeof("change_memory_clear_bit") - 1)},
	{"change_memory_common", (sizeof("change_memory_common") - 1)},
	{"change_memory_set_bit", (sizeof("change_memory_set_bit") - 1)},
	{"clear_page_range", (sizeof("clear_page_range") - 1)},

	{"commit_creds", (sizeof("commit_creds") - 1)},
	{"compat_SyS_bind", (sizeof("compat_SyS_bind") - 1)},
	{"compat_SyS_chroot", (sizeof("compat_SyS_chroot") - 1)},
	{"compat_SyS_connect", (sizeof("compat_SyS_connect") - 1)},
	{"compat_SyS_execve", (sizeof("compat_SyS_execve") - 1)},
	{"compat_SyS_execveat", (sizeof("compat_SyS_execveat") - 1)},
	{"compat_SyS_finit_module", (sizeof("compat_SyS_finit_module") - 1)},
	{"compat_SyS_init_module", (sizeof("compat_SyS_init_module") - 1)},
	{"compat_SyS_mount", (sizeof("compat_SyS_mount") - 1)},
	{"compat_SyS_ptrace", (sizeof("compat_SyS_ptrace") - 1)},

	{"compat_do_execve", (sizeof("compat_do_execve") - 1)},
	{"compat_do_execveat", (sizeof("compat_do_execveat") - 1)},
	{"compat_sys_call_table", (sizeof("compat_sys_call_table") - 1)},
	{"compat_sys_execve", (sizeof("compat_sys_execve") - 1)},
	{"compat_sys_execveat", (sizeof("compat_sys_execveat") - 1)},
	{"compat_sys_mount", (sizeof("compat_sys_mount") - 1)},
	{"compute_sid_handle_invalid_context", (sizeof("compute_sid_handle_invalid_context") - 1)},
	{"create_worker", (sizeof("create_worker") - 1)},

	{"current_mapping", (sizeof("current_mapping") - 1)},
	{"current_mapping_size", (sizeof("current_mapping_size") - 1)},
	{"current_is_single_threaded", (sizeof("current_is_single_threaded") - 1)},

	{"devlist", (sizeof("devlist") - 1)},
	{"do_access_dev_check", (sizeof("do_access_dev_check") - 1)},
	{"do_chroot_check", (sizeof("do_chroot_check") - 1)},
	{"do_disable_selinux_check", (sizeof("do_disable_selinux_check") - 1)},

	{"do_execve", (sizeof("do_execve") - 1)},
	{"do_execve_common", (sizeof("do_execve_common") - 1)},
	{"do_execve_common_check", (sizeof("do_execve_common_check") - 1)},
	{"do_execve_common_file_check", (sizeof("do_execve_common_file_check") - 1)},
	{"do_execveat", (sizeof("do_execveat") - 1)},
	{"do_execveat_common", (sizeof("do_execveat_common") - 1)},

	{"do_insmod_check", (sizeof("do_insmod_check") - 1)},
	{"do_load_selinux_policy_check", (sizeof("do_load_selinux_policy_check") - 1)},
	{"do_mount", (sizeof("do_mount") - 1)},
	{"do_mount_files_sb_check", (sizeof("do_mount_files_sb_check") - 1)},
	{"do_ptrace_check", (sizeof("do_ptrace_check") - 1)},
	{"do_sock_bind_check", (sizeof("do_sock_bind_check") - 1)},
	{"do_sock_connect_check", (sizeof("do_sock_connect_check") - 1)},
	{"do_uid_check", (sizeof("do_uid_check") - 1)},

	{"dumchar_fops", (sizeof("dumchar_fops") - 1)},
	{"dumchar_ioctl", (sizeof("dumchar_ioctl") - 1)},
	{"dumchar_open", (sizeof("dumchar_open") - 1)},
	{"dumchar_probe", (sizeof("dumchar_probe") - 1)},

	{"ebitmap_set_bit", (sizeof("ebitmap_set_bit") - 1)},
	{"el0_svc_common", (sizeof("el0_svc_common") - 1)},
	{"el0_svc_compat", (sizeof("el0_svc_compat") - 1)},
	{"el0_svc_compat_handler", (sizeof("el0_svc_compat_handler") - 1)},
	{"el0_svc_handler", (sizeof("el0_svc_handler") - 1)},
	{"el0_sync", (sizeof("el0_sync") - 1)},
	{"el0_sync_compat", (sizeof("el0_sync_compat") - 1)},

	{"ftrace_arch_code_modify_post_process", (sizeof("ftrace_arch_code_modify_post_process") - 1)},
	{"ftrace_arch_code_modify_prepare", (sizeof("ftrace_arch_code_modify_prepare") - 1)},
	{"ftrace_disable_ftrace_graph_caller", (sizeof("ftrace_disable_ftrace_graph_caller") - 1)},
	{"ftrace_enable_ftrace_graph_caller", (sizeof("ftrace_enable_ftrace_graph_caller") - 1)},
	{"ftrace_make_call", (sizeof("ftrace_make_call") - 1)},
	{"ftrace_make_nop", (sizeof("ftrace_make_nop") - 1)},
	{"ftrace_modify_code", (sizeof("ftrace_modify_code") - 1)},
	{"ftrace_modify_graph_caller", (sizeof("ftrace_modify_graph_caller") - 1)},
	{"ftrace_update_ftrace_func", (sizeof("ftrace_update_ftrace_func") - 1)},

	{"g_boot_mode", (sizeof("g_boot_mode") - 1)},
	{"g_boot_reason", (sizeof("g_boot_reason") - 1)},
	{"generic_prepare", (sizeof("generic_prepare") - 1)},

	{"get_mm_exe_file", (sizeof("get_mm_exe_file") - 1)},
	{"get_task_reap_init_flag", (sizeof("get_task_reap_init_flag") - 1)},

	{"idmap_pg_dir", (sizeof("idmap_pg_dir") - 1)},

	{"in_recovery_mode", (sizeof("in_recovery_mode") - 1)},
	{"in_survival_mode", (sizeof("in_survival_mode") - 1)},
	{"init_cred", (sizeof("init_cred") - 1)},
	{"init_groups", (sizeof("init_groups") - 1)},
	{"init_mm", (sizeof("init_mm") - 1)},

	{"init_pg_dir", (sizeof("init_pg_dir") - 1)},
	{"init_pg_end", (sizeof("init_pg_end") - 1)},
	{"init_sel_fs", (sizeof("init_sel_fs") - 1)},
	{"init_task", (sizeof("init_task") - 1)},
	{"init_user_ns", (sizeof("init_user_ns") - 1)},
	{"io_remap_pfn_range", (sizeof("io_remap_pfn_range") - 1)},
	{"is_dbg_task", (sizeof("is_dbg_task") - 1)},
	{"is_root_dir", (sizeof("is_root_dir") - 1)},
	{"is_root_ent", (sizeof("is_root_ent") - 1)},
	{"is_root_proc_open", (sizeof("is_root_proc_open") - 1)},
	{"is_root_proc_show", (sizeof("is_root_proc_show") - 1)},
	{"is_root_proc_write", (sizeof("is_root_proc_write") - 1)},
	{"isMountFlagEnabled", (sizeof("isMountFlagEnabled") - 1)},
	{"isMountRecord", (sizeof("isMountRecord") - 1)},

	{"isrecoverymode", (sizeof("isrecoverymode") - 1)},
	{"isrecoverymode_detection", (sizeof("isrecoverymode_detection") - 1)},
	{"issurvivalmode", (sizeof("issurvivalmode") - 1)},
	{"issurvivalmode_detection", (sizeof("issurvivalmode_detection") - 1)},

	{"kernel_init", (sizeof("kernel_init") - 1)},
	{"kernel_sock_ioctl", (sizeof("kernel_sock_ioctl") - 1)},
	{"kmem_fops", (sizeof("kmem_fops") - 1)},
	{"kptr_restrict", (sizeof("kptr_restrict") - 1)},

	{"latest_granting", (sizeof("latest_granting") - 1)},
	{"mark_rodata_ro", (sizeof("mark_rodata_ro") - 1)},
	{"mem_fops", (sizeof("mem_fops") - 1)},
	{"mem_text_address_restore", (sizeof("mem_text_address_restore") - 1)},
	{"mem_text_address_writeable", (sizeof("mem_text_address_writeable") - 1)},
	{"mem_text_write_kernel_word", (sizeof("mem_text_write_kernel_word") - 1)},
	{"mem_text_writeable_spinlock", (sizeof("mem_text_writeable_spinlock") - 1)},
	{"mem_text_writeable_spinunlock", (sizeof("mem_text_writeable_spinunlock") - 1)},

	{"mmap_kmem", (sizeof("mmap_kmem") - 1)},
	{"mmap_mem", (sizeof("mmap_mem") - 1)},
	{"mount_read_proc", (sizeof("mount_read_proc") - 1)},
	{"mount_write_proc", (sizeof("mount_write_proc") - 1)},
	{"mountFlag", (sizeof("mountFlag") - 1)},

	{"open_port", (sizeof("open_port") - 1)},
	{"override_creds", (sizeof("override_creds") - 1)},
	{"patch_lock", (sizeof("patch_lock") - 1)},
	{"patch_map", (sizeof("patch_map") - 1)},
	{"patch_text", (sizeof("patch_text") - 1)},
	{"patch_text_stop_machine", (sizeof("patch_text_stop_machine") - 1)},
	{"patch_unmap", (sizeof("patch_unmap") - 1)},

	{"policydb", (sizeof("policydb") - 1)},
	{"port_fops", (sizeof("port_fops") - 1)},
	{"prepare_ftrace_return", (sizeof("prepare_ftrace_return") - 1)},
	{"prepare_kernel_cred", (sizeof("prepare_kernel_cred") - 1)},
	{"ptmx_fops", (sizeof("ptmx_fops") - 1)},
	{"ptmx_open", (sizeof("ptmx_open") - 1)},

	{"ptrace_attach", (sizeof("ptrace_attach") - 1)},

	{"read_kmem", (sizeof("read_kmem") - 1)},
	{"read_mem", (sizeof("read_mem") - 1)},
	{"revert_creds", (sizeof("revert_creds") - 1)},
	{"remap_pfn_range", (sizeof("remap_pfn_range") - 1)},
	{"remount_detection", (sizeof("remount_detection") - 1)},
	{"reset_security_ops", (sizeof("reset_security_ops") - 1)},
	{"ring_buffer_set_clock", (sizeof("ring_buffer_set_clock") - 1)},
	{"root_user", (sizeof("root_user") - 1)},
	{"rs_can_exec", (sizeof("rs_can_exec") - 1)},
	{"rs_get_boot_mode", (sizeof("rs_get_boot_mode") - 1)},
	{"rs_um", (sizeof("rs_um") - 1)},
	{"run_init_process", (sizeof("run_init_process") - 1)},

	{"security_context_to_sid", (sizeof("security_context_to_sid") - 1)},
	{"security_context_to_sid_core", (sizeof("security_context_to_sid_core") - 1)},
	{"security_context_to_sid_force", (sizeof("security_context_to_sid_force") - 1)},
	{"security_sid_to_context", (sizeof("security_sid_to_context") - 1)},
	{"security_sid_to_context_core", (sizeof("security_sid_to_context_core") - 1)},
	{"security_sid_to_context_force", (sizeof("security_sid_to_context_force") - 1)},
	{"sel_enforce_ops", (sizeof("sel_enforce_ops") - 1)},
	{"sel_load_ops", (sizeof("sel_load_ops") - 1)},
	{"sel_fill_super", (sizeof("sel_fill_super") - 1)},
	{"sel_fs_type", (sizeof("sel_fs_type") - 1)},
	{"sel_load_ops", (sizeof("sel_load_ops") - 1)},
	{"sel_mount", (sizeof("sel_mount") - 1)},

	{"sel_read_enforce", (sizeof("sel_read_enforce") - 1)},
	{"sel_write_em_enforce", (sizeof("sel_write_em_enforce") - 1)},
	{"sel_write_enforce", (sizeof("sel_write_enforce") - 1)},
	{"sel_write_load", (sizeof("sel_write_load") - 1)},

	{"selinux_enabled", (sizeof("selinux_enabled") - 1)},
	{"selinux_enforcing", (sizeof("selinux_enforcing") - 1)},
	{"selinux_files", (sizeof("selinux_files") - 1)},
	{"selinux_secctx_to_secid", (sizeof("selinux_secctx_to_secid") - 1)},
	{"selinux_secid_to_secctx", (sizeof("selinux_secid_to_secctx") - 1)},
	{"selinux_status_update_setenforce", (sizeof("selinux_status_update_setenforce") - 1)},
	{"selnl_notify_setenforce", (sizeof("selnl_notify_setenforce") - 1)},
	{"set_all_modules_text_ro", (sizeof("set_all_modules_text_ro") - 1)},
	{"set_all_modules_text_rw", (sizeof("set_all_modules_text_rw") - 1)},
	{"set_kernel_text_ro", (sizeof("set_kernel_text_ro") - 1)},
	{"set_kernel_text_rw", (sizeof("set_kernel_text_rw") - 1)},
	{"set_memory_nx", (sizeof("set_memory_nx") - 1)},
	{"set_memory_ro", (sizeof("set_memory_ro") - 1)},
	{"set_memory_rw", (sizeof("set_memory_rw") - 1)},
	{"set_memory_x", (sizeof("set_memory_x") - 1)},
	{"set_page_attributes", (sizeof("set_page_attributes") - 1)},
	{"set_page_range", (sizeof("set_page_range") - 1)},
	{"set_rs_s_u", (sizeof("set_rs_s_u") - 1)},
	{"sidtab", (sizeof("sidtab") - 1)},
	{"ss_initialized", (sizeof("ss_initialized") - 1)},
	{"ssc_get_boot_mode", (sizeof("ssc_get_boot_mode") - 1)},

	{"swapper_pg_dir", (sizeof("swapper_pg_dir") - 1)},
	{"swapper_pg_end", (sizeof("swapper_pg_end") - 1)},
	{"sys_bind", (sizeof("sys_bind") - 1)},
	{"sys_call_table", (sizeof("sys_call_table") - 1)},
	{"sys_chroot", (sizeof("sys_chroot") - 1)},

	{"sys_close", (sizeof("sys_close") - 1)},
	{"sys_connect", (sizeof("sys_connect") - 1)},
	{"sys_execve", (sizeof("sys_execve") - 1)},
	{"sys_execveat", (sizeof("sys_execveat") - 1)},
	{"sys_fcntl", (sizeof("sys_fcntl") - 1)},
	{"sys_finit_module", (sizeof("sys_finit_module") - 1)},
	{"sys_getegid", (sizeof("sys_getegid") - 1)},
	{"sys_geteuid", (sizeof("sys_geteuid") - 1)},
	{"sys_getgid", (sizeof("sys_getgid") - 1)},
	{"sys_getresgid", (sizeof("sys_getreguid") - 1)},
	{"sys_getresuid", (sizeof("sys_getresuid") - 1)},
	{"sys_getuid", (sizeof("sys_getuid") - 1)},

	{"sys_init_module", (sizeof("sys_init_module") - 1)},
	{"sys_mount", (sizeof("sys_mount") - 1)},
	{"sys_msync", (sizeof("sys_msync") - 1)},
	{"sys_oabi_call_table", (sizeof("sys_oabi_call_table") - 1)},
	{"sys_ptrace", (sizeof("sys_ptrace") - 1)},

	{"sys_setgid", (sizeof("sys_setgid") - 1)},
	{"sys_setregid", (sizeof("sys_setregid") - 1)},
	{"sys_setresgid", (sizeof("sys_setresgid") - 1)},
	{"sys_setresuid", (sizeof("sys_setsreuid") - 1)},
	{"sys_setreuid", (sizeof("sys_setreuid") - 1)},
	{"sys_setuid", (sizeof("sys_setuid") - 1)},
	{"sys_fcntl", (sizeof("sys_fcntl") - 1)},
	{"sysfs_kf_bin_mmap", (sizeof("sysfs_kf_bin_mmap") - 1)},
	{"system_state", (sizeof("system_state") - 1)},

	{"tramp_pg_dir", (sizeof("tramp_pg_dir") - 1)},
	{"try_to_run_init_process", (sizeof("try_to_run_init_process") - 1)},
	{"tty_init_dev", (sizeof("tty_init_dev") - 1)},
	{"tty_release", (sizeof("tty_release") - 1)},

	{"vfsmount_lock", (sizeof("vfsmount_lock") - 1)},
	{"vmalloc_exec", (sizeof("vmalloc_exec") - 1)},
	{"vr_context_to_sid", (sizeof("vr_context_to_sid") - 1)},
	{"vr_is_exec", (sizeof("vr_is_exec") - 1)},
	{"vr_try_update_policy", (sizeof("vr_try_update_policy") - 1)},
	{"wait_for_helper", (sizeof("wait_for_helper") - 1)},
	{"write_kmem", (sizeof("write_kmem") - 1)},
	{"write_mem", (sizeof("write_mem") - 1)},

	{NULL, 0}, /* tail marker */
	};

static inline int vrr_symbol_valid(char *sym_name, unsigned long long sym_addr)
{
	static int root_restrict_special_symbols_sorted;
	static unsigned long long exclude_vectors_sym_start;
	static unsigned long long exclude_vectors_sym_end;

	int i;
	size_t sym_name_len;

	if (!exclude_vectors_sym_start) {
		if (strcmp(sym_name, "ret_fast_syscall"/*"init_linuxrc"*/) == 0) {
			exclude_vectors_sym_start = sym_addr;
			return 0;
		}
	}

	if (!exclude_vectors_sym_end) {
		if (strcmp(sym_name, "trace_do_force_stop") == 0) {
			exclude_vectors_sym_end = sym_addr;
			return 0;
		} else if (strcmp(sym_name, "__entry_text_end") == 0) {
			exclude_vectors_sym_end = sym_addr;
			return 0;
		} else if (strcmp(sym_name, "sys_rt_sigreturn_wrapper") == 0) {
			exclude_vectors_sym_end = sym_addr;
			return 0;
		} else if (strcmp(sym_name, "handle_IRQ") == 0) {
			exclude_vectors_sym_end = sym_addr;
			return 0;
		} else if ((exclude_vectors_sym_start) && (sym_addr > exclude_vectors_sym_start)) {
			if (sym_addr < (exclude_vectors_sym_start + 1024))
				return 0;
			else {
				exclude_vectors_sym_end = sym_addr;
				sym_addr = 0;
			}
		}
	}

	if ((sym_addr > exclude_vectors_sym_start) && (sym_addr <= exclude_vectors_sym_end)) {
		return 0;
	}

	sym_name_len = strlen(sym_name);
	/* Exclude symbols which root restrict require to hide. */
	if (strnstr(sym_name, "_v_r_s_h_s", sym_name_len))
		return 0;

	if (strnstr(sym_name, "aarch64", sym_name_len))
		return 0;

	if (strnstr(sym_name, "kprobe", sym_name_len))
		return 0;

	if (strnstr(sym_name, "kdbg", sym_name_len))
		return 0;

	if (strnstr(sym_name, "ftrace", sym_name_len))
		return 0;

	if (strnstr(sym_name, "__sec_syscall", sym_name_len))
		return 0;

	if (strncmp(sym_name, "__ksymtab_", (sizeof("__ksymtab_") - 1)) == 0)
		return 0;

	if (strncmp(sym_name, "__kcrctab_", (sizeof("__kcrctab_") - 1)) == 0)
		return 0;

	if (strncmp(sym_name, "__kstrtab_", (sizeof("__kstrtab_") - 1)) == 0)
		return 0;

	if (strncmp(sym_name, "__arm64_", (sizeof("__arm64_") - 1)) == 0)
		return 0;

	if (!root_restrict_special_symbols_sorted) {
		qsort(root_restrict_special_symbols, ARRAY_SIZE(root_restrict_special_symbols) - 1, sizeof(root_restrict_special_symbols[0]),
			compare_rs_symbol);

		root_restrict_special_symbols_sorted = 1;
	}
	/* root_restrict_special_symbols[]是按字符串从小到大顺序排的 */
	{
		char *S = sym_name;
		int L, H, I, C;

		L = 0;
		H = ARRAY_SIZE(root_restrict_special_symbols) - 1 - 1; /*excluding last NULL element in array*/
		while (L <= H) {
			I = (L + H) >> 1;
			C = strncmp(root_restrict_special_symbols[I].name, S, root_restrict_special_symbols[I].len);
			if (C < 0) {
				L = I + 1;
			} else if (C > 0) {
				H = I - 1;
			} else {
				return 0;
			}
		}
	}

	return 1;
}

	#define	VIVO_CHECK_SYMBOL_VALID(sym_name, sym_addr)		vrr_symbol_valid((sym_name), (sym_addr))

#else /* CONFIG_ROOT_RESTRICT */

	#define	VIVO_CHECK_SYMBOL_VALID(sym_name, sym_addr)		(1)

#endif /* !CONFIG_ROOT_RESTRICT */

#endif /* __VIVO_VKALLSYMS_HPP */
