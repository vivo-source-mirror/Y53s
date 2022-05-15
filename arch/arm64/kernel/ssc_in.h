
/*
 * please define syscalls that will copy data to user space here
 */

#if !defined(SEC_SYSCALL_SVC_OP)
	#define SEC_SYSCALL_SVC_OP(svc_name)
#endif

#if defined(SEC_SYSCALL_DO_MINIMAL_CHECK)


#if defined(__NR_mount)
	SEC_SYSCALL_SVC_OP(mount)
#endif
#if defined(__NR_fork)
	SEC_SYSCALL_SVC_OP(fork)
#endif
#if defined(__NR_vfork)
	SEC_SYSCALL_SVC_OP(vfork)
#endif
#if defined(__NR_clone)
	SEC_SYSCALL_SVC_OP(clone)
#endif
/*should be defined for SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL*/
#if defined(__NR_execve)
	SEC_SYSCALL_SVC_OP(execve)
#endif
#if defined(__NR_execveat)
	SEC_SYSCALL_SVC_OP(execveat)
#endif
#if defined(__NR_ptrace)
	SEC_SYSCALL_SVC_OP(ptrace)
#endif
#if defined(__NR_open)
	SEC_SYSCALL_SVC_OP(open)
#endif
#if defined(__NR_openat)
	SEC_SYSCALL_SVC_OP(openat)
#endif

#if !defined(CONFIG_MODULE_SIG_FORCE)
#if defined(__NR_init_module)
	SEC_SYSCALL_SVC_OP(init_module)
#endif
#if defined(__NR_finit_module)
	SEC_SYSCALL_SVC_OP(finit_module)
#endif
#endif

#if defined(__NR_setreuid)
	SEC_SYSCALL_SVC_OP(setreuid)
#endif
#if defined(__NR_setuid)
	SEC_SYSCALL_SVC_OP(setuid)
#endif
#if defined(__NR_setresuid)
	SEC_SYSCALL_SVC_OP(setresuid)
#endif

#if defined(__NR_lchown)
	SEC_SYSCALL_SVC_OP(lchown)
#endif
#if defined(__NR_fchown)
	SEC_SYSCALL_SVC_OP(fchown)
#endif
#if defined(__NR_chown)
	SEC_SYSCALL_SVC_OP(chown)
#endif

#if defined(__NR_lchown32)
	SEC_SYSCALL_SVC_OP(lchown32)
#endif
#if defined(__NR_fchown32)
	SEC_SYSCALL_SVC_OP(fchown32)
#endif
#if defined(__NR_chown32)
	SEC_SYSCALL_SVC_OP(chown32)
#endif
#if defined(__NR_fchownat)
	SEC_SYSCALL_SVC_OP(fchownat)
#endif

#if defined(__NR_chmod)
	SEC_SYSCALL_SVC_OP(chmod)
#endif
#if defined(__NR_fchmod)
	SEC_SYSCALL_SVC_OP(fchmod)
#endif
#if defined(__NR_fchmodat)
	SEC_SYSCALL_SVC_OP(fchmodat)
#endif

#if defined(__NR_setxattr)
	SEC_SYSCALL_SVC_OP(setxattr)
#endif
#if defined(__NR_lsetxattr)
	SEC_SYSCALL_SVC_OP(lsetxattr)
#endif
#if defined(__NR_fsetxattr)
	SEC_SYSCALL_SVC_OP(fsetxattr)
#endif

#else /*SEC_SYSCALL_DO_MINIMAL_CHECK*/


#if defined(__NR_mount)
	SEC_SYSCALL_SVC_OP(mount)
#endif
#if defined(__NR_stat)
	SEC_SYSCALL_SVC_OP(stat)
#endif
#if defined(__NR_statefs)
	SEC_SYSCALL_SVC_OP(statfs)
#endif
#if defined(__NR_statfs64)
	SEC_SYSCALL_SVC_OP(statfs64)
#endif
#if defined(__NR_fstatfs)
	SEC_SYSCALL_SVC_OP(fstatfs)
#endif
#if defined(__NR_fstatfs64)
	SEC_SYSCALL_SVC_OP(fstatfs64)
#endif
#if defined(__NR_lstat)
	SEC_SYSCALL_SVC_OP(lstat)
#endif
#if defined(__NR_fstat)
	SEC_SYSCALL_SVC_OP(fstat)
#endif
#if defined(__NR_ustat)
	SEC_SYSCALL_SVC_OP(ustat)
#endif
/*#if defined(__ARCH_WANT_STAT64) || defined(__ARCH_WANT_COMPAT_STAT64)*/
#if defined(__NR_stat64)
	SEC_SYSCALL_SVC_OP(stat64)
#endif
#if defined(__NR_fstat64)
	SEC_SYSCALL_SVC_OP(fstat64)
#endif
#if defined(__NR_lstat64)
	SEC_SYSCALL_SVC_OP(lstat64)
#endif
#if defined(__NR_fstatat64)
	SEC_SYSCALL_SVC_OP(fstatat64)
#endif
/*#endif*/
#if defined(__NR_getxattr)
	SEC_SYSCALL_SVC_OP(getxattr)
#endif
#if defined(__NR_lgetxattr)
	SEC_SYSCALL_SVC_OP(lgetxattr)
#endif
#if defined(__NR_fgetxattr)
	SEC_SYSCALL_SVC_OP(fgetxattr)
#endif
#if defined(__NR_listxattr)
	SEC_SYSCALL_SVC_OP(listxattr)
#endif
#if defined(__NR_llistxattr)
	SEC_SYSCALL_SVC_OP(llistxattr)
#endif
#if defined(__NR_flistxattr)
	SEC_SYSCALL_SVC_OP(flistxattr)
#endif
#if defined(__NR_fcntl)
	SEC_SYSCALL_SVC_OP(fcntl)
#endif
/*#if (BITS_PER_LONG == 32)*/
#if defined(__NR_fcntl64)
	SEC_SYSCALL_SVC_OP(fcntl64)
#endif
/*#endif*/
#if defined(__NR_mincore)
	SEC_SYSCALL_SVC_OP(mincore)
#endif
#if defined(__NR_ioctl)
	SEC_SYSCALL_SVC_OP(ioctl)
#endif
#if defined(__NR_io_getevents)
	SEC_SYSCALL_SVC_OP(io_getevents)
#endif
#if defined(__NR_io_cancel)
	SEC_SYSCALL_SVC_OP(io_cancel)
#endif
#if defined(__NR_readlink)
	SEC_SYSCALL_SVC_OP(readlink)
#endif
#if defined(__NR_getresuid)
	SEC_SYSCALL_SVC_OP(getresuid)
#endif
#if defined(__NR_getresgid)
	SEC_SYSCALL_SVC_OP(getresgid)
#endif
#if defined(__NR_getgroups)
	SEC_SYSCALL_SVC_OP(getgroups)
#endif
#if defined(__NR_read)
	SEC_SYSCALL_SVC_OP(read)
#endif
#if defined(__NR_readv)
	SEC_SYSCALL_SVC_OP(readv)
#endif
#if defined(__NR_pread64)
	SEC_SYSCALL_SVC_OP(pread64)
#endif
#if defined(__NR_preadv)
	SEC_SYSCALL_SVC_OP(preadv)
#endif
#if defined(__NR_preadv2)
	SEC_SYSCALL_SVC_OP(preadv2)
#endif
#if defined(__NR_write)
	SEC_SYSCALL_SVC_OP(write)
#endif
#if defined(__NR_writev)
	SEC_SYSCALL_SVC_OP(writev)
#endif
#if defined(__NR_pwrite64)
	SEC_SYSCALL_SVC_OP(pwrite64)
#endif
#if defined(__NR_pwritev)
	SEC_SYSCALL_SVC_OP(pwritev)
#endif
#if defined(__NR_pwritev2)
	SEC_SYSCALL_SVC_OP(pwritev2)
#endif
#if defined(__NR_getcwd)
	SEC_SYSCALL_SVC_OP(getcwd)
#endif
#if defined(__NR_lookup_dcookie)
	SEC_SYSCALL_SVC_OP(lookup_dcookie)
#endif
#if defined(__NR_quotactl)
	SEC_SYSCALL_SVC_OP(quotactl)
#endif
#if defined(__NR_getdents)
	SEC_SYSCALL_SVC_OP(getdents)
#endif
#if defined(__NR_getdents64)
	SEC_SYSCALL_SVC_OP(getdents64)
#endif
#if defined(__NR_getsockopt)
	SEC_SYSCALL_SVC_OP(getsockopt)
#endif
#if defined(__NR_getsockname)
	SEC_SYSCALL_SVC_OP(getsockname)
#endif
#if defined(__NR_getpeername)
	SEC_SYSCALL_SVC_OP(getpeername)
#endif
#if defined(__NR_recv)
	SEC_SYSCALL_SVC_OP(recv)
#endif
#if defined(__NR_recvfrom)
	SEC_SYSCALL_SVC_OP(recvfrom)
#endif
#if defined(__NR_recvmsg)
	SEC_SYSCALL_SVC_OP(recvmsg)
#endif
#if defined(__NR_recvmmsg)
	SEC_SYSCALL_SVC_OP(recvmmsg)
#endif
#if defined(__NR_gethostname)
	SEC_SYSCALL_SVC_OP(gethostname)
#endif
#if defined(__NR_uname)
	SEC_SYSCALL_SVC_OP(uname)
#endif
#if defined(__NR_olduname)
	SEC_SYSCALL_SVC_OP(olduname)
#endif
#if defined(__NR_getrlimit)
	SEC_SYSCALL_SVC_OP(getrlimit)
#endif
#if defined(__NR_prlimit64)
	SEC_SYSCALL_SVC_OP(prlimit64)
#endif
#if defined(__NR_getrusage)
	SEC_SYSCALL_SVC_OP(getrusage)
#endif
#if defined(__NR_msgrcv)
	SEC_SYSCALL_SVC_OP(msgrcv)
#endif
#if defined(__NR_msgctl)
	SEC_SYSCALL_SVC_OP(msgctl)
#endif
#if defined(__NR_semctl)
	SEC_SYSCALL_SVC_OP(semctl)
#endif
#if defined(__NR_shmctl)
	SEC_SYSCALL_SVC_OP(shmctl)
#endif
#if defined(__NR_ipc)
	SEC_SYSCALL_SVC_OP(ipc)
#endif
#if defined(__NR_mq_timedreceive)
	SEC_SYSCALL_SVC_OP(mq_timedreceive)
#endif
#if defined(__NR_notify)
	SEC_SYSCALL_SVC_OP(notify)
#endif
#if defined(__NR_mq_getsetattr)
	SEC_SYSCALL_SVC_OP(mq_getsetattr)
#endif
#if defined(__NR_pciconfig_read)
	SEC_SYSCALL_SVC_OP(pciconfig_read)
#endif
#if defined(__NR_sysctl)
	SEC_SYSCALL_SVC_OP(sysctl)
#endif
#if defined(__NR_sysinfo)
	SEC_SYSCALL_SVC_OP(sysinfo)
#endif
#if defined(__NR_sysfs)
	SEC_SYSCALL_SVC_OP(sysfs)
#endif
#if defined(__NR_syslog)
	SEC_SYSCALL_SVC_OP(syslog)
#endif
#if defined(__NR_ptrace)
	SEC_SYSCALL_SVC_OP(ptrace)
#endif
#if defined(__NR_request_key)
	SEC_SYSCALL_SVC_OP(request_key)
#endif
#if defined(__NR_keyctl)
	SEC_SYSCALL_SVC_OP(keyctl)
#endif
#if defined(__NR_get_mempolicy)
	SEC_SYSCALL_SVC_OP(get_mempolicy)
#endif
#if defined(__NR_newfstatat)
	SEC_SYSCALL_SVC_OP(newfstatat)
#endif
#if defined(__NR_readlinkat)
	SEC_SYSCALL_SVC_OP(readlinkat)
#endif
#if defined(__NR_splice)
	SEC_SYSCALL_SVC_OP(splice)
#endif
#if defined(__NR_vmsplice)
	SEC_SYSCALL_SVC_OP(vmsplice)
#endif
#if defined(__NR_get_robust_list)
	SEC_SYSCALL_SVC_OP(get_robust_list)
#endif
#if defined(__NR_getcpu)
	SEC_SYSCALL_SVC_OP(getcpu)
#endif
#if defined(__NR_fork)
	SEC_SYSCALL_SVC_OP(fork)
#endif
#if defined(__NR_vfork)
	SEC_SYSCALL_SVC_OP(vfork)
#endif
#if defined(__NR_clone)
	SEC_SYSCALL_SVC_OP(clone)
#endif
/*should be defined for SEC_SYSCALL_CHECK_INIT_STAGE_BY_EXECVE_CALL*/
#if defined(__NR_execve)
	SEC_SYSCALL_SVC_OP(execve)
#endif
#if defined(__NR_name_to_handle_at)
	SEC_SYSCALL_SVC_OP(name_to_handle_at)
#endif
#if defined(__NR_process_vm_readv)
	SEC_SYSCALL_SVC_OP(process_vm_readv)
#endif
#if defined(__NR_getrandom)
	SEC_SYSCALL_SVC_OP(getrandom)
#endif
#if defined(__NR_execveat)
	SEC_SYSCALL_SVC_OP(execveat)
#endif
#if defined(__NR_copy_file_range)
	SEC_SYSCALL_SVC_OP(copy_file_range)
#endif
#if defined(__NR_get_thread_area)
	SEC_SYSCALL_SVC_OP(get_thread_area)
#endif
#if defined(__NR_prctl)
	SEC_SYSCALL_SVC_OP(prctl)
#endif
#if defined(__NR_query_module)
	SEC_SYSCALL_SVC_OP(query_module)
#endif
#if defined(__NR_readdir)
	SEC_SYSCALL_SVC_OP(readdir)
#endif
#if defined(__NR_sigaction)
	SEC_SYSCALL_SVC_OP(sigaction)
#endif
#if defined(__NR_sigprocmask)
	SEC_SYSCALL_SVC_OP(sigprocmask)
#endif
#if defined(__NR_rt_sigaction)
	SEC_SYSCALL_SVC_OP(rt_sigaction)
#endif
#if defined(__NR_rt_sigprocmask)
	SEC_SYSCALL_SVC_OP(rt_sigprocmask)
#endif
#if defined(__NR_rt_sigqueueinfo)
	SEC_SYSCALL_SVC_OP(rt_sigqueueinfo)
#endif
#if defined(__NR_rt_tgsigqueueinfo)
	SEC_SYSCALL_SVC_OP(rt_tgsigqueueinfo)
#endif
#if defined(__NR_sigaltstack)
	SEC_SYSCALL_SVC_OP(sigaltstack)
#endif
#if defined(__NR_sigwaitinfo)
	SEC_SYSCALL_SVC_OP(sigwaitinfo)
#endif
#if defined(__NR_sigtimedwait)
	SEC_SYSCALL_SVC_OP(sigtimedwait)
#endif
#if defined(__NR_rt_sigwaitinfo)
	SEC_SYSCALL_SVC_OP(rt_sigwaitinfo)
#endif
#if defined(__NR_rt_sigtimedwait)
	SEC_SYSCALL_SVC_OP(rt_sigtimedwait)
#endif
#if defined(__NR_sched_getparam)
	SEC_SYSCALL_SVC_OP(sched_getparam)
#endif
#if defined(__NR_sched_getaffinity)
	SEC_SYSCALL_SVC_OP(sched_getaffinity)
#endif
#if defined(__NR_timer_settime)
	SEC_SYSCALL_SVC_OP(timer_settime)
#endif
#if defined(__NR_timer_gettime)
	SEC_SYSCALL_SVC_OP(timer_gettime)
#endif
#if defined(__NR_timerfd_settime)
	SEC_SYSCALL_SVC_OP(timerfd_settime)
#endif
#if defined(__NR_timerfd_gettime)
	SEC_SYSCALL_SVC_OP(timerfd_gettime)
#endif
#if defined(__NR_times)
	SEC_SYSCALL_SVC_OP(times)
#endif
#if defined(__NR_time)
	SEC_SYSCALL_SVC_OP(time)
#endif
#if defined(__NR_ugetrlimit)
	SEC_SYSCALL_SVC_OP(ugetrlimit)
#endif
#if defined(__NR_stime)
	SEC_SYSCALL_SVC_OP(stime)
#endif
#if defined(__NR_gettimeofday)
	SEC_SYSCALL_SVC_OP(gettimeofday)
#endif
#if defined(__NR_adjtimex)
	SEC_SYSCALL_SVC_OP(adjtimex)
#endif
#if defined(__NR_nanosleep)
	SEC_SYSCALL_SVC_OP(nanosleep)
#endif
#if defined(__NR_sigpending)
	SEC_SYSCALL_SVC_OP(sigpending)
#endif
#if defined(__NR_rt_sigpending)
	SEC_SYSCALL_SVC_OP(rt_sigpending)
#endif
#if defined(__NR_capget)
	SEC_SYSCALL_SVC_OP(capget)
#endif
#if defined(__NR_getitimer)
	SEC_SYSCALL_SVC_OP(getitimer)
#endif
#if defined(__NR_clock_gettime)
	SEC_SYSCALL_SVC_OP(clock_gettime)
#endif
#if defined(__NR_clock_adjtime)
	SEC_SYSCALL_SVC_OP(clock_adjtime)
#endif
#if defined(__NR_clock_getres)
	SEC_SYSCALL_SVC_OP(clock_getres)
#endif
#if defined(__NR_clock_nanosleep)
	SEC_SYSCALL_SVC_OP(clock_nanosleep)
#endif
#if defined(__NR_sched_getattr)
	SEC_SYSCALL_SVC_OP(sched_getattr)
#endif
#if defined(__NR_sched_rr_get_interval)
	SEC_SYSCALL_SVC_OP(sched_rr_get_interval)
#endif
#if defined(__NR_wait)
	SEC_SYSCALL_SVC_OP(wait)
#endif
#if defined(__NR_waitpid)
	SEC_SYSCALL_SVC_OP(waitpid)
#endif
#if defined(__NR_waitid)
	SEC_SYSCALL_SVC_OP(waitid)
#endif
#if defined(__NR_futex)
	SEC_SYSCALL_SVC_OP(futex)
#endif
#if defined(__NR_io_submit)
	SEC_SYSCALL_SVC_OP(io_submit)
#endif
#if defined(__NR_sendfile)
	SEC_SYSCALL_SVC_OP(sendfile)
#endif
#if defined(__NR_sendfile64)
	SEC_SYSCALL_SVC_OP(sendfile64)
#endif
#if defined(__NR_llseek)
	SEC_SYSCALL_SVC_OP(llseek)
#endif
#if defined(__NR_accept)
	SEC_SYSCALL_SVC_OP(accept)
#endif
#if defined(__NR_socketpair)
	SEC_SYSCALL_SVC_OP(socketpair)
#endif
#if defined(__NR_select)
	SEC_SYSCALL_SVC_OP(select)
#endif
#if defined(__NR_pselect)
	SEC_SYSCALL_SVC_OP(pselect)
#endif
#if defined(__NR_epoll_ctl)
	SEC_SYSCALL_SVC_OP(epoll_ctl)
#endif
#if defined(__NR_epoll_wait)
	SEC_SYSCALL_SVC_OP(epoll_wait)
#endif
#if defined(__NR_epoll_pwait)
	SEC_SYSCALL_SVC_OP(epoll_pwait)
#endif
#if defined(__NR_pselect6)
	SEC_SYSCALL_SVC_OP(pselect6)
#endif
#if defined(__NR_poll)
	SEC_SYSCALL_SVC_OP(poll)
#endif
#if defined(__NR_ppoll)
	SEC_SYSCALL_SVC_OP(ppoll)
#endif
#if defined(__NR_seccomp)
	SEC_SYSCALL_SVC_OP(seccomp)
#endif
#if defined(__NR_bpf)
	SEC_SYSCALL_SVC_OP(bpf)
#endif
#if defined(__NR_syscall)
	SEC_SYSCALL_SVC_OP(syscall)
#endif


#endif /*!SEC_SYSCALL_DO_MINIMAL_CHECK*/