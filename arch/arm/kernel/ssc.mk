CONFIG_SEC_SYSCALL := y

ifeq (y, $(CONFIG_SEC_SYSCALL))
	KCOV_INSTRUMENT_ssc_utils.o := n
	KASAN_SANITIZE_ssc_utils.o := n
	UBSAN_SANITIZE_ssc_utils.o := n
	CC_INSTRUMENT_ssc_utils.o := n
	KCOV_INSTRUMENT_ssc.o := n
	KASAN_SANITIZE_ssc.o := n
	UBSAN_SANITIZE_ssc.o := n
	CC_INSTRUMENT_ssc.o := n

	ifeq (y, $(CONFIG_ARM64))
		arm64-obj-$(CONFIG_SEC_SYSCALL) += ssc_utils.o ssc_imp.o ssc.o
	else
		obj-$(CONFIG_SEC_SYSCALL) += ssc_utils.o ssc_imp.o ssc.o
	endif

	ifeq (y, $(CONFIG_SECURITY))
		ifeq (y, $(CONFIG_SECURITY_SELINUX))
			ccflags-y += $(addprefix -I, $(obj)/../../../security/selinux)
			#KBUILD_CFLAGS += $(addprefix -I, $(obj)/../../../security/selinux)
		endif
	endif

	#ifeq (y, $(CONFIG_ARM64))
	#	AFLAGS_entry.o	+= -DCONFIG_SEC_SYSCALL
	#else
	#	AFLAGS_entry-common.o	+= -DCONFIG_SEC_SYSCALL
	#endif

	asflags-y	+= -DCONFIG_SEC_SYSCALL
	ccflags-y	+= -DCONFIG_SEC_SYSCALL

	CFLAGS_ssc.o += -fno-stack-protector -O3
	CFLAGS_ssc_utils.o += -fno-stack-protector -O3

	#KBUILD_AFLAGS	+= -DCONFIG_SEC_SYSCALL
	#KBUILD_CFLAGS	+= -DCONFIG_SEC_SYSCALL
	#KBUILD_CPPFLAGS	+= -DCONFIG_SEC_SYSCALL

	ifeq (y, $(CONFIG_ARM64))
		KCOV_INSTRUMENT_ssc_compat.o := n
		KASAN_SANITIZE_ssc_compat.o := n
		UBSAN_SANITIZE_ssc_compat.o := n
		CC_INSTRUMENT_ssc_compat.o := n

		arm64-obj-$(CONFIG_COMPAT) += ssc_compat_imp.o ssc_compat.o
		CFLAGS_ssc_compat.o += -fno-stack-protector -O3
	else
		KCOV_INSTRUMENT_ssc_oabi.o := n
		KASAN_SANITIZE_ssc_oabi.o := n
		UBSAN_SANITIZE_ssc_oabi.o := n
		CC_INSTRUMENT_ssc_oabi.o := n

		obj-$(CONFIG_OABI_COMPAT) += ssc_oabi_imp.o ssc_oabi.o
		CFLAGS_ssc_oabi.o += -fno-stack-protector -O3
	endif
endif
