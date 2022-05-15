obj-$(CONFIG_RSC_V2) := rsc_core.o rsc_sys_api.o
obj-$(CONFIG_RSC_V2_CPU_CAP_NOTIFY) += rsc_cpu_capability.o
obj-$(CONFIG_RSC_TASK_CPU_USAGE) += rsc_cpu_usage.o
obj-$(CONFIG_RSC_UID_IO) += rsc_uid_io.o
obj-$(CONFIG_RSC_CPU_TOP) += rsc_cpu_top.o
obj-$(CONFIG_RSC_IOLIMIT) += rsc_iolimit.o
obj-$(CONFIG_RSC_MEM_MON) += rsc_mem_mon.o
obj-$(CONFIG_RSC_ZRAM) += rsc_zram.o
obj-$(CONFIG_RSC_MEM_DEFRAG) += rsc_mem_defrag.o
obj-$(CONFIG_RSC_BINDER_OPTIMIZATION) += rsc_setting.o
obj-$(CONFIG_RSC_BOOST_TASKKILL) += rsc_boost_taskkill.o
obj-$(CONFIG_RSC_VAUDIT) += rsc_vaudit.o
obj-$(CONFIG_RSC_KSWAPD_IMPROVE) += rsc_vswapd.o
obj-$(CONFIG_RSC_SVP_TASK_SCHEDULE) += rsc_schedule.o
obj-$(CONFIG_RSC_BINDER_EVENT_NOTI) += rsc_binderfreez_notify.o

ifneq (,$(wildcard $(srctree)/../vendor/vivo/proprietary-osc/))
$(obj)/%.o:$(srctree)/../vendor/vivo/proprietary-osc/kernel-4.14/kernel/rsc/%.o
	cp $< $@
endif
