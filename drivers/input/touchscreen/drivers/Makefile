ifeq ($(CONFIG_VIVO_TS),)
  CONFIG_VIVO_TS=y
endif


obj-$(CONFIG_VIVO_TS) += vivo_ts.o

vivo_ts-y += vts_core.o
vivo_ts-y += vts_node.o
vivo_ts-y += vts_incell.o
vivo_ts-y += vts_class.o
vivo_ts-y += vts_report.o
vivo_ts-y += vts_proxminity.o
vivo_ts-y += vts_policy.o
vivo_ts-y += vts_op.o
vivo_ts-y += vts_spi_drv.o
vivo_ts-y += vts_sched.o
vivo_ts-y += vts_policy_iqoo5.o
vivo_ts-y += vts_policy_common.o
vivo_ts-y += vts_policy_gpoint.o
vivo_ts-y += vts_policy_x60.o
vivo_ts-y += vts_collect.o
vivo_ts-y += vts_rejection.o

vivo_ts-y += $(addprefix ftm4/, \
				fts.o \
				fts_driver_test.o \
				fts_gui.o \
			)

vivo_ts-y += $(addprefix ftm4/fts_lib/, \
				ftsCompensation.o \
				ftsCrossCompile.o \
				ftsError.o \
				ftsFlash.o \
				ftsFrame.o \
				ftsGesture.o \
				ftsIO.o \
				ftsTest.o \
				ftsTime.o \
				ftsTool.o \
			)

vivo_ts-y += $(addprefix nt36670/, \
				nt36xxx.o \
				nt36xxx_ext_proc.o \
				nt36xxx_fw_update.o \
				nt36xxx_mp_ctrlram.o \
			)

vivo_ts-y += $(addprefix nt-no-flash/, \
				nt36xxx.o \
				nt36xxx_ext_proc.o \
				nt36xxx_fw_update.o \
				nt36xxx_mp_ctrlram.o \
			)

vivo_ts-y += $(addprefix samsung_y761/, \
				sec_ts.o \
				sec_ts_fn.o \
				sec_ts_fw.o \
				sec_ts_only_vendor.o \
			)

vivo_ts-y += $(addprefix ft8719-no-flash/, \
				focaltech_core.o \
				focaltech_esdcheck.o \
				focaltech_ex_fun.o \
				focaltech_flash.o \
				focaltech_gesture.o \
				focaltech_spi.o \
				focaltech_test.o \
			)

vivo_ts-y += $(addprefix st80y/, \
				fts.o \
				fts_proc.o \
			)

vivo_ts-y += $(addprefix st80y/fts_lib/, \
				ftsLockdownReadwrite.o \
				ftsCompensation.o \
				ftsCore.o \
				ftsError.o \
				ftsFlash.o \
				ftsFrame.o \
				ftsGesture.o \
				ftsIO.o \
				ftsTest.o \
				ftsTime.o \
				ftsTool.o \
			)

vivo_ts-y += $(addprefix gt9886/, \
				goodix_cfg_bin.o \
				goodix_ts_core.o \
				goodix_ts_gesture.o \
				goodix_ts_i2c.o \
				goodix_ts_node.o \
				goodix_ts_spi.o \
				goodix_ts_tools.o \
				goodix_ts_update.o \
			)

vivo_ts-y += $(addprefix ft8756-no-flash/, \
				focaltech_core.o \
				focaltech_esdcheck.o \
				focaltech_ex_fun.o \
				focaltech_flash.o \
				focaltech_gesture.o \
				focaltech_spi.o \
				focaltech_test.o \
			)

vivo_ts-y += $(addprefix himax83113/, \
				himax_common.o \
				himax_debug.o \
				himax_ic_HX83112.o \
				himax_ic_incell_core.o \
				himax_inspection.o \
				himax_platform.o \
			)

vivo_ts-y += $(addprefix ilitek/, \
				ilitek_core.o \
				ilitek_fw.o \
				ilitek_ic.o \
				ilitek_main.o \
				ilitek_node.o \
				ilitek_plat.o \
				ilitek_spi.o \
				ilitek_touch.o \
			)

vivo_ts-y += $(addprefix ili_9882n/, \
				ili9881x.o \
				ili9881x_hostdl.o \
				ili9881x_ic.o \
				ili9881x_node.o \
				ili9881x_plat.o \
				ili9881x_spi.o \
				ili9881x_touch.o \
				vivo_v2.o \
			)

vivo_ts-y += $(addprefix syna_S3908_tcm/, \
				synaptics_tcm_i2c.o \
				synaptics_tcm_core.o \
				synaptics_tcm_touch.o \
				synaptics_tcm_device.o \
				synaptics_tcm_testing.o \
				synaptics_tcm_reflash.o \
				synaptics_tcm_recovery.o \
				synaptics_tcm_diagnostics.o \
				synaptics_tcm_vivo_interface.o \
			)

vivo_ts-y += $(addprefix gt9897/, \
				goodix_brl_i2c.o \
				goodix_brl_spi.o \
				goodix_brl_node.o \
				goodix_ts_core.o \
				goodix_brl_hw.o \
				goodix_cfg_bin.o \
				goodix_ts_utils.o \
				goodix_brl_fwupdate.o \
				goodix_ts_tools.o\
				goodix_ts_gesture.o\
			)

vivo_ts-y += $(addprefix chipone_nl9911c/, \
				cts_i2c_driver.o \
				cts_core.o \
				cts_platform.o \
				cts_sfctrlv2.o \
				cts_spi_flash.o \
				cts_firmware.o \
				cts_tool.o \
				cts_sysfs.o \
				cts_test.o \
				cts_charger_detect.o \
				cts_earjack_detect.o \
				cts_strerror.o \
				cts_vendor_vivo.o \
			)

vivo_ts-y += $(addprefix ft3518u/, \
				focaltech_core.o \
				focaltech_esdcheck.o \
				focaltech_ex_fun.o \
				focaltech_ex_mode.o \
				focaltech_flash.o \
				focaltech_gesture.o \
				focaltech_point_report_check.o \
				focaltech_spi_ft3518u.o \
)

vivo_ts-y += $(addprefix ft3518u/focaltech_flash/, \
				focaltech_upgrade_ft3518u.o \
)

vivo_ts-y += vts_init.o
vivo_ts-y += vts_log_switch.o

ifeq ($(VIVO_CONFIG_MTK_DRM),y)
ccflags-y += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/
endif

LINUX_KERNEL_VERSION := $(shell expr $(VERSION) \* 65536 \+ $(PATCHLEVEL) \* 256 \+ $(SUBLEVEL))
KERNEL_VERSION_COM := $(shell expr $(LINUX_KERNEL_VERSION) \>= 328704)

ifeq ($(KERNEL_VERSION_COM),1)
	ifeq ($(BBK_PRODUCT_OVERSEAS),no)
		ifeq ($(VIVO_DATA_COLLECT_FOR_TP),y)
			KBUILD_CFLAGS += -DTOUCH_DATA_COLLECT
		endif
	endif
else
	ifeq ($(BBK_PRODUCT_OVERSEAS),no)
		KBUILD_CFLAGS += -DTOUCH_DATA_COLLECT
	endif
endif
