#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
ccflags-y += -I$(vivo_top)/$(VIVO_EXTENSION)/source/system/display/vivosrc
ccflags-y := -I$(srctree)/include/drm
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)   \
             -I$(srctree)/drivers/misc/mediatek/smi/            \
             -I$(srctree)/drivers/misc/mediatek/mmp/            \
             -I$(srctree)/drivers/misc/mediatek/mmdvfs/         \
             -I$(srctree)/drivers/iommu/                \
             -I$(srctree)/drivers/devfreq/              \
             -I$(srctree)/drivers/gpu/drm/              \
			 -I$(srctree)/drivers/gpu/drm/mediatek       \
             -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/               \
             -I$(srctree)/drivers/misc/mediatek/include/mt-plat			\
	     -Iinclude

ccflags-$(CONFIG_MTK_CMDQ_MBOX_EXT) += -I$(srctree)/drivers/misc/mediatek/cmdq/mailbox

ifdef CONFIG_LEDS_MTK_DISP
obj-$(CONFIG_LEDS_MTK_DISP)		+= leds-mtk-disp.o
else ifdef CONFIG_LEDS_MTK_PWM
obj-$(CONFIG_LEDS_MTK_PWM)		+= leds-mtk-pwm.o
else
ifeq (y, $(filter y, $(CONFIG_MACH_MT6765) $(CONFIG_MACH_MT6761)))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/mt6765
else ifeq (y, $(filter y, $(CONFIG_MACH_MT6885) $(CONFIG_MACH_MT6893)))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/mt6885
else ifeq (y, $(filter y, $(CONFIG_MACH_MT6853) $(CONFIG_MACH_MT6833)))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/mt6853
else
ccflags-y += -I$(srctree)/drivers/misc/mediatek/leds/$(MTK_PLATFORM)
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include

obj-$(CONFIG_MTK_LEDS) += mtk_leds_drv.o

ifeq (y, $(filter y, $(CONFIG_MACH_MT6765) $(CONFIG_MACH_MT6761)))
obj-y += $(subst ",,mt6765)/
else ifeq (y, $(filter y, $(CONFIG_MACH_MT6885) $(CONFIG_MACH_MT6893)))
obj-y += $(subst ",,mt6885)/
else ifeq (y, $(filter y, $(CONFIG_MACH_MT6853) $(CONFIG_MACH_MT6833)))
obj-y += $(subst ",,mt6853)/
else
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
endif
