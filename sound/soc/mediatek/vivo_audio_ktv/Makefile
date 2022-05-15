#
# Makefile for ktv 
#
# Author: XDmemory
# Date: 2016-07-18

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat

COMMON_CFLAGS := -I$(src)/../common
ccflags-y  += $(COMMON_CFLAGS)
ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -DAUDIO_USING_WRAP_DRIVER
ccflags-y += -D_GIT318_READY
ccflags-y += -D_WCN_SUPPORT

obj-y += vivo_audio_ktv.o

ifneq (,$(wildcard $(srctree)/../vendor/vivo/proprietary-osc/))
$(obj)/%.o:$(srctree)/../vendor/vivo/proprietary-osc/kernel-4.14/$(src)/%.o
	cp $< $@
endif
