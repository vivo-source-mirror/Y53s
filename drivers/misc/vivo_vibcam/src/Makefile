MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/hibcore
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat

obj-$(CONFIG_HALL_MXM1120_DOWN) +=   mxm1120_down.o
obj-$(CONFIG_HALL_MXM1120_UP)   +=   mxm1120_up.o
obj-$(CONFIG_HALL_IST8801_DOWN) +=   ist8801_down.o
obj-$(CONFIG_HALL_IST8801_UP)   +=   ist8801_up.o
obj-$(CONFIG_VIVO_VIB8846)      +=   vivo_vib8846.o
obj-y += vivo_hall_class.o
#obj-y += timed_output.o
