#BUILD_INFAE := spi
#BUILD_PLATFORM := qcom
#BUILD_MODULE := n
#
#ccflags-y += -Wall
#
#ifeq ($(BUILD_PLATFORM),qcom)
#platform=ili9881x_plat
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/oreo_9881n/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/oreo_9881n/firmware/
#endif
#
#ifeq ($(BUILD_INFAE),spi)
#interface=ili9881x_spi
#fwupdate=ili9881x_hostdl
#endif
#
#ifeq ($(BUILD_MODULE),n)
#obj-y += ili9881x.o \
#	$(interface).o \
#	$(platform).o \
#	ili9881x_ic.o \
#	ili9881x_touch.o \
#	ili9881x_mp.o \
#	$(fwupdate).o \
#	ili9881x_node.o
#else
#	obj-m += ilitek.o
#	ilitek-y := ili9881x.o \
#		$(interface).o \
#		$(platform).o \
#		ili9881x_ic.o \
#		ili9881x_touch.o \
#		ili9881x_mp.o \
#		$(fwupdate).o \
#		ili9881x_node.o
#
#KERNEL_DIR= /home/likewise-open/ILI/1061279/workplace/rk3288_sdk/kernel
#all:
#	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
#clean:
#	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
#endif

obj-y += ili9881x.o \
		 ili9881x_spi.o \
		 ili9881x_ic.o \
		 ili9881x_node.o \
		 ili9881x_plat.o \
		 ili9881x_hostdl.o \
		 ili9881x_touch.o \
		 vivo_v2.o
