# include header files for MTK
#ccflags-y += -I$(srctree)/drivers/spi/mediatek/mt6797/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ipio_touch_driver/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
#
#ccflags-y += -Wall
#
#BUILD_INFAE := spi
#BUILD_PLATFORM := qcom
#
#ifeq ($(BUILD_PLATFORM),mtk)
#platform=ilitek_plat_mtk
#endif
#
#ifeq ($(BUILD_PLATFORM),qcom)
#platform=ilitek_plat
#endif
#
#ifeq ($(BUILD_INFAE),i2c)
#interface=ilitek_i2c
#endif
#
#ifeq ($(BUILD_INFAE),spi)
#interface=ilitek_spi
#endif

obj-y += ilitek_main.o \
	ilitek_spi.o \
	ilitek_plat.o \
	ilitek_ic.o \
	ilitek_touch.o \
	ilitek_fw.o \
	ilitek_node.o \
	ilitek_core.o
