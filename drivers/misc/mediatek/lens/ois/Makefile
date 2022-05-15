#
# Makefile for misc devices that really don't fit anywhere else.
#
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/ois/LC89129
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/ois/inc
subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/lens/ois/DW9781C
obj-y += ois_core.o ois_util.o
obj-y += LC89129/
obj-y += DW9781C/