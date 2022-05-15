#
# Makefile for pem drivers
#
obj-y += pem_driver.o

ccflags-y += -Idrivers/pem/include

pem_driver-objs	:= cpulimit/cpulimit.o pem_main.o

ifneq (,$(wildcard $(srctree)/../vendor/vivo/proprietary-osc/))
$(obj)/%.o:$(srctree)/../vendor/vivo/proprietary-osc/kernel-4.14/$(src)/%.o
	cp $< $@

$(obj)/cpulimit/cpulimit.o:$(srctree)/../vendor/vivo/proprietary-osc/kernel-4.14/$(src)/cpulimit/cpulimit.o
	cp $< $@
endif
