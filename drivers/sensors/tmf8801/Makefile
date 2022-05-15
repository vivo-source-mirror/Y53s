ifneq ($(KERNELRELEASE),)
#kbuild part of Makefile
include Kbuild
else
#normal Makefile
all:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_SRC) M=$$PWD modules

modules:
	$(MAKE) ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(LINUX_SRC) M=$$PWD $@

clean:
	$(MAKE) -C $(LINUX_SRC) M=$$PWD clean
endif
