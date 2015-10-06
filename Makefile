ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m := advantech_can_pci.o

# Unfortunately the sja1000.h is an internal header. We need kernel-source
# and locate it from there.
ccflags-y += -I$(srctree)/drivers/net/can/sja1000/

else
# normal makefile
KVERSION ?= `uname -r`
KDIR ?= /lib/modules/$(KVERSION)/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

%:
	$(MAKE) -C $(KDIR) M=$$PWD $@

endif
