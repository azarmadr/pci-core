ifneq ($(KERNELRELEASE),)
# When Makefile is invoked from kernel build system, define objects

ccflags-y += -DGIT_REVISION="$(shell git --git-dir=$(src)/../.git log -n1 --pretty='format:%h (%ci)')"

obj-m += pmc_wb_msi.o pmc_wb_intx.o wishbone.o

else
# When the Makefile is run directly, invoke the kernel build system

KERNELVER ?= `uname -r`
KERNELDIR ?= /lib/modules/$(KERNELVER)/build
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) INSTALL_MOD_PATH=$(STAGING) modules_install
	mkdir -p $(STAGING)/etc/udev/rules.d
	cp 95-pmc-wb.rules $(STAGING)/etc/udev/rules.d
	[ $$(id -u) -ne 0 ] || depmod -a
	[ $$(id -u) -ne 0 ] || groupadd wishbone || true

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

endif
