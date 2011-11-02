TOPDIR          := /usr/src/linux
MOD_ROOT        :=
PWD             := $(shell pwd)

obj-m           := lcdtopgun.o

default:
	$(MAKE) -C $(TOPDIR) SUBDIRS=$(PWD) modules

clean:
	rm -f lcdtopgun.o lcdtopgun.ko
	rm -f lcdtopgun.mod.c lcdtopgun.mod.o
	rm -f Module.symvers
	rm -f .lcdtopgun*
	rm -fr .tmp_versions

install:
	$(MAKE) -C $(TOPDIR) SUBDIRS=$(PWD) modules_install
	depmod -ae

