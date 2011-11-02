TOPDIR          := /usr/src/linux
MOD_ROOT        :=
PWD             := $(shell pwd)

obj-m           := lcdtopgun.o

default:
	$(MAKE) -C $(TOPDIR) SUBDIRS=$(PWD) modules

clean:
	rm -f *.o

install:
	$(MAKE) -C $(TOPDIR) SUBDIRS=$(PWD) modules_install
	depmod -ae

