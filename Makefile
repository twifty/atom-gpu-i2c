CONFIG_MODULE_SIG=n
CONFIG_STACK_VALIDATION=n
MODULE_NAME = aura-gpu

SRCS = \
	atom/atom.c \
	device.c \
	i2c.c \
	main.c

KERNELDIR = /lib/modules/$(shell uname -r)/build
PWD = $(shell pwd)
# KBUILD_EXTRA_SYMBOLS := $(KERNELDIR)/Module.symvers
OBJS = $(SRCS:.c=.o)

ifeq ($(KERNELRELEASE),)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules EXTRA_CFLAGS="-g -DDEBUG -I$(PWD)/../"

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

uninstall:
	sudo rmmod $(MODULE_NAME) || true

install: uninstall all
	sudo insmod $(MODULE_NAME).ko

.PHONY: all clean uninstall install

else

	obj-m += $(MODULE_NAME).o
	$(MODULE_NAME)-y = $(OBJS)

endif
