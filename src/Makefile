obj-m+=gpio-lkm.o
KDIR := ~/bb-kernel/KERNEL
GCC  := arm-linux-gnueabihf-gcc

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	$(GCC) -o gpio-lkm-test gpio-lkm-test
clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean

