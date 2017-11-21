obj-m+=gpio-lkm.o
KDIR := ~/bb-kernel/KERNEL
CC  := arm-linux-gnueabihf-gcc

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	$(CC) gpio-lkm-test.c -o gpio-lkm-test
clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
	rm test
