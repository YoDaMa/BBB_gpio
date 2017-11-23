obj-m+=gpio_simple.o
KDIR := ~/bb-kernel/KERNEL
CC  := arm-linux-gnueabihf-gcc

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	$(CC) gpio-lkm-test.c -o test
clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
	rm test
