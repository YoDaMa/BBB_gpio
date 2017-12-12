obj-m+=gpio_sysfs.o
KDIR := ~/bb-kernel/KERNEL
CC  := arm-linux-gnueabihf-gcc

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules
	$(CC) gpio_simple_test.c -o gpio_simple_test
clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
	rm test
