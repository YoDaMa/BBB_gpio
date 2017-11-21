#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <sys/ioctl.h>
#include "beaglebone-gpio.h"


int main(int argc, char *argv[]) {
    long capValue = 0;
    int fd = open("/dev/gpio424", O_RDWR);
    printf("Measuring capacitance...\n");
    ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
    printf("done.\n");
    ioctl(fd, IOCTL_GET_VALUE, &capValue);
    printf("Value from capacitance measure is; %d \n", capValue);
    close(fd);
    return 0;
}
