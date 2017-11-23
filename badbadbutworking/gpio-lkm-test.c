#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "beaglebone-gpio.h"


int main(int argc, char *argv[]) {
    long capValue = 0;
    long capReturnVal = 0;
    int fd = open("/dev/simplegpio424", O_RDWR);
    if (fd < 0){
      perror("Failed to open the device...");
      return errno;
    }
    int i;
    for (i=0; i<1000; i++) {
            printf("Measuring capacitance...\n");
        ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
        printf("done.\n");
        capReturnVal = ioctl(fd, IOCTL_GET_VALUE, &capValue);
        printf("Capacitance: %ld  | (pointer is %ld) \n", capReturnVal, capValue);
        delay(100);
    }
    close(fd);
    return 0;
}
