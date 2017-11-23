#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "beaglebone-gpio.h"

#define NUM_CALIBRATE 500;

int main(int argc, char *argv[]) {
    long capValue = 0;
    float long capReturnVal = 0;
    float long calibrationVal = 0.0;
    int fd = open("/dev/simplegpio424", O_RDWR);
    if (fd < 0){
      perror("Failed to open the device...");
      return errno;
    }
    printf("Calibrating...\n");
    int i;
    for (i=0; i<NUM_CALIBRATE; i++) {
        ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
        capReturnVal = ioctl(fd, IOCTL_GET_VALUE, &capValue) / CLOCK_SPEED;
        calibrationVal += capReturnVal;
    }
    // set threshold for measuring touch
    calibrationVal = (calibrationVal / NUM_CALIBRATE) * 1.3;


    int 
    for (i=0; i<10000; i++) {
            // printf("Measuring capacitance...\n");
        ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
        // printf("done.\n");
        capReturnVal = ioctl(fd, IOCTL_GET_VALUE, &capValue) * 1.0 / CLOCK_SPEED;

        if (capReturnVal > calibrationVal) {
            printf("(%lf) Touch Detected!\n", capReturnVal);
        } else {
            printf("(%lf) nothing...\n", capReturnVal);
        }
    }
    close(fd);
    return 0;
}
