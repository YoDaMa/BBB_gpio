#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "beaglebone-gpio.h"

#define NUM_CALIBRATE 10000

int main(int argc, char *argv[]) {
    long capValue = 0;
    float capReturnVal = 0;
    static float calibrationVal = 0.0;
    int fd = open("/dev/simplegpio424", O_RDWR);
    if (fd < 0){
      perror("Failed to open the device...");
      return errno;
    }
    printf("Calibrating...\n");
    int i;
    for (i=0; i<NUM_CALIBRATE; i++) {
        ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
        capReturnVal = 1.0 * ioctl(fd, IOCTL_GET_VALUE, &capValue) / CLOCK_SPEED;
        calibrationVal += capReturnVal;
    }
    // set threshold for measuring touch
    calibrationVal = (calibrationVal / NUM_CALIBRATE) * 1.3;
    printf("Finsihed Calibrating. \n");
    printf("Calibration value is: %f  \n", calibrationVal);


    for (i=0; i<10000; i++) {
            // printf("Measuring capacitance...\n");
        ioctl(fd, IOCTL_MEASURE_CAPACITANCE);
        // printf("done.\n");
        capReturnVal = ioctl(fd, IOCTL_GET_VALUE, &capValue) * 1.0 / CLOCK_SPEED;

        if (capReturnVal > calibrationVal) {
            printf("(%f > %f) Purchased 1 Egg\n", capReturnVal, calibrationVal);
            // for (i=0; i<10000; i++) {
            system("./order_egg.sh");
            // do nothing
            sleep(1000);
            // }
        } else {
            // printf("(%f < %f) nothing...\n", capReturnVal, calibrationVal);
        }
    }
    close(fd);
    return 0;
}
