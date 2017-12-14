#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include "beaglebone-gpio.h"

float * updateHist(float *, float);

int main(int argc, char *argv[]) {
    unsigned int count = 0;
    float *dischargeHist;
    float dischargeTime = 0;
    float calibrationTime = 0;
    FILE *file;
    FILE *pipe;
    int err;
    unsigned int calibrating = 1;
	char * place = "can be anything";
    dischargeHist = (float *)malloc(5 * sizeof(float)); 
    int j;
    for (j=0; j < 5; j++) {
        dischargeHist[j] = 0.0;
    } 
    while (1) {
        file = fopen("/sys/elec424/gpio20/touch", "w+");
        if (file == NULL) {
            printf("Failed to Open. \n");
			 printf("Errno: %s\n", strerror(errno));
			 exit(-1);
        }
        err = fwrite(place, sizeof(char), strlen(place), file);
        if (err < 0) {
            printf("Initialization failed. \n");
            exit(-1);
        }
        fclose(file);
        file = fopen("/sys/elec424/gpio20/touch", "w+");
        if (file == NULL) {
            printf("Failed to Open. \n");
			 printf("Errno: %s\n", strerror(errno));
			 exit(-1);
        }
        char *message = malloc(250);
		if ((err = fread(message, sizeof(char), 250, file)) < 0) {
				 printf("Read failed.\n");
				 exit(-1);
		}
        dischargeTime = atol(message);
        dischargeHist = updateHist(dischargeHist, dischargeTime);

            // printf("Measuring capacitance...\n");
        if (calibrating) {
            // for (j=0; j < 100; j++) {
                // printf("Calibrating... Current value: %f \n", dischargeTime);
            // }
            printf("Please enter calibration value: ");
            scanf("%f", &calibrationTime);
            calibrating = !calibrating; 
	    } else {
            int foo = 0;
            // check to make sure the history 
            for (j=0; j<5; j++) {
                if (dischargeHist[j] > calibrationTime) {
                    foo += 1;
                }
            }
            if (foo == 5 && count > 1000) {
                printf("(%f > %f) Ordered !\n", dischargeTime, calibrationTime);
                printf("Count is: %d\n",count);
		pipe = popen("./order_egg.sh", "r");
                // do nothing
                count=0;
		printf("Returning...");
		sleep(1000);
		pclose(pipe);
            } else {
                printf("(%f < %f) nothing...\n", dischargeTime, calibrationTime);
            }
        }
        count++;
        free(message);
        fclose(file);
    }
    return 0;
}


float* updateHist(float *histArray, float newVal) {
    int i;
    for (i=0; i<4; i++) {
        histArray[i] = histArray[i+1];
    }
    histArray[4] = newVal;
    return histArray;
}
