#ifndef _BEAGLEBONE_GPIO_H_
#define _BEAGLEBONE_GPIO_H_

#define GPIO0 0x44e07000
#define GPIO1 0x4804c000            // only need this bank for GPIO1_17
#define GPIO2 0x481ac000
#define GPIO3 0x481ae000
#define GPIO_OE			  0x134		// for enabling output capabilities; 0: Output, 1: Input
#define GPIO_DATAIN       0x138     // for reading data 
#define GPIO_DATAOUT      0x13c     // for setting and reading the data
#define GPIO_FALLINGDETECT 0x14C 	// for enabling falling edge detection for IRQ
#define GPIO_CLEARDATAOUT 0x190     // for clearing the registers
#define GPIO_SETDATAOUT   0x194     // for settings the DATOUT register
#define PIN_17 0x00020000 // hard-coded GPIO1_17
#define PIN_18 0x00040000 // hard-coded GPIO1_17
#define PIN_19 0x00080000 // hard-coded pin 19
#define PIN_20 0x00100000 // hard coded GPIO0_20
#define CLOCK_SPEED 1000000000      // 1 GHZ
#define MYMEM_IOCTL_MAGIC 		245
#define IOCTL_GET_VALUE _IOWR(MYMEM_IOCTL_MAGIC, 0, char *)
/* _IORW means that we're creating an ioctl command 
 * number for passing information from a user process
 * to the kernel module and for passing information from
 * the kernel module to a user process.
 */
#define IOCTL_MEASURE_CAPACITANCE _IOR(MYMEM_IOCTL_MAGIC, 0, char *)
/* _IOR means that we're creating an ioctl command 
 * number for passing information from a user process
 * to the kernel module. 
 */


#endif