# A note on lab 4 implementation

In this folder I have the (non-working) code for parts 2-3 to be able to read the gpio pin and measure capacitance to see if touch has occurred... 

For some reason the code did not work. Because I did not have enough time to continue debugging, I decided to implement it using gpio.h (included in the badbadbutworking folder), so that I could do part 4 and 5 as well. The code in here represents an attempt to write a LKM that writes to Kernel Virtual Memory in order to manipulate the memory-mapped GPIO pins on the Beaglebone. 

If time permits I will continue debugging to finish this implementation. 


