#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <asm/uaccess.h>          // Required for the copy to user function
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include "beaglebone-gpio.h"

#define  DEVICE_NAME "gpio424"    ///< The device will appear at /dev/gpio424 using this value
#define  CLASS_NAME  "elec"        ///< The device class -- this is a character device driver


MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Yoseph Maguire");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("11/20/2017 Trial");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static struct class*  ebbcharClass  = NULL; ///< The device-driver class struct pointer
static struct device* ebbcharDevice = NULL; ///< The device-driver device struct pointer
static unsigned int irqNumber;
static int gpioPIN = 115;                   // P9_27 | GPIO3_19
static void __iomem *gpio_map;
static void *mmio;

static irq_handler_t gpio424_irq_handler(unsigned int irt, void *dev_id, struct pt_regs *regs);

static unsigned int ccrInit = 1;            // used as a flag for the performance counter

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static long dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static long    getPerfCounter(void);
static void custom_set_gpio_direction(int gpio, int is_input);
static void custom_set_gpio_dataout_reg(unsigned offset, int enable);
static int custom_get_gpio_datain(int offset);
static int custom_get_gpio_dataout(int offset);





static long      capacitance; // calculated capacitance
static struct timespec tic, toc, timediff; // used for timer measurements


/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .release = dev_release,
   .unlocked_ioctl = dev_ioctl
};

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init gpio_init(void){
    int result = 0;
    printk(KERN_INFO "EBBChar: Initializing the EBBChar LKM\n");

    // Try to dynamically allocate a major number for the device -- more difficult but worth it
    majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
    if (majorNumber<0){
        printk(KERN_ALERT "EBBChar failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "EBBChar: registered correctly with major number %d\n", majorNumber);

    // Register the device class
    ebbcharClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ebbcharClass)){                // Check for error and clean up if there is
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(ebbcharClass);          // Correct way to return an error on a pointer
    }
    printk(KERN_INFO "EBBChar: device class registered correctly\n");

    // Register the device driver
    ebbcharDevice = device_create(ebbcharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ebbcharDevice)){               // Clean up if there is an error
        class_destroy(ebbcharClass);           // Repeated code but the alternative is goto statements
        unregister_chrdev(majorNumber, DEVICE_NAME);
        printk(KERN_ALERT "Failed to create the device\n");
        return PTR_ERR(ebbcharDevice);
    }
    printk(KERN_INFO "EBBChar: device class created correctly\n"); // Made it! device was initialized

    printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");
    // Is the GPIO a valid GPIO number (e.g., the BBB has 4x32 but not all available)
    if (!gpio_is_valid(gpioPIN)){
        printk(KERN_INFO "GPIO_TEST: invalid LED GPIO\n");
        return -ENODEV;
    }

    // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
    irqNumber = gpio_to_irq(gpioPIN);
    printk(KERN_INFO "GPIO_TEST: The wire is mapped to IRQ: %d\n", irqNumber);

    // This next call requests an interrupt line
    result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) gpio424_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_FALLING,   // Interrupt on rising edge (button press, not release)
                        "elec424_gpio_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

    printk(KERN_INFO "GPIO_TEST: The interrupt request result is: %d\n", result);

    gpio_map = ioremap(GPIO3, 0x1000);
    mmio = ioremap(GPIO3, 0x1000);
    printk(KERN_INFO "GPIO_TEST: IOREMAP Maps GPIO3 to: %p", mmio);


    return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit gpio_exit(void){
    device_destroy(ebbcharClass, MKDEV(majorNumber, 0));     // remove the device
    class_unregister(ebbcharClass);                          // unregister the device class
    class_destroy(ebbcharClass);                             // remove the device class
    unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
    printk(KERN_INFO "EBBChar: Goodbye from the LKM!\n");
    free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
    printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "EBBChar: Device has been opened\n");
   return 0;
}


/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "EBBChar: Device successfully closed\n");
   return 0;
}


static long dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case IOCTL_GET_VALUE:
        {
            printk(KERN_INFO "GPIOTEST: Hello from MYGPIO_GETVALUE\n");
            
            capacitance = timediff.tv_nsec;
            if (copy_to_user((long *) arg, &capacitance, sizeof(long))) {
                printk(KERN_ALERT "GPIOTEST: copy_to_user failed\n");
		        return -EFAULT; 
            }
            printk(KERN_INFO "GPIOTEST: copy_to_user succeeded.\n");
            return capacitance;
        }
        break;

        case IOCTL_MEASURE_CAPACITANCE:
        {
            printk(KERN_INFO "GPIOTEST: Hello from MYGPIO_MEASURE_CAPACITANCE\n");
            custom_set_gpio_direction(PIN_19, 0); // set pin to output
            custom_set_gpio_dataout_reg(PIN_19, 1); // set pin to high
            // THIS IS WHERE THE ERROR IS HAPPENING
            if (!custom_get_gpio_dataout(PIN_19)) {   // gpio is not set to high
                printk(KERN_INFO "GPIOTEST: Dataout register not set to high...\n");
                return -EFAULT;
            }
            getnstimeofday(&tic);
            custom_set_gpio_direction(PIN_19, 1); // set pin to input
        }
        break;
    }
    return 0;
}

static long getPerfCounter(void) {
    long value;
    if (ccrInit) {
        // Enable user mode acces to the performance counter
        asm ("MCR p15, 0, %0, c9, c14, 0\t\n" :: "r" (1));
        // Disable counter overflow intterupts (just in case)
        asm ("MCR p15, 0, %0, c9, c14, 2\t\n" :: "r" (0x8000000f));
        ccrInit = !ccrInit;
    }
    printk(KERN_INFO "User-level acces to CCR has been turned on.\n"); // long long int
    // Read CCNT Register
    asm volatile ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(value));  
    printk(KERN_INFO "Read CCNT Register Successfully \n");
    return value;
}

static irq_handler_t gpio424_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
    getnstimeofday(&toc);
    timediff = timespec_sub(toc,tic); 
    custom_set_gpio_direction(PIN_19, 0);  // set GPIO to output
    custom_set_gpio_dataout_reg(PIN_19, 1); // set GPIO to high
    printk(KERN_INFO "GPIO_TEST: Interrupt!");
    return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

/* =======================================================================
 * START OF GPIO CODE
 */

static void custom_set_gpio_direction(int gpio, int is_input)
{
	// hard code the base value...
	void __iomem *reg = (void *) (long) gpio_map + GPIO_OE; // this is correct base 
	u32 l;
    l = readl_relaxed(reg);
	if (is_input)
		l |= BIT(gpio);
	else
		l &= ~(BIT(gpio));
	writel_relaxed(l, reg);
}

static void custom_set_gpio_dataout_reg(unsigned offset, int enable)
{
    void __iomem *reg = (void *) (long) gpio_map + GPIO_DATAOUT;
	u32 gpio_bit = BIT(offset);
	u32 l;

	l = readl_relaxed(reg);
	if (enable)
		l |= gpio_bit;
	else
		l &= ~gpio_bit;
	writel_relaxed(l, reg);
}

static int custom_get_gpio_datain(int offset)
{
	void __iomem *reg = (void *) (long) gpio_map + GPIO_DATAIN;
	return (readl_relaxed(reg) & (BIT(offset))) != 0;
}

static int custom_get_gpio_dataout(int offset)
{
	void __iomem *reg = (void *) (long) gpio_map + GPIO_DATAOUT;
    // printk(KERN_INFO "CUSTOM_DATAOUT: The register is ");
	return (readl_relaxed(reg) & (BIT(offset))) != 0;
}


/* =======================================================================
 * END OF GPIO CODE
 */

module_init(gpio_init);
module_exit(gpio_exit);
