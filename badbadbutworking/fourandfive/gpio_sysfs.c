/**
 * @file   gpio_test.c
 * @author Derek Molloy
 * @date   19 April 2015
 * @brief  A kernel module for controlling a GPIO LED/button pair. The device mounts devices via
 * sysfs /sys/class/gpio/gpio115 and gpio49. Therefore, this test LKM circuit assumes that an LED
 * is attached to GPIO 49 which is on P9_23 and the button is attached to GPIO 115 on P9_27. There
 * is no requirement for a custom overlay, as the pins are in their default mux mode states.
 * @see http://www.derekmolloy.ie/
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/time.h>       // Using the clock to measure time between button presses


#include "beaglebone-gpio.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yoseph Maguire");
MODULE_DESCRIPTION("A Button GPIO LKM for BBB");
MODULE_VERSION("0.1");

static unsigned int gpioButton = 20;   ///< hard coding the button gpio for this example to P9_27 (GPIO115)
module_param(gpioButton, uint, S_IRUGO);
MODULE_PARM_DESC(gpioButton, " GPIO wire number (default=20)");





static char   gpioName[8] = "gpioXXX";      ///< Null terminated default string -- just in case
static unsigned int irqNumber;          ///< Used to share the IRQ number within this file
static bool   isMeasure = 1;               ///< Use to store the debounce state (on by default)
static struct timespec tic, toc, timediff;





/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);







/** @brief Displays if button debouncing is on or off */
static ssize_t diffTime_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%lu\n", timediff.tv_nsec);
}

/* Displays if measure capacitance is on or off */
static ssize_t touch_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
    printk(KERN_INFO "GPIO_LKM: Hello from ELEC424_SHOW. \n");
    return sprintf(buf, "%d\n", timediff.tv_nsec);
}
 
/** @brief Stores and sets the debounce state */
static ssize_t touch_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   unsigned int temp;
   printk(KERN_INFO "GPIO_LKM: Hello from ELEC424_STORE. \n");
   sscanf(buf, "%du", &temp);                // use a temp varable for correct int->bool
   printk(KERN_INFO "GPIO_LKM: Set GPIO_DEBOUNCE to 0 \n");
   isMeasure = temp;
   gpio_direction_output(gpioButton, 1); // set pin to output
   printk(KERN_INFO "GPIO_LKM: gpioButton set to: %d. \n", gpio_get_value(gpioButton)); 
   getrawmonotonic(&tic);
   printk(KERN_INFO "EBB Button: Capacitance measured.\n");
   gpio_direction_input(gpioButton);
   return count;
}

/**  Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The count variable is associated with the numberPresses variable and it is to be exposed
 *  with mode 0666 using the numberPresses_show and numberPresses_store functions above
 */
static struct kobj_attribute touch_attr = __ATTR(touch, S_IWUSR | S_IRUGO, touch_show, touch_store);


/**  The __ATTR_RO macro defines a read-only attribute. There is no need to identify that the
 *  function is called _show, but it must be present. __ATTR_WO can be  used for a write-only
 *  attribute but only in Linux 3.11.x on.
 */
static struct kobj_attribute diff_attr  = __ATTR_RO(diffTime);  ///< the difference in time attr
 

static struct attribute *ebb_attrs[] = {
      &diff_attr.attr,                   ///< The difference in time between the last two presses
      &touch_attr.attr,               ///< Is the debounce state true or false
      NULL,
};

static struct attribute_group attr_group = {
    .name = gpioName,
    .attrs = ebb_attrs,
};

static struct kobject *ebb_kobj;


/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init ebbgpio_init(void){
   int result = 0;
   printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");
   sprintf(gpioName, "gpio%d", gpioButton); // create the gpio20 name



   // Going to set up the LED. It is a GPIO in output mode and will be on by default
   gpio_request(gpioButton, "sysfs");       // Set up the gpioButton
   gpio_direction_input(gpioButton);        // Set the button GPIO to be an input
   gpio_export(gpioButton, false);          // Causes gpio115 to appear in /sys/class/gpio
			                    // the bool argument prevents the direction from being changed
   // Perform a quick test to see that the button is working as expected on LKM load
   printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));

   // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   irqNumber = gpio_to_irq(gpioButton);
   printk(KERN_INFO "GPIO_TEST: The button is mapped to IRQ: %d\n", irqNumber);

   // This next call requests an interrupt line
   result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) ebbgpio_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_LOW,   // Interrupt on rising edge (button press, not release)
                        "ebb_gpio_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

   printk(KERN_INFO "GPIO_TEST: The interrupt request result is: %d\n", result);
  
    
    ebb_kobj = kobject_create_and_add("elec424", kernel_kobj->parent);
    if (!ebb_kobj){
        printk(KERN_ALERT "EBB BUTTON: FAILED to create kobject mapping \n");
        return -ENOMEM;
    }
    result = sysfs_create_group(ebb_kobj, &attr_group);
    if(result) {
        printk(KERN_ALERT "EBB Button: failed to create sysfs group\n");
        kobject_put(ebb_kobj);
        return result;
    }
    printk(KERN_INFO "GPIO_LKM: device kobject and sysfs created correctly\n");


    return result;

}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required. Used to release the
 *  GPIOs and display cleanup messages.
 */
static void __exit ebbgpio_exit(void){
    printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));
    kobject_put(ebb_kobj);
    free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
    gpio_unexport(gpioButton);               // Unexport the Button GPIO
    gpio_free(gpioButton);                   // Free the Button GPIO
    printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");

}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
    getrawmonotonic(&toc);
    // printk(KERN_INFO "GPIO_LKM: toc (%ld) \n", toc.tv_nsec);
    timediff = timespec_sub(toc, tic);
    // printk(KERN_INFO "GPIO_TEST: timediff (%ld) \n", timediff.tv_nsec);
    gpio_direction_output(gpioButton, 1);
    // printk(KERN_INFO "GPIO_TEST: GPIO_OUT, set to: %d \n", gpio_get_value(gpioButton));
    return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}



/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(ebbgpio_init);
module_exit(ebbgpio_exit);
