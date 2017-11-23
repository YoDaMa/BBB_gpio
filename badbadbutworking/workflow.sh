#!/bin/bash
echo "Automated making and moving to USB..."
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
sudo mv gpio_simple.ko /media/yoseph/rootfs/elec424/
sudo mv test /media/yoseph/rootfs/elec424/
