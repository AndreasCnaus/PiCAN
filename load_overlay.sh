#!/bin/bash

# dtc -@ -I dts -O dtb -o mcp2515.dtbo mcp2515.dts

# Ensure configfs is enabled
sudo mount -t configfs none /sys/kernel/config

#remove dir
sudo sudo rmdir /sys/kernel/config/device-tree/overlays/mcp2515

# Create a directory for the overlay
sudo mkdir /sys/kernel/config/device-tree/overlays/mcp2515

# Load the overlay
sudo cp /home/cnaus/programming/linux_driver/mcp2515_driver/build/dt/mcp2515.dtbo /sys/kernel/config/device-tree/overlays/mcp2515/dtbo

# Check for errors
dmesg | grep -i overlay

