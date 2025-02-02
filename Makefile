# Makefile for building the MCP2515 kernel module

# Module name
obj-m += mcp2515.o

# Source files
mcp2515-objs := src/mcp2515.o src/mcp2515_circular_buffer.o

# Kernel build directory
KDIR := /lib/modules/$(shell uname -r)/build

# Current directory
PWD := $(shell pwd)

# Custom build directory
BUILDDIR := $(PWD)/build

# Compiler flags to include common headers
EXTRA_CFLAGS := -I$(PWD)/common

.PHONY: all clean modules dtbo clean_dt clean_all

# Default target: build the module
all: modules

# Build the kernel module
modules:
	mkdir -p $(BUILDDIR)
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	find $(PWD) \( -path $(BUILDDIR) -prune \) -o \( -name '*.o' -o -name '*.ko' -o -name '*.mod' -o -name '*.mod.c' -o -name '*.cmd' -o -name '*.symvers' -o -name '*.order' \) -exec mv {} $(BUILDDIR) \;

# Compile the Device Tree Overlay source into a binary with the symbol table
dtbo:
	mkdir -p $(BUILDDIR)/dt
	dtc -@ -I dts -O dtb -o $(BUILDDIR)/dt/mcp2515.dtbo dt/mcp2515.dts

# Clean target: remove all files except those with .dtbo extension in the build directory
clean:
	find $(BUILDDIR) -type f ! -name '*.dtbo' -exec rm -f {} +

# Clean Device Tree binaries only
clean_dt:
	rm -rf $(BUILDDIR)/dt

# Clean all files and directories, including kernel module files
clean_all:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -rf $(BUILDDIR)
