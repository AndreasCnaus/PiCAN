    
# PiCAN
This repository consists of custom Linux Device Driver for MCP2515 CAN-chip as well as the user-space configuration- and test tool (see also [test](/PiCAN/test/)).

### Driver Components

| **Component**          | **Description**                                                                 |
|-------------------------|---------------------------------------------------------------------------------|
| **Character Device**    | Exposes `/dev/mcp2515` for custom CAN frame read/write via `read()`/`write()`.      |
| **IOCTL Interface**     | Supports device configuration, such as:<br>- Resetting the device<br>- Getting and setting the operation mode<br>- Managing RX filters.    |
| **Ring Buffers**        | Custom TX/RX ring buffers for data synchronization and access between user space and the MCP2515 internal buffers.  |
| **Mutex/Spinlocks**     | Synchronizes access to shared resources (Ring buffers, SPI).                                       |
| **Interrupt Handling**  | Handles MCP2515 interrupts (RX/TX, errors) via GPIO IRQ.                        |
| **Platform Driver**     | Binds to platform devices (Device Tree) for resource management.                |
| **SPI Subsystem**       | Manages SPI communication with the MCP2515 CAN controller chip.                 |

### Future Development Opportunities

⚙️ **Linux CAN Stack (SocketCAN)**:  
This driver currently uses a custom character device. Future enhancements could include integration with the Linux CAN framework (e.g., `can0`), enabling compatibility with tools like `candump`/`cansend`.  

⚙️ **DMA Support**:  
The driver uses custom ring buffers for TX/RX synchronization. Moving towards **DMA** integration (via Linux kernel APIs like `dmaengine`) could reduce CPU usage. This would involve utilizing the kernel’s DMA subsystem.
