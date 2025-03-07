    
# PiCAN_Sense Project

The **PiCAN_Sense** project is a system built on a Raspberry Pi 4 running the Raspbian GNU/Linux 11 operating system. Its primary function is to read environmental values, such as temperature, pressure, and humidity, from a BME280 sensor via a CAN (Controller Area Network) bus. The system uses a self-written CAN driver for the MCP2515 chip, making these readings available to user-space applications for further processing and analysis.

## System Overview:
           +----------------------------------+
           |     Raspberry Pi 4 / Linux       |
           |  +----------------------------+  |
           |  | Test Tool (User Space)     |  |
           |  |  using MCP2515 Driver      |  |
           |  +----------------------------+  |
           |                | (sys_calls)     |
           |  +----------------------------+  |
           |  | MCP2515 Driver             |  |
           |  +----------------------------+  |
           +---------------+------------------+
                           ^
                           |
                           | (SPI0)
                           |
                           v
           +-------------------------------+
           |      MCP2515 CAN-Shield(1)    |
           +-------------------------------+
                           ^
                           |
                           | (CAN-Bus)
                           |
                           v
           +-------------------------------+
           |      MCP2515 CAN-Shield(2)    |
           +-------------------------------+
                           ^
                           |
                           | (SPI)
                           |
                           v
           +-------------------------------+
           |       RF Breakout Board       |           +-------------------+
           |                               |           |                   |
           |   +-------------------------+ | <--I2C--> +   BME280-Sensor   |
           |   |        STM32WB          | |           |                   |	
           |   +-------------------------+ |           +-------------------+
           +-------------------------------+
           

The PiCAN_Sense system architecture is organized as follows:

1. **Raspberry Pi 4**:
    - **Operating System**: Raspbian GNU/Linux 11 with kernel 6.1.21
    - **Key Software Components**:
        - **Test Tool (User Space)**: A user-space application that interacts with the MCP2515 driver to receive and transmit CAN-Messages.
        - **MCP2515 Driver**: A device driver that facilitates communication with the MCP2515-Chip on the CAN-Shield via SPI0 interface and exposes read/write/ioctl interfaces to the user space by means of a **Character-Device**.

2. **MCP2515 CAN-Shield**:
    - **First CAN-Shield**:
        - Acts as an intermediary between the Raspberry Pi 4 and the CAN-Bus network.
        - Transmits data between CAN-Bus and Raspbbery Pi vis SPI interface.
    - **Second CAN-Shield**:
        - Acts as an intermediary between the RF Breakout Board and the CAN-Bus network.
        -  Transmits data between CAN-Bus and RF Breakout Board via SPI interface.

3. **RF Breakout Board** (see laso: https://github.com/AndreasCnaus/BLE_ESB/tree/main/hardware):
    - **Key Components**:
        - **STM32WB Microcontroller**: Reads BME280 Sensor data via the I2C interface and puts them on the CAN Bus via the CAN-Shield.
        - **BME280 Sensor**: An environmental sensor connected via the I2C bus, measuring temperature, humidity, and pressure.

## Project/Feature status:

![Test Setup](https://github.com/AndreasCnaus/PiCAN_Sense/blob/master/docs/PiCAN_Sense_test_setup.jpg)

-  MCP2515 Driver for STM23WB Microcontroller: **still needs to be implemented**.
- Documentation of implementation and usage: **still needs to be implemented**.
  
## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
