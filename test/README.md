# MCP2515 Device Driver Configuration and Test Tool

This repository contains a command-line tool for configuring and testing the MCP2515 device driver on the Raspberry Pi 4. The test tool ensures that the MCP2515 CAN controller and device driver are working properly.

## Features
- **Configuration of MCP2515-Chip (via IOCTL-Interface)**:
  - Reset-Chip
  - Get/Set the Receive-Filter 2..5
  - Get/Set the Operation Mode
- **Data Transmission**: Write CAN-Message [SID:Data] on the CAN-Bus (or to the TX->RX-Buffer in loopback mode)
- **Data Reception**: Read CAN-Message [SID:Data] from the RX-Buffer

## Usage

### Configuration Mode
Run the tool in configuration mode to set up the MCP2515-Chip:

```sh
./mcp2515_test config
```
![Configuration](../docs/mcp2515_configuration.png)

### Read/Write Mode
Run the tool in read or write mode to transmit or receive CAN-Messages:

```sh
./mcp2515_test read
```

```sh
./mcp2515_test write
```

![Test Verification](../docs/mcp2515_test_verification.png)

## Installation

1. Clone the repository:

    ```sh
    git clone https://github.com/AndreasCnaus/PiCAN_Sense.git
    ```

2. Navigate to the test tool directory:

    ```sh
    cd PiCAN_Sense/test
    ```

3. Build the test tool:

    ```sh
    make
    ```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
