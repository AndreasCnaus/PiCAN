# MCP2515 Device Driver Test Tool

This repository contains a test tool for testing the MCP2515 device driver on the Raspberry Pi 4. 
The test tool ensures that the MCP2515 CAN controller and device driver are working properly.

![Test Verification](https://github.com/AndreasCnaus/PiCAN_Sense/blob/master/docs/mcp2515_test_verification.png)

## Features

- Write CAN-Message [SID:Data] on the CAN-Bus (or to the TX->RX-Buffer in loopback mode)
- Read CAN-Message [SID:Data] from the RX-Buffer
- Configure the MCP2515-Chip (Reset-Chip, Set the Receive-Filter 1..5, Set the Operation Mode) via IOCTL-Interface : **not implemented yet**.

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
## Usage

1. Run the test tool in write mode to create CAN-Message and put it on the CAN-Bus(or to the TX->RX-Buffer in loopback mode):

    ```sh
    ./mcp2515_test write
    ```
2.  Run the test tool in read mode to read CAN-Message if any from the MCP2515 Receive-Buffer:

    ```sh
    ./mcp2515_test read
    ```
    
## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
