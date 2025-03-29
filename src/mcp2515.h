#ifndef MCP2515_H
#define MCP2515_H

// macros
// #define BIT(n) (1U << (n))  // Define BIT(n) macro in not already defined
#define SET_BITS(register, mask) ((register) |= (mask))   // sets several bits in the given register
#define CLEAR_BITS(register, mask) ((register) &= ~(mask))  // clears several bits in the given register 

// MCP2515 Control Registers:
#define MCP2515_BFPCTRL     0x0C // Receive Buffer Pin Control Register (not used in this driver)
#define MCP2515_TXRTSCTRL   0x0D // Transmit Pin Control and Status Register (not used in this driver)
#define MCP2515_CANSTAT     0x0E // CAN Status Register
#define MCP2515_CANCTRL     0x0F // CAN Control Register
#define MCP2515_TEC         0x1C // Transmit Error Counter
#define MCP2515_REC         0x1D // Receive Error Counter
#define MCP2515_CNF3        0x28 // Configuration Register 3
#define MCP2515_CNF2        0x29 // Configuration Register 2
#define MCP2515_CNF1        0x2A // Configuration Register 1
#define MCP2515_CANINTE     0x2B // CAN Interrupt Enable Register
#define MCP2515_CANINTF     0x2C // CAN Interrupt Flag Register
#define MCP2515_EFLG        0x2D // Error Flag Register
#define MCP2515_TXB0CTRL    0x30 // Transmit Buffer 0 Control Register
#define MCP2515_TXB1CTRL    0x40 // Transmit Buffer 1 Control Register
#define MCP2515_TXB2CTRL    0x50 // Transmit Buffer 2 Control Register
#define MCP2515_RXB0CTRL    0x60 // Receive Buffer 0 Control Register
#define MCP2515_RXB1CTRL    0x70 // Receive Buffer 1 Control Register

// MCP2515 Tansmit Buffer n Standard Identifier Registers:
#define MCP2515_TXB0SIDH    0x31 // Transmit Buffer 0 Standard Indetifier Register high
#define MCP2515_TXB0SIDL    0x32 // Transmit Buffer 0 Standard Indetifier Register low 
#define MCP2515_TXB1SIDH    0x41 // Transmit Buffer 1 Standard Indetifier Register high
#define MCP2515_TXB1SIDL    0x42 // Transmit Buffer 1 Standard Indetifier Register low 
#define MCP2515_TXB2SIDH    0x51 // Transmit Buffer 2 Standard Indetifier Register high
#define MCP2515_TXB2SIDL    0x52 // Transmit Buffer 2 Standard Indetifier Register low 

// MCP2515 Transmit Buffer n Data Length Code Registers:
#define MCP2515_TXB0DLC     0x35 // Transmit Buffer 0 Data Length Code Register
#define MCP2515_TXB1DLC     0x45 // Transmit Buffer 1 Data Length Code Register
#define MCP2515_TXB2DLC     0x55 // Transmit Buffer 2 Data Length Code Register

// MCP2515 Transmit Buffer n Data Byte 0 Registers
#define MCP2515_TXB0D0      0x36 // Transmit Buffer 0 Data Byte 0
#define MCP2515_TXB1D0      0x46 // Transmit Buffer 1 Data Byte 0
#define MCP2515_TXB2D0      0x56 // Transmit Buffer 2 Data Byte 0

// MCP2515 Receive Buffer n Standard Identifier Register:
#define MCP2515_RXB0SIDH    0x61 // Receive Buffer 0 Standard Identifier Register high 
#define MCP2515_RXB0SIDL    0x62 // Receive Buffer 0 Standard Identifier Register low
#define MCP2515_RXB1SIDH    0x71 // Receive Buffer 1 Standard Identifier Register high
#define MCP2515_RXB1SIDL    0x72 // Receive Buffer 1 Standard Identifier Register low 

// MCP2515 Receive Buffer n Data Length Code Registers:
#define MCP2515_RXB0DLC     0x65 // Receive Buffer 0 Data Length Code Register
#define MCP2515_RXB1DLC     0x75 // Receive Buffer 1 Data Length Code Register

// MCP2515 Receive Buffer n Data Byte 0 Registers:
#define MCP2515_RXB0D0      0x66 // Receive Buffer 0 Data Byte 0
#define MCP2515_RXB1D0      0x76 // Receive Buffer 1 Data Byte 0

// MCP2515 Filter n Standard Identifier Registers:
#define MCP2515_RXF0SIDH        0x00 // Filter 0 Standard Identifier Register high
#define MCP2515_RXF0SIDL        0x01 // Filter 0 Standard Identifier Register low
#define MCP2515_RXF1SIDH        0x04 // Filter 1 Standard Identifier Register high
#define MCP2515_RXF1SIDL        0x05 // Filter 1 Standard Identifier Register low
#define MCP2515_RXF2SIDH        0x08 // Filter 2 Standard Identifier Register high
#define MCP2515_RXF2SIDL        0x09 // Filter 2 Standard Identifier Register low
#define MCP2515_RXF3SIDH        0x10 // Filter 3 Standard Identifier Register high
#define MCP2515_RXF3SIDL        0x11 // Filter 3 Standard Identifier Register low
#define MCP2515_RXF4SIDH        0x14 // Filter 4 Standard Identifier Register high
#define MCP2515_RXF4SIDL        0x15 // Filter 4 Standard Identifier Register low
#define MCP2515_RXF5SIDH        0x18 // Filter 5 Standard Identifier Register high
#define MCP2515_RXF5SIDL        0x19 // Filter 5 Standard Identifier Register low

// MCP2515 Mask n Standard Identifier Registers:
#define MCP2515_RXM0SIDH        0x20 // Mask 0 Standard Identfier Register high
#define MCP2515_RXM0SIDL        0x21 // Mask 0 Standard Identfier Register low
#define MCP2515_RXM1SIDH        0x24 // Mask 1 Standard Identfier Register high
#define MCP2515_RXM1SIDL        0x25 // Mask 1 Standard Identfier Register low

/* Bit mask and offset definitions: 
* Offset (_O): Specify the starting bit position of the field within the register.
*/
// CANSTAT Register
#define MCP2515_OPMOD2   BIT(7) // CANSTAT: OPMOD2 - Operation mode bit 2
#define MCP2515_OPMOD1   BIT(6) // CANSTAT: OPMOD1 - Operation mode bit 1
#define MCP2515_OPMOD0   BIT(5) // CANSTAT: OPMOD0 - Operation mode bit 0
#define MCP2515_ICOD2    BIT(3) // CANSTAT: ICOD2 - Interrupt code bit 2
#define MCP2515_ICOD1    BIT(2) // CANSTAT: ICOD1 - Interrupt code bit 1
#define MCP2515_ICOD0    BIT(1) // CANSTAT: ICOD0 - Interrupt code bit 0
#define MCP2515_OPMOD_M  (MCP2515_OPMOD2 |  MCP2515_OPMOD1 | MCP2515_OPMOD0) // operation mode bitmask 
#define MCP2515_IOCD_M   (MCP2515_ICOD2 | MCP2515_ICOD1 | MCP2515_ICOD0)     // interrupt code bitmask        

#define MCP2515_OPMOD_O  5      // Offset of the Operation Mode register
#define MCP2515_ICOD_O   1      // Offset of the Interrupt Code register

// CANCTRL Register
#define MCP2515_REQOP2   BIT(7) // CANCTRL: REQOP2 - Request operation mode bit 2
#define MCP2515_REQOP1   BIT(6) // CANCTRL: REQOP1 - Request operation mode bit 1
#define MCP2515_REQOP0   BIT(5) // CANCTRL: REQOP0 - Request operation mode bit 0
#define MCP2515_ABAT     BIT(4) // CANCTRL: ABAT - Abort all pending transmissions
#define MCP2515_OSM      BIT(3) // CANCTRL: OSM - One-shot mode
#define MCP2515_CLKEN    BIT(2) // CANCTRL: CLKEN - Clock output enable
#define MCP2515_CLKPRE1  BIT(1) // CANCTRL: CLKPRE1 - Clock output pin prescaler bit 1
#define MCP2515_CLKPRE0  BIT(0) // CANCTRL: CLKPRE0 - Clock output pin prescaler bit 0

#define MCP2515_REQOP_O  5      // Offset of the Request Operation Mode register
#define MCP2515_CLKPRE_O 0      // Offset of the Clock Output Pin Prescaler register

// CNF3 Register
#define MCP2515_SOF       BIT(7) // CNF3: SOF - Start-of-frame signal bit
#define MCP2515_WAKFIL    BIT(6) // CNF3: WAKFIL - Wake-up filter bit
#define MCP2515_PHSEG22   BIT(2) // CNF3: PHSEG22 - Phase segment 2 bit 2
#define MCP2515_PHSEG21   BIT(1) // CNF3: PHSEG21 - Phase segment 2 bit 1
#define MCP2515_PHSEG20   BIT(0) // CNF3: PHSEG20 - Phase segment 2 bit 0

#define MCP2515_PHSEG2_O  0      // Offset of the Phase Segment 2 register

// CNF2 Register
#define MCP2515_BTLMODE  BIT(7) // CNF2: BTLMODE - Bit timing mode
#define MCP2515_SAM      BIT(6) // CNF2: SAM - Sample point configuration
#define MCP2515_PHSEG12  BIT(5) // CNF2: PHSEG12 - Phase segment 1 bit 2
#define MCP2515_PHSEG11  BIT(4) // CNF2: PHSEG11 - Phase segment 1 bit 1
#define MCP2515_PHSEG10  BIT(3) // CNF2: PHSEG10 - Phase segment 1 bit 0
#define MCP2515_PRSEG2   BIT(2) // CNF2: PRSEG2 - Propagation segment bit 2
#define MCP2515_PRSEG1   BIT(1) // CNF2: PRSEG1 - Propagation segment bit 1
#define MCP2515_PRSEG0   BIT(0) // CNF2: PRSEG0 - Propagation segment bit 0

#define MCP2515_BTLMODE_O   7   // Offset of the Bit Timing mode bit 
#define MCP2515_SAM_O       6   // Offset of the sample point configuration bit 
#define MCP2515_PHSEG1_O    3   // Offset of the Phase Segment 1 register
#define MCP2515_PRSEG_O     0   // Offset of the Propagation Segment register

// CNF1 Register
#define MCP2515_SJW1    BIT(7) // CNF1: SJW1 - Synchronization jump width bit 1
#define MCP2515_SJW0    BIT(6) // CNF1: SJW0 - Synchronization jump width bit 0
#define MCP2515_BRP5    BIT(5) // CNF1: BRP5 - Baud rate prescaler bit 5
#define MCP2515_BRP4    BIT(4) // CNF1: BRP4 - Baud rate prescaler bit 4
#define MCP2515_BRP3    BIT(3) // CNF1: BRP3 - Baud rate prescaler bit 3
#define MCP2515_BRP2    BIT(2) // CNF1: BRP2 - Baud rate prescaler bit 2
#define MCP2515_BRP1    BIT(1) // CNF1: BRP1 - Baud rate prescaler bit 1
#define MCP2515_BRP0    BIT(0) // CNF1: BRP0 - Baud rate prescaler bit 0

#define MCP2515_SJW_M   (MCP2515_SJW0 | MCP2515_SJW1) // Synchronization Jump Width mask 
#define MCP2515_SJW_O    6     // Offset of the Synchronization Jump Width register
#define MCP2515_BRP_M   (MCP2515_BRP5 | MCP2515_BRP4 | MCP2515_BRP3 | MCP2515_BRP2 | MCP2515_BRP1 | MCP2515_BRP0)   // Baudrate Prescaler mask 
#define MCP2515_BRP_O    0     // Offset of the Baud Rate Prescaler register

// CANINTE Register
#define MCP2515_MERRE   BIT(7) // CANINTE: MERRE - Message error interrupt enable
#define MCP2515_WAKIE   BIT(6) // CANINTE: WAKIE - Wake-up interrupt enable
#define MCP2515_ERRIE   BIT(5) // CANINTE: ERRIE - Error interrupt enable
#define MCP2515_TX2IE   BIT(4) // CANINTE: TX2IE - Transmit buffer 2 interrupt enable
#define MCP2515_TX1IE   BIT(3) // CANINTE: TX1IE - Transmit buffer 1 interrupt enable
#define MCP2515_TX0IE   BIT(2) // CANINTE: TX0IE - Transmit buffer 0 interrupt enable
#define MCP2515_RX1IE   BIT(1) // CANINTE: RX1IE - Receive buffer 1 interrupt enable
#define MCP2515_RX0IE   BIT(0) // CANINTE: RX0IE - Receive buffer 0 interrupt enable

// CANINTF Register
#define MCP2515_MERRF   BIT(7) // CANINTF: MERRF - Message error interrupt flag
#define MCP2515_WAKIF   BIT(6) // CANINTF: WAKIF - Wake-up interrupt flag
#define MCP2515_ERRIF   BIT(5) // CANINTF: ERRIF - Error interrupt flag
#define MCP2515_TX2IF   BIT(4) // CANINTF: TX2IF - Transmit buffer 2 empty interrupt flag
#define MCP2515_TX1IF   BIT(3) // CANINTF: TX1IF - Transmit buffer 1 empty interrupt flag
#define MCP2515_TX0IF   BIT(2) // CANINTF: TX0IF - Transmit buffer 0 empty interrupt flag
#define MCP2515_RX1IF   BIT(1) // CANINTF: RX1IF - Receive buffer 1 full interrupt flag
#define MCP2515_RX0IF   BIT(0) // CANINTF: RX0IF - Receive buffer 0 full interrupt flag

// EFLG Register
#define MCP2515_RX1OVR  BIT(7) // EFLG: RX1OVR - Receive buffer 1 overflow flag
#define MCP2515_RX0OVR  BIT(6) // EFLG: RX0OVR - Receive buffer 0 overflow flag
#define MCP2515_TXBO    BIT(5) // EFLG: TXBO - Transmit buffer bus-off flag
#define MCP2515_TXEP    BIT(4) // EFLG: TXEP - Transmit error-passive flag
#define MCP2515_RXEP    BIT(3) // EFLG: RXEP - Receive error-passive flag
#define MCP2515_TXWAR   BIT(2) // EFLG: TXWAR - Transmit error warning flag
#define MCP2515_RXWAR   BIT(1) // EFLG: RXWAR - Receive error warning flag
#define MCP2515_EWARN   BIT(0) // EFLG: EWARN - Error warning flag

// TXB(n)CTRL Register
#define MCP2515_TXB_ABTF    BIT(6) // Message aborted flag bit ( 1 = Message was aborted)
#define MCP2515_TXB_MLOA    BIT(5) // Message lost arbitration bit (1 = Message lost arbitration while being sent)
#define MCP2515_TXB_TXERR   BIT(4) // Transmission error detected bit ( 1 = A bus error occurred while the message was being transmitted)
#define MCP2515_TXB_TXREQ   BIT(3) // Transmit request bit (1 = Buffer is currently pending transmission,  bit is automatically cleared )
#define MCP2515_TXB_TXP1    BIT(1) // Transmit buffer priority bit 1
#define MCP2515_TXB_TXP0    BIT(0) // Transmit buffer priority bit 0

// TXB(n)SIDL Register
#define MCP2515_TXB_SIDL2   BIT(7) // Transmit Buffer Standard Identifier low register, bit 2
#define MCP2515_TXB_SIDL1   BIT(6) // Transmit Buffer Standard Identifier low register, bit 1
#define MCP2515_TXB_SIDL0   BIT(5) // Transmit Buffer Standard Identifier low register, bit 0
#define MCP2515_TXB_EXIDE   BIT(3) // Transmit Buffer Extended Identifier bit (0 = Message will transmit Standard Identifier, 1 = Message will transmit Extended Identifier)

#define MCP2515_TXB_SIDL_M  (MCP2515_TXB_SIDL2 | MCP2515_TXB_SIDL1 | MCP2515_TXB_SIDL0)   // Transmit Buffer: Standard Identifier low register bitmask 
#define MCP2515_TXB_SIDL_O  5      // Transmit buffer: Standard Identifier low register offset 

// TXB(n)DLC
#define MCP2515_TXB_RTR     BIT(6) // Remote Transmition Request bit (0 = data frame, 1 = remote transmit request)
#define MCP2515_TXB_DLC_M   (BIT(3) | BIT(2) | BIT(1) | BIT(0)) // Data Length Code bitmask 

// RXB0CTRL Register
#define MCP2515_RXB0_RXM1    BIT(6) // Receive buffer 0 operating mode bit 1
#define MCP2515_RXB0_RXM0    BIT(5) // Receive buffer 0 operating mode bit 0
#define MCP2515_RXB0_RXRTR   BIT(3) // Received Remote Transfer Request bit (1 = Remote Transfer Request received)
#define MCP2515_RXB0_BUKT    BIT(2) // Rollover Enable bit (0 = Rollover is disabled, 1 = RXB0 message will roll over and be written to RXB1 if RXB0 is full)
#define MCP2515_RXB0_FILHIT0 BIT(0) // Filter Hit bit (indicates which acceptance filter enabled reception of message), (0 = Acceptance Filter 0 (RXF0), 1 = Acceptance Filter 1 (RXF1))

#define MCP2515_RXB0_RXM_O   5      // Offset of the Receive Buffer 0 Operating Mode register

// RXB1CTRL Register
#define MCP2515_RXB1_RXM1    BIT(6) // Receive buffer 1 operating mode bit 1
#define MCP2515_RXB1_RXM0    BIT(5) // Receive buffer 1 operating mode bit 0
#define MCP2515_RXB1_RXRTR   BIT(3) // Received remote transfer request bit (1 = Remote Transfer Request received)
#define MCP2515_RXB1_FILHIT2 BIT(2) // Filter hit bit 2, indicates which acceptance filter enabled reception of message
#define MCP2515_RXB1_FILHIT1 BIT(1) // Filter hit bit 1, indicates which acceptance filter enabled reception of message
#define MCP2515_RXB1_FILHIT0 BIT(0) // Filter hit bit 0, indicates which acceptance filter enabled reception of message

#define MCP2515_RXB1_RXM_O   5      // Offset of the Receive Buffer 1 Operating Mode register

// RXB(n)SIDL Register
#define MCP2515_RXB_SID2    BIT(7) // Receive Buffer Standard Identfier low register, bit 2
#define MCP2515_RXB_SID1    BIT(6) // Receive Buffer Standard Identfier low register, bit 1
#define MCP2515_RXB_SID0    BIT(5) // Receive Buffer Standard Identfier low register, bit 0
#define MCP2515_RXB_SRR     BIT(4) // Receive Buffer Standard Frame Remote Transmit Request bit (0 = Standard frame, 1 = remote transmition request)
#define MCP2515_RXB_IDE     BIT(3) // Receive Buffer Extended Identifier Flag (0 = Standard frame received , 1 = Extended frame received)

#define MCP2515_RXB_SID_M   (MCP2515_RXB_SID2 | MCP2515_RXB_SID1 | MCP2515_RXB_SID0)    // Receive buffer: Standard Identifier low register bitmask
#define MCP2515_RXB_SID_O   5      // Receive Buffer: Standard Identifier low register offset 

// RXB(n)DLC
#define MCP2515_RXB_DLC_M   (BIT(3) | BIT(2) | BIT(1) | BIT(0)) // Data Length Code bitmask 

// RXF(n)SIDL Register 
#define MCP2515_RXF_SID2    BIT(7) // Receive Filter: Standard Identifier low register, bit 2 
#define MCP2515_RXF_SID1    BIT(6) // Receive Filter: Standard Identifier low register, bit 1 
#define MCP2515_RXF_SID0    BIT(5) // Receive Filter: Standard Identifier low register, bit 0
#define MCP2515_RXF_EXIDE   BIT(3) // Receive Filter: Extended Identifier Enable bit ( 0 = Filter is applied only to standard frames, 1 = only to extended frames)

#define MCP2515_RXF_SID_M   (MCP2515_RXF_SID2 | MCP2515_RXF_SID1 | MCP2515_RXF_SID0)    // Receive Filter: Standard Identfier low register bitmask 
#define MCP2515_RXF_SID_O   5      // Receive Filter: Standard Identifier register offset

// RXM(n)SIDL Register
#define MCP2515_RXM_SID2    BIT(7) // Receive Mask: Standard Identifier low register, bit 2 
#define MCP2515_RXM_SID1    BIT(6) // Receive Mask: Standard Identifier low register, bit 1 
#define MCP2515_RXM_SID0    BIT(5) // Receive Mask: Standard Identifier low register, bit 0

#define MCP2515_RXM_SID_M   (MCP2515_RXM_SID2 | MCP2515_RXM_SID1 | MCP2515_RXM_SID0)    // Receive Mask: Standard Identfier low register bitmask
#define MCP2515_RXM_SID_O   5      // Receive Mask: Standard Identifier register offset 

// Value Definitions
// TXP[1:0] Transmit Buffer Priority
#define MCP2515_TXP_H_PRIO          0x03 // 11: Highest message priority
#define MCP2515_TXP_HI_PRIO         0x02 // 10: High intermediate message priority
#define MCP2515_TXP_LI_PRIO         0x01 // 01: Low intermediate message priority
#define MCP2515_TXP_L_PRIO          0x00 // 00: Lowest message priority

// RXM[1:0] Receive Buffer Operating mode bits
#define MCP2515_RXM_RCV_ANY_MSG     0x03 // 11: Turns mask/filters off; receives any message
#define MCP2515_RXM_RCV_VALID_MSG   0x00 // 00: Receives all valid messages using either Standard or Extended Identifiers that meet filter criteria

// RXB1CTRL:FILHIT[2:0]Filter Hit bits (indicates which acceptance filter enabled reception of message)
#define MCP2515_FILHIT_5            0x05 // 101: Acceptance Filter 5 (RXF5)
#define MCP2515_FILHIT_4            0x04 // 100: Acceptance Filter 4 (RXF4)
#define MCP2515_FILHIT_3            0x03 // 011: Acceptance Filter 3 (RXF3)
#define MCP2515_FILHIT_2            0x02 // 010: Acceptance Filter 2 (RXF2)
#define MCP2515_FILHIT_1            0x01 // 001: Acceptance Filter 1 (RXF1) (only if the BUKT bit is set in RXB0CTRL)
#define MCP2515_FILHIT_0            0x00 // 000: Acceptance Filter 0 (RXF0) (only if the BUKT bit is set in RXB0CTRL)

// SJW[1:0] Synchronization Jump Width Length bits
#define MCP2515_SJW_4TQ             0x03 // 11: Synchronization Jump Width Length = 4TQ
#define MCP2515_SJW_3TQ             0x02 // 10: Synchronization Jump Width Length = 3TQ
#define MCP2515_SJW_2TQ             0x01 // 01: Synchronization Jump Width Length = 2TQ
#define MCP2515_SJW_1TQ             0x00 // 00: Synchronization Jump Width Length = 1TQ

// ICOD[2:0] Interrupt Flag Code bits
#define MCP2515_NO_INTERRUPT        0x00 // 000: No interrupt
#define MCP2515_ERROR_INTERRUPT     0x01 // 001: Error interrupt
#define MCP2515_WAKEUP_INTERRUPT    0x02 // 010: Wake-up interrupt
#define MCP2515_TXB0_INTERRUPT      0x03 // 011: TXB0 interrupt
#define MCP2515_TXB1_INTERRUPT      0x04 // 100: TXB1 interrupt
#define MCP2515_TXB2_INTERRUPT      0x05 // 101: TXB2 interrupt
#define MCP2515_RXB0_INTERRUPT      0x06 // 110: RXB0 interrupt
#define MCP2515_RXB1_INTERRUPT      0x07 // 111: RXB1 interrupt

// SPI Instruction Set:
// WRITE command
#define MCP2515_WRITE           0x02 // 0000 0010: Writes data to the register beginning at the selected address
// READ command
#define MCP2515_READ            0x03 // 0000 0011: Reads data from the register beginning at selected address
// BIT MODIFY command
#define MCP2515_BIT_MODIFY      0x05 // 0000 0101: Allows the user to set or clear individual bits in a particular register
// READ STATUS command
#define MCP2515_READ_STATUS     0xA0 // 1010 0000: Quick polling command that reads several status bits for transmit and receive functions
// RX STATUS command
#define MCP2515_RX_STATUS       0xB0 // 1011 0000: Quick polling command that indicates filter match and message type of received message
// RESET command
#define MCP2515_RESET           0xC0 // 1100 0000: Resets internal registers to the default state, sets Configuration mode
// LOAD TX BUFFER command
#define MCP2515_LOAD_TXB0_SIDH  0x40 // 0100 0000: TX Buffer 0, Start at TXB0SIDH (0x31)
#define MCP2515_LOAD_TXB0_D0    0x41 // 0100 0001: TX Buffer 0, Start at TXB0D0 (0x36)
#define MCP2515_LOAD_TXB1_SIDH  0x42 // 0100 0010: TX Buffer 1, Start at TXB1SIDH (0x41)
#define MCP2515_LOAD_TXB1_D0    0x43 // 0100 0011: TX Buffer 1, Start at TXB1D0 (0x46)
#define MCP2515_LOAD_TXB2_SIDH  0x44 // 0100 0100: TX Buffer 2, Start at TXB2SIDH (0x51)
#define MCP2515_LOAD_TXB2_D0    0x45 // 0100 0101: TX Buffer 2, Start at TXB2D0 (0x56)
// READ RX BUFFER command
#define MCP2515_READ_RXB0_SIDH  0x90 // 1001 0000: Receive Buffer 0, Start at RXB0SIDH (0x61)
#define MCP2515_READ_RXB0_D0    0x92 // 1001 0010: Receive Buffer 0, Start at RXB0D0 (0x66)
#define MCP2515_READ_RXB1_SIDH  0x94 // 1001 0100: Receive Buffer 1, Start at RXB1SIDH (0x71)
#define MCP2515_READ_RXB1_D0    0x96 // 1001 0110: Receive Buffer 1, Start at RXB1D0 (0x76)
// RTS (Request-to-Send) command
#define MCP2515_RTS_TXB0        0x81 // Request-to-Send TX Buffer 0
#define MCP2515_RTS_TXB1        0x82 // Request-to-Send TX Buffer 1
#define MCP2515_RTS_TXB2        0x84 // Request-to-Send TX Buffer 2
#define MCP2515_RTS_ALL         0x87 // Request-to-Send All TX Buffers (0, 1, and 2)

#endif /* MCP2515_H*/
