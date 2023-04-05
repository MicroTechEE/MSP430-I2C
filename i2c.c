/*
 * i2c.c
 *
 *  Created with masterful finesse on: Apr 3, 2023 by
 *      Author: Tom Bailey
 */

/*- - - -  #INCLUDES - - - -*/
#include "i2c.h"

/*- - - -  GLOBALS - - - -*/
I2C_Struct i2c;
unsigned char rxJunk, i2cPtrOffset;           // junk is just to fix RX handling if SCL gets stuck LOW, offset is for data ptr




/*- - - -  FUNCTIONS - - - -*/

I2C_Struct* i2c_init(void){
    // Initialize i2c functionality on boot and re-initialize in case of failure

    // Because this function is also used to reset i2c module, misconfigure, toggle, then reconfigure GPIO
    I2C_PORT_SEL0_REG &= ~(I2C_SDA_PIN|I2C_SCL_PIN);
    I2C_PORT_SEL1_REG &= ~(I2C_SDA_PIN|I2C_SCL_PIN);
    I2C_SDA_OUT_REG |= I2C_SDA_PIN;
    I2C_SCL_OUT_REG |= I2C_SCL_PIN;
    delayMs(5);
    I2C_SDA_OUT_REG &= ~I2C_SDA_PIN;
    I2C_SCL_OUT_REG &= ~I2C_SCL_PIN;

    switch(I2C_BASE){
    case EUSCI_B0_BASE:     // Configure GPIO P1.6 SDA DIR = DC SEL = 1,0 - P1.7 SCL DIR = DC SEL = 1,0
        I2C_PORT_SEL0_REG &= ~(I2C_SDA_PIN|I2C_SCL_PIN);
        I2C_PORT_SEL1_REG |= (I2C_SDA_PIN|I2C_SCL_PIN);
        break;
    case EUSCI_B1_BASE:     // Configure GPIO P5.0 SDA DIR = DC SEL = 0,1 - P5.1 SCL DIR = DC SEL = 0,1
    case EUSCI_B2_BASE:     // Configure GPIO P7.0 SDA DIR = DC SEL = 0,1 - P7.1 SCL DIR = DC SEL = 0,1
    case EUSCI_B3_BASE:     // Configure GPIO P6.4 SDA DIR = DC SEL = 0,1 - P6.5 SCL DIR = DC SEL = 0,1
    default:
        I2C_PORT_SEL0_REG |= (I2C_SDA_PIN|I2C_SCL_PIN);
        I2C_PORT_SEL1_REG &= ~(I2C_SDA_PIN|I2C_SCL_PIN);               // This configuration works for 3/4 USCB I2C Modules
    }


    // Configure EUSCI Bx Control Registers
    I2C_CONTROL0_REG |= UCSWRST;                                       // UCSWRST must be set to modify most registers
    I2C_PERIPHERAL_ADDRESS_REG = I2C_DEFAULT_PERIPHERAL_ADDRESS;       // Set expected peripheral address
    I2C_CONTROL0_REG &= UCSLA10__7BIT;                                 // Ensure 7-bit addressing
    I2C_CONTROL0_REG |= (UCTR|UCMODE|UCSYNC|UCSSEL_3|UCMST|UCSWRST);   // Set to I2C Master Transmit Mode using SMCLK, Sync Enabled
    I2C_BIT_RATE_PRESCALER_REG = I2C_PRESCALER_VALUE;                  // Divide SMCLK by 20 - 8MHz / 20 = 400kHz
    I2C_CONTROL1_REG |= UCASTP_2;                                      // Generate STOP Interrupts when TBCNT is reached
    I2C_CONTROL0_REG &= ~UCSWRST;                                      // Enable normal function

    // Initialize and clear I2C Buffer using #define, if this is first init() call
    if(i2c.status != I2C_Reset)
        i2c.dataPtr = (unsigned char*)calloc(I2C_BUFFER_SIZE, sizeof(char));

    __enable_interrupt();                                              // Just in case you haven't done this already

    return &i2c;

}

I2C_Struct* i2c_write(unsigned char peripheralAddress, unsigned char numberBytes, unsigned char* dataPtr){
    // Write is used for reading also, so this gets called in i2c_read()
    // To write bytes, fill the dataPtr with [regAddress][data] etc
    // It's easier to make a separate function call per register, than to configure many registers at once at once

    if(i2c_isBusy())
        i2c_handleError();
    else{
        i2c.expectedNumberBytes = numberBytes;
        i2c.dataPtr = dataPtr;

        // Reconfigure all the registers for this specific transaction
        I2C_CONTROL0_REG |= UCSWRST;
        I2C_PERIPHERAL_ADDRESS_REG = peripheralAddress;                    // Set peripheral address
        I2C_TX_BYTE_COUNTER_THRESHOLD_REG = numberBytes;                   // Byte counter set to length of message
        I2C_CONTROL0_REG |= (UCTR|UCMODE|UCSYNC|UCSSEL_3|UCMST|UCTXSTT);   // Set to I2C Master Transmit Mode using SMCLK, Sync Enabled
        I2C_CONTROL1_REG |= (UCASTP_2|UCCLTO_3);                           // Automatic STOPs, Low Clock Timeout after 34ms
        I2C_BIT_RATE_PRESCALER_REG = I2C_PRESCALER_VALUE;                  // Divide SMCLK by 20 - 8MHz / 20 = 400kHz
        I2C_TX_BUFF = 0x00;                                                // Empty TX Buffer before transmittion
        I2C_INTERRUPT_FLAG_REG = 0x0000;                                   // Initially clear all interrupt flags
        I2C_CONTROL0_REG &= ~UCSWRST;                                      // Ensure this bit is cleared so everything works
        I2C_INTERRUPT_ENABLE_REG = I2C_IE_MASK;                            // Enable selected interrupts

        // At this point, a START condition will trigger, peripheral address is sent and TX BUFF EMPTY Interrupt fires
        delayUs(80);                                                       // Testing reveals 60-80us delay needed if reading immediately after write
    }
    return &i2c;
}

I2C_Struct* i2c_read(unsigned char peripheralAddress, unsigned char numberBytes, unsigned char registerAddress){
    // Uses write to specify starting register and reads subsequent registers defined by numberBytes

    if(i2c_isBusy())
        i2c_handleError();
    else{
        i2c.expectedNumberBytes = 1;
        *i2c.dataPtr = registerAddress;

        i2c_write(peripheralAddress, 1, i2c.dataPtr);                      // Send Register Address

        i2c.expectedNumberBytes = numberBytes;

        // Reconfigure all the registers for this specific transaction
        I2C_CONTROL0_REG |= UCSWRST;
        I2C_PERIPHERAL_ADDRESS_REG = peripheralAddress;                    // Set peripheral address
        I2C_TX_BYTE_COUNTER_THRESHOLD_REG = numberBytes;                   // Byte counter set to length of message
        I2C_CONTROL0_REG &= ~UCTR;                                         // Ensure Receiver Mode
        I2C_CONTROL0_REG |= (UCMODE|UCSYNC|UCSSEL_3|UCMST|UCTXSTT);        // Set to I2C Master Mode using SMCLK, Sync Enabled
        I2C_CONTROL1_REG |= (UCASTP_2|UCCLTO_3);                           // Automatic STOPs, Low Clock Timeout after 34ms
        I2C_BIT_RATE_PRESCALER_REG = I2C_PRESCALER_VALUE;                  // Divide SMCLK by 20 - 8MHz / 20 = 400kHz
        rxJunk = I2C_RX_BUFF;                                              // This is necessary to prevent reading more than what we want
        I2C_INTERRUPT_FLAG_REG = 0x0000;                                   // Initially clear all interrupt flags
        I2C_CONTROL0_REG &= ~UCSWRST;                                      // Ensure this bit is cleared so everything works
        I2C_INTERRUPT_ENABLE_REG = I2C_IE_MASK;                            // Enable selected interrupts - STOP Interrupt

        // At this point, a START condition will trigger, peripheral address is sent and DATA RX Interrupt fires
        delayUs(40*numberBytes);                                           // This seems to work reliably, don't go lower
    }
    return &i2c;
}

I2C_Status i2c_isBusy(void){
    // This function is called before reading and writing to check if line is busy
    // SDA and SCL will hang if something goes wrong, and ain't nobody got time for that
    // If something is wrong, this function will do its best to fix it or return error

    if(I2C_STATUS_REG & UCBBUSY){
        delayMs(50);
        if(I2C_STATUS_REG & UCBBUSY){               // If 50ms delay doesn't work, try to re-initialize i2c module which resets GPIO and everything
            i2c_init();
            delayMs(10);
        }
        if(I2C_STATUS_REG & UCBBUSY)
            return I2C_Line_Busy;                   // We tried, but the line is still busy, maybe RESET MCU
    }
    return I2C_Success;
}

void i2c_handleError(void){
    __no_operation();           // Your code here
}

void delayMs(unsigned int ms){
    unsigned int i;

    for(i=0;i<ms;i++)
        __delay_cycles(8192);   // 8*1024 cycles per ms due to 8MHz clock

}

void delayUs(unsigned int us){
    unsigned int i;

    for(i=0;i<us;i++)
        __delay_cycles(8);      // For loop iteration adds cycles, produces slightly longer delays than 1 us / iteration

}

#pragma vector = I2C_IV_VECTOR
__interrupt void I2C_ISR(void)
{
    __no_operation(); // BP
    switch(__even_in_range(I2C_IV_REG, I2C_IV_9TH_BIT_POSITION)){
    case I2C_IV_DATA_RX:
        if(i2c.expectedNumberBytes){
            *(i2c.dataPtr+i2cPtrOffset++) = I2C_RX_BUFF;
            i2c.expectedNumberBytes--;                  // Receive byte and increment ptr using offset to preserve original ptr value
        }
        else{
            rxJunk = I2C_RX_BUFF;                       // This shouldn't happen, but just in case
        }
        break;
    case I2C_IV_TX_BUFF_EMPTY:
        if(i2c.expectedNumberBytes){
            I2C_TX_BUFF = *(i2c.dataPtr+i2cPtrOffset++);// Send TX Byte until all are sent
            i2c.expectedNumberBytes--;
        }
        break;
    case I2C_IV_START_CONDITION:
        // TODO Figure out why this interrupt never fires
        __no_operation(); // BP
        break;
    case I2C_IV_STOP_CONDITION:                         // Stop condition occurs when TX or RX Byte Threshold has been reached
        I2C_INTERRUPT_ENABLE_REG = 0x00;
        i2cPtrOffset = 0;
        i2c.status = I2C_Success;
        break;
    case I2C_IV_NACK:                                   // Using your own NACK handling code is recommended
        I2C_INTERRUPT_ENABLE_REG = 0x00;
        i2cPtrOffset = 0;
        I2C_CONTROL0_REG |= UCSWRST;                    // Reset I2C Module
        i2c.status = I2C_NACK_Received;
        break;
    case I2C_IV_CLOCK_LOW_TIMEOUT:                      // Clock was low for too long, Get SDA and SCL HIGH and trigger a STOP, reset I2C Module
        I2C_TX_BUFF = 0x00;                             // SDA likes to stay low, let's try reading and writing to make it happy
        rxJunk = I2C_RX_BUFF;                           // Depending on how we got stuck, one of these should work
        I2C_CONTROL0_REG |= UCSWRST;                    // Reset I2C Module
        I2C_INTERRUPT_ENABLE_REG = 0x00;                // Disable Interrupts
        i2cPtrOffset = 0;
        i2c.status = I2C_Timeout;
        break;
    case I2C_IV_BYTE_COUNTER_ZERO:                      // When reading 1 byte, automatic STOPs would occur before DATA_RX interrupts had a chance to fire
        I2C_INTERRUPT_ENABLE_REG |= I2C_IE_STOP_CONDITION;
        break;                                          // Using BYTE COUNTER Interrupts allows us to use any length reads/writes
    case I2C_IV_NONE:                                   //
    case I2C_IV_ARBITRATION_LOST:                       //-
    case I2C_IV_9TH_BIT_POSITION:                       //--
    case I2C_IV_PERIPHERAL3_DATA_RX:                    //---
    case I2C_IV_PERIPHERAL3_DATA_TX:                    //---- These are disabled but can be enabled in i2c_init() using I2C_IE_MASK #define
    case I2C_IV_PERIPHERAL2_DATA_RX:                    //---
    case I2C_IV_PERIPHERAL2_DATA_TX:                    //--
    case I2C_IV_PERIPHERAL1_DATA_RX:                    //-
    case I2C_IV_PERIPHERAL1_DATA_TX:                    //
    default: break;
    }

    __bic_SR_register_on_exit(LPM4_bits);     // Exit LPM
}
