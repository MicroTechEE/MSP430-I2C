/*
 * i2c.h
 *
 *  Created with masterful finesse on: Apr 3, 2023 by
 *      Author: Tom Bailey
 */


/*- - - -  #INCLUDES - - - -*/
#include "msp430.h"
#include "intrinsics.h"
#include "stdlib.h"
#include "gpio.h"

/*- - - -  #DEFINES - - - -*/
// IV - Interrupt Vector
#define I2C_IV_VECTOR                       USCI_B2_VECTOR                  // ** <-- CHANGE THIS IF NEEDED **
#define I2C_IV_REG                          UCB2IV                          // ** <-- CHANGE THIS IF NEEDED **
#define I2C_IV_NONE                         0x00
#define I2C_IV_ARBITRATION_LOST             UCIV__UCALIFG                   // Arbitration lost IV related to UCALIFG flag
#define I2C_IV_NACK                         UCIV__UCNACKIFG                 // NACK IV related to UCNACKIFG flag
#define I2C_IV_START_CONDITION              UCIV__UCSTTIFG                  // Start Condition IV related to UCSTTIFG flag
#define I2C_IV_STOP_CONDITION               UCIV__UCSTPIFG                  // Stop Condition IV related to UCSTPIFG flag
#define I2C_IV_PERIPHERAL3_DATA_RX          UCIV__UCRXIFG3                  // Data RX IV related to UCRXIFG3 flag
#define I2C_IV_PERIPHERAL3_DATA_TX          UCIV__UCTXIFG3                  // TX Buff Empty IV related to UCTXIFG3 flag
#define I2C_IV_PERIPHERAL2_DATA_RX          UCIV__UCRXIFG2                  // Data RX IV related to UCRXIFG2 flag
#define I2C_IV_PERIPHERAL2_DATA_TX          UCIV__UCTXIFG2                  // TX Buff Empty IV related to UCTXIFG2 flag
#define I2C_IV_PERIPHERAL1_DATA_RX          UCIV__UCRXIFG1                  // Data RX IV related to UCRXIFG1 flag
#define I2C_IV_PERIPHERAL1_DATA_TX          UCIV__UCTXIFG1                  // TX Buff Empty IV related to UCTXIFG1 flag
#define I2C_IV_DATA_RX                      UCIV__UCRXIFG0                  // Data RX IV related to UCRXIFG0 flag
#define I2C_IV_TX_BUFF_EMPTY                UCIV__UCTXIFG0                  // TX Buff Empty IV related to UCTXIFG0 flag
#define I2C_IV_BYTE_COUNTER_ZERO            UCIV__UCBCNTIFG                 // Byte Counter Zero IV related to UCBCNTIFG flag
#define I2C_IV_CLOCK_LOW_TIMEOUT            UCIV__UCCLTOIFG                 // Clock Low Timeout IV related to UCCLTOIFG flag
#define I2C_IV_9TH_BIT_POSITION             UCIV__UCBIT9IFG                 // 9th bit IV related to UCBIT9IFG flag

// IE - Interrupt Enable
#define I2C_INTERRUPT_ENABLE_REG            UCB2IE                          // ** <-- CHANGE THIS IF NEEDED **
#define I2C_IE_ARBITRATION_LOST             UCALIE                          // Arbitration lost, only used in Multi-Controller Schema
#define I2C_IE_NACK                         UCNACKIE                        // NACK, Controller did not receive ACK from Peripheral
#define I2C_IE_START_CONDITION              UCSTTIE                         // Start Condition, this never fires for some reason
#define I2C_IE_STOP_CONDITION               UCSTPIE                         // Stop Condition, sequence received or sent, good time to wrap up
#define I2C_IE_PERIPHERAL3_DATA_RX          UCRXIE3                         // Data RX, received byte
#define I2C_IE_PERIPHERAL3_DATA_TX          UCTXIE3                         // TX Buff Empty, ready to send another byte, Peripheral Mode Address 3
#define I2C_IE_PERIPHERAL2_DATA_RX          UCRXIE2                         // Data RX, received another byte, Peripheral Mode Address 3
#define I2C_IE_PERIPHERAL2_DATA_TX          UCTXIE2                         // TX Buff Empty, ready to send another byte, Peripheral Mode Address 2
#define I2C_IE_PERIPHERAL1_DATA_RX          UCRXIE1                         // Data RX, received another byte, Peripheral Mode Address 2
#define I2C_IE_PERIPHERAL1_DATA_TX          UCTXIE1                         // TX Buff Empty, ready to send another byte, Peripheral Mode Address 1
#define I2C_IE_DATA_RX                      UCRXIE0                         // Data RX, received another byte, Controller Mode ( Main )
#define I2C_IE_TX_BUFF_EMPTY                UCTXIE0                         // TX Buff Empty, ready to send another byte Controller Mode ( Main )
#define I2C_IE_BYTE_COUNTER_ZERO            UCBCNTIE                        // Byte Counter Zero, all bytes send/received
#define I2C_IE_CLOCK_LOW_TIMEOUT            UCCLTOIE                        // Clock Low Timeout, Clock was pulled low longer than 34ms, handle error
#define I2C_IE_9TH_BIT_POSITION             UCBIT9IE                        // 9th bit, after byte is sent/received
#define I2C_IE_MASK                         (I2C_IE_NACK|I2C_IE_START_CONDITION|I2C_IE_DATA_RX|I2C_IE_STOP_CONDITION|I2C_IE_TX_BUFF_EMPTY|I2C_IE_CLOCK_LOW_TIMEOUT)

// RX/TX / Init
#define I2C_BASE                            EUSCI_B2_BASE                   // ** v v CHANGE THESE IF NEEDED v v **
#define I2C_BUFFER_SIZE                     50
#define I2C_DEFAULT_PERIPHERAL_ADDRESS      #error "Update Peripheral Address in i2c.h"
#define I2C_PRESCALER_VALUE                 20
#define I2C_CONTROL0_REG                    UCB2CTLW0
#define I2C_CONTROL1_REG                    UCB2CTLW1
#define I2C_TX_BYTE_COUNTER_THRESHOLD_REG   UCB2TBCNT
#define I2C_INTERRUPT_FLAG_REG              UCB2IFG
#define I2C_PERIPHERAL_ADDRESS_REG          UCB2I2CSA
#define I2C_STATUS_REG                      UCB2STAT
#define I2C_TX_BUFF                         UCB2TXBUF
#define I2C_RX_BUFF                         UCB2RXBUF
#define I2C_BIT_RATE_PRESCALER_REG          UCB2BRW

// GPIO
#define I2C_SDA_PORT                        GPIO_PORT_P7                    // ** v v CHANGE THESE IF NEEDED v v **
#define I2C_SDA_PIN                         GPIO_PIN0
#define I2C_SDA_OUT_REG                     P7OUT
#define I2C_SCL_PORT                        GPIO_PORT_P7
#define I2C_SCL_PIN                         GPIO_PIN1
#define I2C_SCL_OUT_REG                     P7OUT
#define I2C_PORT_SEL0_REG                   P7SEL0
#define I2C_PORT_SEL1_REG                   P7SEL1


/*- - - -  TYPEDEFS - - - -*/
typedef enum{
    I2C_Success = 0,
    I2C_Timeout,
    I2C_NACK_Received,
    I2C_Line_Busy,
    I2C_Reset
    // Add yours here

}I2C_Status;

typedef struct
{
    unsigned char* dataPtr;
    unsigned char expectedNumberBytes;
    I2C_Status status;
    // Put things her to assist with transmissions, such as status codes, etc make this yours
}I2C_Struct;



/*- - - -  FUNCTIONS - - - -*/
void i2c_init(void);
void i2c_write(unsigned char peripheralAddress, unsigned char numberBytes, unsigned char* dataPtr);
void i2c_read(unsigned char peripheralAddress, unsigned char numberBytes, unsigned char registerAddress);
void i2c_handleError(void);

I2C_Struct* i2c_getStruct(void);
I2C_Status i2c_isBusy(void);

void delayMs(unsigned int ms);
void delayUs(unsigned int us);

#error "Make sure you change all the i2c #defines to suit your application! Delete me at the bottom of i2c.h"



