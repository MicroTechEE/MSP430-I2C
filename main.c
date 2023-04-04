#include <msp430.h> 


/**
 * main.c
 */


/* #includes */
#include "i2c.h"

/* #defines */
#define LFXIN_Pin       BIT5                // For setting up clocks
#define LFXOUT_Pin      BIT4

/* Functions */
void setupClocks(void);
void setupIO(void);

/* Globals */
I2C_Struct* i2cStructPtr;                   // To access the i2c struct you need a pointer to it, the original lives in i2c.c

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	            // Stop watchdog timer
	
	setupClocks();
	setupIO();

	i2c_init();
	i2cStructPtr = i2c_getStruct();         // Calling this will allow you to point to the i2c struct wherever you are in code ( or you can extern the struct )

	while(1){
	    //           vv-your address     # bytes-v   vvv-register address
	    i2c_read(I2C_DEFAULT_PERIPHERAL_ADDRESS, 5, 0x92);
	    // A good way to test the read function is to find your device's WHO_AM_I or DEVICE_ID register

	    // Set a breakpoint here and check on your struct pointer to find data + status
	    if(*i2cStructPtr->dataPtr)
	        __no_operation();

	    // Handle errors, if any
	    if(i2cStructPtr->status)
	        __no_operation();               // Handle error, Success = 0, Error > 0


	    delayMs(1000);


	}

}

void setupClocks(void)
{
    // Sets main clock to 8MHz and ACLK to 32k crystal
    //setup pin function
    PJSEL0 = (LFXOUT_Pin | LFXIN_Pin);

    // XT1 Setup
    CSCTL0_H = CSKEY >> 8;                                  // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                                     // Set DCO to 8MHz
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;   // selects LFXTclk for Aclk, DCO for SMclk, DCO for Mclk
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;                   // Set all dividers to 1
    CSCTL4 &= ~LFXTOFF;                                     // Enable LFXT1 only (External 32khz crystal)
    do
    {
        CSCTL5 &= ~LFXTOFFG;                                // Clear XT1 fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1&OFIFG);                                 // Test oscillator fault flag
    CSCTL0_H = 0;                                           // Lock CS registers
}

void setupIO(void)
{

    //reset all GPIOs to a known state //set all outputs to zero unless the need to be otherwise
    P1OUT = 0x00;
    P2OUT = 0x00;
    P3OUT = 0x00;
    P4OUT = 0x00;
    P5OUT = 0x00;
    P6OUT = 0x00;
    P7OUT = 0x00;
    P8OUT = 0x00;
    PJOUT = 0x00;

    //set everything to an output unless otherwise needed to be an input
    P1DIR = 0xFF;
    P2DIR = 0xFF;
    P3DIR = 0xFF;
    P4DIR = 0xFF;
    P5DIR = 0xFF;
    P6DIR = 0xFF;
    P7DIR = 0xFF;
    P8DIR = 0xFF;
    PJDIR = 0xFF;

    //Disable Pull up and pull downs
    P1REN = 0x00;
    P2REN = 0x00;
    P3REN = 0x00;
    P4REN = 0x00;
    P5REN = 0x00;
    P6REN = 0x00;
    P7REN = 0x00;
    P8REN = 0x00;
    PJREN = 0x00;

    //leave these to be setup in interface specific functions
    P1SEL0 = 0x00;P2SEL0 = 0x00;P3SEL0 = 0x00;P4SEL0 = 0x00;P5SEL0 = 0x00;P6SEL0 = 0x00;P7SEL0 = 0x00; P8SEL0 = 0x00; //Primary, secondary, and tertiary module function

    //Adding this here shortens our clock settling time after wakeup from 200ms to a few us.
    PJSEL0 = 0x00 | (LFXOUT_Pin | LFXIN_Pin); //For Clock  //this doesnt seem to hurt current draw added in Rev3.0

    P1SEL1 = 0x00;P2SEL1 = 0x00;P3SEL1 = 0x00;P4SEL1 = 0x00;P5SEL1 = 0x00;P6SEL1 = 0x00;P7SEL1 = 0x00; P8SEL1 = 0x00;PJSEL1 = 0x00;


    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
}
