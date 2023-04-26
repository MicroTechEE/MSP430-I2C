# MSP430-I2C

This code is written for MSP430 and tested thouroughly on MSP430FR5994. You will need to change the #defines in i2c.h per your application needs.
This code is extremely well documented and portable.

The Interrupt Service Routines (ISRs) are fairly robust and have many case-scenarios in mind.
Everything is register-level; no TI code used. MSP430.h is all that is needed and should be included in your IDE directory:
i.e. On CCS v 12.10 C:\ti\ccs1210\ccs_base\msp430\include\msp430.h

Please report any issues. I will fix them quickly.

Code recognizes the transition in nomenclature from master/slave to controller/peripheral respectively.

# How to use:

i2c_init() passes the i2c struct pointer as well. You don't have to use it, but it's there if you want.
i2c_init() configures GPIO on boot and on i2c reset. 

i2c_isBusy() will return the status of the i2c SCL line. If pulled low for too long, there should be an interrupt that fires that resets the i2c module.
As a backup, before writing and reading i2c_isBusy() gets called to determine if we have a healthy connection.

If the SCL is stuck low, i2c_isBusy() will attempt to troubleshoot it and may even reset the i2c module by calling i2c_init();

When using i2c_write, you populate the arguments with:
  - Peripheral Address
  - Number of Bytes being written
  - Pointer ( or array ) of data to be written
      * Typically, two bytes are written at a time, first being register address and second is data being written

### Example:
  Write one byte to address 0x10 on our device ( address 0x34 ).
  
    txBuffer[0] = 0x10;
    txBuffer[1] = 0xFF;
    i2c_write(0x34, 2, txBuffer);
  
When using i2c_read, you populate the arguments with:
  * Peripheral address
  * Number of bytes being read
  * Register address to start read operation
      * When reading, you start at an address, and you can read registers in a row depending on your Number of Bytes argument
      * Some devices aren't read using registers, you just 'read the device itself'. For these devices, use i2c_readNoRegister().
      
### Example:
  Read 5 bytes starting at 0x10 for our device ( address 0x34 ). 
  
    i2c_read(0x34, 5, 0x10);

## I2C Struct:

The main I2C Struct consists of the message buffer used to TX and RX. The struct also contains the expected length, in bytes, of the TX or RX payload. Finally, the struct includes an enum called status. Status will update in key points of the process and you will be able to react accordingly.

## I2C Status Enum:

    I2C_Success, I2C_Timeout, I2C_NACK_Received, I2C_Line_Busy, I2C_Reset
    
It would be a good idea to pass your status as an argument when calling i2c_handleError(). I'll let you handle that ( pun intended ).

## Using the Pointers:

i2c_init(), i2c_write(), and i2c_read() return a pointer to the original i2c struct. This is helpful because it allows you to call i2c functions from anywhere in your program while being able to access that struct via a pointer. Also, you can choose to use one of these functions as a conditional argument such as:
    
    if(i2c_read(0x34, 2, 0x10)->status != I2C_Success)
      i2c_handleError();
     
In this example, we attempt to read two bytes from our peripheral device starting at register address 0x10. If the read fails for any reason, we handle it. Currently i2c_handleError() is not populated. This would be application-specific, but the hooks are in the code to handle errors in certain places.
