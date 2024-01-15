# Bus Pirate I2C bus driver

This is an I2C bus driver for the Dangerous Prototypes "Bus Pirate" board
(http://dangerousprototypes.com/docs/Bus_Pirate).  The Bus Pirate is bus
analyzer and master for a variety of embedded bus protocols (I2C, SPI, JTAG,
etc).  This driver implements only the I2C side.

The BP uses an onboard FTDI chip to present itself to Linux as a /dev/ttyUSB
serial port. As a result, this driver is implemented on top of that serial
port as a line discipline driver.  To instantiate the I2C bus, simply
register the N_BUSPIRATE line discipline on the BP's serial port.  The I2C
bus will appear after that. Instructions are below.

This driver assumes your I2C device needs the BP's integrated pull-up
resistors and voltage rails enabled.  It automatically turns them on/off.


Instructions:
---
Add the Bus Pirate line discipline to the kernel's TTY header:
```
include/linux/tty.h
...
  #define N_TI_WL         22      /* for TI's WL BT, FM, GPS combo chips */
  #define N_TRACESINK     23      /* Trace data routing for MIPI P1149.7 */
  #define N_TRACEROUTER   24      /* Trace data routing for MIPI P1149.7 */
+ #define N_BUSPIRATE     25      /* Bus Pirate multifunction adapter */
```
Add the line discipline into the util-linux (https://www.kernel.org/pub/linux/utils/util-linux/):
```
sys-utils/ldattach.c
...
        { "PPS",                N_PPS },
        { "M101",               N_GIGASET_M101 },
        { "GIGASET",            N_GIGASET_M101 },
        { "GIGASET_M101",       N_GIGASET_M101 },
+       { "BUSPIRATE",          25 },
        { NULL,                 0 }
```
Compile the "ldattach" binary. 

Modify "$KDIR" in the Bus Pirate Makefile to point to your kernel source. Make buspirate.ko and load it.  You should see:
```
 [ 2786.741702] buspirate: module loaded
```
Use ldattach to enable the "BUSPIRATE" line discipline on the BP's serial port.  Most likely the BP will use /dev/ttyUSB0:
```
	ldattach BUSPIRATE /dev/ttyUSB0
```
Your I2C bus should now appear:
```
	$ i2cdetect -l
	...
	i2c-8	unknown   	Bus Pirate I2C                  	N/A
```
You can restore the old line discipline when you're done with:
```
	 ldattach TTY /dev/ttyUSB0
```
