ArduinoUSBLinker
================

To use:

 * Load this sketch on to your Arduino or other ATmega.

 * Hook up the serial port/USB from the Arduino to your computer.

 * Make sure the ESC and Arduino have a common ground (GND) connection.

 * Connect the simonk ESC (with bootloader enabled) signal wire to PD2/INT0
   on the Arduino. I recommend a 470 ohm resistor on this line as a general
   protective measure.

 * Finally use something like avrdude to flash over the serial port at 115200
   baud using STK500v2 protocol. Look at the simonk Makefile for avrdude usage
   examples. The LazyZero KKmulticopter Flash tool should work also but I have
   not tested it.


Please look at the top of the source file for notes and more information.

