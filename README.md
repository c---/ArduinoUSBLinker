ArduinoUSBLinker
================

To use:

 * Load this sketch on to your Arduino or other ATmega.

 * Hook up the serial port/USB from the Arduino to your computer.

 * Make sure the ESC and Arduino have a common ground (GND) connection.

 * Connect the simonk ESC (with bootloader enabled) servo signal wire to
   PD2/INT0 on the Arduino. I recommend a 470 ohm resistor in series on this
   line as a general protective measure.

 * Finally use something like avrdude to flash over the serial port at 115200
   baud using STK500v2 protocol. Look at the simonk Makefile for avrdude usage
   examples. The LazyZero KKmulticopter Flash tool should work also but I have
   not tested it.


Please look at the top of the source file for notes and more information.

Advanced usage:

 * The ArduinoUSBLinker supports run-time configuration of which pin to use for
   signaling and at what bit rate to run at. These options are configured by
   using text commands over the serial port.

   The command format is similar to but not the same as the MultiWii serial
   protocol. All messages start with "$M&lt;" followed by a command character
   and its parameter(s).

   Supported commands are:
   $M&lt;In : Initializes pin number n by turning on its pull-up resistor.
              This will hold the ESC in bootloader mode. All ESC pins should
              be initialized first to prevent accidental motor power-up.
              Example: "$M&lt;I1" will initialize pin 1 (PB1).

   $M&lt;Pn : Select pin number n. This also set the pull-up resistor for the
              pin. The currently selected pin will be used for signaling.
              Example: "$M&lt;P18" will select pin 18 (PD3/INT1).

   $M&lt;Bn : Sets the bit rate in microseconds.
              Example: "$M&lt;B32" will set a 32us signaling rate.

   All commands, including invalid commands will return the current settings.
   The settings are followed by a list of all the ports and the starting pin
   number for each.

   Example status line:
   P18:B136:PINS:B0:C8:D16:

   Pin 18 is selected, bit rate is 136us and there are 3 ports:
   B starts with pin 0.
   C starts with pin 8.
   D starts with pin 16.

   This idicates pin 0-7 is PB0..7, pin 8-15 is PC0..7, pin 16-24 is PD0..7,
   and so on. These are the pin numbers used with the above commands.

 * This sketch can be integrated with MultiWii. Copy the "ArduinoUSBLinker.ino"
   file in to the MultiWii sketch folder and rename it as "ArduinoUSBLinker.h".
   Apply the supplied patch "Serial.patch" to the "Serial.ino" file.

   The ArduinoUSBLinker code adds about 3K to the firmware size and depending
   on which MultiWii options are configured there may not be enough room for
   everything.
   
   To enter the ArduinoUSBLinker mode a MultiWii command 211 must be sent using
   the MultiWii serial protocol. This is a binary string of the following
   6 characters (in hex): 24  4D  3C  00        D3    D3
                          '$' 'M' '<' <datalen> <211> <checksum>

   Note the "211" command is subject to change and is not part of the official
   MultiWii code base.

   Once in ArduinoUSBLinker mode it behaves as normal. However, the default
   baud rate is 19200 and it is not recommended to change this to a faster
   speed as higher rates are unreliable when running in MultiWii mode. Note
   that the baud rate will most likely change when entering ArduinoUSBLinker
   mode from normal MultiWii mode.

   To exit ArduinoUSBLinker mode reset the device.
