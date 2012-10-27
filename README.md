ArduinoUSBLinker
================

 ** WARNING **
   Always use great care when working with powered ESC's that are connected
   to motors. Especially when reflashing firmware and reseting devices it is
   easily possible to get a situation where the motors start running on their
   own.

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
              be initialized first to help prevent accidental motor power-up.
              However, always consider the motors live and dangerous at all
              times.
              Example: "$M&lt;I1" will initialize pin 1 (PB1).

   $M&lt;Pn : Select pin number n. This also set the pull-up resistor for the
              pin. The currently selected pin will be used for signaling.
              Example: "$M&lt;P18" will select pin 18 (PD2/INT0).

   $M&lt;Bn : Sets the bit rate in microseconds.
              Example: "$M&lt;B32" will set a 32µs signaling rate.

   All commands, including invalid commands that start with "$M&lt;" will
   return the current settings. The settings are followed by a list of all the
   ports and the starting pin number for each.

   Example status line:
   P18:B136:PINS:B0:C8:D16:

   Pin 18 is selected, bit rate is 136µs and there are 3 ports:
   B starts with pin 0.
   C starts with pin 8.
   D starts with pin 16.

   This indicates pin 0-7 is PB0..7, pin 8-15 is PC0..7, pin 16-24 is PD0..7,
   and so on. These are the pin numbers used with the above commands.

 * This sketch can be integrated with MultiWii (MultiWii\_shared latest SVN
   source). Copy the "ArduinoUSBLinker.ino" file in to the MultiWii sketch
   folder and rename it as "ArduinoUSBLinker.h". Apply the supplied patch
   "Serial.patch" to the MultiWii "Serial.ino" file.

   The ArduinoUSBLinker code adds about 3K to the firmware size and depending
   on which MultiWii options are configured there may not be enough room for
   everything. Efforts will be made in the future to reduce the size but it is
   unknown how much can be done.
   
   To enter the ArduinoUSBLinker mode a MultiWii command 211 must be sent using
   the MultiWii serial protocol. This is a binary string of the following
   6 characters (in hex): <code>24 4D 3C 00 D3 D3</code><br>
   Or in pseudo text: <code>'$' 'M' '&lt;' &lt;datalen&gt; &lt;211&gt; &lt;checksum&gt;</code>

   Note the "211" command is subject to change and is not part of the official
   MultiWii code base.

   Once in ArduinoUSBLinker mode it behaves as normal over the serial port.
   However, the default baud rate is 19200 and it is not recommended to change
   this to a faster speed as higher rates are unreliable when running in
   MultiWii mode. Note that the baud rate will most likely change to 19200 when
   entering ArduinoUSBLinker mode from normal MultiWii mode.

   To exit ArduinoUSBLinker mode reset the device.
