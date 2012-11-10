ArduinoUSBLinker
================

 ** WARNING **
   Always use great care when working with powered ESC's that are connected
   to motors. Especially when reflashing firmware and reseting devices it is
   easily possible to get a situation where the motors start running on their
   own.

To use:

 * Load this sketch on to your Arduino or similar with at least 512 bytes
   RAM.

 * Hook up the serial port/USB from the Arduino to your computer.

 * Make sure the ESC and Arduino have a common ground (GND) connection.

 * Connect an ESC that has the simonk bootloader enabled by connecting the
   servo signal wire to PD2/INT0 on the Arduino. I recommend a 470 ohm resistor
   in series on this line as a general protective measure.

 * Finally use something like avrdude or KKmulticopter Flash Tool to flash over
   the serial port at 19200 baud using STK500v2 protocol. Look at the simonk
   Makefile for avrdude usage examples.


Known issues:
  When applying power to the Arduino and the ESC at the same time the ESC will
  arm before we are able to set the signal pin HIGH. It will still work but be
  careful. Best is to connect and power the Arduino first then power up the
  ESC to ensure that it is held in the bootloader (there should be no beeps
  from the ESC).

  Message sizes of more than 297 bytes (total) not not supported and will
  likely crash this software. The STK500 firmware is currently limited to 281
  bytes so it is not an issue at this time.

  Note that the default serial port rate is 19200 and this is separate from
  the servo wire signaling rate. Make sure your tools are using the same
  serial port rate.

  Both the serial port baud rate and the signaling rate can be changed on the
  fly or stored in EEPROM, see below.


Advanced usage:

 ** WARNING ** It is critically important that you don't have multiple
   applications accessing the serial port at the same time. The Arduino serial
   monitor in particular because it holds the port open and intercepts data.
   Typically in this situation avrdude will get errors, recover, and then
   eventually finish successfully. HOWEVER, dispite finishing successfully your
   firmware will most likely be corrupted. This goes for both reading and
   writing over the serial port.

 * The ArduinoUSBLinker supports run-time configuration of which pin to use for
   signaling and at what bit rate to run at. These options are configured by
   using text commands sent over the serial port.

   The command format is similar to but not the same as the MultiWii serial
   protocol. All messages start with "$M&lt;" followed by a command character
   and its parameter(s). A description of how pin numbers are determined is
   below.

   Supported commands are:

   $M&lt;Pn : Select pin number n. This will also set the pull-up resistor for
              the pin. The currently selected pin will be used for signaling.
              It is recommened to make one pass through all the ESC pins
              selecting each one before doing anything else. This will put them
              all in bootloader mode and prevent "no-signal" beeping.
              Example: "$M&lt;P18" will select pin 18 (PD2/INT0).

   $M&lt;Bn : Sets the bit time in microseconds.
              Example: "$M&lt;B32" will set a 32µs signaling rate.

   $M&lt;Rn : Sets the serial port baud rate. Only for non-MultiWii builds. In
              MultiWii builds this will always return "0" in the status line.
              Example: "$M&lt;R115200" will set the serial port to 115200 bps.

   $M&lt;W  : Write the pin, bit time, and serial port baud to EEPROM. These
              will be restored on reset. Only for non-MultiWii builds.

   All commands, including invalid commands that start with "$M&lt;" will
   print the current settings back over the serial port. The settings are
   followed by a list of all the ports and the starting pin number for each.

   Example status line:
   P18:B136:R9600:PINS:B0:C8:D16:

   Pin 18 is selected, bit rate is 136µs, baud rate is 9600, and there are
   3 ports:
   PORTB starts with pin 0.
   PORTC starts with pin 8.
   PORTD starts with pin 16.

   This indicates pin 0-7 is PB0..7, pin 8-15 is PC0..7, pin 16-24 is PD0..7,
   and so on. These are the pin numbers used with the above commands.

 * This sketch can be integrated with MultiWii and has been tested with
   MultiWii\_shared latest SVN source. Copy the "ArduinoUSBLinker.ino" file in
   to the MultiWii sketch folder and rename it as "ArduinoUSBLinker.h". Apply
   the supplied patch "Serial.patch" to the MultiWii "Serial.ino" file.

   The ArduinoUSBLinker code adds approximately 1800 bytes to the firmware size
   and depending on which MultiWii options are configured there may not be
   enough room for everything.

   Baud rate for the serial interface is the same as MultiWii (usually 115200).
   
   To enter the ArduinoUSBLinker mode a MultiWii command 211 must be sent using
   the MultiWii serial protocol. This is a binary string of the following
   6 characters (in hex): <code>24 4D 3C 00 D3 D3</code><br>
   In pseudo text:
   <code>'$' 'M' '&lt;' &lt;datalen&gt; &lt;211&gt; &lt;checksum&gt;</code>

   Note the "211" command is subject to change and is not part of the official
   MultiWii code base.

   To exit ArduinoUSBLinker mode power cycle or reset the device.
