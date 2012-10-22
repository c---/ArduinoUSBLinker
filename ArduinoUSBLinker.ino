/*
Copyright (C) 2012 Chris Osgood <chris at luadev.com>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.  No
other versions are acceptable.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.
*/

/*
VERSION 0.2

Notes:
  Please try reading from your ESC before trying to burn new firmware.
  Dump the eeprom or something first to make sure the interface is
  working.

Known issues:
  When applying power to this device and the ESC at the same time the
  ESC will arm before we are able to set the signal pin HIGH. It will
  still work but be careful. Best is to connect and power this device
  first then power up the ESC so that it will hold in the bootloader
  (no beeps).

  Message sizes of more than 1021 bytes (total) not not supported and
  will likely crash this software. The STK500 firmware is currently
  limited to 281 bytes so it is not an issue at this time.

  The current design does not allow for very fast bit rates over the
  signal pin. The default is 136µs per bit or about 7353 bps which is
  close to what the USB Linker uses. Less than 64µs does not work well
  due to timing issues. The problem most likely lies with this code
  as it is not as efficient and accurate as it could be and I only
  loosely followed Simon's notes for timing. See BITTIME.

  Note that the default serial port rate is 115200. Make sure your tools
  are using this rate also or change it below; see SERIALRATE.
  avrdude has no problem with the 115200 serial rate and slower signal
  rates but some tools could time out while waiting for the response
  messages. Slower serial port rates work fine but will make transfers
  slower due to the buffered design.
  
Version history:
  0.1 2012-10-21
      Initial release. Maximum speed around 15 kbps (64µs).
  
  0.2 2012-10-22
      Minor code cleanup in ReadLeader().  Added main() implementation to
      prevent Arduino overhead. Maximum speed now around 50 kbps (20µs).
*/

// Signal pin (default: PD2/INT0)
#define ULPORT D
#define ULPIN 2

// Macro expansion macros
#define CONCAT0(x,y) x##y
#define CONCAT(x,y) CONCAT0(x,y)

// These macros concatenate other macros.
// For example if ULPORT is D and ULPIN is 2:
// CONCAT(ULPORT,ULPIN) will produce the macro D2
// CONCAT(PORT,ULPORT) will produce the macro PORTD
// CONCAT(PORT,ULPORTPIN) will produce the macro PORTD2

#define ULPORTPIN CONCAT(ULPORT,ULPIN)
// Set signal pin high
#define PINHIGH   (CONCAT(PORT,ULPORT) |=  (1 << CONCAT(PORT,ULPORTPIN)))
// Set signal pin low
#define PINLOW    (CONCAT(PORT,ULPORT) &= ~(1 << CONCAT(PORT,ULPORTPIN)))
// Set signal pin to output mode
#define PINOUTPUT (CONCAT(DDR,ULPORT)  |=  (1 << CONCAT(DD,ULPORTPIN)))
// Set signal pin to input mode
#define PININPUT  (CONCAT(DDR,ULPORT)  &= ~(1 << CONCAT(DD,ULPORTPIN)))
// Read signal pin status (HIGH or LOW)
#define PINREAD   (CONCAT(PIN,ULPORT)  &   (1 << CONCAT(PIN,ULPORTPIN)))

// Calculates ticks from microseconds
#define MICROS(x) ((F_CPU / 1000000) * x)

#define DELAY delayTicks

// Approximate microseconds for each bit when sending
//#define BITTIME MICROS(136)
#define BITTIME MICROS(20)

#define LONGBITDELAY DELAY(BITTIME / 2)
#define SHORTBITDELAY DELAY(BITTIME / 4)

#define LONGWAIT MICROS(1000)

// Serial port
#define SERIALRATE 115200
#define SERIALTIMEOUT (F_CPU / (SERIALRATE / 16))

///////////////////////////////////////////////////////////////////////////////
// Globals ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Calculated leader timing for receive
static int bitTime, shortBitTime;

static volatile int8_t bitTrigger;
static volatile int lastBitTicks;

///////////////////////////////////////////////////////////////////////////////
// SENDING on signal pin //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
static inline void delayTicks(int count)
{
  TCNT1 = 0;
  while (TCNT1 < count);
}

static inline void Send1()
{
  PINHIGH;
  LONGBITDELAY;
  PINLOW;
  LONGBITDELAY;
}

static inline void Send0()
{
  PINHIGH;
  SHORTBITDELAY;
  PINLOW;
  SHORTBITDELAY;
  PINHIGH;
  SHORTBITDELAY;
  PINLOW;
  SHORTBITDELAY;
}

static inline void SendByte(int b)
{
  (b & B00000001) ? Send1() : Send0();
  (b & B00000010) ? Send1() : Send0();
  (b & B00000100) ? Send1() : Send0();
  (b & B00001000) ? Send1() : Send0();
  (b & B00010000) ? Send1() : Send0();
  (b & B00100000) ? Send1() : Send0();
  (b & B01000000) ? Send1() : Send0();
  (b & B10000000) ? Send1() : Send0();
}

///////////////////////////////////////////////////////////////////////////////
// RECEIVE on signal pin //////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect)
{
  lastBitTicks = TCNT1;
  TCNT1 = 0;
  bitTrigger = 1;
}

static inline int WaitPinHighLow(int timeout)
{
  while (bitTrigger == 0)
    if (TCNT1 > timeout)
      return -1;

  bitTrigger = 0;

  return lastBitTicks;
}

static inline int8_t ReadBit()
{
  int timer = WaitPinHighLow(bitTime * 2);

  if (timer < 0)
    return -1;
  else if (timer < shortBitTime)
  {
    if (WaitPinHighLow(bitTime * 2) < 0) return -1;
    return 0;
  }
  else
    return 1;
}

static int ReadLeader()
{
  // Skip the first few to let things stabilize
  if (WaitPinHighLow(LONGWAIT) < 0) return -1;
  if (WaitPinHighLow(LONGWAIT) < 0) return -2;
  if (WaitPinHighLow(LONGWAIT) < 0) return -3;
  if (WaitPinHighLow(LONGWAIT) < 0) return -4;
  if (WaitPinHighLow(LONGWAIT) < 0) return -5;

  int timer;
  bitTime = 0;

  // Average the next 10 to get our bit timing
  // NOTE: the longest bit time is about 200us or we will overflow
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -6;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -7;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -8;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -9;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -10;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -11;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -12;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -13;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -14;
  bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -15;
  bitTime += timer;

  bitTime /= 10;
  shortBitTime = (bitTime >> 1) + (bitTime >> 2);

  // Read until we get a 0 bit
  while (1)
  {
    int b = ReadBit();

    if (b < 0)
      return -16;
    else if (b == 0)
      break;
  }

  return 0;
}

static inline int ReadByte()
{
  int b;
  int8_t b2;

  if ((b = ReadBit()) < 0) return -1;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 1;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 2;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 3;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 4;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 5;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 6;

  if ((b2 = ReadBit()) < 0) return -1;
  b |= b2 << 7;

  return b;
}

///////////////////////////////////////////////////////////////////////////////
// Arduino ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  PININPUT;
  PINHIGH; // Enable pull-up
  Serial.begin(SERIALRATE);

  // Set up timer1 to count ticks  
  TCCR1B = (1 << CS10);
  TCCR1A = 0;

  // Signal pin timer interrupt
  EICRA |= (1 << ISC01) | (1 << ISC00);
  EIMSK |= (1 << INT0);
  sei();
}

void loop()
{
  // The buffer always has the leader at the start
  static uint8_t buf[1024] = { 0xFF, 0xFF, 0x7F };
  static int buflen, b, i;

main:
  b = Serial.read();
  if (b >= 0)
  {
    // Disable signal pin timer interrupt
    EIMSK = 0;

    buflen = 3;
    buf[buflen++] = b;

    // Buffer data until the serial timeout
    TCNT1 = 0;
    do {
       b = Serial.read();
       if (b >= 0)
       {
         buf[buflen++] = b;
         TCNT1 = 0;
       }
    } while (TCNT1 < SERIALTIMEOUT);

    PINOUTPUT;

    // Send data over signal pin
    for (i = 0; i < buflen; i++)
      SendByte(buf[i]);

    // Trailer
    PINHIGH;
    SHORTBITDELAY;

    PININPUT; // Pull-up is enabled from previous PINHIGH
    EIMSK |= (1 << INT0);
  }
  else if (bitTrigger)
  {
    // Buffer data from signal pin then write to serial port
    buflen = 3;
    bitTrigger = 0;
    if (ReadLeader() == 0)
    {
      while ((b = ReadByte()) != -1)
        buf[buflen++] = b;

      Serial.write(&buf[3], buflen - 3);
    }
  }

  goto main;
}

int main(int argc, char* argv[])
{
  setup();
  while (1) loop();
  return 0;
}

