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
Known issues:
  When applying power to the Arduino and the ESC at the same time the ESC will
  arm before we are able to set the signal pin HIGH. It will still work but be
  careful. Best is to connect and power the Arduino first then power up the
  ESC to ensure that it is held in the bootloader (there should be no beeps
  from the ESC).

  Message sizes of more than 1021 bytes (total) not not supported and will
  likely crash this software. The STK500 firmware is currently limited to 281
  bytes so it is not an issue at this time.

  The default signaling rate over the servo wire is 136µs per bit or about
  7 kbps which is close to what an actual USB Linker uses. The fastest
  supported signaling rate is 20µs (~50 kbps). See BITTIME below to change
  the signaling rate.

  Note that the default serial port rate is 115200 and this is separate from
  the servo wire signaling rate. Make sure your tools are using the same
  serial port rate. See SERIALRATE below.
  
Version history:
  0.4 General code cleanup. Use bit shifting for all multiply and divide
      operations when possible.
      
      Add support for easily changing the signal pin. PD2/INT0 and PD3/INT1
      support full speed signaling while the other pins must be run slower
      due to them having more overhead. Changing the signaling pin requires
      updating ULPORT, ULPIN, PINREAD, PINISR, PININIT, PINENABLE, and
      PINDISABLE. Some preconfigured defaults are below.

  0.3 Set default bit rate to the correct 136µs signaling. Update comments.
      
  0.2 Minor code cleanup in ReadLeader().  Added main() implementation to
      prevent Arduino overhead. Maximum speed now around 50 kbps (20µs).

  0.1 Initial release. Maximum speed around 15 kbps (64µs).
*/

// Macro expansion macros
#define CONCAT0(x,y) x##y
#define CONCAT(x,y) CONCAT0(x,y)

///////////////////////////////////////////////////////////////////////////////
// Signal pin (default: PD2/INT0)
///////////////////////////////////////////////////////////////////////////////
#define ULPORT D
#define ULPIN 2

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

// PD2/INT0
#define PINREAD    (1)
#define PINISR     INT0_vect
#define PININIT    (EICRA = (1 << ISC01) | (1 << ISC00))
#define PINENABLE  (EIMSK = (1 << INT0))
#define PINDISABLE (EIMSK = 0)

// PD3/INT1
/*
#define PINREAD    (1)
#define PINISR     INT1_vect
#define PININIT    (EICRA = (1 << ISC11) | (1 << ISC10))
#define PINENABLE  (EIMSK = (1 << INT1))
#define PINDISABLE (EIMSK = 0)
*/

// PB7..0/PCINT7..0
// PINISR must be manually updated depending on which pin you are using; the
// rest should work automatically based on ULPORT and ULPIN.
/*
#define PINREAD    (CONCAT(PIN,ULPORT) & (1 << CONCAT(PIN,ULPORTPIN)))
#define PINISR     PCINT0_vect
#define PININIT    (PCICR = (1 << CONCAT(PCIE,ULPIN)))
#define PINENABLE  (PCMSK0 = (1 << CONCAT(PCINT,ULPIN)))
#define PINDISABLE (PCMSK0 = 0)
*/

///////////////////////////////////////////////////////////////////////////////

// Calculates ticks from microseconds
#define MICROS(x) ((F_CPU / 1000000) * x)

#define DELAY delayTicks

// Approximate microseconds for each bit when sending
#define BITTIME MICROS(136)
//#define BITTIME MICROS(20)

#define LONGBITDELAY DELAY(BITTIME >> 1)
#define SHORTBITDELAY DELAY(BITTIME >> 2)

#define LONGWAIT MICROS(1000)

// Serial port
#define SERIALRATE 115200
#define SERIALTIMEOUT (F_CPU / (SERIALRATE >> 4))

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

// Calculated leader timing for receive
static uint16_t g_bitTime, g_shortBitTime;

static volatile int8_t g_bitTrigger;
static volatile uint16_t g_lastBitTicks;

///////////////////////////////////////////////////////////////////////////////
// SENDING on signal pin
///////////////////////////////////////////////////////////////////////////////
static inline void delayTicks(uint16_t count)
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

static inline void SendByte(uint8_t b)
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
// RECEIVE on signal pin
///////////////////////////////////////////////////////////////////////////////
ISR(PINISR)
{
  if (PINREAD)
  {
    g_lastBitTicks = TCNT1;
    TCNT1 = 0;
    g_bitTrigger = 1;
  }
}

// NOTE: This has a maximum wait time of about 2000µs before it overflows
static inline int16_t WaitPinHighLow(uint16_t timeout)
{
  while (g_bitTrigger == 0)
    if (TCNT1 > timeout)
      return -1;

  g_bitTrigger = 0;

  return g_lastBitTicks;
}

static inline int8_t ReadBit()
{
  int16_t timer = WaitPinHighLow(g_bitTime << 1);

  if (timer < 0)
    return -1;
  else if (timer < g_shortBitTime)
  {
    if (WaitPinHighLow(g_bitTime << 1) < 0) return -1;
    return 0;
  }
  else
    return 1;
}

static int8_t ReadLeader()
{
  // Skip the first few to let things stabilize
  if (WaitPinHighLow(LONGWAIT) < 0) return -1;
  if (WaitPinHighLow(LONGWAIT) < 0) return -2;
  if (WaitPinHighLow(LONGWAIT) < 0) return -3;
  if (WaitPinHighLow(LONGWAIT) < 0) return -4;
  if (WaitPinHighLow(LONGWAIT) < 0) return -5;
  if (WaitPinHighLow(LONGWAIT) < 0) return -6;
  if (WaitPinHighLow(LONGWAIT) < 0) return -7;
  if (WaitPinHighLow(LONGWAIT) < 0) return -8;
  if (WaitPinHighLow(LONGWAIT) < 0) return -9;

  int16_t timer;
  g_bitTime = 0;

  // Average the next 8 to get our bit timing
  // NOTE: this has a window of around 4000µs before we overflow
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -10;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -11;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -12;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -13;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -14;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -15;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -16;
  g_bitTime += timer;
  if ((timer = WaitPinHighLow(LONGWAIT)) < 0) return -17;
  g_bitTime += timer;

  g_bitTime >>= 3;
  g_shortBitTime = (g_bitTime >> 1) + (g_bitTime >> 2);

  // Read until we get a 0 bit
  while (1)
  {
    int8_t b = ReadBit();

    if (b < 0)
      return -18;
    else if (b == 0)
      break;
  }

  return 0;
}

static inline int16_t ReadByte()
{
  int16_t b;
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
// Arduino
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  PININPUT;
  PINHIGH; // Enable pull-up
  Serial.begin(SERIALRATE);

  cli();

  // Set up timer1 to count ticks  
  TCCR1B = (1 << CS10);
  TCCR1A = 0;

  // Signal pin timer interrupt
  PININIT;
  PINENABLE;

  sei();
}

void loop()
{
  // The buffer always has the leader at the start
  uint8_t buf[1024] = { 0xFF, 0xFF, 0x7F };
  int16_t buflen, b, i;

  while (1)
  {
    b = Serial.read();
    if (b >= 0)
    {
      // Disable signal pin timer interrupt
      PINDISABLE;
  
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
      PINENABLE;
    }
    else if (g_bitTrigger)
    {
      // Buffer data from signal pin then write to serial port

      g_bitTrigger = 0;
      buflen = 3;

      if (ReadLeader() == 0)
      {
        while ((b = ReadByte()) >= 0)
          buf[buflen++] = b;
  
        Serial.write(&buf[3], buflen - 3);
      }
    }
  }
}

int main(int argc, char* argv[])
{
  setup();
  while (1) loop();
  return 0;
}

