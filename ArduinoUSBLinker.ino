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

///////////////////////////////////////////////////////////////////////////////

// Calculates ticks from microseconds
#define MICROS(x) ((F_CPU / 1000000) * x)

#define LONGBITDELAY delayTicks(g_bitTimeSendLong)
#define SHORTBITDELAY delayTicks(g_bitTimeSendShort)

#define LONGWAIT MICROS(1000)

// Serial port
#define SERIALRATE 115200
#define SERIALTIMEOUT (F_CPU / (SERIALRATE >> 4))

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

// Approximate microseconds for each bit when sending
static uint16_t g_bitTimeSend = MICROS(136);
static uint16_t g_bitTimeSendLong = g_bitTimeSend >> 1;
static uint16_t g_bitTimeSendShort = g_bitTimeSend >> 2;

// Calculated leader timing for receive
static uint16_t g_bitTime, g_shortBitTime;

static volatile int8_t g_signalPin, g_bitTrigger;
static volatile uint16_t g_lastBitTicks;

///////////////////////////////////////////////////////////////////////////////
// Signal pin
///////////////////////////////////////////////////////////////////////////////

static inline void SignalPinInit()
{
  SignalPinInput();
  SignalPinHigh(); // Enable pull-up
  
  if (g_signalPin == 18) // INT0
  {
    EICRA = (1 << ISC01) | (1 << ISC00);
    PCICR = 0;
  }
  else if (g_signalPin == 19) // INT1
  {
    EICRA = (1 << ISC11) | (1 << ISC10);
    PCICR = 0;
  }
  else if (g_signalPin > 15)
  {
    PCICR = (1 << PCIE2);
    EICRA = 0;
  }
  else if (g_signalPin > 7)
  {
    PCICR = (1 << PCIE1);
    EICRA = 0;
  }
  else
  {
    PCICR = (1 << PCIE0);
    EICRA = 0;
  }
}

static inline void SignalPinEnable()
{
  if (g_signalPin == 18) // INT0
    EIMSK = (1 << INT0);
  else if (g_signalPin == 19) // INT1
    EIMSK = (1 << INT1);
  else if (g_signalPin > 15)
    PCMSK2 = (1 << (g_signalPin - 16));
  else if (g_signalPin > 7)
    PCMSK1 = (1 << (g_signalPin - 8));
  else
    PCMSK0 = (1 << g_signalPin);
}

static inline void SignalPinDisable()
{
  EIMSK = 0;
  PCMSK2 = 0;
  PCMSK1 = 0;
  PCMSK0 = 0;
}

static inline void SignalPinInput()
{
  if (g_signalPin > 15)
    DDRD &= ~(1 << (g_signalPin - 16));
  else if (g_signalPin > 7)
    DDRC &= ~(1 << (g_signalPin - 8));
  else
    DDRB &= ~(1 << g_signalPin);
}

static inline void SignalPinOutput()
{
  if (g_signalPin > 15)
    DDRD |= (1 << (g_signalPin - 16));
  else if (g_signalPin > 7)
    DDRC |= (1 << (g_signalPin - 8));
  else
    DDRB |= (1 << g_signalPin);
}

static inline void SignalPinHigh()
{
  if (g_signalPin > 15)
    PORTD |= (1 << (g_signalPin - 16));
  else if (g_signalPin > 7)
    PORTC |= (1 << (g_signalPin - 8));
  else
    PORTB |= (1 << g_signalPin);
}

static inline void SignalPinLow()
{
  if (g_signalPin > 15)
    PORTD &= ~(1 << (g_signalPin - 16));
  else if (g_signalPin > 7)
    PORTC &= ~(1 << (g_signalPin - 8));
  else
    PORTB &= ~(1 << g_signalPin);
}

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
  SignalPinHigh();
  LONGBITDELAY;
  SignalPinLow();
  LONGBITDELAY;
}

static inline void Send0()
{
  SignalPinHigh();
  SHORTBITDELAY;
  SignalPinLow();
  SHORTBITDELAY;
  SignalPinHigh();
  SHORTBITDELAY;
  SignalPinLow();
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
static inline void Signal()
{
  g_lastBitTicks = TCNT1;
  TCNT1 = 0;
  g_bitTrigger = 1;
}

ISR(INT0_vect)
{
  Signal();
}

ISR(INT1_vect)
{
  Signal();
}

/* Conflicts with serial port
ISR(PCINT2_vect)
{
  if (PIND & (1 << (g_signalPin - 16)))
    Signal();
}
*/

ISR(PCINT1_vect)
{
  if (PINC & (1 << (g_signalPin - 8)))
    Signal();
}

ISR(PCINT0_vect)
{
  if (PINB & (1 << g_signalPin))
    Signal();
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
  g_signalPin = 18; // INT0
  
  Serial.begin(SERIALRATE);

  cli();

  // Set up timer1 to count ticks  
  TCCR1B = (1 << CS10);
  TCCR1A = 0;

  // Signal pin timer interrupt
  SignalPinInit();
  SignalPinEnable();

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
      SignalPinDisable();
  
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
  
      if (strncmp("USBLINKER:SELECT:", (const char*)&buf[3], 17) == 0)
      {
        buf[buflen] = '\0';
        g_signalPin = atoi((const char*)&buf[20]);
        
        SignalPinInit();
      }
      else if (strncmp("USBLINKER:INIT:", (const char*)&buf[3], 15) == 0)
      {
        buf[buflen] = '\0';
        int8_t oldpin = g_signalPin;
        g_signalPin = atoi((const char*)&buf[18]);
        
        SignalPinInput();
        SignalPinHigh();
        
        g_signalPin = oldpin;
      }
      else if (strncmp("USBLINKER:BITTIME:", (const char*)&buf[3], 18) == 0)
      {
        buf[buflen] = '\0';
        g_bitTimeSend = atoi((const char*)&buf[21]);

        if (g_bitTimeSend < 8 || g_bitTimeSend > 136)
          g_bitTimeSend = 136;

        g_bitTimeSend = MICROS(g_bitTimeSend);
        g_bitTimeSendLong = g_bitTimeSend >> 1;
        g_bitTimeSendShort = g_bitTimeSend >> 2;
      }
      else
      {
        SignalPinOutput();
    
        // Send data over signal pin
        for (i = 0; i < buflen; i++)
          SendByte(buf[i]);
    
        // Trailer
        SignalPinHigh();
        SHORTBITDELAY;
    
        SignalPinInput(); // Pull-up is enabled from previous PINHIGH
      }
      
      SignalPinEnable();
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

