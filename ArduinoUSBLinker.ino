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
  7 kbps which is close to what an actual USB Linker uses.

  Note that the default serial port rate is 115200 and this is separate from
  the servo wire signaling rate. Make sure your tools are using the same
  serial port rate. See SERIALRATE below.
  
Version history:
  0.5 Can now be inserted into MultiWii code base.
  
      Remove interrupt code. Some MultiWii boards can't do interrupts on all
      of their ESC signal pins. Maximum speed around 31 kbps (32µs).
      
      Add runtime configuration commands. The signal pin and bit rate are now
      selectable at runtime. See the README for more information.
  
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

#if defined(MINTHROTTLE)
#define MULTIWII
#define AUL_SERIALRATE 19200
#else
#define AUL_SERIALRATE 115200
#endif

#define AUL_SERIALTIMEOUT (F_CPU / (AUL_SERIALRATE >> 4))

// Calculates ticks from microseconds
#define AUL_MICROS(x) ((F_CPU / 1000000) * x)

#define AUL_LONGBITDELAY delayTicks(g_bitTimeSendLong)
#define AUL_SHORTBITDELAY delayTicks(g_bitTimeSendShort)

#define AUL_LONGWAIT AUL_MICROS(1000)

#define AUL_MIN_BITTIME 8
#define AUL_MAX_BITTIME 136
#define AUL_DEFAULT_BITTIME 136

#define AUL_PININPUT  ((*g_signalDDR)  &= ~(1 << g_signalPinPortNum))
#define AUL_PINOUTPUT ((*g_signalDDR)  |= (1 << g_signalPinPortNum))
#define AUL_PINHIGH   ((*g_signalPORT) |= (1 << g_signalPinPortNum))
#define AUL_PINLOW    ((*g_signalPORT) &= ~(1 << g_signalPinPortNum))

#define AUL_PINREAD   ((*g_signalPIN) & (1 << g_signalPinPortNum))

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

// Approximate microseconds for each bit when sending
static uint16_t g_bitTimeSend = AUL_MICROS(AUL_DEFAULT_BITTIME);
static uint16_t g_bitTimeSendLong = g_bitTimeSend >> 1;
static uint16_t g_bitTimeSendShort = g_bitTimeSend >> 2;

// Calculated leader timing for receive
static uint16_t g_bitTime, g_shortBitTime;

static volatile uint8_t* g_signalDDR;
static volatile uint8_t* g_signalPORT;
static volatile uint8_t* g_signalPIN;
static int8_t g_signalPinPortNum, g_signalPinNum;

///////////////////////////////////////////////////////////////////////////////
// Serial port.  MultiWii helpers.
///////////////////////////////////////////////////////////////////////////////

#if !defined(MULTIWII)

#define AUL_SerialInit(x) Serial.begin(AUL_SERIALRATE)
#define AUL_SerialAvailable() Serial.available()
#define AUL_SerialRead() Serial.read()
#define AUL_SerialWrite(x) Serial.write(x)
#define AUL_SerialWriteBuf(x,y) Serial.write(x,y)
#define AUL_SerialWriteStr(x) Serial.write((const char*)x)

#else // MULTIWII

// Tried to use the MultiWii serial port code but it was even worse than this
// spin-loop method as far as serial errors.

static volatile uint8_t* g_serialUCSRA;
static volatile uint8_t* g_serialUDR;
static uint8_t g_serialRXC, g_serialUDRE;

static void AUL_SerialInit(uint8_t port)
{
  #define BAUD AUL_SERIALRATE
  #include <util/setbaud.h>

  #if USE_2X
    #define AUL_INIT2X(x) UCSR##x##A |= (1 << U2X0)
  #else 
    #define AUL_INIT2X(x) UCSR##x##A &= ~(1 << U2X##x);
  #endif // USE_2X
  
  #define AUL_INIT_PORT(x) \
    UBRR##x##H = UBRRH_VALUE; \
    UBRR##x##L = UBRRL_VALUE; \
    AUL_INIT2X(x); \ 
    UCSR##x##C = (1 << UCSZ##x##1) | (1 << UCSZ##x##0); \
    UCSR##x##B = (1 << RXEN##x) | (1 << TXEN##x); \
    g_serialUCSRA = &UCSR##x##A; \
    g_serialUDR = &UDR##x; \
    g_serialRXC = (1 << RXC##x); \
    g_serialUDRE = (1 << UDRE##x);
  
  switch (port)
  {
  case 0:
    AUL_INIT_PORT(0)
    break;
  #if defined(UBRR1H)
  case 1:
    AUL_INIT_PORT(1)
    break;
  #endif
  #if defined(UBRR2H)
  case 2:
    AUL_INIT_PORT(2)
    break;
  #endif
  #if defined(UBRR3H)
  case 3:
    AUL_INIT_PORT(3)
    break;
  #endif
  #if defined(UBRR4H)
  case 4:
    AUL_INIT_PORT(4)
    break;
  #endif
  default:
    break;
  }
}

#define AUL_SerialAvailable() ((*g_serialUCSRA) & g_serialRXC)

#define AUL_SerialRead() (*g_serialUDR)

#define AUL_SerialWrite(x) { while (!((*g_serialUCSRA) & g_serialUDRE)); (*g_serialUDR) = (x); }

static void AUL_SerialWriteBuf(const uint8_t* b, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++)
    AUL_SerialWrite(b[i]);
}

static void AUL_SerialWriteStr(const char* b)
{
  uint16_t i;
  for (i = 0; b[i] != '\0'; i++)
    AUL_SerialWrite(b[i]);
}

#endif // MULTIWII

///////////////////////////////////////////////////////////////////////////////
// Signal pin
///////////////////////////////////////////////////////////////////////////////

// Clear all timers and PWM settings
static void DisableAllTimers()
{
  #define AUL_RESET_PORT(x) \
    TCCR##x##B = 0; \
    TCCR##x##A = 0;
  
  #if defined(TCCR0B)
    AUL_RESET_PORT(0)
  #endif
  #if defined(TCCR1B)
    AUL_RESET_PORT(1)
  #endif
  #if defined(TCCR2B)
    AUL_RESET_PORT(2)
  #endif
  #if defined(TCCR3B)
    AUL_RESET_PORT(3)
  #endif
  #if defined(TCCR4B)
    AUL_RESET_PORT(4)
  #endif
  #if defined(TCCR5B)
    AUL_RESET_PORT(5)
  #endif
  #if defined(TCCR6B)
    AUL_RESET_PORT(6)
  #endif
}

static void SignalPinStatus(char* buf)
{
  #define AUL_WRITE_PORT_INFO(x) \
    *pos++ = #x[0]; \
    itoa(pincnt, pos, 10); \
    pos = strchr(pos, '\0'); \
    *pos++ = ':'; \
    pincnt += 8;

  char* pos = buf;
  int8_t pincnt = 0;

  *pos++ = 'P';
  *pos++ = 'I';
  *pos++ = 'N';
  *pos++ = 'S';
  *pos++ = ':';
  
  #if defined(PORTB)
    AUL_WRITE_PORT_INFO(B)
  #endif
  #if defined(PORTC)
    AUL_WRITE_PORT_INFO(C)
  #endif
  #if defined(PORTD)
    AUL_WRITE_PORT_INFO(D)
  #endif
  #if defined(PORTE)
    AUL_WRITE_PORT_INFO(E)
  #endif
  #if defined(PORTF)
    AUL_WRITE_PORT_INFO(F)
  #endif
  #if defined(PORTG)
    AUL_WRITE_PORT_INFO(G)
  #endif
  #if defined(PORTH)
    AUL_WRITE_PORT_INFO(H)
  #endif
  #if defined(PORTI)
    AUL_WRITE_PORT_INFO(I)
  #endif
  #if defined(PORTJ)
    AUL_WRITE_PORT_INFO(J)
  #endif
  #if defined(PORTK)
    AUL_WRITE_PORT_INFO(K)
  #endif
  #if defined(PORTL)
    AUL_WRITE_PORT_INFO(L)
  #endif
  
  #if defined(PORTA)
    AUL_WRITE_PORT_INFO(A)
  #endif
  
  *pos = '\0';
}

static void SignalPinInit(int8_t pin)
{
  #define AUL_SETUP_PORT(x) \
    if (pin < (pincnt += 8)) \
    { \
      g_signalDDR = &DDR##x; \
      g_signalPORT = &PORT##x; \
      g_signalPIN = &PIN##x; \
      g_signalPinPortNum = pin - (pincnt - 8); \
      goto finished; \
    }
  
  int8_t pincnt = 0;
  
  g_signalPinNum = pin;

  #if defined(PORTB)
    AUL_SETUP_PORT(B);
  #endif
  #if defined(PORTC)
    AUL_SETUP_PORT(C);
  #endif
  #if defined(PORTD)
    AUL_SETUP_PORT(D);
  #endif
  #if defined(PORTE)
    AUL_SETUP_PORT(E);
  #endif
  #if defined(PORTF)
    AUL_SETUP_PORT(F);
  #endif
  #if defined(PORTG)
    AUL_SETUP_PORT(G);
  #endif
  #if defined(PORTH)
    AUL_SETUP_PORT(H);
  #endif
  #if defined(PORTI)
    AUL_SETUP_PORT(I);
  #endif
  #if defined(PORTJ)
    AUL_SETUP_PORT(J);
  #endif
  #if defined(PORTK)
    AUL_SETUP_PORT(K);
  #endif
  #if defined(PORTL)
    AUL_SETUP_PORT(L);
  #endif
  
  #if defined(PORTA)
    AUL_SETUP_PORT(A);
  #endif

finished:  
  AUL_PININPUT;
  AUL_PINHIGH; // Enable pull-up
}

///////////////////////////////////////////////////////////////////////////////
// SENDING on signal pin
///////////////////////////////////////////////////////////////////////////////
static void delayTicks(uint16_t count)
{
  TCNT1 = 0;
  while (TCNT1 < count);
}

static void Send1()
{
  AUL_PINHIGH;
  AUL_LONGBITDELAY;
  AUL_PINLOW;
  AUL_LONGBITDELAY;
}

static void Send0()
{
  AUL_PINHIGH;
  AUL_SHORTBITDELAY;
  AUL_PINLOW;
  AUL_SHORTBITDELAY;
  AUL_PINHIGH;
  AUL_SHORTBITDELAY;
  AUL_PINLOW;
  AUL_SHORTBITDELAY;
}

static void SendByte(uint8_t b)
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

// NOTE: This has a maximum wait time of about 2000µs before it overflows
static int16_t WaitPinHighLow(uint16_t timeout)
{
  while (AUL_PINREAD)
    if (TCNT1 > timeout)
      return -1;

  while (!AUL_PINREAD)
    if (TCNT1 > timeout)
      return -1;

  uint16_t last = TCNT1;
  TCNT1 = 0;
  
  return last;
}

static int8_t ReadBit()
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
  uint8_t i;
  
  // Skip the first few to let things stabilize
  for (i = 0; i < 9; i++)
    if (WaitPinHighLow(AUL_LONGWAIT) < 0) return -1;

  int16_t timer;
  g_bitTime = 0;

  // Average the next 8 to get our bit timing
  // NOTE: this has a window of around 4000µs before we overflow
  for (i = 0; i < 8; i++)
  {
    if ((timer = WaitPinHighLow(AUL_LONGWAIT)) < 0) return -1;
    g_bitTime += timer;
  }
  
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

static int16_t ReadByte()
{
  int16_t b;
  int8_t b2, i;

  for (i = 0; i < 8; i++)
  {
    if ((b2 = ReadBit()) < 0) return -1;
    b |= b2 << i;
  }
  
  return b;
}

///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////

void AUL_loop(uint8_t port)
{
  // Disable interrupts and timers
  cli();
  DisableAllTimers();
  AUL_SerialInit(port);
  
  #if !defined(MULTIWII)
    sei();
  #endif
  
  // Set up timer1 to count ticks  
  TCCR1B = (1 << CS10);
  TCCR1A = 0;

  SignalPinInit(18); // INT0

  // The buffer always has the leader at the start
  uint8_t buf[1024] = { 0xFF, 0xFF, 0x7F };
  int16_t buflen, i;

  while (1)
  {
    if (AUL_SerialAvailable())
    {
      buflen = 3;
      buf[buflen++] = AUL_SerialRead();
  
      // Buffer data until the serial timeout
      TCNT1 = 0;
      do {
         if (AUL_SerialAvailable())
         {
           buf[buflen++] = AUL_SerialRead();
           TCNT1 = 0;
         }
      } while (TCNT1 < AUL_SERIALTIMEOUT);
      
      if (buf[3] == '$' && buf[4] == 'M' && buf[5] == '<')
      {
        buf[buflen] = '\0';
        
        switch(buf[6])
        {
        case 'B': { // BITTIME
          g_bitTimeSend = atoi((const char*)&buf[7]);
  
          if (g_bitTimeSend < AUL_MIN_BITTIME)
            g_bitTimeSend = AUL_MIN_BITTIME;
          else if (g_bitTimeSend > AUL_MAX_BITTIME)
            g_bitTimeSend = AUL_MAX_BITTIME;
  
          g_bitTimeSend = AUL_MICROS(g_bitTimeSend);
          g_bitTimeSendLong = g_bitTimeSend >> 1;
          g_bitTimeSendShort = g_bitTimeSend >> 2;
          break; }
        case 'I': { // INIT
          int8_t oldpin = g_signalPinNum;
          SignalPinInit(atoi((const char*)&buf[7]));
          AUL_PININPUT;
          AUL_PINHIGH;
          SignalPinInit(oldpin);
          break; }
        case 'P': // SELECT PORT
          SignalPinInit(atoi((const char*)&buf[7]));
          break;
        default:
          break;
        }
        
        // Send status afterwards
        char* pos = (char*)&buf[3];
        *pos++ = 'P';
        itoa(g_signalPinNum, pos, 10);
        pos = strchr(pos, '\0');
        *pos++ = ':';
        *pos++ = 'B';
        itoa(g_bitTimeSend / (F_CPU / 1000000), pos, 10);
        pos = strchr(pos, '\0');
        *pos++ = ':';
        
        SignalPinStatus(pos);
        
        AUL_SerialWriteStr((const char*)&buf[3]);
        AUL_SerialWrite('\n');
      }
      else
      {
        AUL_PINOUTPUT;
    
        // Send data over signal pin
        for (i = 0; i < buflen; i++)
          SendByte(buf[i]);
    
        // Trailer
        AUL_PINHIGH;
        AUL_SHORTBITDELAY;

        AUL_PININPUT; // Pull-up is enabled from previous PINHIGH
      }
    }
    else if (AUL_PINREAD)
    {
      buflen = 3;

      // Buffer data from signal pin then write to serial port
      if (ReadLeader() == 0)
      {
        int16_t b;
        
        while ((b = ReadByte()) >= 0)
          buf[buflen++] = b;
  
        AUL_SerialWriteBuf(&buf[3], buflen - 3);
      }
    }
  }
}

#if !defined(MULTIWII)

int main(int argc, char* argv[])
{
  AUL_loop(0);
  return 0;
}

#endif // MULTIWII

