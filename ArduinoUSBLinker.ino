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

  Message sizes of more than 297 bytes (total) not not supported and will
  likely crash this software. The STK500 firmware is currently limited to 281
  bytes so it is not an issue at this time.

  Note that the default serial port rate is 19200 and this is separate from
  the servo wire signaling rate. Make sure your tools are using the same
  serial port rate. See SERIALRATE below.
  
Version history:
  ??? Buffer size now at 300 bytes for low memory devices. 19200 baud is now
      the default to be compatible with more (all?) devices. Fix for detecting
      MultiWii.

  0.7 Code shrink. Removed the 'I' command because 'P' does the same thing.
      Now adds ~1800 bytes to MultiWii.
      
  0.6 Rework of the whole bit timing system. Improved performance across the
      board. Serial performance in MultiWii is better now and can be run at the
      full 115200 bps. Size is also reduced by a few hundred bytes. Default
      changed to 32µs bit timing so more people can test. I'm not sure if 8 Mhz
      ESC's and/or ESC's without external oscillators will be able to run at
      that signaling rate. Default 115200 bps for MultiWii (NOTE: this has
      changed to 19200 in the most recent version).
  
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

// Check for MultiWii
#if defined(MSP_VERSION)
#define MULTIWII
#endif

#define AUL_SERIALRATE 19200

#define AUL_MIN_BITTIME 8
#define AUL_MAX_BITTIME 136
#define AUL_DEFAULT_BITTIME 32

#define AUL_BUFSIZE 300

#define AUL_SERIALTIMEOUT ((F_CPU >> 7) / (AUL_SERIALRATE >> 4))

#define AUL_PININPUT  ((*g_signalDDR)  &= ~(g_signalPinPortNum))
#define AUL_PINOUTPUT ((*g_signalDDR)  |=  (g_signalPinPortNum))
#define AUL_PINHIGH   ((*g_signalPORT) |=  (g_signalPinPortNum))
#define AUL_PINLOW    ((*g_signalPORT) &= ~(g_signalPinPortNum))

#define AUL_PINREAD   ((*g_signalPIN) & (g_signalPinPortNum))

#define AUL_DELAYTICKS(x) \
  TCNT2 = 0; \
  while (TCNT2 < (x));

#define AUL_SYNC_PRESCALER \
  GTCCR = (1 << PSRASY); \
  while (GTCCR & (1 << PSRASY));

///////////////////////////////////////////////////////////////////////////////
// Globals
///////////////////////////////////////////////////////////////////////////////

// Approximate microseconds for each bit when sending
static uint8_t g_bitTimeSend = AUL_DEFAULT_BITTIME;
static uint8_t g_bitTimeSendHalf = (AUL_DEFAULT_BITTIME >> 1);

// Calculated leader timing for receive
static uint8_t g_bitTime, g_shortBitTime;

static volatile uint8_t* g_signalDDR;
static volatile uint8_t* g_signalPORT;
static volatile uint8_t* g_signalPIN;
static int8_t g_signalPinPortNum, g_signalPinNum;

///////////////////////////////////////////////////////////////////////////////
// stdlib type utility functions (mostly to save space)
///////////////////////////////////////////////////////////////////////////////

// Byte to ASCII base 10
// Returns the address of the null terminator
static char* btoa10(uint8_t n, char *b)
{
   uint8_t i = 0, s;
   
   do {
      s = n % 10;
      n = n / 10;
      b[i++] = '0' + s;
   } while (n > 0);

   b[i] = '\0';
   
   // Reverse the string (uint8 only 3 chars max)
   switch (i)
   {
   case 1:
     break;
   case 2:
     s = b[0];
     b[0] = b[1];
     b[1] = s;
     break;
   case 3:
   default:
     s = b[0];
     b[0] = b[2];
     b[2] = s;
     break;
   }

   return &b[i];
}

// ASCII to byte
static uint8_t atob(const char* s)
{
  uint8_t b = 0;
  while (*s) b = (b << 3) + (b << 1) + (*s++ - '0');
  return(b);
}

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

#define AUL_SerialAvailable() \
  ((*g_serialUCSRA) & g_serialRXC)

#define AUL_SerialRead() \
  (*g_serialUDR)

#define AUL_SerialWrite(x) \
  { while (!((*g_serialUCSRA) & g_serialUDRE)); (*g_serialUDR) = (x); }

static void AUL_SerialWriteBuf(const uint8_t* b, int16_t len)
{
  int16_t i;
  for (i = 0; i < len; i++)
    AUL_SerialWrite(b[i]);
}

static void AUL_SerialWriteStr(const char* b)
{
  int16_t i;
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
    pos = btoa10(pincnt, pos); \
    *pos++ = ':'; \
    pincnt += 8;

  char* pos = buf;
  int8_t pincnt = 0;

  pos[0] = 'P';
  pos[1] = 'I';
  pos[2] = 'N';
  pos[3] = 'S';
  pos[4] = ':';
  pos += 5;
  
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
      g_signalPinPortNum = (1 << (pin - (pincnt - 8))); \
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
  AUL_PINHIGH; // Enable pull-up
  AUL_PININPUT;
}

///////////////////////////////////////////////////////////////////////////////
// SENDING on signal pin
///////////////////////////////////////////////////////////////////////////////

static void SendByte(uint8_t b)
{
  uint8_t i;
  for (i = 1; i; i <<= 1)
  {
    if (b & i)
    {
      AUL_PINHIGH;
      AUL_DELAYTICKS(g_bitTimeSend);
      AUL_PINLOW;
      AUL_DELAYTICKS(g_bitTimeSend);
    }
    else
    {
      AUL_PINHIGH;
      AUL_DELAYTICKS(g_bitTimeSendHalf);
      AUL_PINLOW;
      AUL_DELAYTICKS(g_bitTimeSendHalf);
      AUL_PINHIGH;
      AUL_DELAYTICKS(g_bitTimeSendHalf);
      AUL_PINLOW;
      AUL_DELAYTICKS(g_bitTimeSendHalf);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// RECEIVE on signal pin
///////////////////////////////////////////////////////////////////////////////

#define AUL_SPINPINHIGH \
  TCNT2 = 0; \
  while (AUL_PINREAD) { if (TCNT2 > 250) goto timeout; }

#define AUL_SPINPINLOW \
  TCNT2 = 0; \
  while (!AUL_PINREAD) { if (TCNT2 > 250) goto timeout; }

#define AUL_READBIT \
  AUL_SPINPINLOW \
  AUL_SPINPINHIGH \
  if (TCNT2 <= g_shortBitTime) \
  { \
    AUL_SPINPINLOW \
    AUL_SPINPINHIGH \
    b = 0; \
  } \
  else \
    b = 1;

static int8_t ReadLeader()
{
  uint8_t i;
  
  // Skip the first few to let things stabilize
  for (i = 0; i < 9; i++)
  {
    AUL_SPINPINHIGH
    AUL_SPINPINLOW
  }

  AUL_SPINPINHIGH
  AUL_SPINPINLOW
  AUL_SPINPINHIGH
  g_bitTime = TCNT2;

  g_shortBitTime = (g_bitTime >> 1) + (g_bitTime >> 2);

  // Read until we get a 0 bit
  while (1)
  {
    uint8_t b;
    AUL_READBIT // Sets b to the bit value
    
    if (!b)
      return 0;
  }

timeout:
  return -1;
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
    // Re-enable interrupts for Serial
    sei();
  #else
    #if defined(BUZZERPIN_OFF)
      BUZZERPIN_OFF;
    #endif
    #if defined(LEDPIN_OFF)
      LEDPIN_OFF;
    #endif
  #endif
  
  // Set timer2 to count ticks/8
  TCCR2B = (1 << CS21);

  SignalPinInit(18); // PD2/INT0

  // The buffer always has the leader at the start
  uint8_t buf[AUL_BUFSIZE] = { 0xFF, 0xFF, 0x7F };
  uint8_t lastPin = 0;
  int16_t buflen, i;

  while (1)
  {
    if (AUL_SerialAvailable())
    {
      buflen = 3;
      buf[buflen++] = AUL_SerialRead();
  
      // Temporarily set timer2 to count ticks/128
      TCCR2B = (1 << CS22) | (1 << CS20);  
      AUL_SYNC_PRESCALER;
      TCNT2 = 0;     
      // Buffer data until the serial timeout
      do {
         if (AUL_SerialAvailable())
         {
           buf[buflen++] = AUL_SerialRead();
           TCNT2 = 0;
         }
      } while (TCNT2 < AUL_SERIALTIMEOUT);
      
      // Set timer2 back to normal
      TCCR2B = (1 << CS21);
      
      if (buf[3] == '$' && buf[4] == 'M' && buf[5] == '<')
      {
        buf[buflen] = '\0';
        
        switch(buf[6])
        {
        case 'B': { // BITTIME
          uint8_t t = atob((const char*)&buf[7]);
  
          if (t < AUL_MIN_BITTIME)
            t = AUL_MIN_BITTIME;
          else if (t > AUL_MAX_BITTIME)
            t = AUL_MAX_BITTIME;
  
          g_bitTimeSend = t;
          g_bitTimeSendHalf = (t >> 1);
          break; }
        case 'P': // SELECT PORT
          SignalPinInit(atob((const char*)&buf[7]));
          break;
        default:
          break;
        }
        
        // Send status afterwards
        char* pos = (char*)&buf[3];
        *pos++ = 'P';
        pos = btoa10(g_signalPinNum, pos);
        pos[0] = ':';
        pos[1] = 'B';
        pos += 2;
        pos = btoa10(g_bitTimeSend, pos);
        *pos++ = ':';
        
        SignalPinStatus(pos);
        
        AUL_SerialWriteStr((const char*)&buf[3]);
        AUL_SerialWrite('\n');
      }
      else
      {
        AUL_PINOUTPUT;
        AUL_SYNC_PRESCALER;

        // Send data over signal pin
        for (i = 0; i < buflen; i++)
          SendByte(buf[i]);
    
        // Trailer
        AUL_PINHIGH;
        AUL_DELAYTICKS(g_bitTimeSendHalf);

        AUL_PININPUT; // Pull-up is enabled from previous PINHIGH
        lastPin = 1;
      }
    }
    else
    {
      // Here we look for a low to high transition on the signal pin
      uint8_t curPin = AUL_PINREAD;
      
      if (!lastPin && curPin)
      {
        AUL_SYNC_PRESCALER;
        
        // Buffer data from signal pin then write to serial port
        if (ReadLeader() == 0)
        {
          uint8_t i, byt, b;
          buflen = 3;
          
          // Read bytes until timeout
          while (1)
          {
            for (i = 0, byt = 0; i < 8; i++)
            {
              AUL_READBIT // Sets b to the bit value
              byt |= b << i;
            }
            
            buf[buflen++] = byt;
          }

timeout:
          AUL_SerialWriteBuf(&buf[3], buflen - 3);
        }
      }

      lastPin = curPin;
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

