/*
*   'UU' - Small AVR utility lib
* 
*   Copyright (c) 2014 ALM Co Ltd
*
*   Parts of code based in Wiring/Arduino.h;
*   Copyright (c) 2005-2006 David A. Mellis
*   
*   Permission is hereby granted, free of charge, to any person
*   obtaining a copy of this software and associated documentation files
*   (the "Software"), to deal in the Software without restriction,
*   including without limitation the rights to use, copy, modify, merge,
*   publish, distribute, sublicense, and/or sell copies of the Software,
*   and to permit persons to whom the Software is furnished to do so,
*   subject to the following conditions:
*   
*   The above copyright notice and this permission notice shall be included in all
*   copies or substantial portions of the Software.
*   
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*   SOFTWARE.
*   
*/

#include "uu.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

const uint8_t PROGMEM _pin_table_PGM[] = {

#if  defined(__AVR_ATmega328P__) 
  0,
#endif
#if defined(__AVR_AT90USB1286__)
  (int)&PINA,
#endif
#if defined(__AVR_ATmega644P__)
  (int)&PINA,
#endif
  (int)&PINB,
  (int)&PINC,
  (int)&PIND
  /*(int)&PORTE,*/
  /*(int)&PORTF*/
};

void 
uu_delay_ms(uint16_t ms) 
{
  while  (ms)
    {
      _delay_ms(1);
      ms--;
    }
}

void 
uu_delay_us(unsigned int us)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);

  // #if F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}


void 
uu_pin_mode (Pin pin, bool output)
{
  /* DDRD |= (1<<6); set pin 6 in PORTD to output  */
  /* DDRx &= ~(1<<6); set pin 6 in PORTD to input  */

  if (output)
    *PIN_TO_MODEREG(pin) |= PIN_TO_MASK(pin);
  else
    *PIN_TO_MODEREG(pin) &= ~PIN_TO_MASK(pin);

}

void 
uu_pin_digital_write (Pin pin, bool value)
{
  if (value)
    *PIN_TO_OUTREG(pin) |=  PIN_TO_MASK(pin);
  else
    *PIN_TO_OUTREG(pin) &=~ PIN_TO_MASK(pin);
}

bool 
uu_pin_digital_read (Pin pin)
{
  if ((*PIN_TO_PORTREG(pin) & PIN_TO_MASK(pin)) > 0)
    return HIGH;
  return LOW;
}

void 
uu_pin_shift_out(Pin dataPin, Pin clockPin, uint8_t bitOrder, uint8_t value)
{
  uint8_t mask;
  if (bitOrder == LSBFIRST) 
    {
      for (mask=0x01; mask; mask <<= 1) 
	{
	  uu_pin_digital_write(dataPin, value & mask);
	  uu_pin_digital_write(clockPin, HIGH);
	  uu_pin_digital_write(clockPin, LOW);
	}
    }
  else
    {
      for (mask=0x80; mask; mask >>= 1) 
	{
	  uu_pin_digital_write(dataPin, value & mask);
	  uu_pin_digital_write(clockPin, HIGH);
	  uu_pin_digital_write(clockPin, LOW);
	}
    }
}

uint8_t 
uu_pin_shift_in(Pin dataPin, Pin clockPin, uint8_t bitOrder)
{
  uint8_t mask, value=0;
  if (bitOrder == LSBFIRST) 
    {    
      for (mask=0x01; mask; mask <<= 1) 
	{
	  uu_pin_digital_write(clockPin, HIGH);
	  if (uu_pin_digital_read(dataPin)) value |= mask;
	  uu_pin_digital_write(clockPin, LOW);
	}
    }
  else
    {
      for (mask=0x80; mask; mask >>= 1) 
	{
	  uu_pin_digital_write(clockPin, HIGH);
	  if (uu_pin_digital_read(dataPin)) value |= mask;
	  uu_pin_digital_write(clockPin, LOW);
	}
    }
  return value;
}

void
uu_init(int flags)
{
  sei();
}

void
uu_usart_init()
{
  UBRR0H = (BAUD_PRESCALE >> 8);
  UBRR0L = BAUD_PRESCALE;

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  /* Just TX */
  UCSR0B |= /*_BV(RXEN0) | */ _BV(TXEN0); 
}

void uu_usart_putc(char c) 
{
  if (c == '\n') 
    {
      uu_usart_putc('\r');
    }

  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
}

void uu_usart_puts(char *s) 
{
  while (*s) 
    {
      uu_usart_putc(*s);
      s++;
    }
}

/* via https://github.com/sultanqasim/Stellarino/blob/master/stellarino_usart.c */
void uu_usart_puti(long i)
{
  char digs[12], reversed[12];
  uint8_t a = 0, b, neg = 0;

  if (i < 0)
    {
      neg = 1;
      i = -i;
    }

    do
      {
        b = i % 10;
        digs[a] = b + 48;// Convert to digit ASCII
        i /= 10;
        a++;
      } while (i);

    // Reverse the digits into most significant to least significant
    if (neg)
      {
        reversed[0] = '-';
        for (b = 1; b < a + neg; b++)
	  reversed[b] = digs[a - b];
        a += 1;// Extend the length of the string by 1 due to - sign
      }
    else for (b = 0; b < a; b++)
	   reversed[b] = digs[a - b - 1];

    reversed[a] = '\0';
    uu_usart_puts((char *) reversed);
}
