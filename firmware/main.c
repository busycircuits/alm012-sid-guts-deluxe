/*
*   ALM014 'SID GUTS DULUXE' firmware 
*   
*   Copyright (c) 2014 ALM Co Ltd
*
*   Parts based on code written by Alexis Kotlowy, released in Public Domain. 
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
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "lookup.h"

/* AVR Pins */
#define PIN_MULT_IN PIN_A0 
#define PIN_MULT_A PIN_A1
#define PIN_MULT_B PIN_A2 
#define PIN_MULT_C PIN_A3
#define PIN_MULT_D PIN_A4 
#define PIN_LED_I      PIN_C1 
#define PIN_LED_DATA   PIN_D0
#define PIN_LED_CLOCK  PIN_A5
#define PIN_LED_ENABLE PIN_C2
#define PIN_TX PIN_D1

#define PIN_ADC_MOSI     PIN_B5
#define PIN_ADC_MISO     PIN_B6
#define PIN_ADC_SPICLOCK PIN_B7
#define PIN_ADC_SLAVE_SELECT PIN_C0


/* HW Related defines */
#define CCHAN_R 0
#define CCHAN_S 1
#define CCHAN_D 2
#define CCHAN_A 3
#define CCHAN_NONE 8

#define CCHAN_PWM 4
#define CCHAN_RES 5
#define CCHAN_FILT 6
#define CCHAN_SWITCH_RINGSYNC 9
#define CCHAN_FILTER_TYPE_CV 10
#define CCHAN_CHORD_CV 11
#define CCHAN_EXT_WAVEFORMS 8
#define CCHAN_SWITCH_FILTER 7
#define CCHAN_RINGSYNC 14 // to mod sel
#define CCHAN_SWITCH_WAVEFORM 13
#define CCHAN_WAVEFORM 12
#define CCHAN_CHORD_INV 15

/* LED Flags */
#define LED_TRI   (1<<0)
#define LED_SAW   (1<<1)
#define LED_PULSE (1<<2)
#define LED_NOISE (1<<3)
#define LED_HI    (1<<4)
#define LED_MID   (1<<5)
#define LED_LO    (1<<6)
#define LED_RING  (1<<7)
#define LED_SYNC  (1<<8)

/* Switch flags */
#define SWITCH_FILTER (1<<2)
#define SWITCH_WAVEFORM (1<<3)
#define SWITCH_RINGSYNC (1<<4)

/* Filter flags */
#define FILTER_LP     0
#define FILTER_BP     1 
#define FILTER_HP     2
#define FILTER_NOTCH  3
#define FILTER_NONE   -1

/* Constants for Waveform */
#define WAVEFORM_NONE 0x0
#define WAVEFORM_TRI 0x1
#define WAVEFORM_SAW 0x2
#define WAVEFORM_PULSE 0x4
#define WAVEFORM_NOISE 0x8

#define WAVEFORM_TRISAW WAVEFORM_TRI|WAVEFORM_SAW
#define WAVEFORM_TRIPULSE WAVEFORM_TRI|WAVEFORM_PULSE
#define WAVEFORM_SAWPULSE WAVEFORM_SAW|WAVEFORM_PULSE
#define WAVEFORM_NOWAVE 0x0  
  
/* OSC3 State */
#define STATE_NONE 0
#define STATE_RING 1
#define STATE_SYNC 2

/* SID Realted */
#define VOLUME 15
/* #define CHAN3_OFF (1<<7) */
#define CHAN3_OFF 0

/* Freq LUT */
#define VOLTS_FREQ_MIN 1304
#define VOLTS_FREQ_MAX 41617

#define VOLTS_FREQ_N 3276

typedef struct _Chord 
{
  byte n_freqs;
  byte denom;
  byte numerator[3];
} 
Chord;

#define N_EXT_WAVEFORMS 7 

uint8_t __ext_waveform_list[] = { WAVEFORM_TRI,
				WAVEFORM_SAW,
				WAVEFORM_PULSE,
				WAVEFORM_NOISE,
				WAVEFORM_TRISAW,
				WAVEFORM_TRIPULSE,
				WAVEFORM_SAWPULSE };


#define N_CHORDS 11

Chord _chords[N_CHORDS] =
  { 

    { 0, 1, { 1 }},  	  /* none */
    { 1, 4, { 5 }},  	  /* Power x2 (5th) */
    { 2, 4, { 5, 6 }}, 	  /* Major FIXME: ADD inverted */
    { 2, 10, { 12, 15 }}, /* Minor FIXME: ADD inverted */
    { 2, 6, {8, 9}},      /* Suspended */
    { 2, 16, {20, 25}},   /* Augmented */
    { 2, 20, { 24, 29 }}, /* Diminished */
    { 1, 1, { 2 }},  	  /* Octave x1 - 1 octave a part */
    { 2, 1, { 2, 3 }},    /* Octave x2 - 2 octave a part */
    { 2, 30, { 31, 33 }}, /* Detune 1 */
    { 2, 50, { 51, 53 }}  /* Detune 2 */
  };

typedef struct _SIDState 
{
  int      waveform;
  int      freq_chan_1;
  int      freq_chan_2;
  int      freq_chan_3;
  int      pulse_width;
  int      filter;
  int      resonance;
  int      filter_type;
  uint8_t  hp,bp,lp;
  uint8_t  chan_3_state;
  bool     gate_off;
  int      volume;
  int      chord_index;
  bool     ext_waveforms;
  uint8_t  filter_mask;
  uint8_t  chord_inv;

  uint8_t  ext_waveforms_idx;
  uint8_t  ext_waveforms_cnt;
  
} SIDstate;

SIDstate _sid;
int16_t _tune_offset = 0;

void cycle ();

void mcp3202_init()
{
  uu_pin_mode(PIN_ADC_MOSI, OUTPUT);
  uu_pin_mode(PIN_ADC_MISO, INPUT);
  uu_pin_mode(PIN_ADC_SPICLOCK, OUTPUT);
  uu_pin_mode(PIN_ADC_SLAVE_SELECT, OUTPUT);
}

int mcp3202_read(byte channel)
{
  int adcvalue = 0;
  byte commandbits = B11010000; //command bits - start, mode, chn, MSBF 

  //allow channel selection
  if (channel)
    commandbits = B11110000;

  uu_pin_digital_write (PIN_ADC_SLAVE_SELECT, LOW); //Select adc

  for (int i=7; i>=4; i--){
    uu_pin_digital_write(PIN_ADC_MOSI,commandbits&1<<i);
    //cycle clock
    uu_pin_digital_write(PIN_ADC_SPICLOCK, HIGH);
    uu_pin_digital_write(PIN_ADC_SPICLOCK, LOW);
  }

  uu_pin_digital_write(PIN_ADC_SPICLOCK, HIGH);    //ignores 1 null bit
  uu_pin_digital_write(PIN_ADC_SPICLOCK, LOW);
  
  //read bits from adc
  for (int i=11; i>=0; i--)
    {
      adcvalue += uu_pin_digital_read(PIN_ADC_MISO)<<i;
      //cycle clock
      uu_pin_digital_write(PIN_ADC_SPICLOCK, HIGH);
      uu_pin_digital_write(PIN_ADC_SPICLOCK, LOW);
    }

  uu_pin_digital_write(PIN_ADC_SLAVE_SELECT, HIGH); //turn off device

  return adcvalue;
}

void 
tuning_save()
{
  uu_interrupts_off();
  eeprom_write_word (1, _tune_offset);
  uu_interrupts_on();
}

void
settings_save()
{
  byte b;

  /* 1 byte for speed... */
  b = (_sid.waveform<<4) 	    /* bits 8-4 */
    |(_sid.filter_type<<2)  /* Filter 4-2  */
    |_sid.chan_3_state;	    /* State  2-1  */

  uu_interrupts_off();
  eeprom_write_byte (0, b);
  uu_interrupts_on();
}

void
settings_load()
{
  byte b;

  b = eeprom_read_byte (0);

  if (b == 0xff) // default erased val.. waveform only goes to 0x80
    {
      _tune_offset = 0;
      return;
    }

  _sid.waveform = (b & 0xf0) >> 4;
  _sid.filter_type = (b & 0x0f) >> 2;
  _sid.chan_3_state = (b & 0x3);

  _tune_offset = eeprom_read_word (1);

  switch (_sid.waveform) 
    {
    case WAVEFORM_NONE:
    case WAVEFORM_TRI:
    case WAVEFORM_SAW: 
    case WAVEFORM_PULSE: 
    case WAVEFORM_NOISE: 
      break;
    default:
      /* Something gone wrong - flash corrupt...? defaults */
      _sid.waveform = WAVEFORM_PULSE;
      break;
    }
}

void leds_set_mask(uint32_t mask)
{
  byte shift_mask = mask & 0xFF;

  if (mask & LED_SYNC)
    uu_pin_digital_write(PIN_LED_I, HIGH);
  else
    uu_pin_digital_write(PIN_LED_I, LOW);

  uu_pin_digital_write(PIN_LED_ENABLE, LOW);
  uu_pin_shift_out(PIN_LED_DATA, PIN_LED_CLOCK, MSBFIRST, shift_mask);  
  uu_pin_digital_write(PIN_LED_ENABLE, HIGH);
}

int analog_read()
{
  uint8_t low, high;
  
  /* switch to ADMUX for (1<<6) - AVcc with external capacitor on AREF pin  */
  ADMUX |= 0; // AREF, Internal Vref turned off  & chan 0 
  
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
  /* Start conversion */
  ADCSRA |= (1<<ADSC);
  /* Wait.. */
  while (uu_bit_is_set(ADCSRA, ADSC));
  /* Collect */
  low  = ADCL;
  high = ADCH;

  return (high << 8) | low;
}

void select_chan(int chan)
{
  uu_pin_digital_write (PIN_MULT_A, (chan & 0x01));
  uu_pin_digital_write (PIN_MULT_B, ((chan >> 1) & 0x01));
  uu_pin_digital_write (PIN_MULT_C, ((chan >> 2) & 0x01));
  uu_pin_digital_write (PIN_MULT_D, ((chan >> 3) & 0x01));

  _delay_us(500);
}

int read_chan_analog(int chan)
{
  select_chan(chan);

  return analog_read(); 
}

bool read_chan_digital(int chan)
{
  select_chan(chan);

  return (analog_read() > 512);
}

byte switches_read_mask()
{
  byte button_mask = 0;

  if (read_chan_digital(CCHAN_SWITCH_FILTER))
    button_mask |= SWITCH_FILTER;
    
  if (read_chan_digital(CCHAN_SWITCH_WAVEFORM))
    button_mask |= SWITCH_WAVEFORM;

  if (read_chan_digital(CCHAN_SWITCH_RINGSYNC))
    button_mask |= SWITCH_RINGSYNC;

  return button_mask;
}

void SID_poke (uint8_t port, uint8_t data) 
{
  PORTB = (port & 0x0F) | 0x10;        // Lower bit, keep CS deactive
  PORTD = ((port | 0x20) & 0xF0) >> 2; // Upper bit, reset high
  PORTD |= 0x80;                       // Clock in Address
  _delay_us(10);
  PORTD &= ~(0x80);

  PORTB = (data & 0x0F) | 0x10;  // Lower bit, keep CS deactive
  PORTD = (data & 0xF0) >> 2;    // Upper bit
  PORTB &= ~(0x10);   // Activate /CS
  _delay_us(10);
  PORTB |= 0x10;      // Deactivate /CS
}  

void soundcheck()
{
  int waveforms[] = { WAVEFORM_PULSE, WAVEFORM_SAW, WAVEFORM_TRI, WAVEFORM_NOISE };
  int filters[] = { FILTER_LP, FILTER_BP, FILTER_HP, FILTER_NOTCH };
  int i,j, k;

  /* Test Waveforms */
  for (k=0;k<1;k++)
    {
      unsigned int n,f,d,e;

      SID_poke(4,(WAVEFORM_PULSE<<4)|(0<<2)|(0<<1)|1);

      /* Set pulse width middle */
      SID_poke(2,uu_bit_low_byte(2056));
      SID_poke(3,uu_bit_high_byte(206));

      /* low pass */
      SID_poke(24, (CHAN3_OFF|(0<<6)|(0<<5)|(1<<4)|VOLUME));

      /* resonance */
      SID_poke(23,(8<<4)|9 /* 9 may be safer */);
      f = 1024 << 1;
      SID_poke(21,uu_bit_low_byte(f));     // Set filter value - 11bits
      SID_poke(22, uu_bit_high_byte(f << 5));

      for (j=0;j<4;j++)
	{
	  switch (waveforms[j])
	    {
	    case WAVEFORM_TRI:
	      leds_set_mask(LED_TRI);
	      break;
	    case WAVEFORM_SAW:
	      leds_set_mask(LED_SAW);
	      break;
	    case WAVEFORM_NOISE:
	      leds_set_mask(LED_NOISE);
	      break;
	    case WAVEFORM_PULSE:
	      leds_set_mask(LED_PULSE);
	      break;
	    }

	  for(i=0;i<12;i++) 
	    {
	      f = 100; // pgm_read_word(&volts_to_freq[(1024/12)*i]);

	      SID_poke(4,(waveforms[j]<<4)|(0<<2)|(0<<1)|1);
	      SID_poke(0,f);   // Send frequency to chanel
	      SID_poke(1,f>>8);
	      for(d=0;d<65000;d++)   
		_delay_us(10);
	    }
	}
    }

  /* SWEEP NOISE */
  SID_poke(4,(WAVEFORM_NOISE<<4)|(0<<2)|(0<<1)|1);
  SID_poke(0, 0x9999);
  SID_poke(1, 0x9999>>8);

  for (k=0;k<1;k++)
    for (j=0;j<4;j++)
      {
	switch (filters[j])
	  {
	  case FILTER_NOTCH:
	    SID_poke(24, (CHAN3_OFF|(1<<6)|(0<<5)|(1<<4)|VOLUME));
	    leds_set_mask(LED_LO|LED_HI);
	    break;
	  case FILTER_HP:
	    SID_poke(24, (CHAN3_OFF|(1<<6)|(0<<5)|(0<<4)|VOLUME));
	    leds_set_mask(LED_HI);
	    break;
	  case FILTER_LP:
	    SID_poke(24, (CHAN3_OFF|(0<<6)|(0<<5)|(1<<4)|VOLUME));
	    leds_set_mask(LED_LO);
	    break;
	  case FILTER_BP:
	    SID_poke(24, (CHAN3_OFF|(0<<6)|(1<<5)|(0<<4)|VOLUME));
	    leds_set_mask(LED_MID);
	    break;
	  }
	
	for (i=0;i<1024;i++)
	  {
	    int f = i << 1;
	    SID_poke(21,uu_bit_low_byte(f));     // Set filter value - 11bits
	    SID_poke(22, uu_bit_high_byte(f << 5));
	    _delay_us(10000);
	  }
      }
}

void setup () 
{
  int c, i = 0;

  PORTD = 0;
  PORTB = 0x10;   // PB4 is high to disable Chip Select
  DDRD  = 0xFE;   // Keep RxD as an input, everything else output
  DDRB  = 0xFF;

  uu_init(0);

  uu_pin_mode(PIN_TX, OUTPUT);

  uu_usart_init();
  
  uu_pin_mode(PIN_LED_I, OUTPUT);

  uu_pin_mode(PIN_MULT_IN, INPUT);

  uu_pin_mode(PIN_MULT_A, OUTPUT);
  uu_pin_mode(PIN_MULT_B, OUTPUT);
  uu_pin_mode(PIN_MULT_C, OUTPUT);
  uu_pin_mode(PIN_MULT_D, OUTPUT);

  uu_pin_mode(PIN_LED_DATA, OUTPUT);
  uu_pin_mode(PIN_LED_CLOCK, OUTPUT);
  uu_pin_mode(PIN_LED_ENABLE, OUTPUT);
  
  /* Initialise Timer 0 OC0A to 1MHz on Pin 6. */
  TIMSK2 = 0;
  TCNT2  = 0;     /* Reset timer */
  OCR2A  = 0;     /* Invert clock after this many clock cycles + 1 */
  TCCR2A = (1<<COM0B0) | (2<<WGM00);
  TCCR2B = (2<<CS00); /* Prescaler */

  _sid.waveform = WAVEFORM_NONE;
  _sid.freq_chan_1 = -1;
   _sid.freq_chan_2 = 0;
  _sid.freq_chan_3 = 0;
  _sid.pulse_width = 0;
  _sid.filter = -1;
  _sid.resonance = -1;
  _sid.filter_type = FILTER_LP;
  _sid.hp = _sid.bp = 0; _sid.lp = 1;
  _sid.chan_3_state = STATE_NONE;
  _sid.gate_off = FALSE;
  _sid.volume   = 15;
  _sid.chord_index = 0;
  
  _sid.ext_waveforms = read_chan_digital(CCHAN_EXT_WAVEFORMS);
  _sid.ext_waveforms_idx = 0;
  _sid.ext_waveforms_cnt = 0;
  
  settings_load();

  /* Give swinsid a chance to initialise... seems racey, not sure why */
  for (i=0;i<2000;i++)
    _delay_us(1000);

  /* Clear all regs */
  for(c=0; c<25;c++)
    SID_poke(c,0);

  /* Write some default values to the SID */
  SID_poke(24,15);    /* Turn up the volume */
  SID_poke(5,0);      /* Fast Attack, Decay */
  SID_poke(5+7,0);      /* Fast Attack, Decay */
  SID_poke(5+14,0);      /* Fast Attack, Decay */
  SID_poke(6,0xF0);      /* Full volume on sustain, quick release */
  SID_poke(6+7,0xF0);    /* Full volume on sustain, quick release */
  SID_poke(6+14,0xF0);   /* Full volume on sustain, quick release */
  SID_poke(4,0x21);   /* Enable gate, sawtooth waveform. */

  leds_set_mask(0);

  mcp3202_init();
    
  /* set up Timer 1 for processing input & output at 50hz (like real SID to avoid excessive noise) */
  TCCR1A = 0;                                     /* normal operation */
  TCCR1B = _BV(WGM12) | _BV(CS10) | _BV (CS12);   /* CTC, scale to clock / 1024 */
  OCR1A =  319;                                   /* compare A register value (319 * 1/clock speed * 1024) = 50hz / 20ms */
  TIMSK1 = _BV (OCIE1A);                          /* interrupt on Compare A Match */
}

void
usart_dump_controls()
{
  int i;

  /* clear screen */
  uu_usart_putc(27);
  uu_usart_puts("[2J");
  uu_usart_putc(27);
  uu_usart_puts("[H"); 

  i = read_chan_analog(CCHAN_FILT);

  uu_usart_puts("FILTER: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = read_chan_analog(CCHAN_RES);

  uu_usart_puts("Res: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = read_chan_analog(CCHAN_PWM);

  uu_usart_puts("PWM: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = switches_read_mask();

  uu_usart_puts("Button mask: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');
  
  i = read_chan_analog(CCHAN_CHORD_CV);

  i = ((i-50) * N_CHORDS) / 1000;
  
  uu_usart_puts("CHORD: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = read_chan_digital(CCHAN_EXT_WAVEFORMS);
  uu_usart_puts("EXT WAVEFORMS: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  
  i =  mcp3202_read(1);

  uu_usart_puts("VCO: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i =  mcp3202_read(0);

  uu_usart_puts("OSC 2: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');


  uu_usart_puts("EXT waveforms: ");
  uu_usart_puti(_sid.ext_waveforms);
  uu_usart_putc('\n');

  uu_usart_puts("Filter type: ");
  uu_usart_puti(_sid.filter_type);
  uu_usart_putc('\n');

  i = read_chan_analog(CCHAN_FILTER_TYPE_CV);

  uu_usart_puts("FILTER TYPE CV: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = read_chan_analog(CCHAN_WAVEFORM);

  uu_usart_puts("WAVEFORM CV: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  i = read_chan_analog(CCHAN_RINGSYNC);

  uu_usart_puts("RING SYNC CV: ");
  uu_usart_puti(i);
  uu_usart_putc('\n');

  
}

void cycle () 
{
  static bool last_gate = 0;
  static byte switch_ignore_mask = 0;
  static bool want_tune = FALSE;
  static bool first_run = TRUE;
  static uint8_t dump = 0;
  static int previous_pitch_adc_code = 0;
  
  unsigned int f;
  uint8_t      c;
  int          i, waveform, state, chord_inv;
  bool         sync_waveform = FALSE, sync_filter = FALSE, reset_osc1 = FALSE, chord_change = FALSE;


  int          led_mask    = 0;
  byte         switch_mask = 0;

#define CHECK_SWITCH(key) \
        (switch_mask & (key) && !(switch_ignore_mask & (key)))

   
  switch_mask = switches_read_mask();

  if (++dump > 30)
    {				/* once per second */
      dump = 0;
      //usart_dump_controls();
    }
  
  state = _sid.chan_3_state;

  /* Chord mode */
  i = read_chan_analog(CCHAN_CHORD_CV);
  
  /* scale - assume ADC between 25 & 1000 */  
  i = ((i-25) * N_CHORDS) / 1000;
  
  if (i<0) i = 0;
  if (i >= N_CHORDS) i = N_CHORDS-1;
  
  if (i != _sid.chord_index) 		
    {
      if (_sid.chord_index > 0)
	{
	  /* turn off other oscs...FIXME: check for ring/sync  */
	  for (c = 1;  c < 3; c++)
	    {
	      SID_poke(4 + (c * 7), (_sid.waveform<<4)|0);
	      SID_poke(6 + (c * 7), 0);   // No volume of sustain.. gate is not enough
	    }

	  /* full volume back for osc1 */
	  SID_poke(6, 0xF0);   // No volume of sustain.. gate is not enough
	  
	  sync_waveform = TRUE;
	  chord_change = TRUE;
	}
      
      _sid.chord_index = i;
      
      /* turn up volume (to halfish) */
      for (c = 0;  c < _chords[_sid.chord_index].n_freqs; c++)
	SID_poke(6 + ((c+1) * 7) ,0xF0 /*_sid.volume << 4 was F */);
      
      sync_waveform = TRUE;
      chord_change = TRUE;
    }

  /* hacky.. reset here for extended waveform cycling */
  if (_sid.ext_waveforms)
      _sid.waveform = __ext_waveform_list[_sid.ext_waveforms_idx];
  
  /* 
   *  - Waveform & Filter switches held should turn off the waveform... 
   *  - Waveform display is then blank, no LED lit.
   *  - Can be turned on again by pressing waveform butten
   */
  if ((CHECK_SWITCH(SWITCH_WAVEFORM) && CHECK_SWITCH(SWITCH_RINGSYNC))
      || (CHECK_SWITCH(SWITCH_WAVEFORM) && (switch_ignore_mask & SWITCH_RINGSYNC))
      || (CHECK_SWITCH(SWITCH_RINGSYNC) && (switch_ignore_mask & SWITCH_WAVEFORM)))

    {
      switch_ignore_mask |= SWITCH_WAVEFORM;

      if (_sid.gate_off != TRUE)
	{
	  SID_poke(6,0x00);   // No volume of sustain.. gate is not enough
	  _sid.gate_off = TRUE;
	  sync_waveform = TRUE; // So gate is toggled.
	}
    }
   
  if (CHECK_SWITCH(SWITCH_FILTER))
    {
      _sid.filter_type++;
      
      if (_sid.filter_type > FILTER_NOTCH)
	_sid.filter_type = FILTER_LP;
      
      sync_filter = TRUE;

      switch_ignore_mask |= SWITCH_FILTER;
    }


  i = read_chan_analog(CCHAN_FILTER_TYPE_CV);

  if (i > 275) 
    {
      int filter_t = 0;
      
      if (i < 450)
	{
	  filter_t = FILTER_LP;
	}
      else if (i < 650)
	{
	  filter_t = FILTER_BP;
	}
      else if (i < 775)
	{
	  filter_t = FILTER_HP;
	}
      else 
	{
	  filter_t = FILTER_NOTCH;
	}

      if (filter_t != _sid.filter_type)
	{
	  sync_filter = TRUE;
	  _sid.filter_type = filter_t;
	}
    }

  switch (_sid.filter_type)
    {
    case FILTER_NOTCH:
      led_mask |= (LED_HI|LED_LO);
      _sid.filter_mask = (CHAN3_OFF|(1<<6)|(0<<5)|(1<<4));
      break;
    case FILTER_HP:
      led_mask |= LED_HI;
      _sid.filter_mask = (CHAN3_OFF|(1<<6)|(0<<5)|(0<<4));
      break;
    case FILTER_LP:
      led_mask |= LED_LO;
      _sid.filter_mask = (CHAN3_OFF|(0<<6)|(0<<5)|(1<<4));
      break;
    case FILTER_BP:
      led_mask |= LED_MID;
      _sid.filter_mask = (CHAN3_OFF|(0<<6)|(1<<5)|(0<<4));
      break;
    }

  if (first_run)
    {
      first_run = FALSE;
      sync_filter = TRUE;
      sync_waveform = TRUE;
    }

  /* Initial waveform */
  if (_sid.waveform == WAVEFORM_NONE)
    {
      _sid.waveform = waveform = WAVEFORM_PULSE;
      sync_waveform = TRUE;
    }
  else 
    waveform = _sid.waveform;

  if (CHECK_SWITCH(SWITCH_WAVEFORM))
    {
      if (_sid.gate_off)
	{
	  SID_poke(6,0xF0);   /* Full volume on sustain back on */
	  _sid.gate_off = FALSE;
	  sync_waveform = TRUE;	  
	}
      else
	{
	  if (_sid.ext_waveforms)
	    {
	      _sid.ext_waveforms_idx++;
	      if (_sid.ext_waveforms_idx > N_EXT_WAVEFORMS)
		_sid.ext_waveforms_idx = 0;

	      waveform = __ext_waveform_list[_sid.ext_waveforms_idx];
	    }
	  else
	    {
	      waveform = waveform*2;

	      if (waveform>WAVEFORM_NOISE)
		waveform = WAVEFORM_TRI;
	      
	    }
	  reset_osc1 = TRUE; 	/* More safety on */
	}


      switch_ignore_mask |= SWITCH_WAVEFORM;
    }

  
  /* extra delay here a below can pickup switch reading :/ */
  i = read_chan_analog(CCHAN_WAVEFORM);

  
  if (i > 275) 			/* FIXME: 50 on original guts ??? */
    {
	  if (i < 450)
	    {
	      waveform = WAVEFORM_NOISE;
	    }
	  else if (i < 650)
	    {
	      waveform = WAVEFORM_TRI;
	    }
	  else if (i < 775)
	    {
	      waveform = WAVEFORM_SAW;
	    }
	  else 
	    {
	      waveform = WAVEFORM_PULSE;
	    }
	  
      if (waveform != _sid.waveform) 
	reset_osc1 = TRUE;
    }

  /* Waveform changed make sure we update */
  if (waveform != _sid.waveform) 
    {
      _sid.waveform = waveform;
      sync_waveform = TRUE;
    }

  /* update LED */
  if (_sid.gate_off == FALSE)
    {
      led_mask |= _sid.waveform;
    }
    
  i =  mcp3202_read(1);
  
  /* Note 6 over 4 as seems to be much more noise on ADC */
#define DO_HYST(c,p) if ( ((c) > (p) + 6) || ((c) < (p) - 6) ) { (p) = (c); } else { (c) = (p); }

  DO_HYST(i, previous_pitch_adc_code);
  
  if (i > VOLTS_FREQ_N)
    i = VOLTS_FREQ_N;

  chord_inv = (read_chan_analog(CCHAN_CHORD_INV) >> 7) - 2; /* 2 bit value */

  if (chord_inv < 0)
    chord_inv = 0;

  if (chord_inv != _sid.chord_inv)
    {
      _sid.chord_inv = chord_inv;
      chord_change = TRUE;
    }
   
  if (i != _sid.freq_chan_1 || chord_change == TRUE) 
    {

      _sid.freq_chan_1 = i;
      f = pgm_read_word(&volts_to_fnum[i]);
      
      SID_poke(0,f);   /* Send frequency to chanel */
      SID_poke(1,f>>8);
      
      for (c = 0;  c < _chords[_sid.chord_index].n_freqs; c++)
	{
	  unsigned int root_freq, chord_freq;

	  /* FIXME - check correct by using lookup.h with regular C to check calculations */
	  /* seems inverts are wrong */

	  root_freq = pgm_read_word(&volts_to_freq[i]);
	  chord_freq = (root_freq * _chords[_sid.chord_index].numerator[c]) / _chords[_sid.chord_index].denom;

	  if (_sid.chord_inv > c)
	    chord_freq = chord_freq >> 1;
	  
	  f = (chord_freq * 167772UL) / 10000UL; // * 16777216UL) / 1000000UL; max resolution
	    	  
	  SID_poke(0 + ((c+1) * 7),f);
	  SID_poke(1 + ((c+1) * 7),f>>8);

	}
    }
  
  /*  Pulse width */
  i = (read_chan_analog(CCHAN_PWM) << 2); /* 12 bit value */

  /* 40 * 4 - cuts off so cant be heard */
  if (i<160) i = 160;
  if (i>4095) i = 4095;

  if (i != _sid.pulse_width || chord_change == TRUE)
    {
      SID_poke(2,uu_bit_low_byte(i));    /* Set pulse width low */
      SID_poke(3,uu_bit_high_byte(i));   /* Set pulse width high */
      _sid.pulse_width = i;

      for (c = 0;  c < _chords[_sid.chord_index].n_freqs; c++)
	{
	    SID_poke(2 + ((c+1) * 7),uu_bit_low_byte(i));
	    SID_poke(3 + ((c+1) * 7),uu_bit_high_byte(i));
	}
    }

  /* Filter */
  i = read_chan_analog(CCHAN_FILT) << 1;

  if (i<0) i = 0;

  if (i != _sid.filter)
    {
      SID_poke(21,uu_bit_low_byte(i) & 7);     // Set filter value - 11bits
      SID_poke(22, uu_bit_high_byte(i << 5));

      _sid.filter = i;
    }

  /* Resonance 4bit */
  i = (read_chan_analog(CCHAN_RES) >> 6);
  if (i<0) i = 0;
  if (i>15) i = 15;

  if (i != _sid.resonance) 
    {
      if (_sid.chord_index)
	c = 0x0f; 		/* turn on extra chans */
      else
	c= 9;
      SID_poke(23,(i<<4)|/*9*/0x0f);  /* Set resonance and all channels on */
      _sid.resonance = i;
    }

  if (sync_filter)
    {
      SID_poke(24, _sid.filter_mask|_sid.volume);
    }

  /* Modulation Osc */
  if (CHECK_SWITCH(SWITCH_RINGSYNC))
    {
      if (!_sid.gate_off)
	{
	  state++;

	  if (state > STATE_SYNC)
	    {
	      sync_waveform = TRUE;
	      state = STATE_NONE;
	    }
	}
      switch_ignore_mask |= SWITCH_RINGSYNC;
    }

  i = read_chan_analog(CCHAN_RINGSYNC);

  if (i > 300)
    {
      if (i < 750)
	{
	  state = STATE_SYNC;
	}
      else
	{
	  state = STATE_RING;
	}

      if (state != _sid.chan_3_state)
	reset_osc1 = TRUE; 		/* Safety on */
    }
  else
    {
      if (i > 275 && state != STATE_NONE)
	{
	  state = STATE_NONE;
	  sync_waveform = TRUE;
	  reset_osc1 = TRUE; 
	}
    }
  
  if (state == STATE_SYNC && !_sid.gate_off)
    led_mask |= LED_SYNC;

  if (state == STATE_RING && !_sid.gate_off)
    {
      led_mask |= LED_RING;

      led_mask &= ~(LED_PULSE|LED_SAW|LED_NOISE);
      led_mask |= LED_TRI;
    }

  if (state != STATE_NONE)
    {
      i =  mcp3202_read(0);

      if (i > VOLTS_FREQ_N)
	i = VOLTS_FREQ_N;

      /* FIXME: This essencially overides any chord setting*/
      if (i != _sid.freq_chan_3)
	{
	  _sid.freq_chan_3 = i;

	  f = pgm_read_word(&volts_to_fnum[i]);
	  /* freq of oscillator 3 */
	  SID_poke(14,f); 
	  SID_poke(15,f>>8);
	}
    }

  if (reset_osc1) /* hack to avoid odd lockup with osc1 turning off */
    {
      SID_poke(4,(_sid.waveform<<4)|0);

      for (c = 0;  c < _chords[_sid.chord_index].n_freqs; c++)
	SID_poke(4 + ((c+1) * 7), (_sid.waveform<<4)|0);
    }

  if (sync_waveform || _sid.chan_3_state != state)
    {
      int ring, sync, gate;

      ring = (state == STATE_RING) ? 1 : 0;
      sync = (state == STATE_SYNC) ? 1 : 0;
      gate = (_sid.gate_off == TRUE) ? 0 : 1;

      if (state == STATE_NONE && state != _sid.chan_3_state && _sid.chord_index == 0)
	  /* Make sure oscillator goes off - for swinsid */
	SID_poke(18,(_sid.waveform<<4)|0);

      /* for chords .. will be potentially overiide */
      for (c = 0;  c < _chords[_sid.chord_index].n_freqs; c++)
	SID_poke(4 + ((c+1) * 7), (_sid.waveform<<4)|gate);
      
      if (ring) /* must be triangle for ring sinc */
	{
	  SID_poke(4,(WAVEFORM_TRI<<4)|(ring<<2)|(sync<<1)|gate);
	  SID_poke(18,(_sid.waveform<<4)|gate);
	}
      else
	{
	  SID_poke(4,(_sid.waveform<<4)|(ring<<2)|(sync<<1)|gate);
	}
    }

  _sid.chan_3_state = state;

  if (switch_mask == 0) 	// Nothing pressed so we clear the mask
    switch_ignore_mask = 0; 	// Maybe a time out here to debounce better?
  else
    settings_save(); 		/* something pressed so save settings */

  leds_set_mask(led_mask);

}

ISR(TIMER1_COMPA_vect)
{
  cycle();
}

int main(void)
{
  setup();
  while (TRUE);
}
