/*  Copyright (c) 2017 by Sangbong Lee <sangbong ~AT~ me.com>

  This is Arduino TLC5940 MULTIPLEXING Library.

  This TLC5940 Multiplexing Library is free software: you can redistribute it
  and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  The Arduino TLC5940 Library is distributed in the hope that it will be
  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

   Original
   Alex Leone - acleone ~AT~ u.washington.edu
   08-27-08

  You should have received a copy of the GNU General Public License
  along with The Arduino TLC5940 Library.  If not, see
  <http://www.gnu.org/licenses/>. */

#include <SPI.h>
#include <avr/pgmspace.h>

#define EOP 0

const uint16_t exp_table [] PROGMEM=
{ 
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  //16  
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  //32  
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   42,  //48 
   43,   44,   45,   46,   47,   48,   49,   50,   51,   53,   54,   55,   56,   57,   59,   60,  //64  
   61,   63,   64,   65,   67,   68,   70,   72,   73,   75,   76,   78,   80,   82,   83,   85,  //82  
   87,   89,   91,   93,   95,   97,   99,  102,  104,  106,  109,  111,  114,  116,  119,  121,  //96
  124,  127,  130,  132,  135,  138,  141,  145,  148,  151,  154,  158,  161,  165,  169,  172,  //112
  176,  180,  184,  188,  192,  197,  201,  206,  210,  215,  220,  225,  230,  235,  240,  245,  //128
  251,  256,  262,  268,  274,  280,  286,  292,  299,  305,  312,  319,  326,  334,  341,  349,  //144
  356,  364,  372,  381,  389,  398,  407,  416,  425,  434,  444,  454,  464,  474,  485,  496,  //160
  507,  518,  529,  541,  553,  566,  578,  591,  604,  618,  631,  645,  660,  674,  689,  705,  //176
  720,  736,  753,  770,  787,  804,  822,  840,  859,  878,  898,  918,  938,  959,  980, 1002,  //192
 1024, 1047, 1070, 1094, 1119, 1143, 1169, 1195, 1221, 1249, 1276, 1305, 1334, 1364, 1394, 1425,  //208
 1457, 1489, 1522, 1556, 1591, 1626, 1662, 1699, 1737, 1775, 1815, 1855, 1897, 1939, 1982, 2026,  //224
 2071, 2117, 2164, 2212, 2262, 2312, 2363, 2416, 2470, 2524, 2581, 2638, 2697, 2757, 2818, 2881,  //240
 2945, 3010, 3077, 3146, 3216, 3287, 3360, 3435, 3511, 3590, 3669, 3751, 3834, 3920, 4007, 4095   //256
};

volatile uint8_t needXlat_pulse = 0;
volatile uint8_t needGSdata_update = 0;
volatile uint8_t currentLayer = 0;

uint8_t NUM_LAYER = 16;

uint16_t GSbyteBlock = 1024;

volatile byte val; // Data received from the serial port
volatile uint16_t pos;
volatile byte * ptr;

byte brightnessValue[1024];

SPISettings settingsA(8000000, MSBFIRST, SPI_MODE0);

void setup() {

  //*********************UART initialisation*********************//
  UCSR0A |= (1 << U2X0);                            // Double up UART
  UCSR0B |= (1 << RXEN0)  | (1 << TXEN0) | (1 << RXCIE0); // UART RX, TX und RX Interrupt enable
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00)             ; // Asynchrous 8N1
  UBRR0H = 0;
  UBRR0L = 0;                                       //Baud Rate 2 MBit
  //*********************UART initialisation*********************//

  //*********************GSCLK PIN SETUP*********************//
  DDRD |= (1 << 3); // DIGITAL PIN 3 (PD3) OUTPUT
  DDRD &= ~(1 << 5); //DIGITAL PIN 5 T1 PIN (PD5) INPUT
  //*********************GSCLK PIN SETUP*********************//

  //*********************SCLK, SIN, BLANK, XLAT PIN SETUP*********************//
  DDRB = 0b00101110; //SCLK PIN, SIN(MOSI) PIN, BLANK(OC1B) PIN, XLAT(OC1A) PIN AS OUTPUT
  //PB5 ,PB3 ,PB2, PB1 AS OUTPUT
  PORTB = 0b00000100; //BLANK HIGH, SCLK, SIN, XLAT LOW
  //PB2 HIGH
  //*********************SCLK, SIN, BLANK, XLAT PIN SETUP*********************//

  //*********************Layer select pin*********************//
  DDRC = 0xFF; //PC7-0 as output
  PORTC = 0x10; //layer select off (0b00010000)PC4 HIGH
  //*********************Layer select pin*********************//

  //*********************SPI PIN SETUP*********************//
  SPI.beginTransaction(settingsA);
  SPI.begin();
  //*********************SPI PIN SETUP*********************//

  //*********************GSCLK Timer SETUP (TIMER2)*********************//
  TCCR2A = (1 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22 ); //CTC
  TCCR2A |= (0 << COM2B1) | (1 << COM2B0); //toggle on compare match OC2B pin
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); //No prescaler
  OCR2A = 0; //compare value
  //*********************GSCLK Timer SETUP (TIMER2)*********************//

  //*********************BLANK,XLAT Timer SETUP (TIMER1)*********************//
  TCCR1A  = (1 << WGM11) | (0 << WGM10); // Fast PWM with ICR1 as top
  TCCR1B  = (1 << WGM13) | (1 << WGM12);
  TCCR1B |= (1 << CS12)  | (1 << CS11) | (1 << CS10); // Feed external clock from T1 pin (0c2b)
  TIMSK1 |= (1 << TOIE1);
  ICR1 = 4150; //For every 512us, ISR starts
  //*********************BLANK,XLAT Timer SETUP (TIMER1)*********************//

  //*********************GS value initialisation*********************//
  brightness_Init();
  //*********************GS value initialisation*********************//

  //*********************XLAT PULSE*********************//
  PORTB |= (1 << 1); //XLAT pulse, HIGH to LOW
  PORTB &= (0 << 1); //get ready for a 2nd GS DATA input cycle
  //*********************XLAT PULSE*********************//

  //*********************First GSdata input cycle start, Initial value*********************//
  for (int i = 0; i < GSbyteBlock; i++) {
    transferData(brightnessValue[i]);
  }
  //*********************First GSdata input cycle start, Initial value*********************//
  //*********************Enable global interrupt*********************//
  sei();
  //*********************Enable global interrupt*********************//
}

ISR (TIMER1_OVF_vect)
{
  //********* GSCLK DISABLE*********//
  PORTD &= (0 << 3); //SET GSCLK PIN LOW
  TCCR2A = 0; //DISABLE TIMER2
  TCCR2B = 0; //DISABLE TIMER2
  //********* GSCLK DISABLE*********//

  //********* BLANK PULSE*********//
  PORTB |= (1 << 2);//BLANK HIGH

  if (needXlat_pulse == 1) { //if need xlat pulse
    //********* LATCH PULSE*********//
    PORTB |= (1 << 1); //XLAT HIGH
    PORTB &= (0 << 1); //XLAT LOW
    //********* LATCH PULSE*********//
    needXlat_pulse = 0; //no need to xlat pulse
  }

  layerChange(); //layerChange
  needGSdata_update = 1;

  PORTB &= (0 << 2); //BLANK LOW
  //********* BLANK PULSE*********//

  PORTB |= (1 << 5); //SPI CLOCK pin to give extra count (193th pulse)
  PORTB &= (0 << 5); //SPI PULSE

  //********* GSCLK ENABLE*********//
  TCCR2A = (1 << WGM21) | (0 << WGM20);
  TCCR2B = (0 << WGM22 ); //CTC
  TCCR2A |= (0 << COM2B1) | (1 << COM2B0); //toggle on compare match OC2B pin
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20); //No prescaler
  //********* GSCLK ENABLE*********//

  TCNT1 = 0; //RESTART TIMER COUNTER1
}

ISR (USART_RX_vect) {
  val = UDR0;

  if (val == EOP) {
    UDR0 = val;
    pos = 0;
    ptr = brightnessValue;
    return;
  }

  if (pos == GSbyteBlock) {
    return;
  }

  else {
    *ptr = val;
    ptr++;
    pos++;
  }
}

void loop() {

  if (needGSdata_update == 1) //if layer is changed
  {
    shift_out_data(currentLayer);
    needGSdata_update = 0; //the layer needs to be changed
    needXlat_pulse = 1; //need Xlat pulse
  }
}

//=======================================================================================================================//
//***********************************************************************************************************************//
////////////////////////////////////////////////////LOW LEVEL FUNCTIONS////////////////////////////////////////////////////
//***********************************************************************************************************************//
//=======================================================================================================================//
void shift_out_data(byte layer)
{ 
    
    unsigned int index_offset = (layer) << 6; 
  
    //Shift Out tlc4 Data
    for(byte i = 24; i<32; i++)
    {
      unsigned int index = index_offset + (111-2*(i));
      
      uint8_t * GSVal = brightnessValue + index;
      
      unsigned int t1 = pgm_read_word_near(exp_table + *--GSVal);
      unsigned int t2 = pgm_read_word_near(exp_table + *GSVal);

      byte d1 = (t2 >> 4) | 0x00;
      byte d2 = ((t2 << 4) & 0xF0) | (t1 >> 8);
      byte d3 = (t1 & 0xFF);

      transferData(d1);
      transferData(d2);
      transferData(d3);      
    }
    
    //Shift Out tlc3 Data
    for(byte i = 16; i<24; i++)
    {
      unsigned int index = index_offset + (79-2*(i));
      
      uint8_t * GSVal = brightnessValue + index;
      
      unsigned int t1 = pgm_read_word_near(exp_table + *--GSVal);
      unsigned int t2 = pgm_read_word_near(exp_table + *GSVal);
  
      byte d1 = (t2 >> 4) | 0x00;
      byte d2 = ((t2 << 4) & 0xF0) | (t1 >> 8);
      byte d3 = (t1 & 0xFF);
      
      transferData(d1);
      transferData(d2); 
      transferData(d3);      
    }
           
    //Shift Out tlc2 Data
    for(byte i = 8; i<16; i++)
    {
      unsigned int index = index_offset + (47-2*(i));

      uint8_t * GSVal = brightnessValue + index;
      
      unsigned int t1 = pgm_read_word_near(exp_table + *--GSVal);
      unsigned int t2 = pgm_read_word_near(exp_table + *GSVal);
     
      byte d1 = (t2 >> 4) | 0x00;
      byte d2 = ((t2 << 4) & 0xF0) | (t1 >> 8);
      byte d3 = (t1 & 0xFF);
      
      transferData(d1);
      transferData(d2);   
      transferData(d3);      
    }
    
    //Shift Out tlc1 Data
    for(byte i = 0; i<8; i++)
    {
      unsigned int index = index_offset + (15 - 2*(i));
      
      uint8_t * GSVal = brightnessValue + index;
      
      unsigned int t1 = pgm_read_word_near(exp_table + *--GSVal);
      unsigned int t2 = pgm_read_word_near(exp_table + *GSVal);
      //unsigned int t1 = pgm_read_word_near(exp_table + brightnessValue[index-1]);
      //unsigned int t2 = pgm_read_word_near(exp_table + brightnessValue[index]);
     
      byte d1 = (t2 >> 4) | 0x00;
      byte d2 = ((t2 << 4) & 0xF0) | (t1 >> 8);
      byte d3 = (t1 & 0xFF);
      
      transferData(d1);
      transferData(d2); 
      transferData(d3); 
    }
}

void transferData(byte data) { //8bit transfer

  SPDR = data;
  while (!(SPSR & (1 << SPIF))); // wait until the transmission is completed

}

//***************layer select***************//
void layerChange() {

  PORTC |= 0x10; //layer select off (0b00010000)PC4 HIGH

  PORTC = (( PORTC & 0xF0 ) | ( 0x0F & ( currentLayer))) & 0xEF; //current layer ON,port HIGH

  currentLayer++; //prepare next layer

  if (currentLayer == NUM_LAYER) {
    currentLayer = 0;
    //ranGene();
  }
}
//***************layer select***************//

//***************Initialize Gray Scale Data array***************//
void brightness_Init() {

  for (int i = 0; i < GSbyteBlock; i++) {
    brightnessValue[i] = 0;
  }
}
//***************Initialize Gray Scale Data array***************//

//=======================================================================================================================//
//***********************************************************************************************************************//
/////////////////////////////////////////////////END OF LOW LEVEL FUNCTIONS////////////////////////////////////////////////
//***********************************************************************************************************************//
//=======================================================================================================================//
