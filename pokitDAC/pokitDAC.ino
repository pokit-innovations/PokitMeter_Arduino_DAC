/*
PokitMeter Arduino 1kHz DAC Signal Generator
Copyright (c) Pokit Innovations 2021

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 This assumes an R/2R DAC (or other DAC) connected to PORTB, and
 Arduino 16mhz clock
 
 Output to pb5-pb0 with 6 bits, with MSB on PB5. Bit shift output for a byte.
 */

#include <avr/interrupt.h>
#include <stdlib.h>

const unsigned int TABLE_SIZE = 128;

const unsigned int ALWAYS_ON = 19;  //SCL pin
const unsigned int SQUARE_WAVE = 18;  //SDA pin
const unsigned int IDLE_SIG = 17;  //A3 pin
const unsigned int PWM = 6;

char wavetable [TABLE_SIZE];
int  phase = 0;
int  waveform = 0;

void ioinit (void)
{
  //Initialize output ports

  //PORTB is used for R-2R DAC
  PORTB = B11111111;
  DDRB  = B11111111;

  pinMode(ALWAYS_ON, OUTPUT);
  pinMode(SQUARE_WAVE, OUTPUT);
  pinMode(IDLE_SIG, OUTPUT);
  pinMode(PWM, OUTPUT);

  digitalWrite(ALWAYS_ON, LOW);
  digitalWrite(SQUARE_WAVE, LOW);
  digitalWrite(PWM, LOW);
  
  Serial.begin(115200);
}

void timer_setup(){
  // Clear registers
  TCCR2A = 0;
  TCNT2 = 0;
  TCCR2B = 0;

  OCR2A = 124;  // 128000 Hz (16000000/((124+1)*1))
  
  // Prescaler 1
  TCCR2B |= (1 << CS20);
  // CTC
  TCCR2A |= (1 << WGM21);
  // Output Compare Match A Interrupt Enable
  TIMSK2 |= (1 << OCIE2A);
}

void setup(){          

  ioinit();

  //fliptable();  //DAC is wired in reverse for ease of use...
  cli();
  timer_setup();
  phase = 0;
  sei();

  Serial.println("PokitDAC Running...");
  randomSeed(analogRead(0));
  waveform = random(3);
  //waveform = 0;
  if (waveform == 0) {
    sinewave();
    Serial.println("Randomly picked sine function");
  } else if (waveform == 1 ) {
    rampwave();
    Serial.println("Randomly picked ramp function");
  } else {
    triwave();
    Serial.println("Randomly picked triangle function");
  }
  

}


//ISR(TIMER2_OVF_vect) {
ISR(TIMER2_COMPA_vect) {

  int waveval = wavetable[phase]>>2;
  int squareval = 0;

  squareval = (phase >= TABLE_SIZE>>1);

  //digitalWrite is too slow in interrupts - use direct port access
  PORTC |= 0b00100000; //set PC5
  
  if(squareval) {
    PORTC |= 0b00010000; //set PC4
  } else {
    PORTC &= ~0b00010000; //clear PC4
  }
    
  PORTB=waveval;  // write to port should be single cycle

  phase++;
  if(phase==TABLE_SIZE){
    phase=0;
  }


}


void rampwave(void){
  int stepval = 256/TABLE_SIZE;
  for (int i = 0; i < TABLE_SIZE; i++) {
    wavetable[i] = i*stepval;
  }
}

void triwave(void){
  int half = TABLE_SIZE>>1;
  int stepval = (256/TABLE_SIZE)*2;
  for (int i = 0; i < half; i++) {
    wavetable[i] = i*stepval;
  }
  for (int i = half; i < TABLE_SIZE; i++) {
    wavetable[i] = ((TABLE_SIZE-i)*stepval -1);
  }

}


void sinewave(void){
  wavetable[0] = 128;
  wavetable[1] = 134;
  wavetable[2] = 140;
  wavetable[3] = 146;
  wavetable[4] = 152;
  wavetable[5] = 159;
  wavetable[6] = 165;
  wavetable[7] = 171;
  wavetable[8] = 176;
  wavetable[9] = 182;
  wavetable[10] = 188;
  wavetable[11] = 193;
  wavetable[12] = 199;
  wavetable[13] = 204;
  wavetable[14] = 209;
  wavetable[15] = 213;
  wavetable[16] = 218;
  wavetable[17] = 222;
  wavetable[18] = 226;
  wavetable[19] = 230;
  wavetable[20] = 234;
  wavetable[21] = 237;
  wavetable[22] = 240;
  wavetable[23] = 243;
  wavetable[24] = 246;
  wavetable[25] = 248;
  wavetable[26] = 250;
  wavetable[27] = 252;
  wavetable[28] = 253;
  wavetable[29] = 254;
  wavetable[30] = 255;
  wavetable[31] = 255;
  wavetable[32] = 255;
  wavetable[33] = 255;
  wavetable[34] = 255;
  wavetable[35] = 254;
  wavetable[36] = 253;
  wavetable[37] = 252;
  wavetable[38] = 250;
  wavetable[39] = 248;
  wavetable[40] = 246;
  wavetable[41] = 243;
  wavetable[42] = 240;
  wavetable[43] = 237;
  wavetable[44] = 234;
  wavetable[45] = 230;
  wavetable[46] = 226;
  wavetable[47] = 222;
  wavetable[48] = 218;
  wavetable[49] = 213;
  wavetable[50] = 209;
  wavetable[51] = 204;
  wavetable[52] = 199;
  wavetable[53] = 193;
  wavetable[54] = 188;
  wavetable[55] = 182;
  wavetable[56] = 176;
  wavetable[57] = 171;
  wavetable[58] = 165;
  wavetable[59] = 159;
  wavetable[60] = 152;
  wavetable[61] = 146;
  wavetable[62] = 140;
  wavetable[63] = 134;
  wavetable[64] = 128;
  wavetable[65] = 121;
  wavetable[66] = 115;
  wavetable[67] = 109;
  wavetable[68] = 103;
  wavetable[69] = 96;
  wavetable[70] = 90;
  wavetable[71] = 84;
  wavetable[72] = 79;
  wavetable[73] = 73;
  wavetable[74] = 67;
  wavetable[75] = 62;
  wavetable[76] = 56;
  wavetable[77] = 51;
  wavetable[78] = 46;
  wavetable[79] = 42;
  wavetable[80] = 37;
  wavetable[81] = 33;
  wavetable[82] = 29;
  wavetable[83] = 25;
  wavetable[84] = 21;
  wavetable[85] = 18;
  wavetable[86] = 15;
  wavetable[87] = 12;
  wavetable[88] = 9;
  wavetable[89] = 7;
  wavetable[90] = 5;
  wavetable[91] = 3;
  wavetable[92] = 2;
  wavetable[93] = 1;
  wavetable[94] = 0;
  wavetable[95] = 0;
  wavetable[96] = 0;
  wavetable[97] = 0;
  wavetable[98] = 0;
  wavetable[99] = 1;
  wavetable[100] = 2;
  wavetable[101] = 3;
  wavetable[102] = 5;
  wavetable[103] = 7;
  wavetable[104] = 9;
  wavetable[105] = 12;
  wavetable[106] = 15;
  wavetable[107] = 18;
  wavetable[108] = 21;
  wavetable[109] = 25;
  wavetable[110] = 29;
  wavetable[111] = 33;
  wavetable[112] = 37;
  wavetable[113] = 42;
  wavetable[114] = 46;
  wavetable[115] = 51;
  wavetable[116] = 56;
  wavetable[117] = 62;
  wavetable[118] = 67;
  wavetable[119] = 73;
  wavetable[120] = 79;
  wavetable[121] = 84;
  wavetable[122] = 90;
  wavetable[123] = 96;
  wavetable[124] = 103;
  wavetable[125] = 109;
  wavetable[126] = 115;
  wavetable[127] = 121;
}

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; // swap nibbles
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2; // swap outer 2 bits
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1; // swap odd bits with even bits
   return b;
}

void fliptable(void){
  for (int i =0; i<TABLE_SIZE; i++) {
    wavetable[i] = reverse(wavetable[i]);
  }  
}

void loop()
{
  while (1)
  {
    // "IDLE signal"
    PORTC |= 0b00001000; //set PC3
    PORTC &= ~0b00001000; //clear PC3
  }
  

}
