/*
*
* Kristof Robot
* August 2013
*
* Version 0.1
*
* Sketch to simulate a quadrature encoder signal to test wheel encoder counter - using 20MHz external crystal
* Should also work with other Atmega CPU speeds, with signal will be scaled accordingly.
*
* Hardware: Atmega328p on breadboard with 20Mhz external crystal
* 
* Configure fuses to activate external crystal with following avrdude options: 
* -U lfuse:w:0xf7:m -U hfuse:w:0xda:m -U efuse:w:0x04:m
*
*/

#include <avr/io.h> 
#include <avr/interrupt.h>

//need to be PortB pins, otherwise need to to change sbi commands below
#define CHANNELA_PIN PB2   
#define CHANNELB_PIN PB1

/** Frequency setting **/
//#define SIGNAL_FREQ 2 //divide by 8; 4.883 kHz signal (20Mhz/8(prescaler)/256(overflow)/2(up/down)), 19.531kHz with 4x encoding 
#define SIGNAL_FREQ 3 //divide by 32; 1.221kHz signal, 4.884kHz with 4x encoding
//#define SIGNAL_FREQ 4 //divide by 64; 610Hz signal, 2442Hz with 4x encoding
//#define SIGNAL_FREQ 5 //divide by 128; 305Hz signal, 1221Hz with 4x encoding 
//#define SIGNAL_FREQ 6 //divide by 256; 153Hz signal, 665Hz with 4x encoding
//#define SIGNAL_FREQ 7 //divide by 1024; 38Hz signal, 152Hz with 4x encoding 

/**interrupt routines**/
//Channel A sim
ISR (TIMER2_OVF_vect) {
  PINB = (1<<CHANNELA_PIN); // very fast pin toggling
}

//Channel B sim
ISR (TIMER2_COMPA_vect) {
  PINB = (1<<CHANNELB_PIN); 
}

/**main loop**/
int main(void){
  DDRB = 1<<CHANNELA_PIN | 1<<CHANNELB_PIN; //channel A and B pins as output

  TCCR2A = 0; // normal mode
  TCCR2B = SIGNAL_FREQ; // Change this to change signal frequency
  TIMSK2 = 1<<OCIE2A | 1<<TOIE2; // enable match and overflow interrupts
  
  OCR2A=128; // 50% pwm duty cycle, this causes channel B to lag 90 degrees behind channel A

  //activate interrupts
  sei();

  for(;;){
	//do nothing
  }

  return 0;
}



