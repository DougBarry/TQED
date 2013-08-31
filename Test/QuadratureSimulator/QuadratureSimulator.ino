/*
*
* Kristof Robot
* August 2013
*
* Sketch to simulate a quadrature encoder signal to test wheel encoder counter - using 20MHz external crystal
* TCCR2B =2 -> 4.883 kHz signal (20Mhz/8/256/2), 19.531kHz with 4x encoding 
*        =3 -> 1.221kHz signal, 4.884 kHz with 4x encoding
*        =5 -> 305Hz signal, 1221 Hz with 4x encoding
*        =7 -> 38Hz signal, 152Hz with 4x encoding
*
* Hardware: 
* Atmega328p on breadboard with 20Mhz external crystal
* Configure following fuses to activate external crystal: 
* -U lfuse:w:0xf7:m -U hfuse:w:0xda:m -U efuse:w:0x04:m
*
* Add following to your boards.txt, to upload using Arduino:
uno328p20.name=Arduino Uno (Brown out at 4.3V, Full Swing Crystal 20Mhz)
uno328p20.upload.protocol=arduino
uno328p20.upload.maximum_size=32256
uno328p20.upload.speed=19200
uno328p20.upload.using=arduino:arduinoisp
uno328p20.bootloader.low_fuses=0xf7
uno328p20.bootloader.high_fuses=0xda
uno328p20.bootloader.extended_fuses=0x04
uno328p20.bootloader.path=optiboot
uno328p20.bootloader.file=optiboot_atmega328.hex
uno328p20.bootloader.unlock_bits=0x3F
uno328p20.bootloader.lock_bits=0x0F
uno328p20.build.mcu=atmega328p
uno328p20.build.f_cpu=20000000L
uno328p20.build.core=arduino
uno328p20.build.variant=standard
*/

#include <avr/io.h> 

#define CHANNELA_PIN PB2
#define CHANNELB_PIN PB1

void setup() {
  bitSet(DDRB, CHANNELA_PIN); // Channel A pin
  bitSet(DDRB, CHANNELB_PIN); // Channel B pin
  TCCR2A = 0; // normal mode
  TCCR2B = 2; // Change this to change signal frequency
  TIMSK2 = 1<<OCIE2A | 1<<TOIE2; // enable match and overflow interrupts
  
  OCR2A=128; // 50% pwm duty cycle, this causes channel B to lag 90 degrees behind channel A
}

void loop() {
    //nothing to do, everything is handled in interrupt routines
}

//Channel A sim
ISR(TIMER2_OVF_vect) {
  PINB = (1<<CHANNELA_PIN); // very fast pin toggling
}

//Channel B sim
ISR(TIMER2_COMPA_vect) {
  PINB = (1<<CHANNELB_PIN); 
}


