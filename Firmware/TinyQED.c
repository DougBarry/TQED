/***************************************************************************
* File              : TinyQED
* Compiler          : AVRstudio 6.2
* Revision          : 1.4
* Date              : Saturday, August 31, 2013
* Revised by        : Douglas Barry
*					:  Broke I2C commands out, added min/max/address commands. Moved to Atmel Studio 6.2 and Native compiler.
*					:  Other minor changes.
*					: Kristof Robot
*					:  Added 4x resolution based on http://tutorial.cytron.com.my/2012/01/17/quadrature-encoder/
*					: Adriaan Swanepoel
*                   :  Adapted from Dan Gates I2C analogue slave
*					   http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=51467
*                      and Jim Remington's Quadrature encoder for the O 
*					   http://forum.pololu.com/viewtopic.php?t=484
*
* Target device		: ATtiny45
*
* Connections
*                               ATTiny45
*                 +--------------------------------+
*                 | 1 pb5 reset              VCC 8 | VCC
*       Channel A | 2 pb3                    pb2 7 | SCL
*       Channel B | 3 pb4                    pb1 6 | 
*             GND | 4 GND                    pb0 5 | SDA
*                 +--------------------------------+
****************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <limits.h>
#include "usiTwiSlave.h"
#include "../TQED_I2CCMD.h"

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define DEFAULTADDRESS 0x36
#define DEFAULTADDRESS 0x39

//uncomment for 4x resolution; default is 2x resolution
#define ENCODER_4X_RESOLUTION

union quadruplebyte
{
  int32_t value;
  unsigned char bytes[4];
};

volatile int32_t enc_pos=0L;

#ifdef ENCODER_4X_RESOLUTION
#else
unsigned char enc_dir;
unsigned char enc_last=0;
unsigned char enc_now;
#endif

unsigned char EEMEM slaveaddress = DEFAULTADDRESS;  //Default address

/******************************************************************************
 *
 * 
 *
 * Description:	
 * ARGS:		none
 * RETURN:		none
 *
 *****************************************************************************/
#ifdef ENCODER_4X_RESOLUTION
//4x version, based on http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
ISR (PCINT0_vect)
{
	static uint8_t enc_last=0;
	static const int8_t enc_states [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

	enc_last <<=2; //shift previous state two places
	enc_last |= (PINB & (3 << 3)) >> 3; //read the current state into lowest 2 bits

	enc_pos += enc_states[(enc_last & 0x0f)];
	
}
#else
//2x resolution; default
ISR (PCINT0_vect)
{
	enc_now = (PINB & (3 << 3)) >> 3;             //read the port pins and shift result to bottom bits
	enc_dir = (enc_last & 1) ^ ((enc_now & 2) >> 1); //determine direction of rotation
  
	if (enc_dir == 0)
		enc_pos++;
	else
		enc_pos--;	//update encoder position
  
	enc_last = enc_now;		//remember last state
}
#endif

/******************************************************************************
 *
 * setaddress
 *
 * Description:	Updates the I2C address in the internal eeprom
 * ARGS:		The new I2C address
 * RETURN:		none
 *
 *****************************************************************************/
void setaddress(unsigned char address)
{
	eeprom_write_byte(&slaveaddress, (uint8_t*)(uint16_t)address);
}

/******************************************************************************
 *
 * readaddress
 *
 * Description:	Reads the preprogrammed I2C address from the internal eeprom
 * ARGS:		none
 * RETURN:		The devices I2C address
 *
 *****************************************************************************/
unsigned char readaddress()
{
	return eeprom_read_byte((uint8_t*)&slaveaddress);
}

/******************************************************************************
 *
 * main
 *
 * Description:	Where it all starts...
 * ARGS:		none
 * RETURN:		none
 *
 *****************************************************************************/
int main(void)
{
	unsigned char i2ccmd;
	enc_pos = 0L;
	union quadruplebyte position;

#ifdef ENCODER_4X_RESOLUTION
	//4x resolution
	PCMSK |= (1 << PCINT3)|(1 << PCINT4); // tell pin change mask to listen to pin2 (PB3) and pin3 (PB4)
#else
	//2x resolution; default
	PCMSK |= (1 << PCINT3); // tell pin change mask to listen to pin2
#endif
	GIMSK |= (1 << PCIE);   // enable PCINT interrupt in the general interrupt mask
  
	sei(); //enable interrupts

	cbi(DDRB, DDB3);        // PB3 set up as input
	cbi(DDRB, DDB4);        // PB4 set up as input
	sbi(PORTB, PB3);	    // Set PB3 internal pull up
	sbi(PORTB, PB4);	    // Set PB4 internal pull up

	setaddress(DEFAULTADDRESS);

	//Fix for new MCU and/or user error
	uint8_t address = readaddress();
	if (address >= 127)
	{
		address = DEFAULTADDRESS;
		setaddress(address);
	}

	usiTwiSlaveInit(address);

	for (;;)
	{	
		if (usiTwiDataInReceiveBuffer())
		{
			i2ccmd = usiTwiReceiveByte();
     
			/*
			* Replaced switch statement by if-else construction
			* and put in order of most frequently used
			* to speed up most frequent call (read counter)
			*/
			if (i2ccmd == TQED_READ_COUNTER) //read counter
			{
				 //store current enc_pos in quadruplebyte position variable
				 position.value = enc_pos;
				 usiTwiTransmitByte(position.bytes[0]);
				 usiTwiTransmitByte(position.bytes[1]); 
				 usiTwiTransmitByte(position.bytes[2]);
				 usiTwiTransmitByte(position.bytes[3]); 
			}
			#ifndef ENCODER_4X_RESOLUTION
			else if (i2ccmd == TQED_READ_LASTDIR)
			{
				usiTwiTransmitByte(enc_dir);
			}
			#endif
			else if (i2ccmd == TQED_RESET_COUNTER) enc_pos = 0L; //reset counter
			else if (i2ccmd == TQED_CENTER_COUNTER) enc_pos = 0L; //center counter value
			else if (i2ccmd == TQED_GET_COUNTERMIN)
			{
				position.value = (-(INT32_MAX)-1);
				usiTwiTransmitByte(position.bytes[0]);
				usiTwiTransmitByte(position.bytes[1]);
				usiTwiTransmitByte(position.bytes[2]);
				usiTwiTransmitByte(position.bytes[3]);
			}
			else if (i2ccmd == TQED_GET_COUNTERMAX)
			{
				position.value = INT32_MAX;
				usiTwiTransmitByte(position.bytes[0]);
				usiTwiTransmitByte(position.bytes[1]);
				usiTwiTransmitByte(position.bytes[2]);
				usiTwiTransmitByte(position.bytes[3]);
			}
			else if (i2ccmd == TQED_SET_ADDRESS) setaddress(usiTwiReceiveByte());
			else if (i2ccmd == TQED_GET_ADDRESS) // return current address, as test for connection
			{
				usiTwiTransmitByte(address);
			}
		}
	}
}