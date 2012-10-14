/*

  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)
#define digitalPinHasPWM(p)         ((p) == 4 || (p) == 5 || (p) == 7 || (p) == 11)
#define TIMER0  8 // available on ATMega32/644

#ifdef ARDUINO_MAIN

const static uint8_t SS   = 12;
const static uint8_t MOSI = 13;
const static uint8_t MISO = 14;
const static uint8_t SCK  = 15;

const static uint8_t A0 = 31;
const static uint8_t A1 = 30;
const static uint8_t A2 = 29;
const static uint8_t A3 = 28;
const static uint8_t A4 = 27;
const static uint8_t A5 = 26;
const static uint8_t A6 = 25;
const static uint8_t A7 = 24;
/*
const static uint8_t A0 = 28;
const static uint8_t A1 = 29;
const static uint8_t A2 = 22;
const static uint8_t A3 = 23;
const static uint8_t A4 = 24;
const static uint8_t A5 = 25;
const static uint8_t A6 = 26;
const static uint8_t A7 = 27;
*/

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PD	, // Digital 0 ** PD 0 ** DIP 14 ** USART0_RX	
	PD	, // Digital 1 ** PD 1 ** DIP 15 ** USART0_TX		
	PD  , // Digital 2 ** PD 2 ** DIP 16 ** INT0 
	PD  , // Digital 3 ** PD 3 ** DIP 17 ** INT1
	PD  , // Digital 4 ** PD 4 ** DIP 18 ** OC1B           USABLE 
	PD  , // Digital 5 ** PD 5 ** DIP 19 ** OC1A           USABLE 
	PD  , // Digital 6 ** PD 6 ** DIP 20 ** ICP1           USABLE 
	PD  , // Digital 7 ** PD 7 ** DIP 21 ** OC2            USABLE 
	
	PB  , // Digital 8 ** PB 0 ** DIP 1  ** XCK            USABLE 
	PB  , // Digital 9 ** PB 1 ** DIP 2  ** T1             USABLE 
	PB  , // Digital10 ** PB 2 ** DIP 3  ** INT2/AIN0  
	PB  , // Digital11 ** PB 3 ** DIP 4  ** OC0/AIN1  
	PB  , // Digital12 ** PB 4 ** DIP 5  ** SS
	PB  , // Digital13 ** PB 5 ** DIP 6  ** MOSI
	PB  , // Digital14 ** PB 6 ** DIP 7  ** MISO
	PB  , // Digital15 ** PB 7 ** DIP 8  ** SCK
	
	PC  , // Digital16 ** PC 0 ** DIP 22 **                USABLE 
	PC  , // Digital17 ** PC 1 ** DIP 23 **                USABLE 
	PC  , // Digital18 ** PC 2 ** DIP 24 **                USABLE 
	PC  , // Digital19 ** PC 3 ** DIP 25 **                USABLE 
	PC  , // Digital20 ** PC 4 ** DIP 26 **                USABLE 
	PC  , // Digital21 ** PC 5 ** DIP 27 **                USABLE 
	PC  , // Digital22 ** PC 6 ** DIP 28 **                USABLE 
	PC  , // Digital23 ** PC 7 ** DIP 29 **                USABLE 
	
	PA  , // Digital24 ** PA 7 ** DIP 33 **                USABLE 
	PA  , // Digital25 ** PA 6 ** DIP 34 **                USABLE 
	PA  , // Digital26 ** PA 5 ** DIP 35 **                USABLE 
	PA  , // Digital27 ** PA 4 ** DIP 36 **                USABLE 
	PA  , // Digital28 ** PA 3 ** DIP 37 **                USABLE 
	PA  , // Digital29 ** PA 2 ** DIP 38 **                USABLE 
	PA  , // Digital30 ** PA 1 ** DIP 39 **                USABLE 
	PA  , // Digital31 ** PA 0 ** DIP 40 **                USABLE 
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port D  BV = BitvValue, _BV(a) entspricht (1 << a) */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 8, port B */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(0), /* 16, port C */
	_BV(1),
	_BV(2),
	_BV(3),
	_BV(4),
	_BV(5),
	_BV(6),
	_BV(7),
	_BV(7), /* 24, port A */
	_BV(6),
	_BV(5),
	_BV(4),
	_BV(3),
	_BV(2),
	_BV(1),
	_BV(0),
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,    /* 0 - PD0 */
	NOT_ON_TIMER,	 /* 1 - PD1 */
	NOT_ON_TIMER,	 /* 2 - PD2 */
	NOT_ON_TIMER,    /* 3 - PD3 */
	TIMER1B,         /* 4 - PD4 */
	TIMER1A,         /* 5 - PD5 */
	NOT_ON_TIMER,    /* 6 - PD6 */
	TIMER2,          /* 7 - PD7 */	
	NOT_ON_TIMER,    /* 8 - PB0 */
	NOT_ON_TIMER,    /* 9 - PB1 */
    NOT_ON_TIMER,    /* 10 - PB2 */
	TIMER0,       	 /* 11 - PB3 --- TIMER OC0 */
	NOT_ON_TIMER, /* 12 - PB4 */
	NOT_ON_TIMER, /* 13 - PB5 */
	NOT_ON_TIMER, /* 14 - PB6 */
	NOT_ON_TIMER, /* 15 - PB7 */
	NOT_ON_TIMER, /* 16 - port C */
	NOT_ON_TIMER, /* 17 - PC1 */
	NOT_ON_TIMER, /* 18 - PC2 */
	NOT_ON_TIMER, /* 19 - PC3 */
	NOT_ON_TIMER, /* 20 - PC4 */
	NOT_ON_TIMER, /* 21 - PC5 */
	NOT_ON_TIMER, /* 22 - PC6 */
	NOT_ON_TIMER, /* 23 - PC7 */
	NOT_ON_TIMER, /* 24, port A */
	NOT_ON_TIMER, /* 25 - PA6 */
	NOT_ON_TIMER, /* 26 - PA5 */
	NOT_ON_TIMER, /* 27 - PA4 */
	NOT_ON_TIMER, /* 28 - PA3 */
	NOT_ON_TIMER, /* 29 - PA2 */
	NOT_ON_TIMER, /* 30 - PA1 */
	NOT_ON_TIMER, /* 31 - PA0 */
};

#endif
#endif
