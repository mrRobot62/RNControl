/*



*/
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//--------------------------------------------------------------------------


#include "RNControl.h"



extern "C" {
//#include "WConstants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
}

RNControl::RNControl() {
#if USE_MOTOR != 0
	this->initMotorStuff();
	this->stopMotor(MOTOR_BOTH);
	this->motorOffset.motorOne = 0;
	this->motorOffset.motorTwo = 0;
	this->motorOffset.motorOneReverse = 0;
	this->motorOffset.motorTwoReverse = 0;
	this->minSpeed = 0;
#endif	
}

/************************************************************************************
	methods around digital IO
*************************************************************************************/

void RNControl::initMotorStuff(void) {
	// channels for motor 1
	DDRB |= (1<<PB0) | (1<<PB1);
	// channels for motor 0
	DDRC |= (1<<PC6) | (1<<PC7);
	// PWM-channel for motor 0+1
	DDRD |= (1<<PD4) | (1<<PD5);
	
	//
	// 8bit PWM initialize (non inverted)
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);

	// pwm-freq on 14khz (prescaler to 1)
	TCCR1B = (1<<CS10);

	// disable timer1 interrupts
   	TIMSK &= ~0x3c;	
}


/************************************************************************************
	methods around motors
*************************************************************************************/
#if USE_MOTOR != 0

void RNControl::setMotorMinSpeed(uint8_t minSpeed) {
	this->minSpeed = minSpeed;
}

/**
	Offset ist ein prozentualer Wert zwischen MIN_OFFSET bis MAX_OFFSET
	der auf die aktuelle Geschwindigkeit aufaddiert wird
	
	Beispiel1:
	speed = 150
	offset = +10
	effektiv speed =	150 + 15 = 165
	
	Beispiel2:
	speed = 150
	offset = -10
	effektiv speed =	150 - 15 = 135
	
	
**/
void RNControl::setMotorOffset(uint8_t id, int8_t offset){
	switch(id) {
		case MOTOR_ONE:
			motorOffset.motorOne = constrain(offset,MIN_OFFSET, MAX_OFFSET);
			break;
		case MOTOR_TWO:
			motorOffset.motorTwo = constrain(offset,MIN_OFFSET, MAX_OFFSET);
			break;
			
	}
}

void RNControl::setMotorReverse(uint8_t id, bool reverse) {
	switch(id) {
		case MOTOR_ONE:
			this->motorOffset.motorOneReverse = reverse;
			break;
		case MOTOR_TWO:
			this->motorOffset.motorTwoReverse = reverse;
			break;
		case MOTOR_BOTH:
			this->motorOffset.motorOneReverse = reverse;
			this->motorOffset.motorTwoReverse = reverse;
			break;
	}
}
void RNControl::setDir(uint8_t id, uint8_t dir) {
	switch(dir) {
		case MOTOR_DIR_FORWARD:
			switch (id) {
				case MOTOR_ONE:
					if (this->motorOffset.motorOneReverse == 0) {
						PORTC |= (1<<PC6) ;
						PORTC &= ~(1<<PC7);
					}
					else {
						PORTC |= (1<<PC7) ;
						PORTC &= ~(1<<PC6);			
					}
					break;
				case MOTOR_TWO:
					if (this->motorOffset.motorTwoReverse == 0) {
						PORTB |= (1<<PB0);
						PORTB &= ~(1<<PB1);
					}
					else {
						PORTB |= (1<<PB1);
						PORTB &= ~(1<<PB0);
					}
					break;
			}
			break;
		case MOTOR_DIR_BACKWARD:
			switch (id) {
				case MOTOR_ONE:
					if (this->motorOffset.motorOneReverse == 0) {
						PORTC |= (1<<PC7) ;
						PORTC &= ~(1<<PC6);
					}
					else {
						PORTC |= (1<<PC6) ;
						PORTC &= ~(1<<PC7);			
					}
					break;
				case MOTOR_TWO:
					if (this->motorOffset.motorTwoReverse == 0) {
						PORTB |= (1<<PB1);
						PORTB &= ~(1<<PB0);
					}
					else {
						PORTB |= (1<<PB0);
						PORTB &= ~(1<<PB1);
					}
					break;
			}
			break;	
	}

}

void RNControl::motorExt(int16_t speedMotorOne, int16_t speedMotorTwo) {
	uint8_t dirOne, dirTwo;
	dirOne = (speedMotorOne < 0?MOTOR_DIR_BACKWARD:MOTOR_DIR_FORWARD);
	dirTwo = (speedMotorTwo < 0?MOTOR_DIR_BACKWARD:MOTOR_DIR_FORWARD);
	
	speedMotorOne = speedMotorOne+(speedMotorOne * motorOffset.motorOne) / 100;
	speedMotorTwo = speedMotorTwo+(speedMotorTwo * motorOffset.motorOne) / 100;

	setDir(MOTOR_ONE, dirOne);
	OCR1BL = map(speedMotorOne,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
	
	setDir(MOTOR_TWO, dirTwo);
	OCR1AL = map(speedMotorTwo,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
	
	// #if ((DEBUG_LEVEL > 0) && (DEBUG_LIB == DEBUG_LIB_RNCONTROL))
		Serial.print("ExtSpeed ONE/TWO:");
		Serial.print(speedMotorOne,DEC); Serial.print(" :"); Serial.print((dirOne != 0)?"BW/":"FW/");
		Serial.print(speedMotorTwo,DEC); Serial.print(" :"); Serial.print((dirTwo != 0)?"BW/":"FW/");
		Serial.println();
	// #endif
	
}


void RNControl::motor(uint8_t id, uint16_t speed, uint8_t dir) {
	speed = constrain(speed,1,MAX_SPEED);
	speed = calcMotorMinSpeed(speed);
	switch(id) {
		case MOTOR_ONE:
			// ggf einen Offset auf den Motor addieren
			speed = speed+(speed * motorOffset.motorOne) / 100;
			// speed grundsätzlich in die Range anpassen
			speed = constrain(speed,1,MAX_SPEED);
			// Speed in die Range Min_Speed bis Max_Speed anpassen
			OCR1BL = map(speed,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
			//OCR1BL = (uint8_t)speed;
			//Serial.print("MotorOne:");
			//Serial.println(speed);
			setDir(id, dir);
			break;
		case MOTOR_TWO:
			speed = speed+(speed * motorOffset.motorTwo) / 100;
			// speed grundsätzlich in die Range anpassen
			speed = constrain(speed,1,MAX_SPEED);
			// Speed in die Range Min_Speed bis Max_Speed anpassen
			OCR1AL = map(speed,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
			//OCR1AL = speed;
			// Serial.print("MotorTwo:");
			// Serial.println(speed);
			setDir(id, dir);
			break;				
		case MOTOR_BOTH:
			uint16_t mOne, mTwo;
			mOne = speed+(speed * motorOffset.motorOne) / 100;
			mTwo = speed+(speed * motorOffset.motorTwo) / 100;
			mOne = constrain(mOne,1,MAX_SPEED);
			mTwo = constrain(mTwo,1,MAX_SPEED);
			// Serial.print("MotorOne:"); Serial.println(mOne);
			// Serial.print("MotorTwo:"); Serial.println(mTwo);
			OCR1BL = map(mTwo,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
			OCR1AL = map(mOne,1,MAX_SPEED,this->minSpeed,MAX_SPEED);
			//OCR1AL = mOne;
			//OCR1BL = mTwo;
			
			setDir(MOTOR_ONE, dir);
			setDir(MOTOR_TWO, dir);
			break;				
	}	
}

void RNControl::fwMotor(uint8_t id, uint8_t speed) {
	this->motor(id, speed, FORWARD);	
}

void RNControl::bwMotor(uint8_t id, uint8_t speed) {
	this->motor(id, speed, BACKWARD);		
}

void RNControl::stopMotor(uint8_t id) {
	switch(id) {
		case MOTOR_ONE:
			PORTC &= ~((1<<PC6) | (1<<PC7));
			break;
		case MOTOR_TWO:
			PORTB &= ~((1<<PB0) | (1<<PB1));
			break;				
		case MOTOR_BOTH:
			PORTC &= ~((1<<PC6) | (1<<PC7));
			PORTB &= ~((1<<PB0) | (1<<PB1));
			break;				
	}	
}

void RNControl::arcMotors(uint8_t arcDir, uint8_t arcRadius, uint8_t speed){
	uint8_t offset, speedTmp;
	arcRadius = constrain(arcRadius, 0, MAX_ARC_RADIUS);
	switch(arcDir) {
		case MOTOR_ARC_CW:
			if (arcRadius == 0) {
				fwMotor(MOTOR_ONE,speed);
				bwMotor(MOTOR_TWO,speed);
			}
			else {
				arcRadius = (MAX_ARC_RADIUS+1) - arcRadius;
				arcRadius *= 2;
				offset = ((speed * arcRadius) / 100);
				//offset = constrain(offset,1,speed);
				speedTmp = constrain (speed+offset, 1, MAX_SPEED);
				fwMotor(MOTOR_ONE,speedTmp);
				speedTmp = constrain (speed-offset, 1, MAX_SPEED);
				fwMotor(MOTOR_TWO,speedTmp);
			}
			break;
			
		case MOTOR_ARC_CCW:
			if (arcRadius == 0) {
				bwMotor(MOTOR_ONE,speed);
				fwMotor(MOTOR_TWO,speed);
			}
			else {
				arcRadius = (MAX_ARC_RADIUS+1) - arcRadius;
				arcRadius *= 2;
				offset = ((speed * arcRadius) / 100);
				//offset = constrain(offset,1,speed);
				speedTmp = constrain (speed-offset, 1, MAX_SPEED);
				fwMotor(MOTOR_ONE,speedTmp);
				speedTmp = constrain (speed+offset, 1, MAX_SPEED);
				fwMotor(MOTOR_TWO,speedTmp);
			}
			break;
			
	
	}
}

uint8_t RNControl::calcMotorMinSpeed(uint8_t speed) {
	if ( (minSpeed != 0) && (speed < minSpeed) ) {
		return minSpeed;
	}
	return speed;
}

#endif

/************************************************************************************
	methods around analog io
*************************************************************************************/
uint16_t RNControl::readAnalog(uint8_t pin) {
	adc_value = 0;
	// ADEN activate, set PreScaler = 128 for freq 50-200khz
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	
	ADMUX = pin;
	
	// dummy readout
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {;}	
	adc_value = ADCW;
	
	// real adc read, we read four times
	adc_value = 0;
	for (uint8_t i=0; i < 4; i++) {
		ADCSRA |= (1<<ADSC); 				// single conversion
		while (ADCSRA & (1<<ADSC)) {;}		// wait for finish
		adc_value += ADCW;
	}
	ADCSRA &= ~(1<<ADEN);
	adc_value /= 4;
	
	return adc_value;
	
}

uint16_t RNControl::getButton() {
	uint8_t button = 0; 				// Button number 1-5
	uint16_t buttonPin = this->readAnalog(7);	// read analog port 7
	
	//Serial.println (buttonPin);Serial.print("B:");Serial.println((int)button);
	// pull up for pin 7 
	PORTA |= (1<<7);
	
	if (buttonPin > 1000) return button;
	
	if(		(buttonPin>=400) && (buttonPin<=410)) {button = 1;}
	else if((buttonPin>=335) && (buttonPin<=345)) {button = 2;}
	else if((buttonPin>=262) && (buttonPin<=272)) {button = 3;}
	else if((buttonPin>=182) && (buttonPin<=192)) {button = 4;}
	else if((buttonPin>=100) && (buttonPin<=110)) {button = 5;}
	
	//Serial.println (buttonPin);Serial.print("B2:");Serial.println((int)button);
	return button;
	
}

/************************************************************************************
	methods around serial communication
*************************************************************************************/
void RNControl::initUART(void) {
	UCSRB |= (1<<TXEN);					// uart tx
	UCSRC |= (1<<URSEL) | (3<<UCSZ0);	// 8n1	
	UBRRH = 0;							// high byte 0
	UBRRL = 103;						// (freq / (baud*16))-1  <-- quarz = 16*1000*1000 hz
}

void RNControl::sendChar(uint8_t c) {
	while (!(UCSRA & (1<<UDRE))) {;}	// wait until send is possible
	UDR = c;							// send c to rs232
}

void RNControl::sendString(uint8_t *s) {
	while (*s) {
		sendChar(*s);
		s++;	
	}
}

/************************************************************************************
	methods around sound
*************************************************************************************/
void RNControl::sound (uint16_t freq, uint8_t time){
	time *= 15;
	DDRD |= (1<<7);
	for (uint16_t i=0; i < time; i += (freq << 1)) {
		PORTD |= (1<<7);
		delayMicroseconds(32*(freq<<7));
		PORTD &= ~(1<<7);
		delayMicroseconds(32*(freq<<7));				
	}	
}

/************************************************************************************
	methods around 
*************************************************************************************/
