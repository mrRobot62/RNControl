#ifndef RNCONTROL_H
#define RNCONTROL_H

#include <inttypes.h>

#define VERSION 05
#warning "RNControl Version " VERSION


#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "pins_arduino.h"
    #include "WProgram.h"
#endif

/*******************************************************************
Version					Stand
0.1						first version
0.2			23-05		implement arcMotor, setMotorOffset()
0.3			29-05		Bugfix, implement setMotorReverse()
0.4			22-06		MotorExt, implement motorExt()
0.5			11-12		Make it compatible to Arduino 1.0
********************************************************************/

#define FALSE 0
#define TRUE  1

#define ON 1
#define OFF 0

#define DEBUG_LIB_RNCONTROL 99

// #define DEBUG_LIB DEBUG_LIB_RNCONTROL

// if no motors used, all motor pins are available for normal io
// Timer1 is free for personal use
//	USE_MOTOR 1	=> use motor and timer 1
//  USE_MOTOR 0 => do not use motor
#define USE_MOTOR 1

#if USE_MOTOR != 0
	#define FORWARD 0
	#define BACKWARD 1
	#define MOTOR_ONE 0
	#define MOTOR_TWO 1
	#define MOTOR_BOTH 2
	#define MOTOR_DIR_FORWARD 0
	#define MOTOR_DIR_BACKWARD 1
	#define MOTOR_ARC_CW 0
	#define MOTOR_ARC_CCW 1
	#define MAX_ARC_RADIUS 20
	#define MAX_OFFSET 25
	#define MIN_OFFSET -25
	#define MAX_SPEED 255
	#define MOTOR_REVERSE -1
#endif


class RNControl
{
	private:
		uint16_t adc_value;
		uint8_t useMotorFlag;
		uint8_t minSpeed;
		struct {
			int8_t motorOne;
			int8_t motorTwo;
			int8_t motorOneReverse;
			int8_t motorTwoReverse;
		} motorOffset;
		
		
		void initMotorStuff();
		void setDir(uint8_t id, uint8_t dir);
		uint8_t calcMotorMinSpeed(uint8_t speed);
		
	public:
		RNControl();
		
		// read/write digital port
		// uint8_t TestPortC(uint8_t pin, uint8_t onoff, uint8_t inout);
		
		// read analog port on pin
		uint16_t readAnalog(uint8_t pin);

#if USE_MOTOR != 0	

		//
		// some motors need a minimum PWM-Speed for turn.
		// if minSpeed is set, lower speeds will be set to minSpeed;
		void setMotorMinSpeed(uint8_t minSpeed);

		//
		// set an speed offset for Motor. This can be used if one of the motors
		// is faster/slower as the other on. 
		// The offset is add/sub from the current motor speed
		//
		// 
		void setMotorOffset(uint8_t id, int8_t offset);
	
		//
		// set motor to revers
		// if reverse = true, motor turn CCW
		// if reverse = false, motor turn CW
		void setMotorReverse(uint8_t id, bool reverse);
		
		// generic motor command
		// params: id (0,1), speed (0-255), dir (0,1)
		// if id = MOTOR_BOTH this command set params for both motors
		void motor(uint8_t id, uint16_t speed, uint8_t dir);
		
		// generic motor extend command for both motors
		// handles speed from -255 to +255 
		//
		void motorExt(int16_t speedMotorOne, int16_t speedMotorTwo);
		
		// Motor id with speed forward
		// if id = MOTOR_BOTH this command set params for both motors
		void fwMotor(uint8_t id, uint8_t speed);
		
		// Motor id with speed backward
		// if id = MOTOR_BOTH this command set params for both motors
		void bwMotor(uint8_t id, uint8_t speed);
		
		// use both motors. speed for one motor is lower as form the other
		// bot turns in an arc
		// arcDir : ARC_LEFT, Bot turns to left
		// arcDIR: ARC_RIGHT Bot turns to right
		// radius: (0-10), 0 = center turn
		//					1-10 turn with radius, 10=biggest radius
		// speed: 
		void arcMotors(uint8_t arcDir, uint8_t radius, uint8_t speed);
		
		// set default speed for motors
		//void setDefaultSpeed(uint8_t left, uint8_t right);
		
		void stopMotor(uint8_t id);
		
		
		
#endif		
		// read button on board, return button number (1-5)
		uint16_t getButton();
		
		//
		// initialize build in UART 
		void initUART(void);
		
		//
		// send character to UART
		void sendChar(uint8_t c);
		
		// send string to UART
		void sendString(uint8_t *s);
		
		//
		// make noice
		void sound (uint16_t freq, uint8_t time);
};

#endif
