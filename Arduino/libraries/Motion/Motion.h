#ifndef Motor_h
#define Motor_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include <pins_arduino.h>
#endif

class Motion {
	public:
		void  MoveForward_QR(int spd, int dis);
		void  Turn_R_90(int spd);
		void  Turn_L_90(int spd);
		void  Turn_180(int spd);
		void  setEncoderPin(int encoderAPin, int encoderBPin);
		void  setEncoderPin(int encoderAPin, int encoderBPin, long* accmEncoderTicksCount, long* netEncoderTicksCount);
		long  getAccmEncoderTicks();
		void  MotorRotateCW(char wheel, int spd, int dist);
		void  MotorRotateCCW(char wheel, int spd, int dist);
		void  MotorBrakeLow(char wheel);
		
	private:		
		long* _accmEncoderTicksCount;	//Current accumulated ticks count of encoder
		long* _netEncoderTicksCount;	//Net ticks count of encoder
		char _INA1 = 2;
		char _INB1 = 4;
		char _INA2 = 7;
		char _INB2 = 8;
		char _ENCODE_LA = 5;
		char _ENCODE_LB = 6;
		char _ENCODE_L_LAST;
		char _ENCODE_RA;
		char _ENCODE_RB;
		char MOTOR_L = 0;
		char MOTOR_R = 1;
		char PWM_MOTOR_R = 9;
		char PWM_MOTOR_L = 10;
		void tick(void);
};



#endif