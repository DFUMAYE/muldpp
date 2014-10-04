#include "Motion.h"


/******************************************************************
============================Motion Function=======================
*******************************************************************/

//Robot move forward, spd from 0 - 255, dist
void Motion::MoveForward_QR(int spd, int dis)
{
	analogWrite(PWM_MOTOR_R, spd);
	analogWrite(PWM_MOTOR_L, spd);

	if (spd == 0)
	{
		MotorBrakeLow(MOTOR_L);
		MotorBrakeLow(MOTOR_R);
	}
	else
	{
		MotorRotateCCW(MOTOR_L, spd, 0);
		MotorRotateCW(MOTOR_R, spd, 0);
	}
}
void  Motion::Turn_R_90(int spd)
{
	if (spd == 0)
	{
		MotorBrakeLow(MOTOR_L);
		MotorBrakeLow(MOTOR_R);
	}
	else
	{
		MotorRotateCCW(MOTOR_L, spd, 0);
		MotorRotateCCW(MOTOR_R, spd, 0);
	}
}
void  Motion::Turn_L_90(int spd)
{
	if (spd == 0)
	{
		MotorBrakeLow(MOTOR_L);
		MotorBrakeLow(MOTOR_R);
	}
	else
	{
		MotorRotateCW(MOTOR_L, spd, 0);
		MotorRotateCW(MOTOR_R, spd, 0);
	}
}
void  Motion::Turn_180(int spd)
{
	if (spd == 0)
	{
		MotorBrakeLow(MOTOR_L);
		MotorBrakeLow(MOTOR_R);
	}
	else
	{
		MotorRotateCCW(MOTOR_L, spd, 0);
		MotorRotateCCW(MOTOR_R, spd, 0);
	}
}
/******************************************************************
==========================End Motion Function=====================
*******************************************************************/

void Motion::setEncoderPin(int encoderAPin, int encoderBPin) {
	long a = 0;
	long b = 0;
	this->setEncoderPin(encoderAPin, encoderBPin, &a, &b);
}

void Motion::setEncoderPin(int encoderAPin, int encoderBPin, long* accmEncoderTicksCount, long* netEncoderTicksCount) {
	pinMode(_ENCODE_LA, INPUT);	//Set left encoder pin as input
	pinMode(_ENCODE_LB, INPUT);	//Set left encoder pin as input
	
	digitalWrite(_ENCODE_LA, LOW);
	digitalWrite(_ENCODE_LB, LOW);
	
	//Register interrupt for encoder A pin
	//PCintPort::attachInterrupt(encoderAPin, this, RISING);
	
	_accmEncoderTicksCount = accmEncoderTicksCount;
	_netEncoderTicksCount = netEncoderTicksCount;
}
void Motion::tick(void)
{
	int sig1 = digitalRead(_ENCODE_LA);
	int sig2 = digitalRead(_ENCODE_LB);
	
	if(_ENCODE_L_LAST != (sig1 | (sig2 << 1))) //only signal change will be recorded
	{
		_accmEncoderTicksCount ++;
		
		_ENCODE_L_LAST = (sig1 | (sig2 << 1));	//replace last state with current state
	}
}

long Motion::getAccmEncoderTicks() {	//Get the total number of ticks travelled by the motor
	return *_accmEncoderTicksCount;
}

void Motion::MotorRotateCW(char wheel, int spd, int dist)
{
	switch(wheel)
	{
	case 0:  //Left Wheel
		digitalWrite(_INA2, HIGH);
		digitalWrite(_INB2, LOW);
		tick();
		break;
	case 1:  //Right Wheel
		digitalWrite(_INA1, HIGH);
		digitalWrite(_INB1, LOW);
		break;
	default:
		break;
	}
}
void Motion::MotorRotateCCW(char wheel, int spd, int dist)
{
 switch(wheel)
 {
   case 0:  //Left Wheel
     digitalWrite(_INA2, LOW);
     digitalWrite(_INB2, HIGH);
     break;
   case 1:  //Right Wheel
     digitalWrite(_INA1, LOW);
     digitalWrite(_INB1, HIGH);
     break;
    default:
      break;
 }
}
void Motion::MotorBrakeLow(char wheel)
{
 switch(wheel)
 {
   case 0:  //Left Wheel
     digitalWrite(_INA2, LOW);
     digitalWrite(_INB2, LOW);
     break;
   case 1:  //Right Wheel
     digitalWrite(_INA1, LOW);
     digitalWrite(_INB1, LOW);
     break;
    default:
      break;
 }
}
