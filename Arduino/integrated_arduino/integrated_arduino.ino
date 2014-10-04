#include <PinChangeInt.h>
#include <PID_v1.h>

/*#define MOTOR_BRAKE 0b00
#define FORWARD 0b11
#define TURN_L_90 0b10
#define TURN_R_90 0b01
#define TURN_180 0b00

#define MOTOR_R_FORWARD 0x10
#define MOTOR_L_FORWARD 0x01

#define MOTOR_R_BACKWARD 0x01
#define MOTOR_L_BACKWARD 0x10*/
/******************************************************************
===========================Encoders===========================
*******************************************************************/
//Encoder setting
uint8_t _ENCODE_LA = 5;
uint8_t _ENCODE_RA = 3;
	
//Encoder reading
double prevState_L = 0;
double currState_L = 0;
double acccmCount_L = 0;

double prevState_R = 0;
double currState_R = 0;
double acccmCount_R = 0;

/******************************************************************
=================================Motors===========================
*******************************************************************/
//Left Motor
char MOTOR_L = 0;
uint8_t _PWM_MOTOR_L = 9;
uint8_t _INA1 = 2;
uint8_t _INB1 = 4;
	
//Right Motor
char MOTOR_R = 1;
uint8_t _PWM_MOTOR_R = 10;
uint8_t _INA2 = 7;
uint8_t _INB2 = 8;

/******************************************************************
=================================PID==============================
*******************************************************************/
//PID input using the encoder count
//PID output
double PIDL_output;
double PIDR_output;
double PIDL_setPoint;
double PIDR_setPoint;
PID PID_MotorL(&acccmCount_L, &PIDL_output, &PIDL_setPoint, 1, 0.5, 0.25, DIRECT);
PID PID_MotorR(&acccmCount_R, &PIDR_output, &PIDR_setPoint, 1, 0.5, 0.25, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Setup Started");

  pinMode(_ENCODE_LA,INPUT);
  pinMode(_ENCODE_RA,INPUT);
  PCintPort::attachInterrupt(_ENCODE_LA, &motorLISR, CHANGE);
  PCintPort::attachInterrupt(_ENCODE_RA, &motorRISR, CHANGE);
  
  motorInit();
  
  PID_MotorL.SetMode(AUTOMATIC);
  PID_MotorR.SetMode(AUTOMATIC);
  PID_MotorL.SetOutputLimits(-800, 800);
  PID_MotorR.SetOutputLimits(-800, 800);
  Serial.print("Setup Ended");
}

void loop() {
  //////////////////////////////Available Functions//////////////////////////////////
  //MoveForward(spd, dist) //distance does not works
  //Turn_R_90(spd)
  //Turn_L_90(spd)
  //Turn_180(spd)
  //Stop()
  //////////////////////////////////////////////////////////////////////////////////
  MoveForward(150,0);
    Serial.println("acccmCount_L");
  Serial.println(acccmCount_L);
  Serial.println("acccmCount_R");
  Serial.println(acccmCount_R);
  
  
  //TO DO:
  //1. test whether the interrupt of the encoders work
  //2. calibrate the PID, Ki, Kp, Kd, as well as the limit output
  
  
}

void motorInit()
{
	//right motor
	pinMode(_INA1,OUTPUT);
	pinMode(_INB1,OUTPUT);
	pinMode(_PWM_MOTOR_R,OUTPUT);
	pinMode(A0,INPUT);
	
	//pinMode(_EN1DIAG1,INPUT);
	
	//left motor
	pinMode(_INA2,OUTPUT);
	pinMode(_INB2,OUTPUT);
	pinMode(_PWM_MOTOR_L,OUTPUT);
	pinMode(A1,INPUT);
	
	//pinMode(_EN2DIAG2,INPUT);
	#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
	// Timer 1 configuration
	// prescaler: clockI/O / 1
	// outputs enabled
	// phase-correct PWM
	// top of 400
	//
	// PWM frequency calculation
	// 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
	TCCR1A = 0b10100000;
	TCCR1B = 0b00010001;
	ICR1 = 400;
	#endif
}
/******************************************************************
============================Motion Function=======================
*******************************************************************/
//Robot move forward, spd from 0 - 255, dist
//dist does not work yet
void MoveForward(uint8_t spd, uint8_t dist)
{
	analogWrite(_PWM_MOTOR_R, spd);
	analogWrite(_PWM_MOTOR_L, spd);

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
void  Turn_R_90(uint8_t spd)
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
void  Turn_L_90(uint8_t spd)
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
void  Turn_180(uint8_t spd)
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
void Stop()
{
	MotorBrakeLow(0);
	MotorBrakeLow(1);
}
/******************************************************************
==========================End Motion Function=====================
*******************************************************************/
void MotorRotateCW(uint8_t wheel, uint8_t spd, uint8_t dist)
{
	switch(wheel)
	{
	case 0:  //Left Wheel
		digitalWrite(_INA2, HIGH);
		digitalWrite(_INB2, LOW);
		break;
	case 1:  //Right Wheel
		digitalWrite(_INA1, HIGH);
		digitalWrite(_INB1, LOW);
		break;
	default:
		break;
	}
}
void MotorRotateCCW(uint8_t wheel, uint8_t spd, uint8_t dist)
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
void MotorBrakeLow(uint8_t wheel)
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
void motorLISR()
{
  currState_L = digitalRead(_ENCODE_LA);
  if((prevState_L  == LOW) && (currState_L == HIGH))
  {
    acccmCount_L++;
  }
  prevState_L = currState_L;
 
}
void motorRISR()
{
  currState_R = digitalRead(_ENCODE_RA);
  if((prevState_R  == LOW) && (currState_R == HIGH))
  {
    acccmCount_R++;
  }
  prevState_R = currState_R;
}

