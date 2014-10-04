#include <PID_v1.h>
#include <PinChangeInt.h>

#include "DualVNH5019MotorShield.h"
int mAspe = 9;
int mBspe = 10;
int AdirF = 2;
int AdirB = 4;
int BdirF = 8;
int BdirB = 7;
int encoderAPin = 3;
int encoderBPin = 5;
double encoder0Pos = 0;
double encoder0PosB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
int totalEncoderA = 0;
int totalEncoderB = 0;

double motor1_measure = 0;
double motor1_dist = 0;
double motor1_Setpoint = 136;  //temp use fake value

double motor2_measure = 0;
double motor2_dist = 0;
double motor2_Setpoint = 137; //temp use fake value

int Speed1 = 150;
int Speed2 = 154;

int timer=0; 

PID PID_Motor1(&motor1_measure, &motor1_dist, &motor1_Setpoint, 0.1, 0.32, 0.001, DIRECT);
PID PID_Motor2(&motor2_measure, &motor2_dist, &motor2_Setpoint, 0.1, 0.34, 0.001, DIRECT);

DualVNH5019MotorShield md; 

void setup()
{

  pinMode (encoderAPin,INPUT);
  pinMode (encoderBPin,INPUT);
  pinMode (mAspe,OUTPUT);
  pinMode (mBspe,OUTPUT);
  pinMode (AdirF,OUTPUT);
  pinMode (AdirB,OUTPUT);
  pinMode (BdirF,OUTPUT);
  pinMode (BdirB,OUTPUT);
  Serial.begin (9600);
  md.init();

  PID_Motor1.SetMode(AUTOMATIC);
  PID_Motor2.SetMode(AUTOMATIC);

  //  PID_Motor1.SetSampleTime(50);
  //  PID_Motor2.SetSampleTime(50);

  PID_Motor1.SetOutputLimits(-800, 800);
  PID_Motor2.SetOutputLimits(-800, 800);
  int timer=0;
 PCintPort::attachInterrupt(encoderAPin, motorAISR, RISING);
 PCintPort::attachInterrupt(encoderBPin, motorBISR, RISING);

}
void loop(){
  forward();

}
void forward(){
  motor1_Setpoint = 135;  //temp use fake value
  motor2_Setpoint = 135; //temp use fake value
  Speed1 = 160;
  Speed2 = 166;
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  //2600 110cm
  while(totalEncoderA<=3845){


    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);
    timer++;


    if(timer>=6800)
    {
      motor1_measure = encoder0Pos;
      motor2_measure = encoder0PosB;
      PID_Motor1.Compute();
      PID_Motor2.Compute();



//      Speed1 = ((motor1_dist / motor1_measure) +1) * Speed1;
//      Speed2 = ((motor2_dist / motor2_measure) +1) * Speed2;
      timer = 0;
      Serial.println("---------------------------");
      Serial.println("encoder A"); 
      Serial.println(encoder0Pos); 
      Serial.println("encoder B");
      Serial.println(encoder0PosB);  
      encoder0Pos = 0;
      encoder0PosB = 0;
      Serial.println("motor A:");
      Serial.println(motor1_dist);
      Serial.println(motor1_measure); 
      Serial.println("motor B:");
      Serial.println(motor2_dist);
      Serial.println(motor2_measure); 
      Serial.println("Speed1");
      Serial.println(Speed1);      
      Serial.println("Speed2");
      Serial.println(Speed2);

    }

  }
  Serial.println("totalA");
  Serial.println(totalEncoderA);      
  Serial.println("totalB");
  Serial.println(totalEncoderB);
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(3000);
}


void motorAISR()
{
    encoder0Pos++;
    totalEncoderA ++;
}
void motorBISR()
{
    encoder0PosB++;
    totalEncoderB ++;
}










