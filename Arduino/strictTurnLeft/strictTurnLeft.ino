#include <PID_v1.h>

#include "DualVNH5019MotorShield.h"
int mAspe = 9;
int mBspe = 10;
int AdirF = 2;
int AdirB = 4;
int BdirF = 8;
int BdirB = 7;
int encoderAPin = 3;
int encoderBPin = 5;
int encoder0Pos = 0;
int encoder0PosB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
int totalEncoderA = 0;
int totalEncoderB = 0;

double motor1_measure = 0;
double motor1_dist = 0;
double motor1_Setpoint = 65;  //temp use fake value

double motor2_measure = 0;
double motor2_dist = 0;
double motor2_Setpoint = 65; //temp use fake value

int Speed1 = 100;
int Speed2 = 106;

int timer=0; 

PID PID_Motor1(&motor1_measure, &motor1_dist, &motor1_Setpoint, 0.1, 0.2, 0.001, DIRECT);
PID PID_Motor2(&motor2_measure, &motor2_dist, &motor2_Setpoint, 0.1, 0.2, 0.001, DIRECT);

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

}
void loop(){
  turnLeft();
}
void turnLeft(){
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1= 120;
  Speed2= 124;
  while(totalEncoderA<=389){


    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,HIGH);
    digitalWrite(BdirF,LOW);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);

    timer++;

    /////////////////for encoder
    int n = digitalRead(encoderAPin);
    int n2 = digitalRead(encoderBPin);
    if ((encoder0PinALast == LOW) && (n == HIGH)) {
      encoder0Pos++;
      totalEncoderA ++;
    } 
    if((encoder0PinBLast==LOW)&&(n2==HIGH)){
      encoder0PosB++; 
      totalEncoderB++;
    }
    encoder0PinALast = n;
    encoder0PinBLast = n2;  
    if(timer ==5400)
    {

      Speed1 = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*Speed1;
      Speed2 = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*Speed2;

      //      Serial.println("encoder A"); 
      //      Serial.println(encoder0Pos); 
      //      Serial.println("encoder B");
      //      Serial.println(encoder0PosB);  
      //      Serial.println("motor A:");
      //      Serial.println(motor1_dist);
      //      Serial.println(motor1_measure); 
      //      Serial.println("motor B:");
      //      Serial.println(motor2_dist);
      //      Serial.println(motor2_measure); 
      //      Serial.println("Speed1");
      //      Serial.println(Speed1);      
      //      Serial.println("Speed2");
      //      Serial.println(Speed2);

      timer = 0;
      encoder0Pos = 0;
      encoder0PosB = 0;

    }
  }
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,HIGH);
  digitalWrite(BdirF,LOW);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
