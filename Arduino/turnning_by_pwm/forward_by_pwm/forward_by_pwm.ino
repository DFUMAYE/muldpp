#include "DualVNH5019MotorShield.h"
int SPEED = 80;
int mAspe = 9;
int mBspe = 10;
int AdirF = 2;
int AdirB = 4;
int BdirF = 8;
int BdirB = 7;
int encoderAPin = 5;
int encoderBPin = 3;
int PID = SPEED;
int encoder0Pos = 0;
int encoder0PosB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
int totalEncoderA = 0;
int totalEncoderB = 0;
boolean done = false;
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
  Serial.println("Program starts");

}

void loop(){
  if(done == false){
    for(int i=0;i<60;i++){
      turnRight(PID,SPEED);
      int encoder0Pos = 0;
      int encoder0PosB = 0;

      for(int j=0;j<30000;j++){
        /////////////////for encoder
        int n = digitalRead(encoderAPin);
        int n2 = digitalRead(encoderBPin);
        if ((encoder0PinALast == LOW) && (n == HIGH)) {
          encoder0Pos++;
        } 
        if((encoder0PinBLast==LOW)&&(n2==HIGH)){
          encoder0PosB++; 
        }
        encoder0PinALast = n;
        encoder0PinBLast = n2;
        ////////////////////////
      }
      Serial.print("motorA: ");
      Serial.println(encoder0Pos);
      Serial.print("MotorB: ");
      Serial.println(encoder0PosB);
      Serial.print("/");
      totalEncoderA+=encoder0Pos;
      totalEncoderB+=encoder0PosB;
      Serial.println(totalEncoderA);
      Serial.println(totalEncoderB);
      if(totalEncoderA>=3157){
        stopR();
        delay(5000);
        done = true;
        i=61;
        break;
      }
      PID = (1+((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)*PID;
      Serial.println(PID);
    }
  }
  stopR();
  delay(3000);
}
void turnRight(int a,int b){
  digitalWrite(AdirF,HIGH);
  digitalWrite(BdirB,HIGH);
  analogWrite(mAspe,a);
  analogWrite(mBspe,b);
}
void stopR(){
  digitalWrite(AdirF,LOW);
  digitalWrite(BdirB,LOW);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
}
void turnLeft(int a,int b){
  digitalWrite(AdirB,HIGH);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,a);
  analogWrite(mBspe,b);
}
void forw(int a,int b){
  digitalWrite(AdirF,HIGH);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,a);
  analogWrite(mBspe,b);
}
void backw(int a,int b){
  digitalWrite(AdirB,HIGH);
  digitalWrite(BdirB,HIGH);
  analogWrite(mAspe,a);
  analogWrite(mBspe,b);
}




