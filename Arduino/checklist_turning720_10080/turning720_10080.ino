#include "DualVNH5019MotorShield.h"
int SPEED = 150;
int encoderAPin = 5;
int encoderBPin = 3;
int PID = SPEED-7;
int totalEncoderA = 0;
int totalEncoderB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
DualVNH5019MotorShield md; 
void setup(){
  pinMode (encoderAPin,INPUT);
  pinMode (encoderBPin,INPUT);
  Serial.begin(9600);
  md.init();
}
boolean done = false;
void loop(){
  if(done==false){

    //   digitalWrite(mAdirecPinForw,HIGH);
    //   digitalWrite(mBdirecPinForw,HIGH);
    //   digitalWrite(mAspeedPin,10);
    //   digitalWrite(mBspeedPin,10);

    // i=50 turns 720+~110
    for(int i=0;i<60;i++){
      md.setM1Speed(PID);
      md.setM2Speed(SPEED);
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
      totalEncoderA += encoder0Pos;
      totalEncoderB += encoder0Pos;
      Serial.print("encoderA: ");
      Serial.println(totalEncoderA);
      Serial.print("encoderB: ");
      Serial.println(totalEncoderB);
      Serial.print("/");

      ////////////////////
      //806 for 180 degree , 3200 for 730 degree

      if(totalEncoderA>=3158){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(3000);
        done = true;

        i=61;
        break;
      }
      PID = (1+((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)*PID;
      Serial.println(PID);
    }
  }
  
  SPEED = 150;
  PID = SPEED-7;
  totalEncoderA = 0;
  totalEncoderB = 0;
  
      for(int i=0;i<60;i++){
      md.setM1Speed(-PID);
      md.setM2Speed(-SPEED);
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
      totalEncoderA += encoder0Pos;
      totalEncoderB += encoder0Pos;
      Serial.print("encoderA: ");
      Serial.println(totalEncoderA);
      Serial.print("encoderB: ");
      Serial.println(totalEncoderB);
      Serial.print("/");

      ////////////////////
      //806 for 180 degree , 3200 for 730 degree

      if(totalEncoderA>=3158){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(5000);
        done = true;

        i=61;
        break;
      }
      PID = (1+((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)*PID;
      Serial.println(PID);
    }
  
  md.setM1Speed(0);
  md.setM2Speed(0);

  delay(9000);
}

