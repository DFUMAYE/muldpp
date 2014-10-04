#include "DualVNH5019MotorShield.h"

int SPEED = 150;
int mAspe = 9;
int mBspe = 10;
int AdirF = 2;
int AdirB = 4;
int BdirF = 8;
int BdirB = 7;
int encoderAPin = 5;
int encoderBPin = 3;
int PPP = SPEED-7;
int encoder0Pos = 0;
int encoder0PosB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
int totalEncoderA = 0;
int totalEncoderB = 0;
////PPP
//double Setpoint,Input,Output;
//PPP myPPP(&Input,&Output,&Setpoint,2,5,1,DIRECT);

boolean done =false;
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
//  //PPP
//  Input = PPP;
//  Setpoint = 100;
//  myPPP.setMode(AUTOMATIC);
  
}
void loop(){

//2665 ticks 71cm
//  if(done=false){
    for(int i=0;i<30;i++){
//      digitalWrite(AdirF,HIGH);
//      digitalWrite(BdirF,HIGH);
//      analogWrite(mAspe,PPP);
//      analogWrite(mBspe,SPEED);
      md.setM1Speed(PPP);
      md.setM2Speed(-SPEED);
      int encoder0Pos = 0;
      int encoder0PosB = 0;
      for(int j=0;j<30000;j++){
        /////////////////for encoder
        int n = digitalRead(encoderAPin);
        int n2 = digitalRead(encoderBPin);
        if ((encoder0PinALast == LOW) && (n == HIGH)) {
          encoder0Pos++;
          totalEncoderA ++;
        } 
        if((encoder0PinBLast==LOW)&&(n2==HIGH)){
          encoder0PosB++; 
          totalEncoderB ++;
        }
        encoder0PinALast = n;
        encoder0PinBLast = n2;
        ////////////////////////
              if(totalEncoderA>=2000){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(1000);
//        done = true;

        i=61;
        break;
      }
      }
      Serial.print("motorA: ");
      Serial.println(encoder0Pos);
      Serial.print("MotorB: ");
      Serial.println(encoder0PosB);
      Serial.print("/");
      Serial.println(totalEncoderA);
      Serial.println(totalEncoderB);

      PPP = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*PPP;
      SPEED = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*SPEED;

//     Input = PPP;
      Serial.println(PPP);
      
    }
//  }
//  done = true;
/////////////////////////////////
  SPEED = 150;
  PPP = SPEED-7;
  totalEncoderA = 0;
  totalEncoderB = 0;
  
      for(int i=0;i<60;i++){
      md.setM1Speed(PPP);
      md.setM2Speed(SPEED);
      int encoder0Pos = 0;
      int encoder0PosB = 0;

      for(int j=0;j<30000;j++){
        /////////////////for encoder
        int n = digitalRead(encoderAPin);
        int n2 = digitalRead(encoderBPin);
        if ((encoder0PinALast == LOW) && (n == HIGH)) {
          encoder0Pos++;
          totalEncoderA ++;
        } 
        if((encoder0PinBLast==LOW)&&(n2==HIGH)){
          encoder0PosB++; 
          totalEncoderB ++;
        }
        encoder0PinALast = n;
        encoder0PinBLast = n2;
        ////////////////////////
         ////////////////////
      //806 for 180 degree , 3200 for 730 degree
        if(totalEncoderA>=735){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(1000);
        done = true;

        i=61;
        break;
      }
      }
      Serial.print("motorA: ");
      Serial.println(encoder0Pos);
      Serial.print("MotorB: ");
      Serial.println(encoder0PosB);
      Serial.print("/");
      Serial.print("encoderA: ");
      Serial.println(totalEncoderA);
      Serial.print("encoderB: ");
      Serial.println(totalEncoderB);
      Serial.print("/");

     

      
      PPP = (1+((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)*PPP;
      SPEED = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*SPEED;
      Serial.println(PPP);
    }
    ////////////////////////////
      SPEED = 150;
  PPP = SPEED-7;
  totalEncoderA = 0;
  totalEncoderB = 0;
        for(int i=0;i<30;i++){
//      digitalWrite(AdirF,HIGH);
//      digitalWrite(BdirF,HIGH);
//      analogWrite(mAspe,PPP);
//      analogWrite(mBspe,SPEED);
      md.setM1Speed(PPP);
      md.setM2Speed(-SPEED);
      int encoder0Pos = 0;
      int encoder0PosB = 0;
      for(int j=0;j<30000;j++){
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
        ////////////////////////
        if(totalEncoderA>=2000){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(5000);
//        done = true;

        i=61;
        break;
      }
      }
      Serial.print("motorA: ");
      Serial.println(encoder0Pos);
      Serial.print("MotorB: ");
      Serial.println(encoder0PosB);
      Serial.print("/");
      Serial.println(totalEncoderA);
      Serial.println(totalEncoderB);
      PPP = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*PPP;
      SPEED = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*SPEED;

//     Input = PPP;
      Serial.println(PPP);
      
    }
    ////////////////////
    
    
  md.setM1Speed(0);
  md.setM2Speed(0);

  delay(1000); 
}

