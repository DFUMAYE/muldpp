#include "DualVNH5019MotorShield.h"
int mAspeedPin = 9;
int mBspeedPin = 10;
int mAdirecPinForw = 2;
int mAdirecPinBackw = 4;
int mBdirecPinForw = 8;
int mBdirecPinBackw = 7;
int encoderAPin = 3;
int encoderBPin = 5;
int totalEncoderA =0;
int totalEncoderB =0;
DualVNH5019MotorShield md; 
void setup()
{
   pinMode (encoderAPin,INPUT);
   pinMode (encoderBPin,INPUT);
   Serial.begin (9600);
   md.init();
  
}
void loop(){
   int encoder0Pos = 0;
   int encoder0PosB = 0;
   int encoder0PinALast = 0;
   int encoder0PinBLast = 0;
   for(int j=0;j<200;j++){
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
   }
   totalEncoderA += encoder0Pos;
   totalEncoderB += encoder0PosB;
   
       Serial.print ("number of encoderA: ");
       Serial.println(totalEncoderA);
       Serial.print ("number of encoderB: ");
       Serial.println(totalEncoderB);
}
