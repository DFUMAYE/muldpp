#include <PID_v1.h>

#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#define irL A0
#define irR A1
#define model 1080
SharpIR sharpL(irL,25,93,model);
SharpIR sharpR(irR,25,93,model);

int mAspe = 9;
int mBspe = 10;
int AdirF = 2;
int AdirB = 4;
int BdirF = 8;
int BdirB = 7;
DualVNH5019MotorShield md; 
void setup(){
  Serial.begin(9600);
  pinMode(irL,INPUT);
  pinMode(irR,INPUT);  
}
void loop(){
  delay(300);
  int disL = sharpL.distance();
  Serial.print("L Obstacle distance: ");
  Serial.println(disL);
  int rawL = analogRead(irL);
  Serial.println(rawL);
  int disR = sharpR.distance();
  Serial.print("R Obstacle distance: ");
  Serial.println(disR);
  int rawR = analogRead(irR);
  Serial.println(rawR);
  while(abs(disL-disR)>7&&abs(disL-disR)<110)
  {
    disL = sharpL.distance();
    disR = sharpR.distance();
    if(disL-disR>7){
        
    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,HIGH);
    digitalWrite(BdirF,LOW);
    analogWrite(mAspe,250);
    analogWrite(mBspe,250);
    delay(6);
    
    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,HIGH);
    digitalWrite(BdirF,LOW);
    analogWrite(mAspe,0);
    analogWrite(mBspe,0);
    }
    else{
     
    digitalWrite(AdirF,LOW);
    digitalWrite(AdirB,HIGH);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
    analogWrite(mAspe,250);
    analogWrite(mBspe,250); 
    delay(6);
    
    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,HIGH);
    digitalWrite(BdirF,LOW);
    analogWrite(mAspe,0);
    analogWrite(mBspe,0);
    }
      
  }
  
  
}
