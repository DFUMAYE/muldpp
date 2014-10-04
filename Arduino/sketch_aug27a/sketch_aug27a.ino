#include "DualVNH5019MotorShield.h"
#define SPEED 100;
int mAspeedPin = 9;
int mBspeedPin = 10;
int mAdirecPinForw = 2;
int mAdirecPinBackw = 4;
int mBdirecPinForw = 8;
int mBdirecPinBackw = 7;
int encoderAPin = 3;
int encoderBPin = 5;
int ultrasonic1 = 13; 
int PID = SPEED;
DualVNH5019MotorShield md; 
void setup()
{

   pinMode (encoderAPin,INPUT);
   pinMode (encoderBPin,INPUT);
   pinMode (mAspeedAPin,OUTPUT);
   pinMode (mBspeedBPin,OUTPUT);
   Serial.begin (9600);
   md.init();
}


void loop()
{
//ultrasonic sensor testing
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(ultrasonic1, OUTPUT);
  digitalWrite(ultrasonic1, LOW);
  delay(2);
  digitalWrite(ultrasonic1, HIGH);
  delay(5);
  digitalWrite(ultrasonic1, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ultrasonic1, INPUT);
  duration = pulseIn(ultrasonic1, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(100);

//infrared sensor testing

int IRsensor3Value = analogRead(A3);
Serail.println(sensorValue);
delay(1);



   
//forward


  
  
//  //backward
//int PID = 1;
//for(int i=0;i<200;i++){
//  
//   digitalWrite(mAdirecPinForw,HIGH);
//   digitalWrite(mBdirecPinForw,HIGH);
//   digitalWrite(mAspeedPin,SPEED*PID);
//   digitalWrite(mBspeedPin,SPEED);
//   
//  int encoder0Pos = 0;
//  int encoder0PosB = 0;
//  int encoder0PinALast = 0;
//  int encoder0PinBLast = 0;
//   for(int j=0;j<200;j++){
//  /////////////////for encoder
//  int n = digitalRead(encoderAPin);
//  int n2 = digitalRead(encoderBPin);
//   if ((encoder0PinALast == LOW) && (n == HIGH)) {
//       encoder0Pos++;
//       Serial.print (encoder0Pos);
//       Serial.print ("/");
//   } 
//   if((encoder0PinBLast==LOW)&&(n2==HIGH)){
//       encoder0PosB++; 
//   }
//   encoder0PinALast = n;
//   encoder0PinBLast = n2;
//   ////////////////////////
//   }
//   PID = 1+(encoder0PosB-encoder0Pos)/encoderPos;
//  }

////turn____
//int PID = 1;
//for(int i=0;i<200;i++){
//  
//   digitalWrite(mAdirecPinBackw,1);
//   digitalWrite(mBdirecPinForw,1);
//   digitalWrite(mAspeedPin,SPEED*PID);
//   digitalWrite(mBspeedPin,SPEED);
//   
//  int encoder0Pos = 0;
//  int encoder0PosB = 0;
//  int encoder0PinALast = 0;
//  int encoder0PinBLast = 0;
//   for(int j=0;j<200;j++){
//  /////////////////for encoder
//  int n = digitalRead(encoderAPin);
//  int n2 = digitalRead(encoderBPin);
//   if ((encoder0PinALast == LOW) && (n == HIGH)) {
//       encoder0Pos++;
//       Serial.print (encoder0Pos);
//       Serial.print ("/");
//   } 
//   if((encoder0PinBLast==LOW)&&(n2==HIGH)){
//       encoder0PosB++; 
//   }
//   encoder0PinALast = n;
//   encoder0PinBLast = n2;
//   ////////////////////////
//   }
//   PID = 1+(encoder0PosB-encoder0Pos)/encoderPos;
//  }


////turn___
//int PID = 1;
//for(int i=0;i<200;i++){
//  
//   digitalWrite(mAdirecPinForw,1);
//   digitalWrite(mBdirecPinBackw,1);
//   digitalWrite(mAspeedPin,SPEED*PID);
//   digitalWrite(mBspeedPin,SPEED);
//   
//  int encoder0Pos = 0;
//  int encoder0PosB = 0;
//  int encoder0PinALast = 0;
//  int encoder0PinBLast = 0;
//   for(int j=0;j<200;j++){
//  /////////////////for encoder
//  int n = digitalRead(encoderAPin);
//  int n2 = digitalRead(encoderBPin);
//   if ((encoder0PinALast == LOW) && (n == HIGH)) {
//       encoder0Pos++;
//       Serial.print (encoder0Pos);
//       Serial.print ("/");
//   } 
//   if((encoder0PinBLast==LOW)&&(n2==HIGH)){
//       encoder0PosB++; 
//   }
//   encoder0PinALast = n;
//   encoder0PinBLast = n2;
//   ////////////////////////
//   }
//   PID = 1+(encoder0PosB-encoder0Pos)/encoderPos;
//  }
}
////////////////////for ultralsonic sensor
long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
void forward()
{
  for(int i=0;i<200;i++){
  
  md.setM1Speed(100);
  md.setM2Speed(100);
   
  int encoder0Pos = 0;
  int encoder0PosB = 0;
  int encoder0PinALast = 0;
  int encoder0PinBLast = 0;
   for(int j=0;j<200;j++){
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
       Serial.println (encoder0Pos);
       Serial.println (encoder0PosB);
       Serial.print ("/");
   PID = (1+(encoder0PosB-encoder0Pos)/encoderPos)*PID;
  }
}
//void backward()
//{
//   digitalWrite(mAdirecPinBackw,1);
//   digitalWrite(mBdirecPinBackw,1);
//   digitalWrite(mAspeedPin,100);
//   digitalWrite(mBspeedPin,100);
//   delay(2);
//}
//void turnRight90deg(){
//  
//}
//void turnRight90deg(){
//  
//}

