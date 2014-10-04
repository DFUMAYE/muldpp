#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"
#include <PinChangeInt.h>
#include <SharpIR.h>
#define irM A2
#define irL A3
#define irR A4
#define irShort A0
#define irShort1 A1
#define irLong A5
#define model 1080
#define model2 20150
SharpIR sharpM(irM,25,93,model);
SharpIR sharpL(irL,25,93,model);
SharpIR sharpR(irR,25,93,model);
SharpIR sharpShort(irShort,25,93,model);
SharpIR sharpShort1(irShort1,25,93,model);
SharpIR sharpLong(irLong,25,93,model2);
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

int start =0;

int incomingByte = 0;   // for incoming serial data
int incomingByte2 = 0;   // for incoming serial data
int incomingByte3 = 0;   // for incoming serial data

int irRreading = 0;
int irMreading = 0;
int irLreading = 0;
int irShortreading = 0;
int irLongreading = 0;
int returnReading = 0;

DualVNH5019MotorShield md; 
PID PID_Motor1(&motor1_measure, &motor1_dist, &motor1_Setpoint, 0.1, 0.32, 0.001, DIRECT);
PID PID_Motor2(&motor2_measure, &motor2_dist, &motor2_Setpoint, 0.1, 0.34, 0.001, DIRECT);

void setup() {
  pinMode (encoderAPin,INPUT);
  pinMode (encoderBPin,INPUT);
  pinMode (mAspe,OUTPUT);
  pinMode (mBspe,OUTPUT);
  pinMode (AdirF,OUTPUT);
  pinMode (AdirB,OUTPUT);
  pinMode (BdirF,OUTPUT);
  pinMode (BdirB,OUTPUT);
  pinMode(irM,INPUT);
  pinMode(irL,INPUT);
  pinMode(irR,INPUT);
  pinMode(irLong,INPUT);
  pinMode(irShort,INPUT);
  pinMode(irShort1,INPUT);
  md.init();

  PID_Motor1.SetMode(AUTOMATIC);
  PID_Motor2.SetMode(AUTOMATIC);
  PID_Motor1.SetOutputLimits(-800, 800);
  PID_Motor2.SetOutputLimits(-800, 800);
   PCintPort::attachInterrupt(encoderAPin, motorAISR, RISING);
 PCintPort::attachInterrupt(encoderBPin, motorBISR, RISING);
  Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps

}

void loop() {


  cli();

  // send data only when you receive data:
  if (Serial.available() >1) {
    // read the incoming byte:
    
    incomingByte = Serial.read();
    incomingByte2 = Serial.read();
    Serial.flush();
 //  incomingByte3 = Serial.read();   

    // say what you got:
//    Serial.print("I received: ");
//    Serial.println(incomingByte, DEC);
  }
//      if(start==0){         // send sesor reading at the start
//        delay(200);
//        // returnSensorReading();
//        start=1;
//      }
//      turnLeft90();
//      delay(2000);
  sei();
  switch(incomingByte){
  case 80: 
    returnSensorReading();
    break;
  case 24: 
    turnLeft90();
    returnSensorReading();
    break;
  case 20: 
    turnRight90();
    returnSensorReading();
    break;
  case 16: 
    turn180();
    returnSensorReading();
    break;
  case 28: 
    forwardOneBlock();
    returnSensorReading();
    break; 
  case 44: int numberOfBlock;
    fastForward(numberOfBlock);
    returnSensorReading();
    break;
  case 48: 
    autoAdjust();
    returnSensorReading();
    break;
    
//  case 100 ://need to be change
    
  case 0:
    break;
  default:
    Serial.println(-1);

  }
  cli();
incomingByte = 0;   // for incoming serial data
incomingByte2 = 0;   // for incoming serial data  
}
void turnRight90(){
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1= 120;
  Speed2= 124;
  while(totalEncoderA<=387){


    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,HIGH);
    digitalWrite(BdirF,LOW);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);

    timer++;
  
    if(timer ==5400)
    {

      Speed1 = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*Speed1;
      Speed2 = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*Speed2;

//            Serial.println("encoder A"); 
//            Serial.println(encoder0Pos); 
//            Serial.println("encoder B");
//            Serial.println(encoder0PosB);  
//            Serial.println("motor A:");
//            Serial.println(motor1_dist);
//            Serial.println(motor1_measure); 
//            Serial.println("motor B:");
//            Serial.println(motor2_dist);
//            Serial.println(motor2_measure); 
//            Serial.println("Speed1");
//            Serial.println(Speed1);      
//            Serial.println("Speed2");
//            Serial.println(Speed2);

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
void turnLeft90(){
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1= 120;
  Speed2= 124;
  while(totalEncoderA<=387){


    digitalWrite(AdirF,LOW);
    digitalWrite(AdirB,HIGH);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);

    timer++;

    if(timer ==5400)
    {

      Speed1 = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*Speed1;
      Speed2 = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*Speed2;

//            Serial.println("encoder A"); 
//            Serial.println(encoder0Pos); 
//            Serial.println("encoder B");
//            Serial.println(encoder0PosB);  
//            Serial.println("motor A:");
//            Serial.println(motor1_dist);
//            Serial.println(motor1_measure); 
//            Serial.println("motor B:");
//            Serial.println(motor2_dist);
//            Serial.println(motor2_measure); 
//            Serial.println("Speed1");
//            Serial.println(Speed1);      
//            Serial.println("Speed2");
//            Serial.println(Speed2);

      timer = 0;
      encoder0Pos = 0;
      encoder0PosB = 0;

    }
  }
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
void turn180(){
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1= 120;
  Speed2= 124;
  while(totalEncoderA<=794){


    digitalWrite(AdirF,LOW);
    digitalWrite(AdirB,HIGH);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);

    timer++;
     
    if(timer ==5400)
    {

      Speed1 = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*Speed1;
      Speed2 = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*Speed2;

      timer = 0;
      encoder0Pos = 0;
      encoder0PosB = 0;

    }
  }
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
void forwardOneBlock(){
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1= 100;
  Speed2= 101;
  motor1_Setpoint = 65;
  motor2_Setpoint = 65;
  while(totalEncoderA<=237){


    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
    analogWrite(mAspe,Speed1);
    analogWrite(mBspe,Speed2);

    timer++;
    
    if(timer ==5400)
    {

      Speed1 = (1+(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)/2)*Speed1;
      Speed2 = (1-(((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0PosB)/2)*Speed2;
      timer = 0;
      encoder0Pos = 0;
      encoder0PosB = 0;
    }
  }
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
void fastForward(int numOfBlock){
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
  while(totalEncoderA<=numOfBlock*236.5){


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



      Speed1 = ((motor1_dist / motor1_measure) +1) * Speed1;
      Speed2 = ((motor2_dist / motor2_measure) +1) * Speed2;
      timer = 0;
      encoder0Pos = 0;
      encoder0PosB = 0;
    }

  }
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(3000);
}
int checkIrM(){
  int sensorTimer = millis();
  int disM = 0;
  if(millis() - sensorTimer >= 20){ 
      disM = sharpM.distance();
  }
  if(disM>50&&disM<120){
  return 8 ;}
  else{
  return 0;}
}
int checkIrL(){
  int sensorTimer = millis(); 
  int disL=0;
  if(millis() - sensorTimer >= 20){ 
      disL = sharpL.distance();
  }
    if(disL>50&&disL<120){
  return 4;}
  else{
  return 0;}
}
int checkIrR(){
  int sensorTimer = millis();
  int disR =0;
  if(millis() - sensorTimer >= 20){ 
      disR = sharpR.distance();
  }
      if(disR>50&&disR<120){
  return 16;}
  else{
  return 0;}
}
int checkIrLong(){
  // 1 240
  //2  320
  //3 430
  int sensorTimer = millis();
  int disLong =0;
  if(millis() - sensorTimer >= 20){ 
      disLong = sharpLong.distance();
  }
      if(disLong>50 && disLong<280){
  return 1;}
  else if(disLong>=280 && disLong<375){
    return 2;
  }  
  else if(disLong>=375&&disLong<470){
  return 3;}
  else{
  return 0;
  }
}
int checkIrShort(){
  int sensorTimer = millis();
  int disShort=0;
  if(millis() - sensorTimer >= 20){ 
      disShort = sharpShort.distance();
  }
     if(disShort>50&&disShort<180){
  return 32;}
  else{
  return 0;}
}

int returnSensorReading(){
  
  irRreading = checkIrR();
  irLreading = checkIrL();
  irMreading = checkIrM();
  irShortreading = checkIrShort();
  irLongreading = checkIrLong();
  returnReading = (irRreading|irLreading|irMreading|irShortreading|irLongreading);
  Serial.println(returnReading);
  return(returnReading);
  
}

void autoAdjust(){
  delay(300);
  int disS = sharpShort.distance();
//  Serial.print("L Obstacle distance: ");
//  Serial.println(disS);
  int disS1 = sharpShort1.distance();
//  Serial.print("R Obstacle distance: ");
//  Serial.println(disS1);
  while(abs(disS-disS1)>8&&abs(disS-disS1)<90)
  {
    disS = sharpL.distance();
    disS1 = sharpR.distance();
    if(disS-disS1>8){
        
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


