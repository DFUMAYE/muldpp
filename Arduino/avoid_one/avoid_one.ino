#include <PID_v1.h>

#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#define ir A2
#define model 1080
SharpIR sharp(ir,25,93,model);
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
double motor1_Setpoint = 136;  //temp use fake value

double motor2_measure = 0;
double motor2_dist = 0;
double motor2_Setpoint = 136; //temp use fake value

int Speed1 = 150;
int Speed2 = 154;

int timer=0; 

PID PID_Motor1(&motor1_measure, &motor1_dist, &motor1_Setpoint, 0.1, 0.06, 0.001, DIRECT);
PID PID_Motor2(&motor2_measure, &motor2_dist, &motor2_Setpoint, 0.1, 0.06, 0.001, DIRECT);

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
  pinMode(ir,INPUT);
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
  forward();
  if(check()){
    turnRight();
    forward();
    forward();
    turnLeft();
    forward();
    forward();
    forward();
    forward();
    turnLeft();
    forward();
    forward();
    turnRight();
  }
}
boolean check(){
  int dis = sharp.distance();
  Serial.print("Obstacle distance: ");
  Serial.println(dis);
  if(dis<12){
  return true;
  }
   return false;  
}
void turnRight(){
  motor1_Setpoint = 68; 
  motor2_Setpoint = 68; 
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  Speed1 = 100;
  Speed2 = 100;
  while(totalEncoderA<=331){


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
    
    motor1_measure = encoder0Pos;
    motor2_measure = encoder0PosB;
    if(timer ==4000)
    {
      PID_Motor1.Compute();
      PID_Motor2.Compute();
      
      Speed1 = ((motor1_dist / motor1_measure) +1) * Speed1;
      Speed2 = ((motor2_dist / motor2_measure) +1) * Speed2;
      timer = 0;
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
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
void forward(){
  int Speed1 = 150;
int Speed2 = 154;
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
//2600 110cm
  while(totalEncoderA<=230){


    digitalWrite(AdirF,HIGH);
    digitalWrite(AdirB,LOW);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
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
  
    motor1_measure = encoder0Pos;
    motor2_measure = encoder0PosB;
    if(timer ==6900)
    {
      PID_Motor1.Compute();
      PID_Motor2.Compute();



      Speed1 = ((motor1_dist / motor1_measure) +1) * Speed1;
      Speed2 = ((motor2_dist / motor2_measure) +1) * Speed2;
      timer = 0;
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
  delay(200);
}
void turnLeft(){
  motor1_Setpoint = 68; 
  motor2_Setpoint = 68; 
  encoder0Pos = 0;
  encoder0PosB = 0;
  totalEncoderA = 0;
  totalEncoderB = 0;
  timer=0;
  int Speed1 = 120;
  int Speed2 = 121;
  while(totalEncoderA<=330){


    digitalWrite(AdirF,LOW);
    digitalWrite(AdirB,HIGH);
    digitalWrite(BdirB,LOW);
    digitalWrite(BdirF,HIGH);
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

    motor1_measure = encoder0Pos;
    motor2_measure = encoder0PosB;
    if(timer ==4000)
    {
      PID_Motor1.Compute();
      PID_Motor2.Compute();



      Speed1 = ((motor1_dist / motor1_measure) +1) * Speed1;
      Speed2 = ((motor2_dist / motor2_measure) +1) * Speed2;
      timer = 0;
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
  digitalWrite(AdirF,HIGH);
  digitalWrite(AdirB,LOW);
  digitalWrite(BdirB,LOW);
  digitalWrite(BdirF,HIGH);
  analogWrite(mAspe,0);
  analogWrite(mBspe,0);
  delay(200);
}
  

