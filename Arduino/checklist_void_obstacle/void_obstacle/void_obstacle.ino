#include "DualVNH5019MotorShield.h"
int encoderAPin = 5;
int encoderBPin = 3;
DualVNH5019MotorShield md;

void setup(){
  pinMode (encoderAPin,INPUT);
  pinMode (encoderBPin,INPUT);
  Serial.begin (9600);
  md.init();
}
void loop(){

  checkObstacle();
  forward10cm(;
}

void avoidObstacleM(){
  turnRight();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnRight(); 
  checkObstacle();
  
}
void avoidObstacleL(){
  turnRight();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnRight(); 
  checkObstacle();
  
}
void avoidObstacleR(){
  turnRight();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnLeft();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  forward10cm();
  checkObstacle();
  
  turnRight(); 
  checkObstacle();
  
}
void turnRight(){


}
void turnLeft(){

}
void forward10cm(){

}
void checkObstacle(){
  if(checkObstacleR())
    avoidObstacleR();
  else if(checkObstacleM())
    avoidObstacleM();
  else if(checkObstacleL())
    avoidObstackeL();  
}
boolean checkObstacleR(){

}
boolean checkObstacleM(){

}
boolean checkObstacleL(){

}

