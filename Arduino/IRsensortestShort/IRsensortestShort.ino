
#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#define ir A2
#define model 1080
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y

SharpIR sharp(ir,25,93,model);

void setup(){
  Serial.begin(9600);
  pinMode(ir,INPUT);
  
}

void loop(){
  delay(1000);
  int dis = sharp.distance();
  Serial.print("Obstacle distance: ");
  Serial.println(dis);
}
