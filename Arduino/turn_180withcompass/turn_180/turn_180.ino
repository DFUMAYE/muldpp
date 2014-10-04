#include "DualVNH5019MotorShield.h"
///////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

int SPEED = 150;
int encoderAPin = 5;
int encoderBPin = 3;
int PID = SPEED-7;
int encoder0Pos = 0;
int encoder0PosB = 0;
int encoder0PinALast = 0;
int encoder0PinBLast = 0;
int totalEncoderA = 0;
int totalEncoderB = 0;



/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

DualVNH5019MotorShield md;
void setup()
{
  Serial.begin (9600);
  md.init();
  ///////////////
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); 
  Serial.println("");

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}
void loop(){
  ////////////
  sensors_event_t event; 
  mag.getEvent(&event);
  float starDir = atan2(event.magnetic.y, event.magnetic.x);
  float starDeclinationAngle = 0.22;
  starDir += starDeclinationAngle;
  if(starDir < 0)
    starDir += 2*PI;
  if(starDir > 2*PI)
    starDir -= 2*PI; 
  float starHeadingDegrees = starDir * 180/M_PI;
  //////////
  for(int i=0;i<60;i++){
    md.setM1Speed(PID);
    md.setM2Speed(SPEED);
    int encoder0Pos = 0;
    int encoder0PosB = 0;

    for(int j=0;j<40000;j++){
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
      ///////////////
      sensors_event_t event; 
       mag.getEvent(&event);
      float heading = atan2(event.magnetic.y, event.magnetic.x);
      float declinationAngle = 0.22;
      heading += declinationAngle;
      if(heading < 0)
        heading += 2*PI;
      if(heading > 2*PI)
        heading -= 2*PI;
      float headingDegrees = heading * 180/M_PI;
      Serial.print("start (degrees): "); Serial.println(starHeadingDegrees);
      Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
      ////////////////////////
      if(/*totalEncoderA>=750*/abs((headingDegrees-starDir)-90)<3){
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(3000);

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

    ////////////////////
    //806 for 180 degree , 3200 for 730 degree

    PID = (1+((float)encoder0PosB-(float)encoder0Pos)/(float)encoder0Pos)*PID;
    Serial.println(PID);
  }

  md.setM1Speed(0);
  md.setM2Speed(0);

  delay(1000); 

}
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" uT");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" uT");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

