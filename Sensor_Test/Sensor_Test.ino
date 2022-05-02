#include <SharpDistSensor.h>

int sensor1Pin = A0;
int sensor2Pin = A1;

SharpDistSensor IRsensor1(sensor1Pin, 5);
SharpDistSensor IRsensor2(sensor2Pin, 5);


double returnPos() {
  double pos = 0.0;
  //platform length
  int len = 320;
  int distance1 = IRsensor1.getDist();
  int distance2 = IRsensor2.getDist();

//  if (distance1 < distance2) {
//    if ((distance2 - distance1) < 10) {
//      pos = 0.0;
//    }
//    else {
//      pos = (double)(-1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
//    }
//  }
//  else if (distance1 > distance2) {
//    if ((distance1 - distance2) < 30) {
//      pos = 0.0;
//    }
//    else {
//      pos = (double)(1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
//    }
//  }
//  else {
//    pos = 0.0;
//  }
  
  for(int i=1;i<=10;i++){
    pos = (pos*i+(distance1-distance2))/(i+1);
    delay(5);
  }
  
  
  return (pos+10)/10;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  IRsensor1.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
  IRsensor2.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
}

void loop() {
  // put your main code here, to run repeatedly:
  double dist = (returnPos());
  Serial.println(dist);
}
