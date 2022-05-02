/*
  Test.h - Test library for Wiring - implementation
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// include core Wiring API
#include "Arduino.h"

// include this library's description file
#include "Platform.h"

// include description files for other libraries used (if any)
#include <Servo.h>
//#include <SharpDistSensor.h>

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

Platform::Platform(void)
{
    a = 0.000003;
    b = 0.0002;
    c = -0.0028;
    d = 0.7867;
    e = 45.028;
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void Platform::_init(Servo servo0, Servo servo1, int sensor1Pin, int sensor2Pin, int angle) {
    
    
    
//    Serial.print("a "); Serial.println(a, 6);
//    Serial.print("b "); Serial.println(b, 6);
//    Serial.print("c "); Serial.println(c, 6);
//    Serial.print("d "); Serial.println(d, 6);
//    Serial.print("e "); Serial.println(e, 6);
//    Serial.println(float(a*pow(0,4)), 6);
//    Serial.println(float(b*pow(0,3)), 6);
//    Serial.println(float(c*pow(0,2)), 6);
//    Serial.println(float(d*pow(0,1)), 6);
//    Serial.println(float(e), 6);
//    Serial.println(float(a*pow(0,4))+float(b*pow(0,3))+float(c*pow(0,2))+float(d*pow(0,1))+float(e)), 6;
    Servo0 = servo0;
    Servo1 = servo1;
    setAngle(angle);
    
//    IRsensor1 = SharpDistSensor(sensor1Pin, 5);
//    IRsensor1 = SharpDistSensor(sensor2Pin, 5);
}

void Platform::setAngle(int angle)
{
  // get angles to write to servos
//    Serial.print("poly0 "); Serial.println(_polyModel(angle), 6);
//    Serial.print("poly1 "); Serial.println(_polyModel(-angle), 6);
    SERVO0_ANGLE = _polyModel(angle);
    SERVO1_ANGLE = _polyModel(-angle);
    
    Servo0.writeMicroseconds(_degToUSec(SERVO0_ANGLE) + SERVO_OFFSET_MICROS);
//    Serial.print("Servo0 ");Serial.println(_degToUSec(SERVO0_ANGLE));
//    Serial.print("Servo1 ");Serial.println(_degToUSec(SERVO1_ANGLE));
    Servo1.writeMicroseconds(_degToUSec(180 - SERVO1_ANGLE));
    
//    int k0 = 1; int k1 = 1;
//    if(SERVO0_ANGLE >= servo0Target) { k0 = -1; }
//    if(SERVO1_ANGLE >= servo1Target) { k1 = -1; }
//    while((SERVO0_ANGLE != servo0Target) && (SERVO1_ANGLE != servo1Target)) {
//        if(SERVO0_ANGLE != servo0Target) { SERVO0_ANGLE; }
//    }
//
//    for (int i = SERVO0_ANGLE; i > 0; i--) {
//      Servo0.writeMicroseconds(servoAngle1);
//      Servo1.writeMicroseconds(servoAngle2);
//      delay(stepDelay);
//    }
}

float Platform::_polyModel(int i) {
//    Serial.print("a "); Serial.println(a, 6);
//    Serial.print("b "); Serial.println(b, 6);
//    Serial.print("c "); Serial.println(c, 6);
//    Serial.print("d "); Serial.println(d, 6);
//    Serial.print("e "); Serial.println(e, 6);
//    Serial.println(float(a*pow(0,4)), 6);
//    Serial.println(float(b*pow(0,3)), 6);
//    Serial.println(float(c*pow(0,2)), 6);
//    Serial.println(float(d*pow(0,1)), 6);
//    Serial.println(float(e), 6);
    
 return float(float(a*pow(i,4)) + float(b*pow(i,3)) + float(c*pow(i,2)) + float(d*i) + float(e));
}

int Platform::_degToUSec(float deg) {
  return int(float((deg / 180) * 2000) + 500);
}

//double Platform::getBallPos(void) {
//  double pos = 0.0;
//  //platform length
//  int len = 320;
//  int distance1 = IRsensor1.getDist();
//  int distance2 = IRsensor2.getDist();
//
//  if (distance1 < distance2 && (distance2 - distance1) > 10) {
//      pos = (double)(-1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
//  }
//  else if (distance1 > distance2 && (distance1 - distance2) > 10) {
//        pos = (double)(1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
//  }
//  return pos;
//}
