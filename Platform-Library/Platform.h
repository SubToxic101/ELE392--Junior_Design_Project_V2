/*
  Test.h - Test library for Wiring - description
  Copyright (c) 2006 John Doe.  All right reserved.
*/

// ensure this library description is only included once
#ifndef Test_h
#define Test_h

// include types & constants of Wiring core API
#include "Arduino.h"
#include <Servo.h>
#include <SharpDistSensor.h>

// library interface description
class Platform
{
  // user-accessible "public" interface
  public:
    int SERVO_OFFSET_MICROS = 77;
    int SERVO0_ANGLE;
    int SERVO1_ANGLE;
    Servo Servo0;
    Servo Servo1;
//    SharpDistSensor IRsensor1;
//    SharpDistSensor IRsensor2;
    Platform(void);
    void _init(Servo, Servo, int, int, int);
    void setAngle(int);
//    double getBallPos(void);

  // library-accessible "private" interface
  private:
    float a;
    float b;
    float c;
    float d;
    float e;
    float _polyModel(int);
    int _degToUSec(float);
};

#endif

