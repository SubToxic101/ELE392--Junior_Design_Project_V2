#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <SharpDistSensor.h>
#include <Servo.h>
#include <Platform.h>

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)  unused
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)   unused
#define USMIN  500 // This is the rounded 'minimum' microsecond length
#define USMAX  2500 // This is the rounded 'maximum' microsecond length
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

double posInput = 0;
double newPlatformAngle = 0;
double setPoint = 0;
double sensor1Pin = A1;
double sensor2Pin = A0;

//""""Working""" Pids
double Kp1 = 0.07;
double Ki1 = 0.09;
double Kd1 = 0.02;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Platform platform = Platform();
Servo Servo0;
Servo Servo1;

SharpDistSensor IRsensor1(sensor1Pin, 5);
SharpDistSensor IRsensor2(sensor2Pin, 5);
PID pid(&posInput, &newPlatformAngle, &setPoint, Kp1, Ki1, Kd1, DIRECT);

double returnPos() {
  double pos = 0.0;
  //platform length
  int len = 320;
  int distance1 = IRsensor1.getDist();
  int distance2 = IRsensor2.getDist();

  if (distance1 < distance2) {
    if ((distance2 - distance1) < 20) {
      pos = 0.0;
    }
    else {
      pos = (double)(-1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
    }
  }
  else if (distance1 > distance2) {
    if ((distance1 - distance2) < 20) {
      pos = 0.0;
    }
    else {
      pos = (double)(1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
    }
  }
  else {
    pos = 0.0;
  }
  return pos;
}


void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  Servo0.attach(A4);
  Servo1.attach(A5);
  
  //pwm.begin();
  //pwm.setOscillatorFrequency(25250000);
  //pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  IRsensor1.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
  IRsensor2.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  pid.SetSampleTime(50);
  pid.SetOutputLimits(-20, 20);
  pid.SetMode(AUTOMATIC);
  
  //set servos (arms) to 0 degrees
  //pwm.writeMicroseconds(0,returnUsec(0));
  //pwm.writeMicroseconds(1,returnUsec(180));
  platform._init(Servo0, Servo1, 0);
  delay(3000);
}


void loop() {
  //Loops to cycle servos through full range of motion

  setPoint = 0;
  
  posInput = returnPos();
  //Serial.print("Input :  ");
  //Serial.println(posInput);
  pid.Compute();
  Serial.print("millis: "); Serial.print(millis()); Serial.print(", newPlatformAngle:"); Serial.println(newPlatformAngle);
  platform.setAngle(newPlatformAngle);
  
}
