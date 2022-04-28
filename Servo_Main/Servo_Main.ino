#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <SharpDistSensor.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)  unused
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)   unused
#define USMIN  500 // This is the rounded 'minimum' microsecond length
#define USMAX  2500 // This is the rounded 'maximum' microsecond length
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int servo0Out = 0;
int servo1Out = 0;
double posInput = 0;
double anglOutput = 0;
double setPoint = 0;
double sensor1Pin = A0;
double sensor2Pin = A1;


double a3 = -0.00018934;
double a2 = -0.00057738;
double a1 = -0.74494985;
double a0 = 44.93890101;

double b3 = 0.00018937;
double b2 = -0.00057721;
double b1 = 0.74492032;
double b0 = 44.93859251;

double Kp1 = 0;
double Ki1 = 0;
double Kd1 = 0;



SharpDistSensor IRsensor1(sensor1Pin, 5);
SharpDistSensor IRsensor2(sensor2Pin, 5);
PID platform(&posInput, &anglOutput, &setPoint, Kp1, Ki1, Kd1, DIRECT);

double angle_to_t1(double angle){
  return a3*angle*angle*angle+a2*angle*angle+a1*angle+a0;
}

double angle_to_t2(double angle){
  return b3*angle*angle*angle+b2*angle*angle+b1*angle+b0;
}

//Function for converting degrees to Usec.
int returnUsec(int deg) {
  double uSec;
  uSec = (((double)deg/180)*2000)+500;
  return (int)uSec;
}


double returnPos() {
  double pos = 0.0;
  //platform length
  int len = 320;
  int distance1 = IRsensor1.getDist();
  int distance2 = IRsensor2.getDist();

  if (distance1 < distance2) {
    if ((distance2 - distance1) < 30) {
      pos = 0.0;
    }
    else {
      pos = (double)(-1*((abs(distance1-(len/2))+abs(distance2-(len/2)))/2));
    }
  }
  else if (distance1 > distance2) {
    if ((distance1 - distance2) < 30) {
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
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(25250000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  IRsensor1.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);
  IRsensor2.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  platform.SetSampleTime(50);
  platform.SetOutputLimits(-500, 500);
  platform.SetMode(AUTOMATIC);
  
  //set servos (arms) to 0 degrees
  pwm.writeMicroseconds(0,returnUsec(0));
  pwm.writeMicroseconds(1,returnUsec(180));
  delay(3000);
}


void loop() {
  //Loops to cycle servos through full range of motion
  
  for (int i = 0; i < 180; i++) {
    servo0Out = i;
    servo1Out = i;
    pwm.writeMicroseconds(0,returnUsec(servo0Out));
    pwm.writeMicroseconds(1,returnUsec(180 - servo1Out));
    delay(5);
  }

  for (int g = 180; g > 0; g--) {
    servo0Out = g;
    servo1Out = g;
    pwm.writeMicroseconds(0,returnUsec(servo0Out));
    pwm.writeMicroseconds(1,returnUsec(180 - servo1Out));
    delay(5);
  }
  
  //posInput = returnPos();

  //platform.Compute();
  
  //Commands to write base to arm angle (one servo is reversed):

  /*
  servo0Out = 0;
  servo1Out = 0;
  pwm.writeMicroseconds(0,returnUsec(servo0Out));
  pwm.writeMicroseconds(1,returnUsec(180 - servo1Out));
  //Serial.println(angle_to_t1(0));
  //Serial.println(angle_to_t2(0));
  */
}
