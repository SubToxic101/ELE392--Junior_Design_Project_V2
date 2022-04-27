#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)  unused
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)   unused
#define USMIN  500 // This is the rounded 'minimum' microsecond length
#define USMAX  2500 // This is the rounded 'maximum' microsecond length
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int servo0Out = 0;
int servo1Out = 0;
int inputDeg = 0;
double posInput = 0;
double anglOutput = 0;
double setPoint = 0;
double sensor1 = 0;
double sensor2 = 0;

double Kp1 = 0;
double Ki1 = 0;
double Kd1 = 0;

PID platform(&posInput, &anglOutput, &setPoint, Kp1, Ki1, Kd1, DIRECT);

//Function for converting degrees to Usec.
int returnUsec(int deg) {
  double uSec;
  uSec = (((double)deg/180)*2000)+500;
  return (int)uSec;
}


void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pwm.begin();
  pwm.setOscillatorFrequency(25250000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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
  
  sensor1 = analogRead(A0);
  sensor2 = analogRead(A1);
  
  //Commands to write base to arm angle (one servo is reversed):
  /*
  servo0Out = 90;
  servo1Out = 90;
  pwm.writeMicroseconds(0,returnUsec(servo0Out));
  pwm.writeMicroseconds(1,returnUsec(180 - servo1Out));
  */
}
