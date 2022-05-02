#include <Servo.h>
#include <Platform.h>
#include <SharpDistSensor.h>

int sensor1Pin = A1;
int sensor2Pin = A0;

Servo Servo0;
Servo Servo1;

Platform platform = Platform();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  Servo0.attach(A4);
  Servo1.attach(A5);

  platform._init(Servo0, Servo1, sensor1Pin, sensor2Pin, 0);
  delay(3000);

  //  Serial.println(platform.SERVO0_ANGLE);
}

void loop() {
  platform.setAngle(-15);
  delay(3000);
  platform.setAngle(0);
  delay(3000);
  platform.setAngle(15);
  delay(3000);
  platform.setAngle(0);
  delay(3000);

  // put your main code here, to run repeatedly:
  //platform.setAngle(10);
  //delay(3000);
  //platform.setAngle(15);
  //delay(3000);
  //platform.setAngle(20);
  //delay(3000);
}
