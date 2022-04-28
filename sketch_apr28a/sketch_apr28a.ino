/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo2;
// twelve servo objects can be created on most boards 

int pos = 0;    // variable to store the servo position
int angl1 = 0;
int angl2 = 0;

int returnUsec(int deg) {
  double uSec;
  uSec = (((double)deg/180)*2000)+500;
  return (int)uSec;
}

void setup() {
  myservo.attach(A3);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(A2); 
}

void loop() {
  angl1 = 30;
  angl2 = 60;
  myservo.writeMicroseconds(returnUsec(angl1));
  myservo2.writeMicroseconds(returnUsec(180 - angl2));
  delay(1500);
  angl1 = 45;
  angl2 = 45;
  myservo.writeMicroseconds(returnUsec(angl1));
  myservo2.writeMicroseconds(returnUsec(180 - angl2));
  delay(1500);
  angl1 = 60;
  angl2 = 30;
  myservo.writeMicroseconds(returnUsec(angl1));
  myservo2.writeMicroseconds(returnUsec(180 - angl2));
  delay(1500);
  angl1 = 45;
  angl2 = 45;
  myservo.writeMicroseconds(returnUsec(angl1));
  myservo2.writeMicroseconds(returnUsec(180 - angl2));
  delay(1500);
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}
