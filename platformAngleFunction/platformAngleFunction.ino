#include <Servo.h>

Servo left, right;

void setup() {
  Serial.begin(9600);
  left.attach(A3);
  right.attach(A2);
  

}


void loop() {
  setPlatformAngle(0,left,right);

}

void setPlatformAngle(int pa, Servo l, Servo r) {
  l.writeMicroseconds(int(map(_polyModel(pa), 0, 180, 500, 2500))+77);
  r.writeMicroseconds(int(map(180-_polyModel(-pa), 0, 180, 500, 2500)));

  Serial.print(int(map(_polyModel(pa), 0, 180, 500, 2500)));
  Serial.print(" / ");
  Serial.println(int(map(_polyModel(180-pa), 0, 180, 500, 2500)));
}

double a3 = -0.00018934;
double a2 = -0.00057738;
double a1 = -0.74494985;
double a0 = 44.93890101;

float _polyModel(int i) {
 return float((a3*(double)pow(i,3)+a2*(double)pow(i,2)+a1*(double)i+a0));
}
