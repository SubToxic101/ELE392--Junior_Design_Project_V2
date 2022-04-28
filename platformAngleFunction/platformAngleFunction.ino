
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void setPlatformAngle(int pa, Servo l, Servo r) {
  l.writeMicroseconds(int(map(_polyModel(pa), 0, 180, 500, 2500)));
  r.writeMicroseconds(int(map(_polyModel(-1*pa), 0, 180, 500, 2500)));
}
float _polyModel(int i) {
 return float((0.00018937*pow(i,3)-0.00057721*pow(i,2)+0.74492032*i+44.93859251));
}
