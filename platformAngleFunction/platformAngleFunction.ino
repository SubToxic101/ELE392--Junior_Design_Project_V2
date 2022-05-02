float a = 0.000003;
float b = 0.0002;
float c = -0.0028;
float d = 0.7867;
float e = 45.028;

void setPlatformAngle(int pa, Servo l, Servo r) {
  l.writeMicroseconds(int(map(_polyModel(pa), 0, 180, 500, 2500)));
  r.writeMicroseconds(int(map(_polyModel(-1*pa), 0, 180, 500, 2500)));
}
float _polyModel(int i) {
// return float((0.00018937*pow(i,3)-0.00057721*pow(i,2)+0.74492032*i+44.93859251));
 return float(a*pow(i,4) + b*pow(i,3) + c*pow(i,2) + d*i + e);
}
