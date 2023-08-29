typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

binaryFloat incoming[3];

int r = 6;
int g = 8;
int b = 7;

void setup() {
  Serial.begin(115200);
  pinMode(r,OUTPUT);
  pinMode(g,OUTPUT);
  pinMode(b,OUTPUT);
  // put your setup code here, to run once:
   analogWrite(r,255);
   analogWrite(g,255);
   analogWrite(b,255);

}

void loop() {
  if (Serial.available() >= sizeof(incoming)) { // check if the array has been received
    Serial.readBytes((byte*)incoming,sizeof(incoming));


}
  Serial.println(incoming[0].floatingPoint);
  Serial.println(incoming[1].floatingPoint);
  Serial.println(incoming[2].floatingPoint); 

}
