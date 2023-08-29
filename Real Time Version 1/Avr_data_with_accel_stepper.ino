#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>


typedef union {
  int floatingPoint;
  byte binary[2];
} binaryint;

Servo servo;
int servoPin = 4;
binaryint incoming[4];


const int D_0= 13;
const int S_0= 12;
const int D_1= 11;
const int S_1= 10;
const int D_2= 9;
const int S_2= 5;
int r = 6;
int g = 8;
int b = 7;

AccelStepper M_0 = AccelStepper(1,S_0,D_0);
AccelStepper M_1 = AccelStepper(1,S_1,D_1);
AccelStepper M_2 = AccelStepper(1,S_2,D_2);
unsigned long timer = 0;

void publish();


void setup() {
  Serial.begin(115200);
  M_0.setMaxSpeed(1000);
  M_1.setMaxSpeed(1000);
  M_2.setMaxSpeed(1000);
  pinMode(r,OUTPUT);
  pinMode(g,OUTPUT);
  pinMode(b,OUTPUT);
  // put your setup code here, to run once:
   analogWrite(r,255);
   analogWrite(g,255);
   analogWrite(b,255);
   
   servo.attach(6);

}

void loop() {
    int test1 = incoming[3].floatingPoint;

  if (Serial.available() >= sizeof(incoming)) { // check if the array has been received
    Serial.readBytes((byte*)incoming,sizeof(incoming));
    
}
 
  publish(); 
  Serial.println(incoming[0].floatingPoint);
  Serial.println(incoming[1].floatingPoint);
  Serial.println(incoming[2].floatingPoint); 
  Serial.println(incoming[3].floatingPoint); 
  
  if (incoming[3].floatingPoint!=test1){
  timer = millis();}
 
  if(millis()-timer <=10)
  { // Serial.print("i'm here");
  if (incoming[3].floatingPoint ==0){
    servo10.write(90);}
    else{
      servo10.write(20);}}
}

void publish(){
  
  M_0.setSpeed(incoming[0].floatingPoint);
  M_1.setSpeed(incoming[1].floatingPoint);
  M_2.setSpeed(incoming[2].floatingPoint);
//  currentTime = millis();
//  while(millis()-currentTime<= T){
 
  M_0.runSpeed();
  M_1.runSpeed();
  M_2.runSpeed();}
  
  
