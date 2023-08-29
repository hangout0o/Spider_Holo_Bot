typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;
binaryFloat incoming[3]={1,2,3};



void setup(){
                        //Serial to print data on Serial Monitor
  Serial1.begin(115200,SERIAL_8N1,33,32);        //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
  
}

void loop() {
  // put your main code here, to run repeatedly:
     Serial1.write((byte*)incoming,sizeof(incoming)); 
     delay(10);
     Serial1.flush();
}
