#include <WiFi.h>

const char* ssid = "No wifi";                    //Enter your wifi hotspot ssid
const char* password =  "asdfghjkl:";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.178.171";

typedef union {
  int16_t floatingPoint;
  byte binary[2];
} binaryint;

binaryint incoming[4]={0,0,0,0};
WiFiClient client;

void setup(){
  Serial.begin(115200);                        //Serial to print data on Serial Monitor
  Serial1.begin(115200,SERIAL_8N1,33,32);        //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.println("...");}

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
   if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(200);
    return; }
    
  while(1){
      client.read((byte*)incoming, sizeof(incoming));       //Read the message through the socket until new line char(\n)
      client.print("Hello from ESP32!");

      Serial1.write((byte*)incoming,sizeof(incoming)); 
      Serial.println(incoming[0].floatingPoint);
      Serial.println(incoming[1].floatingPoint);
      Serial.println(incoming[2].floatingPoint); 
      Serial.println(incoming[3].floatingPoint);  
      delay(10);//Send data to AVR
      Serial1.flush();
    }
}
