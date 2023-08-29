#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiScan.h>
#include <WiFiMulti.h>
#include <WiFiGeneric.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>



// WiFi credentials
const char* ssid = "No wifi";                    //Enter your wifi hotspot ssid
const char* password =  "asdfghjkl:";               //Enter your wifi hotspot password
const uint16_t port = 8002;
const char * host = "192.168.80.171";                   //Enter the ip address of your laptop after connecting it to wifi hotspot


 
char incomingPacket[80];
WiFiClient client;

String msg = "0";
float array[3];

void setup(){
   
  Serial.begin(115200);                          //Serial to print data on Serial Monitor
  Serial1.begin(115200,SERIAL_8N1,33,32);        //Serial to transfer data between ESP and AVR. The Serial connection is inbuilt.
  
  
  //Connecting to wifi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}


void loop() {

  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    delay(200);
    return;
  }

  while(1){
      client.read((uint8_t*)array, sizeof(array));       //Read the message through the socket until new line char(\n)
//      msg1 = client.readStringUntil('\n'); 
//      msg2 = client.readStringUntil('\n'); 
      client.print("Hello from ESP32!");
      for(int i=0;i<3;i++){
      Serial.println(array[i]);}//Send an acknowledgement to host(laptop)
     
//      Serial.println(msg1);  
//Print data on Serial monitor
      Serial1.println(msg);                       //Send data to AVR
      Serial1.flush();
    }
}
