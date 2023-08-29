String msg = "0";

int r = 6;
int g = 7;
int b = 8;

void setup() {
  pinMode(r,OUTPUT);
  pinMode(g,OUTPUT);
  pinMode(b,OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available()){                  //Check if any data is available on Serial
    msg = Serial.readStringUntil('\n');    //Read message on Serial until new char(\n) which indicates end of message. Received data is stored in msg
    int x = msg.toInt();
  
    if(x%2 == 0){                         //If data is even, turn on Blue LED
      analogWrite(r,255);
      analogWrite(g,255);
      analogWrite(b,0);
    }
    else{                                //If data is odd, turn on Red LED
      analogWrite(r,0);
      analogWrite(g,255);
      analogWrite(b,255);
    }
  }
}
