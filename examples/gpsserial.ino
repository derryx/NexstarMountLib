#include <TinyGPS.h>
#include <NexstarMount.h>
#include <SoftwareSerial.h>

#define RX_PIN 3
#define TX_PIN 4
#define LED_PIN 13

#define SIGNAL_PIN 12

#define RX2_PIN 7
#define TX2_PIN 8

#define RTS_PIN 5
#define CTS_PIN 6

SoftwareSerial mountserial(RX_PIN,TX_PIN);
SoftwareSerial sendmountserial(RX2_PIN,TX2_PIN);

TinyGPS gps;

NexstarMessageReceiver msg_receiver;
NexstarMessageSender msg_sender(&gps,RTS_PIN,CTS_PIN);

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 1000;           // interval at which to blink (milliseconds)

boolean haveLock=false;

void setup() {
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  
  pinMode(SIGNAL_PIN,OUTPUT);
  digitalWrite(SIGNAL_PIN,LOW);
  
  mountserial.begin(19200);
  
  // Set RTS/CTS to tri-state
  pinModeTri(RTS_PIN);
  pinModeTri(CTS_PIN);
  pinModeTri(RX2_PIN);
  pinModeTri(TX2_PIN);
  msg_receiver.reset();
  
  Serial.begin(38400);
//  send_msg_gps();
//  Serial.end();
//  Serial.begin(9600);
}

void loop() {
  unsigned long fix_age;
  
  if (mountserial.available()) {
    int c=mountserial.read();
    if(msg_receiver.process(c))
    {
      if(msg_sender.handleMessage(&msg_receiver)) {
         mountserial.end();
         pinMode(TX2_PIN,OUTPUT);
         digitalWrite(TX2_PIN,HIGH);
         sendmountserial.begin(19200);
         
         msg_sender.send(&sendmountserial);
         sendmountserial.end();
         pinModeTri(RX2_PIN);
         pinModeTri(TX2_PIN);
         mountserial.begin(19200);
      }
    }
  }
  
  if (Serial.available()) {
    char c=Serial.read();
    gps.encode(c);
  
    gps.get_datetime(NULL,NULL,&fix_age);
    
    if (fix_age==gps.GPS_INVALID_AGE || fix_age>5000 || gps.satellites()==gps.GPS_INVALID_SATELLITES || gps.satellites()<4) {
      haveLock=false;
    } else {
      haveLock=true;
    }
  }

  if (haveLock) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(SIGNAL_PIN, HIGH);         
  } else {
    unsigned long currentMillis = millis();  
    if(currentMillis - previousMillis > interval) {
      // save the last time you blinked the LED 
      previousMillis = currentMillis;   
  
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
  
      // set the LED with the ledState of the variable:
      digitalWrite(LED_PIN, ledState);
      digitalWrite(SIGNAL_PIN, ledState);
    }
  }
}

/*
void send_msg_gps() {
  char message[]="PUBX,41,1,0007,0002,9600,0";
  int i=0;
  int checksum=0;
  
  Serial.print('$');
  
  while (message[i]!='\0') {
    Serial.write(message[i]);
    checksum^=message[i];
    i++;
  }
  Serial.print('*');
  if (checksum&0xFF<0x10) Serial.write('0');
  Serial.println(checksum&0xFF,HEX);
  Serial.flush();
}
*/

inline void pinModeTri(int pin) {
  digitalWrite(pin,LOW);
  pinMode(pin,OUTPUT);
  pinMode(pin,INPUT);
}