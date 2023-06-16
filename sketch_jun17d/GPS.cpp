#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial mySerial(10, 11); // RX, TX

float longitude;
float latitude;

void start() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }


//  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
//  mySerial.println("Hello, world?");
}

char get_all(){
  start();
  return mySerial.read();
}

float get_long(){
  start();
  ////
  char c = mySerial.read();
  //Serial.print(c);
  gps.encode(c);
  ////
//  float longitude = gps.location.lng();
  return gps.location.lng();
}

float get_lati(){
  start();
  ////
  char c = mySerial.read();
  //Serial.print(c);
  gps.encode(c);
  ////
//  float latitude = gps.location.lat();
  return gps.location.lat();
}

float get_alt(){
  start();
  ////
  char c = mySerial.read();
  //Serial.print(c);
  gps.encode(c);
  ////
  return gps.altitude.meters();
}

//void loop() { // run over and over
//  while (mySerial.available() > 0){
//  char c = mySerial.read();
//  //Serial.print(c);
//  gps.encode(c);
//  if(gps.location.isUpdated()){
////  Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
////  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
////  Serial.print("ALT="); Serial.println(gps.altitude.meters());
//  }
//  }
//}
