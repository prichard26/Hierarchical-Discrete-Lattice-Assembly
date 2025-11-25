#include <Wire.h>

#define HeatSwitch PIN_PA3 //resistive heating switch

const int Addr = 12; //i2c addr

float eResp = 10;

const byte bufSize = 6;
char buf[bufSize];

bool attachA = false;

int HeatInterval = 1000;

boolean newData = false;
const byte numChars = 32;
char recievedChars[numChars];
char tempChars[numChars];

int boardAddress = 0;
float desTheta = 0.0;
char cmdType[numChars] = {0};

int incomingByte = 0; // for incoming serial data

//float-string converter
union FSconverter {
    char buffer[4];
    float FSfloat;
  } FSconvert;
union FSconverter2 {
    char buffer[4];
    float FSfloat2;
  } FSconvert2;
//float-string for attach converter
union ISconverter {
  char buffer[4];
  float ISint;
} ISconvert;


void setup()
{
    //i2c
    Wire.begin(Addr); //address, use 8+
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    //Heater
    analogWrite(HeatSwitch, 0);

    //little delay
    delay(5);

    Serial.begin(115200);

    delay(5);
    
    
}

void loop()
{
//  SerialRoutine();//check serial line
  //check if we want to attach new module
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.println(incomingByte);
//    Serial.println(incomingByte, DEC);
    if (incomingByte == 'A'){
      HeatInterval = 30000;
      attachA = true;
    }
  }
  if (attachA == true){
    attachActuator(HeatInterval);
    attachA=false;
  }
 
}
//void getSerialData(){
//  while (Serial.available()>0 && newData == false){
//    //uint8_t buf[1];
//    //Serial.readBytes[buf, 1];
//    
//    char B = Serial.read();
//    //Serial.println(B);
//    static boolean recvInProgress = false;
//    static byte j = 0;
//    char startMarker = '<';
//    char endMarker = '>';
//    //char rc;
//
//    if (recvInProgress == true){
//      if (B != endMarker){
//        recievedChars[j] = B;
//        j++;
//        if (j>=numChars){
//          j=numChars-1;
//        }
//      }
//      else {
//        recievedChars[j] = '\0';
//        recvInProgress = false;
//        j = 0;
//        newData = true;
//      }
//    }
//    else if (B == startMarker){
//      recvInProgress = true;
//    }
//  }
//}
//
//void parseSerialData(){
//  char * strtokIndx; // this is used by strtok() as an index
//
//  strtokIndx = strtok(tempChars,",");      // get the first part - the string
//  strcpy(cmdType, strtokIndx); // copy it to messageFromPC
//
//  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
//  boardAddress = atoi(strtokIndx);     // convert this part to an integer
//
//  strtokIndx = strtok(NULL, ",");
//  desTheta = atof(strtokIndx);     // convert this part to a float
//}
//
//void sendToI2C(){
//  char B = cmdType[0];
//  if (boardAddress==Addr){
//    //do the actions - don't send to bus. 
//    if (B =='M'){
//      Serial.println("attiny no mot");
//    }
//    else if (B == 'A'){
//      HeatInterval = int(1000*desTheta);
//      attachA = true;
//    }
//    else if (B == 'E'){
//      Serial.println(Addr);
//    }
//    else{
//      
//  }
//  }
//  else{
//    Serial.println("attiny only");
//  }
//}
//
//void SerialRoutine(){
//  getSerialData();
//  if (newData == true){
//    strcpy(tempChars, recievedChars);
//    parseSerialData();
////    sendToI2C();
//    newData = false;
//  }
//}


void receiveEvent(int){
  //M means update thetaDes, A means attach new actuator
  static byte ndx=0;
  char rc;
  
  while (Wire.available()>0){
    rc = Wire.read();
    //Serial.println(rc);
    buf[ndx] = rc;
    ndx++;
    if (ndx >= bufSize){
      //prevent accidental overflow
      ndx = bufSize - 1;
    }
  }
  ndx = 0;
  if (buf[0]=='M'){
    // Move to new theta
    for (int i = 1; i <= 4; i++){
      FSconvert.buffer[i-1] = buf[i];
    }
    
    Serial.println("no");
  }
  if (buf[0]=='A'){
    //attach new actuator
    for (int i = 1; i <= 4; i++){
      ISconvert.buffer[i-1] = buf[i];
    }
    HeatInterval = int(1000*ISconvert.ISint);
    attachA = true;
  }
  
}

void requestEvent(){
  //sending status/position back
  Wire.write(buf[0]);
  
  //write to union
  FSconvert2.FSfloat2 = eResp;
  for (int i = 0; i<=3; i++){
    Wire.write(FSconvert2.buffer[i]);
  }
}

void attachActuator(int HeatInterval){
  unsigned long lastIntTime = millis();
  unsigned long intTimeNow = millis();
  //int HeatInterval = 15000; //1000 is 1 sec.
  //Set max limit. 
  if (HeatInterval > 40000){
    HeatInterval = 40000;
  }
  Serial.println("attaching");
  Serial.println(HeatInterval);
  while ((intTimeNow-lastIntTime) <= HeatInterval){
    analogWrite(HeatSwitch, 250);
    intTimeNow = millis();
  }
  analogWrite(HeatSwitch,0);
  Serial.println("done");
  
}
