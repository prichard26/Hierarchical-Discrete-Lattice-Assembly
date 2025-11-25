#include <WiFi.h>
#include <WiFiMulti.h>

#include <WebSocketsServer.h>
#include <WiFiClientSecure.h>

#include <Wire.h>

#include "esp_wpa2.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define HeatSwitch A3

char host[] = "m1 "; // Used for MDNS resolution -- we do not use this no? 

WebSocketsServer webSocket = WebSocketsServer(80);

WiFiMulti wifiMulti;

const int LEDPIN = D0;
bool LEDON = false;

boolean newData = false;
const byte numChars = 32;
char recievedChars[numChars];
char tempChars[numChars];

int boardAddress = 0;
float desTheta = 0.0;
char cmdType[numChars] = {0};

int myAddress = 19; 

char positionBytes[4] = {0};

bool attachA = false;
int HeatInterval = 1000;

//const int HeatSwitch = D3;

void setup()
{
  Wire.setClock(10000);
  Wire.begin();
  delay(5);
  
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Initializing");

  wifiMulti.addAP("bille", "731021exhaust");
  Serial.println("Connecting WiFi...");
  delay(100);
  uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);                       //disable brownout detector
  //192.168.1.142 <- latest address

  if (wifiMulti.run() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("");
    Serial.println("WiFi connection failed!");
  }
  

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH); 

  pinMode(HeatSwitch, OUTPUT);
  digitalWrite(HeatSwitch, LOW);
}

void loop()
{
  SerialRoutine();
  // Look for and handle WebSocket data
  webSocket.loop();
  
}

void getSerialData(){
  while (Serial.available()>0 && newData == false){
    //uint8_t buf[1];
    //Serial.readBytes[buf, 1];
    
    char B = Serial.read();
    //Serial.println(B);
    static boolean recvInProgress = false;
    static byte j = 0;
    char startMarker = '<';
    char endMarker = '>';
    //char rc;

    if (recvInProgress == true){
      if (B != endMarker){
        recievedChars[j] = B;
        j++;
        if (j>=numChars){
          j=numChars-1;
        }
      }
      else {
        recievedChars[j] = '\0';
        recvInProgress = false;
        j = 0;
        newData = true;
      }
    }
    else if (B == startMarker){
      recvInProgress = true;
    }
  }
}

void parseSerialData(){
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  strcpy(cmdType, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  boardAddress = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  desTheta = atof(strtokIndx);     // convert this part to a float
}

void sendToI2C(){
  char B = cmdType[0];
  if (boardAddress == myAddress){
    // do our things
    if (B == 'G'){
      Serial.println("this functionality is not included");
    }
    else if (B == 'P'){
      Serial.println("this functionality is not included");
    }
    
    else if (B == 'A'){
      //attach
//      HeatInterval = int(1000*desTheta);
//      attachA = true;
      Serial.println("A");
    }
    else{
      Serial.println("unrec");
    }
  }
  else{
  if (B=='M'){
      //Move. Command M followed by float (desired angle in radians)
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      
      Wire.beginTransmission(boardAddress);
      Wire.write('M'); //command letter only move
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      //Wire.write(byteTofloatW.buffer);
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.print(desTheta);
      Serial.println(B);
      
    }
   else if (B == 'A'){
      //Attach. Command A followed by nonsense, attiny will attach next one.
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      Wire.beginTransmission(boardAddress);
      Wire.write('A'); //command letter only move
      //Wire.write("BBBB");
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      Wire.endTransmission();
      Serial.println(B);
    }
    else if (B == 'Z'){
      //Zero. Z followed by nonsense. Attiny will execute absolute zeroing. 
      Wire.beginTransmission(boardAddress);
      Wire.write('Z'); //command letter only move
      Wire.write("BBBB");
      Wire.endTransmission();
      //Serial.println('Z');
      Serial.println(B);
    }
    else if (B == 'S'){
      //do something
      Wire.beginTransmission(boardAddress);
      Wire.write('S'); //command letter only move
      Wire.write("BBBB");
      Wire.endTransmission();
      //LEDON = !LEDON;
      //digitalWrite(LEDPIN, LEDON);
      Serial.println(B);
    }
    else if (B == 'H'){
      Wire.beginTransmission(boardAddress);
      Wire.write('H');
      Wire.write("BBBB");
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.println(B);
    }
    else if (B == 'E'){
      //Request status from our single actuator at address 8
      Serial.println(B);
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      //Testing! request 5
      Wire.requestFrom(boardAddress, 5);
      char rc;
      char rcArray[7];
      static byte ndx = 0;
      while (Wire.available() > 0){
        rc = Wire.read();
        rcArray[ndx] = rc;
        ndx++;
        if (ndx >= 6){
          ndx = 5;
        }
      }
      //Serial.println(ndx);
      ndx = 0;
      for (int i = 0; i <=3; i++){
        byteTofloatW.buffer[i] = rcArray[i+1];
        positionBytes[i] = rcArray[i+1];
      }
      //Serial.print(rcArray[0]);
      //positionBytes = byteTofloatW.buffer;
      Serial.println(byteTofloatW.wFloat);
    }
    else if (B == 'J'){
      //Reset i2c bus???
      Serial.println("Checking bus...");
      int rtn = I2C_ClearBus();
      Serial.println(rtn);
      if (rtn == 0){
        Wire.begin();
      }
      Serial.println("done");
    }
    else if (B == 'C'){
      //power cycle
      //removed! 
      Serial.println("no");
    }
    else if (B == 'P'){
      // placement servo
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      
      Wire.beginTransmission(boardAddress);
      Wire.write('P'); //command letter only move
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      //Wire.write(byteTofloatW.buffer);
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.print(desTheta);
      Serial.println(B);
    }
    else if (B == 'V'){
      // placement servo
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      
      Wire.beginTransmission(boardAddress);
      Wire.write('V'); //command letter only move
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      //Wire.write(byteTofloatW.buffer);
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.print(desTheta);
      Serial.println(B);
    }
    else if (B == 'F'){
      //gripper servo
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      
      Wire.beginTransmission(boardAddress);
      Wire.write('F'); //command letter only move
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      //Wire.write(byteTofloatW.buffer);
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.print(desTheta);
      Serial.println(B);
    }
    else if (B == 'R'){
      //gripper servo
      union byteTofloat {
      char buffer[4];
      float wFloat;
      } byteTofloatW;
      
      Wire.beginTransmission(boardAddress);
      Wire.write('R'); //command letter only move
      byteTofloatW.wFloat = desTheta;
      for (int i = 0; i <=3; i++){
        Wire.write(byteTofloatW.buffer[i]);
      }
      //Wire.write(byteTofloatW.buffer);
      Wire.endTransmission();
      Serial.print(boardAddress);
      Serial.print(desTheta);
      Serial.println(B);
    }
    else if (B == '\n'){
      
    }
    else {
      //Serial.println("unrecognized");
      //Serial.println(B);
    }
  }
}

void SerialRoutine(){
  getSerialData();
  if (newData == true){
    strcpy(tempChars, recievedChars);
    parseSerialData();
    sendToI2C();
    newData = false;
  }
}

// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length){
  String output;

  // Figure out the type of WebSocket event
  switch (type)
  {

  // Client has disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  // New client has connected
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connection from ", num);
    Serial.println(ip.toString());
  }
  break;

  // parse message and confirm reciept
  case WStype_TEXT:
    parse_payload(payload, length, num);
    webSocket.sendTXT(4, "Recv");
    break;

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

void parse_payload(uint8_t *payload, size_t length, uint8_t num)
{
  Serial.printf("Received text: %s\n", payload);
  // Commands
  switch (payload[0])
  {
    case 'A':
    {
      cmdType[0] = 'A';
      boardAddress = payload[1];
      desTheta = payload[2];
      sendToI2C();
    } break;
    
    case 'M':
    {
      cmdType[0] = 'M';
      boardAddress = payload[1];
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      byteTofloatW.buffer[0] = payload[2];
      byteTofloatW.buffer[1] = payload[3];
      byteTofloatW.buffer[2] = payload[4];
      byteTofloatW.buffer[3] = payload[5];
      //float desThetaScale = payload[2];
      //desTheta = desThetaScale/100;
      desTheta = byteTofloatW.wFloat;
      sendToI2C();
    } break;
    
    case 'Z':
    {
      cmdType[0] = 'Z';
      boardAddress = payload[1];
      sendToI2C();
    } break;

    case 'H':
    {
      cmdType[0] = 'H';
      boardAddress = payload[1];
      sendToI2C();
    } break;

    case 'E':
    {
      cmdType[0] = 'E';
      boardAddress = payload[1];
      sendToI2C();
      uint8_t * bytePtr = (uint8_t*) &positionBytes;
      webSocket.sendBIN(num, bytePtr, sizeof(positionBytes));
    } break;

    case 'L':
    {
      uint8_t iters = payload[1];
      Serial.println(payload[1]);
      Serial.println(iters);
      for (uint8_t i = 0; i < iters; i++){
        digitalWrite(LEDPIN, LOW);
        delay(500);
        digitalWrite(LEDPIN, HIGH);
        delay(500);
        digitalWrite(LEDPIN, LOW);
        delay(500);
      }
      digitalWrite(LEDPIN, HIGH);
    } break;

    case 'V':
    {
      //Voxel holder
      cmdType[0] = 'V';
      boardAddress = payload[1];
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      byteTofloatW.buffer[0] = payload[2];
      byteTofloatW.buffer[1] = payload[3];
      byteTofloatW.buffer[2] = payload[4];
      byteTofloatW.buffer[3] = payload[5];
      //float desThetaScale = payload[2];
      //desTheta = desThetaScale/100;
      desTheta = byteTofloatW.wFloat;
      sendToI2C();
    } break;

    case 'P':
    {
      //place -- upper rail for the servo holder
      cmdType[0] = 'P';
      boardAddress = payload[1];
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      byteTofloatW.buffer[0] = payload[2];
      byteTofloatW.buffer[1] = payload[3];
      byteTofloatW.buffer[2] = payload[4];
      byteTofloatW.buffer[3] = payload[5];
      //float desThetaScale = payload[2];
      //desTheta = desThetaScale/100;
      desTheta = byteTofloatW.wFloat;
      sendToI2C();
    } break;

    case 'F':
    {
      //place -- lower rail for the servo holder
      cmdType[0] = 'F';
      boardAddress = payload[1];
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      byteTofloatW.buffer[0] = payload[2];
      byteTofloatW.buffer[1] = payload[3];
      byteTofloatW.buffer[2] = payload[4];
      byteTofloatW.buffer[3] = payload[5];
      //float desThetaScale = payload[2];
      //desTheta = desThetaScale/100;
      desTheta = byteTofloatW.wFloat;
      sendToI2C();
    } break;

    case 'R':
    {
      //Rotate the fastener
      cmdType[0] = 'R';
      boardAddress = payload[1];
      union byteTofloat {
        char buffer[4];
        float wFloat;
      } byteTofloatW;
      byteTofloatW.buffer[0] = payload[2];
      byteTofloatW.buffer[1] = payload[3];
      byteTofloatW.buffer[2] = payload[4];
      byteTofloatW.buffer[3] = payload[5];
      //float desThetaScale = payload[2];
      //desTheta = desThetaScale/100;
      desTheta = byteTofloatW.wFloat;
      sendToI2C();
    } break;

    case 'J':
    {
    //Reset i2c bus???
      Serial.println("Checking bus...");
      int rtn = I2C_ClearBus();
      Serial.println(rtn);
      if (rtn == 0){
        Wire.begin();
      }
      else{
        digitalWrite(LEDPIN, LOW);
      }
      Serial.println("done");
    }

    case 'C':
    {
      //tell it how many actuators to loop through
      uint8_t numActuators = payload[2];
      char boardAddresslist[numActuators] = {0};
      for (int i = 0; i < numActuators; i++){
        boardAddresslist[i] = payload[i+3]; //build list of board addresses
      }
      
      switch (payload[1]){
        case 'H':
        {
          for (int i =0; i< numActuators; i++){
            boardAddress = boardAddresslist[i];
            cmdType[0] = 'H';
            sendToI2C();
          }
        }break;

        case 'E':
        {
          for (int i =0; i< numActuators; i++){
            boardAddress = boardAddresslist[i];
            cmdType[0] = 'E';
            sendToI2C();
            uint8_t * bytePtr = (uint8_t*) &positionBytes;
            webSocket.sendBIN(num, bytePtr, sizeof(positionBytes));
          } 
        } break;

        case 'M':
        {
          union byteTofloat {
            char buffer[4];
            float wFloat;
            } byteTofloatW;

          uint8_t floatListLength = (length - numActuators - 3)/4;
          
          float floatList[floatListLength];
          
          uint8_t count = 0;
          
          for (int i=(numActuators+3); i < length; i++){
            if ((i-9)%4==0){
              //read payload values into the float converter buffer
              byteTofloatW.buffer[0]=payload[i];
              byteTofloatW.buffer[1]=payload[i+1];
              byteTofloatW.buffer[2]=payload[i+2];
              byteTofloatW.buffer[3]=payload[i+3];
              //add float value to list
              floatList[count] = byteTofloatW.wFloat;
              //count up
              count +=1;
            }
            else {
              //do nothing
            }
          }
          //pair desired float thetas with board addresses and send over i2c. 
          for (int i = 0; i < floatListLength; i++){
            if (i%numActuators == 0){
              delay(80);//was 100
            }
            cmdType[0] = 'M';
            boardAddress = boardAddresslist[i%numActuators];
            desTheta = floatList[i];
            sendToI2C();
            //delay(100);
          }
        } break;
        
        default:
        {
          Serial.print("invalid char after C: ");
          Serial.println(payload[1]);
        } break;
      }
    }break;
    default:
    {
      Serial.print("invalid CMD: ");
      Serial.println(payload[0]);
    } break;
  }
}

int thetaToMicroseconds(float thetaDes){
  int microSec;
//  float conversionFactor = 2000/3.14159265;
  float conversionFactor = 567.5465;
  float offset = 608;
  microSec = int(thetaDes * conversionFactor + offset);
  return microSec;
}

void attachActuator(int HeatInterval){
  unsigned long lastIntTime = millis();
  unsigned long intTimeNow = millis();
  //int HeatInterval = 15000; //1000 is 1 sec.
  //Set max limit. 
  if (HeatInterval > 40000){
    HeatInterval = 40000;
  }
  Serial.println(HeatInterval);
  while ((intTimeNow-lastIntTime) <= HeatInterval){
    digitalWrite(HeatSwitch, HIGH);
    intTimeNow = millis();
  }
//  analogWrite(HeatSwitch,0);
  digitalWrite(HeatSwitch, LOW);
  Serial.println("done");
  
}

int I2C_ClearBus() {
  
  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5us
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5us
    // The >5us is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5us
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5us
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
