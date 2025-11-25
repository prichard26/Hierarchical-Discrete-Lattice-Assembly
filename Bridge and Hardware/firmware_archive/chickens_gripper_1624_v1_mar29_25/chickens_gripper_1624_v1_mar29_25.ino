
#include <Servo_megaTinyCore.h>
#include <Wire.h>

//FYI for the simple gripper: 0 is fully open, 1.7 is basically fully shut.

#define POT PIN_PA5 //potentiometer
#define PSERVO PIN_PA4 //servo
#define LED_PIN PIN_PA7

boolean newData = false;
const byte numChars = 32;
char recievedChars[numChars];
char tempChars[numChars];

int boardAddress = 25;
float desTheta = 0.0;
char cmdType[numChars] = {0};

int myAddress = 25; 

char positionBytes[4] = {0};

const int Addr = 25;                                              ; //i2c address, use 8+ 
// 0.75 is open, 1.4 is closed, for address 25, which is the first gripper

bool gripper = true; //is gripper or not? (servos have diff reqs).

float thetaDes = 1.5;

int curPot;
float curTheta;

const byte bufSize = 6;
char buf[bufSize];


bool initialized = false;

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


//declare servo object
Servo actuator;

void setup()
{
    //i2c
    Wire.begin(Addr); //address, use 8+
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    //SERIAL for debug
    Serial.begin(115200);
    
    //LIGHT
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);


    //Feedback line
    pinMode(POT, INPUT); 

    //little delay
    delay(5);
    
    
    //read potentiometer
//    for (int i = 1; i<=10; i++){
//      curPot +=analogRead(POT);
//    }
    curPot = analogRead(POT);
    //convert to theta in radians
    //thetaDes = potToTheta(curPot, gripper);
    
}

void loop()
{
  //check Serial routine...
  SerialRoutine();
  
  //Write the desired value to the servo.
  if (initialized){
    actuator.writeMicroseconds(thetaToMicroseconds(thetaDes));
  }
  //actuator.writeMicroseconds(thetaToMicroseconds(thetaDes));
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
  //we're not actually sending to the I2C bus fyi... 
  char B = cmdType[0];
  if (boardAddress == myAddress){
    if (B=='M'){
      thetaDes = desTheta;
      Serial.println(desTheta);
      if (initialized == false){
      actuator.attach(PSERVO);
      initialized = true;
    }
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


void receiveEvent(){
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
    
    thetaDes = FSconvert.FSfloat;
    Serial.println(thetaDes);
//    actuator.attach(PSERVO);
//    initialized = true;
    
    if (initialized == false){
      actuator.attach(PSERVO);
      initialized = true;
    }
  }
  
}

void requestEvent(){
  //sending status/position back
  Wire.write(buf[0]);
  //read potentiometer
  for (int i = 1; i<=10; i++){
    curPot +=analogRead(POT);
  }
  curPot = curPot/10;
  //curPot = analogRead(POT);
  //convert to theta in radians
  curTheta = potToTheta(curPot, gripper);
  Serial.println(curTheta);
  //write to union
  FSconvert2.FSfloat2 = curTheta;
  for (int i = 0; i<=3; i++){
    Wire.write(FSconvert2.buffer[i]);
  }
}

int thetaToMicroseconds(float thetaDes){
  int microSec;
  float conversionFactor = 2000/3.14159265;
  float offset = 500;
  //the two servos nominally have diff conversions, in practice they dont
  microSec = int(thetaDes * conversionFactor + offset);
  return microSec;
}

float potToTheta(int curPot, bool gripper){
  float conversionFactor;
  float offset;
  if (gripper == true){
    conversionFactor = 330/1.57;
    offset = 68;
  }
  else {
    conversionFactor = -1*323/1.57;
    offset = 740;
  }
  return ((curPot - offset)/conversionFactor - 0.15);
}
