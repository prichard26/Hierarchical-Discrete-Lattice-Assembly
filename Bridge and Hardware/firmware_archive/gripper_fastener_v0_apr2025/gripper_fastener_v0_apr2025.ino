#include <Encoder.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

boolean newData = false;
const byte numChars = 32;
char recievedChars[numChars];
char tempChars[numChars];

int boardAddress = 26;
float desTheta = 0.0;
char cmdType[numChars] = {0};

const byte bufSize = 6;
char buf[bufSize];

int myAddress = 26; 

char positionBytes[4] = {0};

const int Addr = 26;//i2c address, use 8+ 

bool initialized1 = false;
bool initialized2 = false;


// Motor pins 
const int motor1_A = 12;
const int motor1_B = 13;
const int motor2_A= 10;
const int motor2_B = 11;
const int motor3_A = 6;
const int motor3_B = 9;
const int motor3_ref = 5;
const int motor2_ref = A1;
const int motor1_ref = A0;

// Encoder pins
const int encoder1A = A2;   // Encoder A for motor 1
const int encoder1B = A3;   // Encoder B for motor 1
const int encoder2A = A5;   // Encoder A for motor 2
const int encoder2B = A4;   // Encoder B for motor 2
const int encoder3A = 24;   // Encoder A for motor 3
const int encoder3B = 25;   // Encoder B for motor 3

// Servo pins
const int servo1Pin = 4; // true gripper: 1.1 is open, 0.65 is closed. 
const int servo2Pin = 1; // holder: 0 is open, 0.75 is closed

// Encoder objects
Encoder encoder1(encoder1A, encoder1B);
Encoder encoder2(encoder2A, encoder2B);
Encoder encoder3(encoder3A, encoder3B);

// Servo objects
Servo servo1;
Servo servo2;

// PID variables
float Kp = 35.0, Ki = 0.0, Kd = 5.0; // Tunable PID constants
long targetPosition1 = 0, targetPosition2 = 0, targetPosition3 = 0;
long prevError1 = 0, prevError2 = 0, prevError3 = 0;
long integral1 = 0, integral2 = 0, integral3 = 0;
long lastTime = 0;

float motor3Speed = 0;

float thetaDes1;
float thetaDes2;

// BUILT in NEOPIXEl 
#define NEOPIXEL_PIN 8
#define NEOPIXEL_COUNT 1
Adafruit_NeoPixel builtinNeopixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

void setup() {
  // Initialize motor pins
  pinMode(motor1_A, OUTPUT);
  pinMode(motor1_B, OUTPUT);
  pinMode(motor2_A, OUTPUT);
  pinMode(motor2_B, OUTPUT);
  pinMode(motor3_A, OUTPUT);
  pinMode(motor3_B, OUTPUT);

  pinMode(motor1_ref, OUTPUT);
  pinMode(motor2_ref, OUTPUT);
  pinMode(motor3_ref, OUTPUT);
  digitalWrite(motor1_ref, HIGH);
  digitalWrite(motor2_ref, HIGH);
  digitalWrite(motor3_ref, HIGH);
  
  
  // Start Serial for debugging
  Serial.begin(115200);
  
  //i2c
  Wire.begin(Addr); //address,
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Initialize servos
//  servo1.attach(servo1Pin);
//  servo2.attach(servo2Pin);
//  // Set initial servo positions (optional)
//  servo1.write(90);  // 90 degrees is neutral
//  servo2.write(90);  // 90 degrees is neutral
  encoder1.write(0);
  encoder2.write(0);
  encoder3.write(0);

  builtinNeopixel.begin();
  builtinNeopixel.setPixelColor(0, 25, 25, 25);
  builtinNeopixel.show();
}

void loop() {
  //check Serial routine...
  SerialRoutine();
  // Read encoder positions
  long position1 = encoder1.read();
  long position2 = encoder2.read();
  long position3 = encoder3.read();

  // PID control for motor 1 - UPPER RAIL MOTOR
  float motor1Speed = pidControl(position1, targetPosition1, 1);
  runMotors(motor1Speed, motor1_A, motor1_B);

  // PID control for motor 2 - THE LOWER RAIL MOTOR
  float motor2Speed = pidControl(position2, targetPosition2, 2);
  runMotors(motor2Speed, motor2_A, motor2_B);

  // MOTOR 3 - we only run at the desired speed
  runMotors(motor3Speed, motor3_A, motor3_B);
  
  if (initialized1){
    servo1.writeMicroseconds(thetaToMicroseconds(thetaDes1));
//    servo2.writeMicroseconds(thetaToMicroseconds(thetaDes2));
  }
  if (initialized2){
//    servo1.writeMicroseconds(thetaToMicroseconds(thetaDes1));
    servo2.writeMicroseconds(thetaToMicroseconds(thetaDes2));
  }

}

// PID control function
float pidControl(long currentPosition, long targetPosition, int motorNum) {
  long error = targetPosition - currentPosition;
  long currentTime = millis();
  long deltaTime = currentTime - lastTime;

  if (deltaTime == 0) return 0;

  // Proportional term
  long proportional = Kp * error;

  // Integral term (accumulating error over time)
  integral1 += error * deltaTime;
  long integral = Ki * integral1;

  // Derivative term (rate of change of error)
  long derivative = Kd * (error - prevError1) / deltaTime;

  // Calculate PID output
  long output = proportional + integral + derivative;

  // Save current error and time for next cycle
  prevError1 = error;
  lastTime = currentTime;

  // Return PWM value (output) limited to a maximum range
  return constrain(output, -255, 255);  // Motor speed ranges from -255 to 255
}

void runMotors(float outPWM, int IN1, int IN2){
  if (outPWM < 0.0){
    analogWrite(IN2, 0);
    analogWrite(IN1, abs(outPWM));
    //Serial.println('a');
  }
  else if (outPWM > 0.0){
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(outPWM));
    //Serial.println('b');
  }
  else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    //Serial.println('c');
  }
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
      // SERVO 1 -> true gripper
      thetaDes1 = desTheta;
      Serial.println(desTheta);
      if (initialized1 == false){
      servo1.attach(servo1Pin);
      initialized1 = true;
    }
    }
    else if (B=='V'){
      //SERVO 2 -> Voxel holder
      thetaDes2 = desTheta;
      Serial.println(desTheta);
      if (initialized2 == false){
      servo2.attach(servo2Pin);
      initialized2 = true;
    }
    }
    else if (B=='P'){
      // Place the voxel, i.e. the upper rail MOTOR1
      targetPosition1 = desTheta;
      Serial.println(desTheta);
    }
    else if (B=='F'){
      // Move the second motor, i.e. the lower rail
      targetPosition2 = desTheta; //  move lower rail 
//      motor3Speed = 50; // spin the fastener
    }
    else if (B=='R'){
      motor3Speed = int(10*desTheta);//set the speed of the fastener
//      motor3Speed = -50; // spin the fastener
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
    
    thetaDes1 = FSconvert.FSfloat;
    Serial.println(thetaDes1);
//    actuator.attach(PSERVO);
//    initialized = true;
    
    if (initialized1 == false){
      servo1.attach(servo1Pin);
//      actuator.attach(servo2);
      initialized1 = true;
    }
  }
  else if (buf[0]=='V'){
    // Voxel gripper
    for (int i = 1; i <= 4; i++){
      FSconvert.buffer[i-1] = buf[i];
    }
    
    thetaDes2 = FSconvert.FSfloat;
    Serial.println(thetaDes2);
//    actuator.attach(PSERVO);
//    initialized = true;
    
    if (initialized2 == false){
//      actuator.attach(servo1);
      servo2.attach(servo2Pin);
      initialized2 = true;
    }
  }
  if (buf[0]=='P'){
    // Upper rail
    for (int i = 1; i <= 4; i++){
      FSconvert.buffer[i-1] = buf[i];
    }
    
    targetPosition1 = FSconvert.FSfloat;
    Serial.println(targetPosition1);
  }
  if (buf[0]=='F'){
    // Lower rail
    for (int i = 1; i <= 4; i++){
      FSconvert.buffer[i-1] = buf[i];
    }
    
    targetPosition2 = FSconvert.FSfloat;
    Serial.println(targetPosition2);
//    motor3Speed = 50;
  }
  if (buf[0]=='R'){
    // fastener motor
    for (int i = 1; i <= 4; i++){
      FSconvert.buffer[i-1] = buf[i];
    }
    
    motor3Speed = int(10*FSconvert.FSfloat);
    Serial.println(motor3Speed);
//    motor3Speed = -50;
  }
  
}

void requestEvent(){
  //sending status/position back
  Wire.write(buf[0]);
  //Dummy variable until I figure out what I want to send. 
  //write to union
  FSconvert2.FSfloat2 = 3.0;
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
