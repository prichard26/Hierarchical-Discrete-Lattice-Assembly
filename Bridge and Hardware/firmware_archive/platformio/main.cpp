#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "SimpleFOCDrivers.h"
#include "drivers/drv8316/drv8316.h"
#include "encoders/as5047/MagneticSensorAS5047.h"
#include <Adafruit_NeoPixel.h>

#define NSLEEP 19
#define NFAULT 18
#define DRVOFF 0
#define VREF 14
#define DRIVER_CS 5

#define SENSOR_CS 10
#define SENSOR_CS_2 4

#define HEATER 1 //"tx" aka PB16 aka 1

#define USB_P 0

bool MAS = 0; // is i2c master?

// BUILT in NEOPIXEl 
#define NEOPIXEL_PIN 8
#define NEOPIXEL_COUNT 1
Adafruit_NeoPixel builtinNeopixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

MagneticSensorSPIConfig_s AS5047 = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 4000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorAS5047 sensor(SENSOR_CS, false, SPISettings(4000000, BitOrder::MSBFIRST, SPI_MODE1));
MagneticSensorAS5047 sensor2(SENSOR_CS_2, false, SPISettings(4000000, BitOrder::MSBFIRST, SPI_MODE1));

// BLDCMotor motor = BLDCMotor(11, 3, 150);
// BLDCMotor motor = BLDCMotor(11,5.50f/2.0);
BLDCMotor motor = BLDCMotor(11);

DRV8316Driver3PWM driver = DRV8316Driver3PWM(13,11,9,DRIVER_CS,false); 

#define ENABLE_LOW 6



//offset
float zero_offset = 0.0;

// instantiate the commander
Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
// void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
// void doMotor(char* cmd){ command.motor(&motor, cmd); }
// void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

// serial-i2c stuff
boolean newData = false;
const byte numChars = 32;
char recievedChars[numChars];
char tempChars[numChars];

int boardAddress = 0;
float desTheta = 0.0;
char cmdType[numChars] = {0};

//ADDRESS!! 
int myAddress = 24;
//HERE!!!

char positionBytes[4] = {0};

const byte bufSize = 6;
char buf[bufSize];

bool attachA = false;
int HeatInterval = 1000;

float curTheta = 0.0;

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

//PID stuff

int PIDtimeInterval = 10; //10millis
unsigned long LastTime = millis();

float Kp = -200; // it goes the opposite way, recall.. was -500 all zero other
float Kd = 1000; 
float Ki = 0;

float maxSpeed = 200;
float errorSum = 0;
float maxError = maxSpeed;
float minError = -1*maxSpeed;
float lastOutput, lastInput;

//target variable
float target_velocity = 0; // what we command from simpleFOC
float target_angle = 0; // the desired angle of the output shaft
float current_angle = 0; // the current angle of the output shaft
float last_target_velocity = 0;

// soft joint limits to avoid improperly parsed I2C commands 

float max_angle = 12;
float min_angle = -12;

void printDRV8316Status();
void printSensorStatus();
void getSerialData();
void parseSerialData();
void sendToI2C();
void SerialRoutine();
void requestEvent();
void receiveEvent(int);
void attachActuator(int HeatInterval);
void PID();

void setup() {
  delay(1000);
  Serial.println("Start");
  pinMode(NSLEEP, OUTPUT);
  digitalWrite(NSLEEP, HIGH);
  
  pinMode(NFAULT, INPUT_PULLUP);

  pinMode(DRVOFF, OUTPUT);
  digitalWrite(DRVOFF, LOW);

  pinMode(VREF, OUTPUT);
  digitalWrite(VREF, HIGH);

  // enable for 3PWM mode
  pinMode(ENABLE_LOW, OUTPUT);
	digitalWrite(ENABLE_LOW, 1); // enable

  // i2C
  // Wire.setClock(10000);
  // Wire.begin(); // master - just for testing on the feather. 
  //else:
  Wire.begin(myAddress); 
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  delay(5);

  motor.useMonitoring(Serial);
  Serial.println("Starting Motor configuration ....");

  sensor.init();
  motor.linkSensor(&sensor);

  sensor2.init();

  // driver config

  driver.voltage_power_supply = 14;
  driver.voltage_limit = 14;

  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.init();
  
  // link the motor and the driver
  motor.linkDriver(&driver);

  motor.voltage_limit = 9;   // [V] WAS 6 FOR ALL OTHER JOINTS... 
 
  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = 0.1;
  // motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0;
  // motor.LPF_velocity.Tf = 0.5;
  // motor.PID_velocity.output_ramp = 50;

  // motor.P_angle.P = 20;

  // motor.current_limit = 1.5;
  motor.velocity_limit = maxSpeed;

  // init motor hardware
  motor.init();

  // // add target command T
  // command.add('T', doTarget, "target velocity");
  // command.add('L', doLimit, "voltage limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  // Serial.println("Set target velocity [rad/s]");
  _delay(1000);
  // DRV8316_PWMMode val = driver.getPWMMode();
	// Serial.print("PWM Mode: ");
	// Serial.println(val);
  // delay(100);
	printDRV8316Status();
  printSensorStatus();
  // delay(100);
  Serial.println(digitalRead(NFAULT));

  motor.initFOC();

  // set the inital target value
  motor.target = 0;//target_velocity, don't move initially

  // define the motor id
  // command.add('M', doMotor, "motor");
  // // add target command T
  // // command.add('T', doTarget, "target angle");
  

  // motor.monitor_start_char = 'M'; // the same latter as the motor id in the commander 
  // motor.monitor_end_char = 'M'; // the same latter as the motor id in the commander 
  // motor.motion_downsample = 5;

  command.verbose = VerboseMode::machine_readable;

  builtinNeopixel.begin();
  builtinNeopixel.setPixelColor(0, 50, 50, 50);
  builtinNeopixel.show();
  // //heater
  // pinMode(HEATER, OUTPUT);
  // analogWrite(HEATER, 0);
  //heater
  pinMode(HEATER, INPUT); // tbh there's something wrong with the analog write situation. we'll try just toggling the pinmode type.
  // digitalWrite(HEATER, 0);
  sensor.update();
  sensor2.update();
  current_angle = sensor2.getAngle();
  target_angle = current_angle;
  lastInput = current_angle;
  LastTime = millis();
  // target_angle = sensor.getAngle();
  
}

void loop() {

  SerialRoutine(); // check for comms (i.e. check for desired angle)

  // sensor2.update(); // update 
  // sensor.update();
  // Serial.print(sensor.getAngle());
  // Serial.print(", ");
  // Serial.println(sensor2.getAngle());

  PID();
  
  motor.loopFOC();
  motor.move(target_velocity);



  
  // motor.move(5);

  // motor.monitor();
  // Serial.print(sensor.getCurrentAngle());
  // Serial.print(", ");
  // Serial.println(sensor.getAngle());
  // pinMode(HEATER, OUTPUT);
  // digitalWrite(HEATER, LOW);
  // command.run(); // won't use their commander
  // printDRV8316Status();
  // delay(1000);
  if (attachA == true){
    Serial.println("attaching");
    Serial.println(HeatInterval);
    builtinNeopixel.setPixelColor(0, 0, 50, 0);
    builtinNeopixel.show();
    pinMode(HEATER, OUTPUT);
    attachActuator(HeatInterval);
    attachA=false;
    builtinNeopixel.setPixelColor(0, 50, 50, 50);
    builtinNeopixel.show();
    // pinMode(HEATER, INPUT);
  }
  //If errors:
  if (digitalRead(NFAULT)==0){
    // builtinNeopixel.setPixelColor(0, 50, 0, 0);
    // builtinNeopixel.show();
    // printDRV8316Status();
    // Serial.println(digitalRead(NFAULT));
    digitalWrite(NSLEEP, HIGH);
    digitalWrite(DRVOFF, LOW);
  }


}

void PID(){
  unsigned long timeNow = millis();
  int deltaT = timeNow - LastTime;
  // check if it's time to run the control loop
  if (deltaT>=PIDtimeInterval){
    // update sensor reading
    sensor2.update();
    current_angle = sensor2.getAngle();

    // calculate error
    float error = target_angle - current_angle;

    //derivative on measurement calculation
    float dError = current_angle - lastInput;

    //OUTPUT
    target_velocity = Kp*error + Kd*dError + Ki*errorSum;
    // target_velocity = (last_target_velocity+target_velocity)/2; //averaging test? 
    //Velocity ramp? 
    if (target_velocity-last_target_velocity>=10){
      target_velocity = last_target_velocity+10;
    }
    else if (target_velocity-last_target_velocity<=-10){
      target_velocity = last_target_velocity-10;
    }

    // enforce velocity limits 
    if (target_velocity>=maxError){
      target_velocity = maxError;
    }
    else if (target_velocity <= minError){
      target_velocity = minError;
    }

    lastInput = current_angle;
    LastTime = timeNow;
    last_target_velocity = target_velocity;
    //prints
    // Serial.print(Kd*dError);
    // Serial.print(", ");
    // Serial.print(current_angle);
    // Serial.print(", ");
    // Serial.println(target_velocity);
    // builtinNeopixel.setPixelColor(0, 50, 50, 50);
    // builtinNeopixel.show();

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
  char B = cmdType[0];
  if (boardAddress==myAddress){
    //do the actions - don't send to bus. 
    if (B =='M'){
      target_angle = desTheta;
      // target_velocity = desTheta;
    }
    else if (B == 'A'){
      HeatInterval = int(1000*desTheta);
      attachA = true;
    }
    else if (B == 'E'){
      sensor2.update();
      Serial.println(sensor2.getAngle());
      Serial.println(sensor2.getCurrentAngle());
    }
    else if (B == 'D'){
      printSensorStatus();
    }
  }
  else if (MAS == 1){
    // if we are master device then send to bus.
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
    if (B=='G'){
        //Move. Command M followed by float (desired angle in radians)
        union byteTofloat {
        char buffer[4];
        float wFloat;
        } byteTofloatW;
        
        Wire.beginTransmission(boardAddress);
        Wire.write('G'); //command letter only move
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
    if (B=='P'){
        //Move. Command M followed by float (desired angle in radians)
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
      else if (B == '\n'){
        
      }
      else {
      //Serial.println("unrecognized");
      //Serial.println(B);
    }
  }
  else{
    Serial.println("Parsing error");
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
    target_angle = FSconvert.FSfloat;
    // if bigger than or less than current angles, simply do not move. 
    if (target_angle >= max_angle){
      sensor2.update();
      current_angle = sensor2.getAngle();
      target_angle = current_angle;
    }
    else if (target_angle <= min_angle){
      sensor2.update();
      current_angle = sensor2.getAngle();
      target_angle = current_angle;
    }
    // target_velocity = FSconvert.FSfloat;
//    actuator.attach(PSERVO);
//    initialized = true;
    
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
  //
  curTheta = sensor2.getAngle();
  //write to union
  FSconvert2.FSfloat2 = curTheta;
  for (int i = 0; i<=3; i++){
    Wire.write(FSconvert2.buffer[i]);
  }
}

void attachActuator(int HeatInterval){
  pinMode(HEATER, OUTPUT);
  unsigned long lastIntTime = millis();
  unsigned long intTimeNow = millis();
  //int HeatInterval = 15000; //1000 is 1 sec.
  //Set max limit. 
  if (HeatInterval > 80000){
    HeatInterval = 80000;
  }
  while ((intTimeNow-lastIntTime) <= HeatInterval){
    // analogWrite(HEATER, 210);// 10V ish, 2.5 A ? 
    digitalWrite(HEATER, HIGH);
    intTimeNow = millis();
  }
  Serial.println("done");
  // analogWrite(HEATER,0);
  digitalWrite(HEATER, LOW);
  pinMode(HEATER, INPUT);
  
}

void printSensorStatus()
{
  AS5047Diagnostics diag = sensor.readDiagnostics();
  Serial.println(F("AS5047 Status:"));
  Serial.print(F("Error: "));
  Serial.println(sensor.isErrorFlag());
  Serial.print(F("COF: "));
  Serial.println(diag.cof);
  float currentAngle = sensor.getCurrentAngle();
  Serial.print(F("Angle: "));
  Serial.println(currentAngle);
  uint16_t magnitude = sensor.readMagnitude();
  Serial.print(F("Magnitude: "));
  Serial.println(magnitude);
  Serial.print(F("Error: "));
  Serial.println(sensor.isErrorFlag());
  Serial.println(diag.magl);
  Serial.println(diag.magh);
  if (sensor.isErrorFlag())
  {
    AS5047Error err = sensor.clearErrorFlag();
    Serial.print(F("Command invalid: "));
    Serial.println(err.commandInvalid);
    Serial.print(F("Framing error: "));
    Serial.println(err.framingError);
    Serial.print(F("Parity error: "));
    Serial.println(err.parityError);
  }
  Serial.println();
}

void printDRV8316Status() {
	DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(2); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(2); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}
