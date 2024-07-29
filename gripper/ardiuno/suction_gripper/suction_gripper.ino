/*
This script tests all of the sensors and outputs in the electronics for the gripper using the OpenRB-150
board using pyserial commands.

Note for future me:
pressure sensor 1 is on C
pressure sensor 2 is on B
pressure sensor 3 is on A

fingers closed servo positions:
70mm: -2220
90mm: -2045

Jacob Karty
6/11/2024
*/

//libraries
#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_VL53L0X.h>

//pins
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define RESET_PIN -1
#define EOC_PIN   -1
#define VALVE 7
// Define params
#define vacuum_on 1
#define vacuum_off 2
#define fingers_engaged 3
#define fingers_disengaged 4
#define multiplexer_engaged 5
#define multiplexer_disengaged 6

//constants
//OpenRB does not require the DIR control pin.
const int DXL_DIR_PIN = -1;
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
const int PCAADR = 0x70;
const int VALVE_DELAY = 10;
const int PUBLISH_DELAY = 2;

// Serial read stuff
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;

// variables for servo movements
boolean movingUp = false;
boolean currentControlGrasp = false;
const int openPos = 0;
const int closedPos = -1900;
const int closed90mm = -2045;
const int closed70mm = -2200;

boolean muxBool = false; //to multiplex or not to multiplex

using namespace ControlTableItem;   //This namespace is required to use Control table item names

//initializations
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {

  // Dynamixel initialization
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_CURRENT); // Set to current mode
  dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, 100); // set max current to  not damage anything
  dxl.torqueOn(DXL_ID);
  dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);

  // Initialize pressure and distance sensors
  mpr.begin();
  lox.begin();
  lox.startRangeContinuous(10);

  // Initialize VALVE pin as output
  delay(10);
  pinMode(VALVE, OUTPUT);
  delay(10);
  digitalWrite(VALVE, LOW);
  delay(10);

  // Serial initialization
  Serial.begin(9600);
  while (!Serial);
  clearInputBuffer();

  // Run initilization sequence to find zero position
  zeroPosition();

  delay(1000);
}


void zeroPosition() {
  // Move the servo to the zero point by using low current and checking if the motor moves.
  dxl.setGoalCurrent(DXL_ID, 50, UNIT_MILLI_AMPERE);
  uint16_t previous_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  delay(200);
  while(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) > previous_position + 3){
    previous_position = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    delay(100);
  }
  // Once it is at the 0 position, set operating mode to single rotation to clip the
  // position stored in RAM to the same value every time. This value is around 100 deg
  dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  // Then move to initial position
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);
  dxl.setGoalPosition(DXL_ID, openPos, UNIT_DEGREE);
}


void loop() {
  //look for command
  recvWithStartEndMarker();

  //excecute command
  parseCommands();


  //this is for precise control of fingers when grasping
  if (movingUp && dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)<closedPos){
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT); // Set to current mode
    dxl.torqueOn(DXL_ID);

    movingUp = false;
    currentControlGrasp = true;
  }
  if(currentControlGrasp){
    //sets the current to desired current for different sized apples. at 90mm: 125 mA, at 70mm: 75mA. Linear between them
    dxl.setGoalCurrent(DXL_ID, currentFunction(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)), UNIT_MILLI_AMPERE);
  }

  //print data to the serial
  if(muxBool){
    multiplex();
  }
}

void recvWithStartEndMarker() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseCommands() {
  int c = 0;
  int c_idx = 0;
  int t_idx = 0;
  char temp[32];

  if (newData == true) {
    // Convert serial monitor value to int and cast as float
    int len = strlen(receivedChars);
    c = (float) atoi(receivedChars);
    newData = false;

    // Manage the serial input accordingly
    if (c==vacuum_on){
      vacuumOn();
    }
    else if (c==vacuum_off){
      vacuumOff();
    }
    else if (c==fingers_engaged){
      engageFingers();
    }
    else if (c==fingers_disengaged){
      disengageFingers();
    }
    else if (c==multiplexer_engaged){
      engageMultiplexer();
    }
    else if (c==multiplexer_disengaged){
      disengageMultiplexer();
    }
    else{
      Serial.println("Unknown input recieved");
    }
  }
}

void vacuumOn(){
  Serial.println("Arduino: turning vacuum on");
  digitalWrite(VALVE, HIGH);
}

void vacuumOff(){
  Serial.println("Arduino: turning vacuum off");
  digitalWrite(VALVE, LOW);
}

void engageFingers(){
  Serial.println("Arduino: engaging fingers");

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION); // set to position mode
  dxl.torqueOn(DXL_ID);

  //move up
  dxl.setGoalPosition(DXL_ID, closedPos-10, UNIT_DEGREE);

  movingUp = true;
  
}

void disengageFingers(){
  //Disengages the fingers by setting the servo to the open position
  Serial.println("Arduino: disengaging fingers");
  
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);

  //move down
  dxl.setGoalPosition(DXL_ID, openPos, UNIT_DEGREE);
  currentControlGrasp = false;
  
}

void engageMultiplexer(){
  //start publishing data
  Serial.println("Arduino: engage multiplexer");
  muxBool = true;
}

void disengageMultiplexer(){
  //stop publishing data
  Serial.println("Arduino: disengage multiplexer");
  muxBool = false;
}

void testInput(float c){
  Serial.println("recieved input");
}

void sendIntSerial(int x) {
  uint8_t LSB = x;
  uint8_t MSB = x >> 8;
  Serial.write(MSB);
  Serial.write(LSB);
}

void clearInputBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void multiplex() {
  //loop through the multiplexor sensors and print their data. Also print data from the motor

  //variables for sensor measurements
  unsigned long currentMillis;
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t pressure_hPa;
  uint16_t distance;

  for (uint8_t i = 0; i < (3 + 1); i++) {
    // NUM_CUPS +1 is because the i2c multiplexer reads the three pressure sensors AND the
    // time of Flight sensor        

    currentMillis = millis();

    pcaselect(i);
    
    // Measure Pressure in the first three (3) channels 
    if(i<3){
      pressure_hPa = mpr.readPressure();  

      //Print the time in ms just to have an idea of long the sensor takes to measure press.
      Serial.println("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Pressure: " + String(pressure_hPa) + " hPa");     
    }

    // ... and then measure Distance in the fourth channel
    else {
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        distance = measure.RangeMilliMeter;        
      } else {
        distance = 1e4;
      }


      Serial.println("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Distance: " + String(distance) + " mm");          
    }
  }

  //Also print out motor current, position, and velocity
  Serial.println("[Ch4] Present Current: " + String(dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE)) + " mA");
  Serial.println("[Ch5] Present Position: " + String(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)) + " degree");
  Serial.println("[Ch6] Present Velocity: " + String(dxl.getPresentVelocity(DXL_ID, UNIT_RPM)) + " rpm");


}

int currentFunction(int position){
  //helper fuction to determine desired current for different sized apples. For a 90 mm diameter apple, use 125mA. Decrease to 75mA when grasping 70mm apple.
  //this is due to the mechanical advantage of the gripper changing as the gripper closes
  int current = 125 - ((float(closed90mm - position)) / (float(closed90mm - closed70mm))) * 50;
  Serial.println(current);
  if(current>125){
    current=125;
  }
  return -current;

}

/**************************** I2C Multiplexer *********************************/
// Reference: https://learn.adafruit.com/adafruit-pca9546-4-channel-i2c-multiplexer/arduino
void pcaselect(uint8_t i) {
  if (i > 4) return;

  Wire.beginTransmission(PCAADR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
