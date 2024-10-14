/* Script for controlling the tandem gripper with an Arduino Zero board 
 *  
 * Alejandro Velasquez
 * velasale@oregonstate.edu
 * 09/27/2024
 *
 * Colaborators
 */

// TODO VALVE
// TODO SENSOR PRESSURE
// TODO SENSOR DISTANCE

// TODO STEPPER
// TODO SERVO

#define Serial SerialUSB    //This trick is meant for the Aduino Zero board

// libraries
#include <Wire.h>
#include <time.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_VL53L0X.h>
#include <Stepper.h>


// stepper pins
const byte ENABLE_PINA = 8;     // pin to enable part A of stepper driver
const byte ENABLE_PINB = 13;    // pin to enable part B of stepper driver
// valve pins
#define VALVE 7                 // pin to turn on/off electric valve
// air-pressure sensor pins
#define RESET_PIN -1
#define EOC_PIN   -1
// parameters
#define vacuum_on 1
#define vacuum_off 2
#define fingers_engaged 3
#define fingers_disengaged 4
#define multiplexer_engaged 5
#define multiplexer_disengaged 6


// constants
const int STEPS_PER_REVOLUTION = 200;   // Adjust according to your motor
const int STEPS = 1500;
const int STEP_SPEED = 15;
/* If L298N driver is used, speed should be within these ranges:
 *   <Accelstepper.h>  450 < x < 1100  units: steps/sec
 *   <Stepper.h>       140 < x < 340   units: rpm
 */
const int CLOSING_SPEED = 240;    
const int CLOSING_SPEED_FAST = 340;
const int OPENING_SPEED = 330;
/* Distances */
const int TOTAL_DISTANCE = 58 * (200/8);   // 58mm * (200 steps / 1rev) * (1rev / 8mm)
const int CLAMP_DISTANCE = 15 * (200/8);
const int INITIAL_DISTANCE = TOTAL_DISTANCE - CLAMP_DISTANCE;
// int target;
const int TOF_CALIB_DISTANCE = 20;        // Calibration of tof sensor

const int PCAADR = 0x70;


// Serial read stuff
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int dataNumber = 0;

boolean muxBool = true; //to multiplex or not to multiplex


// initializations
Stepper gripperStepper(STEPS_PER_REVOLUTION, 9, 10, 11, 12);
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();



void setup() {
  
    // Initialize stepper motor
    gripperStepper.setSpeed(STEP_SPEED);

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
    Serial.begin(115200);
    while (!Serial);
    clearInputBuffer();

    delay(1000);  
}


void loop(){
    //wait for command
    recvWithStartEndMarker();

    //execute command
    parseCommands();

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
  int16_t pressure_hPa;
  int16_t distance;

  for (uint8_t i = 0; i < (3 + 1); i++) {
    // NUM_CUPS +1 is because the i2c multiplexer reads the three pressure sensors AND the
    // time of Flight sensor

    currentMillis = millis();

    pcaselect(i);

    // Measure Pressure in the first three (3) channels
    if(i<3){
      pressure_hPa = mpr.readPressure();

      //Print the time in ms just to have an idea of long the sensor takes to measure press.
      Serial.print("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Pressure: " + String(pressure_hPa) + " hPa");
    }

    // ... and then measure Distance in the fourth channel
    else {
      Serial.println();
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        distance = measure.RangeMilliMeter - TOF_CALIB_DISTANCE;

        if (distance < 0){
          distance = 0;  
        }
        
      } else {
        distance = 1e4;
      }


      Serial.println("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Distance: " + String(distance) + " mm");
    }
  }
}


/**************************** I2C Multiplexer *********************************/
// Reference: https://learn.adafruit.com/adafruit-pca9546-4-channel-i2c-multiplexer/arduino
void pcaselect(uint8_t i) {
  if (i > 4) return;

  Wire.beginTransmission(PCAADR);
  Wire.write(1 << i);
  Wire.endTransmission();
}



void engageFingers(){
  Serial.println("Arduino: engaging fingers");  
  motorSteps(CLOSING_SPEED_FAST, INITIAL_DISTANCE);
  delay(100);  
   motorSteps(CLOSING_SPEED, CLAMP_DISTANCE);
  delay(100);  
}


void disengageFingers(){
  //Disengages the fingers by setting the servo to the open position
  Serial.println("Arduino: disengaging fingers");  
  motorSteps(OPENING_SPEED,-TOTAL_DISTANCE);
  delay(100);
}



void motorSteps(int stp_speed, int stp_distance){    
  digitalWrite(ENABLE_PINA, HIGH);
  digitalWrite(ENABLE_PINB, HIGH);     
   
  gripperStepper.setSpeed(stp_speed);
  gripperStepper.step(stp_distance);
  
  digitalWrite(ENABLE_PINA, LOW);
  digitalWrite(ENABLE_PINB, LOW);      
  delay(100);             
}
