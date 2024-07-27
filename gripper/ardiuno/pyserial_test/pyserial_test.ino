#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;







// Serial read stuff
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;

// Define params
#define vacuum_on 1
#define vacuum_off 2
#define fingers_engaged 3
#define fingers_disengaged 4
#define multiplexer_engaged 5
#define multiplexer_disengaged 6


void setup() {
  Serial.begin(9600);
  while (!Serial);
  clearInputBuffer();

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  delay(100);
}



void loop() {
  recvWithStartEndMarker();
  parseCommands();
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
}

void vacuumOff(){
  Serial.println("Arduino: turning vacuum off");
}

void engageFingers(){
  Serial.println("Arduino: engaging fingers");
}

void disengageFingers(){
  Serial.println("Arduino: disengaging fingers");
}

void engageMultiplexer(){
  Serial.println("Arduino: engage multiplexer");
}

void disengageMultiplexer(){
  Serial.println("Arduino: disengage multiplexer");
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
