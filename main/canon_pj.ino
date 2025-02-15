#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

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
 

const uint8_t DXL_ID_LR = 6;
const uint8_t DXL_ID_UD = 0;
const float DXL_PROTOCOL_VERSION = 1.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
  Serial.begin(9600);
  if (!amg.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(9600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_LR);
  dxl.ping(DXL_ID_UD);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_LR);
  dxl.setOperatingMode(DXL_ID_LR, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_LR);
  dxl.torqueOff(DXL_ID_UD);
  dxl.setOperatingMode(DXL_ID_UD, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_UD);
}

void loop(){
  //불 체크
  int temp = 28;
  float pixels[64];
  amg.readPixels(pixels);
  for (int i = 0; i < 64; i++) {
    if(pixels[i] >= temp){
      //2사분면
      if(i==0 || i==1 || i==2 || i==3 || i==8 || i==9 || i==10 || i==11 || i==16 || i==17 || i==18 || i==19 || i==24 || i==25 || i==26 || i==27 ){
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[4]>= temp || pixels[12]>= temp || pixels[20]>= temp || pixels[28]>= temp || pixels[36]>= temp || pixels[44]>= temp || pixels[52]>= temp || pixels[60]>= temp){
            dxl.setGoalVelocity(DXL_ID_LR, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_LR, 10.0, UNIT_PERCENT);//left
        }
        Serial.println("mode UD");
        delay(1000);
        
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[32]>= temp || pixels[33]>= temp || pixels[34]>= temp || pixels[35]>= temp || pixels[36]>= temp || pixels[37]>= temp || pixels[38]>= temp || pixels[39]>= temp){
            dxl.setGoalVelocity(DXL_ID_UD, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_UD, -45.0, UNIT_PERCENT); //up
        }
        Serial.println("fire check");
        digitalWrite(12,HIGH);
        delay(2500);
        digitalWrite(12,LOW);
        delay(1000);
      }
      //1사분면
      if(i==4 || i==5 || i==6 || i==7 || i==12 || i==13 || i==14 || i==15 || i==20 || i==21 || i==22 || i==23 || i==28 || i==29 || i==30 || i==31 ){
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[4]>= temp || pixels[12]>= temp || pixels[20]>= temp || pixels[28]>= temp || pixels[36]>= temp || pixels[44]>= temp || pixels[52]>= temp || pixels[60]>= temp){
            dxl.setGoalVelocity(DXL_ID_LR, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_LR, -90.0, UNIT_PERCENT);//right
        }
        Serial.println("mode UD");
        delay(1000);
        
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[32]>= temp || pixels[33]>= temp || pixels[34]>= temp || pixels[35]>= temp || pixels[36]>= temp || pixels[37]>= temp || pixels[38]>= temp || pixels[39]>= temp){
            dxl.setGoalVelocity(DXL_ID_UD, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_UD, -45.0, UNIT_PERCENT); //up
        }
        Serial.println("fire check");
        digitalWrite(12,HIGH);
        delay(2500);
        digitalWrite(12,LOW);
        delay(1000);
      }
      //3사분면
      if(i==32 || i==33 || i==34 || i==35 || i==40 || i==41 || i==42 || i==43 || i==48 || i==49 || i==50 || i==51 || i==56 || i==57 || i==58 || i==59 ){
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[4]>= temp || pixels[12]>= temp || pixels[20]>= temp || pixels[28]>= temp || pixels[36]>= temp || pixels[44]>= temp || pixels[52]>= temp || pixels[60]>= temp){
            dxl.setGoalVelocity(DXL_ID_LR, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_LR, 10.0, UNIT_PERCENT);//left
        }
        Serial.println("mode UD");
        delay(1000);
        
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[32]>= temp || pixels[33]>= temp || pixels[34]>= temp || pixels[35]>= temp || pixels[36]>= temp || pixels[37]>= temp || pixels[38]>= temp || pixels[39]>= temp){
            dxl.setGoalVelocity(DXL_ID_UD, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_UD, 40.0, UNIT_PERCENT); //down
        }
        Serial.println("fire check");
        digitalWrite(12,HIGH);
        delay(2500);
        digitalWrite(12,LOW);
        delay(1000);
      }
      //4사분면
      if(i==36 || i==37 || i==38 || i==39 || i==44 || i==45 || i==46 || i==47 || i==52 || i==53 || i==54 || i==55 || i==60 || i==61 || i==62 || i==63 ){
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[4]>= temp || pixels[12]>= temp || pixels[20]>= temp || pixels[28]>= temp || pixels[36]>= temp || pixels[44]>= temp || pixels[52]>= temp || pixels[60]>= temp){
            dxl.setGoalVelocity(DXL_ID_LR, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_LR, -90.0, UNIT_PERCENT);//right
        }
        Serial.println("mode UD");
        delay(1000);
        
        while(true){
          float pixels[64];
          amg.readPixels(pixels);
          for(int i=1; i<=64; i++){
            Serial.print(pixels[i-1]);
            Serial.print(", ");
            if( i%8 == 0 ) Serial.println();
          }
          if(pixels[32]>= temp || pixels[33]>= temp || pixels[34]>= temp || pixels[35]>= temp || pixels[36]>= temp || pixels[37]>= temp || pixels[38]>= temp || pixels[39]>= temp){
            dxl.setGoalVelocity(DXL_ID_UD, 0);
            break;
          }
          dxl.setGoalVelocity(DXL_ID_UD, 40.0, UNIT_PERCENT); //down
        }
        Serial.println("fire check");
        digitalWrite(12,HIGH);
        delay(2500);
        digitalWrite(12,LOW);
        delay(1000);
      }
    }
  }
}
