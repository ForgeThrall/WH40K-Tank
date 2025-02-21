#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

#define LOGGING

// Unofficial standard uuids for UART over BLE. Non-standard magic numbers, yay
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// TODO: actually pick the pin numbers
#define SERVO_PIN_LEFT  15
#define SERVO_PIN_RIGHT 16

int servoPinTest = 14;
int minUs = 1000;
int maxUs = 2000;

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 

// Partially reverse engineered protocol used by Dabble. Sufficient for this use-case
const std::vector<int> CONNECTED_IDENTIFIER           = {0xFF, 0x00, 0x03, 0x00, 0x00, 0x00};
const std::vector<int> DIGITAL_CONTROLLER_IDENTIFIER  = {0xFF, 0x01, 0x01, 0x01, 0x02};
const std::vector<int> JOYSTICK_CONTROLLER_IDENTIFIER = {0xFF, 0x01, 0x02, 0x01, 0x02};
const std::vector<int> JOYSTICK_GYROSCOPE_IDENTIFIER  = {0xFF, 0x01, 0x03, 0x01, 0x02};

// What ratio should the motors run at
// Given an angle in 15 degree increments (24 total in a circle)
// 0 is fully right
const std::vector<float> LEFT_DRIVE_ANGLE_LOOKUP  = { 1,   1,   1,   1,   1,   1,
                                                      1,  .6,  .2, -.2, -.6,  -1,
                                                     -1,  -1,  -1,  -1,  -1,  -1,
                                                     -1, -.6, -.2,  .2,  .6,   1};

const std::vector<float> RIGHT_DRIVE_ANGLE_LOOKUP = {-1, -.6, -.2,  .2,  .6,   1,
                                                      1,   1,   1,   1,   1,   1,
                                                      1,  .6,  .2, -.2, -.6,  -1,
                                                     -1,  -1,  -1,  -1,  -1,  -1};

Servo servoLeft;
Servo servoRight;

// TODO add a speed selector. Tie to start/select to control d-pad speed?
// TODO these can be #defines instead
#define DPAD_U 0b00000100
#define DPAD_R 0b00001000
#define DPAD_D 0b00010000
#define DPAD_L 0b00100000
#define JOYSTICK_MASK 13

int dpadState = 0;
int joystickState = 0;


boolean hasIdentifier(std::vector<int> check, std::vector<int> identifier) {
  if(check.size() < identifier.size()) return false;
  for(int i = 0; i < identifier.size(); i++){
    if(check[i] != identifier[i])
    return false;
  }
  return true;
}

void updateServos() {
  // TODO do stuff with dpad and joyStick. dPad should take priority for the sake of gyro support

  // TODO keep track of previous input to allow for speed change while holding a d-pad input

  // D-pad takes precedence over analog input
  // D-pad is 0x00LDRU00) Left Down Right Up
  // TODO not actually 1 and -1 pwm
  if(joystickState != 0) {
    if(dpadState & DPAD_U) {
      servoLeft.write(1);
      servoRight.write(1);
      return;
    }
    if(dpadState & DPAD_R) {
      servoLeft.write(1);
      servoRight.write(-1);
      return;
    }
    if(dpadState & DPAD_D) {
      servoLeft.write(-1);
      servoRight.write(-1);
      return;
    }
    if(dpadState & DPAD_L) {
      servoLeft.write(-1);
      servoRight.write(1);
      return;
    }
  }
}


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    #ifdef LOGGING
    Serial.println("Bluetooth connected");
    #endif
  };

  void onDisconnect(BLEServer* pServer) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    #ifdef LOGGING
    Serial.println("Resume advertising");
    #endif
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    std::vector<int> message(rxValue.begin(), rxValue.end());

    #ifdef LOGGING
    if(hasIdentifier(message, CONNECTED_IDENTIFIER)) {
      Serial.println("Dabble connected");
      return;
    }
    #endif

    if(hasIdentifier(message, DIGITAL_CONTROLLER_IDENTIFIER)) {
      #ifdef LOGGING
      Serial.print("Digital controller input - ");
      for (int i = 5; i < message.size(); i++){
        // Serial.printf("%02X ", message[i]);
        Serial.printf(BYTE_TO_BINARY_PATTERN " ", BYTE_TO_BINARY(message[i]));
      }
      Serial.println();
      #endif

      // TODO report controller status

      // TEMP testing proof of hardware
      if(message[5] & DPAD_U) {
        servoLeft.write(180);
        servoRight.write(180);
      }
      else if(message[5] & DPAD_L) {
        servoLeft.write(0);
        servoRight.write(180);
      }
      else if(message[5] & DPAD_R) {
        servoLeft.write(180);
        servoRight.write(0);
      }
      else if(message[5] & DPAD_D) {
        servoLeft.write(0);
        servoRight.write(0);
      }
      else {
        servoLeft.write(90);
        servoRight.write(90);
      }

      return;
    }

    if(hasIdentifier(message, JOYSTICK_CONTROLLER_IDENTIFIER)) {
      #ifdef LOGGING
      Serial.print("Joystick controller input - ");
      // Raw state
      Serial.printf(BYTE_TO_BINARY_PATTERN " ", BYTE_TO_BINARY(message[5]));
      // Translated angle and magnitude
      Serial.printf(" %03d %03d", (message[6]>>3) * 15, message[6] & 0b00000111);
      Serial.println();
      #endif
      
      // TODO report controller status

      return;
    }

    if(hasIdentifier(message, JOYSTICK_GYROSCOPE_IDENTIFIER)) {
      #ifdef LOGGING
      Serial.print("Gyro controller input - ");
      Serial.printf(BYTE_TO_BINARY_PATTERN " ", BYTE_TO_BINARY(message[5]));
      Serial.printf(" %03d %03d", (message[6]>>3) * 15, message[6] & 0b00000111);
      Serial.println();
      #endif
      
      // TODO report controller status

      return;
    }

    // Log whatever else
    #ifdef LOGGING
    if (rxValue.length() > 0) {
      Serial.printf("Received Value (length %d): ", rxValue.length());
      for (int i = 0; i < rxValue.length(); i++){
        Serial.printf("%02X ", rxValue[i], HEX);
      }
      Serial.println();
    }
    #endif
  }
};


void setup() {
  #ifdef LOGGING
  Serial.begin(115200);
  #endif

  // Create the BLE Device
  BLEDevice::init("WH40K Tank"); 

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  BLECharacteristic * pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_RX,
                    BLECharacteristic::PROPERTY_WRITE
                  );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  #ifdef LOGGING
  Serial.println("Ready to connect..."); 
  #endif

  // TODO Don't these need initialized before being attached?...
  servoLeft.attach(servoPinTest, minUs, maxUs);
  servoRight.attach(SERVO_PIN_RIGHT);

  servoLeft.setPeriodHertz(50);
}

void loop() { }
