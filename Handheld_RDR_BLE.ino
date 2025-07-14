#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "display.h"

#define RED_INDICATOR 25
#define BLUE_25PERC_INDICATOR 26
#define BLUE_50PERC_INDICATOR 27
#define BLUE_75PERC_INDICATOR 14
#define BLUE_100PERC_INDICATOR 13
#define BUZZER_PIN 23
#define TP4056_STBY_PIN 19
#define TP4056_CHRG_PIN 18
#define BATT_ADC_PIN 4
#define BATTERY_ENABLE_PIN 2

// Battery Parameters
#define MAX_ANALOG_VALUE 1722  // Analog value corresponding to 4.1V
#define MIN_BATTERY_VOLTAGE 2.88
#define MAX_BATTERY_VOLTAGE 4.02
#define NUM_READINGS 10  // Number of readings for the moving average

// Timing Parameters
#define LED_TOGGLE_INTERVAL 500   // 1 second
#define BATTERY_READ_INTERVAL 50  // 50 milliseconds
#define STANDBY_THRESHOLD 60000   // 1 minute
#define PRINT_INTERVAL 2000       // 2 seconds
#define READER_READ_INTERVAL 50   // 5 milliseconds

// Initialize HardwareSerial on UART2 for RFID communication
HardwareSerial RFIDSerial(2);  // Use UART2 for RFID communication

// Create Display instance
Display display;

// BLE UUIDs
#define SERVICE_UUID "52454152-4C59-5445-4348-5056544C5444"         // REARLYTECHPVTLTD
#define CHARACTERISTIC_UUID "52454152-4C59-4848-5244-4E4F54494659"  // REARLYHHRDNOTIFY

int readings[NUM_READINGS];  // Array to store the readings
int readIndex = 0;           // Index of the current reading
int total = 0;               // Total of the readings
int average = 0;             // Average of the readings

unsigned long previousMillis = 0;         // Store last time LED was updated
unsigned long previousBatteryMillis = 0;  // Store last time battery voltage was read
unsigned long stdbyLowStartTime = 0;      // Store the time when STDBY_PIN first went LOW
unsigned long previousPrintMillis = 0;    // Store last time we printed to Serial

float batteryVoltage = 0.0;  // Global variable to store battery voltage

unsigned long lastLowBatteryToggle = 0;
bool lowBatteryLedState = false;
bool isBooting = true;
unsigned long bootStartTime = 0;
unsigned long lastLedSequenceTime = 0;
int currentLedInSequence = 0;
const unsigned long LED_SEQUENCE_INTERVAL = 50;  // 50 milliseconds

// Add these global variables
bool stdbyState = false;
bool chrgState = false;

// Timers for LED toggling
unsigned long lastBlueLedToggle = 0;
unsigned long lastRedLedToggle = 0;
bool redLedState = false;
bool blue25PercLedState = false;
bool blue50PercLedState = false;
bool blue75PercLedState = false;
bool blue100PercLedState = false;

int chrgLedSequence = 0;

// Constants and variables
const int EPC_LENGTH = 12;         // EPC length in bytes
byte lastEPC[EPC_LENGTH] = { 0 };  // Stores the last EPC for comparison
bool isFirstEPC = true;            // Flag to check if this is the first EPC read

// Define the multiple inventory command frame (without checksum and end byte)
byte multipleInventoryCommand[] = {
  0xBB, 0x00, 0x27, 0x00, 0x03, 0x22, 0x27, 0x10
};
byte checksum = 0;
byte endByte = 0x7E;  // End byte for the frame

// Stop multiple inventory command frame
byte stopInventoryCommand[] = {
  0xBB, 0x00, 0x28, 0x00, 0x00, 0x28, 0x7E
};

// Define expected response lengths
const int EPC_RESPONSE_LENGTH = 26;   // EPC frame length
const int ERROR_RESPONSE_LENGTH = 8;  // Error frame length

// BLE Server and Characteristic
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

class BatteryMonitor {
private:
  static const int SAMPLE_SIZE = 20;
  const float VOLTAGE_THRESHOLD = 0.02;
  const float MIN_VALID_VOLTAGE = 2.9;  // Updated to 2.9V as requested
  const float MAX_VALID_VOLTAGE = 4.3;
  const int CHARGING_CHANGE_THRESHOLD = 5;


  float voltageReadings[SAMPLE_SIZE];
  bool chargingReadings[SAMPLE_SIZE];
  int index = 0;
  int sampleCount = 0;
  bool lastBatteryState = false;
  unsigned long lastStateChangeTime = 0;

public:
  const unsigned long DEBOUNCE_TIME = 2500;  // 5 seconds in milliseconds
  BatteryMonitor() {
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      voltageReadings[i] = 0.0;
      chargingReadings[i] = false;
    }
  }

  bool isBatteryPresent(float voltage, bool isCharging) {
    unsigned long currentTime = millis();

    // Update readings
    voltageReadings[index] = voltage;
    chargingReadings[index] = isCharging;
    index = (index + 1) % SAMPLE_SIZE;
    if (sampleCount < SAMPLE_SIZE) sampleCount++;

    if (sampleCount < SAMPLE_SIZE) return true;  // Not enough samples yet

    // Check for rapid voltage fluctuations
    float maxVoltage = voltageReadings[0];
    float minVoltage = voltageReadings[0];
    for (int i = 1; i < SAMPLE_SIZE; i++) {
      maxVoltage = max(maxVoltage, voltageReadings[i]);
      minVoltage = min(minVoltage, voltageReadings[i]);
    }
    bool rapidFluctuation = (maxVoltage - minVoltage) > VOLTAGE_THRESHOLD;

    // Check for inconsistent charging status
    int chargingChanges = 0;
    for (int i = 1; i < SAMPLE_SIZE; i++) {
      if (chargingReadings[i] != chargingReadings[i - 1]) {
        chargingChanges++;
      }
    }
    bool inconsistentCharging = chargingChanges > CHARGING_CHANGE_THRESHOLD;

    // Check if voltage is outside valid range
    bool invalidVoltage = voltage < MIN_VALID_VOLTAGE || voltage > MAX_VALID_VOLTAGE;

    // Determine current battery state
    bool currentBatteryState = !(rapidFluctuation || inconsistentCharging || invalidVoltage);

    // Apply debounce when transitioning from not present to present
    if (currentBatteryState && !lastBatteryState) {
      if (currentTime - lastStateChangeTime >= DEBOUNCE_TIME) {
        lastBatteryState = true;
        lastStateChangeTime = currentTime;
      } else {
        currentBatteryState = false;  // Keep it as not present until debounce time passes
      }
    } else if (!currentBatteryState && lastBatteryState) {
      lastBatteryState = false;
      lastStateChangeTime = currentTime;
    }

    return currentBatteryState;
  }

  bool isBatteryLow(float voltage) {
    return voltage < MIN_VALID_VOLTAGE;
  }
};

// In your main code:
BatteryMonitor batteryMonitor;

class MyBLEServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("Device connected.");
    display.setBluetoothConnected(true);
    display.update();
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("Device disconnected.");
    display.setBluetoothConnected(false);
    display.update();
    BLEDevice::startAdvertising();
  }
};

//<<<<<<< HEAD
void setup() {
  // Initialize Serial for debugging, RFIDSerial for RFID module, and BLE
  Serial.begin(115200);
  RFIDSerial.begin(115200, SERIAL_8N1, 16, 17);  // RX on GPIO16, TX on GPIO17

  // Initialize display
  display.begin();
  //display.setBatteryPercentage(100);  // Set initial battery level
  display.setBluetoothConnected(false);  // Set initial BLE state
  display.setChargingStatus(false);
  //display.setDeviceID(1234);  // Set your device ID
  display.update();  // Update the display

  // Setup the buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure buzzer is off initially

  pinMode(BATTERY_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_ENABLE_PIN, HIGH);  // MOSFET off at boot

  // Initialize BLE
  BLEDevice::init("REARLY_HH_RDR");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyBLEServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  // Add descriptor for write response
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("BLE server setup complete.");

  Serial.println("Stopping any previous inventory process...");
  // Send the stop multiple inventory command
  RFIDSerial.write(stopInventoryCommand, sizeof(stopInventoryCommand));
  delay(100);  // Allow time for the stop command to process

  Serial.println("Starting multiple inventory...");
  // Calculate and append the checksum to the multiple inventory command
  checksum = calculateChecksum(multipleInventoryCommand, sizeof(multipleInventoryCommand));

  // Send the multiple inventory command with checksum and end byte
  RFIDSerial.write(multipleInventoryCommand, sizeof(multipleInventoryCommand));
  RFIDSerial.write(checksum);
  RFIDSerial.write(endByte);

  delay(100);  // Allow time for processing
}
void printStatus(bool batteryPresent, bool batteryLow) {
  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, Charging: ");
  Serial.print(chrgState ? "Yes" : "No");
  Serial.print(", Standby: ");
  Serial.print(stdbyState ? "Yes" : "No");
  Serial.print(", Battery Present: ");
  Serial.print(batteryPresent ? "Yes" : "No");
  Serial.print(", Battery Low: ");
  Serial.println(batteryLow ? "Yes" : "No");
}

void chargingBatteryControl(unsigned long currentMillis) {
  // Read and update global state variables
  stdbyState = digitalRead(TP4056_STBY_PIN) == LOW;
  chrgState = digitalRead(TP4056_CHRG_PIN) == LOW;

  bool batteryPresent = batteryMonitor.isBatteryPresent(batteryVoltage, chrgState);
  bool batteryLow = batteryMonitor.isBatteryLow(batteryVoltage);

  if (!batteryPresent) {
    handleNoBatteryLedSequence(currentMillis);
  } else if (batteryLow) {
    if (currentMillis - lastRedLedToggle >= LED_TOGGLE_INTERVAL) {
      lastRedLedToggle = currentMillis;
      digitalWrite(RED_INDICATOR, redLedState);
      digitalWrite(BLUE_25PERC_INDICATOR, LOW);
      digitalWrite(BLUE_50PERC_INDICATOR, LOW);
      digitalWrite(BLUE_75PERC_INDICATOR, LOW);
      digitalWrite(BLUE_100PERC_INDICATOR, LOW);
      redLedState = !redLedState;
    }
  } else {
    if (stdbyState) {
      digitalWrite(RED_INDICATOR, HIGH);
      digitalWrite(BLUE_25PERC_INDICATOR, HIGH);
      digitalWrite(BLUE_50PERC_INDICATOR, HIGH);
      digitalWrite(BLUE_75PERC_INDICATOR, HIGH);
      digitalWrite(BLUE_100PERC_INDICATOR, HIGH);
    } else if (chrgState) {
      display.setChargingStatus(true);
      //Serial.println("no issues");
      display.update();
      if (currentMillis - lastBlueLedToggle >= LED_TOGGLE_INTERVAL) {
        lastBlueLedToggle = currentMillis;
        switch (chrgLedSequence) {
          case 0:  // zeroPerc
            redLedState = LOW;
            blue25PercLedState = LOW;
            blue50PercLedState = LOW;
            blue75PercLedState = LOW;
            blue100PercLedState = LOW;
            break;

          case 1:  // twentyPerc
            redLedState = HIGH;
            blue25PercLedState = LOW;
            blue50PercLedState = LOW;
            blue75PercLedState = LOW;
            blue100PercLedState = LOW;
            break;

          case 2:  // fortyPerc
            redLedState = HIGH;
            blue25PercLedState = HIGH;
            blue50PercLedState = LOW;
            blue75PercLedState = LOW;
            blue100PercLedState = LOW;
            break;

          case 3:  // sixtyPerc
            redLedState = HIGH;
            blue25PercLedState = HIGH;
            blue50PercLedState = HIGH;
            blue75PercLedState = LOW;
            blue100PercLedState = LOW;
            break;

          case 4:  // eightyPerc
            redLedState = HIGH;
            blue25PercLedState = HIGH;
            blue50PercLedState = HIGH;
            blue75PercLedState = HIGH;
            blue100PercLedState = LOW;
            break;

          case 5:  // hundredPerc
            redLedState = HIGH;
            blue25PercLedState = HIGH;
            blue50PercLedState = HIGH;
            blue75PercLedState = HIGH;
            blue100PercLedState = HIGH;
            break;
        }
        chrgLedSequence = (chrgLedSequence + 1) % 6;
      }
      digitalWrite(RED_INDICATOR, redLedState);
      digitalWrite(BLUE_25PERC_INDICATOR, blue25PercLedState);
      digitalWrite(BLUE_50PERC_INDICATOR, blue50PercLedState);
      digitalWrite(BLUE_75PERC_INDICATOR, blue75PercLedState);
      digitalWrite(BLUE_100PERC_INDICATOR, blue100PercLedState);
    } else {
      if (batteryVoltage >= MAX_BATTERY_VOLTAGE) {
        redLedState = HIGH;
        blue25PercLedState = HIGH;
        blue50PercLedState = HIGH;
        blue75PercLedState = HIGH;
        blue100PercLedState = HIGH;
      } else if (batteryVoltage <= MIN_BATTERY_VOLTAGE) {
        if (currentMillis - lastRedLedToggle >= LED_TOGGLE_INTERVAL) {
          lastRedLedToggle = currentMillis;
          redLedState = !redLedState;
          blue25PercLedState = LOW;
          blue50PercLedState = LOW;
          blue75PercLedState = LOW;
          blue100PercLedState = LOW;
        }
      } else {
        int proportion = ((batteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100;
        display.setBatteryPercentage(proportion);
        display.update();
        redLedState = HIGH;
        blue25PercLedState = (proportion >= 20) ? HIGH : LOW;
        blue50PercLedState = (proportion >= 40) ? HIGH : LOW;
        blue75PercLedState = (proportion >= 60) ? HIGH : LOW;
        blue100PercLedState = (proportion >= 80) ? HIGH : LOW;
      }
      digitalWrite(RED_INDICATOR, redLedState);
      digitalWrite(BLUE_25PERC_INDICATOR, blue25PercLedState);
      digitalWrite(BLUE_50PERC_INDICATOR, blue50PercLedState);
      digitalWrite(BLUE_75PERC_INDICATOR, blue75PercLedState);
      digitalWrite(BLUE_100PERC_INDICATOR, blue100PercLedState);
    }
  }
  // Always update battery voltage reading
  if (currentMillis - previousBatteryMillis >= BATTERY_READ_INTERVAL) {
    previousBatteryMillis = currentMillis;

    digitalWrite(BATTERY_ENABLE_PIN, LOW);
    delayMicroseconds(100);

    int batteryAnalogValue = analogRead(33);
    // Serial.println(batteryAnalogValue);
    digitalWrite(BATTERY_ENABLE_PIN, HIGH);

    total = total - readings[readIndex];
    readings[readIndex] = batteryAnalogValue;
    total = total + readings[readIndex];
    readIndex = (readIndex + 1) % NUM_READINGS;

    average = total / NUM_READINGS;
    // ESP32 ADC resolution is 12-bit (0-4095)
    float vOut = (average * 3.3) / 4095.0;
    // Voltage divider formula: Vin = Vout * (R1 + R2) / R2

    batteryVoltage = average * 0.00278;
    // batteryVoltage = vOut * (10000.0 + 5000.0) / 5000.0;
  }

  // Print status every 2 seconds
  if (currentMillis - previousPrintMillis >= PRINT_INTERVAL) {
    previousPrintMillis = currentMillis;
    printStatus(batteryMonitor.isBatteryPresent(batteryVoltage, chrgState), batteryMonitor.isBatteryLow(batteryVoltage));
    Serial.println(analogRead(33));
  }
}

void handleNoBatteryLedSequence(unsigned long currentMillis) {
  if (currentMillis - lastLedSequenceTime >= LED_SEQUENCE_INTERVAL) {
    lastLedSequenceTime = currentMillis;
    digitalWrite(RED_INDICATOR, redLedState);
    digitalWrite(BLUE_25PERC_INDICATOR, LOW);
    digitalWrite(BLUE_50PERC_INDICATOR, blue50PercLedState);
    digitalWrite(BLUE_75PERC_INDICATOR, LOW);
    digitalWrite(BLUE_100PERC_INDICATOR, blue100PercLedState);
    redLedState = !redLedState;
    blue50PercLedState = !blue50PercLedState;
    blue100PercLedState = !blue100PercLedState;
  }
}

void readerControl() {
  if (RFIDSerial.available() >= ERROR_RESPONSE_LENGTH) {
    int availableBytes = RFIDSerial.available();
    byte response[availableBytes];

    // Read all available bytes
    RFIDSerial.readBytes(response, availableBytes);

    // Iterate through received data
    for (int i = 0; i < availableBytes; i++) {
      if (availableBytes - i >= EPC_RESPONSE_LENGTH && response[i] == 0xBB && response[i + 1] == 0x02) {
        // EPC frame detected, extract EPC
        byte currentEPC[EPC_LENGTH];
        for (int j = 0; j < EPC_LENGTH; j++) {
          currentEPC[j] = response[i + 8 + j];  // EPC data starts at index 8
        }

        // Print and buzz for new EPC
        if (isFirstEPC || !compareEPC(currentEPC, lastEPC)) {
          String epcString = "";
          for (int j = 0; j < EPC_LENGTH; j++) {
            if (currentEPC[j] < 0x10) epcString += "0";  // Leading zero for single-digit hex
            epcString += String(currentEPC[j], HEX);     // Append as a single, space-free string
            // epcString += (char)currentEPC[j]; // Append as a single, space-free string
          }
          uint8_t byte1 = currentEPC[EPC_LENGTH - 2];
          uint8_t byte2 = currentEPC[EPC_LENGTH - 1];

          int deviceID = (byte1 << 8) | byte2;
          // Extract relevant nibbles
          uint8_t nibble4 = (byte1 >> 4) & 0x0F;  // 4th hex digit
          uint8_t nibble3 = byte1 & 0x0F;         // 3rd hex digit
          uint8_t nibble2 = (byte2 >> 4) & 0x0F;  // 2nd hex digit

          int result;
          if (nibble4 == 0 && nibble3 == 0 && nibble2 == 0) {
            result = 1;
          } else if (nibble4 == 0 && nibble3 == 0) {
            result = 2;
          } else if (nibble4 == 0) {
            result = 3;
          } else {
            result = 4;
          }

          display.setNumBuffer(result);
          // Pass to display
          display.setDeviceID(deviceID);
          display.update();

          // Print EPC to Serial
          Serial.println("EPC: " + epcString);

          // Write EPC to BLE Characteristic
          if (deviceConnected) {
            pCharacteristic->setValue(currentEPC, 12);
            pCharacteristic->notify();
            Serial.println("Data written to client: " + epcString);
          }

          // Activate buzzer for 100ms
          digitalWrite(BUZZER_PIN, HIGH);
          delay(100);  // Buzzer on for 100ms
          digitalWrite(BUZZER_PIN, LOW);

          // Update lastEPC with the current EPC
          memcpy(lastEPC, currentEPC, EPC_LENGTH);
          isFirstEPC = false;
        }

        // Move to the next frame
        i += EPC_RESPONSE_LENGTH;

      } else if (availableBytes - i >= ERROR_RESPONSE_LENGTH && response[i] == 0xBB && response[i + 1] == 0x01) {
        // Error or no tag found frame
        i += ERROR_RESPONSE_LENGTH;  // Move to the next frame

      } else {
        // Skip unexpected data
        i++;
      }
    }
  }
}


void loop() {
  static unsigned long lastReaderTime = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastReaderTime > READER_READ_INTERVAL) {
    readerControl();
    lastReaderTime = currentMillis;
  }
  chargingBatteryControl(currentMillis);
  // delay(50); // Adjust delay as needed for processing speed
}

// Compare two EPC arrays
bool compareEPC(byte *epc1, byte *epc2) {
  for (int i = 0; i < EPC_LENGTH; i++) {
    if (epc1[i] != epc2[i]) return false;
  }
  return true;
}

// Calculate checksum for the command array
byte calculateChecksum(byte *buffer, int length) {
  byte sum = 0;
  for (int i = 1; i < length; i++) {  // Sum from Type (index 1) to last byte
    sum += buffer[i];
  }
  return sum;
}