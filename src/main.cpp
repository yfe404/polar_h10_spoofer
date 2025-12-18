/**
 * ESP32 Polar H10 BLE Spoofer
 *
 * Spoofs a Polar H10 heart rate monitor to work with KardiAI/Beatwell app.
 * Implements:
 * - Heart Rate Service (0x180D)
 * - Device Information Service (0x180A)
 * - Battery Service (0x180F)
 * - Polar PMD Service (for ECG streaming)
 *
 * Hardware: ESP32 (any variant with BLE support)
 *
 * IMPORTANT: This is for educational/research purposes only.
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

// Device configuration - mimic Polar H10
#define DEVICE_NAME "Polar H10 0A3BA92B"
#define DEVICE_ID "0A3BA92B"

// Standard BLE Service UUIDs
#define HEART_RATE_SERVICE_UUID        "180D"
#define DEVICE_INFO_SERVICE_UUID       "180A"
#define BATTERY_SERVICE_UUID           "180F"

// Standard BLE Characteristic UUIDs
#define HEART_RATE_MEASUREMENT_UUID    "2A37"
#define BODY_SENSOR_LOCATION_UUID      "2A38"
#define BATTERY_LEVEL_UUID             "2A19"
#define MANUFACTURER_NAME_UUID         "2A29"
#define MODEL_NUMBER_UUID              "2A24"
#define SERIAL_NUMBER_UUID             "2A25"
#define HARDWARE_REVISION_UUID         "2A27"
#define FIRMWARE_REVISION_UUID         "2A26"
#define SOFTWARE_REVISION_UUID         "2A28"
#define SYSTEM_ID_UUID                 "2A23"

// Polar PMD Service UUIDs (proprietary)
#define PMD_SERVICE_UUID               "FB005C80-02E7-F387-1CAD-8ACD2D8DF0C8"
#define PMD_CONTROL_POINT_UUID         "FB005C81-02E7-F387-1CAD-8ACD2D8DF0C8"
#define PMD_DATA_UUID                  "FB005C82-02E7-F387-1CAD-8ACD2D8DF0C8"

// ECG streaming parameters
#define ECG_SAMPLE_RATE 130  // Hz
#define ECG_SAMPLES_PER_PACKET 73  // Samples per BLE packet

BLEServer* pServer = NULL;
BLECharacteristic* pHeartRateMeasurement = NULL;
BLECharacteristic* pPmdControlPoint = NULL;
BLECharacteristic* pPmdData = NULL;
BLECharacteristic* pBatteryLevel = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool ecgStreamingEnabled = false;

// Simulated ECG data (one cardiac cycle at 130Hz, ~1 second)
// Values in microvolts, representing a typical PQRST complex
const int16_t ecgTemplate[] = {
  // Baseline
  0, 5, 10, 5, 0, -5, 0, 5, 0, -5, 0, 5, 10, 5, 0,
  // P wave
  20, 40, 60, 80, 100, 110, 100, 80, 60, 40, 20, 0,
  // PR segment
  0, 0, 5, 0, -5, 0, 5, 0, 0,
  // QRS complex
  -50, -100, -150,  // Q wave
  800, 1000, 1200, 1100, 900, 600, 300, 100,  // R wave
  -200, -300, -250, -150, -50,  // S wave
  // ST segment
  0, 10, 20, 30, 20, 10, 0,
  // T wave
  30, 60, 100, 140, 170, 180, 170, 140, 100, 60, 30, 0,
  // Baseline to next beat
  0, 0, -10, 0, 10, 0, -5, 0, 5, 0, 0, -5, 0, 5, 0, 0,
  0, 0, -10, 0, 10, 0, -5, 0, 5, 0, 0, -5, 0, 5, 0, 0,
  0, 0, -10, 0, 10, 0, -5, 0, 5, 0, 0, -5, 0, 5, 0, 0
};
const int ecgTemplateLength = sizeof(ecgTemplate) / sizeof(ecgTemplate[0]);
int ecgIndex = 0;
uint64_t ecgTimestamp = 0;

// Connection callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      ecgStreamingEnabled = false;
      Serial.println("Device disconnected!");
    }
};

// PMD Control Point callbacks - handles ECG start/stop commands
class PmdControlPointCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("PMD CP READ - returning features");
    }

    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {
        Serial.print("PMD CP received: ");
        for (int i = 0; i < value.length(); i++) {
          Serial.printf("%02X ", value[i]);
        }
        Serial.println();

        uint8_t command = value[0];

        switch (command) {
          case 0x01: // GET_MEASUREMENT_SETTINGS
            {
              Serial.println("GET_MEASUREMENT_SETTINGS for ECG");
              // Response: supported settings for ECG
              // Format: [0x01, 0x00, type, settings...]
              uint8_t response[] = {
                0xF0, 0x01, 0x00,  // Response header
                0x00,  // ECG type
                0x00, 0x01, 0x82, 0x00,  // Sample rate: 130 Hz
                0x01, 0x01, 0x0E, 0x00,  // Resolution: 14 bits
                0x02, 0x01, 0x01, 0x00   // Range: 1 (±3.28mV)
              };
              pPmdControlPoint->setValue(response, sizeof(response));
              pPmdControlPoint->notify();
            }
            break;

          case 0x02: // START_MEASUREMENT
            {
              Serial.println("START ECG MEASUREMENT");
              ecgStreamingEnabled = true;
              ecgTimestamp = esp_timer_get_time() / 1000;  // Current time in ms

              // Response: success
              uint8_t response[] = {0xF0, 0x02, 0x00, 0x00};
              pPmdControlPoint->setValue(response, sizeof(response));
              pPmdControlPoint->notify();
            }
            break;

          case 0x03: // STOP_MEASUREMENT
            {
              Serial.println("STOP ECG MEASUREMENT");
              ecgStreamingEnabled = false;

              // Response: success
              uint8_t response[] = {0xF0, 0x03, 0x00, 0x00};
              pPmdControlPoint->setValue(response, sizeof(response));
              pPmdControlPoint->notify();
            }
            break;

          case 0x04: // GET_SDK_MODE
            {
              Serial.println("GET_SDK_MODE");
              uint8_t response[] = {0xF0, 0x04, 0x00, 0x01};  // SDK mode enabled
              pPmdControlPoint->setValue(response, sizeof(response));
              pPmdControlPoint->notify();
            }
            break;

          default:
            Serial.printf("Unknown PMD command: 0x%02X\n", command);
            // Generic success response
            uint8_t response[] = {0xF0, command, 0x00};
            pPmdControlPoint->setValue(response, sizeof(response));
            pPmdControlPoint->notify();
        }
      }
    }
};

// Security callbacks for pairing
class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() {
    Serial.println("PassKey Request");
    return 123456;
  }

  void onPassKeyNotify(uint32_t pass_key) {
    Serial.printf("PassKey Notify: %d\n", pass_key);
  }

  bool onConfirmPIN(uint32_t pass_key) {
    Serial.printf("Confirm PIN: %d\n", pass_key);
    return true;
  }

  bool onSecurityRequest() {
    Serial.println("Security Request");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) {
    if (auth_cmpl.success) {
      Serial.println("Authentication SUCCESS");
    } else {
      Serial.printf("Authentication FAILED, reason: 0x%x\n", auth_cmpl.fail_reason);
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Polar H10 BLE Spoofer...");

  // Initialize BLE
  BLEDevice::init(DEVICE_NAME);

  // Configure BLE Security - Just Works pairing (no PIN required)
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new MySecurity());

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);  // Enable bonding
  pSecurity->setCapability(ESP_IO_CAP_NONE);  // No IO capability (Just Works)
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create Heart Rate Service
  BLEService *pHeartRateService = pServer->createService(BLEUUID((uint16_t)0x180D));

  pHeartRateMeasurement = pHeartRateService->createCharacteristic(
    BLEUUID((uint16_t)0x2A37),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pHeartRateMeasurement->addDescriptor(new BLE2902());

  // Body Sensor Location: Chest (1)
  BLECharacteristic *pBodySensorLocation = pHeartRateService->createCharacteristic(
    BLEUUID((uint16_t)0x2A38),
    BLECharacteristic::PROPERTY_READ
  );
  uint8_t sensorLocation = 1;  // Chest
  pBodySensorLocation->setValue(&sensorLocation, 1);

  pHeartRateService->start();

  // Create Device Information Service
  BLEService *pDeviceInfoService = pServer->createService(BLEUUID((uint16_t)0x180A));

  BLECharacteristic *pManufacturer = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A29), BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue("Polar Electro Oy");

  BLECharacteristic *pModel = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A24), BLECharacteristic::PROPERTY_READ);
  pModel->setValue("H10");

  BLECharacteristic *pSerial = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A25), BLECharacteristic::PROPERTY_READ);
  pSerial->setValue(DEVICE_ID);

  BLECharacteristic *pHwRev = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A27), BLECharacteristic::PROPERTY_READ);
  pHwRev->setValue("39044024.10");

  BLECharacteristic *pFwRev = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A26), BLECharacteristic::PROPERTY_READ);
  pFwRev->setValue("5.0.0");

  BLECharacteristic *pSwRev = pDeviceInfoService->createCharacteristic(
    BLEUUID((uint16_t)0x2A28), BLECharacteristic::PROPERTY_READ);
  pSwRev->setValue("1.1.5");

  pDeviceInfoService->start();

  // Create Battery Service
  BLEService *pBatteryService = pServer->createService(BLEUUID((uint16_t)0x180F));

  pBatteryLevel = pBatteryService->createCharacteristic(
    BLEUUID((uint16_t)0x2A19),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pBatteryLevel->addDescriptor(new BLE2902());
  uint8_t batteryLevel = 85;
  pBatteryLevel->setValue(&batteryLevel, 1);

  pBatteryService->start();

  // Create Polar PMD Service
  BLEService *pPmdService = pServer->createService(BLEUUID(PMD_SERVICE_UUID));

  // PMD Control Point - for commands AND feature reading
  // SDK reads this characteristic to get supported features
  pPmdControlPoint = pPmdService->createCharacteristic(
    BLEUUID(PMD_CONTROL_POINT_UUID),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
  );
  pPmdControlPoint->setCallbacks(new PmdControlPointCallbacks());
  pPmdControlPoint->addDescriptor(new BLE2902());

  // Set PMD feature data (returned when characteristic is READ)
  // Format: [reserved, features_byte1, features_byte2, ...]
  // features_byte1 bit 0 = ECG, bit 1 = PPG, bit 2 = ACC, bit 3 = PPI
  // For H10 with ECG: data[1] = 0x01 (ECG enabled)
  uint8_t pmdFeatures[] = {0x00, 0x01, 0x00};  // ECG supported
  pPmdControlPoint->setValue(pmdFeatures, sizeof(pmdFeatures));

  // PMD Data - for ECG streaming
  pPmdData = pPmdService->createCharacteristic(
    BLEUUID(PMD_DATA_UUID),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pPmdData->addDescriptor(new BLE2902());

  pPmdService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x180D));  // Heart Rate
  pAdvertising->addServiceUUID(BLEUUID(PMD_SERVICE_UUID));  // PMD
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  // Add Polar manufacturer data (ID 107 = 0x006B)
  // This is REQUIRED for the Polar SDK to recognize the device
  BLEAdvertisementData advData;
  advData.setFlags(0x06);  // General discoverable, BR/EDR not supported
  advData.setCompleteServices(BLEUUID((uint16_t)0x180D));

  // Manufacturer specific data: Polar ID (0x6B 0x00) + device type data
  // Format: 2 bytes manufacturer ID (little-endian) + payload
  char manufacturerData[4];
  manufacturerData[0] = 0x6B;  // Polar manufacturer ID low byte (107)
  manufacturerData[1] = 0x00;  // Polar manufacturer ID high byte
  manufacturerData[2] = 0x72;  // Device type indicator
  manufacturerData[3] = 0x08;  // H10 identifier
  advData.setManufacturerData(std::string(manufacturerData, 4));
  advData.setName(DEVICE_NAME);

  pAdvertising->setAdvertisementData(advData);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started as: " DEVICE_NAME);
  Serial.println("Waiting for connection...");
}

void sendEcgData() {
  if (!ecgStreamingEnabled || !deviceConnected) return;

  // Build ECG data packet
  // Format: [type, timestamp(8 bytes), frame_type, samples...]
  uint8_t packet[200];
  int packetIndex = 0;

  // Measurement type: ECG (0x00)
  packet[packetIndex++] = 0x00;

  // Timestamp (8 bytes, little-endian nanoseconds)
  uint64_t timestamp_ns = ecgTimestamp * 1000000ULL;  // Convert ms to ns
  for (int i = 0; i < 8; i++) {
    packet[packetIndex++] = (timestamp_ns >> (i * 8)) & 0xFF;
  }

  // Frame type (0x00 for raw data)
  packet[packetIndex++] = 0x00;

  // ECG samples (3 bytes each, 24-bit signed, little-endian)
  int samplesToSend = min(ECG_SAMPLES_PER_PACKET, 50);  // Limit packet size
  for (int i = 0; i < samplesToSend; i++) {
    int16_t sample = ecgTemplate[ecgIndex];
    ecgIndex = (ecgIndex + 1) % ecgTemplateLength;

    // Convert to 24-bit (microvolts)
    int32_t sample24 = sample;
    packet[packetIndex++] = sample24 & 0xFF;
    packet[packetIndex++] = (sample24 >> 8) & 0xFF;
    packet[packetIndex++] = (sample24 >> 16) & 0xFF;
  }

  // Send packet
  pPmdData->setValue(packet, packetIndex);
  pPmdData->notify();

  // Update timestamp for next packet
  ecgTimestamp += (samplesToSend * 1000) / ECG_SAMPLE_RATE;  // ms
}

void sendHeartRate() {
  if (!deviceConnected) return;

  // Heart Rate Measurement value
  // Format: [flags, heart_rate, RR_interval (optional)]
  // Flags: 0x00 = HR is uint8, no RR interval
  uint8_t heartRate = 72 + (millis() / 1000) % 10;  // Varies 72-81 BPM
  uint8_t hrValue[] = {0x00, heartRate};
  pHeartRateMeasurement->setValue(hrValue, 2);
  pHeartRateMeasurement->notify();
}

unsigned long lastHeartRateTime = 0;
unsigned long lastEcgTime = 0;

void loop() {
  unsigned long currentTime = millis();

  // Send heart rate every second
  if (currentTime - lastHeartRateTime >= 1000) {
    lastHeartRateTime = currentTime;
    sendHeartRate();
  }

  // Send ECG data at 130 Hz (every ~7.7ms)
  // But we batch samples, so send every ~50ms
  if (ecgStreamingEnabled && (currentTime - lastEcgTime >= 50)) {
    lastEcgTime = currentTime;
    sendEcgData();
  }

  // Handle disconnection/reconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Restarting advertising...");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  delay(1);
}
