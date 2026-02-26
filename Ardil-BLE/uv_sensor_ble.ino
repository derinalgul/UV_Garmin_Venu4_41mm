/**
 * GarminUV Sensor - ESP32 BLE Firmware
 *
 * This firmware runs on an ESP32-DevKitC-32E and broadcasts UV sensor data
 * via Bluetooth Low Energy (BLE) to a Garmin watch.
 *
 * Hardware:
 *   - ESP32-WROOM-32 (DevKitC-32E)
 *   - LTR390 UV Sensor (I2C) - currently using mock data
 *   - 3.7V LiPo battery with TP4056 (future)
 *
 * BLE Service UUID: 00001234-0000-1000-8000-00805f9b34fb
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================================
// BLE UUID Definitions
// ============================================================================

// Custom service for UV sensor data
#define SERVICE_UUID              "00001234-0000-1000-8000-00805f9b34fb"

// UV Index characteristic - current UV index value (0-11+ scale)
#define UV_INDEX_CHAR_UUID        "00001235-0000-1000-8000-00805f9b34fb"

// Cumulative Dose characteristic - accumulated UV exposure in MED units
#define CUMULATIVE_DOSE_CHAR_UUID "00001236-0000-1000-8000-00805f9b34fb"

// Battery Level characteristic - standard BLE battery service UUID
#define BATTERY_LEVEL_CHAR_UUID   "00002a19-0000-1000-8000-00805f9b34fb"

// ============================================================================
// Configuration Constants
// ============================================================================

#define DEVICE_NAME           "GarminUV-Sensor"  // BLE advertised name
#define SAMPLE_INTERVAL_MS    15000              // Sample every 15 seconds
#define DOSE_COEFFICIENT      0.025              // MED per UVI per second
#define SAMPLE_INTERVAL_SEC   15                 // Interval in seconds for dose calc

// ============================================================================
// Global Variables
// ============================================================================

// BLE objects
BLEServer* pServer = nullptr;
BLECharacteristic* pUVIndexChar = nullptr;
BLECharacteristic* pCumulativeDoseChar = nullptr;
BLECharacteristic* pBatteryLevelChar = nullptr;

// Connection state
bool deviceConnected = false;
bool oldDeviceConnected = false;

// UV data
float currentUVIndex = 0.0;
float cumulativeDose = 0.0;

// Timing
unsigned long lastSampleTime = 0;

// Mock data state (for sine wave generation)
float mockTimeCounter = 0.0;

// ============================================================================
// BLE Server Callbacks
// Handles connection and disconnection events
// ============================================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Client disconnected");
  }
};

// ============================================================================
// Mock UV Sensor Functions
// Simulates UV sensor readings with a sine wave pattern (UVI 3-8)
// Replace with actual LTR390 readings when hardware is available
// ============================================================================

float readMockUVIndex() {
  // Generate sine wave between 3 and 8
  // Amplitude = 2.5, offset = 5.5, so range is 5.5 +/- 2.5 = [3, 8]
  // Period of ~60 samples (15 minutes at 15-second intervals)
  float uvIndex = 5.5 + 2.5 * sin(mockTimeCounter);
  mockTimeCounter += 0.1;  // Increment for next reading

  return uvIndex;
}

// ============================================================================
// Real UV Sensor Functions (for future use with LTR390)
// Uncomment and modify when hardware is available
// ============================================================================

/*
#include <Adafruit_LTR390.h>

Adafruit_LTR390 ltr390 = Adafruit_LTR390();

bool initUVSensor() {
  if (!ltr390.begin()) {
    Serial.println("[SENSOR] Could not find LTR390 sensor!");
    return false;
  }

  // Configure sensor for UV mode
  ltr390.setMode(LTR390_MODE_UVS);
  ltr390.setGain(LTR390_GAIN_3);
  ltr390.setResolution(LTR390_RESOLUTION_18BIT);
  ltr390.setThresholds(100, 1000);

  Serial.println("[SENSOR] LTR390 initialized");
  return true;
}

float readRealUVIndex() {
  if (ltr390.newDataAvailable()) {
    // LTR390 provides raw UV counts, convert to UV Index
    // Conversion factor depends on sensor configuration
    // Typical: UVI = raw_uv / 2300 (adjust based on calibration)
    float rawUV = ltr390.readUVS();
    float uvIndex = rawUV / 2300.0;
    return uvIndex;
  }
  return currentUVIndex;  // Return last value if no new data
}
*/

// ============================================================================
// Battery Level Functions
// Returns mock battery percentage (replace with ADC reading for real battery)
// ============================================================================

uint8_t readBatteryLevel() {
  // TODO: Implement real battery voltage reading via ADC
  // For now, return a mock value
  //
  // Real implementation would:
  // 1. Read ADC pin connected to battery voltage divider
  // 2. Convert ADC value to voltage
  // 3. Map voltage (3.0V-4.2V for LiPo) to percentage (0-100%)

  return 85;  // Mock: 85% battery
}

// ============================================================================
// BLE Notification Functions
// Send updated values to connected client
// ============================================================================

void notifyUVIndex(float uvIndex) {
  if (deviceConnected && pUVIndexChar != nullptr) {
    // Set the characteristic value (4 bytes for float)
    pUVIndexChar->setValue(uvIndex);
    // Send notification to subscribed client
    pUVIndexChar->notify();
    Serial.printf("[BLE] Notified UV Index: %.2f\n", uvIndex);
  }
}

void notifyCumulativeDose(float dose) {
  if (deviceConnected && pCumulativeDoseChar != nullptr) {
    // Set the characteristic value (4 bytes for float)
    pCumulativeDoseChar->setValue(dose);
    // Send notification to subscribed client
    pCumulativeDoseChar->notify();
    Serial.printf("[BLE] Notified Cumulative Dose: %.4f MED\n", dose);
  }
}

// ============================================================================
// Cumulative Dose Calculation
// Formula: dose += (UVI × 0.025) × interval_seconds
// Units: Minimal Erythemal Dose (MED) - measure of UV exposure
// ============================================================================

void updateCumulativeDose(float uvIndex, int intervalSeconds) {
  // Calculate dose increment based on UV index and time exposed
  // 0.025 MED per UVI per second is a simplified model
  // In reality, this depends on skin type, angle of exposure, etc.
  float doseIncrement = uvIndex * DOSE_COEFFICIENT * intervalSeconds;
  cumulativeDose += doseIncrement;
}

// ============================================================================
// BLE Initialization
// Sets up BLE server, service, and characteristics
// ============================================================================

void initBLE() {
  Serial.println("[BLE] Initializing...");

  // Initialize BLE device with name
  BLEDevice::init(DEVICE_NAME);

  // Create BLE server and set connection callbacks
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the UV sensor service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // -------------------------------------------------------------------------
  // UV Index Characteristic
  // Properties: Read (client can request value), Notify (server can push updates)
  // -------------------------------------------------------------------------
  pUVIndexChar = pService->createCharacteristic(
    UV_INDEX_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  // Add Client Characteristic Configuration Descriptor (CCCD)
  // This allows clients to subscribe/unsubscribe to notifications
  pUVIndexChar->addDescriptor(new BLE2902());

  // -------------------------------------------------------------------------
  // Cumulative Dose Characteristic
  // Properties: Read, Notify
  // -------------------------------------------------------------------------
  pCumulativeDoseChar = pService->createCharacteristic(
    CUMULATIVE_DOSE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCumulativeDoseChar->addDescriptor(new BLE2902());

  // -------------------------------------------------------------------------
  // Battery Level Characteristic
  // Properties: Read only (standard battery service behavior)
  // -------------------------------------------------------------------------
  pBatteryLevelChar = pService->createCharacteristic(
    BATTERY_LEVEL_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ
  );

  // Initialize characteristic values
  float initialUV = 0.0;
  float initialDose = 0.0;
  uint8_t initialBattery = readBatteryLevel();

  pUVIndexChar->setValue(initialUV);
  pCumulativeDoseChar->setValue(initialDose);
  pBatteryLevelChar->setValue(&initialBattery, 1);

  // Start the service
  pService->start();

  // -------------------------------------------------------------------------
  // Start Advertising
  // Makes the device discoverable to Garmin watch
  // -------------------------------------------------------------------------
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // These settings help with iPhone compatibility
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising started as '" DEVICE_NAME "'");
  Serial.println("[BLE] Service UUID: " SERVICE_UUID);
}

// ============================================================================
// Arduino Setup Function
// Runs once at startup
// ============================================================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);  // Allow serial to stabilize

  Serial.println();
  Serial.println("========================================");
  Serial.println("  GarminUV Sensor - BLE Firmware v1.0");
  Serial.println("========================================");
  Serial.println();

  // Initialize BLE
  initBLE();

  // Initialize mock data timer
  lastSampleTime = millis();

  Serial.println("[MAIN] Setup complete. Waiting for connection...");
  Serial.printf("[MAIN] Sampling interval: %d ms\n", SAMPLE_INTERVAL_MS);
}

// ============================================================================
// Arduino Main Loop
// Runs continuously after setup
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // -------------------------------------------------------------------------
  // Sample UV sensor at defined interval (every 15 seconds)
  // -------------------------------------------------------------------------
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = currentTime;

    // Read UV index (mock data for now)
    currentUVIndex = readMockUVIndex();

    // Update cumulative dose based on current UV exposure
    updateCumulativeDose(currentUVIndex, SAMPLE_INTERVAL_SEC);

    // Log current values to serial
    Serial.println("----------------------------------------");
    Serial.printf("[DATA] UV Index: %.2f\n", currentUVIndex);
    Serial.printf("[DATA] Cumulative Dose: %.4f MED\n", cumulativeDose);
    Serial.printf("[DATA] Battery: %d%%\n", readBatteryLevel());

    // Send notifications if a device is connected
    if (deviceConnected) {
      notifyUVIndex(currentUVIndex);
      notifyCumulativeDose(cumulativeDose);

      // Update battery level (read-only, no notification)
      uint8_t battery = readBatteryLevel();
      pBatteryLevelChar->setValue(&battery, 1);
    }
  }

  // -------------------------------------------------------------------------
  // Handle reconnection
  // When a client disconnects, restart advertising to allow new connections
  // -------------------------------------------------------------------------
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);  // Brief delay before restarting advertising
    pServer->startAdvertising();
    Serial.println("[BLE] Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }

  // Track connection state changes
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Small delay to prevent tight loop
  delay(10);
}
