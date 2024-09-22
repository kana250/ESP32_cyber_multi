#include <Arduino.h>
#include "BLEDevice.h"
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"

#define RX_PIN 3
#define TX_PIN 1

#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;
unsigned long previousMillis = 0;
unsigned long startTime;
float period = 4000;
float maxSpeed = 10;
float speedRef = 0.0;

uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

static float pos = 0.0;

// BLE settings
BLECharacteristic *pSpeedCharacteristic;
BLEAdvertising *pAdvertising;
bool deviceConnected = false;

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890AB"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-BA0987654321"

// Callback for when the client connects or disconnects
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Client connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Client disconnected");
    // Restart advertising when the client disconnects
    pAdvertising->start();
    Serial.println("Advertising restarted");
  }
};

// Callback for when the client writes to the characteristic
class SpeedCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
      float receivedValue = atof(value.c_str());
      Serial.println("New received value: " + String(receivedValue));

      // Check if the received value is 9999 to initialize CAN
      if (receivedValue == 9999)
      {
        Serial.println("Initializing CAN bus...");
        cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
        cybergear.init_motor(MODE_SPEED);
        cybergear.enable_motor(); // Turn on the motor
        cybergear.zero_pos();
        cybergear.set_position_ref(0.0); // Set initial rotor position
        driver_installed = true;
        Serial.println("CAN bus initialized.");
      }
      else
      {
        // Otherwise, update the speedRef
        speedRef = receivedValue;
        if (driver_installed)
        {
          // Immediately apply the new speedRef to the motor if CAN is initialized
          cybergear.set_speed_ref(speedRef);
        }
      }
    }
  }
};

void setup()
{
  USBSerial.begin();
  delay(100);

  // BLE Initialization with device name
  BLEDevice::init("CyberGear"); // Set the BLE device name
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks for connect/disconnect

  // BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // BLE Characteristic
  pSpeedCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);
  pSpeedCharacteristic->setCallbacks(new SpeedCharacteristicCallbacks());
  pSpeedCharacteristic->setValue(String(speedRef).c_str()); // Initial value

  // Start BLE service
  pService->start();

  // Advertising setup
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Functions that control advertising behavior
  pAdvertising->setMaxPreferred(0x12); // Functions that control advertising behavior
  pAdvertising->start();               // Start advertising
  Serial.println("Advertising started");

  // Optimize BLE connection parameters for reduced latency
  pAdvertising->setMinInterval(20); // 20ms
  pAdvertising->setMaxInterval(40); // 40ms
}

static void handle_rx_message(twai_message_t &message)
{
  if (((message.identifier & 0xFF00) >> 8) == CYBERGEAR_CAN_ID)
  {
    cybergear.process_message(message);
  }
}

static void check_alerts()
{
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  if (alerts_triggered & TWAI_ALERT_ERR_PASS)
  {
    USBSerial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
  {
    USBSerial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    USBSerial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED)
  {
    USBSerial.println("Alert: The Transmission failed.");
    USBSerial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    USBSerial.printf("TX error: %d\t", twai_status.tx_error_counter);
    USBSerial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_DATA)
  {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK)
    {
      handle_rx_message(message);
    }
  }

  XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
  // float now_pos = cybergear_status.position;
  // USBSerial.println("POS" + String(cybergear_status.position) + " V" + String(cybergear_status.speed) + " T" + String(cybergear_status.torque) + " temp" + String(cybergear_status.temperature));

  USBSerial.print(">position: ");
  USBSerial.println(String(cybergear_status.position));

  USBSerial.print(">speed: ");
  USBSerial.println(String(cybergear_status.speed));

  USBSerial.print(">torque: ");
  USBSerial.println(String(cybergear_status.torque));

  USBSerial.print(">temperature: ");
  USBSerial.println(String(cybergear_status.temperature));
}

void loop()
{
  if (!driver_installed)
  {
    delay(1000); // If CAN is not initialized, wait
    return;
  }

  // Handle CAN communication and alerts
  check_alerts();

  // Directly set the speedRef to the motor
  cybergear.set_speed_ref(speedRef);
}