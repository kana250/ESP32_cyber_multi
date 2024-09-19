#include <Arduino.h>
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

void setup()
{
  USBSerial.begin();
  delay(100);
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

  cybergear.init_motor(MODE_SPEED);
  cybergear.set_limit_speed(20.0f); /* set the maximum speed of the motor */
  cybergear.enable_motor();         /* turn on the motor */
  cybergear.zero_pos();
  cybergear.set_position_ref(0.0); /* set initial rotor position */
  driver_installed = true;
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
  float now_pos = cybergear_status.position;
  USBSerial.println("POS" + String(cybergear_status.position) + " V" + String(cybergear_status.speed) + " T" + String(cybergear_status.torque) + " temp" + String(cybergear_status.temperature));
}

void loop()
{
  if (!driver_installed)
  {
    delay(1000);
    return;
  }
  check_alerts();
  unsigned long currentTime = millis();
  float elapsedTime = currentTime - startTime;

  float sineValue = sin(2 * PI * elapsedTime / period);
  speedRef = maxSpeed * (sineValue + 1.8) / 2;

  cybergear.set_speed_ref(speedRef);
}
