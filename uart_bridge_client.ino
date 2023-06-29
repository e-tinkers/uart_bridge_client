/* This is a UART Bridge Cleint application that communicating with UART Bridge (or SC18IM704)
 * with user-friendly APIs.
 *
 * author:    Henry Cheung
 * date:      16 June, 2023
 */

#include <Arduino.h>
#include "sc18im704_driver.h"
#include "tiny_oled_driver.h"

#define READ_INTERVAL  5000
#define LED            0     // GPIO0 connect to a LED with a 1k resistor to GND
#define BUTTON         1     // GPIO1 connect to a push button with a 1k resistor to GND

// MCP9808 Temperature Sensor
#define MCP9808_ADDR   0x18
#define MCP_TEMP       0x05
#define MCP_MFR_ID     0x06
#define MCP_DEV_ID     0x07

// OLED 128x32 display module
#define OLED_ADDR      0x3C

unsigned long start_time{0};

void print_mcp_device_info() {
  // write to MCP9808 register MCP_MFR_ID, do not send stop
  i2c_send(MCP9808_ADDR, MCP_MFR_ID, false);  
  uint8_t id[2]{0};
  i2c_receive(MCP9808_ADDR, id, 2);

  // Write to MCP9808 register MCP_DEV_ID, send stop
  i2c_send(MCP9808_ADDR, MCP_DEV_ID, true);   // send stop condition this time
  uint8_t dev_id[2]{0};
  i2c_receive(MCP9808_ADDR, dev_id, 2);       // device_id should be 0x04rr rr as revision number
  Serial.printf("ManufactureID: %2x, DeviceID: %2X, Revision: %2X\n", id[1], dev_id[0], dev_id[1]);
}

void print_mcp_temperature_reading() {
  i2c_send(MCP9808_ADDR, MCP_TEMP, false);  
  delay(250);    // This delay is needed for conversion time, see MCP9808 datasheet
  uint8_t temp_raw[2];
  i2c_receive(MCP9808_ADDR, temp_raw, 2);

  // if sign bit is set, convert the two's compliment to negative value
  uint16_t temp = (temp_raw[0] & 0x1f) << 8 | temp_raw[1];
  if (temp & 0x1000) {
    temp = -( (~temp & 0xfff) + 1 );
  }
  Serial.printf("Temperature: %.1f C\n", temp / 16.0);
}

void setup() {

  Serial.begin(115200);
  uart.begin(115200);       // SC18IM704 default at 9600, upon reset, it sends "OK"
  delay(200);
  
  Serial.printf("\nPress RESET on UART-Bridge to start\n");
  while (true) {
    if(uart.available()) {
      Serial.println(uart.readStringUntil('\n'));
      break;
    }
  }

  // I2C config
  i2c_set_clock(400000L);              // change i2c clock from default 100kHz to 400kHz

  start_time = millis();
}

void loop() {

  if (millis() - start_time > READ_INTERVAL) {
    print_mcp_device_info();

    print_mcp_temperature_reading();
 
    start_time = millis();
  }

  yield();
  
}
