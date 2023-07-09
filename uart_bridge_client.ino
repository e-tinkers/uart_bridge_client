/* This is a UART Bridge Cleint application that communicating with UART Bridge (or SC18IM704)
 * with user-friendly APIs.
 *
 * author:    Henry Cheung
 * date:      16 June, 2023
 */

#include <Arduino.h>
#include "sc18im704_driver.h"

// Configuration
#define USING_SC18IM704    1
#define MCP9808_CONNECTED  0
#define OLED_CONNECTED     0

#define READ_INTERVAL  5000

#if MCP9808_CONNECTED

// MCP9808 Temperature Sensor
#define MCP9808_ADDR   0x18
#define MCP_TEMP       0x05
#define MCP_MFR_ID     0x06
#define MCP_DEV_ID     0x07

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
#endif

#if OLED_CONNECTED
// OLED 128x32 display module
#include "tiny_oled_driver.h"
#define OLED_ADDR      0x3C
#endif
  
void setup() {

  Serial.begin(115200);
#if defined(USING_SC18IM704)
  uart.begin(9600);
#else
  uart.begin(115200);       // SC18IM704 default at 9600, upon reset, it sends "OK"
#endif
  delay(1000);
  
  while(uart.available()) uart.read();
  
  Serial.println(read_chip_id());
  
  // I2C config
//  i2c_set_clock(400000L);              // change i2c clock from default 100kHz to 400kHz

#if USING_SC18IM704
  // blinking SC18IM704 on-board i2c LEDs
  uint8_t cmd[] = { 0xC4, 0x06, 0x11, 0x97, 0x80, 0x00, 0x00, 0xAA };
  i2c_write_array(0x53, cmd, sizeof(cmd), true);
#endif
 
  // gpio_config(15, IO_PUSH_PULL);        // set GPIO pin 0 as OUTPUT Push-Pull
  gpio_config_multiple(0x55AA);            // set GPIO pin 0 - 3 as OUPTUT Push_Pulll
  
}

void loop() {

  static uint8_t i{0};
  
  gpio_write(i, 0);  // LED i OFF
  delay(500);
  gpio_write(i, 1);  // LED i ON
  delay(500);
  i = (i + 1) % 4;   // rotating between LED 0 to 3

#if MCP9808_CONNECTED
    print_mcp_device_info();
    print_mcp_temperature_reading();
#endif
  
}
