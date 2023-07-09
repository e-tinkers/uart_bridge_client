#ifndef _SC18IM704_DRIVER_H
#define _SC18IM704_DRIVER_H

/*
 * This is a library for UART Bridge Client for communicating with 
 * home-brewed UART-Bridge or SC18IM704 chip
 * reference: https://www.nxp.com/docs/en/data-sheet/SC18IM704.pdf
 *            https://www.nxp.com/docs/en/user-manual/UM11664.pdf
 * author:    Henry Cheung
 * date:      7 June, 2023
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial uart(D6, D7);  // rx, tx D6 and D7 defined as in Wemos D1 Mini


// UART Commands
#define CMD_START         'S' // 0x53
#define CMD_STOP          'P' // 0x50
#define CMD_REG_READ      'R' // 0x52
#define CMD_REG_WRITE     'W' // 0x57
#define CMD_GPIO_READ     'I' // 0x49
#define CMD_GPIO_WRITE    'O' // 0x4f
#define CMD_POWER_DOWN    'Z' // 0x5A
#define CMD_READ_ID       'V' // 0x56

// SC18IM704 Registers
#define REG_BAUD_RATE_0    0  // default 0xF0 i.e. Baud rate = 7372800 /(16 + 0x02f0) = 9600
#define REG_BAUD_RATE_1    1  // default 0x02
#define REG_PORT_CONF1     2  // default B01010101 (all as input)
#define REG_PORT_CONF2     3  // default B01010101 (all as input)
#define REG_IO_STATE       4
#define REG_RESERVED       5
#define REG_I2C_BUS_ADDR   6  // default 0x26
#define REG_I2C_CLK_L      7  // default 0x13 i.e. 99kHz, for 375kHz, use 0x05
#define REG_I2C_CLK_H      8  // default 0x00
#define REG_I2C_TIMEOUT    9  // default 0x66
#define REG_I2C_STATE      10 // default I2C_OK, See I2C Status

// GPIO Configuration
#define IO_INPUT           1
#define IO_PUSH_PULL       2
#define IO_OPEN_DRAIN      3

// I2C Clock Configuration
#define I2C_CLK_375KHZ     0x05  // 0x05 mimimum value
#define I2C_CLCK_99KHZ     0x13  // 19 in decimal, default

// I2C Status
#define SC_I2C_OK            0xF0
#define SC_I2C_NACK_ON_ADDR  0xF1
#define SC_I2C_NACK_ON_DATA  0xF2
#define SC_I2C_TIME_OUT      0xF8

#define BUF_SIZE          256

uint8_t rx_buf[BUF_SIZE]{0};

// GPIO Config
// 'W' + REG_PORT_CONFIG1 + pin3-0 + REG_PORT_CONFIG2 + pin7-4 + 'P'
// all gpio_pins are configure as INPUT by default, therefore there is no need to
// explicitly config a gpio_pin as input unless the gpio_pin has been config 

int8_t gpio_config(uint8_t gpio_pin, uint8_t gpio_mode) {
  
  if ((gpio_pin > 7) || (gpio_mode > IO_OPEN_DRAIN)) return -1;

  uint16_t gpio_conf{0};
  
  //TO_DO: instead of using default hardcoded 0x5555, should use the last_config value
  uint16_t target_mode = gpio_mode << (gpio_pin * 2);
  uint16_t mask = ~(0x03 << (gpio_pin * 2));
  uint16_t bits_config = 0x5555 & mask;
  gpio_conf = bits_config | target_mode;

  uint8_t cmd[] = {
    CMD_REG_WRITE, REG_PORT_CONF1, (uint8_t) (gpio_conf & 0x00ff),
    REG_PORT_CONF2, (uint8_t) (gpio_conf >> 8), CMD_STOP
  };
  uart.write(cmd, sizeof(cmd));
  
  return true;
  
}

int8_t gpio_config_multiple(uint16_t gpio_mode) {
  uint8_t cmd[] = {
    CMD_REG_WRITE, REG_PORT_CONF1, (uint8_t) (gpio_mode & 0x00ff),
    REG_PORT_CONF2, (uint8_t) (gpio_mode >> 8), CMD_STOP
  };
  uart.write(cmd, sizeof(cmd));
  
  return true;
}

// GPIO Read All ('I' + 'P')
// return states of all GPIO pins, -1=invalid pin
// For SC18IM704, if the IO pin is set to IO_OPEN_DRAIN, it does not seem to read the correct states,
// only when IO_OPEN_DRAIN, it reports the correct states
int8_t gpio_read_all() {

  uint8_t cmd[] = { CMD_GPIO_READ, CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
  while (!uart.available());
  uart.read();
  return (uint8_t) uart.read();

}


// GPIO Read
// return state of one particular gpio pin, 1=HIGH, 0=LOW, -1=invalid pin
int8_t gpio_read(uint8_t gpio_pin) {
  
  if (gpio_pin > 7) return -1;
  
  uint8_t data = gpio_read_all();
  return (data & (1 << gpio_pin)) >> gpio_pin;
  
}


// GPIO Write ('O' + Data + 'P')
int8_t gpio_write(uint8_t gpio_pin, uint8_t state) {
  
  if ((gpio_pin > 7) && (state > 1)) return -1;
  
  uint8_t pin_states = gpio_read_all();
  uint8_t mask = ~(0xff & (state << gpio_pin));
  
  uint8_t cmd[] = {CMD_GPIO_WRITE, (uint8_t) (mask & pin_states), CMD_STOP };
  uart.write(cmd, sizeof(cmd));

  return 1;
  
}


// Power-down ('Z' + 0x5a + 0xa5 + 'P')
// Sending any character via UART would wake-up the chip but the char will be ignored
void power_down() {

  uint8_t cmd[] = { CMD_POWER_DOWN, 0x5A, 0xA5, CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
}


// Read Chip ID ('V' + 'P')
// resp: "SC18IM704 1.0.1\0" (16-char NULL-terminated string)
String read_chip_id() {

  uint8_t cmd[] = { CMD_READ_ID, CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
  while (!uart.available()) { yield(); };
  String resp = uart.readStringUntil('\0');
  uart.read();
  return resp;
  
}


// Set UART Baud Rate (Default upon reset is 9600 baud)
// 'W' + REG_BAUD_RATE_0 +  setting_l + REG_BAUD_RATE_1 + setting_h  + 'P'
// baud_rate = 7372800 /(16 + REG_BAUD_RATE_SETTING)
// REG_BAUD_RATE_SETTING = (7372800 / baud_rate) - 16;
bool set_baud_rate(uint32_t baud_rate) {

  uint8_t setting_h, setting_l;
  
  switch (baud_rate) {
    case 9600L:   setting_h = 0x02; setting_l = 0xf0; break;
    case 14400L:  setting_h = 0x01; setting_l = 0xf0; break;
    case 19200L:  setting_h = 0x01; setting_l = 0x70; break;
    case 28800L:  setting_h = 0x00; setting_l = 0xf0; break;
    case 38400L:  setting_h = 0x00; setting_l = 0xb0; break;
    case 57600L:  setting_h = 0x00; setting_l = 0x70; break;
    case 76800L:  setting_h = 0x00; setting_l = 0x50; break;
    case 115200L: setting_h = 0x00; setting_l = 0x30; break;
    case 230400L: setting_h = 0x00; setting_l = 0x10; break;
    case 460800L: setting_h = 0x00; setting_l = 0x00; break;  // SC18IM704 supports upto 460800 baud
    default: return false;
  }

  uint8_t cmd[] = { CMD_REG_WRITE, REG_BAUD_RATE_0, setting_l, REG_BAUD_RATE_1, setting_h, CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
  return true;
  
}


// Set I2C Bus Address (i.e. the Master Address of the I2C controller iteself)
// the default bus address is 0x26
bool i2c_bus_address(uint8_t bus_addr) {
  
  if (bus_addr > 0x7f) return false;
  
  uint8_t cmd[] = { CMD_REG_WRITE, REG_I2C_BUS_ADDR, (uint8_t)(bus_addr << 1), CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
  return true;
  
}


// Set I2C clock
// 'W' + REG_I2C_CLK_L + I2C_CLCK_VALUE + REG_I2C_CLK_H + 0 + 'P'
bool i2c_set_clock(uint32_t speed) {
  
  if ((speed != 100000L) && (speed != 400000L)) return false;

  uint8_t clk = (speed == 100000L) ? I2C_CLCK_99KHZ : I2C_CLK_375KHZ;

  uint8_t cmd[] = { CMD_REG_WRITE, REG_I2C_CLK_L, clk, REG_I2C_CLK_H, 0, CMD_STOP };
  uart.write(cmd, sizeof(cmd));

  return true;
  
}

// Send data over i2c
void i2c_write(uint8_t address, uint8_t reg, bool send_stop) {

  uint8_t addr = (uint8_t) (address << 1);
  if (send_stop) {
    uint8_t cmd[] = { CMD_START, addr, 1, reg, CMD_STOP };
    uart.write(cmd, sizeof(cmd));
  }
  else {
    uint8_t cmd[] = { CMD_START, addr, 1, reg };
    uart.write(cmd, sizeof(cmd));
  }

}

// i2c write multiple operation
void i2c_write_array(uint8_t address, uint8_t * data, uint8_t len, bool send_stop) {

  const int asize = len + 4;
  uint8_t cmd[asize]{0};

  cmd[0] = CMD_START;
  cmd[1] = (uint8_t) (address << 1);
  cmd[2] = len;
  memcpy(&cmd[3], data, len);
  if (send_stop) {
    cmd[len+3] = CMD_STOP;
    uart.write(cmd, len+4);
  }
  else {
      uart.write(cmd, len+3);
  }

}

// Receive data from i2c
void i2c_read(uint8_t address, uint8_t * buf, uint8_t len) {

  uint8_t read = (uint8_t) ((address << 1) | 0x01);
  uint8_t cmd[] = { CMD_START, read, len, CMD_STOP };
  uart.write(cmd, sizeof(cmd));

  while (uart.available() < len) yield();

  memset(rx_buf, 0, BUF_SIZE);
  int i=0;
  while (uart.available()) {
    rx_buf[i++] = uart.read();
  }

  memcpy(buf, rx_buf, 2);
  
}


// I2C set timeout
// timeout in seconds = (1 + (8 << REG_I2C_CLK_H) | REG_I2C_CLK_L)) / 15000000 * ((1 + t) * 512)
// the least significant bit = 0, timeout disabled, = 1 timeout enabled
// the default value is B01100110, which means timeout is disabled. If it is set to B01100111, it
// would means a timeout of 21ms for 400kHz, and 70ms for 100kHz.
void i2c_set_timeout(uint8_t t) {

  uint8_t cmd[] = { REG_I2C_TIMEOUT, t, CMD_STOP };
  uart.write(cmd, sizeof(cmd));
  
}


// Read i2c_status
uint8_t i2c_status() {
  
  uint8_t cmd[] = { CMD_REG_READ, REG_I2C_STATE, CMD_STOP };
  uart.write(cmd, sizeof(cmd));

  while (uart.available() < 1) yield();

  return uart.read();
  
}

#endif
