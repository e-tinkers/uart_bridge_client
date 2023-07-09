#ifndef __TINY_OLED_H__
#define __TINY_OLED_H__

/*
 * Tiny OLED driver for 128 x 32 monochrome display using SC18IM704 UART protocol
 * reference: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 * author:    Henry Cheung
 * date:      16 June, 2023
 */


#include <string.h>
#include <stdlib.h>
#include "sc18im704_driver.h"
#include "fonts.h"

// display function mode
#define OLED_COMMAND_MODE        0x00
#define OLED_DATA_MODE           0x40

// Registers
#define OLED_REG_SET_COL         0x21
#define OLED_REG_SET_PAGE        0x22

#define OLED_INIT_LEN            16

uint8_t init_config[OLED_INIT_LEN] = {
  OLED_COMMAND_MODE,
  0xA8, 0x1F, // Set multiplex for 32 rows
  0x8D, 0x14, // Charge pump
  0x20, 0x01, // vertical addressing
  0xA1,       // 0xA0/0xA1 flip horizontally
  0xC8,       // 0xC0/0xC8 flip vertically
  0xDA, 0x02, // Set comp ins
  0xD9, 0xF1, // Set pre charge
  0xDB, 0x40, // Set vcom deselect
  0xAF        // Display on
};

static uint8_t scale = 1;


// strentch a byte into a 16-bit word, based on 
// Henry S. Warran Jr. Hacker's Delight (2rd edition) p. 139-141
uint16_t _stretch (uint8_t x) {
  
  uint16_t d = (uint16_t) x;        // d = 00000000 abcdefgh
  d = (d & 0xF0) << 4 | (d & 0x0F); // d = 0000abcd 0000efgh
  d = (d << 2 | d) & 0x3333;        // d = 00ab00cd 00ef00gh
  d = (d << 1 | d) & 0x5555;        // d = 0a0b0c0d 0e0f0g0h
  return d | d << 1;                // d = aabbccdd eeffgghh
  
}

void set_font_size(uint8_t new_size) {
  scale = new_size;
}

void init_display(uint8_t addr) {

  i2c_write_array(addr, init_config, OLED_INIT_LEN);
  
}


void clear_display(uint8_t addr) {
  // Set column address range 0 - 127, page address range 0 - 3
  uint8_t params[] = {OLED_COMMAND_MODE, OLED_REG_SET_COL, 0, 127, OLED_REG_SET_PAGE, 0, 3};
  i2c_write_array(addr, params, sizeof(params)) ;

  // Write the data in 32 blocks of 16-byte as SSD1306 has an I2C buffer of 16-byte
  uint8_t data[17]; 
  for (int i = 0 ; i < 32; i++) {
    memset(data, 0, sizeof(data));
    data[0] = OLED_DATA_MODE;
    i2c_write_array(addr, data, sizeof(data));
    delay(5);
  }
}


void display_on_off(uint8_t addr, uint8_t display_state) {

  uint8_t cmd[] = {OLED_COMMAND_MODE, display_state};
  i2c_write_array(addr, cmd, sizeof(cmd));

}


// Print a character; line = 0 to 3; column = 0 to 131
void print_char(uint8_t addr, uint8_t c, uint8_t line, uint8_t column) {

  uint8_t data[] = {
    OLED_COMMAND_MODE,
    OLED_REG_SET_COL, column, (uint8_t) (column + (scale * 6)), 
    OLED_REG_SET_PAGE, line, (uint8_t) (line + scale - 1)
  };
  i2c_write_array(addr, data, sizeof(data));
  
  uint8_t col0 = fontTable[c - ' '][0];
  uint16_t col0L = _stretch(col0);
  uint16_t col0R = col0L;

  memset(data, 0, 7);
  data[0] = OLED_DATA_MODE;
  
  for (uint8_t col=1; col<6; col++) {  // skipt col 0, leave it as blank
    uint8_t col1 = fontTable[c - ' '][col];
    uint16_t col1L = _stretch(col1);
    uint16_t col1R = col1L;
    
    if (scale == 1) {
      data[col] = col0;
    }
    else {
      // smooth a font
      for (int i=6; i>=0; i--) {
        for (int j=1; j<3; j++) {
          if (((col0 >> i & 0b11) == (3-j)) && ((col1 >> i & 0b11) == j)) {
            col0R = col0R | 1 << ((i * 2) + j);
            col1L = col1L | 1 << ((i * 2) + 3 - j);
          }
        }
      }
      uint8_t data[] = { OLED_DATA_MODE, (uint8_t) (col0L & 0xFF), (uint8_t) (col0L >> 8), (uint8_t) (col0R & 0xFF), (uint8_t) (col0R >> 8) };
      i2c_write_array(addr, data, sizeof(data));
      col0L = col1L; 
      col0R = col1R;
    }
    col0 = col1;
  }
  
  i2c_write_array(addr, data, sizeof(data));

}


void print_str(uint8_t addr, const char* str, uint8_t line, uint8_t col) {

  for (uint8_t i=0; i<strlen(str); i++) {
    print_char(addr, str[i], line, col);
    col = col + scale * 6;
    delay(10);
  }

}

#endif
