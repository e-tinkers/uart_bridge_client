## UART Bridge Client
This repository contains the code as a UART Bridge client that talks to an ESP UART Bridge (the Bridge) or a 
SC18IM704 UART Bridge chip.

The code was used on an ESP8266 module, with SoftwareSerial connect to the ESP UART Bridge. It is designed for testing the Bridge or a SC18IM704 chip, where the bridge/SC18IM704 has a MCP9808 temperature sensor connected via the I2C, and this client communicates with the I2C temperture sensor via the SC18IM704 UART protocol.

The `font.h` and `tiny_oled_driver.h` is used when an 128x32-pixel SSD1306-based OLED display is connected to the 
I2C bus on the Bridge/SC18IM704 chip.

For more detail about SC18IM704 UART protocol, please refer to [SC18IM704 
datasheet](https://www.nxp.com/docs/en/data-sheet/SC18IM704.pdf).


