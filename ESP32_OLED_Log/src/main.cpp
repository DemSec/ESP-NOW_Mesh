#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "ssd1306.h"
#include "nano_engine.h"
#include "Lea.h"

void setup()
{
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1331_96x64_spi_init(22, 5, 21);
  ssd1306_setMode(LCD_MODE_NORMAL);
  ssd1306_fillScreen8(0x00);
  ssd1306_drawBitmap16(0, 0, 18, 31, Lea);
}

void loop()
{
  
}