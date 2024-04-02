# cgol_i2c_oled_arduino
Conway's Game of Life on ssd1306 I2C with Arduino Uno

OLED screen SSD1306 128х64 0.96"

It produces 17 iterations/second, and most of it for display communication. Without displaying it's 58 iterations/second. 

The alghorithm is from https://forum.arduino.cc/t/conways-game-of-life-with-128x64-graphic-lcd-update-also-with-128x64-spi-oled/210637/25

For I2C i used this library https://github.com/lexus2k/ssd1306
