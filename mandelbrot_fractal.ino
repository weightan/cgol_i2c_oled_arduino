
#include <Wire.h>
#include "ssd1306.h"
#include "intf/ssd1306_interface.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define s_ssd1306_invertByte 0x00000000
#define iter_max 200

void setup() {
  Serial.begin(9600);

  delay(20);

  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();

  outputMatrix();
}

void loop() {
  unsigned long start = millis();
  for (int i=0; i<10; i++) {
    outputMatrix();
  }

  Serial.print(F("Gens/s:"));
  Serial.println(10000/(millis() - start));
}




void outputMatrix() {
  //uint8_t s_ssd1306_invertByte = 0x00000000;

  ssd1306_lcd.set_block(0, 0, 128);

  for (uint16_t col = 0; col < 8; col++) {

    for (uint16_t row = 0; row <= 127; row++) {
      ssd1306_lcd.send_pixels1(s_ssd1306_invertByte^gen_mb_byte(row, col));
      //spi.write(Matrix[row].b[col]);
      //delay(2);
    }	
    ssd1306_lcd.next_page();
  }
  ssd1306_intf.stop();
}


byte gen_mb_byte(uint16_t x, uint16_t y){
  byte out = B00000000;
  float a,b,ca,cb,aa,bb;
  byte n;

  for (uint16_t i = 0; i < 8; i++) {
    a = -2.5*(0.5 - float(x)/64)*0.9 - 1.5;
    b = -2.5*(0.5 - float(y*8+i)/64)*0.9;
    ca = a;
    cb = b;
    n = 0;
    
    while (n < iter_max) {
      aa = a * a - b * b;
      bb = 2 * a * b;
      a = aa + ca;
      b = bb + cb;

      if (abs(a + b) > 50) {
        break;
      }
      n++;
    }
    
    if (n == iter_max) {
      out = out | (1 << i);
      continue;
    }
    
    if(byte(random(iter_max)) + 6 < n){
      out = out | (1 << i);
    }
  }
  return ~out;
}



