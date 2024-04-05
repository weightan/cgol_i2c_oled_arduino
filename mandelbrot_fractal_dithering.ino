
#include <Wire.h>
#include "ssd1306.h"
#include "intf/ssd1306_interface.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define iter_max 100

const uint8_t bayer8[]= {0,32,8,40,2,34,10,42,48,16,56,24,
50,18,58,26,12,44,4,36,14,46,6,38,60,28,52,20,62,30,54,22,3,
35,11,43,1,33,9,41,51,19,59,27,49,17,57,25,15,47,7,39,13,45,
5,37,63,31,55,23,61,29,53,21};


const uint8_t s_ssd1306_invertByte = 0x00000000;
const int bfactor = 64;
float t = 0.2;
uint16_t frameCount = 0;

union MatrixData {
  unsigned long long l;
  byte b[8];
};

MatrixData Matrix[128];

void setup() {
  Serial.begin(9600);

  delay(20);

  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();
}

void loop() {
  // unsigned long start = millis();
  t = 0.2 + 3*float(frameCount%100)/100;

  compute_buffer();
  //Serial.println(F("buffer done"));
  outputMatrix();

  frameCount += 1;
}





void outputMatrix() {

  ssd1306_lcd.set_block(0, 0, 128);

  for (uint16_t col = 0; col < 8; col++) {

    for (uint16_t row = 0; row <= 127; row++) {
      ssd1306_lcd.send_pixels1(s_ssd1306_invertByte ^ Matrix[row].b[col]);
      //spi.write(Matrix[row].b[col]);
    }	
    ssd1306_lcd.next_page();
  }
  ssd1306_intf.stop();
}

void compute_buffer() {
  //Set up initial cells in matrix
 for (uint16_t col = 0; col < 8; col++) {
    for (uint16_t row = 0; row < 128; row++) {
      Matrix[row].b[col] = gen_mb_byte(row, col);
    }
  }
}

byte gen_mb_byte(uint16_t x, uint16_t y){
  byte out = B00000000;
  float a,b,ca,cb,aa,bb, brightness;
  byte n;

  for (uint16_t i = 0; i < 8; i++) {
    a = -2.5*(0.5 - float(x)/64)*t - 1.5; 
    b = -2.5*(0.5 - float(y*8+i)/64)*t;
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

    brightness = easeOutCirc(float(n)/100) - 0.14;

    if(brightness * bfactor * 4 < bayer8[(x%8)*8 +(y*8+i)%8]){
      out = out | (1 << i);
    }
  }
  return ~out;
}

float easeOutCirc(float x) {
	return sqrt(1 - (x-1)*(x-1));
}


