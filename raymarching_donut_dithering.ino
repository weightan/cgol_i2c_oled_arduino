
#include <Wire.h>
#include "ssd1306.h"
#include "intf/ssd1306_interface.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define iter_max 64

const uint8_t bayer8[]= {0,32,8,40,2,34,10,42,48,16,56,24,
50,18,58,26,12,44,4,36,14,46,6,38,60,28,52,20,62,30,54,22,3,
35,11,43,1,33,9,41,51,19,59,27,49,17,57,25,15,47,7,39,13,45,
5,37,63,31,55,23,61,29,53,21};


const uint8_t s_ssd1306_invertByte = 0x00000000;
const uint8_t bfactor = 40;
float t = 0.0;
float cosang = 0;
float sinang = 0;
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
  unsigned long start = millis();
  t = 6.2831853*float(frameCount%50)/50;
  cosang = cos(t);
  sinang = sin(t);

  computeMatrix();
  //Serial.println(F("buffer done"));
  outputMatrix();

  frameCount += 1;

  Serial.println(millis() - start);
}


void outputMatrix() {
  ssd1306_lcd.set_block(0, 0, 128);

  for (uint8_t col = 0; col < 8; col++) {
    for (uint8_t row = 0; row <= 127; row++) {
      ssd1306_lcd.send_pixels1(s_ssd1306_invertByte ^ Matrix[row].b[col]);
    }	
    ssd1306_lcd.next_page();
  }

  ssd1306_intf.stop();
}

void computeMatrix() {
 for (uint8_t col = 0; col < 8; col++) {
    for (uint8_t row = 0; row < 128; row++) {
      Matrix[row].b[col] = gen_mb_byte(row, col);
    }
  }
}

float clip_it(float x, float a, float b){
  if(x>b) return b;
  if(x<a) return a;
  return x;
}

byte gen_mb_byte(uint8_t x, uint8_t y){
  byte out = B00000000;
  float a,b,brightness;
  byte n = 0;

  float r_d_x, r_d_y, r_d_z;
  float w_p_x, w_p_y, w_p_z;
  float n_r;
  float t;
  
  for (uint8_t i = 0; i < 8; i++) {
    a = float(x*5 - 128)/64;
    b = float(((y<<3)+i)*5)/64;

    r_d_x = -2.5 + a;
    r_d_y =  3.0;
    r_d_z = -2.5 + b;

    n_r = sqrt(a*a - 5*a + b*b - 5*b + 21.5);

    r_d_x = r_d_x / n_r;
    r_d_y = r_d_y / n_r;
    r_d_z = r_d_z / n_r;

    t = 0;
    n = 0;

    while (n < iter_max) {
      w_p_x = t * r_d_x;
      w_p_y = t * r_d_y - 3.0;
      w_p_z = t * r_d_z;

      float new_w_p_x = w_p_x*cosang - w_p_y*sinang;
      float new_w_p_y = w_p_x*sinang + w_p_y*cosang;

      float q = sqrt(new_w_p_x*new_w_p_x + w_p_z*w_p_z) - 1.6;
      float dist_field_f = sqrt(q*q + new_w_p_y*new_w_p_y) - 0.5;

      if(dist_field_f < 0.01) break;
		  if(t > 20) break;

      t += dist_field_f;
      n++;
    }

    if (t > 20) {
      out = out | (1 << i);
      continue;
    }

    brightness = clip_it(t*0.185 + float(n)*0.02,0,1);

    if(brightness * 64 > bayer8[(x & B00000111)*8 + ((y*8+i) & B00000111)]){
      out = out | (1 << i);
    }
  }

  return ~out;
}
