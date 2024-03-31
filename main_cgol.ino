
#include <Wire.h>
#include "ssd1306.h"
#include "intf/ssd1306_interface.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// adapted from https://forum.arduino.cc/t/conways-game-of-life-with-128x64-graphic-lcd-update-also-with-128x64-spi-oled/210637/25
// for I2C
// Conway's Game Of Life 128x64
// PaulRB
// Jun 2014

union MatrixData {
  unsigned long long l;
  byte b[8];
};

MatrixData Matrix[129]; // Cell data in ram


void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
 
  delay(2000);

  Matrix[64].l = B0000010; Matrix[64].l = Matrix[64].l << 32;
  Matrix[65].l = B0000111; Matrix[65].l = Matrix[65].l << 32;
  Matrix[66].l = B0000100; Matrix[66].l = Matrix[66].l << 32;
  
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();

  //randomiseMatrix();
  outputMatrix();
}

void loop() {
  unsigned long start = millis();
  for (int i=0; i<1000; i++) {
    generateMatrix();
    outputMatrix();
  }

  Serial.print(F("Gens/s:"));
  Serial.println(1000000/(millis() - start));
}




void outputMatrix() {
  uint8_t s_ssd1306_invertByte = 0x00000000;

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
	
void randomiseMatrix() {

  //Set up initial cells in matrix
  randomSeed(analogRead(0));
  for (byte row = 0; row <= 127; row++) {
    for (byte col = 0; col <= 8; col++) {
      Matrix[row].b[col] = random(0xff);
    }
  }
}
	
void injectGlider() {

  byte col = random(127);
  byte row = random(63);
  Matrix[col++].l |= ((unsigned long long) B0000111) << row;
  Matrix[col++].l |= ((unsigned long long) B0000001) << row;
  Matrix[col++].l |= ((unsigned long long) B0000010) << row;

}
	
int generateMatrix() {



  //Variables holding data on neighbouring cells
  unsigned long long NeighbourN, NeighbourNW, NeighbourNE, CurrCells, NeighbourW, NeighbourE, NeighbourS, NeighbourSW, NeighbourSE;
	
  //Variables used in calculating new cells
  unsigned long long tot1, carry, tot2, tot4, NewCells;
  
  int changes = 0; // counts the changes in the matrix
  static int prevChanges[4]; // counts the changes in the matrix on prev 4 generations
  static int staleCount = 0; // counts the consecutive occurrances of the same number of changes in the matrix

  //set up N, NW, NE, W & E neighbour data
  NeighbourN = Matrix[127].l;
  CurrCells = Matrix[0].l;
  Matrix[128].l = CurrCells;  // copy row 0 to location after last row to remove need for wrap-around code in the loop

  NeighbourNW = NeighbourN >> 1 | NeighbourN << 63; 
  NeighbourNE = NeighbourN << 1 | NeighbourN >> 63;
	
  NeighbourW = CurrCells >> 1 | CurrCells << 63;
  NeighbourE = CurrCells << 1 | CurrCells >> 63;
  
  //Process each row of the matrix
  for (byte row = 0; row <= 127; row++) {
		
    //Pick up new S, SW & SE neighbours
    NeighbourS = Matrix[row + 1].l;
    
    NeighbourSW = NeighbourS >> 1 | NeighbourS << 63;

    NeighbourSE = NeighbourS << 1 | NeighbourS >> 63;

    //Count the live neighbours (in parallel) for the current row of cells
    //However, if total goes over 3, we don't care (see below), so counting stops at 4
    tot1 = NeighbourN;
    tot2 = tot1 & NeighbourNW; tot1 = tot1 ^ NeighbourNW;
    carry = tot1 & NeighbourNE; tot1 = tot1 ^ NeighbourNE; tot4 = tot2 & carry; tot2 = tot2 ^ carry;
    carry = tot1 & NeighbourW; tot1 = tot1 ^ NeighbourW; tot4 = tot2 & carry | tot4; tot2 = tot2 ^ carry;
    carry = tot1 & NeighbourE; tot1 = tot1 ^ NeighbourE; tot4 = tot2 & carry | tot4; tot2 = tot2 ^ carry;
    carry = tot1 & NeighbourS; tot1 = tot1 ^ NeighbourS; tot4 = tot2 & carry | tot4; tot2 = tot2 ^ carry;
    carry = tot1 & NeighbourSW; tot1 = tot1 ^ NeighbourSW; tot4 = tot2 & carry | tot4; tot2 = tot2 ^ carry;
    carry = tot1 & NeighbourSE; tot1 = tot1 ^ NeighbourSE; tot4 = tot2 & carry | tot4; tot2 = tot2 ^ carry;
		
    //Calculate the updated cells:
    // <2 or >3 neighbours, cell dies
    // =2 neighbours, cell continues to live
    // =3 neighbours, new cell born
    NewCells = (CurrCells | tot1) & tot2 & ~ tot4;
    
    //Have any cells changed?
    if (NewCells != CurrCells) {       
      //Count the change for "stale" test
      changes++;
      Matrix[row].l = NewCells;
    }

    //Current cells (before update), E , W, SE, SW and S neighbours become
    //new N, NW, NE, E, W neighbours and current cells for next loop
    NeighbourN = CurrCells;
    NeighbourNW = NeighbourW;
    NeighbourNE = NeighbourE;
    NeighbourE = NeighbourSE;
    NeighbourW = NeighbourSW;
    CurrCells = NeighbourS;
  }
    
  if (changes != prevChanges[0] && changes != prevChanges[1] && changes != prevChanges[2] && changes != prevChanges[3]) {
    staleCount = 0;
  }
  else {
    staleCount++; //Detect "stale" matrix
  }
    
  if (staleCount > 64) injectGlider(); //Inject a glider
  //SerialUSB.println(changes);

  for (int i=3; i>0; i--) {
    prevChanges[i] = prevChanges[i-1];
  }

  prevChanges[0] = changes;
}




