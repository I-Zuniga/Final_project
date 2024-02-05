#include <Arduino.h>
#include <SPI.h>
#include <Wire.h> 
#include <LiquidCrystal.h>
#include <Keypad.h>
//------------------------------------

const int rs = 17, en = 15, d0 = 23, d1 = 22, d2 = 21, d3 = 20;
LiquidCrystal LCD(rs, en, d0, d1, d2, d3);


void setup(){
    LCD.begin(16, 2);
    LCD.print("hi!");

    delay(2000);
    for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
        // scroll one position right:
        LCD.scrollDisplayRight();
        // wait a bit:
        delay(150);
    }
    LCD.clear();
    LCD.setCursor(6, 0);
    LCD.print("Count!");
  // print the number of seconds since reset:
}

void loop() {
  for (int i = 0; i < 10; i ++)
  {
    LCD.setCursor(8, 1);
    // print the number of seconds since reset:
    LCD.print(i);
    delay(1000);
  }
}
