#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <SPI.h>

const byte ROWS = 4; 
const byte COLS = 3; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};  //teensy pins that go to the rows --> 2,7,6,4 
byte colPins[COLS] = {12, 11, 10};  //teensy pins that go to the cols --> 3,1,5

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

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

