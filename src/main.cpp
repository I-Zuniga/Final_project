#include <Arduino.h>
#include <cmath>
#include <iostream>
#include <array>  // std::array

// Keypad and LCD library
#include <LiquidCrystal.h>
#include <Keypad.h>

//   Adafruit BNO055 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

//  Servo library
#include "Servo.h"

// Custom libraries
#include "servo_control.h"
#include "variables.h"
#include "user_interface.h"


/*--------------------------------------------*/
/*-------------SETUP AND MAIN ----------------*/
/*--------------------------------------------*/

void setup()
{ 
  // Begin LCD:
  LCD.begin(16, 2);
  init_lcd();

  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
    
  // Set the servo at 0º:
  changeServoPosition(0, servo1);
  delay(1000);
  
  // Initialize the BNO055 sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Waiting BNO...");
    while(1);
  }
  bno.setExtCrystalUse(true);
  LCD.clear();

  // Calibrate the BNO055 sensor to macth the servo 0 angle (rotations can be different)
  calibration_angle = calibrate_bno();
  delay(1000);

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Opt: 1, 2 or 3:");
}

void loop()
{

  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)
  char OptionKey = customKeypad.getKey();

  if (OptionKey){
    LCD.setCursor(0, 0);
    LCD.clear();
    if (OptionKey == '1') { option1();}
    else if (OptionKey == '2') { option2();}
    else if (OptionKey == '3') { option3();}
    else { not_a_option();}
    delay(2000);
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Opt: 1, 2 or 3:");
  }

}
