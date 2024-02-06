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

/*--------------------------------------------*/
/*----------------LCD CALLS ------------------*/
/*--------------------------------------------*/

void option1() {
      LCD.print("Swepth");   // Print what the chosen option does
      delay(500);
      sweepAngle(servo1);
}

void option2() {
      LCD.print("Imput Angle");   // Print what the chosen option does
      delay(500);
      LCD.setCursor(5, 1);
      LCD.print("[___]");
      LCD.setCursor(6, 1);
      String result = "";
      while (result.length() < 3) {
        char key = customKeypad.getKey();
        if (key >= '0' && key <= '9') {
          LCD.print(key);
          result += key;
        }
      }
      int selected_angle = result.toInt();
      // Clip the angle to 180
      LCD.setCursor(0, 1);
      if (selected_angle > 180) {
        selected_angle = 180;
        LCD.print("MAX 180   ");
      }
      else {
        LCD.print("go to " + String(selected_angle) + "   ");
      }
      changeServoPosition(selected_angle, servo1);
      delay(1000);
}

void option3() {
      LCD.print("PID crtl(exit #)");   // Print what the chosen option does
      delay(500);
      // Make the PID calculation and return the output
      float angle;
      char exit = '0';
      while ( exit != '#'){
        exit = customKeypad.getKey();  
        angle = get_x_axis(calibration_angle, bno);
        Output = pidController.calculate(Setpoint, angle);
        changeServoPosition(angle + Output, servo1);
        LCD.setCursor(4, 1);
        LCD.print("BNO: " + String(angle,2));
      }
}

void not_a_option() {
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Not valid");
      delay(2000);
      LCD.setCursor(0, 0);
      LCD.print("so funny...");
      delay(2000);
      LCD.setCursor(0, 0);
      LCD.print("Now wait...");
      LCD.setCursor(2, 1);
      LCD.print("[..........]");
      LCD.setCursor(3, 1);
      for (byte i = 0; i < 10; i++) {
        delay(1000);
        LCD.print("=");
        }
}

void init_lcd() {
  // Begin LCD:
  LCD.print(" Willkommen!");
  delay(1000);
      for (int positionCounter = 0; positionCounter < 16; positionCounter++) {
        // scroll one position right:
        LCD.scrollDisplayRight();
        // wait a bit:
        delay(150);
    }
    LCD.clear();
    LCD.setCursor(0, 0);
}

float calibrate_bno() {

  float cal_angle = 0;
  // Calibrate the BNO055 sensor to macth the servo 0 angle (rotations can be different)
  LCD.setCursor(0, 0);
  LCD.print(" Calibrating ");

  for (size_t i = 0; i < 5; i++)
  {
    changeServoPosition(0,servo1);
    // Get the angle 
    sensors_event_t event;
    bno.getEvent(&event);
    cal_angle = event.orientation.x;

    LCD.setCursor(0, 1);
    LCD.print("Angle: ");
    LCD.setCursor(8, 1);
    LCD.print(String(cal_angle,2));
    delay(500);
    LCD.clear();
  }
 
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Calibrated for:");
  LCD.setCursor(2, 1);
  LCD.print(String(cal_angle,2));
  delay(2000);

  return cal_angle;
}

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

  // Set PID objetive at 90º
  changeServoPosition(90, servo1);
  delay(1000);
  Setpoint = get_x_axis(calibration_angle, bno);

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
