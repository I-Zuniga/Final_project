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

/*--------------------------------------------*/
/*----------DEFINITIONS & VARIABLES-----------*/
/*--------------------------------------------*/

// Create a servo object for controlling the servo motor
Servo servo1;
#define servoPin 13 // Defines the pin number to which the servo motor is connected

// Create a BNO055 object:
Adafruit_BNO055 bno = Adafruit_BNO055(55); 

// KeyPad Set: 

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

// LCD Set:
const int rs = 17, en = 15, d0 = 23, d1 = 22, d2 = 21, d3 = 20;
LiquidCrystal LCD(rs, en, d0, d1, d2, d3);

// PID Variables
float Setpoint, Input, Output;
float Kp=0.8, Ki=0.8, Kd=0.1;

char option;
float angle;
float calibration_angle = 0;

PIDController pidController(Kp, Ki, Kd);

/*--------------------------------------------*/
/*-------------SETUP AND MAIN ----------------*/
/*--------------------------------------------*/

void setup()
{
  // Begin LCD:
  LCD.begin(16, 2);
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

  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
  // Serial.begin(9600);
    
  // Set the servo at 0º:
  changeServoPosition(0, servo1);
  delay(1000);
  
  // Initialize the BNO055 sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("no BNO055 detected...");
    while(1);
  }
  bno.setExtCrystalUse(true);
  LCD.clear();

  // Calibrate the BNO055 sensor to macth the servo 0 angle (rotations can be different)
  while (int i = 0 < 5) {
    LCD.print(" Calibrating ");
    changeServoPosition(0,servo1);
    // Get the angle 
    sensors_event_t event;
    bno.getEvent(&event);
    angle = event.orientation.x;

    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" Angle: ");
    LCD.setCursor(6, 1);
    LCD.print(angle);
    delay(500);
    LCD.clear();
    calibration_angle = angle;
    i++;
  }
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print(" Calibrated for: ");
  LCD.setCursor(6, 1);
  LCD.print(calibration_angle);
  delay(1000);
  LCD.clear();
  LCD.setCursor(0, 0);
  
  // Set PID objetive at 90º
  changeServoPosition(90, servo1);
  delay(1000);
  Setpoint = get_x_axis(calibration_angle, bno);


  delay(1000);
  LCD.print("Hey sweept");
  sweepAngle(servo1);
  LCD.clear();
  LCD.setCursor(0, 0);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)



  // Serial.print(" Angle: ");
  // Serial.print(angle, '\n');
  // Serial.print(" PID output: ");
  // Serial.print(Output+ angle, '\n');
  // Serial.print(" Setpoint: ");
  // Serial.print(Setpoint, '\n');
  // Serial.print(" Error: ");
  // Serial.print(Setpoint-angle, '\n');
  // Serial.println(" \n");
  // Serial.println("--------------------\n");
  // Serial.println(" \n");

  // get one character from the serial port
  // parser(); // TODO: CHECK WHY THEY ASK TWO TIMES 
  
  LCD.print(" 1, 2 or 3: ");
  char OptionKey = customKeypad.getKey();
  if (OptionKey){
    LCD.setCursor(6, 1);
    LCD.print(OptionKey);   // Print the chosen option
    delay(100);
    LCD.setCursor(0, 0);
    LCD.clear();
    if (OptionKey == 1) {
      LCD.print("Swepth");   // Print what the chosen option does
      delay(500);
      sweepAngle(servo1);
    }
    if (OptionKey == 2) {
      LCD.print("Imput Angle");   // Print what the chosen option does
      delay(500);
      setAngle(servo1);
    }
    if (OptionKey == 3) {
      LCD.print("PID control");   // Print what the chosen option does
      delay(500);
      // Make the PID calculation and return the output
      angle = get_x_axis(calibration_angle, bno);
      Output = pidController.calculate(Setpoint, angle);
      changeServoPosition(angle + Output, servo1);
    }
    else {
      LCD.print("1 2 or 3");   // Print what the chosen option does
    }
    LCD.setCursor(0, 0);
    LCD.clear();
  }
  delay(5000);
  // ---------------------------------------------------------------------------------
}
