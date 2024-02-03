#include <Arduino.h>

#include <cmath>
#include <iostream>
#include <array>  // std::array

 
//  Adafruit BNO055 library
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

Servo servo1; // Creates a servo object for controlling the servo motor
#define servoPin 9 // Defines the pin number to which the servo motor is connected

// Creates a BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
  Serial.begin(9600);

  changeServoPosition(0, servo1);
  delay(3000);

  // Print the options to the serial port
  Serial.println("Initilizing... \n");
  Serial.println("Options: \n");
  Serial.println("--------------------\n");
  Serial.println("a - Manualy change the servo position\n");
  Serial.println("l - Sweep from 0 to 180 degrees\n");
  // Serial.println("s - Keeps the same angle even the base move\n");
  
  // Initialize the BNO055 sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  // Calibrate the offset between BNO frame and Servo frame 
  while (int i = 0  < 5)
  {
    Serial.print(" Waiting for calibrate: ");
    changeServoPosition(0, servo1);
    // Get the angle 
    sensors_event_t event;
    bno.getEvent(&event);
    angle = event.orientation.x;

    Serial.print(" Angle: ");
    Serial.print(angle);
    Serial.println("--------------------");
    delay(500);
    calibration_angle = angle;
    i++;
  }
  
  Serial.println(" Calibrated for angle: ");
  Serial.print(calibration_angle);

  // Set PID angle objetive
  changeServoPosition(90,  servo1); // Focus in the mean angle (maximun rotation)
  delay(1000);
  Setpoint = get_x_axis(calibration_angle, bno);

  // Performs a friendly sweept to check everything is working 
  delay(500);
  Serial.println(" Hello sweept \n");
  sweepAngle(servo1);
}



void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)

  // Make the PID calculation and return the output
  angle = get_x_axis(calibration_angle, bno);
  Output = pidController.calculate(Setpoint, angle);
  changeServoPosition(angle + Output, servo1);

  Serial.print(" Angle: ");
  Serial.print(angle, '\n');
  Serial.print(" PID output: ");
  Serial.print(Output+ angle, '\n');
  Serial.print(" Setpoint: ");
  Serial.print(Setpoint, '\n');
  Serial.print(" Error: ");
  Serial.print(Setpoint-angle, '\n');
  Serial.println(" \n");

  Serial.println("--------------------\n");
  Serial.println(" \n");

  // get one character from the serial port
  // parser(); // TODO: CHECK WHY THEY ASK TWO TIMES 
}
