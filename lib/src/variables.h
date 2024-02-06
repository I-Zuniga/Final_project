#ifndef LIB_SRC_VARIABLES_H_
#define LIB_SRC_VARIABLES_H_

#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <servo_control.h>


// Create a servo object for controlling the servo motor
extern Servo servo1;
#define servoPin 13 // Defines the pin number to which the servo motor is connected

extern Adafruit_BNO055 bno;

// Create a BNO055 object:
const byte ROWS = 4;
const byte COLS = 3;

extern char hexaKeys[ROWS][COLS];
extern byte rowPins[ROWS];
extern byte colPins[COLS];

extern Keypad customKeypad;

// LCD Set:
extern const int rs, en, d0, d1, d2, d3;
extern LiquidCrystal LCD;

// PID Variables
extern float Setpoint, Input, Output;
extern float Kp, Ki, Kd;

extern float angle;
extern float calibration_angle;

extern PIDController pidController;


#endif  // LIB_SRC_VARIABLES_H_