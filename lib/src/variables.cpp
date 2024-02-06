
#include <variables.h>



// Create a servo object for controlling the servo motor
Servo servo1;
#define servoPin 13 // Defines the pin number to which the servo motor is connected

// Create a BNO055 object:
Adafruit_BNO055 bno = Adafruit_BNO055(55); 

// KeyPad Set: 
// const byte ROWS; 
// const byte COLS; 

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
float Kp=0.8, Ki=1.2, Kd=0.1;

float angle;
float calibration_angle = 0;


PIDController pidController(Kp, Ki, Kd);
