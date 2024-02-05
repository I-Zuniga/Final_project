#include <Arduino.h>
#include "Servo.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
// # Install the Adafruit BNO055 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

// Custom libs - Not being used at the moment

#include <PID_v1.h>

#include <array>  // std::array

/*--------------------------------------------*/
/*-----------------DEFINITIONS----------------*/
/*--------------------------------------------*/

// Create a servo object for controlling the servo motor
Servo servo1;
#define servoPin 9 // Defines the pin number to which the servo motor is connected

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

// PID definitions
// define PIN_INPUT 0 // Read
// define PIN_OUTPUT 9 // Write 
double Setpoint, Input, Output;
double Kp=1, Ki=0.25, Kd=0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

char option;

double angle;
double calibration_angle = 0;

int i = 0; // Counter for the calibration


/*--------------------------------------------*/
/*-----------------PID CLASS------------------*/
/*--------------------------------------------*/

class PIDController {
private:
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain
    double prevError;
    double integral;

public:
    PIDController(double p, double i, double d)
        : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

    double calculate(double setpoint, double processVariable) {
        double error = setpoint - processVariable;
        integral += error;
        double derivative = error - prevError;

        double output = kp * error + ki * integral + kd * derivative;

        prevError = error;

        // Clip output to the range [0, 180]
        if (output > 180) {
            output = 180;
        } else if (output < 0) {
            output = 0;
        }

        return output;
    }
};

// Create PID controller self written code 
PIDController pidController(Kp, Ki, Kd);

/*--------------------------------------------*/
/*-----------------FUNTIONS-------------------*/
/*--------------------------------------------*/

// Change the servo position to the input angle s
void changeServoPosition(int angle) {
  servo1.write(angle);
  delay(10);
}

void setAngle() {
  // Read a position value from the keypad
  char customKey = customKeypad.getKey();
  delay(3000);
  if (customKey){
    LCD.clear();
    LCD.setCursor(0, 0); 
    LCD.print(" Option: ");
    LCD.print(customKey);
  }
  // Change the servo position
  changeServoPosition(angle);
  };


void sweepAngle() {
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle < 180; angle++) {
    changeServoPosition(angle);
  }
  // Sweep from 180 to 0 degrees
  for (int angle = 180; angle > 0; angle--) {
    changeServoPosition(angle);
  }
}

void init_bno(){
  
  // Initialize the BNO055 sensor
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Initalizing");
  LCD.setCursor(0, 1);
  LCD.print(" BNO055 ");
  
  /* Initialise the sensor */
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" Error :( ");
    LCD.setCursor(0, 1);
    LCD.print(" Not found ");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  LCD.clear();
  LCD.setCursor(0, 0);
}

// Create a function to return the sensor x:
float get_x_axis(double calibration_angle){
    // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  float x = calibration_angle - event.orientation.x;

  delay(100);
  return x;
}


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
  changeServoPosition(0);
  delay(3000);
  
  // Checking initialization of the BNO055 sensor
  init_bno();
  // I dont know why we need the calibration while -----------------------------------
  while (i < 5) {
    LCD.print(" Calibrating ");
    changeServoPosition(0);
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
    // Change the BNO 
    i++;
  }
  LCD.clear();
  LCD.setCursor(0, 0);
  
  LCD.print("Calibrated:");
  LCD.setCursor(6, 1);
  LCD.print("Angle: ", calibration_angle);

  LCD.clear();
  LCD.setCursor(0, 0);
  // ---------------------------------------------------------------------------------
  //What is this part actually doing and why do we need it here? -----------------------
  // Set PID
  changeServoPosition(90);
  delay(1000);
  Setpoint = get_x_axis(calibration_angle);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,180);
  myPID.SetControllerDirection(DIRECT);
  // -----------------------------------------------------------------------------------

  // delete this part: -----------------------------------------------------------
  delay(1000);
  LCD.print("Hey sweept");
  sweepAngle();
  // ------------------------------------------------------------------------------
  
  LCD.clear();
  LCD.setCursor(0, 0);
}

void loop()
{
  // we dont have a led anymore ------------------------------------------------------
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)
  // ---------------------------------------------------------------------------------
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
      sweepAngle();
    }
    if (OptionKey == 2) {
      LCD.print("Imput Angle");   // Print what the chosen option does
      delay(500);
      setAngle();
    }
    if (OptionKey == 3) {
      LCD.print("Const Angle");   // Print what the chosen option does
      delay(500);
      angle = get_x_axis(calibration_angle);
      Output = pidController.calculate( Setpoint, angle);
      changeServoPosition(Output);
    }
    else {
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Choose valid");
      LCD.setCursor(0, 1);
      LCD.print("option");
    }
    LCD.setCursor(0, 0);
    LCD.clear();
  }
  delay(5000);
  // ---------------------------------------------------------------------------------
}
