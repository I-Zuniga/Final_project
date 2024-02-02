#include <Arduino.h>
#include "Servo.h"

#include <cmath>
#include <iostream>
 
// # Install the Adafruit BNO055 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

// Custom libs - Not being used at the moment
// #include <pid_ctrl.h>

// PID library 
#include <PID_v1.h>

#include <array>  // std::array

/*--------------------------------------------*/
/*-----------------DEFINITIONS----------------*/
/*--------------------------------------------*/

Servo servo1; // Creates a servo object for controlling the servo motor
#define servoPin 9 // Defines the pin number to which the servo motor is connected

// Creates a BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// PID definitions
// #define PIN_INPUT 0 // Read
// #define PIN_OUTPUT 9 // Write 
double Setpoint, Input, Output;
// double Kp=0.8, Ki=0.07, Kd=0.1;
double Kp=0.8, Ki=0.8, Kd=0.1;

// double Kp=2, Ki=0.25, Kd=0.0;

// double Kp=1, Ki=0, Kd=0;

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

    double calculate(double setpoint, double angle) {
        double error = setpoint - angle;
        integral += error;
        double derivative = error - prevError;

        double output = kp * error + ki * integral + kd * derivative;

        prevError = error;

        Serial.print("error: ");
        Serial.print(error);

        // Clip output to the range [0, 180]
        // if (output > 180) {
        //     output = 180;
        // } else if (output < 0) {
        //     output = 0;
        // }

        Serial.print(" output: ");
        Serial.print(output);

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
  // Read a position value from the serial port
  Serial.println("Enter the angle: ");
  delay(3000);
  while (Serial.available() == 0) {
    delay(10);
  }
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();

    // Change the servo position
    changeServoPosition(angle);
  };
}

void performMagCal(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  while (mag != 3) {

    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println("");
  }

  Serial.println("\nMagnetometer calibrated!");
}  

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
  Serial.println("Initializing BNO055...");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 detected \n");
}

// CReate a function to return the sensor data as an array

std::array<float, 3> get_axis(){
    // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  float x = event.orientation.x;
  float y = event.orientation.y;
  float z = event.orientation.z;

  delay(100);
  return {x, y, z};
}

float get_x_axis(double calibration_angle){
    // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  float x = calibration_angle - event.orientation.x;

  delay(10);
  return x;
}

// float get_angle(std::array<float, 3> position){
//   float x = position[0];
//   float y = position[1];
//   float z = position[2];
 
//   float angle = atan2(y, x) * 180 / PI;
//   return angle;
// }

void get_bno(){
  // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  // Display the floating point data
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  delay(100);
}

// Create a parser function to read the serial port and change the servo position
void parser() {
  // get one character from the serial port
  Serial.println("Parser active \n");
  option = Serial.read();

  // Wait until response 
  while (Serial.available() == 0) {
    delay(10);
  }
  // if the character is 'a' then change the servo position
  if (option == 'a') {
    setAngle();
  }
  // if the character is 'l' then sweep the servo position
  if (option == 'l') {
    sweepAngle();
  }
}

// void run_PID(){
//   // Make the PID calculation and return the output
//   Input = analogRead(PIN_INPUT);
//   myPID.Compute();
//   analogWrite(servoPin, Output);
// }

/*--------------------------------------------*/
/*-------------SETUP AND MAIN ----------------*/
/*--------------------------------------------*/


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
  Serial.begin(9600);

  changeServoPosition(0);
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
  // void init_bno(); //TODO CHECK WHY IT DOES NOT WORK


  while (i < 5)
  {
    Serial.print(" Waiting for calibrate: ");
    changeServoPosition(0);
    // Get the angle 
    sensors_event_t event;
    bno.getEvent(&event);
    angle = event.orientation.x;


    Serial.print(" Angle: ");
    Serial.print(angle);
    Serial.println("--------------------");
    delay(500);
    calibration_angle = angle;
    // Change the BNO 
    i++;
  }
  
  Serial.println(" Calibrated for angle: ");
  Serial.print(calibration_angle);

  // Set PID
  changeServoPosition(90);
  delay(1000);
  // Setpoint = 90;
  Setpoint = get_x_axis(calibration_angle);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,180);
  myPID.SetControllerDirection(DIRECT);

  // delay(1000);
  Serial.println(" Hello sweept \n");
  // sweepAngle();
}



void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)

  // Calls the function to get the sensor data
  // get_bno();
  delay(100);

  // Make the PID calculation and return the output
  angle = get_x_axis(calibration_angle);

  // Self coded PID
  Output = pidController.calculate( Setpoint, angle);
  changeServoPosition(angle + Output);

  // //library PID
  // Input = angle;
  // myPID.Compute();
  // changeServoPosition(int(Output));
        // double error = setpoint - processVariable;
  
  Serial.print(" Angle: ");
  Serial.print(angle, '\n');
  // Serial.print(" PID output: ");
  // Serial.print(Output+ angle, '\n');
  Serial.print(" Setpoint: ");
  Serial.print(Setpoint, '\n');
  Serial.print(" Error: ");
  Serial.print(Setpoint-angle, '\n');
  Serial.println(" \n");


  delay(100);
  Serial.println("--------------------\n");
  Serial.println(" \n");

  // get one character from the serial port
  // parser(); // TODO: CHECK WHY THEY ASK TWO TIMES 
}
