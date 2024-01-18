#include <Arduino.h>
#include "Servo.h"
 
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



/*--------------------------------------------*/
/*-----------------DEFINITIONS----------------*/
/*--------------------------------------------*/

Servo servo1; // Creates a servo object for controlling the servo motor
#define servoPin 9 // Defines the pin number to which the servo motor is connected

// Creates a BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// PID definitions
#define PIN_INPUT 0 // Read
#define PIN_OUTPUT 3 // Write 
double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

char option;

/*--------------------------------------------*/
/*-----------------FUNTIONS-------------------*/
/*--------------------------------------------*/

// Change the servo position to the input angle s
void changeServoPosition(int angle) {
  servo1.write(angle);
  delay(50);
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

void sweepAngle() {
  // Sweep from 0 to 90 degrees
  for (int angle = 0; angle < 90; angle++) {
    changeServoPosition(angle);
  }
  // Sweep from 90 to 0 degrees
  for (int angle = 90; angle > 0; angle--) {
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

void run_PID(){
  // Make the PID calculation and return the output
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
}

/*--------------------------------------------*/
/*-------------SETUP AND MAIN ----------------*/
/*--------------------------------------------*/


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
  Serial.begin(9600);

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
  delay(1000); 
  bno.setExtCrystalUse(true);
  // void init_bno(); //TODO CHECK WHY IT DOES NOT WORK

  // PID setup
  Setpoint = 90;
  Input = analogRead(PIN_INPUT);
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)

  // Calls the function to get the sensor data
  get_bno();

  // Make the PID calculation and return the output
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);


  // get one character from the serial port
  // parser(); // TODO: CHECK WHY THEY ASK TWO TIMES 
}
