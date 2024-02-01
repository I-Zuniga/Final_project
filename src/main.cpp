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
// #include <pid_ctrl.h>

// PID library 
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
// define PIN_OUTPUT 3 // Write 
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
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle < 180; angle++) {
    changeServoPosition(angle);
  }
  // Sweep from 90 to 0 degrees
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

// Create a function to return the sensor data as an array

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

  delay(100);
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


  // ------------------------------
  // This almu doesnt touch it cuz idk what is it doing
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
  Serial.begin(9600);
  // ------------------------------
    
  // Set the servo at 0º:
  changeServoPosition(0);
  delay(3000);
  
  // Checking initialization of the BNO055 sensor
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    LCD.setCursor(0, 0); 
    LCD.print(" BNO... ");
    LCD.setCursor(0, 1); 
    LCD.print(" Not detected ");
    while(1);
  }

  bno.setExtCrystalUse(true);
  // void init_bno(); //TODO CHECK WHY IT DOES NOT WORK
  LCD.clear();
  LCD.setCursor(0, 0);
  while (i < 5)
  {
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
    LDC.clear();
    calibration_angle = angle;
    // Change the BNO 
    i++;
  }
  LCD.clear();
  LCD.setCursor(0, 0);
  
  LCD.print(" Calibrated for: ");
  LCD.setCursor(6, 1);
  LCD.print(calibration_angle);

  LCD.clear();
  LCD.setCursor(0, 0);
  // ---------------------------------------------------------------------------------
  // This almu doesnt touch it cuz idk what is it doing
  // Set PID
  changeServoPosition(90);
  delay(1000);
  Setpoint = get_x_axis(calibration_angle);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,180);
  myPID.SetControllerDirection(DIRECT);

  delay(1000);
  Serial.println(" Hello sweept \n");
  sweepAngle();
  // ---------------------------------------------------------------------------------
}

void loop()
{
  LCD.print(" Choose 1, 2 or 3: ");
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
      // HERE WE SHOULD CALL FUNCTION THAT DOES THE SWEPTH
    }
    if (OptionKey == 2) {
      LCD.print("Imput Angle");   // Print what the chosen option does
      delay(500);
      // HERE WE CALL THE FUNCTION THAT IMPUTS ANGLE -- RETURNS ANGLE
      // HERE WE CALL THE FUNCTION THAT MOVES SERVO TO DESIRED ANGLE --INPUT ANGLE, RETURNS NOTHING
    }
    if (OptionKey == 3) {
      LCD.print("Const Angle");   // Print what the chosen option does
      delay(500);
      // HERE WE CALL THE FUNCTION THAT MAINTAINS ANGLE
    }
    else {
      LCD.print("1 2 or 3");   // Print what the chosen option does
    }
    LCD.setCursor(0, 0);
    LCD.clear();
  }
  //Serial.println("--------------------\n");
  //Serial.println("a - Manualy change the servo position\n");
  //Serial.println("l - Sweep from 0 to 180 degrees\n");

  // ---------------------------------------------------------------------------------
  // This almu doesnt touch it cuz idk what is it doing
 
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on (Check if the board is working)
  // Calls the function to get the sensor data
  // get_bno();
  delay(100);

  // Make the PID calculation and return the output
  angle = get_x_axis(calibration_angle);

  // Self coded PID
  Output = pidController.calculate( Setpoint, angle);
  changeServoPosition(Output);

  // //library PID
  // Input = angle;
  // myPID.Compute();
  // changeServoPosition(int(Output));
  
  Serial.print(" Angle: ");
  Serial.print(angle, '\n');
  Serial.print(" PID output: ");
  Serial.print(Output, '\n');
  Serial.print(" Setpoint: ");
  Serial.print(Setpoint, '\n');
  Serial.print(" Error: ");
  Serial.print(angle - Setpoint, '\n');
  Serial.println(" \n");


  delay(100);
  Serial.println("--------------------\n");
  Serial.println(" \n");

  // get one character from the serial port
  // parser(); // TODO: CHECK WHY THEY ASK TWO TIMES
  // ---------------------------------------------------------------------------------
}
