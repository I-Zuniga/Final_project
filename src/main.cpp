/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include "Servo.h"

Servo servo1; // Creates a servo object for controlling the servo motor
#define servoPin 9 // Defines the pin number to which the servo motor is connected

char option;

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

// Create a callabe function from the loop to change the servo position
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

}

void loop()
{


  // // Tell the servo to go to a particular angle:

  // servo1.write(45); // 90 is the angle to which the servo will rotate
  // delay(1000); // Wait for the servo to reach the angle
  // servo1.write(0); // 0 is the angle to which the servo will rotate
  // delay(1000); // Wait for the servo to reach the angle
  // servo1.write(90); // 0 is the angle to which the servo will rotate
  // delay(1000); // Wait for the servo to reach the angle

  // get one character from the serial port
  parser();
}
