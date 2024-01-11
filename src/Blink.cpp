/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include "Servo.h"

Servo servo1; // Creates a servo object for controlling the servo motor
#define servoPin 9 // Defines the pin number to which the servo motor is connected

// Create a 
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  servo1.attach(servoPin); // Attaches the servo on pin 3 to the servo object
}

void loop()
{
  // Turn on the internal led
  digitalWrite(LED_BUILTIN, HIGH); // Led on
  delay(1000);              // Wait for 1 second
  digitalWrite(LED_BUILTIN, LOW);  // Led off
  delay(1000);              // Wait for 1 second

  // Tell the servo to go to a particular angle:

  servo1.write(45); // 90 is the angle to which the servo will rotate
  delay(1000); // Wait for the servo to reach the angle
  servo1.write(0); // 0 is the angle to which the servo will rotate
  delay(1000); // Wait for the servo to reach the angle
  servo1.write(90); // 0 is the angle to which the servo will rotate
  delay(1000); // Wait for the servo to reach the angle


  // // Sweep from 0 to 180 degrees:
  // for (int angle = -90; angle <= 90; angle += 1) {
  //   servo1.write(angle);
  //   delay(50);
  // }
 
}
