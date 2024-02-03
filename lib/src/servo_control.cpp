#include <servo_control.h>
// #include <util.h>



PIDController::PIDController(float p, float i, float d)
        : kp(p), ki(i), kd(d), prevError(0), integral(0) {};

float PIDController::calculate(float setpoint, float angle) {
        float error = setpoint - angle;
        integral += error;
        float derivative = error - prevError;

        float output = kp * error + ki * integral + kd * derivative;

        prevError = error;

        return output;
    }

// Change the servo position to the input angle s
void changeServoPosition(int angle, Servo &servo1) {
  servo1.write(angle);
  delay(10);
}


void setAngle(Servo &servo1) {
  // Read a position value from the serial port
  Serial.println("Enter the angle: ");
  delay(3000);
  while (Serial.available() == 0) {
    delay(10);
  }
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();
    // Change the servo position
    changeServoPosition(angle, servo1);
  };
}

void sweepAngle(Servo &servo1) {
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle < 180; angle++) {
    changeServoPosition(angle, servo1);
  }
  // Sweep from 180 to 0 degrees
  for (int angle = 180; angle > 0; angle--) {
    changeServoPosition(angle, servo1);
  }
}

std::array<float, 3> get_axis_data(Adafruit_BNO055 &bno){
    // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  float x = event.orientation.x;
  float y = event.orientation.y;
  float z = event.orientation.z;

  delay(100);
  return {x, y, z};
}

float get_x_axis(float calibration_angle, Adafruit_BNO055 &bno){
    // Get the sensor data
  sensors_event_t event;
  bno.getEvent(&event);
  float x = calibration_angle - event.orientation.x;

  delay(10);
  return x;
}

void print_bno_data(Adafruit_BNO055 &bno){
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