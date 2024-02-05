#ifndef LIB_SRC_SERVO_CONTROL_H_
#define LIB_SRC_SERVO_CONTROL_H_

#include <Arduino.h>
//  Adafruit BNO055 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
//  Servo library
#include "Servo.h"

#include <array>  // std::array

class PIDController {
private:
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    float prevError;
    float integral;

public:
    PIDController(float p, float i, float d);
    float calculate(float setpoint, float angle);
};


void print_bno_data(Adafruit_BNO055 &bno);
float get_x_axis(float calibration_angle, Adafruit_BNO055 &bno);
std::array<float, 3> get_axis_data(Adafruit_BNO055 &bno);
void changeServoPosition(int angle, Servo &servo1);
void setAngle(Servo &servo1);
void sweepAngle(Servo &servo1);



#endif  // LIB_SRC_SERVO_CTRL_H_
