#ifndef LIB_SRC_USER_INTERFACE_H_
#define LIB_SRC_USER_INTERFACE_H_

#include <Arduino.h>
//  Adafruit BNO055 library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
//  Servo library
#include "Servo.h"

#include "servo_control.h"
#include "variables.h"

#include <array>  // std::array

/*--------------------------------------------*/
/*----------------LCD CALLS ------------------*/
/*--------------------------------------------*/

void option1();
void option2();
void option3();
void not_a_option();
void init_lcd();
float calibrate_bno();

#endif  //LIB_SRC_USER_INTERFACE_H_
