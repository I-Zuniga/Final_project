#include <user_interface.h>


/*--------------------------------------------*/
/*----------------LCD CALLS ------------------*/
/*--------------------------------------------*/

void option1() {
      LCD.print("Swepth t0 180");   // Print what the chosen option does
      delay(500);
      sweepAngle(servo1);
}

void option2() {
      LCD.print("Imput Angle");   // Print what the chosen option does
      delay(500);
      LCD.setCursor(5, 1);
      LCD.print("[___]");
      LCD.setCursor(6, 1);
      String result = "";
      while (result.length() < 3) {
        char key = customKeypad.getKey();
        if (key >= '0' && key <= '9') {
          LCD.print(key);
          result += key;
        }
      }
      int selected_angle = result.toInt();
      // Clip the angle to 180
      LCD.setCursor(0, 1);
      if (selected_angle > 180) {
        selected_angle = 180;
        LCD.print("MAX 180   ");
      }
      else {
        LCD.print("go to " + String(selected_angle) + "   ");
      }
      changeServoPosition(selected_angle, servo1);
      delay(1000);
}

void option3() {
      LCD.print("PID crtl(exit #)");   // Print what the chosen option does
      delay(500);
      // Make the PID calculation and return the output
      calibration_angle = calibrate_bno();

      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("PID crtl(exit #)");   // Print what the chosen option does

      LCD.setCursor(5, 1);
      LCD.print("[___]");
      LCD.setCursor(6, 1);
      String result = "";
      while (result.length() < 3) {
        char key = customKeypad.getKey();
        if (key >= '0' && key <= '9') {
          LCD.print(key);
          result += key;
        }
      }
      int selected_angle = result.toInt();
      // Clip the angle to 180
      LCD.setCursor(0, 1);
      if (selected_angle > 140) {
        selected_angle = 140;
        LCD.print("140         ");
      }
      else if (selected_angle < 40) {
        selected_angle = 40;
        LCD.print("40        ");
      }
      else {
        LCD.print(String(selected_angle)+"      ");
      }

      // Set PID objetive 
      changeServoPosition(selected_angle, servo1);
      delay(500);
      Setpoint = get_x_axis(calibration_angle, bno);

      float angle;
      char exit = '0';
      while ( exit != '#'){
        exit = customKeypad.getKey();  
        angle = get_x_axis(calibration_angle, bno);
        LCD.setCursor(4, 1);
        LCD.print("BNO: " + String(selected_angle - (Setpoint-angle),2));

        if (angle > 270) {
          angle = 0;
        }
        else if (angle < 270 && angle > 180) {
          angle = 180;
        }
        Output = pidController.calculate(Setpoint, angle);
        changeServoPosition(angle + Output, servo1);

      }
}

void not_a_option() {
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Not valid");
      delay(2000);
      LCD.setCursor(0, 0);
      LCD.print("so funny...");
      delay(2000);
      LCD.setCursor(0, 0);
      LCD.print("Now wait...");
      LCD.setCursor(2, 1);
      LCD.print("[..........]");
      LCD.setCursor(3, 1);
      for (byte i = 0; i < 10; i++) {
        delay(1000);
        LCD.print("=");
        }
}

void init_lcd() {
  // Begin LCD:

  LCD.print("Press *");

  char key = customKeypad.getKey();
  while (key != '*') {
    key = customKeypad.getKey();
  }

  LCD.clear();
  LCD.setCursor(0, 0);
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
}

float calibrate_bno() {

  float cal_angle = 0;
  // Calibrate the BNO055 sensor to macth the servo 0 angle (rotations can be different)
  LCD.setCursor(0, 0);
  LCD.print(" Calibrating ");

  for (size_t i = 0; i < 5; i++)
  {
    changeServoPosition(0,servo1);
    // Get the angle 
    sensors_event_t event;
    bno.getEvent(&event);
    cal_angle = event.orientation.x;

    LCD.setCursor(0, 1);
    LCD.print("Angle: ");
    LCD.setCursor(8, 1);
    LCD.print(String(cal_angle,2));
    delay(500);
    LCD.clear();
  }
 
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Calibrated for:");
  LCD.setCursor(2, 1);
  LCD.print(String(cal_angle,2));
  delay(2000);

  if (cal_angle == 0) {
    cal_angle = 360;
  }

  return cal_angle;
}