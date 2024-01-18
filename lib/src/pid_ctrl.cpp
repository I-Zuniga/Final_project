#include <pid_ctrl.h>
// #include <util.h>
#define ABS(a) ((a) < 0 ? (-a) : (a))

PIDControl::PIDControl(){}

PIDControl::~PIDControl(){
  pinMode(_pin, INPUT);
}

void PIDControl::reset(){
    _error[0] = _error[1] = 0;
    _prev_ctrl = 0;
}

void PIDControl::setup_pid(uint8_t pin, float min, float max, float f){
    _pin = pin;
    _min = min;
    _max = max;
    _error[0] = 0;
    _error[1] = 0;
    _prev_ctrl = 0;
    pinMode(pin, OUTPUT);  //  set pin as output
    analogWriteFrequency(pin, ABS(f));  // set PWM frequency
}

/****************************************************
 *             TASK4: PID Controller                *
 * Use PWM to control MOSFET. Use "get_pid_ctrl" to *
 * get corresponding "action". Keep in mind that    *
 * actions must lie within interval of [0, 255].    *
 * Note that value 255 will lead to no action.      *
 * This is a flavor of the arduino library          *
 * Use the CLIP(v, min, max) MACRO to clip actions. *
 * C.f. "util.h" to get a detailed description.     *
 * Be careful how to choose "min" and "max", since  *
 * the ASSR1228 optocoupler has timings that need to*
 * be considered. Cf. datasheet for timings.        *
 ****************************************************/
void PIDControl::pid_ctrl(const pid_params *pid, float error, float dt){
  //  YOUR CODE HERE
  

}

// DO NOT ALTER THIS METHOD
float PIDControl::_get_pid_ctrl(const pid_params *params,
                               const float *error,
                               const float *dt){
  //  Discrete PID Control derived from Z-Transform
  float kd_div_dt = params->k_d / (*dt);
  float ki_div_2_dt = params->k_i / 2.0f * (*dt);
  float kp = params->k_p;
  float ctrl = _prev_ctrl;

  ctrl += (kp + ki_div_2_dt + kd_div_dt) * (*error);
  ctrl -= (kp - ki_div_2_dt + 2.0f * kd_div_dt) * _error[1];
  ctrl += kd_div_dt * _error[0];
  _prev_ctrl = ctrl;

  _error[0] = _error[1];
  _error[1] = *error;

  return ctrl;
}
