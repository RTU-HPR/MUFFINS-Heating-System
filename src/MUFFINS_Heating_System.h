#pragma once
#include <Arduino.h>

class Heating_System
{
public:
  struct Config
  {
    int heater_pin;
    float Kp;
    float Ki;
    float Kd;
    float Kp_limit;
    float Ki_limit;
    float Kd_limit;
    int pwm_min;
    int pwm_max;
    float target_temp;
  };

  struct Data
  {
    bool enabled;
    float proportional_term;
    float integral_term;
    float derivative_term;
    float heater_pwm;
    float target_temp;
  };

private:
  Config _config;

  struct Runtime_Variables
  {
    bool enabled = false;
    unsigned long int last_calc_time = 0;
    float p = 0;
    float d = 0;
    float i = 0;
    float last_p = 0;
    int heater_pwm = 0;
  } _runtime_variables;

  void _calculate_pid_values(const float &temperature);
  void _calculate_heater_pwm();

  void _set_heater_pwm();

public:
  Heating_System();
  ~Heating_System();
  void begin(const Config &config);
  bool update(const float &temperature);
  void reset();
  void info(Data &data);
  void enable_heater();
  void disable_heater();
  bool is_enabled() { return _runtime_variables.enabled; };
};