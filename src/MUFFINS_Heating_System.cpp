#include <MUFFINS_Heating_System.h>

Heating_System::Heating_System()
{
  return;
}

Heating_System::~Heating_System()
{
  return;
}

void Heating_System::_calculate_pid_values(const float &temperature)
{
  // Time since last pid update
  int dt = (millis() - _runtime_variables.last_calc_time);

  _runtime_variables.p = _config.target_temp - temperature;

  // Integral term
  _runtime_variables.i += _runtime_variables.p * dt;

  // Derivative term
  if (_runtime_variables.last_p == 0)
  {
    _runtime_variables.last_p = _runtime_variables.p;
  }
  _runtime_variables.d = (_runtime_variables.p - _runtime_variables.last_p) / (float)dt;

  // Make sure the the PID values are within the limits
  _runtime_variables.p = constrain(_runtime_variables.p, -_config.Kp_limit, _config.Kp_limit);
  _runtime_variables.i = constrain(_runtime_variables.i, 0, _config.Ki_limit / _config.Ki); // Integral term can't be negative in our case
  _runtime_variables.d = constrain(_runtime_variables.d, -_config.Kd_limit, _config.Kd_limit);

  // Save last proportional term for future derivative term calculations
  _runtime_variables.last_p = _runtime_variables.p;

  // Save last PID calculation time for time delta calculations
  _runtime_variables.last_calc_time = millis();
}

void Heating_System::_calculate_heater_pwm()
{
  // Heater power is the sum of all the individual PID values multiplied by their coefficients
  _runtime_variables.heater_pwm = _config.Kp * _runtime_variables.p;
  _runtime_variables.heater_pwm += _config.Ki * _runtime_variables.i;
  _runtime_variables.heater_pwm += _config.Kd * _runtime_variables.d;

  _runtime_variables.heater_pwm = floor(constrain(_runtime_variables.heater_pwm, _config.pwm_min, _config.pwm_max));
}

void Heating_System::_set_heater_pwm()
{
  analogWrite(_config.heater_pin, _runtime_variables.heater_pwm);
}

void Heating_System::begin(const Config &config)
{
  // Save the config
  _config = config;

  // Reset all values to default
  reset();

  // By default, the heater is disabled
  _runtime_variables.enabled = true;
}

void Heating_System::reset()
{
  // Reset all values to default
  _runtime_variables.last_calc_time = millis();
  _runtime_variables.p = 0;
  _runtime_variables.last_p = 0;
  _runtime_variables.d = 0;
  _runtime_variables.i = 0;
  _runtime_variables.heater_pwm = 0;
  // Set the heater PWM to 0
  _set_heater_pwm();
}

void Heating_System::enable_heater()
{
  // Reset all values to default
  reset();

  // Enable the heater
  _runtime_variables.enabled = true;
}

void Heating_System::disable_heater()
{
  // Reset all values to default
  reset();

  // Disable the heater
  _runtime_variables.enabled = false;
}

bool Heating_System::update(const float &temperature)
{
  if (!is_enabled())
  {
    return false;
  }

  _calculate_pid_values(temperature);
  _calculate_heater_pwm();
  _set_heater_pwm();
  return true;
}

void Heating_System::info(Data &data)
{
  data.enabled = _runtime_variables.enabled;
  data.proportional_term = _runtime_variables.p;
  data.integral_term = _runtime_variables.i;
  data.derivative_term = _runtime_variables.d;
  data.heater_pwm = _runtime_variables.heater_pwm;
  data.target_temp = _config.target_temp;
}