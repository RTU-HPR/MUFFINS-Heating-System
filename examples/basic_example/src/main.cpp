#include <Arduino.h>
#include <Wire.h>
#include <MUFFINS_MCP9808.h>
#include <MUFFINS_Heating_System.h>
#include "MUFFINS_ADC_Voltage.h"

// Teleplot is a VsCode extension that allows you to plot data
const bool TELEPLOT_ENABLED = true;

const int SENSOR_POWER_ENABLE_PIN = 17;
const int WIRE0_SCL = 1;
const int WIRE0_SDA = 0;

ADC_Voltage battery_voltage;
ADC_Voltage::Data battery_data;
ADC_Voltage::Config battery_voltage_config = {
    .pin = 26,
    .adc_resolution = 4095,
    .reference_voltage = 3.3,
    .R1_value = 51000,
    .R2_value = 24000};

MCP9808 mcp9808;
MCP9808::Config mcp9808_config = {
    .wire = &Wire,
    .i2c_address = 0x1F,
    .resolution = 3};

Heating_System heating_system;
Heating_System::Config heating_system_config = {
    .heater_pin = 22,
    .Kp = 800,
    .Ki = 0.0004,
    .Kd = 0,
    .Kp_limit = 10000,
    .Ki_limit = 10000,
    .Kd_limit = 10000,
    .pwm_min = 0,
    .pwm_max = 10000,
    .target_temp = 35,
};

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1000);
  }

  analogReadResolution(12);

  pinMode(SENSOR_POWER_ENABLE_PIN, OUTPUT_12MA);
  digitalWrite(SENSOR_POWER_ENABLE_PIN, HIGH);

  // Set the heater pin to output and pull it low
  pinMode(heating_system_config.heater_pin, OUTPUT_12MA);
  digitalWrite(heating_system_config.heater_pin, LOW);
  // The analog write range and frequency has to be changed for heater PWM to work properly
  analogWriteRange(10000); // Don't change this value
  analogWriteFreq(100);    // Don't change this value

  battery_voltage.begin(battery_voltage_config);

  if (Wire.setSCL(WIRE0_SCL) && Wire.setSDA(WIRE0_SDA))
  {
    Wire.begin();
  }

  if (!mcp9808.begin(mcp9808_config))
  {
    while (1)
      ;
  }

  heating_system.begin(heating_system_config);
}

void loop()
{
  mcp9808.read();
  heating_system.update(mcp9808.data.temperature);
  battery_voltage.read(battery_data);

  Heating_System::Data heating_system_data;
  heating_system.info(heating_system_data);

  if (TELEPLOT_ENABLED)
  {
    Serial.print(">Temperature (C):");
    Serial.println(mcp9808.data.temperature);

    Serial.print(">Heater PWM (%):");
    Serial.println((heating_system_data.heater_pwm / heating_system_config.pwm_max) * 100);

    Serial.print(">P (PWM):");
    Serial.println(heating_system_data.proportional_term * heating_system_config.Kp);

    Serial.print(">I (PWM):");
    Serial.println(heating_system_data.integral_term * heating_system_config.Ki);

    Serial.print(">D (PWM):");
    Serial.println(heating_system_data.derivative_term * heating_system_config.Kd);

    Serial.print(">Target Temperature (C):");
    Serial.println(heating_system_data.target_temp);

    Serial.print(">Battery Voltage (V):");
    Serial.println(battery_data.voltage);
  }
  else
  {
    Serial.print("Temperature (C):");
    Serial.println(mcp9808.data.temperature);

    Serial.print("Heater PWM (%):");
    Serial.println((heating_system_data.heater_pwm / heating_system_config.pwm_max) * 100);

    Serial.print("P (PWM):");
    Serial.println(heating_system_data.proportional_term * heating_system_config.Kp);

    Serial.print("I (PWM):");
    Serial.println(heating_system_data.integral_term * heating_system_config.Ki);

    Serial.print("D (PWM):");
    Serial.println(heating_system_data.derivative_term * heating_system_config.Kd);

    Serial.print("Target Temperature (C):");
    Serial.println(heating_system_data.target_temp);

    Serial.print("Battery Voltage (V):");
    Serial.println(battery_data.voltage);
  }
  delay(10);
}