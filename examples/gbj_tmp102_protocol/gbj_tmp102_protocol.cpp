/*
  NAME:
  Typical communication with TMP102 sensor for a protocol decoder.

  DESCRIPTION:
  The sketch executes communication with the sensor in order to expose as much
  as possible from its functionality for testing and presenting protocol
  decoders, e.g., for logic analytical software sigrok.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - Connect ADD0 pin to ground in order to use default I2C address. For other
    connection determine the appropriate address in the begin() method calling.
  - The sketch sets usual temperature limits and some configurations for
    configuration register. Finally measures temperature in one shot.
  - The command sequence is intentionally designed for maximal suppressing
    registers caching.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_tmp102.h"

// Parameters
const float ALERT_TEMP_LOW = 23.5;
const float ALERT_TEMP_HIGH = 28.0;

// Variables
float tempValue;

// Software configuration
gbj_tmp102 sensor = gbj_tmp102();
// gbj_tmp102 sensor = gbj_tmp102(sensor.CLOCK_100KHZ, D2, D1);
// gbj_tmp102 sensor = gbj_tmp102(sensor.CLOCK_400KHZ);

void errorHandler(String location)
{
  Serial.println(sensor.getLastErrorTxt(location));
  Serial.println("---");
  return;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("BEGIN");

  // Initialize sensor - Default address
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    return;
  }

  // Set temperature limits
  if (sensor.isError(sensor.setAlertLow(ALERT_TEMP_LOW)))
  {
    errorHandler("Alert LOW");
    return;
  }
  sensor.getAlertLow();
  sensor.getAlertHigh();
  if (sensor.isError(sensor.setAlertHigh(ALERT_TEMP_HIGH)))
  {
    errorHandler("Alert HIGH");
    return;
  }

  // Set configuration
  sensor.configThermostatMode();
  sensor.configOneshotMode();
  sensor.configConversionRate_1hz(); // Measure once per second
  sensor.configFaults2(); // Two consecutive faults for alerting
  sensor.configAlertActiveHigh(); // Control air conditioner
  // sensor.configAlertActiveLow();  // Control heater
  if (sensor.isError(sensor.setConfiguration()))
  {
    errorHandler("Configuration");
    return;
  }
  // Measure
  tempValue = sensor.measureTemperature();
  // tempValue = sensor.measureTemperatureOneshot();
  if (sensor.isError())
  {
    errorHandler("Temperature");
  }
  Serial.println("END");
}

void loop() {}
