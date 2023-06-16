/*
  NAME:
  Using TMP102 sensor in the role of thermostat for cooling or heating.

  DESCRIPTION:
  The sketch measures temperature and controls an equipment with TMP102 sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - Connect ADD0 pin to ground in order to use default I2C address. For other
    connection determine the appropriate address in the begin() method calling.
  - Connect ALERT pin to microcontroller's digital pin 2 for sensing sensor
    alerts. For other connection change the corresponding constant accordingly.
  - Change temperature limits constants to your needs. The predefined values are
    suitable for heating sensor by a finger touch directly for experimenting
    and cool the sensor by ambient air.
  - Right after sensor power-up the alert is on because of initial state.
  - After configuration of the alerting the alert pin switches to the state
    by ambient temperature.
  - The sketch configures alerting for air conditioning simulating a fan
  control. Right after the temperature reaches or exceeds the upper temperature
  limit the alert pin goes HIGH, built-in LED turns on simulating turning on a
  fan. Right after the temperature sinks to or below the lower temperature limit
    the alert pin goes LOW, built-in LED turns ofsimulating turning off a fan.
  - Initially the alert pin is configured as active low, which means that it
    can control a heater and acts in opposite to controlling a fan. At exceeding
    upper temperature limit the alert pin goes LOW and turns off a heater. At
    sinking below lower temperature limit it turns on a heater.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_tmp102.h"

// Hardware configuration
const unsigned char PIN_TMP102_ALERT = 2;
// const unsigned char PIN_TMP102_ALERT = D3;
const unsigned char PIN_LED = LED_BUILTIN;

// Parameters
const unsigned int PERIOD_MEASURE = 3000;
const float ALERT_TEMP_LOW = 27.5;
const float ALERT_TEMP_HIGH = 30.5;

// Variables
float tempValue;
byte alert;

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
  Serial.println("---");

  // Initialize pins
  pinMode(PIN_TMP102_ALERT, INPUT);
  pinMode(PIN_LED, OUTPUT);

  // Initialize sensor - Default address
  if (sensor.isError(sensor.begin()))
  {
    errorHandler("Begin");
    return;
  }
  // Address
  Serial.print("Address: 0x");
  Serial.println(sensor.getAddress(), HEX);
  Serial.println("---");

  // Set configuration parameters
  sensor.configThermostatMode();
  sensor.configConversionRate_1hz(); // Measure once per second
  sensor.configFaults2(); // Two consecutive faults for alerting
  sensor.configAlertActiveHigh(); // Control air conditioner
  // sensor.configAlertActiveLow();  // Control heater
  if (sensor.isError(sensor.setConfiguration()))
  {
    errorHandler("Configuration");
    return;
  }
  // Set temperature limits
  if (sensor.isError(sensor.setAlertLow(ALERT_TEMP_LOW)))
  {
    errorHandler("Alert LOW");
    return;
  }
  if (sensor.isError(sensor.setAlertHigh(ALERT_TEMP_HIGH)))
  {
    errorHandler("Alert HIGH");
    return;
  }
  Serial.println("Alert LOW: " + String(sensor.getAlertLow()) + " °C");
  Serial.println("Alert HIGH: " + String(sensor.getAlertHigh()) + " °C");
  Serial.println("---");
}

void loop()
{
  // Measure
  tempValue = sensor.measureTemperature();
  // tempValue = sensor.measureTemperatureOneshot();
  if (sensor.isError())
  {
    errorHandler("Temperature");
  }
  Serial.println("Temperature = " + String(tempValue) + " °C");
  // Evaluate alert
  alert = digitalRead(PIN_TMP102_ALERT);
  digitalWrite(PIN_LED, alert);
  // Read alert status
  if (sensor.isError(sensor.getConfiguration()))
  {
    errorHandler("Configuration");
  }
  Serial.println("Alert on pin = " + String(alert ? "ON" : "OFF"));
  Serial.println("Alert in reg = " + String(sensor.getAlert() ? "ON" : "OFF"));
  Serial.println();
  delay(PERIOD_MEASURE);
}
