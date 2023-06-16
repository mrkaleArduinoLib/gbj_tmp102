/*
  NAME:
  Setting temperature limits to TMP102 sensor.

  DESCRIPTION:
  The sketch sends temperature limits to the TMP102 sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - Connect ADD0 pin to ground in order to use default I2C address. For other
    connection determine the appropriate address in the begin() method calling.
  - Connect ALERT pin to microcontroller's digital pin 2 for sensing sensor
    alerts. For other connection change the corresponding constant accordingly.
  - Change temperature limits constants to your needs. The predefined values are
    suitable for heating sensor by a finger touch directly for experimenting
    and cool the sensor by ambient air.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_tmp102.h"

// Lower limit for alerting by sensor
const float ALERT_TEMP_LOW = 27.5;
// Upper limit for alerting by sensor
const float ALERT_TEMP_HIGH = 30.5;

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

  // Set temperature limits LOW
  Serial.print("Alert LOW BEFORE: ");
  Serial.println(sensor.getAlertLow(), 4);
  if (sensor.isError())
  {
    errorHandler("Read OLD Alert LOW");
    return;
  }
  Serial.print("Alert LOW NEW: ");
  Serial.println(ALERT_TEMP_LOW, 4);
  if (sensor.isError(sensor.setAlertLow(ALERT_TEMP_LOW)))
  {
    errorHandler("Write Alert LOW");
    return;
  }
  Serial.print("Alert LOW AFTER: ");
  Serial.println(sensor.getAlertLow(), 4);
  if (sensor.isError())
  {
    errorHandler("Read NEW Alert LOW");
    return;
  }
  Serial.println("---");

  // Set temperature limits HIGH
  Serial.print("Alert HIGH BEFORE: ");
  Serial.println(sensor.getAlertHigh(), 4);
  if (sensor.isError())
  {
    errorHandler("Read OLD Alert HIGH");
    return;
  }
  Serial.print("Alert HIGH NEW: ");
  Serial.println(ALERT_TEMP_HIGH, 4);
  if (sensor.isError(sensor.setAlertHigh(ALERT_TEMP_HIGH)))
  {
    errorHandler("Write Alert HIGH");
    return;
  }
  Serial.print("Alert HIGH AFTER: ");
  Serial.println(sensor.getAlertHigh(), 4);
  if (sensor.isError())
  {
    errorHandler("Read NEW Alert HIGH");
    return;
  }
  Serial.println("---");
}

void loop() {}
