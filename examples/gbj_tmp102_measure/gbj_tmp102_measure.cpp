/*
  NAME:
  Using TMP102 sensor for basic temperature measurement.

  DESCRIPTION:
  The sketch measures temperature with TMP102 sensor.
  - Connect sensor's pins to microcontroller's I2C bus as described in README.md
    for used platform accordingly.
  - Connect ADD0 pin to ground in order to use default I2C address. For other
    connection determine the appropriate address in the begin() method calling.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the MIT License (MIT).

  CREDENTIALS:
  Author: Libor Gabaj
*/
#include "gbj_tmp102.h"

const unsigned int PERIOD_MEASURE = 3000;
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

  // Print header
  Serial.println("Temperature (°C)");
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
  else
  {
    Serial.println(tempValue, 4);
  }
  delay(PERIOD_MEASURE);
}
