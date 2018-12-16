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
#define SKETCH "GBJ_TMP102_LIMITS 1.0.0"

#include "gbj_tmp102.h"

// Parameters
const float ALERT_TEMP_LOW = 27.5;  // Lower limit for alerting by sensor
const float ALERT_TEMP_HIGH = 30.5;  // Upper limit for alerting by sensor

// Software configuration
gbj_tmp102 Sensor = gbj_tmp102();
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_100KHZ, D2, D1);
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_400KHZ);


void errorHandler(String location)
{
  if (Sensor.isSuccess()) return;
  Serial.print(location);
  Serial.print(" - Error: ");
  Serial.print(Sensor.getLastResult());
  Serial.print(" - ");
  switch (Sensor.getLastResult())
  {
    // General
    case gbj_tmp102::ERROR_ADDRESS:
      Serial.println("ERROR_ADDRESS");
      break;

    case gbj_tmp102::ERROR_PINS:
      Serial.println("ERROR_PINS");
      break;

    case gbj_htu21::ERROR_RCV_DATA:
      Serial.println("ERROR_RCV_DATA");
      break;

    // Arduino, Esspressif specific
#if defined(__AVR__) || defined(ESP8266) || defined(ESP32)
    case gbj_tmp102::ERROR_BUFFER:
      Serial.println("ERROR_BUFFER");
      break;

    case gbj_tmp102::ERROR_NACK_DATA:
      Serial.println("ERROR_NACK_DATA");
      break;

    case gbj_tmp102::ERROR_NACK_OTHER:
      Serial.println("ERROR_NACK_OTHER");
      break;

    // Particle specific
#elif defined(PARTICLE)
    case gbj_tmp102::ERROR_BUSY:
      Serial.println("ERROR_BUSY");
      break;

    case gbj_tmp102::ERROR_END:
      Serial.println("ERROR_END");
      break;

    case gbj_tmp102::ERROR_TRANSFER:
      Serial.println("ERROR_TRANSFER");
      break;

    case gbj_tmp102::ERROR_TIMEOUT:
      Serial.println("ERROR_TIMEOUT");
      break;
#endif

    // Sensor specific
    case gbj_tmp102::ERROR_MEASURE_TEMP:
      Serial.println("ERROR_MEASURE_TEMP");
      break;

    default:
      Serial.println("Uknown error");
      break;
  }
}


void setup()
{
  Serial.begin(9600);
  Serial.println(SKETCH);
  Serial.println("Libraries:");
  Serial.println(gbj_twowire::VERSION);
  Serial.println(gbj_tmp102::VERSION);
  Serial.println("---");

  // Initialize sensor
  if (Sensor.begin()) // Use default address
  {
    errorHandler("Begin");
    return;
  }

  // Set temperature limits
  Serial.println("Limits BEFORE");
  Serial.println("Alert LOW  = " + String(Sensor.getAlertLow()) + " 'C");
  Serial.println("Alert HIGH = " + String(Sensor.getAlertHigh()) + " 'C");
  Serial.println("---");
  if (Sensor.setAlertLow(ALERT_TEMP_LOW))
  {
    errorHandler("Alert LOW");
    return;
  }
  if (Sensor.setAlertHigh(ALERT_TEMP_HIGH))
  {
    errorHandler("Alert HIGH");
    return;
  }
  Serial.println("Limits AFTER");
  Serial.println("Alert LOW  = " + String(Sensor.getAlertLow()) + " 'C");
  Serial.println("Alert HIGH = " + String(Sensor.getAlertHigh()) + " 'C");
  Serial.println("---");
  Serial.println("END");
}


void loop() {}
