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
#define SKETCH "GBJ_TMP102_PROTOCOL 1.0.0"

#include "gbj_tmp102.h"


// Software configuration
gbj_tmp102 Sensor = gbj_tmp102();
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_100KHZ, D2, D1);
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_400KHZ);


// Parameters
const float ALERT_TEMP_LOW = 23.5;  // Lower limit for alerting by sensor
const float ALERT_TEMP_HIGH = 28.0;  // Upper limit for alerting by sensor
const byte CONVERSION_RATE = gbj_tmp102::CONVERSION_RATE_1HZ;
const byte FAULT_QUEUE = gbj_tmp102::FAULT_QUEUE_2;
float tempValue;


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

    case gbj_tmp102::ERROR_RCV_DATA:
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
    case gbj_tmp102::ERROR_RESET:
      Serial.println("ERROR_RESET");
      break;

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

  // Temperature limits
  if (Sensor.setAlertLow(ALERT_TEMP_LOW))
  {
    errorHandler("Write Alert LOW");
    return;
  }
  Sensor.getAlertLow();
  Sensor.getAlertHigh();
  if (Sensor.setAlertHigh(ALERT_TEMP_HIGH))
  {
    errorHandler("Write Alert HIGH");
    return;
  }
  Sensor.configOneshotMode();
  Sensor.configConversionRate(CONVERSION_RATE);
  Sensor.configFaultQueue(FAULT_QUEUE);
  Sensor.configAlertActiveHigh();  // Control air conditioner
  // Sensor.configAlertActiveLow();  // Control heater
  if (Sensor.setConfiguration())
  {
    errorHandler("Configuration");
    return;
  }
  // Measure
  tempValue = Sensor.measureTemperature();
  // tempValue = Sensor.measureTemperatureOneshot();
  if (Sensor.isError())
  {
    errorHandler("Temperature");
  }
  Serial.println("END");
}


void loop() {}
