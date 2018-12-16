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
  - The sketch configures alerting for air conditioning simulating a fan control.
    Right after the temperature reaches or exceeds the upper temperature limit
    the alert pin goes HIGH, built-in LED turns on simulating turning on a fan.
    Right after the temperature sinks to or below the lower temperature limit
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
#define SKETCH "GBJ_TMP102_THERMOSTAT 1.0.0"

#include "gbj_tmp102.h"


// Hardware configuration
const unsigned char PIN_TMP102_ALERT = 2;  // Alert pin sensing
// const unsigned char PIN_TMP102_ALERT = D3;  // Alert pin sensing
const unsigned char PIN_LED = LED_BUILTIN;  // Signal diode

// Parameters
const unsigned int PERIOD_MEASURE = 3000;  // Time in miliseconds between measurements
const float ALERT_TEMP_LOW = 27.5;  // Lower limit for alerting by sensor
const float ALERT_TEMP_HIGH = 30.5;  // Upper limit for alerting by sensor

// Software configuration
gbj_tmp102 Sensor = gbj_tmp102();
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_100KHZ, D2, D1);
// gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_400KHZ);
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

  // Initialize pins
  pinMode(PIN_TMP102_ALERT, INPUT);
  pinMode(PIN_LED, OUTPUT);

  // Initialize sensor
  if (Sensor.begin()) // Use default address
  {
    errorHandler("Begin");
    return;
  }

  // Set configuration parameters (usually different from initial ones)
  Sensor.configConversionRate(Sensor.CONVERSION_RATE_1HZ);
  Sensor.configFaultQueue(Sensor.FAULT_QUEUE_2);
  Sensor.configAlertActiveHigh();  // Control air conditioner
  // Sensor.configAlertActiveLow();  // Control heater
  if (Sensor.setConfiguration())
  {
    errorHandler("Configuration");
    return;
  }

  // Set temperature limits
  if (Sensor.setAlertLow(ALERT_TEMP_LOW))
  {
    errorHandler("Alert LOW");
    return;
  }
  else
  {
    Serial.println("Alert LOW  = " + String(Sensor.getAlertLow()) + " 'C");
  }
  if (Sensor.setAlertHigh(ALERT_TEMP_HIGH))
  {
    errorHandler("Alert HIGH");
    return;
  }
  else
  {
    Serial.println("Alert HIGH = " + String(Sensor.getAlertHigh()) + " 'C");
  }
  Serial.println("---");
}


void loop()
{
  if (Sensor.isError()) return;

  // Measure
  tempValue = Sensor.measureTemperature();
  // tempValue = Sensor.measureTemperatureOneshot();
  if (Sensor.isSuccess())
  {
    Serial.println("Temperature = " + String(tempValue) + " 'C");
  }
  else
  {
    Serial.println();
    errorHandler("Temperature");
  }

  // Evaluate alert
  digitalWrite(PIN_LED, digitalRead(PIN_TMP102_ALERT));
  Serial.println("Alert = " + String(Sensor.getAlert() ? "ON" : "OFF"));

  Serial.println();
  delay(PERIOD_MEASURE);
}
