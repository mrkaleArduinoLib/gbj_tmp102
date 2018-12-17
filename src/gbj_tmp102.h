/*
  NAME:
  gbjTMP102

  DESCRIPTION:
  Library for temperature sensor TMP102 on two-wire (I2C) bus.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
  (related to original code) and MIT License (MIT) for added code.

  CREDENTIALS:
  Author: Libor Gabaj
  GitHub: https://github.com/mrkaleArduinoLib/gbj_tmp102.git
*/
#ifndef GBJ_TMP102_H
#define GBJ_TMP102_H

#include "gbj_twowire.h"


class gbj_tmp102 : public gbj_twowire
{
public:
//------------------------------------------------------------------------------
// Public constants
//------------------------------------------------------------------------------
static const String VERSION;
enum Addresses
{
  ADDRESS_GND = 0x48,  // ADD0 pin connected to GND pin (default address)
  ADDRESS_VCC = 0x49,  // ADD0 pin connected to V+ pin
  ADDRESS_SDA = 0x4A,  // ADD0 pin connected to SDA pin
  ADDRESS_SCL = 0x4B,  // ADD0 pin connected to SCL pin
};
enum ErrorCodes
{
  ERROR_RESET = 255,  // Sensor reset failure
  ERROR_MEASURE_TEMP = 254,  // Measuring temperature failure
};
enum ConvertionRate
{
  CONVERSION_PERIOD_4000MS = B00,  // 0.25 Hz
  CONVERSION_PERIOD_1000MS = B01,  // 1 Hz
  CONVERSION_PERIOD_250MS = B10,  // 4 Hz
  CONVERSION_PERIOD_125MS = B11,  // 8 Hz
  CONVERSION_RATE_025HZ = B00,  // 0.25 Hz
  CONVERSION_RATE_1HZ = B01,  // 1 Hz
  CONVERSION_RATE_4HZ = B10,  // 4 Hz
  CONVERSION_RATE_8HZ = B11,  // 8 Hz
};  // CR1 and CR0 bits for configuration register for conversion rate
enum FaultQueue
{
  FAULT_QUEUE_1 = B00,
  FAULT_QUEUE_2 = B01,
  FAULT_QUEUE_4 = B10,
  FAULT_QUEUE_6 = B11,
};  // F1 and F0 bits for configuration register for consecutive faults


//------------------------------------------------------------------------------
// Public methods
//------------------------------------------------------------------------------
/*
  Constructor taken from parent class.
*/
gbj_tmp102(uint32_t clockSpeed = CLOCK_100KHZ, uint8_t pinSDA = 4, uint8_t pinSCL = 5) \
: gbj_twowire(clockSpeed, pinSDA, pinSCL) {};


/*
  Initialize two wire bus and sensor with parameters stored by constructor.

  DESCRIPTION:
  The method sanitizes and stores input parameters to the class instance object,
  which determines the operation modus of the sensor.

  PARAMETERS:
  address - One of 4 possible 7 bit addresses of the sensor determined by
            the connection of the ADD0 pin. If it is not some of expected values,
            it fallbacks to default value.
            - Data type: non-negative integer
            - Default value: ADDRESS_GND
            - Limited range: ADDRESS_GND, ADDRESS_VCC, ADDRESS_SDA, ADDRESS_SCL

  RETURN:
  Result code.
*/
uint8_t begin(uint8_t address = ADDRESS_GND);


/*
  Reset sensor.

  DESCRIPTION:
  The method resets all internal registers to power-up values by using general
  call with reset code.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t reset();


/*
  Measure temperature in one-shot mode.

  DESCRIPTION:
  The method configures shutdown mode and one-shot conversion. It waits until
  conversion finishes and returns ambient temperature in centigrades.
  - The method is useful at very long periods between measurements in order to
    save power consumption.

  PARAMETERS: none

  RETURN:
  Temperature in centigrades or error code ERROR_MEASURE_TEMP.
*/
float measureTemperatureOneshot();


//------------------------------------------------------------------------------
// Public setters - they usually return result code.
//------------------------------------------------------------------------------
inline void setUseValuesTyp() { _status.useValuesTyp = true; };
inline void setUseValuesMax() { _status.useValuesTyp = false; };
/*
  Write configuration register value to the sensor.

  DESCRIPTION:
  The method sends prepared configuration register value to the sensor's
  configuration register.

  PARAMETERS: None

  RETURN:
  Result code.
*/
uint8_t setConfiguration();
// Preparation of configuration register value
inline void configExtendedMode() { _status.configRegister |= (1 << CONFIG_EM); };
inline void configNormalMode() { _status.configRegister &= ~(1 << CONFIG_EM); };
inline void configShutdownMode() { _status.configRegister |= (1 << CONFIG_SD); };
inline void configContinuousMode() { _status.configRegister &= ~(1 << CONFIG_SD); };
inline void configInterruptMode() { _status.configRegister |= (1 << CONFIG_TM); };
inline void configThermostatMode() { _status.configRegister &= ~(1 << CONFIG_TM); };
inline void configAlertActiveHigh() { _status.configRegister |= (1 << CONFIG_POL); };
inline void configAlertActiveLow() { _status.configRegister &= ~(1 << CONFIG_POL); };
inline void configOneshotMode() { _status.configRegister |= (1 << CONFIG_OS); };


/*
  Update conversion rate bits in configuration register value.

  DESCRIPTION:
  The method updates CR1 and CR0 bits in configuration register value demanding
  particular conversion rate.

  PARAMETERS:
  conversionRate - Value of pair of CR1 and CR0 bits. It fallbacks to least
                   significant 2 bits.
                   - Data type: non-negative integer
                   - Default value: none
                   - Limited range: CONVERSION_RATE_025HZ ~ CONVERSION_RATE_8HZ
                                 or CONVERSION_PERIOD_4000MS ~ CONVERSION_PERIOD_125MS

  RETURN: none
*/
void configConversionRate(uint8_t conversionRate);


/*
  Update number of consecutive faults bits in configuration register value.

  DESCRIPTION:
  The method updates F1 and F0 bits in configuration register value demanding
  particular fault queue consecutive faults for raising an alert.

  PARAMETERS:
  faults - Value of pair of F1 and F0 bits. It fallbacks to least
           significant 2 bits.
           - Data type: non-negative integer
           - Default value: none
           - Limited range: FAULT_QUEUE_1 ~ FAULT_QUEUE_6

  RETURN: none
*/
void configFaultQueue(uint8_t faults);


/*
  Read high or low temperature limit from the sensor.

  DESCRIPTION:
  The particular method calculates temperature limit raw value from input
  temperature in centigrades according to the extended mode taken from
  configuration register and writes it to the sensor.

  PARAMETERS:
  temperature - Temperature limit in centigrades.
                - Data type: float
                - Default value: none
                - Limited range: -55.0 ~ 150.0

  RETURN:
  Result code.
*/
uint8_t setAlertLow(float temperature);
uint8_t setAlertHigh(float temperature);
uint8_t setAlerts(float temperatureLow, float temperatureHigh);



//------------------------------------------------------------------------------
// Public getters
//------------------------------------------------------------------------------
inline bool getExtendedMode() { return _status.configRegister & (1 << CONFIG_EM); };
inline bool getNormalMode() { return !getExtendedMode(); };
inline bool getAlert() { return _status.configRegister & (1 << CONFIG_AL); };
inline bool getShutdownMode() { return _status.configRegister & (1 << CONFIG_SD); };
inline bool getContinuousMode() { return !getShutdownMode(); };
inline bool getInterruptMode() { return _status.configRegister & (1 << CONFIG_TM); };
inline bool getThermostatMode() { return !getInterruptMode(); };
inline bool getAlertActiveHigh() { return _status.configRegister & (1 << CONFIG_POL); };
inline bool getAlertActiveLow() { return !getAlertActiveHigh(); };
inline bool getOneshotMode() { return _status.configRegister & (1 << CONFIG_OS); };
inline uint8_t getConversionRate() { return (_status.configRegister >> CONFIG_CR0) & B11; };
inline uint8_t getFaultQueue() { return (_status.configRegister >> CONFIG_F0) & B11; };
inline float getErrorT() { return (float) PARAM_BAD_TEMP; };
inline float measureTemperature() { return readTemperature(CMD_REG_TEMP); };
inline float getAlertLow() { return readTemperature(CMD_REG_TLOW); };
inline float getAlertHigh() { return readTemperature(CMD_REG_THIGH); };


/*
  Read configuration register value from the sensor.

  DESCRIPTION:
  The method reads configuration register and its value stores in the instance
  object, so that it caches it.

  PARAMETERS: none

  RETURN:
  Result code.
*/
uint8_t getConfiguration();


/*
  Calculate temperature or temperature register value.

  DESCRIPTION:
  The overloaded method wraps a formula for calculating temperature in centigrades
  from 16-bit word or vice-versa.
  - The methods are suitable for storing temperatures in EEPROM as binary word
    instead of as float numbers.

  PARAMETERS:
  wordMeasure - Measured binary word. If the least significant bit is set and
                there is extended mode bit set in configuration register, the
                value is considered in 13-bit resolution.
                - Data type: integer
                - Default value: none
                - Limited range: 0 ~ 0xFFFF
  temperature - Temperature in centigrades.
                - Data type: float
                - Default value: none
                - Limited range: -55.0 ~ 150.0
  RETURN:
  Temperature in centigrade, or binary word, or error code ERROR_MEASURE_TEMP.
*/
float calculateTemperature(int16_t wordMeasure);
int16_t calculateTemperature(float temperature);


private:
//------------------------------------------------------------------------------
// Private constants
//------------------------------------------------------------------------------
enum Commands
{
  CMD_REG_TEMP = 0x00,  // Activate temperature register
  CMD_REG_CONF = 0x01,  // Activate configuration register
  CMD_REG_TLOW = 0x02,  // Activate LOW alarm register
  CMD_REG_THIGH = 0x03,  // Activate HIGH alarm register
  CMD_REG_NONE = 0xFF,  // Value for undefined register
};
enum Timing
{
  TIMING_CONVERSION_TYP = 26,  // Typical conversion time in milliseconds
  TIMING_CONVERSION_MAX = 35,  // Maximal conversion time in milliseconds
};
enum ConfigBits
{
  CONFIG_EM = 4,  // Extended mode
  CONFIG_AL = 5,  // Alert
  CONFIG_CR0 = 6,  // Conversion rate
  CONFIG_CR1 = 7,  // Conversion rate
  CONFIG_SD = 8,  // Shutdown mode
  CONFIG_TM = 9,  // Thermostat mode
  CONFIG_POL = 10,  // Polarity
  CONFIG_F0 = 11,  // Fault queue
  CONFIG_F1 = 12,  // Fault queue
  CONFIG_R0 = 13,  // Converter resolution
  CONFIG_R1 = 14,  // Converter resolution
  CONFIG_OS = 15,  // One-shot conversion
};  // Configuration bits order in config register word
enum Params
{
  PARAM_RESET = 0x60A0,  // Configuration register word after software reset
  PARAM_TEMP_RES = 16,  // Temperature resolution in bits per centigrade (1/0.0625)
  PARAM_TEMP_MAX = 150,  // Maximum of temperature range in centigrages
  PARAM_TEMP_MIN = -55,  // Minimum of temperature range in centigrages
  PARAM_BAD_TEMP = 999,  // Unreasonable wrong temperature value
};

//------------------------------------------------------------------------------
// Private attributes
//------------------------------------------------------------------------------
struct
{
  uint8_t pointerRegister;  // Recent value of pointer register
  uint16_t configRegister;  // Recent read or desired value of configuration register
  bool useValuesTyp;  // Flag about using typical values from datasheet
} _status;


//------------------------------------------------------------------------------
// Private methods - they return result code if not stated else
//------------------------------------------------------------------------------
inline bool getUseValuesTyp() { return _status.useValuesTyp; };
uint8_t setAddress(uint8_t address);
uint8_t sensorSend(uint16_t command, uint16_t data);


/*
  Activate register.

  DESCRIPTION:
  The method sends input command to the pointer register if needed in order to
  activate corresponding data register.

  PARAMETERS:
  cmdRegister - Command for selecting internal register.
                - Data type: non-negative integer
                - Default value: none
                - Limited range: CMD_REG_TEMP ~ CMD_REG_THIGH
  RETURN:
  Result code.
*/
uint8_t activateRegister(uint8_t cmdRegister);


/*
  Read temperature value or limits from the sensor.

  DESCRIPTION:
  The method reads temperature or THIGH, or TLOW register and calculates its
  temperature in centigrades according to the extended mode taken from
  configuration register.

  PARAMETERS:
  cmdRegister - Command for particular alert limit internal register.
                - Data type: non-negative integer
                - Default value: none
                - Limited range: CMD_REG_TEMP, CMD_REG_THIGH

  RETURN:
  Temperature limit in centigrades or error code ERROR_MEASURE_TEMP.
*/
float readTemperature(uint8_t cmdRegister);

};

#endif
