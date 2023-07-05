/*
  NAME:
  gbjTMP102

  DESCRIPTION:
  Library for temperature sensor TMP102 on two-wire (I2C) bus.
  - Configuration of the sensor is based on preliminary preparation of the
  configuration register value stored in the internal status structure and
  finally write this value to the sensor.

  LICENSE:
  This program is free software; you can redistribute it and/or modify
  it under the terms of the license GNU GPL v3
  http://www.gnu.org/licenses/gpl-3.0.html (related to original code) and MIT
  License (MIT) for added code.

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
  enum Addresses
  {
    // ADD0 pin connected to GND pin (default address)
    ADDRESS_GND = 0x48,
    // ADD0 pin connected to Vcc pin
    ADDRESS_VCC = 0x49,
    // ADD0 pin connected to SDA pin
    ADDRESS_SDA = 0x4A,
    // ADD0 pin connected to SCL pin
    ADDRESS_SCL = 0x4B,
  };

  gbj_tmp102(ClockSpeeds clockSpeed = ClockSpeeds::CLOCK_100KHZ,
             uint8_t pinSDA = 4,
             uint8_t pinSCL = 5)
    : gbj_twowire(clockSpeed, pinSDA, pinSCL){};

  /*
    Initialize two wire bus and sensor with parameters stored by constructor.

    DESCRIPTION:
    The method sanitizes and stores input parameters to the class instance
    object, which determines the operation modus of the sensor.

    PARAMETERS:
    address - One of 4 possible 7 bit addresses of the sensor determined by the
    connection of the ADD0 pin. If it is not some of expected values, it
    fallbacks to default value.
      - Data type: Addresses
      - Default value: ADDRESS_GND
      - Limited range: ADDRESS_GND, ADDRESS_VCC, ADDRESS_SDA, ADDRESS_SCL

    RETURN: Result code
  */
  inline ResultCodes begin(Addresses address = Addresses::ADDRESS_GND)
  {
    if (isError(gbj_twowire::begin()))
    {
      return getLastResult();
    }
    // General call for resetting to power-up state
    if (isError(busGeneralReset()))
    {
      return setLastResult(ResultCodes::ERROR_RESET);
    }
    if (isError(setAddress(address)))
    {
      return getLastResult();
    }
    if (isError(getConfiguration()))
    {
      return getLastResult();
    }
    setUseValuesMax();
    wait(getConversionTimeTemp());
    return getLastResult();
  }

  /*
    Reset sensor.

    DESCRIPTION:
    The method resets configuration register to power-up value.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes reset()
  {
    // Write default temperature alert limits
    setAlertLow();
    setAlertHigh();
    // Write initial value to configuration register
    status_.configRegister = Params::PARAM_RESET;
    return setConfiguration();
  }

  /*
    Measure temperature in continuous mode.

    DESCRIPTION:
    The method reads the temperature register with value from recent conversion.
    - The continuous mode should be configured before the measurement, if in
    meanwhile the one-shot mode has been set, but just once.
    - Right after power-up or resetting of the sensor the continuous mode is set
    by default.

    PARAMETERS: none

    RETURN: Temperature in centigrades
  */
  inline float measureTemperature()
  {
    return readTemperature(Commands::CMD_REG_TEMP);
  }

  /*
    Measure temperature in one-shot mode.

    DESCRIPTION:
    The method configures shutdown mode and one-shot conversion. It waits until
    conversion finishes and returns ambient temperature in centigrades.
    - The method is useful at very long periods between measurements in order to
      save power consumption.

    PARAMETERS: none

    RETURN: Temperature in centigrades
  */
  inline float measureTemperatureOneshot()
  {
    // Configure sensor
    configShutdownMode();
    configOneshotMode();
    if (isError(setConfiguration()))
    {
      return getLastResult();
    }
    // Wait for conversion
    setDelayReceive(getConversionTimeTemp());
    setTimestamp();
    do
    {
      if (isError(getConfiguration()))
      {
        return getLastResult();
      }
    } while (!getOneshotMode());
    setDelayReceive(0);
    return measureTemperature();
  }

  // Setters
  inline void setUseValuesTyp() { status_.useValuesTyp = true; }
  inline void setUseValuesMax() { status_.useValuesTyp = false; }
  /*

    Write configuration register value to the sensor.

    DESCRIPTION:
    The method sends prepared configuration register value to the sensor's
    configuration register.

    PARAMETERS: None

    RETURN: Result code
  */
  inline ResultCodes setConfiguration()
  {
    return sensorSend(Commands::CMD_REG_CONF, status_.configRegister);
  }
  // Preparation of configuration register value
  inline void configExtendedMode()
  {
    status_.configRegister |= (1 << ConfigBits::CONFIG_EM);
  }
  inline void configNormalMode()
  {
    status_.configRegister &= ~(1 << ConfigBits::CONFIG_EM);
  }
  inline void configShutdownMode()
  {
    status_.configRegister |= (1 << ConfigBits::CONFIG_SD);
  };
  inline void configContinuousMode()
  {
    status_.configRegister &= ~(1 << ConfigBits::CONFIG_SD);
  }
  inline void configInterruptMode()
  {
    status_.configRegister |= (1 << ConfigBits::CONFIG_TM);
  }
  inline void configThermostatMode()
  {
    status_.configRegister &= ~(1 << ConfigBits::CONFIG_TM);
  }
  inline void configAlertActiveHigh()
  {
    status_.configRegister |= (1 << ConfigBits::CONFIG_POL);
  }
  inline void configAlertActiveLow()
  {
    status_.configRegister &= ~(1 << ConfigBits::CONFIG_POL);
  }
  inline void configOneshotMode()
  {
    status_.configRegister |= (1 << ConfigBits::CONFIG_OS);
  }
  inline void configConversionRate_025hz()
  {
    configConversionRate(ConversionRate::CONVERSION_RATE_025HZ);
  }
  inline void configConversionRate_1hz()
  {
    configConversionRate(ConversionRate::CONVERSION_RATE_1HZ);
  }
  inline void configConversionRate_4hz()
  {
    configConversionRate(ConversionRate::CONVERSION_RATE_4HZ);
  }
  inline void configConversionRate_8hz()
  {
    configConversionRate(ConversionRate::CONVERSION_RATE_8HZ);
  }
  inline void configConversionPeriod_4000ms() { configConversionRate_025hz(); }
  inline void configConversionPeriod_1000ms() { configConversionRate_1hz(); }
  inline void configConversionPeriod_250ms() { configConversionRate_4hz(); }
  inline void configConversionPeriod_125ms() { configConversionRate_8hz(); }
  inline void configFaults1() { configFaultQueue(FaultQueue::FAULT_QUEUE_1); }
  inline void configFaults2() { configFaultQueue(FaultQueue::FAULT_QUEUE_2); }
  inline void configFaults4() { configFaultQueue(FaultQueue::FAULT_QUEUE_4); }
  inline void configFaults6() { configFaultQueue(FaultQueue::FAULT_QUEUE_6); }
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

    RETURN: Result code
  */
  inline ResultCodes setAlertLow(float temperature = Params::PARAM_ALERT_LOW)
  {
    return sensorSend(Commands::CMD_REG_TLOW,
                      calculateTemperature(temperature));
  }
  inline ResultCodes setAlertHigh(float temperature = Params::PARAM_ALERT_HIGH)
  {
    return sensorSend(Commands::CMD_REG_THIGH,
                      calculateTemperature(temperature));
  }

  // Getters
  /*
    Read configuration register value from the sensor.

    DESCRIPTION:
    The method reads configuration register and its value stores in the instance
    object, so that it caches it.

    PARAMETERS: none

    RETURN: Result code
  */
  inline ResultCodes getConfiguration()
  {
    return sensorRead(Commands::CMD_REG_CONF, &status_.configRegister);
  }
  inline bool getExtendedMode()
  {
    return status_.configRegister & (1 << ConfigBits::CONFIG_EM);
  }
  inline bool getNormalMode() { return !getExtendedMode(); }
  inline bool getAlert()
  {
    if (isError(getConfiguration()))
    {
      return false;
    }
    return status_.configRegister & (1 << ConfigBits::CONFIG_AL);
  }
  inline bool getShutdownMode()
  {
    return status_.configRegister & (1 << ConfigBits::CONFIG_SD);
  }
  inline bool getContinuousMode() { return !getShutdownMode(); }
  inline bool getInterruptMode()
  {
    return status_.configRegister & (1 << ConfigBits::CONFIG_TM);
  }
  inline bool getThermostatMode() { return !getInterruptMode(); }
  inline float getAlertLow() { return readTemperature(Commands::CMD_REG_TLOW); }
  inline float getAlertHigh()
  {
    return readTemperature(Commands::CMD_REG_THIGH);
  }
  inline bool getAlertActiveHigh()
  {
    return status_.configRegister & (1 << ConfigBits::CONFIG_POL);
  }
  inline bool getAlertActiveLow() { return !getAlertActiveHigh(); }
  inline bool getOneshotMode()
  {
    return status_.configRegister & (1 << ConfigBits::CONFIG_OS);
  }
  inline float getConversionRate()
  {
    float rates[] = {
      0.25,
      1.0,
      4.0,
      8.0,
    };
    uint8_t index = (status_.configRegister >> ConfigBits::CONFIG_CR0) & B11;
    return rates[index];
  }
  inline uint16_t getConversionPeriod()
  {
    uint16_t periods[] = {
      4000,
      1000,
      250,
      125,
    };
    uint8_t index = (status_.configRegister >> ConfigBits::CONFIG_CR0) & B11;
    return periods[index];
  }
  inline uint8_t getFaults()
  {
    uint8_t faults[] = {
      1,
      2,
      4,
      6,
    };
    uint8_t index = (status_.configRegister >> ConfigBits::CONFIG_F0) & B11;
    return faults[index];
  }
  inline float getErrorValue()
  {
    return static_cast<float>(Params::PARAM_BAD_TEMP);
  }

private:
  enum Commands : uint8_t
  {
    // Activate temperature register (power-up default)
    CMD_REG_TEMP = 0x00,
    // Activate configuration register
    CMD_REG_CONF = 0x01,
    // Activate LOW alarm register
    CMD_REG_TLOW = 0x02,
    // Activate HIGH alarm register
    CMD_REG_THIGH = 0x03,
    // Value for undefined register
    CMD_REG_NONE = 0xFF,
  };
  // CR1 and CR0 bits for configuration register for conversion rate
  enum ConversionRate
  {
    // 0.25 Hz
    CONVERSION_RATE_025HZ = B00,
    // 1 Hz
    CONVERSION_RATE_1HZ = B01,
    // 4 Hz
    CONVERSION_RATE_4HZ = B10,
    // 8 Hz
    CONVERSION_RATE_8HZ = B11,
  };
  // F1 and F0 bits for configuration register for consecutive faults
  enum FaultQueue
  {
    FAULT_QUEUE_1 = B00,
    FAULT_QUEUE_2 = B01,
    FAULT_QUEUE_4 = B10,
    FAULT_QUEUE_6 = B11,
  };
  enum Timing : uint8_t
  {
    // Typical conversion time in milliseconds
    TIMING_CONVERSION_TYP = 26,
    // Maximal conversion time in milliseconds
    TIMING_CONVERSION_MAX = 35,
  };
  // Configuration bits order in config register word
  enum ConfigBits : uint8_t
  {
    // Shutdown mode
    CONFIG_SD = 8,
    // Thermostat mode
    CONFIG_TM = 9,
    // Polarity
    CONFIG_POL = 10,
    // Fault queue LSB
    CONFIG_F0 = 11,
    // Fault queue MSB
    CONFIG_F1 = 12,
    // Converter resolution LSB
    CONFIG_R0 = 13,
    // Converter resolution MSB
    CONFIG_R1 = 14,
    // One-shot conversion
    CONFIG_OS = 15,
    // Extended mode
    CONFIG_EM = 4,
    // Alert
    CONFIG_AL = 5,
    // Conversion rate LSB
    CONFIG_CR0 = 6,
    // Conversion rate MSB
    CONFIG_CR1 = 7,
  };
  enum Params
  {
    // Response from general call
    PARAM_POWERUP = 0x06,
    // Configuration register word after software reset
    PARAM_RESET = 0x60A0,
    // Temperature resolution in bits
    // Resolution in centigrades (1/16 = 0.0625)
    PARAM_TEMP_RES = 16,
    // Maximum of temperature range in centigrages
    PARAM_TEMP_MAX = 150,
    // Minimum of temperature range in centigrages
    PARAM_TEMP_MIN = -55,
    // Unreasonable wrong temperature value
    PARAM_BAD_TEMP = 999,
    // Temperature alert low
    PARAM_ALERT_LOW = 75,
    // Temperature alert high
    PARAM_ALERT_HIGH = 80,
  };
  struct Status
  {
    // Recent value of pointer register
    uint8_t pointerRegister;
    // Recent read or desired value of configuration register
    uint16_t configRegister;
    // Flag about using typical values from datasheet
    bool useValuesTyp;
  } status_;
  inline bool getUseValuesTyp() { return status_.useValuesTyp; }
  inline Timing getConversionTimeTemp()
  {
    return getUseValuesTyp() ? Timing::TIMING_CONVERSION_TYP
                             : Timing::TIMING_CONVERSION_MAX;
  }

  /*
    Activate register.

    DESCRIPTION:
    The method sends input command to the pointer register if needed in order
    to activate corresponding data register.

    PARAMETERS:
    cmdRegister - Command for selecting internal register.
      - Data type: Commands
      - Default value: none
      - Limited range: CMD_REG_TEMP ~ CMD_REG_THIGH

    RETURN: Result code
  */
  inline ResultCodes activateRegister(Commands cmdRegister)
  {
    setLastResult();
    if (status_.pointerRegister != cmdRegister)
    {
      status_.pointerRegister = cmdRegister;
      busSend(status_.pointerRegister);
    }
    return getLastResult();
  }

  /*
    Read word from a sensor's register.

    DESCRIPTION:
    - The method activates the pointer register by sending input command.
    - Then the method reads a word from the sensor.
    - All data byte are transmitted MSB first.

    PARAMETERS:
    cmdRegister - Command for selecting internal register by sending it to the
    pointer register.
      - Data type: Commands
      - Default value: none
      - Limited range: CMD_REG_TEMP ~ CMD_REG_THIGH

    data - Pointer to byte buffer array for writing a word from the sensor
      - Data type: pointer to byte array
      - Default value: none
      - Limited range: none

    RETURN: Result code
  */
  inline ResultCodes sensorRead(Commands cmdRegister, uint16_t *data)
  {
    if (isError(activateRegister(cmdRegister)))
    {
      return getLastResult();
    }
    return busReceive(
      reinterpret_cast<uint8_t *>(data), sizeof(uint16_t), REVERSE);
  }

  /*
    Write word to a sensor's register.

    DESCRIPTION:
    - The method activates the pointer register by sending input command.
    - Then the method writes a word to the sensor.
    - All data byte are transmitted MSB first.

    PARAMETERS:
    cmdRegister - Command for selecting internal register by sending it to the
    pointer register.
      - Data type: Commands
      - Default value: none
      - Limited range: CMD_REG_TEMP ~ CMD_REG_THIGH

    data - A word so be sent to the sensor
      - Data type: non-negative integer
      - Default value: none
      - Limited range: 0 ~ 65535

    RETURN: Result code
  */
  inline ResultCodes sensorSend(Commands cmdRegister, uint16_t data)
  {
    status_.pointerRegister = cmdRegister;
    return busSend(cmdRegister, data);
  }

  /*
    Calculate temperature or temperature register value.

    DESCRIPTION:
    The overloaded method wraps a formula for calculating temperature in
    centigrades from 16-bit word or vice-versa.
    - The methods are suitable for storing temperatures in EEPROM as binary word
      instead of as float numbers.

    PARAMETERS:
    wordMeasure - Measured binary word.
      - Data type: integer
      - Default value: none
      - Limited range: 0 ~ 0xFFFF

    temperature - Temperature in centigrades.
      - Data type: float
      - Default value: none
      - Limited range: -55.0 ~ 150.0

    RETURN: Temperature in centigrades or binary word.
  */
  inline float calculateTemperature(int16_t wordMeasure)
  {
    // Negative - Binary twos compliance
    if (wordMeasure & 0x8000)
    {
      wordMeasure = -(~wordMeasure + 1);
    }
    // Only data bits
    wordMeasure >>= (getExtendedMode() ? 3 : 4);
    return static_cast<float>(wordMeasure) / Params::PARAM_TEMP_RES;
  }
  inline int16_t calculateTemperature(float temperature)
  {
    temperature = constrain(temperature,
                            static_cast<float>(Params::PARAM_TEMP_MIN),
                            static_cast<float>(Params::PARAM_TEMP_MAX));
    int16_t wordMeasure = round(temperature * Params::PARAM_TEMP_RES);
    wordMeasure <<= getExtendedMode() ? 3 : 4;
    return wordMeasure;
  }

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

    RETURN: Temperature limit in centigrades or bad measure value
  */
  inline float readTemperature(Commands cmdRegister)
  {
    uint16_t data;
    if (isError(sensorRead(cmdRegister, &data)))
    {
      setLastResult(ResultCodes::ERROR_MEASURE);
      return getErrorValue();
    }
    return calculateTemperature(static_cast<int16_t>(data));
  }

  /*
    Update conversion rate bits in configuration register value.

    DESCRIPTION:
    The method updates CR1 and CR0 bits in configuration register value
    demanding particular conversion rate.

    PARAMETERS:
    conversionRate - Value of pair of CR1 and CR0 bits. It fallbacks to least
    significant 2 bits.
      - Data type: ConversionRate
      - Default value: none
      - Limited range: enumeration

    RETURN: none
  */
  inline void configConversionRate(ConversionRate conversionRate)
  {
    // Clear CR1, CR0 bits
    status_.configRegister &= ~(B11 << ConfigBits::CONFIG_CR0);
    // Set CR1, CR0 bits
    status_.configRegister |= (conversionRate << ConfigBits::CONFIG_CR0);
  }
  /*
    Update number of consecutive faults bits in configuration register value.

    DESCRIPTION:
    The method updates F1 and F0 bits in configuration register value demanding
    particular fault queue consecutive faults for raising an alert.

    PARAMETERS:
    faults - Value of pair of F1 and F0 bits. It fallbacks to least significant
    2 bits.
      - Data type: FaultQueue
      - Default value: none
      - Limited range: enumeration

    RETURN: none
  */
  inline void configFaultQueue(FaultQueue faults)
  {
    // Clear F1, F0 bits
    status_.configRegister &= ~(B11 << ConfigBits::CONFIG_F0);
    // Set F1, F0 bits
    status_.configRegister |= (faults << ConfigBits::CONFIG_F0);
  }
};

#endif
