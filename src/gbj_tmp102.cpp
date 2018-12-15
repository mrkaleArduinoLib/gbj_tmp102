#include "gbj_tmp102.h"
const String gbj_tmp102::VERSION = "GBJ_TMP102 1.0.0";


uint8_t gbj_tmp102::begin(uint8_t address)
{
  _status.pointerRegister = CMD_REG_NONE;
  if (busGeneralReset()) return setLastResult(ERROR_RESET);
  if (setAddress(address)) return getLastResult();
  if (reset()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::reset()
{
  if (_status.pointerRegister != CMD_REG_NONE && busGeneralReset()) return setLastResult(ERROR_RESET);
  if (getConfiguration()) return getLastResult();
  if (_status.configRegister != RESET_REG_CONFIG) return setLastResult(ERROR_RESET);
  getAlertLow();
  getAlertHigh();
  wait(TIMING_CONVERSION);
  return getLastResult();
}


float gbj_tmp102::measureTemperature()
{
  uint8_t data[2];
  setBusRpte();
  if (activateRegister(CMD_REG_TEMP)) return getLastResult();
  if (busReceive(data, 2)) return setLastResult(ERROR_MEASURE_TEMP);
  if (getConfiguration()) return getLastResult();
  int16_t wordMeasure;
  wordMeasure = data[0] << 8;  // MSB
  wordMeasure |= data[1];  // LSB
  return calculateTemperature(wordMeasure);
}


float gbj_tmp102::measureTemperatureOneshot()
{
  // Configure sensor
  configShutdownMode();
  configOneshotMode();
  if (setConfiguration(false)) return getLastResult();
  // Wait for conversion
  do
  {
    if (getConfiguration()) return getLastResult();
    wait(TIMING_CONVERSION);
  }
  while(!getOneshotMode());
  return measureTemperature();
}


float gbj_tmp102::calculateTemperature(int16_t wordMeasure)
{
  // Extended mode (13-bit resolution)
  if (wordMeasure & B1 && getExtendedMode())
  {
    wordMeasure >>= 3;
    if (wordMeasure > 0x0FFF) wordMeasure |= 0xE000;  // 2s complement
  }
  // Normal mode (12-bit resolution)
  else
  {
    wordMeasure >>= 4;
    if (wordMeasure > 0x07FF) wordMeasure |= 0xF000;  // 2s complement
  }
  float temperature = (float) wordMeasure * PARAM_TEMP_BIT;
  return temperature;
}


int16_t gbj_tmp102::calculateTemperature(float temperature)
{
  if (temperature > PARAM_TEMP_MAX) temperature = PARAM_TEMP_MAX;
  if (temperature < PARAM_TEMP_MIN) temperature = PARAM_TEMP_MIN;
  int16_t wordMeasure = round(temperature / PARAM_TEMP_BIT);
  wordMeasure <<= getExtendedMode() ? 3 : 4;
  return wordMeasure;
}


//-------------------------------------------------------------------------
// Setters
//-------------------------------------------------------------------------
uint8_t gbj_tmp102::setAddress(uint8_t address)
{
  switch (address)
  {
    case ADDRESS_GND:
    case ADDRESS_VCC:
    case ADDRESS_SDA:
    case ADDRESS_SCL:
      break;
    default:
      address = ADDRESS_GND;
      break;
  }
  return gbj_twowire::setAddress(address);
}


uint8_t gbj_tmp102::setConfiguration(bool flagWait)
{
  bool origBusStop = getBusStop();
  if (!flagWait) setBusRpte();
  if (sensorSend(CMD_REG_CONF, _status.configRegister)) return getLastResult();
  if (flagWait) wait(TIMING_CONVERSION);
  setBusStopFlag(origBusStop);
  if (getConfiguration()) return getLastResult();
  return getLastResult();
}


void gbj_tmp102::configConversionRate(uint8_t conversionRate)
{
  conversionRate &= B11;  // Use 2 least significant bits only
  _status.configRegister &= ~(B11 << CONFIG_CR0);  // Clear CR1, CR0 bits
  _status.configRegister |= (conversionRate << CONFIG_CR0);  // Set CR1, CR0 bits
}


void gbj_tmp102::configFaultQueue(uint8_t faults)
{
  faults &= B11;  // Use 2 least significant bits only
  _status.configRegister &= ~(B11 << CONFIG_F0);  // Clear F1, F0 bits
  _status.configRegister |= (faults << CONFIG_F0);  // Set F1, F0 bits
}


uint8_t gbj_tmp102::setAlertLow(float temperature)
{
  int16_t temp = calculateTemperature(temperature);
  if (temp == _status.alertLow) return getLastResult();
  if (temp > _status.alertHigh) return setLastResult(ERROR_SETUP_TEMP);
  if (sensorSend(CMD_REG_TLOW, _status.alertLow = temp)) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::setAlertHigh(float temperature)
{
  int16_t temp = calculateTemperature(temperature);
  if (temp == _status.alertHigh) return getLastResult();
  if (temp < _status.alertLow) return setLastResult(ERROR_SETUP_TEMP);
  if (sensorSend(CMD_REG_THIGH, _status.alertHigh = temp)) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::setAlerts(float temperatureLow, float temperatureHigh)
{
  bool origBusStop = getBusStop();
  if (temperatureLow > temperatureHigh)
  {
    float temp = temperatureLow;
    temperatureLow = temperatureHigh;
    temperatureHigh = temp;
  }
  setBusRpte();
  if (setAlertLow(temperatureLow)) return getLastResult();
  setBusStopFlag(origBusStop);
  if (setAlertHigh(temperatureHigh)) return getLastResult();
  return getLastResult();
}


//------------------------------------------------------------------------------
// Getters
//------------------------------------------------------------------------------
float gbj_tmp102::getAlertLow()
{
  uint8_t data[2];
  bool origBusStop = getBusStop();
  setBusRpte();
  if (activateRegister(CMD_REG_TLOW)) return getLastResult();
  setBusStopFlag(origBusStop);
  if (busReceive(data, 2)) return setLastResult(ERROR_MEASURE_TEMP);
  _status.alertLow = data[0] << 8;  // MSB
  _status.alertLow |= data[1];  // LSB
  if (getExtendedMode()) _status.alertLow |= B1;
  return calculateTemperature(_status.alertLow);
}


float gbj_tmp102::getAlertHigh()
{
  uint8_t data[2];
  bool origBusStop = getBusStop();
  setBusRpte();
  if (activateRegister(CMD_REG_THIGH)) return getLastResult();
  setBusStopFlag(origBusStop);
  if (busReceive(data, 2)) return setLastResult(ERROR_MEASURE_TEMP);
  _status.alertHigh = data[0] << 8;  // MSB
  _status.alertHigh |= data[1];  // LSB
  if (getExtendedMode()) _status.alertHigh |= B1;
  return calculateTemperature(_status.alertHigh);
}


uint8_t gbj_tmp102::getConfiguration()
{
  uint8_t data[2];
  bool origBusStop = getBusStop();
  setBusRpte();
  if (activateRegister(CMD_REG_CONF)) return getLastResult();
  setBusStopFlag(origBusStop);
  if (busReceive(data, 2)) return getLastResult();
  _status.configRegister = data[0] << 8;  // MSB
  _status.configRegister |= data[1];  // LSB
  return getLastResult();
}


//------------------------------------------------------------------------------
// Auxilliary methods
//------------------------------------------------------------------------------
uint8_t gbj_tmp102::activateRegister(uint8_t cmdRegister)
{
  if (_status.pointerRegister != cmdRegister)
  {
    if (busSend(cmdRegister)) return getLastResult();
    _status.pointerRegister = cmdRegister;
  }
  return getLastResult();
}


uint8_t gbj_tmp102::sensorSend(uint16_t command, uint16_t data)
{
  if (busSend(command, data)) return getLastResult();
  _status.pointerRegister = command;
  return getLastResult();
}
