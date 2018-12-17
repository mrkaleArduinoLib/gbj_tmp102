#include "gbj_tmp102.h"
const String gbj_tmp102::VERSION = "GBJ_TMP102 1.0.0";


uint8_t gbj_tmp102::begin(uint8_t address)
{
  if (gbj_twowire::begin()) return getLastResult();
  _status.pointerRegister = CMD_REG_NONE;
  if (busGeneralReset()) return setLastResult(ERROR_RESET);
  if (setAddress(address)) return getLastResult();
  setUseValuesMax();
  if (reset()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::reset()
{
  if (_status.pointerRegister != CMD_REG_NONE && busGeneralReset()) return setLastResult(ERROR_RESET);
  if (getConfiguration()) return getLastResult();
  if (_status.configRegister != PARAM_RESET) return setLastResult(ERROR_RESET);
  wait(getUseValuesTyp() ? TIMING_CONVERSION_TYP : TIMING_CONVERSION_MAX);
  return getLastResult();
}


float gbj_tmp102::readTemperature(uint8_t cmdRegister)
{
  uint8_t data[2];
  bool origBusStop = getBusStop();
  do
  {
    setBusRpte();
    if (activateRegister(cmdRegister)) break;
    setBusStopFlag(origBusStop);
    if (busReceive(data, 2)) break;
    int16_t wordMeasure;
    wordMeasure = data[0] << 8;  // MSB
    wordMeasure |= data[1];  // LSB
    if (getExtendedMode()) wordMeasure |= B1;
    return calculateTemperature(wordMeasure);
  } while(false);
  setLastResult(ERROR_MEASURE_TEMP);
  return getErrorT();
}


float gbj_tmp102::measureTemperatureOneshot()
{
  // Configure sensor
  configShutdownMode();
  configOneshotMode();
  if (setConfiguration()) return getLastResult();
  // Wait for conversion
  setDelayReceive(getUseValuesTyp() ? TIMING_CONVERSION_TYP : TIMING_CONVERSION_MAX);
  setTimestampReceive();
  do
  {
    if (getConfiguration()) return getLastResult();
  }
  while(!getOneshotMode());
  resetDelayReceive();
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
  float temperature = (float) wordMeasure / PARAM_TEMP_RES;
  return temperature;
}


int16_t gbj_tmp102::calculateTemperature(float temperature)
{
  if (temperature > (float) PARAM_TEMP_MAX) temperature = (float) PARAM_TEMP_MAX;
  if (temperature < (float) PARAM_TEMP_MIN) temperature = (float) PARAM_TEMP_MIN;
  int16_t wordMeasure = round(temperature * PARAM_TEMP_RES);
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


uint8_t gbj_tmp102::setConfiguration()
{
  if (sensorSend(CMD_REG_CONF, _status.configRegister)) return getLastResult();
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
  if (sensorSend(CMD_REG_TLOW, calculateTemperature(temperature))) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::setAlertHigh(float temperature)
{
  if (sensorSend(CMD_REG_THIGH, calculateTemperature(temperature))) return getLastResult();
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
