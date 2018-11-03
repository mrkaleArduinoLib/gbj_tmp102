#include "gbj_tmp102.h"
const String gbj_tmp102::VERSION = "GBJ_TMP102 1.0.0";


uint8_t gbj_tmp102::begin(uint8_t address)
{
  if (setAddress(address)) return getLastResult();
  if (reset()) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::reset()
{
  if (busGeneralReset()) return setLastResult(ERROR_RESET);
  _status.pointerRegister = CMD_REG_NONE;
  if (readConfigRegister()) return getLastResult();
  if (_status.configRegister != RESET_REG_CONFIG) return setLastResult(ERROR_RESET);
  wait(TIMING_CONVERSION);
  return getLastResult();
}


float gbj_tmp102::measureTemperature()
{
  uint8_t data[2];
  if (activateRegister(CMD_REG_TEMP)) return getLastResult();
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_MEASURE_TEMP);
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
    if (readConfigRegister()) return getLastResult();
    wait(TIMING_CONVERSION);
  }
  while(!getOneshotMode());
  return measureTemperature();
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
  if (sensorSend(CMD_REG_CONF, _status.configRegister)) return getLastResult();
  if (flagWait) wait(TIMING_CONVERSION);
  if (readConfigRegister()) return getLastResult();
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
  if (temperature > getAlertHigh()) return setLastResult(ERROR_SETUP_TEMP);
  if (sensorSend(CMD_REG_TLOW, calculateTemperature(temperature))) return getLastResult();
  return getLastResult();
}


uint8_t gbj_tmp102::setAlertHigh(float temperature)
{
  if (temperature < getAlertLow()) return setLastResult(ERROR_SETUP_TEMP);
  if (sensorSend(CMD_REG_THIGH, calculateTemperature(temperature))) return getLastResult();
  return getLastResult();
}


//------------------------------------------------------------------------------
// Getters
//------------------------------------------------------------------------------
float gbj_tmp102::getAlertLow()
{
  uint8_t data[2];
  if (activateRegister(CMD_REG_TLOW)) return getLastResult();
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_MEASURE_TEMP);
  int16_t wordMeasure;
  wordMeasure = data[0] << 8;  // MSB
  wordMeasure |= data[1];  // LSB
  if (getExtendedMode()) wordMeasure |= B1;
  return calculateTemperature(wordMeasure);
}


float gbj_tmp102::getAlertHigh()
{
  uint8_t data[2];
  if (activateRegister(CMD_REG_THIGH)) return getLastResult();
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return setLastResult(ERROR_MEASURE_TEMP);
  int16_t wordMeasure;
  wordMeasure = data[0] << 8;  // MSB
  wordMeasure |= data[1];  // LSB
  if (getExtendedMode()) wordMeasure |= B1;
  return calculateTemperature(wordMeasure);
}


//------------------------------------------------------------------------------
// Private methods
//------------------------------------------------------------------------------
float gbj_tmp102::calculateTemperature(int16_t wordMeasure)
{
  // Extended mode (13-bit resolution)
  if (wordMeasure & B1)
  {
    wordMeasure >>= 3;
    if (wordMeasure > 0x0FFF) wordMeasure |= 0xE000;  // 2s complement
  }
  else
  {
    wordMeasure >>= 4;
    if (wordMeasure > 0x07FF) wordMeasure |= 0xF000;  // 2s complement
  }
  float temperature = (float) wordMeasure * 0.0625;
  return temperature;
}


int16_t gbj_tmp102::calculateTemperature(float temperature)
{
  if (temperature > 150.0) temperature = 150.0;
  if (temperature < -55.0) temperature = -55.0;
  int16_t wordMeasure = round(temperature / 0.0625);
  wordMeasure <<= getExtendedMode() ? 3 : 4;
  return wordMeasure;
}


uint8_t gbj_tmp102::activateRegister(uint8_t cmdRegister)
{
  if (_status.pointerRegister != cmdRegister)
  {
    if (sensorSend(cmdRegister)) return getLastResult();
  }
  return getLastResult();
}


uint8_t gbj_tmp102::readConfigRegister()
{
  uint8_t data[2];
  if (activateRegister(CMD_REG_CONF)) return getLastResult();
  if (busReceive(data, sizeof(data)/sizeof(data[0]))) return getLastResult();
  _status.configRegister = data[0] << 8;  // MSB
  _status.configRegister |= data[1];  // LSB
  return getLastResult();
}


uint8_t gbj_tmp102::sensorSend(uint16_t command, uint16_t data)
{
  if (busSend(command, data)) return getLastResult();
  _status.pointerRegister = command;
  return getLastResult();
}


uint8_t gbj_tmp102::sensorSend(uint16_t data)
{
  if (busSend(data)) return getLastResult();
  _status.pointerRegister = data;
  return getLastResult();
}
