# gbjTMP102

Library for the temperature sensors _TMP102_ with `two-wire` (also known as <abbr title='Inter-Integrated Circuit'>I2C</abbr>) bus interface.

* Sensor can have following addresses, which allow up to 4 sensors present on the same two-wire bus:
  * `0x48` for ADD0 pin connected to GND (ground)
  * `0x49` for ADD0 pin connected to V+ (power supply positive rail)
  * `0x4A` for ADD0 pin connected to SDA (serial data rail of the two-wire bus)
  * `0x4B` for ADD0 pin connected to SCL (serial clock rail of the two-wire bus)
* The library provides measured temperature in degrees of Celsius.
* For conversion among various temperature unit scales and for calculating dew point temperature use library `gbjAppHelpers`.
* The sensor in normal mode has 12-bit resolution with sensitivity `0.0625 °C/bit` within the measurement range `-55 °C ~ +128 °C`.
* The extended mode has 13-bit resolution, but the same sensitivity and it just extends the upper temperature measurement range up to `+150 °C`.
  * Switching (reconfiguration) to extended mode from normal mode or vice-versa needs a time delay cca `350 milliseconds` after it in order to settle the sensor and stabilize the temperature conversion.
  * Without the delay after switching to extended mode the reading is doubled to real temperature at first reading after switching mode.
  * Without the delay after switching to normal mode the reading is halved to real temperature at first reading after switching mode.
  * Library does not have implemented such specific delay after mode switching due to small usefulness of the extended mode.
  * Library caches pointer and configuration register.


#### Particle hardware configuration
* Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
* Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).

#### Espressif - ESP8266, ESP32 default hardware configuration
* Connect microcontroller's pin `D2` to sensor's pin **SDA** (Serial Data).
* Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).


<a id="dependency"></a>

## Dependency

#### Particle platform
* **Particle.h**: Includes alternative (C++) data type definitions.

#### Arduino platform
* **Arduino.h**: Main include file for the Arduino SDK version greater or equal to 100.
* **inttypes.h**: Integer type conversions. This header file includes the exact-width integer definitions and extends them with additional facilities provided by the implementation.
* **TwoWire**: I2C system library loaded from the file `Wire.h`.

#### Custom Libraries
* **gbjTwoWire**: I2C custom library loaded from the file `gbj_twowire.h`, which provides common bus functionality.


<a id="constants"></a>

## Constants
The library does not have specific error codes. Error codes as well as result code are inherited from the parent library [gbjTwoWire](#dependency) only. The result code and error codes can be tested in the operational code with its method `getLastResult()`, `isError()` or `isSuccess()`.


<a id="addresses"></a>

#### Sensor addresses
* **Addresses::ADDRESS\_GND**: ADD0 pin connected to GND pin (default).
* **Addresses::ADDRESS\_VCC**: ADD0 pin connected to positive power supply rail.
* **Addresses::ADDRESS\_SDA**: ADD0 pin connected to serial data rail of two-wire bus.
* **Addresses::ADDRESS\_SCL**: ADD0 pin connected to serial clock rail of two-wire bus.


### Referencing constants
In a sketch the constants can be referenced in following forms:
* **Static constant** in the form `gbj_tmp102::<enumeration>::<constant>` or shortly `gbj_tmp102::<constant>`, e.g., _gbj_tmp102::Addresses::ADDRESS\_GND_ or _gbj_tmp102::ADDRESS\_GND_.
* **Instance constant** in the form `<object>.<constant>`, e.g., _sensor.ADDRESS\_GND_.
```cpp
gbj_tmp102 sensor = gbj_tmp102(sensor.CLOCK_400KHZ)
```


<a id="configuration"></a>

## Configuration
The configuration of the sensor is realized by the configuration register, which consists of several configuration bits determining its behavior. The library stores (caches) the value of the configuration register in its instance object.

The sensor configuration implemented in the library is based on updating cached configuration value in advanced and finally to send that value to the sensor and write all configuration bits to configuration register at once in order to reduce communication on the two-wire bus in contrast to sending configuration bits to the sensor individually.

It is good practice or sometimes necessary to read the configuration register right before using getters, especially those related to particular configuration bits in order to get its current value.


<a id="interface"></a>

## Interface

#### Main
* [gbj_tmp102()](#gbj_tmp102)
* [begin()](#begin)
* [reset()](#reset)
* [measureTemperature()](#measureTemperature)
* [measureTemperatureOneshot()](#measureTemperatureOneshot)

#### Setters
* [setConfiguration()](#setConfiguration)
* [setAlertLow()](#setAlertValue)
* [setAlertHigh()](#setAlertValue)
* [setUseValuesTyp()](#setUseValues)
* [setUseValuesMax()](#setUseValues)
* [configAlertActiveLow()](#configAlertMode)
* [configAlertActiveHigh()](#configAlertMode)
* [configExtendedMode()](#configResolutionMode)
* [configNormalMode()](#configResolutionMode)
* [configShutdownMode()](#configPowerMode)
* [configContinuousMode()](#configPowerMode)
* [configInterruptMode()](#configActionMode)
* [configThermostatMode()](#configActionMode)
* [configOneshotMode()](#configOneshotMode)
* [configConversionRate_025hz()](#configConversionRate)
* [configConversionRate_1hz()](#configConversionRate)
* [configConversionRate_4hz()](#configConversionRate)
* [configConversionRate_8hz()](#configConversionRate)
* [configConversionPeriod_4000ms()](#configConversionPeriod)
* [configConversionPeriod_1000ms()](#configConversionPeriod)
* [configConversionPeriod_250ms()](#configConversionPeriod)
* [configConversionPeriod_125ms()](#configConversionPeriod)
* [configFaults1()](#configFaults)
* [configFaults2()](#configFaults)
* [configFaults4()](#configFaults)
* [configFaults6()](#configFaults)

#### Getters
* [getConfiguration()](#getConfiguration)
* [getAlertLow()](#getAlertValue)
* [getAlertHigh()](#getAlertValue)
* [getAlertActiveLow()](#getAlertMode)
* [getAlertActiveHigh()](#getAlertMode)
* [getAlert()](#getAlert)
* [getNormalMode()](#getResolutionMode)
* [getExtendedMode()](#getResolutionMode)
* [getContinuousMode()](#getPowerMode)
* [getShutdownMode()](#getPowerMode)
* [getInterruptMode()](#getActionMode)
* [getThermostatMode()](#getActionMode)
* [getOneshotMode()](#getOneshotMode)
* [getConversionRate()](#getConversionRate)
* [getConversionPeriod()](#getConversionPeriod)
* [getFaults()](#getFaults)
* [getErrorT()](#getErrorT)

Other possible setters and getters are inherited from the parent library [gbjTwoWire](#dependency) and described there.


<a id="gbj_tmp102"></a>

## gbj_tmp102()

#### Description
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it with default or specific parameters as defined at constructor of parent library [gbjTwoWire](#dependency).
* Constructor sets parameters specific to the two-wire bus in general.
* All the constructor parameters can be changed dynamically with corresponding setters later in a sketch.

#### Syntax
    gbj_tmp102(uint32_t clockSpeed, uint8_t pinSDA, uint8_t pinSCL)

#### Parameters
* **clockSpeed**: Two-wire bus clock frequency in Hertz.
  * *Valid values*:ClockSpeeds::CLOCK\_100KHZ, ClockSpeeds::CLOCK\_400KHZ
  * *Default value*: ClockSpeeds::CLOCK\_100KHZ

* **pinSDA**: Microcontroller's pin for serial data. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms for communication on the bus. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 4 (GPIO4, D2)

* **pinSCL**: Microcontroller's pin for serial clock. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  * *Valid values*: positive integer
  * *Default value*: 5 (GPIO5, D1)

#### Returns
Object performing the sensor management.
The constructor cannot return [a result or error code](#constants) directly, however, it stores them in the instance object.

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

```cpp
  gbj_tmp102 sensor = gbj_tmp102(); // It is equivalent to
  gbj_tmp102 sensor = gbj_tmp102(sensor.CLOCK_100KHZ, D2, D1)
```

[Back to interface](#interface)


<a id="begin"></a>

## begin()

#### Description
The method takes, sanitizes, and stores sensor parameters to a class instance object and initiates two-wire bus.
* The method sets parameters specific to the sensor itself.
* The method resets the sensor to its power-up state.
* The method resets the sensor by the general call software reset sending the code `0x06` to the two-wire bus at address `0x00` and reads the content of the configuration register to the library instance object.
* All the method parameters can be changed dynamically with corresponding [setters](#interface) later in a sketch.
* The method enables to set the sensor's address according to the ADDR pin, if it is wired to a microcontroller's pin.

#### Syntax
    ResultCodes begin(Addresses address)

#### Parameters
* **address**: One of two possible 7 bit addresses of the sensor corresponding to the ADDR pin connection.
  * *Valid values*: [Addresses::ADDRESS\_GND, Addresses::ADDRESS\_VCC,, Addresses::ADDRESS\_SDA Addresses::ADDRESS\_SCL](#addresses)
  * *Default value*: [gbj\_tmp102::ADDRESS\_GND](#addresses)
    * The default values is set to address corresponding to grounded ADD0 pin.
    * If input value is none of expected ones, the method fallbacks it to default address.
    * Implementing addressing allows up to 4 sensors present on the same two-wire bus.
    * Module boards with the sensor have usually ADD0 grounded and are equipped with soldering pad for reconnecting that pin to V+ rail.

#### Returns
Some of [result or error codes](#constants).

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

``` cpp
gbj_tmp102 sensor = gbj_tmp102();
setup()
{
    sensor.begin();  // It is equivalent to
    sensor.begin(sensor.ADDRESS_GND);
}
```

[Back to interface](#interface)


<a id="reset"></a>

## reset()

#### Description
The method resets the sensor by writing power-up value to the configuration register and its cache in the instance object. It  causes resetting all internal registers to their power-up values, which determine following configuration and values:
* Upper alert temperature limit 80 °C.
* Lower alert temperature limit 75 °C.
* Normal mode (12 bit).
* Conversion rate 4 Hz.
* Continuous power mode (shutdown mode off).
* Thermostat (comparator) mode.
* Alert pin active low.
* Alert active.
* One fault for fault queue.
* One-shot conversion off.

#### Syntax
    ResultCodes reset()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[begin()](#begin)

[Back to interface](#interface)


<a id="measureTemperature"></a>

## measureTemperature()

#### Description
The method measures temperature.
* The method reads the temperature register with value from recent conversion.
* The continuous mode should be configured before the measurement, if in meanwhile the one-shot mode has been set, but just once.
* Right after power-up or resetting of the sensor the continuous mode is set by default.

#### Syntax
    float measureTemperature()

#### Parameters
None

#### Returns
Temperature in centigrade or erroneous value returned by [getErrorT()](#getErrorT). The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[measureTemperatureOneshot()](#measureTemperatureOneshot)

[Back to interface](#interface)


<a id="measureTemperatureOneshot"></a>

## measureTemperatureOneshot()

#### Description
The method configures shutdown mode and one-shot conversion of the sensor. It waits until conversion finishes and returns ambient temperature in centigrade.
* The method is useful at very long periods (couple of minutes and hours) between measurements in order to save power consumption.

#### Syntax
    float measureTemperatureOneshot()

#### Parameters
None

#### Returns
Temperature in centigrade or erroneous value returned by [getErrorT()](#getErrorT). The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[measureTemperature()](#measureTemperature)

[Back to interface](#interface)


<a id="setConfiguration"></a>

## setConfiguration()

#### Description
The method writes the new content of the configuration register stored in the instance object (configuration cache) to the sensor. This content should has been prepared by methods of type `configXXX` right before.

#### Syntax
    ResultCodes setConfiguration()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="getConfiguration"></a>

## getConfiguration()

#### Description
The method reads configuration register and its value stores in the instance object, so that it caches it and enables it for corresponding getters.

#### Syntax
    ResultCodes getConfiguration()

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="setAlertValue"></a>

## setAlertLow(), setAlertHigh()

#### Description
The particular method writes either lower or upper temperature limit.
* A method for particular temperature limit does not check its relation to the related limit. It should be done in a sketch. So that, those methods allows to set high limit less than low limit.

#### Syntax
    ResultCodes setAlertLow(float temperatureLow)
    ResultCodes setAlertHigh(float temperatureHigh)

#### Parameters
* **temperatureLow**: Low temperature limit in centigrade.
  * *Valid values*: -55.0 ~ 150.0
  * *Default value*: 75.0

* **temperatureHigh**: Hight temperature limit in centigrade.
  * *Valid values*: -55.0 ~ 150.0
  * *Default value*: 80.0

#### Returns
Some of [result or error codes](#constants).

#### See also
[getAlertLow(), getAlertHigh()](#getAlertValue)

[Back to interface](#interface)


<a id="getAlertValue"></a>
## getAlertLow(), getAlertHigh()

#### Description
The particular method reads upper or lower temperature limit from the sensor.

#### Syntax
    float getAlertLow()
    float getAlertHigh()

#### Parameters
None

#### Returns
Lower or upper temperature limit in centigrade or erroneous value returned by [getErrorT()](#getErrorT). The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[setAlertLow(), setAlertHigh()](#setAlertValue)

[Back to interface](#interface)


<a id="setUseValues"></a>

## setUseValuesTyp(), setUseValuesMax()

#### Description
The particular method sets the internal flag whether typical or maximal values from the datasheet should be used regarding conversion time.

#### Syntax
    void setUseValuesTyp()
    void setUseValuesMax()

#### Parameters
None

#### Returns
None

[Back to interface](#interface)


<a id="configAlertMode"></a>

## configAlertActiveLow(), configAlertActiveHigh()

#### Description
The particular method updates alert activity bit state in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).

#### Syntax
    void configAlertActiveLow()
    void configAlertActiveHigh()

#### Parameters
None

#### Returns
None

#### See also
[getAlertActiveLow(), getAlertActiveHigh()](#getAlertMode)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getAlertMode"></a>

## getAlertActiveLow(), getAlertActiveHigh()

#### Description
The particular method determines flag about alert activity mode from the cached configuration value.

#### Syntax
    bool getAlertActiveLow()
    bool getAlertActiveHigh()

#### Parameters
None

#### Returns
Flag about set particular alert activity mode.

#### See also
[configAlertActiveLow(), configAlertActiveHigh()](#configAlertMode)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="getAlert"></a>

## getAlert()

#### Description
The method provides flag about state of alert pin from cached configuration value.
* It is suitable for detecting the alert by software without need of hardware sensing the ALERT pin of the sensor.

#### Syntax
    bool getAlert()

#### Parameters
None

#### Returns
Flag about ALERT pin state.

#### See also
[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configResolutionMode"></a>

## configExtendedMode(), configNormalMode()

#### Description
The particular method turns on corresponding resolution mode in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* At _normal_  mode the resolution is the 12-bit resolution.
* At _extended_  mode the resolution is the 13-bit resolution.
* Extended mode does not increase sensitivity, just extents the upper temperature measurement range from +128 to +150 centigrades. So that in normal working conditions it is not very useful.
* After changing resolution mode and writing it to the sensor it is needed to wait cca 350 milliseconds in order to settle the sensor and provide conversion. Otherwise the first conversion after changing resolution to extended mode from normal one doubles the measured temperature and after changing to normal mode from extended one halves the temperature, which might confuse follow-up logic or controlling mechanizm.
* The library does not have extra delay after resolution change implemented, so that it must be enforced in a sketch.

#### Syntax
    void configExtendedMode()
    void configNormalMode()

#### Parameters
None

#### Returns
None

#### See also
[getExtendedMode(), getNormalMode()](#getResolutionMode)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getResolutionMode"></a>

## getExtendedMode(), getNormalMode()

#### Description
The particular method determines flag about resolution mode state from the cached configuration value.

#### Syntax
    bool getExtendedMode()
    bool getNormalMode()

#### Parameters
None

#### Returns
Flag about set particular resolution mode.

#### See also
[configExtendedMode(), configNormalMode()](#configResolutionMode)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configPowerMode"></a>

## configShutdownMode(), configContinuousMode()

#### Description
The particular method turns on corresponding power mode in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* At _shutdown_  mode the sensor turns on all its system except the serial interface and reduces power consumption. This mode is utilized by the method [measurementTemperatureOneshot()](#measurementTemperatureOneshot) for one-shot temperature measurement.
* At _continuous_  mode the sensor performs continuous temperature conversion according to its [current conversion rate](#getConversionRate).

#### Syntax
    void configShutdownMode()
    void configContinuousMode()

#### Parameters
None

#### Returns
None

#### See also
[getShutdownMode(), getContinuousMode()](#getPowerMode)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getPowerMode"></a>

## getShutdownMode(), getContinuousMode()

#### Description
The particular method determines flag about power mode state from the cached configuration value.

#### Syntax
    bool getShutdownMode()
    bool getContinuousMode()

#### Parameters
None

#### Returns
Flag about set particular power mode.

#### See also
[configShutdownMode(), configContinuousMode()](#configPowerMode)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configActionMode"></a>

## configInterruptMode(), configThermostatMode()

#### Description
The particular method turns on corresponding action mode in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* At _interruption_  mode the sensor generates a short impulse on ALERT pin at reaching particular temperature limit with particular polarity according to the [alert activity mode](#configAlertActivityMode). The impulse is active until next reading a sensor's register, usually the temperature one.
* At _termostat_  mode the sensor changes state of ALERT pin at reaching a temperature limit with particular polarity according to the [alert activity mode](#configAlertActivityMode) and keeps it until reaching another temperature limit.

#### Syntax
    void configInterruptMode()
    void configThermostatMode()

#### Parameters
None

#### Returns
None

#### See also
[getInterruptMode(), getThermostatMode()](#getActionMode)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getActionMode"></a>

## getInterruptMode(), getThermostatMode()

#### Description
The particular method determines flag about action mode state from the cached configuration value.

#### Syntax
    bool getInterruptMode()
    bool getThermostatMode()

#### Parameters
None

#### Returns
Flag about set particular action mode.

#### See also
[configInterruptMode(), configThermostatMode()](#configActionMode)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configOneshotMode"></a>

## configOneshotMode()

#### Description
The method turns on one-shot temperature measurement mode in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* This mode is utilized by the method [measurementTemperatureOneshot()](#measurementTemperatureOneshot) for one-shot temperature measurement.

#### Syntax
    void configOneshotMode()

#### Parameters
None

#### Returns
None

#### See also
[getOneshotMode()](#getOneshotMode)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getOneshotMode"></a>
## getOneshotMode()
#### Description
The method provides current consecutive faults in form of value of pair of fault queue bits from the cached configuration value. That value can be compared to corresponding library class constants in order to determine number of consecutive faults.

#### Syntax
    bool getOneshotMode()

#### Parameters
None

#### Returns
Flag about set one-shot measurement mode.

#### See also
[configOneshotMode()](#configOneshotMode)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configConversionRate"></a>

## configConversionRate_025hz(), configConversionRate_1hz(), configConversionRate_4hz(), configConversionRate_8hz()

#### Description
The particular method sets conversion rate bits in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* The rate is determined by desired conversion frequency in Hertz denoted in the method's name.

#### Syntax
    void configConversionRate_025hz()
    void configConversionRate_1hz()
    void configConversionRate_4hz()
    void configConversionRate_8hz()

#### Parameters
None

#### Returns
None

#### See also
[getConversionRate()](#getConversionRate)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getConversionRate"></a>

## getConversionRate()

#### Description
The method provides current conversion rate as a frequency in Hertz based on conversion rate bits from the cached configuration value.

#### Syntax
    float getConversionRate()

#### Parameters
None

#### Returns
Conversion frequency in Hertz.

#### See also
[configConversionRate_025hz(), configConversionRate_1hz(), configConversionRate_4hz(), configConversionRate_8hz()](#configConversionRate)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configConversionPeriod"></a>

## configConversionPeriod_4000ms(), configConversionPeriod_1000ms(), configConversionPeriod_250ms(), configConversionPeriod_125ms()

#### Description
The particular method sets conversion rate bits in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* The rate is determined by desired conversion time period in milliseconds denoted in the method's name.
* The particular method set the corresponding conversion frequency as a reciprocal value.

#### Syntax
    void configConversionPeriod_4000ms()
    void configConversionPeriod_1000ms()
    void configConversionPeriod_250ms()
    void configConversionPeriod_125ms()

#### Parameters
None

#### Returns
None

#### See also
[getConversionPeriod()](#getConversionPeriod)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getConversionPeriod"></a>

## getConversionPeriod()

#### Description
The method provides current conversion rate as a time period in milliseconds based on conversion rate bits from the cached configuration value.

#### Syntax
    uint16_t getConversionPeriod()

#### Parameters
None

#### Returns
Conversion time period in milliseconds.

#### See also
[configConversionPeriod_4000ms(), configConversionPeriod_1000ms(), configConversionPeriod_250ms(), configConversionPeriod_125ms()](#configConversionPeriod)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="configFaults"></a>

## configFaults1(), configFaults2(), configFaults4(), configFaults6()

#### Description
The method sets fault queue bits in the cached configuration value before its sending to the sensor by the method [setConfiguration()](#setConfiguration).
* The number of faults is determined by a digit in the method's name.
* The number determines a number of consecutive faults (breaking a particular temperature limit) to activate particular alert. That number eliminate a environmental noise.

#### Syntax
    void configFaults1()
    void configFaults2()
    void configFaults4()
    void configFaults6()

#### Parameters
None

#### Returns
None

#### See also
[getFaults()](#getFaults)

[setConfiguration()](#setConfiguration)

[Back to interface](#interface)


<a id="getFaults"></a>

## getFaults()

#### Description
The method provides current number of consecutive faults based on fault queue bits from the cached configuration value.

#### Syntax
    uint8_t getFaults()

#### Parameters
None

#### Returns
Number of conscutive faults for a temperature alert activation.

#### See also
[configFaults1(), configFaults2(), configFaults4(), configFaults6()](#configFaults)

[getConfiguration()](#getConfiguration)

[Back to interface](#interface)


<a id="getErrorT"></a>

## getErrorT()

#### Description
The method returns virtually wrong temperature value at erroneous measurement usually at failure of two-wire bus.

#### Syntax
    float getErrorT()

#### Parameters
None

#### Returns
Erroneous temperature value.
The error code can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### See also
[measureTemperature()](#measureTemperature)

[measureTemperatureOneshot()](#measureTemperatureOneshot)

[Back to interface](#interface)
