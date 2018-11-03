<a id="library"></a>
# gbjTMP102
Library for the temperature sensors *TMP102* communicating on two-wire (I2C) bus.
- Sensor address is
  - `0x48` for ADD0 pin connected to GND (ground)
  - `0x49` for ADD0 pin connected to V+ (power supply positive rail)
  - `0x4A` for ADD0 pin connected to SDA (serial data rail of the two-wire bus)
  - `0x4B` for ADD0 pin connected to SCL (serial clock rail of the two-wire bus)
- The library provides measured temperature in degrees of Celsius.
- For conversion among various temperature unit scales and for calculating dew point temperature use library **gbjAppHelpers**.
- The resolution of the sensor in normal mod is 12 bit with sensitivity 0.0625 centigrade with measurement range -55 to +128 centigrade.
- The extended mode has resolution 13 bit, but the same sensitivity and it just extends the upper measurement range up to +150 centigrade.
  - Switching (reconfiguration) to extended mode from normal mode or vice-versa needs a time delay cca 500 milliseconds after it in order to settle the sensor and stabilize the temperature conversion.
  - Without the delay after switching to extended mode the reading is doubled to real temperature at first reading after switching mode.
  - Without the delay after switching to normal mode the reading is halved to real temperature at first reading after switching mode.
  - Library does not have implemented such specific delay after mode switching due to small usefulness of the extended mode.


#### Particle hardware configuration
- Connect microcontroller's pin `D0` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).

#### Arduino UNO hardware configuration
- Connect microcontroller's pin `A4` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `A5` to sensor's pin **SCL** (Serial Clock).

#### Espressif - ESP8266, ESP32 default hardware configuration
- Connect microcontroller's pin `D2` to sensor's pin **SDA** (Serial Data).
- Connect microcontroller's pin `D1` to sensor's pin **SCL** (Serial Clock).


<a id="dependency"></a>
## Dependency

#### Particle platform
- **Particle.h**: Includes alternative (C++) data type definitions.

#### Arduino platform
- **Arduino.h**: Main include file for the Arduino SDK version greater or equal to 100.
- **WProgram.h**: Main include file for the Arduino SDK version less than 100.
- **inttypes.h**: Integer type conversions. This header file includes the exact-width integer definitions and extends them with additional facilities provided by the implementation.
- **TwoWire**: I2C system library loaded from the file *Wire.h*.

#### Custom Libraries
- **gbjTwoWire**: I2C custom library loaded from the file *gbj_twowire.h*. The library [gbjSI70](#library) inherits common bus functionality from this library.


<a id="constants"></a>
## Constants
- **gbj\_tmp102::VERSION**: Name and semantic version of the library.


<a id="addresses"></a>
#### Measurement resolutions
- **gbj\_tmp102::ADDRESS\_GND**: ADD0 pin connected to GND pin (default).
- **gbj\_tmp102::ADDRESS\_VCC**: ADD0 pin connected to positive power supply rail.
- **gbj\_tmp102::ADDRESS\_SDA**: ADD0 pin connected to serial data rail of two-wire bus.
- **gbj\_tmp102::ADDRESS\_SCL**: ADD0 pin connected to serial clock rail of two-wire bus.


<a id="conversions"></a>
#### Conversion rates
- **gbj\_tmp102::CONVERSION\_PERIOD\_4000MS**: Conversion period in milliseconds for conversion frequency 0.25 Hz.
- **gbj\_tmp102::CONVERSION\_PERIOD\_1000MS**: Conversion period in milliseconds for conversion frequency 1 Hz.
- **gbj\_tmp102::CONVERSION\_PERIOD\_250MS**: Conversion period in milliseconds for conversion frequency 4 Hz (default).
- **gbj\_tmp102::CONVERSION\_PERIOD\_125MS**: Conversion period in milliseconds for conversion frequency 8 Hz.
- **gbj\_tmp102::CONVERSION\_RATE\_025HZ**: Conversion frequency 0.25 Hz.
- **gbj\_tmp102::CONVERSION\_RATE\_1HZ**: Conversion frequency 1 Hz.
- **gbj\_tmp102::CONVERSION\_RATE\_4HZ**: Conversion frequency 4 Hz (default).
- **gbj\_tmp102::CONVERSION\_RATE\_8HZ**: Conversion frequency 8 Hz.


<a id="faults"></a>
#### Fault queues
- **gbj\_tmp102::FAULT\_QUEUE\_1**: 1 consecutive fault for changing ALERT pin state (default).
- **gbj\_tmp102::FAULT\_QUEUE\_2**: 2 consecutive faults for changing ALERT pin state.
- **gbj\_tmp102::FAULT\_QUEUE\_4**: 4 consecutive faults for changing ALERT pin state.
- **gbj\_tmp102::FAULT\_QUEUE\_6**: 8 consecutive faults for changing ALERT pin state.


<a id="errors"></a>
#### Error codes
- **gbj\_tmp102::ERROR\_RESET**: Resetting failure.
- **gbj\_tmp102::ERROR\_MEASURE\_TEMP**: Measuring temperature failure.
- **gbj\_tmp102::ERROR\_SETUP\_TEMP**: Temperature limits failure.

Other error codes as well as result code are inherited from the parent library [gbjTwoWire](#dependency).


<a id="interface"></a>
## Interface

#### Main
- [gbj_tmp102()](#gbj_tmp102)
- [begin()](#begin)
- [reset()](#reset)
- [measureTemperature()](#measureTemperature)
- [measureTemperatureOneshot()](#measureTemperatureOneshot)

#### Setters
- [setConfiguration()](#setConfiguration)
- [setAlertLow()](#setAlertValue)
- [setAlertHigh()](#setAlertValue)
- [configAlertActiveLow()](#configAlertMode)
- [configAlertActiveHigh()](#configAlertMode)
- [configExtendedMode()](#configResolutionMode)
- [configNormalMode()](#configResolutionMode)
- [configShutdownMode()](#configPowerMode)
- [configContinuousMode()](#configPowerMode)
- [configInterruptMode()](#configActionMode)
- [configThermostatMode()](#configActionMode)
- [configOneshotMode()](#configOneshotMode)

#### Getters
- [getExtendedMode()](#getResolutionMode)
- [getNormalMode()](#getResolutionMode)
- [getShutdownMode()](#getPowerMode)
- [getContinuousMode()](#getPowerMode)
- [getInterruptMode()](#getActionMode)
- [getThermostatMode()](#getActionMode)
- [getConversionRate()](#getConversionRate)
- [getFaultQueue()](#getFaultQueue)
- [getOneshotMode()](#getOneshotMode)
- [getAlert()](#getAlert)
- [getAlertActiveLow()](#getAlertMode)
- [getAlertActiveHigh()](#getAlertMode)
- [getAlertLow()](#getAlertValue)
- [getAlertHigh()](#getAlertValue)

Other possible setters and getters are inherited from the parent library [gbjTwoWire](#dependency) and described there.


<a id="gbj_tmp102"></a>
## gbj_tmp102()
#### Description
The library does not need special constructor and destructor, so that the inherited ones are good enough and there is no need to define them in the library, just use it with default or specific parameters as defined at constructor of parent library [gbjTwoWire](#dependency).
- Constructor sets parameters specific to the two-wire bus in general.
- All the constructor parameters can be changed dynamically with corresponding setters later in a sketch.

#### Syntax
    gbj_tmp102(uint32_t clockSpeed, bool busStop, uint8_t pinSDA, uint8_t pinSCL);

#### Parameters
<a id="prm_busClock"></a>
- **clockSpeed**: Two-wire bus clock frequency in Hertz. If the clock is not from enumeration, it fallbacks to 100 kHz.
  - *Valid values*: gbj\_tmp102::CLOCK\_100KHZ, gbj\_tmp102::CLOCK\_400KHZ
  - *Default value*: gbj\_tmp102::CLOCK\_100KHZ


<a id="prm_busStop"></a>
- **busStop**: Logical flag about releasing bus after end of transmission.
  - *Valid values*: true, false
    - **true**: Releases the bus after data transmission and enables other master devices to control the bus.
    - **false**: Keeps connection to the bus and enables to begin further data transmission immediately.
  - *Default value*: true


<a id="prm_pinSDA"></a>
- **pinSDA**: Microcontroller's pin for serial data. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms for communication on the bus. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  - *Valid values*: positive integer
  - *Default value*: 4 (GPIO4, D2)


<a id="prm_pinSCL"></a>
- **pinSCL**: Microcontroller's pin for serial clock. It is not a board pin but GPIO number. For hardware two-wire bus platforms it is irrelevant and none of methods utilizes this parameter for such as platforms. On the other hand, for those platforms the parameters might be utilized for storing some specific attribute in the class instance object.
  - *Valid values*: positive integer
  - *Default value*: 5 (GPIO5, D1)

#### Returns
Object performing the sensor management.
The constructor cannot return [a result or error code](#constants) directly, however, it stores them in the instance object. The result can be tested in the operational code with the method [getLastResult()](#getLastResult), [isError()](#isError), or [isSuccess()](#isSuccess).

#### Example
The method has all arguments defaulted and calling without any parameters is equivalent to the calling with all arguments set by corresponding constant with default value:

```cpp
  gbj_tmp102 Sensor = gbj_tmp102(); // It is equivalent to
  gbj_tmp102 Sensor = gbj_tmp102(gbj_tmp102::CLOCK_100KHZ, true, D2, D1);
```

[Back to interface](#interface)


<a id="begin"></a>
## begin()
#### Description
The method takes, sanitizes, and stores sensor parameters to a class instance object and initiates two-wire bus.
- The method sets parameters specific to the sensor itself.
- The method resets the sensor to its power-up state (see details in method [reset](#reset)).
- All the method parameters can be changed dynamically with corresponding [setters](#interface) later in a sketch.

#### Syntax
    uint8_t begin(uint8_t address);

#### Parameters
<a id="prm_address"></a>
- **address**: One of four possible 7 bit addresses of the sensor.
  - *Valid values*: [gbj\_tmp102::ADDRESS\_GND](#addresses), [gbj\_tmp102::ADDRESS\_VCC](#addresses), [gbj\_tmp102::ADDRESS\_SDA](#addresses), [gbj\_tmp102::ADDRESS\_SCL](#addresses).
  - *Default value*: [gbj\_tmp102::ADDRESS\_GND](#addresses)
    - The default values is set to address corresponding to grounded ADD0 pin.
    - If input value is none of expected ones, the method fallbacks it to default address.
    - Implementing addressing allows up to 4 sensors present on the same two-wire bus.
    - Module boards with the sensor have usually ADD0 grounded and are equipped with soldering pad for reconnecting that pin to V+ rail.

#### Returns
Some of [result or error codes](#constants).

#### See also
[reset()](#reset)

[Back to interface](#interface)


<a id="reset"></a>
## reset()
#### Description
The method resets the sensor by the general call software reset sending the code 0x06 to the two-wire bus at address 0x00 and reads the content of the configuration register to the library instance object. Software reset causes resetting all internal registers to their power-up values, which determine following configuration and values:
- Upper alert temperature limit 80 centigrade.
- Lower alert temperature limit 75 centigrade.
- Normal mode (12 bit).
- Alert active.
- Conversion rate 4 Hz.
- Continuous power mode (shutdown mode off).
- Thermostat (comparator) mode.
- Alert pin active low.
- One fault for fault queue.
- One-shot conversion off.

#### Syntax
    uint8_t reset();

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

#### Syntax
    float measureTemperature();

#### Parameters
None

#### Returns
Temperature in centigrade or the error value [gbj\_tmp102::ERROR\_MEASURE\_TEMP](#errors) with corresponding error code in the library object.

#### See also
[measureTemperatureOneshot()](#measureTemperatureOneshot)

[Back to interface](#interface)


<a id="measureTemperatureOneshot"></a>
## measureTemperatureOneshot()
#### Description
The method configures shutdown mode and one-shot conversion of the sensor. It waits until conversion finishes and returns ambient temperature in centigrade.
- The method is useful at very long periods (couple of minutes and hours) between measurements in order to save power consumption.

#### Syntax
    float measureTemperatureOneshot();

#### Parameters
None

#### Returns
Temperature in centigrade or the error value [gbj\_tmp102::ERROR\_MEASURE\_TEMP](#errors) with corresponding error code in the library object.

#### See also
[measureTemperature()](#measureTemperature)

[Back to interface](#interface)


<a id="setResolution"></a>
## setResolution()
#### Description
The method sets the bit resolution by input parameter, which should be appropriate library [constant](#resolution).
The resolution is determined by that constant but in fact it is the bit resolution for temperature.

#### Syntax
    uint8_t setResolution(uint8_t resolution = gbj_tmp102::RESOLUTION_T14_RH12);

#### Parameters
<a id="resolution"></a>
- **resolution**: Desired measurement resolution in bits.
  - *Valid values*:  [gbj\_tmp102::RESOLUTION\_T14\_RH12](#resolution), [gbj\_tmp102::RESOLUTION\_T13\_RH10](#resolution),  [gbj\_tmp102::RESOLUTION\_T12\_RH8](#resolution), or [gbj\_tmp102::RESOLUTION\_T11\_RH11](#resolution)
  - *Default value*: [gbj\_tmp102::RESOLUTION\_T14\_RH12](#resolution)

#### Returns
Some of [result or error codes](#constants).

#### See also
[setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()](#setResolutionTemp)

[setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()](#setResolutionRhum)

[Back to interface](#interface)


<a id="setResolutionTemp"></a>
## setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()
#### Description
The particular method sets the bit resolution for temperature measurement to the value in its name.
The method sets the corresponding bit resolution for the relative humidity measurement at the same time by this relation:

Temperature | Relative Humidity
------ | -------
11 | 11
12 | 8
13 | 10
14 | 12

#### Syntax
    uint8_t setResolutionTemp11();
    uint8_t setResolutionTemp12();
    uint8_t setResolutionTemp13();
    uint8_t setResolutionTemp14();

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[setResolution()](#setResolution)

[getResolutionTemp()](#getResolutionTemp)

[Back to interface](#interface)


<a id="getResolutionTemp"></a>
## getResolutionTemp()
#### Description
The method returns the temperature measurement resolution in bits.

#### Syntax
    uint8_t getResolutionTemp();

#### Parameters
None

#### Returns
Bit resolution (11, 12, 13, or 14) or some of [error codes](#errors).

#### See also
[setResolution()](#setResolution)

[setResolutionTemp11(), setResolutionTemp12(), setResolutionTemp13(), setResolutionTemp14()](#setResolutionTemp)

[Back to interface](#interface)


<a id="setResolutionRhum"></a>
## setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()
#### Description
The particular method sets the bit resolution for relative humidity measurement to the value in its name.
The method sets the corresponding bit resolution for the temperature measurement at the same time by this relation:

Relative Humidity | Temperature
------ | -------
11 | 11
8 | 12
10 | 13
12 | 14

#### Syntax
    uint8_t setResolutionRhum8();
    uint8_t setResolutionRhum10();
    uint8_t setResolutionRhum11();
    uint8_t setResolutionRhum12();

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

#### See also
[setResolution()](#setResolution)

[Back to interface](#interface)


<a id="getResolutionRhum"></a>
## getResolutionRhum()
#### Description
The method returns the relative humidity measurement resolution in bits.

#### Syntax
    uint8_t getResolutionRhum();

#### Parameters
None

#### Returns
Bit resolution (8, 10, 11, or 12) or some of [error codes](#errors).

#### See also
[setResolution()](#setResolution)

[setResolutionRhum8(), setResolutionRhum10(), setResolutionRhum11(), setResolutionRhum12()](#setResolutionRhum)

[Back to interface](#interface)


<a id="setHeater"></a>
## setHeaterEnabled(), setHeaterDisabled()
#### Description
The particular method turns on or off a heater built-in in the sensor.

#### Syntax
    uint8_t setHeaterEnabled();
    uint8_t setHeaterDisabled();

#### Parameters
None

#### Returns
Some of [result or error codes](#constants).

[Back to interface](#interface)


<a id="getHeaterEnabled"></a>
## getHeaterEnabled()
#### Description
The method returns the status of the sensor's heater.

#### Syntax
    bool getHeaterEnabled();

#### Parameters
None

#### Returns
Flag about the heater switched on or off.
- **true**: The heater is on.
- **false**: The heater is off.

#### See also
[setHeaterEnabled()](#setHeater)

[setHeaterDisabled()](#setHeater)

[Back to interface](#interface)


<a id="getSerial"></a>
## getSNA(), getSNB(), getSNC(), getSerialNumber()
#### Description
The particular method returns the corresponding 16-bit or 32-bit part of the serial number and the entire 64-bit serial number of the sensor.

#### Syntax
    uint16_t getSNA();
    uint32_t getSNB();
    uint16_t getSNC();
    uint64_t getSerialNumber();

#### Parameters
None

#### Returns
Particular part of, or entire serial number, or some of [error codes](#errors).

[Back to interface](#interface)


<a id="getVddStatus"></a>
## getVddStatus()
#### Description
The method returns the status of the supply voltage, which the sensor is powered by.

#### Syntax
    bool getVddStatus();

#### Parameters
None

#### Returns
Flag about the correctness of the operating voltage.
- **true**: The voltage is correct.
- **false**: The voltage is incorrect.

[Back to interface](#interface)
