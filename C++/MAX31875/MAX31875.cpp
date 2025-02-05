#include <MAX31875.h>
#include <Wire.h>
#include "math.h"

MAX31875::MAX31875()
{

}

uint8_t MAX31875::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t MAX31875::readRegister(uint8_t registerAddress)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, (uint8_t)2);
    uint16_t value = _wire->read();
    value <<= 8;    
    value |= _wire->read();
    return value;
}

uint8_t MAX31875::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = MAX31875_ADDRESS;
    _wire = wire;

    uint16_t configValue = MAX31875_ONESHOT_ENABLE | MAX31875_0_DOT_25_SAMPLES_PER_SECOND | MAX31875_PEC_DISABLED | MAX31875_TIMEOUT_ENABLED | MAX31875_10_BITS_RESOLUTION | MAX31875_NORMAL_TEMPERATURE_FORMAT | MAX31875_CONTINUOUS_CONVERSION_ENABLED | MAX31875_COMPARATOR_MODE | MAX31875_1_FAULT;
    _resolution = MAX31875_10_BITS_RESOLUTION;

    return ((writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint8_t MAX31875::customInit(uint8_t devAddress, MAX31875OneShot_t oneShot, MAX31875ConversionRate_t convRate, MAX31875Pec_t pec, MAX31875Timeout_t timeout, MAX31875Resolution_t resolution, MAX31875DataFormat_t dataFormat, MAX31875ShuntDown_t shutdownMode, MAX31875CompInit_t compInit, MAX31875FaultQueue_t faults, TwoWire *wire)
{
    _deviceAddress = MAX31875_ADDRESS;
    _wire = wire;
    _resolution = resolution;

    uint16_t configValue = oneShot | convRate | pec | timeout | resolution | dataFormat | shutdownMode | compInit | faults;
    return ((writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}

int16_t MAX31875::getConfig()
{
    int16_t value = readRegister(MAX31875_CONFIGURATION_REGISTER);
    return value;
}

void MAX31875::setLowLimit(int16_t lowLimit)
{
    int16_t value = setTemperatureLimit(lowLimit);
    writeRegister(MAX31875_TEMPERATURE_HYST_REGISTER);
}

void MAX31875::setHighLimit(int16_t highLimit)
{
    int16_t value = setTemperatureLimit(highLimit);
    writeRegister(MAX31875_TEMPERATURE_OS_REGISTER);
}

int16_t MAX31875::setTemperatureLimit(int16_t tempLimit)
{
    uint16_t regValue = abs(lowLimitTemperature) / 7.8125e-3;
    (lowLimitTemperature < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
    return value;
}

float MAX31875::getTemperature()
{
    int16_t rawTemperature = readRegister(MAX31875_TEMPERATURE_REGISTER);
    float temperature = checkTemp(rawTemperature);

    return temperature;
}

bool MAX31875::getOverTemperatureBit()
{
    bool isDataReady;
    uint16_t otBit = readRegister(MAX31875_CONFIGURATION_REGISTER);
    isDataReady = CHECK_BIT(otBit, 15);

    return isDataReady;
}

float MAX31875::checkTemp(int16_t value)
{
    float temperature;
    bool isMsb;

    isMsb = CHECK_BIT(value, 15);
    if (isMsb)
    {
        value = ~value + 0x0001;
    }

    if (_resolution == MAX31875_8_BITS_RESOLUTION)
    {
        temperature = value * MAX31875_8_BITS_RES;
    }
    else if (_resolution == MAX31875_9_BITS_RESOLUTION)
    {
       temperature = value * MAX31875_9_BITS_RES; 
    }
    else if (_resolution == MAX31875_10_BITS_RESOLUTION)
    {
        temperature = value * MAX31875_10_BITS_RES; 
    }
    else
    {
       temperature = value * MAX31875_12_BITS_RES; 
    }

    return temperature;
}