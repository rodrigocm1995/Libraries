#include <INA260.h>
#include "math.h"

INA260::INA260()
{

}


uint16_t INA260::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t INA260::readRegister(uint8_t registerAddress)
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


void INA260::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = INA260_1_SAMPLE | INA260_VBUS_1100US | INA260_SHUNT_1100US | INA260_ISHUNT_VBUS_CONTINUOUS;
    
    writeRegister(INA260_CONFIGURATION_REG, configValue);

    return configValue;
}


void INA260::init(uint8_t devAddress, Ina260Avg samples, Ina260BusVoltageCt vBusCt, Ina260BusIshuntCt iShuntCt, Ina260Mode mode, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = samples | vBusCt | iShuntCt | mode;

    writeRegister(INA260_CONFIGURATION_REG, configValue);

    return configValue;
}

uint16_t INA260::getConfig()
{
    uint16_t value = readRegister(INA260_CONFIGURATION_REG);
    return value;
}

bool INA260::isConnected()
{
  if ((_deviceAddress < 0x40) || (_deviceAddress > 0x4F)) return false;
  _wire->beginTransmission(_deviceAddress);
  return ( _wire->endTransmission() == 0);
}

bool INA260::begin()
{
    if (! isConnected()) return false;
    return true;
}

void INA260::reset()
{
    uint16_t resetValue = 0x8000;
    writeRegister(INA260_CONFIGURATION_REG, resetValue);
}

int16_t checkPolarity(int16_t value)
{
    bool isMsb;

    isMsb = CHECK_BIT(value,15);
    if (isMsb)
    {
        value = ~value + 0x0001
    }

    return (!isMsb ? value : -value);
}

float INA260::getCurrent()
{
    float current;
    int16_t rawValue = readRegister(INA260_CURRENT_REG);
    rawValue = checkPolarity(rawValue);

    current = rawTemperature * 1.25e-3;
    return current;
}

float INA260::getBusVoltage()
{
    uint16_t rawVoltage = readRegister(INA260_BUS_VOLTAGE_REG);
    float voltage = rawVoltage * 1.25e-3;

    return voltage;
}

float INA260::getPower()
{
    uint16_t rawPower = readRegister(INA260_POWER_REG);
    float power = rawPower * 10e-3;

    return power;
}

bool isReady()
{
    bool isConversionReady;
    uint16_t maskRegister = readRegister(INA260_MASK_ENABLE_REG);

    isConversionReady = CHECK_BIT(maskRegister, 3);

    return isConversionReady;
}
