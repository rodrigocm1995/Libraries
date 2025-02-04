#include <TPL1401.h>
#include <Wire.h>
#include "math.h"

TPL1401::TPL1401()
{

}

uint8_t TPL1401::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t TPL1401::readRegister(uint8_t registerAddress)
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

uint8_t TPL1401::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = TPL1401_ADDRESS;
    _wire = wire;

    uint16_t configValue = !TPL1401_DEVICE_LOCK_BIT | TPL1401_RESERVED_BITS | POWER_DOWN_HIGH_IMPEDANCE | !TPL1401_REF_ENABLED;
    return ((writeRegister(TPL1401_GENERAL_CONFIG_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint8_t TPL1401::customInit(uint8_t devAddress, uint16_t deviceLock, Tpl1401DpotPdn_t powerType, uint16_t internalRef, Tpl1401OutSpan_t span, TwoWire *wire)
{
    _deviceAddress = TPL1401_ADDRESS;
    _wire = wire;

    uint16_t configValue = deviceLock | TPL1401_RESERVED_BITS | internalRef | span;
    return ((writeRegister(TPL1401_GENERAL_CONFIG_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint16_t TPL1401::getConfig()
{
    uint16_t value = readRegister(TPL1401_GENERAL_CONFIG_REGISTER);
    return value;
}