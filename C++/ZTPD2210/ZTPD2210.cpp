#include <ZTPD2210.h>
#include <Wire.h>
#include "math.h"

ZTPD2210::ZTPD2210()
{

}

uint8_t ZTPD2210::writeRegister(uint8_t registerAddress, uint8_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(value);
    return _wire->endTransmission();
}

uint64_t ZTPD2210::readRegister(uint8_t registerAddress)
{
    uint8_t registerResponse[7] = {0};
    uint64_t value = 0;

    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, sizeof(registerResponse));
    for (uint8_t i = 0; i < sizeof(registerResponse); i++)
    {
        registerResponse[i] = _wire->read();
    }
    for (uint8_t i = 0; i < sizeof(registerResponse); i++)
    {
        value = (value << 8) | registerResponse[i];
    }

    return value;
}

void ZTPD2210::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = ZTPD2210_ADDRESS;
    _wire = wire;
}

float ZTPD2210::getObjectTemperature()
{
    _fullDataTemperature = readRegister(ZTPD2210_TEMPERATURE_REGISTER);
    uint32_t rawObjectTemperature = (_fullDataTemperature >> 24) & 0xFFFFFF;
    float objectTemperature = (float)rawObjectTemperature/pow(2,24) * 130.0 - 20.0;
    return objectTemperature;
}

float ZTPD2210::getAmbientTemperature()
{
    uint32_t rawAmbientTemperature = _fullDataTemperature & 0xFFFFFF;
    float ambientTemperature = (float)rawAmbientTemperature/pow(2,24) * 105.0 - 20.0;
    return ambientTemperature;  
}
