#include <HDC1080.h>
#include <Wire.h>
#include "math.h"

HDC1080::HDC1080()
{

}

uint8_t HDC1080::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t HDC1080::readRegister(uint8_t registerAddress)
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

uint8_t HDC1080::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = HDC1080_ADDRESS;
    _wire = wire;

    uint16_t configValue = HDC1080_HEATER_DISABLED | HDC1080_TEMP_OR_HUMIDITY | HDC1080_BAT_VOLTAGE_G2_8 | HDC1080_TEMP_14BIT_RESOLUTION | HDC1080_HUMIDITY_14BIT_RESOLUTION;
    return ((writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint8_t HDC1080::customInit(uint8_t devAddress, HDC1080HeaterMode_t heater, HDC1080BAcquisitionMode_t mode, HDC1080BatteryStatus_t batStatus, HDC1080TempResolution_t tempRes, HDC1080HumidityResolution_t humRes, TwoWire *wire)
{
    _deviceAddress = HDC1080_ADDRESS;
    _wire = wire;

    uint16_t configValue = heater | mode | batStatus | tempRes | humRes;
    return ((writeRegister(HDC1080_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint16_t HDC1080::getConfig()
{
    uint16_t value = readRegister(HDC1080_CONFIGURATION_REGISTER);
    return value;
}

uint16_t HDC1080::getTemperature()
{
    uint16_t rawTemp = readRegister(HDC1080_TEMPERATURE_REGISTER);
    float temperature = (double)(rawTemp / pow(2,16)) * 165.0 - 140.0
    return temperature;
}

uint16_t HDC1080::getHumidity()
{
    uint16_t rawHumidity = readRegister(HDC1080_HUMIDITY_REGISTER);
    float humidity = (double)(rawHumidity/pow(2, 16)) * 100.0;

    return humidity;
}
