#include <INA219.h>
#include "math.h"

INA219::INA219()
{

}


uint16_t INA219::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t INA219::readRegister(uint8_t registerAddress)
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


uint16_t INA219::defaultInit(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = INA219_BUSVOLTAGERANGE_32V | INA219_PGAGAIN_320_MILI_VOLT | INA219_BUS_ADC_12_BIT_RESOLUTION | INA219_SHUNT_ADC_12_BIT_RESOLUTION | INA219_SHUNTBUS_CONTINUOUS_MODE;
    
    writeRegister(INA219_CONFIGURATION_REG, configValue);

    return configValue;
}


uint16_t INA219::init(uint8_t devAddress, BusVoltageRange_t busvoltageRange, ShuntPGAGain_t pgaGain, BusADCResolution_t busAdcResolution, ShuntADCResolution_t shuntAdcResolution, Mode_t mode, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = busvoltageRange | pgaGain | busAdcResolution | shuntAdcResolution | mode;

    writeRegister(INA219_CONFIGURATION_REG, configValue);

    return configValue;
}

uint16_t INA219::getConfigRegister()
{
    uint16_t value = readRegister(INA219_CONFIGURATION_REG);
    return value;
}

bool INA219::isConnected()
{
  if ((_deviceAddress < 0x40) || (_deviceAddress > 0x4F)) return false;
  _wire->beginTransmission(_deviceAddress);
  return ( _wire->endTransmission() == 0);
}

bool INA219::begin()
{
    if (! isConnected()) return false;
    return true;
}


uint16_t INA219::setCalibration(float rShuntValue, float maxCurrent)
{
    _rShunt = rShuntValue;
    _maximumCurrent = maxCurrent;

    float currentLsb = maxCurrent/pow(2,15);
    _currentLSB = currentLsb;

    uint32_t cal = trunc(0.04096 /(currentLsb * rShuntValue));
    if (cal > pow(2,15))
    {
        return 0;
    }
    else
    {
        uint16_t call = cal;
        writeRegister(INA219_CALIBRATION_REG, call);
        return call;
    }
}

uint16_t INA219::getCalibrationRegister()
{
    uint16_t value = readRegister(INA219_CALIBRATION_REG);
    return value;
}

float INA219::readShuntVoltage()
{
    uint16_t result = readRegister(INA219_SHUNTVOLTAGE_REG);
    float value = (float)result * 0.01;
    return value;
}

float INA219::readBusVoltage()
{
    uint16_t result = (readRegister(INA219_BUSVOLTAGE_REG)) >> 3;
    float value = (float)result * 0.004;
    return value;
}

float INA219::readCurrent()
{
    uint16_t currentRegister = readRegister(INA219_CURRENT_REG);
    float value = currentRegister * _currentLSB * 1000.0;
    
    return value;
}

float INA219::readPower()
{
    float powerRegister = readRegister(INA219_POWER_REG);
    float powerLsb = 20.0 * _currentLSB;
    float power = powerRegister * powerLsb;

    return power;
}

bool INA219::dataReady()
{
    bool isDataReady;

    uint16_t conversionReady = readRegister(INA219_BUSVOLTAGE_REG);
    isDataReady = CHECK_BIT(conversionReady, 1);

    return isDataReady;
}

uint8_t INA219::correctedFullScaleCalibration(uint16_t calibrationValue, float inaCurrent, float measuredShuntCurrent)
{
    uint16_t calibrationValue = 
    uint16_t correctedFullScaleCalibration = trunc( (calibrationValue * measuredShuntCurrent)/inaCurrent );
    writeRegister(INA219_CALIBRATION_REG, correctedFullScaleCalibration);
}
