#include <TMP117.h>
#include <Wire.h>
#include "math.h"

TMP117::TMP117()
{

}

int16_t TMP117::writeRegister(uint8_t registerAddress, int16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

int16_t TMP117::readRegister(uint8_t registerAddress)
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

bool TMP117::begin(TwoWire *wire)
{
    _wire = wire;
    _deviceAddress = TMP117_ADDRESS1;
    uint8_t wireResponse = 0;
    for (uint8_t devReg = 0x48; devReg < (TMP117_ADDRESS4 + 1); devReg++)
    {
        _wire->beginTransmission(devReg);
        wireResponse = _wire->endTransmission();
        if (wireResponse == 0) 
        {
            _deviceAddress = devReg;
            break;
        }
        delay(10);
    }
    return ((wireResponse == 0) ? 1 : 0);
}
// If the writing process on the device was correct then returns 1, otherwise 0
uint8_t TMP117::init(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = MOD_CONTINUOUS_CONVERSION | CONVERSION_TYPE_4 |  AVERAGED_CONVERSIONS_8 | ALERT_MODE;
    return ((writeRegister(TMP117_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint8_t TMP117::customInit(uint8_t devAddress, tmp117ConversionMode_t conversionMode, tmp117ConversionCycleTime_t conversionCycle, tmp117AveragingMode_t averagingMode, tmp117ThermAlertMode_t mode, tmp117Polarity_t polarity, tmp117Alert_t alert, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = conversionMode | conversionCycle | averagingMode | mode | polarity | alert;

    return ((writeRegister(TMP117_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0);
}

void TMP117::setHighLimit(int8_t highLimitTemperature)
{
    uint16_t regValue = abs(highLimitTemperature) / 7.8125e-3;
    (highLimitTemperature < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
    writeRegister(TMP117_THIGH_LIMIT_REGISTER, regValue);
}

void TMP117::setLowLimit(int8_t lowLimitTemperature)
{
    uint16_t regValue = abs(lowLimitTemperature) / 7.8125e-3;
    (lowLimitTemperature < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
    writeRegister(TMP117_TLOW_LIMIT_REGISTER, regValue);
}

float TMP117::getHighLimit()
{
    float temperatureValue;

    int16_t result = readRegister(TMP117_THIGH_LIMIT_REGISTER);
    temperatureValue = checkAndGetTemperature(result);
    return temperatureValue;
}

float TMP117::getLowLimit()
{
    float temperatureValue;

    int16_t result = readRegister(TMP117_TLOW_LIMIT_REGISTER);
    temperatureValue = checkAndGetTemperature(result);
    return temperatureValue;   
}

uint16_t TMP117::alertAndDataReady(bool *arr)
{
    bool isHighAlert;
    bool isLowAlert;
    bool isDataReady;
    bool flagsArr[3];

    int16_t config = readRegister(TMP117_CONFIGURATION_REGISTER);
    isHighAlert = CHECK_BIT(config, 15);
    isLowAlert  = CHECK_BIT(config, 14);
    isDataReady = CHECK_BIT(config, 13);

    arr[0] = isHighAlert;
    arr[1] = isLowAlert;
    arr[2] = isDataReady;
}

float TMP117::getTemperature()
{
    int16_t rawTemperature = readRegister(TMP117_TEMP_RESULT_REGISTER);
    float temperature = checkAndGetTemperature(rawTemperature);
    return temperature; 
}

float TMP117::checkAndGetTemperature(int16_t value)
{
    float temperature;
	_Bool isMsb;

	isMsb = CHECK_BIT(value, 15);

	if (isMsb)
	{
		value = ~value + 0x0001;
	}

	temperature = value * 7.8125e-3;

	return (!isMsb ? temperature : -temperature);
}