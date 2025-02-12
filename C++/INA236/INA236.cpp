#include <INA236.h>
#include "math.h"

INA236::INA236()
{

}


uint16_t INA236::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t INA236::readRegister(uint8_t registerAddress)
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


uint8_t INA236::defaultInit(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;
    _shuntAdcRange = 2.5e-6;

    uint16_t configValue = INA236_CONFIG_RESERVED | INA236_ADCRANGE_81DOT92_MILIVOLT | INA236_1_SAMPLE | INA236_VBUSCT_1100_US | INA236_VSHCT_1100_US | INA236_CONTINUOUS_SHUNTBUS_VOLTAGE;
    
    return ( (writeRegister(INA236_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint8_t INA236::init(uint8_t devAddress, Ina236AdcRange_t adcRange, Ina236Avg_t samples, Ina236VbusConvertionTime_t busConvertionTime, Ina236ShuntConvertionTime_t shuntConvertionTime, Ina236Mode_t mode, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    if (adcRange == INA236_ADCRANGE_81DOT92_MILIVOLT)
    {
	    _shuntAdcRange = 2.5 * pow(10, -6);
	    _rangeAdc = 0;
    }
    else if (adcRange == INA236_ADCRANGE_20DOT48_MILIVOLT)
    {
	    _shuntAdcRange = 625 * pow(10, -9);
	    _rangeAdc = 1;
    }

    uint16_t configValue = INA236_CONFIG_RESERVED | adcRange | samples | busConvertionTime | shuntConvertionTime | mode;

    return ( (writeRegister(INA236_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint16_t INA236::setCalibration(float rShuntValue, float maxCurrent)
{
    _rShunt = rShuntValue;
    _maximumCurrent = maxCurrent;

    float currentLsbMinimum;
    uint16_t shuntCal;

    currentLsbMinimum = maxCurrent / pow(2, 15);
    if (maxCurrent < 3) currentLsbMinimum = 100.0 * pow(10, -6);
    else if (maxCurrent > 3 && maxCurrent <= 6) currentLsbMinimum  = 200.0 * pow(10, -6);
    else if (maxCurrent > 6 && maxCurrent <= 9) currentLsbMinimum  = 300.0 * pow(10, -6);
    else if (maxCurrent > 9 && maxCurrent <= 13) currentLsbMinimum = 500.0 * pow(10, -6);
    else if (maxCurrent > 13 && maxCurrent <= 16) currentLsbMinimum = 500.0 * pow(10, -6);
    else if (maxCurrent > 16 && maxCurrent <= 19) currentLsbMinimum = 600.0 * pow(10, -6);

    _currentLsbMin = currentLsbMinimum;
    shuntCal = 0.00512 / (currentLsbMinimum * rShuntValue);

    if (_rangeAdc == 1)
    {
        shuntCal = shuntCal / 4;
    }

    writeRegister(INA236_CALIBRATION_REGISTER, shuntCal);
}

void INA236::setMaskRegister(Ina236AlertType_t alertType)
{
    _alertType = alertType;
    uint16_t value;

    switch(alertType)
    {
        case 1:
            value = INA236_SHUNT_OVER_LIMIT;
            break;
        case 2:
            value = INA236_SHUNT_UNDER_LIMIT;
            break;
        case 3:
            value = INA236_BUS_OVER_LIMIT;
            break;
        case 4: 
            value = INA236_BUS_UNDER_LIMIT;
            break;
        case 5:
            value = INA236_POWER_OVER_LIMIT;
            break;
        case 6:
            value = INA236_CONVERSION_READY;
            break;
        default: return 0;
  }
  value = value | 0x0000;
  writeRegister(INA236_MASK_ENABLE_REGISTER, value);
  return value;
}

void INA236::resetMaskRegister()
{
    writeRegister(INA236_MASK_ENABLE_REGISTER, 0x0000);
}

void INA236::setCurrentAlertLimit(float currentLimit)
{
    float shuntVoltageLimit;
    uint16_t alertValue;

    shuntVoltageLimit = currentLimit * _rShunt;
    alertValue = shuntVoltageLimit / _shuntAdcRange;
    writeRegister(INA236_ALERT_LIMIT_REGISTER, alertValue);
}

float INA236::getShuntVoltage()
{
    float result = readRegister(INA236_SHUNT_VOLTAGE_REGISTER);
    result = result *  _shuntAdcRange * 1000.0;

    return result;
}

float INA236::getBusVoltage()
{
	float result = readRegister(INA236_BUS_VOLTAGE_REGISTER);
	result = result * (1.6) * 1e-3;

	return result;
}

float INA236::getCurrent()
{
  float rawCurrent = readRegister(INA236_CURRENT_REGISTER);
  float current = rawCurrent * (_currentLsbMin) * (1000.0);
  return current;
}

float INA236::getPower()
{
  float rawPower = readRegister(INA236_POWER_REGISTER);
  float power = 32.0 * rawPower * (_currentLsbMin) * (1000.0);
  return power;   
}

bool INA236::alertFunctionFlag()
{
	uint16_t result = readRegister(INA236_MASK_ENABLE_REGISTER);
	bool isAlertFunctionFlagTrue = CHECK_BIT(result, 4);
	return isAlertFunctionFlagTrue;   
}

bool INA236::dataReady()
{
    bool isDataReady;

    uint16_t conversionReady = readRegister(INA236_MASK_ENABLE_REGISTER);
    isDataReady = CHECK_BIT(conversionReady, 3);

    return isDataReady;
}
