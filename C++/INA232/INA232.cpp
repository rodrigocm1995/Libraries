#include <INA232.h>
#include "math.h"

INA232::INA232()
{

}


uint16_t INA232::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

uint16_t INA232::readRegister(uint8_t registerAddress)
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


uint8_t INA232::defaultInit(uint8_t devAddress, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;
    _shuntAdcRange = 2.5e-6;

    uint16_t configValue = INA232_CONFIG_RESERVED | INA232_ADCRANGE_81DOT92_MILIVOLT | INA232_1_SAMPLE | INA232_VBUSCT_1100_US | INA232_VSHCT_1100_US | INA232_CONTINUOUS_SHUNTBUS_VOLTAGE;
    
    return ( (writeRegister(INA232_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint8_t INA232::init(uint8_t devAddress, Ina232AdcRange_t adcRange, Ina232Avg_t samples, Ina232VbusConvertionTime_t busConvertionTime, Ina232ShuntConvertionTime_t shuntConvertionTime, Ina232Mode_t mode, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    if (adcRange == INA232_ADCRANGE_81DOT92_MILIVOLT)
    {
	    ina236->shuntAdcRange = 2.5e-6;
	    _rangeAdc = 0;
    }
    else if (adcRange == INA232_ADCRANGE_20DOT48_MILIVOLT)
    {
	    ina236->shuntAdcRange = 625e-9;
	    _rangeAdc = 1;
    }

    uint16_t configValue = INA232_CONFIG_RESERVED | adcRange | samples | busConvertionTime | shuntConvertionTime | mode;

    return ( (writeRegister(INA232_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint16_t INA232::setCalibration(float rShuntValue, float maxCurrent)
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

    writeRegister(INA232_CALIBRATION_REGISTER, shuntCal);
}

void INA232::setMaskRegister(Ina232AlertType_t alertType)
{
    _alertType = alertType;
    uint16_t value;

    switch(alertType)
    {
        case 1:
            value = INA232_SHUNT_OVER_LIMIT;
            break;
        case 2:
            value = INA232_SHUNT_UNDER_LIMIT;
            break;
        case 3:
            value = INA232_BUS_OVER_LIMIT;
            break;
        case 4: 
            value = INA232_BUS_UNDER_LIMIT;
            break;
        case 5:
            value = INA232_POWER_OVER_LIMIT;
            break;
        case 6:
            value = INA232_CONVERSION_READY;
            break;
        default: return 0;
  }
  value = value | 0x0000;
  writeRegister(INA232_MASK_ENABLE_REGISTER, value);
  return value;
}

void INA232::resetMaskRegister()
{
    writeRegister(INA232_MASK_ENABLE_REGISTER, 0x0000);
}

void INA232::setCurrentAlertLimit(float currentLimit)
{
    float shuntVoltageLimit;
    uint16_t alertValue;

    shuntVoltageLimit = currentLimit * _rShunt;
    alertValue = shuntVoltageLimit / _shuntAdcRange;
    writeRegister(INA232_ALERT_LIMIT_REGISTER, alertValue);
}

float INA232::getShuntVoltage()
{
    float result = readRegister(INA232_SHUNT_VOLTAGE_REGISTER);
    result = result *  _shuntAdcRange * 1000.0;

    return result;
}

float INA232::getBusVoltage()
{
	float result = readRegister(INA232_BUS_VOLTAGE_REGISTER);
	result = result * (1.6) * 1e-3;

	return result;
}

float INA232::getCurrent()
{
  float rawCurrent = readRegister(INA232_CURRENT_REGISTER);
  float current = rawCurrent * (_currentLsbMin) * (1000.0);
  return current;
}

float INA232::getPower()
{
  float rawPower = readRegister(INA232_POWER_REGISTER);
  float power = 32.0 * rawPower * (_currentLsbMin) * (1000.0);
  return power;   
}

bool INA232::alertFunctionFlag()
{
	uint16_t result = readRegister(INA232_MASK_ENABLE_REGISTER);
	bool isAlertFunctionFlagTrue = CHECK_BIT(result, 4);
	return isAlertFunctionFlagTrue;   
}

bool INA232::dataReady()
{
    bool isDataReady;

    uint16_t conversionReady = readRegister(INA232_MASK_ENABLE_REGISTER);
    isDataReady = CHECK_BIT(conversionReady, 3);

    return isDataReady;
}