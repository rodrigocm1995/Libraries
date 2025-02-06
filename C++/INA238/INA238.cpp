#include <INA238.h>
#include "math.h"

const uint8_t INA238_regSize[maxRegAddress+1] = {
                                            2,2,2,0,2,2,2,2,\
                                            3,0,0,2,2,2,2,2,\
                                            2,2,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,2,2
};

INA238::INA238()
{

}

bool INA238::begin(uint8_t devAddress, TwoWire *wire)
{
    _shuntAdcResolution = 5e-6;
    _rangeAdc = 0;
    _devAddress = devAddress;
    _wire = wire;

    if (!isConnected()) return false;
    return true;
}

bool INA238::isConnected()
{
  if ((_deviceAddress < 0x40) || (_deviceAddress > 0x4F)) return false;
  _wire->beginTransmission(_deviceAddress);
  return ( _wire->endTransmission() == 0);
}

uint8_t INA238::writeRegister(uint8_t registerAddress, uint16_t value)
{
    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->write(highByte(value));
    _wire->write(lowByte(value));
    return _wire->endTransmission();
}

int32_t INA238::readRegister(uint8_t registerAddress)
{
    uint8_t i;
    int8_t registerResponse[3] = {0};
    int32_t value = 0;

    _wire->beginTransmission(_deviceAddress);
    _wire->write(registerAddress);
    _wire->endTransmission();
    _wire->requestFrom(_deviceAddress, INA238_regSize[registerAddress]);
    for (i = 0; i < INA238_regSize[registerAddress]; i++)
    {
        registerResponse[i] = _wire->read();
    }
    for (i = 0; i < sizeof(registerResponse); i++)
    {
        value = (value << 8) | registerResponse[i];
    }

    return value;
}

uint8_t INA238::init()
{
    _shuntAdcResolution = 5e-6;
    _rangeAdc = 0;
    uint16_t configValue = INA238_0_SEC_DELAY | INA238_163_DOT_84_MV | INA238_CONT_TEMP_SBVOLTAGE | INA238_1052_USEC_VBUS | INA238_1052_USEC_VSH | INA238_1052_USEC_TEMP | INA238_1_COUNT;
    return ( (writeRegister(INA238_CONFIG_REGISTER, configValue) == 0) ? 1 : 0 );
}

uint8_t INA238::init(uint8_t devAddress, TwoWire *wire)
{
    _shuntAdcResolution = 5e-6;
    _rangeAdc = 0;
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = INA238_0_SEC_DELAY | INA238_163_DOT_84_MV;
    return ( (writeRegister(INA238_CONFIG_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint8_t INA238::init(uint8_t devAddress, Ina238ConvDly convDly, Ina238AdcRange adcRange, TwoWire *wire)
{
    _deviceAddress = devAddress;
    _wire = wire;

    if (adcRange == INA238_163_DOT_84_MV)
    {
	    _shuntAdcResolution = 5e-6;
        _rangeAdc = 0;
    }
    else if (adcRange == INA238_40_DOT_96_MV)
    {
	    _shuntAdcResolution = 1.25e-6;
	    _rangeAdc = 1;
    }

    uint16_t configValue = convDly | adcRange;

    return ( (writeRegister(INA236_CONFIGURATION_REGISTER, configValue) == 0) ? 1 : 0 );
}


uint16_t INA238::setCalibration(float rShuntValue, float maxCurrent)
{
    _rShunt = rShuntValue;
    _maximumCurrent = maxCurrent;
    float currentLsbMinimum;
    uint16_t shuntCal;

    currentLsbMinimum = maxCurrent / pow(2, 15);
    shuntCal = 819.2e6 * currentLsbMinimum * rShuntValue;
    _currentLsbMin = currentLsbMinimum;

    if (_rangeAdc == 1)
    {
        shuntCal = shuntCal * 4;
    }

    writeRegister(INA238_SHUNT_CAL_REGISTER, shuntCal);
    return shuntCal;
}


int16_t INA238::setShuntUnderVoltage(float shuntUnderVoltage)
{

}

int16_t INA238::setShuntOverVoltage(float shuntOverVoltage)
{
    float shuntVoltageLimit = _maximumCurrent * _rShunt;
    uint16_t shuntOverVoltageLimitRegister = shuntVoltageLimit/_shuntAdcResolution;

    writeRegister(INA238_SOVL_REGISTER, shuntOverVoltageLimitRegister);
    return shuntOverVoltageLimitRegister;
}

int16_t INA238::getShuntUnderVoltage()
{
    int16_t value = readRegister(INA238_SUVL_REGISTER);
    return value;
}

int16_t INA238::getShuntOverVoltage()
{
    int16_t value = readRegister(INA238_SOVL_REGISTER);
    return value;
}

int16_t INA238::getShuntOverVoltage()
{
    uint16_t value = readRegister(INA238_SOVL_REGISTER);
    return value;
}

void INA238::setMaskRegister(Ina236AlertType_t alertType)
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

void INA238::resetMaskRegister()
{
    writeRegister(INA236_MASK_ENABLE_REGISTER, 0x0000);
}

void INA238::setCurrentAlertLimit(float currentLimit)
{
    float shuntVoltageLimit;
    uint16_t alertValue;

    shuntVoltageLimit = currentLimit * _rShunt;
    alertValue = shuntVoltageLimit / _shuntAdcRange;
    writeRegister(INA236_ALERT_LIMIT_REGISTER, alertValue);
}

/**
  * @brief  Read the value of the Shunt Voltage Register.
  *         Differential voltage measured across the shunt
  *         output. Two's complement value. Conversion factor:
  *         - 5µV/LSB when ADCRANGE = 0
  *         - 1.25µV/LSB when ADCRANGE = 1 
  * @param  none
  * @retval Return the value of the shunt voltage in mV
  */
float INA238::getShuntVoltage()
{
    int16_t rawShuntVoltage = readRegister(INA238_VSHUNT_REGISTER);
    float result = (float)rawShuntVoltage *  _shuntAdcResolution * 1000.0;

    return result;
}

/**
  * @brief  Read the value of the Bus Voltage Register.
  *         Two's complement value. However, always positive
  * @param  none
  * @retval Return the value of the shunt voltage in V
  */
float INA238::getBusVoltage()
{
	uint16_t rawBusRegister = readRegister(INA238_VBUS_REGISTER);
	float result = (float)rawBusRegister * 3.125e-3;

	return result;
}


/**
  * @brief  Read the value of the Current Register. The final value
  *         is scaled by CURRENT_LSB (represented as _currentLsbMin)
  * @param  none
  * @retval Return the current value in mA
  */
float INA238::getCurrent()
{
  int16_t rawCurrent = readRegister(INA238_CURRENT_REGISTER);
  float current = (float)rawCurrent * (_currentLsbMin) * (1000.0); 
  return current;
}

/**
  * @brief  Read the value of the 24-bit Power Register. The final value
  *         is scaled by 0.2 x CURRENT_LSB
  * @param  none
  * @retval Return the Power value in mW
  */
float INA238::getPower()
{
  uint32_t rawPower = readRegister(INA236_POWER_REGISTER);
  float power = 0.2 * rawPower * (_currentLsbMin) * (1000.0);
  return power;   
}

/**
  * @brief  Read the value of the DIETEMP Register. Two's complement value
  *         Conversion Factor: 125 m°C/LSB
  * @param  none
  * @retval Return the the temperature in °C
  */
float INA238::getTemperature()
{
  int16_t rawTemperature = readRegister(INA238_DIETEMP_REGISTER);
  //rawTemperature >>= 3;
  float temperature = rawTemperature * 125e-3;
  return temperature;   
}

bool INA238::alertFunctionFlag()
{
	uint16_t result = readRegister(INA236_MASK_ENABLE_REGISTER);
	bool isAlertFunctionFlagTrue = CHECK_BIT(result, 4);
	return isAlertFunctionFlagTrue;   
}

bool INA238::dataReady()
{
    bool isDataReady;

    uint16_t conversionReady = readRegister(INA238, INA236_MASK_ENABLE_REGISTER);
    isDataReady = CHECK_BIT(conversionReady, 3);

    return isDataReady;
}