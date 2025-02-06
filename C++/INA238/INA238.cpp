#include <INA238.h>
#include "math.h"

const uint8_t INA238_regSize[MAX_REG_ADDRESS+1] = {
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
    _deviceAddress = devAddress;
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

uint8_t INA238::writeRegister(uint8_t registerAddress, int16_t value)
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

void INA238::init()
{
    _shuntAdcResolution = 5e-6;
    _rangeAdc = 0;
    uint16_t configValue = INA238_0_SEC_DELAY | INA238_163_DOT_84_MV;
    writeRegister(INA238_CONFIG_REGISTER, configValue);
    initAdcConfig();
}

void INA238::init(uint8_t devAddress, TwoWire *wire)
{
    _shuntAdcResolution = 5e-6;
    _rangeAdc = 0;
    _deviceAddress = devAddress;
    _wire = wire;

    uint16_t configValue = INA238_0_SEC_DELAY | INA238_163_DOT_84_MV;
    writeRegister(INA238_CONFIG_REGISTER, configValue);
    initAdcConfig();
}


void INA238::init(uint8_t devAddress, Ina238ConvDly convDly, Ina238AdcRange adcRange, TwoWire *wire)
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

    writeRegister(INA238_CONFIG_REGISTER, configValue);
}

void INA238::initAdcConfig()
{
  uint16_t value = INA238_CONT_TEMP_SBVOLTAGE | INA238_1052_USEC_VBUS | INA238_1052_USEC_VSH | INA238_1052_USEC_TEMP | INA238_1_COUNT;
  writeRegister(INA238_ADC_CONFIG_REGISTER, value);
}

void INA238::initAdcConfig(Ina238Mode mode, Ina238VbusConvTime vbusCt, Ina238VshuntConvTime vShuntCt, Ina238TempConvTime tempCt, Ina238Avg avg)
{
  uint16_t value = mode | vbusCt | vShuntCt | tempCt | avg;
  writeRegister(INA238_ADC_CONFIG_REGISTER, value);
}

/**
  * @brief  Compute the calibration value  that will be written into the 
  *         SHUNT_CAL_REGISTER. Also sets defaults Limits:
  *         - setShuntOverVoltage
  *         - setShuntUnderVoltage
  *         - setBusUnderVoltage
  *         - setBusOverVoltage
  * @param  rShuntValue: The shunt resistor value
  * @param  maxCurrent: The maximum current the user wants to measure
  * @retval shuntCalvalue to corroborate the value written in the register
  */
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

    setShuntOverVoltage();
    setShuntUnderVoltage();
    setBusUnderVoltage();
    setBusOverVoltage();
    writeRegister(INA238_SHUNT_CAL_REGISTER, shuntCal);

    return shuntCal;
}

/**
  * @brief  Sets a default threshold for comparison of the Value to detect Shunt
  *         Overvoltage (overcurrent protection). Two's complement value.
  *         This default value is based on the maximum current and Resistance value
  *         selected by the user. 
  * @param  none
  * @retval -
  */
 uint8_t INA238::setShuntOverVoltage()
 {
     float shuntVoltageLimit = _maximumCurrent * _rShunt;
     int16_t shuntOverVoltageLimitRegister = shuntVoltageLimit/_shuntAdcResolution;
 
     writeRegister(INA238_SOVL_REGISTER, shuntOverVoltageLimitRegister);
     return shuntOverVoltageLimitRegister;
 }

/**
  * @brief  Sets a default threshold for comparison of the Value to detect Shunt
  *         Undervoltage (undercurrent protection). Two's complement value.
  *         Write the negative value of the overvoltageLimit. 
  * @param  none
  * @retval -
  */
uint8_t INA238::setShuntUnderVoltage()
{
    int16_t sovl = setShuntOverVoltage();
    sovl = ~sovl + 1;
    writeRegister(INA238_SUVL_REGISTER, sovl);
}

/**
  * @brief  Sets a custom threshold for comparison of the Value to detect
  *         Shunt Undervoltage (undercurrent protection). User must know the
  *         selected full scale range (±163.84mV or ±40.96mV)
  * @param  shuntUnderVoltage: The value of the shunt voltage that will trigger the alert
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t INA238::setShuntUnderVoltage(float shuntUnderVoltage)
{
    int16_t shuntVoltageLimit = shuntUnderVoltage / _shuntAdcResolution;
    writeRegister(INA238_SOVL_REGISTER, shuntVoltageLimit);
    return shuntVoltageLimit;
}

/**
  * @brief  Sets a default threshold for comparison of the Value to detect Shunt
  *         Overvoltage (overcurrent protection). User must know the
  *         selected full scale range (±163.84mV or ±40.96mV)
  * @param  shuntUnderVoltage: The value of the shunt voltage that will trigger the alert
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t INA238::setShuntOvervoltage(float shuntOverVoltage)
{
    int16_t shuntVoltageLimit = shuntOverVoltage / _shuntAdcResolution;
    writeRegister(INA238_SOVL_REGISTER, shuntVoltageLimit);
    return shuntVoltageLimit;
}

int16_t INA238::getTemperatureLimit()
{
  
}

/**
  * @brief  Sets a default threshold of 0 for comparison of the value to detect Bus Undervoltage
  *         (undervoltage protection). Unsigned representation, positive value only.
  *         Conversion Factor: 3.125 mV/LSB
  * @param  busUnderVoltage: float positive value to select the threshold
  *         Range: 0.0-85.0 
  * @retval uint 16-bit data to corroborate the value written in the register
  */
 uint8_t INA238::setBusUnderVoltage()
 {
     writeRegister(INA238_BOVL_REGISTER, 0x0000);
 }

/**
  * @brief  Sets a default maximum threshold for comparison of the value to detect Bus Overvoltage
  *         (overvoltage protection). Unsigned representation, positive value only.
  *         Conversion Factor: 3.125 mV/LSB
  * @param  busUnderVoltage: float positive value to select the threshold
  *         Range: 0.0-85.0 
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint8_t INA238::setBusOverVoltage()
{
    writeRegister(INA238_BOVL_REGISTER, 0x7FFF);
}

/**
  * @brief  Sets a custom threshold for comparison of the value to detect Bus Undervoltage
  *         (undervoltage protection). Unsigned representation, positive value only.
  *         Conversion Factor: 3.125 mV/LSB
  * @param  busUnderVoltage: float positive value to select the threshold
  *         Range: 0.0-85.0 
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint16_t INA238::setBusUnderVoltage(float busUnderVoltage)
{
    uint16_t busVoltageLimit =  busUnderVoltage / 3.125e-3;
    writeRegister(INA238_BOVL_REGISTER, busVoltageLimit);
    return busVoltageLimit;
}

/**
  * @brief  Sets the threshold for comparison of the value to detect Bus Overvoltage
  *         (overvoltage protection). Unsigned representation, positive value only.
  *         Conversion Factor: 3.125 mV/LSB
  * @param  busUnderVoltage: float positive value to select the threshold
  *         Range: 0.0-85.0 
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint16_t INA238::setBusOverVoltage(float busOverVoltage)
{
    uint16_t busVoltageLimit =  busOverVoltage / 3.125e-3;
    writeRegister(INA238_BOVL_REGISTER, busVoltageLimit);
    return busVoltageLimit;
}

/**
  * @brief  Sets the threshold for comparison of the value to detect Bus Over 
  *         temperature measurements. Two's complement value.
  *         Conversion Factor: 125 m°C/LSB.
  * @param  temperature: float positive value to select the threshold temperature
  *         Range: -40.0 to +125.0 
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t  INA238::setTemperatureLimit(float temperature)
{
    int16_t temperatureLimit = temperature / 125e-3;
    writeRegister(INA238_TEMP_LIMIT_REGISTER, temperatureLimit);
    return temperatureLimit;
}

/**
  * @brief  Sets the threshold for comparison of the value to detect power over- 
  *         limit measurements. Unsigned representation, positive value only.
  *         Conversion Factor: 256 * Power_LSB.
  * @param  power: float positive value to select the threshold temperature
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint16_t INA238::setPowerLimit(float power)
{
    uint16_t powerLimit =  power / (256.0 * 0.2 * _currentLsbMin);
    writeRegister(INA238_POWER_LIMIT_REGISTER, powerLimit);
    return powerLimit;
}

/**
  * @brief  Gets the raw value of the SUVL (Shunt Under Voltage Limit) register
  * @param  none
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t INA238::getShuntUnderVoltage()
{
    int16_t value = readRegister(INA238_SUVL_REGISTER);
    return value;
}

/**
  * @brief  Gets the raw value of the SOVL (Shunt Over Voltage Limit) register
  * @param  none
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t INA238::getShuntOverVoltage()
{
    int16_t value = readRegister(INA238_SOVL_REGISTER);
    return value;
}

/**
  * @brief  Gets the raw value of the BUVL (Bus Under Voltage Limit) register
  * @param  none
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint16_t INA238::getBusUnderVoltage()
{
    uint16_t value = readRegister(INA238_BUVL_REGISTER);
    return value;
}

/**
  * @brief  Gets the raw value of the BOVL (Bus Over Voltage Limit) register
  * @param  none
  * @retval uint 16-bit data to corroborate the value written in the register
  */
uint16_t INA238::getBusOverVoltage()
{
    uint16_t value = readRegister(INA238_BOVL_REGISTER);
    return value;
}

/**
  * @brief  Gets the raw value of the Temperature Limit register
  * @param  none
  * @retval int 16-bit data to corroborate the value written in the register
  */
int16_t INA238::getTemperatureLimit()
{
    int16_t value = readRegister(INA238_TEMP_LIMIT_REGISTER);
    return value;
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
  uint32_t rawPower = readRegister(INA238_POWER_REGISTER);
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

