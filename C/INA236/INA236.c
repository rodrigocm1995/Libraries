#include "main.h"
#include "math.h"
#include "INA236.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ina219 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void ina236writeRegister(Ina236_t *ina236, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(ina236->hi2c, (ina236->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t ina236readRegister(Ina236_t *ina236, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(ina236->hi2c, (ina236->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

/**
  * @brief  Initializes the CONFIGURATION register's device with default values 
  * @param  ina236 points to an object of the type Ina236_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  devAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint8_t ina236DefaulInit(Ina236_t *ina236, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  ina236->hi2c = i2c;
  ina236->devAddress = devAddress;
  ina236->shuntAdcRange = 2.5 * pow(10, -6);
  uint16_t configValue;
  uint8_t isDeviceReady;

  configValue = INA236_CONFIG_RESERVED | INA236_ADCRANGE_81DOT92_MILIVOLT | INA236_1_SAMPLE | INA236_VBUSCT_1100_US | INA236_VSHCT_1100_US | INA236_CONTINUOUS_SHUNTBUS_VOLTAGE;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    ina236writeRegister(ina236, INA236_CONFIGURATION_REGISTER, configValue);
    ina236ResetMaskRegister(ina236);
    return 1;
  }

  return 0;
}

uint8_t ina236Init(Ina236_t *ina236, I2C_HandleTypeDef *i2c, uint8_t devAddress, AdcRange_t adcRange, Avg_t samples, VbusConvertionTime_t busConvertionTime, ShuntConvertionTime_t shuntConvertionTime, ina236Mode_t mode)
{
  ina236->hi2c = i2c;
  ina236->devAddress = devAddress;
  uint16_t configValue;
  uint8_t isDeviceReady;

  if (adcRange == INA236_ADCRANGE_81DOT92_MILIVOLT)
  {
	  ina236->shuntAdcRange = 2.5 * pow(10, -6);
	  ina236->rangeAdc = 0;
  }
  else if (adcRange == INA236_ADCRANGE_20DOT48_MILIVOLT)
  {
	  ina236->shuntAdcRange = 625 * pow(10, -9);
	  ina236->rangeAdc = 1;
  }

  configValue = INA236_CONFIG_RESERVED | adcRange | samples | busConvertionTime | shuntConvertionTime | mode;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    ina236writeRegister(ina236, INA236_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0; 
}

/**
  * @brief  Initializes the CONFIGURATION register's device with default values 
  * @param  ina236 points to an object of the type Ina236_t 
  * @param  rShuntValue Shunt Resistor's Value
  * @param  maxCurrent Maximum current monitored
  * @retval 
  */
void ina236SetCalibration(Ina236_t *ina236, double rShuntValue, int maxCurrent)
{
  ina236->shuntResistor = rShuntValue;
  ina236->maximumCurrent = maxCurrent;
  double currentLsbMinimum;
  uint16_t shuntCal;

  currentLsbMinimum = maxCurrent / pow(2, 15);
  if (maxCurrent < 3) currentLsbMinimum = 100.0 * pow(10, -6);
  else if (maxCurrent > 3 && maxCurrent <= 6) currentLsbMinimum  = 200.0 * pow(10, -6);
  else if (maxCurrent > 6 && maxCurrent <= 9) currentLsbMinimum  = 300.0 * pow(10, -6);
  else if (maxCurrent > 9 && maxCurrent <= 13) currentLsbMinimum = 500.0 * pow(10, -6);
  else if (maxCurrent > 13 && maxCurrent <= 16) currentLsbMinimum = 500.0 * pow(10, -6);
  else if (maxCurrent > 16 && maxCurrent <= 19) currentLsbMinimum = 600.0 * pow(10, -6);

  ina236->currentLsbMin = currentLsbMinimum;

  shuntCal = 0.00512 / (currentLsbMinimum * rShuntValue);

  if (ina236->rangeAdc == 1)
  {
    shuntCal = shuntCal / 4;
  }

  ina236writeRegister(ina236, INA236_CALIBRATION_REGISTER, shuntCal);
}

uint8_t ina236SetMaskRegister(Ina236_t *ina236, AlertType_t alertType)
{
  ina236->alertType = alertType;
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
  ina236writeRegister(ina236, INA236_MASK_ENABLE_REGISTER, value);
  return value;
}

void ina236ResetMaskRegister(Ina236_t *ina236)
{
  ina236writeRegister(ina236, INA236_MASK_ENABLE_REGISTER, 0x0000);
}

void ina236SetCurrentAlertLimit(Ina236_t *ina236, double currentLimit)
{
  double shuntVoltageLimit;
  uint16_t alertValue;

  shuntVoltageLimit = currentLimit * (ina236->shuntResistor);
  alertValue = shuntVoltageLimit / (ina236->shuntAdcRange);
  ina236writeRegister(ina236, INA236_ALERT_LIMIT_REGISTER, alertValue);
}

double ina236ReadShuntVoltage(Ina236_t *ina236)
{
	double result = ina236readRegister(ina236, INA236_SHUNT_VOLTAGE_REGISTER);
	result = result * (ina236->shuntAdcRange) * (1000.0);

	return result;
}

double ina236ReadBusVoltage(Ina236_t *ina236)
{
	double result = ina236readRegister(ina236, INA236_BUS_VOLTAGE_REGISTER);
	result = result * (1.6) * pow(10, -3);

	return result;
}

double ina236ReadCurrent(Ina236_t *ina236)
{
  double rawCurrent = ina236readRegister(ina236, INA236_CURRENT_REGISTER);
  double current = rawCurrent * (ina236->currentLsbMin) * (1000.0);
  return current;
}

double ina236ReadPower(Ina236_t *ina236)
{
  double rawPower = ina236readRegister(ina236, INA236_POWER_REGISTER);
  double power = 32.0 * rawPower * (ina236->currentLsbMin) * (1000.0);
  return power;
}

_Bool ina236AlertFunctionFlag(Ina236_t *ina236)
{
	uint16_t result = ina236readRegister(ina236, INA236_MASK_ENABLE_REGISTER);
	_Bool isAlertFunctionFlagTrue = CHECK_BIT(result, 4);
	return isAlertFunctionFlagTrue;
}

_Bool ina236DataReady(Ina236_t *ina236)
{
  _Bool isDataReady;

  uint16_t conversionReady = ina236readRegister(ina236, INA236_MASK_ENABLE_REGISTER);
  isDataReady = CHECK_BIT(conversionReady, 3);

  return isDataReady;
}
