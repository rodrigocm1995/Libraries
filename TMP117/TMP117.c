#include "main.h"
#include "math.h"
#include "TMP117.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  tmp117 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void writeRegister(Tmp117_t *tmp117, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(tmp117->hi2c, (tmp117->devAddress) << 1, TMP117_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(tmp117->hi2c, (tmp117->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t readRegister(Tmp117_t *tmp117, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(tmp117->hi2c, (tmp117->devAddress) << 1, TMP117_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(tmp117->hi2c, (tmp117->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

/**
  * @brief  Create a new instance of the tmp117 temperature sensor setting the I²C port and slave address
  *         Set the Configuration Register with default value 0220h.
  * @param  tmp117 points to an object of type Tmp117_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint8_t tmp117DefaultInit(Tmp117_t *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  tmp117->hi2c = i2c;
  tmp117->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = MOD_CONTINUOUS_CONVERSION | CONVERSION_TYPE_4 |  AVERAGED_CONVERSIONS_8 | ALERT_MODE;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, TMP117_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    writeRegister(tmp117, TMP117_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

/**
  * @brief  Create a new instance of the tmp117 temperature sensor setting the I²C port and slave address
  *         Set the Configuration Register with a custom value
  * @param  tmp117 points to an object of type Tmp117_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @param  conversionMode Indicates if device is used as continuous conversion, shutdown or One-shot mode
  * @param  conversionCycle Set the total conversion cycle time
  * @param  averagingMode Determines the number of conversion results that are collected and average before
  *         updating the temperature register.
  * @param  mode Therm/Alert mode select.
  *         1: Therm mode
  *         0: Alert mode
  * @param  polarity Set the polarity pin bit
  *         1: active high
  *         0: active low
  * @param  alert monitors the state of the Data_Ready Flag on the ALERT pin
  *         1: ALERT pin reflects the status of the data ready flag
  *         0: ALERT pin reflects the status of the alert flags
  * @retval return 1 if OK
  */
uint16_t tmp117Init(Tmp117_t *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress, tmp117ConversionMode_t conversionMode, tmp117ConversionCycleTime_t conversionCycle, tmp117AveragingMode_t averagingMode, tmp117ThermAlertMode_t mode, tmp117Polarity_t polarity, tmp117Alert_t alert)
{
  tmp117->hi2c = i2c;
  tmp117->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = conversionMode | conversionCycle | averagingMode | mode | polarity | alert;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, TMP117_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    writeRegister(tmp117, TMP117_CONFIGURATION_REGISTER, configValue);
    return configValue;
  }

  return 0;
}

/**
  * @brief  Verify if the MSB in target register is set to 1.
  *         If bit is set to 1 then makes the two's complement
  * @param  value: Its is the value obtained from target register
  * @retval positive or negative temperature
  */
double tmp117checkAndGetTemp(uint16_t value)
{
	double temperature;
	_Bool isMsb;

	isMsb = CHECK_BIT(value, 15);

	if (isMsb)
	{
		value = ~value + 0x0001;
	}

	temperature = value * (7.8125 * pow(10, -3));

	return (!isMsb ? temperature : -temperature);
}


/**
  * @brief  Write the High Limit Temperature in TMP117_THIGH_LIMIT_REGISTER
  *         This register is a 16-bit register that stores the high limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  tmp117 points to an object of type Tmp117_t 
  * @param  highLimitTemp The upper desired temperature value, positive or negative
  * @retval none
  */
void tmp117SetHighLimit(Tmp117_t *tmp117, int8_t highLimitTemp)
{
	uint16_t regValue = abs(highLimitTemp) / (7.8125 * pow(10, -3));
  (highLimitTemp < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
  writeRegister(tmp117, TMP117_THIGH_LIMIT_REGISTER, regValue);
}

/**
  * @brief  Reads the TMP117_THIGH_LIMIT_REGISTER
  * @param  tmp117 points to an object of type Tmp117_t 
  * @retval temperature in °C
  */
double tmp117ReadHighLimit(Tmp117_t *tmp117)
{
	  double tempValue;

	  int16_t value = readRegister(tmp117, TMP117_THIGH_LIMIT_REGISTER);
	  tempValue = tmp117checkAndGetTemp(value);

	  return tempValue;
}

/**
  * @brief  Write the Low Limit Temperature in TMP117_TLOW_LIMIT_REGISTER
  *         This register is a 16-bit register that stores the low limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  tmp117 points to an object of type Tmp117_t 
  * @param  lowLimitTemp The lower desired temperature value, positive or negative
  * @retval none
  */
void tmp117SetLowLimit(Tmp117_t *tmp117, int8_t lowLimitTemp)
{
  uint16_t regValue = abs(lowLimitTemp) / (7.8125 * pow(10, -3));

  (lowLimitTemp < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
  writeRegister(tmp117, TMP117_TLOW_LIMIT_REGISTER, regValue);
}

/**
  * @brief  Reads the TMP117_TLOW_LIMIT_REGISTER
  * @param  tmp117 points to an object of type Tmp117_t 
  * @retval temperature in °C
  */
double tmp117ReadLowLimit(Tmp117_t *tmp117)
{
  double tempValue;

  int16_t value = readRegister(tmp117, TMP117_TLOW_LIMIT_REGISTER);
  tempValue = tmp117checkAndGetTemp(value);

  return tempValue;
}


double tmp117ReadTemperature(Tmp117_t *tmp117)
{
  int16_t rawTemp;
  double temperature;

  rawTemp = readRegister(tmp117, TMP117_TEMP_RESULT_REGISTER);
  temperature =  tmp117checkAndGetTemp(rawTemp);
  return temperature;
}

// To program the EEPROM, first unlock the EEPROM by setting the EUN bit in the EEPROM unlock register.
// Programming a single location takes 7ms to complete.
// During programming, the EEPROM_busy flag is set.
// Read this flag to monitor if the programming is complete

void tmp117ProgramEeprom(Tmp117_t *tmp117)
{
  writeRegister(tmp117, TMP117_EEPROM_UNLOCK_REGISTER, TMP117_EEPROM_EUN);
  
}

uint16_t tmp117AlertAndDataReady(Tmp117_t *tmp117, _Bool *arr)
{
  _Bool isHighAlert;
  _Bool isLowAlert;
  _Bool isDataReady;
  _Bool flagsArr[3];

  uint16_t config = readRegister(tmp117, TMP117_CONFIGURATION_REGISTER);
  isHighAlert = CHECK_BIT(config, 15);
  isLowAlert  = CHECK_BIT(config, 14);
  isDataReady = CHECK_BIT(config, 13);

  arr[0] = isHighAlert;
  arr[1] = isLowAlert;
  arr[2] = isDataReady;
}
