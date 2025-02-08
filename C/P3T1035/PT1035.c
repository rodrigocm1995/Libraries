#include "main.h"
#include "math.h"
#include "PT1035.h"


void pt1035WriteRegister8(PT1035_t *pt1035, uint8_t registerAddress, uint8_t value)
{
  uint8_t address[1];
  uint8_t isDeviceReady;
  
  address[0] = value;

  isDeviceReady = HAL_I2C_IsDeviceReady(pt1035->hi2c, (pt1035->devAddress) << 1, PT1035_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(pt1035->hi2c, (pt1035->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

void pt1035WriteRegister16(PT1035_t *pt1035, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(pt1035->hi2c, (pt1035->devAddress) << 1, PT1035_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(pt1035->hi2c, (pt1035->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
 int16_t pt1035ReadRegister8(PT1035_t *pt1035, uint8_t registerAddress)
 {
   uint8_t registerResponse[1];
   uint8_t isDeviceReady;
 
   isDeviceReady = HAL_I2C_IsDeviceReady(pt1035->hi2c, (pt1035->devAddress) << 1, PT1035_TRIALS, HAL_MAX_DELAY);
 
   if (isDeviceReady == HAL_OK)
   {
     HAL_I2C_Mem_Read(pt1035->hi2c, (pt1035->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
   }
 
   return registerResponse[0];
 }

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
int16_t pt1035ReadRegister16(PT1035_t *pt1035, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(pt1035->hi2c, (pt1035->devAddress) << 1, PT1035_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(pt1035->hi2c, (pt1035->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

/**
  * @brief  Create a new instance of the pt1035 temperature sensor setting the I²C port and slave address
  *         Set the Configuration Register with default value 0220h.
  * @param  pt1035 points to an object of type PT1035_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint8_t pt1035Init(PT1035_t *pt1035, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  pt1035->hi2c = i2c;
  pt1035->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint8_t configValue;

  configValue = PT1035_4_SECONDS | PT1035_CONTINUOUS_MODE | PT1035_LATCH_OFF;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, TMP117_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    pt1035WriteRegister8(pt1035, TMP117_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

uint8_t pt1035CustomInit(PT1035_t *pt1035, I2C_HandleTypeDef *i2c, uint8_t devAddress, pt1035ConversionRate_t convRate, pt1035FunctionalMode_t functMode, pt1035LatchMode_t latchMode)
{
  pt1035->hi2c = i2c;
  pt1035->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint8_t configValue;

  configValue = convRate | functMode | latchMode;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, TMP117_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    pt1035WriteRegister8(pt1035, TMP117_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

uint8_t getConfig(PT1035_t *pt1035)
{
  uint8_t value = pt1035ReadRegister8(pt1035, PT1035_CONFIGURATION_REGISTER);
  return value;
}

/**
  * @brief  Verify if the MSB in target register is set to 1.
  *         If bit is set to 1 then makes the two's complement
  * @param  value: Its is the value obtained from target register
  * @retval positive or negative temperature
  */
double pt1035checkAndGetTemp(uint16_t value)
{
	double temperature;
	_Bool isMsb;

	isMsb = CHECK_BIT(value, 15);

	if (isMsb)
	{
		value = ~value + 0x0001;
	}

	temperature = value * (62.5 * pow(10, -3));

	return (!isMsb ? temperature : -temperature);
}


/**
  * @brief  Write the High Limit Temperature in TMP117_THIGH_LIMIT_REGISTER
  *         This register is a 16-bit register that stores the high limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  pt1035 points to an object of type PT1035_t 
  * @param  highLimitTemp The upper desired temperature value, positive or negative
  * @retval none
  */
void pt1035SetHighLimit(PT1035_t *pt1035, int16_t highLimitTemp)
{
	int16_t regValue = abs(highLimitTemp) / (62.5 * pow(10, -3));
  (highLimitTemp < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
  writeRegister(pt1035, PT1035_THIGH_LIMIT_REGISTER, regValue);
}

/**
  * @brief  Reads the TMP117_THIGH_LIMIT_REGISTER
  * @param  pt1035 points to an object of type PT1035_t 
  * @retval temperature in °C
  */
double pt1035GetHighLimit(PT1035_t *pt1035)
{
	  double tempValue;

	  int16_t value = readRegister(pt1035, PT1035_THIGH_LIMIT_REGISTER);
	  tempValue = pt1035checkAndGetTemp(value);

	  return tempValue;
}

/**
  * @brief  Write the Low Limit Temperature in TMP117_TLOW_LIMIT_REGISTER
  *         This register is a 16-bit register that stores the low limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  pt1035 points to an object of type PT1035_t 
  * @param  lowLimitTemp The lower desired temperature value, positive or negative
  * @retval none
  */
void pt1035SetLowLimit(PT1035_t *pt1035, int8_t lowLimitTemp)
{
  uint16_t regValue = abs(lowLimitTemp) / (62.5 * pow(10, -3));

  (lowLimitTemp < 0) ? (regValue = ~regValue + 1) : (regValue = regValue);
  writeRegister(pt1035, PT1035_TLOW_LIMIT_REGISTER, regValue);
}

/**
  * @brief  Reads the TMP117_TLOW_LIMIT_REGISTER
  * @param  pt1035 points to an object of type PT1035_t 
  * @retval temperature in °C
  */
double pt1035ReadLowLimit(PT1035_t *pt1035)
{
  double tempValue;

  int16_t value = readRegister(pt1035, PT1035_TLOW_LIMIT_REGISTER);
  tempValue = pt1035checkAndGetTemp(value);

  return tempValue;
}


double pt1035GetTemperature(PT1035_t *pt1035)
{
  int16_t rawTemp;
  double temperature;

  rawTemp = readRegister(pt1035, PT1035_TEMP_REGISTER) >> 4;
  temperature =  pt1035checkAndGetTemp(rawTemp);
  return temperature;
}

uint16_t pt1035AlertAndDataReady(PT1035_t *pt1035, _Bool *arr)
{
  _Bool isHighAlert;
  _Bool isLowAlert;
  _Bool flagsArr[2];

  uint16_t config = readRegister(pt1035, PT1035_CONFIGURATION_REGISTER);
  isHighAlert = CHECK_BIT(config, 4);
  isLowAlert  = CHECK_BIT(config, 3);

  arr[0] = isHighAlert;
  arr[1] = isLowAlert;
}
