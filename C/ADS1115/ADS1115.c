#include "main.h"
#include "math.h"
#include "ADS1115.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  tmp117 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void Ads1115WriteRegister(ADS1115_t *ads1115, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(ads1115->hi2c, (ads1115->devAddress) << 1, ADS1115_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(ads1115->hi2c, (ads1115->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t Ads1115ReadRegister(ADS1115_t *ads1115, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(ads1115->hi2c, (ads1115->devAddress) << 1, ADS1115_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(ads1115->hi2c, (ads1115->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}


/**
  * @brief  Create a new instance of the ads1115 adc sensor setting the I²C port and slave address
  *         Set the Configuration Register with default value 0220h.
  * @param  tmp117 points to an object of type ADS1115_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint8_t Ads1115Init(ADS1115_t *ads1115, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  ads1115->hi2c = i2c;
  ads1115->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = AINP_IS_AIN0_AINN_IS_AIN1 | FSR_2_DOT_048_V | ADS1115_SINGLE_SHOT | ADS1115_128_SPS | ADS1115_TRADITIONAL_COMP | ADS1115_ACTIVE_LOW | ADS1115_NONLATCHING_COMP | ADS1115_DISABLE_COMPARATOR; 
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, ADS1115_TRIALS, HAL_MAX_DELAY);
  
  if (isDeviceReady == HAL_OK)
  {
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  Ads1115WriteRegister(ads1115, ADS1115_CONFIGURATION_REGISTER, configValue);
    return 1;
  }

  return 0;
}

uint16_t Ads1115GetConfigRegister(ADS1115_t *ads1115)
{
	uint16_t value = Ads1115ReadRegister(ads1115, ADS1115_CONFIGURATION_REGISTER);
	return value;
}


