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
  ads1115->lsbSize = 62.5e-6;
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

uint8_t Ads1115CustomInit(ADS1115_t *ads1115, I2C_HandleTypeDef *i2c, uint8_t devAddress, ADS115Mux muxType, ADS115Pga pga, ADS115Mode mode, ADS1115DataRate dataRate, ADS1115CompMode compMode, ADS1115CompPol pol, ADS1115CompLatch latch, ADS1115CompQueue queue)
{
  ads1115->hi2c = i2c;
  ads1115->devAddress = devAddress;
  uint8_t isDeviceReady;
  uint16_t configValue;

  configValue = muxType | pga | mode | dataRate | compMode | pol | latch | queue;
  isDeviceReady = HAL_I2C_IsDeviceReady(i2c, devAddress << 1, ADS1115_TRIALS, HAL_MAX_DELAY);

  if (pga == FSR_6_DOT_144_V) ads1115->lsbSize = 187.5e-6;
  else if (pga == FSR_4_DOT_096_V) ads1115->lsbSize = 125e-6;
  else if (pga == FSR_2_DOT_048_V) ads1115->lsbSize = 62.5e-6;
  else if (pga == FSR_1_DOT_024_V) ads1115->lsbSize = 31.25e-6;
  else if (pga == FSR_0_DOT_512_V) ads1115->lsbSize = 15.625e-6;
  else if (pga == FSR_0_DOT_256_V) ads1115->lsbSize = 7.8125e-6;

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

uint16_t Ads1115GetConversion(ADS1115_t *ads1115)
{
    uint16_t value = Ads1115ReadRegister(ads1115, ADS1115_CONVERSION_REGISTER);
    return value;
}

double Ads1115GetVoltage(ADS1115_t *ads1115)
{
    uint16_t value = Ads1115GetConversion(ads1115);
    double volt = value * (ads1115->lsbSize);
    return volt;
}



_Bool Ads1115IsDeviceReady(ADS1115_t *ads1115)
{
	uint16_t config = Ads1115ReadRegister(ads1115, ADS1115_CONFIGURATION_REGISTER);
	_Bool isReady = CHECK_BIT(config, 15);

	if (isReady == 0)
		return 1;
	else
		return 0;
}
