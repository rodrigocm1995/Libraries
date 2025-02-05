#include "main.h"
#include "math.h"
#include "ZTPD2210.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  tmp117 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void ZTPD2210writeRegister(ZTPD2210_t *ztpd2210, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(ztpd2210->hi2c, (ztpd2210->devAddress) << 1, ZTPD2210_TRIALS , HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(ztpd2210->hi2c, (ztpd2210->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 64-bit data read from register's device
  */
uint64_t ZTPD2210readRegister(ZTPD2210_t *ztpd2210, uint8_t registerAddress)
{
  uint8_t registerResponse[7];
  uint8_t isDeviceReady;
  uint64_t value = 0;

  isDeviceReady = HAL_I2C_IsDeviceReady(ztpd2210->hi2c, (ztpd2210->devAddress) << 1, ZTPD2210_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(ztpd2210->hi2c, (ztpd2210->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  for (uint8_t i = 0; i < sizeof(registerResponse); i++)
  {
    value = (value << 8) | registerResponse[i];  
  }

  return value;
}

/**
  * @brief  Create a new instance of the ZTPD2210 temperature sensor setting the I²C port and slave address
  * @param  ztpd2210 points to an object of type ZTPD2210_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval none
  */
void ZTPD2210Init(ZTPD2210_t *ztpd2210, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  ztpd2210->hi2c = i2c;
  ztpd2210->devAddress = devAddress;
}

double ZTPD2210getObjectTemperature(ZTPD2210_t *ztpd2210)
{
  ztpd2210->fullDataTemperature = ZTPD2210readRegister(ztpd2210, ZTPD2210_TEMPERATURE_REGISTER);
  uint32_t rawObjectTemperature = (ztpd2210->fullDataTemperature >> 24) & 0xFFFFFF;
  double objectTemperature = (double)rawObjectTemperature/pow(2,24) * 130.0 - 20.0;

  return objectTemperature;
}

double ZTPD2210getAmbientTemperature(ZTPD2210_t *ztpd2210)
{
  uint32_t rawAmbientTemperature = (ztpd2210->fullDataTemperature) & 0xFFFFFF;
  double ambientTemperature = (double)rawAmbientTemperature/pow(2,24) * 105.0 - 20.0;
  
  return ambientTemperature;
}
