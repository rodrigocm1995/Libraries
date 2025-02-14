#include "main.h"
#include "math.h"
#include "INA238.h"

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

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  ina238 Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  value The data that must be written in the register
  */
void ina238writeRegister(Ina238_t *ina238, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(ina238->hi2c, (ina238->devAddress) << 1, INA238_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(ina238->hi2c, (ina238->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t ina238readRegister(Ina238_t *ina238, uint8_t registerAddress)
{
  uint8_t registerResponse[3]; //max buffer size
  uint8_t isDeviceReady;
  uint32_t value = 0;

  isDeviceReady = HAL_I2C_IsDeviceReady(ina238->hi2c, (ina238->devAddress) << 1, INA238_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(ina238->hi2c, (ina238->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, INA238_regSize[registerAddress], sizeof(registerResponse), HAL_MAX_DELAY);
  }

  for (uint8_t i = 1; INA238_regSize[regAddr]+1; i++)
  {
    value = (value << 8) | registerResponse[i];
  }

  return value;
}
