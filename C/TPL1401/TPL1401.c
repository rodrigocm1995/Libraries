#include "main.h"
#include "math.h"
#include "TPL1401.h"

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  TPL1401 Pointer to a TPL1401_t structure that contains the configura-
  *                 tion information for the specified TPL1401 device.
  * @param  registerAddress Internal 8-bits memory address 
  * @param  value The data that must be written into the register.
  */
 HAL_StatusTypeDef tpl1401WriteRegister(TPL1401_t * TPL1401, uint8_t registerAddress, uint16_t value)
 {
  uint8_t addressArray[2];
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef memStatus;

  addressArray[0] = (value >> 8) & 0xFF;
  addressArray[1] = (value >> 0) & 0xFF;

  status = HAL_I2C_IsDeviceReady(TPL1401->hi2c, (TPL1401->devAddress) << 1, TPL1401_TRIALS, HAL_MAX_DELAY);

  if (status == HAL_OK)
  {
    memStatus = HAL_I2C_Mem_Write(TPL1401->hi2c, (TPL1401->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)addressArray, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);

    if (memStatus == HAL_OK)
    {
      return HAL_OK;
    }
    return memStatus;
  }
  return HAL_ERROR;
 }

 /**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  TPL1401 Pointer to a TPL1401_t structure that contains the configura-
  *                 tion information for the specified TPL1401 device.
  * @param  registerAddress Internal 8-bits memory address 
  */
uint16_t tpl1401ReadRegister(TPL1401_t * TPL1401, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef memStatus;

  status = HAL_I2C_IsDeviceReady(TPL1401->hi2c, (TPL1401->devAddress) << 1, TPL1401_TRIALS, HAL_MAX_DELAY);

  if (status == HAL_OK)
  {
    memStatus = HAL_I2C_Mem_Read(TPL1401->hi2c, (TPL1401->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);

    if (memStatus == HAL_OK)
    {
      return ((registerResponse[0] << 8) | registerResponse[1]);
    }
    return memStatus;
  }

  return HAL_ERROR;

}

/**
  * @brief  Initializes the object and write default values into config register
  * @param  TPL1401 Pointer to a TPL1401_t structure that contains the configura-
  *                 tion information for the specified TPL1401 device.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains the confi-
  *             guration information for the specified I2C.
  * @param  devAddress Target device address
  */
int8_t tpl1401DefaultInit(TPL1401_t * TPL1401, I2C_HandleTypeDef * i2c, uint8_t devAddress)
{
  TPL1401->hi2c = i2c;
  TPL1401->devAddress = devAddress;
  HAL_StatusTypeDef status;
  uint16_t configValue;

  configValue = !TPL1401_DEVICE_LOCK_BIT | TPL1401_RESERVED_BITS | POWER_DOWN_HIGH_IMPEDANCE | !TPL1401_REF_ENABLED;
  status = writeRegister(TPL1401, TPL1401_GENERAL_CONFIG_REGISTER, configValue);

  if (status == HAL_OK)
  {
    return configValue;
  }
  return -1;
}

/**
  * @brief  Initializes the object and write in config register
  * @param  TPL1401 Pointer to a TPL1401_t structure that contains the configura-
  *                 tion information for the specified TPL1401 device.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains the confi-
  *             guration information for the specified I2C.
  * @param  devAddress Target device address
  * @param  deviceLock locks all the registers
  * @param  powerType Power up or Power Down the device
  * @param  internalRef Enable de internal 1.21V reference
  * @param  span Set a voltage gain only if internal reference is enabled
  */
HAL_StatusTypeDef tpl1401Init(TPL1401_t * TPL1401, I2C_HandleTypeDef * i2c, uint8_t devAddress, uint16_t deviceLock, DpotPdn_t powerType, uint16_t internalRef, OutSpan_t span)
{
  TPL1401->hi2c = i2c;
  TPL1401->devAddress = devAddress;
  HAL_StatusTypeDef status;
  uint16_t configValue;

  configValue = deviceLock | TPL1401_RESERVED_BITS | internalRef | span;
  status = writeRegister(TPL1401, TPL1401_GENERAL_CONFIG_REGISTER, configValue);

  return status;
}

uint16_t tpl1401ReadConfigRegister(TPL1401_t * TPL1401)
{
  uint16_t value;
  value = readRegister(TPL1401, TPL1401_GENERAL_CONFIG_REGISTER);
  
  return value;
}



