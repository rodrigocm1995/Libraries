#include "main.h"
#include "math.h"
#include "MCP9808.h"

#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

volatile uint16_t response = 0;

void MCP9808_WriteRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
  }
}

void MCP9808_WriteRegister8(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress, uint8_t value)
{
	  uint8_t address[1];
	  address[0] = value;
	  uint8_t isDeviceReady;

	  isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
	    HAL_I2C_Mem_Write(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
	  }
}


/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t MCP9808_ReadRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

uint8_t MCP9808_ReadRegister8(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress)
{
	  uint8_t registerResponse[1] = {0};
	  uint8_t isDeviceReady;

	  isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
	    HAL_I2C_Mem_Read(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
	  }

	  return registerResponse[0];
}


uint16_t MCP9808_GetConfig(MCP9808_HandleTypeDef *mcp9808)
{
  uint16_t data = 0;
  data = MCP9808_ReadRegister(mcp9808, MCP9808_CONFIG_REG);

  return data;
}

void MCP9808_SetHysteresis(MCP9808_HandleTypeDef *mcp9808, MCP9808_Hysteresis_t hyst)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_THYST_MASK) | hyst;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response);
}

void MCP9808_SetPower(MCP9808_HandleTypeDef *mcp9808, Mcp9808_shutdown power)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_SHDN_MASK) | power;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response);
}

void MCP9808_SetLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_crit_lock lock)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_CRIT_LOCK_MASK) | lock;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response);
}

void MCP9808_SetWinLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_window_lock windowLock)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_CRIT_LOCK_MASK) | windowLock;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response);
}

uint16_t MCP9808_GetDeviceId(MCP9808_HandleTypeDef *mcp9808)
{
  uint16_t data = 0;
  data = MCP9808_ReadRegister(mcp9808, MCP9808_MANUFACTURER_ID_REG);

  return data;
}

double MCP9808_GetTemperature(MCP9808_HandleTypeDef *mcp980)
{
	uint16_t data = 0;
	uint8_t upperByte = 0;
	uint8_t lowerByte = 0;
	double temp = 0;
	data = MCP9808_ReadRegister(mcp980, MCP9808_TEMPERATURE_REG);

	//upperByte = MSB(data) & 0x1F;
	//lowerByte = LSB(data);

	//temp = ((upperByte >> 4) + (lowerByte << 4)) * 0.0625;
	data = data & 0x1FFF;
	temp = data * 0.0625;
	return temp;
}

uint8_t MCP9808_GetResolution(MCP9808_HandleTypeDef *mcp9808)
{
	uint8_t data = 0;
	data = MCP9808_ReadRegister8(mcp9808, MCP9808_RESOLUTION_REG);
	return data;
}

void MCP9808_SetResolution(MCP9808_HandleTypeDef *mcp9808, MCP9808_Resolution_t resolution)
{
	MCP9808_WriteRegister8(mcp9808, MCP9808_RESOLUTION_REG, resolution);
}

/**
  * @brief  Create a new instance of the mcp9808 temperature sensor setting the I²C port and slave address
  *         Set the Configuration Register with default value 0220h.
  * @param  mcp9808 points to an object of type MCP9808_HandleTypeDef
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval none
  */
void MCP9808_Init(MCP9808_HandleTypeDef *mcp9808, I2C_HandleTypeDef *i2c)
{
  mcp9808->hi2c = i2c;
  mcp9808->devAddress = MCP9808_ADDRESS;
  MCP9808_SetResolution(mcp9808, MCP9808_0_0625C_RES);
}
