#include "main.h"
#include "math.h"
#include "MCP9808.h"

#define MSB(u16) ((u16 >> 8) & 0xFFU)
#define LSB(u16) ((u16 >> 0) & 0xFFU)
#define maxRegAddress 0x08

volatile uint16_t response;

const uint8_t MCP9808RegSize[maxRegAddress + 1] = {0,2,2,2,2,2,2,2,1};

/**
  * @brief  Write an amount of data to the sensor in blocking mode to a specific memory address
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddres
  * @param  MemAddress  sensor's internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  value Data to be sent
  * @retval HAL_status
  */
HAL_StatusTypeDef MCP9808_WriteRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress, uint16_t value, MCP9808_AddressSize_t size)
{
  uint8_t address[2]; //max buf size
  if(size == MCP9808_MEMADD_SIZE_8BIT)
  {
	  address[0] = (uint8_t)(value);
  }
  else
  {
	  address[0] = MSB(value);
	  address[1] = LSB(value);
  }
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, MCP9808RegSize[registerAddress], HAL_MAX_DELAY);
    return HAL_OK;
  }
  else
	  return HAL_ERROR;
}

/**
  * @brief  Read an amount of data from the sensor in blocking mode from a specific memory address
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval dynamic data read from register's device depending on the register length.
  * 		Min. 8-bit, Max. 16-bit
  */
uint16_t MCP9808_ReadRegister(MCP9808_HandleTypeDef *mcp9808, uint8_t registerAddress)
{
	uint16_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[2] = {0}; // max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(mcp9808->hi2c, (mcp9808->devAddress) << 1, MCP9808_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
		HAL_I2C_Mem_Read(mcp9808->hi2c, (mcp9808->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, MCP9808RegSize[registerAddress], HAL_MAX_DELAY);
	    for (i = 0; i < MCP9808RegSize[registerAddress]; i++)
	    {
	    	value = (value << 8) | registerResponse[i];
	    }

	    return value;
	  }
	  else
		  return HAL_ERROR;
}

/**
  * @brief  The current Configuration Register value is obtained by reading the corresponding
  * 				register from the sensor
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval 16-bit unsigned integer:
  * (+) [15:10] = READ AS '0'
  * (+) [10:9] = T_HYST
  * (+) [8] = SHDN
  * (+) [7] = CRIT LOCK
  * (+) [6] = WIN LOCK
  * (+) [5] = INT CLEAR
  * (+) [4] = ALERT STAT
  * (+) [3] = ALERT CNT
  * (+) [2] = ALERT SEL
  * (+) [1] = ALERT POL
  * (+) [0] = ALERT MOD
  */
uint16_t MCP9808_GetConfig(MCP9808_HandleTypeDef *mcp9808)
{
  uint16_t data = 0;
  data = MCP9808_ReadRegister(mcp9808, MCP9808_CONFIG_REG);

  return data;
}

/**
  * @brief  Allows the user to change the T_UPPER and T_LOWER Limit Hysteresis bits [10:9]
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  hyst:
  * (+) MCP9808_HYST_0_CELSIUS
  * (+) MCP9808_HYST_1_5_CELSIUS
  * (+) MCP9808_HYST_3_0_CELSIUS
  * (+) MCP9808_HYST_6_0_CELSIUS
  * @retval none
  */
void MCP9808_SetHysteresis(MCP9808_HandleTypeDef *mcp9808, MCP9808_Hysteresis_t hyst)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_THYST_MASK) | hyst;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  Allows the user to change the Shutdown bit [8] inside the Configuration Register
  * 		In shutdown, all power-consuming activities are disable, though all register can
  * 		be written to or read.
  * 		This bit cannot be set to '1' when either of the Lock bits is set (bit 6 and bit 7).
  * 		However, it can be cleared to '0' for continuous conversion while locked.
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  windowLock:
  * (+) MCP9808_CRIT_UNLOCKED
  * (+) MCP9808_CRIT_LOCKED
  * @retval none
  */
void MCP9808_SetPower(MCP9808_HandleTypeDef *mcp9808, Mcp9808_shutdown power)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_SHDN_MASK) | power;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  Allows the user to change the Critial Lock bit [7] inside the Configuration Register
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  windowLock:
  * (+) MCP9808_CRIT_UNLOCKED
  * (+) MCP9808_CRIT_LOCKED
  * @retval none
  */
void MCP9808_SetCriticalLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_crit_lock lock)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_CRIT_LOCK_MASK) | lock;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  Allows the user to change the Windows Lock bit [6] inside the Configuration Register
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  windowLock:
  * (+) MCP9808_WIN_UNLOCKED
  * (+) MCP9808_WIN_LOCKED
  * @retval none
  */
void MCP9808_SetWinLock(MCP9808_HandleTypeDef *mcp9808, Mcp9808_window_lock windowLock)
{
  response = MCP9808_GetConfig(mcp9808);
  response = (response & MCP9808_CRIT_LOCK_MASK) | windowLock;
  MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  The current Manufacturer ID value is obtained by reading the corresponding
  * 				register from the sensor
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval unsigned integer data - 0054h
  */
uint16_t MCP9808_GetManufacturerId(MCP9808_HandleTypeDef *mcp9808)
{
  uint16_t data = 0;
  data = MCP9808_ReadRegister(mcp9808, MCP9808_MANUFACTURER_ID_REG);

  return data;
}

/**
  * @brief  The current device ID value is obtained by reading the corresponding
  * 				register from the sensor
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval unsigned integer data - 0400h
  */
uint16_t MCP9808_GetDeviceId(MCP9808_HandleTypeDef *mcp9808)
{
	uint16_t data = 0;
	data = MCP9808_ReadRegister(mcp9808, MCP9808_DEVICE_ID_REG);
	return data;
}

/**
  * @brief  Read the temperature value in degrees Celsius
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval double temperature value
  */
double MCP9808_GetTemperature(MCP9808_HandleTypeDef *mcp980)
{
	uint16_t data = 0;
	//uint8_t upperByte = 0;
	//uint8_t lowerByte = 0;
	double temp = 0;
	data = MCP9808_ReadRegister(mcp980, MCP9808_TEMPERATURE_REG);

	//upperByte = MSB(data) & 0x1F;
	//lowerByte = LSB(data);
	//temp = ((upperByte >> 4) + (lowerByte << 4)) * 0.0625;
	data = data & 0x1FFF;
	temp = data * 0.0625;
	return temp;
}

/**
  * @brief  The current resolution value is obtained by reading the corresponding
  * 				register from the sensor
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval 8-bit unsigned integer data
  * (+) 00h = +0.5°C (t_CONV = 30 ms typical)
  * (+) 01h = +0.25°C (t_CONV = 65 ms typical)
  * (+) 02h = +0.125°C (t_CONV = 130 ms typical)
  * (+) 03h = +0.625°C (t_CONV = 250 ms typical)
  */
uint8_t MCP9808_GetResolution(MCP9808_HandleTypeDef *mcp9808)
{
	uint8_t data = 0;
	data = MCP9808_ReadRegister(mcp9808, MCP9808_RESOLUTION_REG);
	return data;
}

/**
  * @brief  Allows the user to change then sensor resolution
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  resolution:
  * (+) MCP9808_0_5C_RES
  * (+) MCP9808_0_25C_RES
  * (+) MCP9808_0_125C_RES
  * (+) MCP9808_0_0625C_RES
  * @retval none
  */
void MCP9808_SetResolution(MCP9808_HandleTypeDef *mcp9808, MCP9808_Resolution_t resolution)
{
	MCP9808_WriteRegister(mcp9808, MCP9808_RESOLUTION_REG, resolution, MCP9808_MEMADD_SIZE_8BIT);
}

/**
  * @brief  Allows the user to change then sensor Interrupt Clear bit
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  intClear:
  * (+) MCP9808_CLEAR_INT_NO_EFFECT
  * (+) MCP9808_CLEAR_INT_OUTPUT
  * @retval none
  */
void MCP9808_SetIntClear(MCP9808_HandleTypeDef *mcp9808, MCP9808_IntClear_t intClear)
{
	response = MCP9808_GetConfig(mcp9808);
	response = (response & MCP9808_INT_CLEAR_MASK) | intClear;
	MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  Allows the user to change the sensor Alert Output Control bit [3]
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  alertCtrl:
  * (+) MCP9808_ALERT_CTRL_DISABLED
  * (+) MCP9808_ALERT_CTRL_ENABLED
  * @retval none
  */
void MCP9808_SetAlertControl(MCP9808_HandleTypeDef *mcp9808, MCP9808_AlertCtrl_t alertCtrl)
{
	response = MCP9808_GetConfig(mcp9808);
	response = (response & MCP9808_ALERT_CTRL_MASK) | alertCtrl;
	MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}

/**
  * @brief  Allows the user to change the Alert Polarity bit [1]
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  alertPolarity:
  * (+) MCP9808_ALERT_ACTIVE_LOW
  * (+) MCP9808_ALERT_ACTIVE_HIGH
  * @retval none
  */
void MCP9808_SetAlertPolarity(MCP9808_HandleTypeDef *mcp9808, MCP9808_AlertPolarity_t alertPolarity)
{
	response = MCP9808_GetConfig(mcp9808);
	response = (response & MCP9808_ALERT_POLARITY_MASK) | alertPolarity;
	MCP9808_WriteRegister(mcp9808, MCP9808_CONFIG_REG, response, MCP9808_MEMADD_SIZE_16BIT);
}


/**
  * @brief  Create a new instance of the MCP9808 temperature sensor setting the I²C port and slave address
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
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
