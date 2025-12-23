#include "main.h"
#include "math.h"
#include "LTR390UV.h"

#define MSB(u16) ((u16 >> 8) & 0xFFU)
#define LSB(u16) ((u16 >> 0) & 0xFFU)
#define maxRegAddress 0x26

volatile uint8_t response;
volatile uint8_t data;

const uint8_t LTR390UVRegSize[maxRegAddress + 1] = {
  1,0,0,0,1,1,1,1, // 00h - 07h
  0,0,0,0,0,3,0,0, // 08h - 0Fh
  3,0,0,0,0,0,0,0, // 10h - 17h
  0,1,1,0,0,0,0,0, // 18h - 1Fh
  0,3,0,0,3,0,0    // 20h - 26h
};

/**
  * @brief  Write an amount of data to the sensor in blocking mode to a specific memory address
  * @param  LTR390UV Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddres
  * @param  MemAddress  sensor's internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  value Data to be sent
  * @retval HAL_status
  */
HAL_StatusTypeDef LTR390UV_WriteRegister(LTR390UV_HandleTypeDef *LTR390UV, uint8_t registerAddress, uint8_t value)
{
  uint8_t address[1];
  address[0] = value;

  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(LTR390UV->hi2c, (LTR390UV->devAddress) << 1, LTR390UV_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(LTR390UV->hi2c, (LTR390UV->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
    return HAL_OK;
  }
  else
	  return HAL_ERROR;
}

/**
  * @brief  Read an amount of data from the sensor in blocking mode from a specific memory address
  * @param  LTR390UV Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval dynamic data read from register's device depending on the register length.
  * 		Min. 8-bit, Max. 16-bit
  */
uint32_t LTR390UV_ReadRegister(LTR390UV_HandleTypeDef *LTR390UV, uint8_t registerAddress)
{
	uint32_t value = 0;
	uint8_t isDeviceReady;
	uint8_t i;

	uint8_t registerResponse[3] = {0}; // max buf size
	isDeviceReady = HAL_I2C_IsDeviceReady(LTR390UV->hi2c, (LTR390UV->devAddress) << 1, LTR390UV_TRIALS, HAL_MAX_DELAY);

	  if (isDeviceReady == HAL_OK)
	  {
		HAL_I2C_Mem_Read(LTR390UV->hi2c, (LTR390UV->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, LTR390UVRegSize[registerAddress], HAL_MAX_DELAY);
	    for (i = 0; i < LTR390UVRegSize[registerAddress]; i++)
	    {
	    	value = (value << 8) | registerResponse[i];
	    }

	    return value;
	  }
	  else
		  return HAL_ERROR;
}

/**
  * @brief  The current Main Control value is obtained by reading the corresponding
  * 				register from the sensor
  * @param  LTR390UV Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval 8-bit unsigned integer:
  * (+) [7:5] = RESERVED
  * (+) [4] = SW_RESET
  * (+) [3] = UVS_MODE
  * (+) [2] = RESERVED
  * (+) [1] = ALS/UV ENABLE
  * (+) [0] = RESERVED
  */
uint8_t LTR390UV_GetMainCtrl(LTR390UV_HandleTypeDef *ltr390uv)
{
	data = LTR390UV_ReadRegister(ltr390uv, LTR390UV_MAIN_CONTROL_REG);
	return data;
}

uint8_t LTR390UV_GetDeviceId(LTR390UV_HandleTypeDef *ltr390uv)
{
	data = LTR390UV_ReadRegister(ltr390uv, LTR390UV_DEVICE_ID_REG);
	return data;
}

/**
  * @brief  Read the ALS_UVS_MEAS_RATE Register from the sensor.
  * @param  LTR390UV Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @retval 8-bit unsigned integer:
  * (+) [7] = RESERVED. Read '0'
  * (+) [6:4] =ALS/UVS Resolution
  * (+) [3] = RESERVED
  * (+) [2:0] = ALS/UVS NEASUREMENT RATE
  */
uint8_t LTR390UV_GetRateResolution(LTR390UV_HandleTypeDef *ltr390uv)
{
	data = LTR390UV_ReadRegister(ltr390uv, LTR390UV_ALS_UV_MEAS_RATE_REG);
	return data;
}

uint8_t LTR390UV_GetGain(LTR390UV_HandleTypeDef * ltr390uv)
{
	data = LTR390UV_ReadRegister(ltr390uv, LTR390UV_ALS_UVS_GAIN_REG);
	return data;

}

/**
  * @brief  Allows the user to alternate between Ambient Light and Ultraviolet Mode
  * @param  ltr390uv Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  uvsMode:
  * (+) LTR390UV_ALS_MODE
  * (+) LTR390UV_UVS_MODE
  * @retval none
  */
void LTR390UV_SetMode(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_UVS_Mode_HandleTypeDef uvsMode)
{
	response = LTR390UV_GetMainCtrl(ltr390uv);
	response = (response & LTR390UV_UVS_MODE_MASK) | uvsMode;
	LTR390UV_WriteRegister(ltr390uv, LTR390UV_MAIN_CONTROL_REG, response);
}

/**
  * @brief  Allows the user to to enable the ALS/UVS or put the sensor in standby mode
  * @param  ltr390uv Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  enable:
  * (+) LTR390UV_ALS_UVS_STANDBY
  * (+) LTR390UV_ALS_UVS_ACTIVE
  * @retval none
  */
void LTR390UV_SetAlsUvsEnable(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Enable_HandleTypeDef enable)
{
	response = LTR390UV_GetMainCtrl(ltr390uv);
	response = (response & LTR390UV_UVS_ENABLE_MASK) | enable;
	LTR390UV_WriteRegister(ltr390uv, LTR390UV_MAIN_CONTROL_REG, response);
}



/**
  * @brief  Allows the user to select the sensor resolution
  * @param  ltr390uv Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  resolution:
  * (+) LTR390UV_RES_20BIT
  * (+) LTR390UV_RES_19BIT
  * (+) LTR390UV_RES_18BIT
  * (+) LTR390UV_RES_17BIT
  * (+) LTR390UV_RES_16BIT
  * (+) LTR390UV_RES_13BIT
  * @retval none
  */
void LTR390UV_SetResolution(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Resolution_HandleTypeDef resolution)
{
	response = LTR390UV_GetRateResolution(ltr390uv);
	response = (response & LTR390UV_RESOLUTION_MASK) | resolution;
	LTR390UV_WriteRegister(ltr390uv, LTR390UV_ALS_UV_MEAS_RATE_REG, response);

}

double LTR390UV_GetAlsData(LTR390UV_HandleTypeDef *ltr390uv)
{
	uint32_t value = LTR390UV_ReadRegister(ltr390uv, LTR390UV_ALS_DATA_REG);
	double lux;
	uint8_t alsData[3];
	alsData[0] = (value & 0xFF0000) >> 16;
	alsData[1] = (value & 0x00FF00) >>  8;
	alsData[2] = (value & 0x0000FF) >>  0;

	value = alsData[0] + (alsData[1] << 8) + (alsData[2] << 16);
	lux = (0.6 * value) / ((ltr390uv->gain) * 4);
	return lux;
}

/**
  * @brief  Allows the user to select the sensor gain
  * @param  ltr390uv Pointer to a LTR390UV_HandleTypeDef structure that contains
  *                	the configuration information for connecting to the sensor.
  * @param  gain:
  * (+) LTR390UV_GAIN_RANGE_1
  * (+) LTR390UV_GAIN_RANGE_3
  * (+) LTR390UV_GAIN_RANGE_6
  * (+) LTR390UV_GAIN_RANGE_9
  * (+) LTR390UV_GAIN_RANGE_18
  * @retval none
  */
void LTR390UV_SetGain(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Gain_HandleTypeDef gain)
{
	response = LTR390UV_GetGain(ltr390uv);
	response = (response & LTR390UV_GAIN_MASK) | gain;
	LTR390UV_WriteRegister(ltr390uv, LTR390UV_ALS_UVS_GAIN_REG, response);

	if (response == 0){
		ltr390uv->gain = 1;
	}
	else if (response == 1){
		ltr390uv->gain = 3;
	}
	else if (response == 2){
		ltr390uv->gain = 6;
	}
	else if (response == 3){
		ltr390uv->gain = 9;
	}
	else if (response == 4){
		ltr390uv->gain = 18;
	}

}

/**
  * @brief  Create a new instance of the MCP9808 temperature sensor setting the I²C port and slave address
  * @param  mcp9808 Pointer to a MCP9808_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval none
  */
void LTR390UV_Init(LTR390UV_HandleTypeDef *ltr390uv, I2C_HandleTypeDef *i2c)
{
	ltr390uv->hi2c = i2c;
	ltr390uv->devAddress = LTR390UV_ADDRESS;

	LTR390UV_SetAlsUvsEnable(ltr390uv, LTR390UV_ALS_UVS_ACTIVE);
	LTR390UV_SetGain(ltr390uv, LTR390UV_GAIN_RANGE_9);
}
