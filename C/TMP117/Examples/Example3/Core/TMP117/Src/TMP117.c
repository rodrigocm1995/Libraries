#include "main.h"
#include "math.h" // Functions that operate on floating point numbers are in math.h
#include "stdlib.h" // Functions that operate on integers are in stdlib.h
#include "TMP117.h"

volatile uint16_t value;
volatile _Bool valueFlag;


/**
  * @brief  Write an amount of data to the sensor in blocking mode to a specific memory address
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddres
  * @param  MemAddress  sensor's internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  value Data to be sent
  * @retval HAL_status
  */
HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  uint8_t isDeviceReady;

  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  isDeviceReady = HAL_I2C_IsDeviceReady(tmp117->hi2c, (tmp117->devAddress) << 1, TMP117_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Write(tmp117->hi2c, (tmp117->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY);
    return HAL_OK;
  }
  return HAL_ERROR;
}

/**
  * @brief  Read an amount of data from the sensor in blocking mode from a specific memory address
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval unsigned int 16-bit value of the register
  */
uint16_t TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress)
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
  *@brief Get the current value of the TEMP_RESULT register.
  *			This register contains the result of the most recent temperature.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	signed integer 16-bit data
*/
int16_t TMP117_GetTemperature(TMP117_HandleTypeDef *tmp117)
{
	int16_t data = TMP117_ReadRegister(tmp117, TMP117_TEMP_RESULT_REG);
	return data;
}

/**
  *@brief Get the current value of the CONFIGURATION register.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);
	return data;
}

uint16_t TMP117_GetTempHighLimit(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG);
	return data;
}

uint16_t TMP117_GetTempLowLimit(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG);
	return data;
}

uint16_t TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_DEVICE_ID_REG);
	return data;
}

_Bool TMP117_HighAlertFlag(TMP117_HandleTypeDef *tmp117)
{
	value = TMP117_GetConfiguration(tmp117);
	valueFlag = CHECK_BIT(value, 15);
	return valueFlag;
}

_Bool TMP117_LowAlertFlag(TMP117_HandleTypeDef *tmp117)
{
	value = TMP117_GetConfiguration(tmp117);
	valueFlag = CHECK_BIT(value, 14);
	return valueFlag;
}

_Bool TMP117_DataReadyFlag(TMP117_HandleTypeDef *tmp117)
{
	value = TMP117_GetConfiguration(tmp117);
	valueFlag = CHECK_BIT(value, 13);
	return valueFlag;
}

_Bool TMP117_EepromBusyFlag(TMP117_HandleTypeDef *tmp117)
{
	value = TMP117_GetConfiguration(tmp117);
	valueFlag = CHECK_BIT(value, 12);
	return valueFlag;
}

/**
  *@brief Allows the user to set conversion mode of the CONFIGURATION register.
  * The TMP117 can be configured to operate in various modes by using the MOD[1:0] bits.
  * These modes provide flexibility to operate the device in the most power efficient
  * way necessary for the intended application.
  *@param opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param mode:
  *(+) TMP117_CONTINUOUS_MODE: The device continuously performs temperature conversions.
  *		Each conversion consists of an active conversion period followed by an standby
  *		period. The duration of the active conversion period and standby period can be configured
  *		using the CONV[2:0] and AVG[1:0] bits in the CONFIGURATION register.
  *(+) TMP117_SHUTDOWN_MODE: When the MOD[1:0] bits are set to 01 in the CONFIGURATION register,
  *		the device instantly aborts the currently running conversion and enters a low-power
  *		shutdown mode.
  *(+) TMP117_ONE_SHOT_MODE: The TMP117 will run a temperature conversion referred to as a one-
  *		shot conversion. After the device completes a one-shot conversion, the device goes to
  *		the low-power shutdown mode.
  *@retval	none
*/
void TMP117_SetMode(TMP117_HandleTypeDef *tmp117, TMP117_Mode_HandleTypeDef mode)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_MODE_MASK) | mode;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  *@brief Allows the user to set the conversion cycle bit of the CONFIGURATION register.
  *			See the following table for the standby time between conversions
  *			This register controls the major operational modes of the device.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param conv: conversion cycle to be sent
  *(+) TMP117_CONVERSION_TYPE_0
  *(+) TMP117_CONVERSION_TYPE_1
  *(+) TMP117_CONVERSION_TYPE_2
  *(+) TMP117_CONVERSION_TYPE_3
  *(+) TMP117_CONVERSION_TYPE_4
  *(+) TMP117_CONVERSION_TYPE_5
  *(+) TMP117_CONVERSION_TYPE_6
  *(+) TMP117_CONVERSION_TYPE_7
  *	+-----------+------------------------+-----------------------+------------------------+------------------------+
  *	| CONV[2:0] | AVG[1:0] = 00 (No avg) | AVG[1:0] = 01 (8 Avg) | AVG[1:0] = 10 (32 Avg) | AVG[1:0] = 11 (64 Avg) |
  *	+-----------+------------------------+-----------------------+------------------------+------------------------+
  *	|       000 | 15.5ms                 | 125 ms                | 500 ms                 | 1 s                    |
  *	|       001 | 125 ms                 | 125 ms                | 500 ms                 | 1 s                    |
  *	|       010 | 250 ms                 | 250 ms                | 500 ms                 | 1 s                    |
  *	|       011 | 500 ms                 | 500 ms                | 500 ms                 | 1 s                    |
  *	|       100 | 1 s                    | 1 s                   | 1 s                    | 1 s                    |
  *	|       101 | 4 s                    | 4 s                   | 4 s                    | 4 s                    |
  *	|       110 | 8 s                    | 8 s                   | 8 s                    | 8 s                    |
  *	|       111 | 16 s                   | 16 s                  | 16 s                   | 16 s                   |
  *	+-----------+------------------------+-----------------------+------------------------+------------------------+
  *@retval	none
*/
void TMP117_SetConversionCycle(TMP117_HandleTypeDef *tmp117, TMP117_ConversionCycle_HandleTypeDef conv)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_CONV_CYCLE_MASK) | conv;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  *@brief Allows the user to set conversion averaging mode bits of the CONFIGURATION register.
  * Determines the number of conversion results that are collected and averaged before updating the
  * temperature register. The average is an accumulated average and not a running average.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param avg:
  *(+) TMP117_NO_AVERAGING
  *(+) TMP117_AVG_8_CONV
  *(+) TMP117_AVG_32_CONV
  *(+) TMP117_AVG_64_CONV
  *@retval	none
*/
void TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Average_HandleTypeDef avg)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_AVERAGE_MASK) | avg;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  *@brief Allows the user to set the therm/alert mode bit of the CONFIGURATION register.
  * The built-in therm and alert functions of the TMP117 can alert the user if the temperature
  * has crossed a certain temperature limit or if the device is within a certain temperature range.
  * At the end of every conversion, including averaging, the device compares the converted
  * temperature result to the values stored in the LOW LIMIT register and HIGH LIMIT register. The
  * device then either sets or clears the corresponding status flags in the CONFIGURATION register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param tnA:
  *(+) TMP117_THERM_MODE: The device compares the conversion result at the end of every conversion
  *		with the values in the LOW LIMIT register and HIGH LIMIT register and sets the HIGH_Alert
  *		status flag in the CONFIGURATION register if the temperature exceeds the value in the HIGH
  *		LIMIT register. When set, the device clears the HIGH_Alerts status flag if the conversion
  *		result goes below the value in the low limit register.
  *		In this mode, the device asserts the ALERT pin if the HIGH_Alert status flag is set and deasserts
  *		the ALERT pin when the HIGH_Alert status flag is cleared.
  *(+) TMP117_ALERT_MODE: The device compares the conversion result at the end of every conversion
  *		with the values in the LOW LIMIT register and HIGH LIMIT register. If the temperature result
  *		exceeds the value in the HIGH limit register, the HIGH_Alert status flag in the CONFIGURATION
  *		register is set. On the other hand, if the temperature result is lower than the value in the
  *		LOW LIMIR register, the LOW_Alert status flag in the CONFIGURATION register is set.
  *		The device asserts the ALERT pin in this mode when either the HIGH_Alert or the LOW_Alert
  *		status flag is set.
  *@retval	none
*/
void TMP117_SetThermAlertMode(TMP117_HandleTypeDef *tmp117, TMP117_ThermAlertMode_HandleTypeDef tnA)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_AVERAGE_MASK) | tnA;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  *@brief Allows the user to set the ALERT pin polarity bit of the CONFIGURATION register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param polarity:
  *(+) TMP117_ALERT_ACTIVE_HIGH
  *(+) TMP117_ALERT_ACTIVE_LOW
  *@retval	none
*/
void TMP117_SetAlertPinPolarity(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinPolarity_HandleTypeDef polarity)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_ALERT_PIN_POL_MASK) | polarity;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  *@brief Allows the user to set the ALERT pin select bit of the CONFIGURATION register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@param select:
  *(+) TMP117_ALERT_FOR_DATA_READY_FLAG: ALERT pin reflects the status of the data ready flag
  *(+) TMP117_ALERT_FOR_ALERT_FLAGS: ALERT pin reflects the status of the alert flags
  *@retval	none
*/
void TMP117_SetAlertPinSelect(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinSelect_HandleTypeDef select)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_ALERT_PIN_SELECT_MASK) | select;
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

void TMP117_SetSoftReset(TMP117_HandleTypeDef *tmp117)
{
	uint16_t result = TMP117_GetConfiguration(tmp117);
	result = (result & TMP117_SOFT_RESET_MASK) | 0x0002;
}

/**
  * @brief  Set the High Limit Temperature (in °C) in the HIGH LIMIT register
  *         This register is a 16-bit register that stores the low limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  * @param  highLimit The higher desired temperature value, positive or negative
  * @retval none
  */
void TMP117_SetHighLimitTemp(TMP117_HandleTypeDef *tmp117, uint8_t highLimit)
{
	uint16_t hLimit = abs(highLimit) / (7.8125 * pow(10, -3));
	if (highLimit < 0)
	{
		hLimit = ~hLimit + 1;
	}
	TMP117_WriteRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, hLimit);
}

/**
  * @brief  Set the Low Limit Temperature (in °C) in the LOW LIMIT register
  *         This register is a 16-bit register that stores the low limit for
  *         comparison with the temperature result. One LSB equals to 7.8125 m°C.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  * @param  lowLimit The lower desired temperature value, positive or negative
  * @retval none
  */
void TMP117_SetLowLimitTemp(TMP117_HandleTypeDef *tmp117, uint8_t lowLimit)
{
	uint16_t lLimit = abs(lowLimit) / (7.8125 * pow(10, -3));
	if (lowLimit < 0)
	{
		lLimit = ~lLimit + 1;
	}
	TMP117_WriteRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, lLimit);
}

/**
  * @brief  Verify if the MSB in target register is set to 1.
  *         If bit is set to 1 then makes the two's complement
  * @param  value: Its is the value obtained from target register
  * @retval positive or negative temperature
  */
double TMP117_CheckTemperature(uint16_t value)
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

double TMP117_GetTemperatureCelsius(TMP117_HandleTypeDef *tmp117)
{
	  int16_t rawTemp;
	  double temperature;

	  rawTemp = TMP117_GetTemperature(tmp117);
	  temperature =  TMP117_CheckTemperature(rawTemp);
	  return temperature;
}

/**
  * @brief  Create a new instance of the tmp117 temperature sensor setting the I²C port and slave address
  *         Set the Configuration Register with default value 0220h.
  * @param  tmp117 points to an object of type Tmp117_t 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval none
  */
void TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *i2c)
{
  tmp117->hi2c = i2c;
  tmp117->devAddress = TMP117_ADDRESS;
  
  TMP117_SetMode(tmp117, TMP117_CONTINUOUS_MODE);
  TMP117_SetAverage(tmp117, TMP117_AVG_32_CONV);
  TMP117_SetConversionCycle(tmp117, TMP117_CONVERSION_TYPE_6);
  TMP117_SetThermAlertMode(tmp117, TMP117_ALERT_MODE);
  TMP117_SetAlertPinPolarity(tmp117, TMP117_ALERT_ACTIVE_HIGH);
  TMP117_SetAlertPinSelect(tmp117, TMP117_ALERT_FOR_DATA_READY_FLAG);
}


