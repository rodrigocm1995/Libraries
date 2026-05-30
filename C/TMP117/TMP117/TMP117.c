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
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);
	return data;
}

/**
  *@brief Get the current value of the HIGH LIMIT register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetTempHighLimit(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG);
	return data;
}

/**
  *@brief Get the current value of the LOW LIMIT register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetTempLowLimit(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG);
	return data;
}

/**
  *@brief Get the manufacturer ID. This register is intended to help uniquely identify
  *			the device.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_DEVICE_ID_REG);
	return data;
}

uint16_t TMP117_GetEepromUnlock(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_EEPROM_UNLOCK_REG);
	return data;
}

/**
  *@brief Get the current value of the EEPROM1 register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetEeprom1(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_EEPROM1_REG);
	return data;
}

/**
  *@brief Get the current value of the EEPROM2 register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetEeprom2(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_EEPROM2_REG);
	return data;
}

/**
  *@brief Get the current value of the EEPROM3 register.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	unsigned integer 16-bit data
*/
uint16_t TMP117_GetEeprom3(TMP117_HandleTypeDef *tmp117)
{
	uint16_t data = TMP117_ReadRegister(tmp117, TMP117_EEPROM3_REG);
	return data;
}


/**
  *@brief Read the EEPROM_Busy bit of the CONFIGURATION register.
  *The value of the flag indicates that the EEPROM is busy during programming or power-up.
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	Boolean data
*/
_Bool TMP117_EepromBusyFlag(TMP117_HandleTypeDef *tmp117)
{
	value = TMP117_GetConfiguration(tmp117);
	valueFlag = CHECK_BIT(value, 12);
	return valueFlag;
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
	TMP117_WriteRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, lLimit);
}

/**
  *@brief Get the high limit temperature value in degree celsius
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double temperature high limit value
*/
double TMP117_GetHighLimitTemp(TMP117_HandleTypeDef *tmp117)
{
	double temp = 0.0;
	int16_t rawTemp = TMP117_GetTempHighLimit(tmp117);
	temp = TMP117_CheckTemperature(rawTemp);
	return temp;
}


/**
  *@brief Get the low limit temperature value in degree celsius
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double temperature low limit value
*/
double TMP117_GetLowLimitTemp(TMP117_HandleTypeDef *tmp117)
{
	double temp = 0.0;
	int16_t rawTemp = TMP117_GetTempLowLimit(tmp117);
	temp = TMP117_CheckTemperature(rawTemp);
	return temp;
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


/**
  *@brief Get the current value of the temperature in degree celsius
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double temperature low limit value
*/
double TMP117_GetTemperatureCelsius(TMP117_HandleTypeDef *tmp117)
{
	  int16_t rawTemp;
	  double temperature;

	  rawTemp = TMP117_GetTemperature(tmp117);
	  temperature =  TMP117_CheckTemperature(rawTemp);
	  return temperature;
}

/**
  *@brief Allows the user to lock or unlock the EEPROM register
    * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  * @param  lockUnlock
  *(+) TMP117_LOCK_EEPROM: EEPROM is locked for programming, writes to all EEPROM addresses
  *		(such as configuration, limits, and EEPROM locations 1-4) are written to registers
  *		in digital logic and are not programmed in the EEPROM
  *(+) TMP117_UNLOCK_EEPROM: EEPROM unlocked for programming, any writes to programmable
  *		registers program the respective location in the EEPROM
  *@retval	none
*/
void TMP117_SetLockUnlockEeprom(TMP117_HandleTypeDef *tmp117, TMP117_LockUnlock_HandleTypeDef lockUnlock)
{
	uint16_t result = TMP117_GetEepromUnlock(tmp117);
	result = (result & TMP117_LOCK_UNLOCK_EEPROM_MASK) | lockUnlock;
	TMP117_WriteRegister(tmp117, TMP117_EEPROM_UNLOCK_REG, result);
}

HAL_StatusTypeDef TMP117_SetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
	value = TMP117_GetEepromUnlock(tmp117);
	valueFlag = CHECK_BIT(value,15);
	if (valueFlag)
	{
		TMP117_WriteRegister(tmp117, TMP117_EEPROM1_REG, data);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMP117_SetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
	value = TMP117_GetEepromUnlock(tmp117);
	valueFlag = CHECK_BIT(value,15);
	if (valueFlag)
	{
		TMP117_WriteRegister(tmp117, TMP117_EEPROM1_REG, data);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef TMP117_SetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
	value = TMP117_GetEepromUnlock(tmp117);
	valueFlag = CHECK_BIT(value,15);
	if (valueFlag)
	{
		TMP117_WriteRegister(tmp117, TMP117_EEPROM1_REG, data);
		return HAL_OK;
	}
	return HAL_ERROR;
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
  TMP117_SetConversionCycle(tmp117, TMP117_CONVERSION_TYPE_7);
  TMP117_SetThermAlertMode(tmp117, TMP117_ALERT_MODE);
  TMP117_SetAlertPinPolarity(tmp117, TMP117_ALERT_ACTIVE_HIGH);
  TMP117_SetAlertPinSelect(tmp117, TMP117_ALERT_FOR_DATA_READY_FLAG);
}


/**
  * @brief  Reset the TMP117 device registers to their default factory values
  * @note   This function writes the soft reset bit (bit 15) directly to the 
  *         Configuration Register (0x01). The RST bit is self-clearing once 
  *         the reset is executed in the silicon.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval None
  */
void TMP117_ResetDevice(TMP117_HandleTypeDef *tmp117)
{
    // Write the reset bit (bit 15 = 0x8000) directly to the configuration register.
    TMP117_WriteRegister(ina236, TMP117_CONFIGURATION_REG, TMP117_SOFTRESET);
    
    // Wait for a brief delay to ensure that the chip's internal circuits
    // and I2C communication have stabilized after the reset.
    HAL_Delay(2);
}

/**
  * @brief  Configure the ALERT pin function on the TMP117 sensor (Alert flag vs Data Ready flag)
  * @note   This function modifies the DR/Alert bit (bit 2) of the Configuration Register (0x01).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  pinFunction Selected alert pin function mode.
  *                     This parameter can be one of the following values:
  *                     @arg TMP117_ALERT_FOR_ALERT_FLAGS: ALERT pin functions as an alert flag for temperature limits.
  *                     @arg TMP117_ALERT_FOR_DATA_READY_FLAG: ALERT pin functions as a Data Ready flag.
  * @retval None
  */
void TMP117_SetAlertPinFunction(TMP117_HandleTypeDef *tmp117, TMP117_DRALERT_TypeDef pinFunction)
{
	uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the TMP117_DRALERT bit using the mask (bit 2)
    regValue &= ~TMP117_DRALERT_Mask;

    // Set the new DRALERT by shifting it to its position (DRALERT_Pos = 2)
    // and protecting it with the mask
    regValue |= (pinFunction << TMP117_DRALERT_Pos) & TMP117_DRALERT_Mask;

    //Write the resulting value back to the register
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, result);
}

/**
  * @brief  Configure the active polarity of the physical ALERT pin on the TMP117 sensor
  * @note   This function modifies the POL bit (bit 3) of the Configuration Register (0x01).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  polarity Selected alert pin polarity.
  *                  This parameter can be one of the following values:
  *                  @arg TMP117_ALERT_ACTIVE_HIGH: Alert active state is high (voltage is logic high)
  *                  @arg TMP117_ALERT_ACTIVE_LOW: Alert active state is low (voltage is logic low)
  * @retval None
  */
void TMP117_SetAlertPinPolarity(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinPol_TypeDef polarity)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the POL bit using the mask (bit 3)
    regValue &= ~TMP117_POL_Mask;

    // Set the new POL by shifting it to its position (POL_Pos = 3)
    // and protecting it with the mask
    regValue |= (polarity << TMP117_POL_Pos) & TMP117_POL_Mask;

    //Write the resulting value back to the register
    TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the Therm/Alert mode on the TMP117 sensor
  * @details The built-in therm and alert functions of the TMP117 alert the user if the temperature
  *          crosses a certain temperature limit or if the device is within a certain temperature range.
  *          At the end of every conversion (including averaging), the device compares the converted
  *          temperature result to the values stored in the LOW LIMIT and HIGH LIMIT registers, setting
  *          or clearing the corresponding status flags in the CONFIGURATION register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  tnA Selected mode of operation.
  *             This parameter can be one of the following values:
  *             @arg TMP117_THERM_MODE: Thermostat mode. The ALERT pin asserts when the temperature
  *                  exceeds the HIGH LIMIT and deasserts only when it falls below the LOW LIMIT.
  *             @arg TMP117_ALERT_MODE: Alert mode. The ALERT pin asserts when the temperature
  *                  exceeds the HIGH LIMIT or falls below the LOW LIMIT.
  * @retval None
  */
void TMP117_SetThermAlertMode(TMP117_HandleTypeDef *tmp117, TMP117_ThermAlertMode_TypeDef tnA)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the TnA bit using the mask (bit 4)
    regValue &= ~TMP117_TnA_Mask;

    // Set the new TnA by shifting it to its position (TnA_Pos = 4)
    // and protecting it with the mask
    regValue |= (polarity << TMP117_TnA_Pos) & TMP117_TnA_Mask;

    //Write the resulting value back to the register
    TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
}


/**
  * @brief  Configure the number of conversion averages for the TMP117 sensor
  * @note   This function modifies the AVG bits (bits 5 and 6) of the Configuration Register (0x01).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  avg Selected averaging mode.
  *             This parameter can be one of the following values:
  *             @arg TMP117_NO_SAMPLES: No averaging (1 conversion)
  *             @arg TMP117_8_SAMPLES: 8 conversions averaged
  *             @arg TMP117_32_SAMPLES: 32 conversions averaged
  *             @arg TMP117_64_SAMPLES: 64 conversions averaged
  * @retval None
  */
void TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Avg_TypeDef avg)
{
    // Read the current CONFIGURATION register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the AVG bits using the mask (bits 5 and 6)
    regValue &= ~TMP117_AVG_Mask;

    // Set the new AVG by shifting it to its position (AVG_Pos = 5)
    // and protecting it with the mask
    regValue |= (avg << TMP117_AVG_Pos) & TMP117_AVG_Mask;

    // Write the resulting value back to the register
    // (Nota: Corregido INA236_WriteRegister por TMP117_WriteRegister)
    TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the operating mode of the TMP117 sensor (Continuous, Shutdown, or One-Shot)
  * @note   This function modifies the MOD bits (bits 10 and 11) of the Configuration Register (0x01).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  mode Selected operating mode.
  *              This parameter can be one of the following values:
  *              @arg TMP117_CONTINUOUS_MODE: Continuous conversion mode
  *              @arg TMP117_SHUTDOWN_MODE: Low-power shutdown mode
  *              @arg TMP117_ONE_SHOT_MODE: One-shot conversion mode
  * @retval None
  */
void TMP117_SetMode(TMP117_HandleTypeDef *tmp117, TMP117_Mode_TypeDef mode)
{
    // Read the current CONFIGURATION register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the MODE bits using the mask (bits 10 and 11)
    regValue &= ~TMP117_MOD_Mask;

    // Set the new MODE by shifting it to its position (MOD_Pos = 10)
    // and protecting it with the mask
    regValue |= (mode << TMP117_MOD_Pos) & TMP117_MOD_Mask;

    // Write the resulting value back to the register
    TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Check if the TMP117 EEPROM is currently busy
  * @note   This function reads the EEPROM_Busy flag (bit 12) of the Configuration Register (0x01).
  *         This flag indicates whether the EEPROM is busy during programming or during power-up.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval _Bool Status of the EEPROM:
  *         - 1 (true): EEPROM is busy programming or loading values.
  *         - 0 (false): EEPROM is idle and ready.
  */
_Bool TMP117_IsEEPROMBusy(TMP117_HandleTypeDef *tmp117)
{
    // Read the Configuration Register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Mask the value to isolate bit 12 (TMP117_EEPROMBUSY)
    // If the bit is set, (regValue & TMP117_EEPROMBUSY) will be 0x1000 (which is != 0)
    if ((regValue & TMP117_EEPROMBUSY) != 0U)
    {
        return 1; 
    }

    return 0;
}

/**
  * @brief  Check if new temperature conversion data is ready to be read
  * @note   This function reads the DataReady flag (bit 13) of the Configuration Register (0x01).
  *         Every time the temperature register or the configuration register is read, this bit is 
  *         automatically cleared. The bit is set at the end of a conversion when the temperature 
  *         register is updated. Data ready can also be monitored on the physical ALERT pin 
  *         by configuring the DR/Alert bit (bit 2) accordingly.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval _Bool Data ready status:
  *         - 1 (true): New conversion data is ready and can be read.
  *         - 0 (false): No new data is ready (or flag has been cleared by a read operation).
  */
_Bool TMP117_IsDataReady(TMP117_HandleTypeDef *tmp117)
{
    // Read the Configuration Register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);
    // Mask the value to isolate bit 13 (TMP117_DATAREADY)
    // If the bit is set, (regValue & TMP117_DATAREADY) will be 0x2000 (which is != 0)
    if ((regValue & TMP117_DATAREADY) != 0U)
    {
        return 1; 
    }
    return 0;
}

/**
  * @brief  Check if the temperature has fallen below the low limit (Low Alert flag)
  * @note   This function reads the LOW_Alert flag (bit 14) of the Configuration Register (0x01).
  *         In Alert mode, this flag is set to 1 when the conversion result is lower than the low limit 
  *         register, and is cleared to 0 when the configuration register is read.
  *         In Thermostat (Therm) mode, this flag is always set to 0.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval _Bool Low alert status:
  *         - 1 (true): Temperature is lower than the configured low limit (in Alert mode).
  *         - 0 (false): No low alert has occurred, the configuration register was read, or device is in Therm mode.
  */
_Bool TMP117_IsLowAlertSet(TMP117_HandleTypeDef *tmp117)
{
    // Read the Configuration Register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Mask the value to isolate bit 14 (TMP117_LOWALERT)
    // If the bit is set, (regValue & TMP117_LOWALERT) will be 0x4000 (which is != 0)
    if ((regValue & TMP117_LOWALERT) != 0U)
    {
        return 1; 
    }

    return 0;
}

/**
  * @brief  Check if the temperature has exceeded the high limit (High Alert flag)
  * @note   This function reads the HIGH_Alert flag (bit 15) of the Configuration Register (0x01).
  *         The behavior of this flag depends on the configured mode of operation (Alert vs Therm):
  *         - In Alert mode: The flag is set to 1 when the conversion result is higher than the high limit 
  *           and is automatically cleared to 0 when the configuration register is read.
  *         - In Thermostat (Therm) mode: The flag is set to 1 when the conversion result is higher than 
  *           the high limit (therm limit) and is cleared to 0 only when the result falls below the 
  *           low limit (which acts as the hysteresis threshold).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval _Bool High alert status:
  *         - 1 (true): Temperature has exceeded the high limit threshold.
  *         - 0 (false): Temperature is within normal limits, the configuration register was read (Alert mode), 
  *           or the temperature has dropped below the hysteresis threshold (Therm mode).
  */
_Bool TMP117_IsHighAlertSet(TMP117_HandleTypeDef *tmp117)
{
    // Read the Configuration Register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Mask the value to isolate bit 15 (TMP117_HIGHALERT)
    // If the bit is set, (regValue & TMP117_HIGHALERT) will be 0x8000 (which is != 0)
    if ((regValue & TMP117_HIGHALERT) != 0U)
    {
        return 1; 
    }

    return 0;
}