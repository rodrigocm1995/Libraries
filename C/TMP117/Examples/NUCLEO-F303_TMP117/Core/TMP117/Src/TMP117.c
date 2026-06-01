#include "main.h"
#include "math.h" // Functions that operate on floating point numbers are in math.h
#include <stdio.h>
#include "TMP117.h"

/**
  * @brief  Write a 16-bit value to a specific register of the TMP117 device
  * @param  ina236 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  registerAddress Internal register address of the INA236 to write to
  * @param  value The 16-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value)
{
    uint8_t address[2];
    // Split the 16-bit value into two bytes (MSB first)
    address[0] = (value >> 8) & 0xFF;
    address[1] = (value >> 0) & 0xFF;
  
    // Check if the device is ready on the I2C bus
    HAL_StatusTypeDef isDeviceReady = HAL_I2C_IsDeviceReady(tmp117->hi2c, (tmp117->_devAddress) << 1, TMP117_TRIALS, HAL_MAX_DELAY);
    if (isDeviceReady == HAL_OK)
    {
        // Write the 16-bit register to the device
        if (HAL_I2C_Mem_Write(tmp117->hi2c, (tmp117->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY) == HAL_OK)
        {
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 16-bit value from a specific register of the INA236 device
  * @param  ina236 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  registerAddress Internal register address of the TMP117 to read from 
  * @return 16-bit data read from register, or 0xFFFF if the operation fails
  */
uint16_t TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress)
{
    uint8_t registerResponse[2] = {0}; 
    HAL_StatusTypeDef isDeviceReady;   

    // Check if the device is ready on the I2C bus
    isDeviceReady = HAL_I2C_IsDeviceReady(tmp117->hi2c, (tmp117->_devAddress) << 1, TMP117_TRIALS, HAL_MAX_DELAY);

    if (isDeviceReady == HAL_OK)
    {
        // Read the 2-byte register data from the device
        if (HAL_I2C_Mem_Read(tmp117->hi2c, (tmp117->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) == HAL_OK)
        {
            // Combine MSB and LSB to return the 16-bit value
            return (uint16_t)((registerResponse[0] << 8) | registerResponse[1]);
        }       
    }
    // Returns a maximum control value (0xFFFF) indicating a communication error
    return 0xFFFF;
}

/**
  * @brief  Read the raw value from the Configuration register
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @return The 16-bit raw value of the Configuration Register (Address: 0x01), 
  *         or 0xFFFF if I2C communication fails.
  */
uint16_t TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    return regValue;
}

/**
  * @brief  Read the raw value from the Temperature High Limit Register
  * @note   This function retrieves the current configuration of the high limit comparison threshold 
  *         from the Temperature High Limit Register (Address: 0x02).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval uint16_t The 16-bit raw value of the Temperature High Limit Register, 
  *                  or 0xFFFF if I2C communication fails.
  */
uint16_t TMP117_GetTempHighLimitReg(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG);

    return regValue;
}

/**
  * @brief  Read the raw value from the Temperature Low Limit Register
  * @note   This function retrieves the current configuration of the low limit comparison threshold 
  *         from the Temperature Low Limit Register (Address: 0x03).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval uint16_t The 16-bit raw value of the Temperature Low Limit Register, 
  *                  or 0xFFFF if I2C communication fails.
  */
uint16_t TMP117_GetTempLowLimitReg(TMP117_HandleTypeDef *tmp117)
{
	uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG);

	return regValue;
}

/**
  * @brief  Read the unique Device ID from the TMP117 sensor
  * @note   This function reads the Device ID Register (Address: 0x0F) which contains 
  *         the silicon identification signature of the chip.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval uint16_t The 16-bit raw value of the Device ID Register, 
  *                  or 0xFFFF if I2C communication fails.
  */
uint16_t TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117)
{
	uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_DEVICE_ID_REG);

	return regValue;
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
	uint16_t value = TMP117_GetConfiguration(tmp117);
	_Bool valueFlag = CHECK_BIT(value, 12);
	return valueFlag;
}

/**
  *@brief Get the high limit temperature value in degree celsius
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double temperature high limit value
*/
double TMP117_GetHighLimitTemp_C(TMP117_HandleTypeDef *tmp117)
{
	double temp = 0.0;
	int16_t rawTemp = TMP117_GetTempHighLimitReg(tmp117);
	temp = TMP117_CheckTemperature(rawTemp);
	return temp;
}

/**
  *@brief Get the low limit temperature value in degree celsius
  *@param tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *			the configuration information for connecting to the sensor.
  *@retval	double temperature low limit value
*/
double TMP117_GetLowLimitTemp_C(TMP117_HandleTypeDef *tmp117)
{
	double temp = 0.0;
	int16_t rawTemp = TMP117_GetTempLowLimitReg(tmp117);
	temp = TMP117_CheckTemperature(rawTemp);
	return temp;
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
void TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  // Bind physical communication parameters to local handler
    tmp117->hi2c = i2c;
    tmp117->_devAddress = devAddress;
  
    TMP117_SetAlertPinFunction(tmp117, TMP117_ALERT_FOR_DATA_READY_FLAG);
    TMP117_SetAlertPinPolarity(tmp117, TMP117_ALERT_ACTIVE_HIGH);
    TMP117_SetThermAlertMode(tmp117, TMP117_ALERT_MODE);
    TMP117_SetAverage(tmp117, TMP117_32_SAMPLES);
    TMP117_SetMode(tmp117, TMP117_CONTINUOUS_MODE);  
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
    TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, TMP117_SOFTRESET);
    
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
	TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
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
    regValue |= (tnA << TMP117_TnA_Pos) & TMP117_TnA_Mask;

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

    if (avg == TMP117_NO_SAMPLES)
    {
        tmp117->_samples = 1;
    } 
    else if (avg == TMP117_8_SAMPLES)
    {
        tmp117->_samples = 8;
    }
    else if (avg == TMP117_32_SAMPLES)
    {
        tmp117->_samples = 32;
    }
    else if (avg == TMP117_64_SAMPLES)
    {
        tmp117->_samples = 64;
    }
    
    // Clear the AVG bits using the mask (bits 5 and 6)
    regValue &= ~TMP117_AVG_Mask;

    // Set the new AVG by shifting it to its position (AVG_Pos = 5)
    // and protecting it with the mask
    regValue |= (avg << TMP117_AVG_Pos) & TMP117_AVG_Mask;

    // Write the resulting value back to the register
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

/**
  * @brief  Configure the conversion cycle time for the TMP117 sensor
  * @note   This function modifies the CONV bits (bits 7, 8, and 9) of the Configuration Register (0x01).
  *         The requested conversion cycle time must be greater than or equal to the active conversion
  *         time, which is calculated based on the current averages: active_time = _samples * 15.5 ms.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  convTime Selected conversion cycle time.
  *                  This parameter can be one of the following values:
  *                  @arg TMP117_CONV_15_5_MS: 15.5 ms conversion cycle time
  *                  @arg TMP117_CONV_125_MS: 125 ms conversion cycle time
  *                  @arg TMP117_CONV_250_MS: 250 ms conversion cycle time
  *                  @arg TMP117_CONV_500_MS: 500 ms conversion cycle time
  *                  @arg TMP117_CONV_1_S: 1 s conversion cycle time
  *                  @arg TMP117_CONV_4_S: 4 s conversion cycle time
  *                  @arg TMP117_CONV_8_S: 8 s conversion cycle time
  *                  @arg TMP117_CONV_16_S: 16 s conversion cycle time
  * @retval HAL_StatusTypeDef returns:
  *         - HAL_OK: If configuration is successfully validated and written to the register.
  *         - HAL_ERROR: If validation fails (requested time < active time) or if I2C write fails.
  */
HAL_StatusTypeDef TMP117_SetConvTime(TMP117_HandleTypeDef *tmp117, TMP117_ConvTime_TypeDef convTime)
{
    double activeTime = tmp117->_samples * 15.5;
    tmp117->_activeTime = activeTime;
    double requestedTime = 0.0;
    // Map the selected enum to its physical time in milliseconds
    switch (convTime)
    {
        case TMP117_CONV_15_5_MS: requestedTime = 15.5;   break;
        case TMP117_CONV_125_MS:  requestedTime = 125.0;  break;
        case TMP117_CONV_250_MS:  requestedTime = 250.0;  break;
        case TMP117_CONV_500_MS:  requestedTime = 500.0;  break;
        case TMP117_CONV_1_S:     requestedTime = 1000.0; break;
        case TMP117_CONV_4_S:     requestedTime = 4000.0; break;
        case TMP117_CONV_8_S:     requestedTime = 8000.0; break;
        case TMP117_CONV_16_S:    requestedTime = 16000.0;break;
        default:                  requestedTime = 0.0;    break;
    }
    
    tmp117->_requestedTime = requestedTime; 

    // Validate that the total cycle time is not less than the active conversion time
    if (requestedTime < activeTime)
    {
        return HAL_ERROR;
    }
    // Read the current CONFIGURATION register (Address: 0x01)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG);

    // Clear the CONV bits using the mask (bits 7, 8, and 9)
    regValue &= ~TMP117_CONV_Mask;

    // Set the new CONV by shifting it to its position (CONV_Pos = 7)
    // and protecting it with the mask
    regValue |= (convTime << TMP117_CONV_Pos) & TMP117_CONV_Mask;

    // Write the resulting value back to the register and return the HAL write status
    return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Set the temperature high limit threshold for comparison on the TMP117 sensor
  * @details This function writes to the Temperature High Limit Register (Address: 0x02) which 
  *          stores the high threshold. One LSB equals 7.8125 m°C (0.0078125 °C) with a range of ±256 °C.
  *          Negative temperatures are automatically converted to binary two's complement format.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  highLimit High limit temperature threshold in degrees Celsius (°C).
  * @retval None
  */
void TMP117_SetHighLimit_C(TMP117_HandleTypeDef *tmp117, double highLimit)
{
    // One LSB equals 7.8125 m°C = 0.0078125 °C.
    // Convert the double value in °C to 16-bit signed integer (two's complement)
    int16_t regValue = (int16_t)(highLimit / 0.0078125);
    // Write the raw value back to the high limit register
    TMP117_WriteRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Set the temperature low limit threshold for comparison on the TMP117 sensor
  * @details This function writes to the Temperature Low Limit Register (Address: 0x03) which 
  *          stores the low threshold. One LSB equals 7.8125 m°C (0.0078125 °C) with a range of ±256 °C.
  *          Negative temperatures are automatically converted to binary two's complement format.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @param  lowLimit Low limit temperature threshold in degrees Celsius (°C).
  * @retval None
  */
void TMP117_SetLowLimit_C(TMP117_HandleTypeDef *tmp117, double lowLimit)
{
    // One LSB equals 7.8125 m°C = 0.0078125 °C.
    // Convert the double value in °C to 16-bit signed integer (two's complement)
    int16_t regValue = (int16_t)(lowLimit / 0.0078125);
    // Write the raw value back to the low limit register
    TMP117_WriteRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Read the temperature measurement and calculate its value in degrees Celsius (°C)
  * @note   The TMP117 temperature result register (Address: 0x00) has a resolution of 7.8125 m°C/LSB 
  *         (0.0078125 °C/LSB) and represents negative values in binary two's complement format.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified TMP117.
  * @retval double The measured temperature in degrees Celsius (°C), or -999.0 if the read operation fails.
  */
double TMP117_GetTemperature_C(TMP117_HandleTypeDef *tmp117)
{
    // Read the Temperature Result Register (Address: 0x00)
    uint16_t regValue = TMP117_ReadRegister(tmp117, TMP117_TEMP_RESULT_REG);

    // Cast to signed 16-bit integer (two's complement)
    // This preserves the negative sign if the temperature is below 0 °C
    int16_t rawTemp = (int16_t)regValue;

    // Convert raw LSB value to degrees Celsius (1 LSB = 0.0078125 °C)
    return (double)rawTemp * 0.0078125;
}