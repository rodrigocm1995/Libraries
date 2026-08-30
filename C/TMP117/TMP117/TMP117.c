#include "TMP117.h"
#include "i2c_bus.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_def.h"
#include <math.h>   // Required for NAN float error indicator

/* =======================================================================================
   CAPA DE ADAPTACIÓN PRIVADA (WRAPPERS INTERNOS)
   ======================================================================================= */

static inline HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value)
{
    if (tmp117 == NULL || tmp117->hi2c == NULL) return HAL_ERROR;
    return I2C_Bus_WriteRegister16_BE(tmp117->hi2c, tmp117->_devAddress, registerAddress, value);
}

static inline HAL_StatusTypeDef TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t *value)
{
    if (tmp117 == NULL || tmp117->hi2c == NULL || value == NULL) return HAL_ERROR;
    return I2C_Bus_ReadRegister16_BE(tmp117->hi2c, tmp117->_devAddress, registerAddress, value);
}

/* =======================================================================================
   CAPA DE APLICACIÓN (FUNCIONES PÚBLICAS)
   ======================================================================================= */

/**
  * @brief  Initialize the TMP117 sensor handle and configure default operating settings.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  i2c Pointer to a HAL I2C_HandleTypeDef.
  * @param  devAddress The 7-bit physical I2C device address.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    if (tmp117 == NULL || i2c == NULL)
    {
        return HAL_ERROR;
    }

    tmp117->hi2c = i2c;
    tmp117->_devAddress = devAddress;
    tmp117->_samples = 8;              // Default: TMP117_8_SAMPLES -> 8 samples
    tmp117->_activeTime = 124.0f;      // Default active conversion time for 8 samples: 8 * 15.5 ms = 124.0 ms
    tmp117->_requestedTime = 1000.0f;  // Default conversion cycle time: TMP117_CONV_1_S -> 1000 ms
  
    uint16_t config = 0;
    config |= (TMP117_ALERT_FOR_DATA_READY_FLAG << TMP117_DRALERT_Pos) & TMP117_DRALERT;
    config |= (TMP117_ALERT_ACTIVE_HIGH         << TMP117_POL_Pos)     & TMP117_POL;
    config |= (TMP117_ALERT_MODE                << TMP117_TnA_Pos)     & TMP117_TnA;
    config |= (TMP117_8_SAMPLES                 << TMP117_AVG_Pos)     & TMP117_AVG;
    config |= (TMP117_CONV_1_S                  << TMP117_CONV_Pos)    & TMP117_CONV;
    config |= (TMP117_CONTINUOUS_MODE           << TMP117_MOD_Pos)     & TMP117_MOD;

    return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, config);
}

/**
  * @brief  Read the raw 16-bit value from the Configuration register (0x01).
  *         This register controls the operational modes, averaging options,
  *         conversion time cycle, alert pins behavior, and reset status.
  *         Key bits monitored/set in this register:
  *         - Bit 15: HIGH_Alert status flag
  *         - Bit 14: LOW_Alert status flag
  *         - Bit 13: Data_Ready status flag
  *         - Bit 12: EEPROM_Busy status flag
  *         - Bits 11-10: Operating MOD[1:0]
  *         - Bits 9-7: CONV[2:0] conversion cycle time
  *         - Bits 6-5: AVG[1:0] averaging samples count
  *         - Bit 4: T/nA Therm/Alert mode selection
  *         - Bit 3: POL alert pin output polarity
  *         - Bit 2: DR/Alert pin function select
  *         - Bit 1: Soft_Reset bit
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit configuration register value.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG, value);
}

/**
  * @brief  Read the raw value from the Temperature High Limit Register (0x02).
  *         This register contains the 16-bit 2's complement temperature threshold
  *         used to evaluate if a HIGH alert event has occurred.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the raw 16-bit high limit threshold.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetTempHighLimitReg(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, value);
}

/**
  * @brief  Read the raw value from the Temperature Low Limit Register (0x03).
  *         This register contains the 16-bit 2's complement temperature threshold
  *         used to evaluate if a LOW alert event has occurred.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the raw 16-bit low limit threshold.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetTempLowLimitReg(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, value);
}

/**
  * @brief  Read the unique Device ID from the TMP117 sensor (0x0F).
  *         This read-only register returns the hardware identifier for the TMP117.
  *         Expected factory identifier is typically 0x0117.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit device ID value.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_DEVICE_ID_REG, value);
}

/**
  * @brief  Read the raw value from the EEPROM Unlock Register (0x04).
  *         Provides the status of the EEPROM lock, which must be unlocked before
  *         writing parameters to non-volatile EEPROM memory.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the raw 16-bit value of the unlock register.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetEepromUnlock(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM_UNLOCK_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM1 register (0x05).
  *         This register provides general-purpose user-programmable non-volatile memory
  *         or holds alternate configuration settings if enabled.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit register value.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM1_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM2 register (0x06).
  *         This register provides general-purpose user-programmable non-volatile memory.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit register value.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM2_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM3 register (0x08).
  *         This register provides general-purpose user-programmable non-volatile memory.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit register value.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM3_REG, value);
}

/**
  * @brief  Write data to the EEPROM1 register (0x05).
  *         Allows writing 16 bits of non-volatile data. The EEPROM must be unlocked
  *         by writing 0x8000 to the EEPROM_UNLOCK register before writing.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  data 16-bit data to write to the EEPROM1 register.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM1_REG, data);
}

/**
  * @brief  Write data to the EEPROM2 register (0x06).
  *         Allows writing 16 bits of non-volatile data. The EEPROM must be unlocked
  *         by writing 0x8000 to the EEPROM_UNLOCK register before writing.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  data 16-bit data to write to the EEPROM2 register.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM2_REG, data);
}

/**
  * @brief  Write data to the EEPROM3 register (0x08).
  *         Allows writing 16 bits of non-volatile data. The EEPROM must be unlocked
  *         by writing 0x8000 to the EEPROM_UNLOCK register before writing.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  data 16-bit data to write to the EEPROM3 register.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM3_REG, data);
}


/**
  * @brief  Helper function to convert 16-bit raw registers to temperature Celsius.
  * @param  value Raw register value.
  * @retval Temperature in Celsius.
  */
float TMP117_ConvertRawToCelsius(uint16_t value)
{
    int16_t rawTemp = (int16_t)value;
    return (float)rawTemp * 0.0078125;
}

/**
  * @brief  Get the high limit temperature value in degrees Celsius (°C)
  *         This function reads the 16-bit register value of the Temperature High Limit Register,
  *         checks for errors, converts the 2's complement raw value to a floating-point temperature,
  *         and writes the output to the provided pointer.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  temp Pointer to a float where the calculated high limit temperature will be stored.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetHighLimitTemp_C(TMP117_HandleTypeDef *tmp117, float *temp)
{
    if (temp == NULL) return HAL_ERROR;

    uint16_t rawTemp = 0;
    HAL_StatusTypeDef status = TMP117_GetTempHighLimitReg(tmp117, &rawTemp);
    if (status == HAL_OK)
    {
        *temp = TMP117_ConvertRawToCelsius(rawTemp);
    }
    return status;
}

/**
  * @brief  Get the low limit temperature value in degrees Celsius (°C)
  *         This function reads the 16-bit register value of the Temperature Low Limit Register,
  *         checks for errors, converts the 2's complement raw value to a floating-point temperature,
  *         and writes the output to the provided pointer.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  temp Pointer to a float where the calculated low limit temperature will be stored.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetLowLimitTemp_C(TMP117_HandleTypeDef *tmp117, float *temp)
{
    if (temp == NULL) return HAL_ERROR;

    uint16_t rawTemp = 0;
    HAL_StatusTypeDef status = TMP117_GetTempLowLimitReg(tmp117, &rawTemp);
    if (status == HAL_OK)
    {
        *temp = TMP117_ConvertRawToCelsius(rawTemp);
    }
    return status;
}



/**
  * @brief  Reset the TMP117 device registers to their default factory values
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_ResetDevice(TMP117_HandleTypeDef *tmp117)
{
    HAL_StatusTypeDef status = TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, TMP117_SOFTRESET);
    
    if (status != HAL_OK)
    {
        return status;
    }

    uint16_t regValue = TMP117_SOFTRESET;
    uint32_t startTick = HAL_GetTick();
    const uint32_t timeout = 5;

    do
    {
        status = TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG, &regValue);
        if (status != HAL_OK)
        {
            return status;
        }

        if (!(regValue & TMP117_SOFTRESET)) // if RESET bit = 0 then
        {
            tmp117->_samples = 8;
            tmp117->_activeTime = 124.0f;     // 8 * 15.5 ms = 124.0 ms
            tmp117->_requestedTime = 1000.0f; // 1 second = 1000.0 ms
            return HAL_OK;
        }
    }
    while ((HAL_GetTick() - startTick) < timeout);
    
    return HAL_TIMEOUT;
}

/**
  * @brief  Configure the ALERT pin function on the TMP117 sensor.
  *         This register bit (DR/Alert) configures the physical pin alert style:
  *         either as a traditional temperature limit alert or as a conversion data ready flag.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  pinFunction Selected alert pin function mode.
  *         This parameter can be one of the following values:
  *         @arg TMP117_ALERT_FOR_DATA_READY_FLAG: Pin reports Data Ready status (default, 1b)
  *         @arg TMP117_ALERT_FOR_ALERT_FLAGS: Pin reports limit temperature alerts (0b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetAlertPinFunction(TMP117_HandleTypeDef *tmp117, TMP117_DRALERT_TypeDef pinFunction)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        regValue &= ~TMP117_DRALERT_Mask;
        regValue |= (pinFunction << TMP117_DRALERT_Pos) & TMP117_DRALERT_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the active polarity of the physical ALERT pin on the TMP117 sensor.
  *         Sets the output electrical level of the ALERT pin when active.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  polarity Selected alert pin polarity.
  *         This parameter can be one of the following values:
  *         @arg TMP117_ALERT_ACTIVE_HIGH: Active High output (0b)
  *         @arg TMP117_ALERT_ACTIVE_LOW: Active Low output (default, 1b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetAlertPinPolarity(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinPol_TypeDef polarity)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        regValue &= ~TMP117_POL_Mask;
        regValue |= (polarity << TMP117_POL_Pos) & TMP117_POL_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the Therm/Alert mode on the TMP117 sensor.
  *         The T/nA bit determines whether the device operates in Thermostat mode 
  *         (active until temperature drops below high limit - hysteresis) or Alert mode 
  *         (comparator style, cleared upon reading Configuration register).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  tnA Selected mode of operation.
  *         This parameter can be one of the following values:
  *         @arg TMP117_THERM_MODE: Thermostat mode (with hysteresis, 1b)
  *         @arg TMP117_ALERT_MODE: Alert/Comparator mode (default, 0b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetThermAlertMode(TMP117_HandleTypeDef *tmp117, TMP117_ThermAlertMode_TypeDef tnA)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        regValue &= ~TMP117_TnA_Mask;
        regValue |= (tnA << TMP117_TnA_Pos) & TMP117_TnA_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the number of conversion averages for the TMP117 sensor.
  *         Sets the internal hardware oversampling rate to filter out noise and improve resolution.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  avg Selected averaging mode.
  *         This parameter can be one of the following values:
  *         @arg TMP117_NO_SAMPLES: 1 conversion (no averaging, 00b)
  *         @arg TMP117_8_SAMPLES: 8 conversions averaged (default, 01b)
  *         @arg TMP117_32_SAMPLES: 32 conversions averaged (10b)
  *         @arg TMP117_64_SAMPLES: 64 conversions averaged (11b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Avg_TypeDef avg)
{
    if (tmp117 == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
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
        
        // Update cached active conversion time based on the new sample count
        tmp117->_activeTime = (float)tmp117->_samples * 15.5f;
        
        regValue &= ~TMP117_AVG_Mask;
        regValue |= (avg << TMP117_AVG_Pos) & TMP117_AVG_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the operating mode of the TMP117 sensor.
  *         Selects whether the device operates in continuous conversions, shutdown (sleep),
  *         or one-shot triggered measurement mode.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  mode Selected operating mode.
  *         This parameter can be one of the following values:
  *         @arg TMP117_CONTINUOUS_MODE: Continuous Conversion Mode (default, 00b)
  *         @arg TMP117_SHUTDOWN_MODE: Shutdown Mode (low-power sleep, 01b)
  *         @arg TMP117_ONE_SHOT_MODE: One-Shot/Single-Shot Conversion Mode (11b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetMode(TMP117_HandleTypeDef *tmp117, TMP117_Mode_TypeDef mode)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        regValue &= ~TMP117_MOD_Mask;
        regValue |= (mode << TMP117_MOD_Pos) & TMP117_MOD_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Check if the TMP117 EEPROM is currently busy
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Boolean status
  */
_Bool TMP117_IsEEPROMBusy(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        if ((regValue & TMP117_EEPROMBUSY) != 0U)
        {
            return 1; 
        }
    }
    return 0;
}

/**
  * @brief  Check if new temperature conversion data is ready to be read
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Boolean status
  */
_Bool TMP117_IsDataReady(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        if ((regValue & TMP117_DATAREADY) != 0U)
        {
            return 1; 
        }
    }
    return 0;
}

/**
  * @brief  Check if the temperature has fallen below the low limit (Low Alert flag)
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Boolean status
  */
_Bool TMP117_IsLowAlertSet(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        if ((regValue & TMP117_LOWALERT) != 0U)
        {
            return 1; 
        }
    }
    return 0;
}

/**
  * @brief  Check if the temperature has exceeded the high limit (High Alert flag)
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Boolean status
  */
_Bool TMP117_IsHighAlertSet(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        if ((regValue & TMP117_HIGHALERT) != 0U)
        {
            return 1; 
        }
    }
    return 0;
}

/**
  * @brief  Configure the conversion cycle time for the TMP117 sensor.
  *         This parameter sets the duration of the conversion cycle, which must be
  *         longer than the active conversion time (based on the oversampling rate).
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  convTime Selected conversion cycle time.
  *         This parameter can be one of the following values:
  *         @arg TMP117_CONV_15_5_MS: 15.5 ms conversion time (000b)
  *         @arg TMP117_CONV_125_MS: 125 ms conversion time (001b)
  *         @arg TMP117_CONV_250_MS: 250 ms conversion time (010b)
  *         @arg TMP117_CONV_500_MS: 500 ms conversion time (011b)
  *         @arg TMP117_CONV_1_S: 1.0 s conversion time (default, 100b)
  *         @arg TMP117_CONV_4_S: 4.0 s conversion time (101b)
  *         @arg TMP117_CONV_8_S: 8.0 s conversion time (110b)
  *         @arg TMP117_CONV_16_S: 16.0 s conversion time (111b)
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetConvTime(TMP117_HandleTypeDef *tmp117, TMP117_ConvTime_TypeDef convTime)
{
    float activeTime = tmp117->_samples * 15.5;
    tmp117->_activeTime = activeTime;
    float requestedTime = 0.0;
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
    if (requestedTime < activeTime)
    {
        return HAL_ERROR;
    }
    
    uint16_t regValue = 0;
    if (TMP117_GetConfiguration(tmp117, &regValue) == HAL_OK)
    {
        regValue &= ~TMP117_CONV_Mask;
        regValue |= (convTime << TMP117_CONV_Pos) & TMP117_CONV_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Set the temperature high limit threshold for comparison on the TMP117 sensor.
  *         Converts the floating-point temperature in Celsius to a 16-bit 2's complement
  *         register value using the 7.8125 m°C LSB scale, and writes it to the Temperature
  *         High Limit Register (0x02). Used in Therm/Alert interrupt flag triggers.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  highLimit High limit temperature threshold in degrees Celsius (°C).
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetHighLimit_C(TMP117_HandleTypeDef *tmp117, float highLimit)
{
    int16_t regValue = (int16_t)(highLimit / 0.0078125);
    return TMP117_WriteRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Set the temperature low limit threshold for comparison on the TMP117 sensor.
  *         Converts the floating-point temperature in Celsius to a 16-bit 2's complement
  *         register value using the 7.8125 m°C LSB scale, and writes it to the Temperature
  *         Low Limit Register (0x03). Used in Therm/Alert interrupt flag triggers.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  lowLimit Low limit temperature threshold in degrees Celsius (°C).
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_SetLowLimit_C(TMP117_HandleTypeDef *tmp117, float lowLimit)
{
    int16_t regValue = (int16_t)(lowLimit / 0.0078125);
    return TMP117_WriteRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Read the temperature measurement and calculate its value in degrees Celsius (°C).
  *         This function reads the 16-bit register value of the Temperature Result Register (0x00),
  *         converts the 2's complement raw value to a floating-point temperature value in Celsius,
  *         and writes the output to the provided pointer.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  temp Pointer to a float variable where the calculated temperature will be stored.
  * @return HAL status
  */
HAL_StatusTypeDef TMP117_GetTemperature_C(TMP117_HandleTypeDef *tmp117, float *temp)
{
    if (temp == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = TMP117_ReadRegister(tmp117, TMP117_TEMP_RESULT_REG, &regValue);

    if (status == HAL_OK)
    {
        *temp = TMP117_ConvertRawToCelsius(regValue);
    }

    return status;
}
