#include "TMP117.h"
#include "i2c_bus.h"
#include <math.h>   // Required for NAN float error indicator

/* =======================================================================================
   CAPA DE ADAPTACIÓN PRIVADA (WRAPPERS INTERNOS)
   ======================================================================================= */

static inline HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value)
{
    return I2C_Bus_WriteRegister16_BE(tmp117->hi2c, tmp117->_devAddress, registerAddress, value);
}

static inline HAL_StatusTypeDef TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t *value)
{
    return I2C_Bus_ReadRegister16_BE(tmp117->hi2c, tmp117->_devAddress, registerAddress, value);
}

/* =======================================================================================
   CAPA DE APLICACIÓN (FUNCIONES PÚBLICAS)
   ======================================================================================= */

/**
  * @brief  Read the raw value from the Configuration register
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_CONFIGURATION_REG, value);
}

/**
  * @brief  Read the raw value from the Temperature High Limit Register
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetTempHighLimitReg(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, value);
}

/**
  * @brief  Read the raw value from the Temperature Low Limit Register
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetTempLowLimitReg(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, value);
}

/**
  * @brief  Read the unique Device ID from the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_DEVICE_ID_REG, value);
}

/**
  * @brief  Read the raw value from the EEPROM Unlock Register
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetEepromUnlock(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM_UNLOCK_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM1 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM1_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM2 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM2_REG, value);
}

/**
  * @brief  Get the current value of the EEPROM3 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit register value.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_GetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t *value)
{
    return TMP117_ReadRegister(tmp117, TMP117_EEPROM3_REG, value);
}

/**
  * @brief  Write data to the EEPROM1 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  data 16-bit data to write.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM1_REG, data);
}

/**
  * @brief  Write data to the EEPROM2 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  data 16-bit data to write.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM2_REG, data);
}

/**
  * @brief  Write data to the EEPROM3 register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @param  data 16-bit data to write.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t data)
{
    return TMP117_WriteRegister(tmp117, TMP117_EEPROM3_REG, data);
}

/**
  * @brief  Read the EEPROM_Busy bit of the CONFIGURATION register.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef structure.
  * @retval Boolean flag status
  */
_Bool TMP117_EepromBusyFlag(TMP117_HandleTypeDef *tmp117)
{
    uint16_t value = 0;
    if (TMP117_GetConfiguration(tmp117, &value) == HAL_OK)
    {
        return CHECK_BIT(value, 12);
    }
    return 0;
}

/**
  * @brief  Helper function to convert 16-bit raw registers to temperature Celsius.
  * @param  value Raw register value.
  * @retval Temperature in Celsius.
  */
double TMP117_CheckTemperature(uint16_t value)
{
    int16_t rawTemp = (int16_t)value;
    return (double)rawTemp * 0.0078125;
}

/**
  * @brief  Get the high limit temperature value in degree celsius
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Temperature in Celsius, or NAN on error.
  */
double TMP117_GetHighLimitTemp_C(TMP117_HandleTypeDef *tmp117)
{
    uint16_t rawTemp = 0;
    if (TMP117_GetTempHighLimitReg(tmp117, &rawTemp) == HAL_OK)
    {
        return TMP117_CheckTemperature(rawTemp);
    }
    return NAN;
}

/**
  * @brief  Get the low limit temperature value in degree celsius
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval Temperature in Celsius, or NAN on error.
  */
double TMP117_GetLowLimitTemp_C(TMP117_HandleTypeDef *tmp117)
{
    uint16_t rawTemp = 0;
    if (TMP117_GetTempLowLimitReg(tmp117, &rawTemp) == HAL_OK)
    {
        return TMP117_CheckTemperature(rawTemp);
    }
    return NAN;
}

/**
  * @brief  Initialize the TMP117 sensor handle and configure default operating settings.
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  i2c Pointer to a HAL I2C_HandleTypeDef.
  * @param  devAddress The 7-bit physical I2C device address.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    tmp117->hi2c = i2c;
    tmp117->_devAddress = devAddress;
  
    if (TMP117_SetAlertPinFunction(tmp117, TMP117_ALERT_FOR_DATA_READY_FLAG) != HAL_OK) return HAL_ERROR;
    if (TMP117_SetAlertPinPolarity(tmp117, TMP117_ALERT_ACTIVE_HIGH) != HAL_OK) return HAL_ERROR;
    if (TMP117_SetThermAlertMode(tmp117, TMP117_ALERT_MODE) != HAL_OK) return HAL_ERROR;
    if (TMP117_SetAverage(tmp117, TMP117_32_SAMPLES) != HAL_OK) return HAL_ERROR;
    if (TMP117_SetConvTime(tmp117, TMP117_CONV_4_S) != HAL_OK) return HAL_ERROR;
    if (TMP117_SetMode(tmp117, TMP117_CONTINUOUS_MODE) != HAL_OK) return HAL_ERROR;
    
    return HAL_OK;
}

/**
  * @brief  Reset the TMP117 device registers to their default factory values
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_ResetDevice(TMP117_HandleTypeDef *tmp117)
{
    HAL_StatusTypeDef status = TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, TMP117_SOFTRESET);
    if (status == HAL_OK)
    {
        HAL_Delay(2);
    }
    return status;
}

/**
  * @brief  Configure the ALERT pin function on the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  pinFunction Selected alert pin function mode.
  * @retval HAL status
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
  * @brief  Configure the active polarity of the physical ALERT pin on the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  polarity Selected alert pin polarity.
  * @retval HAL status
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
  * @brief  Configure the Therm/Alert mode on the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  tnA Selected mode of operation.
  * @retval HAL status
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
  * @brief  Configure the number of conversion averages for the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  avg Selected averaging mode.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Avg_TypeDef avg)
{
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
        
        regValue &= ~TMP117_AVG_Mask;
        regValue |= (avg << TMP117_AVG_Pos) & TMP117_AVG_Mask;
        return TMP117_WriteRegister(tmp117, TMP117_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the operating mode of the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  mode Selected operating mode.
  * @retval HAL status
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
  * @brief  Configure the conversion cycle time for the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  convTime Selected conversion cycle time.
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetConvTime(TMP117_HandleTypeDef *tmp117, TMP117_ConvTime_TypeDef convTime)
{
    double activeTime = tmp117->_samples * 15.5;
    tmp117->_activeTime = activeTime;
    double requestedTime = 0.0;
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
  * @brief  Set the temperature high limit threshold for comparison on the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  highLimit High limit temperature threshold in degrees Celsius (°C).
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetHighLimit_C(TMP117_HandleTypeDef *tmp117, double highLimit)
{
    int16_t regValue = (int16_t)(highLimit / 0.0078125);
    return TMP117_WriteRegister(tmp117, TMP117_TEMP_HIGH_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Set the temperature low limit threshold for comparison on the TMP117 sensor
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @param  lowLimit Low limit temperature threshold in degrees Celsius (°C).
  * @retval HAL status
  */
HAL_StatusTypeDef TMP117_SetLowLimit_C(TMP117_HandleTypeDef *tmp117, double lowLimit)
{
    int16_t regValue = (int16_t)(lowLimit / 0.0078125);
    return TMP117_WriteRegister(tmp117, TMP117_TEMP_LOW_LIMIT_REG, (uint16_t)regValue);
}

/**
  * @brief  Read the temperature measurement and calculate its value in degrees Celsius (°C)
  * @param  tmp117 Pointer to a TMP117_HandleTypeDef.
  * @retval The measured temperature in degrees Celsius (°C), or NAN if the read operation fails.
  */
double TMP117_GetTemperature_C(TMP117_HandleTypeDef *tmp117)
{
    uint16_t regValue = 0;
    if (TMP117_ReadRegister(tmp117, TMP117_TEMP_RESULT_REG, &regValue) == HAL_OK)
    {
        int16_t rawTemp = (int16_t)regValue;
        return (double)rawTemp * 0.0078125;
    }
    return NAN;
}
