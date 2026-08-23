#include "i2c_bus.h"
#include "math.h"
#include "stm32f3xx_hal_def.h"
#include "INA219.h"

/* =======================================================================================
   CAPA DE ADAPTACIÓN PRIVADA (WRAPPERS INTERNOS)
   ======================================================================================= */

/**
  * @brief  Write a 16-bit register value to the INA219 sensor via I2C
  * @note   This is an internal helper function. Transmits data in Big-Endian format.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  registerAddress Target register address on the INA219.
  * @param  value 16-bit value to be written.
  * @return HAL status
  */
static inline HAL_StatusTypeDef INA219_WriteRegister(INA219_HandleTypeDef *ina219, uint8_t registerAddress, uint16_t value)
{
    return I2C_Bus_WriteRegister16_BE(ina219->hi2c, ina219->_devAddress, registerAddress, value);
}

/**
  * @brief  Read a 16-bit register value from the INA219 sensor via I2C
  * @note   This is an internal helper function. Reads data in Big-Endian format.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  registerAddress Target register address on the INA219.
  * @param  value Pointer to store the read 16-bit value.
  * @return HAL status
  */
static inline HAL_StatusTypeDef INA219_ReadRegister(INA219_HandleTypeDef *ina219, uint8_t registerAddress, uint16_t *value)
{
    return I2C_Bus_ReadRegister16_BE(ina219->hi2c, ina219->_devAddress, registerAddress, value);
}

/**
  * @brief  Round the minimum Current LSB to the next clean 1-2-5 step of a power of 10
  * @note   This is an internal helper function. It implements a standard 1-2-5 rounding 
  *         rule (e.g. 10uA, 20uA, 50uA, 100uA) to select a user-friendly Current LSB.
  *         The selected round LSB is guaranteed to satisfy the datasheet constraint:
  *         lsbMin <= roundedLsb < 2.5 * lsbMin (well below the 8x limit).
  * @param  lsbMin The calculated absolute minimum Current LSB (in Amperes/LSB).
  * @return The rounded, user-friendly Current LSB value (in Amperes).
  */
static float INA219_RoundCurrentLsb(float lsbMin)
{

    if (lsbMin <= 0.0f)
    {
        return 0.0f;
    }

    // Find power of 10 below lsbMin
    float logLsb = log10f(lsbMin);
    float powerOf10 = powf(10.0f, floorf(logLsb));
    
    // Normalize to a value between 1.0 and 10.0
    float normalized = lsbMin / powerOf10;
    float roundedLsb;
    
    // Round up to the nearest 1, 2, or 5 step
    if (normalized <= 1.0f)
    {
        roundedLsb = 1.0f * powerOf10;
    }
    else if (normalized <= 2.0f)
    {
        roundedLsb = 2.0f * powerOf10;
    }
    else if (normalized <= 5.0f)
    {
        roundedLsb = 5.0f * powerOf10;
    }
    else
    {
        roundedLsb = 10.0f * powerOf10;
    }
    
    return roundedLsb;
}

/* =======================================================================================
   CAPA DE APLICACIÓN 
   ======================================================================================= */

/**
  * @brief  Initialize the INA219 device handle and configure default parameter values
  * @note   This function sets up the hi2c handler and sets the default conversion step sizes
  *         for shunt voltage (10 uV) and bus voltage (4 mV) registers.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  i2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress The 7-bit physical I2C device address.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *ina219, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    ina219->hi2c = i2c;
    ina219->_devAddress = devAddress;
    ina219->_vShuntAcc = 0.00001f; // Shunt Voltage, 1 LSB Step size = 10 uV
    ina219->_vBusAcc =  0.004f;    // Bus Voltage, 1 LSB step size = 4 mV

    // Reset the device to ensure a clean default state
    if (INA219_ResetDevice(ina219) != HAL_OK)
    {
        return HAL_ERROR;
    }

    uint16_t config = 0;
    config |= (INA219_BUSVOLTAGERANGE_32V      << INA219_BRGN_Pos) & INA219_BRGN;
    config |= (INA219_PGAGAIN_320_MILI_VOLT    << INA219_PG_Pos)   & INA219_PG;
    config |= (INA219_ADC_128_SAMPLES          << INA219_BADC_Pos) & INA219_BADC;
    config |= (INA219_ADC_128_SAMPLES          << INA219_SADC_Pos) & INA219_SADC;
    config |= (INA219_SHUNTBUS_CONTINUOUS_MODE << INA219_MODE_Pos) & INA219_MODE;

    return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, config);
}

/**
  * @brief  Read the current raw value from the Configuration register
  * @note   The Configuration register contains settings for operation mode, SADC, BADC, PG, and BRNG.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit configuration register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetConfiguration(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_CONFIGURATION_REG, value);
}

/**
  * @brief  Read the current raw value from the Shunt Voltage register
  * @note   Stores the measurement drop across the shunt resistor. 1 LSB = 10 uV.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit raw shunt voltage register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetShuntVoltage(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_SHUNTVOLTAGE_REG, value);
}

/**
  * @brief  Read the current raw value from the Bus Voltage register
  * @note   Stores the bus voltage measurement data. Bits 15-3 are the value, 1 LSB = 4 mV.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit raw bus voltage register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetBusVoltage(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_BUSVOLTAGE_REG, value);
}

/**
  * @brief  Read the current raw value from the Power register
  * @note   Stores the raw calculated load power value. Power LSB = 20 * Current LSB.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit raw power register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetPower(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_POWER_REG, value);
}

/**
  * @brief  Read the current raw value from the Current register
  * @note   Stores the raw calculated load current value. 1 LSB = Current LSB.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit raw current register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetCurrent(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_CURRENT_REG, value);
}

/**
  * @brief  Read the current raw value from the Calibration register
  * @note   Stores the calibration value used to program the Current LSB and Power LSB.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  value Pointer to store the 16-bit raw calibration register value.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_GetCalibration(INA219_HandleTypeDef *ina219, uint16_t *value)
{
    return INA219_ReadRegister(ina219, INA219_CALIBRATION_REG, value);
}

/**
  * @brief  Configure the operating mode of the INA219 sensor
  * @note   This function modifies the MODE bits in the Configuration register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  mode Selected operating mode (e.g. continuous, triggered, power-down).
  *         This parameter can be one of the following values:
  *         @arg INA219_POWERDOWN_MODE
  *         @arg INA219_SHUNTVOLTAGETRIG_MODE
  *         @arg INA219_BUSVOLTAGETRIG_MODE
  *         @arg INA219_SHUNTBUS_TRIG_MODE
  *         @arg INA219_ADC_OFF_DISABLED_MODE
  *         @arg INA219_SHUNTVOLTAGE_CONTINUOUS_MODE
  *         @arg INA219_BUSVOLTAGE_CONTINUOUS_MODE
  *         @arg INA219_SHUNTBUS_CONTINUOUS_MODE
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetMode(INA219_HandleTypeDef *ina219, INA219_Mode_TypeDef mode)
{
    uint16_t regValue = 0;

    if (INA219_GetConfiguration(ina219, &regValue) == HAL_OK)
    {
        regValue &= ~INA219_MODE;
        regValue |= (mode << INA219_MODE_Pos) & INA219_MODE;
        return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the ADC resolution or averaging for the shunt voltage measurement
  * @note   This function modifies the SADC bits in the Configuration register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  shuntADCResolution Selected resolution or number of averages.
  *         This parameter can be one of the following values:
  *         @arg INA219_ADC_9_BIT_RESOLUTION
  *         @arg INA219_ADC_10_BIT_RESOLUTION
  *         @arg INA219_ADC_11_BIT_RESOLUTION
  *         @arg INA219_ADC_12_BIT_RESOLUTION
  *         @arg INA219_ADC_2_SAMPLES
  *         @arg INA219_ADC_4_SAMPLES
  *         @arg INA219_ADC_8_SAMPLES
  *         @arg INA219_ADC_16_SAMPLES
  *         @arg INA219_ADC_32_SAMPLES
  *         @arg INA219_ADC_64_SAMPLES
  *         @arg INA219_ADC_128_SAMPLES
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetShuntADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution_TypeDef shuntADCResolution)
{
    uint16_t regValue = 0;

    if (INA219_GetConfiguration(ina219, &regValue) == HAL_OK)
    {
        regValue &= ~INA219_SADC;
        regValue |= (shuntADCResolution << INA219_SADC_Pos) & INA219_SADC;
        return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the ADC resolution or averaging for the bus voltage measurement
  * @note   This function modifies the BADC bits in the Configuration register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  busADCResolution Selected resolution or number of averages.
  *         This parameter can be one of the following values:
  *         @arg INA219_ADC_9_BIT_RESOLUTION
  *         @arg INA219_ADC_10_BIT_RESOLUTION
  *         @arg INA219_ADC_11_BIT_RESOLUTION
  *         @arg INA219_ADC_12_BIT_RESOLUTION
  *         @arg INA219_ADC_2_SAMPLES
  *         @arg INA219_ADC_4_SAMPLES
  *         @arg INA219_ADC_8_SAMPLES
  *         @arg INA219_ADC_16_SAMPLES
  *         @arg INA219_ADC_32_SAMPLES
  *         @arg INA219_ADC_64_SAMPLES
  *         @arg INA219_ADC_128_SAMPLES
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetBusADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution_TypeDef busADCResolution)
{
    uint16_t regValue = 0;

    if (INA219_GetConfiguration(ina219, &regValue) == HAL_OK)
    {
        regValue &= ~INA219_BADC;
        regValue |= (busADCResolution << INA219_BADC_Pos) & INA219_BADC;
        return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the PGA gain range for the shunt voltage measurement
  * @note   This function modifies the PG bits in the Configuration register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  shuntVoltageRange Selected PGA gain (e.g. 40mV, 80mV, 160mV, 320mV).
  *         This parameter can be one of the following values:
  *         @arg INA219_PGAGAIN_40_MILI_VOLT
  *         @arg INA219_PGAGAIN_80_MILI_VOLT
  *         @arg INA219_PGAGAIN_160_MILI_VOLT
  *         @arg INA219_PGAGAIN_320_MILI_VOLT
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetShuntVoltageRange(INA219_HandleTypeDef *ina219, INA219_ShuntVoltagePGA_TypeDef shuntVoltageRange)
{
    uint16_t regValue = 0;

    if (INA219_GetConfiguration(ina219, &regValue) == HAL_OK)
    {
        regValue &= ~INA219_PG;
        regValue |= (shuntVoltageRange << INA219_PG_Pos) & INA219_PG;
        return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Configure the bus voltage full-scale range on the INA219 sensor
  * @note   This function modifies the BRNG bit in the Configuration register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  busVoltageRange Selected range (16V or 32V).
  *         This parameter can be one of the following values:
  *         @arg INA219_BUSVOLTAGERANGE_16V
  *         @arg INA219_BUSVOLTAGERANGE_32V
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetBusVoltageRange(INA219_HandleTypeDef *ina219, INA219_BusVoltageRange_TypeDef busVoltageRange)
{
    uint16_t regValue = 0;

    if (INA219_GetConfiguration(ina219, &regValue) == HAL_OK)
    {
        regValue &= ~INA219_BRGN;
        regValue |= (busVoltageRange << INA219_BRGN_Pos) & INA219_BRGN;
        return INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, regValue);
    }
    return HAL_ERROR;
}

/**
  * @brief  Reset the INA219 registers to their default factory values
  * @note   This function writes to the RST bit and polls until the reset completes.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_ResetDevice(INA219_HandleTypeDef *ina219)
{
    HAL_StatusTypeDef status = INA219_WriteRegister(ina219, INA219_CONFIGURATION_REG, INA219_RST);

    if (status != HAL_OK)
    {
        return status;
    }

    uint16_t regValue = INA219_RST;
    uint32_t startTick = HAL_GetTick();
    const uint32_t timeout = 100;

    do {
        status = INA219_ReadRegister(ina219, INA219_CONFIGURATION_REG, &regValue);
        if (status != HAL_OK)
        {
            return status;
        }

        if (!(regValue & INA219_RST)) // if RESET bit = 0
        {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - startTick) < timeout);

    return HAL_TIMEOUT;
}

/**
  * @brief  Calculate and write the Calibration register value based on the shunt resistor and max current
  * @note   This function implements the mathematical equations of the INA219 datasheet.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  rShuntValue Value of the shunt resistor in Ohms.
  * @param  maxCurrent Maximum expected current in Amperes.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetCalibration(INA219_HandleTypeDef *ina219, float rShuntValue, float maxCurrent)
{
    if (ina219 == NULL || maxCurrent <= 0.0f || rShuntValue <= 0.0f)
    {
        return HAL_ERROR;
    } 

    // Avoid attempting to measure a current that your shunt cannot handle without saturating the ADC.

    float safeCurrentLimit = 0.32f / rShuntValue;
    if (maxCurrent > safeCurrentLimit)
    {
        return HAL_ERROR;
    }
   
    ina219->_shuntResistor = rShuntValue;
    ina219->_maximumCurrent = maxCurrent;

    float currentLsbMinimum = maxCurrent / 32768.0f;
    ina219->_currentLsbMin = currentLsbMinimum;

    float roundedLsb = INA219_RoundCurrentLsb(currentLsbMinimum);
    ina219->_currentLsb = roundedLsb;

    // Ensure that the virtual LSB is not set too small, as this may produce values exceeding 65535
    // leading to truncation and corrupted sensor data
    float calValue = 0.04096f / (roundedLsb * rShuntValue);
    if (calValue > 65535.0f)
    {
        return HAL_ERROR; 
    }

    uint16_t shuntCal = (uint16_t)calValue;

    return INA219_WriteRegister(ina219, INA219_CALIBRATION_REG, shuntCal);
}

/**
  * @brief  Read the shunt voltage and convert it to millivolts
  * @note   Uses the struct variable _vShuntAcc (which is in Volts) multiplied by 1000.0f to output mV.
  *         This register stores a signed 2's complement value.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  shuntVoltage_mV Pointer to store the calculated shunt voltage in mV.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_ReadShuntVoltage_mV(INA219_HandleTypeDef *ina219, float *shuntVoltage_mV)
{
    if (shuntVoltage_mV == NULL) return HAL_ERROR;
    
    uint16_t regValue = 0;
    HAL_StatusTypeDef status = INA219_GetShuntVoltage(ina219, &regValue);
    
    if (status == HAL_OK)
    {
        int16_t signedValue = (int16_t)regValue;
        *shuntVoltage_mV = (float)signedValue * ina219->_vShuntAcc * 1000.0f;
    }
    return status;
}

/**
  * @brief  Read the bus voltage and convert it to Volts
  * @note   Uses the struct variable _vBusAcc (which is in Volts) directly.
  *         The raw bus voltage value is stored in bits 15-3.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  busVoltage_V Pointer to store the calculated bus voltage in Volts.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_ReadBusVoltage_V(INA219_HandleTypeDef *ina219, float *busVoltage_V)
{
    if (busVoltage_V == NULL) return HAL_ERROR;
    
    uint16_t regValue = 0;
    HAL_StatusTypeDef status = INA219_GetBusVoltage(ina219, &regValue);
    
    if (status == HAL_OK)
    {
        regValue = regValue >> INA219_BD_Pos;
        *busVoltage_V = (float)regValue * ina219->_vBusAcc;
    }
    return status;
}

/**
  * @brief  Read the current flowing through the shunt resistor in Amperes
  * @note   Current is calculated by multiplying the raw signed value by the programmed Current LSB.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  current_A Pointer to store the calculated current in Amperes.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_ReadCurrent_A(INA219_HandleTypeDef *ina219, float *current_A)
{
    if (current_A == NULL) return HAL_ERROR;
    
    uint16_t regValue = 0;
    HAL_StatusTypeDef status = INA219_GetCurrent(ina219, &regValue);
    
    if (status == HAL_OK)
    {
        int16_t signedValue = (int16_t)regValue;
        *current_A = (float)signedValue * ina219->_currentLsb;
    }
    return status;
}

/**
  * @brief  Read the calculated load power in Watts
  * @note   Power is calculated using the internal multiplier: Power = RawPower * 20 * Current_LSB.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  power_W Pointer to store the calculated power in Watts.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_ReadPower_W(INA219_HandleTypeDef *ina219, float *power_W)
{
    if (power_W == NULL) return HAL_ERROR;
    
    uint16_t regValue = 0;
    HAL_StatusTypeDef status = INA219_GetPower(ina219, &regValue);
    
    if (status == HAL_OK)
    {
        // Power is an unsigned 16-bit register.
        *power_W = (float)regValue * ina219->_currentLsb * 20.0f;
    }
    return status;
}

/**
  * @brief  Check if the ADC has completed the conversion and new data is ready
  * @note   Reads bit 1 (CNVR) from the Bus Voltage register.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @return Boolean status (true if data is ready, false otherwise).
  */
_Bool INA219_IsDataReady(INA219_HandleTypeDef *ina219)
{
    uint16_t regValue = 0;
    if (INA219_ReadRegister(ina219, INA219_BUSVOLTAGE_REG, &regValue) == HAL_OK)
    {
        return (regValue & INA219_CNVR_Mask) != 0; // Bit 1 is CNVR
    }
    return 0;
}

/**
  * @brief  Apply a corrected calibration value to adjust system offset and gain errors
  * @note   Uses the expected current and measured current from external instruments to calibrate.
  * @param  ina219 Pointer to a INA219_HandleTypeDef structure.
  * @param  calValue The base calculated calibration value.
  * @param  expectedCurrent Expected current value in Amperes.
  * @param  measuredCurrent Measured current value in Amperes from a multimeter.
  * @return HAL status
  */
HAL_StatusTypeDef INA219_SetCorrectedCalibration(INA219_HandleTypeDef *ina219, uint16_t calValue, float expectedCurrent, float measuredCurrent)
{
    if (expectedCurrent == 0.0f)
    {
        return HAL_ERROR;
    }
    
    uint16_t correctedFullScaleCal = (uint16_t)((float)calValue * measuredCurrent / expectedCurrent);
    return INA219_WriteRegister(ina219, INA219_CALIBRATION_REG, correctedFullScaleCal);
}
