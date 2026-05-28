#include "main.h"
#include "math.h"
#include "INA236.h"
#include <stdio.h>

/* Static helper prototypes */
static double INA236_RoundCurrentLsb(double lsbMin);

/**
  * @brief  Write a 16-bit value to a specific register of the INA236 device
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  registerAddress Internal register address of the INA236 to write to (e.g., 0x00, 0x05)
  * @param  value The 16-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef INA236_WriteRegister(INA236_HandleTypeDef *ina236, uint8_t registerAddress, uint16_t value)
{
  uint8_t address[2];
  HAL_StatusTypeDef isDeviceReady;
  // Split the 16-bit value into two bytes (MSB first)
  address[0] = (value >> 8) & 0xFF;
  address[1] = (value >> 0) & 0xFF;
  
  // Check if the device is ready on the I2C bus
  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->_devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);
  if (isDeviceReady == HAL_OK)
  {
    // Write the 16-bit register to the device
    if (HAL_I2C_Mem_Write(ina236->hi2c, (ina236->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY) == HAL_OK)
    {
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

/**
  * @brief  Read a 16-bit value from a specific register of the INA236 device
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  registerAddress Internal register address of the INA236 to read from (e.g., 0x00, 0x01)
  * @return 16-bit data read from register, or 0xFFFF if the operation fails
  */
uint16_t INA236_ReadRegister(INA236_HandleTypeDef *ina236, uint8_t registerAddress)
{
  uint8_t registerResponse[2] = {0}; 
  HAL_StatusTypeDef isDeviceReady;   

  // Check if the device is ready on the I2C bus
  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->_devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    // Read the 2-byte register data from the device
    if (HAL_I2C_Mem_Read(ina236->hi2c, (ina236->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) == HAL_OK)
    {
        // Combine MSB and LSB to return the 16-bit value
        return (uint16_t)((registerResponse[0] << 8) | registerResponse[1]);
    }    
  }
  // Returns a maximum control value (0xFFFF) indicating a communication error
  return 0xFFFF;
}


/**
  * @brief  Initializes the INA236 device handler and configures its registers
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure (HAL I2C handler).
  * @param  devAddress Target device 7-bit I2C address.
  * @retval HAL_OK: Device was successfully initialized and responded to connection check
  * @retval HAL_ERROR: Device did not respond or configuration failed
  */
HAL_StatusTypeDef INA236_Init(INA236_HandleTypeDef *ina236, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
  // Bind physical communication parameters to local handler
  ina236->hi2c = i2c;
  ina236->_devAddress = devAddress;

  // 1. Verify if the sensor is physically responding on the I2C bus
  if (HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->_devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  // 2. Configure default settings
  INA236_SetMode(ina236, INA236_CONTINUOUS_SHUNT_BUS_VOLTAGE);
  INA236_SetShuntConvTime(ina236, INA236_1100_US);
  INA236_SetBusConvTime(ina236, INA236_1100_US);
  INA236_SetAverage(ina236, INA236_64_SAMPLES);
  INA236_SetAdcRange(ina236, INA236_ADC_RANGE_81_92_MV);

  return HAL_OK;
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
static double INA236_RoundCurrentLsb(double lsbMin)
{
    // Find power of 10 below lsbMin
    double logLsb = log10(lsbMin);
    double powerOf10 = pow(10, floor(logLsb));
    
    // Normalize to a value between 1.0 and 10.0
    double normalized = lsbMin / powerOf10;
    double roundedLsb;
    
    // Round up to the nearest 1, 2, or 5 step
    if (normalized <= 1.0)
    {
        roundedLsb = 1.0 * powerOf10;
    }
    else if (normalized <= 2.0)
    {
        roundedLsb = 2.0 * powerOf10;
    }
    else if (normalized <= 5.0)
    {
        roundedLsb = 5.0 * powerOf10;
    }
    else
    {
        roundedLsb = 10.0 * powerOf10;
    }
    
    return roundedLsb;
}

/**
  * @brief  Calculate and set the calibration value for the INA236 sensor
  * @note   This function calculates the register calibration value based on the shunt resistor 
  *         and the maximum expected current. It programs the INA236_CALIBRATION_REGISTER,
  *         allowing the device to automatically calculate load current and power.
  *         If ADCRANGE = 1 (±20.48 mV), the calibration value is automatically scaled down by 4.
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  rShuntValue Shunt resistor value connected to the chip (in Ohms)
  * @param  maxCurrent Maximum expected load current to be monitored (in Amperes)
  * @retval None
  */
void INA236_SetCalibration(INA236_HandleTypeDef *ina236, double rShuntValue, int maxCurrent)
{
    ina236->_shuntResistor = rShuntValue;
    ina236->_maximumCurrent = (double)maxCurrent;

    // Calculate the minimum Current LSB: Max_Current / 2^15 (2^15 = 32768)
    double currentLsbMinimum = (double)maxCurrent / 32768.0; 
    ina236->_currentLsbMin = currentLsbMinimum;

    // Auto-calculate a clean, round LSB (always satisfies: lsbMin <= lsb < 2.5 * lsbMin)
    double roundedLsb = INA236_RoundCurrentLsb(currentLsbMinimum);
    ina236->_currentLsb = roundedLsb;

    // Calculate the Calibration register value: 0.00512 / (Current_LSB * Rshunt)
    uint16_t shuntCal = 0.00512 / (roundedLsb * rShuntValue);

    // According to datasheet, SHUNT_CAL must be divided by 4 for ADCRANGE = 1 (±20.48 mV)
    if (ina236->_adcRange == 1)
    {
        shuntCal = shuntCal / 4;
    }

    INA236_WriteRegister(ina236, INA236_CALIBRATION_REGISTER, shuntCal);
}

/**
  * @brief  Configure the operating mode of the INA236 device
  * @param  handler Pointer to a INA236_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified INA236.
  * @param  mode Selected operating mode. This parameter can be one of the following values:
  *         @arg INA236_SHUTDOWN: Shutdown mode (0x0)
  *         @arg INA236_SHUNT_VOLTAGE_ONE_SHOT: Shunt voltage triggered mode (0x1)
  *         @arg INA236_BUS_VOLTAGE_ONE_SHOT: Bus voltage triggered mode (0x2)
  *         @arg INA236_SHUNT_BUS_VOLTAGE_ONE_SHOT: Shunt and Bus voltage triggered mode (0x3)
  *         @arg INA236_CONTINUOUS_SHUNT_VOLTAGE: Continuous shunt voltage mode (0x5)
  *         @arg INA236_CONTINUOUS_BUS_VOLTAGE: Continuous bus voltage mode (0x6)
  *         @arg INA236_CONTINUOUS_SHUNT_BUS_VOLTAGE: Continuous shunt and bus voltage mode (0x7)
  * @retval None
  */
void INA236_SetMode(INA236_HandleTypeDef *ina236, INA236_Mode_TypeDef mode)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

    // Clear the MODE bits using the mask (bits 0, 1 and 2)
    regValue &= ~INA236_MODE_Mask;

    // Set the new mode by shifting it to its position (MODE_Pos = 0)
    // and protecting it with the mask
    regValue |= (mode << INA236_MODE_Pos) & INA236_MODE_Mask;

    //Write the resulting value back to the register
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, regValue);
}

/**
  * @brief  Configure the conversion time for the shunt voltage measurement
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  convTime Selected conversion time to be set in the configuration register.
  *                  This parameter can be one of the following values:
  *                  @arg INA236_140_US: 140 microseconds (0x0)
  *                  @arg INA236_204_US: 204 microseconds (0x1)
  *                  @arg INA236_332_US: 332 microseconds (0x2)
  *                  @arg INA236_588_US: 588 microseconds (0x3)
  *                  @arg INA236_1100_US: 1100 microseconds (0x4)
  *                  @arg INA236_2116_US: 2116 microseconds (0x5)
  *                  @arg INA236_4156_US: 4156 microseconds (0x6)
  *                  @arg INA236_8244_US: 8244 microseconds (0x7)
  * @retval None
  */
void INA236_SetShuntConvTime(INA236_HandleTypeDef *ina236, INA236_ConvTime_TypeDef convTime)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

    // Clear the VSHCT bits using the mask (bits 3, 4 and 5)
    regValue &= ~INA236_VSHCT_Mask;

    // Set the new VSHCT by shifting it to its position (VSHCT_Pos = 3)
    // and protecting it with the mask
    regValue |= (convTime << INA236_VSHCT_Pos) & INA236_VSHCT_Mask;

    //Write the resulting value back to the register
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, regValue);
}

/**
  * @brief  Configure the conversion time for the bus voltage measurement
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  convTime Selected conversion time to be set in the configuration register
  *                  This parameter can be one of the following values:
  *                  @arg INA236_140_US: 140 microseconds (0x0)
  *                  @arg INA236_204_US: 204 microseconds (0x1)
  *                  @arg INA236_332_US: 332 microseconds (0x2)
  *                  @arg INA236_588_US: 588 microseconds (0x3)
  *                  @arg INA236_1100_US: 1100 microseconds (0x4)
  *                  @arg INA236_2116_US: 2116 microseconds (0x5)
  *                  @arg INA236_4156_US: 4156 microseconds (0x6)
  *                  @arg INA236_8244_US: 8244 microseconds (0x7)
  * @retval None
  */
void INA236_SetBusConvTime(INA236_HandleTypeDef *ina236, INA236_ConvTime_TypeDef convTime)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

    // Clear the VBUSCT bits using the mask (bits 6, 7 and 8)
    regValue &= ~INA236_VBUSCT_Mask;

    // Set the new VBUSCT by shifting it to its position (VBUSCT_Pos = 6)
    // and protecting it with the mask
    regValue |= (convTime << INA236_VBUSCT_Pos) & INA236_VBUSCT_Mask;

    //Write the resulting value back to the register
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, regValue);
}
/**
  * @brief  Configure the number of samples averaged for the measurements
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  avg Selected sample averaging count to be set in the configuration register.
  *             This parameter can be one of the following values:
  *             @arg INA236_1_SAMPLE: 1 sample (0x0)
  *             @arg INA236_4_SAMPLES: 4 samples (0x1)
  *             @arg INA236_16_SAMPLES: 16 samples (0x2)
  *             @arg INA236_64_SAMPLES: 64 samples (0x3)
  *             @arg INA236_128_SAMPLES: 128 samples (0x4)
  *             @arg INA236_256_SAMPLES: 256 samples (0x5)
  *             @arg INA236_512_SAMPLES: 512 samples (0x6)
  *             @arg INA236_1014_SAMPLES: 1024 samples (0x7)
  * @retval None
  */
void INA236_SetAverage(INA236_HandleTypeDef *ina236, INA236_Avg_TypeDef avg)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

    // Clear the AVG bits using the mask (bits 9, 10 and 11)
    regValue &= ~INA236_AVG_Mask;

    // Set the new AVG by shifting it to its position (AVG_Pos = 9) (FIXED)
    // and protecting it with the mask
    regValue |= (avg << INA236_AVG_Pos) & INA236_AVG_Mask;

    // Write the resulting value back to the register
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, regValue);
}

/**
  * @brief  Configure the shunt voltage ADC full-scale range
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  range Selected ADC input range. This parameter can be one of the following:
  *             @arg INA236_ADC_RANGE_81_92MV: Range is ±81.92 mV (LSB = 2.5 uV)
  *             @arg INA236_ADC_RANGE_20_48MV: Range is ±20.48 mV (LSB = 0.625 uV)
  * @retval None
  */
void INA236_SetAdcRange(INA236_HandleTypeDef *ina236, INA236_AdcRange_TypeDef range)
{
    // Read current register
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

    // Clear ADCRANGE bit (bit 12)
    regValue &= ~INA236_ADCRANGE_Mask;

    // Set the new range in bit 12
    regValue |= (range << INA236_ADCRANGE_Pos) & INA236_ADCRANGE_Mask;

    // Write back to register
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, regValue);
    
    // Update local struct parameters 
    ina236->_adcRange = (_Bool)range;

    // Update the Shunt ADC limit in Volts (V) for physical SI calculations
    if (range == INA236_ADC_RANGE20_48_MV)
    {
        ina236->_shuntAdcRange = 0.02048;   // ±20.48 mV
        ina236->_resolution = 0.000000625;  // 625 nV/LSB 
    }
    else
    {
        ina236->_shuntAdcRange = 0.08192;   // ±81.92 mV
        ina236->_resolution = 0.0000025;    // 2.5 uV/LSB
    }
}

/**
  * @brief  Reset the INA236 device registers to their default factory values
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @retval None
  */
void INA236_ResetDevice(INA236_HandleTypeDef *ina236)
{
    // Escribir directamente el bit de reset (bit 15 = 0x8000) en el registro de configuracion.
    // El bit RST se limpia solo (self-clearing) inmediatamente despues de ejecutar el reset.
    INA236_WriteRegister(ina236, INA236_CONFIGURATION_REGISTER, INA236_RST_Mask);
    
    // Esperar un breve retraso para asegurar que los circuitos internos del chip 
    // y la comunicacion I2C se hayan estabilizado tras el reinicio.
    HAL_Delay(2); 
    
    // Tras el reset, el chip vuelve a su rango de ADC predeterminado de +-81.92 mV.
    ina236->_adcRange = INA236_ADC_RANGE_81_92_MV;
}

/**
  * @brief  Retrieve the Manufacturer ID from the INA236 device
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return 16-bit Manufacturer ID (Expected value is 0x5449), or 0xFFFF if reading fails
  */
uint16_t INA236_GetManufacturerID(INA236_HandleTypeDef *ina236)
{
    // Read the Manufacturer ID register (Address: 0x3E)
    uint16_t regValue = INA236_ReadRegister(ina236, MANUFACTURER_ID_REGISTER);

    return regValue;
}

/**
  * @brief  Retrieve the Device ID from the INA236 device
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return 16-bit Device ID (Expected value is 0xA080), or 0xFFFF if reading fails
  */
uint16_t INA236_GetDeviceID(INA236_HandleTypeDef *ina236)
{
    // Read the Manufacturer ID register (Address: 0x3F)
    uint16_t regValue = INA236_ReadRegister(ina236, DEVICE_ID_REGISTER);

    return regValue;
}

/**
  * @brief  Check if an arithmetic overflow error occurred (OVF bit is set)
  * @note   This flag indicates that the current and power data calculation has overflowed
  *         and the values in those registers may be invalid.
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @retval 1: Math overflow occurred (data may be invalid)
  *         0: No overflow occurred or read error
  */
uint8_t INA236_IsMathOverflowReady(INA236_HandleTypeDef *ina236)
{
    // Read the Mask/Enable Register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Safety check: If read fails (returns 0xFFFF), return 0
    if (regValue == 0xFFFF)
    {
        return 0;
    }

    // Mask the value to isolate bit 2 (OVF - Math Overflow Flag)
    // If the bit is set, (regValue & INA236_OVF) will be 0x004 (which is != 0)
    if ((regValue & INA236_OVF) != 0U)
    {
        return 1; // Bit is 1 (Math Overflow occurred)
    }
    
    return 0; // Bit is 0
}

/**
  * @brief  Check if a new conversion is ready (CVRF bit is set)
  * @note   Conversion Ready Flag bit clears under the following conditions:
  *         1) Writing the configuration register.
  *         2) Reading the Mask/Enable register. 
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @retval 1: New conversion is ready and data is available
  *         0: Conversion is not ready or read error occurred
  */
uint8_t INA236_IsConversionReady(INA236_HandleTypeDef *ina236)
{
    // Read the Mask/Enable Register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Safety check: If read fails (returns 0xFFFF), return 0
    if (regValue == 0xFFFF)
    {
        return 0;
    }

    // Mask the value to isolate bit 10 (CNVR_Mask)
    // If the bit is set, (regValue & INA236_CVRF) will be 0x0008 (which is != 0)
    if ((regValue & INA236_CVRF) != 0U)
    {
        return 1; // Bit is 1 (Conversion Ready)
    }
    
    return 0; // Bit is 0
}

/**
  * @brief  Configure the alert pin polarity (active-low or active-high)
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  polarity Selected alert pin polarity (Normal or Inverted)
  *             @arg INA236_ACTIVE_LOW (Normal: Active-Low open drain)
  *             @arg INA236_ACTIVE_HIGH (Inverted: Active High)
  * @retval None
  */
void INA236_SetAlertPolarity(INA236_HandleTypeDef *ina236, INA236_AlertPol_TypeDef polarity)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Clear the APOL bit using the mask (bit 1)
    regValue &= ~INA236_APOL_Mask;

    // Set the new mode by shifting it to its position (MODE_Pos = 1)
    // and protecting it with the mask
    regValue |= (polarity << INA236_APOL_Pos) & INA236_APOL_Mask;

    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable physical ALERT pin assertion on Conversion Ready event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  cnvr Selected configuration for the Conversion Ready Alert pin assertion.
  *              This parameter can be one of the following:
  *              @arg INA236_CONV_READY_ALERT_DISABLE: ALERT pin does not assert on conversion ready
  *              @arg INA236_CONV_READY_ALERT_ENABLE: ALERT pin asserts when conversion is ready
  * @retval None
  */
void INA236_SetAlertPin(INA236_HandleTypeDef *ina236, INA236_ConvRdy_TypeDef cnvr)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Clear the CNVR bit using the mask (bit 10)
    regValue &= ~INA236_CNVR_Mask;

    // Set the new mode by shifting it to its position (CNVR_Pos = 10)
    // and protecting it with the mask
    regValue |= (cnvr << INA236_CNVR_Pos) & INA236_CNVR_Mask;

    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable the physical ALERT pin assertion on a Power Over Limit event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @param  powerAlert Selected status for the Power Over Limit alert pin assertion.
  *                    This parameter can be one of the following values:
  *                    @arg INA236_POWER_LIMIT_ALERT_DISABLE: ALERT pin does not assert on power over limit (0x0)
  *                    @arg INA236_POWER_LIMIT_ALERT_ENABLE: ALERT pin asserts when power exceeds the Alert Limit register (0x1)
  * @retval None
  */
void INA236_SetPowerOverLimit(INA236_HandleTypeDef *ina236, INA236_PowerAlert_TypeDef powerAlert)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Clear the POL bit using the mask (bit 11)
    regValue &= ~INA236_POL_Mask;

    // Set the new mode by shifting it to its position (POL_Pos = 11)
    // and protecting it with the mask
    regValue |= (powerAlert << INA236_POL_Pos) & INA236_POL_Mask;

    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable the physical ALERT pin assertion on a Bus Under Limit event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  bulAlert Selected status for the Bus Under Limit alert.
  *                  This parameter can be one of the following values:
  *                  @arg INA236_BUS_UNDER_LIMIT_ALERT_DISABLE: ALERT pin does not assert on bus under limit (0x0)
  *                  @arg INA236_BUS_UNDER_LIMIT_ALERT_ENABLE: ALERT pin asserts when bus voltage is below the Alert Limit (0x1)
  * @retval None
  */
void INA236_SetBusUnderLimit(INA236_HandleTypeDef *ina236, INA236_BusUnderLimitAlert_TypeDef bulAlert)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Clear the BUL bit using the mask (bit 12)
    regValue &= ~INA236_BUL_Mask;

    // Set the new mode by shifting it to its position (BUL_Pos = 12)
    // and protecting it with the mask
    regValue |= (bulAlert << INA236_BUL_Pos) & INA236_BUL_Mask;

    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable the physical ALERT pin assertion on a Bus Over Limit event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  bolAlert Selected status for the Bus Over Limit alert.
  *                  This parameter can be one of the following values:
  *                  @arg INA236_BUS_OVER_LIMIT_ALERT_DISABLE: ALERT pin does not assert on bus over limit (0x0)
  *                  @arg INA236_BUS_OVER_LIMIT_ALERT_ENABLE: ALERT pin asserts when bus voltage is above the Alert Limit (0x1)
  * @retval None
  */
void INA236_SetBusOverLimit(INA236_HandleTypeDef *ina236, INA236_BusOverLimitAlert_TypeDef bolAlert)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);
    // Clear the BOL bit using the mask (bit 13)
    regValue &= ~INA236_BOL_Mask;
    // Set the new configuration by shifting it to its position (BOL_Pos = 13)
    // and protecting it with the mask
    regValue |= (bolAlert << INA236_BOL_Pos) & INA236_BOL_Mask;
    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable the physical ALERT pin assertion on a Shunt Under Limit event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  sulAlert Selected status for the Shunt Under Limit alert.
  *                  This parameter can be one of the following values:
  *                  @arg INA236_SHUNT_UNDER_LIMIT_ALERT_DISABLE: ALERT pin does not assert on shunt under limit (0x0)
  *                  @arg INA236_SHUNT_UNDER_LIMIT_ALERT_ENABLE: ALERT pin asserts when shunt voltage is below the Alert Limit (0x1)
  * @retval None
  */
void INA236_SetShuntUnderLimit(INA236_HandleTypeDef *ina236, INA236_ShuntUnderLimitAlert_TypeDef sulAlert)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);
    // Clear the SUL bit using the mask (bit 14)
    regValue &= ~INA236_SUL_Mask;
    // Set the new configuration by shifting it to its position (SUL_Pos = 14)
    // and protecting it with the mask
    regValue |= (sulAlert << INA236_SUL_Pos) & INA236_SUL_Mask;
    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}

/**
  * @brief  Enable or disable the physical ALERT pin assertion on a Shunt Over Limit event
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @param  solAlert Selected status for the Shunt Over Limit alert.
  *                  This parameter can be one of the following values:
  *                  @arg INA236_SHUNT_OVER_LIMIT_ALERT_DISABLE: ALERT pin does not assert on shunt over limit (0x0)
  *                  @arg INA236_SHUNT_OVER_LIMIT_ALERT_ENABLE: ALERT pin asserts when shunt voltage is above the Alert Limit (0x1)
  * @retval None
  */
void INA236_SetShuntOverLimit(INA236_HandleTypeDef *ina236, INA236_ShuntOverLimitAlert_TypeDef solAlert)
{
    // Read the current MASK_ENABLE register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);
    // Clear the SOL bit using the mask (bit 15)
    regValue &= ~INA236_SOL_Mask;
    // Set the new configuration by shifting it to its position (SOL_Pos = 15)
    // and protecting it with the mask
    regValue |= (solAlert << INA236_SOL_Pos) & INA236_SOL_Mask;
    // Write the updated value back to the MASK_ENABLE register
    INA236_WriteRegister(ina236, INA236_MASK_ENABLE_REGISTER, regValue);
}


/** @brief  Retrieve the configured alert pin polarity
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @return Configured alert polarity (INA236_ACTIVE_LOW or INA236_ACTIVE_HIGH)
  */
INA236_AlertPol_TypeDef INA236_GetAlertPolarity(INA236_HandleTypeDef *ina236)
{
    // Read the Mask/Enable Register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Safety check: If reading fails, return the default polarity (active low)
    if (regValue == 0xFFFF)
    {
        return INA236_ACTIVE_LOW;
    }
    // Isolate the APOL bit and shift it right to return the logical value (0 or 1)
    //    (regValue & INA236_APOL) gives 0x0002 or 0x0000.
    //    Shifting it right by INA236_APOL_Pos (1) gives 0x0001 or 0x0000, matching the enum.
    INA236_AlertPol_TypeDef polarity = (INA236_AlertPol_TypeDef)((regValue & INA236_APOL) >> INA236_APOL_Pos);
    
    // Print the corresponding polarity to the console
    if (polarity == INA236_ACTIVE_HIGH)
    {
        printf("Alert pin is configured as ACTIVE-HIGH\r\n");
    }
    else
    {
        printf("Alert pin is configured as ACTIVE-LOW\r\n");
    }

    return polarity;
}   

/**
  * @brief  Retrieve and print the configuration of the ALERT pin assertion on Conversion Ready
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return Configured status for the conversion ready alert pin (INA236_CONV_READY_ALERT_DISABLE 
  *         or INA236_CONV_READY_ALERT_ENABLE)
  */
INA236_ConvRdy_TypeDef INA236_GetAlertPin(INA236_HandleTypeDef *ina236)
{
    // Read the Mask/Enable Register (Address: 0x06)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_MASK_ENABLE_REGISTER);

    // Safety check: If reading fails, return the default value (Disable)
    if (regValue == 0xFFFF)
    {
        printf("ERROR - ALERT pin does not assert on conversion ready\r\n");
        return INA236_CONV_READY_ALERT_DISABLE;
    }

    // Isolate the CNVR bit and shift it right to return the logical value (0 or 1)
    //    (regValue & INA236_CNVR) gives 0x0400 or 0x0000.
    //    Shifting it right by INA236_CNVR_Pos (10) gives 0x0001 or 0x0000, matching the enum.
    INA236_ConvRdy_TypeDef alert = (INA236_ConvRdy_TypeDef)((regValue & INA236_CNVR) >> INA236_CNVR_Pos);
    
    // 4. Print the corresponding alert configuration to the console (FIXED: changed 'polarity' to 'alert')
    if (alert == INA236_CONV_READY_ALERT_ENABLE)
    {
        printf("ALERT pin asserts when conversion is ready\r\n");
    }
    else
    {
        printf("ALERT pin does not assert on conversion ready\r\n");
    }

    return alert;
} 

/**
  * @brief  Read the shunt voltage and calculate its value in millivolts (mV)
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return Shunt voltage reading in millivolts (mV), or 0.0 if the read operation fails
  */
double INA236_GetShuntVoltage_mV(INA236_HandleTypeDef *ina236)
{
    // Read the Shunt Voltage Register (Address: 0x01)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_SHUNT_VOLTAGE_REGISTER);

    // Cast to signed 16-bit integer (two's complement)
    // This preserves the negative sign if current is flowing in reverse
    int16_t rawValue = (int16_t)regValue;

    // Calculate the voltage: raw_value * resolution (in Volts) * 1000 (to convert to mV)
    return (double)rawValue * ina236->_resolution * 1000.0;
}

/**
  * @brief  Read the bus voltage and calculate its value in Volts (V)
  * @note   The INA236 bus voltage register has a fixed resolution of 1.6 mV/LSB (0.0016 V/LSB).
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return Bus voltage in Volts (V), or 0.0 if the read operation fails
  */
double INA236_GetBusVoltage(INA236_HandleTypeDef *ina236)
{   
    // Read the Bus Voltage Register (Address: 0x02)
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_BUS_VOLTAGE_REGISTER);

        // Safety check: Prevent returning a false 104.85V reading if I2C fails (returns 0xFFFF)
    if (regValue == 0xFFFF)
    {
        return 0.0;
    }

     // Convert raw LSB to Volts (1.6 mV per LSB = 0.0016 V)
    return (double)regValue * 0.0016;
}

/**
  * @brief  Read load current and calculate its value in Amperes (A)
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @return Load current in Amperes (A)
  */
double INA236_GetCurrent(INA236_HandleTypeDef *ina236)
{
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_CURRENT_REGISTER);
    if (regValue == 0xFFFF) return 0.0;
    return (double)((int16_t)regValue) * ina236->_currentLsb;
    
}

/**
  * @brief  Read calculated power and calculate its value in Watts (W)
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure.
  * @return Power in Watts (W)
  */
double INA236_GetPower_W(INA236_HandleTypeDef *ina236)
{
    uint16_t regValue = INA236_ReadRegister(ina236, INA236_POWER_REGISTER);
    if (regValue == 0xFFFF) return 0.0;
    // Power LSB is fixed at 32 * Current_LSB
    return (double)regValue * (32.0 * ina236->_currentLsb);
}