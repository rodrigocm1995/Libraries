#include "main.h"
#include "math.h"
#include "INA236.h"

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
  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);
  if (isDeviceReady == HAL_OK)
  {
    // Write the 16-bit register to the device
    if (HAL_I2C_Mem_Write(ina236->hi2c, (ina236->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY) == HAL_OK)
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
  isDeviceReady = HAL_I2C_IsDeviceReady(ina236->hi2c, (ina236->devAddress) << 1, INA236_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    // Read the 2-byte register data from the device
    if (HAL_I2C_Mem_Read(ina236->hi2c, (ina236->devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) == HAL_OK)
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
  * @brief  Initializes the CONFIGURATION register's device with default values 
  * @param  ina236 points to an object of the type Ina236_t 
  * @param  rShuntValue Shunt Resistor's Value
  * @param  maxCurrent Maximum current monitored
  * @retval 
  */
void INA236_SetCalibration(INA236_HandleTypeDef *ina236, double rShuntValue, int maxCurrent)
{
  ina236->_shuntResistor = rShuntValue;
  ina236->_maximumCurrent = maxCurrent;
  double currentLsbMinimum;
  uint16_t shuntCal;

  currentLsbMinimum = (double)maxCurrent / 32768.0; // pow(2, 15) = 32765
  ina236->_currentLsbMin = currentLsbMinimum;

  shuntCal = 0.00512 / (currentLsbMinimum * rShuntValue);

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
    uint16_t regValue;

    // Read the current CONFIGURATION register
    regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

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
    uint16_t regValue;

    // Read the current CONFIGURATION register
    regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

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
    uint16_t regValue;

    // Read the current CONFIGURATION register
    regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

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
    uint16_t regValue;

    // Read the current CONFIGURATION register
    regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

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
    uint16_t regValue;

    // Read current register
    regValue = INA236_ReadRegister(ina236, INA236_CONFIGURATION_REGISTER);

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
    ina236->_adcRange = INA236_ADC_RANGE_81_92MV;
}

/**
  * @brief  Retrieve the Manufacturer ID from the INA236 device
  * @param  ina236 Pointer to a INA236_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified INA236.
  * @return 16-bit Manufacturer ID (Expected value is 0x5449), or 0xFFFF if reading fails
  */
uint16_t INA236_GetManufacturerID(INA236_HandleTypeDef *ina236)
{
    uint16_t regValue;

    // Read the Manufacturer ID register (Address: 0x3E)
    regValue = INA236_ReadRegister(ina236, MANUFACTURER_ID_REGISTER);

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
    uint16_t regValue;

    // Read the Manufacturer ID register (Address: 0x3F)
    regValue = INA236_ReadRegister(ina236, DEVICE_ID_REGISTER);

    return regValue;
}


