#include "main.h"
#include "math.h"
#include "OPT4048.h"

#define CIE_MATRIX_ROWS					  4
#define CIE_MATRIX_COLS					  4
#define ADC_MATRIX_ROWS					  1
#define ADC_MATRIX_COLS				      4

static uint8_t OPT4048_CalculateCRC(uint8_t exp, uint32_t mantissa, uint8_t count);


const double cieMatrix[CIE_MATRIX_ROWS][CIE_MATRIX_COLS] = {{0.000234892992, -0.0000189652390, 0.0000120811684, 0},
															{0.0000407467441, 0.000198958202, -0.0000158848115, 0.00215},
															{0.0000928619404, -0.0000169739553, 0.000674021520, 0},
															{0, 0, 0, 0}};



/**
  * @brief  Write a 16-bit value to a specific register of the OPT4048 device
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  registerAddress Internal register address of the OPT4048 to write to 
  * @param  value The 16-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef OPT4048_WriteRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress, uint16_t value)
{
    uint8_t address[2];
    HAL_StatusTypeDef isDeviceReady;
    // Split the 16-bit value into two bytes (MSB first)
    address[0] = (value >> 8) & 0xFF;
    address[1] = (value >> 0) & 0xFF;
  
    // Check if the device is ready on the I2C bus
    isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);
    if (isDeviceReady == HAL_OK)
    {
        // Write the 16-bit register to the device
        if (HAL_I2C_Mem_Write(opt4048->hi2c, (opt4048->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, sizeof(address), HAL_MAX_DELAY) == HAL_OK)
        {
        return HAL_OK;
        }
    }
    
    return HAL_ERROR;
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  registerAddress Target device address: The device 7 bits address value
  * @retval 16-bit data read from register's device
  */
uint16_t OPT4048_ReadRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress)
{
  uint8_t registerResponse[2];
  uint8_t isDeviceReady;

  isDeviceReady = HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY);

  if (isDeviceReady == HAL_OK)
  {
    HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY);
  }

  return ((registerResponse[0] << 8) | registerResponse[1]);
}

/**
  * @brief  Initialize the OPT4048 sensor instance
  * @details Sets the I2C instance and device address in the handle structure, and 
  *          configures the default settings of the sensor (Fault Count: 2, Alert Pin: Active High,
  *          Operating Mode: Continuous, Conversion Time: 200 ms, Range: Automatic, Burst Mode: Enabled).
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains the configuration 
  *             information for the specified I2C.
  * @param  devAddress 7-bit I2C device address (expected to be OPT4048_ADDRESS = 0x44).
  * @retval None
  */
void OPT4048_Init(OPT4048_HandleTypeDef *opt4048, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    opt4048->hi2c = i2c;
    opt4048->_devAddress = devAddress;

    OPT4048_SetFaultCount(opt4048, OPT4048_TWO_FAULT_COUNT);
    OPT4048_SetAlertPinPolarity(opt4048, OPT4048_ALERT_ACTIVE_HIGH);
    OPT4048_SetMode(opt4048, OPT4048_CONTINUOUS_MODE);
    OPT4048_SetConvTime(opt4048, OPT4048_200_MS);
    OPT4048_SetRange(opt4048, OPT4048_AUTOMATIC_RANGE);
    OPT4048_SetI2CType(opt4048, OPT4048_I2C_BURST_ENABLED);
}

/**
  * @brief  Read the raw ADC register values for all 4 channels (CH0 to CH3)
  * @details Performs a single 16-byte I2C burst read starting from the CH0_0 register (0x00).
  *          This retrieves the raw 16-bit registers (two per channel) in a single transaction.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  regValues Pointer to a uint16_t array of size 8 where the raw register values
  *                   will be stored:
  *                   - regValues[0]: CH0_0 (MSB)
  *                   - regValues[1]: CH0_1 (LSB)
  *                   - regValues[2]: CH1_0 (MSB)
  *                   - regValues[3]: CH1_1 (LSB)
  *                   - regValues[4]: CH2_0 (MSB)
  *                   - regValues[5]: CH2_1 (LSB)
  *                   - regValues[6]: CH3_0 (MSB)
  *                   - regValues[7]: CH3_1 (LSB)
  * @retval HAL_OK: Read operation completed successfully
  * @retval HAL_ERROR: Device is not ready or read operation failed
  */
HAL_StatusTypeDef OPT4048_ReadADCRawValues(OPT4048_HandleTypeDef *opt4048, uint16_t *regValues)
{
    if (regValues == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t tempBuf[16] = {0};

    // Verify that the device is ready on the I2C bus
    if (HAL_I2C_IsDeviceReady(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_TRIALS, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Perform a sequential burst read of 16 bytes starting at register address 0x00 (OPT4048_CH0_0)
    if (HAL_I2C_Mem_Read(opt4048->hi2c, (opt4048->_devAddress) << 1, OPT4048_CH0_0, I2C_MEMADD_SIZE_8BIT, tempBuf, 16, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Convert the 16 bytes received into 8 16-bit registers (MSB first)
    for (uint8_t i = 0; i < 8; i++)
    {
        regValues[i] = (uint16_t)((tempBuf[2 * i] << 8) | tempBuf[2 * i + 1]);
    }

    return HAL_OK;
}

/**
  * @brief  Retrieve the Device ID from the OPT4048 device
  * @param  ina236 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified OPT4048.
  * @return 16-bit Device ID (Expected value is 0x0821), or 0xFFFF if reading fails
  */
uint16_t OPT4048_GetDeviceID(OPT4048_HandleTypeDef *opt4048)
{
    // Read the Manufacturer ID register (Address: 0x11)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_DEVICE_ID_REG);
  
    return regValue;
}

/**
  * @brief  Configure the fault count threshold for the alert mechanism
  * @details Sets the number of consecutive fault measurements required before the 
  *          alert pin is triggered. This prevents false alarms due to optical transients.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  faultCount Number of consecutive faults:
  *                    @arg OPT4048_ONE_FAULT_COUNT   (0x0U): Trigger alert on 1 fault
  *                    @arg OPT4048_TWO_FAULT_COUNT   (0x1U): Trigger alert on 2 consecutive faults
  *                    @arg OPT4048_FOUR_FAULT_COUNT  (0x2U): Trigger alert on 4 consecutive faults
  *                    @arg OPT4048_EIGHT_FAULT_COUNT (0x3U): Trigger alert on 8 consecutive faults
  * @retval None
  */
void OPT4048_SetFaultCount(OPT4048_HandleTypeDef *opt4048, OPT4048_FaultCount_TypeDef faultCount)
{
    // Read the current CONFIGURATION register (Address: 0x0A)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the FAULT_COUNT bits using the mask (bits 0 and 1)
    regValue &= ~OPT4048_FAULT_COUNT_Mask;

    // Set the new mode by shifting it to its position (FAULT_COUNT_Pos = 0)
    // and protecting it with the mask
    regValue |= (faultCount << OPT4048_FAULT_COUNT_Pos) & OPT4048_FAULT_COUNT_Mask;

    // Write the updated value back to the CONFIGURATION register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the electrical polarity of the alert/interrupt pin
  * @details Sets whether the INT pin is active low or active high when an interrupt triggers.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  polarity Target polarity state:
  *                  @arg OPT4048_ALERT_ACTIVE_LOW  (0x0U): Alert pin is active low (default)
  *                  @arg OPT4048_ALERT_ACTIVE_HIGH (0x1U): Alert pin is active high
  * @retval None
  */
void OPT4048_SetAlertPinPolarity(OPT4048_HandleTypeDef *opt4048, OPT4048_AlertPinPol_TypeDef polarity)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the INT_POL bit using the mask (bit 2)
    regValue &= ~OPT4048_INT_POL_Mask;

    // Set the new INT_POL by shifting it to its position (INT_POL_Pos = 2)
    // and protecting it with the mask
    regValue |= (polarity << OPT4048_INT_POL_Pos) & OPT4048_INT_POL_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Set the operating mode of the OPT4048 device
  * @details Configures the measurement execution mode in the CONFIGURATION register.
  *          This controls whether the device is in low-power standby, executing single measurements,
  *          or performing continuous acquisitions.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  mode Target operating mode from OPT4048_Mode_TypeDef:
  *              @arg OPT4048_SHUTDOWN_MODE             (0x0U): Device enters ultra-low power standby mode.
  *              @arg OPT4048_FORCED_AUTORANGE_ONE_SHOT (0x1U): Triggers a single measurement with forced automatic range selection.
  *              @arg OPT4048_ONE_SHOT_MODE             (0x2U): Triggers a single measurement using the manually configured range.
  *              @arg OPT4048_CONTINUOUS_MODE           (0x3U): Device performs measurements continuously at the set conversion rate.
  * @retval None
  */
void OPT4048_SetMode(OPT4048_HandleTypeDef *opt4048, OPT4048_Mode_TypeDef mode)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the OPERATING_MODE bit using the mask (bits 4 and 5)
    regValue &= ~OPT4048_OPERATING_MODE_Mask;

    // Set the new OPERATING_MODE by shifting it to its position (OPERATING_MODE_Pos = 4)
    // and protecting it with the mask
    regValue |= (mode << OPT4048_OPERATING_MODE_Pos) & OPT4048_OPERATING_MODE_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the conversion time for a single measurement channel
  * @details Sets the integration time for a single channel. Since the OPT4048 is a 4-channel
  *          device, the actual time to complete a full measurement cycle across all channels
  *          is 4 times the value configured here.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  convTime Conversion time selection for a single channel:
  *                  @arg OPT4048_600_US  (0x0U): 600 us (Total cycle: 2.4 ms)
  *                  @arg OPT4048_1_MS    (0x1U): 1.0 ms (Total cycle: 4.0 ms)
  *                  @arg OPT4048_1_8_MS  (0x2U): 1.8 ms (Total cycle: 7.2 ms)
  *                  @arg OPT4048_3_4_MS  (0x3U): 3.4 ms (Total cycle: 13.6 ms)
  *                  @arg OPT4048_6_5_MS  (0x4U): 6.5 ms (Total cycle: 26.0 ms)
  *                  @arg OPT4048_12_7_MS (0x5U): 12.7 ms (Total cycle: 50.8 ms)
  *                  @arg OPT4048_25_MS   (0x6U): 25.0 ms (Total cycle: 100.0 ms)
  *                  @arg OPT4048_50_MS   (0x7U): 50.0 ms (Total cycle: 200.0 ms)
  *                  @arg OPT4048_100_MS  (0x8U): 100.0 ms (Total cycle: 400.0 ms)
  *                  @arg OPT4048_200_MS  (0x9U): 200.0 ms (Total cycle: 800.0 ms)
  *                  @arg OPT4048_400_MS  (0xAU): 400.0 ms (Total cycle: 1.6 s)
  *                  @arg OPT4048_800_MS  (0xBU): 800.0 ms (Total cycle: 3.2 s)
  * @retval None
  */
void OPT4048_SetConvTime(OPT4048_HandleTypeDef *opt4048, OPT4048_ConvTime_TypeDef convTime)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the CONVERSION_TIME bits using the mask (bits 6, 7, 8, and 9)
    regValue &= ~OPT4048_CONVERSION_TIME_Mask;

    // Set the new CONVERSION_TIME by shifting it to its position (CONVERSION_TIME_Pos = 6)
    // and protecting it with the mask
    regValue |= (convTime << OPT4048_CONVERSION_TIME_Pos) & OPT4048_CONVERSION_TIME_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the full-scale light measurement range
  * @details Sets the range selection in the CONFIGURATION register.
  *          TI highly recommends using OPT4048_AUTOMATIC_RANGE (0xCU) to allow 
  *          the sensor to dynamically optimize range selection for each channel.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  range Target full-scale light range:
  *               - OPT4048_2254_LUX       (0x0U): Manual range up to 2.25 klux
  *               - OPT4048_4509_LUX       (0x1U): Manual range up to 4.5 klux
  *               - OPT4048_9018_LUX       (0x2U): Manual range up to 9 klux
  *               - OPT4048_18036_LUX      (0x3U): Manual range up to 18 klux
  *               - OPT4048_36071_LUX      (0x4U): Manual range up to 36 klux
  *               - OPT4048_72142_LUX      (0x5U): Manual range up to 72 klux
  *               - OPT4048_144284_LUX     (0x6U): Manual range up to 144 klux
  *               - OPT4048_AUTOMATIC_RANGE (0xCU): Automatic range selection (recommended)
  * @retval None
  */
void OPT4048_SetRange(OPT4048_HandleTypeDef *opt4048, OPT4048_FullScaleRange_TypeDef range)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_REG);

    // Clear the RANGE bits using the mask (bits 10,11, 12, and 13)
    regValue &= ~OPT4048_RANGE_Mask;

    // Set the new RANGE by shifting it to its position (RANGE_Pos = 10)
    // and protecting it with the mask
    regValue |= (range << OPT4048_RANGE_Pos) & OPT4048_RANGE_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Configure the I2C read transaction type (burst or normal)
  * @details Sets the I2C burst read enable bit in the CONFIGURATION_1 register.
  *          Burst mode enables consecutive register reads in a single transaction,
  *          which is required for efficient multi-channel ADC reading.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @param  i2cType Target I2C readout configuration:
  *                 - OPT4048_I2C_BURST_DISABLED (0x0U): Normal single register read
  *                 - OPT4048_I2C_BURST_ENABLED  (0x1U): Sequential register read enabled (default)
  * @retval None
  */
void OPT4048_SetI2CType(OPT4048_HandleTypeDef *opt4048, OPT4048_I2CBurst_TypeDef i2cType)
{
    // Read the current CONFIGURATION_1 register
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_CONFIGURATION_1_REG);

    // Clear the I2C_BURST bit using the mask (bit 0)
    regValue &= ~OPT4048_I2C_BURST_Mask;

    // Set the new RANGE by shifting it to its position (I2C_BURST_Pos = 0)
    // and protecting it with the mask
    regValue |= (i2cType << OPT4048_I2C_BURST_Pos) & OPT4048_I2C_BURST_Mask;

    //Write the resulting value back to the register
    OPT4048_WriteRegister(opt4048, OPT4048_CONFIGURATION_1_REG, regValue); 
}

/**
  * @brief  Check if a new measurement conversion cycle is complete
  * @details Reads the MASK_ENABLE register and checks the CONVERSION_READY_FLAG status bit.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @retval 1 (true): New data is ready to be read from the registers.
  * @retval 0 (false): Conversion is still in progress.
  */
_Bool OPT4048_IsConversionReady(OPT4048_HandleTypeDef *opt4048)
{
    // Read the MASK_ENABLE Register (Address: 0x0C)
    uint16_t regValue = OPT4048_ReadRegister(opt4048, OPT4048_MASK_ENABLE_REG);
    
    // If the bit is set, (regValue & CONVERSION_READY_FLAG) will be 0x004 (which is != 0)
    if ((regValue & OPT4048_CONVERSION_READY_FLAG) != 0U)
    {
        return 1; 
    }
    return 0;
}

/**
  * @brief  Calculate the expected 4-bit CRC for a channel
  * @details Computes a 4-bit CRC from a 20-bit mantissa, 4-bit exponent, and 4-bit counter
  *          according to the OPT4048 datasheet specifications (modulo-2 XOR equations).
  *          Uses GCC built-in popcount for highly optimized bitwise XOR operations.
  * @param  exp 4-bit exponent value of the channel measurement.
  * @param  mantissa 20-bit mantissa value of the channel measurement.
  * @param  count 4-bit sample counter value of the channel measurement.
  * @retval 4-bit calculated CRC checksum.
  */
static uint8_t OPT4048_CalculateCRC(uint8_t exp, uint32_t mantissa, uint8_t count)
{
    // __builtin_popcount is a GCC compiler built-in that returns the number of set bits.
    // In ARM processors, this translates directly to optimized assembly instructions.
    uint32_t pop_all = __builtin_popcount(mantissa) + __builtin_popcount(exp) + __builtin_popcount(count);
    uint8_t crc = (uint8_t)(pop_all % 2);

    uint32_t pop_1 = __builtin_popcount(mantissa & 0xAAAAAU) + __builtin_popcount(exp & 0xAU) + __builtin_popcount(count & 0xAU);
    crc |= (uint8_t)(((pop_1 % 2) << 1));

    uint32_t pop_2 = __builtin_popcount(mantissa & 0x88888U) + __builtin_popcount(exp & 0x8U) + __builtin_popcount(count & 0x8U);
    crc |= (uint8_t)(((pop_2 % 2) << 2));

    uint32_t pop_3 = __builtin_popcount(mantissa & 0x80808U);
    crc |= (uint8_t)(((pop_3 % 2) << 3));

    return crc;
}

/**
  * @brief  Read and calculate the CIE XYZ chromaticity coordinates and Lux value
  * @details Reads all 4 channels in a single burst, decodes the floating-point mantissa 
  *          and exponent representation for each channel, and applies the CIE spectral 
  *          transformation matrix to calculate the x, y, z chromaticity and lux values.
  *          The computed values are stored directly inside the opt4048 handle structure.
  * @param  opt4048 Pointer to a OPT4048_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified OPT4048.
  * @retval HAL_OK: Read and calculations completed successfully
  * @retval HAL_ERROR: Device is not ready, read failed, CRC verification failed, or channels out of sync
  */
HAL_StatusTypeDef OPT4048_GetXYZAndLux(OPT4048_HandleTypeDef *opt4048)
{
    uint16_t regValues[8] = {0};
    double adc[4] = {0.0};

    // 1. Read all 8 registers in one burst
    if (OPT4048_ReadADCRawValues(opt4048, regValues) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 2. Decode each channel's registers, verify CRC, and extract ADC values
    for (uint8_t i = 0; i < OPT4048_NUMBER_OF_CHANNELS; i++)
    {
        uint16_t reg_msb = regValues[2 * i];
        uint16_t reg_lsb = regValues[2 * i + 1];

        // Exponent is in the upper 4 bits of MSB register (bits 15-12)
        uint8_t exponent = (reg_msb & OPT4048_EXPONENT_CH_X) >> OPT4048_EXPONENT_CH_X_Pos;

        // Mantissa is 20 bits total: 12 bits from MSB register (bits 11-0) and 8 bits from LSB register (bits 15-8)
        uint32_t msb_part = (reg_msb & OPT4048_RESULT_MSB_CH_X) >> OPT4048_RESULT_MSB_CH_X_Pos;
        uint32_t lsb_part = (reg_lsb & OPT4048_RESULT_LSB_CH_X) >> OPT4048_RESULT_LSB_CH_X_Pos;
        uint32_t mantissa = (msb_part << 8) | lsb_part;

        // Counter is in bits 7-4 of LSB register
        uint8_t counter = (reg_lsb & OPT4048_COUNTER_CH_X) >> OPT4048_COUNTER_CH_X_Pos;

        // CRC is in bits 3-0 of LSB register
        uint8_t crc_received = (reg_lsb & OPT4048_CRC_CH_X) >> OPT4048_CRC_CH_X_Pos;

        // Verify CRC data integrity
        uint8_t crc_expected = OPT4048_CalculateCRC(exponent, mantissa, counter);
        if (crc_received != crc_expected)
        {
            return HAL_ERROR; // Transmission corruption detected!
        }

        // Calculate final ADC code: mantissa * 2^exponent
        adc[i] = (double)(((uint64_t)mantissa) << exponent);
    }

    // 3. Verify synchronization across all channels
    // Sample counter values should match since they are read in a single burst transaction
    uint8_t count0 = (regValues[1] & OPT4048_COUNTER_CH_X_Mask) >> OPT4048_COUNTER_CH_X_Pos;
    uint8_t count1 = (regValues[3] & OPT4048_COUNTER_CH_X_Mask) >> OPT4048_COUNTER_CH_X_Pos;
    uint8_t count2 = (regValues[5] & OPT4048_COUNTER_CH_X_Mask) >> OPT4048_COUNTER_CH_X_Pos;
    uint8_t count3 = (regValues[7] & OPT4048_COUNTER_CH_X_Mask) >> OPT4048_COUNTER_CH_X_Pos;

    if ((count0 != count1) || (count0 != count2) || (count0 != count3))
    {
        return HAL_ERROR; // Channels are not synchronized (data mismatch)
    }

    // 4. Save sample counters to the handle struct for the host to monitor stuck registers
    opt4048->counterch0 = count0;
    opt4048->counterch1 = count1;
    opt4048->counterch2 = count2;
    opt4048->counterch3 = count3;

    // 5. Tristimulus CIE XYZ matrix transformation
    double X_raw = (adc[0] * cieMatrix[0][0]) + (adc[1] * cieMatrix[1][0]) + (adc[2] * cieMatrix[2][0]);
    double Y_raw = (adc[0] * cieMatrix[0][1]) + (adc[1] * cieMatrix[1][1]) + (adc[2] * cieMatrix[2][1]);
    double Z_raw = (adc[0] * cieMatrix[0][2]) + (adc[1] * cieMatrix[1][2]) + (adc[2] * cieMatrix[2][2]);

    // 6. Calculate chromaticity coordinates x, y, z (sum must be non-zero to avoid division by zero)
    double sum = X_raw + Y_raw + Z_raw;
    if (sum != 0.0)
    {
        opt4048->cieX = X_raw / sum;
        opt4048->cieY = Y_raw / sum;
        opt4048->cieZ = Z_raw / sum;
    }
    else
    {
        opt4048->cieX = 0.0;
        opt4048->cieY = 0.0;
        opt4048->cieZ = 0.0;
    }

    // 7. Calculate Lux from wideband Channel 1 (adc[1]) scaled by 0.00215
    opt4048->lux = adc[1] * 0.00215;

    return HAL_OK;
}


