#include "i2c_bus.h"
#include "math.h"
#include "stm32f3xx_hal_def.h"
#include <stdint.h>
#include "OPT3001.h"

/* Static helper prototypes */
static void OPT3001_SetInnerRange(OPT3001_HandleTypeDef *opt3001, float limit);

/* =======================================================================================
   CAPA DE ADAPTACIÓN PRIVADA (WRAPPERS INTERNOS)
   ======================================================================================= */

/**
  * @brief  Write a 16-bit register value to the OPT3001 sensor via I2C
  * @note   This is an internal helper function. Transmits data in Big-Endian format.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure.
  * @param  registerAddress Target register address on the OPT3001.
  * @param  value 16-bit value to be written.
  * @return HAL status
  */
static inline HAL_StatusTypeDef OPT3001_WriteRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress, uint16_t value)
{
    return I2C_Bus_WriteRegister16_BE(opt3001->hi2c, opt3001->_devAddress, registerAddress, value);
}

/**
  * @brief  Read a 16-bit register value from the OPT3001 sensor via I2C
  * @note   This is an internal helper function. Reads data in Big-Endian format.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure.
  * @param  registerAddress Target register address on the OPT3001.
  * @param  value Pointer to store the read 16-bit value.
  * @return HAL status
  */
static inline HAL_StatusTypeDef OPT3001_ReadRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress, uint16_t *value)
{
    return I2C_Bus_ReadRegister16_BE(opt3001->hi2c, opt3001->_devAddress, registerAddress, value);
}

/* =======================================================================================
   CAPA DE APLICACIÓN 
   ======================================================================================= */

/**
  * @brief  Create a new instance of the OPT3001 ambient light sensor setting the I²C port and slave address
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *                the configuration information for connecting to the sensor.
  * @param  i2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_Init(OPT3001_HandleTypeDef *opt3001, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
	opt3001->_devAddress = devAddress;
	opt3001->hi2c = i2c;

    uint16_t config = 0;
    config |= (OPT3001_TWO_FAULT_COUNTS << OPT3001_FC_Pos)  & OPT3001_FC;
    config |= (OPT3001_ALERT_ACTIVE_LOW << OPT3001_POL_Pos) & OPT3001_POL;
    config |= (OPT3001_LATCH_HYSTERESIS << OPT3001_L_Pos)   & OPT3001_L;
    config |= (OPT3001_CONTINUOUS_MODE  << OPT3001_M_Pos)   & OPT3001_M;
    config |= (OPT3001_800_MS           << OPT3001_CT_Pos)  & OPT3001_CT;
    config |= (OPT3001_AUTOMATIC_RANGE  << OPT3001_RN_Pos)  & OPT3001_RN;

    return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, config);
}

/**
  * @brief  Read the raw value of the RESULT register.
  *         This register contains the result of the most recent light-to-digital conversion.
  *         The 16-bit register consists of two fields:
  *         - Bits [15:12] (E[3:0]): Exponent. Straight binary coding of the range scaling factor.
  *         - Bits [11:0]  (R[11:0]): Fractional result. Straight binary coding (zero to full-scale).
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  value Pointer to store the 16-bit result register value.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetResultReg(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
    return OPT3001_ReadRegister(opt3001, OPT3001_RESULT_REG, value);
}

/**
  * @brief  Read the current value of the CONFIGURATION register.
  *         This register controls the major operational modes of the device, 
  *         such as the full-scale range selection, conversion integration time,
  *         mode of conversion operation, interrupt flags, latching style, 
  *         alert pin polarity, and fault count parameters.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval uint16_t The raw 16-bit contents of the CONFIGURATION register.
  */
HAL_StatusTypeDef OPT3001_GetConfiguration(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
    return OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG, value);
}

/**
  * @brief  Read the raw 16-bit value of the LOW_LIMIT register.
  *         This register sets the lower comparison threshold limit for the interrupt 
  *         reporting mechanisms: the INT pin, the flag high field (FH), and the flag 
  *         low field (FL).
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval uint16_t Raw 16-bit value of the LOW_LIMIT register.
  */
HAL_StatusTypeDef OPT3001_GetLowLimitReg(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
	return OPT3001_ReadRegister(opt3001, OPT3001_LOW_LIMIT_REG, value);
}

/**
  * @brief  Read the raw 16-bit value of the HIGH_LIMIT register.
  *         This register sets the upper comparison threshold limit for the interrupt 
  *         reporting mechanisms: the INT pin, the flag high field (FH), and the flag 
  *         low field (FL).
  *         Its format is identical to the result register, consisting of a 4-bit exponent 
  *         (bits [15:12], HE[3:0]) and a 12-bit mantissa (bits [11:0], TH[11:0]).
  *         Note that the comparison with this register is unaffected by the Mask Enable (ME) bit.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval uint16_t Raw 16-bit value of the HIGH_LIMIT register.
  */
HAL_StatusTypeDef OPT3001_GetHighLimitReg(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
    return OPT3001_ReadRegister(opt3001, OPT3001_HIGH_LIMIT_REG, value);
}

/**
  * @brief  Read the Manufacturer ID register.
  *         This read-only register is used to uniquely identify the device manufacturer.
  *         For Texas Instruments, this register always returns 0x5449 (which corresponds 
  *         to the ASCII characters 'T' and 'I').
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval uint16_t The 16-bit manufacturer ID value (0x5449).
  */
HAL_StatusTypeDef OPT3001_GetManufacturerId(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
    return OPT3001_ReadRegister(opt3001, OPT3001_MANUFACTURER_ID_REG, value);
}

/**
  * @brief  Read the Device ID register.
  *         This read-only register is used to uniquely identify the device model.
  *         For the OPT3001, this register always returns 0x3001.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval uint16_t The 16-bit device ID value (0x3001).
  */
HAL_StatusTypeDef OPT3001_GetDeviceId(OPT3001_HandleTypeDef *opt3001, uint16_t *value)
{
    return OPT3001_ReadRegister(opt3001, OPT3001_DEVICE_ID_REG, value);
}

/**
  * @brief  Allows the user to set the Fault Count inside the CONFIGURATION register.
  *         The fault count field instructs the device as to how many consecutive fault events are
  *         required to trigger the interrupt reporting mechanisms: the INT pin, the flag high field
  *         (FH), and flag low field (FL). The fault events are described in the latch field (L),
  *         flag high field (FH), and flag low field (FL) descriptions.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  faultCount Specifies the number of consecutive fault events required.
  *         This parameter can be one of the following values:
  *         @arg OPT3001_ONE_FAULT_COUNT: One fault count (default, 00b)
  *         @arg OPT3001_TWO_FAULT_COUNTS: Two fault counts (01b)
  *         @arg OPT3001_FOUR_FAULT_COUNTS: Four fault counts (10b)
  *         @arg OPT3001_EIGHT_FAULT_COUNTS: Eight fault counts (11b)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetFaultCount(OPT3001_HandleTypeDef *opt3001, OPT3001_FaultCount_TypeDef faultCount)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_FC_Mask; // Clear the bit
        regValue |= (faultCount << OPT3001_FC_Pos) & OPT3001_FC_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Configure the polarity or active state of the interrupt (INT) pin.
  *         The polarity field controls whether the INT pin behaves as an active low or
  *         active high output. When configured as active low (default), the pin pulls low
  *         upon an interrupt event. When configured as active high, the pin becomes high
  *         impedance (allowing an external pull-up resistor to pull it high) upon an interrupt event.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  polarity Selected polarity for the INT pin.
  *         This parameter can be one of the following values:
  *         @arg OPT3001_ALERT_ACTIVE_LOW: INT pin reports active low (default, 0x0)
  *         @arg OPT3001_ALERT_ACTIVE_HIGH: INT pin reports active high (0x1)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetAlertPinPolarity(OPT3001_HandleTypeDef *opt3001, OPT3001_AlertPinPol_TypeDef polarity)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_POL_Mask; // Clear the bit
        regValue |= (polarity << OPT3001_POL_Pos) & OPT3001_POL_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Configure the interrupt latch behavior (Latch field L) of the interrupt system.
  *         This field controls whether the interrupt reporting mechanisms (the INT pin, 
  *         flag high FH, and flag low FL) behave in a hysteresis-style or latch-style.
  *         - In hysteresis mode (default), the interrupt reporting deasserts automatically 
  *           when the measured light returns within the limits.
  *         - In latch mode, the interrupt reporting remains asserted (latched) until the 
  *           Configuration register is read.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  latchMode Selected latch behavior for the interrupt system.
  *                  This parameter can be one of the following values:
  *                  @arg OPT3001_LATCH_HYSTERESIS: Hysteresis mode (default, 0x0)
  *                  @arg OPT3001_LATCH_LATCHED: Latch mode (0x1)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetLatchMode(OPT3001_HandleTypeDef *opt3001, OPT3001_Latch_TypeDef latchMode)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_L_Mask; // Clear the bit
        regValue |= (latchMode << OPT3001_L_Pos) & OPT3001_L_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Read the Flag Low (FL) field from the CONFIGURATION register.
  *         This read-only field identifies that the result of a conversion is smaller 
  *         than a specified level of interest. FL is set to 1 when the light level is 
  *         smaller than the value in the low-limit register (address 02h) for a consecutive 
  *         number of measurements defined by the fault count field (FC[1:0]).
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval _Bool Status of the Flag Low (FL) field.
  *         - 1 (true): The conversion result is smaller than the low limit threshold.
  *         - 0 (false): The conversion result is not smaller than the low limit threshold.
  */
_Bool OPT3001_IsLowLimitFlagSet(OPT3001_HandleTypeDef *opt3001)
{
    uint16_t regValue = 0;
    if (OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG, &regValue) == HAL_OK)
    {
        return (regValue & OPT3001_FL) != 0U;
    }

    return 0;
}

/**
  * @brief  Read the Flag High (FH) field from the CONFIGURATION register.
  *         This read-only field identifies that the result of a conversion is larger 
  *         than a specified level of interest. FH is set to 1 when the light level is 
  *         larger than the value in the high-limit register (address 03h) for a consecutive 
  *         number of measurements defined by the fault count field (   FC[1:0]).
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval _Bool Status of the Flag High (FH) field.
  *         - 1 (true): The conversion result is larger than the high limit threshold.
  *         - 0 (false): The conversion result is not larger than the high limit threshold.
  */
_Bool OPT3001_IsHighLimitFlagSet(OPT3001_HandleTypeDef *opt3001)
{
    uint16_t regValue = 0;
    if (OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG, &regValue) == HAL_OK)
    {
        return (regValue & OPT3001_FH) != 0U;
    }

    return 0;
}

/**
  * @brief  Read the Conversion Ready Field (CRF) from the CONFIGURATION register.
  *         This read-only field indicates when a conversion completes. The field is set to 1 
  *         at the end of a conversion and is cleared (set to 0) when the configuration register 
  *         is subsequently read or written with any value except one containing the shutdown mode.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval _Bool Status of the Conversion Ready Field (CRF).
  *         - 1 (true): A conversion is complete and new data is ready in the RESULT register.
  *         - 0 (false): A conversion is not yet complete or the flag was cleared by a previous read/write.
  */
_Bool OPT3001_IsConversionReady(OPT3001_HandleTypeDef *opt3001)
{
    uint16_t regValue = 0;
    if (OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG, &regValue) == HAL_OK)
    {
        return (regValue & OPT3001_CRF) != 0U;
    }

    return 0;
}

/**
  * @brief  Read the Overflow Flag (OVF) from the CONFIGURATION register.
  *         This read-only field indicates if the input light level has exceeded the full-scale range:
  *         - Manual Range (RN[3:0] < 12): OVF can be set by a temporary light spike that overloads 
  *           the integrating ADC (even if the final result register is within range), reporting a 
  *           possible conversion error.
  *         - Auto Range (RN[3:0] = 12): OVF is only set if the input light exceeds the maximum 
  *           physical range of the device. The sensor automatically scales up its range and retries 
  *           conversions until no overflow occurs or maximum scale is reached.
  *         The flag is re-evaluated on every measurement.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @retval _Bool Status of the Overflow Flag (OVF).
  *         - 1 (true): An overflow condition occurred during the conversion.
  *         - 0 (false): No overflow occurred.
  */
_Bool OPT3001_IsOverflowFlagSet(OPT3001_HandleTypeDef *opt3001)
{
    uint16_t regValue = 0;
    if (OPT3001_ReadRegister(opt3001, OPT3001_CONFIGURATION_REG, &regValue) == HAL_OK)
    {
        return (regValue & OPT3001_OVF) != 0U;
    }

    return 0;
}

/**
  * @brief  Configure the operating mode of the OPT3001 sensor (Continuous, Shutdown, or One-Shot)
  * @note   This function modifies the M bits (bits 9 and 10) of the Configuration Register (0x01).
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified OPT3001.
  * @param  mode Selected operating mode.
  *         This parameter can be one of the following values:
  *         @arg OPT3001_CONTINUOUS_MODE: Continuous conversion mode
  *         @arg OPT3001_SINGLE_SHOT_MODE: One-shot conversion mode
  *         @arg OPT3001_SHUTDOWN_MODE: Low-power shutdown mode (default)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetMode(OPT3001_HandleTypeDef *opt3001, OPT3001_Mode_TypeDef mode)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_M_Mask; // Clear the bit
        regValue |= (mode << OPT3001_M_Pos) & OPT3001_M_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Configure the conversion (integration) time for the light measurement.
  *         The conversion time determines the duration of the light-to-digital conversion process.
  *         A longer integration time (800 ms) allows for a lower noise measurement, resulting 
  *         in higher precision.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  convTime Selected conversion integration time.
  *         This parameter can be one of the following values:
  *         @arg OPT3001_100_MS: 100 milliseconds integration time (0x0)
  *         @arg OPT3001_800_MS: 800 milliseconds integration time (0x1)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetConvTime(OPT3001_HandleTypeDef *opt3001, OPT3001_ConvTime_TypeDef convTime)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_CT_Mask; // Clear the bit
        regValue |= (convTime << OPT3001_CT_Pos) & OPT3001_CT_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Read the current fault count configuration from the sensor.
  *         This function reads the CONFIGURATION register, extracts the fault count bits (bits 1:0),
  *         and stores the active setting in the provided pointer.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  faultCount Pointer to a OPT3001_FaultCount_TypeDef variable where the setting will be stored:
  *         - OPT3001_ONE_FAULT_COUNT (0x0): 1 fault count
  *         - OPT3001_TWO_FAULT_COUNTS (0x1): 2 fault counts
  *         - OPT3001_FOUR_FAULT_COUNTS (0x2): 4 fault counts
  *         - OPT3001_EIGHT_FAULT_COUNTS (0x3): 8 fault counts
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetFaultCount(OPT3001_HandleTypeDef *opt3001, OPT3001_FaultCount_TypeDef *faultCount)
{
    if (faultCount == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetConfiguration(opt3001, &regValue);

    if (status == HAL_OK)
    {
        *faultCount = (OPT3001_FaultCount_TypeDef)( (regValue & OPT3001_FC) >> OPT3001_FC_Pos );
    }

    return status;
}

/**
  * @brief  Read the current alert pin polarity configuration from the sensor.
  *         This function reads the CONFIGURATION register, extracts the polarity bit (bit 3),
  *         and stores the active setting in the provided pointer.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  polarity Pointer to a OPT3001_AlertPinPol_TypeDef variable where the setting will be stored:
  *         - OPT3001_ALERT_ACTIVE_LOW (0x0): INT pin active low
  *         - OPT3001_ALERT_ACTIVE_HIGH (0x1): INT pin active high
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetAlertPinPolarity(OPT3001_HandleTypeDef *opt3001, OPT3001_AlertPinPol_TypeDef *polarity)
{
    if (polarity == NULL) return HAL_ERROR;
    
    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetConfiguration(opt3001, &regValue);

    if (status == HAL_OK)
    {
        *polarity = (OPT3001_AlertPinPol_TypeDef)( (regValue & OPT3001_POL) >> OPT3001_POL_Pos );
    }

    return status;
}

/**
  * @brief  Read the current interrupt latch mode configuration from the sensor.
  *         This function reads the CONFIGURATION register, extracts the latch bit (bit 4),
  *         and stores the active setting in the provided pointer.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  latchMode Pointer to a OPT3001_Latch_TypeDef variable where the setting will be stored:
  *         - OPT3001_LATCH_HYSTERESIS (0x0): Hysteresis-style interrupts
  *         - OPT3001_LATCH_LATCHED (0x1): Latched interrupts (cleared by reading Config register)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetLatchMode(OPT3001_HandleTypeDef *opt3001, OPT3001_Latch_TypeDef *latchMode)
{
    if (latchMode == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetConfiguration(opt3001, &regValue);

    if (status == HAL_OK)
    {
        *latchMode = (OPT3001_Latch_TypeDef)((regValue & OPT3001_L) >> OPT3001_L_Pos);
    }

    return status; 
}

/**
  * @brief  Read the current operating mode configuration from the sensor.
  *         This function reads the CONFIGURATION register, extracts the mode bits (bits 10:9),
  *         and stores the active setting in the provided pointer.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  mode Pointer to a OPT3001_Mode_TypeDef variable where the setting will be stored:
  *         - OPT3001_SHUTDOWN_MODE (0x0): Low-power shutdown state
  *         - OPT3001_SINGLE_SHOT_MODE (0x1): One-shot conversion mode
  *         - OPT3001_CONTINUOUS_MODE (0x2): Continuous conversion mode
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetMode(OPT3001_HandleTypeDef *opt3001, OPT3001_Mode_TypeDef *mode)
{
    if (mode == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetConfiguration(opt3001, &regValue);

    if (status == HAL_OK)
    {
        *mode = (OPT3001_Mode_TypeDef)((regValue & OPT3001_M) >> OPT3001_M_Pos);
    }

    return status;
}


/**
  * @brief  Read the current conversion (integration) time configuration from the sensor.
  *         This function reads the CONFIGURATION register, extracts the CT bit (bit 11),
  *         and stores the active conversion time setting in the provided pointer.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  convTime Pointer to a OPT3001_ConvTime_TypeDef variable where the setting will be stored:
  *         - OPT3001_100_MS (0x0): 100 ms integration time
  *         - OPT3001_800_MS (0x1): 800 ms integration time
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetConvTime(OPT3001_HandleTypeDef *opt3001, OPT3001_ConvTime_TypeDef *convTime)
{
    if (convTime == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetConfiguration(opt3001, &regValue);

    if (status == HAL_OK)
    {
        *convTime = (OPT3001_ConvTime_TypeDef)((regValue & OPT3001_CT_Mask) >> OPT3001_CT_Pos);
    }

    return status;
}

/**
  * @brief  Configure the full-scale range (Range Number RN field) in the CONFIGURATION register.
  *         This field selects the full-scale lux range of the device. It can be set manually 
  *         to a specific range or set to automatic scaling mode, where the device automatically 
  *         adjusts the range based on ambient light levels to optimize resolution and prevent overflow.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  range Selected full-scale range.
  *         This parameter can be one of the following values:
  *         @arg OPT3001_40_95_LUX: Full-scale range up to 40.95 lux (0x0)
  *         @arg OPT3001_81_90_LUX: Full-scale range up to 81.90 lux (0x1)
  *         @arg OPT3001_163_80_LUX: Full-scale range up to 163.80 lux (0x2)
  *         @arg OPT3001_327_60_LUX: Full-scale range up to 327.60 lux (0x3)
  *         @arg OPT3001_655_20_LUX: Full-scale range up to 655.20 lux (0x4)
  *         @arg OPT3001_1310_40_LUX: Full-scale range up to 1310.40 lux (0x5)
  *         @arg OPT3001_2620_80_LUX: Full-scale range up to 2620.80 lux (0x6)
  *         @arg OPT3001_5241_60_LUX: Full-scale range up to 5241.60 lux (0x7)
  *         @arg OPT3001_10483_20_LUX: Full-scale range up to 10483.20 lux (0x8)
  *         @arg OPT3001_20966_40_LUX: Full-scale range up to 20966.40 lux (0x9)
  *         @arg OPT3001_41932_80_LUX: Full-scale range up to 41932.80 lux (0xA)
  *         @arg OPT3001_83865_60_LUX: Full-scale range up to 83865.60 lux (0xB)
  *         @arg OPT3001_AUTOMATIC_RANGE: Automatic scaling mode (default, 0xC)
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetRangeNumber(OPT3001_HandleTypeDef *opt3001, OPT3001_FullScaleRange_TypeDef range)
{
    uint16_t regValue = 0;

    if (OPT3001_GetConfiguration(opt3001, &regValue) == HAL_OK)
    {
        regValue &= ~OPT3001_RN_Mask; // Clear the bit
        regValue |= (range << OPT3001_RN_Pos) & OPT3001_RN_Mask; // Set the new value
        return OPT3001_WriteRegister(opt3001, OPT3001_CONFIGURATION_REG, regValue); 
    }

    return HAL_ERROR;
}

/**
  * @brief  Determine and set the internal exponent scale factor based on a target lux limit.
  *         This private helper function evaluates a target lux threshold (e.g., for setting 
  *         low/high comparison limits) and maps it to the lowest full-scale range capable of 
  *         representing that value. The selected exponent scale is stored internally in the 
  *         handle structure under the _innerExponent field.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration and state information for the sensor.
  * @param  limit The target light intensity threshold in lux. Must be less than or equal 
  *         to the maximum measurable value (83865.60 lux).
  * @retval None
  */
static void OPT3001_SetInnerRange(OPT3001_HandleTypeDef *opt3001, float limit)
{
	if (limit <= 40.95f) opt3001->_innerExponent = OPT3001_40_95_LUX;
    else if ((limit > 40.95f)    && (limit <= 81.90f))    opt3001->_innerExponent = OPT3001_81_90_LUX;
	else if ((limit > 81.90f)    && (limit <= 163.80f))   opt3001->_innerExponent = OPT3001_163_80_LUX;
	else if ((limit > 163.80f)   && (limit <= 327.60f))   opt3001->_innerExponent = OPT3001_327_60_LUX;
	else if ((limit > 327.60f)   && (limit <= 655.20f))   opt3001->_innerExponent = OPT3001_655_20_LUX;
	else if ((limit > 655.20f)   && (limit <= 1310.40f))  opt3001->_innerExponent = OPT3001_1310_40_LUX;
	else if ((limit > 1310.40f)  && (limit <= 2620.80f))  opt3001->_innerExponent = OPT3001_2620_80_LUX;
	else if ((limit > 2620.80f)  && (limit <= 5241.60f))  opt3001->_innerExponent = OPT3001_5241_60_LUX;
	else if ((limit > 5241.60f)  && (limit <= 10483.20f)) opt3001->_innerExponent = OPT3001_10483_20_LUX;
	else if ((limit > 10483.20f) && (limit <= 20966.40f)) opt3001->_innerExponent = OPT3001_20966_40_LUX;
	else if ((limit > 20966.40f) && (limit <= 41932.80f)) opt3001->_innerExponent = OPT3001_41932_80_LUX;
	else                                                  opt3001->_innerExponent = OPT3001_83865_60_LUX;
}

/**
  * @brief  Set the lower comparison limit threshold in lux.
  *         This function determines the best full-scale range exponent for the target limit,
  *         calculates the corresponding 12-bit mantissa, combines both fields into a raw
  *         16-bit register layout, and writes it to the LOW_LIMIT register.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration and state information for the sensor.
  * @param  lowLimit The target low limit threshold in lux.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetLowLimit(OPT3001_HandleTypeDef *opt3001, float lowLimit)
{
    // Determine and set the inner exponent in opt3001->_innerExponent
    OPT3001_SetInnerRange(opt3001, lowLimit);

	uint8_t exponent = opt3001->_innerExponent;

    // Calculate the mantissa using the formula: limit / (0.01 * 2^exponent)
	float mantissa = lowLimit / (0.01 * (1 << exponent));

    // Combine exponent (shifted to bits 15:12) and mantissa (bits 11:0)
	uint16_t rawValue = (exponent << OPT3001_E_Pos) | (uint16_t)mantissa;

    return OPT3001_WriteRegister(opt3001, OPT3001_LOW_LIMIT_REG, rawValue);
}

/**
  * @brief  Set the higher comparison limit threshold in lux.
  *         This function determines the best full-scale range exponent for the target limit,
  *         calculates the corresponding 12-bit mantissa, combines both fields into a raw
  *         16-bit register layout, and writes it to the HIGH_LIMIT register.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration and state information for the sensor.
  * @param  highLimit The target high limit threshold in lux.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_SetHighLimit(OPT3001_HandleTypeDef *opt3001, float highLimit)
{
    // Determine and set the inner exponent in opt3001->_innerExponent
    OPT3001_SetInnerRange(opt3001, highLimit);

	uint8_t exponent = opt3001->_innerExponent;

    // Calculate the mantissa using the formula: limit / (0.01 * 2^exponent)
	float mantissa = highLimit / (0.01 * (1 << exponent));

    // Combine exponent (shifted to bits 15:12) and mantissa (bits 11:0)
	uint16_t rawValue = (exponent << OPT3001_E_Pos) | (uint16_t)mantissa;

	return OPT3001_WriteRegister(opt3001, OPT3001_HIGH_LIMIT_REG, rawValue);
}

/**
  * @brief  Read the HIGH_LIMIT register and calculate its value in lux.
  *         This function retrieves the raw 16-bit register value, extracts the 
  *         exponent and mantissa fields using their respective masks, and calculates 
  *         the threshold limit in lux using the formula: lux = 0.01 * (2^exponent) * mantissa.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  highLimitLux Pointer to a float variable where the high limit threshold in lux will
  *         be stored.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetHighLimit_Lux(OPT3001_HandleTypeDef *opt3001, float *highLimitLux)
{
    if (highLimitLux == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status =  OPT3001_GetHighLimitReg(opt3001, &regValue);

    if (status == HAL_OK)
    {
        // Extract the exponent value by masking and shifting to its position (bit 12)
        uint8_t exponent = (regValue & OPT3001_E) >> OPT3001_E_Pos;

        // Extract the mantissa value by masking and shifting to its position (bit 0)
        uint16_t mantissa = (regValue & OPT3001_R) >> OPT3001_R_Pos;

        // Calculate the lux value using the formula: 0.01 * (2^exponent) * mantissa
        *highLimitLux = 0.01f * (float)(1 << exponent) * (float)mantissa;
    }

    return status;
}

/**
  * @brief  Read the LOW_LIMIT register and calculate its value in lux.
  *         This function retrieves the raw 16-bit register value, extracts the 
  *         exponent and mantissa fields using their respective masks, and calculates 
  *         the threshold limit in lux using the formula: lux = 0.01 * (2^exponent) * mantissa.
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  lowLimitLux Pointer to a float variable where the low limit threshold in lux will be stored.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetLowLimit_Lux(OPT3001_HandleTypeDef *opt3001, float *lowLimitLux)
{
    if (lowLimitLux == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetLowLimitReg(opt3001, &regValue);
    
    if (status == HAL_OK)
    {
        // Extract the exponent value by masking and shifting to its position (bit 12)
        uint8_t exponent = (regValue & OPT3001_E) >> OPT3001_E_Pos;

        // Extract the mantissa value by masking and shifting to its position (bit 0)
        uint16_t mantissa = (regValue & OPT3001_R) >> OPT3001_R_Pos;

        // Calculate the lux value using the formula: 0.01 * (2^exponent) * mantissa
        *lowLimitLux = 0.01f * (float)(1 << exponent) * (float)mantissa;
    }
    return status;
}

/**
  * @brief  Read the RESULT register and calculate the ambient light level in lux.
  *         The conversion formula is:
  *         lux = 0.01 * (2^E[3:0]) * R[11:0]
  *         Where:
  *         - E[3:0] is the exponent (bits [15:12])
  *         - R[11:0] is the mantissa fractional result (bits [11:0])
  * @param  opt3001 Pointer to a OPT3001_HandleTypeDef structure that contains
  *         the configuration information for connecting to the sensor.
  * @param  lux Pointer to a float variable where the calculated ambient light level in lux will be stored.
  * @return HAL status
  */
HAL_StatusTypeDef OPT3001_GetLux(OPT3001_HandleTypeDef *opt3001, float *lux)
{
    if (lux == NULL) return HAL_ERROR;

    uint16_t regValue = 0;
    HAL_StatusTypeDef status = OPT3001_GetResultReg(opt3001, &regValue);
    
    if (status == HAL_OK)
    {
        // Extract the exponent value by masking and shifting to its position (bit 12)
        uint8_t exponent = (regValue & OPT3001_E) >> OPT3001_E_Pos;

        // Extract the mantissa value by masking and shifting to its position (bit 0)
        uint16_t mantissa = (regValue & OPT3001_R) >> OPT3001_R_Pos;

        // Calculate the lux value using the formula: 0.01 * (2^exponent) * mantissa
        *lux = 0.01f * (float)(1 << exponent) * (float)mantissa;
    }
    return status;
}