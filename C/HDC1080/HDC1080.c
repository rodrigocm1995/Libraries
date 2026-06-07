#include "main.h"
#include "math.h"
#include "HDC1080.h"

/**
  * @brief  Write a 16-bit value to a specific register of the HDC1080 device
  * @param  ina236 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified HDC1080.
  * @param  registerAddress Internal register address of the INA236 to write to
  * @param  value The 16-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef HDC1080_WriteRegister(HDC1080_HandleTypeDef *hdc1080, uint8_t registerAddress, uint16_t value)
{
    uint8_t address[2];
    // Split the 16-bit value into two bytes (MSB first)
    address[0] = (value >> 8) & 0xFF;
    address[1] = (value >> 0) & 0xFF;
  
    // Check if the device is ready on the I2C bus
    HAL_StatusTypeDef isDeviceReady = HAL_I2C_IsDeviceReady(hdc1080->hi2c, (hdc1080->_devAddress) << 1, HDC1080_TRIALS, HAL_MAX_DELAY);
    if (isDeviceReady == HAL_OK)
    {
        // Write the 16-bit register to the device
        if (HAL_I2C_Mem_Write(hdc1080->hi2c, (hdc1080->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_16BIT, HAL_MAX_DELAY) == HAL_OK)
        {
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 16-bit value from a specific register of the INA236 device
  * @param  ina236 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified HDC1080.
  * @param  registerAddress Internal register address of the HDC1080 to read from 
  * @return 16-bit data read from register, or 0xFFFF if the operation fails
  */
uint16_t HDC1080_ReadRegister(HDC1080_HandleTypeDef *hdc1080, uint8_t registerAddress)
{
    uint8_t registerResponse[2] = {0}; 
    HAL_StatusTypeDef isDeviceReady;   

    // Check if the device is ready on the I2C bus
    isDeviceReady = HAL_I2C_IsDeviceReady(hdc1080->hi2c, (hdc1080->_devAddress) << 1, HDC1080_TRIALS, HAL_MAX_DELAY);

    if (isDeviceReady == HAL_OK)
    {
        // Read the 2-byte register data from the device
        if (HAL_I2C_Mem_Read(hdc1080->hi2c, (hdc1080->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) == HAL_OK)
        {
            // Combine MSB and LSB to return the 16-bit value
            return (uint16_t)((registerResponse[0] << 8) | registerResponse[1]);
        }       
    }
    // Returns a maximum control value (0xFFFF) indicating a communication error
    return 0xFFFF;
}

void HDC1080_Init(HDC1080_HandleTypeDef *hdc1080, I2C_HandleTypeDef *i2c, uint8_t devAddress)
{
    hdc1080->hi2c = i2c;
    hdc1080->_devAddress = devAddress;
}

/**
  * @brief  Reset the HDC1080 device registers to their default factory values
  * @note   This function writes the soft reset bit (bit 15) directly to the 
  *         Configuration Register (0x02). The RST bit is self-clearing once 
  *         the reset is executed in the silicon.
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified HDC1080.
  * @retval None
  */
void HDC1080_ResetDevice(HDC1080_HandleTypeDef *hdc1080)
{
    // Write the reset bit (bit 15 = 0x8000) directly to the configuration register.
    HDC1080_WriteRegister(hdc1080, HDC1080_CONFIGURATION_REG, HDC1080_RST);
    
    // Wait for a brief delay to ensure that the chip's internal circuits
    // and I2C communication have stabilized after the reset.
    HAL_Delay(2);
}

/**
  * @brief  Set the relative humidity measurement resolution
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @param  resolution Desired humidity resolution:
  *                    @arg HDC1080_14BIT_RESOLUTION (0x0U) : 14-bit resolution
  *                    @arg HDC1080_11BIT_RESOLUTION (0x1U) : 11-bit resolution
  *                    @arg HDC1080_8BIT_RESOLUTION  (0x2U) : 8-bit resolution
  * @retval None
  */
void HDC1080_SetHumResolution(HDC1080_HandleTypeDef *hdc1080, HDC1080_Resolution_TypeDef resolution)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = HDC1080_ReadRegister(hdc1080, HDC1080_CONFIGURATION_REG);

    // Clear the HRES bits using the mask (bits 9:8)
    regValue &= ~HDC1080_HRES_Mask;

    // Set the new HRES by shifting it to its position (HRES_Pos = 8)
    // and protecting it with the mask
    regValue |= (resolution << HDC1080_HRES_Pos) & HDC1080_HRES_Mask;

    // Write the resulting value back to the register
    HDC1080_WriteRegister(hdc1080, HDC1080_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Set the relative temperature measurement resolution
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @param  resolution Desired htemperature resolution:
  *                    @arg HDC1080_14BIT_RESOLUTION (0x0U) : 14-bit resolution
  *                    @arg HDC1080_11BIT_RESOLUTION (0x1U) : 11-bit resolution
  * @retval None
  */
void HDC1080_SetTempResolution(HDC1080_HandleTypeDef *hdc1080, HDC1080_Resolution_TypeDef resolution)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = HDC1080_ReadRegister(hdc1080, HDC1080_CONFIGURATION_REG);

    // Clear the TRES bits using the mask (bit 10)
    regValue &= ~HDC1080_TRES_Mask;

    // Set the new TRES by shifting it to its position (TRES_Pos = 10)
    // and protecting it with the mask
    regValue |= (resolution << HDC1080_TRES_Pos) & HDC1080_TRES_Mask;
    
    // Write the resulting value back to the register
    HDC1080_WriteRegister(hdc1080, HDC1080_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Set the acquisition mode of the HDC1080 sensor
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @param  mode Desired acquisition mode.
  *              This parameter can be one of the following values:
  *              @arg HDC1080_TEMP_OR_HUMIDITY (0x0U): Temperature or Humidity is acquired individually
  *              @arg HDC1080_TEMP_AND_HUMIDITY (0x1U): Both Temperature and Humidity are acquired in sequence
  * @retval None
  */
void HDC1080_SetMode(HDC1080_HandleTypeDef *hdc1080, HDC1080_Mode_TypeDef mode)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = HDC1080_ReadRegister(hdc1080, HDC1080_CONFIGURATION_REG);

    // Clear the MODE bits using the mask (bit 12)
    regValue &= ~HDC1080_MODE_Mask;

    // Set the new MODE by shifting it to its position (MODE_Pos = 12)
    // and protecting it with the mask
    regValue |= (mode << HDC1080_MODE_Pos) & HDC1080_MODE_Mask;
    
    // Write the resulting value back to the register
    HDC1080_WriteRegister(hdc1080, HDC1080_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Enable or disable the integrated heating element of the HDC1080 sensor
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @param  heat Desired heater state.
  *              This parameter can be one of the following values:
  *              @arg HDC1080_HEATER_DISABLED (0x0U): Turn off the integrated heater
  *              @arg HDC1080_HEATER_ENABLED  (0x1U): Turn on the integrated heater (used to test or clear condensation)
  * @retval None
  */
void HDC1080_SetHeater(HDC1080_HandleTypeDef *hdc1080, HDC1080_Heater_TypeDef heat)
{
    // Read the current CONFIGURATION register
    uint16_t regValue = HDC1080_ReadRegister(hdc1080, HDC1080_CONFIGURATION_REG);

    // Clear the HEAT bit using the mask (bit 13)
    regValue &= ~HDC1080_HEAT_Mask;

    // Set the new HEAT by shifting it to its position (HEAT_Pos = 13)
    // and protecting it with the mask
    regValue |= (heat << HDC1080_HEAT_Pos) & HDC1080_HEAT_Mask;
    
    // Write the resulting value back to the register
    HDC1080_WriteRegister(hdc1080, HDC1080_CONFIGURATION_REG, regValue);
}

/**
  * @brief  Read the Manufacturer ID register from the HDC1080 sensor
  * @details This register contains a factory-programmable identification value that 
  *          identifies this device as being manufactured by Texas Instruments (TI).
  *          It is used to distinguish the HDC1080 from other devices on the I2C bus.
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @return Manufacturer ID (expected value is 0x5449). 
  *         Returns 0xFFFF if a communication error occurs.
  */
uint16_t HDC1080_GetManufacturerID(HDC1080_HandleTypeDef *hdc1080)
{
    uint16_t regValue = HDC_ReadRegister(hdc1080, HDC1080_MANUFACTURER_ID_REG);

	return regValue;
}


/**
  * @brief  Read the Device ID register from the HDC1080 sensor
  * @details This register contains a factory-programmable identification value that 
  *          identifies the model of the device. It is used to distinguish the HDC1080 
  *          from other devices on the I2C bus.
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @return Device ID (expected value is 0x1050). 
  *         Returns 0xFFFF if a communication error occurs.
  */
uint16_t HDC1080_GetDeviceID(HDC1080_HandleTypeDef *hdc1080)
{
    uint16_t regValue = HDC_ReadRegister(hdc1080, HDC1080_DEVICE_ID_REG);

	return regValue;
}

/**
  * @brief  Read the temperature from the HDC1080 sensor in Celsius degrees
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @return Temperature in degrees Celsius
  */
double HDC1080_GetTemp_C(HDC1080_HandleTypeDef *hdc1080)
{
    // Read the Temperature Result Register (Address: 0x00)
    uint16_t regValue = HDC1080_ReadRegister(hdc1080, HDC1080_TEMPERATURE_REG);

    // Convert the raw 16-bit register value using the formula: (Raw / 2^16) * 165 - 40
    double temperature = ((double)regValue / 65536.0) * 165.0 - 40.0;

    return temperature;
}

/**
  * @brief  Read the relative humidity from the HDC1080 sensor in %RH
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @return Relative humidity in percentage (0.0% to 100.0% RH)
  */
double HDC1080_GetHumidity(HDC1080_HandleTypeDef *hdc1080)
{
    // Read the Humidity Result Register (Address: 0x01)
    uint16_t rawHumidity = HDC1080_ReadRegister(hdc1080, HDC1080_HUMIDITY_REG);

    // Convert the raw 16-bit register value using the formula: (Raw / 2^16) * 100
    double humidity = ((double)rawHumidity / 65536.0) * 100.0;

    return humidity;
}

/**
  * @brief  Read and assemble the unique 40-bit Serial Number of the HDC1080
  * @details This function reads three dedicated read-only registers (0xFB, 0xFC, and 0xFD)
  *          to retrieve the device-unique 40-bit serial number.
  *          - Register 0xFB contains SERIAL ID[40:25] (bits 15:0)
  *          - Register 0xFC contains SERIAL ID[24:9] (bits 15:0)
  *          - Register 0xFD contains SERIAL ID[8:0] in its bits 15:7 (bits 6:0 are reserved as 0)
  * @param  hdc1080 Pointer to a HDC1080_HandleTypeDef structure that contains
  *                 the configuration and driver state for the specified HDC1080.
  * @return Unique 40-bit Serial Number. Returns 0 if a communication error occurs.
  */
uint64_t HDC1080_GetSerialNumber(HDC1080_HandleTypeDef *hdc1080)
{
    // Read the three registers that store the serial number parts
    uint16_t rawID1 = HDC1080_ReadRegister(hdc1080, HDC1080_SERIAL_ID1_REG);
    uint16_t rawID2 = HDC1080_ReadRegister(hdc1080, HDC1080_SERIAL_ID2_REG);
    uint16_t rawID3 = HDC1080_ReadRegister(hdc1080, HDC1080_SERIAL_ID3_REG);

    // If a read error occurs on any register, abort and return 0
    if (rawID1 == 0xFFFF || rawID2 == 0xFFFF || rawID3 == 0xFFFF)
    {
        return 0ULL;
    }

    // Align the lower 9 bits of the Serial ID (extracted from bits 15:7 of 0xFD)
    uint16_t rawID3 = rawID3 >> 7;

    // Assemble the 40-bit/41-bit serial number in its natural order:
    // [40:25] from rawID1, [24:9] from rawID2, and [8:0] from serialID_8_0
    uint64_t serialID = ((uint64_t)rawID1 << 25) | 
                        ((uint64_t)rawID2 << 9)  | 
                        (uint64_t)rawID3;
                        
    return serialID;
}