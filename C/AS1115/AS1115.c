#include "main.h"
#include "math.h" // Functions that operate on floating point numbers are in math.h
#include "AS1115.h"

/**
  * @brief  Write a 8-bit value to a specific register of the AS1115 device
  * @param  as1115 Pointer to a AS1115_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified AS1115.
  * @param  registerAddress Internal register address of the AS1115 to write to
  * @param  value The 8-bit data word to be written into the target register
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef AS1115_WriteRegister(AS1115_HandleTypeDef *as1115, uint8_t registerAddress, uint8_t value)
{
    int8_t address[1];
    address[0] = value;

    HAL_StatusTypeDef isDeviceReady = HAL_I2C_IsDeviceReady(as1115->hi2c, (as1115->_devAddress) << 1, AS1115_TRIALS, HAL_MAX_DELAY);
  
    if (isDeviceReady == HAL_OK)
    {
        // Write the 16-bit register to the device
        if (HAL_I2C_Mem_Write(as1115->hi2c, (as1115->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t*)address, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY) == HAL_OK)
        {
        return HAL_OK;
        }
        return HAL_ERROR;
    }
}

/**
  * @brief  Read a 8-bit value from a specific register of the AS1115 device
  * @param  as1115 Pointer to a AS1115_HandleTypeDef structure that contains
  *                the configuration and driver state for the specified AS1115.
  * @param  registerAddress Internal register address of the AS1115 to read from 
  * @return 8-bit data read from register, or 0xFF if the operation fails
  */
uint8_t AS1115_ReadRegister(AS1115_HandleTypeDef *as1115, uint8_t registerAddress)
{
    uint8_t registerResponse[1] = {0};

    HAL_StatusTypeDef isDeviceReady = HAL_I2C_IsDeviceReady(as1115->hi2c, (as1115->_devAddress) << 1, AS1115_TRIALS, HAL_MAX_DELAY);

    if (isDeviceReady == HAL_OK)
    {
        // Read the 1-byte register data from the device
        if (HAL_I2C_Mem_Read(as1115->hi2c, (as1115->_devAddress) << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, registerResponse, sizeof(registerResponse), HAL_MAX_DELAY) == HAL_OK)
        {
            // Combine MSB and LSB to return the 16-bit value
            return registerResponse[0];
        }
        
        return 0xFF;
    }
}

void AS115_SetDisplayTest(AS1115_HandleTypeDef *as1115, AS1115_DisplayTest_TypeDef displayTestMode)
{
    // Read the current DISPLAY TEST MODE register
    uint8_t regValue = AS1115_ReadRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG);

    // Clear the DISP_Test bit (bit 0)
    regValue &= ~AS1115_Display_Test;

    // Set the new DISP_Test value
    regValue |= (displayTestMode << AS1115_Display_Test_Pos) & AS1115_Display_Test;

    //Write the resulting value back to the register
    AS1115_WriteRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG, regValue);
}

void AS115_SetLedShortTest(AS1115_HandleTypeDef *as1115, AS1115_LedShort_TypeDef ledShortMode)
{
    // Read the current DISPLAY TEST MODE register
    uint8_t regValue = AS1115_ReadRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG);

    // Clear the LED_Short bit (bit 1)
    regValue &= ~AS1115_LED_Short;

    // Set the new LED_Short value
    regValue |= (ledShortMode << AS1115_LED_Short_Pos) & AS1115_LED_Short;

    //Write the resulting value back to the register
    AS1115_WriteRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG, regValue);
}

void AS115_SetLedOpenTest(AS1115_HandleTypeDef *as1115, AS1115_LedOpen_TypeDef ledOpenMode)
{
    // Read the current DISPLAY TEST MODE register
    uint8_t regValue = AS1115_ReadRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG);

    // Clear the LED_Short bit (bit 1)
    regValue &= ~AS1115_LED_Open;

    // Set the new LED_Open value
    regValue |= (ledOpenMode << AS1115_LED_Open_Pos) & AS1115_LED_Open;

    //Write the resulting value back to the register
    AS1115_WriteRegister(as1115, AS1115_DISPLAY_TEST_MODE_REG, regValue);
}



