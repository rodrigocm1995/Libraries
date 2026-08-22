#include "i2c_bus.h"

#ifdef HAL_I2C_MODULE_ENABLED

#define I2C_BUS_TRIALS  5
#define I2C_BUS_TIMEOUT 1000

/**
  * @brief  Read N bytes from a specific register of an I2C device
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure representing the physical I2C bus.
  * @param  devAddress 7-bit physical I2C slave address of the target device.
  * @param  regAddress Internal register address to read from.
  * @param  pData Pointer to the buffer where the read data will be stored.
  * @param  length Number of bytes to read from the target register.
  * @retval HAL_OK: Read operation completed successfully
  * @retval HAL_ERROR: Device is not ready or read operation failed
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t *pData, uint16_t length)
{
    if (hi2c == NULL || pData == NULL || length == 0)
    {
        return HAL_ERROR;
    }

    uint8_t shiftedAddress = devAddress << 1;

    if (HAL_I2C_IsDeviceReady(hi2c, shiftedAddress, I2C_BUS_TRIALS, 100) == HAL_OK)
    {
        return HAL_I2C_Mem_Read(hi2c, shiftedAddress, regAddress, I2C_MEMADD_SIZE_8BIT, pData, length, I2C_BUS_TIMEOUT);
    }

    return HAL_ERROR;
}

/**
  * @brief  Write N bytes to a specific register of an I2C device
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure representing the physical I2C bus.
  * @param  devAddress 7-bit physical I2C slave address of the target device.
  * @param  regAddress Internal register address to write to.
  * @param  pData Pointer to the buffer containing the data to be written (read-only).
  * @param  length Number of bytes to write to the target register.
  * @retval HAL_OK: Write operation completed successfully
  * @retval HAL_ERROR: Device is not ready or write operation failed
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, const uint8_t *pData, uint16_t length)
{
    if (hi2c == NULL || pData == NULL || length == 0) 
    {
        return HAL_ERROR;
    }

    uint8_t shiftedAddress = devAddress << 1;

    if (HAL_I2C_IsDeviceReady(hi2c, shiftedAddress, I2C_BUS_TRIALS, 100) == HAL_OK)
    {
        return HAL_I2C_Mem_Write(hi2c, shiftedAddress, regAddress, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, length, I2C_BUS_TIMEOUT);
    }

    return HAL_ERROR;
}

/* =======================================================================================
   Implementation of decoding functions (handling bit width and endianness
   ======================================================================================= */

/* ---------------------------------------------------------------------------------------
   READINGS SECTION (READS)
   --------------------------------------------------------------------------------------- */

/**
  * @brief  Read an 8-bit value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 8-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister8(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t *value)
{
    return I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, value, 1);
}

/**
  * @brief  Read a 16-bit Big Endian (MSB First) value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 16-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister16_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t *value)
{
    uint8_t buf[2] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 2) == HAL_OK)
    {
        *value = (uint16_t)((buf[0] << 8) | buf[1]);
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 16-bit Little Endian (LSB First) value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 16-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister16_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t *value)
{
    uint8_t buf[2] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 2) == HAL_OK)
    {
        *value = (uint16_t)((buf[1] << 8) | buf[0]);
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 24-bit signed Big Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 24-bit signed value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister24_Signed_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, int32_t *value)
{
    uint8_t buf[3] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 3) == HAL_OK)
    {
        uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        // Extensión de signo para enteros con signo de 24 bits
        *value = (raw & 0x800000) ? (int32_t)(raw | 0xFF000000) : (int32_t)raw;
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 24-bit unsigned Big Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 24-bit unsigned value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister24_Unsigned_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value)
{
    uint8_t buf[3] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 3) == HAL_OK)
    {
        *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 32-bit Big Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 32-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister32_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value)
{
    uint8_t buf[4] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 4) == HAL_OK)
    {
        *value = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 32-bit Little Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 32-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister32_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value)
{
    uint8_t buf[4] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 4) == HAL_OK)
    {
        *value = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 64-bit Big Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 64-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister64_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint64_t *value)
{
    uint8_t buf[8] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 8) == HAL_OK)
    {
        *value = ((uint64_t)buf[0] << 56) | ((uint64_t)buf[1] << 48) | ((uint64_t)buf[2] << 40) | ((uint64_t)buf[3] << 32) |
                 ((uint64_t)buf[4] << 24) | ((uint64_t)buf[5] << 16) | ((uint64_t)buf[6] << 8)  | buf[7];
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
  * @brief  Read a 64-bit Little Endian value from a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value Pointer to store the 64-bit value.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_ReadRegister64_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint64_t *value)
{
    uint8_t buf[8] = {0};
    if (I2C_Bus_ReadRegister(hi2c, devAddress, regAddress, buf, 8) == HAL_OK)
    {
        *value = ((uint64_t)buf[7] << 56) | ((uint64_t)buf[6] << 48) | ((uint64_t)buf[5] << 40) | ((uint64_t)buf[4] << 32) |
                 ((uint64_t)buf[3] << 24) | ((uint64_t)buf[2] << 16) | ((uint64_t)buf[1] << 8)  | buf[0];
        return HAL_OK;
    }
    return HAL_ERROR;
}

/* ---------------------------------------------------------------------------------------
   SECCIÓN DE ESCRITURAS (WRITES)
   --------------------------------------------------------------------------------------- */

/**
  * @brief  Write an 8-bit value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 8-bit value to be written.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister8(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t value)
{
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, &value, 1);
}

/**
  * @brief  Write a 16-bit Big Endian value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 16-bit value to be written.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister16_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF; // MSB
    buf[1] = value & 0xFF;        // LSB
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, buf, 2);
}

/**
  * @brief  Write a 16-bit Little Endian value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 16-bit value to be written.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister16_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = value & 0xFF;        // LSB
    buf[1] = (value >> 8) & 0xFF; // MSB
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, buf, 2);
}

/**
  * @brief  Write a 24-bit Big Endian value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 24-bit value to be written (LSB ignored, writes 3 bytes).
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister24_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value)
{
    uint8_t buf[3];
    buf[0] = (value >> 16) & 0xFF; // MSB
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;         // LSB
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, buf, 3);
}

/**
  * @brief  Write a 32-bit Big Endian value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 32-bit value to be written.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister32_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = (value >> 24) & 0xFF; // MSB
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;         // LSB
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, buf, 4);
}

/**
  * @brief  Write a 32-bit Little Endian value to a specific register
  * @param  hi2c Pointer to a HAL I2C_HandleTypeDef structure.
  * @param  devAddress 7-bit physical I2C address.
  * @param  regAddress Internal register address.
  * @param  value The 32-bit value to be written.
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_Bus_WriteRegister32_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = value & 0xFF;         // LSB
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF; // MSB
    return I2C_Bus_WriteRegister(hi2c, devAddress, regAddress, buf, 4);
}


#endif /* HAL_I2C_MODULE_ENABLED */

