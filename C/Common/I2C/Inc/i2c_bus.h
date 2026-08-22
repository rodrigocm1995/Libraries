/**
  *****************************************************************************************
  * @file           : i2c_bus.h
  * @brief          : Generic I2C BUS Abstraction Layer with Endianness Handling
  * 
  * @details
  * EXPLICACIÓN DE ENDIANNESS (ORDENACIÓN DE BYTES):
  * Cuando se transmiten datos multi-byte (16, 24, 32 o 64 bits) por I2C, el orden en el que
  * el sensor envía o recibe los bytes depende de su arquitectura:
  * 
  * - BE (Big Endian / MSB First):
  *   El byte más significativo (MSB) se envía primero. Es el estándar de red y el más común
  *   en sensores de marcas como Texas Instruments, STMicroelectronics, NXP y Bosch.
  *   Ejemplo: El valor 0x1234 se transmite como [0x12] seguido de [0x34].
  * 
  * - LE (Little Endian / LSB First):
  *   El byte menos significativo (LSB) se envía primero. Es el formato de los procesadores
  *   ARM Cortex-M (STM32) y x86. Sensores como Vishay (VCNL) suelen usarlo.
  *   Ejemplo: El valor 0x1234 se transmite como [0x34] seguido de [0x12].
  * 
  * Nota sobre registros de 8 bits:
  * Los registros de 8 bits (1 byte) NO tienen Endianness ya que son un único byte indivisible.
  * 
  * Nota sobre registros de 64 bits o superiores:
  * Se implementan desempaquetadores de 64 bits. Para anchos de registros mayores, variables,
  * o lecturas de ráfagas continuas de datos (buffers), se debe utilizar la función base
  * genérica I2C_Bus_ReadRegister / I2C_Bus_WriteRegister.
  *******************************************************************************************
  */
#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include "main.h"

/* =======================================================================================
   BASE LAYER: Generic functions for buffer transfer by reference
   ======================================================================================= */
HAL_StatusTypeDef I2C_Bus_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t *pData, uint16_t length);
HAL_StatusTypeDef I2C_Bus_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, const uint8_t *pData, uint16_t length);

/* =======================================================================================
   DECODING LAYER: Formatted and typed functions with endianness handling.
   ======================================================================================= */

/* --- Lecturas / Reads --- */
HAL_StatusTypeDef I2C_Bus_ReadRegister8(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t *value);

HAL_StatusTypeDef I2C_Bus_ReadRegister16_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t *value);
HAL_StatusTypeDef I2C_Bus_ReadRegister16_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t *value);

HAL_StatusTypeDef I2C_Bus_ReadRegister24_Signed_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, int32_t *value);
HAL_StatusTypeDef I2C_Bus_ReadRegister24_Unsigned_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value);

HAL_StatusTypeDef I2C_Bus_ReadRegister32_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value);
HAL_StatusTypeDef I2C_Bus_ReadRegister32_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t *value);

HAL_StatusTypeDef I2C_Bus_ReadRegister64_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint64_t *value);
HAL_StatusTypeDef I2C_Bus_ReadRegister64_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint64_t *value);

/* --- Escrituras / Writes --- */
HAL_StatusTypeDef I2C_Bus_WriteRegister8(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint8_t value);

HAL_StatusTypeDef I2C_Bus_WriteRegister16_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t value);
HAL_StatusTypeDef I2C_Bus_WriteRegister16_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint16_t value);

HAL_StatusTypeDef I2C_Bus_WriteRegister24_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value);

HAL_StatusTypeDef I2C_Bus_WriteRegister32_BE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value);
HAL_StatusTypeDef I2C_Bus_WriteRegister32_LE(I2C_HandleTypeDef *hi2c, uint8_t devAddress, uint8_t regAddress, uint32_t value);

#endif /* I2C_BUS_H_ */
