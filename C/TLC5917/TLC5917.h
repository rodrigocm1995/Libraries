#ifndef INC_TLC5917_H_
#define INC_TLC5917_H_

#include "stm32f3xx_hal.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

typedef struct
{
  SPI_HandleTypeDef *spiHandle;
  GPIO_TypeDef      *csPort;
  uint16_t           csPin;
}TLC5917_t;


uint8_t TLC5917_Init(TLC5917_t *tlc5917, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

uint8_t TLC5917_ReadRegister(TLC5917_t *tlc5917, uint8_t registerAddress, uint8_t *data);

uint8_t TLC5917_WriteRegister(TLC5917_t *tlc5917, uint8_t data);

#endif

