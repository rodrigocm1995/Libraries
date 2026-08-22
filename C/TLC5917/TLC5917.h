#ifndef INC_TLC5917_H_
#define INC_TLC5917_H_
#include "stm32f3xx_hal.h"
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
typedef struct
{
  SPI_HandleTypeDef *spiHandle;
  GPIO_TypeDef      *csPort;
  uint16_t           csPin;
} TLC5917_HandleTypeDef;

/* Inicializa el driver asociando el bus SPI y el pin LE (Latch Enable) */
uint8_t TLC5917_Init(TLC5917_HandleTypeDef *tlc5917, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

/* Envía un byte de datos de segmentos al driver y actualiza las salidas */
uint8_t TLC5917_WriteRegister(TLC5917_HandleTypeDef *tlc5917, uint8_t data);
#endif /* INC_TLC5917_H_ */