#include "main.h"
#include "TLC5917.h"

uint8_t TLC5917_Init(TLC5917_HandleTypeDef *tlc5917, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin)
{
  tlc5917->spiHandle = spiHandle;
  tlc5917->csPort    = csPort;
  tlc5917->csPin     = csPin;
  return 1;
}
uint8_t TLC5917_WriteRegister(TLC5917_HandleTypeDef *tlc5917, uint8_t data)
{
  uint8_t txBuf[1] = {data};
  // Asegura que Latch Enable esté en LOW al desplazar datos
  HAL_GPIO_WritePin(tlc5917->csPort, tlc5917->csPin, GPIO_PIN_RESET);
  
  // Transmisión SPI directa bloqueante (el hardware maneja los estados internos de finalización)
  uint8_t status = (HAL_SPI_Transmit(tlc5917->spiHandle, txBuf, 1, HAL_MAX_DELAY) == HAL_OK);
  
  // Genera un pulso controlado (LOW -> HIGH -> LOW) en LE para latchar los datos
  HAL_GPIO_WritePin(tlc5917->csPort, tlc5917->csPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(tlc5917->csPort, tlc5917->csPin, GPIO_PIN_RESET);
  return status;
}