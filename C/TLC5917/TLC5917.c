#include "TLC5917.h"

uint8_t TLC5917_Init(TLC5917_t *tlc5917,
                     SPI_HandleTypeDef *spiHandle, 
		     GPIO_TypeDef *csPort,
                     uint16_t csPin)
{
	tlc5917->spiHandle = spiHandle;
	tlc5917->csPort    = csPort;
	tlc5917->csPin     = csPin;

	return 1;
}


uint8_t TLC5917_WriteRegister(TLC5917_t *tlc5917,
                              uint8_t data)
{
  uint8_t txBuf[1] = {data};

  HAL_GPIO_WritePin(tlc5917->csPort, tlc5917->csPin, GPIO_PIN_RESET);
  uint8_t status = (HAL_SPI_Transmit(tlc5917->spiHandle, txBuf, 1, HAL_MAX_DELAY) == HAL_OK);
  while(HAL_SPI_GetState(tlc5917->spiHandle) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(tlc5917->csPort, tlc5917->csPin, GPIO_PIN_SET);

  return status;
}
