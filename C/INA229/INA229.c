#include <INA229.h>

#define maxRegAddress 0x3F
#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

const uint8_t INA229RegSize[maxRegAddress+1] = {
  2,2,2,2,3,3,2,3,\
  3,5,5,2,2,2,2,2,\
  2,2,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,0,0,\
  0,0,0,0,0,0,2,2
};

void INA229_Init(INA229_t *ina229,
                    SPI_HandleTypeDef *spiHandle, 
					          GPIO_TypeDef *csPort,
                    uint16_t csPin)
{
	ina229->spiHandle = spiHandle;
	ina229->csPort    = csPort;
	ina229->csPin     = csPin;
}

uint8_t INA229_WriteRegister(INA229_t *ina229, uint8_t registerAddress, uint8_t value)
{
  uint8_t txBuf[3] = {0}; // All ritable registers are 2 bytes
  uint8_t rxBuf[3] = {0};

  txBuf[0] = registerAddress << 2; // Address + write bit (ending in 0)
  txBuf[1] = MSB(value);
  txBuf[2] = LSB(value);

  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);
  uint8_t status = (HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, 3, 100) == HAL_OK);
  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);

  return status;
}

uint64_t INA229_ReadRegister(INA229_t *ina229, uint8_t registerAddress)
{
  uint64_t value;
  int i;

  uint8_t txBuf[1] = {0};
  uint8_t rxBuf[6] = {0}; //max buf size

  txBuf[0] = (registerAddress << 2) | 0x01; //Address + read bit (ending in 1)

  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);
  //uint8_t status = (HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, INA229RegSize[registerAddress]+1, 100) == HAL_OK);
  while((HAL_SPI_TransmitReceive(ina229->spiHandle, txBuf, rxBuf, INA229RegSize[registerAddress]+1, 100) != HAL_OK));
  while(HAL_SPI_GetState(ina229->spiHandle) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(ina229->csPort, ina229->csPin, GPIO_PIN_RESET);

  //Combine bytes
  value = 0;

  for (i = 1; i < INA229RegSize[registerAddress]+1; i++)
  {
    value = (value << 8) | rxBuf[i];
  }

  return value;
}

