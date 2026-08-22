#ifndef INC_TLC6984_H_
#define INC_TLC6984_H_



#define TLC6984_FC0_WRITE_REG   0xAA00
#define TLC6984_FC0_READ_REG    0xAA60
#define TLC6984_FC1_WRITE_REG   0xAA01
#define TLC6984_FC1_READ_REG    0xAA61
#define TLC6984_FC2_WRITE_REG   0xAA02



typedef struct
{
  SPI_HandleTypeDef *spiHandle;
  GPIO_TypeDef      *csPort;
  uint16_t           csPin;
}TLC6984_HandleTypeDef;


uint8_t TLC6984_Init(TLC6984_HandleTypeDef *TLC6984, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

uint8_t TLC6984_ReadRegister(TLC6984_HandleTypeDef *TLC6984, uint8_t registerAddress, uint8_t *data);

uint8_t TLC6984_WriteRegister(TLC6984_HandleTypeDef *TLC6984, uint8_t data);

#endif

