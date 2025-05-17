#include "main.h"
#include "SerialPrint.h"

void serialPrint_init(SerialPrint_t *serialPrint, UART_HandleTypeDef *huart){
	serialPrint->huart = huart;
}

serialPrint(SerialPrint_t *serialPrint, uint32_t value, char customString[]){

  uint8_t outputDataSize = 0;
  char outputData[50] = {'\0'};

  outputDataSize = sprintf(outputData, customString, value);
  HAL_UART_Transmit(serialPrint->huart, outputData, outputDataSize, HAL_MAX_DELAY);
}

