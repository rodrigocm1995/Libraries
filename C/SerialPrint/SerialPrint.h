/**
    *******************************************************************************************
  * @file           : SerialPrint.h
  * @brief          : SerialPrint
    *******************************************************************************************
  */

#ifndef INC_SERIALPRINT_H_
#define INC_SERIALPRINT_H_

typedef struct
{
  UART_HandleTypeDef *huart;
}SerialPrint_t;

void serialPrint_init(SerialPrint_t *serialPrint, UART_HandleTypeDef *huart);
void serialPrint(SerialPrint_t *serialPrint, uint32_t value, char customString[]);

#endif

