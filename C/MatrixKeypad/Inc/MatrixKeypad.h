/**
    *******************************************************************************************
  * @file           : MatrixKeypad.h
  * @brief          : Matrix Keypad Library
    *******************************************************************************************
  */

#ifndef INC_MATRIX_KEYPAD_H_
#define INC_MATRIX_KEYPAD_H_

#define NUM_ROWS 4
#define NUM_COLS 4

typedef struct 
{
    GPIO_TypeDef * Port;
    uint16_t Pin;
} GPIO_PortMap_HandleTypeDef;


typedef struct
{
    uint32_t _lastScanTime;
    const GPIO_PortMap_HandleTypeDef *Rows; /* Outputs - Open drain*/
    const GPIO_PortMap_HandleTypeDef *Cols; /*Inputs - internal Pull-Up enabled */ 
    char _lastRawKey;
    char _stableKey;
    uint8_t _debounceCounter;
}Keypad_HandleTypeDef;

void Keypad_Init(Keypad_HandleTypeDef *hkeypad, GPIO_PortMap_HandleTypeDef *rows, GPIO_PortMap_HandleTypeDef *cols);

char Keypad_Scan(Keypad_HandleTypeDef *hkeypad);

void Keypad_Update(Keypad_HandleTypeDef *hkeypad);

void Keypad_OnkeyPress(char key);

#endif
