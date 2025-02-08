/**
    *******************************************************************************************
  * @file           : PT1035.h
  * @brief          : PT1035 Library
    *******************************************************************************************

  * 
  *   
  * @details
  *
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_PT1035_H_
#define INC_PT1035_H_

#define PT1035_DEFAULT_ADDRESS                  0x70
#define PT1035_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define PT1035_TEMP_REGISTER                    0x00
#define PT1035_CONFIGURATION_REGISTER           0x01
#define PT1035_TLOW_LIMIT_REGISTER              0x02
#define PT1035_THIGH_LIMIT_REGISTER             0x03
#define PT1035_MANUFACTURER_ID_REGISTER         0x04

typedef enum
{
  PT1035_4_SECONDS                            = 0x00,
  PT1035_1_SECOND                             = 0x20,
  PT1035_250_MSECONDS                         = 0x40,
  PT1035_125_MSECONDS                         = 0x60,
}pt1035ConversionRate_t;

typedef enum
{
  PT1035_SHUNTDOWN_MODE                       = 0x00,
  PT1035_ONESHOT_MODE                         = 0x01,
  PT1035_CONTINUOUS_MODE                      = 0x02,
} pt1035FunctionalMode_t;

typedef enum
{
  PT1035_LATCH_OFF                           = 0x00,
  PT1035_LATCH_ON                            = 0x04,
} pt1035LatchMode_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t devAddress;
}PT1035_t;


void pt1035WriteRegister8(PT1035_t *pt1035, uint8_t registerAddress, int8_t value);

void pt1035WriteRegister16(PT1035_t *pt1035, uint8_t registerAddress, int16_t value);

int8_t pt1035ReadRegister8(PT1035_t *pt1035, uint8_t registerAddress);

int16_t pt1035ReadRegister16(PT1035_t *pt1035, uint8_t registerAddress);

uint8_t pt1035Init(PT1035_t *pt1035, I2C_HandleTypeDef *i2c, uint8_t devAddress);

uint8_t pt1035getConfig(PT1035_t *pt1035);

uint16_t pt1035CustomInit(PT1035_t *pt1035, I2C_HandleTypeDef *i2c, uint8_t devAddress, pt1035ConversionRate_t convRate, pt1035FunctionalMode_t functMode, pt1035LatchMode_t latchMode);

void pt1035SetHighLimit(PT1035_t *pt1035, int16_t highLimitTemp);

double pt1035GetHighLimit(PT1035_t *pt1035);

void pt1035SetLowLimit(PT1035_t *pt1035, int16_t lowLimitTemp);

double pt1035GetLowLimit(PT1035_t *pt1035);

uint16_t pt1035AlertAndDataReady(PT1035_t *pt1035, _Bool *arr);

double pt1035GetTemperature(PT1035_t *pt1035);


#endif
