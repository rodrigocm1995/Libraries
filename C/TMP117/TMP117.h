/**
    *******************************************************************************************
  * @file           : TMP117.h
  * @brief          : TMP117 Library
    *******************************************************************************************

  * The TMP117 is a high-precision digital temperature sensor. It is designed to meet ASTM E1112
  * and ISO 80601 requirements for electronic patient thermometers. The TMP117 provides a 16-bit
  * temperature result with a resolution of 0.0078 °C and an accurary of up to ±0.1 °C across
  * the temperature range of -20 °C to 50 °C with no calibration. Integrated EEPROM is included 
  * for device programming with an additional 48-bits memory available for general use. The TMP-
  * 117 has in interface that is I²C compatible, programmable alert functionality, and the device
  * can support up to four devices on a single bus.
  * 
  * The low power consumption of the TMP117 minimizes the impact of self-heating on measurement
  * accurary. The TMP117 operates from 1.7 V to 5.5 V and tipically consumes 3.5 µA.
  *   
  * @details
  * TMP117 high-accuracy temperature sensor
  * - ±0.1 °C (maximum) from -20 °C to 50 °C
  * - ±0.15 °C (maximum) from -40 °C to 70 °C
  * - ±0.2 °C (maximum) from  -40 °C to 100 °C
  * - ±0.25 °C (maximum) from -55 °C to 125 °C
  * - ±0.3 °C (maximum) from -55 °C to 150 °C
  * Operating temperature range: -55 °C to 150°C
  * Low power consumption:
  * - 3.5 µA, 1 Hz conversion cycle 
  * - 150 nA shutdown current
  * Supply Range:
  * - 1.7 V to 5.5 V from -55 °C to 70 °C
  * - 1.8 V to 5.5 V from -55 °C to 150 °C
  * 16-bit resolution: 0.0078 °C (1 LSB)
  * Programmable temperature alert limits 
  * Selectable averaging
  * Digital offset for system correction
  * General-purpose EEPROM 48 bits
  * NIST traceability
  * Medical grade: meets ASTM E1112 and ISO 80601-2-56
  * RTDs replacement: PT100, PT500, PT1000
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_TMP117_H_
#define INC_TMP117_H_

#define TMP117_DEFAULT_ADDRESS                  0x48
#define TMP117_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define TMP117_TEMP_RESULT_REGISTER             0x00
#define TMP117_CONFIGURATION_REGISTER           0x01
#define TMP117_THIGH_LIMIT_REGISTER             0x02
#define TMP117_TLOW_LIMIT_REGISTER              0x03
#define TMP117_EEPROM_UNLOCK_REGISTER           0x04
#define TMP117_EEPROM1_REGISTER                 0x05
#define TMP117_EEPROM2_REGISTER                 0x06
#define TMP117_TEMP_OFFSET_REGISTER             0x07
#define TMP117_EEPROM3_REGISTER                 0x08
#define TMP117_DEVICE_ID_REGISTER               0x0F

#define TMP117_EEPROM_EUN                       0x8000

typedef enum
{
  MOD_CONTINUOUS_CONVERSION                   = 0x0000,
  MOD_SHUNTDOWN                               = 0x0400,
  MOD_CONTINUOS_CONVERSION1                   = 0x0800,
  MOD_ONE_SHOT                                = 0x0C00
}tmp117ConversionMode_t;


typedef enum
{
  CONVERSION_TYPE_0                           = 0x0000,
  CONVERSION_TYPE_1                           = 0x0080,
  CONVERSION_TYPE_2                           = 0x0100,
  CONVERSION_TYPE_3                           = 0x0180,
  CONVERSION_TYPE_4                           = 0x0200,
  CONVERSION_TYPE_5                           = 0x0280,
  CONVERSION_TYPE_6                           = 0x0300,
  CONVERSION_TYPE_7                           = 0x0380,
}tmp117ConversionCycleTime_t;

typedef enum
{
  NO_AVERAGING_MODE                           = 0x0000,
  AVERAGED_CONVERSIONS_8                      = 0x0020,
  AVERAGED_CONVERSIONS_32                     = 0x0040,
  AVERAGED_CONVERSIONS_64                     = 0x0060  
} tmp117AveragingMode_t;

typedef enum
{
  THERM_MODE                                  = 0x0010,
  ALERT_MODE                                  = 0x0000,
} tmp117ThermAlertMode_t;

typedef enum
{
  ACTIVE_HIGH_POLARITY                        = 0x0008,
  ACTIVE_LOW_POLARITY                         = 0x0000,
} tmp117Polarity_t;

typedef enum
{
  ALERT_DATAREADY_FLAG_STATUS                 = 0x0004,
  ALERT__FLAGS_STATUS                         = 0x0000,
}tmp117Alert_t;
// To monitor the state of the Data_Ready flag on the ALERT pin

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t devAddress;
}Tmp117_t;


void writeRegister(Tmp117_t *tmp117, uint8_t registerAddress, uint16_t value);

uint16_t readRegister(Tmp117_t *tmp117, uint8_t registerAddress);

uint8_t tmp117DefaultInit(Tmp117_t *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress);

uint16_t tmp117Init(Tmp117_t *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress, tmp117ConversionMode_t conversionMode, tmp117ConversionCycleTime_t conversionCycle, tmp117AveragingMode_t averagingMode, tmp117ThermAlertMode_t mode, tmp117Polarity_t polarity, tmp117Alert_t alert);

void tmp117SetHighLimit(Tmp117_t *tmp117, int8_t highLimitTemp);

double tmp117ReadHighLimit(Tmp117_t *tmp117);

void tmp117SetLowLimit(Tmp117_t *tmp117, int8_t lowLimitTemp);

double tmp117ReadLowLimit(Tmp117_t *tmp117);

double tmp117checkAndGetTemp(uint16_t value);

uint16_t tmp117AlertAndDataReady(Tmp117_t *tmp117, _Bool *arr);

double tmp117ReadTemperature(Tmp117_t *tmp117);


#endif
