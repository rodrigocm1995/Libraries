/**
    *******************************************************************************************
  * @file           : TMP118.h
  * @brief          : TMP118 Library
    *******************************************************************************************

  * The TMP118 is the industry smallest temperature sensor in an industry leading 0.55mm × 0.61mm
  * × 0.24mm PICOSTAR package. The TMP118 provides a 16-bit temperature result with a resolution
  * of 0.007878125°C and an accuracy of up to ±0.1°C across the temperature range of 0°C to 50°C
  * with no additional calibration. The device is designed to help meet the system level ASTM
  * E1112  and ISO 80601 accurary requirements for electronic patient thermometers.
  * 
  * The TMP118 is designed to operate from a supply voltage a low as 1.4V, with a low average and 
  * shuntdown current of 1.4µA (at 1Hz, no averaging) and 80nA, respectively, allowing for a on-
  * demand temperature conversion to maximize battery life. The TMP118 has an interface that is 
  * I²C- and SMBus- compatible and has programmable alert functionality. Logic level as low as 1V
  * can be supported regardless of the main supply rail, enabling interoperability with a low voltage
  * 1.2V controller without an additional level shifter
  *   
  * 
  *   
  * @details
  * Ultra-small, ultra-thin PicoStar package:
  * - Size: 0.55mm × 0.61mm × 0.24mm
  * - Small thermal mass: 0.14 mJ/°C
  * TMP118 high-accuracy temperature sensor:
  * - ±0.05°C (typical) from 0°C to 50°C
  * - ±0.1°C (maximum) from 0°C to 50°C
  * - ±0.2°C (maximum) from -20°C to 85°C
  * - ±0.4°C (maximum) from -40°C to 125°C
  * Low power consumption:
  * - 55µA active current
  * - 80nA shuntdown current
  * - 1.4µA average current, 1Hz conversion cycle
  * Supply range: 1.4V to 5.5V
  * 1.2 V logic compatible (independent of supply)
  * 3 averaging options to reduce measurement noise 
  * 16-bit resolution: 0.0078125°C (LSB)
  * I²C and SMBus compatible
  * I3C Mixed Bus co-existence
  * Programmable temperature alert limits
  * NIST traceability with unique device IDs
  * ---------------------------------------------------------------------
  * |DEVICE             |      HEX       |       BINARY                 |
  * |-------------------------------------------------------------------- 
  * |TMP118A/ TMP118MA  |      0x48      |       1001000'b              |
  * |TMP118B/ TMP118MB  |      0x49      |       1001001'b              |
  * |TMP118C/ TMP118MC  |      0x4A      |       1001010'b              |
  * |TMP118D/ TMP118MC  |      0x4B      |       1001011'b              |
  * |-------------------------------------------------------------------|
  *******************************************************************************************
  */

#ifndef INC_TMP118_H_
#define INC_TMP118_H_

// TMP118 Addresses
#define TMP118A_ADDRESS                      0x48
#define TMP118B_ADDRESS                      0x49
#define TMP118C_ADDRESS                      0x4A   
#define TMP118D_ADDRESS                      0x4B

#define TMP118_TRIALS                           5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// TMP118 Registers
#define TMP118_TEMP_RESULT_REGISTER             0x00
#define TMP118_CONFIGURATION_REGISTER           0x01
#define TMP118_TLOW_LIMIT_REGISTER              0x02
#define TMP118_THIGH_LIMIT_REGISTER             0x03
#define TMP118_DEVICE_ID_REGISTER               0x0B
#define TMP118_UNIQUE_ID0                       0x0C
#define TMP118_UNIQUE_ID1                       0x0D
#define TMP118_UNIQUE_ID2                       0x0E

#define TMP118_LSB                              7.8125E-3

typedef enum
{
  TMP118_ONE_SHOT                             = 0x0000,
  TMP118_CONTINUOUS                           = 0x8000,
}tmp118ConversionMode;


typedef enum
{
  TMP118_ONE_FAULT                            = 0x0000,
  TMP118_TWO_FAULTS                           = 0x0800,
  TMP118_FOUR_FAULTS                          = 0x1000,
  TMP118_SIX_FAULTS                           = 0x1800,
}tmp118Fault;

typedef enum
{
  TMP118_ACTIVE_HIGH_POLARITY                 = 0x0000,
  TMP118_ACTIVE_LOW_POLARITY                  = 0x0400,
} tmp117Polarity;


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

uint16_t tmp117Init(Tmp117_t *tmp117, I2C_HandleTypeDef *i2c, uint8_t devAddress, ConversionMode_t conversionMode, ConversionCycleTime_t conversionCycle, AveragingMode_t averagingMode, ThermAlertMode_t mode, Polarity_t polarity, Alert_t alert);

void tmp117SetHighLimit(Tmp117_t *tmp117, int8_t highLimitTemp);

double tmp117ReadHighLimit(Tmp117_t *tmp117);

void tmp117SetLowLimit(Tmp117_t *tmp117, int8_t lowLimitTemp);

double tmp117ReadLowLimit(Tmp117_t *tmp117);

double tmp117checkAndGetTemp(uint16_t value);

uint16_t tmp117AlertAndDataReady(Tmp117_t *tmp117, _Bool *arr);

double tmp117ReadTemperature(Tmp117_t *tmp117);


#endif

