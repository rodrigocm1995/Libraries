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

#define TMP117_ADDRESS                          0x48
#define TMP117_TRIALS                           5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define TMP117_TEMP_RESULT_REG                  0x00
#define TMP117_CONFIGURATION_REG                0x01
#define TMP117_TEMP_HIGH_LIMIT_REG              0x02
#define TMP117_TEMP_LOW_LIMIT_REG               0x03
#define TMP117_EEPROM_UNLOCK_REG                0x04
#define TMP117_EEPROM1_REG                      0x05
#define TMP117_EEPROM2_REG                      0x06
#define TMP117_TEMP_OFFSET_REG                  0x07
#define TMP117_EEPROM3_REG                      0x08
#define TMP117_DEVICE_ID_REG                    0x0F
#define TMP117_EEPROM_EUN                       0x8000



// Masks
#define TMP117_MODE_MASK						0xF3FF
#define TMP117_AVERAGE_MASK						0xFF9F
#define TMP117_THERM_ALERT_MASK					0xFFEF
#define TMP117_ALERT_PIN_POL_MASK				0xFFF7
#define TMP117_ALERT_PIN_SELECT_MASK			0xFFFB
#define TMP117_SOFT_RESET_MASK					0xFFFD
#define TMP117_CONV_CYCLE_MASK					0xFC7F

typedef enum
{
  TMP117_CONTINUOUS_MODE                   		= 0x0000,
  TMP117_SHUTDOWN_MODE                          = 0x0400,
  TMP117_ONE_SHOT_MODE                   		= 0x0C00,
}TMP117_Mode_HandleTypeDef;

typedef enum
{
  TMP117_CONVERSION_TYPE_0                      = 0x0000,
  TMP117_CONVERSION_TYPE_1                      = 0x0080,
  TMP117_CONVERSION_TYPE_2                      = 0x0100,
  TMP117_CONVERSION_TYPE_3                      = 0x0180,
  TMP117_CONVERSION_TYPE_4                      = 0x0200,
  TMP117_CONVERSION_TYPE_5                      = 0x0280,
  TMP117_CONVERSION_TYPE_6                      = 0x0300,
  TMP117_CONVERSION_TYPE_7                      = 0x0380,
}TMP117_ConversionCycle_HandleTypeDef;

typedef enum
{
  TMP117_NO_AVERAGING                         	= 0x0000,
  TMP117_AVG_8_CONV								= 0x0020,
  TMP117_AVG_32_CONV	                    	= 0x0040,
  TMP117_AVG_64_CONV	                     	= 0x0060,
} TMP117_Average_HandleTypeDef;

typedef enum
{
  TMP117_THERM_MODE                             = 0x0010,
  TMP117_ALERT_MODE                             = 0x0000,
} TMP117_ThermAlertMode_HandleTypeDef;

typedef enum
{
  TMP117_ALERT_ACTIVE_HIGH                      = 0x0008,
  TMP117_ALERT_ACTIVE_LOW                       = 0x0000,
} TMP117_AlertPinPolarity_HandleTypeDef;

typedef enum
{
  TMP117_ALERT_FOR_DATA_READY_FLAG              = 0x0004,
  TMP117_ALERT_FOR_ALERT_FLAGS                  = 0x0000,
}TMP117_AlertPinSelect_HandleTypeDef;


typedef struct
{
  I2C_HandleTypeDef         *hi2c;
  uint8_t                   devAddress;
}TMP117_HandleTypeDef;


HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value);

uint16_t TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress);


int16_t TMP117_GetTemperature(TMP117_HandleTypeDef *tmp117);

uint16_t TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117);

uint16_t TMP117_GetTempHighLimit(TMP117_HandleTypeDef *tmp117);

uint16_t TMP117_GetTempLowLimit(TMP117_HandleTypeDef *tmp117);

uint16_t TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117);


_Bool TMP117_HighAlertFlag(TMP117_HandleTypeDef *tmp117);

_Bool TMP117_LowAlertFlag(TMP117_HandleTypeDef *tmp117);

_Bool TMP117_DataReadyFlag(TMP117_HandleTypeDef *tmp117);

_Bool TMP117_EepromBusyFlag(TMP117_HandleTypeDef *tmp117);


void TMP117_SetMode(TMP117_HandleTypeDef *tmp117, TMP117_Mode_HandleTypeDef mode);

void TMP117_SetConversionCycle(TMP117_HandleTypeDef *tmp117, TMP117_ConversionCycle_HandleTypeDef conv);

void TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Average_HandleTypeDef avg);

void TMP117_SetThermAlertMode(TMP117_HandleTypeDef *tmp117, TMP117_ThermAlertMode_HandleTypeDef tnA);

void TMP117_SetAlertPinPolarity(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinPolarity_HandleTypeDef pol);

void TMP117_SetAlertPinSelect(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinSelect_HandleTypeDef select);

void TMP117_SetSoftReset(TMP117_HandleTypeDef *tmp117);


void TMP117_SetHighLimitTemp(TMP117_HandleTypeDef *tmp117, uint8_t highLimit);

void TMP117_SetLowLimitTemp(TMP117_HandleTypeDef *tmp117, uint8_t lowLimit);


double TMP117_CheckTemperature(uint16_t value);

double TMP117_GetTemperatureCelsius(TMP117_HandleTypeDef *tmp117);


void TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *hi2c);

#endif
