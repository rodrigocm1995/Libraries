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

/*******************  Bits definition for CONFIGURATION register  ******************/
#define TMP117_SOFTRESET_Pos                    (1U)
#define TMP117_SOFTRESET_Mask                   (0x1U << TMP117_SOFTRESET_Pos)
#define TMP117_SOFTRESET                        TMP117_SOFTRESET_Mask

#define TMP117_DRALERT_Pos                      (2U)
#define TMP117_DRALERT_Mask                     (0x1U << TMP117_DRALERT_Pos)
#define TMP117_DRALERT                          TMP117_DRALERT_Mask

#define TMP117_POL_Pos                          (3U)
#define TMP117_POL_Mask                         (0x1U << TMP117_POL_Pos)
#define TMP117_POL                              TMP117_POL_Mask

#define TMP117_TnA_Pos                          (4U)
#define TMP117_TnA_Mask                         (0x1U << TMP117_TnA_Pos)
#define TMP117_TnA                              TMP117_TnA_Mask

#define TMP117_AVG_Pos                          (5U)
#define TMP117_AVG_Mask                         (0x3U << TMP117_AVG_Pos)
#define TMP117_AVG                              TMP117_AVG_Mask

#define TMP117_CONV_Pos                         (7U)
#define TMP117_CONV_Mask                        (0x7U << TMP117_CONV_Pos)
#define TMP117_CONV                             TMP117_CONV_Mask

#define TMP117_MOD_Pos                          (10U)
#define TMP117_MOD_Mask                         (0x3U << TMP117_MOD_Pos)
#define TMP117_MOD                              TMP117_MOD_Mask

#define TMP117_EEPROMBUSY_Pos                   (12U)
#define TMP117_EEPROMBUSY_Mask                  (0x1U << TMP117_EEPROMBUSY_Pos)
#define TMP117_EEPROMBUSY                       TMP117_EEPROMBUSY_Mask

#define TMP117_DATAREADY_Pos                    (13U)
#define TMP117_DATAREADY_Mask                   (0x1U << TMP117_DATAREADY_Pos)
#define TMP117_DATAREADY                        TMP117_DATAREADY_Mask

#define TMP117_LOWALERT_Pos                     (14U)
#define TMP117_LOWALERT_Mask                    (0x1U << TMP117_LOWALERT_Pos)
#define TMP117_LOWALERT                         TMP117_LOWALERT_Mask

#define TMP117_HIGHALERT_Pos                    (15U)
#define TMP117_HIGHALERT_Mask                   (0x1U << TMP117_HIGHALERT_Pos)
#define TMP117_HIGHALERT                        TMP117_HIGHALERT_Mask


typedef enum
{
  TMP117_CONTINUOUS_MODE                   		= 0x0U,
  TMP117_SHUTDOWN_MODE                          = 0x1U,
  TMP117_ONE_SHOT_MODE                   		= 0x3U,
}TMP117_Mode_TypeDef;

typedef enum
{
  TMP117_CONV_15_5_MS                           = 0x0U,
  TMP117_CONV_125_MS                            = 0x1U,
  TMP117_CONV_250_MS                            = 0x2U,
  TMP117_CONV_500_MS                            = 0x3U,
  TMP117_CONV_1_S                               = 0x4U,
  TMP117_CONV_4_S                               = 0x5U,
  TMP117_CONV_8_S                               = 0x6U,
  TMP117_CONV_16_S                              = 0x7U,
} TMP117_ConvTime_TypeDef;

typedef enum
{
  TMP117_NO_SAMPLES                         	= 0x0U,
  TMP117_8_SAMPLES								= 0x1U,
  TMP117_32_SAMPLES	                    	    = 0x2U,
  TMP117_64_SAMPLES	                     	    = 0x3U,
} TMP117_Avg_TypeDef;

typedef enum
{
  TMP117_THERM_MODE                             = 0x1U,
  TMP117_ALERT_MODE                             = 0x0U,
} TMP117_ThermAlertMode_TypeDef;

typedef enum
{
  TMP117_ALERT_ACTIVE_HIGH                      = 0x0U,
  TMP117_ALERT_ACTIVE_LOW                       = 0x1U,
} TMP117_AlertPinPol_TypeDef;

typedef enum
{
  TMP117_ALERT_FOR_DATA_READY_FLAG              = 0x1U,
  TMP117_ALERT_FOR_ALERT_FLAGS                  = 0x0U,
}TMP117_DRALERT_TypeDef;


typedef enum
{
	TMP117_LOCK_EEPROM							= 0x0000,
	TMP117_UNLOCK_EEPROM						= 0x8000,
}TMP117_LockUnlock_TypeDef;

typedef struct
{
    I2C_HandleTypeDef         *hi2c;
    uint8_t                   _devAddress;
    uint8_t                  _samples;
} TMP117_HandleTypeDef;


/* Register Write & Read */
HAL_StatusTypeDef TMP117_WriteRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress, uint16_t value);
uint16_t TMP117_ReadRegister(TMP117_HandleTypeDef *tmp117, uint8_t registerAddress);

/* Configuration Getters */
uint16_t TMP117_GetConfiguration(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetTempHighLimit(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetTempLowLimit(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetDeviceId(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetEepromUnlock(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetEeprom1(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetEeprom2(TMP117_HandleTypeDef *tmp117);
uint16_t TMP117_GetEeprom3(TMP117_HandleTypeDef *tmp117);

/* Diagnostic Readings */
_Bool TMP117_EepromBusyFlag(TMP117_HandleTypeDef *tmp117);
double TMP117_GetHighLimitTemp(TMP117_HandleTypeDef *tmp117);
double TMP117_GetLowLimitTemp(TMP117_HandleTypeDef *tmp117);
double TMP117_CheckTemperature(uint16_t value);
double TMP117_GetTemperatureCelsius(TMP117_HandleTypeDef *tmp117);

/* EEPROM Control */
void TMP117_SetLockUnlockEeprom(TMP117_HandleTypeDef *tmp117, TMP117_LockUnlock_HandleTypeDef lockUnlock);
HAL_StatusTypeDef TMP117_SetEeprom1(TMP117_HandleTypeDef *tmp117, uint16_t data);
HAL_StatusTypeDef TMP117_SetEeprom2(TMP117_HandleTypeDef *tmp117, uint16_t data);
HAL_StatusTypeDef TMP117_SetEeprom3(TMP117_HandleTypeDef *tmp117, uint16_t data);

/* Initialization & Control */
void TMP117_Init(TMP117_HandleTypeDef *tmp117, I2C_HandleTypeDef *i2c);
void TMP117_ResetDevice(TMP117_HandleTypeDef *tmp117);
void TMP117_SetAlertPinFunction(TMP117_HandleTypeDef *tmp117, TMP117_DRALERT_TypeDef pinFunction);
void TMP117_SetAlertPinPolarity(TMP117_HandleTypeDef *tmp117, TMP117_AlertPinPol_TypeDef polarity);
void TMP117_SetThermAlertMode(TMP117_HandleTypeDef *tmp117, TMP117_ThermAlertMode_TypeDef tnA);
void TMP117_SetAverage(TMP117_HandleTypeDef *tmp117, TMP117_Avg_TypeDef avg);
void TMP117_SetMode(TMP117_HandleTypeDef *tmp117, TMP117_Mode_TypeDef mode);

/* Status flags and additional configuration */
_Bool TMP117_IsEEPROMBusy(TMP117_HandleTypeDef *tmp117);
_Bool TMP117_IsDataReady(TMP117_HandleTypeDef *tmp117);
_Bool TMP117_IsLowAlertSet(TMP117_HandleTypeDef *tmp117);
_Bool TMP117_IsHighAlertSet(TMP117_HandleTypeDef *tmp117);
HAL_StatusTypeDef TMP117_SetConvTime(TMP117_HandleTypeDef *tmp117, TMP117_ConvTime_TypeDef convTime);
void TMP117_SetHighLimit(TMP117_HandleTypeDef *tmp117, double highLimit);
void TMP117_SetLowLimit(TMP117_HandleTypeDef *tmp117, double lowLimit);
double TMP117_GetTemperature_C(TMP117_HandleTypeDef *tmp117);

#endif
