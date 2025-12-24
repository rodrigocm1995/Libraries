/**
    *******************************************************************************************
  * @file           : OPT3001.h
  * @brief          : OPT3001 Library
    *******************************************************************************************

  * The OPT3001 is a sensor that measures the intensity of visible light. The spectral response
  * of the sensor tightly matches the photopic response of the human eye and includes singific-
  * ant infrared rejection.
  * The strong IR rejection also aids in maintaining high accuracy when industrial design calls
  * for mounting the sensor under dark glass for aesthetics. It is an ideal preferred replacem-
  * ent for photodiodes, photoresistors, or other ambient light sensors with less human eye ma-
  * tching and IR rejection.
  * Measurements can be made from 0.01 lux up to 83k lux without manually selecting full-scale 
  * ranges by using the built-in, full-scale setting feature. This capability allows light mea-
  * surement over a 23-bit effective dynamic range.
  *   
  * @details
  * Precision Optical Filtering to Match Human Eye: Rejects > 99% (typ) of IR
  * Automatic Full-Scale Setting Feature Simplifies Software and Ensures proper configuration.
  * Measurements: 0.01 lux to 83k lux
  * 23-bit effective dynamic range with automatic gain ranging
  * 12 binary-weighted full-scale range settings: < 0.2% (typ) matching between ranges.
  * Low operating current: 1.8uA (typ)
  * Operating temperature range: -40°C to +85°C.
  * Wide power-supply range: 1.6V to 3.6V
  * 5.5V Tolerant I/O
  * Flexible Interrupt System
  * 
  * @example
  * 
  * The formula to translate the result register's content into lux is given by equation 1:
  * 
  * lux = LSB_Size x R[11:0]                                                              (1) 
  * 
  * Where:
  *   LSB_Size = 0.01 x 2^E[3:0]                                                          (2)
  * 
  * The complete lux equation is shown in equation 3
  * 
  * lux = 0.01 x 2^E[3:0] x R[11:0]

  *******************************************************************************************
  */

#ifndef INC_OPT3001_H_
#define INC_OPT3001_H_

#define OPT3001_ADDRESS		0x44
#define OPT3001_TRIALS		5
#define CHECK_BIT(var,pos)	((var) & (1<<(pos)))

// Registers
#define OPT3001_RESULT_REG				0x00
#define OPT3001_CONFIGURATION_REG		0x01
#define OPT3001_LOW_LIMIT_REG			0x02
#define OPT3001_HIGH_LIMIT_REG			0x03
#define OPT3001_MANUFACTURER_ID_REG		0x7E
#define OPT3001_DEVICE_ID_REG			0x7F

// Mask
#define OPT3001_RANGE_NUMBER_MASK		0x0FFF
#define OPT3001_CONV_TIME_MASK			0xF7FF
#define OPT3001_CONV_MODE_MASK			0xF9FF
#define OPT3001_LATCH_STYLE_MASK		0xFFEF
#define OPT3001_INT_POLARITY_MASK		0xFFF7
#define OPT3001_EXP_MASK				0xFFFB
#define OPT3001_FAULT_COUNT_MASK		0xFFFC

typedef enum
{
	OPT3001_40_95_LUX					= 0x0000,
	OPT3001_81_90_LUX					= 0x1000,
	OPT3001_163_80_LUX					= 0x2000,
	OPT3001_327_60_LUX					= 0x3000,
	OPT3001_655_20_LUX					= 0x4000,
	OPT3001_1310_40_LUX					= 0x5000,
	OPT3001_2620_80_LUX					= 0x6000,
	OPT3001_5241_60_LUX					= 0x7000,
	OPT3001_10483_20_LUX				= 0x8000,
	OPT3001_20966_40_LUX				= 0x9000,
	OPT3001_41932_80_LUX				= 0xA000,
	OPT3001_83865_60_LUX				= 0xB000,
	OPT3001_AUTOMATIC_RANGE				= 0xC000,
}OPT3001_RangeNumber_HandleTypeDef;

typedef enum
{
	OPT3001_CT_100_MS					= 0x0000,
	OPT3001_CT_800_MS					= 0x0800, 
}OPT3001_ConversionTime_HandleTypeDef;

typedef enum
{
	OPT3001_SHUTDOWN					= 0x0000,
	OPT3001_SINGLE_SHOT					= 0x0200,
	OPT3001_CONTINUOUS_CONVERSION		= 0x0400,
}OPT3001_ConversionMode_HandleTypeDef;

typedef enum
{
	OPT3001_HYSTERESIS_STYLE			= 0x0000,
	OPT3001_WINDOW_STYLE				= 0x0000,
}OPT3001_LatchStyle_HandleTypeDef;

typedef enum
{
	OPT3001_INT_ACTIVE_LOW				= 0x0000,
	OPT3001_INT_ACTIVE_HIGH				= 0x0008,
}OPT3001_Int_Polarity_HandleTypeDef;


typedef enum
{
	OPT3001_MASK_EXP_ACTIVE				= 0x0004,
	OPT3001_MASK_EXP_DISABLED			= 0x0000,
}OPT3001_MaskExp_HandleTypeDef;

typedef enum
{
	OPT3001_ONE_FAULT_COUNT				= 0x0000,
	OPT3001_TWO_FAULT_COUNTS			= 0x0001,
	OPT3001_FOUR_FAULT_COUNTS			= 0x0002,
	OPT3001_EIGHT_FAULT_COUNTS			= 0x0003,
}OPT3001_FaultCount_HandleTypeDef;

typedef struct
{
	I2C_HandleTypeDef					*hi2c;
	uint8_t								devAddress;
	OPT3001_RangeNumber_HandleTypeDef 	innerExponent;
}OPT3001_HandleTypeDef;


/*
===========================================================================================================================
Read/Write functions
===========================================================================================================================
*/
HAL_StatusTypeDef OPT3001_WriteRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress, uint16_t value);

uint16_t OPT3001_ReadRegister(OPT3001_HandleTypeDef *opt3001, uint8_t registerAddress);
/*
===========================================================================================================================
End of Read/Write functions
===========================================================================================================================
*/


/*
===========================================================================================================================
Initializing device
===========================================================================================================================
*/
void OPT3001_Init(OPT3001_HandleTypeDef *opt3001, I2C_HandleTypeDef *i2c);
/*
===========================================================================================================================
End of Initializing device
===========================================================================================================================
*/


/*
===========================================================================================================================
Getting register values
===========================================================================================================================
*/
uint16_t OPT3001_GetResult(OPT3001_HandleTypeDef *opt3001);

uint16_t OPT3001_GetConfiguration(OPT3001_HandleTypeDef *opt3001);

uint16_t OPT3001_GetLowLimit(OPT3001_HandleTypeDef *opt3001);
/*
===========================================================================================================================
End of Getting register values
===========================================================================================================================
*/



/*
===========================================================================================================================
Setting CONFIGURATION register bits
===========================================================================================================================
*/
void OPT3001_SetRangeNumber(OPT3001_HandleTypeDef *opt3001, OPT3001_RangeNumber_HandleTypeDef range);

void OPT3001_SetConversionTime(OPT3001_HandleTypeDef *opt3001, OPT3001_ConversionTime_HandleTypeDef);

void OPT3001_SetConversionMode(OPT3001_HandleTypeDef *opt3001, OPT3001_ConversionMode_HandleTypeDef mode);

void OPT3001_SetLatchStyle(OPT3001_HandleTypeDef *opt3001, OPT3001_LatchStyle_HandleTypeDef latch);

void OPT3001_SetIntPolarity(OPT3001_HandleTypeDef *opt3001, OPT3001_Int_Polarity_HandleTypeDef pol);

void OPT3001_SetExponentFIeld(OPT3001_HandleTypeDef *opt3001, OPT3001_MaskExp_HandleTypeDef maskExp);

void OPT3001_SetFaultCount(OPT3001_HandleTypeDef *opt3001, OPT3001_FaultCount_HandleTypeDef faultCount);

/*
===========================================================================================================================
End of Setting CONFIGURATION register bits
===========================================================================================================================
*/


/*
===========================================================================================================================
CONFIGURATION register bits flags
===========================================================================================================================
*/
_Bool OPT3001_OverflowFlag(OPT3001_HandleTypeDef *opt3001);

_Bool OPT3001_ConversionReadyFlag(OPT3001_HandleTypeDef *opt3001);

_Bool OPT3001_HighLimitFlag(OPT3001_HandleTypeDef *opt3001);

_Bool OPT3001_LowLimitFlag(OPT3001_HandleTypeDef *opt3001);
/*
===========================================================================================================================
End of CONFIGURATION register bits flags
===========================================================================================================================
*/


/*
===========================================================================================================================
Setting the lower comparison of the LOW_LIMIT Register
===========================================================================================================================
*/

HAL_StatusTypeDef OPT3001_SetLowLimit(OPT3001_HandleTypeDef *opt3001, double lowLimit);
/*
===========================================================================================================================
End of Setting the lower comparison of the LOW_LIMIT Register
===========================================================================================================================
*/


/*
===========================================================================================================================
Setting the high comparison of the HIGH_LIMIT Register
===========================================================================================================================
*/
HAL_StatusTypeDef OPT3001_SetHighLimit(OPT3001_HandleTypeDef *opt3001, double highLimit);
/*
===========================================================================================================================
End of Setting the high comparison of the HIGH_LIMIT Register
===========================================================================================================================
*/


double OPT3001_GetLux(OPT3001_HandleTypeDef *opt3001);

HAL_StatusTypeDef OPT3001_GetInnerRange(OPT3001_HandleTypeDef *opt3001, double limit);


double OPT3001_GetLowLimitLux(OPT3001_HandleTypeDef *opt3001);

double OPT3001_GetHighLimitLux(OPT3001_HandleTypeDef *opt3001);

#endif
