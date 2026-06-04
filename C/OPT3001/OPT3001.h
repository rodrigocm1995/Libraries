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

/*******************  Bits definition for CONFIGURATION register  ******************/
#define OPT3001_FC_Pos                  (0U)
#define OPT3001_FC_Mask                 (0x3U << OPT3001_FC_Pos)
#define OPT3001_FC                      OPT3001_FC_Mask

#define OPT3001_ME_Pos                  (2U)
#define OPT3001_ME_Mask                 (0x1U << OPT3001_ME_Pos)
#define OPT3001_ME                      OPT3001_ME_Mask

#define OPT3001_POL_Pos                 (3U)
#define OPT3001_POL_Mask                (0x1U << OPT3001_POL_Pos)
#define OPT3001_POL                     OPT3001_POL_Mask

#define OPT3001_L_Pos                   (4U)
#define OPT3001_L_Mask                  (0x1U << OPT3001_L_Pos)
#define OPT3001_L                       OPT3001_L_Mask

#define OPT3001_FL_Pos                  (5U)
#define OPT3001_FL_Mask                 (0x1U << OPT3001_FL_Pos)
#define OPT3001_FL                      OPT3001_FL_Mask

#define OPT3001_FH_Pos                  (6U)
#define OPT3001_FH_Mask                 (0x1U << OPT3001_FH_Pos)
#define OPT3001_FH                      OPT3001_FH_Mask

#define OPT3001_CRF_Pos                 (7U)
#define OPT3001_CRF_Mask                (0x1U << OPT3001_CRF_Pos)
#define OPT3001_CRF                     OPT3001_CRF_Mask

#define OPT3001_OVF_Pos                 (8U)
#define OPT3001_OVF_Mask                (0x1U << OPT3001_OVF_Pos)
#define OPT3001_OVF                     OPT3001_OVF_Mask

#define OPT3001_M_Pos                   (9U)
#define OPT3001_M_Mask                  (0x3U << OPT3001_M_Pos)
#define OPT3001_M                       OPT3001_M_Mask

#define OPT3001_CT_Pos                  (10U)
#define OPT3001_CT_Mask                 (0x1U << OPT3001_CT_Pos)
#define OPT3001_CT                      OPT3001_CT_Mask

#define OPT3001_RN_Pos                  (12U)
#define OPT3001_RN_Mask                 (0xFU << OPT3001_RN_Pos)
#define OPT3001_RN                      OPT3001_RN_Mask

/*******************  Bits definition for RESULT register  ******************/
#define OPT3001_E_Pos                   (12U)
#define OPT3001_E_Mask                  (0xFU << OPT3001_E_Pos)
#define OPT3001_E                       OPT3001_E_Mask

#define OPT3001_R_Pos                   (0U)
#define OPT3001_R_Mask                  (0xFFFU << OPT3001_R_Pos)
#define OPT3001_R                       OPT3001_R_Mask

typedef enum
{
	OPT3001_40_95_LUX					= 0x0U,
	OPT3001_81_90_LUX					= 0x1U,
	OPT3001_163_80_LUX					= 0x2U,
	OPT3001_327_60_LUX					= 0x3U,
	OPT3001_655_20_LUX					= 0x4U,
	OPT3001_1310_40_LUX					= 0x5U,
	OPT3001_2620_80_LUX					= 0x6U,
	OPT3001_5241_60_LUX					= 0x7U,
	OPT3001_10483_20_LUX				= 0x8U,
	OPT3001_20966_40_LUX				= 0x9U,
	OPT3001_41932_80_LUX				= 0xAU,
	OPT3001_83865_60_LUX				= 0xBU,
	OPT3001_AUTOMATIC_RANGE				= 0xCU,
}OPT3001_FullScaleRange_TypeDef;

typedef enum
{
	OPT3001_100_MS				    	= 0x0U,
	OPT3001_800_MS				    	= 0x1U, 
}OPT3001_ConvTime_TypeDef;

typedef enum
{
	OPT3001_SHUTDOWN_MODE				= 0x0U,
	OPT3001_SINGLE_SHOT_MODE		    = 0x1U,
	OPT3001_CONTINUOUS_MODE		        = 0x2U,
}OPT3001_Mode_TypeDef;

typedef enum
{
    OPT3001_LATCH_HYSTERESIS            = 0x0U, /* Hysteresis style: INT pin and flags clear when returning to normal limits */
    OPT3001_LATCH_LATCHED               = 0x1U, /* Latch style: INT pin and flags remain asserted until Configuration register is read */
} OPT3001_Latch_TypeDef;

typedef enum
{
	OPT3001_ALERT_ACTIVE_LOW			= 0x0U,
	OPT3001_ALERT_ACTIVE_HIGH			= 0x1U,
}OPT3001_AlertPinPol_TypeDef;


typedef enum
{
	OPT3001_MASK_EXP_ACTIVE				= 0x0004,
	OPT3001_MASK_EXP_DISABLED			= 0x0000,
}OPT3001_MaskExp_HandleTypeDef;

typedef enum
{
	OPT3001_ONE_FAULT_COUNT				= 0x0U,
	OPT3001_TWO_FAULT_COUNTS			= 0x1U,
	OPT3001_FOUR_FAULT_COUNTS			= 0x2U,
	OPT3001_EIGHT_FAULT_COUNTS			= 0x3U,
}OPT3001_FaultCount_TypeDef;

typedef struct
{
	I2C_HandleTypeDef					*hi2c;
	uint8_t								_devAddress;
	OPT3001_FullScaleRange_TypeDef 	    _innerExponent;
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

uint16_t OPT3001_GetHighLimit(OPT3001_HandleTypeDef *opt3001);

uint16_t OPT3001_GetManufacturerId(OPT3001_HandleTypeDef *opt3001);

uint16_t OPT3001_GetDeviceId(OPT3001_HandleTypeDef *opt3001);
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

void OPT3001_SetFaultCount(OPT3001_HandleTypeDef *opt3001, OPT3001_FaultCount_TypeDef faultCount);

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



/*
===========================================================================================================================
Set the exponent to calculate the upper and lower limits
===========================================================================================================================
*/
HAL_StatusTypeDef OPT3001_GetInnerRange(OPT3001_HandleTypeDef *opt3001, double limit);
/*
===========================================================================================================================
End of Set the exponent to calculate the upper and lower limits
===========================================================================================================================
*/



/*
===========================================================================================================================
Getting the upper and lower limits of the device
===========================================================================================================================
*/
double OPT3001_GetLowLimitLux(OPT3001_HandleTypeDef *opt3001);

double OPT3001_GetHighLimitLux(OPT3001_HandleTypeDef *opt3001);
/*
===========================================================================================================================
End of Getting the upper and lower limits of the device
===========================================================================================================================
*/


/*
===========================================================================================================================
Getting the lux value
===========================================================================================================================
*/
double OPT3001_GetLux(OPT3001_HandleTypeDef *opt3001);
/*
===========================================================================================================================
End of Getting the lux value
===========================================================================================================================
*/

#endif
