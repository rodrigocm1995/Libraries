/**
    *******************************************************************************************
  * @file           : LTR-390UV.h
  * @brief          : LTR-390UV Library
    *******************************************************************************************

  * The LTR-0390UV-01 is an integrated low voltage I2C ambient light sensor (ALS) and ultraviolet
  * light sensor (UVS) in a single miniature 2x2 mm chipled lead-free surface mount package.
  * This sensors converts light intensity to a difital output signal capable of direct I2C inter-
  * face. It provides a linear ALS response over a wide dynamic range, and is well suited to 
  * applications under high ambient brightness.
  *
  *   
  * @details
  * Accuracy:
  * - ±0.25°C (typical) from -40°C to +125°C
  * - ±0.5°C (maximum) from -20°C to +100°C
  * - ±1°C (maximum) from from -40°C to +125°C
  * User selectable Measurement Resolution:
  * - +0.5°C, +0.25°C,m +0.125°C, +0.0625°C
  * User-Programmable Temperature Limits:
  * - Temperature Window Limit
  * - Critical Temperature Limit
  * User programmable Temperature Alert Output
  * Operating voltage range: 2.7V to 5.5V
  * Operating Current: 200µA (typical)
  * Shuntdown current: 0.1µA (typical)
  * 2-Wire interface: I2C/SMBus compatible
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_LTR390UV_H_
#define INC_LTR390UV_H_

#define LTR390UV_ADDRESS                        0x53 
#define LTR390UV_TRIALS                         5

// Registers
#define LTR390UV_MAIN_CTRL_REG                  0x00    /* ALS/UVS operation mode control, SW reset */
#define LTR390UV_ALS_UV_MEAS_RATE_REG           0x04    /* ALS/UVS measurement rate and resolution in Acxtive mode */
#define LTR390UV_ALS_UVS_GAIN_REG               0x05    /* ALS/USV analog Gain range */
#define LTR390UV_DEVICE_ID_REG                  0x06    /* Part number ID and revision ID */
#define LTR390UV_MAIN_STATUS_REG                0x07    /* Power-On status, Interrupt status, Data status */
#define LTR390UV_ALS_DATA_0_REG                 0x0D    /* ALS ADC measurement data, LSB */
#define LTR390UV_ALS_DATA_1_REG                 0x0E    /* ALS ADC measurement data */
#define LTR390UV_ALS_DATA_2_REG                 0x0F    /* ALS ADC measurement data, MSB */
#define LTR390UV_INT_PERSIST_REG                0x10    /* UVS ADC measurement data, LSB */
#define LTR390UV_ALS_UVS_THRESS_UP_REG          0x11    /* UVS ADC measurement data */
#define LTR390UV_ALS_UVS_THRESS_LOW_REG         0x12    /* UVS ADC measurement data, MSB */
#define LTR390UV_INT_CFG_REG                    0x19    /* Interrupt configuration */
#define LTR390UV_INT_PERSIST_REG                0x1A    /* Interrupt persist setting */
#define LTR390UV_ALS_UVS_THRESS_UP_0_REG        0x21    /* ALS/UVS interrupt upper threshold, LSB */
#define LTR390UV_ALS_UVS_THRESS_UP_1_REG        0x22    /* ALS/UVS interrupt upper threshold, intervening bits */
#define LTR390UV_ALS_UVS_THRESS_UP_2_REG        0x23    /* ALS/UVS interrupt upper threshold, MSB */
#define LTR390UV_ALS_UVS_THRESS_LOW_0_REG       0x24    /* ALS/UVS interrupt lower threshold, LSB */
#define LTR390UV_ALS_UVS_THRESS_LOW_1_REG       0x25    /* ALS/UVS interrupt lower threshold, intervening bits */
#define LTR390UV_ALS_UVS_THRESS_LOW_2_REG       0x26    /* ALS/UVS interrupt lower threshold, MSB */


// Masks
#define LTR390UV_UVS_MODE_MASK                   0xF7
#define LTR390UV_UVS_ENABLE_MASK                 0xFD
#define LTR390UV_RESOLUTION_MASK				 0x8F
#define LTR390UV_GAIN_MASK						 0xF8

typedef enum
{
	LTR390UV_ALS_MODE                           = 0x00,
	LTR390UV_UVS_MODE                           = 0x08,
}LTR390UV_UVS_Mode_HandleTypeDef;

typedef enum
{
	LTR390UV_ALS_UVS_STANDBY                    = 0x00,
	LTR390UV_ALS_UVS_ACTIVE                     = 0x02,
}LTR390UV_Enable_HandleTypeDef;

typedef enum
{
	LTR390UV_SOFTWARE_RST_DISABLED              = 0x00,
	LTR390UV_SOFTWARE_RST_ENABLED               = 0x10,
}LTR390UV_Software_Rst_HandleTypeDef;

typedef enum
{
	LTR390UV_RES_20BIT							= 0x00,
	LTR390UV_RES_19BIT							= 0x10,
	LTR390UV_RES_18BIT							= 0x20,
	LTR390UV_RES_17BIT							= 0x30,
	LTR390UV_RES_16BIT							= 0x40,
	LTR390UV_RES_13BIT							= 0x50,
}LTR390UV_Resolution_HandleTypeDef;

typedef enum
{
	LTR390UV_GAIN_RANGE_1						= 0x00,
	LTR390UV_GAIN_RANGE_3						= 0x01,
	LTR390UV_GAIN_RANGE_6						= 0x02,
	LTR390UV_GAIN_RANGE_9						= 0x03,
	LTR390UV_GAIN_RANGE_18						= 0x04,
}LTR390UV_Gain_HandleTypeDef;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t           devAddress;
  uint8_t		 	gain;
}LTR390UV_HandleTypeDef;



HAL_StatusTypeDef LTR390UV_WriteRegister(LTR390UV_HandleTypeDef *ltr390uv, uint8_t registerAddress, uint8_t value);

uint32_t LTR390UV_ReadRegister(LTR390UV_HandleTypeDef *ltr390uv, uint8_t registerAddress);

void LTR390UV_Init(LTR390UV_HandleTypeDef *ltr390uv, I2C_HandleTypeDef *i2c);

/**
    *******************************************************************************************
	  Getting Registers' Values
    *******************************************************************************************
  */

uint8_t LTR390UV_GetMainCtrl(LTR390UV_HandleTypeDef *ltr390uv);

uint8_t LTR390UV_GetDeviceId(LTR390UV_HandleTypeDef *ltr390uv);

uint8_t LTR390UV_GetRateResolution(LTR390UV_HandleTypeDef *ltr390uv);

uint8_t LTR390UV_GetGain(LTR390UV_HandleTypeDef * ltr390uv);

/*
*******************************************************************************************
Setting MAIN CONTROL Register bits
*******************************************************************************************
*/
void LTR390UV_SetMode(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_UVS_Mode_HandleTypeDef uvsMode);

void LTR390UV_SetAlsUvsEnable(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Enable_HandleTypeDef enable);

/*
*******************************************************************************************
Getting MAIN CONTROL REGISTER bits
*******************************************************************************************
*/




/*
*******************************************************************************************
Setting ALS_UVS_MEAS_RATE Register bits
*******************************************************************************************
*/
void LTR390UV_SetResolution(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Resolution_HandleTypeDef resolution);


/*
*******************************************************************************************
Setting ALS_UVS_GAIN Register bits
*******************************************************************************************
*/
void LTR390UV_SetGain(LTR390UV_HandleTypeDef *ltr390uv, LTR390UV_Gain_HandleTypeDef gain);


/*
*******************************************************************************************
Getting ALS_DATA Registers bits
*******************************************************************************************
*/
double LTR390UV_GetAlsData(LTR390UV_HandleTypeDef *ltr390uv);


#endif
