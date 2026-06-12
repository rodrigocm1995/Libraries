/**
    *******************************************************************************************
  * @file           : OPT4048.h
  * @brief          : OPT4048 Library
    *******************************************************************************************

  * The OPT4048 is a single-chip high resolution color sensor, capable of measuring four
  * channels each with specific engineered spectral responses. Three of the four channels
  * closely match the CIE tristimulus spectra with the fourth channel having a wide band
  * spectral response.
  * With measurements from these channels, important characteristics of the lighting envi-
  * ronment can be extracted like:
  * (I)   Light intensity (lux)
  * (II)  Color in CIE XY, LUV coordinates
  * (III) Correlated Color Temperature  
  * 
  * @details
  * High precision, high speed color light-to-digital conversion over high speed I2C interface
  * Four channel sensing using precision optical filters:
  *  - XYZ tristimulus channels with close matching to CIE 1931 spectra with excellent IR
  *    rejection.
  *  - Clear wide bandwidth channel
  * High resolution color measurement in CIE XY, LUV space, correlated color temperature
  *  (CCT) along with ambient lux measurement
  * Semi-logarithmic output with 7 binary logarithmic full-scale light range and highly
  *  linear response within each range
  * Built-in automatic full-scale light range selection logic, which switches measurement
  *  range based on input light condition with excellent gain matching between ranges
  * 26 bits of effective dynamic range from 2.15 mlux to 144 klux
  * 12 configurable conversion times from 600 μs to 800 ms per channel is an excellent
  *  choice for a wide variety of high speed and high precision applications
  * External pin interrupt for hardware synchronized trigger and interrupts
  * Low operating current: 24 μA with ultra-low power standby: 2 μA
  * Operating temperature range: –40°C to +85°C
  * Wide power-supply range: 1.6 V to 3.6 V
  * 5.5-V Tolerant I/Os
  * Selectable I2C address
  * 
  *******************************************************************************************
  */

#ifndef INC_OPT4048_H_
#define INC_OPT4048_H_

#define OPT4048_ADDRESS                     0x44
#define OPT4048_TRIALS                      5
#define OPT4048_NUMBER_OF_CHANNELS          4

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Register
#define OPT4048_CH0_0		                0x00
#define OPT4048_CH0_1		                0x01
#define OPT4048_CH1_0		                0x02
#define OPT4048_CH1_1		                0x03
#define OPT4048_CH2_0		                0x04
#define OPT4048_CH2_1		                0x05
#define OPT4048_CH3_0		                0x06
#define OPT4048_CH3_1		                0x07
#define OPT4048_CONFIGURATION_REG           0x0A
#define OPT4048_CONFIGURATION_1_REG         0x0B
#define OPT4048_MASK_ENABLE_REG             0x0C
#define OPT4048_DEVICE_ID_REG               0x11 

/*******************  Bits definition for CONFIGURATION register  ******************/
#define OPT4048_FAULT_COUNT_Pos                 (0U)
#define OPT4048_FAULT_COUNT_Mask                (0x3U << OPT4048_FAULT_COUNT_Pos)
#define OPT4048_FAULT_COUNT                     OPT4048_FAULT_COUNT_Mask

#define OPT4048_INT_POL_Pos                     (2U)
#define OPT4048_INT_POL_Mask                    (0x1U << OPT4048_INT_POL_Pos)
#define OPT4048_INT_POL                         OPT4048_INT_POL_Mask

#define OPT4048_LATCH_Pos                       (3U)
#define OPT4048_LATCH_Mask                      (0x1U << OPT4048_LATCH_Pos)
#define OPT4048_LATCH                           OPT4048_LATCH_Mask

#define OPT4048_OPERATING_MODE_Pos              (4U)
#define OPT4048_OPERATING_MODE_Mask             (0x3U << OPT4048_OPERATING_MODE_Pos)
#define OPT4048_OPERATING_MODE                  OPT4048_OPERATING_MODE_Mask

#define OPT4048_CONVERSION_TIME_Pos             (6U)
#define OPT4048_CONVERSION_TIME_Mask            (0xFU << OPT4048_CONVERSION_TIME_Pos)
#define OPT4048_CONVERSION_TIME                 OPT4048_CONVERSION_TIME_Mask

#define OPT4048_RANGE_Pos                       (10U)
#define OPT4048_RANGE_Mask                      (0xFU << OPT4048_RANGE_Pos)
#define OPT4048_RANGE                           OPT4048_RANGE_Mask

#define OPT4048_QWAKE_Pos                       (15U)
#define OPT4048_QWAKE_Mask                      (0x1U << OPT4048_QWAKE_Pos)
#define OPT4048_QWAKE                           OPT4048_QWAKE_Mask

/*******************  Bits definition for CONFIGURATION_1 register  ******************/
#define OPT4048_I2C_BURST_Pos                   (0U)
#define OPT4048_I2C_BURST_Mask                  (0x1U << OPT4048_I2C_BURST_Pos)
#define OPT4048_I2C_BURST                       OPT4048_I2C_BURST_Mask

#define OPT4048_INT_CFG_Pos                     (2U)
#define OPT4048_INT_CFG_Mask                    (0x3U << OPT4048_INT_CFG_Pos)
#define OPT4048_INT_CFG                         OPT4048_INT_CFG_Mask

#define OPT4048_INT_DIR_Pos                     (4U)
#define OPT4048_INT_DIR_Mask                    (0x1U << OPT4048_INT_DIR_Pos)
#define OPT4048_INT_DIR                         OPT4048_INT_DIR_Mask

#define OPT4048_THRESHOLD_CH_SEL_Pos            (5U)
#define OPT4048_THRESHOLD_CH_SEL_Mask           (0x3U << OPT4048_THRESHOLD_CH_SEL_Pos)
#define OPT4048_THRESHOLD_CH_SEL                OPT4048_THRESHOLD_CH_SEL_Mask

#define OPT4048_128_Pos                         (7U)
#define OPT4048_128_Mask                        (0x1FFU << OPT4048_128_Pos)
#define OPT4048_128                             OPT4048_128_Mask

/*******************  Bits definition for MASK ENABLE register  ******************/
#define OPT4048_FLAG_L_Pos                      (0U)
#define OPT4048_FLAG_L_Mask                     (0x1U << OPT4048_FLAG_L_Pos)
#define OPT4048_FLAG_L                          OPT4048_FLAG_L_Mask

#define OPT4048_FLAG_H_Pos                      (1U)
#define OPT4048_FLAG_H_Mask                     (0x1U << OPT4048_FLAG_H_Pos)
#define OPT4048_FLAG_H                          OPT4048_FLAG_H_Mask

#define OPT4048_CONVERSION_READY_FLAG_Pos       (2U)
#define OPT4048_CONVERSION_READY_FLAG_Mask      (0x1U << OPT4048_CONVERSION_READY_FLAG_Pos)
#define OPT4048_CONVERSION_READY_FLAG           OPT4048_CONVERSION_READY_FLAG_Mask

#define OPT4048_OVERLOAD_FLAG_Pos               (3U)
#define OPT4048_OVERLOAD_FLAG_Mask              (0x1U << OPT4048_OVERLOAD_FLAG_Pos)
#define OPT4048_OVERLOAD_FLAG                   OPT4048_OVERLOAD_FLAG_Mask

/*******************  Bits definition for CH_X registers  ******************/
#define OPT4048_EXPONENT_CH_X_Pos               (12U)
#define OPT4048_EXPONENT_CH_X_Mask              (0xFU << OPT4048_EXPONENT_CH_X_Pos)
#define OPT4048_EXPONENT_CH_X                   OPT4048_EXPONENT_CH_X_Mask

#define OPT4048_RESULT_MSB_CH_X_Pos             (0U)
#define OPT4048_RESULT_MSB_CH_X_Mask            (0xFFFU << OPT4048_RESULT_MSB_CH_X_Pos)
#define OPT4048_RESULT_MSB_CH_X                 OPT4048_RESULT_MSB_CH_X_Mask

#define OPT4048_RESULT_LSB_CH_X_Pos             (8U)
#define OPT4048_RESULT_LSB_CH_X_Mask            (0xFFU << OPT4048_RESULT_LSB_CH_X_Pos)
#define OPT4048_RESULT_LSB_CH_X                 OPT4048_RESULT_LSB_CH_X_Mask

#define OPT4048_COUNTER_CH_X_Pos                (4U)
#define OPT4048_COUNTER_CH_X_Mask               (0xFU << OPT4048_COUNTER_CH_X_Pos)
#define OPT4048_COUNTER_CH_X                    OPT4048_COUNTER_CH_X_Mask

#define OPT4048_CRC_CH_X_Pos                    (0U)
#define OPT4048_CRC_CH_X_Mask                   (0xFU << OPT4048_CRC_CH_X_Pos)
#define OPT4048_CRC_CH_X                        OPT4048_CRC_CH_X_Mask

typedef enum
{
    OPT4048_ONE_FAULT_COUNT             = 0x0U,
    OPT4048_TWO_FAULT_COUNT             = 0x1U,
    OPT4048_FOUR_FAULT_COUNT            = 0x1U,
    OPT4048_EIGHT_FAULT_COUNT           = 0x2U,
} OPT4048_FaultCount_TypeDef;

typedef enum
{
	OPT4048_ALERT_ACTIVE_LOW			= 0x0U,
	OPT4048_ALERT_ACTIVE_HIGH			= 0x1U,
} OPT4048_AlertPinPol_TypeDef;

typedef enum
{
	OPT4048_SHUTDOWN_MODE				= 0x0U,
	OPT4048_FORCED_AUTORANGE_ONE_SHOT	= 0x1U,
    OPT4048_ONE_SHOT_MODE		        = 0x2U,
	OPT4048_CONTINUOUS_MODE		        = 0x3U,
}OPT4048_Mode_TypeDef;

/**
  * @brief  OPT4048 single-channel conversion time settings.
  * @note   Since the OPT4048 is a 4-channel device, the total time to complete 
  *         a measurement across all channels is 4 times the value selected here.
  */
typedef enum
{
	OPT4048_600_US						= 0x0U,
	OPT4048_1_MS						= 0x1U,
	OPT4048_1_8_MS						= 0x2U,
	OPT4048_3_4_MS						= 0x3U,
	OPT4048_6_5_MS						= 0x4U,
	OPT4048_12_7_MS						= 0x5U,
	OPT4048_25_MS						= 0x6U,
	OPT4048_50_MS						= 0x7U,
	OPT4048_100_MS					    = 0x8U,
	OPT4048_200_MS					    = 0x9U,
	OPT4048_400_MS					    = 0xAU,
	OPT4048_800_MS					    = 0xBU,
} OPT4048_ConvTime_TypeDef;

/**
  * @brief  OPT4048 full-scale light range selection settings.
  * @note   TI highly recommends using the automatic range mode (OPT4048_AUTOMATIC_RANGE = 0xCU).
  *         This mode dynamically and independently optimizes the measurement range for each 
  *         of the 4 channels. Manual ranges disable this automatic selection logic.
  */
typedef enum
{
	OPT4048_2254_LUX					= 0x0U,
	OPT4048_4509_LUX					= 0x1U,
	OPT4048_9018_LUX					= 0x2U,
	OPT4048_18036_LUX					= 0x3U,
	OPT4048_36071_LUX					= 0x4U,
	OPT4048_72142_LUX					= 0x5U,
	OPT4048_144284_LUX					= 0x6U,
	OPT4048_AUTOMATIC_RANGE				= 0xCU,
}OPT4048_FullScaleRange_TypeDef;

typedef enum
{
    OPT4048_QUICK_WAKE_DISABLED         = 0x0U,
    OPT4048_QUICK_WAKE_ENABLED          = 0x1U,
} OPT4048_QuickWake_TypeDef;

typedef enum
{
    OPT4048_I2C_BURST_DISABLED          = 0x0U,
    OPT4048_I2C_BURST_ENABLED           = 0x1U,
} OPT4048_I2CBurst_TypeDef;

typedef enum
{
    OPT4048_SMBUS_ALERT                 = 0x0U,
    OPT4048_DTARDY_NEXT_CHANNEL         = 0x1U,
    OPT4048_DTARDY_ALL_CHANNELs         = 0x2U,
} OPT4048_IntCfg_TypeDef;

typedef enum
{
    OPT4048_INT_AS_INPUT                = 0x0U,
    OPT4048_INT_AS_OUTPUT               = 0x1U,
} OPT4048_AlertPinDirection_TypeDef;

typedef enum
{
    OPT4048_CHANNEL_0_SEL               = 0x1U,
    OPT4048_CHANNEL_1_SEL               = 0x1U,
    OPT4048_CHANNEL_2_SEL               = 0x2U,
    OPT4048_CHANNEL_3_SEL               = 0x3U,
} OPT4048_ChThreshold_TypeDef;

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            _devAddress;
  double 			 cieX;
  double 			 cieY;
  double 		     cieZ;
  double 		     lux;
  uint16_t           counterch0;
  uint16_t           counterch1;
  uint16_t           counterch2;
  uint16_t           counterch3;
} OPT4048_HandleTypeDef;  

typedef enum
{
	OPT4048_POWERDOWN					= 0x0000,
	OPT4048_FORCER_AUTORANGE_ONESHOT    = 0x0010,
	OPT4048_ONESHOT					    = 0x0020,
	OPT4048_CONTINUOUS					= 0x0030
}opt4048ControlMode_t;


HAL_StatusTypeDef OPT4048_WriteRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress, uint16_t value);
uint16_t OPT4048_ReadRegister(OPT4048_HandleTypeDef *opt4048, uint8_t registerAddress);
uint16_t OPT4048_GetDeviceID(OPT4048_HandleTypeDef *opt4048);

void OPT4048_SetFaultCount(OPT4048_HandleTypeDef *opt4048, OPT4048_FaultCount_TypeDef faultCount);
void OPT4048_SetAlertPinPolarity(OPT4048_HandleTypeDef *opt4048, OPT4048_AlertPinPol_TypeDef polarity);
void OPT4048_SetMode(OPT4048_HandleTypeDef *opt4048, OPT4048_Mode_TypeDef mode);
void OPT4048_SetConvTime(OPT4048_HandleTypeDef *opt4048, OPT4048_ConvTime_TypeDef convTime);
void OPT4048_SetRange(OPT4048_HandleTypeDef *opt4048, OPT4048_FullScaleRange_TypeDef range);
void OPT4048_SetI2CType(OPT4048_HandleTypeDef *opt4048, OPT4048_I2CBurst_TypeDef i2cType);

_Bool OPT4048_IsConversionReady(OPT4048_HandleTypeDef *opt4048);
HAL_StatusTypeDef OPT4048_ReadADCRawValues(OPT4048_HandleTypeDef *opt4048, uint16_t *regValues);
HAL_StatusTypeDef OPT4048_GetXYZAndLux(OPT4048_HandleTypeDef *opt4048);

void OPT4048_Init(OPT4048_HandleTypeDef *opt4048, I2C_HandleTypeDef *i2c, uint8_t devAddress);

#endif

