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
#define INC_OPT3001_H

#define OPT3001_DEFAULT_ADDRESS           0x44
#define OPT3001_TRIALS                    5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


// Register
#define OPT3001_RESULT_REGISTER           0x00  // This register conmtains the result of the most recent light to digital conversion.
#define OPT3001_CONFIGURATION_REGISTER    0x01  // This register controls the major operational modes of the device 
#define OPT3001_LOW_LIMIT_REGISTER        0x02
#define OPT3001_HIGH_LIMIT_REGISTER       0x03
#define OPT3001_MANUFACTURER_ID_REGISTER  0x7E
#define OPT3001_DEVICE_ID_REGISTER        0x7F

#define FULL_SCALE_RANGE_40_95            40.95
#define FULL_SCALE_RANGE_81_90            81.90
#define FULL_SCALE_RANGE_163_80           163.80
#define FULL_SCALE_RANGE_327_60           327.60
#define FULL_SCALE_RANGE_655_20           655.20
#define FULL_SCALE_RANGE_1310_40          1310.40
#define FULL_SCALE_RANGE_2620_80          2620.80
#define FULL_SCALE_RANGE_5241_60          5241.60
#define FULL_SCALE_RANGE_10483_20         10483.20
#define FULL_SCALE_RANGE_20966_40         20966.40
#define FULL_SCALE_RANGE_41932_80         41932.80
#define FULL_SCALE_RANGE_83865_60         83865.60

#define FLAG_LOW_FIELD_BIT				  5
#define FLAG_HIGH_FIELD_BIT				  6

typedef enum 
{
  OPT3001_FSR_40DOT95                   = 0x0000, // LSB SIZE (lux per LSB) = 0.01 
  OPT3001_FSR_81DOT90                   = 0x1000, // LSB SIZE (lux per LSB) = 0.02
  OPT3001_FSR_163DOT80                  = 0x2000, // LSB SIZE (lux per LSB) = 0.04
  OPT3001_FSR_327DOT60                  = 0x3000, // LSB SIZE (lux per LSB) = 0.08
  OPT3001_FSR_655DOT20                  = 0x4000, // LSB SIZE (lux per LSB) = 0.16
  OPT3001_FSR_1310DOT40                 = 0x5000, // LSB SIZE (lux per LSB) = 0.32
  OPT3001_FSR_2620DOT80                 = 0x6000, // LSB SIZE (lux per LSB) = 0.64
  OPT3001_FSR_5241DOT60                 = 0x7000, // LSB SIZE (lux per LSB) = 1.28
  OPT3001_FSR_10483DOT20                = 0x8000, // LSB SIZE (lux per LSB) = 2.56
  OPT3001_FSR_20966DOT40                = 0x9000, // LSB SIZE (lux per LSB) = 5.12
  OPT3001_FSR_41932DOT80                = 0xA000, // LSB SIZE (lux per LSB) = 10.24
  OPT3001_FSR_83865DOT60                = 0xB000, // LSB SIZE (lux per LSB) = 20.48
  OPT_3001_AUTOMATIC_FULL_SCALE_MODE    = 0xC000  // Automatic scale
} Exponent_t;

typedef enum 
{
  OPT3001_CT_100_MS                     = 0x0000,
  OPT3001_CT_800_MS                     = 0x0800
} ConversionTime_t;

typedef enum
{
  OPT3001_SHUTDOWN_MODE                 = 0x0000,
  OPT3001_SINGLESHOT_MODE               = 0x0200,
  OPT3001_CONTINUOUS_CONVERSION         = 0x0400
} ModeOfConversion_t;

typedef enum 
{
  TRANSPARENT_HYSTERESIS_STYLE          = 0x0000, // Is tipically used when a single digital signal is desired that indicates whether the input light is higher that or lower than  a light level of interest.
  LATCHED_WINDOWS_STYLE                 = 0x0010
} LatchField_t;

typedef enum
{
  ACTIVE_LOW_POLARITY                   = 0x0000,
  ACTIVE_HIGH_POLARITY                  = 0x0008
}PolarityField_t;

// Exponent field can be disabled (set to zero) by enabling the exponent mask (configuration register ME = 1)
typedef enum
{
  EXPONENT_FIELD_ENABLED                = 0x0004,
  EXPONENT_FIELD_DISABLED               = 0x0000
} MaskExponent_t;  

typedef enum
{
  ONE_FAULT_COUNT                       = 0x0000,
  TWO_FAULT_COUNTS                      = 0x0001,
  FOUR_FAULT_COUNTS                     = 0x0002,
  EIGHT_FAULT_COUNTS                    = 0x0003
} FaultCountField_t;

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            devAddress;
} Opt3001_t;  

void opt3001WriteRegister(Opt3001_t *opt3001, uint8_t registerAddress, uint16_t value);

uint16_t opt3001ReadRegister(Opt3001_t *opt3001, uint8_t registerAddress);

uint8_t opt3001DefaulInit(Opt3001_t *opt3001, I2C_HandleTypeDef *i2c, uint8_t devAddress);

uint8_t opt3001Init(Opt3001_t *opt3001, I2C_HandleTypeDef *i2c, uint8_t devAddress, Exponent_t exponentRange, ConversionTime_t conversionTime, ModeOfConversion_t mode, LatchField_t latchField, PolarityField_t polarity, MaskExponent_t mask, FaultCountField_t faultCount);

void opt3001SetLowLimit(Opt3001_t *opt3001, double lowLimit);

double opt3001ReadLowLimit(Opt3001_t *opt3001);

void opt3001SetHighLimit(Opt3001_t *opt3001, double highLimit);

double opt3001ReadHighLimit(Opt3001_t *opt3001);

uint16_t opt3001ReadManufacturerId(Opt3001_t *opt3001);

uint16_t opt3001ReadConfigurationRegister(Opt3001_t *opt3001);

double opt3001ReadLuxValue(Opt3001_t *opt3001);

_Bool opt3001LowLimitFlag(Opt3001_t *opt3001);

_Bool opt3001HighLimitFlag(Opt3001_t *opt3001);

#endif
