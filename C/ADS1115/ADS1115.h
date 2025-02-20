/**
    *******************************************************************************************
  * @file           : ADS1115.h
  * @brief          : ADS1115 Library
    *******************************************************************************************

  * The ADS111x are precision, low-power 16-bit, I2C compatible, analog-to-digital converters
  * (ADCs) offered in a leadless X2QFN-10 or SOT-10 package. The ADS111x devices incorporate
  * a low drift voltage reference and an oscillator. The ADS1114 and ADS1115 also incorporate
  * a programmable fain amplifier (PGA) and a digital comparator.
  * 
  * The ADS111x devices perform conversions at data rates of up to 860 samples per second (SPS)
  * The PGA offers inpur ranges from ±256mV to ±6.144V, allowing precise large ans small-signal
  * measurements. The ADS1115 features an input multiplexer (MUX) that allows two differential
  * or four single-ended input measurements. Use the digital comparator in the ADS1114 and 
  * ADS1115 for undervoltage and overvoltage protection.
  *   
  * @details
  * Ultra small packages:
  * - X2QFN: 2mm × 1.5mm × 0.4mm
  * - SOT: 2.9mm × 2.8mm × 0.6mm
  * Wide supply range: 2.0V to 5.5V
  * Low current consumption: 150 µA (continuous conversion mode)
  * Programmable data rate: 8 SPS to 860 SPS
  * Single-cycle settling
  * Internal low-drift voltage reference
  * Internal oscillator
  * I2C interface: four pin-selectable addresses
  * Operating temperature range:
  * -40 °C to +125°C
  * Family of devices:
  * - ADS1113: one single-ended (SE) or differential (DE) input
  * - ADS1114: one single-ended (SE) or differential (DE) input with comparator and PGA
  * - ADS1115: four single-ended or two differential inputs with comparator and PGA
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#define ADS1115_DEFAULT_ADDRESS                  0x48
#define ADS1115_TRIALS                          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define ADS1115_CONVERSION_REGISTER              0x00
#define ADS1115_CONFIGURATION_REGISTER           0x01
#define ADS1115_THIGH_LIMIT_REGISTER             0x02
#define ADS1115_TLOW_LIMIT_REGISTER              0x03

typedef enum
{
  AINP_IS_AIN0_AINN_IS_AIN1                    = 0x0000,
  AINP_IS_AIN0_AINN_IS_AIN3                    = 0x1000,
  AINP_IS_AIN1_AINN_IS_AIN3                    = 0x2000,
  AINP_IS_AIN2_AINN_IS_AIN3                    = 0x3000,
  AINP_IS_AIN0_AINN_IS_GND                     = 0x4000,
  AINP_IS_AIN1_AINN_IS_GND                     = 0x5000,
  AINP_IS_AIN2_AINN_IS_GND                     = 0x6000,
  AINP_IS_AIN3_AINN_IS_GND                     = 0x7000,
}ADS115Mux;

typedef enum
{
  FSR_6_DOT_144_V                             = 0x0000,
  FSR_4_DOT_096_V                             = 0x0200,
  FSR_2_DOT_048_V                             = 0x0400,
  FSR_1_DOT_024_V                             = 0x0600,
  FSR_0_DOT_512_V                             = 0x0800,
  FSR_0_DOT_256_V                             = 0x0A00,
}ADS115Pga;

typedef enum
{
  ADS1115_CONTINUOUS_MODE                     = 0x0000,
  ADS1115_SINGLE_SHOT                         = 0x0100,  
} ADS115Mode;

typedef enum
{
  ADS1115_8_SPS                               = 0x0000,
  ADS1115_16_SPS                              = 0x0020,
  ADS1115_32_SPS                              = 0x0040,
  ADS1115_64_SPS                              = 0x0060,
  ADS1115_128_SPS                             = 0x0080,
  ADS1115_250_SPS                             = 0x00A0,
  ADS1115_475_SPS                             = 0x00C0,
  ADS1115_860_SPS                             = 0x00E0,
} ADS1115DataRate;

typedef enum
{
  ADS1115_TRADITIONAL_COMP                    = 0x0000,
  ADS1115_WINDOW_COMP                         = 0x0010,
} ADS1115CompMode;

typedef enum
{
  ADS1115_ACTIVE_LOW                          = 0x0000,
  ADS1115_ACTIVE_HIGH                         = 0x0008,
} ADS1115CompPol;

typedef enum
{
  ADS1115_NONLATCHING_COMP                    = 0x0000,
  ADS1115_LATCHING_COMP                       = 0x0004,
} ADS1115CompLatch;

typedef enum
{
  ADS1115_ONE_CONVERSION                      = 0x0000,
  ADS1115_TWO_CONVERSIONS                     = 0x0001,
  ADS1115_FOUR_CONVERSIONS                    = 0x0002,
  ADS1115_DISABLE_COMPARATOR                  = 0x0003,
} ADS1115CompQueue;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t devAddress;
  double lsbSize;
}ADS1115_t;


void Ads1115WriteRegister(ADS1115_t *ads1115, uint8_t registerAddress, uint16_t value);

uint16_t Ads1115ReadRegister(ADS1115_t *ads1115, uint8_t registerAddress);

uint8_t Ads1115Init(ADS1115_t *ads1115, I2C_HandleTypeDef *i2c, uint8_t devAddress);

uint8_t Ads1115CustomInit(ADS1115_t *ads1115, I2C_HandleTypeDef *i2c, uint8_t devAddress, ADS115Mux muxType, ADS115Pga pga, ADS115Mode mode, ADS1115DataRate dataRate, ADS1115CompMode compMode, ADS1115CompPol pol, ADS1115CompLatch latch, ADS1115CompQueue queue);

uint16_t Ads1115GetConfigRegister(ADS1115_t *ads1115);

uint16_t Ads1115GetConversion(ADS1115_t *ads1115);

double Ads1115GetVoltage(ADS1115_t *ads1115);

_Bool Ads1115IsDeviceReady(ADS1115_t *ads1115);


#endif
