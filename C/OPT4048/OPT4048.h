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

#define OPT4048_ADDRESS                   0x44
#define OPT4048_TRIALS                    5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))



// Register
#define OPT4048_CH0_0		              0x00
#define OPT4048_CH0_1		              0x01
#define OPT4048_CH1_0		              0x02
#define OPT4048_CH1_1		              0x03
#define OPT4048_CH2_0		              0x04
#define OPT4048_CH2_1		              0x05
#define OPT4048_CH3_0		              0x06
#define OPT4048_CH3_1		              0x07
#define OPT4048_CONFIGURATION_REGISTER    0x0A
#define OPT4048_DEVICE_ID_REGISTER        0x11 // This register controls the major operational modes of the device

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

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            devAddress;
  double 			 cieX;
  double 			 cieY;
  double 		     cieZ;
  double 		     lux;
  uint16_t           counterch0;
  uint16_t           counterch1;
  uint16_t           counterch2;
  uint16_t           counterch3;
} Opt4048_t;  

typedef enum
{
	OPT4048_2_2_KLUX					= 0x0000,
	OPT4048_4_5_KLUX					= 0x0400,
	OPT4048_9_0_KLUX				    = 0x0800,
	OPT4048_18_0_KLUX					= 0x0C00,
	OPT4048_36_0_KLUX     				= 0x1000,
	OPT4048_72_0_KLUX					= 0x1400,
	OPT4048_144_0_KLUX					= 0x1800,
	OPT4048_AUTO_RANGE					= 0x3000
} opt4048Range_t;

typedef enum
{
	OPT4048_600_US						= 0x0000,
	OPT4048_1_0_MS						= 0x0040,
	OPT4048_1_8_MS						= 0x0080,
	OPT4048_3_4_MS						= 0x00C0,
	OPT4048_6_5_MS						= 0x0100,
	OPT4048_12_7_MS						= 0x0140,
	OPT4048_25_0_MS						= 0x0180,
	OPT4048_50_0_MS						= 0x01C0,
	OPT4048_100_0_MS					= 0x0200,
	OPT4048_200_0_MS					= 0x0240,
	OPT4048_400_0_MS					= 0x0280,
	OPT4048_800_0_MS					= 0x02C0
} opt4048ConvTime_t;

typedef enum
{
	OPT4048_POWERDOWN					= 0x0000,
	OPT4048_FORCER_AUTORANGE_ONESHOT    = 0x0010,
	OPT4048_ONESHOT					    = 0x0020,
	OPT4048_CONTINUOUS					= 0x0030
}opt4048ControlMode_t;


void opt4048WriteRegister(Opt4048_t *opt4048, uint8_t registerAddress, uint16_t value);

uint16_t opt4048ReadRegister(Opt4048_t *opt4048, uint8_t registerAddress);

int32_t opt4048ReadRegister32(Opt4048_t *opt4048, uint8_t registerAddress);

void opt4048Init(Opt4048_t *opt4048, I2C_HandleTypeDef *i2c);

uint16_t opt4048GetDeviceId(Opt4048_t *opt4048);

uint32_t opt4048GetAdcCode0(Opt4048_t *opt4048);

uint32_t opt4048GetAdcCode1(Opt4048_t *opt4048);

uint32_t opt4048GetAdcCode2(Opt4048_t *opt4048);

uint32_t opt4048GetAdcCode3(Opt4048_t *opt4048);

void opt4048GetAllAdcCodes(Opt4048_t *opt4048);

void opt4048GetCie(Opt4048_t *opt4048);


#endif

