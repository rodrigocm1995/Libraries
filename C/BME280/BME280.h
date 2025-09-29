/**
    *******************************************************************************************
  * @file           : BME280.h
  * @brief          : BME280 Library
    *******************************************************************************************

  * The BME280 is a combined digital humidity, pressure and temperature sensor based on proven 
  * sensing principles. The sensor module is housed in a extremely compact metal-lid LGA package.
  * The BME280 is register and performance compatible to the Bosch Sensortec BMP280 digital 
  * pressure sensor.
  * 
  * The BME280 achieves high performance in all applications requiring humidity and pressure
  * measurement. These emerging applications of home automation control, in-door navigation, 
  * fitness as well as GPS refinement require a high accuracy and a low TCO at the same time.
  * 
  * The humidity sensor provides an extremely fast response time for fast context awareness 
  * applications and high overall accuracy over a wide temperature range.
  * 
  * The pressure sensor is an absolute barometric pressure sensor with extremely high accurary
  * and resolution and drastically lower noise than the Bosch Sensortec BMP180.
  * 
  * The integrated temperature sensor has been optimized for lowest noise and highest resolution.
  * Its output is used for temperature compensation of the pressure and humidity sensor and can
  * also be used for estimation of the ambient temperature.
  * 
  * The sensor provides both SPI and I2C interfaces and can be supplied using 1.71 to 3.6V for
  * the sensor supply VDD and 1.2 to 3.6V for the interface supply VDDIO. Measurements can be 
  * triggered by the host or performed in regulars intervals. When the sensor is disabled,
  * current consumption drops to 0.1µA.
  * 
  * @details
  * Package: 2.5mm × 2.5mm × 0.93mm metal lid LGA
  * Digital Interface: I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 Mhz)
  * Suppy Voltage: 
  * - VDD main supply voltage range: 1.71V to 3.6V
  * - VDDIO interface voltage range: 1.2V TO 3.6V
  * Current Consumption:
  * - 1.8 µA @ 1 Hz humidity and temperature
  * - 2.8 µA @ 1 Hz pressure and temperature
  * - 3.6 uA @ 1 Hz humidity, pressure and temperature 0.1 µA in sleep mode
  * Operating Range: -40 to +85 °C, 0 to 100 % rel, 300 to 1100 hPa
  * Humidity sensor and pressure sensor can be independently enabled/disabled 
  * Register and performance compatible to Bosch Sensortec BMP280 digital pressure sensor
  *
  * 
  *******************************************************************************************
  */

#ifndef _INC_BME280_H_	// preprocessor directive for checking whether a definition exists
#define _INC_BME280_H_
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


// BME280 memory map registers
#define BME280_TRIALS				  	5
#define BME280_ADDRESS             	  	0x76
#define BME280_CONFIG_REG	          	0xF5
#define BME280_CTRL_MEAS_REG	      	0xF4
#define BME280_STATUS_REG	          	0xF3
#define BME280_CTRL_HUM		          	0xF2
#define BME280_DEVICE_ID		      	0xD0
#define BME280_RESET					0xE0

#define BME280_OVERSAMPLE_TEMP_MASK   	0xE0
#define BME280_OVERSAMPLE_PRES_MASK   	0x1C
#define BME2280_TEMP_ADC_MASK			0x000000FFFFFF0000
#define BME2280_PRES_ADC_MASK			0xFFFFFF0000000000
#define BME2280_HUM_ADC_MASK			0x000000000000FFFF
#define BME280_RESET_DEVICE				0xB6

#define BME280_CAL_COEFF_T1				0x88
#define BME280_CAL_COEFF_T2				0x8A
#define BME280_CAL_COEFF_T3				0x8C
#define BME280_CAL_COEFF_P1				0x8E
#define BME280_CAL_COEFF_P2				0x90
#define BME280_CAL_COEFF_P3				0x92
#define BME280_CAL_COEFF_P4				0x94
#define BME280_CAL_COEFF_P5				0x96
#define BME280_CAL_COEFF_P6				0x98
#define BME280_CAL_COEFF_P7				0x9A
#define BME280_CAL_COEFF_P8				0x9C
#define BME280_CAL_COEFF_P9				0x9E
#define BME280_CAL_COEFF_H1				0xA1
#define BME280_CAL_COEFF_H2				0xE1
#define BME280_CAL_COEFF_H3				0xE3
#define BME280_CAL_COEFF_H4				0xE4
#define BME280_CAL_COEFF_H5				0xE5

typedef enum
{
	BME280_STBY_0_5_MS	              =	0x00,
	BME280_STBY_62_5_MS	              =	0x20,
	BME280_STBY_125_0_MS	          =	0x40,
	BME280_STBY_250_0_MS	          =	0x60,
	BME280_STBY_500_0_MS	          =	0x80,
	BME280_STBY_1000_0_MS	          =	0xA0,
	BME280_STBY_10_0_MS	              =	0xC0,
	BME280_STBY_20_0_MS	              =	0xE0	
} Bme280Stby_t;

typedef enum
{
	BME280_FILTER_COEFF_OFF	          =	0x00,
	BME280_FILTER_COEFF_2	          = 0x02,
	BME280_FILTER_COEFF_4	          =	0X04,
	BME280_FILTER_COEFF_8	          =	0x08,
	BME280_FILTER_COEFF_16	          =	0x0C
} Bme280FilterCoeff_t;	

typedef enum
{
	BME280_SAMPLES_OFF	              = 0,
	BME280_SAMPLES_X1	              =	1,
	BME280_SAMPLES_X2	              =	2,
	BME280_SAMPLES_X4	              =	3,
	BME280_SAMPLES_X8	              =	4,
	BME280_SAMPLES_X16	              =	5
} Bme280Samples_t;

typedef enum
{
	BME280_SLEEP_MODE	              =	0,
	BME280_FORCED_MODE	              =	1,
	BME280_NORMAL_MODE	              =	2
} Bme280Mode_t;


typedef struct
{	
	I2C_HandleTypeDef				*hi2c;
	uint8_t							devAddress;
	uint16_t						digT1;
	int16_t			 				digT2;
	int16_t			 				digT3;
	uint16_t			 			digP1;
	int16_t			 				digP2;
	int16_t			 				digP3;
	int16_t			 				digP4;
	int16_t			 				digP5;
	int16_t			 				digP6;
	int16_t			 				digP7;
	int16_t			 				digP8;
	int16_t			 				digP9;
	int8_t			 				digH1;
	int16_t			 				digH2;
	int8_t			 				digH3;
	int16_t			 				digH4;
	int16_t			 				digH5;
	int16_t			 				digH6;

} Bme280_t;


void     	bme280WriteRegister8(Bme280_t *bme280, uint8_t registerAddress, uint8_t value);

void 		bme280WriteRegister16(Bme280_t *bme280, uint8_t registerAddress, uint16_t value);

uint8_t		bme280ReadRegister8(Bme280_t *bme280, uint8_t registerAddress);

int64_t 	bme280ReadRegister(Bme280_t *bme280, uint8_t registerAddress);

void 	  	bme280SetTempSampling(Bme280_t *bme280, Bme280Samples_t tempSampling);

uint8_t		bme280GetTempSampling(Bme280_t *bme280);

void  		bme280SetPressSampling(Bme280_t *bme280, Bme280Samples_t pressSampling);

uint8_t		bme280GetPressSampling(Bme280_t *bme280);

void 	  	bme280SetHumSampling(Bme280_t *bme280, Bme280Samples_t humSampling);

uint8_t		bme280GetHumSampling(Bme280_t *bme280);

void  		bme280SetMode(Bme280_t *bme280, Bme280Mode_t mode);

uint8_t     bme280GetMode(Bme280_t *bme280);

uint8_t 	bme280GetCtrlMeas(Bme280_t *bme280);

uint8_t		bme280GetId(Bme280_t *bme280);

void 		bme280Init(Bme280_t *bme280, I2C_HandleTypeDef *i2c);

void		bme280SetStByTime(Bme280_t *bme280, Bme280Stby_t sbTime);

void		bme280SetFilterCoeff(Bme280_t *bme280, Bme280FilterCoeff_t fCoeff);

uint8_t		bme280GetConfig(Bme280_t *bme280);

uint8_t		bme280GetCtrlMeas(Bme280_t *bme280);

int64_t		bme280GetOutput(Bme280_t *bme280);

void 		bme280GetCalCoeff(Bme280_t *bme280);

double		bme280GetTemperature(Bme280_t *bme280);

double 		bme280GetPressure(Bme280_t *bme280);

double		bme280GetAltitude(Bme280_t *bme280);

_Bool 		bme280CheckDataReady(Bme280_t *bme280);

void 		bme280ResetDevice(Bme280_t *bme280);

#endif

