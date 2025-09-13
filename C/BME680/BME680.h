/**
    *******************************************************************************************

  * @file           : BME680.h
  * @brief          : BME680 Library
    *******************************************************************************************

  * The BME680 is a 4-in-1 sensor with gas, humidity, pressure and temperature measurement based
  * on proven sensing principles. The sensor module is housed in an extremely compact metal-lid
  * LGA package with a footprint of only 3.0 x 3.0 mm2.
  * 
  * @details
  * package: 3.0 mm x 3.0 mm x 0.93 mm metal lid LGA
  * Digital Interface: I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)
  * Supply Voltage : VDD main supply voltage range: 1.71 V to 3.6 V 
  *                  VDDIO interface voltage range: 1.2 V to 3.6 V
  * Current consumption: 2.1 µA at 1 Hz humidity and temperatrure.
  *                      3.1 µA at 1 Hz pressure and temperature
  *                      3.7 µA at 1 Hz humidity, pressure and temperature 
  *                      0.09 - 12 mA for p/h/T/gas depending on operation mode  
  *                      0.15 µA in sleep mode
  * Operating Range: -40 to +85 °C, 0 - 100 % RH, 300 - 1100 hPa
  * Individual humidity, pressure and gas sensor can be independently enabled/disablesRodrigo Castillejos Malpica
  * @example
  * INA236_CALIBRATION_REGISTER is calculated based on the next equation
  * SHUNT_CAL = 0.00512/(Current_LSB x Rshunt).......................(1)
  * 
  *******************************************************************************************
  */

#ifndef INC_BME680_H_
#define INC_BME680_H_

#define BME680_DEFAULT_ADDRESS          0x77
#define BME680_TRIALS                   5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))


#define MEAS_STATUS_0_REGISTER 0x1D
/*
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| Register Name | Address | Content<bit position> |                                Description                                |
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| press_msb     | 0x1F    | press_msb<7:0>        | Contains the MSB part [19:12] of the raw pressure measurement output data |
| press_lsb     | 0x20    | press_lsb<7:0>        | Contains the LSB part [11:4] of the raw pressure measurement output data  |
| press_xlsb    | 0x21    | press_xlsb<7:4>       | Contains the XLSB part [3:0] of the raw pressure measurement output data. |
|               |         |                       | Contents depend on pressure resolution controlled by oversampling setting.|
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
*/
#define PRESSURE_REGISTER      0x1F

/*
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| Register Name | Address | Content<bit position> |                                Description                                |
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| temp_msb      | 0x22    | temp_msb<7:0>         | Contains the MSB part [19:12] of the raw temperature measurement output   |
|               |         |                       | data.                                                                     |
| temp_lsb      | 0x23    | temp_lsb<7:0>         | Contains the LSB part [11:4] of the raw temperature measurement output    |
|               |         |                       | data.                                                                     |
| temp_xlsb     | 0x24    | press_xlsb<7:4>       | Contains the XLSB part [3:0] of the raw temperature measurement output    |
|               |         |                       | data. Contents depend on temperature resolution controlled by oversampling| 
|               |         |                       | setting.                                                                  |
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
*/
#define TEMPERATURE_REGISTER   0x22

/*
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| Register Name | Address | Content<bit position> |                               Description                                 |
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
| hum_msb       | 0x25    | hum_msb<7:0>          | Contains the MSB part [15:8] of the raw humidity measurement output data. |
| hum_lsb       | 0x26    | hum_lsb<7:0>          | Contains the LSB part [7:0] of the raw humidity measurement output data.  |
+---------------+---------+-----------------------+---------------------------------------------------------------------------+
*/
#define HUMIDITY_REGISTER      0x25
#define GAS_R_REGISTER         0x2A
#define CTRL_HUM_REGISTER      0x72
#define CTRL_MEAS_REGISTER     0x74
#define CONFIG_REGISTER        0x75
#define ID_REGISTER            0xD0

// Register addresses for TEMP_COMP calculation
#define PAR_T1				   0xE9
#define PAR_T2				   0x8A
#define PAR_T3				   0x8C
#define TEMP_ADC			   0x22

// Register addresses for PRESS_COMP calculation
#define PAR_P1				   0x8E
#define PAR_P2                 0x90
#define PAR_P3                 0x92
#define PAR_P4                 0x94
#define PAR_P5                 0x96
#define PAR_P6                 0x99
#define PAR_P7                 0x98
#define PAR_P8                 0x9C
#define PAR_P9                 0x9E
#define PAR_P10                0xA0
#define PRESS_ADC              0x1F

// Register addresses for HUM_COMP calculation
#define PAR_H1				   0xE2
#define PAR_H2                 0xE1
#define PAR_H3                 0xE4
#define PAR_H4                 0xE5
#define PAR_H5                 0xE6
#define PAR_H6                 0xE7
#define PAR_H7                 0xE8
#define HUM_ADC                0x25


// Global memory ma

#define STD_PRESSURE 1013.25 //hPa

typedef struct {
  int16_t parT1;
  int16_t parT2;
  int8_t parT3;
  int32_t tempAdc;
  uint16_t parP1;
  uint16_t parP2;
  int8_t parP3;
  uint16_t parP4;
  uint16_t parP5;
  int8_t parP6;
  int8_t parP7;
  uint16_t parP8;
  uint16_t parP9;
  int8_t parP10;
  int16_t parH1;
  int16_t parH2;
  int8_t parH3;
  int8_t parH4;
  int8_t parH5;
  int8_t parH6;
  int8_t parH7;



}bme680_cal_coeff_t;

typedef enum {
  ULTRA_LOW_POWER       = 0x00,
  STANDARD              = 0x01,
  HIGH_RESOLUTION       = 0x02,
  ULTRA_HIGH_RESOLUTION = 0x03
} AccuracyMode_t;


typedef enum {
	SKIPPED = 0,
	OVERSAMPLING_X1  = 1,
	OVERSAMPLING_X2  = 2,
	OVERSAMPLING_X4  = 3,
	OVERSAMPLING_X8  = 4,
	OVERSAMPLING_X16 = 5
} Oversampling_t;

typedef enum {
	SLEEP_MODE = 0,
	FORCED_MODE = 1
} Mode_t;

typedef enum {
	FILTER_COEFF_0   = 0x00,
	FILTER_COEFF_1   = 0x04,
	FILTER_COEFF_3   = 0x08,
	FILTER_COEFF_7   = 0x0C,
	FILTER_COEFF_15  = 0x10,
	FILTER_COEFF_31  = 0x14,
	FILTER_COEFF_63  = 0x18,
	FILTER_COEFF_127 = 0x1C
}IIRFilter_t;

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t            devAddress;
} Bme680_t;


void bme680WriteRegister8(Bme680_t *bme680, uint8_t registerAddress, uint8_t value);

void bme680WriteRegister16(Bme680_t *bme680, uint8_t registerAddress, uint16_t value);

void bme680EnableForcedMode(Bme680_t *bme680);

int32_t bme680ReadRegister(Bme680_t *bme680, uint8_t registerAddress);

void bme680Init(Bme680_t *bme680, I2C_HandleTypeDef *i2c, uint8_t devAddress);

void bme680CustomInit(Bme680_t *bme680, I2C_HandleTypeDef *i2c, uint8_t devAddress, IIRFilter_t coeff);

void bme680Coeff(Bme680_t *bme680);

uint8_t bme680GetId (Bme680_t *bme680);

uint16_t readUncompensatedTemperature(Bme680_t *bme680);

double bme680GetTemperature(Bme680_t *bme680);

double bme680GetPressure(Bme680_t *bme680);

double bme680GetAltitude(Bme680_t *bme680);

double bme680GetHumidity(Bme680_t *bme680);



#endif
