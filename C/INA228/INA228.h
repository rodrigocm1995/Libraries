/**
    *******************************************************************************************
  * @file           : INA228.h
  * @brief          : INA228 Library
    *******************************************************************************************

  * The INA229 is an ultra-precise digital power monitor with a 20-bit delta-sigma ADC specifi-
  * cally designed for current-sensing applications. The device can measure a full-scale diffe-
  * rential input of ±163.84 mV or ±40.96 mV across a resistive shunt sense element with commo-
  * n mode voltage support from -0.3 V to + 85V.
  * 
  * The INA229 reports current, bus voltage, temperature, power, energy and charge accumulation
  * while employing a precision ±0.5% integrated oscillator, all while performing the needed c-
  * alculation in the backgrounds. The integrated temperature sensor is ±1°C accurate for die 
  * temperature measurement and is useful in monitoring the system ambient temperature.
  * 
  * The low offset and gain drift design of the INA229 allows the device to be used in precise
  * systems that do not undergo multi-temperature calibration during manufacturing. Further, t-
  * he very low offset voltage and noise allow for use in mA to kA sensing applications and pr-
  * ovide a wide dynamic range without significant power dissipation losses on the sensing shu-
  * nt element. The low input bias current of the device permits the use of larger current-sen-
  * se resistors, thus providing accurate current measurements in the micro-amp range.
  * 
  * The device allows for selectable ADC conversion times from 50 µs to 4.12 ms as well as sam-
  * ple averaging from 1x to 1024x, which further helps reduce the noise of the measured data. 
  *   
  * @details
  * High Resolution, 20-bit delta-sigma ADC
  * Current monitor accuracy:
  * - Offset voltage: ±1 µV (maximum)
  * - Offset drift: ±0.01 µV/°C (maximum)
  * - Gain error: ±0.05% (maximum)
  * - Gain error drift: ±20 ppm/°C (maximum)
  * - Common mode rejection: 154 dB (minimum)
  * Power monitoring accuracy:
  * - 0.5% full scale, -40°C to +125°C (maximum)
  * Energy and charge accuracy:
  * - 1.0% full scale (maximum)
  * Fast alert response: 75 ns
  * Wide common-mode range: -0.3 V to +85 V
  * Bus voltage sense input: 0 V to 85 V
  * Shunt full-scale differential range: ±163.84 mV / ± 40.96 mV
  * Input bias current: 2.5 nA (maximum)
  * Temperature sensor: ±1°C (maximum at 25°C)
  * Programmable resistor temperature compensation
  * Programmable conversion time and averaging 
  * 10-MHz SPI communication interface
  * Operates from a 2.7 V to 5.5 V supply:
  * - Operational current: 640 µA (typical)
  * - Shuntdown current: 5 µA (maximum) 
  * 
  * @example
  * 
  *******************************************************************************************
  */

#ifndef INC_INA228_H_
#define INC_INA228_H_

#define INA228_ADDRESS							        0x40
#define INA228_TRIALS							          5
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

// Registers
#define INA228_CONFIG_REG						        0x00
#define INA228_ADC_CONFIG_REG					      0x01
#define INA228_SHUNT_CAL_REG					      0x02
#define INA228_SHUNT_TEMPCOEF_REG				    0x03
#define INA228_VSHUNT_REG						        0x04
#define INA228_VBUS_REG							        0x05
#define INA228_DIETEMP_REG						      0x06
#define INA228_CURRENT_REG						      0x07
#define INA228_POWER_REG						        0x08
#define INA228_ENERGY_REG						        0x09
#define INA228_CHARGE_REG						        0x0A
#define INA228_DIAG_ALERT_REG					      0x0B
#define INA228_SOVL_REG							        0x0C
#define INA228_SUVL_REG							        0x0D
#define INA228_BOVL_REG							        0x0E
#define INA228_BUVL_REG							        0x0F
#define INA228_TEMP_LIMIT_REG					      0x10
#define INA228_POWER_LIMIT_REG					    0x11
#define INA228_MANUFACTURER_ID_REG				  0x3E
#define INA228_DEVICE_ID_REG					      0x3F

#define INA228_RESET							          0x8000
#define INA228_RST_ACC							        0x4000

// Masks
#define INA228_CONVDLY_MASK						      0xC03F
#define INA228_TEMPCOMP_MASK					      0xFFDF
#define INA228_ENERGY_CHARGE_RESET_MASK			0xBFFF
#define INA228_ADCRANGE_MASK					      0xFFEF

#define INA228_MODE_MASK						        0x0FFF
#define INA228_VBUSCT_MASK						      0xF1FF
#define INA228_VSHCT_MASK						        0xFE3F
#define INA228_VTCT_MASK						        0xFFC7
#define INA228_AVERAGE_MASK						      0xFFF8

#define INA229_SCALING_FACTOR 					    13107.2E6
#define INA229_DIETEMP_CONV_FACTOR 				  7.8125E-3
#define INA229_VSHUNT_CONV_FACTOR_ADC_0			312.5E-9
#define INA229_VSHUNT_CONV_FACTOR_ADC_1			78.125E-9
#define INA229_VBUS_CONV_FACTOR					    195.3125E-6

typedef enum
{
	INA228_NO_DELAY								            = 0x0000,
	INA228_2_MS_DALY							            = 0x0040,
	INA228_510_MS_DELAY							          = 0x3FC0,
}INA228_ConvDly_HandleTypeDef;

typedef enum
{
	INA228_TEMPERATURE_COMP_DISABLED			    = 0x0000,
	INA228_TEMPERATURE_COMP_ENABLED				    = 0x0020,
}INA228_TempComp_HandleTypeDef;

typedef enum
{
	INA228_ADCRANGE_163_84_MV					        = 0x0000,
	INA228_ADCRANGE_40_96_MV					        = 0x0010,
}INA228_AdcRange_HandleTypeDef;

typedef enum
{
	INA228_SHUTDOWN_MODE						          = 0x0000,
	INA228_BUS_VOLT_ONE_SHOT					        = 0x1000,
  	INA228_SHUNT_VOLT_ONE_SHOT					    = 0x2000,
	INA228_SHUNT_BUS_VOLT_ONE_SHOT				    = 0x3000,
	INA228_TEMP_ONE_SHOT						          = 0x4000,
	INA228_TEMP_BUS_VOLT_ONE_SHOT				      = 0x5000,
	INA228_TEMP_SHUNT_VOLT_ONE_SHOT				    = 0x6000,
	INA228_TEMP_SHUNT_BUS_VOLT_ONE_SHOT			  = 0x7000,
	INA228_BUS_VOLT_CONTINUOUS					      = 0x9000,
	INA228_SHUNT_VOLT_CONTINUOUS				      = 0xA000,
	INA228_SHUNT_BUS_VOLT_CONTINUOUS			    = 0xB000,
	INA228_TEMP_CONTINUOUS						        = 0xC000,
	INA228_TEMP_VUS_VOLT_CONTINUOUS				    = 0xD000,
	INA228_TEMP_SHUNT_VOLT_CONTINUOUS			    = 0xE000,
	INA228_TEMP_SHUNT_BUS_VOLT_CONTINUOUS		  = 0xF000,
}INA228_Mode_HandleTypeDef;

typedef enum
{
	INA228_50_US								              = 0,
	INA228_84_US								              = 1,
	INA228_150_US								              = 2,
	INA228_280_US								              = 3,
	INA228_540_US								              = 4,
	INA228_1052_US								            = 5,
	INA228_2074_US								            = 6,
	INA228_4120_US								            = 7,
}INA228_ConversionTime_HandleTypeDef;

typedef enum
{
	INA228_1_SAMPLE								            = 0x0000,
	INA228_4_SAMPLES							            = 0x0001,
	INA228_16_SAMPLES							            = 0x0002,
	INA228_64_SAMPLES							            = 0x0003,
	INA228_128_SAMPLES							          = 0x0004,
	INA228_256_SAMPLES							          = 0x0005,
	INA228_512_SAMPLES							          = 0x0006,
	INA228_1024_SAMPLES							          = 0x0007
}INA228_Average_HandleTypeDef;

typedef struct
{
	I2C_HandleTypeDef	*hi2c;
	uint8_t 			devAddress;
	_Bool 				AdcRange;
	double 				vshuntConvFactor;
	double 				shuntResistor;
	double 				maximumCurrent;
	double				currentLsb;
}INA228_HandleTypeDef;

HAL_StatusTypeDef INA228_WriteRegister(INA228_HandleTypeDef *ina228, uint8_t registerAddress, uint16_t value);

uint16_t INA228_ReadRegister(INA228_HandleTypeDef *ina228, uint8_t registerAddress);


uint16_t INA228_GetConfig(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetAdcConfig(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetShuntCalibration(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetShuntTemperatureCoefficient(INA228_HandleTypeDef *ina228);

uint32_t INA228_GetShuntVoltage(INA228_HandleTypeDef *ina228);

uint32_t INA228_GetBusVoltage(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetDieTemperature(INA228_HandleTypeDef *ina228);

uint32_t INA228_GetCurrent(INA228_HandleTypeDef *ina228);

uint32_t INA228_GetPower(INA228_HandleTypeDef *ina228);

uint64_t INA228_GetEnergy(INA228_HandleTypeDef *ina228);

uint64_t INA228_GetCharge(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetManufacturerId(INA228_HandleTypeDef *ina228);

uint16_t INA228_GetDeviceId(INA228_HandleTypeDef *ina228);


void INA228_Reset(INA228_HandleTypeDef *ina228);

void INA228_EnergyChargeReset(INA228_HandleTypeDef *ina228);

void INA228_SetConversionDelay(INA228_HandleTypeDef *ina228, INA228_ConvDly_HandleTypeDef delay);

void INA228_SetAdcRange(INA228_HandleTypeDef *ina228, INA228_AdcRange_HandleTypeDef adcRange);


void INA228_SetMode(INA228_HandleTypeDef *ina228, INA228_Mode_HandleTypeDef mode);

void INA228_SetBusVoltageConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef busConvTime);

void INA228_SetShuntVoltageConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef shuntConvTime);

void INA228_SetTemperatureConvTime(INA228_HandleTypeDef *ina228, INA228_ConversionTime_HandleTypeDef tempConvTime);

void INA228_SetAverage(INA228_HandleTypeDef *ina228, INA228_Average_HandleTypeDef avg);

void INA228_SetShuntCalibration(INA228_HandleTypeDef *ina228, double shuntResistor, double maximumCurrent);


double INA228_ReadCurrent(INA228_HandleTypeDef *ina228);


void INA228_Init(INA228_HandleTypeDef *ina228, I2C_HandleTypeDef *i2c);

double INA228_ReadPower(INA228_HandleTypeDef *ina228);

double INA228_ReadEnergy(INA228_HandleTypeDef *ina228);

#endif
