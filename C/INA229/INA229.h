/**
    *******************************************************************************************
  * @file           : INA229.h
  * @brief          : INA229 Library
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

#ifndef INC_INA229_H_
#define INC_INA229_H_

#include "stm32f3xx_hal.h"

#define CONFIG              0x00U
#define ADC_CONFIG          0x01U
#define SHUNT_CAL           0x02U
#define SHUNT_TEMPCO        0x03U
#define VSHUNT              0x04U
#define VBUS                0x05U
#define DIETEMP             0x06U
#define CURRENT             0x07U
#define POWER               0x08U
#define ENERGY              0x09U
#define CHARGE              0x0AU
#define DIAG_ALERT          0x0BU
#define SOVL                0x0CU
#define SUVL                0x0DU
#define BOVL                0x0EU
#define BUVL                0x0FU
#define TEMP_LIMIT          0x10U
#define PWR_LIMIT           0x11U
#define MANUFACTURER_ID     0x3EU
#define DEVICE_ID           0x3FU

#define INA229_SCALING_FACTOR 				13107.2E6
#define INA229_TEMP_CONV_FACTOR 			7.8125E-3
#define INA229_VSHUNT_CONV_FACTOR_ADC_0		        312.5E-9
#define INA229_VSHUNT_CONV_FACTOR_ADC_1		        78.125E-9
#define INA229_VBUS_CONV_FACTOR				195.3125e-9

#define CHECK_BIT(var,pos) ((var) &  (1 << (pos)))
#define _SET_BIT(var,pos)  ((var) |= (1 << (pos)))
#define _CLEAR_BIT(var,pos)  ((var) &= ~(1 << (pos)))

//********************************************************************************************
//CONFIG REGISTER
typedef enum {
	INA229_0_S				= 0x0000,
	INA229_2_MS				= 0x0040,
	INA229510_MS			        = 0x3FC0,
}Ina229_Conv_Dly;

typedef enum {
	INA229_TEMP_COMP_DIS	= 0x0000,
	INA229_TEMP_COMP_ENA	= 0x0020,
}Ina229_Temp_Comp;

typedef enum {
  INA229_ADC_163DOT84_MILIVOLT    = 0x0000, // ADCRANGE = 0 (±163.84mV) = 312.5 nV/LSB
  INA229_ADC_40DOT96_MILIVOLT     = 0x0010  // ADCRANGE = 1 (±40.96mV)  = 78.125 nV/LSB
}Ina229_Adc_Range;
//END OF CONFIG REGISTER
//********************************************************************************************


//********************************************************************************************
//ADC CONFIG REGISTER
typedef enum {
	INA229_SHUNTDOWN				= 0x0000,
	INA229_TRIG_BUS					= 0x1000,
	INA229_TRIG_SHUNT				= 0x2000,
	INA229_TRIG_BUS_SHUNT			        = 0x3000,
	INA229_TRIG_TEMP				= 0x4000,
	INA229_TRIG_TEMP_BUS			        = 0x5000,
	INA229_TRIG_TEMP_SHUNT			        = 0x6000,
	INA229_TRIG_BUS_SHUNT_TEMP		        = 0x7000,
	INA229_BUS_CONT					= 0x9000,
	INA229_SHUNT_CONT				= 0xA000,
	INA229_BUS_SHUNT_CONT			        = 0xB000,
	INA229_TEMP_CONT				= 0xC000,
	INA229_BUS_TEMP					= 0xD000,
	INA229_TEMP_SHUNT				= 0xE000,
	INA229_BUS_SHUNT_TEMP_CONT		        = 0xF000,
}Ina229_Mode;

typedef enum {
	INA229_VBUS_50_US				= 0x0000,
	INA229_VBUS_84_US				= 0x0200,
	INA229_VBUS_150_US				= 0x0400,
	INA229_VBUS_280_US				= 0x0600,
	INA229_VBUS_540_US				= 0x0800,
	INA229_VBUS_1052_US				= 0x0A00,
	INA229_VBUS_2074_US				= 0x0C00,
	INA229_VBUS_4120_US				= 0x0E00,
}Ina229_VBus_CT;

typedef enum {
	INA229_SHUNT_50_US				= 0x0000,
	INA229_SHUNT_84_US				= 0x0040,
	INA229_SHUNT_150_US				= 0x0080,
	INA229_SHUNT_280_US				= 0x00C0,
	INA229_SHUNT_540_US				= 0x0100,
	INA229_SHUNT_1052_US			        = 0x0140,
	INA229_SHUNT_2074_US			        = 0x0180,
	INA229_SHUNT_4120_US			        = 0x01C0,
}Ina229_Shunt_CT;

typedef enum {
	INA229_TEMP_50_US				= 0x0000,
	INA229_TEMP_84_US				= 0x0008,
	INA229_TEMP_150_US				= 0x0010,
	INA229_TEMP_280_US				= 0x0018,
	INA229_TEMP_540_US				= 0x0020,
	INA229_TEMP_1052_US				= 0x0028,
	INA229_TEMP_2074_US				= 0x0030,
	INA229_TEMP_4120_US				= 0x0038,
}Ina229_Temp_CT;

typedef enum {
	INA229_1_SAMPLE					= 0x0000,
	INA229_4_SAMPLES				= 0x0001,
	INA229_16_SAMPLES				= 0x0002,
	INA229_64_SAMPLES				= 0x0003,
	INA229_128_SAMPLES				= 0x0004,
	INA229_256_SAMPLES				= 0x0005,
	INA229_512_SAMPLES				= 0x0006,
	INA229_1024_SAMPLES				= 0x0007
}Ina229_Avg;
//END OF ADC CONFIG REGISTER
//********************************************************************************************

typedef enum {
	INA229_TRANSPARENT				= 0x0000,
	INA229_LATCHED					= 0x8000,
}INA229_Alert_Latch;

typedef enum {
	INA229_DIS_CNVR_ALERT_PIN		= 0x0000,
	INA229_EN_CNVR_ALERT_PIN		= 0x4000,
}Ina229_CNVR;

typedef enum {
	INA229_DIS_COMPARISON_ALERT		= 0x0000,
	INA229_EN_COMPARISON_ALERT		= 0x2000,
}INA229_Slow_Alert;

typedef enum {
	INA229_ACTIVE_LOW_ALERT			= 0x0000,
	INA229_ACTIVE_HIGH_ALERT		= 0x1000,
}INA229_Alert_Polarity;


typedef struct
{
  SPI_HandleTypeDef *spiHandle;
  GPIO_TypeDef      *csPort;
  uint16_t           csPin;

  uint16_t			configRegister;
  uint16_t			adcConfigRegister;
  uint16_t 			shuntCalRegister;
  uint16_t			shuntTempcoRegister;
  uint16_t 			diagAlertRegister;
  uint16_t			sovlRegister;
  uint16_t 			suvlRegister;
  uint16_t			bovlRegister;
  uint16_t 			buvlRegister;
  uint16_t			tempLimitRegister;
  uint16_t			pwrLimitRegister;

  _Bool				adcRange;
  double			currentLsb;
  double 			shuntResistor;
  double 			maximumCurrent;
  double 			shuntCal;
  double 			vshuntConvFactor;
}INA229_t;


void INA229_Init(INA229_t *ina229,
		SPI_HandleTypeDef *spiHandle,
		GPIO_TypeDef *csPort,
		uint16_t csPin);

void INA229_Custom_Init(INA229_t *ina229,
		SPI_HandleTypeDef *spiHandle,
		GPIO_TypeDef *csPort,
		uint16_t csPin,
		Ina229_Conv_Dly dly,
		Ina229_Temp_Comp tpComp,
		Ina229_Adc_Range adcRange);

void INA229_ADC_Config(INA229_t *ina229,
		Ina229_Mode mode,
		Ina229_VBus_CT cbusCt,
		Ina229_Shunt_CT shuntCt,
		Ina229_Temp_CT tempCt,
		Ina229_Avg avg);

void INA229_Temp_ADC_Config(INA229_t *ina229,
		Ina229_Mode,
		Ina229_Temp_CT tempCt,
		Ina229_Avg avg);

uint64_t INA229_ReadRegister(INA229_t *ina229,
		uint8_t registerAddress);

uint8_t INA229_WriteRegister(INA229_t *ina229,
		uint8_t registerAddress,
		uint16_t value);

void INA229_Reset_Registers(INA229_t *ina229);

void INA229_SetCalibration(INA229_t *ina229,
		double shuntResistor,
		uint16_t maxCurrent);

double INA229_Get_Current(INA229_t *ina229);

double INA229_Get_Shunt_Voltage(INA229_t *ina229);

double INA229_Get_Bus_Voltage(INA229_t *ina229);

double INA229_Get_Power(INA229_t *ina229);

double INA229_Get_Energy(INA229_t *ina229);

double INA229_Get_Charge(INA229_t *ina229);

double INA229_Get_Temperature(INA229_t *ina229);

void INA229_Set_Alert(INA229_t *ina229);

_Bool INA229_Get_Energy_OverFlow(INA229_t *ina229);

_Bool INA229_Get_Charge_OverFlow(INA229_t *ina229);

_Bool INA229_Data_Ready(INA229_t *ina229);

#endif
