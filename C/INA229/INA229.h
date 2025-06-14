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
#define SHUNT_CALL          0x02U
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



#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

typedef enum {
  INA229_ADC_163DOT84_MILIVOLT    = 0x0000, // ADCRANGE = 0 (±163.84mV)
  INA229_ADC_40DOT96_MILIVOLT     = 0x0010  // ADCRANGE = 1 (±40.96mV). 625nV/LSB
} INA229_AdcRange_t;



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

  uint16_t			adcRange;
  double			currentLsb;
}INA229_t;


void INA229_Init(INA229_t *ina229, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin);

uint64_t INA229_ReadRegister(INA229_t *ina229, uint8_t registerAddress);

uint8_t INA229_WriteRegister(INA229_t *ina229, uint8_t registerAddress, uint8_t value);

#endif

