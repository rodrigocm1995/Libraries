/**
  ******************************************************************************
  * @file           : INA219.h
  * @brief          : INA219 Library
  ******************************************************************************
  * The INA219 is a current shunt and power monitor with a I2C or SMBUS-compati-
  * ble interface. The device monitors both both shunt voltage drop and bus sup-
  * ply voltagewith programmable conversion times and filtering. A programmable 
  * calibration value, combined with an internal multiplier, enables direct read
  * outs of current in amperes. An additional multiplying register calculates po
  * wer in watts.
  * 
  * @details
  * Senses Bus Voltages from 0 to 26 V
  * Reports Current, Voltage, and Power
  * 16 Programmable Addresses
  * High Accuracy: 0.5% (Maximum) Over Temperature (INA219B)
  * Filtering Options
  * Calibration Registers
  * SOT23-8 and SOIC-8 Packages
  * 
  * @example
  * INA219_CALIBRATION_REGISTER is calculated based on the next equation
  * Cal = trunc(0.04096/(Current_LSB * R_Shunt)).....................(1)
  * 
  * Where:
  *   0.04096 is an internal fixed value used to insure scaling
  *   Current_LSB = Maximum Expected Current / 2^15..................(2)
  * 
  * If user assumes a maximum current of 1A and a R_Shunt of 100mOhm then using
  * (2)
  * Current_LSB = 1/2^15 = 30.51 uA/bit .............................(3)
  * 
  * Replacing (3) in (1) yields
  * Cal = (0.04096)/(30.51exp(-6) * 100exp(-3)) = 13425
  * This value must be written in the 16-bit Calibration Register
  * 
  * 
  ******************************************************************************
  */

// The preceding preprocessor wrapper prevents the code between #ifndef and #endif
// from being included if the name INC_INA219_H_ has been defined.
#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "main.h"
#include "stm32f3xx_hal_def.h"

// Slave address when A1 and A0 pins are tied to GND
#define INA219_DEFAULT_ADDRESS                    0x41
#define INA219_TRIALS                             5

// Registers
#define INA219_CONFIGURATION_REG                  0x00   // All-register reset, settings for bus voltage range, PGA gain, ADC Resolution/averaging
#define INA219_SHUNTVOLTAGE_REG                   0x01   // Shunt voltage measurement data
#define INA219_BUSVOLTAGE_REG                     0x02   // Bus voltage measurement data
#define INA219_POWER_REG                          0x03   // power measurement data
#define INA219_CURRENT_REG                        0x04   // Contains the value of the current flowing through the shunt resistor
#define INA219_CALIBRATION_REG                    0x05   // Sets full-scale range and LSB of current and power measurements. Overall system calibration

/*******************  Bits definition for CONFIGURATION register  ******************/
#define INA219_MODE_Pos                         (0U)
#define INA219_MODE_Mask                        (0x7U << INA219_MODE_Pos)
#define INA219_MODE                             INA219_MODE_Mask

#define INA219_SADC_Pos                         (3U)
#define INA219_SADC_Mask                        (0xFU << INA219_SADC_Pos)
#define INA219_SADC                             INA219_SADC_Mask

#define INA219_BADC_Pos                         (7U)
#define INA219_BADC_Mask                        (0xFU << INA219_BADC_Pos)            
#define INA219_BADC                             INA219_BADC_Mask

#define INA219_PG_Pos                           (11U)
#define INA219_PG_Mask                          (0x3U << INA219_PG_Pos)
#define INA219_PG                               INA219_PG_Mask

#define INA219_BRGN_Pos                         (13U)
#define INA219_BRGN_Mask                        (0x1U << INA219_BRGN_Pos)
#define INA219_BRGN                             INA219_BRGN_Mask

#define INA219_RST_Pos                          (15U)
#define INA219_RST_Mask                         (0x1U << INA219_RST_Pos)
#define INA219_RST                              INA219_RST_Mask

/*******************  Bits definition for BUS VOLTAGE register  ******************/
#define INA219_OVF_Pos                          (0U)
#define INA219_OVF_Mask                         (0x1U << INA219_OVF_Pos)
#define INA219_OVF                              INA219_OVF_Mask

#define INA219_CNVR_Pos                         (1U)
#define INA219_CNVR_Mask                        (0x1U << INA219_CNVR_Pos)
#define INA219_CNVR                             INA219_CNVR_Mask

#define INA219_BD_Pos                           (3U)
#define INA219_BD_Mask                          (0x1FFF << INA219_BD_Pos)
#define INA219_BD                               INA219_BD_Mask

typedef enum {
  INA219_BUSVOLTAGERANGE_16V                  = 0x0U, 
  INA219_BUSVOLTAGERANGE_32V                  = 0x1U,
} INA219_BusVoltageRange_TypeDef;

typedef enum {
  INA219_PGAGAIN_40_MILI_VOLT                 = 0x0U, // Range +- 40mV
  INA219_PGAGAIN_80_MILI_VOLT                 = 0x1U, // Range +- 80mV
  INA219_PGAGAIN_160_MILI_VOLT                = 0x2U, // Range +- 160mV
  INA219_PGAGAIN_320_MILI_VOLT                = 0x3U  // Range +- 320mV 
} INA219_ShuntVoltagePGA_TypeDef;

typedef enum {
  INA219_ADC_9_BIT_RESOLUTION                   = 0x0U,  // 84 us
  INA219_ADC_10_BIT_RESOLUTION                  = 0x1U, // 148 us
  INA219_ADC_11_BIT_RESOLUTION                  = 0x2U, // 276 us
  INA219_ADC_12_BIT_RESOLUTION                  = 0x3U, // 532 us
  INA219_ADC_2_SAMPLES                          = 0x9U, // 1.06 ms
  INA219_ADC_4_SAMPLES                          = 0xAU, // 2.13 ms
  INA219_ADC_8_SAMPLES                          = 0xBU, // 4.26 ms
  INA219_ADC_16_SAMPLES                         = 0xCU, // 8.51 ms
  INA219_ADC_32_SAMPLES                         = 0xDU, // 17.02 ms
  INA219_ADC_64_SAMPLES                         = 0xEU, // 34.05 ms
  INA219_ADC_128_SAMPLES                        = 0xFU  // 68.10 ms
} INA219_ADCResolution_TypeDef;

typedef enum {
  INA219_POWERDOWN_MODE                       = 0x0U,
  INA219_SHUNTVOLTAGETRIG_MODE                = 0x1U,
  INA219_BUSVOLTAGETRIG_MODE                  = 0x2U,
  INA219_SHUNTBUS_TRIG_MODE                   = 0x3U,
  INA219_ADC_OFF_DISABLED_MODE                = 0x4U,
  INA219_SHUNTVOLTAGE_CONTINUOUS_MODE         = 0x5U,
  INA219_BUSVOLTAGE_CONTINUOUS_MODE           = 0x6U,
  INA219_SHUNTBUS_CONTINUOUS_MODE             = 0x7U,
} INA219_Mode_TypeDef;

/**
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains the configuration information for the specified I2C.
  * @param  devAddress Target device address: The device 7 bits address value
  */
typedef struct {
	I2C_HandleTypeDef *hi2c;
  	uint8_t _devAddress;
  	float   _shuntResistor;
  	float   _maximumCurrent;
    float   _currentLsbMin;
    float   _currentLsb;
    float   _vShuntAcc;
    float   _vBusAcc;
} INA219_HandleTypeDef; 

HAL_StatusTypeDef INA219_Init(INA219_HandleTypeDef *ina219, I2C_HandleTypeDef *i2c, uint8_t devAddress);

HAL_StatusTypeDef INA219_GetConfiguration(INA219_HandleTypeDef *ina219, uint16_t *value);
HAL_StatusTypeDef INA219_GetShuntVoltage(INA219_HandleTypeDef *ina219, uint16_t *value);
HAL_StatusTypeDef INA219_GetBusVoltage(INA219_HandleTypeDef *ina219, uint16_t *value);
HAL_StatusTypeDef INA219_GetPower(INA219_HandleTypeDef *ina219, uint16_t *value);
HAL_StatusTypeDef INA219_GetCurrent(INA219_HandleTypeDef *ina219, uint16_t *value);
HAL_StatusTypeDef INA219_GetCalibration(INA219_HandleTypeDef *ina219, uint16_t *value);


HAL_StatusTypeDef INA219_SetMode(INA219_HandleTypeDef *ina219, INA219_Mode_TypeDef mode);
HAL_StatusTypeDef INA219_SetShuntADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution_TypeDef shuntADCResolution);
HAL_StatusTypeDef INA219_SetBusADCResolution(INA219_HandleTypeDef *ina219, INA219_ADCResolution_TypeDef busADCResolution);
HAL_StatusTypeDef INA219_SetShuntVoltageRange(INA219_HandleTypeDef *ina219, INA219_ShuntVoltagePGA_TypeDef shuntVoltageRange);
HAL_StatusTypeDef INA219_SetBusVoltageRange(INA219_HandleTypeDef *ina219, INA219_BusVoltageRange_TypeDef busVoltageRange);
HAL_StatusTypeDef INA219_SetCalibration(INA219_HandleTypeDef *ina219, float rShuntValue, float maxCurrent);
HAL_StatusTypeDef INA219_SetCorrectedCalibration(INA219_HandleTypeDef *ina219, uint16_t calValue, float expectedCurrent, float measuredCurrent);
HAL_StatusTypeDef INA219_ResetDevice(INA219_HandleTypeDef *ina219);

HAL_StatusTypeDef INA219_ReadShuntVoltage_mV(INA219_HandleTypeDef *ina219, float *shuntVoltage_mV);
HAL_StatusTypeDef INA219_ReadBusVoltage_V(INA219_HandleTypeDef *ina219, float *busVoltage_V);
HAL_StatusTypeDef INA219_ReadCurrent_A(INA219_HandleTypeDef *ina219, float *current_A);
HAL_StatusTypeDef INA219_ReadPower_W(INA219_HandleTypeDef *ina219, float *power_W);
_Bool INA219_IsDataReady(INA219_HandleTypeDef *ina219);

#endif
