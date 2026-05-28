/**
    *******************************************************************************************
  * @file           : INA236.h
  * @brief          : INA236 Library
    *******************************************************************************************

  * The INA236 is a 16-bit digital current monitor with an I2C interface that is
  * compliant with a wide range of digital bus voltage such as 1.2V, 1.8V, 3.3V,
  * and 5.0V. The device monitors the voltage across an external sense resistor
  * and reports values for current, bus voltage, and power
  * 
  * @details
  * High-side or low-side current sensing 
  * Operates from a 1.7V to 5.5V power supply
  * Reports current, voltage and power
  * Programmable full scale range: 20mV / 80mV
  * 16-bit ADC resolution
  * 0.1% gain error (maximum)
  * 5uV offset (maximum)
  * Low input bias current: 10nA (maximum)
  * Configurable averaging options
  * General call addressing allows convertion synchronization among devices
  * Alert limits for over and under current events
  * 1.2V compliant I2C bus, SMBus interface
  * Two device address options with a 4-pin selectable address
  * Operating temperature: -40°C y +125°C 
  * 
  * @example
  * INA236_CALIBRATION_REGISTER is calculated based on the next equation
  * SHUNT_CAL = 0.00512/(Current_LSB x Rshunt).......................(1)
  * 
  * Where:
  *   0.00512 is an internal fixed value used to ensure scaling is maintained properly
  *   Current_LSB is a selected value for the current step size in A. Must be greater than 
  *   or equal to CURRENT_LSB (minimum), but less than 8 x CURRENT_LSB (minimun) to 
  *   reduce resolution loss.
  *   The SHUNT_CAL must be divided by 4 for ADCRANGE = 1
  * 
  * CURRENT_LSB (minimum) = Maximum Expected Current / 2^15..........(2)
  * 
  * After programming the SHUNT_CAL register with the calculated value, the measured current
  * in amperes can be read from the CURRENT register. Use equation (3) to calculate the final
  * value scaled by the CURRENT_LSB:
  * 
  * Current[A] = CURRENT_LSB x CURRENT...............................(3)
  * 
  * Where:
  *   CURRENTis the value read from the CURRENT register
  * 
  * Design Parameters
  * * Power Supply Voltage (Vs)        = 3.3V
  * * Bus Supply  rail (VCM)           = 12V
  * * Average Current                  = 6A
  * * OverCurrent fault threshold      = 9A
  * * Maximum current monitored (Imax) = 10A
  * * ADC Range Selection (Vsense_max) = +-81.92mV 
  * 
  * 1. Select the Shunt Resistor
  * 
  * Rshunt < V_SENSE_MAX / I_MAX ....................................(4)
  * 
  *******************************************************************************************
*/

#ifndef INC_INA236_H_
#define INC_INA236_H_

/* Default I2C configuration values */
#define INA236_DEFAULT_ADDRESS          0x40 /* Default I2C address of INA236 (A0=GND, A1=GND) */
#define INA236_TRIALS                   5    /* Number of connection trials for I2C communication */

/* Macro to check the state of a specific bit in a variable */
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

/* INA236 Register Map Addresses */
#define INA236_CONFIGURATION_REGISTER   0x00  /* Configuration Register - Reset Value = 4127h */
#define INA236_SHUNT_VOLTAGE_REGISTER   0x01  /* Shunt Voltage Register - Reset Value = 0000h */
#define INA236_BUS_VOLTAGE_REGISTER     0x02  /* Bus Voltage Register   - Reset Value = 0000h */
#define INA236_POWER_REGISTER           0x03  /* Power Register         - Reset Value = 0000h */
#define INA236_CURRENT_REGISTER         0x04  /* Current Register       - Reset Value = 0000h */
#define INA236_CALIBRATION_REGISTER     0x05  /* Calibration Register   - Reset Value = 0000h */
#define INA236_MASK_ENABLE_REGISTER     0x06  /* Mask/Enable Register   - Reset Value = 0000h */
#define INA236_ALERT_LIMIT_REGISTER     0x07  /* Alert Limit Register   - Reset Value = 0000h */
#define MANUFACTURER_ID_REGISTER        0x3E  /* Device ID Register     - Reset Value = 5449h */  
#define DEVICE_ID_REGISTER              0x3F  /* Manufacturer ID Register -  Reset Value = A080h */

/*******************  Bits definition for CONFIGURATION register  ******************/
#define INA236_MODE_Pos                 (0U)
#define INA236_MODE_Mask                (0x7U << INA236_MODE_Pos)
#define INA236_MODE                     INA236_MODE_Mask
#define INA236_MODE_0                   (0x1U << INA236_MODE_Pos)
#define INA236_MODE_1                   (0x2U << INA236_MODE_Pos)
#define INA236_MODE_2                   (0x4U << INA236_MODE_Pos)

#define INA236_VSHCT_Pos                (3U)
#define INA236_VSHCT_Mask               (0x7U << INA236_VSHCT_Pos)
#define INA236_VSHCT                    INA236_VSHCT_Mask
#define INA236_VSHCT_0                  (0x1U << INA236_VSHCT_Pos)
#define INA236_VSHCT_1                  (0x2U << INA236_VSHCT_Pos)
#define INA236_VSHCT_2                  (0x4U << INA236_VSHCT_Pos)

#define INA236_VBUSCT_Pos               (6U)
#define INA236_VBUSCT_Mask              (0x7U << INA236_VBUSCT_Pos)
#define INA236_VBUSCT                   INA236_VBUSCT_Mask
#define INA236_VBUSCT_0                 (0x1U << INA236_VBUSCT_Pos)
#define INA236_VBUSCT_1                 (0x2U << INA236_VBUSCT_Pos)
#define INA236_VBUSCT_2                 (0x4U << INA236_VBUSCT_Pos)

#define INA236_AVG_Pos                  (9U)
#define INA236_AVG_Mask                 (0x7U << INA236_AVG_Pos)
#define INA236_AVG                      INA236_AVG_Mask
#define INA236_AVG_0                    (0x1U << INA236_AVG_Pos)
#define INA236_AVG_1                    (0x2U << INA236_AVG_Pos)
#define INA236_AVG_2                    (0x4U << INA236_AVG_Pos)

#define INA236_ADCRANGE_Pos             (12U)
#define INA236_ADCRANGE_Mask            (0x1U << INA236_ADCRANGE_Pos)
#define INA236_ADCRANGE                 INA236_ADCRANGE_Mask

#define INA236_RST_Pos                  (15U)
#define INA236_RST_Mask                 (0x1U << INA236_RST_Pos)
#define INA236_RST                      INA236_RST_Mask

/*******************  Bits definition for MASK/ENABLE register  ******************/
#define INA236_LEN_Pos                  (0U)
#define INA236_LEN_Mask                 (0x1U << INA236_LEN_Pos)
#define INA236_LEN                      INA236_LEN_Mask

#define INA236_APOL_Pos                 (1U)
#define INA236_APOL_Mask                (0x1U << INA236_APOL_Pos)
#define INA236_APOL                     INA236_APOL_Mask

#define INA236_OVF_Pos                  (2U)
#define INA236_OVF_Mask                 (0x1U << INA236_OVF_Pos)
#define INA236_OVF                      INA236_OVF_Mask

#define INA236_CVRF_Pos                 (3U)
#define INA236_CVRF_Mask                (0x1U << INA236_CVRF_Pos)
#define INA236_CVRF                     INA236_CVRF_Mask

#define INA236_AFF_Pos                  (4U)
#define INA236_AFF_Mask                 (0x1U << INA236_AFF_Pos)
#define INA236_AFF                      INA236_AFF_Mask

#define INA236_MEMERROR_Pos             (5U)
#define INA236_MEMERROR_Mask            (0x1U << INA236_MEMERROR_Pos)
#define INA236_MEMERROR                 INA236_MEMERROR_Mask

#define INA236_CNVR_Pos                 (10U)
#define INA236_CNVR_Mask                (0x1U << INA236_CNVR_Pos)
#define INA236_CNVR                      INA236_CNVR_Mask

#define INA236_POL_Pos                  (11U)
#define INA236_POL_Mask                 (0x1U << INA236_POL_Pos)
#define INA236_POL                      INA236_POL_Mask

#define INA236_BUL_Pos                  (12U)
#define INA236_BUL_Mask                 (0x1U << INA236_BUL_Pos)
#define INA236_BUL                      INA236_BUL_Mask

#define INA236_BOL_Pos                  (13U)
#define INA236_BOL_Mask                 (0x1U << INA236_BOL_Pos)
#define INA236_BOL                      INA236_BOL_Mask

#define INA236_SUL_Pos                  (14U)
#define INA236_SUL_Mask                 (0x1U << INA236_SUL_Pos)
#define INA236_SUL                      INA236_SUL_Mask

#define INA236_SOL_Pos                  (15U)
#define INA236_SOL_Mask                 (0x1U << INA236_SOL_Pos)
#define INA236_SOL                      INA236_SOL_Mask

typedef enum 
{
    INA236_SHUTDOWN                     = 0x0U,
    INA236_SHUNT_VOLTAGE_ONE_SHOT       = 0x1U,
    INA236_BUS_VOLTAGE_ONE_SHOT         = 0x2U,
    INA236_SHUNT_BUS_VOLTAGE_ONE_SHOT   = 0x3U,
    INA236_CONTINUOUS_SHUNT_VOLTAGE     = 0x5U,
    INA236_CONTINUOUS_BUS_VOLTAGE       = 0x6U,
    INA236_CONTINUOUS_SHUNT_BUS_VOLTAGE = 0x7U
} INA236_Mode_TypeDef;

typedef enum 
{
    INA236_140_US                       = 0x0U,
    INA236_204_US                       = 0x1U,
    INA236_332_US                       = 0x2U,
    INA236_588_US                       = 0x3U,
    INA236_1100_US                      = 0x4U,
    INA236_2116_US                      = 0x5U,
    INA236_4156_US                      = 0x6U,
    INA236_8244_US                      = 0x7U
} INA236_ConvTime_TypeDef;

typedef enum
{
    INA236_1_SAMPLE                     = 0x0U,
    INA236_4_SAMPLES                    = 0x1U,
    INA236_16_SAMPLES                   = 0x2U,
    INA236_64_SAMPLES                   = 0x3U,
    INA236_128_SAMPLES                  = 0x4U,
    INA236_256_SAMPLES                  = 0x5U,
    INA236_512_SAMPLES                  = 0x6U,
    INA236_1014_SAMPLES                 = 0x7U
} INA236_Avg_TypeDef;

typedef enum
{
    INA236_ADC_RANGE_81_92_MV           = 0x0U,
    INA236_ADC_RANGE20_48_MV            = 0x1U
} INA236_AdcRange_TypeDef;

typedef enum
{
    INA236_ACTIVE_LOW                   = 0x0U,
    INA236_ACTIVE_HIGH                  = 0x1U
} INA236_AlertPol_TypeDef;

typedef enum
{
    INA236_CONV_READY_ALERT_DISABLE     = 0x0U,
    INA236_CONV_READY_ALERT_ENABLE      = 0x1U
} INA236_ConvRdy_TypeDef;

typedef enum
{
    INA236_POWER_LIMIT_ALERT_DISABLE    = 0x0U, 
    INA236_POWER_LIMIT_ALERT_ENABLE     = 0x1U  
} INA236_PowerAlert_TypeDef;

typedef enum
{
    INA236_BUS_UNDER_LIMIT_ALERT_DISABLE    = 0x0U, 
    INA236_BUS_UNDER_LIMIT_ALERT_ENABLE     = 0x1U  
} INA236_BusUnderLimitAlert_TypeDef;

typedef enum
{
    INA236_BUS_OVER_LIMIT_ALERT_DISABLE     = 0x0U, 
    INA236_BUS_OVER_LIMIT_ALERT_ENABLE      = 0x1U  
} INA236_BusOverLimitAlert_TypeDef;

typedef enum
{
    INA236_SHUNT_UNDER_LIMIT_ALERT_DISABLE  = 0x0U, 
    INA236_SHUNT_UNDER_LIMIT_ALERT_ENABLE   = 0x1U  
} INA236_ShuntUnderLimitAlert_TypeDef;

typedef enum
{
    INA236_SHUNT_OVER_LIMIT_ALERT_DISABLE   = 0x0U, 
    INA236_SHUNT_OVER_LIMIT_ALERT_ENABLE    = 0x1U  
} INA236_ShuntOverLimitAlert_TypeDef;

typedef enum {
  SOL = 1, // Shunt Over Limit
  SUL = 2, // Shunt Under Limit
  BOL = 3, // Bus Over limit
  BUL = 4, // Bus Under Limit
  POL = 5,  // Power Over Limit
  CNVR = 6 // Conversion Ready
} AlertType_t;

typedef struct 
{
  I2C_HandleTypeDef  *hi2c;
  uint8_t            _devAddress;
  _Bool              _adcRange;
  double             _resolution;
  double             _shuntResistor;
  double             _maximumCurrent;
  double 			 _shuntAdcRange;
  double             _currentLsbMin;
  double            _currentLsb;
  AlertType_t        alertType;
} INA236_HandleTypeDef;


/* Prototypes for low-level register interface */
HAL_StatusTypeDef INA236_WriteRegister(INA236_HandleTypeDef *ina236, uint8_t registerAddress, uint16_t value);
uint16_t INA236_ReadRegister(INA236_HandleTypeDef *ina236, uint8_t registerAddress);

/* Initialization & Calibration */
HAL_StatusTypeDef INA236_Init(INA236_HandleTypeDef *ina236, I2C_HandleTypeDef *i2c, uint8_t devAddress);
void INA236_SetCalibration(INA236_HandleTypeDef *ina236, double rShuntValue, int maxCurrent);
void INA236_ResetDevice(INA236_HandleTypeDef *ina236);

/* Configuration Setters & Getters */
void INA236_SetMode(INA236_HandleTypeDef *ina236, INA236_Mode_TypeDef mode);
void INA236_SetShuntConvTime(INA236_HandleTypeDef *ina236, INA236_ConvTime_TypeDef convTime);
void INA236_SetBusConvTime(INA236_HandleTypeDef *ina236, INA236_ConvTime_TypeDef convTime);
void INA236_SetAverage(INA236_HandleTypeDef *ina236, INA236_Avg_TypeDef avg);
void INA236_SetAdcRange(INA236_HandleTypeDef *ina236, INA236_AdcRange_TypeDef range);

/* Alert Management */
void INA236_SetAlertPolarity(INA236_HandleTypeDef *ina236, INA236_AlertPol_TypeDef polarity);
INA236_AlertPol_TypeDef INA236_GetAlertPolarity(INA236_HandleTypeDef *ina236);
void INA236_SetAlertPin(INA236_HandleTypeDef *ina236, INA236_ConvRdy_TypeDef cnvr);
INA236_ConvRdy_TypeDef INA236_GetAlertPin(INA236_HandleTypeDef *ina236);
void INA236_SetPowerOverLimit(INA236_HandleTypeDef *ina236, INA236_PowerAlert_TypeDef powerAlert);
void INA236_SetBusUnderLimit(INA236_HandleTypeDef *ina236, INA236_BusUnderLimitAlert_TypeDef bulAlert);
void INA236_SetBusOverLimit(INA236_HandleTypeDef *ina236, INA236_BusOverLimitAlert_TypeDef bolAlert);
void INA236_SetShuntUnderLimit(INA236_HandleTypeDef *ina236, INA236_ShuntUnderLimitAlert_TypeDef sulAlert);
void INA236_SetShuntOverLimit(INA236_HandleTypeDef *ina236, INA236_ShuntOverLimitAlert_TypeDef solAlert);

/* Diagnostic Readings */
uint8_t INA236_IsConversionReady(INA236_HandleTypeDef *ina236);
uint8_t INA236_IsMathOverflowReady(INA236_HandleTypeDef *ina236);
uint16_t INA236_GetManufacturerID(INA236_HandleTypeDef *ina236);
uint16_t INA236_GetDeviceID(INA236_HandleTypeDef *ina236);

/* Measurement Readings */
double INA236_GetShuntVoltage_mV(INA236_HandleTypeDef *ina236);
double INA236_GetBusVoltage(INA236_HandleTypeDef *ina236);
double INA236_GetCurrent_A(INA236_HandleTypeDef *ina236);
double INA236_GetPower_W(INA236_HandleTypeDef *ina236);
#endif
