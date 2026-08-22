/**
    *******************************************************************************************
  * @file           : MAX30102.h
  * @brief          : MAX30102 Library
    *******************************************************************************************

    *******************************************************************************************
  */

#ifndef INC_MAX30102_H_
#define INC_MAX30102_H_

#include "main.h" 


#define MAX30102_ADDRESS                          0x48
#define MAX30102_TRIALS                           5

// Registers
/* STATUS */
#define MAX30102_INTERRUPT_STATUS_1_REG           0x00  // Read-only
#define MAX30102_INTERRUPT_STATUS_2_REG           0x01  // Read-only
#define MAX30102_INTERRUPT_ENABLE_1_REG           0x02  // R/W
#define MAX30102_INTERRUPT_ENABLE_2_REG           0x03  // R/W
/*  FIFO */
#define MAX30102_FIFO_WRITE_PINTER_REG            0x04  // R/W
#define MAX30102_OVERFLOW_COUNTER_REG             0x05  // R/W
#define MAX30102_FIFO_READ_POINTER_REG            0x06  // R/W
#define MAX30102_FIFO_DATA_REG                    0x07  // R/W
/* CONFIGURATION */
#define MAX30102_FIFO_CONFIGURATION_REG           0x08  // R/W
#define MAX30102_MODE_CONFIGURATION_REG           0x09  // R/W
#define MAX30102_SPO2_CONFIGURATION_REG           0x0A  // R/W
#define MAX30102_LED_PULSE_AMPLITUDE_1_REG        0x0C  // R/W
#define MAX30102_LED_PULSE_AMPLITUDE_2_REG        0x0D  // R/W
#define MAX30102_MULTI_LED_MODE_CTRL_1_REG        0x11  // R/W
#define MAX30102_MULTI_LED_MODE_CTRL_2_REG        0x12  // R/W
/* DIE TEMPERATURE */
#define MAX30102_DIE_TEMP_REG                     0x1F  // Read-only
#define MAX30102_DIE_TEMP_FRACTION_REG            0x20  // Read-only
#define MAX30102_DIE_TEMP_CONFIG_REG              0x21  // R/W
/* PART ID */
#define MAX30102_REVISION_ID                      0xFE  // Read-only
#define MAX30102_PART_ID                          0xFF  // Read-only

/*******************  Bits definition for INTERRUPT STATUS 1 register  ******************/
#define MAX30102_PWR_RDY_Pos                      (0U)
#define MAX30102_PWR_RDY_Mask                     (0x1U << MAX30102_PWR_RDY_Pos)
#define MAX30102_PWR_RDY                          MAX30102_PWR_RDY_Mask

#define MAX30102_ALC_OVF_Pos                      (5U)
#define MAX30102_ALC_OVF_Mask                     (0x1U << MAX30102_ALC_OVF_Pos)
#define MAX30102_ALC_OVF                          MAX30102_ALC_OVF_Mask

#define MAX30102_PPG_RDY_Pos                      (6U)
#define MAX30102_PPG_RDY_Mask                     (0x1U << MAX30102_PPG_RDY_Pos)
#define MAX30102_PPG_RDY                          MAX30102_PPG_RDY_Mask

#define MAX30102_A_FULL_Pos                       (7U)
#define MAX30102_A_FULL_Mask                      (0x1U << MAX30102_A_FULL_Pos)
#define MAX30102_A_FULL                           MAX30102_A_FULL_Mask

/*******************  Bits definition for INTERRUPT STATUS 2 register  ******************/
#define MAX30102_DIE_TEMP_RDY_Pos                 (1U)
#define MAX30102_DIE_TEMP_RDY_Mask                (0x1U << MAX30102_DIE_TEMP_RDY_Pos)
#define MAX30102_DIE_TEMP_RDY                     MAX30102_DIE_TEMP_RDY_Mask

/*******************  Bits definition for INTERRUPT ENABLE 1 register  ******************/
#define MAX30102_ALC_OVF_EN_Pos                   (5U)
#define MAX30102_ALC_OVF_EN_Mask                  (0x1U << MAX30102_ALC_OVF_EN_Pos)
#define MAX30102_ALC_OVF_EN                       MAX30102_ALC_OVF_EN_Mask

#define MAX30102_PPG_RDY_EN_Pos                   (6U)
#define MAX30102_PPG_RDY_EN_Mask                  (0x1U << MAX30102_PPG_RDY_EN_Pos)
#define MAX30102_PPG_RDY_EN                       MAX30102_PPG_RDY_EN_Mask

#define MAX30102_A_FULL_EN_Pos                    (7U)
#define MAX30102_A_FULL_EN_Mask                   (0x1U << MAX30102_A_FULL_EN_Pos)
#define MAX30102_A_FULL_EN                        MAX30102_A_FULL_EN_Mask

/*******************  Bits definition for INTERRUPT STATUS 2 register  ******************/
#define MAX30102_DIE_TEMP_RDY_EN_Pos              (1U)
#define MAX30102_DIE_TEMP_RDY_EN_Mask             (0x1U << MAX30102_DIE_TEMP_RDY_EN_Pos)
#define MAX30102_DIE_TEMP_RDY_EN                  MAX30102_DIE_TEMP_RDY_EN_Mask

/*******************  Bits definition for FIFO WRITE POINTER register  ******************/
#define MAX30102_FIFO_WR_PTR_Pos                  (0U)
#define MAX30102_FIFO_WR_PTR_Mask                 (0x1FU << MAX30102_FIFO_WR_PTR_Pos)
#define MAX30102_FIFO_WR_PTR                      MAX30102_FIFO_WR_PTR_Mask

/*******************  Bits definition for OVERFLOW COUNTER register  *******************/
#define MAX30102_OVF_COUNTER_Pos                  (0U)
#define MAX30102_OVF_COUNTER_Mask                 (0x1FU << MAX30102_OVF_COUNTER_Pos)
#define MAX30102_OVF_COUNTER                      MAX30102_OVF_COUNTER_Mask

/*******************  Bits definition for FIFO READ POINTER register  ******************/
#define MAX30102_FIFO_RD_PTR_Pos                  (0U)
#define MAX30102_FIFO_RD_PTR_Mask                 (0x1FU << MAX30102_FIFO_RD_PTR_Pos)
#define MAX30102_FIFO_RD_PTR                      MAX30102_FIFO_RD_PTR_Mask

/*******************  Bits definition for FIFO CONFIGURATION register  ****************/
#define MAX30102_FIFO_A_FULL_Pos                  (0U)
#define MAX30102_FIFO_FA_ULL_Mask                 (0xFU << MAX30102_FIFO_A_FULL_Pos)
#define MAX30102_FIFO_A_FULL                      MAX30102_FIFO_FA_ULL_Mask

#define MAX30102_FIFO_ROL_LOVER_EN_Pos            (4U)
#define MAX30102_FIFO_ROL_LOVER_EN_Mask           (0x1U << MAX30102_FIFO_ROL_LOVER_EN_Pos)
#define MAX30102_FIFO_ROL_LOVER_EN                MAX30102_FIFO_ROL_LOVER_EN_Mask

#define MAX30102_SMP_AVE_Pos                      (5U)
#define MAX30102_SMP_AVE_Mask                     (0x7U << MAX30102_SMP_AVE_Pos)
#define MAX30102_SMP_AVE                          MAX30102_SMP_AVE_Mask

/*******************  Bits definition for MODE CONFIGURATION register  ****************/
#define MAX30102_MODE_Pos                        (0U)
#define MAX30102_MODE_Mask                       (0x7U << MAX30102_MODE_Pos)
#define MAX30102_MODE                            MAX30102_MODE_Mask

#define MAX30102_RESET_Pos                       (6U)
#define MAX30102_RESET_Mask                      (0x1U << MAX30102_RESET_Pos)
#define MAX30102_RESET                           MAX30102_RESET_Mask

#define MAX30102_SHDN_Pos                        (7U)
#define MAX30102_SHDN_Mask                       (0x1U << MAX30102_SHDN_Pos)
#define MAX30102_SHDN                            MAX30102_SHDN_Mask

/*******************  Bits definition for SPO2 CONFIGURATION register  ****************/
#define MAX30102_LED_PW_Pos                      (0U)
#define MAX30102_LED_PW_Mask                     (0x3U << MAX30102_LED_PW_Pos)
#define MAX30102_LED_PW                          MAX30102_LED_PW_Mask

#define MAX30102_SPO2_SR_Pos                     (2U)
#define MAX30102_SPO2_SR_Mask                    (0x7U << MAX30102_SPO2_SR_Pos)
#define MAX30102_SPO2_SR                         MAX30102_SPO2_SR_Mask

#define MAX30102_SPO2_ADC_RGE_Pos                (5U)
#define MAX30102_SPO2_ADC_RGE_Mask               (0x3U << MAX30102_SPO2_ADC_RGE_Pos)
#define MAX30102_SPO2_ADC_RGE                    MAX30102_SPO2_ADC_RGE_Mask

typedef enum
{
  MAX30102_1_SAMPLE                             = 0x0U,
  MAX30102_2_SAMPLES                            = 0x1U,
  MAX30102_4_SAMPLES                            = 0x2U,
  MAX30102_8_SAMPLES                            = 0x3U,
  MAX30102_16_SAMPLES                           = 0x4U,
  MAX30102_32_SAMPLES                           = 0x5U,
}MAX30102_SampleAve_TypeDef;

typedef enum
{
  MAX30102_0_EMPTY_DTA_SAMPLES                  = 0x0U,
  MAX30102_1_EMPTY_DTA_SAMPLES                  = 0x1U,
  MAX30102_2_EMPTY_DTA_SAMPLES                  = 0x2U,
  MAX30102_3_EMPTY_DTA_SAMPLES                  = 0x3U,
  MAX30102_4_EMPTY_DTA_SAMPLES                  = 0x4U,
  MAX30102_5_EMPTY_DTA_SAMPLES                  = 0x5U,
  MAX30102_6_EMPTY_DTA_SAMPLES                  = 0x6U,
  MAX30102_7_EMPTY_DTA_SAMPLES                  = 0x7U,
  MAX30102_8_EMPTY_DTA_SAMPLES                  = 0x8U,
  MAX30102_9_EMPTY_DTA_SAMPLES                  = 0x9U,
  MAX30102_10_EMPTY_DTA_SAMPLES                 = 0xAU,
  MAX30102_11_EMPTY_DTA_SAMPLES                 = 0xBU,
  MAX30102_12_EMPTY_DTA_SAMPLES                 = 0xCU,
  MAX30102_13_EMPTY_DTA_SAMPLES                 = 0xDU,
  MAX30102_14_EMPTY_DTA_SAMPLES                 = 0xEU,
  MAX30102_15_EMPTY_DTA_SAMPLES                 = 0xFU,
} MAX30102_FifoAlmostFull_TypeDef;

typedef enum
{
  MAX30102_HEART_RATE_MODE                      = 0x2U,    /* Red only */
  MAX30102_SPO2_MODE                            = 0x3U,    /* Red and IR */
  MAX30102_MULTI_LED_MODE	                    = 0x7U,    /* Red and IR */
} MAX30102_Mode_TypeDef;

typedef enum
{
  MAX30102_FULL_SCALE_2048                      = 0x0U,
  MAX30102_FULL_SCALE_4096                      = 0x1U,
  MAX30102_FULL_SCALE_8192                      = 0x2U,
  MAX30102_FULL_SCALE_16384                     = 0x3U,
} MAX30102_SPO2AdcRange_TypeDef;

typedef enum
{
  MAX30102_50_SAMPLES_PER_SECOND                = 0x0U,
  MAX30102_100_SAMPLES_PER_SECOND               = 0x1U,
  MAX30102_200_SAMPLES_PER_SECOND               = 0x2U,
  MAX30102_400_SAMPLES_PER_SECOND               = 0x3U,
  MAX30102_800_SAMPLES_PER_SECOND               = 0x4U,
  MAX30102_1000_SAMPLES_PER_SECOND              = 0x5U,
  MAX30102_1600_SAMPLES_PER_SECOND              = 0x6U,
  MAX30102_3200_SAMPLES_PER_SECOND              = 0x7U,
} MAX30102_SPO2SampleRate_TypeDef;

typedef enum
{
  MAX30102_PULSE_WITDH_69_US                    = 0x0U,
  MAX30102_PULSE_WITDH_118_US                   = 0x1U,
  MAX30102_PULSE_WITDH_215_US                   = 0x2U,
  MAX30102_PULSE_WITDH_411_US                   = 0x3U,
} MAX30102_LEDPulseWidth_TypeDef;

typedef struct
{
    I2C_HandleTypeDef       *hi2c;
    uint8_t                 _devAddress;
    uint8_t                 _samples;
    double                  _activeTime;
    double                  _requestedTime;
} MAX30102_HandleTypeDef;


/* Configuration Getters (Status-Return Pattern) */
HAL_StatusTypeDef MAX30102_GetConfiguration(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetTempHighLimitReg(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetTempLowLimitReg(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetDeviceId(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetEepromUnlock(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetEeprom1(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetEeprom2(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);
HAL_StatusTypeDef MAX30102_GetEeprom3(MAX30102_HandleTypeDef *MAX30102, uint16_t *value);


/* Diagnostic Readings */
_Bool MAX30102_EepromBusyFlag(MAX30102_HandleTypeDef *MAX30102);
double MAX30102_GetHighLimitTemp_C(MAX30102_HandleTypeDef *MAX30102);
double MAX30102_GetLowLimitTemp_C(MAX30102_HandleTypeDef *MAX30102);
double MAX30102_CheckTemperature(uint16_t value);

/* EEPROM Control */
HAL_StatusTypeDef MAX30102_SetEeprom1(MAX30102_HandleTypeDef *MAX30102, uint16_t data);
HAL_StatusTypeDef MAX30102_SetEeprom2(MAX30102_HandleTypeDef *MAX30102, uint16_t data);
HAL_StatusTypeDef MAX30102_SetEeprom3(MAX30102_HandleTypeDef *MAX30102, uint16_t data);

/* Initialization & Control */
HAL_StatusTypeDef MAX30102_Init(MAX30102_HandleTypeDef *MAX30102, I2C_HandleTypeDef *i2c, uint8_t devAddress);
HAL_StatusTypeDef MAX30102_ResetDevice(MAX30102_HandleTypeDef *MAX30102);
HAL_StatusTypeDef MAX30102_SetAlertPinFunction(MAX30102_HandleTypeDef *MAX30102, MAX30102_DRALERT_TypeDef pinFunction);
HAL_StatusTypeDef MAX30102_SetAlertPinPolarity(MAX30102_HandleTypeDef *MAX30102, MAX30102_AlertPinPol_TypeDef polarity);
HAL_StatusTypeDef MAX30102_SetThermAlertMode(MAX30102_HandleTypeDef *MAX30102, MAX30102_ThermAlertMode_TypeDef tnA);
HAL_StatusTypeDef MAX30102_SetAverage(MAX30102_HandleTypeDef *MAX30102, MAX30102_Avg_TypeDef avg);
HAL_StatusTypeDef MAX30102_SetMode(MAX30102_HandleTypeDef *MAX30102, MAX30102_Mode_TypeDef mode);

/* Status flags and additional configuration */
_Bool MAX30102_IsEEPROMBusy(MAX30102_HandleTypeDef *MAX30102);
_Bool MAX30102_IsDataReady(MAX30102_HandleTypeDef *MAX30102);
_Bool MAX30102_IsLowAlertSet(MAX30102_HandleTypeDef *MAX30102);
_Bool MAX30102_IsHighAlertSet(MAX30102_HandleTypeDef *MAX30102);
HAL_StatusTypeDef MAX30102_SetConvTime(MAX30102_HandleTypeDef *MAX30102, MAX30102_ConvTime_TypeDef convTime);
HAL_StatusTypeDef MAX30102_SetHighLimit_C(MAX30102_HandleTypeDef *MAX30102, double highLimit);
HAL_StatusTypeDef MAX30102_SetLowLimit_C(MAX30102_HandleTypeDef *MAX30102, double lowLimit);
double MAX30102_GetTemperature_C(MAX30102_HandleTypeDef *MAX30102);


#endif
