/**
  ******************************************************************************
  * @file    DRV8251A.h
  * @brief   Header file for the DRV8251A H-Bridge Motor Driver Library.
  *          This library supports speed control (via dual PWM), direction control,
  *          braking, low-power sleep mode, fault checking, and real-time current
  *          monitoring.
  ******************************************************************************
  */

#ifndef DRV8251A_H
#define DRV8251A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

/**
  * @brief DRV8251A drv8251aice Configuration & State Structure
  */
typedef struct {
    /* Timer & PWM Settings (Mandatory) */
    TIM_HandleTypeDef* htim;        /*  Pointer to the STM32 TIM Handle (e.g. &htim3) */
    uint32_t channel1;              /* Timer Channel for IN1 (e.g. TIM_CHANNEL_1) */
    uint32_t channel2;              /* Timer Channel for IN2 (e.g. TIM_CHANNEL_2) */
    uint32_t max_duty;              /* The timer's Auto-Reload Register (ARR) value (represents 100% duty) */

    /* nFAULT Pin Settings (Optional, set Port to NULL if not used) */
    GPIO_TypeDef* nFaultPort;       /* GPIO Port for nFAULT (e.g. GPIOC) */
    uint16_t nFaultPin;             /* GPIO Pin for nFAULT (e.g. GPIO_PIN_13) */

    /* Current Sensing Settings (Optional, set hadc to NULL if not used) */
    ADC_HandleTypeDef* hadc;        /* Pointer to the STM32 ADC Handle (e.g. &hadc1) */
    uint16_t* adc_buffer;           /* Pointer to the ADC DMA circular buffer */
    uint16_t adc_buffer_size;       /* Size of the DMA circular buffer (number of samples) */
    double r_ipropi;                /* Value of the IPROPI resistor to GND in Ohms (e.g. 1000.0) */
    double vref;                    /* Reference voltage at VREF pin in Volts (e.g. 3.3) */
} DRV8251A_HandleTypeDef;

/**
  * @brief  Initializes the DRV8251A motor driver instance.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval HAL status (HAL_OK if initialization succeeded)
  */
HAL_StatusTypeDef DRV8251A_Init(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Sets the motor speed and direction.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @param  speed: Target speed from -1000 to +1000.
  *                Positive = Forward, Negative = Reverse, 0 = Coast / Sleep.
  */
void DRV8251A_SetSpeed(DRV8251A_HandleTypeDef* drv8251a, int16_t speed);

/**
  * @brief  Actively brakes the motor (low-side slow decay).
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  */
void DRV8251A_Brake(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Puts the driver into ultra-low-power sleep mode (forces both IN1/IN2 Low).
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  */
void DRV8251A_Sleep(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Checks if the driver has reported a fault on nFAULT.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval 1 if a fault is active (nFAULT is LOW), 0 otherwise.
  */
uint8_t DRV8251A_CheckFault(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Reads the real-time motor current using the ADC and the IPROPI current mirror.
  * @param  dev: Pointer to the DRV8251A handle structure.
  * @retval Motor current in Amperes (returns 0.0 if ADC is not configured).
  */
double DRV8251A_ReadCurrent(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Reads the motor current using a DMA circular buffer (averages samples for filtering).
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval Motor current in Amperes (returns 0.0 if DMA buffer is not config.
  */
double DRV8251A_ReadCurrentDMA(DRV8251A_HandleTypeDef* drv8251a);

/**
  * @brief  Calculates the hardware current limit (ITRIP) set by the VREF and RIPROPI resistor.
  * @param  drv8251a: Pointer to the DRV8251A handle structure.
  * @retval Current limit in Amperes.
  */
double DRV8251A_GetHardwareCurrentLimit(DRV8251A_HandleTypeDef* drv8251a);

#ifdef __cplusplus
}
#endif

#endif /* DRV8251A_H */